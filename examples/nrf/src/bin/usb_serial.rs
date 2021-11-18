#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::unwrap;
use defmt_rtt as _; // global logger
use embassy::executor::Spawner;
use embassy::io::AsyncBufReadExt;
use embassy::io::AsyncWriteExt;
use embassy::time::{Duration, Timer};
use embassy::util::Forever;
use embassy_hal_common::usb::usb_serial::{UsbSerial, USB_CLASS_CDC};
use embassy_hal_common::usb::{State, Usb};
use embassy_nrf::config::*;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::interrupt;
use embassy_nrf::usbd::UsbPeripheral;
use embassy_nrf::Peripherals;
use embedded_hal::digital::v2::OutputPin;
use futures::future::{select, Either};
use futures::pin_mut;
use nrf_usbd::Usbd;
use panic_probe as _;
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};

use core::sync::atomic::{AtomicUsize, Ordering};

defmt::timestamp! {"{=u64}", {
        static COUNT: AtomicUsize = AtomicUsize::new(0);
        // NOTE(no-CAS) `timestamps` runs with interrupts disabled
        let n = COUNT.load(Ordering::Relaxed);
        COUNT.store(n + 1, Ordering::Relaxed);
        n as u64
    }
}

#[allow(unused)]
pub fn config() -> Config {
    defmt::error!("using config.");
    let mut config = Config::default();
    config.hfclk_source = HfclkSource::ExternalXtal;
    config
}

static USB_BUS: Forever<UsbBusAllocator<Usbd<UsbPeripheral>>> = Forever::new();

#[embassy::main(config = "config()")]
async fn main(_spawner: Spawner, p: Peripherals) {
    Timer::after(Duration::from_millis(300)).await;
    let mut led = Output::new(p.P1_15, Level::Low, OutputDrive::Standard);

    unwrap!(led.set_high());
    Timer::after(Duration::from_millis(300)).await;
    unwrap!(led.set_low());
    Timer::after(Duration::from_millis(300)).await;

    let usb = UsbPeripheral::new(p.USBD);
    let usb = Usbd::new(usb);

    let bus = USB_BUS.put(usb);
    defmt::error!("Hello world");

    let mut read_buf = [0u8; 128];
    let mut write_buf = [0u8; 128];
    let serial = UsbSerial::new(bus, &mut read_buf, &mut write_buf);

    let device = UsbDeviceBuilder::new(bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(USB_CLASS_CDC)
        .build();

    let irq = interrupt::take!(USBD);

    let mut state = State::new();

    // sprinkle some unsafe
    let usb = unsafe { Usb::new(&mut state, device, serial, irq) };
    pin_mut!(usb);

    defmt::error!("pinned");
    // usb.as_mut().start();

    let (mut read_interface, mut write_interface) = usb.as_ref().take_serial_0();

    let mut buf = [0u8; 64];
    loop {
        defmt::error!("looping");
        let mut n = 0;
        let left = {
            let recv_fut = async {
                loop {
                    let byte = unwrap!(read_interface.read_byte().await);
                    unwrap!(write_interface.write_byte(byte).await);
                    buf[n] = byte;

                    n += 1;
                    if byte == b'\n' || byte == b'\r' || n == buf.len() {
                        break;
                    }
                }
            };
            pin_mut!(recv_fut);

            let timeout = Timer::after(Duration::from_ticks(32768 * 10));

            match select(recv_fut, timeout).await {
                Either::Left(_) => true,
                Either::Right(_) => false,
            }
        };

        if left {
            for c in buf[..n].iter_mut() {
                if 0x61 <= *c && *c <= 0x7a {
                    *c &= !0x20;
                }
            }
            unwrap!(write_interface.write_byte(b'\n').await);
            unwrap!(write_interface.write_all(&buf[..n]).await);
            unwrap!(write_interface.write_byte(b'\n').await);
        } else {
            unwrap!(write_interface.write_all(b"\r\nSend something\r\n").await);
        }
    }
}
