#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use defmt_rtt as _; // global logger
use embassy::executor::Executor;
use embassy::util::Forever;
use embassy_stm32::time::U32Ext;
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::Config;
use futures::future::join;
use panic_probe as _;

use cortex_m_rt::entry;

pub fn config() -> Config {
    let mut config = Config::default();
    config.rcc.sys_ck = Some(400.mhz().into());
    config.rcc.hclk = Some(200.mhz().into());
    config.rcc.per_ck = Some(64.mhz().into());
    config
}

#[embassy::task]
async fn main_task() {
    let p = embassy_stm32::init(config());

    defmt::warn!("Connect pins PA0 and PA1 to create serial loopback!");

    let config = usart::Config::default();
    let usart = Uart::new(p.UART4, p.PA1, p.PA0, p.DMA1_CH0, p.DMA1_CH1, config);
    let (mut tx, mut rx) = usart.split();

    const LEN: usize = 136_000;
    let mut output_buffer = [0u8; LEN];
    for index in 0..LEN {
        output_buffer[index] = (index % 255) as u8;
    }

    let mut input_buffer = [0u8; LEN];

    loop {
        info!("Looping back data - transfers should take around 12 s to finish.");
        let (r1, r2) = join(tx.write(&output_buffer), rx.read(&mut input_buffer)).await;
        if r1.is_err() {
            error!("Error while writing.")
        }
        if r2.is_err() {
            error!("Error while reading.")
        }
        defmt::info!("Read bytes {}.", &input_buffer[..128]);
    }
}

static EXECUTOR: Forever<Executor> = Forever::new();

#[entry]
fn main() -> ! {
    info!("Hello World!");

    let executor = EXECUTOR.put(Executor::new());

    executor.run(|spawner| {
        unwrap!(spawner.spawn(main_task()));
    })
}
