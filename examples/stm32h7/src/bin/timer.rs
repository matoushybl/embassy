#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[path = "../example_common.rs"]
mod example_common;
use embassy::executor::Spawner;
use embassy::time::{Duration, Timer};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::time::U32Ext;
use embassy_stm32::timer::basic_timer;
use embassy_stm32::{Config, Peripherals};
use embedded_hal::digital::v2::OutputPin;
use example_common::*;

pub fn config() -> Config {
    let mut config = Config::default();
    config.rcc.sys_ck = Some(400.mhz().into());
    config.rcc.hclk = Some(400.mhz().into());
    config.rcc.pll1.q_ck = Some(100.mhz().into());
    config.rcc.enable_dma1 = true;
    // config.rcc.enable_dma2 = true;
    config.rcc.pclk1 = Some(100.mhz().into());
    config.rcc.pclk2 = Some(100.mhz().into());
    config.rcc.pclk3 = Some(100.mhz().into());
    config.rcc.pclk4 = Some(100.mhz().into());
    config
}

#[embassy::main(config = "config()")]
async fn main(_spawner: Spawner, p: Peripherals) {
    info!("Hello World!");

    let mut led = Output::new(p.PB14, Level::High, Speed::Low);

    let irq = interrupt::take!(TIM1_UP);
    let mut state = basic_timer::State::new();
    let mut timer = basic_timer::Timer::new(&mut state, p.TIM1, irq);
    timer.start(1.hz());

    timer.tick().await;
    error!("tick");
    timer.tick().await;
    error!("tick");

    loop {
        info!("high");
        unwrap!(led.set_high());
        Timer::after(Duration::from_millis(500)).await;

        info!("low");
        unwrap!(led.set_low());
        Timer::after(Duration::from_millis(500)).await;
    }
}
