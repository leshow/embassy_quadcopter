#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{
    gpio::{Level, Output, OutputConfig},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
};

use esp_backtrace as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Change GPIO8 to whichever pin your LED is wired to
    let mut led = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());

    loop {
        led.toggle();
        if led.is_set_high() {
            esp_println::println!("LED toggled on");
        } else {
            esp_println::println!("LED toggled off");
        }
        Timer::after(Duration::from_millis(5000)).await;
    }
}
