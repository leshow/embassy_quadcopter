#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use esp_hal::{
    gpio,
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
};
use mpu9250_async::Mpu6050;

use esp_backtrace as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let mut led = gpio::Output::new(
        peripherals.GPIO8,
        gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );

    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO20)
        .with_scl(peripherals.GPIO21)
        .into_async();

    let mut mpu = Mpu6050::new(i2c);
    let mut delay = Delay;

    mpu.init(&mut delay).await.expect("MPU6050 init failed");
    esp_println::println!("MPU6050 init OK");

    loop {
        match (mpu.get_acc().await, mpu.get_gyro().await) {
            (Ok(acc), Ok(gyro)) => {
                esp_println::println!(
                    "acc: [{:.3}, {:.3}, {:.3}]  gyro: [{:.3}, {:.3}, {:.3}]",
                    acc.x, acc.y, acc.z,
                    gyro.x, gyro.y, gyro.z,
                );
            }
            (Err(e), _) => esp_println::println!("acc error: {:?}", e),
            (_, Err(e)) => esp_println::println!("gyro error: {:?}", e),
        }

        led.toggle();
        Timer::after(Duration::from_millis(500)).await;
    }
}

