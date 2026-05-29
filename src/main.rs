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
use mpu9250_async::Mpu9250;

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

    let mut imu = match Mpu9250::new(i2c, Delay).await {
        Ok(imu) => imu,
        Err(e) => {
            esp_println::println!("MPU9250 init failed: {:?}", e);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    esp_println::println!("MPU9250 initialized");
    loop {
        led.toggle();
        if led.is_set_high() {
            esp_println::println!("LED toggled on");
        } else {
            esp_println::println!("LED toggled off");
        }
        // GY-9250 wiring: SDA -> GPIO4, SCL -> GPIO5, VCC -> 3.3V, GND -> GND, ADO -> GND (addr 0x68)
        match imu.get_all().await {
            Ok(data) => {
                esp_println::println!(
                    "accel x={:.3} y={:.3} z={:.3} g | gyro x={:.3} y={:.3} z={:.3} rad/s | mag x={:.1} y={:.1} z={:.1} uT",
                    data.accel.x,
                    data.accel.y,
                    data.accel.z,
                    data.gyro.x,
                    data.gyro.y,
                    data.gyro.z,
                    data.mag.x,
                    data.mag.y,
                    data.mag.z,
                );
            }
            Err(e) => {
                esp_println::println!("sensor read error: {:?}", e);
            }
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}
