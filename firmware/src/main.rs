#![allow(clippy::too_many_arguments)]
#![cfg_attr(feature = "calibrate", allow(unused))]
#![no_std]
#![no_main]
extern crate alloc;

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    Async, gpio,
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        timer::{self, TimerIFace, config::Duty},
    },
    peripherals::LEDC,
    ram,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println as _;
use static_cell::StaticCell;

esp_bootloader_esp_idf::esp_app_desc!();

mod flight;
mod fusion;
mod motors;
mod sensors;
mod wifi;

use crate::{motors::Motors, sensors::Sensor, wifi::AP};

const LOOP_PERIOD_MS: u64 = 1; // 1000Hz target loop rate; shared by timer and Madgwick sample_period
// if changing duty cycle, change this value. currently 10 bit resolution
const PWM_BITS: u32 = 10;
const PWM_MAX_DUTY: u32 = (1 << PWM_BITS) - 1;

static TIMER: StaticCell<timer::Timer<'static, LowSpeed>> = StaticCell::new();

const fn parse_u64(s: &str) -> u64 {
    // unfortunately parse is not a const fn
    let b = s.as_bytes();
    let mut n = 0u64;
    let mut i = 0;
    // no for loops in const either? damn.
    while i < b.len() {
        n = n * 10 + (b[i] - b'0') as u64;
        i += 1;
    }
    n
}

/// How many loop iterations to skip between log lines.
/// Override at build time: `LOG_RATE_MS=200 cargo flash-c3` (default: 500 ms).
const LOG_EVERY_N: u32 = {
    let ms = match option_env!("LOG_RATE_MS") {
        Some(s) => parse_u64(s),
        None => 500,
    };
    (ms / LOOP_PERIOD_MS) as u32
};

/// cap on throttle for testing
const THROTTLE_CAP: u8 = {
    match option_env!("THROTTLE_CAP") {
        Some(s) => {
            let v = parse_u64(s);
            assert!(v <= 100, "THROTTLE_CAP must be 0..=100");
            v as u8
        }
        None => 100, // no cap default
    }
};

const fn pwm_duty_config(bits: u32) -> Duty {
    match bits {
        1 => Duty::Duty1Bit,
        2 => Duty::Duty2Bit,
        3 => Duty::Duty3Bit,
        4 => Duty::Duty4Bit,
        5 => Duty::Duty5Bit,
        6 => Duty::Duty6Bit,
        7 => Duty::Duty7Bit,
        8 => Duty::Duty8Bit,
        9 => Duty::Duty9Bit,
        10 => Duty::Duty10Bit,
        11 => Duty::Duty11Bit,
        12 => Duty::Duty12Bit,
        13 => Duty::Duty13Bit,
        14 => Duty::Duty14Bit,
        _ => panic!("failed to pick PWM duty"),
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // wifi heap only needed when running the AP, visualize mode just logs over USB
    #[cfg(not(feature = "visualize"))]
    {
        esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
        esp_alloc::heap_allocator!(size: 36 * 1024);
    }

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    #[cfg(not(feature = "visualize"))]
    AP::init(peripherals.WIFI, spawner).await.listen(spawner);

    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO20)
    .with_scl(peripherals.GPIO21)
    .into_async();

    // Wait for ICM20948 to power up before init
    Timer::after_millis(100).await;

    #[cfg(not(any(feature = "calibrate", feature = "visualize")))]
    {
        let int_pin = gpio::Input::new(peripherals.GPIO6, gpio::InputConfig::default());
        run(
            i2c,
            peripherals.LEDC,
            peripherals.GPIO1,  // rear left
            peripherals.GPIO3,  // rear right
            peripherals.GPIO10, // front left
            peripherals.GPIO9,  // front right
            int_pin,
        )
        .await;
    }

    #[cfg(feature = "calibrate")]
    {
        // ICM20948
        let mut sensor = Sensor::init_icm20948(i2c)
            .await
            .expect("ICM20948 init failed");
        sensor.run_calibration().await;
    }

    #[cfg(feature = "visualize")]
    {
        let int_pin = gpio::Input::new(peripherals.GPIO6, gpio::InputConfig::default());
        run_visualizer(i2c, int_pin).await;
    }
}

async fn run(
    i2c: I2c<'_, esp_hal::Async>,
    ledc: LEDC<'static>,
    rear_left_pin: impl gpio::interconnect::PeripheralOutput<'static>,
    rear_right_pin: impl gpio::interconnect::PeripheralOutput<'static>,
    front_left_pin: impl gpio::interconnect::PeripheralOutput<'static>,
    front_right_pin: impl gpio::interconnect::PeripheralOutput<'static>,
    int_pin: gpio::Input<'static>,
) {
    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    // Promote the configured timer to static
    let mut timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    timer
        .configure(timer::config::Config {
            duty: pwm_duty_config(PWM_BITS),
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(78),
        })
        .expect("timer init failed");

    let timer_static = TIMER.init(timer);
    let motors = Motors::init_pwm(
        &ledc,
        timer_static,
        0,
        rear_left_pin,
        rear_right_pin,
        front_left_pin,
        front_right_pin,
    )
    .await;
    // ICM20948
    #[cfg_attr(feature = "dmp", allow(unused_mut))] // only calibrate_* (non-dmp) needs &mut
    let mut sensor = match Sensor::init_icm20948(i2c).await {
        Ok(s) => s,
        Err(e) => {
            defmt::error!("ICM20948 init failed: {}", defmt::Debug2Format(&e));
            panic!("ICM20948 init failed");
        }
    };

    // boot-time calibration - give time to place it flat, level, and step back before
    // sampling starts (gyro needs stillness, accel needs level+still)
    #[cfg(not(feature = "dmp"))]
    {
        defmt::info!("Place the drone level and step back - calibrating in 3s...");
        embassy_time::Timer::after_millis(3000).await;
    }

    #[cfg(all(feature = "telemetry", not(feature = "dmp")))]
    flight::publish_calibrating(false).await;
    #[cfg(not(feature = "dmp"))]
    {
        defmt::info!("Calibrating gyroscope...");
        if let Err(e) = sensor.calibrate_gyroscope(200).await {
            defmt::warn!("gyro calibration failed: {}", defmt::Debug2Format(&e));
        }
        defmt::info!("Calibrating accelerometer - keep level and still...");
        if let Err(e) = sensor.calibrate_accelerometer(200).await {
            defmt::warn!("accel calibration failed: {}", defmt::Debug2Format(&e));
        }
    }

    flight::run_control(sensor, int_pin, motors).await;
}

pub(crate) type Sensor20948<'a> =
    Sensor<icm20948::Icm20948Driver<icm20948::I2cInterface<I2c<'a, Async>>>>;

#[cfg(feature = "visualize")]
async fn run_visualizer(i2c: I2c<'_, esp_hal::Async>, int_pin: gpio::Input<'static>) {
    let sensor = Sensor::init_icm20948(i2c)
        .await
        .expect("ICM20948 init failed");

    #[cfg(feature = "dmp")]
    flight::run_dmp_visualizer(sensor, int_pin).await;
    #[cfg(not(feature = "dmp"))]
    flight::run_fusion_visualizer(sensor, int_pin).await;
}
