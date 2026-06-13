#![allow(clippy::too_many_arguments)]
#![cfg_attr(feature = "calibrate", allow(unused))]
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    gpio,
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
};
use esp_println as _;

esp_bootloader_esp_idf::esp_app_desc!();

mod fusion;
mod sensors;

use crate::fusion::FusionBuilder;
use crate::sensors::{ImuReadMag, Sensor};

const LOOP_PERIOD_MS: u64 = 1; // 1000Hz target loop rate; shared by timer and Madgwick sample_period

/// How many loop iterations to skip between log lines.
/// Override at build time: `LOG_RATE_MS=200 cargo flash-c3` (default: 500 ms).
const LOG_EVERY_N: u32 = {
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
    let ms = match option_env!("LOG_RATE_MS") {
        Some(s) => parse_u64(s),
        None => 500,
    };
    (ms / LOOP_PERIOD_MS) as u32
};

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    #[cfg(not(feature = "calibrate"))]
    let mut led_fwd_pitch = gpio::Output::new(
        peripherals.GPIO10,
        gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    #[cfg(not(feature = "calibrate"))]
    let mut led_bwd_pitch = gpio::Output::new(
        peripherals.GPIO9,
        gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    #[cfg(not(feature = "calibrate"))]
    let mut led_fwd_roll = gpio::Output::new(
        peripherals.GPIO0,
        gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    #[cfg(not(feature = "calibrate"))]
    let mut led_bwd_roll = gpio::Output::new(
        peripherals.GPIO1,
        gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );

    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO20)
        .with_scl(peripherals.GPIO21)
        .into_async();

    // Wait for ICM20948 to power up before init
    Timer::after(Duration::from_millis(100)).await;

    // ICM20948
    let mut sensor = Sensor::init_icm20948(i2c)
        .await
        .expect("ICM20948 init failed");

    #[cfg(feature = "calibrate")]
    sensor.run_calibration().await;

    #[cfg(not(feature = "calibrate"))]
    {
        let mut fusion = FusionBuilder::new()
            .icm20948()
            // .vqf()
            // .mahony()
            .madgwick()
            .build();
        // let mut fusion = FusionBuilder::new().mpu6050().complementary().build();
        let mut last = Instant::now();
        let mut log_counter: u32 = 0;

        loop {
            let now = Instant::now();
            let dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;
            last = now;

            match sensor.read_mag().await {
                Ok((a, g, _m)) => {
                    let quat = fusion.update_imu(dt, a, g);
                    let (roll_rad, pitch_rad, yaw_rad) = quat.euler_angles();
                    let roll_deg = roll_rad * fusion::RAD_TO_DEG;
                    let pitch_deg = pitch_rad * fusion::RAD_TO_DEG;
                    let yaw_deg = yaw_rad * fusion::RAD_TO_DEG;

                    set_lights(
                        roll_deg,
                        pitch_deg,
                        &mut led_fwd_roll,
                        &mut led_bwd_roll,
                        &mut led_fwd_pitch,
                        &mut led_bwd_pitch,
                    );

                    log_counter += 1;
                    if log_counter >= LOG_EVERY_N {
                        log_counter = 0;
                        defmt::info!(
                            "qx: {} qy: {} qz: {} qw: {} \n roll: {}°  pitch: {}°  yaw: {}°",
                            roll_rad,
                            pitch_rad,
                            yaw_rad,
                            quat.w,
                            roll_deg,
                            pitch_deg,
                            yaw_deg
                        );
                    }
                }
                Err(e) => defmt::error!("imu error: {}", defmt::Debug2Format(&e)),
            }

            Timer::after(Duration::from_millis(LOOP_PERIOD_MS)).await;
        }
    }
}

fn set_lights(
    roll_deg: f32,
    pitch_deg: f32,
    led_fwd_roll: &mut gpio::Output<'_>,
    led_bwd_roll: &mut gpio::Output<'_>,
    led_fwd_pitch: &mut gpio::Output<'_>,
    led_bwd_pitch: &mut gpio::Output<'_>,
) {
    // LEDs show pitch (forward/backward tilt)
    let (fwd, bwd) = if pitch_deg.abs() > fusion::STEEP_DEG {
        (true, true)
    } else if pitch_deg > fusion::FLAT_DEG {
        (true, false)
    } else if pitch_deg < -fusion::FLAT_DEG {
        (false, true)
    } else {
        (false, false)
    };

    led_fwd_pitch.set_level(fwd.into());
    led_bwd_pitch.set_level(bwd.into());

    // LEDs show roll
    let (fwd, bwd) = if roll_deg.abs() > fusion::STEEP_DEG {
        (true, true)
    } else if roll_deg > fusion::FLAT_DEG {
        (true, false)
    } else if roll_deg < -fusion::FLAT_DEG {
        (false, true)
    } else {
        (false, false)
    };
    led_fwd_roll.set_level(fwd.into());
    led_bwd_roll.set_level(bwd.into());
}
