#![allow(clippy::too_many_arguments)]
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer};
use esp_hal::{
    gpio,
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
};

// ICM20948 imports (default path)
use icm20948::{I2cInterface, Icm20948Driver, MagConfig};

// MPU6050
// use mpu9250_async::Mpu6050;

use esp_backtrace as _;
use mpu9250_async::Mpu6050;
use nalgebra::Vector3;

esp_bootloader_esp_idf::esp_app_desc!();

mod fusion;
use fusion::FusionBuilder;

const LOOP_PERIOD_MS: u64 = 5; // target loop rate; shared by timer and Madgwick sample_period

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let mut led_fwd_pitch = gpio::Output::new(
        peripherals.GPIO10,
        gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let mut led_bwd_pitch = gpio::Output::new(
        peripherals.GPIO9,
        gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let mut led_fwd_roll = gpio::Output::new(
        peripherals.GPIO0,
        gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );
    let mut led_bwd_roll = gpio::Output::new(
        peripherals.GPIO1,
        gpio::Level::Low,
        esp_hal::gpio::OutputConfig::default(),
    );

    let mut delay = Delay;
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO20)
        .with_scl(peripherals.GPIO21)
        .into_async();

    // ICM20948
    let mut imu = {
        // icm20948 requires CS to VIN to activate i2c
        // CS to GND for SPI
        let interface = I2cInterface::alternative(i2c);
        let mut driver = Icm20948Driver::new(interface);
        driver
            .verify_who_am_i()
            .await
            .expect("ICM20948 WHO_AM_I failed");
        driver.init(&mut delay).await.expect("ICM20948 init failed");
        driver
            .init_magnetometer(MagConfig::default(), &mut delay)
            .await
            .expect("ICM20948 mag init failed");
        esp_println::println!("ICM20948 init OK");
        driver
    };

    // // mpu6050 (no magnetometer)
    // let mut mpu = {
    //     let mut m = Mpu6050::new(i2c);
    //     m.init(&mut delay).await.expect("MPU6050 init failed");
    //     esp_println::println!("MPU6050 init OK");
    //     m
    // };

    let mut fusion = FusionBuilder::new()
        .icm20948()
        .madgwick()
        .sample_period(LOOP_PERIOD_MS as f32 / 1000.0)
        .build();
    // let mut fusion = FusionBuilder::new().mpu6050().complementary().build();
    let mut last = Instant::now();
    let mut log_counter: u32 = 0;

    loop {
        let now = Instant::now();
        let dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;
        last = now;

        // --- ICM20948 loop body ---
        match icm20948_read(&mut imu).await {
            Ok((a, g, m)) => {
                let (roll_deg, pitch_deg, yaw_deg) = fusion.update(dt, a, g, m).unwrap();

                set_lights(
                    roll_deg,
                    pitch_deg,
                    &mut led_fwd_roll,
                    &mut led_bwd_roll,
                    &mut led_fwd_pitch,
                    &mut led_bwd_pitch,
                );

                log_counter += 1;
                if log_counter >= 100 {
                    log_counter = 0;
                    esp_println::println!(
                        "roll: {:.1}\u{b0}  pitch: {:.1}\u{b0}  yaw: {:.1}\u{b0}",
                        roll_deg,
                        pitch_deg,
                        yaw_deg
                    );
                }
            }
            Err(e) => esp_println::println!("imu error: {:?}", e),
        }

        // --- MPU6050 loop body ---
        // match mpu6050_read(&mut mpu).await {
        //     Ok((a_angles, g)) => {
        //         let (roll_deg, pitch_deg) = fusion.update(dt, a_angles, g);
        //         set_lights(
        //             roll_deg,
        //             pitch_deg,
        //             &mut led_fwd_roll,
        //             &mut led_bwd_roll,
        //             &mut led_fwd_pitch,
        //             &mut led_bwd_pitch,
        //         );

        //         log_counter += 1;
        //         if log_counter >= 100 {
        //             log_counter = 0;
        //             esp_println::println!(
        //                 "roll: {:.1}\u{b0}  pitch: {:.1}\u{b0}",
        //                 roll_deg,
        //                 pitch_deg,
        //             );
        //         }
        //     }
        // }

        Timer::after(Duration::from_millis(LOOP_PERIOD_MS)).await;
    }
}

// Reads raw sensor values from the ICM20948.
// Returns (ax, ay, az, gx, gy, mx, my, mz) as plain f32 so the caller
// owns all filter state and LED logic — no generic trait bounds needed.
async fn icm20948_read<I>(
    imu: &mut Icm20948Driver<I2cInterface<I>>,
) -> Result<(Vector3<f32>, Vector3<f32>, Vector3<f32>), icm20948::Error<I::Error>>
where
    I: embedded_hal_async::i2c::I2c,
{
    let acc = imu.read_accelerometer().await?;
    let gyro = imu.read_gyroscope_radians().await?;
    let mag = imu.read_magnetometer().await?;
    Ok((
        Vector3::new(acc.x, acc.y, acc.z),
        Vector3::new(gyro.x, gyro.y, gyro.z),
        Vector3::new(mag.x, mag.y, mag.z),
    ))
}

async fn mpu6050_read<I>(
    mpu: &mut Mpu6050<I>,
) -> Result<(nalgebra::Vector2<f32>, Vector3<f32>), mpu9250_async::Mpu6050Error<I::Error>>
where
    I: embedded_hal_async::i2c::I2c,
{
    let angles = mpu.get_acc_angles().await?;
    let gyro = mpu.get_gyro().await?;
    Ok((angles, gyro))
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
