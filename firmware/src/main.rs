#![allow(clippy::too_many_arguments)]
#![cfg_attr(feature = "calibrate", allow(unused))]
#![no_std]
#![no_main]
extern crate alloc;

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    Async,
    gpio::{self, interconnect::PeripheralOutput},
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    peripherals::LEDC,
    ram,
    time::Rate,
    timer::timg::TimerGroup,
};
use esp_println as _;

esp_bootloader_esp_idf::esp_app_desc!();

mod fusion;
mod sensors;
mod wifi;

#[cfg(not(feature = "dmp"))]
use crate::{
    fusion::FusionBuilder,
    sensors::{ImuRead, ImuReadMag},
};
use crate::{sensors::Sensor, wifi::AP};

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
async fn main(spawner: Spawner) {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // allocate heap for wifi
    esp_alloc::heap_allocator!(#[ram(reclaimed)] size: 64 * 1024);
    esp_alloc::heap_allocator!(size: 36 * 1024);

    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // start AP and spawn UDP listen task
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
            peripherals.GPIO1,
            peripherals.GPIO10,
            peripherals.GPIO0,
            peripherals.GPIO9,
            int_pin,
        )
        .await;
    }

    #[cfg(feature = "calibrate")]
    {
        // ICM20948
        let mut sensor = Sensor::init_icm20948(i2c, LOOP_PERIOD_MS)
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
    init_pwm_motors(
        ledc,
        rear_left_pin,
        rear_right_pin,
        front_left_pin,
        front_right_pin,
    )
    .await;
    // ICM20948
    let sensor = Sensor::init_icm20948(i2c, LOOP_PERIOD_MS)
        .await
        .expect("ICM20948 init failed");

    #[cfg(not(feature = "dmp"))]
    {
        let _ = int_pin;
        software_loop(sensor).await;
    }
    #[cfg(feature = "dmp")]
    run_dmp(sensor, int_pin).await;
}

type Sensor20948<'a> = Sensor<icm20948::Icm20948Driver<icm20948::I2cInterface<I2c<'a, Async>>>>;

#[cfg(not(feature = "dmp"))]
async fn software_loop(mut sensor: Sensor20948<'_>) {
    let mut fusion = FusionBuilder::new()
        .icm20948()
        // .vqf()
        // .mahony()
        .madgwick()
        .build();
    // let mut fusion = FusionBuilder::new().mpu6050().complementary().build();
    let mut last = embassy_time::Instant::now();
    let mut log_counter: u32 = 0;

    loop {
        let now = embassy_time::Instant::now();
        let dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;
        last = now;

        match sensor.read().await {
            Ok((a, g)) => {
                let quat = fusion.update_imu(dt, a, g);
                let (roll_rad, pitch_rad, yaw_rad) = quat.euler_angles();
                let roll_deg = roll_rad * fusion::RAD_TO_DEG;
                let pitch_deg = pitch_rad * fusion::RAD_TO_DEG;
                let yaw_deg = yaw_rad * fusion::RAD_TO_DEG;

                // set_lights(
                //     roll_deg,
                //     pitch_deg,
                //     &mut led_fwd_roll,
                //     &mut led_bwd_roll,
                //     &mut led_fwd_pitch,
                //     &mut led_bwd_pitch,
                // );

                log_counter += 1;
                if log_counter >= LOG_EVERY_N {
                    log_counter = 0;
                    defmt::debug!(
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

        Timer::after_millis(LOOP_PERIOD_MS).await;
    }
}

#[cfg(feature = "dmp")]
async fn run_dmp(mut sensor: Sensor20948<'_>, mut int_pin: gpio::Input<'static>) {
    let mut log_counter: u32 = 0;

    loop {
        use icm20948::dmp::DmpData;

        int_pin.wait_for_high().await;
        match sensor.read_dmp().await {
            Ok(Some(DmpData {
                quaternion_9axis: Some(quat),
                ..
            })) => {
                let euler = quat.to_euler_angles();
                log_counter += 1;
                if log_counter >= LOG_EVERY_N {
                    log_counter = 0;
                    defmt::debug!(
                        "DMP 9axis - w: {} x: {} y: {} z: {} | roll: {}° pitch: {}° yaw: {}°",
                        quat.w,
                        quat.x,
                        quat.y,
                        quat.z,
                        euler.roll * fusion::RAD_TO_DEG,
                        euler.pitch * fusion::RAD_TO_DEG,
                        euler.yaw * fusion::RAD_TO_DEG,
                    );
                }
            }
            Ok(Some(DmpData {
                quaternion_6axis: Some(quat),
                ..
            })) => {
                let euler = quat.to_euler_angles();
                log_counter += 1;
                if log_counter >= LOG_EVERY_N {
                    log_counter = 0;
                    defmt::debug!(
                        "DMP 6axis - w: {} x: {} y: {} z: {} | roll: {}° pitch: {}° yaw: {}°",
                        quat.w,
                        quat.x,
                        quat.y,
                        quat.z,
                        euler.roll * fusion::RAD_TO_DEG,
                        euler.pitch * fusion::RAD_TO_DEG,
                        euler.yaw * fusion::RAD_TO_DEG,
                    );
                }
            }
            Ok(_) => {}
            Err(e) => defmt::error!("DMP read error: {}", defmt::Debug2Format(&e)),
        }
    }
}

#[cfg(feature = "visualize")]
async fn run_visualizer(i2c: I2c<'_, esp_hal::Async>, int_pin: gpio::Input<'static>) {
    let sensor = Sensor::init_icm20948(i2c, LOOP_PERIOD_MS)
        .await
        .expect("ICM20948 init failed");

    #[cfg(not(feature = "dmp"))]
    {
        let _ = int_pin;
        software_loop(sensor).await;
    }
    #[cfg(feature = "dmp")]
    run_dmp(sensor, int_pin).await;
}

async fn init_pwm_motors<
    RL: PeripheralOutput<'static>,
    RR: PeripheralOutput<'static>,
    FL: PeripheralOutput<'static>,
    FR: PeripheralOutput<'static>,
>(
    ledc: LEDC<'static>,
    rear_left_pin: RL,
    rear_right_pin: RR,
    front_left_pin: FL,
    front_right_pin: FR,
) {
    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let duty = 0;
    let mut lstimer0 = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    lstimer0
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty10Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_khz(78),
        })
        .expect("timer init failed");

    let mut chan_rl = ledc.channel(channel::Number::Channel0, rear_left_pin);
    chan_rl
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("rear left motor pwm init failed");

    let mut chan_rr = ledc.channel(channel::Number::Channel1, rear_right_pin);
    chan_rr
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("rear right motor pwm init failed");

    let mut chan_fl = ledc.channel(channel::Number::Channel2, front_left_pin);
    chan_fl
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("front left motor pwm init failed");

    let mut chan_fr = ledc.channel(channel::Number::Channel3, front_right_pin);
    chan_fr
        .configure(channel::config::Config {
            timer: &lstimer0,
            duty_pct: 0,
            drive_mode: esp_hal::gpio::DriveMode::PushPull,
        })
        .expect("front right motor pwm init failed");

    #[cfg(feature = "test_motors")]
    {
        let test_spin = 20;
        defmt::info!("testing front left");
        chan_fl
            .start_duty_fade(0, test_spin, 2_000)
            .expect("failed to set duty");
        Timer::after_secs(3).await;
        defmt::info!("testing front right");
        chan_fr
            .start_duty_fade(0, test_spin, 2_000)
            .expect("failed to set duty");
        Timer::after_secs(3).await;
        defmt::info!("testing rear left");
        chan_rl
            .start_duty_fade(0, test_spin, 2_000)
            .expect("failed to set duty");
        Timer::after_secs(3).await;
        defmt::info!("testing rear right");
        chan_rr
            .start_duty_fade(0, test_spin, 2_000)
            .expect("failed to set duty");
        defmt::info!("motors initialized: all channels {}%", test_spin);
        Timer::after_secs(3).await;

        chan_fl.set_duty(0).expect("failed to set duty");
        chan_fr.set_duty(0).expect("failed to set duty");
        chan_rl.set_duty(0).expect("failed to set duty");
        chan_rr.set_duty(0).expect("failed to set duty");
    }
    defmt::info!("motors initialized: all channels {}%", duty);
}
