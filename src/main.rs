#![allow(clippy::too_many_arguments)]
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Timer};
use esp_hal::{
    Async, gpio,
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    timer::timg::TimerGroup,
};
#[cfg(feature = "mpu6050")]
use mpu9250_async::Mpu6050;

use esp_backtrace as _;

esp_bootloader_esp_idf::esp_app_desc!();

const ALPHA: f32 = 0.98; // complementary filter: trust gyro 98%, accel 2%
const FLAT_DEG: f32 = 10.0; // dead-zone around flat
const STEEP_DEG: f32 = 50.0; // "both LEDs on" threshold

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

    #[cfg(feature = "mpu6050")]
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(peripherals.GPIO20)
        .with_scl(peripherals.GPIO21)
        .into_async();

    #[cfg(feature = "mpu6050")]
    let mut mpu = {
        let mut m = Mpu6050::new(i2c);
        let mut delay = Delay;
        m.init(&mut delay).await.expect("MPU6050 init failed");
        esp_println::println!("MPU6050 init OK");
        m
    };

    let mut angle_pitch: f32 = 0.0; // rotation around Y axis (radians)
    let mut angle_roll: f32 = 0.0; // rotation around X axis (radians)
    let mut last = Instant::now();
    let mut log_counter: u32 = 0;

    loop {
        let now = Instant::now();
        let dt = now.duration_since(last).as_micros() as f32 / 1_000_000.0;
        last = now;

        #[cfg(feature = "mpu6050")]
        if let Ok((roll_deg, pitch_deg)) = complementary_filter(
            &mut mpu,
            &mut angle_roll,
            &mut angle_pitch,
            dt,
            &mut led_fwd_roll,
            &mut led_bwd_roll,
            &mut led_fwd_pitch,
            &mut led_bwd_pitch,
        )
        .await
        {
            log_counter += 1;
            if log_counter >= 100 {
                log_counter = 0;
                esp_println::println!("roll: {:.1}°  pitch: {:.1}°", roll_deg, pitch_deg);
            }
        }

        Timer::after(Duration::from_millis(5)).await; // ~200 Hz
    }
}

#[cfg(feature = "mpu6050")]
async fn complementary_filter(
    mpu: &mut Mpu6050<I2c<'_, Async>>,
    angle_roll: &mut f32,
    angle_pitch: &mut f32,
    dt: f32,
    led_fwd_roll: &mut gpio::Output<'_>,
    led_bwd_roll: &mut gpio::Output<'_>,
    led_fwd_pitch: &mut gpio::Output<'_>,
    led_bwd_pitch: &mut gpio::Output<'_>,
) -> Result<(f32, f32), ()> {
    match (mpu.get_acc_angles().await, mpu.get_gyro().await) {
        (Ok(angles), Ok(gyro)) => {
            // angles[0] = roll  (rotation around X), angles[1] = pitch (rotation around Y)
            // gyro.x = roll rate, gyro.y = pitch rate
            *angle_roll = ALPHA * (*angle_roll + gyro.x * dt) + (1.0 - ALPHA) * angles[0];
            *angle_pitch = ALPHA * (*angle_pitch + gyro.y * dt) + (1.0 - ALPHA) * angles[1];

            let rad_to_deg = 180.0 / core::f32::consts::PI;
            let roll_deg = *angle_roll * rad_to_deg;
            let pitch_deg = *angle_pitch * rad_to_deg;

            // set LEDS
            {
                set_lights(
                    roll_deg,
                    pitch_deg,
                    led_fwd_roll,
                    led_bwd_roll,
                    led_fwd_pitch,
                    led_bwd_pitch,
                );
            }
            Ok((roll_deg, pitch_deg))
        }
        (Err(e), _) => {
            esp_println::println!("acc error: {:?}", e);
            Err(())
        }
        (_, Err(e)) => {
            esp_println::println!("gyro error: {:?}", e);
            Err(())
        }
    }
}

#[cfg(feature = "mpu6050")]
fn set_lights(
    roll_deg: f32,
    pitch_deg: f32,
    led_fwd_roll: &mut gpio::Output<'_>,
    led_bwd_roll: &mut gpio::Output<'_>,
    led_fwd_pitch: &mut gpio::Output<'_>,
    led_bwd_pitch: &mut gpio::Output<'_>,
) {
    // LEDs show pitch (forward/backward tilt)
    let (fwd, bwd) = if pitch_deg.abs() > STEEP_DEG {
        (true, true)
    } else if pitch_deg > FLAT_DEG {
        (true, false)
    } else if pitch_deg < -FLAT_DEG {
        (false, true)
    } else {
        (false, false)
    };

    led_fwd_pitch.set_level(fwd.into());
    led_bwd_pitch.set_level(bwd.into());

    // LEDs show roll
    let (fwd, bwd) = if roll_deg.abs() > STEEP_DEG {
        (true, true)
    } else if roll_deg > FLAT_DEG {
        (true, false)
    } else if roll_deg < -FLAT_DEG {
        (false, true)
    } else {
        (false, false)
    };
    led_fwd_roll.set_level(fwd.into());
    led_bwd_roll.set_level(bwd.into());
}
