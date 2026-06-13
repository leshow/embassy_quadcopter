#![allow(unused)]

use embedded_hal_async::i2c::I2c;
use icm20948::{
    AccelConfig, AccelDlpf, AccelFullScale, GyroConfig, GyroDlpf, GyroFullScale, I2cInterface,
    Icm20948Driver, MagConfig,
};
use mpu9250_async::{Mpu6050, Mpu6050Error};
use nalgebra::{Vector2, Vector3};

pub trait ImuRead {
    type Error;
    /// Returns (accel, gyro). For 6DOF sensors; accel is (roll_rad, pitch_rad, 0.0).
    async fn read(&mut self) -> Result<(Vector3<f32>, Vector3<f32>), Self::Error>;
}

pub trait ImuReadMag: ImuRead {
    /// Returns (accel, gyro, mag). For 9DOF sensors.
    async fn read_mag(&mut self)
    -> Result<(Vector3<f32>, Vector3<f32>, Vector3<f32>), Self::Error>;
}

pub struct Sensor<D> {
    driver: D,
}

// MPU6050 (6DOF, no mag) ---

impl<I: I2c> Sensor<Mpu6050<I>> {
    pub async fn init_mpu6050(i2c: I) -> Result<Self, Mpu6050Error<I::Error>> {
        let mut driver = Mpu6050::new(i2c);
        driver.init(&mut embassy_time::Delay).await?;
        defmt::info!("MPU6050 init OK");
        Ok(Self { driver })
    }
}

impl<I: I2c> ImuRead for Sensor<Mpu6050<I>> {
    type Error = Mpu6050Error<I::Error>;

    async fn read(&mut self) -> Result<(Vector3<f32>, Vector3<f32>), Self::Error> {
        let angles: Vector2<f32> = self.driver.get_acc_angles().await?;
        let gyro: Vector3<f32> = self.driver.get_gyro().await?;
        Ok((Vector3::new(angles.x, angles.y, 0.0), gyro))
    }
}

// ICM20948 (9DOF, with mag) ---

impl<I: I2c> Sensor<Icm20948Driver<I2cInterface<I>>> {
    pub async fn init_icm20948(
        i2c: I,
        loop_period_ms: u64,
    ) -> Result<Self, icm20948::Error<I::Error>> {
        let interface = I2cInterface::alternative(i2c);
        let mut driver = Icm20948Driver::new(interface);
        driver.verify_who_am_i().await?;
        driver.init(&mut embassy_time::Delay).await?;

        // Dividers keep sensor ODR just above the loop rate so there is always
        // a fresh sample ready. Formula: ODR = base / (1 + div) >= 1000 / loop_period_ms
        // => div = base * loop_period_ms / 1000 - 1
        // saturating sub so min is 0
        let accel_div = (1125 * loop_period_ms / 1000).saturating_sub(1) as u16;
        let gyro_div = (1100 * loop_period_ms / 1000).saturating_sub(1) as u8;

        // ±4g range (headroom over default ±2g for vibration/light maneuvers);
        // 111Hz DLPF cuts motor vibration noise without affecting slow flight dynamics.
        driver
            .configure_accelerometer(AccelConfig {
                full_scale: AccelFullScale::G4,
                dlpf: AccelDlpf::Hz111,
                dlpf_enable: true,
                sample_rate_div: accel_div,
            })
            .await?;

        // ±500°/s range (headroom over default ±250°/s); 197Hz DLPF unchanged.
        driver
            .configure_gyroscope(GyroConfig {
                full_scale: GyroFullScale::Dps500,
                dlpf: GyroDlpf::Hz197,
                dlpf_enable: true,
                sample_rate_div: gyro_div,
            })
            .await?;

        driver
            .init_magnetometer(MagConfig::default(), &mut embassy_time::Delay)
            .await
            .inspect_err(|e| {
                defmt::error!("error during init_icm20948 {}", defmt::Debug2Format(e));
            })?;

        #[cfg(feature = "dmp")]
        {
            driver.dmp_init(&mut embassy_time::Delay).await?;
            embassy_time::Timer::after(embassy_time::Duration::from_millis(10)).await;
            let dmp_config = icm20948::dmp::DmpConfig::nine_axis().with_sample_rate(100);
            driver.dmp_configure(&dmp_config).await?;
            driver.dmp_enable(true).await?;
            defmt::info!("ICM20948 DMP enabled (9-axis, 100Hz)");
        }

        #[cfg(not(feature = "dmp"))]
        defmt::info!("ICM20948 init OK (mag enabled)");

        Ok(Self { driver })
    }

    #[cfg(feature = "dmp")]
    pub async fn read_dmp(
        &mut self,
    ) -> Result<Option<icm20948::dmp::DmpData>, icm20948::Error<I::Error>> {
        self.driver.dmp_read_fifo().await
    }
}

impl<I: I2c> ImuRead for Sensor<Icm20948Driver<I2cInterface<I>>> {
    type Error = icm20948::Error<I::Error>;

    async fn read(&mut self) -> Result<(Vector3<f32>, Vector3<f32>), Self::Error> {
        let acc = self.driver.read_accelerometer().await?;
        let gyro = self.driver.read_gyroscope_radians().await?;
        Ok((
            Vector3::new(acc.x, acc.y, acc.z),
            Vector3::new(gyro.x, gyro.y, gyro.z),
        ))
    }
}

#[cfg(feature = "calibrate")]
impl<I: I2c> Sensor<Icm20948Driver<I2cInterface<I>>> {
    pub async fn run_calibration(&mut self) -> ! {
        //     loop {
        //         let a = self.driver.read_accelerometer_raw().await;
        //         let g = self.driver.read_gyroscope_raw().await;
        //         let m = self.driver.read_magnetometer_raw().await;
        //         match (a, g, m) {
        //             (Ok(a), Ok(g), Ok((mx, my, mz))) => esp_println::println!(
        //                 "Raw:{},{},{},{},{},{},{},{},{}",
        //                 a.x, a.y, a.z, g.x, g.y, g.z, mx, my, mz
        //             ),
        //             (Err(e), _, _) => esp_println::println!("accel error: {:?}", e),
        //             (_, Err(e), _) => esp_println::println!("gyro error: {:?}", e),
        //             (_, _, Err(e)) => esp_println::println!("mag error: {:?}", e),
        //         }
        //         embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
        //     }
        // }

        loop {
            match self.driver.read_magnetometer().await {
                Ok(m) => defmt::info!("{},{},{}", m.x, m.y, m.z),
                Err(e) => defmt::error!("mag error: {}", defmt::Debug2Format(&e)),
            }
            embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
        }
    }
}

impl<I: I2c> ImuReadMag for Sensor<Icm20948Driver<I2cInterface<I>>> {
    async fn read_mag(
        &mut self,
    ) -> Result<(Vector3<f32>, Vector3<f32>, Vector3<f32>), Self::Error> {
        let acc = self.driver.read_accelerometer().await?;
        let gyro = self.driver.read_gyroscope_radians().await?;
        let mag = self.driver.read_magnetometer().await?;
        Ok((
            Vector3::new(acc.x, acc.y, acc.z),
            Vector3::new(gyro.x, gyro.y, gyro.z),
            Vector3::new(mag.x, mag.y, mag.z),
        ))
    }
}
