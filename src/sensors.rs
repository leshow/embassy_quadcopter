use embedded_hal_async::i2c::I2c;
use icm20948::{I2cInterface, Icm20948Driver, MagConfig};
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
        esp_println::println!("MPU6050 init OK");
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
    pub async fn init_icm20948(i2c: I) -> Result<Self, icm20948::Error<I::Error>> {
        let interface = I2cInterface::alternative(i2c);
        let mut driver = Icm20948Driver::new(interface);
        driver.verify_who_am_i().await?;
        driver.init(&mut embassy_time::Delay).await?;

        match driver
            .init_magnetometer(MagConfig::default(), &mut embassy_time::Delay)
            .await
        {
            Ok(_) => {
                esp_println::println!("ICM20948 init OK (mag enabled)");
                Ok(Self { driver })
            }
            Err(e) => {
                esp_println::println!("error during init_icm20948 {:?}", e,);
                Err(e)
            }
        }
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
