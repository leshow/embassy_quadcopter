//! AK8963 async magnetometer driver

use embedded_hal_async::{delay::DelayNs, i2c::I2c};

pub const I2C_ADDRESS: u8 = 0x0c;

#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum Register {
    WHOAMI = 0x00,
    INFO = 0x01,
    ST1 = 0x02,
    XOUTL = 0x03,
    XOUTH = 0x04,
    YOUTL = 0x05,
    YOUTH = 0x06,
    ZOUTL = 0x07,
    ZOUTH = 0x08,
    ST2 = 0x09,
    CNTL1 = 0x0A,
    CNTL2 = 0x0B,
    ASTC = 0x0C,
    I2CDIS = 0x0F,
    ASAX = 0x10,
    ASAY = 0x11,
    ASAZ = 0x12,
}

/// ST1 register: bit 0 is DRDY (data ready), bit 1 is DOR (data overrun)
const ST1_DRDY: u8 = 0x01;

/// ST2 register: bit 3 is HOFL (magnetic sensor overflow)
const ST2_HOFL: u8 = 0x08;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Error<E> {
    I2c(E),
    InvalidDevice(u8),
    /// Magnetometer data was not ready (ST1.DRDY == 0)
    DataNotReady,
    /// Magnetic sensor overflowed (ST2.HOFL == 1)
    Overflow,
}

#[derive(Debug)]
pub struct Ak8963 {
    asa: [f32; 3],
}

impl Ak8963 {
    pub async fn init<I2C, E, D>(i2c: &mut I2C, delay: &mut D) -> Result<Self, Error<E>>
    where
        I2C: I2c<Error = E>,
        D: DelayNs,
    {
        let mut ak = Self { asa: [1.0; 3] };

        ak.write(i2c, Register::CNTL2, 0x01).await?;
        delay.delay_ms(100).await;

        let who = ak.read(i2c, Register::WHOAMI).await?;
        if who != 0x48 {
            return Err(Error::InvalidDevice(who));
        }

        ak.write(i2c, Register::CNTL1, 0x0F).await?;
        delay.delay_ms(100).await;

        let asax = ak.read(i2c, Register::ASAX).await?;
        let asay = ak.read(i2c, Register::ASAY).await?;
        let asaz = ak.read(i2c, Register::ASAZ).await?;

        ak.asa[0] = ((asax as f32 - 128.0) * 0.5 / 128.0) + 1.0;
        ak.asa[1] = ((asay as f32 - 128.0) * 0.5 / 128.0) + 1.0;
        ak.asa[2] = ((asaz as f32 - 128.0) * 0.5 / 128.0) + 1.0;

        ak.write(i2c, Register::CNTL1, 0x16).await?;
        delay.delay_ms(100).await;

        Ok(ak)
    }

    pub async fn read_mag<I2C, E, D>(
        &mut self,
        i2c: &mut I2C,
        delay: &mut D,
    ) -> Result<(f32, f32, f32), Error<E>>
    where
        I2C: I2c<Error = E>,
        D: DelayNs,
    {
        for _ in 0..50 {
            let mut buf = [0u8; 6];
            match self.read_raw(i2c, &mut buf).await {
                Ok(()) => {
                    let x = i16::from_le_bytes([buf[0], buf[1]]);
                    let y = i16::from_le_bytes([buf[2], buf[3]]);
                    let z = i16::from_le_bytes([buf[4], buf[5]]);
                    const SCALE: f32 = 4912.0 / 32760.0;
                    return Ok((
                        x as f32 * self.asa[0] * SCALE,
                        y as f32 * self.asa[1] * SCALE,
                        z as f32 * self.asa[2] * SCALE,
                    ));
                }
                Err(Error::DataNotReady) => delay.delay_ms(1).await,
                Err(e) => return Err(e),
            }
        }
        Err(Error::DataNotReady)
    }

    /// Read raw magnetometer bytes into `buf`.
    ///
    /// Follows the required AK8963 read sequence: ST1 → data registers → ST2.
    /// ST2 must be read to clear the data-ready latch; without it, the sensor
    /// will not update the data registers for the next measurement.
    pub async fn read_raw<I2C, E>(&self, i2c: &mut I2C, buf: &mut [u8; 6]) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        // Read ST1 + 6 data bytes + ST2 in one transaction.
        let mut tmp = [0u8; 8];
        i2c.write_read(I2C_ADDRESS, &[Register::ST1 as u8], &mut tmp)
            .await
            .map_err(Error::I2c)?;

        if tmp[0] & ST1_DRDY == 0 {
            return Err(Error::DataNotReady);
        }

        // ST2 check must occur before returning data, but also clears the latch.
        if tmp[7] & ST2_HOFL != 0 {
            return Err(Error::Overflow);
        }

        buf.copy_from_slice(&tmp[1..7]);
        Ok(())
    }

    async fn read<I2C, E>(&self, i2c: &mut I2C, reg: Register) -> Result<u8, Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        let mut buf = [0];
        i2c.write_read(I2C_ADDRESS, &[reg as u8], &mut buf)
            .await
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    async fn write<I2C, E>(&self, i2c: &mut I2C, reg: Register, val: u8) -> Result<(), Error<E>>
    where
        I2C: I2c<Error = E>,
    {
        i2c.write(I2C_ADDRESS, &[reg as u8, val])
            .await
            .map_err(Error::I2c)
    }
}

impl<E> From<Error<E>> for crate::Mpu9250Error<E> {
    fn from(e: Error<E>) -> Self {
        match e {
            Error::I2c(e) => crate::Mpu9250Error::I2c(e),
            Error::InvalidDevice(id) => crate::Mpu9250Error::InvalidChipId(id),
            Error::DataNotReady => crate::Mpu9250Error::MagDataNotReady,
            Error::Overflow => crate::Mpu9250Error::MagOverflow,
        }
    }
}
