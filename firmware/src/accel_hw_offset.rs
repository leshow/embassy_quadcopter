// generated file,
// I left this in here but it's not called because I was lookng through peterkrull's lib
// and it stores hw bias differently from the data sheet. I wasn't sure which to use so
// I opted not to use either and do it in software. leaving this here for possible future use:

// writes accelerometer bias directly into the ICM-20948's hardware offset-cancellation
// registers, the same approach peterkrull/icm20948-async's set_acc_offsets uses - the chip
// subtracts the bias in silicon before we ever read data, so there's no software subtraction
// (ours or the icm20948-rs crate's AccelCalibration::apply()) left to overflow.
//
// NOT WIRED UP: icm20948-rs doesn't expose raw register access on an already-constructed
// Icm20948Driver (the register interface field is private), so using this means reclaiming the
// raw I2C bus via `driver.release()` -> `interface.release()`, writing these registers, then
// rebuilding the driver *without* calling `.init()`/`.reinit()` again - those trigger a device
// reset, which wipes the offset registers straight back to their reset defaults. Kept here,
// unused, as the alternative to the software (flix-style) approach actually wired up.
//
// only does bias/offset - unlike the software approach, there's no hardware scale/gain trim
// register on this chip, so a scale correction can't be expressed this way.

#![allow(dead_code)]

use embedded_hal_async::i2c::I2c;
use icm20948::I2C_ADDRESS_AD0_HIGH;

const REG_BANK_SEL: u8 = 0x7F;
const BANK_1: u8 = 1;

// Bank 1, matches both DS-000189 rev 1.3 sections 9.7-9.12 and peterkrull/icm20948-async's
// src/reg.rs Bank1 enum
const XA_OFFS_H: u8 = 0x14;
const YA_OFFS_H: u8 = 0x17;
const ZA_OFFS_H: u8 = 0x1A;

// matches peterkrull/icm20948-async's set_acc_offsets (src/lib.rs): negate, then split as a
// plain big-endian i16 straight across H and L.
//
// this doesn't match the datasheet's literal bit-field description of the register (XA_OFFS is
// documented as a 15-bit value: H = bits [14:7], L = bits [6:0] in L's bits [7:1], with L's bit
// 0 reserved) - a strict reading of that would pack it like:
//
// fn pack_offset(value: i16) -> [u8; 2] {
//     let bits = (value as u16) & 0x7FFF;
//     let h = (bits >> 7) as u8;
//     let l = ((bits & 0x7F) << 1) as u8;
//     [h, l]
// }
//
// going with peterkrull's version since it's from a driver that's presumably been run against
// real hardware; this file isn't wired up yet regardless, see the module-level note above.
fn pack_offset(value: i16) -> [u8; 2] {
    (-value).to_be_bytes()
}

/// Writes X/Y/Z accelerometer offset-cancellation registers (Bank 1). `offsets` are raw LSB
/// values to cancel out (positive means the sensor reads that much high when it should read 0).
///
/// # Errors
///
/// Returns an error if any I2C transaction fails.
pub async fn write_accel_offsets<I: I2c>(i2c: &mut I, offsets: [i16; 3]) -> Result<(), I::Error> {
    let addr = I2C_ADDRESS_AD0_HIGH;

    i2c.write(addr, &[REG_BANK_SEL, BANK_1 << 4]).await?;

    for (reg, value) in [XA_OFFS_H, YA_OFFS_H, ZA_OFFS_H].into_iter().zip(offsets) {
        let [h, l] = pack_offset(value);
        i2c.write(addr, &[reg, h, l]).await?;
    }

    // back to bank 0, the bank every other driver call expects to find the chip in
    i2c.write(addr, &[REG_BANK_SEL, 0]).await?;

    Ok(())
}
