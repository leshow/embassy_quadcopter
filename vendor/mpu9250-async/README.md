# `mpu9250-async`

## What Works

* Reading the accelerometer, gyroscope, temperature, magnetometer sensor
    * raw
    * scaled
    * roll/pitch estimation
* Motion Detection
* Setting Accel/Gyro Ranges/Sensitivity
* Setting Accel HPF/LPF
* Setting AK8963 Magnetometer

## Basic usage

To use this driver you must provide a concrete `embedded_hal_async` implementation.

## Acknowledgements
This crate was originally forked from and inspired by the [`mpu6050-async` crate](https://crates.io/crates/mpu6050-async) by [max-dau](https://crates.io/users/max-dau). It has been modified to add support for the MPU9250's internal AK8963 magnetometer.