use esp_hal::gpio;

use crate::{LOG_EVERY_N, Motors, Sensor20948, THROTTLE_CAP, fusion, wifi};

// reads one DMP frame, logs the quaternion
// returns None when no usable data was produced this cycle
async fn read_dmp(
    sensor: &mut Sensor20948<'_>,
    log_counter: &mut u32,
) -> Option<icm20948::dmp::Quaternion> {
    use icm20948::dmp::DmpData;
    match sensor.read_dmp().await {
        Ok(Some(DmpData {
            quaternion_6axis: Some(quat),
            ..
        }))
        | Ok(Some(DmpData {
            quaternion_9axis: Some(quat),
            ..
        })) => {
            *log_counter += 1;
            if *log_counter >= LOG_EVERY_N {
                let euler = quat.to_euler_angles();
                *log_counter = 0;
                defmt::debug!(
                    "DMP w: {} x: {} y: {} z: {} | roll: {}° pitch: {}° yaw: {}°",
                    quat.w,
                    quat.x,
                    quat.y,
                    quat.z,
                    euler.roll * fusion::RAD_TO_DEG,
                    euler.pitch * fusion::RAD_TO_DEG,
                    euler.yaw * fusion::RAD_TO_DEG,
                );
            }
            Some(quat)
        }
        Err(icm20948::Error::FifoOverflow) => {
            defmt::warn!("DMP FIFO overflow — resetting");
            sensor.reset_fifo().await.ok();
            None
        }
        Err(e) => {
            defmt::error!("DMP read error: {}", defmt::Debug2Format(&e));
            None
        }
        Ok(_) => None,
    }
}

pub async fn run_dmp(
    mut sensor: Sensor20948<'_>,
    mut int_pin: gpio::Input<'static>,
    motors: Motors<'_>,
) {
    let mut log_counter: u32 = 0;
    let mut last_packet: Option<libs::control::ControlPacket> = None;

    loop {
        int_pin.wait_for_high().await;
        if read_dmp(&mut sensor, &mut log_counter).await.is_none() {
            continue;
        }
        let controls = wifi::CONTROLS.lock().await;
        if let Some(pkt) = *controls {
            if last_packet.is_some_and(|s| !s.armed()) && pkt.flags().armed() {
                defmt::info!("ARMED - fly away!");
            }
            if pkt.flags().armed() {
                use libs::control::ControlPacket;

                defmt::trace!("got control pkt {:?}", defmt::Debug2Format(&pkt));
                let ControlPacket {
                    throttle,
                    roll: _,
                    pitch: _,
                    yaw: _,
                    ..
                } = pkt;
                motors.set_all_duty(throttle.min(THROTTLE_CAP)); // safety cap during testing
            } else {
                // disarm
                motors.set_all_duty(0);
            }
            last_packet = Some(pkt);
        }
    }
}

// visualize-only DMP loop: just log orientation, no motor control, no WiFi
#[cfg(feature = "visualize")]
pub async fn run_dmp_visualizer(mut sensor: Sensor20948<'_>, mut int_pin: gpio::Input<'static>) {
    let mut log_counter: u32 = 0;
    loop {
        int_pin.wait_for_high().await;
        read_dmp(&mut sensor, &mut log_counter).await;
    }
}
