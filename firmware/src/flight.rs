use embassy_time::{Duration, Instant};
use esp_hal::gpio;
#[cfg(feature = "telemetry")]
use libs::control::TelemetryPacket;
#[cfg(feature = "dmp")]
use nalgebra::Quaternion;
use nalgebra::{UnitQuaternion, Vector3};

#[cfg(not(feature = "dmp"))]
use crate::sensors::ImuRead;
use crate::{Motors, Sensor20948, fusion, wifi};

// publishes a telemetry snapshot for the wifi udp_task to reply with on the next control packet.
// called once per loop iteration, from whichever exit point (early continue or full mix) is
// actually taken, so telemetry is only ever built once per tick.
#[cfg(feature = "telemetry")]
#[allow(clippy::too_many_arguments)]
async fn publish_telemetry(
    euler: (f32, f32, f32),       // (roll, pitch, yaw) radians
    motors: (u16, u16, u16, u16), // fl fr rl rr
    armed: bool,
    failsafe: bool,
    gyro: Vector3<f32>,
) {
    let roll_deg = euler.0 * fusion::RAD_TO_DEG;
    let pitch_deg = euler.1 * fusion::RAD_TO_DEG;
    let yaw_deg = euler.2 * fusion::RAD_TO_DEG;

    #[cfg(not(feature = "telemetry-verbose"))]
    let _ = &gyro;

    #[cfg(not(feature = "telemetry-verbose"))]
    let pkt = TelemetryPacket::new(roll_deg, pitch_deg, yaw_deg, motors, armed, failsafe);
    #[cfg(feature = "telemetry-verbose")]
    let pkt = TelemetryPacket::new(roll_deg, pitch_deg, yaw_deg, motors, armed, failsafe, gyro);

    *wifi::TELEMETRY.lock().await = Some((pkt, Instant::now()));
}

// calibrated_gyro is i16 at +/-2000 dps full scale, hardware DLPF at 51 Hz already applied
#[cfg(feature = "dmp")]
const GYRO_SCALE: f32 = 2000.0 * fusion::DEG_TO_RAD / 32768.0; // i16 → rad/s

// max roll/pitch command from stick (+/- 25 deg)
const MAX_TILT_RAD: f32 = 25.0 * fusion::DEG_TO_RAD;

// outer loop P gains, matches flix ROLL_P / YAW_P
const ANGLE_P_ROLL_PITCH: f32 = 6.0;
const ANGLE_P_YAW: f32 = 0.5;

const RATE_KP_ROLL_PITCH: f32 = 0.03;
const RATE_KI_ROLL_PITCH: f32 = 0.01;
const RATE_KD_ROLL_PITCH: f32 = 0.001;

const RATE_KP_YAW: f32 = 0.0;
const RATE_KI_YAW: f32 = 0.0;
const RATE_KD_YAW: f32 = 0.0;

// max angle any single DMP sample can plausibly rotate by since the last accepted sample
#[cfg(feature = "dmp")]
const MAX_QUAT_JUMP_RAD: f32 = 60.0 * fusion::DEG_TO_RAD;

/// PID
///
/// Proportional-Integral-Derivative controller — computes a corrective output
/// from three components of the error signal:
///
/// P proportional: output -> current error (instant response, but can oscillate)
/// I integral:     output -> accumulated past error (eliminates steady-state offset)
/// D derivative:   output -> rate of change of error (damps oscillation)
///
/// Combined: output = kp·e + ki·∫e·dt + kd·(de/dt)
struct Pid {
    /// proportional gain — how hard to push per unit of current error
    kp: f32,
    /// integral gain — how hard to push per unit of accumulated error
    ki: f32,
    /// derivative gain — how hard to damp per unit of error rate
    kd: f32,
    /// running sum of error×dt, clamped to ±integral_limit
    integral: f32,
    /// error from last tick for de/dt; NaN until first update to avoid derivative spike on arm
    prev_error: f32,
    /// anti-windup clamp — keeps integral from growing unbounded
    integral_limit: f32,
}

impl Pid {
    const fn new(kp: f32, ki: f32, kd: f32, integral_limit: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: f32::NAN,
            integral_limit,
        }
    }

    fn update(&mut self, error: f32, dt: f32) -> f32 {
        // matches flix: reset if dt is zero or impossibly large
        if dt <= 0.0 || dt > 0.5 {
            self.reset();
            return self.kp * error;
        }
        self.integral =
            (self.integral + error * dt).clamp(-self.integral_limit, self.integral_limit);
        // no software LPF on derivative — hardware DLPF at 51 Hz handles it
        let derivative = if self.prev_error.is_nan() {
            0.0
        } else {
            (error - self.prev_error) / dt
        };
        self.prev_error = error;
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }

    fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = f32::NAN;
    }
}

// normalises an angle to [-pi, pi] for the shortest-path yaw error
fn wrap_angle(a: f32) -> f32 {
    use core::f32::consts::PI;
    // fmodf gives the same sign as the dividend, so shift into [0, 2pi) before subtracting back
    let r = libm::fmodf(a + PI, 2.0 * PI);
    (if r < 0.0 { r + 2.0 * PI } else { r }) - PI
}
// or:
// use num_traits::ops::euclid::Euclid;

// fn wrap_angle(a: f32) -> f32 {
//     use core::f32::consts::PI;
//     (a + PI).rem_euclid(2.0 * PI) - PI
// }

// reads one DMP FIFO frame, returns (quaternion, gyro_rad_s) or None if no usable data
// DMP software fusion. calibrated_gyro replaces raw gyro register reads
#[cfg(feature = "dmp")]
async fn read_dmp(
    sensor: &mut Sensor20948<'_>,
    log_counter: &mut u32,
    last_quat: &mut Option<icm20948::dmp::Quaternion>,
) -> Option<(UnitQuaternion<f32>, Vector3<f32>)> {
    match sensor.read_dmp().await {
        Ok(Some(data)) => {
            let quat = data.quaternion_6axis.or(data.quaternion_9axis)?;

            // reject corrupted DMP packets - a valid unit quaternion always has norm ~= 1,
            // but a byte-misaligned read (e.g. a flipped header bit) produces components
            // wildly outside that bound rather than a subtle numerical error, so a generous
            // tolerance still catches it without rejecting legitimate quantization noise
            let norm_sq = quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z;
            if !(0.9..=1.1).contains(&norm_sq) {
                defmt::warn!(
                    "DMP quaternion failed norm check: w: {} x: {} y: {} z: {} norm_sq: {}",
                    quat.w,
                    quat.x,
                    quat.y,
                    quat.z,
                    norm_sq
                );
                return None;
            }

            // TODO: maybe remove
            // was getting some corrupted packets out of the FIFO queue
            // reject packets that still pass the norm check above but imply an impossible
            // jump from the last accepted orientation - the 6-axis packet only carries x/y/z
            // on the wire and derives w to force unit norm, so corrupted x/y/z bytes that
            // happen to sum to <= 1 slip past the norm check as a "valid" but wrong orientation
            if let Some(prev) = *last_quat {
                let dot = (prev.w * quat.w + prev.x * quat.x + prev.y * quat.y + prev.z * quat.z)
                    .clamp(-1.0, 1.0);
                let jump = 2.0 * libm::acosf(dot.abs());
                if jump > MAX_QUAT_JUMP_RAD {
                    defmt::warn!(
                        "DMP quaternion failed continuity check: {}° jump from last sample",
                        jump * fusion::RAD_TO_DEG
                    );
                    return None;
                }
            }

            let (gx, gy, gz) = data.calibrated_gyro?;
            let gyro = Vector3::new(
                gx as f32 * GYRO_SCALE,
                gy as f32 * GYRO_SCALE,
                gz as f32 * GYRO_SCALE,
            );
            // validated above (norm + continuity) so this is trusted, legitimate data - but
            // norm_sq is only checked to within 0.9..=1.1, not exactly 1.0, and transform_vector
            // on a non-unit quaternion produces a distorted (not just imprecise) result, so
            // normalize here rather than new_unchecked
            let uq = UnitQuaternion::new_normalize(Quaternion::new(quat.w, quat.x, quat.y, quat.z));
            *log_counter += 1;
            if *log_counter >= crate::LOG_EVERY_N {
                *log_counter = 0;
                let (roll, pitch, yaw) = uq.euler_angles();
                defmt::debug!(
                    "DMP w: {} x: {} y: {} z: {} | roll: {}° pitch: {}° yaw: {}°",
                    uq.w,
                    uq.i,
                    uq.j,
                    uq.k,
                    roll * fusion::RAD_TO_DEG,
                    pitch * fusion::RAD_TO_DEG,
                    yaw * fusion::RAD_TO_DEG,
                );
            }
            *last_quat = Some(quat);
            Some((uq, gyro))
        }
        Err(icm20948::Error::FifoOverflow) => {
            defmt::warn!("DMP FIFO overflow");
            // flag whatever telemetry is already cached so ground control sees this happened,
            // even though we have no fresh sample to publish this tick
            #[cfg(feature = "telemetry")]
            if let Some((pkt, _)) = wifi::TELEMETRY.lock().await.as_mut() {
                pkt.set_fifo_overflow(true);
            }
            None
        }
        Err(e) => {
            defmt::error!("DMP read error: {}", defmt::Debug2Format(&e));
            None
        }
        Ok(None) => None,
    }
}

// generalizes "fuse one accel+gyro sample into an orientation" across whichever filter
// FusionBuilder was configured with. Madgwick/Vqf/Mahony all expose a matching IMU-only
// update_imu for the ICM20948; Complementary isn't included since its only ICM20948 impl
// requires a magnetometer reading, which read_fusion below doesn't take.
#[cfg(not(feature = "dmp"))]
trait ImuFusion {
    fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32>;
}

#[cfg(not(feature = "dmp"))]
impl ImuFusion for fusion::Fusion<fusion::ICM20948, fusion::Madgwick> {
    fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        Self::update_imu(self, dt, a, g)
    }
}

#[cfg(not(feature = "dmp"))]
impl ImuFusion for fusion::Fusion<fusion::ICM20948, fusion::Vqf> {
    fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        Self::update_imu(self, dt, a, g)
    }
}

#[cfg(not(feature = "dmp"))]
impl ImuFusion for fusion::Fusion<fusion::ICM20948, fusion::Mahony> {
    fn update_imu(&mut self, dt: f32, a: Vector3<f32>, g: Vector3<f32>) -> UnitQuaternion<f32> {
        Self::update_imu(self, dt, a, g)
    }
}

// reads raw accel/gyro and fuses them into an orientation quaternion via whichever filter
// implements ImuFusion. same (Quaternion, gyro_rad_s) shape as read_dmp, so the rest of the
// control loop is unchanged regardless of which sensor-read path or filter is active
#[cfg(not(feature = "dmp"))]
async fn read_fusion<F: ImuFusion>(
    sensor: &mut Sensor20948<'_>,
    filter: &mut F,
    dt: f32,
    log_counter: &mut u32,
) -> Option<(UnitQuaternion<f32>, Vector3<f32>)> {
    match sensor.read().await {
        Ok((accel, gyro)) => {
            let quat = filter.update_imu(dt, accel, gyro);

            *log_counter += 1;
            if *log_counter >= crate::LOG_EVERY_N {
                *log_counter = 0;
                let (roll, pitch, yaw) = quat.euler_angles();
                defmt::debug!(
                    "fusion w: {} x: {} y: {} z: {} | roll: {}° pitch: {}° yaw: {}°",
                    quat.w,
                    quat.i,
                    quat.j,
                    quat.k,
                    roll * fusion::RAD_TO_DEG,
                    pitch * fusion::RAD_TO_DEG,
                    yaw * fusion::RAD_TO_DEG,
                );
            }
            Some((quat, gyro))
        }
        Err(e) => {
            defmt::error!("IMU read error: {}", defmt::Debug2Format(&e));
            None
        }
    }
}

fn dur_since(last_instant: &mut Option<Instant>) -> f32 {
    let now = Instant::now();
    let dt = last_instant
        .map(|t| now.duration_since(t).as_micros() as f32 / 1_000_000.0)
        .unwrap_or(0.0);
    *last_instant = Some(now);

    dt
}
// control loop
pub async fn run_control(
    mut sensor: Sensor20948<'_>,
    mut int_pin: gpio::Input<'static>,
    motors: Motors<'_>,
) {
    let mut log_counter: u32 = 0;
    #[cfg(feature = "dmp")]
    let mut last_quat: Option<icm20948::dmp::Quaternion> = None;
    #[cfg(not(feature = "dmp"))]
    let mut fusion_filter = fusion::FusionBuilder::new().icm20948().madgwick().build();

    // inner rate PIDs
    let mut roll_pid = Pid::new(
        RATE_KP_ROLL_PITCH,
        RATE_KI_ROLL_PITCH,
        RATE_KD_ROLL_PITCH,
        0.3,
    );
    let mut pitch_pid = Pid::new(
        RATE_KP_ROLL_PITCH,
        RATE_KI_ROLL_PITCH,
        RATE_KD_ROLL_PITCH,
        0.3,
    );
    let mut yaw_pid = Pid::new(RATE_KP_YAW, RATE_KI_YAW, RATE_KD_YAW, 0.3);

    let mut target_yaw: f32 = 0.0;
    let mut yaw_init = false;
    let mut last_armed = false;
    let mut last_instant: Option<Instant> = None;

    loop {
        int_pin.wait_for_high().await;

        // controls/armed/failsafe check runs every tick, independent of whether the DMP
        // read below succeeds
        let controls = *wifi::CONTROLS.lock().await;
        let fresh = controls.is_some_and(|(_, at)| at.elapsed() < Duration::from_millis(500));
        let armed = fresh && controls.is_some_and(|(p, _)| p.armed());

        if !last_armed && armed {
            defmt::info!("ARMED");
            roll_pid.reset();
            pitch_pid.reset();
            yaw_pid.reset();
            yaw_init = false;
        } else if last_armed && !armed {
            defmt::info!("DISARMED");
        }
        last_armed = armed;

        if !armed {
            motors.turn_off();
        }

        #[cfg(feature = "dmp")]
        let (quat, g, dt) = {
            let (quat, g) = match read_dmp(&mut sensor, &mut log_counter, &mut last_quat).await {
                Some(d) => d,
                None => continue,
            };
            let dt = dur_since(&mut last_instant);
            (quat, g, dt)
        };

        #[cfg(not(feature = "dmp"))]
        let (quat, g, dt) = {
            let dt = dur_since(&mut last_instant);
            let (quat, g) =
                match read_fusion(&mut sensor, &mut fusion_filter, dt, &mut log_counter).await {
                    Some(d) => d,
                    None => continue,
                };
            (quat, g, dt)
        };

        let euler = quat.euler_angles();

        // failsafe: zero motors if no packet, packet is stale (>500 ms), or disarmed
        let pkt = match controls.filter(|_| armed) {
            Some((p, _)) => p,
            None => {
                #[cfg(feature = "telemetry")]
                publish_telemetry(euler, (0, 0, 0, 0), armed, !fresh, g).await;
                continue;
            }
        };

        // would be controlAttitude (flix)
        // DMP gives us the fused quaternion directly instead of running Mahony/Madgwick

        let actual_yaw = euler.2;
        let t = pkt.throttle as f32 / 100.0;

        // latch heading on first armed tick (flix: yawTarget initialised from attitude.getYaw())
        if !yaw_init {
            target_yaw = actual_yaw;
            yaw_init = true;
        }

        // while near the ground keep target tracking actual so handling the drone doesn't build a large error
        if t < 0.15 {
            target_yaw = actual_yaw;
        }

        // heading hold: only update target when yaw stick is active (matches flix interpretControls)
        if pkt.yaw.abs() >= 0.1 {
            target_yaw = actual_yaw;
        }
        // stick feedforward adds directly to yaw rate setpoint (matches flix ratesExtra)
        // deadzone mirrors heading hold threshold — prevents stick drift from killing FR at idle
        let yaw_ff = if pkt.yaw.abs() >= 0.1 {
            -pkt.yaw * core::f32::consts::PI // ±π rad/s
        } else {
            0.0
        };

        // build target attitude quaternion from stick angles (flix Quaternion::fromEuler)
        let target_quat = UnitQuaternion::from_euler_angles(
            pkt.roll * MAX_TILT_RAD,
            pkt.pitch * MAX_TILT_RAD,
            target_yaw,
        );

        // like controlTorque / motor mixing (flix)
        if t < 0.05 {
            motors.turn_off();
            #[cfg(feature = "telemetry")]
            publish_telemetry(euler, (0, 0, 0, 0), armed, !fresh, g).await;
            continue;
        }

        // up-vector cross product gives roll/pitch error (flix rotationVectorBetween)
        // arg order matches flix: actual * target (swapped gives negated error vector)
        let up = Vector3::z();
        let att_err = quat
            .transform_vector(&up)
            .cross(&target_quat.transform_vector(&up)); // flix Vector::rotationVectorBetween - cross product of two up-vectors gives the attitude error
        let roll_rate_sp = ANGLE_P_ROLL_PITCH * att_err.x;
        let pitch_rate_sp = ANGLE_P_ROLL_PITCH * att_err.y;
        let yaw_rate_sp = ANGLE_P_YAW * wrap_angle(target_yaw - actual_yaw) + yaw_ff;

        // like controlRates (flix)
        // calibrated_gyro replaces flix's raw gyro register reads; hardware DLPF replaces software LPF
        let roll_torque = roll_pid.update(roll_rate_sp - g.x, dt);
        let pitch_torque = pitch_pid.update(pitch_rate_sp - g.y, dt);
        let yaw_torque = yaw_pid.update(yaw_rate_sp - g.z, dt);

        let mut fl = t + roll_torque - pitch_torque + yaw_torque;
        let mut fr = t - roll_torque - pitch_torque - yaw_torque;
        let mut rl = t + roll_torque + pitch_torque - yaw_torque;
        let mut rr = t - roll_torque + pitch_torque + yaw_torque;

        // desaturate: reduce all motors equally so the highest stays at 1.0 (flix desaturate())
        let max = fl.max(fr).max(rl).max(rr);
        if max > 1.0 {
            let excess = max - 1.0;
            fl -= excess;
            fr -= excess;
            rl -= excess;
            rr -= excess;
        }

        // disabled: flix doesn't floor individual motors during in-flight mixing either - it
        // only forces a flat idle when thrustTarget < 0.1 (handled above via the t < 0.05 cutoff)
        // and otherwise relies on the final per-motor clamp in Motors::set_motors. leaving this
        // here commented out in case we want it back for a different reason later.
        // let motor_min = 0.05;
        // let min = fl.min(fr).min(rl).min(rr);
        // if min < motor_min {
        //     let deficit = motor_min - min;
        //     fl += deficit;
        //     fr += deficit;
        //     rl += deficit;
        //     rr += deficit;
        // }

        let (dfl, dfr, drl, drr) = motors.set_motors(fl, fr, rl, rr);
        defmt::trace!(
            "torques roll={} pitch={} yaw={} | mix fl={} fr={} rl={} rr={} | duty fl={} fr={} rl={} rr={}",
            roll_torque,
            pitch_torque,
            yaw_torque,
            fl,
            fr,
            rl,
            rr,
            dfl,
            dfr,
            drl,
            drr,
        );

        #[cfg(feature = "telemetry")]
        publish_telemetry(
            euler,
            (dfl as u16, dfr as u16, drl as u16, drr as u16),
            armed,
            false,
            g,
        )
        .await;
    }
}

// visualize-only loop: log orientation, no motor control, no WiFi
#[cfg(all(feature = "visualize", feature = "dmp"))]
pub async fn run_dmp_visualizer(mut sensor: Sensor20948<'_>, mut int_pin: gpio::Input<'static>) {
    let mut log_counter: u32 = 0;
    let mut last_quat: Option<icm20948::dmp::Quaternion> = None;
    loop {
        int_pin.wait_for_high().await;
        read_dmp(&mut sensor, &mut log_counter, &mut last_quat).await;
    }
}

// visualize-only loop for the fusion path: log orientation, no motor control, no WiFi
#[cfg(all(feature = "visualize", not(feature = "dmp")))]
pub async fn run_fusion_visualizer(mut sensor: Sensor20948<'_>, mut int_pin: gpio::Input<'static>) {
    let mut log_counter: u32 = 0;
    let mut fusion_filter = fusion::FusionBuilder::new().icm20948().madgwick().build();
    let mut last_instant: Option<Instant> = None;
    loop {
        int_pin.wait_for_high().await;
        let dt = dur_since(&mut last_instant);
        read_fusion(&mut sensor, &mut fusion_filter, dt, &mut log_counter).await;
    }
}
