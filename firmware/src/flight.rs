use esp_hal::gpio;
use nalgebra::Vector3;

use crate::{Motors, Sensor20948, fusion, wifi};

// calibrated_gyro is i16 at ±2000 dps full scale, hardware DLPF at 51 Hz already applied
const GYRO_SCALE: f32 = 2000.0 * fusion::DEG_TO_RAD / 32768.0; // i16 → rad/s

// max roll/pitch command from stick (±25°)
const MAX_TILT_RAD: f32 = 25.0 * fusion::DEG_TO_RAD;

// outer loop P gains, matches flix ROLL_P / YAW_P
const ANGLE_P_ROLL_PITCH: f32 = 6.0;
const ANGLE_P_YAW: f32 = 3.0;

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

// rotate (0, 0, 1) by quaternion (flix Quaternion::rotateVector(up, q))
fn rotate_up(q: &icm20948::dmp::Quaternion) -> Vector3<f32> {
    let (w, x, y, z) = (q.w, q.x, q.y, q.z);
    Vector3::new(
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        w * w - x * x - y * y + z * z,
    )
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

// flix Quaternion::fromEuler ZYX convention
fn quat_from_euler(roll: f32, pitch: f32, yaw: f32) -> icm20948::dmp::Quaternion {
    let (sr, cr) = (libm::sinf(roll * 0.5), libm::cosf(roll * 0.5));
    let (sp, cp) = (libm::sinf(pitch * 0.5), libm::cosf(pitch * 0.5));
    let (sy, cy) = (libm::sinf(yaw * 0.5), libm::cosf(yaw * 0.5));
    icm20948::dmp::Quaternion::new(
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )
}

// reads one DMP FIFO frame, returns (quaternion, gyro_rad_s) or None if no usable data
// DMP software fusion. calibrated_gyro replaces raw gyro register reads
async fn read_dmp(
    sensor: &mut Sensor20948<'_>,
    log_counter: &mut u32,
) -> Option<(icm20948::dmp::Quaternion, Vector3<f32>)> {
    match sensor.read_dmp().await {
        Ok(Some(data)) => {
            let quat = data.quaternion_6axis.or(data.quaternion_9axis)?;
            let (gx, gy, gz) = data.calibrated_gyro?;
            let gyro = Vector3::new(
                gx as f32 * GYRO_SCALE,
                gy as f32 * GYRO_SCALE,
                gz as f32 * GYRO_SCALE,
            );
            *log_counter += 1;
            if *log_counter >= crate::LOG_EVERY_N {
                *log_counter = 0;
                let e = quat.to_euler_angles();
                defmt::debug!(
                    "DMP w: {} x: {} y: {} z: {} | roll: {}° pitch: {}° yaw: {}°",
                    quat.w,
                    quat.x,
                    quat.y,
                    quat.z,
                    e.roll * fusion::RAD_TO_DEG,
                    e.pitch * fusion::RAD_TO_DEG,
                    e.yaw * fusion::RAD_TO_DEG,
                );
            }
            Some((quat, gyro))
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
        Ok(None) => None,
    }
}

// control loop
pub async fn run_dmp(
    mut sensor: Sensor20948<'_>,
    mut int_pin: gpio::Input<'static>,
    motors: Motors<'_>,
) {
    let mut log_counter: u32 = 0;

    // inner rate PIDs, gains match flix ROLLRATE / PITCHRATE / YAWRATE
    let mut roll_pid = Pid::new(0.05, 0.2, 0.001, 0.3);
    let mut pitch_pid = Pid::new(0.05, 0.2, 0.001, 0.3);
    let mut yaw_pid = Pid::new(0.3, 0.0, 0.0, 0.3);

    let mut target_yaw: f32 = 0.0;
    let mut yaw_init = false;
    let mut last_armed = false;
    let mut last_instant: Option<embassy_time::Instant> = None;

    loop {
        int_pin.wait_for_high().await;

        let (quat, g) = match read_dmp(&mut sensor, &mut log_counter).await {
            Some(d) => d,
            None => continue,
        };

        let now = embassy_time::Instant::now();
        let dt = last_instant
            .map(|t| now.duration_since(t).as_micros() as f32 / 1_000_000.0)
            .unwrap_or(0.0);
        last_instant = Some(now);

        // snapshot controls — failsafe: zero motors if no packet or packet is stale (>500 ms)
        let pkt = match *wifi::CONTROLS.lock().await {
            Some((p, received_at))
                if received_at.elapsed() < embassy_time::Duration::from_millis(500) =>
            {
                p
            }
            _ => {
                motors.set_all_duty(0);
                continue;
            }
        };

        let armed = pkt.armed();

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
            motors.set_all_duty(0);
            continue;
        }

        // would be controlAttitude (flix)
        // DMP gives us the fused quaternion directly instead of running Mahony/Madgwick

        let euler = quat.to_euler_angles();
        let actual_yaw = euler.yaw;

        // latch heading on first armed tick (flix: yawTarget initialised from attitude.getYaw())
        if !yaw_init {
            target_yaw = actual_yaw;
            yaw_init = true;
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
        let target_quat = quat_from_euler(
            pkt.roll * MAX_TILT_RAD,
            pkt.pitch * MAX_TILT_RAD,
            target_yaw,
        );

        // like controlTorque / motor mixing (flix)
        let t = pkt.throttle as f32 / 100.0;
        if t < 0.05 {
            motors.set_all_duty(0);
            continue;
        }

        // up-vector cross product gives roll/pitch error (flix rotationVectorBetween)
        // arg order matches flix: actual * target (swapped gives negated error vector)
        let att_err = rotate_up(&quat).cross(&rotate_up(&target_quat)); // flix Vector::rotationVectorBetween — cross product of two up-vectors gives the attitude error
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
    }
}

// visualize-only loop: log orientation, no motor control, no WiFi
#[cfg(feature = "visualize")]
pub async fn run_dmp_visualizer(mut sensor: Sensor20948<'_>, mut int_pin: gpio::Input<'static>) {
    let mut log_counter: u32 = 0;
    loop {
        int_pin.wait_for_high().await;
        read_dmp(&mut sensor, &mut log_counter).await;
    }
}
