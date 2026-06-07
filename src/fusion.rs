use core::marker::PhantomData;

use nalgebra::{Vector2, Vector3};

pub const FLAT_DEG: f32 = 10.0; // dead-zone around flat
pub const STEEP_DEG: f32 = 50.0; // "both LEDs on" threshold
pub const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

const ALPHA_DEFAULT: f32 = 0.98; // complementary filter: trust gyro 98%, accel 2%

// ── Sensor type tags ──────────────────────────────────────────────────────────
/// ```
///         +Y (forward)
///          ↑
///          |
/// -X ------+------ +X (right)
///          |
///          ↓
///         -Y (back)
///
/// +Z points UP out of the chip surface
/// -Z points DOWN into the desk
/// ```
pub struct ICM20948; // 9DOF: accel + gyro + mag
pub struct MPU6050; // 6DOF: accel + gyro only

// ── Filter type tags ──────────────────────────────────────────────────────────
pub struct Complementary;
pub struct Madgwick;

// ── Builder sentinel types ────────────────────────────────────────────────────
pub struct NoSensor;
pub struct NoFilter;

// ── Fusion state ──────────────────────────────────────────────────────────────
pub struct Fusion<S, F> {
    angle_roll: f32,  // radians, rotation around X axis
    angle_pitch: f32, // radians, rotation around Y axis
    alpha: f32,
    _sensor: PhantomData<S>,
    _filter: PhantomData<F>,
}

// ── Builder ───────────────────────────────────────────────────────────────────
pub struct FusionBuilder<S, F> {
    alpha: f32,
    _sensor: PhantomData<S>,
    _filter: PhantomData<F>,
}

impl FusionBuilder<NoSensor, NoFilter> {
    pub fn new() -> Self {
        Self {
            alpha: ALPHA_DEFAULT,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }
}

// Sensor selection
impl<F> FusionBuilder<NoSensor, F> {
    pub fn icm20948(self) -> FusionBuilder<ICM20948, F> {
        FusionBuilder {
            alpha: self.alpha,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }

    pub fn mpu6050(self) -> FusionBuilder<MPU6050, F> {
        FusionBuilder {
            alpha: self.alpha,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }
}

// Filter selection
impl<S> FusionBuilder<S, NoFilter> {
    pub fn complementary(self) -> FusionBuilder<S, Complementary> {
        FusionBuilder {
            alpha: self.alpha,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }

    pub fn madgwick(self) -> FusionBuilder<S, Madgwick> {
        FusionBuilder {
            alpha: self.alpha,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }
}

// Alpha tuning — complementary filter only
impl<S> FusionBuilder<S, Complementary> {
    pub fn with_alpha(mut self, alpha: f32) -> Self {
        self.alpha = alpha;
        self
    }
}

// build() — only callable once both type params are concrete
impl FusionBuilder<ICM20948, Complementary> {
    pub fn build(self) -> Fusion<ICM20948, Complementary> {
        Fusion {
            angle_roll: 0.0,
            angle_pitch: 0.0,
            alpha: self.alpha,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }
}

impl FusionBuilder<MPU6050, Complementary> {
    pub fn build(self) -> Fusion<MPU6050, Complementary> {
        Fusion {
            angle_roll: 0.0,
            angle_pitch: 0.0,
            alpha: self.alpha,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }
}

impl FusionBuilder<ICM20948, Madgwick> {
    pub fn build(self) -> Fusion<ICM20948, Madgwick> {
        Fusion {
            angle_roll: 0.0,
            angle_pitch: 0.0,
            alpha: self.alpha,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }
}

impl FusionBuilder<MPU6050, Madgwick> {
    pub fn build(self) -> Fusion<MPU6050, Madgwick> {
        Fusion {
            angle_roll: 0.0,
            angle_pitch: 0.0,
            alpha: self.alpha,
            _sensor: PhantomData,
            _filter: PhantomData,
        }
    }
}

impl Fusion<ICM20948, Complementary> {
    /// 9DOF complementary filter. Yaw from tilt-compensated magnetometer.
    /// Returns (roll_deg, pitch_deg, yaw_deg).
    pub fn update(
        &mut self,
        dt: f32,
        a: Vector3<f32>,
        g: Vector3<f32>,
        m: Vector3<f32>,
    ) -> (f32, f32, f32) {
        let acc_roll = libm::atan2f(a.y, libm::sqrtf(a.x * a.x + a.z * a.z));
        let acc_pitch = libm::atan2f(-a.x, libm::sqrtf(a.y * a.y + a.z * a.z));

        (self.angle_roll, self.angle_pitch) = utils::complementary_filter(
            self.angle_roll,
            self.angle_pitch,
            g.x,
            g.y,
            dt,
            acc_roll,
            acc_pitch,
            self.alpha,
        );

        let (sr, cr) = (libm::sinf(self.angle_roll), libm::cosf(self.angle_roll));
        let (sp, cp) = (libm::sinf(self.angle_pitch), libm::cosf(self.angle_pitch));
        let mag_xc = m.x * cp + m.z * sp;
        let mag_yc = m.x * sp * sr + m.y * cr - m.z * cp * sr;
        let yaw_deg = libm::atan2f(-mag_yc, mag_xc) * RAD_TO_DEG;

        (
            self.angle_roll * RAD_TO_DEG,
            self.angle_pitch * RAD_TO_DEG,
            yaw_deg,
        )
    }
}

impl Fusion<MPU6050, Complementary> {
    /// 6DOF complementary filter.
    /// `acc_angles` is the output of `mpu.get_acc_angles()`: [roll_rad, pitch_rad].
    /// No magnetometer, so yaw is unavailable. Returns (roll_deg, pitch_deg).
    pub fn update(&mut self, dt: f32, acc_angles: Vector2<f32>, g: Vector3<f32>) -> (f32, f32) {
        (self.angle_roll, self.angle_pitch) = utils::complementary_filter(
            self.angle_roll,
            self.angle_pitch,
            g.x,
            g.y,
            dt,
            acc_angles[0],
            acc_angles[1],
            self.alpha,
        );

        (self.angle_roll * RAD_TO_DEG, self.angle_pitch * RAD_TO_DEG)
    }
}

impl Fusion<ICM20948, Madgwick> {
    /// 9DOF Madgwick MARG — stub, wired to ahrs crate in next step.
    pub fn update(
        &mut self,
        _dt: f32,
        _a: Vector3<f32>,
        _g: Vector3<f32>,
        _m: Vector3<f32>,
    ) -> (f32, f32, f32) {
        todo!("Madgwick MARG not yet implemented")
    }
}

impl Fusion<MPU6050, Madgwick> {
    /// 6DOF Madgwick IMU-only — stub, wired to ahrs crate in next step.
    pub fn update(&mut self, _dt: f32, _acc_angles: Vector2<f32>, _g: Vector3<f32>) -> (f32, f32) {
        todo!("Madgwick IMU-only not yet implemented")
    }
}

pub mod utils {
    /// Core complementary filter step. Alpha is the gyro trust weight (e.g. 0.98).
    pub fn complementary_filter(
        angle_roll: f32,
        angle_pitch: f32,
        gyro_x: f32,
        gyro_y: f32,
        dt: f32,
        acc_roll: f32,
        acc_pitch: f32,
        alpha: f32,
    ) -> (f32, f32) {
        let angle_roll = alpha * (angle_roll + gyro_x * dt) + (1.0 - alpha) * acc_roll;
        let angle_pitch = alpha * (angle_pitch + gyro_y * dt) + (1.0 - alpha) * acc_pitch;
        (angle_roll, angle_pitch)
    }
}
