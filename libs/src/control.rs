//! control packet shared between the ground control binary and ESP32 firmware
#[cfg(feature = "telemetry-verbose")]
use nalgebra::Vector3;

pub const MAGIC: [u8; 4] = *b"QUAD";
pub const DEFAULT_SIZE: usize = 18; // 4 (magic) + 1 (throttle) + 4+4+4 (roll/pitch/yaw f32 be) + 1 (flags)

/// control packet sent from the ground control PC to the ESP32 over UDP.
/// Serialized as big-endian: 1× u8 + 3× f32 + 1× u8 = 14 bytes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlPacket {
    /// Throttle: 0 (min) to 100 (max), as a duty cycle percentage
    pub throttle: u8,
    /// Roll: -1.0 (left) to 1.0 (right)
    pub roll: f32,
    /// Pitch: -1.0 (forward) to 1.0 (backward)
    pub pitch: f32,
    /// Yaw: -1.0 (left) to 1.0 (right)
    pub yaw: f32,
    /// Armed flag: 0 = disarmed, 1 = armed
    flags: Flags,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Flags(u8);

impl Flags {
    pub fn to_bytes(&self) -> u8 {
        self.0.to_be()
    }

    pub fn from_bytes(byte: u8) -> Self {
        Self(byte)
    }

    pub fn armed(&self) -> bool {
        0x01 & self.0 == 1
    }
    pub fn set_armed(&mut self, b: bool) {
        if b {
            self.0 |= 0x01;
        } else {
            self.0 &= !0x01;
        }
    }

    pub fn failsafe(&self) -> bool {
        0x02 & self.0 != 0
    }
    pub fn set_failsafe(&mut self, b: bool) {
        if b {
            self.0 |= 0x02;
        } else {
            self.0 &= !0x02;
        }
    }

    pub fn fifo_overflow(&self) -> bool {
        0x04 & self.0 != 0
    }
    pub fn set_fifo_overflow(&mut self, b: bool) {
        if b {
            self.0 |= 0x04;
        } else {
            self.0 &= !0x04;
        }
    }

    pub fn calibrating(&self) -> bool {
        0x08 & self.0 != 0
    }
    pub fn set_calibrating(&mut self, b: bool) {
        if b {
            self.0 |= 0x08;
        } else {
            self.0 &= !0x08;
        }
    }

    pub fn calibration_failed(&self) -> bool {
        0x10 & self.0 != 0
    }
    pub fn set_calibration_failed(&mut self, b: bool) {
        if b {
            self.0 |= 0x10;
        } else {
            self.0 &= !0x10;
        }
    }
}

impl ControlPacket {
    pub fn new(throttle: u8, roll: f32, pitch: f32, yaw: f32, armed: bool) -> Self {
        let mut flags = Flags(0);
        flags.set_armed(armed);
        Self {
            throttle,
            roll,
            pitch,
            yaw,
            flags,
        }
    }

    pub fn to_bytes(&self) -> [u8; DEFAULT_SIZE] {
        let mut buf = [0u8; DEFAULT_SIZE];
        buf[0..4].copy_from_slice(&MAGIC);
        buf[4] = self.throttle;
        buf[5..9].copy_from_slice(&self.roll.to_be_bytes());
        buf[9..13].copy_from_slice(&self.pitch.to_be_bytes());
        buf[13..17].copy_from_slice(&self.yaw.to_be_bytes());
        buf[17] = Flags::to_bytes(&self.flags);
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < DEFAULT_SIZE {
            return None;
        }
        if buf[0..4] != MAGIC {
            return None;
        }
        Some(Self {
            throttle: buf[4],
            roll: f32::from_be_bytes(buf[5..9].try_into().ok()?),
            pitch: f32::from_be_bytes(buf[9..13].try_into().ok()?),
            yaw: f32::from_be_bytes(buf[13..17].try_into().ok()?),
            flags: Flags::from_bytes(buf[17]),
        })
    }

    pub fn flags(&self) -> Flags {
        self.flags
    }

    pub fn armed(&self) -> bool {
        self.flags().armed()
    }

    pub fn set_armed(&mut self, b: bool) {
        self.flags.set_armed(b);
        if self.armed() {
            self.throttle = 0;
        }
    }
}

#[cfg(feature = "telemetry")]
pub const TELEMETRY_MAGIC: [u8; 4] = *b"TELM";

// 4 (magic) + 4+4+4 (roll/pitch/yaw f32 be) + 2+2+2+2 (motor duties u16 be) + 1 (flags)
#[cfg(all(feature = "telemetry", not(feature = "telemetry-verbose")))]
pub const TELEMETRY_SIZE: usize = 25;
// base 25 + 4+4+4 (raw gyro rad/s f32 be, telemetry-verbose only)
#[cfg(feature = "telemetry-verbose")]
pub const TELEMETRY_SIZE: usize = 37;

/// telemetry packet sent from the ESP32 back to ground control over UDP, in reply to a
/// received control packet. serialized as big-endian; layout depends on the
/// `telemetry-verbose` feature (must match on both ends of the link).
#[cfg(feature = "telemetry")]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TelemetryPacket {
    /// roll, in degrees
    pub roll: f32,
    /// pitch, in degrees
    pub pitch: f32,
    /// yaw, in degrees
    pub yaw: f32,
    /// motor duty cycles, in raw PWM hardware ticks: (front-left, front-right, rear-left, rear-right)
    pub motors: (u16, u16, u16, u16),
    /// raw gyro rates in rad/s: (x, y, z)
    #[cfg(feature = "telemetry-verbose")]
    pub gyro: Vector3<f32>,
    /// armed + failsafe flags
    flags: Flags,
}

#[cfg(feature = "telemetry")]
impl TelemetryPacket {
    #[cfg(not(feature = "telemetry-verbose"))]
    pub fn new(
        roll: f32,
        pitch: f32,
        yaw: f32,
        motors: (u16, u16, u16, u16),
        armed: bool,
        failsafe: bool,
    ) -> Self {
        let mut flags = Flags(0);
        flags.set_armed(armed);
        flags.set_failsafe(failsafe);
        Self {
            roll,
            pitch,
            yaw,
            motors,
            flags,
        }
    }

    #[cfg(feature = "telemetry-verbose")]
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        roll: f32,
        pitch: f32,
        yaw: f32,
        motors: (u16, u16, u16, u16),
        armed: bool,
        failsafe: bool,
        gyro: Vector3<f32>,
    ) -> Self {
        let mut flags = Flags(0);
        flags.set_armed(armed);
        flags.set_failsafe(failsafe);
        Self {
            roll,
            pitch,
            yaw,
            motors,
            gyro,
            flags,
        }
    }

    /// marks the packet as having been produced right after a DMP FIFO overflow -
    /// set directly on the cached telemetry packet, doesn't require rebuilding one
    pub fn set_fifo_overflow(&mut self, b: bool) {
        self.flags.set_fifo_overflow(b);
    }

    /// marks the packet as produced while the IMU is being calibrated - roll/pitch/yaw/motors
    /// are stale placeholders while this is set, not a live reading
    pub fn set_calibrating(&mut self, b: bool) {
        self.flags.set_calibrating(b);
    }

    /// marks the packet as reporting a failed gyro calibration attempt (boot or on-arm).
    pub fn set_calibration_failed(&mut self, b: bool) {
        self.flags.set_calibration_failed(b);
    }

    pub fn to_bytes(&self) -> [u8; TELEMETRY_SIZE] {
        let mut buf = [0u8; TELEMETRY_SIZE];
        buf[0..4].copy_from_slice(&TELEMETRY_MAGIC);
        buf[4..8].copy_from_slice(&self.roll.to_be_bytes());
        buf[8..12].copy_from_slice(&self.pitch.to_be_bytes());
        buf[12..16].copy_from_slice(&self.yaw.to_be_bytes());
        buf[16..18].copy_from_slice(&self.motors.0.to_be_bytes());
        buf[18..20].copy_from_slice(&self.motors.1.to_be_bytes());
        buf[20..22].copy_from_slice(&self.motors.2.to_be_bytes());
        buf[22..24].copy_from_slice(&self.motors.3.to_be_bytes());
        buf[24] = Flags::to_bytes(&self.flags);
        #[cfg(feature = "telemetry-verbose")]
        {
            buf[25..29].copy_from_slice(&self.gyro.x.to_be_bytes());
            buf[29..33].copy_from_slice(&self.gyro.y.to_be_bytes());
            buf[33..37].copy_from_slice(&self.gyro.z.to_be_bytes());
        }
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < TELEMETRY_SIZE {
            return None;
        }
        if buf[0..4] != TELEMETRY_MAGIC {
            return None;
        }
        Some(Self {
            roll: f32::from_be_bytes(buf[4..8].try_into().ok()?),
            pitch: f32::from_be_bytes(buf[8..12].try_into().ok()?),
            yaw: f32::from_be_bytes(buf[12..16].try_into().ok()?),
            motors: (
                u16::from_be_bytes(buf[16..18].try_into().ok()?),
                u16::from_be_bytes(buf[18..20].try_into().ok()?),
                u16::from_be_bytes(buf[20..22].try_into().ok()?),
                u16::from_be_bytes(buf[22..24].try_into().ok()?),
            ),
            flags: Flags::from_bytes(buf[24]),
            #[cfg(feature = "telemetry-verbose")]
            gyro: Vector3::new(
                f32::from_be_bytes(buf[25..29].try_into().ok()?),
                f32::from_be_bytes(buf[29..33].try_into().ok()?),
                f32::from_be_bytes(buf[33..37].try_into().ok()?),
            ),
        })
    }

    pub fn armed(&self) -> bool {
        self.flags.armed()
    }

    pub fn failsafe(&self) -> bool {
        self.flags.failsafe()
    }

    pub fn fifo_overflow(&self) -> bool {
        self.flags.fifo_overflow()
    }

    pub fn calibrating(&self) -> bool {
        self.flags.calibrating()
    }

    pub fn calibration_failed(&self) -> bool {
        self.flags.calibration_failed()
    }
}

#[cfg(test)]
mod tests {

    use super::*;
    #[test]
    fn test_flags() {
        let mut flags = Flags(0);
        assert!(!flags.armed());
        flags.set_armed(true);
        assert_eq!(flags.to_bytes(), 0x01);
        assert!(flags.armed());
        flags.set_armed(false);
        assert!(!flags.armed());
        assert_eq!(flags.to_bytes(), 0);

        // start with another bit set alongside armed
        let mut flags = Flags::from_bytes(0b0000_0110);
        flags.set_armed(true);
        assert_eq!(flags.to_bytes(), 0b0000_0111);
        assert!(flags.armed());
        flags.set_armed(false);
        assert!(!flags.armed());
        assert_eq!(flags.to_bytes(), 0b0000_0110);
    }

    #[test]
    fn test_flags_failsafe() {
        let mut flags = Flags(0);
        assert!(!flags.failsafe());
        flags.set_failsafe(true);
        assert!(flags.failsafe());
        assert_eq!(flags.to_bytes(), 0b0000_0010);
        flags.set_failsafe(false);
        assert!(!flags.failsafe());
        assert_eq!(flags.to_bytes(), 0);
    }

    #[test]
    fn test_flags_armed_and_failsafe_are_independent_bits() {
        let mut flags = Flags(0);
        flags.set_armed(true);
        flags.set_failsafe(true);
        assert!(flags.armed());
        assert!(flags.failsafe());
        assert_eq!(flags.to_bytes(), 0b0000_0011);

        flags.set_armed(false);
        assert!(!flags.armed());
        assert!(flags.failsafe()); // unaffected by clearing armed
        assert_eq!(flags.to_bytes(), 0b0000_0010);
    }

    #[test]
    fn test_control_packet_roundtrip() {
        let pkt = ControlPacket::new(42, 0.5, -0.25, 1.0, true);
        let bytes = pkt.to_bytes();
        assert_eq!(ControlPacket::from_bytes(&bytes), Some(pkt));
    }

    #[test]
    fn test_control_packet_rejects_bad_magic() {
        let mut bytes = ControlPacket::new(10, 0.0, 0.0, 0.0, false).to_bytes();
        bytes[0] = b'X';
        assert_eq!(ControlPacket::from_bytes(&bytes), None);
    }

    #[test]
    fn test_control_packet_rejects_short_buffer() {
        let bytes = ControlPacket::new(10, 0.0, 0.0, 0.0, false).to_bytes();
        assert_eq!(ControlPacket::from_bytes(&bytes[..DEFAULT_SIZE - 1]), None);
    }

    #[test]
    fn test_control_packet_set_armed_zeros_throttle() {
        // arming always resets throttle to zero, as a safety net
        let mut pkt = ControlPacket::new(80, 0.0, 0.0, 0.0, false);
        pkt.set_armed(true);
        assert!(pkt.armed());
        assert_eq!(pkt.throttle, 0);

        // disarming does not touch throttle
        pkt.throttle = 55;
        pkt.set_armed(false);
        assert!(!pkt.armed());
        assert_eq!(pkt.throttle, 55);
    }

    #[cfg(all(feature = "telemetry", not(feature = "telemetry-verbose")))]
    #[test]
    fn test_telemetry_packet_roundtrip() {
        let pkt = TelemetryPacket::new(12.5, -3.25, 180.0, (100, 200, 300, 400), true, false);
        let bytes = pkt.to_bytes();
        assert_eq!(TelemetryPacket::from_bytes(&bytes), Some(pkt));
    }

    #[cfg(feature = "telemetry-verbose")]
    #[test]
    fn test_telemetry_packet_roundtrip_verbose() {
        let pkt = TelemetryPacket::new(
            12.5,
            -3.25,
            180.0,
            (100, 200, 300, 400),
            true,
            false,
            Vector3::new(0.1, -0.2, 0.3),
        );
        let bytes = pkt.to_bytes();
        assert_eq!(TelemetryPacket::from_bytes(&bytes), Some(pkt));
    }

    #[cfg(feature = "telemetry")]
    #[test]
    fn test_telemetry_packet_rejects_bad_magic() {
        #[cfg(not(feature = "telemetry-verbose"))]
        let pkt = TelemetryPacket::new(0.0, 0.0, 0.0, (0, 0, 0, 0), false, false);
        #[cfg(feature = "telemetry-verbose")]
        let pkt = TelemetryPacket::new(
            0.0,
            0.0,
            0.0,
            (0, 0, 0, 0),
            false,
            false,
            Vector3::new(0.0, 0.0, 0.0),
        );

        let mut bytes = pkt.to_bytes();
        bytes[0] = b'X';
        assert_eq!(TelemetryPacket::from_bytes(&bytes), None);
    }

    #[cfg(feature = "telemetry")]
    #[test]
    fn test_telemetry_packet_rejects_short_buffer() {
        #[cfg(not(feature = "telemetry-verbose"))]
        let pkt = TelemetryPacket::new(0.0, 0.0, 0.0, (0, 0, 0, 0), false, false);
        #[cfg(feature = "telemetry-verbose")]
        let pkt = TelemetryPacket::new(
            0.0,
            0.0,
            0.0,
            (0, 0, 0, 0),
            false,
            false,
            Vector3::new(0.0, 0.0, 0.0),
        );

        let bytes = pkt.to_bytes();
        assert_eq!(
            TelemetryPacket::from_bytes(&bytes[..TELEMETRY_SIZE - 1]),
            None
        );
    }

    #[cfg(feature = "telemetry")]
    #[test]
    fn test_telemetry_packet_armed_and_failsafe() {
        #[cfg(not(feature = "telemetry-verbose"))]
        let pkt = TelemetryPacket::new(0.0, 0.0, 0.0, (0, 0, 0, 0), true, true);
        #[cfg(feature = "telemetry-verbose")]
        let pkt = TelemetryPacket::new(
            0.0,
            0.0,
            0.0,
            (0, 0, 0, 0),
            true,
            true,
            Vector3::new(0.0, 0.0, 0.0),
        );

        assert!(pkt.armed());
        assert!(pkt.failsafe());
        assert!(!pkt.fifo_overflow());
    }

    #[cfg(feature = "telemetry")]
    #[test]
    fn test_telemetry_packet_set_fifo_overflow() {
        #[cfg(not(feature = "telemetry-verbose"))]
        let mut pkt = TelemetryPacket::new(0.0, 0.0, 0.0, (0, 0, 0, 0), true, false);
        #[cfg(feature = "telemetry-verbose")]
        let mut pkt = TelemetryPacket::new(
            0.0,
            0.0,
            0.0,
            (0, 0, 0, 0),
            true,
            false,
            Vector3::new(0.0, 0.0, 0.0),
        );

        assert!(!pkt.fifo_overflow());
        pkt.set_fifo_overflow(true);
        assert!(pkt.fifo_overflow());
        // doesn't disturb the other flags
        assert!(pkt.armed());
        assert!(!pkt.failsafe());

        let bytes = pkt.to_bytes();
        assert_eq!(TelemetryPacket::from_bytes(&bytes), Some(pkt));
    }

    #[cfg(feature = "telemetry")]
    #[test]
    fn test_telemetry_packet_set_calibrating() {
        #[cfg(not(feature = "telemetry-verbose"))]
        let mut pkt = TelemetryPacket::new(0.0, 0.0, 0.0, (0, 0, 0, 0), true, false);
        #[cfg(feature = "telemetry-verbose")]
        let mut pkt = TelemetryPacket::new(
            0.0,
            0.0,
            0.0,
            (0, 0, 0, 0),
            true,
            false,
            Vector3::new(0.0, 0.0, 0.0),
        );

        assert!(!pkt.calibrating());
        pkt.set_calibrating(true);
        assert!(pkt.calibrating());
        // doesn't disturb the other flags
        assert!(pkt.armed());
        assert!(!pkt.failsafe());

        let bytes = pkt.to_bytes();
        assert_eq!(TelemetryPacket::from_bytes(&bytes), Some(pkt));
    }

    #[cfg(feature = "telemetry")]
    #[test]
    fn test_telemetry_packet_set_calibration_failed() {
        #[cfg(not(feature = "telemetry-verbose"))]
        let mut pkt = TelemetryPacket::new(0.0, 0.0, 0.0, (0, 0, 0, 0), true, false);
        #[cfg(feature = "telemetry-verbose")]
        let mut pkt = TelemetryPacket::new(
            0.0,
            0.0,
            0.0,
            (0, 0, 0, 0),
            true,
            false,
            Vector3::new(0.0, 0.0, 0.0),
        );

        assert!(!pkt.calibration_failed());
        pkt.set_calibration_failed(true);
        assert!(pkt.calibration_failed());
        // doesn't disturb the other flags
        assert!(pkt.armed());
        assert!(!pkt.failsafe());
        assert!(!pkt.calibrating());

        let bytes = pkt.to_bytes();
        assert_eq!(TelemetryPacket::from_bytes(&bytes), Some(pkt));
    }
}
