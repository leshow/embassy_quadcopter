//! RC control packet shared between the gamepad binary and ESP32 firmware

/// RC control packet sent from the gamepad PC to the ESP32 over UDP.
/// Serialized as 17 bytes, little-endian floats.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlPacket {
    /// Throttle: 0.0 (min) to 1.0 (max)
    pub throttle: f32,
    /// Roll: -1.0 (left) to 1.0 (right)
    pub roll: f32,
    /// Pitch: -1.0 (forward) to 1.0 (backward)
    pub pitch: f32,
    /// Yaw: -1.0 (left) to 1.0 (right)
    pub yaw: f32,
    /// Armed flag: 0 = disarmed, 1 = armed
    pub armed: u8,
}

impl ControlPacket {
    pub const SIZE: usize = 17;

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut buf = [0u8; Self::SIZE];
        buf[0..4].copy_from_slice(&self.throttle.to_le_bytes());
        buf[4..8].copy_from_slice(&self.roll.to_le_bytes());
        buf[8..12].copy_from_slice(&self.pitch.to_le_bytes());
        buf[12..16].copy_from_slice(&self.yaw.to_le_bytes());
        buf[16] = self.armed;
        buf
    }

    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            throttle: f32::from_le_bytes(bytes[0..4].try_into().ok()?),
            roll: f32::from_le_bytes(bytes[4..8].try_into().ok()?),
            pitch: f32::from_le_bytes(bytes[8..12].try_into().ok()?),
            yaw: f32::from_le_bytes(bytes[12..16].try_into().ok()?),
            armed: bytes[16],
        })
    }
}
