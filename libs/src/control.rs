//! control packet shared between the ground control binary and ESP32 firmware
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

#[cfg(test)]
mod tests {

    use super::*;
    #[test]
    fn test_flags() {
        let mut flags = Flags(0);
        assert_eq!(flags.armed(), false);
        flags.set_armed(true);
        assert_eq!(flags.to_bytes(), 0x01);
        assert_eq!(flags.armed(), true);
        flags.set_armed(false);
        assert_eq!(flags.armed(), false);
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
}
