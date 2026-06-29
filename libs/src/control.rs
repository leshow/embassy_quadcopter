//! control packet shared between the gamepad binary and ESP32 firmware
pub const DEFAULT_SIZE: usize = 17;

/// control packet sent from the gamepad PC to the ESP32 over UDP.
/// Serialized as big-endian: 4× f32 + 1× u8 = 17 bytes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlPacket<const N: usize = DEFAULT_SIZE> {
    /// Throttle: 0.0 (min) to 1.0 (max)
    pub throttle: f32,
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

impl<const N: usize> ControlPacket<N> {
    pub const SIZE: usize = N;

    pub fn to_bytes(&self) -> [u8; N] {
        let mut buf = [0u8; N];
        buf[0..4].copy_from_slice(&self.throttle.to_be_bytes());
        buf[4..8].copy_from_slice(&self.roll.to_be_bytes());
        buf[8..12].copy_from_slice(&self.pitch.to_be_bytes());
        buf[12..16].copy_from_slice(&self.yaw.to_be_bytes());
        buf[16] = Flags::to_bytes(&self.flags);
        buf
    }

    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < Self::SIZE {
            return None;
        }
        Some(Self {
            throttle: f32::from_be_bytes(buf[0..4].try_into().ok()?),
            roll: f32::from_be_bytes(buf[4..8].try_into().ok()?),
            pitch: f32::from_be_bytes(buf[8..12].try_into().ok()?),
            yaw: f32::from_be_bytes(buf[12..16].try_into().ok()?),
            flags: Flags::from_bytes(buf[16]),
        })
    }

    pub fn flags(&self) -> Flags {
        self.flags
    }

    pub fn armed(&self) -> bool {
        self.flags().armed()
    }

    pub fn set_armed(&self, b: bool) {
        self.flags().set_armed(b);
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
