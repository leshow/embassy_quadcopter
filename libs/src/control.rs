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
    pub fn new(f: u8) -> Self {
        Self(f)
    }
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
}
