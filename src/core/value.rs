//! Access widths and endianness.
//!
//! Small, but load-bearing: a big-endian device on a little-endian bus is
//! normal rather than exotic (`ROADMAP.md` §4.1), so byte order is carried
//! explicitly at every boundary instead of being assumed from the host.

use crate::core::error::BusError;

/// The width of a single guest access.
///
/// A real enum rather than a raw byte count: only these widths exist, and
/// exhaustive matching in the dispatch path is worth more than the ability to
/// invent a 3-byte access.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Width {
    /// 8 bits.
    U8,
    /// 16 bits.
    U16,
    /// 32 bits.
    U32,
    /// 64 bits.
    U64,
}

impl Width {
    /// Size of this access in bytes.
    #[inline]
    pub const fn bytes(self) -> u64 {
        match self {
            Width::U8 => 1,
            Width::U16 => 2,
            Width::U32 => 4,
            Width::U64 => 8,
        }
    }

    /// Size of this access in bits.
    #[inline]
    pub const fn bits(self) -> u32 {
        match self {
            Width::U8 => 8,
            Width::U16 => 16,
            Width::U32 => 32,
            Width::U64 => 64,
        }
    }

    /// The width for a byte count, or `None` if no access is that wide.
    #[inline]
    pub const fn from_bytes(n: u64) -> Option<Self> {
        match n {
            1 => Some(Width::U8),
            2 => Some(Width::U16),
            4 => Some(Width::U32),
            8 => Some(Width::U64),
            _ => None,
        }
    }

    /// A mask with this width's low bits set.
    #[inline]
    pub const fn mask(self) -> u64 {
        match self {
            Width::U8 => 0xff,
            Width::U16 => 0xffff,
            Width::U32 => 0xffff_ffff,
            Width::U64 => u64::MAX,
        }
    }

    /// Whether `addr` is naturally aligned for this width.
    #[inline]
    pub const fn is_aligned(self, addr: u64) -> bool {
        addr.is_multiple_of(self.bytes())
    }
}

/// Byte order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Endian {
    /// Least-significant byte first.
    Little,
    /// Most-significant byte first.
    Big,
}

impl Endian {
    /// The host's byte order.
    ///
    /// Used only to decide whether a conversion is a no-op. Guest byte order is
    /// always carried explicitly and never inferred from this.
    pub const HOST: Endian = if cfg!(target_endian = "little") {
        Endian::Little
    } else {
        Endian::Big
    };

    /// Read `width` bytes from the front of `bytes` in this byte order.
    ///
    /// Returns [`BusError::BadAccess`] when the slice is too short, rather than
    /// panicking: a short buffer is a caller bug, but in the dispatch path a
    /// bus fault is more useful than an abort.
    pub fn load(self, bytes: &[u8], width: Width) -> Result<u64, BusError> {
        let n = width.bytes() as usize;
        let src = bytes.get(..n).ok_or(BusError::BadAccess)?;
        let mut v: u64 = 0;
        match self {
            Endian::Little => {
                for (i, b) in src.iter().enumerate() {
                    v |= (*b as u64) << (8 * i);
                }
            }
            Endian::Big => {
                for b in src {
                    v = (v << 8) | (*b as u64);
                }
            }
        }
        Ok(v)
    }

    /// Write the low `width` bytes of `value` into `bytes` in this byte order.
    pub fn store(self, bytes: &mut [u8], width: Width, value: u64) -> Result<(), BusError> {
        let n = width.bytes() as usize;
        let dst = bytes.get_mut(..n).ok_or(BusError::BadAccess)?;
        match self {
            Endian::Little => {
                for (i, b) in dst.iter_mut().enumerate() {
                    *b = (value >> (8 * i)) as u8;
                }
            }
            Endian::Big => {
                for (i, b) in dst.iter_mut().enumerate() {
                    *b = (value >> (8 * (n - 1 - i))) as u8;
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn width_sizes_and_masks_agree() {
        for w in [Width::U8, Width::U16, Width::U32, Width::U64] {
            assert_eq!(w.bits(), (w.bytes() * 8) as u32);
            assert_eq!(Width::from_bytes(w.bytes()), Some(w));
        }
        assert_eq!(Width::from_bytes(3), None);
        assert_eq!(Width::U8.mask(), 0xff);
        assert_eq!(Width::U64.mask(), u64::MAX);
    }

    #[test]
    fn alignment() {
        assert!(Width::U32.is_aligned(0x1000));
        assert!(!Width::U32.is_aligned(0x1002));
        // Every address is aligned for a byte access.
        assert!(Width::U8.is_aligned(0x1001));
    }

    #[test]
    fn load_respects_byte_order() {
        let bytes = [0x78, 0x56, 0x34, 0x12];
        assert_eq!(
            Endian::Little.load(&bytes, Width::U32).unwrap(),
            0x1234_5678
        );
        assert_eq!(Endian::Big.load(&bytes, Width::U32).unwrap(), 0x7856_3412);
        assert_eq!(Endian::Little.load(&bytes, Width::U16).unwrap(), 0x5678);
    }

    #[test]
    fn store_round_trips_through_load() {
        for endian in [Endian::Little, Endian::Big] {
            for (width, value) in [
                (Width::U8, 0xa5u64),
                (Width::U16, 0xbeef),
                (Width::U32, 0xdead_beef),
                (Width::U64, 0x0123_4567_89ab_cdef),
            ] {
                let mut buf = [0u8; 8];
                endian.store(&mut buf, width, value).unwrap();
                assert_eq!(
                    endian.load(&buf, width).unwrap(),
                    value,
                    "{endian:?} {width:?}"
                );
            }
        }
    }

    #[test]
    fn a_short_buffer_is_a_bus_fault_not_a_panic() {
        let bytes = [0u8; 2];
        assert_eq!(
            Endian::Little.load(&bytes, Width::U32),
            Err(BusError::BadAccess)
        );
        let mut small = [0u8; 1];
        assert_eq!(
            Endian::Big.store(&mut small, Width::U64, 0),
            Err(BusError::BadAccess)
        );
    }
}
