//! The ARMv6 media arithmetic, as pure functions on register values.
//!
//! Nothing here touches processor state: each function takes the operands
//! and returns the result together with whatever flag information the
//! instruction reports (`GE` bits, "saturated" for `Q`). The interpreter
//! commits those; the tests at the bottom of this file check the arithmetic
//! without building a core. Keeping them apart is what makes the boundary
//! cases — a lane at exactly `0x8000`, a saturation to one bit — cheap to
//! pin down exhaustively.
//!
//! # Sources
//!
//! *ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition* (ARM DDI
//! 0406C): A2.2.1 (`SignedSatQ`, `UnsignedSatQ`), A2.4 (the `GE` bits and
//! who sets them), and the per-instruction pseudocode in A8.8 for the
//! parallel add/subtract family (`SADD16` through `UHSUB8`), `SEL`, `SSAT`,
//! `USAT`, `SSAT16`, `USAT16`, `PKHBT`/`PKHTB`, the extends, `REV`,
//! `REV16`, `REVSH`, `RBIT`, `USAD8` and the dual multiplies. No emulator
//! source of any licence was consulted (`ROADMAP.md` §1).

use core::fmt;

/// How a parallel add/subtract treats its lanes (DDI 0406C A5.4.1, A5.4.2).
///
/// Only the two *modular* forms set `GE`; the saturating and halving ones
/// leave it alone, because a result that cannot overflow has nothing to
/// report and `SEL` has nothing to select on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ParKind {
    /// Signed, modular, sets `GE`.
    S,
    /// Signed, saturating.
    Q,
    /// Signed, halving.
    Sh,
    /// Unsigned, modular, sets `GE`.
    U,
    /// Unsigned, saturating.
    Uq,
    /// Unsigned, halving.
    Uh,
}

impl ParKind {
    /// The mnemonic prefix.
    #[must_use]
    pub const fn prefix(self) -> &'static str {
        match self {
            ParKind::S => "S",
            ParKind::Q => "Q",
            ParKind::Sh => "SH",
            ParKind::U => "U",
            ParKind::Uq => "UQ",
            ParKind::Uh => "UH",
        }
    }

    /// Whether the lanes are unsigned.
    #[must_use]
    pub const fn unsigned(self) -> bool {
        matches!(self, ParKind::U | ParKind::Uq | ParKind::Uh)
    }

    /// Whether the instruction writes `GE`.
    #[must_use]
    pub const fn sets_ge(self) -> bool {
        matches!(self, ParKind::S | ParKind::U)
    }
}

/// Which lanes a parallel add/subtract works on, and in which direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ParShape {
    /// Two halfword additions.
    Add16,
    /// Add in the high halfword, subtract in the low one, with `Rm`'s
    /// halves exchanged.
    Asx,
    /// Subtract in the high halfword, add in the low one, exchanged.
    Sax,
    /// Two halfword subtractions.
    Sub16,
    /// Four byte additions.
    Add8,
    /// Four byte subtractions.
    Sub8,
}

impl ParShape {
    /// The mnemonic suffix.
    #[must_use]
    pub const fn suffix(self) -> &'static str {
        match self {
            ParShape::Add16 => "ADD16",
            ParShape::Asx => "ASX",
            ParShape::Sax => "SAX",
            ParShape::Sub16 => "SUB16",
            ParShape::Add8 => "ADD8",
            ParShape::Sub8 => "SUB8",
        }
    }
}

/// One parallel add/subtract, fully named.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ParOp {
    /// Signedness and overflow treatment.
    pub kind: ParKind,
    /// Lanes and directions.
    pub shape: ParShape,
}

impl fmt::Display for ParOp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}{}", self.kind.prefix(), self.shape.suffix())
    }
}

/// One lane of `value`, `width` bits wide, as a signed or unsigned integer.
const fn lane(value: u32, index: u32, width: u32, unsigned: bool) -> i32 {
    let raw = (value >> (index * width)) & ((1 << width) - 1);
    if unsigned {
        raw as i32
    } else {
        // Sign-extend by shifting the lane's top bit into bit 31 and back.
        ((raw << (32 - width)) as i32) >> (32 - width)
    }
}

/// One lane's arithmetic, returning the lane result and its `GE` bit.
///
/// `GE` for a signed modular lane is "the true result is non-negative"; for
/// an unsigned addition it is the carry out, and for an unsigned
/// subtraction "no borrow" — A8.8's `SADD16`/`UADD16`/`USUB16` pseudocode.
fn lane_op(kind: ParKind, sub: bool, a: i32, b: i32, width: u32) -> (u32, bool) {
    let exact = if sub { a - b } else { a + b };
    let mask = (1u32 << width) - 1;
    let value = match kind {
        ParKind::S | ParKind::U => exact as u32 & mask,
        ParKind::Q => signed_sat(i64::from(exact), width).0 as u32 & mask,
        ParKind::Uq => unsigned_sat(i64::from(exact), width).0 & mask,
        // Halving: the exact sum or difference fits in `width + 1` bits, so
        // an arithmetic shift of the 32-bit value loses nothing.
        ParKind::Sh | ParKind::Uh => (exact >> 1) as u32 & mask,
    };
    let ge = match kind {
        ParKind::U if !sub => exact >= (1 << width),
        _ => exact >= 0,
    };
    (value, ge)
}

/// A parallel add or subtract: the result and, for the modular forms, the
/// four `GE` bits (bit 0 for the lowest byte).
#[must_use]
pub fn parallel(op: ParOp, a: u32, b: u32) -> (u32, Option<u8>) {
    let unsigned = op.kind.unsigned();
    let (result, ge) = match op.shape {
        ParShape::Add8 | ParShape::Sub8 => {
            let sub = op.shape == ParShape::Sub8;
            let mut result = 0;
            let mut ge = 0u8;
            for i in 0..4 {
                let (v, g) = lane_op(
                    op.kind,
                    sub,
                    lane(a, i, 8, unsigned),
                    lane(b, i, 8, unsigned),
                    8,
                );
                result |= v << (8 * i);
                ge |= u8::from(g) << i;
            }
            (result, ge)
        }
        shape => {
            // ASX and SAX pair `Rn`'s low half with `Rm`'s high half and the
            // other way round; the plain forms keep lanes aligned.
            let exchange = matches!(shape, ParShape::Asx | ParShape::Sax);
            let (b_lo, b_hi) = if exchange {
                (lane(b, 1, 16, unsigned), lane(b, 0, 16, unsigned))
            } else {
                (lane(b, 0, 16, unsigned), lane(b, 1, 16, unsigned))
            };
            let (sub_lo, sub_hi) = match shape {
                ParShape::Add16 => (false, false),
                ParShape::Sub16 => (true, true),
                ParShape::Asx => (true, false),
                _ => (false, true),
            };
            let (lo, g_lo) = lane_op(op.kind, sub_lo, lane(a, 0, 16, unsigned), b_lo, 16);
            let (hi, g_hi) = lane_op(op.kind, sub_hi, lane(a, 1, 16, unsigned), b_hi, 16);
            // A halfword lane owns two `GE` bits, so that `SEL` selects
            // halfwords with the same byte-wise rule.
            (
                lo | (hi << 16),
                (u8::from(g_lo) * 0b0011) | (u8::from(g_hi) * 0b1100),
            )
        }
    };
    (result, op.kind.sets_ge().then_some(ge))
}

/// `SEL`: each byte from `a` where its `GE` bit is set, else from `b`.
#[must_use]
pub const fn select(ge: u8, a: u32, b: u32) -> u32 {
    let mut mask = 0u32;
    let mut i = 0;
    while i < 4 {
        if ge & (1 << i) != 0 {
            mask |= 0xff << (8 * i);
        }
        i += 1;
    }
    (a & mask) | (b & !mask)
}

/// `SignedSatQ(value, bits)`: clamp to a `bits`-bit two's complement range,
/// reporting whether it had to (DDI 0406C A2.2.1). `bits` is `1..=32`.
#[must_use]
pub fn signed_sat(value: i64, bits: u32) -> (i32, bool) {
    let max = (1i64 << (bits - 1)) - 1;
    let min = -(1i64 << (bits - 1));
    if value > max {
        (max as i32, true)
    } else if value < min {
        (min as i32, true)
    } else {
        (value as i32, false)
    }
}

/// `UnsignedSatQ(value, bits)`: clamp to `0..2^bits`. `bits` is `0..=31`.
#[must_use]
pub fn unsigned_sat(value: i64, bits: u32) -> (u32, bool) {
    let max = (1i64 << bits) - 1;
    if value > max {
        (max as u32, true)
    } else if value < 0 {
        (0, true)
    } else {
        (value as u32, false)
    }
}

/// `SSAT16`/`USAT16`: saturate each signed halfword of `value`. Returns the
/// packed result and whether either lane saturated.
#[must_use]
pub fn saturate16(value: u32, bits: u32, unsigned: bool) -> (u32, bool) {
    let mut result = 0;
    let mut saturated = false;
    for i in 0..2 {
        let half = i64::from(lane(value, i, 16, false));
        let (v, q) = if unsigned {
            unsigned_sat(half, bits)
        } else {
            let (v, q) = signed_sat(half, bits);
            (v as u32, q)
        };
        result |= (v & 0xffff) << (16 * i);
        saturated |= q;
    }
    (result, saturated)
}

/// The width an extend instruction takes from its rotated operand.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ExtendSize {
    /// One byte.
    Byte,
    /// One halfword.
    Half,
    /// Bytes 0 and 2, each into a halfword lane (`XTB16`).
    Byte16,
}

impl ExtendSize {
    /// The mnemonic suffix after `SXT`/`UXT` (and after `SXTA`/`UXTA`).
    #[must_use]
    pub const fn suffix(self) -> &'static str {
        match self {
            ExtendSize::Byte => "B",
            ExtendSize::Half => "H",
            ExtendSize::Byte16 => "B16",
        }
    }
}

/// `SXT*`/`UXT*` and the accumulating `SXTA*`/`UXTA*`: extend the rotated
/// `rm` and add `rn` (zero for the non-accumulating forms).
#[must_use]
pub const fn extend(size: ExtendSize, signed: bool, rn: u32, rm: u32, rotate: u32) -> u32 {
    let v = rm.rotate_right(rotate);
    match size {
        ExtendSize::Byte => {
            let x = if signed {
                v as u8 as i8 as i32 as u32
            } else {
                v & 0xff
            };
            rn.wrapping_add(x)
        }
        ExtendSize::Half => {
            let x = if signed {
                v as u16 as i16 as i32 as u32
            } else {
                v & 0xffff
            };
            rn.wrapping_add(x)
        }
        ExtendSize::Byte16 => {
            // Each lane adds independently and wraps within sixteen bits.
            let (b0, b2) = if signed {
                (
                    v as u8 as i8 as i16 as u16,
                    (v >> 16) as u8 as i8 as i16 as u16,
                )
            } else {
                ((v & 0xff) as u16, ((v >> 16) & 0xff) as u16)
            };
            let lo = (rn as u16).wrapping_add(b0);
            let hi = ((rn >> 16) as u16).wrapping_add(b2);
            (lo as u32) | ((hi as u32) << 16)
        }
    }
}

/// The four byte/bit reversals.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum RevOp {
    /// Reverse the four bytes of a word.
    Rev,
    /// Reverse the bytes within each halfword.
    Rev16,
    /// Reverse the low halfword's bytes and sign-extend.
    Revsh,
    /// Reverse all thirty-two bits (v6T2).
    Rbit,
}

impl RevOp {
    /// The assembler mnemonic.
    #[must_use]
    pub const fn mnemonic(self) -> &'static str {
        match self {
            RevOp::Rev => "REV",
            RevOp::Rev16 => "REV16",
            RevOp::Revsh => "REVSH",
            RevOp::Rbit => "RBIT",
        }
    }

    /// Apply it.
    #[must_use]
    pub const fn apply(self, v: u32) -> u32 {
        match self {
            RevOp::Rev => v.swap_bytes(),
            RevOp::Rev16 => ((v & 0x00ff_00ff) << 8) | ((v & 0xff00_ff00) >> 8),
            RevOp::Revsh => (v as u16).swap_bytes() as i16 as i32 as u32,
            RevOp::Rbit => v.reverse_bits(),
        }
    }
}

/// `PKHBT` (`tb == false`): bottom half of `rn`, top half of `rm LSL n`.
/// `PKHTB`: top half of `rn`, bottom half of `rm ASR n`, where an encoded
/// shift of zero means thirty-two (DDI 0406C A8.8, `PKHBT`/`PKHTB`).
#[must_use]
pub const fn pack(tb: bool, rn: u32, rm: u32, amount: u32) -> u32 {
    if tb {
        let shifted = if amount == 0 {
            ((rm as i32) >> 31) as u32
        } else {
            ((rm as i32) >> amount) as u32
        };
        (rn & 0xffff_0000) | (shifted & 0xffff)
    } else {
        (rn & 0xffff) | ((rm << amount) & 0xffff_0000)
    }
}

/// `USAD8`: the sum of the four absolute byte differences.
#[must_use]
pub const fn usad8(a: u32, b: u32) -> u32 {
    let mut sum = 0u32;
    let mut i = 0;
    while i < 4 {
        let x = (a >> (8 * i)) & 0xff;
        let y = (b >> (8 * i)) & 0xff;
        sum += x.abs_diff(y);
        i += 1;
    }
    sum
}

/// The two 16×16 products the dual multiplies form, with `m`'s halves
/// swapped first when `exchange` (the `X` suffix).
#[must_use]
pub const fn dual_products(n: u32, m: u32, exchange: bool) -> (i32, i32) {
    let m = if exchange { m.rotate_right(16) } else { m };
    let lo = (n as u16 as i16 as i32) * (m as u16 as i16 as i32);
    let hi = ((n >> 16) as u16 as i16 as i32) * ((m >> 16) as u16 as i16 as i32);
    (lo, hi)
}

#[cfg(test)]
mod tests {
    use super::*;

    const fn op(kind: ParKind, shape: ParShape) -> ParOp {
        ParOp { kind, shape }
    }

    #[test]
    fn signed_halfword_add_sets_ge_on_non_negative_lanes() {
        // 0x7fff + 1 wraps to 0x8000 but the true result is positive; -2 + 1
        // is negative.
        let (r, ge) = parallel(op(ParKind::S, ParShape::Add16), 0xfffe_7fff, 0x0001_0001);
        assert_eq!(r, 0xffff_8000);
        assert_eq!(ge, Some(0b0011));
    }

    #[test]
    fn unsigned_byte_add_reports_carries() {
        let (r, ge) = parallel(op(ParKind::U, ParShape::Add8), 0xff80_0102, 0x0180_0101);
        assert_eq!(r, 0x0000_0203);
        assert_eq!(ge, Some(0b1100));
    }

    #[test]
    fn unsigned_subtract_ge_means_no_borrow() {
        let (r, ge) = parallel(op(ParKind::U, ParShape::Sub8), 0x0102_0304, 0x0201_0304);
        assert_eq!(r, 0xff01_0000);
        assert_eq!(ge, Some(0b0111));
    }

    #[test]
    fn exchanging_forms_cross_the_halves() {
        // SASX: hi = a.hi + b.lo, lo = a.lo - b.hi.
        let (r, ge) = parallel(op(ParKind::S, ParShape::Asx), 0x0010_0020, 0x0005_0003);
        assert_eq!(r, 0x0013_001b);
        assert_eq!(ge, Some(0b1111));
        // SSAX: hi = a.hi - b.lo, lo = a.lo + b.hi.
        let (r, _) = parallel(op(ParKind::S, ParShape::Sax), 0x0010_0020, 0x0005_0003);
        assert_eq!(r, 0x000d_0025);
    }

    #[test]
    fn saturating_and_halving_forms_leave_ge_alone() {
        let (r, ge) = parallel(op(ParKind::Q, ParShape::Add16), 0x7fff_8000, 0x0001_ffff);
        assert_eq!(r, 0x7fff_8000);
        assert_eq!(ge, None);
        let (r, _) = parallel(op(ParKind::Uq, ParShape::Sub8), 0x0510_ff00, 0x0620_0001);
        assert_eq!(r, 0x0000_ff00);
        let (r, _) = parallel(op(ParKind::Sh, ParShape::Add8), 0x8080_0301, 0x8001_0101);
        assert_eq!(r, 0x80c0_0201);
        let (r, _) = parallel(op(ParKind::Uh, ParShape::Add16), 0xffff_0001, 0xffff_0002);
        assert_eq!(r, 0xffff_0001);
    }

    #[test]
    fn select_picks_bytes_by_ge() {
        assert_eq!(select(0b0101, 0xaabb_ccdd, 0x1122_3344), 0x11bb_33dd);
    }

    #[test]
    fn saturation_boundaries() {
        assert_eq!(signed_sat(127, 8), (127, false));
        assert_eq!(signed_sat(128, 8), (127, true));
        assert_eq!(signed_sat(-129, 8), (-128, true));
        assert_eq!(signed_sat(-1, 1), (-1, false));
        assert_eq!(signed_sat(1, 1), (0, true));
        assert_eq!(signed_sat(i64::from(i32::MIN), 32), (i32::MIN, false));
        assert_eq!(unsigned_sat(-1, 8), (0, true));
        assert_eq!(unsigned_sat(256, 8), (255, true));
        assert_eq!(unsigned_sat(5, 0), (0, true));
        assert_eq!(saturate16(0x8000_7fff, 8, false), (0xff80_007f, true));
        assert_eq!(saturate16(0xffff_0100, 8, true), (0x0000_00ff, true));
    }

    #[test]
    fn extends_rotate_then_extend_then_add() {
        assert_eq!(
            extend(ExtendSize::Byte, true, 0, 0x0000_8000, 8),
            0xffff_ff80
        );
        assert_eq!(
            extend(ExtendSize::Half, false, 1, 0xffff_0000, 16),
            0x1_0000
        );
        assert_eq!(
            extend(ExtendSize::Byte16, true, 0x0001_0001, 0x0080_00ff, 0),
            0xff81_0000
        );
    }

    #[test]
    fn reversals() {
        assert_eq!(RevOp::Rev.apply(0x1234_5678), 0x7856_3412);
        assert_eq!(RevOp::Rev16.apply(0x1234_5678), 0x3412_7856);
        assert_eq!(RevOp::Revsh.apply(0x0000_0080), 0xffff_8000);
        assert_eq!(RevOp::Rbit.apply(0x0000_0001), 0x8000_0000);
    }

    #[test]
    fn packing() {
        assert_eq!(pack(false, 0x1111_2222, 0x3333_4444, 16), 0x4444_2222);
        assert_eq!(pack(true, 0x1111_2222, 0x8000_0000, 0), 0x1111_ffff);
        assert_eq!(pack(true, 0x1111_2222, 0x1234_0000, 16), 0x1111_1234);
    }

    #[test]
    fn sum_of_absolute_differences() {
        assert_eq!(usad8(0x0010_ff00, 0x0100_00ff), 1 + 16 + 255 + 255);
    }
}
