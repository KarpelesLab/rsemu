//! The SIMD and extension halves of the DSP (E) extension.
//!
//! These are pure functions on register values with no processor state
//! involved beyond the `GE` bits they hand back, which makes them the easiest
//! part of the core to test exhaustively — and the part where a sign error
//! is least likely to show up as anything but a wrong number much later.
//!
//! The parallel add/sub family is one operation with three axes: which lanes
//! (`8`, `16`, or one of the two exchanging arrangements), whether the lanes
//! are signed or unsigned, and whether results wrap, saturate or halve. That
//! is eighteen mnemonics from three enum values, which is why this is a
//! function of [`SimdMode`] and [`SimdShape`] rather than eighteen arms.
//!
//! # `GE` and who sets it
//!
//! Only the *wrapping* forms set the `GE` bits — `SADD16` does, `QADD16` and
//! `SHADD16` do not (DDI 0403 A7.7.135's "Operation" and its siblings). `GE`
//! then feeds `SEL`, and that pair is the whole reason the wrapping forms
//! exist: a saturating add has nowhere to report which lanes were greater.
//!
//! # Sources
//!
//! DDI 0403 A7.7's parallel-arithmetic pages (`SADD8`, `SADD16`, `SASX`,
//! `SSAX`, `SSUB8`, `SSUB16` and their `Q`, `SH`, `U`, `UQ` and `UH`
//! variants), and A7.7.184 onward for the extending moves. No emulator source
//! of any licence was consulted (`ROADMAP.md` §1).

use super::isa::{ExtendOp, SimdMode, SimdShape};

/// One parallel add or subtract.
///
/// Returns the result and, for the forms that set them, the four `GE` bits.
#[must_use]
pub(super) fn simd(mode: SimdMode, shape: SimdShape, a: u32, b: u32) -> (u32, Option<u8>) {
    match shape {
        SimdShape::Add8 | SimdShape::Sub8 => simd8(mode, shape == SimdShape::Sub8, a, b),
        _ => simd16(mode, shape, a, b),
    }
}

/// The halfword lanes, including the two exchanging arrangements.
fn simd16(mode: SimdMode, shape: SimdShape, a: u32, b: u32) -> (u32, Option<u8>) {
    let unsigned = mode.is_unsigned();
    let a_lo = lane16(a, 0, unsigned);
    let a_hi = lane16(a, 1, unsigned);
    let (b_lo, b_hi) = match shape {
        // `ASX` and `SAX` cross `Rm`'s halves; which lane is added and which
        // subtracted is the difference between them.
        SimdShape::Asx | SimdShape::Sax => (lane16(b, 1, unsigned), lane16(b, 0, unsigned)),
        _ => (lane16(b, 0, unsigned), lane16(b, 1, unsigned)),
    };
    let (sub_lo, sub_hi) = match shape {
        SimdShape::Add16 => (false, false),
        SimdShape::Sub16 => (true, true),
        // `ASX`: subtract in the low lane, add in the high one.
        SimdShape::Asx => (true, false),
        // `SAX`: add in the low lane, subtract in the high one.
        _ => (false, true),
    };
    let (lo, ge_lo) = lane_op(mode, sub_lo, a_lo, b_lo, 16);
    let (hi, ge_hi) = lane_op(mode, sub_hi, a_hi, b_hi, 16);
    let result = ((lo as u32) & 0xffff) | (((hi as u32) & 0xffff) << 16);
    let ge = if mode.sets_ge() {
        // Each halfword sets two `GE` bits, so that `SEL` can pick halfwords
        // with the same instruction that picks bytes.
        Some((u8::from(ge_lo) * 0b0011) | (u8::from(ge_hi) * 0b1100))
    } else {
        None
    };
    (result, ge)
}

/// The byte lanes.
fn simd8(mode: SimdMode, sub: bool, a: u32, b: u32) -> (u32, Option<u8>) {
    let unsigned = mode.is_unsigned();
    let mut result = 0u32;
    let mut ge = 0u8;
    for k in 0..4 {
        let x = lane8(a, k, unsigned);
        let y = lane8(b, k, unsigned);
        let (v, g) = lane_op(mode, sub, x, y, 8);
        result |= ((v as u32) & 0xff) << (8 * k);
        if g {
            ge |= 1 << k;
        }
    }
    (result, if mode.sets_ge() { Some(ge) } else { None })
}

/// One lane's arithmetic, and whether its `GE` bit is set.
///
/// `GE` means "this lane's result is non-negative" for the signed forms and
/// "this lane did not borrow" — equivalently, "carried" for an add — for the
/// unsigned ones. Both readings are the same statement: the lane's true
/// mathematical result would not have needed the bit the width does not have.
fn lane_op(mode: SimdMode, sub: bool, a: i32, b: i32, width: u32) -> (i32, bool) {
    let raw = if sub { a - b } else { a + b };
    let ge = if mode.is_unsigned() {
        if sub { raw >= 0 } else { raw >= (1 << width) }
    } else {
        raw >= 0
    };
    let value = match mode {
        SimdMode::Signed | SimdMode::Unsigned => raw,
        SimdMode::SignedSat => {
            let max = (1i32 << (width - 1)) - 1;
            let min = -(1i32 << (width - 1));
            raw.clamp(min, max)
        }
        SimdMode::UnsignedSat => raw.clamp(0, (1i32 << width) - 1),
        // The halving forms are exact: the sum of two *n*-bit values needs
        // *n* + 1 bits, and the shift gives back exactly the lane width with
        // no rounding and no overflow.
        SimdMode::SignedHalve => raw >> 1,
        SimdMode::UnsignedHalve => ((raw as u32) >> 1) as i32,
    };
    (value, ge)
}

/// Lane `k` of a halfword-lane operand.
const fn lane16(value: u32, k: u32, unsigned: bool) -> i32 {
    let half = (value >> (16 * k)) as u16;
    if unsigned {
        half as i32
    } else {
        (half as i16) as i32
    }
}

/// Lane `k` of a byte-lane operand.
const fn lane8(value: u32, k: u32, unsigned: bool) -> i32 {
    let byte = (value >> (8 * k)) as u8;
    if unsigned {
        byte as i32
    } else {
        (byte as i8) as i32
    }
}

/// The extending moves' non-accumulating result (DDI 0403 A7.7.184 onward).
///
/// The rotate has already been applied by the caller, because it is part of
/// the addressing of the source rather than of the extension.
#[must_use]
pub(super) const fn extend(op: ExtendOp, rotated: u32) -> u32 {
    match op {
        ExtendOp::Sxtb => ((rotated as u8) as i8) as i32 as u32,
        ExtendOp::Sxth => ((rotated as u16) as i16) as i32 as u32,
        ExtendOp::Uxtb => rotated & 0xff,
        ExtendOp::Uxth => rotated & 0xffff,
        // The `16` forms extend *two* bytes into two halfwords, which is what
        // makes `UXTAB16` a parallel accumulate rather than a scalar one.
        ExtendOp::Sxtb16 => {
            let lo = ((rotated as u8) as i8) as i32 as u32 & 0xffff;
            let hi = (((rotated >> 16) as u8) as i8) as i32 as u32 & 0xffff;
            lo | (hi << 16)
        }
        ExtendOp::Uxtb16 => (rotated & 0xff) | ((rotated >> 16) & 0xff) << 16,
    }
}

/// The accumulating forms: `SXTAB` and friends.
///
/// The `16` variants accumulate each halfword separately and wrap within it,
/// which is why this is not simply `acc.wrapping_add(value)`.
#[must_use]
pub(super) const fn extend_accumulate(op: ExtendOp, acc: u32, value: u32) -> u32 {
    match op {
        ExtendOp::Sxtb16 | ExtendOp::Uxtb16 => {
            let lo = (acc as u16).wrapping_add(value as u16) as u32;
            let hi = ((acc >> 16) as u16).wrapping_add((value >> 16) as u16) as u32;
            lo | (hi << 16)
        }
        _ => acc.wrapping_add(value),
    }
}
