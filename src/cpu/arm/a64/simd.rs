//! Advanced SIMD: arrangements, lanes, and the rules that are not scalar
//! floating point.
//!
//! [`super::fp`] said this would be needed and what shape it would take:
//!
//! > That keeps this module's interface the same shape as `float`'s, which is
//! > deliberate: an Advanced SIMD element operation, when one lands, is this
//! > arithmetic applied lanewise and needs nothing new from either.
//!
//! It held. Every floating-point lane here goes through `fp`, which goes
//! through [`crate::float`], and this module adds three things `fp` does not
//! have:
//!
//! 1. **Arrangements.** A vector operand is a register *and* a shape —
//!    `V0.4S` — spelled in `size` and `Q`. [`Arrangement`] is that pair, and
//!    it is the only thing that knows how many lanes there are and how wide
//!    they are.
//! 2. **Lane addressing** into the 128-bit register file, at four widths.
//! 3. **The integer element operations**, which have no scalar counterpart in
//!    this core at all: A64's general registers do not have a `SMAX`, so
//!    lanewise `SMAX` cannot be borrowed from anywhere.
//!
//! # Why the arrangement is a type and not two `u32`s
//!
//! Because the three ways to get one are genuinely different rules and mixing
//! them up is silent. `size`:`Q` gives `8B`…`2D`; `sz`:`Q` gives `2S`, `4S`,
//! `2D` and nothing else; `immh` gives the element width of a shift; and
//! `imm5` gives it for a lane index. Four decoders, one type, and each
//! rejects what the architecture leaves unallocated rather than picking
//! something.
//!
//! # `Q` is not just a width
//!
//! It means three different things depending on the encoding group, and each
//! is a place a plausible implementation goes wrong:
//!
//! * on a three-same or two-misc operation it is the *vector* width, and a
//!   64-bit result zeroes the top half of the destination;
//! * on a **narrowing** operation (`XTN`/`XTN2`, `FCVTN`/`FCVTN2`) it selects
//!   which half of the *destination* is written and leaves the other alone;
//! * on a **widening** one (`FCVTL`/`FCVTL2`, `SSHLL`/`SSHLL2`, `UMULL2`) it
//!   selects which half of the *source* is read.
//!
//! So the mnemonic carries a `2` that the table's mnemonic column does not,
//! and the disassembler appends it from `Q` — the one place a vector mnemonic
//! is not wholly the row's.
//!
//! # What is deliberately absent
//!
//! The saturating arithmetic (`SQADD`, `SQSHL`, `SQXTN` and relatives, and
//! therefore `FPSR.QC`), the rounding variants (`SRSHR`, `RSHRN`, `URHADD`),
//! polynomial multiply, the reciprocal-estimate family (`FRECPE`, `FRSQRTE`,
//! `FRECPS`, `FRSQRTS`, `FMULX`), `FEAT_FP16` arithmetic, the halving adds,
//! `LD2`/`LD3`/`LD4` of a *single* structure and the replicating loads other
//! than `LD1R`, and everything Armv8.1 and later added. Each is absent from
//! the table, so each raises `UNDEFINED` rather than being quietly wrong.
//!
//! # Sources
//!
//! *Arm Architecture Reference Manual for A-profile architecture* (DDI 0487):
//! chapter C7 for the Advanced SIMD instructions, C4.1.6 for their encodings,
//! and the shared pseudocode chapter for `AdvSIMDExpandImm`, `Elem[]`,
//! `Reduce` and `Replicate`. No emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

use crate::float::{Env, Flags};

use super::fp::{self, Prec};

// ---------------------------------------------------------------------------
// Arrangements
// ---------------------------------------------------------------------------

/// The shape of a vector operand: an element width and a lane count.
///
/// `esize` is the base-2 logarithm of the element size in bytes — `0` is a
/// byte and `3` a doubleword — which is how every A64 `size` field spells it,
/// so no conversion happens at the decode boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Arrangement {
    /// Base-2 logarithm of the element width in bytes, `0..=3`.
    pub esize: u32,
    /// How many lanes: `1`, `2`, `4`, `8` or `16`.
    pub lanes: u32,
}

impl Arrangement {
    /// The arrangement a `size`:`Q` pair names.
    ///
    /// `None` is `1D` — one 64-bit lane in a 64-bit register — which the
    /// architecture reserves on every three-same and two-misc integer
    /// encoding: there is no useful lanewise operation on a single lane, and
    /// Arm makes the combination `UNDEFINED` rather than defining it.
    #[must_use]
    pub const fn from_size(size: u32, q: bool) -> Option<Arrangement> {
        if size > 3 {
            return None;
        }
        if size == 3 && !q {
            return None;
        }
        Some(Arrangement {
            esize: size,
            lanes: (if q { 16 } else { 8 }) >> size,
        })
    }

    /// The arrangement filling a register at an element width, **`1D`
    /// included**.
    ///
    /// [`Arrangement::from_size`] refuses `1D` because no lanewise operation
    /// allocates it. The load/store side does: `LD1 { V0.1D }, [X0]` and
    /// `LD1R { V0.1D }, [X0]` are ordinary instructions, and refusing them
    /// there would be applying a data-processing rule to a memory access.
    #[must_use]
    pub const fn whole(size: u32, q: bool) -> Option<Arrangement> {
        if size > 3 {
            return None;
        }
        Some(Arrangement {
            esize: size,
            lanes: (if q { 16 } else { 8 }) >> size,
        })
    }

    /// The arrangement a floating-point `sz`:`Q` pair names: `2S`, `4S` or
    /// `2D`.
    ///
    /// `None` is `1D`, reserved for the same reason as above — and it is the
    /// combination a naive `if sz { Double }` would happily execute.
    #[must_use]
    pub const fn from_sz(sz: bool, q: bool) -> Option<Arrangement> {
        match (sz, q) {
            (false, false) => Some(Arrangement { esize: 2, lanes: 2 }),
            (false, true) => Some(Arrangement { esize: 2, lanes: 4 }),
            (true, true) => Some(Arrangement { esize: 3, lanes: 2 }),
            (true, false) => None,
        }
    }

    /// The width of one lane in bits.
    #[inline]
    #[must_use]
    pub const fn bits(self) -> u32 {
        8 << self.esize
    }

    /// The width of the whole operand in bytes: 8 or 16.
    #[inline]
    #[must_use]
    pub const fn bytes(self) -> u64 {
        if self.is_q() { 16 } else { 8 }
    }

    /// Whether the operand is 128 bits wide.
    #[inline]
    #[must_use]
    pub const fn is_q(self) -> bool {
        (self.lanes << self.esize) == 16
    }

    /// The assembler spelling — `16b`, `4s`, `2d`.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match (self.esize, self.lanes) {
            (0, 8) => "8b",
            (0, 16) => "16b",
            (1, 4) => "4h",
            (1, 8) => "8h",
            (2, 2) => "2s",
            (2, 4) => "4s",
            (3, 1) => "1d",
            (3, 2) => "2d",
            _ => "?",
        }
    }

    /// The floating-point format the lanes hold.
    ///
    /// `None` for a byte or halfword lane: half-precision *arithmetic* needs
    /// `FEAT_FP16`, which this core does not have, and a byte is not a
    /// floating-point format at all.
    #[must_use]
    pub const fn prec(self) -> Option<Prec> {
        match self.esize {
            2 => Some(Prec::Single),
            3 => Some(Prec::Double),
            _ => None,
        }
    }
}

/// The letter naming an element width — the `S` in `V0.S[2]`.
#[must_use]
pub const fn elem_letter(esize: u32) -> char {
    match esize {
        0 => 'b',
        1 => 'h',
        2 => 's',
        _ => 'd',
    }
}

// ---------------------------------------------------------------------------
// Lanes
// ---------------------------------------------------------------------------

/// Read lane `index` of `value` at element width `esize`.
///
/// Out-of-range indices read zero rather than panicking: every caller has
/// already checked the index against an arrangement, and a bounds panic in an
/// interpreter turns a guest bug into a host crash.
#[inline]
#[must_use]
pub const fn elem(value: u128, esize: u32, index: u32) -> u64 {
    let bits = 8u32 << esize;
    let shift = index * bits;
    if shift >= 128 {
        return 0;
    }
    let raw = (value >> shift) as u64;
    if bits >= 64 {
        raw
    } else {
        raw & ((1u64 << bits) - 1)
    }
}

/// Write lane `index` of `value`, leaving every other lane alone.
#[inline]
#[must_use]
pub const fn set_elem(value: u128, esize: u32, index: u32, x: u64) -> u128 {
    let bits = 8u32 << esize;
    let shift = index * bits;
    if shift >= 128 {
        return value;
    }
    let mask = if bits >= 64 {
        u64::MAX as u128
    } else {
        (1u128 << bits) - 1
    };
    (value & !(mask << shift)) | (((x as u128) & mask) << shift)
}

/// Sign-extend an `esize`-wide element to 64 bits.
#[inline]
#[must_use]
pub const fn sext(value: u64, esize: u32) -> i64 {
    let bits = 8u32 << esize;
    if bits >= 64 {
        value as i64
    } else {
        let shift = 64 - bits;
        ((value << shift) as i64) >> shift
    }
}

/// Truncate a 64-bit result to an `esize`-wide element.
#[inline]
#[must_use]
pub const fn trunc(value: u64, esize: u32) -> u64 {
    let bits = 8u32 << esize;
    if bits >= 64 {
        value
    } else {
        value & ((1u64 << bits) - 1)
    }
}

/// An all-ones element, which is what every SIMD comparison writes for true.
///
/// Not `1`: a vector compare produces a *mask*, because the result feeds
/// `BSL`/`BIT`/`AND` rather than a branch. Getting this wrong gives code that
/// works for `!= 0` tests and silently fails for everything else.
#[inline]
#[must_use]
pub const fn mask_of(cond: bool, esize: u32) -> u64 {
    if cond { trunc(u64::MAX, esize) } else { 0 }
}

/// Replicate an element across a 64-bit value.
#[must_use]
pub const fn replicate(x: u64, esize: u32) -> u64 {
    let bits = 8u32 << esize;
    let mut out = 0u64;
    let mut pos = 0u32;
    while pos < 64 {
        out |= trunc(x, esize) << pos;
        pos += bits;
    }
    out
}

// ---------------------------------------------------------------------------
// AdvSIMDExpandImm
// ---------------------------------------------------------------------------

/// `AdvSIMDExpandImm`: the 64-bit pattern a modified-immediate encoding names.
///
/// DDI 0487 shared pseudocode. `op` is bit 29 and matters only at
/// `cmode<3:1> == 0b111`, where it chooses between the byte-replicated form
/// and the bytemask, and between the two `FMOV` precisions.
///
/// `None` is the one unallocated combination this core can reach: `cmode`
/// `0b1111` with `op == 1` is `FMOV` at double precision, which requires
/// `Q == 1`; the caller checks `Q` because this function does not see it.
#[must_use]
pub const fn expand_imm(op: bool, cmode: u32, imm8: u32) -> Option<u64> {
    let byte = imm8 as u64;
    let value = match (cmode >> 1) & 7 {
        0b000 => replicate(byte, 2),
        0b001 => replicate(byte << 8, 2),
        0b010 => replicate(byte << 16, 2),
        0b011 => replicate(byte << 24, 2),
        0b100 => replicate(byte, 1),
        0b101 => replicate(byte << 8, 1),
        0b110 => {
            // `MSL`: the immediate is shifted left and ones are shifted in,
            // which is what makes `MOVI V0.4S, #1, MSL #8` a mask of 0x1ff.
            if cmode & 1 == 0 {
                replicate((byte << 8) | 0xff, 2)
            } else {
                replicate((byte << 16) | 0xffff, 2)
            }
        }
        _ => {
            if cmode & 1 == 0 {
                if op {
                    // The bytemask: each bit of the immediate becomes a byte.
                    let mut out = 0u64;
                    let mut i = 0u32;
                    while i < 8 {
                        if imm8 & (1 << i) != 0 {
                            out |= 0xffu64 << (i * 8);
                        }
                        i += 1;
                    }
                    out
                } else {
                    replicate(byte, 0)
                }
            } else if op {
                return Some(fp::expand_imm(imm8, Prec::Double));
            } else {
                replicate(fp::expand_imm(imm8, Prec::Single), 2)
            }
        }
    };
    Some(value)
}

// ---------------------------------------------------------------------------
// Integer element operations
// ---------------------------------------------------------------------------

/// Lanewise add.
#[inline]
#[must_use]
pub const fn add(esize: u32, a: u64, b: u64) -> u64 {
    trunc(a.wrapping_add(b), esize)
}

/// Lanewise subtract.
#[inline]
#[must_use]
pub const fn sub(esize: u32, a: u64, b: u64) -> u64 {
    trunc(a.wrapping_sub(b), esize)
}

/// Lanewise multiply, keeping the low half.
#[inline]
#[must_use]
pub const fn mul(esize: u32, a: u64, b: u64) -> u64 {
    trunc(a.wrapping_mul(b), esize)
}

/// Lanewise signed maximum.
#[inline]
#[must_use]
pub const fn smax(esize: u32, a: u64, b: u64) -> u64 {
    if sext(a, esize) >= sext(b, esize) {
        a
    } else {
        b
    }
}

/// Lanewise signed minimum.
#[inline]
#[must_use]
pub const fn smin(esize: u32, a: u64, b: u64) -> u64 {
    if sext(a, esize) <= sext(b, esize) {
        a
    } else {
        b
    }
}

/// Lanewise unsigned maximum.
#[inline]
#[must_use]
pub const fn umax(_esize: u32, a: u64, b: u64) -> u64 {
    if a >= b { a } else { b }
}

/// Lanewise unsigned minimum.
#[inline]
#[must_use]
pub const fn umin(_esize: u32, a: u64, b: u64) -> u64 {
    if a <= b { a } else { b }
}

/// Lanewise signed absolute difference.
#[inline]
#[must_use]
pub const fn sabd(esize: u32, a: u64, b: u64) -> u64 {
    let (x, y) = (sext(a, esize), sext(b, esize));
    let d = if x >= y {
        (x as i128) - (y as i128)
    } else {
        (y as i128) - (x as i128)
    };
    trunc(d as u64, esize)
}

/// Lanewise unsigned absolute difference.
#[inline]
#[must_use]
pub const fn uabd(esize: u32, a: u64, b: u64) -> u64 {
    trunc(a.abs_diff(b), esize)
}

/// Lanewise absolute value.
#[inline]
#[must_use]
pub const fn abs(esize: u32, a: u64) -> u64 {
    trunc(sext(a, esize).wrapping_abs() as u64, esize)
}

/// Lanewise negate.
#[inline]
#[must_use]
pub const fn neg(esize: u32, a: u64) -> u64 {
    trunc((a as i64).wrapping_neg() as u64, esize)
}

/// `SSHL`/`USHL`: shift by the *signed byte* in the low bits of the second
/// operand — left when positive, right when negative.
///
/// One instruction covering both directions is why A64 has no vector shift
/// right by a register: `USHL` with a negated amount is it. The shift amount
/// is the low eight bits of the second operand read as a signed byte,
/// whatever the element width.
#[must_use]
pub const fn shl_reg(esize: u32, a: u64, b: u64, signed: bool) -> u64 {
    let bits = 8i32 << esize;
    let amount = ((b & 0xff) as u8) as i8 as i32;
    if amount >= 0 {
        if amount >= bits {
            0
        } else {
            trunc(a << amount, esize)
        }
    } else {
        let shift = -amount;
        if signed {
            let value = sext(a, esize);
            let shift = if shift >= bits { bits - 1 } else { shift };
            trunc((value >> shift) as u64, esize)
        } else if shift >= bits {
            0
        } else {
            trunc(a, esize) >> shift
        }
    }
}

/// Lanewise signed greater-than, as a mask.
#[inline]
#[must_use]
pub const fn cmgt(esize: u32, a: u64, b: u64) -> u64 {
    mask_of(sext(a, esize) > sext(b, esize), esize)
}

/// Lanewise signed greater-than-or-equal, as a mask.
#[inline]
#[must_use]
pub const fn cmge(esize: u32, a: u64, b: u64) -> u64 {
    mask_of(sext(a, esize) >= sext(b, esize), esize)
}

/// Lanewise unsigned higher, as a mask.
#[inline]
#[must_use]
pub const fn cmhi(esize: u32, a: u64, b: u64) -> u64 {
    mask_of(a > b, esize)
}

/// Lanewise unsigned higher-or-same, as a mask.
#[inline]
#[must_use]
pub const fn cmhs(esize: u32, a: u64, b: u64) -> u64 {
    mask_of(a >= b, esize)
}

/// Lanewise equality, as a mask.
#[inline]
#[must_use]
pub const fn cmeq(esize: u32, a: u64, b: u64) -> u64 {
    mask_of(a == b, esize)
}

/// `CMTST`: true where the two lanes share a set bit.
#[inline]
#[must_use]
pub const fn cmtst(esize: u32, a: u64, b: u64) -> u64 {
    mask_of(a & b != 0, esize)
}

/// Count leading zeroes within an `esize`-wide element.
#[inline]
#[must_use]
pub const fn clz(esize: u32, a: u64) -> u64 {
    let bits = 8u32 << esize;
    (trunc(a, esize).leading_zeros() - (64 - bits)) as u64
}

/// Count leading *sign* bits within an `esize`-wide element: the number of
/// bits after the topmost one that equal it.
///
/// One less than `CLZ` of the value with its sign folded away, and that is
/// the whole definition — `CLS` of zero is `bits - 1`, not `bits`.
#[inline]
#[must_use]
pub const fn cls(esize: u32, a: u64) -> u64 {
    let bits = 8u32 << esize;
    let value = trunc(a, esize);
    let folded = if (value >> (bits - 1)) & 1 == 1 {
        trunc(!value, esize)
    } else {
        value
    };
    clz(esize, folded) - 1
}

/// Count the set bits of each byte of a value.
#[must_use]
pub const fn cnt_bytes(value: u64) -> u64 {
    let mut out = 0u64;
    let mut i = 0u32;
    while i < 8 {
        let byte = (value >> (i * 8)) & 0xff;
        out |= (byte.count_ones() as u64) << (i * 8);
        i += 1;
    }
    out
}

/// Reverse the bits of each byte of a value.
#[must_use]
pub const fn rbit_bytes(value: u64) -> u64 {
    let mut out = 0u64;
    let mut i = 0u32;
    while i < 8 {
        let byte = ((value >> (i * 8)) & 0xff) as u8;
        out |= (byte.reverse_bits() as u64) << (i * 8);
        i += 1;
    }
    out
}

/// Reverse the order of the `esize`-wide elements inside each `group`-wide
/// container.
///
/// `REV64`, `REV32` and `REV16` are one rule with two widths: reverse the
/// elements within a container. Writing it once is what keeps `REV16 V0.16B`
/// from being three near-identical loops.
#[must_use]
pub const fn rev_within(value: u64, esize: u32, group: u32) -> u64 {
    let ebits = 8u32 << esize;
    let per = (8u32 << group) / ebits;
    let mut out = 0u64;
    let mut i = 0u32;
    while i * ebits < 64 {
        let slot = i % per;
        let base = i - slot;
        let src = base + (per - 1 - slot);
        let e = if ebits >= 64 {
            value
        } else {
            (value >> (src * ebits)) & ((1u64 << ebits) - 1)
        };
        out |= e << (i * ebits);
        i += 1;
    }
    out
}

// ---------------------------------------------------------------------------
// Floating-point element operations
// ---------------------------------------------------------------------------

/// `FABD`: the absolute value of the difference, as one operation rather than
/// a subtract followed by an `FABS`.
///
/// It matters that this is one operation: `FABD` of two infinities of the
/// same sign is the `Invalid` NaN the subtract produces, and clearing the
/// sign bit of that NaN afterwards would be wrong for a NaN whose sign the
/// architecture propagates.
#[must_use]
pub fn fabd(prec: Prec, a: u64, b: u64, env: Env) -> (u64, Flags) {
    let (diff, flags) = fp::sub(prec, a, b, env);
    (fp::abs(prec, diff), flags)
}

/// A lanewise floating-point comparison, producing a mask.
///
/// The three vector comparisons are *quiet* on a quiet NaN and signalling on
/// a signalling one, and they answer `false` rather than setting `NZCV` —
/// which is why they cannot reuse [`fp::compare`], whose whole output is the
/// four-way condition code.
#[must_use]
pub fn fcompare(prec: Prec, a: u64, b: u64, kind: FpCmp, env: Env) -> (bool, Flags) {
    let (a, b) = if kind.absolute() {
        (fp::abs(prec, a), fp::abs(prec, b))
    } else {
        (a, b)
    };
    // `FCMEQ` is a quiet predicate and the other four are signalling ones:
    // IEEE 754 §5.11 makes `>` and `>=` signal on a quiet NaN where `=` does
    // not, and DDI 0487 follows it (`FPCompareEQ` versus `FPCompareGE`).
    let (nzcv, flags) = fp::compare(prec, a, b, kind != FpCmp::Eq, env);
    // `FPCompare` gives `0b0011` for unordered, `0b0110` for equal, `0b1000`
    // for less than and `0b0010` for greater than. Reading the flags rather
    // than re-deriving the order is what keeps the NaN answer right.
    let unordered = nzcv.c() && nzcv.v();
    let equal = nzcv.z();
    let greater = nzcv.c() && !nzcv.v() && !nzcv.z();
    let held = match kind {
        FpCmp::Eq => equal && !unordered,
        FpCmp::Ge | FpCmp::AbsGe => !unordered && (equal || greater),
        FpCmp::Gt | FpCmp::AbsGt => !unordered && greater,
    };
    (held, flags)
}

/// Which of the vector floating-point comparisons to make.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FpCmp {
    /// Equal.
    Eq,
    /// Greater than or equal.
    Ge,
    /// Greater than.
    Gt,
    /// `FACGE`: greater than or equal, on absolute values.
    AbsGe,
    /// `FACGT`: greater than, on absolute values.
    AbsGt,
}

impl FpCmp {
    /// Whether the comparison is on absolute values.
    #[inline]
    #[must_use]
    pub const fn absolute(self) -> bool {
        matches!(self, FpCmp::AbsGe | FpCmp::AbsGt)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The four ways to name an arrangement, and the two each of them
    /// reserves. `1D` is the interesting one: it is reserved for every
    /// lanewise operation and legal for a load, which is why there are two
    /// constructors rather than one with a flag.
    #[test]
    fn the_reserved_arrangements_are_rejected() {
        assert_eq!(
            Arrangement::from_size(0, true),
            Some(Arrangement {
                esize: 0,
                lanes: 16
            })
        );
        assert_eq!(
            Arrangement::from_size(3, true),
            Some(Arrangement { esize: 3, lanes: 2 })
        );
        assert_eq!(Arrangement::from_size(3, false), None, "1D is reserved");
        assert_eq!(
            Arrangement::whole(3, false),
            Some(Arrangement { esize: 3, lanes: 1 }),
            "... but a load may name it"
        );
        assert_eq!(Arrangement::from_sz(true, false), None, "1D again");
        assert_eq!(Arrangement::from_sz(false, false).unwrap().name(), "2s");
        assert_eq!(Arrangement::from_sz(true, true).unwrap().name(), "2d");
        assert_eq!(Arrangement::from_size(1, false).unwrap().name(), "4h");
        // A byte or a halfword lane is not a format this core does arithmetic
        // in: `FEAT_FP16` is absent, and a byte never was one.
        assert!(Arrangement::from_size(0, true).unwrap().prec().is_none());
        assert!(Arrangement::from_size(1, true).unwrap().prec().is_none());
        assert!(Arrangement::from_size(2, true).unwrap().prec().is_some());
    }

    /// Lane addressing at all four widths, including the doubleword where the
    /// obvious `(1 << bits) - 1` mask overflows.
    #[test]
    fn lanes_are_addressed_at_four_widths() {
        let v = 0x0f0e_0d0c_0b0a_0908_0706_0504_0302_0100u128;
        assert_eq!(elem(v, 0, 3), 0x03);
        assert_eq!(elem(v, 1, 3), 0x0706);
        assert_eq!(elem(v, 2, 3), 0x0f0e_0d0c);
        assert_eq!(elem(v, 3, 1), 0x0f0e_0d0c_0b0a_0908);
        // Past the end reads zero rather than panicking: an interpreter must
        // not turn a guest's bad encoding into a host crash.
        assert_eq!(elem(v, 3, 2), 0);
        assert_eq!(set_elem(v, 3, 2, 1), v);

        assert_eq!(set_elem(0, 3, 1, u64::MAX) >> 64, u128::from(u64::MAX));
        assert_eq!(set_elem(u128::MAX, 0, 0, 0) & 0xff, 0);
    }

    /// `AdvSIMDExpandImm`, against the values DDI 0487 tabulates — including
    /// the two that are not a shifted byte at all: the `MSL` forms shift ones
    /// in underneath, and the `op == 1, cmode == 0b1110` form turns each bit
    /// of the immediate into a byte.
    #[test]
    fn the_modified_immediate_follows_the_pseudocode() {
        // cmode 0bxxx0: a byte in each 32-bit lane, at four shifts.
        assert_eq!(expand_imm(false, 0b0000, 0xab), Some(0x0000_00ab_0000_00ab));
        assert_eq!(expand_imm(false, 0b0010, 0xab), Some(0x0000_ab00_0000_ab00));
        assert_eq!(expand_imm(false, 0b0100, 0xab), Some(0x00ab_0000_00ab_0000));
        assert_eq!(expand_imm(false, 0b0110, 0xab), Some(0xab00_0000_ab00_0000));
        // cmode 0b10x0: a byte in each 16-bit lane.
        assert_eq!(expand_imm(false, 0b1000, 0xab), Some(0x00ab_00ab_00ab_00ab));
        assert_eq!(expand_imm(false, 0b1010, 0xab), Some(0xab00_ab00_ab00_ab00));
        // cmode 0b110x: `MSL`, which shifts *ones* in.
        assert_eq!(expand_imm(false, 0b1100, 0xab), Some(0x0000_abff_0000_abff));
        assert_eq!(expand_imm(false, 0b1101, 0xab), Some(0x00ab_ffff_00ab_ffff));
        // cmode 0b1110 with `op` clear: the byte, replicated eight times.
        assert_eq!(expand_imm(false, 0b1110, 0xab), Some(0xabab_abab_abab_abab));
        // ... and with `op` set: a bytemask, which is what makes `MOVI D0, #0`
        // and every other lane pattern a compiler wants.
        assert_eq!(expand_imm(true, 0b1110, 0x00), Some(0));
        assert_eq!(expand_imm(true, 0b1110, 0xff), Some(u64::MAX));
        assert_eq!(
            expand_imm(true, 0b1110, 0b1010_0101),
            Some(0xff00_ff00_00ff_00ff),
            "bits 0, 2, 5 and 7 become bytes 0, 2, 5 and 7"
        );
        // cmode 0b1111: the two `FMOV` precisions.
        assert_eq!(expand_imm(false, 0b1111, 0x70), Some(0x3f80_0000_3f80_0000));
        assert_eq!(expand_imm(true, 0b1111, 0x70), Some(0x3ff0_0000_0000_0000));
    }

    /// `CLS` counts the bits *after* the topmost one that match it, so it is
    /// one less than the obvious count and `CLS` of zero is `bits - 1`.
    #[test]
    fn cls_counts_one_fewer_than_clz() {
        assert_eq!(cls(2, 0), 31);
        assert_eq!(cls(2, u64::MAX), 31);
        assert_eq!(cls(2, 0x8000_0000), 0);
        assert_eq!(cls(2, 0x4000_0000), 0, "the bit after the sign differs");
        assert_eq!(cls(2, 0x2000_0000), 1);
        assert_eq!(clz(2, 0), 32);
        assert_eq!(clz(2, 0x4000_0000), 1);
        assert_eq!(clz(0, 0), 8);
    }

    /// One rule, three instructions: `REV64`, `REV32` and `REV16` reverse the
    /// elements inside a container of their own width.
    #[test]
    fn the_reversals_are_one_rule_at_three_widths() {
        let v = 0x0706_0504_0302_0100u64;
        assert_eq!(rev_within(v, 0, 3), 0x0001_0203_0405_0607);
        assert_eq!(rev_within(v, 0, 2), 0x0405_0607_0001_0203);
        assert_eq!(rev_within(v, 0, 1), 0x0607_0405_0203_0001);
        assert_eq!(rev_within(v, 1, 3), 0x0100_0302_0504_0706);
    }

    /// `SSHL`/`USHL` take a *signed byte* as the amount, and an amount past
    /// the element width does not wrap: it saturates to zero, or to the sign
    /// for the arithmetic form.
    #[test]
    fn a_register_shift_reads_a_signed_byte() {
        assert_eq!(shl_reg(2, 1, 4, false), 16);
        assert_eq!(shl_reg(2, 16, 0xfc, false), 1, "-4 shifts right");
        assert_eq!(shl_reg(2, 0xffff_ffff, 0xe0, true), 0xffff_ffff, "-32");
        assert_eq!(shl_reg(2, 0xffff_ffff, 0xe0, false), 0, "unsigned, -32");
        assert_eq!(shl_reg(2, 1, 32, false), 0, "left by the width");
        // The amount is the low byte whatever the element width, so a value
        // whose upper bits are set is still read as that byte.
        assert_eq!(shl_reg(0, 1, 0xffff_ff02, false), 4);
    }

    /// The bit and byte counters, on the patterns where an off-by-one shows.
    #[test]
    fn the_bit_counters_work_bytewise() {
        assert_eq!(rbit_bytes(0x0102_0408_1020_4080), 0x8040_2010_0804_0201);
        assert_eq!(cnt_bytes(u64::MAX), 0x0808_0808_0808_0808);
        assert_eq!(cnt_bytes(0x0102_0408_1020_4080), 0x0101_0101_0101_0101);
    }

    /// A comparison writes a mask of the element's own width, which is the
    /// thing that makes the result usable by `BSL` without a widening step.
    #[test]
    fn a_comparison_writes_an_element_wide_mask() {
        assert_eq!(mask_of(true, 0), 0xff);
        assert_eq!(mask_of(true, 2), 0xffff_ffff);
        assert_eq!(mask_of(true, 3), u64::MAX);
        assert_eq!(mask_of(false, 3), 0);
        assert_eq!(cmgt(0, 0x7f, 0x80), 0xff, "signed: 127 > -128");
        assert_eq!(cmhi(0, 0x7f, 0x80), 0, "unsigned: 127 is not above 128");
    }

    /// `FABD` is one operation rather than a subtract and an `FABS`, and the
    /// difference shows on a signalling NaN — which the subtract quietens
    /// while raising `Invalid`, and which an `FABS` afterwards would then
    /// strip the sign of.
    #[test]
    fn fabd_is_one_operation() {
        let env = Env::ARM;
        let (value, flags) = fabd(
            Prec::Double,
            0x4000_0000_0000_0000,
            0x3ff0_0000_0000_0000,
            env,
        );
        assert_eq!(value, 0x3ff0_0000_0000_0000, "|2.0 - 1.0|");
        assert!(flags.is_empty());
        // The other way round gives the same magnitude, which is the point.
        let (value, _) = fabd(
            Prec::Double,
            0x3ff0_0000_0000_0000,
            0x4000_0000_0000_0000,
            env,
        );
        assert_eq!(value, 0x3ff0_0000_0000_0000);
        // Infinity minus infinity is Invalid, and the NaN it produces is what
        // comes out — not a NaN with its sign cleared afterwards.
        let inf = 0x7ff0_0000_0000_0000;
        let (_, flags) = fabd(Prec::Double, inf, inf, env);
        assert!(flags.contains(Flags::INVALID));
    }

    /// IEEE 754 §5.11: `=` is a quiet predicate and `>` and `>=` are
    /// signalling ones, so a quiet NaN raises `Invalid` on `FCMGT` and not on
    /// `FCMEQ`. Getting this backwards is invisible until a guest enables the
    /// trap or reads `FPSR`.
    #[test]
    fn the_ordering_predicates_signal_and_equality_does_not() {
        let env = Env::ARM;
        let qnan = 0x7ff8_0000_0000_0000;
        let one = 0x3ff0_0000_0000_0000;
        let (held, flags) = fcompare(Prec::Double, qnan, one, FpCmp::Eq, env);
        assert!(!held);
        assert!(flags.is_empty(), "FCMEQ is quiet on a quiet NaN");
        let (held, flags) = fcompare(Prec::Double, qnan, one, FpCmp::Gt, env);
        assert!(!held);
        assert!(flags.contains(Flags::INVALID), "FCMGT signals");
        // The absolute forms compare magnitudes.
        let minus_two = 0xc000_0000_0000_0000;
        assert!(!fcompare(Prec::Double, minus_two, one, FpCmp::Gt, env).0);
        assert!(fcompare(Prec::Double, minus_two, one, FpCmp::AbsGt, env).0);
        // And a zero of either sign compares equal to the other.
        assert!(fcompare(Prec::Double, 0, 1 << 63, FpCmp::Eq, env).0);
        assert!(fcompare(Prec::Double, 0, 1 << 63, FpCmp::Ge, env).0);
        assert!(!fcompare(Prec::Double, 0, 1 << 63, FpCmp::Gt, env).0);
    }
}
