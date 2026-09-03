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
//! 4. **Saturation**, which is the same kind of thing and is listed separately
//!    because it has an *observable* of its own — see below.
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
//! # Saturation is a *cumulative* operation, not a lanewise one
//!
//! Every saturating operation in this file returns `(value, saturated)` rather
//! than a value, and that is not a convenience. `FPSR.QC` is a single sticky
//! bit for the whole register file: a guest that adds sixteen byte pairs and
//! reads `QC` learns that *some* lane clamped, which is information no lane's
//! result carries — a clamped byte is indistinguishable from one that landed
//! on `0x7f` honestly. So the boolean has to travel out of the arithmetic and
//! into the interpreter, which ORs it into `FPSR`.
//!
//! Which is also why this group is worth landing as one piece. `QC` was
//! writable, readable and set by nothing at all, and an instruction added
//! without it would have been a second lie on top of the first.
//!
//! # What is deliberately absent
//!
//! Polynomial multiply, the reciprocal-estimate family (`FRECPE`, `FRSQRTE`,
//! `FRECPS`, `FRSQRTS`, `FMULX`), `FEAT_FP16` arithmetic, the pairwise
//! long adds (`SADDLP`, `UADALP`), `SHLL`, the halving-narrow three-different
//! family (`ADDHN`, `RADDHN`, `SUBHN`, `RSUBHN`), the absolute-difference-long
//! group (`SABAL`, `UABDL`), the saturating **by-element** forms
//! (`SQDMULH`/`SQRDMULH`/`SQDMULL` and relatives with a lane index),
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
///
/// `?` above a doubleword rather than a fifth letter there is not: a
/// narrowing or widening format computes `esize ± 1` from a field the
/// architecture reserves at one end, and printing `d` for a width that does
/// not exist would make a reserved encoding look like a legal one.
#[must_use]
pub const fn elem_letter(esize: u32) -> char {
    match esize {
        0 => 'b',
        1 => 'h',
        2 => 's',
        3 => 'd',
        _ => '?',
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
///
/// The body is [`shift_reg`] with neither rounding nor saturation, because
/// `SSHL`/`USHL` are the corner of one rule that `SRSHL` and `SQSHL` fill in
/// — see [`shift_by`].
#[inline]
#[must_use]
pub const fn shl_reg(esize: u32, a: u64, b: u64, signed: bool) -> u64 {
    shift_reg(esize, a, b, signed, false, SatTo::Wrap).0
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
// Saturation, and the flag that records it
// ---------------------------------------------------------------------------

/// Where a result is clamped, and to what signedness.
///
/// The A64 narrowing families differ only in this and in how they read their
/// source: `SHRN` wraps, `SQSHRN` clamps to a signed range, and `SQSHRUN`
/// reads a *signed* source and clamps to an unsigned one. Naming the bound
/// rather than passing two booleans is what keeps `SQSHRUN` from being written
/// as "unsigned, but signed".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SatTo {
    /// No clamp: the low bits of the exact result, which is what `SHRN`,
    /// `XTN`, `SSHL` and `USHL` do.
    Wrap,
    /// Clamp to `-2^(N-1) ..= 2^(N-1)-1`.
    Signed,
    /// Clamp to `0 ..= 2^N-1`.
    Unsigned,
}

impl SatTo {
    /// Clamp the exact result `x` into an `esize`-wide element, reporting
    /// whether it was out of range.
    ///
    /// DDI 0487's `SatQ`, and the boolean is the whole reason every operation
    /// below returns a pair: it is what sets `FPSR.QC`, and a guest reads `QC`
    /// to learn that a lane it cannot otherwise see was clamped. Dropping the
    /// bit would leave the flag exactly as dead as it was before this group
    /// landed.
    #[must_use]
    pub const fn apply(self, x: i128, esize: u32) -> (u64, bool) {
        let bits = 8u32 << esize;
        match self {
            SatTo::Wrap => (trunc(x as u64, esize), false),
            SatTo::Signed => {
                let max = (1i128 << (bits - 1)) - 1;
                let min = -(1i128 << (bits - 1));
                if x > max {
                    (trunc(max as u64, esize), true)
                } else if x < min {
                    (trunc(min as u64, esize), true)
                } else {
                    (trunc(x as u64, esize), false)
                }
            }
            SatTo::Unsigned => {
                let max = (1i128 << bits) - 1;
                if x > max {
                    (trunc(max as u64, esize), true)
                } else if x < 0 {
                    (0, true)
                } else {
                    (trunc(x as u64, esize), false)
                }
            }
        }
    }
}

/// Read an `esize`-wide element as an unbounded integer.
///
/// `i128` rather than `i64` because that is what "unbounded" has to mean here:
/// the sum of two doublewords, and the doubled product of two words, both
/// leave 64 bits, and the value that does not fit is the whole subject of a
/// saturating operation.
#[inline]
#[must_use]
pub const fn value_of(x: u64, esize: u32, signed: bool) -> i128 {
    if signed {
        sext(x, esize) as i128
    } else {
        trunc(x, esize) as i128
    }
}

/// `SQADD`: a signed add, clamped.
#[inline]
#[must_use]
pub const fn sqadd(esize: u32, a: u64, b: u64) -> (u64, bool) {
    SatTo::Signed.apply(value_of(a, esize, true) + value_of(b, esize, true), esize)
}

/// `UQADD`: an unsigned add, clamped.
#[inline]
#[must_use]
pub const fn uqadd(esize: u32, a: u64, b: u64) -> (u64, bool) {
    SatTo::Unsigned.apply(value_of(a, esize, false) + value_of(b, esize, false), esize)
}

/// `SQSUB`: a signed subtract, clamped.
#[inline]
#[must_use]
pub const fn sqsub(esize: u32, a: u64, b: u64) -> (u64, bool) {
    SatTo::Signed.apply(value_of(a, esize, true) - value_of(b, esize, true), esize)
}

/// `UQSUB`: an unsigned subtract, clamped — and the clamp at zero is the whole
/// instruction, because an unsigned difference has nowhere else to go.
#[inline]
#[must_use]
pub const fn uqsub(esize: u32, a: u64, b: u64) -> (u64, bool) {
    SatTo::Unsigned.apply(value_of(a, esize, false) - value_of(b, esize, false), esize)
}

/// `SUQADD`: add an **unsigned** source to a **signed** accumulator, clamped
/// signed.
///
/// The two operands are read with different signednesses, which is why this is
/// not [`sqadd`] with its arguments swapped: `acc` is the destination
/// register, read as signed, and `x` is `Vn`, read as unsigned.
#[inline]
#[must_use]
pub const fn suqadd(esize: u32, acc: u64, x: u64) -> (u64, bool) {
    SatTo::Signed.apply(
        value_of(acc, esize, true) + value_of(x, esize, false),
        esize,
    )
}

/// `USQADD`: the mirror image — a **signed** source into an **unsigned**
/// accumulator, clamped unsigned.
#[inline]
#[must_use]
pub const fn usqadd(esize: u32, acc: u64, x: u64) -> (u64, bool) {
    SatTo::Unsigned.apply(
        value_of(acc, esize, false) + value_of(x, esize, true),
        esize,
    )
}

/// `SQABS`: absolute value, clamped.
///
/// It saturates at exactly one input — the most negative value, whose absolute
/// value is one past the widest positive one — and that input is the only
/// thing separating it from [`abs`].
#[inline]
#[must_use]
pub const fn sqabs(esize: u32, a: u64) -> (u64, bool) {
    let value = value_of(a, esize, true);
    let magnitude = if value < 0 { -value } else { value };
    SatTo::Signed.apply(magnitude, esize)
}

/// `SQNEG`: negate, clamped. Saturates at the same single input as [`sqabs`].
#[inline]
#[must_use]
pub const fn sqneg(esize: u32, a: u64) -> (u64, bool) {
    SatTo::Signed.apply(-value_of(a, esize, true), esize)
}

/// The shift-by-an-amount rule shared by eight instructions.
///
/// DDI 0487 states `SSHL`, `USHL`, `SRSHL`, `URSHL`, `SQSHL`, `UQSHL`,
/// `SQRSHL` and `UQRSHL` as one piece of pseudocode with three switches, and
/// they are one function here for the same reason: the rounding constant is
/// added to the *unbounded* value before the shift, and the saturation applies
/// to the *unbounded* result after it. Eight separate bodies is how the
/// rounding ends up on the wrong side of the shift.
///
/// A negative `amount` is a shift right, which can neither overflow nor lose
/// its sign, so the `to` bound only ever fires on a left shift.
#[must_use]
pub const fn shift_by(
    esize: u32,
    a: u64,
    amount: i32,
    signed: bool,
    rounding: bool,
    to: SatTo,
) -> (u64, bool) {
    let bits = 8i32 << esize;
    let value = value_of(a, esize, signed);
    if amount >= 0 {
        if value == 0 {
            return (0, false);
        }
        if amount >= bits {
            // Every significant bit has left the element, so the exact result
            // is out of range in whichever direction the value pointed.
            return match to {
                SatTo::Wrap => (0, false),
                _ => to.apply(if value > 0 { i128::MAX } else { i128::MIN }, esize),
            };
        }
        return to.apply(value << amount, esize);
    }
    // Clamped at 127 because a shift amount is a *signed byte* and can name
    // -128, where `1 << (sh - 1)` would leave `i128`. The answer is the same
    // at every amount past the element width: nothing survives the shift.
    let sh = if amount <= -127 {
        127u32
    } else {
        (-amount) as u32
    };
    let round = if rounding { 1i128 << (sh - 1) } else { 0 };
    to.apply((value + round) >> sh, esize)
}

/// The same, taking the amount from the low signed byte of a second operand —
/// which is how A64 spells it, whatever the element width.
#[inline]
#[must_use]
pub const fn shift_reg(
    esize: u32,
    a: u64,
    b: u64,
    signed: bool,
    rounding: bool,
    to: SatTo,
) -> (u64, bool) {
    let amount = ((b & 0xff) as u8) as i8 as i32;
    shift_by(esize, a, amount, signed, rounding, to)
}

/// `SHADD`/`UHADD`/`SRHADD`/`URHADD`/`SHSUB`/`UHSUB`: a sum or difference kept
/// at the element width by shifting it right one place rather than by
/// discarding the carry.
///
/// None of the six can leave the element's range, so **none of them touches
/// `FPSR.QC`** — which is why this returns a plain value while everything else
/// in this section returns a pair. `SRHADD`/`URHADD` are here for the
/// rounding, not for a saturation they do not do.
#[must_use]
pub const fn halve(
    esize: u32,
    a: u64,
    b: u64,
    signed: bool,
    rounding: bool,
    subtract: bool,
) -> u64 {
    let x = value_of(a, esize, signed);
    let y = value_of(b, esize, signed);
    let exact = if subtract {
        x - y
    } else {
        x + y + (rounding as i128)
    };
    trunc((exact >> 1) as u64, esize)
}

/// `SQDMULH`/`SQRDMULH`: the top half of a **doubled** signed product.
///
/// The doubling is what makes it saturate at all. `-2^(N-1)` times itself is
/// `2^(2N-2)`; doubling that gives `2^(2N-1)`, whose top half is `2^(N-1)` —
/// exactly one past the widest positive element. That single input pair is the
/// whole difference between this and a plain multiply-returning-high, and it is
/// the case a test has to contain.
#[must_use]
pub const fn sqdmulh(esize: u32, a: u64, b: u64, rounding: bool) -> (u64, bool) {
    let bits = 8u32 << esize;
    let doubled = 2 * value_of(a, esize, true) * value_of(b, esize, true);
    let product = if rounding {
        doubled + (1i128 << (bits - 1))
    } else {
        doubled
    };
    SatTo::Signed.apply(product >> bits, esize)
}

/// `SQDMULL`: a doubled signed product in an element twice as wide.
///
/// `esize` is the **source** width. The only input that saturates is the pair
/// of most-negative values, for the same reason as [`sqdmulh`].
#[inline]
#[must_use]
pub const fn sqdmull(esize: u32, a: u64, b: u64) -> (u64, bool) {
    SatTo::Signed.apply(
        2 * value_of(a, esize, true) * value_of(b, esize, true),
        esize + 1,
    )
}

/// The shift-right-and-narrow rule, from `SHRN` through `SQRSHRUN` — and the
/// extract-narrows, which are this at `shift == 0`.
///
/// `esize` is the **destination** width and the source is one width wider,
/// which is what "narrow" means here. `signed` says how the source is read and
/// `to` says how the destination is bounded; the two are independent because
/// `SQSHRUN` reads signed and writes unsigned.
#[must_use]
pub const fn shift_narrow(
    esize: u32,
    a: u64,
    shift: u32,
    signed: bool,
    rounding: bool,
    to: SatTo,
) -> (u64, bool) {
    let value = value_of(a, esize + 1, signed);
    let round = if rounding && shift > 0 {
        1i128 << (shift - 1)
    } else {
        0
    };
    to.apply((value + round) >> shift, esize)
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

    // -----------------------------------------------------------------
    // Saturation, against an oracle that is not this file
    //
    // Unlike the floating-point side, this group has a real one on the host:
    // `i8::saturating_add` and its relatives are the same function, written by
    // somebody else, and `checked_*` says whether the ideal result fit — which
    // is exactly `FPSR.QC`'s input. Where the standard library has no
    // equivalent (the doubling multiplies, the rounding shifts) the oracle
    // below is the manual's formula evaluated at a **wider integer type**,
    // which catches a truncation or an off-by-one in the real path but is not
    // an independent implementation, and is not claimed to be.
    // -----------------------------------------------------------------

    /// Every pair of bytes, four operations, value and flag.
    ///
    /// Exhaustive rather than sampled because it is only 65 536 pairs and
    /// because the interesting inputs of a saturating add are exactly the ones
    /// a sample misses.
    #[test]
    fn the_saturating_adds_agree_with_the_host_at_every_byte_pair() {
        for a in 0..=255u32 {
            for b in 0..=255u32 {
                let (x, y) = (u64::from(a), u64::from(b));
                let (sa, sb) = (a as u8 as i8, b as u8 as i8);
                let (ua, ub) = (a as u8, b as u8);

                let (value, sat) = sqadd(0, x, y);
                assert_eq!(value, u64::from(sa.saturating_add(sb) as u8));
                assert_eq!(sat, sa.checked_add(sb).is_none());

                let (value, sat) = uqadd(0, x, y);
                assert_eq!(value, u64::from(ua.saturating_add(ub)));
                assert_eq!(sat, ua.checked_add(ub).is_none());

                let (value, sat) = sqsub(0, x, y);
                assert_eq!(value, u64::from(sa.saturating_sub(sb) as u8));
                assert_eq!(sat, sa.checked_sub(sb).is_none());

                let (value, sat) = uqsub(0, x, y);
                assert_eq!(value, u64::from(ua.saturating_sub(ub)));
                assert_eq!(sat, ua.checked_sub(ub).is_none());
            }
        }
    }

    /// The same four at the doubleword, where the host type is the whole
    /// element and the implementation's `i128` intermediate is the only thing
    /// between them.
    #[test]
    fn the_saturating_adds_agree_with_the_host_at_the_doubleword() {
        const EDGES: &[u64] = &[
            0,
            1,
            2,
            0x7fff_ffff_ffff_fffe,
            0x7fff_ffff_ffff_ffff,
            0x8000_0000_0000_0000,
            0x8000_0000_0000_0001,
            u64::MAX,
            u64::MAX - 1,
            0x1234_5678_9abc_def0,
        ];
        for &x in EDGES {
            for &y in EDGES {
                let (sa, sb) = (x as i64, y as i64);
                assert_eq!(
                    sqadd(3, x, y),
                    (sa.saturating_add(sb) as u64, sa.checked_add(sb).is_none())
                );
                assert_eq!(
                    uqadd(3, x, y),
                    (x.saturating_add(y), x.checked_add(y).is_none())
                );
                assert_eq!(
                    sqsub(3, x, y),
                    (sa.saturating_sub(sb) as u64, sa.checked_sub(sb).is_none())
                );
                assert_eq!(
                    uqsub(3, x, y),
                    (x.saturating_sub(y), x.checked_sub(y).is_none())
                );
            }
        }
    }

    /// `SUQADD` and `USQADD` read their two operands with *different*
    /// signednesses, and swapping them is invisible on most inputs: it shows
    /// only where one operand's top bit is set. Exhaustive at the byte, which
    /// contains every such case.
    #[test]
    fn the_mixed_signedness_accumulates_read_each_operand_its_own_way() {
        for acc in 0..=255u32 {
            for src in 0..=255u32 {
                // SUQADD: a signed accumulator, an unsigned addend, a signed
                // clamp.
                let exact = i32::from(acc as u8 as i8) + i32::from(src as u8);
                let (value, sat) = suqadd(0, u64::from(acc), u64::from(src));
                assert_eq!(value, u64::from(exact.clamp(-128, 127) as u8));
                assert_eq!(sat, !(-128..=127).contains(&exact));

                // USQADD: an unsigned accumulator, a signed addend, an
                // unsigned clamp — which is the one that can go *below* zero.
                let exact = i32::from(acc as u8) + i32::from(src as u8 as i8);
                let (value, sat) = usqadd(0, u64::from(acc), u64::from(src));
                assert_eq!(value, u64::from(exact.clamp(0, 255) as u8));
                assert_eq!(sat, !(0..=255).contains(&exact));
            }
        }
    }

    /// `SQABS` and `SQNEG` saturate at exactly one input, and it is the one an
    /// implementation that reuses [`abs`] silently gets wrong: `wrapping_abs`
    /// of the most negative value is itself.
    #[test]
    fn the_saturating_abs_and_neg_clamp_only_at_the_most_negative() {
        for a in 0..=255u32 {
            let value = a as u8 as i8;
            let x = u64::from(a);
            assert_eq!(
                sqabs(0, x),
                (
                    u64::from(value.saturating_abs() as u8),
                    value.checked_abs().is_none()
                )
            );
            assert_eq!(
                sqneg(0, x),
                (
                    u64::from(value.saturating_neg() as u8),
                    value.checked_neg().is_none()
                )
            );
        }
        // And at the doubleword, where the sign bit is the register's own.
        assert_eq!(sqabs(3, 1 << 63), (i64::MAX as u64, true));
        assert_eq!(sqneg(3, 1 << 63), (i64::MAX as u64, true));
        assert_eq!(sqneg(3, 1), ((-1i64) as u64, false));
    }

    /// The eight shifts, at every byte and every amount a byte can name that
    /// is not a no-op — evaluated against the same rule at sixteen bits, where
    /// nothing overflows and the clamp is explicit.
    #[test]
    fn the_shifts_round_before_they_shift_and_clamp_after() {
        for a in 0..=255u32 {
            for amount in -9i32..=9 {
                let x = u64::from(a);
                let b = u64::from(amount as i8 as u8);
                let signed = i32::from(a as u8 as i8);
                let unsigned = i32::from(a as u8);
                // The rounding constant belongs to the *right* shift only.
                let round = if amount < 0 { 1 << (-amount - 1) } else { 0 };
                let shift = |v: i32, round: i32| -> i32 {
                    if amount >= 0 {
                        if amount >= 31 { 0 } else { v << amount }
                    } else {
                        (v + round) >> (-amount).min(31)
                    }
                };

                let plain = shift(signed, 0);
                assert_eq!(
                    shift_reg(0, x, b, true, false, SatTo::Wrap).0,
                    u64::from(plain as u8),
                    "SSHL {a:#x} by {amount}"
                );
                let rounded = shift(signed, round);
                assert_eq!(
                    shift_reg(0, x, b, true, true, SatTo::Wrap).0,
                    u64::from(rounded as u8),
                    "SRSHL {a:#x} by {amount}"
                );
                assert_eq!(
                    shift_reg(0, x, b, true, false, SatTo::Signed),
                    (
                        u64::from(plain.clamp(-128, 127) as u8),
                        !(-128..=127).contains(&plain)
                    ),
                    "SQSHL {a:#x} by {amount}"
                );
                assert_eq!(
                    shift_reg(0, x, b, true, true, SatTo::Signed),
                    (
                        u64::from(rounded.clamp(-128, 127) as u8),
                        !(-128..=127).contains(&rounded)
                    ),
                    "SQRSHL {a:#x} by {amount}"
                );

                let plain = shift(unsigned, 0);
                assert_eq!(
                    shift_reg(0, x, b, false, false, SatTo::Wrap).0,
                    u64::from(plain as u8),
                    "USHL {a:#x} by {amount}"
                );
                let rounded = shift(unsigned, round);
                assert_eq!(
                    shift_reg(0, x, b, false, true, SatTo::Wrap).0,
                    u64::from(rounded as u8),
                    "URSHL {a:#x} by {amount}"
                );
                assert_eq!(
                    shift_reg(0, x, b, false, false, SatTo::Unsigned),
                    (
                        u64::from(plain.clamp(0, 255) as u8),
                        !(0..=255).contains(&plain)
                    ),
                    "UQSHL {a:#x} by {amount}"
                );
                assert_eq!(
                    shift_reg(0, x, b, false, true, SatTo::Unsigned),
                    (
                        u64::from(rounded.clamp(0, 255) as u8),
                        !(0..=255).contains(&rounded)
                    ),
                    "UQRSHL {a:#x} by {amount}"
                );
                // `SQSHLU` is the asymmetric one: a signed source, an unsigned
                // clamp, so every negative input saturates to zero.
                let plain = shift(signed, 0);
                assert_eq!(
                    shift_reg(0, x, b, true, false, SatTo::Unsigned),
                    (
                        u64::from(plain.clamp(0, 255) as u8),
                        !(0..=255).contains(&plain)
                    ),
                    "SQSHLU {a:#x} by {amount}"
                );
            }
        }
    }

    /// A shift amount is a *signed byte*, so -128 is reachable — and the
    /// rounding constant it names, `1 << 127`, is one place beyond what an
    /// `i128` holds. Everything past the element width gives the same answer,
    /// which is what makes clamping the amount safe rather than approximate.
    ///
    /// The three expectations below were all written wrong the first time, and
    /// the manual's pseudocode is why: the rounding constant is added to the
    /// *value* before the shift, so a rounding right shift of all-ones is
    /// **not** all-ones or zero by symmetry — it is whatever
    /// `(value + 2^(sh-1)) >> sh` comes to, which rounds -1 up to 0 and
    /// `0xffff_ffff_ffff_ffff` up to 1.
    #[test]
    fn a_shift_amount_of_minus_one_hundred_and_twenty_eight_is_reachable() {
        for esize in 0..4u32 {
            let bits = 8i32 << esize;
            for amount in [-128i32, -127, -100, -65, -64, -bits] {
                let b = u64::from(amount as i8 as u8);
                // Unsigned and not rounding: every bit has left the element.
                assert_eq!(
                    shift_reg(esize, u64::MAX, b, false, false, SatTo::Wrap).0,
                    0
                );
                // Signed and not rounding: -1 stays -1 however far it goes.
                assert_eq!(
                    shift_reg(esize, u64::MAX, b, true, false, SatTo::Wrap).0,
                    trunc(u64::MAX, esize)
                );
                // Signed *and* rounding: -1 rounds up to zero.
                assert_eq!(shift_reg(esize, u64::MAX, b, true, true, SatTo::Wrap).0, 0);
                // Unsigned and rounding: all-ones is one below the element's
                // modulus, so it rounds up to 1 at exactly the element width
                // and to 0 at anything wider.
                let want = u64::from(amount == -bits);
                assert_eq!(
                    shift_reg(esize, u64::MAX, b, false, true, SatTo::Wrap).0,
                    want,
                    "URSHL all-ones by {amount} at esize {esize}"
                );
            }
        }
        // A left shift by the whole width is zero when it wraps and a clamp
        // when it saturates — and zero either way if the value was zero.
        assert_eq!(shift_by(0, 1, 8, false, false, SatTo::Wrap), (0, false));
        assert_eq!(shift_by(0, 1, 8, true, false, SatTo::Signed), (127, true));
        assert_eq!(shift_by(0, 0, 100, true, false, SatTo::Signed), (0, false));
    }

    /// The halving adds keep the carry that an ordinary add throws away, and
    /// **none of them saturates** — which is why they are the six operations
    /// in this section that return a value rather than a pair.
    #[test]
    fn the_halving_adds_keep_the_carry_and_never_saturate() {
        for a in 0..=255u32 {
            for b in 0..=255u32 {
                let (x, y) = (u64::from(a), u64::from(b));
                let (sa, sb) = (i32::from(a as u8 as i8), i32::from(b as u8 as i8));
                let (ua, ub) = (i32::from(a as u8), i32::from(b as u8));
                assert_eq!(
                    halve(0, x, y, true, false, false),
                    ((sa + sb) >> 1) as u64 & 0xff
                );
                assert_eq!(
                    halve(0, x, y, false, false, false),
                    ((ua + ub) >> 1) as u64 & 0xff
                );
                assert_eq!(
                    halve(0, x, y, true, true, false),
                    ((sa + sb + 1) >> 1) as u64 & 0xff
                );
                assert_eq!(
                    halve(0, x, y, false, true, false),
                    ((ua + ub + 1) >> 1) as u64 & 0xff
                );
                assert_eq!(
                    halve(0, x, y, true, false, true),
                    ((sa - sb) >> 1) as u64 & 0xff
                );
                assert_eq!(
                    halve(0, x, y, false, false, true),
                    ((ua - ub) >> 1) as u64 & 0xff
                );
            }
        }
        // `UHADD` of two maxima is the case that shows the carry is kept:
        // 0xff + 0xff halved is 0xff, not 0x7f.
        assert_eq!(halve(0, 0xff, 0xff, false, false, false), 0xff);
    }

    /// The doubling multiplies, against the host's `saturating_mul` at twice
    /// the width — which is a genuine oracle for the *long* form, because
    /// `(a as i64).saturating_mul(b as i64).saturating_mul(2)` is exactly
    /// `SignedSat(2 * a * b, 64)` and is somebody else's code.
    #[test]
    fn the_doubling_multiplies_saturate_only_at_the_two_extremes() {
        const EDGES: &[i32] = &[
            0,
            1,
            -1,
            2,
            -2,
            32767,
            -32768,
            i32::MAX,
            i32::MIN,
            0x1234_5678,
            -0x1234_5678,
        ];
        // 16 -> 32, where `i64` holds the doubled product exactly and no
        // clamp happens before the architecture's own. Saturating the oracle
        // *first* is precisely the bug this expectation started with: the
        // pre-clamped product's top half is 0x7fff, which looks like an
        // honest result rather than a saturated one.
        for &a in EDGES {
            for &b in EDGES {
                let (a, b) = (a as i16, b as i16);
                let (x, y) = (u64::from(a as u16), u64::from(b as u16));
                let exact = 2 * i64::from(a) * i64::from(b);
                let bounded = |v: i64| -> (u64, bool) {
                    (
                        v.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as u32 as u64,
                        !(i64::from(i32::MIN)..=i64::from(i32::MAX)).contains(&v),
                    )
                };
                assert_eq!(sqdmull(1, x, y), bounded(exact));
                let narrow = |v: i64| -> (u64, bool) {
                    (
                        v.clamp(-32768, 32767) as i16 as u16 as u64,
                        !(-32768..=32767).contains(&v),
                    )
                };
                assert_eq!(sqdmulh(1, x, y, false), narrow(exact >> 16));
                assert_eq!(sqdmulh(1, x, y, true), narrow((exact + (1 << 15)) >> 16));
            }
        }
        // 32 -> 64, where only the pair of most negative values saturates.
        for &a in EDGES {
            for &b in EDGES {
                let (x, y) = (u64::from(a as u32), u64::from(b as u32));
                let exact = i64::from(a).saturating_mul(i64::from(b)).saturating_mul(2);
                assert_eq!(
                    sqdmull(2, x, y),
                    (exact as u64, a == i32::MIN && b == i32::MIN)
                );
            }
        }
    }

    /// The narrowing family, halfword to byte, at every source value and every
    /// shift — the group where a signed source and an unsigned destination
    /// meet, and where an implementation that reads the source at the
    /// destination's width loses the top half silently.
    #[test]
    fn the_narrowing_shifts_read_a_wide_source_and_bound_a_narrow_result() {
        for a in 0..=0xffffu32 {
            for shift in 0..=8u32 {
                let x = u64::from(a);
                let signed = i32::from(a as u16 as i16);
                let unsigned = i32::from(a as u16);
                let round = if shift > 0 { 1 << (shift - 1) } else { 0 };

                // SHRN / RSHRN: no bound at all.
                assert_eq!(
                    shift_narrow(0, x, shift, false, false, SatTo::Wrap).0,
                    ((unsigned >> shift) & 0xff) as u64
                );
                assert_eq!(
                    shift_narrow(0, x, shift, false, true, SatTo::Wrap).0,
                    (((unsigned + round) >> shift) & 0xff) as u64
                );
                // SQSHRN / SQRSHRN: a signed source, a signed byte.
                let exact = signed >> shift;
                assert_eq!(
                    shift_narrow(0, x, shift, true, false, SatTo::Signed),
                    (
                        exact.clamp(-128, 127) as u8 as u64,
                        !(-128..=127).contains(&exact)
                    )
                );
                let exact = (signed + round) >> shift;
                assert_eq!(
                    shift_narrow(0, x, shift, true, true, SatTo::Signed),
                    (
                        exact.clamp(-128, 127) as u8 as u64,
                        !(-128..=127).contains(&exact)
                    )
                );
                // UQSHRN: an unsigned source, an unsigned byte.
                let exact = unsigned >> shift;
                assert_eq!(
                    shift_narrow(0, x, shift, false, false, SatTo::Unsigned),
                    (exact.clamp(0, 255) as u64, exact > 255)
                );
                // SQSHRUN: the asymmetric one — signed in, unsigned out, so a
                // negative source saturates to zero rather than wrapping to a
                // large byte.
                let exact = signed >> shift;
                assert_eq!(
                    shift_narrow(0, x, shift, true, false, SatTo::Unsigned),
                    (exact.clamp(0, 255) as u64, !(0..=255).contains(&exact))
                );
            }
        }
    }

    /// The extract-narrows are the narrowing shifts by nothing, and the three
    /// of them differ only in how the source is read and where it is bounded.
    #[test]
    fn the_extract_narrows_are_a_narrowing_shift_of_zero() {
        // SQXTN: 0x1234 does not fit a signed byte.
        assert_eq!(
            shift_narrow(0, 0x1234, 0, true, false, SatTo::Signed),
            (127, true)
        );
        assert_eq!(
            shift_narrow(0, 0xffff, 0, true, false, SatTo::Signed),
            (0xff, false),
            "-1 fits"
        );
        // UQXTN reads the same bits as 65535, which does not fit.
        assert_eq!(
            shift_narrow(0, 0xffff, 0, false, false, SatTo::Unsigned),
            (0xff, true)
        );
        // SQXTUN reads -1 and clamps it to zero.
        assert_eq!(
            shift_narrow(0, 0xffff, 0, true, false, SatTo::Unsigned),
            (0, true)
        );
        // And at the widest pair, a doubleword down to a word.
        assert_eq!(
            shift_narrow(2, 0x8000_0000_0000_0000, 0, true, false, SatTo::Signed),
            (0x8000_0000, true)
        );
        assert_eq!(
            shift_narrow(2, 0xffff_ffff, 0, false, false, SatTo::Unsigned),
            (0xffff_ffff, false),
            "the widest word fits a word"
        );
    }

    /// The bounds themselves, at the doubleword, where the obvious
    /// `(1 << bits) - 1` overflows and a `u64` cannot hold the signed range.
    #[test]
    fn the_bounds_are_right_at_the_widest_element() {
        assert_eq!(SatTo::Signed.apply(i128::MAX, 3), (i64::MAX as u64, true));
        assert_eq!(SatTo::Signed.apply(i128::MIN, 3), (i64::MIN as u64, true));
        assert_eq!(SatTo::Unsigned.apply(i128::MAX, 3), (u64::MAX, true));
        assert_eq!(SatTo::Unsigned.apply(-1, 3), (0, true));
        assert_eq!(
            SatTo::Unsigned.apply(i128::from(u64::MAX), 3),
            (u64::MAX, false)
        );
        assert_eq!(
            SatTo::Signed.apply(i128::from(i64::MIN), 3),
            (i64::MIN as u64, false)
        );
        // Wrapping keeps the low bits and never reports a saturation, which is
        // what makes `SHRN` and `SSHL` members of the same family.
        assert_eq!(SatTo::Wrap.apply(0x1234, 0), (0x34, false));
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
