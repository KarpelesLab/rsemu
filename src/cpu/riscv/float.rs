//! Software IEEE-754 binary32 and binary64.
//!
//! `ROADMAP.md` §9.1 makes this a named deliverable rather than an assumption:
//! **guest floating point executed on host floating point cannot give
//! bit-identical results across hosts.** x86 and AArch64 disagree about NaN
//! payloads and about flush-to-zero, wasm canonicalises NaNs, and none of them
//! reports the guest's sticky exception flags. So every `F`/`D` instruction in
//! this core goes through the integer arithmetic below and nothing else — there
//! is no host-float fast path here at all, not even behind a flag, so there is
//! nothing that can silently be left on.
//!
//! # What is implemented
//!
//! * binary32 ([`B32`]) and binary64 ([`B64`]) as one generic implementation
//!   over [`Format`], so the two cannot drift apart.
//! * All five RISC-V rounding modes ([`Round`]).
//! * Subnormal inputs and results, computed exactly — never flushed.
//! * The five sticky flags in `fcsr.fflags` bit order ([`flags`]).
//! * `add`/`sub`/`mul`/`div`/`sqrt`/`fma`, comparisons, `min`/`max`,
//!   `classify`, format conversion, and both directions of integer conversion.
//!
//! # Why the results are reproducible
//!
//! Every operation is computed on a wide integer significand that is **exact**
//! before it is rounded once, at the end, by one private rounding step. There
//! is no double rounding and no host state: `mul` keeps the full 106-bit
//! product, `div` keeps the remainder as a sticky bit, `fma` keeps the whole
//! product and the addend in one 128-bit window, and alignment shifts fold what
//! they drop into a sticky bit rather than discarding it. Two hosts running this
//! code produce the same bits because the code contains no floating point.
//!
//! # Sources
//!
//! *The RISC-V Instruction Set Manual, Volume I: Unprivileged ISA* (CC-BY-4.0)
//! — the "F" and "D" standard extension chapters, in particular the sections
//! on rounding modes, on NaN generation and propagation (RISC-V returns the
//! **canonical** NaN and never propagates a payload, which is why there is no
//! payload logic here), and the FMIN/FMAX and FCVT descriptions. IEEE 754-2019
//! for the arithmetic itself. Tininess is detected **after** rounding, which is
//! the option the RISC-V profile of IEEE 754 selects.

/// The sticky exception flags, in `fcsr.fflags` bit order.
///
/// Named as the specification names them, and ORed into `fcsr` — never
/// cleared by an arithmetic instruction, only by a write to the CSR.
pub mod flags {
    /// Inexact.
    pub const NX: u32 = 0x01;
    /// Underflow: the result is tiny after rounding, and inexact.
    pub const UF: u32 = 0x02;
    /// Overflow.
    pub const OF: u32 = 0x04;
    /// Divide by zero.
    pub const DZ: u32 = 0x08;
    /// Invalid operation.
    pub const NV: u32 = 0x10;
}

/// A rounding mode, as `fcsr.frm` and an instruction's `rm` field encode it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Round {
    /// Round to nearest, ties to even (`RNE`, 0). The reset default.
    Rne,
    /// Round towards zero (`RTZ`, 1).
    Rtz,
    /// Round down, towards minus infinity (`RDN`, 2).
    Rdn,
    /// Round up, towards plus infinity (`RUP`, 3).
    Rup,
    /// Round to nearest, ties away from zero (`RMM`, 4).
    Rmm,
}

impl Round {
    /// Decode a three-bit `rm` field, or `None` for a reserved encoding.
    ///
    /// 5 and 6 are reserved and 7 means "use `fcsr.frm`", which the caller has
    /// to resolve before it gets here — an instruction naming a reserved mode
    /// is illegal, and `None` is how that reaches the decoder.
    #[must_use]
    pub const fn from_bits(bits: u32) -> Option<Round> {
        match bits {
            0 => Some(Round::Rne),
            1 => Some(Round::Rtz),
            2 => Some(Round::Rdn),
            3 => Some(Round::Rup),
            4 => Some(Round::Rmm),
            _ => None,
        }
    }

    /// The `rm` encoding of this mode.
    #[must_use]
    pub const fn bits(self) -> u32 {
        match self {
            Round::Rne => 0,
            Round::Rtz => 1,
            Round::Rdn => 2,
            Round::Rup => 3,
            Round::Rmm => 4,
        }
    }
}

/// A binary interchange format, described by its two field widths.
///
/// One implementation serves both formats: every routine below is generic over
/// this trait and works on the raw bit pattern held in a `u64`, with a
/// binary32 value in the low 32 bits.
pub trait Format: Copy {
    /// Width of the trailing significand field: 23 or 52.
    const SIG_BITS: u32;
    /// Width of the exponent field: 8 or 11.
    const EXP_BITS: u32;

    /// Total width of the format in bits.
    const BITS: u32 = Self::SIG_BITS + Self::EXP_BITS + 1;
    /// Exponent bias.
    const BIAS: i32 = (1i32 << (Self::EXP_BITS - 1)) - 1;
    /// The smallest exponent of a normal number.
    const EMIN: i32 = 1 - Self::BIAS;
    /// The largest exponent of a finite number.
    const EMAX: i32 = Self::BIAS;
    /// A mask of every bit the format occupies.
    const MASK: u64 = u64::MAX >> (64 - Self::BITS);
    /// The sign bit.
    const SIGN: u64 = 1u64 << (Self::BITS - 1);
    /// A mask of the trailing significand field.
    const SIG_MASK: u64 = (1u64 << Self::SIG_BITS) - 1;
    /// The all-ones exponent field, unshifted.
    const EXP_FIELD_MAX: u64 = (1u64 << Self::EXP_BITS) - 1;
    /// The significand bit that distinguishes a quiet NaN from a signaling one.
    const QUIET: u64 = 1u64 << (Self::SIG_BITS - 1);
    /// Positive infinity.
    const INF: u64 = Self::EXP_FIELD_MAX << Self::SIG_BITS;
    /// The single NaN this format ever produces (Volume I, "NaN Generation and
    /// Propagation": RISC-V returns the canonical NaN, never a payload).
    const CANONICAL_NAN: u64 = Self::INF | Self::QUIET;
    /// The largest finite magnitude.
    const MAX_FINITE: u64 = Self::INF - 1;
}

/// IEEE 754 binary32 — the `F` extension's format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct B32;

impl Format for B32 {
    const SIG_BITS: u32 = 23;
    const EXP_BITS: u32 = 8;
}

/// IEEE 754 binary64 — the `D` extension's format.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct B64;

impl Format for B64 {
    const SIG_BITS: u32 = 52;
    const EXP_BITS: u32 = 11;
}

/// How many guard bits [`round_pack`] keeps below the result's last place.
///
/// Three — guard, round and sticky — the classical minimum for a correctly
/// rounded result, and one more than addition strictly needs so a one-bit
/// renormalisation never has to invent a bit.
const EXTRA: u32 = 3;

/// What kind of number a bit pattern is.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Class {
    Zero,
    Finite,
    Inf,
    Nan,
}

/// A bit pattern taken apart.
///
/// `Finite` values satisfy `value = frac * 2^(exp - SIG_BITS)` with no hidden
/// bit implied: a normal number has `frac >= 2^SIG_BITS`, a subnormal has
/// `exp = EMIN` and `frac < 2^SIG_BITS`. One representation for both is what
/// makes alignment in [`add_impl`] a plain exponent difference.
#[derive(Debug, Clone, Copy)]
struct Parts {
    sign: bool,
    exp: i32,
    frac: u64,
    class: Class,
    /// Set only for a signaling NaN, which raises invalid wherever it appears.
    snan: bool,
}

/// Take a bit pattern apart.
fn unpack<F: Format>(bits: u64) -> Parts {
    let bits = bits & F::MASK;
    let sign = bits & F::SIGN != 0;
    let field = (bits >> F::SIG_BITS) & F::EXP_FIELD_MAX;
    let frac = bits & F::SIG_MASK;
    if field == F::EXP_FIELD_MAX {
        if frac == 0 {
            Parts {
                sign,
                exp: 0,
                frac: 0,
                class: Class::Inf,
                snan: false,
            }
        } else {
            Parts {
                sign,
                exp: 0,
                frac,
                class: Class::Nan,
                snan: frac & F::QUIET == 0,
            }
        }
    } else if field == 0 {
        Parts {
            sign,
            exp: F::EMIN,
            frac,
            class: if frac == 0 {
                Class::Zero
            } else {
                Class::Finite
            },
            snan: false,
        }
    } else {
        Parts {
            sign,
            exp: field as i32 - F::BIAS,
            frac: frac | (1u64 << F::SIG_BITS),
            class: Class::Finite,
            snan: false,
        }
    }
}

/// Shift `frac` up until its top bit sits at `SIG_BITS`, for the operations
/// that need a normalised divisor or radicand.
fn normalize<F: Format>(p: &mut Parts) {
    if p.class != Class::Finite || p.frac == 0 {
        return;
    }
    let msb = 63 - p.frac.leading_zeros();
    if msb < F::SIG_BITS {
        let n = F::SIG_BITS - msb;
        p.frac <<= n;
        p.exp -= n as i32;
    }
}

/// A signed zero.
fn zero<F: Format>(sign: bool) -> u64 {
    if sign { F::SIGN } else { 0 }
}

/// A signed infinity.
fn inf<F: Format>(sign: bool) -> u64 {
    if sign { F::SIGN | F::INF } else { F::INF }
}

/// Right shift, folding everything shifted out into bit 0.
///
/// The sticky bit is an OR rather than an add precisely so it can never carry
/// into a meaningful bit: it records *that* something was dropped, which is
/// all any rounding rule needs to know about it.
fn shr_sticky(v: u128, n: u32) -> u128 {
    if n == 0 {
        return v;
    }
    if n >= 128 {
        return u128::from(v != 0);
    }
    let lost = v & ((1u128 << n) - 1);
    (v >> n) | u128::from(lost != 0)
}

/// The 64-bit form of [`shr_sticky`].
fn shr_sticky64(v: u64, n: u32) -> u64 {
    if n == 0 {
        return v;
    }
    if n >= 64 {
        return u64::from(v != 0);
    }
    let lost = v & ((1u64 << n) - 1);
    (v >> n) | u64::from(lost != 0)
}

/// If any operand is a NaN, produce the result the specification demands.
///
/// Always the canonical NaN — RISC-V does not propagate payloads — with
/// invalid raised only for a *signaling* input.
fn nan_result<F: Format>(ps: &[Parts]) -> Option<(u64, u32)> {
    let mut any = false;
    let mut signaling = false;
    for p in ps {
        if p.class == Class::Nan {
            any = true;
            signaling |= p.snan;
        }
    }
    if any {
        Some((F::CANONICAL_NAN, if signaling { flags::NV } else { 0 }))
    } else {
        None
    }
}

/// Round a significand to the format and assemble the result.
///
/// `value = sig * 2^(exp - SIG_BITS - EXTRA)`, with `sig` normalised so its
/// top bit is at `SIG_BITS + EXTRA` and everything already lost folded into
/// bit 0.
fn round_pack<F: Format>(sign: bool, mut exp: i32, mut sig: u64, rm: Round) -> (u64, u32) {
    if sig == 0 {
        return (zero::<F>(sign), 0);
    }
    // Below EMIN the result is subnormal: it is expressed at the fixed
    // exponent EMIN with fewer significant bits, so the precision is lost here
    // rather than pretended away. Never flushed to zero.
    if exp < F::EMIN {
        let n = (F::EMIN - exp) as u32;
        sig = shr_sticky64(sig, n);
        exp = F::EMIN;
    }
    let rem = sig & ((1 << EXTRA) - 1);
    let half = 1u64 << (EXTRA - 1);
    let mut keep = sig >> EXTRA;
    let inexact = rem != 0;
    let up = match rm {
        Round::Rne => rem > half || (rem == half && keep & 1 != 0),
        Round::Rtz => false,
        Round::Rdn => inexact && sign,
        Round::Rup => inexact && !sign,
        Round::Rmm => rem >= half,
    };
    if up {
        // Cannot wrap: `keep` is at most 2^(SIG_BITS+1) before the increment.
        keep += 1;
        if keep == 1u64 << (F::SIG_BITS + 1) {
            keep >>= 1;
            exp += 1;
        }
    }
    if exp > F::EMAX {
        // Which way overflow goes is a property of the rounding mode: only the
        // two round-to-nearest modes always reach infinity.
        let to_inf = match rm {
            Round::Rne | Round::Rmm => true,
            Round::Rtz => false,
            Round::Rdn => sign,
            Round::Rup => !sign,
        };
        let mag = if to_inf { F::INF } else { F::MAX_FINITE };
        return (zero::<F>(sign) | mag, flags::OF | flags::NX);
    }
    let biased = if keep >> F::SIG_BITS != 0 {
        (exp + F::BIAS) as u64
    } else {
        // No hidden bit left: the result is subnormal, or zero.
        0
    };
    let bits = zero::<F>(sign) | (biased << F::SIG_BITS) | (keep & F::SIG_MASK);
    // Tininess after rounding, which is the detection RISC-V selects: a result
    // that rounded up to the smallest normal did not underflow.
    let mut f = if inexact { flags::NX } else { 0 };
    if biased == 0 && inexact {
        f |= flags::UF;
    }
    (bits, f)
}

/// Round a wide significand and assemble the result.
///
/// `value = sig * 2^(exp - SIG_BITS - 64)`: a fixed-point number with 64
/// fraction bits below the format's significand. Every arithmetic routine
/// below produces this shape, so there is exactly one rounding step in the
/// file and no operation can round twice.
fn pack_wide<F: Format>(sign: bool, mut exp: i32, mut sig: u128, rm: Round) -> (u64, u32) {
    if sig == 0 {
        return (zero::<F>(sign), 0);
    }
    let top = 64 + F::SIG_BITS;
    let msb = 127 - sig.leading_zeros();
    if msb > top {
        let n = msb - top;
        sig = shr_sticky(sig, n);
        exp += n as i32;
    } else {
        let n = top - msb;
        sig <<= n;
        exp -= n as i32;
    }
    let narrow = shr_sticky(sig, 64 - EXTRA) as u64;
    round_pack::<F>(sign, exp, narrow, rm)
}

// ---------------------------------------------------------------------------
// Arithmetic
// ---------------------------------------------------------------------------

/// `a + b`.
pub fn add<F: Format>(a: u64, b: u64, rm: Round) -> (u64, u32) {
    add_impl::<F>(a, b, false, rm)
}

/// `a - b`.
pub fn sub<F: Format>(a: u64, b: u64, rm: Round) -> (u64, u32) {
    add_impl::<F>(a, b, true, rm)
}

/// Addition, with `b` optionally negated first.
///
/// Both significands are widened to 128 bits with 64 fraction bits before they
/// are aligned, so an alignment shift of up to 64 places — which covers every
/// case where cancellation can occur — is *exact*. Only a longer shift folds
/// into a sticky bit, and by then the smaller operand is more than 2^63 times
/// smaller and could not have affected anything else.
fn add_impl<F: Format>(a: u64, b: u64, negate: bool, rm: Round) -> (u64, u32) {
    let pa = unpack::<F>(a);
    let mut pb = unpack::<F>(b);
    if negate {
        pb.sign = !pb.sign;
    }
    if let Some(r) = nan_result::<F>(&[pa, pb]) {
        return r;
    }
    match (pa.class, pb.class) {
        (Class::Inf, Class::Inf) => {
            if pa.sign == pb.sign {
                (inf::<F>(pa.sign), 0)
            } else {
                (F::CANONICAL_NAN, flags::NV)
            }
        }
        (Class::Inf, _) => (inf::<F>(pa.sign), 0),
        (_, Class::Inf) => (inf::<F>(pb.sign), 0),
        (Class::Zero, Class::Zero) => {
            // x + (-x) is +0 in every mode but round-down, where it is -0.
            let sign = if pa.sign == pb.sign {
                pa.sign
            } else {
                rm == Round::Rdn
            };
            (zero::<F>(sign), 0)
        }
        (Class::Zero, _) => (zero::<F>(pb.sign) | (b & F::MASK & !F::SIGN), 0),
        (_, Class::Zero) => (a & F::MASK, 0),
        _ => {
            let (hi, lo) = if pa.exp >= pb.exp { (pa, pb) } else { (pb, pa) };
            let sig_hi = u128::from(hi.frac) << 64;
            let sig_lo = shr_sticky(u128::from(lo.frac) << 64, (hi.exp - lo.exp) as u32);
            let (sign, sig) = if hi.sign == lo.sign {
                (hi.sign, sig_hi + sig_lo)
            } else if sig_hi >= sig_lo {
                (hi.sign, sig_hi - sig_lo)
            } else {
                (lo.sign, sig_lo - sig_hi)
            };
            if sig == 0 {
                return (zero::<F>(rm == Round::Rdn), 0);
            }
            pack_wide::<F>(sign, hi.exp, sig, rm)
        }
    }
}

/// `a * b`.
pub fn mul<F: Format>(a: u64, b: u64, rm: Round) -> (u64, u32) {
    let pa = unpack::<F>(a);
    let pb = unpack::<F>(b);
    if let Some(r) = nan_result::<F>(&[pa, pb]) {
        return r;
    }
    let sign = pa.sign ^ pb.sign;
    match (pa.class, pb.class) {
        // Zero times infinity is the classic invalid operation, in either
        // order, and the only way multiplication produces a NaN.
        (Class::Inf, Class::Zero) | (Class::Zero, Class::Inf) => (F::CANONICAL_NAN, flags::NV),
        (Class::Inf, _) | (_, Class::Inf) => (inf::<F>(sign), 0),
        (Class::Zero, _) | (_, Class::Zero) => (zero::<F>(sign), 0),
        _ => {
            // The full product is kept: 2x53 bits fits a u128 with room, so
            // nothing is dropped before the single rounding step.
            let sig = u128::from(pa.frac) * u128::from(pb.frac);
            pack_wide::<F>(sign, pa.exp + pb.exp - F::SIG_BITS as i32 + 64, sig, rm)
        }
    }
}

/// `a / b`.
pub fn div<F: Format>(a: u64, b: u64, rm: Round) -> (u64, u32) {
    let mut pa = unpack::<F>(a);
    let mut pb = unpack::<F>(b);
    if let Some(r) = nan_result::<F>(&[pa, pb]) {
        return r;
    }
    let sign = pa.sign ^ pb.sign;
    match (pa.class, pb.class) {
        (Class::Inf, Class::Inf) | (Class::Zero, Class::Zero) => (F::CANONICAL_NAN, flags::NV),
        (Class::Inf, _) => (inf::<F>(sign), 0),
        (_, Class::Inf) => (zero::<F>(sign), 0),
        (_, Class::Zero) => (inf::<F>(sign), flags::DZ),
        (Class::Zero, _) => (zero::<F>(sign), 0),
        _ => {
            normalize::<F>(&mut pa);
            normalize::<F>(&mut pb);
            let num = u128::from(pa.frac) << 64;
            let den = u128::from(pb.frac);
            let mut q = num / den;
            if num % den != 0 {
                q |= 1;
            }
            pack_wide::<F>(sign, pa.exp - pb.exp + F::SIG_BITS as i32, q, rm)
        }
    }
}

/// The integer square root of a 128-bit value, and whether it was exact.
///
/// Restoring, two bits of radicand per bit of root, so it needs no division —
/// and no floating point, which is the whole point of the file.
fn isqrt(x: u128) -> (u128, bool) {
    if x == 0 {
        return (0, false);
    }
    let mut rem: u128 = 0;
    let mut root: u128 = 0;
    // Start at the highest even bit position at or below the top of `x`.
    let mut shift = (127 - x.leading_zeros()) & !1;
    loop {
        rem = (rem << 2) | ((x >> shift) & 3);
        root <<= 1;
        let trial = (root << 1) | 1;
        if rem >= trial {
            rem -= trial;
            root |= 1;
        }
        if shift == 0 {
            break;
        }
        shift -= 2;
    }
    (root, rem != 0)
}

/// The square root of `a`.
pub fn sqrt<F: Format>(a: u64, rm: Round) -> (u64, u32) {
    let mut pa = unpack::<F>(a);
    if let Some(r) = nan_result::<F>(&[pa]) {
        return r;
    }
    match pa.class {
        Class::Zero => (zero::<F>(pa.sign), 0),
        // Minus infinity and every other negative is invalid; -0 is not, and
        // its square root is -0 (IEEE 754 clause 5.4.1).
        _ if pa.sign => (F::CANONICAL_NAN, flags::NV),
        Class::Inf => (F::INF, 0),
        _ => {
            normalize::<F>(&mut pa);
            let mut t = pa.exp - F::SIG_BITS as i32;
            let mut frac = pa.frac;
            if t & 1 != 0 {
                // The exponent must be even to halve it; moving one power of
                // two into the significand keeps the value identical.
                frac <<= 1;
                t -= 1;
            }
            let (root, rest) = isqrt(u128::from(frac) << 64);
            let mut sig = root;
            if rest {
                sig |= 1;
            }
            pack_wide::<F>(false, t / 2 + F::SIG_BITS as i32 + 32, sig, rm)
        }
    }
}

/// `a * b + c`, with one rounding at the end — a true fused multiply-add.
///
/// The exact 2x53-bit product and the addend share one 128-bit window, placed
/// so that every exponent difference small enough for cancellation to matter
/// is aligned without loss. A larger difference means one term dominates by
/// more than the format's precision, and what falls out of the window can then
/// only ever be a sticky bit.
pub fn fma<F: Format>(a: u64, b: u64, c: u64, rm: Round) -> (u64, u32) {
    let pa = unpack::<F>(a);
    let pb = unpack::<F>(b);
    let pc = unpack::<F>(c);
    // 0 * inf is invalid even when the addend is a NaN, which is why the
    // multiply is checked before the NaN sweep.
    if matches!(
        (pa.class, pb.class),
        (Class::Inf, Class::Zero) | (Class::Zero, Class::Inf)
    ) {
        return (F::CANONICAL_NAN, flags::NV);
    }
    if let Some(r) = nan_result::<F>(&[pa, pb, pc]) {
        return r;
    }
    let psign = pa.sign ^ pb.sign;
    if pa.class == Class::Inf || pb.class == Class::Inf {
        if pc.class == Class::Inf && pc.sign != psign {
            return (F::CANONICAL_NAN, flags::NV);
        }
        return (inf::<F>(psign), 0);
    }
    if pc.class == Class::Inf {
        return (inf::<F>(pc.sign), 0);
    }
    if pa.class == Class::Zero || pb.class == Class::Zero {
        if pc.class == Class::Zero {
            let sign = if psign == pc.sign {
                psign
            } else {
                rm == Round::Rdn
            };
            return (zero::<F>(sign), 0);
        }
        return (zero::<F>(pc.sign) | (c & F::MASK & !F::SIGN), 0);
    }
    if pc.class == Class::Zero {
        return mul::<F>(a, b, rm);
    }

    // The product carries ten spare low bits so a small alignment shift — the
    // only case where cancellation can expose them — is exact.
    const SLACK: u32 = 10;
    let mut psig = (u128::from(pa.frac) * u128::from(pb.frac)) << SLACK;
    let mut pexp = pa.exp + pb.exp - F::SIG_BITS as i32 + 64 - SLACK as i32;
    let mut csig = u128::from(pc.frac) << 64;
    let cexp = pc.exp;
    let exp = if pexp > cexp {
        csig = shr_sticky(csig, (pexp - cexp) as u32);
        pexp
    } else {
        psig = shr_sticky(psig, (cexp - pexp) as u32);
        pexp = cexp;
        pexp
    };
    let (sign, sig) = if psign == pc.sign {
        (psign, psig + csig)
    } else if psig >= csig {
        (psign, psig - csig)
    } else {
        (pc.sign, csig - psig)
    };
    if sig == 0 {
        return (zero::<F>(rm == Round::Rdn), 0);
    }
    pack_wide::<F>(sign, exp, sig, rm)
}

// ---------------------------------------------------------------------------
// Comparison, sign and classification
// ---------------------------------------------------------------------------

/// The ordering of two non-NaN values, as sign and magnitude.
fn ordered_cmp<F: Format>(a: u64, b: u64) -> core::cmp::Ordering {
    use core::cmp::Ordering;
    let a = a & F::MASK;
    let b = b & F::MASK;
    let (sa, sb) = (a & F::SIGN != 0, b & F::SIGN != 0);
    let (ma, mb) = (a & !F::SIGN & F::MASK, b & !F::SIGN & F::MASK);
    if ma == 0 && mb == 0 {
        // +0 and -0 compare equal, which is the one place sign is ignored.
        return Ordering::Equal;
    }
    match (sa, sb) {
        (false, true) => Ordering::Greater,
        (true, false) => Ordering::Less,
        (false, false) => ma.cmp(&mb),
        (true, true) => mb.cmp(&ma),
    }
}

/// `a == b`, the quiet comparison: only a signaling NaN raises invalid.
pub fn eq<F: Format>(a: u64, b: u64) -> (bool, u32) {
    let (pa, pb) = (unpack::<F>(a), unpack::<F>(b));
    if pa.class == Class::Nan || pb.class == Class::Nan {
        let f = if pa.snan || pb.snan { flags::NV } else { 0 };
        return (false, f);
    }
    (ordered_cmp::<F>(a, b) == core::cmp::Ordering::Equal, 0)
}

/// `a < b`, the signaling comparison: **any** NaN raises invalid.
pub fn lt<F: Format>(a: u64, b: u64) -> (bool, u32) {
    let (pa, pb) = (unpack::<F>(a), unpack::<F>(b));
    if pa.class == Class::Nan || pb.class == Class::Nan {
        return (false, flags::NV);
    }
    (ordered_cmp::<F>(a, b) == core::cmp::Ordering::Less, 0)
}

/// `a <= b`, the signaling comparison.
pub fn le<F: Format>(a: u64, b: u64) -> (bool, u32) {
    let (pa, pb) = (unpack::<F>(a), unpack::<F>(b));
    if pa.class == Class::Nan || pb.class == Class::Nan {
        return (false, flags::NV);
    }
    (ordered_cmp::<F>(a, b) != core::cmp::Ordering::Greater, 0)
}

/// `FMIN`: the smaller operand, or the non-NaN one.
pub fn min<F: Format>(a: u64, b: u64) -> (u64, u32) {
    min_max::<F>(a, b, true)
}

/// `FMAX`: the larger operand, or the non-NaN one.
pub fn max<F: Format>(a: u64, b: u64) -> (u64, u32) {
    min_max::<F>(a, b, false)
}

/// The shared body of [`min`] and [`max`].
///
/// Volume I: if both inputs are NaN the result is the canonical NaN, if exactly
/// one is the result is the other operand, and a signaling input raises invalid
/// either way. Minus zero is treated as smaller than plus zero, which a plain
/// magnitude comparison would get wrong.
fn min_max<F: Format>(a: u64, b: u64, want_min: bool) -> (u64, u32) {
    use core::cmp::Ordering;
    let (pa, pb) = (unpack::<F>(a), unpack::<F>(b));
    let f = if pa.snan || pb.snan { flags::NV } else { 0 };
    match (pa.class == Class::Nan, pb.class == Class::Nan) {
        (true, true) => return (F::CANONICAL_NAN, f),
        (true, false) => return (b & F::MASK, f),
        (false, true) => return (a & F::MASK, f),
        (false, false) => {}
    }
    let ord = if pa.class == Class::Zero && pb.class == Class::Zero {
        match (pa.sign, pb.sign) {
            (true, false) => Ordering::Less,
            (false, true) => Ordering::Greater,
            _ => Ordering::Equal,
        }
    } else {
        ordered_cmp::<F>(a, b)
    };
    let take_a = if want_min {
        ord != Ordering::Greater
    } else {
        ord != Ordering::Less
    };
    ((if take_a { a } else { b }) & F::MASK, f)
}

/// The `FCLASS` bit mask for a value.
///
/// Volume I, "Single-Precision Floating-Point Classify Instruction": bit 0 is
/// minus infinity and bit 9 is a quiet NaN, in that order.
pub fn classify<F: Format>(a: u64) -> u64 {
    let p = unpack::<F>(a);
    match p.class {
        Class::Nan => {
            if p.snan {
                1 << 8
            } else {
                1 << 9
            }
        }
        Class::Inf => {
            if p.sign {
                1 << 0
            } else {
                1 << 7
            }
        }
        Class::Zero => {
            if p.sign {
                1 << 3
            } else {
                1 << 4
            }
        }
        Class::Finite => {
            let subnormal = p.frac < (1u64 << F::SIG_BITS);
            match (p.sign, subnormal) {
                (true, false) => 1 << 1,
                (true, true) => 1 << 2,
                (false, true) => 1 << 5,
                (false, false) => 1 << 6,
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Conversion
// ---------------------------------------------------------------------------

/// Convert between the two binary formats.
pub fn convert<A: Format, B: Format>(bits: u64, rm: Round) -> (u64, u32) {
    let p = unpack::<A>(bits);
    match p.class {
        Class::Nan => (B::CANONICAL_NAN, if p.snan { flags::NV } else { 0 }),
        Class::Inf => (inf::<B>(p.sign), 0),
        Class::Zero => (zero::<B>(p.sign), 0),
        Class::Finite => {
            let sig = u128::from(p.frac) << 64;
            let exp = p.exp - A::SIG_BITS as i32 + B::SIG_BITS as i32;
            pack_wide::<B>(p.sign, exp, sig, rm)
        }
    }
}

/// Round a value to an integer, reporting sign, magnitude and inexactness.
///
/// Shared by every `FCVT` to an integer: the magnitude comes back as a `u128`
/// so the range check belongs to the caller, which is the only part that
/// differs between the four integer widths. `None` means NaN or infinity.
fn to_integer<F: Format>(bits: u64, rm: Round) -> Option<(bool, u128, bool)> {
    let p = unpack::<F>(bits);
    match p.class {
        Class::Nan | Class::Inf => None,
        Class::Zero => Some((p.sign, 0, false)),
        Class::Finite => {
            // Scale by 2^EXTRA so the same rounding rules as `round_pack`
            // apply, then shift the guard bits away.
            let shift = p.exp - F::SIG_BITS as i32 + EXTRA as i32;
            let scaled: u128 = if shift >= 0 {
                if shift > 70 {
                    // Far past any integer width; the caller saturates.
                    return Some((p.sign, u128::MAX, false));
                }
                u128::from(p.frac) << shift
            } else {
                shr_sticky(u128::from(p.frac), (-shift) as u32)
            };
            let rem = scaled & ((1 << EXTRA) - 1);
            let half = 1u128 << (EXTRA - 1);
            let mut mag = scaled >> EXTRA;
            let inexact = rem != 0;
            let up = match rm {
                Round::Rne => rem > half || (rem == half && mag & 1 != 0),
                Round::Rtz => false,
                Round::Rdn => inexact && p.sign,
                Round::Rup => inexact && !p.sign,
                Round::Rmm => rem >= half,
            };
            if up {
                mag += 1;
            }
            Some((p.sign, mag, inexact))
        }
    }
}

/// Sign-extend a `bits`-wide two's complement value to 64 bits.
fn sign_extend(v: u64, bits: u32) -> i64 {
    if bits >= 64 {
        v as i64
    } else {
        ((v << (64 - bits)) as i64) >> (64 - bits)
    }
}

/// `FCVT.W.*` / `FCVT.L.*`: to a signed integer `bits` wide, sign-extended.
///
/// Out-of-range and NaN inputs saturate and raise invalid, which is what the
/// specification requires instead of a trap — NaN gives the *most positive*
/// value.
pub fn to_signed<F: Format>(value: u64, bits: u32, rm: Round) -> (i64, u32) {
    let max: u128 = (1u128 << (bits - 1)) - 1;
    let min_mag: u128 = 1u128 << (bits - 1);
    let Some((sign, mag, inexact)) = to_integer::<F>(value, rm) else {
        let p = unpack::<F>(value);
        let out = if p.class == Class::Nan || !p.sign {
            max as i64
        } else {
            sign_extend(min_mag as u64, bits)
        };
        return (out, flags::NV);
    };
    if sign {
        if mag > min_mag {
            (sign_extend(min_mag as u64, bits), flags::NV)
        } else {
            let neg = (mag as u64).wrapping_neg();
            (sign_extend(neg, bits), if inexact { flags::NX } else { 0 })
        }
    } else if mag > max {
        (max as i64, flags::NV)
    } else {
        (mag as i64, if inexact { flags::NX } else { 0 })
    }
}

/// `FCVT.WU.*` / `FCVT.LU.*`: to an unsigned integer `bits` wide.
///
/// A negative input that *rounds* to zero is 0 and merely inexact; anything
/// that rounds below zero is invalid and saturates to 0.
pub fn to_unsigned<F: Format>(value: u64, bits: u32, rm: Round) -> (u64, u32) {
    let max: u128 = if bits >= 64 {
        u128::from(u64::MAX)
    } else {
        (1u128 << bits) - 1
    };
    let Some((sign, mag, inexact)) = to_integer::<F>(value, rm) else {
        let p = unpack::<F>(value);
        let out = if p.class == Class::Nan || !p.sign {
            max as u64
        } else {
            0
        };
        return (out, flags::NV);
    };
    if sign && mag != 0 {
        (0, flags::NV)
    } else if mag > max {
        (max as u64, flags::NV)
    } else {
        (mag as u64, if inexact { flags::NX } else { 0 })
    }
}

/// `FCVT.*.W` / `FCVT.*.L`: from a signed integer `bits` wide.
pub fn from_signed<F: Format>(value: i64, bits: u32, rm: Round) -> (u64, u32) {
    let v = sign_extend(value as u64, bits);
    from_magnitude::<F>(v < 0, u128::from(v.unsigned_abs()), rm)
}

/// `FCVT.*.WU` / `FCVT.*.LU`: from an unsigned integer `bits` wide.
pub fn from_unsigned<F: Format>(value: u64, bits: u32, rm: Round) -> (u64, u32) {
    let v = if bits >= 64 {
        value
    } else {
        value & ((1u64 << bits) - 1)
    };
    from_magnitude::<F>(false, u128::from(v), rm)
}

/// The shared body of the two integer-to-float conversions.
fn from_magnitude<F: Format>(sign: bool, mag: u128, rm: Round) -> (u64, u32) {
    if mag == 0 {
        return (zero::<F>(false), 0);
    }
    pack_wide::<F>(sign, F::SIG_BITS as i32 + 64, mag, rm)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Host `f64` bits, used only to write readable expectations — the
    /// implementation itself never touches host floating point.
    fn d(v: f64) -> u64 {
        v.to_bits()
    }

    fn s(v: f32) -> u64 {
        u64::from(v.to_bits())
    }

    #[test]
    fn addition_is_exact_where_it_should_be() {
        assert_eq!(add::<B64>(d(1.0), d(2.0), Round::Rne), (d(3.0), 0));
        assert_eq!(add::<B64>(d(0.5), d(0.25), Round::Rne), (d(0.75), 0));
        assert_eq!(sub::<B64>(d(1.0), d(1.0), Round::Rne), (d(0.0), 0));
        // x - x is -0 only when rounding down (IEEE 754 clause 6.3).
        assert_eq!(sub::<B64>(d(1.0), d(1.0), Round::Rdn), (d(-0.0), 0));
    }

    #[test]
    fn addition_rounds_to_nearest_even() {
        // 1 + 2^-53 is exactly halfway; ties-to-even keeps 1.0.
        let tiny = 0x3ca0_0000_0000_0000u64;
        let (v, f) = add::<B64>(d(1.0), tiny, Round::Rne);
        assert_eq!(v, d(1.0));
        assert_eq!(f, flags::NX);
        let (v, _) = add::<B64>(d(1.0), tiny, Round::Rup);
        assert_eq!(v, d(1.0) + 1);
    }

    #[test]
    fn cancellation_is_exact() {
        // The case a sticky-only aligner gets wrong.
        let a = d(1.0);
        let b = 0x3ca0_0000_0000_0001u64;
        let (v, f) = sub::<B64>(a, b, Round::Rne);
        assert_eq!(f, flags::NX);
        assert!(v < a);
    }

    #[test]
    fn subnormals_survive() {
        let min_sub = 1u64; // 2^-1074
        assert_eq!(add::<B64>(min_sub, min_sub, Round::Rne), (2, 0));
        // Halving the smallest subnormal underflows to zero, inexactly.
        let (v, f) = mul::<B64>(min_sub, d(0.5), Round::Rne);
        assert_eq!(v, 0);
        assert_eq!(f, flags::NX | flags::UF);
    }

    #[test]
    fn multiplication_keeps_the_whole_product() {
        assert_eq!(mul::<B64>(d(3.0), d(7.0), Round::Rne), (d(21.0), 0));
        assert_eq!(mul::<B32>(s(3.0), s(0.5), Round::Rne), (s(1.5), 0));
        let (v, f) = mul::<B64>(d(0.0), d(f64::INFINITY), Round::Rne);
        assert_eq!(v, B64::CANONICAL_NAN);
        assert_eq!(f, flags::NV);
    }

    #[test]
    fn division_reports_divide_by_zero() {
        assert_eq!(div::<B64>(d(1.0), d(2.0), Round::Rne), (d(0.5), 0));
        let (v, f) = div::<B64>(d(1.0), d(0.0), Round::Rne);
        assert_eq!(v, d(f64::INFINITY));
        assert_eq!(f, flags::DZ);
        let (v, f) = div::<B64>(d(0.0), d(0.0), Round::Rne);
        assert_eq!(v, B64::CANONICAL_NAN);
        assert_eq!(f, flags::NV);
        let (v, f) = div::<B64>(d(1.0), d(3.0), Round::Rne);
        assert_eq!(v, d(1.0f64 / 3.0));
        assert_eq!(f, flags::NX);
    }

    #[test]
    fn square_root_is_correctly_rounded() {
        assert_eq!(sqrt::<B64>(d(4.0), Round::Rne), (d(2.0), 0));
        assert_eq!(sqrt::<B64>(d(0.25), Round::Rne), (d(0.5), 0));
        assert_eq!(sqrt::<B32>(s(9.0), Round::Rne), (s(3.0), 0));
        let (v, f) = sqrt::<B64>(d(2.0), Round::Rne);
        assert_eq!(v, d(core::f64::consts::SQRT_2));
        assert_eq!(f, flags::NX);
        let (v, f) = sqrt::<B64>(d(-1.0), Round::Rne);
        assert_eq!(v, B64::CANONICAL_NAN);
        assert_eq!(f, flags::NV);
        assert_eq!(sqrt::<B64>(d(-0.0), Round::Rne), (d(-0.0), 0));
    }

    #[test]
    fn fused_multiply_add_rounds_once() {
        // (1 + 2^-52)(1 - 2^-52) is 1 - 2^-104: exact in 106 bits, and
        // indistinguishable from 1.0 once rounded. So the fused result is
        // -2^-104 and the separately rounded one is zero, which is the entire
        // point of the instruction.
        let a = d(1.0) + 1;
        let b = d(1.0) - 2;
        let (fused, f) = fma::<B64>(a, b, d(-1.0), Round::Rne);
        assert_eq!(fused, (1u64 << 63) | (919u64 << 52));
        assert_eq!(f, 0);
        let (rounded, _) = mul::<B64>(a, b, Round::Rne);
        let (separate, _) = add::<B64>(rounded, d(-1.0), Round::Rne);
        assert_eq!(separate, d(0.0));
        assert_eq!(fma::<B64>(d(2.0), d(3.0), d(4.0), Round::Rne), (d(10.0), 0));
        assert_eq!(fma::<B64>(d(2.0), d(3.0), d(0.0), Round::Rne), (d(6.0), 0));
        let (v, f) = fma::<B64>(d(0.0), d(f64::INFINITY), d(1.0), Round::Rne);
        assert_eq!(v, B64::CANONICAL_NAN);
        assert_eq!(f, flags::NV);
    }

    #[test]
    fn comparisons_distinguish_quiet_from_signaling() {
        let qnan = B64::CANONICAL_NAN;
        let snan = B64::INF | 1;
        assert_eq!(eq::<B64>(qnan, d(1.0)), (false, 0));
        assert_eq!(eq::<B64>(snan, d(1.0)), (false, flags::NV));
        assert_eq!(lt::<B64>(qnan, d(1.0)), (false, flags::NV));
        assert_eq!(lt::<B64>(d(-1.0), d(1.0)), (true, 0));
        assert_eq!(le::<B64>(d(0.0), d(-0.0)), (true, 0));
        assert_eq!(eq::<B64>(d(0.0), d(-0.0)), (true, 0));
    }

    #[test]
    fn min_max_follow_the_riscv_rules() {
        let qnan = B64::CANONICAL_NAN;
        assert_eq!(min::<B64>(qnan, d(1.0)), (d(1.0), 0));
        assert_eq!(max::<B64>(qnan, d(1.0)), (d(1.0), 0));
        assert_eq!(min::<B64>(qnan, qnan), (qnan, 0));
        assert_eq!(min::<B64>(d(-0.0), d(0.0)), (d(-0.0), 0));
        assert_eq!(max::<B64>(d(-0.0), d(0.0)), (d(0.0), 0));
    }

    #[test]
    fn classification_covers_every_bit() {
        assert_eq!(classify::<B64>(d(f64::NEG_INFINITY)), 1 << 0);
        assert_eq!(classify::<B64>(d(-1.0)), 1 << 1);
        assert_eq!(classify::<B64>(B64::SIGN | 1), 1 << 2);
        assert_eq!(classify::<B64>(d(-0.0)), 1 << 3);
        assert_eq!(classify::<B64>(d(0.0)), 1 << 4);
        assert_eq!(classify::<B64>(1), 1 << 5);
        assert_eq!(classify::<B64>(d(1.0)), 1 << 6);
        assert_eq!(classify::<B64>(d(f64::INFINITY)), 1 << 7);
        assert_eq!(classify::<B64>(B64::INF | 1), 1 << 8);
        assert_eq!(classify::<B64>(B64::CANONICAL_NAN), 1 << 9);
    }

    #[test]
    fn format_conversion_round_trips() {
        assert_eq!(convert::<B32, B64>(s(1.5), Round::Rne), (d(1.5), 0));
        assert_eq!(convert::<B64, B32>(d(1.5), Round::Rne), (s(1.5), 0));
        let (v, f) = convert::<B64, B32>(d(1e300), Round::Rne);
        assert_eq!(v, s(f32::INFINITY));
        assert_eq!(f, flags::OF | flags::NX);
    }

    #[test]
    fn integer_conversion_saturates_instead_of_trapping() {
        assert_eq!(to_signed::<B64>(d(1.9), 32, Round::Rtz), (1, flags::NX));
        assert_eq!(to_signed::<B64>(d(-1.9), 32, Round::Rtz), (-1, flags::NX));
        assert_eq!(to_signed::<B64>(d(1.5), 32, Round::Rne), (2, flags::NX));
        assert_eq!(to_signed::<B64>(d(2.5), 32, Round::Rne), (2, flags::NX));
        assert_eq!(to_signed::<B64>(d(2.5), 32, Round::Rmm), (3, flags::NX));
        assert_eq!(
            to_signed::<B64>(d(1e30), 32, Round::Rtz),
            (i64::from(i32::MAX), flags::NV)
        );
        assert_eq!(
            to_signed::<B64>(B64::CANONICAL_NAN, 32, Round::Rtz),
            (i64::from(i32::MAX), flags::NV)
        );
        assert_eq!(
            to_signed::<B64>(d(-1e30), 64, Round::Rtz),
            (i64::MIN, flags::NV)
        );
        assert_eq!(
            to_signed::<B64>(d(-2147483648.0), 32, Round::Rtz),
            (i64::from(i32::MIN), 0)
        );
        assert_eq!(to_unsigned::<B64>(d(-0.5), 32, Round::Rtz), (0, flags::NX));
        assert_eq!(to_unsigned::<B64>(d(-1.5), 32, Round::Rtz), (0, flags::NV));
        assert_eq!(
            to_unsigned::<B64>(d(4294967295.0), 32, Round::Rtz),
            (0xffff_ffff, 0)
        );
    }

    #[test]
    fn integer_to_float_rounds() {
        assert_eq!(from_signed::<B64>(-3, 32, Round::Rne), (d(-3.0), 0));
        assert_eq!(from_unsigned::<B64>(3, 32, Round::Rne), (d(3.0), 0));
        let (v, f) = from_signed::<B64>((1i64 << 53) + 1, 64, Round::Rne);
        assert_eq!(v, d(9007199254740992.0));
        assert_eq!(f, flags::NX);
        assert_eq!(from_signed::<B64>(i64::from(i32::MIN), 32, Round::Rne).1, 0);
        assert_eq!(from_signed::<B32>(16_777_217, 32, Round::Rne).1, flags::NX);
    }

    #[test]
    fn overflow_depends_on_the_rounding_mode() {
        let big = d(f64::MAX);
        let (v, f) = add::<B64>(big, big, Round::Rne);
        assert_eq!(v, d(f64::INFINITY));
        assert_eq!(f, flags::OF | flags::NX);
        assert_eq!(add::<B64>(big, big, Round::Rtz).0, d(f64::MAX));
        assert_eq!(add::<B64>(big, big, Round::Rdn).0, d(f64::MAX));
    }

    /// A cheap exhaustive-ish differential check against the host FPU for the
    /// cases where the host is known to agree (round-to-nearest, no NaN): it
    /// is not the oracle — the specification is — but a disagreement here is
    /// always a bug in this file.
    #[test]
    fn agrees_with_the_host_on_ordinary_values() {
        let mut x: u64 = 0x1234_5678_9abc_def0;
        for _ in 0..20_000 {
            // A cheap xorshift, so the sample is deterministic.
            x ^= x << 13;
            x ^= x >> 7;
            x ^= x << 17;
            let a = f64::from_bits(x);
            let b = f64::from_bits(x.rotate_left(29));
            if !a.is_finite() || !b.is_finite() {
                continue;
            }
            let want = a + b;
            let (got, _) = add::<B64>(a.to_bits(), b.to_bits(), Round::Rne);
            if want.is_finite() {
                assert_eq!(got, want.to_bits(), "{a:e} + {b:e}");
            }
            let want = a * b;
            let (got, _) = mul::<B64>(a.to_bits(), b.to_bits(), Round::Rne);
            if want.is_finite() {
                assert_eq!(got, want.to_bits(), "{a:e} * {b:e}");
            }
            if b != 0.0 {
                let want = a / b;
                let (got, _) = div::<B64>(a.to_bits(), b.to_bits(), Round::Rne);
                if want.is_finite() {
                    assert_eq!(got, want.to_bits(), "{a:e} / {b:e}");
                }
            }
            if a > 0.0 {
                let want = a.sqrt();
                let (got, _) = sqrt::<B64>(a.to_bits(), Round::Rne);
                assert_eq!(got, want.to_bits(), "sqrt({a:e})");
            }
        }
    }
}
