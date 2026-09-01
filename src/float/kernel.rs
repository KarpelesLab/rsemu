//! The arithmetic kernel: exact integer computation, rounded exactly once.
//!
//! Nothing in this file knows how any format is encoded. A value arrives as
//! [`Parts`] — a sign, an integer significand and the exponent of that
//! significand's last bit — and leaves as [`Outcome`], which the encoding
//! layers ([`super::binary`], [`super::x87`]) turn back into bits. That split
//! is what lets binary32, binary64 and the 80-bit extended format share one
//! implementation of every operation, and it is what makes x87's precision
//! control a [`Spec`] rather than a special case.
//!
//! Sources: IEEE 754-2019 §5 for the operations, §6.2 for NaNs, §7 for the
//! exceptions.

use super::{Env, Flags, MinMax, Propagate, Round, Spec, Tininess};

/// How many guard bits the rounding step keeps below the result's last place.
///
/// Three — guard, round and sticky — the classical minimum for a correctly
/// rounded result, and one more than addition strictly needs so a one-bit
/// renormalisation never has to invent a bit.
const EXTRA: u32 = 3;

/// What kind of number a value is (IEEE 754-2019 §3.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Class {
    /// A signed zero.
    Zero,
    /// A non-zero finite number, normal or subnormal.
    Finite,
    /// A signed infinity.
    Inf,
    /// A NaN, quiet or signaling.
    Nan,
}

/// A value taken apart: `(-1)^sign * frac * 2^exp`.
///
/// There is no hidden bit and no bias here — a normal and a subnormal differ
/// only in how big `frac` happens to be, which is what makes alignment in
/// [`add`] a plain exponent difference. For a NaN, `frac` is the trailing
/// significand field as the source format encodes it, because the propagation
/// rules compare and copy exactly that.
#[derive(Debug, Clone, Copy)]
pub(super) struct Parts {
    /// The sign bit.
    pub sign: bool,
    /// The exponent of `frac`'s least significant bit.
    pub exp: i32,
    /// The significand as an integer, or a NaN's payload.
    pub frac: u64,
    /// Which kind of value this is.
    pub class: Class,
    /// Set only for a signaling NaN, which raises invalid wherever it appears.
    pub snan: bool,
}

impl Parts {
    /// A signed zero.
    pub(super) const fn zero(sign: bool) -> Parts {
        Parts {
            sign,
            exp: 0,
            frac: 0,
            class: Class::Zero,
            snan: false,
        }
    }

    /// Shift `frac` up until its top bit is bit 63, leaving the value alone.
    ///
    /// Every operation below normalises its operands first, which is what
    /// bounds the alignment analysis: two normalised significands differ in
    /// value-exponent by exactly their `exp` difference.
    fn normalized(mut self) -> Parts {
        if self.class == Class::Finite && self.frac != 0 {
            let n = self.frac.leading_zeros();
            self.frac <<= n;
            self.exp -= n as i32;
        }
        self
    }
}

/// A rounded result, still without an encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Rounded {
    /// Zero.
    Zero,
    /// `frac * 2^exp`, with `frac` no wider than the [`Spec`]'s precision.
    Finite { exp: i32, frac: u64 },
    /// Infinity.
    Inf,
}

/// What an operation produced, for the encoding layer to assemble.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Outcome {
    /// A number, with its sign.
    Num(bool, Rounded),
    /// The environment's default NaN.
    DefaultNan,
    /// An operand's NaN, to be returned quiet with this payload.
    Nan { sign: bool, payload: u64 },
}

/// Right shift, folding everything shifted out into bit 0.
///
/// The sticky bit is an OR rather than an add precisely so it can never carry
/// into a meaningful bit: it records *that* something was dropped, which is all
/// any rounding rule needs to know about it.
pub(super) fn shr_sticky(v: u128, n: u32) -> u128 {
    if n == 0 {
        return v;
    }
    if n >= 128 {
        return u128::from(v != 0);
    }
    let lost = v & ((1u128 << n) - 1);
    (v >> n) | u128::from(lost != 0)
}

/// A 256-bit unsigned integer, for the one operation that needs one.
///
/// `fma` holds a 128-bit product and a 64-bit addend in a single window wide
/// enough that no alignment shift can lose a bit that later cancellation could
/// expose. Every other operation fits in a `u128` and uses one.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(super) struct Wide {
    /// The high 128 bits — declared first so the derived ordering is numeric.
    hi: u128,
    /// The low 128 bits.
    lo: u128,
}

impl Wide {
    /// Zero.
    const ZERO: Wide = Wide { hi: 0, lo: 0 };

    /// A 128-bit value, widened.
    fn from_u128(v: u128) -> Wide {
        Wide { hi: 0, lo: v }
    }

    /// `self << n`, for `n < 256`. Bits shifted off the top are lost, which no
    /// caller here allows to happen.
    fn shl(self, n: u32) -> Wide {
        if n == 0 {
            return self;
        }
        if n >= 256 {
            return Wide::ZERO;
        }
        if n >= 128 {
            Wide {
                hi: self.lo << (n - 128),
                lo: 0,
            }
        } else {
            Wide {
                hi: (self.hi << n) | (self.lo >> (128 - n)),
                lo: self.lo << n,
            }
        }
    }

    /// `self >> n`, folding everything shifted out into bit 0.
    fn shr_sticky(self, n: u32) -> Wide {
        if n == 0 {
            return self;
        }
        if n >= 256 {
            return Wide::from_u128(u128::from(self != Wide::ZERO));
        }
        if n >= 128 {
            let lost = self.lo != 0 || (n > 128 && self.hi & ((1u128 << (n - 128)) - 1) != 0);
            Wide {
                hi: 0,
                lo: (self.hi >> (n - 128)) | u128::from(lost),
            }
        } else {
            let lost = self.lo & ((1u128 << n) - 1) != 0;
            Wide {
                hi: self.hi >> n,
                lo: (self.hi << (128 - n)) | (self.lo >> n) | u128::from(lost),
            }
        }
    }

    /// `self + other`, which no caller here lets overflow.
    fn add(self, other: Wide) -> Wide {
        let (lo, carry) = self.lo.overflowing_add(other.lo);
        Wide {
            hi: self
                .hi
                .wrapping_add(other.hi)
                .wrapping_add(u128::from(carry)),
            lo,
        }
    }

    /// `self - other`, which callers only use when `self >= other`.
    fn sub(self, other: Wide) -> Wide {
        let (lo, borrow) = self.lo.overflowing_sub(other.lo);
        Wide {
            hi: self
                .hi
                .wrapping_sub(other.hi)
                .wrapping_sub(u128::from(borrow)),
            lo,
        }
    }

    /// The value as `(sig, shift)` with `value = sig * 2^shift`, everything
    /// below folded into a sticky bit.
    fn narrow(self) -> (u128, u32) {
        if self.hi == 0 {
            return (self.lo, 0);
        }
        let shift = 128 - self.hi.leading_zeros();
        (self.shr_sticky(shift).lo, shift)
    }
}

/// If any operand is a NaN, the result the environment's rules demand.
///
/// IEEE 754-2019 §6.2.3 only *recommends* propagating a payload, so which NaN
/// comes out is [`Propagate`] — see its variants for each guest's rule and its
/// citation. A signaling operand raises invalid under every one of them
/// (§7.2).
pub(super) fn nan_result(ops: &[Parts], env: Env) -> Option<(Outcome, Flags)> {
    let mut any = false;
    let mut signaling = false;
    for p in ops {
        if p.class == Class::Nan {
            any = true;
            signaling |= p.snan;
        }
    }
    if !any {
        return None;
    }
    let flags = if signaling {
        Flags::INVALID
    } else {
        Flags::NONE
    };
    let pick = |p: &Parts| Outcome::Nan {
        sign: p.sign,
        payload: p.frac,
    };
    let first = |want_snan: bool| {
        ops.iter()
            .find(|p| p.class == Class::Nan && p.snan == want_snan)
    };
    let out = match env.nan.propagate {
        Propagate::Default => Outcome::DefaultNan,
        Propagate::FirstNan => pick(ops.iter().find(|p| p.class == Class::Nan)?),
        Propagate::SignalingFirst => pick(first(true).or_else(|| first(false))?),
        Propagate::LargerSignificand => {
            // Table 4-7: a quiet NaN beside a signaling one wins outright, so
            // the significand comparison only ever runs between NaNs of the
            // same kind. `max_by_key` keeps the *last* maximum, and the first
            // operand is the documented tie-break, so the search runs
            // backwards.
            let quiet = first(false).is_some();
            pick(
                ops.iter()
                    .rev()
                    .filter(|p| p.class == Class::Nan && p.snan != quiet)
                    .max_by_key(|p| p.frac)?,
            )
        }
    };
    Some((out, flags))
}

/// Round an exact value to a format and report the exceptions.
///
/// `value = (-1)^sign * sig * 2^exp`, with everything already lost folded into
/// `sig`'s bit 0. This is the only rounding step in the subsystem: every
/// operation hands its exact result here, so no operation can round twice
/// (IEEE 754-2019 §5.1, "as if it first produced an intermediate result
/// correct to infinite precision ... then rounded").
pub(super) fn round_exact(
    sign: bool,
    exp: i32,
    sig: u128,
    spec: Spec,
    env: Env,
) -> (Outcome, Flags) {
    if sig == 0 {
        return (Outcome::Num(sign, Rounded::Zero), Flags::NONE);
    }
    let p = spec.precision;
    // Line the significand up so its top bit is `EXTRA` places above the last
    // bit the format keeps.
    let target = p + EXTRA - 1;
    let msb = 127 - sig.leading_zeros();
    let (mut sig, mut exp) = if msb > target {
        let n = msb - target;
        (shr_sticky(sig, n), exp + n as i32)
    } else {
        let n = target - msb;
        (sig << n, exp - n as i32)
    };
    // The unrounded value is in [2^lead, 2^(lead+1)).
    let lead = exp + target as i32;
    let tiny_before = lead < spec.emin();

    // Below the smallest normal the result lives on the format's fixed
    // subnormal grid: precision is lost here rather than pretended away, and
    // never flushed unless the guest asked for flushing.
    if exp + (EXTRA as i32) < spec.min_ulp {
        let n = (spec.min_ulp - (exp + EXTRA as i32)) as u32;
        sig = shr_sticky(sig, n);
        exp += n as i32;
    }

    let rem = sig & ((1 << EXTRA) - 1);
    let half = 1u128 << (EXTRA - 1);
    let mut keep = sig >> EXTRA;
    let inexact = rem != 0;
    let up = match env.round {
        Round::TiesEven => rem > half || (rem == half && keep & 1 != 0),
        Round::TiesAway => rem >= half,
        Round::TowardZero => false,
        Round::TowardNegative => inexact && sign,
        Round::TowardPositive => inexact && !sign,
    };
    let mut ulp = exp + EXTRA as i32;
    if up {
        keep += 1;
        if keep >> p != 0 {
            // Carried out of the precision. The bit dropped here is zero —
            // `keep` is exactly 2^p — so this is not a second rounding.
            keep >>= 1;
            ulp += 1;
        }
    }
    if keep == 0 {
        // Everything rounded away; `sig` was non-zero, so this is inexact by
        // construction and tiny under either detection.
        return (
            Outcome::Num(sign, Rounded::Zero),
            Flags::INEXACT | Flags::UNDERFLOW,
        );
    }

    let lead_final = ulp + (127 - keep.leading_zeros()) as i32;
    if lead_final > spec.emax {
        // Which way overflow goes is a property of the rounding direction:
        // only the two round-to-nearest attributes always reach infinity
        // (IEEE 754-2019 §7.4).
        let to_inf = match env.round {
            Round::TiesEven | Round::TiesAway => true,
            Round::TowardZero => false,
            Round::TowardNegative => sign,
            Round::TowardPositive => !sign,
        };
        let out = if to_inf {
            Rounded::Inf
        } else {
            Rounded::Finite {
                exp: spec.emax - (p as i32 - 1),
                frac: u64::MAX >> (64 - p),
            }
        };
        return (Outcome::Num(sign, out), Flags::OVERFLOW | Flags::INEXACT);
    }

    let tiny = match env.tininess {
        Tininess::AfterRounding => lead_final < spec.emin(),
        Tininess::BeforeRounding => tiny_before,
    };
    if tiny && env.flush_outputs {
        // x86 `MXCSR.FTZ` and ARM `FPCR.FZ` deliver a zero of the result's
        // sign and report both underflow and inexact, whether or not the
        // subnormal result was itself exact.
        return (
            Outcome::Num(sign, Rounded::Zero),
            Flags::INEXACT | Flags::UNDERFLOW,
        );
    }
    let mut flags = if inexact { Flags::INEXACT } else { Flags::NONE };
    if tiny && inexact {
        // §7.5: underflow is signalled when the result is both tiny and
        // inexact. A subnormal that is exact is not an underflow.
        flags |= Flags::UNDERFLOW;
    }
    (
        Outcome::Num(
            sign,
            Rounded::Finite {
                exp: ulp,
                frac: keep as u64,
            },
        ),
        flags,
    )
}

/// The sign a cancelled-to-zero sum takes: `+0` in every direction but
/// round-toward-negative (IEEE 754-2019 §6.3).
fn cancelled_sign(env: Env) -> bool {
    env.round == Round::TowardNegative
}

/// `a + b`, with `b` optionally negated first — which is how `sub` is spelled.
///
/// Both significands are normalised and then widened with 62 fraction bits, so
/// any alignment shift small enough for cancellation to matter is *exact*.
/// A longer shift means the smaller operand cannot reach the top `p + 3` bits
/// of the result at all, and folding it into a sticky bit loses nothing.
pub(super) fn add(a: Parts, b: Parts, negate: bool, spec: Spec, env: Env) -> (Outcome, Flags) {
    let mut b = b;
    if negate {
        b.sign = !b.sign;
    }
    if let Some(r) = nan_result(&[a, b], env) {
        return r;
    }
    match (a.class, b.class) {
        (Class::Inf, Class::Inf) => {
            if a.sign == b.sign {
                (Outcome::Num(a.sign, Rounded::Inf), Flags::NONE)
            } else {
                // §7.2: the difference of two infinities is invalid.
                (Outcome::DefaultNan, Flags::INVALID)
            }
        }
        (Class::Inf, _) => (Outcome::Num(a.sign, Rounded::Inf), Flags::NONE),
        (_, Class::Inf) => (Outcome::Num(b.sign, Rounded::Inf), Flags::NONE),
        (Class::Zero, Class::Zero) => {
            let sign = if a.sign == b.sign {
                a.sign
            } else {
                cancelled_sign(env)
            };
            (Outcome::Num(sign, Rounded::Zero), Flags::NONE)
        }
        (Class::Zero, _) => round_exact(b.sign, b.exp, u128::from(b.frac), spec, env),
        (_, Class::Zero) => round_exact(a.sign, a.exp, u128::from(a.frac), spec, env),
        _ => {
            let (a, b) = (a.normalized(), b.normalized());
            let (hi, lo) = if a.exp >= b.exp { (a, b) } else { (b, a) };
            const SLACK: u32 = 62;
            let sig_hi = u128::from(hi.frac) << SLACK;
            let sig_lo = shr_sticky(u128::from(lo.frac) << SLACK, (hi.exp - lo.exp) as u32);
            let (sign, sig) = if hi.sign == lo.sign {
                (hi.sign, sig_hi + sig_lo)
            } else if sig_hi >= sig_lo {
                (hi.sign, sig_hi - sig_lo)
            } else {
                (lo.sign, sig_lo - sig_hi)
            };
            if sig == 0 {
                return (
                    Outcome::Num(cancelled_sign(env), Rounded::Zero),
                    Flags::NONE,
                );
            }
            round_exact(sign, hi.exp - SLACK as i32, sig, spec, env)
        }
    }
}

/// `a * b`. The full product is kept — two 64-bit significands fit a `u128`
/// exactly — so nothing is dropped before the single rounding step.
pub(super) fn mul(a: Parts, b: Parts, spec: Spec, env: Env) -> (Outcome, Flags) {
    if let Some(r) = nan_result(&[a, b], env) {
        return r;
    }
    let sign = a.sign ^ b.sign;
    match (a.class, b.class) {
        // §7.2: zero times infinity, in either order, is the only way
        // multiplication produces a NaN.
        (Class::Inf, Class::Zero) | (Class::Zero, Class::Inf) => {
            (Outcome::DefaultNan, Flags::INVALID)
        }
        (Class::Inf, _) | (_, Class::Inf) => (Outcome::Num(sign, Rounded::Inf), Flags::NONE),
        (Class::Zero, _) | (_, Class::Zero) => (Outcome::Num(sign, Rounded::Zero), Flags::NONE),
        _ => {
            let sig = u128::from(a.frac) * u128::from(b.frac);
            round_exact(sign, a.exp + b.exp, sig, spec, env)
        }
    }
}

/// `a / b`.
///
/// The quotient is developed to 71 bits before rounding — more than the widest
/// format's precision plus its guard bits — and the remainder becomes a sticky
/// bit, so the rounding step sees an exact position.
pub(super) fn div(a: Parts, b: Parts, spec: Spec, env: Env) -> (Outcome, Flags) {
    if let Some(r) = nan_result(&[a, b], env) {
        return r;
    }
    let sign = a.sign ^ b.sign;
    match (a.class, b.class) {
        // §7.2: infinity over infinity and zero over zero are invalid.
        (Class::Inf, Class::Inf) | (Class::Zero, Class::Zero) => {
            (Outcome::DefaultNan, Flags::INVALID)
        }
        (Class::Inf, _) => (Outcome::Num(sign, Rounded::Inf), Flags::NONE),
        (_, Class::Inf) => (Outcome::Num(sign, Rounded::Zero), Flags::NONE),
        // §7.3: a finite non-zero over zero is the divideByZero exception, and
        // its result is an exactly infinite value.
        (_, Class::Zero) => (Outcome::Num(sign, Rounded::Inf), Flags::DIV_BY_ZERO),
        (Class::Zero, _) => (Outcome::Num(sign, Rounded::Zero), Flags::NONE),
        _ => {
            let (a, b) = (a.normalized(), b.normalized());
            let den = u128::from(b.frac);
            let n1 = u128::from(a.frac) << 63;
            let (q1, r1) = (n1 / den, n1 % den);
            let n2 = r1 << 8;
            let (q2, r2) = (n2 / den, n2 % den);
            let mut q = (q1 << 8) | q2;
            if r2 != 0 {
                q |= 1;
            }
            round_exact(sign, a.exp - b.exp - 71, q, spec, env)
        }
    }
}

/// The integer square root of a radicand extended by `extra` zero bit-pairs,
/// and whether anything was left over.
///
/// Restoring, two bits of radicand per bit of root, so it needs no division —
/// and no floating point, which is the whole point of the subsystem. The extra
/// pairs are where the root's fraction bits come from: 40 of them put more than
/// 70 significant bits in the root, which covers every format here.
fn isqrt(radicand: u128, extra: u32) -> (u128, bool) {
    if radicand == 0 {
        return (0, false);
    }
    let mut rem: u128 = 0;
    let mut root: u128 = 0;
    // Start at the highest even bit position at or below the top of the value.
    let mut shift = (127 - radicand.leading_zeros()) & !1;
    loop {
        rem = (rem << 2) | ((radicand >> shift) & 3);
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
    for _ in 0..extra {
        rem <<= 2;
        root <<= 1;
        let trial = (root << 1) | 1;
        if rem >= trial {
            rem -= trial;
            root |= 1;
        }
    }
    (root, rem != 0)
}

/// The square root of `a` (IEEE 754-2019 §5.4.1).
pub(super) fn sqrt(a: Parts, spec: Spec, env: Env) -> (Outcome, Flags) {
    if let Some(r) = nan_result(&[a], env) {
        return r;
    }
    match a.class {
        // sqrt(-0) is -0 and is not an invalid operation; every other negative
        // is, including -inf.
        Class::Zero => (Outcome::Num(a.sign, Rounded::Zero), Flags::NONE),
        _ if a.sign => (Outcome::DefaultNan, Flags::INVALID),
        Class::Inf => (Outcome::Num(false, Rounded::Inf), Flags::NONE),
        _ => {
            let a = a.normalized();
            let mut frac = u128::from(a.frac);
            let mut exp = a.exp;
            if exp & 1 != 0 {
                // The exponent has to be even to halve it; moving one power of
                // two into the significand keeps the value identical.
                frac <<= 1;
                exp -= 1;
            }
            const EXTRA_PAIRS: u32 = 40;
            let (root, rest) = isqrt(frac, EXTRA_PAIRS);
            let sig = if rest { root | 1 } else { root };
            round_exact(false, exp / 2 - EXTRA_PAIRS as i32, sig, spec, env)
        }
    }
}

/// `a * b + c` with one rounding at the end — a true fused multiply-add
/// (IEEE 754-2019 §5.4.1).
///
/// The exact product and the addend share one 256-bit window, placed so that
/// every exponent difference small enough for cancellation to matter is
/// aligned without loss. A larger difference means one term dominates by more
/// than the format's precision, and what falls out of the window can then only
/// ever be a sticky bit.
pub(super) fn fma(a: Parts, b: Parts, c: Parts, spec: Spec, env: Env) -> (Outcome, Flags) {
    // 0 * inf is invalid even when the addend is a NaN, so the multiply is
    // checked before the NaN sweep (§7.2).
    if matches!(
        (a.class, b.class),
        (Class::Inf, Class::Zero) | (Class::Zero, Class::Inf)
    ) {
        return (Outcome::DefaultNan, Flags::INVALID);
    }
    if let Some(r) = nan_result(&[a, b, c], env) {
        return r;
    }
    let psign = a.sign ^ b.sign;
    if a.class == Class::Inf || b.class == Class::Inf {
        if c.class == Class::Inf && c.sign != psign {
            return (Outcome::DefaultNan, Flags::INVALID);
        }
        return (Outcome::Num(psign, Rounded::Inf), Flags::NONE);
    }
    if c.class == Class::Inf {
        return (Outcome::Num(c.sign, Rounded::Inf), Flags::NONE);
    }
    if a.class == Class::Zero || b.class == Class::Zero {
        if c.class == Class::Zero {
            let sign = if psign == c.sign {
                psign
            } else {
                cancelled_sign(env)
            };
            return (Outcome::Num(sign, Rounded::Zero), Flags::NONE);
        }
        return round_exact(c.sign, c.exp, u128::from(c.frac), spec, env);
    }
    if c.class == Class::Zero {
        return mul(a, b, spec, env);
    }

    let (a, b, c) = (a.normalized(), b.normalized(), c.normalized());
    let prod = u128::from(a.frac) * u128::from(b.frac);
    let pexp = a.exp + b.exp;
    // A 256-bit window whose top is just above whichever term leads. The
    // product's top bit lands at 249 when it leads and the addend's at 249 when
    // it does, so the sum needs 251 bits and has 256.
    let base = core::cmp::max(pexp + 128, c.exp + 64) - 250;
    let place = |v: u128, exp: i32| {
        let shift = exp - base;
        if shift >= 0 {
            Wide::from_u128(v).shl(shift as u32)
        } else {
            Wide::from_u128(v).shr_sticky((-shift) as u32)
        }
    };
    let psig = place(prod, pexp);
    let csig = place(u128::from(c.frac), c.exp);
    let (sign, sum) = if psign == c.sign {
        (psign, psig.add(csig))
    } else if psig >= csig {
        (psign, psig.sub(csig))
    } else {
        (c.sign, csig.sub(psig))
    };
    if sum == Wide::ZERO {
        return (
            Outcome::Num(cancelled_sign(env), Rounded::Zero),
            Flags::NONE,
        );
    }
    let (sig, shift) = sum.narrow();
    round_exact(sign, base + shift as i32, sig, spec, env)
}

/// The ordering of two non-NaN values.
pub(super) fn compare(a: Parts, b: Parts) -> core::cmp::Ordering {
    use core::cmp::Ordering;
    // A normalised significand always has its top bit at 63, so `(exp, frac)`
    // orders finite magnitudes lexicographically. Zero sorts below every one
    // of them and infinity above.
    let magnitude = |p: &Parts| match p.class {
        Class::Zero => (i32::MIN, 0u64),
        Class::Inf => (i32::MAX, u64::MAX),
        _ => {
            let n = p.normalized();
            (n.exp, n.frac)
        }
    };
    // +0 and -0 compare equal, which is the one place the sign is ignored
    // (IEEE 754-2019 §5.11).
    if a.class == Class::Zero && b.class == Class::Zero {
        return Ordering::Equal;
    }
    match (a.sign, b.sign) {
        (false, true) => Ordering::Greater,
        (true, false) => Ordering::Less,
        (false, false) => magnitude(&a).cmp(&magnitude(&b)),
        (true, true) => magnitude(&b).cmp(&magnitude(&a)),
    }
}

/// `min` or `max` of two values, under the environment's rule.
///
/// Returns the index of the winning operand, or an [`Outcome`] when the answer
/// is a NaN that no operand supplies verbatim. A signaling operand raises
/// invalid under every rule (IEEE 754-2019 §7.2), whichever value wins.
pub(super) fn min_max(
    a: Parts,
    b: Parts,
    want_min: bool,
    env: Env,
) -> (Result<usize, Outcome>, Flags) {
    use core::cmp::Ordering;
    let nan_a = a.class == Class::Nan;
    let nan_b = b.class == Class::Nan;
    let flags = if a.snan || b.snan {
        Flags::INVALID
    } else {
        Flags::NONE
    };
    match env.min_max {
        MinMax::NonNan => match (nan_a, nan_b) {
            (true, true) => return (Err(Outcome::DefaultNan), flags),
            (true, false) => return (Ok(1), flags),
            (false, true) => return (Ok(0), flags),
            (false, false) => {}
        },
        // x86 returns the second source for any NaN and for equal operands,
        // which is why min(-0, +0) and min(+0, -0) both give the second.
        MinMax::SecondOperand => {
            if nan_a || nan_b {
                return (Ok(1), flags);
            }
            let ord = compare(a, b);
            let first_wins = if want_min {
                ord == Ordering::Less
            } else {
                ord == Ordering::Greater
            };
            return (Ok(usize::from(!first_wins)), flags);
        }
        MinMax::PropagateNan => {
            if let Some((out, f)) = nan_result(&[a, b], env) {
                return (Err(out), f);
            }
        }
    }
    // Signed zeros: -0 is smaller than +0, which a magnitude comparison alone
    // would call equal.
    let ord = if a.class == Class::Zero && b.class == Class::Zero {
        match (a.sign, b.sign) {
            (true, false) => Ordering::Less,
            (false, true) => Ordering::Greater,
            _ => Ordering::Equal,
        }
    } else {
        compare(a, b)
    };
    let take_a = if want_min {
        ord != Ordering::Greater
    } else {
        ord != Ordering::Less
    };
    (Ok(usize::from(!take_a)), flags)
}

/// A value rounded to an integer, as sign, magnitude and inexactness.
///
/// The magnitude comes back as a `u128` so the range check belongs to the
/// caller, which is the only part that differs between integer widths.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum IntValue {
    /// The input was a NaN.
    Nan,
    /// The input was an infinity of this sign.
    Inf(bool),
    /// A rounded magnitude, with its sign and whether rounding lost anything.
    Value {
        sign: bool,
        magnitude: u128,
        inexact: bool,
    },
}

/// Round a value to an integer, in the environment's direction.
pub(super) fn to_integer(p: Parts, env: Env) -> IntValue {
    match p.class {
        Class::Nan => IntValue::Nan,
        Class::Inf => IntValue::Inf(p.sign),
        Class::Zero => IntValue::Value {
            sign: p.sign,
            magnitude: 0,
            inexact: false,
        },
        Class::Finite => {
            // Scale by 2^EXTRA so the same rounding rules as `round_exact`
            // apply, then shift the guard bits away.
            let shift = p.exp + EXTRA as i32;
            let scaled: u128 = if shift >= 0 {
                if shift > 63 {
                    // Far past any integer width; the caller saturates.
                    return IntValue::Value {
                        sign: p.sign,
                        magnitude: u128::MAX,
                        inexact: false,
                    };
                }
                u128::from(p.frac) << shift
            } else {
                shr_sticky(u128::from(p.frac), (-shift) as u32)
            };
            let rem = scaled & ((1 << EXTRA) - 1);
            let half = 1u128 << (EXTRA - 1);
            let mut magnitude = scaled >> EXTRA;
            let inexact = rem != 0;
            let up = match env.round {
                Round::TiesEven => rem > half || (rem == half && magnitude & 1 != 0),
                Round::TiesAway => rem >= half,
                Round::TowardZero => false,
                Round::TowardNegative => inexact && p.sign,
                Round::TowardPositive => inexact && !p.sign,
            };
            if up {
                magnitude += 1;
            }
            IntValue::Value {
                sign: p.sign,
                magnitude,
                inexact,
            }
        }
    }
}
