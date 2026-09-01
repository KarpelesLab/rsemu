//! The x87 80-bit double extended format, and its precision control.
//!
//! `ROADMAP.md` §9.1 names this specifically: 80-bit extended precision "exists
//! on no other host at all", and DOS and Win9x guests need it. Rust has no
//! `f80` and no `long double`, so unlike binary32 and binary64 there is not
//! even a host type to be tempted by — every bit of this is integer
//! arithmetic through `super::kernel`, the same kernel the interchange formats
//! use.
//!
//! # What is different about this format
//!
//! * **The integer bit is explicit** (SDM Volume 1, §4.2.2). A normal number
//!   stores its leading 1 rather than implying it, so the same value has
//!   encodings the interchange formats cannot express — and encodings that are
//!   not values at all.
//! * **Unsupported encodings** (§8.2.2): an *unnormal* (a non-zero exponent
//!   with the integer bit clear), a *pseudo-infinity* and a *pseudo-NaN* are
//!   rejected by the 387 and later as invalid operations. A *pseudo-denormal*
//!   — exponent field zero with the integer bit set — is not rejected: it is a
//!   redundant encoding of an ordinary value, and this module reads it as
//!   the value it encodes.
//! * **Precision control** (§8.1.5.2): `FADD`, `FSUB`, `FMUL`, `FDIV` and
//!   `FSQRT` round their significand to 24 or 53 bits when the control word
//!   says so, while keeping the 15-bit exponent range. That is a [`Spec`] with
//!   a shortened precision and an unchanged subnormal grid, which is why
//!   [`super::Spec`] carries `min_ulp` separately from `precision`.
//!
//! Nothing here is wired into a CPU core: this module is the format and its
//! arithmetic, tested standalone.

use super::binary::Format;
use super::kernel::{self, Class, IntValue, Outcome, Parts, Rounded};
use super::{Env, Flags, IntOverflow, Spec};

/// An 80-bit double extended value, as the ten bytes of memory hold it.
///
/// The significand is a full 64 bits including the explicit integer bit; the
/// other field is the 15-bit exponent with the sign in bit 15.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct F80 {
    /// The sign bit (15) and the biased exponent (14:0).
    pub sign_exp: u16,
    /// The significand, integer bit included.
    pub sig: u64,
}

/// The exponent bias (SDM Volume 1, Table 4-3).
const BIAS: i32 = 16383;
/// The largest exponent of a finite number.
const EMAX: i32 = 16383;
/// The all-ones exponent field.
const EXP_FIELD_MAX: u16 = 0x7fff;
/// The explicit integer bit.
const INTEGER_BIT: u64 = 1 << 63;
/// The bit that makes a NaN quiet.
const QUIET_BIT: u64 = 1 << 62;
/// The 63-bit fraction below the integer bit.
const FRAC_MASK: u64 = INTEGER_BIT - 1;

impl F80 {
    /// The arithmetic parameters of the format at full precision: `p = 64`,
    /// `emax = 16383`, and a subnormal grid reaching 2^-16445.
    pub const SPEC: Spec = Spec::interchange(64, EMAX);

    /// Positive zero.
    pub const ZERO: F80 = F80 {
        sign_exp: 0,
        sig: 0,
    };
    /// Positive infinity.
    pub const INFINITY: F80 = F80 {
        sign_exp: EXP_FIELD_MAX,
        sig: INTEGER_BIT,
    };
    /// The QNaN floating-point indefinite: a negative quiet NaN with an
    /// all-zero payload (SDM Volume 1, §4.2.2 and Table 4-1). This is what an
    /// invalid operation delivers under [`Env::X87`](super::Env::X87).
    pub const INDEFINITE: F80 = F80 {
        sign_exp: 0x8000 | EXP_FIELD_MAX,
        sig: INTEGER_BIT | QUIET_BIT,
    };
    /// The largest finite magnitude.
    pub const MAX_FINITE: F80 = F80 {
        sign_exp: EXP_FIELD_MAX - 1,
        sig: u64::MAX,
    };

    /// Build a value from its two fields.
    #[must_use]
    pub const fn new(sign_exp: u16, sig: u64) -> F80 {
        F80 { sign_exp, sig }
    }

    /// The sign bit.
    #[must_use]
    #[inline]
    pub const fn sign(self) -> bool {
        self.sign_exp & 0x8000 != 0
    }

    /// The biased exponent field.
    #[must_use]
    #[inline]
    pub const fn exp_field(self) -> u16 {
        self.sign_exp & EXP_FIELD_MAX
    }

    /// Read the ten bytes of an `FLD m80fp` operand, little-endian: the
    /// significand first, then the sign and exponent.
    #[must_use]
    pub fn from_bytes(bytes: [u8; 10]) -> F80 {
        let mut sig = [0u8; 8];
        sig.copy_from_slice(&bytes[..8]);
        F80 {
            sig: u64::from_le_bytes(sig),
            sign_exp: u16::from_le_bytes([bytes[8], bytes[9]]),
        }
    }

    /// The ten bytes an `FSTP m80fp` writes.
    #[must_use]
    pub fn to_bytes(self) -> [u8; 10] {
        let mut out = [0u8; 10];
        out[..8].copy_from_slice(&self.sig.to_le_bytes());
        out[8..].copy_from_slice(&self.sign_exp.to_le_bytes());
        out
    }
}

/// The rounding precision the x87 control word's `PC` field selects.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Precision {
    /// 24 significand bits (`PC = 00`).
    Single,
    /// 53 significand bits (`PC = 10`).
    Double,
    /// 64 significand bits (`PC = 11`) — the reset default.
    #[default]
    Extended,
}

impl Precision {
    /// Decode the two-bit `PC` field, or `None` for the reserved encoding 01
    /// (SDM Volume 1, §8.1.5.2).
    #[must_use]
    pub const fn from_pc(bits: u32) -> Option<Precision> {
        match bits & 3 {
            0 => Some(Precision::Single),
            2 => Some(Precision::Double),
            3 => Some(Precision::Extended),
            _ => None,
        }
    }

    /// The `PC` encoding of this precision.
    #[must_use]
    pub const fn pc(self) -> u32 {
        match self {
            Precision::Single => 0,
            Precision::Double => 2,
            Precision::Extended => 3,
        }
    }

    /// How many significand bits results keep.
    #[must_use]
    pub const fn bits(self) -> u32 {
        match self {
            Precision::Single => 24,
            Precision::Double => 53,
            Precision::Extended => 64,
        }
    }

    /// The arithmetic parameters: this precision, the 80-bit exponent range.
    ///
    /// Precision control shortens the significand and **not** the exponent
    /// range, which is exactly why a `PC = 53` result is not the same as a
    /// binary64 result near the ends of the range.
    #[must_use]
    pub const fn spec(self) -> Spec {
        F80::SPEC.with_precision(self.bits())
    }
}

/// What an 80-bit encoding means.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum X87Class {
    /// One of IEEE 754-2019 §5.7.2's ten classes. A pseudo-denormal lands here
    /// too, as the ordinary value it encodes.
    Ieee(super::Category),
    /// An unnormal, a pseudo-infinity or a pseudo-NaN: not a value. Using one
    /// as a source operand is an invalid operation on the 387 and later (SDM
    /// Volume 1, §8.2.2).
    Unsupported,
}

/// Take an 80-bit encoding apart, or report that it is not a value.
fn decode(v: F80) -> Option<Parts> {
    let sign = v.sign();
    let field = v.exp_field();
    let integer = v.sig & INTEGER_BIT != 0;
    let frac = v.sig & FRAC_MASK;
    if field == EXP_FIELD_MAX {
        if !integer {
            // Pseudo-infinity and pseudo-NaN.
            return None;
        }
        if frac == 0 {
            return Some(Parts {
                sign,
                exp: 0,
                frac: 0,
                class: Class::Inf,
                snan: false,
            });
        }
        return Some(Parts {
            sign,
            exp: 0,
            frac,
            class: Class::Nan,
            snan: v.sig & QUIET_BIT == 0,
        });
    }
    if field == 0 {
        // Subnormals and pseudo-denormals share one rule: the value is the
        // whole 64-bit significand on the smallest exponent's grid, whether or
        // not the integer bit happens to be set.
        return Some(Parts {
            sign,
            exp: F80::SPEC.min_ulp,
            frac: v.sig,
            class: if v.sig == 0 {
                Class::Zero
            } else {
                Class::Finite
            },
            snan: false,
        });
    }
    if !integer {
        // An unnormal.
        return None;
    }
    Some(Parts {
        sign,
        exp: i32::from(field) - BIAS - 63,
        frac: v.sig,
        class: Class::Finite,
        snan: false,
    })
}

/// Classify an encoding (IEEE 754-2019 §5.7.2, extended with x87's
/// unsupported forms).
#[must_use]
pub fn classify(v: F80) -> X87Class {
    use super::Category;
    let Some(p) = decode(v) else {
        return X87Class::Unsupported;
    };
    X87Class::Ieee(match p.class {
        Class::Nan => {
            if p.snan {
                Category::SignalingNan
            } else {
                Category::QuietNan
            }
        }
        Class::Inf => {
            if p.sign {
                Category::NegativeInfinity
            } else {
                Category::PositiveInfinity
            }
        }
        Class::Zero => {
            if p.sign {
                Category::NegativeZero
            } else {
                Category::PositiveZero
            }
        }
        Class::Finite => {
            // Subnormal means the exponent field is zero *and* the value is
            // below the smallest normal; a pseudo-denormal is not subnormal.
            let subnormal = v.exp_field() == 0 && v.sig & INTEGER_BIT == 0;
            match (p.sign, subnormal) {
                (true, false) => Category::NegativeNormal,
                (true, true) => Category::NegativeSubnormal,
                (false, true) => Category::PositiveSubnormal,
                (false, false) => Category::PositiveNormal,
            }
        }
    })
}

/// Take an operand apart, applying the environment's subnormal reporting.
///
/// The second element is `None` when the encoding is unsupported, which every
/// caller turns into an invalid operation delivering the indefinite.
fn unpack(v: F80, env: Env) -> (Option<Parts>, Flags) {
    let Some(p) = decode(v) else {
        return (None, Flags::INVALID);
    };
    let subnormal = p.class == Class::Finite && v.exp_field() == 0 && v.sig & INTEGER_BIT == 0;
    if !subnormal {
        return (Some(p), Flags::NONE);
    }
    let flags = if env.subnormal_inputs.reports() {
        Flags::DENORMAL
    } else {
        Flags::NONE
    };
    if env.subnormal_inputs.flushes() {
        (Some(Parts::zero(p.sign)), flags)
    } else {
        (Some(p), flags)
    }
}

/// Assemble an outcome into an 80-bit encoding.
fn encode(out: Outcome, env: Env) -> F80 {
    let sign_exp = |sign: bool, field: u16| if sign { 0x8000 | field } else { field };
    match out {
        Outcome::DefaultNan => F80 {
            sign_exp: sign_exp(env.nan.default_sign, EXP_FIELD_MAX),
            sig: INTEGER_BIT | QUIET_BIT,
        },
        Outcome::Nan { sign, payload } => F80 {
            sign_exp: sign_exp(sign, EXP_FIELD_MAX),
            sig: INTEGER_BIT | QUIET_BIT | (payload & FRAC_MASK),
        },
        Outcome::Num(sign, Rounded::Zero) => F80 {
            sign_exp: sign_exp(sign, 0),
            sig: 0,
        },
        Outcome::Num(sign, Rounded::Inf) => F80 {
            sign_exp: sign_exp(sign, EXP_FIELD_MAX),
            sig: INTEGER_BIT,
        },
        Outcome::Num(sign, Rounded::Finite { exp, frac }) => {
            let msb = 63 - frac.leading_zeros();
            let lead = exp + msb as i32;
            if lead >= F80::SPEC.emin() {
                // A shortened precision leaves the low bits of the stored
                // significand zero, which is what the hardware does too.
                F80 {
                    sign_exp: sign_exp(sign, (lead + BIAS) as u16),
                    sig: frac << (63 - msb),
                }
            } else {
                F80 {
                    sign_exp: sign_exp(sign, 0),
                    sig: frac << (exp - F80::SPEC.min_ulp),
                }
            }
        }
    }
}

/// Run a two-operand kernel operation, handling the unsupported encodings.
fn binop(
    a: F80,
    b: F80,
    pc: Precision,
    env: Env,
    op: fn(Parts, Parts, Spec, Env) -> (Outcome, Flags),
) -> (F80, Flags) {
    let (pa, fa) = unpack(a, env);
    let (pb, fb) = unpack(b, env);
    let (Some(pa), Some(pb)) = (pa, pb) else {
        return (encode(Outcome::DefaultNan, env), Flags::INVALID | fa | fb);
    };
    let (out, f) = op(pa, pb, pc.spec(), env);
    (encode(out, env), f | fa | fb)
}

/// `a + b` at the given precision control.
pub fn add(a: F80, b: F80, pc: Precision, env: Env) -> (F80, Flags) {
    binop(a, b, pc, env, |x, y, s, e| kernel::add(x, y, false, s, e))
}

/// `a - b`.
pub fn sub(a: F80, b: F80, pc: Precision, env: Env) -> (F80, Flags) {
    binop(a, b, pc, env, |x, y, s, e| kernel::add(x, y, true, s, e))
}

/// `a * b`.
pub fn mul(a: F80, b: F80, pc: Precision, env: Env) -> (F80, Flags) {
    binop(a, b, pc, env, kernel::mul)
}

/// `a / b`.
pub fn div(a: F80, b: F80, pc: Precision, env: Env) -> (F80, Flags) {
    binop(a, b, pc, env, kernel::div)
}

/// The square root of `a`.
pub fn sqrt(a: F80, pc: Precision, env: Env) -> (F80, Flags) {
    let (pa, fa) = unpack(a, env);
    let Some(pa) = pa else {
        return (encode(Outcome::DefaultNan, env), Flags::INVALID | fa);
    };
    let (out, f) = kernel::sqrt(pa, pc.spec(), env);
    (encode(out, env), f | fa)
}

/// `a * b + c`, rounded once.
///
/// x87 has no fused multiply-add instruction; this is here because the format
/// is a first-class one and the kernel supports it, so an FMA extension over
/// 80-bit values has an implementation waiting rather than a gap.
pub fn fma(a: F80, b: F80, c: F80, pc: Precision, env: Env) -> (F80, Flags) {
    let (pa, fa) = unpack(a, env);
    let (pb, fb) = unpack(b, env);
    let (pc_parts, fc) = unpack(c, env);
    let (Some(pa), Some(pb), Some(pc_parts)) = (pa, pb, pc_parts) else {
        return (
            encode(Outcome::DefaultNan, env),
            Flags::INVALID | fa | fb | fc,
        );
    };
    let (out, f) = kernel::fma(pa, pb, pc_parts, pc.spec(), env);
    (encode(out, env), f | fa | fb | fc)
}

/// The ordering of two values, or `None` when they are unordered — which
/// includes an unsupported encoding, since it is not a value at all.
#[must_use]
pub fn compare(a: F80, b: F80) -> Option<core::cmp::Ordering> {
    let (pa, pb) = (decode(a)?, decode(b)?);
    if pa.class == Class::Nan || pb.class == Class::Nan {
        return None;
    }
    Some(kernel::compare(pa, pb))
}

/// Widen a binary32 or binary64 value into the extended format.
///
/// Always exact for a finite value — every interchange format this crate has
/// fits inside 64 bits of significand and the 15-bit exponent range — so no
/// rounding direction can matter and none is consulted.
pub fn from_binary<F: Format>(bits: u64, env: Env) -> (F80, Flags) {
    let p = super::binary::classify::<F>(bits);
    let sign = bits & F::SIGN != 0;
    let field = (bits >> F::SIG_BITS) & F::EXP_FIELD_MAX;
    let frac = bits & F::SIG_MASK;
    match p {
        super::Category::SignalingNan | super::Category::QuietNan => {
            let quiet = frac & F::QUIET != 0;
            // The payload keeps its position relative to the top of the
            // significand, which is how `FLD m32fp` widens one.
            let payload = (frac & !F::QUIET) << (63 - F::SIG_BITS);
            let out = if env.nan.propagate == super::Propagate::Default {
                Outcome::DefaultNan
            } else {
                Outcome::Nan { sign, payload }
            };
            let flags = if quiet { Flags::NONE } else { Flags::INVALID };
            (encode(out, env), flags)
        }
        super::Category::NegativeInfinity | super::Category::PositiveInfinity => {
            (encode(Outcome::Num(sign, Rounded::Inf), env), Flags::NONE)
        }
        super::Category::NegativeZero | super::Category::PositiveZero => {
            (encode(Outcome::Num(sign, Rounded::Zero), env), Flags::NONE)
        }
        _ => {
            let (frac, exp) = if field == 0 {
                (frac, F::SPEC.min_ulp)
            } else {
                (
                    frac | (1u64 << F::SIG_BITS),
                    field as i32 - F::BIAS - F::SIG_BITS as i32,
                )
            };
            let denorm = if field == 0 && env.subnormal_inputs.reports() {
                Flags::DENORMAL
            } else {
                Flags::NONE
            };
            let (out, f) = kernel::round_exact(sign, exp, u128::from(frac), F80::SPEC, env);
            (encode(out, env), f | denorm)
        }
    }
}

/// Narrow an extended value to binary32 or binary64, rounding once.
pub fn to_binary<F: Format>(v: F80, env: Env) -> (u64, Flags) {
    let (p, fin) = unpack(v, env);
    let Some(p) = p else {
        let sign = if env.nan.default_sign { F::SIGN } else { 0 };
        return (sign | F::QUIET_NAN, Flags::INVALID);
    };
    match p.class {
        Class::Nan => {
            let flags = if p.snan { Flags::INVALID } else { Flags::NONE };
            let bits = if env.nan.propagate == super::Propagate::Default {
                let sign = if env.nan.default_sign { F::SIGN } else { 0 };
                sign | F::QUIET_NAN
            } else {
                let sign = if p.sign { F::SIGN } else { 0 };
                let payload = (p.frac >> (63 - F::SIG_BITS)) & F::SIG_MASK;
                sign | F::INF | F::QUIET | payload
            };
            (bits, flags)
        }
        _ => {
            // The interchange path already encodes every finite, zero and
            // infinite case; reuse it by rounding the parts directly.
            let (out, f) = match p.class {
                Class::Inf => (Outcome::Num(p.sign, Rounded::Inf), Flags::NONE),
                Class::Zero => (Outcome::Num(p.sign, Rounded::Zero), Flags::NONE),
                _ => kernel::round_exact(p.sign, p.exp, u128::from(p.frac), F::SPEC, env),
            };
            (super::binary::encode_outcome::<F>(out, env), f | fin)
        }
    }
}

/// Convert to a signed integer `bits` wide, as `FIST`/`FISTP` do.
pub fn to_signed(v: F80, bits: u32, env: Env) -> (i64, Flags) {
    let (p, fin) = unpack(v, env);
    let max: u128 = (1u128 << (bits - 1)) - 1;
    let min_mag: u128 = 1u128 << (bits - 1);
    let indefinite = if bits >= 64 {
        min_mag as i64
    } else {
        ((min_mag as u64) << (64 - bits)) as i64 >> (64 - bits)
    };
    let Some(p) = p else {
        return (indefinite, Flags::INVALID);
    };
    let saturate = |sign: bool| {
        if env.int_overflow == IntOverflow::Indefinite || sign {
            indefinite
        } else {
            max as i64
        }
    };
    match kernel::to_integer(p, env) {
        IntValue::Nan => {
            let v = match env.int_overflow {
                IntOverflow::SaturateNanMax => max as i64,
                IntOverflow::SaturateNanZero => 0,
                IntOverflow::Indefinite => indefinite,
            };
            (v, Flags::INVALID | fin)
        }
        IntValue::Inf(sign) => (saturate(sign), Flags::INVALID | fin),
        IntValue::Value {
            sign,
            magnitude,
            inexact,
        } => {
            let fits = if sign {
                magnitude <= min_mag
            } else {
                magnitude <= max
            };
            if !fits {
                return (saturate(sign), Flags::INVALID | fin);
            }
            let v = if sign {
                let neg = (magnitude as u64).wrapping_neg();
                if bits >= 64 {
                    neg as i64
                } else {
                    ((neg << (64 - bits)) as i64) >> (64 - bits)
                }
            } else {
                magnitude as i64
            };
            let f = if inexact { Flags::INEXACT } else { Flags::NONE };
            (v, f | fin)
        }
    }
}

/// Convert from a signed integer `bits` wide, as `FILD` does.
///
/// Exact for every width x87 loads: 64 significand bits hold any `i64`.
pub fn from_signed(value: i64, bits: u32, env: Env) -> (F80, Flags) {
    let v = if bits >= 64 {
        value
    } else {
        ((value as u64) << (64 - bits)) as i64 >> (64 - bits)
    };
    if v == 0 {
        return (F80::ZERO, Flags::NONE);
    }
    let (out, f) = kernel::round_exact(v < 0, 0, u128::from(v.unsigned_abs()), F80::SPEC, env);
    (encode(out, env), f)
}
