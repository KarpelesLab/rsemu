//! Differential tests against the host FPU — values **and flags**, in all five
//! rounding directions.
//!
//! `tests.rs` uses the host only for round-to-nearest-even values, because
//! there is no safe way to read the host's status register or to change its
//! rounding mode. This file gets both anyway, for binary32, by doing the
//! host's arithmetic one format up:
//!
//! * The exact result of a binary32 `mul` is a binary64 value (24 + 24
//!   significand bits, an exponent no smaller than 2^-298), so the host's
//!   `f64` multiply *is* the infinitely precise result of IEEE 754-2019 §5.1.
//! * The exact result of `add`, `sub` and `fma` is not always a binary64, but
//!   its **round-to-odd** image is computable with one error-free
//!   transformation (Knuth's TwoSum), and rounding a 53-bit round-to-odd image
//!   to 24 bits gives the same answer in every direction as rounding the exact
//!   value (Boldo and Melquiond, "Emulation of a FMA and correctly-rounded
//!   sums: proved algorithms using rounding to odd", 2008).
//! * A binary32 quotient or square root that is inexact differs from every
//!   binary32 value, and from every midpoint between two of them, by a relative
//!   2^-49 or more — far more than binary64's half-ulp — so the host's `f64`
//!   quotient lies strictly inside the same binary32 interval as the exact one.
//!
//! From the exact value everything else follows by the standard's own
//! definitions, with no second implementation to trust: the two binary32
//! neighbours are the host's nearest-even conversion and its successor or
//! predecessor, the direction picks one, inexact is "the result differs"
//! (§7.6), overflow is §7.4, and tininess is evaluated both ways §7.5 allows —
//! after rounding by repeating the rounding with the value scaled up into the
//! middle of the exponent range, which is "as though the exponent range were
//! unbounded" made literal.
//!
//! binary64 gets the nearest-even flags that fused multiply-add residuals can
//! decide exactly. The kernel is one implementation over every format, so the
//! directed-rounding and underflow coverage above is coverage of binary64 too.

use super::*;

/// The same xorshift `tests.rs` uses, so a failure reproduces.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    /// A finite binary32, weighted toward the places rounding goes wrong:
    /// subnormals, both ends of the exponent range, and significands that are
    /// all ones, a single bit, or one ulp from a power of two.
    fn f32(&mut self) -> f32 {
        let r = self.next();
        let exp = match r & 7 {
            0 => 0,
            1 => 1 + ((r >> 8) % 3) as u32,
            2 => 254 - ((r >> 8) % 3) as u32,
            3 => 120 + ((r >> 8) % 16) as u32,
            _ => ((r >> 8) % 255) as u32,
        };
        let sig = self.sig(23) as u32;
        let sign = ((r >> 3) & 1) as u32;
        f32::from_bits((sign << 31) | (exp << 23) | sig)
    }

    /// A significand field `bits` wide, from the same boundary-heavy mix.
    fn sig(&mut self, bits: u32) -> u64 {
        let r = self.next();
        let mask = (1u64 << bits) - 1;
        match r & 7 {
            0 => 0,
            1 => 1,
            2 => mask,
            3 => mask - 1,
            4 => 1 << ((r >> 8) % u64::from(bits)),
            5 => mask ^ (1 << ((r >> 8) % u64::from(bits))),
            _ => (r >> 16) & mask,
        }
    }

    /// A normal binary32 whose exponent is chosen so that `a * b` (or `a / b`
    /// with `quotient`) lands at binary exponent `target` or one above it.
    fn partner(&mut self, a: f32, target: i32, quotient: bool) -> Option<f32> {
        let ea = ((a.to_bits() >> 23) & 0xff) as i32 - 127;
        let eb = if quotient { ea - target } else { target - ea };
        let field = eb + 127;
        if !(1..=254).contains(&field) || a == 0.0 {
            return None;
        }
        let r = self.next();
        let sig = self.sig(23) as u32;
        Some(f32::from_bits(
            (((r & 1) as u32) << 31) | ((field as u32) << 23) | sig,
        ))
    }
}

/// 2^128 as a binary64: one past binary32's largest finite magnitude.
const TWO_128: f64 = f64::from_bits(0x47f0_0000_0000_0000);
/// 2^64, the scale that moves a tiny binary32 result into the normal range.
const TWO_64: f64 = f64::from_bits(0x43f0_0000_0000_0000);

/// Round a finite, non-zero exact value to binary32 in a given direction.
///
/// The two candidates are the host's nearest-even conversion and its
/// neighbour on the other side of `v`; the direction only chooses.
fn round32(v: f64, round: Round) -> f32 {
    let nearest = (v as f32).abs();
    let mag = v.abs();
    let (lo, hi) = if nearest.is_infinite() {
        (f32::MAX, f32::INFINITY)
    } else if f64::from(nearest) == mag {
        return v as f32;
    } else if f64::from(nearest) < mag {
        (nearest, nearest.next_up())
    } else {
        (nearest.next_down(), nearest)
    };
    let negative = v < 0.0;
    let pick_hi = match round {
        Round::TiesEven => nearest == hi,
        Round::TiesAway => {
            let hi = if hi.is_infinite() {
                TWO_128
            } else {
                f64::from(hi)
            };
            hi - mag <= mag - f64::from(lo)
        }
        Round::TowardZero => false,
        Round::TowardNegative => negative,
        Round::TowardPositive => !negative,
    };
    let m = if pick_hi { hi } else { lo };
    if negative { -m } else { m }
}

/// What IEEE 754-2019 says a binary32 operation with exact result `v`
/// delivers and signals. `v` is finite and non-zero.
fn expect32(v: f64, round: Round, tininess: Tininess) -> (u64, Flags) {
    let r = round32(v, round);
    let mut flags = Flags::NONE;
    if f64::from(r) != v {
        flags |= Flags::INEXACT;
    }
    // §7.4: overflow is decided on the result rounded with an unbounded
    // exponent, so a value past 2^128 overflows even when the direction then
    // delivers the largest finite number.
    if r.is_infinite() || v.abs() >= TWO_128 {
        flags |= Flags::OVERFLOW;
    }
    if v.abs() < f64::from(f32::MIN_POSITIVE) {
        let tiny = match tininess {
            Tininess::BeforeRounding => true,
            // §7.5: "when a non-zero result computed as though the exponent
            // range were unbounded would lie strictly between ±b^emin".
            // Scaling by 2^64 is exact and leaves the precision at 24 bits.
            Tininess::AfterRounding => {
                f64::from(round32(v * TWO_64, round).abs()) < f64::from(f32::MIN_POSITIVE) * TWO_64
            }
        };
        if tiny && flags.contains(Flags::INEXACT) {
            flags |= Flags::UNDERFLOW;
        }
    }
    (u64::from(r.to_bits()), flags)
}

/// `x + y` rounded to odd: exact when the sum is a binary64, and otherwise the
/// neighbour whose last bit is set.
fn add_to_odd(x: f64, y: f64) -> f64 {
    let s = x + y;
    // Knuth's TwoSum: `err` is exactly `(x + y) - s`.
    let bb = s - x;
    let err = (x - (s - bb)) + (y - bb);
    if err == 0.0 || s.to_bits() & 1 == 1 {
        return s;
    }
    // The exact sum is strictly between `s` and one neighbour; `s` is even,
    // so that neighbour is the odd one.
    if (err > 0.0) == (s > 0.0) {
        f64::from_bits(s.to_bits() + 1)
    } else {
        f64::from_bits(s.to_bits() - 1)
    }
}

/// An integer rounded to odd as a binary64.
fn int_to_odd(i: i128) -> f64 {
    let f = i as f64;
    let back = f as i128;
    if back == i || f.to_bits() & 1 == 1 {
        return f;
    }
    if (i > back) == (f > 0.0) {
        f64::from_bits(f.to_bits() + 1)
    } else {
        f64::from_bits(f.to_bits() - 1)
    }
}

const ROUNDS: [Round; 5] = [
    Round::TiesEven,
    Round::TiesAway,
    Round::TowardZero,
    Round::TowardNegative,
    Round::TowardPositive,
];

/// The two environments that differ only in what this file can check: RISC-V
/// detects tininess after rounding and ARM before. Neither reports a subnormal
/// operand, and no operand here is a NaN.
const ENVS: [Env; 2] = [Env::RISCV, Env::ARM];

fn b(v: f32) -> u64 {
    u64::from(v.to_bits())
}

/// Check one exact value against one delivered result, in every direction.
fn check(what: &str, v: f64, got: impl Fn(Env) -> (u64, Flags)) {
    if v == 0.0 || !v.is_finite() {
        return;
    }
    for env in ENVS {
        for round in ROUNDS {
            let env = env.round(round);
            let want = expect32(v, round, env.tininess);
            let (bits, flags) = got(env);
            assert_eq!(
                (bits, flags),
                want,
                "{what}: exact {v:e} ({:#018x}) {round:?} {:?}: got {bits:#010x}, want {:#010x}",
                v.to_bits(),
                env.tininess,
                want.0
            );
        }
    }
}

fn check_all(a: f32, y: f32, c: f32) {
    let (a64, b64, c64) = (f64::from(a), f64::from(y), f64::from(c));
    let name = |op: &str| {
        alloc::format!(
            "{op} a={:#010x} b={:#010x} c={:#010x}",
            a.to_bits(),
            y.to_bits(),
            c.to_bits()
        )
    };
    check(&name("mul"), a64 * b64, |env| mul::<B32>(b(a), b(y), env));
    check(&name("add"), add_to_odd(a64, b64), |env| {
        add::<B32>(b(a), b(y), env)
    });
    check(&name("sub"), add_to_odd(a64, -b64), |env| {
        sub::<B32>(b(a), b(y), env)
    });
    if y != 0.0 {
        check(&name("div"), a64 / b64, |env| div::<B32>(b(a), b(y), env));
    }
    if a > 0.0 {
        check(&name("sqrt"), a64.sqrt(), |env| sqrt::<B32>(b(a), env));
    }
    check(&name("fma"), add_to_odd(a64 * b64, c64), |env| {
        fma::<B32>(b(a), b(y), b(c), env)
    });
}

#[test]
fn binary32_matches_the_host_in_every_direction_flags_included() {
    let mut rng = Rng(0x0123_4567_89ab_cdef);
    for _ in 0..40_000 {
        let (a, y, c) = (rng.f32(), rng.f32(), rng.f32());
        check_all(a, y, c);
    }
}

#[test]
fn binary32_matches_the_host_around_the_underflow_and_overflow_thresholds() {
    // Random operands almost never produce a result within a binade of
    // 2^-126 or 2^128, which is where tininess and overflow are decided, so
    // the second operand's exponent is solved for.
    let mut rng = Rng(0xfeed_face_0bad_f00d);
    let targets = [
        -151, -150, -149, -140, -128, -127, -126, -125, 126, 127, 128,
    ];
    for _ in 0..20_000 {
        let a = rng.f32();
        let c = rng.f32();
        for target in targets {
            if let Some(y) = rng.partner(a, target, false) {
                let (a64, b64) = (f64::from(a), f64::from(y));
                check("mul", a64 * b64, |env| mul::<B32>(b(a), b(y), env));
                // An addend at the threshold, of either sign, is what makes
                // a fused result tiny before rounding and not after.
                let edge = f32::from_bits(0x0080_0000 | (c.to_bits() & 0x8000_0003));
                check("fma", add_to_odd(a64 * b64, f64::from(edge)), |env| {
                    fma::<B32>(b(a), b(y), b(edge), env)
                });
            }
            if let Some(y) = rng.partner(a, target, true) {
                check("div", f64::from(a) / f64::from(y), |env| {
                    div::<B32>(b(a), b(y), env)
                });
            }
        }
    }
}

#[test]
fn narrowing_matches_the_host_in_every_direction_flags_included() {
    let mut rng = Rng(0x5eed_5eed_5eed_5eed);
    for _ in 0..60_000 {
        let r = rng.next();
        // Half the sample is spread over binary64, half is packed into
        // binary32's own range where the conversion actually rounds.
        let exp = if r & 1 == 0 {
            (r >> 8) % 0x7ff
        } else {
            1023 - 152 + (r >> 8) % 284
        };
        let bits = ((r >> 1) & 1) << 63 | exp << 52 | rng.sig(52);
        let v = f64::from_bits(bits);
        check("narrow", v, |env| convert::<B64, B32>(bits, env));
    }
}

#[test]
fn integers_convert_like_the_host_in_every_direction_flags_included() {
    let mut rng = Rng(0x243f_6a88_85a3_08d3);
    for _ in 0..40_000 {
        let r = rng.next();
        // A random width, so the sample is not all 64-bit magnitudes.
        let x = rng.next() >> (r % 64);
        let x = match (r >> 8) & 3 {
            0 => x,
            1 => x.wrapping_neg(),
            2 => x | 1,
            _ => (1u64 << ((r >> 16) % 64))
                .wrapping_add((r >> 24) % 5)
                .wrapping_sub(2),
        };
        check("u64", int_to_odd(i128::from(x)), |env| {
            from_unsigned::<B32>(x, 64, env)
        });
        check("i64", int_to_odd(i128::from(x as i64)), |env| {
            from_signed::<B32>(x as i64, 64, env)
        });
        check("u32", f64::from(x as u32), |env| {
            from_unsigned::<B32>(x, 32, env)
        });
        check("i32", f64::from(x as i32), |env| {
            from_signed::<B32>(x as i64, 32, env)
        });
        // Fixed point: the scale is a power of two, so it is exact here too.
        let fbits = (r >> 32) as u32 % 64;
        let scale = f64::from_bits(u64::from(1023 - fbits) << 52);
        check("ufixed", int_to_odd(i128::from(x)) * scale, |env| {
            from_unsigned_fixed::<B32>(x, 64, fbits, env)
        });
    }
}

/// Round a binary64 to an integral value in a direction, on the host.
fn integral(v: f64, round: Round) -> f64 {
    match round {
        Round::TiesEven => v.round_ties_even(),
        Round::TiesAway => v.round(),
        Round::TowardZero => v.trunc(),
        Round::TowardNegative => v.floor(),
        Round::TowardPositive => v.ceil(),
    }
}

#[test]
fn float_to_integer_matches_the_host_at_every_width_and_direction() {
    // RISC-V's rule is Rust's `as`: saturate, except that a NaN is the most
    // positive value rather than zero, and no operand here is a NaN.
    let mut rng = Rng(0xc0ff_ee00_c0ff_ee00);
    let check_one = |v: f64, got: &dyn Fn(Env, u32) -> ((i64, Flags), (u64, Flags)), what: &str| {
        for round in ROUNDS {
            let env = Env::RISCV.round(round);
            let r = integral(v, round);
            for bits in [32u32, 64] {
                let (signed, unsigned) = got(env, bits);
                let (lo, hi) = if bits == 32 {
                    (-2147483648.0, 2147483648.0)
                } else {
                    (-9223372036854775808.0, 9223372036854775808.0)
                };
                let exact = if r == v { Flags::NONE } else { Flags::INEXACT };
                let want_s = if r >= lo && r < hi {
                    (r as i64, exact)
                } else if bits == 32 {
                    (i64::from(r as i32), Flags::INVALID)
                } else {
                    (r as i64, Flags::INVALID)
                };
                assert_eq!(signed, want_s, "{what} {v:e} -> i{bits} {round:?}");
                let want_u = if r >= 0.0 && r < hi * 2.0 {
                    (r as u64, exact)
                } else if bits == 32 {
                    (u64::from(r as u32), Flags::INVALID)
                } else {
                    (r as u64, Flags::INVALID)
                };
                assert_eq!(unsigned, want_u, "{what} {v:e} -> u{bits} {round:?}");
            }
        }
    };
    for _ in 0..40_000 {
        let r = rng.next();
        // Exponents from 2^-3 to 2^66 cover every rounding case and both
        // range boundaries; the rest of the sample is anywhere at all.
        let exp = if r & 3 != 0 {
            1020 + (r >> 8) % 70
        } else {
            (r >> 8) % 0x7ff
        };
        let bits = ((r >> 2) & 1) << 63 | exp << 52 | rng.sig(52);
        let v = f64::from_bits(bits);
        check_one(
            v,
            &|env, w| {
                (
                    to_signed::<B64>(bits, w, env),
                    to_unsigned::<B64>(bits, w, env),
                )
            },
            "f64",
        );
        let exp = if r & 3 != 0 {
            124 + (r >> 8) % 70
        } else {
            (r >> 8) % 0xff
        };
        let bits32 = ((r >> 2) & 1) << 31 | exp << 23 | rng.sig(23);
        let v = f64::from(f32::from_bits(bits32 as u32));
        check_one(
            v,
            &|env, w| {
                (
                    to_signed::<B32>(bits32, w, env),
                    to_unsigned::<B32>(bits32, w, env),
                )
            },
            "f32",
        );
    }
}

#[test]
fn binary64_flags_match_what_the_host_can_prove() {
    // A fused multiply-add computes the residual of a product, a quotient or
    // a square root exactly whenever that residual is representable, which it
    // is once the operands are kept away from the ends of the exponent range.
    // Zero residual is "exact" and anything else is "inexact" (§7.6).
    let mut rng = Rng(0x1319_8a2e_0370_7344);
    let flag = |inexact: bool| {
        if inexact { Flags::INEXACT } else { Flags::NONE }
    };
    for _ in 0..40_000 {
        let r = rng.next();
        let make = |r: u64, sig: u64| {
            f64::from_bits(((r & 1) << 63) | ((1023 - 200 + (r >> 8) % 400) << 52) | sig)
        };
        let a = make(r, rng.sig(52));
        let y = make(r >> 20, rng.sig(52));
        let (ab, yb) = (a.to_bits(), y.to_bits());

        let p = a * y;
        assert_eq!(
            mul::<B64>(ab, yb, Env::RISCV),
            (p.to_bits(), flag(a.mul_add(y, -p) != 0.0)),
            "{a:e} * {y:e}"
        );
        let q = a / y;
        assert_eq!(
            div::<B64>(ab, yb, Env::RISCV),
            (q.to_bits(), flag(q.mul_add(y, -a) != 0.0)),
            "{a:e} / {y:e}"
        );
        let s = a + y;
        let bb = s - a;
        let err = (a - (s - bb)) + (y - bb);
        if s != 0.0 {
            assert_eq!(
                add::<B64>(ab, yb, Env::RISCV),
                (s.to_bits(), flag(err != 0.0)),
                "{a:e} + {y:e}"
            );
        }
        let root = a.abs().sqrt();
        assert_eq!(
            sqrt::<B64>(ab & !B64::SIGN, Env::RISCV),
            (root.to_bits(), flag(root.mul_add(root, -a.abs()) != 0.0)),
            "sqrt({a:e})"
        );
    }
}
