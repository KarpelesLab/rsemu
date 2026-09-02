//! The soft-float test suite.
//!
//! # Where the oracle stops being an oracle
//!
//! Three kinds of check live here, and they cover different things:
//!
//! 1. **Directed vectors from IEEE 754-2019** and from each guest's manual.
//!    These are the authority for everything the standard leaves to the
//!    implementation: NaN payloads, the default NaN's sign, the flags, the
//!    rounding direction of an overflow, tininess detection.
//! 2. **The host FPU**, in `agrees_with_the_host_on_ordinary_values`. It is a
//!    legitimate oracle **only** for round-to-nearest-even `add`/`sub`/`mul`/
//!    `div`/`sqrt`/`fma` on finite operands with a finite result, because those
//!    are the cases where IEEE-754 makes the answer unique and Rust's `f64`
//!    is an IEEE-754 binary64 that is not flushing subnormals. It is **not** an
//!    oracle for: NaN payloads (the host canonicalises differently per
//!    architecture, and wasm canonicalises harder), the exception flags (there
//!    is no way to read the host's `MXCSR`/`FPSR` without `libc` or inline
//!    assembly, both of which this crate forbids), any rounding mode other than
//!    nearest-even (setting the host mode needs the same forbidden tools), and
//!    the 80-bit format (Rust has no `long double` type at all — see below).
//! 3. **The definition of correct rounding itself**, in `assert_nearest_even`:
//!    exact integer arithmetic verifying that the delivered result is nearer to
//!    the exact value than either neighbour, ties to even (§5.1, §4.3.1). This
//!    is the strongest of the three because it compares against the standard
//!    rather than against another implementation, and it is what tests the
//!    80-bit path's arithmetic.
//!
//! ## The 80-bit path has no host oracle, not even on x86
//!
//! The obvious oracle for extended precision is C's `long double` on x86 Linux.
//! Rust has no such type — `f128` is a different format and is not the x87
//! register format — and reaching a C one would need FFI, which `ROADMAP.md`
//! §0 forbids outright. So the 80-bit checks are: the correct-rounding property
//! above; directed vectors for the encodings x87 alone has (the explicit
//! integer bit, pseudo-denormals, unnormals, pseudo-NaNs); and a **transitive**
//! check — with precision control set to 53 bits and operands well inside
//! binary64's exponent range, an x87 result must equal the binary64 result
//! bit for bit, and binary64 is host-checked. That chain does not reach the
//! full 64-bit significand, which is why the property check exists.
//!
//! Host `f32`/`f64` also appear as a *notation* for writing expectations
//! (`d(1.5)`); that is the test file's own business and does not touch the
//! implementation, which `no_host_float_in_the_implementation` proves.

use super::x87::{F80, Precision, X87Class};
use super::*;

// ---------------------------------------------------------------------------
// Notation
// ---------------------------------------------------------------------------

/// Host `f64` bits, used only to write readable expectations.
fn d(v: f64) -> u64 {
    v.to_bits()
}

/// Host `f32` bits, in the low half of a `u64`.
fn s(v: f32) -> u64 {
    u64::from(v.to_bits())
}

/// RISC-V, round to nearest, ties to even — the environment the ported tests
/// were written against.
const RV: Env = Env::RISCV;

/// The smallest positive binary64 subnormal, 2^-1074.
const MIN_SUB64: u64 = 1;

/// The smallest positive binary64 normal, 2^-1022.
const MIN_NORM64: u64 = 0x0010_0000_0000_0000;

// ---------------------------------------------------------------------------
// The ported RISC-V suite: same assertions, new spelling of the mode
// ---------------------------------------------------------------------------

#[test]
fn addition_is_exact_where_it_should_be() {
    assert_eq!(add::<B64>(d(1.0), d(2.0), RV), (d(3.0), Flags::NONE));
    assert_eq!(add::<B64>(d(0.5), d(0.25), RV), (d(0.75), Flags::NONE));
    assert_eq!(sub::<B64>(d(1.0), d(1.0), RV), (d(0.0), Flags::NONE));
    // x - x is -0 only when rounding down (IEEE 754-2019 §6.3).
    assert_eq!(
        sub::<B64>(d(1.0), d(1.0), RV.round(Round::TowardNegative)),
        (d(-0.0), Flags::NONE)
    );
}

#[test]
fn addition_rounds_to_nearest_even() {
    // 1 + 2^-53 is exactly halfway; ties-to-even keeps 1.0.
    let tiny = 0x3ca0_0000_0000_0000u64;
    let (v, f) = add::<B64>(d(1.0), tiny, RV);
    assert_eq!(v, d(1.0));
    assert_eq!(f, Flags::INEXACT);
    let (v, _) = add::<B64>(d(1.0), tiny, RV.round(Round::TowardPositive));
    assert_eq!(v, d(1.0) + 1);
}

#[test]
fn cancellation_is_exact() {
    // The case a sticky-only aligner gets wrong.
    let a = d(1.0);
    let b = 0x3ca0_0000_0000_0001u64;
    let (v, f) = sub::<B64>(a, b, RV);
    assert_eq!(f, Flags::INEXACT);
    assert!(v < a);
}

#[test]
fn subnormals_survive() {
    assert_eq!(
        add::<B64>(MIN_SUB64, MIN_SUB64, RV),
        (2, Flags::NONE),
        "an exact subnormal sum is exact, and is not an underflow"
    );
    // Halving the smallest subnormal underflows to zero, inexactly.
    let (v, f) = mul::<B64>(MIN_SUB64, d(0.5), RV);
    assert_eq!(v, 0);
    assert_eq!(f, Flags::INEXACT | Flags::UNDERFLOW);
}

#[test]
fn multiplication_keeps_the_whole_product() {
    assert_eq!(mul::<B64>(d(3.0), d(7.0), RV), (d(21.0), Flags::NONE));
    assert_eq!(mul::<B32>(s(3.0), s(0.5), RV), (s(1.5), Flags::NONE));
    let (v, f) = mul::<B64>(d(0.0), d(f64::INFINITY), RV);
    assert_eq!(v, B64::QUIET_NAN);
    assert_eq!(f, Flags::INVALID);
}

#[test]
fn division_reports_divide_by_zero() {
    assert_eq!(div::<B64>(d(1.0), d(2.0), RV), (d(0.5), Flags::NONE));
    let (v, f) = div::<B64>(d(1.0), d(0.0), RV);
    assert_eq!(v, d(f64::INFINITY));
    assert_eq!(f, Flags::DIV_BY_ZERO);
    let (v, f) = div::<B64>(d(0.0), d(0.0), RV);
    assert_eq!(v, B64::QUIET_NAN);
    assert_eq!(f, Flags::INVALID);
    let (v, f) = div::<B64>(d(1.0), d(3.0), RV);
    assert_eq!(v, d(1.0f64 / 3.0));
    assert_eq!(f, Flags::INEXACT);
}

#[test]
fn square_root_is_correctly_rounded() {
    assert_eq!(sqrt::<B64>(d(4.0), RV), (d(2.0), Flags::NONE));
    assert_eq!(sqrt::<B64>(d(0.25), RV), (d(0.5), Flags::NONE));
    assert_eq!(sqrt::<B32>(s(9.0), RV), (s(3.0), Flags::NONE));
    let (v, f) = sqrt::<B64>(d(2.0), RV);
    assert_eq!(v, d(core::f64::consts::SQRT_2));
    assert_eq!(f, Flags::INEXACT);
    let (v, f) = sqrt::<B64>(d(-1.0), RV);
    assert_eq!(v, B64::QUIET_NAN);
    assert_eq!(f, Flags::INVALID);
    // §5.4.1: sqrt(-0) is -0, and is not an invalid operation.
    assert_eq!(sqrt::<B64>(d(-0.0), RV), (d(-0.0), Flags::NONE));
}

#[test]
fn fused_multiply_add_rounds_once() {
    // (1 + 2^-52)(1 - 2^-52) is 1 - 2^-104: exact in 106 bits, and
    // indistinguishable from 1.0 once rounded. So the fused result is
    // -2^-104 away from 1 and the separately rounded one is zero, which is the
    // entire point of the instruction.
    let a = d(1.0) + 1;
    let b = d(1.0) - 2;
    let (fused, f) = fma::<B64>(a, b, d(-1.0), RV);
    assert_eq!(fused, (1u64 << 63) | (919u64 << 52));
    assert_eq!(f, Flags::NONE);
    let (rounded, _) = mul::<B64>(a, b, RV);
    let (separate, _) = add::<B64>(rounded, d(-1.0), RV);
    assert_eq!(separate, d(0.0));
    assert_eq!(
        fma::<B64>(d(2.0), d(3.0), d(4.0), RV),
        (d(10.0), Flags::NONE)
    );
    assert_eq!(
        fma::<B64>(d(2.0), d(3.0), d(0.0), RV),
        (d(6.0), Flags::NONE)
    );
    let (v, f) = fma::<B64>(d(0.0), d(f64::INFINITY), d(1.0), RV);
    assert_eq!(v, B64::QUIET_NAN);
    assert_eq!(f, Flags::INVALID);
}

#[test]
fn comparisons_distinguish_quiet_from_signaling() {
    let qnan = B64::QUIET_NAN;
    let snan = B64::INF | 1;
    assert_eq!(eq::<B64>(qnan, d(1.0)), (false, Flags::NONE));
    assert_eq!(eq::<B64>(snan, d(1.0)), (false, Flags::INVALID));
    assert_eq!(lt::<B64>(qnan, d(1.0)), (false, Flags::INVALID));
    assert_eq!(lt::<B64>(d(-1.0), d(1.0)), (true, Flags::NONE));
    assert_eq!(le::<B64>(d(0.0), d(-0.0)), (true, Flags::NONE));
    assert_eq!(eq::<B64>(d(0.0), d(-0.0)), (true, Flags::NONE));
    assert_eq!(compare::<B64>(qnan, d(1.0)), None);
    assert_eq!(
        compare::<B64>(MIN_SUB64, d(0.0)),
        Some(core::cmp::Ordering::Greater),
        "a subnormal is larger than zero, which a sloppy magnitude order gets wrong"
    );
    assert_eq!(
        compare::<B64>(d(-1.0), MIN_SUB64 | B64::SIGN),
        Some(core::cmp::Ordering::Less)
    );
}

#[test]
fn min_max_follow_the_riscv_rules() {
    let qnan = B64::QUIET_NAN;
    assert_eq!(min::<B64>(qnan, d(1.0), RV), (d(1.0), Flags::NONE));
    assert_eq!(max::<B64>(qnan, d(1.0), RV), (d(1.0), Flags::NONE));
    assert_eq!(min::<B64>(qnan, qnan, RV), (qnan, Flags::NONE));
    assert_eq!(min::<B64>(d(-0.0), d(0.0), RV), (d(-0.0), Flags::NONE));
    assert_eq!(max::<B64>(d(-0.0), d(0.0), RV), (d(0.0), Flags::NONE));
    // A signaling operand raises invalid whichever value wins (§7.2).
    let snan = B64::INF | 1;
    assert_eq!(min::<B64>(snan, d(1.0), RV), (d(1.0), Flags::INVALID));
}

#[test]
fn classification_covers_every_class() {
    let fclass = |b| classify::<B64>(b).riscv_fclass();
    assert_eq!(fclass(d(f64::NEG_INFINITY)), 1 << 0);
    assert_eq!(fclass(d(-1.0)), 1 << 1);
    assert_eq!(fclass(B64::SIGN | 1), 1 << 2);
    assert_eq!(fclass(d(-0.0)), 1 << 3);
    assert_eq!(fclass(d(0.0)), 1 << 4);
    assert_eq!(fclass(1), 1 << 5);
    assert_eq!(fclass(d(1.0)), 1 << 6);
    assert_eq!(fclass(d(f64::INFINITY)), 1 << 7);
    assert_eq!(fclass(B64::INF | 1), 1 << 8);
    assert_eq!(fclass(B64::QUIET_NAN), 1 << 9);
    // The class itself is IEEE 754-2019 §5.7.2's, and does not depend on the
    // guest: only the encoding above does.
    assert_eq!(classify::<B32>(s(-0.0)), Category::NegativeZero);
    assert_eq!(classify::<B32>(1), Category::PositiveSubnormal);
}

#[test]
fn format_conversion_round_trips() {
    assert_eq!(convert::<B32, B64>(s(1.5), RV), (d(1.5), Flags::NONE));
    assert_eq!(convert::<B64, B32>(d(1.5), RV), (s(1.5), Flags::NONE));
    let (v, f) = convert::<B64, B32>(d(1e300), RV);
    assert_eq!(v, s(f32::INFINITY));
    assert_eq!(f, Flags::OVERFLOW | Flags::INEXACT);
}

#[test]
fn integer_conversion_saturates_instead_of_trapping() {
    let rtz = RV.round(Round::TowardZero);
    assert_eq!(to_signed::<B64>(d(1.9), 32, rtz), (1, Flags::INEXACT));
    assert_eq!(to_signed::<B64>(d(-1.9), 32, rtz), (-1, Flags::INEXACT));
    assert_eq!(to_signed::<B64>(d(1.5), 32, RV), (2, Flags::INEXACT));
    assert_eq!(to_signed::<B64>(d(2.5), 32, RV), (2, Flags::INEXACT));
    assert_eq!(
        to_signed::<B64>(d(2.5), 32, RV.round(Round::TiesAway)),
        (3, Flags::INEXACT)
    );
    assert_eq!(
        to_signed::<B64>(d(1e30), 32, rtz),
        (i64::from(i32::MAX), Flags::INVALID)
    );
    assert_eq!(
        to_signed::<B64>(B64::QUIET_NAN, 32, rtz),
        (i64::from(i32::MAX), Flags::INVALID)
    );
    assert_eq!(
        to_signed::<B64>(d(-1e30), 64, rtz),
        (i64::MIN, Flags::INVALID)
    );
    assert_eq!(
        to_signed::<B64>(d(-2147483648.0), 32, rtz),
        (i64::from(i32::MIN), Flags::NONE)
    );
    assert_eq!(to_unsigned::<B64>(d(-0.5), 32, rtz), (0, Flags::INEXACT));
    assert_eq!(to_unsigned::<B64>(d(-1.5), 32, rtz), (0, Flags::INVALID));
    assert_eq!(
        to_unsigned::<B64>(d(4294967295.0), 32, rtz),
        (0xffff_ffff, Flags::NONE)
    );
}

#[test]
fn integer_to_float_rounds() {
    assert_eq!(from_signed::<B64>(-3, 32, RV), (d(-3.0), Flags::NONE));
    assert_eq!(from_unsigned::<B64>(3, 32, RV), (d(3.0), Flags::NONE));
    let (v, f) = from_signed::<B64>((1i64 << 53) + 1, 64, RV);
    assert_eq!(v, d(9007199254740992.0));
    assert_eq!(f, Flags::INEXACT);
    assert_eq!(
        from_signed::<B64>(i64::from(i32::MIN), 32, RV).1,
        Flags::NONE
    );
    assert_eq!(from_signed::<B32>(16_777_217, 32, RV).1, Flags::INEXACT);
}

#[test]
fn overflow_depends_on_the_rounding_mode() {
    let big = d(f64::MAX);
    let (v, f) = add::<B64>(big, big, RV);
    assert_eq!(v, d(f64::INFINITY));
    assert_eq!(f, Flags::OVERFLOW | Flags::INEXACT);
    assert_eq!(
        add::<B64>(big, big, RV.round(Round::TowardZero)).0,
        d(f64::MAX)
    );
    assert_eq!(
        add::<B64>(big, big, RV.round(Round::TowardNegative)).0,
        d(f64::MAX)
    );
    // Downward, a negative overflow does reach infinity.
    assert_eq!(
        add::<B64>(
            big | B64::SIGN,
            big | B64::SIGN,
            RV.round(Round::TowardNegative)
        )
        .0,
        d(f64::NEG_INFINITY)
    );
}

// ---------------------------------------------------------------------------
// Rounding: all five attributes, on the same value
// ---------------------------------------------------------------------------

#[test]
fn every_rounding_attribute_on_one_halfway_case() {
    // 1 + 2^-53 sits exactly halfway between 1 and the next double, so it
    // separates all five of IEEE 754-2019 §4.3's attributes at once.
    let half = 0x3ca0_0000_0000_0000u64;
    let up = d(1.0) + 1;
    let cases = [
        (Round::TiesEven, d(1.0)),
        (Round::TiesAway, up),
        (Round::TowardZero, d(1.0)),
        (Round::TowardNegative, d(1.0)),
        (Round::TowardPositive, up),
    ];
    for (mode, want) in cases {
        let (v, f) = add::<B64>(d(1.0), half, RV.round(mode));
        assert_eq!(v, want, "{mode:?}");
        assert_eq!(f, Flags::INEXACT, "{mode:?}");
    }
    // The negative of the same case: only the two directed modes swap.
    let neg = [
        (Round::TiesEven, d(-1.0)),
        (Round::TiesAway, up | B64::SIGN),
        (Round::TowardZero, d(-1.0)),
        (Round::TowardNegative, up | B64::SIGN),
        (Round::TowardPositive, d(-1.0)),
    ];
    for (mode, want) in neg {
        let (v, _) = sub::<B64>(d(-1.0), half, RV.round(mode));
        assert_eq!(v, want, "{mode:?}");
    }
}

#[test]
fn ties_to_even_and_ties_to_away_differ_only_on_a_tie() {
    // 2.5 and 3.5 rounded to integers: nearest-even gives 2 and 4, away gives
    // 3 and 4.
    let rtz = RV.round(Round::TiesAway);
    assert_eq!(to_signed::<B64>(d(2.5), 32, RV).0, 2);
    assert_eq!(to_signed::<B64>(d(2.5), 32, rtz).0, 3);
    assert_eq!(to_signed::<B64>(d(3.5), 32, RV).0, 4);
    assert_eq!(to_signed::<B64>(d(3.5), 32, rtz).0, 4);
    assert_eq!(to_signed::<B64>(d(-2.5), 32, RV).0, -2);
    assert_eq!(to_signed::<B64>(d(-2.5), 32, rtz).0, -3);
}

#[test]
fn a_subnormal_result_rounds_on_the_subnormal_grid() {
    // Three quarters of the smallest subnormal rounds to one, and one quarter
    // rounds to zero — nearest-even both times, on a grid one bit wide.
    let quarter = mul::<B64>(MIN_SUB64, d(0.25), RV);
    assert_eq!(quarter, (0, Flags::INEXACT | Flags::UNDERFLOW));
    let three_quarters = mul::<B64>(MIN_SUB64, d(0.75), RV);
    assert_eq!(three_quarters, (1, Flags::INEXACT | Flags::UNDERFLOW));
    // Toward zero, both vanish.
    let rtz = RV.round(Round::TowardZero);
    assert_eq!(
        mul::<B64>(MIN_SUB64, d(0.75), rtz),
        (0, Flags::INEXACT | Flags::UNDERFLOW)
    );
    // Toward positive infinity, even a quarter of one becomes one.
    assert_eq!(
        mul::<B64>(MIN_SUB64, d(0.25), RV.round(Round::TowardPositive)),
        (1, Flags::INEXACT | Flags::UNDERFLOW)
    );
}

// ---------------------------------------------------------------------------
// The guest-specific parameters
// ---------------------------------------------------------------------------

#[test]
fn tininess_detection_separates_exactly_one_case() {
    // (1 - 2^-53) * 2^-1022 is exactly halfway between the largest subnormal
    // and the smallest normal, and ties-to-even rounds it *up* to the smallest
    // normal. The exact value is tiny; the rounded one is not. That is the
    // whole difference between IEEE 754-2019 §7.5's two detection methods, and
    // it is the only case where they disagree.
    let a = d(1.0) - 1; // 1 - 2^-53
    let (v, f) = mul::<B64>(a, MIN_NORM64, Env::RISCV);
    assert_eq!(v, MIN_NORM64);
    assert_eq!(f, Flags::INEXACT, "RISC-V detects tininess after rounding");
    let (v, f) = mul::<B64>(a, MIN_NORM64, Env::ARM);
    assert_eq!(v, MIN_NORM64);
    assert_eq!(
        f,
        Flags::INEXACT | Flags::UNDERFLOW,
        "ARM detects tininess before rounding"
    );
    // A result that is tiny either way is an underflow under both.
    let (_, f) = mul::<B64>(MIN_SUB64, d(0.75), Env::RISCV);
    assert!(f.contains(Flags::UNDERFLOW));
    let (_, f) = mul::<B64>(MIN_SUB64, d(0.75), Env::ARM);
    assert!(f.contains(Flags::UNDERFLOW));
}

#[test]
fn flush_to_zero_is_a_mode_not_a_property_of_the_format() {
    // A subnormal *result* becomes a zero of the same sign, and x86 reports
    // both underflow and inexact even though the exact result was neither.
    let ftz = Env::X86_SSE.ftz(true);
    let (v, f) = mul::<B64>(MIN_NORM64, d(0.5), ftz);
    assert_eq!(v, 0);
    assert_eq!(f, Flags::INEXACT | Flags::UNDERFLOW);
    let (v, f) = add::<B64>(MIN_SUB64, MIN_SUB64, ftz);
    assert_eq!(v, 0, "an exact subnormal sum is flushed too");
    assert_eq!(f, Flags::INEXACT | Flags::UNDERFLOW | Flags::DENORMAL);
    // A subnormal *operand* becomes zero under a different mode bit — and
    // `DAZ` suppresses the denormal-operand exception it would otherwise
    // raise (SDM Volume 1, §10.2.3.4), which is why this reports nothing.
    let (v, f) = add::<B64>(MIN_SUB64, d(1.0), Env::X86_SSE.daz(true));
    assert_eq!(v, d(1.0));
    assert_eq!(f, Flags::NONE);
    // ARM's single FZ bit does both halves and *does* report the flushed
    // input, as FPSR.IDC.
    let (v, f) = add::<B64>(MIN_SUB64, d(1.0), Env::ARM.flush(true));
    assert_eq!(v, d(1.0));
    assert_eq!(f, Flags::DENORMAL);
    // Without the mode bits the same operand is exact, and x86 still says it
    // saw a denormal.
    let (v, f) = add::<B64>(MIN_SUB64, d(1.0), Env::X86_SSE);
    assert_eq!(v, d(1.0));
    assert_eq!(f, Flags::DENORMAL | Flags::INEXACT);
    // RISC-V has neither the mode nor the flag.
    let (v, f) = add::<B64>(MIN_SUB64, d(1.0), Env::RISCV);
    assert_eq!(v, d(1.0));
    assert_eq!(f, Flags::INEXACT);
    // The sign survives the flush.
    let (v, _) = add::<B64>(MIN_SUB64 | B64::SIGN, MIN_SUB64 | B64::SIGN, ftz);
    assert_eq!(v, B64::SIGN);
}

#[test]
fn the_default_nan_is_positive_on_riscv_and_negative_on_x86() {
    // RISC-V Volume I's canonical NaN, ARM's default NaN, and x86's "QNaN
    // floating-point indefinite" (SDM Volume 1, Table 4-1) are the same
    // payload with different signs.
    let (v, _) = mul::<B64>(d(0.0), d(f64::INFINITY), Env::RISCV);
    assert_eq!(v, 0x7ff8_0000_0000_0000);
    let (v, _) = mul::<B64>(d(0.0), d(f64::INFINITY), Env::ARM);
    assert_eq!(v, 0x7ff8_0000_0000_0000);
    let (v, _) = mul::<B64>(d(0.0), d(f64::INFINITY), Env::X86_SSE);
    assert_eq!(v, 0xfff8_0000_0000_0000);
    let (v, _) = mul::<B32>(s(0.0), s(f32::INFINITY), Env::X86_SSE);
    assert_eq!(v, 0xffc0_0000);
}

#[test]
fn nan_propagation_is_a_parameter() {
    // Two quiet NaNs with different payloads, and one signaling NaN.
    let a = B64::QUIET_NAN | 0x1111;
    let b = B64::QUIET_NAN | 0x2222;
    let snan = B64::INF | 0x3333;
    let quieted_snan = snan | B64::QUIET;

    // RISC-V discards payloads entirely.
    assert_eq!(add::<B64>(a, b, Env::RISCV).0, B64::QUIET_NAN);
    assert_eq!(
        add::<B64>(snan, d(1.0), Env::RISCV),
        (B64::QUIET_NAN, Flags::INVALID)
    );

    // x86 SSE takes the first source operand, whatever kind it is, and quiets
    // it (SDM Volume 1, Table 4-7).
    assert_eq!(add::<B64>(a, b, Env::X86_SSE).0, a);
    assert_eq!(add::<B64>(b, a, Env::X86_SSE).0, b);
    assert_eq!(
        add::<B64>(snan, a, Env::X86_SSE),
        (quieted_snan, Flags::INVALID)
    );
    assert_eq!(
        add::<B64>(a, snan, Env::X86_SSE),
        (a, Flags::INVALID),
        "a signaling second operand still raises invalid, but does not win"
    );

    // ARM prefers a signaling operand, in operand order.
    assert_eq!(
        add::<B64>(a, snan, Env::ARM),
        (quieted_snan, Flags::INVALID)
    );
    assert_eq!(add::<B64>(b, a, Env::ARM).0, b);
    // With FPCR.DN set it stops propagating at all.
    assert_eq!(add::<B64>(a, b, Env::ARM_DEFAULT_NAN).0, B64::QUIET_NAN);

    // x87 prefers the quiet operand outright, and decides two of a kind by the
    // larger significand.
    assert_eq!(add::<B64>(snan, a, Env::X87), (a, Flags::INVALID));
    assert_eq!(add::<B64>(a, b, Env::X87).0, b, "b has the larger payload");
    assert_eq!(add::<B64>(b, a, Env::X87).0, b);
    let snan_big = B64::INF | 0x4444;
    assert_eq!(
        add::<B64>(snan, snan_big, Env::X87),
        (snan_big | B64::QUIET, Flags::INVALID)
    );
    // Equal significands: the documented tie-break is the first operand.
    let a2 = a | B64::SIGN;
    assert_eq!(add::<B64>(a2, a, Env::X87).0, a2);
}

#[test]
fn a_quiet_nan_operand_alone_never_raises_invalid() {
    // §7.2: only a *signaling* NaN operand signals invalid.
    let q = B64::QUIET_NAN | 7;
    for env in [Env::RISCV, Env::X86_SSE, Env::ARM, Env::X87] {
        assert_eq!(add::<B64>(q, d(1.0), env).1, Flags::NONE);
        assert_eq!(mul::<B64>(q, d(1.0), env).1, Flags::NONE);
        assert_eq!(div::<B64>(q, d(1.0), env).1, Flags::NONE);
        assert_eq!(sqrt::<B64>(q, env).1, Flags::NONE);
    }
}

#[test]
fn min_max_rules_disagree_between_guests() {
    let q = B64::QUIET_NAN;
    // x86 MINSD/MAXSD: the second source wins whenever the comparison does not
    // strictly select the first — a NaN either side, or equal operands.
    assert_eq!(min::<B64>(q, d(1.0), Env::X86_SSE).0, d(1.0));
    assert_eq!(min::<B64>(d(1.0), q, Env::X86_SSE).0, q);
    assert_eq!(min::<B64>(d(-0.0), d(0.0), Env::X86_SSE).0, d(0.0));
    assert_eq!(min::<B64>(d(0.0), d(-0.0), Env::X86_SSE).0, d(-0.0));
    assert_eq!(min::<B64>(d(1.0), d(2.0), Env::X86_SSE).0, d(1.0));
    assert_eq!(max::<B64>(d(1.0), d(2.0), Env::X86_SSE).0, d(2.0));
    // ARM propagates the NaN through its usual rules.
    let payload = B64::QUIET_NAN | 0x99;
    assert_eq!(min::<B64>(payload, d(1.0), Env::ARM).0, payload);
    // RISC-V returns the number.
    assert_eq!(min::<B64>(payload, d(1.0), Env::RISCV).0, d(1.0));
    // An operand the environment flushed has stopped being that subnormal, so
    // the winner is the zero and not the encoding that was handed in.
    let daz = Env::X86_SSE.daz(true);
    assert_eq!(min::<B64>(MIN_SUB64, d(1.0), daz).0, 0);
    assert_eq!(max::<B64>(MIN_SUB64 | B64::SIGN, d(-1.0), daz).0, B64::SIGN);
    // Without the mode bit the same operand comes back untouched.
    assert_eq!(min::<B64>(MIN_SUB64, d(1.0), Env::X86_SSE).0, MIN_SUB64);
}

#[test]
fn integer_conversion_out_of_range_is_a_parameter() {
    let rtz = |e: Env| e.round(Round::TowardZero);
    let nan = B64::QUIET_NAN;
    assert_eq!(
        to_signed::<B64>(nan, 32, rtz(Env::RISCV)).0,
        i64::from(i32::MAX)
    );
    assert_eq!(to_signed::<B64>(nan, 32, rtz(Env::ARM)).0, 0);
    assert_eq!(
        to_signed::<B64>(nan, 32, rtz(Env::X86_SSE)).0,
        i64::from(i32::MIN),
        "x86 delivers the integer indefinite"
    );
    // A positive value out of range: RISC-V and ARM saturate, x86 does not.
    assert_eq!(
        to_signed::<B64>(d(1e30), 32, rtz(Env::RISCV)).0,
        i64::from(i32::MAX)
    );
    assert_eq!(
        to_signed::<B64>(d(1e30), 32, rtz(Env::X86_SSE)).0,
        i64::from(i32::MIN)
    );
    // Every one of them raises invalid.
    for env in [Env::RISCV, Env::ARM, Env::X86_SSE] {
        assert_eq!(to_signed::<B64>(d(1e30), 32, rtz(env)).1, Flags::INVALID);
    }
}

#[test]
fn the_flag_encodings_are_each_guests_own() {
    let all =
        Flags::INVALID | Flags::DIV_BY_ZERO | Flags::OVERFLOW | Flags::UNDERFLOW | Flags::INEXACT;
    // fcsr.fflags: NX UF OF DZ NV, bits 0..4 (RISC-V Volume I).
    assert_eq!(all.to_fcsr(), 0x1f);
    assert_eq!(Flags::INEXACT.to_fcsr(), 1 << 0);
    assert_eq!(Flags::UNDERFLOW.to_fcsr(), 1 << 1);
    assert_eq!(Flags::OVERFLOW.to_fcsr(), 1 << 2);
    assert_eq!(Flags::DIV_BY_ZERO.to_fcsr(), 1 << 3);
    assert_eq!(Flags::INVALID.to_fcsr(), 1 << 4);
    assert_eq!(Flags::DENORMAL.to_fcsr(), 0, "RISC-V has no denormal flag");
    // MXCSR and the x87 status word: IE DE ZE OE UE PE, bits 0..5.
    assert_eq!(Flags::INVALID.to_mxcsr(), 1 << 0);
    assert_eq!(Flags::DENORMAL.to_mxcsr(), 1 << 1);
    assert_eq!(Flags::DIV_BY_ZERO.to_mxcsr(), 1 << 2);
    assert_eq!(Flags::OVERFLOW.to_mxcsr(), 1 << 3);
    assert_eq!(Flags::UNDERFLOW.to_mxcsr(), 1 << 4);
    assert_eq!(Flags::INEXACT.to_mxcsr(), 1 << 5);
    assert_eq!(all.to_x87_status(), all.to_mxcsr());
    // FPSR: IOC DZC OFC UFC IXC at 0..4, IDC away at 7.
    assert_eq!(Flags::INVALID.to_fpsr(), 1 << 0);
    assert_eq!(Flags::DIV_BY_ZERO.to_fpsr(), 1 << 1);
    assert_eq!(Flags::OVERFLOW.to_fpsr(), 1 << 2);
    assert_eq!(Flags::UNDERFLOW.to_fpsr(), 1 << 3);
    assert_eq!(Flags::INEXACT.to_fpsr(), 1 << 4);
    assert_eq!(Flags::DENORMAL.to_fpsr(), 1 << 7);
}

#[test]
fn the_rounding_mode_encodings_are_each_guests_own() {
    // RISC-V Volume I: RNE RTZ RDN RUP RMM, 0..4; 5 and 6 reserved.
    assert_eq!(Round::from_riscv_rm(0), Some(Round::TiesEven));
    assert_eq!(Round::from_riscv_rm(1), Some(Round::TowardZero));
    assert_eq!(Round::from_riscv_rm(2), Some(Round::TowardNegative));
    assert_eq!(Round::from_riscv_rm(3), Some(Round::TowardPositive));
    assert_eq!(Round::from_riscv_rm(4), Some(Round::TiesAway));
    assert_eq!(Round::from_riscv_rm(5), None);
    assert_eq!(Round::from_riscv_rm(7), None);
    for rm in 0..5 {
        assert_eq!(Round::from_riscv_rm(rm).unwrap().riscv_rm(), rm);
    }
    // x86 RC: nearest, down, up, truncate — a different order, and no
    // ties-away at all.
    assert_eq!(Round::from_x86_rc(0), Round::TiesEven);
    assert_eq!(Round::from_x86_rc(1), Round::TowardNegative);
    assert_eq!(Round::from_x86_rc(2), Round::TowardPositive);
    assert_eq!(Round::from_x86_rc(3), Round::TowardZero);
    for rc in 0..4 {
        assert_eq!(Round::from_x86_rc(rc).x86_rc(), rc);
    }
}

#[test]
fn nan_payloads_rescale_across_a_format_conversion() {
    // §6.2.3: a payload should survive where it can. x86 shifts it to keep the
    // leading bits, which is what makes a widen/narrow round trip stable.
    let wide = convert::<B32, B64>(0x7fc0_0000 | 0x12_3456, Env::X86_SSE).0;
    assert_eq!(wide >> 52, 0x7ff);
    let back = convert::<B64, B32>(wide, Env::X86_SSE).0;
    assert_eq!(back, 0x7fc0_0000 | 0x12_3456);
    // RISC-V does not propagate at all, so the round trip is the canonical NaN.
    let wide = convert::<B32, B64>(0x7fc0_0000 | 0x12_3456, Env::RISCV).0;
    assert_eq!(wide, B64::QUIET_NAN);
}

// ---------------------------------------------------------------------------
// Correct rounding, verified against the definition
// ---------------------------------------------------------------------------

/// Unpack a binary64 encoding as `(sign, exp, frac)` with the value equal to
/// `(-1)^sign * frac * 2^exp`. Finite values only.
fn parts64(bits: u64) -> (bool, i32, u64) {
    let sign = bits & B64::SIGN != 0;
    let field = (bits >> 52) & 0x7ff;
    let frac = bits & B64::SIG_MASK;
    if field == 0 {
        (sign, -1074, frac)
    } else {
        (sign, field as i32 - 1023 - 52, frac | 1 << 52)
    }
}

/// Assert that `frac * 2^exp` is the round-to-nearest-even rounding of the
/// exact value `value * 2^value_exp`.
///
/// This is IEEE 754-2019 §5.1 and §4.3.1 applied directly in exact integer
/// arithmetic: the delivered result must be no further from the exact value
/// than either adjacent representable number, and a tie must land on the even
/// significand. Nothing else is consulted — no host FPU and no second
/// implementation — so a disagreement here is unambiguous.
///
/// The caller must not pass a result whose significand is a power of two: the
/// spacing below such a value is half the spacing above it, and this check
/// assumes one ulp on both sides.
fn assert_nearest_even(value: u128, value_exp: i32, frac: u64, exp: i32) {
    assert!(
        exp >= value_exp,
        "the result must be no finer than the exact value"
    );
    let k = (exp - value_exp) as u32;
    let r = u128::from(frac) << k;
    let lo = u128::from(frac - 1) << k;
    let hi = u128::from(frac + 1) << k;
    let d = value.abs_diff(r);
    let dlo = value.abs_diff(lo);
    let dhi = value.abs_diff(hi);
    assert!(d <= dlo && d <= dhi, "not the nearest: {value} vs {r}");
    if d == dlo || d == dhi {
        assert_eq!(frac & 1, 0, "a tie must round to an even significand");
    }
}

#[test]
fn binary64_multiplication_is_correctly_rounded_by_definition() {
    let mut x: u64 = 0x2545_f491_4f6c_dd1d;
    let mut checked = 0;
    for _ in 0..20_000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        // Two normal operands in the middle of the exponent range, so the
        // product can neither overflow nor go subnormal.
        let a = (x & 0x000f_ffff_ffff_ffff) | (0x400u64 << 52);
        let b = (x.rotate_left(31) & 0x000f_ffff_ffff_ffff) | (0x3f0u64 << 52);
        let (got, flags) = mul::<B64>(a, b, RV);
        assert!(!flags.contains(Flags::OVERFLOW) && !flags.contains(Flags::UNDERFLOW));
        let (_, ea, fa) = parts64(a);
        let (_, eb, fb) = parts64(b);
        let (_, er, fr) = parts64(got);
        if fr == 1 << 52 {
            continue; // a power-of-two significand has neighbours of two sizes
        }
        assert_nearest_even(u128::from(fa) * u128::from(fb), ea + eb, fr, er);
        checked += 1;
    }
    assert!(checked > 19_000, "the sample degenerated: {checked}");
}

#[test]
fn binary64_addition_is_correctly_rounded_by_definition() {
    let mut x: u64 = 0x9e37_79b9_7f4a_7c15;
    for _ in 0..20_000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        // Exponents within 40 of each other, so the exact sum fits a u128.
        let ea = 0x400u64 + (x & 0x1f);
        let eb = 0x400u64 + ((x >> 8) & 0x1f);
        let a = (x & 0x000f_ffff_ffff_ffff) | (ea << 52);
        let b = (x.rotate_left(23) & 0x000f_ffff_ffff_ffff) | (eb << 52);
        let (got, _) = add::<B64>(a, b, RV);
        let (_, ea, fa) = parts64(a);
        let (_, eb, fb) = parts64(b);
        let (_, er, fr) = parts64(got);
        if fr == 1 << 52 {
            continue;
        }
        let base = ea.min(eb);
        let exact = (u128::from(fa) << (ea - base)) + (u128::from(fb) << (eb - base));
        assert_nearest_even(exact, base, fr, er);
    }
}

/// The oracle that is not an oracle everywhere: the host FPU.
///
/// Valid only for round-to-nearest-even on finite operands with a finite
/// result, which is exactly what this test restricts itself to. It says
/// nothing about NaN payloads, nothing about the flags, and nothing about the
/// other four rounding attributes — see this module's header.
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
        let (got, _) = add::<B64>(a.to_bits(), b.to_bits(), RV);
        if want.is_finite() {
            assert_eq!(got, want.to_bits(), "{a:e} + {b:e}");
        }
        let want = a - b;
        let (got, _) = sub::<B64>(a.to_bits(), b.to_bits(), RV);
        if want.is_finite() {
            assert_eq!(got, want.to_bits(), "{a:e} - {b:e}");
        }
        let want = a * b;
        let (got, _) = mul::<B64>(a.to_bits(), b.to_bits(), RV);
        if want.is_finite() {
            assert_eq!(got, want.to_bits(), "{a:e} * {b:e}");
        }
        if b != 0.0 {
            let want = a / b;
            let (got, _) = div::<B64>(a.to_bits(), b.to_bits(), RV);
            if want.is_finite() {
                assert_eq!(got, want.to_bits(), "{a:e} / {b:e}");
            }
        }
        if a > 0.0 {
            let want = a.sqrt();
            let (got, _) = sqrt::<B64>(a.to_bits(), RV);
            assert_eq!(got, want.to_bits(), "sqrt({a:e})");
        }
        // `mul_add` is required to be fused, and a fused multiply-add is
        // correctly rounded by IEEE 754-2019 §5.4.1, so it is an oracle
        // wherever hardware or the host's libm provides it.
        let c = f64::from_bits(x.rotate_right(17));
        if c.is_finite() {
            let want = a.mul_add(b, c);
            let (got, _) = fma::<B64>(a.to_bits(), b.to_bits(), c.to_bits(), RV);
            if want.is_finite() {
                assert_eq!(got, want.to_bits(), "fma({a:e}, {b:e}, {c:e})");
            }
        }
        // binary32, on the low half of the same sample.
        let a32 = f32::from_bits(x as u32);
        let b32 = f32::from_bits((x >> 32) as u32);
        if a32.is_finite() && b32.is_finite() {
            let want = a32 * b32;
            let (got, _) = mul::<B32>(u64::from(a32.to_bits()), u64::from(b32.to_bits()), RV);
            if want.is_finite() {
                assert_eq!(got, u64::from(want.to_bits()), "{a32:e} * {b32:e}");
            }
            let want = a32 + b32;
            let (got, _) = add::<B32>(u64::from(a32.to_bits()), u64::from(b32.to_bits()), RV);
            if want.is_finite() {
                assert_eq!(got, u64::from(want.to_bits()), "{a32:e} + {b32:e}");
            }
        }
    }
}

#[test]
fn conversion_to_and_from_binary32_matches_the_host() {
    let mut x: u64 = 0xdead_beef_cafe_0001;
    for _ in 0..10_000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        let a = f64::from_bits(x);
        if a.is_nan() {
            continue;
        }
        let want = a as f32;
        let (got, _) = convert::<B64, B32>(a.to_bits(), RV);
        assert_eq!(got, u64::from(want.to_bits()), "{a:e} as f32");
        let back = f64::from(want);
        let (got, _) = convert::<B32, B64>(u64::from(want.to_bits()), RV);
        assert_eq!(got, back.to_bits());
    }
}

// ---------------------------------------------------------------------------
// x87: the 80-bit format
// ---------------------------------------------------------------------------

/// The x87 environment at nearest-even.
const X87: Env = Env::X87;

/// An 80-bit value holding exactly what a binary64 encoding holds.
fn e(bits: u64) -> F80 {
    x87::from_binary::<B64>(bits, X87).0
}

#[test]
fn the_eighty_bit_encoding_round_trips_through_memory() {
    let one = e(d(1.0));
    assert_eq!(one, F80::new(0x3fff, 0x8000_0000_0000_0000));
    let bytes = one.to_bytes();
    assert_eq!(bytes, [0, 0, 0, 0, 0, 0, 0, 0x80, 0xff, 0x3f]);
    assert_eq!(F80::from_bytes(bytes), one);
    // The integer bit is explicit, so 1.0 stores a leading 1 that binary64
    // only implies.
    assert!(one.sig & (1 << 63) != 0);
    assert!(F80::INDEFINITE.sign());
    assert_eq!(x87::to_binary::<B64>(one, X87), (d(1.0), Flags::NONE));
}

#[test]
fn unsupported_encodings_are_invalid_operands() {
    // SDM Volume 1, §8.2.2: an unnormal (non-zero exponent, integer bit
    // clear), a pseudo-infinity and a pseudo-NaN are not values.
    let unnormal = F80::new(1, 1);
    let pseudo_inf = F80::new(0x7fff, 0);
    let pseudo_nan = F80::new(0x7fff, 1);
    for bad in [unnormal, pseudo_inf, pseudo_nan] {
        assert_eq!(x87::classify(bad), X87Class::Unsupported);
        let (v, f) = x87::add(bad, e(d(1.0)), Precision::Extended, X87);
        assert_eq!(v, F80::INDEFINITE);
        assert!(f.contains(Flags::INVALID));
        assert_eq!(x87::compare(bad, e(d(1.0))), None);
    }
    // A pseudo-denormal is *not* rejected: it is a redundant encoding of an
    // ordinary value, and adding zero to it normalises the encoding.
    let pseudo_denormal = F80::new(0, 1 << 63);
    assert_eq!(
        x87::classify(pseudo_denormal),
        X87Class::Ieee(Category::PositiveNormal)
    );
    let (v, f) = x87::add(pseudo_denormal, F80::ZERO, Precision::Extended, X87);
    assert_eq!(v, F80::new(1, 1 << 63), "2^-16382, normally encoded");
    assert_eq!(f, Flags::NONE);
}

#[test]
fn the_eighty_bit_format_classifies_like_ieee() {
    assert_eq!(
        x87::classify(F80::ZERO),
        X87Class::Ieee(Category::PositiveZero)
    );
    assert_eq!(
        x87::classify(F80::new(0x8000, 0)),
        X87Class::Ieee(Category::NegativeZero)
    );
    assert_eq!(
        x87::classify(F80::INFINITY),
        X87Class::Ieee(Category::PositiveInfinity)
    );
    assert_eq!(
        x87::classify(F80::INDEFINITE),
        X87Class::Ieee(Category::QuietNan)
    );
    assert_eq!(
        x87::classify(F80::new(0x7fff, (1 << 63) | 1)),
        X87Class::Ieee(Category::SignalingNan)
    );
    assert_eq!(
        x87::classify(F80::new(0, 1)),
        X87Class::Ieee(Category::PositiveSubnormal),
        "the smallest subnormal is 2^-16445"
    );
}

#[test]
fn precision_control_shortens_the_significand_and_not_the_exponent() {
    // 1 + 2^-63 is exactly representable in 64 significand bits and in no
    // shorter one.
    let one = e(d(1.0));
    let tiny = F80::new(0x3fff - 63, 1 << 63); // 2^-63
    let (v, f) = x87::add(one, tiny, Precision::Extended, X87);
    assert_eq!(v, F80::new(0x3fff, (1u64 << 63) | 1));
    assert_eq!(f, Flags::NONE);
    let (v, f) = x87::add(one, tiny, Precision::Double, X87);
    assert_eq!(v, one, "53 bits cannot hold it");
    assert_eq!(f, Flags::INEXACT);
    let (v, f) = x87::add(one, tiny, Precision::Single, X87);
    assert_eq!(v, one);
    assert_eq!(f, Flags::INEXACT);
    // A shortened precision leaves the low significand bits zero, exactly as
    // the hardware's register does.
    let third = x87::div(one, e(d(3.0)), Precision::Single, X87).0;
    assert_eq!(third.sig & ((1 << 40) - 1), 0);
    assert_eq!(third.sig >> 40, 0x00aa_aaab); // 1/3 to 24 bits, rounded up
    // ...and the exponent range is still the extended one, which is what makes
    // PC=53 different from binary64: 2^-20000 is a subnormal double and an
    // ordinary normal here.
    let small = x87::div(
        one,
        F80::new(0x3fff + 2_000, 1 << 63),
        Precision::Double,
        X87,
    );
    assert_eq!(small.1, Flags::NONE);
    assert_eq!(small.0.exp_field(), 0x3fff - 2_000);
}

#[test]
fn precision_control_at_53_bits_reproduces_binary64() {
    // The transitive oracle: binary64 is checked against the host, and an x87
    // result at PC=53 with operands well inside the double exponent range must
    // equal it bit for bit. This does not exercise the 64-bit significand —
    // `eighty_bit_multiplication_is_correctly_rounded_by_definition` does.
    let mut x: u64 = 0x0123_4567_89ab_cdef;
    for _ in 0..5_000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        let ea = 0x300u64 + (x & 0xff);
        let eb = 0x300u64 + ((x >> 40) & 0xff);
        let a = (x & 0x000f_ffff_ffff_ffff) | (ea << 52);
        let b = (x.rotate_left(19) & 0x000f_ffff_ffff_ffff) | (eb << 52);
        for (name, want, got) in [
            (
                "add",
                add::<B64>(a, b, RV).0,
                x87::add(e(a), e(b), Precision::Double, X87).0,
            ),
            (
                "sub",
                sub::<B64>(a, b, RV).0,
                x87::sub(e(a), e(b), Precision::Double, X87).0,
            ),
            (
                "mul",
                mul::<B64>(a, b, RV).0,
                x87::mul(e(a), e(b), Precision::Double, X87).0,
            ),
            (
                "div",
                div::<B64>(a, b, RV).0,
                x87::div(e(a), e(b), Precision::Double, X87).0,
            ),
            (
                "sqrt",
                sqrt::<B64>(a, RV).0,
                x87::sqrt(e(a), Precision::Double, X87).0,
            ),
        ] {
            let narrowed = x87::to_binary::<B64>(got, X87).0;
            assert_eq!(narrowed, want, "{name} of {a:#x}, {b:#x}");
        }
    }
}

#[test]
fn precision_control_at_24_bits_reproduces_binary32() {
    let mut x: u64 = 0xfeed_face_1234_5678;
    for _ in 0..5_000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        let a32 = ((x as u32) & 0x007f_ffff) | (0x50 << 23);
        let b32 = (((x >> 32) as u32) & 0x007f_ffff) | (0x40 << 23);
        let (a, b) = (u64::from(a32), u64::from(b32));
        let want = mul::<B32>(a, b, RV).0;
        let wide = (
            x87::from_binary::<B32>(a, X87).0,
            x87::from_binary::<B32>(b, X87).0,
        );
        let got = x87::mul(wide.0, wide.1, Precision::Single, X87).0;
        assert_eq!(x87::to_binary::<B32>(got, X87).0, want);
        let want = div::<B32>(a, b, RV).0;
        let got = x87::div(wide.0, wide.1, Precision::Single, X87).0;
        assert_eq!(x87::to_binary::<B32>(got, X87).0, want);
    }
}

#[test]
fn eighty_bit_multiplication_is_correctly_rounded_by_definition() {
    // Operands with at most 40 significant bits, so the exact product is 80
    // bits — wide enough that rounding to 64 really happens, narrow enough that
    // the check itself stays in a u128 with no rounding of its own.
    let mut x: u64 = 0x5bf0_3635_1f3a_9c11;
    for _ in 0..20_000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        let fa = (x >> 24) | (1 << 39);
        let fb = (x.rotate_left(37) >> 24) | (1 << 39);
        // value = frac * 2^-39, i.e. an exponent field of 0x3fff for a leading
        // bit at 2^0 once the significand is normalised to bit 63.
        let a = F80::new(0x3fff - 24, fa << 24);
        let b = F80::new(0x3fff - 24, fb << 24);
        let (got, flags) = x87::mul(a, b, Precision::Extended, X87);
        assert!(!flags.contains(Flags::OVERFLOW) && !flags.contains(Flags::UNDERFLOW));
        assert_eq!(
            got.sig & (1 << 63),
            1 << 63,
            "a normal result is normalised"
        );
        if got.sig == 1 << 63 {
            continue; // a power-of-two significand, as in the binary64 check
        }
        // Both operands sit at exponent (field - 16383 - 63); the product's
        // exact exponent is the sum, and the result's is its own.
        let ea = i32::from(a.exp_field()) - 16383 - 63;
        let eb = i32::from(b.exp_field()) - 16383 - 63;
        let er = i32::from(got.exp_field()) - 16383 - 63;
        assert_nearest_even(
            u128::from(fa << 24) * u128::from(fb << 24),
            ea + eb,
            got.sig,
            er,
        );
    }
}

#[test]
fn eighty_bit_square_root_is_correctly_rounded_toward_zero() {
    // Rounding toward zero makes correctness a pure integer statement:
    // `r^2 <= value < (r+1)^2` in the units of the result's last place. No
    // oracle at all, only the definition.
    let rtz = X87.round(Round::TowardZero);
    let mut x: u64 = 0x0f0f_1e1e_3c3c_7878;
    for _ in 0..20_000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        // An even exponent, so sqrt halves it exactly and the significand
        // arithmetic below is the whole story.
        let a = F80::new(0x3fff, x | (1 << 63));
        let (got, _) = x87::sqrt(a, Precision::Extended, rtz);
        // a = fa * 2^-63 (exponent field 0x3fff means a leading bit at 2^0),
        // r = fr * 2^-63 as well, so r^2 = fr^2 * 2^-126 and a = fa * 2^-63.
        let fa = u128::from(a.sig) << 63;
        let fr = u128::from(got.sig);
        assert!(fr * fr <= fa, "root too large");
        assert!((fr + 1) * (fr + 1) > fa, "root too small");
    }
}

#[test]
fn the_eighty_bit_exponent_range_is_its_own() {
    // Overflow happens 2^16384 up rather than 2^1024 up, and the largest
    // finite value has every significand bit set.
    let (v, f) = x87::add(F80::MAX_FINITE, F80::MAX_FINITE, Precision::Extended, X87);
    assert_eq!(v, F80::INFINITY);
    assert_eq!(f, Flags::OVERFLOW | Flags::INEXACT);
    let (v, f) = x87::add(
        F80::MAX_FINITE,
        F80::MAX_FINITE,
        Precision::Extended,
        X87.round(Round::TowardZero),
    );
    assert_eq!(v, F80::MAX_FINITE);
    assert_eq!(f, Flags::OVERFLOW | Flags::INEXACT);
    // A double that overflows binary64 is an ordinary number here.
    let big = x87::mul(e(d(1e300)), e(d(1e300)), Precision::Extended, X87);
    assert!(
        !big.1.contains(Flags::OVERFLOW),
        "10^600 is an ordinary number in this format"
    );
    // ...and narrowing it back to binary64 is where the overflow appears.
    let (v, f) = x87::to_binary::<B64>(big.0, X87);
    assert_eq!(v, d(f64::INFINITY));
    assert_eq!(f, Flags::OVERFLOW | Flags::INEXACT);
    // The smallest subnormal is 2^-16445.
    let (v, f) = x87::mul(F80::new(0, 1), e(d(0.5)), Precision::Extended, X87);
    assert_eq!(v, F80::ZERO);
    assert_eq!(
        f,
        Flags::INEXACT | Flags::UNDERFLOW | Flags::DENORMAL,
        "x87 reports the subnormal operand as well (SDM Volume 1, §4.9.1.2)"
    );
}

#[test]
fn eighty_bit_integer_conversion_is_exact_for_every_i64() {
    // 64 significand bits hold any `i64` exactly, which is why `FILD m64int`
    // never rounds — and why x87 is the only format here that can say so.
    for v in [
        0i64,
        1,
        -1,
        i64::MAX,
        i64::MIN,
        (1 << 53) + 1,
        -((1 << 62) + 12345),
    ] {
        let (f80, flags) = x87::from_signed(v, 64, X87);
        assert_eq!(flags, Flags::NONE, "{v}");
        let (back, flags) = x87::to_signed(f80, 64, X87.round(Round::TowardZero));
        assert_eq!(back, v, "{v}");
        assert_eq!(flags, Flags::NONE, "{v}");
    }
    // binary64 cannot: it loses the low bit of 2^53 + 1.
    assert_eq!(from_signed::<B64>((1 << 53) + 1, 64, RV).1, Flags::INEXACT);
    // Out of range delivers the integer indefinite under x87's rules.
    let huge = x87::from_signed(i64::MAX, 64, X87).0;
    let (v, f) = x87::to_signed(huge, 32, X87.round(Round::TowardZero));
    assert_eq!(v, i64::from(i32::MIN));
    assert_eq!(f, Flags::INVALID);
}

#[test]
fn widening_into_the_eighty_bit_format_is_always_exact() {
    let mut x: u64 = 0xabcd_ef01_2345_6789;
    for _ in 0..10_000 {
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        let a = f64::from_bits(x);
        if !a.is_finite() {
            continue;
        }
        let (wide, flags) = x87::from_binary::<B64>(x, X87);
        assert!(!flags.contains(Flags::INEXACT), "{a:e}");
        let (back, flags) = x87::to_binary::<B64>(wide, X87);
        assert_eq!(back, x, "{a:e}");
        assert!(!flags.contains(Flags::INEXACT));
    }
}

#[test]
fn the_eighty_bit_nan_rules_are_x87s() {
    let a = F80::new(0x7fff, (1 << 63) | (1 << 62) | 0x1111);
    let b = F80::new(0x7fff, (1 << 63) | (1 << 62) | 0x2222);
    let snan = F80::new(0x7fff, (1 << 63) | 0x3333);
    // Two quiet NaNs: the larger significand wins.
    assert_eq!(x87::add(a, b, Precision::Extended, X87).0, b);
    assert_eq!(x87::add(b, a, Precision::Extended, X87).0, b);
    // A quiet NaN beside a signaling one wins outright, and invalid is still
    // raised.
    let (v, f) = x87::add(snan, a, Precision::Extended, X87);
    assert_eq!(v, a);
    assert_eq!(f, Flags::INVALID);
    // An invalid operation with no NaN operand delivers the indefinite.
    let (v, f) = x87::div(F80::ZERO, F80::ZERO, Precision::Extended, X87);
    assert_eq!(v, F80::INDEFINITE);
    assert_eq!(f, Flags::INVALID);
    // Narrowing a propagated NaN keeps the leading payload bits.
    let (narrow, _) = x87::to_binary::<B64>(a, X87);
    assert_eq!(narrow >> 52, 0x7ff);
    assert!(narrow & B64::QUIET != 0);
}

#[test]
fn the_precision_control_encoding_is_intels() {
    assert_eq!(Precision::from_pc(0), Some(Precision::Single));
    assert_eq!(Precision::from_pc(1), None, "01 is reserved");
    assert_eq!(Precision::from_pc(2), Some(Precision::Double));
    assert_eq!(Precision::from_pc(3), Some(Precision::Extended));
    assert_eq!(Precision::Single.bits(), 24);
    assert_eq!(Precision::Double.bits(), 53);
    assert_eq!(Precision::Extended.bits(), 64);
    for pc in [0, 2, 3] {
        assert_eq!(Precision::from_pc(pc).unwrap().pc(), pc);
    }
    // Precision control never touches the exponent range.
    for p in [Precision::Single, Precision::Double, Precision::Extended] {
        assert_eq!(p.spec().emax, 16383);
        assert_eq!(p.spec().min_ulp, -16445);
        assert_eq!(p.spec().precision, p.bits());
    }
}

// ---------------------------------------------------------------------------
// The property the whole subsystem exists for
// ---------------------------------------------------------------------------

#[test]
fn no_host_float_in_the_implementation() {
    // ROADMAP.md §9.1: guest floating point must not touch host floating
    // point, or a state hash stops being reproducible across hosts. This test
    // reads the implementation back and says so — comments and this test file
    // excluded, since both may name the host types freely.
    let sources = [
        ("mod.rs", include_str!("mod.rs")),
        ("kernel.rs", include_str!("kernel.rs")),
        ("binary.rs", include_str!("binary.rs")),
        ("x87.rs", include_str!("x87.rs")),
    ];
    for (name, src) in sources {
        for (n, line) in src.lines().enumerate() {
            let code = match line.find("//") {
                Some(i) => &line[..i],
                None => line,
            };
            for needle in ["f32", "f64", "f16", "f128", "float", "sqrtf", "libm"] {
                assert!(
                    !code.contains(needle),
                    "{name}:{}: host floating point in the implementation: {code}",
                    n + 1
                );
            }
        }
    }
}

#[test]
fn the_profiles_differ_where_they_are_documented_to() {
    // A cheap guard against a profile being edited into a copy of another.
    assert_ne!(Env::RISCV, Env::X86_SSE);
    assert_ne!(Env::X86_SSE, Env::X87);
    assert_ne!(Env::ARM, Env::ARM_DEFAULT_NAN);
    assert_eq!(Env::default(), Env::RISCV);
    assert_eq!(
        Env::RISCV.round(Round::TowardZero).round(Round::TiesEven),
        Env::RISCV
    );
    assert!(Env::ARM.flush(true).subnormal_inputs.flushes());
    assert!(Env::ARM.flush(true).subnormal_inputs.reports());
    assert!(Env::ARM.flush(true).flush_outputs);
    assert!(Env::X86_SSE.daz(true).subnormal_inputs.flushes());
    assert!(!Env::X86_SSE.daz(true).subnormal_inputs.reports());
    assert!(Env::X86_SSE.subnormal_inputs.reports());
    // Spec arithmetic: binary64 is (53, 1023) and its smallest subnormal is
    // 2^-1074 (IEEE 754-2019 §3.6).
    assert_eq!(B64::SPEC.precision, 53);
    assert_eq!(B64::SPEC.emax, 1023);
    assert_eq!(B64::SPEC.emin(), -1022);
    assert_eq!(B64::SPEC.min_ulp, -1074);
    assert_eq!(B32::SPEC.min_ulp, -149);
    assert_eq!(F80::SPEC.precision, 64);
    assert_eq!(F80::SPEC.min_ulp, -16445);
}

// ---------------------------------------------------------------------------
// The four operations x87 needs that no interchange format has
// ---------------------------------------------------------------------------

/// An 80-bit value from a significand in `[1, 2)` and an unbiased exponent.
fn e80(exp: i32, sig: u64) -> F80 {
    F80::new((exp + 16383) as u16, sig)
}

/// `1.0`, `4.0`, and the rest of the small integers these tests use.
const F1: F80 = F80::new(0x3fff, 0x8000_0000_0000_0000);
const F4: F80 = F80::new(0x4001, 0x8000_0000_0000_0000);
const F8: F80 = F80::new(0x4002, 0x8000_0000_0000_0000);

#[test]
fn rounding_to_an_integral_value_follows_the_direction_and_reports_movement() {
    // IEEE 754-2019 §5.9's `roundToIntegralExact`, which is `FRNDINT`. The
    // inexact flag is the "Exact" in the operation's name: it fires exactly
    // when the value moved.
    let x87 = Env::X87;
    let two_and_a_half = e80(1, 0xa000_0000_0000_0000);
    let two = e80(1, 0x8000_0000_0000_0000);
    let three = e80(1, 0xc000_0000_0000_0000);

    assert_eq!(
        x87::round_to_integral(two_and_a_half, x87),
        (two, Flags::INEXACT),
        "2.5 ties to even"
    );
    assert_eq!(
        x87::round_to_integral(two_and_a_half, x87.round(Round::TowardPositive)),
        (three, Flags::INEXACT)
    );
    assert_eq!(
        x87::round_to_integral(two_and_a_half, x87.round(Round::TowardZero)),
        (two, Flags::INEXACT)
    );
    // An integer does not move, and nothing is reported — including at a
    // rounding direction that would have moved a fraction.
    assert_eq!(
        x87::round_to_integral(F4, x87.round(Round::TowardPositive)),
        (F4, Flags::NONE)
    );
    // Infinity and NaN pass through; a signaling NaN is quieted and reported.
    assert_eq!(
        x87::round_to_integral(F80::INFINITY, x87),
        (F80::INFINITY, Flags::NONE)
    );
    let snan = F80::new(0x7fff, (1 << 63) | 1);
    let (out, flags) = x87::round_to_integral(snan, x87);
    assert_eq!(flags, Flags::INVALID);
    assert_eq!(x87::classify(out), X87Class::Ieee(Category::QuietNan));
}

#[test]
fn scaling_moves_the_exponent_and_saturates_at_both_ends() {
    // `FSCALE`. Adding to the exponent is exact where multiplying by a
    // computed `2^n` would overflow first, which is the whole reason this is
    // its own operation.
    let x87 = Env::X87;
    assert_eq!(x87::scale(F1, 3, x87), (F8, Flags::NONE));
    assert_eq!(x87::scale(F8, -3, x87), (F1, Flags::NONE));
    // Far past the top of the range: overflow, inexact, and an infinity.
    let (out, flags) = x87::scale(F1, 100_000, x87);
    assert_eq!(out, F80::INFINITY);
    assert!(flags.contains(Flags::OVERFLOW) && flags.contains(Flags::INEXACT));
    // And far past the bottom: underflow to a signed zero.
    let (out, flags) = x87::scale(F1, -100_000, x87);
    assert_eq!(out, F80::ZERO);
    assert!(flags.contains(Flags::UNDERFLOW));
    // A zero and an infinity are unchanged whatever the scale.
    assert_eq!(x87::scale(F80::ZERO, 50, x87), (F80::ZERO, Flags::NONE));
    assert_eq!(
        x87::scale(F80::INFINITY, -50, x87),
        (F80::INFINITY, Flags::NONE)
    );
}

#[test]
fn extract_splits_a_value_into_a_pair_that_multiplies_back() {
    // `FXTRACT`. The significand comes back in `[1, 2)` with the original
    // sign, so `exponent` and `significand` reproduce the input exactly.
    let x87 = Env::X87;
    let twelve = e80(3, 0xc000_0000_0000_0000);
    let (exp, sig, flags) = x87::extract(twelve, x87);
    assert_eq!(flags, Flags::NONE);
    assert_eq!(exp, e80(1, 0xc000_0000_0000_0000), "3.0");
    assert_eq!(sig, e80(0, 0xc000_0000_0000_0000), "1.5");
    assert_eq!(
        x87::mul(exp_of_two(exp), sig, Precision::Extended, x87).0,
        twelve
    );

    // A subnormal normalises, which is why its exponent comes out below the
    // format's own minimum rather than clamped to it.
    let subnormal = F80::new(0, 1);
    let (exp, sig, _) = x87::extract(subnormal, x87);
    assert_eq!(sig, e80(0, 0x8000_0000_0000_0000), "1.0");
    assert_eq!(x87::to_signed(exp, 32, x87).0, -16445);

    // A zero has no exponent at all: the manual's answer is minus infinity and
    // the zero-divide exception, which is the one place x87 raises `#Z` for
    // something that is not a division.
    let (exp, sig, flags) = x87::extract(F80::ZERO, x87);
    assert_eq!(flags, Flags::DIV_BY_ZERO);
    assert_eq!(sig, F80::ZERO);
    assert_eq!(
        x87::classify(exp),
        X87Class::Ieee(Category::NegativeInfinity)
    );
}

/// `2^n` for an exponent `FXTRACT` produced, so a test can multiply back.
fn exp_of_two(exp: F80) -> F80 {
    let (n, _) = x87::to_signed(exp, 32, Env::X87);
    x87::scale(F80::new(0x3fff, 1 << 63), n, Env::X87).0
}

#[test]
fn the_two_remainders_differ_in_how_they_round_the_quotient() {
    // `FPREM` truncates the implied quotient and `FPREM1` rounds it to
    // nearest-even, so `7 mod 4` is 3 one way and -1 the other. Both are
    // exact: the result is a difference of two representable values that is
    // itself representable.
    let x87 = Env::X87;
    let seven = e80(2, 0xe000_0000_0000_0000);
    let three = e80(1, 0xc000_0000_0000_0000);
    let minus_one = F80::new(0xbfff, 0x8000_0000_0000_0000);

    let r = x87::remainder(seven, F4, false, x87);
    assert_eq!(r.value, three);
    assert_eq!(r.flags, Flags::NONE);
    assert!(!r.incomplete);
    assert_eq!(r.quotient, 1, "trunc(7/4) = 1");

    let r = x87::remainder(seven, F4, true, x87);
    assert_eq!(r.value, minus_one, "nearest(7/4) is 2, so 7 - 8");
    assert_eq!(r.quotient, 2);

    // The dividend is its own remainder when it is smaller than the divisor.
    let r = x87::remainder(F1, F4, false, x87);
    assert_eq!(r.value, F1);
    assert_eq!(r.quotient, 0);
    // …but not under the IEEE rule when it is more than half of it: 3 mod 4
    // is -1, because the nearest quotient is one rather than zero.
    let r = x87::remainder(three, F4, true, x87);
    assert_eq!(r.value, minus_one);

    // More than 63 binades apart: the reduction is partial and says so, which
    // is what makes `FPREM` a loop rather than an instruction.
    let huge = e80(4000, 0x8000_0000_0000_0000);
    let r = x87::remainder(huge, F80::new(0x3fff, 0x8000_0000_0000_0001), false, x87);
    assert!(r.incomplete, "63 binades at a time");

    // The three cases with no remainder at all.
    let r = x87::remainder(F80::INFINITY, F4, false, x87);
    assert_eq!(r.flags, Flags::INVALID);
    assert_eq!(r.value, F80::INDEFINITE);
    let r = x87::remainder(F4, F80::ZERO, false, x87);
    assert_eq!(r.flags, Flags::INVALID);
    // An infinite divisor leaves the dividend alone.
    let r = x87::remainder(F4, F80::INFINITY, false, x87);
    assert_eq!(r.value, F4);
    assert_eq!(r.flags, Flags::NONE);
}

#[test]
fn a_complete_remainder_is_always_exact() {
    // Sweep a range of exponent differences and check that nothing ever
    // reports inexact and that `a - q*b` really is what came back. The
    // property is what makes `FPREM` usable for argument reduction at all.
    let x87 = Env::X87;
    for ea in -3..40i32 {
        for sa in [0x8000_0000_0000_0000u64, 0xc000_0000_0000_0001, u64::MAX] {
            let a = e80(ea, sa);
            let b = e80(0, 0xb000_0000_0000_0003);
            let r = x87::remainder(a, b, false, x87);
            assert!(!r.flags.contains(Flags::INEXACT), "ea={ea} sa={sa:#x}");
            if r.incomplete {
                continue;
            }
            // |remainder| < |divisor|, and it has the dividend's sign.
            assert_ne!(
                x87::compare(r.value, F80::ZERO),
                Some(core::cmp::Ordering::Less),
                "the remainder has the dividend's sign"
            );
            assert_eq!(
                x87::compare(r.value, b),
                Some(core::cmp::Ordering::Less),
                "ea={ea}"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// binary16, and round-to-integral
// ---------------------------------------------------------------------------

/// Overflow always reports inexact too (§7.4).
const INEX_OVER: Flags = Flags(Flags::OVERFLOW.0 | Flags::INEXACT.0);

/// Host `f32` bits narrowed to binary16 by the implementation under test, used
/// only to write readable expectations.
fn h(v: f32) -> u64 {
    convert::<B32, B16>(s(v), RV).0
}

#[test]
fn binary16_is_the_same_kernel_at_a_third_of_the_width() {
    // The format's landmarks (IEEE 754-2019 §3.6): bias 15, ten stored
    // significand bits, largest finite 65504, smallest subnormal 2^-24.
    assert_eq!(h(1.0), 0x3c00);
    assert_eq!(h(-2.0), 0xc000);
    assert_eq!(h(65504.0), 0x7bff);
    assert_eq!(B16::MAX_FINITE, 0x7bff);
    assert_eq!(B16::QUIET_NAN, 0x7e00);
    // 65520 is the first value that rounds up to infinity at nearest-even.
    assert_eq!(convert::<B32, B16>(s(65520.0), RV), (0x7c00, INEX_OVER));
    // 2^-24 is exactly the smallest subnormal; 2^-25 is a tie that
    // nearest-even resolves to the even candidate, which is zero.
    assert_eq!(h(f32::from_bits(0x3380_0000)), 1);
    assert_eq!(
        convert::<B32, B16>(s(f32::from_bits(0x3300_0000)), RV),
        (0, Flags::INEXACT | Flags::UNDERFLOW)
    );
    // The arithmetic itself is format-generic: 2^-11 is half an ulp of one in
    // binary16 and the tie rounds to even, which is one.
    assert_eq!(add::<B16>(h(1.0), h(0.000_488_281_25), RV).0, h(1.0));
    // ... while 2^-10 is a whole ulp and does not.
    assert_eq!(add::<B16>(h(1.0), h(0.000_976_562_5), RV).0, 0x3c01);
    assert_eq!(div::<B16>(h(1.0), h(0.0), RV).1, Flags::DIV_BY_ZERO);
}

#[test]
fn round_to_integral_moves_in_the_direction_it_is_given() {
    let rint = |v: f64, r: Round| round_to_integral::<B64>(d(v), RV.round(r), false).0;
    for (v, near, away, zero, down, up) in [
        (2.5, 2.0, 3.0, 2.0, 2.0, 3.0),
        (3.5, 4.0, 4.0, 3.0, 3.0, 4.0),
        (-2.5, -2.0, -3.0, -2.0, -3.0, -2.0),
        (0.5, 0.0, 1.0, 0.0, 0.0, 1.0),
        (-0.5, -0.0, -1.0, -0.0, -1.0, -0.0),
        (1.4, 1.0, 1.0, 1.0, 1.0, 2.0),
    ] {
        assert_eq!(rint(v, Round::TiesEven), d(near), "rint {v} nearest");
        assert_eq!(rint(v, Round::TiesAway), d(away), "rint {v} away");
        assert_eq!(rint(v, Round::TowardZero), d(zero), "rint {v} zero");
        assert_eq!(rint(v, Round::TowardNegative), d(down), "rint {v} down");
        assert_eq!(rint(v, Round::TowardPositive), d(up), "rint {v} up");
    }
}

/// §5.9: the sign of a zero result is the operand's, which is the rule a
/// truncation written as `as i64 as f64` gets wrong.
#[test]
fn round_to_integral_keeps_the_sign_of_a_zero() {
    let rint = |v: f64, r: Round| round_to_integral::<B64>(d(v), RV.round(r), false).0;
    assert_eq!(rint(-0.25, Round::TiesEven), d(-0.0));
    assert_eq!(rint(-0.0, Round::TiesEven), d(-0.0));
    assert_eq!(rint(-0.75, Round::TowardPositive), d(-0.0));
    assert!(rint(-0.25, Round::TiesEven) & B64::SIGN != 0);
}

/// It is not a conversion, so nothing overflows and the huge values no integer
/// could hold come back untouched.
#[test]
fn round_to_integral_is_total() {
    let rint = |bits: u64| round_to_integral::<B64>(bits, RV, false);
    assert_eq!(rint(d(1e300)), (d(1e300), Flags::NONE));
    assert_eq!(rint(d(f64::INFINITY)), (d(f64::INFINITY), Flags::NONE));
    assert_eq!(rint(d(f64::NEG_INFINITY)).0, d(f64::NEG_INFINITY));
    // A quiet NaN passes through under RISC-V's rule as the canonical one.
    assert_eq!(rint(0x7ff8_0000_0000_0abc).0, B64::QUIET_NAN);
    // A signaling NaN raises invalid.
    assert!(rint(0x7ff0_0000_0000_0abc).1.contains(Flags::INVALID));
}

/// The two halves of §5.9: five operations that raise nothing, and
/// `roundToIntegralExact`, which raises inexact when it changed the value.
/// Arm spells that difference `FRINTZ` versus `FRINTX`.
#[test]
fn only_the_exact_form_reports_inexact() {
    assert_eq!(round_to_integral::<B64>(d(2.5), RV, false).1, Flags::NONE);
    assert_eq!(round_to_integral::<B64>(d(2.5), RV, true).1, Flags::INEXACT);
    // An operand that was already integral is not inexact either way, and
    // neither is a zero or an infinity.
    assert_eq!(round_to_integral::<B64>(d(2.0), RV, true).1, Flags::NONE);
    assert_eq!(round_to_integral::<B64>(d(-0.0), RV, true).1, Flags::NONE);
    assert_eq!(
        round_to_integral::<B64>(d(f64::INFINITY), RV, true).1,
        Flags::NONE
    );
}

/// ARM's `FPCR.FZ` flushes a subnormal *operand* to zero and reports it
/// through `FPSR.IDC`, which round-to-integral honours like every other
/// operation.
#[test]
fn round_to_integral_honours_flush_to_zero() {
    let arm = Env::ARM.flush(true);
    let (value, flags) = round_to_integral::<B64>(MIN_SUB64, arm, true);
    assert_eq!(value, 0);
    assert_eq!(flags, Flags::DENORMAL);
    // Without flushing the same operand rounds up to one under FRINTP.
    let up = Env::ARM.round(Round::TowardPositive);
    assert_eq!(round_to_integral::<B64>(MIN_SUB64, up, false).0, d(1.0));
}
