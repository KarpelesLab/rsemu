// A64 conformance: the floating-point rules that are Arm's rather than IEEE's.
//
// Copyright (c) Karpeles Lab Inc. MIT. Written from DDI 0487 and IEEE
// 754-2019; no emulator source of any licence was consulted.
//
// ---------------------------------------------------------------------------
// These expectations are ours
// ---------------------------------------------------------------------------
//
// Unlike fp_arith.rs and fp_convert.rs, nothing here has an external oracle.
// `rustc` has no way to express "round toward +infinity", "what did `FPSR`
// accumulate", or "which NaN survived", so every value below is transcribed
// from DDI 0487 and cited where it is not obvious. These are *directed tests
// that happen to run in a guest* — the crate's unit tests assert the same
// rules against `src/float` directly, and what this file adds is that the
// rules survive the whole path: a real `MSR FPCR`, a real instruction fetch,
// a real `MRS FPSR`.
//
// Two of them are properties rather than transcriptions, and those are the
// stronger assertions: the four rounding directions of one inexact quotient
// must bracket each other in a fixed order whatever the value is, and a
// `FRINTX`/`FRINTZ` pair must differ only in `FPSR.IXC`.

#![no_std]
#![no_main]

include!("rt.rs");

/// `FPSR` cumulative flags (DDI 0487 D17.2, `FPSR`).
mod fpsr_bits {
    pub const IOC: u64 = 1 << 0;
    pub const DZC: u64 = 1 << 1;
    pub const OFC: u64 = 1 << 2;
    pub const UFC: u64 = 1 << 3;
    pub const IXC: u64 = 1 << 4;
    pub const IDC: u64 = 1 << 7;
}

/// `FPCR.RMode`, bits 23:22 — Arm's order, which is not x86's.
mod rmode {
    pub const RN: u64 = 0 << 22;
    pub const RP: u64 = 1 << 22;
    pub const RM: u64 = 2 << 22;
    pub const RZ: u64 = 3 << 22;
}

/// The `NZCV` values `FPCompare` writes (DDI 0487 `FPCompare`).
mod cmp {
    pub const LESS: u64 = 0x8000_0000;
    pub const EQUAL: u64 = 0x6000_0000;
    pub const GREATER: u64 = 0x2000_0000;
    pub const UNORDERED: u64 = 0x3000_0000;
}

/// A `binary64` value the optimiser cannot see through.
///
/// Every operand below goes through this, and it is not a nicety: LLVM
/// materialises a *constant* `0.0` in a `D` register with `MOVI Dd, #0`, which
/// is an Advanced SIMD encoding this core does not implement. Making the
/// operand opaque forces `FMOV Dd, Xn` instead — an ordinary scalar move — so
/// the guest stays inside the instruction set it is testing. It also stops the
/// optimiser folding the arithmetic away, which would make the test vacuous.
#[inline(never)]
fn opaque(bits: u64) -> f64 {
    f64::from_bits(black_box(bits))
}

macro_rules! chk {
    ($case:expr, $tag:expr, $got:expr, $want:expr) => {{
        let got = $got;
        let want = $want;
        if got != want {
            return ($case, got, want, $tag);
        }
    }};
}

/// The same, for a `(value, exceptions)` pair. A flag disagreement reports
/// with 1000 added to the tag, so a failure says which half of the pair was
/// wrong without needing a second register to say it in.
macro_rules! chk2 {
    ($case:expr, $tag:expr, $got:expr, $want:expr) => {{
        let (got_value, got_flags) = $got;
        let (want_value, want_flags) = $want;
        if got_value != want_value {
            return ($case, got_value, want_value, $tag);
        }
        if got_flags != want_flags {
            return ($case, got_flags, want_flags, $tag + 1000);
        }
    }};
}

/// `a / b` at `binary64`, with the exceptions it raised.
fn div_flagged(a: u64, b: u64) -> (u64, u64) {
    let result: f64;
    let flags: u64;
    unsafe {
        core::arch::asm!(
            "msr fpsr, xzr",
            "fdiv {r:d}, {a:d}, {b:d}",
            "mrs {f}, fpsr",
            r = out(vreg) result,
            a = in(vreg) opaque(a),
            b = in(vreg) opaque(b),
            f = out(reg) flags,
            options(nomem, nostack),
        );
    }
    (result.to_bits(), flags)
}

/// `a + b` at `binary64`, with the exceptions it raised.
fn add_flagged(a: u64, b: u64) -> (u64, u64) {
    let result: f64;
    let flags: u64;
    unsafe {
        core::arch::asm!(
            "msr fpsr, xzr",
            "fadd {r:d}, {a:d}, {b:d}",
            "mrs {f}, fpsr",
            r = out(vreg) result,
            a = in(vreg) opaque(a),
            b = in(vreg) opaque(b),
            f = out(reg) flags,
            options(nomem, nostack),
        );
    }
    (result.to_bits(), flags)
}

/// `a * b` at `binary64`, with the exceptions it raised.
fn mul_flagged(a: u64, b: u64) -> (u64, u64) {
    let result: f64;
    let flags: u64;
    unsafe {
        core::arch::asm!(
            "msr fpsr, xzr",
            "fmul {r:d}, {a:d}, {b:d}",
            "mrs {f}, fpsr",
            r = out(vreg) result,
            a = in(vreg) opaque(a),
            b = in(vreg) opaque(b),
            f = out(reg) flags,
            options(nomem, nostack),
        );
    }
    (result.to_bits(), flags)
}

/// `1 / 3` at a given rounding mode, restoring `FPCR` afterwards.
fn div_rounded(a: u64, b: u64, mode: u64) -> u64 {
    let result: f64;
    unsafe {
        core::arch::asm!(
            "mrs {save}, fpcr",
            "msr fpcr, {mode}",
            "isb",
            "fdiv {r:d}, {a:d}, {b:d}",
            "msr fpcr, {save}",
            "isb",
            r = out(vreg) result,
            a = in(vreg) opaque(a),
            b = in(vreg) opaque(b),
            mode = in(reg) mode,
            save = out(reg) _,
            options(nomem, nostack),
        );
    }
    result.to_bits()
}

/// `FCMP` or `FCMPE`, reporting `NZCV` and the exceptions.
fn compare(a: u64, b: u64, signalling: bool) -> (u64, u64) {
    let nzcv: u64;
    let flags: u64;
    unsafe {
        if signalling {
            core::arch::asm!(
                "msr fpsr, xzr",
                "fcmpe {a:d}, {b:d}",
                "mrs {n}, nzcv",
                "mrs {f}, fpsr",
                a = in(vreg) opaque(a),
                b = in(vreg) opaque(b),
                n = out(reg) nzcv,
                f = out(reg) flags,
                options(nomem, nostack),
            );
        } else {
            core::arch::asm!(
                "msr fpsr, xzr",
                "fcmp {a:d}, {b:d}",
                "mrs {n}, nzcv",
                "mrs {f}, fpsr",
                a = in(vreg) opaque(a),
                b = in(vreg) opaque(b),
                n = out(reg) nzcv,
                f = out(reg) flags,
                options(nomem, nostack),
            );
        }
    }
    (nzcv & 0xf000_0000, flags)
}

/// The four `binary64` two-operand instructions the mnemonic column cannot
/// spell as an operator.
macro_rules! binop_d {
    ($name:ident, $insn:literal) => {
        fn $name(a: u64, b: u64) -> u64 {
            let result: f64;
            unsafe {
                core::arch::asm!(
                    concat!($insn, " {r:d}, {a:d}, {b:d}"),
                    r = out(vreg) result,
                    a = in(vreg) opaque(a),
                    b = in(vreg) opaque(b),
                    options(nomem, nostack),
                );
            }
            result.to_bits()
        }
    };
}

binop_d!(fmax, "fmax");
binop_d!(fmin, "fmin");
binop_d!(fmaxnm, "fmaxnm");
binop_d!(fminnm, "fminnm");
binop_d!(fnmul, "fnmul");

/// One-source `binary64` instructions.
macro_rules! unop_d {
    ($name:ident, $insn:literal) => {
        fn $name(a: u64) -> u64 {
            let result: f64;
            unsafe {
                core::arch::asm!(
                    concat!($insn, " {r:d}, {a:d}"),
                    r = out(vreg) result,
                    a = in(vreg) opaque(a),
                    options(nomem, nostack),
                );
            }
            result.to_bits()
        }
    };
}

unop_d!(frintn, "frintn");
unop_d!(frintp, "frintp");
unop_d!(frintm, "frintm");
unop_d!(frintz, "frintz");
unop_d!(frinta, "frinta");
unop_d!(fsqrt, "fsqrt");
unop_d!(fabs_d, "fabs");
unop_d!(fneg_d, "fneg");

/// `FMADD`: `Ra + Rn * Rm`.
macro_rules! triop_d {
    ($name:ident, $insn:literal) => {
        fn $name(addend: u64, op1: u64, op2: u64) -> u64 {
            let result: f64;
            unsafe {
                core::arch::asm!(
                    concat!($insn, " {r:d}, {n:d}, {m:d}, {a:d}"),
                    r = out(vreg) result,
                    n = in(vreg) opaque(op1),
                    m = in(vreg) opaque(op2),
                    a = in(vreg) opaque(addend),
                    options(nomem, nostack),
                );
            }
            result.to_bits()
        }
    };
}

triop_d!(fmadd, "fmadd");
triop_d!(fmsub, "fmsub");
triop_d!(fnmadd, "fnmadd");
triop_d!(fnmsub, "fnmsub");

/// A float-to-integer conversion that names its own rounding direction.
macro_rules! cvt_d {
    ($name:ident, $insn:literal) => {
        fn $name(a: u64) -> u64 {
            let result: u64;
            unsafe {
                core::arch::asm!(
                    concat!($insn, " {r}, {a:d}"),
                    r = out(reg) result,
                    a = in(vreg) opaque(a),
                    options(nomem, nostack),
                );
            }
            result
        }
    };
}

cvt_d!(fcvtns, "fcvtns");
cvt_d!(fcvtas, "fcvtas");
cvt_d!(fcvtms, "fcvtms");
cvt_d!(fcvtps, "fcvtps");
cvt_d!(fcvtzs, "fcvtzs");

/// `FRINTX` and the exceptions it raised — the one rounding instruction that
/// reports inexact.
fn frintx_flagged(a: u64) -> (u64, u64) {
    let result: f64;
    let flags: u64;
    unsafe {
        core::arch::asm!(
            "msr fpsr, xzr",
            "frintx {r:d}, {a:d}",
            "mrs {f}, fpsr",
            r = out(vreg) result,
            a = in(vreg) opaque(a),
            f = out(reg) flags,
            options(nomem, nostack),
        );
    }
    (result.to_bits(), flags)
}

/// `FRINTZ` and the exceptions it raised.
fn frintz_flagged(a: u64) -> (u64, u64) {
    let result: f64;
    let flags: u64;
    unsafe {
        core::arch::asm!(
            "msr fpsr, xzr",
            "frintz {r:d}, {a:d}",
            "mrs {f}, fpsr",
            r = out(vreg) result,
            a = in(vreg) opaque(a),
            f = out(reg) flags,
            options(nomem, nostack),
        );
    }
    (result.to_bits(), flags)
}

/// A 16-byte-aligned buffer for the `LDP Q`/`STP Q` case.
#[repr(C, align(16))]
struct QBuf([u64; 8]);

static mut QBUF: QBuf = QBuf([0; 8]);

const ONE: u64 = 0x3ff0_0000_0000_0000;
const TWO: u64 = 0x4000_0000_0000_0000;
const THREE: u64 = 0x4008_0000_0000_0000;
const ZERO: u64 = 0;
const NEG_ZERO: u64 = 0x8000_0000_0000_0000;
const INF: u64 = 0x7ff0_0000_0000_0000;
const NEG_INF: u64 = 0xfff0_0000_0000_0000;
const QNAN: u64 = 0x7ff8_0000_0000_0000;
const QNAN_PAYLOAD: u64 = 0x7ff8_0000_0000_00aa;
const SNAN_PAYLOAD: u64 = 0x7ff0_0000_0000_00bb;
const MIN_SUBNORMAL: u64 = 1;
const MAX_FINITE: u64 = 0x7fef_ffff_ffff_ffff;
/// 2.5 and -2.5, the values every rounding direction separates.
const TWO_HALF: u64 = 0x4004_0000_0000_0000;
const NEG_TWO_HALF: u64 = 0xc004_0000_0000_0000;

#[allow(clippy::too_many_lines)]
fn run() -> Report {
    // ------------------------------------------------------------------
    // 1. `FPSR` accumulates, and nothing but a write to it clears a flag.
    // ------------------------------------------------------------------
    chk!(1, 0, div_flagged(ONE, THREE).1, fpsr_bits::IXC);
    chk2!(1, 1, div_flagged(ONE, ZERO), (INF, fpsr_bits::DZC));
    chk2!(1, 2, div_flagged(ZERO, ZERO), (QNAN, fpsr_bits::IOC));
    chk2!(1, 3, div_flagged(INF, INF), (QNAN, fpsr_bits::IOC));
    chk2!(1, 4, add_flagged(INF, NEG_INF), (QNAN, fpsr_bits::IOC));
    chk2!(1, 5, add_flagged(ONE, TWO), (THREE, 0));
    // Overflow always reports inexact with it (IEEE 754-2019 §7.4).
    chk2!(
        1,
        6,
        mul_flagged(MAX_FINITE, TWO),
        (INF, fpsr_bits::OFC | fpsr_bits::IXC)
    );
    // Underflow: tiny *and* inexact. The smallest subnormal halved is both.
    chk2!(
        1,
        7,
        mul_flagged(MIN_SUBNORMAL, 0x3fe0_0000_0000_0000),
        (ZERO, fpsr_bits::UFC | fpsr_bits::IXC)
    );
    // ... and a subnormal result that is *exact* is not an underflow, which
    // is the case a flag set from "the result is subnormal" gets wrong.
    chk2!(
        1,
        8,
        mul_flagged(0x0004_0000_0000_0000, 0x3fe0_0000_0000_0000),
        (0x0002_0000_0000_0000, 0)
    );
    // `FPCR.FZ` is clear, so `IDC` is never set: a subnormal operand is used
    // exactly. This is the assertion that catches flush-to-zero left on.
    chk2!(1, 9, add_flagged(MIN_SUBNORMAL, ZERO), (MIN_SUBNORMAL, 0));
    // The flags are sticky (IEEE 754-2019 §7.1): two operations raising
    // different exceptions leave both set, and only a write to `FPSR` clears
    // one.
    let sticky: u64;
    unsafe {
        core::arch::asm!(
            "msr fpsr, xzr",
            "fdiv {t:d}, {a:d}, {b:d}",
            "fdiv {t:d}, {a:d}, {z:d}",
            "mrs {f}, fpsr",
            t = out(vreg) _,
            a = in(vreg) opaque(1.0f64.to_bits()),
            b = in(vreg) opaque(3.0f64.to_bits()),
            z = in(vreg) opaque(0.0f64.to_bits()),
            f = out(reg) sticky,
            options(nomem, nostack),
        );
    }
    chk!(1, 10, sticky, fpsr_bits::IXC | fpsr_bits::DZC);
    // `FPCR.FZ` is clear, so nothing ever sets the input-denormal flag.
    chk!(1, 11, sticky & fpsr_bits::IDC, 0);
    set_fpsr(0);
    chk!(1, 12, fpsr(), 0);

    // ------------------------------------------------------------------
    // 2. `FPCR.RMode` — Arm's encoding, and the ordering property
    // ------------------------------------------------------------------
    //
    // 1/3 is inexact and its nearest `binary64` value lies *below* the exact
    // quotient, so rounding toward zero and toward -infinity give that value
    // and rounding toward +infinity gives the next one up. The anchors are
    // written out; the ordering is asserted as a property so a different value
    // could not satisfy it by accident.
    let rn = div_rounded(ONE, THREE, rmode::RN);
    let rz = div_rounded(ONE, THREE, rmode::RZ);
    let rm = div_rounded(ONE, THREE, rmode::RM);
    let rp = div_rounded(ONE, THREE, rmode::RP);
    chk!(2, 0, rn, 0x3fd5_5555_5555_5555);
    chk!(2, 1, rz, 0x3fd5_5555_5555_5555);
    chk!(2, 2, rm, 0x3fd5_5555_5555_5555);
    chk!(2, 3, rp, 0x3fd5_5555_5555_5556);
    // The same quotient negated: now it is toward-zero and toward-*plus*
    // infinity that agree, which is the asymmetry a sign-blind
    // implementation gets wrong.
    let n_rz = div_rounded(0xbff0_0000_0000_0000, THREE, rmode::RZ);
    let n_rp = div_rounded(0xbff0_0000_0000_0000, THREE, rmode::RP);
    let n_rm = div_rounded(0xbff0_0000_0000_0000, THREE, rmode::RM);
    chk!(2, 4, n_rz, 0xbfd5_5555_5555_5555);
    chk!(2, 5, n_rp, 0xbfd5_5555_5555_5555);
    chk!(2, 6, n_rm, 0xbfd5_5555_5555_5556);
    // `FPCR` came back to its reset value, so the mode really was restored.
    chk!(2, 7, fpcr() & (3 << 22), 0);
    // The properties, independent of the values above.
    chk!(2, 8, u64::from(rz == rm && rp == rz + 1), 1);
    chk!(2, 9, u64::from(rn == rz || rn == rp), 1);

    // ------------------------------------------------------------------
    // 3. `FCMP` writes four `NZCV` patterns, and unordered sets C and V
    // ------------------------------------------------------------------
    chk2!(3, 0, compare(ONE, TWO, false), (cmp::LESS, 0));
    chk2!(3, 1, compare(TWO, TWO, false), (cmp::EQUAL, 0));
    chk2!(3, 2, compare(THREE, TWO, false), (cmp::GREATER, 0));
    chk2!(3, 3, compare(QNAN, TWO, false), (cmp::UNORDERED, 0));
    // `FCMPE` raises invalid on a quiet NaN where `FCMP` does not; both do on
    // a signaling one.
    chk2!(3, 4, compare(QNAN, TWO, true), (cmp::UNORDERED, fpsr_bits::IOC));
    chk2!(
        3,
        5,
        compare(SNAN_PAYLOAD, TWO, false),
        (cmp::UNORDERED, fpsr_bits::IOC)
    );
    // `+0` and `-0` compare equal.
    chk2!(3, 6, compare(ZERO, NEG_ZERO, false), (cmp::EQUAL, 0));
    chk2!(3, 7, compare(NEG_INF, INF, false), (cmp::LESS, 0));

    // ------------------------------------------------------------------
    // 4. `FMAX`/`FMIN` propagate a NaN; the `NM` forms prefer a number
    // ------------------------------------------------------------------
    chk!(4, 0, fmax(ONE, TWO), TWO);
    chk!(4, 1, fmin(ONE, TWO), ONE);
    // `FPMax` of two zeros takes the most positive sign and `FPMin` the most
    // negative, whichever order they arrive in.
    chk!(4, 2, fmax(ZERO, NEG_ZERO), ZERO);
    chk!(4, 3, fmax(NEG_ZERO, ZERO), ZERO);
    chk!(4, 4, fmin(ZERO, NEG_ZERO), NEG_ZERO);
    chk!(4, 5, fmin(NEG_ZERO, ZERO), NEG_ZERO);
    // A quiet NaN wins `FMAX` and loses `FMAXNM`.
    chk!(4, 6, fmax(QNAN_PAYLOAD, TWO), QNAN_PAYLOAD);
    chk!(4, 7, fmaxnm(QNAN_PAYLOAD, TWO), TWO);
    chk!(4, 8, fminnm(TWO, QNAN_PAYLOAD), TWO);
    chk!(4, 9, fmaxnm(QNAN_PAYLOAD, QNAN_PAYLOAD), QNAN_PAYLOAD);
    // `FNMUL` negates the result, sign of a NaN included.
    chk!(4, 10, fnmul(TWO, THREE), 0xc018_0000_0000_0000);
    chk!(4, 11, fnmul(ONE, NEG_ZERO), ZERO);

    // ------------------------------------------------------------------
    // 5. `FMADD` and the three ways of negating it
    // ------------------------------------------------------------------
    //
    // DDI 0487 C7: `FMSUB` negates op1, `FNMADD` negates op1 and the addend,
    // `FNMSUB` negates the addend. Spelling `FNMADD` as `-(a*b+c)` gives the
    // same answer here and a different one for a zero result, which is why
    // this checks a zero too.
    chk!(5, 0, fmadd(ONE, TWO, THREE), 0x401c_0000_0000_0000); // 1 + 6 = 7
    chk!(5, 1, fmsub(ONE, TWO, THREE), 0xc014_0000_0000_0000); // 1 - 6 = -5
    // `FNMADD` negates *both* the addend and op1, so it is `-Ra - Rn*Rm` and
    // not `-(Ra + Rn*Rm)`; `FNMSUB` negates only the addend, giving
    // `Rn*Rm - Ra`. Getting these two the wrong way round is the easy mistake,
    // and the numbers below are the ones that catch it.
    chk!(5, 2, fnmadd(ONE, TWO, THREE), 0xc01c_0000_0000_0000); // -1 - 6 = -7
    chk!(5, 3, fnmsub(ONE, TWO, THREE), 0x4014_0000_0000_0000); // 6 - 1 = 5
    // Fused, and the difference is visible: `(1 + 2^-52) * (1 - 2^-53) - 1`.
    // The exact product is `1 + 2^-53 - 2^-105`, which is a hair below the
    // halfway point between 1 and the next value up — so rounding it *first*
    // gives exactly 1 and the subtraction then gives exactly zero. Rounding
    // once, at the end, keeps the whole 106-bit product and gives
    // `2^-53 - 2^-105`. An implementation that multiplies and then adds
    // returns zero here; one that fuses returns this.
    chk!(
        5,
        4,
        fmadd(0xbff0_0000_0000_0000, 0x3ff0_0000_0000_0001, 0x3fef_ffff_ffff_ffff),
        0x3c9f_ffff_ffff_fffe
    );
    // A quiet-NaN addend with `∞ × 0` gives the *default* NaN and raises
    // invalid — the payload is discarded (DDI 0487 `FPMulAdd`).
    chk!(5, 5, fmadd(QNAN_PAYLOAD, INF, ZERO), QNAN);
    // ... but with an ordinary product the addend's payload survives, and it
    // is searched for *before* op1 and op2.
    chk!(5, 6, fmadd(QNAN_PAYLOAD, ONE, ONE), QNAN_PAYLOAD);
    chk!(5, 7, fmadd(QNAN_PAYLOAD, QNAN, ONE), QNAN_PAYLOAD);
    // A signaling NaN anywhere beats a quiet one everywhere, quieted.
    chk!(5, 8, fmadd(QNAN_PAYLOAD, ONE, SNAN_PAYLOAD), 0x7ff8_0000_0000_00bb);

    // ------------------------------------------------------------------
    // 6. The `FRINT` family, and the one that reports
    // ------------------------------------------------------------------
    chk!(6, 0, frintn(TWO_HALF), TWO); // ties to even
    chk!(6, 1, frinta(TWO_HALF), THREE); // ties away
    chk!(6, 2, frintm(TWO_HALF), TWO);
    chk!(6, 3, frintp(TWO_HALF), THREE);
    chk!(6, 4, frintz(TWO_HALF), TWO);
    chk!(6, 5, frintn(NEG_TWO_HALF), 0xc000_0000_0000_0000); // -2
    chk!(6, 6, frinta(NEG_TWO_HALF), 0xc008_0000_0000_0000); // -3
    chk!(6, 7, frintm(NEG_TWO_HALF), 0xc008_0000_0000_0000);
    chk!(6, 8, frintp(NEG_TWO_HALF), 0xc000_0000_0000_0000);
    chk!(6, 9, frintz(NEG_TWO_HALF), 0xc000_0000_0000_0000);
    // The sign of a zero result is the operand's — `-0.25` rounds to `-0.0`.
    chk!(6, 10, frintz(0xbfd0_0000_0000_0000), NEG_ZERO);
    chk!(6, 11, frintp(0xbfd0_0000_0000_0000), NEG_ZERO);
    // Nothing overflows: a value too large to be anything but an integer
    // comes back untouched.
    chk!(6, 12, frintn(MAX_FINITE), MAX_FINITE);
    chk!(6, 13, frintz(INF), INF);
    // `FRINTX` and `FRINTZ` differ only in `IXC`.
    let (x_value, x_flags) = frintx_flagged(TWO_HALF);
    let (z_value, z_flags) = frintz_flagged(TWO_HALF);
    chk!(6, 14, x_value, TWO);
    chk!(6, 15, z_value, TWO);
    chk!(6, 16, x_flags, fpsr_bits::IXC);
    chk!(6, 17, z_flags, 0);
    // ... and neither reports on an operand that was already integral.
    chk2!(6, 18, frintx_flagged(TWO), (TWO, 0));

    // ------------------------------------------------------------------
    // 7. `FCVT*` — the direction is in the mnemonic
    // ------------------------------------------------------------------
    chk!(7, 0, fcvtns(TWO_HALF), 2);
    chk!(7, 1, fcvtas(TWO_HALF), 3);
    chk!(7, 2, fcvtms(TWO_HALF), 2);
    chk!(7, 3, fcvtps(TWO_HALF), 3);
    chk!(7, 4, fcvtzs(TWO_HALF), 2);
    chk!(7, 5, fcvtns(NEG_TWO_HALF), (-2i64) as u64);
    chk!(7, 6, fcvtas(NEG_TWO_HALF), (-3i64) as u64);
    chk!(7, 7, fcvtms(NEG_TWO_HALF), (-3i64) as u64);
    chk!(7, 8, fcvtps(NEG_TWO_HALF), (-2i64) as u64);
    chk!(7, 9, fcvtzs(NEG_TWO_HALF), (-2i64) as u64);
    // A NaN converts to zero and raises invalid; out of range saturates.
    chk!(7, 10, fcvtzs(QNAN), 0);
    chk!(7, 11, fcvtzs(INF), 0x7fff_ffff_ffff_ffff);
    chk!(7, 12, fcvtzs(NEG_INF), 0x8000_0000_0000_0000);

    // ------------------------------------------------------------------
    // 8. `FABS`, `FNEG` and `FSQRT`
    // ------------------------------------------------------------------
    //
    // `FNEG` is a bit operation: it flips the sign of a signaling NaN without
    // quietening it and without raising invalid, which computing it as
    // `0 - x` would get wrong twice over.
    chk!(8, 0, fneg_d(ONE), 0xbff0_0000_0000_0000);
    chk!(8, 1, fneg_d(NEG_ZERO), ZERO);
    chk!(8, 2, fabs_d(0xbff0_0000_0000_0000), ONE);
    chk!(8, 3, fneg_d(SNAN_PAYLOAD), SNAN_PAYLOAD | (1 << 63));
    chk!(8, 4, fabs_d(SNAN_PAYLOAD | (1 << 63)), SNAN_PAYLOAD);
    chk!(8, 5, fsqrt(0x4010_0000_0000_0000), TWO); // sqrt(4) = 2
    chk!(8, 6, fsqrt(ZERO), ZERO);
    chk!(8, 7, fsqrt(NEG_ZERO), NEG_ZERO);
    chk!(8, 8, fsqrt(INF), INF);
    // sqrt(2), correctly rounded — the value every implementation agrees on
    // and a Newton iteration that stops early does not.
    chk!(8, 9, fsqrt(TWO), 0x3ff6_a09e_667f_3bcd);

    // ------------------------------------------------------------------
    // 9. The register file: a scalar write zeroes the rest
    // ------------------------------------------------------------------
    //
    // DDI 0487 C1.2.2. This is guest-visible and software relies on it.
    let high: u64;
    let low: u64;
    unsafe {
        core::arch::asm!(
            // Fill both halves of V0, then write only its bottom 32 bits.
            "movz {t}, #0x1234",
            "movk {t}, #0x5678, lsl #16",
            "fmov d0, {t}",
            "fmov v0.d[1], {t}",
            "fadd s0, s0, s0",
            "fmov {hi}, v0.d[1]",
            "fmov {lo}, d0",
            t = out(reg) _,
            hi = out(reg) high,
            lo = out(reg) low,
            out("v0") _,
            options(nomem, nostack),
        );
    }
    chk!(9, 0, high, 0);
    // 0x56781234 as a `binary32` is a normal number, so doubling it adds one
    // to the exponent field and leaves the significand alone — and the top
    // half of the register, and the top half of `D0`, are both zero.
    chk!(9, 1, low >> 32, 0);
    chk!(9, 2, low, 0x56f8_1234);
    // `FMOV Vd.D[1], Xn` merges rather than replacing — the one SIMD&FP write
    // that does.
    let merged_hi: u64;
    let merged_lo: u64;
    unsafe {
        core::arch::asm!(
            "fmov d1, {a}",
            "fmov v1.d[1], {b}",
            "fmov {hi}, v1.d[1]",
            "fmov {lo}, d1",
            a = in(reg) 0xaaaa_bbbb_cccc_ddddu64,
            b = in(reg) 0x1111_2222_3333_4444u64,
            hi = out(reg) merged_hi,
            lo = out(reg) merged_lo,
            out("v1") _,
            options(nomem, nostack),
        );
    }
    chk!(9, 3, merged_hi, 0x1111_2222_3333_4444);
    chk!(9, 4, merged_lo, 0xaaaa_bbbb_cccc_dddd);

    // ------------------------------------------------------------------
    // 10. `FMOV` with an immediate, and half precision
    // ------------------------------------------------------------------
    let one_imm: f64;
    let neg_two_imm: f64;
    let half_bits: u64;
    let back: f64;
    unsafe {
        core::arch::asm!(
            "fmov {a:d}, #1.0",
            "fmov {b:d}, #-2.0",
            a = out(vreg) one_imm,
            b = out(vreg) neg_two_imm,
            options(nomem, nostack),
        );
        // `FCVT` to and from half precision is Armv8.0-A: the format exists
        // without `FEAT_FP16`'s arithmetic.
        core::arch::asm!(
            "fcvt h2, {a:d}",
            "fmov {h:w}, s2",
            "fcvt {r:d}, h2",
            a = in(vreg) opaque(1.5f64.to_bits()),
            h = out(reg) half_bits,
            r = out(vreg) back,
            out("v2") _,
            options(nomem, nostack),
        );
    }
    chk!(10, 0, one_imm.to_bits(), ONE);
    chk!(10, 1, neg_two_imm.to_bits(), 0xc000_0000_0000_0000);
    chk!(10, 2, half_bits & 0xffff, 0x3e00); // 1.5 as binary16
    chk!(10, 3, back.to_bits(), 0x3ff8_0000_0000_0000);

    // ------------------------------------------------------------------
    // 11. 128-bit loads and stores
    // ------------------------------------------------------------------
    // A `static` rather than a local: zeroing sixty-four bytes of stack is
    // exactly the shape LLVM lowers to `MOVI V0.2D, #0` plus a `STP Q0, Q0`,
    // and this core has no Advanced SIMD. The guests deliberately stay inside
    // the instruction set the core implements — when the vector instructions
    // land, this can go back to being a local and the difference will show.
    let buf = unsafe { &mut *core::ptr::addr_of_mut!(QBUF) };
    buf.0[0] = 0x0011_2233_4455_6677;
    buf.0[1] = 0x8899_aabb_ccdd_eeff;
    buf.0[2] = 0x0102_0304_0506_0708;
    buf.0[3] = 0x090a_0b0c_0d0e_0f10;
    let (a0, a1, b0, b1): (u64, u64, u64, u64);
    unsafe {
        let p = buf.0.as_mut_ptr();
        core::arch::asm!(
            "ldp q4, q5, [{p}]",
            "stp q5, q4, [{p}, #32]",
            "fmov {a0}, d4",
            "fmov {a1}, v4.d[1]",
            "fmov {b0}, d5",
            "fmov {b1}, v5.d[1]",
            p = in(reg) p,
            a0 = out(reg) a0,
            a1 = out(reg) a1,
            b0 = out(reg) b0,
            b1 = out(reg) b1,
            out("v4") _,
            out("v5") _,
        );
    }
    chk!(11, 0, a0, 0x0011_2233_4455_6677);
    chk!(11, 1, a1, 0x8899_aabb_ccdd_eeff);
    chk!(11, 2, b0, 0x0102_0304_0506_0708);
    chk!(11, 3, b1, 0x090a_0b0c_0d0e_0f10);
    // The pair was written back swapped.
    chk!(11, 4, buf.0[4], 0x0102_0304_0506_0708);
    chk!(11, 5, buf.0[5], 0x090a_0b0c_0d0e_0f10);
    chk!(11, 6, buf.0[6], 0x0011_2233_4455_6677);
    chk!(11, 7, buf.0[7], 0x8899_aabb_ccdd_eeff);

    // ------------------------------------------------------------------
    // 12. `FCSEL` and `FCCMP`
    // ------------------------------------------------------------------
    let selected: f64;
    let nzcv_taken: u64;
    let nzcv_not: u64;
    unsafe {
        core::arch::asm!(
            "fcmp {a:d}, {b:d}",
            "fcsel {r:d}, {a:d}, {b:d}, lt",
            a = in(vreg) opaque(1.0f64.to_bits()),
            b = in(vreg) opaque(2.0f64.to_bits()),
            r = out(vreg) selected,
            options(nomem, nostack),
        );
        // `FCCMP` with a condition that holds does the comparison; with one
        // that does not it forces the `#nzcv` immediate and raises nothing.
        core::arch::asm!(
            "fcmp {a:d}, {a:d}",          // sets Z, so EQ holds
            "fccmp {a:d}, {b:d}, #0, eq", // ... so this compares: 1 < 2, N set
            "mrs {n}, nzcv",
            a = in(vreg) opaque(1.0f64.to_bits()),
            b = in(vreg) opaque(2.0f64.to_bits()),
            n = out(reg) nzcv_taken,
            options(nomem, nostack),
        );
        core::arch::asm!(
            "msr fpsr, xzr",
            "fcmp {a:d}, {a:d}",           // Z set, so NE does not hold
            "fccmpe {a:d}, {s:d}, #7, ne", // ... so a signaling NaN is not read
            "mrs {n}, nzcv",
            "mrs {f}, fpsr",
            a = in(vreg) opaque(1.0f64.to_bits()),
            s = in(vreg) opaque(SNAN_PAYLOAD),
            n = out(reg) nzcv_not,
            f = out(reg) _,
            options(nomem, nostack),
        );
    }
    chk!(12, 0, selected.to_bits(), ONE);
    chk!(12, 1, nzcv_taken & 0xf000_0000, cmp::LESS);
    chk!(12, 2, nzcv_not & 0xf000_0000, 0x7 << 28);
    PASS
}
