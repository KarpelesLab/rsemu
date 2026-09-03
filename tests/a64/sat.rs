// A64 conformance: saturating and rounding integer arithmetic, and `FPSR.QC`.
//
// Copyright (c) Karpeles Lab Inc. MIT. Written from DDI 0487; no emulator
// source of any licence was consulted.
//
// ---------------------------------------------------------------------------
// Why this guest exists, and where its oracle stops
// ---------------------------------------------------------------------------
//
// This group is the one place in the core where an oracle was *easy*, and the
// two halves of the file are the two things that means.
//
// **The first half has a real oracle.** `i8::saturating_add` and its relatives
// are the standard library's functions, and this file computes each
// expectation twice: once on the host, by `rustc`'s constant evaluator, and
// once in the guest, where the loops are long enough that LLVM lowers them to
// vector instructions. Nobody here chose those encodings, and nothing in this
// repository computed the expected values. A disagreement means one of two
// independent implementations is wrong — the same argument `fp_natural.rs`
// makes with `rustc_apfloat`, and a stronger one, because saturating integer
// arithmetic is exactly defined and has no rounding mode to argue about.
//
// What LLVM actually chose, at the commit that added this file, was `SQADD`,
// `SQSUB`, `UQADD` and `UQSUB` at **all four element widths** — `16B`, `8H`,
// `4S` and `2D`. It did *not* choose `SQABS` or `SQNEG`: `saturating_abs` and
// `saturating_neg` lower to a compare-and-select sequence rather than to the
// instruction named after them. The two kernels are kept because they cost
// nothing and still run vector code, but they are **not** evidence about
// `SQABS`, and this file does not claim they are. Every other member of the
// group is reached only by the directed half below.
//
// **The second half has none, and says so.** `FPSR.QC` is the cumulative
// saturation flag, and nothing in ordinary Rust reads it: a saturating add in
// Rust returns the clamped value and nothing else. So the `QC` cases are
// directed, written from DDI 0487, and expressed in inline assembly because
// that is the only route to the register. What they buy instead of an oracle
// is that the flag is observed *through the architecture* — `MSR FPSR, XZR`
// to clear it, the instruction, `MRS Xt, FPSR` to read it — rather than by a
// unit test poking at a struct field. And they are checked by mutation: making
// `SQADD` wrap, dropping the flag from any one instruction, or letting the
// halving adds raise it each makes a named case here fail.
//
// The clamped *value* and the flag are deliberately checked together, because
// a clamped result is indistinguishable from an honest one — `0x7f + 0x01`
// clamping to `0x7f` looks exactly like `0x7e + 0x01` — and the flag is the
// only thing that tells them apart.

#![no_std]
#![no_main]

include!("rt.rs");

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

/// How long the bulk arrays are. Long enough that LLVM vectorises rather than
/// unrolling, which is the whole point: a scalar `saturating_add` lowers to
/// `ADDS`/`CSEL` and would test nothing in this group.
const N: usize = 256;

/// A deterministic spread of bytes with both extremes forced in, because the
/// interesting inputs of a saturating add are exactly the ones a random sample
/// misses.
const fn spread_i8(seed: u32) -> [i8; N] {
    let mut out = [0i8; N];
    let mut state = seed;
    let mut i = 0;
    while i < N {
        state = state.wrapping_mul(1_103_515_245).wrapping_add(12345);
        out[i] = (state >> 16) as i8;
        i += 1;
    }
    out[0] = i8::MIN;
    out[1] = i8::MAX;
    out[2] = -1;
    out[3] = 0;
    out
}

const fn spread_i16(seed: u32) -> [i16; N] {
    let mut out = [0i16; N];
    let mut state = seed;
    let mut i = 0;
    while i < N {
        state = state.wrapping_mul(1_103_515_245).wrapping_add(12345);
        out[i] = (state >> 12) as i16;
        i += 1;
    }
    out[0] = i16::MIN;
    out[1] = i16::MAX;
    out[2] = i16::MIN;
    out[3] = -1;
    out
}

const fn spread_i32(seed: u32) -> [i32; N] {
    let mut out = [0i32; N];
    let mut state = seed;
    let mut i = 0;
    while i < N {
        state = state.wrapping_mul(1_103_515_245).wrapping_add(12345);
        out[i] = (state as i32) >> 2;
        i += 1;
    }
    out[0] = i32::MIN;
    out[1] = i32::MAX;
    out[2] = i32::MIN;
    out[3] = -1;
    out
}

const fn spread_i64(seed: u32) -> [i64; N] {
    let mut out = [0i64; N];
    let mut state = seed;
    let mut i = 0;
    while i < N {
        state = state.wrapping_mul(1_103_515_245).wrapping_add(12345);
        out[i] = ((state as i64) << 33) | (state as i64);
        i += 1;
    }
    out[0] = i64::MIN;
    out[1] = i64::MAX;
    out[2] = i64::MIN;
    out[3] = -1;
    out
}

static A8: [i8; N] = spread_i8(1);
static B8: [i8; N] = spread_i8(7);
static A16: [i16; N] = spread_i16(3);
static B16: [i16; N] = spread_i16(11);
static A32: [i32; N] = spread_i32(5);
static B32: [i32; N] = spread_i32(13);
static A64: [i64; N] = spread_i64(17);
static B64: [i64; N] = spread_i64(23);

// ---------------------------------------------------------------------------
// The kernels. Each is a `const fn`, so the host evaluates it and the guest
// runs the same text — a difference is a difference between `rustc`'s constant
// evaluator and this core's `SQADD`.
// ---------------------------------------------------------------------------

/// A hash of a result array, so a whole array's worth of lanes is one
/// comparison and a single wrong lane still shows.
const fn hash(v: &[i64; N]) -> u64 {
    let mut h = 0xcbf2_9ce4_8422_2325u64;
    let mut i = 0;
    while i < N {
        h = (h ^ v[i] as u64).wrapping_mul(0x0000_0100_0000_01b3);
        i += 1;
    }
    h
}

macro_rules! lanewise {
    ($name:ident, $ty:ty, $src:ident, $other:ident, $op:ident) => {
        const fn $name() -> u64 {
            let mut out = [0i64; N];
            let mut i = 0;
            while i < N {
                out[i] = ($src[i] as $ty).$op($other[i] as $ty) as i64;
                i += 1;
            }
            hash(&out)
        }
    };
}

macro_rules! unary {
    ($name:ident, $ty:ty, $src:ident, $op:ident) => {
        const fn $name() -> u64 {
            let mut out = [0i64; N];
            let mut i = 0;
            while i < N {
                out[i] = ($src[i] as $ty).$op() as i64;
                i += 1;
            }
            hash(&out)
        }
    };
}

lanewise!(sqadd_b, i8, A8, B8, saturating_add);
lanewise!(sqsub_b, i8, A8, B8, saturating_sub);
lanewise!(uqadd_b, u8, A8, B8, saturating_add);
lanewise!(uqsub_b, u8, A8, B8, saturating_sub);
lanewise!(sqadd_h, i16, A16, B16, saturating_add);
lanewise!(sqsub_h, i16, A16, B16, saturating_sub);
lanewise!(uqadd_h, u16, A16, B16, saturating_add);
lanewise!(uqsub_h, u16, A16, B16, saturating_sub);
lanewise!(sqadd_s, i32, A32, B32, saturating_add);
lanewise!(sqsub_s, i32, A32, B32, saturating_sub);
lanewise!(uqadd_s, u32, A32, B32, saturating_add);
lanewise!(uqsub_s, u32, A32, B32, saturating_sub);
lanewise!(sqadd_d, i64, A64, B64, saturating_add);
lanewise!(sqsub_d, i64, A64, B64, saturating_sub);
lanewise!(uqadd_d, u64, A64, B64, saturating_add);
lanewise!(uqsub_d, u64, A64, B64, saturating_sub);
unary!(sqabs_b, i8, A8, saturating_abs);
unary!(sqneg_b, i8, A8, saturating_neg);
unary!(sqabs_h, i16, A16, saturating_abs);
unary!(sqneg_s, i32, A32, saturating_neg);

/// The whole set, evaluated by the host at compile time.
static WANT: [u64; 20] = [
    sqadd_b(),
    sqsub_b(),
    uqadd_b(),
    uqsub_b(),
    sqadd_h(),
    sqsub_h(),
    uqadd_h(),
    uqsub_h(),
    sqadd_s(),
    sqsub_s(),
    uqadd_s(),
    uqsub_s(),
    sqadd_d(),
    sqsub_d(),
    uqadd_d(),
    uqsub_d(),
    sqabs_b(),
    sqneg_b(),
    sqabs_h(),
    sqneg_s(),
];

// ---------------------------------------------------------------------------
// The directed half: `FPSR.QC`
// ---------------------------------------------------------------------------

/// Run one saturating instruction with `V0` and `V1` loaded from `a` and `b`,
/// with `FPSR` cleared first, and report the low doubleword of the result
/// together with `FPSR`.
///
/// The macro exists because the alternative is nine copies of the same six
/// lines with one mnemonic changed, and the mnemonic is the only interesting
/// part.
macro_rules! with_qc {
    ($insn:literal, $a:expr, $b:expr) => {{
        let (out, fpsr): (u64, u64);
        unsafe {
            core::arch::asm!(
                "msr fpsr, xzr",
                "fmov d0, {a}",
                "fmov d1, {b}",
                $insn,
                "fmov {out}, d0",
                "mrs {fpsr}, fpsr",
                a = in(reg) $a,
                b = in(reg) $b,
                out = out(reg) out,
                fpsr = out(reg) fpsr,
                out("v0") _,
                out("v1") _,
                options(nomem, nostack),
            );
        }
        (out, (fpsr >> 27) & 1)
    }};
}

/// One directed case: what the instruction produced and what `QC` became,
/// beside what each should have been.
///
/// The value and the flag travel together on purpose. A clamped result is
/// indistinguishable from an honest one, so checking either alone would let
/// half of a wrong implementation through.
struct QcCase {
    got: u64,
    qc: u64,
    want: u64,
    want_qc: u64,
}

const fn qc(got: u64, qc: u64, want: u64, want_qc: u64) -> QcCase {
    QcCase {
        got,
        qc,
        want,
        want_qc,
    }
}

fn run() -> Report {
    // The one `black_box`: it makes the arrays run-time inputs, which is what
    // stops each loop folding into its answer at compile time. Everything
    // below it is ordinary Rust with no vector type in sight.
    let _ = black_box(&A8);
    let _ = black_box(&B8);
    let _ = black_box(&A16);
    let _ = black_box(&B16);
    let _ = black_box(&A32);
    let _ = black_box(&B32);
    let _ = black_box(&A64);
    let _ = black_box(&B64);

    let got: [u64; 20] = [
        sqadd_b(),
        sqsub_b(),
        uqadd_b(),
        uqsub_b(),
        sqadd_h(),
        sqsub_h(),
        uqadd_h(),
        uqsub_h(),
        sqadd_s(),
        sqsub_s(),
        uqadd_s(),
        uqsub_s(),
        sqadd_d(),
        sqsub_d(),
        uqadd_d(),
        uqsub_d(),
        sqabs_b(),
        sqneg_b(),
        sqabs_h(),
        sqneg_s(),
    ];
    let mut i = 0;
    while i < 20 {
        if got[i] != WANT[i] {
            return (10 + i as u64, got[i], WANT[i], 1);
        }
        i += 1;
    }

    // ---- The directed `QC` cases -------------------------------------
    //
    // Every expectation below is DDI 0487's, transcribed: this half has no
    // oracle and does not pretend to one.

    // `SQADD V0.16B`: 0x7f + 0x01 clamps to 0x7f in every lane, and the flag
    // says so — which the value alone cannot, because an honest 0x7f is the
    // same byte.
    let (a, b) = (0x7f7f_7f7f_7f7f_7f7fu64, 0x0101_0101_0101_0101u64);
    let (v, q) = with_qc!("sqadd v0.16b, v0.16b, v1.16b", a, b);
    let case1 = qc(v, q, 0x7f7f_7f7f_7f7f_7f7f, 1);

    // The same instruction one step below the boundary raises nothing.
    let (v, q) = with_qc!("sqadd v0.16b, v0.16b, v1.16b", 0x7e7e_7e7e_7e7e_7e7eu64, b);
    let case2 = qc(v, q, 0x7f7f_7f7f_7f7f_7f7f, 0);

    // One lane of sixteen — the top byte of the half `FMOV` loaded, with the
    // other fifteen zero. `QC` is cumulative over the whole register, so a
    // single clamp anywhere raises it, and that is information no lane's
    // result carries.
    let (v, q) = with_qc!(
        "sqadd v0.16b, v0.16b, v1.16b",
        0x7f00_0000_0000_0000u64,
        0x0100_0000_0000_0000u64
    );
    let case3 = qc(v, q, 0x7f00_0000_0000_0000, 1);

    // `UQSUB` clamps at zero rather than wrapping to 0xff.
    let (v, q) = with_qc!("uqsub v0.16b, v0.16b, v1.16b", 0u64, 0x0101_0101_0101_0101u64);
    let case4 = qc(v, q, 0, 1);

    // `SRHADD` rounds and cannot saturate, so it must leave `QC` alone even on
    // the inputs where an ordinary add would carry out of the element.
    let (v, q) = with_qc!(
        "srhadd v0.16b, v0.16b, v1.16b",
        0x7f7f_7f7f_7f7f_7f7fu64,
        0x7f7f_7f7f_7f7f_7f7fu64
    );
    let case5 = qc(v, q, 0x7f7f_7f7f_7f7f_7f7f, 0);

    // `SQDMULH V0.8H`: -0x8000 squared, doubled, is one past a signed
    // halfword, and it is the only input pair that saturates.
    let (v, q) = with_qc!(
        "sqdmulh v0.8h, v0.8h, v1.8h",
        0x8000_8000_8000_8000u64,
        0x8000_8000_8000_8000u64
    );
    let case6 = qc(v, q, 0x7fff_7fff_7fff_7fff, 1);

    // `SQXTUN` reads its source signed and bounds the result unsigned, so -1
    // clamps *down* to zero — the asymmetry a single signedness flag cannot
    // express.
    let (v, q) = with_qc!("sqxtun v0.8b, v0.8h", u64::MAX, 0u64);
    let case7 = qc(v, q, 0, 1);

    // A scalar form, at a width the encoding names rather than the doubleword
    // every other scalar in this core uses.
    let (v, q) = with_qc!("sqadd b0, b0, b1", 0x7fu64, 0x01u64);
    let case8 = qc(v, q, 0x7f, 1);

    // `SQSHL` by an immediate shifts *left*: 0x11 by three is 0x88, which does
    // not fit a signed byte.
    let (v, q) = with_qc!("sqshl v0.16b, v0.16b, #3", 0x1111_1111_1111_1111u64, 0u64);
    let case9 = qc(v, q, 0x7f7f_7f7f_7f7f_7f7f, 1);

    // ...and `SQSHRN` shifts right out of the same field: 0x1111 by three is
    // 0x222, which does not fit a signed byte either.
    let (v, q) = with_qc!("sqshrn v0.8b, v0.8h, #3", 0x1111_1111_1111_1111u64, 0u64);
    let case10 = qc(v, q, 0x7f7f_7f7f, 1);

    // A sticky flag: once raised, an instruction that does not saturate must
    // not clear it. The macro clears `FPSR` on entry, so this one does it by
    // hand.
    let sticky: u64;
    unsafe {
        core::arch::asm!(
            "msr fpsr, xzr",
            "movi v0.16b, #0x7f",
            "movi v1.16b, #1",
            "sqadd v0.16b, v0.16b, v1.16b",
            "movi v0.16b, #0",
            "sqadd v0.16b, v0.16b, v1.16b",
            "mrs {sticky}, fpsr",
            sticky = out(reg) sticky,
            out("v0") _,
            out("v1") _,
            options(nomem, nostack),
        );
    }
    let case11 = qc((sticky >> 27) & 1, 0, 1, 0);

    // ...and only a write to `FPSR` clears it, exactly like the exception
    // flags beside it.
    set_fpsr(0);
    let case12 = qc((fpsr() >> 27) & 1, 0, 0, 0);

    // `SUQADD` reads the *destination* as its signed accumulator and `Vn` as
    // an unsigned addend. `V0` holds 1 and `V1` holds 0xff, so the sum is
    // 1 + 255 = 256, which does not fit a signed byte and clamps to 127.
    //
    // The operands are chosen so that reading them the other way round gives a
    // **different** answer: 1 + -1 is 0 and clamps at nothing. The first draft
    // of this case used -1 and 254, which is 253 under either reading — a
    // case that cannot fail, and the mutation run is what found that out.
    let suqadd: u64;
    let suqadd_qc: u64;
    unsafe {
        core::arch::asm!(
            "msr fpsr, xzr",
            "movi v0.16b, #0x01",
            "movi v1.16b, #0xff",
            "suqadd v0.16b, v1.16b",
            "fmov {out}, d0",
            "mrs {fpsr}, fpsr",
            out = out(reg) suqadd,
            fpsr = out(reg) suqadd_qc,
            out("v0") _,
            out("v1") _,
            options(nomem, nostack),
        );
    }
    let case13 = qc(suqadd, (suqadd_qc >> 27) & 1, 0x7f7f_7f7f_7f7f_7f7f, 1);

    // `USQADD` is the mirror, and its operands are chosen the same way: a
    // zero unsigned accumulator taking a signed -1 clamps *down* to zero,
    // where the swapped reading would be 0 + 255 and clamp at nothing.
    let usqadd: u64;
    let usqadd_qc: u64;
    unsafe {
        core::arch::asm!(
            "msr fpsr, xzr",
            "movi v0.16b, #0x00",
            "movi v1.16b, #0xff",
            "usqadd v0.16b, v1.16b",
            "fmov {out}, d0",
            "mrs {fpsr}, fpsr",
            out = out(reg) usqadd,
            fpsr = out(reg) usqadd_qc,
            out("v0") _,
            out("v1") _,
            options(nomem, nostack),
        );
    }
    let case19 = qc(usqadd, (usqadd_qc >> 27) & 1, 0, 1);

    // `SQRSHL` by a negative amount is a rounding shift *right*, which cannot
    // saturate: -1 rounded right by one place is 0, not -1.
    let (v, q) = with_qc!(
        "sqrshl v0.16b, v0.16b, v1.16b",
        u64::MAX,
        0xffff_ffff_ffff_ffffu64
    );
    let case14 = qc(v, q, 0, 0);

    // `SQXTN` bounds the same bits `SQXTUN` did, but signed: -1 fits a signed
    // byte and does not raise the flag.
    let (v, q) = with_qc!("sqxtn v0.8b, v0.8h", u64::MAX, 0u64);
    let case15 = qc(v, q, 0xffff_ffff, 0);

    // `SQSHLU` reads a signed source and bounds the result unsigned, so a
    // negative input clamps *down* to zero however far it is shifted.
    let (v, q) = with_qc!("sqshlu v0.16b, v0.16b, #1", u64::MAX, 0u64);
    let case16 = qc(v, q, 0, 1);

    // `SQDMLAL` saturates twice: the doubled product first, then the
    // accumulation into `Vd`. Both operands are -0x8000, whose doubled square
    // is one past a signed word, and `Vd` starts at zero — so the product
    // clamps and the sum does not.
    let sqdmlal: u64;
    let sqdmlal_qc: u64;
    unsafe {
        core::arch::asm!(
            "msr fpsr, xzr",
            "movi v0.2d, #0",
            "fmov d1, {a}",
            "sqdmlal v0.4s, v1.4h, v1.4h",
            "fmov {out}, d0",
            "mrs {fpsr}, fpsr",
            a = in(reg) 0x8000_8000_8000_8000u64,
            out = out(reg) sqdmlal,
            fpsr = out(reg) sqdmlal_qc,
            out("v0") _,
            out("v1") _,
            options(nomem, nostack),
        );
    }
    let case17 = qc(sqdmlal, (sqdmlal_qc >> 27) & 1, 0x7fff_ffff_7fff_ffff, 1);

    // `SRSHR` rounds and saturates at nothing, and the operands are chosen so
    // that the rounding constant's *position* shows: it is added to the value
    // **before** the shift, not to the result after it. Alternating 3 and 4
    // bytes both round to 2; truncating would give 1 and 2, and rounding
    // afterwards would give 2 and 3, so this one case separates all three.
    let (v, q) = with_qc!("srshr v0.16b, v0.16b, #1", 0x0304_0304_0304_0304u64, 0u64);
    let case18 = qc(v, q, 0x0202_0202_0202_0202, 0);

    let cases = [
        case1, case2, case3, case4, case5, case6, case7, case8, case9, case10, case11, case12,
        case13, case14, case15, case16, case17, case18, case19,
    ];
    let mut i = 0;
    while i < cases.len() {
        let c = &cases[i];
        if c.got != c.want {
            return (100 + i as u64, c.got, c.want, 2);
        }
        if c.qc != c.want_qc {
            return (200 + i as u64, c.qc, c.want_qc, 3);
        }
        i += 1;
    }

    PASS
}
