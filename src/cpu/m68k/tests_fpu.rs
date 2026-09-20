//! Hand-written tests for the MC68881/MC68882 coprocessor.
//!
//! There is no floating-point corpus for this part. The expectations here are
//! the manuals' own: M68881UM §2 for the registers, §3 for the data formats,
//! §4 for each instruction's operation table and Status Register section, and
//! §6 for the exception model. Where a value is a real number rather than a
//! bit pattern the expectation is the **correctly rounded** one, computed
//! independently — the constant ROM against two-hundred-digit arithmetic, the
//! arithmetic against `src/float`'s own tests, which are IEEE 754's.

use crate::core::error::Result;
use crate::float::x87::F80;

use super::tests_68010::{Board, restore, snapshot};
use super::{Config, Coprocessor, M68k, Model, Reg, vector};

/// `1.0`, and the powers of two around it.
const ONE: F80 = F80::new(0x3fff, 1 << 63);
const TWO: F80 = F80::new(0x4000, 1 << 63);
const THREE: F80 = F80::new(0x4000, 0xc000_0000_0000_0000);
const HALF: F80 = F80::new(0x3ffe, 1 << 63);
const MINUS_ONE: F80 = F80::new(0xbfff, 1 << 63);
const ZERO: F80 = F80::ZERO;
const MINUS_ZERO: F80 = F80::new(0x8000, 0);
const INF: F80 = F80::new(0x7fff, 1 << 63);
const MINUS_INF: F80 = F80::new(0xffff, 1 << 63);
/// The NaN the unit makes: all ones (M68881UM §3.2.5).
const CREATED_NAN: F80 = F80::new(0x7fff, u64::MAX);

/// A 68030 with a 68881, booted with `words` at `$400`.
fn board(words: &[u16]) -> Board {
    let b = Board::with_fpu(Model::M68030, Coprocessor::M68881);
    b.boot(words);
    b
}

/// Run one instruction at `$500`.
fn run(b: &Board, words: &[u16]) {
    b.load(0x500, words);
    b.at(0x500);
    b.cpu.step();
}

/// Put a value in `FPn` without executing anything.
fn set_fp(b: &Board, n: usize, value: F80) {
    b.with_regs(|r| r.fp[n] = value);
}

fn fp(b: &Board, n: usize) -> F80 {
    b.cpu.regs().fp[n]
}

/// The exception byte of `FPSR`, in the positions [`bits`] names — bits 15-8
/// of the register, which is where the enable byte's bits are too.
///
/// [`bits`]: super::fpu::bits
fn exc(b: &Board) -> u16 {
    (b.cpu.regs().fpsr & 0x0000_ff00) as u16
}

/// Place the supervisor stack pointer, which is the active `A7`.
fn set_sp(b: &Board, value: u32) {
    b.with_regs(|r| {
        r.a[7] = value;
        r.ssp = value;
    });
}

/// The four condition code bits, as `(N, Z, I, NAN)`.
fn cc(b: &Board) -> (bool, bool, bool, bool) {
    let fpsr = b.cpu.regs().fpsr;
    (
        fpsr & 0x0800_0000 != 0,
        fpsr & 0x0400_0000 != 0,
        fpsr & 0x0200_0000 != 0,
        fpsr & 0x0100_0000 != 0,
    )
}

/// `FMOVE.X FPm,FPn`, and every other register-to-register operation: the
/// command word is `000` then the source, the destination and the opmode.
const fn reg_op(src: u8, dst: u8, opmode: u8) -> u16 {
    ((src as u16) << 10) | ((dst as u16) << 7) | opmode as u16
}

/// The same with an effective-address source of format `fmt`.
const fn mem_op(fmt: u8, dst: u8, opmode: u8) -> u16 {
    0x4000 | ((fmt as u16) << 10) | ((dst as u16) << 7) | opmode as u16
}

#[test]
fn a_coprocessor_needs_a_coprocessor_interface() {
    use crate::core::props::Props;
    // The F-line coprocessor interface arrived with the 68020 (MC68020UM §7).
    for model in ["68000", "68010"] {
        assert!(
            M68k::from_props(&Props::new().with("model", model).with("fpu", "68881")).is_err(),
            "{model} has no coprocessor interface"
        );
    }
    for model in ["68020", "68ec020", "68030", "68ec030"] {
        let cpu = M68k::from_props(&Props::new().with("model", model).with("fpu", "68882"))
            .expect("a coprocessor");
        assert_eq!(cpu.config().fpu, Coprocessor::M68882);
    }
    // And with none attached, the F line is the F line.
    let plain = Board::new(Model::M68030);
    plain.boot(&[0xf200, 0x0000]); // FMOVE.X FP0,FP0
    plain.handler(0, vector::LINE_F, 0x0c00);
    plain.cpu.step();
    assert_eq!(plain.cpu.last_exception(), Some(vector::LINE_F));
}

#[test]
fn a_reset_fills_the_registers_with_positive_nans() {
    // M68881UM §2.1: "A reset function or a restore operation of the null
    // state sets FP0-FP7 to positive non-signaling not-a-numbers", and
    // §3.2.5's NaN has an all-ones mantissa.
    let b = board(&[0x4e71]);
    for n in 0..8 {
        assert_eq!(fp(&b, n), CREATED_NAN, "FP{n}");
    }
    let regs = b.cpu.regs();
    assert_eq!((regs.fpcr, regs.fpsr, regs.fpiar), (0, 0, 0));
}

#[test]
fn fmovecr_loads_the_constant_rom() {
    // The offsets are M68881UM §4's *FMOVECR* table; the values are the
    // correctly rounded ones, computed to two hundred decimal digits.
    let b = board(&[0x4e71]);
    for (offset, expected, what) in [
        (0x00u8, F80::new(0x4000, 0xc90f_daa2_2168_c235), "pi"),
        (0x0b, F80::new(0x3ffd, 0x9a20_9a84_fbcf_f799), "log10(2)"),
        (0x0c, F80::new(0x4000, 0xadf8_5458_a2bb_4a9b), "e"),
        (0x0d, F80::new(0x3fff, 0xb8aa_3b29_5c17_f0bc), "log2(e)"),
        (0x0e, F80::new(0x3ffd, 0xde5b_d8a9_3728_7195), "log10(e)"),
        (0x0f, ZERO, "0.0"),
        (0x30, F80::new(0x3ffe, 0xb172_17f7_d1cf_79ac), "ln(2)"),
        (0x31, F80::new(0x4000, 0x935d_8ddd_aaa8_ac17), "ln(10)"),
        (0x32, ONE, "10^0"),
        (0x33, F80::new(0x4002, 0xa000_0000_0000_0000), "10^1"),
        (0x3f, F80::new(0x7525, 0xc460_5202_8a20_979b), "10^4096"),
    ] {
        run(&b, &[0xf200, 0x5c00 | u16::from(offset)]);
        assert_eq!(fp(&b, 0), expected, "{what}");
    }
    // "The values contained at offsets other than those defined above are
    // reserved for the use of Motorola, and may be different on various mask
    // sets of the FPCP" — this core reads zero.
    run(&b, &[0xf200, 0x5c00 | 0x20]);
    assert_eq!(fp(&b, 0), ZERO);
}

#[test]
fn fmove_widens_every_external_format() {
    let b = board(&[0x4e71]);
    b.with_regs(|r| r.a[0] = 0x2100);
    // Byte, word and long integers are exact.
    b.poke_word(0x2100, 0x0300);
    run(&b, &[0xf210, mem_op(6, 0, 0x00)]); // FMOVE.B (A0),FP0
    assert_eq!(fp(&b, 0), THREE);
    b.poke_word(0x2100, 0xffff);
    run(&b, &[0xf210, mem_op(4, 0, 0x00)]); // FMOVE.W (A0),FP0
    assert_eq!(fp(&b, 0), MINUS_ONE);
    b.poke_long(0x2100, 0x0000_0002);
    run(&b, &[0xf210, mem_op(0, 0, 0x00)]); // FMOVE.L (A0),FP0
    assert_eq!(fp(&b, 0), TWO);
    // Single and double: 1.5 is exact in both.
    let three_halves = F80::new(0x3fff, 0xc000_0000_0000_0000);
    b.poke_long(0x2100, 0x3fc0_0000);
    run(&b, &[0xf210, mem_op(1, 0, 0x00)]); // FMOVE.S (A0),FP0
    assert_eq!(fp(&b, 0), three_halves);
    b.poke_long(0x2100, 0x3ff8_0000);
    b.poke_long(0x2104, 0);
    run(&b, &[0xf210, mem_op(5, 0, 0x00)]); // FMOVE.D (A0),FP0
    assert_eq!(fp(&b, 0), three_halves);
    // Extended is ninety-six bits: the exponent word, a word the processor
    // ignores, then the significand (M68881UM Table 3-3).
    b.poke_long(0x2100, 0x4000_dead);
    b.poke_long(0x2104, 0xc000_0000);
    b.poke_long(0x2108, 0);
    run(&b, &[0xf210, mem_op(2, 0, 0x00)]); // FMOVE.X (A0),FP0
    assert_eq!(fp(&b, 0), THREE);
    // An immediate operand comes out of the instruction stream.
    run(&b, &[0xf23c, mem_op(0, 1, 0x00), 0, 2]); // FMOVE.L #2,FP1
    assert_eq!(fp(&b, 1), TWO);
}

#[test]
fn fmove_narrows_every_external_format() {
    let b = board(&[0x4e71]);
    b.with_regs(|r| r.a[0] = 0x2100);
    set_fp(&b, 0, THREE);
    run(&b, &[0xf210, 0x6000]); // FMOVE.L FP0,(A0)
    assert_eq!(b.peek_long(0x2100), 3);
    run(&b, &[0xf210, 0x6000 | (1 << 10)]); // FMOVE.S FP0,(A0)
    assert_eq!(b.peek_long(0x2100), 0x4040_0000);
    run(&b, &[0xf210, 0x6000 | (5 << 10)]); // FMOVE.D FP0,(A0)
    assert_eq!(b.peek_long(0x2100), 0x4008_0000);
    assert_eq!(b.peek_long(0x2104), 0);
    run(&b, &[0xf210, 0x6000 | (2 << 10)]); // FMOVE.X FP0,(A0)
    assert_eq!(b.peek_long(0x2100), 0x4000_0000, "the unused word is zero");
    assert_eq!(b.peek_long(0x2104), 0xc000_0000);
    assert_eq!(b.peek_long(0x2108), 0);
    // "Condition Codes: Not affected" for a register-to-memory FMOVE
    // (§2.3.1), so the codes are whatever the last arithmetic left.
    set_fp(&b, 1, ZERO);
    run(&b, &[0xf200, reg_op(1, 1, 0x00)]); // FMOVE.X FP1,FP1
    assert_eq!(cc(&b), (false, true, false, false));
    run(&b, &[0xf210, 0x6000 | (1 << 10)]); // FMOVE.S FP0,(A0)
    assert_eq!(cc(&b), (false, true, false, false), "left alone");
}

#[test]
fn the_walking_modes_step_by_the_operands_own_width() {
    // M68881UM §4, *FMOVE*: both `(An)+` and `-(An)` are legal in both
    // directions, and each steps by the *format's* size — twelve bytes for
    // an extended operand, which no integer `Size` can name.
    let b = board(&[0x4e71]);
    b.with_regs(|r| r.a[0] = 0x2100);
    set_fp(&b, 0, THREE);
    run(&b, &[0xf218, 0x6800]); // FMOVE.X FP0,(A0)+
    assert_eq!(b.cpu.regs().a[0], 0x2100 + 12);
    run(&b, &[0xf220, 0x6800]); // FMOVE.X FP0,-(A0)
    assert_eq!(b.cpu.regs().a[0], 0x2100);
    set_fp(&b, 0, ZERO);
    run(&b, &[0xf218, mem_op(2, 0, 0x00)]); // FMOVE.X (A0)+,FP0
    assert_eq!(fp(&b, 0), THREE);
    assert_eq!(b.cpu.regs().a[0], 0x2100 + 12);
    run(&b, &[0xf220, mem_op(5, 1, 0x00)]); // FMOVE.D -(A0),FP1
    assert_eq!(b.cpu.regs().a[0], 0x2100 + 4, "a double is eight bytes");
    // A byte steps by one, a word by two, a long by four.
    b.with_regs(|r| r.a[0] = 0x2100);
    run(&b, &[0xf218, mem_op(6, 0, 0x00)]); // FMOVE.B (A0)+,FP0
    assert_eq!(b.cpu.regs().a[0], 0x2101);
    run(&b, &[0xf218, mem_op(4, 0, 0x00)]); // FMOVE.W (A0)+,FP0
    assert_eq!(b.cpu.regs().a[0], 0x2103);
    run(&b, &[0xf218, mem_op(0, 0, 0x00)]); // FMOVE.L (A0)+,FP0
    assert_eq!(b.cpu.regs().a[0], 0x2107);
}

#[test]
fn an_integer_store_that_does_not_fit_is_an_operand_error() {
    // §6.1.3, Table 6-2: "FMOVE to B, W, or L — Integer
    // Overflow/Underflow, Source is Non-Signaling NAN, or Source is
    // ±infinity", and the result is "the largest positive or negative integer
    // that can fit in the specified destination format size".
    let b = board(&[0x4e71]);
    b.with_regs(|r| r.a[0] = 0x2100);
    set_fp(&b, 0, F80::new(0x4020, 0x8000_0000_0000_0000)); // 2^33
    run(&b, &[0xf210, 0x6000]); // FMOVE.L FP0,(A0)
    assert_eq!(exc(&b) & 0x2000, 0x2000, "OPERR");
    assert_eq!(b.peek_long(0x2100), 0x7fff_ffff);
    set_fp(&b, 0, MINUS_INF);
    run(&b, &[0xf210, 0x6000]);
    assert_eq!(exc(&b) & 0x2000, 0x2000, "OPERR for an infinity");
    assert_eq!(b.peek_long(0x2100), 0x8000_0000);
    // "If the destination is B, W, or L and the floating-point number to be
    // stored is a NAN, then the 8, 16, or 32 most significant bits of the NAN
    // significand are stored as the result."
    set_fp(&b, 0, F80::new(0x7fff, 0xdead_beef_0000_0000));
    run(&b, &[0xf210, 0x6000]);
    assert_eq!(b.peek_long(0x2100), 0xdead_beef);
}

#[test]
fn fmovem_puts_the_registers_in_memory_in_the_same_order_either_way() {
    // M68000PRM §5, *FMOVEM*: the mask's bit 7 is FP7 in predecrement order
    // and FP0 in postincrement order, and the transfer runs from bit 0 in
    // both — so the registers land in memory the same way round whichever
    // form was used, and a save/restore pair agrees.
    let b = board(&[0x4e71]);
    for n in 0..8 {
        set_fp(&b, n, F80::new(0x3fff + n as u16, 1 << 63));
    }
    b.with_regs(|r| r.a[0] = 0x2100);
    run(&b, &[0xf210, 0xf0ff]); // FMOVEM.X FP0-FP7,(A0)  (control order)
    // Ascending memory holds FP7 first.
    for slot in 0..8u64 {
        let at = 0x2100 + 12 * slot;
        let expected = 0x3fff + (7 - slot) as u32;
        assert_eq!(b.peek_long(at) >> 16, expected, "slot {slot}");
    }
    for n in 0..8 {
        set_fp(&b, n, ZERO);
    }
    run(&b, &[0xf210, 0xd0ff]); // FMOVEM.X (A0),FP0-FP7
    for n in 0..8 {
        assert_eq!(fp(&b, n), F80::new(0x3fff + n as u16, 1 << 63), "FP{n}");
    }
    // Predecrement and postincrement are the pair a subroutine uses.
    set_sp(&b, 0x2800);
    run(&b, &[0xf227, 0xe0ff]); // FMOVEM.X FP0-FP7,-(A7)
    assert_eq!(b.cpu.regs().a[7], 0x2800 - 96);
    for n in 0..8 {
        set_fp(&b, n, ZERO);
    }
    run(&b, &[0xf21f, 0xd0ff]); // FMOVEM.X (A7)+,FP0-FP7
    assert_eq!(b.cpu.regs().a[7], 0x2800);
    for n in 0..8 {
        assert_eq!(fp(&b, n), F80::new(0x3fff + n as u16, 1 << 63), "FP{n}");
    }
    // A dynamic list names a data register (bits 6-4 of the command word).
    b.with_regs(|r| r.d[3] = 0x0000_0081);
    for n in 0..8 {
        set_fp(&b, n, ZERO);
    }
    b.poke_long(0x2100, 0x4000_0000);
    b.poke_long(0x2104, 0x8000_0000);
    b.poke_long(0x2108, 0);
    run(&b, &[0xf210, 0xd830]); // FMOVEM.X (A0),D3  (dynamic, control order)
    assert_eq!(fp(&b, 7), TWO, "bit 7 of the mask is FP0 in control order");
}

#[test]
fn the_control_registers_move_both_ways() {
    let b = board(&[0x4e71]);
    // FMOVE.L #$00000030,FPCR — round toward minus infinity.
    run(&b, &[0xf23c, 0x9000, 0, 0x0030]);
    assert_eq!(b.cpu.regs().fpcr, 0x30);
    // FMOVE.L FPCR,D0
    run(&b, &[0xf200, 0xb000]);
    assert_eq!(b.cpu.regs().d[0], 0x30);
    // FPIAR may use an address register, because it holds an address.
    b.with_regs(|r| r.fpiar = 0x1234_5678);
    run(&b, &[0xf208, 0xa400]); // FMOVE.L FPIAR,A0
    assert_eq!(b.cpu.regs().a[0], 0x1234_5678);
    // FPCR may not.
    b.handler(0, vector::LINE_F, 0x0c00);
    run(&b, &[0xf208, 0xb000]); // FMOVE.L FPCR,A0
    assert_eq!(b.cpu.last_exception(), Some(vector::LINE_F));
    // All three at once need memory, in FPCR, FPSR, FPIAR order.
    b.with_regs(|r| {
        r.a[1] = 0x2200;
        r.fpcr = 0x00b0;
        r.fpsr = 0x0f00_0000;
        r.fpiar = 0x0000_0400;
    });
    run(&b, &[0xf211, 0xbc00]); // FMOVEM.L FPCR/FPSR/FPIAR,(A1)
    assert_eq!(b.peek_long(0x2200), 0x00b0);
    assert_eq!(b.peek_long(0x2204), 0x0f00_0000);
    assert_eq!(b.peek_long(0x2208), 0x0000_0400);
    // And these never touch the exception byte or the condition codes
    // (§2.3.1, §2.3.3, §2.4).
    let before = b.cpu.regs().fpsr;
    b.with_regs(|r| r.fpiar = 0);
    run(&b, &[0xf211, 0xa400]); // FMOVE.L FPIAR,(A1)
    assert_eq!(b.cpu.regs().fpsr & 0x0fff_ff00, before & 0x0fff_ff00);
}

#[test]
fn the_condition_codes_are_the_result_data_type() {
    // Table 2-1, every row.
    let b = board(&[0x4e71]);
    for (value, expected, what) in [
        (ONE, (false, false, false, false), "+normalized"),
        (MINUS_ONE, (true, false, false, false), "-normalized"),
        (ZERO, (false, true, false, false), "+0"),
        (MINUS_ZERO, (true, true, false, false), "-0"),
        (INF, (false, false, true, false), "+infinity"),
        (MINUS_INF, (true, false, true, false), "-infinity"),
        (CREATED_NAN, (false, false, false, true), "+NAN"),
        (
            F80::new(0xffff, u64::MAX),
            (true, false, false, true),
            "-NAN",
        ),
    ] {
        set_fp(&b, 1, value);
        run(&b, &[0xf200, reg_op(1, 0, 0x00)]); // FMOVE.X FP1,FP0
        assert_eq!(cc(&b), expected, "{what}");
    }
}

#[test]
fn fcmp_follows_its_operation_table() {
    // M68881UM §4, *FCMP*: a three-way ordered test, not a subtraction. The
    // infinity bit is always cleared, equal infinities compare equal, and an
    // equal pair puts the destination's sign into N.
    let b = board(&[0x4e71]);
    let compare = |dest: F80, src: F80| {
        set_fp(&b, 0, dest);
        set_fp(&b, 1, src);
        run(&b, &[0xf200, reg_op(1, 0, 0x38)]); // FCMP.X FP1,FP0
        cc(&b)
    };
    let none = (false, false, false, false);
    let n = (true, false, false, false);
    let z = (false, true, false, false);
    let nz = (true, true, false, false);
    // In range against in range.
    assert_eq!(compare(TWO, ONE), none, "2 > 1");
    assert_eq!(compare(ONE, TWO), n, "1 < 2");
    assert_eq!(compare(ONE, ONE), z, "1 = 1");
    assert_eq!(compare(MINUS_ONE, MINUS_ONE), nz, "-1 = -1, N from FPn");
    assert_eq!(compare(ONE, MINUS_ONE), none);
    assert_eq!(compare(MINUS_ONE, ONE), n);
    // Zeros.
    assert_eq!(compare(ZERO, ZERO), z);
    assert_eq!(compare(ZERO, MINUS_ZERO), z);
    assert_eq!(compare(MINUS_ZERO, ZERO), nz);
    assert_eq!(compare(MINUS_ZERO, MINUS_ZERO), nz);
    // Infinities: equal ones compare equal, and I is never set.
    assert_eq!(compare(INF, INF), z);
    assert_eq!(compare(MINUS_INF, MINUS_INF), nz);
    assert_eq!(compare(INF, MINUS_INF), none);
    assert_eq!(compare(MINUS_INF, INF), n);
    assert_eq!(compare(ONE, INF), n);
    assert_eq!(compare(ONE, MINUS_INF), none);
    assert_eq!(compare(INF, ONE), none);
    assert_eq!(compare(MINUS_INF, ONE), n);
    assert_eq!(compare(ZERO, INF), n);
    assert_eq!(compare(ZERO, MINUS_INF), none);
    // A NaN in either operand is unordered.
    assert_eq!(compare(ONE, CREATED_NAN), (false, false, false, true));
    assert_eq!(compare(CREATED_NAN, ONE), (false, false, false, true));
}

#[test]
fn the_four_arithmetic_operations_are_exact_where_they_should_be() {
    let b = board(&[0x4e71]);
    let dyadic = |dest: F80, src: F80, opmode: u8| {
        set_fp(&b, 0, dest);
        set_fp(&b, 1, src);
        run(&b, &[0xf200, reg_op(1, 0, opmode)]);
        fp(&b, 0)
    };
    assert_eq!(dyadic(ONE, TWO, 0x22), THREE, "FADD");
    assert_eq!(dyadic(THREE, TWO, 0x28), ONE, "FSUB is FPn - source");
    assert_eq!(
        dyadic(THREE, TWO, 0x23),
        F80::new(0x4001, 0xc000_0000_0000_0000),
        "FMUL"
    );
    assert_eq!(dyadic(ONE, TWO, 0x20), HALF, "FDIV is FPn / source");
    // An inexact result sets INEX2 and nothing else.
    assert_eq!(
        dyadic(ONE, THREE, 0x20),
        F80::new(0x3ffd, 0xaaaa_aaaa_aaaa_aaab)
    );
    assert_eq!(exc(&b), 0x0200, "INEX2");
}

#[test]
fn a_divide_by_zero_and_an_operand_error_are_what_the_manual_says() {
    let b = board(&[0x4e71]);
    // §6.1.6: "For the FDIV ... instructions, return an infinity with the
    // sign set to the exclusive OR of the signs of the input operands."
    set_fp(&b, 0, MINUS_ONE);
    set_fp(&b, 1, ZERO);
    run(&b, &[0xf200, reg_op(1, 0, 0x20)]); // FDIV.X FP1,FP0
    assert_eq!(fp(&b, 0), MINUS_INF);
    assert_eq!(exc(&b), 0x0400, "DZ alone");
    // §6.1.3, Table 6-2: 0/0 is an operand error, and the result is "an
    // extended precision non-signaling NAN (with all ones mantissa)".
    set_fp(&b, 0, ZERO);
    run(&b, &[0xf200, reg_op(1, 0, 0x20)]);
    assert_eq!(fp(&b, 0), CREATED_NAN);
    assert_eq!(exc(&b), 0x2000, "OPERR");
    // (+inf) + (-inf) is the same.
    set_fp(&b, 0, INF);
    set_fp(&b, 1, MINUS_INF);
    run(&b, &[0xf200, reg_op(1, 0, 0x22)]);
    assert_eq!(fp(&b, 0), CREATED_NAN);
    assert_eq!(exc(&b), 0x2000);
    // And a NaN operand propagates instead: "if both operands are
    // non-signaling NANs, then the destination operand ... is returned"
    // (§4.5.4.1).
    let payload = F80::new(0x7fff, 0xc000_0000_0000_1234);
    let other = F80::new(0x7fff, 0xc000_0000_0000_5678);
    set_fp(&b, 0, payload);
    set_fp(&b, 1, other);
    run(&b, &[0xf200, reg_op(1, 0, 0x22)]);
    assert_eq!(fp(&b, 0), payload, "the destination's NaN wins");
    assert_eq!(exc(&b), 0);
    // One NaN alone is the one returned.
    set_fp(&b, 0, ONE);
    run(&b, &[0xf200, reg_op(1, 0, 0x22)]);
    assert_eq!(fp(&b, 0), other);
}

#[test]
fn a_signalling_nan_is_quieted_and_reported() {
    // §4.5.4.2: "the SNAN bit is set in the FPSR EXC byte ... the SNAN is
    // converted to a non-signaling NAN (by setting the SNAN bit in the
    // operand to a one), and the operation continues".
    let b = board(&[0x4e71]);
    let snan = F80::new(0x7fff, 0x8000_0000_0000_0001);
    set_fp(&b, 0, ONE);
    set_fp(&b, 1, snan);
    run(&b, &[0xf200, reg_op(1, 0, 0x22)]); // FADD.X FP1,FP0
    assert_eq!(exc(&b) & 0x4000, 0x4000, "SNAN");
    assert_eq!(fp(&b, 0), F80::new(0x7fff, 0xc000_0000_0000_0001));
}

#[test]
fn the_monadic_operations_are_the_manuals() {
    let b = board(&[0x4e71]);
    let monadic = |src: F80, opmode: u8| {
        set_fp(&b, 1, src);
        set_fp(&b, 0, CREATED_NAN);
        run(&b, &[0xf200, reg_op(1, 0, opmode)]);
        (fp(&b, 0), exc(&b))
    };
    assert_eq!(monadic(MINUS_ONE, 0x18).0, ONE, "FABS");
    assert_eq!(monadic(ONE, 0x1a).0, MINUS_ONE, "FNEG");
    assert_eq!(monadic(MINUS_ZERO, 0x18).0, ZERO, "FABS of -0");
    assert_eq!(
        monadic(F80::new(0x4000, 1 << 63), 0x04).0,
        F80::new(0x3fff, 0xb504_f333_f9de_6484),
        "FSQRT of 2"
    );
    assert_eq!(
        monadic(F80::new(0x4003, 1 << 63), 0x04).0,
        F80::new(0x4001, 1 << 63),
        "FSQRT of 16"
    );
    // A negative square root is an operand error (Table 6-2).
    assert_eq!(monadic(MINUS_ONE, 0x04), (CREATED_NAN, 0x2000));
    // FGETEXP: "±0 gives ±0.0" and an infinity is an operand error.
    assert_eq!(
        monadic(F80::new(0x4003, 1 << 63), 0x1e).0,
        F80::new(0x4001, 1 << 63),
        "FGETEXP of 2^4 is 4"
    );
    assert_eq!(monadic(ZERO, 0x1e), (ZERO, 0));
    assert_eq!(monadic(MINUS_ZERO, 0x1e), (MINUS_ZERO, 0));
    assert_eq!(monadic(INF, 0x1e), (CREATED_NAN, 0x2000));
    // FGETMAN: the significand in [1, 2) with the source's sign.
    assert_eq!(
        monadic(F80::new(0x4003, 0xc000_0000_0000_0000), 0x1f).0,
        F80::new(0x3fff, 0xc000_0000_0000_0000)
    );
    assert_eq!(monadic(ZERO, 0x1f), (ZERO, 0));
    assert_eq!(monadic(INF, 0x1f), (CREATED_NAN, 0x2000));
}

#[test]
fn fint_rounds_by_the_mode_and_fintrz_never_does() {
    let b = board(&[0x4e71]);
    let three_halves = F80::new(0x3fff, 0xc000_0000_0000_0000);
    let rounded = |value: F80, rnd: u32, opmode: u8| {
        b.with_regs(|r| r.fpcr = rnd << 4);
        set_fp(&b, 1, value);
        run(&b, &[0xf200, reg_op(1, 0, opmode)]);
        fp(&b, 0)
    };
    // Round to nearest, ties to even: 1.5 goes to 2.
    assert_eq!(rounded(three_halves, 0, 0x01), TWO);
    // Toward zero, toward minus infinity, toward plus infinity.
    assert_eq!(rounded(three_halves, 1, 0x01), ONE);
    assert_eq!(rounded(three_halves, 2, 0x01), ONE);
    assert_eq!(rounded(three_halves, 3, 0x01), TWO);
    // FINTRZ "always uses the round-to-zero mode, regardless of the current
    // rounding mode".
    assert_eq!(rounded(three_halves, 3, 0x03), ONE);
    assert_eq!(rounded(three_halves, 0, 0x03), ONE);
    b.with_regs(|r| r.fpcr = 0);
}

#[test]
fn fscale_and_the_single_precision_pair() {
    let b = board(&[0x4e71]);
    set_fp(&b, 0, ONE);
    set_fp(&b, 1, THREE);
    run(&b, &[0xf200, reg_op(1, 0, 0x26)]); // FSCALE.X FP1,FP0
    assert_eq!(fp(&b, 0), F80::new(0x4002, 1 << 63), "1 * 2^3");
    // An infinite source is an operand error (Table 6-2).
    set_fp(&b, 1, INF);
    run(&b, &[0xf200, reg_op(1, 0, 0x26)]);
    assert_eq!(exc(&b), 0x2000);
    // FSGLMUL rounds the mantissa to single precision and keeps the extended
    // exponent range (§2.2.2).
    let long_one = F80::new(0x3fff, 0x8000_0000_0000_0001);
    set_fp(&b, 0, long_one);
    set_fp(&b, 1, ONE);
    run(&b, &[0xf200, reg_op(1, 0, 0x27)]); // FSGLMUL
    assert_eq!(fp(&b, 0), ONE, "rounded to twenty-four bits");
    assert_eq!(exc(&b) & 0x0200, 0x0200, "INEX2");
}

#[test]
fn fmod_and_frem_are_exact_and_fill_the_quotient_byte() {
    // §2.3.2: the quotient byte holds "the seven least-significant bits of
    // the quotient (unsigned) and the sign of the entire quotient".
    let b = board(&[0x4e71]);
    let five = F80::new(0x4001, 0xa000_0000_0000_0000);
    let remainder = |dest: F80, src: F80, opmode: u8| {
        set_fp(&b, 0, dest);
        set_fp(&b, 1, src);
        run(&b, &[0xf200, reg_op(1, 0, opmode)]);
        (fp(&b, 0), (b.cpu.regs().fpsr >> 16) as u8)
    };
    // 5 mod 2: quotient 2, remainder 1.
    assert_eq!(remainder(five, TWO, 0x21), (ONE, 2));
    // 5 rem 2 rounds the quotient to nearest even — 2.5 ties to 2 — so the
    // remainder is the same.
    assert_eq!(remainder(five, TWO, 0x25), (ONE, 2));
    // 3 rem 2: the quotient rounds to 2 and the remainder turns negative.
    assert_eq!(remainder(THREE, TWO, 0x25), (MINUS_ONE, 2));
    assert_eq!(remainder(THREE, TWO, 0x21), (ONE, 1), "FMOD truncates");
    // A negative quotient sets the byte's high bit.
    assert_eq!(remainder(five, MINUS_ONE, 0x21).1, 0x85);
    // An infinite destination or a zero source is an operand error.
    assert_eq!(remainder(INF, TWO, 0x21).0, CREATED_NAN);
    assert_eq!(exc(&b), 0x2000);
    assert_eq!(remainder(TWO, ZERO, 0x21).0, CREATED_NAN);
    assert_eq!(exc(&b), 0x2000);
    // Operands far apart in magnitude still complete in one instruction,
    // which is where x87's partial `FPREM` would have stopped.
    let huge = F80::new(0x3fff + 200, 1 << 63);
    assert_eq!(remainder(huge, THREE, 0x21).0, ONE);
}

#[test]
fn the_rounding_precision_shortens_the_exponent_range_as_well() {
    // §2.2.2: "if the single or double precision mode is selected, the
    // exponent value is in the correct range for the single or double
    // precision format" — which x87's own precision control does not do.
    let b = board(&[0x4e71]);
    b.with_regs(|r| r.fpcr = 1 << 6); // single
    set_fp(&b, 0, F80::new(0x4000 + 127, 1 << 63)); // 2^128
    set_fp(&b, 1, TWO);
    run(&b, &[0xf200, reg_op(1, 0, 0x23)]); // FMUL.X FP1,FP0 -> 2^129
    assert_eq!(fp(&b, 0), INF, "overflowed single's range");
    assert_eq!(exc(&b) & 0x1000, 0x1000, "OVFL");
    // The same product in extended precision is an ordinary number.
    b.with_regs(|r| r.fpcr = 0);
    set_fp(&b, 0, F80::new(0x4000 + 127, 1 << 63));
    run(&b, &[0xf200, reg_op(1, 0, 0x23)]);
    assert_eq!(fp(&b, 0), F80::new(0x4000 + 128, 1 << 63));
    assert_eq!(exc(&b), 0);
}

#[test]
fn an_unnormalized_operand_is_normalized_before_use() {
    // M68881UM §3.5.1: "If an external operand is an extended precision
    // unnormalized number, the number is normalized before it is used in an
    // arithmetic operation" — where x87 calls the same encoding unsupported
    // and answers an invalid operation.
    let b = board(&[0x4e71]);
    b.with_regs(|r| r.a[0] = 0x2100);
    // $4001 with the significand one place down is 2.0 written twice.
    b.poke_long(0x2100, 0x4001_0000);
    b.poke_long(0x2104, 0x4000_0000);
    b.poke_long(0x2108, 0);
    run(&b, &[0xf210, mem_op(2, 0, 0x00)]); // FMOVE.X (A0),FP0
    assert_eq!(fp(&b, 0), TWO);
    assert_eq!(exc(&b), 0, "not an invalid operation");
    // A pseudo-infinity is an infinity and a pseudo-NaN is a NaN.
    b.poke_long(0x2100, 0x7fff_0000);
    b.poke_long(0x2104, 0);
    run(&b, &[0xf210, mem_op(2, 0, 0x00)]);
    assert_eq!(fp(&b, 0), INF);
    assert_eq!(exc(&b), 0);
    // And an unnormalized zero is a signed zero.
    b.poke_long(0x2100, 0xc001_0000);
    b.poke_long(0x2104, 0);
    run(&b, &[0xf210, mem_op(2, 0, 0x00)]);
    assert_eq!(fp(&b, 0), MINUS_ZERO);
}

#[test]
fn the_conditional_instructions_test_the_predicate_table() {
    let b = board(&[0x4e71]);
    set_fp(&b, 0, ZERO);
    run(&b, &[0xf200, reg_op(0, 0, 0x3a)]); // FTST.X FP0
    assert_eq!(cc(&b), (false, true, false, false));
    // FBEQ ($01) to $520, taken.
    b.load(0x520, &[0x4e71]);
    run(&b, &[0xf281, 0x001e]);
    assert_eq!(b.cpu.regs().pc, 0x520);
    // FBNE ($0e), not taken: falls through to the word after.
    run(&b, &[0xf28e, 0x001e]);
    assert_eq!(b.cpu.regs().pc, 0x504);
    // FSEQ D0 sets the byte.
    b.with_regs(|r| r.d[0] = 0);
    run(&b, &[0xf240, 0x0001]);
    assert_eq!(b.cpu.regs().d[0] & 0xff, 0xff);
    run(&b, &[0xf240, 0x000e]);
    assert_eq!(b.cpu.regs().d[0] & 0xff, 0x00);
    // FDBEQ D1,$520 — the condition is true, so the loop ends.
    b.with_regs(|r| r.d[1] = 4);
    run(&b, &[0xf249, 0x0001, 0x001c]);
    assert_eq!(b.cpu.regs().d[1], 4, "the counter is left alone");
    // False: the counter decrements and the branch is taken.
    run(&b, &[0xf249, 0x000e, 0x001c]);
    assert_eq!(b.cpu.regs().d[1], 3);
    assert_eq!(b.cpu.regs().pc, 0x520);
    // FTRAPEQ takes vector 7, the one TRAPcc takes.
    b.handler(0, vector::TRAPV, 0x0c00);
    run(&b, &[0xf27c, 0x0001]);
    assert_eq!(b.cpu.last_exception(), Some(vector::TRAPV));
}

#[test]
fn an_unordered_condition_sets_bsun_and_can_trap() {
    // M68000PRM Table 3-23: the sixteen predicates with bit 4 set — the
    // IEEE-nonaware and signalling halves — set BSUN when the NAN bit is set,
    // and the other sixteen do not.
    let b = board(&[0x4e71]);
    set_fp(&b, 0, CREATED_NAN);
    run(&b, &[0xf200, reg_op(0, 0, 0x3a)]); // FTST.X FP0
    assert_eq!(cc(&b), (false, false, false, true));
    // FBOGT ($02) is IEEE aware: no BSUN.
    run(&b, &[0xf282, 0x0004]);
    assert_eq!(exc(&b) & 0x8000, 0, "an aware predicate is quiet");
    // FBGT ($12) is not: BSUN.
    run(&b, &[0xf292, 0x0004]);
    assert_eq!(exc(&b) & 0x8000, 0x8000, "BSUN");
    assert_eq!(b.cpu.last_exception(), None, "the trap is disabled");
    // With the trap enabled it is a *pre-instruction* exception, so the
    // stacked program counter is the branch's own address and an RTE that
    // changes nothing runs into it again (§4, *FBcc*'s note).
    b.with_regs(|r| r.fpcr = 0x8000);
    b.handler(0, vector::FP_BSUN, 0x0c00);
    run(&b, &[0xf292, 0x0004]);
    assert_eq!(b.cpu.last_exception(), Some(vector::FP_BSUN));
    let sp = u64::from(b.cpu.regs().a[7]);
    assert_eq!(b.peek_long(sp + 2), 0x500, "the instruction's own address");
}

#[test]
fn an_enabled_arithmetic_trap_takes_its_vector() {
    // §2.2.1's priority order, and MC68030UM Table 8-1's vectors.
    let b = board(&[0x4e71]);
    for (enable, opmode, dest, src, want) in [
        (0x0400u32, 0x20u8, ONE, ZERO, vector::FP_DIVIDE_BY_ZERO),
        (0x2000, 0x20, ZERO, ZERO, vector::FP_OPERAND_ERROR),
        (0x0200, 0x20, ONE, THREE, vector::FP_INEXACT),
    ] {
        b.with_regs(|r| {
            r.fpcr = enable;
            r.fpsr = 0;
        });
        b.handler(0, want, 0x0c00);
        set_fp(&b, 0, dest);
        set_fp(&b, 1, src);
        run(&b, &[0xf200, reg_op(1, 0, opmode)]);
        assert_eq!(b.cpu.last_exception(), Some(want), "vector {want}");
        // The instruction's address is in FPIAR for the handler (§2.4).
        assert_eq!(b.cpu.regs().fpiar, 0x500);
    }
    // "The destination floating-point data register is not modified" for an
    // enabled DZ (§6.1.6), and *is* for an enabled INEX (§6.1.7).
    b.with_regs(|r| {
        r.fpcr = 0x0400;
        r.fpsr = 0;
    });
    set_fp(&b, 0, ONE);
    set_fp(&b, 1, ZERO);
    run(&b, &[0xf200, reg_op(1, 0, 0x20)]);
    assert_eq!(fp(&b, 0), ONE, "withheld");
    b.with_regs(|r| {
        r.fpcr = 0x0200;
        r.fpsr = 0;
    });
    set_fp(&b, 0, ONE);
    set_fp(&b, 1, THREE);
    run(&b, &[0xf200, reg_op(1, 0, 0x20)]);
    assert_eq!(fp(&b, 0), F80::new(0x3ffd, 0xaaaa_aaaa_aaaa_aaab), "stored");
}

#[test]
fn the_accrued_byte_is_sticky_and_the_exception_byte_is_not() {
    // §2.3.3: the exception byte "is cleared by the FPCP at the start of most
    // operations"; §2.3.4: the accrued byte "contains the history of all
    // floating-point exceptions that have occurred since the user last
    // cleared" it.
    let b = board(&[0x4e71]);
    set_fp(&b, 0, ONE);
    set_fp(&b, 1, THREE);
    run(&b, &[0xf200, reg_op(1, 0, 0x20)]); // FDIV: inexact
    assert_eq!(exc(&b), 0x0200);
    assert_eq!(b.cpu.regs().fpsr & 0xff, 0x08, "AEXC INEX");
    set_fp(&b, 0, ONE);
    set_fp(&b, 1, TWO);
    run(&b, &[0xf200, reg_op(1, 0, 0x23)]); // FMUL: exact
    assert_eq!(exc(&b), 0, "the exception byte was cleared");
    assert_eq!(b.cpu.regs().fpsr & 0xff, 0x08, "the accrued byte was not");
}

#[test]
fn fsave_and_frestore_carry_the_pending_exceptions() {
    // M68881UM §4, *FSAVE*: an untouched unit writes the four-byte null
    // frame; anything else writes the idle frame, twenty-eight bytes on a
    // 68881 and sixty on a 68882.
    let b = board(&[0x4e71]);
    set_sp(&b, 0x2800);
    run(&b, &[0xf327]); // FSAVE -(A7)
    assert_eq!(b.cpu.regs().a[7], 0x2800 - 4, "the null frame");
    assert_eq!(b.peek_long(0x2800 - 4), 0);
    // Touch it, then save again.
    set_fp(&b, 0, ONE);
    set_fp(&b, 1, THREE);
    run(&b, &[0xf200, reg_op(1, 0, 0x20)]); // FDIV: inexact
    set_sp(&b, 0x2800);
    run(&b, &[0xf327]);
    assert_eq!(b.cpu.regs().a[7], 0x2800 - 28, "the 68881's idle frame");
    let frame = 0x2800u64 - 28;
    // The format word carries a version in its high byte and the length of
    // what follows it in the low one, and the word after it is reserved.
    assert_eq!(b.peek_word(frame) & 0xff, 0x18, "the frame's length");
    assert_eq!(b.peek_word(frame + 2), 0, "the reserved word");
    assert_eq!(b.peek_long(frame + 4), 0x0200, "the pending INEX2");
    assert_eq!(exc(&b), 0, "and it was cleared internally");
    // FRESTORE puts it back.
    run(&b, &[0xf35f]); // FRESTORE (A7)+
    assert_eq!(b.cpu.regs().a[7], 0x2800);
    assert_eq!(exc(&b), 0x0200);
    // A null frame is "equivalent to a hardware reset of the FPCP".
    b.poke_long(0x2900, 0);
    b.with_regs(|r| r.a[0] = 0x2900);
    run(&b, &[0xf350]); // FRESTORE (A0)
    assert_eq!(fp(&b, 0), CREATED_NAN);
    assert_eq!(b.cpu.regs().fpsr, 0);
    // A format word nothing here wrote is a format error.
    b.poke_long(0x2900, 0x0118_0000);
    b.handler(0, vector::FORMAT_ERROR, 0x0c00);
    run(&b, &[0xf350]);
    assert_eq!(b.cpu.last_exception(), Some(vector::FORMAT_ERROR));
    // A 68882's idle frame is a different length.
    let big = Board::with_fpu(Model::M68030, Coprocessor::M68882);
    big.boot(&[0x4e71]);
    set_sp(&big, 0x2800);
    big.with_regs(|r| r.fp[0] = ONE);
    run(&big, &[0xf327]);
    assert_eq!(big.cpu.regs().a[7], 0x2800 - 60);
}

#[test]
fn a_coprocessor_snapshot_round_trips() -> Result<()> {
    let b = board(&[0x4e71]);
    b.with_regs(|r| {
        r.fp[3] = F80::new(0x4001, 0xdead_beef_0000_0001);
        r.fpcr = 0x00b0;
        r.fpsr = 0x0a00_0208;
        r.fpiar = 0x0000_0500;
    });
    let bytes = snapshot(&b.cpu)?;
    let other = M68k::new(Config::MC68030.with_fpu(Coprocessor::M68881));
    restore(&other, &bytes)?;
    assert_eq!(other.regs(), b.cpu.regs());
    assert_eq!(snapshot(&other)?, bytes, "a round trip is a fixed point");
    // A core with no coprocessor cannot take it.
    assert!(restore(&M68k::new(Config::MC68030), &bytes).is_err());
    Ok(())
}

#[test]
fn the_coprocessors_registers_can_be_named() {
    let with = Reg::all_for_config(Config::MC68030.with_fpu(Coprocessor::M68881));
    assert!(with.contains(&Reg::Fpcr));
    assert!(with.contains(&Reg::Fpsr));
    assert!(with.contains(&Reg::Fpiar));
    assert!(!Reg::all_for_config(Config::MC68030).contains(&Reg::Fpcr));
    assert_eq!(Reg::from_name("fpiar"), Some(Reg::Fpiar));
    for reg in with {
        assert_eq!(Reg::from_name(&alloc::format!("{reg}")), Some(reg));
    }
}

#[test]
fn a_coprocessor_instruction_is_as_long_as_the_disassembler_says() {
    // The same property the integer sweep asserts: for every encoding that
    // decodes and runs, the bytes the disassembler claims are the bytes the
    // program counter moved. It matters more here than anywhere else,
    // because a floating-point immediate is as wide as its *format* and no
    // `Size` can name twelve bytes.
    use super::disasm::disassemble_with;
    use super::isa::Copro;
    let b = board(&[0x4e71]);
    b.with_regs(|r| {
        r.a[0] = 0x2100;
        r.a[1] = 0x2200;
    });
    let cases: &[&[u16]] = &[
        &[0xf200, reg_op(1, 0, 0x22)],                   // FADD.X FP1,FP0
        &[0xf210, mem_op(2, 0, 0x00)],                   // FMOVE.X (A0),FP0
        &[0xf228, mem_op(5, 0, 0x22), 0x0010],           // FADD.D $10(A0),FP0
        &[0xf239, mem_op(1, 0, 0x22), 0, 0x2100],        // FADD.S ($2100).L,FP0
        &[0xf23c, mem_op(6, 0, 0x22), 0x0002],           // FADD.B #2,FP0
        &[0xf23c, mem_op(0, 0, 0x22), 0, 2],             // FADD.L #2,FP0
        &[0xf23c, mem_op(5, 0, 0x22), 0, 0, 0, 0],       // FADD.D #..,FP0
        &[0xf23c, mem_op(2, 0, 0x22), 0, 0, 0, 0, 0, 0], // FADD.X #..,FP0
        &[0xf200, 0x5c0c],                               // FMOVECR #$c,FP0
        &[0xf210, 0x6800],                               // FMOVE.X FP0,(A0)
        &[0xf210, 0x9000],                               // FMOVE.L (A0),FPCR
        &[0xf211, 0xbc00],                               // FMOVEM.L FPCR/FPSR/FPIAR,(A1)
        &[0xf210, 0xd0ff],                               // FMOVEM.X (A0),FP0-FP7
        &[0xf240, 0x0001],                               // FSEQ D0
        &[0xf281, 0x0002],                               // FBEQ.W
        &[0xf2c1, 0x0000, 0x0004],                       // FBEQ.L
        &[0xf249, 0x0001, 0x0002],                       // FDBEQ D1
        &[0xf27a, 0x0001, 0x1234],                       // FTRAPEQ.W #$1234
        &[0xf27b, 0x0001, 0x1234, 0x5678],               // FTRAPEQ.L
        &[0xf27c, 0x0001],                               // FTRAPEQ
    ];
    for words in cases {
        let claimed = disassemble_with(Model::M68030, Copro::FPU, 0x500, words).len;
        run(&b, words);
        let moved = b.cpu.regs().pc.wrapping_sub(0x500);
        assert_eq!(
            moved,
            u32::from(claimed),
            "{words:04x?}: moved {moved}, disassembler said {claimed}"
        );
    }
}

#[test]
fn the_disassembler_speaks_the_coprocessor() {
    use super::disasm::disassemble_with;
    use super::isa::Copro;
    let text = |words: &[u16]| {
        alloc::format!(
            "{}",
            disassemble_with(Model::M68030, Copro::FPU, 0x400, words)
        )
    };
    assert_eq!(text(&[0xf200, reg_op(1, 2, 0x22)]), "FADD.X FP1,FP2");
    assert_eq!(text(&[0xf210, mem_op(1, 3, 0x22)]), "FADD.S (A0),FP3");
    assert_eq!(text(&[0xf200, reg_op(1, 1, 0x18)]), "FABS.X FP1");
    assert_eq!(text(&[0xf200, reg_op(1, 2, 0x00)]), "FMOVE.X FP1,FP2");
    assert_eq!(text(&[0xf210, 0x6800]), "FMOVE.X FP0,(A0)");
    assert_eq!(text(&[0xf200, 0x5c0c]), "FMOVECR.X #$c,FP0");
    assert_eq!(text(&[0xf210, 0x9000]), "FMOVE.L (A0),FPCR");
    assert_eq!(text(&[0xf210, 0xbc00]), "FMOVE.L FPCR/FPSR/FPIAR,(A0)");
    assert_eq!(
        text(&[0xf210, 0xd0ff]),
        "FMOVEM.X (A0),FP0/FP1/FP2/FP3/FP4/FP5/FP6/FP7"
    );
    assert_eq!(text(&[0xf227, 0xe003]), "FMOVEM.X FP0/FP1,-(A7)");
    assert_eq!(text(&[0xf281, 0x001e]), "FBEQ.W $420");
    assert_eq!(text(&[0xf2c2, 0x0000, 0x0020]), "FBOGT.L $422");
    assert_eq!(text(&[0xf240, 0x0001]), "FSEQ D0");
    assert_eq!(text(&[0xf249, 0x0001, 0x001c]), "FDBEQ D1,$420");
    assert_eq!(text(&[0xf27c, 0x0012]), "FTRAPGT");
    assert_eq!(text(&[0xf327]), "FSAVE -(A7)");
    assert_eq!(text(&[0xf35f]), "FRESTORE (A7)+");
    // Without a coprocessor the same words are the line-F trap.
    assert_eq!(
        alloc::format!(
            "{}",
            disassemble_with(Model::M68030, Copro::NONE, 0x400, &[0xf200, 0x0422])
        ),
        "LINEF $f200"
    );
    // An instruction's length is the one the interpreter walks.
    let len = |words: &[u16]| disassemble_with(Model::M68030, Copro::FPU, 0, words).len;
    assert_eq!(len(&[0xf200, 0x0422]), 4);
    assert_eq!(len(&[0xf23c, mem_op(2, 0, 0x00), 0, 0, 0, 0, 0, 0]), 16);
    assert_eq!(len(&[0xf23c, mem_op(0, 0, 0x00), 0, 0]), 8);
    assert_eq!(len(&[0xf2c2, 0, 0]), 6);
}
