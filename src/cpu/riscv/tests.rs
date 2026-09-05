//! Tests for the RISC-V core.
//!
//! The interesting ones are not "does `add` add". They are the places the
//! specification is surprising and an implementation is likely to be wrong:
//! the sign-extension rules that make RV32 and RV64 one core, the division
//! results that are defined rather than trapping, `LR`/`SC` losing its
//! reservation, the trap-delegation field shuffle, an end-to-end Sv39 walk
//! from supervisor mode, and NaN boxing.
//!
//! Instructions are assembled by the encoders below rather than pasted in as
//! hex, so a test says what it means; `isa`'s own tests already prove those
//! encoders agree with the decoder.

use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::device::{Device, ResetKind};
use crate::core::error::Result;
use crate::core::props::Props;
use crate::core::space::{AddressSpace, RamStore, Region};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::{AtomicU64, Ordering};

use super::csr::{Extensions, Priv, cause, irq, num, status};
use super::isa::Xlen;
use super::{CLASS, Config, Hart, X_NAMES, x_by_name};

// ---------------------------------------------------------------------------
// Assembly
// ---------------------------------------------------------------------------

/// An R-type instruction.
const fn r(opcode: u32, funct3: u32, funct7: u32, rd: u32, rs1: u32, rs2: u32) -> u32 {
    opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | (rs2 << 20) | (funct7 << 25)
}

/// An I-type instruction.
const fn i(opcode: u32, funct3: u32, rd: u32, rs1: u32, imm: i32) -> u32 {
    opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | (((imm as u32) & 0xfff) << 20)
}

/// An S-type instruction.
const fn s(opcode: u32, funct3: u32, rs1: u32, rs2: u32, imm: i32) -> u32 {
    let imm = imm as u32;
    opcode
        | ((imm & 0x1f) << 7)
        | (funct3 << 12)
        | (rs1 << 15)
        | (rs2 << 20)
        | (((imm >> 5) & 0x7f) << 25)
}

/// A B-type instruction.
const fn b(funct3: u32, rs1: u32, rs2: u32, imm: i32) -> u32 {
    let imm = imm as u32;
    0x63 | (((imm >> 11) & 1) << 7)
        | (((imm >> 1) & 0xf) << 8)
        | (funct3 << 12)
        | (rs1 << 15)
        | (rs2 << 20)
        | (((imm >> 5) & 0x3f) << 25)
        | (((imm >> 12) & 1) << 31)
}

/// A J-type instruction.
const fn j(rd: u32, imm: i32) -> u32 {
    let imm = imm as u32;
    0x6f | (rd << 7)
        | (((imm >> 12) & 0xff) << 12)
        | (((imm >> 11) & 1) << 20)
        | (((imm >> 1) & 0x3ff) << 21)
        | (((imm >> 20) & 1) << 31)
}

/// A U-type instruction.
const fn u(opcode: u32, rd: u32, imm: u32) -> u32 {
    opcode | (rd << 7) | (imm & 0xffff_f000)
}

const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    i(0x13, 0, rd, rs1, imm)
}
const fn add(rd: u32, rs1: u32, rs2: u32) -> u32 {
    r(0x33, 0, 0, rd, rs1, rs2)
}
const fn lui(rd: u32, imm: u32) -> u32 {
    u(0x37, rd, imm)
}
const fn auipc(rd: u32, imm: u32) -> u32 {
    u(0x17, rd, imm)
}
const fn csrrw(rd: u32, csr: u32, rs1: u32) -> u32 {
    i(0x73, 1, rd, rs1, csr as i32)
}
const fn csrrs(rd: u32, csr: u32, rs1: u32) -> u32 {
    i(0x73, 2, rd, rs1, csr as i32)
}
const ECALL: u32 = 0x0000_0073;
const EBREAK: u32 = 0x0010_0073;
const MRET: u32 = 0x3020_0073;
const SRET: u32 = 0x1020_0073;
const WFI: u32 = 0x1050_0073;

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// Where the test programs live, and how much RAM they get.
// Deliberately not 0x8000_0000: `lui` sign-extends on RV64, so a test that
// materialised that address with one instruction would get
// 0xffff_ffff_8000_0000 and quietly test the fault path instead.
const BASE: u64 = 0x2000_0000;
const RAM_SIZE: u64 = 0x40_0000;

/// A hart with RAM at [`BASE`] and a program already loaded.
struct Harness {
    hart: Hart,
    ram: Arc<RamStore>,
}

impl Harness {
    /// Build a hart of the given configuration with `program` at [`BASE`].
    fn with(cfg: Config, program: &[u32]) -> Harness {
        let ram = Arc::new(RamStore::new(RAM_SIZE));
        for (n, word) in program.iter().enumerate() {
            for (k, byte) in word.to_le_bytes().iter().enumerate() {
                ram.write_u8(n as u64 * 4 + k as u64, *byte).unwrap();
            }
        }
        let space = AddressSpace::new("mem", 64);
        space
            .topology()
            .map(Region::ram("ram", Arc::clone(&ram)), BASE)
            .unwrap();
        let hart = Hart::new(cfg.with_reset_vector(BASE));
        hart.attach_space(Arc::new(space));
        Harness { hart, ram }
    }

    /// The default: a bare RV64I machine-mode hart.
    fn rv64i(program: &[u32]) -> Harness {
        Harness::with(Config::rv64i(), program)
    }

    /// A full RV64GC hart with supervisor mode and no PMP, so a test can enter
    /// S-mode without programming physical memory protection first.
    fn rv64gc(program: &[u32]) -> Harness {
        let mut cfg = Config::rv64gc();
        cfg.pmp_count = 0;
        Harness::with(cfg, program)
    }

    /// Write one halfword-aligned 16-bit value, for the compressed tests.
    fn put_half(&self, offset: u64, value: u16) {
        for (k, byte) in value.to_le_bytes().iter().enumerate() {
            self.ram.write_u8(offset + k as u64, *byte).unwrap();
        }
    }

    /// Write a 64-bit value into RAM at a guest address.
    fn put_u64(&self, addr: u64, value: u64) {
        for (k, byte) in value.to_le_bytes().iter().enumerate() {
            self.ram.write_u8(addr - BASE + k as u64, *byte).unwrap();
        }
    }

    /// Read a 64-bit value from RAM at a guest address.
    fn get_u64(&self, addr: u64) -> u64 {
        let mut v = 0u64;
        for k in 0..8 {
            v |= u64::from(self.ram.read_u8(addr - BASE + k).unwrap()) << (8 * k);
        }
        v
    }

    /// Execute `n` instructions.
    fn steps(&self, n: usize) {
        for _ in 0..n {
            self.hart.step();
        }
    }
}

// ---------------------------------------------------------------------------
// Integer computation
// ---------------------------------------------------------------------------

#[test]
fn immediates_are_sign_extended() {
    let h = Harness::rv64i(&[addi(10, 0, -1), addi(11, 0, 2047), addi(12, 0, -2048)]);
    h.steps(3);
    assert_eq!(h.hart.x(10), u64::MAX);
    assert_eq!(h.hart.x(11), 2047);
    assert_eq!(h.hart.x(12), (-2048i64) as u64);
}

#[test]
fn x0_is_hard_wired_to_zero() {
    let h = Harness::rv64i(&[addi(0, 0, 42), add(1, 0, 0)]);
    h.steps(2);
    assert_eq!(h.hart.x(0), 0);
    assert_eq!(h.hart.x(1), 0);
    // Even through the debug setter.
    h.hart.set_x(0, 99);
    assert_eq!(h.hart.x(0), 0);
}

#[test]
fn upper_immediates_are_relative_to_the_right_thing() {
    let h = Harness::rv64i(&[lui(10, 0x1234_5000), auipc(11, 0x1000)]);
    h.steps(2);
    assert_eq!(h.hart.x(10), 0x1234_5000);
    assert_eq!(h.hart.x(11), BASE + 4 + 0x1000);
    // LUI sign-extends on RV64: bit 31 set means a negative result.
    let h = Harness::rv64i(&[lui(10, 0x8000_0000)]);
    h.steps(1);
    assert_eq!(h.hart.x(10), 0xffff_ffff_8000_0000);
}

#[test]
fn shifts_use_the_configured_width() {
    // slli a0, a1, 33 is legal on RV64 and illegal on RV32.
    let program = [
        addi(11, 0, 1),
        i(0x13, 1, 10, 11, 33), // slli a0, a1, 33
    ];
    let h = Harness::rv64i(&program);
    h.steps(2);
    assert_eq!(h.hart.x(10), 1 << 33);

    let h = Harness::with(Config::rv64i().with_ext(Extensions::I), &program);
    let mut cfg = h.hart.config();
    cfg.xlen = Xlen::Rv32;
    let h = Harness::with(cfg, &program);
    h.steps(2);
    assert_eq!(
        h.hart.csrs().mcause,
        cause::ILLEGAL_INSN,
        "a 33-bit shift does not exist on RV32"
    );
}

#[test]
fn logical_right_shift_respects_rv32() {
    // -1 >> 1 must be 0x7fff_ffff on RV32, not 0x7fff_ffff_ffff_ffff.
    let program = [
        addi(11, 0, -1),
        i(0x13, 5, 10, 11, 1), // srli a0, a1, 1
        r(0x13, 5, 0x20, 12, 11, 1),
    ];
    let mut cfg = Config::rv64i();
    cfg.xlen = Xlen::Rv32;
    let h = Harness::with(cfg, &program);
    h.steps(3);
    assert_eq!(h.hart.x(10), 0x7fff_ffff);
    assert_eq!(h.hart.x(12), u64::MAX, "arithmetic shift keeps the sign");
}

#[test]
fn word_instructions_sign_extend_their_results() {
    let h = Harness::rv64i(&[
        lui(11, 0x8000_0000),
        i(0x1b, 0, 10, 11, 0),    // addiw a0, a1, 0
        r(0x3b, 1, 0, 12, 11, 0), // sllw a2, a1, x0
        r(0x3b, 5, 0, 13, 11, 0), // srlw a3, a1, x0
    ]);
    h.steps(4);
    assert_eq!(h.hart.x(10), 0xffff_ffff_8000_0000);
    assert_eq!(h.hart.x(12), 0xffff_ffff_8000_0000);
    assert_eq!(h.hart.x(13), 0xffff_ffff_8000_0000);
}

#[test]
fn sltiu_compares_a_sign_extended_immediate_as_unsigned() {
    // The `seqz` idiom: sltiu rd, rs, 1.
    let h = Harness::rv64i(&[
        addi(11, 0, 0),
        i(0x13, 3, 10, 11, 1), // sltiu a0, a1, 1
        addi(11, 0, 1),
        i(0x13, 3, 12, 11, 1),
        i(0x13, 3, 13, 0, -1), // sltiu a3, x0, -1 -> 1
    ]);
    h.steps(5);
    assert_eq!(h.hart.x(10), 1);
    assert_eq!(h.hart.x(12), 0);
    assert_eq!(h.hart.x(13), 1);
}

// ---------------------------------------------------------------------------
// Memory
// ---------------------------------------------------------------------------

#[test]
fn loads_sign_or_zero_extend_by_their_width() {
    let program = [
        lui(11, BASE as u32),
        i(0x03, 0, 10, 11, 0x100), // lb  a0, 0x100(a1)
        i(0x03, 4, 12, 11, 0x100), // lbu a2, 0x100(a1)
    ];
    let h = Harness::rv64i(&program);
    h.ram.write_u8(0x100, 0xff).unwrap();
    h.steps(3);
    assert_eq!(h.hart.x(10), u64::MAX);
    assert_eq!(h.hart.x(12), 0xff);
}

#[test]
fn stores_and_loads_round_trip_at_every_width() {
    let program = [
        lui(11, BASE as u32),
        addi(10, 0, -2),
        s(0x23, 3, 11, 10, 0x200), // sd a0, 0x200(a1)
        i(0x03, 3, 12, 11, 0x200), // ld a2, 0x200(a1)
        i(0x03, 2, 13, 11, 0x200), // lw a3, 0x200(a1)
        i(0x03, 6, 14, 11, 0x200), // lwu a4, 0x200(a1)
    ];
    let h = Harness::rv64i(&program);
    h.steps(6);
    assert_eq!(h.hart.x(12), (-2i64) as u64);
    assert_eq!(h.hart.x(13), (-2i64) as u64);
    assert_eq!(h.hart.x(14), 0xffff_fffe);
}

#[test]
fn a_misaligned_access_is_performed_a_byte_at_a_time() {
    let program = [
        lui(11, BASE as u32),
        addi(10, 0, 0x123),
        s(0x23, 1, 11, 10, 0x201), // sh a0, 0x201(a1) — deliberately odd
        i(0x03, 5, 12, 11, 0x201), // lhu a2, 0x201(a1)
    ];
    let h = Harness::rv64i(&program);
    h.steps(4);
    assert_eq!(h.hart.x(12), 0x123);
    assert_eq!(h.hart.csrs().mcause, 0, "no trap was taken");
}

#[test]
fn a_misaligned_access_traps_when_the_hart_says_it_does_not_support_them() {
    let mut cfg = Config::rv64i();
    cfg.misaligned = false;
    let program = [
        lui(11, BASE as u32),
        i(0x03, 1, 12, 11, 0x201), // lh a2, 0x201(a1)
    ];
    let h = Harness::with(cfg, &program);
    h.steps(2);
    assert_eq!(h.hart.csrs().mcause, cause::LOAD_MISALIGNED);
    assert_eq!(h.hart.csrs().mtval, BASE + 0x201);
}

#[test]
fn an_access_to_nothing_raises_an_access_fault() {
    // A hart *can* report a bus fault, unlike a 6502 — this is that path.
    let h = Harness::rv64i(&[i(0x03, 3, 10, 0, 8)]); // ld a0, 8(x0)
    h.steps(1);
    assert_eq!(h.hart.csrs().mcause, cause::LOAD_ACCESS);
    assert_eq!(h.hart.csrs().mtval, 8);
    assert_eq!(h.hart.bus_faults(), 1);
}

// ---------------------------------------------------------------------------
// Control flow
// ---------------------------------------------------------------------------

#[test]
fn branches_compare_signed_and_unsigned_differently() {
    // -1 is less than 1 signed, and greater unsigned.
    let program = [
        addi(10, 0, -1),
        addi(11, 0, 1),
        b(4, 10, 11, 8),  // blt a0, a1, +8 -> taken
        addi(12, 0, 111), // skipped
        b(6, 10, 11, 8),  // bltu a0, a1, +8 -> not taken
        addi(13, 0, 222), // executed
    ];
    let h = Harness::rv64i(&program);
    h.steps(5);
    assert_eq!(h.hart.x(12), 0);
    assert_eq!(h.hart.x(13), 222);
}

#[test]
fn jal_links_the_following_instruction_and_jalr_clears_the_low_bit() {
    let h = Harness::rv64i(&[j(1, 8), addi(10, 0, 1), addi(10, 0, 2)]);
    h.steps(2);
    assert_eq!(h.hart.x(1), BASE + 4);
    assert_eq!(h.hart.x(10), 2, "the middle instruction was skipped");

    // jalr with a tagged target: the low bit is cleared, not rejected.
    let h = Harness::rv64i(&[lui(11, BASE as u32), i(0x67, 0, 1, 11, 9)]);
    h.steps(2);
    assert_eq!(h.hart.pc(), BASE + 8);
}

#[test]
fn a_misaligned_jump_target_traps_without_c() {
    let h = Harness::rv64i(&[j(1, 6)]);
    h.steps(1);
    assert_eq!(h.hart.csrs().mcause, cause::INSN_MISALIGNED);
    assert_eq!(h.hart.csrs().mtval, BASE + 6);
    // With C the same target is perfectly legal.
    let h = Harness::rv64gc(&[j(1, 6)]);
    h.steps(1);
    assert_eq!(h.hart.pc(), BASE + 6);
}

// ---------------------------------------------------------------------------
// M
// ---------------------------------------------------------------------------

#[test]
fn multiplication_produces_both_halves() {
    let mut cfg = Config::rv64i();
    cfg.ext.m = true;
    let program = [
        addi(10, 0, -1),
        addi(11, 0, -1),
        r(0x33, 0, 1, 12, 10, 11), // mul
        r(0x33, 1, 1, 13, 10, 11), // mulh
        r(0x33, 3, 1, 14, 10, 11), // mulhu
        r(0x33, 2, 1, 15, 10, 11), // mulhsu
    ];
    let h = Harness::with(cfg, &program);
    h.steps(6);
    assert_eq!(h.hart.x(12), 1, "(-1) * (-1) is 1");
    assert_eq!(h.hart.x(13), 0, "the signed high half is zero");
    assert_eq!(h.hart.x(14), 0xffff_ffff_ffff_fffe);
    assert_eq!(h.hart.x(15), u64::MAX);
}

#[test]
fn division_by_zero_and_overflow_have_defined_results_and_do_not_trap() {
    let mut cfg = Config::rv64i();
    cfg.ext.m = true;
    let program = [
        addi(10, 0, 7),
        addi(11, 0, 0),
        r(0x33, 4, 1, 12, 10, 11), // div  a2, a0, zero
        r(0x33, 6, 1, 13, 10, 11), // rem  a3, a0, zero
        r(0x33, 5, 1, 14, 10, 11), // divu a4, a0, zero
        r(0x33, 7, 1, 15, 10, 11), // remu a5, a0, zero
    ];
    let h = Harness::with(cfg, &program);
    h.steps(6);
    assert_eq!(h.hart.x(12), u64::MAX, "division by zero gives all ones");
    assert_eq!(h.hart.x(13), 7, "the remainder is the dividend");
    assert_eq!(h.hart.x(14), u64::MAX);
    assert_eq!(h.hart.x(15), 7);
    assert_eq!(h.hart.csrs().mcause, 0, "and nothing traps");

    // The signed overflow case: the most negative value divided by -1.
    let program = [
        addi(10, 0, 1),
        i(0x13, 1, 10, 10, 63), // slli a0, a0, 63
        addi(11, 0, -1),
        r(0x33, 4, 1, 12, 10, 11),
        r(0x33, 6, 1, 13, 10, 11),
    ];
    let h = Harness::with(cfg, &program);
    h.steps(5);
    assert_eq!(h.hart.x(12), 1u64 << 63);
    assert_eq!(h.hart.x(13), 0);
}

#[test]
fn the_word_divisions_operate_on_the_low_halves() {
    let mut cfg = Config::rv64i();
    cfg.ext.m = true;
    let program = [
        lui(10, 0x8000_0000),
        addi(11, 0, -1),
        r(0x3b, 4, 1, 12, 10, 11), // divw: overflow of the 32-bit domain
        r(0x3b, 6, 1, 13, 10, 11), // remw
    ];
    let h = Harness::with(cfg, &program);
    h.steps(4);
    assert_eq!(h.hart.x(12), 0xffff_ffff_8000_0000);
    assert_eq!(h.hart.x(13), 0);
}

// ---------------------------------------------------------------------------
// A
// ---------------------------------------------------------------------------

/// `lr.d` / `sc.d` / an AMO, given the funct5 the specification assigns.
const fn amo(funct5: u32, funct3: u32, rd: u32, rs1: u32, rs2: u32) -> u32 {
    0x2f | (rd << 7) | (funct3 << 12) | (rs1 << 15) | (rs2 << 20) | (funct5 << 27)
}

#[test]
fn store_conditional_succeeds_only_while_the_reservation_holds() {
    let mut cfg = Config::rv64i();
    cfg.ext.a = true;
    let program = [
        lui(11, BASE as u32),
        addi(11, 11, 0x400),
        amo(0b00010, 3, 10, 11, 0), // lr.d a0, (a1)
        addi(12, 0, 99),
        amo(0b00011, 3, 13, 11, 12), // sc.d a3, a2, (a1)
        amo(0b00011, 3, 14, 11, 12), // sc.d again — the reservation is gone
    ];
    let h = Harness::with(cfg, &program);
    h.steps(6);
    assert_eq!(h.hart.x(13), 0, "the first store-conditional succeeds");
    assert_eq!(h.hart.x(14), 1, "the second one has no reservation left");
    assert_eq!(h.get_u64(BASE + 0x400), 99);
}

#[test]
fn an_intervening_store_breaks_the_reservation() {
    let mut cfg = Config::rv64i();
    cfg.ext.a = true;
    let program = [
        lui(11, BASE as u32),
        addi(11, 11, 0x400),
        amo(0b00010, 3, 10, 11, 0), // lr.d
        addi(12, 0, 1),
        s(0x23, 3, 11, 12, 0),       // sd a2, 0(a1) — clobbers the reservation
        amo(0b00011, 3, 13, 11, 12), // sc.d must fail
    ];
    let h = Harness::with(cfg, &program);
    h.steps(6);
    assert_eq!(h.hart.x(13), 1);
}

/// Two harts, one address space, one word — the reason the reservation cannot
/// live inside a hart.
///
/// Volume I: an `SC` succeeds "only if no other harts or devices have written
/// to the reservation set between the `LR` and the `SC`". While each hart kept
/// its reservation privately, a sibling's store did not break it and this
/// `sc.d` returned 0 — writing over an update the other hart had already made.
/// `machines/riscv-virt` runs two harts; so does every `usermode` process with
/// two threads.
#[test]
fn a_sibling_harts_store_breaks_this_harts_reservation() {
    let mut cfg = Config::rv64i();
    cfg.ext.a = true;
    /// The contended word, well clear of either program.
    const WORD: u64 = BASE + 0x400;
    /// Where the second hart's one instruction lives.
    const SIBLING: u64 = BASE + 0x100;

    let ram = Arc::new(RamStore::new(RAM_SIZE));
    let put = |at: u64, program: &[u32]| {
        for (n, word) in program.iter().enumerate() {
            for (k, byte) in word.to_le_bytes().iter().enumerate() {
                ram.write_u8(at - BASE + n as u64 * 4 + k as u64, *byte)
                    .unwrap();
            }
        }
    };
    // Hart 0 reserves, waits while hart 1 runs, then tries to store.
    put(
        BASE,
        &[
            lui(11, BASE as u32),
            addi(11, 11, 0x400),
            amo(0b00010, 3, 10, 11, 0), // lr.d a0, (a1)
            addi(12, 0, 99),
            amo(0b00011, 3, 13, 11, 12), // sc.d a3, a2, (a1)
        ],
    );
    // Hart 1 writes the same word, and nothing else.
    put(
        SIBLING,
        &[
            lui(11, BASE as u32),
            addi(11, 11, 0x400),
            addi(12, 0, 7),
            s(0x23, 3, 11, 12, 0), // sd a2, 0(a1)
        ],
    );

    let space = Arc::new(AddressSpace::new("mem", 64));
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), BASE)
        .unwrap();
    let a = Hart::new(cfg.with_reset_vector(BASE));
    let b = Hart::new(cfg.with_reset_vector(SIBLING));
    a.attach_space(Arc::clone(&space));
    b.attach_space(Arc::clone(&space));

    // Interleaved the way a scheduler would: the reservation is taken, the
    // quantum ends, the sibling runs to completion, and only then does the
    // store-conditional get its turn.
    for _ in 0..3 {
        a.step();
    }
    for _ in 0..4 {
        b.step();
    }
    for _ in 0..2 {
        a.step();
    }

    assert_eq!(
        a.x(13),
        1,
        "the store-conditional must fail: hart 1 wrote the reservation set"
    );
    assert_eq!(
        read_u64(&ram, WORD),
        7,
        "and hart 1's value must still be there"
    );
}

/// The reservation set is the naturally aligned **word**, not the cache line
/// it happens to sit on.
///
/// Volume I lets an implementation make the set larger, and the eventuality
/// guarantee is what that costs: a set the size of a line fails a constrained
/// LR/SC sequence every time unrelated traffic touches the line. So a sibling's
/// store to the *next* word must leave this reservation standing.
#[test]
fn a_sibling_store_to_the_next_word_leaves_the_reservation_alone() {
    let mut cfg = Config::rv64i();
    cfg.ext.a = true;
    let program = [
        lui(11, BASE as u32),
        addi(11, 11, 0x400),
        amo(0b00010, 3, 10, 11, 0), // lr.d a0, (a1)
        addi(12, 0, 99),
        amo(0b00011, 3, 13, 11, 12), // sc.d a3, a2, (a1)
    ];
    let h = Harness::with(cfg, &program);
    h.steps(3);
    // A second master on the same bus writes the doubleword above the
    // reservation — the neighbour, not the word itself.
    let space = h.hart.space().expect("attached");
    space
        .write(
            BASE + 0x408,
            crate::core::value::Width::U64,
            0x1234,
            crate::core::space::MemAttrs::DEFAULT,
        )
        .expect("the write lands");
    h.steps(2);
    assert_eq!(
        h.hart.x(13),
        0,
        "the store was outside the reservation set, so the sc.d must succeed"
    );
}

/// A trap gives up **both** halves of the reservation.
///
/// The local half is what an `SC` compares and the global half is what a
/// sibling's store clears; leaving the global one standing is not visible to
/// the guest, because the local half already fails the `SC` — but it leaves a
/// slot live in the space's table, which every store in the machine then walks
/// past. The two halves are one piece of state and they are dropped together.
#[test]
fn a_trap_drops_the_global_half_of_the_reservation_too() {
    let mut cfg = Config::rv64i();
    cfg.ext.a = true;
    let program = [
        lui(11, BASE as u32),
        addi(11, 11, 0x400),
        amo(0b00010, 3, 10, 11, 0), // lr.d a0, (a1)
        ECALL,
    ];
    let h = Harness::with(cfg, &program);
    h.steps(3);
    let space = h.hart.space().expect("attached");
    assert_eq!(
        space.monitor().outstanding(),
        1,
        "the lr.d published its reservation"
    );
    h.steps(1);
    assert_eq!(
        space.monitor().outstanding(),
        0,
        "and the trap took it back"
    );
}

/// The global monitor is keyed on the **physical** address, which is the only
/// key two harts with different page tables can collide on.
#[test]
fn the_reservation_is_watched_by_physical_address() {
    // Entry 0 identity-maps the first gigabyte so the code keeps running;
    // entry 1 walks three levels to a page at BASE+0x4000. The hart therefore
    // reserves virtual 0x4000_1000, whose physical address is BASE+0x4000 —
    // two numbers that are nothing like each other.
    let h = Harness::rv64gc(&[]);
    let v = super::mmu::pte::V;
    let x = super::mmu::pte::X;
    let rw = super::mmu::pte::R | super::mmu::pte::W;
    let ad = super::mmu::pte::A | super::mmu::pte::D;
    let root = BASE + 0x1000;
    h.put_u64(root, v | rw | x | ad);
    h.put_u64(root + 8, (((BASE + 0x2000) >> 12) << 10) | v);
    h.put_u64(BASE + 0x2000, (((BASE + 0x3000) >> 12) << 10) | v);
    h.put_u64(
        BASE + 0x3000 + 8,
        (((BASE + 0x4000) >> 12) << 10) | v | rw | ad,
    );

    // lr.d a0, (a1) then sc.d a3, a2, (a1), with a1 = 0x4000_1000.
    h.ram
        .write_at(0, &amo(0b00010, 3, 10, 11, 0).to_le_bytes())
        .unwrap();
    h.ram
        .write_at(4, &amo(0b00011, 3, 13, 11, 12).to_le_bytes())
        .unwrap();
    h.hart.set_x(11, 0x4000_1000);
    h.hart.set_x(12, 99);
    h.hart.set_pc(BASE);
    let mut csrs = h.hart.csrs();
    csrs.satp = (8 << 60) | (root >> 12);
    csrs.priv_mode = Priv::Supervisor;
    h.hart.set_csrs(csrs);

    h.steps(1); // the lr.d takes the reservation
    // Another master writes the same *physical* doubleword, naming it the way
    // it sees it. A monitor keyed on the virtual address would not notice.
    let space = h.hart.space().expect("attached");
    space
        .write(
            BASE + 0x4000,
            crate::core::value::Width::U64,
            7,
            crate::core::space::MemAttrs::DEFAULT,
        )
        .expect("the write lands");
    h.steps(1);
    assert_eq!(
        h.hart.x(13),
        1,
        "the sc.d must fail: the physical word it reserved was written"
    );
    assert_eq!(
        h.get_u64(BASE + 0x4000),
        7,
        "and the other master's value stands"
    );
}

/// Read a 64-bit value out of a `RamStore` at a guest address in [`BASE`]'s
/// region — [`Harness::get_u64`] for a test that builds its own memory.
fn read_u64(ram: &RamStore, addr: u64) -> u64 {
    let mut v = 0u64;
    for k in 0..8 {
        v |= u64::from(ram.read_u8(addr - BASE + k).unwrap()) << (8 * k);
    }
    v
}

#[test]
fn the_amo_family_returns_the_old_value_and_stores_the_new_one() {
    let mut cfg = Config::rv64i();
    cfg.ext.a = true;
    let program = [
        lui(11, BASE as u32),
        addi(11, 11, 0x400),
        addi(12, 0, 5),
        amo(0b00000, 3, 10, 11, 12), // amoadd.d
        amo(0b00001, 3, 13, 11, 12), // amoswap.d
        amo(0b10100, 3, 14, 11, 12), // amomax.d
    ];
    let h = Harness::with(cfg, &program);
    h.put_u64(BASE + 0x400, 7);
    h.steps(6);
    assert_eq!(h.hart.x(10), 7, "amoadd returns the old value");
    assert_eq!(h.hart.x(13), 12, "and the sum was stored");
    assert_eq!(h.hart.x(14), 5);
    assert_eq!(h.get_u64(BASE + 0x400), 5, "max(5, 5)");
}

#[test]
fn a_misaligned_atomic_traps() {
    let mut cfg = Config::rv64i();
    cfg.ext.a = true;
    let program = [
        lui(11, BASE as u32),
        addi(11, 11, 0x401),
        amo(0b00000, 3, 10, 11, 0),
    ];
    let h = Harness::with(cfg, &program);
    h.steps(3);
    assert_eq!(h.hart.csrs().mcause, cause::STORE_MISALIGNED);
}

// ---------------------------------------------------------------------------
// C
// ---------------------------------------------------------------------------

#[test]
fn compressed_instructions_execute_as_their_expansions() {
    let h = Harness::rv64gc(&[]);
    // c.li a0, 5 ; c.addi a0, 1 ; c.mv a1, a0
    h.put_half(0, 0x4515);
    h.put_half(2, 0x0505);
    h.put_half(4, 0x85aa);
    h.steps(3);
    assert_eq!(h.hart.x(10), 6);
    assert_eq!(h.hart.x(11), 6);
    assert_eq!(h.hart.pc(), BASE + 6, "each one advanced the PC by two");
}

#[test]
fn a_compressed_instruction_is_illegal_on_a_core_without_c() {
    let h = Harness::rv64i(&[]);
    h.put_half(0, 0x4515);
    h.steps(1);
    assert_eq!(h.hart.csrs().mcause, cause::ILLEGAL_INSN);
    assert_eq!(h.hart.csrs().mtval, 0x4515);
}

#[test]
fn the_all_zero_halfword_is_permanently_illegal() {
    let h = Harness::rv64gc(&[]);
    h.steps(1);
    assert_eq!(h.hart.csrs().mcause, cause::ILLEGAL_INSN);
}

// ---------------------------------------------------------------------------
// CSRs and traps
// ---------------------------------------------------------------------------

#[test]
fn csr_instructions_read_before_they_write() {
    let h = Harness::rv64i(&[
        addi(10, 0, 0x123),
        csrrw(11, num::MSCRATCH, 10), // a1 gets the old value (0)
        csrrs(12, num::MSCRATCH, 0),  // a2 gets 0x123
    ]);
    h.steps(3);
    assert_eq!(h.hart.x(11), 0);
    assert_eq!(h.hart.x(12), 0x123);
}

#[test]
fn a_write_to_a_read_only_csr_is_illegal() {
    let h = Harness::rv64i(&[addi(10, 0, 1), csrrw(0, num::MHARTID, 10)]);
    h.steps(2);
    assert_eq!(h.hart.csrs().mcause, cause::ILLEGAL_INSN);
    // ...but reading it is fine.
    let h = Harness::rv64i(&[csrrs(10, num::MHARTID, 0)]);
    h.steps(1);
    assert_eq!(h.hart.csrs().mcause, 0);
}

#[test]
fn a_machine_ecall_traps_to_mtvec_and_mret_comes_back() {
    // mtvec points at a handler that is one `mret`, 0x40 bytes in.
    let handler = BASE + 0x40;
    let mut program = vec![
        lui(10, BASE as u32),
        addi(10, 10, 0x40),
        csrrw(0, num::MTVEC, 10),
        ECALL,
        addi(11, 0, 7),
    ];
    program.resize(0x11, 0);
    program[0x10] = MRET;
    let h = Harness::rv64i(&program);
    h.steps(4);
    assert_eq!(h.hart.csrs().mcause, cause::ECALL_M);
    assert_eq!(h.hart.csrs().mepc, BASE + 12, "mepc is the ecall itself");
    assert_eq!(h.hart.pc(), handler);
    // A real handler advances mepc first; the point here is that the return
    // lands exactly where mepc says.
    h.steps(1);
    assert_eq!(h.hart.pc(), BASE + 12);
    assert_eq!(h.hart.priv_mode(), Priv::Machine);
}

#[test]
fn a_trap_saves_and_restores_the_interrupt_enable() {
    let h = Harness::rv64i(&[csrrs(0, num::MSTATUS, 10), ECALL]);
    h.hart.set_x(10, status::MIE);
    h.steps(2);
    let c = h.hart.csrs();
    assert_eq!(
        c.mstatus & status::MIE,
        0,
        "interrupts are off in the handler"
    );
    assert_ne!(c.mstatus & status::MPIE, 0, "and the old value was saved");
    assert_eq!(
        (c.mstatus & status::MPP) >> status::MPP_SHIFT,
        Priv::Machine.bits()
    );
}

#[test]
fn ebreak_reports_its_own_address() {
    let h = Harness::rv64i(&[EBREAK]);
    h.steps(1);
    assert_eq!(h.hart.csrs().mcause, cause::BREAKPOINT);
    assert_eq!(h.hart.csrs().mtval, BASE);
}

#[test]
fn an_exception_is_delegated_to_supervisor_mode_when_medeleg_says_so() {
    let h = Harness::rv64gc(&[
        // medeleg |= 1 << ECALL_U ; stvec = BASE + 0x200
        addi(10, 0, 1),
        i(0x13, 1, 10, 10, cause::ECALL_U as i32), // slli a0, a0, 8
        csrrs(0, num::MEDELEG, 10),
        lui(11, BASE as u32),
        addi(11, 11, 0x200),
        csrrw(0, num::STVEC, 11),
        // mstatus.MPP = User, mepc = the ecall below, then mret into it.
        csrrw(0, num::MSTATUS, 0),
        lui(12, BASE as u32),
        addi(12, 12, 0x40),
        csrrw(0, num::MEPC, 12),
        MRET,
    ]);
    // The user-mode instruction at 0x40 is an ecall.
    h.ram.write_at(0x40, &ECALL.to_le_bytes()).unwrap();
    h.steps(11);
    assert_eq!(h.hart.priv_mode(), Priv::User, "mret dropped to user mode");
    h.steps(1);
    let c = h.hart.csrs();
    assert_eq!(c.priv_mode, Priv::Supervisor, "the trap was delegated");
    assert_eq!(c.scause, cause::ECALL_U);
    assert_eq!(c.sepc, BASE + 0x40);
    assert_eq!(c.mcause, 0, "and the machine registers were not touched");
    assert_eq!(h.hart.pc(), BASE + 0x200);
}

#[test]
fn sret_returns_to_the_privilege_spp_names() {
    let h = Harness::rv64gc(&[
        // sepc = BASE + 0x100, mstatus.SPP = 0 (user)
        lui(10, BASE as u32),
        addi(10, 10, 0x100),
        csrrw(0, num::SEPC, 10),
        SRET,
    ]);
    h.steps(4);
    assert_eq!(h.hart.pc(), BASE + 0x100);
    assert_eq!(h.hart.priv_mode(), Priv::User);
}

#[test]
fn sret_from_supervisor_traps_when_tsr_is_set() {
    let h = Harness::rv64gc(&[
        addi(10, 0, 1),
        i(0x13, 1, 10, 10, 22), // slli a0, a0, 22 -> TSR
        csrrs(0, num::MSTATUS, 10),
        // Enter supervisor mode: MPP = 1.
        addi(11, 0, 1),
        i(0x13, 1, 11, 11, status::MPP_SHIFT as i32),
        csrrs(0, num::MSTATUS, 11),
        lui(12, BASE as u32),
        addi(12, 12, 0x40),
        csrrw(0, num::MEPC, 12),
        MRET,
    ]);
    h.ram.write_at(0x40, &SRET.to_le_bytes()).unwrap();
    h.steps(10);
    assert_eq!(h.hart.priv_mode(), Priv::Supervisor);
    h.steps(1);
    assert_eq!(h.hart.csrs().mcause, cause::ILLEGAL_INSN);
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

#[test]
fn an_enabled_machine_interrupt_is_taken_between_instructions() {
    let h = Harness::rv64i(&[
        addi(10, 0, (irq::MTI) as i32),
        csrrs(0, num::MIE, 10),
        addi(11, 0, status::MIE as i32),
        csrrs(0, num::MSTATUS, 11),
        addi(12, 0, 1),
    ]);
    h.steps(4);
    h.hart.set_interrupt(irq::MTI, true);
    h.steps(1);
    let c = h.hart.csrs();
    assert_ne!(c.mcause >> 63, 0, "the interrupt bit is set");
    assert_eq!(c.mcause & 0xff, cause::IRQ_M_TIMER);
    assert_eq!(c.mepc, BASE + 16, "the interrupted instruction has not run");
    assert_eq!(h.hart.x(12), 0);
}

#[test]
fn a_disabled_interrupt_is_not_taken() {
    let h = Harness::rv64i(&[addi(12, 0, 1)]);
    h.hart.set_interrupt(irq::MTI, true);
    h.steps(1);
    assert_eq!(h.hart.x(12), 1, "mie is clear, so nothing happened");
}

#[test]
fn the_priority_order_is_the_specifications_not_the_numeric_one() {
    let h = Harness::rv64i(&[
        addi(10, 0, -1),
        csrrs(0, num::MIE, 10),
        addi(11, 0, status::MIE as i32),
        csrrs(0, num::MSTATUS, 11),
        addi(12, 0, 1),
    ]);
    h.steps(4);
    // The timer is cause 7 and the external is 11, but external wins.
    h.hart.set_interrupt(irq::MTI | irq::MEI, true);
    h.steps(1);
    assert_eq!(h.hart.csrs().mcause & 0xff, cause::IRQ_M_EXT);
}

#[test]
fn wfi_stalls_until_an_interrupt_arrives() {
    let h = Harness::rv64i(&[
        addi(10, 0, irq::MTI as i32),
        csrrs(0, num::MIE, 10),
        WFI,
        addi(12, 0, 1),
    ]);
    h.steps(3);
    assert!(h.hart.is_waiting());
    let before = h.hart.instret();
    h.steps(5);
    assert_eq!(h.hart.instret(), before, "nothing retired while stalled");
    assert!(h.hart.cycles() > 0, "but time still passed");
    h.hart.set_interrupt(irq::MTI, true);
    h.steps(1);
    assert!(!h.hart.is_waiting());
}

// ---------------------------------------------------------------------------
// Paging
// ---------------------------------------------------------------------------

#[test]
fn an_sv39_walk_translates_a_supervisor_access_end_to_end() {
    // Root table at BASE+0x1000. Entry 0 is a 1 GiB leaf identity-mapping the
    // first gigabyte, so the code the hart fetches stays where it is; entry 1
    // is a pointer, so the data access below takes a real three-level walk.
    let h = Harness::rv64gc(&[]);
    let v = super::mmu::pte::V;
    let x = super::mmu::pte::X;
    let rw = super::mmu::pte::R | super::mmu::pte::W;
    let ad = super::mmu::pte::A | super::mmu::pte::D;
    let root = BASE + 0x1000;
    h.put_u64(root, v | rw | x | ad); // PPN 0: identity for 0..1 GiB
    h.put_u64(root + 8, (((BASE + 0x2000) >> 12) << 10) | v);
    h.put_u64(BASE + 0x2000, (((BASE + 0x3000) >> 12) << 10) | v);
    h.put_u64(
        BASE + 0x3000 + 8,
        (((BASE + 0x4000) >> 12) << 10) | v | rw | ad,
    );
    h.put_u64(BASE + 0x4000, 0xdead_beef);

    // ld a0, 0(a1) with a1 = 0x4000_1000, a virtual address in entry 1's
    // gigabyte.
    h.ram
        .write_at(0, &i(0x03, 3, 10, 11, 0).to_le_bytes())
        .unwrap();
    h.hart.set_x(11, 0x4000_1000);

    // Machine mode is untranslated, so the same instruction reads nothing.
    h.steps(1);
    assert_eq!(h.hart.x(10), 0);

    h.hart.set_pc(BASE);
    let mut csrs = h.hart.csrs();
    csrs.satp = (8 << 60) | (root >> 12);
    csrs.priv_mode = Priv::Supervisor;
    h.hart.set_csrs(csrs);
    h.steps(1);
    assert_eq!(h.hart.x(10), 0xdead_beef);
    // The walk set the accessed bit on the leaf on its way through.
    assert_ne!(h.get_u64(BASE + 0x3000 + 8) & super::mmu::pte::A, 0);
}

#[test]
fn a_supervisor_access_to_an_unmapped_page_raises_a_page_fault() {
    let h = Harness::rv64gc(&[i(0x03, 3, 10, 0, 0x40)]);
    let mut csrs = h.hart.csrs();
    // A root table full of zeroes: nothing is valid.
    csrs.satp = (8 << 60) | ((BASE + 0x8000) >> 12);
    csrs.priv_mode = Priv::Supervisor;
    csrs.stvec = BASE + 0x300;
    csrs.medeleg = 1 << cause::INSN_PAGE_FAULT;
    h.hart.set_csrs(csrs);
    h.steps(1);
    let c = h.hart.csrs();
    assert_eq!(c.scause, cause::INSN_PAGE_FAULT, "the fetch itself faults");
    assert_eq!(c.stval, BASE);
}

#[test]
fn a_debug_listing_follows_the_page_table_and_a_physical_one_does_not() {
    // The defect, stated as a test — and it predates the ARM MMU: this core has
    // had a real Sv39 walker all along and its listing never used it. Entry 0
    // identity-maps the first gigabyte so the code stays fetchable; entry 1
    // points at a three-level walk that lands a 4 KiB page holding one
    // recognisable instruction at virtual 0x4000_1000. Nothing at all is mapped
    // at *physical* 0x4000_1000, so a listing that skipped translation cannot
    // produce a plausible-looking wrong answer — it produces a hole.
    let h = Harness::rv64gc(&[]);
    let v = super::mmu::pte::V;
    let x = super::mmu::pte::X;
    let rw = super::mmu::pte::R | super::mmu::pte::W;
    let ad = super::mmu::pte::A | super::mmu::pte::D;
    let root = BASE + 0x1000;
    h.put_u64(root, v | rw | x | ad);
    h.put_u64(root + 8, (((BASE + 0x2000) >> 12) << 10) | v);
    h.put_u64(BASE + 0x2000, (((BASE + 0x3000) >> 12) << 10) | v);
    h.put_u64(
        BASE + 0x3000 + 8,
        (((BASE + 0x4000) >> 12) << 10) | v | rw | x | ad,
    );
    // `addi a0, a0, 1` at the start of the mapped page.
    h.ram
        .write_at(0x4000, &addi(10, 10, 1).to_le_bytes())
        .unwrap();

    let mut csrs = h.hart.csrs();
    csrs.satp = (8 << 60) | (root >> 12);
    csrs.priv_mode = Priv::Supervisor;
    h.hart.set_csrs(csrs);

    assert_eq!(h.hart.translate_debug(0x4000_1000), Some(BASE + 0x4000));

    let listing = h.hart.disassemble_virtual(0x4000_1000, 1);
    assert_eq!(listing[0].hole, None);
    assert_eq!(listing[0].text, "addi a0, a0, 1");

    // The same number read physically is nowhere, and says so rather than
    // quietly following the table.
    let listing = h.hart.disassemble_physical(0x4000_1000, 1);
    assert_eq!(
        listing[0].hole,
        Some(super::disasm::Missing::Unmapped),
        "a physical listing must not follow the page table"
    );
}

#[test]
fn a_debug_listing_that_runs_off_a_mapped_page_keeps_its_count() {
    // Only one 4 KiB page is mapped in that gigabyte, so a listing near its end
    // decodes what it can and then reports holes — four entries for four asked,
    // each naming why, rather than a short answer with no explanation.
    let h = Harness::rv64gc(&[]);
    let v = super::mmu::pte::V;
    let x = super::mmu::pte::X;
    let rw = super::mmu::pte::R | super::mmu::pte::W;
    let ad = super::mmu::pte::A | super::mmu::pte::D;
    let root = BASE + 0x1000;
    h.put_u64(root, v | rw | x | ad);
    h.put_u64(root + 8, (((BASE + 0x2000) >> 12) << 10) | v);
    h.put_u64(BASE + 0x2000, (((BASE + 0x3000) >> 12) << 10) | v);
    h.put_u64(
        BASE + 0x3000 + 8,
        (((BASE + 0x4000) >> 12) << 10) | v | rw | x | ad,
    );
    h.ram
        .write_at(0x4ffc, &addi(10, 10, 1).to_le_bytes())
        .unwrap();

    let mut csrs = h.hart.csrs();
    csrs.satp = (8 << 60) | (root >> 12);
    csrs.priv_mode = Priv::Supervisor;
    h.hart.set_csrs(csrs);

    let listing = h.hart.disassemble_virtual(0x4000_1ffc, 4);
    assert_eq!(listing.len(), 4, "a hole must not shorten the listing");
    assert_eq!(listing[0].hole, None);
    for one in &listing[1..] {
        assert_eq!(one.hole, Some(super::disasm::Missing::Untranslated));
    }
    assert_eq!(listing[1].addr, 0x4000_2000);
}

#[test]
fn a_debug_listing_sets_no_accessed_bit() {
    // The no-side-effects property where this architecture actually has one:
    // an Sv39 walk sets `A` on the leaf, and a debugger's must not. The
    // fixture's leaf starts with `A` and `D` clear, and executing through it
    // would set them — which is what the walk test above asserts.
    let h = Harness::rv64gc(&[]);
    let v = super::mmu::pte::V;
    let x = super::mmu::pte::X;
    let rw = super::mmu::pte::R | super::mmu::pte::W;
    let root = BASE + 0x1000;
    h.put_u64(root, v | rw | x | super::mmu::pte::A | super::mmu::pte::D);
    h.put_u64(root + 8, (((BASE + 0x2000) >> 12) << 10) | v);
    h.put_u64(BASE + 0x2000, (((BASE + 0x3000) >> 12) << 10) | v);
    let leaf = (((BASE + 0x4000) >> 12) << 10) | v | rw | x;
    h.put_u64(BASE + 0x3000 + 8, leaf);

    let mut csrs = h.hart.csrs();
    csrs.satp = (8 << 60) | (root >> 12);
    csrs.priv_mode = Priv::Supervisor;
    h.hart.set_csrs(csrs);

    let cycles = h.hart.cycles();
    let tlb = h.hart.tlb_stats();
    assert_eq!(h.hart.translate_debug(0x4000_1000), Some(BASE + 0x4000));
    assert_eq!(
        h.get_u64(BASE + 0x3000 + 8),
        leaf,
        "a debug walk set the accessed bit"
    );
    assert_eq!(h.hart.cycles(), cycles, "a debug walk charged a cycle");
    assert_eq!(h.hart.tlb_stats(), tlb, "a debug walk touched the TLB");
}

#[test]
fn the_tlb_absorbs_repeated_translations() {
    let h = Harness::rv64gc(&[]);
    let v = super::mmu::pte::V;
    let perms = super::mmu::pte::R
        | super::mmu::pte::W
        | super::mmu::pte::X
        | super::mmu::pte::A
        | super::mmu::pte::D;
    // One 1 GiB leaf identity-mapping the first gigabyte, which is where the
    // code is. A superpage's page number must be aligned to its own size, so
    // this has to be entry 0 with PPN 0 rather than one covering only RAM.
    h.put_u64(BASE + 0x8000, v | perms);
    for n in 0..8u64 {
        h.ram
            .write_at(n * 4, &addi(10, 10, 1).to_le_bytes())
            .unwrap();
    }
    let mut csrs = h.hart.csrs();
    csrs.satp = (8 << 60) | ((BASE + 0x8000) >> 12);
    csrs.priv_mode = Priv::Supervisor;
    h.hart.set_csrs(csrs);
    h.steps(8);
    assert_eq!(h.hart.x(10), 8);
    let (hits, misses) = h.hart.tlb_stats();
    assert!(hits > misses, "{hits} hits against {misses} misses");
}

// ---------------------------------------------------------------------------
// Floating point
// ---------------------------------------------------------------------------

/// Turn the floating-point unit on, which every FP program must do first.
fn enable_fp(hart: &Hart) {
    let mut csrs = hart.csrs();
    csrs.mstatus |= 1 << status::FS_SHIFT;
    hart.set_csrs(csrs);
}

#[test]
fn floating_point_is_illegal_until_mstatus_says_otherwise() {
    let h = Harness::rv64gc(&[r(0x53, 7, 0, 10, 11, 12)]); // fadd.s
    h.steps(1);
    assert_eq!(h.hart.csrs().mcause, cause::ILLEGAL_INSN);

    let h = Harness::rv64gc(&[r(0x53, 7, 0, 10, 11, 12)]);
    enable_fp(&h.hart);
    h.steps(1);
    assert_eq!(h.hart.csrs().mcause, 0);
}

#[test]
fn single_precision_values_are_nan_boxed() {
    // fmv.w.x fa0, a0 must produce a boxed value; fmv.x.w reads it back.
    let h = Harness::rv64gc(&[
        r(0x53, 0, 0x78, 10, 11, 0), // fmv.w.x fa0, a1
        r(0x53, 0, 0x70, 12, 10, 0), // fmv.x.w a2, fa0
    ]);
    enable_fp(&h.hart);
    h.hart.set_x(11, 0x4000_0000); // 2.0f
    h.steps(2);
    assert_eq!(h.hart.f(10) >> 32, 0xffff_ffff, "the box is all ones");
    assert_eq!(h.hart.x(12), 0x4000_0000);

    // An unboxed value read as a single is the canonical NaN, not the bits.
    let h = Harness::rv64gc(&[r(0x53, 1, 0x70, 12, 10, 0)]); // fclass.s a2, fa0
    enable_fp(&h.hart);
    h.hart.set_f(10, 0x0000_0000_4000_0000); // not boxed
    h.steps(1);
    assert_eq!(h.hart.x(12), 1 << 9, "a quiet NaN, not a normal number");
}

#[test]
fn arithmetic_accumulates_the_sticky_flags() {
    // 1.0 / 3.0 is inexact; the flag survives into fcsr and stays there.
    let h = Harness::rv64gc(&[
        r(0x53, 7, 0x0d, 10, 11, 12), // fdiv.d fa0, fa1, fa2
        r(0x53, 7, 0x01, 13, 11, 12), // fadd.d fa3, fa1, fa2 (exact)
    ]);
    enable_fp(&h.hart);
    h.hart.set_f(11, 1.0f64.to_bits());
    h.hart.set_f(12, 3.0f64.to_bits());
    h.steps(2);
    assert_eq!(h.hart.f(10), (1.0f64 / 3.0).to_bits());
    assert_eq!(
        h.hart.csrs().fcsr & 0x1f,
        u64::from(crate::float::Flags::INEXACT.to_fcsr())
    );
    assert_eq!(h.hart.f(13), 4.0f64.to_bits());
    assert_eq!(
        h.hart.csrs().fcsr & 0x1f,
        u64::from(crate::float::Flags::INEXACT.to_fcsr()),
        "an exact operation does not clear the sticky flag"
    );
}

#[test]
fn the_rounding_mode_comes_from_fcsr_when_the_instruction_is_dynamic() {
    // fadd.d with rm = 7 uses frm; set frm to round-towards-zero and add
    // 1.0 + 2^-53, which rounds down rather than to even.
    let h = Harness::rv64gc(&[
        csrrw(0, num::FRM, 10),
        r(0x53, 7, 0x01, 12, 11, 13), // fadd.d fa2, fa1, fa3
    ]);
    enable_fp(&h.hart);
    h.hart.set_x(10, 1); // RTZ
    h.hart.set_f(11, 1.0f64.to_bits());
    h.hart.set_f(13, 0x3ca0_0000_0000_0001); // slightly over 2^-53
    h.steps(2);
    assert_eq!(h.hart.f(12), 1.0f64.to_bits());
}

#[test]
fn a_reserved_rounding_mode_is_an_illegal_instruction() {
    let h = Harness::rv64gc(&[r(0x53, 5, 0x01, 12, 11, 13)]);
    enable_fp(&h.hart);
    h.steps(1);
    assert_eq!(h.hart.csrs().mcause, cause::ILLEGAL_INSN);
}

#[test]
fn a_float_load_and_store_round_trip_through_memory() {
    let h = Harness::rv64gc(&[
        lui(11, BASE as u32),
        s(0x27, 3, 11, 10, 0x400), // fsd fa0, 0x400(a1)
        i(0x07, 3, 12, 11, 0x400), // fld fa2, 0x400(a1)
        i(0x07, 2, 13, 11, 0x400), // flw fa3, 0x400(a1)
    ]);
    enable_fp(&h.hart);
    h.hart.set_f(10, 0x0123_4567_89ab_cdef);
    h.steps(4);
    assert_eq!(h.hart.f(12), 0x0123_4567_89ab_cdef);
    assert_eq!(h.hart.f(13), 0xffff_ffff_89ab_cdef, "flw NaN-boxes");
}

#[test]
fn fused_multiply_add_writes_the_fused_result() {
    // fmadd.d fa0, fa1, fa2, fa3 with rm = dynamic.
    let word = 0x43 | (10 << 7) | (7 << 12) | (11 << 15) | (12 << 20) | (13 << 27) | (1 << 25);
    let h = Harness::rv64gc(&[word]);
    enable_fp(&h.hart);
    h.hart.set_f(11, (1.0f64 + f64::from_bits(0)).to_bits() + 1);
    h.hart.set_f(12, 1.0f64.to_bits() - 2);
    h.hart.set_f(13, (-1.0f64).to_bits());
    h.steps(1);
    // (1 + 2^-52)(1 - 2^-52) - 1 is exactly -2^-104.
    assert_eq!(h.hart.f(10), (1u64 << 63) | (919u64 << 52));
}

// ---------------------------------------------------------------------------
// Device plumbing
// ---------------------------------------------------------------------------

#[test]
fn save_and_load_round_trip_to_an_identical_state() -> Result<()> {
    let h = Harness::rv64gc(&[addi(10, 0, 1), addi(11, 0, 2), ECALL]);
    enable_fp(&h.hart);
    h.hart.set_f(3, 0x1234);
    h.hart.set_interrupt(irq::MTI, true);
    h.steps(3);

    let mut shape = MachineShape::new();
    shape.add_device("hart", CLASS.name)?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("hart", CLASS.name, CLASS.version)?;
        h.hart.save(&mut chunk)?;
    }
    let bytes = w.to_vec()?;

    let restored = Hart::new(h.hart.config());
    let reader = StateReader::new(&bytes)?;
    let chunk = reader.load("hart", CLASS.name, CLASS.version, &Migrations::new())?;
    let mut cr = chunk.reader();
    restored.load(&mut cr)?;
    cr.end()?;

    assert_eq!(restored.pc(), h.hart.pc());
    assert_eq!(restored.x(10), h.hart.x(10));
    assert_eq!(restored.f(3), h.hart.f(3));
    assert_eq!(restored.csrs().mcause, h.hart.csrs().mcause);
    assert_eq!(restored.interrupts(), h.hart.interrupts());

    let mut shape2 = MachineShape::new();
    shape2.add_device("hart", CLASS.name)?;
    let mut w2 = StateWriter::new(shape2);
    {
        let mut chunk = w2.chunk("hart", CLASS.name, CLASS.version)?;
        restored.save(&mut chunk)?;
    }
    assert_eq!(w2.to_vec()?, bytes, "a round trip must be a fixed point");
    Ok(())
}

#[test]
fn a_reset_returns_the_hart_to_its_reset_vector() {
    let h = Harness::rv64i(&[addi(10, 0, 1), addi(10, 0, 2)]);
    h.steps(2);
    assert_ne!(h.hart.pc(), BASE);
    h.hart.reset(ResetKind::Cold);
    assert_eq!(h.hart.pc(), BASE);
    assert_eq!(h.hart.x(10), 0);
    assert_eq!(h.hart.priv_mode(), Priv::Machine);
}

#[test]
fn a_reset_request_is_latched_and_acted_on_at_the_next_step() {
    let h = Harness::rv64i(&[addi(10, 0, 1), addi(10, 0, 2)]);
    h.steps(2);
    h.hart.request_reset();
    assert_ne!(h.hart.pc(), BASE, "nothing happened yet");
    h.steps(1);
    assert_eq!(h.hart.pc(), BASE + 4, "the reset ran, then one instruction");
    assert_eq!(h.hart.x(10), 1);
}

#[test]
fn a_budget_carries_its_overshoot_into_the_next_one() {
    let h = Harness::rv64i(&[addi(10, 0, 1); 16]);
    // Each instruction charges two accesses (two halfword fetches), so a
    // one-access budget always overshoots by one.
    let mut total = 0;
    for _ in 0..8 {
        total += h.hart.run_budget(1);
    }
    assert_eq!(total, 8, "the scheduler is never told more than it granted");
    assert_eq!(h.hart.cycles(), 8);
}

#[test]
fn construction_from_properties_validates_what_it_is_given() {
    let hart = Hart::from_props(&Props::new().with("xlen", "rv32")).unwrap();
    assert_eq!(hart.config().xlen, Xlen::Rv32);

    let hart = Hart::from_props(&Props::new().with("isa", "imac")).unwrap();
    assert!(hart.config().ext.m && hart.config().ext.a && hart.config().ext.c);
    assert!(!hart.config().ext.f);

    let hart = Hart::from_props(&Props::new().with("isa", "g")).unwrap();
    assert_eq!(hart.config().isa_string(), "rv64imafd");

    // An unknown extension letter is an error rather than a shrug.
    assert!(Hart::from_props(&Props::new().with("isa", "v")).is_err());
    // As is a property nothing accepts.
    assert!(Hart::from_props(&Props::new().with("nonsense", 1u64)).is_err());
    // And an out-of-range PMP count.
    assert!(Hart::from_props(&Props::new().with("pmp", 99u64)).is_err());
}

#[test]
fn d_without_f_is_corrected_rather_than_honoured() {
    let hart = Hart::new(Config::rv64i().with_ext(Extensions {
        d: true,
        ..Extensions::I
    }));
    assert!(hart.config().ext.f, "D implies F");
}

#[test]
fn register_names_round_trip() {
    assert_eq!(x_by_name("a0"), Some(10));
    assert_eq!(x_by_name("x31"), Some(31));
    assert_eq!(x_by_name("fp"), Some(8));
    assert_eq!(x_by_name("zero"), Some(0));
    assert_eq!(x_by_name("nonsense"), None);
    assert_eq!(x_by_name("x32"), None);
    for (n, name) in X_NAMES.iter().enumerate() {
        assert_eq!(x_by_name(name), Some(n as u32));
    }
}

#[test]
fn the_isa_description_covers_the_whole_table() {
    let text = super::describe_isa();
    let lines: Vec<&str> = text.lines().collect();
    assert_eq!(
        lines.len(),
        super::isa::TABLE.len() + super::isa::CTABLE.len()
    );
    assert!(text.contains("fmadd.d"));
    assert!(text.contains("c.addi4spn"));
}

#[test]
fn a_hart_with_no_address_space_reports_no_progress() {
    let hart = Hart::new(Config::rv64i());
    assert_eq!(
        hart.step(),
        0,
        "a caller must treat this as stop, not retry"
    );
}

/// `csrr a0, time` — CSRRS with `rs1 = x0`, so it reads without writing.
const CSRR_A0_TIME: u32 = 0xc010_2573;

#[test]
fn the_time_csr_follows_an_attached_platform_timer() {
    let h = Harness::rv64i(&[CSRR_A0_TIME, CSRR_A0_TIME]);
    let timer = Arc::new(AtomicU64::new(0x1234_5678));
    h.hart.attach_time(Arc::clone(&timer));

    h.hart.step();
    assert_eq!(
        h.hart.x(10),
        0x1234_5678,
        "`time` must report the platform timer, not the hart's own field"
    );

    // The whole point: it keeps up. A guest that read a frozen `time` would
    // compute every deadline as already past and live-lock on its own timer.
    timer.store(0x1234_9999, Ordering::Relaxed);
    h.hart.step();
    assert_eq!(h.hart.x(10), 0x1234_9999);
}

#[test]
fn an_attached_timer_survives_reset() {
    // Wiring, not guest state. A reset re-runs the reset sequence; it does not
    // unplug the CLINT, so `time` must still track it afterwards.
    let h = Harness::rv64i(&[CSRR_A0_TIME]);
    let timer = Arc::new(AtomicU64::new(7));
    h.hart.attach_time(Arc::clone(&timer));

    h.hart.reset(ResetKind::Cold);
    timer.store(99, Ordering::Relaxed);
    h.hart.step();
    assert_eq!(h.hart.x(10), 99, "reset must not detach the platform timer");
}

#[test]
fn set_time_still_works_without_a_clint() {
    // A machine with no CLINT has no cell to attach, and `set_time` remains
    // how it supplies the value.
    let h = Harness::rv64i(&[CSRR_A0_TIME]);
    h.hart.set_time(0x4242);
    h.hart.step();
    assert_eq!(h.hart.x(10), 0x4242);
}

// ---------------------------------------------------------------------------
// The syscall-exit seam (`core::exec`, ROADMAP.md §2.1)
// ---------------------------------------------------------------------------

#[test]
fn an_unmasked_hart_vectors_its_traps_exactly_as_before() {
    // The seam is opt-in and the default is the behaviour every machine in
    // `machines/` depends on. This is the regression that says so.
    use crate::core::exec::{ExitMask, ExitingCore};

    let h = Harness::rv64i(&[ECALL]);
    assert_eq!(h.hart.exit_mask(), ExitMask::NONE);
    // One step: the `ecall` executes and takes its trap. Running further would
    // vector into unmapped memory and overwrite `mcause` with the fault that
    // followed, which says nothing about the seam.
    let (_, exit) = h.hart.step_to_exit();
    assert!(exit.is_none());
    assert_eq!(h.hart.csrs().mcause, cause::ECALL_M);
}

#[test]
fn a_masked_ecall_leaves_the_hart_instead_of_vectoring() {
    use crate::core::exec::{Access, ExitMask, ExitReason, ExitingCore};

    let h = Harness::rv64i(&[addi(10, 0, 7), ECALL, addi(10, 0, 9)]);
    h.hart.set_exit_mask(ExitMask::USER);
    let run = h.hart.run_to_exit_ticks(64);
    let exit = run.exit.expect("the ecall exits");

    assert_eq!(exit.reason, ExitReason::SYSCALL);
    assert_eq!(exit.pc, BASE + 4, "the exit names the ecall itself");
    assert_eq!(exit.len, 4);
    assert_eq!(exit.detail, cause::ECALL_M);
    assert_eq!(exit.access, Access::None);
    assert_eq!(h.hart.x(10), 7, "the instruction before it ran");
    assert_eq!(
        h.hart.pc(),
        BASE + 8,
        "a syscall resumes past the instruction"
    );
    assert_eq!(
        h.hart.csrs().mcause,
        0,
        "nothing vectored, so no cause was written"
    );

    // Resuming continues the program, with no fixup at all.
    h.hart.run_to_exit_ticks(16);
    assert_eq!(h.hart.x(10), 9);
}

#[test]
fn a_masked_fault_reports_its_address_and_its_direction() {
    use crate::core::exec::{Access, ExitMask, ExitReason, ExitingCore};

    // `sd x0, 0(x0)` — a store to an address this harness has no RAM at.
    let sd = s(0x23, 3, 0, 0, 0);
    let h = Harness::rv64i(&[sd]);
    h.hart.set_exit_mask(ExitMask::USER);
    let exit = h
        .hart
        .run_to_exit_ticks(64)
        .exit
        .expect("the store faults out");

    assert_eq!(exit.reason, ExitReason::FAULT);
    assert_eq!(exit.access, Access::Write, "a consumer needs the direction");
    assert_eq!(exit.address, 0);
    assert_eq!(
        h.hart.pc(),
        BASE,
        "a fault resumes *at* the instruction, so mapping a page and \
         carrying on works"
    );
}

#[test]
fn a_masked_ebreak_stops_on_the_breakpoint() {
    use crate::core::exec::{ExitMask, ExitReason, ExitingCore};

    let h = Harness::rv64i(&[EBREAK]);
    h.hart
        .set_exit_mask(ExitMask::NONE.with(ExitReason::BREAKPOINT));
    let exit = h.hart.run_to_exit_ticks(16).exit.expect("ebreak exits");
    assert_eq!(exit.reason, ExitReason::BREAKPOINT);
    assert_eq!(exit.pc, BASE);
    assert_eq!(h.hart.pc(), BASE, "a debugger reports where it stopped");
}

#[test]
fn the_exit_mask_survives_a_reset_because_it_is_not_guest_state() {
    use crate::core::exec::{ExitMask, ExitingCore};

    let h = Harness::rv64i(&[ECALL]);
    h.hart.set_exit_mask(ExitMask::USER);
    h.hart.reset(ResetKind::Cold);
    assert_eq!(
        h.hart.exit_mask(),
        ExitMask::USER,
        "the consumer that armed the mask is still there after a reset"
    );
}

#[test]
fn the_stack_pointer_is_on_the_seam_because_starting_a_thread_needs_it() {
    use crate::core::exec::ExitingCore;

    let h = Harness::rv64i(&[]);
    ExitingCore::set_sp(&h.hart, 0x1234);
    assert_eq!(ExitingCore::sp(&h.hart), 0x1234);
    assert_eq!(h.hart.x(2), 0x1234, "which is `sp` and nothing else");
}
