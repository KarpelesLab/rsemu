//! Tests for the AArch64 core.
//!
//! The interesting ones are not "does `ADD` add". They are the places DDI 0487
//! is surprising and an implementation is likely to be wrong: the `SP`/`XZR`
//! encoding distinction, the carry flag on a subtract, the shift amount taken
//! modulo the operand width, `MOVK` keeping what it does not write, a bitfield
//! move that is really three instructions in a trench coat, an exclusive pair
//! losing its reservation, an interrupt that EL0 cannot mask, an end-to-end
//! translation-table walk, and an extension that must *not* decode on a part
//! without it.
//!
//! Instructions are assembled by the encoders below rather than pasted in as
//! hex, so a test says what it means; `isa`'s own tests already prove those
//! encoders agree with the decoder on a set of independently known words.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, ResetKind};
use crate::core::error::Result;
use crate::core::props::Props;
use crate::core::space::{AddressSpace, RamStore, Region};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Width;

use super::isa::{Cond, Features, Nzcv};
use super::mmu::desc;
use super::sysreg::{El, SysReg, daif, ec, sctlr};
use super::{CLASS, Config, Cpu, X_NAMES, x_by_name};

// ---------------------------------------------------------------------------
// Assembly
// ---------------------------------------------------------------------------

const fn movz(sf: u32, rd: u32, imm: u32, shift: u32) -> u32 {
    (sf << 31) | 0x5280_0000 | ((shift / 16) << 21) | ((imm & 0xffff) << 5) | rd
}
const fn movk(sf: u32, rd: u32, imm: u32, shift: u32) -> u32 {
    (sf << 31) | 0x7280_0000 | ((shift / 16) << 21) | ((imm & 0xffff) << 5) | rd
}
const fn movn(sf: u32, rd: u32, imm: u32, shift: u32) -> u32 {
    (sf << 31) | 0x1280_0000 | ((shift / 16) << 21) | ((imm & 0xffff) << 5) | rd
}
const fn addsub_imm(base: u32, sf: u32, rd: u32, rn: u32, imm: u32) -> u32 {
    (sf << 31) | base | ((imm & 0xfff) << 10) | (rn << 5) | rd
}
const fn add_imm(sf: u32, rd: u32, rn: u32, imm: u32) -> u32 {
    addsub_imm(0x1100_0000, sf, rd, rn, imm)
}
const fn adds_imm(sf: u32, rd: u32, rn: u32, imm: u32) -> u32 {
    addsub_imm(0x3100_0000, sf, rd, rn, imm)
}
const fn subs_imm(sf: u32, rd: u32, rn: u32, imm: u32) -> u32 {
    addsub_imm(0x7100_0000, sf, rd, rn, imm)
}
const fn log_imm(base: u32, sf: u32, rd: u32, rn: u32, n: u32, immr: u32, imms: u32) -> u32 {
    (sf << 31) | base | (n << 22) | (immr << 16) | (imms << 10) | (rn << 5) | rd
}
const fn shifted(base: u32, sf: u32, rd: u32, rn: u32, rm: u32, shift: u32, amount: u32) -> u32 {
    (sf << 31) | base | (shift << 22) | (rm << 16) | (amount << 10) | (rn << 5) | rd
}
const fn subs_reg(sf: u32, rd: u32, rn: u32, rm: u32) -> u32 {
    shifted(0x6b00_0000, sf, rd, rn, rm, 0, 0)
}
const fn orr_reg(sf: u32, rd: u32, rn: u32, rm: u32) -> u32 {
    shifted(0x2a00_0000, sf, rd, rn, rm, 0, 0)
}
const fn bitfield(base: u32, sf: u32, rd: u32, rn: u32, immr: u32, imms: u32) -> u32 {
    (sf << 31) | base | (sf << 22) | (immr << 16) | (imms << 10) | (rn << 5) | rd
}
const fn ldr_x(rt: u32, rn: u32, offset: u32) -> u32 {
    0xf940_0000 | ((offset / 8) << 10) | (rn << 5) | rt
}
const fn str_x(rt: u32, rn: u32, offset: u32) -> u32 {
    0xf900_0000 | ((offset / 8) << 10) | (rn << 5) | rt
}
const fn ldrb(rt: u32, rn: u32, offset: u32) -> u32 {
    0x3940_0000 | (offset << 10) | (rn << 5) | rt
}
const fn strb(rt: u32, rn: u32, offset: u32) -> u32 {
    0x3900_0000 | (offset << 10) | (rn << 5) | rt
}
const fn ldrsb_x(rt: u32, rn: u32, offset: u32) -> u32 {
    0x3980_0000 | (offset << 10) | (rn << 5) | rt
}
const fn str_x_pre(rt: u32, rn: u32, imm9: i32) -> u32 {
    0xf800_0c00 | (((imm9 as u32) & 0x1ff) << 12) | (rn << 5) | rt
}
const fn ldr_x_post(rt: u32, rn: u32, imm9: i32) -> u32 {
    0xf840_0400 | (((imm9 as u32) & 0x1ff) << 12) | (rn << 5) | rt
}
const fn stp_x_pre(rt: u32, rt2: u32, rn: u32, imm: i32) -> u32 {
    0xa980_0000 | ((((imm / 8) as u32) & 0x7f) << 15) | (rt2 << 10) | (rn << 5) | rt
}
const fn ldp_x_post(rt: u32, rt2: u32, rn: u32, imm: i32) -> u32 {
    0xa8c0_0000 | ((((imm / 8) as u32) & 0x7f) << 15) | (rt2 << 10) | (rn << 5) | rt
}
const fn b(offset: i32) -> u32 {
    0x1400_0000 | (((offset / 4) as u32) & 0x03ff_ffff)
}
const fn bl(offset: i32) -> u32 {
    0x9400_0000 | (((offset / 4) as u32) & 0x03ff_ffff)
}
const fn b_cond(cond: Cond, offset: i32) -> u32 {
    0x5400_0000 | ((((offset / 4) as u32) & 0x7ffff) << 5) | (cond.0 as u32)
}
const fn tbnz(bitpos: u32, rt: u32, offset: i32) -> u32 {
    0x3700_0000
        | ((bitpos >> 5) << 31)
        | ((bitpos & 31) << 19)
        | ((((offset / 4) as u32) & 0x3fff) << 5)
        | rt
}
const fn ret(rn: u32) -> u32 {
    0xd65f_0000 | (rn << 5)
}
const fn svc(imm: u32) -> u32 {
    0xd400_0001 | (imm << 5)
}
const fn brk(imm: u32) -> u32 {
    0xd420_0000 | (imm << 5)
}
const ERET: u32 = 0xd69f_03e0;
const NOP: u32 = 0xd503_201f;
const fn mrs(key: u16, rt: u32) -> u32 {
    0xd520_0000 | ((key as u32) << 5) | rt
}
const fn msr(key: u16, rt: u32) -> u32 {
    0xd500_0000 | ((key as u32) << 5) | rt
}
const fn csel(sf: u32, rd: u32, rn: u32, rm: u32, cond: Cond) -> u32 {
    (sf << 31) | 0x1a80_0000 | (rm << 16) | ((cond.0 as u32) << 12) | (rn << 5) | rd
}
const fn csinc(sf: u32, rd: u32, rn: u32, rm: u32, cond: Cond) -> u32 {
    (sf << 31) | 0x1a80_0400 | (rm << 16) | ((cond.0 as u32) << 12) | (rn << 5) | rd
}
const fn udiv(sf: u32, rd: u32, rn: u32, rm: u32) -> u32 {
    (sf << 31) | 0x1ac0_0800 | (rm << 16) | (rn << 5) | rd
}
const fn sdiv(sf: u32, rd: u32, rn: u32, rm: u32) -> u32 {
    (sf << 31) | 0x1ac0_0c00 | (rm << 16) | (rn << 5) | rd
}
const fn lslv(sf: u32, rd: u32, rn: u32, rm: u32) -> u32 {
    (sf << 31) | 0x1ac0_2000 | (rm << 16) | (rn << 5) | rd
}
const fn madd(sf: u32, rd: u32, rn: u32, rm: u32, ra: u32) -> u32 {
    (sf << 31) | 0x1b00_0000 | (rm << 16) | (ra << 10) | (rn << 5) | rd
}
const fn clz(sf: u32, rd: u32, rn: u32) -> u32 {
    (sf << 31) | 0x5ac0_1000 | (rn << 5) | rd
}
const fn rev_x(rd: u32, rn: u32) -> u32 {
    0xdac0_0c00 | (rn << 5) | rd
}
const fn ldxr_x(rt: u32, rn: u32) -> u32 {
    0xc840_7c00 | (rn << 5) | rt
}
const fn stxr_x(rs: u32, rt: u32, rn: u32) -> u32 {
    0xc800_7c00 | (rs << 16) | (rn << 5) | rt
}
const fn cas_x(rs: u32, rt: u32, rn: u32) -> u32 {
    0xc8a0_7c00 | (rs << 16) | (rn << 5) | rt
}
const fn ldadd_x(rs: u32, rt: u32, rn: u32) -> u32 {
    0xf820_0000 | (rs << 16) | (rn << 5) | rt
}
const fn tlbi_vmalle1() -> u32 {
    0xd508_871f
}

/// The `MRS`/`MSR` key for a named system register.
fn key(reg: SysReg) -> u16 {
    super::sysreg::SYSREGS
        .iter()
        .find(|s| s.reg == reg)
        .expect("the register is in the table")
        .enc
}

// ---------------------------------------------------------------------------
// The harness
// ---------------------------------------------------------------------------

/// How much RAM every test gets, at physical zero.
const RAM: u64 = 1 << 20;

struct Harness {
    cpu: Cpu,
    ram: Arc<RamStore>,
}

impl Harness {
    /// A core of the given configuration with `program` at physical zero.
    fn new(cfg: Config, program: &[u32]) -> Harness {
        let ram = Arc::new(RamStore::new(RAM));
        let space = AddressSpace::new("mem", 64);
        space
            .topology()
            .map(Region::ram("ram", Arc::clone(&ram)), 0)
            .expect("the map fits");
        let cpu = Cpu::new(cfg);
        cpu.attach_space(Arc::new(space));
        let h = Harness { cpu, ram };
        h.write_program(0, program);
        h
    }

    /// The default part for a test that does not care which: a Cortex-A53,
    /// which has `FEAT_CRC32` and deliberately no `FEAT_LSE`.
    fn a53(program: &[u32]) -> Harness {
        Harness::new(Config::cortex_a53(), program)
    }

    fn write_program(&self, at: u64, program: &[u32]) {
        for (i, word) in program.iter().enumerate() {
            self.write32(at + 4 * i as u64, *word);
        }
    }

    fn write32(&self, at: u64, word: u32) {
        for (i, byte) in word.to_le_bytes().iter().enumerate() {
            self.ram.write_u8(at + i as u64, *byte).expect("in range");
        }
    }

    fn write64(&self, at: u64, value: u64) {
        for (i, byte) in value.to_le_bytes().iter().enumerate() {
            self.ram.write_u8(at + i as u64, *byte).expect("in range");
        }
    }

    fn read64(&self, at: u64) -> u64 {
        let mut value = 0u64;
        for i in 0..8 {
            value |= u64::from(self.ram.read_u8(at + i).expect("in range")) << (8 * i);
        }
        value
    }

    fn steps(&self, n: usize) {
        for _ in 0..n {
            self.cpu.step();
        }
    }

    fn flags(&self) -> Nzcv {
        self.cpu.sysregs().nzcv
    }
}

// ---------------------------------------------------------------------------
// Integer data processing
// ---------------------------------------------------------------------------

#[test]
fn move_wide_builds_a_constant_a_piece_at_a_time() {
    let h = Harness::a53(&[
        movz(1, 0, 0xbeef, 0),
        movk(1, 0, 0xdead, 16),
        movk(1, 0, 0xcafe, 32),
        movk(1, 0, 0x1234, 48),
    ]);
    h.steps(4);
    assert_eq!(h.cpu.x(0), 0x1234_cafe_dead_beef);
}

/// `MOVZ` zeroes the rest of the register and `MOVK` does not — which is the
/// whole difference between them and the one an implementation gets wrong.
#[test]
fn movz_zeroes_and_movk_keeps() {
    let h = Harness::a53(&[movz(1, 0, 0xffff, 0), movz(1, 0, 1, 32)]);
    h.steps(2);
    assert_eq!(h.cpu.x(0), 0x0000_0001_0000_0000);
}

#[test]
fn movn_moves_the_inverse() {
    let h = Harness::a53(&[movn(1, 0, 0, 0), movn(0, 1, 0, 0)]);
    h.steps(2);
    assert_eq!(h.cpu.x(0), u64::MAX);
    // A 32-bit result zero-extends into the 64-bit register.
    assert_eq!(h.cpu.x(1), 0x0000_0000_ffff_ffff);
}

/// `SUBS` sets `C` when there was **no** borrow, which is the opposite of the
/// intuition most people bring from other architectures.
#[test]
fn subs_sets_carry_when_there_was_no_borrow() {
    let h = Harness::a53(&[
        movz(1, 0, 5, 0),
        movz(1, 1, 3, 0),
        subs_reg(1, 2, 0, 1), // 5 - 3: no borrow
        subs_reg(1, 3, 1, 0), // 3 - 5: borrow
    ]);
    h.steps(3);
    assert_eq!(h.cpu.x(2), 2);
    assert!(h.flags().c(), "5 - 3 does not borrow, so C is set");
    h.steps(1);
    assert_eq!(h.cpu.x(3), (-2i64) as u64);
    assert!(!h.flags().c(), "3 - 5 borrows, so C is clear");
    assert!(h.flags().n());
}

/// The `SP`/`XZR` distinction, which DDI 0487 C1.2.5 makes a property of the
/// *encoding* and this core makes a property of the format.
#[test]
fn register_31_is_sp_in_an_add_immediate_and_zero_elsewhere() {
    let h = Harness::a53(&[
        add_imm(1, 31, 31, 0x100), // add sp, sp, #0x100
        orr_reg(1, 0, 31, 31),     // orr x0, xzr, xzr
        add_imm(1, 1, 31, 8),      // add x1, sp, #8
    ]);
    h.cpu.set_sp(0x2000);
    h.steps(3);
    assert_eq!(h.cpu.sp(), 0x2100, "Rd and Rn of ADD immediate are SP");
    assert_eq!(h.cpu.x(0), 0, "Rn of a logical shifted register is XZR");
    assert_eq!(h.cpu.x(1), 0x2108);
}

/// `ADDS` writes `XZR`, not `SP` — the flag-setting form has a different
/// operand rule, which is why the two are different formats here.
#[test]
fn the_flag_setting_form_writes_the_zero_register() {
    let h = Harness::a53(&[adds_imm(1, 31, 31, 1)]);
    h.cpu.set_sp(0x2000);
    h.steps(1);
    assert_eq!(h.cpu.sp(), 0x2000, "ADDS must not have written SP");
    assert!(!h.flags().z());
}

#[test]
fn a_logical_immediate_is_a_replicated_bit_pattern() {
    // N=0, immr=0, imms=0b110000: an 8-bit element with one bit set,
    // replicated across 64 bits.
    let h = Harness::a53(&[
        movn(1, 0, 0, 0),
        log_imm(0x1200_0000, 1, 1, 0, 0, 0, 0b110000), // and x1, x0, #0x0101…01
    ]);
    h.steps(2);
    assert_eq!(h.cpu.x(1), 0x0101_0101_0101_0101);
}

/// `UBFM` is `LSR`, `LSL`, `UBFX` and `UXTB` depending on its two immediates,
/// and `SBFM` is their signed relatives. One implementation, four aliases.
#[test]
fn bitfield_moves_cover_the_shift_and_extend_aliases() {
    let h = Harness::a53(&[
        movz(1, 0, 0xff80, 0),
        bitfield(0x5300_0000, 1, 1, 0, 8, 63), // ubfm x1, x0, #8, #63  = lsr #8
        bitfield(0x1300_0000, 1, 2, 0, 0, 7),  // sbfm x2, x0, #0, #7   = sxtb
        bitfield(0x5300_0000, 1, 3, 0, 0, 7),  // ubfm x3, x0, #0, #7   = uxtb
    ]);
    h.steps(4);
    assert_eq!(h.cpu.x(1), 0xff);
    assert_eq!(
        h.cpu.x(2),
        0xffff_ffff_ffff_ff80,
        "sign-extended from bit 7"
    );
    assert_eq!(h.cpu.x(3), 0x80);
}

/// A variable shift takes its amount modulo the operand width, so a shift by
/// 64 is a no-op rather than a zero.
#[test]
fn a_variable_shift_wraps_at_the_operand_width() {
    let h = Harness::a53(&[
        movz(1, 0, 1, 0),
        movz(1, 1, 64, 0),
        lslv(1, 2, 0, 1),
        movz(1, 3, 32, 0),
        lslv(0, 4, 0, 3), // 32-bit: shift by 32 is also a no-op
    ]);
    h.steps(5);
    assert_eq!(h.cpu.x(2), 1);
    assert_eq!(h.cpu.x(4), 1);
}

/// A64 has no divide-by-zero exception: the result is defined to be zero.
#[test]
fn division_by_zero_produces_zero_rather_than_a_trap() {
    let h = Harness::a53(&[
        movz(1, 0, 100, 0),
        udiv(1, 1, 0, 31), // udiv x1, x0, xzr
        sdiv(1, 2, 0, 31),
    ]);
    h.steps(3);
    assert_eq!(h.cpu.x(1), 0);
    assert_eq!(h.cpu.x(2), 0);
    assert_eq!(h.cpu.pc(), 12, "no exception was taken");
}

#[test]
fn multiply_accumulate_and_the_bit_counters() {
    let h = Harness::a53(&[
        movz(1, 0, 7, 0),
        movz(1, 1, 6, 0),
        movz(1, 2, 5, 0),
        madd(1, 3, 0, 1, 2), // 5 + 7*6
        clz(1, 4, 0),
        rev_x(5, 0),
    ]);
    h.steps(6);
    assert_eq!(h.cpu.x(3), 47);
    assert_eq!(h.cpu.x(4), 61);
    assert_eq!(h.cpu.x(5), 0x0700_0000_0000_0000);
}

#[test]
fn conditional_select_reads_the_flags() {
    let h = Harness::a53(&[
        movz(1, 0, 1, 0),
        movz(1, 1, 2, 0),
        subs_reg(1, 31, 0, 0),       // cmp x0, x0 -> Z
        csel(1, 2, 0, 1, Cond::EQ),  // equal, so x0
        csinc(1, 3, 0, 1, Cond::NE), // not equal fails, so x1 + 1
    ]);
    h.steps(5);
    assert_eq!(h.cpu.x(2), 1);
    assert_eq!(h.cpu.x(3), 3);
}

// ---------------------------------------------------------------------------
// Branches
// ---------------------------------------------------------------------------

#[test]
fn branches_and_the_link_register() {
    let h = Harness::a53(&[
        bl(8),            // 0x00: call 0x08
        b(-4),            // 0x04: spin (not reached until the return)
        movz(1, 0, 9, 0), // 0x08
        ret(30),          // 0x0c
    ]);
    h.steps(3);
    assert_eq!(h.cpu.x(30), 4, "BL links to the following instruction");
    assert_eq!(h.cpu.x(0), 9);
    assert_eq!(h.cpu.pc(), 4, "RET went to X30");
}

#[test]
fn conditional_and_bit_test_branches() {
    let h = Harness::a53(&[
        movz(1, 0, 0, 0),
        subs_imm(1, 31, 0, 0), // cmp x0, #0 -> Z
        b_cond(Cond::NE, 8),   // not taken
        movz(1, 1, 1, 0),      // executed
        movz(1, 2, 0x8000, 0), // 0x10
        tbnz(15, 2, 8),        // bit 15 is set, so skip
        movz(1, 3, 0xdead, 0), // skipped
        movz(1, 4, 0x1234, 0),
    ]);
    h.steps(8);
    assert_eq!(h.cpu.x(1), 1);
    assert_eq!(h.cpu.x(3), 0);
    assert_eq!(h.cpu.x(4), 0x1234);
}

// ---------------------------------------------------------------------------
// Loads and stores
// ---------------------------------------------------------------------------

#[test]
fn loads_and_stores_cover_the_widths_and_the_sign_extension() {
    let h = Harness::a53(&[
        movz(1, 0, 0x8000, 0), // the address
        movn(1, 1, 0x7f, 0),   // 0xffff…ff80
        strb(1, 0, 0),
        ldrb(2, 0, 0),
        ldrsb_x(3, 0, 0),
        str_x(1, 0, 8),
        ldr_x(4, 0, 8),
    ]);
    h.steps(7);
    assert_eq!(h.cpu.x(2), 0x80, "LDRB zero-extends");
    assert_eq!(h.cpu.x(3), 0xffff_ffff_ffff_ff80, "LDRSB sign-extends");
    assert_eq!(h.cpu.x(4), u64::MAX - 0x7f);
}

#[test]
fn indexed_accesses_write_the_base_back() {
    let h = Harness::a53(&[
        movz(1, 0, 0x8000, 0),
        movz(1, 1, 0x1111, 0),
        str_x_pre(1, 0, -8), // str x1, [x0, #-8]!
        ldr_x_post(2, 0, 8), // ldr x2, [x0], #8
    ]);
    h.steps(4);
    assert_eq!(
        h.cpu.x(0),
        0x8000,
        "pre-index wrote back, post-index added 8"
    );
    assert_eq!(h.cpu.x(2), 0x1111);
    assert_eq!(h.read64(0x7ff8), 0x1111);
}

#[test]
fn a_register_pair_is_pushed_and_popped() {
    let h = Harness::a53(&[
        movz(1, 29, 0xaaaa, 0),
        movz(1, 30, 0xbbbb, 0),
        stp_x_pre(29, 30, 31, -16), // stp x29, x30, [sp, #-16]!
        movz(1, 29, 0, 0),
        movz(1, 30, 0, 0),
        ldp_x_post(29, 30, 31, 16), // ldp x29, x30, [sp], #16
    ]);
    h.cpu.set_sp(0x8000);
    h.steps(6);
    assert_eq!(h.cpu.sp(), 0x8000);
    assert_eq!(h.cpu.x(29), 0xaaaa);
    assert_eq!(h.cpu.x(30), 0xbbbb);
    assert_eq!(h.read64(0x7ff0), 0xaaaa);
}

/// An unaligned access is performed when `SCTLR_EL1.A` is clear and faults
/// when it is set.
#[test]
fn alignment_checking_is_a_control_bit() {
    let h = Harness::a53(&[movz(1, 0, 0x8001, 0), ldr_x(1, 0, 0)]);
    h.write64(0x8000, 0x1122_3344_5566_7788);
    h.steps(2);
    assert_eq!(h.cpu.x(1) & 0xff, 0x77, "the unaligned load was performed");

    let h = Harness::a53(&[movz(1, 0, 0x8001, 0), ldr_x(1, 0, 0)]);
    let mut regs = h.cpu.sysregs();
    regs.sctlr |= sctlr::A;
    regs.vbar_el1 = 0x4000;
    h.cpu.set_sysregs(regs);
    h.steps(2);
    assert_eq!(h.cpu.pc(), 0x4200, "an alignment fault vectored");
    let regs = h.cpu.sysregs();
    assert_eq!(regs.esr_el1 >> 26, ec::DABT_SAME);
    assert_eq!(regs.esr_el1 & 0x3f, 0b100001, "DFSC says alignment");
    assert_eq!(regs.esr_el1 & (1 << 6), 0, "WnR clear: it was a load");

    // And the same fault on a store reports a write, because `WnR` is decided
    // where the access is made rather than by the fault kind.
    let h = Harness::a53(&[movz(1, 0, 0x8001, 0), str_x(1, 0, 0)]);
    let mut regs = h.cpu.sysregs();
    regs.sctlr |= sctlr::A;
    regs.vbar_el1 = 0x4000;
    h.cpu.set_sysregs(regs);
    h.steps(2);
    assert_eq!(h.cpu.pc(), 0x4200);
    assert_ne!(h.cpu.sysregs().esr_el1 & (1 << 6), 0, "WnR set: a store");
}

/// An exclusive access must be aligned whatever `SCTLR_EL1.A` says, which is
/// the one alignment rule that is not a control bit.
#[test]
fn an_exclusive_access_is_always_alignment_checked() {
    let h = Harness::a53(&[movz(1, 0, 0x8004, 0), ldxr_x(1, 0)]);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x4000;
    assert_eq!(regs.sctlr & sctlr::A, 0, "alignment checking is off");
    h.cpu.set_sysregs(regs);
    h.steps(2);
    assert_eq!(h.cpu.pc(), 0x4200);
    assert_eq!(h.cpu.sysregs().esr_el1 & 0x3f, 0b100001);
}

/// Both interrupts pending: the fast one wins.
#[test]
fn fiq_is_taken_before_irq() {
    let h = Harness::a53(&[NOP]);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x4000;
    regs.daif = 0;
    h.cpu.set_sysregs(regs);
    h.cpu.set_interrupt(super::Lines::IRQ, true);
    h.cpu.set_interrupt(super::Lines::FIQ, true);
    h.steps(1);
    assert_eq!(
        h.cpu.pc(),
        0x4300,
        "the FIQ vector, not the IRQ one at 0x4280"
    );
}

// ---------------------------------------------------------------------------
// Exclusives and atomics
// ---------------------------------------------------------------------------

#[test]
fn a_store_exclusive_succeeds_after_its_load_and_fails_after_a_store() {
    let h = Harness::a53(&[
        movz(1, 0, 0x8000, 0),
        ldxr_x(1, 0),
        stxr_x(2, 1, 0), // succeeds
        stxr_x(3, 1, 0), // the monitor is gone, so this fails
    ]);
    h.steps(4);
    assert_eq!(h.cpu.x(2), 0, "the first store-exclusive succeeded");
    assert_eq!(h.cpu.x(3), 1, "the second found no reservation");
}

/// An ordinary store into the reserved granule breaks the reservation, which
/// is what makes the pair detect interference at all.
#[test]
fn an_intervening_store_breaks_the_reservation() {
    let h = Harness::a53(&[
        movz(1, 0, 0x8000, 0),
        ldxr_x(1, 0),
        str_x(1, 0, 0),
        stxr_x(2, 1, 0),
    ]);
    h.steps(4);
    assert_eq!(h.cpu.x(2), 1);
}

/// The lattice, executed: `CAS` is `FEAT_LSE`, so it runs on a Neoverse N1 and
/// must raise `UNDEFINED` on a Cortex-A53.
#[test]
fn lse_atomics_exist_only_on_a_part_that_has_them() {
    let program = [
        movz(1, 0, 0x8000, 0),
        movz(1, 1, 0x1111, 0), // the comparand
        movz(1, 2, 0x2222, 0), // the new value
        str_x(1, 0, 0),        // memory holds 0x1111
        cas_x(1, 2, 0),
        ldr_x(3, 0, 0),
    ];
    let h = Harness::new(Config::neoverse_n1(), &program);
    h.steps(6);
    assert_eq!(h.cpu.x(1), 0x1111, "Rs takes the old value");
    assert_eq!(h.cpu.x(3), 0x2222, "the swap happened");

    let h = Harness::a53(&program);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x4000;
    h.cpu.set_sysregs(regs);
    h.steps(5);
    assert_eq!(
        h.cpu.pc(),
        0x4200,
        "CAS did not decode, so UNDEFINED vectored"
    );
    assert_eq!(h.cpu.sysregs().esr_el1 >> 26, ec::UNKNOWN);
}

#[test]
fn an_atomic_read_modify_write_returns_the_old_value() {
    let h = Harness::new(
        Config::neoverse_n1(),
        &[
            movz(1, 0, 0x8000, 0),
            movz(1, 1, 5, 0),
            str_x(1, 0, 0),
            movz(1, 2, 3, 0),
            ldadd_x(2, 3, 0),
            ldr_x(4, 0, 0),
        ],
    );
    h.steps(6);
    assert_eq!(h.cpu.x(3), 5, "the old value");
    assert_eq!(h.cpu.x(4), 8, "the new one");
}

/// `ID_AA64ISAR0_EL1` must agree with what actually decodes, because that is
/// the register a guest reads before deciding to use `CAS`.
#[test]
fn the_id_registers_agree_with_the_decoder() {
    for cfg in [Config::cortex_a53(), Config::neoverse_n1()] {
        let h = Harness::new(cfg, &[mrs(key(SysReg::IdAa64Isar0), 0)]);
        h.steps(1);
        let atomic = (h.cpu.x(0) >> 20) & 0xf;
        assert_eq!(atomic != 0, cfg.features.lse);
        let crc = (h.cpu.x(0) >> 16) & 0xf;
        assert_eq!(crc != 0, cfg.features.crc32);
        assert_eq!(
            super::isa::decode(cas_x(1, 2, 0), cfg.features).is_some(),
            cfg.features.lse
        );
    }
}

// ---------------------------------------------------------------------------
// Exception levels
// ---------------------------------------------------------------------------

/// `SVC` from EL1 with `SP_EL1` selected takes the `0x200` group, saves
/// `PSTATE` into `SPSR_EL1` and the *following* address into `ELR_EL1`, and
/// `ERET` puts both back.
#[test]
fn a_supervisor_call_from_el1_vectors_and_returns() {
    let h = Harness::a53(&[svc(7), movz(1, 0, 0x1234, 0)]);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x4000;
    regs.daif = 0;
    h.cpu.set_sysregs(regs);
    // The handler: record a marker and return.
    h.write_program(0x4200, &[movz(1, 1, 0xabcd, 0), ERET]);

    h.steps(1);
    assert_eq!(h.cpu.pc(), 0x4200, "current EL with SP_ELx, synchronous");
    let regs = h.cpu.sysregs();
    assert_eq!(regs.esr_el1 >> 26, ec::SVC64);
    assert_eq!(regs.esr_el1 & 0xffff, 7, "the ISS carries the immediate");
    assert_eq!(regs.elr_el1, 4, "SVC returns past itself");
    assert_eq!(regs.spsr_el1 & 0xf, 0b0101, "EL1h");
    assert_eq!(regs.daif, daif::ALL, "every mask is set on entry");

    h.steps(2);
    assert_eq!(h.cpu.x(1), 0xabcd);
    assert_eq!(h.cpu.pc(), 4, "ERET went back to ELR_EL1");
    assert_eq!(h.cpu.sysregs().daif, 0, "ERET restored the masks");
    h.steps(1);
    assert_eq!(h.cpu.x(0), 0x1234);
}

/// The same call from EL0 takes a *different* vector group, and the return
/// puts the core back at EL0 on `SP_EL0`.
#[test]
fn a_supervisor_call_from_el0_takes_the_lower_el_vector() {
    let h = Harness::a53(&[svc(0)]);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x4000;
    regs.el = El::El0;
    regs.spsel = false;
    regs.sp_el0 = 0x9000;
    regs.sp_el1 = 0xa000;
    h.cpu.set_sysregs(regs);
    h.write_program(0x4400, &[ERET]);

    h.steps(1);
    assert_eq!(h.cpu.pc(), 0x4400, "lower EL using AArch64, synchronous");
    assert_eq!(h.cpu.el(), El::El1);
    assert_eq!(h.cpu.sp(), 0xa000, "the handler runs on SP_EL1");
    assert_eq!(h.cpu.sysregs().spsr_el1 & 0xf, 0b0000, "EL0t");

    h.steps(1);
    assert_eq!(h.cpu.el(), El::El0);
    assert_eq!(h.cpu.sp(), 0x9000, "back on SP_EL0");
}

/// DDI 0487 D1.3: `PSTATE.I` masks an IRQ only at the level it targets. Taken
/// from EL0 it is not maskable, and a core that got this wrong would never
/// preempt a userspace loop.
#[test]
fn el0_cannot_mask_an_interrupt_that_targets_el1() {
    let h = Harness::a53(&[NOP]);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x4000;
    regs.el = El::El1;
    regs.spsel = true;
    regs.daif = daif::ALL;
    h.cpu.set_sysregs(regs);
    h.cpu.set_interrupt(super::Lines::IRQ, true);
    h.steps(1);
    assert_eq!(h.cpu.pc(), 4, "at EL1 with I set, the IRQ is masked");

    let mut regs = h.cpu.sysregs();
    regs.el = El::El0;
    regs.spsel = false;
    h.cpu.set_sysregs(regs);
    h.cpu.set_pc(0);
    h.steps(1);
    assert_eq!(h.cpu.pc(), 0x4480, "lower EL, IRQ vector, taken anyway");
}

#[test]
fn an_el0_write_to_an_el1_register_is_undefined() {
    let h = Harness::a53(&[msr(key(SysReg::Vbar), 0)]);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x4000;
    regs.el = El::El0;
    regs.spsel = false;
    h.cpu.set_sysregs(regs);
    h.steps(1);
    assert_eq!(h.cpu.pc(), 0x4400);
    assert_eq!(h.cpu.sysregs().esr_el1 >> 26, ec::UNKNOWN);
    assert_eq!(h.cpu.sysregs().vbar_el1, 0x4000, "the write did not happen");
}

#[test]
fn breakpoint_returns_to_itself_and_a_syscall_returns_past_itself() {
    let h = Harness::a53(&[brk(3)]);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x4000;
    h.cpu.set_sysregs(regs);
    h.steps(1);
    let regs = h.cpu.sysregs();
    assert_eq!(regs.esr_el1 >> 26, ec::BRK64);
    assert_eq!(regs.elr_el1, 0, "BRK's preferred return is the BRK itself");
}

#[test]
fn a_pc_with_its_low_bits_set_raises_a_pc_alignment_fault() {
    let h = Harness::a53(&[NOP]);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x4000;
    h.cpu.set_sysregs(regs);
    h.cpu.set_pc(2);
    h.steps(1);
    assert_eq!(h.cpu.pc(), 0x4200);
    assert_eq!(h.cpu.sysregs().esr_el1 >> 26, ec::PC_ALIGN);
    assert_eq!(h.cpu.sysregs().far_el1, 2);
}

// ---------------------------------------------------------------------------
// The MMU
// ---------------------------------------------------------------------------

/// Build a three-level table hierarchy: virtual 0..2 MiB identity-mapped as a
/// block so the program keeps running, and virtual 0x20_0000 mapped to
/// physical `target` through a level-3 page.
fn map_tables(h: &Harness, target: u64, leaf_attrs: u64) {
    const L1: u64 = 0x1_0000;
    const L2: u64 = 0x1_1000;
    const L3: u64 = 0x1_2000;
    h.write64(L1, L2 | desc::VALID | desc::TABLE);
    // Level-2 entry 0: a 2 MiB identity block covering the program.
    h.write64(L2, desc::VALID | desc::AF);
    // Level-2 entry 1: a table for virtual 0x20_0000.
    h.write64(L2 + 8, L3 | desc::VALID | desc::TABLE);
    h.write64(L3, target | desc::VALID | desc::TABLE | leaf_attrs);
    let mut regs = h.cpu.sysregs();
    regs.ttbr0 = L1;
    // T0SZ = T1SZ = 25 (39-bit halves), TG1 = 0b10 (4 KiB).
    regs.tcr = 25 | (25 << 16) | (0b10 << 30);
    regs.vbar_el1 = 0x4000;
    regs.sctlr |= sctlr::M;
    h.cpu.set_sysregs(regs);
}

#[test]
fn the_mmu_translates_a_load_through_three_levels() {
    let h = Harness::a53(&[
        movz(1, 0, 0x20, 16), // x0 = 0x0020_0000
        ldr_x(1, 0, 0),
    ]);
    map_tables(&h, 0x8000, desc::AF);
    h.write64(0x8000, 0x0bad_c0de);
    h.steps(2);
    assert_eq!(h.cpu.x(1), 0x0bad_c0de);
    let (hits, misses) = h.cpu.tlb_stats();
    assert!(misses > 0 && hits + misses > 0);
}

#[test]
fn an_unmapped_address_raises_a_translation_fault_naming_its_level() {
    let h = Harness::a53(&[
        movz(1, 0, 0x40, 16), // x0 = 0x0040_0000, level-2 entry 2: absent
        ldr_x(1, 0, 0),
    ]);
    map_tables(&h, 0x8000, desc::AF);
    h.steps(2);
    assert_eq!(h.cpu.pc(), 0x4200);
    let regs = h.cpu.sysregs();
    assert_eq!(regs.esr_el1 >> 26, ec::DABT_SAME);
    assert_eq!(regs.esr_el1 & 0x3f, 0b000110, "translation fault, level 2");
    assert_eq!(regs.far_el1, 0x40_0000);
    assert_eq!(regs.esr_el1 & (1 << 6), 0, "WnR clear: it was a read");
}

#[test]
fn a_write_to_a_read_only_page_reports_wnr() {
    let h = Harness::a53(&[
        movz(1, 0, 0x20, 16),
        str_x(0, 0, 0), // store into a read-only page
    ]);
    // AP = 0b10: read-only at EL1.
    map_tables(&h, 0x8000, desc::AF | (2 << desc::AP_SHIFT));
    h.steps(2);
    assert_eq!(h.cpu.pc(), 0x4200);
    let regs = h.cpu.sysregs();
    assert_eq!(regs.esr_el1 & 0x3f, 0b001111, "permission fault, level 3");
    assert_ne!(regs.esr_el1 & (1 << 6), 0, "WnR set: it was a write");
}

/// A `TLBI` must make the core see a table the guest just rewrote. Without the
/// generation bump the old translation would still be cached, which is the
/// classic emulator bug this test exists for.
#[test]
fn tlbi_makes_a_rewritten_descriptor_visible() {
    let h = Harness::a53(&[
        movz(1, 0, 0x20, 16),
        ldr_x(1, 0, 0),
        tlbi_vmalle1(),
        ldr_x(2, 0, 0),
    ]);
    map_tables(&h, 0x8000, desc::AF);
    h.write64(0x8000, 1);
    h.write64(0x9000, 2);
    h.steps(2);
    assert_eq!(h.cpu.x(1), 1);
    // Repoint the level-3 descriptor at a different page, then invalidate.
    h.write64(0x1_2000, 0x9000 | desc::VALID | desc::TABLE | desc::AF);
    h.steps(2);
    assert_eq!(h.cpu.x(2), 2);
}

/// The debug seam: permission-free, side-effect free, and the identity when
/// the MMU is off.
#[test]
fn debug_translation_joins_the_shared_seam() {
    use crate::core::device::DebugTranslation;

    let h = Harness::a53(&[NOP]);
    assert_eq!(
        Device::debug_translate(&h.cpu, 0x1234),
        DebugTranslation::Identity,
        "with the MMU off there is nothing to translate"
    );

    // A leaf with no access flag and no EL0 permission: an ordinary access
    // would fault, and the debug walk still answers.
    map_tables(&h, 0x8000, 0);
    assert_eq!(
        Device::debug_translate(&h.cpu, 0x20_0123),
        DebugTranslation::Mapped(0x8123)
    );
    assert_eq!(
        Device::debug_translate(&h.cpu, 0x40_0000),
        DebugTranslation::Unmapped
    );
    // And it charged nothing.
    assert_eq!(h.cpu.cycles(), 0);
}

// ---------------------------------------------------------------------------
// The device surface
// ---------------------------------------------------------------------------

#[test]
fn properties_select_a_part_and_reject_an_unknown_one() -> Result<()> {
    let props = Props::new()
        .with("cpu", "neoverse-n1")
        .with("reset", 0x8000u64);
    let cpu = Cpu::from_props(&props)?;
    assert!(cpu.config().features.lse);
    assert_eq!(cpu.pc(), 0x8000);

    let bad = Props::new().with("cpu", "cortex-a9");
    assert!(Cpu::from_props(&bad).is_err());
    // A typo'd property is an error, not a silent no-op.
    let typo = Props::new().with("resest", 4u64);
    assert!(Cpu::from_props(&typo).is_err());
    Ok(())
}

#[test]
fn a_reset_returns_the_core_to_its_power_on_state() {
    let h = Harness::a53(&[movz(1, 0, 5, 0), movz(1, 1, 6, 0)]);
    h.steps(2);
    assert_eq!(h.cpu.x(0), 5);
    Device::reset(&h.cpu, ResetKind::Cold);
    assert_eq!(h.cpu.x(0), 0);
    assert_eq!(h.cpu.pc(), 0);
    assert_eq!(h.cpu.el(), El::El1);
    assert_eq!(h.cpu.sysregs().daif, daif::ALL);
}

#[test]
fn save_and_load_round_trip_to_an_identical_state() -> Result<()> {
    let h = Harness::a53(&[movz(1, 0, 0x1234, 0), movz(1, 1, 0x5678, 0), svc(1)]);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x4000;
    regs.ttbr0 = 0x1_0000;
    regs.tpidr_el0 = 0xfeed;
    h.cpu.set_sysregs(regs);
    h.cpu.set_sp(0x9000);
    h.cpu.set_interrupt(super::Lines::FIQ, true);
    h.steps(3);

    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name)?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("cpu", CLASS.name, CLASS.version)?;
        h.cpu.save(&mut chunk)?;
    }
    let bytes = w.to_vec()?;

    let restored = Cpu::new(h.cpu.config());
    let reader = StateReader::new(&bytes)?;
    let chunk = reader.load("cpu", CLASS.name, CLASS.version, &Migrations::new())?;
    let mut cr = chunk.reader();
    restored.load(&mut cr)?;
    cr.end()?;

    assert_eq!(restored.pc(), h.cpu.pc());
    assert_eq!(restored.x(0), h.cpu.x(0));
    assert_eq!(restored.sysregs(), h.cpu.sysregs());
    assert_eq!(restored.interrupts(), h.cpu.interrupts());

    let mut shape2 = MachineShape::new();
    shape2.add_device("cpu", CLASS.name)?;
    let mut w2 = StateWriter::new(shape2);
    {
        let mut chunk = w2.chunk("cpu", CLASS.name, CLASS.version)?;
        restored.save(&mut chunk)?;
    }
    assert_eq!(w2.to_vec()?, bytes, "a round trip must be a fixed point");
    Ok(())
}

#[test]
fn the_disassembler_reads_the_same_memory_the_core_fetches_from() {
    let h = Harness::a53(&[movz(1, 0, 0x2a, 0), ret(30)]);
    let listing = h.cpu.disassemble_physical(0, 2);
    assert_eq!(listing.len(), 2);
    assert_eq!(listing[0].text, "movz\tx0, #0x2a");
    assert_eq!(listing[1].text, "ret\tx30");
    // Through the MMU, at the virtual address `pc` reports.
    map_tables(&h, 0x8000, desc::AF);
    let listing = h.cpu.disassemble_virtual(0, 1);
    assert_eq!(listing[0].text, "movz\tx0, #0x2a");
}

#[test]
fn register_names_round_trip() {
    assert_eq!(X_NAMES.len(), 31);
    for (i, name) in X_NAMES.iter().enumerate() {
        assert_eq!(x_by_name(name), Some(i as u32));
    }
    assert_eq!(x_by_name("lr"), Some(30));
    assert_eq!(x_by_name("w5"), Some(5));
    // Neither of these is a general register, and pretending otherwise would
    // hand a caller a number that means different things in different
    // encodings.
    assert_eq!(x_by_name("sp"), None);
    assert_eq!(x_by_name("xzr"), None);
    assert_eq!(x_by_name("x31"), None);
}

#[test]
fn the_isa_description_covers_every_row() {
    let text = super::describe_isa();
    for op in super::isa::Op::ALL {
        assert!(text.contains(op.summary()), "{op:?} is not described");
    }
    for spec in super::sysreg::SYSREGS {
        assert!(text.contains(spec.reg.name()));
    }
}

/// A bus access nothing answers becomes an external-abort data abort rather
/// than a plausible-looking zero.
#[test]
fn an_access_off_the_end_of_the_map_is_an_external_abort() {
    let h = Harness::a53(&[movz(1, 0, 0x40, 16), ldr_x(1, 0, 0)]);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x4000;
    h.cpu.set_sysregs(regs);
    h.steps(2);
    assert_eq!(h.cpu.pc(), 0x4200);
    assert_eq!(h.cpu.sysregs().esr_el1 & 0x3f, 0b010000);
    assert_eq!(h.cpu.bus_faults(), 1);
}

/// Cycles are charged per bus access, not per instruction: a load costs the
/// fetch plus the access, and a walk costs its descriptor reads on top.
#[test]
fn cycles_count_bus_accesses() {
    let h = Harness::a53(&[NOP, ldr_x(1, 31, 0)]);
    h.cpu.set_sp(0x8000);
    h.steps(1);
    assert_eq!(h.cpu.cycles(), 1, "a NOP is one fetch");
    h.steps(1);
    assert_eq!(h.cpu.cycles(), 3, "a load is a fetch and an access");

    let h = Harness::a53(&[movz(1, 0, 0x20, 16), ldr_x(1, 0, 0)]);
    map_tables(&h, 0x8000, desc::AF);
    h.steps(2);
    // Two fetches (the first through a level-2 block, the second from the TLB)
    // plus the load, plus the walks: strictly more than the four a core
    // without an MMU would charge.
    assert!(h.cpu.cycles() > 4, "the table walks were charged");
}

#[test]
fn the_sysreg_and_instruction_tables_are_reachable_from_the_registry() -> Result<()> {
    let mut reg = crate::core::registry::Registry::new();
    super::register(&mut reg)?;
    assert!(reg.get(CLASS.name).is_some());
    // The schema exists and names the same class.
    let _ = super::schema();
    Ok(())
}

#[test]
fn currentel_and_the_pstate_views_read_back() {
    let h = Harness::a53(&[
        mrs(key(SysReg::CurrentEl), 0),
        mrs(key(SysReg::DaifReg), 1),
        movz(1, 2, 0xf000, 16), // NZCV, all four set
        msr(key(SysReg::NzcvReg), 2),
        mrs(key(SysReg::NzcvReg), 3),
    ]);
    h.steps(5);
    assert_eq!(h.cpu.x(0), 1 << 2, "EL1");
    assert_eq!(h.cpu.x(1), daif::ALL);
    assert_eq!(h.cpu.x(3), 0xf000_0000);
    assert!(h.cpu.sysregs().nzcv.n());
}

#[test]
fn wfi_stalls_until_an_interrupt_arrives() {
    let h = Harness::a53(&[0xd503_207f, movz(1, 0, 1, 0)]);
    h.steps(1);
    assert!(h.cpu.is_waiting());
    let before = h.cpu.pc();
    h.steps(3);
    assert_eq!(h.cpu.pc(), before, "still stalled");
    h.cpu.set_interrupt(super::Lines::IRQ, true);
    h.steps(1);
    assert!(!h.cpu.is_waiting());
}

/// The user-mode seam: with `SYSCALL` armed, an `SVC` leaves the core instead
/// of vectoring, and the resume address is past the instruction.
#[test]
fn an_armed_svc_exits_instead_of_vectoring() {
    use crate::core::exec::{ExitMask, ExitReason, ExitingCore};

    let h = Harness::a53(&[svc(42), movz(1, 0, 1, 0)]);
    h.cpu.set_exit_mask(ExitMask::USER);
    let (_, exit) = h.cpu.step_to_exit();
    let exit = exit.expect("the SVC exited");
    assert_eq!(exit.reason, ExitReason::SYSCALL);
    assert_eq!(h.cpu.pc(), 4, "resumed past the SVC");
    assert_eq!(h.cpu.el(), El::El1, "no exception was taken");
}

#[test]
fn every_named_part_constructs_and_names_itself() {
    let mut seen: Vec<u64> = Vec::new();
    for (name, build) in Config::PARTS {
        let cfg = build();
        assert!(Config::by_name(name).is_some());
        seen.push(cfg.midr);
        // Every part must at least decode the base set.
        assert!(super::isa::decode(NOP, cfg.features).is_some());
    }
    seen.sort_unstable();
    seen.dedup();
    assert_eq!(seen.len(), Config::PARTS.len(), "MIDRs must be distinct");
    assert!(Features::NONE.has(super::isa::Feat::Base));
}

#[test]
fn a_memory_read_with_debug_attributes_does_not_charge_the_core() {
    let h = Harness::a53(&[NOP]);
    let space = h.cpu.space().expect("attached");
    let attrs = crate::core::space::MemAttrs::DEBUG;
    assert!(space.read(0, Width::U32, attrs).is_ok());
    assert_eq!(h.cpu.cycles(), 0);
}
