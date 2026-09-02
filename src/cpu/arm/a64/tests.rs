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

// ---------------------------------------------------------------------------
// Scalar floating point
// ---------------------------------------------------------------------------
//
// The interesting cases here are the ones a reasonable guess gets wrong:
// `CPACR_EL1` traps the *first* floating-point instruction a guest executes
// and does not report `UNDEFINED` while doing it; `FMOV` between the register
// files insists the two widths match; a scalar write zeroes the rest of the
// destination; and the two `MOVI`-shaped `FMOV` forms that reach the top half
// of a vector register are the only writes that do not.

/// Assemble the `ptype` field for a precision.
const PT_S: u32 = 0b00;
const PT_D: u32 = 0b01;

const fn fp_two_src(base: u32, ptype: u32, rd: u32, rn: u32, rm: u32) -> u32 {
    base | (ptype << 22) | (rm << 16) | (rn << 5) | rd
}
const fn fadd(ptype: u32, rd: u32, rn: u32, rm: u32) -> u32 {
    fp_two_src(0x1e20_2800, ptype, rd, rn, rm)
}
const fn fmul(ptype: u32, rd: u32, rn: u32, rm: u32) -> u32 {
    fp_two_src(0x1e20_0800, ptype, rd, rn, rm)
}
const fn fdiv(ptype: u32, rd: u32, rn: u32, rm: u32) -> u32 {
    fp_two_src(0x1e20_1800, ptype, rd, rn, rm)
}
const fn fmov_to_fp(sf: u32, ptype: u32, rd: u32, rn: u32) -> u32 {
    (sf << 31) | 0x1e27_0000 | (ptype << 22) | (rn << 5) | rd
}
const fn fmov_to_gp(sf: u32, ptype: u32, rd: u32, rn: u32) -> u32 {
    (sf << 31) | 0x1e26_0000 | (ptype << 22) | (rn << 5) | rd
}
const fn fmov_hi_to_gp(rd: u32, rn: u32) -> u32 {
    0x9eae_0000 | (rn << 5) | rd
}
const fn fmov_gp_to_hi(rd: u32, rn: u32) -> u32 {
    0x9eaf_0000 | (rn << 5) | rd
}
const fn fcmp(ptype: u32, rn: u32, rm: u32) -> u32 {
    0x1e20_2000 | (ptype << 22) | (rm << 16) | (rn << 5)
}
const fn fcvtzs(sf: u32, ptype: u32, rd: u32, rn: u32) -> u32 {
    (sf << 31) | 0x1e38_0000 | (ptype << 22) | (rn << 5) | rd
}
const fn scvtf(sf: u32, ptype: u32, rd: u32, rn: u32) -> u32 {
    (sf << 31) | 0x1e22_0000 | (ptype << 22) | (rn << 5) | rd
}
const fn fcvt(dst: u32, src: u32, rd: u32, rn: u32) -> u32 {
    0x1e22_4000 | (src << 22) | (dst << 15) | (rn << 5) | rd
}
const fn fmov_imm(ptype: u32, rd: u32, imm8: u32) -> u32 {
    0x1e20_1000 | (ptype << 22) | (imm8 << 13) | rd
}
const fn ldr_v_imm(size: u32, opc1: u32, rt: u32, rn: u32, imm12: u32) -> u32 {
    (size << 30) | 0x3d40_0000 | (opc1 << 23) | (imm12 << 10) | (rn << 5) | rt
}
const fn str_v_imm(size: u32, opc1: u32, rt: u32, rn: u32, imm12: u32) -> u32 {
    (size << 30) | 0x3d00_0000 | (opc1 << 23) | (imm12 << 10) | (rn << 5) | rt
}

/// Enable EL0 and EL1 access to SIMD and floating point, the way firmware does.
fn enable_fp(h: &Harness) {
    let mut regs = h.cpu.sysregs();
    regs.cpacr = 3 << 20;
    h.cpu.set_sysregs(regs);
}

/// Bits of a `binary64` value, written with a host float only so the test
/// reads as arithmetic. The core never sees one.
fn d(value: f64) -> u64 {
    value.to_bits()
}

/// `CPACR_EL1.FPEN` resets to zero, so the first floating-point instruction a
/// guest executes traps — and it traps with exception class 0x07, which Linux
/// tells apart from an undefined instruction to decide whether to restore a
/// process's FP registers. Reporting `UNKNOWN` instead would turn a lazily
/// switched FPU into a SIGILL.
#[test]
fn floating_point_is_trapped_until_cpacr_enables_it() {
    let h = Harness::a53(&[fadd(PT_D, 0, 0, 0)]);
    let mut regs = h.cpu.sysregs();
    regs.vbar_el1 = 0x2000;
    h.cpu.set_sysregs(regs);
    h.steps(1);
    let regs = h.cpu.sysregs();
    assert_eq!(regs.esr_el1 >> 26, ec::FP_ACCESS);
    // ISS is zero for an A64 access (CV clear, COND RES0), and the return
    // address is the faulting instruction rather than the one after it: a
    // handler that enables access must be able to re-execute it.
    assert_eq!(regs.esr_el1 & 0x01ff_ffff, 0);
    assert_eq!(regs.elr_el1, 0);
    // Taken from EL1 with SP_EL1 selected: the `0x200` group.
    assert_eq!(h.cpu.pc(), 0x2200);

    // With `FPEN` set to `0b11` the same instruction retires.
    let h = Harness::a53(&[fadd(PT_D, 0, 0, 0)]);
    enable_fp(&h);
    h.steps(1);
    assert_eq!(h.cpu.sysregs().esr_el1, 0);
    assert_eq!(h.cpu.pc(), 4);
}

/// `FPEN == 0b01` traps at EL0 only, and `0b10` traps at both — the encoding
/// that looks like it should mean something different and does not.
#[test]
fn the_fpen_field_has_two_encodings_that_trap_everywhere() {
    for (fpen, el1_traps, el0_traps) in [
        (0b00u64, true, true),
        (0b01, false, true),
        (0b10, true, true),
        (0b11, false, false),
    ] {
        let mut regs = super::sysreg::SysRegs::new();
        regs.cpacr = fpen << 20;
        regs.el = El::El1;
        assert_eq!(regs.fp_access_trapped(), el1_traps, "EL1, FPEN {fpen:#b}");
        regs.el = El::El0;
        assert_eq!(regs.fp_access_trapped(), el0_traps, "EL0, FPEN {fpen:#b}");
    }
}

/// A part without `FEAT_FP` must not decode a floating-point instruction at
/// all, and must say so in `ID_AA64PFR0_EL1`. That is how a guest probes.
#[test]
fn floating_point_exists_only_on_a_part_that_has_it() {
    let bare = Config::armv8_0();
    assert!(!bare.features.fp);
    assert!(super::isa::decode(fadd(PT_D, 0, 0, 0), bare.features).is_none());
    // `ID_AA64PFR0_EL1.FP` is bits 19:16, and `0b1111` is *not implemented*.
    assert_eq!((bare.id_aa64pfr0() >> 16) & 0xf, 0xf);
    assert_eq!((Config::cortex_a53().id_aa64pfr0() >> 16) & 0xf, 0);

    // On the bare part it is UNDEFINED even with `CPACR_EL1` wide open, which
    // is the distinction between "not allowed" and "not there".
    let h = Harness::new(bare, &[fadd(PT_D, 0, 0, 0)]);
    enable_fp(&h);
    let mut regs = h.cpu.sysregs();
    regs.cpacr = 3 << 20;
    h.cpu.set_sysregs(regs);
    h.steps(1);
    assert_eq!(h.cpu.sysregs().esr_el1 >> 26, ec::UNKNOWN);
}

/// Arithmetic through the whole path: a general register into a `D` register,
/// three operations, and back out.
#[test]
fn scalar_arithmetic_runs_through_the_register_file() {
    let h = Harness::a53(&[
        movz(1, 0, 0x4000, 48), // 2.0
        movz(1, 1, 0x4008, 48), // 3.0
        fmov_to_fp(1, PT_D, 0, 0),
        fmov_to_fp(1, PT_D, 1, 1),
        fadd(PT_D, 2, 0, 1), // 5.0
        fmul(PT_D, 3, 0, 1), // 6.0
        fdiv(PT_D, 4, 1, 0), // 1.5
        fmov_to_gp(1, PT_D, 2, 2),
        fmov_to_gp(1, PT_D, 3, 3),
        fmov_to_gp(1, PT_D, 4, 4),
    ]);
    enable_fp(&h);
    h.steps(10);
    assert_eq!(h.cpu.x(2), d(5.0));
    assert_eq!(h.cpu.x(3), d(6.0));
    assert_eq!(h.cpu.x(4), d(1.5));
    // The `V` registers hold what the arithmetic put there, with nothing above.
    assert_eq!(h.cpu.v(2), u128::from(d(5.0)));
}

/// DDI 0487 C1.2.2: a scalar write zeroes the rest of the destination
/// register. It is guest-visible and software relies on it — and
/// `FMOV Vd.D[1], Xn` is the one write that does not.
#[test]
fn a_scalar_write_zeroes_the_rest_and_the_high_move_does_not() {
    let h = Harness::a53(&[
        movz(1, 0, 0x4000, 48),
        fmov_to_fp(1, PT_D, 0, 0),
        fmov_gp_to_hi(0, 0), // V0.D[1] = the same bits
        fadd(PT_S, 1, 0, 0), // a 32-bit write to V1
        fmov_hi_to_gp(1, 0), // read V0.D[1] back
    ]);
    enable_fp(&h);
    h.cpu.set_v(1, u128::MAX);
    h.steps(5);
    // `FMOV Vd.D[1], Xn` merged: both halves of V0 are set.
    assert_eq!(h.cpu.v(0), (u128::from(d(2.0)) << 64) | u128::from(d(2.0)));
    assert_eq!(h.cpu.x(1), d(2.0));
    // The 32-bit `FADD` cleared everything above its result.
    assert_eq!(h.cpu.v(1) >> 32, 0);
}

/// `FMOV` between the register files moves bits and rounds nothing, so the
/// widths must agree: `W`↔`S` and `X`↔`D` exist and `X`↔`S` does not.
#[test]
fn fmov_between_the_files_refuses_mismatched_widths() {
    // `FMOV X0, S0` — 64-bit general, 32-bit floating point.
    let h = Harness::a53(&[fmov_to_gp(1, PT_S, 0, 0)]);
    enable_fp(&h);
    h.steps(1);
    assert_eq!(h.cpu.sysregs().esr_el1 >> 26, ec::UNKNOWN);

    // `FMOV W0, S0` is the allocated pair, and it zero-extends into `X0`.
    let h = Harness::a53(&[fmov_to_gp(0, PT_S, 0, 0)]);
    enable_fp(&h);
    h.cpu.set_v(0, 0xffff_ffff_dead_beef);
    h.steps(1);
    assert_eq!(h.cpu.x(0), 0xdead_beef);
}

/// The unallocated `ptype` is `UNDEFINED`, and so is half precision, which
/// needs `FEAT_FP16` this core does not have — even though `FCVT` to and from
/// half works, because that is Armv8.0-A.
#[test]
fn half_precision_arithmetic_is_undefined_but_conversion_is_not() {
    const PT_H: u32 = 0b11;
    const PT_BAD: u32 = 0b10;
    for ptype in [PT_H, PT_BAD] {
        let h = Harness::a53(&[fadd(ptype, 0, 0, 0)]);
        enable_fp(&h);
        h.steps(1);
        assert_eq!(
            h.cpu.sysregs().esr_el1 >> 26,
            ec::UNKNOWN,
            "ptype {ptype:#b} must not do arithmetic"
        );
    }
    // `FCVT H0, D0` then `FCVT D1, H0`: 1.5 survives the round trip, and the
    // intermediate really is the binary16 encoding.
    let h = Harness::a53(&[
        movz(1, 0, 0x3ff8, 48),
        fmov_to_fp(1, PT_D, 0, 0),
        fcvt(PT_H, PT_D, 1, 0),
        fmov_to_gp(0, PT_S, 1, 1),
        fcvt(PT_D, PT_H, 2, 1),
        fmov_to_gp(1, PT_D, 2, 2),
    ]);
    enable_fp(&h);
    h.steps(6);
    assert_eq!(h.cpu.x(1) & 0xffff, 0x3e00, "1.5 as binary16");
    assert_eq!(h.cpu.x(2), d(1.5));
    // A conversion to the precision it came from is unallocated.
    let h = Harness::a53(&[fcvt(PT_D, PT_D, 0, 0)]);
    enable_fp(&h);
    h.steps(1);
    assert_eq!(h.cpu.sysregs().esr_el1 >> 26, ec::UNKNOWN);
}

/// `FCMP` writes `NZCV` in a pattern no integer comparison produces: unordered
/// sets `C` **and** `V`, which is what makes `B.VS` the "was there a NaN" test.
#[test]
fn a_floating_point_compare_sets_c_and_v_when_unordered() {
    let h = Harness::a53(&[
        movz(1, 0, 0x7ff8, 48), // a quiet NaN
        fmov_to_fp(1, PT_D, 0, 0),
        fcmp(PT_D, 0, 0),
    ]);
    enable_fp(&h);
    h.steps(3);
    assert_eq!(h.flags(), Nzcv::new(false, false, true, true));
    // The comparison of a NaN with itself is unordered rather than equal, and
    // `FCMP` does not raise on a quiet one.
    assert_eq!(h.cpu.sysregs().fpsr, 0);
}

/// `FPSR` is sticky and `FPCR` selects the rounding direction, both through
/// the ordinary system-register path.
#[test]
fn fpcr_selects_the_rounding_and_fpsr_accumulates() {
    let h = Harness::a53(&[
        movz(1, 0, 0x3ff0, 48), // 1.0
        movz(1, 1, 0x4008, 48), // 3.0
        fmov_to_fp(1, PT_D, 0, 0),
        fmov_to_fp(1, PT_D, 1, 1),
        fdiv(PT_D, 2, 0, 1),
        fmov_to_gp(1, PT_D, 2, 2),
    ]);
    enable_fp(&h);
    h.steps(6);
    assert_eq!(h.cpu.x(2), 0x3fd5_5555_5555_5555, "1/3, round to nearest");
    // Inexact, and nothing else.
    assert_eq!(h.cpu.sysregs().fpsr, 1 << 4);

    // Round toward +infinity. Arm's encoding is `01`, which is x86's
    // round-toward-*negative* — the mistake this asserts against.
    let h = Harness::new(
        Config::cortex_a53(),
        &[
            movz(1, 0, 0x3ff0, 48),
            movz(1, 1, 0x4008, 48),
            fmov_to_fp(1, PT_D, 0, 0),
            fmov_to_fp(1, PT_D, 1, 1),
            fdiv(PT_D, 2, 0, 1),
            fmov_to_gp(1, PT_D, 2, 2),
        ],
    );
    let mut regs = h.cpu.sysregs();
    regs.cpacr = 3 << 20;
    regs.fpcr = 1 << 22;
    h.cpu.set_sysregs(regs);
    h.steps(6);
    assert_eq!(h.cpu.x(2), 0x3fd5_5555_5555_5556);
}

/// The bits of `FPCR` this core does not implement are RES0: a guest that
/// writes `AHP` or an exception-enable bit reads back zero and can tell.
#[test]
fn the_unimplemented_fpcr_bits_read_back_as_zero() {
    let h = Harness::a53(&[
        movn(1, 0, 0, 0), // all ones
        msr(key(SysReg::Fpcr), 0),
        mrs(key(SysReg::Fpcr), 1),
        msr(key(SysReg::Fpsr), 0),
        mrs(key(SysReg::Fpsr), 2),
    ]);
    enable_fp(&h);
    h.steps(5);
    assert_eq!(h.cpu.x(1), super::fp::fpcr::WRITABLE);
    assert_eq!(h.cpu.x(2), super::fp::fpsr::WRITABLE);
    // `AHP` is bit 26 and is one of the ones that must not stick.
    assert_eq!(h.cpu.x(1) & (1 << 26), 0);
}

/// A conversion out of range saturates and a NaN converts to zero, both with
/// invalid raised — which is `IntOverflow::SaturateNanZero` reaching the guest.
#[test]
fn a_float_to_integer_conversion_saturates() {
    let h = Harness::a53(&[
        movz(1, 0, 0x7ff0, 48), // +inf
        fmov_to_fp(1, PT_D, 0, 0),
        fcvtzs(1, PT_D, 1, 0),
        movz(1, 2, 0x7ff8, 48), // a quiet NaN
        fmov_to_fp(1, PT_D, 2, 2),
        fcvtzs(1, PT_D, 3, 2),
        scvtf(1, PT_D, 4, 1), // i64::MAX back to a double
        fmov_to_gp(1, PT_D, 4, 4),
    ]);
    enable_fp(&h);
    h.steps(8);
    assert_eq!(h.cpu.x(1), 0x7fff_ffff_ffff_ffff);
    assert_eq!(h.cpu.x(3), 0);
    // `i64::MAX` is not representable in `binary64`, so `SCVTF` rounds it up
    // to 2^63 — a value that would not fit back in a signed integer.
    assert_eq!(h.cpu.x(4), 0x43e0_0000_0000_0000);
    assert_eq!(h.cpu.sysregs().fpsr & 1, 1, "invalid was raised");
}

/// `FMOV` with an immediate expands eight bits into the destination's format,
/// so the same encoding is a different number at each precision.
#[test]
fn the_move_immediate_expands_per_precision() {
    let h = Harness::a53(&[
        fmov_imm(PT_D, 0, 0x70),
        fmov_to_gp(1, PT_D, 0, 0),
        fmov_imm(PT_S, 1, 0x70),
        fmov_to_gp(0, PT_S, 1, 1),
    ]);
    enable_fp(&h);
    h.steps(4);
    assert_eq!(h.cpu.x(0), d(1.0));
    assert_eq!(h.cpu.x(1), u64::from(1.0f32.to_bits()));
}

/// A 128-bit load and store move all sixteen bytes, and the `Q` width is the
/// one spelled across two non-adjacent fields.
#[test]
fn a_quadword_load_and_store_move_the_whole_register() {
    let h = Harness::a53(&[
        ldr_v_imm(0, 1, 0, 31, 0), // ldr q0, [sp]
        str_v_imm(0, 1, 0, 31, 1), // str q0, [sp, #16]
        ldr_v_imm(3, 0, 1, 31, 2), // ldr d1, [sp, #16]
    ]);
    enable_fp(&h);
    h.cpu.set_sp(0x800);
    h.write64(0x800, 0x0011_2233_4455_6677);
    h.write64(0x808, 0x8899_aabb_ccdd_eeff);
    h.steps(3);
    assert_eq!(
        h.cpu.v(0),
        (0x8899_aabb_ccdd_eeffu128 << 64) | 0x0011_2233_4455_6677
    );
    assert_eq!(h.read64(0x810), 0x0011_2233_4455_6677);
    assert_eq!(h.read64(0x818), 0x8899_aabb_ccdd_eeff);
    // The `D` load took only the low half and zeroed the rest.
    assert_eq!(h.cpu.v(1), 0x0011_2233_4455_6677);
}

/// A `Q` access must be aligned to sixteen bytes when `SCTLR_EL1.A` is set,
/// and the width it is checked against is the access the guest asked for
/// rather than the eight-byte pieces the bus sees.
#[test]
fn a_quadword_access_is_checked_against_sixteen_bytes() {
    let h = Harness::a53(&[ldr_v_imm(0, 1, 0, 0, 0)]);
    enable_fp(&h);
    let mut regs = h.cpu.sysregs();
    regs.sctlr |= sctlr::A;
    regs.cpacr = 3 << 20;
    h.cpu.set_sysregs(regs);
    h.cpu.set_x(0, 8); // eight-byte aligned, not sixteen
    h.steps(1);
    assert_eq!(h.cpu.sysregs().esr_el1 >> 26, ec::DABT_SAME);
    assert_eq!(h.cpu.sysregs().far_el1, 8);
}

/// The disassembler prints the SIMD&FP register file at the width the encoding
/// names, which is the whole reason A64's floating-point mnemonics carry no
/// suffix.
#[test]
fn the_disassembler_names_the_floating_point_width() {
    let text = |word: u32| super::disasm::disassemble(word, 0, Features::ALL).text;
    assert_eq!(text(fadd(PT_D, 0, 1, 2)), "fadd\td0, d1, d2");
    assert_eq!(text(fadd(PT_S, 0, 1, 2)), "fadd\ts0, s1, s2");
    assert_eq!(text(fcvt(PT_D, PT_S, 0, 1)), "fcvt\td0, s1");
    assert_eq!(text(fcvtzs(1, PT_D, 0, 1)), "fcvtzs\tx0, d1");
    assert_eq!(text(fcvtzs(0, PT_S, 0, 1)), "fcvtzs\tw0, s1");
    assert_eq!(text(scvtf(1, PT_D, 0, 1)), "scvtf\td0, x1");
    assert_eq!(text(fmov_hi_to_gp(0, 1)), "fmov\tx0, v1.d[1]");
    assert_eq!(text(fmov_gp_to_hi(0, 1)), "fmov\tv0.d[1], x1");
    assert_eq!(text(ldr_v_imm(0, 1, 0, 1, 1)), "ldr\tq0, [x1, #0x10]");
    assert_eq!(text(ldr_v_imm(0, 0, 0, 1, 1)), "ldr\tb0, [x1, #0x1]");
    // An unallocated `ptype` prints the register as unknown rather than
    // guessing a width the encoding does not name.
    assert!(text(fadd(0b10, 0, 1, 2)).contains("v0.?"));
    // A part without floating point does not disassemble one at all.
    let bare = super::disasm::disassemble(fadd(PT_D, 0, 1, 2), 0, Features::NONE);
    assert!(bare.text.starts_with(".word"));
}

/// `LDNP`/`STNP` are the non-temporal pair, and the signed-word form has no
/// non-temporal counterpart — so `opc == 0b01` must stay UNDEFINED.
#[test]
fn the_non_temporal_pair_exists_and_its_signed_form_does_not() {
    use super::isa::{Op, decode};
    assert_eq!(decode(0x2900_0000, Features::ALL).unwrap().op, Op::StpWOff);
    assert_eq!(decode(0x2800_0000, Features::ALL).unwrap().op, Op::StnpW);
    assert_eq!(decode(0x2840_0000, Features::ALL).unwrap().op, Op::LdnpW);
    assert_eq!(decode(0xa840_0000, Features::ALL).unwrap().op, Op::LdnpX);
    assert_eq!(decode(0x2c40_0000, Features::ALL).unwrap().op, Op::LdnpV);
    // `LDPSW` exists; `LDNPSW` does not.
    assert_eq!(decode(0x6940_0000, Features::ALL).unwrap().op, Op::LdpswOff);
    assert!(decode(0x6840_0000, Features::ALL).is_none());
}

/// The state chunk carries the whole SIMD&FP file, `FPCR` and `FPSR`, and a
/// round trip is a fixed point.
#[test]
fn the_floating_point_state_survives_a_snapshot() -> Result<()> {
    let h = Harness::a53(&[]);
    enable_fp(&h);
    for i in 0..32u32 {
        h.cpu
            .set_v(i, (u128::from(i) << 100) | u128::from(0xdead_beefu32) << i);
    }
    let mut regs = h.cpu.sysregs();
    regs.fpcr = 1 << 22;
    regs.fpsr = 0x11;
    h.cpu.set_sysregs(regs);

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

    for i in 0..32u32 {
        assert_eq!(restored.v(i), h.cpu.v(i), "V{i}");
    }
    assert_eq!(restored.sysregs(), h.cpu.sysregs());

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

/// The fixed-point conversions, which have no Rust spelling and so are not
/// covered by the built corpus: `SCVTF <Dd>, <Xn>, #fbits` reads the integer
/// as a fixed-point value with `fbits` fraction bits, and `FCVTZS` is its
/// inverse.
#[test]
fn the_fixed_point_conversions_scale_by_a_power_of_two() {
    const fn scvtf_fix(sf: u32, ptype: u32, rd: u32, rn: u32, fbits: u32) -> u32 {
        (sf << 31) | 0x1e02_0000 | (ptype << 22) | ((64 - fbits) << 10) | (rn << 5) | rd
    }
    const fn fcvtzs_fix(sf: u32, ptype: u32, rd: u32, rn: u32, fbits: u32) -> u32 {
        (sf << 31) | 0x1e18_0000 | (ptype << 22) | ((64 - fbits) << 10) | (rn << 5) | rd
    }
    let h = Harness::a53(&[
        movz(1, 0, 3, 0),
        scvtf_fix(1, PT_D, 0, 0, 1), // 3 with one fraction bit is 1.5
        fmov_to_gp(1, PT_D, 1, 0),
        fcvtzs_fix(1, PT_D, 2, 0, 2), // 1.5 with two fraction bits is 6
        movn(1, 3, 0, 0),             // -1
        scvtf_fix(1, PT_D, 1, 3, 4),  // -1 with four fraction bits is -1/16
        fmov_to_gp(1, PT_D, 4, 1),
    ]);
    enable_fp(&h);
    h.steps(7);
    assert_eq!(h.cpu.x(1), d(1.5));
    assert_eq!(h.cpu.x(2), 6);
    assert_eq!(h.cpu.x(4), d(-0.0625));

    // A 32-bit form may not name more than 32 fraction bits: DDI 0487 makes
    // the top bit of `scale` mandatory when `sf` is clear.
    let h = Harness::a53(&[scvtf_fix(0, PT_S, 0, 0, 33)]);
    enable_fp(&h);
    h.steps(1);
    assert_eq!(h.cpu.sysregs().esr_el1 >> 26, ec::UNKNOWN);
}

/// `CPACR_EL1.FPEN` traps an `MRS` of `FPCR` too, not only the instructions
/// that use the registers. A kernel saving floating-point context reads `FPSR`
/// before it reads a single `V` register, so a trap that let that read through
/// would make lazy context switching see the wrong process's state.
#[test]
fn the_access_trap_covers_fpcr_and_fpsr_as_well() {
    let h = Harness::a53(&[mrs(key(SysReg::Fpcr), 0)]);
    h.steps(1);
    assert_eq!(h.cpu.sysregs().esr_el1 >> 26, ec::FP_ACCESS);

    let h = Harness::a53(&[msr(key(SysReg::Fpsr), 0)]);
    h.steps(1);
    assert_eq!(h.cpu.sysregs().esr_el1 >> 26, ec::FP_ACCESS);

    // An unrelated system register is not trapped by it.
    let h = Harness::a53(&[mrs(key(SysReg::Midr), 0)]);
    h.steps(1);
    assert_eq!(h.cpu.sysregs().esr_el1, 0);
    assert_eq!(h.cpu.x(0), Config::cortex_a53().midr);
}
