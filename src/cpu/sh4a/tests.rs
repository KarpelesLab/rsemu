//! Tests for the SH-4A core.
//!
//! Instructions are assembled by the small encoders below — transcribed from
//! the *Instruction Code* column of manual section 3.3 — so a test says what
//! it means; `isa`'s own tests prove the decoder agrees with the table. Where
//! a test's expected value comes from a worked example in the manual, the
//! section is named.

use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::device::{Device, Export, ExportId};
use crate::core::error::Result;
use crate::core::exec::{ExitMask, ExitReason, ExitingCore};
use crate::core::props::Props;
use crate::core::space::{AddressSpace, RamStore, Region};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::Ordering;
use crate::core::value::Endian;

use super::disasm::{self, Mode};
use super::exec::{exc, fpscr, sr};
use super::mmu::reg;
use super::{CLASS, Config, Cpu, pack_request};

// ---------------------------------------------------------------------------
// Assembly
// ---------------------------------------------------------------------------

/// `tttt nnnn mmmm llll`.
const fn rr(top: u16, n: usize, m: usize, low: u16) -> u16 {
    (top << 12) | ((n as u16) << 8) | ((m as u16) << 4) | low
}
/// `tttt nnnn llll llll`.
const fn rn(top: u16, n: usize, low: u16) -> u16 {
    (top << 12) | ((n as u16) << 8) | low
}
const NOP: u16 = 0x0009;
const RTS: u16 = 0x000b;
const RTE: u16 = 0x002b;
const SLEEP: u16 = 0x001b;
const CLRT: u16 = 0x0008;
const SETT: u16 = 0x0018;
const SETS: u16 = 0x0058;
const CLRMAC: u16 = 0x0028;
const DIV0U: u16 = 0x0019;
const LDTLB: u16 = 0x0038;
const FSCHG: u16 = 0xf3fd;
const FRCHG: u16 = 0xfbfd;
const fn mov_i(n: usize, imm: i8) -> u16 {
    rn(0xe, n, imm as u8 as u16)
}
const fn mov(m: usize, n: usize) -> u16 {
    rr(6, n, m, 3)
}
const fn add(m: usize, n: usize) -> u16 {
    rr(3, n, m, 0xc)
}
const fn add_i(n: usize, imm: i8) -> u16 {
    rn(7, n, imm as u8 as u16)
}
const fn addc(m: usize, n: usize) -> u16 {
    rr(3, n, m, 0xe)
}
const fn addv(m: usize, n: usize) -> u16 {
    rr(3, n, m, 0xf)
}
const fn subc(m: usize, n: usize) -> u16 {
    rr(3, n, m, 0xa)
}
const fn subv(m: usize, n: usize) -> u16 {
    rr(3, n, m, 0xb)
}
const fn negc(m: usize, n: usize) -> u16 {
    rr(6, n, m, 0xa)
}
const fn movl_s(m: usize, n: usize) -> u16 {
    rr(2, n, m, 2)
}
const fn movl_l(m: usize, n: usize) -> u16 {
    rr(6, n, m, 2)
}
const fn movw_l(m: usize, n: usize) -> u16 {
    rr(6, n, m, 1)
}
const fn movl_m(m: usize, n: usize) -> u16 {
    rr(2, n, m, 6)
}
const fn movl_p(m: usize, n: usize) -> u16 {
    rr(6, n, m, 6)
}
const fn movl_pc(n: usize, disp: u8) -> u16 {
    rn(0xd, n, disp as u16)
}
const fn bra(disp: i16) -> u16 {
    0xa000 | (disp as u16 & 0xfff)
}
const fn bt(disp: i8) -> u16 {
    0x8900 | disp as u8 as u16
}
const fn bf(disp: i8) -> u16 {
    0x8b00 | disp as u8 as u16
}
const fn bts(disp: i8) -> u16 {
    0x8d00 | disp as u8 as u16
}
const fn dt(n: usize) -> u16 {
    rn(4, n, 0x10)
}
const fn jsr(n: usize) -> u16 {
    rn(4, n, 0x0b)
}
const fn trapa(imm: u8) -> u16 {
    0xc300 | imm as u16
}
const fn ldc_sr(m: usize) -> u16 {
    rn(4, m, 0x0e)
}
const fn stc_sr(n: usize) -> u16 {
    rn(0, n, 0x02)
}
const fn ldc_bank(m: usize, b: usize) -> u16 {
    rn(4, m, 0x8e | ((b as u16) << 4))
}
const fn stc_bank(b: usize, n: usize) -> u16 {
    rn(0, n, 0x82 | ((b as u16) << 4))
}
const fn div1(m: usize, n: usize) -> u16 {
    rr(3, n, m, 4)
}
const fn div0s(m: usize, n: usize) -> u16 {
    rr(2, n, m, 7)
}
const fn rotcl(n: usize) -> u16 {
    rn(4, n, 0x24)
}
const fn shad(m: usize, n: usize) -> u16 {
    rr(4, n, m, 0xc)
}
const fn shld(m: usize, n: usize) -> u16 {
    rr(4, n, m, 0xd)
}
const fn mac_l(m: usize, n: usize) -> u16 {
    rr(0, n, m, 0xf)
}
const fn mac_w(m: usize, n: usize) -> u16 {
    rr(4, n, m, 0xf)
}
const fn movli(m: usize) -> u16 {
    rn(0, m, 0x63)
}
const fn movco(n: usize) -> u16 {
    rn(0, n, 0x73)
}
const fn movua(m: usize) -> u16 {
    rn(4, m, 0xa9)
}
const fn movua_p(m: usize) -> u16 {
    rn(4, m, 0xe9)
}
const fn tas(n: usize) -> u16 {
    rn(4, n, 0x1b)
}
const fn pref(n: usize) -> u16 {
    rn(0, n, 0x83)
}
const fn lds_fpscr(m: usize) -> u16 {
    rn(4, m, 0x6a)
}
const fn fadd(m: usize, n: usize) -> u16 {
    rr(0xf, n, m, 0)
}
const fn fmul(m: usize, n: usize) -> u16 {
    rr(0xf, n, m, 2)
}
const fn fdiv(m: usize, n: usize) -> u16 {
    rr(0xf, n, m, 3)
}
const fn fcmp_eq(m: usize, n: usize) -> u16 {
    rr(0xf, n, m, 4)
}
const fn fcmp_gt(m: usize, n: usize) -> u16 {
    rr(0xf, n, m, 5)
}
const fn fmac(m: usize, n: usize) -> u16 {
    rr(0xf, n, m, 0xe)
}
const fn fmov_s(m: usize, n: usize) -> u16 {
    rr(0xf, n, m, 0xa)
}
const fn fmov_l(m: usize, n: usize) -> u16 {
    rr(0xf, n, m, 8)
}
const fn float(n: usize) -> u16 {
    rn(0xf, n, 0x2d)
}
const fn ftrc(m: usize) -> u16 {
    rn(0xf, m, 0x3d)
}
const fn fsqrt(n: usize) -> u16 {
    rn(0xf, n, 0x6d)
}
const fn lds_fpul(m: usize) -> u16 {
    rn(4, m, 0x5a)
}
const fn sts_fpul(n: usize) -> u16 {
    rn(0, n, 0x5a)
}
const fn fipr(m: usize, n: usize) -> u16 {
    0xf0ed | (((n / 4) as u16) << 10) | (((m / 4) as u16) << 8)
}
const fn ftrv(n: usize) -> u16 {
    0xf1fd | (((n / 4) as u16) << 10)
}
const fn fsca(n: usize) -> u16 {
    rn(0xf, n, 0xfd)
}
const fn fcnvsd(n: usize) -> u16 {
    rn(0xf, n, 0xad)
}
const fn fcnvds(m: usize) -> u16 {
    rn(0xf, m, 0xbd)
}

// ---------------------------------------------------------------------------
// The harness
// ---------------------------------------------------------------------------

/// Physical RAM size.
const RAM: u64 = 0x10_0000;
/// Where programs are loaded, physically.
const PROG: u32 = 0x1000;
/// The P2 (uncached, untranslated) view of it, where programs run.
const START: u32 = 0xa000_0000 | PROG;
/// Where `VBR` points: P1 physical zero, so the handlers are at physical
/// 0x100, 0x400 and 0x600.
const VBR: u32 = 0x8000_0000;
/// A data area, physically.
const DATA: u32 = 0x8000;
/// A P4 peripheral window the harness maps on the bus.
const PERIPH: u32 = 0xfe00_0000;

struct Harness {
    cpu: Cpu,
    ram: Arc<RamStore>,
    periph: Arc<RamStore>,
    endian: Endian,
}

impl Harness {
    fn with(cfg: Config, program: &[u16]) -> Harness {
        let ram = Arc::new(RamStore::new(RAM));
        let periph = Arc::new(RamStore::new(0x1000));
        let space = AddressSpace::new("mem", 32).with_endian(cfg.endian);
        {
            let mut t = space.topology();
            t.map(
                Region::ram("ram", Arc::clone(&ram)).with_endian(cfg.endian),
                0,
            )
            .unwrap();
            t.map(
                Region::ram("periph", Arc::clone(&periph)).with_endian(cfg.endian),
                u64::from(PERIPH),
            )
            .unwrap();
        }
        let cpu = Cpu::new(cfg.with_reset_vector(START));
        cpu.attach_space(Arc::new(space));
        cpu.set_vbr(VBR);
        // Kernel code, bank 0, unblocked, nothing masked.
        cpu.set_sr(sr::MD);
        let h = Harness {
            cpu,
            ram,
            periph,
            endian: cfg.endian,
        };
        h.load(PROG, program);
        h
    }

    fn new(program: &[u16]) -> Harness {
        Harness::with(Config::new(), program)
    }

    fn load(&self, at: u32, program: &[u16]) {
        for (i, w) in program.iter().enumerate() {
            self.put16(at + 2 * i as u32, *w);
        }
    }

    fn bytes<const N: usize>(&self, v: [u8; N]) -> [u8; N] {
        let mut v = v;
        if self.endian == Endian::Big {
            v.reverse();
        }
        v
    }

    fn put16(&self, at: u32, v: u16) {
        for (k, b) in self.bytes(v.to_le_bytes()).iter().enumerate() {
            self.ram.write_u8(u64::from(at) + k as u64, *b).unwrap();
        }
    }

    fn put32(&self, at: u32, v: u32) {
        for (k, b) in self.bytes(v.to_le_bytes()).iter().enumerate() {
            self.ram.write_u8(u64::from(at) + k as u64, *b).unwrap();
        }
    }

    fn get32(&self, at: u32) -> u32 {
        let mut b = [0u8; 4];
        for (k, slot) in b.iter_mut().enumerate() {
            *slot = self.ram.read_u8(u64::from(at) + k as u64).unwrap();
        }
        u32::from_le_bytes(self.bytes(b))
    }

    fn steps(&self, n: usize) {
        for _ in 0..n {
            self.cpu.step();
        }
    }

    fn r(&self, n: usize) -> u32 {
        self.cpu.reg(n)
    }

    fn expevt(&self) -> u32 {
        self.cpu.ctl().expevt
    }

    fn t(&self) -> bool {
        self.cpu.sr() & sr::T != 0
    }
}

/// The P1 address of physical `a`.
const fn p1(a: u32) -> u32 {
    0x8000_0000 | a
}

// ---------------------------------------------------------------------------
// Reset and programs that run
// ---------------------------------------------------------------------------

#[test]
fn the_reset_state_is_the_manuals() {
    // Table 2.1: MD = RB = BL = 1, FD = 0, IMASK = 1111; VBR = 0; PC =
    // H'A0000000; FPSCR = H'00040001; EXPEVT = 0 after a power-on reset.
    let cpu = Cpu::new(Config::new());
    assert_eq!(cpu.sr(), sr::MD | sr::RB | sr::BL | sr::IMASK);
    assert_eq!(cpu.pc(), 0xa000_0000);
    assert_eq!(cpu.control()[1], 0);
    assert_eq!(cpu.fpscr(), 0x0004_0001);
    assert_eq!(cpu.ctl().expevt, 0);
    assert_eq!(cpu.ctl().mmucr, 0);
    assert_eq!(cpu.ctl().cpuopm, 0x3e0);
}

#[test]
fn a_counted_loop_runs_and_stores_its_answer() {
    // r1 = 10; r0 = 0; do { r0 += r1 } while (--r1); store r0 at DATA.
    let h = Harness::new(&[
        mov_i(1, 10),
        mov_i(0, 0),
        add(1, 0),     // loop:
        dt(1),         //
        bf(-4),        // back to `add`
        movl_pc(2, 2), // r2 = the literal below
        movl_s(0, 2),
        bra(-2), // spin here
        NOP,
        NOP,
        0,
        0, // literal (aligned at PROG+20)
    ]);
    h.put32(PROG + 20, p1(DATA));
    h.cpu.run(200);
    assert_eq!(h.r(0), 55);
    assert_eq!(h.get32(DATA), 55);
}

#[test]
fn a_call_returns_through_pr() {
    let h = Harness::new(&[
        movl_pc(1, 2), // r1 = sub
        jsr(1),
        mov_i(2, 7), // delay slot: runs before the call
        mov_i(3, 9), // after return
        bra(-1),
        NOP,
        0,
        0,           // literal at PROG+12
        add_i(2, 1), // sub at PROG+16
        RTS,
        add_i(2, 1), // slot of rts
    ]);
    h.put32(PROG + 12, START + 16);
    h.steps(8);
    assert_eq!(h.r(2), 9);
    assert_eq!(h.r(3), 9);
    assert_eq!(h.cpu.system().2, START + 6, "PR = the JSR + 4");
}

// ---------------------------------------------------------------------------
// Delay slots
// ---------------------------------------------------------------------------

#[test]
fn the_slot_runs_before_the_branch_lands() {
    let h = Harness::new(&[bra(2), mov_i(1, 5), mov_i(1, 6), NOP, mov_i(2, 1)]);
    h.steps(1);
    assert_eq!(h.cpu.delay_target(), Some(START + 8));
    assert_eq!(h.cpu.pc(), START + 2);
    h.steps(1);
    assert_eq!(h.r(1), 5);
    assert_eq!(h.cpu.pc(), START + 8);
    h.steps(1);
    assert_eq!(h.r(2), 1);
}

#[test]
fn an_untaken_bt_s_still_has_a_delay_slot() {
    // §3.1: "The instruction following BF/S or BT/S for which the branch is
    // not taken is also a delay slot instruction" — so a branch in it is
    // slot-illegal.
    let h = Harness::new(&[CLRT, bts(4), bra(0)]);
    h.steps(3);
    assert_eq!(h.expevt(), exc::SLOT_ILLEGAL);
    assert_eq!(h.cpu.control()[3], START + 2, "SPC = the BT/S");
}

#[test]
fn a_branch_in_a_slot_is_slot_illegal_and_reports_the_branch() {
    let h = Harness::new(&[NOP, bra(4), bt(0)]);
    h.steps(3);
    assert_eq!(h.expevt(), exc::SLOT_ILLEGAL);
    assert_eq!(h.cpu.control()[3], START + 2);
    assert_eq!(h.cpu.pc(), VBR + 0x100);
}

#[test]
fn pc_relative_loads_and_mova_are_slot_illegal_too() {
    let h = Harness::new(&[bra(4), movl_pc(1, 0)]);
    h.steps(2);
    assert_eq!(h.expevt(), exc::SLOT_ILLEGAL);
    let h = Harness::new(&[bra(4), 0xc700]);
    h.steps(2);
    assert_eq!(h.expevt(), exc::SLOT_ILLEGAL);
}

#[test]
fn an_address_error_in_a_slot_points_spc_at_the_branch() {
    let h = Harness::new(&[mov_i(1, 1), bra(4), movl_l(1, 2)]);
    h.steps(3);
    assert_eq!(h.expevt(), exc::ADDR_READ);
    assert_eq!(h.cpu.control()[3], START + 2);
    assert_eq!(h.cpu.ctl().tea, 1);
}

#[test]
fn no_interrupt_lands_between_a_branch_and_its_slot() {
    let h = Harness::new(&[bra(4), mov_i(1, 1), NOP, NOP, NOP, NOP, NOP]);
    h.steps(1);
    h.cpu.set_interrupt(5, 0x400);
    h.steps(1); // the slot runs; the interrupt waits
    assert_eq!(h.r(1), 1);
    assert_eq!(h.cpu.pc(), START + 12);
    h.steps(1); // now it is taken, with SPC = the branch target
    assert_eq!(h.cpu.pc(), VBR + 0x600);
    assert_eq!(h.cpu.control()[3], START + 12);
}

#[test]
fn the_branch_target_is_the_register_before_the_slot_changed_it() {
    // §10.1.57 note: the target is the register's value at the branch.
    let h = Harness::new(&[
        movl_pc(1, 1),
        0x412b, /* jmp @r1 */
        mov_i(1, 0),
        NOP,
        0,
        0,
    ]);
    h.put32(PROG + 8, START + 8);
    h.steps(3);
    assert_eq!(h.cpu.pc(), START + 8);
}

// ---------------------------------------------------------------------------
// Exceptions
// ---------------------------------------------------------------------------

#[test]
fn trapa_saves_state_and_rte_returns() {
    let h = Harness::new(&[mov_i(15, 0x40), trapa(0x21), mov_i(2, 3), NOP]);
    // The handler at VBR + 0x100: rte with a nop in its slot.
    h.load(0x100, &[RTE, NOP]);
    let sr_before = h.cpu.sr();
    h.steps(2);
    let [_, _, ssr, spc, sgr, _] = h.cpu.control();
    assert_eq!(h.expevt(), exc::TRAPA);
    assert_eq!(h.cpu.ctl().tra, 0x21 << 2);
    assert_eq!(
        spc,
        START + 4,
        "completion type: SPC is the next instruction"
    );
    assert_eq!(ssr, sr_before);
    assert_eq!(sgr, 0x40);
    assert_eq!(
        h.cpu.sr() & (sr::MD | sr::RB | sr::BL),
        sr::MD | sr::RB | sr::BL
    );
    assert_eq!(h.cpu.pc(), VBR + 0x100);
    h.steps(3); // rte, its slot, then the instruction after the trap
    assert_eq!(h.cpu.sr(), sr_before);
    assert_eq!(h.r(2), 3);
}

#[test]
fn an_exception_switches_to_bank_one_and_rte_switches_back() {
    let h = Harness::new(&[mov_i(0, 11), trapa(0), NOP]);
    h.load(0x100, &[mov_i(0, 22), RTE, NOP]);
    h.steps(3);
    assert_eq!(h.r(0), 22, "bank 1's R0 in the handler");
    let (b0, b1) = h.cpu.banks();
    assert_eq!(b0[0], 11);
    assert_eq!(b1[0], 22);
    h.steps(2);
    assert_eq!(h.r(0), 11, "the interrupted code's R0 is untouched");
}

#[test]
fn ldc_and_stc_reach_the_other_bank() {
    let h = Harness::new(&[mov_i(3, 77), ldc_bank(3, 5), stc_bank(5, 4)]);
    h.steps(3);
    let (_, b1) = h.cpu.banks();
    assert_eq!(b1[5], 77, "with RB = 0 the other bank is bank 1");
    assert_eq!(h.r(4), 77);
}

#[test]
fn ldc_sr_moves_the_visible_bank() {
    // SR.RB set while privileged makes R0_BANK1 the visible R0 (§2.2.2), and
    // STC SR reads the value back masked to the writable bits.
    let h = Harness::new(&[mov_i(0, 5), ldc_sr(1), mov_i(0, 6), stc_sr(2)]);
    h.cpu.set_reg(1, !sr::BL);
    h.steps(4);
    let (b0, b1) = h.cpu.banks();
    assert_eq!((b0[0], b1[0]), (5, 6));
    assert_eq!(h.r(2), sr::WRITABLE & !sr::BL);
}

#[test]
fn the_undefined_code_is_a_general_illegal_instruction() {
    let h = Harness::new(&[0xfffd]);
    h.steps(1);
    assert_eq!(h.expevt(), exc::ILLEGAL);
    assert_eq!(h.cpu.control()[3], START);
}

#[test]
fn a_privileged_instruction_in_user_mode_is_illegal_but_gbr_is_not() {
    let h = Harness::new(&[0x401e /* ldc r0,gbr */, stc_sr(1)]);
    // User mode, running from P0 (physical PROG).
    h.cpu.set_sr(0);
    h.cpu.set_pc(PROG);
    h.steps(1);
    assert_eq!(h.cpu.pc(), PROG + 2, "LDC Rm,GBR is unprivileged");
    h.steps(1);
    assert_eq!(h.expevt(), exc::ILLEGAL);
    assert_eq!(h.cpu.control()[3], PROG + 2);
}

#[test]
fn user_mode_cannot_reach_p1() {
    let h = Harness::new(&[movl_pc(1, 0), movl_l(1, 2), 0, 0]);
    h.put32(PROG + 4, p1(DATA));
    h.cpu.set_sr(0);
    h.cpu.set_pc(PROG);
    h.steps(2);
    assert_eq!(h.expevt(), exc::ADDR_READ);
    assert_eq!(h.cpu.ctl().tea, p1(DATA));
    assert_eq!(h.cpu.ctl().pteh & 0xffff_fc00, p1(DATA) & 0xffff_fc00);
}

#[test]
fn a_misaligned_store_is_a_write_address_error() {
    let h = Harness::new(&[mov_i(1, 2), movl_s(0, 1)]);
    h.steps(2);
    assert_eq!(h.expevt(), exc::ADDR_WRITE);
    assert_eq!(h.cpu.ctl().tea, 2);
}

#[test]
fn fpu_disabled_raises_the_general_or_slot_code() {
    let h = Harness::new(&[fadd(0, 1), bra(4), fadd(0, 1)]);
    h.cpu.set_sr(sr::MD | sr::FD);
    h.steps(1);
    assert_eq!(h.expevt(), exc::FPU_DISABLE);
    // And in a slot.
    let h = Harness::new(&[bra(4), fadd(0, 1)]);
    h.cpu.set_sr(sr::MD | sr::FD);
    h.steps(2);
    assert_eq!(h.expevt(), exc::SLOT_FPU_DISABLE);
    assert_eq!(h.cpu.control()[3], START);
    // LDS to FPSCR counts as an FPU instruction (§5.6.2 note).
    let h = Harness::new(&[lds_fpscr(0)]);
    h.cpu.set_sr(sr::MD | sr::FD);
    h.steps(1);
    assert_eq!(h.expevt(), exc::FPU_DISABLE);
}

#[test]
fn an_exception_with_bl_set_is_a_manual_reset() {
    let h = Harness::new(&[0xfffd]);
    h.cpu.set_sr(sr::MD | sr::BL);
    h.steps(1);
    assert_eq!(h.expevt(), exc::MANUAL_RESET);
    assert_eq!(h.cpu.pc(), START);
    assert_eq!(h.cpu.control()[1], 0, "VBR reset");
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

#[test]
fn an_interrupt_needs_a_level_above_imask_and_bl_clear() {
    let h = Harness::new(&[NOP, NOP, NOP, NOP]);
    h.cpu.set_sr(sr::MD | (5 << 4));
    h.cpu.set_interrupt(5, 0x580);
    h.steps(1);
    assert_eq!(h.cpu.pc(), START + 2, "level 5 does not beat IMASK 5");
    h.cpu.set_interrupt(6, 0x580);
    h.cpu.set_sr(sr::MD | sr::BL | (5 << 4));
    h.steps(1);
    assert_eq!(h.cpu.pc(), START + 4, "BL holds it off");
    h.cpu.set_sr(sr::MD | (5 << 4));
    h.steps(1);
    assert_eq!(h.cpu.pc(), VBR + 0x600);
    assert_eq!(h.cpu.ctl().intevt, 0x580);
    assert_eq!(h.cpu.control()[3], START + 4, "SPC = the next instruction");
    assert_eq!(
        h.cpu.sr() & sr::IMASK,
        5 << 4,
        "CPUOPM.INTMU is clear at reset, so IMASK is not raised"
    );
    assert_eq!(h.expevt(), 0, "an interrupt does not touch EXPEVT");
}

#[test]
fn intmu_raises_imask_to_the_accepted_level() {
    let h = Harness::new(&[movl_pc(1, 1), movl_s(2, 1), NOP, NOP, 0, 0]);
    h.put32(PROG + 8, reg::CPUOPM);
    h.cpu.set_reg(2, 0x28);
    h.steps(2);
    assert_eq!(h.cpu.ctl().cpuopm, 0x3e8);
    h.cpu.set_interrupt(9, 0x620);
    h.steps(1);
    assert_eq!(h.cpu.sr() & sr::IMASK, 9 << 4);
}

#[test]
fn nmi_ignores_imask() {
    let h = Harness::new(&[NOP, NOP]);
    h.cpu.set_sr(sr::MD | sr::IMASK);
    h.cpu.raise_nmi();
    h.steps(1);
    assert_eq!(h.cpu.ctl().intevt, exc::NMI);
    assert_eq!(h.cpu.pc(), VBR + 0x600);
    assert!(!h.cpu.lines().nmi_pending(), "the edge is consumed");
}

#[test]
fn sleep_waits_and_an_interrupt_wakes_it_even_with_bl_set() {
    let h = Harness::new(&[SLEEP, mov_i(1, 1)]);
    h.cpu.set_sr(sr::MD | sr::BL);
    h.steps(1);
    assert!(h.cpu.is_sleeping());
    let pc = h.cpu.pc();
    h.cpu.run(1000);
    assert!(h.cpu.is_sleeping(), "nothing to wake it");
    assert_eq!(h.cpu.pc(), pc);
    h.cpu.set_interrupt(3, 0x400);
    h.steps(1);
    assert!(!h.cpu.is_sleeping());
    assert_eq!(h.cpu.pc(), VBR + 0x600);
    assert_eq!(h.cpu.control()[3], START + 2, "returns to after the SLEEP");
}

#[test]
fn the_request_cell_is_exported_and_drives_the_core() {
    let h = Harness::new(&[NOP, NOP]);
    let Some(Export::Cell(cell)) = h.cpu.export(ExportId::INTERRUPT_REQUEST) else {
        panic!("no interrupt-request cell");
    };
    cell.store(pack_request(4, 0x7a0), Ordering::Release);
    h.steps(1);
    assert_eq!(h.cpu.ctl().intevt, 0x7a0);
}

// ---------------------------------------------------------------------------
// Arithmetic
// ---------------------------------------------------------------------------

#[test]
fn div1_divides_64_by_32_as_the_manuals_example_2() {
    // §10.1.17 example 2: R1:R2 (64 bits) / R0 (32 bits) = R2, unsigned.
    let mut prog = Vec::new();
    prog.push(DIV0U);
    for _ in 0..32 {
        prog.push(rotcl(2));
        prog.push(div1(0, 1));
    }
    prog.push(rotcl(2));
    let h = Harness::new(&prog);
    // 0x0000_0001_2345_6789 / 0x1234 = 0x0001_0004_cd9c (fits: R1 < R0).
    h.cpu.set_reg(1, 0x0000_0001);
    h.cpu.set_reg(2, 0x2345_6789);
    h.cpu.set_reg(0, 0x0001_2345);
    h.steps(prog.len());
    let n = 0x0000_0001_2345_6789u64;
    assert_eq!(h.r(2), (n / 0x0001_2345) as u32);
}

#[test]
fn div1_signed_division_follows_example_4() {
    // §10.1.17 example 4: R2 (32 bits) / R0 (32 bits) = R2, signed.
    let mut prog = vec![
        mov(2, 3),
        rotcl(3),
        subc(1, 1),
        rr(2, 3, 3, 0xa), // xor r3,r3
        subc(3, 2),
        div0s(0, 1),
    ];
    for _ in 0..32 {
        prog.push(rotcl(2));
        prog.push(div1(0, 1));
    }
    prog.push(rotcl(2));
    prog.push(addc(3, 2));
    for (dividend, divisor) in [(-1000i32, 7i32), (1000, -7), (-1000, -7), (123_456, 789)] {
        let h = Harness::new(&prog);
        h.cpu.set_reg(2, dividend as u32);
        h.cpu.set_reg(0, divisor as u32);
        h.steps(prog.len());
        assert_eq!(h.r(2) as i32, dividend / divisor, "{dividend} / {divisor}");
    }
}

#[test]
fn carries_borrows_and_overflows_land_in_t() {
    let h = Harness::new(&[
        SETT,
        addc(1, 0),
        CLRT,
        addv(3, 2),
        subc(5, 4),
        subv(7, 6),
        negc(9, 8),
    ]);
    h.cpu.set_reg(0, 0xffff_ffff);
    h.cpu.set_reg(1, 0);
    h.steps(2);
    assert_eq!((h.r(0), h.t()), (0, true), "0xffffffff + 0 + 1 carries");
    h.cpu.set_reg(2, 0x7fff_ffff);
    h.cpu.set_reg(3, 1);
    h.steps(2);
    assert_eq!((h.r(2), h.t()), (0x8000_0000, true), "signed overflow");
    h.cpu.set_reg(4, 0);
    h.cpu.set_reg(5, 1);
    h.steps(1);
    // T was left set by ADDV, so 0 - 1 - 1.
    assert_eq!((h.r(4), h.t()), (0xffff_fffe, true), "0 - 1 - T borrows");
    h.cpu.set_reg(6, 0x8000_0000);
    h.cpu.set_reg(7, 1);
    h.steps(1);
    assert_eq!((h.r(6), h.t()), (0x7fff_ffff, true), "signed underflow");
    h.cpu.set_reg(9, 0);
    h.steps(1);
    // T was left set by SUBV, so 0 - 0 - 1.
    assert_eq!((h.r(8), h.t()), (0xffff_ffff, true));
}

#[test]
fn shad_and_shld_match_the_manuals_examples() {
    // §10.1.61: SHAD R1,R2 with R1 = H'FFFFFFEC, R2 = H'80180000 gives
    // H'FFFFF801; SHAD R3,R4 with R3 = H'14, R4 = H'FFFFF801 gives H'80100000.
    let h = Harness::new(&[shad(1, 2), shad(3, 4), shld(5, 6), shld(7, 8)]);
    h.cpu.set_reg(1, 0xffff_ffec);
    h.cpu.set_reg(2, 0x8018_0000);
    h.cpu.set_reg(3, 0x14);
    h.cpu.set_reg(4, 0xffff_f801);
    h.cpu.set_reg(5, 0xffff_ffec);
    h.cpu.set_reg(6, 0x8018_0000);
    h.cpu.set_reg(7, 0xffff_ffe0); // -32: the "all zeros" case
    h.cpu.set_reg(8, 0x1234_5678);
    h.steps(4);
    assert_eq!(h.r(2), 0xffff_f801);
    assert_eq!(h.r(4), 0x8010_0000);
    assert_eq!(h.r(6), 0x0000_0801);
    assert_eq!(h.r(8), 0);
}

#[test]
fn mac_l_accumulates_and_saturates_at_48_bits() {
    // §10.1.28's example tables: H'1234ABCD * H'0123ABCD + H'5678EF01 *
    // H'4567DEF0, accumulated from zero.
    let h = Harness::new(&[CLRMAC, mac_l(0, 1), mac_l(0, 1)]);
    let (tbl_m, tbl_n) = (DATA, DATA + 0x10);
    h.put32(tbl_m, 0x1234_abcd);
    h.put32(tbl_m + 4, 0x5678_ef01);
    h.put32(tbl_n, 0x0123_abcd);
    h.put32(tbl_n + 4, 0x4567_def0);
    h.cpu.set_reg(0, p1(tbl_m));
    h.cpu.set_reg(1, p1(tbl_n));
    h.steps(3);
    let want = 0x1234_abcdi64 * 0x0123_abcd + 0x5678_ef01i64 * 0x4567_def0;
    let (mach, macl, _) = h.cpu.system();
    assert_eq!(((u64::from(mach) << 32) | u64::from(macl)) as i64, want);
    assert_eq!(h.r(0), p1(tbl_m) + 8);
    // With S set, the same sum saturates at 2^47 - 1.
    let h = Harness::new(&[SETS, CLRMAC, mac_l(0, 1), mac_l(0, 1)]);
    h.put32(tbl_m, 0x7fff_ffff);
    h.put32(tbl_m + 4, 0x7fff_ffff);
    h.put32(tbl_n, 0x7fff_ffff);
    h.put32(tbl_n + 4, 0x7fff_ffff);
    h.cpu.set_reg(0, p1(tbl_m));
    h.cpu.set_reg(1, p1(tbl_n));
    h.steps(4);
    let (mach, macl, _) = h.cpu.system();
    assert_eq!((mach & 0xffff, macl), (0x7fff, 0xffff_ffff));
}

#[test]
fn mac_w_accumulates_and_saturates_at_32_bits() {
    let h = Harness::new(&[CLRMAC, mac_w(0, 1), mac_w(0, 1)]);
    h.put16(DATA, 0x1234);
    h.put16(DATA + 2, 0x5678);
    h.put16(DATA + 0x10, 0x0123);
    h.put16(DATA + 0x12, 0x4567);
    h.cpu.set_reg(0, p1(DATA));
    h.cpu.set_reg(1, p1(DATA + 0x10));
    h.steps(3);
    let (_, macl, _) = h.cpu.system();
    assert_eq!(macl as i32, 0x1234 * 0x0123 + 0x5678 * 0x4567);
    let h = Harness::new(&[SETS, mac_w(0, 1)]);
    h.put16(DATA, 0x7fff);
    h.put16(DATA + 0x10, 0x7fff);
    h.cpu.set_mac(0, 0x7fff_0000);
    h.cpu.set_reg(0, p1(DATA));
    h.cpu.set_reg(1, p1(DATA + 0x10));
    h.steps(2);
    let (mach, macl, _) = h.cpu.system();
    assert_eq!(macl, 0x7fff_ffff);
    assert_eq!(mach & 1, 1, "overflow sets MACH's LSB");
}

#[test]
fn post_increment_onto_itself_keeps_the_loaded_value() {
    // §10.1.30: `if (n!=m) R[m] += 4`.
    let h = Harness::new(&[movl_p(1, 1), movl_m(2, 2)]);
    h.put32(DATA, 0xdead_beef);
    h.cpu.set_reg(1, p1(DATA));
    h.cpu.set_reg(2, p1(DATA + 0x20));
    h.steps(2);
    assert_eq!(h.r(1), 0xdead_beef);
    assert_eq!(h.r(2), p1(DATA + 0x1c));
    assert_eq!(
        h.get32(DATA + 0x1c),
        p1(DATA + 0x20),
        "the old value was stored"
    );
}

#[test]
fn tas_tests_and_sets_bit_seven() {
    let h = Harness::new(&[tas(1), tas(1)]);
    h.cpu.set_reg(1, p1(DATA));
    h.steps(1);
    assert!(h.t());
    assert_eq!(h.ram.read_u8(u64::from(DATA)).unwrap(), 0x80);
    h.steps(1);
    assert!(!h.t());
}

#[test]
fn movli_movco_succeed_alone_and_fail_after_an_interrupt() {
    let h = Harness::new(&[
        movli(1),
        add_i(0, 1),
        movco(1),
        movli(1),
        add_i(0, 1),
        movco(1),
    ]);
    h.load(0x600, &[RTE, NOP]);
    h.put32(DATA, 41);
    h.cpu.set_reg(1, p1(DATA));
    h.steps(3);
    assert!(h.t());
    assert_eq!(h.get32(DATA), 42);
    h.steps(1); // movli: LDST set
    h.cpu.set_interrupt(1, 0x400);
    h.steps(1); // interrupt taken, LDST cleared
    h.cpu.set_interrupt(0, 0);
    h.steps(2); // rte, slot
    h.steps(2); // add, movco
    assert!(!h.t(), "the link was broken");
    assert_eq!(h.get32(DATA), 42);
}

#[test]
fn movua_loads_from_any_address() {
    let h = Harness::new(&[movua(1), movua_p(2)]);
    for (i, b) in [0x11u8, 0x22, 0x33, 0x44, 0x55, 0x66].iter().enumerate() {
        h.ram.write_u8(u64::from(DATA) + i as u64, *b).unwrap();
    }
    h.cpu.set_reg(1, p1(DATA + 1));
    h.cpu.set_reg(2, p1(DATA + 2));
    h.steps(2);
    assert_eq!(h.r(0), 0x6655_4433);
    assert_eq!(h.r(2), p1(DATA + 6));
}

// ---------------------------------------------------------------------------
// The FPU
// ---------------------------------------------------------------------------

/// A single's bits.
fn fb(v: f32) -> u32 {
    v.to_bits()
}

/// An FPU harness with DN = 0, round to nearest, single precision.
fn fpu(program: &[u16]) -> Harness {
    let h = Harness::new(program);
    h.cpu.set_fpscr(0);
    h
}

#[test]
fn fadd_fmul_fdiv_compute_ieee_results() {
    let h = fpu(&[fadd(1, 0), fmul(3, 2), fdiv(5, 4)]);
    h.cpu.set_fpr(0, 0, fb(1.5));
    h.cpu.set_fpr(0, 1, fb(2.25));
    h.cpu.set_fpr(0, 2, fb(3.0));
    h.cpu.set_fpr(0, 3, fb(-0.5));
    h.cpu.set_fpr(0, 4, fb(1.0));
    h.cpu.set_fpr(0, 5, fb(3.0));
    h.steps(3);
    let f = h.cpu.fpr()[0];
    assert_eq!(f[0], fb(3.75));
    assert_eq!(f[2], fb(-1.5));
    assert_eq!(f[4], fb(1.0 / 3.0));
    assert_ne!(h.cpu.fpscr() & 0x1000, 0, "1/3 is inexact");
}

#[test]
fn round_to_zero_truncates() {
    let h = fpu(&[fdiv(1, 0)]);
    h.cpu.set_fpscr(1);
    h.cpu.set_fpr(0, 0, fb(1.0));
    h.cpu.set_fpr(0, 1, fb(3.0));
    h.steps(1);
    assert_eq!(
        h.cpu.fpr()[0][0],
        fb(1.0 / 3.0) - 1,
        "nearest rounds up here; RZ does not"
    );
}

#[test]
fn division_by_zero_gives_infinity_and_sets_z() {
    let h = fpu(&[fdiv(1, 0)]);
    h.cpu.set_fpr(0, 0, fb(-2.0));
    h.cpu.set_fpr(0, 1, 0);
    h.steps(1);
    assert_eq!(h.cpu.fpr()[0][0], 0xff80_0000);
    assert_eq!(h.cpu.fpscr() & 0x8020, 0x8020, "cause Z and flag Z");
}

#[test]
fn an_enabled_exception_traps_and_leaves_the_destination() {
    let h = fpu(&[fdiv(1, 0)]);
    h.cpu.set_fpscr(0x400); // enable Z
    h.cpu.set_fpr(0, 0, fb(2.0));
    h.cpu.set_fpr(0, 1, 0);
    h.steps(1);
    assert_eq!(h.expevt(), exc::FPU);
    assert_eq!(h.cpu.fpr()[0][0], fb(2.0));
    assert_eq!(h.cpu.control()[3], START, "re-execution type");
}

#[test]
fn nans_follow_the_sh4_convention() {
    // A NaN with the fraction MSB *set* is signaling on this FPU (§6.2.2):
    // invalid, and the result is the one qNaN, H'7FBFFFFF.
    let h = fpu(&[fadd(1, 0), fadd(3, 2), fcmp_eq(5, 4), fcmp_gt(5, 4)]);
    h.cpu.set_fpr(0, 0, fb(1.0));
    h.cpu.set_fpr(0, 1, 0x7fc0_0000);
    h.cpu.set_fpr(0, 2, fb(1.0));
    h.cpu.set_fpr(0, 3, 0x7f80_0001);
    h.cpu.set_fpr(0, 4, fb(1.0));
    h.cpu.set_fpr(0, 5, 0x7f80_0001);
    h.steps(1);
    assert_eq!(h.cpu.fpr()[0][0], 0x7fbf_ffff);
    assert_ne!(h.cpu.fpscr() & 0x10000, 0, "sNaN is invalid");
    h.steps(1);
    assert_eq!(h.cpu.fpr()[0][2], 0x7fbf_ffff);
    assert_eq!(h.cpu.fpscr() & 0x10000, 0, "qNaN is not");
    h.steps(1);
    assert!(!h.t());
    assert_eq!(
        h.cpu.fpscr() & 0x10000,
        0,
        "FCMP/EQ with a qNaN is just unequal"
    );
    h.steps(1);
    assert_ne!(h.cpu.fpscr() & 0x10000, 0, "FCMP/GT with a qNaN is invalid");
}

#[test]
fn a_denormal_operand_with_dn_clear_is_an_fpu_error() {
    let h = fpu(&[fadd(1, 0)]);
    h.cpu.set_fpr(0, 0, fb(1.0));
    h.cpu.set_fpr(0, 1, 1);
    h.steps(1);
    assert_eq!(h.expevt(), exc::FPU);
    assert_ne!(h.cpu.fpscr() & 0x20000, 0, "cause E");
    // With DN set it is a zero.
    let h = fpu(&[fadd(1, 0)]);
    h.cpu.set_fpscr(fpscr::DN);
    h.cpu.set_fpr(0, 0, fb(1.0));
    h.cpu.set_fpr(0, 1, 1);
    h.steps(1);
    assert_eq!(h.cpu.fpr()[0][0], fb(1.0));
}

#[test]
fn fmac_rounds_once() {
    // (1 + 2^-12)^2 - 1 = 2^-11 + 2^-24 exactly; rounding the product first
    // would lose the 2^-24 (it is half an ulp of 1, and ties go to even).
    let x = fb(1.0) + (1 << 11);
    let h = fpu(&[fmac(1, 2)]);
    h.cpu.set_fpr(0, 0, x);
    h.cpu.set_fpr(0, 1, x);
    h.cpu.set_fpr(0, 2, fb(-1.0));
    h.steps(1);
    let want = (2.0f64.powi(-11) + 2.0f64.powi(-24)) as f32;
    assert_eq!(h.cpu.fpr()[0][2], fb(want));
}

#[test]
fn float_ftrc_and_fsqrt() {
    let h = fpu(&[
        lds_fpul(1),
        float(0),
        fsqrt(0),
        ftrc(0),
        sts_fpul(2),
        ftrc(3),
        sts_fpul(4),
    ]);
    h.cpu.set_reg(1, 49);
    h.cpu.set_fpr(0, 3, fb(-3.0e9));
    h.steps(5);
    assert_eq!(h.cpu.fpr()[0][0], fb(7.0));
    assert_eq!(h.r(2), 7);
    h.steps(2);
    assert_eq!(h.r(4), 0x8000_0000, "out of range saturates");
    assert_ne!(h.cpu.fpscr() & 0x10000, 0, "and is invalid");
}

#[test]
fn double_precision_and_conversions() {
    let h = fpu(&[fadd(2, 0), fcnvds(0), fcnvsd(4)]);
    h.cpu.set_fpscr(fpscr::PR);
    let put_d = |n: usize, v: f64| {
        let b = v.to_bits();
        h.cpu.set_fpr(0, n, (b >> 32) as u32);
        h.cpu.set_fpr(0, n + 1, b as u32);
    };
    put_d(0, 1.25);
    put_d(2, 2.5);
    h.steps(2);
    let f = h.cpu.fpr()[0];
    assert_eq!((u64::from(f[0]) << 32) | u64::from(f[1]), 3.75f64.to_bits());
    assert_eq!(h.cpu.fpul(), fb(3.75));
    h.steps(1);
    let f = h.cpu.fpr()[0];
    assert_eq!((u64::from(f[4]) << 32) | u64::from(f[5]), 3.75f64.to_bits());
}

#[test]
fn fipr_and_ftrv() {
    let h = fpu(&[fipr(4, 0), FRCHG, ftrv(8)]);
    for i in 0..4 {
        h.cpu.set_fpr(0, i, fb(i as f32 + 1.0)); // FV0 = 1 2 3 4
        h.cpu.set_fpr(0, 4 + i, fb(2.0)); // FV4 = 2 2 2 2
    }
    h.steps(1);
    assert_eq!(h.cpu.fpr()[0][3], fb(20.0));
    assert_ne!(h.cpu.fpscr() & 0x1000, 0, "always inexact");
    // FTRV: XMTRX = the identity scaled by 3, FV8 = 1 2 3 4. After FRCHG, FR
    // is bank 1 and XMTRX is bank 0.
    for i in 0..16 {
        h.cpu.set_fpr(0, i, if i % 5 == 0 { fb(3.0) } else { 0 });
    }
    for i in 0..4 {
        h.cpu.set_fpr(1, 8 + i, fb(i as f32 + 1.0));
    }
    h.steps(2);
    let f = h.cpu.fpr()[1];
    assert_eq!(
        [f[8], f[9], f[10], f[11]],
        [fb(3.0), fb(6.0), fb(9.0), fb(12.0)]
    );
}

#[test]
fn fsca_gives_sine_and_cosine_of_a_fraction_of_a_turn() {
    let h = fpu(&[lds_fpul(1), fsca(2)]);
    h.cpu.set_reg(1, 0x4000); // a quarter turn
    h.steps(2);
    let f = h.cpu.fpr()[0];
    assert_eq!(f[2], fb(1.0));
    assert!(f32::from_bits(f[3]).abs() < 1e-6);
}

#[test]
fn pair_moves_follow_endianness_and_pr() {
    // Little-endian, SZ = 1, PR = 0: FR(2i) at the lower address.
    let h = fpu(&[FSCHG, fmov_s(0, 1), fmov_l(1, 2)]);
    h.cpu.set_fpr(0, 0, 0x1111_1111);
    h.cpu.set_fpr(0, 1, 0x2222_2222);
    h.cpu.set_reg(1, p1(DATA));
    h.steps(2);
    assert_eq!(h.get32(DATA), 0x1111_1111);
    assert_eq!(h.get32(DATA + 4), 0x2222_2222);
    h.steps(1);
    assert_eq!(&h.cpu.fpr()[0][2..4], &[0x1111_1111, 0x2222_2222]);
    // Little-endian with PR = 1: the pair is one little-endian double.
    let h = fpu(&[fmov_s(0, 1)]);
    h.cpu.set_fpscr(fpscr::PR | fpscr::SZ);
    h.cpu.set_fpr(0, 0, 0x1111_1111);
    h.cpu.set_fpr(0, 1, 0x2222_2222);
    h.cpu.set_reg(1, p1(DATA));
    h.steps(1);
    assert_eq!(h.get32(DATA), 0x2222_2222);
    assert_eq!(h.get32(DATA + 4), 0x1111_1111);
}

// ---------------------------------------------------------------------------
// The MMU and P4
// ---------------------------------------------------------------------------

/// Write a P4 register by program: `mov.l r2,@r1` with r1/r2 preset.
fn poke(h: &Harness, addr: u32, value: u32) {
    let pc = h.cpu.pc();
    h.load(pc & 0x1fff_ffff, &[movl_s(2, 1)]);
    h.cpu.set_reg(1, addr);
    h.cpu.set_reg(2, value);
    h.steps(1);
}

/// Execute one instruction at the current PC.
fn exec1(h: &Harness, insn: u16) {
    let pc = h.cpu.pc();
    h.load(pc & 0x1fff_ffff, &[insn]);
    h.steps(1);
}

/// Read a P4 register by program.
fn peek(h: &Harness, addr: u32) -> u32 {
    let pc = h.cpu.pc();
    h.load(pc & 0x1fff_ffff, &[movl_l(1, 2)]);
    h.cpu.set_reg(1, addr);
    h.steps(1);
    h.r(2)
}

#[test]
fn the_cores_own_registers_answer_in_p4() {
    let h = Harness::new(&[]);
    poke(&h, reg::TTB, 0x1234_5678);
    assert_eq!(peek(&h, reg::TTB), 0x1234_5678);
    poke(&h, reg::EXPEVT, 0xffff);
    assert_eq!(peek(&h, reg::EXPEVT), 0xfff);
    assert_eq!(peek(&h, reg::PVR), super::DEFAULT_PVR);
    poke(&h, reg::MMUCR, 0xffff_ffff);
    assert_eq!(peek(&h, reg::MMUCR), 0xfcfc_ff01, "TI reads as zero");
}

#[test]
fn other_p4_addresses_go_to_the_bus() {
    let h = Harness::new(&[]);
    poke(&h, PERIPH + 0x10, 0xcafe_f00d);
    let mut b = [0u8; 4];
    for (k, slot) in b.iter_mut().enumerate() {
        *slot = h.periph.read_u8(0x10 + k as u64).unwrap();
    }
    assert_eq!(u32::from_le_bytes(b), 0xcafe_f00d);
    assert_eq!(peek(&h, PERIPH + 0x10), 0xcafe_f00d);
}

#[test]
fn ldtlb_then_at_translates_p0() {
    // Map virtual 0x0040_0000 (4 KB page) to physical DATA, ASID 3, PR = 11,
    // D = 1, V = 1.
    let h = Harness::new(&[]);
    poke(&h, reg::PTEH, 0x0040_0003);
    poke(&h, reg::PTEL, DATA | 0x100 | 0x10 | 0x60 | 0x4);
    exec1(&h, LDTLB);
    poke(&h, reg::MMUCR, 1);
    h.put32(DATA + 0x24, 0x5555_aaaa);
    assert_eq!(peek(&h, 0x0040_0024), 0x5555_aaaa);
    // And the memory-mapped UTLB data array shows the entry.
    let urc = 0;
    let data = peek(&h, 0xf700_0000 | (urc << 8));
    assert_eq!(data & 0x1fff_fc00, DATA);
    assert_ne!(data & 0x100, 0);
}

#[test]
fn a_tlb_miss_vectors_to_0x400_and_fills_tea_and_pteh() {
    let h = Harness::new(&[]);
    poke(&h, reg::PTEH, 0x0000_0007);
    poke(&h, reg::MMUCR, 1);
    let pc = h.cpu.pc();
    h.load(pc & 0x1fff_ffff, &[movl_s(2, 1)]);
    h.cpu.set_reg(1, 0x0012_3458);
    h.steps(1);
    assert_eq!(h.expevt(), exc::TLB_MISS_WRITE);
    assert_eq!(h.cpu.pc(), VBR + 0x400);
    assert_eq!(h.cpu.ctl().tea, 0x0012_3458);
    assert_eq!(h.cpu.ctl().pteh, 0x0012_3400 | 7, "VPN set, ASID kept");
}

#[test]
fn a_clean_page_raises_initial_page_write_and_protection_is_checked() {
    let h = Harness::new(&[]);
    // A 1 KB page, PR = 10 (read-only for both modes), D = 0.
    poke(&h, reg::PTEH, 0x0040_0000);
    poke(&h, reg::PTEL, DATA | 0x100 | 0x40);
    exec1(&h, LDTLB);
    poke(&h, reg::MMUCR, 1);
    poke(&h, 0x0040_0000, 1);
    assert_eq!(h.expevt(), exc::TLB_PROT_WRITE);
    // Make it writable but clean, in the same UTLB entry: MMUCR = 1 again
    // puts URC back at zero.
    h.cpu.set_sr(sr::MD);
    h.cpu.set_pc(START + 0x40);
    poke(&h, reg::MMUCR, 1);
    poke(&h, reg::PTEL, DATA | 0x100 | 0x60);
    exec1(&h, LDTLB);
    poke(&h, 0x0040_0000, 1);
    assert_eq!(h.expevt(), exc::INITIAL_WRITE);
}

#[test]
fn an_instruction_fetch_refills_the_itlb_from_the_utlb() {
    // Code at physical 0x2000 mapped at virtual 0x0010_0000.
    let h = Harness::new(&[]);
    h.load(0x2000, &[mov_i(5, 42), NOP]);
    poke(&h, reg::PTEH, 0x0010_0000);
    poke(&h, reg::PTEL, 0x2000 | 0x100 | 0x10 | 0x60 | 0x4);
    exec1(&h, LDTLB);
    poke(&h, reg::MMUCR, 1);
    h.cpu.set_pc(0x0010_0000);
    h.steps(1);
    assert_eq!(h.r(5), 42);
    let itlb = h.cpu.mmu().itlb;
    assert!(itlb.iter().any(|e| e.v && e.vpn == 0x0010_0000));
}

#[test]
fn the_pmb_maps_p1_in_32_bit_mode() {
    let h = Harness::new(&[]);
    // PMB entry 0: virtual 0x8000_0000 (16 MB) -> physical 0x0000_0000;
    // entry 1: virtual 0xa000_0000 -> physical 0x0000_0000.
    poke(&h, 0xf610_0000, 0x8000_0100);
    poke(&h, 0xf710_0000, 0x0000_0100);
    poke(&h, 0xf610_0100, 0xa000_0100);
    poke(&h, 0xf710_0100, 0x0000_0100);
    poke(&h, reg::PASCR, 0x8000_0000);
    h.put32(DATA, 0x0bad_cafe);
    assert_eq!(peek(&h, p1(DATA)), 0x0bad_cafe);
    // Unmapped P1 address: a reset with EXPEVT = H'140.
    let pc = h.cpu.pc();
    h.load(pc & 0x1fff_ffff, &[movl_l(1, 2)]);
    h.cpu.set_reg(1, 0x9000_0000);
    h.steps(1);
    assert_eq!(h.expevt(), exc::TLB_MULTI);
    assert_eq!(h.cpu.ctl().tea, 0x9000_0000);
}

#[test]
fn the_store_queue_flushes_32_bytes_through_qacr() {
    let h = Harness::new(&[]);
    poke(&h, reg::QACR0, 0);
    for i in 0..8 {
        poke(&h, 0xe000_0000 + 4 * i, 0x100 + i);
    }
    let pc = h.cpu.pc();
    h.load(pc & 0x1fff_ffff, &[pref(1)]);
    h.cpu.set_reg(1, 0xe000_0000 | DATA);
    h.steps(1);
    for i in 0..8 {
        assert_eq!(h.get32(DATA + 4 * i), 0x100 + i);
    }
    // And the queue reads back at H'FF001000.
    assert_eq!(peek(&h, reg::SQ_READ + 8), 0x102);
}

// ---------------------------------------------------------------------------
// Byte order
// ---------------------------------------------------------------------------

#[test]
fn a_big_endian_core_runs_the_same_program() {
    let h = Harness::with(
        Config::new().with_endian(Endian::Big),
        &[movw_l(1, 2), movua(3)],
    );
    h.put16(DATA, 0x8001);
    for (i, b) in [0x11u8, 0x22, 0x33, 0x44, 0x55].iter().enumerate() {
        h.ram
            .write_u8(u64::from(DATA) + 0x10 + i as u64, *b)
            .unwrap();
    }
    h.cpu.set_reg(1, p1(DATA));
    h.cpu.set_reg(3, p1(DATA + 0x11));
    h.steps(2);
    assert_eq!(h.r(2), 0xffff_8001);
    assert_eq!(h.r(0), 0x2233_4455);
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

#[test]
fn hold_keeps_the_core_in_reset_and_release_starts_at_the_reset_address() {
    let cfg = Config {
        held: true,
        ..Config::new()
    };
    let h = Harness::with(cfg, &[mov_i(1, 1)]);
    h.load(0x3000, &[mov_i(1, 2)]);
    h.steps(3);
    assert_eq!(h.r(1), 0, "held: nothing runs");
    let Some(Export::Cell(at)) = h.cpu.export(ExportId::RESET_ADDRESS) else {
        panic!("no reset-address cell");
    };
    at.store(0xa000_3000, Ordering::Release);
    h.cpu.lines().set_hold(false);
    h.steps(1);
    assert_eq!(h.r(1), 2);
}

#[test]
fn an_unknown_property_is_refused() {
    assert!(Cpu::from_props(&Props::new().with("endain", "big")).is_err());
    let props = Props::new()
        .with("endian", "big")
        .with("reset", 0xa000_0000u64);
    let cpu = Cpu::from_props(&props).unwrap();
    assert_eq!(cpu.config().endian, Endian::Big);
}

#[test]
fn trapa_can_leave_the_core_as_a_syscall_exit() {
    let h = Harness::new(&[trapa(5), NOP]);
    h.cpu
        .set_exit_mask(ExitMask::NONE.with(ExitReason::SYSCALL));
    let (_, exit) = h.cpu.step_to_exit();
    let exit = exit.expect("an exit");
    assert_eq!(exit.reason, ExitReason::SYSCALL);
    assert_eq!(exit.pc, u64::from(START));
    assert_eq!(h.cpu.pc(), START + 2, "resumes after the trap");
    assert_eq!(h.cpu.ctl().tra, 5 << 2);
}

#[test]
fn a_listing_of_a_program_reads_back() {
    let h = Harness::new(&[mov_i(1, 10), dt(1), bf(-3), RTS, NOP]);
    let text: Vec<_> = h
        .cpu
        .disassemble(START, 5)
        .iter()
        .map(|d| d.text.clone())
        .collect();
    assert_eq!(
        text,
        ["mov #0xa, r1", "dt r1", "bf 0xa0001002", "rts", "nop"]
    );
    assert!(h.cpu.disassemble(START, 5)[3].delay_slot);
    assert_eq!(
        disasm::format_word(
            0xf020,
            0,
            Mode {
                pr: true,
                sz: false
            }
        ),
        "fadd dr2, dr0"
    );
}

/// A processor's state as a snapshot chunk.
fn save_bytes(cpu: &Cpu) -> Result<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name)?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("cpu", CLASS.name, CLASS.version)?;
        cpu.save(&mut chunk)?;
    }
    w.to_vec()
}

/// Save a processor and load it into a fresh one.
fn round_trip(cpu: &Cpu) -> Result<(Cpu, Vec<u8>)> {
    let bytes = save_bytes(cpu)?;
    let restored = Cpu::new(cpu.config());
    let reader = StateReader::new(&bytes)?;
    let chunk = reader.load("cpu", CLASS.name, CLASS.version, &Migrations::new())?;
    let mut cr = chunk.reader();
    restored.load(&mut cr)?;
    cr.end()?;
    Ok((restored, bytes))
}

#[test]
fn save_and_load_round_trip_to_an_identical_state() -> Result<()> {
    // Taken mid delay slot, with bank 1 visible, a TLB entry, an FPU mode and
    // a pending request — the things a naive snapshot loses.
    let h = Harness::new(&[mov_i(0, 9), trapa(0), NOP]);
    h.load(0x100, &[mov_i(0, 3), bra(4), NOP, NOP, NOP, NOP]);
    h.cpu.set_fpscr(fpscr::FR | fpscr::SZ);
    h.cpu.set_fpr(1, 7, 0x4049_0fdb);
    h.cpu.set_interrupt(2, 0x400);
    h.cpu.set_sr(sr::MD | sr::IMASK);
    h.steps(4);
    assert!(h.cpu.delay_target().is_some());
    let (restored, bytes) = round_trip(&h.cpu)?;
    assert_eq!(
        save_bytes(&restored)?,
        bytes,
        "a round trip is a fixed point"
    );
    assert_eq!(restored.delay_target(), h.cpu.delay_target());
    assert_eq!(restored.banks(), h.cpu.banks());
    assert_eq!(restored.reg(0), 3);
    assert_eq!(restored.fpr(), h.cpu.fpr());
    assert_eq!(restored.lines().request(), (2, 0x400));
    Ok(())
}
