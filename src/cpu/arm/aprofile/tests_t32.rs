//! Tests for Thumb-2: the 32-bit T32 encodings, `IT`, the ARMv6 and v6T2
//! 16-bit additions, and the Thumb-state PC and interworking rules.
//!
//! Every program here was assembled with GNU `as -mthumb -march=armv7-a`
//! (plus `sec`, `mp`, `idiv`, `vfpv3`) and read back with `objcopy` —
//! running a tool, not reading its source — and the listing sits beside each
//! array. Expected results are worked from DDI 0406C's pseudocode by hand.
//! The disassembly corpus at the bottom is `objdump -d`'s own text for the
//! same bytes, normalised (lower case, comments dropped, `r9`–`r12` rather
//! than `sb`/`sl`/`fp`/`ip`, branch targets as `0x%08x`).
//!
//! As in `tests_v7.rs`, two things matter as much as the new instructions
//! working: an ARMv5TE part must decode Thumb exactly as it always did, and
//! an instruction a part lacks must be UNDEFINED rather than execute.

use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::space::{AddressSpace, RamStore, Region};

use super::isa::{Cond, Insn};
use super::thumb::{self, Thumb};
use super::thumb2::{self, T32};
use super::*;

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

const RAM: usize = 0x2_0000;
const CODE: u32 = 0x1000;
const DATA: u32 = 0x8000;

struct Rig {
    cpu: Arc<Arm>,
    ram: Arc<RamStore>,
}

impl Rig {
    fn with_config(cfg: Config) -> Rig {
        let ram = Arc::new(RamStore::new(RAM as u64));
        let space = AddressSpace::new("cpu", 32);
        space
            .topology()
            .map(Region::ram("ram", Arc::clone(&ram)), 0)
            .expect("ram maps");
        let cpu = Arc::new(Arm::new(cfg));
        cpu.attach_space(Arc::new(space));
        Rig { cpu, ram }
    }

    /// A Cortex-A9's instruction set, with no CP15 so addresses are flat.
    fn a9() -> Rig {
        Rig::with_arch(Arch::CORTEX_A9)
    }

    fn with_arch(arch: Arch) -> Rig {
        Rig::with_config(Config {
            arch,
            ..Config::ARM926EJS
        })
    }

    fn poke(&self, addr: u32, word: u32) {
        for (i, b) in word.to_le_bytes().iter().enumerate() {
            self.ram.write_u8(u64::from(addr) + i as u64, *b).unwrap();
        }
    }

    fn poke_half(&self, addr: u32, half: u16) {
        for (i, b) in half.to_le_bytes().iter().enumerate() {
            self.ram.write_u8(u64::from(addr) + i as u64, *b).unwrap();
        }
    }

    fn poke_byte(&self, addr: u32, byte: u8) {
        self.ram.write_u8(u64::from(addr), byte).unwrap();
    }

    fn peek(&self, addr: u32) -> u32 {
        let mut bytes = [0u8; 4];
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = self.ram.read_u8(u64::from(addr) + i as u64).unwrap();
        }
        u32::from_le_bytes(bytes)
    }

    fn peek_byte(&self, addr: u32) -> u8 {
        self.ram.read_u8(u64::from(addr)).unwrap()
    }

    /// Load `code` at `at`, run the reset sequence, and start there in
    /// Thumb state, System mode, interrupts masked.
    fn thumb_at(&self, at: u32, code: &[u16]) {
        for (i, h) in code.iter().enumerate() {
            self.poke_half(at + 2 * i as u32, *h);
        }
        self.cpu.step();
        self.cpu
            .set_cpsr(u32::from(Mode::SYSTEM.0) | psr::I | psr::F | psr::T);
        self.cpu.set_pc(at);
    }

    fn thumb(&self, code: &[u16]) {
        self.thumb_at(CODE, code);
    }

    fn steps(&self, n: usize) {
        for _ in 0..n {
            self.cpu.step();
        }
    }

    /// Step until the `BKPT` that ends every program takes its Prefetch
    /// Abort, and say where it was.
    fn run_to_bkpt(&self) -> u32 {
        for _ in 0..10_000 {
            self.cpu.step();
            if self.cpu.mode() == Mode::ABORT && self.cpu.pc() == 0x0c {
                // `LR_abt` is the BKPT's address plus four.
                return self.r(14).wrapping_sub(4);
            }
        }
        panic!("no BKPT reached; pc = {:#x}", self.cpu.pc());
    }

    fn r(&self, i: u8) -> u32 {
        self.cpu.reg(i)
    }

    fn set(&self, i: u8, v: u32) {
        self.cpu.set_reg(i, v);
    }

    fn cpsr(&self) -> u32 {
        self.cpu.cpsr()
    }

    fn flags(&self) -> u32 {
        self.cpsr() & (psr::N | psr::Z | psr::C | psr::V)
    }

    fn it_bits(&self) -> u32 {
        self.cpsr() & psr::IT
    }

    fn spsr(&self) -> u32 {
        self.cpu.regs().spsr().unwrap()
    }
}

/// The `CPSR.IT` bits for an eight-bit `ITSTATE`.
const fn it_psr(it: u32) -> u32 {
    ((it >> 2) << 10) | ((it & 3) << 25)
}

// ---------------------------------------------------------------------------
// ARMv5 is untouched
// ---------------------------------------------------------------------------

#[test]
fn an_armv5_part_decodes_thumb_exactly_as_it_always_did() {
    // A digest of every 16-bit decode (value and text), taken before any of
    // the Thumb-2 work existed.
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for raw in 0..=u16::MAX {
        let d = thumb::decode(raw);
        assert_eq!(d, thumb::decode_for(&Arch::V5TE, raw));
        for b in format!("{d:?}|{d}").bytes() {
            h ^= u64::from(b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    assert_eq!(h, 0x31e7_704b_9121_b193);
}

#[test]
fn a_v5_part_runs_a_bl_pair_as_two_instructions() {
    // `bl +0x100` as ARMv5T splits it: prefix, then suffix, each a step.
    let m = Rig::with_config(Config::ARM926EJS);
    m.thumb(&[0xf000, 0xf880]);
    m.cpu.step();
    assert_eq!(
        m.cpu.pc(),
        CODE + 2,
        "the prefix is an instruction of its own"
    );
    assert_eq!(m.r(14), CODE + 4);
    m.cpu.step();
    assert_eq!(m.cpu.pc(), CODE + 4 + 0x100);
    assert_eq!(m.r(14), (CODE + 4) | 1);
}

#[test]
fn a_v5_part_takes_undefined_on_the_later_16_bit_encodings() {
    for half in [0xba08u16, 0xb24c, 0xb672, 0xb658, 0xb100, 0xbf08, 0xbf20] {
        let m = Rig::with_config(Config::ARM926EJS);
        m.thumb(&[half]);
        m.cpu.step();
        assert_eq!(m.cpu.mode(), Mode::UNDEFINED, "{half:#06x}");
        assert!(matches!(thumb::decode(half), Thumb::Undefined));
    }
}

#[test]
fn a_v5_part_ignores_it_bits_an_msr_left_in_cpsr() {
    // ARMv5's MSR writes whole bytes, so bits 15..10 can hold anything. They
    // are not ITSTATE there: a 16-bit ADDS still sets the flags.
    let m = Rig::with_config(Config::ARM926EJS);
    m.thumb(&[0x3101, 0xbe00]); // adds r1, #1 ; bkpt
    m.cpu.set_cpsr(m.cpsr() | it_psr(0x04));
    m.set(1, u32::MAX);
    m.cpu.step();
    assert_eq!(m.r(1), 0);
    assert_ne!(m.cpsr() & psr::Z, 0);
}

// ---------------------------------------------------------------------------
// 32-bit data processing
// ---------------------------------------------------------------------------

#[test]
fn modified_immediates_expand_and_carry() {
    let m = Rig::a9();
    m.thumb(DP_IMM);
    m.set(1, 0xffff_ff00);
    m.set(3, 0x1234_5678);
    m.set(6, u32::MAX);
    m.run_to_bkpt();
    assert_eq!(m.r(0), 0, "adds.w");
    assert_eq!(m.r(2), 0xffff_ff78, "orn");
    assert_eq!(m.r(4), 0x8000_0000, "movs.w");
    assert_eq!(m.r(5), 0x00ab_00ab, "ands.w, a replicated pattern");
    assert_eq!(m.r(7), 0x7f80_0000, "an odd rotation");
    assert_eq!(m.r(8), 0x100, "rsb.w");
    // The last flag-setter is `cmp.w 0xffffff00, #0x100`.
    assert_eq!(m.flags(), psr::N | psr::C);
}

#[test]
fn a_rotated_immediate_sets_carry_from_bit_31_and_a_replicated_one_does_not() {
    let m = Rig::a9();
    // movs.w r4, #0x80000000 ; ands.w r5, r6, #0x00ab00ab ; bkpt
    m.thumb(&[0xf05f, 0x4400, 0xf016, 0x15ab, 0xbe00]);
    m.set(6, u32::MAX);
    m.cpu.step();
    assert_eq!(m.flags(), psr::N | psr::C);
    m.cpu.set_cpsr(m.cpsr() & !psr::C);
    m.cpu.step();
    assert_eq!(m.flags() & psr::C, 0, "C is untouched by 0x00XY00XY");
}

#[test]
fn shifted_register_forms_share_the_a32_shifter() {
    let m = Rig::a9();
    m.thumb(DP_SHIFT);
    m.set(1, 0x8000_0003);
    m.set(2, 0x11);
    m.set(5, 4);
    m.cpu.set_cpsr(m.cpsr() | psr::C);
    m.cpu.step();
    assert_eq!(m.r(0), 0x8000_008b);
    m.cpu.step();
    assert_eq!(m.r(3), 0xc000_0001, "RRX brings the carry in");
    assert_ne!(m.cpsr() & psr::C, 0, "and takes bit 0 out");
    m.cpu.step();
    assert_eq!(m.r(4), 0x110, "lsls.w by register");
    assert_eq!(m.cpsr() & psr::C, 0);
    m.cpu.step();
    assert_eq!(m.r(6), u32::MAX, "orns with LSR #32");
    m.cpu.step();
    assert_eq!(m.flags(), psr::Z, "teq.w");
    m.cpu.step();
    assert_eq!(m.r(7), 0x8000_0000, "pkhtb");
}

#[test]
fn plain_immediates_bitfields_and_saturation() {
    let m = Rig::a9();
    m.thumb(PLAIN_IMM);
    m.set(1, 0x1234_5678);
    m.set(5, 0);
    m.set(6, u32::MAX);
    m.run_to_bkpt();
    assert_eq!(m.r(0), 0x1234_6677, "addw");
    assert_eq!(m.r(2), 0x1234_5677, "subw");
    assert_eq!(m.r(3), CODE + 0x34, "adr.w from Align(PC, 4)");
    assert_eq!(m.r(4), 0xabcd_1234, "movw/movt");
    assert_eq!(m.r(5), 0x800, "bfi");
    assert_eq!(m.r(6), 0xffff_0000, "bfc");
    assert_eq!(m.r(7), 0x67, "sbfx");
    assert_eq!(m.r(8), 0x67, "ubfx");
    assert_eq!(m.r(9), 127, "ssat");
    assert_eq!(m.r(10), 255, "usat");
    assert_eq!(m.r(11), 0x0007_0007, "ssat16");
    assert_ne!(m.cpsr() & psr::Q, 0);
}

// ---------------------------------------------------------------------------
// Loads and stores
// ---------------------------------------------------------------------------

#[test]
fn single_loads_and_stores_in_every_addressing_form() {
    let m = Rig::a9();
    m.thumb(LDST);
    m.poke(DATA + 0x10, 0x80ff_7f01);
    m.poke(DATA + 0x14, 0xaaaa_0001);
    m.poke(DATA + 0x0c, 0xbbbb_0002);
    m.poke(DATA + 0x24, 0xcccc_0003);
    m.poke(DATA + 0x30, 0xdddd_0004);
    m.poke(DATA + 0x18, 0xeeee_0005);
    m.set(1, DATA + 0x10);
    m.set(4, DATA + 0x20);
    m.set(6, DATA + 0x30);
    m.set(8, 2);
    m.run_to_bkpt();
    assert_eq!(m.r(0), 0xaaaa_0001);
    assert_eq!(m.r(2), 0xbbbb_0002, "negative imm8");
    assert_eq!((m.r(3), m.r(4)), (0xcccc_0003, DATA + 0x24), "pre-indexed");
    assert_eq!((m.r(5), m.r(6)), (0xdddd_0004, DATA + 0x38), "post-indexed");
    assert_eq!(m.r(7), 0xeeee_0005, "register, LSL #2");
    assert_eq!(m.r(9), 0xffff_ff80, "ldrsb.w");
    assert_eq!(m.r(10), 0xffff_80ff, "ldrsh.w");
    assert_eq!(m.peek(DATA + 0x50) & 0xffff, 0x0001, "strh.w");
    assert_eq!(m.peek_byte(DATA + 0x0f), 0x01, "strb, negative offset");
    assert_eq!(m.r(11), 0xaaaa_0001, "ldrt: offset, no writeback");
    assert_eq!(m.r(1), DATA + 0x10);
}

#[test]
fn literal_loads_use_the_word_aligned_pc() {
    // The first load sits at an address that is two modulo four, so PC + 4
    // is not word-aligned and only `Align(PC, 4)` finds the pool.
    let m = Rig::a9();
    m.thumb(LITERAL);
    m.run_to_bkpt();
    assert_eq!(m.r(0), 0x1111_1111);
    assert_eq!(m.r(1), 0x2222_2222);
    assert_eq!((m.r(2), m.r(3)), (0x1111_1111, 0x2222_2222), "ldrd literal");
    assert_eq!(m.r(4), 0x2222_2222, "16-bit literal");
}

#[test]
fn doubleword_and_exclusive_transfers() {
    let m = Rig::a9();
    m.thumb(DUAL_EX);
    m.poke(DATA + 0x18, 1);
    m.poke(DATA + 0x1c, 2);
    m.poke(DATA + 0x44, 0x55);
    m.poke(DATA + 0x60, 0x77);
    m.poke(DATA + 0x64, 0x88);
    m.set(1, DATA + 0x10);
    m.set(4, DATA + 0x40);
    m.set(6, 0x66);
    m.set(10, DATA + 0x60);
    m.run_to_bkpt();
    assert_eq!((m.r(0), m.r(2)), (1, 2), "ldrd into a non-consecutive pair");
    assert_eq!((m.peek(DATA + 8), m.peek(DATA + 0xc)), (1, 2));
    assert_eq!(m.r(1), DATA + 8, "strd writeback");
    assert_eq!(m.r(3), 0x55, "ldrex with an offset");
    assert_eq!(m.r(5), 0, "the first strex succeeds");
    assert_eq!(m.peek(DATA + 0x44), 0x66);
    assert_eq!(m.r(7), 1, "the second finds the monitor open");
    assert_eq!((m.r(8), m.r(9)), (0x77, 0x88), "ldrexd r8, r9");
    assert_eq!(m.r(11), 0, "strexd");
    assert_eq!((m.peek(DATA + 0x60), m.peek(DATA + 0x64)), (DATA + 8, 2));
}

#[test]
fn a_misaligned_ldrd_faults_on_armv7() {
    let m = Rig::a9();
    m.thumb(&[0xe9d1, 0x0202, 0xbe00]); // ldrd r0, r2, [r1, #8]
    m.set(1, DATA + 2);
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::ABORT);
    assert_eq!(m.cpu.pc(), 0x10, "a Data Abort");
    assert_eq!(m.r(14), CODE + 8);
}

#[test]
fn table_branches_index_bytes_and_halfwords() {
    let m = Rig::a9();
    m.thumb(TABLE);
    m.set(0, 1);
    m.set(2, 1);
    m.run_to_bkpt();
    assert_eq!(m.r(1), 11, "tbb picked case 1");
    assert_eq!(m.r(3), 21, "tbh picked case 1");
    let m = Rig::a9();
    m.thumb(TABLE);
    m.set(0, 2);
    m.set(2, 0);
    m.run_to_bkpt();
    assert_eq!((m.r(1), m.r(3)), (12, 20));
}

// ---------------------------------------------------------------------------
// Multiplies and media
// ---------------------------------------------------------------------------

#[test]
fn multiplies_share_the_a32_arithmetic() {
    let m = Rig::a9();
    m.thumb(MULDIV);
    m.set(1, 7);
    m.set(2, 0xffff_fffd);
    m.set(4, 100);
    m.set(8, 1);
    m.set(9, 2);
    m.steps(7);
    assert_eq!(m.r(0), 0xffff_ffeb, "mul");
    assert_eq!(m.r(3), 79, "mla");
    assert_eq!(m.r(5), 121, "mls");
    assert_eq!((m.r(6), m.r(7)), (0xffff_ffeb, u32::MAX), "smull");
    assert_eq!((m.r(8), m.r(9)), (0xffff_ffee, 6), "umaal");
    assert_eq!(m.r(10), 93, "smlabt");
    assert_eq!(m.r(11), 79, "smlad");
    // A Cortex-A9 has no divider: SDIV is UNDEFINED, and a Thumb Undefined
    // Instruction returns to the instruction plus two (B1.9, the link
    // offsets), whatever its width.
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::UNDEFINED);
    assert_eq!(m.r(14), CODE + 0x1c + 2);
    assert_ne!(m.spsr() & psr::T, 0);
}

#[test]
fn a_part_with_the_thumb_divider_divides() {
    let mut arch = Arch::CORTEX_A9;
    arch.ext.idiv_thumb = true;
    let m = Rig::with_arch(arch);
    m.thumb(MULDIV);
    m.set(1, 7);
    m.set(2, 0xffff_fffd);
    m.run_to_bkpt();
    assert_eq!(m.r(12), 0xffff_fffe, "7 / -3 truncates to -2");
}

#[test]
fn media_instructions_share_the_a32_lanes() {
    let m = Rig::a9();
    m.thumb(MEDIA);
    m.set(1, 0x7fff_8001);
    m.set(2, 0x0001_ffff);
    m.run_to_bkpt();
    assert_eq!(m.r(0), 0x8000_8000, "sadd16");
    assert_eq!(m.r(3), 0x7fff_ffff, "sel on GE = 1100");
    assert_eq!(m.r(4), 0x0180_ff7f, "rev.w");
    assert_eq!(m.r(5), 0x8001_fffe, "rbit");
    assert_eq!(m.r(6), 1, "clz");
    assert_eq!(m.r(7), 0x7fff_ffff, "qadd saturates");
    assert_ne!(m.cpsr() & psr::Q, 0);
    assert_eq!(m.r(8), 0x0080_ff7f, "sxtab16 with ROR #8");
    assert_eq!(m.r(9), 762, "usad8");
    assert_eq!(m.r(10), 0x3f80_bf80, "uhadd8");
}

// ---------------------------------------------------------------------------
// Branches and interworking
// ---------------------------------------------------------------------------

#[test]
fn branches_links_and_interworking_both_ways() {
    let m = Rig::a9();
    m.thumb(BRANCH);
    m.set(0, 0);
    m.set(1, 0);
    // b.w, cmp, beq.w: past both `movs r0`.
    m.steps(3);
    assert_eq!(m.cpu.pc(), CODE + 0xe);
    // bl: LR is the next instruction with bit 0 set.
    m.cpu.step();
    assert_eq!(m.cpu.pc(), CODE + 0x18);
    assert_eq!(m.r(14), (CODE + 0x12) | 1);
    m.steps(2); // movs r2, #42 ; bx lr
    assert_eq!(m.cpu.pc(), CODE + 0x12);
    assert!(m.cpu.is_thumb());
    // blx (immediate) from Thumb: ARM state, at Align(PC, 4) + imm.
    m.cpu.step();
    assert!(!m.cpu.is_thumb());
    assert_eq!(m.cpu.pc(), CODE + 0x1c);
    assert_eq!(m.r(14), (CODE + 0x16) | 1);
    m.steps(2); // mov r3, #7 ; bx lr — back to Thumb
    assert!(m.cpu.is_thumb());
    assert_eq!(m.cpu.pc(), CODE + 0x16);
    assert_eq!(m.run_to_bkpt(), CODE + 0x16);
    assert_eq!((m.r(0), m.r(2), m.r(3)), (0, 42, 7));
}

#[test]
fn an_arm_blx_lands_in_thumb_and_a_32_bit_bl_returns() {
    // ARM: blx +8 (to CODE + 0x10, Thumb). Thumb there: bl to a `bx lr`.
    let m = Rig::a9();
    m.poke(CODE, 0xfa00_0002); // blx 0x1010
    m.poke(CODE + 4, 0xe120_0070); // bkpt
    m.poke_half(CODE + 0x10, 0x2005); // movs r0, #5
    m.poke_half(CODE + 0x12, 0x4770); // bx lr
    m.cpu.step();
    m.cpu.set_cpsr(u32::from(Mode::SYSTEM.0) | psr::I | psr::F);
    m.cpu.set_pc(CODE);
    m.cpu.step();
    assert!(m.cpu.is_thumb());
    assert_eq!(m.cpu.pc(), CODE + 0x10);
    assert_eq!(m.r(14), CODE + 4);
    m.steps(2);
    assert!(!m.cpu.is_thumb(), "bx lr to an even address is ARM state");
    assert_eq!(m.cpu.pc(), CODE + 4);
    assert_eq!(m.r(0), 5);
}

#[test]
fn block_transfers_and_pop_pc_interworks() {
    let m = Rig::a9();
    m.thumb(LDM);
    m.poke(CODE + 0x100, 0xe120_0070); // ARM bkpt
    m.set(13, DATA + 0x100);
    m.set(4, 4);
    m.set(5, 5);
    m.set(8, 8);
    m.set(14, 0x1234);
    m.set(6, DATA + 0x80);
    m.poke(DATA + 0x70, 0x99);
    m.poke(DATA + 0x74, 0xaa);
    m.poke(DATA + 0x100, 0x44);
    m.poke(DATA + 0x104, CODE + 0x100);
    m.steps(4);
    assert_eq!((m.r(0), m.r(1), m.r(2), m.r(3)), (4, 5, 8, 0x1234));
    assert_eq!(m.r(13), DATA + 0x100);
    assert_eq!(m.r(6), DATA + 0x78);
    assert_eq!((m.peek(DATA + 0x78), m.peek(DATA + 0x7c)), (4, 8));
    assert_eq!((m.r(9), m.r(10)), (0x99, 0xaa));
    m.cpu.step();
    assert_eq!(m.r(4), 0x44);
    assert!(!m.cpu.is_thumb(), "LoadWritePC of an even address");
    assert_eq!(m.cpu.pc(), CODE + 0x100);
}

// ---------------------------------------------------------------------------
// System instructions
// ---------------------------------------------------------------------------

#[test]
fn mrs_msr_and_cps_from_thumb() {
    let m = Rig::a9();
    m.thumb(SYS);
    m.set(1, 0xa000_0000 | u32::from(Mode::SYSTEM.0) | psr::I);
    m.cpu.step();
    assert_eq!(m.r(0), m.cpsr());
    m.cpu.step();
    assert_eq!(m.flags(), psr::N | psr::C);
    assert!(m.cpu.is_thumb(), "MSR cannot write the execution state");
    m.cpu.step();
    assert_ne!(m.cpsr() & psr::I, 0);
    m.cpu.step();
    assert_eq!(m.cpsr() & psr::I, 0, "cpsie.w i");
}

#[test]
fn srs_and_rfe_from_thumb() {
    let m = Rig::a9();
    m.thumb(SRS_RFE);
    m.cpu
        .set_cpsr(u32::from(Mode::SUPERVISOR.0) | psr::I | psr::F | psr::T);
    m.set(13, DATA + 0x200);
    m.set(14, CODE + 0x40);
    let mut regs = m.cpu.regs();
    regs.set_spsr(u32::from(Mode::SYSTEM.0) | psr::T | psr::Z);
    m.cpu.set_regs(regs);
    m.poke_half(CODE + 0x40, 0xbe00);
    m.cpu.step();
    assert_eq!(m.r(13), DATA + 0x1f8);
    assert_eq!(m.peek(DATA + 0x1f8), CODE + 0x40);
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::SYSTEM);
    assert!(m.cpu.is_thumb());
    assert_ne!(m.cpsr() & psr::Z, 0);
    assert_eq!(m.cpu.pc(), CODE + 0x40);
}

#[test]
fn coprocessor_15_from_thumb_with_privilege() {
    let m = Rig::with_config(Config::CORTEX_A9);
    m.thumb(CP15);
    m.cpu.step();
    assert_eq!(m.r(0), cp15v7::id::MIDR);
    // The same MRC from User mode is UNDEFINED.
    let m = Rig::with_config(Config::CORTEX_A9);
    m.thumb(CP15);
    m.cpu.set_cpsr(u32::from(Mode::USER.0) | psr::T);
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::UNDEFINED);
}

#[cfg(feature = "cpu-arm-aprofile-vfp")]
#[test]
fn vfp_from_thumb() {
    let m = Rig::a9();
    m.thumb(VFP);
    let mut v = m.cpu.vfp().unwrap();
    v.fpexc = vfp::fpexc::EN;
    m.cpu.set_vfp(v);
    m.set(0, 1.5f32.to_bits());
    m.set(1, 2.25f32.to_bits());
    m.run_to_bkpt();
    assert_eq!(m.r(2), 3.75f32.to_bits());
}

// ---------------------------------------------------------------------------
// The 16-bit additions
// ---------------------------------------------------------------------------

#[test]
fn the_armv6_16_bit_instructions() {
    let m = Rig::a9();
    m.thumb(V6_16);
    m.cpu.set_cpsr(m.cpsr() & !psr::I);
    m.set(1, 0x1234_80f1);
    m.set(7, DATA);
    m.poke(DATA, 0x1122_3344);
    m.run_to_bkpt();
    assert_eq!(m.r(0), 0xf180_3412, "rev");
    assert_eq!(m.r(2), 0x3412_f180, "rev16");
    assert_eq!(m.r(3), 0xffff_f180, "revsh");
    assert_eq!(m.r(4), 0xffff_fff1, "sxtb");
    assert_eq!(m.r(5), 0x80f1, "uxth");
    assert_eq!(m.r(6), 0x4433_2211, "a load under SETEND BE");
    assert_eq!(m.spsr() & psr::E, 0, "SETEND LE again");
    assert_ne!(m.spsr() & psr::I, 0, "cpsid i");
}

#[test]
fn cbz_and_cbnz() {
    for (r0, r2, r1_after, r3_after) in [(0, 5, 0, 0), (1, 0, 1, 1)] {
        let m = Rig::a9();
        m.thumb(CBZ);
        m.set(0, r0);
        m.set(1, 0);
        m.set(2, r2);
        m.set(3, 0);
        m.run_to_bkpt();
        assert_eq!((m.r(1), m.r(3)), (r1_after, r3_after), "r0={r0} r2={r2}");
    }
}

#[test]
fn the_16_bit_hints() {
    // wfi: the core halts, and an interrupt wakes it.
    let m = Rig::a9();
    m.thumb(&[0xbf30, 0xbe00]);
    m.cpu.step();
    assert!(m.cpu.is_halted());
    assert_eq!(m.cpu.pc(), CODE + 2);
    // sev ; wfe: the event is consumed and the wait falls through.
    let m = Rig::a9();
    m.thumb(&[0xbf40, 0xbf20, 0xbe00]);
    m.steps(2);
    assert!(!m.cpu.is_halted());
    assert_eq!(m.cpu.pc(), CODE + 4);
}

// ---------------------------------------------------------------------------
// IT blocks
// ---------------------------------------------------------------------------

#[test]
fn an_ittee_block_follows_flags_that_change_inside_it() {
    let m = Rig::a9();
    m.thumb(IT_FLAGS);
    m.set(1, u32::MAX);
    m.set(2, 0);
    m.set(3, 0);
    m.steps(3);
    assert_eq!(
        m.it_bits(),
        it_psr(0x07),
        "ittee eq: firstcond 0000, mask 0111"
    );
    m.run_to_bkpt();
    // addseq.w wraps r1 to zero (Z stays set); subseq.w takes r0 to -1 and
    // clears Z — so both "else" instructions now pass.
    assert_eq!(m.r(0), u32::MAX);
    assert_eq!(m.r(1), 0);
    assert_eq!((m.r(2), m.r(3)), (1, 1));
    assert_eq!(m.spsr() & (psr::N | psr::Z | psr::C | psr::V), psr::N);
    assert_eq!(m.spsr() & psr::IT, 0, "the block is over by the BKPT");
}

#[test]
fn a_16_bit_adds_inside_an_it_block_does_not_set_flags() {
    let m = Rig::a9();
    m.thumb(IT_NOFLAGS);
    m.set(1, u32::MAX);
    m.set(2, 0);
    m.run_to_bkpt();
    assert_eq!(m.r(1), 0, "addeq executed");
    assert_eq!(m.r(2), 0xffff_fffb, "subeq executed");
    assert_eq!(
        m.spsr() & (psr::N | psr::Z | psr::C | psr::V),
        psr::Z | psr::C,
        "the flags are cmp's, untouched by either"
    );
}

#[test]
fn a_skipped_instruction_has_no_side_effects_and_still_advances() {
    // The condition fails for both, so the unmapped load never happens.
    let m = Rig::a9();
    m.thumb(IT_ABORT);
    m.set(0, 1);
    m.set(2, 0x9000_0000);
    m.cpu.set_cpsr(m.cpsr() & !psr::Z);
    // `cmp r0, r0` would set Z; start after it instead.
    m.cpu.set_pc(CODE + 2);
    m.steps(3);
    assert_eq!(m.cpu.mode(), Mode::SYSTEM);
    assert_eq!(m.cpu.pc(), CODE + 8);
    assert_eq!(m.it_bits(), 0);
    assert_eq!(m.r(3), 0);
}

#[test]
fn a_branch_ends_an_it_block() {
    for (r0, r1) in [(1, 9), (0, 3)] {
        let m = Rig::a9();
        m.thumb(IT_BRANCH);
        m.set(0, r0);
        m.set(1, 0);
        m.run_to_bkpt();
        assert_eq!(m.r(1), r1, "r0 = {r0}");
        assert_eq!(m.spsr() & psr::IT, 0);
    }
}

#[test]
fn an_interrupt_between_instructions_of_a_block_resumes_it() {
    let m = Rig::a9();
    m.thumb(IT_IRQ);
    m.poke(0x18, 0xe25e_f004); // subs pc, lr, #4
    m.cpu.set_cpsr(m.cpsr() & !psr::I);
    m.steps(3); // cmp ; itte eq ; addeq r1
    assert_eq!(m.r(1), 1);
    assert_eq!(m.it_bits(), it_psr(0x0c), "the second instruction's state");
    m.cpu.set_irq(true);
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::IRQ);
    assert_eq!(m.it_bits(), 0, "exception entry clears ITSTATE");
    assert_eq!(m.spsr() & psr::IT, it_psr(0x0c), "and the SPSR keeps it");
    assert_eq!(m.r(14), CODE + 6 + 4);
    m.cpu.set_irq(false);
    m.cpu.step(); // subs pc, lr, #4
    assert_eq!(m.cpu.pc(), CODE + 6);
    assert!(m.cpu.is_thumb());
    assert_eq!(m.it_bits(), it_psr(0x0c));
    m.run_to_bkpt();
    assert_eq!((m.r(1), m.r(2), m.r(3)), (1, 1, 0), "addne stayed skipped");
}

#[test]
fn a_data_abort_inside_a_block_saves_the_unadvanced_state() {
    let m = Rig::a9();
    m.thumb(IT_ABORT);
    m.set(2, 0x9000_0000);
    m.steps(3);
    assert_eq!(m.cpu.mode(), Mode::ABORT);
    assert_eq!(m.cpu.pc(), 0x10);
    assert_eq!(m.r(14), CODE + 4 + 8, "the aborting ldreq plus eight");
    assert_eq!(m.spsr() & psr::IT, it_psr(0x04), "ldreq's own state");
}

#[test]
fn svc_inside_a_block_saves_the_advanced_state() {
    let m = Rig::a9();
    m.thumb(IT_SVC);
    m.poke(0x08, 0xe1b0_f00e); // movs pc, lr
    m.set(3, 0);
    m.steps(3);
    assert_eq!(m.cpu.mode(), Mode::SUPERVISOR);
    assert_eq!(m.r(14), CODE + 6, "the next instruction");
    assert_eq!(m.spsr() & psr::IT, it_psr(0x08), "addeq's state");
    assert_eq!(m.cpu.last_swi(), 7);
    m.cpu.step();
    assert_eq!(m.cpu.pc(), CODE + 6);
    m.run_to_bkpt();
    assert_eq!(m.r(3), 1, "addeq ran under the restored block");
}

#[test]
fn an_undefined_32_bit_instruction_links_to_its_second_halfword() {
    let m = Rig::a9();
    m.thumb(UNDEF32);
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::UNDEFINED);
    assert_eq!(m.r(14), CODE + 2);
}

#[test]
fn a_fetch_fault_on_the_second_halfword_is_a_prefetch_abort() {
    // The last halfword of RAM holds a 32-bit instruction's first half.
    let m = Rig::a9();
    let at = RAM as u32 - 2;
    m.thumb_at(at, &[0xf8d1]);
    m.cpu.step();
    assert_eq!(m.cpu.mode(), Mode::ABORT);
    assert_eq!(m.cpu.pc(), 0x0c);
    assert_eq!(m.r(14), at + 4);
}

// ---------------------------------------------------------------------------
// A whole program
// ---------------------------------------------------------------------------

#[test]
fn a_straight_line_program_runs_end_to_end() {
    let m = Rig::a9();
    m.thumb(PROGRAM);
    for (i, b) in b"hello\0".iter().enumerate() {
        m.poke_byte(DATA + i as u32, *b);
    }
    m.poke_half(CODE + 0x200, 0xbe00);
    m.set(0, DATA);
    m.set(1, DATA + 0x100);
    m.set(4, 0x4444);
    m.set(13, DATA + 0x1000);
    m.set(14, (CODE + 0x200) | 1);
    assert_eq!(m.run_to_bkpt(), CODE + 0x200);
    let copied: Vec<u8> = (0..6).map(|i| m.peek_byte(DATA + 0x100 + i)).collect();
    assert_eq!(copied, b"hello\0");
    assert_eq!(m.r(0), 5, "length");
    assert_eq!(m.r(2), 532, "byte sum");
    assert_eq!(m.r(3), 55, "1 + ... + 10");
    assert_eq!(m.r(5), 3025, "squared");
    assert_eq!(m.r(6), 0x1234_5678);
    assert_eq!(m.r(7), 0x56);
    assert_eq!(m.r(4), 0x4444, "callee-saved");
    // The BKPT left the core in Abort mode, with its own SP.
    assert_eq!(m.cpu.regs().reg_in_mode(Mode::SYSTEM, 13), DATA + 0x1000);
}

// ---------------------------------------------------------------------------
// Decode never panics, execution never panics
// ---------------------------------------------------------------------------

/// A small deterministic generator.
struct XorShift(u32);

impl XorShift {
    fn next(&mut self) -> u32 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 17;
        self.0 ^= self.0 << 5;
        self.0
    }
}

#[test]
fn decoding_and_printing_any_t32_pair_never_panics() {
    let arch = Arch::CORTEX_A9;
    let mut rng = XorShift(0x2545_f491);
    let mut defined = 0u32;
    for _ in 0..(1u32 << 20) {
        let x = rng.next();
        // Force a 32-bit first halfword: 11101, 11110 or 11111.
        let hw1 = 0xe800 + ((x >> 16) % 0x1800) as u16;
        let hw2 = x as u16;
        let t = T32::decode_for(&arch, hw1, hw2);
        assert!(matches!(t, T32::Wide(_)));
        let _ = format!("{}", t.ual(Some(0x1000), None));
        let _ = format!("{}", t.ual(None, Some(Cond::NE)));
        if let T32::Wide(d) = t {
            let _ = format!("{d}");
            if !d.is_undefined() {
                defined += 1;
            }
        }
    }
    // Most of the space is allocated.
    assert!(defined > 1 << 19, "{defined}");
    for raw in 0..=u16::MAX {
        let t = T32::decode_for(&arch, raw, 0);
        let _ = format!("{}", t.ual(Some(0), Some(Cond::GT)));
    }
}

#[test]
fn executing_random_thumb_never_panics() {
    let m = Rig::with_config(Config::CORTEX_A9);
    let mut rng = XorShift(0x9e37_79b9);
    m.cpu.step();
    for _ in 0..40_000 {
        let x = rng.next();
        let y = rng.next();
        m.poke(CODE, x);
        let mut regs = m.cpu.regs();
        for r in 0..13 {
            // Mostly addresses inside RAM, sometimes anything.
            regs.r[r] = if rng.next() & 3 == 0 {
                rng.next()
            } else {
                DATA + (rng.next() & 0xfff)
            };
        }
        regs.r[13] = DATA + 0x4000;
        regs.r[15] = CODE;
        let mode = if y & 1 == 0 {
            Mode::SYSTEM
        } else {
            Mode::SUPERVISOR
        };
        regs.write_cpsr(u32::from(mode.0) | psr::I | psr::F | psr::T | (y & 0xf000_0000));
        if y & 0x10 != 0 {
            // Sometimes inside an IT block.
            regs.cpsr |= it_psr((y >> 8) & 0xff);
        }
        m.cpu.set_regs(regs);
        m.cpu.step();
    }
}

// ---------------------------------------------------------------------------
// Disassembly
// ---------------------------------------------------------------------------

#[test]
fn ual_spot_checks() {
    let a9 = Arch::CORTEX_A9;
    let text = |hw1: u16, hw2: u16| format!("{}", T32::decode_for(&a9, hw1, hw2).ual(None, None));
    assert_eq!(text(0xf511, 0x7080), "ADDS.W r0, r1, #256");
    assert_eq!(text(0xf8d1, 0x0004), "LDR.W r0, [r1, #4]");
    assert_eq!(text(0xe8df, 0xf000), "TBB [pc, r0]");
    assert_eq!(text(0xbf07, 0), "ITTEE EQ");
    assert_eq!(text(0xbf1b, 0), "ITTET NE");
    let listing = disasm::disassemble_run_for(&a9, CODE, 3, true, |a| {
        let image: [u8; 8] = [0x08, 0xbf, 0x08, 0x46, 0x10, 0xb1, 0x00, 0xbf];
        image
            .get((a - CODE) as usize)
            .copied()
            .ok_or(disasm::Missing::Unmapped)
    });
    let lines: Vec<String> = listing.iter().map(|l| format!("{l}")).collect();
    assert_eq!(lines[0], "00001000: bf08      IT EQ");
    assert_eq!(lines[1], "00001002: 4608      MOVEQ r0, r1");
    assert_eq!(lines[2], "00001004: b110      CBZ r0, 0x0000100c");
    assert_eq!(listing[2].branch_target(), Some(0x100c));
}

/// Encodings whose `objdump` spelling is a different style rather than a
/// different instruction: the coprocessor forms (`p15`/`#0`/`c1` against
/// `15`/`0`/`cr1`/`{0}`), `BKPT`'s hex operand, `SVC` without its `#`, and a
/// VFP immediate `objdump` prints as its raw eight bits where UAL (and this
/// disassembler) prints the value.
fn style_differs(expected: &str) -> bool {
    [
        "mcr", "mrc", "mcrr", "mrrc", "cdp", "ldc", "stc", "bkpt", "svc", ".word", ".short",
    ]
    .iter()
    .any(|m| expected.starts_with(m))
        || (expected.starts_with("vmov.f") && expected.contains('#'))
}

/// `objdump` still spells the non-shareable barrier option by its ARMv6
/// name, `UN`; DDI 0406C calls it `NSH`.
fn modern_spelling(expected: &str) -> String {
    expected
        .to_lowercase()
        .replace("dsb un", "dsb nsh")
        .replace("dmb un", "dmb nsh")
}

#[test]
fn t32_listing_matches_objdump() {
    // The corpus includes `SDIV`/`UDIV`, so list it as a part with the Thumb
    // divider; a Cortex-A9 would rightly list them as UNDEFINED.
    let mut a9 = Arch::CORTEX_A9;
    a9.ext.idiv_thumb = true;
    let words = T32_CORPUS_IMAGE.len() / 2;
    let listing = disasm::disassemble_run_for(&a9, 0, words, true, |a| {
        T32_CORPUS_IMAGE
            .get(a as usize)
            .copied()
            .ok_or(disasm::Missing::Unmapped)
    });
    let mut compared = 0;
    let mut wrong = Vec::new();
    for &(addr, expected) in T32_CORPUS {
        if style_differs(expected) {
            continue;
        }
        let Some(entry) = listing.iter().find(|l| l.addr() == addr) else {
            wrong.push(format!("{addr:#x}: not listed; expected `{expected}`"));
            continue;
        };
        let disasm::Listed::Thumb2 { insn, it, .. } = *entry else {
            panic!("not a Thumb-2 listing");
        };
        let ours = format!("{}", insn.ual(Some(addr), it)).to_lowercase();
        let expected = modern_spelling(expected);
        compared += 1;
        if ours != expected {
            wrong.push(format!("{addr:#x}: ours `{ours}`, objdump `{expected}`"));
        }
    }
    assert!(
        wrong.is_empty(),
        "{} of {compared} differ:\n{}",
        wrong.len(),
        wrong.join("\n")
    );
    assert!(compared > 600, "{compared}");
}

#[test]
fn the_decoder_reuses_the_a32_variants() {
    // One description per operation: T32's SADD16 and A32's are one value.
    let t = thumb2::decode_for(&Arch::CORTEX_A9, 0xfa91, 0xf002);
    let a = isa::decode_for(&Arch::CORTEX_A9, 0xe611_0f12); // sadd16 r0, r1, r2
    assert_eq!(t.insn, a.insn);
    assert!(matches!(
        thumb2::decode_for(&Arch::CORTEX_A9, 0xe8df, 0xf000).insn,
        Insn::TableBranch { .. }
    ));
    // A part without Thumb-2 has no 32-bit encodings.
    assert!(thumb2::decode_for(&Arch::V5TE, 0xfa91, 0xf002).is_undefined());
}

// ---------------------------------------------------------------------------
// Assembled programs
// ---------------------------------------------------------------------------

/// ```text
/// 0: f511 7080  adds.w r0, r1, #256 @ 0x100
/// 4: f063 02ff  orn r2, r3, #255 @ 0xff
/// 8: f05f 4400  movs.w r4, #2147483648 @ 0x80000000
/// c: f016 15ab  ands.w r5, r6, #11206827 @ 0xab00ab
/// 10: f04f 47ff  mov.w r7, #2139095040 @ 0x7f800000
/// 14: f5b1 7f80  cmp.w r1, #256 @ 0x100
/// 18: f1c1 0800  rsb r8, r1, #0
/// 1c: be00       bkpt 0x0000
/// ```
const DP_IMM: &[u16] = &[
    0xf511, 0x7080, 0xf063, 0x02ff, 0xf05f, 0x4400, 0xf016, 0x15ab, 0xf04f, 0x47ff, 0xf5b1, 0x7f80,
    0xf1c1, 0x0800, 0xbe00,
];

/// ```text
/// 0: eb01 00c2  add.w r0, r1, r2, lsl #3
/// 4: ea5f 0331  movs.w r3, r1, rrx
/// 8: fa12 f405  lsls.w r4, r2, r5
/// c: ea71 0612  orns r6, r1, r2, lsr #32
/// 10: ea91 0f01  teq r1, r1
/// 14: eac1 4722  pkhtb r7, r1, r2, asr #16
/// 18: be00       bkpt 0x0000
/// ```
const DP_SHIFT: &[u16] = &[
    0xeb01, 0x00c2, 0xea5f, 0x0331, 0xfa12, 0xf405, 0xea71, 0x0612, 0xea91, 0x0f01, 0xeac1, 0x4722,
    0xbe00,
];

/// ```text
/// 0: f601 70ff  addw r0, r1, #4095 @ 0xfff
/// 4: f2a1 0201  subw r2, r1, #1
/// 8: bf00       nop
/// a: f20f 0328  addw r3, pc, #40 @ 0x28
/// e: f241 2434  movw r4, #4660 @ 0x1234
/// 12: f6ca 34cd  movt r4, #43981 @ 0xabcd
/// 16: f361 250b  bfi r5, r1, #8, #4
/// 1a: f36f 060f  bfc r6, #0, #16
/// 1e: f341 1707  sbfx r7, r1, #4, #8
/// 22: f3c1 1807  ubfx r8, r1, #4, #8
/// 26: f301 0907  ssat r9, #8, r1
/// 2a: f3a2 0a48  usat sl, #8, r2, asr #1
/// 2e: f321 0b03  ssat16 fp, #4, r1
/// 32: be00       bkpt 0x0000
/// 34: 00000000  .word 0x00000000
/// ```
const PLAIN_IMM: &[u16] = &[
    0xf601, 0x70ff, 0xf2a1, 0x0201, 0xbf00, 0xf20f, 0x0328, 0xf241, 0x2434, 0xf6ca, 0x34cd, 0xf361,
    0x250b, 0xf36f, 0x060f, 0xf341, 0x1707, 0xf3c1, 0x1807, 0xf301, 0x0907, 0xf3a2, 0x0a48, 0xf321,
    0x0b03, 0xbe00, 0x0000, 0x0000,
];

/// ```text
/// 0: f8d1 0004  ldr.w r0, [r1, #4]
/// 4: f851 2c04  ldr.w r2, [r1, #-4]
/// 8: f854 3f04  ldr.w r3, [r4, #4]!
/// c: f856 5b08  ldr.w r5, [r6], #8
/// 10: f851 7028  ldr.w r7, [r1, r8, lsl #2]
/// 14: f991 9003  ldrsb.w r9, [r1, #3]
/// 18: f9b1 a002  ldrsh.w sl, [r1, #2]
/// 1c: f8a1 0040  strh.w r0, [r1, #64] @ 0x40
/// 20: f801 0c01  strb.w r0, [r1, #-1]
/// 24: f851 be04  ldrt fp, [r1, #4]
/// 28: be00       bkpt 0x0000
/// ```
const LDST: &[u16] = &[
    0xf8d1, 0x0004, 0xf851, 0x2c04, 0xf854, 0x3f04, 0xf856, 0x5b08, 0xf851, 0x7028, 0xf991, 0x9003,
    0xf9b1, 0xa002, 0xf8a1, 0x0040, 0xf801, 0x0c01, 0xf851, 0xbe04, 0xbe00,
];

/// ```text
/// 0: bf00       nop
/// 2: f8df 0010  ldr.w r0, [pc, #16] @ 14 <lit1>
/// 6: f8df 1010  ldr.w r1, [pc, #16] @ 18 <lit2>
/// a: e9df 2302  ldrd r2, r3, [pc, #8] @ 14 <lit1>
/// e: 4c02       ldr r4, [pc, #8] @ (18 <lit2>)
/// 10: be00       bkpt 0x0000
/// 12: bf00       nop
/// 14: 11111111  .word 0x11111111
/// 18: 22222222  .word 0x22222222
/// ```
const LITERAL: &[u16] = &[
    0xbf00, 0xf8df, 0x0010, 0xf8df, 0x1010, 0xe9df, 0x2302, 0x4c02, 0xbe00, 0xbf00, 0x1111, 0x1111,
    0x2222, 0x2222,
];

/// ```text
/// 0: e9d1 0202  ldrd r0, r2, [r1, #8]
/// 4: e961 0202  strd r0, r2, [r1, #-8]!
/// 8: e854 3f01  ldrex r3, [r4, #4]
/// c: e844 6501  strex r5, r6, [r4, #4]
/// 10: e844 6701  strex r7, r6, [r4, #4]
/// 14: e8da 897f  ldrexd r8, r9, [sl]
/// 18: e8ca 127b  strexd fp, r1, r2, [sl]
/// 1c: be00       bkpt 0x0000
/// ```
const DUAL_EX: &[u16] = &[
    0xe9d1, 0x0202, 0xe961, 0x0202, 0xe854, 0x3f01, 0xe844, 0x6501, 0xe844, 0x6701, 0xe8da, 0x897f,
    0xe8ca, 0x127b, 0xbe00,
];

/// ```text
/// 0: e8df f000  tbb [pc, r0]
/// 4: 00060402  .word 0x00060402
/// 8: 210a       movs r1, #10
/// a: e002       b.n 12 <done>
/// c: 210b       movs r1, #11
/// e: e000       b.n 12 <done>
/// 10: 210c       movs r1, #12
/// 12: e8df f012  tbh [pc, r2, lsl #1]
/// 16: 0002       .short 0x0002
/// 18: 0004       .short 0x0004
/// 1a: 2314       movs r3, #20
/// 1c: e000       b.n 20 <end>
/// 1e: 2315       movs r3, #21
/// 20: be00       bkpt 0x0000
/// ```
const TABLE: &[u16] = &[
    0xe8df, 0xf000, 0x0402, 0x0006, 0x210a, 0xe002, 0x210b, 0xe000, 0x210c, 0xe8df, 0xf012, 0x0002,
    0x0004, 0x2314, 0xe000, 0x2315, 0xbe00,
];

/// ```text
/// 0: fb01 f002  mul.w r0, r1, r2
/// 4: fb01 4302  mla r3, r1, r2, r4
/// 8: fb01 4512  mls r5, r1, r2, r4
/// c: fb81 6702  smull r6, r7, r1, r2
/// 10: fbe1 8962  umaal r8, r9, r1, r2
/// 14: fb11 4a12  smlabt sl, r1, r2, r4
/// 18: fb21 4b02  smlad fp, r1, r2, r4
/// 1c: fb91 fcf2  sdiv ip, r1, r2
/// 20: be00       bkpt 0x0000
/// ```
const MULDIV: &[u16] = &[
    0xfb01, 0xf002, 0xfb01, 0x4302, 0xfb01, 0x4512, 0xfb81, 0x6702, 0xfbe1, 0x8962, 0xfb11, 0x4a12,
    0xfb21, 0x4b02, 0xfb91, 0xfcf2, 0xbe00,
];

/// ```text
/// 0: fa91 f002  sadd16 r0, r1, r2
/// 4: faa1 f382  sel r3, r1, r2
/// 8: fa91 f481  rev.w r4, r1
/// c: fa91 f5a1  rbit r5, r1
/// 10: fab1 f681  clz r6, r1
/// 14: fa81 f781  qadd r7, r1, r1
/// 18: fa22 f891  sxtab16 r8, r2, r1, ror #8
/// 1c: fb71 f902  usad8 r9, r1, r2
/// 20: fa81 fa62  uhadd8 sl, r1, r2
/// 24: be00       bkpt 0x0000
/// ```
const MEDIA: &[u16] = &[
    0xfa91, 0xf002, 0xfaa1, 0xf382, 0xfa91, 0xf481, 0xfa91, 0xf5a1, 0xfab1, 0xf681, 0xfa81, 0xf781,
    0xfa22, 0xf891, 0xfb71, 0xf902, 0xfa81, 0xfa62, 0xbe00,
];

/// ```text
/// 0: f000 b801  b.w 6 <fwd>
/// 4: 2001       movs r0, #1
/// 6: 2900       cmp r1, #0
/// 8: f000 8001  beq.w e <taken>
/// c: 2002       movs r0, #2
/// e: f000 f803  bl 18 <sub>
/// 12: f000 e804  blx 1c <arm_sub>
/// 16: be00       bkpt 0x0000
/// 18: 222a       movs r2, #42 @ 0x2a
/// 1a: 4770       bx lr
/// 1c: e3a03007  mov r3, #7
/// 20: e12fff1e  bx lr
/// ```
const BRANCH: &[u16] = &[
    0xf000, 0xb801, 0x2001, 0x2900, 0xf000, 0x8001, 0x2002, 0xf000, 0xf803, 0xf000, 0xe804, 0xbe00,
    0x222a, 0x4770, 0x3007, 0xe3a0, 0xff1e, 0xe12f,
];

/// ```text
/// 0: e92d 4130  stmdb sp!, {r4, r5, r8, lr}
/// 4: e8bd 000f  ldmia.w sp!, {r0, r1, r2, r3}
/// 8: e926 0101  stmdb r6!, {r0, r8}
/// c: e916 0600  ldmdb r6, {r9, sl}
/// 10: e8bd 8010  ldmia.w sp!, {r4, pc}
/// ```
const LDM: &[u16] = &[
    0xe92d, 0x4130, 0xe8bd, 0x000f, 0xe926, 0x0101, 0xe916, 0x0600, 0xe8bd, 0x8010,
];

/// ```text
/// 0: f3ef 8000  mrs r0, CPSR
/// 4: f381 8900  msr CPSR_fc, r1
/// 8: f3af 8640  cpsid.w i
/// c: f3af 8440  cpsie.w i
/// 10: f3ff 8200  mrs r2, SPSR
/// 14: be00       bkpt 0x0000
/// ```
const SYS: &[u16] = &[
    0xf3ef, 0x8000, 0xf381, 0x8900, 0xf3af, 0x8640, 0xf3af, 0x8440, 0xf3ff, 0x8200, 0xbe00,
];

/// ```text
/// 0: 2000       movs r0, #0
/// 2: 2800       cmp r0, #0
/// 4: bf07       ittee eq
/// 6: f111 0101  addseq.w r1, r1, #1
/// a: f1b0 0001  subseq.w r0, r0, #1
/// e: 3201       addne r2, #1
/// 10: 3301       addne r3, #1
/// 12: be00       bkpt 0x0000
/// ```
const IT_FLAGS: &[u16] = &[
    0x2000, 0x2800, 0xbf07, 0xf111, 0x0101, 0xf1b0, 0x0001, 0x3201, 0x3301, 0xbe00,
];

/// ```text
/// 0: 2001       movs r0, #1
/// 2: 2801       cmp r0, #1
/// 4: bf04       itt eq
/// 6: 3101       addeq r1, #1
/// 8: 3a05       subeq r2, #5
/// a: be00       bkpt 0x0000
/// ```
const IT_NOFLAGS: &[u16] = &[0x2001, 0x2801, 0xbf04, 0x3101, 0x3a05, 0xbe00];

/// ```text
/// 0: 2801       cmp r0, #1
/// 2: bf04       itt eq
/// 4: 2109       moveq r1, #9
/// 6: f000 b801  beq.w c <target>
/// a: 2103       movs r1, #3
/// c: be00       bkpt 0x0000
/// ```
const IT_BRANCH: &[u16] = &[0x2801, 0xbf04, 0x2109, 0xf000, 0xb801, 0x2103, 0xbe00];

/// ```text
/// 0: b100       cbz r0, 4 <zero>
/// 2: 2101       movs r1, #1
/// 4: b902       cbnz r2, 8 <nonzero>
/// 6: 2301       movs r3, #1
/// 8: be00       bkpt 0x0000
/// ```
const CBZ: &[u16] = &[0xb100, 0x2101, 0xb902, 0x2301, 0xbe00];

/// ```text
/// 0: ba08       rev r0, r1
/// 2: ba4a       rev16 r2, r1
/// 4: bacb       revsh r3, r1
/// 6: b24c       sxtb r4, r1
/// 8: b28d       uxth r5, r1
/// a: b658       setend be
/// c: 683e       ldr r6, [r7, #0]
/// e: b650       setend le
/// 10: b672       cpsid i
/// 12: be00       bkpt 0x0000
/// ```
const V6_16: &[u16] = &[
    0xba08, 0xba4a, 0xbacb, 0xb24c, 0xb28d, 0xb658, 0x683e, 0xb650, 0xb672, 0xbe00,
];

/// ```text
/// 0: 4280       cmp r0, r0
/// 2: bf06       itte eq
/// 4: 3101       addeq r1, #1
/// 6: 3201       addeq r2, #1
/// 8: 3301       addne r3, #1
/// a: be00       bkpt 0x0000
/// ```
const IT_IRQ: &[u16] = &[0x4280, 0xbf06, 0x3101, 0x3201, 0x3301, 0xbe00];

/// ```text
/// 0: 4280       cmp r0, r0
/// 2: bf04       itt eq
/// 4: 6811       ldreq r1, [r2, #0]
/// 6: 3301       addeq r3, #1
/// 8: be00       bkpt 0x0000
/// ```
const IT_ABORT: &[u16] = &[0x4280, 0xbf04, 0x6811, 0x3301, 0xbe00];

/// ```text
/// 0: 4280       cmp r0, r0
/// 2: bf04       itt eq
/// 4: df07       svceq 7
/// 6: 3301       addeq r3, #1
/// 8: be00       bkpt 0x0000
/// ```
const IT_SVC: &[u16] = &[0x4280, 0xbf04, 0xdf07, 0x3301, 0xbe00];

/// ```text
/// 0: f7f0 a000  udf.w #0
/// ```
const UNDEF32: &[u16] = &[0xf7f0, 0xa000];

/// ```text
/// 0: b510       push {r4, lr}
/// 2: 4604       mov r4, r0
/// 4: 2200       movs r2, #0
/// 6: f810 3b01  ldrb.w r3, [r0], #1
/// a: f801 3b01  strb.w r3, [r1], #1
/// e: 441a       add r2, r3
/// 10: 2b00       cmp r3, #0
/// 12: d1f8       bne.n 6 <copy>
/// 14: 1b00       subs r0, r0, r4
/// 16: 3801       subs r0, #1
/// 18: 2300       movs r3, #0
/// 1a: f04f 0c0a  mov.w ip, #10
/// 1e: 4463       add r3, ip
/// 20: f1bc 0c01  subs.w ip, ip, #1
/// 24: d1fb       bne.n 1e <loop>
/// 26: f000 f801  bl 2c <square>
/// 2a: bd10       pop {r4, pc}
/// 2c: fb03 f503  mul.w r5, r3, r3
/// 30: f245 6678  movw r6, #22136 @ 0x5678
/// 34: f2c1 2634  movt r6, #4660 @ 0x1234
/// 38: f3c6 2707  ubfx r7, r6, #8, #8
/// 3c: 4770       bx lr
/// ```
const PROGRAM: &[u16] = &[
    0xb510, 0x4604, 0x2200, 0xf810, 0x3b01, 0xf801, 0x3b01, 0x441a, 0x2b00, 0xd1f8, 0x1b00, 0x3801,
    0x2300, 0xf04f, 0x0c0a, 0x4463, 0xf1bc, 0x0c01, 0xd1fb, 0xf000, 0xf801, 0xbd10, 0xfb03, 0xf503,
    0xf245, 0x6678, 0xf2c1, 0x2634, 0xf3c6, 0x2707, 0x4770,
];

/// ```text
/// 0: ee00 0a10  vmov s0, r0
/// 4: ee00 1a90  vmov s1, r1
/// 8: ee30 1a20  vadd.f32 s2, s0, s1
/// c: ee11 2a10  vmov r2, s2
/// 10: be00       bkpt 0x0000
/// ```
const VFP: &[u16] = &[
    0xee00, 0x0a10, 0xee00, 0x1a90, 0xee30, 0x1a20, 0xee11, 0x2a10, 0xbe00,
];

/// ```text
/// 0: ee10 0f10  mrc 15, 0, r0, cr0, cr0, {0}
/// 4: be00       bkpt 0x0000
/// ```
const CP15: &[u16] = &[0xee10, 0x0f10, 0xbe00];

/// ```text
/// 0: e82d c013  srsdb sp!, #19
/// 4: e9bd c000  rfeia sp!
/// ```
const SRS_RFE: &[u16] = &[0xe82d, 0xc013, 0xe9bd, 0xc000];

// ---------------------------------------------------------------------------
// The objdump corpus
// ---------------------------------------------------------------------------

/// `(address, objdump text)` for the corpus image; see `T32_CORPUS_IMAGE`.
/// Generated by assembling with GNU `as` and listing with GNU `objdump -d`,
/// normalised: lower case, comments dropped, branch targets as `0x%08x`.
const T32_CORPUS: &[(u32, &str)] = &[
    (0x0, "and.w r1, r6, #0"),
    (0x4, "ands.w r2, r7, #1"),
    (0x8, "and.w r3, r8, #255"),
    (0xc, "ands.w r4, r9, #256"),
    (0x10, "and.w r5, r10, #2139095040"),
    (0x14, "ands.w r6, r11, #1426085120"),
    (0x18, "bic.w r1, r6, #0"),
    (0x1c, "bics.w r2, r7, #1"),
    (0x20, "bic.w r3, r8, #255"),
    (0x24, "bics.w r4, r9, #256"),
    (0x28, "bic.w r5, r10, #2139095040"),
    (0x2c, "bics.w r6, r11, #1426085120"),
    (0x30, "orr.w r1, r6, #0"),
    (0x34, "orrs.w r2, r7, #1"),
    (0x38, "orr.w r3, r8, #255"),
    (0x3c, "orrs.w r4, r9, #256"),
    (0x40, "orr.w r5, r10, #2139095040"),
    (0x44, "orrs.w r6, r11, #1426085120"),
    (0x48, "orn r1, r6, #0"),
    (0x4c, "orns r2, r7, #1"),
    (0x50, "orn r3, r8, #255"),
    (0x54, "orns r4, r9, #256"),
    (0x58, "orn r5, r10, #2139095040"),
    (0x5c, "orns r6, r11, #1426085120"),
    (0x60, "eor.w r1, r6, #0"),
    (0x64, "eors.w r2, r7, #1"),
    (0x68, "eor.w r3, r8, #255"),
    (0x6c, "eors.w r4, r9, #256"),
    (0x70, "eor.w r5, r10, #2139095040"),
    (0x74, "eors.w r6, r11, #1426085120"),
    (0x78, "add.w r1, r6, #0"),
    (0x7c, "adds.w r2, r7, #1"),
    (0x80, "add.w r3, r8, #255"),
    (0x84, "adds.w r4, r9, #256"),
    (0x88, "add.w r5, r10, #2139095040"),
    (0x8c, "adds.w r6, r11, #1426085120"),
    (0x90, "adc.w r1, r6, #0"),
    (0x94, "adcs.w r2, r7, #1"),
    (0x98, "adc.w r3, r8, #255"),
    (0x9c, "adcs.w r4, r9, #256"),
    (0xa0, "adc.w r5, r10, #2139095040"),
    (0xa4, "adcs.w r6, r11, #1426085120"),
    (0xa8, "sbc.w r1, r6, #0"),
    (0xac, "sbcs.w r2, r7, #1"),
    (0xb0, "sbc.w r3, r8, #255"),
    (0xb4, "sbcs.w r4, r9, #256"),
    (0xb8, "sbc.w r5, r10, #2139095040"),
    (0xbc, "sbcs.w r6, r11, #1426085120"),
    (0xc0, "sub.w r1, r6, #0"),
    (0xc4, "subs.w r2, r7, #1"),
    (0xc8, "sub.w r3, r8, #255"),
    (0xcc, "subs.w r4, r9, #256"),
    (0xd0, "sub.w r5, r10, #2139095040"),
    (0xd4, "subs.w r6, r11, #1426085120"),
    (0xd8, "rsb r1, r6, #0"),
    (0xdc, "rsbs r2, r7, #1"),
    (0xe0, "rsb r3, r8, #255"),
    (0xe4, "rsbs r4, r9, #256"),
    (0xe8, "rsb r5, r10, #2139095040"),
    (0xec, "rsbs r6, r11, #1426085120"),
    (0xf0, "tst.w r9, #0"),
    (0xf4, "tst.w r9, #1"),
    (0xf8, "tst.w r9, #255"),
    (0xfc, "tst.w r9, #256"),
    (0x100, "teq r9, #0"),
    (0x104, "teq r9, #1"),
    (0x108, "teq r9, #255"),
    (0x10c, "teq r9, #256"),
    (0x110, "cmp.w r9, #0"),
    (0x114, "cmp.w r9, #1"),
    (0x118, "cmp.w r9, #255"),
    (0x11c, "cmp.w r9, #256"),
    (0x120, "cmn.w r9, #0"),
    (0x124, "cmn.w r9, #1"),
    (0x128, "cmn.w r9, #255"),
    (0x12c, "cmn.w r9, #256"),
    (0x130, "mov.w r10, #0"),
    (0x134, "mvn.w r11, #0"),
    (0x138, "mov.w r10, #1"),
    (0x13c, "mvn.w r11, #1"),
    (0x140, "mov.w r10, #255"),
    (0x144, "mvn.w r11, #255"),
    (0x148, "mov.w r10, #256"),
    (0x14c, "mvn.w r11, #256"),
    (0x150, "mov.w r10, #2139095040"),
    (0x154, "mvn.w r11, #2139095040"),
    (0x158, "mov.w r10, #1426085120"),
    (0x15c, "mvn.w r11, #1426085120"),
    (0x160, "mov.w r10, #11206827"),
    (0x164, "mvn.w r11, #11206827"),
    (0x168, "mov.w r10, #2880154539"),
    (0x16c, "mvn.w r11, #2880154539"),
    (0x170, "mov.w r10, #2147483648"),
    (0x174, "mvn.w r11, #2147483648"),
    (0x178, "mov.w r10, #1020"),
    (0x17c, "mvn.w r11, #1020"),
    (0x180, "movs.w r2, #2147483648"),
    (0x184, "and.w r2, r5, r9"),
    (0x188, "ands.w r3, r6, r10, lsl #1"),
    (0x18c, "and.w r4, r7, r11, lsl #31"),
    (0x190, "and.w r5, r8, r12, lsr #1"),
    (0x194, "ands.w r6, r9, r0, lsr #32"),
    (0x198, "and.w r7, r10, r1, asr #7"),
    (0x19c, "and.w r8, r11, r2, asr #32"),
    (0x1a0, "ands.w r9, r12, r3, ror #5"),
    (0x1a4, "and.w r10, r0, r4, rrx"),
    (0x1a8, "bic.w r2, r5, r9"),
    (0x1ac, "bics.w r3, r6, r10, lsl #1"),
    (0x1b0, "bic.w r4, r7, r11, lsl #31"),
    (0x1b4, "bic.w r5, r8, r12, lsr #1"),
    (0x1b8, "bics.w r6, r9, r0, lsr #32"),
    (0x1bc, "bic.w r7, r10, r1, asr #7"),
    (0x1c0, "bic.w r8, r11, r2, asr #32"),
    (0x1c4, "bics.w r9, r12, r3, ror #5"),
    (0x1c8, "bic.w r10, r0, r4, rrx"),
    (0x1cc, "orr.w r2, r5, r9"),
    (0x1d0, "orrs.w r3, r6, r10, lsl #1"),
    (0x1d4, "orr.w r4, r7, r11, lsl #31"),
    (0x1d8, "orr.w r5, r8, r12, lsr #1"),
    (0x1dc, "orrs.w r6, r9, r0, lsr #32"),
    (0x1e0, "orr.w r7, r10, r1, asr #7"),
    (0x1e4, "orr.w r8, r11, r2, asr #32"),
    (0x1e8, "orrs.w r9, r12, r3, ror #5"),
    (0x1ec, "orr.w r10, r0, r4, rrx"),
    (0x1f0, "eor.w r2, r5, r9"),
    (0x1f4, "eors.w r3, r6, r10, lsl #1"),
    (0x1f8, "eor.w r4, r7, r11, lsl #31"),
    (0x1fc, "eor.w r5, r8, r12, lsr #1"),
    (0x200, "eors.w r6, r9, r0, lsr #32"),
    (0x204, "eor.w r7, r10, r1, asr #7"),
    (0x208, "eor.w r8, r11, r2, asr #32"),
    (0x20c, "eors.w r9, r12, r3, ror #5"),
    (0x210, "eor.w r10, r0, r4, rrx"),
    (0x214, "add.w r2, r5, r9"),
    (0x218, "adds.w r3, r6, r10, lsl #1"),
    (0x21c, "add.w r4, r7, r11, lsl #31"),
    (0x220, "add.w r5, r8, r12, lsr #1"),
    (0x224, "adds.w r6, r9, r0, lsr #32"),
    (0x228, "add.w r7, r10, r1, asr #7"),
    (0x22c, "add.w r8, r11, r2, asr #32"),
    (0x230, "adds.w r9, r12, r3, ror #5"),
    (0x234, "add.w r10, r0, r4, rrx"),
    (0x238, "adc.w r2, r5, r9"),
    (0x23c, "adcs.w r3, r6, r10, lsl #1"),
    (0x240, "adc.w r4, r7, r11, lsl #31"),
    (0x244, "adc.w r5, r8, r12, lsr #1"),
    (0x248, "adcs.w r6, r9, r0, lsr #32"),
    (0x24c, "adc.w r7, r10, r1, asr #7"),
    (0x250, "adc.w r8, r11, r2, asr #32"),
    (0x254, "adcs.w r9, r12, r3, ror #5"),
    (0x258, "adc.w r10, r0, r4, rrx"),
    (0x25c, "sbc.w r2, r5, r9"),
    (0x260, "sbcs.w r3, r6, r10, lsl #1"),
    (0x264, "sbc.w r4, r7, r11, lsl #31"),
    (0x268, "sbc.w r5, r8, r12, lsr #1"),
    (0x26c, "sbcs.w r6, r9, r0, lsr #32"),
    (0x270, "sbc.w r7, r10, r1, asr #7"),
    (0x274, "sbc.w r8, r11, r2, asr #32"),
    (0x278, "sbcs.w r9, r12, r3, ror #5"),
    (0x27c, "sbc.w r10, r0, r4, rrx"),
    (0x280, "sub.w r2, r5, r9"),
    (0x284, "subs.w r3, r6, r10, lsl #1"),
    (0x288, "sub.w r4, r7, r11, lsl #31"),
    (0x28c, "sub.w r5, r8, r12, lsr #1"),
    (0x290, "subs.w r6, r9, r0, lsr #32"),
    (0x294, "sub.w r7, r10, r1, asr #7"),
    (0x298, "sub.w r8, r11, r2, asr #32"),
    (0x29c, "subs.w r9, r12, r3, ror #5"),
    (0x2a0, "sub.w r10, r0, r4, rrx"),
    (0x2a4, "rsb r2, r5, r9"),
    (0x2a8, "rsbs r3, r6, r10, lsl #1"),
    (0x2ac, "rsb r4, r7, r11, lsl #31"),
    (0x2b0, "rsb r5, r8, r12, lsr #1"),
    (0x2b4, "rsbs r6, r9, r0, lsr #32"),
    (0x2b8, "rsb r7, r10, r1, asr #7"),
    (0x2bc, "rsb r8, r11, r2, asr #32"),
    (0x2c0, "rsbs r9, r12, r3, ror #5"),
    (0x2c4, "rsb r10, r0, r4, rrx"),
    (0x2c8, "orn r2, r5, r9"),
    (0x2cc, "orns r3, r6, r10, lsl #1"),
    (0x2d0, "orn r4, r7, r11, lsl #31"),
    (0x2d4, "orn r5, r8, r12, lsr #1"),
    (0x2d8, "orns r6, r9, r0, lsr #32"),
    (0x2dc, "orn r7, r10, r1, asr #7"),
    (0x2e0, "orn r8, r11, r2, asr #32"),
    (0x2e4, "orns r9, r12, r3, ror #5"),
    (0x2e8, "orn r10, r0, r4, rrx"),
    (0x2ec, "tst.w r3, r12"),
    (0x2f0, "tst.w r3, r12, lsl #1"),
    (0x2f4, "tst.w r3, r12, lsl #31"),
    (0x2f8, "tst.w r3, r12, lsr #1"),
    (0x2fc, "tst.w r3, r12, lsr #32"),
    (0x300, "teq r3, r12"),
    (0x304, "teq r3, r12, lsl #1"),
    (0x308, "teq r3, r12, lsl #31"),
    (0x30c, "teq r3, r12, lsr #1"),
    (0x310, "teq r3, r12, lsr #32"),
    (0x314, "cmp.w r3, r12"),
    (0x318, "cmp.w r3, r12, lsl #1"),
    (0x31c, "cmp.w r3, r12, lsl #31"),
    (0x320, "cmp.w r3, r12, lsr #1"),
    (0x324, "cmp.w r3, r12, lsr #32"),
    (0x328, "cmn.w r3, r12"),
    (0x32c, "cmn.w r3, r12, lsl #1"),
    (0x330, "cmn.w r3, r12, lsl #31"),
    (0x334, "cmn.w r3, r12, lsr #1"),
    (0x338, "cmn.w r3, r12, lsr #32"),
    (0x33c, "mov.w r8, r9"),
    (0x340, "mvn.w r8, r9"),
    (0x344, "mov.w r8, r9, lsl #1"),
    (0x348, "mvn.w r8, r9, lsl #1"),
    (0x34c, "mov.w r8, r9, lsl #31"),
    (0x350, "mvn.w r8, r9, lsl #31"),
    (0x354, "mov.w r8, r9, lsr #1"),
    (0x358, "mvn.w r8, r9, lsr #1"),
    (0x35c, "mov.w r8, r9, lsr #32"),
    (0x360, "mvn.w r8, r9, lsr #32"),
    (0x364, "mov.w r8, r9, asr #7"),
    (0x368, "mvn.w r8, r9, asr #7"),
    (0x36c, "mov.w r8, r9, asr #32"),
    (0x370, "mvn.w r8, r9, asr #32"),
    (0x374, "mov.w r8, r9, ror #5"),
    (0x378, "mvn.w r8, r9, ror #5"),
    (0x37c, "mov.w r8, r9, rrx"),
    (0x380, "mvn.w r8, r9, rrx"),
    (0x384, "movs.w r8, r9"),
    (0x388, "mov.w r8, r9"),
    (0x38c, "lsl.w r1, r2, r3"),
    (0x390, "lsls.w r9, r10, r11"),
    (0x394, "lsr.w r1, r2, r3"),
    (0x398, "lsrs.w r9, r10, r11"),
    (0x39c, "asr.w r1, r2, r3"),
    (0x3a0, "asrs.w r9, r10, r11"),
    (0x3a4, "ror.w r1, r2, r3"),
    (0x3a8, "rors.w r9, r10, r11"),
    (0x3ac, "pkhbt r0, r1, r2"),
    (0x3b0, "pkhbt r0, r1, r2, lsl #16"),
    (0x3b4, "pkhtb r3, r4, r5, asr #16"),
    (0x3b8, "pkhtb r3, r4, r5, asr #32"),
    (0x3bc, "addw r0, r1, #0"),
    (0x3c0, "addw r0, r1, #4095"),
    (0x3c4, "subw r2, r3, #1234"),
    (0x3c8, "addw r0, sp, #8"),
    (0x3cc, "movw r0, #0"),
    (0x3d0, "movw r12, #65535"),
    (0x3d4, "movt r3, #32768"),
    (0x3d8, "bfi r0, r1, #0, #32"),
    (0x3dc, "bfi r0, r1, #31, #1"),
    (0x3e0, "bfc r5, #8, #16"),
    (0x3e4, "sbfx r0, r1, #0, #32"),
    (0x3e8, "sbfx r0, r1, #31, #1"),
    (0x3ec, "ubfx r7, r8, #4, #12"),
    (0x3f0, "ssat r0, #1, r1"),
    (0x3f4, "ssat r0, #32, r1, lsl #31"),
    (0x3f8, "ssat r0, #16, r1, asr #1"),
    (0x3fc, "usat r0, #0, r1"),
    (0x400, "usat r0, #31, r1, asr #31"),
    (0x404, "ssat16 r2, #1, r3"),
    (0x408, "ssat16 r2, #16, r3"),
    (0x40c, "usat16 r2, #0, r3"),
    (0x410, "usat16 r2, #15, r3"),
    (0x414, "ldr.w r1, [r2]"),
    (0x418, "ldr.w r9, [r10, #4095]"),
    (0x41c, "ldr.w r1, [r2, #-255]"),
    (0x420, "ldr.w r3, [r4, #16]!"),
    (0x424, "ldr.w r3, [r4, #-16]!"),
    (0x428, "ldr.w r5, [r6], #1"),
    (0x42c, "ldr.w r5, [r6], #-255"),
    (0x430, "ldr.w r7, [r8, r9]"),
    (0x434, "ldr.w r7, [r8, r9, lsl #3]"),
    (0x438, "ldrb.w r1, [r2]"),
    (0x43c, "ldrb.w r9, [r10, #4095]"),
    (0x440, "ldrb.w r1, [r2, #-255]"),
    (0x444, "ldrb.w r3, [r4, #16]!"),
    (0x448, "ldrb.w r3, [r4, #-16]!"),
    (0x44c, "ldrb.w r5, [r6], #1"),
    (0x450, "ldrb.w r5, [r6], #-255"),
    (0x454, "ldrb.w r7, [r8, r9]"),
    (0x458, "ldrb.w r7, [r8, r9, lsl #3]"),
    (0x45c, "ldrh.w r1, [r2]"),
    (0x460, "ldrh.w r9, [r10, #4095]"),
    (0x464, "ldrh.w r1, [r2, #-255]"),
    (0x468, "ldrh.w r3, [r4, #16]!"),
    (0x46c, "ldrh.w r3, [r4, #-16]!"),
    (0x470, "ldrh.w r5, [r6], #1"),
    (0x474, "ldrh.w r5, [r6], #-255"),
    (0x478, "ldrh.w r7, [r8, r9]"),
    (0x47c, "ldrh.w r7, [r8, r9, lsl #3]"),
    (0x480, "ldrsb.w r1, [r2]"),
    (0x484, "ldrsb.w r9, [r10, #4095]"),
    (0x488, "ldrsb.w r1, [r2, #-255]"),
    (0x48c, "ldrsb.w r3, [r4, #16]!"),
    (0x490, "ldrsb.w r3, [r4, #-16]!"),
    (0x494, "ldrsb.w r5, [r6], #1"),
    (0x498, "ldrsb.w r5, [r6], #-255"),
    (0x49c, "ldrsb.w r7, [r8, r9]"),
    (0x4a0, "ldrsb.w r7, [r8, r9, lsl #3]"),
    (0x4a4, "ldrsh.w r1, [r2]"),
    (0x4a8, "ldrsh.w r9, [r10, #4095]"),
    (0x4ac, "ldrsh.w r1, [r2, #-255]"),
    (0x4b0, "ldrsh.w r3, [r4, #16]!"),
    (0x4b4, "ldrsh.w r3, [r4, #-16]!"),
    (0x4b8, "ldrsh.w r5, [r6], #1"),
    (0x4bc, "ldrsh.w r5, [r6], #-255"),
    (0x4c0, "ldrsh.w r7, [r8, r9]"),
    (0x4c4, "ldrsh.w r7, [r8, r9, lsl #3]"),
    (0x4c8, "str.w r1, [r2]"),
    (0x4cc, "str.w r9, [r10, #4095]"),
    (0x4d0, "str.w r1, [r2, #-255]"),
    (0x4d4, "str.w r3, [r4, #16]!"),
    (0x4d8, "str.w r3, [r4, #-16]!"),
    (0x4dc, "str.w r5, [r6], #1"),
    (0x4e0, "str.w r5, [r6], #-255"),
    (0x4e4, "str.w r7, [r8, r9]"),
    (0x4e8, "str.w r7, [r8, r9, lsl #3]"),
    (0x4ec, "strb.w r1, [r2]"),
    (0x4f0, "strb.w r9, [r10, #4095]"),
    (0x4f4, "strb.w r1, [r2, #-255]"),
    (0x4f8, "strb.w r3, [r4, #16]!"),
    (0x4fc, "strb.w r3, [r4, #-16]!"),
    (0x500, "strb.w r5, [r6], #1"),
    (0x504, "strb.w r5, [r6], #-255"),
    (0x508, "strb.w r7, [r8, r9]"),
    (0x50c, "strb.w r7, [r8, r9, lsl #3]"),
    (0x510, "strh.w r1, [r2]"),
    (0x514, "strh.w r9, [r10, #4095]"),
    (0x518, "strh.w r1, [r2, #-255]"),
    (0x51c, "strh.w r3, [r4, #16]!"),
    (0x520, "strh.w r3, [r4, #-16]!"),
    (0x524, "strh.w r5, [r6], #1"),
    (0x528, "strh.w r5, [r6], #-255"),
    (0x52c, "strh.w r7, [r8, r9]"),
    (0x530, "strh.w r7, [r8, r9, lsl #3]"),
    (0x534, "ldrt r1, [r2]"),
    (0x538, "ldrt r1, [r2, #255]"),
    (0x53c, "ldrbt r1, [r2]"),
    (0x540, "ldrbt r1, [r2, #255]"),
    (0x544, "ldrht r1, [r2]"),
    (0x548, "ldrht r1, [r2, #255]"),
    (0x54c, "ldrsbt r1, [r2]"),
    (0x550, "ldrsbt r1, [r2, #255]"),
    (0x554, "ldrsht r1, [r2]"),
    (0x558, "ldrsht r1, [r2, #255]"),
    (0x55c, "strt r1, [r2]"),
    (0x560, "strt r1, [r2, #255]"),
    (0x564, "strbt r1, [r2]"),
    (0x568, "strbt r1, [r2, #255]"),
    (0x56c, "strht r1, [r2]"),
    (0x570, "strht r1, [r2, #255]"),
    (0x574, "ldr.w r0, [pc, #8]"),
    (0x578, "ldr.w r0, [pc, #-8]"),
    (0x57c, "ldrb.w r0, [pc, #4]"),
    (0x580, "ldrsh.w r0, [pc, #-4]"),
    (0x584, "ldr.w pc, [r0, #4]"),
    (0x588, "ldr.w pc, [sp], #4"),
    (0x58c, "pld [r0]"),
    (0x590, "pld [r1, #4095]"),
    (0x594, "pld [r2, #-128]"),
    (0x598, "pld [r3, r4, lsl #2]"),
    (0x59c, "pldw [r0]"),
    (0x5a0, "pldw [r1, #4095]"),
    (0x5a4, "pldw [r2, #-128]"),
    (0x5a8, "pldw [r3, r4, lsl #2]"),
    (0x5ac, "pli [r0]"),
    (0x5b0, "pli [r1, #4095]"),
    (0x5b4, "pli [r2, #-128]"),
    (0x5b8, "pli [r3, r4, lsl #2]"),
    (0x5bc, "pld [pc, #16]"),
    (0x5c0, "pli [pc, #-16]"),
    (0x5c4, "ldrd r0, r1, [r2]"),
    (0x5c8, "ldrd r0, r2, [r3, #1020]"),
    (0x5cc, "ldrd r4, r5, [r6, #-8]!"),
    (0x5d0, "strd r4, r5, [r6], #8"),
    (0x5d4, "strd r8, r9, [sp, #-1020]"),
    (0x5d8, "ldrd r0, r1, [pc, #8]"),
    (0x5dc, "ldrex r0, [r1]"),
    (0x5e0, "ldrex r0, [r1, #1020]"),
    (0x5e4, "strex r2, r0, [r1]"),
    (0x5e8, "strex r2, r0, [r1, #4]"),
    (0x5ec, "ldrexb r3, [r4]"),
    (0x5f0, "ldrexh r3, [r4]"),
    (0x5f4, "ldrexd r0, r1, [r2]"),
    (0x5f8, "ldrexd r4, r9, [r2]"),
    (0x5fc, "strexb r5, r6, [r7]"),
    (0x600, "strexh r5, r6, [r7]"),
    (0x604, "strexd r5, r2, r3, [r7]"),
    (0x608, "tbb [r0, r1]"),
    (0x60c, "tbh [r0, r1, lsl #1]"),
    (0x610, "tbb [pc, r2]"),
    (0x614, "ldmia.w r0!, {r1, r2, r3}"),
    (0x618, "ldmia.w r0, {r1, r9}"),
    (0x61c, "stmia.w r0!, {r1, r12, lr}"),
    (0x620, "ldmdb r4!, {r5, r6}"),
    (0x624, "stmdb r4, {r5, r6, lr}"),
    (0x628, "stmdb sp!, {r4, r5, r6, r7, r8, r9, r10, r11, lr}"),
    (0x62c, "ldmia.w sp!, {r4, r5, r6, r7, r8, r9, r10, r11, pc}"),
    (0x630, "str.w r8, [sp, #-4]!"),
    (0x634, "ldmia.w sp!, {r0, pc}"),
    (0x638, "srsdb sp!, #19"),
    (0x63c, "srsia sp, #31"),
    (0x640, "rfedb r0"),
    (0x644, "rfeia r1!"),
    (0x648, "sxth.w r1, r2"),
    (0x64c, "sxth.w r9, r10, ror #8"),
    (0x650, "sxth.w r9, r10, ror #24"),
    (0x654, "uxth.w r1, r2"),
    (0x658, "uxth.w r9, r10, ror #8"),
    (0x65c, "uxth.w r9, r10, ror #24"),
    (0x660, "sxtb16 r1, r2"),
    (0x664, "sxtb16 r9, r10, ror #8"),
    (0x668, "sxtb16 r9, r10, ror #24"),
    (0x66c, "uxtb16 r1, r2"),
    (0x670, "uxtb16 r9, r10, ror #8"),
    (0x674, "uxtb16 r9, r10, ror #24"),
    (0x678, "sxtb.w r1, r2"),
    (0x67c, "sxtb.w r9, r10, ror #8"),
    (0x680, "sxtb.w r9, r10, ror #24"),
    (0x684, "uxtb.w r1, r2"),
    (0x688, "uxtb.w r9, r10, ror #8"),
    (0x68c, "uxtb.w r9, r10, ror #24"),
    (0x690, "sxtah r1, r2, r3"),
    (0x694, "sxtah r1, r2, r3, ror #16"),
    (0x698, "uxtah r1, r2, r3"),
    (0x69c, "uxtah r1, r2, r3, ror #16"),
    (0x6a0, "sxtab16 r1, r2, r3"),
    (0x6a4, "sxtab16 r1, r2, r3, ror #16"),
    (0x6a8, "uxtab16 r1, r2, r3"),
    (0x6ac, "uxtab16 r1, r2, r3, ror #16"),
    (0x6b0, "sxtab r1, r2, r3"),
    (0x6b4, "sxtab r1, r2, r3, ror #16"),
    (0x6b8, "uxtab r1, r2, r3"),
    (0x6bc, "uxtab r1, r2, r3, ror #16"),
    (0x6c0, "sadd16 r1, r2, r3"),
    (0x6c4, "sasx r1, r2, r3"),
    (0x6c8, "ssax r1, r2, r3"),
    (0x6cc, "ssub16 r1, r2, r3"),
    (0x6d0, "sadd8 r1, r2, r3"),
    (0x6d4, "ssub8 r1, r2, r3"),
    (0x6d8, "qadd16 r1, r2, r3"),
    (0x6dc, "qasx r1, r2, r3"),
    (0x6e0, "qsax r1, r2, r3"),
    (0x6e4, "qsub16 r1, r2, r3"),
    (0x6e8, "qadd8 r1, r2, r3"),
    (0x6ec, "qsub8 r1, r2, r3"),
    (0x6f0, "shadd16 r1, r2, r3"),
    (0x6f4, "shasx r1, r2, r3"),
    (0x6f8, "shsax r1, r2, r3"),
    (0x6fc, "shsub16 r1, r2, r3"),
    (0x700, "shadd8 r1, r2, r3"),
    (0x704, "shsub8 r1, r2, r3"),
    (0x708, "uadd16 r1, r2, r3"),
    (0x70c, "uasx r1, r2, r3"),
    (0x710, "usax r1, r2, r3"),
    (0x714, "usub16 r1, r2, r3"),
    (0x718, "uadd8 r1, r2, r3"),
    (0x71c, "usub8 r1, r2, r3"),
    (0x720, "uqadd16 r1, r2, r3"),
    (0x724, "uqasx r1, r2, r3"),
    (0x728, "uqsax r1, r2, r3"),
    (0x72c, "uqsub16 r1, r2, r3"),
    (0x730, "uqadd8 r1, r2, r3"),
    (0x734, "uqsub8 r1, r2, r3"),
    (0x738, "uhadd16 r1, r2, r3"),
    (0x73c, "uhasx r1, r2, r3"),
    (0x740, "uhsax r1, r2, r3"),
    (0x744, "uhsub16 r1, r2, r3"),
    (0x748, "uhadd8 r1, r2, r3"),
    (0x74c, "uhsub8 r1, r2, r3"),
    (0x750, "qadd r1, r2, r3"),
    (0x754, "qdadd r1, r2, r3"),
    (0x758, "qsub r1, r2, r3"),
    (0x75c, "qdsub r1, r2, r3"),
    (0x760, "rev.w r1, r2"),
    (0x764, "rev16.w r1, r2"),
    (0x768, "revsh.w r1, r2"),
    (0x76c, "rbit r1, r2"),
    (0x770, "rev.w r9, r10"),
    (0x774, "sel r1, r2, r3"),
    (0x778, "clz r1, r2"),
    (0x77c, "mul.w r1, r2, r3"),
    (0x780, "mul.w r9, r10, r11"),
    (0x784, "mla r1, r2, r3, r4"),
    (0x788, "mls r1, r2, r3, r4"),
    (0x78c, "smulbb r1, r2, r3"),
    (0x790, "smlabb r1, r2, r3, r4"),
    (0x794, "smlalbb r1, r2, r3, r4"),
    (0x798, "smulbt r1, r2, r3"),
    (0x79c, "smlabt r1, r2, r3, r4"),
    (0x7a0, "smlalbt r1, r2, r3, r4"),
    (0x7a4, "smultb r1, r2, r3"),
    (0x7a8, "smlatb r1, r2, r3, r4"),
    (0x7ac, "smlaltb r1, r2, r3, r4"),
    (0x7b0, "smultt r1, r2, r3"),
    (0x7b4, "smlatt r1, r2, r3, r4"),
    (0x7b8, "smlaltt r1, r2, r3, r4"),
    (0x7bc, "smulwb r1, r2, r3"),
    (0x7c0, "smlawb r1, r2, r3, r4"),
    (0x7c4, "smulwt r1, r2, r3"),
    (0x7c8, "smlawt r1, r2, r3, r4"),
    (0x7cc, "smuad r1, r2, r3"),
    (0x7d0, "smuadx r1, r2, r3"),
    (0x7d4, "smusd r1, r2, r3"),
    (0x7d8, "smusdx r1, r2, r3"),
    (0x7dc, "smlad r1, r2, r3, r4"),
    (0x7e0, "smladx r1, r2, r3, r4"),
    (0x7e4, "smlsd r1, r2, r3, r4"),
    (0x7e8, "smlsdx r1, r2, r3, r4"),
    (0x7ec, "smlald r1, r2, r3, r4"),
    (0x7f0, "smlaldx r1, r2, r3, r4"),
    (0x7f4, "smlsld r1, r2, r3, r4"),
    (0x7f8, "smlsldx r1, r2, r3, r4"),
    (0x7fc, "smmul r1, r2, r3"),
    (0x800, "smmulr r1, r2, r3"),
    (0x804, "smmla r1, r2, r3, r4"),
    (0x808, "smmlar r1, r2, r3, r4"),
    (0x80c, "smmls r1, r2, r3, r4"),
    (0x810, "smmlsr r1, r2, r3, r4"),
    (0x814, "usad8 r1, r2, r3"),
    (0x818, "usada8 r1, r2, r3, r4"),
    (0x81c, "smull r1, r2, r3, r4"),
    (0x820, "umull r1, r2, r3, r4"),
    (0x824, "smlal r1, r2, r3, r4"),
    (0x828, "umlal r1, r2, r3, r4"),
    (0x82c, "umaal r1, r2, r3, r4"),
    (0x830, "sdiv r1, r2, r3"),
    (0x834, "udiv r9, r10, r11"),
    (0x838, "b.w 0x00000a0c"),
    (0x83c, "bl 0x00000a0c"),
    (0x840, "blx 0x00000a10"),
    (0x844, "beq.w 0x00000a0c"),
    (0x848, "bne.w 0x00000a0c"),
    (0x84c, "bcs.w 0x00000a0c"),
    (0x850, "bcc.w 0x00000a0c"),
    (0x854, "bmi.w 0x00000a0c"),
    (0x858, "bpl.w 0x00000a0c"),
    (0x85c, "bvs.w 0x00000a0c"),
    (0x860, "bvc.w 0x00000a0c"),
    (0x864, "bhi.w 0x00000a0c"),
    (0x868, "bls.w 0x00000a0c"),
    (0x86c, "bge.w 0x00000a0c"),
    (0x870, "blt.w 0x00000a0c"),
    (0x874, "bgt.w 0x00000a0c"),
    (0x878, "ble.w 0x00000a0c"),
    (0x87c, "mrs r0, CPSR"),
    (0x880, "mrs r9, SPSR"),
    (0x884, "msr CPSR_f, r0"),
    (0x888, "msr CPSR_fsxc, r1"),
    (0x88c, "msr SPSR_c, r2"),
    (0x890, "msr SPSR_fs, r2"),
    (0x894, "cpsid.w i"),
    (0x898, "cpsie.w if"),
    (0x89c, "cpsid aif, #19"),
    (0x8a0, "cpsie f, #31"),
    (0x8a4, "cps #16"),
    (0x8a8, "nop.w"),
    (0x8ac, "yield.w"),
    (0x8b0, "wfe.w"),
    (0x8b4, "wfi.w"),
    (0x8b8, "sev.w"),
    (0x8bc, "dbg #5"),
    (0x8c0, "dmb sy"),
    (0x8c4, "dmb ish"),
    (0x8c8, "dmb ishst"),
    (0x8cc, "dsb osh"),
    (0x8d0, "dsb un"),
    (0x8d4, "isb sy"),
    (0x8d8, "clrex"),
    (0x8dc, "bxj r5"),
    (0x8e0, "subs pc, lr, #0"),
    (0x8e4, "subs pc, lr, #255"),
    (0x8e8, "smc #3"),
    (0x8ec, "udf.w #0"),
    (0x8f0, "udf.w #65535"),
    (0x8f4, "vadd.f32 s0, s1, s2"),
    (0x8f8, "vsub.f64 d0, d1, d2"),
    (0x8fc, "vmul.f32 s3, s4, s5"),
    (0x900, "vdiv.f64 d8, d9, d10"),
    (0x904, "vmov.f32 s0, #112"),
    (0x908, "vmov r0, s1"),
    (0x90c, "vmov s2, r3"),
    (0x910, "vmov r0, r1, d2"),
    (0x914, "vcmp.f32 s0, s1"),
    (0x918, "vcmpe.f64 d0, #0.0"),
    (0x91c, "vcvt.s32.f64 s0, d1"),
    (0x920, "vcvt.f64.f32 d0, s1"),
    (0x924, "vldr d0, [r0, #8]"),
    (0x928, "vstr s1, [r2, #-4]"),
    (0x92c, "vldmia r0!, {d0-d3}"),
    (0x930, "vpush {d8-d15}"),
    (0x934, "vpop {s16-s31}"),
    (0x938, "vmrs r0, fpscr"),
    (0x93c, "vmrs APSR_nzcv, fpscr"),
    (0x940, "vmsr fpscr, r1"),
    (0x944, "vsqrt.f32 s0, s1"),
    (0x948, "vabs.f64 d0, d1"),
    (0x94c, "vneg.f32 s0, s1"),
    (0x950, "movs r0, r1"),
    (0x952, "lsls r0, r1, #3"),
    (0x954, "lsrs r0, r1, #32"),
    (0x956, "asrs r2, r3, #1"),
    (0x958, "adds r0, r1, r2"),
    (0x95a, "subs r0, r1, #7"),
    (0x95c, "movs r0, #255"),
    (0x95e, "cmp r0, #10"),
    (0x960, "adds r3, #1"),
    (0x962, "subs r3, #200"),
    (0x964, "ands r1, r2"),
    (0x966, "eors r1, r2"),
    (0x968, "lsls r1, r2"),
    (0x96a, "lsrs r1, r2"),
    (0x96c, "asrs r1, r2"),
    (0x96e, "adcs r1, r2"),
    (0x970, "sbcs r1, r2"),
    (0x972, "rors r1, r2"),
    (0x974, "tst r1, r2"),
    (0x976, "cmp r1, r2"),
    (0x978, "cmn r1, r2"),
    (0x97a, "orrs r1, r2"),
    (0x97c, "bics r1, r2"),
    (0x97e, "mvns r1, r2"),
    (0x980, "negs r1, r2"),
    (0x982, "muls r1, r2"),
    (0x984, "add r8, r1"),
    (0x986, "add r1, r8"),
    (0x988, "mov r8, r9"),
    (0x98a, "mov r0, r1"),
    (0x98c, "cmp r8, r9"),
    (0x98e, "bx lr"),
    (0x990, "blx r3"),
    (0x992, "ldr r0, [pc, #16]"),
    (0x994, "ldr r0, [r1, r2]"),
    (0x996, "strb r0, [r1, r2]"),
    (0x998, "ldrsh r0, [r1, r2]"),
    (0x99a, "ldr r0, [r1, #4]"),
    (0x99c, "ldr r0, [r1, #0]"),
    (0x99e, "ldrb r0, [r1, #31]"),
    (0x9a0, "strh r0, [r1, #62]"),
    (0x9a2, "ldr r0, [sp, #1020]"),
    (0x9a4, "str r1, [sp, #4]"),
    (0x9a6, "add r0, sp, #4"),
    (0x9a8, "add sp, #8"),
    (0x9aa, "sub sp, #508"),
    (0x9ac, "push {r4, r5, lr}"),
    (0x9ae, "pop {r0, pc}"),
    (0x9b0, "ldmia r0!, {r1, r2}"),
    (0x9b2, "ldmia r0, {r0, r1}"),
    (0x9b4, "stmia r1!, {r2, r3}"),
    (0x9b6, "svc 5"),
    (0x9b8, "rev r0, r1"),
    (0x9ba, "rev16 r2, r3"),
    (0x9bc, "revsh r4, r5"),
    (0x9be, "sxth r0, r1"),
    (0x9c0, "sxtb r0, r1"),
    (0x9c2, "uxth r0, r1"),
    (0x9c4, "uxtb r0, r1"),
    (0x9c6, "cpsie i"),
    (0x9c8, "cpsid if"),
    (0x9ca, "setend be"),
    (0x9cc, "setend le"),
    (0x9ce, "nop"),
    (0x9d0, "yield"),
    (0x9d2, "wfe"),
    (0x9d4, "wfi"),
    (0x9d6, "sev"),
    (0x9d8, "udf #7"),
    (0x9da, "cbz r0, 0x00000a0a"),
    (0x9dc, "cbnz r7, 0x00000a0a"),
    (0x9de, "b.n 0x00000a0a"),
    (0x9e0, "beq.n 0x00000a0a"),
    (0x9e2, "it eq"),
    (0x9e4, "moveq r0, r1"),
    (0x9e6, "ite ne"),
    (0x9e8, "addne r0, #1"),
    (0x9ea, "addeq r0, #2"),
    (0x9ec, "ittee gt"),
    (0x9ee, "movgt r0, #1"),
    (0x9f0, "lslgt r1, r2, #3"),
    (0x9f2, "addle.w r0, r1, #256"),
    (0x9f6, "ldrle.w r1, [r2, #4]"),
    (0x9fa, "itt cs"),
    (0x9fc, "mulcs r1, r2"),
    (0x9fe, "bxcs lr"),
    (0xa00, "ittt mi"),
    (0xa02, "addmi r0, r0, r1"),
    (0xa04, "ldrmi r0, [r1, #0]"),
    (0xa06, "bmi.w 0x00000a0c"),
    (0xa0a, "nop"),
    (0xa0c, "nop.w"),
    (0xa10, "nop.w"),
];

/// The assembled bytes the listing above describes, loaded at address 0.
const T32_CORPUS_IMAGE: &[u8] = &[
    0x06, 0xf0, 0x00, 0x01, 0x17, 0xf0, 0x01, 0x02, 0x08, 0xf0, 0xff, 0x03, 0x19, 0xf4, 0x80, 0x74,
    0x0a, 0xf0, 0xff, 0x45, 0x1b, 0xf0, 0x55, 0x26, 0x26, 0xf0, 0x00, 0x01, 0x37, 0xf0, 0x01, 0x02,
    0x28, 0xf0, 0xff, 0x03, 0x39, 0xf4, 0x80, 0x74, 0x2a, 0xf0, 0xff, 0x45, 0x3b, 0xf0, 0x55, 0x26,
    0x46, 0xf0, 0x00, 0x01, 0x57, 0xf0, 0x01, 0x02, 0x48, 0xf0, 0xff, 0x03, 0x59, 0xf4, 0x80, 0x74,
    0x4a, 0xf0, 0xff, 0x45, 0x5b, 0xf0, 0x55, 0x26, 0x66, 0xf0, 0x00, 0x01, 0x77, 0xf0, 0x01, 0x02,
    0x68, 0xf0, 0xff, 0x03, 0x79, 0xf4, 0x80, 0x74, 0x6a, 0xf0, 0xff, 0x45, 0x7b, 0xf0, 0x55, 0x26,
    0x86, 0xf0, 0x00, 0x01, 0x97, 0xf0, 0x01, 0x02, 0x88, 0xf0, 0xff, 0x03, 0x99, 0xf4, 0x80, 0x74,
    0x8a, 0xf0, 0xff, 0x45, 0x9b, 0xf0, 0x55, 0x26, 0x06, 0xf1, 0x00, 0x01, 0x17, 0xf1, 0x01, 0x02,
    0x08, 0xf1, 0xff, 0x03, 0x19, 0xf5, 0x80, 0x74, 0x0a, 0xf1, 0xff, 0x45, 0x1b, 0xf1, 0x55, 0x26,
    0x46, 0xf1, 0x00, 0x01, 0x57, 0xf1, 0x01, 0x02, 0x48, 0xf1, 0xff, 0x03, 0x59, 0xf5, 0x80, 0x74,
    0x4a, 0xf1, 0xff, 0x45, 0x5b, 0xf1, 0x55, 0x26, 0x66, 0xf1, 0x00, 0x01, 0x77, 0xf1, 0x01, 0x02,
    0x68, 0xf1, 0xff, 0x03, 0x79, 0xf5, 0x80, 0x74, 0x6a, 0xf1, 0xff, 0x45, 0x7b, 0xf1, 0x55, 0x26,
    0xa6, 0xf1, 0x00, 0x01, 0xb7, 0xf1, 0x01, 0x02, 0xa8, 0xf1, 0xff, 0x03, 0xb9, 0xf5, 0x80, 0x74,
    0xaa, 0xf1, 0xff, 0x45, 0xbb, 0xf1, 0x55, 0x26, 0xc6, 0xf1, 0x00, 0x01, 0xd7, 0xf1, 0x01, 0x02,
    0xc8, 0xf1, 0xff, 0x03, 0xd9, 0xf5, 0x80, 0x74, 0xca, 0xf1, 0xff, 0x45, 0xdb, 0xf1, 0x55, 0x26,
    0x19, 0xf0, 0x00, 0x0f, 0x19, 0xf0, 0x01, 0x0f, 0x19, 0xf0, 0xff, 0x0f, 0x19, 0xf4, 0x80, 0x7f,
    0x99, 0xf0, 0x00, 0x0f, 0x99, 0xf0, 0x01, 0x0f, 0x99, 0xf0, 0xff, 0x0f, 0x99, 0xf4, 0x80, 0x7f,
    0xb9, 0xf1, 0x00, 0x0f, 0xb9, 0xf1, 0x01, 0x0f, 0xb9, 0xf1, 0xff, 0x0f, 0xb9, 0xf5, 0x80, 0x7f,
    0x19, 0xf1, 0x00, 0x0f, 0x19, 0xf1, 0x01, 0x0f, 0x19, 0xf1, 0xff, 0x0f, 0x19, 0xf5, 0x80, 0x7f,
    0x4f, 0xf0, 0x00, 0x0a, 0x6f, 0xf0, 0x00, 0x0b, 0x4f, 0xf0, 0x01, 0x0a, 0x6f, 0xf0, 0x01, 0x0b,
    0x4f, 0xf0, 0xff, 0x0a, 0x6f, 0xf0, 0xff, 0x0b, 0x4f, 0xf4, 0x80, 0x7a, 0x6f, 0xf4, 0x80, 0x7b,
    0x4f, 0xf0, 0xff, 0x4a, 0x6f, 0xf0, 0xff, 0x4b, 0x4f, 0xf0, 0x55, 0x2a, 0x6f, 0xf0, 0x55, 0x2b,
    0x4f, 0xf0, 0xab, 0x1a, 0x6f, 0xf0, 0xab, 0x1b, 0x4f, 0xf0, 0xab, 0x3a, 0x6f, 0xf0, 0xab, 0x3b,
    0x4f, 0xf0, 0x00, 0x4a, 0x6f, 0xf0, 0x00, 0x4b, 0x4f, 0xf4, 0x7f, 0x7a, 0x6f, 0xf4, 0x7f, 0x7b,
    0x5f, 0xf0, 0x00, 0x42, 0x05, 0xea, 0x09, 0x02, 0x16, 0xea, 0x4a, 0x03, 0x07, 0xea, 0xcb, 0x74,
    0x08, 0xea, 0x5c, 0x05, 0x19, 0xea, 0x10, 0x06, 0x0a, 0xea, 0xe1, 0x17, 0x0b, 0xea, 0x22, 0x08,
    0x1c, 0xea, 0x73, 0x19, 0x00, 0xea, 0x34, 0x0a, 0x25, 0xea, 0x09, 0x02, 0x36, 0xea, 0x4a, 0x03,
    0x27, 0xea, 0xcb, 0x74, 0x28, 0xea, 0x5c, 0x05, 0x39, 0xea, 0x10, 0x06, 0x2a, 0xea, 0xe1, 0x17,
    0x2b, 0xea, 0x22, 0x08, 0x3c, 0xea, 0x73, 0x19, 0x20, 0xea, 0x34, 0x0a, 0x45, 0xea, 0x09, 0x02,
    0x56, 0xea, 0x4a, 0x03, 0x47, 0xea, 0xcb, 0x74, 0x48, 0xea, 0x5c, 0x05, 0x59, 0xea, 0x10, 0x06,
    0x4a, 0xea, 0xe1, 0x17, 0x4b, 0xea, 0x22, 0x08, 0x5c, 0xea, 0x73, 0x19, 0x40, 0xea, 0x34, 0x0a,
    0x85, 0xea, 0x09, 0x02, 0x96, 0xea, 0x4a, 0x03, 0x87, 0xea, 0xcb, 0x74, 0x88, 0xea, 0x5c, 0x05,
    0x99, 0xea, 0x10, 0x06, 0x8a, 0xea, 0xe1, 0x17, 0x8b, 0xea, 0x22, 0x08, 0x9c, 0xea, 0x73, 0x19,
    0x80, 0xea, 0x34, 0x0a, 0x05, 0xeb, 0x09, 0x02, 0x16, 0xeb, 0x4a, 0x03, 0x07, 0xeb, 0xcb, 0x74,
    0x08, 0xeb, 0x5c, 0x05, 0x19, 0xeb, 0x10, 0x06, 0x0a, 0xeb, 0xe1, 0x17, 0x0b, 0xeb, 0x22, 0x08,
    0x1c, 0xeb, 0x73, 0x19, 0x00, 0xeb, 0x34, 0x0a, 0x45, 0xeb, 0x09, 0x02, 0x56, 0xeb, 0x4a, 0x03,
    0x47, 0xeb, 0xcb, 0x74, 0x48, 0xeb, 0x5c, 0x05, 0x59, 0xeb, 0x10, 0x06, 0x4a, 0xeb, 0xe1, 0x17,
    0x4b, 0xeb, 0x22, 0x08, 0x5c, 0xeb, 0x73, 0x19, 0x40, 0xeb, 0x34, 0x0a, 0x65, 0xeb, 0x09, 0x02,
    0x76, 0xeb, 0x4a, 0x03, 0x67, 0xeb, 0xcb, 0x74, 0x68, 0xeb, 0x5c, 0x05, 0x79, 0xeb, 0x10, 0x06,
    0x6a, 0xeb, 0xe1, 0x17, 0x6b, 0xeb, 0x22, 0x08, 0x7c, 0xeb, 0x73, 0x19, 0x60, 0xeb, 0x34, 0x0a,
    0xa5, 0xeb, 0x09, 0x02, 0xb6, 0xeb, 0x4a, 0x03, 0xa7, 0xeb, 0xcb, 0x74, 0xa8, 0xeb, 0x5c, 0x05,
    0xb9, 0xeb, 0x10, 0x06, 0xaa, 0xeb, 0xe1, 0x17, 0xab, 0xeb, 0x22, 0x08, 0xbc, 0xeb, 0x73, 0x19,
    0xa0, 0xeb, 0x34, 0x0a, 0xc5, 0xeb, 0x09, 0x02, 0xd6, 0xeb, 0x4a, 0x03, 0xc7, 0xeb, 0xcb, 0x74,
    0xc8, 0xeb, 0x5c, 0x05, 0xd9, 0xeb, 0x10, 0x06, 0xca, 0xeb, 0xe1, 0x17, 0xcb, 0xeb, 0x22, 0x08,
    0xdc, 0xeb, 0x73, 0x19, 0xc0, 0xeb, 0x34, 0x0a, 0x65, 0xea, 0x09, 0x02, 0x76, 0xea, 0x4a, 0x03,
    0x67, 0xea, 0xcb, 0x74, 0x68, 0xea, 0x5c, 0x05, 0x79, 0xea, 0x10, 0x06, 0x6a, 0xea, 0xe1, 0x17,
    0x6b, 0xea, 0x22, 0x08, 0x7c, 0xea, 0x73, 0x19, 0x60, 0xea, 0x34, 0x0a, 0x13, 0xea, 0x0c, 0x0f,
    0x13, 0xea, 0x4c, 0x0f, 0x13, 0xea, 0xcc, 0x7f, 0x13, 0xea, 0x5c, 0x0f, 0x13, 0xea, 0x1c, 0x0f,
    0x93, 0xea, 0x0c, 0x0f, 0x93, 0xea, 0x4c, 0x0f, 0x93, 0xea, 0xcc, 0x7f, 0x93, 0xea, 0x5c, 0x0f,
    0x93, 0xea, 0x1c, 0x0f, 0xb3, 0xeb, 0x0c, 0x0f, 0xb3, 0xeb, 0x4c, 0x0f, 0xb3, 0xeb, 0xcc, 0x7f,
    0xb3, 0xeb, 0x5c, 0x0f, 0xb3, 0xeb, 0x1c, 0x0f, 0x13, 0xeb, 0x0c, 0x0f, 0x13, 0xeb, 0x4c, 0x0f,
    0x13, 0xeb, 0xcc, 0x7f, 0x13, 0xeb, 0x5c, 0x0f, 0x13, 0xeb, 0x1c, 0x0f, 0x4f, 0xea, 0x09, 0x08,
    0x6f, 0xea, 0x09, 0x08, 0x4f, 0xea, 0x49, 0x08, 0x6f, 0xea, 0x49, 0x08, 0x4f, 0xea, 0xc9, 0x78,
    0x6f, 0xea, 0xc9, 0x78, 0x4f, 0xea, 0x59, 0x08, 0x6f, 0xea, 0x59, 0x08, 0x4f, 0xea, 0x19, 0x08,
    0x6f, 0xea, 0x19, 0x08, 0x4f, 0xea, 0xe9, 0x18, 0x6f, 0xea, 0xe9, 0x18, 0x4f, 0xea, 0x29, 0x08,
    0x6f, 0xea, 0x29, 0x08, 0x4f, 0xea, 0x79, 0x18, 0x6f, 0xea, 0x79, 0x18, 0x4f, 0xea, 0x39, 0x08,
    0x6f, 0xea, 0x39, 0x08, 0x5f, 0xea, 0x09, 0x08, 0x4f, 0xea, 0x09, 0x08, 0x02, 0xfa, 0x03, 0xf1,
    0x1a, 0xfa, 0x0b, 0xf9, 0x22, 0xfa, 0x03, 0xf1, 0x3a, 0xfa, 0x0b, 0xf9, 0x42, 0xfa, 0x03, 0xf1,
    0x5a, 0xfa, 0x0b, 0xf9, 0x62, 0xfa, 0x03, 0xf1, 0x7a, 0xfa, 0x0b, 0xf9, 0xc1, 0xea, 0x02, 0x00,
    0xc1, 0xea, 0x02, 0x40, 0xc4, 0xea, 0x25, 0x43, 0xc4, 0xea, 0x25, 0x03, 0x01, 0xf2, 0x00, 0x00,
    0x01, 0xf6, 0xff, 0x70, 0xa3, 0xf2, 0xd2, 0x42, 0x0d, 0xf2, 0x08, 0x00, 0x40, 0xf2, 0x00, 0x00,
    0x4f, 0xf6, 0xff, 0x7c, 0xc8, 0xf2, 0x00, 0x03, 0x61, 0xf3, 0x1f, 0x00, 0x61, 0xf3, 0xdf, 0x70,
    0x6f, 0xf3, 0x17, 0x25, 0x41, 0xf3, 0x1f, 0x00, 0x41, 0xf3, 0xc0, 0x70, 0xc8, 0xf3, 0x0b, 0x17,
    0x01, 0xf3, 0x00, 0x00, 0x01, 0xf3, 0xdf, 0x70, 0x21, 0xf3, 0x4f, 0x00, 0x81, 0xf3, 0x00, 0x00,
    0xa1, 0xf3, 0xdf, 0x70, 0x23, 0xf3, 0x00, 0x02, 0x23, 0xf3, 0x0f, 0x02, 0xa3, 0xf3, 0x00, 0x02,
    0xa3, 0xf3, 0x0f, 0x02, 0xd2, 0xf8, 0x00, 0x10, 0xda, 0xf8, 0xff, 0x9f, 0x52, 0xf8, 0xff, 0x1c,
    0x54, 0xf8, 0x10, 0x3f, 0x54, 0xf8, 0x10, 0x3d, 0x56, 0xf8, 0x01, 0x5b, 0x56, 0xf8, 0xff, 0x59,
    0x58, 0xf8, 0x09, 0x70, 0x58, 0xf8, 0x39, 0x70, 0x92, 0xf8, 0x00, 0x10, 0x9a, 0xf8, 0xff, 0x9f,
    0x12, 0xf8, 0xff, 0x1c, 0x14, 0xf8, 0x10, 0x3f, 0x14, 0xf8, 0x10, 0x3d, 0x16, 0xf8, 0x01, 0x5b,
    0x16, 0xf8, 0xff, 0x59, 0x18, 0xf8, 0x09, 0x70, 0x18, 0xf8, 0x39, 0x70, 0xb2, 0xf8, 0x00, 0x10,
    0xba, 0xf8, 0xff, 0x9f, 0x32, 0xf8, 0xff, 0x1c, 0x34, 0xf8, 0x10, 0x3f, 0x34, 0xf8, 0x10, 0x3d,
    0x36, 0xf8, 0x01, 0x5b, 0x36, 0xf8, 0xff, 0x59, 0x38, 0xf8, 0x09, 0x70, 0x38, 0xf8, 0x39, 0x70,
    0x92, 0xf9, 0x00, 0x10, 0x9a, 0xf9, 0xff, 0x9f, 0x12, 0xf9, 0xff, 0x1c, 0x14, 0xf9, 0x10, 0x3f,
    0x14, 0xf9, 0x10, 0x3d, 0x16, 0xf9, 0x01, 0x5b, 0x16, 0xf9, 0xff, 0x59, 0x18, 0xf9, 0x09, 0x70,
    0x18, 0xf9, 0x39, 0x70, 0xb2, 0xf9, 0x00, 0x10, 0xba, 0xf9, 0xff, 0x9f, 0x32, 0xf9, 0xff, 0x1c,
    0x34, 0xf9, 0x10, 0x3f, 0x34, 0xf9, 0x10, 0x3d, 0x36, 0xf9, 0x01, 0x5b, 0x36, 0xf9, 0xff, 0x59,
    0x38, 0xf9, 0x09, 0x70, 0x38, 0xf9, 0x39, 0x70, 0xc2, 0xf8, 0x00, 0x10, 0xca, 0xf8, 0xff, 0x9f,
    0x42, 0xf8, 0xff, 0x1c, 0x44, 0xf8, 0x10, 0x3f, 0x44, 0xf8, 0x10, 0x3d, 0x46, 0xf8, 0x01, 0x5b,
    0x46, 0xf8, 0xff, 0x59, 0x48, 0xf8, 0x09, 0x70, 0x48, 0xf8, 0x39, 0x70, 0x82, 0xf8, 0x00, 0x10,
    0x8a, 0xf8, 0xff, 0x9f, 0x02, 0xf8, 0xff, 0x1c, 0x04, 0xf8, 0x10, 0x3f, 0x04, 0xf8, 0x10, 0x3d,
    0x06, 0xf8, 0x01, 0x5b, 0x06, 0xf8, 0xff, 0x59, 0x08, 0xf8, 0x09, 0x70, 0x08, 0xf8, 0x39, 0x70,
    0xa2, 0xf8, 0x00, 0x10, 0xaa, 0xf8, 0xff, 0x9f, 0x22, 0xf8, 0xff, 0x1c, 0x24, 0xf8, 0x10, 0x3f,
    0x24, 0xf8, 0x10, 0x3d, 0x26, 0xf8, 0x01, 0x5b, 0x26, 0xf8, 0xff, 0x59, 0x28, 0xf8, 0x09, 0x70,
    0x28, 0xf8, 0x39, 0x70, 0x52, 0xf8, 0x00, 0x1e, 0x52, 0xf8, 0xff, 0x1e, 0x12, 0xf8, 0x00, 0x1e,
    0x12, 0xf8, 0xff, 0x1e, 0x32, 0xf8, 0x00, 0x1e, 0x32, 0xf8, 0xff, 0x1e, 0x12, 0xf9, 0x00, 0x1e,
    0x12, 0xf9, 0xff, 0x1e, 0x32, 0xf9, 0x00, 0x1e, 0x32, 0xf9, 0xff, 0x1e, 0x42, 0xf8, 0x00, 0x1e,
    0x42, 0xf8, 0xff, 0x1e, 0x02, 0xf8, 0x00, 0x1e, 0x02, 0xf8, 0xff, 0x1e, 0x22, 0xf8, 0x00, 0x1e,
    0x22, 0xf8, 0xff, 0x1e, 0xdf, 0xf8, 0x08, 0x00, 0x5f, 0xf8, 0x08, 0x00, 0x9f, 0xf8, 0x04, 0x00,
    0x3f, 0xf9, 0x04, 0x00, 0xd0, 0xf8, 0x04, 0xf0, 0x5d, 0xf8, 0x04, 0xfb, 0x90, 0xf8, 0x00, 0xf0,
    0x91, 0xf8, 0xff, 0xff, 0x12, 0xf8, 0x80, 0xfc, 0x13, 0xf8, 0x24, 0xf0, 0xb0, 0xf8, 0x00, 0xf0,
    0xb1, 0xf8, 0xff, 0xff, 0x32, 0xf8, 0x80, 0xfc, 0x33, 0xf8, 0x24, 0xf0, 0x90, 0xf9, 0x00, 0xf0,
    0x91, 0xf9, 0xff, 0xff, 0x12, 0xf9, 0x80, 0xfc, 0x13, 0xf9, 0x24, 0xf0, 0x9f, 0xf8, 0x10, 0xf0,
    0x1f, 0xf9, 0x10, 0xf0, 0xd2, 0xe9, 0x00, 0x01, 0xd3, 0xe9, 0xff, 0x02, 0x76, 0xe9, 0x02, 0x45,
    0xe6, 0xe8, 0x02, 0x45, 0x4d, 0xe9, 0xff, 0x89, 0xdf, 0xe9, 0x02, 0x01, 0x51, 0xe8, 0x00, 0x0f,
    0x51, 0xe8, 0xff, 0x0f, 0x41, 0xe8, 0x00, 0x02, 0x41, 0xe8, 0x01, 0x02, 0xd4, 0xe8, 0x4f, 0x3f,
    0xd4, 0xe8, 0x5f, 0x3f, 0xd2, 0xe8, 0x7f, 0x01, 0xd2, 0xe8, 0x7f, 0x49, 0xc7, 0xe8, 0x45, 0x6f,
    0xc7, 0xe8, 0x55, 0x6f, 0xc7, 0xe8, 0x75, 0x23, 0xd0, 0xe8, 0x01, 0xf0, 0xd0, 0xe8, 0x11, 0xf0,
    0xdf, 0xe8, 0x02, 0xf0, 0xb0, 0xe8, 0x0e, 0x00, 0x90, 0xe8, 0x02, 0x02, 0xa0, 0xe8, 0x02, 0x50,
    0x34, 0xe9, 0x60, 0x00, 0x04, 0xe9, 0x60, 0x40, 0x2d, 0xe9, 0xf0, 0x4f, 0xbd, 0xe8, 0xf0, 0x8f,
    0x4d, 0xf8, 0x04, 0x8d, 0xbd, 0xe8, 0x01, 0x80, 0x2d, 0xe8, 0x13, 0xc0, 0x8d, 0xe9, 0x1f, 0xc0,
    0x10, 0xe8, 0x00, 0xc0, 0xb1, 0xe9, 0x00, 0xc0, 0x0f, 0xfa, 0x82, 0xf1, 0x0f, 0xfa, 0x9a, 0xf9,
    0x0f, 0xfa, 0xba, 0xf9, 0x1f, 0xfa, 0x82, 0xf1, 0x1f, 0xfa, 0x9a, 0xf9, 0x1f, 0xfa, 0xba, 0xf9,
    0x2f, 0xfa, 0x82, 0xf1, 0x2f, 0xfa, 0x9a, 0xf9, 0x2f, 0xfa, 0xba, 0xf9, 0x3f, 0xfa, 0x82, 0xf1,
    0x3f, 0xfa, 0x9a, 0xf9, 0x3f, 0xfa, 0xba, 0xf9, 0x4f, 0xfa, 0x82, 0xf1, 0x4f, 0xfa, 0x9a, 0xf9,
    0x4f, 0xfa, 0xba, 0xf9, 0x5f, 0xfa, 0x82, 0xf1, 0x5f, 0xfa, 0x9a, 0xf9, 0x5f, 0xfa, 0xba, 0xf9,
    0x02, 0xfa, 0x83, 0xf1, 0x02, 0xfa, 0xa3, 0xf1, 0x12, 0xfa, 0x83, 0xf1, 0x12, 0xfa, 0xa3, 0xf1,
    0x22, 0xfa, 0x83, 0xf1, 0x22, 0xfa, 0xa3, 0xf1, 0x32, 0xfa, 0x83, 0xf1, 0x32, 0xfa, 0xa3, 0xf1,
    0x42, 0xfa, 0x83, 0xf1, 0x42, 0xfa, 0xa3, 0xf1, 0x52, 0xfa, 0x83, 0xf1, 0x52, 0xfa, 0xa3, 0xf1,
    0x92, 0xfa, 0x03, 0xf1, 0xa2, 0xfa, 0x03, 0xf1, 0xe2, 0xfa, 0x03, 0xf1, 0xd2, 0xfa, 0x03, 0xf1,
    0x82, 0xfa, 0x03, 0xf1, 0xc2, 0xfa, 0x03, 0xf1, 0x92, 0xfa, 0x13, 0xf1, 0xa2, 0xfa, 0x13, 0xf1,
    0xe2, 0xfa, 0x13, 0xf1, 0xd2, 0xfa, 0x13, 0xf1, 0x82, 0xfa, 0x13, 0xf1, 0xc2, 0xfa, 0x13, 0xf1,
    0x92, 0xfa, 0x23, 0xf1, 0xa2, 0xfa, 0x23, 0xf1, 0xe2, 0xfa, 0x23, 0xf1, 0xd2, 0xfa, 0x23, 0xf1,
    0x82, 0xfa, 0x23, 0xf1, 0xc2, 0xfa, 0x23, 0xf1, 0x92, 0xfa, 0x43, 0xf1, 0xa2, 0xfa, 0x43, 0xf1,
    0xe2, 0xfa, 0x43, 0xf1, 0xd2, 0xfa, 0x43, 0xf1, 0x82, 0xfa, 0x43, 0xf1, 0xc2, 0xfa, 0x43, 0xf1,
    0x92, 0xfa, 0x53, 0xf1, 0xa2, 0xfa, 0x53, 0xf1, 0xe2, 0xfa, 0x53, 0xf1, 0xd2, 0xfa, 0x53, 0xf1,
    0x82, 0xfa, 0x53, 0xf1, 0xc2, 0xfa, 0x53, 0xf1, 0x92, 0xfa, 0x63, 0xf1, 0xa2, 0xfa, 0x63, 0xf1,
    0xe2, 0xfa, 0x63, 0xf1, 0xd2, 0xfa, 0x63, 0xf1, 0x82, 0xfa, 0x63, 0xf1, 0xc2, 0xfa, 0x63, 0xf1,
    0x83, 0xfa, 0x82, 0xf1, 0x83, 0xfa, 0x92, 0xf1, 0x83, 0xfa, 0xa2, 0xf1, 0x83, 0xfa, 0xb2, 0xf1,
    0x92, 0xfa, 0x82, 0xf1, 0x92, 0xfa, 0x92, 0xf1, 0x92, 0xfa, 0xb2, 0xf1, 0x92, 0xfa, 0xa2, 0xf1,
    0x9a, 0xfa, 0x8a, 0xf9, 0xa2, 0xfa, 0x83, 0xf1, 0xb2, 0xfa, 0x82, 0xf1, 0x02, 0xfb, 0x03, 0xf1,
    0x0a, 0xfb, 0x0b, 0xf9, 0x02, 0xfb, 0x03, 0x41, 0x02, 0xfb, 0x13, 0x41, 0x12, 0xfb, 0x03, 0xf1,
    0x12, 0xfb, 0x03, 0x41, 0xc3, 0xfb, 0x84, 0x12, 0x12, 0xfb, 0x13, 0xf1, 0x12, 0xfb, 0x13, 0x41,
    0xc3, 0xfb, 0x94, 0x12, 0x12, 0xfb, 0x23, 0xf1, 0x12, 0xfb, 0x23, 0x41, 0xc3, 0xfb, 0xa4, 0x12,
    0x12, 0xfb, 0x33, 0xf1, 0x12, 0xfb, 0x33, 0x41, 0xc3, 0xfb, 0xb4, 0x12, 0x32, 0xfb, 0x03, 0xf1,
    0x32, 0xfb, 0x03, 0x41, 0x32, 0xfb, 0x13, 0xf1, 0x32, 0xfb, 0x13, 0x41, 0x22, 0xfb, 0x03, 0xf1,
    0x22, 0xfb, 0x13, 0xf1, 0x42, 0xfb, 0x03, 0xf1, 0x42, 0xfb, 0x13, 0xf1, 0x22, 0xfb, 0x03, 0x41,
    0x22, 0xfb, 0x13, 0x41, 0x42, 0xfb, 0x03, 0x41, 0x42, 0xfb, 0x13, 0x41, 0xc3, 0xfb, 0xc4, 0x12,
    0xc3, 0xfb, 0xd4, 0x12, 0xd3, 0xfb, 0xc4, 0x12, 0xd3, 0xfb, 0xd4, 0x12, 0x52, 0xfb, 0x03, 0xf1,
    0x52, 0xfb, 0x13, 0xf1, 0x52, 0xfb, 0x03, 0x41, 0x52, 0xfb, 0x13, 0x41, 0x62, 0xfb, 0x03, 0x41,
    0x62, 0xfb, 0x13, 0x41, 0x72, 0xfb, 0x03, 0xf1, 0x72, 0xfb, 0x03, 0x41, 0x83, 0xfb, 0x04, 0x12,
    0xa3, 0xfb, 0x04, 0x12, 0xc3, 0xfb, 0x04, 0x12, 0xe3, 0xfb, 0x04, 0x12, 0xe3, 0xfb, 0x64, 0x12,
    0x92, 0xfb, 0xf3, 0xf1, 0xba, 0xfb, 0xfb, 0xf9, 0x00, 0xf0, 0xe8, 0xb8, 0x00, 0xf0, 0xe6, 0xf8,
    0x00, 0xf0, 0xe6, 0xe8, 0x00, 0xf0, 0xe2, 0x80, 0x40, 0xf0, 0xe0, 0x80, 0x80, 0xf0, 0xde, 0x80,
    0xc0, 0xf0, 0xdc, 0x80, 0x00, 0xf1, 0xda, 0x80, 0x40, 0xf1, 0xd8, 0x80, 0x80, 0xf1, 0xd6, 0x80,
    0xc0, 0xf1, 0xd4, 0x80, 0x00, 0xf2, 0xd2, 0x80, 0x40, 0xf2, 0xd0, 0x80, 0x80, 0xf2, 0xce, 0x80,
    0xc0, 0xf2, 0xcc, 0x80, 0x00, 0xf3, 0xca, 0x80, 0x40, 0xf3, 0xc8, 0x80, 0xef, 0xf3, 0x00, 0x80,
    0xff, 0xf3, 0x00, 0x89, 0x80, 0xf3, 0x00, 0x88, 0x81, 0xf3, 0x00, 0x8f, 0x92, 0xf3, 0x00, 0x81,
    0x92, 0xf3, 0x00, 0x8c, 0xaf, 0xf3, 0x40, 0x86, 0xaf, 0xf3, 0x60, 0x84, 0xaf, 0xf3, 0xf3, 0x87,
    0xaf, 0xf3, 0x3f, 0x85, 0xaf, 0xf3, 0x10, 0x81, 0xaf, 0xf3, 0x00, 0x80, 0xaf, 0xf3, 0x01, 0x80,
    0xaf, 0xf3, 0x02, 0x80, 0xaf, 0xf3, 0x03, 0x80, 0xaf, 0xf3, 0x04, 0x80, 0xaf, 0xf3, 0xf5, 0x80,
    0xbf, 0xf3, 0x5f, 0x8f, 0xbf, 0xf3, 0x5b, 0x8f, 0xbf, 0xf3, 0x5a, 0x8f, 0xbf, 0xf3, 0x43, 0x8f,
    0xbf, 0xf3, 0x47, 0x8f, 0xbf, 0xf3, 0x6f, 0x8f, 0xbf, 0xf3, 0x2f, 0x8f, 0xc5, 0xf3, 0x00, 0x8f,
    0xde, 0xf3, 0x00, 0x8f, 0xde, 0xf3, 0xff, 0x8f, 0xf3, 0xf7, 0x00, 0x80, 0xf0, 0xf7, 0x00, 0xa0,
    0xff, 0xf7, 0xff, 0xaf, 0x30, 0xee, 0x81, 0x0a, 0x31, 0xee, 0x42, 0x0b, 0x62, 0xee, 0x22, 0x1a,
    0x89, 0xee, 0x0a, 0x8b, 0xb7, 0xee, 0x00, 0x0a, 0x10, 0xee, 0x90, 0x0a, 0x01, 0xee, 0x10, 0x3a,
    0x51, 0xec, 0x12, 0x0b, 0xb4, 0xee, 0x60, 0x0a, 0xb5, 0xee, 0xc0, 0x0b, 0xbd, 0xee, 0xc1, 0x0b,
    0xb7, 0xee, 0xe0, 0x0a, 0x90, 0xed, 0x02, 0x0b, 0x42, 0xed, 0x01, 0x0a, 0xb0, 0xec, 0x08, 0x0b,
    0x2d, 0xed, 0x10, 0x8b, 0xbd, 0xec, 0x10, 0x8a, 0xf1, 0xee, 0x10, 0x0a, 0xf1, 0xee, 0x10, 0xfa,
    0xe1, 0xee, 0x10, 0x1a, 0xb1, 0xee, 0xe0, 0x0a, 0xb0, 0xee, 0xc1, 0x0b, 0xb1, 0xee, 0x60, 0x0a,
    0x08, 0x00, 0xc8, 0x00, 0x08, 0x08, 0x5a, 0x10, 0x88, 0x18, 0xc8, 0x1f, 0xff, 0x20, 0x0a, 0x28,
    0x01, 0x33, 0xc8, 0x3b, 0x11, 0x40, 0x51, 0x40, 0x91, 0x40, 0xd1, 0x40, 0x11, 0x41, 0x51, 0x41,
    0x91, 0x41, 0xd1, 0x41, 0x11, 0x42, 0x91, 0x42, 0xd1, 0x42, 0x11, 0x43, 0x91, 0x43, 0xd1, 0x43,
    0x51, 0x42, 0x51, 0x43, 0x88, 0x44, 0x41, 0x44, 0xc8, 0x46, 0x08, 0x46, 0xc8, 0x45, 0x70, 0x47,
    0x98, 0x47, 0x04, 0x48, 0x88, 0x58, 0x88, 0x54, 0x88, 0x5e, 0x48, 0x68, 0x08, 0x68, 0xc8, 0x7f,
    0xc8, 0x87, 0xff, 0x98, 0x01, 0x91, 0x01, 0xa8, 0x02, 0xb0, 0xff, 0xb0, 0x30, 0xb5, 0x01, 0xbd,
    0x06, 0xc8, 0x03, 0xc8, 0x0c, 0xc1, 0x05, 0xdf, 0x08, 0xba, 0x5a, 0xba, 0xec, 0xba, 0x08, 0xb2,
    0x48, 0xb2, 0x88, 0xb2, 0xc8, 0xb2, 0x62, 0xb6, 0x73, 0xb6, 0x58, 0xb6, 0x50, 0xb6, 0x00, 0xbf,
    0x10, 0xbf, 0x20, 0xbf, 0x30, 0xbf, 0x40, 0xbf, 0x07, 0xde, 0xb0, 0xb1, 0xaf, 0xb9, 0x14, 0xe0,
    0x13, 0xd0, 0x08, 0xbf, 0x08, 0x46, 0x14, 0xbf, 0x01, 0x30, 0x02, 0x30, 0xc7, 0xbf, 0x01, 0x20,
    0xd1, 0x00, 0x01, 0xf5, 0x80, 0x70, 0xd2, 0xf8, 0x04, 0x10, 0x24, 0xbf, 0x51, 0x43, 0x70, 0x47,
    0x42, 0xbf, 0x40, 0x18, 0x08, 0x68, 0x00, 0xf0, 0x01, 0xb8, 0x00, 0xbf, 0xaf, 0xf3, 0x00, 0x80,
    0xaf, 0xf3, 0x00, 0x80,
];
