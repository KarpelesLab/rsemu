//! Tests for the ARMv5TE core.
//!
//! The corpus in [`super::conformance`] covers the ARMv4T subset and nothing
//! else, so everything ARMv5 added — `CLZ`, both `BLX` forms, `BKPT`, the DSP
//! extensions, interworking loads — is tested here or it is not tested at all.
//! So are the parts no single-step corpus can reach: the exception model, mode
//! banking, the coprocessor seam and the snapshot round trip.
//!
//! Encodings are written as raw words with the assembler syntax in a comment.
//! That is deliberate: an assembler in the test file would be a second
//! implementation of the encoding, and the whole point of [`super::isa`] is
//! that there is only one.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, ResetKind};

use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, UnassignedPolicy,
};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::sync::{self, LockRank};
use crate::core::value::{Endian, Width};

use super::cp::{AccessKind, Coprocessor, Cp15Stub, CpEffect, CpFault, CpOp, Fault, FlatMmu, Mmu};
use super::*;

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// One bus access as the core made it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Access {
    addr: u32,
    value: u32,
    width: Width,
    write: bool,
}

/// Byte-addressed RAM that records every access.
///
/// A plain `RamStore` would be less code, but then a timing or ordering
/// mistake would be invisible: the tests that say something about *when* an
/// access happens assert the log, not just the final memory image.
#[derive(Debug)]
struct LogRam {
    inner: sync::Mutex<(Vec<u8>, Vec<Access>)>,
    base: u32,
}

impl LogRam {
    fn new(base: u32, len: usize) -> Arc<LogRam> {
        Arc::new(LogRam {
            inner: sync::Mutex::with_rank(LockRank::DEVICE, (alloc::vec![0u8; len], Vec::new())),
            base,
        })
    }
}

impl MemOps for LogRam {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let mut m = self.inner.lock();
        let at = offset as usize;
        if at + dst.len() > m.0.len() {
            return Err(crate::core::error::BusError::BadAccess);
        }
        dst.copy_from_slice(&m.0[at..at + dst.len()]);
        if !attrs.debug {
            let mut value = 0u32;
            for (i, byte) in dst.iter().enumerate().take(4) {
                value |= u32::from(*byte) << (8 * i);
            }
            let width = Width::from_bytes(dst.len() as u64).unwrap_or(Width::U8);
            let addr = self.base.wrapping_add(offset as u32);
            m.1.push(Access {
                addr,
                value,
                width,
                write: false,
            });
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let mut m = self.inner.lock();
        let at = offset as usize;
        if at + src.len() > m.0.len() {
            return Err(crate::core::error::BusError::BadAccess);
        }
        m.0[at..at + src.len()].copy_from_slice(src);
        if !attrs.debug {
            let mut value = 0u32;
            for (i, byte) in src.iter().enumerate().take(4) {
                value |= u32::from(*byte) << (8 * i);
            }
            let width = Width::from_bytes(src.len() as u64).unwrap_or(Width::U8);
            let addr = self.base.wrapping_add(offset as u32);
            m.1.push(Access {
                addr,
                value,
                width,
                write: true,
            });
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

/// The size of the low RAM every test gets.
const RAM_SIZE: usize = 0x2_0000;
/// Where the high vector page lives, when a test asks for one.
const HIGH_VECTORS: u32 = 0xffff_0000;

struct Harness {
    cpu: Arc<Arm>,
    ram: Arc<LogRam>,
    high: Arc<LogRam>,
}

impl Harness {
    fn with_config(cfg: Config) -> Harness {
        let ram = LogRam::new(0, RAM_SIZE);
        let high = LogRam::new(HIGH_VECTORS, 0x1000);
        let space = AddressSpace::new("cpu", 32).with_unassigned(UnassignedPolicy::FAULT);
        {
            let mut topo = space.topology();
            topo.map(Region::io("ram", RAM_SIZE as u64, ram.clone()), 0)
                .expect("low ram maps");
            topo.map(
                Region::io("high", 0x1000, high.clone()),
                u64::from(HIGH_VECTORS),
            )
            .expect("vector page maps");
        }
        let cpu = Arc::new(Arm::new(cfg));
        cpu.attach_space(Arc::new(space));
        Harness { cpu, ram, high }
    }

    fn new() -> Harness {
        Harness::with_config(Config::ARM926EJS)
    }

    fn poke(&self, addr: u32, word: u32) {
        let mut m = self.ram.inner.lock();
        let at = addr as usize;
        m.0[at..at + 4].copy_from_slice(&word.to_le_bytes());
    }

    fn poke_half(&self, addr: u32, half: u16) {
        let mut m = self.ram.inner.lock();
        let at = addr as usize;
        m.0[at..at + 2].copy_from_slice(&half.to_le_bytes());
    }

    fn peek(&self, addr: u32) -> u32 {
        let m = self.ram.inner.lock();
        let at = addr as usize;
        u32::from_le_bytes([m.0[at], m.0[at + 1], m.0[at + 2], m.0[at + 3]])
    }

    fn peek_byte(&self, addr: u32) -> u8 {
        self.ram.inner.lock().0[addr as usize]
    }

    /// Load ARM words at `addr`.
    fn program(&self, addr: u32, words: &[u32]) {
        for (i, word) in words.iter().enumerate() {
            self.poke(addr + 4 * i as u32, *word);
        }
    }

    /// Load Thumb halfwords at `addr`.
    fn program_thumb(&self, addr: u32, halves: &[u16]) {
        for (i, half) in halves.iter().enumerate() {
            self.poke_half(addr + 2 * i as u32, *half);
        }
    }

    /// Consume the reset sequence and start executing at `pc` in System mode.
    ///
    /// System mode rather than Supervisor so that the tests which do not care
    /// about banking see one register file, and privileged so that `MSR` and
    /// the `S` bit behave.
    fn boot(&self, pc: u32) {
        self.cpu.step();
        self.cpu.set_cpsr(u32::from(Mode::SYSTEM.0));
        self.cpu.set_pc(pc);
        self.clear_log();
    }

    fn clear_log(&self) {
        self.ram.inner.lock().1.clear();
        self.high.inner.lock().1.clear();
    }

    fn log(&self) -> Vec<Access> {
        self.ram.inner.lock().1.clone()
    }

    fn step(&self) -> u64 {
        self.cpu.step()
    }

    fn regs(&self) -> Regs {
        self.cpu.regs()
    }
}

/// A machine with a program at `0x1000`, booted and ready to step.
fn running(words: &[u32]) -> Harness {
    let h = Harness::new();
    h.program(0x1000, words);
    h.boot(0x1000);
    h
}

/// The same, in Thumb state.
fn running_thumb(halves: &[u16]) -> Harness {
    let h = Harness::new();
    h.program_thumb(0x1000, halves);
    h.boot(0x1000);
    h.cpu.set_cpsr(u32::from(Mode::SYSTEM.0) | psr::T);
    h
}

// ---------------------------------------------------------------------------
// Reset and the register file
// ---------------------------------------------------------------------------

#[test]
fn reset_enters_supervisor_with_both_interrupts_masked() {
    let h = Harness::new();
    assert!(h.cpu.reset_pending());
    h.step();
    let r = h.regs();
    assert_eq!(r.pc(), 0);
    assert_eq!(r.mode(), Mode::SUPERVISOR);
    assert!(r.cpsr & psr::I != 0);
    assert!(r.cpsr & psr::F != 0);
    assert!(r.cpsr & psr::T == 0);
    assert!(!h.cpu.reset_pending());
}

#[test]
fn high_vectors_move_the_whole_table() {
    let h = Harness::with_config(Config::ARM926EJS.with_high_vectors(true));
    h.step();
    assert_eq!(h.regs().pc(), HIGH_VECTORS);
}

#[test]
fn banking_moves_sp_and_lr_but_shares_the_rest() {
    let mut r = Regs::new();
    r.write_cpsr(u32::from(Mode::SYSTEM.0));
    r.r[13] = 0x1000;
    r.r[14] = 0x2000;
    r.r[5] = 0x5555;
    r.write_cpsr(u32::from(Mode::IRQ.0));
    // A fresh bank, and the shared registers untouched.
    assert_eq!(r.r[13], 0);
    assert_eq!(r.r[5], 0x5555);
    r.r[13] = 0x9000;
    r.write_cpsr(u32::from(Mode::SYSTEM.0));
    assert_eq!(r.r[13], 0x1000);
    assert_eq!(r.r[14], 0x2000);
    assert_eq!(r.reg_in_mode(Mode::IRQ, 13), 0x9000);
}

#[test]
fn fiq_banks_five_more_registers_than_anyone_else() {
    let mut r = Regs::new();
    r.write_cpsr(u32::from(Mode::SYSTEM.0));
    for i in 8..=12 {
        r.r[i] = 0x1000 + i as u32;
    }
    r.write_cpsr(u32::from(Mode::FIQ.0));
    for i in 8..=12 {
        assert_eq!(r.r[i], 0, "r{i} should be the FIQ bank");
        r.r[i] = 0xf000 + i as u32;
    }
    // IRQ shares the user bank for r8-r12, which is why only FIQ is "fast".
    r.write_cpsr(u32::from(Mode::IRQ.0));
    for i in 8..=12 {
        assert_eq!(r.r[i], 0x1000 + i as u32);
    }
    assert_eq!(r.reg_in_mode(Mode::FIQ, 10), 0xf00a);
}

#[test]
fn user_and_system_share_one_bank() {
    let mut r = Regs::new();
    r.write_cpsr(u32::from(Mode::USER.0));
    r.r[13] = 0xdead;
    r.write_cpsr(u32::from(Mode::SYSTEM.0));
    assert_eq!(r.r[13], 0xdead);
    assert!(Mode::SYSTEM.is_privileged());
    assert!(!Mode::USER.is_privileged());
    assert_eq!(Mode::USER.spsr_index(), None);
    assert_eq!(Mode::SYSTEM.spsr_index(), None);
}

// ---------------------------------------------------------------------------
// Data processing and the barrel shifter
// ---------------------------------------------------------------------------

#[test]
fn mov_immediate_and_the_pc_advance() {
    let h = running(&[0xe3a0_0042]); // MOV r0, #0x42
    let cycles = h.step();
    assert_eq!(h.cpu.reg(0), 0x42);
    assert_eq!(h.cpu.pc(), 0x1004);
    // One fetch, nothing else.
    assert_eq!(cycles, 1);
}

#[test]
fn add_sets_carry_and_overflow_from_the_adder() {
    // ADDS r0, r1, r2
    let h = running(&[0xe091_0002]);
    h.cpu.set_reg(1, 0x8000_0000);
    h.cpu.set_reg(2, 0x8000_0000);
    h.step();
    assert_eq!(h.cpu.reg(0), 0);
    let cpsr = h.cpu.cpsr();
    assert!(cpsr & psr::Z != 0, "zero");
    assert!(cpsr & psr::C != 0, "carry out");
    assert!(cpsr & psr::V != 0, "signed overflow");
    assert!(cpsr & psr::N == 0);
}

#[test]
fn subtract_leaves_carry_set_when_there_was_no_borrow() {
    // SUBS r0, r1, r2
    let h = running(&[0xe051_0002, 0xe051_0002]);
    h.cpu.set_reg(1, 5);
    h.cpu.set_reg(2, 3);
    h.step();
    assert_eq!(h.cpu.reg(0), 2);
    assert!(h.cpu.cpsr() & psr::C != 0, "no borrow means C set");

    h.cpu.set_reg(1, 3);
    h.cpu.set_reg(2, 5);
    h.step();
    assert_eq!(h.cpu.reg(0), 0xffff_fffe);
    assert!(h.cpu.cpsr() & psr::C == 0, "borrow means C clear");
    assert!(h.cpu.cpsr() & psr::N != 0);
}

#[test]
fn adc_and_sbc_read_the_carry_in() {
    // ADCS r0, r1, r2 ; SBCS r3, r1, r2
    let h = running(&[0xe0b1_0002, 0xe0d1_3002]);
    h.cpu.set_reg(1, 10);
    h.cpu.set_reg(2, 20);
    h.cpu.set_cpsr(h.cpu.cpsr() | psr::C);
    h.step();
    assert_eq!(h.cpu.reg(0), 31);

    h.cpu.set_cpsr(h.cpu.cpsr() & !psr::C);
    h.cpu.set_reg(1, 20);
    h.cpu.set_reg(2, 10);
    h.step();
    // SBC with C clear subtracts one more.
    assert_eq!(h.cpu.reg(3), 9);
}

#[test]
fn a_logical_operation_takes_its_carry_from_the_shifter() {
    // MOVS r0, r1, LSL #1 — bit 31 of r1 falls out into C.
    let h = running(&[0xe1b0_0081]);
    h.cpu.set_reg(1, 0x8000_0001);
    h.cpu.set_cpsr(h.cpu.cpsr() & !psr::C);
    h.step();
    assert_eq!(h.cpu.reg(0), 2);
    assert!(h.cpu.cpsr() & psr::C != 0);
    // V is untouched by a logical operation.
    assert!(h.cpu.cpsr() & psr::V == 0);
}

#[test]
fn lsr_zero_means_thirty_two_and_asr_zero_means_the_sign() {
    // MOVS r0, r1, LSR #32 ; MOVS r2, r1, ASR #32
    let h = running(&[0xe1b0_0021, 0xe1b0_2041]);
    h.cpu.set_reg(1, 0x8000_0000);
    h.step();
    assert_eq!(h.cpu.reg(0), 0);
    assert!(h.cpu.cpsr() & psr::C != 0, "bit 31 went to the carry");
    h.step();
    assert_eq!(h.cpu.reg(2), 0xffff_ffff);
}

#[test]
fn ror_zero_is_rrx_and_rotates_through_the_carry() {
    // MOVS r0, r1, RRX
    let h = running(&[0xe1b0_0061]);
    h.cpu.set_reg(1, 0x0000_0003);
    h.cpu.set_cpsr(h.cpu.cpsr() | psr::C);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x8000_0001);
    assert!(h.cpu.cpsr() & psr::C != 0, "the old bit 0");
}

#[test]
fn a_register_controlled_shift_costs_a_cycle_and_saturates_at_thirty_two() {
    // MOVS r0, r1, LSL r2
    let h = running(&[0xe1b0_0211, 0xe1b0_0211, 0xe1b0_0211]);
    h.cpu.set_reg(1, 0xffff_ffff);

    h.cpu.set_reg(2, 0);
    h.cpu.set_cpsr(h.cpu.cpsr() | psr::C);
    let cycles = h.step();
    assert_eq!(h.cpu.reg(0), 0xffff_ffff);
    assert!(h.cpu.cpsr() & psr::C != 0, "a zero shift leaves C alone");
    assert_eq!(cycles, 2, "one fetch plus one internal cycle");

    h.cpu.set_reg(2, 32);
    h.step();
    assert_eq!(h.cpu.reg(0), 0);
    assert!(h.cpu.cpsr() & psr::C != 0, "LSL #32 leaves bit 0 in C");

    h.cpu.set_reg(2, 33);
    h.step();
    assert_eq!(h.cpu.reg(0), 0);
    assert!(h.cpu.cpsr() & psr::C == 0, "past 32 everything is gone");
}

#[test]
fn a_register_shift_makes_the_pc_read_twelve_ahead() {
    // MOV r0, pc, LSL r2 with r2 = 0 — the ARM7TDMI pipeline showing through.
    let h = running(&[0xe1a0_021f]);
    h.cpu.set_reg(2, 0);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x1000 + 12);
}

#[test]
fn an_immediate_rotate_sets_the_carry_and_a_zero_rotate_does_not() {
    // MOVS r0, #0x0200_0000  (imm8 = 0x02, rotate = 4 -> ror 8)
    // MOVS r1, #0x8000_0000  (imm8 = 0x02, rotate = 1 -> ror 2)
    let h = running(&[0xe3b0_0402, 0xe3b0_1102, 0xe3b0_2001]);
    h.cpu.set_cpsr(h.cpu.cpsr() & !psr::C);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x0200_0000);
    assert!(h.cpu.cpsr() & psr::C == 0, "bit 31 of the result is clear");
    h.step();
    assert_eq!(h.cpu.reg(1), 0x8000_0000);
    assert!(h.cpu.cpsr() & psr::C != 0, "bit 31 of the result is set");
    h.step();
    assert!(
        h.cpu.cpsr() & psr::C != 0,
        "a zero rotate must leave C exactly as it was"
    );
}

#[test]
fn conditions_gate_execution() {
    // MOVEQ r0, #1 ; MOVNE r1, #1
    let h = running(&[0x03a0_0001, 0x13a0_1001]);
    h.cpu.set_cpsr(h.cpu.cpsr() & !psr::Z);
    h.step();
    h.step();
    assert_eq!(h.cpu.reg(0), 0);
    assert_eq!(h.cpu.reg(1), 1);
}

#[test]
fn every_condition_agrees_with_its_definition() {
    // Exhaustive over all sixteen flag combinations, which is cheap and is the
    // only way to be sure about HI/LS and GT/LE.
    for flags in 0u32..16 {
        let psr = flags << 28;
        let (n, z, c, v) = (
            flags & 0b1000 != 0,
            flags & 0b0100 != 0,
            flags & 0b0010 != 0,
            flags & 0b0001 != 0,
        );
        let expect = [
            z,
            !z,
            c,
            !c,
            n,
            !n,
            v,
            !v,
            c && !z,
            !c || z,
            n == v,
            n != v,
            !z && n == v,
            z || n != v,
            true,
            false,
        ];
        for (i, want) in expect.iter().enumerate() {
            assert_eq!(
                isa::Cond(i as u8).passes(psr),
                *want,
                "cond {i} with flags {flags:04b}"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Multiply
// ---------------------------------------------------------------------------

#[test]
fn mul_and_mla() {
    // MULS r0, r1, r2 ; MLA r3, r1, r2, r4
    let h = running(&[0xe010_0291, 0xe023_4291]);
    h.cpu.set_reg(1, 7);
    h.cpu.set_reg(2, 6);
    h.step();
    assert_eq!(h.cpu.reg(0), 42);
    assert!(h.cpu.cpsr() & psr::Z == 0);

    h.cpu.set_reg(4, 8);
    h.step();
    assert_eq!(h.cpu.reg(3), 50);
}

#[test]
fn mul_leaves_the_carry_flag_alone_in_armv5() {
    // MULS r0, r1, r2 — ARMv4 destroyed C here; ARMv5 does not.
    let h = running(&[0xe010_0291]);
    h.cpu.set_reg(1, 3);
    h.cpu.set_reg(2, 3);
    h.cpu.set_cpsr(h.cpu.cpsr() | psr::C | psr::V);
    h.step();
    assert!(h.cpu.cpsr() & psr::C != 0);
    assert!(h.cpu.cpsr() & psr::V != 0);
}

#[test]
fn long_multiplies_split_across_the_register_pair() {
    // UMULL r0, r1, r2, r3 ; SMULL r4, r5, r2, r3
    let h = running(&[0xe081_0392, 0xe0c5_4392]);
    h.cpu.set_reg(2, 0xffff_ffff);
    h.cpu.set_reg(3, 2);
    h.step();
    assert_eq!(h.cpu.reg(0), 0xffff_fffe);
    assert_eq!(h.cpu.reg(1), 1);
    h.step();
    // Signed: -1 * 2 = -2, sign-extended across both halves.
    assert_eq!(h.cpu.reg(4), 0xffff_fffe);
    assert_eq!(h.cpu.reg(5), 0xffff_ffff);
}

#[test]
fn umlal_accumulates_into_the_pair_and_sets_n_from_bit_sixty_three() {
    // UMLALS r0, r1, r2, r3
    let h = running(&[0xe0b1_0392]);
    h.cpu.set_reg(0, 1);
    h.cpu.set_reg(1, 0x8000_0000);
    h.cpu.set_reg(2, 0);
    h.cpu.set_reg(3, 0);
    h.step();
    assert_eq!(h.cpu.reg(0), 1);
    assert_eq!(h.cpu.reg(1), 0x8000_0000);
    assert!(h.cpu.cpsr() & psr::N != 0);
    assert!(h.cpu.cpsr() & psr::Z == 0);
}

#[test]
fn the_multiplier_terminates_early_on_a_small_operand() {
    // MUL r0, r1, r2 with a one-byte and a four-byte multiplier.
    let h = running(&[0xe000_0291, 0xe000_0291]);
    h.cpu.set_reg(1, 1);
    h.cpu.set_reg(2, 0xff);
    let small = h.step();
    h.cpu.set_reg(2, 0x0100_0000);
    let large = h.step();
    assert_eq!(small, 2, "one fetch plus one multiplier cycle");
    assert_eq!(large, 5, "one fetch plus four");
}

// ---------------------------------------------------------------------------
// Loads and stores
// ---------------------------------------------------------------------------

#[test]
fn ldr_and_str_with_every_indexing_mode() {
    // LDR r0, [r1, #4] ; LDR r2, [r1, #4]! ; LDR r3, [r1], #4
    let h = running(&[0xe591_0004, 0xe5b1_2004, 0xe491_3004]);
    h.poke(0x2004, 0xaabb_ccdd);
    h.poke(0x2008, 0x1122_3344);
    h.cpu.set_reg(1, 0x2000);

    h.step();
    assert_eq!(h.cpu.reg(0), 0xaabb_ccdd);
    assert_eq!(h.cpu.reg(1), 0x2000, "no writeback");

    h.step();
    assert_eq!(h.cpu.reg(2), 0xaabb_ccdd);
    assert_eq!(h.cpu.reg(1), 0x2004, "pre-indexed writeback");

    h.step();
    assert_eq!(h.cpu.reg(3), 0xaabb_ccdd, "post-index uses the old base");
    assert_eq!(h.cpu.reg(1), 0x2008);
}

#[test]
fn an_unaligned_word_load_rotates_rather_than_faulting() {
    // LDR r0, [r1]
    let h = running(&[0xe591_0000]);
    h.poke(0x2000, 0x1122_3344);
    h.cpu.set_reg(1, 0x2001);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x4411_2233, "rotated right by eight");
    // The bus saw one word access at the aligned address.
    let log = h.log();
    assert_eq!(log[1].addr, 0x2000);
    assert_eq!(log[1].width, Width::U32);
}

#[test]
fn alignment_faults_can_be_turned_on() {
    let h = Harness::with_config(Config::ARM926EJS.with_alignment_faults(true));
    h.program(0x1000, &[0xe591_0000]); // LDR r0, [r1]
    h.boot(0x1000);
    h.cpu.set_reg(1, 0x2001);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::ABORT);
    assert_eq!(h.cpu.pc(), Exception::DataAbort.vector());
}

#[test]
fn an_unaligned_word_store_drops_the_low_bits() {
    // STR r0, [r1]
    let h = running(&[0xe581_0000]);
    h.cpu.set_reg(0, 0xdead_beef);
    h.cpu.set_reg(1, 0x2003);
    h.step();
    assert_eq!(h.peek(0x2000), 0xdead_beef);
}

#[test]
fn byte_accesses_touch_one_lane() {
    // LDRB r0, [r1, #1] ; STRB r2, [r1, #2]
    let h = running(&[0xe5d1_0001, 0xe5c1_2002]);
    h.poke(0x2000, 0x1122_3344);
    h.cpu.set_reg(1, 0x2000);
    h.cpu.set_reg(2, 0xffff_ff99);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x33);
    h.step();
    assert_eq!(h.peek_byte(0x2002), 0x99);
    assert_eq!(h.peek(0x2000), 0x1199_3344, "only lane 2 changed");
}

#[test]
fn halfword_and_signed_loads() {
    // LDRH r0,[r1] ; LDRSH r2,[r1] ; LDRSB r3,[r1] ; STRH r4,[r1,#4]
    let h = running(&[0xe1d1_00b0, 0xe1d1_20f0, 0xe1d1_30d0, 0xe1c1_40b4]);
    h.poke(0x2000, 0x0000_ff80);
    h.cpu.set_reg(1, 0x2000);
    h.step();
    assert_eq!(h.cpu.reg(0), 0xff80);
    h.step();
    assert_eq!(h.cpu.reg(2), 0xffff_ff80, "sign-extended halfword");
    h.step();
    assert_eq!(h.cpu.reg(3), 0xffff_ff80, "sign-extended byte");
    h.cpu.set_reg(4, 0x1234_5678);
    h.step();
    assert_eq!(h.peek(0x2004) & 0xffff, 0x5678);
}

#[test]
fn ldrd_and_strd_move_a_register_pair() {
    // LDRD r0, [r1] ; STRD r2, [r1, #8]
    let h = running(&[0xe1c1_00d0, 0xe1c1_20f8]);
    h.poke(0x2000, 0x1111_1111);
    h.poke(0x2004, 0x2222_2222);
    h.cpu.set_reg(1, 0x2000);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x1111_1111);
    assert_eq!(h.cpu.reg(1), 0x2222_2222);

    h.cpu.set_reg(1, 0x2000);
    h.cpu.set_reg(2, 0xaaaa_aaaa);
    h.cpu.set_reg(3, 0xbbbb_bbbb);
    h.step();
    assert_eq!(h.peek(0x2008), 0xaaaa_aaaa);
    assert_eq!(h.peek(0x200c), 0xbbbb_bbbb);
}

#[test]
fn an_odd_register_makes_ldrd_undefined() {
    // LDRD r1, [r2] — Rd must be even.
    let h = running(&[0xe1c2_10d0]);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::UNDEFINED);
}

#[test]
fn storing_the_pc_uses_the_configured_offset() {
    // STR pc, [r1]
    let h = running(&[0xe581_f000]);
    h.cpu.set_reg(1, 0x2000);
    h.step();
    assert_eq!(h.peek(0x2000), 0x1000 + 8, "ARM926EJ-S stores pc + 8");

    let h = Harness::with_config(Config::ARM7TDMI);
    h.program(0x1000, &[0xe581_f000]);
    h.boot(0x1000);
    h.cpu.set_reg(1, 0x2000);
    h.step();
    assert_eq!(h.peek(0x2000), 0x1000 + 12, "ARM7TDMI stores pc + 12");
}

#[test]
fn loading_the_pc_interworks_in_armv5() {
    // LDR pc, [r1]
    let h = running(&[0xe591_f000]);
    h.poke(0x2000, 0x0000_3001); // odd: a Thumb target
    h.cpu.set_reg(1, 0x2000);
    h.step();
    assert_eq!(h.cpu.pc(), 0x3000);
    assert!(h.cpu.is_thumb());
}

#[test]
fn ldr_timing_is_fetch_plus_access_plus_one() {
    // LDR r0, [r1] ; STR r0, [r1]
    let h = running(&[0xe591_0000, 0xe581_0000]);
    h.cpu.set_reg(1, 0x2000);
    assert_eq!(h.step(), 3, "1S + 1N + 1I");
    assert_eq!(h.step(), 2, "2N");
}

#[test]
fn swp_reads_then_writes_the_same_address() {
    // SWP r0, r1, [r2]
    let h = running(&[0xe102_0091]);
    h.poke(0x2000, 0x1234_5678);
    h.cpu.set_reg(1, 0xabcd_ef01);
    h.cpu.set_reg(2, 0x2000);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x1234_5678);
    assert_eq!(h.peek(0x2000), 0xabcd_ef01);
    let log = h.log();
    assert_eq!(log.len(), 3, "fetch, read, write");
    assert!(!log[1].write && log[1].addr == 0x2000);
    assert!(log[2].write && log[2].addr == 0x2000);
}

#[test]
fn swpb_moves_one_byte() {
    // SWPB r0, r1, [r2]
    let h = running(&[0xe142_0091]);
    h.poke(0x2000, 0x1234_5678);
    h.cpu.set_reg(1, 0xaa);
    h.cpu.set_reg(2, 0x2001);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x56);
    assert_eq!(h.peek(0x2000), 0x1234_aa78);
}

// ---------------------------------------------------------------------------
// Block transfers
// ---------------------------------------------------------------------------

#[test]
fn ldm_and_stm_transfer_lowest_register_to_lowest_address() {
    // STMIA r0, {r1, r2, r3}
    let h = running(&[0xe880_000e]);
    h.cpu.set_reg(0, 0x2000);
    h.cpu.set_reg(1, 0x1111);
    h.cpu.set_reg(2, 0x2222);
    h.cpu.set_reg(3, 0x3333);
    h.step();
    assert_eq!(h.peek(0x2000), 0x1111);
    assert_eq!(h.peek(0x2004), 0x2222);
    assert_eq!(h.peek(0x2008), 0x3333);
}

#[test]
fn all_four_stack_modes_land_where_the_manual_says() {
    // STMIA/STMIB/STMDA/STMDB r0!, {r1, r2}
    let h = running(&[0xe8a0_0006, 0xe9a0_0006, 0xe820_0006, 0xe920_0006]);
    h.cpu.set_reg(1, 0xaa);
    h.cpu.set_reg(2, 0xbb);

    for (base, first, writeback) in [
        (0x2000u32, 0x2000u32, 0x2008u32), // IA: base, then up
        (0x3000, 0x3004, 0x3008),          // IB: up, then store
        (0x4000, 0x3ffc, 0x3ff8),          // DA: the last word lands on base
        (0x5000, 0x4ff8, 0x4ff8),          // DB: down, then store
    ] {
        h.cpu.set_reg(0, base);
        h.step();
        assert_eq!(h.peek(first), 0xaa, "base {base:#x}");
        assert_eq!(h.peek(first + 4), 0xbb, "base {base:#x}");
        assert_eq!(h.cpu.reg(0), writeback, "base {base:#x}");
    }
}

#[test]
fn push_and_pop_round_trip_through_the_stack() {
    // STMDB sp!, {r0-r3} ; LDMIA sp!, {r4-r7}
    let h = running(&[0xe92d_000f, 0xe8bd_00f0]);
    h.cpu.set_reg(13, 0x3000);
    for i in 0..4 {
        h.cpu.set_reg(i, 0x100 + u32::from(i));
    }
    h.step();
    assert_eq!(h.cpu.reg(13), 0x2ff0);
    h.step();
    assert_eq!(h.cpu.reg(13), 0x3000);
    for i in 0..4 {
        assert_eq!(h.cpu.reg(4 + i), 0x100 + u32::from(i));
    }
}

#[test]
fn ldm_timing_is_one_cycle_per_register_plus_two() {
    // LDMIA r0, {r1-r4} ; STMIA r0, {r1-r4}
    let h = running(&[0xe890_001e, 0xe880_001e]);
    h.cpu.set_reg(0, 0x2000);
    assert_eq!(h.step(), 6, "4S + 1N + 1I");
    assert_eq!(h.step(), 5, "3S + 2N");
}

#[test]
fn stm_stores_the_original_base_when_it_is_lowest_in_the_list() {
    // STMIA r0!, {r0, r1}
    let h = running(&[0xe8a0_0003]);
    h.cpu.set_reg(0, 0x2000);
    h.cpu.set_reg(1, 0xbb);
    h.step();
    assert_eq!(h.peek(0x2000), 0x2000, "the unmodified base");
    assert_eq!(h.cpu.reg(0), 0x2008);

    // ...and the written-back value when it is not.
    // STMIA r1!, {r0, r1}
    let h = running(&[0xe8a1_0003]);
    h.cpu.set_reg(0, 0xaa);
    h.cpu.set_reg(1, 0x2000);
    h.step();
    assert_eq!(h.peek(0x2004), 0x2008, "the written-back base");
}

#[test]
fn ldm_with_the_base_in_the_list_keeps_the_loaded_value() {
    // LDMIA r0!, {r0, r1}
    let h = running(&[0xe8b0_0003]);
    h.poke(0x2000, 0xcafe);
    h.poke(0x2004, 0xbabe);
    h.cpu.set_reg(0, 0x2000);
    h.step();
    assert_eq!(h.cpu.reg(0), 0xcafe);
    assert_eq!(h.cpu.reg(1), 0xbabe);
}

#[test]
fn an_empty_register_list_moves_the_pc_and_the_base_by_sixty_four() {
    // LDMIA r0!, {} — UNPREDICTABLE architecturally; this is ARM7TDMI's answer.
    let h = running(&[0xe8b0_0000]);
    h.poke(0x2000, 0x4000);
    h.cpu.set_reg(0, 0x2000);
    h.step();
    assert_eq!(h.cpu.pc(), 0x4000);
    assert_eq!(h.cpu.reg(0), 0x2040);
}

#[test]
fn the_s_bit_reaches_the_user_bank() {
    // STMIA r0, {r13}^ — stores the *user* sp while running in IRQ mode.
    let h = running(&[0xe8c0_2000]);
    h.cpu.set_cpsr(u32::from(Mode::SYSTEM.0));
    h.cpu.set_reg(13, 0x1234_5678);
    h.cpu.set_cpsr(u32::from(Mode::IRQ.0));
    h.cpu.set_reg(13, 0x9999_9999);
    h.cpu.set_reg(0, 0x2000);
    h.step();
    assert_eq!(h.peek(0x2000), 0x1234_5678, "the user-mode sp, not IRQ's");
}

// ---------------------------------------------------------------------------
// Branches and interworking
// ---------------------------------------------------------------------------

#[test]
fn branch_and_branch_with_link() {
    // B +0 (to pc + 8) ; BL +0
    let h = running(&[0xea00_0000, 0x0, 0xeb00_0000]);
    let cycles = h.step();
    assert_eq!(h.cpu.pc(), 0x1008);
    assert_eq!(cycles, 3, "2S + 1N");
    h.cpu.set_pc(0x1008);
    h.step();
    assert_eq!(h.cpu.pc(), 0x1010);
    assert_eq!(h.cpu.reg(14), 0x100c, "lr is the instruction plus four");
}

#[test]
fn bx_and_blx_switch_instruction_set_on_bit_zero() {
    // BX r0 ; BLX r1
    let h = running(&[0xe12f_ff10, 0xe12f_ff31]);
    h.cpu.set_reg(0, 0x2001);
    h.step();
    assert!(h.cpu.is_thumb());
    assert_eq!(h.cpu.pc(), 0x2000);

    let h = running(&[0xe12f_ff10, 0xe12f_ff31]);
    h.cpu.set_pc(0x1004);
    h.cpu.set_reg(1, 0x3000);
    h.step();
    assert!(!h.cpu.is_thumb());
    assert_eq!(h.cpu.pc(), 0x3000);
    assert_eq!(h.cpu.reg(14), 0x1008);
}

#[test]
fn blx_immediate_always_lands_in_thumb() {
    // BLX +0 with H set: pc + 8 + 2.
    let h = running(&[0xfb00_0000]);
    h.step();
    assert!(h.cpu.is_thumb());
    assert_eq!(h.cpu.pc(), 0x1008 + 2);
    assert_eq!(h.cpu.reg(14), 0x1004);
}

#[test]
fn a_data_processing_write_to_the_pc_is_a_plain_branch() {
    // MOV pc, r0 — ARMv5 ignores bit 0 here rather than interworking.
    let h = running(&[0xe1a0_f000]);
    h.cpu.set_reg(0, 0x2001);
    h.step();
    assert_eq!(h.cpu.pc(), 0x2000);
    assert!(!h.cpu.is_thumb());
}

// ---------------------------------------------------------------------------
// The v5 additions
// ---------------------------------------------------------------------------

#[test]
fn clz_counts_leading_zeros_including_the_all_zero_case() {
    // CLZ r0, r1
    let h = running(&[0xe16f_0f11, 0xe16f_0f11, 0xe16f_0f11]);
    h.cpu.set_reg(1, 0x8000_0000);
    h.step();
    assert_eq!(h.cpu.reg(0), 0);
    h.cpu.set_reg(1, 1);
    h.step();
    assert_eq!(h.cpu.reg(0), 31);
    h.cpu.set_reg(1, 0);
    h.step();
    assert_eq!(h.cpu.reg(0), 32);
}

#[test]
fn bkpt_takes_a_prefetch_abort_and_records_its_comment() {
    // BKPT #0x1234
    let h = running(&[0xe121_2374]);
    h.step();
    assert_eq!(h.cpu.last_bkpt(), 0x1234);
    assert_eq!(h.cpu.mode(), Mode::ABORT);
    assert_eq!(h.cpu.pc(), Exception::PrefetchAbort.vector());
    assert_eq!(h.cpu.reg(14), 0x1004);
}

#[test]
fn pld_does_nothing_observable() {
    // PLD [r1]
    let h = running(&[0xf5d1_f000]);
    let before = h.regs();
    let cycles = h.step();
    let after = h.regs();
    assert_eq!(after.r[15], 0x1004);
    assert_eq!(cycles, 1);
    for i in 0..15 {
        assert_eq!(before.r[i], after.r[i]);
    }
    assert_eq!(before.cpsr, after.cpsr);
}

#[test]
fn the_saturating_arithmetic_clamps_and_sets_q() {
    // QADD r0, r1, r2 ; QSUB r3, r1, r2 ; QDADD r4, r1, r2
    let h = running(&[0xe102_0051, 0xe122_3051, 0xe142_4051]);
    h.cpu.set_reg(1, 0x7fff_ffff);
    h.cpu.set_reg(2, 1);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x7fff_ffff);
    assert!(h.cpu.cpsr() & psr::Q != 0, "Q is sticky and was set");

    h.cpu.set_cpsr(h.cpu.cpsr() & !psr::Q);
    h.cpu.set_reg(1, 0x8000_0000);
    h.cpu.set_reg(2, 1);
    h.step();
    assert_eq!(h.cpu.reg(3), 0x8000_0000);
    assert!(h.cpu.cpsr() & psr::Q != 0);

    // QDADD doubles Rn before adding, and each half saturates on its own.
    h.cpu.set_cpsr(h.cpu.cpsr() & !psr::Q);
    h.cpu.set_reg(1, 1);
    h.cpu.set_reg(2, 0x2000_0000);
    h.step();
    assert_eq!(h.cpu.reg(4), 0x4000_0001);
    assert!(h.cpu.cpsr() & psr::Q == 0, "nothing saturated");
}

#[test]
fn qdadd_saturates_the_doubling_before_the_addition() {
    // QDADD r4, r1, r2 with r2 = 0x40000000: doubling alone overflows, so the
    // addend is already clamped when it reaches the adder (ARM ARM A4.1.28).
    let h = running(&[0xe142_4051]);
    h.cpu.set_reg(1, 0);
    h.cpu.set_reg(2, 0x4000_0000);
    h.step();
    assert_eq!(h.cpu.reg(4), 0x7fff_ffff);
    assert!(h.cpu.cpsr() & psr::Q != 0);
}

#[test]
fn q_is_sticky_until_msr_clears_it() {
    // QADD r0, r1, r2 ; MSR CPSR_f, #0
    let h = running(&[0xe102_0051, 0xe328_f000]);
    h.cpu.set_reg(1, 0x7fff_ffff);
    h.cpu.set_reg(2, 1);
    h.step();
    assert!(h.cpu.cpsr() & psr::Q != 0);
    h.step();
    assert!(h.cpu.cpsr() & psr::Q == 0);
}

#[test]
fn the_half_word_multiplies_pick_the_right_halves() {
    // SMULBB r0,r1,r2 ; SMULTB r3,r1,r2 ; SMULBT r6,r1,r2 ; SMLABB r4,r1,r2,r5
    let h = running(&[0xe160_0281, 0xe163_02a1, 0xe166_02c1, 0xe104_5281]);
    h.cpu.set_reg(1, 0x0002_0003);
    h.cpu.set_reg(2, 0x0004_0005);
    h.step();
    assert_eq!(h.cpu.reg(0), 3 * 5);
    h.step();
    assert_eq!(h.cpu.reg(3), 2 * 5, "<x> is bit 5 and selects a half of Rm");
    h.step();
    assert_eq!(h.cpu.reg(6), 3 * 4, "<y> is bit 6 and selects a half of Rs");

    h.cpu.set_reg(5, 100);
    h.step();
    assert_eq!(h.cpu.reg(4), 15 + 100);
}

#[test]
fn smlaw_and_smulw_shift_the_wide_product_down_by_sixteen() {
    // SMULWB r0, r1, r2
    let h = running(&[0xe120_02a1]);
    h.cpu.set_reg(1, 0x0001_0000);
    h.cpu.set_reg(2, 3);
    h.step();
    assert_eq!(h.cpu.reg(0), 3);
}

#[test]
fn smlal_half_accumulates_into_the_pair() {
    // SMLALBB r0, r1, r2, r3 — RdLo = r0, RdHi = r1.
    let h = running(&[0xe141_0382]);
    h.cpu.set_reg(0, 10);
    h.cpu.set_reg(1, 0);
    h.cpu.set_reg(2, 0xffff); // -1 in the bottom half
    h.cpu.set_reg(3, 4);
    h.step();
    assert_eq!(h.cpu.reg(0), 6);
    assert_eq!(h.cpu.reg(1), 0);
}

// ---------------------------------------------------------------------------
// Status registers
// ---------------------------------------------------------------------------

#[test]
fn mrs_and_msr_move_the_status_register() {
    // MRS r0, CPSR ; MSR CPSR_c, r1
    let h = running(&[0xe10f_0000, 0xe121_f001]);
    h.step();
    assert_eq!(h.cpu.reg(0), h.cpu.cpsr());

    h.cpu.set_reg(1, u32::from(Mode::IRQ.0) | psr::I);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::IRQ);
    assert!(h.cpu.cpsr() & psr::I != 0);
}

#[test]
fn msr_cannot_change_the_thumb_bit() {
    // MSR CPSR_c, r1 with T set in the source.
    let h = running(&[0xe121_f001]);
    h.cpu.set_reg(1, u32::from(Mode::SYSTEM.0) | psr::T);
    h.step();
    assert!(!h.cpu.is_thumb(), "the T bit is not MSR's to change");
}

#[test]
fn user_mode_msr_reaches_only_the_flags() {
    // MSR CPSR_cf, r1 in User mode.
    let h = running(&[0xe129_f001]);
    h.cpu.set_cpsr(u32::from(Mode::USER.0));
    h.cpu.set_reg(1, u32::from(Mode::SUPERVISOR.0) | psr::N);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::USER, "the control byte is protected");
    assert!(h.cpu.cpsr() & psr::N != 0, "the flags byte is not");
}

// ---------------------------------------------------------------------------
// Exceptions
// ---------------------------------------------------------------------------

#[test]
fn swi_enters_supervisor_and_saves_the_return_address() {
    // SWI #0x123456
    let h = running(&[0xef12_3456]);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::SUPERVISOR);
    assert_eq!(h.cpu.pc(), Exception::Swi.vector());
    assert_eq!(h.cpu.reg(14), 0x1004);
    assert!(h.cpu.cpsr() & psr::I != 0);
    assert!(h.cpu.cpsr() & psr::F == 0, "SWI does not mask FIQ");
    assert_eq!(h.cpu.last_swi(), 0x123456);
    assert_eq!(h.regs().spsr(), Some(u32::from(Mode::SYSTEM.0)));
}

#[test]
fn an_undefined_encoding_takes_the_undefined_exception() {
    // A `011` group encoding with bit 4 set: the ARMv6 media space.
    let h = running(&[0xe7f0_00f0]);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::UNDEFINED);
    assert_eq!(h.cpu.pc(), Exception::Undefined.vector());
    assert_eq!(h.cpu.reg(14), 0x1004);
}

#[test]
fn irq_is_taken_between_instructions_and_masked_by_the_i_bit() {
    let h = running(&[0xe3a0_0001, 0xe3a0_0002]);
    h.cpu.set_cpsr(u32::from(Mode::SYSTEM.0) | psr::I);
    h.cpu.set_irq(true);
    h.step();
    assert_eq!(h.cpu.reg(0), 1, "masked, so the instruction ran");

    h.cpu.set_cpsr(u32::from(Mode::SYSTEM.0));
    h.step();
    assert_eq!(h.cpu.mode(), Mode::IRQ);
    assert_eq!(h.cpu.pc(), Exception::Irq.vector());
    assert_eq!(h.cpu.reg(14), 0x1004 + 4, "the next instruction plus four");
    assert!(h.cpu.cpsr() & psr::I != 0);
    assert!(h.cpu.cpsr() & psr::F == 0, "an IRQ handler stays FIQ-able");
}

#[test]
fn fiq_outranks_irq_and_masks_both() {
    let h = running(&[0xe3a0_0001]);
    h.cpu.set_cpsr(u32::from(Mode::SYSTEM.0));
    h.cpu.set_irq(true);
    h.cpu.set_fiq(true);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::FIQ);
    assert_eq!(h.cpu.pc(), Exception::Fiq.vector());
    assert!(h.cpu.cpsr() & psr::I != 0);
    assert!(h.cpu.cpsr() & psr::F != 0);
}

#[test]
fn an_exception_return_restores_the_whole_cpsr() {
    // SWI #0 at 0x1000, then SUBS pc, lr, #4 at the SWI vector.
    let h = Harness::new();
    h.program(0x1000, &[0xef00_0000]);
    h.program(Exception::Swi.vector(), &[0xe25e_f004]);
    h.boot(0x1000);
    h.cpu.set_cpsr(u32::from(Mode::SYSTEM.0) | psr::C);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::SUPERVISOR);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::SYSTEM);
    assert_eq!(h.cpu.pc(), 0x1000, "SUBS pc, lr, #4 re-runs the SWI");
    assert!(h.cpu.cpsr() & psr::C != 0);
    assert!(
        h.cpu.cpsr() & psr::I == 0,
        "the mask came back with the SPSR"
    );
}

#[test]
fn ldm_with_the_s_bit_and_pc_is_the_other_exception_return() {
    // LDMIA sp!, {r0, pc}^
    let h = running(&[0xe8fd_8001]);
    h.cpu.set_cpsr(u32::from(Mode::IRQ.0) | psr::I);
    h.cpu.set_reg(13, 0x2000);
    h.poke(0x2000, 0xaaaa);
    h.poke(0x2004, 0x1234);
    let mut regs = h.regs();
    regs.set_spsr(u32::from(Mode::SYSTEM.0) | psr::V);
    h.cpu.set_regs(regs);
    h.step();
    assert_eq!(h.cpu.reg(0), 0xaaaa);
    assert_eq!(h.cpu.pc(), 0x1234);
    assert_eq!(h.cpu.mode(), Mode::SYSTEM);
    assert!(h.cpu.cpsr() & psr::V != 0);
}

#[test]
fn a_refused_access_is_an_external_abort() {
    // LDR r0, [r1] pointing into the hole between the two mapped regions.
    let h = running(&[0xe591_0000]);
    h.cpu.set_reg(1, 0x8000_0000);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::ABORT);
    assert_eq!(h.cpu.pc(), Exception::DataAbort.vector());
    assert_eq!(h.cpu.reg(14), 0x1000 + 8, "a data abort saves pc + 8");
    let (count, last) = h.cpu.bus_faults();
    assert_eq!(count, 1);
    assert_eq!(last, 0x8000_0000);
}

#[test]
fn a_data_abort_restores_the_base_register() {
    // LDR r0, [r1, #4]! into unmapped space.
    let h = running(&[0xe5b1_0004]);
    h.cpu.set_reg(1, 0x8000_0000);
    h.step();
    assert_eq!(h.cpu.reg(1), 0x8000_0000, "base restored abort model");
}

#[test]
fn exception_priorities_are_ordered_highest_first() {
    let order = [
        Exception::Reset,
        Exception::DataAbort,
        Exception::Fiq,
        Exception::Irq,
        Exception::PrefetchAbort,
        Exception::Undefined,
        Exception::Swi,
    ];
    for pair in order.windows(2) {
        assert!(pair[0] < pair[1], "{:?} outranks {:?}", pair[0], pair[1]);
    }
    assert_eq!(Exception::Reset.vector(), 0x00);
    assert_eq!(Exception::Fiq.vector(), 0x1c);
    assert!(Exception::Fiq.masks_fiq());
    assert!(!Exception::Irq.masks_fiq());
}

// ---------------------------------------------------------------------------
// Thumb
// ---------------------------------------------------------------------------

#[test]
fn thumb_shifts_and_moves_set_the_flags() {
    // LSL r0, r1, #3 ; MOV r2, #0
    let h = running_thumb(&[0x00c8, 0x2200]);
    h.cpu.set_reg(1, 0x1000_0001);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x8000_0008);
    assert!(h.cpu.cpsr() & psr::N != 0);
    assert_eq!(h.cpu.pc(), 0x1002);
    h.step();
    assert!(h.cpu.cpsr() & psr::Z != 0);
}

#[test]
fn thumb_add_and_subtract() {
    // ADD r0, r1, r2 ; SUB r3, r1, #2
    let h = running_thumb(&[0x1888, 0x1e8b]);
    h.cpu.set_reg(1, 10);
    h.cpu.set_reg(2, 5);
    h.step();
    assert_eq!(h.cpu.reg(0), 15);
    h.step();
    assert_eq!(h.cpu.reg(3), 8);
}

#[test]
fn thumb_alu_covers_neg_and_mul() {
    // NEG r0, r1 ; MUL r2, r3
    let h = running_thumb(&[0x4248, 0x435a]);
    h.cpu.set_reg(1, 5);
    h.step();
    assert_eq!(h.cpu.reg(0), (-5i32) as u32);
    assert!(h.cpu.cpsr() & psr::N != 0);

    h.cpu.set_reg(2, 6);
    h.cpu.set_reg(3, 7);
    h.step();
    assert_eq!(h.cpu.reg(2), 42);
}

#[test]
fn thumb_high_register_operations_do_not_touch_the_flags() {
    // MOV r8, r0 ; ADD r0, r8
    let h = running_thumb(&[0x4680, 0x4440]);
    h.cpu.set_reg(0, 0x8000_0000);
    h.cpu.set_cpsr(h.cpu.cpsr() & !psr::N);
    h.step();
    assert_eq!(h.cpu.reg(8), 0x8000_0000);
    assert!(h.cpu.cpsr() & psr::N == 0, "MOV Rd, Rm sets no flags here");
    h.step();
    assert_eq!(h.cpu.reg(0), 0);
    assert!(h.cpu.cpsr() & psr::Z == 0, "and neither does ADD");
}

#[test]
fn thumb_bx_returns_to_arm_state() {
    // BX r0
    let h = running_thumb(&[0x4700]);
    h.cpu.set_reg(0, 0x2000);
    h.step();
    assert!(!h.cpu.is_thumb());
    assert_eq!(h.cpu.pc(), 0x2000);
}

#[test]
fn thumb_blx_register_leaves_an_odd_return_address() {
    // BLX r1
    let h = running_thumb(&[0x4788]);
    h.cpu.set_reg(1, 0x3000);
    h.step();
    assert!(!h.cpu.is_thumb());
    assert_eq!(h.cpu.pc(), 0x3000);
    assert_eq!(h.cpu.reg(14), 0x1003, "bit 0 set: the return is to Thumb");
}

#[test]
fn thumb_literal_loads_are_word_aligned_from_pc_plus_four() {
    // At 0x1002 so that (pc + 4) & ~3 differs from pc + 4.
    let h = Harness::new();
    h.program_thumb(0x1002, &[0x4801]); // LDR r0, [pc, #4]
    h.boot(0x1002);
    h.cpu.set_cpsr(u32::from(Mode::SYSTEM.0) | psr::T);
    h.poke(0x1008, 0xfeed_face);
    h.step();
    assert_eq!(h.cpu.reg(0), 0xfeed_face);
}

#[test]
fn thumb_memory_operations() {
    // STR r0,[r1,r2] ; LDRB r3,[r1,#1] ; LDRH r4,[r1,#2] ; STR r0,[sp,#4]
    let h = running_thumb(&[0x5088, 0x784b, 0x884c, 0x9001]);
    h.cpu.set_reg(0, 0x1122_3344);
    h.cpu.set_reg(1, 0x2000);
    h.cpu.set_reg(2, 0);
    h.step();
    assert_eq!(h.peek(0x2000), 0x1122_3344);
    h.step();
    assert_eq!(h.cpu.reg(3), 0x33);
    h.step();
    assert_eq!(h.cpu.reg(4), 0x1122);
    h.cpu.set_reg(13, 0x3000);
    h.step();
    assert_eq!(h.peek(0x3004), 0x1122_3344);
}

#[test]
fn thumb_push_and_pop_including_lr_and_pc() {
    // PUSH {r0, lr} ; POP {r0, pc}
    let h = running_thumb(&[0xb501, 0xbd01]);
    h.cpu.set_reg(0, 0xaaaa);
    h.cpu.set_reg(13, 0x3000);
    h.cpu.set_reg(14, 0x1201); // an odd Thumb return address
    h.step();
    assert_eq!(h.cpu.reg(13), 0x2ff8);
    assert_eq!(h.peek(0x2ff8), 0xaaaa);
    assert_eq!(h.peek(0x2ffc), 0x1201);

    h.cpu.set_pc(0x1002);
    h.step();
    assert_eq!(h.cpu.reg(13), 0x3000);
    assert_eq!(h.cpu.pc(), 0x1200);
    assert!(h.cpu.is_thumb(), "POP {{pc}} interworks in ARMv5");
}

#[test]
fn thumb_stack_adjustment_and_address_formation() {
    // ADD sp, #8 ; SUB sp, #4 ; ADD r0, sp, #4 ; ADD r1, pc, #4
    let h = running_thumb(&[0xb002, 0xb081, 0xa801, 0xa101]);
    h.cpu.set_reg(13, 0x3000);
    h.step();
    assert_eq!(h.cpu.reg(13), 0x3008);
    h.step();
    assert_eq!(h.cpu.reg(13), 0x3004);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x3008);
    h.step();
    // (pc + 4) & ~3 + 4, with pc = 0x1006.
    assert_eq!(h.cpu.reg(1), 0x1008 + 4);
}

#[test]
fn thumb_conditional_and_unconditional_branches() {
    // BEQ +4 ; B +2
    let h = running_thumb(&[0xd001, 0xe000]);
    h.cpu.set_cpsr(h.cpu.cpsr() & !psr::Z);
    h.step();
    assert_eq!(h.cpu.pc(), 0x1002, "not taken");
    h.cpu.set_pc(0x1000);
    h.cpu.set_cpsr(h.cpu.cpsr() | psr::Z);
    h.step();
    assert_eq!(h.cpu.pc(), 0x1006, "pc + 4 + 2");
}

#[test]
fn the_thumb_bl_pair_is_two_instructions() {
    // BL to 0x1000 + 4 + 0x20: prefix carries the high bits, suffix the low.
    let h = running_thumb(&[0xf000, 0xf810]);
    h.step();
    assert_eq!(h.cpu.reg(14), 0x1004, "the prefix only computes lr");
    assert_eq!(h.cpu.pc(), 0x1002);
    h.step();
    assert_eq!(h.cpu.pc(), 0x1024);
    assert_eq!(h.cpu.reg(14), 0x1005);
    assert!(h.cpu.is_thumb());
}

#[test]
fn the_thumb_blx_pair_lands_in_arm_state_word_aligned() {
    // BLX with a suffix that would otherwise leave bit 1 set.
    let h = running_thumb(&[0xf000, 0xe811]);
    h.step();
    h.step();
    assert!(!h.cpu.is_thumb());
    assert_eq!(h.cpu.pc() & 3, 0, "the ARM target is word-aligned");
    assert_eq!(h.cpu.reg(14), 0x1005);
}

#[test]
fn a_thumb_swi_saves_the_halfword_return_address() {
    // SWI #0x12
    let h = running_thumb(&[0xdf12]);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::SUPERVISOR);
    assert_eq!(h.cpu.reg(14), 0x1002);
    assert!(!h.cpu.is_thumb(), "exceptions are entered in ARM state");
    assert_eq!(h.cpu.last_swi(), 0x12);
}

#[test]
fn an_undefined_thumb_encoding_is_still_undefined() {
    // 1101 1110 is architecturally undefined.
    let h = running_thumb(&[0xde00]);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::UNDEFINED);
    assert_eq!(h.cpu.reg(14), 0x1002);
}

#[test]
fn interworking_round_trips_through_both_states() {
    // ARM: BX r0 into Thumb at 0x2000; Thumb: BX r1 back to ARM at 0x1008.
    let h = Harness::new();
    h.program(0x1000, &[0xe12f_ff10]);
    h.program_thumb(0x2000, &[0x4708]); // BX r1
    h.boot(0x1000);
    h.cpu.set_reg(0, 0x2001);
    h.cpu.set_reg(1, 0x1008);
    h.step();
    assert!(h.cpu.is_thumb());
    h.step();
    assert!(!h.cpu.is_thumb());
    assert_eq!(h.cpu.pc(), 0x1008);
}

// ---------------------------------------------------------------------------
// Coprocessors and the MMU seam
// ---------------------------------------------------------------------------

#[test]
fn a_coprocessor_instruction_with_no_coprocessor_is_undefined() {
    // MRC p15, 0, r0, c0, c0, 0
    let h = running(&[0xee10_0f10]);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::UNDEFINED);
}

#[test]
fn the_cp15_stub_answers_mrc_and_remembers_mcr() {
    // MRC p15,0,r0,c0,c0,0 ; MCR p15,0,r1,c1,c0,0 ; MRC p15,0,r2,c1,c0,0
    let h = running(&[0xee10_0f10, 0xee01_1f10, 0xee11_2f10]);
    let cp15 = Arc::new(Cp15Stub::default());
    h.cpu.attach_coprocessor(15, cp15.clone());
    h.cpu.attach_mmu(cp15);
    h.step();
    assert_eq!(h.cpu.reg(0), Cp15Stub::ARM926EJS_ID);
    h.cpu.set_reg(1, 0x1234);
    h.step();
    h.step();
    assert_eq!(h.cpu.reg(2), 0x1234);
}

#[test]
fn cp15_can_move_the_vectors_at_runtime() {
    // MCR p15, 0, r0, c1, c0, 0 with the V bit set, then SWI.
    let h = running(&[0xee01_0f10, 0xef00_0000]);
    let cp15 = Arc::new(Cp15Stub::default());
    h.cpu.attach_coprocessor(15, cp15.clone());
    h.cpu.attach_mmu(cp15);
    h.cpu.set_reg(0, 1 << 13);
    h.step();
    h.step();
    assert_eq!(h.cpu.pc(), HIGH_VECTORS + Exception::Swi.vector());
}

#[test]
fn wait_for_interrupt_halts_until_a_line_comes_up() {
    // MCR p15, 0, r0, c7, c0, 4
    let h = running(&[0xee07_0f90, 0xe3a0_0001]);
    let cp15 = Arc::new(Cp15Stub::default());
    h.cpu.attach_coprocessor(15, cp15.clone());
    h.cpu.attach_mmu(cp15);
    h.cpu.set_cpsr(u32::from(Mode::SYSTEM.0) | psr::I);
    h.step();
    assert!(h.cpu.is_halted());
    assert_eq!(h.step(), 1, "a halted core idles rather than stopping");
    assert!(h.cpu.is_halted());
    // The line wakes it even though I is set; the exception is then masked.
    h.cpu.set_irq(true);
    h.step();
    assert!(!h.cpu.is_halted());
    assert_eq!(h.cpu.reg(0), 1);
}

/// An MMU that maps one page somewhere else and refuses another.
#[derive(Debug)]
struct TestMmu;

impl Mmu for TestMmu {
    fn translate(
        &self,
        va: u32,
        kind: AccessKind,
        _privileged: bool,
    ) -> core::result::Result<u32, Fault> {
        if va & 0xffff_f000 == 0x5000 && !kind.is_fetch() {
            return Err(Fault::TRANSLATION_PAGE);
        }
        if va & 0xffff_f000 == 0x4000 {
            return Ok(va - 0x4000 + 0x2000);
        }
        Ok(va)
    }
}

#[test]
fn an_mmu_can_relocate_and_can_fault() {
    // LDR r0, [r1] twice: once through the remap, once into the hole.
    let h = running(&[0xe591_0000, 0xe591_0000]);
    h.cpu.attach_mmu(Arc::new(TestMmu));
    h.poke(0x2010, 0xc0ffee);
    h.cpu.set_reg(1, 0x4010);
    h.step();
    assert_eq!(h.cpu.reg(0), 0xc0ffee);

    h.cpu.set_reg(1, 0x5000);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::ABORT);
}

/// A coprocessor that counts what it was asked and can be told to refuse.
#[derive(Debug, Default)]
struct CountingCp {
    calls: sync::Mutex<Vec<CpOp>>,
}

impl Coprocessor for CountingCp {
    fn mcr(&self, op: CpOp, _value: u32) -> core::result::Result<CpEffect, CpFault> {
        if op.crn == 9 {
            return Err(CpFault::Undefined);
        }
        self.calls.lock().push(op);
        Ok(CpEffect::NONE)
    }
}

#[test]
fn a_coprocessor_refusing_an_encoding_makes_it_undefined() {
    // MCR p14, 0, r0, c9, c0, 0 — the coprocessor declines c9.
    let h = running(&[0xee09_0e10]);
    h.cpu
        .attach_coprocessor(14, Arc::new(CountingCp::default()));
    h.step();
    assert_eq!(h.cpu.mode(), Mode::UNDEFINED);
}

// ---------------------------------------------------------------------------
// Endianness
// ---------------------------------------------------------------------------

#[test]
fn a_big_endian_core_assembles_words_the_other_way_round() {
    let h = Harness::with_config(Config::ARM926EJS.with_endian(Endian::Big));
    // With the `B` bit set an ARMv5 fetches its instructions big-endian too,
    // so the whole image is stored the other way round — including the code.
    h.program(0x1000, &[0xe591_0000u32.swap_bytes()]); // LDR r0, [r1]
    h.boot(0x1000);
    h.cpu.set_reg(1, 0x2000);
    h.poke(0x2000, 0x1122_3344);
    h.step();
    assert_eq!(h.cpu.reg(0), 0x4433_2211);
}

// ---------------------------------------------------------------------------
// The device surface
// ---------------------------------------------------------------------------

#[test]
fn the_state_round_trips_through_a_snapshot() {
    let h = running(&[0xe3a0_0042]);
    h.step();
    h.cpu.set_irq(true);
    h.cpu.set_reg(13, 0xdead_beef);
    h.cpu.set_cpsr(u32::from(Mode::FIQ.0));
    h.cpu.set_reg(9, 0x9999);

    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name).unwrap();
    let mut writer = StateWriter::new(shape);
    {
        let mut chunk = writer.chunk("cpu", CLASS.name, CLASS.version).unwrap();
        h.cpu.save(&mut chunk).unwrap();
    }
    let bytes = writer.to_vec().unwrap();

    let restored = Arm::new(Config::ARM926EJS);
    let reader = StateReader::new(&bytes).unwrap();
    let migrations = Migrations::new();
    let chunk = reader
        .load("cpu", CLASS.name, CLASS.version, &migrations)
        .unwrap();
    restored.load(&mut chunk.reader()).unwrap();

    assert_eq!(restored.regs(), h.regs());
    assert_eq!(restored.cycles(), h.cpu.cycles());
    assert!(restored.irq_asserted());
}

#[test]
fn the_device_surface_is_wired_up() {
    let h = running(&[0xe3a0_0042, 0xe3a0_0043]);
    assert!(Device::is_runnable(h.cpu.as_ref()));
    assert_eq!(h.cpu.class().name, "cpu.arm");

    let used = Device::run(
        h.cpu.as_ref(),
        crate::core::sched::Budget {
            until: crate::core::clock::GlobalTime::ZERO,
            ticks: 2,
        },
    );
    assert!(used.ticks >= 2);
    assert_eq!(h.cpu.reg(0), 0x43);
}

#[test]
fn realize_refuses_a_core_with_no_address_space() {
    let cpu = Arm::new(Config::ARM926EJS);
    let mut deferred = crate::core::device::Deferred::new();
    let mut ctx = crate::core::device::RealizeCtx::new(
        "cpu",
        crate::core::space::RequesterId::ANONYMOUS,
        &mut deferred,
    );
    assert!(cpu.realize(&mut ctx).is_err());
}

#[test]
fn a_cold_reset_returns_to_power_on_state() {
    let h = running(&[0xe3a0_0042]);
    h.step();
    h.cpu.set_irq(true);
    h.cpu.reset(ResetKind::Cold);
    assert_eq!(h.cpu.regs(), Regs::new());
    assert!(!h.cpu.irq_asserted());
    assert!(h.cpu.reset_pending());
}

#[test]
fn a_warm_reset_keeps_the_registers_and_the_input_levels() {
    let h = running(&[0xe3a0_0042]);
    h.step();
    h.cpu.set_irq(true);
    h.cpu.reset(ResetKind::Warm);
    assert_eq!(h.cpu.reg(0), 0x42);
    assert!(h.cpu.irq_asserted());
    assert!(h.cpu.reset_pending());
}

#[test]
fn properties_build_a_configured_core() {
    let props = crate::core::props::Props::new()
        .with("big-endian", true)
        .with("high-vectors", true)
        .with("store-pc-offset", 12u64);
    let cpu = Arm::from_props(&props).unwrap();
    assert_eq!(cpu.config().endian, Endian::Big);
    assert!(cpu.config().high_vectors);
    assert_eq!(cpu.config().store_pc_offset, 12);

    let bad = crate::core::props::Props::new().with("nonsense", 1u64);
    assert!(Arm::from_props(&bad).is_err());
}

#[test]
fn an_interrupt_pin_drives_the_input() {
    use crate::core::wire::{Level, WireId, WireSink};
    let h = running(&[0xe3a0_0042]);
    let pin = InterruptPin::new(h.cpu.clone(), Interrupt::Fiq, &[WireId(1), WireId(2)]);
    assert_eq!(pin.which(), Interrupt::Fiq);
    pin.set_level(WireId(1), 0, Level::High);
    assert!(h.cpu.fiq_asserted());
    // Wire-OR: the other driver still holds it up.
    pin.set_level(WireId(2), 0, Level::High);
    pin.set_level(WireId(1), 0, Level::Low);
    assert!(h.cpu.fiq_asserted());
    pin.set_level(WireId(2), 0, Level::Low);
    assert!(!h.cpu.fiq_asserted());
}

#[test]
fn a_core_with_no_address_space_stops_rather_than_spinning() {
    let cpu = Arm::new(Config::ARM926EJS);
    assert_eq!(cpu.step(), 0);
    assert_eq!(cpu.run(1000), 0);
}

// ---------------------------------------------------------------------------
// Decode and disassembly
// ---------------------------------------------------------------------------

#[test]
fn the_disassembler_prints_what_the_decoder_decoded() {
    for (word, text) in [
        (0xe3a0_0042u32, "MOV r0, #66"),
        (0xe081_0002, "ADD r0, r1, r2"),
        (0xe1b0_0081, "MOVS r0, r1, LSL #1"),
        (0xe1b0_0061, "MOVS r0, r1, RRX"),
        (0xe591_0004, "LDR r0, [r1, #4]"),
        (0xe5b1_0004, "LDR r0, [r1, #4]!"),
        (0xe491_0004, "LDR r0, [r1], #4"),
        (0xe4b1_0004, "LDRT r0, [r1], #4"),
        (0xe1d1_00b0, "LDRH r0, [r1]"),
        (0xe92d_000f, "STMDB sp!, {r0-r3}"),
        (0xe8fd_8001, "LDMIA sp!, {r0, pc}^"),
        (0xe12f_ff10, "BX r0"),
        (0xe16f_0f11, "CLZ r0, r1"),
        (0xe102_0051, "QADD r0, r1, r2"),
        (0xe160_0281, "SMULBB r0, r1, r2"),
        (0xe10f_0000, "MRS r0, CPSR"),
        (0xe121_f001, "MSR CPSR_c, r1"),
        (0xef12_3456, "SWI #1193046"),
        (0xee10_0f10, "MRC p15, #0, r0, c0, c0, #0"),
        (0xe102_0091, "SWP r0, r1, [r2]"),
        (0xe000_0291, "MUL r0, r1, r2"),
        (0xe081_0392, "UMULL r0, r1, r2, r3"),
    ] {
        let d = isa::decode(word);
        assert_eq!(alloc::format!("{d}"), text, "{word:08x}");
    }
}

#[test]
fn the_thumb_disassembler_prints_thumb_syntax() {
    for (half, text) in [
        (0x00c8u16, "LSL r0, r1, #3"),
        (0x1888, "ADD r0, r1, r2"),
        (0x2042, "MOV r0, #66"),
        (0x4348, "MUL r0, r1"),
        (0x4680, "MOV r8, r0"),
        (0x4700, "BX r0"),
        (0x4801, "LDR r0, [pc, #4]"),
        (0x6848, "LDR r0, [r1, #4]"),
        (0xb501, "PUSH {r0, lr}"),
        (0xbd01, "POP {r0, pc}"),
        (0xc806, "LDMIA r0!, {r1, r2}"),
        (0xdf12, "SWI #18"),
    ] {
        let d = thumb::decode(half);
        assert_eq!(alloc::format!("{d}"), text, "{half:04x}");
    }
}

#[test]
fn a_listing_resolves_branch_targets() {
    let listed = disasm::disassemble_arm(0x1000, 0xea00_0000);
    assert_eq!(listed.branch_target(), Some(0x1008));
    assert!(alloc::format!("{listed}").contains("0x00001008"));

    let listed = disasm::disassemble_thumb(0x1000, 0xd001);
    assert_eq!(listed.branch_target(), Some(0x1006));
}

#[test]
fn the_core_disassembles_its_own_memory_without_side_effects() {
    let h = running(&[0xe3a0_0042, 0xe081_0002]);
    let before = h.log().len();
    let listing = h.cpu.disassemble(0x1000, 2, false);
    assert_eq!(listing.len(), 2);
    assert!(alloc::format!("{}", listing[0]).contains("MOV r0, #66"));
    assert!(alloc::format!("{}", listing[1]).contains("ADD r0, r1, r2"));
    assert_eq!(h.log().len(), before, "a debug read leaves no trace");
}

#[test]
fn an_unreadable_listing_says_so_rather_than_inventing_bytes() {
    let listing = disasm::disassemble_run(0x1000, 2, false, |_| None);
    assert!(matches!(listing[0], disasm::Listed::Unreadable { .. }));
    assert_eq!(listing[1].addr(), 0x1004);
}

#[test]
fn every_condition_field_decodes_and_the_unconditional_space_is_separate() {
    // The same MOV under all fifteen real conditions, plus the 0b1111 space.
    for cond in 0u32..15 {
        let word = (cond << 28) | 0x03a0_0042;
        let d = isa::decode(word);
        assert_eq!(d.cond, isa::Cond(cond as u8));
        assert!(matches!(d.insn, isa::Insn::DataProc { .. }));
    }
    let d = isa::decode(0xf3a0_0042);
    assert_eq!(d.cond, isa::Cond::AL);
    assert!(d.is_undefined(), "not a data-processing instruction at all");
}

#[test]
fn decoding_never_panics_over_a_wide_sweep_of_encodings() {
    // Not exhaustive over 2^32, but wide enough to catch a shift that
    // overflows or an index that goes out of range.
    let mut word = 0x9e37_79b9u32; // an odd multiplier, for a cheap spread
    for _ in 0..200_000 {
        let d = isa::decode(word);
        let _ = alloc::format!("{d}");
        word = word.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
    }
    for half in 0..=u16::MAX {
        let d = thumb::decode(half);
        let _ = alloc::format!("{d}");
    }
}

#[test]
fn a_flat_mmu_is_the_default_and_needs_no_configuration() {
    let cpu = Arm::new(Config::ARM926EJS);
    // The default MMU is FlatMmu; asserting it through behaviour rather than
    // through a getter keeps the seam free to change shape.
    assert_eq!(
        FlatMmu.translate(0x1234, AccessKind::Fetch, true),
        Ok(0x1234)
    );
    assert_eq!(cpu.config().endian, Endian::Little);
}

// ---------------------------------------------------------------------------
// The seams a downstream SoC crate leans on
// ---------------------------------------------------------------------------

/// Records the privilege every translation was asked about.
#[derive(Debug, Default)]
struct PrivilegeSpy {
    seen: sync::Mutex<Vec<(u32, AccessKind, bool)>>,
}

impl Mmu for PrivilegeSpy {
    fn translate(
        &self,
        va: u32,
        kind: AccessKind,
        privileged: bool,
    ) -> core::result::Result<u32, Fault> {
        self.seen.lock().push((va, kind, privileged));
        Ok(va)
    }
}

#[test]
fn the_t_forms_ask_the_mmu_as_if_unprivileged() {
    // LDR r0, [r1] ; LDRT r2, [r1], #0
    let h = running(&[0xe591_0000, 0xe4b1_2000]);
    let spy = Arc::new(PrivilegeSpy::default());
    h.cpu.attach_mmu(spy.clone());
    h.cpu.set_reg(1, 0x2000);
    h.step();
    h.step();
    let seen = spy.seen.lock().clone();
    let data: Vec<_> = seen
        .iter()
        .filter(|(_, kind, _)| !kind.is_fetch())
        .collect();
    assert_eq!(data.len(), 2);
    assert!(data[0].2, "a plain LDR in System mode is privileged");
    assert!(!data[1].2, "LDRT asks as if it were user code");
}

#[test]
fn an_unmapped_fetch_is_a_prefetch_abort() {
    let h = running(&[0xe3a0_0042]);
    h.cpu.set_pc(0x8000_0000);
    h.step();
    assert_eq!(h.cpu.mode(), Mode::ABORT);
    assert_eq!(h.cpu.pc(), Exception::PrefetchAbort.vector());
    assert_eq!(
        h.cpu.reg(14),
        0x8000_0000 + 4,
        "a prefetch abort saves the faulting address plus four"
    );
}

/// An MMU that latches what it was told, the way a real CP15 fills FSR and FAR.
#[derive(Debug, Default)]
struct AbortRecorder {
    last: sync::Mutex<Option<(u32, Fault, AccessKind)>>,
}

impl Mmu for AbortRecorder {
    fn translate(
        &self,
        va: u32,
        _kind: AccessKind,
        _privileged: bool,
    ) -> core::result::Result<u32, Fault> {
        Ok(va)
    }

    fn report_abort(&self, va: u32, fault: Fault, kind: AccessKind) {
        *self.last.lock() = Some((va, fault, kind));
    }
}

#[test]
fn an_external_abort_is_reported_to_the_mmu_that_did_not_cause_it() {
    // LDR r0, [r1] into the hole — the address space refused, not the MMU, and
    // CP15 still has to publish it.
    let h = running(&[0xe591_0000]);
    let recorder = Arc::new(AbortRecorder::default());
    h.cpu.attach_mmu(recorder.clone());
    h.cpu.set_reg(1, 0x8000_0000);
    h.step();
    let last = *recorder.last.lock();
    assert_eq!(last, Some((0x8000_0000, Fault::EXTERNAL, AccessKind::Read)));
}

/// A coprocessor with one readable word, for `MRC` to `R15` and `LDC`/`STC`.
#[derive(Debug)]
struct WordCp(u32);

impl Coprocessor for WordCp {
    fn mrc(&self, _op: CpOp) -> core::result::Result<u32, CpFault> {
        Ok(self.0)
    }

    fn transfer_len(&self, _op: super::cp::CpTransfer) -> core::result::Result<u8, CpFault> {
        Ok(2)
    }

    fn read_word(
        &self,
        _op: super::cp::CpTransfer,
        index: u8,
    ) -> core::result::Result<u32, CpFault> {
        Ok(self.0 + u32::from(index))
    }

    fn write_word(
        &self,
        _op: super::cp::CpTransfer,
        _index: u8,
        _value: u32,
    ) -> core::result::Result<(), CpFault> {
        Ok(())
    }
}

#[test]
fn mrc_to_r15_loads_the_flags_rather_than_the_pc() {
    // MRC p14, 0, pc, c0, c0, 0
    let h = running(&[0xee10_fe10]);
    h.cpu.attach_coprocessor(14, Arc::new(WordCp(0x9000_0000)));
    h.cpu.set_cpsr(u32::from(Mode::SYSTEM.0) | psr::Q);
    h.step();
    assert_eq!(h.cpu.pc(), 0x1004, "the PC advanced normally");
    // 0x9 is `1001`: N and V set, Z and C clear.
    assert!(h.cpu.cpsr() & psr::N != 0);
    assert!(h.cpu.cpsr() & psr::Z == 0);
    assert!(h.cpu.cpsr() & psr::C == 0);
    assert!(h.cpu.cpsr() & psr::V != 0);
    assert!(h.cpu.cpsr() & psr::Q != 0, "only NZCV are touched");
}

#[test]
fn stc_writes_as_many_words_as_the_coprocessor_asks_for() {
    // STC p14, c0, [r1] — the coprocessor says two words.
    let h = running(&[0xed81_0e00]);
    h.cpu.attach_coprocessor(14, Arc::new(WordCp(0x1000)));
    h.cpu.set_reg(1, 0x2000);
    h.step();
    assert_eq!(h.peek(0x2000), 0x1000);
    assert_eq!(h.peek(0x2004), 0x1001);
}

#[test]
fn ldm_with_the_s_bit_loads_into_the_user_bank() {
    // LDMIA r0, {r13}^ while in IRQ mode: the user sp changes, IRQ's does not.
    let h = running(&[0xe8d0_2000]);
    h.cpu.set_cpsr(u32::from(Mode::SYSTEM.0));
    h.cpu.set_reg(13, 0x1111_1111);
    h.cpu.set_cpsr(u32::from(Mode::IRQ.0));
    h.cpu.set_reg(13, 0x2222_2222);
    h.cpu.set_reg(0, 0x2000);
    h.poke(0x2000, 0x3333_3333);
    h.step();
    assert_eq!(h.cpu.reg(13), 0x2222_2222, "the IRQ sp is untouched");
    assert_eq!(
        h.regs().reg_in_mode(Mode::USER, 13),
        0x3333_3333,
        "the user sp took the load"
    );
}

#[test]
fn msr_writes_the_spsr_of_the_current_mode_only() {
    // MSR SPSR_fsxc, r0
    let h = running(&[0xe16f_f000]);
    h.cpu.set_cpsr(u32::from(Mode::ABORT.0));
    h.cpu.set_reg(0, 0xf000_0000 | u32::from(Mode::USER.0));
    h.step();
    let regs = h.regs();
    assert_eq!(regs.spsr(), Some(0xf000_0000 | u32::from(Mode::USER.0)));
    // The other four SPSRs are untouched.
    assert_eq!(regs.spsr[Mode::IRQ.spsr_index().unwrap()], 0);
}

#[test]
fn the_compare_operations_leave_their_destination_alone() {
    // CMP/TST/TEQ/CMN with r0, r1 and a nominal Rd of r2.
    let h = running(&[0xe150_2001, 0xe110_2001, 0xe130_2001, 0xe170_2001]);
    h.cpu.set_reg(0, 4);
    h.cpu.set_reg(1, 4);
    h.cpu.set_reg(2, 0xdead);
    for _ in 0..4 {
        h.step();
        assert_eq!(h.cpu.reg(2), 0xdead);
    }
    assert!(h.cpu.cpsr() & psr::Z == 0, "the last one was CMN 4, 4");
}

#[test]
fn a_negative_branch_offset_goes_backwards() {
    // B -8: the offset field is 0xfffffe, so pc + 8 - 8.
    let h = running(&[0xeaff_fffe]);
    h.step();
    assert_eq!(h.cpu.pc(), 0x1000);
}

#[test]
fn run_stops_after_the_instruction_that_crosses_the_budget() {
    let h = running(&[0xe3a0_0001, 0xe3a0_0002, 0xe3a0_0003]);
    let used = h.cpu.run(2);
    assert_eq!(used, 2, "two one-cycle instructions");
    assert_eq!(h.cpu.reg(0), 2);
}
