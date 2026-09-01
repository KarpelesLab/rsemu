//! Tests for the MIPS core.
//!
//! The interesting ones are not "does `addu` add". They are the five places
//! this architecture is unlike every other core in this tree and an
//! implementation is likely to be wrong:
//!
//! 1. **Branch delay slots**, and specifically that an exception taken in one
//!    sets `Cause.BD` and points `EPC` at the *branch*.
//! 2. **Load delay slots** — MIPS I has no interlock, so the instruction after
//!    a load sees the destination register's old value.
//! 3. `LWL`/`LWR`/`SWL`/`SWR`, whose byte tables flip with the endianness pin.
//! 4. The **R3000** CP0: a three-deep `KU`/`IE` stack that `RFE` pops, no
//!    `EXL`, no `Wired`, and vectors that move with `BEV`.
//! 5. **Cache isolation**, where a store must not reach memory.
//!
//! Instructions are assembled by the encoders below rather than pasted in as
//! hex, so a test says what it means; `isa`'s own tests already prove the
//! decoder agrees with the table those encoders are written against.

use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::device::{Device, ResetKind};
use crate::core::error::Result;
use crate::core::exec::{ExitMask, ExitReason, ExitingCore};
use crate::core::props::Props;
use crate::core::space::{AddressSpace, RamStore, Region};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::value::Endian as RegionEndian;

use super::cp0::{self, Cp0, TlbEntry, cause_bits, exc, reg, status};
use super::isa::Endian;
use super::{Arch, CLASS, Config, Cpu};

// ---------------------------------------------------------------------------
// Assembly
// ---------------------------------------------------------------------------

/// A `SPECIAL` instruction: opcode zero, selected by the function code.
const fn special(funct: u32, rd: u32, rs: u32, rt: u32, sa: u32) -> u32 {
    (rs << 21) | (rt << 16) | (rd << 11) | (sa << 6) | funct
}

/// An immediate-format instruction.
const fn itype(op: u32, rs: u32, rt: u32, imm: u32) -> u32 {
    (op << 26) | (rs << 21) | (rt << 16) | (imm & 0xffff)
}

const fn nop() -> u32 {
    0
}
const fn addu(rd: u32, rs: u32, rt: u32) -> u32 {
    special(0x21, rd, rs, rt, 0)
}
const fn add(rd: u32, rs: u32, rt: u32) -> u32 {
    special(0x20, rd, rs, rt, 0)
}
const fn subu(rd: u32, rs: u32, rt: u32) -> u32 {
    special(0x23, rd, rs, rt, 0)
}
const fn or(rd: u32, rs: u32, rt: u32) -> u32 {
    special(0x25, rd, rs, rt, 0)
}
const fn sltu(rd: u32, rs: u32, rt: u32) -> u32 {
    special(0x2b, rd, rs, rt, 0)
}
const fn sll(rd: u32, rt: u32, sa: u32) -> u32 {
    special(0x00, rd, 0, rt, sa)
}
const fn sra(rd: u32, rt: u32, sa: u32) -> u32 {
    special(0x03, rd, 0, rt, sa)
}
const fn mult(rs: u32, rt: u32) -> u32 {
    special(0x18, 0, rs, rt, 0)
}
const fn multu(rs: u32, rt: u32) -> u32 {
    special(0x19, 0, rs, rt, 0)
}
const fn div(rs: u32, rt: u32) -> u32 {
    special(0x1a, 0, rs, rt, 0)
}
const fn mfhi(rd: u32) -> u32 {
    special(0x10, rd, 0, 0, 0)
}
const fn mflo(rd: u32) -> u32 {
    special(0x12, rd, 0, 0, 0)
}
const fn jr(rs: u32) -> u32 {
    special(0x08, 0, rs, 0, 0)
}
const fn jalr(rd: u32, rs: u32) -> u32 {
    special(0x09, rd, rs, 0, 0)
}
const SYSCALL: u32 = special(0x0c, 0, 0, 0, 0);
const BREAK: u32 = special(0x0d, 0, 0, 0, 0);
const RFE: u32 = 0x4200_0010;
const TLBWI: u32 = 0x4200_0002;
const TLBR: u32 = 0x4200_0001;
const TLBP: u32 = 0x4200_0008;
/// A coprocessor-2 operation — the encoding a guest uses to probe for a GTE.
const COP2: u32 = 0x4800_0000;

const fn addiu(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x09, rs, rt, imm as u32)
}
const fn addi(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x08, rs, rt, imm as u32)
}
const fn ori(rt: u32, rs: u32, imm: u32) -> u32 {
    itype(0x0d, rs, rt, imm)
}
const fn lui(rt: u32, imm: u32) -> u32 {
    itype(0x0f, 0, rt, imm)
}
const fn lw(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x23, rs, rt, imm as u32)
}
const fn lb(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x20, rs, rt, imm as u32)
}
const fn lhu(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x25, rs, rt, imm as u32)
}
const fn sw(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x2b, rs, rt, imm as u32)
}
const fn sb(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x28, rs, rt, imm as u32)
}
const fn lwl(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x22, rs, rt, imm as u32)
}
const fn lwr(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x26, rs, rt, imm as u32)
}
const fn swl(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x2a, rs, rt, imm as u32)
}
const fn swr(rt: u32, rs: u32, imm: i32) -> u32 {
    itype(0x2e, rs, rt, imm as u32)
}
const fn mfc0(rt: u32, cr: u32) -> u32 {
    0x4000_0000 | (rt << 16) | (cr << 11)
}
const fn mtc0(rt: u32, cr: u32) -> u32 {
    0x4080_0000 | (rt << 16) | (cr << 11)
}
const fn j(target: u32) -> u32 {
    (0x02 << 26) | ((target >> 2) & 0x03ff_ffff)
}
const fn jal(target: u32) -> u32 {
    (0x03 << 26) | ((target >> 2) & 0x03ff_ffff)
}

/// A branch whose displacement is worked out from **word indices** into the
/// program, which is how a test wants to say "branch back three".
///
/// The displacement counts from the delay slot, so it is `to - (from + 1)` and
/// not `to - from`. Writing that once here is the point.
const fn branch(op: u32, rs: u32, rt: u32, from: u32, to: u32) -> u32 {
    itype(op, rs, rt, (to as i32 - from as i32 - 1) as u32)
}
const fn beq(rs: u32, rt: u32, from: u32, to: u32) -> u32 {
    branch(0x04, rs, rt, from, to)
}
const fn bne(rs: u32, rt: u32, from: u32, to: u32) -> u32 {
    branch(0x05, rs, rt, from, to)
}
const fn bgez(rs: u32, from: u32, to: u32) -> u32 {
    branch(0x01, rs, 0x01, from, to)
}
const fn bltzal(rs: u32, from: u32, to: u32) -> u32 {
    branch(0x01, rs, 0x10, from, to)
}

// Register numbers, by their o32 names.
const V0: u32 = 2;
const A0: u32 = 4;
const T0: u32 = 8;
const T1: u32 = 9;
const T2: u32 = 10;
const T3: u32 = 11;
const K0: u32 = 26;
const SP: u32 = 29;
const RA: u32 = 31;

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// How much RAM a test machine has, at physical zero.
const RAM_SIZE: u64 = 0x10_0000;

/// The `kseg0` view of physical zero, where test programs run. Cached and
/// direct-mapped, which is where real MIPS kernel code lives.
const BASE: u32 = 0x8000_0000;

/// A processor with RAM at physical zero and a program already loaded.
struct Harness {
    cpu: Cpu,
    ram: Arc<RamStore>,
    endian: Endian,
}

impl Harness {
    /// Build a processor of the given configuration with `program` at physical
    /// zero, reachable through `kseg0` at [`BASE`].
    fn with(cfg: Config, program: &[u32]) -> Harness {
        let ram = Arc::new(RamStore::new(RAM_SIZE));
        let endian = cfg.endian;
        let region = Region::ram("ram", Arc::clone(&ram)).with_endian(if endian.is_big() {
            RegionEndian::Big
        } else {
            RegionEndian::Little
        });
        let space = AddressSpace::new("mem", 32);
        space.topology().map(region, 0).unwrap();
        let cpu = Cpu::new(cfg.with_reset_vector(BASE));
        cpu.attach_space(Arc::new(space));
        // A processor comes out of reset with `BEV` set, so its vectors are in
        // the boot ROM at 0xBFC0_01xx. A test program stands in for the kernel
        // code firmware has already handed control to, so it runs with the
        // cached vectors selected — which is also where a handler can be
        // written into RAM. `bev_moves_the_vectors_into_the_boot_rom` puts it
        // back and checks the other half.
        let mut c = cpu.cp0();
        c.status &= !status::BEV;
        cpu.set_cp0(c);
        let h = Harness { cpu, ram, endian };
        for (n, word) in program.iter().enumerate() {
            h.put_word(n as u64 * 4, *word);
        }
        h
    }

    /// A plain R3000A, little-endian.
    fn r3000a(program: &[u32]) -> Harness {
        Harness::with(Config::new(Arch::R3000A), program)
    }

    /// Write one word into RAM at a *physical* offset, in the guest's byte
    /// order.
    fn put_word(&self, offset: u64, word: u32) {
        let bytes = if self.endian.is_big() {
            word.to_be_bytes()
        } else {
            word.to_le_bytes()
        };
        for (k, byte) in bytes.iter().enumerate() {
            self.ram.write_u8(offset + k as u64, *byte).unwrap();
        }
    }

    /// Read one word out of RAM at a physical offset.
    fn get_word(&self, offset: u64) -> u32 {
        let mut b = [0u8; 4];
        for (k, slot) in b.iter_mut().enumerate() {
            *slot = self.ram.read_u8(offset + k as u64).unwrap();
        }
        if self.endian.is_big() {
            u32::from_be_bytes(b)
        } else {
            u32::from_le_bytes(b)
        }
    }

    /// Execute `n` instructions.
    fn steps(&self, n: usize) {
        for _ in 0..n {
            self.cpu.step();
        }
    }

    /// The exception code the most recent exception recorded.
    fn exc_code(&self) -> u32 {
        (self.cpu.cp0().cause & cause_bits::EXC_CODE) >> cause_bits::EXC_SHIFT
    }

    /// Whether the most recent exception was taken in a delay slot.
    fn bd(&self) -> bool {
        self.cpu.cp0().cause & cause_bits::BD != 0
    }
}

// ---------------------------------------------------------------------------
// A program that runs
// ---------------------------------------------------------------------------

#[test]
fn a_loop_runs_to_the_right_answer_and_writes_it_to_memory() {
    // The claim that matters: a real program, with a real backwards branch and
    // a real delay slot, computes the right number and stores it.
    //
    //        lui   $t0, 0x8000        # the kseg0 base
    //        addiu $t1, $zero, 0      # sum
    //        addiu $t2, $zero, 10     # counter
    //  loop: addu  $t1, $t1, $t2
    //        addiu $t2, $t2, -1
    //        bne   $t2, $zero, loop
    //        nop                      # the delay slot
    //        sw    $t1, 0x200($t0)
    //        j     .                  # spin
    //        nop
    const LOOP: u32 = 3;
    const BNE: u32 = 5;
    let program = [
        lui(T0, 0x8000),
        addiu(T1, 0, 0),
        addiu(T2, 0, 10),
        addu(T1, T1, T2),
        addiu(T2, T2, -1),
        bne(T2, 0, BNE, LOOP),
        nop(),
        sw(T1, T0, 0x200),
        j(BASE + 8 * 4),
        nop(),
    ];
    let h = Harness::r3000a(&program);
    // Three setup instructions, ten iterations of four, then the store.
    h.steps(3 + 10 * 4 + 1);
    assert_eq!(h.cpu.reg(T1), 55, "10 + 9 + … + 1");
    assert_eq!(h.get_word(0x200), 55, "and it reached memory");
}

#[test]
fn a_call_and_return_through_ra_works() {
    //        jal   f          # links 0x8000000c, not 0x80000008
    //        nop
    //        j     .
    //  f:    addiu $v0, $zero, 7
    //        jr    $ra
    //        nop
    let f = BASE + 3 * 4;
    let program = [
        jal(f),
        nop(),
        j(BASE + 2 * 4),
        addiu(V0, 0, 7),
        jr(RA),
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(2);
    assert_eq!(
        h.cpu.reg(RA),
        BASE + 8,
        "the link is the instruction after the delay slot"
    );
    h.steps(3);
    assert_eq!(h.cpu.reg(V0), 7);
    assert_eq!(h.cpu.pc(), BASE + 8, "and control came back past the slot");
}

// ---------------------------------------------------------------------------
// Hazard 1: branch delay slots
// ---------------------------------------------------------------------------

#[test]
fn the_instruction_after_a_taken_branch_executes() {
    // The branch skips its target's predecessor but never its own delay slot.
    let program = [
        beq(0, 0, 0, 3), // taken, to index 3
        addiu(T0, 0, 1), // the delay slot: runs
        addiu(T1, 0, 1), // skipped
        addiu(T2, 0, 1), // the target
    ];
    let h = Harness::r3000a(&program);
    h.steps(3);
    assert_eq!(h.cpu.reg(T0), 1, "the delay slot ran");
    assert_eq!(h.cpu.reg(T1), 0, "the instruction after it did not");
    assert_eq!(h.cpu.reg(T2), 1, "and the target did");
}

#[test]
fn the_instruction_after_an_untaken_branch_also_executes() {
    // Not a special case: the delay slot is fetched before the branch is
    // resolved, so it runs whatever the condition said.
    let program = [
        bne(0, 0, 0, 3), // never taken
        addiu(T0, 0, 1), // the delay slot: runs anyway
        addiu(T1, 0, 1),
    ];
    let h = Harness::r3000a(&program);
    h.steps(3);
    assert_eq!(h.cpu.reg(T0), 1);
    assert_eq!(h.cpu.reg(T1), 1);
}

#[test]
fn an_exception_in_a_delay_slot_sets_bd_and_points_epc_at_the_branch() {
    // The single most important test in this file. An exception taken on a
    // delay-slot instruction must set `Cause.BD` and put the **branch's**
    // address in `EPC`, so that returning re-executes the branch and its
    // decision is remade. A model that puts the delay slot's own address there
    // returns into a delay slot with no branch in front of it, and falls
    // straight through instead of jumping.
    let program = [
        beq(0, 0, 0, 4), // the branch, at BASE
        lw(T0, 0, 1),    // the delay slot: address 1, misaligned -> AdEL
        nop(),
        nop(),
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(2);
    let c = h.cpu.cp0();
    assert_eq!(h.exc_code(), exc::ADEL, "a misaligned load");
    assert!(h.bd(), "Cause.BD must say the fault was in a delay slot");
    assert_eq!(
        c.epc,
        BASE,
        "EPC names the branch at {BASE:#x}, not the delay slot at {:#x}",
        BASE + 4
    );
    assert_eq!(c.bad_vaddr, 1);
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
}

#[test]
fn an_exception_in_an_untaken_branchs_delay_slot_still_sets_bd() {
    // The delay slot is fetched before the branch is resolved, so it is a
    // delay slot whichever way the branch went. A model that set the flag only
    // on the taken path has *identical control flow* — which is why nothing
    // else in a test suite notices — and a wrong `EPC` exactly when an
    // interrupt lands after a branch that fell through.
    let program = [
        bne(0, 0, 0, 4), // never taken
        lw(T0, 0, 1),    // the delay slot, misaligned -> AdEL
        nop(),
        nop(),
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(2);
    assert_eq!(h.exc_code(), exc::ADEL);
    assert!(h.bd(), "an untaken branch still has a delay slot");
    assert_eq!(h.cpu.cp0().epc, BASE, "and EPC still names the branch");
}

#[test]
fn the_same_exception_outside_a_delay_slot_reports_its_own_address() {
    // The control for the test above: identical fault, no branch in front of
    // it, so `BD` is clear and `EPC` is the faulting instruction. Without this
    // pair, a model that hard-coded `EPC = pc - 4` would pass the first test.
    let program = [nop(), lw(T0, 0, 1), nop()];
    let h = Harness::r3000a(&program);
    h.steps(2);
    assert_eq!(h.exc_code(), exc::ADEL);
    assert!(!h.bd(), "no branch preceded it");
    assert_eq!(h.cpu.cp0().epc, BASE + 4);
}

#[test]
fn an_interrupt_taken_in_a_delay_slot_reports_the_branch_too() {
    // The case that makes this matter in practice: a timer interrupt arriving
    // while the processor happens to be in a delay slot. It happens constantly
    // and nothing else in a test suite will notice it.
    let program = [
        nop(),
        beq(0, 0, 1, 8), // the branch, at BASE + 4
        addiu(T0, 0, 1), // the delay slot, at BASE + 8
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(2);
    assert!(h.cpu.in_delay_slot(), "poised on the delay slot");

    // Enable interrupts, unmask hardware pin 0, and assert it.
    let mut c = h.cpu.cp0();
    c.status |= status::IEC | status::IM;
    h.cpu.set_cp0(c);
    h.cpu.set_interrupt(0, true);

    h.steps(1);
    assert_eq!(h.exc_code(), exc::INT);
    assert!(h.bd(), "the interrupt landed in a delay slot");
    assert_eq!(h.cpu.cp0().epc, BASE + 4, "EPC is the branch");
    assert_eq!(
        h.cpu.reg(T0),
        0,
        "the delay slot was aborted, not completed"
    );
}

#[test]
fn a_jump_target_is_formed_from_the_delay_slots_region() {
    // `J` takes its top four bits from the delay slot's program counter, so a
    // jump in the last word of a 256 MB region lands in the *next* one. Modelled
    // by putting the jump where its delay slot crosses the boundary is not
    // practical in a 1 MiB test machine, so this asserts the arithmetic through
    // a real execution in the region the test program lives in.
    let program = [j(BASE + 3 * 4), nop(), nop(), addiu(T0, 0, 1)];
    let h = Harness::r3000a(&program);
    h.steps(3);
    assert_eq!(h.cpu.pc(), BASE + 4 * 4);
    assert_eq!(h.cpu.reg(T0), 1);
}

#[test]
fn a_conditional_link_writes_ra_even_when_it_does_not_branch() {
    // `BLTZAL`/`BGEZAL` link unconditionally and branch conditionally. A model
    // that put the link inside the taken arm leaves `$ra` stale, which shows up
    // as a wild return much later.
    let program = [
        addiu(T0, 0, 1), // positive, so `bltzal` will not branch
        bltzal(T0, 1, 8),
        nop(),
        addiu(T1, 0, 5),
    ];
    let h = Harness::r3000a(&program);
    h.steps(3);
    assert_eq!(
        h.cpu.reg(RA),
        BASE + 3 * 4,
        "the link is written whether or not the branch is taken"
    );
    h.steps(1);
    assert_eq!(h.cpu.reg(T1), 5, "and control fell through");
}

// ---------------------------------------------------------------------------
// Hazard 2: load delay slots
// ---------------------------------------------------------------------------

#[test]
fn the_instruction_after_a_load_sees_the_registers_old_value() {
    // MIPS I has no load interlock. This is architectural and guest-visible:
    // real R3000 code and every MIPS I assembler depend on it, and a core that
    // interlocks computes different answers.
    let program = [
        lui(T0, 0x8000),
        addiu(T1, 0, 0x55), // t1 = 0x55
        lw(T1, T0, 0x200),  // load 0x1234 into t1 — not visible yet
        addu(T2, T1, 0),    // t2 = the OLD t1
        addu(T3, T1, 0),    // t3 = the NEW t1
    ];
    let h = Harness::r3000a(&program);
    h.put_word(0x200, 0x1234);
    h.steps(4);
    assert_eq!(
        h.cpu.reg(T2),
        0x55,
        "the load delay slot must see the old value"
    );
    h.steps(1);
    assert_eq!(h.cpu.reg(T3), 0x1234, "and the one after it the new one");
}

#[test]
fn the_load_is_visible_through_the_pending_slot_before_it_lands() {
    // The delayed write is state, and a debugger that showed only the register
    // file would be lying about what the next instruction reads.
    let program = [lui(T0, 0x8000), lw(T1, T0, 0x200), nop()];
    let h = Harness::r3000a(&program);
    h.put_word(0x200, 0xabcd);
    h.steps(2);
    assert_eq!(h.cpu.reg(T1), 0, "not in the register file yet");
    assert_eq!(h.cpu.pending_load(), Some((T1, 0xabcd)));
    h.steps(1);
    assert_eq!(h.cpu.reg(T1), 0xabcd);
    assert_eq!(h.cpu.pending_load(), None);
}

#[test]
fn an_instruction_that_writes_the_loaded_register_beats_the_load() {
    // Both write `$t1`, and the load's write-back is a cycle earlier, so the
    // ordinary instruction wins. Getting this backwards makes the load appear
    // to overwrite results computed after it.
    let program = [
        lui(T0, 0x8000),
        lw(T1, T0, 0x200),
        addiu(T1, 0, 0x77), // in the load delay slot, writing the same register
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.put_word(0x200, 0x1234);
    h.steps(4);
    assert_eq!(h.cpu.reg(T1), 0x77, "the arithmetic result survives");
}

#[test]
fn a_second_load_to_the_same_register_cancels_the_first() {
    // There is one delayed-write slot, and the second load claims it — so the
    // first load's value is never seen *at all*, not even for the one
    // instruction it would otherwise be visible for. That is the same rule an
    // ordinary instruction gets by writing its result after the settle, and
    // making the two differ would mean the hardware distinguished
    // `lw $t1,x; lw $t1,y` from `lw $t1,x; addiu $t1,…`, which nothing does.
    let program = [
        lui(T0, 0x8000),
        addiu(T1, 0, 0x99),
        lw(T1, T0, 0x200),
        lw(T1, T0, 0x204),
        addu(T2, T1, 0), // reads t1 while the second load is in flight
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.put_word(0x200, 0x1111);
    h.put_word(0x204, 0x2222);
    h.steps(5);
    assert_eq!(
        h.cpu.reg(T2),
        0x99,
        "the first load's value must never become visible"
    );
    h.steps(1);
    assert_eq!(h.cpu.reg(T1), 0x2222);
}

#[test]
fn a_loads_own_base_register_is_read_before_the_pending_load_settles() {
    // `lw $t0, 0(…)` then `lw $t1, 0($t0)`: the second load's *address* comes
    // from the old `$t0`, because a load reads its operands in the same stage
    // every other instruction does.
    let program = [
        lui(T0, 0x8000),
        lw(T0, T0, 0x200), // t0 <- 0x8000_0300, eventually
        lw(T1, T0, 0x204), // but this uses the old t0 = 0x8000_0000
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.put_word(0x200, 0x8000_0300);
    h.put_word(0x204, 0xdead);
    h.put_word(0x304, 0xbeef);
    h.steps(4);
    assert_eq!(h.cpu.reg(T1), 0xdead, "the old base was used");
}

#[test]
fn a_pending_load_still_lands_when_the_next_instruction_faults() {
    // R3000 exceptions are precise: the load instruction retired, so its
    // write-back happened even though the instruction after it never ran.
    let program = [
        lui(T0, 0x8000),
        lw(T1, T0, 0x200),
        lw(T2, 0, 1), // misaligned: faults in the load delay slot
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.put_word(0x200, 0x4242);
    h.steps(3);
    assert_eq!(h.exc_code(), exc::ADEL);
    assert_eq!(
        h.cpu.reg(T1),
        0x4242,
        "the completed load wrote back before the exception"
    );
}

#[test]
fn mfc0_has_a_load_delay_as_well() {
    // A coprocessor-to-register move arrives a stage late for the same reason
    // a load does. Kernels that read `Cause` and branch on it immediately are
    // the code this catches.
    let program = [
        addiu(T1, 0, 0x33),
        mfc0(T1, reg::PRID),
        addu(T2, T1, 0), // sees the OLD t1
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(3);
    assert_eq!(h.cpu.reg(T2), 0x33);
    assert_eq!(h.cpu.reg(T1), h.cpu.config().prid);
}

#[test]
fn a_part_with_a_load_interlock_makes_the_value_visible_at_once() {
    // MIPS II onwards. Not a preset this core ships, which is exactly why the
    // field has to be exercised: a `Config` nothing tests is a `Config` that
    // will be wrong the day someone needs it.
    let mut arch = Arch::R3000A;
    arch.load_interlock = true;
    let program = [
        lui(T0, 0x8000),
        addiu(T1, 0, 0x55),
        lw(T1, T0, 0x200),
        addu(T2, T1, 0),
    ];
    let h = Harness::with(Config::new(arch), &program);
    h.put_word(0x200, 0x1234);
    h.steps(4);
    assert_eq!(h.cpu.reg(T2), 0x1234, "no delay on an interlocked part");
    assert_eq!(h.cpu.pending_load(), None);
}

// ---------------------------------------------------------------------------
// Hazard 3: the unaligned transfers
// ---------------------------------------------------------------------------

/// Eight bytes of known data at physical `0x100`.
const UNALIGNED: [u8; 8] = [0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef];

fn unaligned_harness(endian: Endian, program: &[u32]) -> Harness {
    let h = Harness::with(Config::new(Arch::R3000A).with_endian(endian), program);
    for (i, b) in UNALIGNED.iter().enumerate() {
        h.ram.write_u8(0x100 + i as u64, *b).unwrap();
    }
    h
}

#[test]
fn an_adjacent_lwl_lwr_pair_loads_an_unaligned_word() {
    // The idiom every MIPS compiler emits, and the reason the pair is allowed
    // to sit adjacent with no `NOP`: the second one sees the first one's
    // result even though a load's value is otherwise a stage late.
    for endian in [Endian::Big, Endian::Little] {
        let (left, right) = if endian.is_big() {
            (0x101, 0x104)
        } else {
            (0x104, 0x101)
        };
        let program = [
            lui(T0, 0x8000),
            lwl(T1, T0, left),
            lwr(T1, T0, right),
            nop(),
        ];
        let h = unaligned_harness(endian, &program);
        h.steps(4);
        let want = if endian.is_big() {
            u32::from_be_bytes([0x23, 0x45, 0x67, 0x89])
        } else {
            u32::from_le_bytes([0x23, 0x45, 0x67, 0x89])
        };
        assert_eq!(h.cpu.reg(T1), want, "unaligned load, {endian}");
    }
}

#[test]
fn an_swl_swr_pair_stores_an_unaligned_word() {
    for endian in [Endian::Big, Endian::Little] {
        let (left, right) = if endian.is_big() {
            (0x201, 0x204)
        } else {
            (0x204, 0x201)
        };
        let program = [
            lui(T0, 0x8000),
            lui(T1, 0x1122),
            ori(T1, T1, 0x3344),
            swl(T1, T0, left),
            swr(T1, T0, right),
            nop(),
        ];
        let h = unaligned_harness(endian, &program);
        // Fill the target with a pattern so an over-wide store is visible.
        for i in 0..8 {
            h.ram.write_u8(0x200 + i, 0xff).unwrap();
        }
        h.steps(6);
        let want = if endian.is_big() {
            [0x11, 0x22, 0x33, 0x44]
        } else {
            [0x44, 0x33, 0x22, 0x11]
        };
        for (i, b) in want.iter().enumerate() {
            assert_eq!(
                h.ram.read_u8(0x201 + i as u64).unwrap(),
                *b,
                "byte {i} of the unaligned store, {endian}"
            );
        }
        assert_eq!(h.ram.read_u8(0x200).unwrap(), 0xff, "wrote too far left");
        assert_eq!(h.ram.read_u8(0x205).unwrap(), 0xff, "wrote too far right");
    }
}

#[test]
fn the_byte_wise_store_agrees_with_the_word_merge_it_replaces() {
    // `SWL`/`SWR` drive byte enables and never read, so the interpreter writes
    // the covered bytes one at a time rather than reading the word and merging
    // it back. That has to produce exactly what the manual's word-level table
    // says — at every offset, in both byte orders, for both halves — or the
    // two descriptions of one instruction have drifted.
    for endian in [Endian::Big, Endian::Little] {
        for byte in 0..4u32 {
            for left in [true, false] {
                let insn = if left { swl(T1, T0, 0) } else { swr(T1, T0, 0) };
                let addr = 0x300 + byte;
                let program = [
                    lui(T0, 0x8000),
                    ori(T0, T0, addr),
                    lui(T1, 0x1122),
                    ori(T1, T1, 0x3344),
                    insn,
                    nop(),
                ];
                let h = Harness::with(Config::new(Arch::R3000A).with_endian(endian), &program);
                let before = 0x5566_7788u32;
                h.put_word(0x300, before);
                h.steps(6);
                let want = if left {
                    super::isa::swl(before, 0x1122_3344, addr, endian)
                } else {
                    super::isa::swr(before, 0x1122_3344, addr, endian)
                };
                assert_eq!(
                    h.get_word(0x300),
                    want,
                    "{} at +{byte}, {endian}",
                    if left { "swl" } else { "swr" }
                );
            }
        }
    }
}

#[test]
fn an_unaligned_transfer_does_not_raise_an_address_error() {
    // Its whole reason to exist. A model that ran the alignment check first
    // would turn every unaligned access a compiler emits into an exception.
    let program = [lui(T0, 0x8000), lwl(T1, T0, 0x101), nop()];
    let h = unaligned_harness(Endian::Little, &program);
    h.steps(3);
    assert_eq!(h.cpu.cp0().cause & cause_bits::EXC_CODE, 0);
    assert_eq!(h.cpu.pc(), BASE + 3 * 4, "no vector was taken");
}

#[test]
fn the_ordinary_widths_do_raise_one() {
    for (insn, code) in [
        (lw(T1, 0, 0x102), exc::ADEL),
        (lhu(T1, 0, 0x101), exc::ADEL),
        (sw(T1, 0, 0x102), exc::ADES),
    ] {
        let h = Harness::r3000a(&[insn]);
        h.steps(1);
        assert_eq!(h.exc_code(), code, "for {insn:#010x}");
        assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
    }
    // A byte access is aligned by definition. Through `kseg0`, because a
    // `kuseg` address with no TLB entry behind it would fault for an entirely
    // different reason and the test would pass without proving anything.
    let h = Harness::r3000a(&[lui(T0, 0x8000), lb(T1, T0, 0x101), nop()]);
    h.steps(2);
    assert_eq!(h.cpu.pc(), BASE + 8);
}

#[test]
fn a_big_endian_core_reads_halfwords_the_other_way_round() {
    for endian in [Endian::Big, Endian::Little] {
        let program = [lui(T0, 0x8000), lhu(T1, T0, 0x100), nop()];
        let h = unaligned_harness(endian, &program);
        h.steps(3);
        let want = if endian.is_big() { 0x0123 } else { 0x2301 };
        assert_eq!(h.cpu.reg(T1), want, "{endian}");
    }
}

// ---------------------------------------------------------------------------
// Hazard 4: the R3000 CP0
// ---------------------------------------------------------------------------

#[test]
fn a_syscall_vectors_and_the_status_stack_carries_the_old_mode() {
    let program = [nop(), SYSCALL, nop()];
    let h = Harness::r3000a(&program);
    let mut c = h.cpu.cp0();
    // Start in user mode with interrupts on — except that a user-mode fetch
    // from kseg0 is itself an address error, so set the stack up as if the
    // kernel had been entered from user code and is running the test.
    c.status = status::IEC | status::KUP | status::IEP;
    h.cpu.set_cp0(c);
    h.steps(2);

    let c = h.cpu.cp0();
    assert_eq!(h.exc_code(), exc::SYS);
    assert_eq!(c.epc, BASE + 4);
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
    assert!(c.kernel_mode() && !c.interrupts_enabled());
    assert_ne!(
        c.status & status::IEP,
        0,
        "the interrupt enable that was current is now previous"
    );
}

#[test]
fn rfe_pops_the_status_stack_and_does_not_jump() {
    // `RFE` is not a return instruction. It restores the mode stack and
    // nothing else, and it is executed *in the delay slot* of the `jr $k0`
    // that does the jumping. A model that made it jump would work by accident
    // in the common idiom and break the moment a kernel used it any other way.
    let program = [RFE, addiu(T0, 0, 1)];
    let h = Harness::r3000a(&program);
    let mut c = h.cpu.cp0();
    c.status = status::KUP | status::IEP;
    h.cpu.set_cp0(c);
    h.steps(1);

    let c = h.cpu.cp0();
    assert_eq!(
        h.cpu.pc(),
        BASE + 4,
        "RFE did not change the program counter"
    );
    assert_ne!(c.status & status::KUC, 0, "back in user mode");
    assert_ne!(c.status & status::IEC, 0, "with interrupts back on");
}

#[test]
fn the_canonical_return_sequence_gets_back_to_the_faulting_instruction() {
    // What a real handler does, end to end: read `EPC`, jump to it, and pop
    // the status stack in the delay slot.
    //
    //   0: syscall                      <- the fault
    //   1: addiu $t0, $zero, 1          <- where we must come back to
    //   ...
    //  handler at the general vector, which is 0x8000_0080 = word 32:
    //  32: mfc0 $k0, $epc
    //  33: nop                          <- the load delay
    //  34: addiu $k0, $k0, 4            <- resume past the syscall
    //  35: jr   $k0
    //  36: rfe                          <- in the delay slot
    let mut program = vec![nop(); 40];
    program[0] = SYSCALL;
    program[1] = addiu(T0, 0, 1);
    program[32] = mfc0(K0, reg::EPC);
    program[33] = nop();
    program[34] = addiu(K0, K0, 4);
    program[35] = jr(K0);
    program[36] = RFE;
    let h = Harness::r3000a(&program);
    let mut c = h.cpu.cp0();
    c.status = status::IEC | status::KUP | status::IEP;
    h.cpu.set_cp0(c);

    h.steps(1);
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
    h.steps(5); // mfc0, nop, addiu, jr, rfe
    assert_eq!(
        h.cpu.pc(),
        BASE + 4,
        "back at the instruction after syscall"
    );
    let c = h.cpu.cp0();
    assert_ne!(c.status & status::IEC, 0, "and the stack was popped");
    h.steps(1);
    assert_eq!(h.cpu.reg(T0), 1);
}

#[test]
fn bev_moves_the_vectors_into_the_boot_rom() {
    // A processor comes out of reset with `BEV` set, because the cached
    // vectors at 0x8000_00xx are in RAM that nothing has written yet.
    let h = Harness::r3000a(&[SYSCALL]);
    let mut c = h.cpu.cp0();
    c.status |= status::BEV;
    h.cpu.set_cp0(c);
    h.steps(1);
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR_BEV);
    assert_eq!(cp0::GENERAL_VECTOR_BEV, 0xbfc0_0180);
}

#[test]
fn an_interrupt_needs_the_global_enable_and_the_mask_and_a_pin() {
    let h = Harness::r3000a(&[nop(); 8]);
    h.cpu.set_interrupt(2, true);
    h.steps(1);
    assert_eq!(h.cpu.pc(), BASE + 4, "masked and disabled: nothing happens");

    let mut c = h.cpu.cp0();
    c.status |= status::IEC; // enabled but still masked
    h.cpu.set_cp0(c);
    h.steps(1);
    assert_eq!(h.cpu.pc(), BASE + 8, "still masked");

    let mut c = h.cpu.cp0();
    // Pin 2 is Cause.IP[4], so the mask bit is Status.IM[4].
    c.status |= 1 << (status::IM_SHIFT + 4);
    h.cpu.set_cp0(c);
    h.steps(1);
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
    assert_eq!(h.exc_code(), exc::INT);
    assert_ne!(
        h.cpu.cp0().cause_with(h.cpu.interrupts()) & (1 << (cause_bits::IP_SHIFT + 4)),
        0,
        "and Cause.IP reports the pin"
    );
}

#[test]
fn a_software_interrupt_is_requested_through_cause() {
    // The only two bits of `Cause` software may write.
    let program = [ori(T0, 0, 1 << 8), mtc0(T0, reg::CAUSE), nop(), nop()];
    let h = Harness::r3000a(&program);
    let mut c = h.cpu.cp0();
    c.status |= status::IEC | (1 << status::IM_SHIFT);
    h.cpu.set_cp0(c);
    h.steps(3);
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
    assert_eq!(h.exc_code(), exc::INT);
}

#[test]
fn software_cannot_write_the_exception_code_into_cause() {
    // A guest that could would make the register a lie, and a handler reading
    // it back would dispatch on a cause nothing raised.
    let program = [
        lui(T0, 0xffff),
        ori(T0, T0, 0xffff),
        mtc0(T0, reg::CAUSE),
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(4);
    let c = h.cpu.cp0();
    assert_eq!(c.cause & cause_bits::EXC_CODE, 0);
    assert_eq!(c.cause & cause_bits::BD, 0);
    assert_eq!(
        c.cause & cause_bits::SW,
        cause_bits::SW,
        "but the two software bits took"
    );
}

#[test]
fn a_coprocessor_this_part_lacks_raises_cpu_with_ce_naming_it() {
    // How a guest probes for a GTE. Answering `RI` instead would tell it the
    // instruction does not exist rather than that the coprocessor does not,
    // and answering nothing at all would tell it there *is* one.
    let h = Harness::r3000a(&[COP2, nop()]);
    h.steps(1);
    let c = h.cpu.cp0();
    assert_eq!(h.exc_code(), exc::CPU);
    assert_eq!(
        (c.cause & cause_bits::CE) >> cause_bits::CE_SHIFT,
        2,
        "Cause.CE names coprocessor 2"
    );
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
}

#[test]
fn a_cu_bit_for_an_absent_coprocessor_reads_back_as_zero() {
    // The other half of feature probing: setting the bit and reading it back
    // must not claim a coprocessor the part does not have.
    let program = [lui(T0, 0xf000), mtc0(T0, reg::STATUS), nop(), nop()];
    let h = Harness::r3000a(&program);
    h.steps(3);
    let c = h.cpu.cp0();
    assert_ne!(c.status & status::CU0, 0, "CU0 exists");
    assert_eq!(c.status & 0xe000_0000, 0, "CU1..CU3 do not");
}

#[test]
fn cop0_is_reachable_from_kernel_mode_without_cu0() {
    // A reset handler runs before it has written `Status` at all, so kernel
    // mode has to reach CP0 unconditionally.
    let h = Harness::r3000a(&[mfc0(T0, reg::PRID), nop()]);
    h.steps(2);
    assert_eq!(h.cpu.reg(T0), h.cpu.config().prid);
}

#[test]
fn a_user_program_running_out_of_a_mapped_page_cannot_reach_cop0() {
    // The whole transition, the way a kernel really does it: program a TLB
    // entry for the user page, set `Status.KUp`, then `jr` to the user address
    // with `RFE` in the delay slot so the mode change and the jump land
    // together. The user code then touches CP0 with `CU0` clear and must get
    // coprocessor-unusable with `Cause.CE` naming coprocessor 0.
    //
    // Worth the setup: it is the only test here that executes an instruction
    // in user mode, out of a TLB-mapped page, having got there the way real
    // code gets there.
    // The user page is `kuseg` 0x0001_0000, mapped to physical frame 0 — so
    // the code at physical 0x100 is reachable at 0x0001_0100. Not page zero:
    // a reset TLB is 64 zeroed entries, every one of which claims page zero,
    // so adding a real mapping for it would be a shutdown rather than a hit
    // (`a_reset_tlb_full_of_identical_entries_is_a_shutdown`).
    const USER_PAGE: u32 = 0x0001_0000;
    const USER: u32 = USER_PAGE | 0x0000_0100;
    let mut program = vec![nop(); 80];
    program[0] = ori(T0, 0, 0); // Index = 0
    program[1] = mtc0(T0, reg::INDEX);
    program[2] = lui(T0, USER_PAGE >> 16); // EntryHi: the user VPN, ASID 0
    program[3] = mtc0(T0, reg::ENTRY_HI);
    program[4] = ori(T0, 0, (1 << 9) | (1 << 10)); // EntryLo: PFN 0, V and D
    program[5] = mtc0(T0, reg::ENTRY_LO);
    program[6] = TLBWI;
    program[7] = ori(T0, 0, status::KUP); // previous mode = user
    program[8] = mtc0(T0, reg::STATUS);
    program[9] = lui(K0, USER >> 16);
    program[10] = ori(K0, K0, USER & 0xffff);
    program[11] = jr(K0);
    program[12] = RFE; // the delay slot: pop the stack as control transfers
    program[0x100 / 4] = mfc0(T1, reg::PRID);

    let h = Harness::r3000a(&program);
    h.steps(13);
    assert_eq!(h.cpu.pc(), USER, "running out of the mapped user page");
    assert!(!h.cpu.cp0().kernel_mode(), "and in user mode");

    h.steps(1);
    assert_eq!(h.exc_code(), exc::CPU);
    assert_eq!(
        h.cpu.cp0().cause & cause_bits::CE,
        0,
        "CE names coprocessor 0"
    );
    assert_eq!(h.cpu.cp0().epc, USER);
    assert!(
        h.cpu.cp0().kernel_mode(),
        "and the exception re-entered the kernel"
    );
}

#[test]
fn a_user_mode_reference_to_a_kernel_segment_is_an_address_error() {
    // The fetch itself faults, because the program counter is in kseg0.
    let h = Harness::r3000a(&[nop(), nop()]);
    let mut c = h.cpu.cp0();
    c.status |= status::KUC;
    h.cpu.set_cp0(c);
    h.steps(1);
    assert_eq!(h.exc_code(), exc::ADEL);
    assert_eq!(h.cpu.cp0().bad_vaddr, BASE);
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
}

#[test]
fn kseg0_and_kseg1_are_two_views_of_the_same_physical_memory() {
    // The single most load-bearing fact about the MIPS memory map: firmware
    // runs uncached out of kseg1 and then jumps to the same code in kseg0.
    let program = [
        lui(T0, 0x8000),
        lui(T1, 0xa000),
        addiu(T2, 0, 0x5a),
        sw(T2, T0, 0x300), // through kseg0
        lw(T3, T1, 0x300), // read back through kseg1
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(6);
    assert_eq!(h.cpu.reg(T3), 0x5a);
    assert_eq!(h.get_word(0x300), 0x5a, "and it really is physical 0x300");
}

#[test]
fn an_overflow_traps_and_the_unsigned_form_does_not() {
    let h = Harness::r3000a(&[lui(T0, 0x7fff), ori(T0, T0, 0xffff), addi(T1, T0, 1), nop()]);
    h.steps(3);
    assert_eq!(h.exc_code(), exc::OV);
    assert_eq!(h.cpu.reg(T1), 0, "and nothing was written");

    let h = Harness::r3000a(&[
        lui(T0, 0x7fff),
        ori(T0, T0, 0xffff),
        addiu(T1, T0, 1),
        nop(),
    ]);
    h.steps(4);
    assert_eq!(h.cpu.reg(T1), 0x8000_0000);

    // The three-register forms behave the same way as their immediates.
    let h = Harness::r3000a(&[
        lui(T0, 0x7fff),
        ori(T0, T0, 0xffff),
        addiu(T2, 0, 1),
        add(T1, T0, T2),
        nop(),
    ]);
    h.steps(4);
    assert_eq!(h.exc_code(), exc::OV);
    let h = Harness::r3000a(&[
        lui(T0, 0x7fff),
        ori(T0, T0, 0xffff),
        addiu(T2, 0, 1),
        addu(T1, T0, T2),
        nop(),
    ]);
    h.steps(5);
    assert_eq!(h.cpu.reg(T1), 0x8000_0000);
}

#[test]
fn multiply_and_divide_land_in_hi_and_lo() {
    let program = [
        lui(T0, 0x0001),
        ori(T0, T0, 0x0000), // 0x10000
        multu(T0, T0),
        mfhi(T1),
        mflo(T2),
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(6);
    assert_eq!(h.cpu.reg(T1), 1, "0x10000 squared is 2^32");
    assert_eq!(h.cpu.reg(T2), 0);

    // Signed multiply, where the sign of the 64-bit product has to reach `HI`.
    let program = [
        addiu(T0, 0, -2),
        addiu(T1, 0, 3),
        mult(T0, T1),
        mfhi(T2),
        mflo(T3),
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(6);
    assert_eq!(h.cpu.reg(T3) as i32, -6);
    assert_eq!(h.cpu.reg(T2), 0xffff_ffff, "sign-extended into HI");

    // Signed, and division by zero, neither of which traps.
    let program = [
        addiu(T0, 0, -7),
        addiu(T1, 0, 2),
        div(T0, T1),
        mfhi(T2),
        mflo(T3),
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(6);
    assert_eq!(h.cpu.reg(T2) as i32, -1, "the remainder keeps the sign");
    assert_eq!(h.cpu.reg(T3) as i32, -3);

    let program = [addiu(T0, 0, 7), div(T0, 0), mflo(T2), nop()];
    let h = Harness::r3000a(&program);
    h.steps(4);
    assert_eq!(h.cpu.pc(), BASE + 4 * 4, "no exception was taken");
    assert_eq!(h.cpu.reg(T2), 0xffff_ffff);
}

#[test]
fn an_unassigned_encoding_is_a_reserved_instruction() {
    // Primary opcode 0x1f is not assigned on MIPS I.
    let h = Harness::r3000a(&[0x7c00_0000, nop()]);
    h.steps(1);
    assert_eq!(h.exc_code(), exc::RI);
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
}

#[test]
fn a_break_is_its_own_exception() {
    let h = Harness::r3000a(&[BREAK, nop()]);
    h.steps(1);
    assert_eq!(h.exc_code(), exc::BP);
    assert_eq!(h.cpu.cp0().epc, BASE);
}

// ---------------------------------------------------------------------------
// The TLB, and the part that has none
// ---------------------------------------------------------------------------

/// Give a processor a mapping of `vaddr`'s page to `phys`'s frame, written the
/// way an operating system would: through `EntryHi`/`EntryLo` and `TLBWI`.
fn program_tlb_entry(index: u32, vpn: u32, pfn: u32, flags: u32) -> Vec<u32> {
    vec![
        lui(T0, index << 8 >> 16),
        ori(T0, 0, index << 8),
        mtc0(T0, reg::INDEX),
        lui(T0, vpn >> 16),
        ori(T0, T0, vpn & 0xffff),
        mtc0(T0, reg::ENTRY_HI),
        lui(T0, pfn >> 16),
        ori(T0, T0, (pfn & 0xffff) | flags),
        mtc0(T0, reg::ENTRY_LO),
        TLBWI,
    ]
}

#[test]
fn a_program_can_map_a_kuseg_page_and_use_it() {
    // End to end through the guest's own instructions: write a TLB entry, read
    // it back with `TLBR`, probe for it with `TLBP`, then store through the
    // mapping and check the physical byte.
    const V: u32 = 0x0001_0000;
    const P: u32 = 0x0000_2000;
    // V (valid) and D (writable).
    let mut program = program_tlb_entry(5, V, P, (1 << 9) | (1 << 10));
    program.extend([
        TLBP,
        mfc0(T1, reg::INDEX),
        nop(),
        lui(T2, V >> 16),
        addiu(T3, 0, 0x77),
        sw(T3, T2, 0x40),
        nop(),
    ]);
    let h = Harness::r3000a(&program);
    h.steps(program.len());
    assert_eq!(h.cpu.cp0().cause & cause_bits::EXC_CODE, 0, "no exception");
    assert_eq!(h.cpu.reg(T1) >> 8, 5, "TLBP found the entry at index 5");
    assert_eq!(
        h.get_word(u64::from(P) + 0x40),
        0x77,
        "the store went through the mapping"
    );

    // And `TLBR` hands back exactly what was written.
    let h2 = Harness::r3000a(&program);
    h2.steps(10);
    let tlb = h2.cpu.tlb();
    assert_eq!(tlb.entry(5).vpn(), V);
    assert_eq!(tlb.entry(5).pfn(), P);
    assert!(tlb.entry(5).valid() && tlb.entry(5).writable());
}

#[test]
fn tlbr_reads_an_entry_back_into_the_registers() {
    let mut program = program_tlb_entry(9, 0x0002_0000, 0x0000_3000, 1 << 9);
    program.extend([
        // Scribble the registers so a `TLBR` that did nothing would show.
        lui(T0, 0),
        mtc0(T0, reg::ENTRY_HI),
        mtc0(T0, reg::ENTRY_LO),
        TLBR,
        nop(),
    ]);
    let h = Harness::r3000a(&program);
    h.steps(program.len());
    let c = h.cpu.cp0();
    assert_eq!(c.entry_hi & 0xffff_f000, 0x0002_0000);
    assert_eq!(c.entry_lo & 0xffff_f000, 0x0000_3000);
}

#[test]
fn a_kuseg_miss_takes_the_refill_vector_and_a_kseg2_miss_does_not() {
    // The distinction is the whole reason the R3000 has two vectors: the
    // refill handler is a hand-tuned fast path for the *user* page table, and
    // a kernel mapping is not on it.
    let h = Harness::r3000a(&[lw(T0, 0, 0x1000), nop()]);
    h.steps(1);
    assert_eq!(h.exc_code(), exc::TLBL);
    assert_eq!(
        h.cpu.pc(),
        cp0::REFILL_VECTOR,
        "kuseg misses go to 0x80000000"
    );
    let c = h.cpu.cp0();
    assert_eq!(c.bad_vaddr, 0x1000);
    assert_eq!(c.entry_hi & 0xffff_f000, 0x1000, "EntryHi holds the page");
    assert_eq!((c.context >> 2) & 0x7ffff, 1, "and Context indexes it");

    let h = Harness::r3000a(&[lui(T0, 0xc000), lw(T1, T0, 0), nop()]);
    h.steps(2);
    assert_eq!(h.exc_code(), exc::TLBL);
    assert_eq!(
        h.cpu.pc(),
        cp0::GENERAL_VECTOR,
        "kseg2 misses go to the general vector"
    );
}

#[test]
fn a_matching_entry_with_v_clear_takes_the_general_vector() {
    // Not a refill: the page table already has this page, so the fast handler
    // has nothing to add.
    let program = program_tlb_entry(0, 0x0001_0000, 0x0000_2000, 0); // V clear
    let mut program = program;
    program.extend([lui(T2, 1), lw(T3, T2, 0), nop()]);
    let h = Harness::r3000a(&program);
    h.steps(12);
    assert_eq!(h.exc_code(), exc::TLBL);
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
}

#[test]
fn a_store_to_a_page_whose_dirty_bit_is_clear_raises_mod() {
    // The `D` bit is a write-enable, not a hardware-maintained dirty bit: a
    // kernel clears it to catch the first write.
    let mut program = program_tlb_entry(0, 0x0001_0000, 0x0000_2000, 1 << 9); // V, no D
    program.extend([lui(T2, 1), sw(0, T2, 0), nop()]);
    let h = Harness::r3000a(&program);
    h.steps(12);
    assert_eq!(h.exc_code(), exc::MOD);
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
    // A *load* from the same page is fine.
    let mut program = program_tlb_entry(0, 0x0001_0000, 0x0000_2000, 1 << 9);
    program.extend([lui(T2, 1), lw(T3, T2, 0), nop()]);
    let h = Harness::r3000a(&program);
    h.steps(12);
    assert_eq!(h.cpu.cp0().cause & cause_bits::EXC_CODE, 0);
}

#[test]
fn a_reset_tlb_full_of_identical_entries_is_a_shutdown() {
    // A reset TLB is 64 zeroed entries, and matching is on `EntryHi` alone —
    // the `V` bit does not take an entry out of the comparison. So every one
    // of them claims virtual page zero, and the first access to that page
    // matches all 64 at once, which real silicon reports as a TLB shutdown and
    // latches in `Status.TS` until the next reset.
    //
    // This is why a MIPS kernel writes a distinct unmapped VPN into every
    // entry before it uses the TLB, and modelling it as a quiet hit on entry
    // zero would hide the bug in a kernel that forgot to.
    let h = Harness::r3000a(&[lw(T0, 0, 0x40), nop()]);
    assert_eq!(h.cpu.cp0().status & status::TS, 0);
    h.steps(1);
    assert_eq!(h.exc_code(), exc::TLBL);
    assert_ne!(h.cpu.cp0().status & status::TS, 0, "TS latched");
    // And an address on a page nothing claims is an ordinary refill.
    let h = Harness::r3000a(&[lw(T0, 0, 0x1040), nop()]);
    h.steps(1);
    assert_eq!(h.cpu.cp0().status & status::TS, 0);
}

#[test]
fn a_part_with_no_tlb_refuses_the_tlb_instructions() {
    // `ROADMAP.md` §6.1.1: an instruction the configured part lacks must trap,
    // not execute. On the LR33300 there is no TLB to write.
    for insn in [TLBWI, TLBR, TLBP, 0x4200_0006] {
        let h = Harness::with(Config::new(Arch::LR33300), &[insn, nop()]);
        h.steps(1);
        assert_eq!(
            h.exc_code(),
            exc::RI,
            "{insn:#010x} must be a reserved instruction with no TLB"
        );
    }
    // And `RFE`, which is a CP0 instruction but not a TLB one, still works.
    let h = Harness::with(Config::new(Arch::LR33300), &[RFE, nop()]);
    h.steps(1);
    assert_eq!(h.cpu.pc(), BASE + 4);
}

#[test]
fn kuseg_is_the_identity_on_a_part_with_no_tlb() {
    // Which is why a PlayStation's RAM answers at 0x0000_0000, 0x8000_0000 and
    // 0xA000_0000 alike. On a part *with* a TLB the same access is a refill.
    let program = [
        addiu(T2, 0, 0x5a),
        sw(T2, 0, 0x400), // kuseg address 0x400
        lui(T0, 0x8000),
        lw(T3, T0, 0x400), // the kseg0 view of physical 0x400
        nop(),
    ];
    let h = Harness::with(Config::new(Arch::LR33300), &program);
    h.steps(5);
    assert_eq!(h.cpu.cp0().cause & cause_bits::EXC_CODE, 0, "no fault");
    assert_eq!(h.cpu.reg(T3), 0x5a);

    let h = Harness::r3000a(&program);
    h.steps(2);
    assert_eq!(h.exc_code(), exc::TLBS, "with a TLB the same store misses");
}

#[test]
fn random_never_selects_a_wired_entry() {
    // There is no `Wired` register on an R3000 — that is an R4000 addition —
    // and the boundary is hard-wired at eight. A kernel keeps its permanent
    // mappings in slots 0 to 7 and does not have to program anything.
    let h = Harness::r3000a(&[nop(); 200]);
    let mut lowest = u32::MAX;
    for _ in 0..200 {
        h.steps(1);
        lowest = lowest.min(h.cpu.cp0().random);
    }
    assert_eq!(lowest, cp0::TLB_WIRED);
}

// ---------------------------------------------------------------------------
// Hazard 5: cache isolation
// ---------------------------------------------------------------------------

#[test]
fn an_isolated_store_never_reaches_memory() {
    // Firmware sets `Status.IsC` and scribbles the data cache to size it and
    // to invalidate it. A model that let those stores through would silently
    // corrupt guest RAM — which is exactly the "an access is never a silent
    // success" rule.
    let program = [
        lui(T0, 0x8000),
        addiu(T1, 0, 0x5a),
        sw(T1, T0, 0x500), // ordinary: reaches memory
        lui(T2, status::ISC >> 16),
        mtc0(T2, reg::STATUS),
        addiu(T1, 0, 0x77),
        sw(T1, T0, 0x500), // isolated: must not
        lw(T3, T0, 0x500), // reads the cache, not memory
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(9);
    assert_eq!(
        h.get_word(0x500),
        0x5a,
        "the isolated store reached RAM and corrupted it"
    );
    assert_eq!(h.cpu.reg(T3), 0x77, "and the cache holds what was written");
}

#[test]
fn clearing_isolation_brings_memory_back() {
    let program = [
        lui(T0, 0x8000),
        addiu(T1, 0, 0x5a),
        sw(T1, T0, 0x500),
        lui(T2, status::ISC >> 16),
        mtc0(T2, reg::STATUS),
        addiu(T1, 0, 0x77),
        sw(T1, T0, 0x500),
        mtc0(0, reg::STATUS), // isolation off
        lw(T3, T0, 0x500),
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(10);
    assert_eq!(h.cpu.reg(T3), 0x5a, "memory was never touched");
}

#[test]
fn the_swapped_cache_is_a_different_array() {
    // `SwC` sends an isolated access to the instruction cache instead, which
    // is how firmware invalidates it. Two arrays, so a value written to one
    // must not appear in the other.
    let program = [
        lui(T0, 0x8000),
        lui(T2, status::ISC >> 16),
        mtc0(T2, reg::STATUS),
        addiu(T1, 0, 0x11),
        sw(T1, T0, 0x40), // into the data cache
        lui(T2, (status::ISC | status::SWC) >> 16),
        mtc0(T2, reg::STATUS),
        lw(T3, T0, 0x40), // out of the instruction cache
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(9);
    assert_eq!(h.cpu.reg(T3), 0, "the two arrays are not the same store");
}

#[test]
fn an_isolated_byte_store_and_word_load_agree_on_byte_order() {
    for endian in [Endian::Big, Endian::Little] {
        let program = [
            lui(T0, 0x8000),
            lui(T2, status::ISC >> 16),
            mtc0(T2, reg::STATUS),
            addiu(T1, 0, 0x12),
            sb(T1, T0, 0x40),
            lw(T3, T0, 0x40),
            nop(),
        ];
        let h = Harness::with(Config::new(Arch::R3000A).with_endian(endian), &program);
        h.steps(7);
        let want = if endian.is_big() {
            0x1200_0000
        } else {
            0x0000_0012
        };
        assert_eq!(h.cpu.reg(T3), want, "{endian}");
    }
}

// ---------------------------------------------------------------------------
// The level-3 seam
// ---------------------------------------------------------------------------

#[test]
fn an_armed_syscall_leaves_the_core_instead_of_vectoring() {
    let program = [addiu(V0, 0, 4001), SYSCALL, addiu(A0, 0, 9), nop()];
    let h = Harness::r3000a(&program);
    h.cpu.set_exit_mask(ExitMask::USER);
    let run = h.cpu.run_to_exit_ticks(100);
    let exit = run.exit.expect("the syscall exits");
    assert_eq!(exit.reason, ExitReason::SYSCALL);
    assert_eq!(exit.pc, u64::from(BASE + 4));
    assert_eq!(exit.len, 4);
    assert_eq!(exit.detail, u64::from(exc::SYS));
    assert_eq!(
        h.cpu.pc(),
        BASE + 8,
        "the core is already past it, so a consumer resumes by running"
    );
    assert_eq!(h.cpu.reg(V0), 4001, "the syscall number is readable");
    // Resuming runs the instruction after the syscall.
    h.steps(1);
    assert_eq!(h.cpu.reg(A0), 9);
}

#[test]
fn a_fault_exit_rewinds_the_whole_control_pair() {
    // A consumer that services a fault and resumes must get a processor still
    // in the middle of the branch it was in the middle of. Rewinding to
    // `pc + 4` and clearing the delay-slot flag would lose the branch target.
    let program = [beq(0, 0, 0, 4), lw(T0, 0, 1), nop(), nop(), addiu(T1, 0, 3)];
    let h = Harness::r3000a(&program);
    h.cpu.set_exit_mask(ExitMask::USER);
    h.steps(1); // the branch
    assert!(h.cpu.in_delay_slot());
    let run = h.cpu.run_to_exit_ticks(100);
    let exit = run.exit.expect("the misaligned load exits");
    assert_eq!(exit.reason, ExitReason::FAULT);
    assert_eq!(exit.pc, u64::from(BASE + 4));
    assert_eq!(exit.address, 1);
    assert_eq!(h.cpu.pc(), BASE + 4, "still on the faulting instruction");
    assert_eq!(
        h.cpu.next_pc(),
        BASE + 4 * 4,
        "and still holding the target"
    );
    assert!(h.cpu.in_delay_slot(), "and still in the delay slot");

    // Patch the fault away and let it run on; the branch must still be taken.
    h.put_word(4, nop());
    h.cpu.set_exit_mask(ExitMask::NONE);
    h.steps(2);
    assert_eq!(h.cpu.reg(T1), 3, "control reached the branch target");
}

#[test]
fn an_unarmed_syscall_vectors_the_way_a_machine_needs() {
    let h = Harness::r3000a(&[SYSCALL, nop()]);
    let run = h.cpu.run_to_exit_ticks(1);
    assert!(run.exit.is_none());
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR);
}

#[test]
fn the_exiting_core_trait_reaches_the_stack_pointer() {
    let h = Harness::r3000a(&[nop()]);
    ExitingCore::set_sp(&h.cpu, 0x8000_f000);
    assert_eq!(h.cpu.reg(SP), 0x8000_f000);
    assert_eq!(ExitingCore::sp(&h.cpu), 0x8000_f000);
    ExitingCore::set_pc(&h.cpu, u64::from(BASE + 0x40));
    assert_eq!(h.cpu.pc(), BASE + 0x40);
    assert_eq!(h.cpu.next_pc(), BASE + 0x44, "and the pair came with it");
    assert!(!h.cpu.in_delay_slot());
}

// ---------------------------------------------------------------------------
// Device plumbing
// ---------------------------------------------------------------------------

#[test]
fn save_and_load_round_trip_to_an_identical_state() -> Result<()> {
    // Taken twice, at the two moments a naive snapshot loses something: with a
    // **load in flight**, and **between a branch and its delay slot**. They
    // cannot happen at once — a branch is not a load, so the instruction that
    // set the delay-slot flag cannot also have issued a load — which is
    // exactly why both have to be checked.
    let program = [
        lui(T0, 0x8000),
        lw(T1, T0, 0x200),
        beq(0, 0, 2, 6),
        addiu(T2, 0, 1),
        nop(),
        nop(),
        addiu(T3, 0, 2),
    ];
    let h = Harness::r3000a(&program);
    h.put_word(0x200, 0xfeed);
    h.cpu.set_interrupt(3, true);

    h.steps(2); // lui, lw — a load is on its way and not yet in the file
    assert_eq!(h.cpu.pending_load(), Some((T1, 0xfeed)));
    assert_eq!(h.cpu.reg(T1), 0);
    let (restored, bytes) = round_trip(&h.cpu)?;
    assert_eq!(restored.pending_load(), Some((T1, 0xfeed)));
    assert_eq!(restored.reg(T1), 0, "and it has not landed early");
    assert_eq!(restored.interrupts(), h.cpu.interrupts());
    assert_eq!(
        save_bytes(&restored)?,
        bytes,
        "a round trip is a fixed point"
    );

    h.steps(1); // beq — now poised on the delay slot
    assert!(h.cpu.in_delay_slot());
    let (restored, bytes) = round_trip(&h.cpu)?;
    assert!(restored.in_delay_slot(), "the delay slot survived");
    assert_eq!(restored.pc(), h.cpu.pc());
    assert_eq!(restored.next_pc(), h.cpu.next_pc(), "and so did the target");
    assert_eq!(save_bytes(&restored)?, bytes);
    Ok(())
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
fn a_restored_processor_continues_the_branch_it_was_in_the_middle_of() {
    // The behavioural half of the round-trip test: restore, run on, and check
    // that both the branch and the pending load did what they were going to.
    let program = [
        lui(T0, 0x8000),
        lw(T1, T0, 0x200),
        beq(0, 0, 2, 6),
        addiu(T2, 0, 1),
        nop(),
        nop(),
        addu(T3, T1, 0),
    ];
    let h = Harness::r3000a(&program);
    h.put_word(0x200, 0xfeed);
    h.steps(3);

    let mut shape = MachineShape::new();
    shape.add_device("cpu", CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("cpu", CLASS.name, CLASS.version).unwrap();
        h.cpu.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let other = Harness::r3000a(&program);
    other.put_word(0x200, 0xfeed);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("cpu", CLASS.name, CLASS.version, &Migrations::new())
        .unwrap();
    other.cpu.load(&mut chunk.reader()).unwrap();

    for machine in [&h, &other] {
        machine.steps(2); // the delay slot, then the branch target
        assert_eq!(machine.cpu.reg(T2), 1, "the delay slot ran");
        assert_eq!(machine.cpu.reg(T3), 0xfeed, "and the load had landed");
    }
}

#[test]
fn a_reset_returns_the_processor_to_its_reset_vector() {
    let h = Harness::r3000a(&[addiu(T0, 0, 1), addiu(T0, 0, 2)]);
    h.steps(2);
    assert_ne!(h.cpu.pc(), BASE);
    h.cpu.reset(ResetKind::Cold);
    assert_eq!(h.cpu.pc(), BASE);
    assert_eq!(h.cpu.reg(T0), 0);
    assert!(h.cpu.cp0().kernel_mode());
    assert_ne!(h.cpu.cp0().status & status::BEV, 0, "BEV comes up set");
}

#[test]
fn a_reset_request_is_latched_and_acted_on_at_the_next_step() {
    let h = Harness::r3000a(&[addiu(T0, 0, 1), addiu(T0, 0, 2)]);
    h.steps(1);
    h.cpu.request_reset();
    assert_eq!(h.cpu.pc(), BASE + 4, "not yet");
    h.steps(1);
    assert_eq!(
        h.cpu.pc(),
        BASE + 4,
        "the reset happened, then one step ran"
    );
    assert_eq!(h.cpu.reg(T0), 1);
}

#[test]
fn a_budget_is_never_overrun_and_the_debt_is_carried() {
    let h = Harness::r3000a(&[nop(); 64]);
    let mut total = 0u64;
    for _ in 0..8 {
        let used = h.cpu.run_budget(3);
        assert!(used <= 3, "a budget of 3 reported {used}");
        total += used;
    }
    assert_eq!(total, 24);
    // Every instruction charges exactly one access here — one fetch, no data
    // access — so the counters agree.
    assert_eq!(h.cpu.cycles(), 24 + h.cpu.cycle_debt());
}

#[test]
fn an_access_to_nothing_becomes_a_bus_error() {
    // The RAM is 1 MiB, so physical 0x0080_0000 is a hole. A MIPS processor
    // can report that to the guest, unlike a 6502.
    let h = Harness::r3000a(&[lui(T0, 0x8080), lw(T1, T0, 0), nop()]);
    h.steps(2);
    assert_eq!(h.exc_code(), exc::DBE);
    assert_eq!(h.cpu.bus_faults(), 1);
}

#[test]
fn properties_build_the_part_they_name() {
    let props = Props::new().with("arch", "lr33300").with("endian", "big");
    let cpu = Cpu::from_props(&props).unwrap();
    assert_eq!(cpu.config().arch.part, "lr33300");
    assert!(!cpu.config().arch.tlb);
    assert_eq!(cpu.config().endian, Endian::Big);

    // A typo is an error rather than a silent default.
    assert!(Cpu::from_props(&Props::new().with("arch", "r4000")).is_err());
    assert!(Cpu::from_props(&Props::new().with("nonsense", 1u64)).is_err());
}

#[test]
fn the_presets_are_distinct_and_findable_by_name() {
    for arch in Arch::ALL {
        assert_eq!(Arch::by_name(arch.part), Some(*arch));
    }
    assert_eq!(Arch::by_name("r4000"), None);
    // The axis that changes what a memory access *is*, so it is the one worth
    // pinning: the LSI part has no TLB and the MIPS-branded ones do.
    assert_eq!(
        Arch::ALL.iter().filter(|a| a.tlb).count(),
        2,
        "r3000a and r3051 have a TLB; lr33300 does not"
    );
    assert_eq!(Arch::by_name("lr33300").map(|a| a.tlb), Some(false));
    assert_eq!(Arch::IDT_R3051.dcache_bytes, 2048);
}

#[test]
fn a_cache_size_that_is_not_a_power_of_two_is_refused_at_construction() {
    // `new` validates and `realize` acts (`ROADMAP.md` §4.4). A masked index
    // into a 3000-byte array would alias unpredictably.
    let mut arch = Arch::R3000A;
    arch.dcache_bytes = 3000;
    assert!(Cpu::try_new(Config::new(arch)).is_err());
}

#[test]
fn the_disassembler_reads_the_program_the_processor_is_running() {
    let program = [lui(T0, 0x8000), beq(0, 0, 1, 3), nop(), addiu(T1, 0, 5)];
    let h = Harness::r3000a(&program);
    let listing = h.cpu.disassemble(BASE, 4);
    assert_eq!(listing.len(), 4);
    assert_eq!(listing[0].text, "lui t0, 0x8000");
    assert!(listing[1].delay_slot, "the branch marks its delay slot");
    assert_eq!(listing[2].text, "nop");
    assert_eq!(listing[3].text, "addiu t1, zero, 5");
}

#[test]
fn the_isa_description_covers_every_row() {
    let text = super::describe_isa();
    for insn in super::isa::TABLE {
        assert!(
            text.contains(insn.op.mnemonic()),
            "{} is missing from `describe`",
            insn.op.mnemonic()
        );
    }
}

#[test]
fn shifts_and_comparisons_do_what_the_manual_says() {
    let program = [
        lui(T0, 0x8000), // 0x8000_0000
        sra(T1, T0, 4),  // arithmetic: sign extends
        sll(T2, T0, 1),  // shifts the sign bit out
        sltu(T3, T0, 0), // unsigned: 0x8000_0000 is not < 0
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(5);
    assert_eq!(h.cpu.reg(T1), 0xf800_0000);
    assert_eq!(h.cpu.reg(T2), 0);
    assert_eq!(h.cpu.reg(T3), 0);
}

#[test]
fn r0_stays_zero_however_hard_a_program_tries() {
    let program = [
        addiu(0, 0, -1),
        lui(0, 0xffff),
        lui(T0, 0x8000),
        lw(0, T0, 0x200), // even through the load delay slot
        nop(),
        addu(T1, 0, 0),
        subu(T2, 0, 0),
        or(T3, 0, 0),
    ];
    let h = Harness::r3000a(&program);
    h.put_word(0x200, 0xffff_ffff);
    h.steps(8);
    assert_eq!(h.cpu.reg(0), 0);
    assert_eq!(h.cpu.reg(T1), 0);
    assert_eq!(h.cpu.reg(T2), 0);
    assert_eq!(h.cpu.reg(T3), 0);
}

#[test]
fn jalr_links_before_it_jumps() {
    // `jalr $ra, $ra` is legal precisely because the link is written first.
    let target = BASE + 4 * 4;
    let program = [
        lui(RA, target >> 16),
        ori(RA, RA, target & 0xffff),
        jalr(RA, RA),
        nop(),
        addiu(T0, 0, 1),
    ];
    let h = Harness::r3000a(&program);
    h.steps(4);
    assert_eq!(h.cpu.pc(), target);
    assert_eq!(h.cpu.reg(RA), BASE + 4 * 4, "the old target was linked");
    h.steps(1);
    assert_eq!(h.cpu.reg(T0), 1);
}

#[test]
fn jalr_writes_the_register_it_names_and_not_ra_by_default() {
    // `jalr $rs` is an *assembler* abbreviation for `jalr $ra, $rs`, not a
    // hardware default. `jalr $zero, $rs` therefore discards the link, and a
    // model that substituted `$31` for a zero `rd` would corrupt the return
    // address of whatever called the code doing it.
    let target = BASE + 4 * 4;
    let program = [
        lui(T0, target >> 16),
        ori(T0, T0, target & 0xffff),
        addiu(RA, 0, 0x55),
        jalr(0, T0), // link discarded
        nop(),
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(5);
    assert_eq!(h.cpu.reg(RA), 0x55, "$ra was not touched");
    assert_eq!(h.cpu.pc(), target, "and the jump still happened");
}

#[test]
fn every_regimm_encoding_branches_and_only_two_of_them_link() {
    // The `rt` field selects the comparison with bit 0 alone, so all 32
    // encodings exist; the link is narrower and happens only for `10000` and
    // `10001`. Executed rather than merely decoded, because the two halves of
    // the rule live in different places — the table and the register write.
    for rt in 0..32u32 {
        let program = [
            addiu(T0, 0, -1), // negative, so an LTZ branches and a GEZ does not
            itype(0x01, T0, rt, 2),
            addiu(T1, 0, 1),
            nop(),
        ];
        let h = Harness::r3000a(&program);
        h.steps(3);
        let links = rt == 0x10 || rt == 0x11;
        assert_eq!(
            h.cpu.reg(RA) != 0,
            links,
            "rt = {rt:05b} linked = {}",
            h.cpu.reg(RA) != 0
        );
        let taken = rt & 1 == 0; // bit 0 clear is "less than zero"
        assert_eq!(
            h.cpu.pc() != BASE + 3 * 4,
            taken,
            "rt = {rt:05b} branch taken = {}",
            h.cpu.pc() != BASE + 3 * 4
        );
    }
}

#[test]
fn a_branch_in_a_delay_slot_measures_its_target_from_where_it_really_lands() {
    // A branch inside another branch's delay slot is UNPREDICTABLE, and this
    // is what happens: the outer branch still lands, and the inner one's
    // displacement — and a jump's 256 MB region — are measured from *there*,
    // because that is where the inner branch's own delay slot really is.
    //
    // The common case is the same statement: the delay slot is at `pc + 4` and
    // the arithmetic is the familiar one. Measuring from `pc + 4` regardless
    // gets the nested case wrong in the top four bits of a `J`.
    let outer = BASE + 8 * 4;
    let program = [
        j(outer),       // 0: the outer jump
        j(BASE + 0x40), // 1: in its delay slot
        nop(),          // 2
        nop(),
        nop(),
        nop(),
        nop(),
        nop(),
        addiu(T0, 0, 1), // 8: the outer target, and the inner jump's slot
        nop(),
    ];
    let h = Harness::r3000a(&program);
    h.steps(2);
    assert_eq!(h.cpu.pc(), outer, "the outer jump still landed");
    assert!(h.cpu.in_delay_slot());
    assert_eq!(
        h.cpu.next_pc(),
        BASE + 0x40,
        "and the inner jump's target was formed from the outer one's"
    );
    h.steps(1);
    assert_eq!(h.cpu.reg(T0), 1, "the outer target ran as the inner slot");
    assert_eq!(h.cpu.pc(), BASE + 0x40);
}

#[test]
fn a_bgez_on_zero_is_taken() {
    let program = [
        bgez(0, 0, 3),
        addiu(T0, 0, 1),
        addiu(T1, 0, 1),
        addiu(T2, 0, 1),
    ];
    let h = Harness::r3000a(&program);
    h.steps(3);
    assert_eq!(h.cpu.reg(T0), 1, "delay slot");
    assert_eq!(h.cpu.reg(T1), 0, "skipped");
    assert_eq!(h.cpu.reg(T2), 1, "target");
}

#[test]
fn a_second_exception_pushes_the_stack_again_because_there_is_no_exl() {
    // The R3000 has no `Status.EXL` and no protection against re-entry. A
    // model that borrowed the R4000's would suppress the second exception,
    // which is a different machine.
    let mut program = vec![nop(); 40];
    program[0] = SYSCALL;
    program[32] = SYSCALL; // the handler faults immediately
    let h = Harness::r3000a(&program);
    let mut c = h.cpu.cp0();
    c.status = status::IEC | status::KUP | status::IEP;
    h.cpu.set_cp0(c);
    h.steps(2);
    assert_eq!(h.cpu.pc(), cp0::GENERAL_VECTOR, "it vectored again");
    assert_eq!(
        h.cpu.cp0().epc,
        cp0::GENERAL_VECTOR,
        "and EPC now names the handler, losing the original return"
    );
}

#[test]
fn a_cp0_register_this_part_does_not_have_reads_as_zero() {
    let h = Harness::r3000a(&[mfc0(T0, 20), nop()]);
    h.steps(2);
    assert_eq!(h.cpu.reg(T0), 0);
    assert_eq!(
        h.cpu.cp0().cause & cause_bits::EXC_CODE,
        0,
        "and does not trap"
    );
}

#[test]
fn a_clone_of_the_cp0_file_is_debug_and_independent() {
    // `#[derive(Debug)]` on every public type, and a copy a debugger takes
    // must not alias the running processor.
    let h = Harness::r3000a(&[nop()]);
    let mut c: Cp0 = h.cpu.cp0();
    let before = c.status;
    c.status = 0xdead_beef;
    assert_eq!(h.cpu.cp0().status, before);
    assert!(!alloc::format!("{c:?}").is_empty());

    let entry = TlbEntry {
        hi: 0x1000_0000,
        lo: 0x0020_0300,
    };
    assert!(!alloc::format!("{entry:?}").is_empty());
    assert!(!alloc::format!("{:?}", h.cpu).is_empty());
}
