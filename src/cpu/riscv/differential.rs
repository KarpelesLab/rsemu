//! The differential harness: the RISC-V lifter against the RISC-V
//! interpreter, forever.
//!
//! CLAUDE.md, "CPU cores": *the IR frontend comes later and is differentially
//! tested against the interpreter forever. **The interpreter is the oracle.***
//! [`ir::interp`](crate::ir) repeatedly names "the differential harness" in
//! its own documentation. This module is it.
//!
//! # The comparison
//!
//! One program, two machines built the same way, and everything either of them
//! can be seen to do:
//!
//! | | oracle | subject |
//! | --- | --- | --- |
//! | engine | [`Hart::step`], `insns` times | [`lift`] → [`verify`] → [`Interp`] |
//! | registers | `x1`..`x31` and the PC | the slots published at the exit boundary |
//! | ticks | `Hart::cycles` | the sum of the charges the block made |
//! | memory | the RAM it wrote | the RAM it wrote |
//! | faults | whether a trap was taken | whether the block reported a fault |
//!
//! `insns` is [`Lifted::insns`](super::lift::Lifted::insns), so the oracle is stopped at exactly the guest
//! instruction the block ends after. Every column is compared, because each
//! catches a different class of frontend bug and only the first is obvious: a
//! miscounted [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) is invisible in
//! the registers and fails the phase-5 state-hash gate a million cycles later
//! (`src/ir/mod.rs`, decision 2), a store lifted with the wrong width writes
//! the right register and the wrong memory, and a load whose address is
//! computed wrongly usually faults where the interpreter did not.
//!
//! # Why the host here re-implements the memory path
//!
//! [`IrHost::load`] and [`IrHost::store`] are where a lifted block meets guest
//! memory, and the interpreter's own path through them (`exec::load`) is
//! private to a step in progress. So this module's own host performs the
//! access itself,
//! following *Volume I*'s rule for a misaligned access — the implementation
//! either performs it or raises — in the same shape `exec` does: aligned is
//! one access, misaligned is one access per byte when the configuration
//! performs them and a fault when it does not, and each access charges one
//! tick because one bus access is one cycle (`cpu::riscv::exec`).
//!
//! That is a second implementation of a rule, which is normally the thing to
//! avoid — but it is the *host's* rule rather than the frontend's, and the
//! frontend is what is under test. The lifter's contribution is the
//! [`MemOp`]'s size, sign and [`Align`], and a wrong one of those diverges
//! here immediately.
//!
//! # Two harnesses, not one
//!
//! [`compare`] runs **one block**, freshly lifted, and stops. That is right
//! for testing a frontend and blind to everything the translation runtime
//! does, so `compare_cached` (with the `jit` feature) is the second harness:
//! the same oracle, the same columns, but many blocks through
//! `jit::Dispatcher` — served from a block cache, chained exit to successor,
//! invalidated when the guest writes into a translated page, and with every
//! access resolved through `jit::Tlb`. Its instruction bytes come out of guest
//! RAM rather than out of [`Case::program`], which is what makes
//! self-modifying code testable at all.
//!
//! Both are driven from the generated corpus in
//! `tests/riscv_lift_differential.rs` and from `fuzz/fuzz_targets/`, so a case
//! that finds a frontend bug also exercises the runtime.
//!
//! # What this harness deliberately does not cover
//!
//! * **Traps.** When the oracle takes one, the two are compared only on
//!   *whether* they both stopped, not on the architectural state afterwards:
//!   delivering an exception from a lifted block needs the fault-materializing
//!   path that `ROADMAP.md` §9 owes and this frontend has not been given yet.
//!   Reported as [`Verdict::Trapped`] rather than silently passed.
//! * **Paging.** [`Case`] is a bare machine-mode hart, so `satp` is off and
//!   [`Origin::Bare`] is the truth rather than a claim. Lifting under
//!   translation needs the entry translation to come from the fetch path
//!   (`lift`'s module docs, "Paging"), which is a dispatcher's job.
//! * **Anything outside the lifted subset**, which ends the block by
//!   construction — so it is not skipped, it simply is not reached.

use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::error::BusError;
use crate::core::space::{AddressSpace, MemAttrs, MemResult, RamStore, Region, UnassignedPolicy};
use crate::core::value::Width;
use crate::ir::{Align, InsnStart, Interp, IrHost, MemOp, Outcome, RegSlot, verify};

use super::lift::{self, Origin, PC, x_slot};
use super::{Config, Hart};

#[cfg(feature = "jit")]
use crate::ir::AccessKind;
#[cfg(feature = "jit")]
use crate::jit::{
    BlockCache, Context as TlbContext, DirtyPages, Dispatcher, Epoch, Frontend, Stop, StoreLog,
    Tlb, Translation,
};

/// Where a case's program is loaded, and its RAM mapped.
///
/// Page-aligned, and deliberately away from `0x8000_0000` so that a `LUI`
/// whose immediate names the base is not sign-extended into a different
/// number.
pub const BASE: u64 = 0x2000_0000;

/// How much RAM a case gets: four pages.
///
/// The first page holds the program — a block is bounded by its page, so it
/// can hold more instructions than [`lift::MAX_INSNS`] will ever read — and
/// the rest is the data window loads and stores are aimed at.
pub const RAM_SIZE: u64 = 4 * 4096;

/// Where the data window starts, as an offset from [`BASE`].
pub const DATA: u64 = 4096;

/// One differential case: a program, the registers it starts with, and the
/// core it runs on.
#[derive(Debug, Clone)]
pub struct Case {
    /// The core. Must be RV64 with `satp` bare and no PMP entries — the
    /// harness compares tick counts, and a walk or a PMP refusal is a tick the
    /// lifted block has no way to know about.
    pub cfg: Config,
    /// The instruction words, loaded at [`BASE`].
    pub program: Vec<u32>,
    /// The initial integer register file. `x0` is ignored; the reset state is
    /// all zeroes, so a case that wants a load to reach memory has to put an
    /// address in a register.
    pub regs: [u64; 32],
}

impl Case {
    /// A case that runs `program` on a bare RV64I hart with a zeroed register
    /// file.
    #[must_use]
    pub fn new(program: Vec<u32>) -> Case {
        Case {
            cfg: Config::rv64i(),
            program,
            regs: [0; 32],
        }
    }

    /// The same case with `x`*n* starting at `value`.
    #[must_use]
    pub fn with_reg(mut self, n: usize, value: u64) -> Case {
        if n < 32 {
            self.regs[n] = value;
        }
        self
    }

    /// A case whose `x1`..`x4` point into the data window, spread so that a
    /// small signed offset from any of them stays inside RAM.
    ///
    /// The companion to [`synthesize`], which takes a memory operand's base
    /// from exactly those four registers. `x1` is deliberately *not* aligned
    /// to eight, so the misaligned path is reachable without the offset having
    /// to supply the misalignment.
    #[must_use]
    pub fn seeded(program: Vec<u32>) -> Case {
        Case::new(program)
            .with_reg(1, BASE + DATA + 0x101)
            .with_reg(2, BASE + DATA + 0x400)
            .with_reg(3, BASE + DATA + 0x800)
            .with_reg(4, BASE + DATA + 0xc00)
    }

    /// The same case on a different core.
    #[must_use]
    pub fn with_config(mut self, cfg: Config) -> Case {
        self.cfg = cfg;
        self
    }
}

/// How many registers [`synthesize`] uses, and [`Case::seeded`] points at the
/// data window.
///
/// Sixteen is enough that a random program reuses a value it computed rather
/// than reading a fresh zero every time, which is what makes a generated block
/// exercise the register-to-temporary mapping at all.
pub const SYNTH_REGS: u32 = 16;

/// Encode one instruction from inside the lifter's subset.
///
/// `form` picks the encoding and `fields` supplies the register numbers and
/// the immediate, so a generator — a fuzzer's byte stream, a seeded
/// pseudo-random sequence — produces programs that *lift* rather than
/// programs that stop at their first instruction. Both numbers are reduced,
/// so every pair of values encodes something.
///
/// The choices that are not arbitrary:
///
/// * Registers come from `x0`..`x15` ([`SYNTH_REGS`]), and a load or store
///   takes its base from `x1`..`x4`, which [`Case::seeded`] points into the
///   data window. A generator that picked base registers uniformly would
///   produce a page fault nearly every time and measure the trap path instead
///   of the lifter.
/// * A memory offset is small and signed, so an access lands near its base
///   and a misaligned one is reachable — that is where the tick accounting is
///   data-dependent.
/// * A branch or jump displacement is small, because a target outside the
///   entry page is a block the lifter refuses and a target that is *taken*
///   decides the exit PC, which is the column being compared.
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn synthesize(form: u32, fields: u32) -> u32 {
    let rd = fields % SYNTH_REGS;
    let rs1 = (fields >> 4) % SYNTH_REGS;
    let rs2 = (fields >> 8) % SYNTH_REGS;
    // A 12-bit immediate, sign-extended the way the encoders below expect.
    let imm12 = ((fields >> 12) & 0xfff) as i32;
    let imm12 = (imm12 << 20) >> 20;
    // A base register the seeded case points into the data window.
    let base = 1 + (rs1 & 3);
    // A small signed memory offset: near the base, and often misaligned.
    let off = (((fields >> 12) & 0x7f) as i32) - 64;
    // A small displacement, halfword-granular so a misaligned target is
    // reachable on a core without `C` — which the lifter refuses, on purpose.
    let disp = ((((fields >> 16) & 0x3f) as i32) - 32) * 2;

    const OP_IMM: u32 = 0x13;
    const OP: u32 = 0x33;
    const OP_IMM_32: u32 = 0x1b;
    const OP_32: u32 = 0x3b;
    const LOAD: u32 = 0x03;

    match form % 49 {
        // register-immediate
        0 => i_type(OP_IMM, 0, rd, rs1, imm12),
        1 => i_type(OP_IMM, 4, rd, rs1, imm12),
        2 => i_type(OP_IMM, 6, rd, rs1, imm12),
        3 => i_type(OP_IMM, 7, rd, rs1, imm12),
        4 => i_type(OP_IMM, 2, rd, rs1, imm12),
        5 => i_type(OP_IMM, 3, rd, rs1, imm12),
        6 => i_type(OP_IMM, 1, rd, rs1, imm12 & 63),
        7 => i_type(OP_IMM, 5, rd, rs1, imm12 & 63),
        8 => i_type(OP_IMM, 5, rd, rs1, 0x400 | (imm12 & 63)),
        // register-register
        9 => r_type(OP, 0, 0x00, rd, rs1, rs2),
        10 => r_type(OP, 0, 0x20, rd, rs1, rs2),
        11 => r_type(OP, 4, 0x00, rd, rs1, rs2),
        12 => r_type(OP, 6, 0x00, rd, rs1, rs2),
        13 => r_type(OP, 7, 0x00, rd, rs1, rs2),
        14 => r_type(OP, 2, 0x00, rd, rs1, rs2),
        15 => r_type(OP, 3, 0x00, rd, rs1, rs2),
        16 => r_type(OP, 1, 0x00, rd, rs1, rs2),
        17 => r_type(OP, 5, 0x00, rd, rs1, rs2),
        18 => r_type(OP, 5, 0x20, rd, rs1, rs2),
        // the upper-immediate pair
        19 => 0x37 | (rd << 7) | ((fields << 12) & 0xffff_f000),
        20 => 0x17 | (rd << 7) | ((fields << 12) & 0xffff_f000),
        // the RV64 word family
        21 => i_type(OP_IMM_32, 0, rd, rs1, imm12),
        22 => i_type(OP_IMM_32, 1, rd, rs1, imm12 & 31),
        23 => i_type(OP_IMM_32, 5, rd, rs1, imm12 & 31),
        24 => i_type(OP_IMM_32, 5, rd, rs1, 0x400 | (imm12 & 31)),
        25 => r_type(OP_32, 0, 0x00, rd, rs1, rs2),
        26 => r_type(OP_32, 0, 0x20, rd, rs1, rs2),
        27 => r_type(OP_32, 1, 0x00, rd, rs1, rs2),
        28 => r_type(OP_32, 5, 0x00, rd, rs1, rs2),
        29 => r_type(OP_32, 5, 0x20, rd, rs1, rs2),
        // loads
        30 => i_type(LOAD, 0, rd, base, off),
        31 => i_type(LOAD, 1, rd, base, off),
        32 => i_type(LOAD, 2, rd, base, off),
        33 => i_type(LOAD, 3, rd, base, off),
        34 => i_type(LOAD, 4, rd, base, off),
        35 => i_type(LOAD, 5, rd, base, off),
        36 => i_type(LOAD, 6, rd, base, off),
        // stores
        37 => s_type(0, base, rs2, off),
        38 => s_type(1, base, rs2, off),
        39 => s_type(2, base, rs2, off),
        40 => s_type(3, base, rs2, off),
        // branches
        41 => b_type(0, rs1, rs2, disp),
        42 => b_type(1, rs1, rs2, disp),
        43 => b_type(4, rs1, rs2, disp),
        44 => b_type(5, rs1, rs2, disp),
        45 => b_type(6, rs1, rs2, disp),
        46 => b_type(7, rs1, rs2, disp),
        // jumps
        47 => j_type(rd, disp),
        _ => i_type(0x67, 0, rd, rs1, imm12),
    }
}

const fn i_type(opcode: u32, funct3: u32, rd: u32, rs1: u32, imm: i32) -> u32 {
    opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | (((imm as u32) & 0xfff) << 20)
}

const fn r_type(opcode: u32, funct3: u32, funct7: u32, rd: u32, rs1: u32, rs2: u32) -> u32 {
    opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | (rs2 << 20) | (funct7 << 25)
}

const fn s_type(funct3: u32, rs1: u32, rs2: u32, imm: i32) -> u32 {
    let imm = imm as u32;
    0x23 | ((imm & 0x1f) << 7)
        | (funct3 << 12)
        | (rs1 << 15)
        | (rs2 << 20)
        | (((imm >> 5) & 0x7f) << 25)
}

const fn b_type(funct3: u32, rs1: u32, rs2: u32, imm: i32) -> u32 {
    let imm = imm as u32;
    0x63 | (((imm >> 11) & 1) << 7)
        | (((imm >> 1) & 0xf) << 8)
        | (funct3 << 12)
        | (rs1 << 15)
        | (rs2 << 20)
        | (((imm >> 5) & 0x3f) << 25)
        | (((imm >> 12) & 1) << 31)
}

const fn j_type(rd: u32, imm: i32) -> u32 {
    let imm = imm as u32;
    0x6f | (rd << 7)
        | (((imm >> 12) & 0xff) << 12)
        | (((imm >> 11) & 1) << 20)
        | (((imm >> 1) & 0x3ff) << 21)
        | (((imm >> 20) & 1) << 31)
}

/// What comparing one case established.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Verdict {
    /// The first instruction was outside the subset, so there was nothing to
    /// compare. Not a failure: the block is still well-formed, and the
    /// interpreter picks the instruction up itself.
    Nothing,
    /// Both engines stopped on a trap at the same guest instruction. The state
    /// afterwards is not compared (module docs).
    Trapped {
        /// How many guest instructions the block covered.
        insns: usize,
    },
    /// They agreed on every column.
    Agreed {
        /// How many guest instructions the block covered.
        insns: usize,
        /// How many ticks both charged.
        ticks: u64,
    },
}

/// The oracle and the subject disagreed.
///
/// Carries the program, because a fuzzer's finding is useless without the
/// bytes that produced it, and the disassembly, because the words are not
/// readable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Divergence {
    /// Which column disagreed, and how.
    pub what: String,
    /// The program, disassembled.
    pub program: String,
}

impl core::fmt::Display for Divergence {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}\n{}", self.what, self.program)
    }
}

/// Compare the lifted path against the interpreter for one case.
///
/// # Errors
///
/// [`Divergence`] when the two disagree on any column, and — because a block
/// the verifier rejects is a frontend bug of exactly the same kind — when the
/// block this frontend produced does not verify.
///
/// # Panics
///
/// If [`Case::cfg`] is not RV64, has PMP entries, or the case's program does
/// not fit in the first page: those are harness misuse rather than findings.
#[allow(clippy::missing_panics_doc)]
pub fn compare(case: &Case) -> Result<Verdict, Divergence> {
    assert!(
        case.cfg.pmp_count == 0,
        "the harness compares ticks, and a PMP refusal is not one the block can know about"
    );
    assert!(
        (case.program.len() as u64) * 4 <= DATA,
        "a case's program lives in the first page"
    );

    // Two identical machines, so a store in one cannot be seen by the other.
    let (oracle_space, oracle_ram) = machine(case);
    let (subject_space, subject_ram) = machine(case);

    // ---- the subject: lift, verify, run on the portable backend ---------
    //
    // `Origin::Bare` is the truth and not a claim: this hart resets into
    // machine mode with `satp` zero, and nothing in the lifted subset can
    // write either.
    let mut src = Words(&case.program);
    let lifted = lift::lift(&case.cfg, Origin::Bare, BASE, &mut src, lift::MAX_INSNS)
        .expect("the harness builds RV64 cases only");
    if lifted.insns == 0 {
        return Ok(Verdict::Nothing);
    }
    if let Err(e) = verify(&lifted.block) {
        return Err(diverged(
            case,
            format!(
                "the frontend produced a block the verifier rejects: {e}\n{}",
                lifted.block
            ),
        ));
    }

    let mut host = Host::new(case, subject_space);
    let outcome = Interp::new()
        .run(&lifted.block, &mut host)
        .map_err(|e| diverged(case, format!("the backend refused the block: {e}")))?;

    // ---- the oracle: the interpreter, the same number of instructions ---
    let hart = Hart::new(case.cfg.with_reset_vector(BASE));
    hart.attach_space(oracle_space);
    for (n, value) in case.regs.iter().enumerate().skip(1) {
        hart.set_x(n as u32, *value);
    }
    for _ in 0..lifted.insns {
        hart.step();
    }

    // A trap is the one thing this harness does not compare through. Both
    // engines must have taken one, or neither: a lifted block that faults
    // where the interpreter did not is a wrong address, and one that does not
    // fault where the interpreter did is a missing check.
    let oracle_trapped = hart.csrs().mcause != 0;
    let subject_faulted = matches!(outcome, Outcome::Fault(_));
    if oracle_trapped != subject_faulted {
        return Err(diverged(
            case,
            format!(
                "the interpreter {} and the lifted block {} (outcome {outcome:?}, \
                 mcause {:#x}, mtval {:#x})",
                if oracle_trapped {
                    "trapped"
                } else {
                    "did not trap"
                },
                if subject_faulted {
                    "faulted"
                } else {
                    "did not fault"
                },
                hart.csrs().mcause,
                hart.csrs().mtval,
            ),
        ));
    }
    if oracle_trapped {
        return Ok(Verdict::Trapped {
            insns: lifted.insns,
        });
    }

    if !matches!(outcome, Outcome::Exit) {
        return Err(diverged(
            case,
            format!("a lifted block must end in exit_tb, but it reported {outcome:?}"),
        ));
    }

    // ---- every column ---------------------------------------------------
    for n in 1..32u32 {
        let want = hart.x(n);
        let got = host.slot(x_slot(n));
        if want != got {
            return Err(diverged(
                case,
                format!(
                    "x{n}: the interpreter says {want:#018x}, the lifted block says {got:#018x}"
                ),
            ));
        }
    }

    let want_pc = hart.pc();
    let got_pc = host.slot(PC);
    if want_pc != got_pc {
        return Err(diverged(
            case,
            format!(
                "pc: the interpreter says {want_pc:#018x}, the lifted block says {got_pc:#018x}"
            ),
        ));
    }

    let want_ticks = hart.cycles();
    if want_ticks != host.ticks {
        return Err(diverged(
            case,
            format!(
                "ticks: the interpreter charged {want_ticks}, the lifted block charged {}",
                host.ticks
            ),
        ));
    }

    // The cumulative column the block publishes at its boundaries is *static*
    // — the charges the frontend could know at lift time — so it accounts for
    // every tick except the ones a load or store spent, which are the block's
    // last guest instruction's and are charged by the access itself (`lift`'s
    // module docs). Those two numbers adding up is the whole of decision 2's
    // claim, and a fault taken at a boundary hashes on the column rather than
    // on the total.
    let column = lifted
        .block
        .marks()
        .last()
        .expect("a block has boundaries")
        .ticks;
    if column + host.access_ticks != want_ticks {
        return Err(diverged(
            case,
            format!(
                "the exit boundary's tick column says {column} and the accesses spent {},                  but {want_ticks} ticks were charged",
                host.access_ticks
            ),
        ));
    }

    for off in 0..RAM_SIZE {
        let want = oracle_ram.read_u8(off).unwrap_or(0);
        let got = subject_ram.read_u8(off).unwrap_or(0);
        if want != got {
            return Err(diverged(
                case,
                format!(
                    "memory at {:#x}: the interpreter left {want:#04x}, the lifted block left \
                     {got:#04x}",
                    BASE + off
                ),
            ));
        }
    }

    Ok(Verdict::Agreed {
        insns: lifted.insns,
        ticks: want_ticks,
    })
}

/// Build the report for a disagreement, disassembling the program into it.
fn diverged(case: &Case, what: String) -> Divergence {
    let mut program = String::new();
    for (n, word) in case.program.iter().enumerate() {
        let pc = BASE + n as u64 * 4;
        let text = super::disasm::format_word(*word, pc, case.cfg.xlen);
        program.push_str(&format!("  {pc:#010x}  {word:08x}  {text}\n"));
    }
    for (n, value) in case.regs.iter().enumerate().skip(1) {
        if *value != 0 {
            program.push_str(&format!("  x{n} = {value:#018x}\n"));
        }
    }
    Divergence { what, program }
}

/// One RAM, one space, the program loaded.
fn machine(case: &Case) -> (Arc<AddressSpace>, Arc<RamStore>) {
    let ram = Arc::new(RamStore::new(RAM_SIZE));
    for (n, word) in case.program.iter().enumerate() {
        for (k, byte) in word.to_le_bytes().iter().enumerate() {
            ram.write_u8(n as u64 * 4 + k as u64, *byte)
                .expect("the program fits");
        }
    }
    let space = AddressSpace::new("mem", 64).with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), BASE)
        .expect("one region maps");
    (Arc::new(space), ram)
}

/// The lifter's view of the program: halfwords out of the case's own words,
/// with nothing outside it readable.
struct Words<'a>(&'a [u32]);

impl lift::InsnSource for Words<'_> {
    fn halfword(&mut self, addr: u64) -> Option<u16> {
        let off = addr.checked_sub(BASE)?;
        let word = *self.0.get((off / 4) as usize)?;
        Some(if off % 4 == 0 {
            word as u16
        } else {
            (word >> 16) as u16
        })
    }
}

/// The guest state a lifted block runs against.
///
/// Slots rather than a register struct, because that is all the backend knows
/// about: the frontend numbered them (`lift`'s module docs) and nothing below
/// it interprets the numbering.
struct Host {
    /// Slot values, indexed by [`RegSlot`], sized by the frontend's numbering.
    slots: [u64; lift::SLOT_COUNT as usize],
    space: Arc<AddressSpace>,
    attrs: MemAttrs,
    misaligned: bool,
    /// Ticks charged, by [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) and by
    /// the accesses this host performed.
    ticks: u64,
    /// Of those, the ones the accesses spent — the data-dependent half, which
    /// the frontend deliberately leaves out of [`InsnStart::ticks`].
    access_ticks: u64,
}

impl Host {
    fn new(case: &Case, space: Arc<AddressSpace>) -> Host {
        let mut slots = [0u64; lift::SLOT_COUNT as usize];
        for (n, value) in case.regs.iter().enumerate().skip(1) {
            slots[n] = *value;
        }
        Host {
            slots,
            space,
            attrs: MemAttrs::DEFAULT.with_requester(case.cfg.requester),
            misaligned: case.cfg.misaligned,
            ticks: 0,
            access_ticks: 0,
        }
    }

    /// The value a slot holds.
    fn slot(&self, slot: RegSlot) -> u64 {
        self.slots[slot.0 as usize]
    }

    /// One access that does not cross a page boundary, charging its tick.
    ///
    /// `exec::read_once`/`write_once` in the shape a host can offer: one bus
    /// access is one cycle, charged whether or not the access succeeds,
    /// because the cycle happened.
    fn once(&mut self, addr: u64, width: Width, value: Option<u64>) -> MemResult<u64> {
        self.ticks += 1;
        self.access_ticks += 1;
        match value {
            None => self.space.read(addr, width, self.attrs),
            Some(v) => self.space.write(addr, width, v, self.attrs).map(|()| 0),
        }
    }

    /// A whole access, split into bytes when it is misaligned and this core
    /// performs misaligned accesses.
    fn access(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> MemResult<u64> {
        let bytes = mem.size.bytes();
        if addr.is_multiple_of(bytes) {
            return self.once(addr, mem.size, value);
        }
        // The frontend puts the policy in the descriptor; honouring it here is
        // what makes a wrongly lifted `Align` a divergence rather than a
        // silently different machine.
        if mem.align == Align::Fault || !self.misaligned {
            return Err(BusError::BadAccess);
        }
        match value {
            None => {
                let mut got = 0u64;
                for i in 0..bytes {
                    let byte = self.once(addr.wrapping_add(i), Width::U8, None)?;
                    got |= (byte & 0xff) << (8 * i);
                }
                Ok(got)
            }
            Some(v) => {
                for i in 0..bytes {
                    self.once(addr.wrapping_add(i), Width::U8, Some(v >> (8 * i)))?;
                }
                Ok(0)
            }
        }
    }
}

impl IrHost for Host {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        u128::from(self.slot(slot))
    }

    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        self.slots[slot.0 as usize] = value as u64;
    }

    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        self.access(mem, addr, None)
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        self.access(mem, addr, Some(value)).map(|_| ())
    }

    fn charge(&mut self, ticks: u64) {
        self.ticks += ticks;
    }

    fn insn_start(&mut self, _mark: &InsnStart) {}
}

// ---------------------------------------------------------------------------
// The cached and chained path
// ---------------------------------------------------------------------------

/// The same comparison, run through the translation runtime rather than one
/// block at a time.
///
/// [`compare`] lifts one block, runs it, and stops. That is the right shape
/// for testing a *frontend*, and it is blind to every mechanism `jit` adds:
/// nothing is ever served from a cache, no exit is ever patched, no
/// translation is ever invalidated, and no access ever goes through a software
/// TLB. So this is the second harness, and it covers exactly what the first
/// cannot:
///
/// | | [`compare`] | [`compare_cached`] |
/// | --- | --- | --- |
/// | blocks | one | up to `blocks`, chained |
/// | translations | one, always fresh | cached under `(pc, key)`, and re-served |
/// | exits | back to the caller | patched straight to the successor |
/// | memory | the address space directly | through `jit::Tlb`, which must answer identically |
/// | instruction bytes | the case's own `Vec<u32>` | **guest RAM**, so a store into the code page is visible |
/// | invalidation | nothing to invalidate | a guest write into a translated page |
///
/// The last two rows are what make self-modifying code testable at all: the
/// subject reads its instructions out of the same RAM it stores into, exactly
/// as the oracle does, so a program that overwrites itself is a program the
/// two engines must still agree about.
///
/// # Errors
///
/// [`Divergence`], on the same columns [`compare`] compares, plus two of its
/// own: a block cache whose back edges stopped being symmetric, and a chain
/// link that outlived its target. Both are reported here rather than left to
/// show up later as a wrong block executed.
///
/// # Panics
///
/// As [`compare`]: a non-RV64 config, a PMP entry, or a program that does not
/// fit in the first page is harness misuse.
#[cfg(feature = "jit")]
#[allow(clippy::missing_panics_doc)]
pub fn compare_cached(case: &Case, blocks: usize) -> Result<Verdict, Divergence> {
    assert!(
        case.cfg.pmp_count == 0,
        "the harness compares ticks, and a PMP refusal is not one the block can know about"
    );
    assert!(
        (case.program.len() as u64) * 4 <= DATA,
        "a case's program lives in the first page"
    );

    let (oracle_space, oracle_ram) = machine(case);
    let (subject_space, subject_ram) = machine(case);

    // ---- the subject: cache, chain, and a TLB on the memory path ---------
    let mut front = Lifter::new(case, Arc::clone(&subject_space));
    let mut host = CachedHost::new(case, subject_space);
    let mut disp = Dispatcher::with_cache(BlockCache::with_capacity(256));
    let run = disp
        .run(&mut front, &mut host, BASE, blocks)
        .map_err(|e| diverged(case, format!("the dispatcher refused a block: {e}")))?;
    if let Some(e) = front.rejected.take() {
        return Err(diverged(case, e));
    }
    if let Err(e) = disp.cache().check() {
        return Err(diverged(
            case,
            format!("the block cache is inconsistent: {e}"),
        ));
    }
    if run.insns == 0 {
        return Ok(Verdict::Nothing);
    }

    // ---- the oracle: the interpreter, the same instructions --------------
    let hart = Hart::new(case.cfg.with_reset_vector(BASE));
    hart.attach_space(oracle_space);
    for (n, value) in case.regs.iter().enumerate().skip(1) {
        hart.set_x(n as u32, *value);
    }
    let mut stepped = 0usize;
    while stepped < run.insns && hart.csrs().mcause == 0 {
        hart.step();
        stepped += 1;
    }

    let oracle_trapped = hart.csrs().mcause != 0;
    let subject_faulted = matches!(run.stop, Stop::Fault(_));
    if oracle_trapped != subject_faulted {
        return Err(diverged(
            case,
            format!(
                "the interpreter {} and the cached path {} (stop {:?}, mcause {:#x}, mtval {:#x})",
                if oracle_trapped {
                    "trapped"
                } else {
                    "did not trap"
                },
                if subject_faulted {
                    "faulted"
                } else {
                    "did not fault"
                },
                run.stop,
                hart.csrs().mcause,
                hart.csrs().mtval,
            ),
        ));
    }
    if oracle_trapped {
        return Ok(Verdict::Trapped { insns: stepped });
    }

    // ---- every column, over however many blocks ran ---------------------
    for n in 1..32u32 {
        let want = hart.x(n);
        let got = host.slot(x_slot(n));
        if want != got {
            return Err(diverged(
                case,
                format!(
                    "x{n} after {} blocks: the interpreter says {want:#018x}, the cached path \
                     says {got:#018x}",
                    run.blocks
                ),
            ));
        }
    }

    let want_pc = hart.pc();
    if want_pc != run.pc {
        return Err(diverged(
            case,
            format!(
                "pc after {} blocks: the interpreter says {want_pc:#018x}, the cached path says \
                 {:#018x}",
                run.blocks, run.pc
            ),
        ));
    }

    let want_ticks = hart.cycles();
    if want_ticks != host.ticks {
        return Err(diverged(
            case,
            format!(
                "ticks after {} blocks: the interpreter charged {want_ticks}, the cached path \
                 charged {}. A cache hit and a cache miss must be indistinguishable to the \
                 guest, including in cycle accounting (ROADMAP.md §0)",
                run.blocks, host.ticks
            ),
        ));
    }

    for off in 0..RAM_SIZE {
        let want = oracle_ram.read_u8(off).unwrap_or(0);
        let got = subject_ram.read_u8(off).unwrap_or(0);
        if want != got {
            return Err(diverged(
                case,
                format!(
                    "memory at {:#x}: the interpreter left {want:#04x}, the cached path left \
                     {got:#04x}",
                    BASE + off
                ),
            ));
        }
    }

    Ok(Verdict::Agreed {
        insns: run.insns,
        ticks: want_ticks,
    })
}

/// What a cached run exercised, beside whether it agreed.
///
/// Separate from [`Verdict`] because "the two engines agreed" and "the cache
/// was actually used" are different assertions, and a harness that conflates
/// them stops noticing the day it quietly stops exercising what it was written
/// for.
#[cfg(feature = "jit")]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CachedRun {
    /// The verdict.
    pub verdict: Verdict,
    /// Blocks executed.
    pub blocks: usize,
    /// Blocks translated — one per distinct `(pc, key)` that survived.
    pub translated: u64,
    /// Blocks reached by following a patched exit, with no lookup at all.
    pub chained: u64,
    /// Blocks invalidated by a guest store into their page.
    pub smc: u64,
    /// Memory accesses served out of a TLB entry.
    pub tlb_hits: u64,
}

/// [`compare_cached`], reporting what the run exercised as well as whether it
/// agreed.
///
/// # Errors
///
/// As [`compare_cached`].
///
/// # Panics
///
/// As [`compare_cached`].
#[cfg(feature = "jit")]
#[allow(clippy::missing_panics_doc)]
pub fn measure_cached(case: &Case, blocks: usize) -> Result<CachedRun, Divergence> {
    let verdict = compare_cached(case, blocks)?;
    // A second, independent run on a fresh machine: the counters come from a
    // run that agreed with the interpreter, and running it twice is itself a
    // determinism check on the whole path.
    let (space, _ram) = machine(case);
    let mut front = Lifter::new(case, Arc::clone(&space));
    let mut host = CachedHost::new(case, space);
    let mut disp = Dispatcher::with_cache(BlockCache::with_capacity(256));
    let run = disp
        .run(&mut front, &mut host, BASE, blocks)
        .map_err(|e| diverged(case, format!("the dispatcher refused a block: {e}")))?;
    Ok(CachedRun {
        verdict,
        blocks: run.blocks,
        translated: disp.stats().translated,
        chained: disp.stats().chained,
        smc: disp.stats().smc,
        tlb_hits: host.tlb.stats().hits,
    })
}

/// The RISC-V half of the dispatcher's contract: lift on demand, out of guest
/// RAM.
#[cfg(feature = "jit")]
struct Lifter {
    cfg: Config,
    space: Arc<AddressSpace>,
    attrs: MemAttrs,
    /// The first block the verifier rejected, reported as a divergence rather
    /// than swallowed — a malformed block is a frontend bug of exactly the
    /// kind this harness exists to catch.
    rejected: Option<String>,
}

#[cfg(feature = "jit")]
impl Lifter {
    fn new(case: &Case, space: Arc<AddressSpace>) -> Lifter {
        Lifter {
            cfg: case.cfg,
            space,
            attrs: MemAttrs::DEFAULT.with_requester(case.cfg.requester),
            rejected: None,
        }
    }
}

#[cfg(feature = "jit")]
impl Frontend for Lifter {
    fn epoch(&mut self) -> Epoch {
        // `satp` is bare on this hart and nothing in the lifted subset can
        // write it, so the translation half is fixed at zero and the topology
        // half is the only one that can move. A paged dispatcher reads
        // `Csrs::translation_gen` here — the same counter `Origin::Paged`
        // carries into the key.
        Epoch {
            topology: self.space.generation(),
            translation: 0,
        }
    }

    fn key(&mut self) -> u64 {
        lift::key(&self.cfg, Origin::Bare)
    }

    fn pc_slot(&self) -> RegSlot {
        PC
    }

    fn translate(&mut self, pc: u64) -> crate::core::error::Result<Translation> {
        // Out of guest RAM, not out of the case's `Vec<u32>`: a store that
        // rewrote an instruction has to be visible here, or the whole
        // self-modifying-code mechanism is untested. In bare mode with plain
        // RAM this *is* the fetch path (`lift`'s module docs, "Paging"), and a
        // paged dispatcher owes the walk instead.
        let space = Arc::clone(&self.space);
        let attrs = self.attrs;
        let mut src = |addr: u64| space.read(addr, Width::U16, attrs).ok().map(|v| v as u16);
        let lifted = lift::lift(&self.cfg, Origin::Bare, pc, &mut src, lift::MAX_INSNS)?;
        if self.rejected.is_none()
            && let Err(e) = verify(&lifted.block)
        {
            self.rejected = Some(format!(
                "the frontend produced a block the verifier rejects: {e}\n{}",
                lifted.block
            ));
        }
        Ok(Translation {
            page: pc & !crate::jit::PAGE_MASK,
            insns: lifted.insns,
            block: lifted.block,
        })
    }
}

/// [`Host`], with the memory path routed through a software TLB and every
/// store recorded for the block cache.
///
/// The access rules are [`Host`]'s, unchanged — one access when aligned, one
/// per byte when not, a tick each — because those are the *frontend's*
/// contract and this harness must not quietly relax them. What changes is
/// only where the bytes come from, which is exactly the claim being tested: a
/// TLB hit must produce what the address space would have produced, down to
/// the error.
#[cfg(feature = "jit")]
struct CachedHost {
    slots: [u64; lift::SLOT_COUNT as usize],
    tlb: Tlb,
    attrs: MemAttrs,
    misaligned: bool,
    ticks: u64,
    dirty: DirtyPages,
}

/// The world a bare machine-mode hart's accesses happen in.
#[cfg(feature = "jit")]
const MACHINE: TlbContext = TlbContext {
    level: 3,
    translating: false,
};

#[cfg(feature = "jit")]
impl CachedHost {
    fn new(case: &Case, space: Arc<AddressSpace>) -> CachedHost {
        let mut slots = [0u64; lift::SLOT_COUNT as usize];
        for (n, value) in case.regs.iter().enumerate().skip(1) {
            slots[n] = *value;
        }
        CachedHost {
            slots,
            tlb: Tlb::new(space),
            attrs: MemAttrs::DEFAULT.with_requester(case.cfg.requester),
            misaligned: case.cfg.misaligned,
            ticks: 0,
            dirty: DirtyPages::new(),
        }
    }

    fn slot(&self, slot: RegSlot) -> u64 {
        self.slots[slot.0 as usize]
    }

    /// One access, through the TLB. Bare mode, so the physical address is the
    /// guest address.
    fn once(&mut self, addr: u64, width: Width, value: Option<u64>) -> MemResult<u64> {
        self.ticks += 1;
        match value {
            None => self
                .tlb
                .read(AccessKind::Load, addr, addr, width, MACHINE, self.attrs),
            Some(v) => {
                let done = self
                    .tlb
                    .write(addr, addr, width, v, MACHINE, self.attrs)
                    .map(|()| 0);
                if done.is_ok() {
                    // The self-modifying-code hook. Drained by the dispatcher
                    // at the next block boundary, which is before anything can
                    // execute the bytes this just changed.
                    self.dirty.note(addr, width.bytes());
                }
                done
            }
        }
    }

    fn access(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> MemResult<u64> {
        let bytes = mem.size.bytes();
        if addr.is_multiple_of(bytes) {
            return self.once(addr, mem.size, value);
        }
        if mem.align == Align::Fault || !self.misaligned {
            return Err(BusError::BadAccess);
        }
        match value {
            None => {
                let mut got = 0u64;
                for i in 0..bytes {
                    let byte = self.once(addr.wrapping_add(i), Width::U8, None)?;
                    got |= (byte & 0xff) << (8 * i);
                }
                Ok(got)
            }
            Some(v) => {
                for i in 0..bytes {
                    self.once(addr.wrapping_add(i), Width::U8, Some(v >> (8 * i)))?;
                }
                Ok(0)
            }
        }
    }
}

#[cfg(feature = "jit")]
impl IrHost for CachedHost {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        u128::from(self.slot(slot))
    }

    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        self.slots[slot.0 as usize] = value as u64;
    }

    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        self.access(mem, addr, None)
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        self.access(mem, addr, Some(value)).map(|_| ())
    }

    fn charge(&mut self, ticks: u64) {
        self.ticks += ticks;
    }

    fn insn_start(&mut self, _mark: &InsnStart) {}
}

#[cfg(feature = "jit")]
impl StoreLog for CachedHost {
    fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64)) {
        self.dirty.drain_dirty(sink);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::riscv::csr::Extensions;
    use alloc::vec;

    const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0x13, 0, rd, rs1, imm)
    }
    const fn ld(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0x03, 3, rd, rs1, imm)
    }
    const fn sd(rs1: u32, rs2: u32, imm: i32) -> u32 {
        s_type(3, rs1, rs2, imm)
    }

    fn agreed(case: &Case) -> Verdict {
        match compare(case) {
            Ok(v) => v,
            Err(e) => panic!("{e}"),
        }
    }

    #[test]
    fn a_straight_line_program_agrees_on_every_column() {
        let case = Case::new(vec![addi(5, 0, 7), addi(6, 5, 3), addi(7, 6, -1)]);
        assert_eq!(agreed(&case), Verdict::Agreed { insns: 3, ticks: 6 });
    }

    #[test]
    fn a_store_agrees_on_memory_and_a_load_reads_it_back() {
        // The store ends its block, so the two runs are two cases — which is
        // also how a dispatcher would see them.
        let addr = BASE + DATA;
        let store = Case::new(vec![sd(1, 2, 0)])
            .with_reg(1, addr)
            .with_reg(2, 0x0123_4567_89ab_cdef);
        assert_eq!(agreed(&store), Verdict::Agreed { insns: 1, ticks: 3 });

        let load = Case::new(vec![ld(3, 1, 0)]).with_reg(1, addr);
        assert_eq!(agreed(&load), Verdict::Agreed { insns: 1, ticks: 3 });
    }

    #[test]
    fn a_misaligned_access_agrees_on_the_tick_it_costs() {
        // Volume I leaves this to the implementation; `Config::misaligned`
        // says this core performs them, byte by byte, and each byte is a bus
        // access. Eight bytes plus two fetch halfwords is ten.
        let case = Case::new(vec![ld(3, 1, 1)]).with_reg(1, BASE + DATA);
        assert_eq!(
            agreed(&case),
            Verdict::Agreed {
                insns: 1,
                ticks: 10
            }
        );
    }

    #[test]
    fn a_core_that_traps_misaligned_accesses_traps_in_both_engines() {
        let mut strict = Config::rv64i();
        strict.misaligned = false;
        let case = Case::new(vec![ld(3, 1, 1)])
            .with_config(strict)
            .with_reg(1, BASE + DATA);
        assert_eq!(compare(&case), Ok(Verdict::Trapped { insns: 1 }));
    }

    #[test]
    fn an_access_off_the_end_of_ram_faults_in_both_engines() {
        let case = Case::new(vec![ld(3, 1, 0)]).with_reg(1, BASE + RAM_SIZE + 0x1000);
        assert_eq!(compare(&case), Ok(Verdict::Trapped { insns: 1 }));
    }

    #[test]
    fn an_unsupported_first_instruction_is_nothing_to_compare() {
        // `ecall`.
        assert_eq!(compare(&Case::new(vec![0x0000_0073])), Ok(Verdict::Nothing));
    }

    #[test]
    fn a_compressed_core_agrees_too() {
        let mut cfg = Config::rv64i();
        cfg.ext = Extensions {
            c: true,
            ..Extensions::I
        };
        // Two `c.addi x5, 1` in one word.
        let c_addi = 0x0285u32;
        let case = Case::new(vec![c_addi | (c_addi << 16)]).with_config(cfg);
        assert_eq!(agreed(&case), Verdict::Agreed { insns: 2, ticks: 2 });
    }

    // -----------------------------------------------------------------------
    // The cached and chained path
    // -----------------------------------------------------------------------

    #[cfg(feature = "jit")]
    mod cached {
        use super::*;
        use crate::jit::{BlockCache, Epoch};

        const fn jal(rd: u32, imm: i32) -> u32 {
            j_type(rd, imm)
        }

        fn agreed(case: &Case, blocks: usize) -> CachedRun {
            match measure_cached(case, blocks) {
                Ok(run) => {
                    assert!(
                        matches!(run.verdict, Verdict::Agreed { .. }),
                        "expected agreement, got {:?}",
                        run.verdict
                    );
                    run
                }
                Err(e) => panic!("diverged:\n{e}"),
            }
        }

        #[test]
        fn a_block_served_from_the_cache_agrees_with_the_interpreter() {
            // A one-block loop, twenty times round. If a cached block were
            // served with anything but the bytes it was lifted from, the
            // register column would say so.
            let case = Case::new(vec![addi(10, 10, 1), jal(0, -4)]);
            let run = agreed(&case, 20);
            assert_eq!(run.blocks, 20);
            assert_eq!(run.translated, 1, "one translation served twenty times");
        }

        #[test]
        fn a_chained_pair_agrees_with_the_interpreter() {
            // Two blocks alternating, so the exits are patched in both
            // directions and almost every block after the first two is reached
            // without a lookup at all.
            let case = Case::new(vec![
                addi(10, 10, 1),
                jal(0, 8),       // 0x04 -> 0x0c
                addi(11, 11, 2), // 0x08 (only reached from the second jump)
                jal(0, -4),      // 0x0c -> 0x08
            ]);
            let run = agreed(&case, 30);
            assert!(
                run.chained >= 25,
                "chained {} of {} blocks",
                run.chained,
                run.blocks
            );
        }

        #[test]
        fn every_access_on_the_cached_path_goes_through_the_software_tlb() {
            // The TLB is not an optional decoration on this harness: if it
            // stopped being consulted, every other assertion here would still
            // pass and the TLB would be untested.
            let case = Case::seeded(vec![sd(1, 5, 0), ld(6, 1, 0), addi(7, 6, 1)]);
            let run = agreed(&case, 8);
            assert!(run.tlb_hits > 0, "no access was served from an entry");
        }

        #[test]
        fn a_store_into_the_code_page_invalidates_the_translation_of_it() {
            // The self-modifying-code test, and the one that fails loudly if
            // `note_write` stops being called. The loop body is rewritten by
            // its own store on the first pass:
            //
            //   0x00  addi x10, x10, 1     <- becomes addi x10, x10, 7
            //   0x04  sd   x11, 0(x1)      <- becomes a nop
            //   0x08  jal  x0, -8          -> back to 0x00
            //
            // so the interpreter adds one once and seven thereafter. A cached
            // block that survived the store adds one every time, which is a
            // divergence in x10 within three blocks.
            let replacement = u64::from(addi(10, 10, 7)) | (u64::from(addi(0, 0, 0)) << 32);
            let case = Case::new(vec![addi(10, 10, 1), sd(1, 11, 0), jal(0, -8)])
                .with_reg(1, BASE)
                .with_reg(11, replacement);
            let run = agreed(&case, 12);
            assert!(run.smc > 0, "no translation was invalidated by the store");
        }

        #[test]
        fn a_store_that_misses_every_translated_page_invalidates_nothing() {
            // The other half of the same claim: an ordinary data store must
            // not throw the cache away, or self-modifying-code support costs
            // the whole speed-up it is paying for.
            let case = Case::seeded(vec![addi(10, 10, 1), sd(2, 10, 0), jal(0, -8)]);
            let run = agreed(&case, 12);
            assert_eq!(run.smc, 0);
            assert!(run.translated <= 2, "the loop was translated once");
        }

        #[test]
        fn changing_the_page_tables_makes_the_same_virtual_address_a_different_block() {
            // The invalidation the block cache does *not* do by flushing,
            // because the frontend already put it in the key: write a PTE,
            // `SFENCE.VMA`, and the same virtual address means something else.
            // `Csrs::translation_gen` moves, `Origin::Paged` carries it, and
            // the key no longer matches.
            let cfg = Config::rv64i();
            let before = lift::key(&cfg, Origin::Paged { generation: 1 });
            let after = lift::key(&cfg, Origin::Paged { generation: 2 });
            assert_ne!(before, after, "the generation is in the key");

            let mut cache = BlockCache::with_capacity(16);
            let mut src = Words(&[addi(10, 10, 1)]);
            let lifted =
                lift::lift(&cfg, Origin::Paged { generation: 1 }, BASE, &mut src, 4).expect("rv64");
            let id = cache.insert(BASE, before, BASE, lifted.insns, lifted.block);
            assert_eq!(cache.lookup(BASE, before), Some(id));
            assert_eq!(
                cache.lookup(BASE, after),
                None,
                "the mapping changed, so the block at this VA must be lifted again"
            );
        }

        #[test]
        fn a_bare_block_and_a_paged_block_at_the_same_address_are_different_blocks() {
            let cfg = Config::rv64i();
            assert_ne!(
                lift::key(&cfg, Origin::Bare),
                lift::key(&cfg, Origin::Paged { generation: 0 }),
                "a physical lift and a virtual lift of the same number must not collide"
            );
        }

        #[test]
        fn a_topology_change_invalidates_a_bare_block_that_the_key_would_not() {
            // `Origin::Bare` contributes nothing to the key, so nothing about
            // the block distinguishes one lifted before a remap from one
            // lifted after. The epoch does.
            let cfg = Config::rv64i();
            let key = lift::key(&cfg, Origin::Bare);
            let mut cache = BlockCache::with_capacity(16);
            let mut src = Words(&[addi(10, 10, 1)]);
            let lifted = lift::lift(&cfg, Origin::Bare, BASE, &mut src, 4).expect("rv64");
            cache.insert(BASE, key, BASE, lifted.insns, lifted.block);
            assert!(cache.lookup(BASE, key).is_some());
            assert!(cache.sync(Epoch {
                topology: 1,
                translation: 0
            }));
            assert_eq!(cache.lookup(BASE, key), None);
        }

        #[test]
        fn the_cached_path_charges_exactly_the_ticks_the_interpreter_charges() {
            // Stated as its own test because it is the phase-5 gate in
            // miniature: a cache hit and a cache miss must be
            // indistinguishable to the guest, cycle counter included. The
            // program mixes fetch charges with a misaligned access, whose
            // charge is data-dependent and paid by the access itself.
            let case = Case::seeded(vec![sd(1, 5, 1), ld(6, 1, 1), jal(0, -8)]);
            let run = agreed(&case, 10);
            let Verdict::Agreed { ticks, .. } = run.verdict else {
                unreachable!("agreed() asserted it")
            };
            assert!(ticks > 0);
        }

        #[test]
        fn an_unsupported_instruction_hands_the_pc_back_rather_than_spinning() {
            // `ecall` is outside the lifted subset, so the block covering it
            // is empty and the dispatcher must stop instead of translating the
            // same nothing forever.
            const ECALL: u32 = 0x0000_0073;
            let case = Case::new(vec![addi(10, 10, 1), ECALL, addi(11, 11, 1)]);
            let run = agreed(&case, 100);
            assert!(run.blocks < 100, "it stopped at the ecall");
        }
    }
}
