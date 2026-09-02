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
//! | registers | `x1`..`x31` and the PC | the slots the block materialized |
//! | ticks | `Hart::cycles` | the sum of the charges the block made |
//! | memory | the RAM it wrote | the RAM it wrote |
//! | faults | whether a trap was taken, where, and in what state | whether the block faulted, where, and in what state |
//!
//! `insns` is what the block **retired**, which is
//! [`Interp::boundaries`](crate::ir::Interp::boundaries) minus one rather than
//! [`Lifted::insns`](super::lift::Lifted::insns): a superblock covers every
//! instruction on the path it inlined and retires only the ones it reached, so
//! the static count would stop the oracle in the wrong place. Every column is
//! compared, because each catches a different class of frontend bug and only
//! the first is obvious: a miscounted
//! [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) is invisible in the registers
//! and fails the phase-5 state-hash gate a million cycles later
//! (`src/ir/mod.rs`, decision 2), a store lifted with the wrong width writes
//! the right register and the wrong memory, and a load whose address is
//! computed wrongly usually faults where the interpreter did not.
//!
//! # Faults are compared through, not around
//!
//! A trap used to end the comparison — both engines had to *stop*, and nothing
//! after that was checked. That was tolerable while a memory access was the
//! last instruction in its block, and it is not once traces exist: a fault in
//! the **middle** of a trace has to reconstruct architectural state from a
//! boundary record, with a dozen guest registers living in temporaries and
//! nothing written back. `precise_state` is that comparison — every integer
//! register, the faulting instruction's PC against `mepc`, and the cycle
//! counter — and it runs on both harnesses.
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
//! * **What a trap does next.** The state *at* the fault is compared in full;
//!   what the guest's own trap handler then does with it is not, because
//!   vectoring into `mtvec` is the interpreter's job and a lifted block hands
//!   the fault back rather than delivering it. Reported as
//!   [`Verdict::Trapped`].
//! * **Code a running trace overwrites.** RISC-V requires a `FENCE.I` between
//!   a store to instruction memory and executing it, so a trace runs to its
//!   end on the bytes it was lifted from while the oracle, being an
//!   interpreter, sees the new ones. That is a legal disagreement rather than
//!   a bug, so the harness's self-modifying-code cases put the store and the
//!   re-execution in different blocks — see
//!   `a_store_into_the_code_page_invalidates_the_translation_of_it`.
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
use crate::ir::{Align, Fault, InsnStart, Interp, IrHost, MemOp, Outcome, RegSlot, verify};

use super::lift::{self, Origin, PC, Shape, x_slot};
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
    /// How much the lifter is allowed to swallow into one block.
    ///
    /// Every shape is a separate frontend to test, not a setting: they emit
    /// different IR from the same bytes — a [`Shape::Trace`] branch is a
    /// [`Opcode::BRCOND`](crate::ir::Opcode::BRCOND) and a side exit where a
    /// [`Shape::BasicBlock`] one is a `setcond`/`movcond` pair — and all of
    /// them must agree with the one interpreter.
    pub shape: Shape,
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
            shape: Shape::default(),
        }
    }

    /// The same case lifted under `shape`.
    #[must_use]
    pub fn with_shape(mut self, shape: Shape) -> Case {
        self.shape = shape;
        self
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
    /// Both engines stopped on a trap at the same guest instruction, in the
    /// same architectural state (`precise_state`).
    Trapped {
        /// How many guest instructions **retired** before the trap.
        ///
        /// The faulting instruction is not one of them: it opened its boundary
        /// and did not complete, which is exactly what makes the state at the
        /// fault comparable.
        insns: usize,
    },
    /// They agreed on every column.
    Agreed {
        /// How many guest instructions the block retired.
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
    let lifted = lift::lift(
        &case.cfg,
        Origin::Bare,
        BASE,
        &mut src,
        lift::MAX_INSNS,
        case.shape,
    )
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
    let mut interp = Interp::new();
    let outcome = interp
        .run(&lifted.block, &mut host)
        .map_err(|e| diverged(case, format!("the backend refused the block: {e}")))?;

    // How many guest instructions actually retired, which is not
    // `Lifted::insns` once a block has side exits: a trace covers every
    // instruction on the path it inlined and retires only the ones it reached.
    let retired = interp.boundaries().saturating_sub(1) as usize;
    let subject_faulted = matches!(outcome, Outcome::Fault(_));

    // ---- the oracle: the interpreter, the same instructions -------------
    let hart = Hart::new(case.cfg.with_reset_vector(BASE));
    hart.attach_space(oracle_space);
    for (n, value) in case.regs.iter().enumerate().skip(1) {
        hart.set_x(n as u32, *value);
    }
    // One more step when the subject faulted: the faulting instruction is the
    // one that did *not* retire, and the oracle has to attempt it to trap on
    // it. Stepping one at a time and stopping at the first trap keeps a trap
    // the subject did not predict from being run past into a trap handler.
    let want = retired + usize::from(subject_faulted);
    let mut stepped = 0usize;
    while stepped < want && hart.csrs().mcause == 0 {
        hart.step();
        stepped += 1;
    }

    let oracle_trapped = hart.csrs().mcause != 0;
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
    if let Outcome::Fault(fault) = &outcome {
        precise_state(case, &hart, fault, &host.slots, host.ticks, "the block")?;
        memory(case, &oracle_ram, &subject_ram)?;
        return Ok(Verdict::Trapped { insns: retired });
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
    let column = interp
        .mark()
        .and_then(|m| lifted.block.marks().get(m as usize))
        .expect("a block that ran reached a boundary")
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

    memory(case, &oracle_ram, &subject_ram)?;

    Ok(Verdict::Agreed {
        insns: retired,
        ticks: want_ticks,
    })
}

/// Compare guest RAM byte for byte.
fn memory(case: &Case, oracle: &RamStore, subject: &RamStore) -> Result<(), Divergence> {
    for off in 0..RAM_SIZE {
        let want = oracle.read_u8(off).unwrap_or(0);
        let got = subject.read_u8(off).unwrap_or(0);
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
    Ok(())
}

/// The architectural state at a fault, against the interpreter's.
///
/// **This is the hard half of `ROADMAP.md` §9**, and the half a superblock
/// makes hard: *"when a load faults halfway through a translated block, the
/// guest must observe exactly the architectural state its ISA specifies at that
/// instruction — the right PC, the right registers, and nothing from
/// instructions that had not yet retired."* A trace faults with a dozen guest
/// registers living in temporaries and a PC that is a constant in a boundary
/// record rather than anything the block computed, so "the right registers" is
/// a claim about the whole lazy-publication scheme
/// (`ir::interp`, "Materializing guest state") rather than about the load.
///
/// Three columns, and each fails differently:
///
/// * **Every integer register.** The interpreter's `x1`..`x31` against the
///   slots the fault materialized. A trace that published the *wrong*
///   boundary's mapping shows up here and nowhere else.
/// * **The PC.** [`Fault::pc`] against `mepc`, which `enter_trap` sets to the
///   faulting instruction's own address. A trace that reported the block's
///   entry PC, or the next instruction's, is caught by this alone.
/// * **The cycle counter.** Every tick charged, against `Hart::cycles`. The
///   faulting access charges for the bus cycles it made before it failed — the
///   interpreter's `read_once` charges and *then* reads — so this is not
///   "ticks up to the boundary", and a subject that reconciled the two would
///   differ here.
fn precise_state(
    case: &Case,
    hart: &Hart,
    fault: &Fault,
    slots: &[u64; lift::SLOT_COUNT as usize],
    ticks: u64,
    what: &str,
) -> Result<(), Divergence> {
    for n in 1..32u32 {
        let want = hart.x(n);
        let got = slots[x_slot(n).0 as usize];
        if want != got {
            return Err(diverged(
                case,
                format!(
                    "x{n} at the fault: the interpreter says {want:#018x}, {what} says \
                     {got:#018x} ({fault:?}, mcause {:#x})",
                    hart.csrs().mcause,
                ),
            ));
        }
    }

    let want_pc = hart.csrs().mepc;
    if want_pc != fault.pc {
        return Err(diverged(
            case,
            format!(
                "the faulting instruction's pc: the interpreter took the trap at \
                 {want_pc:#018x} (mepc), {what} reported {:#018x}",
                fault.pc
            ),
        ));
    }

    let want_ticks = hart.cycles();
    if want_ticks != ticks {
        return Err(diverged(
            case,
            format!(
                "ticks at the fault: the interpreter charged {want_ticks}, {what} charged \
                 {ticks}. A fault mid-block must leave the cycle counter where the \
                 interpreter leaves it, or the state hash differs (ROADMAP.md §0)"
            ),
        ));
    }
    Ok(())
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
    // `Run::insns` is what retired, counted from the boundaries the backend
    // passed rather than summed from the blocks' static instruction counts —
    // a trace that left through a side exit retires fewer than it covers.
    let subject_faulted = matches!(run.stop, Stop::Fault(_));
    let want = run.insns + usize::from(subject_faulted);
    let mut stepped = 0usize;
    while stepped < want && hart.csrs().mcause == 0 {
        hart.step();
        stepped += 1;
    }

    let oracle_trapped = hart.csrs().mcause != 0;
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
    if let Stop::Fault(fault) = &run.stop {
        // The fault-in-the-middle-of-a-trace case, and the reason this harness
        // extends to it: many blocks have run, the faulting one merged across
        // however many branches it could, and the state the fault reports must
        // still be the one instruction's the ISA names.
        precise_state(
            case,
            &hart,
            fault,
            &host.slots,
            host.ticks,
            "the cached path",
        )?;
        memory(case, &oracle_ram, &subject_ram)?;
        return Ok(Verdict::Trapped { insns: run.insns });
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

    memory(case, &oracle_ram, &subject_ram)?;

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
    /// Guest instructions retired across those blocks.
    ///
    /// The number a superblock is *for*: the same block budget retires far
    /// more instructions once direct branches are merged, and this is where
    /// that shows up without a stopwatch.
    pub insns_retired: usize,
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
        insns_retired: run.insns,
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
    shape: Shape,
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
            shape: case.shape,
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
        lift::key(&self.cfg, Origin::Bare, self.shape)
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
        let lifted = lift::lift(
            &self.cfg,
            Origin::Bare,
            pc,
            &mut src,
            lift::MAX_INSNS,
            self.shape,
        )?;
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
    const fn beq(rs1: u32, rs2: u32, imm: i32) -> u32 {
        b_type(0, rs1, rs2, imm)
    }
    const fn jal(rd: u32, imm: i32) -> u32 {
        j_type(rd, imm)
    }
    /// `ecall`: outside the lifted subset, so it ends a block cleanly.
    const ECALL: u32 = 0x0000_0073;

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
        // Nothing retired: the load is the faulting instruction.
        assert_eq!(compare(&case), Ok(Verdict::Trapped { insns: 0 }));
    }

    #[test]
    fn an_access_off_the_end_of_ram_faults_in_both_engines() {
        let case = Case::new(vec![ld(3, 1, 0)]).with_reg(1, BASE + RAM_SIZE + 0x1000);
        assert_eq!(compare(&case), Ok(Verdict::Trapped { insns: 0 }));
    }

    #[test]
    fn a_fault_in_the_middle_of_a_trace_reports_the_interpreters_exact_state() {
        // The test `ROADMAP.md` §9 is really asking for, and the one a
        // superblock makes hard. The trace is ten instructions long and merges
        // across a direct jump; the ninth faults, and by then eight guest
        // registers are living in temporaries that were never written back.
        //
        // Everything `precise_state` compares is checked here by construction:
        // every integer register (eight of them changed since block entry, and
        // `x9` is assigned *after* the faulting load, so a trace that published
        // the wrong boundary reports it early), the faulting instruction's PC
        // against `mepc`, and the cycle counter — including the tick the failed
        // access itself charged.
        let case = Case::new(vec![
            addi(2, 0, 0x11), // 0x00
            addi(3, 2, 0x22), // 0x04
            addi(4, 3, 0x33), // 0x08
            jal(0, 8),        // 0x0c -> 0x14, merged
            addi(31, 0, -1),  // 0x10  never executed
            addi(5, 4, 0x44), // 0x14
            addi(6, 5, 0x55), // 0x18
            addi(7, 6, 0x66), // 0x1c
            addi(8, 7, 0x77), // 0x20
            ld(9, 1, 0),      // 0x24  faults: x1 is off the end of RAM
            addi(10, 0, -1),  // 0x28  never executed
        ])
        .with_reg(1, BASE + RAM_SIZE + 0x4000);

        // Eight instructions retired; the load did not.
        assert_eq!(compare(&case), Ok(Verdict::Trapped { insns: 8 }));

        // and the same program through the whole runtime, where the faulting
        // block is reached after a cache lookup rather than freshly lifted.
        #[cfg(feature = "jit")]
        assert_eq!(compare_cached(&case, 4), Ok(Verdict::Trapped { insns: 8 }));
    }

    #[test]
    fn a_side_exit_that_is_taken_leaves_with_the_registers_the_interpreter_has() {
        // A trace inlines the fall-through of a forward branch and turns the
        // taken side into an exit; this program takes it. The exit's own
        // boundary is then the only thing that carries `x5` out of the block —
        // emptying its live map leaves `x5` at its pre-block value and nothing
        // else notices, which is exactly what this test was added for after a
        // mutation survived every other case in the file.
        let case = Case::new(vec![
            addi(5, 0, 7), // 0x00
            beq(0, 0, 12), // 0x04  always taken -> 0x10, so the exit is taken
            addi(6, 0, 1), // 0x08  inlined but never executed
            addi(7, 0, 2), // 0x0c  likewise
            addi(8, 0, 3), // 0x10  the target; a later block's problem
        ]);
        assert_eq!(compare(&case), Ok(Verdict::Agreed { insns: 2, ticks: 4 }));
    }

    #[test]
    fn a_long_trace_charges_every_instruction_it_merged_in() {
        // Thirty-two iterations of a two-instruction loop merged into one
        // block, and the tick column is compared against the interpreter's
        // cycle counter for all sixty-four. A trace that charged only the
        // instructions before some limit passes every short case in this file.
        let case = Case::new(vec![addi(10, 10, 1), jal(0, -4)]);
        assert_eq!(
            compare(&case),
            Ok(Verdict::Agreed {
                insns: 64,
                ticks: 128
            })
        );
    }

    #[test]
    fn a_fault_after_a_side_exit_was_not_taken_still_reports_exact_state() {
        // The same claim on the other kind of merged boundary: the branch is
        // not taken, so the trace runs *through* the side exit's sequence
        // without entering it, and the fault two instructions later must still
        // name the right registers and PC.
        let case = Case::new(vec![
            addi(2, 0, 5),  // 0x00
            beq(2, 0, 12),  // 0x04  not taken (x2 = 5): side exit skipped
            addi(3, 2, 7),  // 0x08
            ld(4, 1, 0),    // 0x0c  faults
            addi(5, 0, -1), // 0x10  the branch's target, never executed
        ])
        .with_reg(1, BASE + RAM_SIZE + 0x4000);
        assert_eq!(compare(&case), Ok(Verdict::Trapped { insns: 3 }));
    }

    #[test]
    fn every_shape_agrees_with_the_interpreter_about_the_same_program() {
        // The three shapes are three frontends over one oracle. A program with
        // a jump, a branch, a load and a store exercises every place they
        // differ, and they must all reach the same register file — which is
        // what makes the benchmark's attribution honest as well.
        let program = vec![
            addi(5, 0, 3),
            sd(1, 5, 0),
            ld(6, 1, 0),
            beq(6, 5, 8),
            addi(7, 0, -1),
            addi(8, 6, 1),
            ECALL,
        ];
        for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
            let case = Case::seeded(program.clone()).with_shape(shape);
            match compare(&case) {
                Ok(Verdict::Agreed { .. } | Verdict::Trapped { .. }) => {}
                Ok(other) => panic!("{shape:?} produced {other:?}"),
                Err(e) => panic!("{shape:?} diverged:\n{e}"),
            }
        }
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

        const fn jalr(rd: u32, rs1: u32, imm: i32) -> u32 {
            i_type(0x67, 0, rd, rs1, imm)
        }

        /// A core with `C`, which is the only way a `JALR` is in the subset —
        /// and a `JALR` is the only back edge a trace does **not** merge, so
        /// it is how a test gets one block per loop iteration.
        fn indirect_loop() -> Config {
            let mut cfg = Config::rv64i();
            cfg.ext = Extensions {
                c: true,
                ..Extensions::I
            };
            cfg
        }

        #[test]
        fn a_store_into_the_code_page_invalidates_the_translation_of_it() {
            // The self-modifying-code test, and the one that fails loudly if
            // `note_write` stops being called. The loop body is rewritten by
            // its own store on the first pass:
            //
            //   0x00  addi x10, x10, 1     <- becomes addi x10, x10, 7
            //   0x04  sd   x11, 0(x1)      <- becomes a nop
            //   0x08  jalr x0, 0(x12)      -> back to 0x00
            //
            // so the interpreter adds one once and seven thereafter. A cached
            // block that survived the store adds one every time, which is a
            // divergence in x10 within three blocks.
            //
            // The back edge is a `JALR` rather than a `JAL` on purpose. A `JAL`
            // is a direct branch and a trace merges straight through it, and
            // then the running trace executes the bytes it was *lifted* from
            // while the oracle, being an interpreter, executes the new ones —
            // a disagreement RISC-V allows (the guest owes a `FENCE.I`) and
            // this harness has no way to express. An indirect back edge ends
            // the block, so the store and the re-execution are in different
            // translations, which is exactly the case the mechanism is for.
            let replacement = u64::from(addi(10, 10, 7)) | (u64::from(addi(0, 0, 0)) << 32);
            let case = Case::new(vec![addi(10, 10, 1), sd(1, 11, 0), jalr(0, 12, 0)])
                .with_config(indirect_loop())
                .with_reg(1, BASE)
                .with_reg(11, replacement)
                .with_reg(12, BASE);
            let run = agreed(&case, 12);
            assert!(run.smc > 0, "no translation was invalidated by the store");
            assert!(run.translated > 1, "the loop was never lifted again");
        }

        #[test]
        fn a_store_in_the_middle_of_a_trace_invalidates_the_trace() {
            // Invalidation *mid-trace*: the store is the third of five merged
            // instructions and rewrites bytes the trace has already run past,
            // so the running translation is unaffected and the next one is
            // lifted from the new bytes. Both engines must agree throughout —
            // which they can only do because what the store rewrote is behind
            // the store rather than ahead of it.
            //
            //   0x00  addi x10, x10, 1     <- becomes addi x10, x10, 7
            //   0x04  addi x13, x13, 1     <- rewritten with itself
            //   0x08  sd   x11, 0(x1)
            //   0x0c  addi x14, x14, 1
            //   0x10  jalr x0, 0(x12)      -> back to 0x00
            let replacement = u64::from(addi(10, 10, 7)) | (u64::from(addi(13, 13, 1)) << 32);
            let case = Case::new(vec![
                addi(10, 10, 1),
                addi(13, 13, 1),
                sd(1, 11, 0),
                addi(14, 14, 1),
                jalr(0, 12, 0),
            ])
            .with_config(indirect_loop())
            .with_reg(1, BASE)
            .with_reg(11, replacement)
            .with_reg(12, BASE);
            let run = agreed(&case, 10);
            assert!(
                run.smc > 0,
                "the trace was not invalidated by its own store"
            );
            assert!(run.translated > 1, "the trace was never lifted again");
        }

        #[test]
        fn a_store_that_misses_every_translated_page_invalidates_nothing() {
            // The other half of the same claim: an ordinary data store must
            // not throw the cache away, or self-modifying-code support costs
            // the whole speed-up it is paying for.
            let case = Case::seeded(vec![addi(10, 10, 1), sd(2, 10, 0), jal(0, -8)]);
            let run = agreed(&case, 12);
            assert_eq!(run.smc, 0);
            assert!(
                run.translated <= 3,
                "the loop was translated {} times, so the cache is being thrown away",
                run.translated
            );
        }

        #[test]
        fn a_trace_merges_a_loop_into_one_block_and_still_agrees() {
            // The measurement claim, asserted rather than only benchmarked: the
            // same loop is one translation per iteration under the old shape
            // and one translation for thirty-two iterations under a trace,
            // with the same register file at the end of both.
            let program = vec![addi(10, 10, 1), jal(0, -4)];
            let basic = agreed(
                &Case::new(program.clone()).with_shape(Shape::BasicBlock),
                24,
            );
            let trace = agreed(&Case::new(program).with_shape(Shape::Trace), 24);
            assert_eq!(basic.blocks, 24);
            assert_eq!(trace.blocks, 24);
            // Two guest instructions per iteration, so a trace covers
            // thirty-two of them where a basic block covered one.
            assert!(
                trace.insns_retired > basic.insns_retired * 20,
                "a trace retired {} instructions in the same block budget where basic blocks \
                 retired {}",
                trace.insns_retired,
                basic.insns_retired
            );
        }

        #[test]
        fn changing_the_page_tables_makes_the_same_virtual_address_a_different_block() {
            // The invalidation the block cache does *not* do by flushing,
            // because the frontend already put it in the key: write a PTE,
            // `SFENCE.VMA`, and the same virtual address means something else.
            // `Csrs::translation_gen` moves, `Origin::Paged` carries it, and
            // the key no longer matches.
            let cfg = Config::rv64i();
            let before = lift::key(&cfg, Origin::Paged { generation: 1 }, Shape::default());
            let after = lift::key(&cfg, Origin::Paged { generation: 2 }, Shape::default());
            assert_ne!(before, after, "the generation is in the key");

            let mut cache = BlockCache::with_capacity(16);
            let mut src = Words(&[addi(10, 10, 1)]);
            let lifted = lift::lift(
                &cfg,
                Origin::Paged { generation: 1 },
                BASE,
                &mut src,
                4,
                Shape::default(),
            )
            .expect("rv64");
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
                lift::key(&cfg, Origin::Bare, Shape::default()),
                lift::key(&cfg, Origin::Paged { generation: 0 }, Shape::default()),
                "a physical lift and a virtual lift of the same number must not collide"
            );
        }

        #[test]
        fn a_topology_change_invalidates_a_bare_block_that_the_key_would_not() {
            // `Origin::Bare` contributes nothing to the key, so nothing about
            // the block distinguishes one lifted before a remap from one
            // lifted after. The epoch does.
            let cfg = Config::rv64i();
            let key = lift::key(&cfg, Origin::Bare, Shape::default());
            let mut cache = BlockCache::with_capacity(16);
            let mut src = Words(&[addi(10, 10, 1)]);
            let lifted =
                lift::lift(&cfg, Origin::Bare, BASE, &mut src, 4, Shape::default()).expect("rv64");
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
