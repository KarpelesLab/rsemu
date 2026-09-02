//! The differential harness: the x86 lifter against the x86 interpreter,
//! forever.
//!
//! CLAUDE.md, "CPU cores": *the IR frontend comes later and is differentially
//! tested against the interpreter forever. **The interpreter is the oracle.***
//! This module is that, for [`lift`].
//!
//! # The comparison
//!
//! One program, two machines built the same way, and everything either of them
//! can be seen to do:
//!
//! | | oracle | subject |
//! | --- | --- | --- |
//! | engine | [`X86::step`], `insns` times | [`lift`] → [`verify`] → [`Interp`] |
//! | registers | `EAX`..`EDI` and `EIP` | the slots the block materialized |
//! | **flags** | `EFLAGS`, every bit | the six flag slots plus `EFLAGS_REST`, reassembled |
//! | ticks | `X86::cycles` | the sum of the charges the block made |
//! | the static column | — | [`InsnStart::ticks`] at the exit, plus what the accesses spent |
//! | memory | the RAM it wrote | the RAM it wrote |
//! | faults | whether a trap was taken, where, and in what state | whether the block faulted, where, and in what state |
//!
//! The flags row is the one this harness exists for. Six flags are written by
//! nearly every arithmetic instruction and read by almost none, the lifter
//! elides the ones it can prove unobservable ([`lift::Flags::Elide`]), and
//! dead-code elimination then removes the arithmetic behind them. Every one of
//! those steps is a place to be wrong in a way no register comparison notices,
//! so `EFLAGS` is compared **whole**, at the end of every case and at every
//! fault.
//!
//! # How a trap is detected, and why nothing is delivered
//!
//! The oracle's `IDTR` limit is **zero**, so the first exception cannot read
//! its gate, escalates to `#DF`, fails again, and shuts the processor down —
//! which is what [`X86::is_halted`] then reports. That is deliberate rather
//! than lazy, and it buys three things a working interrupt descriptor table
//! would cost:
//!
//! * **The architectural state at the fault is what is left behind.**
//!   `Exec::deliver` restores its pre-instruction register snapshot before
//!   each retry, and `Exec::step` took that snapshot before decoding — so a
//!   faulting instruction is architecturally as if it had never started, and
//!   `X86::regs` afterwards *is* the state to compare against. The same thing
//!   the IR gives for free through lazy publication (`ir::interp`).
//! * **No ticks are charged for the delivery.** `protected_interrupt` rejects
//!   a vector past the table's limit before it reads anything, so the cycle
//!   counter still holds exactly what the faulting instruction spent — which
//!   is what makes the tick column comparable at a fault rather than only at
//!   an exit.
//! * **No memory is written.** A real gate would push a trap frame onto the
//!   stack, and the byte-for-byte RAM comparison would then have to know
//!   about it.
//!
//! # The machine
//!
//! A 386 in 32-bit protected mode with paging off — the world
//! [`lift::World::of`] accepts, and the one `pc-at` firmware and FreeDOS run
//! in. `CS` is flat, because a computed near transfer would otherwise need a
//! conditional `#GP` the IR cannot express; the five data segments have a base
//! of [`BASE`] and a limit of `RAM_SIZE - 1`, which is what gives this harness
//! a **fault to compare**: an offset past the end of RAM raises `#GP`, or
//! `#SS` through the stack, before anything reaches the bus. Every offset
//! inside the limit lands inside mapped RAM, so no access ever reaches an
//! unassigned address and the interpreter's open-bus path is never a source of
//! disagreement.
//!
//! # Why the host here re-implements the memory path
//!
//! [`IrHost::load`] and [`IrHost::store`] are where a lifted block meets guest
//! memory, and the interpreter's own path through them is private to a step in
//! progress. So this module's host performs the access itself, in the same
//! shape `Exec` does: the segment check `prot::Exec::seg_linear` makes, then
//! one bus transaction charged at the part's [`Variant::bus_clocks`]. Only the
//! *limit* half of that check is implemented, and deliberately: every segment
//! this machine builds is present, readable and writable, so a permission
//! check could never fire — and a check that cannot fire is one nobody would
//! notice going wrong.
//!
//! That is a second implementation of a rule, which is normally the thing to
//! avoid — but it is the *host's* rule rather than the frontend's, and the
//! frontend is what is under test. The lifter's contribution is the
//! [`MemOp`]'s size and its `SegId`, and a wrong one of those diverges here
//! immediately.
//!
//! # Two harnesses, not one
//!
//! [`compare`] runs **one block**, freshly lifted, and stops. That is right
//! for testing a frontend and blind to everything the translation runtime
//! does, so [`compare_cached`] (with the `jit` feature) is the second harness:
//! the same oracle, the same columns, but many blocks through
//! `jit::Dispatcher` — served from a block cache, chained exit to successor,
//! invalidated when the guest writes into a translated page, and with the
//! instruction bytes coming out of **guest RAM** rather than out of
//! [`Case::program`]. That last difference is what makes self-modifying code
//! testable at all, and on x86 it is not optional: the architecture makes a
//! coherent instruction cache a guarantee rather than a courtesy, so a store
//! into a running block's own page has to be honoured before the next
//! instruction — which is exactly what [`lift::Smc::Guard`] emits and what
//! `a_store_into_the_running_blocks_own_page_is_honoured_immediately` checks.
//!
//! # What breaking it deliberately caught
//!
//! A harness nobody has watched fail is a harness that passes. Twenty bugs
//! were injected into [`lift`] one at a time and the suite run
//! against each; **nineteen** were caught, and the twentieth is written up
//! below because it is a finding rather than a gap.
//!
//! Six of them are flags — the auxiliary carry taken from bit 3 instead of
//! bit 4, the parity inverted, `SHL`'s carry read off the result rather than
//! off the bit above the operand's width, `INC` clobbering the carry it must
//! preserve, `AND` leaving the auxiliary carry alone, and a multiply's
//! undefined four taken from the low half of the product. Four are the
//! translation's own machinery: a boundary eliding a flag at an instruction
//! that *can* fault, the exit boundary eliding flags at all, a trace's side
//! exit inverted, and a block reporting its program-order successor rather
//! than the transfer's target. Three are arithmetic: subtract-with-borrow's
//! carry ignoring the borrow in, a 16-bit register write clobbering the upper
//! half, and the effective address widened and never masked. Three are the
//! rules that decide what may be *removed* or *deferred*: a guest load made
//! eliminable, the instruction's own clocks left out of the charge, and the
//! effective address computed after the instruction had already moved a
//! register. And two are self-modifying code: the in-block page guard removed
//! outright, and a `CALL`'s guard resuming after the call rather than at its
//! target.
//!
//! Three of those needed a case the generated corpus does not reach, and each
//! got one: `a_call_that_rewrites_its_own_target_resumes_at_the_target`,
//! `a_compare_against_memory_still_makes_its_bus_cycle` — for which
//! [`synthesize`] also grew three memory-comparison forms, because a load
//! whose only consumer is a flag is the shape that makes [`MemOp`]'s
//! `volatile` load-bearing — and
//! `a_pop_into_a_stack_relative_address_uses_the_stack_pointer_it_started_with`,
//! whose first draft wrote the same zero to both the right address and the
//! wrong one and caught nothing.
//!
//! The one that survived: making a `CL` shift claim it writes its flags
//! **unconditionally**, so the boundary before it elides them. That is a wrong
//! statement about the instruction and it is not an observable one, for a
//! reason worth writing down — a `CL` shift with a memory destination is
//! outside the subset, so such a shift cannot fault, so nothing can look at
//! the flags the boundary dropped. It is asserted by shape instead, in
//! `lift`'s `a_shift_by_cl_elides_nothing_because_it_may_write_nothing`, and
//! it becomes differentially observable the day that form is lifted.
//!
//! # What this harness deliberately does not cover
//!
//! * **What a trap does next.** The state *at* the fault is compared in full;
//!   vectoring is the interpreter's job and a lifted block hands the fault
//!   back rather than delivering it.
//! * **Anything outside the lifted subset**, which ends the block by
//!   construction — so it is not skipped, it simply is not reached.
//! * **Paging, real mode, long mode and the segment loads**, none of which
//!   [`lift::World::of`] accepts.

use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::error::BusError;
use crate::core::space::{AddressSpace, MemAttrs, MemResult, RamStore, Region};
use crate::ir::{InsnStart, Interp, IrHost, MemOp, Outcome, RegSlot, verify};

use super::isa::seg;
use super::lift::{
    self, ARITH_MASK, EFLAGS_REST, EIP, FLAG_BITS, FLAG_SLOTS, Flags, SLOT_COUNT, Shape, Smc,
    World, r_slot,
};
use super::prot::{SegReg, Sys, ar, cr0};
use super::{Config, Regs, Variant, X86, flags};

#[cfg(feature = "jit")]
use crate::core::value::Width;
#[cfg(feature = "jit")]
use crate::ir::AccessKind;
#[cfg(feature = "jit")]
use crate::jit::{
    BlockCache, Context as TlbContext, DirtyPages, Dispatcher, Epoch, Frontend, Stop, StoreLog,
    Tlb, Translation,
};

/// Where a case's RAM is mapped, and where its program starts.
///
/// One megabyte: page-aligned, above everything a PC's low memory would hold,
/// and far enough from zero that a small negative displacement off a seeded
/// pointer does not wrap into nothing.
pub const BASE: u64 = 0x0010_0000;

/// How much RAM a case gets: four pages.
///
/// The first page holds the program — a block is bounded by its page, so it
/// can hold more instructions than [`lift::MAX_INSNS`] will ever read — and
/// the rest is the data window that loads and stores are aimed at. It is also
/// the data segments' limit, which is what makes an access past it a `#GP`.
pub const RAM_SIZE: u64 = 4 * 4096;

/// Where the data window starts, as an offset from [`BASE`].
pub const DATA: u64 = 4096;

/// Where the stack pointer starts, as an offset from [`BASE`].
///
/// In the middle of the data window with room on both sides, so a run of
/// pushes reaches neither the code page below it nor the segment limit above.
pub const STACK: u64 = DATA + 0x800;

/// The selector the flat code segment is loaded from.
const CODE_SEL: u16 = 0x08;
/// The selector the data segments are loaded from.
const DATA_SEL: u16 = 0x10;

/// A 32-bit ring-0 code segment: present, readable, executable, `D` set.
const CODE32: u32 = ar::PRESENT | ar::S | ar::CODE | ar::RW | ar::ACCESSED | ar::DB;
/// A 32-bit ring-0 data segment: present, writable, `B` set.
const DATA32: u32 = ar::PRESENT | ar::S | ar::RW | ar::ACCESSED | ar::DB;

/// One differential case: a program, the state it starts with, and the three
/// lifter policies it is lifted under.
#[derive(Debug, Clone)]
pub struct Case {
    /// Which part. Must be one [`World::of`] accepts.
    pub variant: Variant,
    /// The instruction bytes, loaded at [`BASE`] and entered at `CS:BASE`.
    pub program: Vec<u8>,
    /// The initial `EAX`..`EDI`, in ModRM order. `ESP` is overwritten with
    /// [`STACK`] unless [`Case::keep_esp`] is set, because a random stack
    /// pointer makes every push a fault and measures the trap path instead of
    /// the lifter.
    pub regs: [u32; 8],
    /// Whether [`Case::regs`]'s `ESP` is used as given.
    pub keep_esp: bool,
    /// The initial `EFLAGS`, before normalisation.
    pub eflags: u32,
    /// How much the lifter may swallow into one block.
    pub shape: Shape,
    /// What a store does to the block it is in.
    pub smc: Smc,
    /// Whether a boundary names every flag.
    ///
    /// Every one of these three is a separate frontend to test rather than a
    /// setting: they emit different IR from the same bytes, all of them are in
    /// the cache key, and all of them must agree with the one interpreter.
    pub flags: Flags,
}

impl Case {
    /// A case that runs `program` on a 386 with a zeroed register file.
    #[must_use]
    pub fn new(program: Vec<u8>) -> Case {
        Case {
            variant: Variant::I80386,
            program,
            regs: [0; 8],
            keep_esp: false,
            eflags: flags::ALWAYS_SET,
            shape: Shape::default(),
            smc: Smc::default(),
            flags: Flags::default(),
        }
    }

    /// A case whose `EAX`..`EBX` point into the data window, spread so that a
    /// small signed displacement off any of them stays inside RAM.
    ///
    /// The companion to [`synthesize`], which takes a memory operand's base
    /// from exactly those four registers. `EAX` is deliberately *not* aligned
    /// to four, so a misaligned access is reachable without the displacement
    /// having to supply the misalignment.
    #[must_use]
    pub fn seeded(program: Vec<u8>) -> Case {
        let mut case = Case::new(program);
        case.regs[0] = (DATA + 0x101) as u32;
        case.regs[1] = (DATA + 0x300) as u32;
        case.regs[2] = (DATA + 0x1000) as u32;
        case.regs[3] = (DATA + 0x1800) as u32;
        // Something in every other register, so a generated program reuses a
        // value rather than reading a fresh zero every time.
        case.regs[5] = 0x8000_0001;
        case.regs[6] = 0x0000_ffff;
        case.regs[7] = 0x7fff_ffff;
        case
    }

    /// The same case lifted under `shape`.
    #[must_use]
    pub const fn with_shape(mut self, shape: Shape) -> Case {
        self.shape = shape;
        self
    }

    /// The same case under a different self-modifying-code policy.
    #[must_use]
    pub const fn with_smc(mut self, smc: Smc) -> Case {
        self.smc = smc;
        self
    }

    /// The same case under a different flag policy.
    #[must_use]
    pub const fn with_flags(mut self, policy: Flags) -> Case {
        self.flags = policy;
        self
    }

    /// The same case with a register preset.
    #[must_use]
    pub const fn with_reg(mut self, n: usize, value: u32) -> Case {
        if n < 8 {
            self.regs[n] = value;
        }
        self
    }

    /// The same case starting with `EFLAGS` set to `value`.
    #[must_use]
    pub const fn with_eflags(mut self, value: u32) -> Case {
        self.eflags = value;
        self
    }

    /// The register file this case actually starts with.
    fn start_regs(&self) -> [u32; 8] {
        let mut regs = self.regs;
        if !self.keep_esp {
            regs[4] = STACK as u32;
        }
        regs
    }

    fn start_eflags(&self) -> u32 {
        Regs::normalise_flags(self.variant, self.eflags)
    }
}

/// What comparing one case established.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Verdict {
    /// The first instruction was outside the subset, so there was nothing to
    /// compare. Not a failure: the block is still well-formed, and the
    /// interpreter picks the instruction up itself.
    Nothing,
    /// Both engines stopped on a fault at the same guest instruction, in the
    /// same architectural state.
    Trapped {
        /// How many guest instructions **retired** before the fault. The
        /// faulting one is not among them: it opened its boundary and did not
        /// complete, which is exactly what makes the state comparable.
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
/// bytes that produced it, and the disassembly, because the bytes are not
/// readable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Divergence {
    /// Which column disagreed, and how.
    pub what: String,
    /// The program, disassembled, and the registers it started with.
    pub program: String,
}

impl core::fmt::Display for Divergence {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}\n{}", self.what, self.program)
    }
}

// ---------------------------------------------------------------------------
// The single-block harness
// ---------------------------------------------------------------------------

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
/// If the case's program does not fit in the first page, which is harness
/// misuse rather than a finding.
#[allow(clippy::missing_panics_doc)]
pub fn compare(case: &Case) -> Result<Verdict, Divergence> {
    assert!(
        (case.program.len() as u64) < DATA,
        "a case's program lives in the first page"
    );

    let world = world(case);
    // Two identical machines, so a store in one cannot be seen by the other.
    let (oracle_space, oracle_ram) = machine(case);
    let (subject_space, subject_ram) = machine(case);

    // ---- the subject: lift, verify, run on the portable backend ----------
    let mut src = Bytes {
        program: &case.program,
    };
    let lifted = lift::lift(
        &world,
        BASE,
        &mut src,
        lift::MAX_INSNS,
        case.shape,
        case.smc,
        case.flags,
    )
    .map_err(|e| diverged(case, format!("the frontend refused the case: {e}")))?;
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
    // `Lifted::insns` once a block has side exits.
    let retired = interp.boundaries().saturating_sub(1) as usize;
    let subject_faulted = matches!(outcome, Outcome::Fault(_));

    // ---- the oracle: the interpreter, the same instructions --------------
    let cpu = oracle(case, oracle_space);
    let want = retired + usize::from(subject_faulted);
    let mut stepped = 0usize;
    while stepped < want && !cpu.is_halted() {
        cpu.step();
        stepped += 1;
    }

    let oracle_trapped = cpu.is_halted();
    if oracle_trapped != subject_faulted {
        return Err(diverged(
            case,
            format!(
                "the interpreter {} and the lifted block {} (outcome {outcome:?}, after {stepped} \
                 of {want} steps)",
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
            ),
        ));
    }

    if let Outcome::Fault(fault) = &outcome {
        state(case, &cpu, &host, fault.pc, "the block", true)?;
        memory(case, &oracle_ram, &subject_ram)?;
        return Ok(Verdict::Trapped { insns: retired });
    }

    if !matches!(outcome, Outcome::Exit) {
        return Err(diverged(
            case,
            format!("a lifted block must end in exit_tb, but it reported {outcome:?}"),
        ));
    }

    let pc = host.slot(EIP);
    state(case, &cpu, &host, pc, "the lifted block", false)?;

    // The cumulative column the block publishes at its boundaries is *static*
    // — the charges the frontend could know at lift time — and the accesses
    // spend the rest. Those two adding up is the whole of the IR's decision 2,
    // and a fault taken at a boundary hashes on the column rather than on the
    // total.
    let column = interp
        .mark()
        .and_then(|m| lifted.block.marks().get(m as usize))
        .map_or(0, |m| m.ticks);
    if column + host.access_ticks != host.ticks {
        return Err(diverged(
            case,
            format!(
                "the exit boundary's tick column says {column} and the accesses spent {}, \
                 but {} ticks were charged",
                host.access_ticks, host.ticks
            ),
        ));
    }

    memory(case, &oracle_ram, &subject_ram)?;

    Ok(Verdict::Agreed {
        insns: retired,
        ticks: host.ticks,
    })
}

/// Every architectural column: the eight registers, `EIP`, `EFLAGS` whole, and
/// the cycle counter.
///
/// **This is the hard half of `ROADMAP.md` §9** when `at_fault` holds: *"when
/// a load faults halfway through a translated block, the guest must observe
/// exactly the architectural state its ISA specifies at that instruction — the
/// right PC, the right registers, and nothing from instructions that had not
/// yet retired."* A trace faults with eight registers and six flags living in
/// temporaries and a `EIP` that is a constant in a boundary record rather than
/// anything the block computed, so "the right registers" is a claim about the
/// whole lazy-publication scheme rather than about the load.
fn state(
    case: &Case,
    cpu: &X86,
    host: &Host,
    pc: u64,
    what: &str,
    at_fault: bool,
) -> Result<(), Divergence> {
    let regs = cpu.regs();
    let when = if at_fault { " at the fault" } else { "" };
    for n in 0..8u8 {
        let want = regs.dword(n);
        let got = host.slot(r_slot(n)) as u32;
        if want != got {
            return Err(diverged(
                case,
                format!(
                    "{}{when}: the interpreter says {want:#010x}, {what} says {got:#010x}",
                    REG_NAMES[n as usize]
                ),
            ));
        }
    }

    // At an exit the resume `EIP` is in its slot; at a fault it is not, and
    // deliberately so — `EIP` is bound only at an exit boundary, and the
    // architectural program counter of a faulting instruction is carried by
    // `Fault::pc` instead. A frontend that bound it at every boundary would
    // spend a constant move per guest instruction to say what the boundary
    // record already says.
    let want_pc = regs.rip as u32;
    let got_pc = pc as u32;
    if want_pc != got_pc {
        return Err(diverged(
            case,
            format!("eip{when}: the interpreter says {want_pc:#010x}, {what} says {got_pc:#010x}"),
        ));
    }

    // The flags, whole. Six of them live in their own slots and the rest in
    // one more; a lifter that elided a flag it should have kept, or that
    // published the wrong boundary's map, shows up here and nowhere else.
    let want_flags = regs.eflags;
    let got_flags = host.eflags();
    if want_flags != got_flags {
        let differing = want_flags ^ got_flags;
        return Err(diverged(
            case,
            format!(
                "eflags{when}: the interpreter says {want_flags:#010x}, {what} says \
                 {got_flags:#010x} — differing in {}",
                name_flags(differing)
            ),
        ));
    }

    let want_ticks = cpu.cycles();
    if want_ticks != host.ticks {
        return Err(diverged(
            case,
            format!(
                "ticks{when}: the interpreter charged {want_ticks}, {what} charged {}. A cache \
                 hit and a cache miss must be indistinguishable to the guest, including in cycle \
                 accounting (ROADMAP.md §0)",
                host.ticks
            ),
        ));
    }
    Ok(())
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

/// The names of the flags in a mask, for a report nobody has to decode by
/// hand.
fn name_flags(mask: u32) -> String {
    let mut out = String::new();
    for (bit, name) in [
        (flags::CF, "CF"),
        (flags::PF, "PF"),
        (flags::AF, "AF"),
        (flags::ZF, "ZF"),
        (flags::SF, "SF"),
        (flags::OF, "OF"),
        (flags::TF, "TF"),
        (flags::IF, "IF"),
        (flags::DF, "DF"),
    ] {
        if mask & bit != 0 {
            if !out.is_empty() {
                out.push(' ');
            }
            out.push_str(name);
        }
    }
    if out.is_empty() {
        out.push_str("no named bit");
    }
    out
}

const REG_NAMES: [&str; 8] = ["eax", "ecx", "edx", "ebx", "esp", "ebp", "esi", "edi"];

/// Build the report for a disagreement, disassembling the program into it.
fn diverged(case: &Case, what: String) -> Divergence {
    let mut program = String::new();
    let bytes = &case.program;
    let listing = super::disasm::disassemble_run_as(
        case.variant.map(),
        super::isa::Bits::B32,
        CODE_SEL,
        BASE,
        32,
        |addr| {
            addr.checked_sub(BASE)
                .and_then(|off| usize::try_from(off).ok())
                .and_then(|i| bytes.get(i))
                .copied()
        },
    );
    for line in listing {
        program.push_str(&format!("  {:#010x}  {line}\n", line.ip));
    }
    let regs = case.start_regs();
    for (n, value) in regs.iter().enumerate() {
        if *value != 0 {
            program.push_str(&format!("  {} = {value:#010x}\n", REG_NAMES[n]));
        }
    }
    program.push_str(&format!(
        "  eflags = {:#010x}  shape {:?}  smc {:?}  flags {:?}\n",
        case.start_eflags(),
        case.shape,
        case.smc,
        case.flags
    ));
    Divergence { what, program }
}

/// The configuration the oracle is built from.
#[must_use]
pub fn config(case: &Case) -> Config {
    Config::I8088.with_variant(case.variant)
}

/// The world this case lifts in.
///
/// Written out rather than derived, because the machine below is built to *be*
/// this world; `a_hand_written_world_is_the_one_world_of_finds` asserts that
/// [`World::of`] agrees, which is the property that matters.
///
/// Public so that `benches/x86_dispatch.rs` measures **this** machine rather
/// than one of its own that has drifted from it.
#[must_use]
pub fn world(case: &Case) -> World {
    // Flat code, based data. `CS` is the odd one out and has to be written
    // down as such: the code lives at linear [`BASE`] because `EIP` starts
    // there, not because the segment moves it.
    let mut seg_base = [BASE; seg::COUNT];
    seg_base[usize::from(seg::CS)] = 0;
    World {
        variant: case.variant,
        cs_base: 0,
        seg_base,
        // A 386 and a 486 both have `CMOVcc` clear by default, and `Exec`
        // raises `#UD` for one — so this has to be the core's own answer or the
        // lifter lifts an instruction the interpreter refuses.
        cmov: config(case).features.cmov,
        generation: 0,
    }
}

/// One RAM, one space, the program loaded.
///
/// Public for the same reason [`world`] is: a benchmark that built its own
/// machine would eventually measure a different one.
#[must_use]
pub fn machine(case: &Case) -> (Arc<AddressSpace>, Arc<RamStore>) {
    let ram = Arc::new(RamStore::new(RAM_SIZE));
    for (n, byte) in case.program.iter().enumerate() {
        ram.write_u8(n as u64, *byte).expect("the program fits");
    }
    // A byte the lifter refuses, so a run that falls off the end of the
    // program stops cleanly rather than executing whatever the data window
    // happens to hold.
    ram.write_u8(case.program.len() as u64, 0xf4)
        .expect("the terminator fits");
    let space = AddressSpace::new("mem", 32);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), BASE)
        .expect("one region maps");
    (Arc::new(space), ram)
}

/// A core already in the world [`world`] describes, with the reset sequence
/// discharged and its interrupt table deliberately unusable.
///
/// See the module docs for why the table is unusable rather than absent.
#[must_use]
pub fn oracle(case: &Case, space: Arc<AddressSpace>) -> X86 {
    let cpu = X86::new(config(case));
    cpu.attach_space(space);

    let mut sys = Sys::reset();
    sys.cr0 |= cr0::PE;
    // Zero limit, on purpose: see the module docs. The first exception cannot
    // read its gate, escalates, and shuts the processor down with the
    // architectural state of the faulting instruction still in place.
    sys.idtr.base = 0;
    sys.idtr.limit = 0;
    sys.gdtr.base = 0;
    sys.gdtr.limit = 0;
    sys.segs[usize::from(seg::CS)] = SegReg {
        selector: CODE_SEL,
        base: 0,
        limit: 0xffff_ffff,
        ar: CODE32,
    };
    for index in [seg::DS, seg::ES, seg::SS, seg::FS, seg::GS] {
        sys.segs[usize::from(index)] = SegReg {
            selector: DATA_SEL,
            base: BASE,
            limit: (RAM_SIZE - 1) as u32,
            ar: DATA32,
        };
    }
    cpu.set_sys(sys);

    let mut regs = Regs::new();
    regs.cs = CODE_SEL;
    regs.ss = DATA_SEL;
    regs.ds = DATA_SEL;
    regs.es = DATA_SEL;
    regs.fs = DATA_SEL;
    regs.gs = DATA_SEL;
    let start = case.start_regs();
    for (n, value) in start.iter().enumerate() {
        regs.set_dword(n as u8, *value);
    }
    regs.rip = BASE;
    regs.eflags = case.start_eflags();
    cpu.set_regs(regs);
    cpu.session.lock().state.reset_pending = false;
    cpu
}

/// The lifter's view of the program: bytes out of the case's own vector, with
/// nothing outside it readable.
struct Bytes<'a> {
    program: &'a [u8],
}

impl lift::InsnSource for Bytes<'_> {
    fn byte(&mut self, addr: u64) -> Option<u8> {
        let off = addr.checked_sub(BASE)?;
        self.program.get(usize::try_from(off).ok()?).copied()
    }
}

// ---------------------------------------------------------------------------
// The guest state a lifted block runs against
// ---------------------------------------------------------------------------

/// The segment checks a data access makes, and the bus transaction it costs.
///
/// `Exec::seg_linear` in the shape a host can offer: present, permission,
/// limit, then base — and the checks happen *before* anything is charged,
/// which is why a `#GP` costs nothing and a real access costs one bus cycle.
#[derive(Debug, Clone, Copy)]
struct Segments {
    base: [u64; seg::COUNT],
    limit: [u64; seg::COUNT],
}

impl Segments {
    const fn flat_data() -> Segments {
        Segments {
            base: [BASE; seg::COUNT],
            limit: [RAM_SIZE - 1; seg::COUNT],
        }
    }

    /// The linear address, or the fault a segment check raised.
    fn linear(&self, sr: u8, offset: u64, size: u64) -> MemResult<u64> {
        let sr = usize::from(sr);
        let last = offset.checked_add(size - 1).ok_or(BusError::Protected)?;
        if last > self.limit[sr] {
            // `#GP`, or `#SS` through the stack. The IR carries one bus error
            // and the vector is the interpreter's business; what matters here
            // is that both engines agree there *was* a fault, at the same
            // instruction, in the same state.
            return Err(BusError::Protected);
        }
        Ok(self.base[sr].wrapping_add(offset))
    }
}

/// The guest state a lifted block runs against.
///
/// Slots rather than a register struct, because that is all the backend knows
/// about: the frontend numbered them and nothing below it interprets the
/// numbering.
struct Host {
    slots: [u64; SLOT_COUNT as usize],
    space: Arc<AddressSpace>,
    attrs: MemAttrs,
    segs: Segments,
    bus: u64,
    /// Ticks charged, by `CHARGE` and by the accesses this host performed.
    ticks: u64,
    /// Of those, the ones the accesses spent — the data-dependent half, which
    /// the frontend deliberately leaves out of [`InsnStart::ticks`].
    access_ticks: u64,
}

impl Host {
    fn new(case: &Case, space: Arc<AddressSpace>) -> Host {
        let mut slots = [0u64; SLOT_COUNT as usize];
        let start = case.start_regs();
        for (n, value) in start.iter().enumerate() {
            slots[n] = u64::from(*value);
        }
        slots[EIP.0 as usize] = BASE;
        let eflags = case.start_eflags();
        for (i, bit) in FLAG_BITS.iter().enumerate() {
            slots[FLAG_SLOTS[i].0 as usize] = u64::from(eflags & bit != 0);
        }
        slots[EFLAGS_REST.0 as usize] = u64::from(eflags & !ARITH_MASK);
        Host {
            slots,
            space,
            attrs: MemAttrs::DEFAULT,
            segs: Segments::flat_data(),
            bus: u64::from(case.variant.bus_clocks()),
            ticks: 0,
            access_ticks: 0,
        }
    }

    fn slot(&self, slot: RegSlot) -> u64 {
        self.slots[slot.0 as usize]
    }

    /// The packed flags word, reassembled from the seven slots that hold it.
    fn eflags(&self) -> u32 {
        let mut value = self.slots[EFLAGS_REST.0 as usize] as u32;
        for (i, bit) in FLAG_BITS.iter().enumerate() {
            if self.slots[FLAG_SLOTS[i].0 as usize] & 1 != 0 {
                value |= bit;
            }
        }
        value
    }

    fn charge_bus(&mut self) {
        self.ticks += self.bus;
        self.access_ticks += self.bus;
    }

    fn access(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> MemResult<u64> {
        let sr = mem.seg.map_or(seg::DS, |s| s.0);
        let lin = self.segs.linear(sr, addr, mem.size.bytes())?;
        // Paging is out of the lifted subset, so a whole access is one bus
        // transaction whatever its alignment: only a page crossing splits one,
        // and `Exec::linear_read` only splits when paging is on.
        self.charge_bus();
        match value {
            None => self.space.read(lin, mem.size, self.attrs),
            Some(v) => self.space.write(lin, mem.size, v, self.attrs).map(|()| 0),
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
/// [`compare`] lifts one block, runs it, and stops. That is the right shape for
/// testing a *frontend*, and it is blind to every mechanism `jit` adds. So this
/// is the second harness:
///
/// | | [`compare`] | [`compare_cached`] |
/// | --- | --- | --- |
/// | blocks | one | up to `blocks`, chained |
/// | translations | one, always fresh | cached under `(pc, key)`, and re-served |
/// | exits | back to the caller | patched straight to the successor |
/// | memory | the address space directly | through `jit::Tlb`, which must answer identically |
/// | instruction bytes | the case's own `Vec<u8>` | **guest RAM**, so a store into the code page is visible |
/// | invalidation | nothing to invalidate | a guest write into a translated page |
///
/// The last two rows are what make self-modifying code testable, and on x86
/// that is the whole point: the architecture guarantees a coherent instruction
/// cache, so a store into a running block's own page must be honoured before
/// the next instruction executes.
///
/// # Errors
///
/// [`Divergence`], on the same columns [`compare`] compares, plus one of its
/// own: a block cache whose back edges stopped being symmetric.
///
/// # Panics
///
/// As [`compare`].
#[cfg(feature = "jit")]
#[allow(clippy::missing_panics_doc)]
pub fn compare_cached(case: &Case, blocks: usize) -> Result<Verdict, Divergence> {
    assert!(
        (case.program.len() as u64) < DATA,
        "a case's program lives in the first page"
    );

    let (oracle_space, oracle_ram) = machine(case);
    let (subject_space, subject_ram) = machine(case);

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

    let cpu = oracle(case, oracle_space);
    let subject_faulted = matches!(run.stop, Stop::Fault(_));
    let want = run.insns + usize::from(subject_faulted);
    let mut stepped = 0usize;
    while stepped < want && !cpu.is_halted() {
        cpu.step();
        stepped += 1;
    }

    let oracle_trapped = cpu.is_halted();
    if oracle_trapped != subject_faulted {
        return Err(diverged(
            case,
            format!(
                "the interpreter {} and the cached path {} (stop {:?}, after {stepped} of {want} \
                 steps)",
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
            ),
        ));
    }

    // `Run::pc` is where the dispatcher would resume, read out of the `EIP`
    // slot at an ordinary exit; at a fault it is the faulting instruction's own
    // address, which `Fault::pc` carries instead.
    let pc = match &run.stop {
        Stop::Fault(fault) => fault.pc,
        _ => run.pc,
    };
    let view = HostView {
        slots: host.slots,
        ticks: host.ticks,
        access_ticks: 0,
    };
    state(
        case,
        &cpu,
        &view.as_host(),
        pc,
        if subject_faulted {
            "the cached path at the fault"
        } else {
            "the cached path"
        },
        subject_faulted,
    )?;
    memory(case, &oracle_ram, &subject_ram)?;

    if subject_faulted {
        return Ok(Verdict::Trapped { insns: run.insns });
    }
    Ok(Verdict::Agreed {
        insns: run.insns,
        ticks: host.ticks,
    })
}

/// Enough of a [`Host`] for [`state`] to read, without a second address space.
#[cfg(feature = "jit")]
struct HostView {
    slots: [u64; SLOT_COUNT as usize],
    ticks: u64,
    access_ticks: u64,
}

#[cfg(feature = "jit")]
impl HostView {
    fn as_host(&self) -> Host {
        Host {
            slots: self.slots,
            space: Arc::new(AddressSpace::new("view", 32)),
            attrs: MemAttrs::DEFAULT,
            segs: Segments::flat_data(),
            bus: 0,
            ticks: self.ticks,
            access_ticks: self.access_ticks,
        }
    }
}

/// [`compare_cached`], reporting what the run exercised as well as whether it
/// agreed.
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
    pub insns_retired: usize,
    /// Blocks translated — one per distinct `(pc, key)` that survived.
    pub translated: u64,
    /// Blocks reached by following a patched exit, with no lookup at all.
    pub chained: u64,
    /// Blocks invalidated by a guest store into their page.
    pub smc: u64,
}

/// [`compare_cached`], with the counters that say what it exercised.
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
    })
}

/// The x86 half of the dispatcher's contract: lift on demand, out of guest RAM.
#[cfg(feature = "jit")]
struct Lifter {
    world: World,
    shape: Shape,
    smc: Smc,
    flags: Flags,
    space: Arc<AddressSpace>,
    attrs: MemAttrs,
    /// The first block the verifier rejected, reported as a divergence rather
    /// than swallowed.
    rejected: Option<String>,
}

#[cfg(feature = "jit")]
impl Lifter {
    fn new(case: &Case, space: Arc<AddressSpace>) -> Lifter {
        Lifter {
            world: world(case),
            shape: case.shape,
            smc: case.smc,
            flags: case.flags,
            space,
            attrs: MemAttrs::DEFAULT,
            rejected: None,
        }
    }
}

#[cfg(feature = "jit")]
impl Frontend for Lifter {
    fn epoch(&mut self) -> Epoch {
        // Nothing in the lifted subset can change the world — no segment load,
        // no `CR0` write, no `LGDT` — so the world generation never moves and
        // the topology half is the only one that can. A dispatcher wired to a
        // real machine bumps `World::generation` instead, and that lands in
        // `Block::key` rather than here.
        Epoch {
            topology: self.space.generation(),
            translation: 0,
        }
    }

    fn key(&mut self) -> u64 {
        lift::key(&self.world, self.shape, self.smc, self.flags)
    }

    fn pc_slot(&self) -> RegSlot {
        EIP
    }

    fn translate(&mut self, pc: u64) -> crate::core::error::Result<Translation> {
        // Out of guest RAM, not out of the case's `Vec<u8>`: a store that
        // rewrote an instruction has to be visible here, or the whole
        // self-modifying-code mechanism is untested. With paging off and a
        // flat code segment this *is* the fetch path.
        let space = Arc::clone(&self.space);
        let attrs = self.attrs;
        let mut src = |addr: u64| space.read(addr, Width::U8, attrs).ok().map(|v| v as u8);
        let lifted = lift::lift(
            &self.world,
            pc,
            &mut src,
            lift::MAX_INSNS,
            self.shape,
            self.smc,
            self.flags,
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
            page: lifted.page,
            insns: lifted.insns,
            block: lifted.block,
        })
    }
}

/// [`Host`], with the memory path routed through a software TLB and every store
/// recorded for the block cache.
#[cfg(feature = "jit")]
struct CachedHost {
    slots: [u64; SLOT_COUNT as usize],
    tlb: Tlb,
    attrs: MemAttrs,
    segs: Segments,
    bus: u64,
    ticks: u64,
    dirty: DirtyPages,
}

/// The world a ring-0 access happens in, with paging off.
#[cfg(feature = "jit")]
const RING0: TlbContext = TlbContext {
    level: 0,
    translating: false,
};

#[cfg(feature = "jit")]
impl CachedHost {
    fn new(case: &Case, space: Arc<AddressSpace>) -> CachedHost {
        let seed = Host::new(case, Arc::clone(&space));
        CachedHost {
            slots: seed.slots,
            tlb: Tlb::new(space),
            attrs: MemAttrs::DEFAULT,
            segs: Segments::flat_data(),
            bus: u64::from(case.variant.bus_clocks()),
            ticks: 0,
            dirty: DirtyPages::new(),
        }
    }

    fn access(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> MemResult<u64> {
        let sr = mem.seg.map_or(seg::DS, |s| s.0);
        let lin = self.segs.linear(sr, addr, mem.size.bytes())?;
        self.ticks += self.bus;
        match value {
            None => self
                .tlb
                .read(AccessKind::Load, lin, lin, mem.size, RING0, self.attrs),
            Some(v) => {
                let done = self
                    .tlb
                    .write(lin, lin, mem.size, v, RING0, self.attrs)
                    .map(|()| 0);
                if done.is_ok() {
                    // The self-modifying-code hook. Drained by the dispatcher
                    // at the next block boundary — which the lifter's own
                    // page guard is what *makes* reachable in time, because on
                    // x86 the next instruction may be the one that was
                    // rewritten.
                    self.dirty.note(lin, mem.size.bytes());
                }
                done
            }
        }
    }
}

#[cfg(feature = "jit")]
impl IrHost for CachedHost {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        u128::from(self.slots[slot.0 as usize])
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

// ---------------------------------------------------------------------------
// The generator
// ---------------------------------------------------------------------------

/// How many general registers [`synthesize`] writes to.
///
/// Seven, not eight: `ESP` is left alone, because a random stack pointer makes
/// every push a segment fault and the corpus would measure the trap path
/// instead of the lifter.
pub const SYNTH_REGS: [u8; 7] = [0, 1, 2, 3, 5, 6, 7];

/// Encode one instruction from inside the lifter's subset.
///
/// `form` picks the encoding and `fields` supplies the register numbers, the
/// immediate and the displacement, so a generator — a fuzzer's byte stream, a
/// seeded pseudo-random sequence — produces programs that *lift* rather than
/// programs that stop at their first instruction. Both numbers are reduced, so
/// every pair of values encodes something.
///
/// The choices that are not arbitrary:
///
/// * A memory operand takes its base from `EAX`..`EBX`, which [`Case::seeded`]
///   points into the data window, with an 8-bit signed displacement. A
///   generator that picked base registers uniformly would fault nearly every
///   time.
/// * Nothing writes `ESP`. See [`SYNTH_REGS`].
/// * A branch displacement is small and signed, so a target stays inside the
///   entry page — a target outside it is a block the lifter refuses, which is
///   correct and uninteresting to generate a thousand of.
#[must_use]
#[allow(clippy::too_many_lines)]
pub fn synthesize(form: u32, fields: u32) -> Vec<u8> {
    let reg = SYNTH_REGS[(fields % 7) as usize];
    let rm = SYNTH_REGS[((fields >> 4) % 7) as usize];
    let base = ((fields >> 8) & 3) as u8;
    let disp = (((fields >> 12) & 0x7f) as i32 - 64) as i8;
    let imm8 = (fields >> 16) as u8;
    let imm32 = fields.rotate_left(11);
    let cc = ((fields >> 20) & 15) as u8;
    // Halfword-granular and small, so a taken branch lands on a real
    // instruction boundary more often than not and never leaves the page.
    let rel = ((((fields >> 24) & 0x1f) as i32) - 16) as i8;

    // `mod=11` — both operands are registers.
    let rr = |op: u8, r: u8, m: u8| vec![op, 0xc0 | (r << 3) | m];
    // `mod=01` — a base register and an 8-bit displacement. Base 4 would need
    // a SIB byte and base 5 is `EBP`, so the two are simply not generated.
    let rm8 = |op: u8, r: u8, b: u8, d: i8| vec![op, 0x40 | (r << 3) | b, d as u8];

    match form % 49 {
        // -- the ALU, register to register, at three widths -----------------
        0 => rr(0x01, reg, rm),         // add r/m32, r32
        1 => rr(0x03, reg, rm),         // add r32, r/m32
        2 => rr(0x00, reg & 3, rm & 3), // add r/m8, r8
        3 => rr(0x09, reg, rm),         // or
        4 => rr(0x11, reg, rm),         // adc
        5 => rr(0x19, reg, rm),         // sbb
        6 => rr(0x21, reg, rm),         // and
        7 => rr(0x29, reg, rm),         // sub
        8 => rr(0x31, reg, rm),         // xor
        9 => rr(0x39, reg, rm),         // cmp
        10 => rr(0x85, reg, rm),        // test
        // -- the ALU against memory ----------------------------------------
        11 => rm8(0x01, reg, base, disp),
        12 => rm8(0x03, reg, base, disp),
        13 => rm8(0x29, reg, base, disp),
        14 => rm8(0x33, reg, base, disp),
        15 => rm8(0x89, reg, base, disp),     // mov [base+d], r32
        16 => rm8(0x8b, reg, base, disp),     // mov r32, [base+d]
        17 => rm8(0x88, reg & 3, base, disp), // mov [base+d], r8
        18 => rm8(0x8a, reg & 3, base, disp), // mov r8, [base+d]
        19 => rm8(0x8d, reg, base, disp),     // lea
        // -- immediates -----------------------------------------------------
        20 => {
            // group 81 /n imm32
            let ext = (fields >> 24) as u8 & 7;
            let mut out = vec![0x81, 0xc0 | (ext << 3) | rm];
            out.extend_from_slice(&imm32.to_le_bytes());
            out
        }
        21 => {
            // group 83 /n imm8, sign-extended
            let ext = (fields >> 24) as u8 & 7;
            vec![0x83, 0xc0 | (ext << 3) | rm, imm8]
        }
        22 => {
            // mov r32, imm32
            let mut out = vec![0xb8 | reg];
            out.extend_from_slice(&imm32.to_le_bytes());
            out
        }
        23 => vec![0xb0 | (reg & 3), imm8], // mov r8, imm8
        24 => vec![0x40 | reg],             // inc r32
        25 => vec![0x48 | reg],             // dec r32
        // -- shifts and rotates ---------------------------------------------
        26 => {
            let ext = (fields >> 24) as u8 & 7;
            vec![0xc1, 0xc0 | (ext << 3) | rm, imm8 & 0x1f]
        }
        27 => {
            let ext = (fields >> 24) as u8 & 7;
            vec![0xd1, 0xc0 | (ext << 3) | rm]
        }
        28 => {
            let ext = (fields >> 24) as u8 & 7;
            vec![0xd3, 0xc0 | (ext << 3) | rm]
        }
        29 => {
            // the byte forms, where the flag widths are narrowest
            let ext = (fields >> 24) as u8 & 7;
            vec![0xc0, 0xc0 | (ext << 3) | (rm & 3), imm8 & 0x1f]
        }
        // -- multiplies -----------------------------------------------------
        30 => vec![0xf7, 0xe0 | rm],                    // mul r/m32
        31 => vec![0xf7, 0xe8 | rm],                    // imul r/m32
        32 => vec![0xf6, 0xe0 | (rm & 3)],              // mul r/m8
        33 => vec![0x0f, 0xaf, 0xc0 | (reg << 3) | rm], // imul r32, r/m32
        34 => vec![0x6b, 0xc0 | (reg << 3) | rm, imm8], // imul r32, r/m32, imm8
        // -- the unary group ------------------------------------------------
        35 => vec![0xf7, 0xd0 | rm], // not
        36 => vec![0xf7, 0xd8 | rm], // neg
        // -- extensions and bit scans ---------------------------------------
        37 => vec![0x0f, 0xb6, 0xc0 | (reg << 3) | (rm & 3)], // movzx r32, r8
        38 => vec![0x0f, 0xbe, 0xc0 | (reg << 3) | (rm & 3)], // movsx r32, r8
        39 => vec![0x0f, 0xbc, 0xc0 | (reg << 3) | rm],       // bsf
        40 => vec![0x0f, 0xbd, 0xc0 | (reg << 3) | rm],       // bsr
        // -- the condition codes, read three different ways -----------------
        41 => vec![0x70 | cc, rel as u8],             // jcc rel8
        42 => vec![0x0f, 0x90 | cc, 0xc0 | (rm & 3)], // setcc r/m8
        43 => vec![0x0f, 0x40 | cc, 0xc0 | (reg << 3) | rm], // cmovcc
        // -- the stack, and the flag instructions ---------------------------
        44 => vec![0x50 | reg, 0x58 | rm], // push then pop, so the stack stays put
        // -- a load whose only consumer is the flags -----------------------
        //
        // The shape that makes `MemOp::volatile` load-bearing: nothing keeps
        // the value, so a lifter that marked the load eliminable would let
        // dead-code elimination take the bus cycle and its tick with it — and
        // only once the flags it fed are themselves elided, which is why this
        // needs the generator rather than a hand-written case.
        45 => rm8(0x3b, reg, base, disp), // cmp r32, [base+d]
        46 => rm8(0x39, reg, base, disp), // cmp [base+d], r32
        47 => rm8(0x85, reg, base, disp), // test [base+d], r32
        _ => match (fields >> 28) & 7 {
            0 => vec![0xf8],             // clc
            1 => vec![0xf9],             // stc
            2 => vec![0xf5],             // cmc
            3 => vec![0x9f],             // lahf
            4 => vec![0x9e],             // sahf
            5 => vec![0x98],             // cwde
            6 => vec![0x99],             // cdq
            _ => vec![0x0f, 0xc8 | reg], // bswap
        },
    }
}

/// A whole program of `len` generated instructions, from a seeded generator.
///
/// The generator is a 64-bit linear congruential sequence — Knuth's MMIX
/// multiplier and increment — so the corpus is identical on every machine and
/// in every run (`ROADMAP.md` §0): a failure is reproducible from the seed
/// printed beside it, and a new failure is a real regression rather than a
/// different draw.
#[must_use]
pub fn program(seed: u64, len: usize) -> Vec<u8> {
    let mut state = seed;
    let mut out = Vec::new();
    for _ in 0..len {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        out.extend_from_slice(&synthesize((state >> 40) as u32, state as u32));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The machine this harness builds must be the world the lifter is told it
    /// is in, and [`World::of`] is the only thing that can say so.
    ///
    /// Without this the two could drift and every case would still pass, for
    /// the worst possible reason: the subject and the oracle would agree
    /// because the subject was told what the oracle was doing.
    #[test]
    fn a_hand_written_world_is_the_one_world_of_finds() {
        let case = Case::new(vec![0xf4]);
        let (space, _ram) = machine(&case);
        let cpu = oracle(&case, space);
        let found = World::of(&cpu.regs(), &cpu.sys(), &config(&case), cpu.a20_open(), 0)
            .expect("the harness builds a world the frontend lifts");
        assert_eq!(found, world(&case));
    }

    #[test]
    fn a_handful_of_instructions_agree() {
        // mov eax, 0x12345678 ; add eax, ecx ; sub eax, 1 ; inc ebx ; hlt
        let program = vec![
            0xb8, 0x78, 0x56, 0x34, 0x12, 0x01, 0xc8, 0x83, 0xe8, 0x01, 0x43, 0xf4,
        ];
        let case = Case::seeded(program);
        match compare(&case) {
            Ok(v) => assert!(matches!(v, Verdict::Agreed { insns: 4, .. }), "{v:?}"),
            Err(e) => panic!("{e}"),
        }
    }

    #[test]
    fn the_generator_produces_programs_the_frontend_actually_lifts() {
        // A generator that had stopped producing encodings in the subset would
        // leave every sweep passing and measuring nothing.
        let mut lifted = 0usize;
        for n in 0..200u64 {
            let case = Case::seeded(program(0x1234_0000 + n, 6));
            if let Ok(Verdict::Agreed { insns, .. } | Verdict::Trapped { insns }) = compare(&case) {
                lifted += insns;
            }
        }
        assert!(lifted > 400, "only {lifted} guest instructions were lifted");
    }

    #[test]
    fn the_synthesizer_is_total() {
        // Every pair of numbers has to encode *something*, or a fuzzer's byte
        // stream turns into an empty program rather than a case.
        for form in 0..64u32 {
            for fields in [0u32, 0x1234_5678, u32::MAX, 0x8000_0001] {
                assert!(!synthesize(form, fields).is_empty(), "{form}/{fields:#x}");
            }
        }
    }
}
