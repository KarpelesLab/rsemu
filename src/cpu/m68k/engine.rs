//! The translated execution engine: [`lift`](super::lift)'s blocks run on the
//! portable IR backend, with the interpreter underneath everything it declines.
//!
//! `engine = "ir"` on `cpu.m68k` selects it. What it is *not* is a second
//! semantics: `ROADMAP.md` §0 requires a bit-identical state hash across the
//! interpreter and a translated engine for the same guest, so every column a
//! guest or a snapshot can see — registers, `SR`, the program counter, the
//! **prefetch queue**, memory, faults, and the cycle count — comes out the
//! same number. [`differential`](super::differential) is what says so.
//!
//! # The shape, in one paragraph
//!
//! One call to `advance` runs **one block, or one interpreted instruction**.
//! A block is lifted once, cached under its entry PC, and executed by
//! [`Interp`] — the portable backend, which runs anywhere the crate does,
//! `no_std` and both wasm targets included. There is no host code generator
//! here and no block chaining: a backend that lowers these blocks lives in
//! `jit/`, above the `std` line, and chaining needs a successor-linking design
//! this frontend does not have yet.
//!
//! # Four reasons a block does not run, and the interpreter picks it up
//!
//! 1. **The core is not in a liftable state** (`unliftable`): a pending reset,
//!    a halt, `STOP`, an `RTE` replay in flight, **T** set in `SR` (every
//!    instruction would take a trace exception), an odd program counter, a
//!    pending interrupt, or any model but a 68000.
//! 2. **The instruction at the PC is outside the subset**, so the lift
//!    produced a block covering nothing. Recorded, so the next pass does not
//!    lift it again.
//! 3. **A fault.** See below.
//! 4. **The block left part-way through** — [`Outcome::Spent`] — which is not
//!    a fallback at all: the guest is standing at a boundary and the next call
//!    picks up from there.
//!
//! # A fault restarts the instruction on the interpreter
//!
//! `lift`'s module docs have the argument in full. The short form: a 68000's
//! mid-instruction fault is visible in registers the instruction has already
//! changed and in a stack frame whose program counter depends on how far the
//! prefetch got, and the IR cannot publish a write that lands between two
//! boundaries. So `advance` unwinds the partial instruction's charges —
//! which is the reconciliation
//! [`Fault`](crate::ir::Fault) documents for a core that restarts rather than
//! resumes — and hands that one instruction to `Exec::step`, which redoes it
//! and faults where the hardware does.
//!
//! The two numbers to unwind by are **the host's**, not the IR's, and that is
//! worth stating because getting it wrong was a real defect rather than a
//! hypothetical: `Fault::charged_ticks` and `Fault::retired_ticks` are both
//! measured from [`Interp`]'s own counter, which sees
//! [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) immediates and **not** what a
//! host charges inside an access. On a core where every bus cycle is four
//! host-charged ticks that is most of the count. `Finished::retired` has the
//! worked example.
//!
//! The lifted subset is chosen so that this is **exact**: no lifted
//! instruction commits more than one store, and the store is its last memory
//! access, so a fault can only arrive with nothing committed. `Host::store`
//! asserts it in a debug build rather than trusting the frontend.
//!
//! # Self-modifying code
//!
//! A cached block is a translation of bytes, so it is only valid while those
//! bytes are what they were. Two mechanisms, and neither is a heuristic:
//!
//! * **Validation on dispatch.** `Entry::seen` is every `(address, word)`
//!   pair the lifter read, and a cache hit re-reads them. A single mismatch
//!   drops the entry and lifts again. That is O(the block's length) per
//!   dispatch and it is the honest price of having no store log in
//!   `Exec` — a host backend would replace it with one, and
//!   `cpu::riscv::engine`'s `Host::note_writes` is what that looks like.
//! * **Leaving the block.** Validation cannot help a store the *running*
//!   block makes, so `Host::store` notices a store into the running block's
//!   own [`lift::WINDOW`] and `Host::spent` then leaves at the next guest
//!   instruction boundary — the boundary the store's own instruction ends at,
//!   which is where the effect can first be honoured.
//!
//! ## And the third mechanism, which is the one that is easy to miss
//!
//! A 68000 fetches **two words ahead**, so the instruction at a block's entry
//! PC is the word already in `prefetch[0]` — not whatever is at that address
//! now. A store that landed on it since is not seen by the instruction that
//! consumes it, and there is no way back.
//!
//! So `Reader` answers the block's first two words out of `State::prefetch`
//! and only the words from `pc + 4` on out of memory, and a cached entry
//! records the queue it was lifted with so a core whose queue has moved on
//! re-lifts. `docs/cpu/m68k.md` has the generated case that found this; the
//! short version is `OR.B D7,(A3)+` writing into its own code window two
//! bytes ahead of itself, where lifting from memory executed the word the
//! store had just written and the interpreter executed the word it had
//! already fetched.
//!
//! One narrower skew is left and is written down rather than rounded up: an
//! *extension* word further ahead than the queue reaches is baked in as a
//! constant at lift time, so a store onto it between the lift and the run is
//! not seen. Inside a block that cannot happen — the window guard ends the
//! block at the store, and an instruction's own fetches precede its own store
//! — so it needs a second bus master writing the code a block is running.
//!
//! # Counting what the frontend actually carries
//!
//! [`Stats`] and [`Runtime::declines`] are the instrument, and the reason it
//! exists is that the differential sweeps cannot answer the question it
//! answers: a frontend that lifted *nothing* would pass every one of them,
//! because a fallback agrees with the interpreter by construction. So the
//! engine counts, and two closures make the counts a measurement rather than
//! a sample:
//!
//! * **every fallback is attributed to a row.** [`Entry::decline`] is resolved
//!   to a [`DeclineRow`] index when a PC is lifted, and the dispatch path pays
//!   one indexed add for it; the rows sum to [`Stats::interpreted`], so there
//!   is no "other" bucket for the ones nobody counted.
//! * **every block execution is attributed to an outcome.** The five
//!   `ended_*` counters plus [`Stats::spent`] plus [`Stats::faults`] partition
//!   [`Stats::executed`].
//!
//! Both are asserted, in this file's tests and in `tests/m68k_lift_rate.rs`
//! on a running board. What it costs is about **1 %** of the translated
//! engine's time on a Macintosh Plus ROM boot — 4 121 ms against 4 019 ms
//! with the three increments removed, best of six, which is at the noise
//! floor of the measurement. `docs/cpu/m68k.md`, *The lift rate, measured*,
//! has that number and the rates it bought.
//!
//! # Sources
//!
//! `exec.rs` is the oracle for every number here; where a cycle count or an
//! access order is asserted, the citation is in `lift.rs` beside the code that
//! emits it. No emulator source of any licence was opened (`ROADMAP.md` §1).

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::core::error::BusError;
use crate::core::space::{AddressSpace, MemAttrs, MemResult};
use crate::core::value::Width;
use crate::ir::{Align, Block, InsnStart, Interp, IrHost, MemOp, Outcome, RegSlot, verify};

use super::exec::{Exec, State};
use super::isa::Model;
use super::lift::{self, Decline, Declined, SLOT_COUNT, Stop};
use super::{Config, Lines, flags};

/// How many blocks the cache holds before it is emptied.
///
/// A flat bound rather than an eviction policy: a 68000's address space is at
/// most sixteen megabytes and a translation is small, so the interesting
/// question is not which block to throw away but whether the map can grow
/// without bound. It cannot. Clearing the lot is a blunt answer to a rare
/// event, which is the same answer `cpu::riscv::engine`'s `Unlifted::clear`
/// gives for the same reason.
const MAX_BLOCKS: usize = 4096;

/// One cached translation, and the bytes it is a translation *of*.
#[derive(Debug)]
struct Entry {
    /// The block, or `None` when the instruction at this PC is outside the
    /// lifted subset — recorded so the next pass does not lift it again.
    block: Option<Block>,
    /// Every `(address, word)` pair the lifter read **out of memory**, in the
    /// order it read them. Re-read on every hit; see the module docs.
    seen: Vec<(u32, u16)>,
    /// Why lifting stopped where it did.
    ///
    /// Recorded rather than recomputed because it is the answer to "did this
    /// block end at a decline or at a terminator the guest asked for", and
    /// the lift it came from happens once while the block runs many times.
    stop: Stop,
    /// Which row of [`Runtime::declines`] this PC's fallback belongs to, on an
    /// entry that lifted nothing.
    ///
    /// An index rather than the [`Declined`] itself so the dispatch path pays
    /// one indexed add: resolving a `(category, mnemonic)` pair to a row is a
    /// linear scan, and it happens once per lift rather than once per
    /// fallback. `None` on an entry that holds a block — a block's own
    /// [`Stop::Unsupported`] is the *next* PC's decline and is counted there.
    decline: Option<u32>,
    /// The prefetch queue the lift was made with.
    ///
    /// The block's first two words came from here rather than from memory
    /// (`Reader::word`), so a translation is only valid for a core whose queue
    /// still holds them. It normally does — the queue's invariant is that they
    /// *are* the words at `pc` and `pc + 2` — and when it does not, this is
    /// what notices.
    queue: [u16; 2],
}

/// What a translated core has done, and what it holds.
///
/// A statistic and never a behaviour — the two engines are indistinguishable
/// to the guest — so nothing here is snapshotted.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Stats {
    /// Blocks lifted.
    pub lifted: u64,
    /// Blocks executed.
    pub executed: u64,
    /// Guest instructions retired inside a block.
    pub retired: u64,
    /// Instructions the interpreter executed because no block could.
    pub interpreted: u64,
    /// Faults a block took, each handed back to the interpreter.
    pub faults: u64,
    /// Cache entries dropped because the guest rewrote their bytes.
    pub invalidated: u64,
    /// Blocks that left part-way through at a guest instruction boundary.
    pub spent: u64,
    /// Blocks that ran to their terminator having stopped at an encoding
    /// the frontend declined — so the *next* guest instruction is a fallback.
    ///
    /// This and the four rows below it partition the block executions that
    /// reached a terminator, which is [`executed`](Stats::executed) less
    /// [`spent`](Stats::spent) and [`faults`](Stats::faults). The question
    /// they answer is the one a lifted subset with real exclusions has to
    /// answer: how often does a block end because the subset ran out, rather
    /// than because the guest transferred control?
    pub ended_unsupported: u64,
    /// Blocks that ended at a transfer of control — a branch, `DBcc`, `JMP`
    /// or `RTS`. The natural terminator.
    pub ended_transfer: u64,
    /// Blocks that ended at the [`lift::WINDOW`] boundary.
    pub ended_window: u64,
    /// Blocks that ended at [`lift::MAX_INSNS`].
    pub ended_limit: u64,
    /// Blocks that ended because the instruction words could not be read.
    pub ended_unreadable: u64,
    /// Guest instructions retired **in the unit the interpreter counts**:
    /// `Exec::step`s, so an exception sequence is one and an instruction is
    /// one.
    ///
    /// The column [`differential`](super::differential) drives the oracle by,
    /// which is why it is a statistic worth keeping rather than a curiosity:
    /// a translated unit is one *block*, and the harness has to know how many
    /// interpreter steps that block was worth to step the oracle exactly that
    /// far.
    pub steps: u64,
}

/// One row of the decline histogram: how many guest instructions the
/// interpreter took for one `(category, what)` pair.
///
/// Execution-weighted, not static: a `JSR` in a loop is counted every time
/// round it. That is the weighting the question needs — a frontend that
/// declines one encoding a program executes a million times has worse
/// coverage than one that declines a thousand it executes once.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeclineRow {
    /// Which of the five categories.
    pub reason: Decline,
    /// The mnemonic, or a [`Decline::STATE`] cause's label.
    pub what: &'static str,
    /// Guest instructions the interpreter executed for it.
    pub count: u64,
}

/// The [`Decline::STATE`] causes, in the order [`unliftable`] checks them.
///
/// A fixed array rather than a row in [`Runtime::declines`] because this is
/// checked at *every* dispatch: the index is the whole lookup, where a row
/// would be a scan. The labels are the report's, and the order is the check's,
/// so a core that is both stopped and interrupted is counted where the
/// dispatcher actually refused it.
const STATE_CAUSES: [&str; 8] = [
    "not-68000",
    "reset-pending",
    "halted",
    "stopped",
    "rte-replay",
    "trace",
    "odd-pc",
    "interrupt",
];

/// The translation cache, and the backend that runs what is in it.
///
/// Lives in the core's session, behind the same lock the interpreter's state
/// is behind, and is **derived state**: never serialized, thrown away on a
/// reset and whenever the topology generation moves (CLAUDE.md, "Devices").
#[derive(Debug)]
pub(super) struct Runtime {
    entries: BTreeMap<u32, Entry>,
    /// Reused across blocks: a translator that allocates per block pays for it
    /// on every block.
    interp: Interp,
    /// The address-space generation the entries were lifted under.
    generation: u64,
    stats: Stats,
    /// The decline histogram, one row per `(category, what)` pair seen.
    ///
    /// Insertion-ordered and never cleared, so an [`Entry::decline`] index
    /// stays valid across a cache flush. Bounded by five categories times the
    /// mnemonics `isa.rs` has, which is a few hundred rows at the very most
    /// and a handful in practice.
    declines: Vec<DeclineRow>,
    /// Fallbacks by [`STATE_CAUSES`] index.
    states: [u64; STATE_CAUSES.len()],
}

impl Runtime {
    pub(super) fn new() -> Runtime {
        Runtime {
            entries: BTreeMap::new(),
            interp: Interp::new(),
            generation: 0,
            stats: Stats::default(),
            declines: Vec::new(),
            states: [0; STATE_CAUSES.len()],
        }
    }

    /// What this core's translated engine has done.
    pub(super) fn stats(&self) -> Stats {
        self.stats
    }

    /// Every guest instruction the interpreter took, by why a block could not.
    ///
    /// Ordered: the five categories in [`Decline::ALL`] order, and within a
    /// category the order the pairs were first seen. Deterministic, because a
    /// report is ordered output (CLAUDE.md, *Determinism*).
    ///
    /// The rows sum to [`Stats::interpreted`]. That is the property worth
    /// having — a histogram that does not account for every fallback is a
    /// histogram whose largest bucket is "other" — and
    /// `tests/m68k_mini_board.rs` asserts it on a running board.
    pub(super) fn declines(&self) -> Vec<DeclineRow> {
        let mut rows = Vec::with_capacity(self.declines.len() + STATE_CAUSES.len());
        for &reason in Decline::ALL {
            rows.extend(self.declines.iter().copied().filter(|r| r.reason == reason));
            if reason == Decline::STATE {
                for (i, &count) in self.states.iter().enumerate() {
                    if count != 0 {
                        rows.push(DeclineRow {
                            reason: Decline::STATE,
                            what: STATE_CAUSES[i],
                            count,
                        });
                    }
                }
            }
        }
        rows
    }

    /// The row `what` belongs in, appending one if this is the first time.
    ///
    /// A linear scan, and deliberately: it runs once per *lift*, the vector is
    /// tiny, and the alternative — a map keyed on a pair — would put an
    /// allocation and a comparison chain on a path the dispatch loop shares.
    fn row_for(&mut self, declined: Declined) -> u32 {
        let found = self
            .declines
            .iter()
            .position(|r| r.reason == declined.reason && r.what == declined.what);
        let at = match found {
            Some(at) => at,
            None => {
                self.declines.push(DeclineRow {
                    reason: declined.reason,
                    what: declined.what,
                    count: 0,
                });
                self.declines.len() - 1
            }
        };
        // A row index is `u32` in `Entry`, and a saturating conversion is the
        // honest failure: the vector cannot reach four billion rows, and a
        // count landing on row zero would be a wrong number rather than a
        // panic.
        u32::try_from(at).unwrap_or(0)
    }

    /// Throw every translation away.
    ///
    /// The histogram is not thrown away with it: it is a record of what this
    /// core *did*, and an [`Entry::decline`] index into it stays valid because
    /// rows are only ever appended.
    pub(super) fn flush(&mut self) {
        self.entries.clear();
    }
}

/// Why the core is in no state for a lifted block to run — as an index into
/// [`STATE_CAUSES`] — or `None` when it is.
///
/// Each of these is something `Exec::step_inner` does *before* it reaches an
/// instruction, or something the lifted subset cannot express, and a block
/// that ran anyway would skip it.
///
/// It returns the *cause* rather than a boolean because "the core was not
/// liftable" is four different facts about a real guest — a `STOP` loop, an
/// interrupt at every vertical blank, a trace bit, a halt — and a measurement
/// that could not tell them apart could not say whether the frontend's
/// coverage was the frontend's fault.
fn unliftable(state: &State, cfg: &Config, lines: &Lines) -> Option<usize> {
    if cfg.model != Model::M68000 {
        return Some(0);
    }
    if state.reset_pending {
        return Some(1);
    }
    if state.halted {
        return Some(2);
    }
    if state.stopped {
        return Some(3);
    }
    if state.replay.is_some() {
        return Some(4);
    }
    // **T** means every instruction ends in a trace exception, which is
    // exception processing rather than an instruction (MC68000UM §6.2.5).
    if state.sr & flags::T != 0 {
        return Some(5);
    }
    // An odd program counter is an address error on the fetch, and the
    // block's entry word could not have been read at lift time either.
    if state.pc & 1 != 0 {
        return Some(6);
    }
    if lines.interrupt_pending(state.ipl_mask()) {
        return Some(7);
    }
    None
}

/// Advance the core by one *unit of this engine*: one block, or — where a
/// block would be wrong — one interpreted instruction.
///
/// `allowance` is what is left of the caller's tick budget, and it is an
/// allowance rather than advice: `Host::spent` compares it against the ticks
/// charged at every guest instruction boundary but a block's first, and a
/// block that has spent it leaves *there* — at the same instruction an
/// interpreted core would have stopped at, with the same `State::debt`. Pass
/// [`u64::MAX`] to mean "run the whole block".
///
/// Returns the cycles charged, in the same currency and with the same meaning
/// as `Exec::step`, so a run loop cannot tell which engine it is driving.
///
/// # Panics
///
/// If a lifted block reaches an op the IR backend does not implement, or the
/// backend refuses the block. Neither is a guest condition: it is this crate's
/// own frontend emitting something its own backend cannot execute, and the
/// architectural state at that point is not reconstructible — so it is
/// reported loudly rather than papered over, exactly as `cpu::riscv::engine`
/// does.
pub(super) fn advance(
    rt: &mut Runtime,
    state: &mut State,
    space: &AddressSpace,
    cfg: &Config,
    lines: &Lines,
    allowance: u64,
) -> u64 {
    // Derived state, invalidated by the topology generation counter.
    let generation = space.generation();
    if rt.generation != generation {
        rt.generation = generation;
        rt.entries.clear();
    }
    if let Some(cause) = unliftable(state, cfg, lines) {
        rt.stats.interpreted += 1;
        rt.stats.steps += 1;
        rt.states[cause] += 1;
        return Exec::new(state, space, cfg, lines).step();
    }
    let pc = state.pc;
    ensure(rt, pc, state.prefetch, space, cfg);

    let Runtime {
        entries,
        interp,
        stats,
        declines,
        ..
    } = rt;
    let entry = entries.get(&pc);
    let Some((block, stop)) = entry.and_then(|e| e.block.as_ref().map(|b| (b, e.stop))) else {
        stats.interpreted += 1;
        stats.steps += 1;
        // One indexed add, which is what the row index in `Entry` buys: the
        // pair was resolved to a row when this PC was lifted.
        if let Some(row) = entry.and_then(|e| e.decline).map(|r| r as usize)
            && let Some(row) = declines.get_mut(row)
        {
            row.count += 1;
        }
        return Exec::new(state, space, cfg, lines).step();
    };

    stats.executed += 1;
    let mut host = Host::new(state, space, cfg, lines, allowance, pc & !lift::WINDOW_MASK);
    let outcome = interp.run(block, &mut host);
    let retired = interp.boundaries().saturating_sub(1);
    let Finished {
        slots,
        used,
        retired: at_boundary,
    } = host.finish();

    // Every retired instruction, back into the architectural register file.
    state.d.copy_from_slice(&slots[0..8]);
    state.a.copy_from_slice(&slots[8..16]);
    // **Only the condition codes.** No lifted instruction writes **S**, **T**
    // or the interrupt mask, so the stack-pointer bank cannot have moved under
    // the block and `State::set_sr`'s swap is not owed (`lift`'s module docs).
    state.sr = (state.sr & !flags::CCR) | (slots[SR_INDEX] as u16 & flags::CCR);
    state.prefetch = [slots[P0_INDEX] as u16, slots[P1_INDEX] as u16];

    let outcome = match outcome {
        Ok(outcome) => outcome,
        Err(e) => panic!("the m68k frontend produced a block the IR backend refused: {e}"),
    };
    stats.retired += retired;
    stats.steps += retired;

    match outcome {
        Outcome::Fault(fault) => {
            stats.faults += 1;
            // The faulting instruction is the one that did *not* retire, and
            // the interpreter is about to take it — instruction and exception
            // together, which is one `Exec::step` and so one oracle step.
            stats.steps += 1;
            restart(
                state,
                space,
                cfg,
                lines,
                fault.pc as u32,
                used.saturating_sub(at_boundary),
                at_boundary,
            )
        }
        Outcome::Unsupported { op, at } => panic!(
            "the m68k frontend emitted {op} at index {at}, which the IR backend cannot execute"
        ),
        Outcome::Spent { pc } => {
            stats.spent += 1;
            state.pc = pc as u32;
            used.max(1)
        }
        // `Exit` is the only other outcome this frontend produces: it emits no
        // `goto_tb` and no `lookup_and_goto`, because chaining is `jit/`'s.
        // This is the one path that reached the block's terminator, so it is
        // the one that attributes it.
        _ => {
            match stop {
                Stop::Unsupported => stats.ended_unsupported += 1,
                Stop::Transfer => stats.ended_transfer += 1,
                Stop::Window => stats.ended_window += 1,
                Stop::Limit => stats.ended_limit += 1,
                Stop::Unreadable => stats.ended_unreadable += 1,
            }
            state.pc = slots[PC_INDEX];
            used.max(1)
        }
    }
}

/// Unwind the faulting instruction and let the interpreter take it.
///
/// The block stopped *at* the faulting instruction with the state that
/// instruction started with — that is what [`Interp`] publishes from the
/// boundary's live mapping — so the two numbers [`Fault`] reports are all that
/// is left to reconcile: `charged_ticks - retired_ticks` is what the partial
/// instruction spent, and a core that restarts it must give those back or pay
/// for them twice.
fn restart(
    state: &mut State,
    space: &AddressSpace,
    cfg: &Config,
    lines: &Lines,
    at: u32,
    partial: u64,
    retired: u64,
) -> u64 {
    state.cycles = state.cycles.wrapping_sub(partial);
    state.pc = at;
    // The queue as of that boundary is already in `state.prefetch`: the block
    // published it from the boundary's live mapping, or never touched it, and
    // either way it is what this instruction's own fetches left.
    let again = Exec::new(state, space, cfg, lines).step();
    retired.wrapping_add(again).max(1)
}

/// Make sure the cache holds an answer for `pc`, lifting one if it does not.
fn ensure(rt: &mut Runtime, pc: u32, queue: [u16; 2], space: &AddressSpace, cfg: &Config) {
    if let Some(entry) = rt.entries.get(&pc) {
        if entry.queue == queue && valid(entry, space, cfg) {
            return;
        }
        rt.entries.remove(&pc);
        rt.stats.invalidated += 1;
    }
    if rt.entries.len() >= MAX_BLOCKS {
        rt.entries.clear();
    }
    let mut reader = Reader {
        space,
        attrs: MemAttrs::DEBUG.with_requester(cfg.requester),
        mask: cfg.model.address_mask(),
        pc,
        queue,
        seen: Vec::new(),
    };
    // A read-ahead rather than a fetch: this reads up to `MAX_INSNS`
    // instructions the guest has not asked for, which is the one place this
    // core reads guest memory the way a debugger does (CLAUDE.md, "a debugger
    // read must not pop a FIFO"). The block's *own* fetch accesses are
    // performed at run time, through the host, with ordinary attributes.
    let lifted = lift::lift(cfg.model, pc, &mut reader, lift::MAX_INSNS);
    let entry = match lifted {
        Ok(lifted) if lifted.insns > 0 => {
            debug_assert!(
                verify(&lifted.block).is_ok(),
                "the m68k frontend produced a block the verifier rejects: {:?}\n{}",
                verify(&lifted.block),
                lifted.block
            );
            rt.stats.lifted += 1;
            Entry {
                block: Some(lifted.block),
                stop: lifted.stop,
                // A block's own stop is the *next* PC's decline; it is counted
                // there, against the entry that lifts nothing.
                decline: None,
                seen: reader.seen,
                queue,
            }
        }
        // Nothing lifted, or a model this frontend refuses. Either way the
        // answer is recorded so the next pass costs a map lookup rather than a
        // decode and an allocation — and so does the reason, which is what
        // makes the fallback attributable.
        Ok(lifted) => {
            let decline = match lifted.declined {
                Some(declined) => Some(rt.row_for(declined)),
                // `Stop::Unreadable` on the very first word: the PC points at
                // memory that answers nothing, so the interpreter's own fetch
                // is what takes the bus error. Not about the encoding, so it
                // goes under `STATE` — and it takes a *row* rather than a
                // fixed slot because the entry is cached, and every later
                // dispatch at this PC is another fallback to attribute.
                None => Some(rt.row_for(Declined {
                    reason: Decline::STATE,
                    what: "unreadable",
                })),
            };
            Entry {
                block: None,
                stop: lifted.stop,
                decline,
                seen: reader.seen,
                queue,
            }
        }
        // A model this frontend refuses outright. `unliftable` has already
        // caught it at dispatch, so this is unreachable from `advance` and is
        // attributed rather than left blank in case another caller arrives.
        Err(_) => {
            let row = rt.row_for(Declined {
                reason: Decline::MODEL,
                what: "not-68000",
            });
            Entry {
                block: None,
                stop: Stop::Unsupported,
                decline: Some(row),
                seen: reader.seen,
                queue,
            }
        }
    };
    rt.entries.insert(pc, entry);
}

/// Whether the bytes an entry was built from are still the bytes in memory.
fn valid(entry: &Entry, space: &AddressSpace, cfg: &Config) -> bool {
    let attrs = MemAttrs::DEBUG.with_requester(cfg.requester);
    let mask = cfg.model.address_mask();
    entry.seen.iter().all(|&(addr, word)| {
        let at = u64::from(addr & mask);
        space
            .read(at, Width::U16, attrs)
            .is_ok_and(|v| v as u16 == word)
    })
}

/// The lifter's window onto guest memory, recording what it hands out.
struct Reader<'a> {
    space: &'a AddressSpace,
    attrs: MemAttrs,
    mask: u32,
    /// The block's entry PC, which is where the prefetch queue answers.
    pc: u32,
    /// `State::prefetch` as the core holds it.
    queue: [u16; 2],
    seen: Vec<(u32, u16)>,
}

impl lift::InsnSource for Reader<'_> {
    fn word(&mut self, addr: u32) -> Option<u16> {
        // An odd instruction address is an address error rather than a word,
        // and the lifter refuses the block; reading one here would silently
        // give it bytes the processor never fetches.
        if addr & 1 != 0 {
            return None;
        }
        // **The first two words come out of the prefetch queue, not memory.**
        //
        // This is not an optimization and it is not belt and braces: it is the
        // difference between executing the guest's instruction and executing
        // the bytes that happen to be at its address. `prefetch[0]` is the word
        // at `pc` *as fetched* and `prefetch[1]` the word at `pc + 2`, and a
        // store that landed on either of them since is not seen by the
        // instruction that consumes them — a 68000 fetches two words ahead and
        // there is no way back.
        //
        // A generated case found this: `OR.B D7,(A3)+` writing into its own
        // code window two bytes ahead of itself, so the interpreter executed
        // the word it had already fetched and a block lifted from memory
        // executed the word the store had just written. Two cycles apart, and
        // an entirely different instruction.
        //
        // Everything from `pc + 4` on has *not* been fetched yet, so memory is
        // the right source for it, and only those reads go in `seen` — the two
        // queue words are validated against the queue instead (`valid`).
        if addr == self.pc {
            return Some(self.queue[0]);
        }
        if addr == self.pc.wrapping_add(2) {
            return Some(self.queue[1]);
        }
        let at = u64::from(addr & self.mask);
        let word = self.space.read(at, Width::U16, self.attrs).ok()? as u16;
        self.seen.push((addr, word));
        Some(word)
    }
}

// The slot numbering, as indices into the host's flat array. Named here rather
// than cast at each use so a slot added to `lift` cannot silently shift one.
const SR_INDEX: usize = 16;
const PC_INDEX: usize = 17;
const P0_INDEX: usize = 18;
const P1_INDEX: usize = 19;

/// What a run left behind.
struct Finished {
    slots: [u32; SLOT_COUNT as usize],
    /// Every tick the run charged.
    used: u64,
    /// The ticks charged when the last boundary was reached — so
    /// `used - retired` is exactly what a partial instruction spent.
    ///
    /// Tracked here rather than read off [`Fault`](crate::ir::Fault), and that
    /// is the fix for a real defect rather than a preference.
    /// `Fault::charged_ticks` and `Fault::retired_ticks` are both measured
    /// from [`Interp`]'s own counter, which sees
    /// [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) immediates and **not**
    /// the ticks a host charges inside an access. On a core whose every bus
    /// cycle is four host-charged ticks that is most of the count: the first
    /// version of `restart` unwound a faulting `ADD.W (d8,An,Xn),Dn` by the
    /// two internal cycles its `CHARGE` carried and left the eight its two
    /// bus cycles had spent, so the guest's cycle counter ran eight ahead of
    /// the interpreter's for the rest of the run.
    retired: u64,
}

/// The [`IrHost`] a lifted 68000 block runs against.
///
/// It is the whole of the semantics the IR does not carry: the guest register
/// file as a flat slot array, the address space, and the four-cycle charge
/// every bus access makes.
struct Host<'a> {
    state: &'a mut State,
    space: &'a AddressSpace,
    lines: &'a Lines,
    attrs: MemAttrs,
    /// The address pins this model has. A 68000 drives twenty-four, and the
    /// wrap is observable — `exec::bus_addr` masks here and so does this.
    mask: u32,
    slots: [u32; SLOT_COUNT as usize],
    allowance: u64,
    used: u64,
    /// Ticks charged when the last guest instruction boundary was reached.
    retired: u64,
    /// Stores this guest instruction has committed. The restartability budget
    /// is one (module docs).
    stores: u32,
    /// A store landed in the running block's own window, so the block must
    /// leave at the next guest instruction boundary.
    smc: bool,
    /// The interrupt mask the block started with. **S**, **T** and the
    /// interrupt mask cannot change under a lifted block, so this is constant
    /// for the run and is read once rather than out of a slot the block may
    /// have rebound.
    ipl_mask: u8,
    code_window: u32,
}

impl<'a> Host<'a> {
    fn new(
        state: &'a mut State,
        space: &'a AddressSpace,
        cfg: &Config,
        lines: &'a Lines,
        allowance: u64,
        code_window: u32,
    ) -> Host<'a> {
        let mut slots = [0u32; SLOT_COUNT as usize];
        slots[0..8].copy_from_slice(&state.d);
        slots[8..16].copy_from_slice(&state.a);
        slots[SR_INDEX] = u32::from(state.sr);
        slots[PC_INDEX] = state.pc;
        slots[P0_INDEX] = u32::from(state.prefetch[0]);
        slots[P1_INDEX] = u32::from(state.prefetch[1]);
        let ipl_mask = state.ipl_mask();
        // Every access a lifted block makes carries the privilege the core was
        // in when the block started, which is what `Exec::attrs` carries —
        // and it cannot change, because `MOVE to SR` is not lifted.
        let attrs = MemAttrs::DEFAULT
            .with_requester(cfg.requester)
            .with_privileged(state.supervisor());
        Host {
            state,
            space,
            lines,
            attrs,
            mask: cfg.model.address_mask(),
            slots,
            allowance,
            used: 0,
            retired: 0,
            stores: 0,
            smc: false,
            ipl_mask,
            code_window,
        }
    }

    fn finish(self) -> Finished {
        Finished {
            slots: self.slots,
            used: self.used,
            retired: self.retired,
        }
    }

    /// One bus access: four clocks, charged where `Exec::read_byte` and
    /// friends charge them — *after* the alignment check and *before* the
    /// access, so an address error costs nothing and a refused access costs
    /// four.
    fn bus_cycle(&mut self) {
        self.used = self.used.wrapping_add(4);
        self.state.cycles = self.state.cycles.wrapping_add(4);
    }

    /// Whether `addr` is an address error for this access.
    ///
    /// "An odd address is an address error on a 68000 and a 68010"
    /// (`exec::read_word_fc`), for a word or long operand and for every
    /// instruction fetch. A byte access has no alignment rule, which the
    /// frontend says by giving it [`Align::None`].
    const fn misaligned(mem: &MemOp, addr: u32) -> bool {
        matches!(mem.align, Align::Fault) && addr & 1 != 0
    }
}

impl IrHost for Host<'_> {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        u128::from(self.slots.get(slot.0 as usize).copied().unwrap_or(0))
    }

    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        if let Some(s) = self.slots.get_mut(slot.0 as usize) {
            *s = value as u32;
        }
    }

    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        // Computed in the guest's width by the block, masked to the pins here:
        // on a 68000 the wrap at twenty-four bits is observable.
        let addr = addr as u32;
        if Self::misaligned(mem, addr) {
            return Err(BusError::BadAccess);
        }
        self.bus_cycle();
        let at = u64::from(addr & self.mask);
        self.space.read(at, mem.size, self.attrs)
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        let addr = addr as u32;
        if Self::misaligned(mem, addr) {
            return Err(BusError::BadAccess);
        }
        self.stores += 1;
        debug_assert!(
            self.stores <= 1,
            "the m68k frontend lifted an instruction that commits {} stores; a fault after \
             the first cannot be restarted (lift.rs, \"A fault is handled by restarting the \
             instruction\")",
            self.stores
        );
        if addr & !lift::WINDOW_MASK == self.code_window {
            self.smc = true;
        }
        self.bus_cycle();
        let at = u64::from(addr & self.mask);
        self.space.write(at, mem.size, value, self.attrs)
    }

    fn charge(&mut self, ticks: u64) {
        self.used = self.used.wrapping_add(ticks);
        self.state.cycles = self.state.cycles.wrapping_add(ticks);
    }

    fn insn_start(&mut self, _mark: &InsnStart) {
        // Restart granularity is the guest instruction, so the previous one's
        // store stops counting against this one's budget — and the tick mark a
        // restart unwinds back to moves here.
        self.stores = 0;
        self.retired = self.used;
    }

    fn spent(&self) -> bool {
        // Monotone in each term, which is what the seam requires: ticks only
        // rise, the store guard only latches, and an interrupt only becomes
        // pending. `interrupt_pending` is the non-consuming form on purpose —
        // see `Lines::interrupt_pending`.
        self.used >= self.allowance || self.smc || self.lines.interrupt_pending(self.ipl_mask)
    }
}

#[cfg(test)]
mod tests {
    use super::super::differential::{CODE, Case, DATA, compare, measure, stats_for};
    use super::super::lift::Decline;
    use super::super::{Engine, M68k};
    use crate::core::props::{Props, Value};
    use alloc::vec;
    use alloc::vec::Vec;

    /// `STOP #$2700`.
    const STOP: [u16; 2] = [0x4e72, 0x2700];

    #[track_caller]
    fn agreed(case: &Case) {
        if let Err(d) = compare(case) {
            panic!("{d}");
        }
    }

    #[test]
    fn the_property_selects_the_engine_and_the_default_is_the_interpreter() {
        let cpu = M68k::from_props(&Props::new()).expect("an empty property set is a 68000");
        assert_eq!(cpu.engine(), Engine::Interp);
        let props = Props::new().with("engine", Value::Str("ir".into()));
        let cpu = M68k::from_props(&props).expect("`ir` is a 68000 engine");
        assert_eq!(cpu.engine(), Engine::Ir);
        let props = Props::new().with("engine", Value::Str("interp".into()));
        let cpu = M68k::from_props(&props).expect("`interp` still works");
        assert_eq!(cpu.engine(), Engine::Interp);
    }

    #[test]
    fn a_nonsense_engine_is_refused_rather_than_ignored() {
        let props = Props::new().with("engine", Value::Str("jit-host".into()));
        M68k::from_props(&props).expect_err("an engine that does not exist is an error");
    }

    #[test]
    fn a_core_that_never_ran_has_no_statistics_and_one_that_did_has_some() {
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Ir);
        assert_eq!(cpu.ir_stats(), None, "nothing has run yet");
        let stats = stats_for(&Case::seeded(vec![0x4e71, STOP[0], STOP[1]]).with_units(3))
            .expect("a core that ran has statistics");
        assert!(stats.executed > 0, "{stats:?}");
        assert!(stats.retired > 0, "{stats:?}");
    }

    #[test]
    fn a_pending_interrupt_keeps_a_block_from_running_at_any_boundary() {
        // `liftable` refuses a block while one is pending, and `Host::spent`
        // leaves the block at the next boundary if one arrives inside it —
        // because the interpreter checks for one before every instruction, and
        // the two engines have to take it at the same one.
        let program = vec![0x4e71, 0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(4);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Ir);
        cpu.attach_space(space);
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        // An interrupt mask of 3 and a level of 5, so the request is above it.
        regs.sr = super::super::flags::S | 0x0300;
        cpu.set_regs(regs);
        cpu.set_ipl(5);
        let before = cpu.ir_stats().unwrap_or_default();
        cpu.step();
        let after = cpu.ir_stats().unwrap_or_default();
        assert_eq!(
            after.executed, before.executed,
            "no block may run with an interrupt pending: {after:?}"
        );
        assert!(
            cpu.regs().sr & super::super::flags::IPL == 0x0500,
            "vectored"
        );
    }

    #[test]
    fn a_budget_no_whole_block_fits_in_still_retires_inside_one() {
        // `Host::spent` is the seam that lets a block leave part-way through,
        // and `run_budget` is what hands it the remaining allowance. Both
        // engines must end the quantum on the same instruction with the same
        // debt, which is a column `compare` checks — this asserts the *shape*:
        // a four-cycle budget over a block of `NOP`s stops after one.
        let program = vec![0x4e71, 0x4e71, 0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(4);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Ir);
        cpu.attach_space(space);
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.run_budget(4);
        assert_eq!(cpu.regs().pc, CODE + 2, "one NOP and no more");
        let stats = cpu.ir_stats().expect("it ran");
        assert_eq!(stats.spent, 1, "the block left at a boundary: {stats:?}");
    }

    #[test]
    fn a_topology_change_throws_the_translations_away() {
        // Derived state, invalidated by the generation counter (CLAUDE.md,
        // "Devices"). A block lifted before a remap must not survive it.
        let program = vec![0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(3);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Ir);
        cpu.attach_space(alloc::sync::Arc::clone(&space));
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.step();
        let before = cpu.ir_stats().expect("it ran").lifted;
        assert!(before > 0);
        // Any topology change moves the generation, whatever it does.
        {
            let mut topo = space.topology();
            topo.rebuild();
        }
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.step();
        let after = cpu.ir_stats().expect("it ran").lifted;
        assert!(
            after > before,
            "the cache was not thrown away: {before} {after}"
        );
    }

    #[test]
    fn a_store_the_guest_makes_into_a_translated_window_invalidates_it() {
        // The other half of the self-modifying-code answer: `Host::spent`
        // leaves the block, and the *next* dispatch has to notice the bytes
        // changed rather than serving the stale translation.
        //
        // MOVE.W D0,(A2) with A2 aimed at the instruction after it, run twice
        // round a loop so the second pass is a lookup.
        let program = vec![0x3480, 0x4e71, 0x60fa, STOP[0], STOP[1]];
        let case = Case::seeded(program)
            .with_a(2, CODE + 2)
            .with_d(0, 0x4e71)
            .with_units(12);
        agreed(&case);
        let stats = stats_for(&case).expect("it ran");
        assert!(
            stats.invalidated > 0 || stats.lifted > 1,
            "a rewritten window must not be served from the cache: {stats:?}"
        );
    }

    #[test]
    fn a_reset_throws_the_translations_away_and_the_run_still_agrees() {
        let program = vec![0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(3);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Ir);
        cpu.attach_space(space);
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.step();
        cpu.request_reset();
        cpu.step();
        assert_eq!(cpu.regs().pc, CODE, "the reset vector points at the code");
    }

    #[test]
    fn a_data_access_reaches_the_pins_the_model_has() {
        // A 68000 drives twenty-four address pins and the wrap is observable:
        // `Host::load` masks with `Model::address_mask` exactly as
        // `exec::bus_addr` does. Reaching `DATA` through an address above the
        // pins must land on `DATA`.
        let case = Case::seeded(vec![0xd052, STOP[0], STOP[1]])
            .with_a(2, 0xff00_0000 | DATA)
            .with_units(2);
        agreed(&case);
    }

    // -- the instrument -------------------------------------------------
    //
    // A counter whose own arithmetic is untested is a counter that will be
    // quoted in a document and be wrong. These are the three properties the
    // measurement rests on, and each one is a *closure* property: every
    // fallback is attributed to a row, every block execution is attributed to
    // an outcome, and a category is the category the docs name.

    /// Every guest instruction the interpreter took is in exactly one row.
    ///
    /// Without this the histogram's largest bucket is silently "the ones
    /// nobody counted", and the measurement this instrument exists for reads
    /// as a lift rate better than it is.
    #[test]
    fn the_decline_histogram_accounts_for_every_fallback() {
        // A program with all five shapes in it: a lifted `MOVE`, a declined
        // `JSR` (two stores), a declined `MULU` (not written yet), a declined
        // `ASL D1,D2` (a register count), and `STOP`, which leaves the core in
        // a state no block may run in.
        let program = vec![
            0x3001, // MOVE.W D1,D0        -- lifted
            0x4eb9, 0x0000, 0x1010, // JSR $00001010    -- stores
            0xc2c3, // MULU D3,D1          -- gap
            0xe3a2, // ASL.L D1,D2         -- charge
            0x4e71, // NOP                 -- lifted
            STOP[0], STOP[1], // STOP #$2700       -- gap, then state
            0x4e71,
        ];
        let case = Case::seeded(program).with_units(24);
        agreed(&case);
        let (stats, rows) = measure(&case);
        let stats = stats.expect("it ran");
        let total: u64 = rows.iter().map(|r| r.count).sum();
        assert_eq!(
            total, stats.interpreted,
            "the histogram sums to {total} and {} instructions fell back:\n{rows:#?}",
            stats.interpreted
        );
        assert!(stats.interpreted > 0, "the fallback ran: {stats:?}");
        assert!(stats.retired > 0, "and so did a block: {stats:?}");
    }

    /// Every block execution ended in exactly one of the ways counted.
    ///
    /// `executed` is the divisor of "mean instructions per block", so a block
    /// counted as executed and attributed to no outcome would move that mean
    /// without moving anything that explains it.
    #[test]
    fn every_block_execution_is_attributed_to_an_outcome() {
        // A loop, so blocks are executed many times and the budget cuts one
        // short: `DBF D0,*` around a `MOVE` and an `ADD`.
        let program = vec![
            0x3001, // MOVE.W D1,D0
            0xd280, // ADD.L D0,D1
            0x51c8, 0xfffa, // DBF D0,$1000
            STOP[0], STOP[1],
        ];
        let case = Case::seeded(program).with_d(0, 40).with_units(80);
        agreed(&case);
        let (stats, _) = measure(&case);
        let stats = stats.expect("it ran");
        let ended = stats.ended_unsupported
            + stats.ended_transfer
            + stats.ended_window
            + stats.ended_limit
            + stats.ended_unreadable;
        assert_eq!(
            stats.executed,
            ended + stats.spent + stats.faults,
            "{} blocks ran; {ended} reached a terminator, {} left early and {} faulted: {stats:?}",
            stats.executed,
            stats.spent,
            stats.faults
        );
        assert!(
            stats.ended_transfer > 0,
            "a `DBcc` loop ends its blocks at a transfer: {stats:?}"
        );
    }

    /// The category and the mnemonic are the ones `docs/cpu/m68k.md` names.
    ///
    /// The one assertion that would catch a decline moved from one bucket to
    /// another by a later change to `classify` — which is exactly what would
    /// make a re-measurement incomparable with this one.
    #[test]
    fn a_decline_is_reported_under_the_category_the_docs_name() {
        let want: &[(&[u16], Decline, &str)] = &[
            // `JSR $00001010` pushes a long: two word stores.
            (&[0x4eb9, 0x0000, 0x1010], Decline::STORES, "JSR"),
            // `BSR.W` the same.
            (&[0x6100, 0x0004], Decline::STORES, "BSR"),
            // `PEA $00001010` and `LINK A2,#0` likewise.
            (&[0x4879, 0x0000, 0x1010], Decline::STORES, "PEA"),
            (&[0x4e52, 0x0000], Decline::STORES, "LINK"),
            // `MOVE.L D1,(A2)` is a long memory destination.
            (&[0x2481], Decline::STORES, "MOVE"),
            // `MOVEM.L D0-D1,(A2)` is one store per register.
            (&[0x48d2, 0x0003], Decline::STORES, "MOVEM"),
            // `MULU D3,D1`: a cycle count out of the microcode's loop shape.
            (&[0xc2c3], Decline::GAP, "MULU"),
            // `ASL.L D1,D2`: two cycles a bit, at a run-time count.
            (&[0xe3a2], Decline::CHARGE, "ASL"),
            // `SNE D0`: two extra cycles when the byte is set.
            (&[0x56c0], Decline::CHARGE, "S"),
        ];
        for &(program, reason, what) in want {
            let mut words = program.to_vec();
            words.extend_from_slice(&STOP);
            let case = Case::seeded(words).with_units(4);
            agreed(&case);
            let (_, rows) = measure(&case);
            let found: Vec<_> = rows
                .iter()
                .filter(|r| r.count > 0 && r.reason != Decline::STATE)
                .collect();
            assert!(
                found
                    .iter()
                    .any(|r| r.reason == reason && r.what == what && r.count > 0),
                "{program:04x?} should be declined as {}/{what}, and the rows are {found:#?}",
                reason.name()
            );
        }
    }

    /// A `STOP` is counted as a *state* cause, by name, rather than as an
    /// encoding the subset is missing.
    ///
    /// The distinction the measurement turns on: the first is a guest waiting
    /// for an interrupt, which no frontend can lift and no `set_slot` would
    /// help, and the second is work somebody could do.
    #[test]
    fn a_stopped_core_is_counted_as_a_state_cause() {
        let case = Case::seeded(vec![0x4e71, STOP[0], STOP[1]]).with_units(8);
        agreed(&case);
        let (_, rows) = measure(&case);
        let stopped = rows
            .iter()
            .find(|r| r.reason == Decline::STATE && r.what == "stopped")
            .map_or(0, |r| r.count);
        assert!(stopped > 0, "a `STOP`ped core is counted: {rows:#?}");
    }
}
