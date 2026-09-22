//! The translated execution engine: [`lift`](super::lift)'s blocks run on
//! `jit`'s dispatcher, with the interpreter underneath everything it declines.
//!
//! `engine = "jit"` and `engine = "jit-host"` on `cpu.m68k` select it. What it
//! is *not* is a second semantics: `ROADMAP.md` §0 requires a bit-identical
//! state hash across the interpreter and a translated engine for the same
//! guest, so every column a guest or a snapshot can see — registers, `SR`, the
//! program counter, the **prefetch queue**, memory, faults, and the cycle
//! count — comes out the same number.
//! [`differential`](super::differential) is what says so.
//!
//! # Why this is `jit::Dispatcher` and not a cache of its own
//!
//! It used to be a cache of its own, and the measurement that ended that is
//! worth keeping because the answer was not the expected one.
//! `docs/cpu/m68k.md` named three reasons the translated engine was 2.5×
//! *slower* than the interpreter it exists to accelerate: no host code
//! generator, no block chaining, and a cache that re-read every word a block
//! was lifted from on every dispatch. The obvious suspect was chaining — a
//! 68000 block is 2.38 guest instructions on a Macintosh Plus ROM, so a
//! dispatch round trip is amortised over almost nothing.
//!
//! Under callgrind, on one virtual second of that ROM, it was not:
//!
//! | | host instructions | share |
//! | --- | ---: | ---: |
//! | executing the IR (`Interp::step`, `Interp::set`, and the loop over them) | 6.33 G | **71.3%** |
//! | the scheduler and the devices | 0.80 G | 9.0% |
//! | the address space, under a block's own accesses | 0.72 G | 8.1% |
//! | the interpreter, on what the subset declines | 0.35 G | 3.9% |
//! | **the cache lookup, the dispatch and `Host` setup** | **0.21 G** | **2.4%** |
//! | **re-validating a block's words on every dispatch** | **0.10 G** | **1.2%** |
//!
//! The frontend emits **51 IR ops per guest instruction** — it performs its
//! fetches and it computes every condition-code bit — and `ir::Interp` spends
//! about 190 host instructions on each one. Chaining and validation together
//! are 3.6% of the run. **The executor was the whole of it**, and the two
//! small rows only become worth removing once it is gone, which is exactly
//! what moving here does: `jit::Dispatcher` brings a host code generator, the
//! block cache with direct-linked chaining, and a page-granular
//! self-modifying-code filter that replaces the re-read, in one move.
//!
//! # The shape, in one paragraph
//!
//! One call to `advance` runs **up to [`CHAIN`] blocks, or one interpreted
//! instruction**. A block is lifted once, cached under `(entry PC, key)`, and
//! executed by `jit::host`'s generated code where a backend takes it and by
//! `ir::Interp` where one does not — the two being indistinguishable to the
//! guest, which is what makes the interpreter still the oracle.
//!
//! # The key is the prefetch queue, and that is not an optimisation
//!
//! A 68000 fetches **two words ahead**, so the instruction at a block's entry
//! PC is the word already in `prefetch[0]` — not whatever is at that address
//! now. [`Reader`] therefore answers the entry PC and the word after it out of
//! `State::prefetch` and everything from `pc + 4` on out of memory.
//!
//! That makes the queue part of what names the translation, and it has to be
//! in the **cache key** rather than merely recorded: a store that rewrites the
//! word at a block's entry invalidates the block through the page filter, the
//! block is re-lifted from the queue the guest had *then*, and control
//! reaching that PC again with the queue holding the *new* word would
//! otherwise be served the stale translation. [`cache_key`] is where the two
//! words go, beside the model.
//!
//! # Four reasons a block does not run, and the interpreter picks it up
//!
//! 1. **The core is not in a liftable state** ([`unliftable_at`]): a pending
//!    reset, a halt, `STOP`, an `RTE` replay in flight, **T** set in `SR`
//!    (every instruction would take a trace exception), an odd program
//!    counter, a pending interrupt, or any model but a 68000.
//! 2. **The instruction at the PC is outside the subset**, so the lift
//!    produced a block covering nothing. Recorded in [`Unlifted`], so the next
//!    pass does not lift it again.
//! 3. **A fault.** See below.
//! 4. **The run left part-way through a block** — `Stop::Spent` — which is not
//!    a fallback at all: the guest is standing at a boundary and the next call
//!    picks up from there.
//!
//! # A fault restarts the instruction on the interpreter
//!
//! `lift`'s module docs have the argument in full. The short form: a 68000's
//! mid-instruction fault is visible in registers the instruction has already
//! changed and in a stack frame whose program counter depends on how far the
//! prefetch got, and the IR cannot publish a write that lands between two
//! boundaries. So `advance` unwinds the partial instruction's charges — which
//! is the reconciliation [`Fault`](crate::ir::Fault) documents for a core that
//! restarts rather than resumes — and hands that one instruction to
//! `Exec::step`, which redoes it and faults where the hardware does.
//!
//! The two numbers to unwind by are **the host's**, not the IR's, and that is
//! worth stating because getting it wrong was a real defect rather than a
//! hypothetical: `Fault::charged_ticks` and `Fault::retired_ticks` are both
//! measured from the backend's own counter, which sees
//! [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) immediates and **not** what a
//! host charges inside an access. On a core where every bus cycle is four
//! host-charged ticks that is most of the count. [`Host::retired`] has the
//! worked example.
//!
//! The lifted subset is chosen so that this is **exact**: no lifted
//! instruction commits more than one store, and the store is its last memory
//! access, so a fault can only arrive with nothing committed. [`Host::store`]
//! asserts it in a debug build rather than trusting the frontend.
//!
//! # Self-modifying code
//!
//! A cached block is a translation of bytes, so it is only valid while those
//! bytes are what they were. **Three** mechanisms, and none is a heuristic:
//!
//! * **The page filter.** Every store a block makes is recorded, by
//!   guest-physical [`lift::WINDOW`] page, in a [`DirtyPages`] log the
//!   dispatcher drains at the next block boundary and matches against the page
//!   each translation was lifted from. That is `ROADMAP.md` §9.1's third
//!   mechanism, and it is what replaced re-reading a block's words on every
//!   dispatch.
//! * **The other half of it**, for everything the subset declines: an
//!   interpreted instruction reports what it wrote through `Exec::wrote`, and
//!   [`drain`] hands that to the same filter. Without it a `MOVEM` — which is
//!   not lifted, and which is what a graphics primitive saves registers with —
//!   could write over a cached translation and leave it cached.
//! * **Leaving the block.** A filter drained at a boundary cannot help a store
//!   the *running* block makes into its own window, so [`Host::store`] notices
//!   one and [`Host::spent`] then leaves at the next guest instruction
//!   boundary — the boundary the store's own instruction ends at, which is
//!   where the effect can first be honoured. A 68000 has no instruction cache
//!   and no `FENCE.I`: coherency is architectural but for the queue, so this
//!   guard is not optional the way `cpu::riscv`'s is.
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
//! engine counts, and **every fallback is attributed to a row**:
//! [`Unlifted`] carries the [`DeclineRow`] index a PC's fallback belongs to,
//! resolved once when the PC is lifted, and the dispatch path pays one indexed
//! add for it. The rows sum to [`Stats::interpreted`], so there is no "other"
//! bucket for the ones nobody counted. Both this file's tests and
//! `tests/m68k_lift_rate.rs` assert that closure on a running board.
//!
//! # Sources
//!
//! `exec.rs` is the oracle for every number here; where a cycle count or an
//! access order is asserted, the citation is in `lift.rs` beside the code that
//! emits it. No emulator source of any licence was opened (`ROADMAP.md` §1).

use alloc::vec;
use alloc::vec::Vec;

use crate::core::error::{BusError, Result};
use crate::core::space::{AddressSpace, MemAttrs, MemResult};
use crate::core::value::Width;
use crate::ir::{Align, InsnStart, IrHost, MemOp, RegSlot, verify};
use crate::jit::{
    BlockCache, DirtyPages, Dispatcher, Entry, Epoch, FastMem, Frontend, Stop, StoreLog,
    Translation,
};

use super::exec::{Exec, State};
use super::isa::Model;
use super::lift::{self, Decline, Declined, SLOT_COUNT};
use super::{Config, Engine, Lines, flags};

/// How many blocks one call to [`advance`] may run.
///
/// The bound on a chain, and therefore the bound on how long a safe point can
/// be delayed: [`CHAIN`] × [`lift::MAX_INSNS`] is **512 guest instructions**,
/// which on a 7.8 MHz 68000 is a few thousand cycles. `cpu::riscv::engine`
/// states its own the same way and for the same reason.
///
/// It is also the unit [`differential`](super::differential) compares at: one
/// `M68k::step` is one chain, and the harness steps the oracle by
/// [`Stats::steps`] to match.
const CHAIN: usize = 16;

/// How many blocks this core's cache holds before it evicts.
///
/// A 68000 addresses sixteen megabytes and this frontend keys a block on its
/// prefetch queue as well as its PC, so the working set is bounded by the code
/// the guest runs rather than by the space: a twelve-second Macintosh Plus ROM
/// boot lifts about 1 600 distinct blocks and a Classic about 2 400. Eight
/// thousand is `jit::BlockCache`'s own default and holds both several times
/// over.
const BLOCKS: usize = 8192;

/// How much memory `engine = "jit-host"` reserves for generated code.
///
/// Small beside `cpu::riscv::engine`'s 256 MiB, and for a reason rather than
/// by oversight: a 68000's whole address space is sixteen megabytes and its
/// blocks are 2.4 guest instructions long, so a working set that fills this is
/// a guest that has rewritten its code thousands of times over. A buffer that
/// fills is reset, which costs a recompile and no wrong answer.
#[cfg(any(
    all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
    all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
))]
const CODE_BUFFER: u64 = 32 << 20;

/// How many PCs [`Unlifted`] remembers.
///
/// Direct-mapped and lossy on purpose: a miss costs one lift that produces
/// nothing, which is the price the bespoke cache used to pay on every fallback
/// before it too remembered them.
const UNLIFTED_SLOTS: usize = 4096;

// The slot numbering, as indices into the host's flat array. Named here rather
// than cast at each use so a slot added to `lift` cannot silently shift one.
const SR_INDEX: usize = 16;
const PC_INDEX: usize = 17;
const P0_INDEX: usize = 18;
const P1_INDEX: usize = 19;

/// What names a translation besides its entry PC.
///
/// The model, because this frontend refuses every other one; and **both
/// prefetch words**, because [`Reader`] lifts the block's first two words out
/// of the queue rather than out of memory. See the module docs for why
/// recording them is not enough and they have to be in the key.
#[must_use]
fn cache_key(model: Model, queue: [u16; 2]) -> u64 {
    lift::key(model) | (u64::from(queue[0]) << 16) | (u64::from(queue[1]) << 32)
}

/// The guest-physical page a translation entered at `pc` was lifted from.
///
/// A block never leaves its [`lift::WINDOW`], which is the same four kilobytes
/// `jit::PAGE_SIZE` is, so one page is the whole answer. Masked to the pins
/// this model drives first: the wrap at twenty-four bits is observable, and a
/// store at `$FF00_1000` and a fetch at `$0000_1000` reach the same byte.
#[must_use]
fn lifted_page(pc: u32, mask: u32) -> u64 {
    u64::from((pc & mask) & !lift::WINDOW_MASK)
}

/// What a translated core has done, and what it holds.
///
/// A statistic and never a behaviour — the engines are indistinguishable to
/// the guest — so nothing here is snapshotted.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Stats {
    /// Distinct blocks lifted.
    pub lifted: u64,
    /// Blocks executed.
    pub executed: u64,
    /// Blocks reached by following a patched exit rather than by a lookup.
    pub chained: u64,
    /// Blocks entered by a **direct link**: a jump from the predecessor's own
    /// compiled code, with no return to the dispatch loop at all.
    ///
    /// A subset of [`chained`](Stats::chained), and the one that says whether
    /// the patch reached generated code.
    pub linked: u64,
    /// Blocks executed as compiled host code rather than as interpreted IR.
    pub compiled: u64,
    /// Guest instructions retired inside a block.
    pub retired: u64,
    /// Instructions the interpreter executed because no block could.
    pub interpreted: u64,
    /// Faults a block took, each handed back to the interpreter.
    pub faults: u64,
    /// Translations dropped because the guest rewrote the page they were
    /// lifted from — the two rows below, added.
    pub invalidated: u64,
    /// Translations a store from a **block** invalidated, through
    /// [`StoreLog`].
    pub invalidated_in_block: u64,
    /// Translations a store from an **interpreted instruction** invalidated,
    /// through `drain`.
    ///
    /// Separate from the row above because they are separate mechanisms on
    /// separate paths, and a single total lets either of them stop working
    /// while the other keeps the number above zero.
    pub invalidated_interpreted: u64,
    /// Runs that stopped part-way through a block, at a guest instruction
    /// boundary, because the caller's tick allowance ran out.
    pub spent: u64,
    /// Runs that stopped at an instruction no block could carry, so the
    /// interpreter took it.
    pub declined: u64,
    /// Guest instructions retired **in the unit the interpreter counts**:
    /// `Exec::step`s, so an exception sequence is one and an instruction is
    /// one.
    ///
    /// The column [`differential`](super::differential) drives the oracle by,
    /// which is why it is a statistic worth keeping rather than a curiosity:
    /// a translated unit is a whole *chain* of blocks, and the harness has to
    /// know how many interpreter steps that chain was worth to step the oracle
    /// exactly that far.
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

/// The [`Decline::STATE`] causes, in the order [`unliftable_at`] checks them.
///
/// A fixed array rather than a row in [`Runtime::declines`] because this is
/// checked at *every* boundary: the index is the whole lookup, where a row
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

// ---------------------------------------------------------------------------
// The PCs there is no block at
// ---------------------------------------------------------------------------

/// The `(pc, key)` pairs a lift produced nothing for, and the histogram row
/// each one's fallback belongs to.
///
/// Direct-mapped, lossy and cheap. `jit::BlockCache` holds translations, so a
/// PC with none would otherwise be re-lifted on every dispatch — and on a real
/// Macintosh ROM 6.2% of executed instructions are outside the subset, which
/// is 6.2% of instructions paying a decode and an allocation each.
#[derive(Debug)]
struct Unlifted {
    slots: alloc::boxed::Box<[Slot]>,
}

#[derive(Debug, Clone, Copy, Default)]
struct Slot {
    pc: u32,
    key: u64,
    /// Which row of [`Runtime::declines`] this PC's fallback belongs to.
    row: u32,
    live: bool,
}

impl Unlifted {
    fn new() -> Unlifted {
        Unlifted {
            slots: vec![Slot::default(); UNLIFTED_SLOTS].into_boxed_slice(),
        }
    }

    /// The low bit of a 68000 instruction address is always zero, so it
    /// carries nothing.
    #[inline]
    fn index(pc: u32) -> usize {
        ((pc >> 1) as usize) & (UNLIFTED_SLOTS - 1)
    }

    /// The histogram row for this `(pc, key)`, if the last lift here produced
    /// nothing.
    #[inline]
    fn holds(&self, pc: u32, key: u64) -> Option<u32> {
        let slot = &self.slots[Unlifted::index(pc)];
        (slot.live && slot.pc == pc && slot.key == key).then_some(slot.row)
    }

    #[inline]
    fn note(&mut self, pc: u32, key: u64, row: u32) {
        self.slots[Unlifted::index(pc)] = Slot {
            pc,
            key,
            row,
            live: true,
        };
    }

    fn clear(&mut self) {
        self.slots.fill(Slot::default());
    }
}

// ---------------------------------------------------------------------------
// What a core keeps between chains
// ---------------------------------------------------------------------------

/// The dispatcher, the PCs there is no block at, and the census beside them.
///
/// Lives in the core's session, behind the same lock the interpreter's state
/// is behind, and is **derived state**: never serialized, thrown away on a
/// reset and whenever the topology generation moves (CLAUDE.md, "Devices").
#[derive(Debug)]
pub(super) struct Runtime {
    disp: Dispatcher,
    unlifted: Unlifted,
    /// What this file counts beside [`Dispatcher::stats`].
    local: Local,
    /// The decline histogram, one row per `(category, what)` pair seen.
    ///
    /// Insertion-ordered and never cleared, so an [`Unlifted`] row index stays
    /// valid across a cache flush. Bounded by five categories times the
    /// mnemonics `isa.rs` has, which is a few hundred rows at the very most
    /// and a handful in practice.
    declines: Vec<DeclineRow>,
    /// Fallbacks by [`STATE_CAUSES`] index.
    states: [u64; STATE_CAUSES.len()],
    /// The address space generation [`Unlifted`] was filled under.
    ///
    /// The block cache has its own answer to staleness — `BlockCache::sync`
    /// against [`Frontend::epoch`] — and [`Unlifted`] is outside it. A stale
    /// entry there is never a *wrong* answer, because what it sends the PC to
    /// is the oracle, but it would pin an instruction to the interpreter for
    /// the rest of a run after a remap made it liftable. One relaxed atomic
    /// load per call is the price of not having to reason about that.
    generation: u64,
}

/// The counters the dispatcher does not keep.
#[derive(Debug, Default, Clone, Copy)]
struct Local {
    retired: u64,
    interpreted: u64,
    faults: u64,
    spent: u64,
    declined: u64,
    smc: u64,
    steps: u64,
}

impl Runtime {
    /// A fresh runtime, with whichever backend `engine` asks for.
    ///
    /// A build or a host without the one asked for gets the portable backend
    /// instead, which is not a failure and not a different guest
    /// (`ROADMAP.md` §9, "Backends") — the same fallback `cpu::riscv` makes,
    /// so a machine file is portable and a measurement is not silently of
    /// something else.
    pub(super) fn new(engine: Engine) -> Runtime {
        let disp = Dispatcher::with_cache(BlockCache::with_capacity(BLOCKS));
        #[cfg(any(
            all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
            all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
        ))]
        let disp = match (engine == Engine::JitHost)
            .then(|| crate::jit::host::Engine::with_capacity(CODE_BUFFER))
            .flatten()
        {
            Some(host) => disp.with_backend(host),
            None => disp,
        };
        let _ = engine;
        Runtime {
            disp,
            unlifted: Unlifted::new(),
            local: Local::default(),
            declines: Vec::new(),
            states: [0; STATE_CAUSES.len()],
            generation: 0,
        }
    }

    /// What this core's translated engine has done.
    pub(super) fn stats(&self) -> Stats {
        let d = self.disp.stats();
        Stats {
            lifted: d.translated,
            executed: d.blocks,
            chained: d.chained,
            linked: d.linked,
            compiled: d.compiled,
            retired: self.local.retired,
            interpreted: self.local.interpreted,
            faults: self.local.faults,
            invalidated: d.smc + self.local.smc,
            invalidated_in_block: d.smc,
            invalidated_interpreted: self.local.smc,
            spent: self.local.spent,
            declined: self.local.declined,
            steps: self.local.steps,
        }
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

    /// Throw every translation away.
    ///
    /// The histogram is not thrown away with it: it is a record of what this
    /// core *did*, and an [`Unlifted`] row index into it stays valid because
    /// rows are only ever appended.
    pub(super) fn flush(&mut self) {
        self.disp.cache_mut().flush();
        self.unlifted.clear();
    }
}

/// The row `what` belongs in, appending one if this is the first time.
///
/// A linear scan, and deliberately: it runs once per *lift*, the vector is
/// tiny, and the alternative — a map keyed on a pair — would put an allocation
/// and a comparison chain on a path the dispatch loop shares.
fn row_for(declines: &mut Vec<DeclineRow>, declined: Declined) -> u32 {
    let found = declines
        .iter()
        .position(|r| r.reason == declined.reason && r.what == declined.what);
    let at = match found {
        Some(at) => at,
        None => {
            declines.push(DeclineRow {
                reason: declined.reason,
                what: declined.what,
                count: 0,
            });
            declines.len() - 1
        }
    };
    // A row index is `u32` in [`Slot`], and a saturating conversion is the
    // honest failure: the vector cannot reach four billion rows, and a count
    // landing on row zero would be a wrong number rather than a panic.
    u32::try_from(at).unwrap_or(0)
}

/// Why the core is in no state for a lifted block to run at `pc` — as an index
/// into [`STATE_CAUSES`] — or `None` when it is.
///
/// Each of these is something `Exec::step_inner` does *before* it reaches an
/// instruction, or something the lifted subset cannot express, and a block that
/// ran anyway would skip it.
///
/// It returns the *cause* rather than a boolean because "the core was not
/// liftable" is four different facts about a real guest — a `STOP` loop, an
/// interrupt at every vertical blank, a trace bit, a halt — and a measurement
/// that could not tell them apart could not say whether the frontend's
/// coverage was the frontend's fault.
///
/// `pc` is passed rather than read out of `state`, because at a **chained**
/// boundary the architectural PC is still in the host's slots and has not been
/// written back. Nothing else this reads can move under a block: no lifted
/// instruction writes **S**, **T** or the interrupt mask, and none of them can
/// halt, stop, reset or start an `RTE` replay.
fn unliftable_at(state: &State, cfg: &Config, lines: &Lines, pc: u32) -> Option<usize> {
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
    if pc & 1 != 0 {
        return Some(6);
    }
    if lines.interrupt_pending(state.ipl_mask()) {
        return Some(7);
    }
    None
}

/// Advance the core by one *unit of this engine*: a chain of up to [`CHAIN`]
/// blocks, or — where a block would be wrong — one interpreted instruction.
///
/// `allowance` is what is left of the caller's tick budget, and it is an
/// allowance rather than advice: [`Host::spent`] compares it against the ticks
/// charged at every guest instruction boundary but the run's first, and a run
/// that has spent it leaves *there* — at the same instruction an interpreted
/// core would have stopped at, with the same `State::debt`. Pass [`u64::MAX`]
/// to mean "run the whole chain".
///
/// Returns the cycles charged, in the same currency and with the same meaning
/// as `Exec::step`, so a run loop cannot tell which engine it is driving.
///
/// # Panics
///
/// If a lifted block reaches an op the backend does not implement. That is not
/// a guest condition: it is this crate's own frontend emitting something its
/// own backend cannot execute, and the architectural state at that point is not
/// reconstructible — so it is reported loudly rather than papered over, exactly
/// as `cpu::riscv::engine` does.
pub(super) fn advance(
    rt: &mut Runtime,
    state: &mut State,
    space: &AddressSpace,
    cfg: &Config,
    lines: &Lines,
    allowance: u64,
) -> u64 {
    let Runtime {
        disp,
        unlifted,
        local,
        declines,
        states,
        generation,
    } = rt;

    // Derived state, invalidated by the topology generation counter
    // (CLAUDE.md, "Devices"). The *block* cache does this itself, inside the
    // dispatcher and at every boundary rather than every call; this is the
    // half of the derived state that lives outside it.
    let now = space.generation();
    if *generation != now {
        *generation = now;
        unlifted.clear();
    }

    // The entry work for the *first* block, done here rather than through
    // [`Frontend::enter`], because reaching the interpreter for an instruction
    // outside the subset should not cost a frontend, a host and a dispatcher
    // round trip. The dispatcher's first `enter` is then a no-op; see
    // [`Lifter::admitted`].
    let pc = state.pc;
    if let Some(cause) = unliftable_at(state, cfg, lines, pc) {
        states[cause] += 1;
        return interpret(local, disp, unlifted, state, space, cfg, lines);
    }
    let key = cache_key(cfg.model, state.prefetch);
    if let Some(row) = unlifted.holds(pc, key) {
        if let Some(row) = declines.get_mut(row as usize) {
            row.count += 1;
        }
        return interpret(local, disp, unlifted, state, space, cfg, lines);
    }

    let mut front = Lifter {
        cfg,
        space,
        // A read-ahead rather than a fetch: this reads up to `MAX_INSNS`
        // instructions the guest has not asked for, which is the one place
        // this core reads guest memory the way a debugger does (CLAUDE.md, "a
        // debugger read must not pop a FIFO"). The block's *own* fetch
        // accesses are performed at run time, through the host, with ordinary
        // attributes.
        attrs: MemAttrs::DEBUG.with_requester(cfg.requester),
        mask: cfg.model.address_mask(),
        queue: state.prefetch,
        key,
        admitted: true,
        // Reborrowed rather than moved, so both are usable again the moment
        // this frontend is dropped — the fallback path below needs them.
        unlifted: &mut *unlifted,
        declines: &mut *declines,
        declined_row: None,
        rejected: false,
    };
    let mut host = Host::new(state, space, cfg, lines, allowance, pc);
    let run = disp.run(&mut front, &mut host, u64::from(pc), CHAIN);

    let Lifter {
        declined_row,
        rejected,
        ..
    } = front;
    debug_assert!(
        !rejected,
        "the m68k frontend produced a block the verifier rejects"
    );

    // Nothing executed, which on this frontend has exactly one cause: the
    // first block was pre-admitted above, so the only boundary the dispatcher
    // could refuse is a lift that covered no instruction at all. That PC has
    // just been resolved to a histogram row and remembered in [`Unlifted`];
    // count it here, because this is the call that interprets it.
    //
    // Every *other* fallback is counted by the next call's prologue rather
    // than where it was noticed — a chained boundary the dispatcher would not
    // enter leaves the guest standing on the instruction and returns, and the
    // prologue is what then finds the same cause and hands it to the
    // interpreter. One increment per interpreted instruction, wherever it came
    // from, which is the closure the histogram rests on.
    //
    // The `Err` arm joins it: the only refusal this frontend has is a model it
    // will not lift, which `unliftable_at` has already caught above, and
    // degrading to the interpreter is what `ROADMAP.md` §9 asks for anyway.
    let run = match run {
        Ok(run) if run.blocks > 0 => run,
        _ => {
            drop(host);
            if let Some(row) = declined_row.and_then(|r| declines.get_mut(r as usize)) {
                row.count += 1;
            }
            return interpret(local, disp, unlifted, state, space, cfg, lines);
        }
    };

    let Host {
        slots,
        used,
        retired,
        ..
    } = host;

    // Every retired instruction, back into the architectural register file.
    state.d.copy_from_slice(&slots[0..8]);
    state.a.copy_from_slice(&slots[8..16]);
    // **Only the condition codes.** No lifted instruction writes **S**, **T**
    // or the interrupt mask, so the stack-pointer bank cannot have moved under
    // the block and `State::set_sr`'s swap is not owed (`lift`'s module docs).
    state.sr = (state.sr & !flags::CCR) | (slots[SR_INDEX] as u16 & flags::CCR);
    state.prefetch = [slots[P0_INDEX] as u16, slots[P1_INDEX] as u16];
    local.retired = local.retired.wrapping_add(run.insns as u64);
    local.steps = local.steps.wrapping_add(run.insns as u64);

    match run.stop {
        Stop::Fault(fault) => {
            local.faults += 1;
            // The faulting instruction is the one that did *not* retire, and
            // the interpreter is about to take it — instruction and exception
            // together, which is one `Exec::step` and so one oracle step.
            local.steps += 1;
            state.cycles = state.cycles.wrapping_sub(used.saturating_sub(retired));
            state.pc = fault.pc as u32;
            // The queue as of that boundary is already in `state.prefetch`:
            // the block published it from the boundary's live mapping, or
            // never touched it, and either way it is what this instruction's
            // own fetches left.
            let mut exec = Exec::new(state, space, cfg, lines);
            let again = exec.step();
            drain(local, disp, unlifted, &exec);
            retired.wrapping_add(again).max(1)
        }
        Stop::Unsupported { op, at } => {
            panic!("the m68k frontend emitted {op} at index {at}, which the backend cannot execute")
        }
        stop => {
            match stop {
                Stop::Spent => local.spent += 1,
                // The boundary the chain would not enter: an encoding outside
                // the subset, or a state no block may run in. Nothing is
                // attributed here — the guest is left standing on that
                // instruction and the *next* call's prologue finds the same
                // cause, counts it and interprets it.
                Stop::Declined | Stop::Untranslatable { .. } => local.declined += 1,
                _ => {}
            }
            state.pc = run.pc as u32;
            used.max(1)
        }
    }
}

/// Interpret one instruction, and tell the block cache what it wrote.
#[allow(clippy::too_many_arguments)]
fn interpret(
    local: &mut Local,
    disp: &mut Dispatcher,
    unlifted: &mut Unlifted,
    state: &mut State,
    space: &AddressSpace,
    cfg: &Config,
    lines: &Lines,
) -> u64 {
    local.interpreted += 1;
    local.steps += 1;
    let mut exec = Exec::new(state, space, cfg, lines);
    let used = exec.step();
    drain(local, disp, unlifted, &exec);
    used
}

/// Hand what an interpreted instruction wrote to the block cache.
///
/// The block path reports its stores through [`StoreLog`] and the dispatcher
/// drains those itself; this is the other half, for every instruction outside
/// the lifted subset — a `MOVEM` saving a register set, a `MOVE.L` into a long
/// memory destination, a byte written by an exception handler.
fn drain(local: &mut Local, disp: &mut Dispatcher, unlifted: &mut Unlifted, exec: &Exec<'_>) {
    let (pages, overflowed) = exec.wrote();
    if pages.is_empty() && !overflowed {
        return;
    }
    let mut hit = 0usize;
    if overflowed {
        // More pages than the log holds. It cannot happen — the widest step is
        // a sixteen-register `MOVEM`, which is sixty-four contiguous bytes —
        // and if it ever does, throwing the cache away is the answer that
        // cannot be wrong.
        hit += disp.cache().len();
        disp.cache_mut().flush();
    } else {
        for &page in pages {
            hit += disp.cache_mut().note_write(u64::from(page), 1);
        }
    }
    local.smc = local.smc.wrapping_add(hit as u64);
    if hit > 0 {
        // A page a translation came from has changed, so every answer in
        // [`Unlifted`] may have changed with it — an instruction that was
        // outside the subset can have been overwritten by one that is not.
        unlifted.clear();
    }
}

// ---------------------------------------------------------------------------
// The frontend
// ---------------------------------------------------------------------------

/// The 68000 half of the dispatcher's contract, over a real core.
struct Lifter<'a> {
    cfg: &'a Config,
    space: &'a AddressSpace,
    attrs: MemAttrs,
    mask: u32,
    /// The prefetch queue the block about to be lifted is lifted *with*, which
    /// is where its first two words come from.
    queue: [u16; 2],
    /// [`cache_key`] of that queue, answered to [`Frontend::key`].
    key: u64,
    /// Whether [`advance`]'s prologue has already admitted the entry PC, so
    /// the dispatcher's first `enter` neither checks nor keys twice. Consumed
    /// by the first call and false ever after.
    admitted: bool,
    unlifted: &'a mut Unlifted,
    declines: &'a mut Vec<DeclineRow>,
    /// The histogram row [`Lifter::translate`] resolved for a lift that
    /// covered nothing, so [`advance`] can count it on the one path where the
    /// fallback is taken in the same call.
    declined_row: Option<u32>,
    /// Whether the verifier rejected a block this run lifted. A frontend bug
    /// rather than a guest one, so it is asserted on in a debug build and
    /// ignored in a release one — the block still runs, and the differential
    /// harness is where a malformed block is supposed to be caught.
    rejected: bool,
}

impl<'a, 'h> Frontend<Host<'h>> for Lifter<'a> {
    fn epoch(&mut self) -> Epoch {
        Epoch {
            // Read live, at every boundary: a chained successor must not be
            // served out of a cache lifted through a topology a store in the
            // block before it replaced. One relaxed atomic load.
            topology: self.space.generation(),
            // Zero, and deliberately: a 68000 has no memory management unit,
            // so there is no translation generation to be stale against.
            translation: 0,
        }
    }

    fn enter(&mut self, pc: u64, host: &mut Host<'h>) -> Result<Entry> {
        // The first block of a run was admitted by `advance` before this
        // frontend existed, and admitting it twice would key it twice.
        if core::mem::take(&mut self.admitted) {
            return Ok(Entry::Ready);
        }
        let pc = pc as u32;
        // Nothing is counted on either refusal: the run ends here, the guest
        // is left standing on the instruction, and `advance`'s prologue is
        // what finds the same cause on the next call and hands it to the
        // interpreter. Counting here as well would count it twice.
        if unliftable_at(host.state, self.cfg, host.lines, pc).is_some() {
            return Ok(Entry::Leave);
        }
        // The queue at a chained boundary is in the host's slots, not in
        // `State`: registers stay in slots between the blocks of a chain and
        // are written back only when the run ends.
        self.queue = [host.slots[P0_INDEX] as u16, host.slots[P1_INDEX] as u16];
        self.key = cache_key(self.cfg.model, self.queue);
        if self.unlifted.holds(pc, self.key).is_some() {
            return Ok(Entry::Leave);
        }
        // Whatever this boundary resolves to, the block that runs after it is
        // a different block in a different window, so the window the store
        // guard compares against is replaced rather than kept.
        host.code_window = pc & !lift::WINDOW_MASK;
        Ok(Entry::Ready)
    }

    fn key(&mut self) -> u64 {
        self.key
    }

    fn pc_slot(&self) -> RegSlot {
        lift::PC
    }

    fn translate(&mut self, pc: u64) -> Result<Translation> {
        let pc = pc as u32;
        let mut reader = Reader {
            space: self.space,
            attrs: self.attrs,
            mask: self.mask,
            pc,
            queue: self.queue,
        };
        let lifted = lift::lift(self.cfg.model, pc, &mut reader, lift::MAX_INSNS)?;
        if verify(&lifted.block).is_err() {
            self.rejected = true;
        }
        if lifted.insns == 0 {
            // A lift that produced nothing is remembered, which is what sends
            // the next pass straight to the interpreter instead of back
            // through here — and so is *why*, which is what makes the fallback
            // attributable. `Stop::Unreadable` on the very first word is not
            // about the encoding at all: the PC points at memory that answers
            // nothing, so the interpreter's own fetch is what takes the bus
            // error.
            let declined = lifted.declined.unwrap_or(Declined {
                reason: Decline::STATE,
                what: "unreadable",
            });
            let row = row_for(self.declines, declined);
            self.unlifted.note(pc, self.key, row);
            self.declined_row = Some(row);
        }
        Ok(Translation {
            page: lifted_page(pc, self.mask),
            insns: lifted.insns,
            block: lifted.block,
        })
    }
}

/// The lifter's window onto guest memory.
struct Reader<'a> {
    space: &'a AddressSpace,
    attrs: MemAttrs,
    mask: u32,
    /// The block's entry PC, which is where the prefetch queue answers.
    pc: u32,
    /// `State::prefetch` as the core holds it.
    queue: [u16; 2],
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
        // the right source for it. The two queue words are in the cache key
        // instead (`cache_key`).
        if addr == self.pc {
            return Some(self.queue[0]);
        }
        if addr == self.pc.wrapping_add(2) {
            return Some(self.queue[1]);
        }
        let at = u64::from(addr & self.mask);
        Some(self.space.read(at, Width::U16, self.attrs).ok()? as u16)
    }
}

// ---------------------------------------------------------------------------
// The host
// ---------------------------------------------------------------------------

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
    /// Ticks charged when the last guest instruction boundary was reached — so
    /// `used - retired` is exactly what a partial instruction spent.
    ///
    /// Tracked here rather than read off [`Fault`](crate::ir::Fault), and that
    /// is the fix for a real defect rather than a preference.
    /// `Fault::charged_ticks` and `Fault::retired_ticks` are both measured
    /// from the backend's own counter, which sees
    /// [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) immediates and **not**
    /// the ticks a host charges inside an access. On a core whose every bus
    /// cycle is four host-charged ticks that is most of the count: the first
    /// version of the restart path unwound a faulting `ADD.W (d8,An,Xn),Dn` by
    /// the two internal cycles its `CHARGE` carried and left the eight its two
    /// bus cycles had spent, so the guest's cycle counter ran eight ahead of
    /// the interpreter's for the rest of the run.
    retired: u64,
    /// Stores this guest instruction has committed. The restartability budget
    /// is one (module docs).
    stores: u32,
    /// A store landed in the running block's own window, so the block must
    /// leave at the next guest instruction boundary.
    smc: bool,
    /// The interrupt mask the run started with. **S**, **T** and the interrupt
    /// mask cannot change under a lifted block, so this is constant for the
    /// run and is read once rather than out of a slot a block may have
    /// rebound.
    ipl_mask: u8,
    /// The [`lift::WINDOW`] the running block's instructions came from,
    /// replaced at every block boundary by [`Lifter::enter`].
    code_window: u32,
    /// The guest-physical pages this run's blocks have written, for the
    /// dispatcher to drain against the block cache at the next boundary.
    dirty: DirtyPages,
}

impl<'a> Host<'a> {
    fn new(
        state: &'a mut State,
        space: &'a AddressSpace,
        cfg: &Config,
        lines: &'a Lines,
        allowance: u64,
        pc: u32,
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
            code_window: pc & !lift::WINDOW_MASK,
            dirty: DirtyPages::new(),
        }
    }

    /// One bus access: four clocks, charged where `Exec::read_byte` and
    /// friends charge them — *after* the alignment check and *before* the
    /// access, so an address error costs nothing and a refused access costs
    /// four.
    #[inline]
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
    #[inline]
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
        // On the **physical** address, which is the address a translation was
        // lifted from: the dispatcher matches this against `Translation::page`
        // at the next block boundary.
        self.dirty.note(at, mem.size.bytes());
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

    #[inline]
    fn spent(&self) -> bool {
        // Monotone in each term, which is what the seam requires: ticks only
        // rise, the store guard only latches, and an interrupt only becomes
        // pending. `interrupt_pending` is the non-consuming form on purpose —
        // see `Lines::interrupt_pending`.
        self.used >= self.allowance || self.smc || self.lines.interrupt_pending(self.ipl_mask)
    }
}

impl StoreLog for Host<'_> {
    fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64)) {
        self.dirty.drain_dirty(sink);
    }
}

/// Nothing is inlined, and that is a measurement rather than an omission.
///
/// [`FastMem`] lets a backend serve an aligned load out of the software TLB
/// without calling back into the host, and `ROADMAP.md` §9.1 makes that its
/// first mechanism. A 68000 has no memory management unit, so the table would
/// be exact — and `AddressSpace::read` is 41% of what this core's half of a
/// Macintosh Plus boot costs, so there is something there to take.
///
/// What bounds it is the **board**, not the core: 29.9% of that board's guest
/// accesses are decoded by `mac.glue`, which is a `MemOps` device forwarding
/// into a private space of its own, and `jit::Tlb::fill` admits plain RAM
/// leaves and nothing else. So the ceiling is the other 70%, reached by a
/// table that has to be filled, synchronised at every boundary and probed on
/// every access. It is the next thing to measure, and it is not this change.
impl FastMem for Host<'_> {}

#[cfg(test)]
mod tests {
    use super::super::differential::{CODE, Case, DATA, compare, measure, stats_for};
    use super::super::lift::Decline;
    use super::super::{Engine, M68k};
    use super::{CHAIN, lift};
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
        let props = Props::new().with("engine", Value::Str("jit".into()));
        let cpu = M68k::from_props(&props).expect("`jit` is a 68000 engine");
        assert_eq!(cpu.engine(), Engine::Jit);
        let props = Props::new().with("engine", Value::Str("jit-host".into()));
        let cpu = M68k::from_props(&props).expect("`jit-host` is a 68000 engine");
        assert_eq!(cpu.engine(), Engine::JitHost);
        let props = Props::new().with("engine", Value::Str("interp".into()));
        let cpu = M68k::from_props(&props).expect("`interp` still works");
        assert_eq!(cpu.engine(), Engine::Interp);
    }

    #[test]
    fn a_nonsense_engine_is_refused_rather_than_ignored() {
        let props = Props::new().with("engine", Value::Str("ir".into()));
        M68k::from_props(&props).expect_err("an engine that does not exist is an error");
    }

    /// The three four-kilobyte constants this engine equates are one number.
    ///
    /// `lift::WINDOW` bounds a block, `jit::PAGE_SIZE` is what the block
    /// cache's store filter matches on, and `exec::WRITE_PAGE` is what an
    /// interpreted store is recorded at. A change to any one of them without
    /// the others would leave a store landing in one page and a translation
    /// remembered in another, which no test above this would catch.
    #[test]
    fn the_block_window_and_the_store_filters_page_are_the_same_page() {
        assert_eq!(u64::from(lift::WINDOW), crate::jit::PAGE_SIZE);
        assert_eq!(lift::WINDOW, super::super::exec::WRITE_PAGE);
    }

    #[test]
    fn a_core_that_never_ran_has_no_statistics_and_one_that_did_has_some() {
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Jit);
        assert_eq!(cpu.jit_stats(), None, "nothing has run yet");
        let stats = stats_for(&Case::seeded(vec![0x4e71, STOP[0], STOP[1]]).with_units(3))
            .expect("a core that ran has statistics");
        assert!(stats.executed > 0, "{stats:?}");
        assert!(stats.retired > 0, "{stats:?}");
    }

    #[test]
    fn a_pending_interrupt_keeps_a_block_from_running_at_any_boundary() {
        // `unliftable_at` refuses a block while one is pending, and
        // `Host::spent` leaves the block at the next boundary if one arrives
        // inside it — because the interpreter checks for one before every
        // instruction, and the two engines have to take it at the same one.
        let program = vec![0x4e71, 0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(4);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Jit);
        cpu.attach_space(space);
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        // An interrupt mask of 3 and a level of 5, so the request is above it.
        regs.sr = super::super::flags::S | 0x0300;
        cpu.set_regs(regs);
        cpu.set_ipl(5);
        let before = cpu.jit_stats().unwrap_or_default();
        cpu.step();
        let after = cpu.jit_stats().unwrap_or_default();
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
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Jit);
        cpu.attach_space(space);
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.run_budget(4);
        assert_eq!(cpu.regs().pc, CODE + 2, "one NOP and no more");
        let stats = cpu.jit_stats().expect("it ran");
        assert_eq!(stats.spent, 1, "the run left at a boundary: {stats:?}");
    }

    #[test]
    fn a_topology_change_throws_the_translations_away() {
        // Derived state, invalidated by the generation counter (CLAUDE.md,
        // "Devices"). A block lifted before a remap must not survive it.
        let program = vec![0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(3);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Jit);
        cpu.attach_space(alloc::sync::Arc::clone(&space));
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.step();
        let before = cpu.jit_stats().expect("it ran").lifted;
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
        let after = cpu.jit_stats().expect("it ran").lifted;
        assert!(
            after > before,
            "the cache was not thrown away: {before} {after}"
        );
    }

    #[test]
    fn a_store_the_guest_makes_into_a_translated_window_invalidates_it() {
        // The store filter, end to end: `Host::store` records the page, the
        // dispatcher drains it at the next boundary, and the translation of
        // that page is dropped rather than served again.
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

    /// The other half of the store filter: an **interpreted** instruction's
    /// store has to reach it too.
    ///
    /// `MOVEM.L D0-D1,(A2)` is outside the lifted subset — it commits one
    /// store per register — so nothing in the block path sees it. Without
    /// `Exec::wrote` and [`drain`](super::drain) it would rewrite a
    /// translation and leave it cached, which is the one bug this mechanism
    /// exists for.
    #[test]
    fn a_store_an_interpreted_instruction_makes_invalidates_a_translation() {
        //   CODE+0: 4e71        NOP                  -- lifted, so a block is cached
        //   CODE+2: 48d2 0003   MOVEM.L D0-D1,(A2)   -- declined, and A2 is CODE
        //   CODE+6: 60f8        BRA CODE
        //
        // `D0:D1` hold the eight bytes at `CODE` — `4e71 48d2 0003 60f8` — so
        // the `MOVEM` writes the loop back over itself unchanged and the guest
        // goes round again. The bytes not changing is the point: what is being
        // tested is that the *cache* notices the write, not that the program
        // does something different afterwards.
        let program = vec![
            0x4e71, // NOP
            0x48d2, 0x0003, // MOVEM.L D0-D1,(A2)
            0x60f8, // BRA CODE
            STOP[0], STOP[1],
        ];
        let case = Case::seeded(program)
            .with_a(2, CODE)
            .with_d(0, 0x4e71_48d2)
            .with_d(1, 0x0003_60f8)
            .with_units(24);
        agreed(&case);
        let stats = stats_for(&case).expect("it ran");
        assert!(
            stats.invalidated_interpreted > 0,
            "an interpreted store must reach the block cache: {stats:?}"
        );
    }

    /// A space swapped under the core takes every translation with it.
    ///
    /// The third of the three ways guest memory changes without this engine
    /// hearing about it, and the one neither counter covers: a topology
    /// generation belongs to *one* space, and the store filter only hears
    /// about stores this core made. A snapshot restore is the fourth and is
    /// the same call — `M68k::load` flushes for exactly this reason — and a
    /// reset is the fifth.
    #[test]
    fn a_space_swapped_under_the_core_throws_the_translations_away() {
        let program = vec![0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(3);
        let (first, _ram) = super::super::differential::space_for(&case);
        let (second, _ram2) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Jit);
        cpu.attach_space(first);
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.step();
        let before = cpu.jit_stats().expect("it ran").lifted;
        assert!(before > 0);
        cpu.attach_space(second);
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.step();
        let after = cpu.jit_stats().expect("it ran").lifted;
        assert!(
            after > before,
            "the cache survived a new address space: {before} {after}"
        );
    }

    #[test]
    fn a_reset_throws_the_translations_away_and_the_run_still_agrees() {
        let program = vec![0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(3);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Jit);
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

    /// A chain runs several blocks per call, and stops at [`CHAIN`] of them.
    ///
    /// The number the safe point is stated as (`ROADMAP.md` §4.7): a raised
    /// exit flag is honoured within one chain, so the bound has to be a bound
    /// rather than "as many as the budget allowed".
    #[test]
    fn one_call_runs_a_chain_and_stops_at_the_stated_bound() {
        // A two-instruction loop, so a chain is the only way to run more than
        // one block in a call.
        let program = vec![0x4e71, 0x60fc, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(1);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Jit);
        cpu.attach_space(space);
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x60fc];
        cpu.set_regs(regs);
        cpu.step();
        let stats = cpu.jit_stats().expect("it ran");
        assert!(
            stats.executed > 1,
            "one call ran {} blocks, so nothing chained: {stats:?}",
            stats.executed
        );
        assert!(
            stats.executed <= CHAIN as u64,
            "one call ran {} blocks, and the safe point is stated as {CHAIN}",
            stats.executed
        );
        assert!(
            stats.chained > 0,
            "a patched exit was never followed: {stats:?}"
        );
    }

    // -- the instrument -------------------------------------------------
    //
    // A counter whose own arithmetic is untested is a counter that will be
    // quoted in a document and be wrong. These are the properties the
    // measurement rests on, and the first is a *closure* property: every
    // fallback is attributed to a row, and the rows sum to `interpreted`.

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

    /// Every executed guest instruction is in exactly one of three columns.
    ///
    /// `retired + interpreted + faults` is the divisor of the lift rate, so a
    /// column that double-counts or drops an instruction moves the published
    /// number without moving anything that explains it.
    #[test]
    fn every_executed_instruction_is_retired_interpreted_or_restarted() {
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
        assert_eq!(
            stats.steps,
            stats.retired + stats.interpreted + stats.faults,
            "{stats:?}"
        );
        assert!(stats.retired > 0, "a block carried the loop: {stats:?}");
        assert!(stats.chained > 0, "and the loop chained: {stats:?}");
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
