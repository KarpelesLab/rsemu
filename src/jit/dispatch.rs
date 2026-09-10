//! The dispatcher: the loop that keeps a guest inside translated code.
//!
//! Lift on a miss, cache under `(pc, key)`, patch the exit to its successor,
//! and go round again without leaving for the interpreter. This is where the
//! three mechanisms in [`jit`](super) meet, and it is deliberately the only
//! place that knows the order they run in.
//!
//! # What is generic, and what a guest supplies
//!
//! Nothing here knows what a RISC-V is. A guest supplies a [`Frontend`] —
//! which world it is in ([`Frontend::key`], [`Frontend::epoch`]), how to lift
//! one block ([`Frontend::translate`]), what a block owes before it runs
//! ([`Frontend::enter`]), and which slot the guest PC lands in at a block exit
//! ([`Frontend::pc_slot`]) — and an [`IrHost`](crate::ir::IrHost) that also
//! implements [`StoreLog`], so guest writes can be matched against cached
//! translations.
//!
//! [`Frontend::enter`] is the one that makes chaining reachable, and it is
//! worth saying why a *cache* needed a hook at all. Following a patched exit
//! skips the hash lookup, which was never the expensive part; what it really
//! skips is everything a caller does *between* two `Dispatcher::run` calls.
//! But a guest whose instruction fetch translates owes that translation on
//! every block execution — a cached block that skipped it would cost fewer
//! ticks than the uncached one it replaced — so before this hook existed the
//! only honest budget was one block, and `DispatchStats::chained` was zero in
//! every run of a real guest. Now it is called once per block iteration, the
//! chained ones included.
//!
//! # A link: when the patched exit reaches generated code
//!
//! The paragraph above is about *chaining*, which is a cache lookup skipped.
//! A **link** is the same edge taken by the host processor: the predecessor's
//! compiled code jumps into the successor's, and the Rust frame the block was
//! entered through is never unwound. That is direct block linking, the
//! technique every production translator uses — Bala, Duesterwald and
//! Banerjia's Dynamo (PLDI 2000) calls it *fragment linking*, and Smith and
//! Nair's *Virtual Machines* (2005) §2.6 sets it out as "translation chaining"
//! — and `ROADMAP.md` §9.1's second mechanism is only half done without it.
//!
//! What it removes is not the hash lookup. It is the **re-entry**: a
//! twenty-eight-field context, a thunk table, a six-register prologue and its
//! epilogue, and the dispatcher's own call into an engine, once per block.
//!
//! What it may **not** remove is everything else this loop does at a boundary,
//! and that is the whole design. [`Chain`] is where those checks moved, in the
//! order this loop makes them, and its documentation is the list. Two of them
//! are the ones a naive link gets wrong: a chain that never returns to Rust
//! cannot be preempted, so [`IrHost::spent`](crate::ir::IrHost::spent) and the
//! [`ExitFlag`] are asked at every linked boundary too; and a block a guest
//! store invalidated must not be jumped into, so stores are drained against
//! the cache *before* the successor is chosen.
//!
//! A link is refused — and the run comes back here — whenever the successor
//! needs something a chain may not do: lifting, compiling, or a longer
//! temporary frame. [`DispatchStats::linked`] counts the ones that were taken,
//! separately from [`DispatchStats::chained`], because the two were the same
//! number for as long as chaining meant only a lookup skipped.
//!
//! # Why self-modifying code is reported rather than intercepted
//!
//! A guest store goes through [`IrHost::store`](crate::ir::IrHost::store),
//! which the dispatcher never sees, and putting the block cache behind a lock
//! so the store path could reach it would put a lock on the one path that
//! cannot afford one. So a host **accumulates** the guest-physical pages it
//! wrote ([`DirtyPages`] is a ready-made accumulator) and the dispatcher
//! drains them at each block boundary.
//!
//! Draining at a boundary rather than at the store is the granularity RISC-V
//! asks for, and now the *only* one available: the ISA requires a `FENCE.I`
//! between a store to instruction memory and executing it, so a store's effect
//! on **later** blocks is all it promises. That used to be belt and braces —
//! the lifter ended a block at its first access, so nothing after a store in
//! the same block existed to be modified — and superblocks spend the braces: a
//! trace runs to its end on the bytes it was lifted from, and a store it made
//! into its own page invalidates it for the *next* execution. A guest that
//! wants otherwise owes a `FENCE.I`. An x86 frontend needs the check *within*
//! a block — x86 makes coherent instruction caches architectural — and will
//! need a finer hook than this one; that is recorded here rather than
//! discovered later.
//!
//! # Two budgets, in two currencies
//!
//! [`Dispatcher::run`] takes a budget in **blocks**, which is the dispatcher's
//! own and bounds the loop. It is not the budget a CPU core has: a core is
//! handed a quantum in *ticks*, and until this paragraph existed the only way
//! to spend one safely was for a frontend to prove, at every boundary, that
//! the block it was about to enter could not overrun what was left — a block
//! runs to a terminator once started, so its **worst case** had to fit.
//!
//! Measured on `pc64` — nine hundred guest seconds of a 6.6 kernel — that
//! guard was 2.02% of all guest instructions, three times everything outside
//! the lifted subset put together, and it could not be tightened: admitted
//! blocks were bounded at a mean of 185 ticks and refused ones at 1 636
//! against a mean 827 left, so it is a small population of fat blocks and not
//! ordinary blocks caught in a tail.
//!
//! The second currency is therefore the host's.
//! [`IrHost::spent`](crate::ir::IrHost::spent) is asked here at each block
//! boundary and by the backends at each guest instruction boundary, and a
//! `true` stops the run with [`Stop::Spent`] — at an instruction boundary,
//! with the guest's state published, exactly where a core's own interpreter
//! would have stopped. A frontend can then admit a block it is not sure fits,
//! which is the whole of it: the guard stops refusing, and a block that
//! overruns leaves instead.
//!
//! The default answer is `false`, so a host that says nothing gets the
//! behaviour it had: a run bounded only by blocks — and that default costs
//! **0.29%** of host instructions, measured by callgrind over the same 120
//! guest seconds of `pc64`, which is the `test`/`jcc` `jit::x86` now emits
//! after each region's flush.
//!
//! What it buys, on the same board and the same nine hundred guest seconds the
//! table above profiled: **95.4% of guest instructions retiring inside a block
//! becomes 97.3%**, the interpreted remainder falls from 83.5 M to 49.1 M, and
//! the run goes from a median 89.4 s of wall clock to 76.3 s over three
//! interleaved reps. Under callgrind, on a 120-second window where the guard
//! is the only thing still refusing blocks, adopting it is **−9.4%** of host
//! instructions.
//!
//! Two things a host must get right before it may say `true`, both of which
//! were found by a guest that stopped booting rather than by a test:
//! [`IrHost::spent`](crate::ir::IrHost::spent) states them.
//!
//! # Safe points
//!
//! A [`Dispatcher`] carrying an [`ExitFlag`] tests it at each block boundary
//! and stops with [`Stop::Exit`]. That is §4.7's protocol exactly: a
//! generation counter plus a per-CPU flag checked at block boundaries, never a
//! signal, because wasm has none.
//!
//! A trace has *fewer* boundaries than the basic blocks it replaces, so the
//! delay before a raised flag is honoured is bounded by a frontend's own
//! instruction limit rather than by a basic block's length — sixty-four guest
//! instructions for the RISC-V frontend. That is the price of merging, it is
//! bounded, and it is checked by
//! `a_raised_exit_flag_stops_within_one_block_however_long_the_block_is`.

use alloc::vec::Vec;

use crate::core::error::Result;
use crate::core::sched::ExitFlag;
use crate::ir::{Block, Fault, Interp, IrHost, Opcode, Outcome, RegSlot};
use crate::jit::cache::{BlockCache, BlockId, CacheStats};
use crate::jit::fast::FastMem;
use crate::jit::tlb::{Epoch, PAGE_MASK, PAGE_SIZE};

#[cfg(any(
    all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
    all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
))]
use core::marker::PhantomData;

#[cfg(any(
    all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
    all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
))]
use crate::core::error::Error;
#[cfg(any(
    all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
    all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
))]
use crate::jit::cache::CodeRef;
#[cfg(any(
    all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
    all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
))]
use crate::jit::host::{Engine, Linkage};

/// One freshly lifted block, and what the cache needs to know about it.
#[derive(Debug)]
pub struct Translation {
    /// The block.
    pub block: Block,
    /// The guest-**physical** page its bytes were read from.
    ///
    /// Physical, not virtual: a guest write is matched against this, and a
    /// write arrives at a physical address. A block never leaves the page it
    /// started on, so one page is the whole answer.
    pub page: u64,
    /// How many guest instructions the block covers.
    ///
    /// Zero means the frontend could not lift the instruction at the entry PC,
    /// and the dispatcher stops with [`Stop::Untranslatable`] rather than
    /// spinning on a block that cannot advance the PC.
    ///
    /// A **static** count, and not the one [`Run::insns`] reports: a
    /// superblock covers every instruction on the path it inlined, and a run
    /// that leaves through a side exit retires fewer of them. What retired is
    /// counted by [`Interp::boundaries`](crate::ir::Interp::boundaries).
    pub insns: usize,
}

/// What a frontend says when a block boundary is reached.
///
/// The answer to [`Frontend::enter`]. `Leave` is not an error and not a fault:
/// it is a guest whose *next* block should not run — the instruction at that
/// PC is outside the lifted subset, or the block's worst case does not fit
/// what is left of the caller's budget, or entering it trapped. The dispatcher
/// stops with [`Stop::Declined`], and what that means is the frontend's own
/// business; for a CPU core it means its own interpreter takes over.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Entry {
    /// Execute the block at this PC.
    Ready,
    /// Do not. Stop the run here, with the guest at this PC.
    Leave,
}

/// What a dispatcher needs from a guest.
///
/// Generic over the host, and *only* so that [`Frontend::enter`] can be handed
/// it. Entering a block is a hart action — it translates the entry fetch, it
/// may walk a page table, it charges for the walk and it can trap — and the
/// state all four of those need belongs to the [`IrHost`], not to the lifter.
/// Every other method ignores the parameter, which is why a frontend with
/// nothing to do at a boundary reads `impl<H> Frontend<H> for …`.
pub trait Frontend<H: ?Sized> {
    /// The counters this guest's translations are stale against.
    ///
    /// Read at **every** block boundary, not once per [`Dispatcher::run`], so
    /// a stop-the-world retopology is observed before the next block rather
    /// than after the chain that followed it. It is therefore on the hot path:
    /// answer it with an atomic load, not with a lock or a walk.
    fn epoch(&mut self) -> Epoch;

    /// Everything a block owes *before* it runs, on every execution.
    ///
    /// Called once per block iteration — including the chained ones, which is
    /// the whole reason it exists. A translated block skips its own entry
    /// fetch, but a guest whose fetch *translates* still owes that translation
    /// every time the block runs: a cached block that skipped it would cost
    /// fewer ticks than the uncached one it replaced, and the two engines
    /// would stop agreeing on the cycle counter (`ROADMAP.md` §0). Without a
    /// hook here a dispatcher can only be driven one block at a time, which is
    /// what gave up §9's second mechanism — `chained: 0` in every run of a
    /// real guest, on the one machine the JIT exists for.
    ///
    /// It runs **after** the budget and safe-point checks and **before**
    /// [`Frontend::key`], so a frontend may compute the key here out of what
    /// the entry resolved to. The RISC-V engine does exactly that: a block is
    /// keyed on the physical page its entry translation just produced.
    ///
    /// The default is [`Entry::Ready`]. A guest with nothing to do at a
    /// boundary says so by not implementing this.
    ///
    /// # Errors
    ///
    /// Whatever the frontend says. A dispatcher does not try to recover — an
    /// ordinary guest condition is [`Entry::Leave`] rather than an error.
    fn enter(&mut self, pc: u64, host: &mut H) -> Result<Entry> {
        let _ = (pc, host);
        Ok(Entry::Ready)
    }

    /// The rest of the cache key beside the guest PC — the value the frontend
    /// puts in [`Block::key`](crate::ir::Block::key).
    fn key(&mut self) -> u64;

    /// The slot a block leaves the guest PC in at its exit boundary.
    fn pc_slot(&self) -> RegSlot;

    /// Lift the block at `pc`.
    ///
    /// # Errors
    ///
    /// Whatever the frontend says. A dispatcher does not try to recover.
    fn translate(&mut self, pc: u64) -> Result<Translation>;
}

/// A host that reports which guest-physical pages its stores touched.
///
/// The self-modifying-code half of the contract. A host that cannot write
/// guest memory implements this as an empty method.
pub trait StoreLog {
    /// Hand over the pages stored to since the last call, and forget them.
    fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64));
}

/// A ready-made accumulator a host can embed to satisfy [`StoreLog`].
///
/// Records pages, not addresses, and de-duplicates against the most recent —
/// a guest memcpy walks one page for hundreds of stores, and a list with one
/// entry per store would be the expensive part of the mechanism.
#[derive(Debug, Clone, Default)]
pub struct DirtyPages {
    pages: Vec<u64>,
}

impl DirtyPages {
    /// An empty log.
    #[must_use]
    pub fn new() -> DirtyPages {
        DirtyPages::default()
    }

    /// Record a store of `len` bytes at guest-physical `phys`.
    #[inline]
    pub fn note(&mut self, phys: u64, len: u64) {
        if len == 0 {
            return;
        }
        let first = phys & !PAGE_MASK;
        let last = phys.saturating_add(len - 1) & !PAGE_MASK;
        let mut page = first;
        loop {
            if self.pages.last() != Some(&page) {
                self.pages.push(page);
            }
            if page >= last {
                break;
            }
            page = page.saturating_add(PAGE_SIZE);
        }
    }

    /// Whether anything has been recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pages.is_empty()
    }
}

impl StoreLog for DirtyPages {
    fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64)) {
        for page in self.pages.drain(..) {
            sink(page);
        }
    }
}

/// Why a run stopped.
#[derive(Debug, Clone, PartialEq, Eq)]
#[non_exhaustive]
pub enum Stop {
    /// The block budget ran out. The guest is mid-flight and the PC is live.
    Budget,
    /// The **tick** budget ran out: [`IrHost::spent`](crate::ir::IrHost::spent)
    /// answered `true`, either at a block boundary or inside a block.
    ///
    /// A different currency from [`Stop::Budget`], and a different owner: the
    /// count of blocks is the dispatcher's, and the ticks are the host's, so
    /// only the host can say when they are gone. The guest is left at
    /// [`Run::pc`], which is a guest instruction boundary that has not
    /// started, and everything before it has retired.
    Spent,
    /// The safe-point flag was raised (`ROADMAP.md` §4.7).
    Exit,
    /// A guest access faulted. The guest's own fault path takes it from here.
    Fault(Fault),
    /// The block reached an op this backend does not implement.
    Unsupported {
        /// The op.
        op: Opcode,
        /// Its index in the block.
        at: usize,
    },
    /// The frontend could not lift the instruction at this PC, so the guest's
    /// own interpreter has to execute it.
    Untranslatable {
        /// The guest PC.
        pc: u64,
    },
    /// [`Frontend::enter`] answered [`Entry::Leave`]: the block at
    /// [`Run::pc`] was not entered, and why is the frontend's own business.
    Declined,
}

/// What a run did.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Run {
    /// The guest PC to resume at.
    pub pc: u64,
    /// How many blocks executed.
    pub blocks: usize,
    /// How many guest instructions those blocks **retired**.
    ///
    /// Counted from the boundaries the backend actually passed, not summed
    /// from [`Translation::insns`]: a trace that leaves through a side exit
    /// retires fewer instructions than it covers, and a block that faulted
    /// retires everything before the faulting instruction and no more. A
    /// caller that steps an oracle this many times — the differential harness
    /// does — gets a wrong answer from the static number and a right one from
    /// this.
    pub insns: usize,
    /// Why it stopped.
    pub stop: Stop,
}

/// What a dispatcher has been asked to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DispatchStats {
    /// Blocks executed.
    pub blocks: u64,
    /// Blocks reached by following a patched exit.
    pub chained: u64,
    /// Blocks reached by a hash lookup.
    pub looked_up: u64,
    /// Blocks entered by a **direct link**: a jump from the predecessor's own
    /// compiled code, with no return to this loop at all.
    ///
    /// A subset of [`DispatchStats::chained`], and the one that says whether
    /// the patch reached generated code. A block is counted here only when
    /// everything a boundary owes was answered inside the chain — see
    /// [`Chain`] — so the difference between the two is exactly the population
    /// a link could not serve: a successor not yet compiled, one compiled in
    /// an older code-buffer generation, or one needing a longer temporary
    /// frame than the chain is standing on.
    pub linked: u64,
    /// Blocks translated.
    pub translated: u64,
    /// Blocks invalidated by a guest store.
    pub smc: u64,
    /// Times the epoch moved and the caches were resynchronised.
    pub resyncs: u64,
    /// Blocks executed as compiled host code rather than interpreted.
    ///
    /// The two engines are indistinguishable to the guest, so this is a
    /// statistic and never a behaviour — but a backend whose coverage is
    /// unmeasured is a backend whose coverage rots, which is why it is
    /// counted rather than assumed.
    pub compiled: u64,
}

/// A block boundary a chain got past and could not follow.
///
/// The reason this is not simply "go round the loop again": by the time a
/// chain gives up, [`Frontend::enter`] has already run for `pc` — it had to,
/// because [`Frontend::key`] is answered out of what the entry resolved to —
/// and on a guest whose fetch translates that is a page-table walk, charged in
/// ticks. Going round the top of the loop would charge it a second time, which
/// is the shape of bug `cpu::arm::a64::engine` documents at its own prologue.
/// So [`Dispatcher::run`] resumes *below* the boundary work with whatever the
/// chain already established.
#[derive(Debug, Clone, Copy)]
struct Resumed {
    /// The guest PC the entry work was done for.
    pc: u64,
    /// What [`Frontend::key`] answered there.
    key: u64,
    /// The predecessor a link would be patched from.
    from: Option<BlockId>,
    /// The block the cache held, and whether a patched exit found it.
    ///
    /// `None` means the cache had none and the loop must lift one. `Some`
    /// means the hit is counted and the predecessor patched already.
    found: Option<(BlockId, bool)>,
}

/// What executing one block — or a whole chain of them — did.
#[derive(Debug)]
struct Ran {
    /// The last block's outcome.
    outcome: Outcome,
    /// How many blocks executed.
    blocks: usize,
    /// What they retired.
    insns: usize,
    /// Whether the boundary after the last block has been dealt with here.
    ///
    /// True only for a chain: the drain, the epoch, the entry work and the
    /// lookup all happen inside generated code, and the loop must not do them
    /// again. `resumed` and `stopped` are what came of them.
    closed: bool,
    /// The boundary the chain stopped at, when it stopped at one it could not
    /// follow.
    resumed: Option<Resumed>,
    /// The chain ending the whole run, and where the guest is.
    stopped: Option<(Stop, u64)>,
}

/// The policy behind a direct link: what happens at a block boundary reached
/// **inside** generated code.
///
/// `ROADMAP.md` §9.1's second mechanism carried to its end. The block cache
/// has patched exits since it existed, but following one still meant returning
/// to Rust and entering the successor through `Dispatcher::execute` — a
/// context of twenty-eight fields, a thunk table, a prologue and an epilogue
/// per block, measured at ~490 host instructions against a mean block of 10.8
/// guest instructions on an AArch64 Linux boot. A link makes the successor a
/// jump from the predecessor's own code, and this type is everything that jump
/// may not skip.
///
/// # What a link must not break
///
/// Every one of these is a check [`Dispatcher::run`]'s loop makes at a block
/// boundary, and [`Chain::step`] makes them in the same order for the same
/// reasons:
///
/// * **The tick allowance.** [`IrHost::spent`] decides where a quantum ends,
///   and a chain that never returned to Rust could not be preempted. It is
///   asked here, at every boundary, exactly where the loop asks it.
/// * **The safe point.** A raised [`ExitFlag`] is honoured within one block
///   (see the module docs) — including a block reached by a link.
/// * **Invalidation.** A guest store is drained against the cache *before* the
///   successor is chosen, so a block a store invalidated is never jumped to;
///   and the epoch is read at every boundary, so a retopology flushes the
///   cache before the link that would otherwise have followed it.
/// * **Staleness.** A [`CodeRef`] from before a code-buffer reset names
///   whatever took its place, so a link is refused unless the reference
///   carries the generation the engine is serving.
/// * **Determinism.** Nothing above is skipped and nothing is reordered, so a
///   linked chain retires exactly what an unlinked one does: the same faults
///   at the same instructions, the same tick counts, the same oracle.
///
/// # What it may not do
///
/// **Translate, compile, or grow the temporary frame.** All three would run
/// while the engine is executing: compiling can reset the code buffer under
/// the code that called this, and growing the frame moves it out from under
/// the register holding its address. A successor needing any of them is not
/// linked to — the chain stops there, and the loop, which may do all three,
/// enters it the ordinary way.
#[cfg(any(
    all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
    all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
))]
#[derive(Debug)]
pub struct Chain<'a, F: ?Sized, H: ?Sized> {
    cache: &'a mut BlockCache,
    front: &'a mut F,
    stats: &'a mut DispatchStats,
    exit: Option<&'a ExitFlag>,
    /// What the engine's code buffer looks like for as long as this runs.
    link: Linkage,
    /// The slot a block leaves the guest PC in.
    pc_slot: RegSlot,
    /// How many blocks the caller's budget still allows.
    remaining: usize,
    /// The block currently executing.
    id: BlockId,
    /// The predecessor of the next link.
    from: Option<BlockId>,
    /// Blocks this chain executed.
    blocks: usize,
    /// What they retired.
    insns: usize,
    /// The boundary it could not follow.
    resumed: Option<Resumed>,
    /// The run it ended outright.
    stopped: Option<(Stop, u64)>,
    /// What [`Frontend::enter`] said, when it said an error.
    err: Option<Error>,
    _host: PhantomData<*mut H>,
}

/// What [`Chain::step`] answers.
#[cfg(any(
    all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
    all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// Leave generated code. The [`Chain`] holds why.
    Leave,
    /// Jump to a successor's chain entry.
    Go {
        /// The block there.
        id: BlockId,
        /// Its compiled code.
        code: CodeRef,
        /// The host address of its chain entry.
        entry: u64,
    },
}

#[cfg(any(
    all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
    all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
))]
impl<F, H> Chain<'_, F, H>
where
    F: Frontend<H> + ?Sized,
    H: IrHost + StoreLog + FastMem + ?Sized,
{
    /// The slot a block leaves the guest PC in, for the backend that has to
    /// read an `exit_tb`'s successor out of it.
    #[inline]
    #[must_use]
    pub fn pc_slot(&self) -> RegSlot {
        self.pc_slot
    }

    /// The block an id names, for the backend that has to point its context at
    /// one.
    #[inline]
    #[must_use]
    pub fn block(&self, id: BlockId) -> Option<&Block> {
        self.cache.block(id)
    }

    /// One block boundary, reached inside generated code.
    ///
    /// `next` is where the guest is going, or `None` when the block did not
    /// reach a terminator — a fault, or the tick allowance — and `retired` is
    /// what that block retired. Answers with the successor to jump to, or
    /// [`Step::Leave`].
    ///
    /// This is [`Dispatcher::run`]'s loop body from *"guest stores land before
    /// the next block is chosen"* through the link patch, **moved**: the same
    /// work in the same order, which is the point. See the type's docs.
    pub fn step(&mut self, next: Option<u64>, retired: usize, host: &mut H) -> Step {
        self.stats.blocks += 1;
        self.blocks += 1;
        self.insns += retired;

        // Guest stores land before the next block is chosen, so a block
        // invalidated by one is never served afterwards — and never jumped to,
        // which is the same rule one level down.
        let cache = &mut *self.cache;
        let mut hit = 0usize;
        host.drain_dirty(&mut |page| hit += cache.note_write(page, 1));
        self.stats.smc += hit as u64;
        // A block that wrote into its own page is gone, and the id that named
        // it may already have been reused, so it cannot be a link's
        // predecessor.
        self.from = self.cache.block(self.id).is_some().then_some(self.id);

        let Some(pc) = next else {
            return Step::Leave;
        };
        if self.blocks >= self.remaining {
            self.stopped = Some((Stop::Budget, pc));
            return Step::Leave;
        }
        if self.exit.is_some_and(ExitFlag::raised) {
            self.stopped = Some((Stop::Exit, pc));
            return Step::Leave;
        }
        if host.spent() {
            self.stopped = Some((Stop::Spent, pc));
            return Step::Leave;
        }
        if self.cache.sync(self.front.epoch()) {
            self.stats.resyncs += 1;
            self.from = None;
        }
        match self.front.enter(pc, host) {
            Ok(Entry::Ready) => {}
            Ok(Entry::Leave) => {
                self.stopped = Some((Stop::Declined, pc));
                return Step::Leave;
            }
            Err(e) => {
                // Not a guest condition, so it ends the run — but it ends it
                // through the caller, which is the only place a `Result` can
                // be returned from once the loop is inside generated code.
                self.err = Some(e);
                self.stopped = Some((Stop::Declined, pc));
                return Step::Leave;
            }
        }
        let key = self.front.key();
        let found = match self.from.and_then(|f| self.cache.follow(f, pc, key)) {
            Some(id) => Some((id, true)),
            None => self.cache.lookup(pc, key).map(|id| {
                self.stats.looked_up += 1;
                (id, false)
            }),
        };
        let Some((id, chained)) = found else {
            // Nothing cached: lifting is the loop's job, not this one's.
            self.resumed = Some(Resumed {
                pc,
                key,
                from: self.from,
                found: None,
            });
            return Step::Leave;
        };
        if chained {
            self.stats.chained += 1;
        } else if let Some(f) = self.from {
            self.cache.link(f, pc, id);
        }
        // Compiled, in the generation this engine is serving — a reference
        // from before a code-buffer reset names whatever took its place, and
        // the engine cannot be asked, because it is running. And short enough
        // for the frame every block of a chain shares, which cannot be grown
        // from here: growing it moves it, and the running code holds its
        // address in a register.
        //
        // Spelled as three `if`s rather than as a chain of `Option` combinators
        // because a closure that captures `self` is not inlined into a thunk
        // generated code enters through a function pointer: as
        // `Option::filter` this cost 27 host instructions per block entry on
        // an AArch64 Linux boot, against the four compares it is.
        if let Some(code) = self.cache.code(id)
            && code.generation == self.link.generation
            && let Some(block) = self.cache.block(id)
            && block.temp_count() <= self.link.temps
        {
            self.stats.compiled += 1;
            self.stats.linked += 1;
            self.id = id;
            return Step::Go {
                id,
                code,
                entry: self.link.base.wrapping_add(code.chain),
            };
        }
        self.resumed = Some(Resumed {
            pc,
            key,
            from: self.from,
            found: Some((id, chained)),
        });
        Step::Leave
    }
}

/// The loop that keeps a guest inside translated code.
#[derive(Debug)]
pub struct Dispatcher {
    cache: BlockCache,
    interp: Interp,
    /// The host code generator, when there is one and it has been given.
    ///
    /// `None` is the whole of the portable path: every block is interpreted,
    /// which is what `no_std`, wasm and any host without a backend do.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    backend: Option<Engine>,
    exit: Option<ExitFlag>,
    stats: DispatchStats,
}

impl Dispatcher {
    /// A dispatcher over a default-sized cache.
    #[must_use]
    pub fn new() -> Dispatcher {
        Dispatcher::with_cache(BlockCache::new())
    }

    /// A dispatcher over `cache`.
    #[must_use]
    pub fn with_cache(cache: BlockCache) -> Dispatcher {
        Dispatcher {
            cache,
            interp: Interp::new(),
            #[cfg(any(
                all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
                all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
            ))]
            backend: None,
            exit: None,
            stats: DispatchStats::default(),
        }
    }

    /// The same dispatcher, compiling blocks with `engine` where it can.
    ///
    /// A block the engine refuses runs on [`Interp`](crate::ir::Interp), and
    /// the two are indistinguishable to the guest — same registers, same
    /// memory, same faults, same ticks, in the same order — which is the claim
    /// both differential harnesses check. So this is a speed knob and never a
    /// semantic one.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    #[cfg_attr(docsrs, doc(cfg(any(feature = "jit-x86", feature = "jit-arm64"))))]
    #[must_use]
    pub fn with_backend(mut self, engine: Engine) -> Dispatcher {
        self.backend = Some(engine);
        self
    }

    /// The host code generator, if this dispatcher has one.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    #[cfg_attr(docsrs, doc(cfg(any(feature = "jit-x86", feature = "jit-arm64"))))]
    #[inline]
    #[must_use]
    pub fn backend(&self) -> Option<&Engine> {
        self.backend.as_ref()
    }

    /// The same dispatcher, unwinding when `flag` is raised.
    #[must_use]
    pub fn with_exit_flag(mut self, flag: ExitFlag) -> Dispatcher {
        self.exit = Some(flag);
        self
    }

    /// The block cache, for statistics and for a caller that invalidates.
    #[inline]
    #[must_use]
    pub fn cache(&self) -> &BlockCache {
        &self.cache
    }

    /// The block cache, mutably.
    #[inline]
    pub fn cache_mut(&mut self) -> &mut BlockCache {
        &mut self.cache
    }

    /// What this dispatcher has been asked to do.
    #[inline]
    #[must_use]
    pub fn stats(&self) -> DispatchStats {
        self.stats
    }

    /// The cache's own statistics.
    #[inline]
    #[must_use]
    pub fn cache_stats(&self) -> CacheStats {
        self.cache.stats()
    }

    /// Run at most `budget` blocks from `pc`.
    ///
    /// And at most as many ticks as the host says it has:
    /// [`IrHost::spent`](crate::ir::IrHost::spent) ends the run with
    /// [`Stop::Spent`], at a block boundary here or at a guest instruction
    /// boundary inside a block. A run always executes at least one block and a
    /// block always retires at least one guest instruction, whatever that
    /// method says — see the module docs for why that is not a rounding error
    /// but the property a core's two engines agree through.
    ///
    /// # Errors
    ///
    /// Whatever [`Frontend::translate`] or
    /// [`Interp::run`](crate::ir::Interp::run) said. Neither is recoverable
    /// here: a frontend that cannot lift says so with `insns == 0`, and a
    /// backend error is a malformed block.
    ///
    /// # Panics
    ///
    /// Never: every block reached through the cache is one this run inserted
    /// or found, and both are checked.
    pub fn run<F, H>(
        &mut self,
        front: &mut F,
        host: &mut H,
        mut pc: u64,
        budget: usize,
    ) -> Result<Run>
    where
        F: Frontend<H> + ?Sized,
        H: IrHost + StoreLog + FastMem,
    {
        let pc_slot = front.pc_slot();
        let mut from: Option<BlockId> = None;
        let mut blocks = 0usize;
        let mut insns = 0usize;
        // Set when a chain stopped at a boundary it had already done the entry
        // work for. See [`Resumed`]: going round the top of the loop would
        // charge that work twice.
        let mut resumed: Option<Resumed> = None;

        let stop = loop {
            // The cache key for this boundary, and — when a chain resolved it
            // already — the block it found.
            let (key, mut ready) = match resumed.take() {
                Some(r) => {
                    pc = r.pc;
                    from = r.from;
                    (r.key, r.found)
                }
                None => {
                    if blocks >= budget {
                        break Stop::Budget;
                    }
                    if self.exit.as_ref().is_some_and(ExitFlag::raised) {
                        break Stop::Exit;
                    }
                    // The tick budget, at a block boundary. Skipped for the
                    // first block of a run, and that is the same rule
                    // [`Interp`](crate::ir::Interp) applies to the first
                    // boundary of a block: a caller whose own interpreter
                    // always retires one guest instruction per call must not
                    // be handed a run that retired none, or the two stop
                    // agreeing about how far a quantum got. So a run executes
                    // at least one block, a block retires at least one
                    // instruction, and after that the host decides.
                    if blocks > 0 && host.spent() {
                        break Stop::Spent;
                    }
                    // Per block, not per run. A guest store can remap an
                    // address space, a store ends its block, and a chained
                    // successor would otherwise be served out of a cache
                    // lifted through the topology that store replaced — a
                    // window that did not exist while a run was one block
                    // long. The predecessor goes with it: a flush retires
                    // every id, so following a link from before one would
                    // reach whatever took the slot.
                    if self.cache.sync(front.epoch()) {
                        self.stats.resyncs += 1;
                        from = None;
                    }
                    // What this block owes before it exists as far as this
                    // loop is concerned: the entry translation, and whatever
                    // else the guest decides at a boundary. It is inside the
                    // loop rather than before it because a *chained* successor
                    // owes exactly the same thing, and a dispatcher that only
                    // charged the first block would make a chain cheaper than
                    // the blocks it replaced.
                    if front.enter(pc, host)? == Entry::Leave {
                        break Stop::Declined;
                    }
                    (front.key(), None)
                }
            };
            // A chain that already looked this up also already counted the hit
            // and patched the predecessor, so neither happens twice.
            let patched = ready.is_some();
            let (id, chained) = match ready.take() {
                Some(found) => found,
                None => match from.and_then(|f| self.cache.follow(f, pc, key)) {
                    Some(id) => (id, true),
                    None => match self.cache.lookup(pc, key) {
                        Some(id) => {
                            self.stats.looked_up += 1;
                            (id, false)
                        }
                        None => {
                            let t = front.translate(pc)?;
                            self.stats.translated += 1;
                            if t.insns == 0 {
                                break Stop::Untranslatable { pc };
                            }
                            (self.cache.insert(pc, key, t.page, t.insns, t.block), false)
                        }
                    },
                },
            };
            if patched {
            } else if chained {
                self.stats.chained += 1;
            } else if let Some(f) = from {
                // The patch. Next time this predecessor exits to this PC it
                // reaches the successor with no lookup at all.
                self.cache.link(f, pc, id);
            }

            // Compiled if there is a backend and it takes this block, and
            // interpreted otherwise. The two are indistinguishable to the
            // guest, including in cycle accounting, so `retired` is read off
            // whichever ran rather than off a fixed engine — a run that read
            // the wrong one would tell an oracle to step the wrong number of
            // times, which is the bug the retired count exists to avoid.
            //
            // And it may be more than one block: a compiled block whose
            // successor is compiled too jumps straight to it, and everything
            // this loop does at a boundary is done by [`Chain::step`] instead.
            let ran = self.execute(front, id, host, pc_slot, budget - blocks)?;
            blocks += ran.blocks;
            insns += ran.insns;

            if let Some(r) = ran.resumed {
                resumed = Some(r);
                continue;
            }
            if let Some((stop, at)) = ran.stopped {
                pc = at;
                break stop;
            }
            if !ran.closed {
                self.stats.blocks += 1;
                // Guest stores land before the next block is chosen, so a
                // block invalidated by one is never served afterwards.
                let cache = &mut self.cache;
                let mut hit = 0usize;
                host.drain_dirty(&mut |page| hit += cache.note_write(page, 1));
                self.stats.smc += hit as u64;
                let survived = self.cache.block(id).is_some();
                match ran.outcome {
                    Outcome::Exit => pc = host.read_slot(pc_slot) as u64,
                    Outcome::Goto { pc: next } | Outcome::Lookup { pc: next } => pc = next,
                    // The block left part-way through, at a guest instruction
                    // boundary it published its state at. The chain ends here
                    // whatever the block budget still allows: the ticks are
                    // gone, and the next block would spend ticks the caller
                    // does not have.
                    //
                    // **The one mutation survivor of this change**, and it is
                    // stated rather than tested away. Deleting the `break`
                    // leaves every test passing, because the loop then goes
                    // round once, reads the epoch, and stops at the top with
                    // the same `Stop::Spent` at the same PC —
                    // `Frontend::enter` is never reached, so nothing is
                    // charged and nothing is translated. The two are
                    // equivalent *exactly while*
                    // [`IrHost::spent`](crate::ir::IrHost::spent)'s
                    // monotonicity holds, which is the contract that method
                    // states and cannot check. This break is what makes the
                    // equivalence not matter.
                    Outcome::Spent { pc: at } => {
                        pc = at;
                        break Stop::Spent;
                    }
                    Outcome::Fault(f) => break Stop::Fault(f),
                    Outcome::Unsupported { op, at } => break Stop::Unsupported { op, at },
                }
                // A block that wrote into its own page is gone, and the id
                // that named it may already have been reused, so it cannot be
                // the predecessor of the next link.
                from = survived.then_some(id);
                continue;
            }

            // A chain whose last block did not reach a terminator. Its own
            // outcome is the answer, and the boundary work is already done.
            match ran.outcome {
                Outcome::Spent { pc: at } => {
                    pc = at;
                    break Stop::Spent;
                }
                Outcome::Fault(f) => break Stop::Fault(f),
                Outcome::Unsupported { op, at } => break Stop::Unsupported { op, at },
                // Unreachable: a chain that reached a terminator either
                // followed it or recorded why it could not, both of which are
                // handled above. Stopping is the conservative answer, and the
                // guest is at a boundary either way.
                Outcome::Exit => {
                    pc = host.read_slot(pc_slot) as u64;
                    break Stop::Budget;
                }
                Outcome::Goto { pc: next } | Outcome::Lookup { pc: next } => {
                    pc = next;
                    break Stop::Budget;
                }
            }
        };

        Ok(Run {
            pc,
            blocks,
            insns,
            stop,
        })
    }

    /// Execute the block `id` names — and everything a link takes it on to —
    /// and say what happened.
    ///
    /// The one place that chooses an engine. A backend that takes the block
    /// runs it; anything else — no backend, a refusal, a code handle that a
    /// buffer reset invalidated — interprets, which is `ROADMAP.md` §9's
    /// *"degrades in speed rather than failing to run"* applied one level down,
    /// to a block rather than to a host.
    ///
    /// A compiled block is entered through `Engine::run_chained`, so it may
    /// jump straight on to a compiled successor without unwinding. What that
    /// costs here is the [`Chain`] it has to build; what it costs the caller is
    /// that the boundary after the last block may already have been dealt with
    /// — see [`Ran::closed`].
    fn execute<F, H>(
        &mut self,
        front: &mut F,
        id: BlockId,
        host: &mut H,
        pc_slot: RegSlot,
        remaining: usize,
    ) -> Result<Ran>
    where
        F: Frontend<H> + ?Sized,
        H: IrHost + StoreLog + FastMem,
    {
        #[cfg(any(
            all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
            all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
        ))]
        {
            // Destructured, because compiling reads the block out of the cache
            // while the engine is borrowed mutably, and the two are different
            // fields of the same struct.
            let Dispatcher {
                cache,
                backend,
                stats,
                exit,
                ..
            } = self;
            if let Some(engine) = backend.as_mut() {
                let block = cache
                    .block(id)
                    .expect("a block just found or just inserted is resident");
                let temps = block.temp_count();
                let code = match cache.code(id).filter(|c| engine.is_live(*c)) {
                    Some(code) => Some(code),
                    // A refusal is not an error and is not recorded against
                    // the block: the engine counts it, and the next time this
                    // block is reached it is refused again for the same
                    // reason, which costs a compile attempt and nothing else.
                    None => engine.compile(block).ok(),
                };
                if let Some(code) = code {
                    cache.set_code(id, code);
                    // The temporary frame the whole chain shares, grown here
                    // because this is the last point at which growing it is
                    // safe: from a boundary inside generated code it would
                    // move out from under the register holding its address.
                    engine.reserve_temps(temps);
                    let mut chain = Chain {
                        link: engine.linkage(),
                        cache,
                        front,
                        stats,
                        exit: exit.as_ref(),
                        pc_slot,
                        remaining: remaining.max(1),
                        id,
                        from: None,
                        blocks: 0,
                        insns: 0,
                        resumed: None,
                        stopped: None,
                        err: None,
                        _host: PhantomData,
                    };
                    if let Some(outcome) = engine.run_chained(id, code, host, &mut chain) {
                        // This block; `Chain::step` counts each one a link
                        // took it on to.
                        chain.stats.compiled += 1;
                        let Chain {
                            blocks,
                            insns,
                            resumed,
                            stopped,
                            err,
                            ..
                        } = chain;
                        let outcome = outcome?;
                        if let Some(e) = err {
                            return Err(e);
                        }
                        return Ok(Ran {
                            outcome,
                            blocks,
                            insns,
                            closed: true,
                            resumed,
                            stopped,
                        });
                    }
                }
            }
        }
        #[cfg(not(any(
            all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
            all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
        )))]
        // No backend on this target, so nothing chains and nothing links:
        // these three describe a chain and there is none to describe.
        let _ = (&*front, pc_slot, remaining);
        let block = self
            .cache
            .block(id)
            .expect("a block just found or just inserted is resident");
        let outcome = self.interp.run(block, host)?;
        Ok(Ran {
            outcome,
            blocks: 1,
            insns: self.interp.boundaries().saturating_sub(1) as usize,
            closed: false,
            resumed: None,
            stopped: None,
        })
    }
}

impl Default for Dispatcher {
    fn default() -> Dispatcher {
        Dispatcher::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::error::BusError;
    use crate::core::space::MemResult;
    use crate::core::value::Width;
    use crate::ir::{BlockBuilder, Const, InsnStart, MemOp, Type};
    use alloc::vec;

    const PC: RegSlot = RegSlot(0);

    /// A block that leaves `next` in the PC slot and exits.
    fn straight(pc: u64, next: u64) -> Block {
        let mut b = BlockBuilder::new(pc, 0);
        b.insn_start(InsnStart {
            pc,
            next_pc: next,
            ticks: 0,
            live: Vec::new(),
        });
        b.charge(1);
        let t = b.imm(Type::I64, Const::Int(u128::from(next)));
        b.insn_start(InsnStart {
            pc: next,
            next_pc: next,
            ticks: 1,
            live: vec![(PC, t)],
        });
        b.exit_tb();
        b.finish()
    }

    /// A block whose successor is **computed**: the exit boundary carries a
    /// placeholder PC and the real one in the slot its map publishes.
    ///
    /// The shape `cpu::x86::lift` closes a `RET` with — `InsnStart::pc` is
    /// static, so a block that does not know its successor at lift time puts
    /// the program-order address there and the truth in [`RegSlot`] — and the
    /// reason a run may not leave at an exit boundary.
    fn computed(pc: u64, next: u64, placeholder: u64) -> Block {
        let mut b = BlockBuilder::new(pc, 0);
        b.insn_start(InsnStart {
            pc,
            next_pc: next,
            ticks: 0,
            live: Vec::new(),
        });
        b.charge(1);
        let t = b.imm(Type::I64, Const::Int(u128::from(next)));
        b.insn_start(InsnStart {
            pc: placeholder,
            next_pc: placeholder,
            ticks: 1,
            live: vec![(PC, t)],
        });
        b.exit_tb();
        b.finish()
    }

    /// A frontend serving [`computed`] blocks, all lying about their exit PC.
    struct Computed {
        step: u64,
        limit: u64,
    }

    impl<H: ?Sized> Frontend<H> for Computed {
        fn epoch(&mut self) -> Epoch {
            Epoch::default()
        }
        fn key(&mut self) -> u64 {
            0
        }
        fn pc_slot(&self) -> RegSlot {
            PC
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            let next = if pc + self.step >= self.limit {
                0x1000
            } else {
                pc + self.step
            };
            Ok(Translation {
                block: computed(pc, next, 0xdead_0000 | pc),
                page: pc & !PAGE_MASK,
                insns: 1,
            })
        }
    }

    /// A frontend over a fixed loop of blocks.
    ///
    /// Named for the loop rather than for the chaining, because [`Chain`] is
    /// the thing that links them and two of those in one file is one too many.
    struct Ring {
        /// `pc -> next pc`, for as many blocks as the test wants.
        step: u64,
        limit: u64,
        epoch: Epoch,
        key: u64,
        translated: Vec<u64>,
    }

    impl<H: ?Sized> Frontend<H> for Ring {
        fn epoch(&mut self) -> Epoch {
            self.epoch
        }
        fn key(&mut self) -> u64 {
            self.key
        }
        fn pc_slot(&self) -> RegSlot {
            PC
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            self.translated.push(pc);
            let next = if pc + self.step >= self.limit {
                0x1000
            } else {
                pc + self.step
            };
            Ok(Translation {
                block: straight(pc, next),
                page: pc & !PAGE_MASK,
                insns: 1,
            })
        }
    }

    #[derive(Default)]
    struct Host {
        slots: [u64; 4],
        ticks: u64,
        dirty: DirtyPages,
        /// The tick allowance, or `None` for a host that never stops a block —
        /// which is every host that existed before [`IrHost::spent`] did.
        allowance: Option<u64>,
    }

    impl Host {
        /// A host that must leave at the first boundary past `ticks`.
        fn within(ticks: u64) -> Host {
            Host {
                allowance: Some(ticks),
                ..Host::default()
            }
        }
    }

    impl IrHost for Host {
        fn read_slot(&mut self, slot: RegSlot) -> u128 {
            u128::from(self.slots[slot.0 as usize])
        }
        fn write_slot(&mut self, slot: RegSlot, value: u128) {
            self.slots[slot.0 as usize] = value as u64;
        }
        fn load(&mut self, _mem: &MemOp, _addr: u64) -> MemResult<u64> {
            Err(BusError::Unassigned)
        }
        fn store(&mut self, mem: &MemOp, addr: u64, _value: u64) -> MemResult {
            self.dirty.note(addr, mem.size.bytes());
            Ok(())
        }
        fn charge(&mut self, ticks: u64) {
            self.ticks += ticks;
        }
        fn insn_start(&mut self, _mark: &InsnStart) {}
        fn spent(&self) -> bool {
            self.allowance.is_some_and(|a| self.ticks >= a)
        }
    }

    impl StoreLog for Host {
        fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64)) {
            self.dirty.drain_dirty(sink);
        }
    }

    // No software TLB here, so no fast path to publish: this host's loads all
    // take the call, which is the default and always correct.
    impl FastMem for Host {}

    fn chain(step: u64, limit: u64) -> Ring {
        Ring {
            step,
            limit,
            epoch: Epoch::default(),
            key: 0,
            translated: Vec::new(),
        }
    }

    #[test]
    fn a_loop_is_translated_once_and_then_chained() {
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = chain(4, 0x1010);
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 400).expect("runs");
        assert_eq!(run.blocks, 400);
        assert_eq!(run.insns, 400);
        assert_eq!(run.stop, Stop::Budget);
        // Four distinct blocks in the loop, translated once each.
        assert_eq!(f.translated.len(), 4);
        assert_eq!(d.stats().translated, 4);
        // and after the first time round, every edge is a patched exit.
        assert!(
            d.stats().chained >= 390,
            "chained {} of {}",
            d.stats().chained,
            run.blocks
        );
        assert_eq!(d.cache_stats().stale_links, 0);
        d.cache().check().expect("consistent");
    }

    #[test]
    fn every_tick_is_charged_whether_the_block_was_cached_or_not() {
        // A cache hit and a cache miss must be indistinguishable to the guest,
        // including in cycle accounting (`ROADMAP.md` §0). Each block charges
        // one, so the total is the block count however the blocks were found.
        let mut d = Dispatcher::new();
        let mut f = chain(4, 0x1010);
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 97).expect("runs");
        assert_eq!(h.ticks, run.blocks as u64);
        assert!(d.stats().chained > 0, "and chaining really happened");
    }

    /// The same guard, with the blocks executed as host code.
    ///
    /// `ROADMAP.md` §0 requires a bit-identical state hash *across the
    /// interpreter and the JIT for the same guest*, and the cycle counter is in
    /// that hash. So the two engines must charge the same ticks at the same
    /// points, whichever ran the block — and the run is done twice, once each
    /// way, on the same programs, so the numbers are compared rather than
    /// merely asserted.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    #[test]
    fn a_compiled_block_charges_exactly_what_an_interpreted_one_charges() {
        let mut interpreted = Dispatcher::new();
        let mut hi = Host::default();
        let a = interpreted
            .run(&mut chain(4, 0x1010), &mut hi, 0x1000, 97)
            .expect("runs");

        let mut compiled = Dispatcher::new()
            .with_backend(crate::jit::host::Engine::new().expect("a W^X code buffer"));
        let mut hc = Host::default();
        let b = compiled
            .run(&mut chain(4, 0x1010), &mut hc, 0x1000, 97)
            .expect("runs");

        assert!(compiled.stats().compiled > 0, "nothing was compiled");
        assert_eq!(hi.ticks, hc.ticks, "the cycle counters must agree");
        assert_eq!(
            a.insns, b.insns,
            "and so must the retired instruction count"
        );
        assert_eq!(a.pc, b.pc);
        assert_eq!(a.stop, b.stop);
        assert_eq!(hi.slots, hc.slots, "and the guest's own state");
    }

    /// A block covering `insns` guest instructions, with a side exit taken
    /// when `leave_at` is reached.
    ///
    /// The superblock shape in miniature: several boundaries, two terminators,
    /// and a forward branch over the first exit sequence.
    fn trace(pc: u64, insns: u64, leave_at: Option<u64>, after: u64) -> Block {
        let mut b = BlockBuilder::new(pc, 0);
        // The branch jumps *over* the exit sequence, so a zero here is the
        // side exit being taken — the inversion `cpu::riscv::lift` emits.
        let skip = b.imm(Type::I1, Const::Int(0));
        let mut ticks = 0u64;
        for i in 0..insns {
            b.insn_start(InsnStart {
                pc: pc + i * 4,
                next_pc: pc + (i + 1) * 4,
                ticks,
                live: Vec::new(),
            });
            b.charge(1);
            ticks += 1;
            if leave_at == Some(i) {
                // The side exit, inline and branched over — exactly the shape
                // `cpu::riscv::lift` emits.
                let over = b.emit_raw(
                    Opcode::BRCOND,
                    Type::I64,
                    None,
                    None,
                    &[skip],
                    None,
                    None,
                    0,
                );
                let t = b.imm(Type::I64, Const::Int(u128::from(after)));
                b.insn_start(InsnStart {
                    pc: after,
                    next_pc: after,
                    ticks,
                    live: vec![(PC, t)],
                });
                b.exit_tb();
                b.patch_aux(over, b.next_index() as u32);
            }
        }
        let t = b.imm(Type::I64, Const::Int(u128::from(after)));
        b.insn_start(InsnStart {
            pc: after,
            next_pc: after,
            ticks,
            live: vec![(PC, t)],
        });
        b.exit_tb();
        b.finish()
    }

    /// A frontend serving one trace, over and over.
    struct Traces {
        insns: u64,
        leave_at: Option<u64>,
        epoch: Epoch,
    }

    impl<H: ?Sized> Frontend<H> for Traces {
        fn epoch(&mut self) -> Epoch {
            self.epoch
        }
        fn key(&mut self) -> u64 {
            0
        }
        fn pc_slot(&self) -> RegSlot {
            PC
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            Ok(Translation {
                block: trace(pc, self.insns, self.leave_at, pc),
                page: pc & !PAGE_MASK,
                // Deliberately the *static* count, which is what a superblock
                // covers and not what a run through it retires.
                insns: self.insns as usize,
            })
        }
    }

    #[test]
    fn a_side_exit_retires_fewer_instructions_than_the_trace_covers() {
        // The static count would say sixteen a block; the run leaves through
        // the side exit after five. A dispatcher that reported the static
        // number would tell an oracle to step three times too far.
        let mut d = Dispatcher::new();
        let mut f = Traces {
            insns: 16,
            leave_at: Some(4),
            epoch: Epoch::default(),
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 10).expect("runs");
        assert_eq!(run.blocks, 10);
        assert_eq!(
            run.insns, 50,
            "five guest instructions a block, not sixteen"
        );
        // and the ticks agree with the instructions, not with the coverage.
        assert_eq!(h.ticks, 50);
    }

    #[test]
    fn a_trace_that_runs_to_its_end_retires_everything_it_covers() {
        let mut d = Dispatcher::new();
        let mut f = Traces {
            insns: 16,
            leave_at: None,
            epoch: Epoch::default(),
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 10).expect("runs");
        assert_eq!(run.insns, 160);
        assert_eq!(h.ticks, 160);
    }

    // ---- the tick allowance ------------------------------------------

    #[test]
    fn a_block_leaves_at_the_first_instruction_boundary_past_its_allowance() {
        // The whole mechanism in one assertion: a sixteen-instruction trace,
        // an allowance of five, and the run stops *inside* it having retired
        // exactly five — where the block used to be refused outright because
        // its worst case did not fit.
        let mut d = Dispatcher::new();
        let mut f = Traces {
            insns: 16,
            leave_at: None,
            epoch: Epoch::default(),
        };
        let mut h = Host::within(5);
        let run = d.run(&mut f, &mut h, 0x1000, 10).expect("runs");
        assert_eq!(run.stop, Stop::Spent);
        assert_eq!(run.blocks, 1, "one block was entered");
        assert_eq!(run.insns, 5, "and five of its sixteen instructions retired");
        assert_eq!(h.ticks, 5, "charging exactly what retired, and no more");
        assert_eq!(
            run.pc,
            0x1000 + 5 * 4,
            "the guest stands at the instruction that did not start"
        );
    }

    #[test]
    fn an_allowance_that_is_already_spent_still_retires_one_instruction() {
        // The floor, and it is not a rounding error. A caller's own
        // interpreter always retires one guest instruction per call — that is
        // what `while used < allowance { advance() }` means — so a run that
        // retired none would leave the guest exactly where it found it and the
        // two engines would part company about how far the quantum got. So the
        // first block of a run always starts, and the first boundary of a
        // block never stops it.
        let mut d = Dispatcher::new();
        let mut f = Traces {
            insns: 16,
            leave_at: None,
            epoch: Epoch::default(),
        };
        let mut h = Host::within(0);
        let run = d.run(&mut f, &mut h, 0x1000, 10).expect("runs");
        assert_eq!(run.stop, Stop::Spent);
        assert_eq!(run.blocks, 1);
        assert_eq!(run.insns, 1);
        assert_eq!(h.ticks, 1);
        assert_eq!(run.pc, 0x1004);
    }

    #[test]
    fn a_spent_allowance_ends_the_chain_at_a_block_boundary_too() {
        // The other half of the check: one-instruction blocks, so the
        // allowance runs out between them rather than inside one. The chain
        // stops, and the block budget of forty never comes into it.
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = chain(4, 0x1010);
        let mut h = Host::within(3);
        let run = d.run(&mut f, &mut h, 0x1000, 40).expect("runs");
        assert_eq!(run.stop, Stop::Spent);
        assert_eq!(run.insns, 3);
        assert_eq!(h.ticks, 3);
        assert_eq!(run.pc, 0x100c);
    }

    #[test]
    fn a_run_never_leaves_at_an_exit_boundary_because_its_pc_may_be_a_placeholder() {
        // The bug this pins cost a Linux guest a jump into the middle of
        // nowhere. `InsnStart::pc` is a **static** column, and a block whose
        // successor is computed — every `RET`, every indirect jump — cannot
        // know it at lift time, so it writes the program-order address there
        // and publishes the real one through the slot. A run that leaves at
        // such a boundary and believes `mark.pc` resumes the guest one
        // instruction past the end of the block.
        //
        // The answer is not to special-case it but to never stop there: the
        // terminator is the very next IR instruction, it charges nothing, and
        // the block ends of its own accord with the PC its map published. So
        // the allowance is spent exactly at the exit boundary here, and the
        // guest still comes out at the successor.
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = Computed {
            step: 4,
            limit: 0x1010,
        };
        let mut h = Host::within(1);
        let run = d.run(&mut f, &mut h, 0x1000, 40).expect("runs");
        assert_eq!(run.stop, Stop::Spent);
        assert_eq!(run.insns, 1);
        assert_eq!(
            run.pc, 0x1004,
            "the run resumed at the exit boundary's placeholder"
        );
        assert_eq!(h.slots[PC.0 as usize], 0x1004);
    }

    /// The same, compiled.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    #[test]
    fn a_compiled_run_never_leaves_at_an_exit_boundary_either() {
        // The backend is told which boundaries are exit boundaries at compile
        // time, because its replay is handed a range of events and never sees
        // an instruction index. A backend that was told wrong would resume the
        // guest at the placeholder while the interpreter resumed it correctly,
        // which is a divergence no state comparison of a *finished* block can
        // reach.
        for allowance in [0u64, 1, 2, 5] {
            let mut interpreted = Dispatcher::with_cache(BlockCache::with_capacity(64));
            let mut hi = Host::within(allowance);
            let a = interpreted
                .run(
                    &mut Computed {
                        step: 4,
                        limit: 0x1010,
                    },
                    &mut hi,
                    0x1000,
                    40,
                )
                .expect("runs");

            let mut compiled = Dispatcher::with_cache(BlockCache::with_capacity(64))
                .with_backend(crate::jit::host::Engine::new().expect("a W^X code buffer"));
            let mut hc = Host::within(allowance);
            let b = compiled
                .run(
                    &mut Computed {
                        step: 4,
                        limit: 0x1010,
                    },
                    &mut hc,
                    0x1000,
                    40,
                )
                .expect("runs");

            assert!(compiled.stats().compiled > 0, "nothing was compiled");
            assert_eq!(a.pc, b.pc, "at an allowance of {allowance}");
            assert_eq!(a.stop, b.stop, "at an allowance of {allowance}");
            assert_eq!(a.insns, b.insns, "at an allowance of {allowance}");
            assert_eq!(hi.slots, hc.slots, "at an allowance of {allowance}");
            assert_eq!(
                a.pc & 0xdead_0000,
                0,
                "the run resumed at a placeholder: {:#x}",
                a.pc
            );
        }
    }

    #[test]
    fn a_host_that_says_nothing_is_bounded_by_blocks_exactly_as_before() {
        // The default, asserted rather than assumed: every host in the tree
        // that predates `IrHost::spent` must run identically.
        let mut d = Dispatcher::new();
        let mut f = Traces {
            insns: 16,
            leave_at: None,
            epoch: Epoch::default(),
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 10).expect("runs");
        assert_eq!(run.stop, Stop::Budget);
        assert_eq!(run.insns, 160);
    }

    /// The allowance, with the blocks executed as host code.
    ///
    /// `ROADMAP.md` §0's claim applied to the newest way a block can end: the
    /// two engines must leave at the **same** boundary, with the same ticks
    /// charged and the same guest state, or a core that admits a block it is
    /// not sure fits produces a different state hash depending on which engine
    /// ran it. Compared rather than asserted — both runs, same programs.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    #[test]
    fn a_compiled_block_leaves_at_exactly_the_boundary_the_interpreter_leaves_at() {
        for allowance in [0u64, 1, 5, 15, 16, 31, 100] {
            let traces = || Traces {
                insns: 16,
                leave_at: None,
                epoch: Epoch::default(),
            };
            let mut interpreted = Dispatcher::new();
            let mut hi = Host::within(allowance);
            let a = interpreted
                .run(&mut traces(), &mut hi, 0x1000, 10)
                .expect("runs");

            let mut compiled = Dispatcher::new()
                .with_backend(crate::jit::host::Engine::new().expect("a W^X code buffer"));
            let mut hc = Host::within(allowance);
            let b = compiled
                .run(&mut traces(), &mut hc, 0x1000, 10)
                .expect("runs");

            assert!(
                compiled.stats().compiled > 0,
                "nothing was compiled at {allowance}"
            );
            assert_eq!(a.stop, b.stop, "at an allowance of {allowance}");
            assert_eq!(a.pc, b.pc, "at an allowance of {allowance}");
            assert_eq!(a.insns, b.insns, "at an allowance of {allowance}");
            assert_eq!(a.blocks, b.blocks, "at an allowance of {allowance}");
            assert_eq!(hi.ticks, hc.ticks, "at an allowance of {allowance}");
            assert_eq!(hi.slots, hc.slots, "at an allowance of {allowance}");
        }
    }

    /// The same, over a chain of one-instruction blocks.
    ///
    /// A different shape on purpose: here the allowance runs out between
    /// blocks as often as inside one, so the dispatcher's own boundary check
    /// and the backend's have to agree with the interpreter's *together*.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    #[test]
    fn a_compiled_chain_leaves_where_an_interpreted_chain_leaves() {
        for allowance in [0u64, 1, 2, 3, 7, 40] {
            let mut interpreted = Dispatcher::with_cache(BlockCache::with_capacity(64));
            let mut hi = Host::within(allowance);
            let a = interpreted
                .run(&mut chain(4, 0x1010), &mut hi, 0x1000, 40)
                .expect("runs");

            let mut compiled = Dispatcher::with_cache(BlockCache::with_capacity(64))
                .with_backend(crate::jit::host::Engine::new().expect("a W^X code buffer"));
            let mut hc = Host::within(allowance);
            let b = compiled
                .run(&mut chain(4, 0x1010), &mut hc, 0x1000, 40)
                .expect("runs");

            assert!(compiled.stats().compiled > 0, "nothing was compiled");
            assert_eq!(a.stop, b.stop, "at an allowance of {allowance}");
            assert_eq!(a.pc, b.pc, "at an allowance of {allowance}");
            assert_eq!(a.insns, b.insns, "at an allowance of {allowance}");
            assert_eq!(hi.ticks, hc.ticks, "at an allowance of {allowance}");
            assert_eq!(hi.slots, hc.slots, "at an allowance of {allowance}");
        }
    }

    #[test]
    fn a_raised_exit_flag_stops_within_one_block_however_long_the_block_is() {
        // A trace has fewer boundaries than the basic blocks it replaces, so
        // the safe-point protocol's promise weakens from "one basic block" to
        // "one translation" — bounded by a frontend's instruction limit
        // (`ROADMAP.md` §4.7, and `cpu::riscv::lift::MAX_INSNS`). Bounded is
        // the claim, so this asserts the bound rather than the old wording.
        let flag = ExitFlag::default();
        let mut d = Dispatcher::new().with_exit_flag(flag.clone());
        let mut f = Traces {
            insns: 64,
            leave_at: None,
            epoch: Epoch::default(),
        };
        let mut h = Host::default();
        d.run(&mut f, &mut h, 0x1000, 1).expect("runs");
        flag.raise();
        let run = d.run(&mut f, &mut h, 0x1000, 100).expect("runs");
        assert_eq!(run.stop, Stop::Exit);
        assert_eq!(run.blocks, 0, "no block starts once the flag is up");
    }

    /// The safe-point bound, with the blocks executed as host code.
    ///
    /// A compiled block has exactly the same boundaries as the interpreted one
    /// — the code generator changes how a block runs, never where it ends — so
    /// `ROADMAP.md` §4.7's protocol is unchanged and the delay before a raised
    /// flag is honoured is still bounded by a frontend's own instruction limit.
    /// That is asserted rather than argued, because "the backend did not change
    /// it" is the kind of claim that stops being true quietly.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    #[test]
    fn a_raised_exit_flag_stops_a_compiled_run_within_one_block_too() {
        let flag = ExitFlag::default();
        let mut d = Dispatcher::new()
            .with_exit_flag(flag.clone())
            .with_backend(crate::jit::host::Engine::new().expect("a W^X code buffer"));
        let mut f = Traces {
            insns: 64,
            leave_at: None,
            epoch: Epoch::default(),
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 4).expect("runs");
        assert_eq!(run.insns, 4 * 64, "every merged instruction retired");
        assert!(d.stats().compiled > 0, "the blocks really were compiled");
        flag.raise();
        let run = d.run(&mut f, &mut h, 0x1000, 100).expect("runs");
        assert_eq!(run.stop, Stop::Exit);
        assert_eq!(run.blocks, 0, "no block starts once the flag is up");
    }

    /// A block that stores `next` into memory at `store_at`, leaves `next` in
    /// the PC slot, and exits.
    ///
    /// The store is what a self-modifying guest does: it goes through
    /// [`IrHost::store`], the test host records the page, and the dispatcher
    /// drains it at the boundary.
    fn writer(pc: u64, next: u64, store_at: u64) -> Block {
        let mut b = BlockBuilder::new(pc, 0);
        b.insn_start(InsnStart {
            pc,
            next_pc: next,
            ticks: 0,
            live: Vec::new(),
        });
        b.charge(1);
        let addr = b.imm(Type::I64, Const::Int(u128::from(store_at)));
        let value = b.imm(Type::I64, Const::Int(0));
        b.store(Type::I64, addr, value, MemOp::store(Width::U8));
        let t = b.imm(Type::I64, Const::Int(u128::from(next)));
        b.insn_start(InsnStart {
            pc: next,
            next_pc: next,
            ticks: 1,
            live: vec![(PC, t)],
        });
        b.exit_tb();
        b.finish()
    }

    /// A two-block loop whose first block writes into the **second** block's
    /// page every time it runs, and whose second block writes somewhere
    /// nothing was ever lifted from.
    ///
    /// A self-modifying guest, in the shape that matters to a link: the
    /// successor is invalidated by the predecessor, between the predecessor's
    /// last instruction and the jump that would have entered it.
    struct Writer {
        translated: usize,
    }

    impl<H: ?Sized> Frontend<H> for Writer {
        fn epoch(&mut self) -> Epoch {
            Epoch::default()
        }
        fn key(&mut self) -> u64 {
            0
        }
        fn pc_slot(&self) -> RegSlot {
            PC
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            self.translated += 1;
            let (next, at) = if pc == 0x1000 {
                (0x2000, 0x2000)
            } else {
                (0x1000, 0x9000)
            };
            Ok(Translation {
                block: writer(pc, next, at),
                page: pc & !PAGE_MASK,
                insns: 1,
            })
        }
    }

    /// The measurement this whole mechanism exists for, asserted rather than
    /// assumed: a compiled chain is *entered by a jump*, and every block past
    /// the first is reached without returning to this loop at all.
    ///
    /// [`DispatchStats::chained`] does not say this — it was above zero long
    /// before a link reached generated code — which is exactly why
    /// [`DispatchStats::linked`] is a separate counter.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    #[test]
    fn a_compiled_chain_past_its_first_block_never_returns_to_this_loop() {
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64))
            .with_backend(crate::jit::host::Engine::new().expect("a W^X code buffer"));
        let mut f = chain(4, 0x1010);
        let mut h = Host::default();
        // Once round to lift and compile the four blocks.
        d.run(&mut f, &mut h, 0x1000, 8).expect("runs");
        let before = d.stats().linked;
        let run = d.run(&mut f, &mut h, 0x1000, 64).expect("runs");
        assert_eq!(run.blocks, 64);
        assert_eq!(run.insns, 64);
        assert_eq!(run.stop, Stop::Budget);
        assert_eq!(
            d.stats().linked - before,
            63,
            "every block but the first was entered by a jump"
        );
        assert_eq!(d.cache_stats().stale_links, 0);
        d.cache().check().expect("consistent");
    }

    /// The safe point, inside a chain that never unwinds.
    ///
    /// A link is a jump from one block's code into another's, so the loop that
    /// used to test the flag between them is not running. [`Chain::step`] tests
    /// it instead, and this is what says so: the flag goes up while a chain is
    /// mid-flight and the run stops at the very next boundary, with the guest
    /// standing where the unlinked dispatcher would have left it.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    #[test]
    fn a_raised_exit_flag_stops_a_linked_chain_at_the_next_boundary() {
        let flag = ExitFlag::default();
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64))
            .with_exit_flag(flag.clone())
            .with_backend(crate::jit::host::Engine::new().expect("a W^X code buffer"));
        let mut f = Gate {
            chain: chain(4, 0x1010),
            seen: Vec::new(),
            limit: usize::MAX,
        };
        let mut h = Host::default();
        d.run(&mut f, &mut h, 0x1000, 8).expect("runs");
        assert!(d.stats().linked > 0, "nothing was linked");

        // The flag goes up *during* the chain: `Gate::enter` is called from
        // inside generated code at every boundary, so raising it at the third
        // one is raising it with a compiled block in flight.
        struct Raiser {
            inner: Gate,
            flag: ExitFlag,
            at: usize,
            seen: usize,
        }
        impl<H: ?Sized> Frontend<H> for Raiser {
            fn epoch(&mut self) -> Epoch {
                Frontend::<H>::epoch(&mut self.inner)
            }
            fn enter(&mut self, pc: u64, host: &mut H) -> Result<Entry> {
                self.seen += 1;
                if self.seen == self.at {
                    self.flag.raise();
                }
                Frontend::<H>::enter(&mut self.inner, pc, host)
            }
            fn key(&mut self) -> u64 {
                Frontend::<H>::key(&mut self.inner)
            }
            fn pc_slot(&self) -> RegSlot {
                Frontend::<H>::pc_slot(&self.inner)
            }
            fn translate(&mut self, pc: u64) -> Result<Translation> {
                Frontend::<H>::translate(&mut self.inner, pc)
            }
        }

        let mut r = Raiser {
            inner: f,
            flag,
            at: 3,
            seen: 0,
        };
        let run = d.run(&mut r, &mut h, 0x1000, 64).expect("runs");
        assert_eq!(run.stop, Stop::Exit);
        // Three boundaries were entered, so three blocks ran and the fourth
        // did not: the flag was honoured within one block of being raised.
        assert_eq!(run.blocks, 3);
        assert_eq!(run.insns, 3);
        assert_eq!(run.pc, 0x100c);
    }

    /// Self-modifying code, one level down: a block that writes into its
    /// successor's page must not be *jumped into* it either.
    ///
    /// The drain happens at the boundary inside generated code, before the
    /// successor is chosen, which is the same order [`Dispatcher::run`] uses
    /// and the reason a link cannot outrun an invalidation.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    #[test]
    fn a_block_that_writes_into_its_successors_page_is_not_linked_to_it() {
        let mut interpreted = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut fi = Writer { translated: 0 };
        let mut hi = Host::default();
        let a = interpreted.run(&mut fi, &mut hi, 0x1000, 12).expect("runs");

        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64))
            .with_backend(crate::jit::host::Engine::new().expect("a W^X code buffer"));
        let mut f = Writer { translated: 0 };
        let mut h = Host::default();
        let b = d.run(&mut f, &mut h, 0x1000, 12).expect("runs");

        assert_eq!(a, b, "a linked chain saw what the interpreter saw");
        assert_eq!(hi.slots, h.slots);
        // The predecessor never links to a successor its own store killed, so
        // the successor is lifted afresh on every pass — six of them, plus the
        // predecessor's one. Five invalidations rather than six: the first
        // pass writes into a page nothing had been lifted from yet.
        assert_eq!(d.stats().smc, 5, "the successor was invalidated each pass");
        assert_eq!(d.stats().translated, fi.translated as u64);
        assert_eq!(f.translated, 7);
        // The other edge is linkable and was linked, so this is not a run in
        // which linking simply never happened.
        assert!(d.stats().linked > 0, "nothing was linked");
        assert_eq!(d.cache_stats().stale_links, 0);
        d.cache().check().expect("consistent");
    }

    /// A code buffer small enough to fill and be thrown away under a running
    /// guest, which is the one thing a link may never follow into.
    ///
    /// A [`CodeRef`] from before a reset names whatever took its index, and a
    /// link resolved from one would be a jump into another block's code — or
    /// into the middle of an instruction. [`Chain`] refuses a reference whose
    /// generation is not the one the engine is serving, and this is what says
    /// so: the run still agrees with the interpreter, block for block and tick
    /// for tick, with the buffer resetting throughout.
    #[cfg(any(
        all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
        all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
    ))]
    #[test]
    fn a_code_buffer_reset_under_a_running_chain_is_never_jumped_into() {
        // `Writer` invalidates its own successor on every pass, so the
        // engine compiles for ever and a one-page buffer fills again and
        // again — which is the churn this needs and a static loop cannot
        // produce.
        let mut interpreted = Dispatcher::with_cache(BlockCache::with_capacity(256));
        let mut hi = Host::default();
        let a = interpreted
            .run(&mut Writer { translated: 0 }, &mut hi, 0x1000, 400)
            .expect("runs");

        let engine = crate::jit::host::Engine::with_capacity(4096).expect("a W^X code buffer");
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(256)).with_backend(engine);
        let mut h = Host::default();
        let b = d
            .run(&mut Writer { translated: 0 }, &mut h, 0x1000, 400)
            .expect("runs");

        assert!(
            d.backend().expect("a backend").stats().resets > 0,
            "the buffer never filled, so this proves nothing"
        );
        assert!(d.stats().linked > 0, "nothing was linked");
        assert_eq!(a, b, "a chain across a reset saw what the interpreter saw");
        assert_eq!(hi.slots, h.slots);
        assert_eq!(hi.ticks, h.ticks);
        assert_eq!(d.cache_stats().stale_links, 0);
        d.cache().check().expect("consistent");
    }

    #[test]
    fn a_raised_exit_flag_stops_at_a_block_boundary() {
        let flag = ExitFlag::default();
        let mut d = Dispatcher::new().with_exit_flag(flag.clone());
        let mut f = chain(4, 0x1010);
        let mut h = Host::default();
        assert_eq!(
            d.run(&mut f, &mut h, 0x1000, 10).expect("runs").stop,
            Stop::Budget
        );
        flag.raise();
        let run = d.run(&mut f, &mut h, 0x1000, 10).expect("runs");
        assert_eq!(run.stop, Stop::Exit);
        assert_eq!(run.blocks, 0, "no block starts once the flag is up");
    }

    #[test]
    fn an_epoch_change_between_runs_resynchronises_the_cache() {
        let mut d = Dispatcher::new();
        let mut f = chain(4, 0x1010);
        let mut h = Host::default();
        d.run(&mut f, &mut h, 0x1000, 20).expect("runs");
        assert_eq!(d.stats().translated, 4);
        f.epoch.topology += 1;
        d.run(&mut f, &mut h, 0x1000, 20).expect("runs");
        assert_eq!(d.stats().resyncs, 1);
        assert_eq!(d.stats().translated, 8, "every block was lifted again");
    }

    /// A [`Ring`] that records every PC it was entered at and refuses to
    /// enter the `limit`th.
    struct Gate {
        chain: Ring,
        seen: Vec<u64>,
        limit: usize,
    }

    impl<H: ?Sized> Frontend<H> for Gate {
        fn epoch(&mut self) -> Epoch {
            Frontend::<H>::epoch(&mut self.chain)
        }
        fn enter(&mut self, pc: u64, _host: &mut H) -> Result<Entry> {
            self.seen.push(pc);
            Ok(if self.seen.len() > self.limit {
                Entry::Leave
            } else {
                Entry::Ready
            })
        }
        fn key(&mut self) -> u64 {
            Frontend::<H>::key(&mut self.chain)
        }
        fn pc_slot(&self) -> RegSlot {
            Frontend::<H>::pc_slot(&self.chain)
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            Frontend::<H>::translate(&mut self.chain, pc)
        }
    }

    #[test]
    fn a_boundary_hook_is_called_once_per_block_chained_or_not() {
        // The property `cpu::riscv::engine` depends on for its cycle counter:
        // a chained successor is entered exactly as an unchained one is, so a
        // guest that charges for its entry fetch charges the same whichever
        // way the block was reached.
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = Gate {
            chain: chain(4, 0x1010),
            seen: Vec::new(),
            limit: usize::MAX,
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 40).expect("runs");
        assert_eq!(run.blocks, 40);
        assert_eq!(f.seen.len(), 40, "one entry per block, chained included");
        assert!(d.stats().chained > 0, "and chaining really happened");
        // The PCs are the ones the blocks ran at, in order, round the loop.
        assert_eq!(&f.seen[..5], &[0x1000, 0x1004, 0x1008, 0x100c, 0x1000]);
    }

    /// A frontend that computes its key in `enter`, as the RISC-V engine does,
    /// and records what `key` was asked for and whether `enter` had run.
    struct Ordered {
        chain: Ring,
        entered: Option<u64>,
        asked: Vec<(Option<u64>, u64)>,
    }

    impl<H: ?Sized> Frontend<H> for Ordered {
        fn epoch(&mut self) -> Epoch {
            self.chain.epoch
        }
        fn enter(&mut self, pc: u64, _host: &mut H) -> Result<Entry> {
            self.entered = Some(pc);
            Ok(Entry::Ready)
        }
        fn key(&mut self) -> u64 {
            let key = self.entered.unwrap_or(u64::MAX);
            self.asked.push((self.entered, key));
            key
        }
        fn pc_slot(&self) -> RegSlot {
            Frontend::<H>::pc_slot(&self.chain)
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            Frontend::<H>::translate(&mut self.chain, pc)
        }
    }

    #[test]
    fn a_boundary_hook_runs_before_the_key_it_computes_is_read() {
        // The order is documented and load-bearing: `cpu::riscv::engine`
        // resolves its entry fetch to a physical page in `enter` and *is* that
        // page in `key`. Asked the other way round, every block would be
        // cached under its predecessor's world — which on a guest whose blocks
        // share a page is invisible until one of them does not.
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = Ordered {
            chain: chain(4, 0x1010),
            entered: None,
            asked: Vec::new(),
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 12).expect("runs");
        assert_eq!(run.blocks, 12);
        assert_eq!(f.asked.len(), 12, "one key per block");
        assert!(
            f.asked.iter().all(|&(entered, key)| entered == Some(key)),
            "`key` was asked before `enter` set it: {:?}",
            f.asked
        );
    }

    #[test]
    fn a_frontend_that_declines_a_boundary_stops_the_run_there() {
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = Gate {
            chain: chain(4, 0x1010),
            seen: Vec::new(),
            limit: 3,
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 40).expect("runs");
        assert_eq!(run.stop, Stop::Declined);
        assert_eq!(run.blocks, 3, "the declined block did not run");
        assert_eq!(run.insns, 3);
        assert_eq!(run.pc, 0x100c, "and the guest is left standing at it");
        assert_eq!(h.ticks, 3, "the declined block charged nothing");
    }

    /// A [`Ring`] whose topology generation moves partway through a run.
    struct Shifting {
        chain: Ring,
        seen: usize,
        at: usize,
    }

    impl<H: ?Sized> Frontend<H> for Shifting {
        fn epoch(&mut self) -> Epoch {
            self.seen += 1;
            if self.seen > self.at {
                self.chain.epoch.topology = 1;
            }
            self.chain.epoch
        }
        fn key(&mut self) -> u64 {
            Frontend::<H>::key(&mut self.chain)
        }
        fn pc_slot(&self) -> RegSlot {
            Frontend::<H>::pc_slot(&self.chain)
        }
        fn translate(&mut self, pc: u64) -> Result<Translation> {
            Frontend::<H>::translate(&mut self.chain, pc)
        }
    }

    #[test]
    fn a_retopology_partway_through_a_run_is_seen_before_the_next_block() {
        // The window chaining opens and this closes: a guest store can remap
        // an address space, a store ends its block, and the *next* block of
        // the chain would otherwise come out of a cache lifted through the
        // topology that store replaced. The epoch is therefore read at every
        // boundary rather than once per run.
        let mut d = Dispatcher::with_cache(BlockCache::with_capacity(64));
        let mut f = Shifting {
            chain: chain(4, 0x1010),
            seen: 0,
            at: 5,
        };
        let mut h = Host::default();
        let run = d.run(&mut f, &mut h, 0x1000, 20).expect("runs");
        assert_eq!(run.blocks, 20, "the run still finishes");
        assert_eq!(d.stats().resyncs, 1, "and resynchronised inside it");
        assert_eq!(
            d.stats().translated,
            8,
            "four blocks before the flush and four after"
        );
        // The predecessor is dropped with the cache, so no link is followed
        // into a slot the flush retired.
        assert_eq!(d.cache_stats().stale_links, 0);
        d.cache().check().expect("consistent");
    }

    #[test]
    fn a_key_change_is_a_different_translation_at_the_same_pc() {
        let mut d = Dispatcher::new();
        let mut f = chain(4, 0x1010);
        let mut h = Host::default();
        d.run(&mut f, &mut h, 0x1000, 20).expect("runs");
        f.key = 1;
        d.run(&mut f, &mut h, 0x1000, 20).expect("runs");
        assert_eq!(d.stats().translated, 8);
        assert_eq!(d.cache_stats().stale_links, 0);
        d.cache().check().expect("consistent");
    }
}
