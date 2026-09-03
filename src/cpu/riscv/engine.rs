//! The translated execution engine: this hart's blocks, through [`jit`].
//!
//! `ROADMAP.md` §9's translation pipeline had a frontend
//! ([`lift`](super::lift)), a runtime ([`jit`]) and a backend
//! ([`jit::x86`](crate::jit::x86)) — and nothing joining them to a *machine*.
//! Both differential harnesses drove the dispatcher from a synthetic hart, so
//! the speedup the code generator measures was real and unreachable: every
//! core's `engine` property accepted only `"interp"`. This module is the join.
//! `engine = "jit"` on a `cpu.riscv` object reaches [`advance`], and
//! `machines/riscv-virt.machine` boots on it.
//!
//! [`jit`]: crate::jit
//!
//! # The claim, and what it costs to keep
//!
//! *A cache hit, a cache miss, an interpreted run and a compiled run are
//! indistinguishable to the guest, **including cycle counts**.*
//! (`ROADMAP.md` §0.) That is not a hope here, it is the shape of the file —
//! each rule below exists because the alternative diverges:
//!
//! * **The memory path is the interpreter's, literally.** [`IrHost::load`] and
//!   [`IrHost::store`] here call `Exec::load` and `Exec::store` — the same
//!   functions `Exec::step` calls, over the same [`mmu::Tlb`], charging the
//!   same ticks through the same `Exec::charge`. Not *a* memory path that
//!   agrees, *the* memory path. A second implementation would have to
//!   reproduce the misaligned split, the per-byte translation, the PMP check
//!   and a walk's tick cost, and `differential`'s own host is the evidence
//!   that reproducing them is a job rather than a line.
//! * **The entry fetch translation happens on every block execution**, not
//!   once at lift time. A translated block skips the fetch, but the
//!   interpreter's first fetch *translates*, and a translation that misses the
//!   hart's TLB walks and charges for the walk. A cached block that skipped it
//!   would run the same instructions for fewer ticks the second time round.
//!   That used to bound a dispatcher call to **one block**, because
//!   [`Frontend`] had no per-entry hook and a chained successor would have
//!   skipped its own translation — so block chaining, `ROADMAP.md` §9's second
//!   mechanism, never ran. It has one now ([`Frontend::enter`]), [`admit`] is
//!   what it calls, and a call runs up to [`CHAIN`] blocks.
//! * **A block never runs unless its worst case fits the budget left.**
//!   Otherwise the guest's *stopping point* inside a scheduler quantum would
//!   depend on the engine: an interpreter overruns its budget by one
//!   instruction and a trace by up to sixty-four, the overrun is carried as
//!   `State::debt`, and both numbers are in the snapshot a machine's state
//!   hash is taken over. So the tail of every quantum is interpreted and the
//!   two engines stop on the same instruction with the same debt. [`Costs`] is
//!   what keeps the bound tight enough for that tail to be short — and the
//!   guard is asked once per block of a chain, against what the chain has
//!   *left*, not once per [`advance`].
//! * **A pending interrupt is looked for at every block boundary**, chained
//!   ones included, which is what keeps a sixteen-block chain
//!   indistinguishable from sixteen one-block calls. A store into the CLINT or
//!   the PLIC ends its block by construction, so the interrupt it raises is
//!   seen before the next block starts.
//!
//! # What it buys, measured
//!
//! On the guest this exists for — `machines/riscv-virt.machine` booting
//! OpenSBI 1.6 and a Debian RISC-V `Image` with a busybox initramfs, 512 MiB of
//! DRAM, **four minutes of virtual time**, so well past the shell prompt — one
//! binary, three engines, back to back:
//!
//! | `engine` | wall clock | vs the interpreter |
//! | --- | --- | --- |
//! | `interp` | 150.6 s | — |
//! | `jit` | 115.6 s | **1.30×** |
//! | `jit-host` | 87.6 s | **1.72×** |
//!
//! Median of three **interleaved** reps — one of each engine, in turn, three
//! times over — because the interpreter is the control and a control measured
//! in a different sitting is not one. The nine runs spread by 0.5%; two sweeps
//! of the *same* binary taken twenty minutes apart on this machine differed by
//! 12%.
//!
//! All three end on the same guest state, byte for byte:
//! `state hash 0xf86f099b07119370`. `tests/riscv_virt_engines.rs` asserts that
//! equality on every commit, on a cheaper guest.
//!
//! **The host code generator used to lose to the portable one**, at 0.50× the
//! interpreter, and what fixed it was not the code it emits. Two things were.
//!
//! **Blocks are chained now.** [`Frontend::enter`] is the hook that allows it,
//! [`CHAIN`] is how far, and on this guest **86% of blocks are reached by
//! following a patched exit** — 43.2 million of 50.0 million over sixty
//! seconds of guest time, against a `chained` count that was previously zero
//! in every run. What that buys is not the hash lookup it skips; it is
//! everything around a four-instruction block that a one-block call had to
//! pay in full.
//!
//! **And a compile stopped costing 144 µs.** The 256 MiB code buffer this
//! engine asks for was being `mprotect`ed end to end twice per compiled block
//! — 433 600 cycles a block, against 13 850 for the code generation it was
//! protecting. `jit::x86::buf` flips a page-sized window instead, and the same
//! sixty seconds of guest time went from **25.6 billion cycles in `mprotect`
//! to 1.3 billion**. The two fixes of the previous round were fighting each
//! other: the buffer was grown to stop resets, and growing it made every flip
//! proportionally slower.
//!
//! What is left is honest and worth knowing. The compiled path's *execution*
//! was never the problem — measured over the same run it costs 768 cycles a
//! block against the IR interpreter's 896, so it was ahead before compilation
//! was counted. And the speedup is 1.72×, not the 8–22× the code generator
//! measures on a benchmark, because the last of `ROADMAP.md` §9's four
//! mechanisms is still out of reach: no [`LoadPlan`](crate::jit::LoadPlan) is
//! published (see [`FastMem`] below), so a guest access costs a call whichever
//! engine runs it. A block here lifts to about forty-seven IR instructions and
//! compiles to 949 bytes of host code; 59 269 of them are compiled in sixty
//! seconds and none is refused.
//!
//! # What is checked at a block boundary rather than at an instruction
//!
//! **Pending interrupts**, and it is worth being exact about why that is
//! sound. Within a block nothing this hart does can raise one: a CSR write, an
//! `MRET`, a `WFI` and an `SFENCE.VMA` are all outside the lifted subset and
//! end the block, a **store** ends the block by construction
//! ([`lift`](super::lift), "A store still ends the block"), and the platform
//! timer is a value another runnable publishes between quanta. What is left is
//! a **load** from a device that raises an interrupt as a side effect of being
//! read, which nothing on a `virt` board does — a PLIC claim and a 16550 `RBR`
//! read both *lower* a line. Recorded here rather than discovered later.
//!
//! The interrupt check is asked **per block**, and that is unchanged by
//! chaining: [`admit`] runs it at every boundary, so a chain is as prompt as
//! the one-block calls it replaced. The **safe-point flag** is not, and that
//! is the price of chaining, stated rather than implied: `Hart::run_budget`
//! tests it between calls to [`advance`], so it used to be honoured within one
//! block and is now honoured within [`CHAIN`] of them. See that constant.
//!
//! # Self-modifying code, and the one case that is not covered
//!
//! A store from a **translated block** is reported through [`StoreLog`] and
//! drained by the dispatcher at the next boundary. A store from an
//! **interpreted instruction** — an `AMO`, an `SC`, anything outside the
//! subset — is reported through the same `Exec` field the interpreter fills,
//! and [`advance`] drains it the same way. Between them that is every byte
//! *this hart* writes, invalidated by guest-physical page, which is the
//! contract `jit::dispatch` states: a host accumulates the pages **it** wrote.
//!
//! Bytes written by something that is **not** this hart — a DMA engine filling
//! a page cache, another hart — are outside that contract and are not caught
//! here. The obvious hook is `FENCE.I`, which is the only notice RISC-V gives
//! that bytes one did not write are now code, and it was tried: it makes the
//! block cache useless. Linux issues one from `flush_icache_pte` on essentially
//! every executable page it maps and from `switch_mm` besides — **39 442 of
//! them in thirty seconds of guest time**, one per ~750 blocks executed, each
//! throwing the whole cache away before it could warm. Measured: 3.6 million
//! translations instead of 59 thousand, and a boot that took thirty-eight
//! times as long as the interpreter's.
//!
//! So it is not the hook, and the honest statement is that this is a **known
//! gap** rather than a solved problem: a guest that demand-pages an executable
//! off a virtio disk and runs it at a virtual address whose *physical* page
//! previously held other code, without any store from this hart in between,
//! can execute a stale translation. Closing it needs a write notification from
//! the address space for masters that are not the CPU — a `core::space` seam
//! that does not exist — not another architectural instruction.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;

use crate::core::error::{BusError, Result};
use crate::core::exec::{Exit, ExitMask};
use crate::core::space::{AddressSpace, MemAttrs, MemResult};
use crate::core::value::Width;
use crate::ir::{Block, InsnStart, IrHost, MemOp, Opcode, RegSlot, verify};
use crate::jit::{
    BlockCache, DirtyPages, Dispatcher, Entry, Epoch, FastMem, Frontend, PAGE_MASK, Stop, StoreLog,
    Translation,
};

use super::Config;
use super::csr::{Lines, cause};
use super::exec::{Exec, State, Trap};
use super::lift::{self, Origin, PC, Shape};
use super::mmu::{self, Access};

/// How much of a block the frontend is allowed to swallow.
///
/// [`Shape::Trace`] is the dispatcher's shape: direct branches are merged, so
/// a loop unrolls into one translation and a guest register stays in a
/// temporary across the whole of it.
const SHAPE: Shape = Shape::Trace;

/// How many blocks one [`advance`] may chain before it hands control back.
///
/// `ROADMAP.md` §9's second mechanism is a block cache *"with block chaining
/// (patch the exit jump directly to the successor)"*, and until
/// [`Frontend::enter`] existed this engine could not use it: a chained
/// successor would have skipped the entry translation every block owes, so the
/// dispatcher had to be driven one block at a time and `DispatchStats::chained`
/// was zero in every run of a real guest. It is not a hash lookup this buys
/// back — that was never the expensive part — it is everything *around* a
/// block: an `Exec`, a `Host` and its register-file copy in and out, a
/// `Lifter`, a cache resynchronisation and a trip through `Hart::run_budget`,
/// all of which a four-instruction block used to pay in full.
///
/// **What it costs is the safe point, and the new bound is stated rather than
/// implied.** `Hart::run_budget` tests `ROADMAP.md` §4.7's exit flag between
/// calls to [`advance`], so a raised flag used to be honoured within one block
/// — at most [`lift::MAX_INSNS`] guest instructions. It is now honoured within
/// at most `CHAIN` blocks, so **1 024 guest instructions**, and still within
/// what is left of the quantum's tick budget, because every block of the chain
/// is admitted against that budget by [`admit`] before it runs.
/// `a_chain_is_bounded_by_the_stated_safe_point_number` asserts the block half
/// of that, and `the_same_agreement_holds_over_budgets_a_block_does_not_fit_in`
/// the tick half.
///
/// Sixteen rather than sixty-four because the curve is flat past it — the
/// per-block cost being amortized is a fixed overhead, so the second block of
/// a chain removes half of it and the sixteenth removes a fifteenth — and a
/// safe point is worth more than the last percent.
const CHAIN: usize = 16;

/// The most bus accesses one Sv39 walk can make: three levels of descriptor
/// reads and at most one accessed/dirty write-back.
///
/// Used only to bound a block's worst case, so a walk that makes fewer makes
/// the bound conservative rather than wrong.
const WALK_ACCESSES: u64 = 4;

/// How many blocks this hart's cache holds before it evicts.
///
/// `jit::BlockCache`'s own default is 8 192, and a Linux guest wants more than
/// that: measured over four minutes of guest time, 8 192 gave **1 096 143
/// insertions against 1 079 470 evictions** — a working set thrashing a cache
/// too small to hold it, where every eviction is a re-lift and, with a host
/// code generator attached, a re-compile. At 65 536 the same run inserts 138
/// 290 and evicts 55 825, and the compiled path stops spending most of its
/// time in the compiler.
///
/// The number is a bound, not an allocation: a board whose guest has a small
/// working set never fills it.
const BLOCKS: usize = 65536;

/// How many `(pc, key) -> worst-case ticks` answers are remembered.
///
/// Direct-mapped and keyed by the guest PC, exactly as the block cache is, and
/// sized with it so a resident block usually has a resident cost. A miss is a
/// conservative answer, never a wrong one.
const COST_SLOTS: usize = 65536;

/// How big a host code buffer this hart asks for: 256 MiB.
///
/// `jit::x86`'s own default is one mebibyte, sized against *"a block compiles
/// to a few hundred bytes"*. A real guest says otherwise. A RISC-V block under
/// Linux is a handful of guest instructions — a store ends a block, and so does
/// every instruction outside the lifted subset — and lifts to about forty-seven
/// IR instructions, which this backend emits **949 bytes** of host code for
/// (measured over 59 269 compiles of a Linux boot). [`BLOCKS`] live blocks
/// therefore want about sixty mebibytes, and the buffer is append-only and
/// reclaimed only by a reset that throws every compiled block away, so
/// undersizing it is not a small cost: at one mebibyte it reset **2 626 times
/// in ten seconds of guest time** and compiled nineteen thousand blocks 886 000
/// times, and at 32 MiB it reset 111 times in four minutes. At 256 MiB a
/// four-minute boot writes about 225 MB and resets once.
///
/// The mapping is anonymous, so what is not written is not resident, and it is
/// only asked for at all by `engine = "jit-host"`. Asking for too much costs
/// address space and **nothing per compile** — that last part is new, and it is
/// why the number can stand: `jit::x86::buf` flips a page-sized window rather
/// than the whole mapping, so a bigger buffer no longer makes every `mprotect`
/// slower. It did, and the two together were the largest single cost in the
/// compiled engine.
#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
const CODE_BUFFER: u64 = 256 << 20;

// ---------------------------------------------------------------------------
// What a hart keeps between blocks
// ---------------------------------------------------------------------------

/// This hart's translation state: the dispatcher, and the costs beside it.
///
/// **Derived state in the strict sense** (`ROADMAP.md` §4.5): never
/// serialized, and thrown away by a reset, by a snapshot restore and by a
/// debugger overwriting the CSR file. That is also what makes a snapshot
/// interchangeable between any two engines — there is nothing engine-specific
/// in one to interchange.
#[derive(Debug)]
pub(super) struct Jit {
    disp: Dispatcher,
    costs: Costs,
}

impl Jit {
    /// A fresh engine.
    ///
    /// `host_code` asks for the host code generator; a build or a host without
    /// one gets the portable backend instead, which is not a failure and not a
    /// different guest (`ROADMAP.md` §9, "Backends").
    pub(super) fn new(host_code: bool) -> Jit {
        let disp = Dispatcher::with_cache(BlockCache::with_capacity(BLOCKS));
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        let disp = match host_code
            .then(|| crate::jit::x86::Engine::with_capacity(CODE_BUFFER))
            .flatten()
        {
            Some(engine) => disp.with_backend(engine),
            None => disp,
        };
        let _ = host_code;
        Jit {
            disp,
            costs: Costs::new(),
        }
    }

    /// Throw every translation away.
    pub(super) fn flush(&mut self) {
        self.disp.cache_mut().flush();
        self.costs.clear();
    }

    /// Blocks executed, and how many of those ran as host code.
    pub(super) fn stats(&self) -> (u64, u64) {
        let s = self.disp.stats();
        (s.blocks, s.compiled)
    }
}

/// A direct-mapped table of `(pc, key) -> the most ticks that block can spend`,
/// with zero meaning *there is no block here*.
///
/// Two jobs, both about not paying for the same answer twice.
///
/// The budget guard needs an upper bound *before* a block runs, and computing
/// one means walking the block's ops — which costs more than running the
/// compiled block does. So it is computed once, where the block is lifted, and
/// remembered here. A collision loses an answer and costs a conservative
/// bound.
///
/// A recorded **zero** is the other job: the instruction at that PC is outside
/// the lifted subset, so there is nothing to translate and the interpreter
/// should be reached directly. Without it every `amoadd`, every `csrrw` and
/// every `ecall` costs a dispatcher round trip and a fresh [`lift::lift`] that
/// fails at its first instruction — measured at **42 million wasted lifts** in
/// four minutes of guest time on a Linux boot, against 1.1 million real ones,
/// because RISC-V atomics are not in the subset and a kernel is full of them.
/// Zero is a safe sentinel because a block that exists charges at least one
/// tick for its own fetch.
///
/// The negative half is discarded whenever a guest write invalidates a
/// translation, because the bytes it was an answer about may be different now.
#[derive(Debug)]
struct Costs {
    slots: Box<[Slot]>,
}

#[derive(Debug, Clone, Copy, Default)]
struct Slot {
    pc: u64,
    key: u64,
    ticks: u64,
    live: bool,
}

impl Costs {
    fn new() -> Costs {
        Costs {
            slots: vec![Slot::default(); COST_SLOTS].into_boxed_slice(),
        }
    }

    /// The low bit of a guest PC is always zero, so it carries nothing.
    #[inline]
    fn index(pc: u64) -> usize {
        ((pc >> 1) as usize) & (COST_SLOTS - 1)
    }

    #[inline]
    fn get(&self, pc: u64, key: u64) -> Option<u64> {
        let slot = &self.slots[Costs::index(pc)];
        (slot.live && slot.pc == pc && slot.key == key).then_some(slot.ticks)
    }

    #[inline]
    fn put(&mut self, pc: u64, key: u64, ticks: u64) {
        self.slots[Costs::index(pc)] = Slot {
            pc,
            key,
            ticks,
            live: true,
        };
    }

    fn clear(&mut self) {
        self.slots.fill(Slot::default());
    }
}

/// What one guest access can cost this hart, at worst.
///
/// One bus cycle when aligned; one per byte when it splits, which a hart that
/// performs misaligned accesses may do; and a page-table walk in front of each
/// of those when translation is on, because each byte of a split access is
/// translated on its own and may miss.
const fn per_access(cfg: &Config, translating: bool) -> u64 {
    let split = if cfg.misaligned { 8 } else { 1 };
    let walk = if translating { WALK_ACCESSES } else { 0 };
    split * (1 + walk)
}

/// The most ticks `block` can charge, read off its ops.
///
/// Every [`Opcode::CHARGE`] is counted even though a run takes one path
/// through a trace, and every access is charged its worst case, so this
/// over-estimates by construction — the only direction that is safe.
fn block_bound(block: &Block, access: u64, entry: u64) -> u64 {
    let mut ticks = entry;
    for inst in block.insts() {
        match inst.op {
            Opcode::CHARGE => {
                ticks = ticks.saturating_add(inst.imm.map_or(0, |c| c.bits() as u64));
            }
            Opcode::LD | Opcode::ST => ticks = ticks.saturating_add(access),
            _ => {}
        }
    }
    ticks
}

/// The bound for a block nothing is known about: the frontend's whole
/// instruction limit, every instruction an uncompressed access.
const fn worst_bound(cfg: &Config, translating: bool) -> u64 {
    let entry = if translating { WALK_ACCESSES } else { 0 };
    lift::MAX_INSNS as u64 * (2 + per_access(cfg, translating)) + entry
}

/// What names a block besides its guest PC: the world it was lifted in.
///
/// [`Origin::of`] answers this from `satp` and `Csrs::translation_gen`, and
/// under a real operating system that counter is **the wrong one** — not
/// unsound, but so pessimistic that the cache stops working. It is bumped by
/// every `SRET` and `MRET` and by every `sstatus` write, because `SUM`, `MXR`
/// and `MPRV` change what a translation *permits* and the hart's own TLB is
/// tagged with it. Linux returns from a trap thousands of times per
/// millisecond, so a block cache keyed on it misses every time, re-lifts
/// sixty-four instructions, and is slower than the interpreter it replaced.
/// Measured: a `riscv-virt` Linux boot ran four times slower under the JIT
/// than under the interpreter until this was keyed differently.
///
/// What actually names a block is the **physical page its bytes came from**,
/// which the entry translation has just resolved:
///
/// * a different mapping means different bytes, a different physical page and
///   a different key, so a stale block cannot be served;
/// * the same physical page with its bytes rewritten is caught by the block
///   cache's own self-modifying-code invalidation, which is *already* by
///   physical page;
/// * the same physical page with changed permissions is caught by the entry
///   translation, which is redone on every execution and faults before the
///   block runs;
/// * every access inside the block translates live, through the hart's MMU.
///
/// So this is strictly *more* precise than the generation rather than less: it
/// distinguishes exactly what decides the block's meaning and nothing else. It
/// rides in [`Origin::Paged`]'s field because that is the sanctioned encoder
/// for a block key ([`lift::key`]), and a second encoding of the same bits is
/// how two of them drift apart.
const fn key_origin(translating: bool, phys: u64) -> Origin {
    if translating {
        Origin::Paged {
            generation: phys >> 12,
        }
    } else {
        // Bare: the guest PC *is* the physical address, so the PC half of the
        // cache key already carries everything the page would.
        Origin::Bare
    }
}

// ---------------------------------------------------------------------------
// Admitting a block
// ---------------------------------------------------------------------------

/// What entering a block resolved to, once it is going to run.
///
/// The entry translation names the block ([`key_origin`]) and bounds what the
/// lifter may read, so it is carried rather than recomputed.
#[derive(Debug, Clone, Copy)]
struct Admitted {
    origin: Origin,
    key: u64,
    /// The virtual page the entry PC is on, and the physical page it resolved
    /// to. A block never leaves that page, so one translation covers every
    /// byte the lifter may read.
    page: u64,
    base: u64,
    /// What one guest access costs at worst, and what the entry fetch costs.
    access: u64,
    entry: u64,
}

/// Whether a block may run at `pc`, and what it costs to find out.
#[derive(Debug)]
enum Admit {
    /// It may.
    Ready(Admitted),
    /// It may not, and the reason is one the interpreter answers: a pending
    /// interrupt, a stalled `WFI`, an instruction outside the lifted subset,
    /// or a worst case that does not fit what is left of the budget.
    Interpret,
    /// The entry fetch itself faulted.
    Trap(Trap),
}

/// Everything a block owes before it runs — for the first block of a run and
/// for every chained successor alike, which is the whole point of it being one
/// function.
///
/// Three things happen here and the order is load-bearing.
///
/// **The interrupt check first.** A pending interrupt, a stalled `WFI` and an
/// instruction outside the lifted subset are all the interpreter's, and
/// `Exec::step` is how each is taken. Asking first is not an optimization:
/// `step` takes the trap, and a block run instead would take it up to
/// sixty-four instructions late. Asking it *per block* rather than per run is
/// what keeps a chained run indistinguishable from a sequence of one-block
/// ones: a store into the CLINT or the PLIC ends its block, and the interrupt
/// it raises is seen at the very next boundary.
///
/// **Then the entry fetch translation**, charged exactly as the interpreter's
/// first fetch charges it, and performed on every execution rather than at
/// lift time, because a cached block must cost what an uncached one cost. Two
/// bytes is the low halfword's width: `exec::fetch` translates for that first,
/// and translates again for the high half, which then hits.
///
/// **Then the budget guard**, and it is after the translation because the
/// translation is also what *names* the block. A guard that declined
/// afterwards has not wasted the walk: the interpreter's own fetch then finds
/// the entry this translation just filled, and charges exactly what it would
/// have charged anyway.
fn admit(cfg: &Config, costs: &Costs, exec: &mut Exec<'_>, pc: u64, remaining: u64) -> Admit {
    if exec.pending_interrupt().is_some() || exec.st.wfi {
        return Admit::Interpret;
    }
    let mode = exec.st.csrs.priv_mode;
    let translating = mmu::translation_active(&exec.st.csrs, mode);
    let phys = match exec.translate(pc, Access::Fetch, 2) {
        Ok(phys) => phys,
        Err(trap) => return Admit::Trap(trap),
    };
    let origin = key_origin(translating, phys);
    let key = lift::key(cfg, origin, SHAPE);

    // Known unliftable, or too big for what is left of the budget: either way
    // the interpreter takes this instruction, and reaching it without a lift
    // that fails at its first instruction is the whole point of remembering
    // the first.
    let bound = match costs.get(pc, key) {
        Some(0) => return Admit::Interpret,
        Some(bound) => bound,
        None => worst_bound(cfg, translating),
    };
    if bound > remaining.saturating_sub(exec.used) {
        return Admit::Interpret;
    }

    Admit::Ready(Admitted {
        origin,
        key,
        page: pc & !PAGE_MASK,
        base: phys & !PAGE_MASK,
        access: per_access(cfg, translating),
        entry: if translating { WALK_ACCESSES } else { 0 },
    })
}

// ---------------------------------------------------------------------------
// One step of the run loop
// ---------------------------------------------------------------------------

/// Execute a chain of blocks, or — where a block would be wrong — nothing, and
/// leave the instruction to the interpreter.
///
/// Reports the bus accesses charged and the [`Exit`] the step produced, in the
/// same currency and with the same meaning as `Hart::step_to_exit`, so a run
/// loop cannot tell which engine it is driving.
///
/// `remaining` is what is left of the caller's budget. A block whose worst
/// case does not fit is not run and the instruction is interpreted instead, so
/// that the hart stops where an interpreted hart would stop — and that holds
/// for every block of a chain, not only the first, because [`admit`] is asked
/// again at each boundary with what the chain has spent so far deducted.
///
/// # Panics
///
/// If a lifted block reaches an op the IR backend does not implement. That is
/// not a guest condition and not a host condition — it is this crate's own
/// frontend emitting something its own backend cannot execute — and the
/// architectural state at that point is not reconstructible, so it is reported
/// loudly rather than papered over.
#[allow(clippy::too_many_arguments)]
pub(super) fn advance(
    jit: &mut Jit,
    state: &mut State,
    tlb: &mut mmu::Tlb,
    space: &Arc<AddressSpace>,
    cfg: &Config,
    lines: &Lines,
    exits: ExitMask,
    remaining: u64,
) -> (u64, Option<Exit>) {
    let Jit { disp, costs } = jit;
    let mut exec = Exec::new(state, tlb, space, cfg, lines, exits);
    let pc = exec.st.pc;

    // The entry work for the *first* block, done here rather than through
    // [`Frontend::enter`], because the overwhelmingly common answer on a real
    // guest is "not a block at all" — an `amoadd`, a `csrrw`, an `ecall`,
    // forty-two million of them in four minutes — and reaching the interpreter
    // for one should not cost a frontend, a host and a dispatcher round trip.
    // The dispatcher's first `enter` is then a no-op; see `Lifter::admitted`.
    let at = match admit(cfg, costs, &mut exec, pc, remaining) {
        Admit::Ready(at) => at,
        Admit::Interpret => return interpret(disp, costs, exec),
        Admit::Trap(trap) => return deliver(disp, costs, exec, trap, pc, pc),
    };

    let mut front = Lifter {
        cfg,
        at,
        space,
        // Lifting reads *ahead* of the guest: up to sixty-four instructions it
        // has not asked for. A fetch is an ordinary access and a read-ahead is
        // not, so this is the one place in the core that reads guest memory
        // the way a debugger does — CLAUDE.md's "a debugger read must not pop
        // a FIFO" is exactly the hazard, and a NOR bank in its command state
        // is exactly the device. Nothing about the *translation* is relaxed:
        // that happened above, through the fetch path, with its walk and its
        // accessed bit.
        attrs: MemAttrs::DEBUG.with_requester(cfg.requester),
        costs,
        remaining,
        admitted: true,
        entry_trap: None,
        rejected: None,
    };

    let mut host = Host::new(&mut exec, pc);
    let run = match disp.run(&mut front, &mut host, pc, CHAIN) {
        Ok(run) => run,
        // The only refusal this frontend has is an RV32 configuration, which
        // the property reader rejects before a hart is built. Degrade rather
        // than fail the machine (`ROADMAP.md` §9).
        Err(_) => {
            drop(host);
            let Lifter { costs, .. } = front;
            return interpret(disp, costs, exec);
        }
    };
    let Host {
        slots, trap, mark, ..
    } = host;
    let Lifter {
        costs,
        entry_trap,
        rejected,
        ..
    } = front;
    debug_assert!(
        rejected.is_none(),
        "the RISC-V frontend emitted a block the verifier rejects: {rejected:?}"
    );

    if run.blocks == 0 {
        // Nothing executed: the instruction at `pc` is outside the lifted
        // subset, and `Frontend::translate` has just recorded that so the next
        // pass skips straight to here. The interpreter takes it, and its own
        // fetch translation now hits the TLB the translation above filled — so
        // what it charges is what a purely interpreted hart would have
        // charged.
        return interpret(disp, costs, exec);
    }

    // Every retired instruction, back into the architectural register file.
    // `x0` is hard-wired: the frontend never binds it, and forcing it here
    // costs one store and removes the question.
    exec.st.x[1..32].copy_from_slice(&slots[1..32]);
    exec.st.x[0] = 0;
    if exec.st.csrs.mcountinhibit & 0b100 == 0 {
        exec.st.csrs.minstret = exec.st.csrs.minstret.wrapping_add(run.insns as u64);
    }

    match run.stop {
        Stop::Fault(fault) => {
            // The block stopped *at* the faulting instruction with the
            // architectural state that instruction should see — which is what
            // `differential::precise_state` asserts, and why nothing is
            // reconstructed here. The trap is the one the memory path raised,
            // carrying the cause and the `mtval` the interpreter would carry.
            let trap = trap.unwrap_or(Trap {
                cause: cause::LOAD_ACCESS,
                tval: fault.pc,
            });
            let next = mark.map_or(fault.pc, |m| m.1);
            deliver(disp, costs, exec, trap, fault.pc, next)
        }
        Stop::Unsupported { op, at } => panic!(
            "the RISC-V frontend emitted {op} at index {at}, which the IR backend cannot execute"
        ),
        // A chained boundary whose entry fetch faulted. The instruction at
        // `run.pc` has not started, so its own PC is both where the trap is
        // taken and where it resumes — the same pair the prologue's `deliver`
        // above passes, and the same one `Exec::step`'s fetch would produce.
        Stop::Declined if entry_trap.is_some() => {
            let trap = entry_trap.expect("just tested");
            deliver(disp, costs, exec, trap, run.pc, run.pc)
        }
        // `Budget` ends a full chain, `Declined` a short one, and both leave
        // the guest at `run.pc` for the run loop to pick up. `Exit` cannot
        // happen: no safe-point flag is given to the dispatcher, because the
        // run loop above checks it between calls.
        _ => {
            exec.st.pc = cfg.xlen.trunc(run.pc);
            let used = exec.used;
            drain(disp, costs, &mut exec);
            (used.max(1), None)
        }
    }
}

/// Interpret one instruction, and tell the block cache what it wrote.
fn interpret(disp: &mut Dispatcher, costs: &mut Costs, mut exec: Exec<'_>) -> (u64, Option<Exit>) {
    let used = exec.step();
    let exit = exec.take_exit();
    drain(disp, costs, &mut exec);
    (used, exit)
}

/// Take a trap the block or the entry fetch raised, exactly as `Exec::step`
/// takes one: out of the hart when the mask says so, into the guest's handler
/// otherwise.
fn deliver(
    disp: &mut Dispatcher,
    costs: &mut Costs,
    mut exec: Exec<'_>,
    trap: Trap,
    at: u64,
    next: u64,
) -> (u64, Option<Exit>) {
    exec.this_pc = at;
    exec.next_pc = next;
    let out = match exec.exit_for(&trap) {
        Some(exit) => {
            exec.st.pc = exec.cfg.xlen.trunc(exit.resume_pc());
            Some(exit)
        }
        None => {
            exec.enter_trap(trap, false);
            exec.st.pc = exec.next_pc;
            None
        }
    };
    let used = exec.used;
    drain(disp, costs, &mut exec);
    (used.max(1), out)
}

/// Hand what an interpreted instruction wrote to the block cache.
///
/// The block path reports its stores through [`StoreLog`] and the dispatcher
/// drains those itself; this is the other half, for every instruction outside
/// the lifted subset — an `AMO`, an `SC`, a byte written by a trap handler
/// running through the interpreter.
fn drain(disp: &mut Dispatcher, costs: &mut Costs, exec: &mut Exec<'_>) {
    let mut hit = 0usize;
    for i in 0..exec.wrote_n as usize {
        hit += disp.cache_mut().note_write(exec.wrote[i], 1);
    }
    exec.wrote_n = 0;
    if hit > 0 {
        // A page a translation came from has changed, so every *negative*
        // answer in the cost table may have changed with it — an instruction
        // that was outside the subset can have been overwritten by one that is
        // not. Clearing the lot is a blunt answer to a rare event: eight
        // thousand of these in four minutes of guest time, against three
        // hundred million blocks.
        costs.clear();
    }
}

// ---------------------------------------------------------------------------
// The frontend
// ---------------------------------------------------------------------------

/// The RISC-V half of the dispatcher's contract, over a real hart.
struct Lifter<'a> {
    cfg: &'a Config,
    /// What the current block's entry resolved to — replaced at every
    /// boundary by [`Lifter::enter`], because a chained successor is on its
    /// own page, under its own key.
    at: Admitted,
    space: &'a AddressSpace,
    attrs: MemAttrs,
    costs: &'a mut Costs,
    /// What [`advance`] was given, so a chained boundary can guard the next
    /// block against what the chain has *left* rather than against the whole.
    remaining: u64,
    /// Whether [`advance`]'s prologue has already admitted the entry PC, so
    /// the dispatcher's first `enter` neither translates nor charges twice.
    /// Consumed by the first call and false ever after.
    admitted: bool,
    /// A trap raised by a *chained* boundary's entry fetch. The prologue's own
    /// trap never lands here — it is delivered before a dispatcher exists.
    entry_trap: Option<Trap>,
    /// The first block the verifier rejected, if any. A frontend bug rather
    /// than a guest one, so it is asserted on in a debug build and ignored in
    /// a release one — the block still runs, and the differential harness is
    /// where a malformed block is supposed to be caught.
    rejected: Option<String>,
}

impl<'h, 'e> Frontend<Host<'h, 'e>> for Lifter<'_> {
    fn epoch(&mut self) -> Epoch {
        Epoch {
            // Read live, at every boundary: a chained successor must not be
            // served out of a cache lifted through a topology a store in the
            // block before it replaced. One relaxed atomic load.
            topology: self.space.generation(),
            // Zero, and deliberately: `Epoch::translation` is what a cache
            // keyed on the guest MMU's generation is stale against, and these
            // blocks are not keyed on it — `key_origin` puts the physical page
            // in the key instead, which is both narrower and exact. A
            // generation here would ask for a full flush on every `SRET`.
            translation: 0,
        }
    }

    fn enter(&mut self, pc: u64, host: &mut Host<'h, 'e>) -> Result<Entry> {
        // The first block of a run was admitted by `advance` before this
        // frontend existed, and admitting it twice would translate twice and
        // charge the walk twice.
        if core::mem::take(&mut self.admitted) {
            return Ok(Entry::Ready);
        }
        // Registers live in the host's slots between the blocks of a chain and
        // are written back only when the run ends. Nothing `admit` reads is
        // one of them — it reads the CSR file, the hart's TLB and the tick
        // counter — so a chained boundary sees the same world a fresh
        // `advance` would have seen.
        let entry = match admit(self.cfg, self.costs, host.exec, pc, self.remaining) {
            Admit::Ready(at) => {
                self.at = at;
                Entry::Ready
            }
            Admit::Interpret => Entry::Leave,
            Admit::Trap(trap) => {
                self.entry_trap = Some(trap);
                Entry::Leave
            }
        };
        Ok(entry)
    }

    fn key(&mut self) -> u64 {
        self.at.key
    }

    fn pc_slot(&self) -> RegSlot {
        PC
    }

    fn translate(&mut self, pc: u64) -> Result<Translation> {
        let space = self.space;
        let attrs = self.attrs;
        let base = self.at.base;
        let page = self.at.page;
        let mut src = |addr: u64| {
            // Outside the entry page there is no translation to read through,
            // so the lifter is told the bytes are unreadable and ends the
            // block. It would have ended it at the page bound anyway; this is
            // the belt.
            if addr & !PAGE_MASK != page {
                return None;
            }
            space
                .read(base | (addr & PAGE_MASK), Width::U16, attrs)
                .ok()
                .map(|v| v as u16)
        };
        let lifted = lift::lift(
            self.cfg,
            self.at.origin,
            pc,
            &mut src,
            lift::MAX_INSNS,
            SHAPE,
        )?;
        if self.rejected.is_none()
            && let Err(e) = verify(&lifted.block)
        {
            self.rejected = Some(alloc::format!("{e}"));
        }
        // Zero when nothing could be lifted, which is what sends the next
        // pass straight to the interpreter instead of back through here.
        let bound = if lifted.insns > 0 {
            block_bound(&lifted.block, self.at.access, self.at.entry)
        } else {
            0
        };
        self.costs.put(pc, self.at.key, bound);
        Ok(Translation {
            page: base,
            insns: lifted.insns,
            block: lifted.block,
        })
    }
}

// ---------------------------------------------------------------------------
// The host
// ---------------------------------------------------------------------------

/// The guest state a block reads and writes, over the interpreter's own memory
/// path.
struct Host<'a, 'e> {
    exec: &'a mut Exec<'e>,
    slots: [u64; lift::SLOT_COUNT as usize],
    /// The trap the memory path raised, kept because [`IrHost::load`] can only
    /// report a [`BusError`] and a RISC-V trap is a cause *and* an `mtval`.
    trap: Option<Trap>,
    /// The last guest instruction boundary the block announced, as
    /// `(pc, next_pc)`. A fault names its own PC; the length that
    /// [`Exit::len`] is derived from comes from here.
    mark: Option<(u64, u64)>,
    dirty: DirtyPages,
}

impl<'a, 'e> Host<'a, 'e> {
    fn new(exec: &'a mut Exec<'e>, pc: u64) -> Host<'a, 'e> {
        let mut slots = [0u64; lift::SLOT_COUNT as usize];
        slots[..32].copy_from_slice(&exec.st.x);
        slots[PC.0 as usize] = pc;
        Host {
            exec,
            slots,
            trap: None,
            mark: None,
            dirty: DirtyPages::new(),
        }
    }

    /// Report a trap as the bus error the IR speaks, keeping the cause.
    fn fault(&mut self, trap: Trap) -> BusError {
        self.trap = Some(trap);
        BusError::BadAccess
    }

    /// Move whatever the last access wrote into the dirty log.
    ///
    /// Whatever landed, landed: a misaligned store that faulted on its second
    /// page still wrote the first, and a translation of those bytes is stale
    /// either way.
    fn note_writes(&mut self) {
        for i in 0..self.exec.wrote_n as usize {
            self.dirty.note(self.exec.wrote[i], 1);
        }
        self.exec.wrote_n = 0;
    }
}

impl IrHost for Host<'_, '_> {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        u128::from(self.slots[slot.0 as usize])
    }

    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        self.slots[slot.0 as usize] = value as u64;
    }

    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        match self.exec.load(addr, mem.size.bytes()) {
            Ok(v) => Ok(v),
            Err(trap) => Err(self.fault(trap)),
        }
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        let done = self.exec.store(addr, mem.size.bytes(), value);
        self.note_writes();
        match done {
            Ok(()) => Ok(()),
            Err(trap) => Err(self.fault(trap)),
        }
    }

    fn charge(&mut self, ticks: u64) {
        for _ in 0..ticks {
            self.exec.charge();
        }
    }

    fn insn_start(&mut self, mark: &InsnStart) {
        self.mark = Some((mark.pc, mark.next_pc));
    }
}

impl StoreLog for Host<'_, '_> {
    fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64)) {
        self.dirty.drain_dirty(sink);
    }
}

/// No inlined fast path, deliberately.
///
/// A backend that inlines a load skips [`IrHost::load`] entirely — and with it
/// this hart's page-table walk, its PMP check, and the ticks both spend. The
/// only world in which those are free is one with translation off and no PMP
/// entry that could refuse, and a plan whose validity is decided per access is
/// not a plan. `jit::fast`'s own documentation says not publishing one is the
/// honest default; the price is a call per guest access, which is what the
/// interpreter pays anyway.
impl FastMem for Host<'_, '_> {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{RamStore, Region};
    use crate::cpu::riscv::{Engine, Hart};

    /// How much RAM a test hart gets, at address zero.
    const RAM: u64 = 0x1_0000;

    /// x5 counts up, storing and reloading through x7, closed by a backward
    /// `jal`. Every instruction is in the lifted subset, the loop's back edge
    /// is direct so a trace merges it, and the store is what ends each block.
    const LOOP: [u32; 7] = [
        0x0000_0397, // auipc x7, 0        ; x7 = 0
        0x0000_0293, // addi  x5, x0, 0
        0x0010_0313, // addi  x6, x0, 1
        0x0062_82b3, // add   x5, x5, x6   ; the loop starts here
        0x0453_b823, // sd    x5, 80(x7)
        0x0503_be03, // ld    x28, 80(x7)
        0xff5f_f06f, // jal   x0, -12
    ];

    /// [`LOOP`] with its scratch word on a **different page** from its code.
    ///
    /// `LOOP` stores at 80, which is inside the page it was lifted from, so
    /// every block invalidates itself, every pass re-lifts, and nothing is
    /// ever chained. That makes it the right fixture for self-modifying code
    /// and a useless one for the block cache — measured, on the way to writing
    /// `a_chain_really_chains_now_that_a_boundary_has_a_hook`: 1 024 blocks,
    /// 1 024 translations and 1 024 invalidations. `lui x7, 1` moves the
    /// scratch word to 0x1050 and the loop starts behaving like code.
    const FAR_LOOP: [u32; 7] = [
        0x0000_13b7, // lui   x7, 1        ; x7 = 0x1000, the next page
        0x0000_0293, // addi  x5, x0, 0
        0x0010_0313, // addi  x6, x0, 1
        0x0062_82b3, // add   x5, x5, x6   ; the loop starts here
        0x0453_b823, // sd    x5, 80(x7)
        0x0503_be03, // ld    x28, 80(x7)
        0xff5f_f06f, // jal   x0, -12
    ];

    /// A hart with `program` at zero and [`RAM`] bytes of RAM under it.
    fn hart(engine: Engine, program: &[u32]) -> Hart {
        let ram = Arc::new(RamStore::new(RAM));
        for (i, word) in program.iter().enumerate() {
            for (j, byte) in word.to_le_bytes().iter().enumerate() {
                ram.write_u8((i * 4 + j) as u64, *byte).expect("in range");
            }
        }
        let space = AddressSpace::new("mem", 64);
        space
            .topology()
            .map(Region::ram("ram", ram), 0)
            .expect("nothing else is mapped");
        let hart = Hart::new(Config::rv64gc().with_reset_vector(0)).with_engine(engine);
        hart.attach_space(Arc::new(space));
        hart
    }

    /// Run both harts on the same budgets and compare everything a guest, a
    /// snapshot or a state hash can see.
    ///
    /// Every case runs twice, once against each JIT engine: the host code
    /// generator is a third implementation of the same block, and the claim is
    /// about all three.
    fn agree(program: &[u32], budget: u64, quanta: usize) -> (Hart, Hart) {
        let out = agree_on(Engine::Jit, program, budget, quanta);
        agree_on(Engine::JitHost, program, budget, quanta);
        out
    }

    fn agree_on(engine: Engine, program: &[u32], budget: u64, quanta: usize) -> (Hart, Hart) {
        let interp = hart(Engine::Interp, program);
        let jit = hart(engine, program);
        for n in 0..quanta {
            let a = interp.run_budget(budget);
            let b = jit.run_budget(budget);
            assert_eq!(
                a, b,
                "quantum {n}: {engine:?} and the interpreter consumed different budgets"
            );
        }
        for n in 0..32 {
            assert_eq!(
                interp.x(n),
                jit.x(n),
                "x{n} under {engine:?}: the interpreter says {:#018x}, the JIT says {:#018x}",
                interp.x(n),
                jit.x(n),
            );
        }
        assert_eq!(interp.pc(), jit.pc(), "the program counter");
        assert_eq!(
            interp.cycles(),
            jit.cycles(),
            "the cycle counter. A compiled block must charge exactly what an \
             interpreted one charges (ROADMAP.md §0)"
        );
        assert_eq!(interp.instret(), jit.instret(), "instructions retired");
        assert_eq!(
            interp.cycle_debt(),
            jit.cycle_debt(),
            "the carried overrun. A block that ran past the budget where an \
             instruction would not have puts the two engines on different \
             instructions for the rest of the run"
        );
        assert_eq!(interp.csrs().mcause, jit.csrs().mcause, "the trap cause");
        assert_eq!(interp.csrs().mepc, jit.csrs().mepc, "mepc");
        assert_eq!(interp.csrs().mtval, jit.csrs().mtval, "mtval");
        (interp, jit)
    }

    #[test]
    fn a_translated_hart_and_an_interpreted_one_agree_on_every_column() {
        let (interp, jit) = agree(&LOOP, 1000, 64);
        assert!(interp.cycles() > 1000, "the run was too short to mean much");
        let (blocks, _) = jit.jit_stats().expect("a JIT hart keeps statistics");
        assert!(blocks > 0, "no block ran, so nothing was compared");
    }

    #[test]
    fn a_chained_run_agrees_with_the_interpreter_on_every_column_too() {
        // The columns `agree` compares are the ones a chain could move: the
        // cycle counter, because a chained successor still owes its entry
        // translation; the retired count, because it is now summed over
        // several blocks; and the carried debt, because the budget guard is
        // now asked once per block rather than once per `advance`.
        let (interp, jit) = agree(&FAR_LOOP, 1000, 64);
        assert!(interp.cycles() > 1000, "the run was too short to mean much");
        assert_eq!(interp.x(28), jit.x(28), "the value reloaded from memory");
    }

    #[test]
    fn the_same_agreement_holds_over_budgets_a_block_does_not_fit_in() {
        // Twelve ticks is under the worst case of every block here, so the
        // budget guard interprets almost everything — which must give the same
        // answer as interpreting everything, and as blocks.
        agree(&LOOP, 12, 400);
    }

    #[test]
    fn a_fault_in_the_middle_of_a_block_traps_where_the_interpreter_traps() {
        // The same loop with its data pointer moved off the end of RAM, so the
        // store faults — mid-block, after two instructions have retired into
        // temporaries that only the fault path publishes.
        let mut program = LOOP;
        program[0] = 0x0001_0397; // auipc x7, 0x10 ; x7 = 0x10000, past the RAM
        let (interp, jit) = agree(&program, 1000, 8);
        assert_ne!(interp.csrs().mcause, 0, "the fixture never faulted");
        assert_eq!(jit.csrs().mcause, interp.csrs().mcause);
    }

    #[test]
    fn a_store_into_the_code_page_is_honoured_by_the_next_block() {
        // `sd` writes x5 over the instruction at 0x18 (the `jal`), which the
        // block was lifted from. The next time round the loop both engines
        // must execute the new bytes.
        let program = [
            0x0000_0397, // auipc x7, 0
            0x0000_0293, // addi  x5, x0, 0
            0x0010_0313, // addi  x6, x0, 1
            0x0062_82b3, // add   x5, x5, x6
            0x0183_b823, // sd    x5, 16(x7)   ; over 0x10 and 0x14
            0x0000_0013, // nop
            0xff1f_f06f, // jal   x0, -16
        ];
        agree(&program, 1000, 32);
    }

    #[test]
    fn an_instruction_outside_the_subset_is_interpreted_and_charged_the_same() {
        // `csrr x5, mcycle` is outside the lifted subset, so every pass round
        // this loop leaves the block cache and comes back.
        let program = [
            0x0000_0397, // auipc x7, 0
            0xb002_92f3, // csrrs x5, mcycle, x5
            0x0053_b823, // sd    x5, 16(x7)
            0xff9f_f06f, // jal   x0, -8
        ];
        agree(&program, 1000, 32);
    }

    /// A block that ends by jumping past the end of RAM, so the *next*
    /// boundary's entry fetch faults.
    ///
    /// The fault is at a chained boundary rather than in [`advance`]'s
    /// prologue, which is the only way to reach [`Admit::Trap`] through
    /// [`Frontend::enter`]. `mtvec` is zero, so the trap lands back at the top
    /// and the fixture loops.
    const FETCH_FAULT: [u32; 2] = [
        0x0002_03b7, // lui  x7, 0x20     ; x7 = 0x20000, past the end of RAM
        0x0003_8067, // jalr x0, 0(x7)
    ];

    /// [`FAR_LOOP`] with its load and store misaligned by one byte.
    ///
    /// `Config::misaligned` says this core performs them, byte by byte, and
    /// each byte is a bus access — so one of these costs eight ticks where an
    /// aligned one costs one, and a budget guard that assumed alignment would
    /// admit a block into eight times too little budget.
    const MISALIGNED: [u32; 7] = [
        0x0000_13b7, // lui   x7, 1       ; x7 = 0x1000, the next page
        0x0000_0293, // addi  x5, x0, 0
        0x0010_0313, // addi  x6, x0, 1
        0x0062_82b3, // add   x5, x5, x6  ; the loop starts here
        0x0453_b8a3, // sd    x5, 81(x7)
        0x0513_be03, // ld    x28, 81(x7)
        0xff5f_f06f, // jal   x0, -12
    ];

    /// `addi x5, x5, 1` — one liftable instruction, used to fill a page.
    const COUNT_UP: u32 = 0x0012_8293;

    /// A hart taken apart, so that [`advance`] and [`admit`] can be driven
    /// directly.
    ///
    /// `Hart` is the wrong instrument for three of the four answers [`admit`]
    /// gives, and that is not an accident of its API: an interrupt arrives
    /// over a wire, a `WFI` from an instruction, and a *cold* cost table only
    /// exists before the first lift. Each is one line here.
    struct Bench {
        space: Arc<AddressSpace>,
        cfg: Config,
        state: State,
        tlb: mmu::Tlb,
        lines: Lines,
        jit: Jit,
    }

    impl Bench {
        /// `program` at address zero, [`RAM`] bytes of RAM under it.
        fn new(program: &[u32]) -> Bench {
            let ram = Arc::new(RamStore::new(RAM));
            for (i, word) in program.iter().enumerate() {
                for (j, byte) in word.to_le_bytes().iter().enumerate() {
                    ram.write_u8((i * 4 + j) as u64, *byte).expect("in range");
                }
            }
            let space = AddressSpace::new("mem", 64);
            space
                .topology()
                .map(Region::ram("ram", ram), 0)
                .expect("nothing else is mapped");
            let cfg = Config::rv64gc().with_reset_vector(0);
            Bench {
                space: Arc::new(space),
                state: State::new(&cfg),
                cfg,
                tlb: mmu::Tlb::new(),
                lines: Lines::default(),
                jit: Jit::new(false),
            }
        }

        /// Write `value` into guest physical memory, for building page tables.
        fn poke(&self, addr: u64, value: u64) {
            for i in 0..8 {
                self.space
                    .write(
                        addr + i,
                        Width::U8,
                        (value >> (8 * i)) & 0xff,
                        MemAttrs::DEFAULT,
                    )
                    .expect("in RAM");
            }
        }

        /// The same bench with Sv39 translation on, `program` at **physical**
        /// `0x4000`, and virtual page zero mapped to it.
        ///
        /// Three levels, root at 0x1000, exactly as `mmu`'s own fixture builds
        /// them. It exists because nothing else in `cargo test` runs a
        /// **paged** JIT hart: the block key, the walk's tick cost and the
        /// per-access split all take a different branch under translation, and
        /// until this fixture every one of them was reached only by a Linux
        /// guest nobody's `cargo test` downloads.
        fn paged(program: &[u32]) -> Bench {
            use super::super::mmu::pte;
            let mut b = Bench::new(&[]);
            // No PMP entries, as `mmu`'s own Sv39 fixture does it: with
            // entries implemented and none configured, an S-mode access is
            // denied and the walk under test never happens.
            b.cfg.pmp_count = 0;
            b.state = State::new(&b.cfg);
            for (i, word) in program.iter().enumerate() {
                b.poke(0x4000 + (i as u64) * 4, u64::from(*word));
            }
            let leaf = pte::V | pte::R | pte::X | pte::A;
            b.poke(0x1000, ((0x2000 >> 12) << 10) | pte::V);
            b.poke(0x2000, ((0x3000 >> 12) << 10) | pte::V);
            b.poke(0x3000, ((0x4000 >> 12) << 10) | leaf);
            b.state.csrs.satp = (8 << 60) | (0x1000 >> 12);
            b.state.csrs.priv_mode = super::super::csr::Priv::Supervisor;
            b.state.pc = 0;
            b
        }

        /// [`Bench::paged`] with its whole mapped page full of [`COUNT_UP`].
        ///
        /// A block never leaves the page it started on ([`lift`]), so the last
        /// block on this page ends *at the page boundary* and exits to
        /// `0x1000` — which nothing maps. That makes the fault land at a
        /// **chained** boundary, which is the only way to reach
        /// [`Admit::Trap`] through [`Frontend::enter`].
        ///
        /// Two earlier shapes of this fixture reached the *prologue*'s trap
        /// instead, and both are worth writing down because either mistake
        /// silently tests nothing:
        ///
        /// * a `jalr` to an unmapped page — but `jalr` is outside the lifted
        ///   subset, so the instruction before the jump is interpreted and the
        ///   fault lands in the next `advance`'s prologue;
        /// * entering at the *start* of the page — 1 024 instructions is
        ///   exactly [`CHAIN`] × [`lift::MAX_INSNS`], so the chain runs out of
        ///   budget on the boundary before the fault.
        ///
        /// Entering sixty-four instructions from the end makes the second
        /// boundary of the first chain the faulting one, which is the arm.
        fn paged_off_the_end() -> Bench {
            let mut b = Bench::paged(&[]);
            let pair = u64::from(COUNT_UP) | (u64::from(COUNT_UP) << 32);
            for i in 0..512 {
                b.poke(0x4000 + i * 8, pair);
            }
            b.state.pc = 0x1000 - (lift::MAX_INSNS as u64) * 4;
            b
        }

        /// Run `engine` until the guest takes a trap, and say what it cost.
        ///
        /// The tick count *to the trap* is the measurement: delivering an
        /// entry fault a round trip late walks the same faulting fetch twice,
        /// and a walk that faults fills no TLB entry to make the second one
        /// free.
        fn run_until_trap(&mut self, engine: fn(&mut Bench, u64) -> u64) -> u64 {
            let mut used = 0;
            while self.state.csrs.mcause == 0 && used < 100_000 {
                let n = engine(self, 100_000 - used);
                if n == 0 {
                    break;
                }
                used += n;
            }
            used
        }

        /// Ask [`admit`] what it would do at the current PC.
        fn admit(&mut self, remaining: u64) -> Admit {
            let Bench {
                space,
                cfg,
                state,
                tlb,
                lines,
                jit,
            } = self;
            let mut exec = Exec::new(state, tlb, space, cfg, lines, ExitMask::NONE);
            let pc = exec.st.pc;
            admit(cfg, &jit.costs, &mut exec, pc, remaining)
        }

        /// One [`advance`], reporting what it consumed.
        fn advance(&mut self, budget: u64) -> u64 {
            let Bench {
                space,
                cfg,
                state,
                tlb,
                lines,
                jit,
            } = self;
            advance(jit, state, tlb, space, cfg, lines, ExitMask::NONE, budget).0
        }

        /// One interpreted instruction, reporting what it consumed.
        fn step(&mut self) -> u64 {
            let Bench {
                space,
                cfg,
                state,
                tlb,
                lines,
                ..
            } = self;
            Exec::new(state, tlb, space, cfg, lines, ExitMask::NONE).step()
        }
    }

    /// Drive [`advance`] directly, so the dispatcher's own statistics are
    /// reachable — `Hart::jit_stats` reports only two of them.
    ///
    /// Returns the statistics and the ticks the run consumed.
    fn drive(program: &[u32], budget: u64, calls: usize) -> (crate::jit::DispatchStats, u64) {
        let mut b = Bench::new(program);
        let mut used = 0;
        for _ in 0..calls {
            used += b.advance(budget);
        }
        (b.jit.disp.stats(), used)
    }

    #[test]
    fn a_pending_interrupt_keeps_a_block_from_running_at_any_boundary() {
        // `admit` asks this first, and asking it *per block* is the whole
        // argument that a sixteen-block chain is as prompt as sixteen
        // one-block calls: `Exec::step` is what takes the trap, and a block
        // run instead would take it up to `lift::MAX_INSNS` instructions late.
        use crate::cpu::riscv::csr::{irq, status};
        let mut b = Bench::new(&FAR_LOOP);
        assert!(
            matches!(b.admit(u64::MAX), Admit::Ready(_)),
            "the fixture has to be liftable, or this proves nothing"
        );
        b.state.csrs.mie |= irq::MTI;
        b.state.csrs.mstatus |= status::MIE;
        b.lines.set_pending(irq::MTI, true);
        assert!(
            matches!(b.admit(u64::MAX), Admit::Interpret),
            "a hart with a pending machine timer interrupt must reach the \
             interpreter, whatever is liftable at its PC"
        );
    }

    #[test]
    fn a_stalled_wfi_keeps_a_block_from_running_too() {
        // The other half of the same guard. A `WFI` that has not been woken is
        // not an instruction a block can retire past.
        let mut b = Bench::new(&FAR_LOOP);
        assert!(matches!(b.admit(u64::MAX), Admit::Ready(_)));
        b.state.wfi = true;
        assert!(matches!(b.admit(u64::MAX), Admit::Interpret));
    }

    #[test]
    fn a_block_nothing_is_known_about_is_guarded_by_the_whole_worst_case() {
        // The `None` arm of the cost lookup, which is every block's *first*
        // execution. Guarding it with anything less than `worst_bound` lets a
        // block spend more than the quantum had left, and the two engines then
        // stop on different instructions with different debt — the divergence
        // `ROADMAP.md` §0 forbids.
        let worst = worst_bound(&Config::rv64gc(), false);
        let mut b = Bench::new(&FAR_LOOP);
        assert!(
            matches!(b.admit(worst - 1), Admit::Interpret),
            "a budget one short of the worst case must decline"
        );
        assert!(
            matches!(b.admit(worst), Admit::Ready(_)),
            "and exactly enough must be enough"
        );
    }

    #[test]
    fn a_remembered_cost_lets_a_block_run_in_a_budget_its_worst_case_would_not_fit() {
        // What [`Costs`] is *for*: after one lift the guard knows this block
        // spends a handful of ticks rather than the frontend's whole
        // instruction limit, and a budget between the two admits it. A cost
        // filed under the wrong key is a table that never answers, and the
        // engine silently falls back to interpreting almost everything.
        let worst = worst_bound(&Config::rv64gc(), false);
        let mut b = Bench::new(&FAR_LOOP);
        // Warm the table: one generous call, which lifts and files the cost.
        b.advance(u64::MAX / 2);
        let before = b.jit.disp.stats().blocks;
        assert!(before > 0, "nothing ran, so nothing was costed");
        // Now a budget the *worst* case does not fit but the real one does.
        for _ in 0..8 {
            b.advance(worst / 4);
        }
        assert!(
            b.jit.disp.stats().blocks > before,
            "no block ran under a budget its remembered cost fits: the cost \
             table is not answering ({:?})",
            b.jit.disp.stats()
        );
    }

    #[test]
    fn an_entry_fetch_that_faults_traps_where_the_interpreter_traps() {
        // The `Admit::Trap` arm, and the only path that reaches it: a block
        // that *ends* by jumping somewhere unmapped, so the fault happens at
        // the next boundary of a chain rather than in `advance`'s prologue.
        // Interpreting instead would walk and charge for the fetch twice —
        // once here and once in `Exec::step` — because a faulting translation
        // fills no TLB entry.
        let (interp, jit) = agree(&FETCH_FAULT, 1000, 8);
        assert_ne!(
            interp.csrs().mcause,
            0,
            "the fixture never faulted, so it tests nothing"
        );
        assert_eq!(interp.csrs().mepc, jit.csrs().mepc);
        assert_eq!(
            interp.csrs().mtval,
            jit.csrs().mtval,
            "and the faulting address, which is the PC the chain jumped to"
        );
    }

    #[test]
    fn a_misaligned_access_is_costed_at_what_it_can_really_spend() {
        // `per_access` says a misaligned access on this core is one bus cycle
        // *per byte*, and the budget guard has to believe it: a block costed
        // as though every access were aligned is admitted into a budget it
        // then overruns. Every column of `agree` — cycles, debt, the stopping
        // instruction — moves when that happens.
        //
        // Two things have to be true at once for the mis-costing to show, and
        // getting either wrong lets the mutant walk out — both did, in turn:
        //
        // * the budget must sit **between** the wrong bound and the right one,
        //   and those are *measured*, not guessed: this fixture's hot block is
        //   four instructions with two accesses, so eight ticks of fetch plus
        //   either two (an access costed as aligned) or sixteen. Ten and
        //   twenty-four. Two budgets picked by eye fell outside that window
        //   and the mutant walked out of both;
        // * and the cost table must already **know** this block, because a
        //   cold table answers `worst_bound` — 640 ticks — which no small
        //   budget admits, so nothing is ever lifted and both engines
        //   interpret everything in perfect agreement.
        //
        // And even inside the window the budget has to be one where the
        // *interpreter* would stop somewhere else: at sixteen it runs the same
        // four instructions and overruns to twenty-four exactly as the block
        // does, because the last instruction is the expensive one. So the
        // budget is swept rather than picked.
        //
        // So: warm at a budget everything fits, then squeeze.
        for (engine, squeeze) in [Engine::Jit, Engine::JitHost]
            .into_iter()
            .flat_map(|e| [10, 12, 14, 18, 20, 22].map(|b| (e, b)))
        {
            let interp = hart(Engine::Interp, &MISALIGNED);
            let jit = hart(engine, &MISALIGNED);
            for _ in 0..8 {
                assert_eq!(interp.run_budget(1000), jit.run_budget(1000));
            }
            for n in 0..200 {
                assert_eq!(
                    interp.run_budget(squeeze),
                    jit.run_budget(squeeze),
                    "quantum {n} under {engine:?} at a budget of {squeeze}"
                );
            }
            assert_eq!(interp.cycles(), jit.cycles(), "the cycle counter");
            assert_eq!(interp.cycle_debt(), jit.cycle_debt(), "the carried debt");
            assert_eq!(interp.instret(), jit.instret(), "instructions retired");
            for r in 0..32 {
                assert_eq!(interp.x(r), jit.x(r), "x{r} under {engine:?}");
            }
        }
    }

    #[test]
    fn a_chained_entry_fetch_that_page_faults_traps_where_the_interpreter_traps() {
        // The [`Admit::Trap`] arm, and the only hart that can reach it: in
        // bare mode with no PMP `Exec::translate` cannot fail, so a bad PC
        // faults later, in the *fetch*, and never through `admit` at all. A
        // paged hart faults in the walk.
        //
        // The block here ends by jumping to a virtual page nothing maps, so
        // the fault is at the chain's next boundary. Handing the PC back
        // instead of delivering it there would walk and charge for the same
        // fetch twice, because a faulting translation fills no TLB entry —
        // which is a cycle-counter divergence, and `ROADMAP.md` §0 makes that
        // a state-hash divergence.
        let mut jit = Bench::paged_off_the_end();
        let mut oracle = Bench::paged_off_the_end();
        let jit_ticks = jit.run_until_trap(Bench::advance);
        let oracle_ticks = oracle.run_until_trap(|b, _| b.step());
        assert_ne!(
            jit.state.csrs.mcause, 0,
            "the fixture never faulted, so it tests nothing"
        );
        assert_eq!(jit.state.csrs.mcause, oracle.state.csrs.mcause, "the cause");
        assert_eq!(
            jit.state.csrs.mepc, oracle.state.csrs.mepc,
            "mepc, which is the PC the chain fell through to and not the one \
             it started at"
        );
        assert_eq!(jit.state.csrs.mtval, oracle.state.csrs.mtval, "mtval");
        assert_eq!(
            jit.state.csrs.minstret, oracle.state.csrs.minstret,
            "instructions retired before the trap"
        );
        assert_eq!(
            jit_ticks, oracle_ticks,
            "and the ticks the trap cost to reach: a chained entry fault \
             delivered a round trip late walks the same fetch twice"
        );
    }

    #[test]
    fn a_paged_block_is_named_by_the_physical_page_its_entry_resolved_to() {
        // `key_origin`, which is the fix a Linux boot needed: keyed on
        // `Csrs::translation_gen` the cache missed on every `SRET` and ran
        // four times slower than the interpreter. Keyed on the physical page
        // the entry translation just produced, it is exact — and until this
        // fixture existed nothing but that Linux boot ever took the branch.
        let mut b = Bench::paged(&FAR_LOOP);
        let Admit::Ready(at) = b.admit(u64::MAX) else {
            panic!("the mapped page must be liftable");
        };
        assert_eq!(at.base, 0x4000, "the bytes come from the mapped page");
        assert_eq!(at.origin, Origin::Paged { generation: 0x4 });
        assert_ne!(
            at.key,
            lift::key(&b.cfg, Origin::Bare, SHAPE),
            "a paged block must not key identically to a bare one at the same PC"
        );
        assert_eq!(at.entry, WALK_ACCESSES, "and the walk is charged for");
        assert_eq!(at.access, per_access(&b.cfg, true));

        // The same virtual page, a different physical one. The bytes have
        // changed, so the key must: a stale block must be unreachable rather
        // than merely unlikely.
        use super::super::mmu::pte;
        b.poke(
            0x3000,
            ((0x5000 >> 12) << 10) | pte::V | pte::R | pte::X | pte::A,
        );
        b.tlb.flush();
        let Admit::Ready(moved) = b.admit(u64::MAX) else {
            panic!("still liftable");
        };
        assert_ne!(moved.key, at.key, "a remapped page is a different block");
        assert_eq!(moved.base, 0x5000);
    }

    #[test]
    fn a_topology_change_throws_the_translations_away() {
        // `Frontend::epoch` is read at every boundary and answered from the
        // address space's own generation. Answering it with a constant — or
        // reading it once per run — leaves a chained successor executing a
        // block lifted through a topology that no longer exists.
        let mut b = Bench::new(&FAR_LOOP);
        for _ in 0..8 {
            b.advance(u64::MAX / 2);
        }
        let before = b.jit.disp.stats().translated;
        let resyncs = b.jit.disp.stats().resyncs;
        assert!(before > 0, "nothing was translated, so nothing can be lost");
        b.space
            .topology()
            .map(
                Region::ram("more", Arc::new(RamStore::new(0x1000))),
                0x20000,
            )
            .expect("that address is free");
        b.advance(u64::MAX / 2);
        assert_eq!(
            b.jit.disp.stats().resyncs,
            resyncs + 1,
            "the epoch move was missed"
        );
        assert!(
            b.jit.disp.stats().translated > before,
            "the cache survived a retopology"
        );
    }

    #[test]
    fn the_safe_point_bound_is_the_number_the_documentation_states() {
        // `CHAIN` is a tunable and this is not a second opinion about its
        // value: it is the claim that goes with it. `ROADMAP.md` §4.7's flag
        // is tested between `advance` calls, so what one call may execute is
        // the bound, and the docs say 1 024 guest instructions. Moving the
        // constant without moving the claim is what this catches.
        assert_eq!(CHAIN * lift::MAX_INSNS, 1024);
    }

    #[test]
    fn a_chain_really_chains_now_that_a_boundary_has_a_hook() {
        // The measurement this exists for: `DispatchStats::chained` was zero
        // in *every* run of a real guest, because a dispatcher without
        // `Frontend::enter` had to be driven one block at a time. `LOOP` is
        // four blocks round a back edge, so after the first pass every edge is
        // a patched exit.
        let (stats, _) = drive(&FAR_LOOP, 100_000, 64);
        assert!(stats.blocks > 64, "the run was too short to chain anything");
        assert!(
            stats.chained > 0,
            "no block was reached by following a patched exit: {stats:?}"
        );
        // Per `advance` call, at most one block is looked up or translated and
        // the rest are followed, so the chained share is the bulk of it.
        assert!(
            stats.chained > stats.blocks / 2,
            "chaining is reaching only {} of {} blocks: {stats:?}",
            stats.chained,
            stats.blocks
        );
    }

    #[test]
    fn a_chain_is_bounded_by_the_stated_safe_point_number() {
        // `Hart::run_budget` tests the safe-point flag between `advance`
        // calls, so what one call may execute *is* the safe-point bound
        // (`ROADMAP.md` §4.7). One call, an unbounded budget, and the answer
        // must be `CHAIN` rather than "as many as the budget allowed".
        let (stats, _) = drive(&FAR_LOOP, u64::MAX / 2, 1);
        assert!(
            stats.blocks <= CHAIN as u64,
            "one advance ran {} blocks, and the safe point is stated as {CHAIN}",
            stats.blocks
        );
        assert_eq!(
            stats.blocks, CHAIN as u64,
            "and a loop with nothing to decline should reach the bound exactly"
        );
    }

    #[test]
    fn a_chain_never_spends_more_than_the_budget_it_was_given() {
        // The other half of the bound, and the one that keeps the two engines
        // on the same instruction: every block of a chain is admitted against
        // what the chain has *left*, not against what it started with. A
        // budget one block cannot fit means no block runs at all.
        for budget in [1, 4, 12, 64, 512] {
            let (stats, used) = drive(&FAR_LOOP, budget, 1);
            assert!(
                used <= budget.max(1) + worst_bound(&Config::rv64gc(), false),
                "a run given {budget} spent {used} over {} blocks",
                stats.blocks
            );
        }
    }

    #[test]
    fn a_hart_that_never_ran_has_no_statistics_and_one_that_did_has_some() {
        let jit = hart(Engine::Jit, &LOOP);
        assert_eq!(jit.jit_stats(), None, "nothing has run yet");
        jit.run_budget(1000);
        assert!(jit.jit_stats().is_some());
    }

    #[test]
    fn the_host_code_generator_is_asked_for_only_by_the_engine_that_names_it() {
        // `jit` and `jit-host` are the same guest and a different backend, and
        // the statistic is the only thing that can tell them apart — which is
        // exactly why it exists.
        let plain = hart(Engine::Jit, &LOOP);
        let host = hart(Engine::JitHost, &LOOP);
        for _ in 0..16 {
            plain.run_budget(1000);
            host.run_budget(1000);
        }
        let (blocks, compiled) = plain.jit_stats().expect("statistics");
        assert!(blocks > 0);
        assert_eq!(compiled, 0, "`jit` must not reach for a code generator");
        let (blocks, compiled) = host.jit_stats().expect("statistics");
        assert!(blocks > 0);
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(
            compiled > 0,
            "`jit-host` on a host with a code generator must compile something"
        );
        #[cfg(not(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64")))]
        assert_eq!(compiled, 0, "no backend on this host, so nothing compiles");
    }

    #[test]
    fn a_reset_throws_the_translations_away_and_the_run_still_agrees() {
        use crate::core::device::{Device, ResetKind};
        let interp = hart(Engine::Interp, &LOOP);
        let jit = hart(Engine::JitHost, &LOOP);
        for _ in 0..8 {
            interp.run_budget(1000);
            jit.run_budget(1000);
        }
        let (before, _) = jit.jit_stats().expect("statistics");
        assert!(
            before > 0,
            "nothing was cached, so there is nothing to drop"
        );
        Device::reset(&interp, ResetKind::Warm);
        Device::reset(&jit, ResetKind::Warm);
        // A reset is a topology-free way to change every byte a block was
        // lifted from, so the translations go; the run after it must still be
        // the interpreter's, tick for tick.
        for _ in 0..8 {
            interp.run_budget(1000);
            jit.run_budget(1000);
        }
        assert_eq!(interp.x(5), jit.x(5));
        assert_eq!(interp.pc(), jit.pc());
        assert_eq!(interp.cycles(), jit.cycles());
        assert_eq!(interp.instret(), jit.instret());
    }

    #[test]
    fn the_worst_case_bound_is_never_below_what_a_block_can_spend() {
        // The guard is only sound if the bound is an over-estimate, so this
        // asserts the direction rather than the value: the fallback bound must
        // cover the exact one for a full-length block of accesses.
        let cfg = Config::rv64gc();
        for translating in [false, true] {
            let exact = lift::MAX_INSNS as u64 * (2 + per_access(&cfg, translating))
                + if translating { WALK_ACCESSES } else { 0 };
            assert_eq!(worst_bound(&cfg, translating), exact);
            assert!(worst_bound(&cfg, translating) >= worst_bound(&cfg, false));
        }
    }

    #[test]
    fn a_cost_table_answers_only_for_the_block_it_recorded() {
        let mut costs = Costs::new();
        costs.put(0x1000, 7, 42);
        assert_eq!(costs.get(0x1000, 7), Some(42));
        assert_eq!(costs.get(0x1000, 8), None, "a different world");
        assert_eq!(costs.get(0x1002, 7), None, "a different pc");
        costs.clear();
        assert_eq!(costs.get(0x1000, 7), None);
    }
}
