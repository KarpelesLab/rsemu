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
//!   that reproducing them is a job rather than a line. The one thing that
//!   does *not* take that path is an aligned load of plain RAM whose
//!   translation this hart's TLB already holds, which compiled code serves
//!   inline — and the whole of [`FastMem`] below is the argument that those
//!   two are the same thing.
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
//! * **A block leaves at a guest instruction boundary when the caller's ticks
//!   run out.** Otherwise the guest's *stopping point* inside a scheduler
//!   quantum would depend on the engine: an interpreter overruns its budget by
//!   one instruction and a trace by up to sixty-four, the overrun is carried
//!   as `State::debt`, and both numbers are in the snapshot a machine's state
//!   hash is taken over. [`Host::spent`] is the whole of it — `Exec::used`
//!   against what [`advance`] was handed, asked at every boundary but a
//!   block's first and an exit, and answered by publishing the boundary's live
//!   map exactly as a fault does.
//!
//!   It replaced a *guard*: a block used to run only where its worst case,
//!   read off its ops, fitted what was left, so the tail of every quantum was
//!   interpreted. That was sound and it was expensive — on the x86 core, which
//!   has the fatter blocks and the bigger measurement, it was 2.02% of the
//!   whole guest instruction stream, three times everything outside the lifted
//!   subset put together. Nothing here computes a block's cost any more, and
//!   [`Unlifted`] is what is left of the table that used to hold one.
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
//! DRAM, `--headless` — one binary, three engines, over two spans of guest
//! time: sixty seconds, which is the boot, and four minutes, which is well
//! past the shell prompt:
//!
//! | `engine` | 60 s of guest time | 240 s of guest time |
//! | --- | --- | --- |
//! | `interp` | 36.6 s | 156.9 s |
//! | `jit` | 25.2 s (1.45×) | 120.4 s (1.30×) |
//! | `jit-host` | **14.9 s (2.45×)** | **68.7 s (2.28×)** |
//!
//! Median of six **interleaved** reps at sixty seconds and three at four
//! minutes — one of each engine, in turn, round and round — because the
//! interpreter is the control and a control measured in a different sitting is
//! not one. Not merely good practice: this table's predecessor was taken on a
//! different host CPU, where the same binary varied by 12% between sweeps
//! twenty minutes apart, which was larger than most of the effects below.
//! Numbers from two sittings cannot be reconciled, only re-measured.
//!
//! All three end on the same guest state, byte for byte:
//! `state hash 0x267fac762c374054` at four minutes.
//! `tests/riscv_virt_engines.rs` asserts that equality on every commit, on a
//! cheaper guest.
//!
//! **The host code generator used to lose to the portable one**, at 0.50× the
//! interpreter. Three things fixed it, and none of them was the code it emits.
//!
//! **Blocks are chained.** [`Frontend::enter`] is the hook that allows it,
//! [`CHAIN`] is how far, and on this guest **86% of blocks are reached by
//! following a patched exit** — 43.2 million of 50.0 million over sixty
//! seconds of guest time, against a `chained` count that was previously zero
//! in every run. What that buys is not the hash lookup it skips; it is
//! everything around a four-instruction block that a one-block call had to
//! pay in full.
//!
//! **A compile stopped costing 144 µs.** The 256 MiB code buffer this engine
//! asks for was being `mprotect`ed end to end twice per compiled block —
//! 433 600 cycles a block, against 13 850 for the code generation it was
//! protecting. `jit::x86::buf` flips a page-sized window instead, and the same
//! sixty seconds of guest time went from **25.6 billion cycles in `mprotect`
//! to 1.3 billion**.
//!
//! **And a guest load stopped costing a call.** The last of `ROADMAP.md` §9's
//! four mechanisms — the software TLB, *inlined* — is reached from a hart now
//! that this one publishes a [`MemPlan`](crate::jit::MemPlan); [`FastMem`]
//! below is what it may cover and why. Over three seconds of this guest's boot
//! the backend serves **3 250 549 of 3 340 050 compiled loads inline, 97.3%**,
//! and the host instructions the whole run executes fall from 18.17 billion to
//! 16.09 billion. Set beside the run's fixed tail — the CLI hashes 512 MiB of
//! guest RAM when it ends, which is 7.76 billion of either figure and has
//! nothing to do with the engine — the emulation itself went from 10.42
//! billion instructions to 8.34, **20% fewer**. It is worth 13% of the wall
//! clock rather than 20% (78.9 s to 68.7 s over four minutes), which is the
//! usual gap between an instruction count and a clock: what was removed —
//! `pmp_allows`, the flat-view walk, `AddressSpace::read` — is branch-heavy
//! ALU work, and what replaced it is a dependent load out of an 8 KiB table.
//!
//! What is left is honest and worth knowing. The compiled path's *execution*
//! was never the problem — measured over the same run it costs 768 cycles a
//! block against the IR interpreter's 896, so it was ahead before compilation
//! was counted. A block here lifts to about forty-seven IR instructions and
//! compiles to 949 bytes of host code; 59 269 of them are compiled in sixty
//! seconds and none is refused. And what that profile said costs most was not
//! the load path at all: the PMP check every translation still made, and the
//! stores the backend deliberately did not inline.
//!
//! **Both are done now.** Measured the same way — `Hart::advance` taken
//! *inclusively* under callgrind, which is emulation and nothing else: no
//! machine build, no loaders, and not the 512 MiB state hash the CLI takes
//! when a run ends — over the same three seconds of the same guest:
//!
//! | | host instructions in `Hart::advance` |
//! | --- | --- |
//! | as the inlined load left it | 7.37 G |
//! | with the PMP scan memoized | 5.99 G (−18.8%) |
//! | with stores inlined as well | **5.25 G (−28.7%)** |
//!
//! **`pmp_allows` was a fifth of it**, and that was a surprise rather than a
//! plan: a linear scan of sixteen entries, each decoding an address register,
//! **265 host instructions a call over 5 515 181 calls** — one per guest
//! access *and* one per block `admit` lets through. `mmu`'s `PmpSpan`
//! memoizes that scan over the largest interval its answer is provably
//! constant on, one slot per privilege mode, thrown away when
//! `Csrs::pmp_gen` moves. The interpreter gains as much as the JIT does,
//! because the scan was on the path both take.
//!
//! **And a guest store stopped costing a call too.** **1 749 886 of 1 753 140
//! compiled stores — 99.8%** — are now the same inlined probe a load makes
//! plus one thunk that pays what moving the bytes did not: the tick, the
//! `RamStore`'s own dirty bitmap, a broken reservation, and the
//! guest-physical page the block cache invalidates translations on.
//! `jit::fast`'s `FastMem::store_plan` states what a store plan promises and
//! why it is a stronger promise than a load's; `Exec::refresh_shadow` is
//! where the store set is filled, and the PTE dirty bit an inlined store never
//! sets was set by the walk that filled it.
//!
//! What the profile says now, over that same three seconds: `jit::x86::rt`'s
//! flush thunk (13.3 M calls, 1.16 G — but most of that is the charges and
//! the boundaries it *replays*, which is work the interpreter does too, so the
//! call overhead in it is nearer 0.13 G), `admit`'s per-block entry
//! translation (0.68 G), the dispatcher itself (0.47 G), and the guest-slot
//! reads a lifted block still makes (8.4 M calls, 0.26 G). The largest call
//! *count* is now `get_slot`, and the flush emitted ahead of each one is the
//! other half of it.
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
//! read.
//!
//! **That last sentence used to end "which nothing on a `virt` board does",
//! and it was wrong.** The CLINT does exactly that, and by design: it is a
//! lazily-advanced device (`ROADMAP.md` §4.2), so `MemOps::read` catches it up
//! to this hart's *live* position before it answers, and a comparator crossed
//! inside that catch-up drives `mtip` there and then. Ordinarily the
//! comparator is also where the round ends —
//! `Scheduler::natural_target` takes the soonest event a lazily-advanced
//! device has of its own — so the rise lands on a quantum boundary and both
//! engines see it in the same place. The window is a comparator the guest
//! moves **into the round that is already running**: `natural_target` was
//! computed when the round began and does not move, so the next read of
//! `mtime` crosses it. `mtip` then rises between two instructions of a lifted
//! block, where `Exec::step` would have taken the trap at the next one — and
//! the block ran on to its natural end, up to [`lift::MAX_INSNS`] instructions
//! later, with `mepc` naming the wrong instruction and the two engines on
//! different cycle counts for the rest of the run.
//!
//! [`IrHost::load`] closes it, and [`Host::hand_back`] is how the answer
//! reaches the boundary without [`IrHost::spent`] growing a second load. It is
//! the same *class* as the three A64 defects `docs/testing/long-run.md`
//! records — where a quantum ends, not what an instruction computes — and a
//! different mechanism: A64's generic timer is counted off the core's own
//! cycle counter, so the walk in [`admit`] could cross it; nothing on this
//! hart is driven off its tick count, and the raiser is a device on the other
//! side of a load.
//!
//! The interrupt check is asked **per block**, and that is unchanged by
//! chaining: [`admit`] runs it at every boundary, so a chain is as prompt as
//! the one-block calls it replaced. The **safe-point flag** is not, and that
//! is the price of chaining, stated rather than implied: `Hart::run_budget`
//! tests it between calls to [`advance`], so it used to be honoured within one
//! block and is now honoured within [`CHAIN`] of them. See that constant.
//!
//! # What is still not covered, and what would make it reachable
//!
//! **A sibling hart's IPI** — a store into another hart's `MSIP` — was the
//! hypothesis this was looked into for, and under
//! `ThreadingMode::Deterministic` it cannot be reached: the sibling only runs
//! in its own quantum, so the write lands while this hart is between quanta
//! and both engines see it at the next [`admit`]. Under
//! `ThreadingMode::Parallel` two harts really are inside the same round at
//! once and the wire really can move mid-block — but that mode gives up
//! reproducibility by construction and `Machine::state_hash` refuses in it, so
//! there is no oracle to diverge from. [`IrHost::load`] happens to cover the
//! `MSIP` case anyway, because a hart that *reads* anything in the CLINT
//! window asks the same question; a hart that reads nothing does not.
//!
//! **A page table in a device.** [`admit`] asks `Exec::pending_interrupt`
//! before it charges the entry translation, so a walk between the two is
//! unwatched — the window A64's `engine::leave_at` exists for. On this hart it
//! is empty for a different reason: the walk reads page-table entries, and a
//! `virt` board puts page tables in DRAM. A board that mapped them over a
//! lazily-advanced device would reopen it.
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
use crate::core::space::{AddressSpace, MemAttrs, MemResult, MonitorSlot};
use crate::core::value::Width;
use crate::ir::{InsnStart, IrHost, MemOp, RegSlot, verify};
use crate::jit::{
    BlockCache, DirtyPages, Dispatcher, Entry, Epoch, FastMem, Frontend, MemPlan, PAGE_MASK, Stop,
    StoreLog, Translation,
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
/// is asked [`Host::spent`] before it runs and every boundary inside one is
/// asked it too.
/// `a_chain_is_bounded_by_the_stated_safe_point_number` asserts the block half
/// of that, and
/// `the_same_agreement_holds_over_budgets_a_block_leaves_part_way_through` the
/// tick half.
///
/// Sixteen rather than sixty-four because the curve is flat past it — the
/// per-block cost being amortized is a fixed overhead, so the second block of
/// a chain removes half of it and the sixteenth removes a fifteenth — and a
/// safe point is worth more than the last percent.
const CHAIN: usize = 16;

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

/// How many *"there is no block at this PC"* answers are remembered.
///
/// Direct-mapped and keyed by the guest PC, exactly as the block cache is, and
/// sized with it so a resident block usually has a resident answer. A miss
/// costs one wasted lift, never a wrong decision.
const UNLIFTED_SLOTS: usize = 65536;

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

/// This hart's translation state: the dispatcher, and the PCs there is no
/// block at beside it.
///
/// **Derived state in the strict sense** (`ROADMAP.md` §4.5): never
/// serialized, and thrown away by a reset, by a snapshot restore and by a
/// debugger overwriting the CSR file. That is also what makes a snapshot
/// interchangeable between any two engines — there is nothing engine-specific
/// in one to interchange.
#[derive(Debug)]
pub(super) struct Jit {
    disp: Dispatcher,
    unlifted: Unlifted,
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
            unlifted: Unlifted::new(),
        }
    }

    /// Throw every translation away.
    pub(super) fn flush(&mut self) {
        self.disp.cache_mut().flush();
        self.unlifted.clear();
    }

    /// Blocks executed, and how many of those ran as host code.
    pub(super) fn stats(&self) -> (u64, u64) {
        let s = self.disp.stats();
        (s.blocks, s.compiled)
    }

    /// Whether this engine can use a [`mmu::Tlb`] shadow.
    ///
    /// Only the host code generator inlines a load ([`FastMem`]); the portable
    /// backend calls [`IrHost::load`] for every access, so a shadow attached
    /// for it would be filled and never read. The shadow is not free — a fill
    /// probes the flat view — so it is asked for by the one engine that reads
    /// it, and a `jit-host` that fell back to the portable backend because the
    /// host is not x86-64 Linux does not ask.
    pub(super) fn wants_shadow(&self) -> bool {
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        {
            self.disp.backend().is_some()
        }
        #[cfg(not(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64")))]
        {
            false
        }
    }
}

/// A direct-mapped table of the `(pc, key)` pairs there is **no block** at.
///
/// One job, and it is about not paying for the same answer twice: the
/// instruction at that PC is outside the lifted subset, so there is nothing to
/// translate and the interpreter should be reached directly. Without it every
/// `amoadd`, every `csrrw` and every `ecall` costs a dispatcher round trip and
/// a fresh [`lift::lift`] that fails at its first instruction — measured at
/// **42 million wasted lifts** in four minutes of guest time on a Linux boot,
/// against 1.1 million real ones, because RISC-V atomics are not in the subset
/// and a kernel is full of them.
///
/// It used to hold a *number* as well — the most ticks the block at that PC
/// could charge, computed once where the block was lifted because walking a
/// block's ops costs more than running the compiled block does. [`Host::spent`]
/// retired the guard that number was for, so the only answer left is the
/// negative one, and recording only negatives means a positive answer can no
/// longer evict a negative one from the slot they share.
///
/// It is discarded whenever a guest write invalidates a translation, because
/// the bytes it was an answer about may be different now.
#[derive(Debug)]
struct Unlifted {
    slots: Box<[Slot]>,
}

#[derive(Debug, Clone, Copy, Default)]
struct Slot {
    pc: u64,
    key: u64,
    live: bool,
}

impl Unlifted {
    fn new() -> Unlifted {
        Unlifted {
            slots: vec![Slot::default(); UNLIFTED_SLOTS].into_boxed_slice(),
        }
    }

    /// The low bit of a guest PC is always zero, so it carries nothing.
    #[inline]
    fn index(pc: u64) -> usize {
        ((pc >> 1) as usize) & (UNLIFTED_SLOTS - 1)
    }

    /// Whether the last lift at this `(pc, key)` produced nothing.
    #[inline]
    fn holds(&self, pc: u64, key: u64) -> bool {
        let slot = &self.slots[Unlifted::index(pc)];
        slot.live && slot.pc == pc && slot.key == key
    }

    #[inline]
    fn note(&mut self, pc: u64, key: u64) {
        self.slots[Unlifted::index(pc)] = Slot {
            pc,
            key,
            live: true,
        };
    }

    fn clear(&mut self) {
        self.slots.fill(Slot::default());
    }
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
}

/// Whether a block may run at `pc`.
#[derive(Debug)]
enum Admit {
    /// It may.
    Ready(Admitted),
    /// It may not, and the reason is one the interpreter answers: a pending
    /// interrupt, a stalled `WFI`, or an instruction outside the lifted
    /// subset.
    Interpret,
    /// The entry fetch itself faulted.
    Trap(Trap),
}

/// Everything a block owes before it runs — for the first block of a run and
/// for every chained successor alike, which is the whole point of it being one
/// function.
///
/// Two things happen here and the order is load-bearing.
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
/// and translates again for the high half, which then hits. It is also what
/// *names* the block, which is why the lookup below comes after it.
///
/// There used to be a third — the budget guard — and [`Host::spent`] is what
/// removed it: a block no longer has to prove that its worst case fits what is
/// left of the caller's ticks, because a block that overruns leaves at a guest
/// instruction boundary instead.
fn admit(cfg: &Config, unlifted: &Unlifted, exec: &mut Exec<'_>, pc: u64) -> Admit {
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

    // Known unliftable: the interpreter takes this instruction, and reaching
    // it without a lift that fails at its first instruction is the whole point
    // of remembering it.
    if unlifted.holds(pc, key) {
        return Admit::Interpret;
    }

    Admit::Ready(Admitted {
        origin,
        key,
        page: pc & !PAGE_MASK,
        base: phys & !PAGE_MASK,
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
/// `remaining` is what is left of the caller's budget, and it is the hart's
/// tick allowance rather than advice: [`Host::spent`] compares it against
/// `Exec::used` at every guest instruction boundary but a block's first and an
/// exit, and a block that has spent it leaves there — at the same instruction
/// an interpreted hart would have stopped at, with the same `State::debt`.
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
    monitor: Option<&MonitorSlot>,
    remaining: u64,
) -> (u64, Option<Exit>) {
    let Jit { disp, unlifted } = jit;
    let mut exec = Exec::new(state, tlb, space, cfg, lines, exits, monitor);
    let pc = exec.st.pc;

    // The entry work for the *first* block, done here rather than through
    // [`Frontend::enter`], because the overwhelmingly common answer on a real
    // guest is "not a block at all" — an `amoadd`, a `csrrw`, an `ecall`,
    // forty-two million of them in four minutes — and reaching the interpreter
    // for one should not cost a frontend, a host and a dispatcher round trip.
    // The dispatcher's first `enter` is then a no-op; see `Lifter::admitted`.
    let at = match admit(cfg, unlifted, &mut exec, pc) {
        Admit::Ready(at) => at,
        Admit::Interpret => return interpret(disp, unlifted, exec),
        Admit::Trap(trap) => return deliver(disp, unlifted, exec, trap, pc, pc),
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
        //
        // What a debug read does relax is the **mapping's** `Perms::EXEC`, and
        // deliberately — `MemAttrs::read_perm` will not ask for it when
        // `MemAttrs::debug` is set. `Lifter::translate` therefore asks for it
        // separately rather than through these attributes.
        attrs: MemAttrs::DEBUG.with_requester(cfg.requester),
        unlifted,
        admitted: true,
        entry_trap: None,
        rejected: None,
    };

    let mut host = Host::new(&mut exec, pc, remaining);
    let run = match disp.run(&mut front, &mut host, pc, CHAIN) {
        Ok(run) => run,
        // The only refusal this frontend has is an RV32 configuration, which
        // the property reader rejects before a hart is built. Degrade rather
        // than fail the machine (`ROADMAP.md` §9).
        Err(_) => {
            drop(host);
            let Lifter { unlifted, .. } = front;
            return interpret(disp, unlifted, exec);
        }
    };
    let Host {
        slots, trap, mark, ..
    } = host;
    let Lifter {
        unlifted,
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
        return interpret(disp, unlifted, exec);
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
            deliver(disp, unlifted, exec, trap, fault.pc, next)
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
            deliver(disp, unlifted, exec, trap, run.pc, run.pc)
        }
        // `Budget` ends a full chain, `Declined` a short one, `Spent` a block
        // that left part-way through because the hart's ticks ran out, and all
        // three leave the guest at `run.pc` for the run loop to pick up.
        // `Spent` needs no arm of its own for exactly that reason: the state
        // is published from the boundary's live map and `run.pc` is the guest
        // instruction that has not started, which is what this arm already
        // does with every other stop. `Exit` cannot happen: no safe-point flag
        // is given to the dispatcher, because the run loop above checks it
        // between calls.
        _ => {
            exec.st.pc = cfg.xlen.trunc(run.pc);
            let used = exec.used;
            drain(disp, unlifted, &mut exec);
            (used.max(1), None)
        }
    }
}

/// Interpret one instruction, and tell the block cache what it wrote.
fn interpret(
    disp: &mut Dispatcher,
    unlifted: &mut Unlifted,
    mut exec: Exec<'_>,
) -> (u64, Option<Exit>) {
    let used = exec.step();
    let exit = exec.take_exit();
    drain(disp, unlifted, &mut exec);
    (used, exit)
}

/// Take a trap the block or the entry fetch raised, exactly as `Exec::step`
/// takes one: out of the hart when the mask says so, into the guest's handler
/// otherwise.
fn deliver(
    disp: &mut Dispatcher,
    unlifted: &mut Unlifted,
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
    drain(disp, unlifted, &mut exec);
    (used.max(1), out)
}

/// Hand what an interpreted instruction wrote to the block cache.
///
/// The block path reports its stores through [`StoreLog`] and the dispatcher
/// drains those itself; this is the other half, for every instruction outside
/// the lifted subset — an `AMO`, an `SC`, a byte written by a trap handler
/// running through the interpreter.
fn drain(disp: &mut Dispatcher, unlifted: &mut Unlifted, exec: &mut Exec<'_>) {
    let mut hit = 0usize;
    for i in 0..exec.wrote_n as usize {
        hit += disp.cache_mut().note_write(exec.wrote[i], 1);
    }
    exec.wrote_n = 0;
    if hit > 0 {
        // A page a translation came from has changed, so every answer in
        // [`Unlifted`] may have changed with it — an instruction that was
        // outside the subset can have been overwritten by one that is not.
        // Clearing the lot is a blunt answer to a rare event: eight thousand
        // of these in four minutes of guest time, against three hundred
        // million blocks.
        unlifted.clear();
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
    unlifted: &'a mut Unlifted,
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
        let entry = match admit(self.cfg, self.unlifted, host.exec, pc) {
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

    /// Lift at `pc`, reading only what the entry translation covers **and what
    /// the bus would let this hart fetch**.
    ///
    /// The second half is the one that is not obvious. [`admit`] has already
    /// run the *MMU* fetch path, so `satp`'s page tables and PMP have had their
    /// say; what it has not consulted is the mapping's
    /// [`Perms::EXEC`](crate::core::space::Perms::EXEC), because that is
    /// enforced against a read carrying `AccessPurpose::FETCH` and the reads
    /// below deliberately carry [`MemAttrs::DEBUG`] instead — a lift must not
    /// put a NOR bank into its command state. So the permission is asked for
    /// separately, by [`jit::executable_run`](crate::jit::executable_run), and
    /// the bytes still come out with no side effects.
    ///
    /// Per *halfword* rather than per page, because a flat entry is not a page:
    /// a 4 KiB page can hold an executable mapping and a non-executable one,
    /// and the interpreter would fetch its way to the boundary and take an
    /// instruction access fault there. A lift that stops at the same halfword
    /// leaves the interpreter standing on the same instruction, which is what
    /// makes the two engines report the same cause at the same `mepc`. The
    /// probe is memoised over the run the entry covers, so the ordinary
    /// whole-page-one-mapping case pays for one.
    ///
    /// **Nothing here is on the block-entry path.** This runs once per
    /// translation; a cached block re-enters through [`admit`] and touches none
    /// of it. That is sound because a permission change is a retopology, and
    /// [`Frontend::epoch`] already drops every block lifted before it.
    fn translate(&mut self, pc: u64) -> Result<Translation> {
        let space = self.space;
        let attrs = self.attrs;
        let base = self.at.base;
        let page = self.at.page;
        // Empty: `run.0 > run.1`, so nothing is inside it and the first
        // halfword probes.
        let mut run = (1u64, 0u64);
        let mut refused = false;
        let mut src = |addr: u64| {
            // Outside the entry page there is no translation to read through,
            // so the lifter is told the bytes are unreadable and ends the
            // block. It would have ended it at the page bound anyway; this is
            // the belt.
            if addr & !PAGE_MASK != page {
                return None;
            }
            let phys = base | (addr & PAGE_MASK);
            let end = phys.wrapping_add(2);
            if phys < run.0 || end > run.1 {
                match crate::jit::executable_run(space, phys) {
                    // The whole halfword has to be inside one permitting run.
                    // One straddling two of them is refused rather than
                    // stitched: the interpreter would ask the bus twice and
                    // this would have to model both answers, and handing the
                    // instruction to the interpreter is always right.
                    Some(r) if phys >= r.0 && end <= r.1 => run = r,
                    _ => {
                        refused = true;
                        return None;
                    }
                }
            }
            space.read(phys, Width::U16, attrs).ok().map(|v| v as u16)
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
        // A lift that produced nothing is remembered, which is what sends the
        // next pass straight to the interpreter instead of back through here.
        //
        // Except a lift the **bus** refused outright. That is the same outcome
        // and a different fact: [`Unlifted`]'s only cleaner is a store into a
        // lifted page, so an entry written here would outlive the retopology
        // that granted `Perms::EXEC` and pin the PC to the interpreter for the
        // rest of the run. Never a wrong answer — the interpreter is the
        // oracle — just a permanent one.
        if lifted.insns == 0 && !refused {
            self.unlifted.note(pc, self.at.key);
        }
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
    /// What [`advance`] was given: the ticks this run may charge before a
    /// block has to stop at its next guest instruction boundary.
    ///
    /// The whole of what this hart owes [`IrHost::spent`]. It is a function of
    /// ticks already charged and it is monotone, because `Exec::used` only
    /// ever grows within one [`advance`].
    allowance: u64,
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
    fn new(exec: &'a mut Exec<'e>, pc: u64, allowance: u64) -> Host<'a, 'e> {
        let mut slots = [0u64; lift::SLOT_COUNT as usize];
        slots[..32].copy_from_slice(&exec.st.x);
        slots[PC.0 as usize] = pc;
        Host {
            exec,
            allowance,
            slots,
            trap: None,
            mark: None,
            dirty: DirtyPages::new(),
        }
    }

    /// Retire what is left of this run's `allowance`, so the block leaves at
    /// its next guest instruction boundary.
    ///
    /// The whole of how an interrupt reaches [`IrHost::spent`], and it costs
    /// that function *nothing*, in the strong sense that its body is
    /// unchanged: a boundary already loads `allowance` and compares
    /// `Exec::used` against it, and zero is the value that comparison is
    /// always true for. A second field would have been a second load on the
    /// hottest path in the file — asked at every guest instruction boundary of
    /// every block — to carry one bit that this one already has room for.
    ///
    /// What is left to pay for is the question in [`IrHost::load`], and it was
    /// measured under callgrind rather than on a clock, because the host this
    /// was written on was carrying a load average of thirty and the wall-clock
    /// spread between two runs of the *unmodified* binary was larger than the
    /// effect. `Hart::advance` taken inclusively over the whole of
    /// `benches/jit_dispatch --smoke` — three engines, four workloads, six
    /// quanta — went from **1 038 017 036 host instructions to 1 039 145 936,
    /// +0.11%**. The variant that carried a separate `bool` measured
    /// 1 037 957 044, *below* the baseline: the two bracket it, so the honest
    /// reading is that both are under this instrument's resolution on this mix
    /// and the fold is worth having for the shape rather than for the number.
    ///
    /// The ticks are not lost. `advance` reports `Exec::used`, and
    /// `Hart::run_budget` loops until *its* allowance is spent, so what this
    /// gives up is the rest of the **run**, not the rest of the quantum —
    /// which is exactly right, because the rest of the quantum now belongs to
    /// the interpreter and to the trap it is about to take.
    #[inline]
    fn hand_back(&mut self) {
        self.allowance = 0;
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

    /// The interpreter's own load, and then the interpreter's own interrupt
    /// question — because this is the one call a block makes that can change
    /// the answer to it.
    ///
    /// Everything else inside a block leaves the interrupt inputs alone.
    /// `Exec::charge` counts ticks and nothing on this hart is driven off that
    /// count; a **store** ends the block by construction
    /// ([`lift`](super::lift), "A store still ends the block"), so whatever it
    /// raises is seen by the next boundary's [`admit`]; and every CSR write,
    /// `MRET`, `WFI` and `SFENCE.VMA` is outside the lifted subset, so the
    /// enables and the delegation this hart decides with are constant for the
    /// length of a run. A **load** is what is left, and the module docs used
    /// to dismiss it: *"a load from a device that raises an interrupt as a
    /// side effect of being read, which nothing on a `virt` board does"*. The
    /// CLINT does. It is a lazily-advanced device
    /// (`ROADMAP.md` §4.2), so `MemOps::read` catches it up to this hart's
    /// live position before answering — and a comparator the guest moved into
    /// the current round, after `Scheduler::natural_target` had already chosen
    /// where the round ends, is crossed by that catch-up rather than by the
    /// round. `mtip` then rises **between two instructions of a lifted
    /// block**, where `Exec::step` would have taken the trap at the next one
    /// and a block would run on to its natural end.
    ///
    /// So the answer is asked here rather than at every boundary: one call on
    /// a path that already crosses the whole memory path, against a load and a
    /// compare per boundary in [`IrHost::spent`]. `Exec::pending_interrupt` is
    /// the interpreter's predicate rather than a second copy of it, and its
    /// own first act is one atomic read masked with `mie`, so a load that
    /// raises nothing pays for that and stops.
    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        let done = self.exec.load(addr, mem.size.bytes());
        if self.exec.pending_interrupt().is_some() {
            self.hand_back();
        }
        match done {
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

    /// One load and one compare, at every guest instruction boundary but a
    /// block's first and an exit.
    ///
    /// `>=` rather than `>`: `Hart::run_budget` loops while `used < allowance`,
    /// so an interpreted hart stops at the first instruction that takes the
    /// count to the budget or past it, and a block that stopped one
    /// instruction later would leave the two engines on different instructions
    /// with different `State::debt` for the rest of the run.
    ///
    /// It answers a **second** question with the same compare: an interrupt
    /// that arrived inside this run. [`Host::hand_back`] retires the allowance
    /// when [`IrHost::load`] finds one, so the block leaves at the boundary
    /// after the load, [`advance`] returns, and the next call's [`admit`]
    /// finds the interrupt pending and hands the instruction to the
    /// interpreter — which is exactly the sequence an interpreted hart runs,
    /// one `step` charging the load and the next taking the trap.
    #[inline]
    fn spent(&self) -> bool {
        self.exec.used >= self.allowance
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

/// The inlined fast path, and exactly what a plan may cover.
///
/// A backend that inlines a load skips [`IrHost::load`] entirely — and with it
/// this hart's page-table walk, its PMP check, and the ticks both spend. That
/// used to be the argument for publishing nothing: a walk cannot be skipped
/// per access, so a plan whose validity is decided per access is not a plan.
///
/// **The plan is not per access. It is per page, and it is decided by the
/// table the walk already filled.** `mmu::Tlb` gained a shadow
/// ([`mmu::Tlb::attach_shadow`]) which `Exec::translate` writes in the same
/// breath as its own entry, at the same index, for the same virtual page. So a
/// shadow entry exists only where this hart's TLB *also* holds the
/// translation, and an inlined load that hits one is a load whose walk had
/// already been performed and charged for — which is why the whole cost it
/// still owes is the one tick [`FastMem::note_fast_load`] charges.
///
/// The three things the compiled path cannot do are therefore done once, at
/// fill time, rather than never:
///
/// * the **walk**, by the hart, on the miss that filled the entry — and the
///   entry dies with the hart's own, because both are written by the same
///   eviction and both carry `Csrs::translation_gen`;
/// * **PMP**, whose answer must be uniform over the whole page before the page
///   may be cached at all (`mmu::pmp_page_uniform`) — a page it refuses is
///   remembered as slow, and a page it answers unevenly over is never cached;
/// * the **fault**, which cannot arise: an entry exists only over plain
///   little-endian RAM covering its whole page, with the permissions the slow
///   path checks and no constraint left to apply (`jit::Tlb::fill`).
///
/// Everything a plan does not cover still calls [`IrHost::load`] and gets this
/// hart's answer — a store, a fetch, a misaligned access, an access to a
/// device, a page whose translation has been evicted, and every access at all
/// on a build without a shadow.
impl FastMem for Host<'_, '_> {
    fn load_plan(&mut self) -> Option<MemPlan> {
        self.exec.mem_plan(Access::Load)
    }

    fn note_fast_load(&mut self) {
        // `Exec::read_once`, with the translation known cached and the access
        // itself already done: one bus access is one cycle, and the walk was
        // charged when the entry was filled.
        self.exec.charge();
    }

    fn store_plan(&mut self) -> Option<MemPlan> {
        self.exec.mem_plan(Access::Store)
    }

    fn note_fast_store(&mut self, addr: u64, bytes: u64) {
        // `Exec::write_once` minus the bytes, and then the same move of
        // `Exec::wrote` into this host's dirty log that `IrHost::store` makes
        // — so a page written by a compiled store and one written by an
        // interpreted store reach `StoreLog` by the same route.
        self.exec.note_fast_store(addr, bytes);
        self.note_writes();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{Mapping, MonitorSlot, Perms, RamStore, Region};
    use crate::cpu::riscv::{Engine, Hart};

    /// How much RAM a test hart gets, at address zero.
    const RAM: u64 = 0x1_0000;

    /// A one-register block that asserts this hart's machine timer line when
    /// it is read, and does nothing else.
    #[derive(Debug)]
    struct RaiseOnRead {
        hart: crate::core::sync::Mutex<alloc::sync::Weak<Hart>>,
    }

    impl crate::core::space::MemOps for RaiseOnRead {
        fn read(&self, _offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
            dst.fill(0);
            if attrs.debug {
                return Ok(());
            }
            if let Some(hart) = self.hart.lock().upgrade() {
                hart.set_interrupt(crate::cpu::riscv::irq::MTI, true);
            }
            Ok(())
        }

        fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
            Ok(())
        }
    }

    /// `RAISER` at zero with [`RaiseOnRead`] mapped just past the RAM.
    fn raising_hart(engine: Engine) -> Arc<Hart> {
        let ram = Arc::new(RamStore::new(RAM));
        write_words(&ram, 0, &RAISER);
        let dev = Arc::new(RaiseOnRead {
            hart: crate::core::sync::Mutex::with_rank(
                crate::core::sync::LockRank::LEAF,
                alloc::sync::Weak::new(),
            ),
        });
        let space = AddressSpace::new("mem", 64);
        {
            let mut topo = space.topology();
            topo.map_with_perms(Region::ram("ram", ram), 0, Perms::RWX)
                .expect("nothing else is mapped");
            topo.map(
                Region::io(
                    "raiser",
                    0x1000,
                    Arc::clone(&dev) as Arc<dyn crate::core::space::MemOps>,
                ),
                RAM,
            )
            .expect("it does not overlap the RAM");
        }
        let hart = Arc::new(Hart::new(Config::rv64gc().with_reset_vector(0)).with_engine(engine));
        hart.attach_space(Arc::new(space));
        *dev.hart.lock() = Arc::downgrade(&hart);
        hart
    }

    /// A load from a device that raises an interrupt, four instructions from
    /// the end of a lifted trace.
    const RAISER: [u32; 9] = [
        0x0001_03b7, // lui    x7, 0x10        ; the device, just past the RAM
        0x3004_6073, // csrrsi x0, mstatus, 8  ; MIE
        0x0800_0313, // addi   x6, x0, 0x80    ; MTI
        0x3043_2073, // csrrs  x0, mie, x6
        0x0003_be03, // ld     x28, 0(x7)      ; raises MTI   <- 0x10
        0x0012_8293, // addi   x5, x5, 1
        0x0012_8293, // addi   x5, x5, 1
        0x0012_8293, // addi   x5, x5, 1
        0xff1f_f06f, // jal    x0, -16
    ];

    #[test]
    fn a_load_that_raises_an_interrupt_is_taken_where_the_interpreter_takes_it() {
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = raising_hart(Engine::Interp);
            let jit = raising_hart(engine);
            for n in 0..8 {
                let a = interp.run_budget(1000);
                let b = jit.run_budget(1000);
                assert_eq!(a, b, "quantum {n}: different budgets under {engine:?}");
            }
            assert_eq!(
                interp.csrs().mcause,
                (1u64 << 63) | 7,
                "the fixture never took a machine timer interrupt"
            );
            assert_eq!(
                interp.csrs().mepc,
                jit.csrs().mepc,
                "mepc under {engine:?}: interpreter {:#x}, translated {:#x}",
                interp.csrs().mepc,
                jit.csrs().mepc,
            );
            assert_eq!(interp.x(5), jit.x(5), "x5 under {engine:?}");
            assert_eq!(interp.cycles(), jit.cycles(), "cycles under {engine:?}");
        }
    }

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
        hart_mapped(engine, program, Perms::RWX)
    }

    /// [`hart`], with the RAM mapped on `perms` rather than on everything.
    fn hart_mapped(engine: Engine, program: &[u32], perms: Perms) -> Hart {
        let ram = Arc::new(RamStore::new(RAM));
        write_words(&ram, 0, program);
        let space = AddressSpace::new("mem", 64);
        space
            .topology()
            .map_with_perms(Region::ram("ram", ram), 0, perms)
            .expect("nothing else is mapped");
        let hart = Hart::new(Config::rv64gc().with_reset_vector(0)).with_engine(engine);
        hart.attach_space(Arc::new(space));
        hart
    }

    /// `words` into `store`, little-endian, starting at `at`.
    fn write_words(store: &RamStore, at: u64, words: &[u32]) {
        for (i, word) in words.iter().enumerate() {
            for (j, byte) in word.to_le_bytes().iter().enumerate() {
                store
                    .write_u8(at + (i * 4 + j) as u64, *byte)
                    .expect("in range");
            }
        }
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
        agree_built(engine, |e| hart(e, program), budget, quanta)
    }

    /// [`agree_on`], over a fixture that builds its own address space.
    ///
    /// The permission cases need two mappings with different [`Perms`] rather
    /// than one flat RAM, and every column [`agree_on`] compares is exactly the
    /// set they care about — an instruction access fault is `mcause`, `mepc`,
    /// `mtval` and a PC.
    fn agree_built(
        engine: Engine,
        build: impl Fn(Engine) -> Hart,
        budget: u64,
        quanta: usize,
    ) -> (Hart, Hart) {
        let interp = build(Engine::Interp);
        let jit = build(engine);
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

    /// Two instructions on an executable page, the second a jump into the
    /// *next* page — which is mapped without [`Perms::EXEC`].
    ///
    /// The jump leaves the page, so the block ends and the refusal lands at a
    /// **chained** boundary, through `Frontend::enter` and
    /// `Frontend::translate` rather than through [`advance`]'s prologue.
    const INTO_NX: [u32; 2] = [
        0x0010_0313, // addi x6, x0, 1
        0x7fd0_006f, // jal  x0, 0xffc     ; from 0x4 to 0x1000
    ];

    /// What sits on the page that may not be fetched.
    ///
    /// Both instructions are inside the lifted subset and neither traps, so a
    /// lifter that never asks about execute permission produces a real block
    /// out of it and runs it — which is exactly the divergence.
    const NX_PAGE: [u32; 2] = [
        0x0070_0293, // addi x5, x0, 7
        0x0000_006f, // jal  x0, 0
    ];

    /// A hart whose first page may be fetched and whose second may not.
    ///
    /// Two stores and two mappings rather than one store and two permissions,
    /// because permission is a property of the mapping — and this is the shape
    /// a board with a data-only aperture beside its ROM actually has.
    fn split_hart(engine: Engine) -> Hart {
        let code = Arc::new(RamStore::new(0x1000));
        write_words(&code, 0, &INTO_NX);
        let data = Arc::new(RamStore::new(0x1000));
        write_words(&data, 0, &NX_PAGE);
        let space = AddressSpace::new("mem", 64);
        {
            let mut topo = space.topology();
            topo.map_with_perms(Region::ram("code", code), 0, Perms::RX)
                .expect("nothing else is mapped");
            topo.map_with_perms(Region::ram("nx", data), 0x1000, Perms::RW)
                .expect("it does not overlap the code");
        }
        let hart = Hart::new(Config::rv64gc().with_reset_vector(0)).with_engine(engine);
        hart.attach_space(Arc::new(space));
        hart
    }

    /// [`Perms::EXEC`] is enforced against a *translated* block, not only
    /// against an interpreted fetch.
    ///
    /// `MemAttrs::purpose` makes a fetch a fetch and `FlatLeaf::read` refuses
    /// one on a mapping without [`Perms::EXEC`] — but only the interpreter's
    /// fetch *is* that read. A lift reads ahead of the guest and therefore
    /// reads with [`MemAttrs::DEBUG`], which by design never asks for `EXEC`,
    /// and [`admit`] consults the *MMU* and PMP rather than the mapping. Before
    /// `jit::executable_run` this fixture ran the whole loop under the JIT
    /// while the interpreter took an instruction access fault on its first
    /// instruction.
    ///
    /// The reset vector itself is on the refusing mapping here, so the refusal
    /// is in [`advance`]'s prologue: the very first lift.
    #[test]
    fn a_mapping_without_exec_refuses_a_translated_block_too() {
        for engine in [Engine::Jit, Engine::JitHost] {
            let (interp, jit) = agree_built(engine, |e| hart_mapped(e, &LOOP, Perms::RW), 1000, 4);
            assert_eq!(
                interp.csrs().mcause,
                cause::INSN_ACCESS,
                "the interpreter took some other trap, so this fixture is not \
                 about execute permission under {engine:?}"
            );
            assert_eq!(
                interp.x(5),
                0,
                "the loop body ran on a mapping that refuses a fetch"
            );
            assert_eq!(
                jit.jit_stats().expect("a jit hart").0,
                0,
                "a block ran out of a mapping whose fetch the interpreter \
                 refuses, under {engine:?}"
            );
        }
    }

    /// The same claim at a **chained** boundary, which is the only way to reach
    /// `Frontend::enter` and the one path [`advance`]'s prologue does not
    /// cover.
    #[test]
    fn a_chain_into_a_non_executable_page_faults_where_the_interpreter_does() {
        for engine in [Engine::Jit, Engine::JitHost] {
            let (interp, jit) = agree_built(engine, split_hart, 1000, 4);
            assert_eq!(
                interp.csrs().mcause,
                cause::INSN_ACCESS,
                "the interpreter took some other trap under {engine:?}"
            );
            assert_eq!(
                interp.x(5),
                0,
                "the non-executable page ran under the interpreter, so the \
                 fixture is not testing what it says it is"
            );
            let blocks = jit.jit_stats().expect("a jit hart").0;
            assert!(
                blocks > 0,
                "the executable page never produced a block, so nothing was \
                 chained *from* and the boundary is untested"
            );
        }
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
        // several blocks; and the carried debt, because the tick allowance is
        // asked at every boundary of every block of a chain.
        let (interp, jit) = agree(&FAR_LOOP, 1000, 64);
        assert!(interp.cycles() > 1000, "the run was too short to mean much");
        assert_eq!(interp.x(28), jit.x(28), "the value reloaded from memory");
    }

    #[test]
    fn the_same_agreement_holds_over_budgets_a_block_leaves_part_way_through() {
        // Twelve ticks is under what most blocks here charge end to end, so
        // nearly every block leaves at a guest instruction boundary rather
        // than at its terminator — which must give the same answer as
        // interpreting everything, and as blocks that run to the end.
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

        /// The same bench with the host code generator, and with the shadow
        /// the inlined fast path probes.
        ///
        /// Both, together, because they are one mechanism: the portable
        /// backend never reads a shadow and the code generator can only reach
        /// one that this hart's own TLB is keeping in lockstep.
        fn with_host_code(mut self) -> Bench {
            self.jit = Jit::new(true);
            if self.jit.wants_shadow() {
                self.tlb.attach_shadow(Arc::clone(&self.space));
            }
            self
        }

        /// What the shadow has been asked to do.
        ///
        /// The counters, not the timing: a fixture whose two pages evict each
        /// other never serves a load inline at all, so `fast_loads` cannot say
        /// whether it reached the arm it was built for. These can.
        fn shadow_stats(&mut self) -> crate::jit::TlbStats {
            self.tlb
                .shadow_mut()
                .map_or_else(Default::default, |s| s.stats())
        }

        /// How many guest loads the compiled path served without a call.
        ///
        /// Zero on a host with no code generator, which is why every test that
        /// asserts a *positive* count is gated on having one: a fixture that
        /// silently reaches nothing is worse than no fixture.
        fn fast_loads(&self) -> u64 {
            #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
            {
                self.jit
                    .disp
                    .backend()
                    .map_or(0, |engine| engine.stats().fast_loads)
            }
            #[cfg(not(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64")))]
            {
                0
            }
        }

        /// How many guest stores the compiled path served without the host's
        /// own store path.
        ///
        /// Zero on a host with no code generator, exactly as
        /// [`Bench::fast_loads`] is, and for the same reason.
        fn fast_stores(&self) -> u64 {
            #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
            {
                self.jit
                    .disp
                    .backend()
                    .map_or(0, |engine| engine.stats().fast_stores)
            }
            #[cfg(not(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64")))]
            {
                0
            }
        }

        /// Map one more virtual page, at `vpn`, onto physical `phys`.
        ///
        /// [`Bench::paged`]'s tables are three levels with the leaf table at
        /// 0x3000, and every `vpn` below 512 indexes it directly — so a second
        /// page is one write.
        fn map_page(&self, vpn: u64, phys: u64) {
            use super::super::mmu::pte;
            let leaf = pte::V | pte::R | pte::W | pte::X | pte::A | pte::D;
            self.poke(0x3000 + vpn * 8, ((phys >> 12) << 10) | leaf);
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
        fn admit(&mut self) -> Admit {
            let Bench {
                space,
                cfg,
                state,
                tlb,
                lines,
                jit,
            } = self;
            let mut exec = Exec::new(state, tlb, space, cfg, lines, ExitMask::NONE, None);
            let pc = exec.st.pc;
            admit(cfg, &jit.unlifted, &mut exec, pc)
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
            advance(
                jit,
                state,
                tlb,
                space,
                cfg,
                lines,
                ExitMask::NONE,
                None,
                budget,
            )
            .0
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
            Exec::new(state, tlb, space, cfg, lines, ExitMask::NONE, None).step()
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
            matches!(b.admit(), Admit::Ready(_)),
            "the fixture has to be liftable, or this proves nothing"
        );
        b.state.csrs.mie |= irq::MTI;
        b.state.csrs.mstatus |= status::MIE;
        b.lines.set_pending(irq::MTI, true);
        assert!(
            matches!(b.admit(), Admit::Interpret),
            "a hart with a pending machine timer interrupt must reach the \
             interpreter, whatever is liftable at its PC"
        );
    }

    #[test]
    fn a_stalled_wfi_keeps_a_block_from_running_too() {
        // The other half of the same guard. A `WFI` that has not been woken is
        // not an instruction a block can retire past.
        let mut b = Bench::new(&FAR_LOOP);
        assert!(matches!(b.admit(), Admit::Ready(_)));
        b.state.wfi = true;
        assert!(matches!(b.admit(), Admit::Interpret));
    }

    #[test]
    fn a_budget_no_whole_block_fits_in_still_retires_inside_one() {
        // What adopting [`IrHost::spent`] is *for*, asserted rather than left
        // to a boot log. Sixteen ticks is far under what a full-length block
        // of this fixture charges — a `lift::MAX_INSNS` trace of uncompressed
        // instructions with accesses spends hundreds — so under the guard this
        // replaced every one of these calls interpreted a single instruction
        // and no block ever ran. Now the block runs and leaves at the first
        // boundary past the allowance.
        let mut b = Bench::new(&FAR_LOOP);
        // Warm the cache at a budget everything fits, so the measurement is
        // about admission rather than about a first lift.
        b.advance(u64::MAX / 2);
        let before = b.jit.disp.stats().blocks;
        assert!(before > 0, "nothing ran, so nothing was warmed");
        for _ in 0..64 {
            b.advance(16);
        }
        assert!(
            b.jit.disp.stats().blocks > before + 32,
            "a budget smaller than a whole block ran {} blocks in 64 calls: a \
             block is being refused for ticks it could have left part-way \
             through ({:?})",
            b.jit.disp.stats().blocks - before,
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
    fn a_misaligned_access_costs_a_block_what_it_costs_the_interpreter() {
        // A misaligned access on this core is one bus cycle *per byte*, so
        // this fixture's hot block spends eight ticks of fetch and sixteen of
        // access rather than the two an aligned block would — and a budget
        // inside that window is one where a block that miscounted its own
        // charges, or asked [`Host::spent`] at the wrong boundary, stops
        // somewhere the interpreter does not. Every column below — cycles,
        // debt, `instret`, the register file — moves when it does.
        //
        // The budget has to be one where the *interpreter* would stop
        // somewhere else: at sixteen it runs the same four instructions and
        // overruns to twenty-four exactly as the block does, because the last
        // instruction is the expensive one. So the budget is swept rather than
        // picked; two picked by eye let a mutant of the cost model this
        // replaced walk out.
        //
        // Warm at a budget everything fits, then squeeze.
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
        let Admit::Ready(at) = b.admit() else {
            panic!("the mapped page must be liftable");
        };
        assert_eq!(at.base, 0x4000, "the bytes come from the mapped page");
        assert_eq!(at.origin, Origin::Paged { generation: 0x4 });
        assert_ne!(
            at.key,
            lift::key(&b.cfg, Origin::Bare, SHAPE),
            "a paged block must not key identically to a bare one at the same PC"
        );

        // The same virtual page, a different physical one. The bytes have
        // changed, so the key must: a stale block must be unreachable rather
        // than merely unlikely.
        use super::super::mmu::pte;
        b.poke(
            0x3000,
            ((0x5000 >> 12) << 10) | pte::V | pte::R | pte::X | pte::A,
        );
        b.tlb.flush();
        let Admit::Ready(moved) = b.admit() else {
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
    fn a_remap_under_a_warm_shadow_is_not_served_from_the_old_store() {
        // The one path on which a stale shadow entry is *wrong bytes* rather
        // than a wrong tick, and the reason the plan is resynchronised at
        // every block boundary rather than only where the table is filled.
        //
        // A retopology does not flush the hart's own TLB — nothing about a
        // guest translation changed — so the load after it **hits** that TLB
        // and never reaches `refresh_shadow`, which is where the other sync
        // lives. If the plan a block publishes were not synchronised too, the
        // inlined probe would match an entry whose host address still points
        // into the store that used to be there. The old store is still
        // allocated, so there is no fault to notice: the guest simply reads
        // the bytes it read before the remap.
        // The data lives in a region of its own, so the remap can replace what
        // the load reads without also replacing the code that reads it.
        let mut b = Bench::new(&REMOTE_LOAD).with_host_code();
        let first_store = Arc::new(RamStore::new(0x1000));
        for i in 0..8u64 {
            first_store.write_u8(i, 0x11).expect("in range");
        }
        let id = b
            .space
            .topology()
            .map(Region::ram("data", first_store), DATA)
            .expect("that address is free");
        for _ in 0..8 {
            b.advance(10_000);
        }
        assert_eq!(
            b.state.x[28], 0x1111_1111_1111_1111,
            "the fixture never read the region it is about to replace"
        );
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(b.fast_loads() > 0, "the shadow was never warm");
        // Put different bytes behind the very page the shadow has cached.
        let replacement = Arc::new(RamStore::new(0x1000));
        for i in 0..8u64 {
            replacement.write_u8(i, 0xa5).expect("in range");
        }
        b.space
            .topology()
            .replace(id, Mapping::new(Region::ram("swapped", replacement), DATA))
            .expect("the same span, a different store");
        for _ in 0..8 {
            b.advance(10_000);
        }
        assert_eq!(
            b.state.x[28], 0xa5a5_a5a5_a5a5_a5a5,
            "the compiled hart read the store the remap replaced"
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
        // on the same instruction: [`Host::spent`] is asked at every boundary
        // of every block of a chain against what the chain has *left*, so a
        // run overruns by **one guest instruction**, exactly as an interpreted
        // hart does. The bound asserted here is therefore the worst any single
        // instruction in this fixture can charge and not a block's worth of
        // them — a guard reinstated in any form would still pass the loose
        // version of this and a chain that ignored the allowance would not
        // pass this one.
        //
        // Twelve ticks: an uncompressed instruction charges two for its fetch
        // and this fixture's widest access is an aligned doubleword, which is
        // one more; twelve is comfortably over that and far under the 128 a
        // full `lift::MAX_INSNS` block of them would spend.
        for budget in [1, 4, 12, 64, 512] {
            let (stats, used) = drive(&FAR_LOOP, budget, 1);
            assert!(
                used <= budget.max(1) + 12,
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
    fn the_unliftable_table_answers_only_for_the_block_it_recorded() {
        let mut unlifted = Unlifted::new();
        unlifted.note(0x1000, 7);
        assert!(unlifted.holds(0x1000, 7));
        assert!(!unlifted.holds(0x1000, 8), "a different world");
        assert!(!unlifted.holds(0x1002, 7), "a different pc");
        unlifted.clear();
        assert!(!unlifted.holds(0x1000, 7));
    }

    // -----------------------------------------------------------------
    // The inlined fast path
    // -----------------------------------------------------------------

    /// `x7 = 0`, then a load from virtual zero in a tight loop.
    ///
    /// Virtual page zero is the page [`Bench::paged`] maps, so the load reads
    /// the code it is running — which is exactly what makes this a load of
    /// plain RAM through a translation the hart's TLB holds. No store, so the
    /// block never invalidates itself and the loop is a trace.
    const PAGED_LOAD: [u32; 3] = [
        0x0000_0393, // addi x7, x0, 0
        0x0003_be03, // ld   x28, 0(x7)   ; the loop starts here
        0xffdf_f06f, // jal  x0, -4
    ];

    /// Two loads a megabyte apart, so they collide in one TLB slot.
    ///
    /// 256 pages is [`mmu::TLB_ENTRIES`], and both tables index a page by the
    /// same low bits — so each load evicts the other's translation and every
    /// one of them walks. That is the fixture the whole lockstep argument
    /// turns on: a shadow that did *not* evict in step with the hart's own TLB
    /// would keep both entries, serve both loads inline, and charge one tick
    /// where the interpreter charges a walk as well.
    const PAGED_COLLIDE: [u32; 5] = [
        0x0000_0393, // addi x7, x0, 0
        0x0010_0437, // lui  x8, 0x100    ; x8 = 0x100000, vpn 256
        0x0003_be03, // ld   x28, 0(x7)   ; the loop starts here
        0x0004_3e83, // ld   x29, 0(x8)
        0xff9f_f06f, // jal  x0, -8
    ];

    /// Run `program` on a paged hart under both engines and assert they agree
    /// on every column a guest can see — the cycle count above all, because
    /// that is the one an inlined load is most able to get wrong.
    ///
    /// Returns the compiled bench, so a caller can go on to ask whether the
    /// path it meant to test was reached at all.
    fn paged_engines_agree(fixture: fn() -> Bench, calls: usize) -> Bench {
        let mut jit = fixture().with_host_code();
        for _ in 0..calls {
            jit.advance(10_000);
        }
        assert!(jit.state.csrs.minstret > 0, "the compiled hart ran nothing");
        let mut interp = fixture();
        while interp.state.csrs.minstret < jit.state.csrs.minstret {
            interp.step();
        }
        assert_eq!(
            interp.state.csrs.minstret, jit.state.csrs.minstret,
            "the two harts retired different instructions"
        );
        assert_eq!(
            interp.state.cycles, jit.state.cycles,
            "the compiled hart charged a different number of bus cycles"
        );
        assert_eq!(interp.state.x, jit.state.x, "registers");
        assert_eq!(interp.state.pc, jit.state.pc, "pc");
        jit
    }

    /// [`PAGED_COLLIDE`] with its second page mapped, a megabyte up.
    fn paged_collide() -> Bench {
        let b = Bench::paged(&PAGED_COLLIDE);
        b.map_page(mmu::TLB_ENTRIES as u64, 0x5000);
        b
    }

    /// [`PAGED_LOAD`] on the page [`Bench::paged`] already maps.
    fn paged_load() -> Bench {
        Bench::paged(&PAGED_LOAD)
    }

    /// [`paged_collide`] with PMP made non-uniform over **one** of its two
    /// pages.
    ///
    /// The two pages share a TLB slot, so each load evicts the other's
    /// translation; one of them can be cached and the other cannot. That is
    /// the combination a page the shadow *refuses* has to survive: refusing is
    /// not "leave the slot alone", it is "write the slot, saying no". A
    /// refusal that skipped the write would leave the **other** page's entry
    /// sitting in it — live, matching its own tag, and standing for a
    /// translation this hart's TLB has just evicted.
    ///
    /// Entry 0 lies strictly inside physical 0x4000 and grants everything the
    /// guest actually does, so nothing traps and nothing changes except which
    /// of the two pages may be served inline.
    fn paged_collide_split_pmp() -> Bench {
        use super::super::csr::{PMP_ENTRIES, Priv};
        let mut b = paged_collide();
        b.cfg.pmp_count = PMP_ENTRIES;
        b.state = State::new(&b.cfg);
        b.state.csrs.satp = (8 << 60) | (0x1000 >> 12);
        b.state.csrs.priv_mode = Priv::Supervisor;
        b.state.pc = 0;
        b.state.csrs.pmpaddr[0] = (0x4010u64 >> 2) | 1; // 16 bytes at 0x4010
        b.state.csrs.pmpcfg[0] = 0b0001_1111; // NAPOT, R, W and X
        b.state.csrs.pmpaddr[1] = u64::MAX >> 10; // everything else
        b.state.csrs.pmpcfg[1] = 0b0001_1111; // NAPOT, R, W and X
        b
    }

    /// `x7 = 0`, then a load from a fixed address in a tight loop, in **bare
    /// mode** — no `satp`, no walk, no page table.
    ///
    /// A bare hart reaches [`Exec::refresh_shadow`] down its other arm: the
    /// hart's own TLB is not consulted at all, so there is no eviction to be
    /// in lockstep with and the shadow is filled on demand instead. Nothing
    /// tested that arm until a mutant that disabled it walked out.
    const BARE_LOAD: [u32; 3] = [
        0x0000_0393, // addi x7, x0, 0
        0x0003_be03, // ld   x28, 0(x7)   ; the loop starts here
        0xffdf_f06f, // jal  x0, -4
    ];

    /// Where [`REMOTE_LOAD`] reads from: a region of its own, clear of the
    /// code, so a remap can replace one without the other.
    const DATA: u64 = 0x2_0000;

    /// [`BARE_LOAD`], reading from [`DATA`] instead of from its own code page.
    const REMOTE_LOAD: [u32; 3] = [
        0x0002_03b7, // lui  x7, 0x20      ; x7 = 0x20000
        0x0003_be03, // ld   x28, 0(x7)    ; the loop starts here
        0xffdf_f06f, // jal  x0, -4
    ];

    #[test]
    fn a_bare_load_is_served_inline_too() {
        // Bare mode is the *easy* half of the argument — one bus cycle whether
        // the resolution was cached or not, because there is no walk to skip —
        // and it was the half with no test. Both that it happens and that it
        // costs the same are asserted, because a fill on the wrong arm shows
        // up only as a fast path that quietly never fires.
        let mut jit = Bench::new(&BARE_LOAD).with_host_code();
        for _ in 0..8 {
            jit.advance(10_000);
        }
        assert!(jit.state.csrs.minstret > 0, "the compiled hart ran nothing");
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(
            jit.fast_loads() > 0,
            "a bare hart never served a load inline"
        );
        let mut interp = Bench::new(&BARE_LOAD);
        while interp.state.csrs.minstret < jit.state.csrs.minstret {
            interp.step();
        }
        assert_eq!(interp.state.csrs.minstret, jit.state.csrs.minstret);
        assert_eq!(
            interp.state.cycles, jit.state.cycles,
            "the compiled hart charged a different number of bus cycles"
        );
        assert_eq!(interp.state.x, jit.state.x, "registers");
        // And the page really was resolved once rather than on every access:
        // the bare arm asks the flat view only when the slot is empty. Only
        // where there is a shadow at all — a build or a host without a code
        // generator attaches none, and a table nothing reads is a cost with no
        // benefit (`Jit::wants_shadow`).
        let stats = jit.shadow_stats();
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        {
            assert!(stats.fills > 0, "nothing was ever cached");
            assert!(
                stats.fills < jit.state.csrs.minstret,
                "the bare arm re-resolved the page on every access"
            );
        }
        let _ = stats;
    }

    /// A machine-mode hart whose **loads** translate and whose fetches do not.
    ///
    /// `MPRV` with `MPP = S` is how firmware reads a supervisor's memory, and
    /// it is the one configuration in which `effective_priv` gives a different
    /// answer for a fetch than for a load. The distinction is not cosmetic:
    /// the plan a block publishes and the tag a fill writes have to agree
    /// about *which* answer, or a load under `MPRV` probes with the tag a
    /// plain machine-mode load left behind — same address, no translation,
    /// different physical page, no fault. Wrong bytes, silently.
    ///
    /// The program loads from virtual zero, which S-mode maps to physical
    /// 0x4000 while machine mode would read the reset vector at zero. So the
    /// two answers are distinguishable in the register file, not merely in the
    /// cycle count.
    fn mprv_bench() -> Bench {
        use super::super::csr::{Priv, status};
        let mut b = Bench::paged(&PAGED_LOAD);
        // Machine mode, but MPRV is set and MPP is Supervisor, so a *load*
        // translates through `satp` and a fetch does not.
        b.state.csrs.priv_mode = Priv::Machine;
        b.state.csrs.mstatus |= status::MPRV;
        b.state.csrs.mstatus = (b.state.csrs.mstatus & !status::MPP)
            | ((Priv::Supervisor.bits()) << status::MPP_SHIFT);
        // The code has to be fetched *untranslated*, so it goes at physical
        // zero as well as at 0x4000 where the page table points.
        for (i, word) in PAGED_LOAD.iter().enumerate() {
            b.poke((i as u64) * 4, u64::from(*word));
        }
        b.state.pc = 0;
        b
    }

    #[test]
    fn a_load_under_mprv_is_tagged_with_the_privilege_it_actually_translates_in() {
        let mut jit = mprv_bench().with_host_code();
        for _ in 0..8 {
            jit.advance(10_000);
        }
        assert!(jit.state.csrs.minstret > 0, "the compiled hart ran nothing");
        let mut interp = mprv_bench();
        while interp.state.csrs.minstret < jit.state.csrs.minstret {
            interp.step();
        }
        assert_eq!(interp.state.csrs.minstret, jit.state.csrs.minstret);
        assert_eq!(
            interp.state.cycles, jit.state.cycles,
            "the compiled hart charged a different number of bus cycles"
        );
        assert_eq!(
            interp.state.x, jit.state.x,
            "the compiled hart read different bytes"
        );
        // The fixture is only a test of the tag if the load really did
        // translate: virtual zero resolves to physical 0x4000, whose first
        // word is the program, not the reset vector at physical zero.
        assert_eq!(
            jit.state.x[28] & 0xffff_ffff,
            u64::from(PAGED_LOAD[0]),
            "the load did not go through the page table"
        );
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(
            jit.fast_loads() > 0,
            "the fixture never reached the inlined path"
        );
    }

    /// `x7 = 0x800`, then a store and a reload in a tight loop, in **bare
    /// mode**.
    ///
    /// The store target is clear of the code, so the write never lands on the
    /// instructions and the block cache is not invalidated every pass — which
    /// is what lets the loop reach the compiled path more than once.
    const BARE_STORE: [u32; 5] = [
        0x0000_0013, // nop                ; x7 is seeded, so this is filler
        0x0000_0293, // addi x5, x0, 0
        0x0052_8293, // addi x5, x5, 5     ; the loop starts here
        0x0053_b023, // sd   x5, 0(x7)
        0xff9f_f06f, // jal  x0, -8
    ];

    /// [`BARE_STORE`] with `x7` seeded, because `addi` cannot express a
    /// positive 0x800 in one instruction and a second one would add a block
    /// boundary the fixture is not about.
    fn bare_store() -> Bench {
        let mut b = Bench::new(&BARE_STORE);
        b.state.x[7] = 0x800;
        b.state.pc = 4;
        b
    }

    #[test]
    fn a_bare_store_is_served_inline_too() {
        let mut jit = bare_store().with_host_code();
        for _ in 0..8 {
            jit.advance(10_000);
        }
        assert!(jit.state.csrs.minstret > 0, "the compiled hart ran nothing");
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(
            jit.fast_stores() > 0,
            "a bare hart never served a store inline"
        );
        let mut interp = bare_store();
        while interp.state.csrs.minstret < jit.state.csrs.minstret {
            interp.step();
        }
        assert_eq!(interp.state.csrs.minstret, jit.state.csrs.minstret);
        assert_eq!(
            interp.state.cycles, jit.state.cycles,
            "the compiled hart charged a different number of bus cycles for a store"
        );
        assert_eq!(interp.state.x, jit.state.x, "registers");
        // And the bytes really landed: the two harts wrote the same word, and
        // it is the word the register file says they wrote. A fast path that
        // wrote nowhere would pass every column above.
        let read = |b: &Bench| {
            b.space
                .read(0x800, Width::U64, MemAttrs::DEFAULT)
                .expect("in RAM")
        };
        assert_eq!(read(&jit), read(&interp), "guest memory");
        assert_eq!(read(&jit), jit.state.x[5], "the store wrote the wrong word");
    }

    /// A loop that rewrites an instruction it is about to execute.
    ///
    /// `sb x5, 0x43(x0)` writes the **top byte** of the word at 0x40, which is
    /// `imm[11:4]` of an `addi` — so the opcode stays valid and the increment
    /// changes on every pass. `x28` therefore accumulates a different number
    /// each time round, and only a hart that re-lifted the page after the
    /// write gets the same total as the interpreter.
    ///
    /// The store is inlined from the second pass onward: the block cache is
    /// invalidated by the write, but the shadow's store entry is not. That is
    /// the arm where nothing except `note_fast_store` pays the guest-physical
    /// dirty log, and a compiled hart that skipped it runs the *previous*
    /// pass's instruction — which is exactly the bug Linux hits in module
    /// loading and in alternatives patching.
    const SELF_MODIFY: [u32; 18] = [
        0x0012_8293, // 0x00: addi x5, x5, 1
        0x0450_01a3, // 0x04: sb   x5, 0x43(x0)   ; imm[11:4] of the word at 0x40
        0x0380_006f, // 0x08: jal  x0, +0x38      ; -> 0x40
        0x0000_0013, // 0x0c: nop
        0x0000_0013, // 0x10
        0x0000_0013, // 0x14
        0x0000_0013, // 0x18
        0x0000_0013, // 0x1c
        0x0000_0013, // 0x20
        0x0000_0013, // 0x24
        0x0000_0013, // 0x28
        0x0000_0013, // 0x2c
        0x0000_0013, // 0x30
        0x0000_0013, // 0x34
        0x0000_0013, // 0x38
        0x0000_0013, // 0x3c
        0x000e_0e13, // 0x40: addi x28, x28, 0    ; the immediate is rewritten
        0xfbdf_f06f, // 0x44: jal  x0, -0x44      ; -> 0x00
    ];

    #[test]
    fn a_store_the_backend_inlined_still_invalidates_the_block_it_landed_on() {
        let mut jit = Bench::new(&SELF_MODIFY).with_host_code();
        for _ in 0..16 {
            jit.advance(10_000);
        }
        assert!(jit.state.csrs.minstret > 0, "the compiled hart ran nothing");
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(
            jit.fast_stores() > 0,
            "the self-modifying store was never inlined, so this tests nothing"
        );
        let mut interp = Bench::new(&SELF_MODIFY);
        while interp.state.csrs.minstret < jit.state.csrs.minstret {
            interp.step();
        }
        assert_eq!(interp.state.csrs.minstret, jit.state.csrs.minstret);
        assert!(
            jit.state.x[5] > 2,
            "the fixture never got past its second pass"
        );
        assert_eq!(
            interp.state.x, jit.state.x,
            "the compiled hart executed different bytes after rewriting them"
        );
        assert_eq!(interp.state.pc, jit.state.pc, "pc");
        assert_eq!(interp.state.cycles, jit.state.cycles, "the cycle counter");
    }

    /// `lr.d`, an ordinary store into the reservation, then `sc.d`.
    ///
    /// The reservation is the one piece of guest-visible state a store touches
    /// that has nothing to do with memory, and an inlined store never reaches
    /// `Exec::store` where the rule lives. An `sc` that succeeded because the
    /// compiled path forgot to break the reservation is a lost update in guest
    /// software and shows up nowhere else.
    ///
    /// `lr.d` and `sc.d` are outside the lifted subset, so they are
    /// interpreted and the `sd` between them sits in a compiled block of its
    /// own. `x11` accumulates `sc.d`'s result: 1 on every pass if the
    /// reservation was broken, 0 if it was not.
    const RESERVED_STORE: [u32; 9] = [
        0x0000_0013, // 0x00: nop                 ; x7 is seeded
        0x0013_0313, // 0x04: addi x6, x6, 1      ; the loop starts here
        0x1003_b2af, // 0x08: lr.d  x5, (x7)
        0x0014_8493, // 0x0c: addi x9, x9, 1
        0x0063_b023, // 0x10: sd    x6, 0(x7)     ; breaks the reservation
        0x0015_0513, // 0x14: addi x10, x10, 1
        0x1863_b42f, // 0x18: sc.d  x8, x6, (x7)  ; must fail
        0x0085_85b3, // 0x1c: add   x11, x11, x8
        0xfe5f_f06f, // 0x20: jal   x0, -0x1c
    ];

    /// [`RESERVED_STORE`] with its scratch word on the page above the code, so
    /// the store does not invalidate the block it is in.
    fn reserved_store() -> Bench {
        let mut b = Bench::new(&RESERVED_STORE);
        b.state.x[7] = 0x1000;
        b.state.pc = 4;
        b
    }

    /// `note_fast_store` in isolation, under **translation**: the global
    /// monitor is keyed on the physical address, and an inlined store is the
    /// one place in this core that has to translate the pair itself.
    ///
    /// Driven directly rather than through a run, because a run cannot be made
    /// to prove it: with the MMU off a virtual address and a physical one are
    /// the same number, and a paged run interleaves interpreted stores that
    /// reach the monitor through `SpaceView::write_span` anyway. One call, one
    /// page table, no ambiguity.
    #[test]
    fn note_fast_store_reports_the_physical_address() {
        use super::super::mmu::pte;
        // `Bench::paged` maps virtual page 0 onto physical 0x4000, but its leaf
        // is read-execute; a store needs write and dirty.
        let mut b = Bench::paged(&[]);
        b.poke(
            0x3000,
            ((0x4000 >> 12) << 10) | pte::V | pte::R | pte::W | pte::X | pte::A | pte::D,
        );
        let sibling = MonitorSlot::new(Arc::clone(&b.space), 3).expect("a free slot");
        b.tlb.attach_shadow(Arc::clone(&b.space));

        let Bench {
            space,
            cfg,
            state,
            tlb,
            lines,
            ..
        } = &mut b;
        let mut exec = Exec::new(state, tlb, space, cfg, lines, ExitMask::NONE, None);
        // One ordinary store first, so the software TLB and the shadow beside
        // it hold an entry for the page: an inlined store is only ever issued
        // against an entry generated code has already resolved.
        exec.store(0x40, 8, 0).expect("the page is writable RAM");

        // Virtual 0x40 is physical 0x4040, and those are the two numbers a
        // virtual key would confuse.
        sibling.reserve(0x4040);
        assert!(sibling.holds(), "the reservation was taken");
        exec.note_fast_store(0x40, 8);
        assert!(
            !sibling.holds(),
            "an inlined store named its virtual address to the global monitor, \
             which is keyed on the physical one"
        );
    }

    #[test]
    fn an_inlined_store_breaks_a_siblings_reservation_too() {
        // The other half, and the one SMP needs.
        // [`an_inlined_store_breaks_a_reservation_it_lands_in`] proves a
        // compiled store breaks the *storing* hart's own reservation; this one
        // proves it reaches the **global** monitor, where a sibling hart's
        // reservation lives. An inlined store never calls
        // `SpaceView::write_span`, so `Exec::note_fast_store` has to tell the
        // monitor itself — and 99.8% of this core's stores are inlined.
        //
        // The sibling is a bare `MonitorSlot` rather than a second hart because
        // what is under test is the hook, not the interleaving: a registration
        // on the same space is exactly what a second hart would hold.
        let mut jit = bare_store().with_host_code();
        // Warm the loop first, so the stores in the budget under test come out
        // of generated code rather than out of the interpreter.
        for _ in 0..8 {
            jit.advance(10_000);
        }
        let before = jit.fast_stores();

        let sibling =
            MonitorSlot::new(Arc::clone(&jit.space), 3).expect("the space has a free slot");
        // `BARE_STORE` stores to `x7` = 0x800, and with no translation active
        // that is also the physical address.
        sibling.reserve(0x800);
        assert!(sibling.holds(), "the reservation was taken");
        jit.advance(10_000);

        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(
            jit.fast_stores() > before,
            "no store was inlined in the budget under test"
        );
        let _ = before;
        assert!(
            !sibling.holds(),
            "a compiled store left a sibling's reservation standing, so its              `sc.d` would succeed against a word this hart had already              overwritten"
        );
    }

    #[test]
    fn an_inlined_store_breaks_a_reservation_it_lands_in() {
        let mut jit = reserved_store().with_host_code();
        // More calls than the other fixtures need: `lr.d` and `sc.d` are
        // outside the lifted subset, so each pass leaves the compiled path
        // twice and one `advance` gets through less than one iteration.
        for _ in 0..200 {
            jit.advance(10_000);
        }
        assert!(jit.state.csrs.minstret > 0, "the compiled hart ran nothing");
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(
            jit.fast_stores() > 0,
            "the store between the pair was never inlined"
        );
        assert!(
            jit.state.x[11] > 1,
            "every store-conditional succeeded, so the reservation survived a \
             store into it"
        );
        let mut interp = reserved_store();
        while interp.state.csrs.minstret < jit.state.csrs.minstret {
            interp.step();
        }
        assert_eq!(interp.state.csrs.minstret, jit.state.csrs.minstret);
        assert_eq!(interp.state.x, jit.state.x, "registers");
        assert_eq!(interp.state.cycles, jit.state.cycles, "the cycle counter");
    }

    /// [`BARE_STORE`] at a **byte** width, so the truncation the fast path
    /// applies is visible in the neighbouring bytes.
    ///
    /// A `mov` of the wrong width through a host pointer overwrites guest
    /// bytes the store never named, and nothing about the storing register
    /// would show it.
    const BARE_STORE_BYTE: [u32; 5] = [
        0x0000_0013, // nop
        0x0000_0293, // addi x5, x0, 0
        0x0052_8293, // addi x5, x5, 5     ; the loop starts here
        0x0053_8023, // sb   x5, 0(x7)
        0xff9f_f06f, // jal  x0, -8
    ];

    #[test]
    fn an_inlined_store_writes_its_width_and_not_a_byte_more() {
        let mut jit = Bench::new(&BARE_STORE_BYTE).with_host_code();
        jit.state.x[7] = 0x800;
        jit.state.pc = 4;
        // A recognisable pattern in the seven bytes above the target, so an
        // over-wide store shows up as a changed neighbour rather than as a
        // zero that was already there.
        for i in 1..8u64 {
            jit.poke(0x800 + i, 0xa5);
        }
        for _ in 0..8 {
            jit.advance(10_000);
        }
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(jit.fast_stores() > 0, "the byte store was never inlined");
        for i in 1..8u64 {
            assert_eq!(
                jit.space
                    .read(0x800 + i, Width::U8, MemAttrs::DEFAULT)
                    .expect("in RAM"),
                0xa5,
                "a one-byte store disturbed the byte at +{i}"
            );
        }
    }

    /// [`Bench::paged`] with a **writable** leaf, and a store loop on it.
    ///
    /// The leaf carries `A` but deliberately **not** `D`, because that is a
    /// store plan's hardest promise: a compiled store skips the walk, and it
    /// may only do so because the walk that filled the store entry set the
    /// dirty bit first.
    const PAGED_STORE: [u32; 4] = [
        0x0000_0393, // addi x7, x0, 0
        0x0052_8293, // addi x5, x5, 5    ; the loop starts here
        0x0453_b823, // sd   x5, 80(x7)
        0xff9f_f06f, // jal  x0, -8
    ];

    fn paged_store() -> Bench {
        use super::super::mmu::pte;
        let b = Bench::paged(&PAGED_STORE);
        // Readable, writable, executable and accessed — but not dirty.
        let leaf = pte::V | pte::R | pte::W | pte::X | pte::A;
        b.poke(0x3000, ((0x4000 >> 12) << 10) | leaf);
        b
    }

    #[test]
    fn a_paged_store_is_served_inline_and_costs_what_the_interpreter_charged() {
        let jit = paged_engines_agree(paged_store, 8);
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(
            jit.fast_stores() > 0,
            "the fixture never reached the path it exists to test"
        );
        let _ = jit;
    }

    #[test]
    fn a_store_to_a_clean_page_sets_its_dirty_bit_before_anything_is_inlined() {
        use super::super::mmu::pte;
        let mut jit = paged_store().with_host_code();
        let leaf = |b: &Bench| {
            b.space
                .read(0x3000, Width::U64, MemAttrs::DEFAULT)
                .expect("in RAM")
        };
        assert_eq!(leaf(&jit) & pte::D, 0, "the fixture starts clean");
        for _ in 0..8 {
            jit.advance(10_000);
        }
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(jit.fast_stores() > 0, "no store was ever inlined");
        assert_ne!(
            leaf(&jit) & pte::D,
            0,
            "a compiled store left the page's dirty bit clear, which is the one \
             thing a store plan may never do"
        );
    }

    /// [`PAGED_STORE`] on a page the guest may **read** but not write.
    ///
    /// The two sets are the point: a load may be inlined here and a store may
    /// not, because `W` is a different bit from `R` and the shadow's store set
    /// is filled only by a walk that checked it. A store plan that reused the
    /// load set would write straight through a read-only mapping.
    fn paged_read_only() -> Bench {
        use super::super::mmu::pte;
        let b = Bench::paged(&PAGED_STORE);
        let leaf = pte::V | pte::R | pte::X | pte::A;
        b.poke(0x3000, ((0x4000 >> 12) << 10) | leaf);
        b
    }

    #[test]
    fn a_store_to_a_page_without_write_permission_traps_under_both_engines() {
        use super::super::csr::cause;
        // Only as far as the first trap: `mtvec` is zero and machine mode
        // fetches virtual zero untranslated, where this fixture has nothing,
        // so running on would replace the cause under test with an illegal
        // instruction.
        let mut jit = paged_read_only().with_host_code();
        let jit_ticks = jit.run_until_trap(Bench::advance);
        let mut interp = paged_read_only();
        let interp_ticks = interp.run_until_trap(|b, _| b.step());
        assert_eq!(
            jit.state.csrs.mcause,
            cause::STORE_PAGE_FAULT,
            "the compiled hart wrote through a read-only mapping"
        );
        assert_eq!(interp.state.csrs.mcause, jit.state.csrs.mcause);
        assert_eq!(interp.state.csrs.mtval, jit.state.csrs.mtval);
        assert_eq!(
            interp_ticks, jit_ticks,
            "the trap cost a different number of ticks"
        );
        // The word the store aimed at is still what the page was built with,
        // not the register the loop was counting in.
        assert_eq!(
            jit.space
                .read(0x4050, Width::U64, MemAttrs::DEFAULT)
                .expect("in RAM"),
            0,
            "the refused store landed anyway"
        );
    }

    #[test]
    fn a_paged_load_is_served_inline_and_costs_what_the_interpreter_charged() {
        let jit = paged_engines_agree(paged_load, 8);
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(
            jit.fast_loads() > 0,
            "the fixture never reached the path it exists to test"
        );
        let _ = jit;
    }

    #[test]
    fn a_translation_the_hart_evicted_is_not_served_inline() {
        // Two pages one slot apart. The agreement is the assertion — if the
        // shadow outlived the hart's own entry, the compiled hart would charge
        // one tick for a load the interpreter walks for, and `cycles` would
        // part company.
        let jit = paged_engines_agree(paged_collide, 8);
        assert_ne!(jit.state.x[28], 0, "the first load read the code page");
        assert_eq!(jit.state.x[8], 0x10_0000, "the second page was addressed");
    }

    /// [`Bench::paged`] with PMP on and an entry lying *wholly inside* the
    /// page the load reads.
    ///
    /// Everything the guest does is still permitted — the point is that the
    /// answer is reached through two different entries depending on where in
    /// the page you ask, and the compiled path cannot see which.
    fn paged_with_split_pmp() -> Bench {
        use super::super::csr::{PMP_ENTRIES, Priv};
        let mut b = Bench::paged(&PAGED_LOAD);
        b.cfg.pmp_count = PMP_ENTRIES;
        b.state = State::new(&b.cfg);
        b.state.csrs.satp = (8 << 60) | (0x1000 >> 12);
        b.state.csrs.priv_mode = Priv::Supervisor;
        b.state.pc = 0;
        // Strictly inside the page, touching neither end of it — the case
        // `pmp_allows` asked about the whole page cannot see.
        b.state.csrs.pmpaddr[0] = (0x4010u64 >> 2) | 1; // 16 bytes at 0x4010
        b.state.csrs.pmpcfg[0] = 0b0001_1101; // NAPOT, R and X
        b.state.csrs.pmpaddr[1] = u64::MAX >> 10; // everything else
        b.state.csrs.pmpcfg[1] = 0b0001_1111; // NAPOT, R, W and X
        b
    }

    /// [`Bench::paged`] with a store loop, and PMP over its page granting
    /// **loads but not stores**.
    ///
    /// PMP is *uniform* over the page here — no entry's edge falls inside it —
    /// so the question `Exec::refresh_shadow` asks is not "is this page
    /// uniform" but "what does PMP say about the access kind I am filling
    /// for". A fill that asked about loads would cache the page and let a
    /// compiled store write straight through a refusal the interpreter takes
    /// an access fault on.
    ///
    /// Entry 0 covers exactly the 4 KiB page at physical 0x4000 with `R` and
    /// `X` and no `W`; entry 1 grants everything else, so the walk's own PTE
    /// reads still pass.
    fn paged_store_denied_by_pmp() -> Bench {
        use super::super::csr::{PMP_ENTRIES, Priv};
        use super::super::mmu::pte;
        let mut b = Bench::paged(&PAGED_STORE);
        let leaf = pte::V | pte::R | pte::W | pte::X | pte::A | pte::D;
        b.poke(0x3000, ((0x4000 >> 12) << 10) | leaf);
        b.cfg.pmp_count = PMP_ENTRIES;
        b.state = State::new(&b.cfg);
        b.state.csrs.satp = (8 << 60) | (0x1000 >> 12);
        b.state.csrs.priv_mode = Priv::Supervisor;
        b.state.pc = 0;
        // NAPOT, 4 KiB at 0x4000: the base is `0x4000 >> 2` with nine trailing
        // ones, which is what encodes 2^12 bytes.
        b.state.csrs.pmpaddr[0] = (0x4000u64 >> 2) | 0x1ff;
        b.state.csrs.pmpcfg[0] = 0b0001_1101; // NAPOT, R and X — no W
        b.state.csrs.pmpaddr[1] = u64::MAX >> 10;
        b.state.csrs.pmpcfg[1] = 0b0001_1111; // NAPOT, R, W and X
        b
    }

    #[test]
    fn a_page_pmp_denies_stores_over_is_never_stored_to_inline() {
        use super::super::csr::cause;
        // Only as far as the first trap: `mtvec` is zero and machine mode
        // fetches virtual zero untranslated, where this fixture has nothing.
        let mut jit = paged_store_denied_by_pmp().with_host_code();
        let jit_ticks = jit.run_until_trap(Bench::advance);
        let mut interp = paged_store_denied_by_pmp();
        let interp_ticks = interp.run_until_trap(|b, _| b.step());
        assert_eq!(
            jit.state.csrs.mcause,
            cause::STORE_ACCESS,
            "the fixture did not reach the refusal it exists for"
        );
        assert_eq!(interp.state.csrs.mcause, jit.state.csrs.mcause);
        assert_eq!(interp.state.csrs.mtval, jit.state.csrs.mtval);
        assert_eq!(
            interp_ticks, jit_ticks,
            "the trap cost a different number of ticks"
        );
        assert_eq!(
            jit.fast_stores(),
            0,
            "a page PMP refuses stores over must never be stored to inline"
        );
        // And it is a refusal rather than an absence: the shadow *was* asked,
        // and said no every time. A fill that asked the load question would
        // have said yes, and the counters are the only place that shows.
        let stats = jit.shadow_stats();
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        {
            assert!(stats.fills > 0, "the shadow was never asked about the page");
            assert_eq!(
                stats.refused, stats.fills,
                "the page was cached for stores although PMP refuses them"
            );
        }
        let _ = stats;
    }

    #[test]
    fn a_page_the_shadow_refuses_still_evicts_what_shared_its_slot() {
        // One page cacheable, one not, both in the same slot. If a refusal
        // left the slot as it found it, the cacheable page's entry would
        // outlive the translation it stands for and the compiled hart would
        // serve a load the interpreter walks for. The cycle counts are the
        // assertion.
        let mut jit = paged_engines_agree(paged_collide_split_pmp, 8);
        assert_eq!(jit.state.csrs.mcause, 0, "the guest trapped");
        // And the fixture really is the mixed case, which the timing cannot
        // show: these two pages evict each other every iteration, so *neither*
        // is ever served inline and `fast_loads` is zero either way. What
        // matters is that both arms of `refresh_shadow` ran — some page was
        // cached, some page was refused — because a refusal that never
        // happened cannot fail to evict.
        let stats = jit.shadow_stats();
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        {
            assert!(stats.refused > 0, "no page was ever refused");
            assert!(
                stats.fills > stats.refused,
                "no page was ever cacheable, so nothing was refused *against*"
            );
        }
        let _ = stats;
    }

    #[test]
    fn a_page_pmp_does_not_answer_uniformly_over_is_never_served_inline() {
        let mut jit = paged_with_split_pmp().with_host_code();
        for _ in 0..8 {
            jit.advance(10_000);
        }
        assert!(jit.state.csrs.minstret > 0, "the compiled hart ran nothing");
        assert_eq!(
            jit.fast_loads(),
            0,
            "a page PMP is not uniform over must never be served inline"
        );
        // And it is a refusal to go fast, not a refusal to run: the guest took
        // no trap and reached the same state a purely interpreted one would.
        assert_eq!(jit.state.csrs.mcause, 0, "the guest trapped");
        let mut interp = paged_with_split_pmp();
        while interp.state.csrs.minstret < jit.state.csrs.minstret {
            interp.step();
        }
        assert_eq!(interp.state.csrs.minstret, jit.state.csrs.minstret);
        assert_eq!(interp.state.cycles, jit.state.cycles);
        assert_eq!(interp.state.x, jit.state.x);
    }

    #[test]
    fn a_hart_without_a_code_generator_never_grows_a_shadow() {
        // The shadow is not free — a fill probes the flat view — and only the
        // host code generator reads one. The portable backend must not pay for
        // a table nothing looks at.
        let plain = hart(Engine::Jit, &LOOP);
        for _ in 0..8 {
            plain.run_budget(1000);
        }
        assert!(!plain.has_shadow(), "`jit` asked for a shadow");
        let host = hart(Engine::JitHost, &LOOP);
        for _ in 0..8 {
            host.run_budget(1000);
        }
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        assert!(host.has_shadow(), "`jit-host` did not ask for one");
        #[cfg(not(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64")))]
        assert!(
            !host.has_shadow(),
            "no backend on this host, so nothing reads a shadow"
        );
        let interp = hart(Engine::Interp, &LOOP);
        for _ in 0..8 {
            interp.run_budget(1000);
        }
        assert!(!interp.has_shadow(), "the oracle asked for a shadow");
    }
}
