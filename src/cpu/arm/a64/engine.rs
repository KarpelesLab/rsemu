//! The translated execution engine: this core's blocks, through [`jit`].
//!
//! [`lift`](super::lift) is the frontend, [`jit`] is the runtime, and this
//! module is what joins them to a *machine*: `engine = "jit"` on a
//! `cpu.arm.a64` object reaches [`advance`], and `machines/arm64-virt.machine`
//! boots on it.
//!
//! [`jit`]: crate::jit
//!
//! # The claim, and what it costs to keep
//!
//! *A cache hit, a cache miss, an interpreted run and a compiled run are
//! indistinguishable to the guest, **including cycle counts**.*
//! (`ROADMAP.md` §0.) That is the shape of this file rather than a hope — each
//! rule below exists because the alternative diverges:
//!
//! * **The memory path is the interpreter's, literally.** [`IrHost::load`] and
//!   [`IrHost::store`] here call `Exec::load` and `Exec::store` — the same
//!   functions `Exec::step` calls, over the same `mmu::Tlb`, charging the same
//!   ticks through the same `Exec::charge`. Not *a* memory path that agrees,
//!   *the* memory path. A second implementation would have to reproduce the
//!   alignment check, the per-byte split, the walk's tick cost and the broken
//!   reservation, and `differential`'s own host is the evidence that
//!   reproducing them is a job rather than a line.
//! * **The entry fetch translation happens on every block execution**, not
//!   once at lift time. A translated block skips the fetch, but the
//!   interpreter's first fetch *translates*, and a translation that misses the
//!   core's TLB walks and charges for the walk. A cached block that skipped it
//!   would run the same instructions for fewer ticks the second time round.
//!   [`admit`] is where it happens, [`Frontend::enter`] is the hook that lets
//!   a *chained* successor pay it too, and a call runs up to [`CHAIN`] blocks.
//!
//!   The other half of that rule is the one the RISC-V engine records as
//!   having cost it four ticks of drift on a cached corpus, and it is honoured
//!   here: a **failed** entry translation must not be charged twice. When
//!   [`admit`] answers [`Admit::Trap`] the block does not run and the trap is
//!   delivered from the walk that just happened — the interpreter is not asked
//!   to re-walk, because it would charge the walk again.
//! * **A block leaves at the instruction boundary the quantum runs out on.**
//!   The guest's *stopping point* inside a scheduler quantum must not depend
//!   on the engine: the overrun is carried as `State::debt`, and both it and
//!   the cycle count are in the snapshot a machine's state hash is taken over.
//!   [`IrHost::spent`] is the seam — [`Host::allowance`] against `Exec::used`,
//!   asked at every guest instruction boundary but a block's first — and it
//!   makes the stopping point *equal* to an interpreted core's rather than
//!   merely bounded, because `Cpu::run_budget` runs an instruction on exactly
//!   the same condition.
//!
//!   It replaced a guard that refused any block whose worst case did not fit,
//!   and that guard is the largest number this file has ever recorded. A cold
//!   PC's worst case is 5 188 ticks — sixty-four instructions of a split,
//!   walked pair access — against an `arm64-virt` quantum of 10 000, so the
//!   last half of every quantum could admit nothing; and the PC after each
//!   interpreted instruction is in the middle of a block, so the tail never
//!   recovered. `Probe`, which lifted a cold PC to price it rather than
//!   guessing, closed half of that. The seam closes the rest and deletes both.
//! * **A pending interrupt is looked for at every block boundary**, chained
//!   ones included, which is what keeps a sixteen-block chain
//!   indistinguishable from sixteen one-block calls. A store into the GIC ends
//!   its block by construction ([`lift`](super::lift), "A store ends the
//!   block"), so the interrupt it raises is seen before the next block starts.
//!
//! # What it buys, measured
//!
//! On the guest this exists for — `machines/arm64-virt.machine` booting
//! Linux 6.12.94 `arm64` with a busybox initramfs, 1 GiB of DRAM, `engine =
//! "jit-host"` — over the **boot**, from reset to `/init`'s own banner. The
//! window is a guest-side milestone rather than a wall-clock one, so both
//! columns do exactly the same guest work: the same 137 984 quanta and the
//! same 1 071 503 716 guest instructions, give or take the three that an
//! exception entry is counted as when it is taken from the interpreter rather
//! than out of a block. The median of three interleaved reps is reported,
//! because the host is shared and a control measured in a different sitting is
//! not one.
//!
//! | | before [`IrHost::spent`] | with it |
//! | --- | --- | --- |
//! | wall clock | 23.30 s | **19.54 s** (1.19×) |
//! | guest instructions retired **inside** a block | 1 045 147 550 (97.54%) | 1 065 491 373 (**99.44%**) |
//! | guest instructions the interpreter took | 26 356 169 | **6 012 343** (−77.2%) |
//!
//! Every run of both columns charged the same cycles and ended on the same
//! state hash, which is the claim at the top of this file: the guard went, and
//! the guest did not notice.
//!
//! An older sweep, over twenty seconds of virtual time and against the
//! interpreter, is the ratio worth keeping beside that: `interp` 18.94 s,
//! `jit` 10.42 s (1.82×), `jit-host` 3.37 s (**5.61×**). What the mechanisms
//! did over that run:
//!
//! | | |
//! | --- | --- |
//! | blocks executed | 23 810 578 |
//! | of those, compiled to host code | 23 809 916 (**99.997%**) |
//! | of those, reached by a patched exit | 20 571 853 (86.4%) |
//! | distinct blocks lifted | 17 638 |
//! | compiled loads served by an inlined probe | 18 712 518 |
//! | compiled stores served the same way | 13 350 310 |
//! | translations a guest store invalidated | 10 037 |
//!
//! The blocks the code generator refused are the ones holding a `UDIV` or an
//! `SDIV`, which are the only two ops this frontend emits that `jit::x86` does
//! not lower. That is 0.003%, and it is why [`lift`](super::lift) goes out of
//! its way not to emit [`Opcode::ADDC`](crate::ir::Opcode::ADDC): the
//! architecture's own `AddWithCarry` is refused too, and `CMP` is `SUBS`, so a
//! lifter that used it would have had *every* block refused rather than six
//! hundred.
//!
//! The inlined probes are worth a step of that ratio on their own: before
//! `mmu::Tlb` had a shadow to publish, the same sweep put `jit-host` at
//! 6.92 s and **2.54×**.
//!
//! ## Where the interpreter is still reached, and where it used to be
//!
//! The instructions the interpreter ran over the boot above, by the reason
//! [`admit`] gave:
//!
//! | why the interpreter ran it | instructions | of all interpreted |
//! | --- | --- | --- |
//! | outside the lifted subset, already known | 6 010 437 | **99.97%** |
//! | a lift that produced nothing (the first sighting of such a PC) | 1 614 | 0.03% |
//! | a pending interrupt or a stalled `WFI` | 292 | 0.00% |
//! | the budget guard declined a block | **0** | — |
//!
//! That last row is the whole of this round. The same boot before the seam
//! interpreted 26 356 169 instructions, and since the guest work is identical
//! the difference — **20 343 826**, 77.2% of them, 1.90% of every guest
//! instruction in the boot — was the guard and nothing else.
//!
//! What is left is the frontend's documented exclusions, and they were
//! profiled by decoded table row over a boot of this kind: `MRS` 1 024 738 (of
//! which `SP_EL0` alone is 645 329 — Linux's `current`), the exclusives and
//! the acquire/release accesses 339 581, `MSR` 224 806, `SYS` — the
//! `DC`/`IC`/`TLBI` maintenance operations — 204 552, and `RBIT` 1 584. **Not
//! one SIMD or floating-point instruction executed**, and no `LDTR`/`STTR`
//! either: an `arm64` kernel booting to a busybox shell uses neither, so the
//! largest documented absence in the frontend is worth nothing at all on the
//! guest this core exists to run. That is written down so the next person
//! picks by measurement rather than by list order.
//!
//! # What is checked at a block boundary rather than at an instruction
//!
//! **Pending interrupts and the `WFI` stall**, and within a block nothing this
//! core does can raise either: `MSR`, `MRS`, `ERET`, `WFI`, `SVC` and every
//! system operation are outside the lifted subset and end the block, a
//! **store** ends the block by construction, and a line asserted from outside
//! changes between quanta, where both engines meet it at the same boundary.
//!
//! **The generic timer is not in that list, and used to be.** Its comparator
//! is reached by ticks the block itself charges, so nothing outside the run
//! decides when — an interpreted core samples `CNTPCT_EL0` once per
//! instruction and acts at the next boundary, and a chain that ran on to its
//! natural end took the same interrupt up to [`lift::MAX_INSNS`] instructions
//! later. That was written down here as "a real, bounded imprecision", and
//! bounded is not the property that matters: `ROADMAP.md` §0 asks for one
//! state hash across the engines, and an interrupt taken tens of ticks late is
//! a different `ELR_EL1` and, on a real guest, a different scheduling decision
//! after it. It cost an arm64 Linux boot its agreement at **23.46 seconds** of
//! guest time — the first timer to fire while the guest was inside lifted code
//! rather than parked in `WFI` — and nothing before that saw it.
//!
//! `Exec::timer_edge` closes it: the cycle the comparator is crossed on is
//! computed once per [`advance`] (the registers that decide it are written by
//! `MSR` and a block ends at one), and [`IrHost::spent`] compares the tick
//! counter against it at every guest instruction boundary, so the block leaves
//! on the boundary the interpreter would have taken the interrupt after.
//! `Exec::publish_timer_levels` is then called once per [`advance`] for the
//! same reason it is called once per `Exec::step`.
//!
//! [`Admitted::leave`] is the other half, and it took a third divergence to
//! find: that edge is computed **after** [`admit`] has looked for a pending
//! interrupt and then charged the entry translation, so a walk between the two
//! can cross the comparator with nobody left to notice. `Exec::timer_edge`
//! answers [`u64::MAX`] for an already-crossed comparator, which is the right
//! answer to its own question and the wrong edge for a run: the run has to
//! leave at its first boundary, not never. It needs a cold instruction fetch —
//! a `TLBI` — on the same instruction a timer fires on, which the synthetic
//! workload in `tests/engine_longrun.rs` reaches in 0.417 s of guest time and
//! a forty-second arm64 Linux boot does not reach at all.
//!
//! That started life as a four-line `leave_at` beside [`Host::new`], which
//! could only see the *first* block of a run and could only see the *timer*.
//! It is now the same question [`admit`] asks for the other two cores, at
//! every block boundary and about every interrupt: a walk's reads are as
//! capable of raising one as a walk's ticks, and a translation table over a
//! device is where that happens. See [`admit`].
//!
//! # Self-modifying code, and the one case that is not covered
//!
//! A store from a **translated block** is reported through [`StoreLog`] and
//! drained by the dispatcher at the next boundary. A store from an
//! **interpreted instruction** — an exclusive, an atomic, anything outside the
//! subset — is reported through the same `Exec::wrote` field the interpreter
//! fills, and [`advance`] drains it the same way.
//!
//! Bytes written by something that is **not** this core — a DMA engine, a
//! second core — are outside `jit::dispatch`'s contract and are not caught
//! here. The obvious hook is A64's own cache maintenance (`IC IVAU`, or the
//! `DSB`/`ISB` pair around it), and the RISC-V engine's measurement of the
//! equivalent — `FENCE.I`, 39 442 of them in thirty seconds of Linux guest
//! time, each throwing the whole cache away before it could warm — is why it is
//! not the hook here either. Closing it needs a write notification from the
//! address space for masters that are not the CPU, which `core::space` does
//! not have.
//!
//! # The inlined memory path, and the three AArch64 questions it raised
//!
//! `jit::fast` lets a host publish the software-TLB set its own accesses
//! resolve through, so generated code can inline a load instead of calling
//! back. The condition is not *"no walk is owed"* but *"a hit in the published
//! table implies a hit in the table that owes the walk"*, and a host makes
//! that true by writing the two in lockstep — which is `mmu::Tlb`'s shadow,
//! filled by `Exec::translate` at the same index for the same page.
//!
//! Three things about this architecture looked as though they might forbid it,
//! and none does:
//!
//! * **`TCR_EL1.TBI`.** Address tagging would put two virtual addresses that
//!   differ in their top byte on one page, so a shadow keyed on the tagged
//!   address and a walk that ignored the tag would disagree. This core does
//!   not implement `TBI` at all — `mmu`'s regime selection reads the full
//!   64-bit address, and an address carrying a tag falls in neither half and
//!   takes a translation fault — so there is nothing to strip and nothing to
//!   get wrong. If it is ever implemented, the tag must be stripped in exactly
//!   one place and both tables must read it.
//! * **The two `TTBR`s.** Which base a walk starts from is a pure function of
//!   the virtual address, so it cannot change under a cached entry the way an
//!   x86 segment base can. That is what makes AArch64 structurally closer to
//!   RISC-V here than to the core that cannot publish a plan at all.
//! * **Granule selection.** `TCR_EL1.TG0`/`TG1` could in principle name a
//!   16 KiB or 64 KiB page, which `jit::Tlb`'s fixed 4 KiB index would then
//!   sub-divide. This core implements only the 4 KiB granule and *faults* on a
//!   `TCR` naming another (`mmu`'s `regime`), so the two agree by
//!   construction — and if a larger granule ever lands, a finer shadow index
//!   is conservative rather than wrong.
//!
//! The **ASID** is the one that needed checking rather than dismissing, and it
//! turned out to be already solved: `jit::Tlb` stamps `Epoch::translation`
//! into its tag, and an AArch64 ASID lives in `TTBR0_EL1[63:48]`, so changing
//! it is a `TTBR0_EL1` write and a `TTBR0_EL1` write bumps
//! `SysRegs::translation_gen` — the same counter this core's own TLB is tagged
//! with. The two go stale together, which is exactly the lockstep `jit::fast`
//! asks for.
//!
//! What AArch64 does *not* need is the half that cost the RISC-V engine the
//! most care: a protection check the page tables know nothing about. There is
//! no PMP here, so a page the walk permits is a page the compiled path may
//! reach, and `refresh_shadow` has nothing to refuse a page over.

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

use super::exec::{Exec, State, Trap};
use super::isa::Nzcv;
use super::lift::{self, Origin, PC, SP, Shape, Smc, World};
use super::mmu::{Access, Tlb};
use super::sysreg::sctlr;
use super::{Config, Lines};

/// How much of a block the frontend is allowed to swallow.
///
/// [`Shape::Trace`] is the dispatcher's shape: direct branches are merged, so
/// a loop unrolls into one translation and a guest register stays in a
/// temporary across the whole of it.
const SHAPE: Shape = Shape::Trace;

/// What a store does to the block it is in.
///
/// [`Smc::HostGuard`], because this core can afford the better of the two
/// answers: [`Host::note_writes`] already sees the **guest-physical** page of
/// every store a block makes, compiled or interpreted, on its way into
/// [`DirtyPages`], and comparing it against [`Admitted::base`] is one
/// comparison against a field that is already in cache.
///
/// [`Smc::EndBlock`] is what this core shipped first, and what it cost is the
/// number `benches/a64_linux_boot.rs` was written to find: a block ended at
/// every store, so a real arm64 Linux boot ran **6.44 guest instructions per
/// block** against a frontend limit of 64, and 56% of the profile was per-block
/// cost divided by that number.
///
/// `cpu::x86::lift` reached the same question first and answered it in the IR,
/// with a guard that compares *linear* pages — and then had to refuse its own
/// answer under paging, because two linear pages may alias one physical page.
/// This is the same idea with the comparison moved to the one place that sees
/// physical addresses, which is why it holds under translation and x86's does
/// not. `cpu::arm::a64::lift`'s module docs have the argument in full.
const SMC: Smc = Smc::HostGuard;

/// How many blocks one [`advance`] may chain before it hands control back.
///
/// `ROADMAP.md` §9's second mechanism is a block cache *with block chaining*,
/// and [`Frontend::enter`] is what makes it reachable: a chained successor
/// still owes the entry translation every block owes, so without that hook a
/// dispatcher can only be driven one block at a time. What chaining buys is
/// not the hash lookup it skips — that was never the expensive part — it is
/// everything *around* a short block: an `Exec`, a `Host` and its register-file
/// copy in and out, a `Lifter`, a cache resynchronisation and a trip through
/// `Cpu::run_budget`.
///
/// **What it costs is the safe point, and the new bound is stated rather than
/// implied.** `Cpu::run_budget` tests `ROADMAP.md` §4.7's exit flag between
/// calls to [`advance`], so a raised flag used to be honoured within one
/// instruction and is now honoured within at most `CHAIN` blocks — 1 024 guest
/// instructions — and still within what is left of the quantum's tick budget,
/// because every block of the chain is admitted against that budget before it
/// runs.
///
/// Sixteen rather than sixty-four because the curve is flat past it: the
/// per-block cost being amortized is a fixed overhead, so the second block of
/// a chain removes half of it and the sixteenth removes a fifteenth, and a
/// safe point is worth more than the last percent.
const CHAIN: usize = 16;

/// How many blocks this core's cache holds before it evicts.
///
/// `jit::BlockCache`'s own default is 8 192, which a Linux guest thrashes: the
/// RISC-V engine measured 1 096 143 insertions against 1 079 470 evictions
/// over four minutes of guest time at that size, where every eviction is a
/// re-lift and, with a code generator attached, a re-compile. The number is a
/// bound rather than an allocation — a board whose guest has a small working
/// set never fills it.
const BLOCKS: usize = 65536;

/// How many `(pc, key) -> inside the subset` answers are remembered.
///
/// Direct-mapped and keyed by the guest PC, exactly as the block cache is, and
/// sized with it so a resident block usually has a resident answer. A miss
/// costs a dispatcher round trip, never a wrong result.
const SUBSET_SLOTS: usize = 65536;

/// How big a host code buffer this core asks for: 256 MiB.
///
/// The mapping is anonymous, so what is not written is not resident, and it is
/// only asked for at all by `engine = "jit-host"`. `jit::x86::buf` flips a
/// page-sized window rather than the whole mapping, so a bigger buffer costs
/// address space and nothing per compile — which is what lets the number stand
/// where the RISC-V engine measured a 32 MiB buffer resetting 111 times in
/// four minutes of guest time.
#[cfg(any(
    all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
    all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
))]
const CODE_BUFFER: u64 = 256 << 20;

// ---------------------------------------------------------------------------
// What a core keeps between blocks
// ---------------------------------------------------------------------------

/// This core's translation state: the dispatcher, and the costs beside it.
///
/// **Derived state in the strict sense** (`ROADMAP.md` §4.5): never
/// serialized, and thrown away by a reset and by a snapshot restore. That is
/// also what makes a snapshot interchangeable between any two engines — there
/// is nothing engine-specific in one to interchange.
#[derive(Debug)]
pub(super) struct Jit {
    disp: Dispatcher,
    subset: Subset,
    retired: u64,
    interpreted: u64,
    /// Translations an *interpreted* store invalidated.
    ///
    /// Counted here rather than read off `DispatchStats::smc`, which only
    /// sees what a **block** wrote: the dispatcher drains `StoreLog` itself
    /// and never learns about [`drain`], so a statistic that read only its
    /// counter would report zero however well the other half worked. A
    /// mutation pass is how that was found, by way of an assertion that could
    /// not hold.
    smc: u64,
}

/// What this core's translated engine has done.
///
/// A statistic and never a behaviour — the engines are indistinguishable to
/// the guest — but every "did the mechanism actually run" number here is
/// separate from every "did they agree" assertion, because a backend whose
/// coverage is unmeasured is a backend whose coverage rots. A mutation pass
/// over this file is the reason [`Stats::smc`] exists: switching off the
/// self-modifying-code drain left every test in the tree passing, since the
/// only guests that exercised it never executed the bytes they rewrote.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Stats {
    /// Blocks executed.
    pub blocks: u64,
    /// Blocks executed as host code rather than interpreted IR.
    pub compiled: u64,
    /// Blocks reached by following a patched exit rather than a lookup.
    pub chained: u64,
    /// Distinct blocks lifted.
    pub translated: u64,
    /// Translations a store from a **block** invalidated, through
    /// [`StoreLog`].
    pub smc: u64,
    /// Translations a store from an **interpreted instruction** invalidated,
    /// through `drain`.
    ///
    /// Separate from [`Stats::smc`] because they are separate mechanisms on
    /// separate paths, and a single total lets either of them stop working
    /// while the other keeps the number above zero — which a mutation pass
    /// demonstrated by switching one off and watching the assertion hold.
    pub smc_interpreted: u64,
    /// Guest instructions that retired **inside** a block.
    pub retired: u64,
    /// Guest instructions the interpreter executed, one per call.
    pub interpreted: u64,
    /// Compiled loads served from an **inlined** software-TLB probe, with no
    /// call back into this core's memory path.
    pub fast_loads: u64,
    /// Compiled stores served the same way.
    pub fast_stores: u64,
}

impl Jit {
    /// A fresh engine.
    ///
    /// `host_code` asks for the host code generator; a build or a host without
    /// one gets the portable backend instead, which is not a failure and not a
    /// different guest (`ROADMAP.md` §9, "Backends").
    pub(super) fn new(host_code: bool) -> Jit {
        let disp = Dispatcher::with_cache(BlockCache::with_capacity(BLOCKS));
        #[cfg(any(
            all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
            all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
        ))]
        let disp = match host_code
            .then(|| crate::jit::host::Engine::with_capacity(CODE_BUFFER))
            .flatten()
        {
            Some(engine) => disp.with_backend(engine),
            None => disp,
        };
        let _ = host_code;
        Jit {
            disp,
            subset: Subset::new(),
            retired: 0,
            interpreted: 0,
            smc: 0,
        }
    }

    /// Throw every translation away.
    pub(super) fn flush(&mut self) {
        self.disp.cache_mut().flush();
        self.subset.clear();
    }

    /// What this engine has done.
    pub(super) fn stats(&self) -> Stats {
        let s = self.disp.stats();
        Stats {
            blocks: s.blocks,
            compiled: s.compiled,
            chained: s.chained,
            translated: s.translated,
            smc: s.smc,
            smc_interpreted: self.smc,
            retired: self.retired,
            interpreted: self.interpreted,
            fast_loads: self.fast().0,
            fast_stores: self.fast().1,
        }
    }

    /// What the host code generator's inlined probes served, if there is one.
    fn fast(&self) -> (u64, u64) {
        #[cfg(any(
            all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
            all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
        ))]
        {
            self.disp
                .backend()
                .map(crate::jit::host::Engine::stats)
                .map_or((0, 0), |s| (s.fast_loads, s.fast_stores))
        }
        #[cfg(not(any(
            all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
            all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
        )))]
        {
            (0, 0)
        }
    }

    /// Whether this engine can use an `mmu::Tlb` shadow.
    ///
    /// Only the host code generator inlines an access ([`FastMem`]); the
    /// portable backend calls [`IrHost::load`] for every one, so a shadow
    /// attached for it would be filled and never read. The shadow is not free
    /// — a fill probes the address space's flat view — so it is asked for by
    /// the one engine that reads it, and a `jit-host` that fell back to the
    /// portable backend — no `jit::host` backend for this target — does not
    /// ask.
    pub(super) fn wants_shadow(&self) -> bool {
        #[cfg(any(
            all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
            all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
        ))]
        {
            self.disp.backend().is_some()
        }
        #[cfg(not(any(
            all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
            all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
        )))]
        {
            false
        }
    }

    /// Hand the pages an *interpreted* instruction wrote to the block cache.
    ///
    /// The other half of the self-modifying-code contract: a block's own
    /// stores go through [`StoreLog`] and the dispatcher drains those itself,
    /// and this is for every instruction outside the lifted subset — an
    /// exclusive, an atomic, a byte written by a trap handler.
    pub(super) fn note_writes(&mut self, exec: &mut Exec<'_>) {
        let mut hit = 0usize;
        for i in 0..exec.wrote_n as usize {
            hit += self.disp.cache_mut().note_write(exec.wrote[i], 1);
        }
        exec.wrote_n = 0;
        self.smc = self.smc.wrapping_add(hit as u64);
        if hit > 0 {
            // A page a translation came from has changed, so every *negative*
            // answer in the table may have changed with it: an instruction
            // that was outside the subset can have been overwritten by one
            // that is not.
            self.subset.clear();
        }
    }
}

/// A direct-mapped table of `(pc, key) -> is there a block here`.
///
/// One job, and it is not the budget: reaching the interpreter for an
/// instruction outside the lifted subset without paying a dispatcher round
/// trip and a [`lift::lift`] that fails at its first instruction. Without it
/// every `MSR`, every `SVC` and every floating-point instruction costs both,
/// and an AArch64 kernel is full of all three.
///
/// **It used to hold a worst-case tick bound too, and that is gone.** The
/// bound existed for a guard that refused any block whose worst case did not
/// fit what was left of the quantum, because a block that overran would stop
/// the guest somewhere an interpreted core would not. [`IrHost::spent`] makes
/// that guard unnecessary: a block now *leaves* at the instruction boundary
/// where the allowance runs out, which is the same instruction the interpreter
/// stops on, so there is nothing left to price. A collision here loses an
/// answer and costs one dispatcher round trip.
#[derive(Debug)]
struct Subset {
    slots: Box<[Slot]>,
    /// Which era of the table an entry has to carry to be believed.
    ///
    /// **Bumped rather than swept**, because a sweep is not free at this size
    /// and it happens on every self-modifying-code hit: a guest store into a
    /// page a translation came from clears the whole table, and a Linux boot
    /// does that thousands of times. Sixty-five thousand slots of thirty-two
    /// bytes is two megabytes of `memset` per clear, on the emulation thread,
    /// for a table that is a pure cache — so the invalidation is a counter and
    /// a stale slot is simply never believed again.
    era: u64,
}

#[derive(Debug, Clone, Copy, Default)]
struct Slot {
    pc: u64,
    key: u64,
    /// Whether the instruction at that PC is inside the lifted subset.
    inside: bool,
    /// The [`Subset::era`] this answer was recorded under; zero is *never*,
    /// which is what makes a freshly allocated table empty.
    era: u64,
}

impl Subset {
    fn new() -> Subset {
        Subset {
            slots: vec![Slot::default(); SUBSET_SLOTS].into_boxed_slice(),
            era: 1,
        }
    }

    /// Every A64 instruction is four bytes, so the low two bits of a guest PC
    /// carry nothing.
    #[inline]
    fn index(pc: u64) -> usize {
        ((pc >> 2) as usize) & (SUBSET_SLOTS - 1)
    }

    #[inline]
    fn get(&self, pc: u64, key: u64) -> Option<bool> {
        let slot = &self.slots[Subset::index(pc)];
        (slot.era == self.era && slot.pc == pc && slot.key == key).then_some(slot.inside)
    }

    #[inline]
    fn put(&mut self, pc: u64, key: u64, inside: bool) {
        let era = self.era;
        self.slots[Subset::index(pc)] = Slot {
            pc,
            key,
            inside,
            era,
        };
    }

    fn clear(&mut self) {
        self.era += 1;
    }
}

/// What names a block besides its guest PC: the world it was lifted in.
///
/// The **physical page the entry fetch resolved to**, never
/// `SysRegs::translation_gen`. The generation is bumped by every `TLBI` and by
/// every write to `TTBR0_EL1`, `TTBR1_EL1`, `TCR_EL1` and `SCTLR_EL1`, so a
/// Linux guest bumps it on every `switch_mm` and every unmap; a cache keyed on
/// it would miss every time and be slower than the interpreter it replaced,
/// which is exactly what the RISC-V engine measured before it was keyed
/// differently. The physical page is strictly *more* precise rather than less:
/// it distinguishes exactly what decides the block's meaning and nothing else.
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
#[derive(Debug, Clone, Copy)]
struct Admitted {
    world: World,
    key: u64,
    /// The virtual page the entry PC is on, and the physical page it resolved
    /// to. A block never leaves that page, so one translation covers every
    /// byte the lifter may read.
    page: u64,
    base: u64,
    /// Whether the **entry translation itself** raised an interrupt, so this
    /// block must leave at its first guest instruction boundary.
    ///
    /// [`admit`]'s two halves are asked in this order and cannot be swapped:
    /// the interrupt question decides whether a block runs at all, and the
    /// translation is what names it. So there is a window between them, and a
    /// translation-table walk lives in it — see [`admit`]'s own docs.
    leave: bool,
}

/// Whether a block may run at `pc`, and what it costs to find out.
#[derive(Debug)]
enum Admit {
    /// It may.
    Ready(Admitted),
    /// It may not, and the reason is one the interpreter answers: a pending
    /// interrupt, a stalled `WFI`, a misaligned PC, or an instruction outside
    /// the lifted subset.
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
/// **The interrupt check first.** A pending interrupt and a stalled `WFI` are
/// both the interpreter's, and `Exec::step` is how each is taken. Asking first
/// is not an optimization: `step` takes the trap, and a block run instead
/// would take it up to sixty-four instructions late. Asking it *per block*
/// rather than per run is what keeps a chained run indistinguishable from a
/// sequence of one-block ones.
///
/// **Then the entry fetch translation**, charged exactly as the interpreter's
/// own fetch charges it, and performed on every execution rather than at lift
/// time, because a cached block must cost what an uncached one cost. It is
/// also what *names* the block, which is why the subset lookup is after it.
///
/// # There is no budget guard here any more
///
/// There used to be a third thing: a block was refused unless its worst case
/// fitted what was left of the quantum, because a block that overran would
/// stop the guest on a different instruction from the one an interpreted core
/// stops on — and `State::debt`, the carried overrun, is in the snapshot a
/// machine's state hash is taken over. Everything that computed that worst
/// case is gone with it: `WALK_ACCESSES`, `per_access`, `worst_bound`,
/// `block_bound`, the tick column of [`Subset`], and `Probe` — the guard's own
/// lifter, which existed only because a guard that *guesses* a cold PC's worst
/// case declines the whole tail of every quantum and never learns better.
///
/// [`IrHost::spent`] replaced all of it with a comparison of two fields.
/// [`Host::allowance`] is what [`advance`] was given and `Exec::used` is what
/// this call has charged, so a block leaves at the first instruction boundary
/// where the second reaches the first — which is the boundary the interpreter
/// would have stopped on, because `Cpu::run_budget` runs an instruction
/// exactly while `used < allowance`. The stopping point is now equal by
/// construction rather than by a bound being conservative enough.
///
/// # The window between the two, and [`Admitted::leave`]
///
/// The order above means the interrupt inputs are looked at and *then* the
/// entry translation runs, so anything the translation does to those inputs
/// happens after the last look. A translation that hits the TLB does nothing
/// at all — but a **miss walks the translation tables**, and that walk can
/// move the answer in two separate ways.
///
/// * **Its ticks.** They are charged to this core's own cycle counter, which
///   is what the generic timer is counted off, so three or four of them can
///   cross a comparator the check a moment earlier found un-crossed. That is
///   the defect this file found first, and
///   `the_generic_timer_is_taken_at_the_same_instruction_across_a_tlbi` is it:
///   only a cold *instruction-fetch* translation charges them, and `mmu::Tlb`
///   keeps fetch, load and store entries in three separate sets, so a `TLBI`
///   or a guest executing from more pages than the fetch set holds are the
///   only ways in (`docs/testing/long-run.md`).
/// * **Its reads.** A descriptor read is an ordinary physical access that the
///   address space answers however the board decided, so a translation table
///   over a lazily-advanced device is a device read — the same thing
///   `cpu::riscv::engine`'s and `cpu::x86::engine`'s `IrHost::load` had to
///   start asking about. Neither `arm.gic` nor anything else on `arm64-virt`
///   accepts the `Width::U64` a descriptor read carries, so this half is
///   unreachable *on those boards*; nothing about the core or the address
///   space enforces it, `TTBR0_EL1` is a guest-written register with no range
///   check, and the other two cores' boards are not so lucky.
///
/// Both are one question — *"is an interrupt pending that was not pending a
/// moment ago"* — and the cost of asking it is one comparison of `Exec::used`
/// against itself, because **only a walk charges**: on a hit, with the MMU
/// off, and on every warm block the count is unchanged and
/// `Exec::pending_interrupt` is never asked a second time. That gate is also
/// what keeps a `true` answer from being a throughput cliff, and it is
/// stricter than the one `leave_at` used: a comparator stays crossed until the
/// guest re-arms it, so *"an interrupt is pending"* on its own describes long
/// stretches of ordinary code, while *"a walk just happened and now one is"*
/// describes the boundary the interpreter would have stopped at.
///
/// [`Host::hand_back`] is what a `true` answer does, and it leaves the run at
/// the boundary **after one retired instruction** — because `ir::Interp` never
/// asks [`IrHost::spent`] at a block's first boundary and `jit::dispatch` never
/// asks at a run's first block. That is exactly `Exec::step_once`: charge the
/// fetch, run the instruction, take the interrupt on the next call.
///
/// # The one exit that is still not covered
///
/// [`Admitted::leave`] answers for a block that runs. It cannot answer for the
/// third way out of this function: a **known-unliftable PC on a cold page**,
/// where the walk happens, raises, and then `subset.get` sends the instruction
/// to `Exec::step_once` — whose own first act is to take the pending
/// interrupt, so the instruction never runs and `ELR_EL1` names it rather than
/// its successor. An interpreted core would have run it: its step looked at
/// the wire *before* its fetch charged the walk. Reachable by the timer here,
/// not only by a device.
///
/// It is the same window, and closing it needs something this file does not
/// have: a way to ask `Exec` for one step with the interrupt check already
/// discharged.
fn admit(cfg: &Config, subset: &mut Subset, exec: &mut Exec<'_>, pc: u64) -> Admit {
    if exec.pending_interrupt().is_some() || exec.st.wfi {
        return Admit::Interpret;
    }
    let translating = exec.st.sys.mmu_enabled();
    let strict_align = exec.st.sys.sctlr & sctlr::A != 0;
    let charged = exec.used;
    let phys = match exec.translate_fetch(pc) {
        Ok(phys) => phys,
        Err(trap) => return Admit::Trap(trap),
    };
    // A walk, and only a walk, can have moved the answer above.
    let leave = exec.used != charged && exec.pending_interrupt().is_some();
    let world = World {
        features: cfg.features,
        origin: key_origin(translating, phys),
        strict_align,
    };
    let key = lift::key(&world, SHAPE, SMC);

    // Known unliftable: the interpreter takes this instruction, and reaching
    // it without a dispatcher round trip and a lift that fails at its first
    // instruction is the whole point of remembering.
    if subset.get(pc, key) == Some(false) {
        return Admit::Interpret;
    }

    Admit::Ready(Admitted {
        world,
        key,
        page: pc & !PAGE_MASK,
        base: phys & !PAGE_MASK,
        leave,
    })
}

// ---------------------------------------------------------------------------
// One step of the run loop
// ---------------------------------------------------------------------------

/// Execute a chain of blocks, or — where a block would be wrong — nothing, and
/// leave the instruction to the interpreter.
///
/// Reports the bus accesses charged and the [`Exit`] the step produced, in the
/// same currency and with the same meaning as `Cpu::step_to_exit`, so a run
/// loop cannot tell which engine it is driving.
///
/// `remaining` is what is left of the caller's budget, and it becomes
/// [`Host::allowance`]: a block runs until an instruction boundary at which
/// this call has charged that many ticks and then **leaves**, through
/// [`IrHost::spent`], with the guest standing on the instruction an
/// interpreted core would have stopped on. That holds for every block of a
/// chain rather than only the first — `jit::dispatch` asks at each block
/// boundary too.
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
    tlb: &mut Tlb,
    space: &Arc<AddressSpace>,
    cfg: &Config,
    lines: &Lines,
    exits: ExitMask,
    monitor: Option<&MonitorSlot>,
    remaining: u64,
) -> (u64, Option<Exit>) {
    let Jit {
        disp,
        subset,
        retired,
        interpreted,
        smc,
    } = jit;
    let mut exec = Exec::new(state, tlb, space, cfg, lines, exits, monitor);
    let pc = exec.st.pc;

    // The entry work for the *first* block, done here rather than through
    // `Frontend::enter`, because the overwhelmingly common answer on a real
    // guest is "not a block at all" — an `MSR`, an `SVC`, a floating-point
    // instruction — and reaching the interpreter for one should not cost a
    // frontend, a host and a dispatcher round trip.
    let at = match admit(cfg, subset, &mut exec, pc) {
        Admit::Ready(at) => at,
        Admit::Interpret => return interpret(interpreted, smc, disp, subset, exec),
        // The instruction at `pc` has not started, so its own PC is both where
        // the trap is taken and where it resumes — the same pair
        // `Exec::step_once` would produce for a fetch abort. The walk this
        // translation just charged is *not* re-charged: the interpreter is not
        // asked to fetch again, because it would walk again.
        Admit::Trap(trap) => return deliver(smc, disp, subset, exec, trap, pc, pc),
    };

    let mut front = Lifter {
        cfg,
        at,
        space,
        attrs: MemAttrs::DEBUG.with_requester(cfg.requester),
        subset,
        admitted: true,
        entry_trap: None,
        refused: false,
        rejected: None,
    };

    let mut host = Host::new(&mut exec, pc, remaining, front.at.base);
    if front.at.leave {
        // The entry translation walked, and the walk raised the interrupt.
        // One instruction retires and the run hands the boundary back, which
        // is what `Exec::step_once` does: it charges the fetch, runs the
        // instruction, and takes the interrupt on its next call.
        host.hand_back();
    }
    let run = match disp.run(&mut front, &mut host, pc, CHAIN) {
        Ok(run) => run,
        // This frontend refuses no world, so this is unreachable; degrade
        // rather than fail the machine if it ever is not (`ROADMAP.md` §9).
        Err(_) => {
            drop(host);
            let Lifter { subset, .. } = front;
            return interpret(interpreted, smc, disp, subset, exec);
        }
    };
    let Host {
        slots, trap, mark, ..
    } = host;
    let Lifter {
        subset,
        entry_trap,
        rejected,
        ..
    } = front;
    debug_assert!(
        rejected.is_none(),
        "the AArch64 frontend emitted a block the verifier rejects: {rejected:?}"
    );

    if run.blocks == 0 {
        // Nothing executed: the instruction at `pc` is outside the lifted
        // subset, and `Frontend::translate` has just recorded that so the next
        // pass skips straight to here. The interpreter takes it, and its own
        // fetch translation now hits the TLB the translation above filled — so
        // what it charges is what a purely interpreted core would.
        return interpret(interpreted, smc, disp, subset, exec);
    }

    *retired = retired.wrapping_add(run.insns as u64);

    // Every retired instruction, back into the architectural state.
    exec.st.x.copy_from_slice(&slots[..31]);
    exec.st.sys.set_sp(slots[SP.0 as usize]);
    exec.st.sys.nzcv = Nzcv::new(
        slots[lift::N.0 as usize] & 1 != 0,
        slots[lift::Z.0 as usize] & 1 != 0,
        slots[lift::C.0 as usize] & 1 != 0,
        slots[lift::V.0 as usize] & 1 != 0,
    );

    match run.stop {
        Stop::Fault(fault) => {
            // The block stopped *at* the faulting instruction with the
            // architectural state that instruction should see, which is what
            // `differential`'s fault path asserts and why nothing is
            // reconstructed here. The trap is the one the memory path raised,
            // carrying the syndrome and the faulting address the interpreter
            // would carry.
            let trap = trap.unwrap_or_else(|| Trap::data_abort_at(fault.pc));
            let next = mark.map_or(fault.pc, |m| m.1);
            deliver(smc, disp, subset, exec, trap, fault.pc, next)
        }
        Stop::Unsupported { op, at } => panic!(
            "the AArch64 frontend emitted {op} at index {at}, which the IR backend cannot execute"
        ),
        // A chained boundary whose entry fetch faulted. The instruction at
        // `run.pc` has not started, so its own PC is both where the trap is
        // taken and where it resumes.
        Stop::Declined if entry_trap.is_some() => {
            let trap = entry_trap.expect("just tested");
            deliver(smc, disp, subset, exec, trap, run.pc, run.pc)
        }
        // A chained boundary the frontend declined, and a block whose lift
        // produced nothing: both mean the instruction at `run.pc` is one the
        // **interpreter** has to take, and it is taken here rather than after
        // a return to the run loop.
        //
        // # Why it cannot wait for the next call
        //
        // `Frontend::enter` has already charged that instruction's entry
        // translation — the walk, on a TLB miss, which is three or four
        // accesses on a real guest — through [`admit`]. Returning here leaves
        // those ticks charged with *nothing of that instruction executed*, and
        // `Cpu::run_budget` then tests `used < allowance` in exactly that
        // window. The interpreter has no such window: it charges the walk and
        // the fetch inside one `Exec::step`, so its budget test stands in
        // front of both. A quantum whose last few ticks land there therefore
        // stops the two engines on **different instructions** — the
        // interpreter runs the instruction, the translated core stops in front
        // of it — and `State::debt`, which the machine's state hash covers,
        // parts with them. It self-corrects at the next quantum, so a small
        // run never sees it; over an arm64 Linux boot it is the whole
        // divergence, because the quantum this lands on eventually is one that
        // ends with an interrupt pending and the two cores take it at
        // different PCs.
        //
        // Interpreting here puts the walk and the instruction it was for on
        // the same side of that test, which is where the interpreter has them.
        // It costs nothing else: this is the same `interpret` the prologue
        // reaches for an entry PC outside the subset, one dispatcher round
        // trip earlier than before.
        Stop::Declined | Stop::Untranslatable { .. } => {
            exec.st.pc = run.pc;
            interpret(interpreted, smc, disp, subset, exec)
        }
        // `Budget` ends a full chain and `Spent` one that left part-way
        // through a block because the caller's tick allowance ran out — both
        // leave the guest at `run.pc` for the run loop to pick up. **`Spent`
        // needs no arm of its own, and that is the point**: a block that
        // leaves at a boundary has published the architectural state of that
        // boundary through its live mapping, so there is nothing to
        // reconstruct. `Exit` cannot happen: no safe-point flag is given to
        // the dispatcher, because the run loop above checks it between calls.
        _ => {
            exec.st.pc = run.pc;
            // Once per call, whatever the call did — the same rule
            // `Exec::step` follows. A chain that ran only blocks charges the
            // ticks the generic timer's comparator is reached by, and the wire
            // out has to follow within the same call or an idle kernel waits
            // for an interrupt its own timer already raised.
            //
            // **This line used to be untested and is not any more**, and
            // the reason is the seam. A mutation pass removed it and
            // everything still passed, because every quantum's tail was
            // interpreted — the old guard declined once the remaining budget
            // fell below a block's bound — and `Exec::step` published on the
            // way past. With the guard gone a quantum can end *inside* a
            // block, so a run whose whole quantum is blocks reaches this line
            // or reaches nothing;
            // `the_timer_levels_are_published_when_a_quantum_ends_in_a_block`
            // is the assertion.
            exec.publish_timer_levels();
            let used = exec.used;
            drain(smc, disp, subset, &mut exec);
            (used.max(1), None)
        }
    }
}

/// Interpret one instruction, and tell the block cache what it wrote.
fn interpret(
    interpreted: &mut u64,
    smc: &mut u64,
    disp: &mut Dispatcher,
    subset: &mut Subset,
    mut exec: Exec<'_>,
) -> (u64, Option<Exit>) {
    *interpreted = interpreted.wrapping_add(1);
    let used = exec.step();
    let exit = exec.take_exit();
    drain(smc, disp, subset, &mut exec);
    (used, exit)
}

/// Take a trap the block or an entry fetch raised, exactly as `Exec::step`
/// takes one: out of the core when the mask says so, into the guest's vector
/// table otherwise.
fn deliver(
    smc: &mut u64,
    disp: &mut Dispatcher,
    subset: &mut Subset,
    mut exec: Exec<'_>,
    trap: Trap,
    at: u64,
    next: u64,
) -> (u64, Option<Exit>) {
    let out = exec.take_trap(trap, at, next);
    // The generic timer's comparator can be reached by the accesses the block
    // charged, and the wire out has to follow within the same call or an idle
    // kernel waits for an interrupt its own timer already raised.
    exec.publish_timer_levels();
    let used = exec.used;
    drain(smc, disp, subset, &mut exec);
    (used.max(1), out)
}

/// Hand what an interpreted instruction wrote to the block cache.
fn drain(smc: &mut u64, disp: &mut Dispatcher, subset: &mut Subset, exec: &mut Exec<'_>) {
    let mut hit = 0usize;
    for i in 0..exec.wrote_n as usize {
        hit += disp.cache_mut().note_write(exec.wrote[i], 1);
    }
    exec.wrote_n = 0;
    *smc = smc.wrapping_add(hit as u64);
    if hit > 0 {
        subset.clear();
    }
}

// ---------------------------------------------------------------------------
// The frontend
// ---------------------------------------------------------------------------

/// The AArch64 half of the dispatcher's contract, over a real core.
struct Lifter<'a> {
    cfg: &'a Config,
    /// What the current block's entry resolved to — replaced at every boundary
    /// by [`Lifter::enter`], because a chained successor is on its own page,
    /// under its own key.
    at: Admitted,
    space: &'a AddressSpace,
    /// Lifting reads *ahead* of the guest: up to sixty-four instructions it
    /// has not asked for. A fetch is an ordinary access and a read-ahead is
    /// not, so this is the one place in the core that reads guest memory the
    /// way a debugger does — CLAUDE.md's "a debugger read must not pop a FIFO"
    /// is exactly the hazard. Nothing about the *translation* is relaxed: that
    /// happens in [`admit`], through the fetch path, with its walk and its
    /// permission check.
    ///
    /// What a debug read does relax is the **mapping's** `Perms::EXEC`, and
    /// deliberately — `MemAttrs::read_perm` will not ask for it when
    /// `MemAttrs::debug` is set, because a monitor disassembling a page is a
    /// debugger doing its job. So [`Lifter::lift`] asks for it separately
    /// rather than through these attributes.
    attrs: MemAttrs,
    subset: &'a mut Subset,
    /// Whether [`advance`]'s prologue has already admitted the entry PC, so
    /// the dispatcher's first `enter` neither translates nor charges twice.
    admitted: bool,
    /// A trap raised by a *chained* boundary's entry fetch. The prologue's own
    /// trap never lands here — it is delivered before a dispatcher exists.
    entry_trap: Option<Trap>,
    /// Whether the last lift stopped because the **bus** refused a fetch of
    /// those bytes rather than because the instruction is outside the subset.
    ///
    /// The two are the same outcome — the interpreter takes it — and a
    /// different fact, which is why [`Subset`] is not told about this one.
    /// That table's only cleaner is a store into a lifted page, so an entry
    /// written here would outlive the retopology that granted `Perms::EXEC`
    /// and pin the PC to the interpreter for the rest of the run. Never a
    /// wrong answer, since the interpreter is the oracle; just a permanent one.
    refused: bool,
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
            // generation here would ask for a full flush on every `TLBI`.
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
        // Whatever this boundary resolves to, the block that runs after it is
        // a different block on a different page, so the code page the store
        // guard compares against is replaced rather than kept. Set before
        // `admit` can decline, because a declined boundary interprets and an
        // interpreted store is drained by `drain` rather than by this host.
        host.code_page = u64::MAX;
        host.topology = host.exec.topology();
        // Registers live in the host's slots between the blocks of a chain and
        // are written back only when the run ends. Nothing `admit` reads is one
        // of them — it reads the system registers, the core's TLB and the tick
        // counter — so a chained boundary sees the same world a fresh
        // `advance` would have seen.
        let entry = match admit(self.cfg, self.subset, host.exec, pc) {
            Admit::Ready(at) => {
                host.code_page = at.base;
                if at.leave {
                    // The same window as the prologue's, at a chained
                    // boundary: `Dispatcher::run` asked [`IrHost::spent`]
                    // before this call and the walk below it raised the
                    // interrupt, so the answer has changed since. `timer_edge`
                    // does not need the same treatment — it is an absolute
                    // cycle count, so a walk that pushes the counter past it
                    // is caught by the very next boundary's comparison.
                    host.hand_back();
                }
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
        let lifted = self.lift(pc)?;
        if self.rejected.is_none()
            && let Err(e) = verify(&lifted.block)
        {
            self.rejected = Some(alloc::format!("{e}"));
        }
        // False when nothing could be lifted, which is what sends the next
        // pass straight to the interpreter instead of back through here. A
        // lift the bus refused outright is *not* recorded — see
        // [`Lifter::refused`].
        if !(self.refused && lifted.insns == 0) {
            self.subset.put(pc, self.at.key, lifted.insns > 0);
        }
        Ok(Translation {
            page: self.at.base,
            insns: lifted.insns,
            block: lifted.block,
        })
    }
}

impl Lifter<'_> {
    /// Lift at `pc`, reading only what the entry translation covers **and what
    /// the bus would let this core fetch**.
    ///
    /// The second half is the one that is not obvious. `admit` has already run
    /// the *MMU* fetch path, so the guest's own page tables have had their say;
    /// what it has not consulted is the mapping's [`Perms::EXEC`], because that
    /// is enforced against a read carrying `AccessPurpose::FETCH` and the reads
    /// below deliberately carry [`MemAttrs::DEBUG`] instead — a lift must not
    /// pop a FIFO. So the permission is asked for separately, by
    /// [`jit::executable_run`](crate::jit::executable_run), and the bytes still
    /// come out with no side effects.
    ///
    /// Per *word* rather than per page, because a flat entry is not a page: a
    /// 4 KiB page can hold an executable mapping and a non-executable one, and
    /// the interpreter would fetch its way to the boundary and abort there. A
    /// lift that stops at the same word leaves the interpreter standing on the
    /// same instruction, which is what makes the two engines report the same
    /// abort at the same PC. The probe is memoised over the run the entry
    /// covers, so the ordinary whole-page-one-mapping case pays for one.
    ///
    /// **Nothing here is on the block-entry path.** This runs once per
    /// translation; a cached block re-enters through `admit` and touches none
    /// of it. That is sound because a permission change is a retopology, and
    /// `Frontend::epoch` already drops every block lifted before it.
    fn lift(&mut self, pc: u64) -> Result<lift::Lifted> {
        let space = self.space;
        let attrs = self.attrs;
        let base = self.at.base;
        let page = self.at.page;
        // Empty: `run.0 > run.1`, so nothing is inside it and the first
        // word probes.
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
            let end = phys.wrapping_add(4);
            if phys < run.0 || end > run.1 {
                match crate::jit::executable_run(space, phys) {
                    // The whole word has to be inside one permitting run. A
                    // word straddling two of them is refused rather than
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
            space.read(phys, Width::U32, attrs).ok().map(|v| v as u32)
        };
        let out = lift::lift(&self.at.world, pc, &mut src, lift::MAX_INSNS, SHAPE, SMC);
        self.refused = refused;
        out
    }
}

// ---------------------------------------------------------------------------
// The host
// ---------------------------------------------------------------------------

/// The guest state a block reads and writes, over the interpreter's own memory
/// path.
struct Host<'a, 'e> {
    exec: &'a mut Exec<'e>,
    /// The ticks [`advance`] was given, against which `Exec::used` is compared
    /// at every guest instruction boundary — see [`IrHost::spent`].
    ///
    /// The whole of the budget mechanism, and it is two fields because that is
    /// what the seam costs: it is read once per boundary on the hot path, so
    /// anything computed here would be felt.
    allowance: u64,
    /// The cycle count at which this core's own generic timer reaches its
    /// comparator — `Exec::timer_edge`, asked once per run because a block
    /// cannot write the registers it is computed from.
    ///
    /// The second half of [`IrHost::spent`], and it is here for the same
    /// reason `allowance` is: a field read at every guest instruction
    /// boundary, so the work that produced it happens once.
    ///
    /// It answers *"when do the timer's outputs next change"*, which is not
    /// *"when must this run leave"* — the two part for a comparator that is
    /// **already** crossed, where the first is [`u64::MAX`] and the second is
    /// "at the first boundary". That case belongs to [`admit`] rather than to
    /// this field: nothing but the entry translation can have crossed the
    /// comparator since `admit` last looked, and [`Admitted::leave`] is what
    /// `admit` says so with.
    timer_edge: u64,
    slots: [u64; lift::SLOT_COUNT as usize],
    /// The trap the memory path raised, kept because [`IrHost::load`] can only
    /// report a [`BusError`] and an A64 trap is a syndrome, a faulting address
    /// and a return policy.
    trap: Option<Trap>,
    /// The last guest instruction boundary the block announced, as
    /// `(pc, next_pc)`.
    mark: Option<(u64, u64)>,
    dirty: DirtyPages,
    /// The guest-**physical** page the running block's instructions came from,
    /// replaced at every block boundary by [`Frontend::enter`].
    ///
    /// The whole of [`Smc::HostGuard`] on this side: [`Host::note_writes`]
    /// compares every store's physical page against it, and a match retires
    /// the allowance so the block leaves at the boundary the store's own
    /// instruction ends at — which is exactly where `Smc::EndBlock` used to
    /// put a block boundary, reached only when a store really did land in the
    /// code.
    code_page: u64,
    /// The address space's topology generation as of this block's entry.
    ///
    /// A backend takes the inlined memory path's host pointers out of the
    /// shadow TLB **once per block** and they are valid until the TLB is
    /// flushed, which used to be impossible inside a block because a store
    /// ended one. A store to a device that remaps is the way in, so
    /// [`IrHost::store`] compares this against the live generation and leaves
    /// the block when it moved. Nothing else can move it: a `TLBI` and every
    /// write to `TTBR0_EL1`, `TTBR1_EL1`, `TCR_EL1` and `SCTLR_EL1` are
    /// outside the lifted subset, and an inlined store reaches plain RAM only.
    topology: u64,
}

impl<'a, 'e> Host<'a, 'e> {
    /// Retire what is left of this run's `allowance`, so the block leaves at
    /// its next guest instruction boundary.
    ///
    /// How an interrupt raised by the **entry translation** reaches
    /// [`IrHost::spent`], and it costs that function nothing: a boundary
    /// already compares `Exec::used` against `allowance`, and zero is the
    /// value that comparison is always true for. The RISC-V and x86 engines
    /// carry the identical two lines for the identical window; this core is
    /// the one that found it, from the other side.
    ///
    /// The ticks are not lost. [`advance`] reports `Exec::used` and
    /// `Cpu::run_budget` loops until *its* allowance is spent, so what this
    /// gives up is the rest of the **run**, not the rest of the quantum —
    /// which is right, because the rest of the quantum belongs to the
    /// interpreter and to the interrupt it is about to take.
    ///
    /// [`Cpu::run_budget`]: super::Cpu::run_budget
    #[inline]
    fn hand_back(&mut self) {
        self.allowance = 0;
    }

    fn new(exec: &'a mut Exec<'e>, pc: u64, allowance: u64, code_page: u64) -> Host<'a, 'e> {
        let mut slots = [0u64; lift::SLOT_COUNT as usize];
        slots[..31].copy_from_slice(&exec.st.x);
        slots[SP.0 as usize] = exec.st.sys.sp();
        let flags = exec.st.sys.nzcv;
        slots[lift::N.0 as usize] = u64::from(flags.n());
        slots[lift::Z.0 as usize] = u64::from(flags.z());
        slots[lift::C.0 as usize] = u64::from(flags.c());
        slots[lift::V.0 as usize] = u64::from(flags.v());
        slots[PC.0 as usize] = pc;
        Host {
            timer_edge: exec.timer_edge(),
            topology: exec.topology(),
            exec,
            allowance,
            slots,
            trap: None,
            mark: None,
            dirty: DirtyPages::new(),
            code_page,
        }
    }

    /// Report a trap as the bus error the IR speaks, keeping the syndrome.
    fn fault(&mut self, trap: Trap) -> BusError {
        self.trap = Some(trap);
        BusError::BadAccess
    }

    /// Move whatever the last access wrote into the dirty log, and leave the
    /// block if any of it landed in the block's own code.
    ///
    /// Whatever landed, landed: a split store that faulted on its second page
    /// still wrote the first, and a translation of those bytes is stale either
    /// way.
    ///
    /// The one place [`Smc::HostGuard`] is implemented, and the reason it is
    /// one place: an inlined store reaches it through
    /// [`FastMem::note_fast_store`] and every other store through
    /// [`IrHost::store`], so the compiled path and the interpreted one cannot
    /// disagree about what a store into the code page does. Measured over
    /// twenty guest seconds of an arm64 Linux boot, the comparison cost
    /// **55 971 311 host instructions** — 414 078 569 to 470 049 880 for this
    /// function — against 7.15 **G** the policy saved.
    fn note_writes(&mut self) {
        for i in 0..self.exec.wrote_n as usize {
            let phys = self.exec.wrote[i];
            self.dirty.note(phys, 1);
            // [`Smc::HostGuard`], and the whole of it. A store into the page
            // this block's own instructions came from means every instruction
            // after it is a translation of bytes that no longer exist, so the
            // block leaves at its next guest instruction boundary — where
            // `Dispatcher::run` drains the log above into the block cache and
            // the next pass lifts the bytes the store left.
            //
            // Physical at both ends: `Exec::wrote` records the address the bus
            // transaction reached and `Admitted::base` is what the entry
            // translation resolved to, so a store through a second mapping of
            // the code page is caught. That is the case `cpu::x86::lift`'s
            // linear guard cannot see, and it is why this core does not need
            // to fall back to `Smc::EndBlock` under translation.
            if phys & !PAGE_MASK == self.code_page {
                self.hand_back();
            }
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

    /// Perform a store, and answer the two questions a store used to answer
    /// by ending the block.
    ///
    /// Both are asked here rather than at every guest instruction boundary
    /// because this is where the answer can change, and [`Host::hand_back`]
    /// carries it to the boundary for free. Neither can arrive through
    /// [`FastMem::note_fast_store`]: a plan covers plain little-endian RAM
    /// over a whole page, so an inlined store reaches no device and remaps
    /// nothing.
    ///
    /// * **An interrupt this store raised.** A write to the GIC, or to a
    ///   device that answers by pulling a wire, brings a line up between two
    ///   instructions of a lifted block — where `Exec::step` would take it at
    ///   the next one. [`admit`] cannot see it, because it runs at a block
    ///   boundary and under [`Smc::HostGuard`] there no longer is one after a
    ///   store. `cpu::x86::engine` pays this on exactly the same argument, and
    ///   said so first.
    /// * **A store that remapped the address space**, which retires the host
    ///   pointers the backend took out of the shadow TLB at block entry. See
    ///   [`Host::topology`].
    ///
    /// The cost is only on the path a plan does not cover — a device access, a
    /// misaligned store, a page whose shadow entry has been evicted — and it
    /// is not visible against what [`admit`] asks the first of these anyway:
    /// across this change `Exec::pending_interrupt` went **down**, from
    /// 1 188 799 920 host instructions to 806 436 078 over twenty guest
    /// seconds of the boot, because there are 40% fewer blocks to admit.
    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        let done = self.exec.store(addr, mem.size.bytes(), value);
        self.note_writes();
        if self.exec.pending_interrupt().is_some() || self.exec.topology() != self.topology {
            self.hand_back();
        }
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

    /// Whether this call's tick allowance is gone, **or this core's own timer
    /// has reached its comparator**.
    ///
    /// The second condition is not a budget and is not an optimisation: it is
    /// what keeps the two engines on one instruction. The interpreter samples
    /// its generic timer once per instruction — `Exec::step` publishes the
    /// outward level and `Exec::pending_interrupt` reads the internal one — so
    /// a comparator reached by an instruction's own accesses is acted on at
    /// the next instruction boundary. A chain that ran on to its natural end
    /// would take the same interrupt tens of ticks later, at a different
    /// `ELR_EL1`; over an arm64 Linux boot that is the divergence, and it
    /// arrives the first time a timer fires while the guest is inside lifted
    /// code rather than parked in `WFI`. Leaving here hands the boundary back
    /// to [`advance`], which publishes, and to `admit`, which takes the
    /// interrupt through the interpreter exactly as an interpreted core does.
    ///
    /// `Exec::timer_edge` states why one comparison against a number computed
    /// at the start of the run is the whole test.
    ///
    /// Monotone because `Exec::used` only grows within one [`advance`], which
    /// is what [`IrHost::spent`] requires and cannot check. Equal to
    /// `Cpu::run_budget`'s own condition by construction: that loop calls
    /// [`advance`] with `allowance - used` and steps while `used < allowance`,
    /// so `Exec::used >= allowance` here is exactly the point at which an
    /// interpreted core would have stopped.
    ///
    /// **This core owes the frontend obligation nothing**, which is the other
    /// half of the seam's contract and the half `cpu::x86::lift` had to pay
    /// for. Every boundary's live mapping must be architecturally *complete*,
    /// not merely monotone; `lift::Lifter::live_regs` names every general
    /// register, `SP` and every flag that a temporary currently shadows, and
    /// nothing in this frontend elides a slot on the grounds that the
    /// instruction about to run overwrites it. So a boundary here is always
    /// resumable and no lifter change was needed.
    #[inline]
    fn spent(&self) -> bool {
        self.exec.used >= self.allowance || self.exec.st.cycles >= self.timer_edge
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
/// this core's translation-table walk and the ticks it spends. That looks like
/// the argument for publishing nothing: a walk cannot be skipped per access,
/// so a plan whose validity is decided per access is not a plan.
///
/// **The plan is not per access. It is per page, and it is decided by the
/// table the walk already filled.** `mmu::Tlb` has a shadow
/// (`mmu::Tlb::attach_shadow`) that `Exec::translate` writes in the same
/// breath as its own entry, at the same index, for the same virtual page. So a
/// shadow entry exists only where this core's TLB *also* holds the
/// translation, and an inlined access that hits one is an access whose walk
/// had already been performed and charged for — which is why the whole cost it
/// still owes is the one tick [`FastMem::note_fast_load`] charges.
///
/// Two things the compiled path cannot do are therefore done once, at fill
/// time, rather than never:
///
/// * the **walk**, by this core, on the miss that filled the entry — and the
///   entry dies with the core's own, because both are written by the same
///   eviction and both carry `SysRegs::translation_gen`, which every `TLBI`
///   and every write to `TTBR0_EL1`, `TTBR1_EL1`, `TCR_EL1` and `SCTLR_EL1`
///   bumps, the ASID among them because it lives in `TTBR0_EL1[63:48]`;
/// * the **fault**, which cannot arise: an entry exists only over plain
///   little-endian RAM covering its whole page, with the permissions the slow
///   path checks and no constraint left to apply (`jit::Tlb::fill`). AArch64
///   has no PMP, so unlike RISC-V there is no second permission scheme left
///   over for a page to be refused on.
///
/// Everything a plan does not cover still calls [`IrHost::load`] and gets this
/// core's answer — a fetch, a misaligned access, an access to a device, an
/// unprivileged `LDTR`, a page whose translation has been evicted, and every
/// access at all on a build without a shadow.
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
    use crate::core::space::{MonitorSlot, Perms, RamStore, Region};
    use crate::cpu::arm::a64::mmu::desc;
    use crate::cpu::arm::a64::{Cpu, Engine};

    /// How much RAM a test core gets, at address zero.
    const RAM: u64 = 0x4_0000;

    /// Where the exception vectors live in a test that installs one.
    const VBAR: u64 = 0x800;

    /// The offset of the "current EL with `SP_ELx`, IRQ" vector.
    ///
    /// **`0x280`, not `0x200`.** The first four slots of the table are the
    /// `SP_EL0` ones, and a core that has selected `SP_ELx` — which is the
    /// reset state — uses the second four. A handler written at `0x200` is
    /// the *synchronous* one, so an IRQ lands on a zeroed word, takes an
    /// `UNDEFINED` exception, and reaches the handler by the wrong door with
    /// `ELR_EL1` naming the vector rather than the interrupted instruction.
    const IRQ_VECTOR: u64 = 0x280;

    /// A counting loop whose scratch word is on a **different page** from its
    /// code, so the store does not invalidate the block it is in and the cache
    /// can actually warm.
    ///
    /// Every instruction is inside the lifted subset and the back edge is a
    /// direct `B`, so a trace merges it.
    ///
    /// It sets the **flags** and moves the **stack pointer** as well as the
    /// register file, and it reads both back **in a later block** — the store
    /// ends the block, so the `CSEL` and the `MOV Xd, SP` after it resolve
    /// their operands through `Opcode::GET_SLOT` rather than through a
    /// temporary. That last part is what a mutation pass demanded: dropping
    /// the flags from `advance`'s write-back survived a loop that merely
    /// *set* them, because the interpreted tail of each quantum recomputed
    /// them before anything looked. A value that is only ever written is a
    /// value no test can miss being wrong.
    const LOOP: [u32; 10] = [
        0xd282_0007, // movz x7, #0x1000     ; the scratch page
        0xd280_0025, // movz x5, #1
        0x9100_04a5, // add  x5, x5, #1      ; the loop starts here
        0xf100_0cbf, // cmp  x5, #3          ; writes all four flags
        0x9100_43ff, // add  sp, sp, #16
        0xf900_00e5, // str  x5, [x7]        ; ends the block
        0x9a87_00a9, // csel x9, x5, x7, eq  ; reads the flags out of the slot
        0x9100_03ea, // mov  x10, sp         ; reads the stack pointer likewise
        0xf940_00e6, // ldr  x6, [x7]
        0x17ff_fff9, // b    .-28
    ];

    /// The same loop with its scratch word **inside** the code page, which is
    /// the self-modifying-code fixture: every block invalidates itself.
    const NEAR_LOOP: [u32; 10] = [
        0xd280_0807, // movz x7, #0x40
        0xd280_0025, // movz x5, #1
        0x9100_04a5, // add  x5, x5, #1
        0xf100_0cbf, // cmp  x5, #3
        0x9100_43ff, // add  sp, sp, #16
        0xf900_00e5, // str  x5, [x7]
        0x9a87_00a9, // csel x9, x5, x7, eq
        0x9100_03ea, // mov  x10, sp
        0xf940_00e6, // ldr  x6, [x7]
        0x17ff_fff9, // b    .-28
    ];

    /// A core with `program` at zero and [`RAM`] bytes of RAM under it.
    fn core(engine: Engine, program: &[u32]) -> Cpu {
        core_mapped(engine, program, Perms::RWX)
    }

    /// [`core`], with the RAM mapped on `perms` rather than on everything.
    fn core_mapped(engine: Engine, program: &[u32], perms: Perms) -> Cpu {
        let ram = Arc::new(RamStore::new(RAM));
        write_words(&ram, 0, program);
        let space = AddressSpace::new("mem", 64);
        space
            .topology()
            .map_with_perms(Region::ram("ram", ram), 0, perms)
            .expect("nothing else is mapped");
        let cpu = Cpu::new(Config::cortex_a53().with_reset_vector(0)).with_engine(engine);
        cpu.attach_space(Arc::new(space));
        cpu
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

    /// Run both cores on the same budgets and compare everything a guest, a
    /// snapshot or a state hash can see.
    fn agree_on(engine: Engine, program: &[u32], budget: u64, quanta: usize) -> (Cpu, Cpu) {
        agree_built(engine, |e| core(e, program), budget, quanta)
    }

    /// [`agree_on`], over a fixture that builds its own address space.
    ///
    /// The permission cases need two mappings with different [`Perms`] rather
    /// than one flat RAM, and every column [`agree_on`] compares is exactly the
    /// set they care about — an instruction abort is `ESR_EL1`, `ELR_EL1`,
    /// `FAR_EL1` and a PC.
    fn agree_built(
        engine: Engine,
        build: impl Fn(Engine) -> Cpu,
        budget: u64,
        quanta: usize,
    ) -> (Cpu, Cpu) {
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
        for n in 0..31 {
            assert_eq!(
                interp.x(n),
                jit.x(n),
                "x{n} under {engine:?}: the interpreter says {:#018x}, the JIT says {:#018x}",
                interp.x(n),
                jit.x(n),
            );
        }
        assert_eq!(interp.pc(), jit.pc(), "the program counter");
        assert_eq!(interp.sp(), jit.sp(), "the stack pointer");
        assert_eq!(
            interp.sysregs().nzcv,
            jit.sysregs().nzcv,
            "PSTATE.NZCV. A compiled block must set the flags an interpreted \
             one sets, and A64 spells `CMP` as `SUBS`"
        );
        assert_eq!(
            interp.cycles(),
            jit.cycles(),
            "the cycle counter. A compiled block must charge exactly what an \
             interpreted one charges (ROADMAP.md §0)"
        );
        assert_eq!(
            interp.cycle_debt(),
            jit.cycle_debt(),
            "the carried overrun. A block that ran past the budget where an \
             instruction would not have puts the two engines on different \
             instructions for the rest of the run"
        );
        assert_eq!(interp.sysregs().esr_el1, jit.sysregs().esr_el1, "ESR_EL1");
        assert_eq!(interp.sysregs().elr_el1, jit.sysregs().elr_el1, "ELR_EL1");
        assert_eq!(interp.sysregs().far_el1, jit.sysregs().far_el1, "FAR_EL1");
        (interp, jit)
    }

    /// Every case runs against both JIT engines: the host code generator is a
    /// third implementation of the same block, and the claim is about all three.
    fn agree(program: &[u32], budget: u64, quanta: usize) -> (Cpu, Cpu) {
        let out = agree_on(Engine::Jit, program, budget, quanta);
        agree_on(Engine::JitHost, program, budget, quanta);
        out
    }

    /// Two instructions on an executable page, ending in a branch into the
    /// *next* page — which is mapped without [`Perms::EXEC`].
    ///
    /// The branch leaves the page, so the block ends and the refusal lands at a
    /// **chained** boundary, through `Frontend::enter` and
    /// `Frontend::translate` rather than through [`advance`]'s prologue.
    const INTO_NX: [u32; 2] = [
        0xd282_0007, // movz x7, #0x1000
        0x1400_03ff, // b    .+0xffc          ; from 0x4 to 0x1000
    ];

    /// What sits on the page that may not be fetched.
    ///
    /// Every word is inside the lifted subset and none of it traps, so a lifter
    /// that never asks about execute permission produces a real block out of it
    /// and runs it — which is exactly the divergence.
    const NX_PAGE: [u32; 3] = [
        0x9100_04a5, // add x5, x5, #1
        0x9100_04a5, // add x5, x5, #1
        0x1400_0000, // b   .
    ];

    /// A core whose first page may be fetched and whose second may not.
    ///
    /// Two stores and two mappings rather than one store and two permissions,
    /// because permission is a property of the mapping — and this is the shape
    /// a board with a data-only aperture beside its ROM actually has.
    fn split_core(engine: Engine) -> Cpu {
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
        let cpu = Cpu::new(Config::cortex_a53().with_reset_vector(0)).with_engine(engine);
        cpu.attach_space(Arc::new(space));
        cpu
    }

    /// [`Perms::EXEC`] is enforced against a *translated* block, not only
    /// against an interpreted fetch.
    ///
    /// `MemAttrs::purpose` makes a fetch a fetch and `FlatLeaf::read` refuses
    /// one on a mapping without [`Perms::EXEC`] — but only the interpreter's
    /// fetch *is* that read. A lift reads ahead of the guest and therefore
    /// reads with `MemAttrs::DEBUG`, which by design never asks for `EXEC`, and
    /// [`admit`] consults the *MMU* rather than the mapping. Before
    /// `jit::executable_run` this fixture ran the whole loop under the JIT
    /// while the interpreter aborted on its first instruction.
    ///
    /// The reset vector itself is on the refusing mapping here, so the refusal
    /// is in [`advance`]'s prologue: the very first lift.
    #[test]
    fn a_mapping_without_exec_refuses_a_translated_block_too() {
        for engine in [Engine::Jit, Engine::JitHost] {
            let (interp, jit) = agree_built(engine, |e| core_mapped(e, &LOOP, Perms::RW), 4096, 4);
            assert_ne!(
                interp.sysregs().esr_el1,
                0,
                "the interpreter never aborted, so this compared two working \
                 cores under {engine:?}"
            );
            assert_eq!(
                interp.x(5),
                0,
                "the loop body ran on a mapping that refuses a fetch"
            );
            assert_eq!(
                jit.jit_stats().expect("a jit core").blocks,
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
    fn a_chain_into_a_non_executable_page_aborts_where_the_interpreter_does() {
        for engine in [Engine::Jit, Engine::JitHost] {
            let (interp, jit) = agree_built(engine, split_core, 4096, 4);
            assert_ne!(
                interp.sysregs().esr_el1,
                0,
                "the interpreter never aborted under {engine:?}"
            );
            assert_eq!(
                interp.x(5),
                0,
                "the non-executable page ran under the interpreter, so the \
                 fixture is not testing what it says it is"
            );
            let stats = jit.jit_stats().expect("a jit core");
            assert!(
                stats.blocks > 0,
                "the executable page never produced a block, so nothing was \
                 chained *from* and the boundary is untested: {stats:?}"
            );
        }
    }

    #[test]
    fn a_translated_core_and_an_interpreted_one_agree_on_every_column() {
        // The budget has to exceed a *cold* block's worst case or `admit`
        // declines every block and never learns the real one, which is the
        // shape of the guard rather than a defect: the tail of a quantum is
        // interpreted so that both engines stop on the same instruction.
        let (interp, jit) = agree(&LOOP, 4096, 8);
        assert!(interp.cycles() > 100, "the loop ran");
        let stats = jit.jit_stats().expect("a jit core");
        assert!(
            stats.blocks > 0,
            "no block ran, so this compared two interpreters"
        );
        assert!(
            stats.retired > stats.interpreted,
            "most instructions were interpreted, which is not a translated run: {stats:?}"
        );
        assert!(stats.chained > 0, "no exit was ever patched: {stats:?}");
        assert!(
            stats.retired > stats.blocks,
            "a trace retires more than one instruction per block, so a \
             `retired` that counted anything else would be below this: {stats:?}"
        );
        // The flags and the stack pointer both moved, which is what makes
        // `advance`'s write-back of them load-bearing rather than decorative.
        assert_ne!(interp.sp(), 0, "the loop moved the stack pointer");
        assert_ne!(interp.sysregs().nzcv.0, 0, "the loop set the flags");
    }

    #[test]
    fn a_block_that_writes_its_own_page_invalidates_its_translation() {
        // Both JIT engines, and separately: under `jit-host` the store is
        // **inlined**, so it reaches the block cache through
        // `Exec::note_fast_store` rather than through `IrHost::store`, and a
        // mutation pass found that path could stop reporting without anything
        // noticing while the portable one still did.
        for engine in [Engine::Jit, Engine::JitHost] {
            let (_, jit) = agree_on(engine, &NEAR_LOOP, 4096, 8);
            let stats = jit.jit_stats().expect("a jit core");
            // Proportional rather than merely non-zero: under `jit-host` all
            // but a handful of these stores are **inlined**, so a
            // `note_fast_store` that stopped reporting its page would still
            // leave the one or two that took the call — and `smc > 0` would
            // hold while the mechanism was dead. A mutation pass found exactly
            // that.
            assert!(
                stats.smc * 2 > stats.blocks,
                "a store into the block's own page invalidated almost no \
                 translation under {engine:?}: {stats:?}"
            );
        }
        let (_, jit) = agree(&NEAR_LOOP, 4096, 8);
        let stats = jit.jit_stats().expect("a jit core");
        // Correctness — the two engines agreeing — is what `agree` asserted.
        // This is the *mechanism*: without the store log reaching the block
        // cache the two would still agree here, because these bytes are data
        // rather than instructions, and the check would quietly test nothing.
        assert!(
            stats.smc > 0,
            "a store into the block's own page invalidated no translation: {stats:?}"
        );
        assert!(stats.translated > stats.smc / 2, "{stats:?}");
    }

    #[test]
    fn the_two_engines_agree_over_budgets_no_block_fits_in() {
        // Three ticks is less than any block of this program charges, so
        // every block leaves at its first boundary — and the run must still
        // consume exactly what an interpreted core consumes, and stop on the
        // same instruction.
        agree(&LOOP, 3, 40);
    }

    #[test]
    fn a_quantum_too_small_for_a_whole_block_still_retires_inside_one() {
        // The defect [`IrHost::spent`] closes, in one assertion, and the whole
        // reason this core adopted the seam.
        //
        // 2 594 ticks is half of what a cold block used to be *guessed* to
        // cost — sixty-four instructions of a pair access, split into bytes,
        // each byte walking four levels — and every quantum on
        // `machines/arm64-virt.machine` ends in a stretch below it. The old
        // guard declined every block there, the interpreter took one
        // instruction, and the PC after it was in the middle of a block and so
        // was uncosted too, so the tail never recovered: **`retired` here was
        // zero**, and over half of every real quantum was interpreted an
        // instruction at a time.
        //
        // Now the block runs and leaves at the boundary the allowance runs
        // out on, so a short quantum retires in blocks like a long one.
        let (interp, jit) = agree(&LOOP, 2_594, 40);
        assert!(interp.cycles() > 100, "the loop ran");
        let stats = jit.jit_stats().expect("a jit core");
        assert!(
            stats.retired > 8 * stats.interpreted,
            "a budget smaller than a whole block sent the run to the \
             interpreter: {stats:?}"
        );
    }

    /// An arithmetic loop that touches no memory at all.
    ///
    /// Every instruction charges exactly one tick — its own fetch — so a
    /// budget sweep over this program lands the seam's boundary on a different
    /// instruction at every budget, with no access cost to blur it. On a
    /// program *with* accesses the accesses dominate and most budgets land in
    /// the same place.
    const ALU_LOOP: [u32; 5] = [
        0x9100_04a5, // add  x5, x5, #1
        0x8b05_00c6, // add  x6, x6, x5
        0xca05_0108, // eor  x8, x8, x5
        0xcb06_00e7, // sub  x7, x7, x6
        0x17ff_fffc, // b    .-16
    ];

    #[test]
    fn a_block_that_only_computes_leaves_on_the_interpreters_instruction() {
        // Swept across every budget, because the failure is not "wrong answer"
        // but "stopped one instruction later than the interpreter would" —
        // which shows in `cycles` and `cycle_debt` and in nothing else. One
        // tick per instruction here, so every one of these budgets stops the
        // block at a different boundary.
        for budget in 1..=64u64 {
            agree_on(Engine::Jit, &ALU_LOOP, budget, 12);
        }
        // Above a trace's own bound — this loop unrolls to `MAX_INSNS`, so its
        // block charges 64 ticks and nothing else — so that whole blocks run
        // too.
        let (_, jit) = agree_on(Engine::Jit, &ALU_LOOP, 256, 40);
        let stats = jit.jit_stats().expect("a jit core");
        assert!(stats.retired > stats.interpreted, "no block ran: {stats:?}");
    }

    #[test]
    fn a_paged_core_reaches_the_interpreter_for_an_unliftable_pc_without_lifting() {
        // [`Subset`]'s negative answer, under a paged MMU — the world where
        // the table used to hold a *tick bound* and where the sentinel for
        // "nothing here" collided with what an empty block honestly bounds at
        // (four, the entry walk). A guard that recorded the one as the other
        // admitted the `mrs`, reached the dispatcher, lifted nothing and
        // interpreted anyway, every time round the loop, with every column
        // still agreeing; a mutation pass found exactly that. Recording a
        // boolean makes the collision unrepresentable.
        // Back to the `mrs` every iteration, so the sentinel is met again and
        // again rather than once — and `0xd538_0000`, a real `MRS`, because
        // an encoding that names no system register is UNDEFINED and would
        // send the core round a vector table instead of round this loop.
        let program = [
            0xd538_0000, // mrs x0, midr_el1   ; outside the subset
            0x9100_0421, // add x1, x1, #1
            0x9100_0442, // add x2, x2, #1
            0x17ff_fffd, // b   .-12
        ];
        let interp = core(Engine::Interp, &program);
        let jit = core(Engine::Jit, &program);
        enable_mmu(&interp);
        enable_mmu(&jit);
        for _ in 0..40 {
            assert_eq!(interp.run_budget(128), jit.run_budget(128));
        }
        for n in 0..31 {
            assert_eq!(interp.x(n), jit.x(n), "x{n}");
        }
        assert_eq!(interp.cycles(), jit.cycles(), "the cycle counter");
        let stats = jit.jit_stats().expect("a jit core");
        assert!(stats.interpreted > 8, "the `mrs` came round: {stats:?}");
        assert!(
            stats.blocks > 8,
            "no block ran, so nothing here is about the sentinel: {stats:?}"
        );
        assert!(
            stats.translated <= 8,
            "the `mrs` cost a fresh translation every time it came round: \
             {stats:?}"
        );
    }

    /// Warm a core, snapshot it, rewrite an instruction under it, restore, and
    /// carry on — which is what a debugger does to publish a patch and what a
    /// rewind does every time.
    ///
    /// `ROADMAP.md` §4.5: derived state is never serialized and is always
    /// invalidated. A block cache is derived state by that definition, and a
    /// `load` replaces the memory a block was lifted from — so a core that
    /// kept its blocks runs instructions the snapshot does not contain, and
    /// nothing else notices, because `jit::dispatch`'s invalidation watches
    /// *guest stores* and a restore is not one. Measured before the fix: a
    /// patched instruction in a hot loop still ran stale 107 times out of
    /// 66 667 iterations.
    fn patch_across(engine: Engine, restore: bool) -> (Cpu, Cpu) {
        use super::super::CLASS;
        use crate::core::device::{Device, ResetKind};
        use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
        // Named rather than inherited: the test module compiles in a
        // `no_std` build too, where the prelude has no `Vec`.
        use alloc::vec::Vec;

        let interp = core(Engine::Interp, &LOOP);
        let jit = core(engine, &LOOP);
        // Warm: by now the loop is lifted, compiled and chained.
        for _ in 0..8 {
            assert_eq!(interp.run_budget(4096), jit.run_budget(4096));
        }
        assert!(
            jit.jit_stats().expect("a jit core").retired > 100,
            "the loop never warmed up under {engine:?}"
        );
        let mut saved: Vec<Vec<u8>> = Vec::new();
        for cpu in [&interp, &jit] {
            let mut shape = MachineShape::new();
            shape.add_device("cpu", CLASS.name).expect("one device");
            let mut w = StateWriter::new(shape);
            {
                let mut chunk = w.chunk("cpu", CLASS.name, CLASS.version).expect("a chunk");
                cpu.save(&mut chunk).expect("the core saves");
            }
            saved.push(w.to_vec().expect("a snapshot"));
            // `add x5, x5, #1` becomes `add x5, x5, #2`, in the page the warm
            // blocks were lifted from.
            let space = cpu.space().expect("the core has its space");
            space
                .write(8, Width::U32, 0x9100_08a5, MemAttrs::DEFAULT)
                .expect("in RAM");
        }
        for (bytes, cpu) in saved.iter().zip([&interp, &jit]) {
            if restore {
                let reader = StateReader::new(bytes).expect("a reader");
                let chunk = reader
                    .load("cpu", CLASS.name, CLASS.version, &Migrations::new())
                    .expect("the chunk");
                let mut cr = chunk.reader();
                cpu.load(&mut cr).expect("the core loads");
                cr.end().expect("the whole chunk");
            } else {
                cpu.reset(ResetKind::Warm);
            }
        }
        for _ in 0..8 {
            assert_eq!(
                interp.run_budget(4096),
                jit.run_budget(4096),
                "the two engines consumed different amounts after the patch"
            );
        }
        (interp, jit)
    }

    #[test]
    fn a_snapshot_restore_throws_the_block_cache_away() {
        for engine in [Engine::Jit, Engine::JitHost] {
            let (interp, jit) = patch_across(engine, true);
            for n in 0..31 {
                assert_eq!(
                    interp.x(n),
                    jit.x(n),
                    "x{n} under {engine:?}: a block lifted before the restore \
                     ran afterwards"
                );
            }
            assert_eq!(interp.cycles(), jit.cycles(), "the cycle counter");
            assert_ne!(interp.x(5), 0, "the patched loop ran at all");
        }
    }

    #[test]
    fn a_reset_throws_the_block_cache_away_too() {
        for engine in [Engine::Jit, Engine::JitHost] {
            let (interp, jit) = patch_across(engine, false);
            for n in 0..31 {
                assert_eq!(
                    interp.x(n),
                    jit.x(n),
                    "x{n} under {engine:?}: a block lifted before the reset \
                     ran afterwards"
                );
            }
            assert_eq!(interp.cycles(), jit.cycles(), "the cycle counter");
            assert_ne!(interp.x(5), 0, "the patched loop ran at all");
        }
    }

    #[test]
    fn the_timer_levels_are_published_when_a_quantum_ends_in_a_block() {
        // `advance`'s "once per call, whatever the call did" line, which was
        // unreachable while the budget guard existed: a quantum's tail was
        // always interpreted, and `Exec::step` publishes on the way past. The
        // seam makes a run that is nothing but blocks possible, so the line is
        // now the only thing that raises a timer interrupt such a run reaches.
        //
        // The comparator is set behind the counter, so the level is high the
        // moment any tick is charged; the loop touches no memory, so the whole
        // quantum is one chain of blocks and `Exec::step` is never called.
        let jit = core(Engine::Jit, &ALU_LOOP);
        // Routed out of the core, which is what a board with a GIC does and
        // what makes `publish_timer_levels` do anything at all.
        jit.lines.route_timer(Lines::TIMER_PHYS);
        let mut regs = jit.sysregs();
        // Enabled, unmasked, and a comparator the counter is already past.
        regs.cntp_ctl = 1;
        regs.cntp_cval = 1;
        jit.set_sysregs(regs);
        assert_eq!(jit.lines.timer_level(), 0, "nothing published yet");
        jit.run_budget(4096);
        let stats = jit.jit_stats().expect("a jit core");
        assert_eq!(
            stats.interpreted, 0,
            "the quantum reached the interpreter, so this proves nothing: \
             {stats:?}"
        );
        assert_eq!(
            jit.lines.timer_level(),
            Lines::TIMER_PHYS,
            "a run made entirely of blocks never published the timer level"
        );
    }

    #[test]
    fn clearing_the_subset_table_forgets_every_answer_at_once() {
        // The invalidation is a counter rather than a sweep, because a guest
        // store into a page a translation came from clears the whole table and
        // a Linux boot does that thousands of times. A counter that did not
        // advance would keep serving *negative* answers for instructions that
        // have been overwritten, which is exactly the answer an overwrite can
        // invalidate.
        let mut subset = Subset::new();
        subset.put(0x1000, 7, false);
        assert_eq!(subset.get(0x1000, 7), Some(false));
        assert_eq!(subset.get(0x1000, 8), None, "the key is part of the answer");
        subset.clear();
        assert_eq!(
            subset.get(0x1000, 7),
            None,
            "a cleared table answers nothing"
        );
        subset.put(0x1000, 7, true);
        assert_eq!(subset.get(0x1000, 7), Some(true), "and can be filled again");
    }

    #[test]
    fn an_instruction_outside_the_subset_is_interpreted_without_a_wasted_lift() {
        // `mrs x0, midr_el1` is outside the subset; [`Subset`] remembers that
        // so the next pass reaches the interpreter directly.
        //
        // **0xd538_0000, not 0xd530_0000**, which is what this test used to
        // say. The latter names no allocated system register, so it is
        // UNDEFINED rather than merely unlifted: it trapped, and with
        // `VBAR_EL1` at zero the core spent the whole run going round an
        // exception vector full of zero words. The two engines agreed about
        // that perfectly, and the test passed while exercising none of what it
        // names.
        let program = [0xd538_0000, 0xd280_0025, 0x17ff_fffe];
        let (_, jit) = agree(&program, 4096, 4);
        // *Without a wasted lift*, which is the half of the sentence no
        // comparison can assert: admitting a PC known to hold nothing still
        // reaches the right answer, through a dispatcher round trip and a
        // fresh `lift` that fails at its first instruction — every time round
        // the loop. A mutation pass that ignored the negative answer left this
        // test green, so what is asserted is the count rather than the
        // outcome.
        let stats = jit.jit_stats().expect("a jit core");
        assert!(
            stats.interpreted > 4,
            "the `mrs` was reached more than once: {stats:?}"
        );
        assert!(
            stats.translated <= 4,
            "the `mrs` cost a fresh translation every time it came round: \
             {stats:?}"
        );
    }

    #[test]
    fn the_two_engines_agree_at_every_budget_across_the_seam() {
        // Where a quantum ends, swept rather than sampled.
        //
        // [`IrHost::spent`] puts that boundary at a *guest instruction*, so it
        // is a different instruction for every budget and there is no cliff to
        // sample near. The assertion that matters is therefore not "some small
        // budget works" but "every budget works" — and the columns `agree_on`
        // compares include `cycle_debt`, the carried overrun, which is exactly
        // what a block that left one instruction late would move.
        for budget in 1..=96u64 {
            agree_on(Engine::Jit, &LOOP, budget, 12);
        }
        // The host code generator over the same sweep, sampled coarsely
        // because it compiles every distinct block and the cliff is the same
        // one.
        for budget in (1..=96u64).step_by(3) {
            agree_on(Engine::JitHost, &LOOP, budget, 12);
        }
    }

    #[test]
    fn a_paged_core_agrees_at_every_budget_across_the_seam_too() {
        // The paged world charges a four-level walk in front of every fetch
        // and every access, so the same budget buys a fifth of the
        // instructions — a different boundary at every budget again, and it is
        // the world a real guest runs in.
        for budget in (1..=256u64).step_by(5) {
            let interp = core(Engine::Interp, &LOOP);
            let jit = core(Engine::Jit, &LOOP);
            enable_mmu(&interp);
            enable_mmu(&jit);
            for n in 0..12 {
                assert_eq!(
                    interp.run_budget(budget),
                    jit.run_budget(budget),
                    "budget {budget}, quantum {n}: the two engines consumed \
                     different amounts"
                );
            }
            assert_eq!(
                interp.pc(),
                jit.pc(),
                "budget {budget}: the program counter"
            );
            assert_eq!(
                interp.cycles(),
                jit.cycles(),
                "budget {budget}: the cycle counter"
            );
            assert_eq!(
                interp.cycle_debt(),
                jit.cycle_debt(),
                "budget {budget}: the carried overrun"
            );
            for n in 0..31 {
                assert_eq!(interp.x(n), jit.x(n), "budget {budget}: x{n}");
            }
        }
    }

    /// Turn a core's MMU on over a three-level hierarchy that identity-maps
    /// the first two mebibytes as a block.
    ///
    /// Bare mode makes `Exec::translate_fetch` the identity, so **every**
    /// paging rule this engine has is untested without this: the walk it
    /// charges at each block boundary, `key_origin` putting a physical page in
    /// the cache key, and the trap a failed entry fetch delivers. A mutation
    /// pass found all three survived a bare-mode-only suite.
    fn enable_mmu(cpu: &Cpu) {
        const L1: u64 = 0x2_0000;
        const L2: u64 = 0x2_1000;
        let space = cpu.space().expect("the core has its space");
        let put = |addr: u64, value: u64| {
            space
                .write(addr, Width::U64, value, MemAttrs::DEFAULT)
                .expect("inside RAM");
        };
        const L3: u64 = 0x2_2000;
        put(L1, L2 | desc::VALID | desc::TABLE);
        // A 2 MiB identity block at level 2, which covers all of this RAM.
        put(L2, desc::VALID | desc::AF);
        // Level-2 entry 1: a table, so that virtual `RO_PAGE` can be a
        // **read-only** 4 KiB page over physical 0x3_0000. `AP[2]` — bit 7 —
        // is what makes it read-only at both levels.
        put(L2 + 8, L3 | desc::VALID | desc::TABLE);
        put(
            L3,
            0x3_0000 | desc::VALID | desc::TABLE | desc::AF | (2 << desc::AP_SHIFT),
        );
        // Level-3 entry 1: virtual `MOVED_PAGE`, writable, over physical
        // 0x3_1000 — the page `a_remapping_after_a_tlbi_is_seen_by_a_compiled_load`
        // moves out from under a cached translation.
        put(L3 + 8, 0x3_1000 | desc::VALID | desc::TABLE | desc::AF);
        let mut sys = cpu.sysregs();
        sys.ttbr0 = L1;
        // T0SZ = T1SZ = 25 (39-bit halves), TG1 = 0b10 (the 4 KiB granule).
        sys.tcr = 25 | (25 << 16) | (0b10 << 30);
        sys.sctlr |= sctlr::M;
        cpu.set_sysregs(sys);
    }

    /// A block sets the flags and the stack pointer and then the core stalls,
    /// so what a snapshot sees is what the **write-back** left rather than
    /// what an interpreted tail recomputed.
    ///
    /// The loops above cannot make this claim and a mutation pass proved it:
    /// dropping `PSTATE.NZCV` from `advance`'s write-back survived both of
    /// them, because the tail of every quantum is interpreted and a loop runs
    /// its `CMP` again before anything looks. Here nothing runs again — `WFI`
    /// stalls the core and every later step charges a tick and changes
    /// nothing — so the last write of each column is the one under test.
    const SETTLE: [u32; 4] = [
        0xd280_0025, // movz x5, #1
        0xf100_0cbf, // cmp  x5, #3          ; N set, Z C V clear
        0x9100_43ff, // add  sp, sp, #16
        0xd503_207f, // wfi
    ];

    #[test]
    fn a_block_publishes_every_column_it_changed_before_the_core_settles() {
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = core(Engine::Interp, &SETTLE);
            let jit = core(engine, &SETTLE);
            for _ in 0..4 {
                interp.run_budget(4096);
                jit.run_budget(4096);
            }
            assert!(interp.is_waiting(), "the core reached the `wfi`");
            assert_eq!(
                interp.sysregs().nzcv,
                jit.sysregs().nzcv,
                "PSTATE.NZCV under {engine:?}: a block set the flags and \
                 nothing ran again to set them a second time"
            );
            assert_ne!(interp.sysregs().nzcv.0, 0, "the compare set something");
            assert_eq!(interp.sp(), jit.sp(), "sp under {engine:?}");
            assert_eq!(interp.sp(), 16, "the stack pointer moved once");
            assert_eq!(interp.cycles(), jit.cycles(), "cycles under {engine:?}");
            assert!(
                jit.jit_stats().expect("a jit core").blocks > 0,
                "no block ran under {engine:?}"
            );
        }
    }

    #[test]
    fn a_store_the_interpreter_made_invalidates_a_translation_too() {
        // `STLR` is outside the lifted subset, so this store is executed by
        // the interpreter and reported through `Exec::wrote` rather than
        // through `StoreLog` — a **different** drain, on a path the block
        // path never takes. A mutation pass found that switching it off left
        // every test passing, because nothing else in the tree ever made an
        // interpreted store into a page a translation had come from.
        let program = [
            0xd280_0807, // movz x7, #0x40      ; inside this code page
            0xd280_0025, // movz x5, #1
            0x9100_04a5, // add  x5, x5, #1     ; the loop starts here
            0xc89f_fce5, // stlr x5, [x7]       ; interpreted, into our own page
            0xf940_00e6, // ldr  x6, [x7]
            0x17ff_fffd, // b    .-12
        ];
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = core(Engine::Interp, &program);
            let jit = core(engine, &program);
            for n in 0..8 {
                assert_eq!(
                    interp.run_budget(4096),
                    jit.run_budget(4096),
                    "quantum {n} under {engine:?}"
                );
            }
            for n in 0..31 {
                assert_eq!(interp.x(n), jit.x(n), "x{n} under {engine:?}");
            }
            assert_eq!(interp.cycles(), jit.cycles(), "cycles under {engine:?}");
            let stats = jit.jit_stats().expect("a jit core");
            assert!(stats.blocks > 0, "no block ran under {engine:?}");
            assert!(
                stats.interpreted > 0,
                "the `stlr` was never interpreted under {engine:?}"
            );
            assert!(
                stats.smc_interpreted > 0,
                "the interpreted store invalidated no translation under \
                 {engine:?}: {stats:?}"
            );
            assert_eq!(
                stats.smc, 0,
                "no block in this program stores, so the other drain must be \
                 idle: {stats:?}"
            );
        }
    }

    #[test]
    fn an_unaligned_pc_aborts_the_same_way_in_both_engines() {
        // The entry fetch is the *fetch* path, alignment check included: a
        // translated core that resolved a PC without asking would lift a block
        // from a misaligned address instead of raising. `Exec::fetch` raises
        // `PC_ALIGN` before it translates anything, and so does
        // `Exec::translate_fetch`.
        // The bytes at offset 2 spell a `NOP`, deliberately: a translated core
        // that skipped the check would **lift and run** one rather than
        // raising, and a program whose misaligned bytes happened not to decode
        // would hide that behind an ordinary `Stop::Unsupported`. A mutation
        // pass found exactly that hiding place.
        let program = [0x201f_0000u32, 0x0000_d503, 0x1400_0000];
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = core(Engine::Interp, &program);
            let jit = core(engine, &program);
            for cpu in [&interp, &jit] {
                let mut sys = cpu.sysregs();
                sys.vbar_el1 = VBAR;
                cpu.set_sysregs(sys);
                cpu.set_pc(2);
            }
            for n in 0..2 {
                assert_eq!(
                    interp.run_budget(4096),
                    jit.run_budget(4096),
                    "quantum {n} under {engine:?}"
                );
            }
            assert_eq!(
                interp.sysregs().esr_el1,
                jit.sysregs().esr_el1,
                "ESR_EL1 under {engine:?}"
            );
            assert_eq!(interp.sysregs().far_el1, 2, "the faulting address");
            assert_eq!(interp.sysregs().far_el1, jit.sysregs().far_el1);
            assert_eq!(interp.pc(), jit.pc(), "the pc under {engine:?}");
            assert_eq!(interp.cycles(), jit.cycles(), "cycles under {engine:?}");
        }
    }

    #[test]
    fn an_entry_fetch_that_faults_is_charged_exactly_once() {
        // The walk `admit` made is the one the trap is delivered from: asking
        // the interpreter to fetch again would walk again, and the two engines
        // would then disagree about the cycle counter by one walk. That is the
        // four ticks of drift the RISC-V engine records having found on a
        // cached corpus, and this is the case that would produce it.
        //
        // **One `advance` each, not a quantum.** `run_budget` stops as soon as
        // the budget is spent, so it equalises the tick total by construction
        // and a double charge disappears into where the run stops rather than
        // into the counter. `Cpu::run(1)` performs exactly one step and
        // reports what it cost, which is the number under test. A mutation
        // pass found this test could not fail until it was written this way.
        let program = [0x1410_0000u32]; // b .+4 MiB, out of the mapped block
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = core(Engine::Interp, &program);
            let jit = core(engine, &program);
            enable_mmu(&interp);
            enable_mmu(&jit);
            for cpu in [&interp, &jit] {
                let mut sys = cpu.sysregs();
                sys.vbar_el1 = VBAR;
                cpu.set_sysregs(sys);
                cpu.set_pc(0x40_0000);
            }
            let want = interp.run(1);
            let got = jit.run(1);
            assert_eq!(
                want, got,
                "one faulting entry fetch cost {want} ticks interpreted and \
                 {got} translated, under {engine:?}"
            );
            assert!(want > 1, "the walk was charged at all: {want}");
            assert_eq!(interp.cycles(), jit.cycles(), "cycles under {engine:?}");
            assert_ne!(interp.sysregs().esr_el1, 0, "the fetch faulted");
            assert_eq!(
                interp.sysregs().esr_el1,
                jit.sysregs().esr_el1,
                "ESR_EL1 under {engine:?}"
            );
            assert_eq!(interp.sysregs().far_el1, jit.sysregs().far_el1, "FAR_EL1");
            assert_eq!(interp.pc(), jit.pc(), "the pc under {engine:?}");
        }
    }

    /// Whether this build has a host code generator, which is the only thing
    /// that inlines an access.
    const HOST_BACKEND: bool = cfg!(any(
        all(
            feature = "jit-x86",
            target_os = "linux",
            target_arch = "x86_64"
        ),
        all(
            feature = "jit-arm64",
            target_os = "linux",
            target_arch = "aarch64"
        )
    ));

    /// The virtual page `enable_mmu` maps read-only.
    const RO_PAGE: u64 = 0x20_0000;

    /// The virtual page `enable_mmu` maps writably, and that a test remaps.
    const MOVED_PAGE: u64 = 0x20_1000;

    #[test]
    fn a_compiled_store_to_a_read_only_page_faults() {
        // The **load** set and the **store** set are not interchangeable, and
        // this is the case that says so: a load fills the load set for this
        // page, and a store served through that entry would write a page the
        // walk refuses. A store entry exists only because a walk *for a store*
        // succeeded, which is what checked `AP[2]`.
        //
        // Reached from a block, not from the interpreter: the load and the
        // store are both in the lifted subset, and the store is compiled and
        // would be inlined if its plan were the wrong one.
        let program = [
            0xd2a0_0407, // movz x7, #0x20, lsl #16   ; the read-only page
            0xf940_00e5, // ldr  x5, [x7]             ; fills the load set
            0xf900_00e5, // str  x5, [x7]             ; must fault
            0x1400_0000, // b    .
        ];
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = core(Engine::Interp, &program);
            let jit = core(engine, &program);
            enable_mmu(&interp);
            enable_mmu(&jit);
            for cpu in [&interp, &jit] {
                let mut sys = cpu.sysregs();
                sys.vbar_el1 = VBAR;
                cpu.set_sysregs(sys);
            }
            for n in 0..3 {
                assert_eq!(
                    interp.run_budget(8192),
                    jit.run_budget(8192),
                    "quantum {n} under {engine:?}"
                );
            }
            assert_ne!(interp.sysregs().esr_el1, 0, "the store faulted");
            assert_eq!(
                interp.sysregs().far_el1,
                RO_PAGE,
                "the faulting address is the read-only page"
            );
            assert_eq!(
                interp.sysregs().esr_el1,
                jit.sysregs().esr_el1,
                "ESR_EL1 under {engine:?}: a compiled store went through a \
                 page the walk refuses"
            );
            assert_eq!(interp.sysregs().far_el1, jit.sysregs().far_el1, "FAR_EL1");
            assert_eq!(interp.cycles(), jit.cycles(), "cycles under {engine:?}");
            // and the page really is untouched in both.
            for cpu in [&interp, &jit] {
                let space = cpu.space().expect("the core has its space");
                assert_eq!(
                    space
                        .read(0x3_0000, Width::U64, MemAttrs::DEFAULT)
                        .expect("mapped"),
                    0,
                    "a refused store wrote the page anyway, under {engine:?}"
                );
            }
        }
    }

    #[test]
    fn a_remapping_after_a_tlbi_is_seen_by_a_compiled_load() {
        // The shadow's stamp is `SysRegs::translation_gen`, which is what a
        // `TLBI` and every write to `TTBR0_EL1`, `TTBR1_EL1`, `TCR_EL1` and
        // `SCTLR_EL1` bump — and it is the *only* thing standing between a
        // compiled load and a page the guest has since moved. A mutation pass
        // found that a fill which stamped zero instead survived everything
        // else in this file, because nothing else ever remapped a page under a
        // cached translation.
        let program = [
            0xd282_0007, // movz x7, #0x1000
            0xf2a0_0407, // movk x7, #0x20, lsl #16   ; x7 = MOVED_PAGE
            0xf940_00e5, // ldr  x5, [x7]
            0x17ff_ffff, // b    .-4                  ; so the load repeats
        ];
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = core(Engine::Interp, &program);
            let jit = core(engine, &program);
            for cpu in [&interp, &jit] {
                enable_mmu(cpu);
                let space = cpu.space().expect("the core has its space");
                space
                    .write(0x3_1000, Width::U64, 0x1111, MemAttrs::DEFAULT)
                    .expect("inside RAM");
                space
                    .write(0x3_2000, Width::U64, 0x2222, MemAttrs::DEFAULT)
                    .expect("inside RAM");
            }
            for _ in 0..4 {
                interp.run_budget(8192);
                jit.run_budget(8192);
            }
            assert_eq!(interp.x(7), MOVED_PAGE, "the program addressed the page");
            assert_eq!(interp.x(5), 0x1111, "the first mapping was read");
            assert_eq!(interp.x(5), jit.x(5), "x5 under {engine:?}");

            // Move the page and invalidate, exactly as a `TLBI` does: the
            // generation is the whole of the invalidation here.
            for cpu in [&interp, &jit] {
                let space = cpu.space().expect("the core has its space");
                space
                    .write(
                        0x2_2000 + 8,
                        Width::U64,
                        0x3_2000 | desc::VALID | desc::TABLE | desc::AF,
                        MemAttrs::DEFAULT,
                    )
                    .expect("inside RAM");
                let mut sys = cpu.sysregs();
                sys.translation_gen = sys.translation_gen.wrapping_add(1);
                cpu.set_sysregs(sys);
            }
            let before = jit.jit_stats().expect("a jit core").fast_loads;
            for _ in 0..4 {
                interp.run_budget(8192);
                jit.run_budget(8192);
            }
            assert_eq!(interp.x(5), 0x2222, "the interpreter saw the new mapping");
            assert_eq!(
                interp.x(5),
                jit.x(5),
                "x5 under {engine:?}: a compiled load read a page the guest \
                 had already moved out from under it"
            );
            assert_eq!(interp.cycles(), jit.cycles(), "cycles under {engine:?}");
            // And the inlined path is live on *both* sides of the bump, which
            // is what makes the comparison above about the stamp rather than
            // about a shadow that quietly stopped working: a fill that stamped
            // the wrong generation would be invisible to the plan, every
            // access would take the call, and the answers would still agree.
            let stats = jit.jit_stats().expect("a jit core");
            if engine == Engine::JitHost && HOST_BACKEND {
                assert!(before > 0, "nothing was inlined before the remap");
                assert!(
                    stats.fast_loads > before,
                    "nothing was inlined after the remap: {stats:?}"
                );
            }
        }
    }

    #[test]
    fn a_compiled_store_breaks_an_exclusive_reservation() {
        // The exclusives are outside the lifted subset, so a `LDXR`/`STXR`
        // pair is interpreted — but a **compiled** store between them still
        // has to break the reservation, and an inlined one does not go through
        // `Exec::store`, which is where that normally happens. A mutation pass
        // found that dropping it from `Exec::note_fast_store` survived
        // everything else here.
        let program = [
            0xd282_0007, // movz x7, #0x1000        ; the scratch page
            0xc85f_7ce5, // ldxr x5, [x7]           ; interpreted; takes it
            0x9100_04c6, // add  x6, x6, #1
            0xf900_00e6, // str  x6, [x7]           ; compiled, and inlined
            0xc808_7ce6, // stxr w8, x6, [x7]       ; interpreted; must fail
            0x17ff_fffc, // b    .-16                ; round again, so the
                         //                            store is inlined from
                         //                            the second pass on
        ];
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = core(Engine::Interp, &program);
            let jit = core(engine, &program);
            for _ in 0..4 {
                interp.run_budget(4096);
                jit.run_budget(4096);
            }
            assert_eq!(
                interp.x(8),
                1,
                "the store-exclusive failed, because the store between the \
                 pair broke the reservation"
            );
            assert_eq!(
                interp.x(8),
                jit.x(8),
                "x8 under {engine:?}: a compiled store left the reservation \
                 standing, so a `STXR` that must fail succeeded"
            );
            assert_eq!(interp.cycles(), jit.cycles(), "cycles under {engine:?}");
            // The store has to be the *inlined* one, or this tests
            // `Exec::store`'s reservation rule rather than
            // `Exec::note_fast_store`'s — which is the one with no other
            // coverage.
            let stats = jit.jit_stats().expect("a jit core");
            if engine == Engine::JitHost && HOST_BACKEND {
                assert!(
                    stats.fast_stores > 0,
                    "the store between the pair was never inlined: {stats:?}"
                );
            }
        }
    }

    /// `note_fast_store` in isolation: the store has already happened through
    /// a host pointer, and everything the guest can still observe about it is
    /// owed from here.
    ///
    /// Driven directly rather than through a run, because a run cannot be made
    /// *pure*: `run_budget` always interprets a handful of instructions at its
    /// entry, one of them is the loop's store, and an interpreted store reaches
    /// the monitor through `SpaceView::write_span` — which would let a missing
    /// hook here pass unnoticed. One call, no run, no ambiguity.
    #[test]
    fn note_fast_store_tells_the_global_monitor_itself() {
        let cpu = core(Engine::JitHost, &LOOP);
        let space = cpu.space().expect("the core has a space");
        let cfg = cpu.config();
        let lines = Lines::default();
        let mut state = State::new(&cfg);
        let mut tlb = Tlb::new();
        tlb.attach_shadow(Arc::clone(&space));

        // One ordinary store first, so the software TLB and the shadow beside
        // it hold an entry for the page: an inlined store is only ever issued
        // against an entry generated code has already resolved.
        let mut exec = Exec::new(
            &mut state,
            &mut tlb,
            &space,
            &cfg,
            &lines,
            ExitMask::NONE,
            None,
        );
        exec.store(0x1000, 8, 0).expect("the page is RAM");

        let sibling = MonitorSlot::new(Arc::clone(&space), 4).expect("a free slot");
        sibling.reserve(0x1000);
        assert!(sibling.holds(), "the reservation was taken");
        exec.note_fast_store(0x1000, 8);
        assert!(
            !sibling.holds(),
            "an inlined store did not reach the global monitor, so a sibling's \
             `STXR` would succeed against a word this core had overwritten"
        );
    }

    #[test]
    fn a_compiled_store_breaks_a_siblings_reservation_too() {
        // The other half, and the one SMP needs. The test above proves a
        // compiled store breaks the *storing* core's own reservation; this one
        // proves it reaches the **global** monitor, where a sibling's
        // reservation lives. An inlined store never calls
        // `SpaceView::write_span`, so `Exec::note_fast_store` has to tell the
        // monitor itself — and 99% of this core's stores are inlined.
        //
        // The sibling is a bare `MonitorSlot` rather than a second `Cpu`
        // because what is under test is the hook, not the interleaving: a
        // registration on the same space is exactly what a second core would
        // hold.
        for engine in [Engine::Jit, Engine::JitHost] {
            let cpu = core(engine, &LOOP);
            let space = cpu.space().expect("the core has a space");
            // Warm the loop first, so that by the time the reservation is
            // taken every store in the next budget is coming out of generated
            // code rather than out of the interpreter.
            for _ in 0..4 {
                cpu.run_budget(4096);
            }
            let before = cpu.jit_stats().expect("a jit core");

            let sibling = MonitorSlot::new(Arc::clone(&space), 4).expect("a free slot");
            // `LOOP` stores to 0x1000, and with the MMU off that is also the
            // physical address.
            sibling.reserve(0x1000);
            assert!(sibling.holds(), "the reservation was taken");
            cpu.run_budget(4096);

            let stats = cpu.jit_stats().expect("a jit core");
            if engine == Engine::JitHost && HOST_BACKEND {
                assert!(
                    stats.fast_stores > before.fast_stores,
                    "no store was inlined in the budget under test: {stats:?}"
                );
            }
            assert!(
                !sibling.holds(),
                "a store under {engine:?} left a sibling's reservation                  standing, so its `STXR` would succeed against a word this                  core had already overwritten"
            );
        }
    }

    #[test]
    fn a_compiled_access_is_served_from_the_inlined_probe() {
        // The claim `FastMem` makes, asserted rather than assumed: a compiled
        // load and a compiled store resolve through the shadow this core fills
        // in lockstep with its own TLB, without calling back. Only the host
        // code generator inlines anything, so the portable backend must have
        // served none — which is the other half of the claim, because a
        // non-zero count there would mean the number came from somewhere other
        // than generated code.
        for engine in [Engine::Jit, Engine::JitHost] {
            for paged in [false, true] {
                let interp = core(Engine::Interp, &LOOP);
                let jit = core(engine, &LOOP);
                if paged {
                    enable_mmu(&interp);
                    enable_mmu(&jit);
                }
                for n in 0..8 {
                    assert_eq!(
                        interp.run_budget(8192),
                        jit.run_budget(8192),
                        "quantum {n} under {engine:?}, paged {paged}"
                    );
                }
                assert_eq!(
                    interp.cycles(),
                    jit.cycles(),
                    "the cycle counter under {engine:?}, paged {paged}: an \
                     inlined access must charge exactly what the call it \
                     replaced charged"
                );
                for n in 0..31 {
                    assert_eq!(interp.x(n), jit.x(n), "x{n} under {engine:?}");
                }
                let stats = jit.jit_stats().expect("a jit core");
                if engine == Engine::JitHost && HOST_BACKEND {
                    assert!(
                        stats.fast_loads > 0,
                        "no compiled load was inlined, paged {paged}: {stats:?}"
                    );
                    assert!(
                        stats.fast_stores > 0,
                        "no compiled store was inlined, paged {paged}: {stats:?}"
                    );
                } else {
                    assert_eq!(
                        (stats.fast_loads, stats.fast_stores),
                        (0, 0),
                        "the portable backend inlines nothing: {stats:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn the_two_engines_agree_with_the_mmu_on() {
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = core(Engine::Interp, &LOOP);
            let jit = core(engine, &LOOP);
            enable_mmu(&interp);
            enable_mmu(&jit);
            // Bigger than a *paged* cold block's worst case, which is four
            // times a bare one's because every access can walk: a smaller
            // budget declines every block and the run is a pair of
            // interpreters agreeing with each other.
            for n in 0..8 {
                assert_eq!(
                    interp.run_budget(8192),
                    jit.run_budget(8192),
                    "quantum {n} under {engine:?}"
                );
            }
            for n in 0..31 {
                assert_eq!(interp.x(n), jit.x(n), "x{n} under {engine:?}");
            }
            assert_eq!(interp.pc(), jit.pc(), "the pc under {engine:?}");
            assert_eq!(interp.sp(), jit.sp(), "sp under {engine:?}");
            assert_eq!(
                interp.sysregs().nzcv,
                jit.sysregs().nzcv,
                "PSTATE.NZCV under {engine:?}"
            );
            assert_eq!(
                interp.cycles(),
                jit.cycles(),
                "the cycle counter under {engine:?}. A translated core owes the \
                 entry fetch's walk on every block execution, chained ones \
                 included"
            );
            let (hits, misses) = jit.tlb_stats();
            assert!(
                hits + misses > 0,
                "the MMU was never asked under {engine:?}"
            );
            let stats = jit.jit_stats().expect("a jit core");
            assert!(stats.blocks > 0, "no block ran under {engine:?}");
        }
    }

    /// A loop that leaves its own page for an instruction outside the subset,
    /// which is the shape a real guest reaches an `MRS` in: a `BL` into
    /// another function, and the first thing that function does is read
    /// `SP_EL0`.
    ///
    /// The page matters. `mmu::Tlb` is indexed by 4 KiB virtual page whatever
    /// the mapping's granule, so a branch to `PAGE_TWO` is a **TLB miss** and
    /// the entry translation of the instruction there costs a walk — which is
    /// the whole of what this fixture is for. A loop that stayed on one page
    /// charges nothing at that boundary and cannot show the defect.
    const OFF_PAGE: [u32; 3] = [
        0x9100_0421, // add x1, x1, #1
        0x9100_0442, // add x2, x2, #1
        0x1400_03fe, // b   0x1000            ; +0xff8, onto the next page
    ];

    /// Where [`OFF_PAGE`] branches to, and back from.
    const PAGE_TWO: u64 = 0x1000;

    /// What sits there: an `MRS` the frontend does not lift, then the way back.
    const PAGE_TWO_CODE: [u32; 2] = [
        0xd538_0000, // mrs x0, midr_el1      ; outside the subset
        0x17ff_fbff, // b   -0x1004           ; back to the top
    ];

    #[test]
    fn a_declined_chained_boundary_charges_its_walk_with_the_instruction_it_belongs_to() {
        // `Frontend::enter` charges the entry translation of a chained
        // successor **before** the frontend has said whether it will run one,
        // and for an instruction outside the lifted subset it never does. The
        // walk is charged, nothing executes, and `advance` used to return
        // there — putting `Cpu::run_budget`'s `used < allowance` test between
        // an instruction's translation and its fetch. The interpreter has no
        // such point: `Exec::step` charges the walk and the fetch together, so
        // its budget test stands in front of both.
        //
        // So for the budgets whose quantum ends in that window the two engines
        // stop on different instructions and carry a different `State::debt` —
        // both of which the machine's state hash covers. It self-corrects at
        // the next quantum, which is why a small run never sees it and why the
        // sweep below is over *every* budget rather than a chosen one: budget
        // 6 is the first that lands there, and on an arm64 Linux boot the same
        // window is met about twenty times in twenty-five seconds.
        for budget in 1..=64u64 {
            let interp = core(Engine::Interp, &OFF_PAGE);
            let jit = core(Engine::Jit, &OFF_PAGE);
            for cpu in [&interp, &jit] {
                let space = cpu.space().expect("the core has its space");
                for (n, word) in PAGE_TWO_CODE.iter().enumerate() {
                    space
                        .write(
                            PAGE_TWO + 4 * n as u64,
                            Width::U32,
                            u64::from(*word),
                            MemAttrs::DEFAULT,
                        )
                        .expect("inside RAM");
                }
                enable_mmu(cpu);
            }
            for n in 0..8 {
                assert_eq!(
                    (
                        interp.run_budget(budget),
                        interp.pc(),
                        interp.cycles(),
                        interp.cycle_debt()
                    ),
                    (
                        jit.run_budget(budget),
                        jit.pc(),
                        jit.cycles(),
                        jit.cycle_debt()
                    ),
                    "budget {budget}, quantum {n}: the two engines stopped in \
                     different places. A chained boundary the frontend declined \
                     charged that instruction's walk and then handed the run \
                     loop a budget test the interpreter takes on the other side \
                     of it"
                );
            }
        }
    }

    #[test]
    fn the_generic_timer_is_taken_at_the_same_instruction_by_both_engines() {
        // The interrupt below is raised by **this core's own tick counting**,
        // which is the case the test after this one cannot cover: a line
        // asserted from outside changes between quanta and both engines see it
        // at the same boundary by construction, while the generic timer's
        // comparator is reached *inside* a run. An interpreted core samples it
        // once per instruction; a chain that ran on to its natural end takes
        // the same interrupt tens of ticks later, at a different `ELR_EL1`,
        // and everything the guest does with that return address follows it.
        //
        // `ALU_LOOP` touches no memory, so a warm quantum is one long chain
        // with no store to end it — which is exactly the window in which a
        // real guest's timer fires while it is doing work rather than parked
        // in `WFI`. That is why an arm64 Linux boot agrees for twenty-three
        // seconds and then does not.
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = core(Engine::Interp, &ALU_LOOP);
            let jit = core(engine, &ALU_LOOP);
            for cpu in [&interp, &jit] {
                let mut sys = cpu.sysregs();
                sys.daif = 0;
                sys.vbar_el1 = VBAR;
                cpu.set_sysregs(sys);
                // The same handler `an_interrupt_is_taken_at_the_same_instruction`
                // installs, and for the same three reasons: mask the source,
                // then stop, so `ELR_EL1` records where the interrupt was
                // taken instead of converging on the handler's own address.
                let space = cpu.space().expect("the core has its space");
                for (n, word) in [0xd503_42dfu64, 0x1400_0000].iter().enumerate() {
                    space
                        .write(
                            VBAR + IRQ_VECTOR + 4 * n as u64,
                            Width::U32,
                            *word,
                            MemAttrs::DEFAULT,
                        )
                        .expect("inside RAM");
                }
            }
            // Warm both, so the translated core is running a chain rather than
            // lifting one when the comparator is reached.
            for _ in 0..2 {
                interp.run_budget(4096);
                jit.run_budget(4096);
            }
            let cfg = Config::cortex_a53();
            for cpu in [&interp, &jit] {
                let mut sys = cpu.sysregs();
                // Enabled and unmasked, with a deadline well inside the next
                // quantum's chain rather than at its edge.
                sys.cntp_ctl = 1;
                sys.cntp_cval = cfg.counter_at(cpu.cycles()) + 37;
                cpu.set_sysregs(sys);
            }
            for n in 0..4 {
                assert_eq!(
                    interp.run_budget(4096),
                    jit.run_budget(4096),
                    "quantum {n} after the timer was armed, under {engine:?}"
                );
            }
            assert_eq!(
                interp.sysregs().elr_el1,
                jit.sysregs().elr_el1,
                "ELR_EL1 under {engine:?}: the two engines took the generic \
                 timer's interrupt at different instructions"
            );
            assert_ne!(
                interp.sysregs().elr_el1,
                0,
                "the interrupt was never taken, so this proves nothing"
            );
            assert_eq!(interp.pc(), jit.pc(), "the pc under {engine:?}");
            assert_eq!(
                interp.cycles(),
                jit.cycles(),
                "the cycle counter under {engine:?}"
            );
            for n in 0..31 {
                assert_eq!(interp.x(n), jit.x(n), "x{n} under {engine:?}");
            }
            let stats = jit.jit_stats().expect("a jit core");
            assert!(stats.blocks > 0, "no block ran under {engine:?}");
        }
    }

    /// A loop that flushes its own translations, so the entry fetch after the
    /// `TLBI` is a **cold walk** — the window [`Admitted::leave`] closes.
    ///
    /// The `TLBI` is outside the lifted subset, so `advance` interprets it and
    /// returns; the next call starts at `0x0c` with nothing in the fetch set,
    /// walks, and charges for it. Everything after it is one chain of lifted
    /// ALU, which is where the timer has to be able to fire.
    const TLBI_LOOP: [u32; 8] = [
        0x9100_0400, // add  x0, x0, #1
        0x8b00_0021, // add  x1, x1, x0
        0xd508_871f, // tlbi vmalle1        ; every pass makes this page cold
        0x8b01_0042, // add  x2, x2, x1
        0x8b02_0063, // add  x3, x3, x2
        0x8b03_0084, // add  x4, x4, x3
        0x8b04_00a5, // add  x5, x5, x4
        0x17ff_fff9, // b    .-28
    ];

    /// The body of the two tests below: arm the comparator at every offset
    /// in a sweep, and require the two engines to take the interrupt at the
    /// same instruction whichever cold walk the edge lands inside.
    ///
    /// `Config::cortex_a53` divides the counter by one, so every tick is a
    /// comparator and a walk is two or three of them wide. Somewhere in the
    /// sweep the edge lands inside one — which is why this sweeps rather
    /// than naming an offset.
    fn timer_across_a_cold_walk(program: &[u32], tail: &[u32]) {
        for engine in [Engine::Jit, Engine::JitHost] {
            let mut taken = 0usize;
            for delta in 0..48u64 {
                let interp = core(Engine::Interp, program);
                let jit = core(engine, program);
                for cpu in [&interp, &jit] {
                    let space = cpu.space().expect("the core has its space");
                    // The second page, where there is one.
                    for (n, word) in tail.iter().enumerate() {
                        space
                            .write(
                                0x1000 + 4 * n as u64,
                                Width::U32,
                                u64::from(*word),
                                MemAttrs::DEFAULT,
                            )
                            .expect("inside RAM");
                    }
                    // The same handler the test above installs: mask the
                    // source, then stop, so `ELR_EL1` records where the
                    // interrupt was taken rather than converging on the
                    // handler's own address.
                    for (n, word) in [0xd503_42dfu64, 0x1400_0000].iter().enumerate() {
                        space
                            .write(
                                VBAR + IRQ_VECTOR + 4 * n as u64,
                                Width::U32,
                                *word,
                                MemAttrs::DEFAULT,
                            )
                            .expect("inside RAM");
                    }
                    enable_mmu(cpu);
                    let mut sys = cpu.sysregs();
                    sys.daif = 0;
                    sys.vbar_el1 = VBAR;
                    cpu.set_sysregs(sys);
                }
                // Warm both, so the translated core is running a chain rather
                // than lifting one when the comparator is reached.
                for _ in 0..3 {
                    interp.run_budget(4096);
                    jit.run_budget(4096);
                }
                assert_eq!(
                    interp.cycles(),
                    jit.cycles(),
                    "delta {delta} under {engine:?}: the two engines parted \
                     while warming, before the timer was ever armed"
                );
                let cfg = Config::cortex_a53();
                for cpu in [&interp, &jit] {
                    let mut sys = cpu.sysregs();
                    sys.cntp_ctl = 1;
                    sys.cntp_cval = cfg.counter_at(cpu.cycles()) + delta;
                    cpu.set_sysregs(sys);
                }
                for n in 0..3 {
                    assert_eq!(
                        interp.run_budget(4096),
                        jit.run_budget(4096),
                        "delta {delta}, quantum {n} under {engine:?}"
                    );
                }
                assert_eq!(
                    interp.sysregs().elr_el1,
                    jit.sysregs().elr_el1,
                    "ELR_EL1 at delta {delta} under {engine:?}: the two engines \
                     took the generic timer's interrupt at different \
                     instructions"
                );
                assert_eq!(interp.pc(), jit.pc(), "the pc at delta {delta}");
                assert_eq!(
                    interp.cycles(),
                    jit.cycles(),
                    "the cycle counter at delta {delta}"
                );
                assert_eq!(
                    interp.cycle_debt(),
                    jit.cycle_debt(),
                    "the carried overrun at delta {delta}"
                );
                for n in 0..31 {
                    assert_eq!(interp.x(n), jit.x(n), "x{n} at delta {delta}");
                }
                if interp.sysregs().elr_el1 != 0 {
                    taken += 1;
                }
                let stats = jit.jit_stats().expect("a jit core");
                assert!(stats.blocks > 0, "no block ran at delta {delta}");
            }
            assert!(
                taken > 0,
                "the timer never fired under {engine:?}, so this proves nothing"
            );
        }
    }

    #[test]
    fn the_generic_timer_is_taken_at_the_same_instruction_across_a_tlbi() {
        // The defect [`Admitted::leave`] exists for, and it is not the one the test
        // above covers. There the comparator is crossed by a tick a *block*
        // charged, and `IrHost::spent` sees it. Here it is crossed by the
        // entry translation `admit` charges **after** it has looked for a
        // pending interrupt and **before** `Host::new` computes the edge — and
        // `Exec::timer_edge` answers `u64::MAX` for a comparator already
        // crossed, so the run took that for its edge and no boundary inside it
        // ever left. The interrupt waited for the next chained boundary's
        // `admit`, two guest instructions further on.
        //
        // It takes a cold *instruction-fetch* translation to open, which on
        // this core only a `TLBI` produces — `mmu::Tlb` keeps fetch, load and
        // store entries in three separate sets, so no amount of data-side
        // pressure evicts a code page. `tests/engine_longrun.rs` reaches it
        // from a synthetic guest in 0.417 s and a forty-second arm64 Linux
        // boot does not reach it at all, which is why this sweeps the arming
        // offset instead of naming one: `Config::cortex_a53` divides the
        // counter by one, so every tick is a comparator and the walk is two or
        // three of them wide. Somewhere in the sweep the edge lands inside it.
        timer_across_a_cold_walk(&TLBI_LOOP, &[]);
    }

    /// [`TLBI_LOOP`] with its tail on the **next page**, so the walk the
    /// timer has to be able to fire inside is a *chained* boundary's.
    ///
    /// `TLBI_LOOP` reaches `admit` through [`advance`]'s prologue and
    /// nothing else: one page, one entry translation per run. The `b` here
    /// leaves the page mid-chain, so the successor's entry translation
    /// happens inside `Frontend::enter` instead — the same window at the
    /// other call site, and the one a single-page loop cannot reach.
    const TLBI_PAGE_LOOP: [u32; 5] = [
        0x9100_0400, // add  x0, x0, #1
        0x8b00_0021, // add  x1, x1, x0
        0xd508_871f, // tlbi vmalle1        ; every pass makes both pages cold
        0x8b01_0042, // add  x2, x2, x1
        0x1400_03fc, // b    .+0xff0        ; to 0x1000, the next page
    ];

    /// The other page of [`TLBI_PAGE_LOOP`], at `0x1000`.
    const TLBI_PAGE_LOOP_TAIL: [u32; 4] = [
        0x8b02_0063, // add  x3, x3, x2
        0x8b03_0084, // add  x4, x4, x3
        0x8b04_00a5, // add  x5, x5, x4
        0x17ff_fbfd, // b    .-0x100c       ; back to 0
    ];

    #[test]
    fn the_generic_timer_is_taken_at_the_same_instruction_across_a_chained_page() {
        // The other call site, and — for the *timer* — the one that needs no
        // fix. `admit` runs from `advance`'s prologue and from
        // `Frontend::enter`, and only the first is on a single-page loop: a
        // chain that never leaves its page re-enters through a translation
        // that hits. Here it does not, so the walk happens inside `enter`,
        // after `Dispatcher::run` has already asked `IrHost::spent` for this
        // block. `Host::timer_edge` covers it anyway, and that is the point
        // worth pinning: it is an absolute cycle count, so a walk that pushes
        // the counter past it is caught by the very next boundary's
        // comparison rather than needing to be noticed when it happens.
        // `Admitted::leave` is what the same call site needs for a line a
        // walk's *reads* raise, which has no second chance — see
        // `a_walk_that_raises_an_interrupt_is_taken_where_the_interpreter_takes_it`.
        timer_across_a_cold_walk(&TLBI_PAGE_LOOP, &TLBI_PAGE_LOOP_TAIL);
    }

    /// A level-1 translation table that lives **in a device** rather than in
    /// DRAM, and asserts this core's `IRQ` line on the `at`-th walk that reads
    /// it.
    ///
    /// The other half of the window [`Admitted::leave`] closes, and the half
    /// the timer cannot reach. `TTBR0_EL1` is a guest-written register and
    /// `mmu`'s walker masks it rather than range-checking it, and the
    /// descriptors come back through the ordinary [`AddressSpace`] with
    /// `MemAttrs::debug` clear, so whatever is mapped there answers.
    ///
    /// Nothing on `arm64-virt` can *be* that thing today, and it is worth
    /// being exact about why: a descriptor read is always [`Width::U64`], and
    /// `arm.gic`'s distributor accepts `U8`..`U32` and its CPU interface only
    /// `U32`, so a walk over `GICC_IAR` takes an external abort instead of
    /// acknowledging an interrupt. That is a width constraint on one device on
    /// one board, not a property of this core — `riscv.clint` and `pc.hpet`
    /// both take the width their architecture's walk uses — so the engine
    /// closes the window rather than leaning on it.
    #[derive(Debug)]
    struct TableThatRaises {
        cpu: crate::core::sync::Mutex<alloc::sync::Weak<Cpu>>,
        /// How many walks have read it, and which one raises.
        seen: crate::core::sync::AtomicU64,
        at: u64,
    }

    impl crate::core::space::MemOps for TableThatRaises {
        fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
            // Only index 0 is ever asked for: a level-1 entry covers 1 GiB and
            // every virtual address this fixture uses is in the first of them.
            let word = if offset == 0 {
                DEVICE_L2 | desc::VALID | desc::TABLE
            } else {
                0
            };
            for (i, byte) in dst.iter_mut().enumerate() {
                *byte = (word >> (8 * i)) as u8;
            }
            if attrs.debug {
                return Ok(());
            }
            if self.seen.fetch_add(1, crate::core::sync::Ordering::Relaxed) == self.at
                && let Some(cpu) = self.cpu.lock().upgrade()
            {
                cpu.set_interrupt(Lines::IRQ, true);
            }
            Ok(())
        }

        fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
            Ok(())
        }
    }

    /// Where [`TableThatRaises`] is mapped, and therefore what `TTBR0_EL1`
    /// points at: just past the RAM, so nothing overlaps.
    const DEVICE_L1: u64 = RAM;

    /// The level-2 table [`TableThatRaises`]'s one descriptor points at, in
    /// RAM. Its entry 0 is a 2 MiB identity block, which covers every page
    /// this fixture uses.
    const DEVICE_L2: u64 = 0x3_0000;

    /// A core running [`TLBI_PAGE_LOOP`] whose level-1 table is a device.
    fn walking_core(engine: Engine, at: u64) -> Arc<Cpu> {
        let ram = Arc::new(RamStore::new(RAM));
        write_words(&ram, 0, &TLBI_PAGE_LOOP);
        write_words(&ram, 0x1000, &TLBI_PAGE_LOOP_TAIL);
        // The handler `timer_across_a_cold_walk` installs: mask the source,
        // then stop, so `ELR_EL1` records where the interrupt was taken.
        write_words(&ram, VBAR + IRQ_VECTOR, &[0xd503_42df, 0x1400_0000]);
        // Level 2, entry 0: a 2 MiB identity block.
        write_words(&ram, DEVICE_L2, &[(desc::VALID | desc::AF) as u32, 0]);
        let table = Arc::new(TableThatRaises {
            cpu: crate::core::sync::Mutex::with_rank(
                crate::core::sync::LockRank::LEAF,
                alloc::sync::Weak::new(),
            ),
            seen: crate::core::sync::AtomicU64::new(0),
            at,
        });
        let space = AddressSpace::new("mem", 64);
        {
            let mut topo = space.topology();
            topo.map_with_perms(Region::ram("ram", ram), 0, Perms::RWX)
                .expect("nothing else is mapped");
            topo.map(
                Region::io(
                    "table",
                    0x1000,
                    Arc::clone(&table) as Arc<dyn crate::core::space::MemOps>,
                ),
                DEVICE_L1,
            )
            .expect("it does not overlap the RAM");
        }
        let cpu = Arc::new(Cpu::new(Config::cortex_a53().with_reset_vector(0)).with_engine(engine));
        cpu.attach_space(Arc::new(space));
        let mut sys = cpu.sysregs();
        sys.ttbr0 = DEVICE_L1;
        // T0SZ = T1SZ = 25 (39-bit halves), TG1 = 0b10 (the 4 KiB granule) —
        // `enable_mmu`'s, so the walk starts at level 1 and the device's one
        // descriptor is the top of it.
        sys.tcr = 25 | (25 << 16) | (0b10 << 30);
        sys.sctlr |= sctlr::M;
        sys.daif = 0;
        sys.vbar_el1 = VBAR;
        cpu.set_sysregs(sys);
        *table.cpu.lock() = Arc::downgrade(&cpu);
        cpu
    }

    #[test]
    fn a_walk_that_raises_an_interrupt_is_taken_where_the_interpreter_takes_it() {
        // The half of the entry-translation window the generic timer cannot
        // demonstrate. A walk whose *ticks* cross the comparator is caught at
        // a chained boundary anyway, because `Host::timer_edge` is an absolute
        // cycle count and the very next comparison sees it; a walk whose
        // *reads* raise a line has no such second chance, and before
        // `Admitted::leave` nothing looked between `Exec::pending_interrupt`
        // and the block's first instruction. Sweeping which walk raises puts
        // the line on `advance`'s prologue translation and on a chained
        // boundary's in turn.
        for engine in [Engine::Jit, Engine::JitHost] {
            let mut taken = 0usize;
            for at in 0..8u64 {
                let interp = walking_core(Engine::Interp, at);
                let jit = walking_core(engine, at);
                for n in 0..6 {
                    assert_eq!(
                        interp.run_budget(4096),
                        jit.run_budget(4096),
                        "walk {at}, quantum {n} under {engine:?}"
                    );
                }
                assert_eq!(
                    interp.sysregs().elr_el1,
                    jit.sysregs().elr_el1,
                    "ELR_EL1 at walk {at} under {engine:?}: the two engines took \
                     the interrupt at different instructions"
                );
                assert_eq!(interp.pc(), jit.pc(), "the pc at walk {at}");
                assert_eq!(
                    interp.cycles(),
                    jit.cycles(),
                    "the cycle counter at walk {at}"
                );
                assert_eq!(
                    interp.cycle_debt(),
                    jit.cycle_debt(),
                    "the carried overrun at walk {at}"
                );
                for n in 0..31 {
                    assert_eq!(interp.x(n), jit.x(n), "x{n} at walk {at}");
                }
                if interp.sysregs().elr_el1 != 0 {
                    taken += 1;
                }
                let stats = jit.jit_stats().expect("a jit core");
                assert!(stats.blocks > 0, "no block ran at walk {at}");
            }
            assert!(
                taken > 0,
                "the line never came up under {engine:?}, so this proves nothing"
            );
        }
    }

    #[test]
    fn an_interrupt_is_taken_at_the_same_instruction_by_both_engines() {
        // Nothing in the lifted subset can raise one, so an interrupt is
        // looked for at every block boundary rather than every instruction —
        // and a translated core that skipped that check would take it up to
        // sixty-four instructions late. Asserted rather than argued.
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = core(Engine::Interp, &LOOP);
            let jit = core(engine, &LOOP);
            for cpu in [&interp, &jit] {
                // `PSTATE.DAIF` is all set out of reset, so an IRQ routed to
                // EL1 and taken at EL1 would be masked.
                let mut sys = cpu.sysregs();
                sys.daif = 0;
                sys.vbar_el1 = VBAR;
                cpu.set_sysregs(sys);
                // A handler that masks the line and **stops**, at the
                // "current EL with SP_ELx, IRQ" slot. Without one the guest
                // executes the zeroed vector, takes a second exception, and
                // overwrites `ELR_EL1` with the vector's own address — which
                // is the same fixed point whenever the first interrupt was
                // taken, so the column under test converges and the comparison
                // proves nothing. A mutation pass found exactly that: dropping
                // the interrupt check from `admit` survived until this handler
                // was here. `MSR DAIFSet, #2` before the `WFI` is the other
                // half: the line is level-triggered and nothing lowers it, so
                // an unmasked handler would be re-entered and `ELR_EL1` would
                // converge on the handler's own address instead. The `B .` is
                // the third half: `WFI` does **not** stall while a line is
                // asserted, masked or not — a pending interrupt is a wake-up
                // event whatever `PSTATE.I` says (DDI 0487 D1) — so a handler
                // that ended in one would run off into the zeroed page after
                // it.
                let space = cpu.space().expect("the core has its space");
                for (n, word) in [0xd503_42dfu64, 0x1400_0000].iter().enumerate() {
                    space
                        .write(
                            VBAR + IRQ_VECTOR + 4 * n as u64,
                            Width::U32,
                            *word,
                            MemAttrs::DEFAULT,
                        )
                        .expect("inside RAM");
                }
            }
            for _ in 0..2 {
                interp.run_budget(4096);
                jit.run_budget(4096);
            }
            interp.set_interrupt(Lines::IRQ, true);
            jit.set_interrupt(Lines::IRQ, true);
            for n in 0..4 {
                assert_eq!(
                    interp.run_budget(4096),
                    jit.run_budget(4096),
                    "quantum {n} after the interrupt, under {engine:?}"
                );
            }
            assert_eq!(interp.pc(), jit.pc(), "the pc under {engine:?}");
            assert_eq!(
                interp.sysregs().elr_el1,
                jit.sysregs().elr_el1,
                "ELR_EL1 under {engine:?}: the two engines took the interrupt at \
                 different instructions"
            );
            assert_eq!(interp.cycles(), jit.cycles(), "cycles under {engine:?}");
            assert_ne!(interp.sysregs().elr_el1, 0, "the interrupt was taken");
            assert_eq!(
                interp.pc(),
                VBAR + IRQ_VECTOR + 4,
                "the core is spinning in the handler"
            );
            assert!(
                interp.sysregs().elr_el1 < 4 * LOOP.len() as u64,
                "ELR_EL1 is {:#x}, which is not an instruction of the program: \
                 the interrupt reached the handler by some other door and the \
                 column this test compares has converged on a fixed point",
                interp.sysregs().elr_el1
            );
        }
    }

    #[test]
    fn a_chain_is_bounded_by_the_stated_safe_point_number() {
        // `Cpu::run_budget` tests the exit flag between calls to `advance`, so
        // a chain is how long a raised flag can go unhonoured. The bound is
        // `CHAIN` blocks, and this asserts the dispatcher is actually given it.
        assert_eq!(CHAIN, 16);
        assert_eq!(lift::MAX_INSNS, 64);
    }

    #[test]
    fn the_subset_table_remembers_a_block_and_a_non_block_apart() {
        let mut subset = Subset::new();
        assert_eq!(subset.get(0x1000, 7), None, "unknown is not the same as no");
        subset.put(0x1000, 7, false);
        assert_eq!(subset.get(0x1000, 7), Some(false));
        // A different world at the same PC is a different answer.
        assert_eq!(subset.get(0x1000, 8), None);
        subset.put(0x1000, 7, true);
        assert_eq!(subset.get(0x1000, 7), Some(true));
        subset.clear();
        assert_eq!(subset.get(0x1000, 7), None);
    }

    #[test]
    fn a_bare_block_and_a_paged_one_key_differently() {
        assert_eq!(key_origin(false, 0x4000), Origin::Bare);
        assert_eq!(
            key_origin(true, 0x4123),
            Origin::Paged { generation: 4 },
            "the key is the physical page, not the offset within it"
        );
    }
}
