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
//! * **A block never runs unless its worst case fits the budget left.**
//!   Otherwise the guest's *stopping point* inside a scheduler quantum would
//!   depend on the engine: an interpreter overruns its budget by one
//!   instruction and a trace by up to sixty-four, the overrun is carried as
//!   `State::debt`, and both numbers are in the snapshot a machine's state
//!   hash is taken over. [`Costs`] keeps the bound tight enough for that tail
//!   to be short, and the guard is asked once per block of a chain, against
//!   what the chain has *left*.
//! * **A pending interrupt is looked for at every block boundary**, chained
//!   ones included, which is what keeps a sixteen-block chain
//!   indistinguishable from sixteen one-block calls. A store into the GIC ends
//!   its block by construction ([`lift`](super::lift), "A store ends the
//!   block"), so the interrupt it raises is seen before the next block starts.
//!
//! # What it buys, measured
//!
//! On the guest this exists for — `machines/arm64-virt.machine` booting
//! Linux 6.12 `arm64` with a busybox initramfs, 512 MiB of DRAM — one binary,
//! three engines, over twenty seconds of virtual time:
//!
//! | `engine` | 20 s of guest time |
//! | --- | --- |
//! | `interp` | 17.35 s |
//! | `jit` | 11.21 s (1.55×) |
//! | `jit-host` | **6.34 s (2.73×)** |
//!
//! Median of three **interleaved** reps — one of each engine, in turn, round
//! and round — because the interpreter is the control and a control measured
//! in a different sitting is not one. All nine runs charged 199 990 000 cycles
//! and finished on one state hash, `0x415f52aebd310878`.
//!
//! What the mechanisms did, over that run:
//!
//! | | |
//! | --- | --- |
//! | blocks executed | 18 774 915 |
//! | of those, compiled to host code | 18 774 316 (**99.997%**) |
//! | of those, reached by a patched exit | 14 341 043 (76.4%) |
//! | distinct blocks lifted | 16 655 |
//! | guest instructions retired **inside** a block | 123 794 600 (**79.2%**) |
//! | compiled loads served by an inlined probe | 16 149 884 |
//! | compiled stores served the same way | 10 535 057 |
//! | translations a guest store invalidated | 7 801 |
//!
//! The 599 blocks the code generator refused are the ones holding a `UDIV` or
//! an `SDIV`, which are the only two ops this frontend emits that `jit::x86`
//! does not lower. That is 0.003%, and it is why [`lift`](super::lift) goes
//! out of its way not to emit [`Opcode::ADDC`](crate::ir::Opcode::ADDC): the
//! architecture's own `AddWithCarry` is refused too, and `CMP` is `SUBS`, so a
//! lifter that used it would have had *every* block refused rather than six
//! hundred.
//!
//! The inlined probes are worth the last step of that ratio on their own:
//! before `mmu::Tlb` had a shadow to publish, the same sweep put `jit-host` at
//! 6.92 s and **2.54×**.
//!
//! # What is checked at a block boundary rather than at an instruction
//!
//! **Pending interrupts, the `WFI` stall and the generic timer**, and it is
//! worth being exact about why that is sound. Within a block nothing this core
//! does can raise one: `MSR`, `MRS`, `ERET`, `WFI`, `SVC` and every system
//! operation are outside the lifted subset and end the block, a **store** ends
//! the block by construction, and the timer's comparator is reached by ticks
//! the block charges — which are read at the next boundary, at most
//! [`lift::MAX_INSNS`] instructions later.
//!
//! That last one is a real, bounded imprecision and it is stated rather than
//! implied: an interpreted core samples `CNTPCT_EL0` against the comparator
//! once per instruction, and a translated one samples it once per block. The
//! two therefore raise the timer interrupt at the same *tick* — the counter is
//! the core's own tick count either way — but a translated core can be up to a
//! block late in *noticing*. `Exec::publish_timer_levels` is called once per
//! [`advance`] for the same reason it is called once per `Exec::step`.
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
use crate::core::space::{AddressSpace, MemAttrs, MemResult};
use crate::core::value::Width;
use crate::ir::{Block, InsnStart, IrHost, MemOp, Opcode, RegSlot, verify};
use crate::jit::{
    BlockCache, DirtyPages, Dispatcher, Entry, Epoch, FastMem, Frontend, MemPlan, PAGE_MASK, Stop,
    StoreLog, Translation,
};

use super::exec::{Exec, State, Trap};
use super::isa::Nzcv;
use super::lift::{self, Origin, PC, SP, Shape, World};
use super::mmu::{Access, Tlb};
use super::sysreg::sctlr;
use super::{Config, Lines};

/// How much of a block the frontend is allowed to swallow.
///
/// [`Shape::Trace`] is the dispatcher's shape: direct branches are merged, so
/// a loop unrolls into one translation and a guest register stays in a
/// temporary across the whole of it.
const SHAPE: Shape = Shape::Trace;

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

/// The most bus accesses one VMSAv8-64 walk can make.
///
/// Four: the deepest regime this core implements starts at level 0 and reads
/// one descriptor per level, and without `FEAT_HAFDBS` an AArch64 walk never
/// writes one back — which is why there is no `+1` here where the RISC-V
/// engine has one for its accessed/dirty write.
///
/// Used only to bound a block's worst case, so a shallower walk makes the
/// bound conservative rather than wrong.
const WALK_ACCESSES: u64 = 4;

/// How many blocks this core's cache holds before it evicts.
///
/// `jit::BlockCache`'s own default is 8 192, which a Linux guest thrashes: the
/// RISC-V engine measured 1 096 143 insertions against 1 079 470 evictions
/// over four minutes of guest time at that size, where every eviction is a
/// re-lift and, with a code generator attached, a re-compile. The number is a
/// bound rather than an allocation — a board whose guest has a small working
/// set never fills it.
const BLOCKS: usize = 65536;

/// How many `(pc, key) -> worst-case ticks` answers are remembered.
///
/// Direct-mapped and keyed by the guest PC, exactly as the block cache is, and
/// sized with it so a resident block usually has a resident cost. A miss is a
/// conservative answer, never a wrong one.
const COST_SLOTS: usize = 65536;

/// How big a host code buffer this core asks for: 256 MiB.
///
/// The mapping is anonymous, so what is not written is not resident, and it is
/// only asked for at all by `engine = "jit-host"`. `jit::x86::buf` flips a
/// page-sized window rather than the whole mapping, so a bigger buffer costs
/// address space and nothing per compile — which is what lets the number stand
/// where the RISC-V engine measured a 32 MiB buffer resetting 111 times in
/// four minutes of guest time.
#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
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
    costs: Costs,
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
            retired: 0,
            interpreted: 0,
            smc: 0,
        }
    }

    /// Throw every translation away.
    pub(super) fn flush(&mut self) {
        self.disp.cache_mut().flush();
        self.costs.clear();
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
        #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
        {
            self.disp
                .backend()
                .map(crate::jit::x86::Engine::stats)
                .map_or((0, 0), |s| (s.fast_loads, s.fast_stores))
        }
        #[cfg(not(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64")))]
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
    /// portable backend because the host is not x86-64 Linux does not ask.
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
            // answer in the cost table may have changed with it: an
            // instruction that was outside the subset can have been
            // overwritten by one that is not.
            self.costs.clear();
        }
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
/// should be reached directly. Without it every `MSR`, every `SVC` and every
/// floating-point instruction costs a dispatcher round trip and a fresh
/// [`lift::lift`] that fails at its first instruction — and an AArch64 kernel
/// is full of all three. Zero is a safe sentinel because a block that exists
/// charges at least one tick for its own fetch.
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

    /// Every A64 instruction is four bytes, so the low two bits of a guest PC
    /// carry nothing.
    #[inline]
    fn index(pc: u64) -> usize {
        ((pc >> 2) as usize) & (COST_SLOTS - 1)
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

/// What one guest access can cost this core, at worst.
///
/// One bus cycle when naturally aligned; one per byte when it splits, which a
/// core with `SCTLR_EL1.A` clear does rather than faulting; and a walk in
/// front of each of those when translation is on, because each byte of a split
/// access is translated on its own and may miss.
const fn per_access(strict_align: bool, translating: bool) -> u64 {
    let split = if strict_align { 1 } else { 8 };
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
/// instruction limit, every instruction a pair access.
///
/// A **pair**, which is the number [`lift::MAX_INSNS`]'s own documentation is
/// derived from: `LDP`/`STP` make two accesses from one instruction, so the
/// worst guest instruction costs its fetch plus two of them.
const fn worst_bound(strict_align: bool, translating: bool) -> u64 {
    let entry = if translating { WALK_ACCESSES } else { 0 };
    lift::MAX_INSNS as u64 * (1 + 2 * per_access(strict_align, translating)) + entry
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
    /// interrupt, a stalled `WFI`, a misaligned PC, an instruction outside the
    /// lifted subset, or a worst case that does not fit what is left of the
    /// budget.
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
/// **The interrupt check first.** A pending interrupt and a stalled `WFI` are
/// both the interpreter's, and `Exec::step` is how each is taken. Asking first
/// is not an optimization: `step` takes the trap, and a block run instead
/// would take it up to sixty-four instructions late. Asking it *per block*
/// rather than per run is what keeps a chained run indistinguishable from a
/// sequence of one-block ones.
///
/// **Then the entry fetch translation**, charged exactly as the interpreter's
/// own fetch charges it, and performed on every execution rather than at lift
/// time, because a cached block must cost what an uncached one cost.
///
/// **Then the budget guard**, and it is after the translation because the
/// translation is also what *names* the block. A guard that declines
/// afterwards has not wasted the walk: the interpreter's own fetch then finds
/// the entry this translation just filled, and charges what it would have
/// charged anyway.
fn admit(cfg: &Config, costs: &Costs, exec: &mut Exec<'_>, pc: u64, remaining: u64) -> Admit {
    if exec.pending_interrupt().is_some() || exec.st.wfi {
        return Admit::Interpret;
    }
    let translating = exec.st.sys.mmu_enabled();
    let strict_align = exec.st.sys.sctlr & sctlr::A != 0;
    let phys = match exec.translate_fetch(pc) {
        Ok(phys) => phys,
        Err(trap) => return Admit::Trap(trap),
    };
    let world = World {
        features: cfg.features,
        origin: key_origin(translating, phys),
        strict_align,
    };
    let key = lift::key(&world, SHAPE);

    // Known unliftable, or too big for what is left of the budget: either way
    // the interpreter takes this instruction, and reaching it without a lift
    // that fails at its first instruction is the whole point of remembering
    // the first.
    let bound = match costs.get(pc, key) {
        Some(0) => return Admit::Interpret,
        Some(bound) => bound,
        None => worst_bound(strict_align, translating),
    };
    if bound > remaining.saturating_sub(exec.used) {
        return Admit::Interpret;
    }

    Admit::Ready(Admitted {
        world,
        key,
        page: pc & !PAGE_MASK,
        base: phys & !PAGE_MASK,
        access: per_access(strict_align, translating),
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
/// same currency and with the same meaning as `Cpu::step_to_exit`, so a run
/// loop cannot tell which engine it is driving.
///
/// `remaining` is what is left of the caller's budget. A block whose worst
/// case does not fit is not run and the instruction is interpreted instead, so
/// that the core stops where an interpreted core would stop — and that holds
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
    tlb: &mut Tlb,
    space: &Arc<AddressSpace>,
    cfg: &Config,
    lines: &Lines,
    exits: ExitMask,
    remaining: u64,
) -> (u64, Option<Exit>) {
    let Jit {
        disp,
        costs,
        retired,
        interpreted,
        smc,
    } = jit;
    let mut exec = Exec::new(state, tlb, space, cfg, lines, exits);
    let pc = exec.st.pc;

    // The entry work for the *first* block, done here rather than through
    // `Frontend::enter`, because the overwhelmingly common answer on a real
    // guest is "not a block at all" — an `MSR`, an `SVC`, a floating-point
    // instruction — and reaching the interpreter for one should not cost a
    // frontend, a host and a dispatcher round trip.
    let at = match admit(cfg, costs, &mut exec, pc, remaining) {
        Admit::Ready(at) => at,
        Admit::Interpret => return interpret(interpreted, smc, disp, costs, exec),
        // The instruction at `pc` has not started, so its own PC is both where
        // the trap is taken and where it resumes — the same pair
        // `Exec::step_once` would produce for a fetch abort. The walk this
        // translation just charged is *not* re-charged: the interpreter is not
        // asked to fetch again, because it would walk again.
        Admit::Trap(trap) => return deliver(smc, disp, costs, exec, trap, pc, pc),
    };

    let mut front = Lifter {
        cfg,
        at,
        space,
        // Lifting reads *ahead* of the guest: up to sixty-four instructions it
        // has not asked for. A fetch is an ordinary access and a read-ahead is
        // not, so this is the one place in the core that reads guest memory
        // the way a debugger does — CLAUDE.md's "a debugger read must not pop
        // a FIFO" is exactly the hazard. Nothing about the *translation* is
        // relaxed: that happened above, through the fetch path, with its walk
        // and its permission check.
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
        // This frontend refuses no world, so this is unreachable; degrade
        // rather than fail the machine if it ever is not (`ROADMAP.md` §9).
        Err(_) => {
            drop(host);
            let Lifter { costs, .. } = front;
            return interpret(interpreted, smc, disp, costs, exec);
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
        "the AArch64 frontend emitted a block the verifier rejects: {rejected:?}"
    );

    if run.blocks == 0 {
        // Nothing executed: the instruction at `pc` is outside the lifted
        // subset, and `Frontend::translate` has just recorded that so the next
        // pass skips straight to here. The interpreter takes it, and its own
        // fetch translation now hits the TLB the translation above filled — so
        // what it charges is what a purely interpreted core would.
        return interpret(interpreted, smc, disp, costs, exec);
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
            deliver(smc, disp, costs, exec, trap, fault.pc, next)
        }
        Stop::Unsupported { op, at } => panic!(
            "the AArch64 frontend emitted {op} at index {at}, which the IR backend cannot execute"
        ),
        // A chained boundary whose entry fetch faulted. The instruction at
        // `run.pc` has not started, so its own PC is both where the trap is
        // taken and where it resumes.
        Stop::Declined if entry_trap.is_some() => {
            let trap = entry_trap.expect("just tested");
            deliver(smc, disp, costs, exec, trap, run.pc, run.pc)
        }
        // `Budget` ends a full chain, `Declined` a short one, `Untranslatable`
        // one that reached an instruction outside the subset, and all three
        // leave the guest at `run.pc` for the run loop to pick up. `Exit`
        // cannot happen: no safe-point flag is given to the dispatcher,
        // because the run loop above checks it between calls.
        _ => {
            exec.st.pc = run.pc;
            // Once per call, whatever the call did — the same rule
            // `Exec::step` follows. A chain that ran only blocks charges the
            // ticks the generic timer's comparator is reached by, and the wire
            // out has to follow within the same call or an idle kernel waits
            // for an interrupt its own timer already raised.
            //
            // **No test isolates this line, and that is stated rather than
            // hidden.** A mutation pass removed it and everything still
            // passed, because every quantum's tail is interpreted — `admit`
            // declines once the remaining budget is smaller than a block's
            // bound — and `Exec::step` publishes on the way past. It is here
            // because a guest whose whole quantum fits in one chain would not
            // get that, and because the rule "once per call, whatever the call
            // did" is easier to keep than the case analysis that says when it
            // may be skipped.
            exec.publish_timer_levels();
            let used = exec.used;
            drain(smc, disp, costs, &mut exec);
            (used.max(1), None)
        }
    }
}

/// Interpret one instruction, and tell the block cache what it wrote.
fn interpret(
    interpreted: &mut u64,
    smc: &mut u64,
    disp: &mut Dispatcher,
    costs: &mut Costs,
    mut exec: Exec<'_>,
) -> (u64, Option<Exit>) {
    *interpreted = interpreted.wrapping_add(1);
    let used = exec.step();
    let exit = exec.take_exit();
    drain(smc, disp, costs, &mut exec);
    (used, exit)
}

/// Take a trap the block or an entry fetch raised, exactly as `Exec::step`
/// takes one: out of the core when the mask says so, into the guest's vector
/// table otherwise.
fn deliver(
    smc: &mut u64,
    disp: &mut Dispatcher,
    costs: &mut Costs,
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
    drain(smc, disp, costs, &mut exec);
    (used.max(1), out)
}

/// Hand what an interpreted instruction wrote to the block cache.
fn drain(smc: &mut u64, disp: &mut Dispatcher, costs: &mut Costs, exec: &mut Exec<'_>) {
    let mut hit = 0usize;
    for i in 0..exec.wrote_n as usize {
        hit += disp.cache_mut().note_write(exec.wrote[i], 1);
    }
    exec.wrote_n = 0;
    *smc = smc.wrapping_add(hit as u64);
    if hit > 0 {
        costs.clear();
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
    attrs: MemAttrs,
    costs: &'a mut Costs,
    /// What [`advance`] was given, so a chained boundary can guard the next
    /// block against what the chain has *left* rather than against the whole.
    remaining: u64,
    /// Whether [`advance`]'s prologue has already admitted the entry PC, so
    /// the dispatcher's first `enter` neither translates nor charges twice.
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
        // Registers live in the host's slots between the blocks of a chain and
        // are written back only when the run ends. Nothing `admit` reads is one
        // of them — it reads the system registers, the core's TLB and the tick
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
                .read(base | (addr & PAGE_MASK), Width::U32, attrs)
                .ok()
                .map(|v| v as u32)
        };
        let lifted = lift::lift(&self.at.world, pc, &mut src, lift::MAX_INSNS, SHAPE)?;
        if self.rejected.is_none()
            && let Err(e) = verify(&lifted.block)
        {
            self.rejected = Some(alloc::format!("{e}"));
        }
        // Zero when nothing could be lifted, which is what sends the next pass
        // straight to the interpreter instead of back through here.
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
    /// report a [`BusError`] and an A64 trap is a syndrome, a faulting address
    /// and a return policy.
    trap: Option<Trap>,
    /// The last guest instruction boundary the block announced, as
    /// `(pc, next_pc)`.
    mark: Option<(u64, u64)>,
    dirty: DirtyPages,
}

impl<'a, 'e> Host<'a, 'e> {
    fn new(exec: &'a mut Exec<'e>, pc: u64) -> Host<'a, 'e> {
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
            exec,
            slots,
            trap: None,
            mark: None,
            dirty: DirtyPages::new(),
        }
    }

    /// Report a trap as the bus error the IR speaks, keeping the syndrome.
    fn fault(&mut self, trap: Trap) -> BusError {
        self.trap = Some(trap);
        BusError::BadAccess
    }

    /// Move whatever the last access wrote into the dirty log.
    ///
    /// Whatever landed, landed: a split store that faulted on its second page
    /// still wrote the first, and a translation of those bytes is stale either
    /// way.
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
    use crate::core::space::{RamStore, Region};
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
        let cpu = Cpu::new(Config::cortex_a53().with_reset_vector(0)).with_engine(engine);
        cpu.attach_space(Arc::new(space));
        cpu
    }

    /// Run both cores on the same budgets and compare everything a guest, a
    /// snapshot or a state hash can see.
    fn agree_on(engine: Engine, program: &[u32], budget: u64, quanta: usize) -> (Cpu, Cpu) {
        let interp = core(Engine::Interp, program);
        let jit = core(engine, program);
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
        // Smaller than the cold worst case, so `admit` declines every block
        // and the whole run is interpreted — which must still consume exactly
        // what an interpreted core consumes.
        agree(&LOOP, 3, 40);
    }

    #[test]
    fn an_instruction_outside_the_subset_is_interpreted_without_a_wasted_lift() {
        // `mrs x0, midr_el1` is outside the subset; the cost table remembers
        // that so the next pass reaches the interpreter directly.
        let program = [0xd530_0000, 0xd280_0025, 0x17ff_ffff];
        agree(&program, 4096, 4);
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
    const HOST_BACKEND: bool = cfg!(all(
        feature = "jit-x86",
        target_os = "linux",
        target_arch = "x86_64"
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
    fn a_cold_block_fits_inside_a_scheduler_quantum() {
        // `cpu::x86::lift` records what happens when this is not checked: at
        // 64 instructions an x86 cold block bounds above
        // `max_ticks_per_quantum`, so no block is ever admitted. Asserted
        // here rather than left in prose, in the worst world this core has —
        // translation on, alignment checking off, every instruction a pair.
        let cap = crate::core::sched::SchedulerConfig::default().max_ticks_per_quantum;
        let worst = worst_bound(false, true);
        assert!(
            worst < cap,
            "a cold block bounds at {worst} ticks against a {cap}-tick quantum, \
             so no block would ever be admitted"
        );
        assert_eq!(worst, 64 * (1 + 2 * 40) + 4);
    }

    #[test]
    fn the_cost_table_remembers_a_block_and_a_non_block_apart() {
        let mut costs = Costs::new();
        assert_eq!(costs.get(0x1000, 7), None);
        costs.put(0x1000, 7, 0);
        assert_eq!(costs.get(0x1000, 7), Some(0));
        // A different world at the same PC is a different answer.
        assert_eq!(costs.get(0x1000, 8), None);
        costs.put(0x1000, 7, 42);
        assert_eq!(costs.get(0x1000, 7), Some(42));
        costs.clear();
        assert_eq!(costs.get(0x1000, 7), None);
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
