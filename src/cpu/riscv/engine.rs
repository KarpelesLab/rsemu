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
//!   That is also what bounds a dispatcher call to **one block**: [`Frontend`]
//!   has no per-entry hook, so letting `Dispatcher::run` chain a successor
//!   would skip that successor's entry translation. Block chaining is the one
//!   §9 mechanism this engine gives up, and buying it back is a seam change
//!   rather than a change here.
//! * **A block never runs unless its worst case fits the budget left.**
//!   Otherwise the guest's *stopping point* inside a scheduler quantum would
//!   depend on the engine: an interpreter overruns its budget by one
//!   instruction and a trace by up to sixty-four, the overrun is carried as
//!   `State::debt`, and both numbers are in the snapshot a machine's state
//!   hash is taken over. So the tail of every quantum is interpreted and the
//!   two engines stop on the same instruction with the same debt. [`Costs`] is
//!   what keeps the bound tight enough for that tail to be short.
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
//! | `interp` | 150.3 s | — |
//! | `jit` | 121.2 s | **1.24×** |
//! | `jit-host` | 299.1 s | 0.50× |
//!
//! All three end on the same guest state, byte for byte:
//! `state hash 0xf86f099b07119370`. `tests/riscv_virt_engines.rs` asserts that
//! equality on every commit, on a cheaper guest.
//!
//! Two things there are worth saying out loud rather than leaving to be
//! rediscovered.
//!
//! **The host code generator currently costs more than it saves**, which is
//! why it is a separate engine value rather than what `jit` does wherever it
//! can. It is not the code it emits; it is what surrounds it. A block on this
//! guest is about four guest instructions long — 293 million blocks for 2.4
//! billion ticks — and per block the compiled path re-zeroes a temporary
//! frame, builds a call context, and then calls back into the host across a C
//! ABI for every `CHARGE`, every `INSN_START` and every guest access, where the
//! IR interpreter's equivalents are monomorphized and inlined. Measured at
//! roughly 0.8 µs a block over and above interpreting the same IR, with
//! compilation itself accounted for separately (the run above compiles 162
//! thousand blocks and resets its code buffer once). Per-*instruction*
//! bookkeeping over four-instruction blocks is the whole of it.
//!
//! **And the speedup is 1.24×, not the 8–22× the code generator measures on a
//! benchmark**, because two of `ROADMAP.md` §9's four mechanisms are not
//! reachable from here: blocks are not chained (above), and no
//! [`LoadPlan`](crate::jit::LoadPlan) is published (see [`FastMem`] below), so
//! a guest access costs a call whichever engine runs it. It is also worth
//! knowing that the number moves with the guest's phase: over the first thirty
//! seconds, which is firmware and early kernel, the same measurement gives
//! 1.46×.
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
//! read both *lower* a line. Recorded here rather than discovered later. The
//! safe-point flag has the same granularity for the same reason
//! (`ROADMAP.md` §4.7), bounded by [`lift::MAX_INSNS`].
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
    BlockCache, DirtyPages, Dispatcher, Epoch, FastMem, Frontend, PAGE_MASK, Stop, StoreLog,
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
/// Linux is about four guest instructions long — a store ends a block, and so
/// does every instruction outside the lifted subset — and this backend emits
/// roughly three kilobytes of host code for one, so [`BLOCKS`] live blocks want
/// two hundred. The buffer is append-only and reclaimed only by a reset that
/// throws every compiled block away, so undersizing it is not a small cost: at
/// one mebibyte it reset **2 626 times in ten seconds of guest time** and
/// compiled nineteen thousand blocks 886 000 times, and at 32 MiB it reset 111
/// times in four minutes and compiled 1.59 million. At 256 MiB the same run
/// resets once and compiles 162 thousand.
///
/// The mapping is anonymous, so what is not written is not resident, and it is
/// only asked for at all by `engine = "jit-host"`. The cost of asking for too
/// much is address space; the cost of asking for too little is measured above.
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
// One step of the run loop
// ---------------------------------------------------------------------------

/// Execute one block, or — where a block would be wrong — one instruction.
///
/// Reports the bus accesses charged and the [`Exit`] the step produced, in the
/// same currency and with the same meaning as `Hart::step_to_exit`, so a run
/// loop cannot tell which engine it is driving.
///
/// `remaining` is what is left of the caller's budget. A block whose worst
/// case does not fit is not run and the instruction is interpreted instead, so
/// that the hart stops where an interpreted hart would stop.
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

    // A pending interrupt, a stalled `WFI` and an instruction outside the
    // lifted subset are all the interpreter's, and `Exec::step` is how each is
    // taken. Asking first is not an optimization: `step` takes the trap, and a
    // block run instead would take it up to sixty-four instructions late.
    if exec.pending_interrupt().is_some() || exec.st.wfi {
        return interpret(disp, costs, exec);
    }

    let pc = exec.st.pc;
    let mode = exec.st.csrs.priv_mode;
    let translating = mmu::translation_active(&exec.st.csrs, mode);

    // The entry fetch translation, charged exactly as the interpreter's first
    // fetch charges it — and performed here, on every execution, rather than
    // at lift time, because a cached block must cost what an uncached one
    // cost. Two bytes is the low halfword's width: `exec::fetch` translates
    // for that first, and translates again for the high half, which then hits.
    //
    // It happens before the budget guard because it is also what *names* the
    // block. A guard that declined afterwards would not have wasted the walk:
    // the interpreter's own fetch then finds the entry this translation just
    // filled, and charges exactly what it would have charged anyway.
    let phys = match exec.translate(pc, Access::Fetch, 2) {
        Ok(phys) => phys,
        Err(trap) => return deliver(disp, costs, exec, trap, pc, pc),
    };
    let origin = key_origin(translating, phys);
    let key = lift::key(cfg, origin, SHAPE);

    // Known unliftable, or too big for what is left of the budget: either way
    // the interpreter takes this instruction, and reaching it without a
    // dispatcher round trip is the whole point of remembering the first.
    let bound = match costs.get(pc, key) {
        Some(0) => return interpret(disp, costs, exec),
        Some(bound) => bound,
        None => worst_bound(cfg, translating),
    };
    if bound > remaining {
        return interpret(disp, costs, exec);
    }

    let mut front = Lifter {
        cfg,
        origin,
        key,
        epoch: Epoch {
            topology: space.generation(),
            // Zero, and deliberately: `Epoch::translation` is what a cache
            // keyed on the guest MMU's generation is stale against, and these
            // blocks are not keyed on it — `key_origin` puts the physical page
            // in the key instead, which is both narrower and exact. A
            // generation here would ask for a full flush on every `SRET`.
            translation: 0,
        },
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
        page: pc & !PAGE_MASK,
        base: phys & !PAGE_MASK,
        costs,
        access: per_access(cfg, translating),
        entry: if translating { WALK_ACCESSES } else { 0 },
        rejected: None,
    };

    let mut host = Host::new(&mut exec, pc);
    // One block, and one is the contract: the dispatcher has no hook between a
    // block and its successor, and every block owes an entry translation.
    let run = match disp.run(&mut front, &mut host, pc, 1) {
        Ok(run) => run,
        // The only refusal this frontend has is an RV32 configuration, which
        // the property reader rejects before a hart is built. Degrade rather
        // than fail the machine (`ROADMAP.md` §9).
        Err(_) => {
            drop(host);
            return interpret(disp, costs, exec);
        }
    };
    let Host {
        slots, trap, mark, ..
    } = host;
    debug_assert!(
        front.rejected.is_none(),
        "the RISC-V frontend emitted a block the verifier rejects: {:?}",
        front.rejected
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
        // `Budget` is the ordinary end of a one-block run. `Exit` cannot
        // happen: no safe-point flag is given to the dispatcher, because the
        // run loop above checks it at the same granularity.
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
    origin: Origin,
    key: u64,
    epoch: Epoch,
    space: &'a AddressSpace,
    attrs: MemAttrs,
    /// The virtual page the entry PC is on, and the physical page it resolved
    /// to. A block never leaves that page, so one translation covers every
    /// byte the lifter may read.
    page: u64,
    base: u64,
    costs: &'a mut Costs,
    access: u64,
    entry: u64,
    /// The first block the verifier rejected, if any. A frontend bug rather
    /// than a guest one, so it is asserted on in a debug build and ignored in
    /// a release one — the block still runs, and the differential harness is
    /// where a malformed block is supposed to be caught.
    rejected: Option<String>,
}

impl Frontend for Lifter<'_> {
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
        let space = self.space;
        let attrs = self.attrs;
        let base = self.base;
        let page = self.page;
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
        let lifted = lift::lift(self.cfg, self.origin, pc, &mut src, lift::MAX_INSNS, SHAPE)?;
        if self.rejected.is_none()
            && let Err(e) = verify(&lifted.block)
        {
            self.rejected = Some(alloc::format!("{e}"));
        }
        // Zero when nothing could be lifted, which is what sends the next
        // pass straight to the interpreter instead of back through here.
        let bound = if lifted.insns > 0 {
            block_bound(&lifted.block, self.access, self.entry)
        } else {
            0
        };
        self.costs.put(pc, self.key, bound);
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
