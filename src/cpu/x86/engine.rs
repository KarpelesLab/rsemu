//! The translated execution engine: this core's blocks, through [`jit`].
//!
//! `cpu::x86::lift` ends with a paragraph that was true for four rounds and is
//! not any more:
//!
//! > widening this world makes a 64-bit block *translatable*, and nothing
//! > more. No CPU core has a JIT execution path […] Both halves are needed,
//! > and this one is the second.
//!
//! This module is the first half. `engine = "jit"` on a `cpu.x86` object
//! reaches [`advance`], `engine = "jit-host"` attaches
//! [`jit::x86`](crate::jit::x86) under it, and `pc64`, `q35-linux`, `q35` and
//! `pc-at` all boot on either.
//!
//! [`jit`]: crate::jit
//!
//! It is `cpu::riscv::engine`'s shape, deliberately, because the claim both
//! files exist to keep is the same one — *a cache hit, a cache miss, an
//! interpreted run and a compiled run are indistinguishable to the guest,
//! including cycle counts* (`ROADMAP.md` §0). What follows is only the places
//! x86 forced a different answer, since the RISC-V file argues the rest.
//!
//! # The memory path is the interpreter's, literally
//!
//! [`IrHost::load`] and [`IrHost::store`] here call `Exec::read_mem` and
//! `Exec::write_mem` — the functions `Exec::step` itself calls — over one
//! [`Exec`] that lives for the whole of an [`advance`]. So the segment check,
//! the page-crossing split, the translation with its accessed and dirty bits,
//! the walk's tick cost and the bus transaction are one implementation rather
//! than two. The two previous rounds of this work each expected to need
//! something new from the `jit` seam and each found this was the answer
//! instead.
//!
//! # Four things x86 needed that RISC-V did not
//!
//! **1. The open bus is architectural state, and a block makes no fetches.**
//! `State::open_bus` is *in the snapshot*, so it is in `Machine::state_hash`,
//! and the interpreter writes it on every bus cycle — including the one it
//! spends fetching each instruction byte. A translated block fetches nothing,
//! so it would leave a different byte there and two engines would hash
//! differently while agreeing about every register. The rule the interpreter
//! implements, read off `Exec::instruction`, is: *after guest instruction `i`,
//! the open bus holds the top byte of `i`'s last data access if it made one,
//! and the last byte of `i`'s own encoding otherwise*. The first half a block
//! already gets right, because its accesses go through `Exec::phys_read` and
//! `Exec::phys_write`. The second half is [`close_bus`], which reads that
//! one byte back at the end of a run — and the length it needs is free:
//! `InsnStart::next_pc` is the instruction's own successor even for a taken
//! branch (`lift::Lifter::insn`), so `next_pc - pc` is the encoding length,
//! and an exit boundary is the one with `next_pc == pc`.
//!
//! **2. Every write this core makes is collected in one place.** RISC-V's
//! interpreter reports its stores through a field its `Exec` fills and its
//! blocks report theirs through [`StoreLog`]; here both are the same field,
//! because both go through `Exec::phys_write`. That is strictly more than the
//! IR can see — a task switch's stack frame, `REP MOVSB`, the accessed-bit
//! write-back of a walk — and on x86 it has to be, since the architecture
//! makes a coherent instruction cache a guarantee rather than a courtesy and
//! most of what writes code is outside the lifted subset.
//!
//! **3. A block below long mode assumes the upper halves are clean.**
//! `lift::Lifter::read_reg` reads a 32-bit operand as *the whole slot* when
//! the world is not long — "a slot holds the architectural register, so below
//! long mode it **is** the doubleword" — while `Regs::dword` truncates. Those
//! agree exactly when bits 32-63 of every general register are zero, which is
//! true from reset and stays true while `Regs::set_dword` zero-extends… and is
//! **not** true in compatibility mode, where a 64-bit kernel's dirty registers
//! are still there when a 32-bit code segment starts executing. So [`admit`]
//! checks it: a non-long world with a dirty upper half is interpreted. Eight
//! `or`s per block boundary, and the alternative is a wrong 32-bit operand
//! with nothing to report it. See [`narrow_state_is_clean`].
//!
//! **4. The world is a lift-time constant and something has to name it.**
//! `lift::World` carries `CS.base`, the six segment bases, the code segment's
//! width and the part's features, and `lift::key` folds `World::generation`
//! into the cache key **under [`Origin::Flat`] only** — a paged block is named
//! by the physical address its entry resolved to, which subsumes all of it.
//! Nothing keeps that counter, so this module does: [`Boundary::world_of`] compares
//! the world it just derived against the last one and bumps on a difference.
//! It is exact rather than conservative, and it is cheap for the reason the
//! key is arranged the way it is: in long mode `SWAPGS` moves `GS.base` on
//! every kernel entry, and under paging that moves no key at all.
//!
//! # The budget guard, and why the frontend's instruction limit is sixteen
//!
//! A block runs only when its worst case fits what is left of the caller's
//! budget, so that the guest stops on the same instruction whichever engine
//! drives it and carries the same `State::debt` into the next quantum — both
//! of which are in the snapshot the machine's state hash is taken over.
//!
//! [`Costs`] remembers a lifted block's real bound, and the *cold* answer is
//! [`worst_bound`]. On x86 that number is much larger than RISC-V's, and it
//! decides how long a block may be rather than the other way round:
//!
//! | | worst, per guest instruction |
//! | --- | --- |
//! | fetch | `bus_clocks × 15` — the longest legal encoding |
//! | the operation | 80 — `IMUL`, the largest `Op::clocks` in the lifted subset |
//! | data | two accesses, each a page-crossing eight-byte one: two walks and eight bus transactions |
//!
//! At `bus_clocks = 2` under four-level paging that is 206 ticks an
//! instruction, so [`MAX_INSNS`]`= 64` would bound a cold block at 13 200 —
//! larger than `SchedulerConfig::max_ticks_per_quantum`, which is 10 000, so
//! **no block would ever run**. Sixteen bounds it at 3 312, which fits inside
//! two thirds of a full quantum; a block that has been lifted once is admitted
//! against its real cost from then on and the limit stops mattering. It costs
//! little else: under [`Smc::EndBlock`] — the policy paging forces — a store
//! is the last instruction in its block anyway, and x86 code stores every few
//! instructions.
//!
//! # What is checked at a block boundary rather than at an instruction
//!
//! Everything `Exec::step` decides before it decodes: a pending reset, an
//! `INIT`, a Start-Up, a shutdown, a halt, the interrupt shadow, `NMI`, a
//! maskable interrupt with `IF` set, and the trap flag. [`admit`] asks all of
//! them, at **every** boundary of a chain, non-destructively — the interpreter
//! is what *takes* each one, and asking a block to run first would take it up
//! to sixteen instructions late.
//!
//! Nothing in the lifted subset can raise one from inside a block: `STI`,
//! `CLI`, `POPF`, `HLT`, `INT`, `IRET`, every segment load and every `MOV` to
//! a control register are outside it and end the block. A **store** into an
//! interrupt controller is the one thing that can, and it is seen at the very
//! next boundary — which under paging is the next instruction, because
//! [`Smc::EndBlock`] ends the block there.
//!
//! # Self-modifying code, and the gap that is left
//!
//! Bytes written by something that is **not** this core — a DMA engine, an
//! AHCI or NVMe controller filling a page cache, another processor, a
//! debugger writing a breakpoint — are outside `jit::dispatch`'s contract,
//! which is that a host accumulates the pages *it* wrote. That is the same
//! known gap `cpu::riscv::engine` states, from the same cause: closing it
//! needs a write notification from the address space for masters that are not
//! this CPU, which `core::space` does not have.
//!
//! It is worth being exact about what is *not* in that gap here, because x86
//! makes more of it than RISC-V does. A store from a compiled block, a store
//! from an interpreted instruction, a `REP MOVSB`, the stack frame an
//! exception pushes and the accessed-bit write-back of a page-table walk are
//! all collected, because all of them go through `Exec::phys_write`.
//!
//! # What it buys, measured
//!
//! See `docs/platforms/pc64.md`.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;

use crate::core::error::{BusError, Result};
use crate::core::space::{AddressSpace, MemAttrs, MemResult};
use crate::core::value::Width;
use crate::ir::{Block, InsnStart, IrHost, MemOp, Opcode, RegSlot, verify};
use crate::jit::{
    BlockCache, DirtyPages, Dispatcher, Entry, Epoch, FastMem, Frontend, PAGE_MASK, Stop, StoreLog,
    Translation,
};

use super::exec::{Exec, Fault, State};
use super::lift::{
    self, ARITH_MASK, EFLAGS_REST, FLAG_BITS, FLAG_SLOTS, Flags, Origin, RIP, SLOT_COUNT, Shape,
    Smc, World, r_slot,
};
use super::paging::Access;
use super::prot::canonical;
use super::{Config, Lines, flags, isa::seg};

/// How much of a block the frontend is allowed to swallow.
///
/// Direct branches are merged, so a loop unrolls into one translation and a
/// guest register stays in a temporary across the whole of it.
const SHAPE: Shape = Shape::Trace;

/// Whether a boundary names every flag or only the observable ones.
///
/// [`Flags::Elide`] is the frontend's default and the one the differential
/// corpus is measured on; `benches/x86_dispatch.rs` reports what it is worth.
const FLAGS: Flags = Flags::Elide;

/// How many guest instructions one block may cover.
///
/// Sixteen rather than `lift::MAX_INSNS`'s sixty-four, and the reason is the
/// budget guard rather than the frontend — see the module docs.
const MAX_INSNS: usize = 16;

/// How many blocks one [`advance`] may chain before it hands control back.
///
/// Sixteen, as on the other core, and for the same reason: what chaining buys
/// is not the hash lookup it skips but everything around a short block that a
/// one-block call pays in full — an `Exec`, a `Host` and its slot copy in and
/// out, a lifter, a cache resynchronisation and a trip through
/// `X86::run_budget`.
///
/// x86 has no safe-point flag to delay: `X86` never held an
/// [`ExitFlag`](crate::core::sched::ExitFlag) and `Device::run` still stops
/// only at the end of its budget, so unlike on the RISC-V core this bound
/// costs nothing that was previously bounded more tightly.
const CHAIN: usize = 16;

/// How many blocks this core's cache holds before it evicts.
///
/// `jit::BlockCache`'s own default is 8 192 and a Linux guest wants more; the
/// number is a bound rather than an allocation, so a board whose guest has a
/// small working set never fills it.
const BLOCKS: usize = 65536;

/// How many `(pc, key) -> worst-case ticks` answers are remembered.
const COST_SLOTS: usize = 65536;

/// The most clocks any operation in the lifted subset charges statically.
///
/// `IMUL` (`Op::clocks`), which is the largest; `DIV` and `IDIV` are bigger
/// and are out of the subset.
const MAX_OP_CLOCKS: u64 = 80;

/// The longest legal x86 encoding. A 386 raises `#GP` for anything longer, and
/// [`lift`] ends the block before it.
const MAX_INSN_BYTES: u64 = 15;

/// The deepest page-table walk this core makes: IA-32e's four levels.
///
/// Used only to bound a block's worst case, so a two- or three-level walk
/// makes the bound conservative rather than wrong.
const WALK_LEVELS: u64 = 4;

/// How big a host code buffer this core asks for: 256 MiB.
///
/// The same number `cpu::riscv::engine` argues for, and the argument carries:
/// the buffer is append-only and reclaimed only by a reset that throws every
/// compiled block away, an x86 block lifts to more IR than an RV64 one rather
/// than less, and `jit::x86::buf` flips a page-sized window rather than the
/// whole mapping so a larger buffer costs address space and nothing per
/// compile.
#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
const CODE_BUFFER: u64 = 256 << 20;

// ---------------------------------------------------------------------------
// What a core keeps between blocks
// ---------------------------------------------------------------------------

/// This core's translation state.
///
/// **Derived state in the strict sense** (`ROADMAP.md` §4.5): never
/// serialized, thrown away by a reset and by a snapshot restore. That is also
/// what makes a snapshot interchangeable between any two engines and with
/// `accel::state` — there is nothing engine-specific in one to interchange.
#[derive(Debug)]
pub(super) struct Jit {
    disp: Dispatcher,
    /// Everything a *boundary* consults, kept apart from the dispatcher
    /// because `Dispatcher::run` borrows itself for the length of a chain and
    /// the frontend it drives has to reach these at every block of one.
    at: Boundary,
}

/// What a JIT core has been asked to do.
///
/// **The honest headline is `retired` against `interpreted`**, not the
/// speedup: it is the fraction of the guest's own instructions that this
/// engine executed as compiled code rather than handing back, and a lifted
/// subset with real exclusions (`cpu::x86::lift`, "The subset, exactly") is
/// worth exactly that fraction. Every one of these is a statistic and never a
/// behaviour — the engines are indistinguishable to the guest — but a
/// mechanism whose coverage is unmeasured is a mechanism whose coverage rots.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct Stats {
    /// Blocks executed.
    pub blocks: u64,
    /// Of those, the ones that ran as host code rather than through the IR
    /// interpreter. Zero under `engine = "jit"`, by construction.
    pub compiled: u64,
    /// Of those, the ones reached by following a patched exit with no lookup.
    pub chained: u64,
    /// Blocks translated — one per distinct `(pc, key)` that survived.
    pub translated: u64,
    /// Blocks thrown away because the guest wrote into the page they came
    /// from.
    pub invalidated: u64,
    /// Guest instructions **retired inside a block**.
    pub retired: u64,
    /// Guest instructions the interpreter took, because a block would have
    /// been wrong: an encoding outside the lifted subset, a world the frontend
    /// refuses, a pending interrupt, or a worst case that did not fit what was
    /// left of the budget.
    pub interpreted: u64,
}

/// What deciding whether a block may run needs, and the dispatcher does not.
#[derive(Debug)]
struct Boundary {
    costs: Costs,
    /// Guest instructions retired inside a block, and taken by the
    /// interpreter. See [`Stats`].
    retired: u64,
    interpreted: u64,
    /// The last world derived, with its generation and origin normalised away,
    /// and the counter that names it. See the module docs.
    seen: Option<World>,
    generation: u64,
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
            at: Boundary {
                costs: Costs::new(),
                retired: 0,
                interpreted: 0,
                seen: None,
                generation: 0,
            },
        }
    }

    /// Throw every translation away.
    pub(super) fn flush(&mut self) {
        self.disp.cache_mut().flush();
        self.at.costs.clear();
        // The world counter is *not* reset. It only ever has to separate two
        // worlds that could otherwise key alike, and a monotonic counter does
        // that whether or not the cache behind it was emptied; restarting it
        // at zero after a flush would be the one way to make two different
        // worlds share a key.
        self.at.seen = None;
    }

    /// What this engine has been asked to do.
    pub(super) fn stats(&self) -> Stats {
        let s = self.disp.stats();
        Stats {
            blocks: s.blocks,
            compiled: s.compiled,
            chained: s.chained,
            translated: s.translated,
            invalidated: s.smc,
            retired: self.at.retired,
            interpreted: self.at.interpreted,
        }
    }

    /// Hand what an interpreted instruction wrote to the block cache.
    ///
    /// `X86::step` is the entry point that interprets one instruction without
    /// going through [`advance`] — a monitor stepping, a test driving the core
    /// by hand — and a store it makes into a translated page has to be honoured
    /// exactly as one from the run loop is.
    pub(super) fn note_writes(&mut self, exec: &mut Exec<'_>) {
        let Jit { disp, at } = self;
        drain(disp, at, exec);
    }
}

impl Boundary {
    /// The world `state` is in, named by a counter this engine keeps.
    ///
    /// `None` when [`World::of`] refuses the processor's current mode, which
    /// is the ordinary answer in real mode, in virtual-8086 mode, on a 386 or
    /// 486 with paging on, and with the A20 gate shut.
    fn world_of(
        &mut self,
        state: &State,
        cfg: &Config,
        lines: &Lines,
        origin: Origin,
    ) -> Option<World> {
        let a20_open = lines.a20_mask() == u32::MAX;
        let mut world = World::of(
            &state.regs,
            &state.sys,
            cfg,
            a20_open,
            self.generation,
            origin,
        )?;
        // The generation names everything in `World` that `lift::key` does not
        // spell out, and it is bumped by comparing rather than by guessing
        // which register write could have moved one. Both fields that are not
        // part of the comparison are normalised out: the origin is in the key
        // on its own, and the generation is what is being decided.
        let mut bare = world;
        bare.generation = 0;
        bare.origin = Origin::Flat;
        if self.seen != Some(bare) {
            self.generation = self.generation.wrapping_add(1);
            self.seen = Some(bare);
            world.generation = self.generation;
        }
        Some(world)
    }
}

/// A direct-mapped table of `(pc, key) -> the most ticks that block can spend`,
/// with zero meaning *there is no block here*.
///
/// Two jobs, both about not paying for the same answer twice — the budget
/// guard needs an upper bound *before* a block runs, and computing one means
/// walking the block's ops; and a recorded **zero** says the instruction at
/// that PC is outside the lifted subset, so the interpreter should be reached
/// without a dispatcher round trip and a lift that fails at its first
/// instruction. On x86 that second job matters more than on RISC-V, not less:
/// `REP MOVSB`, `IRET`, `INT`, every `MOV` to a control register, every
/// segment load and the whole of SSE are outside the subset, and a kernel is
/// full of them.
///
/// A collision loses an answer and costs a conservative bound, never a wrong
/// one — [`worst_bound`] is what a miss falls back to, and it is a true upper
/// bound on any block this engine will lift.
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

    /// x86 instructions are byte-aligned, so every bit of the PC carries.
    #[inline]
    fn index(pc: u64) -> usize {
        (pc as usize) & (COST_SLOTS - 1)
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

// ---------------------------------------------------------------------------
// What a block can cost
// ---------------------------------------------------------------------------

/// What one guest data access can cost this core, at worst.
///
/// With `CR0.PG` clear an access of any width is **one** bus transaction:
/// `Exec::linear_read` only splits when paging is on, and `Exec::phys_read`
/// charges once for one, two, four or eight bytes.
///
/// With it set the worst case is an eight-byte access crossing a page
/// boundary: `Exec::linear_write` translates every byte, so two of them can
/// miss and walk — but only two, because there are only two pages — and then
/// eight one-byte transactions reach the bus. A walk is at most
/// [`WALK_LEVELS`] descriptor reads plus that many accessed/dirty write-backs.
const fn per_access(bus: u64, paged: bool) -> u64 {
    if paged {
        2 * (2 * WALK_LEVELS) * bus + 8 * bus
    } else {
        bus
    }
}

/// What the entry fetch translation can cost, which is nothing with paging off.
const fn entry_cost(bus: u64, paged: bool) -> u64 {
    if paged { 2 * WALK_LEVELS * bus } else { 0 }
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

/// The bound for a block nothing is known about.
///
/// [`MAX_INSNS`] instructions, each the longest legal encoding, each the
/// slowest operation in the subset, each making the two data accesses the
/// widest form in the subset can make.
const fn worst_bound(bus: u64, paged: bool) -> u64 {
    let per_insn = bus * MAX_INSN_BYTES + MAX_OP_CLOCKS + 2 * per_access(bus, paged);
    entry_cost(bus, paged) + MAX_INSNS as u64 * per_insn
}

/// Whether every general register's upper half — and the program counter's —
/// is zero.
///
/// The precondition `lift::Lifter::read_reg` has below long mode, checked
/// rather than assumed: it reads a 32-bit operand as *the whole slot* there
/// ("a slot holds the architectural register, so below long mode it **is** the
/// doubleword"), while `Regs::dword` truncates. The two agree exactly when
/// bits 32-63 are zero. Only registers 0-7 can be named without a `REX` prefix
/// and only long mode has one, but all sixteen are tested because the cost is
/// the same and the claim is then about the slot file rather than about the
/// decoder.
///
/// The **program counter** is in for the same reason one step along: `lift`
/// masks it with `World::addr_mask`, which is thirty-two bits below long mode,
/// and `Exec::fetch_byte` preserves whatever sits above that instead. Clean
/// from reset and kept clean by `Regs::set_dword`, which zero-extends —
/// compatibility mode is where it stops being true, and there a 64-bit
/// kernel's leftovers are still in the file when a 32-bit code segment starts
/// executing.
#[inline]
fn narrow_state_is_clean(state: &State) -> bool {
    let mut all = state.regs.rip;
    for n in 0..16u8 {
        all |= state.regs.qword(n);
    }
    all >> 32 == 0
}

// ---------------------------------------------------------------------------
// Admitting a block
// ---------------------------------------------------------------------------

/// What entering a block resolved to, once it is going to run.
#[derive(Debug, Clone, Copy)]
struct Admitted {
    world: World,
    key: u64,
    /// The linear page the block is bounded by, and the physical frame the
    /// entry translation resolved it to. Equal with `CR0.PG` clear.
    linear_page: u64,
    frame: u64,
    /// What one guest data access costs at worst, and what the entry fetch
    /// translation costs at worst.
    access: u64,
    entry: u64,
}

/// Whether a block may run at `pc`, and what it costs to find out.
#[derive(Debug)]
enum Admit {
    /// It may.
    Ready(Box<Admitted>),
    /// It may not, and the reason is one the interpreter answers.
    Interpret,
}

/// Everything a block owes before it runs — for the first block of a run and
/// for every chained successor alike.
///
/// The order is load-bearing and is the RISC-V engine's: **the events the
/// interpreter takes before it decodes**, then **the entry fetch translation**,
/// then **the budget guard**.
///
/// The middle one is the contract `lift`'s module docs call *"the one that
/// looks like a working JIT"*: a cached block makes no fetches, but the
/// instruction it replaced translated its first byte through the fetch path,
/// walking the tables on a buffer miss and charging for the walk. Doing it on
/// every execution rather than once at lift time is what makes a served block
/// cost what an uncached one cost — and a **failed** translation is rolled
/// back rather than charged, because the block then does not run and the
/// interpreter's own fetch is about to walk those same tables. A successful
/// one is different in exactly the way that matters: it filled the buffer, so
/// the interpreter's fetch finds the entry and charges nothing.
fn admit(at: &mut Boundary, exec: &mut Exec<'_>, pc: u64, remaining: u64) -> Admit {
    // Everything `Exec::step` decides before it decodes, asked without taking
    // any of them: each is the interpreter's to take, and a block run first
    // would take it up to `MAX_INSNS` instructions late.
    let st = &*exec.state;
    if st.reset_pending
        || st.shutdown
        || st.halted
        || st.wait_for_sipi
        || st.int_shadow
        || st.regs.eflags & flags::TF != 0
    {
        return Admit::Interpret;
    }
    let lines = exec.lines;
    if lines.init_latched()
        || lines.init_held()
        || lines.startup_pending().is_some()
        || lines.nmi_pending()
        || (st.regs.eflags & flags::IF != 0 && lines.intr_pending())
    {
        return Admit::Interpret;
    }

    let paged = st.sys.paging();
    let origin = if paged {
        // A placeholder the entry translation replaces below. `World::of`
        // refuses an origin that disagrees with `CR0.PG`, which is what makes
        // the pair unstatable rather than merely discouraged.
        Origin::Paged { phys: 0 }
    } else {
        Origin::Flat
    };
    let cfg = exec.cfg;
    let Some(mut world) = at.world_of(st, cfg, lines, origin) else {
        return Admit::Interpret;
    };
    if !world.long() && !narrow_state_is_clean(st) {
        return Admit::Interpret;
    }

    // `Exec::fetch_at` checks this before it translates anything, and it is
    // `#GP` rather than `#PF`, so the interpreter has to be the one to raise
    // it. Below long mode `World::of` has already insisted on a flat 4 GiB
    // code segment, which discharges the limit check for every offset at once.
    let linear = world.linear(pc);
    if world.long() && !canonical(linear) {
        return Admit::Interpret;
    }

    let frame = if paged {
        let user = exec.cpl() == 3;
        let before = exec.state.cycles;
        let spent_before = exec.used;
        match exec.translate_access(linear, Access::fetch(user)) {
            Ok(phys) => {
                world.origin = Origin::Paged { phys };
                phys & !PAGE_MASK
            }
            Err(_) => {
                // Not charged: the block does not run, the interpreter walks
                // these same tables itself, and a walk that faults filled
                // nothing to make the second one free. `CR2` is latched twice
                // with the same value, which the interpreter is about to do
                // anyway.
                exec.state.cycles = before;
                exec.used = spent_before;
                return Admit::Interpret;
            }
        }
    } else {
        linear & !PAGE_MASK
    };

    let smc = if paged { Smc::EndBlock } else { Smc::Guard };
    let key = lift::key(&world, SHAPE, smc, FLAGS);
    let bus = u64::from(cfg.variant.bus_clocks());
    let bound = match at.costs.get(pc, key) {
        // Known unliftable: the interpreter takes this instruction, and
        // reaching it without a dispatcher round trip and a lift that fails at
        // its first instruction is the whole point of remembering it.
        Some(0) => return Admit::Interpret,
        Some(bound) => bound,
        None => worst_bound(bus, paged),
    };
    if bound > remaining.saturating_sub(exec.used) {
        return Admit::Interpret;
    }

    Admit::Ready(Box::new(Admitted {
        world,
        key,
        linear_page: linear & !PAGE_MASK,
        frame,
        access: per_access(bus, paged),
        entry: entry_cost(bus, paged),
    }))
}

// ---------------------------------------------------------------------------
// One step of the run loop
// ---------------------------------------------------------------------------

/// Execute a chain of blocks, or — where a block would be wrong — one
/// interpreted instruction.
///
/// Reports the clocks charged, in the same currency and with the same meaning
/// as [`X86::step`](super::X86::step), so a run loop cannot tell which engine
/// it is driving: zero means *stop*, and it is returned only where the
/// interpreter would return it.
///
/// `remaining` is what is left of the caller's budget, and it is not advisory
/// — see the module docs.
///
/// # Panics
///
/// If a lifted block reaches an op the IR backend does not implement. That is
/// this crate's own frontend emitting something its own backend cannot
/// execute, and the architectural state at that point is not reconstructible.
pub(super) fn advance(
    jit: &mut Jit,
    state: &mut State,
    mem: &Arc<AddressSpace>,
    io: Option<&AddressSpace>,
    cfg: &Config,
    lines: &Lines,
    remaining: u64,
) -> u64 {
    let Jit { disp, at: bound } = jit;
    let mut exec = Exec::new(state, mem, io, cfg, lines);
    let pc = exec.state.regs.rip;

    // The entry work for the *first* block, done here rather than through
    // `Frontend::enter`, because the overwhelmingly common answer on a real
    // guest is "not a block at all" and reaching the interpreter for one
    // should not cost a frontend, a host and a dispatcher round trip. The
    // dispatcher's first `enter` is then a no-op; see `Lifter::admitted`.
    let admitted = match admit(bound, &mut exec, pc, remaining) {
        Admit::Ready(at) => *at,
        Admit::Interpret => return interpret(disp, bound, exec),
    };

    let mut front = Lifter {
        at: admitted,
        space: mem,
        // Lifting reads *ahead* of the guest: up to sixteen instructions it
        // has not asked for. A fetch is an ordinary access and a read-ahead is
        // not, so this is the one place in the core that reads guest memory
        // the way a debugger does -- CLAUDE.md's "a debugger read must not pop
        // a FIFO" is exactly the hazard, and a NOR bank in its command state
        // is exactly the device. Nothing about the *translation* is relaxed:
        // that happened in `admit`, through the fetch path, with its walk and
        // its accessed bit.
        attrs: MemAttrs::DEBUG.with_requester(cfg.requester),
        bound,
        remaining,
        admitted: true,
        rejected: None,
    };
    let mut host = Host::new(&mut exec, pc, &admitted);
    let run = match disp.run(&mut front, &mut host, pc, CHAIN) {
        Ok(run) => run,
        // Nothing this frontend refuses is reachable from a world `World::of`
        // accepted -- `lift` errors only on a zero instruction limit and on
        // the in-block guard under paging, and `admit` chooses neither.
        // Degrade rather than fail the machine (`ROADMAP.md` section 9).
        Err(_) => {
            drop(host);
            drop(front);
            return interpret(disp, bound, exec);
        }
    };
    let Host {
        slots,
        fault,
        end,
        end_access,
        cur_end,
        cur_access,
        frame,
        world,
        overflowed,
        ..
    } = host;
    let Lifter {
        rejected, bound, ..
    } = front;
    debug_assert!(
        rejected.is_none(),
        "the x86 frontend emitted a block the verifier rejects: {rejected:?}"
    );
    if overflowed {
        // One instruction wrote more distinct pages than `Exec::wrote` holds,
        // so the list stopped being the whole truth. See [`drain`], which
        // argues why that is unreachable and why a full flush is the answer if
        // it ever is not.
        disp.cache_mut().flush();
        bound.costs.clear();
    }

    bound.retired = bound.retired.wrapping_add(run.insns as u64);

    if run.blocks == 0 {
        // Nothing executed: the instruction at `pc` is outside the lifted
        // subset, and `Frontend::translate` has just recorded that so the next
        // pass skips straight to the interpreter. Its own fetch translation
        // now hits the buffer `admit` filled, so what it charges is what a
        // purely interpreted core would have charged.
        return interpret(disp, bound, exec);
    }

    match run.stop {
        Stop::Fault(f) => {
            // The block stopped *at* the faulting instruction with the state
            // that instruction should see, which is what the IR's lazy
            // publication gives and what `differential::compare` asserts. The
            // interpreter's own fault path then takes over verbatim -- and the
            // register file it rolls back to a pre-instruction snapshot is
            // exactly what publishing this boundary's map has just produced.
            publish(exec.state, &world, &slots, f.pc);
            close_bus(&mut exec, &world, frame, cur_end, cur_access);
            let fault = fault.unwrap_or_else(|| Fault::gp(0));
            exec.entry = exec.state.regs;
            exec.state.queue.flush();
            exec.deliver(fault);
            let used = exec.used;
            drain(disp, bound, &mut exec);
            used.max(1)
        }
        Stop::Unsupported { op, at } => panic!(
            "the x86 frontend emitted {op} at index {at}, which the IR backend cannot execute"
        ),
        // `Budget` ends a full chain, `Declined` a short one, `Untranslatable`
        // a boundary whose instruction is outside the subset, and all three
        // leave the guest at `run.pc` for the run loop to pick up. `Exit`
        // cannot happen: no safe-point flag is given to the dispatcher.
        _ => {
            publish(exec.state, &world, &slots, run.pc);
            close_bus(&mut exec, &world, frame, end, end_access);
            let used = exec.used;
            drain(disp, bound, &mut exec);
            used.max(1)
        }
    }
}

/// Interpret one instruction, and tell the block cache what it wrote.
fn interpret(disp: &mut Dispatcher, bound: &mut Boundary, mut exec: Exec<'_>) -> u64 {
    let used = exec.step();
    bound.interpreted = bound.interpreted.wrapping_add(1);
    drain(disp, bound, &mut exec);
    used
}

/// Write the slot file back into the architectural register file.
///
/// `EFLAGS` is reassembled from the seven slots that hold it rather than
/// normalised: the rest slot is `eflags & !ARITH_MASK` and the six flags are
/// the bits it is missing, so the two halves reproduce exactly the word the
/// block started from wherever nothing wrote one.
fn publish(state: &mut State, world: &World, slots: &[u64; SLOT_COUNT as usize], pc: u64) {
    for n in 0..16u8 {
        state.regs.set_qword(n, slots[r_slot(n).0 as usize]);
    }
    state.regs.rip = pc & world.addr_mask();
    let mut eflags = slots[EFLAGS_REST.0 as usize] as u32;
    for (i, bit) in FLAG_BITS.iter().enumerate() {
        if slots[FLAG_SLOTS[i].0 as usize] & 1 != 0 {
            eflags |= bit;
        }
    }
    state.regs.eflags = eflags;
}

/// Leave the open bus holding what the interpreter would have left there.
///
/// A block makes no instruction fetches, and on this core every fetch is a bus
/// cycle that latches its byte (`Exec::fetch_at` through `Exec::phys_read`).
/// So after a guest instruction that made no data access the interpreter's
/// open bus holds the **last byte of that instruction's own encoding**, and a
/// block would have left whatever its previous access did. `State::open_bus`
/// is in the snapshot, so that is a state-hash divergence rather than a
/// curiosity.
///
/// `end` is the linear address of the last retired instruction's last byte,
/// tracked in [`Host::insn_start`]; `access` says whether that instruction
/// made a data access, in which case the bus already holds the right byte and
/// nothing is read here.
///
/// The read is a debug read of the physical byte and is deliberately **not**
/// charged: it stands in for a bus cycle whose clocks the block's own
/// `CHARGE` already paid for.
fn close_bus(exec: &mut Exec<'_>, world: &World, frame: u64, end: Option<u64>, access: bool) {
    let Some(end) = end.filter(|_| !access) else {
        return;
    };
    let linear = world.linear(end);
    let phys = match world.origin {
        Origin::Flat => linear,
        Origin::Paged { .. } => frame | (linear & PAGE_MASK),
    };
    let attrs = MemAttrs::DEBUG.with_requester(exec.cfg.requester);
    if let Ok(byte) = exec.mem.read(phys, Width::U8, attrs) {
        exec.state.open_bus = byte as u8;
    }
}

/// Hand what this core wrote to the block cache.
///
/// One drain for both engines, because `Exec::phys_write` is one funnel: a
/// compiled block's store, an interpreted instruction's, the stack frame an
/// exception pushes and a walk's accessed-bit write-back all arrive here.
fn drain(disp: &mut Dispatcher, bound: &mut Boundary, exec: &mut Exec<'_>) {
    let mut hit = 0usize;
    if core::mem::take(&mut exec.wrote_over) {
        // More distinct pages than the list holds, so it is no longer the
        // whole truth. Unreachable rather than merely unlikely: the widest
        // thing one x86 instruction can do is two page-crossing accesses under
        // a four-level walk, which is ten distinct pages against a list of
        // twenty-four, and the list is emptied after every store and after
        // every interpreted instruction. A full flush is the cheap sound
        // answer if it ever stops being unreachable.
        disp.cache_mut().flush();
        hit += 1;
    }
    for i in 0..exec.wrote_n as usize {
        hit += disp.cache_mut().note_write(exec.wrote[i], 1);
    }
    exec.wrote_n = 0;
    if hit > 0 {
        // A page a translation came from has changed, so every *negative*
        // answer in the cost table may have changed with it: an instruction
        // that was outside the subset can have been overwritten by one that is
        // not.
        bound.costs.clear();
    }
}

// ---------------------------------------------------------------------------
// The frontend
// ---------------------------------------------------------------------------

/// The x86 half of the dispatcher's contract, over a real core.
struct Lifter<'a> {
    /// What the current block's entry resolved to — replaced at every boundary
    /// by [`Lifter::enter`], because a chained successor is on its own page,
    /// under its own key.
    at: Admitted,
    space: &'a AddressSpace,
    attrs: MemAttrs,
    /// The cost table and the world counter, which a chained boundary consults
    /// through [`admit`] exactly as [`advance`]'s prologue does.
    bound: &'a mut Boundary,
    /// What [`advance`] was given, so a chained boundary guards the next block
    /// against what the chain has *left* rather than against the whole.
    remaining: u64,
    /// Whether [`advance`]'s prologue has already admitted the entry PC, so
    /// the dispatcher's first `enter` neither translates nor charges twice.
    admitted: bool,
    /// The first block the verifier rejected, if any. A frontend bug rather
    /// than a guest one, asserted on in a debug build and ignored in a release
    /// one - the block still runs, and the differential harness is where a
    /// malformed block is supposed to be caught.
    rejected: Option<String>,
}

impl<'h, 'e> Frontend<Host<'h, 'e>> for Lifter<'_> {
    fn epoch(&mut self) -> Epoch {
        Epoch {
            // Read live, at every boundary: a chained successor must not be
            // served out of a cache lifted through a topology a store in the
            // block before it replaced. One relaxed atomic load.
            topology: self.space.generation(),
            // Zero, and deliberately. A paged block is keyed on the physical
            // address its entry resolved to and a flat one on `World::generation`,
            // both of which are in `Block::key`; a counter here would ask for a
            // full flush on every `INVLPG`.
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
        // of them — it reads the system registers, the interrupt pins, the
        // translation buffer and the tick counter — so a chained boundary sees
        // the same world a fresh `advance` would have seen.
        //
        // Except the one thing that *is* a slot: the upper halves below long
        // mode, which `advance` checked against the register file. Nothing in
        // the lifted subset can dirty one in a world where the check passed —
        // a value that becomes a register is masked to the operand's width and
        // a 32-bit world has no 64-bit operand — so re-reading the file here
        // asks the same question of the same answer.
        match admit(self.bound, host.exec, pc, self.remaining) {
            Admit::Ready(at) => {
                self.at = *at;
                // The host follows the block across a page: [`close_bus`] reads
                // the last retired instruction's last byte back through the
                // frame of whichever block retired it.
                host.frame = self.at.frame;
                host.world = self.at.world;
                Ok(Entry::Ready)
            }
            Admit::Interpret => Ok(Entry::Leave),
        }
    }

    fn key(&mut self) -> u64 {
        self.at.key
    }

    fn pc_slot(&self) -> RegSlot {
        RIP
    }

    fn translate(&mut self, pc: u64) -> Result<Translation> {
        // Out of guest memory rather than out of anything cached, because a
        // store that rewrote an instruction has to be visible here. The lifter
        // reads **linear** addresses and a block never leaves the page its
        // entry is on, so under paging the one translation `admit` just made
        // covers every byte it may read: the offset within the page is carried
        // and the frame comes from the entry.
        let space = self.space;
        let attrs = self.attrs;
        let page = self.at.linear_page;
        let frame = self.at.frame;
        let paged = self.at.world.origin.paged();
        let mut src = |addr: u64| {
            if addr & !PAGE_MASK != page {
                return None;
            }
            let at = if paged {
                frame | (addr & PAGE_MASK)
            } else {
                addr
            };
            space.read(at, Width::U8, attrs).ok().map(|v| v as u8)
        };
        let smc = if paged { Smc::EndBlock } else { Smc::Guard };
        let lifted = lift::lift(&self.at.world, pc, &mut src, MAX_INSNS, SHAPE, smc, FLAGS)?;
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
        self.bound.costs.put(pc, self.at.key, bound);
        Ok(Translation {
            page: lifted.page,
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
    slots: [u64; SLOT_COUNT as usize],
    /// The x86 fault the memory path raised, kept because [`IrHost::load`] can
    /// only report a [`BusError`] and an x86 fault is a vector *and* an error
    /// code — and `#SS` through the stack is a different vector from `#GP`
    /// through everything else.
    fault: Option<Fault>,
    dirty: DirtyPages,
    /// The world and the physical frame of the block currently running, so
    /// [`close_bus`] can read the last instruction's last byte back.
    world: World,
    frame: u64,
    /// The last byte of the instruction whose boundary is open, and whether it
    /// has made a data access. See [`close_bus`].
    cur_end: Option<u64>,
    cur_access: bool,
    /// The same pair for the last instruction that **retired**.
    end: Option<u64>,
    end_access: bool,
    /// Whether a write overran `Exec::wrote`, so the dirty log is incomplete
    /// and [`advance`] has to throw every translation away. See [`drain`].
    overflowed: bool,
}

impl<'a, 'e> Host<'a, 'e> {
    fn new(exec: &'a mut Exec<'e>, pc: u64, at: &Admitted) -> Host<'a, 'e> {
        let mut slots = [0u64; SLOT_COUNT as usize];
        for n in 0..16u8 {
            slots[r_slot(n).0 as usize] = exec.state.regs.qword(n);
        }
        slots[RIP.0 as usize] = pc;
        let eflags = exec.state.regs.eflags;
        for (i, bit) in FLAG_BITS.iter().enumerate() {
            slots[FLAG_SLOTS[i].0 as usize] = u64::from(eflags & bit != 0);
        }
        slots[EFLAGS_REST.0 as usize] = u64::from(eflags & !ARITH_MASK);
        Host {
            exec,
            slots,
            fault: None,
            dirty: DirtyPages::new(),
            world: at.world,
            frame: at.frame,
            cur_end: None,
            cur_access: false,
            end: None,
            end_access: false,
            overflowed: false,
        }
    }

    /// Whether the access that just returned put anything on the bus, read off
    /// the clock rather than off the call.
    ///
    /// [`close_bus`] needs *"did this guest instruction drive the bus after
    /// its own fetch"*, and "it made an access" is not that question: a
    /// segment-limit violation is checked before anything is charged, so a
    /// faulting `mov [esi], eax` puts nothing on the bus and the byte left
    /// there is the last one the instruction's own encoding was fetched from.
    /// A `#PF`, by contrast, reads page-table descriptors on the way to
    /// failing, and those *are* bus cycles that latch their bytes. One
    /// comparison against `Exec::used` distinguishes them exactly, because
    /// every bus transaction on this core charges through `Exec::charge` and
    /// nothing else in an access does.
    #[inline]
    fn spent_a_cycle(&mut self, before: u64) {
        self.cur_access |= self.exec.used != before;
    }

    /// Report an x86 fault as the bus error the IR speaks, keeping the vector.
    fn raise(&mut self, fault: Fault) -> BusError {
        self.fault = Some(fault);
        BusError::Protected
    }

    /// Move whatever this core has written into the dirty log.
    ///
    /// Whatever landed, landed: a store that crossed a page boundary and
    /// faulted on the second page still wrote the first, and a translation of
    /// those bytes is stale either way.
    fn note_writes(&mut self) {
        for i in 0..self.exec.wrote_n as usize {
            self.dirty.note(self.exec.wrote[i], 1);
        }
        self.exec.wrote_n = 0;
        self.overflowed |= core::mem::take(&mut self.exec.wrote_over);
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
        let sr = mem.seg.map_or(seg::DS, |s| s.0);
        let before = self.exec.used;
        let done = self.exec.read_mem(sr, addr, mem.size.bytes() as u8);
        self.spent_a_cycle(before);
        match done {
            Ok(v) => Ok(v),
            Err(fault) => Err(self.raise(fault)),
        }
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        let sr = mem.seg.map_or(seg::DS, |s| s.0);
        let before = self.exec.used;
        let done = self.exec.write_mem(sr, addr, mem.size.bytes() as u8, value);
        self.spent_a_cycle(before);
        self.note_writes();
        match done {
            Ok(()) => Ok(()),
            Err(fault) => Err(self.raise(fault)),
        }
    }

    fn charge(&mut self, ticks: u64) {
        // One call rather than a loop: `Exec::charge` takes a clock count, and
        // the block's static column is already that count.
        self.exec.charge(ticks as u32);
    }

    fn insn_start(&mut self, mark: &InsnStart) {
        // The instruction whose boundary was open has just finished.
        self.end = self.cur_end;
        self.end_access = self.cur_access;
        self.cur_access = false;
        // An **exit** boundary begins no guest instruction and says so by
        // carrying `next_pc == pc`; every instruction boundary carries its own
        // successor, taken branch or not, so the difference is the encoding
        // length (`lift::Lifter::insn`).
        let len = mark.next_pc.wrapping_sub(mark.pc);
        self.cur_end = (len != 0).then(|| mark.pc.wrapping_add(len - 1));
    }
}

impl StoreLog for Host<'_, '_> {
    fn drain_dirty(&mut self, sink: &mut dyn FnMut(u64)) {
        self.dirty.drain_dirty(sink);
    }
}

/// **x86 publishes no inlined load path, and that is a property of the guest.**
///
/// A load's address here is an *effective* address: the segment base is added
/// and the limit checked before anything reaches a table, and the frontend
/// says so by giving every [`MemOp`] a `SegId`. The backend refuses to inline
/// a segmented access for the same reason (`jit::x86`'s `Compiler::inlinable`),
/// so this is the same answer said twice. Inlining x86's loads means lowering
/// the segment fold into generated code — a base add and a limit compare
/// against state a `MOV DS, ax` can change between two instructions — and that
/// is a frontend change rather than a wiring one.
impl FastMem for Host<'_, '_> {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::RamStore;
    use crate::cpu::x86::differential::{self, Case};
    use crate::cpu::x86::{Engine, X86};

    /// A core on `space`, in the world `case` describes, running on `engine`.
    fn core(case: &Case, space: Arc<AddressSpace>, engine: Engine) -> X86 {
        differential::oracle(case, space).with_engine(engine)
    }

    /// The byte the open bus is holding — architectural state that is in the
    /// snapshot and therefore in the machine's state hash, and the one column
    /// a translated block gets wrong for free because it makes no fetches.
    fn open_bus(cpu: &X86) -> u8 {
        cpu.session.lock().state.open_bus
    }

    /// Every column a guest, a snapshot or a state hash can see, compared
    /// between an interpreted core and a translated one.
    ///
    /// `quanta` budgets of `budget` clocks each, handed out exactly as the
    /// scheduler hands them out, so the *stopping point* is compared as well
    /// as the arithmetic: a block that ran past a budget where an instruction
    /// would not have puts the two cores on different instructions for the
    /// rest of the run, and `State::debt` is what records that.
    fn agree_on(case: &Case, engine: Engine, budget: u64, quanta: usize) -> X86 {
        let (space_a, ram_a) = differential::machine(case);
        let (space_b, ram_b) = differential::machine(case);
        let interp = core(case, space_a, Engine::Interp);
        let jit = core(case, space_b, engine);
        for n in 0..quanta {
            let a = interp.run_budget(budget);
            let b = jit.run_budget(budget);
            assert_eq!(
                a, b,
                "quantum {n}: {engine:?} and the interpreter consumed different budgets"
            );
            let want = interp.regs();
            let got = jit.regs();
            for r in 0..16u8 {
                assert_eq!(
                    want.qword(r),
                    got.qword(r),
                    "quantum {n}, register {r} under {engine:?}: the interpreter says \
                     {:#018x}, the JIT says {:#018x}",
                    want.qword(r),
                    got.qword(r),
                );
            }
            assert_eq!(want.rip, got.rip, "quantum {n}: the program counter");
            assert_eq!(want.eflags, got.eflags, "quantum {n}: EFLAGS");
            assert_eq!(
                interp.cycles(),
                jit.cycles(),
                "quantum {n}: the cycle counter. A compiled block must charge exactly \
                 what an interpreted one charges (ROADMAP.md §0)"
            );
            assert_eq!(
                interp.cycle_debt(),
                jit.cycle_debt(),
                "quantum {n}: the carried overrun"
            );
            assert_eq!(
                interp.is_halted(),
                jit.is_halted(),
                "quantum {n}: whether the core stopped"
            );
            assert_eq!(
                open_bus(&interp),
                open_bus(&jit),
                "quantum {n}: the open bus. It is in the snapshot, so it is in the \
                 machine's state hash — and a block makes no instruction fetches, \
                 which is what `close_bus` is for"
            );
            memory_agrees(&ram_a, &ram_b, n);
        }
        jit
    }

    fn memory_agrees(want: &Arc<RamStore>, got: &Arc<RamStore>, quantum: usize) {
        let len = want.len();
        assert_eq!(len, got.len());
        for at in 0..len {
            let a = want.read_u8(at).expect("in range");
            let b = got.read_u8(at).expect("in range");
            assert_eq!(
                a, b,
                "quantum {quantum}: guest RAM differs at {at:#x}: {a:#04x} against {b:#04x}"
            );
        }
    }

    /// Both JIT engines against the interpreter: the host code generator is a
    /// third implementation of the same block and the claim is about all three.
    fn agree(case: &Case, budget: u64, quanta: usize) -> X86 {
        agree_on(case, Engine::Jit, budget, quanta);
        agree_on(case, Engine::JitHost, budget, quanta)
    }

    /// A generated program, which is what the differential corpus runs.
    fn seeded(seed: u64) -> Case {
        Case::seeded(differential::program(seed, 24))
    }

    fn seeded64(seed: u64) -> Case {
        Case::seeded(differential::program64(seed, 24)).long()
    }

    #[test]
    fn a_flat_core_and_an_interpreted_one_agree_on_every_column() {
        let jit = agree(&seeded(0x51ee), 8_000, 8);
        let stats = jit.jit_stats().expect("a JIT core keeps statistics");
        assert!(stats.blocks > 0, "no block ran, so nothing was compared");
    }

    #[test]
    fn a_paged_core_agrees_too_and_really_translates() {
        let jit = agree(&seeded(0x9a13).paged(), 8_000, 8);
        let stats = jit.jit_stats().expect("statistics");
        assert!(stats.blocks > 0, "no block ran under paging");
    }

    #[test]
    fn a_long_mode_core_agrees_and_really_translates() {
        let jit = agree(&seeded64(0x2c07), 8_000, 8);
        let stats = jit.jit_stats().expect("statistics");
        assert!(stats.blocks > 0, "no block ran in long mode");
    }

    #[test]
    fn the_same_agreement_holds_over_budgets_no_block_fits_in() {
        // Forty clocks is under the worst case of every block here, so the
        // budget guard interprets almost everything — which must give the same
        // answer as interpreting everything, and as blocks.
        agree(&seeded(0x51ee), 40, 200);
        agree(&seeded64(0x2c07).paged(), 40, 200);
    }

    #[test]
    fn several_generated_programs_agree_in_every_world() {
        for seed in 0..12u64 {
            agree_on(&seeded(seed * 7 + 1), Engine::Jit, 6_000, 6);
            agree_on(&seeded(seed * 7 + 1).paged(), Engine::JitHost, 6_000, 6);
            agree_on(&seeded64(seed * 13 + 3), Engine::JitHost, 6_000, 6);
        }
    }

    #[test]
    fn a_store_into_the_running_page_is_honoured_by_the_next_block() {
        // `mov [ebx], al` where `ebx` points at the code itself. The block was
        // lifted from those bytes, so both engines must execute what the store
        // left rather than what was there — which under `Smc::Guard` is the
        // in-block page test and under `Smc::EndBlock` the page drain at the
        // next boundary.
        let program = alloc::vec![
            0xb8, 0x90, 0x90, 0x90, 0x90, // mov eax, 0x90909090
            0x88, 0x03, // mov [ebx], al
            0x40, // inc eax
            0xeb, 0xf7, // jmp back to the top
        ];
        let mut case = Case::seeded(program);
        // `ebx` at the fourth byte of the program: the immediate of the `mov`
        // the loop starts with, which is inside the block being executed.
        case.regs[3] = differential::BASE + 2;
        case.pointers = 0;
        agree(&case, 8_000, 6);
    }

    #[test]
    fn an_instruction_outside_the_subset_is_interpreted_and_charged_the_same() {
        // `cli`, `sti` and `pushf`/`popf` are all outside the lifted subset, so
        // every pass round this loop leaves the block cache and comes back —
        // which is the path `Costs`'s zero sentinel exists for.
        let program = alloc::vec![
            0xfa, // cli
            0x9c, // pushf
            0x9d, // popf
            0xfb, // sti
            0x40, // inc eax
            0xeb, 0xf9, // jmp back to the top
        ];
        let mut case = Case::seeded(program);
        case.regs[3] = 0;
        agree(&case, 8_000, 6);
    }

    #[test]
    fn the_cold_bound_fits_inside_a_scheduler_quantum() {
        // The number this file's `MAX_INSNS` is chosen by, asserted rather than
        // left in prose: `SchedulerConfig::max_ticks_per_quantum` is 10 000, and
        // a cold block whose worst case exceeded it could never be admitted at
        // all, so the engine would silently be the interpreter.
        let cap = crate::core::sched::SchedulerConfig::default().max_ticks_per_quantum;
        for bus in [2u64, 4] {
            for paged in [false, true] {
                assert!(
                    worst_bound(bus, paged) < cap,
                    "a cold block at bus={bus} paged={paged} is bounded at {}, which does \
                     not fit a {cap}-tick quantum: no block would ever run",
                    worst_bound(bus, paged),
                );
            }
        }
    }
}
