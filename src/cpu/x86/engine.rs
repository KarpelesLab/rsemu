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
//! The inlined memory path made that first half stop being free, and it is the
//! one place where this core needs something from the `jit` seam that the seam
//! does not have. An access the **backend** serves never reaches
//! [`IrHost::load`], and [`FastMem::note_fast_load`] is handed neither the
//! address nor the value, so there is nothing for `Exec::phys_read` to latch
//! and nothing here to latch it from. The block carries the byte instead, in
//! `lift::OPEN_BUS`, and [`Host::bus_locked`] is what stops a published slot
//! from overwriting a byte a *call* left — including the one a page-table walk
//! leaves on its way to a fault.
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
//! # The tick allowance, and what the frontend's instruction limit costs now
//!
//! A quantum is a count of ticks, and the guest has to stop on the same
//! instruction whichever engine drives it, carrying the same `State::debt`
//! into the next quantum — both of which are in the snapshot the machine's
//! state hash is taken over. There are two ways to arrange that and this file
//! has now used both.
//!
//! It used to be a **guard**: a block ran only where its worst case, read off
//! its ops, fitted what was left. Sound, and expensive, because a worst case
//! is not a cost. The cold bound — sixteen instructions, each the longest
//! legal encoding, each `IMUL`, each making two page-crossing eight-byte
//! accesses — was 3 312 ticks under a four-level walk against the **185** an
//! admitted block really spent on average, so a boundary reached in the tail
//! of a quantum was interpreted whatever was actually there. Measured on
//! `pc64` over nine hundred guest seconds of a 6.6 kernel: **36.4 M guest
//! instructions, 2.02% of the whole run**, three times everything outside the
//! lifted subset put together.
//!
//! It is now an **allowance**. [`Host::spent`] compares `Exec::used` against
//! what [`advance`] was handed; `ir::Interp` and `jit::x86`'s replay ask it at
//! every guest instruction boundary but a block's first and an exit, and
//! `jit::dispatch` asks it at every block boundary but a run's first. A `true`
//! stops with `Outcome::Spent` — architectural state published from that
//! boundary's live map exactly as at a fault, the guest standing at an
//! instruction that has not started. So a block that overruns *leaves*
//! instead of being refused before it starts, at precisely the instruction an
//! interpreted core would have stopped at, and nothing here has to know what a
//! block costs. `Stop::Spent` needs no arm of its own in [`advance`] for the
//! same reason: publishing at `run.pc` is what every other stop already does.
//!
//! Same board, same nine hundred seconds, same kernel and initramfs:
//!
//! | | guarded | allowance |
//! | --- | ---: | ---: |
//! | retired inside a block | 97.3% | **99.3%** |
//! | interpreted | 48 592 900 | **12 248 632** |
//! | translations | 223 965 | 132 053 |
//!
//! What is left is the exclusion list and almost nothing else: 11.9 M
//! encodings outside the lifted subset and 0.35 M interrupt shadows and pins.
//! `docs/platforms/pc64.md` has the profile.
//!
//! ## What it costs, and the frontend is what pays
//!
//! [`FLAGS`] is [`Flags::Eager`] because of this and for no other reason. The
//! precondition [`IrHost::spent`] states is that **every boundary's live map
//! is architecturally complete**, which is strictly stronger than the
//! monotonicity `InsnStart::live` asks for — and under [`Flags::Elide`] this
//! frontend omits the flags an instruction is about to write and lets the
//! dead-code pass delete the arithmetic behind them. Sound to a terminator,
//! sound at a fault, unsound at a boundary the guest is *resumed* from.
//!
//! Priced under callgrind on `pc64`, 120 guest seconds, the same binary either
//! way: eager flags are **+2.55%** of host instructions and the allowance they
//! buy is **−9.27%**, so the pair is **−6.95%** against the guarded
//! tree. `benches/x86_dispatch.rs` measures the elision on its own, per block,
//! and says what is being given up: 2.2× on a flag-heavy loop.
//!
//! ## And [`MAX_INSNS`] was a number the guard chose
//!
//! Sixteen was here because the cold bound scaled with it: at
//! `lift::MAX_INSNS`'s sixty-four a cold block was bounded at 13 200 ticks,
//! above `SchedulerConfig::max_ticks_per_quantum`'s 10 000, so **no block
//! would ever have run**. Nothing computes a bound now, so that constraint is
//! gone and the constant was re-measured over the same nine hundred seconds:
//!
//! | [`MAX_INSNS`] | retired | interpreted | blocks executed | translations |
//! | ---: | ---: | ---: | ---: | ---: |
//! | 8 | 99.3% | 12 248 632 | 438 391 122 | 146 565 |
//! | 12 | 99.3% | 12 248 632 | 383 942 288 | 135 065 |
//! | 16 | 99.3% | 12 248 632 | 364 481 972 | 132 053 |
//! | 24 | 99.3% | 12 248 632 | 349 598 502 | 130 527 |
//! | 32 | 99.3% | 12 248 632 | 341 582 275 | 130 197 |
//! | 48 | 99.3% | 12 248 632 | 339 978 477 | 129 913 |
//! | 64 | 99.3% | 12 248 632 | 339 317 888 | 129 895 |
//!
//! **The retired count is identical at every value, to the instruction.** That
//! column used to run from 98.0% at eight down to 96.5% at thirty-two, and
//! every bit of that spread was the guard rather than the frontend: a longer
//! block had a larger worst case and was refused more often. What
//! [`MAX_INSNS`] still decides is how many *pieces* the same work is cut into
//! — 438 M block entries at eight against 339 M at sixty-four, each entry an
//! [`admit`] with a fetch-path translation and a world derivation in it — and
//! that curve is flat past thirty-two.
//!
//! So the constant was chosen under callgrind instead — host instructions,
//! which do not care what else is running on the machine. The first choice was
//! made over 120 guest seconds and landed on sixteen at the bottom of a U whose
//! upper arm was **temporaries**, and that arm belonged to the seam rather than
//! to this file: `jit::x86::rt`'s `Engine::run` did `temps.clear();
//! temps.resize(block.temp_count(), 0)` on *every block execution*, so running a
//! block had a term proportional to how many temporaries the whole block
//! declared and none at all to how many instructions of it actually ran —
//! exactly the wrong shape once a block may leave part-way through. That was 69
//! host instructions per block execution at sixteen and **310 at thirty-two**.
//!
//! The seam no longer does it. `Engine::run` keeps one frame at the high-water
//! mark of every block it has run and never clears it, which takes the zero-fill
//! to 0.06 instructions per block execution at *both* values and deletes the
//! upper arm of the U. Re-measured over 1 200 guest seconds of the same board,
//! nine runs that each retired 2 051 816 640 guest instructions in a block and
//! interpreted 19 069 320 — identical to the instruction, so nothing but the
//! host cost differs:
//!
//! | [`MAX_INSNS`] | host instructions | against sixteen |
//! | ---: | ---: | ---: |
//! | 8 | 1 648 143 347 872 | +5.8% |
//! | 12 | 1 583 825 664 065 | +1.6% |
//! | 16 | 1 558 317 676 940 | — |
//! | 24 | 1 546 324 452 654 | −0.8% |
//! | **32** | **1 531 989 462 623** | **−1.7%** |
//! | 48 | 1 531 468 039 487 | −1.7% |
//! | 64 | 1 532 158 352 347 | −1.7% |
//!
//! What is left is the lower arm on its own: going *down* costs block entries,
//! at eight the same guest work cut into 438 M of them against 364 M, each one
//! an [`admit`] with the pins, a world derivation, a fetch-path translation and
//! a table probe in it. Past thirty-two the curve is flat to within 0.05%.
//!
//! **Thirty-two is the constant that follows.** The numerical minimum is at
//! forty-eight, by 0.034% — 521 M instructions out of 1.53 T, which is real and
//! deterministic and not worth having: a raised exit flag is honoured within one
//! block (`jit::dispatch`, "Safe points"), so [`MAX_INSNS`] is also the
//! safe-point latency, and paying 50% more of it for 0.03% is the wrong trade.
//! `docs/platforms/pc64.md` carries the measurement in full.
//!
//! ## What a mutation sweep says about this
//!
//! Fourteen bugs were injected one at a time, across this file and
//! `cpu::riscv::engine`. What is worth recording is the two that were expected
//! to walk out and did not:
//!
//! * **[`FLAGS`] flipped back to [`Flags::Elide`].** [`IrHost::spent`] says
//!   plainly that nothing below a frontend can check its precondition, and
//!   that is true of the IR — but this core's own fixtures do check it:
//!   `a_pending_interrupt_is_taken_at_the_instruction_the_interpreter_takes_it_at`
//!   and `a_budget_no_whole_block_fits_in_still_retires_inside_one` both fail,
//!   because `agree_on` compares `EFLAGS` after every quantum and a quantum
//!   that ends inside a block is exactly where an elided flag is stale. The
//!   precondition is unchecked where it is *stated* and checked where it is
//!   *adopted*, which is the arrangement to want.
//! * **A [`Host::spent`] that always answers `true`.** Not a bug at all, which
//!   is why it is interesting: a block that retires one guest instruction and
//!   leaves is what the interpreter does, tick for tick, so every differential
//!   column agrees under it and the engine is silently an interpreter with a
//!   translator bolted on.
//!   `a_block_given_a_whole_quantum_retires_more_than_one_instruction` is what
//!   catches it, and it is the only assertion here about the *shape* of a run
//!   rather than its result.
//!
//! The survivor is [`Unlifted`] answering nothing, and it is correct either
//! way: the lift runs, produces zero instructions, and `Dispatcher::run`
//! reports `Stop::Untranslatable`. It is a performance table whose whole
//! argument is a measurement — 42 M wasted lifts in four minutes on the
//! sibling core — so a test here would be asserting the measurement rather
//! than the behaviour.
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
//! Nothing this core *computes* can raise one from inside a block: `STI`,
//! `CLI`, `POPF`, `HLT`, `INT`, `IRET`, every segment load and every `MOV` to
//! a control register are outside the lifted subset and end the block. A
//! **store** into an interrupt controller can, and it is seen at the very next
//! boundary — which under paging is the next instruction, because
//! [`Smc::EndBlock`] ends the block there.
//!
//! **A load can too, and so can a store with paging off**, and that was missed
//! until a workload was written for it. The store half is the easier of the
//! two to miss twice: `admit` picks [`Smc::EndBlock`] only *under* paging, and
//! with paging off the policy is [`Smc::Guard`], where a store is an ordinary
//! instruction and the block runs on past it. The local APIC and the HPET are lazily-advanced devices
//! (`ROADMAP.md` §4.2): `MemOps::read` catches the chip up to this core's
//! *live* position before answering, so a timer that expires inside that
//! catch-up requests its vector there and then and `INTR` rises **between two
//! instructions of a lifted block**. Ordinarily it cannot, because
//! `Scheduler::natural_target` ends the round on the soonest event a
//! lazily-advanced device has of its own — so the expiry is a quantum
//! boundary and both engines see it in the same place. The window is a timer
//! the guest **reprograms into the round that is already running**: the target
//! was chosen when the round began and does not move, so the next read of the
//! APIC's current-count register, or of the HPET's main counter, crosses it.
//! [`IrHost::load`] and [`IrHost::store`] are where that is now noticed and
//! [`Host::hand_back`] is how it reaches the boundary. The regression test is
//! `a_load_that_raises_intr_is_taken_where_the_interpreter_takes_it`; before
//! it, the interpreter took the interrupt with `EAX` at zero and a translated
//! core with `EAX` at nineteen.
//!
//! The third way in is the **entry translation itself**, and it is now closed
//! rather than stated as not covered. [`admit`] asks the pins and *then*
//! charges the entry translation, so a page-table walk between the two used to
//! be unwatched — and a walk is not arithmetic, it is a physical read that the
//! address space answers however the board decided. This paragraph used to end
//! "on a PC that is empty — page tables live in DRAM", which is a statement
//! about what a *guest* does: `CR3` is a guest-written register, `paging::walk`
//! masks it and range-checks nothing, and the entries come back through
//! `Exec::phys_read`. `machines/pc64.machine` is genuinely the empty case — it
//! maps no MMIO at all — but every `pc-at` and `q35` board maps `pc.hpet` at
//! `0xfed0_0000` on `U32` and `U64`, which are exactly the widths a legacy and
//! an IA-32e descriptor read use, and `pc.lapic` at `0xfee0_0000` on `U32`,
//! which a non-PAE walk uses. Both are lazily advanced, so answering catches
//! the chip up and can drive a line.
//!
//! It is also the window `cpu::arm::a64::engine`'s `admit` closes from the
//! other side — a walk whose own **ticks** cross a comparator, which is a
//! shape this core does not have because nothing here is driven off
//! `Exec::used`. [`Admitted::leave`] carries the answer: [`admit`] re-asks
//! [`pins_pending`] when — and only when — the translation charged, and
//! [`Host::hand_back`] does the rest.
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
use crate::ir::{InsnStart, IrHost, MemOp, RegSlot, verify};
use crate::jit::{
    BlockCache, DirtyPages, Dispatcher, Entry, Epoch, FastMem, Frontend, PAGE_MASK, Stop, StoreLog,
    Translation,
};

use super::exec::{Ex, Exec, Fault, State, VEC_SS};
#[cfg(test)]
use super::isa::seg;
use super::lift::{
    self, ARITH_MASK, EFLAGS_REST, FLAG_BITS, FLAG_SLOTS, Flags, OPEN_BUS, Origin, RIP, SLOT_COUNT,
    Shape, Smc, World, r_slot,
};
use super::paging::Access;
use super::prot::canonical;
use super::{Config, Lines, flags};

/// How much of a block the frontend is allowed to swallow.
///
/// Direct branches are merged, so a loop unrolls into one translation and a
/// guest register stays in a temporary across the whole of it.
const SHAPE: Shape = Shape::Trace;

/// Whether a boundary names every flag or only the observable ones.
///
/// **[`Flags::Eager`], and it is what the tick allowance costs.**
/// [`Flags::Elide`] leaves out of a boundary's live map the arithmetic flags
/// the instruction it begins is about to write, and the dead-code pass then
/// deletes the arithmetic that would have produced them. That is sound for a
/// block that runs to its terminator, and sound at a fault — elision is
/// refused wherever an instruction can take one — and it is **not** sound for
/// a block that *leaves*: the guest resumes standing at that boundary with an
/// interrupt able to push a stale `EFLAGS`, and the two engines end the
/// quantum with different flags.
///
/// [`IrHost::spent`] states that precondition and says plainly that nothing
/// below the frontend can check it, so the choice is made here and it is made
/// eagerly. Measured under callgrind on `pc64` the elision was worth 2.8% of
/// host instructions and the allowance it blocks is worth several times that;
/// `benches/x86_dispatch.rs` reports the per-block half of the same trade.
const FLAGS: Flags = Flags::Eager;

/// How many guest instructions one block may cover.
///
/// Thirty-two rather than `lift::MAX_INSNS`'s sixty-four. The budget guard
/// that once forced sixteen is gone; what decides it now is that this number
/// is also the safe-point latency, and the throughput curve is flat above
/// thirty-two — see the module docs.
const MAX_INSNS: usize = 32;

/// What a store does to the block it is in.
///
/// **[`Smc::HostGuard`], in both worlds**, and it replaced a split: paging on
/// meant [`Smc::EndBlock`] — a store was the last guest instruction in its
/// block — and paging off meant [`Smc::Guard`], the three-instruction linear
/// comparison [`lift`] emits after each store. The split existed because the
/// in-block guard can only see the address the guest computed, and under
/// paging two linear pages may alias one physical page, so it is refused
/// there.
///
/// [`Host::note_writes`] makes the same comparison one step earlier and on the
/// **physical** page at both ends: `Exec::wrote` records the address a bus
/// write reached and [`Admitted::frame`] is what the entry translation
/// resolved to. That has no aliasing hole, it emits no IR, and it is the same
/// code for a compiled store and an interpreted one. `cpu::arm::a64::lift`
/// took this route first and `cpu::x86::lift`'s module docs recorded that it
/// would work here and that nobody had done it; this is that, done.
///
/// What it replaced is the expensive one: `admit` picked [`Smc::EndBlock`]
/// whenever the guest paged, which is every guest worth measuring, so a store
/// cost a whole block dispatch. `docs/platforms/pc64.md` has the boot it was
/// measured on.
const SMC: Smc = Smc::HostGuard;

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

/// How many *"there is no block at this PC"* answers are remembered.
const UNLIFTED_SLOTS: usize = 65536;

/// How big a host code buffer this core asks for: 256 MiB.
///
/// The same number `cpu::riscv::engine` argues for, and the argument carries:
/// the buffer is append-only and reclaimed only by a reset that throws every
/// compiled block away, an x86 block lifts to more IR than an RV64 one rather
/// than less, and `jit::x86::buf` flips a page-sized window rather than the
/// whole mapping so a larger buffer costs address space and nothing per
/// compile.
#[cfg(any(
    all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"),
    all(feature = "jit-arm64", target_os = "linux", target_arch = "aarch64")
))]
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
    ///
    /// On `pc64`, nine hundred guest seconds of a 6.6 kernel and a busybox
    /// userspace, that is 48.5 M against 1 749.7 M retired — and the split
    /// inside it is worth knowing before optimising either half: 11.7 M are
    /// encodings outside the subset and 36.6 M are inside it and were refused
    /// at a boundary, nearly all of them in the tail of a quantum.
    pub interpreted: u64,
    /// Compiled loads served from an **inlined** software-TLB probe, with no
    /// call back into this core's memory path.
    ///
    /// Zero under `engine = "jit"`, on a part with `Buffers::Unified`, and in
    /// every world `cpu::x86::lift::Lifter::flat` refuses — which is every one
    /// below long mode. See this file's `impl FastMem`.
    pub fast_loads: u64,
    /// Compiled stores served the same way.
    pub fast_stores: u64,
}

/// What deciding whether a block may run needs, and the dispatcher does not.
#[derive(Debug)]
struct Boundary {
    unlifted: Unlifted,
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
            at: Boundary {
                unlifted: Unlifted::new(),
                retired: 0,
                interpreted: 0,
                seen: None,
                generation: 0,
            },
        }
    }

    /// Whether this engine can use a `paging::Tlb` shadow.
    ///
    /// Only the host code generator inlines an access ([`FastMem`]); the
    /// portable backend calls [`IrHost::load`] for every one, so a shadow
    /// attached for it would be filled and never read. A fill probes the
    /// address space's flat view, so it is asked for by the one engine that
    /// reads it, and a `jit-host` that fell back to the portable backend —
    /// no `jit::host` backend for this target — does not ask.
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

    /// Throw every translation away.
    pub(super) fn flush(&mut self) {
        self.disp.cache_mut().flush();
        self.at.unlifted.clear();
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

/// A direct-mapped table of the `(pc, key)` pairs there is **no block** at.
///
/// One job, and it is about not paying for the same answer twice: the
/// instruction at that PC is outside the lifted subset, so the interpreter
/// should be reached without a dispatcher round trip and a fresh lift that
/// fails at its first instruction. On x86 that matters more than on RISC-V,
/// not less: `REP MOVSB`, `IRET`, `INT`, `CMPXCHG`, every `MOV` to a control
/// register, every segment load and the whole of SSE are outside the subset,
/// and a kernel is full of them.
///
/// It used to hold a *number* as well — the most ticks the block at that PC
/// could charge — because a block ran only when its worst case fitted what was
/// left of the caller's budget, and computing that meant walking the block's
/// ops. [`IrHost::spent`] retired the guard and the number with it: a block
/// that overruns leaves at a guest instruction boundary now instead of being
/// refused before it starts, so nothing asks what a block costs and the only
/// answer left worth remembering is the negative one.
///
/// Recording **only** negatives is what that leaves, and it is strictly better
/// than the table that held both: a positive answer can no longer evict a
/// negative one from the slot they share.
///
/// The table is deliberately small — 65 536 slots, direct-mapped on the low
/// bits of the PC. A 262 144-slot table with the PC's high bits folded in cut
/// collisions from 73 M to 20 M over a nine-hundred-second boot and made the
/// run **10% slower**: the answers it holds are consulted once per block
/// boundary and eight megabytes of them do not stay in a host cache. The
/// retired fraction was identical to three significant figures either way.
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

    /// x86 instructions are byte-aligned, so every bit of the PC carries.
    #[inline]
    fn index(pc: u64) -> usize {
        (pc as usize) & (UNLIFTED_SLOTS - 1)
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
    /// Whether the **entry translation itself** brought an interrupt pin up,
    /// so this block must leave at its first guest instruction boundary.
    ///
    /// [`admit`]'s halves are asked in an order that cannot be swapped — the
    /// pins decide whether a block runs at all and the translation is what
    /// names it — so there is a window between them, and a page-table walk
    /// lives in it. See [`admit`]'s own docs.
    leave: bool,
}

/// Whether a block may run at `pc`.
#[derive(Debug)]
enum Admit {
    /// It may.
    Ready(Box<Admitted>),
    /// It may not, and the reason is one the interpreter answers.
    Interpret,
}

/// The **pins** `Exec::step` decides on before it decodes, asked without
/// taking any of them.
///
/// Split out of [`admit`] because it is asked from two places and the two must
/// be the same question: once before a block runs, and once from
/// [`IrHost::load`] and [`IrHost::store`] for the two things a running block
/// does that can change the answer. `iflag` is passed rather than read because a block holds `EFLAGS`
/// in its own slots, not in `State::regs`.
///
/// The state half of the test stays in [`admit`]: a shutdown, a halt, a `HLT`
/// waiting for a `SIPI`, an interrupt shadow and the trap flag are all
/// `State` this core writes with instructions the frontend does not lift, so
/// none of them can change inside a block.
#[inline]
fn pins_pending(lines: &Lines, iflag: bool) -> bool {
    lines.init_latched()
        || lines.init_held()
        || lines.startup_pending().is_some()
        || lines.nmi_pending()
        || (iflag && lines.intr_pending())
}

/// Everything a block owes before it runs — for the first block of a run and
/// for every chained successor alike.
///
/// The order is load-bearing and is the RISC-V engine's: **the events the
/// interpreter takes before it decodes**, then **the entry fetch translation**.
/// There used to be a third — the budget guard — and [`IrHost::spent`] is what
/// removed it: a block no longer has to prove that its worst case fits before
/// it starts, because a block that overruns leaves at a guest instruction
/// boundary instead.
///
/// The second is the contract `lift`'s module docs call *"the one that
/// looks like a working JIT"*: a cached block makes no fetches, but the
/// instruction it replaced translated its first byte through the fetch path,
/// walking the tables on a buffer miss and charging for the walk. Doing it on
/// every execution rather than once at lift time is what makes a served block
/// cost what an uncached one cost — and a **failed** translation is rolled
/// back rather than charged, because the block then does not run and the
/// interpreter's own fetch is about to walk those same tables. A successful
/// one is different in exactly the way that matters: it filled the buffer, so
/// the interpreter's fetch finds the entry and charges nothing.
///
/// # The window between the two, and [`Admitted::leave`]
///
/// The order means the pins are looked at and *then* the entry translation
/// runs, so anything the translation does to them happens after the last look.
/// A translation that hits the buffer does nothing at all — but a **miss walks
/// the page tables**, and a descriptor read is an ordinary physical access
/// that the address space answers however the board decided. Put a page table
/// over `pc.hpet` or `pc.lapic`, both of which take the width a walk reads at
/// and both of which are lazily advanced, and the walk brings `INTR` up
/// between [`pins_pending`] and the block's first instruction — with nobody
/// left to look, so the block runs to its natural end.
///
/// The cost of closing it is one comparison of `Exec::used` against a value
/// this function already had for the roll-back path, because **only a walk
/// charges**: a buffer hit returns before `Exec::phys_read` and an unpaged
/// core never gets here at all.
///
/// # The one exit that is still not covered
///
/// [`Admitted::leave`] answers for a block that runs. It cannot answer for the
/// third way out of this function: a **known-unliftable PC on a cold page**,
/// where the walk happens, brings a pin up, and then `at.unlifted.holds` sends
/// the instruction to `Exec::step` — whose own first act is to take the
/// interrupt, so the instruction never runs and `RIP` names it rather than its
/// successor. An interpreted core would have run it: its `step` looked at the
/// pins *before* its fetch charged the walk.
///
/// It is the same window with the same reachability — the walk still has to
/// read a device — and closing it needs something this file does not have: a
/// way to ask `Exec` for one step with the pin check already discharged.
fn admit(at: &mut Boundary, exec: &mut Exec<'_>, pc: u64) -> Admit {
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
    if pins_pending(lines, st.regs.eflags & flags::IF != 0) {
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

    let mut leave = false;
    let frame = if paged {
        let user = exec.cpl() == 3;
        let before = exec.state.cycles;
        let spent_before = exec.used;
        match exec.translate_access(linear, Access::fetch(user)) {
            Ok(phys) => {
                // A walk, and only a walk, can have moved the answer the pins
                // gave above: a buffer hit returns before `phys_read` and
                // charges nothing, so the comparison is exact and free.
                leave = exec.used != spent_before
                    && pins_pending(lines, exec.state.regs.eflags & flags::IF != 0);
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

    let key = lift::key(&world, SHAPE, SMC, FLAGS);
    // Known unliftable: the interpreter takes this instruction, and reaching
    // it without a dispatcher round trip and a lift that fails at its first
    // instruction is the whole point of remembering it.
    if at.unlifted.holds(pc, key) {
        return Admit::Interpret;
    }

    Admit::Ready(Box::new(Admitted {
        world,
        key,
        linear_page: linear & !PAGE_MASK,
        frame,
        leave,
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
    let admitted = match admit(bound, &mut exec, pc) {
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
        admitted: true,
        rejected: None,
    };
    let mut host = Host::new(&mut exec, pc, &admitted, remaining);
    if admitted.leave {
        // The entry translation walked, and the walk brought a pin up. One
        // instruction retires and the run hands the boundary back, which is
        // what `Exec::step` does: it charges the fetch, runs the instruction,
        // and takes the interrupt on its next call.
        host.hand_back();
    }
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
        bound.unlifted.clear();
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
        // a boundary whose instruction is outside the subset, `Spent` a block
        // that left part-way through because the caller's ticks ran out, and
        // all four leave the guest at `run.pc` for the run loop to pick up.
        // `Spent` needs no arm of its own for exactly that reason: the state
        // is published from the boundary's live map and `run.pc` is the guest
        // instruction that has not started, which is what this arm already
        // does with every other stop. `Exit` cannot happen: no safe-point flag
        // is given to the dispatcher.
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
        // A page a translation came from has changed, so every answer in
        // [`Unlifted`] may have changed with it: an instruction that was
        // outside the subset can have been overwritten by one that is not.
        bound.unlifted.clear();
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
    /// The unliftable-PC table and the world counter, which a chained boundary
    /// consults through [`admit`] exactly as [`advance`]'s prologue does.
    bound: &'a mut Boundary,
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
        match admit(self.bound, host.exec, pc) {
            Admit::Ready(at) => {
                self.at = *at;
                if self.at.leave {
                    // The same window as the prologue's, at a chained
                    // boundary: `Dispatcher::run` asked [`IrHost::spent`]
                    // before this call and the walk below it raised a pin, so
                    // the answer has changed since.
                    host.hand_back();
                }
                // The host follows the block across a page: [`close_bus`] reads
                // the last retired instruction's last byte back through the
                // frame of whichever block retired it.
                host.frame = self.at.frame;
                host.world = self.at.world;
                host.topology = host.exec.mem.generation();
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
        let lifted = lift::lift(&self.at.world, pc, &mut src, MAX_INSNS, SHAPE, SMC, FLAGS)?;
        if self.rejected.is_none()
            && let Err(e) = verify(&lifted.block)
        {
            self.rejected = Some(alloc::format!("{e}"));
        }
        // A lift that produced nothing is remembered, which is what sends the
        // next pass straight to the interpreter instead of back through here.
        if lifted.insns == 0 {
            self.bound.unlifted.note(pc, self.at.key);
        }
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
    /// What [`advance`] was given: the ticks this run may charge before a
    /// block has to stop at its next guest instruction boundary.
    ///
    /// The whole of what this core owes [`IrHost::spent`]. It is a function of
    /// ticks already charged and it is monotone, because `Exec::used` only
    /// ever grows within one [`advance`].
    allowance: u64,
    slots: [u64; SLOT_COUNT as usize],
    /// The x86 fault the memory path raised, kept because [`IrHost::load`] can
    /// only report a [`BusError`] and an x86 fault is a vector *and* an error
    /// code — and `#SS` through the stack is a different vector from `#GP`
    /// through everything else.
    fault: Option<Fault>,
    dirty: DirtyPages,
    /// The world and the physical frame of the block currently running.
    ///
    /// [`close_bus`] reads the last instruction's last byte back through the
    /// frame, and [`Smc::HostGuard`] compares every store's physical page
    /// against it. Both follow the block across a page: [`Lifter::enter`]
    /// replaces the pair at every chained boundary.
    world: World,
    frame: u64,
    /// The address space's topology generation as of this block's entry.
    ///
    /// The second of the two things "a store ends the block" used to answer
    /// for free, and the one that is easy to miss now that it does not. See
    /// [`IrHost::store`].
    topology: u64,
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
    /// Whether `State::open_bus` is `Exec`'s to own rather than the block's.
    ///
    /// The open bus is the one piece of architectural state a block cannot
    /// simply carry in a slot, because both engines write it and they write it
    /// at different moments. An access the backend serves leaves the byte in
    /// [`OPEN_BUS`], which arrives here through [`IrHost::write_slot`] at the
    /// boundary the block leaves at; an access that took the **call** latched
    /// the byte itself, in `Exec::phys_read` — and so did the page-table walk
    /// of an access that then *faulted*, which is the case that makes this a
    /// flag rather than an ordering.
    ///
    /// Set by every access that reaches [`IrHost::load`] or [`IrHost::store`],
    /// cleared by every one the backend served. While it is set, a published
    /// [`OPEN_BUS`] is dropped: `Exec` has the newer byte, by construction.
    bus_locked: bool,
}

impl<'a, 'e> Host<'a, 'e> {
    fn new(exec: &'a mut Exec<'e>, pc: u64, at: &Admitted, allowance: u64) -> Host<'a, 'e> {
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
        let topology = exec.mem.generation();
        Host {
            exec,
            allowance,
            slots,
            fault: None,
            dirty: DirtyPages::new(),
            world: at.world,
            frame: at.frame,
            topology,
            cur_end: None,
            cur_access: false,
            end: None,
            end_access: false,
            overflowed: false,
            // Nothing has run, so nothing has replaced what `Exec` already
            // holds.
            bus_locked: true,
        }
    }

    /// Whether an interrupt pin has come up since this run started.
    ///
    /// `EFLAGS` comes out of the slots rather than out of `State::regs`,
    /// because inside a block the slots are where it lives — `IF` is in
    /// [`EFLAGS_REST`], the half [`ARITH_MASK`] leaves alone.
    #[inline]
    fn pins(&self) -> bool {
        let iflag = self.slots[EFLAGS_REST.0 as usize] & u64::from(flags::IF) != 0;
        pins_pending(self.exec.lines, iflag)
    }

    /// Retire what is left of this run's `allowance`, so the block leaves at
    /// its next guest instruction boundary.
    ///
    /// The whole of how an interrupt reaches [`IrHost::spent`], and it costs
    /// that function *nothing*, in the strong sense that its body is
    /// unchanged: a boundary already loads `allowance` and compares
    /// `Exec::used` against it, and zero is the value that comparison is
    /// always true for. A second field would have been a second load on the
    /// hottest path in the file to carry one bit this one already has room
    /// for.
    ///
    /// What is left to pay for is the question in [`IrHost::load`] and
    /// [`IrHost::store`], and this core pays it on **every** access rather than
    /// on the few a plan does not cover, because x86 publishes no inlined
    /// memory path (see [`FastMem`]). Measured under callgrind — a clock was
    /// useless on the host this was written on, which was carrying a load
    /// average of thirty — `X86::run_budget` taken inclusively over the whole
    /// of `benches/x86_dispatch --smoke` went from **1 219 379 096 host
    /// instructions to 1 221 612 704, +0.18%**.
    ///
    /// The ticks are not lost. [`advance`] reports `Exec::used`, and
    /// `X86::run_budget` loops until *its* allowance is spent, so what this
    /// gives up is the rest of the **run**, not the rest of the quantum.
    #[inline]
    fn hand_back(&mut self) {
        self.allowance = 0;
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

    /// One access, through whichever of the two paths its [`MemOp`] names.
    ///
    /// `value` is `Some` for a store and `None` for a load; a store answers
    /// zero.
    ///
    /// # The two paths, and why they are the same answer
    ///
    /// A [`MemOp`] that carries a segment register is an **effective** address:
    /// `Exec::read_mem` folds the base in and checks the limit, the access
    /// rights and the present bit, and that is the whole of what x86 has above
    /// paging. A [`MemOp`] with no segment is a **linear** address, and
    /// `lift::Lifter::flat` emits one only in long mode and only through `CS`,
    /// `DS`, `ES` or `SS`, whose bases are architecturally zero — so the fold it
    /// skipped was the identity and the checks it skipped do not exist
    /// (*Intel SDM* volume 3 §3.4.4).
    ///
    /// What is left of `Exec::seg_linear` in that world is the canonical test,
    /// which is here, and the vector it raises, which is `#GP(0)` for the three
    /// data segments and `#SS(0)` for the stack — and the stack's has already
    /// been asked in the block, because [`lift::STACK`] is the only place that
    /// distinction survives the segment register being dropped.
    ///
    /// The two paths are therefore one answer written twice, which is the
    /// property `differential::compare` checks instruction by instruction.
    fn linear_or_segmented(&mut self, mem: &MemOp, addr: u64, value: Option<u64>) -> Ex<u64> {
        let bytes = mem.size.bytes() as u8;
        match (mem.seg, value) {
            (Some(s), None) => self.exec.read_mem(s.0, addr, bytes),
            (Some(s), Some(v)) => self.exec.write_mem(s.0, addr, bytes, v).map(|()| 0),
            (None, _) if !canonical(addr) => Err(Fault::gp(0)),
            (None, None) => self.exec.linear_read(addr, bytes),
            (None, Some(v)) => self.exec.linear_write(addr, bytes, v).map(|()| 0),
        }
    }

    /// Report an x86 fault as the bus error the IR speaks, keeping the vector.
    fn raise(&mut self, fault: Fault) -> BusError {
        self.fault = Some(fault);
        BusError::Protected
    }

    /// Move whatever this core has written into the dirty log, and leave the
    /// block if any of it landed in the block's own code.
    ///
    /// Whatever landed, landed: a store that crossed a page boundary and
    /// faulted on the second page still wrote the first, and a translation of
    /// those bytes is stale either way.
    ///
    /// **The one place [`Smc::HostGuard`] is implemented**, and it is one
    /// place because x86 publishes no inlined store path (see [`FastMem`]):
    /// every store a block makes arrives through [`IrHost::store`], so there
    /// is nothing for a code generator to emit and nothing for the two paths
    /// to disagree about.
    fn note_writes(&mut self) {
        for i in 0..self.exec.wrote_n as usize {
            // A page, not an address: `Exec::note_write` masks before it
            // records, which is what makes the comparison below one `==`.
            let page = self.exec.wrote[i];
            self.dirty.note(page, 1);
            // [`Smc::HostGuard`], and the whole of it. A store into the page
            // this block's own instructions came from means every instruction
            // after it is a translation of bytes that no longer exist, so the
            // block leaves at its next guest instruction boundary -- which for
            // a store is the one its own instruction ends at, and which is
            // where `Dispatcher::run` drains the log above into the block
            // cache so the next pass lifts the bytes the store left.
            //
            // Physical at both ends: `Exec::wrote` records the address the bus
            // transaction reached, post-A20, and [`Admitted::frame`] is what
            // the entry translation resolved to. That is the aliasing hole
            // `lift`'s in-block guard has and cannot close, and closing it is
            // what lets this core keep one policy in both worlds instead of
            // falling back to [`Smc::EndBlock`] whenever the guest pages.
            if page == self.frame {
                self.hand_back();
            }
        }
        self.exec.wrote_n = 0;
        // The list stopped being the whole truth, so nothing can be ruled out
        // -- including this block's own page. [`advance`] throws every
        // translation away when it sees this; leaving the block is what keeps
        // the *running* one from executing past a store it cannot account for.
        if core::mem::take(&mut self.exec.wrote_over) {
            self.overflowed = true;
            self.hand_back();
        }
    }
}

impl IrHost for Host<'_, '_> {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        u128::from(self.slots[slot.0 as usize])
    }

    /// # The one slot that is not a slot
    ///
    /// [`OPEN_BUS`] goes straight through to `Exec` rather than into the array,
    /// and it is dropped entirely while [`Host::bus_locked`] holds. Both halves
    /// are load-bearing:
    ///
    /// * **Straight through**, because [`publish`] writes the register file and
    ///   `EFLAGS` and there is no third thing for the open bus to be folded
    ///   into. Publication is the only time a slot arrives here, and by then
    ///   the block has finished.
    /// * **Dropped while locked**, because a fault that a *call* raised has
    ///   already left a byte here — an access that takes a `#PF` reads page-table
    ///   entries on the way to failing, and those are bus cycles that latch —
    ///   and the block is publishing the boundary the faulting instruction
    ///   *started* at, whose byte is one transaction older.
    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        if slot == OPEN_BUS {
            if !self.bus_locked {
                self.exec.state.open_bus = value as u8;
            }
            return;
        }
        self.slots[slot.0 as usize] = value as u64;
    }

    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        // Not an access at all: `lift::TRANSFER` is the frontend asking
        // whether a computed near transfer may go here, which is
        // `Exec::jump_near`'s canonical test and nothing else. No bus cycle,
        // no clocks, no memory — so `spent_a_cycle` is not consulted either,
        // and the open bus keeps whatever the transfer's own operand read left
        // on it.
        if mem.space == lift::TRANSFER {
            return if canonical(addr) {
                Ok(0)
            } else {
                Err(self.raise(Fault::gp(0)))
            };
        }
        // The same non-access, for the stack. See [`lift::STACK`]: the inlined
        // memory path drops the segment register, and this is the one thing
        // `Exec::seg_linear` reads out of it that a data segment's answer does
        // not cover.
        if mem.space == lift::STACK {
            return if canonical(addr) {
                Ok(0)
            } else {
                Err(self.raise(Fault::coded(VEC_SS, 0)))
            };
        }
        let before = self.exec.used;
        let done = self.linear_or_segmented(mem, addr, None);
        self.spent_a_cycle(before);
        self.bus_locked = true;
        // One of the two calls a block makes that can bring an interrupt pin
        // up, and the one that is easy to miss.
        //
        // Nothing else in a block touches them: `Exec::charge` counts clocks
        // and nothing on this core is driven off that count, and `HLT`, `CLI`,
        // `STI`, `IRET` and every `MOV` to a control register are outside the
        // lifted subset. A **load** is not a hypothetical: the local APIC and
        // the HPET are lazily-advanced devices (`ROADMAP.md` §4.2), so
        // `MemOps::read` catches the chip up to this core's live position
        // before it answers. An APIC timer whose initial count the guest wrote
        // *into the current round*, after `Scheduler::natural_target` had
        // already chosen where the round ends, expires inside that catch-up —
        // so `INTR` rises between two instructions of a lifted block, where
        // `Exec::step` would have taken it at the next one.
        //
        // Asked here rather than at every guest instruction boundary because
        // this is where the answer can change, and [`Host::hand_back`] is what
        // carries it to the boundary for free.
        if self.pins() {
            self.hand_back();
        }
        match done {
            Ok(v) => Ok(v),
            Err(fault) => Err(self.raise(fault)),
        }
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        let before = self.exec.used;
        let done = self.linear_or_segmented(mem, addr, Some(value)).map(|_| ());
        self.spent_a_cycle(before);
        self.bus_locked = true;
        self.note_writes();
        // The other two, and unlike the RISC-V engine this core cannot lean on
        // "a store ends the block": under [`Smc::HostGuard`] a store is an
        // ordinary instruction in every world, so both of these are as
        // mid-block as the load above and neither has a boundary to be noticed
        // at any more.
        //
        // * **An interrupt this store raised.** A write into an interrupt
        //   controller — an APIC `ICR` write, an 8259A command — or into any
        //   device that answers by pulling a wire, brings a line up between
        //   two instructions of a lifted block, where `Exec::step` would have
        //   taken it at the next one.
        // * **A store that remapped the address space.** `Frontend::epoch`
        //   reads the topology generation at every *block* boundary, which was
        //   enough while a store was the last instruction in its block and is
        //   not now: the rest of this block would otherwise run against a
        //   topology the store replaced. It costs one relaxed atomic load per
        //   store; `cpu::arm::a64::engine` pays the same one for the same
        //   reason, plus the shadow-TLB pointers this core does not take out
        //   because it publishes no [`FastMem`] plan.
        if self.pins() || self.exec.mem.generation() != self.topology {
            self.hand_back();
        }
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

    /// One load and one compare, at every guest instruction boundary but a
    /// block's first and an exit.
    ///
    /// `>=` rather than `>`: `X86::run_budget` loops while `used < allowance`,
    /// so an interpreted core stops at the first instruction that takes the
    /// count to the budget or past it, and a block that stopped one
    /// instruction later would leave the two engines on different instructions
    /// with different `State::debt` for the rest of the run.
    ///
    /// It answers a **second** question with the same compare — *"has an
    /// interrupt pin come up since this run started"*. [`Host::hand_back`]
    /// retires the allowance when [`IrHost::load`] finds one, which puts the
    /// interrupt on the instruction `Exec::step` would have put it on: the
    /// block leaves at the boundary after the load, [`advance`] returns, and
    /// the next call's [`admit`] hands the instruction to the interpreter,
    /// which is where the vector is fetched and the frame pushed.
    #[inline]
    fn spent(&self) -> bool {
        self.exec.used >= self.allowance
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

/// **x86 publishes an inlined memory path in long mode and refuses one
/// everywhere else**, and the refusals are the interesting half.
///
/// A load's address on this core is an *effective* address: the segment base is
/// added and the limit checked before anything reaches a table, which is why
/// the frontend gives a [`MemOp`] a `SegId` and why `jit::x86`'s
/// `Compiler::inlinable` refuses a segmented access. Nothing about that has
/// changed. What changed is that in **long mode** the fold is the identity for
/// `CS`, `DS`, `ES` and `SS` — bases defined to be zero, no limit, no access
/// rights (*Intel SDM* volume 3 §3.4.4) — so `lift::Lifter::flat` emits the
/// linear address it already computed and clears the register, and the backend
/// inlines it with no segment logic of its own.
///
/// The table the probe reads is `paging::Tlb`'s shadow, written in lockstep
/// with this core's own data buffer so that a hit here implies a hit there and
/// the walk an inlined access skips is one that has already been charged. What
/// is left for the host to pay is in [`FastMem::note_fast_load`] and
/// [`FastMem::note_fast_store`].
///
/// # What this refuses, said plainly
///
/// * **Everything below long mode.** A 32-bit protected-mode `DS` has a real
///   base and a real limit, both of them hidden descriptor state that a
///   `MOV DS, ax` replaces, and a block keyed under [`Origin::Paged`] is keyed
///   by the physical page of its bytes rather than by `World::generation`.
/// * **`FS` and `GS`, always.** Their bases come from an MSR and are live
///   state, which is the same objection.
/// * **Paging off**, where nothing fills the shadow, and **the A20 gate shut**,
///   where a host pointer resolved before `Exec::masked` names a different page
///   from the one the bus would reach.
/// * **Every instruction that touches memory twice**, or that can fault after
///   its access — a read-modify-write, a computed `RET`, `PUSH [m]`. The reason
///   is the open bus and is written out on `lift::Plan::one_last_access`.
/// * **A part with `Buffers::Unified`**, a 386 and a 486, whose data
///   translations land in a thirty-two-slot array a sixty-four-slot shadow
///   cannot index in lockstep with.
impl FastMem for Host<'_, '_> {
    fn load_plan(&mut self) -> Option<crate::jit::MemPlan> {
        self.exec.mem_plan(crate::ir::AccessKind::Load)
    }

    fn store_plan(&mut self) -> Option<crate::jit::MemPlan> {
        self.exec.mem_plan(crate::ir::AccessKind::Store)
    }

    /// One aligned load was served inline; charge for it.
    ///
    /// Everything [`IrHost::load`] would have done, minus the bytes:
    ///
    /// * **one bus cycle's clocks**, and only one — a hit in the shadow implies
    ///   a hit in this core's own buffer, so the walk was charged when the entry
    ///   was filled;
    /// * **`close_bus`'s question**, which is *did this guest instruction drive
    ///   the bus after its own fetch*, and an inlined load did;
    /// * **the open bus goes back to being the block's**, because this access
    ///   latched nothing in `Exec` and the byte is in [`OPEN_BUS`].
    ///
    /// What is deliberately **not** here is the interrupt-pin check
    /// [`IrHost::load`] makes. That exists because a read of a lazily-advanced
    /// device — the local APIC, the HPET — catches the chip up and can bring a
    /// line up mid-block. An inlined load reaches plain RAM by construction:
    /// `jit::Tlb::fill` caches only a page that resolves to a direct `RamStore`
    /// with permissive constraints, so there is no device to advance and no pin
    /// that can rise.
    fn note_fast_load(&mut self) {
        let clocks = self.exec.variant().bus_clocks();
        self.exec.charge(clocks);
        self.cur_access = true;
        self.bus_locked = false;
    }

    /// One aligned store was served inline; pay what moving the bytes did not.
    ///
    /// Generated code wrote through the host address the shadow's store entry
    /// carries and did nothing else. This is the rest of `Exec::phys_write`:
    ///
    /// * **one bus cycle's clocks**, as above;
    /// * **the `RamStore`'s own dirty bitmap**, which `jit::Tlb::note_fast_store`
    ///   marks — the only record a framebuffer refresh or a live snapshot has;
    /// * **the global exclusive monitor**, broken by `SpaceView::write` on every
    ///   store that goes through the address space and by nothing at all on one
    ///   that does not. A sibling core's reservation surviving a store because
    ///   the store happened to be compiled is the same defect in a faster
    ///   wrapper. Once rather than on both sides of the transfer, because the
    ///   bytes had already landed before this was called and there is no "before"
    ///   left to take;
    /// * **`Exec::wrote`**, and through [`Host::note_writes`] both of the things
    ///   it feeds: the guest-physical dirty log the dispatcher drains to
    ///   invalidate a translation of the page just written, and `Smc::HostGuard`
    ///   — the comparison against this block's own frame that makes x86's
    ///   coherent instruction cache architectural rather than aspirational.
    ///
    /// Called *after* the bytes have landed, which is unobservable: nothing runs
    /// in between and the fast path cannot fault.
    ///
    /// # Panics
    ///
    /// Never on a guest condition. A shadow entry that does not resolve to RAM
    /// cannot have been used by generated code, which found a host address in it
    /// one instruction earlier; reaching that arm is a code-generation bug, and
    /// losing the dirty page silently would hide it.
    fn note_fast_store(&mut self, addr: u64, bytes: u64) {
        let clocks = self.exec.variant().bus_clocks();
        self.exec.charge(clocks);
        self.cur_access = true;
        self.bus_locked = false;
        let phys = self
            .exec
            .state
            .tlb
            .shadow_mut()
            .and_then(|shadow| shadow.note_fast_store(addr, bytes));
        let Some(phys) = phys else {
            debug_assert!(false, "an inlined store used an entry that holds no RAM");
            return;
        };
        self.exec.mem.monitor().note_store(phys, bytes);
        self.exec.note_write(phys);
        self.note_writes();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{RamStore, Region};
    use crate::cpu::x86::differential::{self, Case};
    use crate::cpu::x86::paging::pte;
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

    /// A one-register block that asserts this core's `INTR` pin when it is
    /// read, and does nothing else.
    #[derive(Debug)]
    struct RaiseOnRead {
        cpu: crate::core::sync::Mutex<alloc::sync::Weak<X86>>,
    }

    impl crate::core::space::MemOps for RaiseOnRead {
        fn read(&self, _offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
            dst.fill(0);
            if attrs.debug {
                return Ok(());
            }
            if let Some(cpu) = self.cpu.lock().upgrade() {
                cpu.set_intr_vector(0x20);
                cpu.set_intr(true);
            }
            Ok(())
        }

        fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
            if let Some(cpu) = self.cpu.lock().upgrade() {
                cpu.set_intr_vector(0x20);
                cpu.set_intr(true);
            }
            Ok(())
        }
    }

    /// `program` at [`differential::BASE`], with [`RaiseOnRead`] on the last
    /// page of the data segment.
    fn raising_core(engine: Engine, program: &[u8]) -> Arc<X86> {
        let case = Case::new(program.to_vec()).with_eflags(flags::ALWAYS_SET | flags::IF);
        let ram = Arc::new(RamStore::new(RAISER_RAM));
        for (n, byte) in program.iter().enumerate() {
            ram.write_u8(n as u64, *byte).expect("the program fits");
        }
        let dev = Arc::new(RaiseOnRead {
            cpu: crate::core::sync::Mutex::with_rank(
                crate::core::sync::LockRank::LEAF,
                alloc::sync::Weak::new(),
            ),
        });
        let space = AddressSpace::new("mem", 32);
        {
            let mut topo = space.topology();
            topo.map(Region::ram("ram", ram), differential::BASE)
                .expect("one region maps");
            topo.map(
                Region::io(
                    "raiser",
                    0x1000,
                    Arc::clone(&dev) as Arc<dyn crate::core::space::MemOps>,
                ),
                differential::BASE + RAISER_RAM,
            )
            .expect("it does not overlap the RAM");
        }
        let cpu = Arc::new(core(&case, Arc::new(space), engine));
        *dev.cpu.lock() = Arc::downgrade(&cpu);
        cpu
    }

    /// How much plain RAM [`raising_core`] maps before the device page.
    const RAISER_RAM: u64 = 3 * 4096;

    /// A load from a device that raises `INTR`, three instructions from the
    /// end of a lifted trace.
    ///
    /// ```text
    ///   top:
    ///     8b 1d 00 30 00 00   mov ebx, [0x3000]   ; the device: raises INTR
    ///     40                  inc eax
    ///     40                  inc eax
    ///     40                  inc eax
    ///     eb f5               jmp top
    /// ```
    const RAISER: [u8; 11] = [
        0x8b, 0x1d, 0x00, 0x30, 0x00, 0x00, // mov ebx, [0x3000]
        0x40, // inc eax
        0x40, // inc eax
        0x40, // inc eax
        0xeb, 0xf5, // jmp top
    ];

    /// The same, with a **store**. A store is only the last instruction of its
    /// block under [`Smc::EndBlock`], which `admit` picks under paging; this
    /// case is unpaged, so the policy is [`Smc::Guard`] and the three `inc`s
    /// below are in the same block as the write that raises `INTR`.
    ///
    /// ```text
    ///   top:
    ///     89 05 00 30 00 00   mov [0x3000], eax   ; the device: raises INTR
    ///     40                  inc eax
    ///     40                  inc eax
    ///     40                  inc eax
    ///     eb f5               jmp top
    /// ```
    const RAISER_STORE: [u8; 11] = [
        0x89, 0x05, 0x00, 0x30, 0x00, 0x00, // mov [0x3000], eax
        0x40, // inc eax
        0x40, // inc eax
        0x40, // inc eax
        0xeb, 0xf5, // jmp top
    ];

    /// Both engines against the interpreter on `program`, which raises `INTR`
    /// from inside a block.
    fn agree_on_a_raiser(program: &[u8]) {
        for engine in [Engine::Jit, Engine::JitHost] {
            let interp = raising_core(Engine::Interp, program);
            let jit = raising_core(engine, program);
            for n in 0..4 {
                let a = interp.run_budget(4_000);
                let b = jit.run_budget(4_000);
                assert_eq!(a, b, "quantum {n}: different budgets under {engine:?}");
            }
            assert!(
                interp.is_halted(),
                "the fixture never took the interrupt, so it tested nothing"
            );
            assert_eq!(
                interp.regs().qword(0),
                jit.regs().qword(0),
                "EAX under {engine:?}: the interpreter says {:#x}, the JIT says {:#x}",
                interp.regs().qword(0),
                jit.regs().qword(0),
            );
            assert_eq!(interp.regs().rip, jit.regs().rip, "RIP under {engine:?}");
            assert_eq!(interp.cycles(), jit.cycles(), "cycles under {engine:?}");
        }
    }

    /// A legacy page **directory** that lives in a device rather than in DRAM,
    /// and asserts `INTR` on the `at`-th walk that reads it.
    ///
    /// Not a hypothetical shape. `CR3` is a guest-written register and
    /// `paging::walk` masks it rather than range-checking it, and the entries
    /// come back through `Exec::phys_read` — the ordinary address space, with
    /// `MemAttrs::debug` clear — so whatever is mapped there answers and gets
    /// to do what it likes. Every `pc-at` and `q35` board maps `pc.hpet` at
    /// `0xfed0_0000` with constraints of `U32` and `U64`, which is exactly the
    /// width of a legacy or an IA-32e descriptor read; the HPET is lazily
    /// advanced, so answering catches it up to this core and requests a vector
    /// for any comparator that expired. That is this device, with the
    /// incidental parts removed.
    #[derive(Debug)]
    struct DirectoryThatRaises {
        cpu: crate::core::sync::Mutex<alloc::sync::Weak<X86>>,
        /// How many walks have read it, and which one raises.
        seen: crate::core::sync::AtomicU64,
        at: u64,
    }

    impl crate::core::space::MemOps for DirectoryThatRaises {
        fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
            // Only index 0 is ever asked for: every linear address this
            // fixture uses is below 4 MiB, so `linear >> 22` is zero.
            let word = if offset == 0 {
                differential::PTAB | pte::PRESENT | pte::WRITABLE
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
                cpu.set_intr_vector(0x20);
                cpu.set_intr(true);
            }
            Ok(())
        }

        /// The walk's accessed-bit write-back lands here and is dropped, so
        /// the bit never sticks and every walk writes it again. Deterministic,
        /// identical under both engines, and deliberately silent: this fixture
        /// is about what a *read* can do.
        fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
            Ok(())
        }
    }

    /// Where [`DirectoryThatRaises`] is mapped, and therefore what `CR3`
    /// points at: just past the eight physical pages a paged case uses, so
    /// nothing overlaps.
    const PDIR_DEV: u64 = differential::BASE + differential::PAGED_RAM_SIZE;

    /// A paged core whose page directory is [`DirectoryThatRaises`].
    ///
    /// The physical layout is `differential::machine`'s for a paged case —
    /// the page table at [`differential::PTAB`], the four guest pages at
    /// [`differential::PAGED_PROGRAM`] — with the *directory* moved out of
    /// RAM and into the device. `map_pages`' own directory entry is the one
    /// thing this does not write, because the device is it.
    fn walking_core(engine: Engine, at: u64) -> Arc<X86> {
        let case = Case::new(alloc::vec![])
            .paged()
            .with_eflags(flags::ALWAYS_SET | flags::IF);
        let ram = Arc::new(RamStore::new(differential::PAGED_RAM_SIZE));
        let put = |at: u64, bytes: &[u8]| {
            for (n, byte) in bytes.iter().enumerate() {
                ram.write_u8(at - differential::BASE + n as u64, *byte)
                    .expect("inside the physical RAM");
            }
        };
        put(differential::PAGED_PROGRAM, &INVLPG_LOOP);
        put(differential::PAGED_PROGRAM + 0x1000, &INVLPG_LOOP_TAIL);
        // The page table the device's one directory entry points at: the four
        // guest pages, identity in the sense that matters — linear `BASE + k`
        // pages onto physical `PAGED_PROGRAM + k` pages, which is what
        // `differential::map_pages` writes.
        let first = (differential::BASE >> 12) & 0x3ff;
        for k in 0..differential::RAM_SIZE / 4096 {
            let entry = (differential::PAGED_PROGRAM + k * 4096) | pte::PRESENT | pte::WRITABLE;
            put(
                differential::PTAB + 4 * (first + k),
                &(entry as u32).to_le_bytes(),
            );
        }
        let dir = Arc::new(DirectoryThatRaises {
            cpu: crate::core::sync::Mutex::with_rank(
                crate::core::sync::LockRank::LEAF,
                alloc::sync::Weak::new(),
            ),
            seen: crate::core::sync::AtomicU64::new(0),
            at,
        });
        let space = AddressSpace::new("mem", 32);
        {
            let mut topo = space.topology();
            topo.map(Region::ram("ram", ram), differential::BASE)
                .expect("one region maps");
            topo.map(
                Region::io(
                    "directory",
                    0x1000,
                    Arc::clone(&dir) as Arc<dyn crate::core::space::MemOps>,
                ),
                PDIR_DEV,
            )
            .expect("it does not overlap the RAM");
        }
        let cpu = Arc::new(core(&case, Arc::new(space), engine));
        let mut sys = cpu.sys();
        sys.cr3 = PDIR_DEV;
        cpu.set_sys(sys);
        *dir.cpu.lock() = Arc::downgrade(&cpu);
        cpu
    }

    /// A loop that flushes its own translations and then runs **across a page
    /// boundary**, so both of the two places a cold walk can happen are on it.
    ///
    /// `INVLPG` is outside the lifted subset, so `advance` interprets it and
    /// returns; the next call starts at `BASE + 0x0f` with nothing in the
    /// translation buffer for this page and walks in [`admit`]'s **prologue**.
    /// The `jmp` then leaves the page, and the successor's entry translation
    /// is a second cold walk — this one inside `Frontend::enter`, at a
    /// *chained* boundary, which is a different call site with the same
    /// window.
    ///
    /// ```text
    ///   top:                            ; linear BASE
    ///     40                inc eax
    ///     0f 01 3d 00100000 invlpg [0x1000]   ; the next page
    ///     0f 01 3d 00000000 invlpg [0]        ; this one, from on top of it
    ///     43                inc ebx
    ///     43                inc ebx
    ///     e9 ea0f0000       jmp  far          ; to BASE + 0x1000
    /// ```
    const INVLPG_LOOP: [u8; 22] = [
        0x40, // inc eax
        0x0f, 0x01, 0x3d, 0x00, 0x10, 0x00, 0x00, // invlpg [ds:0x1000]
        0x0f, 0x01, 0x3d, 0x00, 0x00, 0x00, 0x00, // invlpg [ds:0]
        0x43, // inc ebx
        0x43, // inc ebx
        0xe9, 0xea, 0x0f, 0x00, 0x00, // jmp BASE + 0x1000
    ];

    /// The other page of [`INVLPG_LOOP`], at linear `BASE + 0x1000`.
    ///
    /// ```text
    ///   far:
    ///     41 41 41          inc ecx  (three times)
    ///     e9 f8efffff       jmp top
    /// ```
    const INVLPG_LOOP_TAIL: [u8; 8] = [
        0x41, 0x41, 0x41, // inc ecx
        0xe9, 0xf8, 0xef, 0xff, 0xff, // jmp BASE
    ];

    #[test]
    fn a_walk_that_raises_intr_is_taken_where_the_interpreter_takes_it() {
        // The window `admit` closes with `Admitted::leave`, and the one the
        // module docs used to argue away with "on a PC that is empty — page
        // tables live in DRAM". Where a *board* puts them is not the question:
        // `CR3` is the guest's, and nothing between it and the bus says a
        // descriptor has to come out of RAM.
        //
        // `admit` asks the pins and *then* charges the entry fetch
        // translation, so the walk in between used to be unwatched: `INTR`
        // came up, nothing looked again, and the block ran on to its natural
        // end with `RIP` naming an instruction the interpreter had already
        // passed. Sweeping which walk raises puts the interrupt on the
        // prologue's translation and on a chained boundary's in turn.
        for engine in [Engine::Jit, Engine::JitHost] {
            let mut taken = 0usize;
            for at in 0..8u64 {
                let interp = walking_core(Engine::Interp, at);
                let jit = walking_core(engine, at);
                for n in 0..6 {
                    let a = interp.run_budget(4_000);
                    let b = jit.run_budget(4_000);
                    assert_eq!(
                        a, b,
                        "walk {at}, quantum {n}: different budgets under {engine:?}"
                    );
                }
                if interp.is_halted() {
                    taken += 1;
                }
                assert_eq!(
                    interp.regs().rip,
                    jit.regs().rip,
                    "walk {at} under {engine:?}: RIP {:#x} against {:#x}",
                    interp.regs().rip,
                    jit.regs().rip,
                );
                for r in 0..4u8 {
                    assert_eq!(
                        interp.regs().qword(r),
                        jit.regs().qword(r),
                        "walk {at}, register {r} under {engine:?}"
                    );
                }
                assert_eq!(
                    interp.cycles(),
                    jit.cycles(),
                    "walk {at}: cycles under {engine:?}"
                );
                assert_eq!(
                    interp.is_halted(),
                    jit.is_halted(),
                    "walk {at}: whether the core stopped, under {engine:?}"
                );
                let stats = jit.jit_stats().expect("a JIT core keeps statistics");
                assert!(stats.blocks > 0, "walk {at}: no block ran under {engine:?}");
            }
            assert!(
                taken > 0,
                "the fixture never took the interrupt under {engine:?}, so it \
                 proves nothing"
            );
        }
    }

    #[test]
    fn a_load_that_raises_intr_is_taken_where_the_interpreter_takes_it() {
        agree_on_a_raiser(&RAISER);
    }

    #[test]
    fn a_store_that_raises_intr_is_taken_where_the_interpreter_takes_it() {
        agree_on_a_raiser(&RAISER_STORE);
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
        agree_poking(case, engine, budget, quanta, |_, _| {})
    }

    /// The same, with `poke` applied to **both** cores before each quantum.
    ///
    /// Everything the interpreter decides before it decodes — a pending reset,
    /// an `NMI`, a maskable interrupt, the trap flag — arrives from outside the
    /// register file, so a comparison that can only run a program cannot reach
    /// any of it. This is how: whatever `poke` does to one core it does to the
    /// other, at the same instant in each one's own run.
    fn agree_poking(
        case: &Case,
        engine: Engine,
        budget: u64,
        quanta: usize,
        poke: impl Fn(&X86, usize),
    ) -> X86 {
        let (space_a, ram_a) = differential::machine(case);
        let (space_b, ram_b) = differential::machine(case);
        let interp = core(case, space_a, Engine::Interp);
        let jit = core(case, space_b, engine);
        for n in 0..quanta {
            poke(&interp, n);
            poke(&jit, n);
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
            // `CR2` is latched by a page fault and by nothing else, so it says
            // whether a *translation* went wrong somewhere neither the
            // registers nor the clock would show.
            assert_eq!(
                interp.sys().cr2,
                jit.sys().cr2,
                "quantum {n}: the faulting linear address"
            );
            assert_eq!(interp.sys().cr3, jit.sys().cr3, "quantum {n}: CR3");
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

    /// A loop that never ends, so a run of many quanta is many quanta of
    /// *guest* rather than a halted processor being compared with itself.
    ///
    /// ```text
    ///   mov eax, [ecx]        ; a load
    ///   add eax, 1            ; every flag
    ///   mov [ecx], eax        ; a store, which under paging ends the block
    ///   mov ebx, [edx+4]
    ///   add ebx, eax
    ///   mov [edx+4], ebx
    ///   shl ebx, 3
    ///   cmp eax, 0x1000
    ///   jne top               ; the back edge a trace merges
    ///   jmp top
    /// ```
    ///
    /// `ECX` and `EDX` are two of the pointers `Case::seeded` aims at the data
    /// window, so the same bytes run in all three worlds.
    const BUSY_LOOP: [u8; 27] = [
        0x8b, 0x01, // mov eax, [ecx]
        0x83, 0xc0, 0x01, // add eax, 1
        0x89, 0x01, // mov [ecx], eax
        0x8b, 0x5a, 0x04, // mov ebx, [edx+4]
        0x01, 0xc3, // add ebx, eax
        0x89, 0x5a, 0x04, // mov [edx+4], ebx
        0xc1, 0xe3, 0x03, // shl ebx, 3
        0x3d, 0x00, 0x10, 0x00, 0x00, // cmp eax, 0x1000
        0x75, 0xe7, // jne top
        0xeb, 0xe5, // jmp top
    ];

    /// [`BUSY_LOOP`] in each of the four worlds `World::of` accepts: flat
    /// 32-bit protected mode, the legacy two-level paged one, long mode, and
    /// **compatibility mode** — long mode's four-level walk under a 32-bit
    /// code segment, which is the world a 64-bit kernel is in whenever it runs
    /// a 32-bit program and the one [`narrow_state_is_clean`] exists for.
    fn busy(world: usize) -> Case {
        let case = Case::seeded(BUSY_LOOP.to_vec());
        match world {
            0 => case,
            1 => case.paged(),
            2 => case.long(),
            _ => case.compat(),
        }
    }

    /// How many of them there are, so a loop cannot silently stop covering one.
    const WORLDS: usize = 4;

    /// Whether this build has a host code generator at all.
    ///
    /// `Engine::JitHost` is accepted on every target — it falls back to the
    /// portable backend where there is nothing to generate code with — so the
    /// three tests below, which exist to say the agreement was reached over the
    /// *inlined* path, have nothing to observe on such a host and would assert
    /// against a counter that can only be zero. macOS is the case in CI:
    /// `aarch64-apple-darwin` matches neither backend, because `jit::arm64`'s
    /// buffer and runtime are Linux-only.
    ///
    /// Written as a `cfg!` expression rather than a `#[cfg]` item so it is
    /// compiled in every configuration and read by plain `if`s. A `const` whose
    /// only readers sit inside `#[cfg]` blocks is *dead code* in the builds that
    /// gate them out, and `dead_code` is denied — that exact shape has broken
    /// this tree twice.
    const HOST_COMPILES: bool = cfg!(any(
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
        // Forty clocks is under what most blocks here charge end to end, so
        // nearly every block leaves part-way through at a guest instruction
        // boundary — which must give the same answer as interpreting
        // everything, and as blocks that run to a terminator.
        agree(&seeded(0x51ee), 40, 200);
        agree(&seeded64(0x2c07).paged(), 40, 200);
    }

    #[test]
    fn several_generated_programs_agree_in_every_world() {
        for seed in 0..12u64 {
            agree_on(&seeded(seed * 7 + 1), Engine::Jit, 6_000, 6);
            agree_on(&seeded(seed * 7 + 1).paged(), Engine::JitHost, 6_000, 6);
            agree_on(&seeded64(seed * 13 + 3), Engine::JitHost, 6_000, 6);
            // Compatibility mode runs the *32-bit* corpus, because the decoder
            // is driven at `Bits::B32` there.
            agree_on(&seeded(seed * 11 + 5).compat(), Engine::JitHost, 6_000, 6);
        }
    }

    // -- the inlined memory path -----------------------------------------
    //
    // The differential harnesses above already say the two engines agree; what
    // these say is that the agreement is reached over the *inlined* path rather
    // than over a plan nothing ever published, and that the three things an
    // inlined store walks past are paid.

    /// Long mode serves its memory inline, and no other world does.
    ///
    /// [`BUSY_LOOP`] in each of the four worlds `World::of` accepts: two loads
    /// and two stores per pass, none of them a read-modify-write, so every one
    /// of them is a candidate. Only long mode has segment bases the frontend
    /// may drop — see `lift::Lifter::flat` — and compatibility mode is long
    /// mode's *tables* under a 32-bit code segment, which is not the same
    /// question and is refused.
    #[test]
    fn long_mode_serves_its_memory_inline_and_no_other_world_does() {
        let jit = agree(&busy(2), 8_000, 8);
        let s = jit.jit_stats().expect("statistics");
        if HOST_COMPILES {
            assert!(s.fast_loads > 0, "the shadow was never warm for a load");
            assert!(s.fast_stores > 0, "the shadow was never warm for a store");
        } else {
            assert_eq!(
                (s.fast_loads, s.fast_stores),
                (0, 0),
                "nothing generates code on this host, so nothing may be inlined"
            );
        }
        for world in [0usize, 1, 3] {
            let jit = agree(&busy(world), 8_000, 8);
            let s = jit.jit_stats().expect("statistics");
            assert_eq!(
                (s.fast_loads, s.fast_stores),
                (0, 0),
                "world {world} has segment bases, so nothing in it may be inlined"
            );
        }
    }

    /// An inlined store marks the dirty bitmap an interpreted one marks.
    ///
    /// Generated code writes through the host address the shadow carries, and
    /// `RamStore::write_at` — which is what marks the bitmap — never runs. That
    /// bitmap is the only record a framebuffer refresh or a live snapshot has
    /// (`ROADMAP.md` §4.1), and losing it is silent in every other test here.
    ///
    /// Compared against the interpreter rather than asserted against a page
    /// number, because the page a generated program's store lands on is not
    /// this test's business and the *set* of them is exactly the invariant.
    #[test]
    fn an_inlined_store_leaves_the_dirty_pages_an_interpreted_one_leaves() {
        let case = busy(2);
        let (space_a, ram_a) = differential::machine(&case);
        let (space_b, ram_b) = differential::machine(&case);
        let interp = core(&case, space_a, Engine::Interp);
        let jit = core(&case, space_b, Engine::JitHost);
        interp.run_budget(40_000);
        jit.run_budget(40_000);
        assert!(
            !HOST_COMPILES || jit.jit_stats().expect("statistics").fast_stores > 0,
            "no store was served inline, so this proves nothing"
        );
        let pages = |ram: &RamStore| {
            let mut out = alloc::vec::Vec::new();
            ram.for_each_dirty_page(|p| out.push(p));
            out
        };
        assert_eq!(
            pages(&ram_a),
            pages(&ram_b),
            "a compiled store left a different set of pages dirty"
        );
    }

    /// And it breaks a sibling's reservation, which `SpaceView::write` does for
    /// every store that goes through the address space and nothing at all does
    /// for one that does not.
    ///
    /// One monitor slot per page of guest RAM, reserved on both machines, and
    /// the two sets of survivors compared: a reservation that outlived a store
    /// because the store happened to be compiled is a lost update in guest
    /// software on whichever core held it.
    #[test]
    fn an_inlined_store_breaks_the_reservations_an_interpreted_one_breaks() {
        let case = busy(2);
        let (space_a, _ram_a) = differential::machine(&case);
        let (space_b, _ram_b) = differential::machine(&case);
        // A kibibyte is the widest granule a slot carries
        // (`monitor::MAX_GRANULE_SHIFT`), so this is four per page.
        let reserve = |space: &Arc<AddressSpace>| {
            (0..case.ram_size() / 1024)
                .map(|granule| {
                    let slot = crate::core::space::MonitorSlot::new(Arc::clone(space), 10)
                        .expect("a monitor slot");
                    slot.reserve(differential::BASE + granule * 1024);
                    slot
                })
                .collect::<alloc::vec::Vec<_>>()
        };
        let held_a = reserve(&space_a);
        let held_b = reserve(&space_b);
        let interp = core(&case, Arc::clone(&space_a), Engine::Interp);
        let jit = core(&case, Arc::clone(&space_b), Engine::JitHost);
        interp.run_budget(40_000);
        jit.run_budget(40_000);
        assert!(
            !HOST_COMPILES || jit.jit_stats().expect("statistics").fast_stores > 0,
            "no store was served inline, so this proves nothing"
        );
        let survivors = |slots: &[crate::core::space::MonitorSlot]| {
            slots
                .iter()
                .map(|s| s.holds())
                .collect::<alloc::vec::Vec<_>>()
        };
        assert_eq!(
            survivors(&held_a),
            survivors(&held_b),
            "a compiled store left a reservation an interpreted one broke"
        );
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
    fn the_agreement_holds_at_the_budgets_a_block_leaves_part_way_through() {
        // A budget of a few thousand clocks lands *inside* a block rather than
        // between two, so this is the sweep [`Host::spent`] has to survive: a
        // block that stops one boundary early or one boundary late puts the
        // two engines on different instructions with different `State::debt`
        // for the rest of the run, and nothing else here would notice.
        //
        // The fixture is [`BUSY_LOOP`] rather than a generated program for a
        // reason worth writing down: a generated case runs twenty-four
        // instructions and halts, so every quantum after the first compares a
        // stopped processor with itself and the budget never decides anything.
        // That is how three mutations of the bound this replaced survived a
        // sweep that looked thorough.
        for budget in [1_400u64, 1_900, 2_600, 3_400, 4_200, 5_600, 7_000] {
            for world in 0..WORLDS {
                agree_on(&busy(world), Engine::Jit, budget, 24);
                agree_on(&busy(world), Engine::JitHost, budget, 24);
            }
        }
    }

    #[test]
    fn a_block_given_a_whole_quantum_retires_more_than_one_instruction() {
        // The other side of the allowance, and the only assertion in this file
        // about the *shape* of a run rather than its result.
        //
        // A [`Host::spent`] that answered `true` at every boundary would be
        // **correct**: a block that retires one guest instruction and leaves is
        // exactly what the interpreter does, tick for tick, so `agree_on`
        // passes in every column and the engine is silently an interpreter
        // with a translator bolted on. A mutation sweep found it and nothing
        // else in the tree caught it.
        //
        // Eight thousand ticks is a whole scheduler quantum and every fixture
        // here loops, so a block really does run to a terminator. Two is a
        // floor with room under it: on a real 64-bit kernel a block covers
        // five guest instructions, and under [`Smc::EndBlock`] — the policy
        // paging forces, and the worst case here — [`BUSY_LOOP`]'s two stores
        // cut its ten instructions into pieces of three and six.
        for world in 0..WORLDS {
            for engine in [Engine::Jit, Engine::JitHost] {
                let jit = agree_on(&busy(world), engine, 8_000, 8);
                let stats = jit.jit_stats().expect("statistics");
                assert!(stats.blocks > 0, "world {world}: no block ran");
                assert!(
                    stats.retired > 2 * stats.blocks,
                    "world {world} under {engine:?}: {} instructions retired over {} \
                     blocks — a block is leaving at its first boundary rather than \
                     running",
                    stats.retired,
                    stats.blocks,
                );
            }
        }
    }

    #[test]
    fn a_budget_no_whole_block_fits_in_still_retires_inside_one() {
        // What adopting [`IrHost::spent`] is *for*, asserted rather than left
        // to a boot log. Two thousand ticks is less than the cold worst case
        // the budget guard used to demand under paging (3 312) and more than
        // any block in [`BUSY_LOOP`] really spends, which is exactly the
        // window that guard closed: a boundary reached with less than the cold
        // bound left was interpreted, and the lift that would have contradicted
        // the bound was the thing that never ran. On a real 64-bit kernel that
        // was 2.02% of every guest instruction — three times everything
        // outside the lifted subset put together.
        //
        // A guard restored in any form fails this: `retired` collapses to zero
        // for the paged worlds, because every block there is refused.
        for world in [1, 2, 3] {
            for engine in [Engine::Jit, Engine::JitHost] {
                let jit = agree_on(&busy(world), engine, 2_000, 24);
                let stats = jit.jit_stats().expect("statistics");
                assert!(
                    stats.retired > stats.interpreted,
                    "world {world} under {engine:?}: {} retired against {} interpreted — a \
                     block is being refused for a budget it could have left part-way \
                     through",
                    stats.retired,
                    stats.interpreted,
                );
                // And nothing is re-translated to find that out. A block that
                // leaves is still in the cache, so a loop whose body is cached
                // is lifted a handful of times over twenty-four quanta rather
                // than once per pass.
                assert!(
                    stats.translated < 64,
                    "world {world} under {engine:?}: {} translations for {} blocks -- a \
                     block that left part-way through is being re-lifted",
                    stats.translated,
                    stats.blocks,
                );
            }
        }
    }

    /// `EFLAGS` with the trap flag set, which makes every instruction a
    /// single-step exception.
    fn with_trap_flag(case: Case) -> Case {
        case.with_eflags(flags::ALWAYS_SET | flags::TF)
    }

    #[test]
    fn the_trap_flag_takes_the_instruction_away_from_the_block() {
        // `Exec::step` samples `TF` before it decodes and delivers the debug
        // exception *after* the instruction, so a block of sixteen would trap
        // fifteen instructions late. The interrupt table's limit is zero here,
        // so the first one shuts the processor down — at a cycle count both
        // engines have to agree on.
        agree(&with_trap_flag(busy(0)), 8_000, 4);
        agree(&with_trap_flag(busy(2)), 8_000, 4);
    }

    #[test]
    fn a_pending_interrupt_is_taken_at_the_instruction_the_interpreter_takes_it_at() {
        // Three pins, each raised part-way through a run on both cores at the
        // same instant: a maskable interrupt with `IF` set, a non-maskable
        // one, and a reset request. Every one of them is something
        // `Exec::step` acts on before it decodes, and a block admitted with
        // one pending would take it up to sixteen instructions late.
        let case = busy(0).with_eflags(flags::ALWAYS_SET | flags::IF);
        for engine in [Engine::Jit, Engine::JitHost] {
            agree_poking(&case, engine, 4_000, 8, |cpu, n| {
                if n == 2 {
                    cpu.set_intr_vector(0x20);
                    cpu.set_intr(true);
                }
            });
            agree_poking(&case, engine, 4_000, 8, |cpu, n| {
                if n == 2 {
                    cpu.pulse_nmi();
                }
            });
            agree_poking(&case, engine, 4_000, 8, |cpu, n| {
                if n == 2 {
                    cpu.request_reset();
                }
            });
        }
    }

    /// A long-mode loop whose every pass makes a **computed** near transfer.
    ///
    /// ```text
    ///   top:
    ///     48 ff c0            inc rax
    ///     e8 02 00 00 00      call +2       ; the subroutine below
    ///     eb f6               jmp top
    ///     c3                  ret
    /// ```
    ///
    /// `RET` was 34.2 M of the 46 M instructions this engine handed back on a
    /// 900-second `pc64` boot — a function return is the commonest computed
    /// transfer there is — and until `lift::TRANSFER` it ended a block in long
    /// mode and cost a round trip through the interpreter. Every pass here
    /// makes one, so a block really does execute the check.
    const CALL_RET: [u8; 11] = [
        0x48, 0xff, 0xc0, // inc rax
        0xe8, 0x02, 0x00, 0x00, 0x00, // call +2
        0xeb, 0xf6, // jmp top
        0xc3, // ret
    ];

    #[test]
    fn a_computed_near_transfer_retires_inside_a_block_in_long_mode() {
        let case = Case::seeded(CALL_RET.to_vec()).long();
        let jit = agree(&case, 8_000, 8);
        let stats = jit.jit_stats().expect("statistics");
        assert!(stats.blocks > 0, "no block ran, so nothing was compared");
        // The loop is four instructions and every one of them is inside the
        // subset, so the overwhelming majority must retire in a block. A
        // frontend that had gone back to refusing the `RET` would still agree
        // with the interpreter here and would fail this.
        assert!(
            stats.retired > stats.interpreted * 4,
            "{} retired against {} interpreted: the return is not being lifted",
            stats.retired,
            stats.interpreted
        );
    }

    #[test]
    fn a_non_canonical_computed_transfer_faults_the_same_in_every_engine() {
        // `movabs r15, 0x1234_5678_9abc_def0` — bits 63..47 are not all equal,
        // so it is not a canonical address — then `jmp r15`. `Exec::jump_near`
        // raises `#GP(0)` at the transfer with the pre-instruction state
        // restored; the fixture's interrupt table has a limit of zero, so the
        // first exception escalates and shuts the processor down, which is a
        // compared column.
        let program = alloc::vec![
            0x49, 0xbf, 0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x41, 0xff, 0xe7,
        ];
        let case = Case::seeded(program).long();
        let jit = agree(&case, 8_000, 4);
        assert!(
            jit.is_halted(),
            "the fixture never faulted, so it tested nothing"
        );
    }

    #[test]
    fn a_narrow_world_with_a_dirty_upper_half_is_left_to_the_interpreter() {
        // The precondition `lift::Lifter::read_reg` has below long mode: it
        // reads a 32-bit operand as the whole slot, while `Regs::dword`
        // truncates. Compatibility mode is where the two stop agreeing, and
        // this is that state written down — a 64-bit kernel's leftovers still
        // in the file when a 32-bit code segment runs.
        let case = seeded(0x7c3a);
        let (space_a, ram_a) = differential::machine(&case);
        let (space_b, ram_b) = differential::machine(&case);
        let interp = core(&case, space_a, Engine::Interp);
        let jit = core(&case, space_b, Engine::JitHost);
        for cpu in [&interp, &jit] {
            let mut regs = cpu.regs();
            for n in [0u8, 1, 2, 5, 6, 7] {
                regs.set_qword(n, regs.qword(n) | 0x0000_00ff_0000_0000);
            }
            cpu.set_regs(regs);
        }
        for n in 0..8 {
            assert_eq!(interp.run_budget(8_000), jit.run_budget(8_000));
            let (want, got) = (interp.regs(), jit.regs());
            for r in 0..16u8 {
                assert_eq!(want.qword(r), got.qword(r), "quantum {n}, register {r}");
            }
            assert_eq!(want.eflags, got.eflags, "quantum {n}: EFLAGS");
            assert_eq!(interp.cycles(), jit.cycles(), "quantum {n}: the clock");
            memory_agrees(&ram_a, &ram_b, n);
        }
    }

    /// Two programs at the same `EIP` in two different code segments.
    ///
    /// `A` and `B` differ only in the constant they load, and each is a loop,
    /// so which one a core is executing is visible in `EAX` forever.
    /// `add eax, imm8` closed by a `jmp` back to itself: the constant
    /// **accumulates**, so which code segment's block ran is visible in `EAX`
    /// however many instructions the interpreter takes afterwards.
    ///
    /// A `mov` of the constant would not do, and the first draft of this test
    /// was one: the interpreter's own instruction at the end of a quantum
    /// wrote the *right* answer over the stale block's wrong one, and the
    /// fixture reported agreement while the engine served a translation from a
    /// code segment that no longer existed.
    const WORLD_A: [u8; 5] = [0x83, 0xc0, 0x11, 0xeb, 0xfb];
    const WORLD_B: [u8; 5] = [0x83, 0xc0, 0x22, 0xeb, 0xfb];

    #[test]
    fn a_block_lifted_in_one_world_is_not_served_in_another() {
        // `lift::key` folds `World::generation` into the cache key under
        // `Origin::Flat`, and nothing in the crate keeps that counter —
        // `Boundary::world_of` does, by comparing. A counter that never moved
        // would serve `WORLD_A`'s block at `EIP` zero after `CS.base` had been
        // moved to `WORLD_B`, which is a stale translation with no store to
        // invalidate it and no page to invalidate it by.
        let case = Case::new(WORLD_A.to_vec());
        let (space_a, _a) = differential::machine(&case);
        let (space_b, _b) = differential::machine(&case);
        for space in [&space_a, &space_b] {
            for (n, byte) in WORLD_B.iter().enumerate() {
                space
                    .write(
                        differential::BASE + 0x2000 + n as u64,
                        Width::U8,
                        u64::from(*byte),
                        MemAttrs::DEFAULT,
                    )
                    .expect("in RAM");
            }
        }
        let interp = core(&case, space_a, Engine::Interp);
        let jit = core(&case, space_b, Engine::JitHost);
        for (world, base) in [(0u64, 0u64), (1, 0x2000)] {
            for cpu in [&interp, &jit] {
                let mut sys = cpu.sys();
                sys.segs[usize::from(seg::CS)].base = differential::BASE + base;
                cpu.set_sys(sys);
                let mut regs = cpu.regs();
                regs.rip = 0;
                cpu.set_regs(regs);
            }
            for n in 0..6 {
                assert_eq!(interp.run_budget(6_000), jit.run_budget(6_000));
                assert_eq!(
                    interp.regs().qword(0),
                    jit.regs().qword(0),
                    "world {world}, quantum {n}: the constant the loop loads says which \
                     code segment's block ran"
                );
                assert_eq!(interp.cycles(), jit.cycles());
            }
        }
        assert!(
            interp.regs().qword(0) > 0x1000,
            "the fixture never ran for long enough to accumulate anything"
        );
    }

    /// A loop that stores through `SS` and loads it back.
    ///
    /// `89 04 24` and `8b 1c 24` use `ESP` as a SIB base, which selects the
    /// **stack** segment rather than `DS` — so with the two segments at
    /// different bases the bytes land somewhere a `DS` access would never
    /// reach, and guest RAM says which one happened.
    /// `EBX` **accumulates** what the load read, for the reason
    /// [`WORLD_A`] spells out: a register the loop merely overwrites is
    /// repaired by the interpreter's own next instruction at the end of a
    /// quantum, and the comparison then agrees while the engine has been
    /// reading through the wrong segment all along.
    const THROUGH_SS: [u8; 10] = [
        0x89, 0x04, 0x24, // mov [esp], eax
        0x03, 0x1c, 0x24, // add ebx, [esp]
        0xff, 0xc0, // inc eax
        0xeb, 0xf6, // jmp back to the top
    ];

    #[test]
    fn an_access_goes_through_the_segment_the_frontend_named() {
        // `MemOp::seg` carries the register rather than the frontend folding a
        // base in, because the fault differs by register and the base is
        // hidden state. A host that ignored it would be right on every machine
        // whose segments share a base, which is every fixture in
        // `differential` — so this one moves `SS`.
        // `EAX` starts at something that is not zero, because guest RAM starts
        // at zero and a store of zero to the wrong address is invisible in it.
        let case = Case::new(THROUGH_SS.to_vec()).with_reg(0, 0x1234_5678);
        let (space_a, ram_a) = differential::machine(&case);
        let (space_b, ram_b) = differential::machine(&case);
        let interp = core(&case, space_a, Engine::Interp);
        let jit = core(&case, space_b, Engine::JitHost);
        for cpu in [&interp, &jit] {
            let mut sys = cpu.sys();
            sys.segs[usize::from(seg::SS)].base = differential::BASE + 0x1000;
            cpu.set_sys(sys);
        }
        for n in 0..8 {
            assert_eq!(interp.run_budget(6_000), jit.run_budget(6_000));
            assert_eq!(interp.cycles(), jit.cycles(), "quantum {n}: the clock");
            assert_eq!(
                interp.regs().qword(3),
                jit.regs().qword(3),
                "quantum {n}: what the load read back says which segment it went through"
            );
            memory_agrees(&ram_a, &ram_b, n);
        }
        assert!(
            jit.jit_stats().expect("statistics").blocks > 0,
            "no block ran, so the host's segment handling was never reached"
        );
        assert!(
            interp.regs().qword(3) > 0x1000,
            "the fixture never accumulated anything"
        );
    }

    /// A loop that rewrites the immediate of its own first instruction.
    ///
    /// ```text
    ///   b8 05 00 00 00      mov eax, 5        ; the byte at +1 is the target
    ///   83 c0 01            add eax, 1
    ///   a3 00 11 00 00      mov [0x1100], eax ; a store on another page
    ///   88 05 01 00 00 00   mov [0x1], al     ; and one into this very block
    ///   eb eb               jmp back
    /// ```
    ///
    /// Every pass reads the immediate the pass before it wrote, so a stale
    /// translation is visible in `EAX` immediately and in guest RAM after it.
    const SELF_MODIFYING: [u8; 21] = [
        0xb8, 0x05, 0x00, 0x00, 0x00, // mov eax, 5
        0x83, 0xc0, 0x01, // add eax, 1
        0xa3, 0x00, 0x11, 0x00, 0x00, // mov [0x1100], eax
        0x88, 0x05, 0x01, 0x00, 0x00, 0x00, // mov [0x1], al
        0xeb, 0xeb, // jmp back to the top
    ];

    #[test]
    fn a_store_into_the_running_page_is_honoured_by_the_next_block() {
        // All three worlds, because [`SMC`] is now one mechanism in all of
        // them: [`Host::note_writes`] compares the guest-physical page a store
        // reached against the physical page the block's own bytes came from,
        // retires the run's allowance, and the dispatcher's page drain at the
        // next boundary is what throws the translation away. Physical at both
        // ends, which is the part a linear guard could not be — and the reason
        // the paged and long rows are the same mechanism as the flat one
        // rather than [`Smc::EndBlock`] standing in for it.
        agree(&Case::new(SELF_MODIFYING.to_vec()), 8_000, 12);
        agree(&Case::new(SELF_MODIFYING.to_vec()).paged(), 8_000, 12);
        agree(&Case::new(SELF_MODIFYING.to_vec()).long(), 8_000, 12);
    }

    /// A store that misses the code page costs no block boundary — with paging
    /// **on**, where it used to cost one every time.
    ///
    /// The `agree` fixtures above prove the two engines match; nothing in them
    /// would notice [`SMC`] quietly reverting to [`Smc::EndBlock`], because
    /// ending a block more often is *correct* and only slower. This is the
    /// number that would move, on the engine rather than in the harness: the
    /// loop below stores to a fixed address on another page once per pass, and
    /// under [`Smc::EndBlock`] each of those was a block of its own.
    #[test]
    fn a_paged_store_that_misses_the_code_page_costs_no_boundary() {
        // ```text
        //   83 c0 01            add eax, 1
        //   a3 00 11 00 00      mov [0x1100], eax   ; another page entirely
        //   83 c3 01            add ebx, 1
        //   eb f5               jmp back to the top
        // ```
        const LOOP: [u8; 13] = [
            0x83, 0xc0, 0x01, // add eax, 1
            0xa3, 0x00, 0x11, 0x00, 0x00, // mov [0x1100], eax
            0x83, 0xc3, 0x01, // add ebx, 1
            0xeb, 0xf3, // jmp back
        ];
        let case = Case::new(LOOP.to_vec()).paged();
        let (space, _ram) = differential::machine(&case);
        let cpu = core(&case, space, Engine::JitHost);
        cpu.run_budget(20_000);
        let stats = cpu
            .jit_stats()
            .expect("a translated engine keeps statistics");
        assert!(stats.blocks > 0, "no block ran, so nothing was measured");
        // Three guest instructions and a taken branch per pass, all in the
        // entry page, so a trace covers whole passes. Under `Smc::EndBlock`
        // the store ended one block per pass and the ratio could not exceed
        // two; the assertion is deliberately well under what was measured
        // (about ten) so it is a policy check rather than a tuning check.
        let per_block = stats.retired as f64 / stats.blocks as f64;
        assert!(
            per_block > 4.0,
            "{per_block:.2} guest instructions per block: a store is ending the block again              ({stats:?})"
        );
    }

    #[test]
    fn a_non_canonical_program_counter_is_the_interpreters_general_protection() {
        // `Exec::fetch_at` checks the canonical form *before* it translates
        // anything and raises `#GP(0)` rather than `#PF`, so a boundary that
        // went straight to the page tables would translate an address the
        // processor never had — and here it would succeed, because the low
        // forty-eight bits of this one are the window the fixture maps.
        let case = busy(2);
        let (space_a, _a) = differential::machine(&case);
        let (space_b, _b) = differential::machine(&case);
        let interp = core(&case, space_a, Engine::Interp);
        let jit = core(&case, space_b, Engine::JitHost);
        for cpu in [&interp, &jit] {
            let mut regs = cpu.regs();
            regs.rip = (1u64 << 48) | differential::BASE;
            cpu.set_regs(regs);
        }
        for n in 0..4 {
            assert_eq!(interp.run_budget(6_000), jit.run_budget(6_000));
            assert_eq!(interp.regs().rip, jit.regs().rip, "quantum {n}: RIP");
            assert_eq!(interp.cycles(), jit.cycles(), "quantum {n}: the clock");
            assert_eq!(interp.sys().cr2, jit.sys().cr2, "quantum {n}: CR2");
            assert_eq!(interp.is_halted(), jit.is_halted(), "quantum {n}: halted");
        }
        assert!(
            interp.is_halted(),
            "the fixture never faulted, so it tested nothing"
        );
    }
}
