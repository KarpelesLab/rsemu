//! The boundary between generated code and Rust: the execution context, the
//! thunk table, and the engine that enters a compiled block.
//!
//! # Why this file also opts into `unsafe`
//!
//! It is the **same** sanctioned subsystem as [`buf`](super::buf) — the JIT
//! code buffer (`ROADMAP.md` §0) — seen from the other side. A code buffer
//! that could be written and made executable but never *called* would be an
//! ornament, and a call from machine code back into a Rust `IrHost` cannot be
//! expressed without reconstituting `&mut` references from the pointers the
//! generated code was handed. Splitting the two files does not split the
//! subsystem; it separates *mapping memory* from *crossing the boundary*, so
//! each invariant is stated next to the code that upholds it.
//!
//! Nothing else in `jit/` contains `unsafe`, and nothing in `ir/` does.
//!
//! # The contract generated code is compiled against
//!
//! One argument, in `rdi`: a pointer to a [`Ctx`]. One result, in `rax`: a
//! [`status`] code. Everything else is reached through the context, which is why
//! [`Entry`](super::buf::Entry) does not have to know what a guest is.
//!
//! [`Ctx`] is `#[repr(C)]` and its field offsets are compiled in as
//! immediates. `the_context_layout_a_code_generator_bakes_in_is_the_one_rust_built`
//! asserts every one of them against `offset_of!`, so adding a field is a test
//! failure rather than a miscompile.
//!
//! # What the engine owes the interpreter
//!
//! `ir::Interp` is the oracle (CLAUDE.md, "CPU cores", one level down), so
//! this engine reproduces its *observable* behaviour exactly and not merely its
//! results:
//!
//! * [`IrHost::charge`] is called once per [`Opcode::CHARGE`], with that
//!   opcode's immediate, unbatched — the count is hashed output rather than a
//!   budget (`ir`'s module docs, decision 2).
//! * [`IrHost::insn_start`] is called at every boundary, after the boundary's
//!   own bookkeeping and before anything that follows it.
//! * Guest state is materialized **lazily**, at exactly the three points the
//!   interpreter lists: on the way out of a run, before a helper (which this
//!   backend refuses to compile), and before a [`Opcode::GET_SLOT`] that reads
//!   a slot the pending boundary shadows.
//! * A [`BusError::Retry`] after a commit is rejected rather than delivered.
//!
//! The first two are **deferred, not batched**, and the distinction is the
//! whole of `flush_thunk`: generated code emits nothing at a charge or a
//! boundary, and the calls happen — same count, same arguments, same order —
//! at the next point the host could observe anything. See
//! [`compile`](mod@super::compile)'s "Deferred bookkeeping" for why the range a
//! flush is handed is exactly the set of instructions that ran.

#![allow(unsafe_code)]

use alloc::vec::Vec;
use core::ffi::c_void;

use crate::core::error::{BusError, Error, Result};
use crate::ir::{Block, Fault, InsnStart, IrHost, MemOp, Opcode, Outcome, RegSlot};
use crate::jit::dispatch::{Chain, Frontend, Step, StoreLog};
use crate::jit::{BlockId, CodeRef, FastMem};

use super::buf::{CodeBuf, DEFAULT_CAPACITY};
use super::compile::{Compiled, Refusal, Regs, compile_with};

/// Why a compiled block stopped, as generated code reports it in `rax`.
///
/// A plain integer rather than an enum, because the producer is machine code.
pub mod status {
    /// `exit_tb`: back to the dispatcher, with the PC in a guest slot.
    pub const EXIT: u64 = 0;
    /// `goto_tb`: on to a statically known successor, in the context's `out_pc`.
    pub const GOTO: u64 = 1;
    /// `lookup_and_goto`: the successor is the computed PC in `out_pc`.
    pub const LOOKUP: u64 = 2;
    /// A guest access faulted; the fault fields carry where and why.
    pub const FAULT: u64 = 3;
    /// The host's tick allowance was spent at a boundary, and the block left
    /// there. The context's `boundary_pc` is where the guest is standing.
    pub const SPENT: u64 = 4;
}

/// The bus errors, as generated code passes them back.
///
/// Zero is *no error*, so every variant is one more than its position. The
/// mapping is written twice on purpose — once each way — and
/// `every_bus_error_survives_the_round_trip` checks the pair, because a
/// silently mistranslated error is a fault delivered with the wrong cause.
const fn error_code(e: BusError) -> u64 {
    match e {
        BusError::Unassigned => 1,
        BusError::BadAccess => 2,
        BusError::Protected => 3,
        BusError::Retry => 4,
    }
}

const fn error_of(code: u64) -> BusError {
    match code {
        1 => BusError::Unassigned,
        3 => BusError::Protected,
        4 => BusError::Retry,
        // Anything unaccounted for is the most conservative answer rather than
        // a panic: generated code cannot produce one, and a fuzzed buffer
        // should not be able to turn a wrong integer into a wrong *kind* of
        // fault.
        _ => BusError::BadAccess,
    }
}

/// One deferred bookkeeping event: what an [`Opcode::CHARGE`] or an
/// [`Opcode::INSN_START`] does when its region is replayed.
///
/// A compiled block carries these in a dense array rather than having
/// `flush_thunk` read the IR again, and the difference is not tidiness. An
/// [`Inst`](crate::ir::Inst) is wide — an `Option<Const>` alone is thirty-two
/// bytes — so re-walking a 215-instruction block on every one of the hundred
/// thousand times a hot trace runs streams megabytes of cold data past the
/// cache. That was measured, as a **25% loss** against the code this replaced,
/// which is the whole reason this type exists: sixteen bytes per event, in
/// order, and only the events.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// [`IrHost::charge`], with the tick count [`Opcode::CHARGE`] carried.
    Charge(u64),
    /// [`IrHost::insn_start`], by index into [`Block::marks`].
    Boundary {
        /// The index into [`Block::marks`].
        mark: u32,
        /// Whether a terminator follows this boundary, which is what makes it
        /// an **exit** boundary: the block may not be left there, because
        /// [`InsnStart::pc`] is then a static placeholder and the real
        /// successor is in the slot the map publishes. `plan` reads it off the
        /// block, once, at compile time — the replay cannot see instruction
        /// indices and must be told.
        exit: bool,
    },
}

/// The execution context a compiled block runs against.
///
/// Every field is a `u64` or a pointer, so generated code writes each one with
/// a single `mov` and no sub-register aliasing. The layout is load-bearing:
/// see the module docs.
#[repr(C)]
#[derive(Debug)]
pub struct Ctx {
    /// The temporary frame: **at least** one `u64` per
    /// [`Temp`](crate::ir::Temp) the block declares.
    ///
    /// At least, and not exactly, because [`Engine`] keeps one buffer at the
    /// high-water mark of every block it has run rather than resizing it per
    /// execution. Generated code indexes it by temporary number and never
    /// reads its length, so a longer frame is the same frame.
    pub temps: *mut u64,
    /// The thunk table.
    pub vt: *const Vtable,
    /// The `IrHost`, type-erased. Every thunk casts it back.
    pub host: *mut c_void,
    /// The block being executed, for the thunks that need its marks.
    pub block: *const Block,
    /// The software TLB's load set, or null when the host published none.
    pub tlb_base: *const u8,
    /// `entries - 1` for that set.
    pub tlb_mask: u64,
    /// Everything a TLB tag carries besides the page number.
    pub tag_bits: u64,
    /// Where a `goto_tb` or `lookup_and_goto` is going.
    pub out_pc: u64,
    /// Ticks charged by [`Opcode::CHARGE`], as `Interp::ticks` counts them.
    pub ticks: u64,
    /// [`Ctx::ticks`] as of the current boundary.
    pub retired: u64,
    /// How many boundaries the run has passed.
    pub boundaries: u64,
    /// The current boundary's guest PC.
    pub boundary_pc: u64,
    /// The current boundary's index in [`Block::marks`], or -1.
    pub mark: i64,
    /// The index of the faulting instruction.
    pub fault_at: u64,
    /// The faulting access's error, encoded so that zero means success.
    pub fault_error: u64,
    /// Whether anything has been committed since the boundary.
    pub committed: u64,
    /// Whether the boundary's live mapping has been written out.
    pub published: u64,
    /// Loads served entirely from an inlined TLB probe.
    pub fast_hits: u64,
    /// The software TLB's store set, or null when the host published none.
    ///
    /// A separate table from [`Ctx::tlb_base`], not the same one indexed
    /// differently: an entry in the store set was admitted on write permission
    /// and one in the load set on read permission, and the two are different
    /// bits in both the topology and the guest's own page tables.
    pub st_base: *const u8,
    /// `entries - 1` for that set.
    pub st_mask: u64,
    /// Everything a store's TLB tag carries besides the page number.
    pub st_tag: u64,
    /// Stores served entirely from an inlined TLB probe.
    pub fast_writes: u64,
    /// The compiled block's deferred bookkeeping, in instruction order.
    ///
    /// Generated code never names this — it passes a *range* into it — so it
    /// sits past every offset [`off`] declares.
    pub events: *const Event,
    /// How many [`Ctx::events`] there are, so a range can be clamped.
    pub event_count: u64,
    /// The direct-link thunk, as a plain address, or zero for no linking.
    ///
    /// A [`ChainFn`] the dispatcher monomorphised over its frontend *and* its
    /// host, which is why it is here and not a slot of [`Vtable`]: that table
    /// is built from the host type alone, and a block compiled against one
    /// host is meant to run against any other value of it.
    pub chain: u64,
    /// What the chain thunk is handed besides this context.
    ///
    /// Opaque here on purpose — it is the dispatcher's own control block, and
    /// only the thunk knows its type. Never touched by generated code.
    pub chain_ctl: *mut c_void,
    /// The [`status`] the block reached its exit with, parked across the call
    /// to the chain thunk.
    ///
    /// Generated code has one register to leave in and the thunk destroys it,
    /// so the status goes through the context rather than through the stack:
    /// the chain pad is reached from a fault sequence as well as from a
    /// terminator, and a stack slot would have to be live down every one of
    /// those paths.
    pub out_status: u64,
    /// How many blocks the chain executed inside generated code.
    ///
    /// The thunk cannot reach [`EngineStats`] — the engine is executing while
    /// it runs — so it counts here and [`Engine::run`] folds it in afterwards.
    pub blocks_run: u64,
}

/// Byte offsets into [`Ctx`], as generated code bakes them in.
///
/// Six fields have no entry, and their absence is the shape of this backend
/// rather than an oversight: [`Ctx::ticks`], [`Ctx::retired`],
/// [`Ctx::boundaries`], [`Ctx::boundary_pc`], [`Ctx::mark`] and
/// [`Ctx::published`] are written only by `flush_thunk`, in Rust, through
/// the struct. Generated code stopped naming them when the bookkeeping moved
/// there.
pub mod off {
    /// [`Ctx::temps`](super::Ctx::temps).
    pub const TEMPS: i32 = 0;
    /// [`Ctx::vt`](super::Ctx::vt).
    pub const VT: i32 = 8;
    /// [`Ctx::tlb_base`](super::Ctx::tlb_base).
    pub const TLB_BASE: i32 = 32;
    /// [`Ctx::tlb_mask`](super::Ctx::tlb_mask).
    pub const TLB_MASK: i32 = 40;
    /// [`Ctx::tag_bits`](super::Ctx::tag_bits).
    pub const TAG_BITS: i32 = 48;
    /// [`Ctx::out_pc`](super::Ctx::out_pc).
    pub const OUT_PC: i32 = 56;
    /// [`Ctx::fault_at`](super::Ctx::fault_at).
    pub const FAULT_AT: i32 = 104;
    /// [`Ctx::fault_error`](super::Ctx::fault_error).
    pub const FAULT_ERROR: i32 = 112;
    /// [`Ctx::committed`](super::Ctx::committed).
    pub const COMMITTED: i32 = 120;
    /// [`Ctx::fast_hits`](super::Ctx::fast_hits).
    pub const FAST_HITS: i32 = 136;
    /// [`Ctx::st_base`](super::Ctx::st_base).
    pub const ST_BASE: i32 = 144;
    /// [`Ctx::st_mask`](super::Ctx::st_mask).
    pub const ST_MASK: i32 = 152;
    /// [`Ctx::st_tag`](super::Ctx::st_tag).
    pub const ST_TAG: i32 = 160;
    /// [`Ctx::fast_writes`](super::Ctx::fast_writes).
    pub const FAST_WRITES: i32 = 168;
    /// [`Ctx::chain`](super::Ctx::chain).
    pub const CHAIN: i32 = 192;
    /// [`Ctx::out_status`](super::Ctx::out_status).
    pub const OUT_STATUS: i32 = 208;
}

/// The direct-link thunk: what a block calls at its exit instead of returning.
///
/// Handed the [`Ctx`] it is running against — [`Ctx::chain_ctl`] is how it
/// finds everything else — and answers with the host address of the
/// successor's **chain entry**, or zero for *stop here*. `ROADMAP.md` §9.1's
/// second mechanism, finished: the cache has patched exits since it existed,
/// and this is the patch reaching generated code.
///
/// # Safety
///
/// An implementation is entered from machine code, so it must be `extern
/// "sysv64"`, must not unwind, and must return either zero or the address of a
/// chain entry in a code buffer that is live and executable — see
/// `jit::dispatch`'s `chain_thunk`, which is the only implementation.
pub type ChainFn = unsafe extern "sysv64" fn(*mut c_void) -> u64;

/// What a dispatcher needs to know about an engine's code buffer to resolve a
/// link without touching the engine.
///
/// The engine is *executing* while the chain thunk runs — its `&mut self` is
/// live in [`Engine::run_chained`]'s frame — so the thunk is handed this
/// snapshot instead. Every field is stable for the whole of a run, and that is
/// a claim rather than a hope: the one thing that invalidates any of them is
/// [`Engine::compile`], and a chain never compiles.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Linkage {
    /// The code buffer's base address.
    pub base: u64,
    /// The generation a [`CodeRef`] must carry to be worth jumping to.
    pub generation: u64,
    /// How many temporaries the shared frame holds.
    ///
    /// A block needing more may not be linked to: growing the frame would move
    /// it, and the code standing on it holds its address in a register.
    pub temps: usize,
}

/// The thunks generated code calls, one table per host type.
///
/// Indirect through a table rather than an immediate call address, because the
/// addresses are monomorphized per `H` and a block compiled for one host must
/// be runnable against another of the same type without being compiled again.
///
/// [`IrHost::charge`] and [`IrHost::insn_start`] have no slot here any more.
/// Generated code never calls either one: both are reached from
/// [`Vtable::flush`], once per region rather than once per guest instruction.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Vtable {
    /// Replay a range of the block's charges and boundaries.
    ///
    /// Returns non-zero when [`IrHost::spent`] stopped the replay at a
    /// boundary, which generated code answers by leaving the block with
    /// [`status::SPENT`].
    pub flush: unsafe extern "sysv64" fn(*mut c_void, u64, u64) -> u64,
    /// [`IrHost::read_slot`], publishing first if the slot is shadowed.
    pub get_slot: unsafe extern "sysv64" fn(*mut c_void, u64) -> u64,
    /// [`IrHost::load`]. Returns an error code; the value goes to `out`.
    pub load: unsafe extern "sysv64" fn(*mut c_void, *const MemOp, u64, *mut u64) -> u64,
    /// [`IrHost::store`]. Returns an error code.
    pub store: unsafe extern "sysv64" fn(*mut c_void, *const MemOp, u64, u64) -> u64,
    /// [`FastMem::note_fast_load`], for an access the backend served itself.
    pub fast_tick: unsafe extern "sysv64" fn(*mut c_void),
    /// [`FastMem::note_fast_store`], for a store the backend served itself.
    ///
    /// Takes the guest address and the width, because everything the host
    /// still owes — the tick, the store's dirty bitmap, the guest-physical
    /// dirty log, the broken reservation — is a function of exactly those two
    /// and the table the plan named.
    pub fast_store: unsafe extern "sysv64" fn(*mut c_void, u64, u64),
}

/// Byte offsets into [`Vtable`], as generated code bakes them in.
pub mod vt {
    /// [`Vtable::flush`](super::Vtable::flush).
    pub const FLUSH: i32 = 0;
    /// [`Vtable::get_slot`](super::Vtable::get_slot).
    pub const GET_SLOT: i32 = 8;
    /// [`Vtable::load`](super::Vtable::load).
    pub const LOAD: i32 = 16;
    /// [`Vtable::store`](super::Vtable::store).
    pub const STORE: i32 = 24;
    /// [`Vtable::fast_tick`](super::Vtable::fast_tick).
    pub const FAST_TICK: i32 = 32;
    /// [`Vtable::fast_store`](super::Vtable::fast_store).
    pub const FAST_STORE: i32 = 40;
}

/// Reconstitute the context a thunk was handed.
///
/// # Safety
///
/// `ctx` must be the pointer generated code was entered with, which
/// [`Engine::run`] takes from a live `&mut Ctx` it holds for the whole call.
#[inline]
unsafe fn ctx<'a>(ctx: *mut c_void) -> &'a mut Ctx {
    // SAFETY: the caller's obligation, stated above. The reference does not
    // outlive the thunk, and generated code holds no Rust reference of its
    // own, so this is the only live borrow of the context while it exists.
    unsafe { &mut *ctx.cast::<Ctx>() }
}

/// Reconstitute the host a context names.
///
/// # Safety
///
/// `c.host` must be a pointer to a live `H` — [`Engine::run`] takes it from
/// the `&mut H` it was called with and holds that borrow across the call — and
/// `H` must be the type the [`Vtable`] was built for, which it is because the
/// two are set from the same monomorphization.
#[inline]
unsafe fn host_of<'a, H>(c: &mut Ctx) -> &'a mut H {
    // SAFETY: the caller's obligation, stated above.
    unsafe { &mut *c.host.cast::<H>() }
}

/// The temporary frame a context names.
///
/// # Safety
///
/// `c.temps` must point at `len` initialized `u64`s, which [`Engine::run`]
/// establishes from a `Vec` grown to at least the block's temporary count.
/// *Initialized*, not meaningful: a slot the executed path never assigned
/// holds whatever an earlier block left there.
#[inline]
unsafe fn temps_of<'a>(c: &Ctx, len: usize) -> &'a [u64] {
    // SAFETY: the caller's obligation, stated above.
    unsafe { core::slice::from_raw_parts(c.temps, len) }
}

/// Materialize the pending boundary's live mapping into guest state.
///
/// `ir::interp`'s `publish`, in the shape a thunk can call: idempotent, a no-op
/// when nothing is pending, and reading the temporaries out of the frame rather
/// than out of an interpreter.
fn publish<H: IrHost + ?Sized>(c: &mut Ctx, block: &Block, temps: &[u64], host: &mut H) {
    if c.published != 0 {
        return;
    }
    c.published = 1;
    let Ok(index) = usize::try_from(c.mark) else {
        return;
    };
    let Some(mark) = block.marks().get(index) else {
        return;
    };
    for &(slot, temp) in &mark.live {
        if let Some(value) = temps.get(temp.index()).copied() {
            host.write_slot(slot, u128::from(value));
        }
    }
}

/// Whether the pending boundary binds `slot` to a temporary.
fn shadowed(c: &Ctx, block: &Block, slot: RegSlot) -> bool {
    if c.published != 0 {
        return false;
    }
    usize::try_from(c.mark)
        .ok()
        .and_then(|i| block.marks().get(i))
        .is_some_and(|mark| mark.live.iter().any(|&(s, _)| s == slot))
}

/// Replay instructions `lo .. hi` of the block: its charges and its
/// boundaries, to the context and to the host.
///
/// The one thunk on the hot path, and the reason the hot path has so little
/// left on it. This is `ir::Interp`'s [`Opcode::CHARGE`] and
/// [`Opcode::INSN_START`] arms **moved**, not batched: the same calls, with
/// the same arguments, in the same order. Generated code contributes only the
/// range, which it knows statically — see
/// [`compile`](super::compile)'s "Deferred bookkeeping" for why a static range
/// is exactly the set of instructions that ran.
///
/// A host cannot tell the difference, because nothing it can observe happens
/// between an instruction and its replay: between two flush points generated
/// code touches only the temporary frame and this context, and every other
/// thunk — a load, a store, a slot read, an inlined access's tick — has a
/// flush emitted ahead of it.
unsafe extern "sysv64" fn flush_thunk<H: IrHost + FastMem>(
    raw: *mut c_void,
    lo: u64,
    hi: u64,
) -> u64 {
    // SAFETY: `raw` is the context `Engine::run` entered generated code with,
    // `c.host` is the `&mut H` it was called with, `c.block` is the `&Block`
    // it holds for the whole call, and `c.events` points at the `Box<[Event]>`
    // the `Compiled` owns for at least as long as its code. All four are live
    // for the whole call and name four distinct objects, so the `&mut Ctx`,
    // the `&mut H`, the `&Block` and the `&[Event]` here do not alias. See
    // `ctx` and `host_of`. The range is clamped rather than trusted.
    unsafe {
        let c = ctx(raw);
        let block = &*c.block;
        let all = core::slice::from_raw_parts(c.events, c.event_count as usize);
        let hi = (hi as usize).min(all.len());
        // Clamped to `hi`, not to the table. `compile` never emits a reversed
        // range — `plan` only ever hands out `(region, here)` with
        // `region < here` — so the two clamps are the same function on every
        // input generated code can produce, and a mutation between them
        // survives every test. Recorded rather than tuned away, because they
        // are *not* the same function on a range that never happens: clamping
        // to the table would leave `lo > hi` and index a backwards slice.
        // This is the arm that answers a corrupted immediate with nothing
        // rather than with a panic in generated-code territory.
        let lo = (lo as usize).min(hi);
        for event in &all[lo..hi] {
            match *event {
                Event::Charge(ticks) => {
                    // The context first, then the host — and the order is
                    // *unobservable*, which is worth writing down because a
                    // mutation that swaps it survives every test in the tree.
                    // `IrHost::charge` is handed a `u64` and nothing else; the
                    // context is a local of `Engine::run` whose only pointer
                    // lives in generated code's argument register and in this
                    // thunk's own parameter, so an `H` cannot reach it to read
                    // or to write. Two writes to disjoint objects with no
                    // intervening read commute. The order kept is `Interp`'s.
                    c.ticks = c.ticks.wrapping_add(ticks);
                    c.committed = 1;
                    host_of::<H>(c).charge(ticks);
                }
                Event::Boundary { mark: index, exit } => {
                    // `compile` refuses a marker pointing at no record, so the
                    // skip is unreachable rather than a boundary lost.
                    let Some(mark) = block.marks().get(index as usize) else {
                        continue;
                    };
                    let mark: &InsnStart = mark;
                    c.mark = i64::from(index);
                    c.published = 0;
                    c.boundaries = c.boundaries.wrapping_add(1);
                    c.boundary_pc = mark.pc;
                    // The charged count rather than `mark.ticks`, exactly as
                    // `Interp` does it: the static column undercounts once an
                    // access in this block has spent a data-dependent tick.
                    c.retired = c.ticks;
                    // Restart granularity is the guest instruction, so the
                    // previous one's commits stop blocking a retry here.
                    c.committed = 0;
                    host_of::<H>(c).insn_start(mark);
                    // The tick allowance, asked exactly where `Interp` asks
                    // it and under exactly the same two guards: never at the
                    // block's first boundary, and never at an exit boundary,
                    // whose `pc` is a placeholder when the successor is
                    // computed. Replaying stops here, so the
                    // charges of the events after it are never made -- which
                    // is right, because the instructions they belong to are
                    // being *unwound*. Everything between this boundary and
                    // the flush point is a region, and a region contains no
                    // call site by construction (`compile`'s "Deferred
                    // bookkeeping"), so all it did was write temporaries the
                    // block is about to abandon. The pending boundary's own
                    // live temporaries are untouched by them -- a temporary
                    // is defined once -- which is the same fact the fault
                    // path already rests on.
                    if c.boundaries > 1 && !exit && host_of::<H>(c).spent() {
                        return 1;
                    }
                }
            }
        }
        0
    }
}

unsafe extern "sysv64" fn get_slot_thunk<H: IrHost + FastMem>(raw: *mut c_void, slot: u64) -> u64 {
    // SAFETY: as `insn_start_thunk`. `temps_of` is given the block's own
    // temporary count, which is the length `Engine::run` sized the frame to.
    unsafe {
        let c = ctx(raw);
        let block = &*c.block;
        let slot = RegSlot(slot as u16);
        if shadowed(c, block, slot) {
            // Guest state is published lazily, so a slot the current boundary
            // binds is stale in the host until it is written out. Reading it
            // without this would hand back the value from before the temporary
            // took over — `ir::interp`, "Materializing guest state".
            let temps = temps_of(c, block.temp_count());
            let host = host_of::<H>(c);
            publish(c, block, temps, host);
        }
        host_of::<H>(c).read_slot(slot) as u64
    }
}

unsafe extern "sysv64" fn load_thunk<H: IrHost + FastMem>(
    raw: *mut c_void,
    mem: *const MemOp,
    addr: u64,
    out: *mut u64,
) -> u64 {
    // SAFETY: as `charge_thunk`. `mem` points into the compiled block's own
    // descriptor table, which `Compiled` owns in a `Box<[MemOp]>` that outlives
    // the run; `out` is the eight bytes of stack the prologue reserved.
    unsafe {
        let c = ctx(raw);
        let mem = &*mem;
        match host_of::<H>(c).load(mem, addr) {
            Ok(value) => {
                *out = value;
                0
            }
            Err(e) => error_code(e),
        }
    }
}

unsafe extern "sysv64" fn store_thunk<H: IrHost + FastMem>(
    raw: *mut c_void,
    mem: *const MemOp,
    addr: u64,
    value: u64,
) -> u64 {
    // SAFETY: as `load_thunk`.
    unsafe {
        let c = ctx(raw);
        let mem = &*mem;
        match host_of::<H>(c).store(mem, addr, value) {
            Ok(()) => 0,
            Err(e) => error_code(e),
        }
    }
}

unsafe extern "sysv64" fn fast_tick_thunk<H: IrHost + FastMem>(raw: *mut c_void) {
    // SAFETY: as `charge_thunk`.
    unsafe {
        let c = ctx(raw);
        host_of::<H>(c).note_fast_load();
    }
}

unsafe extern "sysv64" fn fast_store_thunk<H: IrHost + FastMem>(
    raw: *mut c_void,
    addr: u64,
    bytes: u64,
) {
    // SAFETY: as `charge_thunk`.
    unsafe {
        let c = ctx(raw);
        host_of::<H>(c).note_fast_store(addr, bytes);
    }
}

/// The direct link, from the inside: close the block that just exited, and
/// open the one a [`Chain`] hands back.
///
/// The whole of what generated code does at a block boundary once linking is
/// on. Everything here is what [`Engine::run`] and `Dispatcher::run` used to
/// do around a block, in the same order — the publish that materializes the
/// exit boundary, the drain that matches guest stores against the cache, the
/// budget, the safe point, the tick allowance, the epoch, the entry work and
/// the lookup — minus the two frames, the context and the thunk table, which
/// is the point of the exercise.
///
/// The TLB parameters are re-taken, and that is not tidiness: `Frontend::enter`
/// can walk a guest page table, a walk can miss and fill, and a fill can
/// replace the table the inlined probe reads. The old block's copy was taken
/// before any of that.
unsafe extern "sysv64" fn chain_thunk<F, H>(raw: *mut c_void) -> u64
where
    F: Frontend<H> + ?Sized,
    H: IrHost + StoreLog + FastMem,
{
    // SAFETY: `raw` is the context `Engine::run_chained` entered generated
    // code with; `c.host` is the `&mut H` it was called with; `c.block` is the
    // block this context was last opened on, which `Chain::step` keeps
    // resident (it never inserts, and it re-points this field before anything
    // can invalidate what it named); and `c.chain_ctl` is the `&mut Chain` the
    // same call was given. All four are live for the whole call and name four
    // distinct objects, so the references taken here do not alias.
    unsafe {
        let c = ctx(raw);
        let block = &*c.block;
        let temps = temps_of(c, block.temp_count());
        let host = host_of::<H>(c);
        // Before anything can observe guest state — the drain below reaches a
        // host, and a fault path reads the boundary's slots.
        publish(c, block, temps, host);
        // Every exit is preceded by one boundary that begins no guest
        // instruction, and exactly one exit is reached, so this is what
        // retired — at a fault too.
        let retired = c.boundaries.saturating_sub(1) as usize;
        let chain = &mut *c.chain_ctl.cast::<Chain<'_, F, H>>();
        let next = match c.out_status {
            status::GOTO | status::LOOKUP => Some(c.out_pc),
            status::EXIT => Some(host_of::<H>(c).read_slot(chain.pc_slot()) as u64),
            // A fault, or the tick allowance. There is no successor and the
            // block's own status is what the run reports.
            _ => None,
        };
        let Step::Go { id, code, entry } = chain.step(next, retired, host_of::<H>(c)) else {
            return 0;
        };
        let Some(block) = chain.block(id) else {
            // `Chain::step` only hands back a resident block, so this is
            // unreachable rather than a case — and leaving is the answer that
            // cannot be wrong.
            return 0;
        };
        c.block = core::ptr::from_ref(block);
        c.events = code.events as *const Event;
        c.event_count = u64::from(code.event_count);
        // Everything `Engine::run` sets afresh for a block, set afresh. The
        // two counters it does *not* reset are `fast_hits` and `fast_writes`,
        // which are the engine's own statistics and are folded in once when
        // the chain ends.
        let host = host_of::<H>(c);
        let plan = host.load_plan();
        let stores = host.store_plan();
        c.tlb_base = plan.map_or(core::ptr::null(), |p| p.set.base);
        c.tlb_mask = plan.map_or(0, |p| p.set.mask);
        c.tag_bits = plan.map_or(0, |p| p.tag);
        c.st_base = stores.map_or(core::ptr::null(), |p| p.set.base);
        c.st_mask = stores.map_or(0, |p| p.set.mask);
        c.st_tag = stores.map_or(0, |p| p.tag);
        c.ticks = 0;
        c.retired = 0;
        c.boundaries = 0;
        c.boundary_pc = block.entry_pc;
        c.mark = -1;
        c.committed = 0;
        c.blocks_run = c.blocks_run.wrapping_add(1);
        // Four of `Engine::run`'s twenty-eight fields are deliberately not
        // reset. `out_pc`, `fault_at` and `fault_error` are written by the
        // path that reads them and by no other — a terminator and the fault
        // sequence — so a leftover is never read; and `published` is already
        // one, because the publish above set it.
        entry
    }
}

impl Vtable {
    /// The thunks for one host type.
    #[must_use]
    pub fn of<H: IrHost + FastMem>() -> Vtable {
        Vtable {
            flush: flush_thunk::<H>,
            get_slot: get_slot_thunk::<H>,
            load: load_thunk::<H>,
            store: store_thunk::<H>,
            fast_tick: fast_tick_thunk::<H>,
            fast_store: fast_store_thunk::<H>,
        }
    }
}

/// What an engine has been asked to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EngineStats {
    /// Blocks compiled to host code.
    pub compiled: u64,
    /// Blocks the compiler refused, which run on the interpreter instead.
    pub refused: u64,
    /// Blocks executed as host code.
    pub executed: u64,
    /// Times the code buffer filled and was thrown away.
    pub resets: u64,
    /// Guest loads served by an inlined TLB probe, with no call at all.
    pub fast_loads: u64,
    /// Guest stores served by an inlined TLB probe, with only the thunk that
    /// reports the write.
    pub fast_stores: u64,
}

/// The x86-64 backend: a code buffer, the blocks in it, and a way in.
///
/// Mirrors [`Interp`](crate::ir::Interp)'s surface where the two overlap —
/// [`Engine::ticks`], [`Engine::boundaries`], [`Engine::mark`] — because a
/// dispatcher reads exactly those off whichever engine ran the block, and a
/// backend that reported them differently would make a run's retired
/// instruction count depend on which engine executed it.
#[derive(Debug)]
pub struct Engine {
    buf: CodeBuf,
    arena: Vec<Compiled>,
    /// The temporary frame, shared by every block this engine runs.
    ///
    /// Grown to fit and never cleared — see [`Engine::run`]. Its length is the
    /// largest temporary count this engine has been handed, so a small block
    /// runs against a frame with a large block's leftovers past its own end
    /// *and inside it*.
    temps: Vec<u64>,
    /// Which block's code last ran, so [`Engine::temp_value`] knows which
    /// temporaries that code wrote into the frame.
    last: Option<CodeRef>,
    regs: Regs,
    ticks: u64,
    boundaries: u64,
    mark: Option<u32>,
    stats: EngineStats,
}

impl Engine {
    /// An engine over a [`DEFAULT_CAPACITY`]-byte buffer, or `None` where the
    /// kernel would not give one.
    #[must_use]
    pub fn new() -> Option<Engine> {
        Engine::with_capacity(DEFAULT_CAPACITY)
    }

    /// An engine over a buffer of `bytes`.
    #[must_use]
    pub fn with_capacity(bytes: u64) -> Option<Engine> {
        Some(Engine {
            buf: CodeBuf::new(bytes)?,
            arena: Vec::new(),
            temps: Vec::new(),
            last: None,
            regs: Regs::default(),
            ticks: 0,
            boundaries: 0,
            mark: None,
            stats: EngineStats::default(),
        })
    }

    /// Where blocks compiled from now on keep their temporaries.
    ///
    /// [`Regs::Frame`] is the backend without the register allocator, kept
    /// runnable as the control: it is what the differential in
    /// `jit::x86::tests` compares against and what the two benchmarks' columns
    /// separate. Blocks already compiled keep the policy they were compiled
    /// under.
    pub fn set_regs(&mut self, regs: Regs) {
        self.regs = regs;
    }

    /// Which policy new blocks are compiled under.
    #[inline]
    #[must_use]
    pub fn regs(&self) -> Regs {
        self.regs
    }

    /// Fill the temporary frame with `value`, for a differential that needs to
    /// know what a run does *not* depend on.
    ///
    /// The frame is not cleared between blocks ([`Engine::run`] says why), so
    /// "the executed path assigns every temporary anything reads" stopped
    /// being enforced by construction and became a property with nothing
    /// asserting it. Running one block twice under two different fills and
    /// requiring identical guest-visible output *is* that assertion, and it is
    /// the only reason this exists — which is why it is not compiled into a
    /// shipping build.
    #[cfg(test)]
    pub(super) fn seed_frame(&mut self, value: u64, temps: usize) {
        self.temps.clear();
        self.temps.resize(temps, value);
    }

    /// What this engine has been asked to do.
    #[inline]
    #[must_use]
    pub fn stats(&self) -> EngineStats {
        self.stats
    }

    /// Ticks charged during the last run.
    #[inline]
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.ticks
    }

    /// Boundaries passed during the last run.
    #[inline]
    #[must_use]
    pub fn boundaries(&self) -> u64 {
        self.boundaries
    }

    /// The boundary the last run reached.
    #[inline]
    #[must_use]
    pub fn mark(&self) -> Option<u32> {
        self.mark
    }

    /// The value a temporary held when the last run stopped, if the run kept
    /// it.
    ///
    /// The counterpart of
    /// [`Interp::temp_value`](crate::ir::Interp::temp_value), and what a
    /// backend-level differential compares — but not on every temporary, and
    /// the difference is the register allocator rather than an omission. A
    /// temporary the allocator kept in a host register is **gone** once the
    /// block has returned and the epilogue has restored the caller's
    /// registers; its frame slot holds the zero the frame was cleared to, and
    /// handing that back as a value would be a lie a differential would then
    /// assert. So this returns `None` for one.
    ///
    /// What it never returns `None` for is a temporary an
    /// [`InsnStart`] names, because that is the state the exception path
    /// materializes and the backend writes those through to the frame at their
    /// definition. That is the property, and `jit::x86::tests`' `agree_under`
    /// asserts it on every block the differential generates, faulting or not.
    ///
    /// `Some` is **not** a claim that the last run assigned the temporary.
    /// The frame is not cleared between blocks, so a temporary whose
    /// definition the executed path jumped over — the inline exit sequence a
    /// `brcond` branches around — reads back as whatever an earlier block left
    /// in that slot. The IR says nothing about such a temporary's value either,
    /// which is why nothing that is not a debugging aid may read one:
    /// `agree_under` proves the frame's prior contents reach no guest-visible
    /// output by running each block twice against two different fills.
    #[inline]
    #[must_use]
    pub fn temp_value(&self, temp: crate::ir::Temp) -> Option<u64> {
        // Through `is_live`, because a code-buffer reset clears the arena and
        // refills it: an index from before one names a different block's
        // allocation, and answering out of that would be a wrong answer rather
        // than a missing one.
        let last = self.last?;
        if !self.is_live(last) {
            return None;
        }
        if !self.arena.get(last.index as usize)?.frame_backed(temp) {
            return None;
        }
        self.temps.get(temp.index()).copied()
    }

    /// Compile `block`, or say why not.
    ///
    /// A refusal is not an error: the interpreter is always available, and a
    /// backend that covers the common shapes and declines the rest is the
    /// design (`ROADMAP.md` §9, "Backends"). The one recoverable failure is a
    /// full buffer, which resets it and tries once more — a reset invalidates
    /// every outstanding [`CodeRef`], which is safe because a stale one is
    /// rejected on use.
    ///
    /// # Errors
    ///
    /// [`Refusal`], naming the op or the shape that stopped it.
    pub fn compile(&mut self, block: &Block) -> core::result::Result<CodeRef, Refusal> {
        let compiled = match compile_with(block, self.regs) {
            Ok(c) => c,
            Err(e) => {
                self.stats.refused += 1;
                return Err(e);
            }
        };
        let offset = match self.buf.push(compiled.code()) {
            Some(at) => at,
            None => {
                self.buf.reset();
                self.arena.clear();
                self.stats.resets += 1;
                self.buf
                    .push(compiled.code())
                    .ok_or(Refusal::CodeBufferFull)?
            }
        };
        let index = u32::try_from(self.arena.len()).map_err(|_| Refusal::CodeBufferFull)?;
        let placed = compiled.at(offset);
        let chain = placed.chain_entry();
        self.arena.push(placed);
        // The `Box<[Event]>`'s allocation, which does not move when the arena
        // grows and is freed only by the reset that bumps the generation.
        let events = self.arena[index as usize].events();
        let (at, count) = (events.as_ptr() as u64, events.len() as u32);
        self.stats.compiled += 1;
        Ok(CodeRef {
            index,
            generation: self.buf.generation(),
            chain,
            events: at,
            event_count: count,
        })
    }

    /// What a dispatcher needs to resolve a link while this engine is running.
    ///
    /// Taken once, before entering generated code. See [`Linkage`].
    #[inline]
    #[must_use]
    pub fn linkage(&self) -> Linkage {
        Linkage {
            base: self.buf.base(),
            generation: self.buf.generation(),
            temps: self.temps.len(),
        }
    }

    /// Make the shared temporary frame at least `temps` long.
    ///
    /// Called before entering a chain, because it cannot be called during one:
    /// growing the frame can move it, and every block standing on it holds its
    /// address in a register. A block needing more than this leaves the chain
    /// instead — see [`Chain`].
    pub fn reserve_temps(&mut self, temps: usize) {
        if self.temps.len() < temps {
            self.temps.resize(temps, 0);
        }
    }

    /// Whether `code` still names live host code.
    #[inline]
    #[must_use]
    pub fn is_live(&self, code: CodeRef) -> bool {
        code.generation == self.buf.generation() && (code.index as usize) < self.arena.len()
    }

    /// Execute `block` as host code, and say why it stopped.
    ///
    /// `None` when `code` is stale — the buffer was reset under it — which the
    /// caller answers by compiling the block again.
    ///
    /// # Errors
    ///
    /// [`Error::Bus`] carrying [`BusError::Retry`] when the host asks to retry
    /// an access that can no longer be retried, exactly as
    /// [`Interp::run`](crate::ir::Interp::run) does.
    pub fn run<H: IrHost + FastMem>(
        &mut self,
        block: &Block,
        code: CodeRef,
        host: &mut H,
    ) -> Option<Result<Outcome>> {
        // SAFETY: `block` is a live reference held for the whole call, and no
        // chain hook is given, so nothing re-points the context at anything
        // else.
        unsafe { self.enter_at(core::ptr::from_ref(block), code, host, None) }
    }

    /// Execute the block `id` names, and let it **jump straight on** to its
    /// successors.
    ///
    /// The same run [`Engine::run`] does, with one thing added: at each block's
    /// exit the generated code calls back into `chain`, and where that answers
    /// with a successor the code *jumps* to it — no epilogue, no prologue, no
    /// context rebuilt, no thunk table rebuilt. Those, and not the hash lookup
    /// [`BlockCache::follow`](crate::jit::BlockCache::follow) already skipped,
    /// are what a block boundary costs.
    ///
    /// What comes back describes the **last** block of the chain: its
    /// [`Outcome`], its [`Engine::boundaries`], its [`Engine::mark`].
    /// Every earlier block was closed off by [`Chain::step`], which is where a
    /// caller's per-block bookkeeping has to live once the loop is inside
    /// generated code — the exit boundary's publish included, so this does not
    /// do one.
    ///
    /// `None` when `code` is stale, exactly as [`Engine::run`].
    ///
    /// # Errors
    ///
    /// As [`Engine::run`].
    ///
    /// # Panics
    ///
    /// If `id` names no resident block. The caller has just found or inserted
    /// it, so this is the same claim [`Dispatcher`](crate::jit::Dispatcher)
    /// makes everywhere else it reaches into the cache.
    pub fn run_chained<F, H>(
        &mut self,
        id: BlockId,
        code: CodeRef,
        host: &mut H,
        chain: &mut Chain<'_, F, H>,
    ) -> Option<Result<Outcome>>
    where
        F: Frontend<H> + ?Sized,
        H: IrHost + StoreLog + FastMem,
    {
        let block = chain.block(id).expect("the entry block is resident");
        let block: *const Block = core::ptr::from_ref(block);
        let hook = (
            chain_thunk::<F, H> as ChainFn,
            core::ptr::from_mut(chain).cast::<c_void>(),
        );
        // SAFETY: `block` names a `Block` the chain's own cache holds, and the
        // chain outlives this call. What keeps it valid for the whole run is
        // `Chain::step`'s contract: it never inserts, so the cache's slot
        // vector never reallocates, and it re-points the context at its
        // successor before anything can invalidate the block it replaced.
        unsafe { self.enter_at(block, code, host, Some(hook)) }
    }

    /// Enter `block`'s compiled code, with or without a link hook.
    ///
    /// # Safety
    ///
    /// `block` must point at a live [`Block`] for the whole call. Without a
    /// hook that is the caller's own reference; with one it is also
    /// [`Chain::step`]'s obligation, which re-points the context before
    /// anything can invalidate what it named.
    unsafe fn enter_at<H: IrHost + FastMem>(
        &mut self,
        block: *const Block,
        code: CodeRef,
        host: &mut H,
        chain: Option<(ChainFn, *mut c_void)>,
    ) -> Option<Result<Outcome>> {
        if !self.is_live(code) {
            return None;
        }
        // SAFETY: the caller's obligation, stated above. Nothing has run yet,
        // so this is the reference the caller handed over. It is read out
        // here and not held: a chain re-points the context at its successors,
        // and a `&Block` left standing over that would be a reference to a
        // block the cache may since have dropped.
        let (temp_count, entry_pc) = unsafe { ((*block).temp_count(), (*block).entry_pc) };
        let compiled = &self.arena[code.index as usize];
        let offset = compiled.offset();
        // The deferred bookkeeping, taken as a raw slice: the `Box`'s
        // allocation does not move while `self` is borrowed across the call.
        let events = compiled.events();
        let (events, event_count) = (events.as_ptr(), events.len() as u64);
        self.last = Some(code);
        // A high-water mark, not a fresh frame. `clear()` + `resize(n, 0)`
        // zeroed one `u64` per temporary the block *declares* on every
        // execution, so entering a block carried a term proportional to how
        // much of it there was and none at all to how much of it ran — exactly
        // the wrong shape now that a block may leave at any guest instruction
        // boundary on the tick allowance. `docs/platforms/pc64.md` has what it
        // cost and what removing it moved.
        //
        // Nothing needs the zeroes. `compile` writes every frame-homed
        // temporary at its definition and refuses a block that reads one
        // before its definition, so every frame slot generated code reads was
        // written by this execution. What is left over from an earlier block
        // is reachable only through `Engine::temp_value`, whose contract says
        // so, and only for a temporary whose definition a `brcond` jumped
        // over.
        if self.temps.len() < temp_count {
            self.temps.resize(temp_count, 0);
        }

        // The inlined fast path's parameters, taken once per block. The
        // pointer is valid until the TLB is flushed, and a flush happens at a
        // block boundary (`Tlb::sync`) — never inside one.
        let plan = host.load_plan();
        let stores = host.store_plan();
        let vt = Vtable::of::<H>();
        let mut ctx = Ctx {
            temps: self.temps.as_mut_ptr(),
            vt: &raw const vt,
            host: core::ptr::from_mut(host).cast::<c_void>(),
            block,
            tlb_base: plan.map_or(core::ptr::null(), |p| p.set.base),
            tlb_mask: plan.map_or(0, |p| p.set.mask),
            tag_bits: plan.map_or(0, |p| p.tag),
            out_pc: 0,
            ticks: 0,
            retired: 0,
            boundaries: 0,
            boundary_pc: entry_pc,
            mark: -1,
            fault_at: 0,
            fault_error: 0,
            committed: 0,
            published: 1,
            fast_hits: 0,
            st_base: stores.map_or(core::ptr::null(), |p| p.set.base),
            st_mask: stores.map_or(0, |p| p.set.mask),
            st_tag: stores.map_or(0, |p| p.tag),
            fast_writes: 0,
            events,
            event_count,
            chain: chain.map_or(0, |(f, _)| f as usize as u64),
            chain_ctl: chain.map_or(core::ptr::null_mut(), |(_, c)| c),
            out_status: 0,
            blocks_run: 0,
        };

        // SAFETY: `offset` names the first byte of a function this buffer
        // holds, in the current generation (`is_live` above), emitted by
        // `compile` — which produces exactly one shape of function: the System
        // V `extern "sysv64" fn(*mut c_void) -> u64` `Entry` names. It reads
        // and writes the `Ctx` behind its argument, the temporary frame that
        // context points at, the `MemOp` table `compiled` owns, and guest RAM
        // through host addresses taken from live TLB entries — all of which
        // are alive for the whole call, because `self` and `host` are borrowed
        // mutably across it and `ctx` is a local.
        //
        // Guest RAM is both read and written that way. An entry carries a host
        // address only for a whole page of a `RamStore` that outlives this
        // call (the plan is taken above, and the `Arc` lives in the host's own
        // TLB), and generated code touches exactly the bytes of the access —
        // `store_trunc` emits the width the guest asked for. Those bytes are
        // `AtomicU8`, written elsewhere with relaxed stores, and the emitted
        // `mov` is the instruction a relaxed atomic byte store compiles to; no
        // Rust reference to them is ever formed, which is the obligation
        // `RamStore::host_ptr` states. What that documentation additionally
        // forbids — writing without marking the store dirty — is paid by
        // `FastMem::note_fast_store`, called on the same path before anything
        // can observe the write.
        let entry = unsafe { self.buf.entry(offset) }?;
        // SAFETY: as above. `ctx` is a live, initialized `Ctx` and the pointer
        // does not escape the call.
        let stop = unsafe { entry(core::ptr::from_mut(&mut ctx).cast::<c_void>()) };

        if ctx.blocks_run != 0 {
            // The frame holds the *last* block of the chain's temporaries, and
            // this engine no longer knows which block that was — the thunk
            // followed the links, not this frame. `temp_value` answers `None`
            // rather than reading one block's allocation against another's
            // frame, which is the only wrong answer it could give.
            self.last = None;
        }
        self.stats.executed += 1 + ctx.blocks_run;
        self.stats.fast_loads += ctx.fast_hits;
        self.stats.fast_stores += ctx.fast_writes;
        self.ticks = ctx.ticks;
        self.boundaries = ctx.boundaries;
        self.mark = u32::try_from(ctx.mark).ok();

        // Whatever happened — an exit, a fault, a stale branch — the guest's
        // architectural state is materialized before the caller can look at
        // it, in one place, exactly as `Interp::run` does it.
        //
        // Except on the chained path, where the thunk has already done it for
        // every block including this one. That is not an optimisation: a chain
        // whose last block wrote into its own page has had that block
        // invalidated by the thunk's drain, and `ctx.block` then names a
        // `Block` the cache has dropped. The publish has to happen before the
        // drain, which is where the thunk does it, and there is nothing left
        // to do here.
        if ctx.chain == 0 {
            // SAFETY: with no hook nothing re-pointed the context, so this is
            // still the caller's own live reference.
            publish(&mut ctx, unsafe { &*block }, &self.temps, host);
        }

        Some(match stop {
            status::EXIT => Ok(Outcome::Exit),
            status::GOTO => Ok(Outcome::Goto { pc: ctx.out_pc }),
            status::LOOKUP => Ok(Outcome::Lookup { pc: ctx.out_pc }),
            // The boundary the flush stopped at, which is the one the context
            // still names: the guest instruction that has not started.
            status::SPENT => Ok(Outcome::Spent {
                pc: ctx.boundary_pc,
            }),
            _ => {
                let error = error_of(ctx.fault_error);
                if error == BusError::Retry && ctx.committed != 0 {
                    // The guest instruction has already changed something the
                    // world can see, so there is nothing left to restart from.
                    Err(Error::Bus(BusError::Retry))
                } else {
                    Ok(Outcome::Fault(Fault {
                        error,
                        at: ctx.fault_at as usize,
                        mark: self.mark,
                        pc: ctx.boundary_pc,
                        retired_ticks: ctx.retired,
                        charged_ticks: ctx.ticks,
                        restartable: ctx.committed == 0,
                    }))
                }
            }
        })
    }
}

/// The ops this backend compiles, for a caller that wants to know before it
/// tries.
///
/// The union of what the RISC-V, x86 and A64 frontends emit, plus the handful
/// of neighbours that cost nothing extra once their family is in. Everything
/// else is a [`Refusal`] and runs on the interpreter.
///
/// One list, not two: this is `compile::compiles` re-exported, so a caller
/// asking the runtime and the code generator the same question cannot get two
/// answers.
#[must_use]
pub fn compiles(op: Opcode) -> bool {
    super::compile::compiles(op)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_context_layout_a_code_generator_bakes_in_is_the_one_rust_built() {
        // Generated code reaches every one of these with an immediate
        // displacement it cannot re-derive. Adding a field or reordering two
        // would silently point a `mov` at the wrong one, so the agreement is
        // asserted rather than commented.
        assert_eq!(core::mem::offset_of!(Ctx, temps) as i32, off::TEMPS);
        assert_eq!(core::mem::offset_of!(Ctx, vt) as i32, off::VT);
        assert_eq!(core::mem::offset_of!(Ctx, tlb_base) as i32, off::TLB_BASE);
        assert_eq!(core::mem::offset_of!(Ctx, tlb_mask) as i32, off::TLB_MASK);
        assert_eq!(core::mem::offset_of!(Ctx, tag_bits) as i32, off::TAG_BITS);
        assert_eq!(core::mem::offset_of!(Ctx, out_pc) as i32, off::OUT_PC);
        assert_eq!(core::mem::offset_of!(Ctx, fault_at) as i32, off::FAULT_AT);
        assert_eq!(
            core::mem::offset_of!(Ctx, fault_error) as i32,
            off::FAULT_ERROR
        );
        assert_eq!(core::mem::offset_of!(Ctx, committed) as i32, off::COMMITTED);
        assert_eq!(core::mem::offset_of!(Ctx, fast_hits) as i32, off::FAST_HITS);
        assert_eq!(core::mem::offset_of!(Ctx, st_base) as i32, off::ST_BASE);
        assert_eq!(core::mem::offset_of!(Ctx, st_mask) as i32, off::ST_MASK);
        assert_eq!(core::mem::offset_of!(Ctx, st_tag) as i32, off::ST_TAG);
        assert_eq!(
            core::mem::offset_of!(Ctx, fast_writes) as i32,
            off::FAST_WRITES
        );
        // The deferred bookkeeping sits past every declared offset it needs to
        // be past, which is what lets it be added without moving anything
        // generated code names.
        assert!(core::mem::offset_of!(Ctx, events) > off::FAST_WRITES as usize);
        // The direct link's two, which generated code reaches with an
        // immediate displacement exactly as it reaches the rest.
        assert_eq!(core::mem::offset_of!(Ctx, chain) as i32, off::CHAIN);
        assert_eq!(
            core::mem::offset_of!(Ctx, out_status) as i32,
            off::OUT_STATUS
        );

        // The six the bookkeeping took back are still where a displacement
        // *would* have reached them. They are asserted anyway, and not out of
        // symmetry: `committed` and `fast_hits` sit past all six, so a field
        // that moved would move those two with it, and the arithmetic that
        // says it does not belongs somewhere a reader can check it.
        assert_eq!(core::mem::offset_of!(Ctx, ticks), 64);
        assert_eq!(core::mem::offset_of!(Ctx, retired), 72);
        assert_eq!(core::mem::offset_of!(Ctx, boundaries), 80);
        assert_eq!(core::mem::offset_of!(Ctx, boundary_pc), 88);
        assert_eq!(core::mem::offset_of!(Ctx, mark), 96);
        assert_eq!(core::mem::offset_of!(Ctx, published), 128);
    }

    #[test]
    fn the_thunk_table_layout_is_the_one_generated_code_indexes() {
        assert_eq!(core::mem::offset_of!(Vtable, flush) as i32, vt::FLUSH);
        assert_eq!(core::mem::offset_of!(Vtable, get_slot) as i32, vt::GET_SLOT);
        assert_eq!(core::mem::offset_of!(Vtable, load) as i32, vt::LOAD);
        assert_eq!(core::mem::offset_of!(Vtable, store) as i32, vt::STORE);
        assert_eq!(
            core::mem::offset_of!(Vtable, fast_tick) as i32,
            vt::FAST_TICK
        );
        assert_eq!(
            core::mem::offset_of!(Vtable, fast_store) as i32,
            vt::FAST_STORE
        );
    }

    #[test]
    fn every_bus_error_survives_the_round_trip() {
        for e in [
            BusError::Unassigned,
            BusError::BadAccess,
            BusError::Protected,
            BusError::Retry,
        ] {
            assert_eq!(error_of(error_code(e)), e);
            assert_ne!(error_code(e), 0, "zero is reserved for success");
        }
    }
}
