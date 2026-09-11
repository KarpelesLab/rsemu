//! The boundary between generated code and Rust: the execution context, the
//! thunk table, and the bookkeeping a flush replays.
//!
//! # Why this file opts into `unsafe`
//!
//! It is the **same** sanctioned subsystem as `buf` — the JIT
//! code buffer (`ROADMAP.md` §0, CLAUDE.md's "`unsafe`") — seen from the side
//! that crosses back into Rust. A call from machine code into a Rust `IrHost`
//! cannot be expressed without reconstituting `&mut` references from the
//! pointers generated code was handed. This is not an eighth site: it is the
//! second backend of the second sanctioned one, and it adds no new kind of
//! obligation. Every block below states the same invariants
//! `jit::x86::rt`'s do, because they are the same invariants.
//!
//! # Why it is not `cfg`-gated to an aarch64 host
//!
//! Nothing here executes anything. A [`Ctx`] is a `#[repr(C)]` struct, a
//! [`Vtable`] is six `extern "C"` function pointers, and the thunks are
//! ordinary Rust that reads them — all of which compile, and whose *layout*
//! can be asserted, on any host. That matters more here than it would for the
//! x86 backend: the field offsets below are baked into generated code as
//! immediates, and this project's CI runs most of its jobs on x86-64, so a
//! layout assertion that only ran on the aarch64 runner would be a layout
//! assertion that mostly does not run. `buf` and
//! `rt` are the two files that genuinely need the host, and they
//! are gated; this one is not.
//!
//! # The contract generated code is compiled against
//!
//! One argument, in `x0`: a pointer to a [`Ctx`]. One result, in `x0`: a
//! [`status`] code. `x19` holds the context for the body of the block, `x20`
//! the temporary frame and `x21` this table; `x19`–`x28` and `x29`/`x30` are
//! saved and restored by the prologue and epilogue. That is the *Procedure
//! Call Standard for the Arm 64-bit Architecture* (**AAPCS64**, Arm IHI 0055),
//! §6.1.1: `x0`–`x7` pass arguments and return results, `x9`–`x15` are
//! corruptible, `x16`/`x17` are the intra-procedure-call scratch registers,
//! `x18` is the platform register and is **never touched here**, and
//! `x19`–`x28` are callee-saved.
//!
//! The ABI is named as `extern "C"`, which on an aarch64 target *is* AAPCS64.
//! The x86 backend writes `extern "sysv64"` because that spelling exists;
//! there is no `extern "aapcs64"`, and `extern "C"` is the correct and only
//! spelling for the platform ABI a compiled block must follow.

#![allow(unsafe_code)]

use core::ffi::c_void;

use crate::core::error::BusError;
use crate::ir::{Block, InsnStart, IrHost, MemOp, RegSlot};
use crate::jit::FastMem;

/// Why a compiled block stopped, as generated code reports it in `x0`.
///
/// A plain integer rather than an enum, because the producer is machine code.
/// The values are [`jit::x86::rt::status`](crate::jit) — deliberately the same
/// numbers, because a dispatcher that grew a second mapping would be a second
/// thing to get wrong.
pub mod status {
    /// `exit_tb`: back to the dispatcher, with the PC in a guest slot.
    pub const EXIT: u64 = 0;
    /// `goto_tb`: on to a statically known successor, in the context's
    /// `out_pc`.
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
/// Zero is *no error*, so every variant is one more than its position. Written
/// twice on purpose — once each way — and
/// `every_bus_error_survives_the_round_trip` checks the pair.
const fn error_code(e: BusError) -> u64 {
    match e {
        BusError::Unassigned => 1,
        BusError::BadAccess => 2,
        BusError::Protected => 3,
        BusError::Retry => 4,
    }
}

/// The error a code names.
///
/// `pub` rather than `pub(super)` because `rt` — its only caller —
/// is compiled on one host and this file on every host, and a function whose
/// visibility made it dead code off aarch64 would be a warning everywhere
/// else.
pub const fn error_of(code: u64) -> BusError {
    match code {
        1 => BusError::Unassigned,
        3 => BusError::Protected,
        4 => BusError::Retry,
        // Anything unaccounted for is the most conservative answer rather than
        // a panic.
        _ => BusError::BadAccess,
    }
}

/// One deferred bookkeeping event: what an [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) or an
/// [`Opcode::INSN_START`](crate::ir::Opcode::INSN_START) does when its region is replayed.
///
/// A compiled block carries these in a dense array rather than having
/// `flush_thunk` read the IR again; `jit::x86::rt::Event` says what that was
/// worth (a 25% loss when it walked the `Inst` array instead), and the same
/// argument holds here for the same reason — an `Inst` is wide and a hot trace
/// runs a hundred thousand times.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// [`IrHost::charge`], with the tick count [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) carried.
    ///
    /// Only a charge `plan` could not fuse into the boundary ahead of it; see
    /// [`Event::Boundary::ticks`], and `jit::x86::rt::Event` for why.
    Charge(u64),
    /// [`IrHost::insn_start`], by index into [`Block::marks`], and the charge
    /// that follows it.
    Boundary {
        /// The index into [`Block::marks`].
        mark: u32,
        /// Whether a terminator follows this boundary, which is what makes it
        /// an **exit** boundary: the block may not be left there, because
        /// [`InsnStart::pc`] is then a static placeholder and the real
        /// successor is in the slot the map publishes.
        exit: bool,
        /// The [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) fused into this
        /// boundary, or zero for none — `jit::x86::rt::Event` has the argument
        /// and the rule, and `plan` here is the same fusion.
        ticks: u64,
    },
}

/// The same sixteen bytes `jit::x86::rt::Event` asserts, for the same reason.
const _: () = assert!(core::mem::size_of::<Event>() == 16);

/// The execution context a compiled block runs against.
///
/// Every field is a `u64` or a pointer, so generated code writes each one with
/// a single `STR` and no sub-register aliasing. The layout is load-bearing —
/// and it is deliberately **the x86 backend's layout, field for field**, so
/// that the two backends can be read against each other and so that
/// `the_two_backends_agree_about_where_every_context_field_lives` can assert
/// it where both are compiled. See this module's own docs, and the note in
/// `jit::arm64` about hoisting the pair into one shared file.
#[repr(C)]
#[derive(Debug)]
pub struct Ctx {
    /// The temporary frame: **at least** one `u64` per
    /// [`Temp`](crate::ir::Temp) the block declares.
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
    /// Ticks charged by [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE), as `Interp::ticks` counts them.
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
    pub st_base: *const u8,
    /// `entries - 1` for that set.
    pub st_mask: u64,
    /// Everything a store's TLB tag carries besides the page number.
    pub st_tag: u64,
    /// Stores served entirely from an inlined TLB probe.
    pub fast_writes: u64,
    /// The compiled block's deferred bookkeeping, in instruction order.
    pub events: *const Event,
    /// How many [`Ctx::events`] there are, so a range can be clamped.
    pub event_count: u64,
    /// The direct-link thunk, as a plain address, or zero for no linking.
    ///
    /// A `ChainFn` the dispatcher monomorphised over its frontend *and* its
    /// host, which is why it is here and not a slot of [`Vtable`]: that table
    /// is built from the host type alone.
    pub chain: u64,
    /// What the chain thunk is handed besides this context — the dispatcher's
    /// own control block, opaque here.
    pub chain_ctl: *mut c_void,
    /// The [`status`] the block reached its exit with, parked across the call
    /// to the chain thunk, which destroys `x0`.
    pub out_status: u64,
    /// How many blocks the chain executed inside generated code.
    pub blocks_run: u64,
}

/// Byte offsets into [`Ctx`], as generated code bakes them in.
///
/// Every one of them is a multiple of eight and below 32 760, which is what
/// makes each reachable by one `LDR`/`STR` with the unsigned scaled 12-bit
/// immediate — A64's only offset form, and the reason this backend needs the
/// context to be small and flat rather than merely `#[repr(C)]`.
///
/// Six fields have no entry, exactly as in the x86 backend: [`Ctx::ticks`],
/// [`Ctx::retired`], [`Ctx::boundaries`], [`Ctx::boundary_pc`], [`Ctx::mark`]
/// and [`Ctx::published`] are written only by `flush_thunk`, in Rust,
/// through the struct.
pub mod off {
    /// [`Ctx::temps`](super::Ctx::temps).
    pub const TEMPS: u64 = 0;
    /// [`Ctx::vt`](super::Ctx::vt).
    pub const VT: u64 = 8;
    /// [`Ctx::tlb_base`](super::Ctx::tlb_base).
    pub const TLB_BASE: u64 = 32;
    /// [`Ctx::tlb_mask`](super::Ctx::tlb_mask).
    pub const TLB_MASK: u64 = 40;
    /// [`Ctx::tag_bits`](super::Ctx::tag_bits).
    pub const TAG_BITS: u64 = 48;
    /// [`Ctx::out_pc`](super::Ctx::out_pc).
    pub const OUT_PC: u64 = 56;
    /// [`Ctx::fault_at`](super::Ctx::fault_at).
    pub const FAULT_AT: u64 = 104;
    /// [`Ctx::fault_error`](super::Ctx::fault_error).
    pub const FAULT_ERROR: u64 = 112;
    /// [`Ctx::committed`](super::Ctx::committed).
    pub const COMMITTED: u64 = 120;
    /// [`Ctx::fast_hits`](super::Ctx::fast_hits).
    pub const FAST_HITS: u64 = 136;
    /// [`Ctx::st_base`](super::Ctx::st_base).
    pub const ST_BASE: u64 = 144;
    /// [`Ctx::st_mask`](super::Ctx::st_mask).
    pub const ST_MASK: u64 = 152;
    /// [`Ctx::st_tag`](super::Ctx::st_tag).
    pub const ST_TAG: u64 = 160;
    /// [`Ctx::fast_writes`](super::Ctx::fast_writes).
    pub const FAST_WRITES: u64 = 168;
    /// [`Ctx::chain`](super::Ctx::chain).
    pub const CHAIN: u64 = 192;
    /// [`Ctx::out_status`](super::Ctx::out_status).
    pub const OUT_STATUS: u64 = 208;
}

/// The direct-link thunk: what a block calls at its exit instead of returning.
///
/// Handed the [`Ctx`] it is running against — [`Ctx::chain_ctl`] is how it
/// finds everything else — and answers with the host address of the
/// successor's **chain entry**, or zero for *stop here*.
///
/// # Safety
///
/// An implementation is entered from machine code, so it must be `extern "C"`,
/// must not unwind, and must return either zero or the address of a chain
/// entry in a code buffer that is live and executable — see `rt`'s
/// `chain_thunk`, which is the only implementation.
pub type ChainFn = unsafe extern "C" fn(*mut c_void) -> u64;

/// The thunks generated code calls, one table per host type.
///
/// Indirect through a table rather than an immediate call address, because the
/// addresses are monomorphized per `H` and a block compiled for one host must
/// be runnable against another of the same type without being compiled again.
/// On this architecture there is a second reason: A64 has no call-with-64-bit-
/// immediate at all, so an absolute call address would be four `MOVZ`/`MOVK`
/// and a `BLR` where a table is one `LDR` and a `BLR`.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Vtable {
    /// Replay a range of the block's charges and boundaries.
    ///
    /// Returns non-zero when [`IrHost::spent`] stopped the replay at a
    /// boundary, which generated code answers by leaving the block with
    /// [`status::SPENT`].
    pub flush: unsafe extern "C" fn(*mut c_void, u64, u64) -> u64,
    /// [`IrHost::read_slot`], publishing first if the slot is shadowed.
    pub get_slot: unsafe extern "C" fn(*mut c_void, u64) -> u64,
    /// [`IrHost::load`]. Returns an error code; the value goes to `out`.
    pub load: unsafe extern "C" fn(*mut c_void, *const MemOp, u64, *mut u64) -> u64,
    /// [`IrHost::store`]. Returns an error code.
    pub store: unsafe extern "C" fn(*mut c_void, *const MemOp, u64, u64) -> u64,
    /// [`FastMem::note_fast_load`], for an access the backend served itself.
    pub fast_tick: unsafe extern "C" fn(*mut c_void),
    /// [`FastMem::note_fast_store`], for a store the backend served itself.
    pub fast_store: unsafe extern "C" fn(*mut c_void, u64, u64),
}

/// Byte offsets into [`Vtable`], as generated code bakes them in.
pub mod vt {
    /// [`Vtable::flush`](super::Vtable::flush).
    pub const FLUSH: u64 = 0;
    /// [`Vtable::get_slot`](super::Vtable::get_slot).
    pub const GET_SLOT: u64 = 8;
    /// [`Vtable::load`](super::Vtable::load).
    pub const LOAD: u64 = 16;
    /// [`Vtable::store`](super::Vtable::store).
    pub const STORE: u64 = 24;
    /// [`Vtable::fast_tick`](super::Vtable::fast_tick).
    pub const FAST_TICK: u64 = 32;
    /// [`Vtable::fast_store`](super::Vtable::fast_store).
    pub const FAST_STORE: u64 = 40;
}

/// Reconstitute the context a thunk was handed.
///
/// # Safety
///
/// `ctx` must be the pointer generated code was entered with, which
/// `Engine::run` takes from a live `&mut Ctx` it holds for the whole call.
#[inline]
pub(super) unsafe fn ctx_of<'a>(ctx: *mut c_void) -> &'a mut Ctx {
    // SAFETY: the caller's obligation, stated above. The reference does not
    // outlive the thunk, and generated code holds no Rust reference of its
    // own, so this is the only live borrow of the context while it exists.
    unsafe { &mut *ctx.cast::<Ctx>() }
}

/// Reconstitute the host a context names.
///
/// # Safety
///
/// `c.host` must be a pointer to a live `H` — `Engine::run` takes it from the
/// `&mut H` it was called with and holds that borrow across the call — and `H`
/// must be the type the [`Vtable`] was built for, which it is because the two
/// are set from the same monomorphization.
#[inline]
pub(super) unsafe fn host_of<'a, H>(c: &mut Ctx) -> &'a mut H {
    // SAFETY: the caller's obligation, stated above.
    unsafe { &mut *c.host.cast::<H>() }
}

/// The temporary frame a context names.
///
/// # Safety
///
/// `c.temps` must point at `len` initialized `u64`s, which `Engine::run`
/// establishes from a `Vec` grown to at least the block's temporary count.
/// *Initialized*, not meaningful: a slot the executed path never assigned
/// holds whatever an earlier block left there.
#[inline]
pub(super) unsafe fn temps_of<'a>(c: &Ctx, len: usize) -> &'a [u64] {
    // SAFETY: the caller's obligation, stated above.
    unsafe { core::slice::from_raw_parts(c.temps, len) }
}

/// Materialize the pending boundary's live mapping into guest state.
///
/// `ir::interp`'s `publish`, in the shape a thunk can call: idempotent, a
/// no-op when nothing is pending, and reading the temporaries out of the frame
/// rather than out of an interpreter.
pub fn publish<H: IrHost + ?Sized>(c: &mut Ctx, block: &Block, temps: &[u64], host: &mut H) {
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
/// `ir::Interp`'s [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) and [`Opcode::INSN_START`](crate::ir::Opcode::INSN_START) arms
/// **moved**, not batched: the same calls, with the same arguments, in the
/// same order. Generated code contributes only the range, which it knows
/// statically — see [`compile`](super::compile)'s "Deferred bookkeeping" for
/// why a static range is exactly the set of instructions that ran.
unsafe extern "C" fn flush_thunk<H: IrHost + FastMem>(raw: *mut c_void, lo: u64, hi: u64) -> u64 {
    // SAFETY: `raw` is the context `Engine::run` entered generated code with,
    // `c.host` is the `&mut H` it was called with, `c.block` is the `&Block`
    // it holds for the whole call, and `c.events` points at the `Box<[Event]>`
    // the `Compiled` owns for at least as long as its code. All four are live
    // for the whole call and name four distinct objects, so the `&mut Ctx`,
    // the `&mut H`, the `&Block` and the `&[Event]` here do not alias. The
    // range is clamped rather than trusted.
    unsafe {
        let c = ctx_of(raw);
        let block = &*c.block;
        let all = core::slice::from_raw_parts(c.events, c.event_count as usize);
        let hi = (hi as usize).min(all.len());
        // Clamped to `hi`, not to the table, so a corrupted immediate cannot
        // produce a backwards slice — the same two clamps, in the same order,
        // as the x86 backend's, and for the reason written out there.
        let lo = (lo as usize).min(hi);
        for event in &all[lo..hi] {
            match *event {
                Event::Charge(ticks) => {
                    c.ticks = c.ticks.wrapping_add(ticks);
                    c.committed = 1;
                    host_of::<H>(c).charge(ticks);
                }
                Event::Boundary {
                    mark: index,
                    exit,
                    ticks,
                } => {
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
                    // `Interp` does it.
                    c.retired = c.ticks;
                    // Restart granularity is the guest instruction.
                    c.committed = 0;
                    host_of::<H>(c).insn_start(mark);
                    // The tick allowance, asked exactly where `Interp` asks it
                    // and under exactly the same two guards: never at the
                    // block's first boundary, and never at an exit boundary.
                    if c.boundaries > 1 && !exit && host_of::<H>(c).spent() {
                        return 1;
                    }
                    // The fused charge, **after** the return above: a boundary
                    // that stops the block unwinds the guest instruction it
                    // begins, and that instruction's own charge is part of
                    // what is unwound. `jit::x86::rt`'s flush has the long
                    // form.
                    if ticks != 0 {
                        c.ticks = c.ticks.wrapping_add(ticks);
                        c.committed = 1;
                        host_of::<H>(c).charge(ticks);
                    }
                }
            }
        }
        0
    }
}

unsafe extern "C" fn get_slot_thunk<H: IrHost + FastMem>(raw: *mut c_void, slot: u64) -> u64 {
    // SAFETY: as `flush_thunk`. `temps_of` is given the block's own temporary
    // count, which is the length `Engine::run` sized the frame to.
    unsafe {
        let c = ctx_of(raw);
        let block = &*c.block;
        let slot = RegSlot(slot as u16);
        if shadowed(c, block, slot) {
            // Guest state is published lazily, so a slot the current boundary
            // binds is stale in the host until it is written out.
            let temps = temps_of(c, block.temp_count());
            let host = host_of::<H>(c);
            publish(c, block, temps, host);
        }
        host_of::<H>(c).read_slot(slot) as u64
    }
}

unsafe extern "C" fn load_thunk<H: IrHost + FastMem>(
    raw: *mut c_void,
    mem: *const MemOp,
    addr: u64,
    out: *mut u64,
) -> u64 {
    // SAFETY: as `flush_thunk`. `mem` points into the compiled block's own
    // descriptor table, which `Compiled` owns in a `Box<[MemOp]>` that
    // outlives the run; `out` is the eight bytes of stack the prologue
    // reserved.
    unsafe {
        let c = ctx_of(raw);
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

unsafe extern "C" fn store_thunk<H: IrHost + FastMem>(
    raw: *mut c_void,
    mem: *const MemOp,
    addr: u64,
    value: u64,
) -> u64 {
    // SAFETY: as `load_thunk`.
    unsafe {
        let c = ctx_of(raw);
        let mem = &*mem;
        match host_of::<H>(c).store(mem, addr, value) {
            Ok(()) => 0,
            Err(e) => error_code(e),
        }
    }
}

unsafe extern "C" fn fast_tick_thunk<H: IrHost + FastMem>(raw: *mut c_void) {
    // SAFETY: as `flush_thunk`.
    unsafe {
        let c = ctx_of(raw);
        host_of::<H>(c).note_fast_load();
    }
}

unsafe extern "C" fn fast_store_thunk<H: IrHost + FastMem>(
    raw: *mut c_void,
    addr: u64,
    bytes: u64,
) {
    // SAFETY: as `flush_thunk`.
    unsafe {
        let c = ctx_of(raw);
        host_of::<H>(c).note_fast_store(addr, bytes);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_context_layout_a_code_generator_bakes_in_is_the_one_rust_built() {
        // Generated code reaches every one of these with an immediate
        // displacement it cannot re-derive. Adding a field or reordering two
        // would silently point an `LDR` at the wrong one.
        assert_eq!(core::mem::offset_of!(Ctx, temps) as u64, off::TEMPS);
        assert_eq!(core::mem::offset_of!(Ctx, vt) as u64, off::VT);
        assert_eq!(core::mem::offset_of!(Ctx, tlb_base) as u64, off::TLB_BASE);
        assert_eq!(core::mem::offset_of!(Ctx, tlb_mask) as u64, off::TLB_MASK);
        assert_eq!(core::mem::offset_of!(Ctx, tag_bits) as u64, off::TAG_BITS);
        assert_eq!(core::mem::offset_of!(Ctx, out_pc) as u64, off::OUT_PC);
        assert_eq!(core::mem::offset_of!(Ctx, fault_at) as u64, off::FAULT_AT);
        assert_eq!(
            core::mem::offset_of!(Ctx, fault_error) as u64,
            off::FAULT_ERROR
        );
        assert_eq!(core::mem::offset_of!(Ctx, committed) as u64, off::COMMITTED);
        assert_eq!(core::mem::offset_of!(Ctx, fast_hits) as u64, off::FAST_HITS);
        assert_eq!(core::mem::offset_of!(Ctx, st_base) as u64, off::ST_BASE);
        assert_eq!(core::mem::offset_of!(Ctx, st_mask) as u64, off::ST_MASK);
        assert_eq!(core::mem::offset_of!(Ctx, st_tag) as u64, off::ST_TAG);
        assert_eq!(
            core::mem::offset_of!(Ctx, fast_writes) as u64,
            off::FAST_WRITES
        );
        assert!(core::mem::offset_of!(Ctx, events) > off::FAST_WRITES as usize);
        // The direct link's two, which generated code reaches with a scaled
        // immediate exactly as it reaches the rest.
        assert_eq!(core::mem::offset_of!(Ctx, chain) as u64, off::CHAIN);
        assert_eq!(
            core::mem::offset_of!(Ctx, out_status) as u64,
            off::OUT_STATUS
        );
    }

    #[test]
    fn every_context_offset_is_reachable_by_one_scaled_load() {
        // A64's only offset form for a 64-bit load is an unsigned 12-bit
        // immediate scaled by eight, so every field this backend names has to
        // be eight-aligned and inside 32 760 bytes. That is a property of the
        // *architecture* rather than of this struct, which is why it is
        // asserted rather than assumed: a context that grew past it would
        // still compile and would emit no load at all.
        for at in [
            off::TEMPS,
            off::VT,
            off::TLB_BASE,
            off::TLB_MASK,
            off::TAG_BITS,
            off::OUT_PC,
            off::FAULT_AT,
            off::FAULT_ERROR,
            off::COMMITTED,
            off::FAST_HITS,
            off::ST_BASE,
            off::ST_MASK,
            off::ST_TAG,
            off::FAST_WRITES,
            off::CHAIN,
            off::OUT_STATUS,
        ] {
            assert!(at.is_multiple_of(8) && at / 8 <= 4095, "{at} is reachable");
        }
    }

    #[test]
    fn the_thunk_table_layout_is_the_one_generated_code_indexes() {
        assert_eq!(core::mem::offset_of!(Vtable, flush) as u64, vt::FLUSH);
        assert_eq!(core::mem::offset_of!(Vtable, get_slot) as u64, vt::GET_SLOT);
        assert_eq!(core::mem::offset_of!(Vtable, load) as u64, vt::LOAD);
        assert_eq!(core::mem::offset_of!(Vtable, store) as u64, vt::STORE);
        assert_eq!(
            core::mem::offset_of!(Vtable, fast_tick) as u64,
            vt::FAST_TICK
        );
        assert_eq!(
            core::mem::offset_of!(Vtable, fast_store) as u64,
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

    /// The two backends' contexts are the same layout, where both are built.
    ///
    /// They are two `struct`s in two files today, and this is what stops them
    /// from drifting apart while that is true: a field added to one and not
    /// the other is a failure here rather than a divergence discovered on a
    /// runner nobody was looking at. It only runs on an x86-64 Linux host with
    /// `jit-x86` on — which is most of this project's CI — and the day the
    /// pair is hoisted into one shared file it should be deleted, not kept.
    #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
    #[test]
    fn the_two_backends_agree_about_where_every_context_field_lives() {
        use crate::jit::x86::rt::off as x86;
        assert_eq!(off::TEMPS as i32, x86::TEMPS);
        assert_eq!(off::VT as i32, x86::VT);
        assert_eq!(off::TLB_BASE as i32, x86::TLB_BASE);
        assert_eq!(off::TLB_MASK as i32, x86::TLB_MASK);
        assert_eq!(off::TAG_BITS as i32, x86::TAG_BITS);
        assert_eq!(off::OUT_PC as i32, x86::OUT_PC);
        assert_eq!(off::FAULT_AT as i32, x86::FAULT_AT);
        assert_eq!(off::FAULT_ERROR as i32, x86::FAULT_ERROR);
        assert_eq!(off::COMMITTED as i32, x86::COMMITTED);
        assert_eq!(off::FAST_HITS as i32, x86::FAST_HITS);
        assert_eq!(off::ST_BASE as i32, x86::ST_BASE);
        assert_eq!(off::ST_MASK as i32, x86::ST_MASK);
        assert_eq!(off::ST_TAG as i32, x86::ST_TAG);
        assert_eq!(off::FAST_WRITES as i32, x86::FAST_WRITES);
        assert_eq!(
            core::mem::size_of::<Ctx>(),
            core::mem::size_of::<crate::jit::x86::rt::Ctx>(),
            "one context, two declarations"
        );
    }

    /// And about what each status code means.
    #[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
    #[test]
    fn the_two_backends_agree_about_every_status_code() {
        use crate::jit::x86::rt::status as x86;
        assert_eq!(status::EXIT, x86::EXIT);
        assert_eq!(status::GOTO, x86::GOTO);
        assert_eq!(status::LOOKUP, x86::LOOKUP);
        assert_eq!(status::FAULT, x86::FAULT);
        assert_eq!(status::SPENT, x86::SPENT);
    }
}
