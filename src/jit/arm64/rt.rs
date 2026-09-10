//! Entering compiled code: the engine, the buffer it keeps and the run it
//! reports.
//!
//! The half of the boundary that is *this host* — [`abi`](super::abi) is the
//! half that is not. Everything here needs a real A64 machine, because
//! everything here is about calling into bytes, so this file and
//! [`buf`](super::buf) are the two the module gates on `target_arch`.
//!
//! # Why this file also opts into `unsafe`
//!
//! Same subsystem, same sanction: the JIT code buffer. Crossing into generated
//! code is one `unsafe` call, and its obligations are the ones
//! [`CodeBuf::entry`] states.
//!
//! # What the engine owes the interpreter
//!
//! `ir::Interp` is the oracle (CLAUDE.md, "CPU cores", one level down), so
//! this engine reproduces its *observable* behaviour exactly and not merely
//! its results: one [`IrHost::charge`] per [`Opcode::CHARGE`], one
//! [`IrHost::insn_start`] per boundary, guest state published at the same
//! points, and a [`BusError::Retry`] after a commit rejected rather than
//! delivered. The first two are deferred rather than batched, which is
//! [`abi`](super::abi)'s `flush_thunk`.
//!
//! [`Opcode::CHARGE`]: crate::ir::Opcode::CHARGE

#![allow(unsafe_code)]

use alloc::vec::Vec;
use core::ffi::c_void;

use crate::core::error::{BusError, Error, Result};
use crate::ir::{Block, Fault, IrHost, Outcome};
use crate::jit::dispatch::{Chain, Frontend, Step, StoreLog};
use crate::jit::{BlockId, CodeRef, FastMem};

use super::abi::{ChainFn, Ctx, Event, Vtable, error_of, publish, status};
use super::buf::{CodeBuf, DEFAULT_CAPACITY};
use super::compile::{Compiled, Refusal, Regs, compile_with};

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

/// What a dispatcher needs to know about an engine's code buffer to resolve a
/// link without touching the engine.
///
/// The engine is *executing* while the chain thunk runs — its `&mut self` is
/// live in [`Engine::run_chained`]'s frame — so the thunk is handed this
/// snapshot instead. Every field is stable for the whole of a run, and that is
/// a claim rather than a hope: the one thing that invalidates any of them is
/// [`Engine::compile`], and a chain never compiles. The x86-64 backend offers
/// the same type under the same name.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Linkage {
    /// The code buffer's base address.
    pub base: u64,
    /// The generation a [`CodeRef`] must carry to be worth branching to.
    pub generation: u64,
    /// How many temporaries the shared frame holds.
    ///
    /// A block needing more may not be linked to: growing the frame would move
    /// it, and the code standing on it holds its address in a register.
    pub temps: usize,
}

/// The direct link, from the inside: close the block that just exited, and
/// open the one a [`Chain`] hands back.
///
/// The aarch64 twin of `jit::x86::rt`'s thunk, and the same code but for the
/// calling convention: everything `Engine::run` and `Dispatcher::run` used to
/// do around a block, in the same order, minus the two frames, the context and
/// the thunk table.
///
/// The TLB parameters are re-taken, and that is not tidiness: `Frontend::enter`
/// can walk a guest page table, a walk can miss and fill, and a fill can
/// replace the table the inlined probe reads.
unsafe extern "C" fn chain_thunk<F, H>(raw: *mut c_void) -> u64
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
        let c = super::abi::ctx_of(raw);
        let block = &*c.block;
        let temps = super::abi::temps_of(c, block.temp_count());
        // Before anything can observe guest state — the drain below reaches a
        // host, and a fault path reads the boundary's slots.
        let host = super::abi::host_of::<H>(c);
        publish(c, block, temps, host);
        // Every exit is preceded by one boundary that begins no guest
        // instruction, and exactly one exit is reached, so this is what
        // retired — at a fault too.
        let retired = c.boundaries.saturating_sub(1) as usize;
        let chain = &mut *c.chain_ctl.cast::<Chain<'_, F, H>>();
        let next = match c.out_status {
            status::GOTO | status::LOOKUP => Some(c.out_pc),
            status::EXIT => Some(super::abi::host_of::<H>(c).read_slot(chain.pc_slot()) as u64),

            // A fault, or the tick allowance. There is no successor and the
            // block's own status is what the run reports.
            _ => None,
        };
        let Step::Go { id, code, entry } = chain.step(next, retired, super::abi::host_of::<H>(c))
        else {
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
        let host = super::abi::host_of::<H>(c);
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

/// The aarch64 backend: a code buffer, the blocks in it, and a way in.
///
/// Mirrors [`Interp`](crate::ir::Interp)'s surface where the two overlap —
/// [`Engine::ticks`], [`Engine::boundaries`], [`Engine::mark`] — because a
/// dispatcher reads exactly those off whichever engine ran the block, and a
/// backend that reported them differently would make a run's retired
/// instruction count depend on which engine executed it. It is the same
/// surface `jit::x86::rt::Engine` offers, method for method, and deliberately
/// so: the frontends that will eventually reach for it should need a `cfg` and
/// not a second code path.
#[derive(Debug)]
pub struct Engine {
    buf: CodeBuf,
    arena: Vec<Compiled>,
    /// The temporary frame, shared by every block this engine runs.
    ///
    /// Grown to fit and never cleared — see [`Engine::run`].
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
    /// runnable as the control the differential compares against. Blocks
    /// already compiled keep the policy they were compiled under.
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
    /// `None` for a temporary the allocator kept only in a host register: it
    /// is gone once the epilogue has restored the caller's registers, and
    /// handing back its frame slot would be a lie a differential would then
    /// assert. What this never returns `None` for is a temporary an
    /// `InsnStart` names, because that is the state the exception path
    /// materializes and the backend writes those through to the frame.
    #[inline]
    #[must_use]
    pub fn temp_value(&self, temp: crate::ir::Temp) -> Option<u64> {
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
    /// full buffer, which resets it and tries once more.
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
    /// address in a register.
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

    /// Execute the block `id` names, and let it **branch straight on** to its
    /// successors.
    ///
    /// The aarch64 twin of `jit::x86::rt`'s method, with the same contract:
    /// what comes back describes the **last** block of the chain, and every
    /// earlier block was closed off by [`Chain::step`] — the exit boundary's
    /// publish included, so this does not do one.
    ///
    /// # Errors
    ///
    /// As [`Engine::run`].
    ///
    /// # Panics
    ///
    /// If `id` names no resident block, which the caller has just found or
    /// inserted.
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
        // so this is the reference the caller handed over. It is read out here
        // and not held: a chain re-points the context at its successors, and a
        // `&Block` left standing over that would be a reference to a block the
        // cache may since have dropped.
        let (temp_count, entry_pc) = unsafe { ((*block).temp_count(), (*block).entry_pc) };
        let compiled = &self.arena[code.index as usize];
        let offset = compiled.offset();
        // The deferred bookkeeping, taken as a raw slice: the `Box`'s
        // allocation does not move while `self` is borrowed across the call.
        let events = compiled.events();
        let (events, event_count) = (events.as_ptr(), events.len() as u64);
        self.last = Some(code);
        // A high-water mark, not a fresh frame: `compile` writes every
        // frame-homed temporary at its definition and refuses a block that
        // reads one before its definition, so every frame slot generated code
        // reads was written by this execution.
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
        // `compile` — which produces exactly one shape of function: the
        // AAPCS64 `extern "C" fn(*mut c_void) -> u64` `Entry` names. It reads
        // and writes the `Ctx` behind its argument, the temporary frame that
        // context points at, the `MemOp` table `compiled` owns, and guest RAM
        // through host addresses taken from live TLB entries — all of which
        // are alive for the whole call, because `self` and `host` are borrowed
        // mutably across it and `ctx` is a local. `entry` has also run the
        // cache maintenance A64 requires between writing those bytes and
        // fetching them; see `buf`.
        //
        // Guest RAM is both read and written that way. An entry carries a host
        // address only for a whole page of a `RamStore` that outlives this
        // call, and generated code touches exactly the bytes of the access —
        // `store_trunc` emits the width the guest asked for. Those bytes are
        // `AtomicU8`, written elsewhere with relaxed stores, and the emitted
        // `STR` is the instruction a relaxed atomic byte store compiles to; no
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
            // this engine no longer knows which block that was.
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
        // every block including this one — and where `ctx.block` may name a
        // `Block` the cache has since dropped.
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
