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
use crate::jit::{CodeRef, FastMem};

use super::abi::{Ctx, Vtable, error_of, publish, status};
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
        self.arena.push(compiled.at(offset));
        self.stats.compiled += 1;
        Ok(CodeRef {
            index,
            generation: self.buf.generation(),
        })
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
        if !self.is_live(code) {
            return None;
        }
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
        if self.temps.len() < block.temp_count() {
            self.temps.resize(block.temp_count(), 0);
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
            block: core::ptr::from_ref(block),
            tlb_base: plan.map_or(core::ptr::null(), |p| p.set.base),
            tlb_mask: plan.map_or(0, |p| p.set.mask),
            tag_bits: plan.map_or(0, |p| p.tag),
            out_pc: 0,
            ticks: 0,
            retired: 0,
            boundaries: 0,
            boundary_pc: block.entry_pc,
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

        self.stats.executed += 1;
        self.stats.fast_loads += ctx.fast_hits;
        self.stats.fast_stores += ctx.fast_writes;
        self.ticks = ctx.ticks;
        self.boundaries = ctx.boundaries;
        self.mark = u32::try_from(ctx.mark).ok();

        // Whatever happened — an exit, a fault, a stale branch — the guest's
        // architectural state is materialized before the caller can look at
        // it, in one place, exactly as `Interp::run` does it.
        publish(&mut ctx, block, &self.temps, host);

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
