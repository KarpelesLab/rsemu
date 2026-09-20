//! The engine: modules in a bounded table, the imports they call, and the
//! run that turns a status code back into an [`Outcome`].
//!
//! # Where the IR's semantics live
//!
//! In `Thunks`, and nowhere else. A generated module does arithmetic and
//! control flow; every *observable* thing a block does — reading a guest slot,
//! a load, a store, a tick, a boundary, a fault — is an import call, and this
//! file is what those imports do. It is deliberately a transcription of
//! `ir::interp`'s corresponding arms rather than a fresh reading of the IR:
//! the interpreter is the oracle (CLAUDE.md, "CPU cores"), so where the two
//! could differ, this one is wrong by definition.
//!
//! # No `unsafe`
//!
//! Not one block, and not one `#[allow(unsafe_code)]`. Both native backends
//! need the JIT code buffer — CLAUDE.md's second sanctioned site — because a
//! host must be handed a raw pointer to cross into machine code and must
//! reconstitute `&mut` references to cross back. Neither crossing exists here:
//! a module is a `Vec<u8>`, entering it is a function call in safe Rust, and an
//! import's arguments are integers. That is the one unambiguous engineering
//! win in this backend and it is worth stating in the file rather than only in
//! a design note.
//!
//! # Bounded, with eviction
//!
//! `ROADMAP.md` §11.4: *"module count is bounded with an LRU eviction of cold
//! code"*. [`Engine`] holds a fixed table of slots; a slot carries its own
//! generation, and evicting one bumps that generation so every [`CodeRef`]
//! naming it stops being live. That is the same staleness answer the native
//! backends give for a code-buffer reset — [`Engine::is_live`] — narrowed from
//! the whole buffer to one slot, which is what makes eviction *least recently
//! used* rather than *throw everything away*.

use alloc::format;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::error::{BusError, Error, Result};
use crate::ir::{Block, Fault, IrHost, MemOp, Opcode, Outcome, RegSlot, Temp};
use crate::jit::cache::CodeRef;

use super::abi::{answer, func, note, status, temp_offset};
use super::compile::{Compiled, Refusal, compile};
use super::exec::{self, Env, Program};

/// How many modules an engine keeps by default.
///
/// A module per block, so this is a block cache in front of a block cache and
/// it does not need to be large: the outer [`BlockCache`](crate::jit::BlockCache)
/// holds tens of thousands of blocks and this holds the ones that were hot
/// enough to be compiled recently. Eviction costs a recompile, not a wrong
/// answer.
pub const DEFAULT_MODULES: usize = 4096;

/// What an engine has been asked to do.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EngineStats {
    /// Blocks lowered to a module.
    pub compiled: u64,
    /// Blocks refused, which run on the interpreter instead.
    pub refused: u64,
    /// Modules entered.
    pub executed: u64,
    /// Modules evicted to make room.
    pub evicted: u64,
    /// Bytes of wasm currently resident.
    pub bytes: u64,
}

/// One resident module.
#[derive(Debug)]
struct Resident {
    program: Program,
    compiled: Compiled,
}

#[derive(Debug, Default)]
struct Slot {
    generation: u64,
    used: u64,
    resident: Option<Resident>,
}

/// The run state a block's imports maintain.
///
/// Field for field what [`Interp`](crate::ir::Interp) holds, because a
/// compiled run and an interpreted one have to be indistinguishable to the
/// guest and the easiest way to be sure is to keep the same state under the
/// same names.
#[derive(Debug, Default)]
struct RunState {
    ticks: u64,
    mark: Option<u32>,
    published: bool,
    boundaries: u64,
    boundary_pc: u64,
    retired: u64,
    committed: bool,
    fault: Option<Fault>,
    failure: Option<String>,
}

/// A wasm JIT engine.
#[derive(Debug)]
pub struct Engine {
    slots: Vec<Slot>,
    /// A monotonic counter, so "least recently used" is a comparison.
    clock: u64,
    /// The linear memory a generated module imports, holding the frame.
    mem: Vec<u8>,
    state: RunState,
    stats: EngineStats,
}

impl Default for Engine {
    fn default() -> Engine {
        Engine::new()
    }
}

impl Engine {
    /// An engine holding [`DEFAULT_MODULES`] modules.
    #[must_use]
    pub fn new() -> Engine {
        Engine::with_capacity(DEFAULT_MODULES)
    }

    /// An engine holding `modules` modules.
    #[must_use]
    pub fn with_capacity(modules: usize) -> Engine {
        let modules = modules.max(1);
        let mut slots = Vec::with_capacity(modules);
        slots.resize_with(modules, Slot::default);
        Engine {
            slots,
            clock: 0,
            // One wasm page, which is what the module's memory import asks
            // for, grown on demand by a block with a long temporary frame.
            mem: vec![0u8; 65536],
            state: RunState::default(),
            stats: EngineStats::default(),
        }
    }

    /// What this engine has been asked to do.
    #[inline]
    #[must_use]
    pub fn stats(&self) -> EngineStats {
        self.stats
    }

    /// Ticks charged during the last run, as `Interp::ticks` counts them.
    #[inline]
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.state.ticks
    }

    /// How many boundaries the last run passed.
    #[inline]
    #[must_use]
    pub fn boundaries(&self) -> u64 {
        self.state.boundaries
    }

    /// The boundary the last run reached, by index into `Block::marks`.
    #[inline]
    #[must_use]
    pub fn mark(&self) -> Option<u32> {
        self.state.mark
    }

    /// What a temporary held when the last run ended, for a differential
    /// harness.
    ///
    /// Only the temporaries a boundary names are readable, because only those
    /// are written through to the frame — the rest lived in wasm locals, which
    /// stopped existing when the function returned. `None` therefore means
    /// *not observable* rather than *not set*, and a harness compares the
    /// intersection.
    #[must_use]
    pub fn temp_value(&self, code: CodeRef, temp: Temp) -> Option<u64> {
        let slot = self.slots.get(code.index as usize)?;
        if slot.generation != code.generation {
            return None;
        }
        let resident = slot.resident.as_ref()?;
        if !resident.compiled.written_through(temp) {
            return None;
        }
        self.frame_read(temp.0)
    }

    /// Whether `code` still names a resident module.
    ///
    /// The same question [`jit::x86::rt`](crate::jit) asks of its code buffer,
    /// answered per slot rather than per buffer: an eviction bumps one slot's
    /// generation, so a `CodeRef` into it stops matching and the dispatcher
    /// recompiles.
    #[must_use]
    pub fn is_live(&self, code: CodeRef) -> bool {
        self.slots
            .get(code.index as usize)
            .is_some_and(|s| s.generation == code.generation && s.resident.is_some())
    }

    /// Lower `block` to a module and make it resident.
    ///
    /// # Errors
    ///
    /// A [`Refusal`]. Never an `Error`: a refused block is interpreted.
    pub fn compile(&mut self, block: &Block) -> core::result::Result<CodeRef, Refusal> {
        let compiled = compile(block).inspect_err(|_| self.stats.refused += 1)?;
        // A module this engine emitted that this engine cannot decode is a bug
        // in one of the two files, so it is refused rather than panicked on —
        // the fuzz targets reach `compile` and a panic there is a finding
        // about the wrong thing.
        let program = exec::parse(compiled.module())
            .map_err(|_| Refusal::Shape("the emitted module does not decode"))?;

        let index = self.evict();
        self.clock += 1;
        let bytes = compiled.module().len() as u64;
        let slot = &mut self.slots[index];
        if let Some(old) = slot.resident.take() {
            self.stats.bytes -= old.compiled.module().len() as u64;
            self.stats.evicted += 1;
            slot.generation += 1;
        }
        slot.used = self.clock;
        slot.resident = Some(Resident { program, compiled });
        self.stats.bytes += bytes;
        self.stats.compiled += 1;

        Ok(CodeRef {
            index: index as u32,
            generation: slot.generation,
            // A wasm module has no chain entry and no event table: every exit
            // returns to the dispatcher, because an instantiated module's
            // branches cannot be patched (`ROADMAP.md` §11.4).
            chain: 0,
            events: 0,
            event_count: 0,
        })
    }

    /// The slot a new module goes in: a free one, or the least recently used.
    fn evict(&mut self) -> usize {
        let mut best = 0usize;
        let mut best_used = u64::MAX;
        for (i, slot) in self.slots.iter().enumerate() {
            if slot.resident.is_none() {
                return i;
            }
            if slot.used < best_used {
                best_used = slot.used;
                best = i;
            }
        }
        best
    }

    /// Execute the block `code` names against `host`.
    ///
    /// `None` when `code` is no longer live, which the dispatcher answers by
    /// compiling again or by interpreting — the same shape
    /// `jit::x86::rt::Engine::run` has, for the same reason.
    ///
    /// # Errors
    ///
    /// [`Error::Ir`] for a malformed block, and [`Error::Bus`] carrying
    /// [`BusError::Retry`] when a host asks to retry an access that can no
    /// longer be retried. Exactly `Interp::run`'s two, because they are the
    /// same two conditions.
    pub fn run<H: IrHost + ?Sized>(
        &mut self,
        block: &Block,
        code: CodeRef,
        host: &mut H,
    ) -> Option<Result<Outcome>> {
        if !self.is_live(code) {
            return None;
        }
        let index = code.index as usize;
        self.clock += 1;
        self.slots[index].used = self.clock;
        self.stats.executed += 1;

        self.state = RunState {
            published: true,
            boundary_pc: block.entry_pc,
            ..RunState::default()
        };

        let Engine {
            slots, mem, state, ..
        } = self;
        let resident = slots[index]
            .resident
            .as_ref()
            .expect("liveness was just checked");
        let frame = resident.compiled.frame_bytes();
        if mem.len() < frame {
            // Round up to whole wasm pages, which is the unit a real embedder
            // grows a memory in (core specification §4.4.7).
            mem.resize(frame.div_ceil(65536) * 65536, 0);
        }
        mem[..frame].fill(0);

        // The frame starts at zero: this engine's linear memory holds nothing
        // else. An embedder's would, which is why the offset is a parameter of
        // the generated function rather than baked into it.
        const FRAME: u32 = 0;
        let mut env = Thunks {
            state,
            host,
            block,
            mems: resident.compiled.mem_ops(),
            frame: FRAME,
        };
        let outcome = exec::run(
            &resident.program,
            &[0, i64::from(FRAME)],
            mem.as_mut_slice(),
            &mut env,
        );

        let out = match outcome {
            Err(e) => Err(Error::Ir(format!(
                "block {:#x} under the wasm backend: {e}",
                block.entry_pc
            ))),
            Ok(code) => self.finish(block, code),
        };
        // Whatever happened, the guest's architectural state is materialized
        // before the caller can look at it — `Interp::run`'s own rule, and its
        // reason: one place rather than six.
        self.publish(block, host);
        Some(out)
    }

    /// Turn the status a module returned into an [`Outcome`].
    fn finish(&mut self, block: &Block, code: i64) -> Result<Outcome> {
        match code {
            status::EXIT => Ok(Outcome::Exit),
            status::GOTO => Ok(Outcome::Goto {
                pc: self.out_word(),
            }),
            status::LOOKUP => Ok(Outcome::Lookup {
                pc: self.out_word(),
            }),
            status::FAULT => {
                Ok(Outcome::Fault(self.state.fault.expect(
                    "a fault status is only returned once a fault is recorded",
                )))
            }
            status::SPENT => {
                let pc = self
                    .state
                    .mark
                    .and_then(|m| block.marks().get(m as usize))
                    .map_or(block.entry_pc, |m| m.pc);
                Ok(Outcome::Spent { pc })
            }
            status::ERROR => match self.state.failure.take() {
                Some(what) if what == BUS_RETRY => Err(Error::Bus(BusError::Retry)),
                Some(what) => Err(Error::Ir(what)),
                None => Err(Error::Ir(format!(
                    "block {:#x} ran off the end without reaching a terminator",
                    block.entry_pc
                ))),
            },
            other => Err(Error::Ir(format!(
                "the wasm backend returned an unknown status {other}"
            ))),
        }
    }

    /// The frame's out word: a successor PC, or a load's result.
    fn out_word(&self) -> u64 {
        let mut buf = [0u8; 8];
        buf.copy_from_slice(&self.mem[..8]);
        u64::from_le_bytes(buf)
    }

    /// Temporary `n`, from the frame.
    fn frame_read(&self, n: u32) -> Option<u64> {
        let at = temp_offset(n) as usize;
        let bytes = self.mem.get(at..at + 8)?;
        let mut buf = [0u8; 8];
        buf.copy_from_slice(bytes);
        Some(u64::from_le_bytes(buf))
    }

    /// Materialize the pending boundary's live mapping into guest state.
    fn publish<H: IrHost + ?Sized>(&mut self, block: &Block, host: &mut H) {
        if self.state.published {
            return;
        }
        self.state.published = true;
        let Some(mark) = self.state.mark.and_then(|m| block.marks().get(m as usize)) else {
            return;
        };
        for &(slot, temp) in &mark.live {
            if let Some(value) = self.frame_read(temp.0) {
                host.write_slot(slot, u128::from(value));
            }
        }
    }
}

/// The message a rejected retry carries, so the status round-trip can name it.
const BUS_RETRY: &str = "a retry that can no longer be one";

/// The imports a generated module calls, and what they do.
struct Thunks<'a, H: ?Sized> {
    state: &'a mut RunState,
    host: &'a mut H,
    block: &'a Block,
    mems: &'a [MemOp],
    /// Where the frame this block was entered with starts.
    ///
    /// Carried rather than assumed, because a load leaves its value in the
    /// frame's *out word* and generated code reads it back from there: the two
    /// have to agree about where the frame is, and an agreement that holds
    /// only because both happen to say zero is one edit from being wrong.
    frame: u32,
}

impl<H: IrHost + ?Sized> Thunks<'_, H> {
    /// Whether the pending boundary binds `slot` to a temporary.
    fn shadowed(&self, slot: RegSlot) -> bool {
        !self.state.published
            && self
                .state
                .mark
                .and_then(|m| self.block.marks().get(m as usize))
                .is_some_and(|mark| mark.live.iter().any(|&(s, _)| s == slot))
    }

    /// `Engine::publish`, reachable from inside a run.
    fn publish(&mut self, mem: &[u8]) {
        let base = self.frame as usize;
        if self.state.published {
            return;
        }
        self.state.published = true;
        let Some(mark) = self
            .state
            .mark
            .and_then(|m| self.block.marks().get(m as usize))
        else {
            return;
        };
        for &(slot, temp) in &mark.live {
            let at = base + temp_offset(temp.0) as usize;
            if let Some(bytes) = mem.get(at..at + 8) {
                let mut buf = [0u8; 8];
                buf.copy_from_slice(bytes);
                self.host
                    .write_slot(slot, u128::from(u64::from_le_bytes(buf)));
            }
        }
    }

    /// Record a bus fault, or reject it if it is a retry that cannot be one.
    fn fault(&mut self, at: usize, error: BusError) -> i64 {
        if error == BusError::Retry && self.state.committed {
            // The guest instruction has already changed something the world can
            // see, so there is nothing left to restart from. Rejected here
            // rather than passed on, exactly as `Interp::fault` rejects it.
            self.state.failure = Some(String::from(BUS_RETRY));
            return status::ERROR;
        }
        self.state.fault = Some(Fault {
            error,
            at,
            mark: self.state.mark,
            pc: self.state.boundary_pc,
            retired_ticks: self.state.retired,
            charged_ticks: self.state.ticks,
            restartable: !self.state.committed,
        });
        status::FAULT
    }
}

impl<H: IrHost + ?Sized> Env for Thunks<'_, H> {
    fn call(&mut self, index: u32, args: &[i64], mem: &mut [u8]) -> i64 {
        match index {
            func::SLOT => {
                let slot = RegSlot(args[1] as u16);
                if self.shadowed(slot) {
                    // Guest state is published lazily, so a slot the current
                    // boundary binds is stale in the host until it is written
                    // out. `ir::interp`'s GET_SLOT arm has the argument.
                    self.publish(mem);
                }
                self.host.read_slot(slot) as u64 as i64
            }
            func::LOAD => {
                let Some(&mem_op) = self.mems.get(args[1] as usize) else {
                    self.state.failure = Some(String::from("a load named no access descriptor"));
                    return status::ERROR;
                };
                let at = args[3] as usize;
                // A volatile load is a bus cycle whose occurrence the guest can
                // observe even when its value is discarded, so it commits.
                if mem_op.volatile {
                    self.state.committed = true;
                }
                match self.host.load(&mem_op, args[2] as u64) {
                    Ok(v) => {
                        let out = self.frame as usize;
                        mem[out..out + 8].copy_from_slice(&v.to_le_bytes());
                        i64::from(answer::OK)
                    }
                    Err(e) => self.fault(at, e),
                }
            }
            func::STORE => {
                let Some(&mem_op) = self.mems.get(args[1] as usize) else {
                    self.state.failure = Some(String::from("a store named no access descriptor"));
                    return status::ERROR;
                };
                let at = args[4] as usize;
                self.state.committed = true;
                match self.host.store(&mem_op, args[2] as u64, args[3] as u64) {
                    Ok(()) => i64::from(answer::OK),
                    Err(e) => self.fault(at, e),
                }
            }
            func::NOTE => match args[1] as i32 {
                note::CHARGE => {
                    let ticks = args[2] as u64;
                    // Exactly, where it was written: the count is hashed output.
                    self.state.ticks = self.state.ticks.wrapping_add(ticks);
                    self.state.committed = true;
                    self.host.charge(ticks);
                    i64::from(answer::OK)
                }
                kind @ (note::BOUNDARY | note::BOUNDARY_EXIT) => {
                    let index = args[2] as u32;
                    let Some(mark) = self.block.marks().get(index as usize) else {
                        self.state.failure =
                            Some(String::from("the boundary marker points at no record"));
                        return status::ERROR;
                    };
                    // The mapping is remembered, not written out: the previous
                    // boundary's is superseded here.
                    self.state.mark = Some(index);
                    self.state.published = false;
                    self.state.boundaries = self.state.boundaries.wrapping_add(1);
                    self.state.boundary_pc = mark.pc;
                    self.state.retired = self.state.ticks;
                    self.state.committed = false;
                    self.host.insn_start(mark);
                    // The tick allowance, asked here and nowhere else, and not
                    // at the block's first boundary nor at an exit boundary —
                    // `ir::interp`'s INSN_START arm says why for both. Which
                    // kind this is was decided statically at compile time.
                    if kind == note::BOUNDARY && self.state.boundaries > 1 && self.host.spent() {
                        status::SPENT
                    } else {
                        i64::from(answer::OK)
                    }
                }
                other => {
                    self.state.failure = Some(format!("an unknown note kind {other}"));
                    status::ERROR
                }
            },
            other => {
                self.state.failure = Some(format!("a call to import {other}, which is not one"));
                status::ERROR
            }
        }
    }
}

/// Whether this backend lowers `op`, for a caller that has no block yet.
#[must_use]
pub fn compiles(op: Opcode) -> bool {
    super::compile::compiles(op)
}
