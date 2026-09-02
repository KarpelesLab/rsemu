//! The portable IR interpreter backend.
//!
//! `ROADMAP.md` §9 lists this alongside the host code generators, "so an
//! unsupported host degrades in speed rather than failing to run", and §11's
//! target table has a bare-metal row whose engine it is. It is therefore the
//! one backend that must build wherever the IR does — `no_std + alloc`, and
//! with **no `unsafe` at all**: the crate's six sanctioned opt-ins
//! (CLAUDE.md, "`unsafe`") include the JIT's code buffer, which is in `jit/`,
//! and nothing here needs one.
//!
//! It is also the JIT's oracle. CLAUDE.md's "CPU cores" rule makes the guest
//! interpreter the oracle for its own frontend; one level down, this backend
//! is the oracle for every host backend, because a block executed here and the
//! same block executed as generated code must agree on temporaries, on guest
//! memory, and — decision 2 in the module docs — on the tick count, or the
//! phase-5 state-hash gate fails.
//!
//! # Shape
//!
//! [`Interp::run`] takes a [`Block`] and something that owns guest state, and
//! returns why the block stopped ([`Outcome`]). Everything the IR cannot do
//! itself goes through [`IrHost`]: the guest register file, guest memory, the
//! tick counter, and helper calls. The interpreter holds only the temporaries.
//!
//! Values live as `u128` masked to their temporary's type, always — an `i32`
//! temporary is never allowed to carry bits above 32, so every op may assume
//! canonical inputs and each one masks its own output exactly once.
//!
//! # The two decisions this backend has to honour
//!
//! * **[`Opcode::CHARGE`] charges exactly.** The immediate is handed to
//!   [`IrHost::charge`] unrounded and unbatched, at the point it appears.
//! * **[`Opcode::INSN_START`] is a barrier, and it is *lazy*.** The boundary
//!   is announced, and the live `(slot, temp)` mapping is remembered rather
//!   than written out. Architectural state is materialized through
//!   [`IrHost::write_slot`] only when something can observe it — see
//!   "Materializing guest state" below. That is `ROADMAP.md` §9's design
//!   verbatim (*"on a fault, the runtime … materializes the architectural
//!   state from the recorded mapping"*), and it is what lets a guest register
//!   stay in a temporary across a whole superblock instead of being written
//!   back once per guest instruction.
//!
//! # Materializing guest state
//!
//! Between boundaries the host's slot storage is **stale**: the truth is in
//! the temporaries the current [`InsnStart::live`] mapping names. The
//! interpreter publishes that mapping — exactly once, whatever happens
//! afterwards — immediately before any of:
//!
//! * returning an [`Outcome`] or an [`Error`] from [`Interp::run`], which
//!   covers every exit, every fault and every unsupported op;
//! * [`IrHost::call_helper`], because a helper is arbitrary Rust that may read
//!   the guest's registers;
//! * [`IrHost::read_slot`] for a slot the current mapping binds, so a
//!   [`Opcode::GET_SLOT`] can never read a value a temporary has superseded.
//!
//! Nothing else can see guest state. [`IrHost::load`], [`IrHost::store`] and
//! [`IrHost::rmw`] reach guest *memory*, which no design in this crate lets a
//! device use to read a CPU's register file, and [`IrHost::insn_start`] is
//! told the boundary's PC and tick column directly.
//!
//! The saving is the point: a 64-instruction trace with fifteen live registers
//! wrote roughly nine hundred slots eagerly and writes fifteen now.
//!
//! # Where this backend has to *choose*
//!
//! Two things the IR deliberately leaves open, decided here and only here:
//!
//! * **Out-of-range shifts.** [`Opcode::SHL`], [`Opcode::SHR`] and
//!   [`Opcode::SAR`] are documented as undefined when the amount is at least
//!   the type's width, because every guest disagrees and each frontend emits
//!   its own guard. This backend takes the *mathematical* result — zero, or an
//!   all-ones sign fill for [`Opcode::SAR`] — rather than the mask-the-count
//!   behaviour x86-64 and aarch64 give for free. That is the useful choice for
//!   an oracle: a frontend that forgot its guard then **diverges** from the
//!   masking host backends and the differential test reports it, where copying
//!   x86's masking would hide the same bug until it reached a host that does
//!   not mask.
//! * **Where the bitfield ops keep their position and length.** An `Inst` has
//!   one immediate and one `aux` word, and [`Opcode::DEPOSIT`] and
//!   [`Opcode::EXTRACT`] need two numbers, so they are packed into `aux` by
//!   [`bitfield_aux`](crate::ir::bitfield_aux). A backend-local convention is
//!   not where this belongs;
//!   it is recorded here until the first frontend says what it wants.

use crate::core::error::{BusError, Error, Result};
use crate::core::space::MemResult;
use crate::core::value::Width;
use crate::ir::block::{Block, InsnStart, Inst, RegSlot};
use crate::ir::op::{AccessKind, Cond, MemOp, Opcode, Sign, bitfield_parts};
use crate::ir::types::{Temp, Type};
use alloc::format;
use alloc::vec::Vec;

/// Everything the IR cannot do for itself.
///
/// Deliberately small: the guest register file, guest memory, the tick
/// counter, and a way out to code written in Rust. A host implements it over
/// whatever it already has — a CPU core's registers plus an address space, a
/// differential test's scratch arrays — and the interpreter never looks inside
/// any of it.
///
/// `&mut self` throughout, because this is per-CPU execution state reached
/// from one thread at a time, not a device shared between them.
pub trait IrHost {
    /// Read a slot of guest architectural state.
    ///
    /// A [`RegSlot`] is an opaque index into the *host's* slot space: the
    /// frontend numbers it and the backend never interprets the numbering.
    ///
    /// Called for [`Opcode::GET_SLOT`], which is how a block names a guest
    /// register it did not compute — a frontend materializes the state it
    /// reads into temporaries and publishes it back at each boundary, and this
    /// is the materializing half.
    ///
    /// Two other consumers read the same slot space through the same trait,
    /// and should: the fault handler that reconstructs architectural state
    /// from an [`InsnStart`], and the differential harness that compares this
    /// backend against another. The first of those is
    /// `cpu::riscv::differential`, which runs a lifted RISC-V block here and
    /// against the guest's own interpreter.
    fn read_slot(&mut self, slot: RegSlot) -> u128;

    /// Write a slot of guest architectural state.
    ///
    /// Called for every live pair at an [`Opcode::INSN_START`], before the
    /// boundary is announced.
    fn write_slot(&mut self, slot: RegSlot, value: u128);

    /// Perform a guest load.
    ///
    /// The value comes back zero-extended in the low [`MemOp::size`] bits; the
    /// interpreter applies [`MemOp::sign`] and widens to the destination type,
    /// so a host implements one width-driven read and nothing else.
    ///
    /// # Errors
    ///
    /// Whatever the bus said. The interpreter stops the block and reports it.
    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64>;

    /// Perform a guest store. `value` is already truncated to [`MemOp::size`].
    ///
    /// # Errors
    ///
    /// Whatever the bus said.
    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult;

    /// Charge guest ticks.
    ///
    /// Exactly what [`Opcode::CHARGE`] said, where it said it. The count is a
    /// hashed output rather than a budget (module docs, decision 2), so a host
    /// that rounds or batches produces a different state hash from the
    /// interpreter for the same guest.
    fn charge(&mut self, ticks: u64);

    /// A guest instruction boundary has been reached.
    ///
    /// The live slots have **not** been published: they are materialized
    /// lazily, at the points the module docs list, so a host may not read its
    /// own slot storage from here and expect the boundary's state. What it may
    /// read is what it is handed — [`InsnStart::pc`] and [`InsnStart::ticks`]
    /// — which is what a host records so that a fault taken before the next
    /// boundary is delivered at the architecturally correct place.
    fn insn_start(&mut self, mark: &InsnStart);

    /// Perform an atomic read-modify-write, returning the value the location
    /// held before it.
    ///
    /// One method for the whole family, selected by `op`, because eleven trait
    /// methods that differ only in an operator is not a small trait. The
    /// conventions:
    ///
    /// * [`Opcode::CMPXCHG`] — swap in `arg` if the location holds `compare`;
    ///   return the previous value either way.
    /// * [`Opcode::XCHG`] and the `FETCH_*` family — combine `arg` into the
    ///   location, return the previous value. `compare` is unused.
    /// * [`Opcode::LD_EXCL`] — take a reservation and return the value.
    /// * [`Opcode::ST_EXCL`] — store `arg` if the reservation still holds;
    ///   return `1` if the store happened and `0` if it did not. Frontends
    ///   invert as their ISA requires: ARM's `STREX` and RISC-V's `SC` both
    ///   write **zero** on success.
    ///
    /// # Errors
    ///
    /// Whatever the bus said. Defaulted to [`BusError::BadAccess`] so a host
    /// with no atomics — a single-core 8-bit machine, most of our guests —
    /// implements nothing.
    fn rmw(
        &mut self,
        op: Opcode,
        mem: &MemOp,
        addr: u64,
        arg: u64,
        compare: u64,
    ) -> MemResult<u64> {
        let _ = (op, mem, addr, arg, compare);
        Err(BusError::BadAccess)
    }

    /// A memory fence. A no-op on a host with one thread of guest execution.
    fn fence(&mut self) {}

    /// Call a helper by id, with the block's arguments, returning its one or
    /// two results.
    ///
    /// Two results because every soft-float entry point returns a value *and*
    /// its accrued exception flags (`ROADMAP.md` §9.1); a helper with one
    /// result returns anything for the second and the block ignores it.
    ///
    /// # Errors
    ///
    /// [`Error`] rather than [`BusError`]: a helper is arbitrary Rust and its
    /// failures are not all bus faults. A helper that takes a guest
    /// *exception* does not fail — it returns normally, having recorded the
    /// exception in the host's own state.
    fn call_helper(&mut self, id: u32, args: &[u128]) -> Result<(u128, u128)> {
        let _ = (id, args);
        Err(Error::Unimplemented("an IR helper call"))
    }
}

/// Why a block stopped.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Outcome {
    /// [`Opcode::EXIT_TB`]: back to the dispatcher.
    Exit,
    /// [`Opcode::GOTO_TB`]: on to a statically known successor.
    Goto {
        /// The successor's guest PC.
        pc: u64,
    },
    /// [`Opcode::LOOKUP_AND_GOTO`]: the successor is named by a computed PC.
    Lookup {
        /// The computed guest PC to look up.
        pc: u64,
    },
    /// A guest access faulted, and the block stopped where it faulted.
    Fault(Fault),
    /// An opcode this backend does not implement.
    ///
    /// Not an error: [`Opcode`] is an extensible enumeration precisely so a
    /// block can carry an op that predates a given backend, and the caller's
    /// answer is to run the guest's own interpreter for that instruction.
    Unsupported {
        /// The op.
        op: Opcode,
        /// Its index in the block.
        at: usize,
    },
}

/// Where and how a guest access faulted.
///
/// Carries what a fault handler needs to deliver the exception without
/// re-deriving it: which guest instruction was executing, at what PC, and how
/// many ticks had retired when it started.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Fault {
    /// What the bus said.
    pub error: BusError,
    /// The index of the faulting instruction within the block.
    pub at: usize,
    /// The index into [`Block::marks`] of the boundary being executed, or
    /// `None` if the block faulted before its first [`Opcode::INSN_START`].
    pub mark: Option<u32>,
    /// The guest PC of that boundary — the block's entry PC when there is
    /// none, which is the same answer.
    pub pc: u64,
    /// Ticks retired at that boundary, counted from block entry.
    ///
    /// The architectural count: what the guest's cycle counter must read if
    /// the faulting instruction is treated as not having started.
    ///
    /// **Measured, not read off the boundary.** [`InsnStart::ticks`] is the
    /// *static* column — the charges a frontend could know at lift time — and
    /// a block that has already performed a data-dependent access (a
    /// misaligned load splitting into bytes, a page-table walk) has spent more
    /// than that. This is the number actually charged when the boundary was
    /// reached, so it stays exact however many accesses a superblock made
    /// before the faulting instruction.
    pub retired_ticks: u64,
    /// Ticks actually charged to the host when the access faulted.
    ///
    /// At or after [`Fault::retired_ticks`], and the difference is exactly the
    /// charges the faulting instruction had already made. A core that restarts
    /// a faulting instruction must reconcile the two; one that resumes it need
    /// not. `ROADMAP.md` §9 makes restart-versus-resume a per-architecture
    /// policy, so this backend reports both numbers and decides neither.
    pub charged_ticks: u64,
    /// Whether nothing has been committed since the boundary.
    ///
    /// [`BusError::Retry`] is only legal while this holds — a retry that
    /// re-runs a half-completed guest instruction is a correctness bug — so a
    /// host that answers `Retry` once it is false gets that answer rejected
    /// rather than delivered.
    pub restartable: bool,
}

/// The portable IR interpreter.
///
/// Holds the temporary file across a run so a differential harness can compare
/// it against a host backend's registers, and reuses its allocations across
/// blocks — a translator that allocates per block pays for it on every block.
#[derive(Debug, Default)]
pub struct Interp {
    temps: Vec<u128>,
    args: Vec<u128>,
    ticks: u64,
    mark: Option<u32>,
    /// Whether [`Interp::mark`]'s live mapping has been written out yet.
    ///
    /// The whole of the lazy-publication scheme (module docs): `false` means
    /// the host's slot storage is behind the temporaries.
    published: bool,
    boundaries: u64,
    boundary_pc: u64,
    retired: u64,
    committed: bool,
}

/// What executing one instruction decided about the next.
enum Step {
    Next,
    Jump(usize),
    Done(Outcome),
}

impl Interp {
    /// A budget on executed instructions, so a malformed backward
    /// [`Opcode::BRCOND`] fails a fuzz case instead of hanging it.
    ///
    /// Generous: real blocks are tens of instructions and the branches within
    /// one are forward, so nothing well-formed comes near this.
    const STEP_LIMIT: u64 = 1 << 24;

    /// A fresh interpreter.
    #[must_use]
    pub fn new() -> Interp {
        Interp::default()
    }

    /// Execute `block` against `host`, and say why it stopped.
    ///
    /// The block is expected to have passed [`verify`](crate::ir::verify)
    /// first; it is not required to, and an unverified block that is malformed
    /// yields [`Error::Ir`] rather than a panic, because the fuzz targets run
    /// this path.
    ///
    /// # Errors
    ///
    /// [`Error::Ir`] for a malformed block, whatever a helper returned, and
    /// [`Error::Bus`] carrying [`BusError::Retry`] when a host asks to retry an
    /// access that can no longer be retried (see [`Fault::restartable`]).
    pub fn run<H: IrHost + ?Sized>(&mut self, block: &Block, host: &mut H) -> Result<Outcome> {
        self.temps.clear();
        self.temps.resize(block.temp_count(), 0);
        self.args.clear();
        self.ticks = 0;
        self.mark = None;
        self.published = true;
        self.boundaries = 0;
        self.boundary_pc = block.entry_pc;
        self.retired = 0;
        self.committed = false;

        let outcome = self.execute(block, host);
        // Whatever happened — an exit, a fault, a malformed block — the guest's
        // architectural state is materialized before the caller can look at it.
        // One place rather than eight, because a path that forgot would leave
        // registers in temporaries nobody can reach (module docs).
        self.publish(block, host);
        outcome
    }

    fn execute<H: IrHost + ?Sized>(&mut self, block: &Block, host: &mut H) -> Result<Outcome> {
        let insts = block.insts();
        let mut at = 0usize;
        let mut steps = 0u64;
        while let Some(inst) = insts.get(at) {
            steps += 1;
            if steps > Self::STEP_LIMIT {
                return Err(Error::Ir(format!(
                    "block {:#x} did not terminate within {} instructions",
                    block.entry_pc,
                    Self::STEP_LIMIT
                )));
            }
            match self.step(block, inst, at, host)? {
                Step::Next => at += 1,
                Step::Jump(target) => {
                    if target >= insts.len() {
                        return Err(ir_err(
                            at,
                            inst.op,
                            "the branch target is outside the block",
                        ));
                    }
                    at = target;
                }
                Step::Done(outcome) => return Ok(outcome),
            }
        }
        Err(Error::Ir(format!(
            "block {:#x} ran off the end without reaching a terminator",
            block.entry_pc
        )))
    }

    /// Ticks charged during the run so far.
    ///
    /// The number a differential test compares against the host backend's, and
    /// against the boundary [`Interp::mark`] names — its [`InsnStart::ticks`]
    /// is the *static* column, and the difference is exactly what the block's
    /// accesses spent.
    #[inline]
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.ticks
    }

    /// The value a temporary holds, masked to its type.
    #[inline]
    #[must_use]
    pub fn temp_value(&self, temp: Temp) -> Option<u128> {
        self.temps.get(temp.index()).copied()
    }

    /// How many [`Opcode::INSN_START`] boundaries the run passed.
    ///
    /// The **dynamic** instruction count, and the only honest one once a block
    /// has more than one exit: a superblock's static instruction count says how
    /// many guest instructions it *covers*, and a run that leaves through a
    /// side exit retires fewer than that. Every frontend in the tree closes a
    /// block with one boundary that begins no instruction — the exit boundary
    /// carrying the outgoing register map — and exactly one of those is reached
    /// on any path, so **`boundaries() - 1` is the number of guest instructions
    /// that retired**. At a fault the same expression is still right, and for
    /// the same reason: the faulting instruction opened its boundary and did
    /// not retire.
    #[inline]
    #[must_use]
    pub fn boundaries(&self) -> u64 {
        self.boundaries
    }

    /// The index into [`Block::marks`](crate::ir::Block::marks) of the boundary
    /// the run last reached, or `None` if it reached none.
    ///
    /// What a caller reads the *static* tick column off, which is the column
    /// a differential harness cross-checks against the ticks actually charged.
    #[inline]
    #[must_use]
    pub fn mark(&self) -> Option<u32> {
        self.mark
    }

    /// Materialize the pending boundary's live mapping into guest state.
    ///
    /// Idempotent, and a no-op when there is nothing pending, so every caller
    /// can be unconditional. See the module docs for where it is called from
    /// and why nowhere else needs it.
    fn publish<H: IrHost + ?Sized>(&mut self, block: &Block, host: &mut H) {
        if self.published {
            return;
        }
        self.published = true;
        let Some(mark) = self.mark.and_then(|m| block.marks().get(m as usize)) else {
            return;
        };
        for &(slot, temp) in &mark.live {
            // A temporary the block never allocated is a malformed block, which
            // the verifier rejects; publishing nothing for it is better than
            // failing here, because this runs on the way out of an error.
            if let Some(value) = self.temps.get(temp.index()).copied() {
                host.write_slot(slot, value);
            }
        }
    }

    /// Whether the pending boundary binds `slot` to a temporary.
    ///
    /// The guard on [`Opcode::GET_SLOT`]: a read of a slot a temporary shadows
    /// has to see the temporary. A linear scan over a mapping that is at most a
    /// register file wide, and one the first frontend never triggers — it emits
    /// a slot read only for a register nothing has bound.
    fn shadowed(&self, block: &Block, slot: RegSlot) -> bool {
        !self.published
            && self
                .mark
                .and_then(|m| block.marks().get(m as usize))
                .is_some_and(|mark| mark.live.iter().any(|&(s, _)| s == slot))
    }

    #[inline]
    fn get(&self, temp: Temp, at: usize, op: Opcode) -> Result<u128> {
        self.temps
            .get(temp.index())
            .copied()
            .ok_or_else(|| ir_err(at, op, "an operand was never allocated in this block"))
    }

    #[inline]
    fn set(&mut self, block: &Block, temp: Temp, value: u128, at: usize, op: Opcode) -> Result<()> {
        let ty = block
            .type_of(temp)
            .ok_or_else(|| ir_err(at, op, "a result was never allocated in this block"))?;
        let slot = self
            .temps
            .get_mut(temp.index())
            .ok_or_else(|| ir_err(at, op, "a result was never allocated in this block"))?;
        *slot = value & mask_bits(ty.bits());
        Ok(())
    }

    /// Report a bus fault, or reject it if it is a retry that cannot be one.
    fn fault(&self, at: usize, error: BusError) -> Result<Step> {
        if error == BusError::Retry && self.committed {
            // The guest instruction has already changed something the world can
            // see, so there is nothing left to restart from. Rejected here
            // rather than passed on, because a consumer that acted on it would
            // re-run the committed half.
            return Err(Error::Bus(BusError::Retry));
        }
        Ok(Step::Done(Outcome::Fault(Fault {
            error,
            at,
            mark: self.mark,
            pc: self.boundary_pc,
            retired_ticks: self.retired,
            charged_ticks: self.ticks,
            restartable: !self.committed,
        })))
    }

    fn step<H: IrHost + ?Sized>(
        &mut self,
        block: &Block,
        inst: &Inst,
        at: usize,
        host: &mut H,
    ) -> Result<Step> {
        let op = inst.op;
        let ty = inst.ty;
        let srcs = block.srcs(at);
        let w = ty.bits();
        let m = mask_bits(w);

        // Fetching a source is common enough to deserve a name; the arity check
        // is what turns a malformed block into a message instead of a panic.
        let src = |i: usize| -> Result<Temp> {
            srcs.get(i)
                .copied()
                .ok_or_else(|| ir_err(at, op, "too few source operands"))
        };

        match op {
            // ---- Data movement -------------------------------------------
            Opcode::MOV => {
                let value = match (srcs.first(), inst.imm) {
                    (Some(&s), _) => self.get(s, at, op)?,
                    (None, Some(c)) => c.bits(),
                    (None, None) => {
                        return Err(ir_err(at, op, "a mov needs a source or an immediate"));
                    }
                };
                self.write(block, inst, value, at)?;
            }
            Opcode::GET_SLOT => {
                // The slot rides in `aux`; the verifier has already rejected a
                // slot read that carries operands.
                // `set` canonicalises to the temporary's width, so a host
                // that hands back a wider value cannot smuggle bits in.
                let slot = RegSlot(inst.aux as u16);
                if self.shadowed(block, slot) {
                    // Guest state is published lazily, so a slot the current
                    // boundary binds is stale in the host until it is written
                    // out. Reading it without this would hand back the value
                    // from before the temporary took over.
                    self.publish(block, host);
                }
                let value = host.read_slot(slot);
                self.write(block, inst, value, at)?;
            }
            Opcode::EXT_S => {
                let s = src(0)?;
                let from = block.type_of(s).ok_or_else(|| {
                    ir_err(at, op, "the source was never allocated in this block")
                })?;
                let value = sext(self.get(s, at, op)?, from.bits()) as u128;
                self.write(block, inst, value, at)?;
            }
            Opcode::EXT_Z | Opcode::TRUNC => {
                // Both are a mask to the destination's width: values are held
                // canonically masked, so a zero-extend has nothing to add and a
                // truncation has only to drop what the wider type carried.
                let value = self.get(src(0)?, at, op)?;
                self.write(block, inst, value, at)?;
            }
            Opcode::BSWAP => {
                let lane = match inst.imm {
                    Some(c) => u32::try_from(c.bits())
                        .map_err(|_| ir_err(at, op, "the lane width is absurd"))?,
                    None => w,
                };
                if lane < 8 || !lane.is_multiple_of(8) || !w.is_multiple_of(lane) {
                    return Err(ir_err(
                        at,
                        op,
                        "the lane width must be whole bytes and divide the type",
                    ));
                }
                let value = bswap(self.get(src(0)?, at, op)?, w, lane);
                self.write(block, inst, value, at)?;
            }
            Opcode::DEPOSIT => {
                let (pos, len) = bitfield_parts(inst.aux);
                let field = field_mask(pos, len, w)
                    .ok_or_else(|| ir_err(at, op, "the bitfield does not fit within the type"))?;
                let into = self.get(src(0)?, at, op)?;
                let what = self.get(src(1)?, at, op)?;
                let value = (into & !field) | ((what << pos) & field);
                self.write(block, inst, value, at)?;
            }
            Opcode::EXTRACT => {
                // Unsigned. A signed field is this followed by an `ext_s`,
                // which is one op rather than a second opcode.
                let (pos, len) = bitfield_parts(inst.aux);
                let field = field_mask(pos, len, w)
                    .ok_or_else(|| ir_err(at, op, "the bitfield does not fit within the type"))?;
                let value = (self.get(src(0)?, at, op)? & field) >> pos;
                self.write(block, inst, value, at)?;
            }

            // ---- Arithmetic ----------------------------------------------
            Opcode::ADD | Opcode::SUB | Opcode::MUL | Opcode::NEG => {
                require_int(ty, at, op)?;
                let a = self.get(src(0)?, at, op)?;
                // Wrapping, deliberately (CLAUDE.md, "Arithmetic"): guest
                // arithmetic wraps by definition, and the mask makes the wrap
                // happen at the *guest's* width rather than at u128's.
                let value = match op {
                    Opcode::NEG => 0u128.wrapping_sub(a),
                    _ => {
                        let b = self.get(src(1)?, at, op)?;
                        match op {
                            Opcode::ADD => a.wrapping_add(b),
                            Opcode::SUB => a.wrapping_sub(b),
                            _ => a.wrapping_mul(b),
                        }
                    }
                } & m;
                self.write(block, inst, value, at)?;
            }
            Opcode::DIV_S | Opcode::DIV_U | Opcode::REM_S | Opcode::REM_U => {
                require_int(ty, at, op)?;
                let a = self.get(src(0)?, at, op)?;
                let b = self.get(src(1)?, at, op)?;
                if b == 0 {
                    // Every guest defines this differently and none of them the
                    // way the host does, which is why the op's own docs say a
                    // frontend guards it. Reaching here is that missing guard.
                    return Err(ir_err(at, op, "divide by zero; the frontend owes a guard"));
                }
                let value = match op {
                    // `wrapping_div` rather than `/`, so the one signed overflow
                    // case (MIN / -1) yields MIN instead of panicking in a debug
                    // build and wrapping in a release one.
                    Opcode::DIV_S => sext(a, w).wrapping_div(sext(b, w)) as u128,
                    Opcode::REM_S => sext(a, w).wrapping_rem(sext(b, w)) as u128,
                    Opcode::DIV_U => a / b,
                    _ => a % b,
                } & m;
                self.write(block, inst, value, at)?;
            }
            Opcode::ADDC | Opcode::SUBB => {
                require_int(ty, at, op)?;
                let a = self.get(src(0)?, at, op)?;
                let b = self.get(src(1)?, at, op)?;
                let c = self.get(src(2)?, at, op)? & 1;
                let (value, out) = if op == Opcode::ADDC {
                    let (s1, o1) = a.overflowing_add(b);
                    let (s2, o2) = s1.overflowing_add(c);
                    // Below 128 bits the carry is the bit that fell off the top
                    // of the guest's width; at 128 it is u128's own overflow.
                    let out = if w >= 128 {
                        o1 || o2
                    } else {
                        (s2 >> w) & 1 == 1
                    };
                    (s2 & m, out)
                } else {
                    // Borrow convention, as the op documents: a one in means one
                    // was owed. ARM and the 6502 carry the complement, and their
                    // frontends invert.
                    let (t, o) = b.overflowing_add(c);
                    (a.wrapping_sub(b).wrapping_sub(c) & m, o || a < t)
                };
                self.write(block, inst, value, at)?;
                let carry = inst
                    .dst2
                    .ok_or_else(|| ir_err(at, op, "a carry op must produce its carry out"))?;
                self.set(block, carry, u128::from(out), at, op)?;
            }
            Opcode::MULU2 | Opcode::MULS2 | Opcode::MULHSU => {
                require_int(ty, at, op)?;
                if w > 64 {
                    // A 128-bit widening multiply needs a 256-bit product and no
                    // guest asks for one; when one does it is a helper call, not
                    // a wider accumulator here.
                    return Ok(Step::Done(Outcome::Unsupported { op, at }));
                }
                let a = self.get(src(0)?, at, op)?;
                let b = self.get(src(1)?, at, op)?;
                let product = match op {
                    Opcode::MULU2 => a.wrapping_mul(b),
                    Opcode::MULS2 => sext(a, w).wrapping_mul(sext(b, w)) as u128,
                    // Signed by unsigned: RISC-V's `mulhsu`, which neither of
                    // the others expresses.
                    _ => sext(a, w).wrapping_mul(b as i128) as u128,
                };
                let high = (product >> w) & m;
                if op == Opcode::MULHSU {
                    self.write(block, inst, high, at)?;
                } else {
                    self.write(block, inst, product & m, at)?;
                    let hi = inst.dst2.ok_or_else(|| {
                        ir_err(at, op, "a widening multiply must produce its high half")
                    })?;
                    self.set(block, hi, high, at, op)?;
                }
            }

            // ---- Logic and shifts ----------------------------------------
            Opcode::AND | Opcode::OR | Opcode::XOR | Opcode::ANDC | Opcode::NOT => {
                let a = self.get(src(0)?, at, op)?;
                let value = match op {
                    Opcode::NOT => !a,
                    _ => {
                        let b = self.get(src(1)?, at, op)?;
                        match op {
                            Opcode::AND => a & b,
                            Opcode::OR => a | b,
                            Opcode::XOR => a ^ b,
                            _ => a & !b,
                        }
                    }
                } & m;
                self.write(block, inst, value, at)?;
            }
            Opcode::SHL | Opcode::SHR | Opcode::SAR => {
                require_int(ty, at, op)?;
                let a = self.get(src(0)?, at, op)?;
                let amount = self.get(src(1)?, at, op)?;
                // Out of range is undefined in the IR; this backend takes the
                // mathematical answer. See the module docs for why not x86's.
                let value = if amount >= u128::from(w) {
                    match op {
                        Opcode::SAR if sext(a, w) < 0 => m,
                        _ => 0,
                    }
                } else {
                    let n = amount as u32;
                    match op {
                        Opcode::SHL => a << n,
                        Opcode::SHR => a >> n,
                        _ => (sext(a, w) >> n) as u128,
                    }
                } & m;
                self.write(block, inst, value, at)?;
            }
            Opcode::ROTL | Opcode::ROTR => {
                require_int(ty, at, op)?;
                let a = self.get(src(0)?, at, op)?;
                // A rotate is defined for every amount, so unlike the shifts
                // this reduces rather than saturating.
                let n = (self.get(src(1)?, at, op)? % u128::from(w)) as u32;
                let n = if op == Opcode::ROTL { n } else { (w - n) % w };
                let value = if n == 0 {
                    a
                } else {
                    ((a << n) | (a >> (w - n))) & m
                };
                self.write(block, inst, value, at)?;
            }
            Opcode::ROTLC | Opcode::ROTRC => {
                require_int(ty, at, op)?;
                let a = self.get(src(0)?, at, op)?;
                let c = self.get(src(1)?, at, op)? & 1;
                // An (N+1)-bit rotate by one: the carry is the extra bit.
                let (value, out) = if op == Opcode::ROTLC {
                    (((a << 1) | c) & m, (a >> (w - 1)) & 1)
                } else {
                    (((a >> 1) | (c << (w - 1))) & m, a & 1)
                };
                self.write(block, inst, value, at)?;
                let carry = inst
                    .dst2
                    .ok_or_else(|| ir_err(at, op, "a carry op must produce its carry out"))?;
                self.set(block, carry, out, at, op)?;
            }

            // ---- Bit counting --------------------------------------------
            Opcode::CLZ | Opcode::CTZ | Opcode::POPCOUNT => {
                require_int(ty, at, op)?;
                let a = self.get(src(0)?, at, op)?;
                let value = u128::from(match op {
                    // Counted within the guest's width, not u128's: a value is
                    // held masked, so the leading zeros above the width are ours
                    // and have to come off.
                    //
                    // **Saturating, and that is a fix rather than a nicety.**
                    // The IR does not oblige an operand to carry the
                    // instruction's type, so `clz.i32` over an `i64` temporary
                    // holding a value above `2^32` sees fewer than `128 - 32`
                    // leading zeros and this subtraction used to underflow —
                    // a debug-build panic, on a block `verify` accepted.
                    // `verify` now rejects that shape (its `CLZ` arm says so),
                    // which makes this unreachable through the supported path;
                    // it saturates anyway, because `Interp` is reachable
                    // without the verifier and a panic in the oracle is worse
                    // than a wrong answer on a block nothing should have built.
                    Opcode::CLZ => a.leading_zeros().saturating_sub(128 - w),
                    Opcode::CTZ => a.trailing_zeros().min(w),
                    _ => a.count_ones(),
                });
                self.write(block, inst, value, at)?;
            }

            // ---- Compare and branch ---------------------------------------
            Opcode::SETCOND => {
                let cond = inst
                    .cond
                    .ok_or_else(|| ir_err(at, op, "a comparison needs a condition"))?;
                let a = self.get(src(0)?, at, op)?;
                let b = self.get(src(1)?, at, op)?;
                self.write(block, inst, u128::from(compare(cond, w, a, b)), at)?;
            }
            Opcode::MOVCOND => {
                // Two shapes, because both are natural and neither is more
                // correct: select on a one-bit temporary, or compare a pair and
                // select — which is what an ISA's conditional move actually is.
                let (taken, t, f) = match (inst.cond, srcs.len()) {
                    (Some(cond), 4) => {
                        let a = self.get(src(0)?, at, op)?;
                        let b = self.get(src(1)?, at, op)?;
                        (compare(cond, w, a, b), src(2)?, src(3)?)
                    }
                    (_, 3) => (self.get(src(0)?, at, op)? & 1 == 1, src(1)?, src(2)?),
                    _ => {
                        return Err(ir_err(
                            at,
                            op,
                            "a movcond takes a selector and two values, or a condition and four",
                        ));
                    }
                };
                let value = self.get(if taken { t } else { f }, at, op)?;
                self.write(block, inst, value, at)?;
            }
            Opcode::BRCOND => {
                let taken = match (inst.cond, srcs.len()) {
                    (Some(cond), 2) => {
                        let a = self.get(src(0)?, at, op)?;
                        let b = self.get(src(1)?, at, op)?;
                        compare(cond, w, a, b)
                    }
                    (_, 1) => self.get(src(0)?, at, op)? & 1 == 1,
                    _ => {
                        return Err(ir_err(
                            at,
                            op,
                            "a brcond takes a selector, or a condition and two values",
                        ));
                    }
                };
                // The target is an instruction index within this block — the
                // "label" the `aux` field's documentation names.
                if taken {
                    return Ok(Step::Jump(inst.aux as usize));
                }
            }

            // ---- Memory ---------------------------------------------------
            Opcode::LD => {
                let mem = inst
                    .mem
                    .ok_or_else(|| ir_err(at, op, "a memory op needs a MemOp descriptor"))?;
                let addr = self.get(src(0)?, at, op)? as u64;
                // A volatile load is a bus cycle whose occurrence the guest can
                // observe even when its value is discarded, so it commits.
                if mem.volatile {
                    self.committed = true;
                }
                let raw = match host.load(&mem, addr) {
                    Ok(v) => v,
                    Err(e) => return self.fault(at, e),
                };
                let bits = mem.size.bits();
                let value = match mem.sign {
                    Sign::Unsigned => u128::from(raw) & mask_bits(bits),
                    Sign::Signed => sext(u128::from(raw) & mask_bits(bits), bits) as u128,
                };
                self.write(block, inst, value, at)?;
            }
            Opcode::ST => {
                let mem = inst
                    .mem
                    .ok_or_else(|| ir_err(at, op, "a memory op needs a MemOp descriptor"))?;
                let addr = self.get(src(0)?, at, op)? as u64;
                let value = (self.get(src(1)?, at, op)? as u64) & mem.size.mask();
                self.committed = true;
                if let Err(e) = host.store(&mem, addr, value) {
                    return self.fault(at, e);
                }
            }

            // ---- Atomics --------------------------------------------------
            Opcode::FENCE => {
                self.committed = true;
                host.fence();
            }
            Opcode::CMPXCHG | Opcode::XCHG | Opcode::LD_EXCL | Opcode::ST_EXCL => {
                return self.atomic(block, inst, at, host);
            }
            other if is_fetch(other) => {
                return self.atomic(block, inst, at, host);
            }

            // ---- Control and side effects ---------------------------------
            Opcode::GOTO_TB => {
                let pc = inst
                    .imm
                    .ok_or_else(|| ir_err(at, op, "a goto_tb needs its successor's PC"))?;
                return Ok(Step::Done(Outcome::Goto {
                    pc: pc.bits() as u64,
                }));
            }
            Opcode::EXIT_TB => return Ok(Step::Done(Outcome::Exit)),
            Opcode::LOOKUP_AND_GOTO => {
                let pc = self.get(src(0)?, at, op)? as u64;
                return Ok(Step::Done(Outcome::Lookup { pc }));
            }
            Opcode::CALL_HELPER => {
                self.args.clear();
                for i in 0..srcs.len() {
                    let v = self.get(src(i)?, at, op)?;
                    self.args.push(v);
                }
                self.committed = true;
                // A helper is arbitrary Rust and may read the guest's registers
                // — a mode change is a helper call and a barrier for exactly
                // this reason (the `ir` module docs, decision 4) — so the
                // pending boundary is written out before it runs.
                self.publish(block, host);
                let (first, second) = host.call_helper(inst.aux, &self.args)?;
                if let Some(dst) = inst.dst {
                    self.set(block, dst, first, at, op)?;
                }
                if let Some(dst2) = inst.dst2 {
                    self.set(block, dst2, second, at, op)?;
                }
            }
            Opcode::CHARGE => {
                let ticks = inst
                    .imm
                    .ok_or_else(|| ir_err(at, op, "a charge needs a tick count"))?
                    .bits() as u64;
                // Exactly, where it was written: the count is hashed output.
                self.ticks = self.ticks.wrapping_add(ticks);
                self.committed = true;
                host.charge(ticks);
            }
            Opcode::INSN_START => {
                let mark = block
                    .marks()
                    .get(inst.aux as usize)
                    .ok_or_else(|| ir_err(at, op, "the boundary marker points at no record"))?;
                // The mapping is *remembered*, not written out: the previous
                // boundary's is superseded here, so publishing it would be work
                // thrown away, and the new one is only needed if something can
                // observe guest state before the next boundary (module docs).
                // Every live temporary named here is already assigned — the
                // verifier checks that — so nothing is lost by deferring.
                self.mark = Some(inst.aux);
                self.published = false;
                self.boundaries = self.boundaries.wrapping_add(1);
                self.boundary_pc = mark.pc;
                // The charged count rather than `mark.ticks`, which is the
                // static column and undercounts once an access in this block
                // has spent a data-dependent tick.
                self.retired = self.ticks;
                // Restart granularity is the guest instruction, so the previous
                // one's commits stop blocking a retry here.
                self.committed = false;
                host.insn_start(mark);
            }

            _ => return Ok(Step::Done(Outcome::Unsupported { op, at })),
        }
        Ok(Step::Next)
    }

    /// The atomic family, which shares an address, an argument and a result.
    fn atomic<H: IrHost + ?Sized>(
        &mut self,
        block: &Block,
        inst: &Inst,
        at: usize,
        host: &mut H,
    ) -> Result<Step> {
        let op = inst.op;
        let srcs = block.srcs(at);
        let width = match inst.ty {
            Type::I32 => Width::U32,
            Type::I64 => Width::U64,
            // The IR has no i8 or i16, so a byte-wide `lock xadd` has no type to
            // name here. It is a helper call until the IR grows one.
            _ => return Err(ir_err(at, op, "an atomic must be i32 or i64")),
        };
        let mut mem = MemOp::load(width);
        mem.kind = if op == Opcode::LD_EXCL {
            AccessKind::Load
        } else {
            AccessKind::Store
        };
        // An atomic is never eliminable, whatever its value is used for.
        mem.volatile = true;

        let fetch = |i: usize| -> Result<Temp> {
            srcs.get(i)
                .copied()
                .ok_or_else(|| ir_err(at, op, "too few source operands"))
        };
        let addr = self.get(fetch(0)?, at, op)? as u64;
        let (arg, compare_with) = match op {
            Opcode::LD_EXCL => (0, 0),
            Opcode::CMPXCHG => (
                self.get(fetch(2)?, at, op)? as u64,
                self.get(fetch(1)?, at, op)? as u64,
            ),
            _ => (self.get(fetch(1)?, at, op)? as u64, 0),
        };

        self.committed = true;
        let previous = match host.rmw(op, &mem, addr, arg, compare_with) {
            Ok(v) => v,
            Err(e) => return self.fault(at, e),
        };
        if let Some(dst) = inst.dst {
            self.set(block, dst, u128::from(previous), at, op)?;
        }
        if op == Opcode::CMPXCHG
            && let Some(dst2) = inst.dst2
        {
            // The second result is "did it swap", which the caller cannot always
            // derive: a location that already held the desired value returns the
            // same previous value either way.
            self.set(block, dst2, u128::from(previous == compare_with), at, op)?;
        }
        Ok(Step::Next)
    }

    /// Write an instruction's primary result, where it has one.
    #[inline]
    fn write(&mut self, block: &Block, inst: &Inst, value: u128, at: usize) -> Result<()> {
        match inst.dst {
            Some(dst) => self.set(block, dst, value, at, inst.op),
            None => Err(ir_err(at, inst.op, "this op must have a destination")),
        }
    }
}

/// A mask with `bits` low bits set, saturating at 128.
#[inline]
const fn mask_bits(bits: u32) -> u128 {
    if bits >= 128 {
        u128::MAX
    } else {
        (1u128 << bits) - 1
    }
}

/// Sign-extend the low `bits` of `value` to a full `i128`.
#[inline]
const fn sext(value: u128, bits: u32) -> i128 {
    if bits >= 128 {
        value as i128
    } else {
        let shift = 128 - bits;
        ((value << shift) as i128) >> shift
    }
}

/// The mask of a `len`-bit field at `pos`, or `None` if it leaves the type.
#[inline]
const fn field_mask(pos: u32, len: u32, width: u32) -> Option<u128> {
    if len == 0 || pos + len > width {
        return None;
    }
    Some(mask_bits(len) << pos)
}

/// Reverse the byte order within each `lane`-bit lane of a `width`-bit value.
fn bswap(value: u128, width: u32, lane: u32) -> u128 {
    let bytes = lane / 8;
    let lane_mask = mask_bits(lane);
    let mut out = 0u128;
    let mut base = 0;
    while base < width {
        let piece = (value >> base) & lane_mask;
        let mut swapped = 0u128;
        for i in 0..bytes {
            let byte = (piece >> (8 * i)) & 0xff;
            swapped |= byte << (8 * (bytes - 1 - i));
        }
        out |= swapped << base;
        base += lane;
    }
    out
}

/// Evaluate a comparison at `width` bits.
fn compare(cond: Cond, width: u32, a: u128, b: u128) -> bool {
    match cond {
        Cond::Eq => a == b,
        Cond::Ne => a != b,
        Cond::LtU => a < b,
        Cond::LeU => a <= b,
        Cond::GtU => a > b,
        Cond::GeU => a >= b,
        Cond::LtS => sext(a, width) < sext(b, width),
        Cond::LeS => sext(a, width) <= sext(b, width),
        Cond::GtS => sext(a, width) > sext(b, width),
        Cond::GeS => sext(a, width) >= sext(b, width),
    }
}

/// Whether an opcode is one of the atomic read-modify-writes.
#[inline]
fn is_fetch(op: Opcode) -> bool {
    op.0 >= Opcode::FETCH_ADD.0 && op.0 <= Opcode::FETCH_UMAX.0
}

/// Reject a non-integer type where arithmetic is being asked for.
///
/// The IR never performs float arithmetic — tier-1 floating point is a helper
/// call into soft-float — so an `f64` reaching an `add` is a frontend bug and
/// not a missing feature.
#[inline]
fn require_int(ty: Type, at: usize, op: Opcode) -> Result<()> {
    if ty.is_int() {
        Ok(())
    } else {
        Err(ir_err(at, op, "arithmetic on a non-integer type"))
    }
}

/// An error naming the instruction, in the shape the verifier uses.
fn ir_err(at: usize, op: Opcode, what: &str) -> Error {
    Error::Ir(format!("instruction {at} ({op}): {what}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ir::block::{BlockBuilder, InsnStart};
    use crate::ir::op::bitfield_aux;
    use crate::ir::types::Const;
    use crate::ir::verify::verify;
    use alloc::collections::BTreeMap;
    use alloc::vec;
    use alloc::vec::Vec;

    /// A host over a sparse slot space and a sparse byte map.
    ///
    /// Sparse on purpose: the interpreter must never assume a slot number is
    /// small or an address is inside anything, because the first belongs to the
    /// frontend and the second to the machine.
    #[derive(Debug, Default)]
    struct Host {
        slots: BTreeMap<u16, u128>,
        mem: BTreeMap<u64, u8>,
        ticks: u64,
        boundaries: Vec<(u64, u64, u64)>,
        published: Vec<(u16, u128)>,
        faults: BTreeMap<u64, BusError>,
        helpers: Vec<(u32, Vec<u128>)>,
        fences: u32,
        reserved: Option<u64>,
    }

    impl Host {
        fn peek(&self, addr: u64, width: Width) -> u64 {
            let mut v = 0u64;
            for i in 0..width.bytes() {
                let byte = self.mem.get(&(addr.wrapping_add(i))).copied().unwrap_or(0);
                v |= u64::from(byte) << (8 * i);
            }
            v
        }

        fn poke(&mut self, addr: u64, width: Width, value: u64) {
            for i in 0..width.bytes() {
                self.mem
                    .insert(addr.wrapping_add(i), (value >> (8 * i)) as u8);
            }
        }
    }

    impl IrHost for Host {
        fn read_slot(&mut self, slot: RegSlot) -> u128 {
            self.slots.get(&slot.0).copied().unwrap_or(0)
        }

        fn write_slot(&mut self, slot: RegSlot, value: u128) {
            self.slots.insert(slot.0, value);
            self.published.push((slot.0, value));
        }

        fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
            match self.faults.get(&addr) {
                Some(e) => Err(*e),
                None => Ok(self.peek(addr, mem.size)),
            }
        }

        fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
            match self.faults.get(&addr) {
                Some(e) => Err(*e),
                None => {
                    self.poke(addr, mem.size, value);
                    Ok(())
                }
            }
        }

        fn charge(&mut self, ticks: u64) {
            self.ticks += ticks;
        }

        fn insn_start(&mut self, mark: &InsnStart) {
            self.boundaries.push((mark.pc, mark.next_pc, mark.ticks));
        }

        fn rmw(
            &mut self,
            op: Opcode,
            mem: &MemOp,
            addr: u64,
            arg: u64,
            compare: u64,
        ) -> MemResult<u64> {
            if let Some(e) = self.faults.get(&addr) {
                return Err(*e);
            }
            let bits = mem.size.bits();
            let previous = self.peek(addr, mem.size);
            let next = match op {
                Opcode::CMPXCHG => {
                    if previous == compare {
                        arg
                    } else {
                        previous
                    }
                }
                Opcode::XCHG => arg,
                Opcode::FETCH_ADD => previous.wrapping_add(arg),
                Opcode::FETCH_AND => previous & arg,
                Opcode::FETCH_OR => previous | arg,
                Opcode::FETCH_XOR => previous ^ arg,
                Opcode::FETCH_UMIN => previous.min(arg),
                Opcode::FETCH_UMAX => previous.max(arg),
                Opcode::FETCH_SMIN | Opcode::FETCH_SMAX => {
                    let (p, a) = (
                        sext(u128::from(previous), bits),
                        sext(u128::from(arg), bits),
                    );
                    let take_arg = if op == Opcode::FETCH_SMIN {
                        a < p
                    } else {
                        a > p
                    };
                    if take_arg { arg } else { previous }
                }
                Opcode::LD_EXCL => {
                    self.reserved = Some(addr);
                    return Ok(previous);
                }
                Opcode::ST_EXCL => {
                    let held = self.reserved == Some(addr);
                    if held {
                        self.poke(addr, mem.size, arg);
                        self.reserved = None;
                    }
                    return Ok(u64::from(held));
                }
                _ => return Err(BusError::BadAccess),
            };
            self.poke(addr, mem.size, next);
            Ok(previous)
        }

        fn fence(&mut self) {
            self.fences += 1;
        }

        fn call_helper(&mut self, id: u32, args: &[u128]) -> Result<(u128, u128)> {
            self.helpers.push((id, args.to_vec()));
            // A soft-float shape: a value and its flags. Deterministic, so the
            // test can assert both results reached their temporaries.
            let sum = args.iter().copied().fold(0u128, u128::wrapping_add);
            Ok((sum, args.len() as u128))
        }
    }

    fn mark(pc: u64, ticks: u64) -> InsnStart {
        InsnStart {
            pc,
            next_pc: pc + 4,
            ticks,
            live: Vec::new(),
        }
    }

    /// A builder with its first guest instruction already open, which is what
    /// the verifier requires before anything may charge.
    fn started() -> BlockBuilder {
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0));
        b
    }

    /// Verify, then run. Every block in this module goes through the verifier
    /// first: a backend that needs something the verifier rejects has invented
    /// a dialect, and no host backend would be handed the same block.
    fn try_run(b: BlockBuilder, host: &mut Host) -> (Interp, Result<Outcome>) {
        let block = b.finish();
        verify(&block).expect("the verifier accepts what these tests emit");
        let mut interp = Interp::new();
        let outcome = interp.run(&block, host);
        (interp, outcome)
    }

    fn run(b: BlockBuilder, host: &mut Host) -> (Interp, Outcome) {
        let (interp, outcome) = try_run(b, host);
        let outcome = outcome.expect("the block runs to a stopping point");
        (interp, outcome)
    }

    #[test]
    fn arithmetic_wraps_at_the_guest_width_not_the_hosts() {
        let mut b = started();
        let big = b.imm(Type::I32, Const::Int(0xffff_ffff));
        let one = b.imm(Type::I32, Const::Int(1));
        let sum = b.binary(Opcode::ADD, Type::I32, big, one);
        let diff = b.binary(Opcode::SUB, Type::I32, one, big);
        let product = b.binary(Opcode::MUL, Type::I32, big, big);
        let negated = b.unary(Opcode::NEG, Type::I32, one);
        b.exit_tb();

        let (i, outcome) = run(b, &mut Host::default());
        assert_eq!(outcome, Outcome::Exit);
        assert_eq!(i.temp_value(sum), Some(0));
        // 1 - 0xffffffff is 2 in 32 bits, not a negative i128.
        assert_eq!(i.temp_value(diff), Some(2));
        assert_eq!(i.temp_value(product), Some(1));
        assert_eq!(i.temp_value(negated), Some(0xffff_ffff));
    }

    #[test]
    fn logic_extension_and_truncation_stay_within_their_types() {
        let mut b = started();
        let a = b.imm(Type::I32, Const::Int(0xf0f0_ff00));
        let c = b.imm(Type::I32, Const::Int(0x0f0f_00ff));
        let and = b.binary(Opcode::AND, Type::I32, a, c);
        let or = b.binary(Opcode::OR, Type::I32, a, c);
        let xor = b.binary(Opcode::XOR, Type::I32, a, c);
        let not = b.unary(Opcode::NOT, Type::I32, a);
        // ARM's BIC, which is why `andc` survived the cull of §9's list.
        let andc = b.binary(Opcode::ANDC, Type::I32, a, c);
        let narrow = b.imm(Type::I32, Const::Int(0x8000_0001));
        let widened = b.unary(Opcode::EXT_S, Type::I64, narrow);
        let zeroed = b.unary(Opcode::EXT_Z, Type::I64, narrow);
        let cut = b.unary(Opcode::TRUNC, Type::I32, widened);
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        assert_eq!(i.temp_value(and), Some(0));
        assert_eq!(i.temp_value(or), Some(0xffff_ffff));
        assert_eq!(i.temp_value(xor), Some(0xffff_ffff));
        assert_eq!(i.temp_value(not), Some(0x0f0f_00ff));
        assert_eq!(i.temp_value(andc), Some(0xf0f0_ff00));
        assert_eq!(i.temp_value(widened), Some(0xffff_ffff_8000_0001));
        assert_eq!(i.temp_value(zeroed), Some(0x8000_0001));
        assert_eq!(i.temp_value(cut), Some(0x8000_0001));
    }

    #[test]
    fn add_and_subtract_carry_one_bit_in_and_one_bit_out() {
        let mut b = started();
        let big = b.imm(Type::I32, Const::Int(0xffff_ffff));
        let zero = b.imm(Type::I32, Const::Int(0));
        let one = b.imm(Type::I32, Const::Int(1));
        let set = b.imm(Type::I1, Const::Int(1));
        let clear = b.imm(Type::I1, Const::Int(0));
        // 0xffffffff + 0 + 1 wraps and carries out: the 6502's ADC, exactly.
        let (sum, carry) = b.addc(Opcode::ADDC, Type::I32, big, zero, set);
        // 0 - 1 - 0 borrows.
        let (diff, borrow) = b.addc(Opcode::SUBB, Type::I32, zero, one, clear);
        // 1 - 1 - 1 borrows too, and only the carry in causes it.
        let (edge, edge_borrow) = b.addc(Opcode::SUBB, Type::I32, one, one, set);
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        assert_eq!(i.temp_value(sum), Some(0));
        assert_eq!(i.temp_value(carry), Some(1));
        assert_eq!(i.temp_value(diff), Some(0xffff_ffff));
        assert_eq!(i.temp_value(borrow), Some(1));
        assert_eq!(i.temp_value(edge), Some(0xffff_ffff));
        assert_eq!(i.temp_value(edge_borrow), Some(1));
    }

    #[test]
    fn the_carry_ops_carry_at_the_widest_type_too() {
        // i128 is where "the bit above the width" and "the host's own overflow"
        // stop being the same expression, so it gets its own case.
        let mut b = started();
        let max = b.imm(Type::I128, Const::Int(u128::MAX));
        let zero = b.imm(Type::I128, Const::Int(0));
        let set = b.imm(Type::I1, Const::Int(1));
        let (sum, carry) = b.addc(Opcode::ADDC, Type::I128, max, zero, set);
        let (diff, borrow) = b.addc(Opcode::SUBB, Type::I128, zero, zero, set);
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        assert_eq!(i.temp_value(sum), Some(0));
        assert_eq!(i.temp_value(carry), Some(1));
        assert_eq!(i.temp_value(diff), Some(u128::MAX));
        assert_eq!(i.temp_value(borrow), Some(1));
    }

    #[test]
    fn the_rotates_through_carry_are_an_n_plus_one_bit_rotate() {
        let mut b = started();
        let value = b.imm(Type::I32, Const::Int(0x8000_0001));
        let set = b.imm(Type::I1, Const::Int(1));
        let clear = b.imm(Type::I1, Const::Int(0));
        let left = b.temp(Type::I32);
        let left_carry = b.temp(Type::I1);
        b.emit_raw(
            Opcode::ROTLC,
            Type::I32,
            Some(left),
            Some(left_carry),
            &[value, clear],
            None,
            None,
            0,
        );
        let right = b.temp(Type::I32);
        let right_carry = b.temp(Type::I1);
        b.emit_raw(
            Opcode::ROTRC,
            Type::I32,
            Some(right),
            Some(right_carry),
            &[value, set],
            None,
            None,
            0,
        );
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        // The top bit leaves through the carry, the carry enters at the bottom.
        assert_eq!(i.temp_value(left), Some(2));
        assert_eq!(i.temp_value(left_carry), Some(1));
        // ARM's RRX: the carry becomes bit 31, bit 0 becomes the carry.
        assert_eq!(i.temp_value(right), Some(0xc000_0000));
        assert_eq!(i.temp_value(right_carry), Some(1));
    }

    #[test]
    fn deposit_and_extract_move_a_bitfield() {
        // x86's sub-register aliasing: AH is bits 8..16 of EAX, which is what
        // makes these two ops load-bearing rather than decorative.
        let mut b = started();
        let eax = b.imm(Type::I32, Const::Int(0x1234_5678));
        let ah = b.temp(Type::I32);
        b.emit_raw(
            Opcode::EXTRACT,
            Type::I32,
            Some(ah),
            None,
            &[eax],
            None,
            None,
            bitfield_aux(8, 8),
        );
        let new_ah = b.imm(Type::I32, Const::Int(0xff));
        let updated = b.temp(Type::I32);
        b.emit_raw(
            Opcode::DEPOSIT,
            Type::I32,
            Some(updated),
            None,
            &[eax, new_ah],
            None,
            None,
            bitfield_aux(8, 8),
        );
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        assert_eq!(i.temp_value(ah), Some(0x56));
        assert_eq!(i.temp_value(updated), Some(0x1234_ff78));
    }

    #[test]
    fn the_widening_multiplies_keep_both_halves() {
        let mut b = started();
        let a = b.imm(Type::I32, Const::Int(0xffff_ffff));
        let c = b.imm(Type::I32, Const::Int(0xffff_ffff));
        let (ulo, uhi) = (b.temp(Type::I32), b.temp(Type::I32));
        b.emit_raw(
            Opcode::MULU2,
            Type::I32,
            Some(ulo),
            Some(uhi),
            &[a, c],
            None,
            None,
            0,
        );
        let (slo, shi) = (b.temp(Type::I32), b.temp(Type::I32));
        b.emit_raw(
            Opcode::MULS2,
            Type::I32,
            Some(slo),
            Some(shi),
            &[a, c],
            None,
            None,
            0,
        );
        // mulhsu: -1 as signed times 0xffffffff as unsigned.
        let hsu = b.temp(Type::I32);
        b.emit_raw(
            Opcode::MULHSU,
            Type::I32,
            Some(hsu),
            None,
            &[a, c],
            None,
            None,
            0,
        );
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        // 0xffffffff squared is 0xfffffffe00000001 unsigned.
        assert_eq!(i.temp_value(ulo), Some(1));
        assert_eq!(i.temp_value(uhi), Some(0xffff_fffe));
        // (-1) * (-1) is 1 signed, so the high half is all zeros.
        assert_eq!(i.temp_value(slo), Some(1));
        assert_eq!(i.temp_value(shi), Some(0));
        // (-1) * 0xffffffff is -0xffffffff, whose high 32 bits are 0xffffffff.
        assert_eq!(i.temp_value(hsu), Some(0xffff_ffff));
    }

    #[test]
    fn a_widening_multiply_wider_than_the_product_is_unsupported() {
        let mut b = started();
        let a = b.imm(Type::I128, Const::Int(3));
        let (lo, hi) = (b.temp(Type::I128), b.temp(Type::I128));
        b.emit_raw(
            Opcode::MULU2,
            Type::I128,
            Some(lo),
            Some(hi),
            &[a, a],
            None,
            None,
            0,
        );
        b.exit_tb();

        let (_, outcome) = run(b, &mut Host::default());
        assert_eq!(
            outcome,
            Outcome::Unsupported {
                op: Opcode::MULU2,
                at: 2
            }
        );
    }

    #[test]
    fn division_is_signed_where_it_says_it_is() {
        let mut b = started();
        let minus_seven = b.imm(Type::I32, Const::Int(0xffff_fff9));
        let two = b.imm(Type::I32, Const::Int(2));
        let qs = b.binary(Opcode::DIV_S, Type::I32, minus_seven, two);
        let rs = b.binary(Opcode::REM_S, Type::I32, minus_seven, two);
        let qu = b.binary(Opcode::DIV_U, Type::I32, minus_seven, two);
        let ru = b.binary(Opcode::REM_U, Type::I32, minus_seven, two);
        // The one signed overflow: INT_MIN / -1 must not panic in a debug build.
        let int_min = b.imm(Type::I32, Const::Int(0x8000_0000));
        let minus_one = b.imm(Type::I32, Const::Int(0xffff_ffff));
        let overflow = b.binary(Opcode::DIV_S, Type::I32, int_min, minus_one);
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        assert_eq!(i.temp_value(qs), Some(0xffff_fffd)); // -3, truncated toward zero
        assert_eq!(i.temp_value(rs), Some(0xffff_ffff)); // -1
        assert_eq!(i.temp_value(qu), Some(0x7fff_fffc));
        assert_eq!(i.temp_value(ru), Some(1));
        assert_eq!(i.temp_value(overflow), Some(0x8000_0000));
    }

    #[test]
    fn divide_by_zero_names_the_frontends_missing_guard() {
        let mut b = started();
        let a = b.imm(Type::I32, Const::Int(1));
        let zero = b.imm(Type::I32, Const::Int(0));
        let _ = b.binary(Opcode::DIV_U, Type::I32, a, zero);
        b.exit_tb();

        let (_, outcome) = try_run(b, &mut Host::default());
        let err = outcome.expect_err("an unguarded divide is a frontend bug");
        assert!(format!("{err}").contains("divide by zero"), "{err}");
    }

    #[test]
    fn an_out_of_range_shift_takes_the_mathematical_answer() {
        // The IR leaves this undefined and each frontend guards it; this backend
        // answers zero (or a sign fill) rather than masking the count like x86,
        // so a missing guard shows up as a differential failure.
        let mut b = started();
        let value = b.imm(Type::I32, Const::Int(0x8000_0001));
        let over = b.imm(Type::I32, Const::Int(32));
        let absurd = b.imm(Type::I32, Const::Int(0xffff_ffff));
        let shl = b.binary(Opcode::SHL, Type::I32, value, over);
        let shr = b.binary(Opcode::SHR, Type::I32, value, over);
        let sar = b.binary(Opcode::SAR, Type::I32, value, over);
        let sar_far = b.binary(Opcode::SAR, Type::I32, value, absurd);
        let positive = b.imm(Type::I32, Const::Int(1));
        let sar_positive = b.binary(Opcode::SAR, Type::I32, positive, over);
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        assert_eq!(i.temp_value(shl), Some(0));
        assert_eq!(i.temp_value(shr), Some(0));
        // Sign fill, and it does not matter how far past the width the count is.
        assert_eq!(i.temp_value(sar), Some(0xffff_ffff));
        assert_eq!(i.temp_value(sar_far), Some(0xffff_ffff));
        assert_eq!(i.temp_value(sar_positive), Some(0));
    }

    #[test]
    fn in_range_shifts_and_rotates_agree_with_the_arithmetic() {
        let mut b = started();
        let value = b.imm(Type::I32, Const::Int(0x8000_0001));
        let four = b.imm(Type::I32, Const::Int(4));
        let shl = b.binary(Opcode::SHL, Type::I32, value, four);
        let shr = b.binary(Opcode::SHR, Type::I32, value, four);
        let sar = b.binary(Opcode::SAR, Type::I32, value, four);
        let rotl = b.binary(Opcode::ROTL, Type::I32, value, four);
        let rotr = b.binary(Opcode::ROTR, Type::I32, value, four);
        // A rotate is defined at every amount, so a full turn is the identity
        // rather than the shifts' saturating answer.
        let full = b.imm(Type::I32, Const::Int(32));
        let turn = b.binary(Opcode::ROTL, Type::I32, value, full);
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        assert_eq!(i.temp_value(shl), Some(0x0000_0010));
        assert_eq!(i.temp_value(shr), Some(0x0800_0000));
        assert_eq!(i.temp_value(sar), Some(0xf800_0000));
        assert_eq!(i.temp_value(rotl), Some(0x0000_0018));
        assert_eq!(i.temp_value(rotr), Some(0x1800_0000));
        assert_eq!(i.temp_value(turn), Some(0x8000_0001));
    }

    #[test]
    fn a_bit_count_over_a_wider_operand_saturates_rather_than_panicking() {
        // `verify` rejects this block — the width of these ops is an operand,
        // not just the width the result is masked to — but `Interp` is
        // reachable without it: `jit::Dispatcher` does not verify, and neither
        // does a `no_std` board running the portable backend. This used to be
        // `leading_zeros() - (128 - w)` and it underflowed, which is a panic in
        // a debug build and a nonsense answer in a release one. Saturating is
        // an answer; panicking in the oracle is not.
        let mut b = started();
        let wide = b.imm(Type::I64, Const::Int(0x1_0000_0000));
        let narrow = b.temp(Type::I32);
        b.emit_raw(
            Opcode::CLZ,
            Type::I32,
            Some(narrow),
            None,
            &[wide],
            None,
            None,
            0,
        );
        b.exit_tb();
        let block = b.finish();
        crate::ir::verify(&block).expect_err("the verifier is the real answer");

        let mut i = Interp::new();
        let out = i.run(&block, &mut Host::default());
        assert!(out.is_ok(), "{out:?}");
        assert_eq!(i.temp_value(narrow), Some(0));
    }

    #[test]
    fn bit_counting_is_within_the_type_and_not_the_host_word() {
        let mut b = started();
        let value = b.imm(Type::I32, Const::Int(0x0000_ff00));
        let zero = b.imm(Type::I32, Const::Int(0));
        let clz = b.unary(Opcode::CLZ, Type::I32, value);
        let ctz = b.unary(Opcode::CTZ, Type::I32, value);
        let pop = b.unary(Opcode::POPCOUNT, Type::I32, value);
        // The ops' docs fix the zero case at the type's width, which is the only
        // part of it a guest can disagree about.
        let clz_zero = b.unary(Opcode::CLZ, Type::I32, zero);
        let ctz_zero = b.unary(Opcode::CTZ, Type::I32, zero);
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        assert_eq!(i.temp_value(clz), Some(16));
        assert_eq!(i.temp_value(ctz), Some(8));
        assert_eq!(i.temp_value(pop), Some(8));
        assert_eq!(i.temp_value(clz_zero), Some(32));
        assert_eq!(i.temp_value(ctz_zero), Some(32));
    }

    #[test]
    fn bswap_reverses_within_a_lane_not_across_the_word() {
        // ARM's REV16 swaps within halfwords; a whole-word swap is the same op
        // with a lane as wide as the type.
        let mut b = started();
        let value = b.imm(Type::I32, Const::Int(0x1122_3344));
        let whole = b.temp(Type::I32);
        b.emit_raw(
            Opcode::BSWAP,
            Type::I32,
            Some(whole),
            None,
            &[value],
            None,
            None,
            0,
        );
        let halves = b.temp(Type::I32);
        b.emit_raw(
            Opcode::BSWAP,
            Type::I32,
            Some(halves),
            None,
            &[value],
            Some(Const::Int(16)),
            None,
            0,
        );
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        assert_eq!(i.temp_value(whole), Some(0x4433_2211));
        assert_eq!(i.temp_value(halves), Some(0x2211_4433));
    }

    #[test]
    fn setcond_and_movcond_agree_about_signedness() {
        let mut b = started();
        let minus_one = b.imm(Type::I32, Const::Int(0xffff_ffff));
        let one = b.imm(Type::I32, Const::Int(1));
        let signed = b.setcond(Cond::LtS, Type::I32, minus_one, one);
        let unsigned = b.setcond(Cond::LtU, Type::I32, minus_one, one);
        let equal = b.setcond(Cond::Eq, Type::I32, one, one);
        // The three-operand shape: select on a one-bit temporary.
        let picked = b.emit(Opcode::MOVCOND, Type::I32, &[signed, one, minus_one]);
        // The four-operand shape: compare a pair and select, which is what an
        // ISA's conditional move actually is.
        let compared = b.temp(Type::I32);
        b.emit_raw(
            Opcode::MOVCOND,
            Type::I32,
            Some(compared),
            None,
            &[minus_one, one, one, minus_one],
            None,
            Some(Cond::GtU),
            0,
        );
        b.exit_tb();

        let (i, _) = run(b, &mut Host::default());
        assert_eq!(i.temp_value(signed), Some(1));
        assert_eq!(i.temp_value(unsigned), Some(0));
        assert_eq!(i.temp_value(equal), Some(1));
        assert_eq!(i.temp_value(picked), Some(1));
        // 0xffffffff > 1 unsigned, so the true arm wins.
        assert_eq!(i.temp_value(compared), Some(1));
    }

    #[test]
    fn a_taken_branch_skips_what_it_jumps_over() {
        let mut b = started();
        let one = b.imm(Type::I32, Const::Int(1));
        let taken = b.imm(Type::I1, Const::Int(1));
        let br = b.emit_raw(
            Opcode::BRCOND,
            Type::I1,
            None,
            None,
            &[taken],
            None,
            None,
            0,
        );
        // Skipped: if this ran, its temporary would hold 2 rather than 0.
        let doubled = b.binary(Opcode::ADD, Type::I32, one, one);
        let target = b.next_index();
        b.patch_aux(br, target as u32);
        let after = b.binary(Opcode::ADD, Type::I32, one, one);
        b.exit_tb();

        let (i, outcome) = run(b, &mut Host::default());
        assert_eq!(outcome, Outcome::Exit);
        assert_eq!(i.temp_value(doubled), Some(0));
        assert_eq!(i.temp_value(after), Some(2));
    }

    #[test]
    fn the_terminators_say_where_they_go() {
        let mut host = Host::default();

        let mut b = started();
        b.emit_raw(
            Opcode::GOTO_TB,
            Type::I64,
            None,
            None,
            &[],
            Some(Const::Int(0x2000)),
            None,
            0,
        );
        let (_, outcome) = run(b, &mut host);
        assert_eq!(outcome, Outcome::Goto { pc: 0x2000 });

        let mut b = started();
        let pc = b.imm(Type::I64, Const::Int(0x3000));
        b.emit_raw(
            Opcode::LOOKUP_AND_GOTO,
            Type::I64,
            None,
            None,
            &[pc],
            None,
            None,
            0,
        );
        let (_, outcome) = run(b, &mut host);
        assert_eq!(outcome, Outcome::Lookup { pc: 0x3000 });
    }

    #[test]
    fn loads_and_stores_reach_the_host_and_honour_their_descriptor() {
        let mut host = Host::default();
        host.mem.insert(0x40, 0x80);
        host.mem.insert(0x41, 0x12);

        let mut b = started();
        let addr = b.imm(Type::I64, Const::Int(0x40));
        let byte = b.load(Type::I32, addr, MemOp::load(Width::U8));
        let mut signed = MemOp::load(Width::U8);
        signed.sign = Sign::Signed;
        let sbyte = b.load(Type::I32, addr, signed);
        let half = b.load(Type::I32, addr, MemOp::load(Width::U16));
        // A store truncates to the descriptor's width, not the temporary's.
        let wide = b.imm(Type::I32, Const::Int(0xdead_beef));
        let dst = b.imm(Type::I64, Const::Int(0x50));
        b.store(Type::I32, dst, wide, MemOp::store(Width::U8));
        b.exit_tb();

        let (i, outcome) = run(b, &mut host);
        assert_eq!(outcome, Outcome::Exit);
        assert_eq!(i.temp_value(byte), Some(0x80));
        assert_eq!(i.temp_value(sbyte), Some(0xffff_ff80));
        assert_eq!(i.temp_value(half), Some(0x1280));
        assert_eq!(host.mem.get(&0x50).copied(), Some(0xef));
        assert_eq!(host.mem.get(&0x51), None);
    }

    #[test]
    fn a_faulting_access_stops_the_block_at_its_boundary() {
        let mut host = Host::default();
        host.faults.insert(0x2000, BusError::Unassigned);

        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0));
        b.charge(2);
        b.insn_start(mark(0x1004, 2));
        b.charge(3);
        let addr = b.imm(Type::I64, Const::Int(0x2000));
        let _ = b.load(Type::I32, addr, MemOp::load(Width::U32));
        // Never reached: the block stops where it faulted.
        let _ = b.binary(Opcode::ADD, Type::I64, addr, addr);
        b.exit_tb();

        let (i, outcome) = run(b, &mut host);
        let Outcome::Fault(fault) = outcome else {
            panic!("a fault, not {outcome:?}");
        };
        assert_eq!(fault.error, BusError::Unassigned);
        assert_eq!(fault.mark, Some(1));
        assert_eq!(fault.pc, 0x1004);
        // The architectural count is the boundary's; the charged count includes
        // what the faulting instruction had already spent.
        assert_eq!(fault.retired_ticks, 2);
        assert_eq!(fault.charged_ticks, 5);
        assert_eq!(i.ticks(), 5);
        // The charge committed, so this fault is not one the guest can restart.
        assert!(!fault.restartable);
    }

    #[test]
    fn a_fault_before_any_boundary_reports_the_blocks_entry_pc() {
        let mut host = Host::default();
        host.faults.insert(0x2000, BusError::Protected);

        let mut b = BlockBuilder::new(0x9000, 0);
        let addr = b.imm(Type::I64, Const::Int(0x2000));
        let _ = b.load(Type::I32, addr, MemOp::load(Width::U8));
        b.exit_tb();

        let (_, outcome) = run(b, &mut host);
        let Outcome::Fault(fault) = outcome else {
            panic!("a fault, not {outcome:?}");
        };
        assert_eq!(fault.mark, None);
        assert_eq!(fault.pc, 0x9000);
        assert_eq!(fault.retired_ticks, 0);
    }

    #[test]
    fn a_retry_is_deliverable_only_before_a_side_effect() {
        let mut host = Host::default();
        host.faults.insert(0x2000, BusError::Retry);

        let mut b = started();
        let addr = b.imm(Type::I64, Const::Int(0x2000));
        let _ = b.load(Type::I32, addr, MemOp::load(Width::U8));
        b.exit_tb();
        let (_, outcome) = run(b, &mut host);
        let Outcome::Fault(fault) = outcome else {
            panic!("a fault, not {outcome:?}");
        };
        assert_eq!(fault.error, BusError::Retry);
        assert!(fault.restartable);

        // Once the guest instruction has charged a tick there is nothing to
        // restart from, and the answer is rejected rather than passed on.
        let mut b = started();
        b.charge(1);
        let addr = b.imm(Type::I64, Const::Int(0x2000));
        let _ = b.load(Type::I32, addr, MemOp::load(Width::U8));
        b.exit_tb();
        let (_, outcome) = try_run(b, &mut host);
        assert_eq!(
            outcome.expect_err("a retry after a commit is not deliverable"),
            Error::Bus(BusError::Retry)
        );
    }

    #[test]
    fn a_boundary_resets_what_a_retry_is_measured_against() {
        // Restart granularity is the guest instruction, so the previous one's
        // commits must stop blocking a retry at the next boundary.
        let mut host = Host::default();
        host.faults.insert(0x2000, BusError::Retry);

        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0));
        b.charge(4);
        b.insn_start(mark(0x1004, 4));
        let addr = b.imm(Type::I64, Const::Int(0x2000));
        let _ = b.load(Type::I32, addr, MemOp::load(Width::U8));
        b.exit_tb();

        let (_, outcome) = run(b, &mut host);
        let Outcome::Fault(fault) = outcome else {
            panic!("a fault, not {outcome:?}");
        };
        assert!(fault.restartable);
        assert_eq!(fault.retired_ticks, 4);
    }

    #[test]
    fn ticks_are_charged_exactly_and_boundaries_are_published_in_order() {
        let mut host = Host::default();
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0));
        b.charge(2);
        b.insn_start(mark(0x1004, 2));
        b.charge(3);
        b.charge(1);
        b.insn_start(mark(0x1008, 6));
        b.charge(4);
        b.exit_tb();

        let (i, outcome) = run(b, &mut host);
        assert_eq!(outcome, Outcome::Exit);
        // Exactly, unrounded and unbatched: the count is hashed output.
        assert_eq!(host.ticks, 10);
        assert_eq!(i.ticks(), 10);
        assert_eq!(
            host.boundaries,
            vec![
                (0x1000, 0x1004, 0),
                (0x1004, 0x1008, 2),
                (0x1008, 0x100c, 6)
            ]
        );
    }

    #[test]
    fn guest_state_is_written_out_once_however_many_boundaries_a_trace_has() {
        // The saving superblocks are for: a trace binds the same slot at every
        // boundary and the host must see one write, not one per instruction.
        // Four boundaries, one slot, one `write_slot`.
        let mut host = Host::default();
        let mut b = BlockBuilder::new(0x1000, 0);
        let mut last = b.imm(Type::I64, Const::Int(0));
        for i in 0..4u64 {
            b.insn_start(InsnStart {
                pc: 0x1000 + i * 4,
                next_pc: 0x1004 + i * 4,
                ticks: i,
                live: vec![(RegSlot(3), last)],
            });
            b.charge(1);
            let one = b.imm(Type::I64, Const::Int(1));
            last = b.binary(Opcode::ADD, Type::I64, last, one);
        }
        b.insn_start(InsnStart {
            pc: 0x1010,
            next_pc: 0x1010,
            ticks: 4,
            live: vec![(RegSlot(3), last)],
        });
        b.exit_tb();

        let (_, outcome) = run(b, &mut host);
        assert_eq!(outcome, Outcome::Exit);
        assert_eq!(
            host.published,
            vec![(3, 4)],
            "guest state went out once, holding the last boundary's value"
        );
    }

    #[test]
    fn a_fault_materializes_the_boundary_it_faulted_at_and_not_a_later_one() {
        // The precise-exception claim at the level of the backend: two guest
        // instructions, the second faults, and the slot must hold what the
        // *second* boundary named — not the value the first bound and not one
        // the faulting instruction computed.
        let mut host = Host::default();
        host.faults.insert(0x80, BusError::BadAccess);

        let mut b = BlockBuilder::new(0x1000, 0);
        let first = b.imm(Type::I64, Const::Int(0x11));
        b.insn_start(InsnStart {
            pc: 0x1000,
            next_pc: 0x1004,
            ticks: 0,
            live: vec![(RegSlot(3), first)],
        });
        b.charge(1);
        let second = b.imm(Type::I64, Const::Int(0x22));
        b.insn_start(InsnStart {
            pc: 0x1004,
            next_pc: 0x1008,
            ticks: 1,
            live: vec![(RegSlot(3), second)],
        });
        b.charge(1);
        let addr = b.imm(Type::I64, Const::Int(0x80));
        let mut mem = MemOp::load(Width::U64);
        mem.volatile = true;
        let loaded = b.load(Type::I64, addr, mem);
        b.insn_start(InsnStart {
            pc: 0x1008,
            next_pc: 0x1008,
            ticks: 2,
            live: vec![(RegSlot(3), loaded)],
        });
        b.exit_tb();

        let (i, outcome) = run(b, &mut host);
        let Outcome::Fault(fault) = outcome else {
            panic!("the load faults");
        };
        assert_eq!(fault.pc, 0x1004);
        assert_eq!(host.read_slot(RegSlot(3)), 0x22, "the faulting boundary's");
        assert_eq!(host.published, vec![(3, 0x22)], "and only that one");
        // Two boundaries were passed, so one guest instruction retired.
        assert_eq!(i.boundaries(), 2);
        assert_eq!(i.mark(), Some(1));
    }

    #[test]
    fn a_boundary_publishes_its_live_slots_before_it_announces_itself() {
        let mut host = Host::default();
        let mut b = BlockBuilder::new(0x1000, 0);
        b.insn_start(mark(0x1000, 0));
        let a = b.imm(Type::I32, Const::Int(0xaaaa));
        let flag = b.imm(Type::I1, Const::Int(1));
        b.charge(1);
        b.insn_start(InsnStart {
            pc: 0x1004,
            next_pc: 0x1008,
            ticks: 1,
            // A flag is a slot like any other, and the numbering is the
            // frontend's: 0x2000 here is not "register 0x2000".
            live: vec![(RegSlot(3), a), (RegSlot(0x2000), flag)],
        });
        b.exit_tb();

        let (_, outcome) = run(b, &mut host);
        assert_eq!(outcome, Outcome::Exit);
        assert_eq!(host.published, vec![(3, 0xaaaa), (0x2000, 1)]);
        assert_eq!(host.read_slot(RegSlot(3)), 0xaaaa);
        assert_eq!(host.read_slot(RegSlot(0x2000)), 1);
        // A slot nobody published reads as whatever the host had.
        assert_eq!(host.read_slot(RegSlot(9)), 0);
    }

    #[test]
    fn a_slot_read_of_a_shadowed_slot_sees_the_temporary_and_not_the_host() {
        // Lazy publication has exactly one way to be observed from inside a
        // block: a `get_slot` naming a slot the current boundary binds. The
        // first frontend never emits one, so this is the guard that keeps a
        // later frontend from finding out the hard way.
        let mut host = Host::default();
        host.slots.insert(3, 0x1111);

        let mut b = BlockBuilder::new(0x1000, 0);
        let fresh = b.imm(Type::I64, Const::Int(0x2222));
        b.insn_start(InsnStart {
            pc: 0x1000,
            next_pc: 0x1004,
            ticks: 0,
            live: vec![(RegSlot(3), fresh)],
        });
        b.charge(1);
        let read_back = b.get_slot(Type::I64, RegSlot(3));
        b.insn_start(InsnStart {
            pc: 0x1004,
            next_pc: 0x1004,
            ticks: 1,
            live: vec![(RegSlot(9), read_back)],
        });
        b.exit_tb();

        let (i, outcome) = run(b, &mut host);
        assert_eq!(outcome, Outcome::Exit);
        assert_eq!(
            i.temp_value(read_back),
            Some(0x2222),
            "the slot read saw the value the host still held"
        );
    }

    #[test]
    fn the_atomics_go_out_through_one_host_method() {
        let mut host = Host::default();
        host.poke(0x80, Width::U32, 7);

        let mut b = started();
        let addr = b.imm(Type::I64, Const::Int(0x80));
        let seven = b.imm(Type::I32, Const::Int(7));
        let nine = b.imm(Type::I32, Const::Int(9));
        let previous = b.temp(Type::I32);
        let swapped = b.temp(Type::I1);
        b.emit_raw(
            Opcode::CMPXCHG,
            Type::I32,
            Some(previous),
            Some(swapped),
            &[addr, seven, nine],
            None,
            None,
            0,
        );
        let added = b.temp(Type::I32);
        b.emit_raw(
            Opcode::FETCH_ADD,
            Type::I32,
            Some(added),
            None,
            &[addr, seven],
            None,
            None,
            0,
        );
        let exchanged = b.temp(Type::I32);
        b.emit_raw(
            Opcode::XCHG,
            Type::I32,
            Some(exchanged),
            None,
            &[addr, nine],
            None,
            None,
            0,
        );
        b.emit_raw(Opcode::FENCE, Type::I32, None, None, &[], None, None, 0);
        b.exit_tb();

        let (i, outcome) = run(b, &mut host);
        assert_eq!(outcome, Outcome::Exit);
        assert_eq!(i.temp_value(previous), Some(7));
        assert_eq!(i.temp_value(swapped), Some(1));
        assert_eq!(i.temp_value(added), Some(9));
        assert_eq!(i.temp_value(exchanged), Some(16));
        assert_eq!(host.peek(0x80, Width::U32), 9);
        assert_eq!(host.fences, 1);
    }

    #[test]
    fn a_reservation_survives_until_something_takes_it() {
        // The case `cmpxchg` cannot express: the monitor is CPU state, and the
        // store-exclusive reports whether it still held.
        let mut host = Host::default();
        host.poke(0x100, Width::U32, 1);

        let mut b = started();
        let addr = b.imm(Type::I64, Const::Int(0x100));
        let value = b.imm(Type::I32, Const::Int(0x55));
        let loaded = b.temp(Type::I32);
        b.emit_raw(
            Opcode::LD_EXCL,
            Type::I32,
            Some(loaded),
            None,
            &[addr],
            None,
            None,
            0,
        );
        let first = b.temp(Type::I32);
        b.emit_raw(
            Opcode::ST_EXCL,
            Type::I32,
            Some(first),
            None,
            &[addr, value],
            None,
            None,
            0,
        );
        let second = b.temp(Type::I32);
        b.emit_raw(
            Opcode::ST_EXCL,
            Type::I32,
            Some(second),
            None,
            &[addr, value],
            None,
            None,
            0,
        );
        b.exit_tb();

        let (i, _) = run(b, &mut host);
        assert_eq!(i.temp_value(loaded), Some(1));
        // One means the store happened; a frontend inverts for ARM and RISC-V,
        // which both write zero on success.
        assert_eq!(i.temp_value(first), Some(1));
        assert_eq!(i.temp_value(second), Some(0));
        assert_eq!(host.peek(0x100, Width::U32), 0x55);
    }

    #[test]
    fn an_atomic_whose_type_has_no_access_width_is_rejected() {
        let mut b = started();
        let addr = b.imm(Type::I64, Const::Int(0x80));
        let one = b.imm(Type::I128, Const::Int(1));
        let previous = b.temp(Type::I128);
        b.emit_raw(
            Opcode::FETCH_ADD,
            Type::I128,
            Some(previous),
            None,
            &[addr, one],
            None,
            None,
            0,
        );
        b.exit_tb();

        let (_, outcome) = try_run(b, &mut Host::default());
        let err = outcome.expect_err("i128 is not an access width");
        assert!(format!("{err}").contains("must be i32 or i64"), "{err}");
    }

    #[test]
    fn a_helper_call_carries_its_arguments_and_both_results() {
        let mut host = Host::default();
        let mut b = started();
        let a = b.imm(Type::I64, Const::Int(5));
        let c = b.imm(Type::I64, Const::Int(6));
        let value = b.temp(Type::I64);
        let flags = b.temp(Type::I64);
        b.emit_raw(
            Opcode::CALL_HELPER,
            Type::I64,
            Some(value),
            Some(flags),
            &[a, c],
            None,
            None,
            42,
        );
        b.exit_tb();

        let (i, outcome) = run(b, &mut host);
        assert_eq!(outcome, Outcome::Exit);
        assert_eq!(host.helpers, vec![(42, vec![5, 6])]);
        assert_eq!(i.temp_value(value), Some(11));
        // The second result: soft-float returns a value and its flags.
        assert_eq!(i.temp_value(flags), Some(2));
    }

    #[test]
    fn a_helpers_failure_is_the_blocks_failure() {
        let mut b = started();
        let value = b.temp(Type::I64);
        b.emit_raw(
            Opcode::CALL_HELPER,
            Type::I64,
            Some(value),
            None,
            &[],
            None,
            None,
            1,
        );
        b.exit_tb();

        // The defaulted `call_helper`, which is what a host with none has.
        struct Bare;
        impl IrHost for Bare {
            fn read_slot(&mut self, _: RegSlot) -> u128 {
                0
            }
            fn write_slot(&mut self, _: RegSlot, _: u128) {}
            fn load(&mut self, _: &MemOp, _: u64) -> MemResult<u64> {
                Err(BusError::Unassigned)
            }
            fn store(&mut self, _: &MemOp, _: u64, _: u64) -> MemResult {
                Err(BusError::Unassigned)
            }
            fn charge(&mut self, _: u64) {}
            fn insn_start(&mut self, _: &InsnStart) {}
        }

        let block = b.finish();
        verify(&block).expect("the verifier accepts what these tests emit");
        // Through a trait object, because `run` takes `?Sized` so a dispatcher
        // can hold one host behind a pointer.
        let mut bare = Bare;
        let host: &mut dyn IrHost = &mut bare;
        let err = Interp::new()
            .run(&block, host)
            .expect_err("a host with no helpers says so");
        assert_eq!(err, Error::Unimplemented("an IR helper call"));
    }

    #[test]
    fn an_op_this_backend_does_not_lower_is_reported_rather_than_guessed() {
        // `phi` is real and required — superblocks span branches — but nothing
        // in an instruction records which predecessor each operand came from,
        // so it cannot be executed rather than merely being unimplemented.
        let mut b = started();
        let a = b.imm(Type::I32, Const::Int(1));
        let merged = b.temp(Type::I32);
        b.emit_raw(
            Opcode::PHI,
            Type::I32,
            Some(merged),
            None,
            &[a, a],
            None,
            None,
            0,
        );
        b.exit_tb();
        let (_, outcome) = run(b, &mut Host::default());
        assert_eq!(
            outcome,
            Outcome::Unsupported {
                op: Opcode::PHI,
                at: 2
            }
        );

        // And an opcode from beyond the defined set, which is the whole point of
        // the extensible-enumeration pattern.
        let mut b = started();
        b.emit_void(Opcode(0xfff), Type::I32, &[]);
        b.exit_tb();
        let (_, outcome) = run(b, &mut Host::default());
        assert_eq!(
            outcome,
            Outcome::Unsupported {
                op: Opcode(0xfff),
                at: 1
            }
        );
    }

    #[test]
    fn a_malformed_block_is_a_message_and_not_a_panic() {
        // The fuzz targets run unverified blocks through here, so every bad
        // shape has to come back as an error naming the instruction.
        let mut b = started();
        let a = b.imm(Type::I32, Const::Int(1));
        let _ = b.emit(Opcode::ADD, Type::I32, &[a]);
        b.exit_tb();
        let block = b.finish();
        let err = Interp::new()
            .run(&block, &mut Host::default())
            .expect_err("too few operands");
        assert!(
            format!("{err}").contains("too few source operands"),
            "{err}"
        );

        // A backward branch that never stops.
        let mut b = started();
        let taken = b.imm(Type::I1, Const::Int(1));
        let here = b.next_index();
        b.emit_raw(
            Opcode::BRCOND,
            Type::I1,
            None,
            None,
            &[taken],
            None,
            None,
            here as u32,
        );
        b.exit_tb();
        let block = b.finish();
        let err = Interp::new()
            .run(&block, &mut Host::default())
            .expect_err("a block that never terminates");
        assert!(format!("{err}").contains("did not terminate"), "{err}");
    }

    #[test]
    fn float_types_may_be_carried_but_not_added() {
        let mut b = started();
        // Carrying is fine — a helper call takes and returns these.
        let bits = b.imm(Type::F64, Const::F64Bits(0x7ff0_0000_0000_0001));
        let copied = b.unary(Opcode::MOV, Type::F64, bits);
        b.exit_tb();
        let (i, _) = run(b, &mut Host::default());
        assert_eq!(i.temp_value(copied), Some(0x7ff0_0000_0000_0001));

        let mut b = started();
        let x = b.imm(Type::F64, Const::F64Bits(1));
        let _ = b.binary(Opcode::ADD, Type::F64, x, x);
        b.exit_tb();
        let (_, outcome) = try_run(b, &mut Host::default());
        let err = outcome.expect_err("the IR never does float arithmetic");
        assert!(format!("{err}").contains("non-integer type"), "{err}");
    }

    #[test]
    fn the_bitfield_payload_survives_a_round_trip() {
        assert_eq!(bitfield_parts(bitfield_aux(8, 8)), (8, 8));
        assert_eq!(bitfield_parts(bitfield_aux(0, 128)), (0, 128));
        assert_eq!(bitfield_parts(bitfield_aux(63, 1)), (63, 1));
    }

    #[test]
    fn a_bitfield_that_leaves_its_type_is_rejected() {
        let mut b = started();
        let a = b.imm(Type::I32, Const::Int(0));
        let out = b.temp(Type::I32);
        b.emit_raw(
            Opcode::EXTRACT,
            Type::I32,
            Some(out),
            None,
            &[a],
            None,
            None,
            bitfield_aux(24, 16),
        );
        b.exit_tb();
        let (_, outcome) = try_run(b, &mut Host::default());
        let err = outcome.expect_err("the field runs off the end of the type");
        assert!(format!("{err}").contains("does not fit"), "{err}");
    }
}
