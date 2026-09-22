//! The translated execution engine: [`lift`](super::lift)'s blocks run on the
//! portable IR backend, with the interpreter underneath everything it declines.
//!
//! `engine = "ir"` on `cpu.m68k` selects it. What it is *not* is a second
//! semantics: `ROADMAP.md` §0 requires a bit-identical state hash across the
//! interpreter and a translated engine for the same guest, so every column a
//! guest or a snapshot can see — registers, `SR`, the program counter, the
//! **prefetch queue**, memory, faults, and the cycle count — comes out the
//! same number. [`differential`](super::differential) is what says so.
//!
//! # The shape, in one paragraph
//!
//! One call to [`advance`] runs **one block, or one interpreted instruction**.
//! A block is lifted once, cached under its entry PC, and executed by
//! [`Interp`] — the portable backend, which runs anywhere the crate does,
//! `no_std` and both wasm targets included. There is no host code generator
//! here and no block chaining: a backend that lowers these blocks lives in
//! `jit/`, above the `std` line, and chaining needs a successor-linking design
//! this frontend does not have yet.
//!
//! # Four reasons a block does not run, and the interpreter picks it up
//!
//! 1. **The core is not in a liftable state** ([`liftable`]): a pending reset,
//!    a halt, `STOP`, an `RTE` replay in flight, **T** set in `SR` (every
//!    instruction would take a trace exception), an odd program counter, a
//!    pending interrupt, or any model but a 68000.
//! 2. **The instruction at the PC is outside the subset**, so the lift
//!    produced a block covering nothing. Recorded, so the next pass does not
//!    lift it again.
//! 3. **A fault.** See below.
//! 4. **The block left part-way through** — [`Outcome::Spent`] — which is not
//!    a fallback at all: the guest is standing at a boundary and the next call
//!    picks up from there.
//!
//! # A fault restarts the instruction on the interpreter
//!
//! `lift`'s module docs have the argument in full. The short form: a 68000's
//! mid-instruction fault is visible in registers the instruction has already
//! changed and in a stack frame whose program counter depends on how far the
//! prefetch got, and the IR cannot publish a write that lands between two
//! boundaries. So [`advance`] unwinds the partial instruction's charges —
//! which is the reconciliation
//! [`Fault`](crate::ir::Fault) documents for a core that restarts rather than
//! resumes — and hands that one instruction to `Exec::step`, which redoes it
//! and faults where the hardware does.
//!
//! The two numbers to unwind by are **the host's**, not the IR's, and that is
//! worth stating because getting it wrong was a real defect rather than a
//! hypothetical: `Fault::charged_ticks` and `Fault::retired_ticks` are both
//! measured from [`Interp`]'s own counter, which sees
//! [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) immediates and **not** what a
//! host charges inside an access. On a core where every bus cycle is four
//! host-charged ticks that is most of the count. [`Finished::retired`] has the
//! worked example.
//!
//! The lifted subset is chosen so that this is **exact**: no lifted
//! instruction commits more than one store, and the store is its last memory
//! access, so a fault can only arrive with nothing committed. [`Host::store`]
//! asserts it in a debug build rather than trusting the frontend.
//!
//! # Self-modifying code
//!
//! A cached block is a translation of bytes, so it is only valid while those
//! bytes are what they were. Two mechanisms, and neither is a heuristic:
//!
//! * **Validation on dispatch.** [`Entry::seen`] is every `(address, word)`
//!   pair the lifter read, and a cache hit re-reads them. A single mismatch
//!   drops the entry and lifts again. That is O(the block's length) per
//!   dispatch and it is the honest price of having no store log in
//!   `Exec` — a host backend would replace it with one, and
//!   `cpu::riscv::engine`'s `Host::note_writes` is what that looks like.
//! * **Leaving the block.** Validation cannot help a store the *running*
//!   block makes, so [`Host::store`] notices a store into the running block's
//!   own [`lift::WINDOW`] and [`Host::spent`] then leaves at the next guest
//!   instruction boundary — the boundary the store's own instruction ends at,
//!   which is where the effect can first be honoured.
//!
//! One skew is left and is written down rather than rounded up: the 68000
//! fetches two words ahead, so a store that lands on a word *already in the
//! prefetch queue* is not seen by the instruction that consumes it, and this
//! frontend bakes extension words in as constants at lift time. Inside a
//! block that is unreachable — the guard above ends the block at the store —
//! so it needs a second bus master writing the code a block is running. See
//! `docs/cpu/m68k-ir.md`.
//!
//! # Sources
//!
//! `exec.rs` is the oracle for every number here; where a cycle count or an
//! access order is asserted, the citation is in `lift.rs` beside the code that
//! emits it. No emulator source of any licence was opened (`ROADMAP.md` §1).

use alloc::collections::BTreeMap;
use alloc::vec::Vec;

use crate::core::error::BusError;
use crate::core::space::{AddressSpace, MemAttrs, MemResult};
use crate::core::value::Width;
use crate::ir::{Align, Block, InsnStart, Interp, IrHost, MemOp, Outcome, RegSlot, verify};

use super::exec::{Exec, State};
use super::isa::Model;
use super::lift::{self, SLOT_COUNT};
use super::{Config, Lines, flags};

/// How many blocks the cache holds before it is emptied.
///
/// A flat bound rather than an eviction policy: a 68000's address space is at
/// most sixteen megabytes and a translation is small, so the interesting
/// question is not which block to throw away but whether the map can grow
/// without bound. It cannot. Clearing the lot is a blunt answer to a rare
/// event, which is the same answer `cpu::riscv::engine`'s `Unlifted::clear`
/// gives for the same reason.
const MAX_BLOCKS: usize = 4096;

/// One cached translation, and the bytes it is a translation *of*.
#[derive(Debug)]
struct Entry {
    /// The block, or `None` when the instruction at this PC is outside the
    /// lifted subset — recorded so the next pass does not lift it again.
    block: Option<Block>,
    /// Every `(address, word)` pair the lifter read, in the order it read
    /// them. Re-read on every hit; see the module docs.
    seen: Vec<(u32, u16)>,
}

/// What a translated core has done, and what it holds.
///
/// A statistic and never a behaviour — the two engines are indistinguishable
/// to the guest — so nothing here is snapshotted.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct Stats {
    /// Blocks lifted.
    pub lifted: u64,
    /// Blocks executed.
    pub executed: u64,
    /// Guest instructions retired inside a block.
    pub retired: u64,
    /// Instructions the interpreter executed because no block could.
    pub interpreted: u64,
    /// Faults a block took, each handed back to the interpreter.
    pub faults: u64,
    /// Cache entries dropped because the guest rewrote their bytes.
    pub invalidated: u64,
    /// Blocks that left part-way through at a guest instruction boundary.
    pub spent: u64,
    /// Guest instructions retired **in the unit the interpreter counts**:
    /// `Exec::step`s, so an exception sequence is one and an instruction is
    /// one.
    ///
    /// The column [`differential`](super::differential) drives the oracle by,
    /// which is why it is a statistic worth keeping rather than a curiosity:
    /// a translated unit is one *block*, and the harness has to know how many
    /// interpreter steps that block was worth to step the oracle exactly that
    /// far.
    pub steps: u64,
}

/// The translation cache, and the backend that runs what is in it.
///
/// Lives in the core's session, behind the same lock the interpreter's state
/// is behind, and is **derived state**: never serialized, thrown away on a
/// reset and whenever the topology generation moves (CLAUDE.md, "Devices").
#[derive(Debug)]
pub(super) struct Runtime {
    entries: BTreeMap<u32, Entry>,
    /// Reused across blocks: a translator that allocates per block pays for it
    /// on every block.
    interp: Interp,
    /// The address-space generation the entries were lifted under.
    generation: u64,
    stats: Stats,
}

impl Runtime {
    pub(super) fn new() -> Runtime {
        Runtime {
            entries: BTreeMap::new(),
            interp: Interp::new(),
            generation: 0,
            stats: Stats::default(),
        }
    }

    /// What this core's translated engine has done.
    pub(super) fn stats(&self) -> Stats {
        self.stats
    }

    /// Throw every translation away.
    pub(super) fn flush(&mut self) {
        self.entries.clear();
    }
}

/// Whether the core is in a state a lifted block may run in.
///
/// Each of these is something `Exec::step_inner` does *before* it reaches an
/// instruction, or something the lifted subset cannot express, and a block
/// that ran anyway would skip it.
fn liftable(state: &State, cfg: &Config, lines: &Lines) -> bool {
    cfg.model == Model::M68000
        && !state.reset_pending
        && !state.halted
        && !state.stopped
        && state.replay.is_none()
        // **T** means every instruction ends in a trace exception, which is
        // exception processing rather than an instruction (MC68000UM §6.2.5).
        && state.sr & flags::T == 0
        // An odd program counter is an address error on the fetch, and the
        // block's entry word could not have been read at lift time either.
        && state.pc & 1 == 0
        && !lines.interrupt_pending(state.ipl_mask())
}

/// Advance the core by one *unit of this engine*: one block, or — where a
/// block would be wrong — one interpreted instruction.
///
/// `allowance` is what is left of the caller's tick budget, and it is an
/// allowance rather than advice: [`Host::spent`] compares it against the ticks
/// charged at every guest instruction boundary but a block's first, and a
/// block that has spent it leaves *there* — at the same instruction an
/// interpreted core would have stopped at, with the same `State::debt`. Pass
/// [`u64::MAX`] to mean "run the whole block".
///
/// Returns the cycles charged, in the same currency and with the same meaning
/// as `Exec::step`, so a run loop cannot tell which engine it is driving.
///
/// # Panics
///
/// If a lifted block reaches an op the IR backend does not implement, or the
/// backend refuses the block. Neither is a guest condition: it is this crate's
/// own frontend emitting something its own backend cannot execute, and the
/// architectural state at that point is not reconstructible — so it is
/// reported loudly rather than papered over, exactly as `cpu::riscv::engine`
/// does.
pub(super) fn advance(
    rt: &mut Runtime,
    state: &mut State,
    space: &AddressSpace,
    cfg: &Config,
    lines: &Lines,
    allowance: u64,
) -> u64 {
    // Derived state, invalidated by the topology generation counter.
    let generation = space.generation();
    if rt.generation != generation {
        rt.generation = generation;
        rt.entries.clear();
    }
    if !liftable(state, cfg, lines) {
        rt.stats.interpreted += 1;
        rt.stats.steps += 1;
        return Exec::new(state, space, cfg, lines).step();
    }
    let pc = state.pc;
    ensure(rt, pc, space, cfg);

    let Runtime {
        entries,
        interp,
        stats,
        ..
    } = rt;
    let Some(block) = entries.get(&pc).and_then(|e| e.block.as_ref()) else {
        stats.interpreted += 1;
        stats.steps += 1;
        return Exec::new(state, space, cfg, lines).step();
    };

    stats.executed += 1;
    let mut host = Host::new(state, space, cfg, lines, allowance, pc & !lift::WINDOW_MASK);
    let outcome = interp.run(block, &mut host);
    let retired = interp.boundaries().saturating_sub(1);
    let Finished {
        slots,
        used,
        retired: at_boundary,
    } = host.finish();

    // Every retired instruction, back into the architectural register file.
    state.d.copy_from_slice(&slots[0..8]);
    state.a.copy_from_slice(&slots[8..16]);
    // **Only the condition codes.** No lifted instruction writes **S**, **T**
    // or the interrupt mask, so the stack-pointer bank cannot have moved under
    // the block and `State::set_sr`'s swap is not owed (`lift`'s module docs).
    state.sr = (state.sr & !flags::CCR) | (slots[SR_INDEX] as u16 & flags::CCR);
    state.prefetch = [slots[P0_INDEX] as u16, slots[P1_INDEX] as u16];

    let outcome = match outcome {
        Ok(outcome) => outcome,
        Err(e) => panic!("the m68k frontend produced a block the IR backend refused: {e}"),
    };
    stats.retired += retired;
    stats.steps += retired;

    match outcome {
        Outcome::Fault(fault) => {
            stats.faults += 1;
            // The faulting instruction is the one that did *not* retire, and
            // the interpreter is about to take it — instruction and exception
            // together, which is one `Exec::step` and so one oracle step.
            stats.steps += 1;
            restart(
                state,
                space,
                cfg,
                lines,
                fault.pc as u32,
                used.saturating_sub(at_boundary),
                at_boundary,
            )
        }
        Outcome::Unsupported { op, at } => panic!(
            "the m68k frontend emitted {op} at index {at}, which the IR backend cannot execute"
        ),
        Outcome::Spent { pc } => {
            stats.spent += 1;
            state.pc = pc as u32;
            used.max(1)
        }
        // `Exit` is the only other outcome this frontend produces: it emits no
        // `goto_tb` and no `lookup_and_goto`, because chaining is `jit/`'s.
        _ => {
            state.pc = slots[PC_INDEX];
            used.max(1)
        }
    }
}

/// Unwind the faulting instruction and let the interpreter take it.
///
/// The block stopped *at* the faulting instruction with the state that
/// instruction started with — that is what [`Interp`] publishes from the
/// boundary's live mapping — so the two numbers [`Fault`] reports are all that
/// is left to reconcile: `charged_ticks - retired_ticks` is what the partial
/// instruction spent, and a core that restarts it must give those back or pay
/// for them twice.
fn restart(
    state: &mut State,
    space: &AddressSpace,
    cfg: &Config,
    lines: &Lines,
    at: u32,
    partial: u64,
    retired: u64,
) -> u64 {
    state.cycles = state.cycles.wrapping_sub(partial);
    state.pc = at;
    // The queue as of that boundary is already in `state.prefetch`: the block
    // published it from the boundary's live mapping, or never touched it, and
    // either way it is what this instruction's own fetches left.
    let again = Exec::new(state, space, cfg, lines).step();
    retired.wrapping_add(again).max(1)
}

/// Make sure the cache holds an answer for `pc`, lifting one if it does not.
fn ensure(rt: &mut Runtime, pc: u32, space: &AddressSpace, cfg: &Config) {
    if let Some(entry) = rt.entries.get(&pc) {
        if valid(entry, space, cfg) {
            return;
        }
        rt.entries.remove(&pc);
        rt.stats.invalidated += 1;
    }
    if rt.entries.len() >= MAX_BLOCKS {
        rt.entries.clear();
    }
    let mut reader = Reader {
        space,
        attrs: MemAttrs::DEBUG.with_requester(cfg.requester),
        mask: cfg.model.address_mask(),
        seen: Vec::new(),
    };
    // A read-ahead rather than a fetch: this reads up to `MAX_INSNS`
    // instructions the guest has not asked for, which is the one place this
    // core reads guest memory the way a debugger does (CLAUDE.md, "a debugger
    // read must not pop a FIFO"). The block's *own* fetch accesses are
    // performed at run time, through the host, with ordinary attributes.
    let lifted = lift::lift(cfg.model, pc, &mut reader, lift::MAX_INSNS);
    let entry = match lifted {
        Ok(lifted) if lifted.insns > 0 => {
            debug_assert!(
                verify(&lifted.block).is_ok(),
                "the m68k frontend produced a block the verifier rejects: {:?}\n{}",
                verify(&lifted.block),
                lifted.block
            );
            rt.stats.lifted += 1;
            Entry {
                block: Some(lifted.block),
                seen: reader.seen,
            }
        }
        // Nothing lifted, or a model this frontend refuses. Either way the
        // answer is recorded so the next pass costs a map lookup rather than a
        // decode and an allocation.
        _ => Entry {
            block: None,
            seen: reader.seen,
        },
    };
    rt.entries.insert(pc, entry);
}

/// Whether the bytes an entry was built from are still the bytes in memory.
fn valid(entry: &Entry, space: &AddressSpace, cfg: &Config) -> bool {
    let attrs = MemAttrs::DEBUG.with_requester(cfg.requester);
    let mask = cfg.model.address_mask();
    entry.seen.iter().all(|&(addr, word)| {
        let at = u64::from(addr & mask);
        space
            .read(at, Width::U16, attrs)
            .is_ok_and(|v| v as u16 == word)
    })
}

/// The lifter's window onto guest memory, recording what it hands out.
struct Reader<'a> {
    space: &'a AddressSpace,
    attrs: MemAttrs,
    mask: u32,
    seen: Vec<(u32, u16)>,
}

impl lift::InsnSource for Reader<'_> {
    fn word(&mut self, addr: u32) -> Option<u16> {
        // An odd instruction address is an address error rather than a word,
        // and the lifter refuses the block; reading one here would silently
        // give it bytes the processor never fetches.
        if addr & 1 != 0 {
            return None;
        }
        let at = u64::from(addr & self.mask);
        let word = self.space.read(at, Width::U16, self.attrs).ok()? as u16;
        self.seen.push((addr, word));
        Some(word)
    }
}

// The slot numbering, as indices into the host's flat array. Named here rather
// than cast at each use so a slot added to `lift` cannot silently shift one.
const SR_INDEX: usize = 16;
const PC_INDEX: usize = 17;
const P0_INDEX: usize = 18;
const P1_INDEX: usize = 19;

/// What a run left behind.
struct Finished {
    slots: [u32; SLOT_COUNT as usize],
    /// Every tick the run charged.
    used: u64,
    /// The ticks charged when the last boundary was reached — so
    /// `used - retired` is exactly what a partial instruction spent.
    ///
    /// Tracked here rather than read off [`Fault`](crate::ir::Fault), and that
    /// is the fix for a real defect rather than a preference.
    /// `Fault::charged_ticks` and `Fault::retired_ticks` are both measured
    /// from [`Interp`]'s own counter, which sees
    /// [`Opcode::CHARGE`](crate::ir::Opcode::CHARGE) immediates and **not**
    /// the ticks a host charges inside an access. On a core whose every bus
    /// cycle is four host-charged ticks that is most of the count: the first
    /// version of `restart` unwound a faulting `ADD.W (d8,An,Xn),Dn` by the
    /// two internal cycles its `CHARGE` carried and left the eight its two
    /// bus cycles had spent, so the guest's cycle counter ran eight ahead of
    /// the interpreter's for the rest of the run.
    retired: u64,
}

/// The [`IrHost`] a lifted 68000 block runs against.
///
/// It is the whole of the semantics the IR does not carry: the guest register
/// file as a flat slot array, the address space, and the four-cycle charge
/// every bus access makes.
struct Host<'a> {
    state: &'a mut State,
    space: &'a AddressSpace,
    lines: &'a Lines,
    attrs: MemAttrs,
    /// The address pins this model has. A 68000 drives twenty-four, and the
    /// wrap is observable — `exec::bus_addr` masks here and so does this.
    mask: u32,
    slots: [u32; SLOT_COUNT as usize],
    allowance: u64,
    used: u64,
    /// Ticks charged when the last guest instruction boundary was reached.
    retired: u64,
    /// Stores this guest instruction has committed. The restartability budget
    /// is one (module docs).
    stores: u32,
    /// A store landed in the running block's own window, so the block must
    /// leave at the next guest instruction boundary.
    smc: bool,
    /// The interrupt mask the block started with. **S**, **T** and the
    /// interrupt mask cannot change under a lifted block, so this is constant
    /// for the run and is read once rather than out of a slot the block may
    /// have rebound.
    ipl_mask: u8,
    code_window: u32,
}

impl<'a> Host<'a> {
    fn new(
        state: &'a mut State,
        space: &'a AddressSpace,
        cfg: &Config,
        lines: &'a Lines,
        allowance: u64,
        code_window: u32,
    ) -> Host<'a> {
        let mut slots = [0u32; SLOT_COUNT as usize];
        slots[0..8].copy_from_slice(&state.d);
        slots[8..16].copy_from_slice(&state.a);
        slots[SR_INDEX] = u32::from(state.sr);
        slots[PC_INDEX] = state.pc;
        slots[P0_INDEX] = u32::from(state.prefetch[0]);
        slots[P1_INDEX] = u32::from(state.prefetch[1]);
        let ipl_mask = state.ipl_mask();
        // Every access a lifted block makes carries the privilege the core was
        // in when the block started, which is what `Exec::attrs` carries —
        // and it cannot change, because `MOVE to SR` is not lifted.
        let attrs = MemAttrs::DEFAULT
            .with_requester(cfg.requester)
            .with_privileged(state.supervisor());
        Host {
            state,
            space,
            lines,
            attrs,
            mask: cfg.model.address_mask(),
            slots,
            allowance,
            used: 0,
            retired: 0,
            stores: 0,
            smc: false,
            ipl_mask,
            code_window,
        }
    }

    fn finish(self) -> Finished {
        Finished {
            slots: self.slots,
            used: self.used,
            retired: self.retired,
        }
    }

    /// One bus access: four clocks, charged where `Exec::read_byte` and
    /// friends charge them — *after* the alignment check and *before* the
    /// access, so an address error costs nothing and a refused access costs
    /// four.
    fn bus_cycle(&mut self) {
        self.used = self.used.wrapping_add(4);
        self.state.cycles = self.state.cycles.wrapping_add(4);
    }

    /// Whether `addr` is an address error for this access.
    ///
    /// "An odd address is an address error on a 68000 and a 68010"
    /// (`exec::read_word_fc`), for a word or long operand and for every
    /// instruction fetch. A byte access has no alignment rule, which the
    /// frontend says by giving it [`Align::None`].
    const fn misaligned(mem: &MemOp, addr: u32) -> bool {
        matches!(mem.align, Align::Fault) && addr & 1 != 0
    }
}

impl IrHost for Host<'_> {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        u128::from(self.slots.get(slot.0 as usize).copied().unwrap_or(0))
    }

    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        if let Some(s) = self.slots.get_mut(slot.0 as usize) {
            *s = value as u32;
        }
    }

    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        // Computed in the guest's width by the block, masked to the pins here:
        // on a 68000 the wrap at twenty-four bits is observable.
        let addr = addr as u32;
        if Self::misaligned(mem, addr) {
            return Err(BusError::BadAccess);
        }
        self.bus_cycle();
        let at = u64::from(addr & self.mask);
        self.space.read(at, mem.size, self.attrs)
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        let addr = addr as u32;
        if Self::misaligned(mem, addr) {
            return Err(BusError::BadAccess);
        }
        self.stores += 1;
        debug_assert!(
            self.stores <= 1,
            "the m68k frontend lifted an instruction that commits {} stores; a fault after \
             the first cannot be restarted (lift.rs, \"A fault is handled by restarting the \
             instruction\")",
            self.stores
        );
        if addr & !lift::WINDOW_MASK == self.code_window {
            self.smc = true;
        }
        self.bus_cycle();
        let at = u64::from(addr & self.mask);
        self.space.write(at, mem.size, value, self.attrs)
    }

    fn charge(&mut self, ticks: u64) {
        self.used = self.used.wrapping_add(ticks);
        self.state.cycles = self.state.cycles.wrapping_add(ticks);
    }

    fn insn_start(&mut self, _mark: &InsnStart) {
        // Restart granularity is the guest instruction, so the previous one's
        // store stops counting against this one's budget — and the tick mark a
        // restart unwinds back to moves here.
        self.stores = 0;
        self.retired = self.used;
    }

    fn spent(&self) -> bool {
        // Monotone in each term, which is what the seam requires: ticks only
        // rise, the store guard only latches, and an interrupt only becomes
        // pending. `interrupt_pending` is the non-consuming form on purpose —
        // see `Lines::interrupt_pending`.
        self.used >= self.allowance
            || self.smc
            || self.lines.interrupt_pending(self.ipl_mask)
    }
}

#[cfg(test)]
mod tests {
    use super::super::differential::{CODE, Case, DATA, compare, stats_for};
    use super::super::{Engine, M68k};
    use crate::core::props::{Props, Value};
    use alloc::vec;

    /// `STOP #$2700`.
    const STOP: [u16; 2] = [0x4e72, 0x2700];

    #[track_caller]
    fn agreed(case: &Case) {
        if let Err(d) = compare(case) {
            panic!("{d}");
        }
    }

    #[test]
    fn the_property_selects_the_engine_and_the_default_is_the_interpreter() {
        let cpu = M68k::from_props(&Props::new()).expect("an empty property set is a 68000");
        assert_eq!(cpu.engine(), Engine::Interp);
        let props = Props::new().with("engine", Value::Str("ir".into()));
        let cpu = M68k::from_props(&props).expect("`ir` is a 68000 engine");
        assert_eq!(cpu.engine(), Engine::Ir);
        let props = Props::new().with("engine", Value::Str("interp".into()));
        let cpu = M68k::from_props(&props).expect("`interp` still works");
        assert_eq!(cpu.engine(), Engine::Interp);
    }

    #[test]
    fn a_nonsense_engine_is_refused_rather_than_ignored() {
        let props = Props::new().with("engine", Value::Str("jit-host".into()));
        M68k::from_props(&props).expect_err("an engine that does not exist is an error");
    }

    #[test]
    fn a_core_that_never_ran_has_no_statistics_and_one_that_did_has_some() {
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Ir);
        assert_eq!(cpu.ir_stats(), None, "nothing has run yet");
        let stats = stats_for(&Case::seeded(vec![0x4e71, STOP[0], STOP[1]]).with_units(3))
            .expect("a core that ran has statistics");
        assert!(stats.executed > 0, "{stats:?}");
        assert!(stats.retired > 0, "{stats:?}");
    }

    #[test]
    fn a_pending_interrupt_keeps_a_block_from_running_at_any_boundary() {
        // `liftable` refuses a block while one is pending, and `Host::spent`
        // leaves the block at the next boundary if one arrives inside it —
        // because the interpreter checks for one before every instruction, and
        // the two engines have to take it at the same one.
        let program = vec![0x4e71, 0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(4);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Ir);
        cpu.attach_space(space);
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        // An interrupt mask of 3 and a level of 5, so the request is above it.
        regs.sr = super::super::flags::S | 0x0300;
        cpu.set_regs(regs);
        cpu.set_ipl(5);
        let before = cpu.ir_stats().unwrap_or_default();
        cpu.step();
        let after = cpu.ir_stats().unwrap_or_default();
        assert_eq!(
            after.executed, before.executed,
            "no block may run with an interrupt pending: {after:?}"
        );
        assert!(cpu.regs().sr & super::super::flags::IPL == 0x0500, "vectored");
    }

    #[test]
    fn a_budget_no_whole_block_fits_in_still_retires_inside_one() {
        // `Host::spent` is the seam that lets a block leave part-way through,
        // and `run_budget` is what hands it the remaining allowance. Both
        // engines must end the quantum on the same instruction with the same
        // debt, which is a column `compare` checks — this asserts the *shape*:
        // a four-cycle budget over a block of `NOP`s stops after one.
        let program = vec![0x4e71, 0x4e71, 0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(4);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Ir);
        cpu.attach_space(space);
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.run_budget(4);
        assert_eq!(cpu.regs().pc, CODE + 2, "one NOP and no more");
        let stats = cpu.ir_stats().expect("it ran");
        assert_eq!(stats.spent, 1, "the block left at a boundary: {stats:?}");
    }

    #[test]
    fn a_topology_change_throws_the_translations_away() {
        // Derived state, invalidated by the generation counter (CLAUDE.md,
        // "Devices"). A block lifted before a remap must not survive it.
        let program = vec![0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(3);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Ir);
        cpu.attach_space(alloc::sync::Arc::clone(&space));
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.step();
        let before = cpu.ir_stats().expect("it ran").lifted;
        assert!(before > 0);
        // Any topology change moves the generation, whatever it does.
        {
            let mut topo = space.topology();
            topo.rebuild();
        }
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.step();
        let after = cpu.ir_stats().expect("it ran").lifted;
        assert!(after > before, "the cache was not thrown away: {before} {after}");
    }

    #[test]
    fn a_store_the_guest_makes_into_a_translated_window_invalidates_it() {
        // The other half of the self-modifying-code answer: `Host::spent`
        // leaves the block, and the *next* dispatch has to notice the bytes
        // changed rather than serving the stale translation.
        //
        // MOVE.W D0,(A2) with A2 aimed at the instruction after it, run twice
        // round a loop so the second pass is a lookup.
        let program = vec![0x3480, 0x4e71, 0x60fa, STOP[0], STOP[1]];
        let case = Case::seeded(program)
            .with_a(2, CODE + 2)
            .with_d(0, 0x4e71)
            .with_units(12);
        agreed(&case);
        let stats = stats_for(&case).expect("it ran");
        assert!(
            stats.invalidated > 0 || stats.lifted > 1,
            "a rewritten window must not be served from the cache: {stats:?}"
        );
    }

    #[test]
    fn a_reset_throws_the_translations_away_and_the_run_still_agrees() {
        let program = vec![0x4e71, 0x4e71, STOP[0], STOP[1]];
        let case = Case::seeded(program).with_units(3);
        let (space, _ram) = super::super::differential::space_for(&case);
        let cpu = M68k::new(super::super::Config::default()).with_engine(Engine::Ir);
        cpu.attach_space(space);
        cpu.step();
        let mut regs = cpu.regs();
        regs.pc = CODE;
        regs.prefetch = [0x4e71, 0x4e71];
        cpu.set_regs(regs);
        cpu.step();
        cpu.request_reset();
        cpu.step();
        assert_eq!(cpu.regs().pc, CODE, "the reset vector points at the code");
    }

    #[test]
    fn a_data_access_reaches_the_pins_the_model_has() {
        // A 68000 drives twenty-four address pins and the wrap is observable:
        // `Host::load` masks with `Model::address_mask` exactly as
        // `exec::bus_addr` does. Reaching `DATA` through an address above the
        // pins must land on `DATA`.
        let case = Case::seeded(vec![0xd052, STOP[0], STOP[1]])
            .with_a(2, 0xff00_0000 | DATA)
            .with_units(2);
        agreed(&case);
    }
}
