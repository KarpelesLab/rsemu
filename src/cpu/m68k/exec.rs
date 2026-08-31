//! The MC68000 interpreter.
//!
//! # One bus access is four cycles, and nothing else costs anything
//!
//! A 68000 bus cycle is four clocks (MC68000UM §5.1, *Data Transfer
//! Operations*), and every published instruction time is a sum of bus cycles
//! and internal cycles. So this interpreter has no per-instruction cycle
//! table: [`Exec::read_word`] and friends charge four each, and
//! [`Exec::internal`] charges the microcode idle time the manual's section 8
//! tables call for, at the point in the instruction where it happens. A
//! `MOVE.B (d16,An),(d16,An)` costs 20 because it makes five bus accesses,
//! not because a table says 20.
//!
//! # The prefetch queue is real state
//!
//! The 68000 keeps two instruction words on hand and refills them one bus
//! cycle at a time. That is observable — it is why `MOVE.W <mem>,($xxxxxxxx).L`
//! writes its operand *before* its last instruction fetch while the same move
//! from a register does not, and why the program counter an address-error
//! frame pushes is not the address of the faulting instruction — so it is
//! modelled explicitly rather than approximated by "fetch the whole
//! instruction, then execute it".
//!
//! The invariant is one line:
//!
//! > [`State::prefetch`]`[0]` is the word at [`State::pc`], and `prefetch[1]`
//! > is the word at `pc + 2`.
//!
//! Executing an instruction *slides* the queue once per instruction word:
//! [`Exec::slide`] shifts `prefetch[1]` down, reads a fresh word from
//! `pc + 4`, and advances `pc` by two. An extension word is therefore taken
//! from `prefetch[1]` (which is where the assembler put it) and the slide that
//! follows is the bus cycle the manual counts against that operand. The last
//! slide of an instruction is what leaves `prefetch[0]` holding the *next*
//! opcode.
//!
//! Because `pc` only moves when a slide completes, the value an exception
//! frame pushes is exactly the hardware's — which is the whole reason to model
//! the queue rather than a byte cursor.
//!
//! # Sources
//!
//! *M68000 Family Programmer's Reference Manual* (M68000PRM) for every
//! instruction's operation and condition-code rules — the per-instruction
//! pages, which are the only place the irregular rules are stated. The
//! *MC68000 User's Manual* (MC68000UM) §6 for exception processing and the two
//! stack-frame formats, and §8 for instruction timing. `docs/cpu/other.md`
//! records where to find both. No copyleft emulator was consulted.

use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

use super::isa::{Arg, Cond, Insn, Mode, Op, Size, decode, ea_of};
use super::{ADDRESS_MASK, Config, Lines, flags, vector};

/// Function codes, as they appear on FC0–FC2 and in a group-0 stack frame's
/// special status word (MC68000UM §3.1.1).
mod fc {
    /// User data space.
    pub(super) const USER_DATA: u8 = 1;
    /// User program space.
    pub(super) const USER_PROGRAM: u8 = 2;
    /// Supervisor data space.
    pub(super) const SUPER_DATA: u8 = 5;
    /// Supervisor program space.
    pub(super) const SUPER_PROGRAM: u8 = 6;
}

/// Everything one core owns, minus the interrupt pins.
///
/// Split from [`super::M68k`] because the pins live outside the execution
/// lock: a device asserting an interrupt from inside a CPU-initiated write
/// would otherwise re-enter the CPU's own critical section and deadlock (the
/// re-entrancy contract, `ROADMAP.md` §4.7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct State {
    /// The eight data registers.
    pub d: [u32; 8],
    /// The eight address registers. `a[7]` is whichever stack pointer the
    /// current privilege state selects.
    pub a: [u32; 8],
    /// The stack pointer that is *not* currently `a[7]`.
    ///
    /// One register file with a shadow, rather than two named pointers, so
    /// every `a[reg]` access in the interpreter stays a plain array index and
    /// the bank swap happens in exactly one place ([`State::set_sr`]).
    pub other_sp: u32,
    /// The program counter: the address of `prefetch[0]`.
    pub pc: u32,
    /// The status register. See [`super::flags`].
    pub sr: u16,
    /// The two-word instruction prefetch queue.
    pub prefetch: [u16; 2],
    /// Bus and internal cycles since power-on.
    pub cycles: u64,
    /// A double bus fault stopped the processor; only a reset restarts it.
    pub halted: bool,
    /// `STOP` was executed and no interrupt has arrived yet.
    pub stopped: bool,
    /// A reset was requested and its sequence has not run yet.
    pub reset_pending: bool,
    /// How many accesses the address space refused.
    pub faults: u64,
    /// Address of the most recent refused access.
    pub last_fault: u32,
}

impl State {
    /// Power-on state, before the reset sequence has run.
    pub(super) const fn new() -> State {
        State {
            d: [0; 8],
            a: [0; 8],
            other_sp: 0,
            pc: 0,
            // Supervisor state with every interrupt masked, which is what a
            // reset leaves behind (MC68000UM §6.2.6).
            sr: flags::S | flags::IPL,
            prefetch: [0; 2],
            cycles: 0,
            halted: false,
            stopped: false,
            reset_pending: true,
            faults: 0,
            last_fault: 0,
        }
    }

    /// Whether the core is in supervisor state.
    #[inline]
    pub(super) const fn supervisor(&self) -> bool {
        self.sr & flags::S != 0
    }

    /// The user stack pointer, whichever bank it is in.
    #[must_use]
    pub(super) const fn usp(&self) -> u32 {
        if self.supervisor() {
            self.other_sp
        } else {
            self.a[7]
        }
    }

    /// The supervisor stack pointer, whichever bank it is in.
    #[must_use]
    pub(super) const fn ssp(&self) -> u32 {
        if self.supervisor() {
            self.a[7]
        } else {
            self.other_sp
        }
    }

    /// Overwrite the user stack pointer.
    pub(super) const fn set_usp(&mut self, value: u32) {
        if self.supervisor() {
            self.other_sp = value;
        } else {
            self.a[7] = value;
        }
    }

    /// Write the status register, swapping stack pointers if **S** changed.
    ///
    /// The bank swap is the whole reason this is a method: `A7` names a
    /// different physical register in the two states, and a `MOVE to SR` that
    /// left the old one in place is the classic supervisor-mode bug
    /// (M68000PRM, *MOVE to SR*).
    pub(super) const fn set_sr(&mut self, value: u16) {
        let value = value & flags::IMPLEMENTED;
        if (value & flags::S) != (self.sr & flags::S) {
            let active = self.a[7];
            self.a[7] = self.other_sp;
            self.other_sp = active;
        }
        self.sr = value;
    }

    /// The condition code register: the low byte of `SR`.
    #[inline]
    pub(super) const fn ccr(&self) -> u8 {
        (self.sr & flags::CCR) as u8
    }

    /// Whether a status flag is set.
    #[inline]
    pub(super) const fn flag(&self, mask: u16) -> bool {
        self.sr & mask != 0
    }

    /// The interrupt priority mask, 0–7.
    #[inline]
    pub(super) const fn ipl_mask(&self) -> u8 {
        ((self.sr & flags::IPL) >> 8) as u8
    }

    /// Evaluate one of the sixteen condition codes against the current flags.
    ///
    /// M68000PRM §3.2, *Condition Tests*. Written out rather than derived from
    /// a formula because the manual writes it out: `GT` is
    /// `N·V·Z̄ + N̄·V̄·Z̄`, and any "simplification" of that is where the bugs
    /// live.
    #[must_use]
    pub(super) const fn test(&self, cond: Cond) -> bool {
        let c = self.flag(flags::C);
        let v = self.flag(flags::V);
        let z = self.flag(flags::Z);
        let n = self.flag(flags::N);
        match cond.0 {
            0x0 => true,
            0x1 => false,
            0x2 => !c && !z,
            0x3 => c || z,
            0x4 => !c,
            0x5 => c,
            0x6 => !z,
            0x7 => z,
            0x8 => !v,
            0x9 => v,
            0xa => !n,
            0xb => n,
            0xc => n == v,
            0xd => n != v,
            0xe => !z && (n == v),
            _ => z || (n != v),
        }
    }
}

/// Why an instruction stopped early.
///
/// Every 68000 exception aborts the instruction that raised it, so the
/// interpreter's operand helpers return `Result<_, Trap>` and the instruction
/// body is written with `?`. The alternative — a status flag checked after
/// every access — is how a half-completed instruction gets committed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Trap {
    /// A word or long access to an odd address, or an instruction fetch from
    /// one. Group 0: a fourteen-byte frame.
    Address {
        /// The address the instruction tried to reach, untruncated.
        addr: u32,
        /// Whether the access was a read.
        read: bool,
        /// The function code that would have been driven.
        fc: u8,
    },
    /// The address space refused the access. Group 0, same frame shape.
    Bus {
        /// The address the instruction tried to reach, untruncated.
        addr: u32,
        /// Whether the access was a read.
        read: bool,
        /// The function code that would have been driven.
        fc: u8,
    },
    /// An ordinary vectored exception with the six-byte frame.
    Vectored {
        /// The vector number.
        vector: u8,
        /// The program counter to push.
        pc: u32,
    },
}

impl Trap {
    /// A group-1 or group-2 exception through `vector`, pushing `pc`.
    const fn at(vector: u8, pc: u32) -> Trap {
        Trap::Vectored { vector, pc }
    }
}

/// A resolved operand: where a value is, not what it is.
///
/// Resolving separately from reading is what makes a read-modify-write
/// instruction address memory once — `ADDQ #1,(A0)+` increments `A0` once, not
/// twice — and what lets `MOVE` compute its destination address before the
/// write without duplicating the addressing-mode logic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Loc {
    /// Data register `n`.
    D(u8),
    /// Address register `n`.
    A(u8),
    /// A memory location.
    Mem(u32),
    /// A value with no home: an immediate, or a field extracted from the
    /// opcode.
    Value(u32),
    /// A memory location whose current contents have already been read.
    ///
    /// `ADDX`, `SUBX`, `ABCD` and `SBCD` fetch their destination as part of
    /// the predecrement walk that computes its address, so re-reading it would
    /// put a bus cycle on the wire that hardware does not.
    Prefetched(u32, u32),
}

/// One instruction's worth of execution, borrowing everything it needs.
pub(super) struct Exec<'a> {
    state: &'a mut State,
    space: &'a AddressSpace,
    cfg: &'a Config,
    lines: &'a Lines,
    /// The opcode word being executed, kept for a group-0 frame's instruction
    /// register field.
    opcode: u16,
    /// Cycles this step has charged.
    used: u64,
    /// Whether the source operand of the `MOVE` in progress came from memory.
    source_was_memory: bool,
    /// A `MOVE` destination's postincrement, owed once its write lands.
    deferred_postincrement: Option<(u8, u32)>,
    /// Internal cycles the next exception spends before it pushes anything.
    prologue: u32,
    /// Slides an instruction deferred past its operand write.
    ///
    /// `MOVE <ea>,(xxx).L` performs its write *before* the last instruction
    /// fetch, which is visible in the program counter an address-error frame
    /// pushes. Rather than special-case that in three places, the destination
    /// resolver records the debt and [`Exec::settle`] pays it.
    deferred_slides: u32,
}

impl<'a> Exec<'a> {
    /// Borrow a core for one step.
    pub(super) fn new(
        state: &'a mut State,
        space: &'a AddressSpace,
        cfg: &'a Config,
        lines: &'a Lines,
    ) -> Exec<'a> {
        Exec {
            state,
            space,
            cfg,
            lines,
            opcode: 0,
            used: 0,
            prologue: 4,
            source_was_memory: false,
            deferred_postincrement: None,
            deferred_slides: 0,
        }
    }

    /// Run one reset sequence, exception sequence, or instruction.
    ///
    /// Returns the cycles charged; zero only when the core is halted, which a
    /// scheduler must notice rather than spin on.
    pub(super) fn step(&mut self) -> u64 {
        if self.state.reset_pending {
            self.reset_sequence();
            return self.used;
        }
        if self.state.halted {
            return 0;
        }
        if let Some(level) = self.pending_interrupt() {
            self.state.stopped = false;
            self.take_interrupt(level);
            return self.used;
        }
        if self.state.stopped {
            // A stopped 68000 still drives the bus refresh; nothing is
            // fetched, so charge the four cycles a bus cycle would have taken
            // and let the scheduler move time forward.
            self.internal(4);
            return self.used;
        }
        self.instruction();
        self.used
    }

    // ------------------------------------------------------------------
    // The clock
    // ------------------------------------------------------------------

    /// Charge internal (non-bus) cycles.
    fn internal(&mut self, cycles: u32) {
        // Wrapping on both, deliberately: a cycle counter that panics after
        // 2^64 clocks would be a worse failure than one that wraps, and a
        // debug build must not behave differently from a release one.
        self.used = self.used.wrapping_add(u64::from(cycles));
        self.state.cycles = self.state.cycles.wrapping_add(u64::from(cycles));
    }

    /// The attributes every access this core makes carries.
    fn attrs(&self) -> MemAttrs {
        MemAttrs::DEFAULT
            .with_requester(self.cfg.requester)
            .with_privileged(self.state.supervisor())
    }

    /// The function code for a data access in the current privilege state.
    fn data_fc(&self) -> u8 {
        if self.state.supervisor() {
            fc::SUPER_DATA
        } else {
            fc::USER_DATA
        }
    }

    /// The function code for an instruction fetch.
    fn program_fc(&self) -> u8 {
        if self.state.supervisor() {
            fc::SUPER_PROGRAM
        } else {
            fc::USER_PROGRAM
        }
    }

    /// One byte read. Byte accesses have no alignment rule.
    fn read_byte(&mut self, addr: u32) -> Result<u8, Trap> {
        self.internal(4);
        let fc = self.data_fc();
        match self
            .space
            .read(u64::from(addr & ADDRESS_MASK), Width::U8, self.attrs())
        {
            Ok(v) => Ok(v as u8),
            Err(_) => Err(self.bus_fault(addr, true, fc)),
        }
    }

    /// One word read, faulting on an odd address.
    fn read_word(&mut self, addr: u32) -> Result<u16, Trap> {
        let fc = self.data_fc();
        self.read_word_fc(addr, fc)
    }

    /// One word read with an explicit function code, so an instruction fetch
    /// can report itself as program space in a group-0 frame.
    fn read_word_fc(&mut self, addr: u32, fc: u8) -> Result<u16, Trap> {
        if addr & 1 != 0 {
            return Err(Trap::Address {
                addr,
                read: true,
                fc,
            });
        }
        self.internal(4);
        match self
            .space
            .read(u64::from(addr & ADDRESS_MASK), Width::U16, self.attrs())
        {
            Ok(v) => Ok(v as u16),
            Err(_) => Err(self.bus_fault(addr, true, fc)),
        }
    }

    /// One long read: two word accesses, high word first.
    ///
    /// The 68000 has a 16-bit data bus, so there is no such thing as a 32-bit
    /// bus cycle; a device watching the bus sees two.
    fn read_long(&mut self, addr: u32) -> Result<u32, Trap> {
        let hi = self.read_word(addr)?;
        let lo = self.read_word(addr.wrapping_add(2))?;
        Ok((u32::from(hi) << 16) | u32::from(lo))
    }

    /// One byte write.
    fn write_byte(&mut self, addr: u32, value: u8) -> Result<(), Trap> {
        self.internal(4);
        let fc = self.data_fc();
        match self.space.write(
            u64::from(addr & ADDRESS_MASK),
            Width::U8,
            u64::from(value),
            self.attrs(),
        ) {
            Ok(()) => Ok(()),
            Err(_) => Err(self.bus_fault(addr, false, fc)),
        }
    }

    /// One word write, faulting on an odd address.
    fn write_word(&mut self, addr: u32, value: u16) -> Result<(), Trap> {
        let fc = self.data_fc();
        if addr & 1 != 0 {
            return Err(Trap::Address {
                addr,
                read: false,
                fc,
            });
        }
        self.internal(4);
        match self.space.write(
            u64::from(addr & ADDRESS_MASK),
            Width::U16,
            u64::from(value),
            self.attrs(),
        ) {
            Ok(()) => Ok(()),
            Err(_) => Err(self.bus_fault(addr, false, fc)),
        }
    }

    /// One long write: two word accesses, high word first.
    fn write_long(&mut self, addr: u32, value: u32) -> Result<(), Trap> {
        self.write_word(addr, (value >> 16) as u16)?;
        self.write_word(addr.wrapping_add(2), value as u16)
    }

    /// A long write that puts the **low** word out first.
    ///
    /// Two cases need it, for the same reason: the word the datapath is
    /// already holding goes out first. `MOVE.L <ea>,-(An)` has decremented the
    /// register by four and writes the low half before fetching the other one,
    /// and every read-modify-write — `ADD.L D0,(A0)`, `CLR.L`, `NOT.L`, a
    /// memory shift — finished its read with the low word and writes that
    /// back first. It is visible on the bus, so a device with side effects can
    /// tell, and it is not an implementation detail we get to choose.
    fn write_long_low_first(&mut self, addr: u32, value: u32) -> Result<(), Trap> {
        self.write_word(addr.wrapping_add(2), value as u16)?;
        self.write_word(addr, (value >> 16) as u16)
    }

    /// Record a refused access and turn it into a bus error.
    ///
    /// **Known hazard**, and the same one `ROADMAP.md` §4.1 records against
    /// the 6502 core: an access that meets a retopology in flight comes back
    /// as [`BusError::Retry`](crate::core::error::BusError::Retry), meaning
    /// "nothing happened, reissue" rather than "the hardware refused". This
    /// core cannot reissue — it holds `BUS` across the access and the
    /// retopology takes `BUS` underneath `TOPOLOGY`, so spinning here closes a
    /// deadlock cycle — so a retry becomes a bus-error exception, which is
    /// guest-visible and depends on host timing. The safe-point protocol
    /// (§4.7) is what makes it unreachable, and it does not exist yet. Until
    /// then a machine that remaps a space under a running CPU can see a
    /// spurious vector-2 exception, and `bus_faults` is where it shows up.
    fn bus_fault(&mut self, addr: u32, read: bool, fc: u8) -> Trap {
        self.state.faults = self.state.faults.wrapping_add(1);
        self.state.last_fault = addr;
        Trap::Bus { addr, read, fc }
    }

    // ------------------------------------------------------------------
    // The prefetch queue
    // ------------------------------------------------------------------

    /// Slide the queue one word: shift, refill from `pc + 4`, advance `pc`.
    ///
    /// This is the only place `pc` moves during an instruction, which is what
    /// makes the value an address-error frame pushes correct.
    fn slide(&mut self) -> Result<(), Trap> {
        let fetch = self.state.pc.wrapping_add(4);
        let fc = self.program_fc();
        let word = self.read_word_fc(fetch, fc)?;
        self.state.prefetch[0] = self.state.prefetch[1];
        self.state.prefetch[1] = word;
        self.state.pc = self.state.pc.wrapping_add(2);
        Ok(())
    }

    /// Take the next extension word, charging `delay` internal cycles between
    /// reading it and the refill it causes.
    ///
    /// The delay is where an indexed mode's two internal cycles go: the
    /// manual's timing puts them before the refill, and a bus trace shows them
    /// there (MC68000UM Table 8-1).
    fn ext(&mut self, delay: u32) -> Result<u16, Trap> {
        let word = self.state.prefetch[1];
        if delay != 0 {
            self.internal(delay);
        }
        self.slide()?;
        Ok(word)
    }

    /// Take the next extension word but leave its refill for [`Exec::settle`].
    fn ext_deferred(&mut self) -> u16 {
        let word = self.state.prefetch[1];
        self.deferred_slides += 1;
        word
    }

    /// Pay off any deferred refill, then perform the instruction's final
    /// slide, which is what leaves the next opcode in `prefetch[0]`.
    fn settle(&mut self) -> Result<(), Trap> {
        while self.deferred_slides > 0 {
            self.deferred_slides -= 1;
            self.slide()?;
        }
        self.slide()
    }

    /// Reload both prefetch words from `target` — a branch, a jump, a return,
    /// or the last step of exception processing.
    ///
    /// `gap` is internal time spent between the two fetches. A branch has
    /// none; exception processing has two cycles there, and they are in every
    /// published exception time.
    ///
    /// The program counter moves to `target - 4` *before* the first fetch and
    /// steps as each word lands, because the queue invariant is that `pc`
    /// addresses `prefetch[0]` and neither word has arrived yet. That is not
    /// bookkeeping for its own sake: a branch to an odd address takes an
    /// address error here, and the frame it pushes carries exactly this value.
    fn refill(&mut self, target: u32, gap: u32) -> Result<(), Trap> {
        self.deferred_slides = 0;
        let fc = self.program_fc();
        self.state.pc = target.wrapping_sub(4);
        let first = self.read_word_fc(target, fc)?;
        self.state.pc = target.wrapping_sub(2);
        if gap != 0 {
            self.internal(gap);
        }
        let second = self.read_word_fc(target.wrapping_add(2), fc)?;
        self.state.pc = target;
        self.state.prefetch = [first, second];
        Ok(())
    }

    // ------------------------------------------------------------------
    // Reset, interrupts and exceptions
    // ------------------------------------------------------------------

    /// The reset sequence: supervisor state, interrupts masked, `SSP` and `PC`
    /// from vectors 0 and 1 (MC68000UM §6.2.6).
    fn reset_sequence(&mut self) {
        self.state.reset_pending = false;
        self.state.halted = false;
        self.state.stopped = false;
        // Through set_sr, not by assignment: a reset taken in user state has
        // to swap the stack-pointer banks before vector 0 is loaded, or the
        // supervisor stack pointer lands in the user bank and the user one is
        // silently lost.
        self.state.set_sr(flags::S | flags::IPL);
        self.internal(4);
        let outcome = (|| -> Result<(), Trap> {
            let ssp = self.read_long(0)?;
            let pc = self.read_long(4)?;
            self.state.a[7] = ssp;
            self.refill(pc, 0)
        })();
        if outcome.is_err() {
            // Nothing can be done about a reset vector that cannot be read.
            self.state.halted = true;
        }
    }

    /// The interrupt level that should be taken now, if any.
    ///
    /// Levels one to six are level-sensitive and are taken while they exceed
    /// the mask in `SR`. Level seven is the non-maskable one, and it is
    /// **edge-triggered**: the 68000 recognises a *transition* to seven, not
    /// the level itself, so a source that holds the pins at seven interrupts
    /// once rather than forever (MC68000UM §6.3.2). Treating it as
    /// level-sensitive is the classic way to make a machine with a wired
    /// non-maskable button lock up the moment it is pressed.
    fn pending_interrupt(&self) -> Option<u8> {
        // The latch is consumed whatever the pins say *now*. A step can cover
        // many clocks, so a source that pulses level seven and lets go is the
        // normal case rather than a race; checking the current level first
        // would drop the edge and then deliver it at some unrelated later
        // moment when the pins happened to read seven again.
        if self.lines.take_level_seven() {
            return Some(7);
        }
        let level = self.lines.ipl();
        if level != 0 && level > self.state.ipl_mask() {
            Some(level)
        } else {
            None
        }
    }

    /// Acknowledge and vector an interrupt.
    ///
    /// # The acknowledge cycle is timed but not driven
    ///
    /// Hardware runs an interrupt-acknowledge bus cycle here — function code
    /// 7, CPU space, the level on A3–A1 — and either a device answers with a
    /// vector number or `VPA` asks for the autovector. This core charges that
    /// cycle's four clocks (the published time is 44 cycles, five reads and
    /// three writes: the fifth read is the acknowledge) but does not put it on
    /// the bus, because CPU space is a function code and `MemAttrs` does not
    /// carry one. A controller therefore supplies its vector through
    /// [`M68k::set_interrupt_vector`](super::M68k::set_interrupt_vector)
    /// instead of answering an access, and the vector is *consumed*, so the
    /// next acknowledge autovectors again unless the controller arms another.
    fn take_interrupt(&mut self, level: u8) {
        let vector = self
            .lines
            .take_vector()
            .unwrap_or(vector::AUTOVECTOR_BASE.wrapping_add(level));
        let pc = self.state.pc;
        let sr = self.state.sr;
        // The mask rises to the level being serviced, so the handler is not
        // immediately re-entered by its own source.
        let raised = (sr & !flags::IPL) | (u16::from(level) << 8);
        // Ten cycles more prologue than any other exception: four of them are
        // the acknowledge cycle above, and the rest is the microcode deciding
        // what to do with the answer (MC68000UM Table 8-14, 44(5/3)).
        self.prologue = 14;
        self.enter_exception(vector, pc, raised, None);
    }

    /// Perform exception processing for a group-1 or group-2 exception.
    ///
    /// `new_sr` is the status register the handler starts with, before **S**
    /// is forced and **T** cleared; passing it in is how an interrupt raises
    /// the mask and everything else does not.
    fn enter_exception(&mut self, vector: u8, pc: u32, new_sr: u16, group0: Option<Group0>) {
        let saved_sr = self.state.sr;
        // Supervisor state, tracing off. The status register pushed is the one
        // from *before* this (MC68000UM §6.2).
        self.state.set_sr((new_sr | flags::S) & !flags::T);
        // Any exception resumes a stopped processor, including the trace
        // exception a `STOP` executed with T set leaves behind.
        self.state.stopped = false;
        // Four cycles of deciding what to do, for most exceptions. `TRAPV`
        // spends none — it already knew — `CHK` spends two more unless the
        // bound test is what failed, and an interrupt spends ten more because
        // it has an acknowledge cycle to run first.
        self.internal(self.prologue);
        let outcome = (|| -> Result<(), Trap> {
            let mut sp = self.state.a[7];
            // The 68000 writes the frame in this order, which is neither
            // ascending nor descending; it is visible on the bus.
            sp = sp.wrapping_sub(2);
            self.write_word(sp, pc as u16)?;
            sp = sp.wrapping_sub(4);
            self.write_word(sp, saved_sr)?;
            self.write_word(sp.wrapping_add(2), (pc >> 16) as u16)?;
            if let Some(g0) = group0 {
                sp = sp.wrapping_sub(2);
                self.write_word(sp, g0.ir)?;
                sp = sp.wrapping_sub(2);
                self.write_word(sp, g0.addr as u16)?;
                sp = sp.wrapping_sub(4);
                self.write_word(sp, g0.ssw)?;
                self.write_word(sp.wrapping_add(2), (g0.addr >> 16) as u16)?;
            }
            self.state.a[7] = sp;
            let base = u32::from(vector) * 4;
            let target = self.read_long(base)?;
            self.refill(target, 2)
        })();
        if outcome.is_err() {
            // A fault while taking an exception is the double bus fault: the
            // 68000 asserts HALT and stops until reset (MC68000UM §6.2.5).
            self.state.halted = true;
        }
    }

    /// The special status word a group-0 frame carries.
    ///
    /// Bits 2–0 are the function code the failed access drove, bit 3 is the
    /// instruction/not bit and bit 4 is read/write. The remaining eleven bits
    /// are documented as undefined; the hardware leaves the instruction
    /// register's bits there, and the corpus this core is measured against
    /// expects them, so they are reproduced rather than zeroed
    /// (MC68000UM §6.3.9).
    ///
    /// Bit 3 is set for a *program-space* access and clear for a data one,
    /// which is worth spelling out because the manual's name for it invites
    /// the opposite reading. "I/N" is instruction / **not** — meaning "was the
    /// processor in the middle of executing an instruction (0) or between
    /// instructions (1)" — and an instruction fetch is by definition the
    /// latter, while the data access an instruction makes is the former. The
    /// polarity here is the one the conformance corpus measures on every
    /// address error it contains.
    fn special_status(&self, read: bool, fc: u8) -> u16 {
        let not_instruction = fc == self.program_fc();
        (self.opcode & !0x001f)
            | (u16::from(read) << 4)
            | (u16::from(not_instruction) << 3)
            | u16::from(fc & 7)
    }

    /// Turn a [`Trap`] into exception processing.
    fn service(&mut self, trap: Trap) {
        match trap {
            Trap::Address { addr, read, fc } | Trap::Bus { addr, read, fc } => {
                let vector = if matches!(trap, Trap::Address { .. }) {
                    vector::ADDRESS_ERROR
                } else {
                    vector::BUS_ERROR
                };
                let ssw = self.special_status(read, fc);
                let g0 = Group0 {
                    ssw,
                    addr,
                    ir: self.opcode,
                };
                let pc = self.state.pc;
                let sr = self.state.sr;
                self.enter_exception(vector, pc, sr, Some(g0));
            }
            Trap::Vectored { vector, pc } => {
                let sr = self.state.sr;
                self.enter_exception(vector, pc, sr, None);
            }
        }
    }

    // ------------------------------------------------------------------
    // Instruction dispatch
    // ------------------------------------------------------------------

    /// Fetch, decode and execute one instruction, then service any exception
    /// it raised and any pending trace.
    fn instruction(&mut self) {
        self.opcode = self.state.prefetch[0];
        let pc0 = self.state.pc;
        let traced = self.state.flag(flags::T);
        let insn = decode(self.opcode);

        let outcome = if insn.privileged && !self.state.supervisor() {
            // A privilege violation is detected before anything is fetched, so
            // the pushed program counter is the instruction's own.
            Err(Trap::at(vector::PRIVILEGE, pc0))
        } else {
            self.execute(insn, pc0)
        };

        match outcome {
            Ok(()) => {
                if traced {
                    let pc = self.state.pc;
                    self.service(Trap::at(vector::TRACE, pc));
                }
            }
            Err(trap) => self.service(trap),
        }
    }

    /// Execute one decoded instruction.
    ///
    /// `pc0` is the address of the opcode word, which several exceptions push.
    #[allow(clippy::too_many_lines)]
    fn execute(&mut self, insn: Insn, pc0: u32) -> Result<(), Trap> {
        let opcode = self.opcode;
        let size = match insn.size.resolve(opcode) {
            Some(size) => size,
            // decode() already turned this into Op::Illegal; the row it
            // returned carries no size, so any value will do.
            None => Size::Word,
        };
        match insn.op {
            Op::Illegal => Err(Trap::at(vector::ILLEGAL, pc0)),
            Op::LineA => Err(Trap::at(vector::LINE_A, pc0)),
            Op::LineF => Err(Trap::at(vector::LINE_F, pc0)),
            Op::Nop => self.settle(),
            Op::Reset => {
                // 124 clocks of RESET asserted, then the final prefetch.
                self.internal(128);
                self.lines.pulse_reset();
                self.settle()
            }
            Op::Stop => {
                let word = self.ext(0)?;
                self.state.set_sr(word);
                // The queue is settled *before* stopping, so the program
                // counter an arriving interrupt pushes is the instruction
                // after this one. A stopped 68000 cannot fetch, so hardware
                // makes these two bus cycles on the way out instead of the way
                // in; the state either way is the same, and the alternative is
                // an RTE that returns into the middle of the STOP.
                self.settle()?;
                self.state.stopped = true;
                Ok(())
            }

            Op::Move | Op::Movea => self.op_move(insn, size),
            Op::Moveq => {
                let value = i32::from(opcode as i8) as u32;
                self.state.d[reg_hi(opcode)] = value;
                self.set_logic_flags(value, Size::Long);
                self.settle()
            }

            Op::Add | Op::Addi | Op::Addq => self.op_binary(insn, size, BinOp::Add),
            Op::Sub | Op::Subi | Op::Subq => self.op_binary(insn, size, BinOp::Sub),
            Op::And | Op::Andi => self.op_binary(insn, size, BinOp::And),
            Op::Or | Op::Ori => self.op_binary(insn, size, BinOp::Or),
            Op::Eor | Op::Eori => self.op_binary(insn, size, BinOp::Eor),
            Op::Cmp | Op::Cmpi => self.op_compare(insn, size),
            Op::Cmpm => self.op_cmpm(size),
            Op::Adda | Op::Suba => self.op_adda(insn, size),
            Op::Cmpa => self.op_cmpa(size),
            Op::Addx | Op::Subx => self.op_addx(insn, size),
            Op::Abcd | Op::Sbcd => self.op_bcd(insn),
            Op::Nbcd => self.op_nbcd(),

            Op::Neg | Op::Negx | Op::Not | Op::Clr => self.op_unary(insn, size),
            Op::Tst => self.op_tst(size),
            Op::Tas => self.op_tas(),
            Op::Ext => {
                let n = reg_lo(opcode);
                let value = if size == Size::Long {
                    i32::from(self.state.d[n] as i16) as u32
                } else {
                    let byte = self.state.d[n] as i8;
                    (self.state.d[n] & 0xffff_0000) | u32::from(byte as u16)
                };
                self.state.d[n] = value;
                self.set_logic_flags(value, size);
                self.settle()
            }
            Op::Swap => {
                let n = reg_lo(opcode);
                let value = self.state.d[n].rotate_left(16);
                self.state.d[n] = value;
                self.set_logic_flags(value, Size::Long);
                self.settle()
            }
            Op::Exg => self.op_exg(insn),

            Op::Muls | Op::Mulu => self.op_mul(insn),
            Op::Divs | Op::Divu => self.op_div(insn, pc0),
            Op::Chk => self.op_chk(),

            Op::Btst | Op::Bchg | Op::Bclr | Op::Bset => self.op_bit(insn, size),
            Op::Asl | Op::Asr | Op::Lsl | Op::Lsr | Op::Rol | Op::Ror | Op::Roxl | Op::Roxr => {
                self.op_shift(insn, size)
            }

            Op::Lea => {
                let Loc::Mem(addr) = self.resolve_control(Arg::Ea, ExtraCycles::Control)? else {
                    return Err(Trap::at(vector::ILLEGAL, pc0));
                };
                self.state.a[reg_hi(opcode)] = addr;
                self.settle()
            }
            Op::Pea => self.op_pea(),
            Op::Jmp => {
                let target = self.jump_target()?;
                self.refill(target, 0)
            }
            Op::Jsr => self.op_jsr(pc0),
            Op::Bra | Op::Bsr | Op::Bcc => self.op_branch(insn),
            Op::Dbcc => self.op_dbcc(),
            Op::Scc => self.op_scc(),
            Op::Rts => {
                let sp = self.state.a[7];
                let target = self.read_long(sp)?;
                self.state.a[7] = sp.wrapping_add(4);
                self.refill(target, 0)
            }
            // RTE and RTR read their six-byte frame in an order that is
            // neither ascending nor a long access: the high half of the
            // program counter, then the status word, then the low half. The
            // microcode is holding the status word's slot open while it
            // decides what privilege state to return to.
            Op::Rtr => {
                let sp = self.state.a[7];
                let high = self.read_word(sp.wrapping_add(2))?;
                let ccr = self.read_word(sp)?;
                let low = self.read_word(sp.wrapping_add(4))?;
                self.state.a[7] = sp.wrapping_add(6);
                let sr = (self.state.sr & !flags::CCR) | (ccr & flags::CCR);
                self.state.set_sr(sr);
                self.refill((u32::from(high) << 16) | u32::from(low), 0)
            }
            Op::Rte => {
                let sp = self.state.a[7];
                let high = self.read_word(sp.wrapping_add(2))?;
                let sr = self.read_word(sp)?;
                let low = self.read_word(sp.wrapping_add(4))?;
                self.state.a[7] = sp.wrapping_add(6);
                self.state.set_sr(sr);
                self.refill((u32::from(high) << 16) | u32::from(low), 0)
            }
            Op::Trap => {
                let n = (opcode & 0xf) as u8;
                // A trap pushes the address of the *next* instruction. No
                // prefetch happens first, so that address is computed rather
                // than reached by sliding the queue.
                let next = self.state.pc.wrapping_add(2);
                Err(Trap::at(vector::TRAP_BASE.wrapping_add(n), next))
            }
            Op::Trapv => {
                // Unlike TRAP, TRAPV finishes its prefetch first, so the
                // address it pushes is reached rather than computed.
                self.settle()?;
                if self.state.flag(flags::V) {
                    self.prologue = 0;
                    let pc = self.state.pc;
                    return Err(Trap::at(vector::TRAPV, pc));
                }
                Ok(())
            }
            Op::Link => self.op_link(),
            Op::Unlk => {
                let n = reg_lo(opcode);
                let frame = self.state.a[n];
                let saved = self.read_long(frame)?;
                // The stack pointer is restored first and the register second,
                // so `UNLK A7` ends up holding the popped value rather than
                // the frame pointer plus four.
                self.state.a[7] = frame.wrapping_add(4);
                self.state.a[n] = saved;
                self.settle()
            }

            Op::MoveFromSr => self.op_move_from_sr(),
            Op::MoveToCcr | Op::MoveToSr => self.op_move_to_sr(insn),
            Op::MoveUsp => {
                let n = reg_lo(opcode);
                if insn.src == Arg::Usp {
                    self.state.a[n] = self.state.usp();
                } else {
                    let value = self.state.a[n];
                    self.state.set_usp(value);
                }
                self.settle()
            }
            Op::OriToCcr | Op::AndiToCcr | Op::EoriToCcr => self.op_imm_to_ccr(insn.op),
            Op::OriToSr | Op::AndiToSr | Op::EoriToSr => self.op_imm_to_sr(insn.op),
            Op::Movem => self.op_movem(insn, size),
            Op::Movep => self.op_movep(insn, size),
        }
    }

    // ------------------------------------------------------------------
    // Operand resolution
    // ------------------------------------------------------------------

    /// Resolve an operand slot to a [`Loc`].
    fn resolve(&mut self, arg: Arg, size: Size) -> Result<Loc, Trap> {
        let opcode = self.opcode;
        match arg {
            Arg::None => Ok(Loc::Value(0)),
            Arg::DnHi => Ok(Loc::D(reg_hi(opcode) as u8)),
            Arg::DnLo => Ok(Loc::D(reg_lo(opcode) as u8)),
            Arg::AnHi => Ok(Loc::A(reg_hi(opcode) as u8)),
            Arg::AnLo => Ok(Loc::A(reg_lo(opcode) as u8)),
            Arg::Quick => {
                let q = (opcode >> 9) & 7;
                Ok(Loc::Value(if q == 0 { 8 } else { u32::from(q) }))
            }
            Arg::QuickByte => Ok(Loc::Value(i32::from(opcode as i8) as u32)),
            Arg::Vector => Ok(Loc::Value(u32::from(opcode & 0xf))),
            Arg::Imm => {
                let value = match size {
                    Size::Byte => u32::from(self.ext(0)? & 0xff),
                    Size::Word => u32::from(self.ext(0)?),
                    Size::Long => {
                        let hi = self.ext(0)?;
                        let lo = self.ext(0)?;
                        (u32::from(hi) << 16) | u32::from(lo)
                    }
                };
                Ok(Loc::Value(value))
            }
            Arg::Disp16 => {
                let word = self.ext(0)?;
                Ok(Loc::Value(i32::from(word as i16) as u32))
            }
            Arg::Ccr => Ok(Loc::Value(u32::from(self.state.ccr()))),
            Arg::Sr => Ok(Loc::Value(u32::from(self.state.sr))),
            Arg::Usp => Ok(Loc::Value(self.state.usp())),
            Arg::Ea | Arg::EaDst => self.resolve_ea(arg, size, ExtraCycles::Operand),
            // The register-pair, register-list, shift-count, branch-offset and
            // MOVEP slots are not values in an addressing sense: the
            // instruction bodies that use them read the opcode themselves,
            // because what they need is a register *number* or a direction
            // rather than an operand. Nothing routes them through here, and
            // the arm is spelled out so that adding an `Arg` cannot silently
            // start resolving to zero.
            Arg::RmLo
            | Arg::RmHi
            | Arg::PostLo
            | Arg::PostHi
            | Arg::ShiftCount
            | Arg::RegList
            | Arg::MovepEa
            | Arg::BitNumber
            | Arg::Disp8 => {
                debug_assert!(false, "{arg:?} is not resolved as an operand");
                Ok(Loc::Value(0))
            }
        }
    }

    /// Resolve an effective address for an instruction that only wants the
    /// address, never the operand — `LEA`, `PEA`, `JMP`, `JSR`.
    fn resolve_control(&mut self, arg: Arg, extra: ExtraCycles) -> Result<Loc, Trap> {
        self.resolve_ea(arg, Size::Long, extra)
    }

    fn resolve_ea(&mut self, arg: Arg, size: Size, extra: ExtraCycles) -> Result<Loc, Trap> {
        let opcode = self.opcode;
        let Some((mode, reg)) = ea_of(arg, opcode) else {
            // decode() rejected this already; reaching here means a caller
            // asked for an address the row does not have.
            debug_assert!(false, "{opcode:04x} has no effective address in {arg:?}");
            return Ok(Loc::Value(0));
        };
        let reg = reg as usize;
        match mode {
            Mode::DataReg => Ok(Loc::D(reg as u8)),
            Mode::AddrReg => Ok(Loc::A(reg as u8)),
            Mode::Indirect => Ok(Loc::Mem(self.state.a[reg])),
            Mode::PostInc => {
                let addr = self.state.a[reg];
                // The register advances as the address is calculated, so an
                // access that faults still leaves it advanced — except as a
                // `MOVE` destination, where nothing has touched the address
                // register by the time the write is attempted.
                if extra == ExtraCycles::MoveDest {
                    self.deferred_postincrement = Some((reg as u8, step(size, reg)));
                } else {
                    self.state.a[reg] = addr.wrapping_add(step(size, reg));
                }
                Ok(Loc::Mem(addr))
            }
            Mode::PreDec => {
                // Two internal cycles for the decrement, before the access —
                // except as a `MOVE` destination, where the decrement overlaps
                // the prefetch that a move performs before its write and costs
                // nothing (MC68000UM Table 8-5 against Table 8-1).
                if extra != ExtraCycles::MoveDest {
                    self.internal(2);
                }
                let addr = self.state.a[reg].wrapping_sub(step(size, reg));
                // As a MOVE destination the register is stepped by the write
                // itself, a word at a time; anywhere else the whole decrement
                // happens here.
                if extra != ExtraCycles::MoveDest {
                    self.state.a[reg] = addr;
                }
                Ok(Loc::Mem(addr))
            }
            Mode::Disp16 => {
                let disp = i32::from(self.ext(0)? as i16) as u32;
                Ok(Loc::Mem(self.state.a[reg].wrapping_add(disp)))
            }
            Mode::Index8 => {
                let word = self.ext(extra.index_delay())?;
                Ok(Loc::Mem(self.index_address(self.state.a[reg], word)))
            }
            Mode::AbsShort => {
                let word = self.ext(0)?;
                Ok(Loc::Mem(i32::from(word as i16) as u32))
            }
            Mode::AbsLong => {
                let hi = self.ext(0)?;
                // A `MOVE` to an absolute long address writes its operand
                // *before* the last of its own instruction fetches — but only
                // when the source came out of memory. With a register or
                // immediate source the microcode has a spare cycle and does
                // the fetch first. The difference is visible in the program
                // counter an address error pushes, so it is not a free choice.
                let defer = extra == ExtraCycles::MoveDest && self.source_was_memory;
                let lo = if defer {
                    self.ext_deferred()
                } else {
                    self.ext(0)?
                };
                Ok(Loc::Mem((u32::from(hi) << 16) | u32::from(lo)))
            }
            Mode::PcDisp16 => {
                // The base is the address of the extension word itself, which
                // is `pc + 2` while the word is still in the queue.
                let base = self.state.pc.wrapping_add(2);
                let disp = i32::from(self.ext(0)? as i16) as u32;
                Ok(Loc::Mem(base.wrapping_add(disp)))
            }
            Mode::PcIndex8 => {
                let base = self.state.pc.wrapping_add(2);
                let word = self.ext(extra.index_delay())?;
                Ok(Loc::Mem(self.index_address(base, word)))
            }
            Mode::Imm => {
                let value = match size {
                    Size::Byte => u32::from(self.ext(0)? & 0xff),
                    Size::Word => u32::from(self.ext(0)?),
                    Size::Long => {
                        let hi = self.ext(0)?;
                        let lo = self.ext(0)?;
                        (u32::from(hi) << 16) | u32::from(lo)
                    }
                };
                Ok(Loc::Value(value))
            }
        }
    }

    /// Compute a `JMP` or `JSR` target.
    ///
    /// Not the same calculation as any other effective address, in two ways
    /// the bus shows. The **last extension word is taken straight out of the
    /// prefetch queue with no refill**: the queue is about to be reloaded from
    /// the target, so paying for a fetch that will be thrown away would be a
    /// wasted bus cycle and the 68000 does not make one — which is why
    /// `JMP (d16,An)` is ten cycles and not fourteen. And the microcode's
    /// address arithmetic is charged as a block up front rather than spread
    /// around a fetch (MC68000UM Table 8-13).
    fn jump_target(&mut self) -> Result<u32, Trap> {
        let Some((mode, reg)) = ea_of(Arg::Ea, self.opcode) else {
            // decode() only lets control modes reach here.
            debug_assert!(false, "{:04x} is not a jump", self.opcode);
            return Ok(self.state.pc);
        };
        let reg = reg as usize;
        let queued = self.state.prefetch[1];
        Ok(match mode {
            Mode::Indirect => self.state.a[reg],
            Mode::Disp16 => {
                self.internal(2);
                self.state.a[reg].wrapping_add(i32::from(queued as i16) as u32)
            }
            Mode::Index8 => {
                self.internal(6);
                self.index_address(self.state.a[reg], queued)
            }
            Mode::AbsShort => {
                self.internal(2);
                i32::from(queued as i16) as u32
            }
            Mode::AbsLong => {
                // Two words, and only the first of them is worth a refill.
                let hi = self.ext(0)?;
                (u32::from(hi) << 16) | u32::from(self.state.prefetch[1])
            }
            Mode::PcDisp16 => {
                self.internal(2);
                self.state
                    .pc
                    .wrapping_add(2)
                    .wrapping_add(i32::from(queued as i16) as u32)
            }
            Mode::PcIndex8 => {
                self.internal(6);
                let base = self.state.pc.wrapping_add(2);
                self.index_address(base, queued)
            }
            Mode::DataReg | Mode::AddrReg | Mode::PostInc | Mode::PreDec | Mode::Imm => {
                debug_assert!(false, "{:04x} jumps to a non-control mode", self.opcode);
                self.state.pc
            }
        })
    }

    /// Apply a brief extension word to a base address.
    ///
    /// Bit 15 selects the data or address file, bits 14–12 the register, bit
    /// 11 whether the index is the sign-extended low word or the whole
    /// register, and bits 7–0 are a signed displacement. Bits 10–8 are the
    /// 68020's scale and full-format bits and are ignored here, which is what
    /// a 68000 does with them (M68000PRM §2.1).
    fn index_address(&self, base: u32, ext: u16) -> u32 {
        let reg = ((ext >> 12) & 7) as usize;
        let value = if ext & 0x8000 != 0 {
            self.state.a[reg]
        } else {
            self.state.d[reg]
        };
        let index = if ext & 0x0800 != 0 {
            value
        } else {
            i32::from(value as i16) as u32
        };
        let disp = i32::from(ext as i8) as u32;
        base.wrapping_add(index).wrapping_add(disp)
    }

    /// Read a resolved operand.
    fn read_loc(&mut self, loc: Loc, size: Size) -> Result<u32, Trap> {
        Ok(match loc {
            Loc::D(n) => self.state.d[n as usize] & size.mask(),
            Loc::A(n) => self.state.a[n as usize] & size.mask(),
            Loc::Value(v) => v & size.mask(),
            Loc::Prefetched(_, value) => value & size.mask(),
            Loc::Mem(addr) => match size {
                Size::Byte => u32::from(self.read_byte(addr)?),
                Size::Word => u32::from(self.read_word(addr)?),
                Size::Long => self.read_long(addr)?,
            },
        })
    }

    /// Write a resolved operand.
    fn write_loc(&mut self, loc: Loc, size: Size, value: u32) -> Result<(), Trap> {
        match loc {
            Loc::D(n) => {
                let n = n as usize;
                self.state.d[n] = merge(self.state.d[n], value, size);
            }
            // Every write to an address register is 32 bits wide, whatever the
            // instruction's size says (M68000PRM §1.2).
            Loc::A(n) => self.state.a[n as usize] = value,
            Loc::Value(_) => {}
            Loc::Prefetched(addr, _) | Loc::Mem(addr) => match size {
                Size::Byte => self.write_byte(addr, value as u8)?,
                Size::Word => self.write_word(addr, value as u16)?,
                Size::Long => self.write_long(addr, value)?,
            },
        }
        Ok(())
    }

    /// Commit the result of a read-modify-write.
    ///
    /// Identical to [`Exec::write_loc`] except for a long memory destination,
    /// which goes out low word first — see [`Exec::write_long_low_first`].
    fn write_back(&mut self, loc: Loc, size: Size, value: u32) -> Result<(), Trap> {
        if let (Loc::Mem(addr) | Loc::Prefetched(addr, _), Size::Long) = (loc, size) {
            return self.write_long_low_first(addr, value);
        }
        self.write_loc(loc, size, value)
    }

    // ------------------------------------------------------------------
    // Flags
    // ------------------------------------------------------------------

    fn set_flag(&mut self, mask: u16, on: bool) {
        if on {
            self.state.sr |= mask;
        } else {
            self.state.sr &= !mask;
        }
    }

    /// `N` and `Z` from a result, `V` and `C` cleared — the logical and move
    /// rule, which is the only regular one in the instruction set.
    fn set_logic_flags(&mut self, value: u32, size: Size) {
        let value = value & size.mask();
        self.set_flag(flags::N, value & size.sign_bit() != 0);
        self.set_flag(flags::Z, value == 0);
        self.set_flag(flags::V, false);
        self.set_flag(flags::C, false);
    }

    // ------------------------------------------------------------------
    // Instruction bodies
    // ------------------------------------------------------------------

    fn op_move(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        self.source_was_memory =
            ea_of(insn.src, self.opcode).is_some_and(|(mode, _)| mode.is_memory());
        let src = self.resolve(insn.src, size)?;
        let value = self.read_loc(src, size)?;
        if insn.op == Op::Movea {
            // MOVEA.W sign-extends into the whole register.
            let value = if size == Size::Word {
                i32::from(value as i16) as u32
            } else {
                value
            };
            self.state.a[reg_hi(self.opcode)] = value;
            return self.settle();
        }
        let dst = self.resolve_ea(insn.dst, size, ExtraCycles::MoveDest)?;
        self.set_logic_flags(value, size);
        // A predecrement destination is the one `MOVE` that prefetches before
        // it writes, and its long form puts the low word out first.
        if let Some((Mode::PreDec, reg)) = ea_of(insn.dst, self.opcode) {
            let reg = reg as usize;
            self.settle()?;
            if size == Size::Long {
                // Two word writes, low half first, with the register stepping
                // between them: a fault on the first leaves it two bytes down,
                // not four, and the handler can tell.
                let low = self.state.a[reg].wrapping_sub(2);
                self.state.a[reg] = low;
                self.write_word(low, value as u16)?;
                let high = low.wrapping_sub(2);
                self.state.a[reg] = high;
                self.write_word(high, (value >> 16) as u16)?;
            } else {
                let addr = self.state.a[reg].wrapping_sub(step(size, reg));
                self.state.a[reg] = addr;
                self.write_loc(Loc::Mem(addr), size, value)?;
            }
        } else {
            self.write_loc(dst, size, value)?;
            self.settle()?;
        }
        if let Some((reg, by)) = self.deferred_postincrement.take() {
            let reg = reg as usize;
            self.state.a[reg] = self.state.a[reg].wrapping_add(by);
        }
        Ok(())
    }

    fn op_binary(&mut self, insn: Insn, size: Size, kind: BinOp) -> Result<(), Trap> {
        let src = self.resolve(insn.src, size)?;
        let src_value = self.read_loc(src, size)?;
        let dst = self.resolve(insn.dst, size)?;
        // ADDQ/SUBQ on an address register is a full 32-bit add that touches
        // no flags at all (M68000PRM, ADDQ).
        if let Loc::A(n) = dst
            && matches!(insn.op, Op::Addq | Op::Subq)
        {
            let base = self.state.a[n as usize];
            self.state.a[n as usize] = if kind == BinOp::Add {
                base.wrapping_add(src_value)
            } else {
                base.wrapping_sub(src_value)
            };
            // A long quick add to an address register is two cycles cheaper
            // than a word one: the word form has to sign-extend first.
            self.internal(if size == Size::Long { 2 } else { 4 });
            return self.settle();
        }
        let dst_value = self.read_loc(dst, size)?;
        let result = match kind {
            BinOp::Add => {
                let r = dst_value.wrapping_add(src_value) & size.mask();
                self.set_add_flags(src_value, dst_value, r, size, true);
                r
            }
            BinOp::Sub => {
                let r = dst_value.wrapping_sub(src_value) & size.mask();
                self.set_sub_flags(src_value, dst_value, r, size, true);
                r
            }
            BinOp::And => {
                let r = dst_value & src_value & size.mask();
                self.set_logic_flags(r, size);
                r
            }
            BinOp::Or => {
                let r = (dst_value | src_value) & size.mask();
                self.set_logic_flags(r, size);
                r
            }
            BinOp::Eor => {
                let r = (dst_value ^ src_value) & size.mask();
                self.set_logic_flags(r, size);
                r
            }
        };
        self.arith_internal(insn, size, dst);
        if matches!(dst, Loc::D(_) | Loc::A(_)) {
            self.write_loc(dst, size, result)?;
            self.settle()
        } else {
            // A memory destination is written after the final prefetch on a
            // 68000; the write is the last bus cycle of a read-modify-write
            // only for TAS (MC68000UM Table 8-5).
            self.settle()?;
            self.write_back(dst, size, result)
        }
    }

    /// The long-operand penalty the manual marks with a double dagger.
    ///
    /// A 32-bit ALU operation into a data register costs two extra cycles, and
    /// four when the source needed no bus cycle of its own — a register or an
    /// immediate (MC68000UM Table 8-5, note **).
    fn arith_internal(&mut self, insn: Insn, size: Size, dst: Loc) {
        if size != Size::Long || !matches!(dst, Loc::D(_)) {
            return;
        }
        let cheap_source = match insn.src {
            // EOR is the one row whose source is named directly rather than
            // through an effective address, and it is still a register.
            Arg::Imm | Arg::Quick | Arg::DnHi => true,
            Arg::Ea => matches!(
                ea_of(Arg::Ea, self.opcode),
                Some((Mode::DataReg | Mode::AddrReg | Mode::Imm, _))
            ),
            _ => false,
        };
        self.internal(if cheap_source { 4 } else { 2 });
    }

    fn op_compare(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let src = self.resolve(insn.src, size)?;
        let src_value = self.read_loc(src, size)?;
        let dst = self.resolve(insn.dst, size)?;
        let dst_value = self.read_loc(dst, size)?;
        let result = dst_value.wrapping_sub(src_value) & size.mask();
        // CMP leaves X alone: it is a test, not an arithmetic step.
        self.set_sub_flags(src_value, dst_value, result, size, false);
        if size == Size::Long && matches!(dst, Loc::D(_)) {
            self.internal(2);
        }
        self.settle()
    }

    fn op_cmpm(&mut self, size: Size) -> Result<(), Trap> {
        let y = reg_lo(self.opcode);
        let x = reg_hi(self.opcode);
        let src_addr = self.state.a[y];
        self.state.a[y] = src_addr.wrapping_add(step(size, y));
        let src_value = self.read_loc(Loc::Mem(src_addr), size)?;
        let dst_addr = self.state.a[x];
        self.state.a[x] = dst_addr.wrapping_add(step(size, x));
        let dst_value = self.read_loc(Loc::Mem(dst_addr), size)?;
        let result = dst_value.wrapping_sub(src_value) & size.mask();
        self.set_sub_flags(src_value, dst_value, result, size, false);
        self.settle()
    }

    fn op_adda(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let src = self.resolve(insn.src, size)?;
        let raw = self.read_loc(src, size)?;
        // A word source is sign-extended to 32 bits before the add; the
        // operation itself is always long (M68000PRM, ADDA).
        let value = if size == Size::Word {
            i32::from(raw as i16) as u32
        } else {
            raw
        };
        let n = reg_hi(self.opcode);
        let base = self.state.a[n];
        self.state.a[n] = if insn.op == Op::Adda {
            base.wrapping_add(value)
        } else {
            base.wrapping_sub(value)
        };
        let cheap = matches!(
            ea_of(Arg::Ea, self.opcode),
            Some((Mode::DataReg | Mode::AddrReg | Mode::Imm, _))
        );
        self.internal(if size == Size::Word || cheap { 4 } else { 2 });
        self.settle()
    }

    fn op_cmpa(&mut self, size: Size) -> Result<(), Trap> {
        let src = self.resolve(Arg::Ea, size)?;
        let raw = self.read_loc(src, size)?;
        let value = if size == Size::Word {
            i32::from(raw as i16) as u32
        } else {
            raw
        };
        let dst_value = self.state.a[reg_hi(self.opcode)];
        let result = dst_value.wrapping_sub(value);
        self.set_sub_flags(value, dst_value, result, Size::Long, false);
        self.internal(2);
        self.settle()
    }

    fn op_addx(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let x = u32::from(self.state.flag(flags::X));
        let memory = self.opcode & 0x0008 != 0;
        let (src_value, dst) = if memory {
            let y = reg_lo(self.opcode);
            let xr = reg_hi(self.opcode);
            // One pair of internal cycles for the whole instruction, not one
            // per predecrement: the second address calculation overlaps the
            // first operand's fetch (MC68000UM Table 8-8).
            self.internal(2);
            let src_value = self.read_predecrement(y, size)?;
            let dst_value = self.read_predecrement(xr, size)?;
            (src_value, Loc::Prefetched(self.state.a[xr], dst_value))
        } else {
            let src_value = self.state.d[reg_lo(self.opcode)] & size.mask();
            (src_value, Loc::D(reg_hi(self.opcode) as u8))
        };
        let dst_value = self.read_loc(dst, size)?;
        // Z is only ever *cleared* by an extended operation: a multi-precision
        // sum is zero only if every step of it was, so a zero result leaves Z
        // exactly as the previous step left it (M68000PRM, ADDX).
        let was_zero = self.state.flag(flags::Z);
        let result = if insn.op == Op::Addx {
            let r = dst_value.wrapping_add(src_value).wrapping_add(x) & size.mask();
            self.set_add_flags(src_value, dst_value, r, size, true);
            r
        } else {
            let r = dst_value.wrapping_sub(src_value).wrapping_sub(x) & size.mask();
            self.set_sub_flags(src_value, dst_value, r, size, true);
            r
        };
        self.set_flag(flags::Z, result == 0 && was_zero);
        if size == Size::Long && matches!(dst, Loc::D(_)) {
            self.internal(4);
        }
        if let Loc::D(_) = dst {
            self.write_loc(dst, size, result)?;
            return self.settle();
        }
        // A long extended result puts its low word out, prefetches, and only
        // then writes the high word — the prefetch lands *between* the two
        // halves of the write, which no other instruction does.
        let (Loc::Mem(addr) | Loc::Prefetched(addr, _)) = dst else {
            return self.settle();
        };
        if size == Size::Long {
            self.write_word(addr.wrapping_add(2), result as u16)?;
            self.settle()?;
            return self.write_word(addr, (result >> 16) as u16);
        }
        self.settle()?;
        self.write_back(dst, size, result)
    }

    fn op_bcd(&mut self, insn: Insn) -> Result<(), Trap> {
        let size = Size::Byte;
        let memory = self.opcode & 0x0008 != 0;
        let (src_value, dst) = if memory {
            let y = reg_lo(self.opcode);
            let x = reg_hi(self.opcode);
            let src_value = self.read_predecrement(y, size)?;
            let dst_value = self.read_predecrement(x, size)?;
            (src_value, Loc::Prefetched(self.state.a[x], dst_value))
        } else {
            (
                self.state.d[reg_lo(self.opcode)] & 0xff,
                Loc::D(reg_hi(self.opcode) as u8),
            )
        };
        let dst_value = self.read_loc(dst, size)?;
        let result = if insn.op == Op::Abcd {
            self.bcd_add(src_value, dst_value)
        } else {
            self.bcd_sub(src_value, dst_value)
        };
        self.internal(2);
        if matches!(dst, Loc::D(_)) {
            self.write_loc(dst, size, result)?;
            self.settle()
        } else {
            self.settle()?;
            self.write_back(dst, size, result)
        }
    }

    fn op_nbcd(&mut self) -> Result<(), Trap> {
        let dst = self.resolve(Arg::Ea, Size::Byte)?;
        let value = self.read_loc(dst, Size::Byte)?;
        let result = self.bcd_sub(value, 0);
        if matches!(dst, Loc::D(_)) {
            self.internal(2);
            self.write_loc(dst, Size::Byte, result)?;
            self.settle()
        } else {
            self.settle()?;
            self.write_back(dst, Size::Byte, result)
        }
    }

    fn op_unary(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let dst = self.resolve(insn.dst, size)?;
        // CLR still reads its destination on a 68000 — the read is a real bus
        // cycle and a device can see it (MC68000UM Table 8-6, and the reason
        // CLR is not usable on a read-sensitive register).
        let value = self.read_loc(dst, size)?;
        let x = u32::from(self.state.flag(flags::X));
        let result = match insn.op {
            Op::Clr => {
                self.set_flag(flags::N, false);
                self.set_flag(flags::Z, true);
                self.set_flag(flags::V, false);
                self.set_flag(flags::C, false);
                0
            }
            Op::Not => {
                let r = !value & size.mask();
                self.set_logic_flags(r, size);
                r
            }
            Op::Neg => {
                let r = 0u32.wrapping_sub(value) & size.mask();
                self.set_sub_flags(value, 0, r, size, true);
                r
            }
            _ => {
                let r = 0u32.wrapping_sub(value).wrapping_sub(x) & size.mask();
                let before = self.state.flag(flags::Z);
                self.set_sub_flags(value, 0, r, size, true);
                self.set_flag(flags::Z, if r == 0 { before } else { false });
                r
            }
        };
        if size == Size::Long && matches!(dst, Loc::D(_)) {
            self.internal(2);
        }
        if matches!(dst, Loc::D(_) | Loc::A(_)) {
            self.write_loc(dst, size, result)?;
            self.settle()
        } else {
            self.settle()?;
            self.write_back(dst, size, result)
        }
    }

    fn op_tst(&mut self, size: Size) -> Result<(), Trap> {
        let src = self.resolve(Arg::Ea, size)?;
        let value = self.read_loc(src, size)?;
        self.set_logic_flags(value, size);
        self.settle()
    }

    fn op_tas(&mut self) -> Result<(), Trap> {
        let dst = self.resolve(Arg::Ea, Size::Byte)?;
        let value = self.read_loc(dst, Size::Byte)?;
        self.set_logic_flags(value, Size::Byte);
        let result = value | 0x80;
        if matches!(dst, Loc::D(_)) {
            self.write_loc(dst, Size::Byte, result)?;
            self.settle()
        } else {
            // The read-modify-write cycle is indivisible: the write follows
            // the read immediately, before the prefetch (M68000PRM, TAS).
            self.internal(2);
            self.write_loc(dst, Size::Byte, result)?;
            self.settle()
        }
    }

    fn op_exg(&mut self, insn: Insn) -> Result<(), Trap> {
        let hi = reg_hi(self.opcode);
        let lo = reg_lo(self.opcode);
        match (insn.src, insn.dst) {
            (Arg::DnHi, Arg::DnLo) => self.state.d.swap(hi, lo),
            (Arg::AnHi, Arg::AnLo) => self.state.a.swap(hi, lo),
            _ => {
                core::mem::swap(&mut self.state.d[hi], &mut self.state.a[lo]);
            }
        }
        self.internal(2);
        self.settle()
    }

    fn op_mul(&mut self, insn: Insn) -> Result<(), Trap> {
        let src = self.resolve(Arg::Ea, Size::Word)?;
        let source = self.read_loc(src, Size::Word)? as u16;
        let n = reg_hi(self.opcode);
        let dest = self.state.d[n] as u16;
        let (result, extra) = if insn.op == Op::Mulu {
            // 38 cycles plus two per one bit in the source (MC68000UM
            // Table 8-6); four of the 38 are the final prefetch.
            let product = u32::from(source).wrapping_mul(u32::from(dest));
            (product, 34 + 2 * source.count_ones())
        } else {
            let product = (i32::from(source as i16)).wrapping_mul(i32::from(dest as i16)) as u32;
            // MULS counts the 01 and 10 pairs of the source with a zero
            // appended below it, which is the Booth encoding the microcode
            // steps through. Sixteen pairs, so bit 16 of the difference is not
            // one of them — counting it makes every negative multiplier two
            // cycles too slow.
            let pairs = (u32::from(source) << 1) ^ u32::from(source);
            (product, 34 + 2 * (pairs & 0xffff).count_ones())
        };
        self.internal(extra);
        self.state.d[n] = result;
        self.set_logic_flags(result, Size::Long);
        self.settle()
    }

    fn op_div(&mut self, insn: Insn, pc0: u32) -> Result<(), Trap> {
        let src = self.resolve(Arg::Ea, Size::Word)?;
        let divisor = self.read_loc(src, Size::Word)? as u16;
        let n = reg_hi(self.opcode);
        let dividend = self.state.d[n];
        if divisor == 0 {
            // Division by zero does not prefetch, does not advance the queue,
            // and pushes the address of the instruction itself. The condition
            // codes the manual leaves undefined are cleared, X excepted.
            //
            // The corpus contains exactly one divide-by-zero vector, so this
            // rests on a single measurement; it is recorded here rather than
            // guessed at from the manual, which says only "undefined".
            self.set_flag(flags::N, false);
            self.set_flag(flags::Z, false);
            self.set_flag(flags::V, false);
            self.set_flag(flags::C, false);
            self.prologue = 8;
            return Err(Trap::at(vector::DIVIDE_BY_ZERO, pc0));
        }
        if insn.op == Op::Divu {
            let quotient = dividend / u32::from(divisor);
            let remainder = dividend % u32::from(divisor);
            self.internal(divu_cycles(dividend, divisor));
            if quotient > 0xffff {
                // Overflow leaves the destination untouched and sets V.
                self.set_flag(flags::V, true);
                self.set_flag(flags::C, false);
                return self.settle();
            }
            self.state.d[n] = (remainder << 16) | (quotient & 0xffff);
            self.set_logic_flags(quotient & 0xffff, Size::Word);
        } else {
            let dividend = dividend as i32;
            let divisor = i32::from(divisor as i16);
            self.internal(divs_cycles(dividend, divisor as i16));
            let quotient = dividend.wrapping_div(divisor);
            let remainder = dividend.wrapping_rem(divisor);
            if !(-0x8000..=0x7fff).contains(&quotient) {
                self.set_flag(flags::V, true);
                self.set_flag(flags::C, false);
                return self.settle();
            }
            self.state.d[n] = ((remainder as u32) << 16) | (quotient as u32 & 0xffff);
            self.set_logic_flags(quotient as u32 & 0xffff, Size::Word);
        }
        self.settle()
    }

    fn op_chk(&mut self) -> Result<(), Trap> {
        let src = self.resolve(Arg::Ea, Size::Word)?;
        let bound = self.read_loc(src, Size::Word)? as i16;
        let value = self.state.d[reg_hi(self.opcode)] as i16;
        // The manual defines N only for the two out-of-bounds cases and calls
        // Z, V and C undefined. The hardware clears those three and — this is
        // the part no document states — leaves **N alone** when the register
        // is in range. Only the two cases that trap write it, which is exactly
        // the two cases the manual defines.
        self.set_flag(flags::Z, false);
        self.set_flag(flags::V, false);
        self.set_flag(flags::C, false);
        // Unlike a trap, CHK completes its prefetch before vectoring, so the
        // address it pushes is the next instruction's by construction.
        self.settle()?;
        if value < 0 || value > bound {
            self.set_flag(flags::N, value < 0);
            // Two extra cycles of prologue unless the bound test is what
            // failed: a register above its bound is decided a test earlier
            // than a negative one.
            self.prologue = if value > bound { 4 } else { 6 };
            let pc = self.state.pc;
            return Err(Trap::at(vector::CHK, pc));
        }
        self.internal(6);
        Ok(())
    }

    fn op_bit(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let bit = match insn.src {
            Arg::BitNumber => u32::from(self.ext(0)? & 0xff),
            _ => self.state.d[reg_hi(self.opcode)],
        };
        let dst = self.resolve(insn.dst, size)?;
        let width = if size == Size::Long { 32 } else { 8 };
        let bit = bit % width;
        let value = self.read_loc(dst, size)?;
        let mask = 1u32 << bit;
        self.set_flag(flags::Z, value & mask == 0);
        let result = match insn.op {
            Op::Btst => {
                // A long test costs two more, and so does one whose operand is
                // an immediate: the bit number has to be reduced modulo the
                // operand size either way, and nothing else is happening.
                if size == Size::Long
                    || matches!(ea_of(insn.dst, self.opcode), Some((Mode::Imm, _)))
                {
                    self.internal(2);
                }
                return self.settle();
            }
            Op::Bchg => value ^ mask,
            Op::Bclr => value & !mask,
            _ => value | mask,
        };
        if matches!(dst, Loc::D(_)) {
            // A long bit operation on a register costs two more, and BCLR two
            // more again (MC68000UM Table 8-7).
            self.internal(if bit >= 16 { 4 } else { 2 });
            if insn.op == Op::Bclr {
                self.internal(2);
            }
            self.write_loc(dst, size, result)?;
            self.settle()
        } else {
            self.settle()?;
            self.write_back(dst, size, result)
        }
    }

    fn op_shift(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        if insn.dst == Arg::Ea {
            // The memory form shifts one bit of one word.
            let dst = self.resolve(Arg::Ea, Size::Word)?;
            let value = self.read_loc(dst, Size::Word)?;
            let result = self.shift(insn.op, value, 1, Size::Word);
            self.settle()?;
            return self.write_back(dst, Size::Word, result);
        }
        let count = if self.opcode & 0x0020 == 0 {
            let q = (self.opcode >> 9) & 7;
            if q == 0 { 8 } else { u32::from(q) }
        } else {
            self.state.d[reg_hi(self.opcode)] % 64
        };
        let n = reg_lo(self.opcode);
        let value = self.state.d[n] & size.mask();
        let result = self.shift(insn.op, value, count, size);
        self.state.d[n] = merge(self.state.d[n], result, size);
        // Two cycles per bit, on top of the two (word) or four (long) the
        // instruction costs before it starts (MC68000UM Table 8-12).
        self.internal(if size == Size::Long { 4 } else { 2 } + 2 * count);
        self.settle()
    }

    /// One shift or rotate, setting the flags the manual gives it.
    ///
    /// The shift count is taken modulo 64, so it routinely exceeds the operand
    /// width, and what happens then is the part worth stating: **the carry runs
    /// out when the operand does.** The 68000 shifts one bit per cycle through
    /// a register of the operand's width, so a count larger than that width
    /// shifts in nothing but fill and leaves `C` and `X` clear — including for
    /// `ASR` of a negative value, whose *result* is all ones while its carry is
    /// zero. Shifting the sign bit back out again, which is what a naive loop
    /// does, is wrong at exactly the boundary a compiler's shift-by-register
    /// code lands on.
    fn shift(&mut self, op: Op, value: u32, count: u32, size: Size) -> u32 {
        let bits = size.bytes() * 8;
        let mask = size.mask();
        let sign = size.sign_bit();
        let mut result = value & mask;
        let mut carry = false;
        let mut overflow = false;
        // Past the operand's width the register holds only fill, so the loop
        // has nothing left to do.
        let steps = count.min(bits);
        let exhausted = count > bits;
        match op {
            Op::Asl => {
                for _ in 0..steps {
                    carry = result & sign != 0;
                    let next = (result << 1) & mask;
                    // V is set if the sign bit changed at *any* point in the
                    // shift, not just at the end (M68000PRM, ASL).
                    if (next ^ result) & sign != 0 {
                        overflow = true;
                    }
                    result = next;
                }
                if exhausted {
                    carry = false;
                }
            }
            Op::Asr => {
                for _ in 0..steps {
                    carry = result & 1 != 0;
                    result = (result >> 1) | (result & sign);
                }
                if exhausted {
                    carry = false;
                }
            }
            Op::Lsl => {
                for _ in 0..steps {
                    carry = result & sign != 0;
                    result = (result << 1) & mask;
                }
                if exhausted {
                    carry = false;
                }
            }
            Op::Lsr => {
                for _ in 0..steps {
                    carry = result & 1 != 0;
                    result >>= 1;
                }
                if exhausted {
                    carry = false;
                }
            }
            Op::Rol => {
                for _ in 0..count {
                    carry = result & sign != 0;
                    result = ((result << 1) | u32::from(carry)) & mask;
                }
            }
            Op::Ror => {
                for _ in 0..count {
                    carry = result & 1 != 0;
                    result = (result >> 1) | (if carry { sign } else { 0 });
                }
            }
            Op::Roxl => {
                let mut x = self.state.flag(flags::X);
                for _ in 0..count {
                    carry = result & sign != 0;
                    result = ((result << 1) | u32::from(x)) & mask;
                    x = carry;
                }
                if count == 0 {
                    carry = x;
                } else {
                    self.set_flag(flags::X, x);
                }
                self.set_flag(flags::N, result & sign != 0);
                self.set_flag(flags::Z, result == 0);
                self.set_flag(flags::V, false);
                self.set_flag(flags::C, carry);
                return result;
            }
            // ROXR, and the catch-all only because `Op` is non-exhaustive.
            _ => {
                let mut x = self.state.flag(flags::X);
                for _ in 0..count {
                    carry = result & 1 != 0;
                    result = (result >> 1) | (if x { sign } else { 0 });
                    x = carry;
                }
                if count == 0 {
                    carry = x;
                } else {
                    self.set_flag(flags::X, x);
                }
                self.set_flag(flags::N, result & sign != 0);
                self.set_flag(flags::Z, result == 0);
                self.set_flag(flags::V, false);
                self.set_flag(flags::C, carry);
                return result;
            }
        }
        self.set_flag(flags::N, result & sign != 0);
        self.set_flag(flags::Z, result == 0);
        self.set_flag(flags::V, overflow);
        self.set_flag(flags::C, count != 0 && carry);
        // A plain rotate does not touch X — only the shifts and the two
        // rotate-through-extend forms do, and the latter set it themselves.
        // Getting this wrong quietly breaks every multi-precision routine that
        // rotates a mask between `ADDX` steps.
        if count != 0 && matches!(op, Op::Asl | Op::Asr | Op::Lsl | Op::Lsr) {
            self.set_flag(flags::X, carry);
        }
        result
    }

    fn op_pea(&mut self) -> Result<(), Trap> {
        let Loc::Mem(addr) = self.resolve_control(Arg::Ea, ExtraCycles::Control)? else {
            let pc = self.state.pc;
            return Err(Trap::at(vector::ILLEGAL, pc));
        };
        let sp = self.state.a[7].wrapping_sub(4);
        self.state.a[7] = sp;
        // An absolute operand needs no address arithmetic, so the push happens
        // a bus cycle earlier — before the final prefetch rather than after.
        let absolute = matches!(
            ea_of(Arg::Ea, self.opcode),
            Some((Mode::AbsShort | Mode::AbsLong, _))
        );
        if absolute {
            self.write_long(sp, addr)?;
            return self.settle();
        }
        self.settle()?;
        self.write_long(sp, addr)
    }

    /// `JSR`, whose push lands *between* the two fetches of the reloaded
    /// queue.
    ///
    /// That ordering is not decoration: it is why a `JSR` to an odd address
    /// pushes nothing at all before its address-error frame, and so why the
    /// handler sees the stack pointer the caller had.
    fn op_jsr(&mut self, pc0: u32) -> Result<(), Trap> {
        let target = self.jump_target()?;
        // The return address is the byte after the whole instruction, which
        // the queue never slid to: the last extension word was read out of it
        // rather than fetched.
        let words =
            1 + ea_of(Arg::Ea, self.opcode).map_or(0, |(mode, _)| mode.ext_words(Size::Long));
        let ret = pc0.wrapping_add(2 * words);

        self.deferred_slides = 0;
        let fc = self.program_fc();
        self.state.pc = target.wrapping_sub(4);
        let first = self.read_word_fc(target, fc)?;
        self.state.pc = target.wrapping_sub(2);
        let sp = self.state.a[7].wrapping_sub(4);
        self.state.a[7] = sp;
        self.write_long(sp, ret)?;
        let second = self.read_word_fc(target.wrapping_add(2), fc)?;
        self.state.pc = target;
        self.state.prefetch = [first, second];
        Ok(())
    }

    fn op_branch(&mut self, insn: Insn) -> Result<(), Trap> {
        let byte = self.opcode as i8;
        let taken = match insn.op {
            Op::Bcc => self.state.test(Cond::from_opcode(self.opcode)),
            _ => true,
        };
        // The base for a branch is the address of the word after the opcode.
        let base = self.state.pc.wrapping_add(2);
        if byte == 0 {
            let word = self.state.prefetch[1];
            if !taken {
                // A word displacement that is not taken still costs the fetch.
                self.internal(4);
                self.ext(0)?;
                return self.settle();
            }
            let target = base.wrapping_add(i32::from(word as i16) as u32);
            self.internal(2);
            if insn.op == Op::Bsr {
                let ret = base.wrapping_add(2);
                let sp = self.state.a[7].wrapping_sub(4);
                self.state.a[7] = sp;
                self.write_long(sp, ret)?;
            }
            return self.refill(target, 0);
        }
        if !taken {
            self.internal(4);
            return self.settle();
        }
        let target = base.wrapping_add(i32::from(byte) as u32);
        self.internal(2);
        if insn.op == Op::Bsr {
            let ret = base;
            let sp = self.state.a[7].wrapping_sub(4);
            self.state.a[7] = sp;
            self.write_long(sp, ret)?;
        }
        self.refill(target, 0)
    }

    fn op_dbcc(&mut self) -> Result<(), Trap> {
        let base = self.state.pc.wrapping_add(2);
        let n = reg_lo(self.opcode);
        if self.state.test(Cond::from_opcode(self.opcode)) {
            // Condition true: the loop is over, the counter is left alone.
            self.internal(4);
            self.ext(0)?;
            return self.settle();
        }
        let counter = (self.state.d[n] as u16).wrapping_sub(1);
        self.state.d[n] = merge(self.state.d[n], u32::from(counter), Size::Word);
        if counter == 0xffff {
            // The counter ran out: fall through, and pay for the two fetches.
            self.internal(6);
            self.ext(0)?;
            return self.settle();
        }
        let word = self.state.prefetch[1];
        self.internal(2);
        let target = base.wrapping_add(i32::from(word as i16) as u32);
        self.refill(target, 0)
    }

    fn op_scc(&mut self) -> Result<(), Trap> {
        let set = self.state.test(Cond::from_opcode(self.opcode));
        let dst = self.resolve(Arg::Ea, Size::Byte)?;
        let value = if set { 0xff } else { 0x00 };
        if !matches!(dst, Loc::D(_)) {
            // A memory destination is read before it is written, exactly as
            // CLR reads one: the 68000 has no write-only bus cycle, and a
            // read-sensitive register notices.
            self.read_loc(dst, Size::Byte)?;
        }
        if matches!(dst, Loc::D(_)) {
            // Two extra cycles when the byte is set, which is the one place a
            // 68000's timing depends on a condition (MC68000UM Table 8-11).
            if set {
                self.internal(2);
            }
            self.write_loc(dst, Size::Byte, value)?;
            self.settle()
        } else {
            self.settle()?;
            self.write_back(dst, Size::Byte, value)
        }
    }

    fn op_link(&mut self) -> Result<(), Trap> {
        let n = reg_lo(self.opcode);
        let disp = i32::from(self.ext(0)? as i16) as u32;
        let sp = self.state.a[7].wrapping_sub(4);
        // The stack pointer is decremented *before* the register is read, so
        // `LINK A7,#d` pushes the new A7 rather than the old one — the one
        // case where the two differ, and the one the manual is explicit about.
        self.state.a[7] = sp;
        let value = self.state.a[n];
        self.write_long(sp, value)?;
        self.state.a[n] = sp;
        self.state.a[7] = sp.wrapping_add(disp);
        self.settle()
    }

    fn op_move_from_sr(&mut self) -> Result<(), Trap> {
        let dst = self.resolve(Arg::Ea, Size::Word)?;
        // The 68000 reads the destination first, which is why MOVE from SR is
        // a read-modify-write and the 68010 replaced it (MC68000UM Table 8-6).
        let _ = self.read_loc(dst, Size::Word)?;
        let sr = self.state.sr;
        if matches!(dst, Loc::D(_)) {
            self.internal(2);
            self.write_loc(dst, Size::Word, u32::from(sr))?;
            return self.settle();
        }
        self.settle()?;
        self.write_loc(dst, Size::Word, u32::from(sr))
    }

    fn op_move_to_sr(&mut self, insn: Insn) -> Result<(), Trap> {
        let src = self.resolve(Arg::Ea, Size::Word)?;
        let value = self.read_loc(src, Size::Word)? as u16;
        if insn.op == Op::MoveToCcr {
            let sr = (self.state.sr & !flags::CCR) | (value & flags::CCR);
            self.state.set_sr(sr);
        } else {
            self.state.set_sr(value);
        }
        self.internal(4);
        // Both forms *reload* the queue rather than sliding it. Writing SR can
        // change the privilege state, and the word already in the queue was
        // fetched with the old function code — so the 68000 fetches it again,
        // which is visible on FC0-FC2 as well as in the cycle count.
        let next = self.state.pc.wrapping_add(2);
        self.refill(next, 0)
    }

    fn op_imm_to_ccr(&mut self, op: Op) -> Result<(), Trap> {
        let value = self.ext(0)? & 0xff;
        let ccr = u16::from(self.state.ccr());
        let result = match op {
            Op::OriToCcr => ccr | value,
            Op::AndiToCcr => ccr & value,
            _ => ccr ^ value,
        };
        let sr = (self.state.sr & !flags::CCR) | (result & flags::CCR);
        self.state.set_sr(sr);
        self.internal(8);
        self.idle_fetch()?;
        self.settle()
    }

    fn op_imm_to_sr(&mut self, op: Op) -> Result<(), Trap> {
        let value = self.ext(0)?;
        let sr = self.state.sr;
        let result = match op {
            Op::OriToSr => sr | value,
            Op::AndiToSr => sr & value,
            _ => sr ^ value,
        };
        self.state.set_sr(result);
        self.internal(8);
        self.idle_fetch()?;
        self.settle()
    }

    /// Re-read the word already sitting in `prefetch[1]`, discarding it.
    ///
    /// The immediate-to-`SR`/`CCR` instructions really do this: the microcode
    /// spends a bus cycle fetching a word it already has, which is why they
    /// cost twenty cycles for what looks like sixteen cycles of work. It is a
    /// visible access, so a device on the bus sees it.
    fn idle_fetch(&mut self) -> Result<(), Trap> {
        let addr = self.state.pc.wrapping_add(2);
        let fc = self.program_fc();
        self.read_word_fc(addr, fc)?;
        Ok(())
    }

    /// `MOVEM`, which is a **word engine**, not a register engine.
    ///
    /// The distinction is invisible until something goes wrong. A long
    /// transfer is two independent word accesses, and the address register a
    /// predecrement or postincrement form is walking is updated between them —
    /// so an address error half way through a `MOVEM.L` leaves that register
    /// two bytes into the element that failed, not four. Modelling it as "per
    /// register, then update" gets every ordinary case right and that one
    /// wrong, which is the sort of difference an exception handler notices.
    ///
    /// The other two things worth stating: a predecrement destination walks
    /// the mask backwards, `A7` first, and a memory-to-register form always
    /// reads one word past the last register it loads and throws it away
    /// (M68000PRM, *MOVEM*).
    fn op_movem(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let mask = self.ext(0)?;
        let to_memory = insn.dst == Arg::Ea;
        let Some((mode, reg)) = ea_of(Arg::Ea, self.opcode) else {
            let pc = self.state.pc;
            return Err(Trap::at(vector::ILLEGAL, pc));
        };
        let reg = reg as usize;
        let long = size == Size::Long;

        if to_memory && mode == Mode::PreDec {
            // The register being walked is stored with the value it had
            // *before* the instruction started, not the value it has reached
            // by the time its turn comes. That is a 68000 behaviour the 68020
            // changed, and `MOVEM.L A7/D0-D7,-(A7)` depends on it.
            let initial = self.state.a[reg];
            for bit in 0..16u32 {
                if mask & (1 << bit) == 0 {
                    continue;
                }
                let index = 15 - bit;
                let value = if index as usize == reg + 8 {
                    initial
                } else {
                    self.register(index)
                };
                // The address register is updated *after* each word lands, so
                // a fault leaves it addressing the word before the one that
                // failed.
                let low = self.state.a[reg].wrapping_sub(2);
                self.write_word(low, value as u16)?;
                self.state.a[reg] = low;
                if long {
                    let high = low.wrapping_sub(2);
                    self.write_word(high, (value >> 16) as u16)?;
                    self.state.a[reg] = high;
                }
            }
            return self.settle();
        }

        let mut addr = if !to_memory && mode == Mode::PostInc {
            self.state.a[reg]
        } else {
            let Loc::Mem(addr) = self.resolve_control(Arg::Ea, ExtraCycles::Operand)? else {
                let pc = self.state.pc;
                return Err(Trap::at(vector::ILLEGAL, pc));
            };
            addr
        };
        let walking = !to_memory && mode == Mode::PostInc;

        for bit in 0..16u32 {
            if mask & (1 << bit) == 0 {
                continue;
            }
            if to_memory {
                let value = self.register(bit);
                if long {
                    self.write_word(addr, (value >> 16) as u16)?;
                    self.write_word(addr.wrapping_add(2), value as u16)?;
                } else {
                    self.write_word(addr, value as u16)?;
                }
                addr = addr.wrapping_add(if long { 4 } else { 2 });
            } else {
                if walking {
                    self.state.a[reg] = addr.wrapping_add(2);
                }
                let high = self.read_word(addr)?;
                let value = if long {
                    if walking {
                        self.state.a[reg] = addr.wrapping_add(4);
                    }
                    let low = self.read_word(addr.wrapping_add(2))?;
                    (u32::from(high) << 16) | u32::from(low)
                } else {
                    i32::from(high as i16) as u32
                };
                addr = addr.wrapping_add(if long { 4 } else { 2 });
                self.set_register(bit, value);
            }
        }
        if !to_memory {
            // One word past the end, read and discarded. It is a real bus
            // cycle, and a MOVEM that ends at the top of a mapped region can
            // fault on it.
            if walking {
                self.state.a[reg] = addr.wrapping_add(2);
            }
            self.read_word(addr)?;
            if walking {
                self.state.a[reg] = addr;
            }
        }
        self.settle()
    }

    /// One of the sixteen registers a `MOVEM` mask can name: `D0`-`D7` then
    /// `A0`-`A7`.
    fn register(&self, index: u32) -> u32 {
        if index < 8 {
            self.state.d[index as usize]
        } else {
            self.state.a[(index - 8) as usize]
        }
    }

    /// Load one of those sixteen registers.
    fn set_register(&mut self, index: u32, value: u32) {
        if index < 8 {
            self.state.d[index as usize] = value;
        } else {
            self.state.a[(index - 8) as usize] = value;
        }
    }

    fn op_movep(&mut self, insn: Insn, size: Size) -> Result<(), Trap> {
        let n = reg_hi(self.opcode);
        let areg = reg_lo(self.opcode);
        let disp = i32::from(self.ext(0)? as i16) as u32;
        let base = self.state.a[areg].wrapping_add(disp);
        let to_memory = insn.dst == Arg::MovepEa;
        let count = if size == Size::Long { 4 } else { 2 };
        if to_memory {
            let value = self.state.d[n];
            for i in 0..count {
                let shift = 8 * (count - 1 - i);
                let byte = (value >> shift) as u8;
                self.write_byte(base.wrapping_add(i * 2), byte)?;
            }
        } else {
            let mut value = 0u32;
            for i in 0..count {
                let byte = self.read_byte(base.wrapping_add(i * 2))?;
                value = (value << 8) | u32::from(byte);
            }
            self.state.d[n] = merge(self.state.d[n], value, size);
        }
        self.settle()
    }

    // ------------------------------------------------------------------
    // Shared arithmetic
    // ------------------------------------------------------------------

    /// `-(An)` as `ADDX`, `SUBX`, `ABCD` and `SBCD` perform it: **one word at
    /// a time**.
    ///
    /// Those four are the only instructions whose predecrement is not a single
    /// address calculation. A long operand is two separate steps — decrement
    /// two, read a word, decrement two, read a word — which is why the low
    /// half comes off the bus first and why an address error part way through
    /// leaves the register two bytes further on than a `-(An)` anywhere else
    /// would. Multi-precision code that catches its own bus errors can see the
    /// difference.
    fn predecrement(&mut self, reg: usize, size: Size) -> u32 {
        let addr = self.state.a[reg].wrapping_sub(step(size, reg));
        self.state.a[reg] = addr;
        addr
    }

    /// Read the operand a predecrement step just addressed.
    fn read_predecrement(&mut self, reg: usize, size: Size) -> Result<u32, Trap> {
        if size != Size::Long {
            let addr = self.predecrement(reg, size);
            return self.read_loc(Loc::Mem(addr), size);
        }
        let low_at = self.state.a[reg].wrapping_sub(2);
        self.state.a[reg] = low_at;
        let low = self.read_word(low_at)?;
        let high_at = low_at.wrapping_sub(2);
        self.state.a[reg] = high_at;
        let high = self.read_word(high_at)?;
        Ok((u32::from(high) << 16) | u32::from(low))
    }

    /// `N`, `Z`, `V`, `C` (and `X` when `extend`) for an addition.
    fn set_add_flags(&mut self, src: u32, dst: u32, result: u32, size: Size, extend: bool) {
        let sign = size.sign_bit();
        let sm = src & sign != 0;
        let dm = dst & sign != 0;
        let rm = result & sign != 0;
        let carry = (sm && dm) || (!rm && (sm || dm));
        let overflow = (sm && dm && !rm) || (!sm && !dm && rm);
        self.set_flag(flags::N, rm);
        self.set_flag(flags::Z, result & size.mask() == 0);
        self.set_flag(flags::V, overflow);
        self.set_flag(flags::C, carry);
        if extend {
            self.set_flag(flags::X, carry);
        }
    }

    /// `N`, `Z`, `V`, `C` (and `X` when `extend`) for a subtraction.
    fn set_sub_flags(&mut self, src: u32, dst: u32, result: u32, size: Size, extend: bool) {
        let sign = size.sign_bit();
        let sm = src & sign != 0;
        let dm = dst & sign != 0;
        let rm = result & sign != 0;
        let borrow = (sm && !dm) || (rm && (sm || !dm));
        let overflow = (!sm && dm && !rm) || (sm && !dm && rm);
        self.set_flag(flags::N, rm);
        self.set_flag(flags::Z, result & size.mask() == 0);
        self.set_flag(flags::V, overflow);
        self.set_flag(flags::C, borrow);
        if extend {
            self.set_flag(flags::X, borrow);
        }
    }

    /// `ABCD`'s decimal addition.
    ///
    /// Binary addition with a nibble correction, which is what the hardware
    /// does — the difference matters because the operands are not required to
    /// be valid BCD and the result for invalid input is well defined even
    /// though the manual calls it undefined (M68000PRM, ABCD).
    fn bcd_add(&mut self, src: u32, dst: u32) -> u32 {
        let x = u32::from(self.state.flag(flags::X));
        let low = (src & 0x0f) + (dst & 0x0f) + x;
        let binary = (src & 0xff) + (dst & 0xff) + x;
        // The decimal carry is decided on the *binary* sum, before the low
        // nibble's correction is folded in. That distinction is only visible
        // for operands that are not valid BCD — `$2d + $69` corrects to `$9c`
        // with no carry, where testing after the correction would wrongly
        // carry — but the hardware is unambiguous about it and real code has
        // been known to rely on it.
        let carry = binary > 0x99;
        let mut result = binary;
        if low > 9 {
            result += 6;
        }
        if carry {
            result += 0x60;
        }
        let result = result & 0xff;
        self.set_flag(flags::C, carry);
        self.set_flag(flags::X, carry);
        self.set_flag(flags::N, result & 0x80 != 0);
        // V is documented as undefined and is not: it reports either decimal
        // correction carrying the result across the sign boundary, which is
        // the one thing the adder can cheaply notice.
        self.set_flag(flags::V, !binary & result & 0x80 != 0);
        if result != 0 {
            self.set_flag(flags::Z, false);
        }
        result
    }

    /// `SBCD`/`NBCD`'s decimal subtraction.
    fn bcd_sub(&mut self, src: u32, dst: u32) -> u32 {
        // Signed arithmetic, because two *different* borrows have to be told
        // apart and a `0x100` test conflates them once a value has gone more
        // than one place negative. The **binary** subtraction's borrow decides
        // whether the tens digit needs correcting; the borrow reported in C
        // and X is the one out of the units-corrected result. `$f0 - $ef`
        // borrows nothing in binary, needs no tens correction, and still sets
        // the carry — because subtracting the units correction is what takes
        // it below zero.
        let x = i32::from(self.state.flag(flags::X));
        let src = (src & 0xff) as i32;
        let dst = (dst & 0xff) as i32;
        let low = (dst & 0x0f) - (src & 0x0f) - x;
        let mut result = dst - src - x;
        let binary = (result as u32) & 0xff;
        let binary_borrow = result < 0;
        if low < 0 {
            result -= 6;
        }
        let borrow = result < 0;
        if binary_borrow {
            result -= 0x60;
        }
        let result = (result as u32) & 0xff;
        self.set_flag(flags::C, borrow);
        self.set_flag(flags::X, borrow);
        self.set_flag(flags::N, result & 0x80 != 0);
        // As for ABCD, against the uncorrected binary difference.
        self.set_flag(flags::V, binary & !result & 0x80 != 0);
        if result != 0 {
            self.set_flag(flags::Z, false);
        }
        result
    }
}

/// The fourteen-byte group-0 stack frame's extra fields.
#[derive(Debug, Clone, Copy)]
struct Group0 {
    ssw: u16,
    addr: u32,
    ir: u16,
}

/// Which binary operation an ALU row performs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum BinOp {
    Add,
    Sub,
    And,
    Or,
    Eor,
}

/// How much internal time an addressing mode costs beyond its bus cycles.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ExtraCycles {
    /// The mode is fetching an operand.
    Operand,
    /// The mode is `MOVE`'s destination, whose last absolute-long fetch is
    /// deferred past the write.
    MoveDest,
    /// The mode is only computing an address, as for `LEA` and `PEA`.
    Control,
}

impl ExtraCycles {
    /// The internal cycles an indexed mode costs before its refill.
    const fn index_delay(self) -> u32 {
        match self {
            ExtraCycles::Operand | ExtraCycles::MoveDest => 2,
            // LEA and PEA pay two more than an operand fetch does, because
            // nothing else in the instruction overlaps the adder.
            ExtraCycles::Control => 4,
        }
    }
}

/// The data register named by bits 11–9.
#[inline]
const fn reg_hi(opcode: u16) -> usize {
    ((opcode >> 9) & 7) as usize
}

/// The register named by bits 2–0.
#[inline]
const fn reg_lo(opcode: u16) -> usize {
    (opcode & 7) as usize
}

/// How far an autoincrement mode steps.
///
/// A byte access through `A7` steps by two, because the 68000 keeps the stack
/// pointer even; there is no such rule for `A0`–`A6` (M68000PRM §1.2).
#[inline]
const fn step(size: Size, reg: usize) -> u32 {
    match size {
        Size::Byte if reg == 7 => 2,
        other => other.bytes(),
    }
}

/// Replace the low `size` bits of `old` with `value`.
#[inline]
const fn merge(old: u32, value: u32, size: Size) -> u32 {
    (old & !size.mask()) | (value & size.mask())
}

/// `DIVU`'s data-dependent execution time.
///
/// The manual publishes only the maximum, 140 cycles (MC68000UM Table 8-6),
/// because the microcode's loop exits early — so this is derived from the
/// loop's shape instead, and checked against every division vector in the
/// conformance corpus.
///
/// The loop is restoring division, fifteen iterations, one quotient bit each.
/// What varies is how much work an iteration does: shifting a one out of the
/// top of the partial remainder means the subtraction is known to be needed
/// and costs four; otherwise the comparison costs six when the subtraction
/// happens and eight when it does not. An overflow — a quotient that will not
/// fit in sixteen bits — is detected before the loop starts and costs six.
fn divu_cycles(dividend: u32, divisor: u16) -> u32 {
    if divisor == 0 {
        return 0;
    }
    if (dividend >> 16) >= u32::from(divisor) {
        return 6;
    }
    let divisor = u32::from(divisor);
    let mut cycles = 12;
    let mut high = dividend >> 16;
    let mut low = dividend & 0xffff;
    for _ in 0..15 {
        let carried = high & 0x8000 != 0;
        high = ((high << 1) | (low >> 15)) & 0xffff;
        low = (low << 1) & 0xffff;
        if carried {
            high = high.wrapping_sub(divisor) & 0xffff;
            cycles += 4;
        } else if high >= divisor {
            high -= divisor;
            cycles += 6;
        } else {
            cycles += 8;
        }
    }
    cycles
}

/// `DIVS`'s data-dependent execution time, on the same footing.
///
/// The signed loop does not branch on the partial remainder the way `DIVU`'s
/// does, so its shape is simpler: a fixed cost that depends on the signs, two
/// cycles for every *zero* bit of the quotient's magnitude, and two more when
/// its lowest bit is set. Overflow is again decided before the loop.
fn divs_cycles(dividend: i32, divisor: i16) -> u32 {
    if divisor == 0 {
        return 0;
    }
    let magnitude = dividend.unsigned_abs() / u32::from(divisor.unsigned_abs());
    let negative = (dividend < 0) != (divisor < 0);
    let limit = if negative { 0x8000 } else { 0x7fff };
    if magnitude > limit {
        return 12 + if dividend < 0 { 2 } else { 0 };
    }
    // The overflow test above caps the magnitude at $8000, which has exactly
    // one bit set, so this subtraction has fifteen to give. Raising that cap
    // would underflow it.
    debug_assert!(magnitude <= 0x8000);
    let zeros = 15 - (magnitude & 0xffff).count_ones();
    116 + if dividend < 0 { 4 } else { 0 }
        + if negative { 2 } else { 0 }
        + 2 * zeros
        + 2 * (magnitude & 1)
}
