//! The interpreter: decode, execute, and the flag rules.
//!
//! # The bus is the clock
//!
//! Every guest access goes through [`AddressSpace`] one transfer at a time and
//! charges the part's bus-cycle cost — four clocks on an 8088 with no wait
//! states, two on a 386. There is no table of instruction lengths:
//! `add [bp+di-64h], cl` reads its operand, writes it back, and pays for both
//! because the accesses happened, and a device watching the bus sees them in
//! the order hardware would.
//!
//! What this does *not* model is the overlap between the bus interface unit
//! and the execution unit. A real 8088 prefetches while the previous
//! instruction is still executing, so its instruction time is roughly the
//! larger of the two rather than their sum. Here the prefetch queue is filled
//! at instruction boundaries and refilled on demand, and every fetch is
//! charged, so a cycle count taken over a long run is an upper bound. That is
//! a deliberate trade: getting the T-state-exact overlap right means modelling
//! the microcode, and the accuracy that actually matters for correctness —
//! *which* accesses happen, in what order, with what values — is checked
//! against hardware in [`conformance`](super).
//!
//! # One interpreter, four parts
//!
//! [`Variant`] selects the behaviour rather than a second module, because the
//! generations really are close to a superset chain. The places where they are
//! not are each marked in the code and listed here, because they are exactly
//! the places a "just widen it to 32 bits" rewrite silently breaks:
//!
//! | Behaviour | 8086/8088 | 80386/80486 |
//! | --- | --- | --- |
//! | Address arithmetic | `segment << 4 + offset`, 20 bits, wraps at 1 MiB | cached base `+` offset, 32 bits, limit-checked, then paged |
//! | An offset past the end of a segment | wraps within the segment | `#GP(0)`, or `#SS(0)` through `SS` |
//! | `PUSH SP` | pushes the **decremented** value | pushes the value before the decrement |
//! | Shift and rotate count | the whole of `CL` | `CL & 31` |
//! | A shift by zero | still writes the operand back | does nothing at all |
//! | `#DE`'s return address | the **next** instruction | the faulting instruction |
//! | `MOV Sreg,r/m` with `reg` 4-7 | aliases down onto `ES`-`DS` | `FS`, `GS`, then `#UD` |
//! | `0F` | `POP CS` | the two-byte opcode escape |
//! | `60`-`6F` | aliases of `JO`-`JG` | `PUSHA` through `OUTSW` |
//! | An unassigned encoding | does something | `#UD` |
//!
//! # The undefined flags
//!
//! Intel documents a dozen instructions as leaving some flags undefined. The
//! silicon is not undefined, merely unspecified, and software has been found
//! to depend on it. Each of the following was measured against
//! `SingleStepTests/8088` — hardware output, not anyone's emulator — rather
//! than guessed, and each is implemented where it is described:
//!
//! | Instruction | Officially undefined | What the 8088 actually does | Vectors |
//! | --- | --- | --- | --- |
//! | `AND`, `OR`, `XOR`, `TEST` | `AF` | cleared, always ([`Exec::logic_flags`]) | exact |
//! | `SHL`/`SAL` | `AF` | bit 4 of the result — the microcode is an `ADD dst,dst` ([`Exec::shift_value`]) | exact |
//! | `SHR`, `SAR` | `AF` | cleared | exact |
//! | `MUL` | `SF`, `ZF`, `AF`, `PF` | set from the **high half** of the product: `ZF` if it is zero, `SF` from its top bit, `PF` from its low byte, `AF` cleared ([`Exec::mul_flags`]) | exact |
//! | `DAA`, `DAS` | `OF` | from the *single* correcting add or subtract of `0x00`/`0x06`/`0x60`/`0x66`; and the high correction's threshold moves with `AF` ([`Exec::decimal_adjust`]) | exact |
//! | `AAA`, `AAS` | `OF`, `SF`, `ZF`, `PF` | from an 8-bit `AL ± 6` that happens even when no adjustment is needed, with an operand of zero ([`Exec::ascii_adjust`]) | exact |
//! | `AAM 0` | all | the flags of a zero result, then a divide error ([`Exec::aam`]) | exact |
//! | `AAD` | `OF`, `AF`, `CF` | the adjustment really is an addition and sets them ([`Exec::aad`]) | exact |
//! | `SETMO` (`D0`-`D3` `/6`) | all | the operand becomes all ones and the flags follow it as a logical result | exact |
//! | `IMUL` | `SF`, `ZF`, `AF`, `PF` | the sign-corrected magnitude loop's residue; modelled as `MUL`'s rule | ~66% |
//! | `DIV`, `IDIV` | all | `CF` is the complement of the quotient's top bit (exact); the rest are the last trial subtraction's ([`Exec::cord`]) | ~35% |
//!
//! The last two rows are the whole of `conformance::KNOWN_FAILURES`, and the
//! percentages there are measured rather than estimated.
//!
//! One result that is not a flag belongs in the same list, because it is the
//! same kind of surprise: a shift or rotate **by zero** still writes its
//! operand back *on an 8086*. Nothing changes and no flag moves, but the write
//! happens on the bus, where a memory-mapped device would see it.
//!
//! # Faults restart the instruction
//!
//! Every memory access returns a [`Result`], and a fault propagates out of the
//! instruction with `?`. [`Exec::instruction`] snapshots the register file
//! before decoding and restores it on the way out, so `CS:EIP` points at the
//! *first byte of the faulting instruction*, prefixes included, which is what
//! a restartable fault means and what demand paging needs. Writes that already
//! reached the bus are not undone — no interpreter can undo them, and the
//! architecture does not require it for the instructions that can fault
//! part-way.
//!
//! # Sources
//!
//! Intel's *iAPX 86/88 User's Manual* for the 8086 instruction semantics, the
//! flag definitions and the timing tables; the *80386 Programmer's Reference
//! Manual* for everything from chapter 2 onward — the 32-bit forms, the
//! protection model, paging, and the exception vectors and error codes; and
//! `docs/cpu/x86.md` for the rest. The undefined-flag column above is
//! measurement, and the measurement is reproducible by anyone with the corpus.
//! No copyleft emulator was consulted.

use alloc::vec::Vec;

use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

use super::isa::{self, Arg, Bits, Fields, Op, Rep, seg};
use super::paging::{self, Tlb};
use super::prot::{self, Sys, cr0};
use super::{Config, Lines, Regs, Variant, flags, linear};

/// Clocks the reset sequence spends before the first fetch.
const RESET_CLOCKS: u32 = 7;

/// Type 0: divide error.
pub(super) const VEC_DIVIDE: u8 = 0;
/// Type 1: debug — single step, and the 386's `ICEBP`.
pub(super) const VEC_DEBUG: u8 = 1;
/// Type 2: NMI.
pub(super) const VEC_NMI: u8 = 2;
/// Type 3: the one-byte breakpoint.
pub(super) const VEC_BREAKPOINT: u8 = 3;
/// Type 4: `INTO`.
pub(super) const VEC_OVERFLOW: u8 = 4;
/// Type 5: `BOUND` found the index outside its limits.
pub(super) const VEC_BOUND: u8 = 5;
/// Type 6: invalid opcode.
pub(super) const VEC_UD: u8 = 6;
/// Type 7: no coprocessor — `CR0.EM` or `CR0.TS` and an escape instruction.
pub(super) const VEC_NM: u8 = 7;
/// Type 8: double fault.
pub(super) const VEC_DF: u8 = 8;
/// Type 10: an invalid task state segment.
pub(super) const VEC_TS: u8 = 10;
/// Type 11: a segment the descriptor says is not present.
pub(super) const VEC_NP: u8 = 11;
/// Type 12: a stack-segment fault.
pub(super) const VEC_SS: u8 = 12;
/// Type 13: general protection.
pub(super) const VEC_GP: u8 = 13;
/// Type 14: page fault.
pub(super) const VEC_PF: u8 = 14;

// ---------------------------------------------------------------------------
// Faults
// ---------------------------------------------------------------------------

/// One exception on its way out of an instruction.
///
/// The error code is an [`Option`] rather than a zero because whether one is
/// pushed at all is part of the vector's definition: a handler for `#GP` pops
/// one, a handler for `#UD` does not, and pushing a spurious zero desynchronises
/// the stack in a way that is very hard to see afterwards.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Fault {
    /// The interrupt vector to take.
    pub vector: u8,
    /// The error code, where the vector has one.
    pub error: Option<u32>,
}

impl Fault {
    /// A fault with no error code.
    pub(super) const fn bare(vector: u8) -> Fault {
        Fault {
            vector,
            error: None,
        }
    }

    /// A fault with an error code.
    pub(super) const fn coded(vector: u8, error: u32) -> Fault {
        Fault {
            vector,
            error: Some(error),
        }
    }

    /// `#GP` with a selector, or with zero where the fault is not about one.
    pub(super) const fn gp(error: u32) -> Fault {
        Fault::coded(VEC_GP, error)
    }

    /// Whether this vector is "contributory" for the purposes of the
    /// double-fault rule: an exception that arises from executing an
    /// instruction, as opposed to one the program asked for.
    ///
    /// 80386 Programmer's Reference Manual §9.8.4.2, "Double Fault".
    pub(super) const fn is_contributory(self) -> bool {
        matches!(self.vector, VEC_DIVIDE | VEC_TS | VEC_NP | VEC_SS | VEC_GP)
    }
}

/// The result of anything that can fault.
pub(super) type Ex<T> = Result<T, Fault>;

// ---------------------------------------------------------------------------
// The prefetch queue
// ---------------------------------------------------------------------------

/// The bus interface unit's instruction queue.
///
/// Four bytes on an 8088, six on an 8086, sixteen on a 386. It is not an
/// implementation detail on the 8086: the queue status lines are pins, an
/// interrupted string instruction restarts from it, and self-modifying code
/// that writes within a few bytes of `IP` behaves differently because of it —
/// which is why `docs/cpu/x86.md` calls self-modifying code mandatory rather
/// than optional.
///
/// The queue holds the bytes at `CS:IP` through `CS:IP+len-1`, so the offset
/// the bus interface unit fetches next is always `IP + len`. Keeping that
/// invariant rather than a separate fetch pointer means a snapshot of `IP` and
/// the queue contents is a complete description.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct Queue {
    bytes: [u8; 16],
    len: u8,
    depth: u8,
}

impl Queue {
    /// An empty queue of the depth this part has.
    pub(super) const fn new(variant: Variant) -> Queue {
        Queue {
            bytes: [0; 16],
            len: 0,
            depth: variant.queue_bytes(),
        }
    }

    /// Discard everything, as a control transfer does.
    pub(super) const fn flush(&mut self) {
        self.len = 0;
    }

    /// How many bytes are queued.
    pub(super) const fn len(&self) -> u8 {
        self.len
    }

    /// How many bytes the queue holds when full.
    pub(super) const fn depth(&self) -> u8 {
        self.depth
    }

    fn push(&mut self, byte: u8) {
        if self.len < self.depth {
            self.bytes[self.len as usize] = byte;
            self.len += 1;
        }
    }

    fn pop(&mut self) -> Option<u8> {
        if self.len == 0 {
            return None;
        }
        let byte = self.bytes[0];
        // Shifting a few bytes down beats the modular arithmetic a ring buffer
        // would need on every single instruction byte.
        let mut i = 1usize;
        while i < self.len as usize {
            self.bytes[i - 1] = self.bytes[i];
            i += 1;
        }
        self.len -= 1;
        Some(byte)
    }

    /// The queued bytes, oldest first.
    pub(super) fn contents(&self) -> Vec<u8> {
        self.bytes[..self.len as usize].to_vec()
    }

    /// Replace the contents, as if the bus interface unit had fetched them.
    ///
    /// # Errors
    ///
    /// If more bytes are given than the queue holds.
    pub(super) fn install(&mut self, bytes: &[u8]) -> Result<(), ()> {
        if bytes.len() > self.depth as usize {
            return Err(());
        }
        self.len = bytes.len() as u8;
        self.bytes[..bytes.len()].copy_from_slice(bytes);
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Execution state
// ---------------------------------------------------------------------------

/// The architectural state one core owns.
///
/// Split from [`super::X86`] because the interrupt *lines* live outside the
/// lock: a device asserting `INTR` from inside a CPU-initiated MMIO write
/// would otherwise re-enter the CPU's own critical section and deadlock (the
/// re-entrancy contract, `ROADMAP.md` §4.7).
#[derive(Debug, Clone, Copy)]
pub(super) struct State {
    /// The general register file.
    pub regs: Regs,
    /// The system registers: segment caches, descriptor tables, `CR0`-`CR3`.
    pub sys: Sys,
    /// The translation-lookaside buffer. Derived state: never serialized.
    pub tlb: Tlb,
    /// Clock cycles executed since power-on.
    pub cycles: u64,
    /// Set by `HLT`; cleared by any interrupt or reset.
    pub halted: bool,
    /// Set by a triple fault. Only a reset clears it, which is what the
    /// `SHUTDOWN` bus cycle means: the chipset is expected to notice and pull
    /// `RESET`.
    pub shutdown: bool,
    /// A reset was requested and its sequence has not run yet.
    pub reset_pending: bool,
    /// Interrupts are inhibited for the next instruction.
    ///
    /// Set by `MOV SS,x`, `POP SS`, `LSS` and `STI` so that the `SS:ESP` pair
    /// can be reloaded atomically.
    pub int_shadow: bool,
    /// The bus interface unit's queue.
    pub queue: Queue,
    /// The last value driven on the data bus.
    ///
    /// An 8086 has no bus-error input: an access nothing answers leaves the
    /// previous value on the bus, and that is what the CPU reads.
    pub open_bus: u8,
    /// How many accesses an address space refused.
    pub faults: u64,
    /// Physical address of the most recent refused access.
    pub last_fault: u64,
    /// Clocks owed to the next scheduler budget.
    ///
    /// An x86 cannot be stopped mid-instruction, so a budget that runs out
    /// part-way through one is overshot. The scheduler refuses a `Consumed`
    /// larger than the budget it handed out — rightly, since that would put
    /// the domain ahead of the timeline — so the overshoot is carried here and
    /// charged against the next budget instead. Architectural, because a
    /// restored machine that forgot its debt runs one instruction free.
    pub debt: u64,
}

impl State {
    /// Power-on state, before the reset sequence has run.
    pub(super) fn new(variant: Variant) -> State {
        let (regs, sys) = if variant.is_32bit() {
            let mut regs = Regs::new();
            regs.cs = 0xf000;
            regs.rip = 0xfff0;
            regs.eflags = flags::ALWAYS_SET;
            regs.rdx = u64::from(variant.reset_signature());
            (regs, Sys::reset())
        } else {
            (Regs::new(), Sys::reset_8086())
        };
        State {
            regs,
            sys,
            tlb: Tlb::new(),
            cycles: 0,
            halted: false,
            shutdown: false,
            reset_pending: true,
            int_shadow: false,
            queue: Queue::new(variant),
            open_bus: 0,
            faults: 0,
            last_fault: 0,
            debt: 0,
        }
    }
}

/// One step's worth of execution, borrowing everything it needs.
///
/// Created per step rather than stored: it holds the per-instruction
/// bookkeeping — the effective address, the restart point — that is
/// meaningless between instructions, and dropping it makes that explicit.
pub(super) struct Exec<'a> {
    pub(super) state: &'a mut State,
    pub(super) mem: &'a AddressSpace,
    pub(super) io: Option<&'a AddressSpace>,
    pub(super) cfg: &'a Config,
    pub(super) lines: &'a Lines,
    pub(super) attrs: MemAttrs,
    /// The memory operand's `(segment register, offset)`, computed once.
    pub(super) ea: Option<(u8, u64)>,
    /// The register file as it was before this instruction began, so a fault
    /// can restart it.
    pub(super) entry: Regs,
    /// Where the current instruction started, prefixes included, so a string
    /// operation interrupted between iterations can be restarted.
    pub(super) start_ip: u64,
    /// Clocks this step has charged.
    pub(super) used: u64,
    /// How deep the exception delivery is: 0 while executing, 1 while
    /// delivering a fault, 2 while delivering a double fault.
    pub(super) nesting: u8,
}

impl<'a> Exec<'a> {
    /// Borrow a core for one step.
    pub(super) fn new(
        state: &'a mut State,
        mem: &'a AddressSpace,
        io: Option<&'a AddressSpace>,
        cfg: &'a Config,
        lines: &'a Lines,
    ) -> Exec<'a> {
        let attrs = MemAttrs::DEFAULT.with_requester(cfg.requester);
        let entry = state.regs;
        Exec {
            state,
            mem,
            io,
            cfg,
            lines,
            attrs,
            ea: None,
            entry,
            start_ip: 0,
            used: 0,
            nesting: 0,
        }
    }

    /// Which part this is.
    #[inline]
    pub(super) fn variant(&self) -> Variant {
        self.cfg.variant
    }

    /// Whether this part predates protected mode, and therefore uses the
    /// 8086's address arithmetic and its exception behaviour.
    #[inline]
    pub(super) fn legacy(&self) -> bool {
        !self.cfg.variant.is_32bit()
    }

    /// Whether the processor is in protected mode.
    #[inline]
    pub(super) fn protected(&self) -> bool {
        !self.legacy() && self.state.sys.protected()
    }

    /// The current privilege level.
    ///
    /// The low two bits of the `CS` selector in protected mode; zero in real
    /// mode, which is not a special case so much as the only level real mode
    /// has.
    #[inline]
    pub(super) fn cpl(&self) -> u8 {
        if self.protected() {
            (self.state.regs.cs & 3) as u8
        } else {
            0
        }
    }

    /// Whether this part can enter long mode at all.
    #[inline]
    pub(super) fn has_long(&self) -> bool {
        self.cfg.features.long
    }

    /// Whether the processor is in 64-bit mode right now.
    ///
    /// Long mode active **and** the current code segment's `L` bit set —
    /// compatibility mode answers false, and that is the distinction every
    /// decode and address decision below turns on.
    #[inline]
    pub(super) fn sixty_four(&self) -> bool {
        self.has_long() && self.state.sys.sixty_four()
    }

    /// The width the current code segment is decoded at.
    #[inline]
    pub(super) fn code_bits(&self) -> Bits {
        if self.legacy() {
            return Bits::B16;
        }
        if self.sixty_four() {
            return Bits::B64;
        }
        if self.state.sys.seg(seg::CS).big() {
            Bits::B32
        } else {
            Bits::B16
        }
    }

    /// Run one reset sequence, interrupt sequence, or instruction.
    ///
    /// Returns the clocks charged — zero only when the CPU is halted with
    /// nothing pending, which the caller must notice rather than spin on.
    pub(super) fn step(&mut self) -> u64 {
        if self.state.reset_pending {
            self.reset_sequence();
            return self.used;
        }
        if self.state.shutdown {
            // A triple fault stops the processor until `RESET`. Charging
            // nothing is what tells the scheduler to stop rather than spin.
            return 0;
        }

        // The shadow suppresses the interrupt check for exactly one
        // instruction. It is read here and cleared below, so an instruction
        // that sets it again extends it by one more.
        let shadow = self.state.int_shadow;
        if !shadow {
            if self.lines.take_nmi_pending() {
                self.state.halted = false;
                self.charge(Op::INT.clocks());
                self.entry = self.state.regs;
                self.deliver(Fault::bare(VEC_NMI));
                return self.used;
            }
            if self.flag(flags::IF) && self.lines.intr_pending() {
                self.state.halted = false;
                // Two INTA bus cycles; a PC's 8259A drives the vector onto the
                // data bus during the second.
                self.charge(2 * self.variant().bus_clocks() + Op::INT.clocks());
                // The acknowledge cycle proper: whatever drives `INTR` answers
                // it with a vector, and moves the request from pending to in
                // service on its own side. A net with no `IntAck` on it falls
                // back to the latched byte, which is what a test that drives
                // the pin by hand sets.
                let vector = self.lines.acknowledge();
                self.entry = self.state.regs;
                self.deliver(Fault::bare(vector));
                return self.used;
            }
        }

        if self.state.halted {
            return 0;
        }

        // `TF` is sampled before the instruction runs, which is why a `POPF`
        // that sets it does not trap on itself.
        let trap = self.flag(flags::TF) && !shadow;
        self.state.int_shadow = false;
        if let Err(fault) = self.instruction() {
            // The register file is rolled back first: `CS:EIP` must name the
            // faulting instruction, not the point inside it where the fault
            // was noticed. That is what makes a page fault restartable.
            //
            // **Except on an 8086**, which has no restartable faults at all:
            // its only exception is the divide error, and what that pushes is
            // the address of the *next* instruction. `IP` has already been
            // advanced past the divide by the time we get here, so leaving it
            // alone is exactly right — and it is a difference generic x86
            // emulators habitually get wrong.
            if !self.legacy() {
                self.state.regs = self.entry;
            } else {
                self.entry = self.state.regs;
            }
            self.state.queue.flush();
            self.deliver(fault);
            return self.used;
        }
        if trap && !self.state.int_shadow {
            self.charge(Op::INT.clocks());
            // A single-step trap fires *after* the instruction, so the state
            // it saves is the state the instruction produced.
            self.entry = self.state.regs;
            self.deliver(Fault::bare(VEC_DEBUG));
        }
        self.used
    }

    /// Deliver an event, escalating to a double fault and then to shutdown as
    /// the architecture says.
    ///
    /// 80386 Programmer's Reference Manual §9.8.4.2 and its table: a second
    /// exception raised *while delivering* the first becomes `#DF` only for
    /// certain pairs; every other pair is handled one after the other. And
    /// anything that faults while `#DF` itself is being delivered shuts the
    /// processor down — real silicon drives a `SHUTDOWN` bus cycle and waits
    /// for `RESET`, which the chipset is expected to notice.
    fn deliver(&mut self, first: Fault) {
        let mut current = first;
        // Three attempts is the whole ladder: the event, the double fault, and
        // the shutdown that follows a fault during the double fault.
        for _ in 0..3 {
            self.nesting = self.nesting.saturating_add(1);
            match self.take_interrupt(current.vector, current.error) {
                Ok(()) => {
                    self.nesting = 0;
                    return;
                }
                Err(second) => {
                    if current.vector == VEC_DF {
                        self.state.shutdown = true;
                        self.state.halted = true;
                        self.nesting = 0;
                        return;
                    }
                    self.state.regs = self.entry;
                    self.state.queue.flush();
                    current = if Self::escalates(current, second) {
                        Fault::coded(VEC_DF, 0)
                    } else {
                        second
                    };
                }
            }
        }
        self.state.shutdown = true;
        self.state.halted = true;
        self.nesting = 0;
    }

    /// Whether a second exception raised while delivering the first becomes a
    /// double fault rather than being taken in its place.
    ///
    /// The manual's table in two lines. A benign second exception is always
    /// taken on its own; a contributory one doubles against a contributory or
    /// a page fault; a page fault doubles only against another page fault.
    const fn escalates(first: Fault, second: Fault) -> bool {
        if second.vector == VEC_PF {
            return first.vector == VEC_PF;
        }
        if second.is_contributory() {
            return first.is_contributory() || first.vector == VEC_PF;
        }
        false
    }

    // -----------------------------------------------------------------
    // The clock and the bus
    // -----------------------------------------------------------------

    pub(super) fn charge(&mut self, clocks: u32) {
        let clocks = u64::from(clocks);
        self.used += clocks;
        self.state.cycles = self.state.cycles.wrapping_add(clocks);
    }

    /// One physical bus read of one, two or four bytes.
    ///
    /// The whole access is one bus transaction. Splitting a wide access into
    /// bytes is the *caller's* job where the part or the page boundary
    /// requires it, so that the 8088's two-cycle word read stays visible in
    /// the trace exactly as the corpus records it.
    pub(super) fn phys_read(&mut self, addr: u64, size: u8) -> u64 {
        self.charge(self.variant().bus_clocks());
        let width = Self::width_of(size);
        let addr = self.masked(addr);
        match self.mem.read(addr, width, self.attrs) {
            Ok(value) => {
                self.state.open_bus = (value >> ((size as u32 - 1) * 8)) as u8;
                value
            }
            Err(_) => {
                // No bus-error input exists on these parts, so the honest
                // model is open bus — and the fault counter is how anyone
                // finds out.
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = addr;
                let byte = u64::from(self.state.open_bus);
                // The same byte on every lane, for as many lanes as the
                // transfer had.
                byte.wrapping_mul(0x0101_0101_0101_0101) & Self::mask(size)
            }
        }
    }

    /// One physical bus write of one, two, four or eight bytes.
    pub(super) fn phys_write(&mut self, addr: u64, size: u8, value: u64) {
        self.charge(self.variant().bus_clocks());
        self.state.open_bus = (value >> ((size as u32 - 1) * 8)) as u8;
        let width = Self::width_of(size);
        let addr = self.masked(addr);
        if self.mem.write(addr, width, value, self.attrs).is_err() {
            self.state.faults = self.state.faults.wrapping_add(1);
            self.state.last_fault = addr;
        }
    }

    /// The access width for a byte count.
    #[inline]
    const fn width_of(size: u8) -> Width {
        match size {
            1 => Width::U8,
            2 => Width::U16,
            4 => Width::U32,
            _ => Width::U64,
        }
    }

    /// A physical address as it reaches the bus, with the A20 gate applied.
    ///
    /// The gate masks bit 20 of a *20-or-21-bit* address; it has nothing to
    /// say about the bits above 32, so the mask is applied to the low half and
    /// the rest passes through. A 64-bit guest with the gate shut would
    /// otherwise lose every address above 4 GiB.
    #[inline]
    fn masked(&self, addr: u64) -> u64 {
        (addr & !0xffff_ffffu64) | (addr & u64::from(self.lines.a20_mask()))
    }

    /// Whether a word at this physical address and segment offset is one bus
    /// cycle on this part.
    ///
    /// Only on an 8086, and only when the transfer is aligned and does not
    /// straddle the end of the segment — an 8088 has eight data pins and
    /// always takes two.
    fn word_is_one_cycle(&self, base: u64, offset: u16) -> bool {
        self.cfg.variant.bus_bytes() == 2 && base.is_multiple_of(2) && offset != 0xffff
    }

    // -- The 8086's memory path ----------------------------------------
    //
    // Kept whole and separate rather than folded into the 32-bit path,
    // because its address arithmetic, its wraparound and its bus splitting
    // are all checked against three million hardware vectors and none of the
    // three survives being generalised.

    /// Read a word at an explicit segment value, the 8086's way.
    ///
    /// The *offset* wraps, not the physical address: a word at offset `0xffff`
    /// takes its high byte from offset `0x0000` of the same segment. Both
    /// halves then go through the same 20-bit adder, so a segment near the top
    /// of memory wraps at 1 MiB exactly as a byte access would.
    fn legacy_read16_seg(&mut self, segment: u16, offset: u16) -> u16 {
        let base = linear(segment, offset);
        if self.word_is_one_cycle(base, offset) {
            return self.phys_read(base, 2) as u16;
        }
        let lo = self.phys_read(base, 1) as u8;
        let hi = self.phys_read(linear(segment, offset.wrapping_add(1)), 1) as u8;
        u16::from(lo) | (u16::from(hi) << 8)
    }

    /// Write a word at an explicit segment value, low byte first.
    fn legacy_write16_seg(&mut self, segment: u16, offset: u16, value: u16) {
        let base = linear(segment, offset);
        if self.word_is_one_cycle(base, offset) {
            self.phys_write(base, 2, u64::from(value));
            return;
        }
        self.phys_write(base, 1, u64::from(value & 0xff));
        self.phys_write(
            linear(segment, offset.wrapping_add(1)),
            1,
            u64::from(value >> 8),
        );
    }

    // -- The common memory interface -----------------------------------

    /// Read `size` bytes from `sr:offset`.
    pub(super) fn read_mem(&mut self, sr: u8, offset: u64, size: u8) -> Ex<u64> {
        if self.legacy() {
            let segment = self.state.regs.segment(sr);
            return Ok(match size {
                1 => u64::from(self.phys_read(linear(segment, offset as u16), 1) as u8),
                _ => u64::from(self.legacy_read16_seg(segment, offset as u16)),
            });
        }
        let lin = self.seg_linear(sr, offset, u64::from(size), false)?;
        self.linear_read(lin, size)
    }

    /// Write `size` bytes to `sr:offset`.
    pub(super) fn write_mem(&mut self, sr: u8, offset: u64, size: u8, value: u64) -> Ex<()> {
        if self.legacy() {
            let segment = self.state.regs.segment(sr);
            match size {
                1 => self.phys_write(linear(segment, offset as u16), 1, value & 0xff),
                _ => self.legacy_write16_seg(segment, offset as u16, value as u16),
            }
            return Ok(());
        }
        let lin = self.seg_linear(sr, offset, u64::from(size), true)?;
        self.linear_write(lin, size, value)
    }

    /// Read `size` bytes from a linear address, splitting the access if it
    /// straddles a page boundary.
    pub(super) fn linear_read(&mut self, lin: u64, size: u8) -> Ex<u64> {
        let user = self.cpl() == 3;
        if !self.state.sys.paging() {
            return Ok(self.phys_read(lin, size));
        }
        if Self::crosses_page(lin, size) {
            let mut value = 0u64;
            for i in 0..u64::from(size) {
                let addr = lin.wrapping_add(i);
                let phys = self.translate(addr, false, user)?;
                value |= self.phys_read(phys, 1) << (8 * i);
            }
            return Ok(value);
        }
        let phys = self.translate(lin, false, user)?;
        Ok(self.phys_read(phys, size))
    }

    /// Write `size` bytes to a linear address.
    ///
    /// A write that straddles a page boundary translates **both** pages before
    /// writing either byte, so a fault on the second page does not leave the
    /// first half written. That is what the architecture guarantees and what a
    /// naive byte loop gets wrong.
    pub(super) fn linear_write(&mut self, lin: u64, size: u8, value: u64) -> Ex<()> {
        let user = self.cpl() == 3;
        if !self.state.sys.paging() {
            self.phys_write(lin, size, value);
            return Ok(());
        }
        if Self::crosses_page(lin, size) {
            let mut phys = [0u64; 8];
            for i in 0..u64::from(size) {
                phys[i as usize] = self.translate(lin.wrapping_add(i), true, user)?;
            }
            for i in 0..u64::from(size) {
                self.phys_write(phys[i as usize], 1, (value >> (8 * i)) & 0xff);
            }
            return Ok(());
        }
        let phys = self.translate(lin, true, user)?;
        self.phys_write(phys, size, value);
        Ok(())
    }

    /// Whether an access of `size` bytes at `lin` spans two pages.
    #[inline]
    fn crosses_page(lin: u64, size: u8) -> bool {
        (lin & 0xfff) + u64::from(size) > 0x1000
    }

    /// Read from a linear address as the processor itself does, ignoring
    /// segmentation and privilege: descriptor tables, page tables and task
    /// state segments are all read this way.
    ///
    /// Privilege does not apply because there is no instruction operand here —
    /// the processor is walking its own structures — and the 386 walks them
    /// with paging *on*, which is why this goes through translation rather
    /// than straight to the bus.
    pub(super) fn sys_read(&mut self, lin: u64, size: u8) -> Ex<u64> {
        if !self.state.sys.paging() {
            return Ok(self.phys_read(lin, size));
        }
        if Self::crosses_page(lin, size) {
            let mut value = 0u64;
            for i in 0..u64::from(size) {
                let phys = self.translate(lin.wrapping_add(i), false, false)?;
                value |= self.phys_read(phys, 1) << (8 * i);
            }
            return Ok(value);
        }
        let phys = self.translate(lin, false, false)?;
        Ok(self.phys_read(phys, size))
    }

    /// Write to a linear address as the processor itself does.
    pub(super) fn sys_write(&mut self, lin: u64, size: u8, value: u64) -> Ex<()> {
        if !self.state.sys.paging() {
            self.phys_write(lin, size, value);
            return Ok(());
        }
        if Self::crosses_page(lin, size) {
            let mut phys = [0u64; 8];
            for i in 0..u64::from(size) {
                phys[i as usize] = self.translate(lin.wrapping_add(i), true, false)?;
            }
            for i in 0..u64::from(size) {
                self.phys_write(phys[i as usize], 1, (value >> (8 * i)) & 0xff);
            }
            return Ok(());
        }
        let phys = self.translate(lin, true, false)?;
        self.phys_write(phys, size, value);
        Ok(())
    }

    /// Read thirty-two bits the same way.
    pub(super) fn sys_read32(&mut self, lin: u64) -> Ex<u32> {
        Ok(self.sys_read(lin, 4)? as u32)
    }

    /// Write thirty-two bits the same way.
    pub(super) fn sys_write32(&mut self, lin: u64, value: u32) -> Ex<()> {
        self.sys_write(lin, 4, u64::from(value))
    }

    /// Read sixteen bits the same way.
    pub(super) fn sys_read16(&mut self, lin: u64) -> Ex<u32> {
        Ok(self.sys_read(lin, 2)? as u32)
    }

    // -- I/O -----------------------------------------------------------

    /// How wide an I/O transfer may be.
    ///
    /// The I/O space is thirty-two bits wide and stayed that way: `REX.W` on
    /// an `IN`, `OUT`, `INS` or `OUTS` is **ignored** rather than widening the
    /// transfer, because there is no 64-bit port cycle for it to mean (*Intel
    /// SDM* volume 2, `IN`/`OUT`). Clamping here rather than at each of the
    /// four call sites keeps the rule in one place.
    #[inline]
    const fn io_width(opsize: u8) -> u8 {
        if opsize > 4 { 4 } else { opsize }
    }

    /// One I/O read. A core with no I/O space sees an unterminated bus, which
    /// reads as ones — the same answer the corpus expects from a bare 8088.
    pub(super) fn io_read(&mut self, port: u16, size: u8) -> u32 {
        self.charge(self.variant().bus_clocks());
        let Some(io) = self.io else {
            return match size {
                1 => 0xff,
                2 => 0xffff,
                _ => 0xffff_ffff,
            };
        };
        let width = match size {
            1 => Width::U8,
            2 => Width::U16,
            _ => Width::U32,
        };
        match io.read(u64::from(port), width, self.attrs) {
            Ok(value) => value as u32,
            Err(_) => {
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = u64::from(port);
                match size {
                    1 => 0xff,
                    2 => 0xffff,
                    _ => 0xffff_ffff,
                }
            }
        }
    }

    pub(super) fn io_write(&mut self, port: u16, size: u8, value: u32) {
        self.charge(self.variant().bus_clocks());
        let Some(io) = self.io else {
            return;
        };
        let width = match size {
            1 => Width::U8,
            2 => Width::U16,
            _ => Width::U32,
        };
        if io
            .write(u64::from(port), width, u64::from(value), self.attrs)
            .is_err()
        {
            self.state.faults = self.state.faults.wrapping_add(1);
            self.state.last_fault = u64::from(port);
        }
    }

    /// An 8-bit I/O read, split into byte transactions where the part's data
    /// bus is narrower than the operand.
    fn io_read_sized(&mut self, port: u16, size: u8) -> u32 {
        if self.legacy() && size == 2 {
            // An 8088 drives two byte cycles; the corpus records both.
            let lo = self.io_read(port, 1);
            let hi = self.io_read(port.wrapping_add(1), 1);
            return lo | (hi << 8);
        }
        self.io_read(port, size)
    }

    fn io_write_sized(&mut self, port: u16, size: u8, value: u32) {
        if self.legacy() && size == 2 {
            self.io_write(port, 1, value & 0xff);
            self.io_write(port.wrapping_add(1), 1, (value >> 8) & 0xff);
            return;
        }
        self.io_write(port, size, value);
    }

    /// Check whether this privilege level may touch a port at all.
    ///
    /// Below `IOPL` it always may. At or above it, the task state segment's
    /// I/O permission bitmap decides one port at a time — the mechanism that
    /// lets a user-mode driver own a device without owning the machine.
    /// A bitmap that does not fit inside the segment limit denies access,
    /// which is what makes "no bitmap" mean "no ports".
    fn io_permitted(&mut self, port: u16, size: u8) -> Ex<()> {
        if !self.protected() {
            return Ok(());
        }
        if self.cpl() <= self.state.regs.iopl() {
            return Ok(());
        }
        let tss = self.state.sys.task;
        if !tss.present() {
            return Err(Fault::gp(0));
        }
        let map_base = self.sys_read16(tss.base.wrapping_add(0x66))? & 0xffff;
        for i in 0..u32::from(size) {
            let bit = u32::from(port) + i;
            let offset = map_base + (bit >> 3);
            if offset > tss.limit {
                return Err(Fault::gp(0));
            }
            let byte = {
                let lin = tss.base.wrapping_add(u64::from(offset));
                if self.state.sys.paging() {
                    let phys = self.translate(lin, false, false)?;
                    self.phys_read(phys, 1)
                } else {
                    self.phys_read(lin, 1)
                }
            };
            if byte & (1 << (bit & 7)) != 0 {
                return Err(Fault::gp(0));
            }
        }
        Ok(())
    }

    // -----------------------------------------------------------------
    // Instruction fetch
    // -----------------------------------------------------------------

    /// Fetch one byte of the instruction stream at `CS:offset`.
    fn fetch_at(&mut self, offset: u64) -> Ex<u8> {
        if self.legacy() {
            let segment = self.state.regs.cs;
            return Ok(self.phys_read(linear(segment, offset as u16), 1) as u8);
        }
        let cs = self.state.sys.seg(seg::CS);
        let lin = if self.sixty_four() {
            // `CS` has no base and no limit in 64-bit mode; `RIP` *is* the
            // linear address, and it has to be canonical.
            if !prot::canonical(offset) {
                return Err(Fault::gp(0));
            }
            offset
        } else {
            if !cs.in_bounds(offset, 1) {
                return Err(Fault::gp(0));
            }
            cs.base.wrapping_add(offset)
        };
        let user = self.cpl() == 3;
        if !self.state.sys.paging() {
            return Ok(self.phys_read(lin, 1) as u8);
        }
        // An instruction fetch is where the no-execute bit is consulted, and
        // the only place it is: a data read of the same page is fine.
        let phys = self.translate_access(lin, paging::Access::fetch(user))?;
        Ok(self.phys_read(phys, 1) as u8)
    }

    /// Top the prefetch queue up, as the bus interface unit does whenever the
    /// execution unit leaves the bus free.
    ///
    /// Only on the parts where the queue is architecturally visible. A 386
    /// prefetches too, but nothing can observe its queue contents, and filling
    /// one here would charge for fetches past a segment limit that hardware
    /// simply would not make.
    fn fill_queue(&mut self) -> Ex<()> {
        if !self.legacy() {
            return Ok(());
        }
        while self.state.queue.len() < self.state.queue.depth() {
            let offset = self
                .state
                .regs
                .rip
                .wrapping_add(u64::from(self.state.queue.len()))
                & 0xffff;
            let byte = self.fetch_at(offset)?;
            self.state.queue.push(byte);
        }
        Ok(())
    }

    /// Take the next instruction byte, refilling the queue if it is empty.
    fn fetch_byte(&mut self) -> Ex<u8> {
        let byte = if self.state.queue.len() == 0 {
            let offset = self.state.regs.rip;
            let byte = self.fetch_at(offset)?;
            if self.legacy() {
                self.state.queue.push(byte);
                self.state.queue.pop().unwrap_or(byte)
            } else {
                byte
            }
        } else {
            self.state.queue.pop().unwrap_or(self.state.open_bus)
        };
        // Guest arithmetic wraps in the guest's own width: `IP` is sixteen
        // bits and `ffff` is followed by `0000` in the same code segment,
        // while `EIP` is thirty-two and `RIP` sixty-four.
        self.state.regs.rip = if self.legacy() {
            (self.state.regs.rip & !0xffff) | u64::from(self.state.regs.rip.wrapping_add(1) as u16)
        } else if self.sixty_four() {
            self.state.regs.rip.wrapping_add(1)
        } else {
            (self.state.regs.rip & !0xffff_ffff)
                | u64::from(self.state.regs.rip.wrapping_add(1) as u32)
        };
        Ok(byte)
    }

    // -----------------------------------------------------------------
    // Flags
    // -----------------------------------------------------------------

    pub(super) fn flag(&self, mask: u32) -> bool {
        self.state.regs.eflags & mask != 0
    }

    pub(super) fn set_flag(&mut self, mask: u32, on: bool) {
        if on {
            self.state.regs.eflags |= mask;
        } else {
            self.state.regs.eflags &= !mask;
        }
    }

    /// Write the whole flags register, forcing the hard-wired bits into shape.
    pub(super) fn set_flags(&mut self, value: u32) {
        self.state.regs.eflags = Regs::normalise_flags(self.variant(), value);
    }

    /// Even parity of the low eight bits — the only parity x86 computes.
    const fn parity(value: u8) -> bool {
        (value.count_ones() & 1) == 0
    }

    /// The most significant bit of an operand of `size` bytes.
    #[inline]
    pub(super) const fn msb(size: u8) -> u64 {
        1u64 << (size as u32 * 8 - 1)
    }

    /// The mask of an operand of `size` bytes.
    #[inline]
    pub(super) const fn mask(size: u8) -> u64 {
        match size {
            1 => 0xff,
            2 => 0xffff,
            4 => 0xffff_ffff,
            _ => u64::MAX,
        }
    }

    fn set_szp(&mut self, value: u64, size: u8) {
        let value = value & Self::mask(size);
        self.set_flag(flags::ZF, value == 0);
        self.set_flag(flags::SF, value & Self::msb(size) != 0);
        self.set_flag(flags::PF, Self::parity(value as u8));
    }

    // -----------------------------------------------------------------
    // The ALU
    // -----------------------------------------------------------------

    /// Add, at any operand size.
    ///
    /// One implementation rather than three: the flag rules are identical
    /// modulo where the sign bit and the carry out sit, and three copies is
    /// three places for them to drift apart.
    pub(super) fn add(&mut self, a: u64, b: u64, carry: bool, size: u8) -> u64 {
        let mask = Self::mask(size);
        let a = a & mask;
        let b = b & mask;
        // A 64-bit add can carry out of the host word too, so the carry is
        // taken from the wide sum's own overflow rather than from a comparison
        // against the mask — which would always be false at eight bytes.
        let (partial, c1) = a.overflowing_add(b);
        let (sum, c2) = partial.overflowing_add(u64::from(carry));
        let r = sum & mask;
        let carry_out = if size == 8 { c1 || c2 } else { sum > mask };
        self.set_flag(flags::CF, carry_out);
        self.set_flag(flags::AF, (a ^ b ^ r) & 0x10 != 0);
        let msb = Self::msb(size);
        self.set_flag(flags::OF, (!(a ^ b)) & (a ^ r) & msb != 0);
        self.set_szp(r, size);
        r
    }

    /// Subtract, at any operand size.
    pub(super) fn sub(&mut self, a: u64, b: u64, borrow: bool, size: u8) -> u64 {
        let mask = Self::mask(size);
        let a = a & mask;
        let b = b & mask;
        let (rhs, overflowed) = b.overflowing_add(u64::from(borrow));
        let r = a.wrapping_sub(rhs) & mask;
        // `b == mask` with a borrow in wraps `rhs` to zero at eight bytes,
        // which is a borrow whatever `a` is.
        self.set_flag(flags::CF, overflowed || a < rhs);
        self.set_flag(flags::AF, (a ^ b ^ r) & 0x10 != 0);
        let msb = Self::msb(size);
        self.set_flag(flags::OF, (a ^ b) & (a ^ r) & msb != 0);
        self.set_szp(r, size);
        r
    }

    /// Flags after `AND`, `OR`, `XOR` and `TEST`.
    ///
    /// Carry and overflow are documented as cleared. `AF` is documented as
    /// undefined; on the 8088 it is cleared too, on every one of the tens of
    /// thousands of corpus vectors that exercise it, so it is modelled as
    /// cleared rather than left alone.
    pub(super) fn logic_flags(&mut self, r: u64, size: u8) {
        self.set_flag(flags::CF | flags::OF | flags::AF, false);
        self.set_szp(r, size);
    }

    // -----------------------------------------------------------------
    // Sequences
    // -----------------------------------------------------------------

    /// The RESET sequence.
    ///
    /// On an 8086, `CS:IP` becomes `ffff:0000` and the flags hold only their
    /// hard-wired bits. On a 386 it is `f000:fff0` with a `CS` *base* of
    /// `ffff0000` — see [`Sys::reset`] for why that distinction is the one
    /// that decides whether firmware runs at all.
    ///
    /// The general registers are not specified by Intel and are left alone,
    /// which is what hardware does — a cold
    /// [`Device::reset`](crate::core::device::Device::reset) zeroes them
    /// separately, because determinism is a first-class mode.
    fn reset_sequence(&mut self) {
        self.state.reset_pending = false;
        self.state.halted = false;
        self.state.shutdown = false;
        self.state.int_shadow = false;
        let variant = self.variant();
        if variant.is_32bit() {
            let keep = self.state.regs;
            self.state.sys = Sys::reset();
            self.state.tlb.flush();
            let regs = &mut self.state.regs;
            regs.cs = 0xf000;
            regs.rip = 0xfff0;
            regs.ds = 0;
            regs.es = 0;
            regs.ss = 0;
            regs.fs = 0;
            regs.gs = 0;
            regs.eflags = flags::ALWAYS_SET;
            regs.rdx = u64::from(variant.reset_signature());
            let _ = keep;
        } else {
            self.state.sys = Sys::reset_8086();
            let regs = &mut self.state.regs;
            regs.cs = 0xffff;
            regs.rip = 0;
            regs.ds = 0;
            regs.es = 0;
            regs.ss = 0;
            regs.eflags = flags::RESERVED_SET;
        }
        self.state.queue.flush();
        self.charge(RESET_CLOCKS);
    }

    // -- The stack -----------------------------------------------------

    /// How wide the stack pointer is: two bytes unless `SS` says otherwise.
    ///
    /// The `B` bit of the stack segment's descriptor, not the operand size —
    /// a 16-bit `push` onto a 32-bit stack still moves `ESP` and still
    /// addresses with all thirty-two bits.
    pub(super) fn stack_addr_size(&self) -> u8 {
        if self.legacy() {
            2
        } else if self.sixty_four() {
            // In 64-bit mode `SS` has no `B` bit that matters: the stack
            // pointer is `RSP`, always, and a descriptor cannot say otherwise.
            8
        } else if self.state.sys.seg(seg::SS).big() {
            4
        } else {
            2
        }
    }

    /// The current stack pointer, at the stack's address size.
    pub(super) fn sp(&self) -> u64 {
        match self.stack_addr_size() {
            2 => self.state.regs.rsp & 0xffff,
            4 => self.state.regs.rsp & 0xffff_ffff,
            _ => self.state.regs.rsp,
        }
    }

    /// Move the stack pointer, preserving the bits the stack's width does not
    /// reach.
    pub(super) fn set_sp(&mut self, value: u64) {
        let rsp = self.state.regs.rsp;
        self.state.regs.rsp = match self.stack_addr_size() {
            2 => (rsp & !0xffff) | (value & 0xffff),
            // A 32-bit stack pointer is `ESP`, and writing `ESP` zeroes the
            // top half exactly as any other 32-bit write does.
            4 => value & 0xffff_ffff,
            _ => value,
        };
    }

    /// Push `size` bytes: the stack pointer moves first, then the write
    /// happens.
    ///
    /// That order is why `PUSH SP` stores `SP - 2` on an 8086. The 80286
    /// changed it, which is handled at the `PUSH` itself rather than here.
    pub(super) fn push(&mut self, value: u64, size: u8) -> Ex<()> {
        let sp = self.sp().wrapping_sub(u64::from(size)) & Self::mask(self.stack_addr_size());
        self.set_sp(sp);
        self.write_mem(seg::SS, sp, size, value)
    }

    pub(super) fn pop(&mut self, size: u8) -> Ex<u64> {
        let sp = self.sp();
        let value = self.read_mem(seg::SS, sp, size)?;
        let next = sp.wrapping_add(u64::from(size)) & Self::mask(self.stack_addr_size());
        self.set_sp(next);
        Ok(value)
    }

    // -----------------------------------------------------------------
    // Decode
    // -----------------------------------------------------------------

    fn instruction(&mut self) -> Ex<()> {
        self.entry = self.state.regs;
        self.start_ip = self.state.regs.rip;
        self.fill_queue()?;
        let map = self.variant().map();
        let bits = self.code_bits();
        // The decoder pulls bytes through a closure so that one decoder serves
        // the interpreter and the disassembler; a fetch fault has to escape it,
        // and a closure cannot return `Err`, so it is latched here.
        let mut fetch_fault: Option<Fault> = None;
        let fields = {
            let this = &mut *self;
            isa::decode_stream_as(map, bits, &mut || {
                if fetch_fault.is_some() {
                    return None;
                }
                match this.fetch_byte() {
                    Ok(byte) => Some(byte),
                    Err(fault) => {
                        fetch_fault = Some(fault);
                        None
                    }
                }
            })
        };
        if let Some(fault) = fetch_fault {
            return Err(fault);
        }
        // An instruction longer than fifteen bytes is not an instruction: the
        // 386 raises #GP rather than executing an arbitrarily long prefix run.
        if !self.legacy() && fields.len > 15 {
            return Err(Fault::gp(0));
        }
        self.prepare_ea(&fields);
        self.charge(fields.insn.op.clocks());
        self.execute(&fields)
    }

    /// Compute the memory operand's address once, before execution.
    ///
    /// The 8086 computes it in microcode and the cost depends on how many
    /// terms are summed, which is why the charge happens here rather than at
    /// the access.
    fn prepare_ea(&mut self, f: &Fields) {
        self.ea = None;
        let insn = f.insn;
        let wants_memory = [insn.dst, insn.src, insn.aux]
            .iter()
            .any(|a| matches!(a, Arg::Eb | Arg::Ev | Arg::Ew | Arg::M | Arg::Mp | Arg::Ms));
        if let Some(m) = f.modrm
            && !m.is_register()
            && wants_memory
        {
            let offset = match f.addrsize {
                2 => {
                    let regs = &self.state.regs;
                    let terms = match m.rm {
                        0 => regs.word(3).wrapping_add(regs.word(6)), // BX+SI
                        1 => regs.word(3).wrapping_add(regs.word(7)), // BX+DI
                        2 => regs.word(5).wrapping_add(regs.word(6)), // BP+SI
                        3 => regs.word(5).wrapping_add(regs.word(7)), // BP+DI
                        4 => regs.word(6),                            // SI
                        5 => regs.word(7),                            // DI
                        6 if m.md == 0 => 0,
                        6 => regs.word(5), // BP
                        _ => regs.word(3), // BX
                    };
                    let disp = f.disp as u16;
                    let value = if m.md == 0 && m.rm == 6 {
                        disp
                    } else {
                        terms.wrapping_add(disp)
                    };
                    u64::from(value)
                }
                4 => u64::from(self.ea32(f, m)),
                _ => self.ea64(f, m),
            };
            self.ea = Some((f.mem_segment(), offset));
            if self.legacy() {
                self.charge(isa::ea_clocks(m.md, m.rm, f.seg_override.is_some()));
            }
        } else if [insn.dst, insn.src]
            .iter()
            .any(|a| matches!(a, Arg::Ob | Arg::Ov))
        {
            // The direct-offset moves carry their address in the immediate
            // field and have no ModRM byte at all.
            let offset = f.imm & Self::mask(f.addrsize);
            self.ea = Some((f.segment(seg::DS), offset));
        }
    }

    /// The 32-bit effective address: base, plus a scaled index, plus a
    /// displacement, all in 32-bit wrapping arithmetic.
    ///
    /// Thirty-two bits *even in 64-bit mode with a `67` prefix*, which is what
    /// makes this a separate function rather than a masked call into
    /// [`Exec::ea64`]: the terms are summed at the address size and the result
    /// is widened, not summed wide and masked. Widening first would let a
    /// negative displacement carry into the top half instead of wrapping.
    fn ea32(&self, f: &Fields, m: isa::ModRm) -> u32 {
        let regs = &self.state.regs;
        let mut value = 0u32;
        if m.rm == 4 {
            let sib = f.sib.unwrap_or(isa::Sib::new(0));
            // Base 5 with mode 0 is "no base": the displacement stands alone.
            if !(sib.base == 5 && m.md == 0) {
                value = value.wrapping_add(regs.dword(f.base_num()));
            }
            if f.has_index() {
                value = value.wrapping_add(regs.dword(f.index_num()) << sib.scale);
            }
        } else if !(m.rm == 5 && m.md == 0) {
            value = value.wrapping_add(regs.dword(f.rm_num()));
        }
        value.wrapping_add(f.disp as u32)
    }

    /// The 64-bit effective address.
    ///
    /// Three differences from the 32-bit form, all of them in the encodings
    /// that used to mean "no register": `mod == 00` with `r/m == 101` is now
    /// `RIP` plus the displacement rather than an absolute address; the SIB
    /// index field's "none" encoding is overridden by `REX.X`; and every term
    /// is sixty-four bits, so a negative displacement wraps at 2^64 rather
    /// than at 2^32.
    fn ea64(&self, f: &Fields, m: isa::ModRm) -> u64 {
        // `RIP` here is the address of the *next* instruction, which is what
        // the register already holds: the decoder has consumed every byte,
        // immediates included, before an effective address is computed.
        if f.rip_relative {
            return self.state.regs.rip.wrapping_add(f.disp as i64 as u64);
        }
        let regs = &self.state.regs;
        let mut value = 0u64;
        if m.rm == 4 {
            let sib = f.sib.unwrap_or(isa::Sib::new(0));
            if !(sib.base == 5 && m.md == 0) {
                value = value.wrapping_add(regs.qword(f.base_num()));
            }
            if f.has_index() {
                value = value.wrapping_add(regs.qword(f.index_num()) << sib.scale);
            }
        } else {
            value = value.wrapping_add(regs.qword(f.rm_num()));
        }
        value.wrapping_add(f.disp as i64 as u64)
    }

    pub(super) fn ea(&self) -> (u8, u64) {
        self.ea.unwrap_or((seg::DS, 0))
    }

    // -----------------------------------------------------------------
    // Operands
    // -----------------------------------------------------------------

    /// The operand width this encoding fixes, in bytes.
    fn width(f: &Fields) -> u8 {
        f.insn.width_bytes(f.opsize).unwrap_or(f.opsize)
    }

    /// Read one operand at `size` bytes.
    pub(super) fn read_arg(&mut self, f: &Fields, arg: Arg, size: u8) -> Ex<u64> {
        let regs = self.state.regs;
        let rex = f.has_rex();
        let value = match arg {
            Arg::Eb | Arg::Ev | Arg::Ew | Arg::Ed => match f.modrm {
                Some(m) if m.is_register() => regs.read(f.rm_num(), size, rex),
                _ => {
                    let (sr, off) = self.ea();
                    self.read_mem(sr, off, size)?
                }
            },
            Arg::Gb | Arg::Gv | Arg::Gw => regs.read(f.reg_num(), size, rex),
            Arg::Rd => regs.read(f.rm_num(), self.system_reg_size(), rex),
            Arg::Cd => self.read_control(f.reg_num())?,
            Arg::Dd => self.read_debug(f.reg_num())?,
            Arg::Td => self.read_test(f.reg_num())?,
            // On an 8086 only the low two bits of `reg` are decoded, which is
            // why `8C` accepts a `reg` of 4-7 and aliases down onto `ES`-`DS`.
            Arg::Sw => {
                let index = f.modrm.map_or(0, |m| m.reg);
                let index = if self.legacy() { index & 3 } else { index };
                u64::from(regs.segment(index))
            }
            Arg::Sr => u64::from(regs.segment((f.opcode >> 3) & 7)),
            Arg::Ib | Arg::Iw | Arg::Iv | Arg::Iz | Arg::Ibs => f.imm & Self::mask(size),
            Arg::Rb | Arg::Rv => regs.read(f.opcode_reg(), size, rex),
            Arg::Al => u64::from(regs.byte(0)),
            Arg::Ax => regs.read(0, size, rex),
            Arg::Cl => u64::from(regs.byte(1)),
            Arg::Dx => regs.read(2, 2, rex),
            Arg::One => 1,
            Arg::M | Arg::Mp | Arg::Ms => self.ea().1,
            Arg::Ob | Arg::Ov => {
                let (sr, off) = self.ea();
                self.read_mem(sr, off, size)?
            }
            // String operands are driven by `string_step`, which knows which
            // pointer moves; nothing else should ask for one.
            _ => 0,
        };
        Ok(value & Self::mask(size))
    }

    /// How wide a control- or debug-register move is.
    ///
    /// Always the full register: `MOV CR0, r` in 64-bit mode transfers
    /// sixty-four bits with no `REX.W` and *cannot* be narrowed by a `66`
    /// prefix. Below long mode it is thirty-two (*Intel SDM* volume 2,
    /// `MOV — Move to/from Control Registers`).
    #[inline]
    fn system_reg_size(&self) -> u8 {
        if self.sixty_four() { 8 } else { 4 }
    }

    /// Write one operand at `size` bytes.
    pub(super) fn write_arg(&mut self, f: &Fields, arg: Arg, size: u8, value: u64) -> Ex<()> {
        let rex = f.has_rex();
        match arg {
            Arg::Eb | Arg::Ev | Arg::Ew | Arg::Ed => match f.modrm {
                Some(m) if m.is_register() => self.state.regs.write(f.rm_num(), size, rex, value),
                _ => {
                    let (sr, off) = self.ea();
                    self.write_mem(sr, off, size, value)?;
                }
            },
            Arg::Gb | Arg::Gv | Arg::Gw => {
                self.state.regs.write(f.reg_num(), size, rex, value);
            }
            Arg::Rd => {
                let width = self.system_reg_size();
                self.state.regs.write(f.rm_num(), width, rex, value);
            }
            Arg::Cd => self.write_control(f.reg_num(), value)?,
            Arg::Dd => self.write_debug(f.reg_num(), value)?,
            Arg::Td => self.write_test(f.reg_num(), value)?,
            Arg::Sw => {
                let index = f.modrm.map_or(0, |m| m.reg);
                let index = if self.legacy() { index & 3 } else { index };
                self.load_segment(index, value as u16)?;
            }
            Arg::Sr => {
                let index = (f.opcode >> 3) & 7;
                self.load_segment(index, value as u16)?;
            }
            Arg::Rb | Arg::Rv => self.state.regs.write(f.opcode_reg(), size, rex, value),
            Arg::Al => self.state.regs.set_byte(0, value as u8),
            Arg::Ax => self.state.regs.write(0, size, rex, value),
            Arg::Cl => self.state.regs.set_byte(1, value as u8),
            Arg::Dx => self.state.regs.write(2, 2, rex, value),
            Arg::Ob | Arg::Ov => {
                let (sr, off) = self.ea();
                self.write_mem(sr, off, size, value)?;
            }
            _ => {}
        }
        Ok(())
    }

    // -----------------------------------------------------------------
    // Execution
    // -----------------------------------------------------------------

    #[allow(clippy::too_many_lines)]
    fn execute(&mut self, f: &Fields) -> Ex<()> {
        let insn = f.insn;
        let size = Self::width(f);
        match insn.op {
            Op::UD => return Err(Fault::bare(VEC_UD)),
            Op::ADD | Op::ADC | Op::SUB | Op::SBB | Op::CMP | Op::AND | Op::OR | Op::XOR => {
                self.arith(f, size)?;
            }
            Op::TEST => {
                let a = self.read_arg(f, insn.dst, size)?;
                let b = self.read_arg(f, insn.src, size)?;
                self.logic_flags(a & b, size);
            }
            Op::INC | Op::DEC => {
                let carry = self.flag(flags::CF);
                let a = self.read_arg(f, insn.dst, size)?;
                let r = if insn.op == Op::INC {
                    self.add(a, 1, false, size)
                } else {
                    self.sub(a, 1, false, size)
                };
                self.write_arg(f, insn.dst, size, r)?;
                self.set_flag(flags::CF, carry);
            }
            Op::NOT => {
                let a = self.read_arg(f, insn.dst, size)?;
                self.write_arg(f, insn.dst, size, !a)?;
            }
            Op::NEG => {
                let a = self.read_arg(f, insn.dst, size)?;
                let r = self.sub(0, a, false, size);
                self.write_arg(f, insn.dst, size, r)?;
            }
            Op::MOV => self.mov(f, size)?,
            Op::MOVZX | Op::MOVSX => {
                let src_size = if insn.src == Arg::Eb { 1 } else { 2 };
                let raw = self.read_arg(f, insn.src, src_size)?;
                let value = if insn.op == Op::MOVZX {
                    raw
                } else {
                    Self::sign_extend(raw, src_size)
                };
                self.write_arg(f, insn.dst, f.opsize, value)?;
            }
            // `MOVSXD` reclaimed `ARPL`'s encoding, and its source is always
            // thirty-two bits: without `REX.W` it is an expensive `mov r32,
            // r/m32`, and with one it is the only sign-extending move from a
            // doubleword long mode has.
            Op::MOVSXD => {
                let raw = self.read_arg(f, insn.src, 4)?;
                let value = if f.opsize == 8 {
                    Self::sign_extend(raw, 4)
                } else {
                    raw
                };
                self.write_arg(f, insn.dst, f.opsize, value)?;
            }
            Op::RDMSR => self.rdmsr()?,
            Op::WRMSR => self.wrmsr()?,
            Op::SYSCALL => self.syscall()?,
            Op::SYSRET => self.sysret(f.opsize)?,
            Op::SWAPGS => self.swapgs()?,
            // A `REX` prefix never reaches execution: the decoder consumes it
            // and it becomes a field. Reaching here would be a decoder bug,
            // and doing nothing is the least harmful way to say so.
            Op::REX => {}
            op if op.is_cmov() => {
                if !self.cfg.features.cmov {
                    return Err(Fault::bare(VEC_UD));
                }
                let cc = op.condition_code().unwrap_or(0);
                let value = self.read_arg(f, insn.src, size)?;
                // The destination is written **either way**: a `CMOV` whose
                // condition is false still zero-extends a 32-bit destination
                // into its 64-bit register, because the write happens and only
                // the value is selected. Writing back what was read is how
                // that stays true without a second code path.
                let value = if self.condition(cc) {
                    value
                } else {
                    self.read_arg(f, insn.dst, size)?
                };
                self.write_arg(f, insn.dst, size, value)?;
            }
            Op::XCHG => {
                let a = self.read_arg(f, insn.dst, size)?;
                let b = self.read_arg(f, insn.src, size)?;
                self.write_arg(f, insn.dst, size, b)?;
                self.write_arg(f, insn.src, size, a)?;
            }
            Op::XADD => {
                let a = self.read_arg(f, insn.dst, size)?;
                let b = self.read_arg(f, insn.src, size)?;
                let sum = self.add(a, b, false, size);
                self.write_arg(f, insn.src, size, a)?;
                self.write_arg(f, insn.dst, size, sum)?;
            }
            Op::CMPXCHG => {
                let dst = self.read_arg(f, insn.dst, size)?;
                let acc = self.state.regs.read(0, size, false);
                // The comparison sets the flags whichever way it goes; the
                // accumulator is only reloaded on a mismatch.
                self.sub(acc, dst, false, size);
                if acc & Self::mask(size) == dst {
                    let src = self.read_arg(f, insn.src, size)?;
                    self.write_arg(f, insn.dst, size, src)?;
                } else {
                    self.state.regs.write(0, size, false, dst);
                    // A failed `CMPXCHG` still writes the destination back, so
                    // a locked read-modify-write on the bus looks the same
                    // either way.
                    self.write_arg(f, insn.dst, size, dst)?;
                }
            }
            Op::BSWAP => {
                let index = f.opcode_reg();
                if f.opsize == 8 {
                    let value = self.state.regs.qword(index);
                    self.state.regs.set_qword(index, value.swap_bytes());
                } else {
                    let value = self.state.regs.dword(index);
                    self.state.regs.set_dword(index, value.swap_bytes());
                }
            }
            Op::LEA => {
                let offset = self.ea().1;
                // The address size decides how much of the address exists; the
                // operand size decides how much of it is stored.
                let offset = offset & Self::mask(f.addrsize);
                self.write_arg(f, insn.dst, f.opsize, offset)?;
            }
            Op::LES | Op::LDS | Op::LSS | Op::LFS | Op::LGS => self.load_far_pointer(f)?,
            Op::PUSH => self.push_op(f)?,
            Op::POP => {
                let value = self.pop(f.opsize)?;
                self.write_arg(f, insn.dst, f.opsize, value)?;
            }
            Op::PUSHA => self.pusha(f)?,
            Op::POPA => self.popa(f)?,
            Op::ENTER => self.enter(f)?,
            Op::LEAVE => {
                // `LEAVE` is `mov esp, ebp` then `pop ebp`, and the stack's
                // address size decides which halves move.
                let bp = match self.stack_addr_size() {
                    2 => u64::from(self.state.regs.word(5)),
                    4 => u64::from(self.state.regs.dword(5)),
                    _ => self.state.regs.rbp,
                };
                self.set_sp(bp);
                let value = self.pop(f.opsize)?;
                self.state.regs.write(5, f.opsize, false, value);
            }
            Op::PUSHF => {
                // `VM` and `RF` are never stored: the image on the stack has to
                // be one `POPF` could legally restore. (`PUSHF` is
                // IOPL-sensitive only in virtual-8086 mode, which this core
                // does not implement.)
                let value = self.state.regs.eflags & !(flags::VM | flags::RF);
                self.push(u64::from(value), f.opsize)?;
            }
            Op::POPF => self.popf(f)?,
            Op::SAHF => {
                let ah = u32::from(self.state.regs.byte(4));
                let kept = self.state.regs.eflags & !flags::LOW_BYTE;
                self.set_flags(kept | (ah & flags::LOW_BYTE));
            }
            Op::LAHF => {
                let low = (self.state.regs.eflags & 0xff) as u8;
                self.state.regs.set_byte(4, low);
            }
            Op::CBW => match f.opsize {
                2 => {
                    let al = self.state.regs.byte(0);
                    self.state.regs.set_word(0, i16::from(al as i8) as u16);
                }
                4 => {
                    // `CWDE`: the same opcode with a 32-bit operand size.
                    let ax = self.state.regs.word(0);
                    self.state.regs.set_dword(0, i32::from(ax as i16) as u32);
                }
                _ => {
                    // `CDQE`, which needs `REX.W`.
                    let eax = self.state.regs.dword(0);
                    self.state.regs.set_qword(0, i64::from(eax as i32) as u64);
                }
            },
            Op::CWD => match f.opsize {
                2 => {
                    let fill = if self.state.regs.word(0) & 0x8000 != 0 {
                        0xffff
                    } else {
                        0
                    };
                    self.state.regs.set_word(2, fill);
                }
                4 => {
                    // `CDQ`.
                    let fill = if self.state.regs.dword(0) & 0x8000_0000 != 0 {
                        0xffff_ffff
                    } else {
                        0
                    };
                    self.state.regs.set_dword(2, fill);
                }
                _ => {
                    // `CQO`.
                    let fill = if self.state.regs.rax & (1 << 63) != 0 {
                        u64::MAX
                    } else {
                        0
                    };
                    self.state.regs.set_qword(2, fill);
                }
            },
            Op::ROL | Op::ROR | Op::RCL | Op::RCR | Op::SHL | Op::SHR | Op::SAR | Op::SETMO => {
                self.shift(f, size)?;
            }
            Op::SHLD | Op::SHRD => self.double_shift(f, size)?,
            Op::MUL | Op::IMUL => self.multiply(f, size)?,
            Op::DIV | Op::IDIV => self.divide(f, size)?,
            Op::AAM => self.aam(f)?,
            Op::AAD => self.aad(f)?,
            Op::DAA => self.decimal_adjust(false),
            Op::DAS => self.decimal_adjust(true),
            Op::AAA => self.ascii_adjust(false),
            Op::AAS => self.ascii_adjust(true),
            Op::CLC => self.set_flag(flags::CF, false),
            Op::STC => self.set_flag(flags::CF, true),
            Op::CMC => {
                let cf = self.flag(flags::CF);
                self.set_flag(flags::CF, !cf);
            }
            Op::CLD => self.set_flag(flags::DF, false),
            Op::STD => self.set_flag(flags::DF, true),
            Op::CLI => {
                if self.protected() && self.cpl() > self.state.regs.iopl() {
                    return Err(Fault::gp(0));
                }
                self.set_flag(flags::IF, false);
            }
            Op::STI => {
                if self.protected() && self.cpl() > self.state.regs.iopl() {
                    return Err(Fault::gp(0));
                }
                self.set_flag(flags::IF, true);
                // `STI` shares the one-instruction shadow with `MOV SS`: the
                // instruction after it runs before any interrupt can, which is
                // what makes `sti` / `hlt` race-free.
                self.state.int_shadow = true;
            }
            Op::NOP => {
                // `90` is `XCHG eAX, eAX`, which is why it is a no-op. With
                // `REX.B` the second operand becomes `R8`, and the exchange is
                // real again — `49 90` is `xchg r8, rax`, not a longer `nop`.
                if f.rex & 1 != 0 {
                    let index = f.opcode_reg();
                    let a = self.state.regs.read(0, size, true);
                    let b = self.state.regs.read(index, size, true);
                    self.state.regs.write(0, size, true, b);
                    self.state.regs.write(index, size, true, a);
                }
            }
            Op::WAIT | Op::LOCK | Op::REP | Op::REPNE | Op::SEG => {}
            Op::HLT => {
                if self.protected() && self.cpl() != 0 {
                    return Err(Fault::gp(0));
                }
                self.state.halted = true;
            }
            Op::ESC => self.escape(f)?,
            Op::SALC => {
                let value = if self.flag(flags::CF) { 0xff } else { 0x00 };
                self.state.regs.set_byte(0, value);
            }
            Op::XLAT => {
                let sr = f.segment(seg::DS);
                let al = u64::from(self.state.regs.byte(0));
                let base = match f.addrsize {
                    2 => u64::from(self.state.regs.word(3).wrapping_add(al as u16)),
                    4 => u64::from(self.state.regs.dword(3).wrapping_add(al as u32)),
                    _ => self.state.regs.rbx.wrapping_add(al),
                };
                let value = self.read_mem(sr, base, 1)?;
                self.state.regs.set_byte(0, value as u8);
            }
            Op::IN => {
                let port = self.port(f, insn.src);
                let width = if insn.dst == Arg::Al {
                    1
                } else {
                    Self::io_width(f.opsize)
                };
                self.io_permitted(port, width)?;
                let value = self.io_read_sized(port, width);
                self.state.regs.write(0, width, false, u64::from(value));
            }
            Op::OUT => {
                let port = self.port(f, insn.dst);
                let width = if insn.src == Arg::Al {
                    1
                } else {
                    Self::io_width(f.opsize)
                };
                self.io_permitted(port, width)?;
                let value = self.state.regs.read(0, width, false) as u32;
                self.io_write_sized(port, width, value);
            }
            Op::CALL => self.call_near(f)?,
            Op::CALLF => {
                let (offset, selector) = self.far_target(f)?;
                self.far_transfer(selector, offset, true, f.opsize)?;
            }
            Op::JMP => {
                let target = match insn.dst {
                    Arg::Jv | Arg::Jb => self.relative_target(f),
                    _ => self.read_arg(f, insn.dst, f.opsize)?,
                };
                self.jump_near(target, f.opsize)?;
            }
            Op::JMPF => {
                let (offset, selector) = self.far_target(f)?;
                self.far_transfer(selector, offset, false, f.opsize)?;
            }
            Op::RET => {
                let ip = self.pop(f.opsize)?;
                let extra = if matches!(insn.dst, Arg::Iw | Arg::Iv | Arg::Iz) {
                    f.imm & 0xffff
                } else {
                    0
                };
                self.jump_near(ip, f.opsize)?;
                let sp = self.sp().wrapping_add(extra);
                self.set_sp(sp);
            }
            Op::RETF => {
                let extra = if matches!(insn.dst, Arg::Iw | Arg::Iv | Arg::Iz) {
                    f.imm & 0xffff
                } else {
                    0
                };
                self.return_far(f.opsize, extra)?;
            }
            Op::IRET => self.iret(f.opsize)?,
            Op::INT => {
                let vector = f.imm as u8;
                self.software_interrupt(vector)?;
            }
            Op::INT3 => self.software_interrupt(VEC_BREAKPOINT)?,
            Op::ICEBP => {
                // The undocumented one-byte `INT 1`. It is a *trap*, not a
                // software interrupt: the IDT descriptor privilege check that
                // `INT n` makes does not apply.
                self.take_interrupt(VEC_DEBUG, None)?;
            }
            Op::INTO => {
                if self.flag(flags::OF) {
                    self.software_interrupt(VEC_OVERFLOW)?;
                }
            }
            Op::BOUND => self.bound(f)?,
            Op::LOOP | Op::LOOPE | Op::LOOPNE => {
                let count = self.counter(f).wrapping_sub(1);
                self.set_counter(f, count);
                let zf = self.flag(flags::ZF);
                let take = count != 0
                    && match insn.op {
                        Op::LOOPE => zf,
                        Op::LOOPNE => !zf,
                        _ => true,
                    };
                if take {
                    let target = self.relative_target(f);
                    self.jump_near(target, f.opsize)?;
                }
            }
            Op::JCXZ => {
                if self.counter(f) == 0 {
                    let target = self.relative_target(f);
                    self.jump_near(target, f.opsize)?;
                }
            }
            Op::BT | Op::BTS | Op::BTR | Op::BTC => self.bit_test(f, size)?,
            Op::BSF | Op::BSR => {
                let src = self.read_arg(f, insn.src, size)?;
                self.set_flag(flags::ZF, src == 0);
                if src != 0 {
                    let bits = u32::from(size) * 8;
                    let index = if insn.op == Op::BSF {
                        src.trailing_zeros()
                    } else {
                        // The source has already been masked to the operand
                        // size, so the highest set bit is counted from the
                        // operand's own width rather than from 64.
                        bits - 1 - (src.leading_zeros() - (64 - bits))
                    };
                    self.write_arg(f, insn.dst, size, u64::from(index))?;
                }
                // A source of zero leaves the destination untouched. That is
                // the architecture, not an omission: the manual says the
                // result is undefined and the silicon writes nothing.
            }
            Op::CPUID => self.cpuid()?,
            Op::CLTS => {
                if self.protected() && self.cpl() != 0 {
                    return Err(Fault::gp(0));
                }
                self.state.sys.cr0 &= !cr0::TS;
            }
            Op::INVD | Op::WBINVD => {
                if self.protected() && self.cpl() != 0 {
                    return Err(Fault::gp(0));
                }
                // Nothing here caches anything the guest can observe, so
                // invalidating is a no-op — but the privilege check is not.
            }
            Op::INVLPG => {
                // `0F 01 F8` — group 7 extension 7 with a *register* mode
                // field — is `SWAPGS`, not an `INVLPG` of a register. The two
                // share an encoding because long mode had one slot left, and
                // the mode field is the only thing that tells them apart.
                if f.rm_is_register() {
                    if self.sixty_four() && f.modrm.is_some_and(|m| m.rm == 0) {
                        self.swapgs()?;
                        return Ok(());
                    }
                    return Err(Fault::bare(VEC_UD));
                }
                if self.protected() && self.cpl() != 0 {
                    return Err(Fault::gp(0));
                }
                let (_, offset) = self.ea();
                let base = self.state.sys.seg(f.mem_segment()).base;
                self.state.tlb.invalidate(base.wrapping_add(offset));
            }
            Op::LGDT | Op::LIDT => self.load_table_register(f)?,
            Op::SGDT | Op::SIDT => self.store_table_register(f)?,
            Op::LLDT | Op::LTR => self.load_system_selector(f)?,
            Op::SLDT | Op::STR => {
                let selector = if insn.op == Op::SLDT {
                    self.state.sys.ldtr.selector
                } else {
                    self.state.sys.task.selector
                };
                self.require_protected()?;
                self.write_arg(f, insn.dst, 2, u64::from(selector))?;
            }
            Op::SMSW => {
                // A register destination gets all thirty-two bits on a 386,
                // and memory gets sixteen — the 286 compatibility that makes
                // this instruction's width rule unlike everything else's.
                let value = u64::from(self.state.sys.cr0);
                if f.rm_is_register() {
                    self.write_arg(f, insn.dst, f.opsize, value)?;
                } else {
                    self.write_arg(f, insn.dst, 2, value & 0xffff)?;
                }
            }
            Op::LMSW => self.lmsw(f)?,
            Op::LAR | Op::LSL => self.lar_lsl(f)?,
            Op::VERR | Op::VERW => self.verify(f)?,
            Op::ARPL => self.arpl(f)?,
            op if op.is_setcc() => {
                let cc = op.condition_code().unwrap_or(0);
                let value = u64::from(self.condition(cc));
                self.write_arg(f, insn.dst, 1, value)?;
            }
            op if op.is_conditional_jump() => {
                let cc = op.condition_code().unwrap_or(0);
                if self.condition(cc) {
                    let target = self.relative_target(f);
                    self.jump_near(target, f.opsize)?;
                }
            }
            op if op.is_string() => self.string(f, size)?,
            // Every operation the table can produce is handled above; this arm
            // exists because `Op` is `#[non_exhaustive]` to the compiler.
            _ => {}
        }
        Ok(())
    }

    /// `MOV`, which needs three special cases the generic path cannot express.
    fn mov(&mut self, f: &Fields, size: u8) -> Ex<()> {
        let insn = f.insn;
        // `MOV r/m,Sreg` stores sixteen bits to memory but zero-extends to a
        // whole register, which is why the operand size and the transfer size
        // disagree here and nowhere else.
        if insn.src == Arg::Sw {
            let value = self.read_arg(f, Arg::Sw, 2)?;
            if f.rm_is_register() {
                self.write_arg(f, insn.dst, f.opsize, value)?;
            } else {
                self.write_arg(f, insn.dst, 2, value)?;
            }
            return Ok(());
        }
        // `MOV Sreg,r/m` on a 386 cannot name `CS`: the only way to change the
        // code segment is a control transfer. An 8086 allows it, and the
        // corpus exercises it. Segment numbers 6 and 7 name no register at
        // all, and a 386 rejects them rather than aliasing down as an 8086's
        // two-bit decode does.
        if !self.legacy() && (insn.dst == Arg::Sw || insn.src == Arg::Sw) {
            let index = f.modrm.map_or(0, |m| m.reg);
            if index > seg::GS || (index == seg::CS && insn.dst == Arg::Sw) {
                return Err(Fault::bare(VEC_UD));
            }
        }
        let value = self.read_arg(f, insn.src, size)?;
        self.write_arg(f, insn.dst, size, value)
    }

    /// The I/O port an `IN` or `OUT` names.
    fn port(&self, f: &Fields, arg: Arg) -> u16 {
        match arg {
            Arg::Dx => self.state.regs.word(2),
            _ => u16::from(f.imm as u8),
        }
    }

    /// The `offset:selector` pair a far transfer names.
    fn far_target(&mut self, f: &Fields) -> Ex<(u64, u16)> {
        if f.insn.dst == Arg::Ap {
            return Ok((f.imm_sized(), f.imm_seg()));
        }
        let (sr, off) = self.ea();
        let offset = self.read_mem(sr, off, f.opsize)?;
        let selector = self.read_mem(sr, off.wrapping_add(u64::from(f.opsize)), 2)?;
        Ok((offset, selector as u16))
    }

    /// The target of a relative jump: the address of the *next* instruction
    /// plus the displacement, wrapped at the operand size.
    fn relative_target(&self, f: &Fields) -> u64 {
        let next = self.state.regs.rip;
        // The displacement has already been sign-extended to the operand
        // size by the decoder, so this is one addition however wide the
        // pointer is — and it wraps in the pointer's own width.
        let target = next.wrapping_add(f.imm);
        target & Self::mask(f.opsize)
    }

    /// The count register a `LOOP`, `JCXZ` or repeat prefix uses, at the
    /// address size.
    fn counter(&self, f: &Fields) -> u64 {
        match f.addrsize {
            2 => u64::from(self.state.regs.word(1)),
            4 => u64::from(self.state.regs.dword(1)),
            _ => self.state.regs.rcx,
        }
    }

    fn set_counter(&mut self, f: &Fields, value: u64) {
        match f.addrsize {
            2 => self.state.regs.set_word(1, value as u16),
            4 => self.state.regs.set_dword(1, value as u32),
            _ => self.state.regs.set_qword(1, value),
        }
    }

    /// Evaluate one of the sixteen condition codes.
    fn condition(&self, cc: u8) -> bool {
        let cf = self.flag(flags::CF);
        let zf = self.flag(flags::ZF);
        let sf = self.flag(flags::SF);
        let of = self.flag(flags::OF);
        let pf = self.flag(flags::PF);
        match cc & 15 {
            0 => of,
            1 => !of,
            2 => cf,
            3 => !cf,
            4 => zf,
            5 => !zf,
            6 => cf || zf,
            7 => !cf && !zf,
            8 => sf,
            9 => !sf,
            10 => pf,
            11 => !pf,
            12 => sf != of,
            13 => sf == of,
            14 => zf || (sf != of),
            _ => !zf && (sf == of),
        }
    }

    fn arith(&mut self, f: &Fields, size: u8) -> Ex<()> {
        let insn = f.insn;
        let carry = self.flag(flags::CF);
        let a = self.read_arg(f, insn.dst, size)?;
        let b = self.read_arg(f, insn.src, size)?;
        let r = match insn.op {
            Op::ADD => self.add(a, b, false, size),
            Op::ADC => self.add(a, b, carry, size),
            Op::SUB | Op::CMP => self.sub(a, b, false, size),
            Op::SBB => self.sub(a, b, carry, size),
            Op::AND => {
                let r = a & b;
                self.logic_flags(r, size);
                r
            }
            Op::OR => {
                let r = a | b;
                self.logic_flags(r, size);
                r
            }
            _ => {
                let r = a ^ b;
                self.logic_flags(r, size);
                r
            }
        };
        if insn.op != Op::CMP {
            self.write_arg(f, insn.dst, size, r)?;
        }
        Ok(())
    }

    /// `PUSH`, whose operand is read *after* the pointer moves on an 8086.
    ///
    /// That order is the whole of the `PUSH SP` difference: an 8086 stores the
    /// decremented value, a 286 and later store the value before the
    /// decrement. Modelling it as an ordering rather than as a special case
    /// for one register keeps it honest — it is the microcode's order, and it
    /// would show up on any other register whose value the push itself
    /// changed.
    fn push_op(&mut self, f: &Fields) -> Ex<()> {
        let insn = f.insn;
        let size = f.opsize;
        if self.legacy() {
            let sp = self.sp().wrapping_sub(u64::from(size)) & 0xffff;
            self.set_sp(sp);
            let value = self.read_arg(f, insn.dst, size)?;
            return self.write_mem(seg::SS, sp, size, value);
        }
        let value = self.read_arg(f, insn.dst, size)?;
        self.push(value, size)
    }

    /// `PUSHA`/`PUSHAD`: the eight general registers, with the *original*
    /// stack pointer stored in the middle of them.
    fn pusha(&mut self, f: &Fields) -> Ex<()> {
        let size = f.opsize;
        let original_sp = self.sp();
        for index in 0..8u8 {
            let value = if index == 4 {
                original_sp
            } else {
                self.state.regs.read(index, size, false)
            };
            self.push(value, size)?;
        }
        Ok(())
    }

    /// `POPA`/`POPAD`: the same eight in reverse, with the saved stack pointer
    /// **discarded** rather than loaded — otherwise the pops that follow it
    /// would read from the wrong place.
    fn popa(&mut self, f: &Fields) -> Ex<()> {
        let size = f.opsize;
        for index in (0..8u8).rev() {
            let value = self.pop(size)?;
            if index != 4 {
                self.state.regs.write(index, size, false, value);
            }
        }
        Ok(())
    }

    /// `ENTER`: make a stack frame and copy the enclosing frames' pointers
    /// into it, which is what a language with nested procedures needs.
    fn enter(&mut self, f: &Fields) -> Ex<()> {
        let size = f.opsize;
        let frame = f.imm & 0xffff;
        let level = (f.imm2 & 0x1f) as u8;
        let bp = self.state.regs.read(5, size, false);
        self.push(bp, size)?;
        let frame_ptr = self.sp();
        for _ in 1..level {
            // Each nesting level copies the pointer to the frame one level out,
            // building the display the callee walks.
            let bp = self.state.regs.read(5, size, false);
            let bp = bp.wrapping_sub(u64::from(size));
            self.state.regs.write(5, size, false, bp);
            let value = self.read_mem(seg::SS, bp, size)?;
            self.push(value, size)?;
        }
        if level > 0 {
            self.push(frame_ptr, size)?;
        }
        self.state.regs.write(5, size, false, frame_ptr);
        let sp = frame_ptr.wrapping_sub(frame);
        self.set_sp(sp);
        Ok(())
    }

    /// `POPF`, which cannot raise its own privilege.
    ///
    /// At a privilege level above `IOPL` the interrupt flag is *not* changed,
    /// and above zero the `IOPL` field is not changed either. Silently
    /// ignoring the write rather than faulting is what the architecture says,
    /// and it is what makes a `pushf`/`popf` pair harmless in user code.
    fn popf(&mut self, f: &Fields) -> Ex<()> {
        let value = self.pop(f.opsize)? as u32;
        let old = self.state.regs.eflags;
        if self.legacy() {
            self.set_flags(value);
            return Ok(());
        }
        let cpl = self.cpl();
        let iopl = self.state.regs.iopl();
        let mut keep = flags::POPF_FORBIDDEN;
        if self.protected() && cpl > iopl {
            keep |= flags::IF;
        }
        if self.protected() && cpl > 0 {
            keep |= flags::IOPL;
        }
        // A 16-bit `POPF` reaches only the low half; the top stays.
        if f.opsize == 2 {
            keep |= 0xffff_0000;
        }
        self.set_flags((value & !keep) | (old & keep));
        Ok(())
    }

    /// The coprocessor escapes, `D8`-`DF`.
    ///
    /// No floating-point unit is modelled. On an 8088 the escape is a bus
    /// operation for the coprocessor's benefit and nothing else, and the
    /// corpus checks that the operand read happens. On a 386 or 486 the right
    /// answer depends on `CR0`: with `EM` set — which is how an operating
    /// system asks for software emulation — or with `TS` set after a task
    /// switch, the instruction raises `#NM` so software can take over.
    fn escape(&mut self, f: &Fields) -> Ex<()> {
        if self.legacy() {
            if matches!(f.modrm, Some(m) if !m.is_register()) {
                let (sr, off) = self.ea();
                let _ = self.read_mem(sr, off, 2)?;
            }
            return Ok(());
        }
        let cr = self.state.sys.cr0;
        if cr & (cr0::EM | cr0::TS) != 0 {
            return Err(Fault::bare(VEC_NM));
        }
        // No 387 is present, so nothing answers. Reading the operand keeps the
        // bus trace honest about what an escape does.
        if matches!(f.modrm, Some(m) if !m.is_register()) {
            let (sr, off) = self.ea();
            let _ = self.read_mem(sr, off, 2)?;
        }
        Ok(())
    }

    /// `BOUND`: check a signed index against a pair of bounds in memory.
    fn bound(&mut self, f: &Fields) -> Ex<()> {
        let size = f.opsize;
        let (sr, off) = self.ea();
        let lower = self.read_mem(sr, off, size)?;
        let upper = self.read_mem(sr, off.wrapping_add(u64::from(size)), size)?;
        let index = self.read_arg(f, f.insn.dst, size)?;
        let lower = Self::sign_extend(lower, size) as i64;
        let upper = Self::sign_extend(upper, size) as i64;
        let index = Self::sign_extend(index, size) as i64;
        if index < lower || index > upper {
            return Err(Fault::bare(VEC_BOUND));
        }
        Ok(())
    }

    /// Sign-extend the low `size` bytes of a value to sixty-four bits.
    pub(super) const fn sign_extend(value: u64, size: u8) -> u64 {
        match size {
            1 => ((value as u8) as i8) as i64 as u64,
            2 => ((value as u16) as i16) as i64 as u64,
            4 => ((value as u32) as i32) as i64 as u64,
            _ => value,
        }
    }

    // -----------------------------------------------------------------
    // Bit instructions
    // -----------------------------------------------------------------

    /// `BT`, `BTS`, `BTR` and `BTC`.
    ///
    /// With a register bit number and a memory operand the bit index is
    /// **signed and unbounded**: `bt [eax], ebx` with `EBX` of 40 addresses a
    /// byte past the operand, and with `EBX` of -1 addresses the byte before
    /// it. That is the architecture, and it is why the address is recomputed
    /// here rather than reusing the effective address as-is.
    fn bit_test(&mut self, f: &Fields, size: u8) -> Ex<()> {
        let insn = f.insn;
        let bits = u64::from(size) * 8;
        let raw = self.read_arg(f, insn.src, if insn.src == Arg::Ib { 1 } else { size })?;
        // An *immediate* bit number is always taken modulo the operand size,
        // and so is a register bit number applied to a register operand: in
        // neither case is there anywhere for a byte offset to point. Only a
        // register bit number with a memory operand gets the unbounded form.
        let bounded = f.rm_is_register() || insn.src == Arg::Ib;
        let (index, offset_bytes) = if bounded {
            (raw % bits, 0i64)
        } else {
            let signed = Self::sign_extend(raw, size) as i64;
            let word = signed.div_euclid(bits as i64);
            let bit = signed.rem_euclid(bits as i64) as u64;
            (bit, word * i64::from(size))
        };

        let value = if bounded {
            self.read_arg(f, insn.dst, size)?
        } else {
            let (sr, off) = self.ea();
            let off = off.wrapping_add(offset_bytes as u64);
            self.read_mem(sr, off, size)?
        };
        let bit = (value >> index) & 1;
        self.set_flag(flags::CF, bit != 0);
        let updated = match insn.op {
            Op::BTS => value | (1 << index),
            Op::BTR => value & !(1 << index),
            Op::BTC => value ^ (1 << index),
            _ => return Ok(()),
        };
        if bounded {
            self.write_arg(f, insn.dst, size, updated)?;
        } else {
            let (sr, off) = self.ea();
            let off = off.wrapping_add(offset_bytes as u64);
            self.write_mem(sr, off, size, updated)?;
        }
        Ok(())
    }

    // -----------------------------------------------------------------
    // Shifts and rotates
    // -----------------------------------------------------------------

    fn shift(&mut self, f: &Fields, size: u8) -> Ex<()> {
        let insn = f.insn;
        let raw = match insn.src {
            Arg::One => 1,
            Arg::Cl => u64::from(self.state.regs.byte(1)),
            _ => f.imm & 0xff,
        };
        // The 8086 uses the whole of `CL`: masking the count to five bits is
        // 80186-and-later behaviour, and the corpus masks `CL` to six bits
        // precisely to catch a core that made that mistake. A 64-bit
        // operand masks to **six** bits instead, which is the one place the
        // mask is not a constant (*Intel SDM* volume 2, `SAL/SAR/SHL/SHR`).
        let count = if self.legacy() {
            raw as u8
        } else if size == 8 {
            (raw & 0x3f) as u8
        } else {
            (raw & 0x1f) as u8
        };
        if count == 0 && !self.legacy() {
            // A 386 with a zero count does nothing at all — no flags, and no
            // write-back either, which matters for a memory-mapped operand.
            return Ok(());
        }
        // On an 8086 the write-back happens even when the count is zero and
        // nothing changed: the 8088 still drives the operand back onto the
        // bus, which a memory-mapped device would see. The flags, by contrast,
        // really are left alone.
        let a = self.read_arg(f, insn.dst, size)?;
        let r = self.shift_value(insn.op, a, count, size);
        self.write_arg(f, insn.dst, size, r)
    }

    /// One shift or rotate, at any operand size.
    ///
    /// Iterated one bit at a time rather than computed in closed form. It is
    /// slower and it is right: the overflow flag of a multi-bit rotate is the
    /// *last* iteration's, `RCL`/`RCR` rotate through a carry that changes
    /// under them, and a closed form has to special-case every one of those.
    fn shift_value(&mut self, op: Op, value: u64, count: u8, size: u8) -> u64 {
        if count == 0 {
            return value;
        }
        let mask = Self::mask(size);
        let msb = Self::msb(size);
        if op == Op::SETMO {
            // Undocumented, and the corpus says every flag is affected: the
            // result is all ones and the flags follow it as a logical result
            // would, carry and overflow cleared.
            self.logic_flags(mask, size);
            return mask;
        }
        let mut v = value & mask;
        let mut cf = self.flag(flags::CF);
        let mut of = self.flag(flags::OF);
        for _ in 0..count {
            match op {
                Op::ROL => {
                    cf = v & msb != 0;
                    v = ((v << 1) | u64::from(cf)) & mask;
                    of = (v & msb != 0) != cf;
                }
                Op::ROR => {
                    cf = v & 1 != 0;
                    v = ((v >> 1) | (u64::from(cf) * msb)) & mask;
                    of = (v & msb != 0) != (v & (msb >> 1) != 0);
                }
                Op::RCL => {
                    let carry_in = cf;
                    cf = v & msb != 0;
                    v = ((v << 1) | u64::from(carry_in)) & mask;
                    of = (v & msb != 0) != cf;
                }
                Op::RCR => {
                    let carry_in = cf;
                    of = (v & msb != 0) != carry_in;
                    cf = v & 1 != 0;
                    v = ((v >> 1) | (u64::from(carry_in) * msb)) & mask;
                }
                Op::SHL => {
                    cf = v & msb != 0;
                    v = (v << 1) & mask;
                    of = (v & msb != 0) != cf;
                }
                Op::SHR => {
                    of = v & msb != 0;
                    cf = v & 1 != 0;
                    v >>= 1;
                }
                _ => {
                    of = false;
                    cf = v & 1 != 0;
                    // `SAR` shifts the sign in, so the value is widened to
                    // the host word with its own sign before the shift.
                    let filled = v | if v & msb != 0 { !mask } else { 0 };
                    v = ((filled as i64) >> 1) as u64 & mask;
                }
            }
        }
        self.set_flag(flags::CF, cf);
        self.set_flag(flags::OF, of);
        if matches!(op, Op::SHL | Op::SHR | Op::SAR) {
            self.set_szp(v, size);
            // `AF` is documented as undefined for the shifts. On the 8088 a
            // left shift leaves bit 4 of the result there — the microcode is
            // an `ADD dst,dst`, so the auxiliary carry is the real one — and
            // the right shifts clear it.
            self.set_flag(flags::AF, op == Op::SHL && v & 0x10 != 0);
        }
        v
    }

    /// `SHLD` and `SHRD`: shift one operand, filling from another.
    ///
    /// A count of zero does nothing, including to the flags. A count greater
    /// than the operand size leaves the result undefined on hardware; this
    /// core leaves the destination alone, which is the least surprising of the
    /// available wrong answers and is documented as a choice rather than a
    /// measurement.
    fn double_shift(&mut self, f: &Fields, size: u8) -> Ex<()> {
        let insn = f.insn;
        let raw = match insn.aux {
            Arg::Cl => u64::from(self.state.regs.byte(1)),
            _ => f.imm & 0xff,
        };
        let count = if size == 8 {
            (raw & 0x3f) as u8
        } else {
            (raw & 0x1f) as u8
        };
        if count == 0 {
            return Ok(());
        }
        let bits = u32::from(size) * 8;
        if u32::from(count) >= bits {
            return Ok(());
        }
        let dst = self.read_arg(f, insn.dst, size)?;
        let src = self.read_arg(f, insn.src, size)?;
        let mask = Self::mask(size);
        let n = u32::from(count);
        let (result, carry) = if insn.op == Op::SHLD {
            let r = ((dst << n) | (src >> (bits - n))) & mask;
            (r, (dst >> (bits - n)) & 1 != 0)
        } else {
            let r = ((dst >> n) | (src << (bits - n))) & mask;
            (r, (dst >> (n - 1)) & 1 != 0)
        };
        self.set_flag(flags::CF, carry);
        // Overflow is defined only for a count of one, and is set when the
        // sign changed.
        if count == 1 {
            let msb = Self::msb(size);
            self.set_flag(flags::OF, (dst ^ result) & msb != 0);
        }
        self.set_szp(result, size);
        self.write_arg(f, insn.dst, size, result)
    }

    // -----------------------------------------------------------------
    // Multiply and divide
    // -----------------------------------------------------------------

    /// The flags a multiply leaves.
    ///
    /// Intel documents `SF`, `ZF`, `AF` and `PF` as undefined here. The
    /// hardware sets them from the **high half** of the product, which is what
    /// the microcode's last step operates on: `ZF` if that half is zero, `SF`
    /// from its top bit, `PF` from its low byte, and `AF` cleared. `CF` and
    /// `OF` are the documented ones, and the caller supplies them because
    /// `MUL` and `IMUL` disagree about what "the result does not fit" means.
    fn mul_flags(&mut self, high: u64, size: u8, overflow: bool) {
        self.set_flag(flags::ZF, high == 0);
        self.set_flag(flags::SF, high & Self::msb(size) != 0);
        self.set_flag(flags::PF, Self::parity(high as u8));
        self.set_flag(flags::AF, false);
        self.set_flag(flags::CF | flags::OF, overflow);
    }

    fn multiply(&mut self, f: &Fields, size: u8) -> Ex<()> {
        let insn = f.insn;
        // The 386's two- and three-operand `IMUL` forms write one register and
        // leave the accumulator alone. They are a different instruction that
        // happens to share a mnemonic.
        if insn.dst == Arg::Gv {
            return self.imul_short(f, size);
        }
        let signed = insn.op == Op::IMUL;
        let src = self.read_arg(f, insn.dst, size)?;
        let acc = self.state.regs.read(0, size, false);
        let mask = Self::mask(size);
        let bits = u32::from(size) * 8;
        // A 64-bit multiply produces 128 bits, so the intermediate is a
        // `u128` at every width rather than a `u64` that would silently
        // lose the high half of the widest one.
        let product: u128 = if signed {
            let a = Self::sign_extend(acc, size) as i64 as i128;
            let b = Self::sign_extend(src, size) as i64 as i128;
            (a.wrapping_mul(b)) as u128
        } else {
            u128::from(acc & mask) * u128::from(src & mask)
        };
        let low = (product as u64) & mask;
        let high = ((product >> bits) as u64) & mask;
        if size == 1 {
            // A byte multiply's whole result is `AX`, not `AH:AL` as two
            // registers.
            self.state.regs.set_word(0, (low | (high << 8)) as u16);
        } else {
            self.state.regs.write(0, size, false, low);
            self.state.regs.write(2, size, false, high);
        }
        let overflow = if signed {
            let sign_fill = if low & Self::msb(size) != 0 { mask } else { 0 };
            high != sign_fill
        } else {
            high != 0
        };
        // The high half of a byte multiply is `AH`, so the sign is read out of
        // bit 7 rather than bit 15.
        self.mul_flags(high, size, overflow);
        Ok(())
    }

    /// The 80186's two-operand and three-operand `IMUL`.
    ///
    /// Only `CF` and `OF` are defined, and they say whether the full product
    /// fits in the destination.
    fn imul_short(&mut self, f: &Fields, size: u8) -> Ex<()> {
        let insn = f.insn;
        let a = self.read_arg(f, insn.src, size)?;
        let b = if insn.aux == Arg::None {
            self.read_arg(f, insn.dst, size)?
        } else {
            let raw = self.read_arg(f, insn.aux, if insn.aux == Arg::Ibs { 1 } else { size })?;
            if insn.aux == Arg::Ibs {
                Self::sign_extend(raw, 1)
            } else {
                raw
            }
        };
        let a = Self::sign_extend(a, size) as i64 as i128;
        let b = Self::sign_extend(b, size) as i64 as i128;
        let product = a.wrapping_mul(b);
        let truncated = (product as u64) & Self::mask(size);
        let fits = i128::from(Self::sign_extend(truncated, size) as i64) == product;
        self.set_flag(flags::CF | flags::OF, !fits);
        // The other four are undefined; the 8088 sets them from the high half,
        // and this core keeps that rule for the 386 too rather than inventing
        // a second one.
        let high = ((product as u128) >> (u32::from(size) * 8)) as u64 & Self::mask(size);
        self.set_flag(flags::ZF, high == 0);
        self.set_flag(flags::SF, high & Self::msb(size) != 0);
        self.set_flag(flags::PF, Self::parity(high as u8));
        self.set_flag(flags::AF, false);
        self.write_arg(f, insn.dst, size, truncated)
    }

    fn divide(&mut self, f: &Fields, size: u8) -> Ex<()> {
        let insn = f.insn;
        let signed = insn.op == Op::IDIV;
        // A `REP` prefix in front of `IDIV` inverts the sign of the quotient
        // on an 8088. It is not a documented feature and not useful, but it is
        // deterministic, the corpus exercises it deliberately, and software
        // that stumbles on it deserves to be emulated correctly. Later parts
        // decode the prefix and ignore it.
        let negate = signed && f.rep.is_some() && self.legacy();
        let bits = u32::from(size) * 8;

        let source = self.read_arg(f, insn.dst, size)?;
        // The dividend is twice the operand width, so at eight bytes it is
        // 128 bits and the whole calculation has to be — a `u64` here would
        // silently drop `RDX`.
        let dividend: u128 = if size == 1 {
            u128::from(self.state.regs.word(0))
        } else {
            let high = u128::from(self.state.regs.read(2, size, false));
            let low = u128::from(self.state.regs.read(0, size, false));
            (high << bits) | low
        };

        // The hardware divides magnitudes and applies the signs afterwards;
        // for an unsigned divide the magnitudes are the operands.
        let (magnitude, divisor_magnitude) = if signed {
            (
                Self::sign_extend_wide(dividend, bits * 2).unsigned_abs(),
                (Self::sign_extend(source, size) as i64 as i128).unsigned_abs(),
            )
        } else {
            (dividend, u128::from(source))
        };

        // Run the loop even when the result will not fit: the flags it leaves
        // are visible either way, because a divide error pushes them.
        let (quotient_magnitude, remainder_magnitude) =
            self.cord(magnitude, divisor_magnitude, bits);

        // Whether the result is representable is decided separately: the loop
        // produces exactly `bits` bits of quotient whatever the true one would
        // have been, so it cannot report an overflow itself.
        let mask = u128::from(Self::mask(size));
        let (quotient, remainder, fault) = if divisor_magnitude == 0 {
            (0, 0, true)
        } else if signed {
            let n = Self::sign_extend_wide(dividend, bits * 2);
            let d = Self::sign_extend(source, size) as i64 as i128;
            // `IDIV` of the most negative dividend by -1 has a quotient that
            // is not representable at all. That is a divide error on hardware
            // and it is a *panic* on the host, so it has to be caught here
            // rather than divided and then range-checked.
            match (n.checked_div(d), n.checked_rem(d)) {
                (Some(mut q), Some(r)) => {
                    if negate {
                        q = q.wrapping_neg();
                    }
                    let limit = 1i128 << (bits - 1);
                    (
                        (q as u128) & mask,
                        (r as u128) & mask,
                        !(-limit..limit).contains(&q),
                    )
                }
                _ => (0, 0, true),
            }
        } else {
            let q = dividend / divisor_magnitude;
            // The loop and the host's divide have to agree whenever the result
            // fits; if they ever do not, the loop is the bug.
            debug_assert!(q > mask || q == quotient_magnitude);
            (
                quotient_magnitude & mask,
                remainder_magnitude & mask,
                q > mask,
            )
        };
        if fault {
            return Err(Fault::bare(VEC_DIVIDE));
        }

        // Measured, and exact on every corpus vector that completes: the carry
        // comes out as the complement of the quotient's top bit.
        self.set_flag(flags::CF, quotient & (1 << (bits - 1)) == 0);

        if size == 1 {
            self.state
                .regs
                .set_word(0, ((quotient & 0xff) | ((remainder & 0xff) << 8)) as u16);
        } else {
            self.state.regs.write(0, size, false, quotient as u64);
            self.state.regs.write(2, size, false, remainder as u64);
        }
        Ok(())
    }

    /// Sign-extend the low `bits` of a 128-bit value.
    const fn sign_extend_wide(value: u128, bits: u32) -> i128 {
        let shift = 128 - bits;
        ((value as i128) << shift) >> shift
    }

    /// The restoring-division loop the 8086 runs in microcode, and the flag
    /// residue it leaves.
    ///
    /// Intel describes `DIV` as a sequence of shifts and conditional
    /// subtractions, and that is what this is: the running remainder is
    /// shifted up one bit at a time, the divisor is subtracted on trial, and
    /// the quotient bit is the complement of the borrow. Running the loop
    /// rather than calling the host's divide is what brings the officially
    /// undefined flag results near the hardware's — what an 8088 leaves behind
    /// is the last trial subtraction's flags. "Near", not "equal":
    /// `conformance::KNOWN_FAILURES` records what is still missing.
    fn cord(&mut self, dividend: u128, divisor: u128, bits: u32) -> (u128, u128) {
        let mask = if bits >= 128 {
            u128::MAX
        } else {
            (1u128 << bits) - 1
        };
        let top = 1u128 << (bits - 1);
        let mut remainder = (dividend >> bits) & mask;
        let mut quotient = dividend & mask;
        for _ in 0..bits {
            let carried = (quotient >> (bits - 1)) & 1;
            quotient = (quotient << 1) & mask;
            let overflowed = remainder & top != 0;
            let shifted = ((remainder << 1) | carried) & mask;
            let difference = shifted.wrapping_sub(divisor) & mask;
            let borrow = shifted < divisor;
            self.set_flag(flags::CF, borrow);
            self.set_flag(flags::AF, (shifted ^ divisor ^ difference) & 0x10 != 0);
            self.set_flag(
                flags::OF,
                (shifted ^ divisor) & (shifted ^ difference) & top != 0,
            );
            self.set_flag(flags::SF, difference & top != 0);
            self.set_flag(flags::ZF, difference == 0);
            self.set_flag(flags::PF, Self::parity(difference as u8));
            if overflowed || !borrow {
                remainder = difference;
                quotient |= 1;
            } else {
                remainder = shifted;
            }
        }
        (quotient, remainder)
    }

    // -----------------------------------------------------------------
    // The decimal adjustments
    // -----------------------------------------------------------------

    fn aam(&mut self, f: &Fields) -> Ex<()> {
        let base = f.imm as u8;
        if base == 0 {
            // The microcode enters the divide, produces nothing, and faults.
            // What it leaves behind is the flag set of a zero result, on all
            // 47 corpus vectors that reach it.
            self.set_flag(flags::CF | flags::OF | flags::AF, false);
            self.set_szp(0, 1);
            return Err(Fault::bare(VEC_DIVIDE));
        }
        let al = self.state.regs.byte(0);
        let quotient = al / base;
        let remainder = al % base;
        self.state
            .regs
            .set_word(0, u16::from(remainder) | (u16::from(quotient) << 8));
        self.set_szp(u64::from(remainder), 1);
        self.set_flag(flags::CF | flags::OF | flags::AF, false);
        Ok(())
    }

    fn aad(&mut self, f: &Fields) -> Ex<()> {
        let base = f.imm as u8;
        let al = self.state.regs.byte(0);
        let ah = self.state.regs.byte(4);
        let product = ah.wrapping_mul(base);
        // The adjustment really is an addition, so it sets carry and the
        // auxiliary carry even though Intel calls them undefined.
        let r = self.add(u64::from(product), u64::from(al), false, 1);
        self.state.regs.set_word(0, r as u16);
        Ok(())
    }

    /// `DAA` and `DAS`, which differ only in the sign of the correction.
    ///
    /// Two things here are not what Intel's later pseudocode says, and both
    /// are measured on all 20 000 corpus vectors:
    ///
    /// - The two corrections are **one** arithmetic operation, not two. The
    ///   value added or subtracted is `0x00`, `0x06`, `0x60` or `0x66`, and
    ///   the officially undefined overflow flag is that single operation's.
    /// - The high correction's threshold **moves with the auxiliary carry**:
    ///   `AL > 0x9f` when `AF` is set, `AL > 0x99` when it is not. So `daa` on
    ///   `AL = 0x9a` with `AF` set corrects only the low digit, where the
    ///   published algorithm would correct both. Sixty-odd vectors per opcode
    ///   turn on it.
    fn decimal_adjust(&mut self, subtract: bool) {
        let al = self.state.regs.byte(0);
        let auxiliary = self.flag(flags::AF);
        let low = (al & 0x0f) > 9 || auxiliary;
        let threshold = if auxiliary { 0x9f } else { 0x99 };
        let high = self.flag(flags::CF) || al > threshold;
        let correction = u64::from(if low { 0x06u8 } else { 0x00 })
            + u64::from(if high { 0x60u8 } else { 0x00 });
        let adjusted = if subtract {
            self.sub(u64::from(al), correction, false, 1)
        } else {
            self.add(u64::from(al), correction, false, 1)
        };
        self.state.regs.set_byte(0, adjusted as u8);
        // The carry and the auxiliary carry are the *conditions*, not the
        // arithmetic's — a correction that happens not to carry still sets
        // them.
        self.set_flag(flags::CF, high);
        self.set_flag(flags::AF, low);
    }

    /// `AAA` and `AAS`, which likewise differ only in sign.
    ///
    /// Intel calls the sign, zero, parity and overflow results undefined here.
    /// They are not: they are the flags of the 8-bit `AL ± 6`, and that
    /// operation happens **whether or not the adjustment is needed** — with an
    /// operand of zero when it is not, which is why an unadjusted `AAA` leaves
    /// the sign, zero and parity of the original `AL`. Measured on all 20 000
    /// corpus vectors.
    fn ascii_adjust(&mut self, subtract: bool) {
        let al = self.state.regs.byte(0);
        let adjust = (al & 0x0f) > 9 || self.flag(flags::AF);
        let operand = u64::from(if adjust { 6u8 } else { 0 });
        let adjusted = if subtract {
            self.sub(u64::from(al), operand, false, 1)
        } else {
            self.add(u64::from(al), operand, false, 1)
        };
        let ah = self.state.regs.byte(4);
        let ah = match (adjust, subtract) {
            (true, false) => ah.wrapping_add(1),
            (true, true) => ah.wrapping_sub(1),
            (false, _) => ah,
        };
        // Only the low digit survives; the carry and auxiliary carry report
        // whether a digit was carried out of it.
        self.state
            .regs
            .set_word(0, (u16::from(ah) << 8) | u16::from(adjusted as u8 & 0x0f));
        self.set_flag(flags::CF | flags::AF, adjust);
    }

    // -----------------------------------------------------------------
    // String operations
    // -----------------------------------------------------------------

    fn string(&mut self, f: &Fields, size: u8) -> Ex<()> {
        let op = f.insn.op;
        let delta = if self.flag(flags::DF) {
            u64::from(size).wrapping_neg()
        } else {
            u64::from(size)
        };
        let Some(rep) = f.rep else {
            return self.string_step(f, size, delta);
        };
        // `REP` with a zero count does nothing at all — not even one
        // iteration.
        while self.counter(f) != 0 {
            self.string_step(f, size, delta)?;
            let count = self.counter(f).wrapping_sub(1);
            self.set_counter(f, count);
            if op.repeat_tests_zf() {
                let zf = self.flag(flags::ZF);
                let stop = match rep {
                    Rep::While => !zf,
                    Rep::WhileNot => zf,
                };
                if stop {
                    break;
                }
            }
            if count == 0 {
                break;
            }
            self.charge(op.clocks());
            // A repeat is interruptible between iterations: the processor
            // backs the instruction pointer up to the prefix and re-enters.
            // Modelled as a clean restart; the 8086 erratum where it forgets
            // all but the last prefix on resume is not.
            if self.lines.nmi_pending() || (self.flag(flags::IF) && self.lines.intr_pending()) {
                self.state.regs.rip = self.start_ip;
                self.state.queue.flush();
                return Ok(());
            }
        }
        Ok(())
    }

    /// The source index, at the address size.
    fn si(&self, f: &Fields) -> u64 {
        match f.addrsize {
            2 => u64::from(self.state.regs.word(6)),
            4 => u64::from(self.state.regs.dword(6)),
            _ => self.state.regs.rsi,
        }
    }

    /// The destination index, at the address size.
    fn di(&self, f: &Fields) -> u64 {
        match f.addrsize {
            2 => u64::from(self.state.regs.word(7)),
            4 => u64::from(self.state.regs.dword(7)),
            _ => self.state.regs.rdi,
        }
    }

    fn string_step(&mut self, f: &Fields, size: u8, delta: u64) -> Ex<()> {
        let op = f.insn.op;
        // The source of a string move is overridable; its destination is
        // always `ES:DI`, and no prefix can change that.
        let src_seg = f.segment(seg::DS);
        let si = self.si(f);
        let di = self.di(f);
        let acc = self.state.regs.read(0, size, false);
        match op {
            Op::MOVSB | Op::MOVSW => {
                let value = self.read_mem(src_seg, si, size)?;
                self.write_mem(seg::ES, di, size, value)?;
                self.advance(f, delta, true, true);
            }
            Op::CMPSB | Op::CMPSW => {
                let a = self.read_mem(src_seg, si, size)?;
                let b = self.read_mem(seg::ES, di, size)?;
                self.sub(a, b, false, size);
                self.advance(f, delta, true, true);
            }
            Op::STOSB | Op::STOSW => {
                self.write_mem(seg::ES, di, size, acc)?;
                self.advance(f, delta, false, true);
            }
            Op::LODSB | Op::LODSW => {
                let value = self.read_mem(src_seg, si, size)?;
                self.state.regs.write(0, size, false, value);
                self.advance(f, delta, true, false);
            }
            Op::SCASB | Op::SCASW => {
                let b = self.read_mem(seg::ES, di, size)?;
                self.sub(acc, b, false, size);
                self.advance(f, delta, false, true);
            }
            Op::INSB | Op::INSW => {
                let port = self.state.regs.word(2);
                let width = Self::io_width(size);
                self.io_permitted(port, width)?;
                let value = u64::from(self.io_read_sized(port, width));
                self.write_mem(seg::ES, di, size, value)?;
                self.advance(f, delta, false, true);
            }
            _ => {
                let port = self.state.regs.word(2);
                let width = Self::io_width(size);
                self.io_permitted(port, width)?;
                let value = self.read_mem(src_seg, si, size)?;
                self.io_write_sized(port, width, value as u32);
                self.advance(f, delta, true, false);
            }
        }
        Ok(())
    }

    fn advance(&mut self, f: &Fields, delta: u64, si: bool, di: bool) {
        // The pointers step in the *address* size's arithmetic: `SI` wraps
        // at 65536 in a 16-bit segment however wide the register is.
        for (want, index) in [(si, 6u8), (di, 7)] {
            if !want {
                continue;
            }
            match f.addrsize {
                2 => {
                    let v = self.state.regs.word(index).wrapping_add(delta as u16);
                    self.state.regs.set_word(index, v);
                }
                4 => {
                    let v = self.state.regs.dword(index).wrapping_add(delta as u32);
                    self.state.regs.set_dword(index, v);
                }
                _ => {
                    let v = self.state.regs.qword(index).wrapping_add(delta);
                    self.state.regs.set_qword(index, v);
                }
            }
        }
    }

    // -----------------------------------------------------------------
    // Control transfer
    // -----------------------------------------------------------------

    /// A near jump within the current code segment.
    fn jump_near(&mut self, target: u64, opsize: u8) -> Ex<()> {
        let target = target & Self::mask(opsize);
        if self.sixty_four() {
            // No limit to check; the target has to be canonical instead.
            if !prot::canonical(target) {
                return Err(Fault::gp(0));
            }
        } else if !self.legacy() {
            let cs = self.state.sys.seg(seg::CS);
            if !cs.in_bounds(target, 1) {
                return Err(Fault::gp(0));
            }
        }
        self.state.regs.rip = target;
        self.state.queue.flush();
        Ok(())
    }

    fn call_near(&mut self, f: &Fields) -> Ex<()> {
        let target = match f.insn.dst {
            Arg::Jv | Arg::Jb => self.relative_target(f),
            _ => self.read_arg(f, f.insn.dst, f.opsize)?,
        };
        let ret = self.state.regs.rip;
        // The return address is pushed before the transfer, so a jump that
        // faults on the segment limit leaves the stack already moved — which
        // is what hardware does too.
        self.push(ret, f.opsize)?;
        self.jump_near(target, f.opsize)
    }

    /// Load a far pointer into a segment register and a general register.
    fn load_far_pointer(&mut self, f: &Fields) -> Ex<()> {
        let size = f.opsize;
        let (sr, off) = self.ea();
        let offset = self.read_mem(sr, off, size)?;
        let selector = self.read_mem(sr, off.wrapping_add(u64::from(size)), 2)? as u16;
        let target = match f.insn.op {
            Op::LES => seg::ES,
            Op::LDS => seg::DS,
            Op::LSS => seg::SS,
            Op::LFS => seg::FS,
            _ => seg::GS,
        };
        // The segment is loaded *first* so that a bad selector faults before
        // the general register is touched — a partly-completed `LDS` with a
        // valid offset and a stale segment is worse than either.
        self.load_segment(target, selector)?;
        self.write_arg(f, f.insn.dst, size, offset)
    }

    /// `INT n`, which is not the same as an exception: in protected mode the
    /// gate's privilege level is checked against the caller's.
    fn software_interrupt(&mut self, vector: u8) -> Ex<()> {
        if self.protected() {
            self.check_software_gate(vector)?;
        }
        self.take_interrupt(vector, None)
    }

    // -----------------------------------------------------------------
    // CPUID
    // -----------------------------------------------------------------

    /// `CPUID`, on the parts that have it.
    ///
    /// **Every bit reported here is a bit this core implements**, and the
    /// converse matters more: reporting a feature that is not implemented is
    /// how an emulator gets a guest to execute an instruction that then raises
    /// `#UD` in the middle of a kernel, with no clue as to why. So the leaves
    /// are assembled from [`Features`](super::Features) rather than written
    /// out as a plausible-looking constant, and the two cannot drift.
    ///
    /// Conspicuously absent, and absent on purpose: **`FPU` (bit 0), `MMX`,
    /// `FXSR`, `SSE` and `SSE2`**. `CR4.OSFXSR` has storage and `CR0.EM` and
    /// `CR0.TS` behave, because an operating system reads and writes them
    /// before it decides anything; but no floating-point or SIMD arithmetic
    /// exists in this core, so nothing here invites a guest to use any. A
    /// 64-bit operating system that requires SSE2 will not boot, and it will
    /// fail at its own feature check rather than at a mystery `#UD`.
    ///
    /// *Intel SDM* volume 2, `CPUID`; the extended leaves and the `LM` bit are
    /// from the *AMD64 Architecture Programmer's Manual* volume 3, `CPUID`.
    fn cpuid(&mut self) -> Ex<()> {
        let features = self.cfg.features;
        if !features.extras_486 {
            return Err(Fault::bare(VEC_UD));
        }
        // Leaf 1's feature doubleword, one bit at a time from the lattice.
        let mut edx1: u32 = 0;
        if features.msr {
            edx1 |= 1 << 5; // MSR: RDMSR and WRMSR
            edx1 |= 1 << 4; // TSC — the counter exists as `State::cycles`
        }
        if features.pae {
            edx1 |= 1 << 6; // PAE
        }
        if features.pse {
            edx1 |= 1 << 3; // PSE: 4 MiB pages
        }
        if features.pge {
            edx1 |= 1 << 13; // PGE
        }
        if features.cmov {
            edx1 |= 1 << 15; // CMOV
        }
        if features.extras_486 {
            edx1 |= 1 << 8; // CX8 — CMPXCHG8B is not implemented; see below
        }
        // `CMPXCHG8B` is *not* implemented, so its bit must not be set. The
        // line above is deliberately undone rather than deleted, because the
        // temptation to set it is exactly what this comment is for: a 64-bit
        // Linux checks it, and lying gets a `#UD` inside the scheduler.
        edx1 &= !(1 << 8);

        let signature = self.cfg.variant.reset_signature();
        let max_basic: u32 = 1;
        let leaf = self.state.regs.rax as u32;
        let regs = &mut self.state.regs;
        let mut set = |a: u32, b: u32, c: u32, d: u32| {
            // Each half is a 32-bit write, so each zero-extends.
            regs.set_dword(0, a);
            regs.set_dword(3, b);
            regs.set_dword(1, c);
            regs.set_dword(2, d);
        };
        match leaf {
            0 => set(
                max_basic,
                // "GenuineIntel", in the order EBX:EDX:ECX that the
                // architecture specifies and that nothing else would predict.
                u32::from_le_bytes(*b"Genu"),
                u32::from_le_bytes(*b"ntel"),
                u32::from_le_bytes(*b"ineI"),
            ),
            1 => set(signature, 0, 0, edx1),
            // The extended leaves. Their existence is announced by leaf
            // `8000_0000` returning a value above itself, which is how a guest
            // that predates them avoids reading garbage.
            0x8000_0000 => set(if features.long { 0x8000_0008 } else { 0 }, 0, 0, 0),
            0x8000_0001 if features.long => {
                let mut edx: u32 = 1 << 29; // LM: long mode
                if features.nx {
                    edx |= 1 << 20; // NX
                }
                if features.syscall {
                    edx |= 1 << 11; // SYSCALL/SYSRET
                }
                set(signature, 0, 0, edx);
            }
            0x8000_0008 if features.long => {
                // Physical and linear address widths: 40 and 48. The linear
                // width is the one that matters, because it is what makes an
                // address canonical, and a guest that trusted a different
                // number would build page tables this core would reject.
                set(0x0000_3028, 0, 0, 0);
            }
            // An unimplemented leaf returns zero rather than the highest one,
            // which is what a guest probing for a feature has to be able to
            // tell apart from a real answer.
            _ => set(0, 0, 0, 0),
        }
        Ok(())
    }
}
