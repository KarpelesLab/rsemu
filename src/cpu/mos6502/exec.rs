//! The cycle-accurate interpreter.
//!
//! # One cycle is one bus access
//!
//! Every 6502 clock is a read or a write — the chip has no idle cycle and no
//! internal state it can hide — so this interpreter has no cycle counter to
//! add to. [`Exec::read`] and [`Exec::write`] *are* the clock: each one
//! charges exactly one cycle and drives it through
//! [`AddressSpace`](crate::core::space::AddressSpace). A dummy read is
//! therefore not an approximation of timing, it is the timing, and it lands on
//! the bus where hardware would see it. That matters: the NES's `$2007` port
//! advances on any read, `$4016` clocks the controller shift register, and a
//! read-modify-write's write-back of the *old* value is how several mapper
//! tricks work.
//!
//! # Two parts through one interpreter
//!
//! [`Config::variant`](super::Config::variant) picks the opcode matrix, and
//! `Exec::cmos` gates the dozen places where the W65C02S's *bus* differs from
//! the NMOS part's: the double read in a read-modify-write, the address of an
//! index fix-up, the sixth cycle of `JMP (abs)`, the decimal correction cycle,
//! and the D flag on entering an interrupt. They are gated where they happen
//! rather than collected into a second interpreter, because everything else —
//! the cycle accounting, the interrupt sampling, the stack, the branches — is
//! the same machine and duplicating it would be duplicating the bugs too.
//!
//! # Interrupt sampling
//!
//! NESdev's *CPU interrupts* page is precise about when the lines are looked
//! at: it is "the status of the interrupt lines at the end of the
//! second-to-last cycle that matters", and "interrupts are always polled
//! before the second CPU cycle (the operand fetch), but not before the third
//! CPU cycle on a taken branch".
//!
//! Expressed as "poll at the *start* of every cycle but the first", the last
//! poll an instruction performs is the one at the start of its final cycle,
//! which is the end of the second-to-last — so the rule falls out rather than
//! being special-cased, and the branch quirk is one suppressed poll
//! ([`Exec::skip_poll`]).
//!
//! Two consequences the wiki calls out come for free:
//!
//! - `CLI`, `SEI` and `PLP` change **I** after the poll has already happened,
//!   so their effect is delayed by one instruction. Here they update `P` after
//!   their last bus cycle, which is after that cycle's poll.
//! - `RTI` pulls `P` on its fourth cycle, before the final poll, so it affects
//!   interrupt inhibition immediately.
//!
//! # Sources
//!
//! NESdev wiki *CPU interrupts*, *CPU addressing modes* and *CPU unofficial
//! opcodes*; the masswerk instruction reference; Bruce Clark's "Decimal mode
//! in the 6502" (6502.org) for the decimal `ADC`/`SBC` algorithm. See
//! `docs/cpu/6502.md`.

use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

use super::isa::{Access, BitOp, Insn, Mode, Op, decode_as};
use super::{Config, Interrupt, Lines, Regs, flags};

/// Where the interrupt sequence was entered from.
///
/// The sequence is one piece of hardware used three ways; what differs is the
/// **B** bit pushed and which vector is fetched — and the vector can still be
/// stolen by an NMI (see [`Exec::sequence`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    Brk,
    Irq,
    Nmi,
}

/// Vector addresses (MOS 6500 family programming manual).
const NMI_VECTOR: u16 = 0xfffa;
const RESET_VECTOR: u16 = 0xfffc;
const IRQ_VECTOR: u16 = 0xfffe;

/// How many cycles of the jammed bus pattern a `JAM` instruction emits.
///
/// A jammed 6502 cycles forever; this is how long `SingleStepTests/65x02`
/// watches it, which makes it the one number a conformance run can agree on.
const JAM_TAIL: u32 = 9;

/// The architectural state one core owns.
///
/// Split from [`super::Mos6502`] because the interrupt *lines* live outside
/// the lock: a device asserting IRQ from inside a CPU-initiated MMIO write
/// would otherwise re-enter the CPU's own critical section and deadlock (the
/// re-entrancy contract, `ROADMAP.md` §4.7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct State {
    /// The register file.
    pub regs: Regs,
    /// Bus cycles executed since power-on.
    pub cycles: u64,
    /// Set by `JAM` on the NMOS part and by `STP` on the CMOS one: the CPU is
    /// frozen until reset.
    pub halted: bool,
    /// Set by the CMOS `WAI`: the part has stopped and is waiting for an
    /// interrupt line, which is not the same as halted — any interrupt
    /// releases it, and `/RES` is not required.
    pub waiting: bool,
    /// A reset was requested and its 7-cycle sequence has not run yet.
    pub reset_pending: bool,
    /// What the last poll latched, serviced before the next instruction.
    pub pending: Option<Interrupt>,
    /// The last value driven on the data bus.
    ///
    /// An NMOS 6502 has no bus-error input: an access nothing answers leaves
    /// the previous value on the bus, and that is what the CPU reads. Modelled
    /// rather than guessed at, because the NES depends on open-bus reads.
    pub open_bus: u8,
    /// How many accesses the address space refused.
    pub faults: u64,
    /// Address of the most recent refused access.
    pub last_fault: u16,
    /// Cycles already executed past the last budget, owed to the next one.
    ///
    /// A 6502 cannot be stopped mid-instruction, so a budget that runs out in
    /// the middle of one is overrun by up to seven cycles. The scheduler treats
    /// an overrun as fatal (`core::sched`), and rightly — so the overshoot is
    /// carried here and deducted from the following budget instead. It is
    /// architectural in the only sense that matters: a snapshot that dropped it
    /// would resume a few cycles ahead of where it was saved.
    pub debt: u64,
}

impl State {
    /// Power-on state, before the reset sequence has run.
    pub(super) const fn new() -> State {
        State {
            regs: Regs::new(),
            cycles: 0,
            halted: false,
            waiting: false,
            reset_pending: true,
            pending: None,
            open_bus: 0,
            faults: 0,
            last_fault: 0,
            debt: 0,
        }
    }
}

/// One instruction's worth of execution, borrowing everything it needs.
///
/// Created per step rather than stored: it holds the cycle bookkeeping that is
/// meaningless between instructions, and dropping it makes that explicit.
pub(super) struct Exec<'a> {
    state: &'a mut State,
    space: &'a AddressSpace,
    cfg: &'a Config,
    lines: &'a Lines,
    attrs: MemAttrs,
    /// Whether this is the CMOS part. Read on nearly every cycle, so it is
    /// hoisted out of [`Config`] once per instruction rather than matched on a
    /// variant each time.
    cmos: bool,
    /// Cycles into the current instruction. Cycle 0 is the opcode fetch, whose
    /// poll belonged to the previous instruction.
    icycle: u32,
    /// Suppress the next poll — the third cycle of a taken branch.
    skip_poll: bool,
    /// Interrupt sequences do not poll: at least one instruction of the
    /// handler always runs before another interrupt is taken.
    polling: bool,
    /// Cycles this step has charged.
    used: u64,
}

impl<'a> Exec<'a> {
    /// Borrow a core for one step.
    pub(super) fn new(
        state: &'a mut State,
        space: &'a AddressSpace,
        cfg: &'a Config,
        lines: &'a Lines,
    ) -> Exec<'a> {
        let attrs = MemAttrs::DEFAULT.with_requester(cfg.requester);
        Exec {
            state,
            space,
            cfg,
            lines,
            attrs,
            cmos: cfg.variant.is_cmos(),
            icycle: 0,
            skip_poll: false,
            polling: true,
            used: 0,
        }
    }

    /// Run one reset sequence, interrupt sequence, or instruction.
    ///
    /// Returns the number of bus cycles charged — zero only when the CPU is
    /// halted, which the caller must notice rather than spin on.
    pub(super) fn step(&mut self) -> u64 {
        if self.state.reset_pending {
            self.reset_sequence();
        } else if self.state.halted {
            return 0;
        } else if self.state.waiting && !self.wake() {
            // `WAI` stops the part with RDY low: no bus cycle is initiated, but
            // time still passes, and a scheduler that saw zero cycles would
            // treat the machine as dead instead of merely idle.
            self.stall();
        } else if let Some(kind) = self.state.pending.take() {
            self.polling = false;
            let source = match kind {
                Interrupt::Nmi => Source::Nmi,
                Interrupt::Irq => Source::Irq,
            };
            // A hardware interrupt reads the opcode it is about to discard,
            // twice, without advancing PC.
            let pc = self.state.regs.pc;
            self.read(pc);
            self.read(pc);
            self.sequence(source);
        } else {
            self.instruction();
        }
        self.used
    }

    // -----------------------------------------------------------------
    // The clock: every access is one cycle
    // -----------------------------------------------------------------

    /// Charge a cycle and poll the interrupt lines if this is not the first.
    fn begin_cycle(&mut self) {
        if self.icycle > 0 {
            if self.skip_poll {
                self.skip_poll = false;
            } else {
                self.poll();
            }
        }
        self.icycle += 1;
        self.used += 1;
        self.state.cycles = self.state.cycles.wrapping_add(1);
    }

    /// Charge a cycle that makes no bus access.
    ///
    /// The only one in this interpreter, and it exists because a `WAI`ing
    /// W65C02S really does stop driving the bus without stopping the clock.
    /// Nothing else may use it: a cycle that is not an access is a cycle a
    /// device cannot see, which is exactly what the rest of this file is built
    /// to avoid.
    fn stall(&mut self) {
        self.icycle += 1;
        self.used += 1;
        self.state.cycles = self.state.cycles.wrapping_add(1);
    }

    /// Whether an interrupt line has released a `WAI`, latching what to do next.
    ///
    /// The W65C02S wakes on IRQ **even when I masks it** (datasheet, `WAI`):
    /// what the mask decides is whether the handler runs or execution simply
    /// continues at the instruction after the `WAI`. A reset does not come
    /// through here — it is checked before this, and clears the flag itself.
    fn wake(&mut self) -> bool {
        let nmi = self.lines.nmi_pending();
        let irq = self.lines.irq_asserted();
        if !nmi && !irq {
            return false;
        }
        self.state.waiting = false;
        self.state.pending = if nmi {
            Some(Interrupt::Nmi)
        } else if !self.flag(flags::I) {
            Some(Interrupt::Irq)
        } else {
            None
        };
        true
    }

    /// Latch what the interrupt lines say right now.
    ///
    /// Overwrites rather than accumulates: an IRQ that drops between two polls
    /// is not taken, which is what a level-sensitive input means. The NMI
    /// latch is edge-set and sticky, so it survives until it is serviced.
    fn poll(&mut self) {
        if !self.polling {
            return;
        }
        self.state.pending = if self.lines.nmi_pending() {
            Some(Interrupt::Nmi)
        } else if self.lines.irq_asserted() && !self.flag(flags::I) {
            Some(Interrupt::Irq)
        } else {
            None
        };
    }

    /// One read cycle.
    fn read(&mut self, addr: u16) -> u8 {
        self.begin_cycle();
        match self.space.read(u64::from(addr), Width::U8, self.attrs) {
            Ok(v) => {
                let byte = v as u8;
                self.state.open_bus = byte;
                byte
            }
            Err(_) => {
                // No bus-error input exists on this chip, so the honest model
                // is open bus — and the fault counter is how anyone finds out.
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = addr;
                self.state.open_bus
            }
        }
    }

    /// One write cycle.
    fn write(&mut self, addr: u16, value: u8) {
        self.begin_cycle();
        self.state.open_bus = value;
        if self
            .space
            .write(u64::from(addr), Width::U8, u64::from(value), self.attrs)
            .is_err()
        {
            self.state.faults = self.state.faults.wrapping_add(1);
            self.state.last_fault = addr;
        }
    }

    /// Read the byte at PC and advance it.
    fn fetch(&mut self) -> u8 {
        let pc = self.state.regs.pc;
        let byte = self.read(pc);
        // Guest arithmetic wraps: PC is 16 bits and $ffff is followed by $0000.
        self.state.regs.pc = pc.wrapping_add(1);
        byte
    }

    /// Push a byte and decrement S, which wraps inside page one.
    fn push(&mut self, value: u8) {
        let s = self.state.regs.s;
        self.write(0x0100 | u16::from(s), value);
        self.state.regs.s = s.wrapping_sub(1);
    }

    /// Increment S, then read the byte it now points at.
    fn pull(&mut self) -> u8 {
        let s = self.state.regs.s.wrapping_add(1);
        self.state.regs.s = s;
        self.read(0x0100 | u16::from(s))
    }

    /// A read of the stack top that discards its result — the internal cycle
    /// `PLA`, `RTS`, `RTI` and `JSR` spend, which is visible on the bus.
    fn peek_stack(&mut self) {
        let addr = 0x0100 | u16::from(self.state.regs.s);
        self.read(addr);
    }

    // -----------------------------------------------------------------
    // Flags
    // -----------------------------------------------------------------

    fn flag(&self, mask: u8) -> bool {
        self.state.regs.p & mask != 0
    }

    fn set_flag(&mut self, mask: u8, on: bool) {
        if on {
            self.state.regs.p |= mask;
        } else {
            self.state.regs.p &= !mask;
        }
    }

    /// Set N and Z from a result, the way nearly every instruction does.
    fn set_nz(&mut self, value: u8) {
        self.set_flag(flags::Z, value == 0);
        self.set_flag(flags::N, value & 0x80 != 0);
    }

    // -----------------------------------------------------------------
    // Sequences
    // -----------------------------------------------------------------

    /// The seven cycles shared by RESET, IRQ, NMI and BRK, from the push of
    /// PCH onwards.
    ///
    /// The **vector is not decided until the pushes are done**: NESdev's *CPU
    /// interrupts* page documents that an NMI asserted during the first four
    /// cycles of a BRK or IRQ sequence steals the vector while the sequence
    /// otherwise runs unchanged — so a hijacked BRK still pushes **B** set and
    /// still returns through the NMI handler.
    fn sequence(&mut self, source: Source) {
        let pc = self.state.regs.pc;
        self.push((pc >> 8) as u8);
        self.push(pc as u8);

        // End of the fourth cycle: the hijack point.
        let stolen = self.lines.take_nmi_pending();
        let vector = if stolen || source == Source::Nmi {
            NMI_VECTOR
        } else {
            IRQ_VECTOR
        };

        // B is not a register bit: it only exists in the byte that reaches the
        // stack, set by a software break and clear for a hardware interrupt.
        let pushed = match source {
            Source::Brk => self.state.regs.p | flags::B | flags::U,
            Source::Irq | Source::Nmi => (self.state.regs.p | flags::U) & !flags::B,
        };
        self.push(pushed);
        self.set_flag(flags::I, true);
        if self.cmos {
            // The CMOS part clears D on entering an interrupt, so a handler no
            // longer has to open with CLD to be safe (W65C02S datasheet). The
            // *pushed* byte still carries the old D, which is why this happens
            // after the push and not before.
            self.set_flag(flags::D, false);
        }

        let lo = self.read(vector);
        let hi = self.read(vector.wrapping_add(1));
        self.state.regs.pc = u16::from(lo) | (u16::from(hi) << 8);
    }

    /// The reset sequence: an interrupt sequence whose three pushes are reads.
    ///
    /// The stack is not written — S is merely decremented three times, which
    /// is why a 6502 comes up with `S = $fd` from a zeroed stack pointer.
    fn reset_sequence(&mut self) {
        self.polling = false;
        self.state.reset_pending = false;
        self.state.halted = false;
        self.state.waiting = false;
        self.state.pending = None;
        let pc = self.state.regs.pc;
        self.read(pc);
        self.read(pc);
        for _ in 0..3 {
            self.peek_stack();
            self.state.regs.s = self.state.regs.s.wrapping_sub(1);
        }
        self.state.regs.p |= flags::U | flags::I;
        if self.cmos {
            // Reset clears D on the CMOS part too; on the NMOS one D comes up
            // undefined and software is expected to CLD.
            self.state.regs.p &= !flags::D;
        }
        let lo = self.read(RESET_VECTOR);
        let hi = self.read(RESET_VECTOR.wrapping_add(1));
        self.state.regs.pc = u16::from(lo) | (u16::from(hi) << 8);
    }

    // -----------------------------------------------------------------
    // Instructions
    // -----------------------------------------------------------------

    fn instruction(&mut self) {
        let opcode = self.fetch();
        let insn = decode_as(self.cfg.variant, opcode);
        if let Some(bit) = insn.op.bit_op() {
            // RMB/SMB are ordinary read-modify-writes and fall through to
            // `operate`; BBR/BBS are branches with a memory test in front and
            // need their own sequence.
            if matches!(bit, BitOp::BranchClear(_) | BitOp::BranchSet(_)) {
                self.bit_branch(bit);
                return;
            }
        }
        match insn.op {
            Op::BRK => {
                // The signature byte is fetched and discarded; PC advances
                // past it, which is why BRK returns to PC + 2.
                self.polling = false;
                self.fetch();
                self.sequence(Source::Brk);
            }
            Op::JSR => self.jsr(),
            Op::RTS => self.rts(),
            Op::RTI => self.rti(),
            Op::PHA | Op::PHP | Op::PHX | Op::PHY => self.push_insn(insn.op),
            Op::PLA | Op::PLP | Op::PLX | Op::PLY => self.pull_insn(insn.op),
            Op::JMP => self.jmp(insn.mode),
            Op::JAM => self.jam(),
            Op::STP => self.stp(),
            Op::WAI => self.wai(),
            op if op.is_branch() => self.branch(op),
            _ => self.operate(insn),
        }
    }

    /// The generic path: resolve the operand, then act on it.
    fn operate(&mut self, insn: Insn) {
        let loc = self.resolve(insn);
        match insn.access {
            Access::None => self.implied(insn),
            Access::Read => {
                let value = if insn.mode == Mode::Immediate {
                    loc.immediate
                } else {
                    self.read(loc.addr)
                };
                if self.cmos && matches!(insn.op, Op::ADC | Op::SBC) && self.decimal() {
                    // Decimal ADC and SBC cost one more cycle on the CMOS part,
                    // which is the price of the corrected N and Z (W65C02S
                    // datasheet, table 7-1 note). It is spent re-reading the
                    // operand; for an immediate that is the byte after the
                    // opcode, the last one the instruction fetched.
                    let at = if insn.mode == Mode::Immediate {
                        self.state.regs.pc.wrapping_sub(1)
                    } else {
                        loc.addr
                    };
                    self.read(at);
                }
                self.read_op(insn, value);
            }
            Access::Write => {
                let (addr, value) = self.write_op(insn.op, loc);
                self.write(addr, value);
            }
            Access::Modify => {
                let old = self.read(loc.addr);
                if self.cmos {
                    // Where the NMOS part writes the unmodified byte back, the
                    // CMOS one reads the address a second time. Both are on the
                    // bus and both are load-bearing: the NMOS double write is
                    // what makes `INC $2002` tricks work, and the CMOS double
                    // read is why they do not port.
                    self.read(loc.addr);
                } else {
                    self.write(loc.addr, old);
                }
                let new = self.modify_op(insn.op, old);
                self.write(loc.addr, new);
            }
        }
    }

    // -----------------------------------------------------------------
    // Addressing
    // -----------------------------------------------------------------

    fn resolve(&mut self, insn: Insn) -> Located {
        let mode = insn.mode;
        let mut out = Located::default();
        match mode {
            // One byte, one cycle: the opcode fetch was the whole instruction.
            Mode::Single => {}
            Mode::Implied | Mode::Accumulator => {
                // The dummy read of the byte after the opcode, which PC does
                // not advance over.
                let pc = self.state.regs.pc;
                self.read(pc);
            }
            Mode::Immediate => out.immediate = self.fetch(),
            Mode::ZeroPage => out.addr = u16::from(self.fetch()),
            Mode::ZeroPageX | Mode::ZeroPageY => {
                let base = self.fetch();
                // The un-indexed address is read and discarded while the adder
                // runs.
                self.read(u16::from(base));
                let index = if mode == Mode::ZeroPageX {
                    self.state.regs.x
                } else {
                    self.state.regs.y
                };
                // Page-zero indexing wraps inside page zero: $ff + 1 is $00,
                // not $0100. Computed in the guest's 8-bit width, then
                // widened.
                out.addr = u16::from(base.wrapping_add(index));
            }
            Mode::Absolute => {
                let lo = self.fetch();
                let hi = self.fetch();
                out.base_hi = hi;
                out.addr = u16::from(lo) | (u16::from(hi) << 8);
            }
            Mode::AbsoluteX | Mode::AbsoluteY => {
                let lo = self.fetch();
                let hi = self.fetch();
                let base = u16::from(lo) | (u16::from(hi) << 8);
                let index = if mode == Mode::AbsoluteX {
                    self.state.regs.x
                } else {
                    self.state.regs.y
                };
                out.base_hi = hi;
                out.addr = self.index(base, index, insn, &mut out.crossed);
            }
            Mode::ZeroPageIndirect => {
                // The CMOS addition the NMOS ALU group never had: a page-zero
                // pointer with no index at either end.
                let ptr = self.fetch();
                let lo = self.read(u16::from(ptr));
                let hi = self.read(u16::from(ptr.wrapping_add(1)));
                out.base_hi = hi;
                out.addr = u16::from(lo) | (u16::from(hi) << 8);
            }
            Mode::IndirectX => {
                let ptr = self.fetch();
                self.read(u16::from(ptr));
                let at = ptr.wrapping_add(self.state.regs.x);
                let lo = self.read(u16::from(at));
                let hi = self.read(u16::from(at.wrapping_add(1)));
                out.base_hi = hi;
                out.addr = u16::from(lo) | (u16::from(hi) << 8);
            }
            Mode::IndirectY => {
                let ptr = self.fetch();
                let lo = self.read(u16::from(ptr));
                // The pointer's high byte comes from page zero too, wrapping.
                let hi = self.read(u16::from(ptr.wrapping_add(1)));
                let base = u16::from(lo) | (u16::from(hi) << 8);
                out.base_hi = hi;
                out.addr = self.index(base, self.state.regs.y, insn, &mut out.crossed);
            }
            Mode::Relative
            | Mode::Indirect
            | Mode::AbsoluteIndirectX
            | Mode::ZeroPageRelative
            | Mode::Break => {
                debug_assert!(false, "{mode:?} is resolved by its own handler");
            }
        }
        out
    }

    /// Add an index to a base address, spending the fix-up cycle where the
    /// hardware does.
    ///
    /// The low byte is added first and the carry into the high byte costs a
    /// cycle. A *read* pays only when the carry happens; a write or a
    /// read-modify-write always spends the cycle, because the CPU cannot know
    /// in advance whether it will need it and must not write to the unfixed
    /// address. On the NMOS part the dummy access lands on that unfixed
    /// address, which is why `STA $20ff,X` touches `$2000`-page hardware; the
    /// CMOS part re-reads its own last operand byte instead, and pays the cycle
    /// in fewer cases.
    fn index(&mut self, base: u16, index: u8, insn: Insn, crossed: &mut bool) -> u16 {
        let addr = base.wrapping_add(u16::from(index));
        *crossed = (addr & 0xff00) != (base & 0xff00);
        let always = match insn.access {
            Access::Read | Access::None => false,
            Access::Write => true,
            // The CMOS part shortened the indexed shifts to six cycles when the
            // index does not carry, but left INC and DEC at seven (W65C02S
            // datasheet, table 7-1). The NMOS part always spends the cycle.
            Access::Modify => !self.cmos || matches!(insn.op, Op::INC | Op::DEC),
        };
        if *crossed || always {
            let at = if self.cmos {
                // The CMOS part re-reads the instruction's last operand byte
                // rather than driving the unfixed address, which is why
                // `STA $20ff,X` no longer pokes `$2000`-page hardware on its
                // way past. PC is already sitting after the operands.
                self.state.regs.pc.wrapping_sub(1)
            } else {
                (base & 0xff00) | (addr & 0x00ff)
            };
            self.read(at);
        }
        addr
    }

    // -----------------------------------------------------------------
    // Operations
    // -----------------------------------------------------------------

    /// Implied and accumulator-mode instructions, after their dummy read.
    fn implied(&mut self, insn: Insn) {
        let (op, mode) = (insn.op, insn.mode);
        let regs = self.state.regs;
        match op {
            Op::CLC => self.set_flag(flags::C, false),
            Op::SEC => self.set_flag(flags::C, true),
            Op::CLD => self.set_flag(flags::D, false),
            Op::SED => self.set_flag(flags::D, true),
            Op::CLV => self.set_flag(flags::V, false),
            // CLI and SEI land here *after* this cycle's poll, which is what
            // delays their effect by one instruction.
            Op::CLI => self.set_flag(flags::I, false),
            Op::SEI => self.set_flag(flags::I, true),
            Op::INX => {
                let v = regs.x.wrapping_add(1);
                self.state.regs.x = v;
                self.set_nz(v);
            }
            Op::INY => {
                let v = regs.y.wrapping_add(1);
                self.state.regs.y = v;
                self.set_nz(v);
            }
            Op::DEX => {
                let v = regs.x.wrapping_sub(1);
                self.state.regs.x = v;
                self.set_nz(v);
            }
            Op::DEY => {
                let v = regs.y.wrapping_sub(1);
                self.state.regs.y = v;
                self.set_nz(v);
            }
            Op::TAX => {
                self.state.regs.x = regs.a;
                self.set_nz(regs.a);
            }
            Op::TAY => {
                self.state.regs.y = regs.a;
                self.set_nz(regs.a);
            }
            Op::TXA => {
                self.state.regs.a = regs.x;
                self.set_nz(regs.x);
            }
            Op::TYA => {
                self.state.regs.a = regs.y;
                self.set_nz(regs.y);
            }
            Op::TSX => {
                self.state.regs.x = regs.s;
                self.set_nz(regs.s);
            }
            // The one transfer that sets no flags, because S is not a data
            // register.
            Op::TXS => self.state.regs.s = regs.x,
            // The CMOS three-byte NOPs ($5c, $dc, $fc) spend a fourth cycle
            // re-reading their last operand byte; the one-byte, one-cycle ones
            // ($x3, $xB) spend nothing at all, and `NOP` proper is the two
            // cycles `resolve` already charged.
            Op::NOP => {
                if mode == Mode::Absolute {
                    let at = self.state.regs.pc.wrapping_sub(1);
                    self.read(at);
                }
            }
            // CMOS only: the accumulator finally gets the increment and the
            // decrement the index registers always had.
            Op::INC | Op::DEC if mode == Mode::Accumulator => {
                let v = if op == Op::INC {
                    regs.a.wrapping_add(1)
                } else {
                    regs.a.wrapping_sub(1)
                };
                self.state.regs.a = v;
                self.set_nz(v);
            }
            Op::ASL | Op::LSR | Op::ROL | Op::ROR => {
                debug_assert_eq!(mode, Mode::Accumulator);
                let v = self.shift(op, regs.a);
                self.state.regs.a = v;
            }
            other => debug_assert!(false, "{other:?} is not an implied instruction"),
        }
    }

    /// Instructions that read one byte.
    fn read_op(&mut self, insn: Insn, value: u8) {
        let op = insn.op;
        let a = self.state.regs.a;
        match op {
            Op::LDA => {
                self.state.regs.a = value;
                self.set_nz(value);
            }
            Op::LDX => {
                self.state.regs.x = value;
                self.set_nz(value);
            }
            Op::LDY => {
                self.state.regs.y = value;
                self.set_nz(value);
            }
            Op::LAX => {
                self.state.regs.a = value;
                self.state.regs.x = value;
                self.set_nz(value);
            }
            Op::ORA => {
                let v = a | value;
                self.state.regs.a = v;
                self.set_nz(v);
            }
            Op::AND => {
                let v = a & value;
                self.state.regs.a = v;
                self.set_nz(v);
            }
            Op::EOR => {
                let v = a ^ value;
                self.state.regs.a = v;
                self.set_nz(v);
            }
            Op::ADC => self.adc(value),
            Op::SBC | Op::USBC => self.sbc(value),
            Op::CMP => self.compare(a, value),
            Op::CPX => {
                let x = self.state.regs.x;
                self.compare(x, value);
            }
            Op::CPY => {
                let y = self.state.regs.y;
                self.compare(y, value);
            }
            Op::BIT => {
                // Z from the AND, but N and V straight out of the operand's
                // top two bits — the only instruction that does this.
                self.set_flag(flags::Z, a & value == 0);
                // Except in the CMOS immediate form, which touches Z alone:
                // there is no memory byte whose top two bits could reach N and
                // V, and BIT # exists to test a mask without disturbing them
                // (W65C02S datasheet).
                if insn.mode != Mode::Immediate {
                    self.set_flag(flags::N, value & 0x80 != 0);
                    self.set_flag(flags::V, value & 0x40 != 0);
                }
            }
            Op::NOP => {}
            Op::ANC => {
                let v = a & value;
                self.state.regs.a = v;
                self.set_nz(v);
                // Carry ends up wherever the sign did: the ASL that never ran.
                self.set_flag(flags::C, v & 0x80 != 0);
            }
            Op::ALR => {
                let t = a & value;
                self.set_flag(flags::C, t & 0x01 != 0);
                let v = t >> 1;
                self.state.regs.a = v;
                self.set_nz(v);
            }
            Op::ARR => self.arr(value),
            Op::ANE => {
                // Unstable: the accumulator is first OR'd with a constant that
                // depends on the chip and its temperature. `Config::magic`
                // names the one this build uses.
                let v = (a | self.cfg.magic) & self.state.regs.x & value;
                self.state.regs.a = v;
                self.set_nz(v);
            }
            Op::LXA => {
                let v = (a | self.cfg.magic) & value;
                self.state.regs.a = v;
                self.state.regs.x = v;
                self.set_nz(v);
            }
            Op::SBX => {
                // A CMP and a DEX at once: the subtract ignores carry in but
                // sets it, and the result goes to X.
                let t = a & self.state.regs.x;
                self.set_flag(flags::C, t >= value);
                let v = t.wrapping_sub(value);
                self.state.regs.x = v;
                self.set_nz(v);
            }
            Op::LAS => {
                let v = value & self.state.regs.s;
                self.state.regs.a = v;
                self.state.regs.x = v;
                self.state.regs.s = v;
                self.set_nz(v);
            }
            other => debug_assert!(false, "{other:?} is not a read instruction"),
        }
    }

    /// Instructions that write one byte, returning where and what.
    ///
    /// Returns the address as well as the value because the unstable stores
    /// can *change* it: when the index carries into the high byte, the value
    /// being stored is what ends up driving the high address lines.
    fn write_op(&mut self, op: Op, loc: Located) -> (u16, u8) {
        let regs = self.state.regs;
        match op {
            Op::STA => (loc.addr, regs.a),
            Op::STX => (loc.addr, regs.x),
            Op::STY => (loc.addr, regs.y),
            Op::STZ => (loc.addr, 0),
            Op::SAX => (loc.addr, regs.a & regs.x),
            Op::SHA => self.unstable_store(regs.a & regs.x, loc),
            Op::SHX => self.unstable_store(regs.x, loc),
            Op::SHY => self.unstable_store(regs.y, loc),
            Op::TAS => {
                // S is loaded whatever else happens, which is the only reason
                // anyone ever used this opcode.
                self.state.regs.s = regs.a & regs.x;
                self.unstable_store(regs.a & regs.x, loc)
            }
            other => {
                debug_assert!(false, "{other:?} is not a write instruction");
                (loc.addr, regs.a)
            }
        }
    }

    /// `SHA`/`SHX`/`SHY`/`TAS`: store `reg AND (high byte of base + 1)`.
    ///
    /// Unstable in a specific, reproducible way. The value is computed from
    /// the *un-indexed* high byte, and if the index carried into that high
    /// byte the store lands at `(value << 8) | low` instead — the value is on
    /// the bus while the high address byte is being driven, so it wins. This
    /// is the behaviour `SingleStepTests/65x02` expects
    /// (`docs/cpu/6502.md`).
    fn unstable_store(&mut self, reg: u8, loc: Located) -> (u16, u8) {
        let value = reg & loc.base_hi.wrapping_add(1);
        let addr = if loc.crossed {
            (u16::from(value) << 8) | (loc.addr & 0x00ff)
        } else {
            loc.addr
        };
        (addr, value)
    }

    /// Read-modify-write instructions, given the byte just read.
    fn modify_op(&mut self, op: Op, value: u8) -> u8 {
        if let Some(bit) = op.bit_op() {
            // `RMB<n>` and `SMB<n>` set or clear one bit and touch no flag at
            // all — the only read-modify-writes in the family that do not.
            return match bit {
                BitOp::Reset(_) => value & !bit.mask(),
                BitOp::Set(_) => value | bit.mask(),
                other => {
                    debug_assert!(false, "{other:?} branches, it does not modify");
                    value
                }
            };
        }
        match op {
            Op::ASL | Op::LSR | Op::ROL | Op::ROR => self.shift(op, value),
            Op::INC => {
                let v = value.wrapping_add(1);
                self.set_nz(v);
                v
            }
            Op::DEC => {
                let v = value.wrapping_sub(1);
                self.set_nz(v);
                v
            }
            // `TSB` and `TRB` set or clear the bits the accumulator selects and
            // report, in Z alone, whether any of them were already set. Not a
            // `BIT`: N and V are untouched, because the test is of the mask
            // rather than of the byte (W65C02S datasheet).
            Op::TSB | Op::TRB => {
                let a = self.state.regs.a;
                self.set_flag(flags::Z, a & value == 0);
                if op == Op::TSB { value | a } else { value & !a }
            }
            // The combined ones: shift the memory operand, then fold it into
            // the accumulator. Both halves set their own flags, the second
            // winning where they overlap.
            Op::SLO => {
                let v = self.shift(Op::ASL, value);
                let a = self.state.regs.a | v;
                self.state.regs.a = a;
                self.set_nz(a);
                v
            }
            Op::SRE => {
                let v = self.shift(Op::LSR, value);
                let a = self.state.regs.a ^ v;
                self.state.regs.a = a;
                self.set_nz(a);
                v
            }
            Op::RLA => {
                let v = self.shift(Op::ROL, value);
                let a = self.state.regs.a & v;
                self.state.regs.a = a;
                self.set_nz(a);
                v
            }
            Op::RRA => {
                let v = self.shift(Op::ROR, value);
                self.adc(v);
                v
            }
            Op::ISC => {
                let v = value.wrapping_add(1);
                self.sbc(v);
                v
            }
            Op::DCP => {
                let v = value.wrapping_sub(1);
                let a = self.state.regs.a;
                self.compare(a, v);
                v
            }
            other => {
                debug_assert!(false, "{other:?} is not a read-modify-write instruction");
                value
            }
        }
    }

    /// The four shifts, shared by the accumulator, memory and the combined
    /// undocumented forms.
    fn shift(&mut self, op: Op, value: u8) -> u8 {
        let carry_in = u8::from(self.flag(flags::C));
        let (result, carry_out) = match op {
            Op::ASL => (value << 1, value & 0x80 != 0),
            Op::LSR => (value >> 1, value & 0x01 != 0),
            Op::ROL => ((value << 1) | carry_in, value & 0x80 != 0),
            Op::ROR => ((value >> 1) | (carry_in << 7), value & 0x01 != 0),
            other => {
                debug_assert!(false, "{other:?} is not a shift");
                (value, false)
            }
        };
        self.set_flag(flags::C, carry_out);
        self.set_nz(result);
        result
    }

    // -----------------------------------------------------------------
    // Arithmetic
    // -----------------------------------------------------------------

    /// Add with carry, binary or packed BCD.
    ///
    /// The decimal path follows Bruce Clark's "Decimal mode in the 6502"
    /// (6502.org), sequences 1 and 2: the accumulator and carry come from the
    /// corrected sum, N and V from the *intermediate* before the high-nibble
    /// fix-up, and Z — uniquely — from the plain binary sum, because the zero
    /// flag is computed by hardware that never sees the decimal correction.
    fn adc(&mut self, m: u8) {
        let a = self.state.regs.a;
        let c = u16::from(self.flag(flags::C));
        let binary = u16::from(a) + u16::from(m) + c;

        if self.decimal() {
            let mut low = u16::from(a & 0x0f) + u16::from(m & 0x0f) + c;
            if low >= 0x0a {
                low = ((low + 0x06) & 0x0f) + 0x10;
            }
            let mut sum = u16::from(a & 0xf0) + u16::from(m & 0xf0) + low;
            let intermediate = sum as u8;
            self.set_flag(flags::N, intermediate & 0x80 != 0);
            self.set_flag(flags::V, (!(a ^ m) & (a ^ intermediate) & 0x80) != 0);
            self.set_flag(flags::Z, (binary as u8) == 0);
            if sum >= 0xa0 {
                sum += 0x60;
            }
            self.set_flag(flags::C, sum >= 0x100);
            let result = sum as u8;
            self.state.regs.a = result;
            if self.cmos {
                // The CMOS adder latches its flags after the decimal
                // correction, so N and Z describe the answer instead of an
                // intermediate the NMOS part happened to expose. V still comes
                // out of the binary adder, which is what it always measured.
                self.set_nz(result);
            }
        } else {
            let result = binary as u8;
            self.set_flag(flags::C, binary > 0xff);
            // Overflow is a sign question: both inputs agreed and the answer
            // did not.
            self.set_flag(flags::V, (!(a ^ m) & (a ^ result) & 0x80) != 0);
            self.state.regs.a = result;
            self.set_nz(result);
        }
    }

    /// Subtract with borrow.
    ///
    /// Every flag is the binary one even in decimal mode — an NMOS asymmetry
    /// with `ADC`, and one Clark's sequence 3 is explicit about. Only the
    /// accumulator is corrected.
    fn sbc(&mut self, m: u8) {
        let a = self.state.regs.a;
        let borrow = i32::from(!self.flag(flags::C));
        let binary = i32::from(a) - i32::from(m) - borrow;
        let result = binary as u8;

        self.set_flag(flags::C, binary >= 0);
        // Inputs of opposite sign, answer with the wrong one.
        self.set_flag(flags::V, ((a ^ m) & (a ^ result) & 0x80) != 0);
        self.set_nz(result);

        if !self.decimal() {
            self.state.regs.a = result;
            return;
        }

        let corrected = if self.cmos {
            // The CMOS subtractor takes the binary difference and applies both
            // corrections to *it*, rather than correcting nibble by nibble on
            // the way. The two agree on every valid BCD pair and disagree on
            // the rest — `$10 - $fc` is `$ae` here and `$be` on an NMOS part
            // (Clark, "Decimal mode in the 6502", the 65C02 sequences).
            let low = i32::from(a & 0x0f) - i32::from(m & 0x0f) - borrow;
            let mut out = binary;
            if out < 0 {
                out -= 0x60;
            }
            if low < 0 {
                out -= 0x06;
            }
            out as u8
        } else {
            let mut low = i32::from(a & 0x0f) - i32::from(m & 0x0f) - borrow;
            if low < 0 {
                low = ((low - 0x06) & 0x0f) - 0x10;
            }
            let mut sum = i32::from(a & 0xf0) - i32::from(m & 0xf0) + low;
            if sum < 0 {
                sum -= 0x60;
            }
            sum as u8
        };
        self.state.regs.a = corrected;
        if self.cmos {
            // As with ADC: the corrected value is what reaches the flag logic.
            self.set_nz(corrected);
        }
    }

    /// Compare a register against memory: a subtract that keeps only flags.
    fn compare(&mut self, reg: u8, m: u8) {
        self.set_flag(flags::C, reg >= m);
        let result = reg.wrapping_sub(m);
        self.set_nz(result);
    }

    /// `ARR`: AND with the operand, then a rotate that runs through the adder.
    ///
    /// The carry and overflow come out of the adder rather than out of the
    /// shifter, which is why C is bit 6 of the result and V is bit 6 XOR
    /// bit 5. In decimal mode it additionally applies the BCD fix-ups, per the
    /// undocumented-opcode literature (`docs/cpu/6502.md`).
    fn arr(&mut self, m: u8) {
        let t = self.state.regs.a & m;
        let carry_in = self.flag(flags::C);
        let rotated = (t >> 1) | (u8::from(carry_in) << 7);

        if self.decimal() {
            self.set_flag(flags::N, carry_in);
            self.set_flag(flags::Z, rotated == 0);
            self.set_flag(flags::V, (t ^ rotated) & 0x40 != 0);
            let mut out = rotated;
            if u16::from(t & 0x0f) + u16::from(t & 0x01) > 0x05 {
                out = (out & 0xf0) | (out.wrapping_add(0x06) & 0x0f);
            }
            if u16::from(t & 0xf0) + u16::from(t & 0x10) > 0x50 {
                out = out.wrapping_add(0x60);
                self.set_flag(flags::C, true);
            } else {
                self.set_flag(flags::C, false);
            }
            self.state.regs.a = out;
        } else {
            self.state.regs.a = rotated;
            self.set_nz(rotated);
            self.set_flag(flags::C, t & 0x80 != 0);
            self.set_flag(flags::V, (t ^ (t << 1)) & 0x80 != 0);
        }
    }

    /// Whether this instruction should do decimal arithmetic.
    ///
    /// Two conditions, and they are different in kind: the guest's D flag, and
    /// whether the part *has* decimal mode at all. The RP2A03 in the NES does
    /// not, which is a property of the chip and so a construction property —
    /// never a `#[cfg]` (`docs/cpu/6502.md`).
    fn decimal(&self) -> bool {
        self.cfg.decimal && self.flag(flags::D)
    }

    // -----------------------------------------------------------------
    // Control flow
    // -----------------------------------------------------------------

    fn branch(&mut self, op: Op) {
        let offset = self.fetch();
        let taken = match op {
            Op::BPL => !self.flag(flags::N),
            Op::BMI => self.flag(flags::N),
            Op::BVC => !self.flag(flags::V),
            Op::BVS => self.flag(flags::V),
            Op::BCC => !self.flag(flags::C),
            Op::BCS => self.flag(flags::C),
            Op::BNE => !self.flag(flags::Z),
            Op::BEQ => self.flag(flags::Z),
            // CMOS only, and the reason it exists: a relative JMP that costs
            // three cycles and two bytes instead of three and three.
            Op::BRA => true,
            other => {
                debug_assert!(false, "{other:?} is not a branch");
                false
            }
        };
        if !taken {
            return;
        }
        // "Interrupts are ... not [polled] before the third CPU cycle on a
        // taken branch" — NESdev, CPU interrupts. A pending IRQ therefore
        // waits out the instruction after the branch as well.
        self.skip_poll = true;

        let pc = self.state.regs.pc;
        // The third cycle fetches the next opcode and throws it away while the
        // low byte of PC is fixed up.
        self.read(pc);
        // The displacement is signed; sign-extending through i8 and adding in
        // 16 bits is the guest's own arithmetic, and it wraps.
        let target = pc.wrapping_add(offset as i8 as u16);
        if (target & 0xff00) != (pc & 0xff00) {
            // The fourth cycle only exists when the high byte needs fixing,
            // and it reads from the half-fixed address.
            self.read((pc & 0xff00) | (target & 0x00ff));
        }
        self.state.regs.pc = target;
    }

    /// `BBR<n>` and `BBS<n>`: test one bit of a page-zero byte, then branch.
    ///
    /// Five cycles, six when taken, seven when the branch also crosses a page.
    /// The page-zero byte is read *twice* — these share the read-modify-write
    /// datapath even though they write nothing back — and the page-cross fix-up
    /// repeats the read at PC rather than reaching for the half-fixed address
    /// an ordinary branch drives. Both are on the bus (W65C02S datasheet).
    fn bit_branch(&mut self, bit: BitOp) {
        let zp = u16::from(self.fetch());
        let value = self.read(zp);
        self.read(zp);
        let offset = self.fetch();
        let taken = match bit {
            BitOp::BranchClear(_) => value & bit.mask() == 0,
            BitOp::BranchSet(_) => value & bit.mask() != 0,
            other => {
                debug_assert!(false, "{other:?} modifies, it does not branch");
                false
            }
        };
        if !taken {
            return;
        }
        self.skip_poll = true;
        let pc = self.state.regs.pc;
        self.read(pc);
        let target = pc.wrapping_add(offset as i8 as u16);
        if (target & 0xff00) != (pc & 0xff00) {
            self.read(pc);
        }
        self.state.regs.pc = target;
    }

    /// `STP`: stop the oscillator until `/RES`.
    ///
    /// Three cycles, then nothing at all — not even the jammed bus pattern a
    /// `JAM` leaves behind, because a stopped clock has no cycles to spend. The
    /// caller has to notice [`State::halted`]; only a reset clears it.
    fn stp(&mut self) {
        let pc = self.state.regs.pc;
        self.read(pc);
        self.read(pc);
        self.state.halted = true;
    }

    /// `WAI`: stop until an interrupt line moves.
    ///
    /// Three cycles, then the part stalls with RDY low. Unlike `STP` this is
    /// not a halt: any interrupt releases it, and whether the handler then runs
    /// is the I flag's business rather than the instruction's — see
    /// [`Exec::wake`]. The point of it is to answer an interrupt in a bounded
    /// number of cycles instead of whenever the current instruction happens to
    /// finish (W65C02S datasheet).
    fn wai(&mut self) {
        let pc = self.state.regs.pc;
        self.read(pc);
        self.read(pc);
        self.state.waiting = true;
    }

    fn jmp(&mut self, mode: Mode) {
        let lo = self.fetch();
        let hi = self.fetch();
        let ptr = u16::from(lo) | (u16::from(hi) << 8);
        self.state.regs.pc = match mode {
            Mode::Absolute => ptr,
            Mode::Indirect => {
                let target_lo = self.read(ptr);
                // The famous NMOS bug: the pointer's high byte is never
                // incremented, so `JMP ($10ff)` takes its high byte from
                // $1000. Faithfully reproduced — software depends on it.
                let wrapped = (ptr & 0xff00) | u16::from((ptr as u8).wrapping_add(1));
                let target_hi = self.read(wrapped);
                if self.cmos {
                    // The CMOS part makes the same wrong access and then spends
                    // a sixth cycle reading the right one, which is how the bug
                    // got fixed without the addressing hardware learning to
                    // carry: the second read wins.
                    let fixed = self.read(ptr.wrapping_add(1));
                    u16::from(target_lo) | (u16::from(fixed) << 8)
                } else {
                    u16::from(target_lo) | (u16::from(target_hi) << 8)
                }
            }
            Mode::AbsoluteIndirectX => {
                // The fix-up cycle re-reads the *first* operand byte, not the
                // last one every other indexed mode goes back to. PC is sitting
                // after both operands.
                let at = self.state.regs.pc.wrapping_sub(2);
                self.read(at);
                let at = ptr.wrapping_add(u16::from(self.state.regs.x));
                let target_lo = self.read(at);
                let target_hi = self.read(at.wrapping_add(1));
                u16::from(target_lo) | (u16::from(target_hi) << 8)
            }
            other => {
                debug_assert!(false, "JMP cannot use {other:?}");
                ptr
            }
        };
    }

    fn jsr(&mut self) {
        let lo = self.fetch();
        // An internal cycle that shows up on the bus as a stack read.
        self.peek_stack();
        // What is pushed is the address of the *last* byte of the JSR, which
        // is why RTS increments the pulled address.
        let ret = self.state.regs.pc;
        self.push((ret >> 8) as u8);
        self.push(ret as u8);
        let hi = self.read(ret);
        self.state.regs.pc = u16::from(lo) | (u16::from(hi) << 8);
    }

    fn rts(&mut self) {
        let pc = self.state.regs.pc;
        self.read(pc);
        self.peek_stack();
        let lo = self.pull();
        let hi = self.pull();
        let target = u16::from(lo) | (u16::from(hi) << 8);
        // The final cycle reads the byte at the pulled address and discards
        // it, then PC is incremented past it.
        self.read(target);
        self.state.regs.pc = target.wrapping_add(1);
    }

    fn rti(&mut self) {
        let pc = self.state.regs.pc;
        self.read(pc);
        self.peek_stack();
        let p = self.pull();
        // B has no register bit to return to; bit 5 always reads as one.
        self.state.regs.p = (p | flags::U) & !flags::B;
        let lo = self.pull();
        let hi = self.pull();
        self.state.regs.pc = u16::from(lo) | (u16::from(hi) << 8);
    }

    fn push_insn(&mut self, op: Op) {
        let pc = self.state.regs.pc;
        self.read(pc);
        let value = match op {
            Op::PHA => self.state.regs.a,
            Op::PHX => self.state.regs.x,
            Op::PHY => self.state.regs.y,
            _ => self.state.regs.p | flags::B | flags::U,
        };
        self.push(value);
    }

    fn pull_insn(&mut self, op: Op) {
        let pc = self.state.regs.pc;
        self.read(pc);
        self.peek_stack();
        let value = self.pull();
        match op {
            Op::PLA => {
                self.state.regs.a = value;
                self.set_nz(value);
            }
            Op::PLX => {
                self.state.regs.x = value;
                self.set_nz(value);
            }
            Op::PLY => {
                self.state.regs.y = value;
                self.set_nz(value);
            }
            // Like CLI and SEI, this lands after the final cycle's poll, so a
            // pulled I flag takes effect one instruction late.
            _ => self.state.regs.p = (value | flags::U) & !flags::B,
        }
    }

    /// `JAM`: the instruction that never finishes.
    ///
    /// The timing generator stops advancing, so the chip keeps cycling with
    /// the address bus stuck near the top of memory. `SingleStepTests/65x02`
    /// records that pattern — `$ffff`, `$fffe`, `$fffe`, then `$ffff` for as
    /// long as it watches — and it is the only observable thing a jammed 6502
    /// does, so it is reproduced here.
    ///
    /// Where the corpus stops watching is arbitrary and a real part never
    /// does; [`JAM_TAIL`] is that window, and after it
    /// [`step`](Exec::step) charges nothing and the caller has to notice
    /// [`State::halted`] rather than spin.
    fn jam(&mut self) {
        let pc = self.state.regs.pc;
        self.read(pc);
        for cycle in 0..JAM_TAIL {
            // $ffff, then $fffe twice, then $ffff from there on.
            let addr = if (1..=2).contains(&cycle) {
                0xfffe
            } else {
                0xffff
            };
            self.read(addr);
        }
        self.state.halted = true;
    }
}

/// Where an instruction's operand is, and what the address computation saw on
/// the way there.
///
/// `base_hi` and `crossed` exist for the unstable stores, which are the only
/// instructions whose *result* depends on the addressing hardware rather than
/// just on the address.
#[derive(Debug, Clone, Copy, Default)]
struct Located {
    /// The effective address.
    addr: u16,
    /// The operand byte, for immediate mode.
    immediate: u8,
    /// High byte of the address before indexing.
    base_hi: u8,
    /// Whether indexing carried into the high byte.
    crossed: bool,
}
