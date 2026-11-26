//! The cycle-accurate interpreter.
//!
//! # One cycle is one bus cycle
//!
//! The W65C816S datasheet's table 5-7 lists every instruction cycle by cycle,
//! and every one of those cycles is one of three things: a read, a write, or
//! an *internal operation* — a cycle on which VDA and VPA are both low, so no
//! memory is selected, but which still takes a clock and still drives an
//! address. All three go through [`Exec::cycle`], which is the only place a
//! cycle is charged, and which asks the board's [`AccessTiming`] how many
//! clocks of the core's domain it costs. On a SNES that is 6, 8 or 12 master
//! clocks depending on the address, and the board knows that; the CPU does
//! not.
//!
//! The pins the datasheet defines for telling those cycles apart — VDA, VPA,
//! VPB, RWB, E, M/X and MLB — are what [`BusCycle::pins`] carries, so a board
//! can tell an opcode fetch from a data read from an internal cycle without
//! decoding the instruction stream itself.
//!
//! # Interrupt sampling
//!
//! The 65C816 recognises an interrupt at the end of the instruction in
//! progress; as on the 6502 family the lines are effectively looked at before
//! the final cycle. Expressed as "poll at the start of every cycle but the
//! first", the last poll an instruction makes is the one that counts, which
//! is the same rule `cpu::mos6502` documents and the same reason `CLI` and
//! `PLP` take effect one instruction late.
//!
//! # Emulation-mode wrapping
//!
//! The datasheet (§2.21, §7) and Bruce Clark's *65C816 opcodes* tutorial on
//! 6502.org agree on the shape, and `SingleStepTests/65816` pins the detail:
//!
//! - the stack lives in page one in emulation mode, and the 6502-era push and
//!   pull instructions wrap inside it; the instructions that are new to the
//!   65C816 (`PEA`, `PEI`, `PER`, `PHD`, `PLD`, `JSL`, `RTL`, `JSR (a,X)`) run
//!   the 16-bit stack pointer and put the high byte back to `$01` only when
//!   they finish;
//! - with `DL = 0` the direct page behaves like page zero, so `d,X` and the
//!   pointer fetches of `(d)`, `(d,X)` and `(d),Y` wrap inside the page; the
//!   long-pointer modes `[d]` and `[d],Y` do not;
//! - a read-modify-write writes the unmodified value back on its modify cycle
//!   instead of spending an internal operation there, as an NMOS 6502 does.
//!
//! # Sources
//!
//! W65C816S datasheet (WDC), §2 (registers and pins), §4 (addressing modes),
//! §5 table 5-7 (cycle-by-cycle operation) and §7 (caveats); Eyes & Lichty,
//! *Programming the 65816*; Bruce Clark, *65C816 opcodes*, 6502.org, for the
//! wrapping rules and the decimal-mode flags. The bus traces of
//! `SingleStepTests/65816` (MIT) as the oracle. No emulator source was
//! consulted.

use crate::core::device::{Arbitration, CycleGate};
use crate::core::sched::TickCursor;
use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

use super::isa::{Access, Insn, Mode, Op, decode};
use super::{AccessTiming, BusCycle, Interrupt, Lines, Pins, Regs, flags};

/// The vectors, all in bank 0 (datasheet table 6-1).
mod vector {
    pub(super) const COP_NATIVE: u16 = 0xffe4;
    pub(super) const BRK_NATIVE: u16 = 0xffe6;
    pub(super) const ABORT_NATIVE: u16 = 0xffe8;
    pub(super) const NMI_NATIVE: u16 = 0xffea;
    pub(super) const IRQ_NATIVE: u16 = 0xffee;
    pub(super) const COP_EMULATION: u16 = 0xfff4;
    pub(super) const ABORT_EMULATION: u16 = 0xfff8;
    pub(super) const NMI_EMULATION: u16 = 0xfffa;
    pub(super) const RESET: u16 = 0xfffc;
    pub(super) const IRQ_EMULATION: u16 = 0xfffe;
}

/// What entered the interrupt sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Source {
    Brk,
    Cop,
    Irq,
    Nmi,
    Abort,
}

/// The architectural state one core owns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct State {
    /// The register file.
    pub regs: Regs,
    /// Bus cycles executed since power-on — reads, writes and internal
    /// operations, but not cycles `/RDY` held.
    pub cycles: u64,
    /// Clocks of the core's domain since power-on: the sum of what
    /// [`AccessTiming`] charged for every cycle, held ones included.
    pub clocks: u64,
    /// `STP` stopped the clock; only a reset restarts it.
    pub halted: bool,
    /// `WAI` is waiting for an interrupt line.
    pub waiting: bool,
    /// A reset was requested and its sequence has not run yet.
    pub reset_pending: bool,
    /// What the last poll latched, serviced before the next instruction.
    pub pending: Option<Interrupt>,
    /// The last value on the data bus — what a read nothing answers returns.
    pub open_bus: u8,
    /// How many accesses the address space refused.
    pub faults: u64,
    /// Address of the most recent refused access.
    pub last_fault: u32,
    /// Clocks already executed past the last budget, owed to the next one.
    pub debt: u64,
}

impl State {
    /// Power-on state, before the reset sequence has run.
    pub(super) const fn new() -> State {
        State {
            regs: Regs::new(),
            cycles: 0,
            clocks: 0,
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

/// An effective address and how the byte after it is found.
#[derive(Debug, Clone, Copy)]
struct Ea {
    addr: u32,
    /// The second byte of a 16-bit operand wraps inside bank 0 (direct page
    /// and stack-relative data) rather than carrying into the next bank.
    bank0: bool,
}

impl Ea {
    fn next(self) -> u32 {
        if self.bank0 {
            u32::from((self.addr as u16).wrapping_add(1))
        } else {
            (self.addr + 1) & 0xff_ffff
        }
    }
}

/// Join a bank and a 16-bit offset.
#[inline]
fn long(bank: u8, addr: u16) -> u32 {
    (u32::from(bank) << 16) | u32::from(addr)
}

/// One step's worth of execution, borrowing everything it needs.
pub(super) struct Exec<'a> {
    state: &'a mut State,
    space: &'a AddressSpace,
    lines: &'a Lines,
    attrs: MemAttrs,
    timing: Option<&'a dyn AccessTiming>,
    cursor: Option<&'a TickCursor>,
    rdy: Option<&'a dyn CycleGate>,
    /// Cycles into the current instruction; the opcode fetch is the first.
    icycle: u32,
    /// Interrupt sequences do not poll.
    polling: bool,
    /// Clocks this step has charged.
    used: u64,
    /// MLB is asserted: a read-modify-write is in progress.
    ml: bool,
    /// The instruction runs the 16-bit stack pointer even in emulation mode
    /// and restores its high byte when it finishes.
    long_stack: bool,
}

impl<'a> Exec<'a> {
    pub(super) fn new(
        state: &'a mut State,
        space: &'a AddressSpace,
        lines: &'a Lines,
        attrs: MemAttrs,
    ) -> Exec<'a> {
        Exec {
            state,
            space,
            lines,
            attrs,
            timing: None,
            cursor: None,
            rdy: None,
            icycle: 0,
            polling: true,
            used: 0,
            ml: false,
            long_stack: false,
        }
    }

    pub(super) fn with_timing(mut self, timing: Option<&'a dyn AccessTiming>) -> Exec<'a> {
        self.timing = timing;
        self
    }

    pub(super) fn with_cursor(mut self, cursor: Option<&'a TickCursor>) -> Exec<'a> {
        self.cursor = cursor;
        self
    }

    pub(super) fn with_rdy(mut self, gate: Option<&'a dyn CycleGate>) -> Exec<'a> {
        self.rdy = gate;
        self
    }

    /// Run one reset sequence, interrupt sequence, wait cycle or instruction.
    ///
    /// Returns the clocks charged — zero only when the core is stopped.
    pub(super) fn step(&mut self) -> u64 {
        if self.state.reset_pending {
            self.reset_sequence();
        } else if self.state.halted {
            return 0;
        } else if self.state.waiting && !self.wake() {
            self.stall();
        } else if let Some(kind) = self.state.pending.take() {
            self.polling = false;
            let source = match kind {
                Interrupt::Nmi => Source::Nmi,
                Interrupt::Irq => Source::Irq,
                Interrupt::Abort => Source::Abort,
            };
            // A hardware interrupt fetches the opcode it is about to discard
            // and spends an internal cycle at the same address; PC does not
            // move (table 5-7, item 22a).
            let pc = self.pc_addr();
            self.read(pc, Pins::VDA | Pins::VPA);
            self.io(pc);
            self.sequence(source);
        } else {
            self.instruction();
        }
        self.used
    }

    // -----------------------------------------------------------------
    // The clock
    // -----------------------------------------------------------------

    /// The E, M and X outputs, as the pins show them this cycle.
    fn status_pins(&self) -> u16 {
        let r = &self.state.regs;
        let mut p = 0;
        if r.e {
            p |= Pins::E;
        }
        if r.p & flags::M != 0 {
            p |= Pins::M;
        }
        if r.p & flags::X != 0 {
            p |= Pins::X;
        }
        if self.ml {
            p |= Pins::ML;
        }
        p
    }

    /// Charge one cycle driving `addr` with `pins`, and poll.
    fn cycle(&mut self, addr: u32, pins: u16, data: Option<u8>) {
        self.icycle += 1;
        let cycle = BusCycle {
            addr,
            pins: Pins(pins | self.status_pins()),
            data,
        };
        let cost = self.timing.map_or(1, |t| t.clocks(cycle).max(1));
        self.state.cycles = self.state.cycles.wrapping_add(1);
        self.charge(cost);
        self.lines.sample_edges();
        if self.icycle > 1 {
            self.poll();
        }
    }

    fn charge(&mut self, clocks: u64) {
        self.used += clocks;
        self.state.clocks = self.state.clocks.wrapping_add(clocks);
        if let Some(cursor) = self.cursor {
            cursor.set(self.state.clocks);
        }
    }

    /// Give `/RDY` its say after a cycle.
    ///
    /// The W65C816S can be held on any cycle, write included: RDY low simply
    /// stretches the cycle in progress (datasheet §2.24). A held cycle drives
    /// nothing new and costs whatever the board says a held cycle costs.
    fn arbitrate(&mut self, addr: u32, write: bool) {
        let Some(gate) = self.rdy else {
            return;
        };
        loop {
            match gate.arbitrate(
                self.state.clocks,
                u64::from(addr),
                self.state.open_bus,
                write,
            ) {
                Arbitration::Release => break,
                Arbitration::Halted => {
                    self.held(addr);
                    break;
                }
                Arbitration::Hold => self.held(addr),
                Arbitration::Steal(byte) => {
                    self.held(addr);
                    self.state.open_bus = byte;
                }
            }
        }
    }

    /// One cycle `/RDY` held: no access, no poll, but time passes.
    fn held(&mut self, addr: u32) {
        let cycle = BusCycle {
            addr,
            pins: Pins(Pins::HELD | self.status_pins()),
            data: None,
        };
        let cost = self.timing.map_or(1, |t| t.clocks(cycle).max(1));
        self.charge(cost);
        self.lines.sample_edges();
    }

    /// A `WAI` cycle: the part is stopped with RDY low and the clock running.
    fn stall(&mut self) {
        let pc = self.pc_addr();
        self.held(pc);
    }

    /// Whether an interrupt line has released a `WAI`.
    ///
    /// The datasheet is explicit that IRQB releases `WAI` **even with I
    /// set**: the mask only decides whether the handler runs or execution
    /// continues after the `WAI`.
    fn wake(&mut self) -> bool {
        let nmi = self.lines.nmi_pending();
        let irq = self.lines.irq_asserted();
        let abort = self.lines.abort_pending();
        if !nmi && !irq && !abort {
            return false;
        }
        self.state.waiting = false;
        self.state.pending = if nmi {
            Some(Interrupt::Nmi)
        } else if irq && self.state.regs.p & flags::I == 0 {
            Some(Interrupt::Irq)
        } else {
            None
        };
        true
    }

    fn poll(&mut self) {
        if !self.polling {
            return;
        }
        self.state.pending = if self.lines.nmi_pending() {
            Some(Interrupt::Nmi)
        } else if self.lines.irq_asserted() && self.state.regs.p & flags::I == 0 {
            Some(Interrupt::Irq)
        } else {
            None
        };
    }

    // -----------------------------------------------------------------
    // Bus cycles
    // -----------------------------------------------------------------

    /// A read cycle with VDA/VPA/VPB as given.
    ///
    /// A refused access is **open bus**: the 65C816 has no bus-error input,
    /// so whatever was last on the data bus is what the CPU latches — the
    /// SNES's "MDR", which real software depends on. The fault counter is how
    /// a hole in a memory map stays visible rather than silent.
    fn read(&mut self, addr: u32, pins: u16) -> u8 {
        self.cycle(addr, pins, None);
        let attrs = self.attrs.with_bus(self.state.open_bus);
        let value = match self.space.read_driven(u64::from(addr), Width::U8, attrs) {
            Ok((v, driven)) => {
                let byte = v as u8;
                if driven {
                    self.state.open_bus = byte;
                }
                byte
            }
            Err(_) => {
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = addr;
                self.state.open_bus
            }
        };
        self.arbitrate(addr, false);
        value
    }

    fn write(&mut self, addr: u32, value: u8) {
        self.cycle(addr, Pins::VDA | Pins::WRITE, Some(value));
        self.state.open_bus = value;
        let attrs = self.attrs.with_bus(value);
        if self
            .space
            .write(u64::from(addr), Width::U8, u64::from(value), attrs)
            .is_err()
        {
            self.state.faults = self.state.faults.wrapping_add(1);
            self.state.last_fault = addr;
        }
        self.arbitrate(addr, true);
    }

    /// An internal operation: an address is driven, nothing is selected.
    fn io(&mut self, addr: u32) {
        self.cycle(addr, 0, None);
        self.arbitrate(addr, false);
    }

    /// The emulation-mode modify cycle: RWB low with the old value on the
    /// bus, but VDA and VPA low, so nothing is selected.
    fn dummy_write(&mut self, addr: u32, value: u8) {
        self.cycle(addr, Pins::WRITE, Some(value));
        self.state.open_bus = value;
        self.arbitrate(addr, true);
    }

    fn read_data(&mut self, addr: u32) -> u8 {
        self.read(addr, Pins::VDA)
    }

    fn pc_addr(&self) -> u32 {
        long(self.state.regs.pbr, self.state.regs.pc)
    }

    /// The address of the byte just fetched — where several internal cycles
    /// park the bus.
    fn last_fetch(&self) -> u32 {
        long(self.state.regs.pbr, self.state.regs.pc.wrapping_sub(1))
    }

    /// Fetch an operand byte (VPA) and advance PC, which wraps in its bank.
    fn fetch(&mut self) -> u8 {
        let at = self.pc_addr();
        let b = self.read(at, Pins::VPA);
        self.state.regs.pc = self.state.regs.pc.wrapping_add(1);
        b
    }

    fn fetch16(&mut self) -> u16 {
        let lo = self.fetch();
        let hi = self.fetch();
        u16::from_le_bytes([lo, hi])
    }

    // -----------------------------------------------------------------
    // Flags and widths
    // -----------------------------------------------------------------

    fn e(&self) -> bool {
        self.state.regs.e
    }

    fn m8(&self) -> bool {
        self.state.regs.p & flags::M != 0
    }

    fn x8(&self) -> bool {
        self.state.regs.p & flags::X != 0
    }

    fn flag(&self, f: u8) -> bool {
        self.state.regs.p & f != 0
    }

    fn set_flag(&mut self, f: u8, on: bool) {
        if on {
            self.state.regs.p |= f;
        } else {
            self.state.regs.p &= !f;
        }
    }

    fn set_nz(&mut self, v: u16, wide: bool) {
        if wide {
            self.set_flag(flags::Z, v == 0);
            self.set_flag(flags::N, v & 0x8000 != 0);
        } else {
            self.set_flag(flags::Z, v & 0xff == 0);
            self.set_flag(flags::N, v & 0x80 != 0);
        }
    }

    /// Put P back into a shape the hardware can hold: emulation mode pins M
    /// and X, and an 8-bit index register has no high byte.
    fn fix_p(&mut self) {
        self.state.regs.normalize();
    }

    // -----------------------------------------------------------------
    // The stack
    // -----------------------------------------------------------------

    fn push(&mut self, v: u8) {
        let s = self.state.regs.s;
        self.write(u32::from(s), v);
        self.state.regs.s = if self.e() && !self.long_stack {
            0x100 | u16::from((s as u8).wrapping_sub(1))
        } else {
            s.wrapping_sub(1)
        };
    }

    fn push16(&mut self, v: u16) {
        self.push((v >> 8) as u8);
        self.push(v as u8);
    }

    fn pull(&mut self) -> u8 {
        let s = self.state.regs.s;
        let s = if self.e() && !self.long_stack {
            0x100 | u16::from((s as u8).wrapping_add(1))
        } else {
            s.wrapping_add(1)
        };
        self.state.regs.s = s;
        self.read_data(u32::from(s))
    }

    fn pull16(&mut self) -> u16 {
        let lo = self.pull();
        let hi = self.pull();
        u16::from_le_bytes([lo, hi])
    }

    // -----------------------------------------------------------------
    // Sequences
    // -----------------------------------------------------------------

    /// The pushes and vector fetch shared by every interrupt and by
    /// `BRK`/`COP`, once PC holds the return address.
    fn sequence(&mut self, source: Source) {
        let e = self.e();
        if !e {
            let pbr = self.state.regs.pbr;
            self.push(pbr);
        }
        let pc = self.state.regs.pc;
        self.push16(pc);
        let p = self.state.regs.p;
        // In emulation mode bit 4 of the pushed byte is B: set for a
        // software interrupt, clear for a hardware one.
        let pushed = if e {
            match source {
                Source::Brk | Source::Cop => p | flags::B,
                _ => p & !flags::B,
            }
        } else {
            p
        };
        self.push(pushed);
        self.set_flag(flags::I, true);
        self.set_flag(flags::D, false);
        self.state.regs.pbr = 0;
        let v = match (source, e) {
            (Source::Brk, false) => vector::BRK_NATIVE,
            (Source::Cop, false) => vector::COP_NATIVE,
            (Source::Irq, false) => vector::IRQ_NATIVE,
            (Source::Nmi, false) => vector::NMI_NATIVE,
            (Source::Abort, false) => vector::ABORT_NATIVE,
            (Source::Brk | Source::Irq, true) => vector::IRQ_EMULATION,
            (Source::Cop, true) => vector::COP_EMULATION,
            (Source::Nmi, true) => vector::NMI_EMULATION,
            (Source::Abort, true) => vector::ABORT_EMULATION,
        };
        if source == Source::Nmi {
            self.lines.take_nmi_pending();
        }
        let lo = self.read(u32::from(v), Pins::VDA | Pins::VPB);
        let hi = self.read(u32::from(v.wrapping_add(1)), Pins::VDA | Pins::VPB);
        self.state.regs.pc = u16::from_le_bytes([lo, hi]);
    }

    /// The reset sequence (datasheet table 5-7 item 22b, and table 2-2 for
    /// what it leaves in the registers).
    fn reset_sequence(&mut self) {
        self.polling = false;
        self.state.reset_pending = false;
        self.state.halted = false;
        self.state.waiting = false;
        self.state.pending = None;
        {
            let r = &mut self.state.regs;
            r.e = true;
            r.d = 0;
            r.dbr = 0;
            r.pbr = 0;
            r.p |= flags::M | flags::X | flags::I;
            r.p &= !flags::D;
            r.normalize();
        }
        let pc = self.pc_addr();
        self.io(pc);
        self.io(pc);
        // Three cycles where the pushes would be, with RWB held high. SL is
        // left as it was: table 2-2 lists it as not initialised.
        let s = self.state.regs.s;
        for k in 0..3u16 {
            self.io(u32::from(0x100 | (s.wrapping_sub(k) & 0xff)));
        }
        let lo = self.read(u32::from(vector::RESET), Pins::VDA | Pins::VPB);
        let hi = self.read(u32::from(vector::RESET + 1), Pins::VDA | Pins::VPB);
        self.state.regs.pc = u16::from_le_bytes([lo, hi]);
    }

    // -----------------------------------------------------------------
    // Instructions
    // -----------------------------------------------------------------

    fn instruction(&mut self) {
        // What an ABORT restores: the registers as they were before the
        // opcode fetch, PC pointing at the opcode (datasheet §2.1, ABORTB).
        let before = self.state.regs;
        let at = self.pc_addr();
        let opcode = self.read(at, Pins::VDA | Pins::VPA);
        self.state.regs.pc = self.state.regs.pc.wrapping_add(1);
        let insn = decode(opcode);
        self.execute(insn);
        if self.long_stack && self.e() {
            let s = &mut self.state.regs.s;
            *s = 0x100 | (*s & 0xff);
        }
        self.ml = false;
        if self.lines.take_abort_pending() {
            self.state.regs = before;
            self.state.waiting = false;
            self.state.halted = false;
            self.state.pending = Some(Interrupt::Abort);
        }
    }

    fn execute(&mut self, insn: Insn) {
        match insn.op {
            Op::BRK | Op::COP => {
                self.polling = false;
                self.fetch();
                self.sequence(if insn.op == Op::BRK {
                    Source::Brk
                } else {
                    Source::Cop
                });
            }
            Op::WDM => {
                // Two bytes, but the second is never fetched: the cycle is an
                // internal operation parked on it, and PC steps over it.
                let pc = self.pc_addr();
                self.io(pc);
                self.state.regs.pc = self.state.regs.pc.wrapping_add(1);
            }
            Op::JMP | Op::JML | Op::JSR | Op::JSL => self.jump(insn),
            Op::RTS => {
                let pc = self.pc_addr();
                self.io(pc);
                self.io(pc);
                let v = self.pull16();
                let s = self.state.regs.s;
                self.io(u32::from(s));
                self.state.regs.pc = v.wrapping_add(1);
            }
            Op::RTL => {
                self.long_stack = true;
                let pc = self.pc_addr();
                self.io(pc);
                self.io(pc);
                let v = self.pull16();
                let bank = self.pull();
                self.state.regs.pc = v.wrapping_add(1);
                self.state.regs.pbr = bank;
            }
            Op::RTI => {
                let pc = self.pc_addr();
                self.io(pc);
                self.io(pc);
                // The pulled P reaches the M/X outputs only once the
                // instruction ends: the PC and PBR pulls still show the old
                // widths.
                let p = self.pull();
                let v = self.pull16();
                self.state.regs.pc = v;
                if !self.e() {
                    let bank = self.pull();
                    self.state.regs.pbr = bank;
                }
                self.state.regs.p = p;
                self.fix_p();
            }
            Op::PHA | Op::PHX | Op::PHY | Op::PHP | Op::PHB | Op::PHK | Op::PHD => {
                let pc = self.pc_addr();
                self.io(pc);
                let r = self.state.regs;
                match insn.op {
                    Op::PHA if !self.m8() => self.push16(r.a),
                    Op::PHA => self.push(r.a as u8),
                    Op::PHX if !self.x8() => self.push16(r.x),
                    Op::PHX => self.push(r.x as u8),
                    Op::PHY if !self.x8() => self.push16(r.y),
                    Op::PHY => self.push(r.y as u8),
                    Op::PHP => self.push(r.p),
                    Op::PHB => self.push(r.dbr),
                    Op::PHK => self.push(r.pbr),
                    _ => {
                        self.long_stack = true;
                        self.push16(r.d);
                    }
                }
            }
            Op::PLA | Op::PLX | Op::PLY | Op::PLP | Op::PLB | Op::PLD => {
                let pc = self.pc_addr();
                self.io(pc);
                self.io(pc);
                match insn.op {
                    Op::PLA => {
                        if self.m8() {
                            let v = self.pull();
                            self.state.regs.a = (self.state.regs.a & 0xff00) | u16::from(v);
                            self.set_nz(u16::from(v), false);
                        } else {
                            let v = self.pull16();
                            self.state.regs.a = v;
                            self.set_nz(v, true);
                        }
                    }
                    Op::PLX | Op::PLY => {
                        let wide = !self.x8();
                        let v = if wide {
                            self.pull16()
                        } else {
                            u16::from(self.pull())
                        };
                        if insn.op == Op::PLX {
                            self.state.regs.x = v;
                        } else {
                            self.state.regs.y = v;
                        }
                        self.set_nz(v, wide);
                    }
                    Op::PLP => {
                        let p = self.pull();
                        self.state.regs.p = p;
                        self.fix_p();
                    }
                    Op::PLB => {
                        // New to the 65C816, so it runs the 16-bit stack
                        // pointer in emulation mode too: S = $01FF pulls
                        // from $0200.
                        self.long_stack = true;
                        let v = self.pull();
                        self.state.regs.dbr = v;
                        self.set_nz(u16::from(v), false);
                    }
                    _ => {
                        self.long_stack = true;
                        let v = self.pull16();
                        self.state.regs.d = v;
                        self.set_nz(v, true);
                    }
                }
            }
            Op::PEA => {
                self.long_stack = true;
                let v = self.fetch16();
                self.push16(v);
            }
            Op::PEI => {
                self.long_stack = true;
                let off = self.fetch();
                self.dp_penalty();
                // A new instruction, so no page wrap for the pointer even
                // with DL = 0 in emulation mode.
                let lo = self.read_data(self.dp(u16::from(off), 0, false));
                let hi = self.read_data(self.dp(u16::from(off), 1, false));
                self.push16(u16::from_le_bytes([lo, hi]));
            }
            Op::PER => {
                self.long_stack = true;
                let w = self.fetch16();
                let at = self.last_fetch();
                self.io(at);
                let v = self.state.regs.pc.wrapping_add(w);
                self.push16(v);
            }
            Op::REP | Op::SEP => {
                let imm = self.fetch();
                let at = self.last_fetch();
                self.io(at);
                if insn.op == Op::REP {
                    self.state.regs.p &= !imm;
                } else {
                    self.state.regs.p |= imm;
                }
                self.fix_p();
            }
            Op::XCE => {
                let pc = self.pc_addr();
                self.io(pc);
                let c = self.flag(flags::C);
                let e = self.state.regs.e;
                self.set_flag(flags::C, e);
                self.state.regs.e = c;
                if c {
                    self.state.regs.p |= flags::M | flags::X;
                }
                self.fix_p();
            }
            Op::XBA => {
                let pc = self.pc_addr();
                self.io(pc);
                self.io(pc);
                let a = self.state.regs.a.rotate_left(8);
                self.state.regs.a = a;
                self.set_nz(a & 0xff, false);
            }
            Op::WAI | Op::STP => {
                let pc = self.pc_addr();
                self.io(pc);
                self.io(pc);
                if insn.op == Op::WAI {
                    self.state.waiting = true;
                } else {
                    self.state.halted = true;
                }
            }
            Op::MVN | Op::MVP => self.block_move(insn.op == Op::MVN),
            Op::BRL => {
                let w = self.fetch16();
                let at = self.last_fetch();
                self.io(at);
                self.state.regs.pc = self.state.regs.pc.wrapping_add(w);
            }
            op if op.is_branch() => self.branch(op),
            _ => self.operate(insn),
        }
    }

    fn branch(&mut self, op: Op) {
        let off = self.fetch() as i8;
        let taken = match op {
            Op::BCC => !self.flag(flags::C),
            Op::BCS => self.flag(flags::C),
            Op::BEQ => self.flag(flags::Z),
            Op::BNE => !self.flag(flags::Z),
            Op::BMI => self.flag(flags::N),
            Op::BPL => !self.flag(flags::N),
            Op::BVC => !self.flag(flags::V),
            Op::BVS => self.flag(flags::V),
            _ => true,
        };
        if !taken {
            return;
        }
        let at = self.last_fetch();
        self.io(at);
        let pc = self.state.regs.pc;
        let target = pc.wrapping_add(off as u16);
        if self.e() && (target & 0xff00) != (pc & 0xff00) {
            self.io(at);
        }
        self.state.regs.pc = target;
    }

    fn jump(&mut self, insn: Insn) {
        match (insn.op, insn.mode) {
            (Op::JMP, Mode::Absolute) => {
                self.state.regs.pc = self.fetch16();
            }
            (Op::JML, Mode::AbsoluteLong) => {
                let w = self.fetch16();
                let bank = self.fetch();
                self.state.regs.pc = w;
                self.state.regs.pbr = bank;
            }
            (Op::JMP, Mode::AbsoluteIndirect) => {
                let a = self.fetch16();
                let lo = self.read_data(u32::from(a));
                let hi = self.read_data(u32::from(a.wrapping_add(1)));
                self.state.regs.pc = u16::from_le_bytes([lo, hi]);
            }
            (Op::JML, _) => {
                let a = self.fetch16();
                let lo = self.read_data(u32::from(a));
                let hi = self.read_data(u32::from(a.wrapping_add(1)));
                let bank = self.read_data(u32::from(a.wrapping_add(2)));
                self.state.regs.pc = u16::from_le_bytes([lo, hi]);
                self.state.regs.pbr = bank;
            }
            (Op::JMP, _) => {
                let a = self.fetch16();
                let at = self.last_fetch();
                self.io(at);
                let ptr = a.wrapping_add(self.state.regs.x);
                let pbr = self.state.regs.pbr;
                let lo = self.read_data(long(pbr, ptr));
                let hi = self.read_data(long(pbr, ptr.wrapping_add(1)));
                self.state.regs.pc = u16::from_le_bytes([lo, hi]);
            }
            (Op::JSR, Mode::Absolute) => {
                let a = self.fetch16();
                let at = self.last_fetch();
                self.io(at);
                let ret = self.state.regs.pc.wrapping_sub(1);
                self.push16(ret);
                self.state.regs.pc = a;
            }
            (Op::JSL, _) => {
                self.long_stack = true;
                let a = self.fetch16();
                let pbr = self.state.regs.pbr;
                self.push(pbr);
                let s = self.state.regs.s.wrapping_add(1);
                self.io(u32::from(s));
                let bank = self.fetch();
                let ret = self.state.regs.pc.wrapping_sub(1);
                self.push16(ret);
                self.state.regs.pc = a;
                self.state.regs.pbr = bank;
            }
            _ => {
                // JSR (a,x)
                self.long_stack = true;
                let lo = self.fetch();
                let ret = self.state.regs.pc;
                self.push16(ret);
                let hi = self.fetch();
                let at = self.last_fetch();
                self.io(at);
                let ptr = u16::from_le_bytes([lo, hi]).wrapping_add(self.state.regs.x);
                let pbr = self.state.regs.pbr;
                let lo = self.read_data(long(pbr, ptr));
                let hi = self.read_data(long(pbr, ptr.wrapping_add(1)));
                self.state.regs.pc = u16::from_le_bytes([lo, hi]);
            }
        }
    }

    /// One byte of `MVN`/`MVP`. The instruction re-executes itself — PC goes
    /// back three bytes — until the count in C runs out at `$FFFF`, which is
    /// how an interrupt can land between any two bytes of a move.
    fn block_move(&mut self, increment: bool) {
        let dst = self.fetch();
        let src = self.fetch();
        let r = self.state.regs;
        let v = self.read_data(long(src, r.x));
        let to = long(dst, r.y);
        self.write(to, v);
        self.state.regs.dbr = dst;
        self.io(to);
        self.io(to);
        let step = |v: u16, x8: bool| {
            let n = if increment {
                v.wrapping_add(1)
            } else {
                v.wrapping_sub(1)
            };
            if x8 { n & 0xff } else { n }
        };
        let x8 = self.x8();
        let r = &mut self.state.regs;
        r.x = step(r.x, x8);
        r.y = step(r.y, x8);
        r.a = r.a.wrapping_sub(1);
        if r.a != 0xffff {
            r.pc = r.pc.wrapping_sub(3);
        }
    }

    // -----------------------------------------------------------------
    // Addressing
    // -----------------------------------------------------------------

    /// The extra internal cycle every direct-page mode spends when `DL` is
    /// not zero (table 5-7 note 2), parked on the operand byte.
    fn dp_penalty(&mut self) {
        if self.state.regs.d & 0xff != 0 {
            let at = self.last_fetch();
            self.io(at);
        }
    }

    /// A direct-page address: `D + off + k` in bank 0.
    ///
    /// With `page` set, emulation mode with `DL = 0` keeps the result in the
    /// direct page the way page zero wraps on a 6502.
    fn dp(&self, off: u16, k: u16, page: bool) -> u32 {
        let d = self.state.regs.d;
        if page && self.e() && d & 0xff == 0 {
            u32::from((d & 0xff00) | (off.wrapping_add(k) & 0xff))
        } else {
            u32::from(d.wrapping_add(off).wrapping_add(k))
        }
    }

    fn index(&self, y: bool) -> u16 {
        if y {
            self.state.regs.y
        } else {
            self.state.regs.x
        }
    }

    /// The internal cycle an indexed access spends fixing up its address:
    /// always for a write or a read-modify-write, and for a read when the
    /// index is 16 bits or the low byte carried (table 5-7 note 4).
    fn index_penalty(&mut self, base: u32, idx: u16, access: Access) {
        let carry = (base & 0xff) + u32::from(idx & 0xff) > 0xff;
        if access != Access::Read || !self.x8() || carry {
            let at = (base & 0xff_ff00) | ((base + u32::from(idx)) & 0xff);
            self.io(at);
        }
    }

    fn resolve(&mut self, mode: Mode, access: Access) -> Ea {
        let dbr = self.state.regs.dbr;
        let linear = |addr: u32| Ea {
            addr: addr & 0xff_ffff,
            bank0: false,
        };
        match mode {
            Mode::Absolute => {
                let a = self.fetch16();
                linear(long(dbr, a))
            }
            Mode::AbsoluteX | Mode::AbsoluteY => {
                let a = self.fetch16();
                let idx = self.index(mode == Mode::AbsoluteY);
                let base = long(dbr, a);
                self.index_penalty(base, idx, access);
                linear(base + u32::from(idx))
            }
            Mode::AbsoluteLong | Mode::AbsoluteLongX => {
                let a = self.fetch16();
                let bank = self.fetch();
                let base = long(bank, a);
                if mode == Mode::AbsoluteLongX {
                    linear(base + u32::from(self.state.regs.x))
                } else {
                    linear(base)
                }
            }
            Mode::Direct => {
                let off = self.fetch();
                self.dp_penalty();
                Ea {
                    addr: self.dp(u16::from(off), 0, true),
                    bank0: true,
                }
            }
            Mode::DirectX | Mode::DirectY => {
                let off = self.fetch();
                self.dp_penalty();
                let at = self.last_fetch();
                self.io(at);
                let idx = self.index(mode == Mode::DirectY);
                Ea {
                    addr: self.dp(u16::from(off).wrapping_add(idx), 0, true),
                    bank0: true,
                }
            }
            Mode::DirectIndirect | Mode::DirectIndexedIndirect => {
                let off = self.fetch();
                self.dp_penalty();
                let mut base = u16::from(off);
                if mode == Mode::DirectIndexedIndirect {
                    let at = self.last_fetch();
                    self.io(at);
                    base = base.wrapping_add(self.state.regs.x);
                }
                let lo = self.read_data(self.dp(base, 0, true));
                let hi = self.read_data(self.dp(base, 1, true));
                linear(long(dbr, u16::from_le_bytes([lo, hi])))
            }
            Mode::DirectIndirectIndexed => {
                let off = self.fetch();
                self.dp_penalty();
                let lo = self.read_data(self.dp(u16::from(off), 0, true));
                let hi = self.read_data(self.dp(u16::from(off), 1, true));
                let base = long(dbr, u16::from_le_bytes([lo, hi]));
                let y = self.state.regs.y;
                self.index_penalty(base, y, access);
                linear(base + u32::from(y))
            }
            Mode::DirectIndirectLong | Mode::DirectIndirectLongIndexed => {
                let off = u16::from(self.fetch());
                self.dp_penalty();
                let lo = self.read_data(self.dp(off, 0, false));
                let hi = self.read_data(self.dp(off, 1, false));
                let bank = self.read_data(self.dp(off, 2, false));
                let base = long(bank, u16::from_le_bytes([lo, hi]));
                if mode == Mode::DirectIndirectLongIndexed {
                    linear(base + u32::from(self.state.regs.y))
                } else {
                    linear(base)
                }
            }
            Mode::StackRelative => {
                let off = self.fetch();
                let at = self.last_fetch();
                self.io(at);
                Ea {
                    addr: u32::from(self.state.regs.s.wrapping_add(u16::from(off))),
                    bank0: true,
                }
            }
            Mode::StackRelativeIndirectIndexed => {
                let off = self.fetch();
                let at = self.last_fetch();
                self.io(at);
                let p = self.state.regs.s.wrapping_add(u16::from(off));
                let lo = self.read_data(u32::from(p));
                let hi = self.read_data(u32::from(p.wrapping_add(1)));
                self.io(u32::from(p.wrapping_add(1)));
                let base = long(dbr, u16::from_le_bytes([lo, hi]));
                linear(base + u32::from(self.state.regs.y))
            }
            _ => unreachable!("{mode:?} has no memory operand"),
        }
    }

    // -----------------------------------------------------------------
    // The generic path
    // -----------------------------------------------------------------

    fn operate(&mut self, insn: Insn) {
        let wide = if insn.op.index_width() {
            !self.x8()
        } else {
            !self.m8()
        };
        match insn.mode {
            Mode::Implied => {
                let pc = self.pc_addr();
                self.io(pc);
                self.implied(insn.op);
                return;
            }
            Mode::Accumulator => {
                let pc = self.pc_addr();
                self.io(pc);
                let a = self.state.regs.a;
                let v = self.modify(insn.op, a, wide);
                self.state.regs.a = if wide { v } else { (a & 0xff00) | (v & 0xff) };
                return;
            }
            Mode::ImmediateM | Mode::ImmediateX => {
                let v = if wide {
                    self.fetch16()
                } else {
                    u16::from(self.fetch())
                };
                self.alu(insn.op, v, wide, true);
                return;
            }
            _ => {}
        }
        let ea = self.resolve(insn.mode, insn.access);
        match insn.access {
            Access::Read => {
                let lo = self.read_data(ea.addr);
                let v = if wide {
                    let hi = self.read_data(ea.next());
                    u16::from_le_bytes([lo, hi])
                } else {
                    u16::from(lo)
                };
                self.alu(insn.op, v, wide, false);
            }
            Access::Write => {
                let r = self.state.regs;
                let v = match insn.op {
                    Op::STA => r.a,
                    Op::STX => r.x,
                    Op::STY => r.y,
                    _ => 0,
                };
                self.write(ea.addr, v as u8);
                if wide {
                    self.write(ea.next(), (v >> 8) as u8);
                }
            }
            Access::Modify => {
                self.ml = true;
                let lo = self.read_data(ea.addr);
                let old = if wide {
                    let hi = self.read_data(ea.next());
                    u16::from_le_bytes([lo, hi])
                } else {
                    u16::from(lo)
                };
                if self.e() {
                    self.dummy_write(ea.addr, lo);
                } else {
                    let last = if wide { ea.next() } else { ea.addr };
                    self.io(last);
                }
                let v = self.modify(insn.op, old, wide);
                if wide {
                    self.write(ea.next(), (v >> 8) as u8);
                }
                self.write(ea.addr, v as u8);
                self.ml = false;
            }
            Access::None => unreachable!("{insn:?} has no operand access"),
        }
    }

    fn implied(&mut self, op: Op) {
        let x8 = self.x8();
        let m8 = self.m8();
        let r = self.state.regs;
        match op {
            Op::CLC => self.set_flag(flags::C, false),
            Op::SEC => self.set_flag(flags::C, true),
            Op::CLI => self.set_flag(flags::I, false),
            Op::SEI => self.set_flag(flags::I, true),
            Op::CLD => self.set_flag(flags::D, false),
            Op::SED => self.set_flag(flags::D, true),
            Op::CLV => self.set_flag(flags::V, false),
            Op::NOP => {}
            Op::INX | Op::DEX | Op::INY | Op::DEY => {
                let y = matches!(op, Op::INY | Op::DEY);
                let v = if y { r.y } else { r.x };
                let v = if matches!(op, Op::INX | Op::INY) {
                    v.wrapping_add(1)
                } else {
                    v.wrapping_sub(1)
                };
                let v = if x8 { v & 0xff } else { v };
                if y {
                    self.state.regs.y = v;
                } else {
                    self.state.regs.x = v;
                }
                self.set_nz(v, !x8);
            }
            Op::TAX | Op::TAY => {
                let v = if x8 { r.a & 0xff } else { r.a };
                if op == Op::TAX {
                    self.state.regs.x = v;
                } else {
                    self.state.regs.y = v;
                }
                self.set_nz(v, !x8);
            }
            Op::TXA | Op::TYA => {
                let src = if op == Op::TXA { r.x } else { r.y };
                if m8 {
                    self.state.regs.a = (r.a & 0xff00) | (src & 0xff);
                    self.set_nz(src, false);
                } else {
                    self.state.regs.a = src;
                    self.set_nz(src, true);
                }
            }
            Op::TXY | Op::TYX => {
                let v = if op == Op::TXY { r.x } else { r.y };
                if op == Op::TXY {
                    self.state.regs.y = v;
                } else {
                    self.state.regs.x = v;
                }
                self.set_nz(v, !x8);
            }
            Op::TSX => {
                let v = if x8 { r.s & 0xff } else { r.s };
                self.state.regs.x = v;
                self.set_nz(v, !x8);
            }
            Op::TXS => {
                self.state.regs.s = if r.e { 0x100 | (r.x & 0xff) } else { r.x };
            }
            Op::TCS => {
                self.state.regs.s = if r.e { 0x100 | (r.a & 0xff) } else { r.a };
            }
            Op::TSC => {
                self.state.regs.a = r.s;
                self.set_nz(r.s, true);
            }
            Op::TCD => {
                self.state.regs.d = r.a;
                self.set_nz(r.a, true);
            }
            Op::TDC => {
                self.state.regs.a = r.d;
                self.set_nz(r.d, true);
            }
            _ => unreachable!("{op:?} is not an implied operation"),
        }
    }

    /// The read-only operations: loads, the ALU, compares and `BIT`.
    fn alu(&mut self, op: Op, v: u16, wide: bool, immediate: bool) {
        let mask: u16 = if wide { 0xffff } else { 0x00ff };
        let a = self.state.regs.a;
        let set_a = |s: &mut Self, r: u16| {
            s.state.regs.a = if wide { r } else { (a & 0xff00) | (r & 0xff) };
            s.set_nz(r, wide);
        };
        match op {
            Op::LDA => set_a(self, v),
            Op::ORA => set_a(self, a | v),
            Op::AND => set_a(self, a & v),
            Op::EOR => set_a(self, a ^ v),
            Op::ADC => {
                let r = self.add(a & mask, v, wide);
                set_a(self, r);
            }
            Op::SBC => {
                let r = self.subtract(a & mask, v, wide);
                set_a(self, r);
            }
            Op::CMP | Op::CPX | Op::CPY => {
                let reg = match op {
                    Op::CMP => a,
                    Op::CPX => self.state.regs.x,
                    _ => self.state.regs.y,
                } & mask;
                let r = reg.wrapping_sub(v) & mask;
                self.set_flag(flags::C, reg >= v);
                self.set_nz(r, wide);
            }
            Op::BIT => {
                self.set_flag(flags::Z, a & v & mask == 0);
                if !immediate {
                    let sign = if wide { 0x8000 } else { 0x80 };
                    self.set_flag(flags::N, v & sign != 0);
                    self.set_flag(flags::V, v & (sign >> 1) != 0);
                }
            }
            Op::LDX => {
                self.state.regs.x = v;
                self.set_nz(v, wide);
            }
            Op::LDY => {
                self.state.regs.y = v;
                self.set_nz(v, wide);
            }
            _ => unreachable!("{op:?} is not a read operation"),
        }
    }

    /// The read-modify-write operations, on memory or the accumulator.
    fn modify(&mut self, op: Op, v: u16, wide: bool) -> u16 {
        let mask: u16 = if wide { 0xffff } else { 0x00ff };
        let sign: u16 = if wide { 0x8000 } else { 0x80 };
        let v = v & mask;
        let r = match op {
            Op::ASL => {
                self.set_flag(flags::C, v & sign != 0);
                (v << 1) & mask
            }
            Op::LSR => {
                self.set_flag(flags::C, v & 1 != 0);
                v >> 1
            }
            Op::ROL => {
                let c = u16::from(self.flag(flags::C));
                self.set_flag(flags::C, v & sign != 0);
                ((v << 1) | c) & mask
            }
            Op::ROR => {
                let c = if self.flag(flags::C) { sign } else { 0 };
                self.set_flag(flags::C, v & 1 != 0);
                (v >> 1) | c
            }
            Op::INC => v.wrapping_add(1) & mask,
            Op::DEC => v.wrapping_sub(1) & mask,
            Op::TSB | Op::TRB => {
                let a = self.state.regs.a & mask;
                self.set_flag(flags::Z, a & v == 0);
                return if op == Op::TSB { v | a } else { v & !a };
            }
            _ => unreachable!("{op:?} is not a read-modify-write"),
        };
        self.set_nz(r, wide);
        r
    }

    /// `ADC`, binary or decimal, eight or sixteen bits.
    ///
    /// Decimal mode adds digit by digit, carrying a decimal carry between
    /// them, and sets N and Z from the corrected result (unlike an NMOS
    /// 6502). V is taken before the top digit is corrected, which is where
    /// the binary adder's sign overflow is on this family (Clark, *Decimal
    /// Mode*, 6502.org).
    fn add(&mut self, a: u16, b: u16, wide: bool) -> u16 {
        let sign: u32 = if wide { 0x8000 } else { 0x80 };
        let mask: u32 = if wide { 0xffff } else { 0xff };
        let (a, b) = (u32::from(a), u32::from(b));
        let c = u32::from(self.flag(flags::C));
        let (r, carry, v) = if self.flag(flags::D) {
            let digits = if wide { 4 } else { 2 };
            let mut result = 0u32;
            let mut carry = c;
            let mut overflow = false;
            for i in 0..digits {
                let shift = 4 * i;
                let mut s = ((a >> shift) & 0xf) + ((b >> shift) & 0xf) + carry;
                if i == digits - 1 {
                    let pre = (result | (s << shift)) & mask;
                    overflow = (!(a ^ b) & (a ^ pre) & sign) != 0;
                }
                if s > 9 {
                    s += 6;
                }
                carry = u32::from(s > 0xf);
                result |= (s & 0xf) << shift;
            }
            (result, carry != 0, overflow)
        } else {
            let r = a + b + c;
            let v = (!(a ^ b) & (a ^ r) & sign) != 0;
            (r & mask, r > mask, v)
        };
        self.set_flag(flags::C, carry);
        self.set_flag(flags::V, v);
        r as u16
    }

    /// `SBC`: the adder with the operand inverted, and in decimal mode a
    /// digit-wise correction where a digit borrowed.
    fn subtract(&mut self, a: u16, b: u16, wide: bool) -> u16 {
        let sign: u32 = if wide { 0x8000 } else { 0x80 };
        let mask: u32 = if wide { 0xffff } else { 0xff };
        let (a, nb) = (u32::from(a), u32::from(!b) & mask);
        let c = u32::from(self.flag(flags::C));
        let (r, carry, v) = if self.flag(flags::D) {
            let digits = if wide { 4 } else { 2 };
            let mut result = 0u32;
            let mut carry = c;
            let mut overflow = false;
            for i in 0..digits {
                let shift = 4 * i;
                let mut s = ((a >> shift) & 0xf) + ((nb >> shift) & 0xf) + carry;
                if i == digits - 1 {
                    let pre = (result | (s << shift)) & mask;
                    overflow = (!(a ^ nb) & (a ^ pre) & sign) != 0;
                }
                carry = u32::from(s > 0xf);
                if carry == 0 {
                    s = s.wrapping_sub(6);
                }
                result |= (s & 0xf) << shift;
            }
            (result, carry != 0, overflow)
        } else {
            let r = a + nb + c;
            let v = (!(a ^ nb) & (a ^ r) & sign) != 0;
            (r & mask, r > mask, v)
        };
        self.set_flag(flags::C, carry);
        self.set_flag(flags::V, v);
        r as u16
    }
}
