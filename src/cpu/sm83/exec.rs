//! The M-cycle-accurate interpreter.
//!
//! # One machine cycle is one bus access, or one documented idle
//!
//! An SM83 machine cycle is four clocks and does exactly one thing: a read, a
//! write, or a documented internal operation. So this interpreter has no cycle
//! counter to add to independently of what it does. [`Exec::read`] and
//! [`Exec::write`] *are* the clock, and [`Exec::idle`] is the third kind — the
//! `PUSH` predecrement, the taken branch's reload, `ADD HL,rr`'s second half.
//! Every `idle` call in this file carries the reason it exists, because an
//! unexplained one is indistinguishable from a fudge factor.
//!
//! The consequence is that `LD A,(HL)` costs two cycles *because* it fetched an
//! opcode and read a byte, not because a table says 2. Change the operand and
//! the timing follows, which is what keeps
//! [`isa`](super::isa) free of a cycle column.
//!
//! # Interrupts
//!
//! The SM83 has five, at `$40`, `$48`, `$50`, `$58` and `$60`, prioritised low
//! bit first. Two registers gate them: `IF` (`$FF0F`), which devices set, and
//! `IE` (`$FFFF`), which the program sets. Both live in [`super::Lines`] — that
//! is, outside this interpreter's lock — because a device raising an interrupt
//! from inside a write the CPU itself issued must not have to re-enter the CPU's
//! own critical section (`ROADMAP.md` §4.7).
//!
//! Three behaviours here are what the accuracy suites are actually testing:
//!
//! * **`EI` is one instruction late.** `IME` is set as the *following*
//!   instruction begins, not when `EI` retires, so `EI` immediately followed by
//!   `DI` never lets an interrupt through — `DI` clears the flag that `EI`
//!   armed. See [`Exec::step`], where the ordering is three lines and the whole
//!   behaviour.
//! * **The vector is chosen after the return address is pushed.** The dispatch
//!   sequence pushes `PC` and only then reads `IE & IF` to decide where to go.
//!   A stack that has descended to `$FFFE`/`$FFFF` therefore overwrites `IE`
//!   with the pushed byte, and the interrupt can end up dispatching to a
//!   different vector — or to `$0000` when the overwrite leaves nothing
//!   pending. This is Gekkio's `ie_push`, and it falls out of doing the steps in
//!   the documented order rather than being special-cased.
//! * **The `HALT` bug.** `HALT` executed with `IME` clear while an interrupt is
//!   already pending does not halt: the processor reads the next byte and then
//!   fails to advance `PC` past it, so that byte executes twice.
//!
//! # Sources
//!
//! [Pan Docs](https://gbdev.io/pandocs/) (CC0) — *CPU Instruction Set*, *CPU
//! Registers and Flags*, *Interrupts*, *Halt*. Sub-instruction ordering is from
//! Gekkio's *Game Boy: Complete Technical Reference*. No emulator source was
//! consulted.

use crate::core::sched::TickCursor;
use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

use super::isa::{Cond, Insn, Op, Operand, Reg8, Reg16, decode, decode_cb};
use super::{Config, Lines, Regs, flags};

/// The five interrupt vectors, in priority order: VBlank, LCD STAT, timer,
/// serial, joypad (Pan Docs, *Interrupts*).
pub const VECTORS: [u16; 5] = [0x0040, 0x0048, 0x0050, 0x0058, 0x0060];

/// What the core is doing between instructions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(super) enum Mode {
    /// Fetching and executing.
    #[default]
    Running,
    /// `HALT`: the clock runs, the CPU does not, until `IE & IF` is non-zero.
    Halted,
    /// `STOP`: waiting for a joypad line to go active.
    Stopped,
    /// One of the eleven unimplemented opcodes was executed. Only a reset gets
    /// out of this, which is what the hardware does.
    Locked,
}

impl Mode {
    /// The tag this mode is snapshotted as.
    pub(super) const fn tag(self) -> u8 {
        match self {
            Mode::Running => 0,
            Mode::Halted => 1,
            Mode::Stopped => 2,
            Mode::Locked => 3,
        }
    }

    /// The mode a snapshot tag names.
    pub(super) const fn from_tag(tag: u8) -> Option<Mode> {
        match tag {
            0 => Some(Mode::Running),
            1 => Some(Mode::Halted),
            2 => Some(Mode::Stopped),
            3 => Some(Mode::Locked),
            _ => None,
        }
    }
}

/// The architectural state one core owns.
///
/// Split from [`super::Sm83`] because the interrupt *registers* live outside the
/// lock: a device setting `IF` from inside a CPU-initiated write would otherwise
/// re-enter the CPU's own critical section and deadlock (the re-entrancy
/// contract, `ROADMAP.md` §4.7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct State {
    /// The register file.
    pub regs: Regs,
    /// Machine cycles executed since power-on. **M-cycles, not clocks**: this
    /// core's clock domain counts M-cycles, so one tick here is four crystal
    /// periods.
    pub cycles: u64,
    /// The interrupt master enable.
    pub ime: bool,
    /// `EI` was executed and `IME` is owed to the next instruction.
    pub ei_pending: bool,
    /// What the core is doing between instructions.
    pub mode: Mode,
    /// The `HALT` bug is armed: the next opcode fetch must not advance `PC`.
    pub halt_bug: bool,
    /// Cycles already executed past the last budget, owed to the next one.
    ///
    /// An instruction cannot be stopped halfway, so a budget that runs out in
    /// the middle of one is overrun by up to
    /// [`MAX_INSTRUCTION_CYCLES`](super::MAX_INSTRUCTION_CYCLES) cycles. The
    /// scheduler treats an overrun as fatal (`core::sched`), so the overshoot is
    /// carried here and deducted from the following budget instead. See
    /// [`Sm83::run_budget`](super::Sm83::run_budget) for why carrying it beats
    /// declining to start the instruction.
    pub debt: u64,
    /// How many accesses the address space refused.
    pub faults: u64,
    /// Address of the most recent refused access.
    pub last_fault: u16,
}

impl State {
    /// Power-on state.
    ///
    /// Registers are zero and `PC` is `$0000`, which is where a real DMG starts
    /// — in the boot ROM. A machine with no boot ROM image asks
    /// [`Config::post_boot`](super::Config::post_boot) for the register file the
    /// boot ROM would have left behind; that is applied by `reset`, not here, so
    /// that this stays the honest power-on state.
    pub(super) const fn new() -> State {
        State {
            regs: Regs::new(),
            cycles: 0,
            ime: false,
            ei_pending: false,
            mode: Mode::Running,
            halt_bug: false,
            debt: 0,
            faults: 0,
            last_fault: 0,
        }
    }
}

/// One step's worth of execution, borrowing everything it needs.
///
/// Created per step rather than stored: it holds bookkeeping that is meaningless
/// between instructions, and dropping it makes that explicit.
pub(super) struct Exec<'a> {
    state: &'a mut State,
    space: &'a AddressSpace,
    lines: &'a Lines,
    attrs: MemAttrs,
    /// Where to publish the cycle counter as the step runs, if anyone asked.
    cursor: Option<&'a TickCursor>,
    /// Cycles this step has charged.
    used: u64,
}

impl<'a> Exec<'a> {
    /// Borrow a core for one step.
    pub(super) fn new(
        state: &'a mut State,
        space: &'a AddressSpace,
        cfg: &Config,
        lines: &'a Lines,
    ) -> Exec<'a> {
        Exec {
            state,
            space,
            lines,
            attrs: MemAttrs::DEFAULT.with_requester(cfg.requester),
            cursor: None,
            used: 0,
        }
    }

    /// Publish each machine cycle to `cursor` as it is charged.
    pub(super) fn with_cursor(mut self, cursor: Option<&'a TickCursor>) -> Exec<'a> {
        self.cursor = cursor;
        self
    }

    /// Run one instruction, one interrupt dispatch, or one idle cycle.
    ///
    /// Returns the machine cycles charged. **Never zero**: a halted SM83 is not
    /// a stopped one, its clock keeps running, and the timer and the PPU keep
    /// counting — which is precisely how it gets woken again. A core that
    /// reported zero here would starve the very device that has to interrupt it.
    pub(super) fn step(&mut self) -> u64 {
        match self.state.mode {
            // Only a reset leaves this, so there is nothing to test for.
            Mode::Locked => {
                self.idle();
                return self.used;
            }
            Mode::Halted => {
                if self.pending() == 0 {
                    self.idle();
                    return self.used;
                }
                // Pan Docs, *Halt*: the wake is unconditional on `IE & IF`, and
                // whether the handler runs is `IME`'s business, checked below.
                self.state.mode = Mode::Running;
            }
            Mode::Stopped => {
                if !self.lines.stop_wake() {
                    self.idle();
                    return self.used;
                }
                self.state.mode = Mode::Running;
            }
            Mode::Running => {}
        }

        if self.state.ime && self.pending() != 0 {
            self.dispatch_interrupt();
            return self.used;
        }
        // `EI`'s effect lands here — as the *next* instruction begins, after the
        // check above has already been made with the old `IME`. That is the
        // whole of the one-instruction delay, and it is also why `EI; DI` never
        // lets an interrupt through: `DI` runs below and clears what this set.
        if self.state.ei_pending {
            self.state.ime = true;
            self.state.ei_pending = false;
        }
        self.instruction();
        self.used
    }

    /// Which interrupts are both requested and enabled.
    fn pending(&self) -> u8 {
        self.lines.pending()
    }

    // -----------------------------------------------------------------
    // The clock: one access or one documented idle per cycle
    // -----------------------------------------------------------------

    /// Charge one machine cycle.
    ///
    /// The counter moves *before* the access, and the published value is
    /// therefore the number of the cycle the access falls in rather than the one
    /// before it. That is the right end of the M-cycle: the SM83 puts the
    /// address out over the first half and latches the data at the end, so a
    /// device answering this access has to have run to the boundary this cycle
    /// closes (Gekkio, *Game Boy: Complete Technical Reference*, §"Memory
    /// access timing"). Publishing the boundary it *opened* would put every
    /// read four dots early, which is a whole PPU dot-quartet and shows up
    /// directly in Gekkio's `intr_2_mode0_timing` group.
    fn tick(&mut self) {
        self.used += 1;
        self.state.cycles = self.state.cycles.wrapping_add(1);
        if let Some(cursor) = self.cursor {
            cursor.set(self.state.cycles);
        }
    }

    /// One machine cycle in which the chip does something internal rather than
    /// touching the bus.
    ///
    /// Unlike a 6502 — which has no such cycle and where every clock is an
    /// access — the SM83 genuinely idles, and the count of idles is part of
    /// each instruction's documented timing. Every caller says which one it is.
    fn idle(&mut self) {
        self.tick();
    }

    /// One read cycle.
    fn read(&mut self, addr: u16) -> u8 {
        self.tick();
        match self.space.read(u64::from(addr), Width::U8, self.attrs) {
            Ok(v) => v as u8,
            Err(_) => {
                // The SM83 has no bus-error input. An address nobody answers
                // reads as `$FF` on a Game Boy — the data bus is pulled up —
                // and the space's own unassigned policy normally produces that;
                // reaching here means the policy said *fault*, and the honest
                // answer is still the idle bus level. The counter is how anyone
                // finds out.
                self.state.faults = self.state.faults.wrapping_add(1);
                self.state.last_fault = addr;
                0xff
            }
        }
    }

    /// One write cycle.
    fn write(&mut self, addr: u16, value: u8) {
        self.tick();
        if self
            .space
            .write(u64::from(addr), Width::U8, u64::from(value), self.attrs)
            .is_err()
        {
            self.state.faults = self.state.faults.wrapping_add(1);
            self.state.last_fault = addr;
        }
    }

    /// Read the byte at `PC` and advance it.
    ///
    /// The one place the `HALT` bug lives: when it is armed, `PC` is left where
    /// it was, so the byte just read is read again as the next opcode.
    fn fetch(&mut self) -> u8 {
        let pc = self.state.regs.pc;
        let byte = self.read(pc);
        if self.state.halt_bug {
            self.state.halt_bug = false;
        } else {
            // Guest arithmetic wraps: `PC` is 16 bits and `$ffff` is followed by
            // `$0000`.
            self.state.regs.pc = pc.wrapping_add(1);
        }
        byte
    }

    /// Fetch a little-endian immediate word: two cycles, low byte first.
    fn fetch16(&mut self) -> u16 {
        let lo = self.fetch();
        let hi = self.fetch();
        u16::from(lo) | (u16::from(hi) << 8)
    }

    /// Push a byte: predecrement, then write.
    fn push8(&mut self, value: u8) {
        let sp = self.state.regs.sp.wrapping_sub(1);
        self.state.regs.sp = sp;
        self.write(sp, value);
    }

    /// Push a word, high byte first — which is what puts the low byte at the
    /// lower address.
    fn push16(&mut self, value: u16) {
        self.push8((value >> 8) as u8);
        self.push8(value as u8);
    }

    /// Pop a byte: read, then postincrement.
    fn pop8(&mut self) -> u8 {
        let sp = self.state.regs.sp;
        let byte = self.read(sp);
        self.state.regs.sp = sp.wrapping_add(1);
        byte
    }

    /// Pop a word, low byte first.
    fn pop16(&mut self) -> u16 {
        let lo = self.pop8();
        let hi = self.pop8();
        u16::from(lo) | (u16::from(hi) << 8)
    }

    // -----------------------------------------------------------------
    // Interrupts
    // -----------------------------------------------------------------

    /// The five-cycle dispatch sequence (Pan Docs, *Interrupts*).
    ///
    /// Two internal cycles, the two pushes, and the cycle that loads `PC`. The
    /// order matters and is not cosmetic: **the vector is decided after the
    /// pushes**, from a fresh read of `IE & IF`, so a stack that has walked down
    /// onto `$FFFF` overwrites `IE` with the byte it just pushed and changes —
    /// or removes — the interrupt being taken. `$0000` is where the chip goes
    /// when the re-read finds nothing.
    fn dispatch_interrupt(&mut self) {
        self.state.ime = false;
        self.state.ei_pending = false;
        self.idle(); // dispatch cycle 1: nothing on the bus
        self.idle(); // dispatch cycle 2: still nothing
        let pc = self.state.regs.pc;
        self.push8((pc >> 8) as u8);
        self.push8(pc as u8);
        let pending = self.pending();
        let target = if pending == 0 {
            0x0000
        } else {
            let bit = pending.trailing_zeros() as u8;
            self.lines.clear_request(bit);
            VECTORS[bit as usize]
        };
        self.state.regs.pc = target;
        self.idle(); // dispatch cycle 5: `PC` is loaded
    }

    // -----------------------------------------------------------------
    // Flags
    // -----------------------------------------------------------------

    fn flag(&self, mask: u8) -> bool {
        self.state.regs.f & mask != 0
    }

    fn set_flag(&mut self, mask: u8, on: bool) {
        if on {
            self.state.regs.f |= mask;
        } else {
            self.state.regs.f &= !mask;
        }
    }

    /// Set all four flags at once.
    ///
    /// Writing `F` wholesale rather than bit by bit is safe here precisely
    /// because the low nibble has no storage: every flag-setting instruction
    /// that reaches this function defines all four, and the bits below them are
    /// zero on hardware too.
    fn set_flags(&mut self, z: bool, n: bool, h: bool, c: bool) {
        let mut f = 0u8;
        if z {
            f |= flags::Z;
        }
        if n {
            f |= flags::N;
        }
        if h {
            f |= flags::H;
        }
        if c {
            f |= flags::C;
        }
        self.state.regs.f = f;
    }

    /// Whether a branch condition holds.
    fn cond(&self, c: Cond) -> bool {
        match c {
            Cond::Nz => !self.flag(flags::Z),
            Cond::Z => self.flag(flags::Z),
            Cond::Nc => !self.flag(flags::C),
            Cond::C => self.flag(flags::C),
        }
    }

    // -----------------------------------------------------------------
    // Operands
    // -----------------------------------------------------------------

    /// The effective address of a memory operand, charging any immediate
    /// fetches it needs.
    ///
    /// The `(HL±)` post-adjust happens here, once, so that a read-modify-write
    /// through one of them cannot adjust twice. Nothing in the instruction set
    /// does that, and nothing should be able to.
    fn address(&mut self, operand: Operand) -> u16 {
        match operand {
            Operand::MemHl => self.state.regs.hl(),
            Operand::MemReg16(r) => self.state.regs.get16(r),
            Operand::MemHlInc => {
                let hl = self.state.regs.hl();
                self.state.regs.set_hl(hl.wrapping_add(1));
                hl
            }
            Operand::MemHlDec => {
                let hl = self.state.regs.hl();
                self.state.regs.set_hl(hl.wrapping_sub(1));
                hl
            }
            Operand::MemHighC => 0xff00 | u16::from(self.state.regs.c),
            Operand::MemHighImm8 => 0xff00 | u16::from(self.fetch()),
            Operand::MemImm16 => self.fetch16(),
            other => unreachable!("{other:?} is not a memory operand"),
        }
    }

    /// Read an 8-bit operand, charging whatever accesses it implies.
    fn read8(&mut self, operand: Operand) -> u8 {
        match operand {
            Operand::Reg(r) => self.state.regs.get8(r),
            // `Imm8` and `Rel8` differ only in how the *instruction* reads the
            // byte; both are one fetch from the stream.
            Operand::Imm8 | Operand::Rel8 => self.fetch(),
            _ => {
                let addr = self.address(operand);
                self.read(addr)
            }
        }
    }

    /// Write an 8-bit operand, charging whatever accesses it implies.
    fn write8(&mut self, operand: Operand, value: u8) {
        match operand {
            Operand::Reg(r) => self.state.regs.set8(r, value),
            _ => {
                let addr = self.address(operand);
                self.write(addr, value);
            }
        }
    }

    /// Read a 16-bit operand.
    fn read16(&mut self, operand: Operand) -> u16 {
        match operand {
            Operand::Reg16(r) => self.state.regs.get16(r),
            Operand::Imm16 => self.fetch16(),
            Operand::SpRel8 => {
                let e = self.fetch() as i8;
                let sp = self.state.regs.sp;
                // Pan Docs, *CPU Instruction Set*: the half-carry and carry come
                // from the *low byte* addition, and Z and N are cleared —
                // `SP+e8` is a byte-wise operation wearing a 16-bit result.
                let lo = u16::from(sp as u8) + u16::from(e as u8);
                let h = (sp & 0x0f) + (u16::from(e as u8) & 0x0f) > 0x0f;
                let c = lo > 0xff;
                self.set_flags(false, false, h, c);
                // One internal cycle: the adder is eight bits wide, so the high
                // half takes a second pass.
                self.idle();
                sp.wrapping_add(e as u16)
            }
            other => unreachable!("{other:?} is not a 16-bit source"),
        }
    }

    /// Write a 16-bit operand.
    fn write16(&mut self, operand: Operand, value: u16) {
        match operand {
            Operand::Reg16(r) => self.state.regs.set16(r, value),
            Operand::MemImm16 => {
                let addr = self.address(operand);
                self.write(addr, value as u8);
                self.write(addr.wrapping_add(1), (value >> 8) as u8);
            }
            other => unreachable!("{other:?} is not a 16-bit destination"),
        }
    }

    // -----------------------------------------------------------------
    // Instructions
    // -----------------------------------------------------------------

    /// Fetch, decode and execute one instruction.
    fn instruction(&mut self) {
        let opcode = self.fetch();
        let insn = decode(opcode);
        if insn.op == Op::PREFIX {
            let second = self.fetch();
            self.prefixed(decode_cb(second));
            return;
        }
        self.execute(insn);
    }

    #[allow(clippy::too_many_lines)]
    fn execute(&mut self, insn: Insn) {
        let Insn { op, dst, src, .. } = insn;
        match op {
            Op::NOP => {}

            Op::LD | Op::LDH => {
                if dst.is_wide() || src.is_wide() {
                    if dst == Operand::SP && src == Operand::HL {
                        // The only 16-bit register-to-register move, and the
                        // only one that costs a cycle: the value goes through
                        // the address latch (Gekkio, `LD SP,HL`).
                        let hl = self.state.regs.hl();
                        self.idle();
                        self.state.regs.sp = hl;
                    } else {
                        let value = self.read16(src);
                        self.write16(dst, value);
                    }
                } else {
                    let value = self.read8(src);
                    self.write8(dst, value);
                }
            }

            Op::INC | Op::DEC => {
                if dst.is_wide() {
                    let value = self.read16(dst);
                    // The 16-bit increment runs in the address unit, which needs
                    // a cycle of its own and touches no flags.
                    self.idle();
                    let result = if op == Op::INC {
                        value.wrapping_add(1)
                    } else {
                        value.wrapping_sub(1)
                    };
                    self.write16(dst, result);
                } else {
                    let value = self.read8(dst);
                    let result = if op == Op::INC {
                        let r = value.wrapping_add(1);
                        // C is untouched by INC/DEC — the one place the flag
                        // update is not wholesale.
                        self.set_flag(flags::Z, r == 0);
                        self.set_flag(flags::N, false);
                        self.set_flag(flags::H, value & 0x0f == 0x0f);
                        r
                    } else {
                        let r = value.wrapping_sub(1);
                        self.set_flag(flags::Z, r == 0);
                        self.set_flag(flags::N, true);
                        self.set_flag(flags::H, value & 0x0f == 0);
                        r
                    };
                    self.write8(dst, result);
                }
            }

            Op::ADD if dst == Operand::HL => {
                let value = self.read16(src);
                let hl = self.state.regs.hl();
                let result = hl.wrapping_add(value);
                // Z is untouched; H is the carry out of bit 11 and C out of
                // bit 15 (Pan Docs, *CPU Instruction Set*).
                self.set_flag(flags::N, false);
                self.set_flag(flags::H, (hl & 0x0fff) + (value & 0x0fff) > 0x0fff);
                self.set_flag(flags::C, u32::from(hl) + u32::from(value) > 0xffff);
                // The 16-bit adder is two passes over an 8-bit unit.
                self.idle();
                self.state.regs.set_hl(result);
            }

            Op::ADD if dst == Operand::SP => {
                let e = self.read8(src) as i8;
                let sp = self.state.regs.sp;
                let h = (sp & 0x0f) + (u16::from(e as u8) & 0x0f) > 0x0f;
                let c = (sp & 0xff) + (u16::from(e as u8) & 0xff) > 0xff;
                self.set_flags(false, false, h, c);
                // Two internal cycles rather than one: the result goes back to
                // `SP` through the address unit, which `LD HL,SP+e8` skips.
                self.idle();
                self.idle();
                self.state.regs.sp = sp.wrapping_add(e as u16);
            }

            Op::ADD | Op::ADC | Op::SUB | Op::SBC | Op::AND | Op::XOR | Op::OR | Op::CP => {
                let value = self.read8(src);
                self.alu(op, value);
            }

            Op::RLCA | Op::RRCA | Op::RLA | Op::RRA => {
                let a = self.state.regs.a;
                let carry = self.flag(flags::C);
                let (result, c) = match op {
                    Op::RLCA => (a.rotate_left(1), a & 0x80 != 0),
                    Op::RRCA => (a.rotate_right(1), a & 0x01 != 0),
                    Op::RLA => ((a << 1) | u8::from(carry), a & 0x80 != 0),
                    _ => ((a >> 1) | (u8::from(carry) << 7), a & 0x01 != 0),
                };
                // The accumulator forms clear Z unconditionally, unlike their
                // `$CB` twins which set it from the result. Emulators get this
                // backwards more often than any other flag rule on this chip.
                self.set_flags(false, false, false, c);
                self.state.regs.a = result;
            }

            Op::DAA => self.daa(),

            Op::CPL => {
                self.state.regs.a = !self.state.regs.a;
                self.set_flag(flags::N, true);
                self.set_flag(flags::H, true);
            }

            Op::SCF => {
                self.set_flag(flags::N, false);
                self.set_flag(flags::H, false);
                self.set_flag(flags::C, true);
            }

            Op::CCF => {
                let c = self.flag(flags::C);
                self.set_flag(flags::N, false);
                self.set_flag(flags::H, false);
                self.set_flag(flags::C, !c);
            }

            Op::JR => {
                let e = self.read8(Operand::Rel8) as i8;
                if self.branch_taken(dst) {
                    // The taken branch reloads the pipeline.
                    self.idle();
                    self.state.regs.pc = self.state.regs.pc.wrapping_add(e as u16);
                }
            }

            Op::JP => {
                if src == Operand::HL {
                    // `JP HL` is one cycle: the value is already in the address
                    // unit, so there is nothing to reload.
                    self.state.regs.pc = self.state.regs.hl();
                } else {
                    let target = self.fetch16();
                    if self.branch_taken(dst) {
                        self.idle();
                        self.state.regs.pc = target;
                    }
                }
            }

            Op::CALL => {
                let target = self.fetch16();
                if self.branch_taken(dst) {
                    // The predecrement of `SP` costs a cycle before either push.
                    self.idle();
                    let ret = self.state.regs.pc;
                    self.push16(ret);
                    self.state.regs.pc = target;
                }
            }

            Op::RET => {
                if let Operand::Cond(c) = dst {
                    // A conditional `RET` spends a cycle deciding, whether or
                    // not it then returns — which is why `RET cc` not taken is
                    // two cycles and `RET` is four.
                    self.idle();
                    if !self.cond(c) {
                        return;
                    }
                }
                let target = self.pop16();
                self.idle();
                self.state.regs.pc = target;
            }

            Op::RETI => {
                let target = self.pop16();
                self.idle();
                self.state.regs.pc = target;
                // Unlike `EI`, `RETI` enables interrupts immediately.
                self.state.ime = true;
                self.state.ei_pending = false;
            }

            Op::RST => {
                let Operand::Vector(v) = dst else {
                    unreachable!("decode fills in every RST vector");
                };
                self.idle();
                let ret = self.state.regs.pc;
                self.push16(ret);
                self.state.regs.pc = u16::from(v);
            }

            Op::PUSH => {
                let value = self.read16(dst);
                self.idle();
                self.push16(value);
            }

            Op::POP => {
                let value = self.pop16();
                self.write16(dst, value);
            }

            Op::DI => {
                self.state.ime = false;
                // Cancels an `EI` from the immediately preceding instruction,
                // which is the documented `EI; DI` behaviour.
                self.state.ei_pending = false;
            }

            Op::EI => self.state.ei_pending = true,

            Op::HALT => {
                if !self.state.ime && self.pending() != 0 {
                    // The HALT bug: the chip does not halt, and the *next* byte
                    // is fetched twice (Pan Docs, *Halt*).
                    self.state.halt_bug = true;
                } else {
                    self.state.mode = Mode::Halted;
                }
            }

            Op::STOP => {
                // `STOP` is two bytes and the second is ignored. What it does
                // beyond stopping the clock — resetting the divider, and the
                // several documented ways it can be entered wrongly — is the
                // divider's business and is not modelled here; see the module
                // documentation of `dev::gb::timer`.
                let _ = self.read8(src);
                self.state.mode = Mode::Stopped;
            }

            Op::LOCK => self.state.mode = Mode::Locked,

            // `$CB` never reaches here: `instruction` consumes it.
            Op::PREFIX => unreachable!("the prefix is handled before dispatch"),

            // Every `$CB` operation, reached only through `prefixed`.
            Op::RLC
            | Op::RRC
            | Op::RL
            | Op::RR
            | Op::SLA
            | Op::SRA
            | Op::SWAP
            | Op::SRL
            | Op::BIT
            | Op::RES
            | Op::SET => {
                unreachable!("{op:?} is a $CB-page operation")
            }
        }
    }

    /// Whether a branch with this destination operand is taken.
    ///
    /// `Operand::None` means unconditional, which is the only reason this is not
    /// simply [`Exec::cond`].
    fn branch_taken(&self, dst: Operand) -> bool {
        match dst {
            Operand::Cond(c) => self.cond(c),
            _ => true,
        }
    }

    /// The eight arithmetic and logical operations on the accumulator.
    fn alu(&mut self, op: Op, value: u8) {
        let a = self.state.regs.a;
        let carry = u8::from(self.flag(flags::C));
        match op {
            Op::ADD | Op::ADC => {
                let c = if op == Op::ADC { carry } else { 0 };
                let sum = u16::from(a) + u16::from(value) + u16::from(c);
                let result = sum as u8;
                self.set_flags(
                    result == 0,
                    false,
                    (a & 0x0f) + (value & 0x0f) + c > 0x0f,
                    sum > 0xff,
                );
                self.state.regs.a = result;
            }
            Op::SUB | Op::SBC | Op::CP => {
                let c = if op == Op::SBC { carry } else { 0 };
                let diff = i16::from(a) - i16::from(value) - i16::from(c);
                let result = diff as u8;
                self.set_flags(
                    result == 0,
                    true,
                    i16::from(a & 0x0f) - i16::from(value & 0x0f) - i16::from(c) < 0,
                    diff < 0,
                );
                // `CP` is `SUB` that throws the result away, which is the whole
                // difference between them.
                if op != Op::CP {
                    self.state.regs.a = result;
                }
            }
            Op::AND => {
                let result = a & value;
                // H set, and only here: the one logical operation that sets it.
                self.set_flags(result == 0, false, true, false);
                self.state.regs.a = result;
            }
            Op::XOR => {
                let result = a ^ value;
                self.set_flags(result == 0, false, false, false);
                self.state.regs.a = result;
            }
            Op::OR => {
                let result = a | value;
                self.set_flags(result == 0, false, false, false);
                self.state.regs.a = result;
            }
            other => unreachable!("{other:?} is not an ALU operation"),
        }
    }

    /// `DAA`, which is where the SM83 stops resembling a Z80.
    ///
    /// The chip has no BCD adder: `DAA` reads back the **N** flag left by the
    /// last add or subtract and corrects in the same direction. Pan Docs states
    /// the rule as: add `$06` when the half-carry says a nibble overflowed, add
    /// `$60` when the carry says the byte did, and after an addition also when
    /// the value is out of BCD range on its own; then subtract instead of adding
    /// if **N** is set. **H** is always cleared and **C** is only ever set,
    /// never cleared, because a decimal carry that has already happened cannot
    /// un-happen.
    fn daa(&mut self) {
        let n = self.flag(flags::N);
        let h = self.flag(flags::H);
        let mut carry = self.flag(flags::C);
        let a = self.state.regs.a;

        let mut adjust = 0u8;
        if h || (!n && a & 0x0f > 0x09) {
            adjust |= 0x06;
        }
        if carry || (!n && a > 0x99) {
            adjust |= 0x60;
            carry = true;
        }
        let result = if n {
            a.wrapping_sub(adjust)
        } else {
            a.wrapping_add(adjust)
        };
        self.state.regs.a = result;
        self.set_flag(flags::Z, result == 0);
        self.set_flag(flags::H, false);
        self.set_flag(flags::C, carry);
    }

    /// The `$CB` page: rotates, shifts, `SWAP`, and the bit operations.
    fn prefixed(&mut self, insn: Insn) {
        let carry = self.flag(flags::C);
        match insn.op {
            Op::BIT => {
                let Operand::Bit(bit) = insn.dst else {
                    unreachable!("BIT always carries its bit index");
                };
                let value = self.read8(insn.src);
                // C is untouched, H is set, and Z is the *complement* of the
                // bit. No write-back, which is why `BIT n,(HL)` is one cycle
                // shorter than `RES`/`SET`.
                self.set_flag(flags::Z, value & (1 << bit) == 0);
                self.set_flag(flags::N, false);
                self.set_flag(flags::H, true);
            }
            Op::RES | Op::SET => {
                let Operand::Bit(bit) = insn.dst else {
                    unreachable!("RES/SET always carry their bit index");
                };
                let value = self.read8(insn.src);
                let result = if insn.op == Op::SET {
                    value | (1 << bit)
                } else {
                    value & !(1 << bit)
                };
                self.write8(insn.src, result);
            }
            op => {
                let value = self.read8(insn.dst);
                let (result, c) = match op {
                    Op::RLC => (value.rotate_left(1), value & 0x80 != 0),
                    Op::RRC => (value.rotate_right(1), value & 0x01 != 0),
                    Op::RL => ((value << 1) | u8::from(carry), value & 0x80 != 0),
                    Op::RR => ((value >> 1) | (u8::from(carry) << 7), value & 0x01 != 0),
                    Op::SLA => (value << 1, value & 0x80 != 0),
                    // Arithmetic: bit 7 is duplicated rather than shifted in.
                    Op::SRA => ((value >> 1) | (value & 0x80), value & 0x01 != 0),
                    // The SM83's own addition, and the only nibble-swap on any
                    // 8080 descendant. It clears the carry.
                    Op::SWAP => (value.rotate_left(4), false),
                    Op::SRL => (value >> 1, value & 0x01 != 0),
                    other => unreachable!("{other:?} is not a $CB shift"),
                };
                // Unlike `RLCA` and friends, these set Z from the result.
                self.set_flags(result == 0, false, false, c);
                self.write8(insn.dst, result);
            }
        }
    }
}

impl Regs {
    /// Read one of the seven byte registers.
    #[inline]
    fn get8(&self, r: Reg8) -> u8 {
        match r {
            Reg8::B => self.b,
            Reg8::C => self.c,
            Reg8::D => self.d,
            Reg8::E => self.e,
            Reg8::H => self.h,
            Reg8::L => self.l,
            Reg8::A => self.a,
        }
    }

    /// Write one of the seven byte registers.
    #[inline]
    fn set8(&mut self, r: Reg8, value: u8) {
        match r {
            Reg8::B => self.b = value,
            Reg8::C => self.c = value,
            Reg8::D => self.d = value,
            Reg8::E => self.e = value,
            Reg8::H => self.h = value,
            Reg8::L => self.l = value,
            Reg8::A => self.a = value,
        }
    }

    /// Read one of the register pairs.
    #[inline]
    fn get16(&self, r: Reg16) -> u16 {
        match r {
            Reg16::Bc => (u16::from(self.b) << 8) | u16::from(self.c),
            Reg16::De => (u16::from(self.d) << 8) | u16::from(self.e),
            Reg16::Hl => self.hl(),
            Reg16::Sp => self.sp,
            Reg16::Af => (u16::from(self.a) << 8) | u16::from(self.f),
        }
    }

    /// Write one of the register pairs.
    #[inline]
    fn set16(&mut self, r: Reg16, value: u16) {
        let (hi, lo) = ((value >> 8) as u8, value as u8);
        match r {
            Reg16::Bc => {
                self.b = hi;
                self.c = lo;
            }
            Reg16::De => {
                self.d = hi;
                self.e = lo;
            }
            Reg16::Hl => self.set_hl(value),
            Reg16::Sp => self.sp = value,
            Reg16::Af => {
                self.a = hi;
                // `POP AF` is the only way to write `F` wholesale, and the low
                // nibble has no storage: it reads back as zero however hard the
                // program pushes.
                self.f = lo & 0xf0;
            }
        }
    }
}
