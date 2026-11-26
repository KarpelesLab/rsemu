//! The cycle-accurate interpreter.
//!
//! # One cycle is one bus access, or one documented internal cycle
//!
//! Every SPC700 cycle is a read, a write, or an internal operation, and this
//! interpreter has no cycle counter it bumps independently of what it does:
//! [`Bus::read`], [`Bus::write`] and [`Bus::idle`] *are* the clock. A bus that
//! counts calls gets the cycle count for free, and a bus that clocks timers
//! and a DSP on every call — the S-SMP's — gets them advanced at the right
//! sub-instruction cycle.
//!
//! The sequences below are the ones `SingleStepTests/spc700` records,
//! including the reads whose data the processor throws away:
//!
//! * a one-byte instruction reads the byte after its opcode and discards it
//!   ([`Exec::dummy`]) — `NOP`, `INC A`, `MOV A,X`, every `PUSH`/`POP`;
//! * a store reads its target before writing it (`MOV dp,A` is fetch, fetch,
//!   read `dp`, write `dp`), except `MOV dp,dp` and `MOV (X)+,A`;
//! * `TSET1`/`TCLR1` read their target twice.
//!
//! A real dummy read is not cosmetic on this machine: reading `$FD`–`$FF`
//! clears a timer counter, so an emulator that skips the read that
//! `MOV $FD,A` performs is a timer that never resets.
//!
//! # Sources
//!
//! The SNESdev wiki's *SPC-700 instruction set* for semantics and flags;
//! nocash *fullsnes* (*SNES APU SPC700 CPU*) for `DAA`/`DAS`, `MUL`, and the
//! documented `DIV` overflow result; anomie's *SPC700 doc* for the same
//! overflow rule; and the `SingleStepTests/spc700` traces (MIT) for the
//! position of every dummy read and internal cycle. No emulator source was
//! consulted.

use super::isa::{Insn, Op, Operand, decode};
use super::{Regs, flags};

/// What the processor's pins are connected to.
///
/// **Every call is one processor cycle.** That is the whole contract, and it
/// is what lets one interpreter serve two very different buses: the
/// standalone `cpu.spc700` device forwards to an
/// [`AddressSpace`](crate::core::space::AddressSpace), and the S-SMP
/// (`dev::snes::apu`) decodes its own sixty-four kilobytes and clocks its
/// timers and its DSP on every call.
pub trait Bus {
    /// One read cycle at `addr`.
    fn read(&mut self, addr: u16) -> u8;
    /// One write cycle.
    fn write(&mut self, addr: u16, value: u8);
    /// One internal cycle: no address the traces can see.
    fn idle(&mut self);
}

/// What the core is doing between instructions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    /// The reset sequence has not run yet: the next step loads `PC` from
    /// `$FFFE`.
    #[default]
    Reset,
    /// Fetching and executing.
    Running,
    /// `SLEEP`: waiting for an interrupt the SNES never delivers.
    Sleeping,
    /// `STOP`: stopped until reset.
    Stopped,
}

impl Mode {
    /// The tag this mode is snapshotted as.
    #[must_use]
    pub const fn tag(self) -> u8 {
        match self {
            Mode::Reset => 0,
            Mode::Running => 1,
            Mode::Sleeping => 2,
            Mode::Stopped => 3,
        }
    }

    /// The mode a snapshot tag names.
    #[must_use]
    pub const fn from_tag(tag: u8) -> Option<Mode> {
        match tag {
            0 => Some(Mode::Reset),
            1 => Some(Mode::Running),
            2 => Some(Mode::Sleeping),
            3 => Some(Mode::Stopped),
            _ => None,
        }
    }
}

/// The architectural state of one SPC700, independent of what it is wired to.
///
/// Public so that a chip which *contains* an SPC700 — the S-SMP — can own one
/// and step it over its own [`Bus`] without an address space in between.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Core {
    /// The register file.
    pub regs: Regs,
    /// Run state.
    pub mode: Mode,
    /// Cycles executed since power-on.
    pub cycles: u64,
}

impl Core {
    /// A core at power-on: registers zero, reset sequence pending.
    #[must_use]
    pub const fn new() -> Core {
        Core {
            regs: Regs::new(),
            mode: Mode::Reset,
            cycles: 0,
        }
    }

    /// Execute one instruction (or one reset sequence, or one halted cycle
    /// pair) and return the cycles it took. Never zero.
    pub fn step<B: Bus + ?Sized>(&mut self, bus: &mut B) -> u64 {
        let mut e = Exec {
            r: &mut self.regs,
            bus,
            used: 0,
            mode: self.mode,
        };
        match e.mode {
            Mode::Reset => {
                // The reset vector. How many cycles the chip spends before the
                // first fetch is not documented in any source used here; the
                // two vector reads are what is certain.
                let lo = e.read(0xfffe);
                let hi = e.read(0xffff);
                e.r.pc = u16::from_le_bytes([lo, hi]);
                e.mode = Mode::Running;
            }
            Mode::Running => e.execute(),
            Mode::Sleeping | Mode::Stopped => {
                // What the SingleStepTests traces show a halted SPC700 doing:
                // reading the byte after the opcode, then an internal cycle,
                // forever.
                e.dummy();
                e.idle();
            }
        }
        let used = e.used;
        self.mode = e.mode;
        self.cycles = self.cycles.wrapping_add(used);
        used
    }
}

/// One instruction's worth of execution.
struct Exec<'a, B: Bus + ?Sized> {
    r: &'a mut Regs,
    bus: &'a mut B,
    used: u64,
    mode: Mode,
}

/// Which byte register an operand names, for the generic paths.
#[derive(Clone, Copy)]
enum R8 {
    A,
    X,
    Y,
    Sp,
    Psw,
}

impl<B: Bus + ?Sized> Exec<'_, B> {
    // -- the clock -----------------------------------------------------------

    #[inline]
    fn read(&mut self, addr: u16) -> u8 {
        self.used += 1;
        self.bus.read(addr)
    }

    #[inline]
    fn write(&mut self, addr: u16, value: u8) {
        self.used += 1;
        self.bus.write(addr, value);
    }

    #[inline]
    fn idle(&mut self) {
        self.used += 1;
        self.bus.idle();
    }

    /// Read the byte at `PC` and advance past it.
    #[inline]
    fn fetch(&mut self) -> u8 {
        let pc = self.r.pc;
        self.r.pc = pc.wrapping_add(1);
        self.read(pc)
    }

    fn fetch16(&mut self) -> u16 {
        let lo = self.fetch();
        let hi = self.fetch();
        u16::from_le_bytes([lo, hi])
    }

    /// Read the byte at `PC` and throw it away, `PC` unchanged: the second
    /// cycle of every one-byte instruction.
    fn dummy(&mut self) {
        let pc = self.r.pc;
        self.read(pc);
    }

    // -- addressing ----------------------------------------------------------

    /// A direct-page address: `$00xx` with `P` clear, `$01xx` with it set.
    #[inline]
    fn dp(&self, offset: u8) -> u16 {
        let page = if self.r.psw & flags::P != 0 { 0x100 } else { 0 };
        page | u16::from(offset)
    }

    fn push(&mut self, value: u8) {
        let sp = self.r.sp;
        self.write(0x100 | u16::from(sp), value);
        self.r.sp = sp.wrapping_sub(1);
    }

    fn pop(&mut self) -> u8 {
        self.r.sp = self.r.sp.wrapping_add(1);
        let sp = self.r.sp;
        self.read(0x100 | u16::from(sp))
    }

    fn reg(&self, r: R8) -> u8 {
        match r {
            R8::A => self.r.a,
            R8::X => self.r.x,
            R8::Y => self.r.y,
            R8::Sp => self.r.sp,
            R8::Psw => self.r.psw,
        }
    }

    fn set_reg(&mut self, r: R8, v: u8) {
        match r {
            R8::A => self.r.a = v,
            R8::X => self.r.x = v,
            R8::Y => self.r.y = v,
            R8::Sp => self.r.sp = v,
            R8::Psw => self.r.psw = v,
        }
    }

    /// The register an operand names, if it names one.
    fn r8(op: Operand) -> Option<R8> {
        match op {
            Operand::A => Some(R8::A),
            Operand::X => Some(R8::X),
            Operand::Y => Some(R8::Y),
            Operand::Sp => Some(R8::Sp),
            Operand::Psw => Some(R8::Psw),
            _ => None,
        }
    }

    /// The address a memory operand resolves to, performing exactly the
    /// fetches and internal cycles its *store/read-modify-write* form takes.
    fn address(&mut self, op: Operand, for_store: bool) -> u16 {
        match op {
            Operand::Dp => {
                let d = self.fetch();
                self.dp(d)
            }
            Operand::DpX | Operand::DpY => {
                let d = self.fetch();
                self.idle();
                let i = if op == Operand::DpX {
                    self.r.x
                } else {
                    self.r.y
                };
                self.dp(d.wrapping_add(i))
            }
            Operand::Abs => self.fetch16(),
            Operand::AbsX | Operand::AbsY => {
                let a = self.fetch16();
                self.idle();
                let i = if op == Operand::AbsX {
                    self.r.x
                } else {
                    self.r.y
                };
                // Guest arithmetic wraps: sixteen-bit address space.
                a.wrapping_add(u16::from(i))
            }
            Operand::IndX => {
                self.dummy();
                self.dp(self.r.x)
            }
            Operand::IndY => self.dp(self.r.y),
            Operand::DpIndX => {
                let d = self.fetch();
                self.idle();
                let base = d.wrapping_add(self.r.x);
                let lo = self.read(self.dp(base));
                let hi = self.read(self.dp(base.wrapping_add(1)));
                u16::from_le_bytes([lo, hi])
            }
            Operand::DpIndY => {
                let d = self.fetch();
                // The read and the store forms put their internal cycle in
                // different places — before the pointer for a read, after it
                // for a store — and the traces are unambiguous about both.
                if !for_store {
                    self.idle();
                }
                let lo = self.read(self.dp(d));
                let hi = self.read(self.dp(d.wrapping_add(1)));
                if for_store {
                    self.idle();
                }
                u16::from_le_bytes([lo, hi]).wrapping_add(u16::from(self.r.y))
            }
            // Not a memory operand; the table never routes one here.
            _ => 0,
        }
    }

    /// A read-only operand's value, with that addressing mode's bus sequence.
    fn load(&mut self, op: Operand) -> u8 {
        match op {
            Operand::Imm => self.fetch(),
            Operand::IndXInc => {
                self.dummy();
                let v = self.read(self.dp(self.r.x));
                self.idle();
                self.r.x = self.r.x.wrapping_add(1);
                v
            }
            _ => match Self::r8(op) {
                Some(r) => self.reg(r),
                None => {
                    let a = self.address(op, false);
                    self.read(a)
                }
            },
        }
    }

    // -- flags ---------------------------------------------------------------

    fn set_flag(&mut self, mask: u8, on: bool) {
        if on {
            self.r.psw |= mask;
        } else {
            self.r.psw &= !mask;
        }
    }

    fn flag(&self, mask: u8) -> bool {
        self.r.psw & mask != 0
    }

    fn nz(&mut self, v: u8) {
        self.set_flag(flags::N, v & 0x80 != 0);
        self.set_flag(flags::Z, v == 0);
    }

    fn nz16(&mut self, v: u16) {
        self.set_flag(flags::N, v & 0x8000 != 0);
        self.set_flag(flags::Z, v == 0);
    }

    fn adc(&mut self, a: u8, b: u8) -> u8 {
        let c = u16::from(self.flag(flags::C));
        let sum = u16::from(a) + u16::from(b) + c;
        let r = sum as u8;
        self.set_flag(flags::V, (!(a ^ b) & (a ^ r)) & 0x80 != 0);
        self.set_flag(flags::H, (a ^ b ^ r) & 0x10 != 0);
        self.set_flag(flags::C, sum > 0xff);
        self.nz(r);
        r
    }

    /// Subtract with borrow, as the hardware does it: add the complement.
    fn sbc(&mut self, a: u8, b: u8) -> u8 {
        self.adc(a, !b)
    }

    fn cmp(&mut self, a: u8, b: u8) {
        let r = a.wrapping_sub(b);
        self.set_flag(flags::C, a >= b);
        self.nz(r);
    }

    /// The two-operand ALU. `None` for `CMP`, which writes nothing back.
    fn alu(&mut self, op: Op, a: u8, b: u8) -> Option<u8> {
        let r = match op {
            Op::OR => a | b,
            Op::AND => a & b,
            Op::EOR => a ^ b,
            Op::ADC => return Some(self.adc(a, b)),
            Op::SBC => return Some(self.sbc(a, b)),
            _ => {
                self.cmp(a, b);
                return None;
            }
        };
        self.nz(r);
        Some(r)
    }

    /// The one-operand shifts, rotates and steps.
    fn rmw(&mut self, op: Op, v: u8) -> u8 {
        let c = self.flag(flags::C);
        let r = match op {
            Op::ASL => {
                self.set_flag(flags::C, v & 0x80 != 0);
                v << 1
            }
            Op::ROL => {
                self.set_flag(flags::C, v & 0x80 != 0);
                (v << 1) | u8::from(c)
            }
            Op::LSR => {
                self.set_flag(flags::C, v & 1 != 0);
                v >> 1
            }
            Op::ROR => {
                self.set_flag(flags::C, v & 1 != 0);
                (v >> 1) | (u8::from(c) << 7)
            }
            Op::INC => v.wrapping_add(1),
            _ => v.wrapping_sub(1),
        };
        self.nz(r);
        r
    }

    fn branch(&mut self, taken: bool) {
        let rel = self.fetch();
        if taken {
            // Two internal cycles to reload PC, from the traces.
            self.idle();
            self.idle();
            self.r.pc = self.r.pc.wrapping_add(rel as i8 as u16);
        }
    }

    fn ya(&self) -> u16 {
        u16::from_le_bytes([self.r.a, self.r.y])
    }

    fn set_ya(&mut self, v: u16) {
        let [a, y] = v.to_le_bytes();
        self.r.a = a;
        self.r.y = y;
    }

    // -- execution -----------------------------------------------------------

    fn execute(&mut self) {
        let opcode = self.fetch();
        let insn = decode(opcode);
        match insn.op {
            Op::NOP => self.dummy(),
            Op::OR | Op::AND | Op::EOR | Op::ADC | Op::SBC | Op::CMP => self.alu_insn(insn),
            Op::MOV => self.mov(insn),
            Op::ASL | Op::ROL | Op::LSR | Op::ROR | Op::INC | Op::DEC => {
                if let Some(r) = Self::r8(insn.dst) {
                    self.dummy();
                    let v = self.reg(r);
                    let res = self.rmw(insn.op, v);
                    self.set_reg(r, res);
                } else {
                    let a = self.address(insn.dst, true);
                    let v = self.read(a);
                    let res = self.rmw(insn.op, v);
                    self.write(a, res);
                }
            }

            // -- branches -----------------------------------------------------
            Op::BRA => self.branch(true),
            Op::BEQ => self.branch(self.flag(flags::Z)),
            Op::BNE => self.branch(!self.flag(flags::Z)),
            Op::BCS => self.branch(self.flag(flags::C)),
            Op::BCC => self.branch(!self.flag(flags::C)),
            Op::BVS => self.branch(self.flag(flags::V)),
            Op::BVC => self.branch(!self.flag(flags::V)),
            Op::BMI => self.branch(self.flag(flags::N)),
            Op::BPL => self.branch(!self.flag(flags::N)),
            Op::BBS | Op::BBC => {
                let Operand::DpBit(bit) = insn.dst else {
                    return;
                };
                let d = self.fetch();
                let v = self.read(self.dp(d));
                self.idle();
                let set = v & (1 << bit) != 0;
                self.branch(set == (insn.op == Op::BBS));
            }
            Op::CBNE => {
                let a = if insn.dst == Operand::DpX {
                    let d = self.fetch();
                    self.idle();
                    self.dp(d.wrapping_add(self.r.x))
                } else {
                    let d = self.fetch();
                    self.dp(d)
                };
                let v = self.read(a);
                self.idle();
                let ne = self.r.a != v;
                self.branch(ne);
            }
            Op::DBNZ => {
                if insn.dst == Operand::Y {
                    self.dummy();
                    self.idle();
                    self.r.y = self.r.y.wrapping_sub(1);
                    let nz = self.r.y != 0;
                    self.branch(nz);
                } else {
                    let d = self.fetch();
                    let a = self.dp(d);
                    let v = self.read(a).wrapping_sub(1);
                    self.write(a, v);
                    self.branch(v != 0);
                }
            }

            // -- bits ---------------------------------------------------------
            Op::SET1 | Op::CLR1 => {
                let Operand::DpBit(bit) = insn.dst else {
                    return;
                };
                let d = self.fetch();
                let a = self.dp(d);
                let v = self.read(a);
                let r = if insn.op == Op::SET1 {
                    v | (1 << bit)
                } else {
                    v & !(1 << bit)
                };
                self.write(a, r);
            }
            Op::TSET1 | Op::TCLR1 => {
                let a = self.fetch16();
                let v = self.read(a);
                self.read(a);
                // Flags as `CMP A,m` would set N and Z; carry untouched.
                let diff = self.r.a.wrapping_sub(v);
                self.nz(diff);
                let r = if insn.op == Op::TSET1 {
                    v | self.r.a
                } else {
                    v & !self.r.a
                };
                self.write(a, r);
            }
            Op::OR1 | Op::AND1 | Op::EOR1 | Op::MOV1 | Op::NOT1 => self.bit_insn(insn),

            // -- words --------------------------------------------------------
            Op::MOVW => {
                let d = self.fetch();
                let lo_a = self.dp(d);
                let hi_a = self.dp(d.wrapping_add(1));
                if insn.dst == Operand::Ya {
                    let lo = self.read(lo_a);
                    self.idle();
                    let hi = self.read(hi_a);
                    let v = u16::from_le_bytes([lo, hi]);
                    self.set_ya(v);
                    self.nz16(v);
                } else {
                    self.read(lo_a);
                    self.write(lo_a, self.r.a);
                    self.write(hi_a, self.r.y);
                }
            }
            Op::INCW | Op::DECW => {
                let d = self.fetch();
                let lo_a = self.dp(d);
                let hi_a = self.dp(d.wrapping_add(1));
                let lo = self.read(lo_a);
                let step: u16 = if insn.op == Op::INCW { 1 } else { 0xffff };
                // The low byte is written back before the high byte is read,
                // which is why the word is computed in two halves.
                let low_result = u16::from(lo).wrapping_add(step);
                self.write(lo_a, low_result as u8);
                let hi = self.read(hi_a);
                let v = u16::from_le_bytes([lo, hi]).wrapping_add(step);
                self.write(hi_a, (v >> 8) as u8);
                self.nz16(v);
            }
            Op::CMPW | Op::ADDW | Op::SUBW => {
                let d = self.fetch();
                let lo = self.read(self.dp(d));
                if insn.op != Op::CMPW {
                    self.idle();
                }
                let hi = self.read(self.dp(d.wrapping_add(1)));
                let w = u16::from_le_bytes([lo, hi]);
                let ya = self.ya();
                match insn.op {
                    Op::CMPW => {
                        self.set_flag(flags::C, ya >= w);
                        self.nz16(ya.wrapping_sub(w));
                    }
                    Op::ADDW => {
                        // Byte-wise, carry clear into the low half: H and V
                        // then come out of the high byte, i.e. bits 11 and 15.
                        self.set_flag(flags::C, false);
                        let l = self.adc(self.r.a, lo);
                        let h = self.adc(self.r.y, hi);
                        let v = u16::from_le_bytes([l, h]);
                        self.set_ya(v);
                        self.nz16(v);
                    }
                    _ => {
                        self.set_flag(flags::C, true);
                        let l = self.sbc(self.r.a, lo);
                        let h = self.sbc(self.r.y, hi);
                        let v = u16::from_le_bytes([l, h]);
                        self.set_ya(v);
                        self.nz16(v);
                    }
                }
            }
            Op::MUL => {
                self.dummy();
                for _ in 0..7 {
                    self.idle();
                }
                let v = u16::from(self.r.y) * u16::from(self.r.a);
                self.set_ya(v);
                // N and Z come from Y — the high byte — alone (fullsnes).
                self.nz(self.r.y);
            }
            Op::DIV => {
                self.dummy();
                for _ in 0..10 {
                    self.idle();
                }
                self.div();
            }

            // -- decimal and nibbles -----------------------------------------
            Op::DAA => {
                self.dummy();
                self.idle();
                let mut a = self.r.a;
                if self.flag(flags::C) || a > 0x99 {
                    a = a.wrapping_add(0x60);
                    self.set_flag(flags::C, true);
                }
                if self.flag(flags::H) || (a & 0x0f) > 9 {
                    a = a.wrapping_add(6);
                }
                self.r.a = a;
                self.nz(a);
            }
            Op::DAS => {
                self.dummy();
                self.idle();
                let mut a = self.r.a;
                if !self.flag(flags::C) || a > 0x99 {
                    a = a.wrapping_sub(0x60);
                    self.set_flag(flags::C, false);
                }
                if !self.flag(flags::H) || (a & 0x0f) > 9 {
                    a = a.wrapping_sub(6);
                }
                self.r.a = a;
                self.nz(a);
            }
            Op::XCN => {
                self.dummy();
                self.idle();
                self.idle();
                self.idle();
                self.r.a = self.r.a.rotate_left(4);
                self.nz(self.r.a);
            }

            // -- stack and control flow ----------------------------------------
            Op::PUSH => {
                self.dummy();
                let v = Self::r8(insn.dst).map_or(0, |r| self.reg(r));
                self.push(v);
                self.idle();
            }
            Op::POP => {
                self.dummy();
                self.idle();
                let v = self.pop();
                if let Some(r) = Self::r8(insn.dst) {
                    self.set_reg(r, v);
                }
            }
            Op::JMP => {
                if insn.dst == Operand::AbsIndX {
                    let a = self.fetch16().wrapping_add(u16::from(self.r.x));
                    self.idle();
                    let lo = self.read(a);
                    let hi = self.read(a.wrapping_add(1));
                    self.r.pc = u16::from_le_bytes([lo, hi]);
                } else {
                    self.r.pc = self.fetch16();
                }
            }
            Op::CALL => {
                let target = self.fetch16();
                self.idle();
                self.push_pc();
                self.idle();
                self.idle();
                self.r.pc = target;
            }
            Op::PCALL => {
                let u = self.fetch();
                self.idle();
                self.push_pc();
                self.idle();
                self.r.pc = 0xff00 | u16::from(u);
            }
            Op::TCALL => {
                let Operand::Vector(n) = insn.dst else {
                    return;
                };
                self.dummy();
                self.idle();
                self.push_pc();
                self.idle();
                let v = 0xffde - 2 * u16::from(n);
                let lo = self.read(v);
                let hi = self.read(v + 1);
                self.r.pc = u16::from_le_bytes([lo, hi]);
            }
            Op::BRK => {
                self.dummy();
                self.push_pc();
                self.push(self.r.psw);
                self.idle();
                let lo = self.read(0xffde);
                let hi = self.read(0xffdf);
                self.r.pc = u16::from_le_bytes([lo, hi]);
                self.set_flag(flags::B, true);
                self.set_flag(flags::I, false);
            }
            Op::RET => {
                self.dummy();
                self.idle();
                let lo = self.pop();
                let hi = self.pop();
                self.r.pc = u16::from_le_bytes([lo, hi]);
            }
            Op::RETI => {
                self.dummy();
                self.idle();
                self.r.psw = self.pop();
                let lo = self.pop();
                let hi = self.pop();
                self.r.pc = u16::from_le_bytes([lo, hi]);
            }

            // -- flags --------------------------------------------------------
            // `CLRC`/`SETC` are two cycles; `NOTC`, `EI` and `DI` add an
            // internal one (the traces, and fullsnes's cycle table).
            Op::CLRC => {
                self.dummy();
                self.set_flag(flags::C, false);
            }
            Op::SETC => {
                self.dummy();
                self.set_flag(flags::C, true);
            }
            Op::NOTC | Op::EI | Op::DI => {
                self.dummy();
                self.idle();
                match insn.op {
                    Op::NOTC => self.r.psw ^= flags::C,
                    Op::EI => self.set_flag(flags::I, true),
                    _ => self.set_flag(flags::I, false),
                }
            }
            Op::CLRP => {
                self.dummy();
                self.set_flag(flags::P, false);
            }
            Op::SETP => {
                self.dummy();
                self.set_flag(flags::P, true);
            }
            Op::CLRV => {
                self.dummy();
                self.set_flag(flags::V, false);
                self.set_flag(flags::H, false);
            }
            Op::SLEEP | Op::STOP => {
                self.dummy();
                self.idle();
                self.mode = if insn.op == Op::SLEEP {
                    Mode::Sleeping
                } else {
                    Mode::Stopped
                };
            }
            // `Op` is non-exhaustive for downstream crates only; every variant
            // is handled above.
            #[allow(unreachable_patterns)]
            _ => self.dummy(),
        }
    }

    fn push_pc(&mut self) {
        let [lo, hi] = self.r.pc.to_le_bytes();
        self.push(hi);
        self.push(lo);
    }

    fn alu_insn(&mut self, insn: Insn) {
        if let Some(r) = Self::r8(insn.dst) {
            let b = self.load(insn.src);
            let a = self.reg(r);
            if let Some(v) = self.alu(insn.op, a, b) {
                self.set_reg(r, v);
            }
            return;
        }
        // Memory destination: `dp,#imm`, `dp,dp` or `(X),(Y)`.
        let (addr, b) = match (insn.dst, insn.src) {
            (Operand::Dp, Operand::Imm) => {
                let b = self.fetch();
                let d = self.fetch();
                (self.dp(d), b)
            }
            (Operand::Dp, _) => {
                let b = self.load(Operand::Dp);
                let d = self.fetch();
                (self.dp(d), b)
            }
            _ => {
                self.dummy();
                let b = self.read(self.dp(self.r.y));
                (self.dp(self.r.x), b)
            }
        };
        let a = self.read(addr);
        match self.alu(insn.op, a, b) {
            Some(v) => self.write(addr, v),
            None => self.idle(),
        }
    }

    fn mov(&mut self, insn: Insn) {
        match (Self::r8(insn.dst), Self::r8(insn.src)) {
            (Some(d), Some(s)) => {
                self.dummy();
                let v = self.reg(s);
                self.set_reg(d, v);
                // `MOV SP,X` is the one register move that sets no flags.
                if !matches!(d, R8::Sp) {
                    self.nz(v);
                }
            }
            (Some(d), None) => {
                let v = self.load(insn.src);
                self.set_reg(d, v);
                self.nz(v);
            }
            (None, Some(s)) => {
                let v = self.reg(s);
                if insn.dst == Operand::IndXInc {
                    self.dummy();
                    self.idle();
                    let a = self.dp(self.r.x);
                    self.write(a, v);
                    self.r.x = self.r.x.wrapping_add(1);
                } else {
                    let a = self.address(insn.dst, true);
                    self.read(a);
                    self.write(a, v);
                }
            }
            (None, None) => {
                if insn.src == Operand::Imm {
                    // `MOV dp,#imm`: reads its target before writing it.
                    let v = self.fetch();
                    let d = self.fetch();
                    let a = self.dp(d);
                    self.read(a);
                    self.write(a, v);
                } else {
                    // `MOV dp,dp`: the one store with no read of its target.
                    let v = self.load(Operand::Dp);
                    let d = self.fetch();
                    let a = self.dp(d);
                    self.write(a, v);
                }
            }
        }
    }

    fn bit_insn(&mut self, insn: Insn) {
        let w = self.fetch16();
        let addr = w & 0x1fff;
        let bit = (w >> 13) as u8;
        let v = self.read(addr);
        let set = v & (1 << bit) != 0;
        let c = self.flag(flags::C);
        match (insn.op, insn.dst, insn.src) {
            (Op::OR1, _, src) => {
                let b = if src == Operand::MemBitNot { !set } else { set };
                self.idle();
                self.set_flag(flags::C, c | b);
            }
            (Op::AND1, _, src) => {
                let b = if src == Operand::MemBitNot { !set } else { set };
                self.set_flag(flags::C, c & b);
            }
            (Op::EOR1, _, _) => {
                self.idle();
                self.set_flag(flags::C, c ^ set);
            }
            (Op::MOV1, Operand::C, _) => self.set_flag(flags::C, set),
            (Op::MOV1, _, _) => {
                self.idle();
                let r = if c { v | (1 << bit) } else { v & !(1 << bit) };
                self.write(addr, r);
            }
            _ => self.write(addr, v ^ (1 << bit)),
        }
    }

    /// `DIV YA,X`.
    ///
    /// When the quotient fits in nine bits the result is the ordinary one.
    /// When it does not, the chip's shift-and-subtract divider produces a
    /// specific, documented wrong answer, and software that divides by a small
    /// `X` sees it — fullsnes and anomie both give the closed form used here.
    fn div(&mut self) {
        let ya = u32::from(self.ya());
        let x = u32::from(self.r.x);
        let y = u32::from(self.r.y);
        self.set_flag(flags::V, y >= x);
        self.set_flag(flags::H, (y & 0x0f) >= (x & 0x0f));
        let (q, r) = if y < (x << 1) {
            // `y < 2x` implies `x > 0`, so this divides by a non-zero X.
            (ya / x, ya % x)
        } else {
            let rest = ya - (x << 9);
            let d = 256 - x;
            (255 - rest / d, x + rest % d)
        };
        self.r.a = q as u8;
        self.r.y = r as u8;
        self.nz(self.r.a);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Flat 64 KiB, counting nothing — `Core::step`'s return is the count.
    struct Flat([u8; 0x10000]);

    impl Bus for Flat {
        fn read(&mut self, addr: u16) -> u8 {
            self.0[addr as usize]
        }
        fn write(&mut self, addr: u16, value: u8) {
            self.0[addr as usize] = value;
        }
        fn idle(&mut self) {}
    }

    fn run(code: &[u8], setup: impl FnOnce(&mut Regs)) -> (Core, alloc::boxed::Box<Flat>, u64) {
        let mut bus = alloc::boxed::Box::new(Flat([0; 0x10000]));
        bus.0[0x200..0x200 + code.len()].copy_from_slice(code);
        let mut core = Core::new();
        core.mode = Mode::Running;
        core.regs.pc = 0x200;
        setup(&mut core.regs);
        let n = core.step(&mut *bus);
        (core, bus, n)
    }

    #[test]
    fn div_overflow_follows_the_documented_closed_form() {
        // $FFFF / 1: the quotient does not fit and the hardware says so.
        let (core, _, n) = run(&[0x9e], |r| {
            r.a = 0xff;
            r.y = 0xff;
            r.x = 0x01;
        });
        assert_eq!(n, 12);
        assert!(core.regs.psw & flags::V != 0);
        // Ordinary case: 1000 / 7 = 142 r 6.
        let (core, _, _) = run(&[0x9e], |r| {
            r.a = 0xe8;
            r.y = 0x03;
            r.x = 7;
        });
        assert_eq!((core.regs.a, core.regs.y), (142, 6));
    }

    #[test]
    fn mul_takes_nine_cycles_and_flags_from_y() {
        let (core, _, n) = run(&[0xcf], |r| {
            r.a = 0x10;
            r.y = 0x10;
        });
        assert_eq!(n, 9);
        assert_eq!(core.regs.y, 0x01);
        assert_eq!(core.regs.a, 0x00);
        assert_eq!(core.regs.psw & flags::Z, 0, "Z comes from Y, which is 1");
    }

    #[test]
    fn the_direct_page_follows_p() {
        let (_, bus, _) = run(&[0xc4, 0x10], |r| {
            r.a = 0x5a;
            r.psw = flags::P;
        });
        assert_eq!(bus.0[0x110], 0x5a);
        assert_eq!(bus.0[0x010], 0x00);
    }

    #[test]
    fn sleep_halts_and_keeps_the_clock_running() {
        let mut bus = alloc::boxed::Box::new(Flat([0; 0x10000]));
        bus.0[0x200] = 0xef;
        let mut core = Core::new();
        core.mode = Mode::Running;
        core.regs.pc = 0x200;
        assert_eq!(core.step(&mut *bus), 3);
        assert_eq!(core.mode, Mode::Sleeping);
        assert_eq!(core.step(&mut *bus), 2);
        assert_eq!(core.regs.pc, 0x201);
    }

    #[test]
    fn reset_loads_the_vector() {
        let mut bus = alloc::boxed::Box::new(Flat([0; 0x10000]));
        bus.0[0xfffe] = 0xc0;
        bus.0[0xffff] = 0xff;
        let mut core = Core::new();
        core.step(&mut *bus);
        assert_eq!(core.regs.pc, 0xffc0);
        assert_eq!(core.mode, Mode::Running);
    }
}
