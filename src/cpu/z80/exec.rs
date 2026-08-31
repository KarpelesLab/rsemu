//! The cycle-accurate interpreter.
//!
//! # One M-cycle is one bus access
//!
//! Unlike a 6502, a Z80 clock is not always a bus access: an M-cycle is three,
//! four or five T-states, and some M-cycles request nothing at all. So the
//! rule here is one level up from the 6502's — **every T-state is charged
//! because an M-cycle happened**, and the M-cycle's length is a property of
//! what it does, never of a per-opcode table:
//!
//! | M-cycle | T-states | Bus |
//! | --- | --- | --- |
//! | opcode fetch (`M1`) | 4 | read at PC, then the refresh address |
//! | memory read | 3 | read |
//! | memory write | 3 | write |
//! | I/O read or write | 4 | one wait state, which is what the extra T is |
//! | internal | 1 each | nothing; the address pins hold the last value |
//!
//! `INC (HL)` is therefore 4 + 3 + 1 + 3 = 11 T-states *because* it fetches,
//! reads, thinks for one T-state and writes — not because a table says 11.
//! Every count in this file falls out of that, and [`CycleLog`] records the
//! sequence so a conformance run can compare it against hardware.
//!
//! # The internal registers this core models from the start
//!
//! Three pieces of Z80 state are invisible in the programming model and
//! observable in the flags, so retrofitting them is a rewrite:
//!
//! - **`WZ`** (`MEMPTR`), the internal address latch. `BIT n,(HL)` takes its
//!   undocumented flag bits from `W`, so a core without it fails every serious
//!   test suite.
//! - **`Q`**, the flag value the last flag-*writing* instruction produced, or
//!   zero if the last instruction wrote no flags. `SCF` and `CCF` compute
//!   their undocumented bits from `((Q ^ F) | A)`, which is how "the previous
//!   instruction did not touch the flags" becomes visible.
//! - **the `LD A,I` latch**, which makes the parity flag those two
//!   instructions copy from `IFF2` come out clear when an interrupt lands
//!   during them.
//!
//! # Interrupt sampling
//!
//! The Z80 samples `INT` and `NMI` during the last T-state of an
//! instruction's last M-cycle, so the boundary between two steps is the same
//! instant and [`Exec::step`] can test them at the top. `EI` defers acceptance
//! by exactly one instruction — that is what makes `EI` / `RET` at the end of
//! a handler work — and a `$dd`/`$fd`/`$cb`/`$ed` prefix defers it too, which
//! comes for free because a prefixed instruction is one step here.
//!
//! # Sources
//!
//! Zilog **UM0080** for the instruction semantics, the interrupt modes and the
//! M-cycle timing diagrams; Sean Young's *Undocumented Z80 Documented* v0.91
//! for the flag rules the manual omits (`SCF`/`CCF`, `BIT n,(HL)`, the block
//! instructions, `IN r,(C)`); the *MEMPTR* write-up for the `WZ` rules. The
//! block-I/O repeat flags and the `Q` interaction were **checked against**
//! `SingleStepTests/z80` (MIT), which is measured hardware behaviour rather
//! than anyone's implementation of it. See `docs/cpu/z80-sm83.md`.

use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::value::Width;

use super::isa::{self, Cond, Index, Insn, Op, Operand, R8, R16};
use super::{BusCycle, Config, CycleLog, Lines, MCycle, Regs, flags};

/// The architectural state one core owns.
///
/// Split from [`super::Z80`] because the interrupt *lines* live outside the
/// lock: a device asserting `INT` from inside a CPU-initiated write would
/// otherwise re-enter the CPU's own critical section and deadlock (the
/// re-entrancy contract, `ROADMAP.md` §4.7).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct State {
    /// The register file, shadow set and `WZ` included.
    pub regs: Regs,
    /// Interrupt enable flip-flop 1: whether `INT` is accepted.
    pub iff1: bool,
    /// Interrupt enable flip-flop 2: `IFF1`'s backup across an `NMI`, and the
    /// value `LD A,I` copies into the parity flag.
    pub iff2: bool,
    /// The selected interrupt mode, 0 to 2.
    pub im: u8,
    /// Set by `HALT`; cleared by an interrupt or a reset.
    pub halted: bool,
    /// `EI` was the last instruction, so one more must run before `INT` is
    /// accepted.
    pub ei_pending: bool,
    /// `LD A,I` or `LD A,R` was the last instruction.
    pub after_ld_ir: bool,
    /// `Q`: the flags the last flag-writing instruction produced, or zero.
    pub q: u8,
    /// T-states executed since power-on.
    pub cycles: u64,
    /// A reset was requested and its sequence has not run yet.
    pub reset_pending: bool,
    /// How many accesses the address space refused.
    pub faults: u64,
    /// Address of the most recent refused access.
    pub last_fault: u16,
    /// The bus activity of the most recent step.
    pub trace: CycleLog,
}

impl State {
    /// Power-on state, before the reset sequence has run.
    pub(super) const fn new() -> State {
        State {
            regs: Regs::new(),
            iff1: false,
            iff2: false,
            im: 0,
            halted: false,
            ei_pending: false,
            after_ld_ir: false,
            q: 0,
            cycles: 0,
            reset_pending: true,
            faults: 0,
            last_fault: 0,
            trace: CycleLog::new(),
        }
    }
}

/// `S`, `Z`, `Y`, `X` and parity from one result byte — the flag pattern the
/// logical operations, the shifts and `IN r,(C)` all share.
#[inline]
fn sz53p(v: u8) -> u8 {
    let mut f = v & (flags::S | flags::XF | flags::YF);
    if v == 0 {
        f |= flags::Z;
    }
    if v.count_ones().is_multiple_of(2) {
        f |= flags::PV;
    }
    f
}

/// Even parity, as the Z80's `P/V` flag reports it.
#[inline]
fn parity(v: u8) -> bool {
    v.count_ones().is_multiple_of(2)
}

/// One instruction's worth of execution, borrowing everything it needs.
pub(super) struct Exec<'a> {
    state: &'a mut State,
    mem: &'a AddressSpace,
    io: Option<&'a AddressSpace>,
    cfg: &'a Config,
    lines: &'a Lines,
    attrs: MemAttrs,
    /// T-states this step has charged.
    used: u64,
    /// What the address pins hold: the last address any M-cycle drove, which
    /// is what an internal cycle leaves there.
    latch: u16,
    /// `Q` as the *previous* instruction left it, which is the value `SCF` and
    /// `CCF` read.
    prev_q: u8,
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
        let latch = state.regs.pc;
        let prev_q = state.q;
        Exec {
            state,
            mem,
            io,
            cfg,
            lines,
            attrs,
            used: 0,
            latch,
            prev_q,
        }
    }

    /// Run one reset sequence, interrupt sequence, halt cycle or instruction.
    pub(super) fn step(&mut self) -> u64 {
        self.state.trace.clear();
        if self.state.reset_pending {
            self.reset_sequence();
            return self.used;
        }
        // Sampled here because "between two steps" is the same instant as
        // "the last T-state of the last M-cycle". A pending EI hides both
        // lines for exactly one instruction.
        if !self.state.ei_pending {
            if self.lines.take_nmi_pending() {
                self.nmi_sequence();
                return self.used;
            }
            if self.state.iff1 && self.lines.irq_asserted() {
                self.irq_sequence();
                return self.used;
            }
        }
        if self.state.halted {
            self.halt_cycle();
            return self.used;
        }
        self.instruction();
        self.used
    }

    // -----------------------------------------------------------------
    // The clock: every T-state belongs to an M-cycle
    // -----------------------------------------------------------------

    fn charge(&mut self, tstates: u8) {
        self.used += u64::from(tstates);
        self.state.cycles = self.state.cycles.wrapping_add(u64::from(tstates));
    }

    fn log(&mut self, cycle: BusCycle) {
        self.state.trace.push(cycle);
    }

    /// An `M1` opcode fetch at `addr`: four T-states, a read, and the refresh
    /// address on the pins for the last two of them.
    ///
    /// `R` counts memory refresh, and only its low seven bits are a counter —
    /// bit 7 is a latch the program owns and the increment never carries into
    /// it (UM0080 §"CPU Registers"). The refresh address carries `R` as it was
    /// *before* the increment.
    fn m1(&mut self, addr: u16) -> u8 {
        // `Q` is cleared by the opcode fetch itself, not by the instruction:
        // that is why `DD 37` is a different `SCF` from a bare `37`, since the
        // prefix's own M1 has already wiped what the previous instruction
        // left behind.
        self.prev_q = self.state.q;
        self.state.q = 0;
        let refresh = (u16::from(self.state.regs.i) << 8) | u16::from(self.state.regs.r);
        let r = self.state.regs.r;
        self.state.regs.r = (r & 0x80) | (r.wrapping_add(1) & 0x7f);
        self.charge(4);
        let value = self.bus_read(addr);
        self.latch = refresh;
        self.log(BusCycle {
            kind: MCycle::Fetch,
            addr,
            value,
            refresh,
            tstates: 4,
        });
        value
    }

    /// Fetch the opcode at PC and advance past it.
    fn fetch_opcode(&mut self) -> u8 {
        let pc = self.state.regs.pc;
        self.state.regs.pc = pc.wrapping_add(1);
        self.m1(pc)
    }

    /// One memory read M-cycle: three T-states.
    fn read(&mut self, addr: u16) -> u8 {
        self.charge(3);
        let value = self.bus_read(addr);
        self.latch = addr;
        self.log(BusCycle {
            kind: MCycle::Read,
            addr,
            value,
            refresh: 0,
            tstates: 3,
        });
        value
    }

    /// One memory write M-cycle: three T-states.
    fn write(&mut self, addr: u16, value: u8) {
        self.charge(3);
        if self
            .mem
            .write(u64::from(addr), Width::U8, u64::from(value), self.attrs)
            .is_err()
        {
            self.fault(addr);
        }
        self.latch = addr;
        self.log(BusCycle {
            kind: MCycle::Write,
            addr,
            value,
            refresh: 0,
            tstates: 3,
        });
    }

    /// One I/O read M-cycle: four T-states, because the Z80 inserts one wait
    /// state automatically so slow peripherals need no `WAIT` logic
    /// (UM0080 §"Input or Output Cycles").
    fn io_read(&mut self, port: u16) -> u8 {
        self.charge(4);
        let value = match self.io {
            Some(space) => match space.read(u64::from(port), Width::U8, self.attrs) {
                Ok(v) => v as u8,
                Err(_) => {
                    self.fault(port);
                    self.cfg.floating_bus
                }
            },
            // A machine that wired no I/O space is not a machine with a fault
            // at every port; it is one where nothing answers, which on a Z80
            // bus reads as the floating value.
            None => self.cfg.floating_bus,
        };
        self.latch = port;
        self.log(BusCycle {
            kind: MCycle::PortRead,
            addr: port,
            value,
            refresh: 0,
            tstates: 4,
        });
        value
    }

    /// One I/O write M-cycle: four T-states.
    fn io_write(&mut self, port: u16, value: u8) {
        self.charge(4);
        if let Some(space) = self.io
            && space
                .write(u64::from(port), Width::U8, u64::from(value), self.attrs)
                .is_err()
        {
            self.fault(port);
        }
        self.latch = port;
        self.log(BusCycle {
            kind: MCycle::PortWrite,
            addr: port,
            value,
            refresh: 0,
            tstates: 4,
        });
    }

    /// `n` internal T-states: the adder, the incrementer, the branch
    /// displacement. No bus request, and the address pins keep whatever the
    /// last M-cycle left on them.
    fn idle(&mut self, tstates: u8) {
        if tstates == 0 {
            return;
        }
        self.charge(tstates);
        let addr = self.latch;
        self.log(BusCycle {
            kind: MCycle::Internal,
            addr,
            value: 0,
            refresh: 0,
            tstates,
        });
    }

    /// The interrupt-acknowledge cycle: an `M1` that takes its byte from the
    /// interrupting device rather than from memory, stretched to seven
    /// T-states by the two automatic wait states (UM0080 §"Interrupt Request
    /// / Acknowledge Cycle").
    fn int_ack(&mut self) -> u8 {
        let refresh = (u16::from(self.state.regs.i) << 8) | u16::from(self.state.regs.r);
        let r = self.state.regs.r;
        self.state.regs.r = (r & 0x80) | (r.wrapping_add(1) & 0x7f);
        self.charge(7);
        let value = self.lines.vector();
        let addr = self.state.regs.pc;
        self.latch = refresh;
        self.log(BusCycle {
            kind: MCycle::Ack,
            addr,
            value,
            refresh,
            tstates: 7,
        });
        value
    }

    fn bus_read(&mut self, addr: u16) -> u8 {
        match self.mem.read(u64::from(addr), Width::U8, self.attrs) {
            Ok(v) => v as u8,
            Err(_) => {
                self.fault(addr);
                self.cfg.floating_bus
            }
        }
    }

    fn fault(&mut self, addr: u16) {
        self.state.faults = self.state.faults.wrapping_add(1);
        self.state.last_fault = addr;
    }

    // -- composite accesses ---------------------------------------------

    /// Read the byte at PC and advance it.
    fn fetch(&mut self) -> u8 {
        let pc = self.state.regs.pc;
        // Guest arithmetic wraps: PC is 16 bits and $ffff is followed by $0000.
        self.state.regs.pc = pc.wrapping_add(1);
        self.read(pc)
    }

    /// Read the little-endian word at PC and advance past it.
    fn fetch_word(&mut self) -> u16 {
        let lo = self.fetch();
        let hi = self.fetch();
        u16::from(lo) | (u16::from(hi) << 8)
    }

    fn read_word(&mut self, addr: u16) -> u16 {
        let lo = self.read(addr);
        let hi = self.read(addr.wrapping_add(1));
        u16::from(lo) | (u16::from(hi) << 8)
    }

    fn write_word(&mut self, addr: u16, value: u16) {
        self.write(addr, value as u8);
        self.write(addr.wrapping_add(1), (value >> 8) as u8);
    }

    /// Push a word: the high byte first, at `SP - 1`.
    fn push_word(&mut self, value: u16) {
        let sp = self.state.regs.sp.wrapping_sub(1);
        self.write(sp, (value >> 8) as u8);
        let sp = sp.wrapping_sub(1);
        self.write(sp, value as u8);
        self.state.regs.sp = sp;
    }

    fn pop_word(&mut self) -> u16 {
        let sp = self.state.regs.sp;
        let value = self.read_word(sp);
        self.state.regs.sp = sp.wrapping_add(2);
        value
    }

    // -----------------------------------------------------------------
    // Registers
    // -----------------------------------------------------------------

    fn get8(&self, r: R8) -> u8 {
        let g = &self.state.regs;
        match r {
            R8::A => g.a,
            R8::B => g.b,
            R8::C => g.c,
            R8::D => g.d,
            R8::E => g.e,
            R8::H => g.h,
            R8::L => g.l,
            R8::I => g.i,
            R8::R => g.r,
            R8::Ixh => (g.ix >> 8) as u8,
            R8::Ixl => g.ix as u8,
            R8::Iyh => (g.iy >> 8) as u8,
            R8::Iyl => g.iy as u8,
        }
    }

    fn set8(&mut self, r: R8, v: u8) {
        let g = &mut self.state.regs;
        match r {
            R8::A => g.a = v,
            R8::B => g.b = v,
            R8::C => g.c = v,
            R8::D => g.d = v,
            R8::E => g.e = v,
            R8::H => g.h = v,
            R8::L => g.l = v,
            R8::I => g.i = v,
            R8::R => g.r = v,
            R8::Ixh => g.ix = (g.ix & 0x00ff) | (u16::from(v) << 8),
            R8::Ixl => g.ix = (g.ix & 0xff00) | u16::from(v),
            R8::Iyh => g.iy = (g.iy & 0x00ff) | (u16::from(v) << 8),
            R8::Iyl => g.iy = (g.iy & 0xff00) | u16::from(v),
        }
    }

    fn get16(&self, r: R16) -> u16 {
        let g = &self.state.regs;
        match r {
            R16::Af => g.af(),
            R16::Bc => g.bc(),
            R16::De => g.de(),
            R16::Hl => g.hl(),
            R16::Sp => g.sp,
            R16::Ix => g.ix,
            R16::Iy => g.iy,
            R16::AfAlt => g.af_alt,
        }
    }

    fn set16(&mut self, r: R16, v: u16) {
        let g = &mut self.state.regs;
        match r {
            R16::Af => g.set_af(v),
            R16::Bc => g.set_bc(v),
            R16::De => g.set_de(v),
            R16::Hl => g.set_hl(v),
            R16::Sp => g.sp = v,
            R16::Ix => g.ix = v,
            R16::Iy => g.iy = v,
            R16::AfAlt => g.af_alt = v,
        }
    }

    // -----------------------------------------------------------------
    // Flags
    // -----------------------------------------------------------------

    #[inline]
    fn f(&self) -> u8 {
        self.state.regs.f
    }

    /// Write the flags **and** `Q`, which is the point: `Q` is defined as "the
    /// flags the last flag-writing instruction produced", so it can only stay
    /// correct if every such write goes through here.
    #[inline]
    fn set_f(&mut self, v: u8) {
        self.state.regs.f = v;
        self.state.q = v;
    }

    fn cond_holds(&self, cond: Cond) -> bool {
        let f = self.f();
        match cond {
            Cond::Always => true,
            Cond::Nz => f & flags::Z == 0,
            Cond::Z => f & flags::Z != 0,
            Cond::Nc => f & flags::C == 0,
            Cond::C => f & flags::C != 0,
            Cond::Po => f & flags::PV == 0,
            Cond::Pe => f & flags::PV != 0,
            Cond::P => f & flags::S == 0,
            Cond::M => f & flags::S != 0,
        }
    }

    // -----------------------------------------------------------------
    // Reset and interrupts
    // -----------------------------------------------------------------

    /// The reset sequence: `PC`, `I` and `R` are cleared, both interrupt
    /// flip-flops are reset and mode 0 is selected (UM0080 §"Reset").
    ///
    /// Three T-states, which is how long `RESET` must be held; there is no
    /// vector fetch, because a Z80 starts at `$0000`. Nothing else moves —
    /// the manual is specific about what the pin touches, and a warm reset
    /// that quietly wiped `AF` or `SP` would be a worse lie than a core that
    /// comes up with them zeroed.
    fn reset_sequence(&mut self) {
        self.state.reset_pending = false;
        self.state.halted = false;
        self.state.iff1 = false;
        self.state.iff2 = false;
        self.state.im = 0;
        self.state.ei_pending = false;
        self.state.after_ld_ir = false;
        self.state.q = 0;
        self.state.regs.pc = 0;
        self.state.regs.i = 0;
        self.state.regs.r = 0;
        self.state.regs.wz = 0;
        self.idle(3);
    }

    /// `NMI`: acknowledge, save `IFF1` in `IFF2`, push and vector to `$0066`.
    /// Eleven T-states.
    fn nmi_sequence(&mut self) {
        self.enter_interrupt();
        let pc = self.state.regs.pc;
        // The acknowledge is an ordinary M1 whose opcode is thrown away, with
        // one extra T-state.
        self.m1(pc);
        self.idle(1);
        self.state.iff2 = self.state.iff1;
        self.state.iff1 = false;
        self.push_word(pc);
        self.state.regs.pc = 0x0066;
        self.state.regs.wz = 0x0066;
    }

    /// A maskable interrupt, in whichever mode is selected.
    fn irq_sequence(&mut self) {
        self.enter_interrupt();
        self.state.iff1 = false;
        self.state.iff2 = false;
        let vector = self.int_ack();
        match self.state.im {
            // Mode 0 executes the byte the device put on the bus. Real systems
            // put an `RST n` there; a multi-byte opcode would need its operand
            // bytes on the bus too, which no rsemu bus models, so only the
            // one-byte forms are honoured.
            0 => {
                let pc = self.state.regs.pc;
                if vector & 0xc7 == 0xc7 {
                    self.push_word(pc);
                    let target = u16::from(vector & 0x38);
                    self.state.regs.pc = target;
                    self.state.regs.wz = target;
                } else {
                    self.exec_base(vector, None);
                }
            }
            1 => {
                let pc = self.state.regs.pc;
                self.push_word(pc);
                self.state.regs.pc = 0x0038;
                self.state.regs.wz = 0x0038;
            }
            _ => {
                let pc = self.state.regs.pc;
                self.push_word(pc);
                let table = (u16::from(self.state.regs.i) << 8) | u16::from(vector);
                let target = self.read_word(table);
                self.state.regs.pc = target;
                self.state.regs.wz = target;
            }
        }
    }

    /// State every interrupt entry shares.
    fn enter_interrupt(&mut self) {
        self.state.halted = false;
        self.state.ei_pending = false;
        // `LD A,I` sets the parity flag from IFF2, and hardware clears it
        // again if an interrupt lands while the instruction is finishing.
        // Between steps that means: if the latch is set, the previous
        // instruction was one of the pair and its P/V was a lie.
        if self.state.after_ld_ir {
            let f = self.state.regs.f & !flags::PV;
            self.state.regs.f = f;
            if self.state.q != 0 {
                self.state.q = f;
            }
            self.state.after_ld_ir = false;
        }
        self.state.q = 0;
    }

    /// One M-cycle of the halted state.
    ///
    /// A halted Z80 is not stopped: it keeps issuing `M1` cycles so that
    /// dynamic RAM stays refreshed, re-fetching the `HALT` opcode itself
    /// (UM0080 §"HALT"). `PC` already points past the instruction, which is
    /// why an interrupt taken here returns to the right place.
    fn halt_cycle(&mut self) {
        let at = self.state.regs.pc.wrapping_sub(1);
        self.m1(at);
        self.state.q = 0;
    }

    // -----------------------------------------------------------------
    // Instruction dispatch
    // -----------------------------------------------------------------

    fn instruction(&mut self) {
        self.state.ei_pending = false;
        self.state.after_ld_ir = false;
        let mut index: Option<Index> = None;
        loop {
            let opcode = self.fetch_opcode();
            match opcode {
                // A second index prefix replaces the first, and each one costs
                // its own M1 cycle and its own R increment. `DD DD 00` is
                // three fetches and a NOP.
                0xdd => index = Some(Index::Ix),
                0xfd => index = Some(Index::Iy),
                0xcb => {
                    match index {
                        Some(i) => self.exec_ddcb(i),
                        None => self.exec_cb(),
                    }
                    return;
                }
                // The ED page is not indexable, and a prefix in front of it is
                // simply discarded.
                0xed => {
                    self.exec_ed();
                    return;
                }
                _ => {
                    self.exec_base(opcode, index);
                    return;
                }
            }
        }
    }

    // -----------------------------------------------------------------
    // Operand plumbing
    // -----------------------------------------------------------------

    /// The effective address of a memory operand.
    ///
    /// For `(IX+d)` this is where the displacement is fetched, where `WZ`
    /// becomes `IX + d`, and where the adder spends `idle` internal T-states —
    /// five for every form but `LD (IX+d),n`, which spends two of them after
    /// the immediate byte instead.
    fn mem_addr(&mut self, operand: Operand, idle: u8) -> u16 {
        match operand {
            Operand::Ind(p) => self.get16(p),
            Operand::Idx(p) => {
                let d = self.fetch() as i8 as i16 as u16;
                let ea = self.get16(p).wrapping_add(d);
                self.state.regs.wz = ea;
                self.idle(idle);
                ea
            }
            _ => unreachable!("not a memory operand"),
        }
    }

    fn is_mem(operand: Operand) -> bool {
        matches!(operand, Operand::Ind(_) | Operand::Idx(_))
    }

    /// An 8-bit source that is not memory: a register or the immediate byte.
    fn value8(&mut self, operand: Operand) -> u8 {
        match operand {
            Operand::Reg(r) => self.get8(r),
            Operand::Imm8 => self.fetch(),
            _ => unreachable!("not an 8-bit value operand"),
        }
    }

    // -----------------------------------------------------------------
    // The base page
    // -----------------------------------------------------------------

    fn exec_base(&mut self, opcode: u8, index: Option<Index>) {
        let raw = isa::decode(opcode);
        let insn = match index {
            Some(i) => isa::index_substitute(raw, i),
            None => raw,
        };
        match insn.op {
            Op::NOP => {}
            Op::HALT => self.state.halted = true,
            Op::LD => self.op_ld(insn),
            Op::INC | Op::DEC => self.op_incdec(insn),
            Op::ADD | Op::ADC | Op::SUB | Op::SBC | Op::AND | Op::XOR | Op::OR | Op::CP => {
                self.op_alu(insn);
            }
            Op::RLCA | Op::RRCA | Op::RLA | Op::RRA => self.op_rot_a(insn.op),
            Op::DAA => self.op_daa(),
            Op::CPL => self.op_cpl(),
            Op::SCF => self.op_scf(),
            Op::CCF => self.op_ccf(),
            Op::JR => self.op_jr(insn),
            Op::DJNZ => self.op_djnz(),
            Op::JP => self.op_jp(insn),
            Op::CALL => self.op_call(insn),
            Op::RET => self.op_ret(insn),
            Op::RST => self.op_rst(insn),
            Op::PUSH => self.op_push(insn),
            Op::POP => self.op_pop(insn),
            Op::EX => self.op_ex(insn),
            Op::EXX => self.op_exx(),
            Op::IN => self.op_in_imm(),
            Op::OUT => self.op_out_imm(),
            Op::DI => {
                self.state.iff1 = false;
                self.state.iff2 = false;
            }
            Op::EI => {
                self.state.iff1 = true;
                self.state.iff2 = true;
                self.state.ei_pending = true;
            }
            // Every base-page row is one of the above; the prefix rows never
            // reach here because `instruction` consumes them.
            _ => {}
        }
    }

    fn op_ld(&mut self, insn: Insn) {
        use Operand as O;
        match (insn.dst, insn.src) {
            // -- 16-bit ------------------------------------------------
            (O::Reg16(R16::Sp), O::Reg16(s)) => {
                // LD SP,HL / LD SP,IX: two internal T-states for the transfer.
                self.idle(2);
                let v = self.get16(s);
                self.state.regs.sp = v;
            }
            (O::Reg16(d), O::Imm16) => {
                let v = self.fetch_word();
                self.set16(d, v);
            }
            (O::Reg16(d), O::Abs) => {
                let a = self.fetch_word();
                let v = self.read_word(a);
                self.state.regs.wz = a.wrapping_add(1);
                self.set16(d, v);
            }
            (O::Abs, O::Reg16(s)) => {
                let a = self.fetch_word();
                let v = self.get16(s);
                self.write_word(a, v);
                self.state.regs.wz = a.wrapping_add(1);
            }
            // -- the accumulator's own addressing modes -----------------
            //
            // Only `(BC)` and `(DE)` latch WZ. `LD A,(HL)` is an ordinary
            // register load that happens to name a pair, and it leaves the
            // latch alone.
            (O::Reg(R8::A), O::Ind(p @ (R16::Bc | R16::De))) => {
                let a = self.get16(p);
                let v = self.read(a);
                self.state.regs.wz = a.wrapping_add(1);
                self.state.regs.a = v;
            }
            (O::Ind(p @ (R16::Bc | R16::De)), O::Reg(R8::A)) => {
                let a = self.get16(p);
                let value = self.state.regs.a;
                self.write(a, value);
                self.set_wz_after_a_store(a, value);
            }
            (O::Reg(R8::A), O::Abs) => {
                let a = self.fetch_word();
                let v = self.read(a);
                self.state.regs.wz = a.wrapping_add(1);
                self.state.regs.a = v;
            }
            (O::Abs, O::Reg(R8::A)) => {
                let a = self.fetch_word();
                let value = self.state.regs.a;
                self.write(a, value);
                self.set_wz_after_a_store(a, value);
            }
            // -- 8-bit -------------------------------------------------
            (dst, O::Imm8) if Self::is_mem(dst) => {
                // `LD (IX+d),n` puts the displacement *before* the immediate,
                // and the adder gets two of its five internal T-states after
                // the byte rather than five before it.
                let indexed = matches!(dst, O::Idx(_));
                let ea = self.mem_addr(dst, 0);
                let v = self.fetch();
                self.idle(if indexed { 2 } else { 0 });
                self.write(ea, v);
            }
            (dst, src) if Self::is_mem(src) => {
                let ea = self.mem_addr(src, 5);
                let v = self.read(ea);
                match dst {
                    O::Reg(r) => self.set8(r, v),
                    _ => unreachable!("a memory source loads a register"),
                }
            }
            (dst, src) if Self::is_mem(dst) => {
                let ea = self.mem_addr(dst, 5);
                let v = self.value8(src);
                self.write(ea, v);
            }
            (O::Reg(d), src) => {
                let v = self.value8(src);
                self.set8(d, v);
            }
            _ => unreachable!("unhandled LD shape"),
        }
    }

    /// `WZ` after a store through the accumulator.
    ///
    /// The low half is the incremented address as usual, but the high half is
    /// `A` — the byte that was on the data bus — because nothing drove the
    /// upper address latch (the *MEMPTR* write-up, §"LD (nn),A").
    fn set_wz_after_a_store(&mut self, addr: u16, value: u8) {
        self.state.regs.wz = ((addr.wrapping_add(1)) & 0x00ff) | (u16::from(value) << 8);
    }

    fn op_incdec(&mut self, insn: Insn) {
        let up = insn.op == Op::INC;
        match insn.dst {
            Operand::Reg16(p) => {
                self.idle(2);
                let v = self.get16(p);
                let v = if up {
                    v.wrapping_add(1)
                } else {
                    v.wrapping_sub(1)
                };
                self.set16(p, v);
            }
            Operand::Reg(r) => {
                let v = self.get8(r);
                let v = if up { self.inc8(v) } else { self.dec8(v) };
                self.set8(r, v);
            }
            operand => {
                let ea = self.mem_addr(operand, 5);
                let v = self.read(ea);
                self.idle(1);
                let v = if up { self.inc8(v) } else { self.dec8(v) };
                self.write(ea, v);
            }
        }
    }

    fn op_alu(&mut self, insn: Insn) {
        // `ADD HL,rp` is the one 16-bit member of the family.
        if let Operand::Reg16(d) = insn.dst
            && let Operand::Reg16(s) = insn.src
        {
            self.idle(7);
            let a = self.get16(d);
            let b = self.get16(s);
            self.state.regs.wz = a.wrapping_add(1);
            let r = self.add16(a, b);
            self.set16(d, r);
            return;
        }
        let v = if Self::is_mem(insn.src) {
            let ea = self.mem_addr(insn.src, 5);
            self.read(ea)
        } else {
            self.value8(insn.src)
        };
        self.apply_alu(insn.op, v);
    }

    fn apply_alu(&mut self, op: Op, v: u8) {
        let carry = self.f() & flags::C != 0;
        match op {
            Op::ADD => self.alu_add(v, false),
            Op::ADC => self.alu_add(v, carry),
            Op::SUB => self.alu_sub(v, false, true),
            Op::SBC => self.alu_sub(v, carry, true),
            Op::CP => self.alu_sub(v, false, false),
            Op::AND => {
                let r = self.state.regs.a & v;
                self.state.regs.a = r;
                self.set_f(sz53p(r) | flags::H);
            }
            Op::XOR => {
                let r = self.state.regs.a ^ v;
                self.state.regs.a = r;
                self.set_f(sz53p(r));
            }
            Op::OR => {
                let r = self.state.regs.a | v;
                self.state.regs.a = r;
                self.set_f(sz53p(r));
            }
            _ => unreachable!("not an ALU operation"),
        }
    }

    fn op_rot_a(&mut self, op: Op) {
        let a = self.state.regs.a;
        let c = self.f() & flags::C != 0;
        let (r, carry) = match op {
            Op::RLCA => (a.rotate_left(1), a & 0x80 != 0),
            Op::RRCA => (a.rotate_right(1), a & 0x01 != 0),
            Op::RLA => ((a << 1) | u8::from(c), a & 0x80 != 0),
            Op::RRA => ((a >> 1) | (u8::from(c) << 7), a & 0x01 != 0),
            _ => unreachable!("not an accumulator rotate"),
        };
        // S, Z and P/V survive; the undocumented bits come from the result.
        let mut f = (self.f() & (flags::S | flags::Z | flags::PV)) | (r & flags::XY);
        if carry {
            f |= flags::C;
        }
        self.state.regs.a = r;
        self.set_f(f);
    }

    /// `DAA`: fold the packed-BCD correction back into `A` (UM0080's `DAA`
    /// table, which this reproduces as a computation rather than a lookup).
    fn op_daa(&mut self) {
        let a = self.state.regs.a;
        let f0 = self.f();
        let mut adjust = 0u8;
        let mut carry = f0 & flags::C != 0;
        if f0 & flags::H != 0 || a & 0x0f > 9 {
            adjust |= 0x06;
        }
        if carry || a > 0x99 {
            adjust |= 0x60;
            carry = true;
        }
        let r = if f0 & flags::N != 0 {
            a.wrapping_sub(adjust)
        } else {
            a.wrapping_add(adjust)
        };
        let mut f = sz53p(r) | (f0 & flags::N);
        // The half-carry is whatever bit 4 did, in either direction.
        if (a ^ r) & 0x10 != 0 {
            f |= flags::H;
        }
        if carry {
            f |= flags::C;
        }
        self.state.regs.a = r;
        self.set_f(f);
    }

    fn op_cpl(&mut self) {
        let a = !self.state.regs.a;
        self.state.regs.a = a;
        let f = (self.f() & (flags::S | flags::Z | flags::PV | flags::C))
            | flags::H
            | flags::N
            | (a & flags::XY);
        self.set_f(f);
    }

    /// `SCF` and `CCF` take their undocumented bits from `((Q ^ F) | A)`.
    ///
    /// When the previous instruction wrote flags, `Q == F` and the expression
    /// collapses to `A`. When it did not, `Q == 0` and the bits are `F | A` —
    /// which is the observable difference, and the whole reason `Q` exists.
    fn scf_ccf_xy(&mut self, f: u8) -> u8 {
        ((self.prev_q ^ f) | self.state.regs.a) & flags::XY
    }

    fn op_scf(&mut self) {
        let f0 = self.f();
        let xy = self.scf_ccf_xy(f0);
        let f = (f0 & !(flags::H | flags::N | flags::XY)) | flags::C | xy;
        self.set_f(f);
    }

    fn op_ccf(&mut self) {
        let f0 = self.f();
        let xy = self.scf_ccf_xy(f0);
        let mut f = (f0 & !(flags::H | flags::N | flags::C | flags::XY)) | xy;
        // The old carry lands in the half-carry, which is how `CCF` stays
        // reversible.
        if f0 & flags::C != 0 {
            f |= flags::H;
        } else {
            f |= flags::C;
        }
        self.set_f(f);
    }

    fn op_jr(&mut self, insn: Insn) {
        let d = self.fetch() as i8 as i16 as u16;
        if self.cond_holds(insn.cond) {
            self.idle(5);
            let target = self.state.regs.pc.wrapping_add(d);
            self.state.regs.pc = target;
            self.state.regs.wz = target;
        }
    }

    fn op_djnz(&mut self) {
        self.idle(1);
        let d = self.fetch() as i8 as i16 as u16;
        let b = self.state.regs.b.wrapping_sub(1);
        self.state.regs.b = b;
        if b != 0 {
            self.idle(5);
            let target = self.state.regs.pc.wrapping_add(d);
            self.state.regs.pc = target;
            self.state.regs.wz = target;
        }
    }

    fn op_jp(&mut self, insn: Insn) {
        if let Operand::Ptr(p) = insn.src {
            // `JP (HL)` reads no memory and leaves WZ alone: nothing was
            // latched, because nothing was addressed.
            self.state.regs.pc = self.get16(p);
            return;
        }
        let target = self.fetch_word();
        // WZ is loaded whether or not the branch is taken — the address went
        // through the latch either way.
        self.state.regs.wz = target;
        if self.cond_holds(insn.cond) {
            self.state.regs.pc = target;
        }
    }

    fn op_call(&mut self, insn: Insn) {
        let target = self.fetch_word();
        self.state.regs.wz = target;
        if self.cond_holds(insn.cond) {
            self.idle(1);
            let ret = self.state.regs.pc;
            self.push_word(ret);
            self.state.regs.pc = target;
        }
    }

    fn op_ret(&mut self, insn: Insn) {
        if insn.cond != Cond::Always {
            // The condition is evaluated in one internal T-state before the
            // stack is touched, which is why `RET cc` costs five when it does
            // nothing.
            self.idle(1);
            if !self.cond_holds(insn.cond) {
                return;
            }
        }
        let target = self.pop_word();
        self.state.regs.pc = target;
        self.state.regs.wz = target;
    }

    fn op_rst(&mut self, insn: Insn) {
        let Operand::Rst(target) = insn.src else {
            unreachable!("RST always carries its target")
        };
        self.idle(1);
        let ret = self.state.regs.pc;
        self.push_word(ret);
        let target = u16::from(target);
        self.state.regs.pc = target;
        self.state.regs.wz = target;
    }

    fn op_push(&mut self, insn: Insn) {
        let Operand::Reg16(p) = insn.dst else {
            unreachable!("PUSH always names a pair")
        };
        self.idle(1);
        let v = self.get16(p);
        self.push_word(v);
    }

    fn op_pop(&mut self, insn: Insn) {
        let Operand::Reg16(p) = insn.dst else {
            unreachable!("POP always names a pair")
        };
        let v = self.pop_word();
        // `POP AF` writes F without computing it, so Q stays zero: this
        // deliberately does not go through `set_f`.
        self.set16(p, v);
    }

    fn op_ex(&mut self, insn: Insn) {
        use Operand as O;
        match (insn.dst, insn.src) {
            (O::Reg16(R16::Af), O::Reg16(R16::AfAlt)) => {
                let af = self.state.regs.af();
                let alt = self.state.regs.af_alt;
                self.state.regs.af_alt = af;
                self.state.regs.a = (alt >> 8) as u8;
                // Not `set_f`: the exchange moves flags, it does not compute
                // them, so Q stays zero.
                self.state.regs.f = alt as u8;
            }
            (O::Reg16(R16::De), O::Reg16(R16::Hl)) => {
                let de = self.state.regs.de();
                let hl = self.state.regs.hl();
                self.state.regs.set_de(hl);
                self.state.regs.set_hl(de);
            }
            (O::Ind(R16::Sp), O::Reg16(p)) => {
                let sp = self.state.regs.sp;
                let popped = self.read_word(sp);
                self.idle(1);
                let held = self.get16(p);
                self.write(sp.wrapping_add(1), (held >> 8) as u8);
                self.write(sp, held as u8);
                self.idle(2);
                self.set16(p, popped);
                self.state.regs.wz = popped;
            }
            _ => unreachable!("unhandled EX shape"),
        }
    }

    fn op_exx(&mut self) {
        let g = &mut self.state.regs;
        let bc = g.bc();
        let de = g.de();
        let hl = g.hl();
        g.set_bc(g.bc_alt);
        g.set_de(g.de_alt);
        g.set_hl(g.hl_alt);
        g.bc_alt = bc;
        g.de_alt = de;
        g.hl_alt = hl;
    }

    /// `IN A,(n)`: the accumulator supplies the high half of the port
    /// address, and `WZ` ends at that address plus one.
    fn op_in_imm(&mut self) {
        let n = self.fetch();
        let port = (u16::from(self.state.regs.a) << 8) | u16::from(n);
        let v = self.io_read(port);
        self.state.regs.wz = port.wrapping_add(1);
        self.state.regs.a = v;
    }

    /// `OUT (n),A`: like the store forms of `LD`, only the low half of `WZ`
    /// is incremented and the high half comes from `A`.
    fn op_out_imm(&mut self) {
        let n = self.fetch();
        let a = self.state.regs.a;
        let port = (u16::from(a) << 8) | u16::from(n);
        self.io_write(port, a);
        self.state.regs.wz = (u16::from(n.wrapping_add(1))) | (u16::from(a) << 8);
    }

    // -----------------------------------------------------------------
    // The CB page
    // -----------------------------------------------------------------

    fn exec_cb(&mut self) {
        let opcode = self.fetch_opcode();
        let insn = isa::decode_cb(opcode);
        match insn.op {
            Op::BIT => {
                let Operand::Bit(n) = insn.dst else {
                    unreachable!("BIT carries its bit index")
                };
                match insn.src {
                    Operand::Reg(r) => {
                        let v = self.get8(r);
                        self.op_bit(n, v, v);
                    }
                    operand => {
                        let ea = self.mem_addr(operand, 0);
                        let v = self.read(ea);
                        self.idle(1);
                        // Through `(HL)` the undocumented bits come from the
                        // *internal* address latch, not from the byte read.
                        let xy = (self.state.regs.wz >> 8) as u8;
                        self.op_bit(n, v, xy);
                    }
                }
            }
            Op::RES | Op::SET => {
                let Operand::Bit(n) = insn.dst else {
                    unreachable!("RES/SET carry their bit index")
                };
                let mask = 1u8 << n;
                let apply = |v: u8| {
                    if insn.op == Op::SET {
                        v | mask
                    } else {
                        v & !mask
                    }
                };
                match insn.src {
                    Operand::Reg(r) => {
                        let v = apply(self.get8(r));
                        self.set8(r, v);
                    }
                    operand => {
                        let ea = self.mem_addr(operand, 0);
                        let v = self.read(ea);
                        self.idle(1);
                        self.write(ea, apply(v));
                    }
                }
            }
            op => match insn.dst {
                Operand::Reg(r) => {
                    let v = self.get8(r);
                    let v = self.rotate(op, v);
                    self.set8(r, v);
                }
                operand => {
                    let ea = self.mem_addr(operand, 0);
                    let v = self.read(ea);
                    self.idle(1);
                    let v = self.rotate(op, v);
                    self.write(ea, v);
                }
            },
        }
    }

    /// The `DDCB`/`FDCB` page.
    ///
    /// The byte order is the oddity: `$dd $cb d opcode`, so the displacement
    /// is read *before* the opcode that uses it, and the opcode itself is an
    /// ordinary memory read rather than an `M1` — which is why `R` advances by
    /// two across a four-byte instruction.
    fn exec_ddcb(&mut self, index: Index) {
        let d = self.fetch() as i8 as i16 as u16;
        let opcode = self.fetch();
        self.idle(2);
        let insn = isa::decode_ddcb(opcode, index);
        let ea = self.get16(index.reg16()).wrapping_add(d);
        self.state.regs.wz = ea;
        let v = self.read(ea);
        self.idle(1);
        match insn.op {
            Op::BIT => {
                let Operand::Bit(n) = insn.dst else {
                    unreachable!("BIT carries its bit index")
                };
                let xy = (ea >> 8) as u8;
                self.op_bit(n, v, xy);
            }
            Op::RES | Op::SET => {
                let Operand::Bit(n) = insn.dst else {
                    unreachable!("RES/SET carry their bit index")
                };
                let mask = 1u8 << n;
                let r = if insn.op == Op::SET {
                    v | mask
                } else {
                    v & !mask
                };
                self.write(ea, r);
                if let Some(also) = insn.also {
                    self.set8(also, r);
                }
            }
            op => {
                let r = self.rotate(op, v);
                self.write(ea, r);
                if let Some(also) = insn.also {
                    self.set8(also, r);
                }
            }
        }
    }

    fn rotate(&mut self, op: Op, v: u8) -> u8 {
        let c = self.f() & flags::C != 0;
        let (r, carry) = match op {
            Op::RLC => (v.rotate_left(1), v & 0x80 != 0),
            Op::RRC => (v.rotate_right(1), v & 0x01 != 0),
            Op::RL => ((v << 1) | u8::from(c), v & 0x80 != 0),
            Op::RR => ((v >> 1) | (u8::from(c) << 7), v & 0x01 != 0),
            Op::SLA => (v << 1, v & 0x80 != 0),
            Op::SRA => ((v >> 1) | (v & 0x80), v & 0x01 != 0),
            // `SLL` shifts a one in at the bottom, which is the whole reason
            // it is undocumented rather than a second `SLA`.
            Op::SLL => ((v << 1) | 1, v & 0x80 != 0),
            Op::SRL => (v >> 1, v & 0x01 != 0),
            _ => unreachable!("not a rotate or shift"),
        };
        let mut f = sz53p(r);
        if carry {
            f |= flags::C;
        }
        self.set_f(f);
        r
    }

    /// `BIT n,s`: `Z` and `P/V` are the bit's complement, `H` is set, `S` is
    /// the bit itself but only for bit 7, and the carry is untouched.
    fn op_bit(&mut self, n: u8, v: u8, xy_from: u8) {
        let set = v & (1u8 << n) != 0;
        let mut f = (self.f() & flags::C) | flags::H | (xy_from & flags::XY);
        if !set {
            f |= flags::Z | flags::PV;
        }
        if n == 7 && set {
            f |= flags::S;
        }
        self.set_f(f);
    }

    // -----------------------------------------------------------------
    // The ED page
    // -----------------------------------------------------------------

    fn exec_ed(&mut self) {
        let opcode = self.fetch_opcode();
        let insn = isa::decode_ed(opcode);
        match insn.op {
            // Every hole on the page is two M1 cycles and nothing else, which
            // is exactly what has already been charged.
            Op::NOP => {}
            Op::IN => {
                let port = self.state.regs.bc();
                let v = self.io_read(port);
                self.state.regs.wz = port.wrapping_add(1);
                if let Operand::Reg(r) = insn.dst {
                    self.set8(r, v);
                }
                let f = sz53p(v) | (self.f() & flags::C);
                self.set_f(f);
            }
            Op::OUT => {
                let port = self.state.regs.bc();
                let v = match insn.src {
                    Operand::Reg(r) => self.get8(r),
                    // The undocumented `OUT (C),0` writes a literal, which is
                    // $00 on an NMOS Z80 and $ff on the CMOS part.
                    _ => self.cfg.out_c_zero,
                };
                self.io_write(port, v);
                self.state.regs.wz = port.wrapping_add(1);
            }
            Op::ADC | Op::SBC => {
                self.idle(7);
                let Operand::Reg16(s) = insn.src else {
                    unreachable!("16-bit ADC/SBC name a pair")
                };
                let a = self.state.regs.hl();
                let b = self.get16(s);
                self.state.regs.wz = a.wrapping_add(1);
                let r = if insn.op == Op::ADC {
                    self.adc16(a, b)
                } else {
                    self.sbc16(a, b)
                };
                self.state.regs.set_hl(r);
            }
            Op::LD => self.ed_ld(insn),
            Op::NEG => {
                let v = self.state.regs.a;
                self.state.regs.a = 0;
                self.alu_sub(v, false, true);
            }
            Op::RETN | Op::RETI => {
                let target = self.pop_word();
                self.state.regs.pc = target;
                self.state.regs.wz = target;
                // Both encodings restore IFF1 from its NMI backup; the only
                // difference is the pulse RETI puts on the bus for a daisy
                // chain, which nothing here models yet.
                self.state.iff1 = self.state.iff2;
            }
            Op::IM => {
                let Operand::Mode(m) = insn.src else {
                    unreachable!("IM carries its mode")
                };
                self.state.im = m;
            }
            Op::RRD | Op::RLD => self.ed_rotate_digit(insn.op),
            Op::LDI | Op::LDD | Op::LDIR | Op::LDDR => self.block_copy(insn.op),
            Op::CPI | Op::CPD | Op::CPIR | Op::CPDR => self.block_compare(insn.op),
            Op::INI | Op::IND | Op::INIR | Op::INDR => self.block_in(insn.op),
            Op::OUTI | Op::OUTD | Op::OTIR | Op::OTDR => self.block_out(insn.op),
            _ => {}
        }
    }

    fn ed_ld(&mut self, insn: Insn) {
        use Operand as O;
        match (insn.dst, insn.src) {
            (O::Abs, O::Reg16(s)) => {
                let a = self.fetch_word();
                let v = self.get16(s);
                self.write_word(a, v);
                self.state.regs.wz = a.wrapping_add(1);
            }
            (O::Reg16(d), O::Abs) => {
                let a = self.fetch_word();
                let v = self.read_word(a);
                self.state.regs.wz = a.wrapping_add(1);
                self.set16(d, v);
            }
            // LD I,A and LD R,A: one internal T-state, no flags.
            (O::Reg(d @ (R8::I | R8::R)), O::Reg(R8::A)) => {
                self.idle(1);
                let v = self.state.regs.a;
                self.set8(d, v);
            }
            // LD A,I and LD A,R: the parity flag is a copy of IFF2, which is
            // how a program reads an otherwise invisible flip-flop.
            (O::Reg(R8::A), O::Reg(s @ (R8::I | R8::R))) => {
                self.idle(1);
                let v = self.get8(s);
                self.state.regs.a = v;
                let mut f = sz53p(v) & !flags::PV;
                if self.state.iff2 {
                    f |= flags::PV;
                }
                f |= self.f() & flags::C;
                self.set_f(f);
                self.state.after_ld_ir = true;
            }
            _ => unreachable!("unhandled ED LD shape"),
        }
    }

    /// `RRD` and `RLD` rotate one BCD digit between `A` and `(HL)`.
    fn ed_rotate_digit(&mut self, op: Op) {
        let hl = self.state.regs.hl();
        let m = self.read(hl);
        self.idle(4);
        let a = self.state.regs.a;
        let (new_m, new_a) = if op == Op::RRD {
            (((a & 0x0f) << 4) | (m >> 4), (a & 0xf0) | (m & 0x0f))
        } else {
            ((m << 4) | (a & 0x0f), (a & 0xf0) | (m >> 4))
        };
        self.write(hl, new_m);
        self.state.regs.wz = hl.wrapping_add(1);
        self.state.regs.a = new_a;
        let f = sz53p(new_a) | (self.f() & flags::C);
        self.set_f(f);
    }

    /// The repeat tail every `xxxR` block instruction shares.
    ///
    /// Backing `PC` up by two re-executes the instruction, which is how the
    /// Z80 stays interruptible mid-block. The undocumented flag bits then come
    /// from the high byte of that `PC` rather than from the data, and `WZ`
    /// ends one past it.
    fn block_repeat(&mut self) {
        self.idle(5);
        let pc = self.state.regs.pc.wrapping_sub(2);
        self.state.regs.pc = pc;
        self.state.regs.wz = pc.wrapping_add(1);
        let f = (self.f() & !flags::XY) | (((pc >> 8) as u8) & flags::XY);
        self.set_f(f);
    }

    fn block_copy(&mut self, op: Op) {
        let up = matches!(op, Op::LDI | Op::LDIR);
        let hl = self.state.regs.hl();
        let de = self.state.regs.de();
        let v = self.read(hl);
        self.write(de, v);
        self.idle(2);
        let step = if up { 1u16 } else { 0xffffu16 };
        self.state.regs.set_hl(hl.wrapping_add(step));
        self.state.regs.set_de(de.wrapping_add(step));
        let bc = self.state.regs.bc().wrapping_sub(1);
        self.state.regs.set_bc(bc);
        // The undocumented bits come from `A + the byte moved`: bit 1 of the
        // sum lands in YF and bit 3 in XF.
        let n = self.state.regs.a.wrapping_add(v);
        let mut f = self.f() & (flags::S | flags::Z | flags::C);
        f |= (n << 4) & flags::YF;
        f |= n & flags::XF;
        if bc != 0 {
            f |= flags::PV;
        }
        self.set_f(f);
        if bc != 0 && matches!(op, Op::LDIR | Op::LDDR) {
            self.block_repeat();
        }
    }

    fn block_compare(&mut self, op: Op) {
        let up = matches!(op, Op::CPI | Op::CPIR);
        let hl = self.state.regs.hl();
        let v = self.read(hl);
        self.idle(5);
        let step = if up { 1u16 } else { 0xffffu16 };
        self.state.regs.set_hl(hl.wrapping_add(step));
        self.state.regs.wz = self.state.regs.wz.wrapping_add(step);
        let bc = self.state.regs.bc().wrapping_sub(1);
        self.state.regs.set_bc(bc);
        let a = self.state.regs.a;
        let diff = a.wrapping_sub(v);
        let half = (a & 0x0f) < (v & 0x0f);
        let mut f = flags::N | (self.f() & flags::C) | (diff & flags::S);
        if diff == 0 {
            f |= flags::Z;
        }
        if half {
            f |= flags::H;
        }
        if bc != 0 {
            f |= flags::PV;
        }
        // As with the copies, but from the difference minus the borrow.
        let n = diff.wrapping_sub(u8::from(half));
        f |= (n << 4) & flags::YF;
        f |= n & flags::XF;
        self.set_f(f);
        if bc != 0 && diff != 0 && matches!(op, Op::CPIR | Op::CPDR) {
            self.block_repeat();
        }
    }

    /// The flag rule the four block-I/O instructions share.
    ///
    /// `k` is the byte moved plus the neighbouring counter — `C ± 1` for the
    /// input forms, `L` for the output ones — and it decides both the carry
    /// and the parity, which is why block I/O leaves such an odd `F` behind
    /// (Sean Young, *Undocumented Z80 Documented* §4.3, checked against
    /// `SingleStepTests/z80`).
    fn block_io_flags(&mut self, b: u8, value: u8, k: u16) {
        let cf = k > 0xff;
        let mut f = b & (flags::S | flags::XY);
        if b == 0 {
            f |= flags::Z;
        }
        if value & 0x80 != 0 {
            f |= flags::N;
        }
        if cf {
            f |= flags::H | flags::C;
        }
        if parity(((k & 7) as u8) ^ b) {
            f |= flags::PV;
        }
        self.set_f(f);
    }

    /// The extra flag mangling the *repeating* block-I/O forms perform on
    /// every iteration but the last.
    ///
    /// Undocumented and asymmetric: the half-carry becomes a nibble-boundary
    /// test on the counter, and the parity is re-derived from a neighbouring
    /// value. Measured against `SingleStepTests/z80`; no manual describes it.
    fn block_io_repeat_flags(&mut self, b: u8, value: u8, k: u16) {
        let cf = k > 0xff;
        let mut f = self.f();
        let pf = f & flags::PV != 0;
        let (half, neighbour) = if !cf {
            (false, b)
        } else if value & 0x80 != 0 {
            (b & 0x0f == 0x00, b.wrapping_sub(1))
        } else {
            (b & 0x0f == 0x0f, b.wrapping_add(1))
        };
        f &= !(flags::H | flags::PV);
        if half {
            f |= flags::H;
        }
        if pf == parity(neighbour & 7) {
            f |= flags::PV;
        }
        self.set_f(f);
        self.block_repeat();
    }

    fn block_in(&mut self, op: Op) {
        let up = matches!(op, Op::INI | Op::INIR);
        self.idle(1);
        let bc = self.state.regs.bc();
        let value = self.io_read(bc);
        let step = if up { 1u16 } else { 0xffffu16 };
        self.state.regs.wz = bc.wrapping_add(step);
        let b = self.state.regs.b.wrapping_sub(1);
        self.state.regs.b = b;
        let hl = self.state.regs.hl();
        self.write(hl, value);
        self.state.regs.set_hl(hl.wrapping_add(step));
        let c = self.state.regs.c;
        let k = u16::from(value)
            + u16::from(if up {
                c.wrapping_add(1)
            } else {
                c.wrapping_sub(1)
            });
        self.block_io_flags(b, value, k);
        if b != 0 && matches!(op, Op::INIR | Op::INDR) {
            self.block_io_repeat_flags(b, value, k);
        }
    }

    fn block_out(&mut self, op: Op) {
        let up = matches!(op, Op::OUTI | Op::OTIR);
        self.idle(1);
        let hl = self.state.regs.hl();
        let value = self.read(hl);
        let b = self.state.regs.b.wrapping_sub(1);
        self.state.regs.b = b;
        let step = if up { 1u16 } else { 0xffffu16 };
        self.state.regs.set_hl(hl.wrapping_add(step));
        // B is decremented before the port address is driven, unlike the input
        // forms — the one asymmetry between them.
        let bc = self.state.regs.bc();
        self.io_write(bc, value);
        self.state.regs.wz = bc.wrapping_add(step);
        let k = u16::from(value) + u16::from(self.state.regs.l);
        self.block_io_flags(b, value, k);
        if b != 0 && matches!(op, Op::OTIR | Op::OTDR) {
            self.block_io_repeat_flags(b, value, k);
        }
    }

    // -----------------------------------------------------------------
    // The ALU
    // -----------------------------------------------------------------

    fn alu_add(&mut self, v: u8, carry: bool) {
        let a = self.state.regs.a;
        let c = u16::from(carry);
        let wide = u16::from(a) + u16::from(v) + c;
        let r = wide as u8;
        let mut f = r & (flags::S | flags::XY);
        if r == 0 {
            f |= flags::Z;
        }
        if u16::from(a & 0x0f) + u16::from(v & 0x0f) + c > 0x0f {
            f |= flags::H;
        }
        // Overflow is "the operands agreed on a sign and the result did not".
        if (a ^ v) & 0x80 == 0 && (a ^ r) & 0x80 != 0 {
            f |= flags::PV;
        }
        if wide > 0xff {
            f |= flags::C;
        }
        self.state.regs.a = r;
        self.set_f(f);
    }

    /// Subtraction, shared by `SUB`, `SBC`, `CP` and `NEG`.
    ///
    /// `store` is what separates `CP` from the rest, and it changes more than
    /// where the result goes: `CP` takes its undocumented flag bits from the
    /// *operand*, because the result never reaches a register and its bits
    /// never reach the flag latch either.
    fn alu_sub(&mut self, v: u8, carry: bool, store: bool) {
        let a = self.state.regs.a;
        let c = u16::from(carry);
        let wide = u16::from(a).wrapping_sub(u16::from(v)).wrapping_sub(c);
        let r = wide as u8;
        let mut f = flags::N | (r & flags::S);
        if r == 0 {
            f |= flags::Z;
        }
        if i16::from(a & 0x0f) - i16::from(v & 0x0f) - (c as i16) < 0 {
            f |= flags::H;
        }
        if (a ^ v) & 0x80 != 0 && (a ^ r) & 0x80 != 0 {
            f |= flags::PV;
        }
        if wide & 0x100 != 0 {
            f |= flags::C;
        }
        f |= if store { r } else { v } & flags::XY;
        if store {
            self.state.regs.a = r;
        }
        self.set_f(f);
    }

    fn inc8(&mut self, v: u8) -> u8 {
        let r = v.wrapping_add(1);
        let mut f = (self.f() & flags::C) | (r & (flags::S | flags::XY));
        if r == 0 {
            f |= flags::Z;
        }
        if r & 0x0f == 0 {
            f |= flags::H;
        }
        if v == 0x7f {
            f |= flags::PV;
        }
        self.set_f(f);
        r
    }

    fn dec8(&mut self, v: u8) -> u8 {
        let r = v.wrapping_sub(1);
        let mut f = (self.f() & flags::C) | flags::N | (r & (flags::S | flags::XY));
        if r == 0 {
            f |= flags::Z;
        }
        if v & 0x0f == 0 {
            f |= flags::H;
        }
        if v == 0x80 {
            f |= flags::PV;
        }
        self.set_f(f);
        r
    }

    /// `ADD HL,rp`: only the carries move, and the undocumented bits come from
    /// the high byte of the result.
    fn add16(&mut self, a: u16, b: u16) -> u16 {
        let wide = u32::from(a) + u32::from(b);
        let r = wide as u16;
        let mut f = self.f() & (flags::S | flags::Z | flags::PV);
        f |= ((r >> 8) as u8) & flags::XY;
        if (a & 0x0fff) + (b & 0x0fff) > 0x0fff {
            f |= flags::H;
        }
        if wide > 0xffff {
            f |= flags::C;
        }
        self.set_f(f);
        r
    }

    fn adc16(&mut self, a: u16, b: u16) -> u16 {
        let c = u32::from(self.f() & flags::C != 0);
        let wide = u32::from(a) + u32::from(b) + c;
        let r = wide as u16;
        let mut f = ((r >> 8) as u8) & (flags::S | flags::XY);
        if r == 0 {
            f |= flags::Z;
        }
        if u32::from(a & 0x0fff) + u32::from(b & 0x0fff) + c > 0x0fff {
            f |= flags::H;
        }
        if (a ^ b) & 0x8000 == 0 && (a ^ r) & 0x8000 != 0 {
            f |= flags::PV;
        }
        if wide > 0xffff {
            f |= flags::C;
        }
        self.set_f(f);
        r
    }

    fn sbc16(&mut self, a: u16, b: u16) -> u16 {
        let c = u32::from(self.f() & flags::C != 0);
        let wide = u32::from(a).wrapping_sub(u32::from(b)).wrapping_sub(c);
        let r = wide as u16;
        let mut f = flags::N | (((r >> 8) as u8) & (flags::S | flags::XY));
        if r == 0 {
            f |= flags::Z;
        }
        if i32::from(a & 0x0fff) - i32::from(b & 0x0fff) - (c as i32) < 0 {
            f |= flags::H;
        }
        if (a ^ b) & 0x8000 != 0 && (a ^ r) & 0x8000 != 0 {
            f |= flags::PV;
        }
        if wide & 0x1_0000 != 0 {
            f |= flags::C;
        }
        self.set_f(f);
        r
    }
}
