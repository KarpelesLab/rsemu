//! The MOS 6502 and its relatives — a cycle-accurate interpreter.
//!
//! Covers the NMOS 6502 and the Ricoh RP2A03/RP2A07 in the NES, which is the
//! same core with decimal mode disabled. All 256 opcodes are implemented,
//! undocumented ones included, because real software depends on them
//! (`docs/cpu/6502.md`).
//!
//! # What "cycle-accurate" means here
//!
//! Not "the instruction took five cycles". Every cycle is a read or a write on
//! the guest's [`AddressSpace`], including the dummy reads that page-crossing
//! and read-modify-write instructions perform, and the interpreter has no
//! cycle counter to bump independently of the bus — a cycle is charged
//! *because* an access was made. A device watching the bus sees what hardware
//! would see, which is the whole point on a machine like the NES where reading
//! `$2007` twice is a different program from reading it once.
//!
//! # Assembling one
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::cpu::mos6502::{Config, Mos6502};
//!
//! // 64 KiB of RAM, a reset vector at $fffc, and one LDA #$42.
//! let ram = Arc::new(RamStore::new(0x1_0000));
//! ram.write_u8(0xfffc, 0x00).unwrap();
//! ram.write_u8(0xfffd, 0xc0).unwrap();
//! ram.write_u8(0xc000, 0xa9).unwrap();
//! ram.write_u8(0xc001, 0x42).unwrap();
//!
//! let space = AddressSpace::new("cpu", 16);
//! space.topology().map(Region::ram("ram", ram), 0).unwrap();
//!
//! let cpu = Mos6502::new(Config::default());
//! cpu.attach_space(Arc::new(space));
//! cpu.step();              // the 7-cycle reset sequence
//! cpu.step();              // LDA #$42
//! assert_eq!(cpu.regs().a, 0x42);
//! assert_eq!(cpu.cycles(), 9);
//! ```
//!
//! # Modules
//!
//! | Module | Holds |
//! | --- | --- |
//! | [`isa`] | the one declarative instruction table; decode and disassembly both read it |
//! | [`disasm`] | the disassembler generated from that table |
//! | `exec` (private) | the interpreter: one bus access per cycle |
//!
//! # Sources
//!
//! Hardware documentation only (`ROADMAP.md` §1): the NESdev wiki's *CPU*,
//! *CPU addressing modes*, *CPU interrupts* and *CPU unofficial opcodes*
//! pages, the masswerk and Obelisk instruction references, Bruce Clark's
//! decimal-mode paper on 6502.org, and the W65C02S datasheet for the parts the
//! CMOS successor documents better. Cross-checked against `../gones/cpu6502`,
//! which is ours and MIT (© Mark Karpelès). No copyleft emulator was consulted.

pub mod disasm;
mod exec;
pub mod isa;

#[cfg(test)]
mod tests;

// The conformance runner reads a downloaded corpus off the filesystem, so it
// exists only where there is one (`ROADMAP.md` §12).
#[cfg(all(test, feature = "std"))]
mod conformance;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, Initiator, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU32, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

use exec::{Exec, State};

/// The processor status bits.
///
/// Bit 5 is not a flag: it has no storage and reads as one. Bit 4 is not one
/// either — **B** exists only in the byte an interrupt or `PHP` pushes, which
/// is why `PLP` and `RTI` mask it out again.
pub mod flags {
    /// Carry.
    pub const C: u8 = 0x01;
    /// Zero.
    pub const Z: u8 = 0x02;
    /// Interrupt disable.
    pub const I: u8 = 0x04;
    /// Decimal mode.
    pub const D: u8 = 0x08;
    /// Break — only ever seen on the stack.
    pub const B: u8 = 0x10;
    /// Unused, and always set.
    pub const U: u8 = 0x20;
    /// Overflow.
    pub const V: u8 = 0x40;
    /// Negative.
    pub const N: u8 = 0x80;
}

/// The architectural register file.
///
/// Public and `Copy` because a debugger, a tracer and a test all want to read
/// it out and put it back — this is the surface a future gdbstub serialises
/// (`ROADMAP.md` §9's debug story), and [`Reg`] enumerates it by name.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Regs {
    /// Accumulator.
    pub a: u8,
    /// Index register X.
    pub x: u8,
    /// Index register Y.
    pub y: u8,
    /// Stack pointer; the stack lives at `$0100 + S`.
    pub s: u8,
    /// Processor status. See [`flags`].
    pub p: u8,
    /// Program counter.
    pub pc: u16,
}

impl Regs {
    /// The state a cold power-on leaves behind, *before* the reset sequence.
    ///
    /// The sequence itself decrements S three times and sets **I**, which is
    /// where the familiar `S = $fd` comes from.
    #[must_use]
    pub const fn new() -> Regs {
        Regs {
            a: 0,
            x: 0,
            y: 0,
            s: 0,
            p: flags::U,
            pc: 0,
        }
    }

    /// Whether a status flag is set.
    #[inline]
    #[must_use]
    pub const fn flag(&self, mask: u8) -> bool {
        self.p & mask != 0
    }
}

impl fmt::Display for Regs {
    /// The one-line form a trace log wants: `A:00 X:00 Y:00 P:24 SP:fd PC:c000`.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "A:{:02x} X:{:02x} Y:{:02x} P:{:02x} SP:{:02x} PC:{:04x}",
            self.a, self.x, self.y, self.p, self.s, self.pc
        )
    }
}

/// One named register, for a debugger that works by name or index.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Reg {
    /// Accumulator.
    A,
    /// Index register X.
    X,
    /// Index register Y.
    Y,
    /// Stack pointer.
    S,
    /// Processor status.
    P,
    /// Program counter.
    Pc,
}

impl Reg {
    /// Every register, in the order a debugger should list them.
    pub const ALL: &'static [Reg] = &[Reg::A, Reg::X, Reg::Y, Reg::S, Reg::P, Reg::Pc];

    /// The register's name, lowercase, as gdb and the monitor spell it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Reg::A => "a",
            Reg::X => "x",
            Reg::Y => "y",
            Reg::S => "s",
            Reg::P => "p",
            Reg::Pc => "pc",
        }
    }

    /// How wide the register is.
    #[must_use]
    pub const fn width(self) -> Width {
        match self {
            Reg::Pc => Width::U16,
            _ => Width::U8,
        }
    }

    /// Read this register out of a register file.
    #[must_use]
    pub const fn get(self, regs: &Regs) -> u16 {
        match self {
            Reg::A => regs.a as u16,
            Reg::X => regs.x as u16,
            Reg::Y => regs.y as u16,
            Reg::S => regs.s as u16,
            Reg::P => regs.p as u16,
            Reg::Pc => regs.pc,
        }
    }

    /// Write this register into a register file, truncating to its width.
    pub const fn set(self, regs: &mut Regs, value: u16) {
        match self {
            Reg::A => regs.a = value as u8,
            Reg::X => regs.x = value as u8,
            Reg::Y => regs.y = value as u8,
            Reg::S => regs.s = value as u8,
            Reg::P => regs.p = value as u8,
            Reg::Pc => regs.pc = value,
        }
    }

    /// Look a register up by name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Reg> {
        Reg::ALL.iter().copied().find(|r| r.name() == name)
    }
}

impl fmt::Display for Reg {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// Which interrupt a poll latched.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Interrupt {
    /// Maskable, level-sensitive, vectored through `$fffe`.
    Irq,
    /// Non-maskable, edge-sensitive, vectored through `$fffa`.
    Nmi,
}

/// How this particular part differs from the generic 6502.
///
/// Construction properties, never `#[cfg]`: one build of rsemu has to be able
/// to run a NES *and* an Apple II, and the difference between their CPUs is
/// two values (`docs/cpu/6502.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Whether the part implements decimal mode.
    ///
    /// True on a MOS 6502; **false on the NES's RP2A03**, whose BCD adder was
    /// removed. When false, `ADC` and `SBC` ignore the **D** flag — but the
    /// flag still exists and `SED`/`CLD`/`PHP` still see it.
    pub decimal: bool,
    /// The "magic constant" `ANE` and `LXA` OR into the accumulator.
    ///
    /// These two opcodes are analog: the value depends on the chip series and
    /// its temperature. `$ee` is the value `SingleStepTests/65x02` was
    /// generated with, so it is the default; a machine reproducing a specific
    /// board may want a different one.
    pub magic: u8,
    /// This core's identity in `MemAttrs::requester`, for an IOMMU or a
    /// per-master filter.
    pub requester: RequesterId,
}

impl Config {
    /// A plain NMOS 6502: decimal mode present.
    pub const NMOS_6502: Config = Config {
        decimal: true,
        magic: 0xee,
        requester: RequesterId::ANONYMOUS,
    };

    /// The Ricoh RP2A03 in the NES: the same core without decimal mode.
    pub const RP2A03: Config = Config {
        decimal: false,
        ..Config::NMOS_6502
    };

    /// Same configuration, with a different requester id.
    #[must_use]
    pub const fn with_requester(mut self, id: RequesterId) -> Self {
        self.requester = id;
        self
    }

    /// Same configuration, with a different `ANE`/`LXA` constant.
    #[must_use]
    pub const fn with_magic(mut self, magic: u8) -> Self {
        self.magic = magic;
        self
    }
}

impl Default for Config {
    fn default() -> Self {
        Config::NMOS_6502
    }
}

/// The interrupt input pins, kept outside the execution lock.
///
/// Deliberately atomics rather than fields under the mutex: a device asserting
/// IRQ from inside a write the CPU itself issued would otherwise re-enter the
/// CPU's own critical section, which is a deadlock under `native-std` and a
/// panic under `single`. The re-entrancy contract says mutate your own state
/// in a short critical section and act outward afterwards; a pin that is one
/// atomic store needs no critical section at all (`ROADMAP.md` §4.7).
#[derive(Debug, Default)]
pub(crate) struct Lines {
    /// IRQ is level-sensitive: it is taken whenever it is asserted and **I**
    /// is clear.
    irq: AtomicBool,
    /// The last level seen on NMI, for edge detection.
    nmi_level: AtomicBool,
    /// NMI is edge-sensitive: a high-going edge sets this latch, which stays
    /// set until the interrupt is serviced, however long that takes.
    nmi_latch: AtomicBool,
    /// A `/RES` assertion nobody has acted on yet.
    ///
    /// An atomic rather than `State::reset_pending` for the same reason the
    /// interrupt pins are: the line can be driven from inside a write the CPU
    /// itself issued, and reaching for the execution lock from there would
    /// deadlock. [`Mos6502::step`] moves it into the execution state on its
    /// way past.
    reset_req: AtomicBool,
}

impl Lines {
    fn set_irq(&self, asserted: bool) {
        self.irq.store(asserted, Ordering::Release);
    }

    fn irq_asserted(&self) -> bool {
        self.irq.load(Ordering::Acquire)
    }

    /// Drive the NMI pin, latching a high-going edge.
    fn set_nmi(&self, asserted: bool) {
        let previous = self.nmi_level.swap(asserted, Ordering::AcqRel);
        if asserted && !previous {
            self.nmi_latch.store(true, Ordering::Release);
        }
    }

    fn nmi_pending(&self) -> bool {
        self.nmi_latch.load(Ordering::Acquire)
    }

    /// Consume the NMI latch, reporting whether it was set.
    fn take_nmi_pending(&self) -> bool {
        self.nmi_latch.swap(false, Ordering::AcqRel)
    }

    /// Drop a latched edge without touching the input levels.
    fn clear_nmi_latch(&self) {
        self.nmi_latch.store(false, Ordering::Release);
    }

    /// Latch a reset request. Idempotent: two pulses are one reset.
    fn request_reset(&self) {
        self.reset_req.store(true, Ordering::Release);
    }

    /// Consume the reset request, reporting whether there was one.
    fn take_reset_request(&self) -> bool {
        self.reset_req.swap(false, Ordering::AcqRel)
    }

    fn snapshot(&self) -> (bool, bool, bool, bool) {
        (
            self.irq_asserted(),
            self.nmi_level.load(Ordering::Acquire),
            self.nmi_pending(),
            self.reset_req.load(Ordering::Acquire),
        )
    }

    fn restore(&self, (irq, level, latch, reset): (bool, bool, bool, bool)) {
        self.irq.store(irq, Ordering::Release);
        self.nmi_level.store(level, Ordering::Release);
        self.nmi_latch.store(latch, Ordering::Release);
        self.reset_req.store(reset, Ordering::Release);
    }
}

/// Everything the interpreter needs to mutate, behind one lock.
#[derive(Debug)]
struct Session {
    state: State,
    space: Option<Arc<AddressSpace>>,
}

/// A MOS 6502 core.
///
/// # Locking
///
/// Execution state sits behind one [`sync::Mutex`] at [`LockRank::BUS`]. That
/// rank, rather than `DEVICE`, because a CPU is a bus master: it holds this
/// lock while calling into device models, which take their own `DEVICE`-ranked
/// locks, which drive `WIRE`-ranked lines. The ladder runs in the direction
/// calls travel, so the debug order check passes for the real call graph and
/// fires on an inverted one.
///
/// The interrupt pins are *not* under that lock: they are atomics, so a device
/// asserting IRQ from inside a write the CPU itself issued cannot re-enter the
/// CPU's own critical section.
#[derive(Debug)]
pub struct Mos6502 {
    cfg: Config,
    lines: Arc<Lines>,
    session: sync::Mutex<Session>,
    /// This core's identity in `MemAttrs::requester`.
    ///
    /// Separate from [`Config`] because the machine layer assigns it at bind
    /// time, long after construction, and every method a device has takes
    /// `&self`.
    requester: AtomicU32,
    /// The wire sinks handed out by [`Device::sink`], kept alive here.
    ///
    /// A net holds only a *weak* reference to a sink (`core::device`), so
    /// something has to own the strong one, and it has to be the device — a
    /// pin owned by the net would be an ownership cycle through the wire.
    /// These hold an `Arc<Lines>` rather than an `Arc<Mos6502>` for exactly
    /// that reason.
    pins: sync::Mutex<Pins>,
}

/// The sinks this core has published, one per input pin.
#[derive(Debug, Default)]
struct Pins {
    irq: Option<Arc<InterruptPin>>,
    nmi: Option<Arc<InterruptPin>>,
    reset: Option<Arc<ResetPin>>,
}

impl Mos6502 {
    /// A core in its power-on state, with no address space yet.
    ///
    /// Two-phase construction (`ROADMAP.md` §4.4): nothing observable happens
    /// until [`attach_space`](Mos6502::attach_space) and
    /// [`Device::realize`]. The first [`step`](Mos6502::step) runs the reset
    /// sequence, which is where the reset vector is read.
    #[must_use]
    pub fn new(cfg: Config) -> Mos6502 {
        Mos6502 {
            cfg,
            lines: Arc::new(Lines::default()),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(),
                    space: None,
                },
            ),
            requester: AtomicU32::new(cfg.requester.0),
            pins: sync::Mutex::new(Pins::default()),
        }
    }

    /// The configuration as it stands, with the bind-time requester id folded
    /// in.
    ///
    /// One relaxed load per instruction, which is nothing next to the seven
    /// bus accesses around it, and it keeps the id out of the execution lock.
    fn effective_config(&self) -> Config {
        Config {
            requester: RequesterId(self.requester.load(Ordering::Relaxed)),
            ..self.cfg
        }
    }

    /// Set the id accesses this core initiates carry.
    ///
    /// The machine layer calls this from `bind`; a caller wiring a core up by
    /// hand can call it directly or put the id in [`Config`].
    pub fn set_requester(&self, id: RequesterId) {
        self.requester.store(id.0, Ordering::Relaxed);
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If a property has the wrong type, `magic` does not fit in a byte, or a
    /// property nothing here accepts was given — a typo'd property that was
    /// silently ignored is an afternoon lost.
    pub fn from_props(props: &Props) -> Result<Mos6502> {
        let mut r = props.reader();
        let decimal = r.or("decimal", true)?;
        let magic = r.or_range("magic", 0xeeu64, 0..=0xff)?;
        // Accepted, and for now only one value is: `ROADMAP.md` §5's example
        // writes `engine = "interp"`, and the IR frontend is phase 5. Rejecting
        // the spelling the language documents would be the wrong half to be
        // strict about.
        let _ = r.or_enum("engine", "interp", &["interp"])?;
        r.finish()?;
        Ok(Mos6502::new(Config {
            decimal,
            magic: magic as u8,
            requester: RequesterId::ANONYMOUS,
        }))
    }

    /// This core's configuration.
    #[must_use]
    pub fn config(&self) -> Config {
        self.cfg
    }

    /// Give the core the address space it executes from.
    ///
    /// Separate from construction because the space is built by the machine
    /// assembly layer, which does not exist yet; when `RealizeCtx` grows space
    /// accessors this moves into [`Device::realize`] and the method stays as
    /// the way a test wires one up.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().space = Some(space);
    }

    /// The address space this core executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().space.clone()
    }

    /// The register file.
    #[must_use]
    pub fn regs(&self) -> Regs {
        self.session.lock().state.regs
    }

    /// Overwrite the register file — a debugger, a test vector, a snapshot.
    pub fn set_regs(&self, regs: Regs) {
        self.session.lock().state.regs = regs;
    }

    /// Read one register by name.
    #[must_use]
    pub fn reg(&self, reg: Reg) -> u16 {
        reg.get(&self.session.lock().state.regs)
    }

    /// Write one register by name.
    pub fn set_reg(&self, reg: Reg, value: u16) {
        reg.set(&mut self.session.lock().state.regs, value);
    }

    /// Bus cycles executed since power-on.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// Whether a `JAM` opcode has frozen the core.
    ///
    /// A jammed 6502 stops fetching until reset. [`step`](Mos6502::step)
    /// returns zero cycles once this is true, so a scheduler must notice it
    /// rather than spin.
    #[must_use]
    pub fn is_halted(&self) -> bool {
        self.session.lock().state.halted
    }

    /// Whether a reset sequence is still owed.
    #[must_use]
    pub fn reset_pending(&self) -> bool {
        self.session.lock().state.reset_pending
    }

    /// How many accesses the address space refused, and where the last one
    /// was.
    ///
    /// The NMOS 6502 has no bus-error input, so a refused access cannot raise
    /// an exception: the read returns whatever was last on the data bus, which
    /// is what open-bus hardware does. This counter is how that becomes
    /// visible instead of silent — a machine whose memory map has a hole will
    /// show it climbing.
    #[must_use]
    pub fn bus_faults(&self) -> (u64, u16) {
        let s = self.session.lock();
        (s.state.faults, s.state.last_fault)
    }

    /// Drive the IRQ pin. Level-sensitive: it is taken while asserted and
    /// **I** is clear.
    ///
    /// `asserted` is the logical level, not the pin's: a real `/IRQ` is
    /// active-low, and inverting it belongs to whatever models the wire.
    pub fn set_irq(&self, asserted: bool) {
        self.lines.set_irq(asserted);
    }

    /// Whether IRQ is currently asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.lines.irq_asserted()
    }

    /// Drive the NMI pin. Edge-sensitive: a high-going edge latches, and the
    /// latch survives until the interrupt is taken.
    pub fn set_nmi(&self, asserted: bool) {
        self.lines.set_nmi(asserted);
    }

    /// A complete NMI pulse, for a caller that does not model the pin's level.
    pub fn pulse_nmi(&self) {
        self.lines.set_nmi(true);
        self.lines.set_nmi(false);
    }

    /// Whether an NMI edge is latched and not yet serviced.
    #[must_use]
    pub fn nmi_pending(&self) -> bool {
        self.lines.nmi_pending()
    }

    /// What the most recent interrupt poll latched.
    ///
    /// Set at the start of an instruction's final cycle and serviced before
    /// the next instruction starts, so between steps this is "what happens
    /// next".
    #[must_use]
    pub fn pending_interrupt(&self) -> Option<Interrupt> {
        self.session.lock().state.pending
    }

    /// Request a reset sequence without changing any register.
    ///
    /// The sequence runs on the next [`step`](Mos6502::step), because that is
    /// when the CPU can read the reset vector — a reset is a signal, not a
    /// method call.
    pub fn request_reset(&self) {
        self.session.lock().state.reset_pending = true;
    }

    /// Execute one reset sequence, interrupt sequence, or instruction.
    ///
    /// Returns the bus cycles charged: zero if the core is halted or has no
    /// address space, which the caller must treat as "stop", not "retry".
    pub fn step(&self) -> u64 {
        let cfg = self.effective_config();
        let mut session = self.session.lock();
        let Session { state, space } = &mut *session;
        // A `/RES` assertion is latched outside the lock; this is where it
        // becomes execution state. A jammed core wakes up, which is what the
        // pin is for.
        if self.lines.take_reset_request() {
            state.reset_pending = true;
            state.halted = false;
            state.pending = None;
        }
        let Some(space) = space.clone() else {
            return 0;
        };
        Exec::new(state, &space, &cfg, &self.lines).step()
    }

    /// Execute until at least `budget` cycles have been charged.
    ///
    /// Returns the cycles actually used, which overshoots by at most one
    /// instruction — the 6502 cannot be stopped mid-instruction, and
    /// pretending otherwise is how a scheduler ends up with a CPU in an
    /// impossible state. Stops early if the core halts.
    pub fn run(&self, budget: u64) -> u64 {
        let mut used = 0;
        while used < budget {
            let n = self.step();
            if n == 0 {
                break;
            }
            used += n;
        }
        used
    }

    /// Run a scheduler budget of `ticks` cycles, reporting exactly how many
    /// were consumed — never more.
    ///
    /// A 6502 cannot be stopped mid-instruction, so the last instruction of a
    /// budget usually runs past its end. The scheduler treats an overrun as
    /// fatal, and rightly: the overrun has already executed past an event that
    /// should have stopped it. So the overshoot is *carried* — deducted from
    /// the next budget through `State::debt` — which keeps the core's cycle
    /// count and the domain's tick count in step over any number of quanta
    /// while never letting a single one overrun.
    ///
    /// A halted core consumes only the debt it owed plus whatever it managed,
    /// which is how the scheduler sees a `JAM` rather than spinning on it.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            // The last instruction was longer than this whole budget: charge
            // the budget against the debt and execute nothing.
            self.session.lock().state.debt = owed - ticks;
            return ticks;
        }
        let allowance = ticks - owed;
        let mut used = 0u64;
        while used < allowance {
            let n = self.step();
            if n == 0 {
                // Halted, or no address space. Either way, stop — retrying
                // would spin.
                break;
            }
            used += n;
        }
        if used >= allowance {
            self.session.lock().state.debt = used - allowance;
            ticks
        } else {
            self.session.lock().state.debt = 0;
            owed + used
        }
    }

    /// Cycles owed to the next budget — see [`run_budget`](Mos6502::run_budget).
    #[must_use]
    pub fn cycle_debt(&self) -> u64 {
        self.session.lock().state.debt
    }

    /// Disassemble `count` instructions starting at `pc`, reading guest memory
    /// with debug attributes.
    ///
    /// Debug attributes are the point: a monitor listing the code around PC
    /// must not pop a FIFO or clear a status bit on the way (`ROADMAP.md` §15,
    /// invariant 5).
    #[must_use]
    pub fn disassemble(&self, pc: u16, count: usize) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        disasm::disassemble_run(pc, count, |addr| {
            space
                .read(u64::from(addr), Width::U8, MemAttrs::DEBUG)
                .ok()
                .map(|v| v as u8)
        })
    }
}

/// The `cpu.mos6502` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.mos6502",
    // v2 added the cycle debt `run_budget` carries between quanta and the
    // latched `/RES` request; v1 snapshots predate any released build.
    version: 2,
    summary: "MOS 6502 / Ricoh RP2A03 8-bit CPU core, cycle-accurate interpreter",
    properties: &[
        PropertySpec {
            name: "decimal",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether the part has decimal mode; false for the NES's RP2A03",
        },
        PropertySpec {
            name: "magic",
            kind: ValueKind::Uint,
            required: false,
            summary: "the analog constant ANE and LXA OR into the accumulator (default 0xee)",
        },
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine; only `interp` exists until phase 5",
        },
    ],
    construct: |props| Ok(Box::new(Mos6502::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4), so the machine assembly layer calls this from its own
/// `#[cfg(feature = "cpu-mos6502")]` arm.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for Mos6502 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. A CPU with no address space cannot fetch, but
        // realize runs *before* the machine binds one — that check belongs to
        // `Instance::bind`, which is where the space arrives.
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // The fan-in can only be built now: it is told its sources at
        // construction and no `WireId` existed when this core was made.
        //
        // Every pin is named the way the package names it, minus the bar:
        // `/IRQ` is asserted low on real silicon, and inverting a level
        // belongs to whatever models the wire, not to the core.
        let mut pins = self.pins.lock();
        let sink: Arc<dyn WireSink> = match port {
            "irq" => {
                let pin = Arc::new(InterruptPin::from_lines(
                    Arc::clone(&self.lines),
                    Interrupt::Irq,
                    sources,
                ));
                pins.irq = Some(Arc::clone(&pin));
                pin
            }
            "nmi" => {
                let pin = Arc::new(InterruptPin::from_lines(
                    Arc::clone(&self.lines),
                    Interrupt::Nmi,
                    sources,
                ));
                pins.nmi = Some(Arc::clone(&pin));
                pin
            }
            "reset" => {
                let pin = Arc::new(ResetPin::new(Arc::clone(&self.lines), sources));
                pins.reset = Some(Arc::clone(&pin));
                pin
            }
            _ => return None,
        };
        Some(SinkPin { sink, line: 0 })
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }

    fn reset(&self, kind: ResetKind) {
        let mut session = self.session.lock();
        if kind == ResetKind::Cold {
            // A cold start has no defined register contents on real hardware;
            // zeroing them is the reproducible choice, and determinism is a
            // first-class mode (`ROADMAP.md` §0).
            session.state = State::new();
        } else {
            // A warm reset is a pulse on the RES pin: registers keep their
            // values, and only the sequence's own effects apply.
            session.state.reset_pending = true;
            session.state.halted = false;
            session.state.pending = None;
        }
        drop(session);
        if kind == ResetKind::Cold {
            self.lines.restore((false, false, false, false));
        } else {
            // The input *levels* belong to whatever drives them, not to the
            // CPU — clearing them here would make a reset lie about the
            // machine. The edge latch is internal, so it goes.
            self.lines.clear_nmi_latch();
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.session.lock().state;
        w.write_u8(state.regs.a)?;
        w.write_u8(state.regs.x)?;
        w.write_u8(state.regs.y)?;
        w.write_u8(state.regs.s)?;
        w.write_u8(state.regs.p)?;
        w.write_u16(state.regs.pc)?;
        w.write_u64(state.cycles)?;
        w.write_bool(state.halted)?;
        w.write_bool(state.reset_pending)?;
        w.write_u8(match state.pending {
            None => 0,
            Some(Interrupt::Irq) => 1,
            Some(Interrupt::Nmi) => 2,
        })?;
        w.write_u8(state.open_bus)?;
        w.write_u64(state.faults)?;
        w.write_u16(state.last_fault)?;
        w.write_u64(state.debt)?;
        let (irq, nmi_level, nmi_latch, reset_req) = self.lines.snapshot();
        w.write_bool(irq)?;
        w.write_bool(nmi_level)?;
        w.write_bool(nmi_latch)?;
        w.write_bool(reset_req)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new();
        state.regs.a = r.read_u8()?;
        state.regs.x = r.read_u8()?;
        state.regs.y = r.read_u8()?;
        state.regs.s = r.read_u8()?;
        state.regs.p = r.read_u8()?;
        state.regs.pc = r.read_u16()?;
        state.cycles = r.read_u64()?;
        state.halted = r.read_bool()?;
        state.reset_pending = r.read_bool()?;
        state.pending = match r.read_u8()? {
            0 => None,
            1 => Some(Interrupt::Irq),
            2 => Some(Interrupt::Nmi),
            other => {
                return Err(Error::State(alloc::format!(
                    "unknown pending interrupt tag {other}"
                )));
            }
        };
        state.open_bus = r.read_u8()?;
        state.faults = r.read_u64()?;
        state.last_fault = r.read_u16()?;
        state.debt = r.read_u64()?;
        let irq = r.read_bool()?;
        let nmi_level = r.read_bool()?;
        let nmi_latch = r.read_bool()?;
        let reset_req = r.read_bool()?;
        self.session.lock().state = state;
        self.lines.restore((irq, nmi_level, nmi_latch, reset_req));
        Ok(())
    }
}

impl Initiator for Mos6502 {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

/// The machine layer's half: a 6502 needs an address space, and this is where
/// the machine gives it one.
impl crate::machine::Instance for Mos6502 {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        // A CPU with no address space cannot fetch, and a machine that runs
        // zero instructions and says nothing is the worst of both worlds.
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: alloc::string::String::from(ctx.path()),
            message: alloc::string::String::from(
                "a 6502 needs an address space to fetch from (`space = cpubus`)",
            ),
        })?;
        self.attach_space(Arc::clone(space));
        self.set_requester(ctx.requester());
        Ok(())
    }
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| {
        Ok(Arc::new(Mos6502::from_props(props)?))
    })
}

/// What the validator should know about `cpu.mos6502`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("decimal", ValueKind::Bool))
        .prop(PropSchema::new("magic", ValueKind::Uint).range(0, 0xff))
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        // Inputs only: a 6502 drives no line this core models. `/SYNC` and
        // `R/W` are real pins, but nothing yet listens to either.
        .port("irq", PortDir::In)
        .port("nmi", PortDir::In)
        .port("reset", PortDir::In)
}

/// One of the CPU's two interrupt inputs, as something a [`Wire`] can drive.
///
/// A wire hands each sink the level of the *driver that changed*, not the
/// resolved level of the net, because a net with several drivers is resolved
/// by whoever cares. The NES's IRQ line has three (APU frame counter, DMC, and
/// the cartridge), so this keeps a [`FanIn`] and wire-ORs them — which is what
/// the open-collector line does in hardware.
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct InterruptPin {
    lines: Arc<Lines>,
    which: Interrupt,
    inputs: FanIn,
    resolve: Resolve,
}

impl InterruptPin {
    /// Connect `which` pin of `cpu` to a net driven by `sources`.
    ///
    /// Wire-OR by default: any source asserting asserts the pin, which is how
    /// an open-collector interrupt line behaves.
    ///
    /// The pin keeps a handle on the core's *input latches*, not on the core:
    /// the core owns the pin (something must, since a net holds only a weak
    /// reference to its sinks), and a pin that owned the core back would be a
    /// cycle the machine could never drop.
    #[must_use]
    pub fn new(cpu: Arc<Mos6502>, which: Interrupt, sources: &[WireId]) -> InterruptPin {
        InterruptPin::from_lines(Arc::clone(&cpu.lines), which, sources)
    }

    /// The same, given the latches directly.
    fn from_lines(lines: Arc<Lines>, which: Interrupt, sources: &[WireId]) -> InterruptPin {
        InterruptPin {
            lines,
            which,
            inputs: FanIn::new(sources),
            resolve: Resolve::Or,
        }
    }

    /// The same pin with an explicit resolution rule.
    #[must_use]
    pub fn with_resolve(mut self, resolve: Resolve) -> Self {
        self.resolve = resolve;
        self
    }

    /// Which pin this is.
    #[must_use]
    pub fn which(&self) -> Interrupt {
        self.which
    }

    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for InterruptPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let asserted = self.inputs.resolve(self.resolve).is_high();
        match self.which {
            Interrupt::Irq => self.lines.set_irq(asserted),
            Interrupt::Nmi => self.lines.set_nmi(asserted),
        }
    }
}

/// The core's `/RES` input, as something a [`Wire`] can drive.
///
/// Separate from [`InterruptPin`] because a reset is not an interrupt: it has
/// no vector to poll for, no mask, and it un-jams a core that a `JAM` opcode
/// froze. Asserting the line latches a request; the sequence itself runs on the
/// next [`step`](Mos6502::step), because that is when the CPU can read the
/// reset vector — a reset is a signal, not a method call.
///
/// Wire-OR, like the interrupt pins: any source holding the line asserts it.
///
/// [`Wire`]: crate::core::wire::Wire
#[derive(Debug)]
pub struct ResetPin {
    lines: Arc<Lines>,
    inputs: FanIn,
    resolve: Resolve,
}

impl ResetPin {
    /// Connect the `/RES` pin of `cpu` to a net driven by `sources`.
    #[must_use]
    pub fn new_for(cpu: Arc<Mos6502>, sources: &[WireId]) -> ResetPin {
        ResetPin::new(Arc::clone(&cpu.lines), sources)
    }

    fn new(lines: Arc<Lines>, sources: &[WireId]) -> ResetPin {
        ResetPin {
            lines,
            inputs: FanIn::new(sources),
            resolve: Resolve::Or,
        }
    }

    /// The per-source levels currently seen.
    #[must_use]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }
}

impl WireSink for ResetPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // Latch on assertion rather than on release. A real 6502 samples `/RES`
        // and begins its sequence once the line goes back high, but the
        // difference is invisible to a guest — nothing executes while the line
        // is held — and latching on the edge that *arrives* means a machine
        // whose reset button is still down still comes up, instead of waiting
        // for a release nobody modelled.
        if self.inputs.resolve(self.resolve).is_high() {
            self.lines.request_reset();
        }
    }
}

/// A description of this core for `rsemu describe cpu.mos6502`.
///
/// Built from [`isa::TABLE`], so it cannot drift from what the interpreter
/// implements.
#[must_use]
pub fn describe_isa() -> String {
    use core::fmt::Write as _;
    let mut out = String::new();
    for opcode in 0..=255u8 {
        let insn = isa::decode(opcode);
        let mark = match insn.class {
            isa::Class::Documented => ' ',
            isa::Class::Undocumented => '*',
            isa::Class::Unstable => '!',
        };
        let _ = writeln!(
            out,
            "{opcode:02x} {mark}{:<4} {:<6} {}",
            insn.op.mnemonic(),
            insn.mode.name(),
            insn.op.summary()
        );
    }
    out
}
