//! The WDC W65C816S — a cycle-accurate interpreter.
//!
//! The 16-bit successor to the 65C02, and the CPU of the SNES (as the core of
//! Ricoh's 5A22) and the Apple IIGS. A separate core rather than a
//! `cpu::mos6502` variant: it has 16-bit A, X, Y, S and D, a 24-bit address
//! bus with separate program and data banks, the E flag that switches a
//! whole 6502 personality on and off, accumulator and index widths that
//! change the *length* of an instruction, and bank-wrapping rules that differ
//! per addressing mode. Sharing an interpreter with the 6502 would mean
//! sharing none of the parts that are hard.
//!
//! # What "cycle-accurate" means here
//!
//! Every cycle the datasheet's table 5-7 lists is made, in order: a read, a
//! write, or an **internal operation** — a cycle with VDA and VPA both low,
//! on which no memory is selected but time passes and an address is still
//! driven. Each one is charged through [`AccessTiming`], which the board
//! supplies: on a SNES a cycle costs 6, 8 or 12 master clocks depending on
//! the address it drives, and that is the board's knowledge, not the CPU's.
//! With no provider every cycle costs one clock of the core's domain, so a
//! generic 65C816 machine runs at one cycle per tick.
//!
//! The pins that classify a cycle — VDA, VPA, VPB, RWB, E, M/X and MLB — are
//! reported in [`BusCycle::pins`], which is how a board tells an opcode fetch
//! from a data read from an internal cycle.
//!
//! # Interrupts
//!
//! | Input | Sensitivity | Notes |
//! | --- | --- | --- |
//! | `irq` | level | masked by I; releases `WAI` even when masked |
//! | `nmi` | edge | latched on assertion, sampled once per cycle |
//! | `abort` | edge | the instruction in progress completes its bus cycles but its register changes are undone, and the ABORT vector is taken with the aborted opcode as the return address |
//! | `reset` | edge | runs the reset sequence and wakes `STP` |
//!
//! `WAI` stops the core with RDY low until an interrupt line moves; `STP`
//! stops the clock until reset. `/RDY` is the [`CycleGate`] a DMA unit
//! publishes, as on the 6502: the SNES's DMA halts the CPU through it.
//!
//! # Open bus
//!
//! The 65C816 has no bus-error input. A read nothing answers — an address the
//! space refuses, or one whose region does not drive the data bus — returns
//! the last value on the data bus, which on a SNES is the "MDR" that real
//! software reads back. It includes the operand bytes of the instruction in
//! progress, since those were on the bus too. [`W65c816::bus_faults`] counts
//! the refused accesses so a hole in a memory map stays visible.
//!
//! # Assembling one
//!
//! ```
//! use std::sync::Arc;
//! use rsemu::core::space::{AddressSpace, RamStore, Region};
//! use rsemu::cpu::w65c816::W65c816;
//!
//! // 64 KiB of RAM in bank 0: a reset vector at $fffc, then CLC; XCE;
//! // REP #$20; LDA #$1234.
//! let ram = Arc::new(RamStore::new(0x1_0000));
//! for (i, b) in [0x18, 0xfb, 0xc2, 0x20, 0xa9, 0x34, 0x12].iter().enumerate() {
//!     ram.write_u8(0x8000 + i as u64, *b).unwrap();
//! }
//! ram.write_u8(0xfffd, 0x80).unwrap();
//!
//! let space = AddressSpace::new("cpu", 24);
//! space.topology().map(Region::ram("ram", ram), 0).unwrap();
//!
//! let cpu = W65c816::new();
//! cpu.attach_space(Arc::new(space));
//! for _ in 0..5 {
//!     cpu.step(); // reset, CLC, XCE, REP, LDA
//! }
//! assert!(!cpu.regs().e);
//! assert_eq!(cpu.regs().a, 0x1234);
//! ```
//!
//! # Sources
//!
//! Hardware documentation only (CLAUDE.md, *Provenance*): the **W65C816S
//! datasheet** (Western Design Center) — pin descriptions §2, addressing
//! modes §4, the opcode matrix and table 5-7's cycle-by-cycle operation §5,
//! the vectors in table 6-1, the caveats in §7; Eyes & Lichty, *Programming
//! the 65816*; Bruce Clark's *65C816 opcodes* tutorial and decimal-mode notes
//! on 6502.org. `SingleStepTests/65816` (MIT, Thomas Harte et al.) is the
//! oracle for the bus traces. No emulator source was consulted. See
//! `docs/cpu/65816.md`.

pub mod disasm;
mod exec;
pub mod isa;

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    CycleGate, Device, DeviceClass, ExportId, Initiator, PropertySpec, RealizeCtx, ResetKind,
    SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::sched::{Budget, Consumed, ExitFlag, TickCursor};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{self, AtomicBool, AtomicU32, LockRank, Ordering};
use crate::core::value::Width;
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};

use exec::{Exec, State};

/// The processor status bits.
///
/// Bits 4 and 5 are X and M in native mode. In emulation mode both are held
/// set, and bit 4 of a byte an interrupt pushes is the 6502's B instead.
pub mod flags {
    /// Carry.
    pub const C: u8 = 0x01;
    /// Zero.
    pub const Z: u8 = 0x02;
    /// Interrupt disable.
    pub const I: u8 = 0x04;
    /// Decimal mode.
    pub const D: u8 = 0x08;
    /// Index registers are 8 bits (native mode).
    pub const X: u8 = 0x10;
    /// Break, in a byte pushed in emulation mode.
    pub const B: u8 = 0x10;
    /// Accumulator and memory are 8 bits (native mode).
    pub const M: u8 = 0x20;
    /// Overflow.
    pub const V: u8 = 0x40;
    /// Negative.
    pub const N: u8 = 0x80;
}

/// The architectural register file.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Regs {
    /// The accumulator, both bytes (C); with M set only the low byte (A) is
    /// operated on and the high byte (B) is kept.
    pub a: u16,
    /// Index X. Its high byte is zero whenever X is 8 bits.
    pub x: u16,
    /// Index Y. Its high byte is zero whenever X is 8 bits.
    pub y: u16,
    /// Stack pointer. `$01xx` in emulation mode.
    pub s: u16,
    /// Direct register: the base of the direct page.
    pub d: u16,
    /// Program counter, within the program bank.
    pub pc: u16,
    /// Program bank register.
    pub pbr: u8,
    /// Data bank register.
    pub dbr: u8,
    /// Processor status. See [`flags`].
    pub p: u8,
    /// Emulation mode.
    pub e: bool,
}

impl Regs {
    /// The power-on state before the reset sequence: emulation mode.
    #[must_use]
    pub const fn new() -> Regs {
        Regs {
            a: 0,
            x: 0,
            y: 0,
            s: 0x01ff,
            d: 0,
            pc: 0,
            pbr: 0,
            dbr: 0,
            p: flags::M | flags::X | flags::I,
            e: true,
        }
    }

    /// Force the register file into a shape the hardware can hold.
    ///
    /// Emulation mode pins M and X set and S's high byte at `$01`, and an
    /// 8-bit index register has no high byte (datasheet §2.8, §2.21). A
    /// debugger or a test vector can write anything; this is what the chip
    /// would actually contain.
    pub fn normalize(&mut self) {
        if self.e {
            self.p |= flags::M | flags::X;
            self.s = 0x0100 | (self.s & 0xff);
        }
        if self.p & flags::X != 0 {
            self.x &= 0xff;
            self.y &= 0xff;
        }
    }

    /// The 24-bit address of the next instruction.
    #[must_use]
    pub const fn pc24(&self) -> u32 {
        ((self.pbr as u32) << 16) | self.pc as u32
    }
}

impl fmt::Display for Regs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "A:{:04x} X:{:04x} Y:{:04x} S:{:04x} D:{:04x} DB:{:02x} P:{:02x} E:{} PC:{:02x}:{:04x}",
            self.a,
            self.x,
            self.y,
            self.s,
            self.d,
            self.dbr,
            self.p,
            u8::from(self.e),
            self.pbr,
            self.pc
        )
    }
}

/// Which interrupt a poll latched.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Interrupt {
    /// Maskable, level-sensitive.
    Irq,
    /// Non-maskable, edge-sensitive.
    Nmi,
    /// The ABORTB input: the instruction it interrupted was rolled back.
    Abort,
}

/// The bus signals one cycle drives, as the W65C816S pins name them.
///
/// A bit set means the signal is *active*, whatever its electrical polarity:
/// [`VPB`](Pins::VPB) set means the active-low VPB pin is low.
///
/// | VDA | VPA | Cycle |
/// | --- | --- | --- |
/// | 1 | 1 | opcode fetch |
/// | 0 | 1 | operand (program) fetch |
/// | 1 | 0 | data access |
/// | 0 | 0 | internal operation: no memory is selected |
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Pins(pub u16);

impl Pins {
    /// Valid data address.
    pub const VDA: u16 = 0x01;
    /// Valid program address.
    pub const VPA: u16 = 0x02;
    /// Vector pull: one of the two vector bytes of an interrupt.
    pub const VPB: u16 = 0x04;
    /// RWB low: a write.
    pub const WRITE: u16 = 0x08;
    /// The E output: emulation mode.
    pub const E: u16 = 0x10;
    /// The M flag, as the MX output shows it during φ2-low.
    pub const M: u16 = 0x20;
    /// The X flag, as the MX output shows it during φ2-high.
    pub const X: u16 = 0x40;
    /// Memory lock: a read-modify-write is in progress.
    pub const ML: u16 = 0x80;
    /// Not a pin: `/RDY` is holding this cycle and the core drives nothing new.
    pub const HELD: u16 = 0x100;

    /// Whether `bit` is set.
    #[inline]
    #[must_use]
    pub const fn has(self, bit: u16) -> bool {
        self.0 & bit != 0
    }

    /// An internal operation: neither VDA nor VPA, so no memory is selected.
    #[inline]
    #[must_use]
    pub const fn is_internal(self) -> bool {
        self.0 & (Pins::VDA | Pins::VPA) == 0
    }

    /// A write cycle.
    #[inline]
    #[must_use]
    pub const fn is_write(self) -> bool {
        self.has(Pins::WRITE)
    }

    /// An opcode fetch: VDA and VPA both high.
    #[inline]
    #[must_use]
    pub const fn is_opcode_fetch(self) -> bool {
        self.0 & (Pins::VDA | Pins::VPA) == Pins::VDA | Pins::VPA
    }
}

/// One bus cycle, as [`AccessTiming`] is asked about it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BusCycle {
    /// The 24-bit address driven, `BA0-BA7` on top of `A0-A15`. Driven on
    /// internal cycles too; the datasheet's table 5-7 says which address.
    pub addr: u32,
    /// The signals that classify the cycle.
    pub pins: Pins,
    /// The byte the core is about to drive, on a write cycle — including the
    /// emulation-mode modify cycle, which drives the old value with nothing
    /// selected. `None` on a read or an internal operation.
    pub data: Option<u8>,
}

/// The board's answer to "how long does this cycle take?".
///
/// The SNES's 5A22 runs a 65C816 core whose cycles are 6, 8 or 12 master
/// clocks long depending on the address (FastROM, SlowROM, the joypad
/// registers), with internal operations at 6 — a fact about the board's
/// address decoder, not about the CPU. So the CPU asks.
///
/// Published by a board device as [`ExportId::ACCESS_TIMING`], in an
/// [`Export::Opaque`](crate::core::device::Export::Opaque) holding a
/// [`TimingPort`], and named by the core's `timing` property.
///
/// # The contract
///
/// - Called **once per cycle, before the access**, from inside the core's
///   execution lock — so it may take its own (device-ranked) locks, and must
///   not call back into the CPU.
/// - Returns how many clocks of the core's domain the cycle costs; zero is
///   treated as one, so a buggy provider cannot stop time.
/// - Cycles `/RDY` holds are asked about too, with [`Pins::HELD`] set.
/// - The core's published [`TickCursor`] already includes the returned cost
///   when the access reaches the bus, so a device catching itself up sees
///   the end of the cycle.
pub trait AccessTiming: Send + Sync + fmt::Debug {
    /// Clocks the cycle costs.
    fn clocks(&self, cycle: BusCycle) -> u64;
}

/// The payload [`ExportId::ACCESS_TIMING`] travels in.
///
/// A concrete wrapper because an opaque export is downcast to a concrete
/// type, and the provider behind it is a trait object.
#[derive(Debug, Clone)]
pub struct TimingPort(pub Arc<dyn AccessTiming>);

/// The interrupt and reset inputs, kept outside the execution lock for the
/// same reason `cpu::mos6502` keeps its own there: a device asserting a line
/// from inside an access the CPU made must not re-enter the CPU's lock.
#[derive(Debug, Default)]
pub(crate) struct Lines {
    irq: AtomicBool,
    nmi_level: AtomicBool,
    nmi_sampled: AtomicBool,
    nmi_latch: AtomicBool,
    abort_level: AtomicBool,
    abort_sampled: AtomicBool,
    abort_latch: AtomicBool,
    reset_req: AtomicBool,
}

impl Lines {
    fn irq_asserted(&self) -> bool {
        self.irq.load(Ordering::Acquire)
    }

    /// Clock both edge detectors: once per cycle, and nowhere else.
    fn sample_edges(&self) {
        let now = self.nmi_level.load(Ordering::Acquire);
        if now && !self.nmi_sampled.swap(now, Ordering::AcqRel) {
            self.nmi_latch.store(true, Ordering::Release);
        } else {
            self.nmi_sampled.store(now, Ordering::Release);
        }
        let now = self.abort_level.load(Ordering::Acquire);
        if now && !self.abort_sampled.swap(now, Ordering::AcqRel) {
            self.abort_latch.store(true, Ordering::Release);
        } else {
            self.abort_sampled.store(now, Ordering::Release);
        }
    }

    fn nmi_pending(&self) -> bool {
        self.nmi_latch.load(Ordering::Acquire)
    }

    fn take_nmi_pending(&self) -> bool {
        self.nmi_latch.swap(false, Ordering::AcqRel)
    }

    fn abort_pending(&self) -> bool {
        self.abort_latch.load(Ordering::Acquire)
    }

    fn take_abort_pending(&self) -> bool {
        self.abort_latch.swap(false, Ordering::AcqRel)
    }

    fn take_reset_request(&self) -> bool {
        self.reset_req.swap(false, Ordering::AcqRel)
    }

    fn snapshot(&self) -> [bool; 6] {
        [
            self.irq.load(Ordering::Acquire),
            self.nmi_level.load(Ordering::Acquire),
            self.nmi_latch.load(Ordering::Acquire),
            self.abort_level.load(Ordering::Acquire),
            self.abort_latch.load(Ordering::Acquire),
            self.reset_req.load(Ordering::Acquire),
        ]
    }

    fn restore(&self, [irq, nmi, nmi_latch, abort, abort_latch, reset]: [bool; 6]) {
        self.irq.store(irq, Ordering::Release);
        self.nmi_level.store(nmi, Ordering::Release);
        // The detectors come back standing on the level they were saved at,
        // so a restore does not manufacture an edge out of the load itself.
        self.nmi_sampled.store(nmi, Ordering::Release);
        self.nmi_latch.store(nmi_latch, Ordering::Release);
        self.abort_level.store(abort, Ordering::Release);
        self.abort_sampled.store(abort, Ordering::Release);
        self.abort_latch.store(abort_latch, Ordering::Release);
        self.reset_req.store(reset, Ordering::Release);
    }
}

#[derive(Debug)]
struct Session {
    state: State,
    space: Option<Arc<AddressSpace>>,
}

/// What the machine layer wires onto a core after it is built.
#[derive(Debug, Default, Clone)]
struct CoreLinks {
    cursor: Option<TickCursor>,
    rdy: Option<Arc<dyn CycleGate>>,
    timing: Option<Arc<dyn AccessTiming>>,
    rdy_link: Option<String>,
    timing_link: Option<String>,
}

/// One input pin, as something a wire can drive. Wire-OR over its sources.
#[derive(Debug)]
struct InputPin {
    lines: Arc<Lines>,
    which: Input,
    inputs: FanIn,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Input {
    Irq,
    Nmi,
    Abort,
    Reset,
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let on = self.inputs.resolve(Resolve::Or).is_high();
        let l = &self.lines;
        match self.which {
            Input::Irq => l.irq.store(on, Ordering::Release),
            Input::Nmi => l.nmi_level.store(on, Ordering::Release),
            Input::Abort => l.abort_level.store(on, Ordering::Release),
            Input::Reset => {
                if on {
                    l.reset_req.store(true, Ordering::Release);
                }
            }
        }
    }
}

/// A WDC W65C816S core.
///
/// Locked the way `cpu::mos6502` is: execution state behind one
/// [`sync::Mutex`] at [`LockRank::BUS`], because the core holds it while
/// calling into devices; the input pins as atomics outside it.
#[derive(Debug)]
pub struct W65c816 {
    lines: Arc<Lines>,
    session: sync::Mutex<Session>,
    requester: AtomicU32,
    /// The sinks handed out by [`Device::sink`], kept alive here because a
    /// net holds only a weak reference.
    pins: sync::Mutex<Vec<Arc<InputPin>>>,
    links: sync::Mutex<CoreLinks>,
}

impl Default for W65c816 {
    fn default() -> Self {
        W65c816::new()
    }
}

impl W65c816 {
    /// A core in its power-on state, with no address space yet. The first
    /// [`step`](W65c816::step) runs the reset sequence.
    #[must_use]
    pub fn new() -> W65c816 {
        W65c816 {
            lines: Arc::new(Lines::default()),
            session: sync::Mutex::with_rank(
                LockRank::BUS,
                Session {
                    state: State::new(),
                    space: None,
                },
            ),
            requester: AtomicU32::new(RequesterId::ANONYMOUS.0),
            pins: sync::Mutex::new(Vec::new()),
            links: sync::Mutex::new(CoreLinks::default()),
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// A property of the wrong type, or one nothing here accepts.
    pub fn from_props(props: &Props) -> Result<W65c816> {
        let mut r = props.reader();
        let _ = r.or_enum("engine", "interp", &["interp"])?;
        let rdy = r.optional_link("rdy")?.map(|l| String::from(l.as_str()));
        let timing = r.optional_link("timing")?.map(|l| String::from(l.as_str()));
        r.finish()?;
        let cpu = W65c816::new();
        {
            let mut links = cpu.links.lock();
            links.rdy_link = rdy;
            links.timing_link = timing;
        }
        Ok(cpu)
    }

    /// Set the id accesses this core initiates carry.
    pub fn set_requester(&self, id: RequesterId) {
        self.requester.store(id.0, Ordering::Relaxed);
    }

    /// Give the core the address space it executes from: 24 bits wide, one
    /// space for program and data alike.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        self.session.lock().space = Some(space);
    }

    /// The address space this core executes from, if one is attached.
    #[must_use]
    pub fn space(&self) -> Option<Arc<AddressSpace>> {
        self.session.lock().space.clone()
    }

    /// Publish this core's clock counter here as it executes.
    pub fn attach_cursor(&self, cursor: TickCursor) {
        self.links.lock().cursor = Some(cursor);
    }

    /// Connect `/RDY` to an arbiter that may hold this core.
    pub fn attach_rdy(&self, gate: Arc<dyn CycleGate>) {
        self.links.lock().rdy = Some(gate);
    }

    /// Have `timing` say what each cycle costs. Without one, every cycle is
    /// one clock.
    pub fn attach_timing(&self, timing: Arc<dyn AccessTiming>) {
        self.links.lock().timing = Some(timing);
    }

    /// The register file.
    #[must_use]
    pub fn regs(&self) -> Regs {
        self.session.lock().state.regs
    }

    /// Overwrite the register file, [normalised](Regs::normalize) to what the
    /// hardware can hold.
    pub fn set_regs(&self, mut regs: Regs) {
        regs.normalize();
        self.session.lock().state.regs = regs;
    }

    /// Bus cycles executed since power-on.
    #[must_use]
    pub fn cycles(&self) -> u64 {
        self.session.lock().state.cycles
    }

    /// Clocks of the core's domain since power-on.
    #[must_use]
    pub fn clocks(&self) -> u64 {
        self.session.lock().state.clocks
    }

    /// Whether `STP` has stopped the core. Only a reset restarts it.
    #[must_use]
    pub fn is_halted(&self) -> bool {
        self.session.lock().state.halted
    }

    /// Whether `WAI` is waiting for an interrupt.
    #[must_use]
    pub fn is_waiting(&self) -> bool {
        self.session.lock().state.waiting
    }

    /// Whether a reset sequence is still owed.
    #[must_use]
    pub fn reset_pending(&self) -> bool {
        self.session.lock().state.reset_pending
    }

    /// How many accesses the space refused, and where the last one was.
    #[must_use]
    pub fn bus_faults(&self) -> (u64, u32) {
        let s = self.session.lock();
        (s.state.faults, s.state.last_fault)
    }

    /// The last value on the data bus: what an open-bus read returns.
    #[must_use]
    pub fn open_bus(&self) -> u8 {
        self.session.lock().state.open_bus
    }

    /// Drive IRQB. Level-sensitive; `asserted` is the logical level.
    pub fn set_irq(&self, asserted: bool) {
        self.lines.irq.store(asserted, Ordering::Release);
    }

    /// Drive NMIB. Edge-sensitive: an assertion latches.
    pub fn set_nmi(&self, asserted: bool) {
        self.lines.nmi_level.store(asserted, Ordering::Release);
    }

    /// A complete NMI pulse the edge detector is guaranteed to see.
    pub fn pulse_nmi(&self) {
        self.set_nmi(true);
        self.lines.sample_edges();
        self.set_nmi(false);
    }

    /// Drive ABORTB. Edge-sensitive: an assertion aborts the instruction in
    /// progress (or the next one, between instructions).
    pub fn set_abort(&self, asserted: bool) {
        self.lines.abort_level.store(asserted, Ordering::Release);
    }

    /// Whether an NMI edge is latched and not yet serviced.
    #[must_use]
    pub fn nmi_pending(&self) -> bool {
        self.lines.nmi_pending()
    }

    /// What the most recent poll latched, to be serviced next.
    #[must_use]
    pub fn pending_interrupt(&self) -> Option<Interrupt> {
        self.session.lock().state.pending
    }

    /// Request a reset sequence on the next step.
    pub fn request_reset(&self) {
        self.lines.reset_req.store(true, Ordering::Release);
    }

    /// Execute one reset sequence, interrupt sequence, `WAI` cycle or
    /// instruction (one byte of a block move is one instruction).
    ///
    /// Returns the clocks charged: zero if the core is stopped or has no
    /// address space, which the caller must treat as "stop".
    pub fn step(&self) -> u64 {
        let links = self.links.lock().clone();
        let attrs =
            MemAttrs::DEFAULT.with_requester(RequesterId(self.requester.load(Ordering::Relaxed)));
        let mut session = self.session.lock();
        let Session { state, space } = &mut *session;
        if self.lines.take_reset_request() {
            state.reset_pending = true;
            state.halted = false;
            state.waiting = false;
            state.pending = None;
        }
        let Some(space) = space.clone() else {
            return 0;
        };
        Exec::new(state, &space, &self.lines, attrs)
            .with_timing(links.timing.as_deref())
            .with_cursor(links.cursor.as_ref())
            .with_rdy(links.rdy.as_deref())
            .step()
    }

    /// Execute until at least `budget` clocks have been charged.
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

    /// Run a scheduler budget of `ticks` clocks, reporting exactly how many
    /// were consumed and carrying any overshoot to the next budget — the
    /// scheme `cpu::mos6502::Mos6502::run_budget` documents.
    pub fn run_budget(&self, ticks: u64) -> u64 {
        let owed = self.session.lock().state.debt;
        if owed >= ticks {
            self.session.lock().state.debt = owed - ticks;
            return ticks;
        }
        let exit = self.links.lock().cursor.as_ref().map(TickCursor::exit_flag);
        let allowance = ticks - owed;
        let mut used = 0u64;
        while used < allowance {
            let n = self.step();
            if n == 0 {
                break;
            }
            used += n;
            if exit.as_ref().is_some_and(ExitFlag::raised) {
                break;
            }
        }
        if used >= allowance {
            self.session.lock().state.debt = used - allowance;
            ticks
        } else {
            self.session.lock().state.debt = 0;
            owed + used
        }
    }

    /// Disassemble `count` instructions from the current PC, under the
    /// current M, X and E, reading memory with debug attributes so nothing
    /// is disturbed.
    #[must_use]
    pub fn disassemble_here(&self, count: usize) -> Vec<disasm::Disassembled> {
        let r = self.regs();
        self.disassemble(r.pc24(), count, disasm::Widths::from_p(r.p, r.e))
    }

    /// Disassemble `count` instructions at `pc` under `widths`.
    #[must_use]
    pub fn disassemble(
        &self,
        pc: u32,
        count: usize,
        widths: disasm::Widths,
    ) -> Vec<disasm::Disassembled> {
        let Some(space) = self.space() else {
            return Vec::new();
        };
        disasm::disassemble_run(pc, count, widths, |addr| {
            space
                .read(u64::from(addr), Width::U8, MemAttrs::DEBUG)
                .ok()
                .map(|v| v as u8)
        })
    }
}

/// The `cpu.w65c816` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "cpu.w65c816",
    // v2 appended the levels the NMI and ABORT edge detectors last sampled;
    // see [`migrations`].
    version: 2,
    summary: "WDC W65C816S 16-bit CPU core (SNES, Apple IIGS), cycle-accurate interpreter",
    properties: &[
        PropertySpec {
            name: "engine",
            kind: ValueKind::Str,
            required: false,
            summary: "which execution engine; only `interp` exists",
        },
        PropertySpec {
            name: "rdy",
            kind: ValueKind::Link,
            required: false,
            summary: "the object that drives /RDY: a DMA unit that halts this core",
        },
        PropertySpec {
            name: "timing",
            kind: ValueKind::Link,
            required: false,
            summary: "the object that says how many clocks each bus cycle costs \
                      (publishes `bus access timing`); one clock per cycle without it",
        },
    ],
    construct: |props| Ok(Box::new(W65c816::from_props(props)?)),
};

/// Add this core's class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut Registry) -> Result<()> {
    reg.add(&CLASS)
}

/// Carry a v1 `cpu.w65c816` chunk forward.
///
/// v2 appends the levels the NMI and ABORT edge detectors last sampled. A v1
/// `load` rebuilt each as its pin's own level, so that is what the step
/// writes: a v1 chunk ends with the six line bytes `irq`, `nmi_level`,
/// `nmi_latch`, `abort_level`, `abort_latch`, `reset`. A v1 snapshot therefore
/// behaves exactly as it always did — an edge that arrived after the core's
/// last cycle is lost, because the chunk never recorded that it was one.
///
/// # Errors
///
/// If the step is already registered.
pub fn migrations(migrations: &mut crate::core::state::Migrations) -> Result<()> {
    migrations.register(CLASS.name, 1, |r, out| {
        let body = r.take(r.remaining())?;
        let lines = body
            .len()
            .checked_sub(6)
            .and_then(|at| body.get(at..))
            .ok_or_else(|| Error::State(String::from("a v1 cpu.w65c816 chunk is too short")))?;
        let (nmi_level, abort_level) = (lines[1], lines[3]);
        out.extend_from_slice(body);
        out.write_u8(nmi_level)?;
        out.write_u8(abort_level)
    })
}

impl Device for W65c816 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let which = match port {
            "irq" => Input::Irq,
            "nmi" => Input::Nmi,
            "abort" => Input::Abort,
            "reset" => Input::Reset,
            _ => return None,
        };
        let pin = Arc::new(InputPin {
            lines: Arc::clone(&self.lines),
            which,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn attach_cursor(&self, cursor: TickCursor) {
        W65c816::attach_cursor(self, cursor);
    }

    fn run(&self, budget: Budget) -> Consumed {
        Consumed::new(self.run_budget(budget.ticks))
    }

    fn reset(&self, kind: ResetKind) {
        let mut session = self.session.lock();
        if kind == ResetKind::Cold {
            session.state = State::new();
        } else {
            let s = &mut session.state;
            s.reset_pending = true;
            s.halted = false;
            s.waiting = false;
            s.pending = None;
        }
        drop(session);
        if kind == ResetKind::Cold {
            self.lines.restore([false; 6]);
        } else {
            self.lines.nmi_latch.store(false, Ordering::Release);
            self.lines.abort_latch.store(false, Ordering::Release);
        }
    }

    // The layout is load-bearing beyond this file: `host::gdb::arch` reads
    // registers out of it by offset. `pc` then `pbr` make a little-endian
    // 24-bit program counter at offset 10.
    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.session.lock().state;
        let r = s.regs;
        w.write_u16(r.a)?;
        w.write_u16(r.x)?;
        w.write_u16(r.y)?;
        w.write_u16(r.s)?;
        w.write_u16(r.d)?;
        w.write_u16(r.pc)?;
        w.write_u8(r.pbr)?;
        w.write_u8(r.dbr)?;
        w.write_u8(r.p)?;
        w.write_bool(r.e)?;
        w.write_u64(s.cycles)?;
        w.write_u64(s.clocks)?;
        w.write_bool(s.halted)?;
        w.write_bool(s.waiting)?;
        w.write_bool(s.reset_pending)?;
        w.write_u8(match s.pending {
            None => 0,
            Some(Interrupt::Irq) => 1,
            Some(Interrupt::Nmi) => 2,
            Some(Interrupt::Abort) => 3,
        })?;
        w.write_u8(s.open_bus)?;
        w.write_u64(s.faults)?;
        w.write_u32(s.last_fault)?;
        w.write_u64(s.debt)?;
        for b in self.lines.snapshot() {
            w.write_bool(b)?;
        }
        // v2: the levels the two edge detectors saw on the last cycle they
        // were clocked. Not the pins' levels — a pin can have moved since, and
        // the edge that makes is the one a restore must not lose.
        w.write_bool(self.lines.nmi_sampled.load(Ordering::Acquire))?;
        w.write_bool(self.lines.abort_sampled.load(Ordering::Acquire))?;
        Ok(())
    }

    fn load(&self, rd: &mut ChunkReader<'_>) -> Result<()> {
        let mut s = State::new();
        let r = &mut s.regs;
        r.a = rd.read_u16()?;
        r.x = rd.read_u16()?;
        r.y = rd.read_u16()?;
        r.s = rd.read_u16()?;
        r.d = rd.read_u16()?;
        r.pc = rd.read_u16()?;
        r.pbr = rd.read_u8()?;
        r.dbr = rd.read_u8()?;
        r.p = rd.read_u8()?;
        r.e = rd.read_bool()?;
        s.cycles = rd.read_u64()?;
        s.clocks = rd.read_u64()?;
        s.halted = rd.read_bool()?;
        s.waiting = rd.read_bool()?;
        s.reset_pending = rd.read_bool()?;
        s.pending = match rd.read_u8()? {
            0 => None,
            1 => Some(Interrupt::Irq),
            2 => Some(Interrupt::Nmi),
            3 => Some(Interrupt::Abort),
            other => {
                return Err(Error::State(alloc::format!(
                    "unknown pending interrupt tag {other}"
                )));
            }
        };
        s.open_bus = rd.read_u8()?;
        s.faults = rd.read_u64()?;
        s.last_fault = rd.read_u32()?;
        s.debt = rd.read_u64()?;
        let mut lines = [false; 6];
        for b in &mut lines {
            *b = rd.read_bool()?;
        }
        let nmi_sampled = rd.read_bool()?;
        let abort_sampled = rd.read_bool()?;
        self.session.lock().state = s;
        self.lines.restore(lines);
        self.lines.nmi_sampled.store(nmi_sampled, Ordering::Release);
        self.lines
            .abort_sampled
            .store(abort_sampled, Ordering::Release);
        Ok(())
    }
}

impl Initiator for W65c816 {
    fn requester(&self) -> RequesterId {
        RequesterId(self.requester.load(Ordering::Relaxed))
    }
}

impl crate::machine::Instance for W65c816 {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "a 65C816 needs an address space to fetch from (`space = cpubus`)",
            ),
        })?;
        self.attach_space(Arc::clone(space));
        self.set_requester(ctx.requester());
        let (rdy, timing) = {
            let l = self.links.lock();
            (l.rdy_link.clone(), l.timing_link.clone())
        };
        if let Some(name) = rdy {
            self.attach_rdy(ctx.export_gate(&name, ExportId::CYCLE_GATE)?);
        }
        if let Some(name) = timing {
            let port = ctx.export_as::<TimingPort>(&name, ExportId::ACCESS_TIMING)?;
            self.attach_timing(Arc::clone(&port.0));
        }
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
        Ok(Arc::new(W65c816::from_props(props)?))
    })
}

/// What the validator should know about `cpu.w65c816`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp"]))
        .prop(PropSchema::new("rdy", ValueKind::Link))
        .prop(PropSchema::new("timing", ValueKind::Link))
        .port("irq", PortDir::In)
        .port("nmi", PortDir::In)
        .port("abort", PortDir::In)
        .port("reset", PortDir::In)
}

/// A description of the instruction set for `rsemu describe`, built from
/// [`isa::TABLE`] so it cannot drift from what the interpreter implements.
#[must_use]
pub fn describe_isa() -> String {
    use core::fmt::Write as _;
    let mut out = String::new();
    for opcode in 0..=255u8 {
        let insn = isa::decode(opcode);
        let _ = writeln!(
            out,
            "{opcode:02x} {:<4} {:<8} {}",
            insn.op.mnemonic(),
            insn.mode.name(),
            insn.op.summary()
        );
    }
    out
}
