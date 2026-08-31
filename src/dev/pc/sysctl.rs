//! The AT's two system control ports: 0x61 (port B) and 0x92 (port A).
//!
//! # Sources
//!
//! * *IBM Personal Computer AT Technical Reference* (1984), system board
//!   description and I/O address map: port 0x61's bit assignments, the refresh
//!   toggle, the timer 2 gate and output, and the parity / I/O channel check
//!   pair that drive NMI.
//! * Ralf Brown's Interrupt List, ports section, entries for 0061 and 0092 —
//!   the bit-level read/write behaviour the board, not a data sheet, defines.
//! * The PS/2 and later AT chipset convention for port 0x92 ("System Control
//!   Port A"): bit 0 fast reset, bit 1 A20 gate.
//!
//! No emulator source was consulted (`CLAUDE.md`, provenance).
//!
//! # Why this is a device at all
//!
//! Neither port is a chip. They are latches and gates soldered onto the system
//! board: a couple of flip-flops, a gate onto the 8254's `GATE2` input, a tap
//! off the DRAM refresh request, and a second path to the two things the 8042
//! used to do alone. That is precisely why they get a device of their own
//! rather than being smuggled into the timer or the keyboard controller. Port
//! 0x61 is not part of the 8254 — it *gates* the 8254 — and port 0x92 is not
//! part of the 8042; hiding either inside a chip model would make that chip
//! answer an address it does not decode, and would put the A20 gate in two
//! places at once when the board's A20 is the wired-OR of both paths.
//!
//! Two ports thirty-one addresses apart are two regions, not one window with a
//! hole in it, so a machine file maps [`region("portb")`](Device::region) at
//! 0x61 and [`region("porta")`](Device::region) at 0x92. `""` is port B,
//! because it is the one every AT has.
//!
//! # Port B, 0x61
//!
//! ```text
//!   bit 7  parity check status        read-only; cleared by writing bit 2
//!   bit 6  I/O channel check status   read-only; cleared by writing bit 3
//!   bit 5  timer 2 output             follows the `timer2` input pin
//!   bit 4  refresh toggle             flips on each edge of the `refresh` pin
//!   bit 3  I/O channel check enable   latched, reads back
//!   bit 2  parity check enable        latched, reads back
//!   bit 1  speaker data enable        latched, reads back
//!   bit 0  timer 2 gate               latched, reads back, drives `gate2`
//! ```
//!
//! Bits 4 and 5 are the interesting ones, and both are **inputs**, not
//! inventions of this device:
//!
//! * Bit 5 is the 8254's `OUT2` pin brought straight to the bus. Firmware
//!   calibrates loops by spinning on it, so it has to follow the wire rather
//!   than be synthesised from a counter this device does not own.
//! * Bit 4 is the DRAM refresh request, which on an AT is counter 1's output.
//!   Firmware uses it as a coarse timing reference and one well-known
//!   power-on self-test spins waiting for it to change, so a model that never
//!   moved it would hang. It is modelled as an input pin the machine file
//!   wires to the timer, and this device invents no timing of its own
//!   (`CLAUDE.md`: the scheduler owns time). It toggles on **each** edge, so
//!   the same wiring works whether counter 1 is programmed to emit a narrow
//!   pulse per refresh or a square wave.
//!
//! Writing bit 2 or bit 3 clears the matching status bit. There is no separate
//! acknowledge register on the AT: the write that re-arms the check is the
//! write that clears it, which is how a parity NMI handler gets out of its own
//! interrupt.
//!
//! # Port A, 0x92
//!
//! ```text
//!   bit 1  A20 gate     latched, drives `a20`
//!   bit 0  fast reset   write-1 pulses `reset`; always reads back clear
//! ```
//!
//! Both are the chipset's fast path to jobs the 8042 originally did with a
//! command byte and a several-microsecond handshake. The board's A20 is the
//! wired-OR of this pin and the keyboard controller's, which is a machine-file
//! wiring question, not something this device resolves.
//!
//! Bit 0 reads back clear even though every other bit is latched. It has to:
//! the canonical A20 sequence is a read-modify-write (`in al,0x92` / `or al,2`
//! / `out 0x92,al`), and a bit 0 that read back set would reset the machine on
//! the way past.
//!
//! # No `speaker` property
//!
//! There is deliberately none. The only named-signal seam in the tree is
//! `dev::riscv::syscon`'s, and it does not suit: it lives behind the
//! `dev-riscv` feature, so a PC build would have to link RISC-V devices to
//! reach it, and its payload is a one-shot power request rather than an
//! observable level. Inventing a second seam for one bit is worse than having
//! none, so a test that wants to know the speaker is gated on reads port B
//! back or watches the `gate2` pin — which is the same information the 8254
//! gets.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.sysctl";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space each port answers.
///
/// One byte each. The two are thirty-one addresses apart, so they are separate
/// regions rather than one window — see the module docs.
pub const REGISTER_WINDOW_LEN: u64 = 1;

// -- port B (0x61) ----------------------------------------------------------

/// Timer 2 gate. Latched, and driven out on the `gate2` pin.
const B_GATE2: u8 = 0x01;
/// Speaker data enable. Latched; the 8254's output is ANDed with it on the
/// board, which is a wiring question rather than one for this latch.
const B_SPEAKER: u8 = 0x02;
/// Parity check enable. Latched, and writing it clears [`B_PARITY_STATUS`].
const B_PARITY_ENABLE: u8 = 0x04;
/// I/O channel check enable. Latched, and writing it clears [`B_IOCHK_STATUS`].
const B_IOCHK_ENABLE: u8 = 0x08;
/// The refresh toggle, driven by the `refresh` input pin.
const B_REFRESH: u8 = 0x10;
/// Timer 2 output, driven by the `timer2` input pin.
const B_TIMER2_OUT: u8 = 0x20;
/// I/O channel check status: an adapter reported a failure.
const B_IOCHK_STATUS: u8 = 0x40;
/// Parity check status: a memory board reported bad parity.
const B_PARITY_STATUS: u8 = 0x80;
/// The four bits a write latches. The top nibble is status and inputs, and a
/// write to it lands nowhere.
const B_LATCH_MASK: u8 = B_GATE2 | B_SPEAKER | B_PARITY_ENABLE | B_IOCHK_ENABLE;

// -- port A (0x92) ----------------------------------------------------------

/// Fast reset: writing it set pulses the CPU's reset line.
const A_FAST_RESET: u8 = 0x01;
/// The fast A20 gate, driven out on the `a20` pin.
const A_GATE_A20: u8 = 0x02;

// -- input pins -------------------------------------------------------------

/// The line number [`Device::sink`] hands out for the `refresh` input.
const LINE_REFRESH: u32 = 0;
/// The line number [`Device::sink`] hands out for the `timer2` input.
const LINE_TIMER2: u32 = 1;

/// Everything a snapshot has to carry.
///
/// The wire handles are not here: they are the machine's topology, rebuilt by
/// realize on the far side of a load (`CLAUDE.md`: derived state is never
/// serialized).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct State {
    /// The port B write latch, bits 0-3 only.
    port_b: u8,
    /// The port A latch. Bit 0 is always clear in here — see the module docs.
    port_a: u8,
    /// Port B bit 7: a memory board reported bad parity.
    parity_status: bool,
    /// Port B bit 6: an adapter reported an I/O channel check.
    iochk_status: bool,
    /// Port B bit 4. Not a function of the input level: it is a divide-by-two
    /// of the refresh request, so it survives across edges and is saved.
    refresh_toggle: bool,
    /// The last level seen on the `refresh` pin, so an edge can be told from a
    /// repeat after a load.
    refresh_in: bool,
    /// The last level seen on the `timer2` pin, which port B bit 5 reports.
    timer2_in: bool,
}

/// The latches, and the three pins they drive.
struct Registers {
    state: Mutex<State>,
    /// The 8254's `GATE2` input. At [`LockRank::LEAF`] so it can be driven
    /// with nothing else held.
    gate2: Mutex<Option<WireSource>>,
    /// The board's fast A20 path, wire-ORed with the 8042's.
    a20: Mutex<Option<WireSource>>,
    /// The CPU reset line. Pulsed, never held.
    reset: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

/// Drive `pin`, if it is connected. Never called with the state lock held.
fn drive(pin: &Mutex<Option<WireSource>>, level: Level) {
    let out = pin.lock().clone();
    if let Some(out) = out {
        out.set(level);
    }
}

impl Registers {
    /// Port B as a read produces it.
    ///
    /// Deliberately free of side effects, which is what makes `MemAttrs::debug`
    /// a non-decision here. The tempting shortcut — toggling bit 4 on every
    /// read, because that makes a firmware spin loop terminate without any
    /// timing behind it — would make a debugger's window refresh change the
    /// machine, and would put this device in charge of a rate it has no clock
    /// for.
    fn read_b(&self) -> u8 {
        let s = self.state.lock();
        let mut value = s.port_b & B_LATCH_MASK;
        if s.refresh_toggle {
            value |= B_REFRESH;
        }
        if s.timer2_in {
            value |= B_TIMER2_OUT;
        }
        if s.iochk_status {
            value |= B_IOCHK_STATUS;
        }
        if s.parity_status {
            value |= B_PARITY_STATUS;
        }
        value
    }

    /// Latch a port B write and gate the timer.
    fn write_b(&self, value: u8) {
        let gate2 = {
            let mut s = self.state.lock();
            s.port_b = value & B_LATCH_MASK;
            // The write that re-arms a check is the write that clears it: the
            // AT has no separate acknowledge register, and this is how a parity
            // NMI handler stops being re-entered.
            if value & B_PARITY_ENABLE != 0 {
                s.parity_status = false;
            }
            if value & B_IOCHK_ENABLE != 0 {
                s.iochk_status = false;
            }
            Level::from_bool(s.port_b & B_GATE2 != 0)
        };
        drive(&self.gate2, gate2);
    }

    /// Port A as a read produces it. Bit 0 is never set — module docs.
    fn read_a(&self) -> u8 {
        self.state.lock().port_a
    }

    /// Latch a port A write, move A20, and pulse reset if asked.
    fn write_a(&self, value: u8) {
        let (a20, fast_reset) = {
            let mut s = self.state.lock();
            s.port_a = value & !A_FAST_RESET;
            (
                Level::from_bool(value & A_GATE_A20 != 0),
                value & A_FAST_RESET != 0,
            )
        };
        // A20 first: a write that sets both bits is asking for a reset with the
        // gate already where it wants it, and the reset pulse re-enters this
        // device's own `reset` through the machine's reset tree.
        drive(&self.a20, a20);
        if fast_reset {
            // A pulse, not a level. Holding reset would need somebody to
            // release it, and nothing on the board does.
            let out = self.reset.lock().clone();
            if let Some(out) = out {
                out.pulse(Level::High);
            }
        }
    }

    /// The `refresh` pin moved. Each edge flips bit 4.
    fn refresh_edge(&self, level: Level) {
        let mut s = self.state.lock();
        if s.refresh_in == level.is_high() {
            return;
        }
        s.refresh_in = level.is_high();
        s.refresh_toggle = !s.refresh_toggle;
    }

    /// The `timer2` pin moved. Bit 5 is the pin, with nothing in between.
    fn timer2_level(&self, level: Level) {
        self.state.lock().timer2_in = level.is_high();
    }

    /// Drive both level outputs from the current latches.
    fn drive_outputs(&self) {
        let (gate2, a20) = {
            let s = self.state.lock();
            (
                Level::from_bool(s.port_b & B_GATE2 != 0),
                Level::from_bool(s.port_a & A_GATE_A20 != 0),
            )
        };
        drive(&self.gate2, gate2);
        drive(&self.a20, a20);
    }
}

/// Port 0x61, as something an address space can dispatch to.
#[derive(Debug)]
struct PortB(Arc<Registers>);

/// Port 0x92, as something an address space can dispatch to.
#[derive(Debug)]
struct PortA(Arc<Registers>);

/// An 8-bit port on an 8-bit bus. Both ports are one byte at one address, so a
/// wider access is a decode this board never performs.
fn byte_port() -> AccessConstraints {
    AccessConstraints::word(Width::U8, Endian::Little)
}

/// Both ports are one byte at one address, so anything else is a decode this
/// board never performs.
fn only_byte_zero(offset: u64, len: usize) -> MemResult {
    if offset == 0 && len == 1 {
        Ok(())
    } else {
        Err(BusError::BadAccess)
    }
}

impl MemOps for PortB {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        only_byte_zero(offset, dst.len())?;
        // No `debug` branch: the read has no side effects to suppress, and in
        // particular it does not touch the refresh toggle.
        dst[0] = self.0.read_b();
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        only_byte_zero(offset, src.len())?;
        if attrs.debug {
            // Every bit here does something: bit 0 gates the timer, bit 1 the
            // speaker, and bits 2 and 3 acknowledge an NMI. There is no
            // harmless subset to allow.
            return Err(BusError::BadAccess);
        }
        self.0.write_b(src[0]);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        byte_port()
    }
}

impl MemOps for PortA {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        only_byte_zero(offset, dst.len())?;
        dst[0] = self.0.read_a();
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        only_byte_zero(offset, src.len())?;
        if attrs.debug {
            // A debug write of bit 0 would reset the machine somebody is
            // debugging, and of bit 1 would move A20 under the guest's feet.
            return Err(BusError::BadAccess);
        }
        self.0.write_a(src[0]);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        byte_port()
    }
}

/// One of the two input pins, with the fan-in that makes a shared net correct.
#[derive(Debug)]
struct InputPin {
    regs: Arc<Registers>,
    /// [`LINE_REFRESH`] or [`LINE_TIMER2`].
    line: u32,
    inputs: FanIn,
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // Wired-OR, so a second driver on the same net cannot make the pin drop
        // while the first still asserts (`core::wire`, module docs).
        let level = self.inputs.resolve(Resolve::Or);
        if self.line == LINE_REFRESH {
            self.regs.refresh_edge(level);
        } else {
            self.regs.timer2_level(level);
        }
    }
}

/// The AT's system control ports.
#[derive(Debug)]
pub struct SysCtl {
    regs: Arc<Registers>,
    port_b: RegionRef,
    port_a: RegionRef,
    /// The sinks handed out by [`Device::sink`], kept alive here: a net holds
    /// only a weak reference to a sink, so the device owns the strong one.
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl SysCtl {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<SysCtl> {
        props.reader().finish()?;
        Ok(SysCtl::default_device())
    }

    /// One with no properties set.
    #[must_use]
    pub fn default_device() -> SysCtl {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            gate2: Mutex::with_rank(LockRank::LEAF, None),
            a20: Mutex::with_rank(LockRank::LEAF, None),
            reset: Mutex::with_rank(LockRank::LEAF, None),
        });
        let port_b: RegionRef = Arc::new(Region::io(
            "pc.sysctl.portb",
            REGISTER_WINDOW_LEN,
            Arc::new(PortB(Arc::clone(&regs))) as Arc<dyn MemOps>,
        ));
        let port_a: RegionRef = Arc::new(Region::io(
            "pc.sysctl.porta",
            REGISTER_WINDOW_LEN,
            Arc::new(PortA(Arc::clone(&regs))) as Arc<dyn MemOps>,
        ));
        SysCtl {
            regs,
            port_b,
            port_a,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Latch a parity check, as a memory board's `/PCHK` would.
    ///
    /// Port B bit 7 then reads set until the guest writes bit 2. Nothing on
    /// this board asserts it yet — the AT drives NMI from it, and the NMI path
    /// is the machine file's — so it arrives as a method rather than as a third
    /// input pin.
    pub fn raise_parity_check(&self) {
        self.regs.state.lock().parity_status = true;
    }

    /// Latch an I/O channel check, as an adapter's `/IOCHCK` would.
    ///
    /// Port B bit 6 then reads set until the guest writes bit 3.
    pub fn raise_io_channel_check(&self) {
        self.regs.state.lock().iochk_status = true;
    }
}

/// The `pc.sysctl` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the AT system control ports: speaker gate, refresh toggle, A20 and fast reset",
    properties: &[],
    construct: |props| Ok(Box::new(SysCtl::new(props)?)),
};

/// The error for a pin name this device does not have.
fn unknown_pin(port: &str) -> Error {
    Error::Config {
        at: port.to_string(),
        message: String::from(
            "the system control ports take `refresh` and `timer2` in, \
             and drive `gate2`, `a20` and `reset` out",
        ),
    }
}

impl Device for SysCtl {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both a cold and a warm reset land here with everything clear, which
        // is the state the board powers up in: speaker silent, A20 masked. The
        // remembered input levels go too — realize re-announces them, and a
        // stale level would put a phantom edge in the first refresh toggle.
        *self.regs.state.lock() = State::default();
        self.regs.drive_outputs();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            // `""` is port B: it is the one every AT has, and the one a machine
            // file that maps only a speaker gate wants.
            "" | "portb" => Some(Arc::clone(&self.port_b)),
            "porta" => Some(Arc::clone(&self.port_a)),
            _ => None,
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let line = match port {
            "refresh" => LINE_REFRESH,
            "timer2" => LINE_TIMER2,
            _ => return None,
        };
        let pin = Arc::new(InputPin {
            regs: Arc::clone(&self.regs),
            line,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let pin = match port {
            "gate2" => &self.regs.gate2,
            "a20" => &self.regs.a20,
            "reset" => &self.regs.reset,
            _ => return Err(unknown_pin(port)),
        };
        *pin.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        match port {
            // Both idle low out of reset, but a snapshot loaded before the
            // sweep can leave either high, and the 8254 has to be told.
            "gate2" | "a20" => self.regs.drive_outputs(),
            // `reset` is a pulse, not a level: it has no idle level to announce
            // beyond the low a fresh net already sits at, and driving it here
            // would reset the machine as it comes up.
            _ => {}
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = *self.regs.state.lock();
        w.write_u8(s.port_b)?;
        w.write_u8(s.port_a)?;
        w.write_bool(s.parity_status)?;
        w.write_bool(s.iochk_status)?;
        w.write_bool(s.refresh_toggle)?;
        // The remembered input levels, not the wires: a `FanIn` is rebuilt from
        // the machine's topology, but whether the refresh pin was last seen
        // high decides whether the next notification is an edge.
        w.write_bool(s.refresh_in)?;
        w.write_bool(s.timer2_in)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let state = State {
            port_b: r.read_u8()? & B_LATCH_MASK,
            port_a: r.read_u8()? & !A_FAST_RESET,
            parity_status: r.read_bool()?,
            iochk_status: r.read_bool()?,
            refresh_toggle: r.read_bool()?,
            refresh_in: r.read_bool()?,
            timer2_in: r.read_bool()?,
        };
        *self.regs.state.lock() = state;
        // The gate and A20 are levels the rest of the machine has to agree
        // with, so they are re-driven; reset is a pulse and has nothing to
        // restore.
        self.regs.drive_outputs();
        Ok(())
    }
}

impl Instance for SysCtl {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(SysCtl::new(props)?)))
}

/// What the validator should know about `pc.sysctl`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::PortDir;
    ClassSchema::new(CLASS_NAME)
        .region("")
        .region("portb")
        .region("porta")
        .port("refresh", PortDir::In)
        .port("timer2", PortDir::In)
        .port("gate2", PortDir::Out)
        .port("a20", PortDir::Out)
        .port("reset", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireIdAllocator};

    /// A probe that remembers the last level and counts rising edges, which is
    /// the only way to see a pulse after the fact.
    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
        rises: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            if level.is_high() {
                self.rises.fetch_add(1, Ordering::Relaxed);
            }
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
        }
    }

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) == 1
        }

        fn rises(&self) -> u32 {
            self.rises.load(Ordering::Relaxed)
        }
    }

    /// Attach a probe to output pin `port`.
    fn watch(dev: &SysCtl, port: &str) -> Arc<Probe> {
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        dev.connect(port, WireSource::new(wire, id))
            .expect("the system control ports drive this pin");
        probe
    }

    /// A driver for input pin `port`.
    fn feed(dev: &SysCtl, port: &str) -> WireSource {
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let pin = dev
            .sink(port, &[id])
            .expect("the system control ports listen on this pin");
        let wire = Wire::builder()
            .source(id)
            .sink(pin.sink, pin.line)
            .build_shared();
        WireSource::new(wire, id)
    }

    fn peek(dev: &SysCtl, region: &str) -> u8 {
        let mut byte = [0u8; 1];
        ops(dev, region)
            .read(0, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    fn poke(dev: &SysCtl, region: &str, value: u8) {
        ops(dev, region)
            .write(0, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    /// The `MemOps` behind a named region, which is what a machine file maps.
    fn ops(dev: &SysCtl, region: &str) -> Arc<dyn MemOps> {
        match region {
            "porta" => Arc::new(PortA(Arc::clone(&dev.regs))) as Arc<dyn MemOps>,
            _ => Arc::new(PortB(Arc::clone(&dev.regs))) as Arc<dyn MemOps>,
        }
    }

    #[test]
    fn port_b_bit_0_gates_the_timer_and_reads_back() {
        let dev = SysCtl::default_device();
        let gate2 = watch(&dev, "gate2");
        assert!(!gate2.high(), "the speaker is silent at power-on");

        poke(&dev, "portb", B_GATE2);
        assert!(gate2.high());
        assert_eq!(peek(&dev, "portb") & B_LATCH_MASK, B_GATE2);

        poke(&dev, "portb", B_GATE2 | B_SPEAKER);
        assert!(gate2.high(), "and stays gated");
        assert_eq!(peek(&dev, "portb") & B_LATCH_MASK, B_GATE2 | B_SPEAKER);

        poke(&dev, "portb", 0);
        assert!(!gate2.high());
    }

    #[test]
    fn port_b_bit_5_follows_the_timer2_pin() {
        // Firmware calibrates by spinning on this bit, so it has to be the pin
        // and not a value this device made up.
        let dev = SysCtl::default_device();
        let timer2 = feed(&dev, "timer2");
        assert_eq!(peek(&dev, "portb") & B_TIMER2_OUT, 0);
        timer2.set(Level::High);
        assert_eq!(peek(&dev, "portb") & B_TIMER2_OUT, B_TIMER2_OUT);
        timer2.set(Level::Low);
        assert_eq!(peek(&dev, "portb") & B_TIMER2_OUT, 0);
    }

    #[test]
    fn each_refresh_edge_flips_port_b_bit_4() {
        let dev = SysCtl::default_device();
        let refresh = feed(&dev, "refresh");
        let mut expected = 0u8;
        for _ in 0..4 {
            for level in [Level::High, Level::Low] {
                refresh.set(level);
                expected ^= B_REFRESH;
                assert_eq!(peek(&dev, "portb") & B_REFRESH, expected);
            }
        }
        // A repeat of the level already seen is not an edge.
        refresh.set(Level::Low);
        assert_eq!(peek(&dev, "portb") & B_REFRESH, expected);
    }

    #[test]
    fn port_a_bit_1_drives_a20_and_bit_0_pulses_reset() {
        let dev = SysCtl::default_device();
        let a20 = watch(&dev, "a20");
        let reset = watch(&dev, "reset");

        poke(&dev, "porta", A_GATE_A20);
        assert!(a20.high());
        assert_eq!(reset.rises(), 0, "A20 alone resets nothing");
        assert_eq!(peek(&dev, "porta"), A_GATE_A20);

        poke(&dev, "porta", A_GATE_A20 | A_FAST_RESET);
        assert_eq!(reset.rises(), 1);
        assert!(!reset.high(), "a pulse, not a level");
        assert!(a20.high(), "and A20 stayed where it was put");
        // Bit 0 reads back clear, or the read-modify-write every A20 routine
        // performs would reset the machine on its way past.
        assert_eq!(peek(&dev, "porta"), A_GATE_A20);

        poke(&dev, "porta", 0);
        assert!(!a20.high());
    }

    #[test]
    fn writing_the_enable_bits_clears_the_check_status_bits() {
        // The AT has no acknowledge register: this write *is* how a parity NMI
        // handler stops being re-entered.
        let dev = SysCtl::default_device();
        dev.raise_parity_check();
        dev.raise_io_channel_check();
        assert_eq!(
            peek(&dev, "portb") & (B_PARITY_STATUS | B_IOCHK_STATUS),
            B_PARITY_STATUS | B_IOCHK_STATUS
        );

        poke(&dev, "portb", B_PARITY_ENABLE);
        assert_eq!(peek(&dev, "portb") & B_PARITY_STATUS, 0);
        assert_eq!(
            peek(&dev, "portb") & B_IOCHK_STATUS,
            B_IOCHK_STATUS,
            "the other one stands"
        );

        poke(&dev, "portb", B_IOCHK_ENABLE);
        assert_eq!(peek(&dev, "portb") & B_IOCHK_STATUS, 0);
    }

    #[test]
    fn a_debug_read_of_port_b_changes_nothing_and_a_debug_write_is_refused() {
        let dev = SysCtl::default_device();
        let refresh = feed(&dev, "refresh");
        let gate2 = watch(&dev, "gate2");
        refresh.set(Level::High);
        let before = peek(&dev, "portb");
        assert_eq!(before & B_REFRESH, B_REFRESH);

        let mut byte = [0u8; 1];
        for _ in 0..3 {
            ops(&dev, "portb")
                .read(0, &mut byte, MemAttrs::DEBUG)
                .expect("a debug read is legal");
            assert_eq!(byte[0], before, "the refresh toggle did not move");
        }

        assert!(
            ops(&dev, "portb")
                .write(0, &[B_GATE2], MemAttrs::DEBUG)
                .is_err()
        );
        assert!(!gate2.high(), "and nothing was gated");
        assert!(
            ops(&dev, "porta")
                .write(0, &[A_FAST_RESET], MemAttrs::DEBUG)
                .is_err()
        );
    }

    #[test]
    fn an_access_that_is_not_a_single_byte_at_offset_zero_is_refused() {
        let dev = SysCtl::default_device();
        for region in ["portb", "porta"] {
            let ops = ops(&dev, region);
            assert!(ops.read(0, &mut [0u8; 2], MemAttrs::DEFAULT).is_err());
            assert!(ops.read(1, &mut [0u8; 1], MemAttrs::DEFAULT).is_err());
            assert!(ops.write(0, &[0u8; 4], MemAttrs::DEFAULT).is_err());
            assert!(ops.write(1, &[0u8], MemAttrs::DEFAULT).is_err());
        }
    }

    #[test]
    fn the_two_ports_are_separate_regions() {
        let dev = SysCtl::default_device();
        assert!(dev.region("").is_some());
        assert!(dev.region("portb").is_some());
        assert!(dev.region("porta").is_some());
        assert!(
            dev.region("regs").is_none(),
            "one name per port, not a lump"
        );
        // A write to one is not a write to the other: they are 0x61 and 0x92.
        poke(&dev, "portb", B_GATE2 | B_SPEAKER);
        assert_eq!(peek(&dev, "porta"), 0);
        poke(&dev, "porta", A_GATE_A20);
        assert_eq!(peek(&dev, "portb") & B_LATCH_MASK, B_GATE2 | B_SPEAKER);
    }

    #[test]
    fn an_unknown_pin_is_an_error_rather_than_a_silent_no_op() {
        let dev = SysCtl::default_device();
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let wire = Wire::builder().source(id).build_shared();
        assert!(dev.connect("speaker", WireSource::new(wire, id)).is_err());
        assert!(dev.sink("gate2", &[id]).is_none(), "gate2 is an output");
    }

    #[test]
    fn a_reset_silences_the_speaker_and_masks_a20() {
        let dev = SysCtl::default_device();
        let gate2 = watch(&dev, "gate2");
        let a20 = watch(&dev, "a20");
        poke(&dev, "portb", B_GATE2 | B_SPEAKER);
        poke(&dev, "porta", A_GATE_A20);
        dev.raise_parity_check();

        dev.reset(ResetKind::Cold);
        assert!(!gate2.high());
        assert!(!a20.high());
        assert_eq!(peek(&dev, "portb"), 0);
        assert_eq!(peek(&dev, "porta"), 0);
    }

    /// Save `dev` into a one-device snapshot.
    fn save_image(dev: &SysCtl) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("sysctl", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("sysctl", CLASS.name, CLASS.version).unwrap();
            dev.save(&mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn a_snapshot_round_trips_every_latch_and_both_input_levels() {
        let saved = SysCtl::default_device();
        let refresh = feed(&saved, "refresh");
        let timer2 = feed(&saved, "timer2");
        poke(&saved, "portb", B_GATE2 | B_SPEAKER | B_IOCHK_ENABLE);
        poke(&saved, "porta", A_GATE_A20 | 0x40);
        saved.raise_parity_check();
        // An odd number of edges, so the toggle is set and the pin is high:
        // the two are independent and a snapshot that conflated them would
        // still pass a one-edge test.
        refresh.set(Level::High);
        timer2.set(Level::High);
        let image = save_image(&saved);

        let restored = SysCtl::default_device();
        let gate2 = watch(&restored, "gate2");
        let a20 = watch(&restored, "a20");
        let reader = StateReader::new(&image).unwrap();
        let chunk = reader
            .load("sysctl", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        assert_eq!(peek(&restored, "portb"), peek(&saved, "portb"));
        assert_eq!(peek(&restored, "porta"), peek(&saved, "porta"));
        assert!(gate2.high(), "the levels were re-driven on load");
        assert!(a20.high());
        // The remembered refresh level came back, so the next notification of
        // the same level is still not an edge.
        let refresh = feed(&restored, "refresh");
        refresh.set(Level::High);
        assert_eq!(
            peek(&restored, "portb") & B_REFRESH,
            peek(&saved, "portb") & B_REFRESH
        );

        assert_eq!(save_image(&restored), image, "byte-identical");
    }
}
