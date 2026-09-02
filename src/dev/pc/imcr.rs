//! The **Interrupt Mode Configuration Register** at ports 0x22/0x23: the
//! switch between PIC mode and virtual-wire mode on a board that has both an
//! 8259A and a local APIC.
//!
//! # Sources
//!
//! * *MultiProcessor Specification*, version 1.4, **§3.6.2.1, "PIC Mode"**:
//!
//!   > PIC Mode bypasses all APIC components and forces the system to operate
//!   > in single-processor mode. […] This mode is implemented by an interrupt
//!   > mode configuration register (IMCR), […] The IMCR is supported by two
//!   > read/writable or write-only I/O ports, 22h and 23h, which receive
//!   > address and data respectively. To access the IMCR, write a value of 70h
//!   > to I/O port 22h, which selects the IMCR. Then write the data to I/O port
//!   > 23h. The power-on default value is zero, which connects the NMI and 8259
//!   > INTR lines directly to the BSP. Writing a value of 01h forces the NMI
//!   > and 8259 INTR signals to pass through the APIC.
//!
//! * Intel SDM Volume 3A §10.5.1 for what the far end of the switch does: an
//!   `ExtINT`-programmed `LINT0` runs an acknowledge cycle against the external
//!   8259A and takes the vector from it.
//!
//! **No emulator source was consulted** (`CLAUDE.md`, provenance).
//!
//! # Why this is a device rather than a bit in some chip
//!
//! Because it is not in any chip. The IMCR is a gate on the system board
//! between the master 8259A's `INT` pin and two places it can land: the
//! processor's own `INTR` pin, and the local APIC's `LINT0`. Neither the 8259A
//! nor the APIC nor the processor owns it — an MP-capable board simply routes
//! one output to one of two destinations and puts the selector behind two I/O
//! ports. It is the same argument [`sysctl`](super::sysctl) makes for port 0x61
//! and port 0x92: latches and gates soldered onto the board are a device here
//! because nothing else can honestly hold them.
//!
//! It is also what makes an APIC **additive** to a board that already boots. A
//! machine file that simply moved `pic1.int` from `cpu0.intr` to `lapic0.lint0`
//! would be a board whose timer tick stops reaching the processor until
//! firmware software-enables the APIC and programs `LINT0` for `ExtINT`. That
//! is true of real hardware in *virtual wire* mode and false of real hardware
//! out of reset, because out of reset a real board is in PIC mode: the APIC is
//! bypassed, the 8259A drives the pin directly, and DOS works. This device is
//! that "out of reset".
//!
//! ```text
//!                          IMCR = 0 (power-on)   IMCR = 1
//!   pic1.int  ->  imcr  ->     cpu0.intr           lapic0.lint0
//! ```
//!
//! # The acknowledge travels back the same way
//!
//! `INTR` is not only a level: whoever takes the interrupt runs an acknowledge
//! cycle back down the net to fetch the vector ([`IntAck`]). So this device
//! forwards one as well as the other — it is handed the 8259A's handler on its
//! input pin and offers its own on both outputs, which pass the cycle straight
//! through. A mux that carried the level and dropped the acknowledge would
//! deliver interrupt 0 to every guest on the board.
//!
//! # What is not modelled
//!
//! **The NMI half of the same switch.** §3.6.2.1 says the IMCR routes NMI as
//! well as `INTR`, and this device routes only `INTR` — because on the boards
//! it is fitted to nothing drives an NMI into it. A device must not invent a
//! level for an input pin (`CLAUDE.md`), and inventing a *pin* to hold one
//! would be worse. When a board grows an NMI source, this grows a second pair
//! of pins and the register bit already says which way they point.
//!
//! **Ports 0x22/0x23 as anything else.** A Cyrix processor's configuration
//! registers live at the same two addresses, and a chipset index/data pair
//! sometimes does too. This decodes them as the MP specification's IMCR and
//! nothing else; an index other than 70h selects nothing, so a write to 0x23
//! after one is discarded and a read of it answers ones, which is what an
//! unterminated bus does.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{
    FanIn, IntAck, IntAckCycle, IntAckHandlers, IntAckResponse, Level, Resolve, WireId, WireSink,
    WireSource,
};
use crate::machine::realize::Instance;
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.imcr";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much I/O space the pair decodes: the index at 0x22 and the data at
/// 0x23 (MP specification §3.6.2.1).
pub const REGISTER_WINDOW_LEN: u64 = 2;

/// The index that selects the IMCR itself (§3.6.2.1, "write a value of 70h to
/// I/O port 22h").
pub const IMCR_INDEX: u8 = 0x70;

/// The IMCR value that routes `INTR` through the APIC (§3.6.2.1, "writing a
/// value of 01h").
const IMCR_VIA_APIC: u8 = 0x01;

/// The one input pin's line number.
const LINE_INT: u32 = 0;

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, Default)]
struct State {
    /// What the last write to 0x22 selected.
    index: u8,
    /// The register itself. Zero — PIC mode — at power on, which §3.6.2.1
    /// states outright and which is the whole reason this device makes an APIC
    /// additive rather than disruptive.
    imcr: u8,
    /// The level the master 8259A is currently driving onto the input pin.
    ///
    /// Remembered rather than re-read, because the destination changes under it
    /// the instant the guest writes the register and the level does not: a
    /// board that forgot it would leave the newly selected destination low
    /// until the next 8259A transition, which on a machine with one pending
    /// interrupt is for ever.
    asserted: bool,
}

/// The gate: state, pins, and the acknowledge it forwards.
struct Registers {
    state: Mutex<State>,
    /// The processor's own `INTR` pin. At [`LockRank::LEAF`] so it can be
    /// driven with nothing else held.
    intr: Mutex<Option<WireSource>>,
    /// The local APIC's `LINT0`.
    lint0: Mutex<Option<WireSource>>,
    /// What answers an acknowledge cycle arriving on either output: the 8259A
    /// on the input pin. A list rather than one handler because that is what
    /// `core::wire` hands out and because a net may have more than one driver.
    upstream: IntAckHandlers,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<in use>"),
        };
        s.field("upstream", &self.upstream.len()).finish()
    }
}

/// Drive one output, with no lock held.
fn drive(holder: &Mutex<Option<WireSource>>, level: Level) {
    let out = holder.lock().clone();
    if let Some(out) = out {
        out.set(level);
    }
}

impl Registers {
    fn new() -> Registers {
        Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            intr: Mutex::with_rank(LockRank::LEAF, None),
            lint0: Mutex::with_rank(LockRank::LEAF, None),
            upstream: IntAckHandlers::new(),
        }
    }

    /// Whether the register currently routes `INTR` through the APIC.
    fn via_apic(&self) -> bool {
        self.state.lock().imcr & IMCR_VIA_APIC != 0
    }

    /// Put both outputs where the register and the input level say they belong.
    ///
    /// The unselected destination is driven **low**, not left alone: switching
    /// modes with an interrupt pending must not leave the abandoned pin high
    /// for ever, and a level output has to say what it is doing even when the
    /// answer is nothing.
    fn drive_outputs(&self) {
        let (intr, lint0) = {
            let s = self.state.lock();
            let via_apic = s.imcr & IMCR_VIA_APIC != 0;
            (
                Level::from_bool(s.asserted && !via_apic),
                Level::from_bool(s.asserted && via_apic),
            )
        };
        drive(&self.intr, intr);
        drive(&self.lint0, lint0);
    }

    /// The 8259A's `INT` pin moved.
    fn input_level(&self, level: Level) {
        {
            let mut s = self.state.lock();
            let asserted = level == Level::High;
            if s.asserted == asserted {
                return;
            }
            s.asserted = asserted;
        }
        self.drive_outputs();
    }

    /// Read one of the two ports. `debug` changes nothing here — neither read
    /// has a side effect — but it is taken so the contract is visible.
    fn read(&self, offset: u64) -> u8 {
        let s = self.state.lock();
        match offset {
            0 => s.index,
            _ if s.index == IMCR_INDEX => s.imcr,
            // Nothing is selected, so nothing answers and the bus reads as
            // ones. §3.6.2.1 leaves this to the board; ones is what an
            // unterminated ISA bus does.
            _ => 0xff,
        }
    }

    /// Write one of the two ports. Returns whether the routing moved.
    fn write(&self, offset: u64, value: u8) -> bool {
        let mut s = self.state.lock();
        if offset == 0 {
            s.index = value;
            return false;
        }
        if s.index != IMCR_INDEX {
            return false;
        }
        // Only bit 0 is defined. The rest are dropped rather than latched:
        // §3.6.2.1 gives the register two values and nothing else, and a bit
        // that read back but did nothing would be an invention.
        let imcr = value & IMCR_VIA_APIC;
        let moved = imcr != s.imcr;
        s.imcr = imcr;
        moved
    }
}

/// One output's acknowledge forwarder.
///
/// **Gated on the register**, and it has to be. A local APIC drives the same
/// `INTR` net this device's direct path does, and it never declines a cycle —
/// SDM Vol 3A §10.9 makes the spurious vector the defined answer to "you asked
/// and there is nothing". So on a net with two controllers the one that is not
/// currently routing anything must say so, or whichever the CPU happens to ask
/// first wins and half the machine's interrupts arrive with the wrong vector.
#[derive(Debug)]
struct AckPath {
    regs: Arc<Registers>,
    /// The register value this path is the selected one for.
    when_via_apic: bool,
}

impl IntAck for AckPath {
    fn acknowledge(&self, cycle: IntAckCycle) -> IntAckResponse {
        if self.regs.via_apic() != self.when_via_apic {
            return IntAckResponse::Declined;
        }
        self.regs.upstream.run(cycle)
    }
}

/// The two ports, 0x22 and 0x23.
#[derive(Debug)]
struct Ports(Arc<Registers>);

impl MemOps for Ports {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // No `debug` branch: neither port has a side effect on a read — the
        // index latch is only moved by a write — so a debugger may look freely,
        // and in particular looking cannot move the IMCR.
        *byte = self.0.read(offset);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write here would move the interrupt path out from under
            // the guest, which is exactly what `MemAttrs::debug` forbids.
            return Err(BusError::BadAccess);
        }
        if self.0.write(offset, *value) {
            // Outside the critical section, which is the re-entrancy contract:
            // this drives a pin that reaches a processor.
            self.0.drive_outputs();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

/// The input pin, with the fan-in that makes a shared net correct.
#[derive(Debug)]
struct InputPin {
    regs: Arc<Registers>,
    inputs: FanIn,
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // Wired-OR: a second controller on the same net cannot make the pin
        // drop while the first still asserts (`core::wire`, module docs).
        self.regs.input_level(self.inputs.resolve(Resolve::Or));
    }
}

/// The MP specification's interrupt mode configuration register.
#[derive(Debug)]
pub struct Imcr {
    regs: Arc<Registers>,
    ports: RegionRef,
    /// The two acknowledge forwarders, one per output. **The device owns
    /// them**: what reaches a net is a `Weak`, so one built on the fly would
    /// arrive already dead.
    intr_ack: Arc<AckPath>,
    lint0_ack: Arc<AckPath>,
    /// The sinks handed out by [`Device::sink`], kept alive here: a net holds
    /// only a weak reference to a sink, so the device owns the strong one.
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl Imcr {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Imcr> {
        props.reader().finish()?;
        Ok(Imcr::build())
    }

    fn build() -> Imcr {
        let regs = Arc::new(Registers::new());
        let ports: RegionRef = Arc::new(Region::io(
            "pc.imcr.regs",
            REGISTER_WINDOW_LEN,
            Arc::new(Ports(Arc::clone(&regs))) as Arc<dyn MemOps>,
        ));
        Imcr {
            intr_ack: Arc::new(AckPath {
                regs: Arc::clone(&regs),
                when_via_apic: false,
            }),
            lint0_ack: Arc::new(AckPath {
                regs: Arc::clone(&regs),
                when_via_apic: true,
            }),
            regs,
            ports,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Whether the 8259A's `INT` is currently routed through the local APIC.
    #[must_use]
    pub fn via_apic(&self) -> bool {
        self.regs.via_apic()
    }
}

/// The `pc.imcr` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the MP specification's interrupt mode configuration register, at 0x22/0x23",
    properties: &[],
    construct: |props| Ok(Box::new(Imcr::new(props)?)),
};

/// The error for a pin name this device does not have.
fn unknown_pin(port: &str) -> Error {
    Error::Config {
        at: port.to_string(),
        message: String::from("the IMCR takes `int` in, and drives `intr` and `lint0` out"),
    }
}

impl Device for Imcr {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Back to PIC mode, which is what §3.6.2.1 calls the power-on default
        // and what a warm reset has to reach as well: firmware that rebooted
        // out of symmetric I/O mode would otherwise find its timer tick going
        // to an APIC it has not switched on yet.
        //
        // The remembered *input* level stays. It is not this device's to clear:
        // it is what the 8259A is driving onto the pin, and resetting a gate
        // does not reach across a wire and change what feeds it. Forgetting it
        // would leave both outputs low with an interrupt pending, until the
        // next transition the 8259A happened to make.
        {
            let mut s = self.regs.state.lock();
            *s = State {
                asserted: s.asserted,
                ..State::default()
            };
        }
        self.regs.drive_outputs();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.ports)),
            _ => None,
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port != "int" {
            return None;
        }
        let pin = Arc::new(InputPin {
            regs: Arc::clone(&self.regs),
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: LINE_INT,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let pin = match port {
            "intr" => &self.regs.intr,
            "lint0" => &self.regs.lint0,
            _ => return Err(unknown_pin(port)),
        };
        *pin.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        // Both idle low out of reset, but a snapshot loaded before the sweep
        // can leave either high — and which one is high is the register's
        // doing, so neither can be announced without the other.
        if port == "intr" || port == "lint0" {
            self.regs.drive_outputs();
        }
    }

    fn int_ack(&self, port: &str) -> Option<Arc<dyn IntAck>> {
        // One handler per output, because each answers only while the register
        // selects it — see `AckPath`. The device owns both `Arc`s; the net gets
        // a `Weak`, so one built here would arrive already dead.
        match port {
            "intr" => Some(Arc::clone(&self.intr_ack) as Arc<dyn IntAck>),
            "lint0" => Some(Arc::clone(&self.lint0_ack) as Arc<dyn IntAck>),
            _ => None,
        }
    }

    fn attach_int_ack(&self, port: &str, ack: Weak<dyn IntAck>) {
        if port == "int" {
            self.regs.upstream.attach(ack);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = *self.regs.state.lock();
        w.write_u8(s.index)?;
        w.write_u8(s.imcr)?;
        // The remembered input level, for the reason `reset` keeps it: whether
        // the newly selected output should come up high depends on it, and a
        // snapshot that dropped it would restore a machine with a pending
        // interrupt nobody is being told about.
        w.write_bool(s.asserted)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let state = State {
            index: r.read_u8()?,
            imcr: r.read_u8()? & IMCR_VIA_APIC,
            asserted: r.read_bool()?,
        };
        *self.regs.state.lock() = state;
        self.regs.drive_outputs();
        Ok(())
    }
}

impl Instance for Imcr {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS_NAME`] so a machine description can instantiate it.
///
/// # Errors
///
/// [`Error::Config`] if the name is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Imcr::new(props)?)))
}

/// What the validator should know about this class.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::PortDir;
    ClassSchema::new(CLASS_NAME)
        .region("")
        .region("regs")
        .port("int", PortDir::In)
        .port("intr", PortDir::Out)
        .port("lint0", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::wire::{Wire, WireIdAllocator};
    use core::sync::atomic::{AtomicU32, Ordering};

    /// A pin that remembers the last level it was told.
    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
        }
    }

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) == 1
        }
    }

    /// An 8259A that answers one vector, so an acknowledge that arrives is
    /// distinguishable from one that was dropped.
    #[derive(Debug)]
    struct Stub8259(u32);

    impl IntAck for Stub8259 {
        fn acknowledge(&self, _cycle: IntAckCycle) -> IntAckResponse {
            IntAckResponse::Vector(self.0)
        }
    }

    /// The device with both outputs probed and the input pin wired, plus the
    /// id the imaginary 8259A drives that pin with.
    struct Wired {
        imcr: Imcr,
        intr: Arc<Probe>,
        lint0: Arc<Probe>,
        pin: Arc<dyn WireSink>,
        src: WireId,
    }

    fn wired() -> Wired {
        let imcr = Imcr::build();
        let ids = WireIdAllocator::new();
        let mut probes = Vec::new();
        for port in ["intr", "lint0"] {
            let id = ids.alloc();
            let probe = Arc::new(Probe::default());
            let wire = Wire::builder()
                .source(id)
                .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
                .build_shared();
            Device::connect(&imcr, port, WireSource::new(wire, id)).expect("both outputs exist");
            // What `Machine::sweep` does after realize (§4.3).
            Device::announce(&imcr, port);
            probes.push(probe);
        }
        let src = ids.alloc();
        let pin = Device::sink(&imcr, "int", &[src]).expect("the input pin exists");
        let lint0 = probes.pop().expect("two");
        let intr = probes.pop().expect("two");
        Wired {
            imcr,
            intr,
            lint0,
            pin: pin.sink,
            src,
        }
    }

    impl Wired {
        /// Drive the input pin, as the master 8259A would.
        fn drive_int(&self, level: Level) {
            self.pin.set_level(self.src, LINE_INT, level);
        }
    }

    fn read(imcr: &Imcr, offset: u64) -> u8 {
        let mut byte = [0u8; 1];
        Ports(Arc::clone(&imcr.regs))
            .read(offset, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    fn write(imcr: &Imcr, offset: u64, value: u8) {
        Ports(Arc::clone(&imcr.regs))
            .write(offset, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    /// Select the IMCR and write it, the way §3.6.2.1 says to.
    fn set_imcr(imcr: &Imcr, value: u8) {
        write(imcr, 0, IMCR_INDEX);
        write(imcr, 1, value);
    }

    #[test]
    fn the_power_on_default_is_pic_mode() {
        let w = wired();
        assert!(!w.imcr.via_apic(), "MP spec 3.6.2.1: the default is zero");
        w.drive_int(Level::High);
        assert!(w.intr.high(), "the 8259A reaches the processor directly");
        assert!(!w.lint0.high(), "and the APIC is bypassed");
    }

    #[test]
    fn writing_one_moves_the_line_to_the_apic_and_writing_zero_moves_it_back() {
        let w = wired();
        w.drive_int(Level::High);

        set_imcr(&w.imcr, 0x01);
        assert!(w.imcr.via_apic());
        assert!(!w.intr.high(), "the direct path is released");
        assert!(w.lint0.high(), "and LINT0 picks the pending interrupt up");

        set_imcr(&w.imcr, 0x00);
        assert!(w.intr.high(), "and back, with the interrupt still pending");
        assert!(!w.lint0.high());
    }

    #[test]
    fn the_register_answers_only_when_it_is_selected() {
        let w = wired();
        set_imcr(&w.imcr, 0x01);
        write(&w.imcr, 0, IMCR_INDEX);
        assert_eq!(read(&w.imcr, 1), 0x01);
        assert_eq!(read(&w.imcr, 0), IMCR_INDEX, "the index reads back");

        // Any other index selects nothing at all: the data port answers ones
        // and a write to it changes nothing.
        write(&w.imcr, 0, 0x71);
        assert_eq!(read(&w.imcr, 1), 0xff);
        write(&w.imcr, 1, 0x00);
        assert!(
            w.imcr.via_apic(),
            "a write to an unselected register moved the mode"
        );
    }

    #[test]
    fn only_bit_zero_of_the_register_exists() {
        let w = wired();
        set_imcr(&w.imcr, 0xfe);
        assert!(
            !w.imcr.via_apic(),
            "bit 0 clear is PIC mode whatever else is set"
        );
        write(&w.imcr, 0, IMCR_INDEX);
        assert_eq!(read(&w.imcr, 1), 0x00, "and the rest did not latch");
    }

    #[test]
    fn the_acknowledge_reaches_the_8259a_through_whichever_output_is_selected() {
        let w = wired();
        let pic: Arc<dyn IntAck> = Arc::new(Stub8259(0x08));
        Device::attach_int_ack(&w.imcr, "int", Arc::downgrade(&pic));
        let intr = Device::int_ack(&w.imcr, "intr").expect("the direct path vectors");
        let lint0 = Device::int_ack(&w.imcr, "lint0").expect("the APIC path vectors");

        // PIC mode: the direct path carries the vector and the APIC path
        // declines, so a local APIC sharing the processor's `INTR` net cannot
        // answer a cycle this device is the one driving.
        assert_eq!(
            intr.acknowledge(IntAckCycle::vector_only()),
            IntAckResponse::Vector(0x08)
        );
        assert_eq!(
            lint0.acknowledge(IntAckCycle::vector_only()),
            IntAckResponse::Declined
        );

        set_imcr(&w.imcr, 0x01);
        assert_eq!(
            intr.acknowledge(IntAckCycle::vector_only()),
            IntAckResponse::Declined,
            "in APIC mode this device is not driving the processor's pin"
        );
        assert_eq!(
            lint0.acknowledge(IntAckCycle::vector_only()),
            IntAckResponse::Vector(0x08)
        );

        assert!(
            Device::int_ack(&w.imcr, "int").is_none(),
            "an input pin offers nothing"
        );
    }

    #[test]
    fn a_debug_write_cannot_move_the_interrupt_path() {
        let w = wired();
        let ops = Ports(Arc::clone(&w.imcr.regs));
        assert!(ops.write(0, &[IMCR_INDEX], MemAttrs::DEBUG).is_err());
        assert!(ops.write(1, &[0x01], MemAttrs::DEBUG).is_err());
        assert!(!w.imcr.via_apic(), "and nothing moved");
        // A debug *read* is the same read, because neither port has a side
        // effect: the index latch only moves on a write.
        let mut byte = [0u8; 1];
        ops.read(0, &mut byte, MemAttrs::DEBUG).expect("legal");
        assert_eq!(byte[0], 0x00, "nothing has been selected");
    }

    #[test]
    fn a_reset_returns_to_pic_mode_without_forgetting_the_pending_interrupt() {
        let w = wired();
        set_imcr(&w.imcr, 0x01);
        w.drive_int(Level::High);
        assert!(w.lint0.high());

        Device::reset(&w.imcr, ResetKind::Warm);
        assert!(
            !w.imcr.via_apic(),
            "a warm reset is a power-on for this latch"
        );
        assert!(
            w.intr.high(),
            "and the pending interrupt found the new path"
        );
        assert!(!w.lint0.high());
    }

    #[test]
    fn an_access_that_is_not_a_single_byte_is_refused() {
        let w = wired();
        let ops = Ports(Arc::clone(&w.imcr.regs));
        let mut two = [0u8; 2];
        assert!(ops.read(0, &mut two, MemAttrs::DEFAULT).is_err());
        assert!(ops.write(0, &[0, 0], MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn a_pin_this_device_does_not_drive_is_a_configuration_error() {
        let imcr = Imcr::build();
        let id = WireIdAllocator::new().alloc();
        let wire = Wire::builder().source(id).build_shared();
        assert!(Device::connect(&imcr, "nmi", WireSource::new(wire, id)).is_err());
        assert!(Device::sink(&imcr, "nmi", &[]).is_none());
        assert!(Device::region(&imcr, "nmi").is_none());
        assert!(Device::region(&imcr, "regs").is_some());
    }

    #[test]
    fn a_snapshot_round_trips_every_bit_of_architectural_state() {
        let w = wired();
        write(&w.imcr, 0, 0x71);
        set_imcr(&w.imcr, 0x01);
        w.drive_int(Level::High);

        let image = |dev: &Imcr| {
            let mut shape = MachineShape::new();
            shape.add_device("imcr", CLASS.name).unwrap();
            let mut out = StateWriter::new(shape);
            {
                let mut chunk = out.chunk("imcr", CLASS.name, CLASS.version).unwrap();
                Device::save(dev, &mut chunk).unwrap();
            }
            out.to_vec().unwrap()
        };

        let first = image(&w.imcr);
        let r = wired();
        let reader = StateReader::new(&first).unwrap();
        let chunk = reader
            .load("imcr", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        Device::load(&r.imcr, &mut chunk.reader()).unwrap();

        assert_eq!(image(&r.imcr), first, "the two images are identical");
        assert!(r.imcr.via_apic(), "the mode came back");
        assert!(r.lint0.high(), "and so did the interrupt it was routing");
        assert!(!r.intr.high());
    }

    #[test]
    fn properties_are_checked_rather_than_ignored() {
        assert!(Imcr::new(&Props::new()).is_ok());
        assert!(Imcr::new(&Props::new().with("mode", "apic")).is_err());
    }
}
