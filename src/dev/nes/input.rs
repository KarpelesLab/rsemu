//! The two controller ports at `$4016` and `$4017`.
//!
//! Sources: the NESdev wiki, [Standard
//! controller](https://www.nesdev.org/wiki/Standard_controller), [Controller
//! port registers](https://www.nesdev.org/wiki/Controller_port_registers) and
//! [Input devices](https://www.nesdev.org/wiki/Input_devices).
//!
//! # What the hardware is
//!
//! Almost nothing. Each port carries a clock line, a latch line and a data
//! line. Inside a standard controller a 4021 shift register samples all eight
//! buttons in parallel while the latch line is high and shifts one bit out on
//! every subsequent clock. The console drives both latch lines from bit 0 of
//! `$4016` and clocks a port's register by *reading* that port.
//!
//! ```text
//!   $4016 write   bit 0 -> OUT0, the latch (strobe) line of both ports
//!   $4016 read    bit 0 <- controller 1 serial data; bits 7-5 open bus
//!   $4017 read    bit 0 <- controller 2 serial data; bits 7-5 open bus
//! ```
//!
//! Three consequences follow, and software depends on all three:
//!
//! * **While the strobe is high the register reloads continuously**, so every
//!   read of `$4016` returns the A button and nothing advances.
//! * **The bit order is the register's output order**: A, B, Select, Start, Up,
//!   Down, Left, Right — which is why [`buttons`] numbers A as bit 7.
//! * **After the eighth read an official NES pad returns 1 forever** (until the
//!   next strobe), because the 4021's serial input is tied high. A Famicom's
//!   hardwired pads return 0 instead. The NES behaviour is the one modelled;
//!   `AccuracyCoin` and most late software rely on it to count the pads.
//!
//! # `$4017` is shared with the APU
//!
//! On the real chip `$4017` is the controller-2 port on a **read** and the APU
//! frame counter on a **write**. `core::space` routes a whole mapping to one
//! device, so a machine description can give `$4017` to the APU or to this
//! device but not to both, and the shipped NES machines give it to the APU —
//! losing the frame counter is much worse than losing player two. [`PORT2`] is
//! published regardless, so a machine that wants controller 2 more than it
//! wants the frame IRQ can map it. Splitting the address properly wants a
//! read/write-split mapping in `core::space`, which is a framework change and
//! not this device's to make.
//!
//! # Where the buttons come from
//!
//! Through [`pads`], a process-wide table of named pad ports — the same shape
//! as [`crate::host::chardev::ports`], and for the same reason: a *name* is the
//! only thing that can travel from a machine description into a device
//! constructor. The host (or a test) opens the port by name and stores a byte;
//! the device reads it when the guest strobes. Input crossing into the machine
//! by exactly one narrow, named seam is what makes it recordable
//! (`CLAUDE.md`, determinism).

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU8, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::machine::realize::Instance;

/// The class name a machine description would use.
const CLASS_NAME: &str = "nes.ports";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The name of the region that decodes `$4016`.
pub const PORT1: &str = "port1";

/// The name of the region that decodes `$4017` — see the [module docs](self)
/// for why the shipped machines do not map it.
pub const PORT2: &str = "port2";

/// The pad port a machine gets when its description names none.
pub const DEFAULT_PAD_PORT: &str = "nes-pads";

/// Which bits of a `$4016`/`$4017` read the port does *not* drive.
///
/// Bits 7-5 are not connected to the controller at all: they float, and what
/// floats is whatever the master last left on the data bus — for an ordinary
/// `LDA $4016` that is `$40`, the high byte of the address it just put out
/// (NESdev, "Open bus behavior"). Bits 4-0 belong to the port, and on an
/// NES-001 only D0, D3 and D4 have anything on them.
const OPEN_BUS_BITS: u8 = 0xe0;

/// Controller button bits, in the shift register's output order.
///
/// Bit 7 is what the *first* read after a strobe returns. That is the order the
/// hardware shifts in and the order every NES read routine assembles, so it is
/// the order the host seam speaks.
pub mod buttons {
    /// The A button — the first bit out.
    pub const A: u8 = 0x80;
    /// The B button.
    pub const B: u8 = 0x40;
    /// Select.
    pub const SELECT: u8 = 0x20;
    /// Start.
    pub const START: u8 = 0x10;
    /// D-pad up.
    pub const UP: u8 = 0x08;
    /// D-pad down.
    pub const DOWN: u8 = 0x04;
    /// D-pad left.
    pub const LEFT: u8 = 0x02;
    /// D-pad right — the last bit out.
    pub const RIGHT: u8 = 0x01;
    /// Nothing held.
    pub const NONE: u8 = 0x00;
}

// ---------------------------------------------------------------------------
// the host seam
// ---------------------------------------------------------------------------

/// What the host holds: the current button state of one console's two pads.
///
/// Level, not events. The console samples this whenever the guest strobes, so a
/// button is "held" for as long as the host leaves the bit set — which is also
/// what makes the seam replayable: the state at each sample is the whole of the
/// input.
#[derive(Debug, Default)]
pub struct Pad {
    /// Controller 1 and controller 2, in [`buttons`] order.
    ///
    /// Atomics rather than a lock: the host thread writes and the emulation
    /// thread reads, on the guest's hot path, and there is nothing to keep
    /// consistent between the two bytes.
    held: [AtomicU8; 2],
}

impl Pad {
    /// A pad port with nothing held.
    #[must_use]
    pub fn new() -> Pad {
        Pad::default()
    }

    /// Set what controller `port` (0 or 1) is holding.
    ///
    /// Out-of-range ports are ignored: a host that asks for controller 5 has a
    /// bug, but dropping the press is better than panicking inside a frame.
    pub fn set(&self, port: usize, held: u8) {
        if let Some(cell) = self.held.get(port) {
            cell.store(held, Ordering::Relaxed);
        }
    }

    /// What controller `port` is holding. Nothing, for a port that does not
    /// exist.
    #[must_use]
    pub fn get(&self, port: usize) -> u8 {
        self.held
            .get(port)
            .map_or(buttons::NONE, |c| c.load(Ordering::Relaxed))
    }
}

/// The process-wide table of named pad ports.
///
/// See the [module docs](self) for why a name is the only thing that can travel
/// from a machine description into a device constructor, and
/// [`crate::host::chardev::ports`] for the same pattern applied to a terminal.
pub mod pads {
    use super::Pad;
    use alloc::collections::BTreeMap;
    use alloc::string::{String, ToString};
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::sync::{LockRank, Mutex};

    /// Name to pad. `BTreeMap`, so listing is in name order rather than hash
    /// order (`CLAUDE.md`, determinism).
    static TABLE: Mutex<BTreeMap<String, Arc<Pad>>> =
        Mutex::with_rank(LockRank::LEAF, BTreeMap::new());

    /// The pad port called `name`, creating it if this is the first mention.
    ///
    /// Both ends call this: the device during construction, the host before it
    /// starts pressing buttons. Whichever runs first makes the port.
    #[must_use]
    pub fn open(name: &str) -> Arc<Pad> {
        let mut table = TABLE.lock();
        if let Some(pad) = table.get(name) {
            return Arc::clone(pad);
        }
        let pad = Arc::new(Pad::new());
        table.insert(name.to_string(), Arc::clone(&pad));
        pad
    }

    /// The pad port called `name`, if it has been opened.
    #[must_use]
    pub fn get(name: &str) -> Option<Arc<Pad>> {
        TABLE.lock().get(name).map(Arc::clone)
    }

    /// Forget `name`, reporting whether there was one.
    ///
    /// Anything still holding the `Arc` keeps working; this only removes the
    /// table's own reference, so a later [`open`] of the same name is a fresh
    /// port. For tests that want the name back.
    pub fn close(name: &str) -> bool {
        TABLE.lock().remove(name).is_some()
    }

    /// Every open port's name, in order.
    #[must_use]
    pub fn names() -> Vec<String> {
        TABLE.lock().keys().cloned().collect()
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The latch and the two shift registers — everything `$4016` writes touch.
///
/// `Default` is power-on: the strobe line low and the registers holding
/// whatever the last latch left. Zero is the reproducible choice, and
/// determinism is the non-negotiable one (`ROADMAP.md` §0).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct Regs {
    /// OUT0. While set, both registers reload from the pads continuously.
    strobe: bool,
    /// The two shift registers, MSB first: bit 7 is the next bit out.
    ///
    /// Shifting in a 1 at the bottom is what gives an official NES pad its
    /// "1 forever after the eighth read" behaviour, for free and for the same
    /// reason the hardware has it — the 4021's serial input is tied high.
    shift: [u8; 2],
}

/// What the device and its two memory ports both hold.
struct Shared {
    /// Where the buttons come from.
    pad: Arc<Pad>,
    /// The port's own registers. `DEVICE`-ranked and never held across an
    /// outward call — there are none to make.
    regs: Mutex<Regs>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared").field("regs", &self.regs).finish()
    }
}

impl Shared {
    /// Reload both registers from the pads, as a high latch line does.
    fn latch(&self, regs: &mut Regs) {
        regs.shift = [self.pad.get(0), self.pad.get(1)];
    }

    /// One read of a port, as a byte on the CPU data bus.
    ///
    /// `advance` is false for a debug read: a monitor looking at `$4016` must
    /// not clock the controller (`ROADMAP.md` §15, invariant 5).
    fn read_port(&self, port: usize, bus: u8, advance: bool) -> u8 {
        let mut regs = self.regs.lock();
        if regs.strobe {
            // Latched continuously while the line is high, so every read is the
            // A button and nothing shifts.
            self.latch(&mut regs);
        }
        let bit = regs.shift[port] >> 7;
        if advance && !regs.strobe {
            regs.shift[port] = (regs.shift[port] << 1) | 1;
        }
        (bus & OPEN_BUS_BITS) | bit
    }

    /// A write of `value` to `$4016`.
    fn write_strobe(&self, value: u8) {
        let mut regs = self.regs.lock();
        let was = regs.strobe;
        regs.strobe = value & 1 != 0;
        // The 4021 is *transparent* while the latch line is high, not
        // edge-triggered: it tracks the buttons the whole time and freezes them
        // on the falling edge. So sample both while the line is high and on the
        // write that takes it low, or a game that strobes, waits and then
        // releases would read the buttons as they were at the rising edge.
        if regs.strobe || was {
            self.latch(&mut regs);
        }
    }
}

/// The NES's two controller ports.
///
/// Cloneable handles onto one piece of hardware: [`Device::region`] hands out
/// the two one-byte apertures while the machine keeps the device.
#[derive(Debug)]
pub struct NesPorts {
    shared: Arc<Shared>,
    /// `$4016`, built once at construction so two `map` statements naming it
    /// get one region.
    port1: RegionRef,
    /// `$4017`.
    port2: RegionRef,
    /// The pad port's name, for diagnostics.
    port_name: String,
}

impl NesPorts {
    /// Validate properties and allocate. Performs no outward action.
    ///
    /// Properties: `pads`, the name of the host pad port to read
    /// ([`DEFAULT_PAD_PORT`] if absent).
    ///
    /// # Errors
    ///
    /// [`crate::Error::Property`] for an unknown or ill-typed property.
    pub fn new(props: &Props) -> Result<NesPorts> {
        let mut r = props.reader();
        let name: String = r.or("pads", String::from(DEFAULT_PAD_PORT))?;
        r.finish()?;
        Ok(NesPorts::with_pad(pads::open(&name), name))
    }

    /// Build one against a pad port held directly, for a caller assembling a
    /// NES without the DSL.
    #[must_use]
    pub fn with_pad(pad: Arc<Pad>, port_name: String) -> NesPorts {
        let shared = Arc::new(Shared {
            pad,
            regs: Mutex::with_rank(LockRank::DEVICE, Regs::default()),
        });
        let port = |index: usize, name: &'static str| {
            Arc::new(MmioRegion::io(
                name,
                1,
                Arc::new(PortWindow {
                    shared: Arc::clone(&shared),
                    index,
                }) as Arc<dyn MemOps>,
            )) as RegionRef
        };
        NesPorts {
            port1: port(0, "nes.ports.4016"),
            port2: port(1, "nes.ports.4017"),
            shared,
            port_name,
        }
    }

    /// The pad port this device reads its buttons from.
    #[must_use]
    pub fn pad(&self) -> &Arc<Pad> {
        &self.shared.pad
    }

    /// The name that pad port is registered under.
    #[must_use]
    pub fn pad_name(&self) -> &str {
        &self.port_name
    }
}

/// One of the two one-byte windows onto a [`NesPorts`].
#[derive(Debug)]
struct PortWindow {
    shared: Arc<Shared>,
    /// 0 for `$4016`, 1 for `$4017`.
    index: usize,
}

impl MemOps for PortWindow {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let ([byte], 0) = (dst, offset) else {
            return Err(BusError::BadAccess);
        };
        *byte = self.shared.read_port(self.index, attrs.bus, !attrs.debug);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let ([value], 0) = (src, offset) else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write would latch the pads for real; the monitor has to
            // go through the device's own API to say it meant it.
            return Ok(());
        }
        // Only `$4016` carries OUT0. A write to `$4017` is the APU frame
        // counter's, and the port hardware ignores it — see the module docs.
        if self.index == 0 {
            self.shared.write_strobe(*value);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

impl Device for NesPorts {
    fn class(&self) -> &'static DeviceClass {
        &PORTS_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. The ports drive no line, take no clock, and are
        // placed by `map` statements like every other aperture.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: /RES clears the output latch, and the shift registers
        // hold nothing a reset would preserve. What the *host* is holding is
        // not the machine's state and is deliberately untouched — releasing
        // the player's thumb on reset would be a strange thing to model.
        *self.shared.regs.lock() = Regs::default();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let regs = *self.shared.regs.lock();
        w.write_bool(regs.strobe)?;
        w.write_u8(regs.shift[0])?;
        w.write_u8(regs.shift[1])
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let strobe = r.read_bool()?;
        let first = r.read_u8()?;
        let second = r.read_u8()?;
        *self.shared.regs.lock() = Regs {
            strobe,
            shift: [first, second],
        };
        Ok(())
    }

    /// One of the two ports, by name.
    ///
    /// The empty name gets nothing: a device with two identical one-byte
    /// apertures has no "the" region, and quietly handing back `$4016` would
    /// leave a machine that looked complete with player two unmapped.
    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            PORT1 => Some(Arc::clone(&self.port1)),
            PORT2 => Some(Arc::clone(&self.port2)),
            _ => None,
        }
    }
}

/// The machine layer's half: the ports take no clock, no space and no pin, so
/// binding them is nothing at all.
///
/// The `impl` still has to exist — a class with no [`Instance`] publishes no
/// regions to the machine graph, and `map cpubus 0x4016 = ports.port1` would be
/// told the class publishes none.
impl Instance for NesPorts {}

/// The properties [`PORTS_CLASS`] accepts.
static PORTS_PROPERTIES: &[PropertySpec] = &[PropertySpec {
    name: "pads",
    kind: ValueKind::Str,
    required: false,
    summary: "the host pad port to read buttons from, by name (default \"nes-pads\")",
}];

/// The device class, as `nes.ports` in a machine description.
pub static PORTS_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "NES controller ports ($4016/$4017): the OUT0 latch and two 8-bit shift registers",
    properties: PORTS_PROPERTIES,
    construct: |props| Ok(Box::new(NesPorts::new(props)?) as Box<dyn Device>),
};

/// Add [`PORTS_CLASS`] to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if the class name is already taken.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&PORTS_CLASS)
}

/// Bind [`PORTS_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(NesPorts::new(props)?)))
}

/// What the validator should know about `nes.ports`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("pads", ValueKind::Str))
        .region(PORT1)
        .region(PORT2)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::AddressSpace;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    /// A device on its own pad port, so tests cannot see each other's buttons
    /// through the process-wide table.
    fn ports(name: &str) -> NesPorts {
        pads::close(name);
        NesPorts::with_pad(pads::open(name), String::from(name))
    }

    /// Strobe high then low, then read eight bits out of port 1.
    fn sequence(p: &NesPorts) -> u8 {
        p.shared.write_strobe(1);
        p.shared.write_strobe(0);
        let mut out = 0u8;
        for _ in 0..8 {
            out = (out << 1) | (p.shared.read_port(0, 0x40, true) & 1);
        }
        out
    }

    #[test]
    fn the_shift_register_reports_the_buttons_a_first() {
        let p = ports("test-order");
        p.pad().set(0, buttons::START | buttons::RIGHT);
        // Read back in the order the hardware shifts: the byte reassembles
        // exactly as it was handed in, which is the whole contract of the bit
        // order in `buttons`.
        assert_eq!(sequence(&p), buttons::START | buttons::RIGHT);

        p.pad().set(0, buttons::A);
        assert_eq!(sequence(&p), buttons::A);
        p.pad().set(0, buttons::NONE);
        assert_eq!(sequence(&p), 0);
    }

    #[test]
    fn an_official_pad_reads_one_after_the_eighth_read() {
        let p = ports("test-ninth");
        p.pad().set(0, buttons::NONE);
        p.shared.write_strobe(1);
        p.shared.write_strobe(0);
        for _ in 0..8 {
            assert_eq!(p.shared.read_port(0, 0x40, true) & 1, 0);
        }
        // The 4021's serial input is tied high on an NES pad, so everything
        // after the eighth clock is a 1. Software counts pads with this.
        for _ in 0..4 {
            assert_eq!(p.shared.read_port(0, 0x40, true) & 1, 1);
        }
    }

    #[test]
    fn a_high_strobe_reloads_forever() {
        let p = ports("test-strobe");
        p.pad().set(0, buttons::A);
        p.shared.write_strobe(1);
        // Latched continuously: every read is A, and nothing shifts past it.
        for _ in 0..16 {
            assert_eq!(p.shared.read_port(0, 0x40, true) & 1, 1);
        }
        // Release the strobe and what was latched is the state at the falling
        // edge, not the state at the rising one.
        p.pad().set(0, buttons::NONE);
        p.shared.write_strobe(0);
        assert_eq!(
            p.shared.read_port(0, 0x40, true) & 1,
            0,
            "A was released first"
        );
    }

    #[test]
    fn the_upper_bits_are_open_bus() {
        let p = ports("test-openbus");
        p.pad().set(0, buttons::A);
        p.shared.write_strobe(1);
        p.shared.write_strobe(0);
        // Bits 7-5 come from the CPU's own bus, and for a read of $4016 that
        // is the high byte of the address.
        assert_eq!(p.shared.read_port(0, 0x40, true), 0x40 | 1);
    }

    #[test]
    fn the_two_ports_are_independent() {
        let p = ports("test-two");
        p.pad().set(0, buttons::A);
        p.pad().set(1, buttons::RIGHT);
        p.shared.write_strobe(1);
        p.shared.write_strobe(0);
        // Port 1 shifts A out first; port 2's A is clear and its Right is last.
        assert_eq!(p.shared.read_port(0, 0x40, true) & 1, 1);
        assert_eq!(p.shared.read_port(1, 0x40, true) & 1, 0);
        for _ in 0..6 {
            let _ = p.shared.read_port(1, 0x40, true);
        }
        assert_eq!(
            p.shared.read_port(1, 0x40, true) & 1,
            1,
            "Right, the eighth bit"
        );
    }

    #[test]
    fn a_debug_read_does_not_clock_the_controller() {
        let p = ports("test-debug");
        p.pad().set(0, buttons::A);
        p.shared.write_strobe(1);
        p.shared.write_strobe(0);
        for _ in 0..8 {
            assert_eq!(p.shared.read_port(0, 0x40, false), 0x40 | 1, "still A");
        }
        // And the guest's own first read still gets the first bit.
        assert_eq!(p.shared.read_port(0, 0x40, true) & 1, 1);
    }

    #[test]
    fn the_ports_answer_through_an_address_space() {
        let p = ports("test-space");
        p.pad().set(0, buttons::SELECT);
        let space = AddressSpace::new("cpu", 16);
        {
            let mut topo = space.topology();
            topo.map(p.region(PORT1).expect("port1"), 0x4016)
                .expect("maps");
            topo.map(p.region(PORT2).expect("port2"), 0x4017)
                .expect("maps");
        }
        let wr = |v: u64| {
            space
                .write(0x4016, Width::U8, v, MemAttrs::DEFAULT)
                .expect("writable")
        };
        let rd = || {
            space
                .read(0x4016, Width::U8, MemAttrs::DEFAULT)
                .expect("readable") as u8
        };
        wr(1);
        wr(0);
        let mut out = 0u8;
        for _ in 0..8 {
            out = (out << 1) | (rd() & 1);
        }
        assert_eq!(out, buttons::SELECT);

        // A debug read of the same port disturbs nothing.
        let before = space
            .read(0x4016, Width::U8, MemAttrs::DEBUG)
            .expect("readable");
        assert_eq!(
            space
                .read(0x4016, Width::U8, MemAttrs::DEBUG)
                .expect("readable"),
            before
        );

        // The empty region name gets nothing: there is no "the" port.
        assert!(p.region("").is_none());
    }

    #[test]
    fn state_round_trips() {
        let p = ports("test-state");
        p.pad().set(0, buttons::B | buttons::DOWN);
        p.shared.write_strobe(1);
        p.shared.write_strobe(0);
        let _ = p.shared.read_port(0, 0x40, true);

        let mut shape = MachineShape::new();
        shape.add_device("ports", CLASS_NAME).expect("unique path");
        let mut writer = StateWriter::new(shape);
        let mut chunk = writer
            .chunk("ports", CLASS_NAME, STATE_VERSION)
            .expect("one chunk");
        p.save(&mut chunk).expect("saves");
        let bytes = writer.to_vec().expect("encodes");

        let other = ports("test-state-2");
        let reader = StateReader::new(&bytes).expect("decodes");
        let chunk = reader
            .load("ports", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .expect("finds the chunk");
        other.load(&mut chunk.reader()).expect("loads");
        // Copied out one at a time: two `DEVICE`-ranked locks held together is
        // a lock-order violation, and `core::sync` says so in debug builds.
        let restored = *other.shared.regs.lock();
        let original = *p.shared.regs.lock();
        assert_eq!(restored, original);

        // And it keeps shifting from where the original stood.
        for _ in 0..7 {
            assert_eq!(
                other.shared.read_port(0, 0x40, true) & 1,
                p.shared.read_port(0, 0x40, true) & 1
            );
        }
    }

    #[test]
    fn a_reset_clears_the_latch_but_not_the_players_thumb() {
        let p = ports("test-reset");
        p.pad().set(0, buttons::A);
        p.shared.write_strobe(1);
        p.reset(ResetKind::Cold);
        assert!(!p.shared.regs.lock().strobe);
        assert_eq!(p.pad().get(0), buttons::A, "the host still holds A");
        assert_eq!(sequence(&p), buttons::A);
    }

    #[test]
    fn the_pad_table_hands_the_same_port_to_both_ends() {
        pads::close("test-table");
        let host = pads::open("test-table");
        let device = NesPorts::new(&Props::new().with("pads", "test-table")).expect("constructs");
        assert_eq!(device.pad_name(), "test-table");
        host.set(0, buttons::UP);
        assert_eq!(sequence(&device), buttons::UP);
        assert!(pads::names().iter().any(|n| n == "test-table"));
        assert!(pads::get("test-table").is_some());
        assert!(pads::close("test-table"));
    }

    #[test]
    fn an_unknown_property_is_refused() {
        let e = NesPorts::new(&Props::new().with("padz", "x")).expect_err("typo");
        assert!(alloc::format!("{e}").contains("padz"), "{e}");
    }
}
