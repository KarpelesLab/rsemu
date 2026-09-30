//! One R-Car GPIO bank: thirty-two pins, their direction, their output latch,
//! and edge- or level-sensitive interrupts on the ones configured for it.
//!
//! # Register map
//!
//! | Offset | Name | Here |
//! | --- | --- | --- |
//! | `0x00` | `IOINTSEL` | a set bit makes the pin an interrupt input |
//! | `0x04` | `INOUTSEL` | a set bit makes the pin an output |
//! | `0x08` | `OUTDT` | the output latch |
//! | `0x0c` | `INDT` | the pins: inputs as driven, outputs as latched |
//! | `0x10` | `INTDT` | interrupts detected (read-only) |
//! | `0x14` | `INTCLR` | write 1 to clear an `INTDT` bit (write-only) |
//! | `0x18` | `INTMSK` | read: the mask; write 1 to mask |
//! | `0x1c` | `MSKCLR` | write 1 to unmask (write-only) |
//! | `0x20` | `POSNEG` | a set bit detects low levels and falling edges |
//! | `0x24` | `EDGLEVEL` | a set bit detects edges rather than levels |
//! | `0x28` | `FILONOFF` | the input filter: stored, and inert |
//!
//! The mask registers' reset state is everything masked.
//!
//! # Pins
//!
//! Each pin is a wire both ways: `in0`–`in31` are inputs the machine drives
//! (a card-detect switch, a sub-processor's handshake line), and
//! `out0`–`out31` follow `OUTDT` while the pin is an output. A pin nothing
//! drives reads the `inputs` property's bit for it — the board's pull-ups and
//! pull-downs, which is what an unconnected pin reads on the silicon too. A
//! wired pin reads its net, which idles low until its driver says otherwise.
//! `irq` is the bank's one interrupt, asserted while an unmasked `INTDT` bit
//! is set.
//!
//! # Sources
//!
//! The GPIO chapter of the Renesas R-Car hardware manual for the register
//! names and the interrupt model, and a black-box trace of the Alphard navi's
//! U-Boot and kernel for which registers they use and how. No kernel or
//! emulator source was consulted.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::Width;
use crate::core::wire::{Level, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.gpio";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space one bank answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x100;

/// The bank's interrupt output.
pub const IRQ_PIN: &str = "irq";

const IOINTSEL: u64 = 0x00;
const INOUTSEL: u64 = 0x04;
const OUTDT: u64 = 0x08;
const INDT: u64 = 0x0c;
const INTDT: u64 = 0x10;
const INTCLR: u64 = 0x14;
const INTMSK: u64 = 0x18;
const MSKCLR: u64 = 0x1c;
const POSNEG: u64 = 0x20;
const EDGLEVEL: u64 = 0x24;
const FILONOFF: u64 = 0x28;

/// The guest-visible state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct State {
    iointsel: u32,
    inoutsel: u32,
    outdt: u32,
    /// Latched edge detections. Level detections are computed, not latched.
    edges: u32,
    intmsk: u32,
    posneg: u32,
    edglevel: u32,
    filonoff: u32,
    /// The input pins as the machine drives them.
    pins: u32,
    /// Which input pins something drives; the rest read `pull`.
    driven: u32,
}

impl State {
    fn reset(pull: u32) -> State {
        State {
            intmsk: u32::MAX,
            pins: pull,
            ..State::default()
        }
    }

    /// What `INDT` reads: inputs as driven, outputs as latched.
    fn indt(&self) -> u32 {
        (self.pins & !self.inoutsel) | (self.outdt & self.inoutsel)
    }

    /// What `INTDT` reads: latched edges on edge pins, and live levels on
    /// level pins, only for pins that are interrupt inputs.
    fn intdt(&self) -> u32 {
        let active = self.pins ^ self.posneg; // a set bit is the detected level
        let level = active & !self.edglevel;
        ((self.edges & self.edglevel) | level) & self.iointsel & !self.inoutsel
    }

    fn irq(&self) -> bool {
        self.intdt() & !self.intmsk != 0
    }

    /// The machine drove input `line` to `high`: record it, and latch an
    /// edge if one of the configured polarity just happened.
    fn drive(&mut self, line: u32, high: bool) {
        let bit = 1u32 << line;
        self.driven |= bit;
        let was = self.pins & bit != 0;
        if was == high {
            return;
        }
        if high {
            self.pins |= bit;
        } else {
            self.pins &= !bit;
        }
        // POSNEG clear: rising edges; set: falling edges.
        let rising = high;
        let wanted = self.posneg & bit == 0;
        if rising == wanted {
            self.edges |= bit;
        }
    }
}

struct Registers {
    pull: u32,
    state: Mutex<State>,
    irq: Mutex<Option<WireSource>>,
    outs: Mutex<[Option<WireSource>; 32]>,
    /// The output levels last driven, so a write that changes nothing about a
    /// pin drives nothing.
    driven_out: Mutex<Option<u32>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Registers")
            .field("pull", &self.pull)
            .finish_non_exhaustive()
    }
}

impl Registers {
    /// Drive the interrupt and every output pin from the state. Called with
    /// no lock held.
    fn refresh(&self, force: bool) {
        let (irq, outs, dir) = {
            let s = self.state.lock();
            (s.irq(), s.outdt, s.inoutsel)
        };
        let irq_out = self.irq.lock().clone();
        if let Some(out) = irq_out {
            out.set(Level::from_bool(irq));
        }
        // An input pin's output wire rests low; an output drives its latch.
        let levels = outs & dir;
        let changed = {
            let mut last = self.driven_out.lock();
            let changed = match *last {
                Some(prev) if !force => prev ^ levels,
                _ => u32::MAX,
            };
            *last = Some(levels);
            changed
        };
        if changed == 0 {
            return;
        }
        let pins = self.outs.lock().clone();
        for (n, pin) in pins.iter().enumerate() {
            if changed & (1 << n) != 0
                && let Some(out) = pin
            {
                out.set(Level::from_bool(levels & (1 << n) != 0));
            }
        }
    }

    fn read_reg(&self, s: &State, offset: u64) -> u32 {
        match offset {
            IOINTSEL => s.iointsel,
            INOUTSEL => s.inoutsel,
            OUTDT => s.outdt,
            INDT => s.indt(),
            INTDT => s.intdt(),
            INTMSK => s.intmsk,
            POSNEG => s.posneg,
            EDGLEVEL => s.edglevel,
            FILONOFF => s.filonoff,
            // INTCLR and MSKCLR are write-only and read as zero.
            _ => 0,
        }
    }

    fn write_reg(&self, s: &mut State, offset: u64, v: u32) {
        match offset {
            IOINTSEL => s.iointsel = v,
            INOUTSEL => s.inoutsel = v,
            OUTDT => s.outdt = v,
            INTCLR => s.edges &= !v,
            INTMSK => s.intmsk |= v,
            MSKCLR => s.intmsk &= !v,
            POSNEG => s.posneg = v,
            EDGLEVEL => s.edglevel = v,
            FILONOFF => s.filonoff = v,
            _ => {}
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        // Reads have no side effects, so a debugger's read is the guest's.
        let v = self.read_reg(&self.state.lock(), offset & !3);
        dst.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        let v = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        self.write_reg(&mut self.state.lock(), offset & !3, v);
        self.refresh(false);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, crate::core::value::Endian::Little)
    }
}

/// One input pin's sink.
struct InputPin {
    regs: Arc<Registers>,
}

impl WireSink for InputPin {
    fn set_level(&self, _src: WireId, line: u32, level: Level) {
        self.regs.state.lock().drive(line & 31, level.is_high());
        self.regs.refresh(false);
    }
}

/// One GPIO bank.
#[derive(Debug)]
pub struct Gpio {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The input sinks handed out, kept alive here: a net holds its sinks
    /// weakly.
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl fmt::Debug for InputPin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("InputPin")
    }
}

impl Gpio {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a property of the wrong kind or one this class
    /// does not know.
    pub fn new(props: &Props) -> Result<Gpio> {
        let mut r = props.reader();
        let pull = r.or_range("inputs", 0u64, 0..=u64::from(u32::MAX))? as u32;
        r.finish()?;
        Ok(Gpio::with_pull(pull))
    }

    /// A bank whose undriven inputs read `pull`.
    #[must_use]
    pub fn with_pull(pull: u32) -> Gpio {
        let regs = Arc::new(Registers {
            pull,
            state: Mutex::with_rank(LockRank::DEVICE, State::reset(pull)),
            irq: Mutex::with_rank(LockRank::LEAF, None),
            outs: Mutex::with_rank(LockRank::LEAF, [const { None }; 32]),
            driven_out: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            "rcar.gpio",
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Gpio {
            regs,
            region,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Drive input pin `line` from outside, as a test or a host would.
    pub fn set_input(&self, line: u32, high: bool) {
        self.regs.state.lock().drive(line & 31, high);
        self.regs.refresh(false);
    }

    /// Whether the interrupt output is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.regs.state.lock().irq()
    }
}

fn pin_number(port: &str, prefix: &str) -> Option<u32> {
    let n: u32 = port.strip_prefix(prefix)?.parse().ok()?;
    (n < 32).then_some(n)
}

/// The `rcar.gpio` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "one R-Car GPIO bank: 32 pins, direction, output latch, edge/level interrupts",
    properties: &[PropertySpec {
        name: "inputs",
        kind: ValueKind::Uint,
        required: false,
        summary: "what each undriven input pin reads, one bit per pin: the board's pulls (default 0)",
    }],
    construct: |props| Ok(Box::new(Gpio::new(props)?)),
};

impl Device for Gpio {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut s = self.regs.state.lock();
            // The pins keep what the machine drives on them; the controller
            // forgets everything it was told.
            let (pins, driven) = (s.pins, s.driven);
            *s = State::reset(self.regs.pull);
            s.pins = (pins & driven) | (self.regs.pull & !driven);
            s.driven = driven;
        }
        self.regs.refresh(true);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        let line = pin_number(port, "in")?;
        {
            // A wired pin reads its net, and a fresh net idles low: the pull
            // only answers for pins nothing is connected to. The realize
            // sweep then delivers whatever the driver announces.
            let mut s = self.regs.state.lock();
            s.driven |= 1 << line;
            s.pins &= !(1 << line);
        }
        let pin = Arc::new(InputPin {
            regs: Arc::clone(&self.regs),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port == IRQ_PIN {
            *self.regs.irq.lock() = Some(source);
            return Ok(());
        }
        let Some(n) = pin_number(port, "out") else {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("a GPIO bank drives `irq` and `out0`..`out31`"),
            });
        };
        self.regs.outs.lock()[n as usize] = Some(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.regs.refresh(true);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = *self.regs.state.lock();
        for v in [
            s.iointsel, s.inoutsel, s.outdt, s.edges, s.intmsk, s.posneg, s.edglevel, s.filonoff,
            s.pins, s.driven,
        ] {
            w.write_u32(v)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let s = State {
            iointsel: r.read_u32()?,
            inoutsel: r.read_u32()?,
            outdt: r.read_u32()?,
            edges: r.read_u32()?,
            intmsk: r.read_u32()?,
            posneg: r.read_u32()?,
            edglevel: r.read_u32()?,
            filonoff: r.read_u32()?,
            pins: r.read_u32()?,
            driven: r.read_u32()?,
        };
        *self.regs.state.lock() = s;
        self.regs.refresh(true);
        Ok(())
    }
}

impl Instance for Gpio {}

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind this class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Gpio::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    let mut s = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("inputs", ValueKind::Uint).range(0, u64::from(u32::MAX)))
        .region("")
        .region("regs")
        .port(IRQ_PIN, PortDir::Out);
    for n in 0..32 {
        s = s
            .port(alloc::format!("in{n}"), PortDir::In)
            .port(alloc::format!("out{n}"), PortDir::Out);
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn w(g: &Gpio, off: u64, v: u32) {
        g.regs.write(off, &v.to_le_bytes(), MemAttrs::DEFAULT).unwrap();
    }

    fn r(g: &Gpio, off: u64) -> u32 {
        let mut b = [0u8; 4];
        g.regs.read(off, &mut b, MemAttrs::DEFAULT).unwrap();
        u32::from_le_bytes(b)
    }

    #[test]
    fn undriven_inputs_read_the_pulls_and_outputs_read_their_latch() {
        let g = Gpio::with_pull(0x0000_00f0);
        assert_eq!(r(&g, INDT), 0xf0);
        w(&g, INOUTSEL, 0x1);
        w(&g, OUTDT, 0x1);
        assert_eq!(r(&g, INDT), 0xf1);
    }

    #[test]
    fn a_falling_edge_latches_until_cleared_and_masking_gates_the_irq() {
        let g = Gpio::with_pull(1 << 28);
        w(&g, IOINTSEL, 1 << 28);
        w(&g, EDGLEVEL, 1 << 28);
        w(&g, POSNEG, 1 << 28); // falling
        w(&g, MSKCLR, 1 << 28);
        assert!(!g.irq_asserted());
        g.set_input(28, false);
        assert_eq!(r(&g, INTDT), 1 << 28);
        assert!(g.irq_asserted());
        g.set_input(28, true);
        assert!(g.irq_asserted(), "an edge stays latched");
        w(&g, INTCLR, 1 << 28);
        assert!(!g.irq_asserted());
        g.set_input(28, false);
        w(&g, INTMSK, 1 << 28);
        assert!(!g.irq_asserted(), "masked");
        assert_eq!(r(&g, INTDT), 1 << 28, "but still detected");
    }

    #[test]
    fn a_level_interrupt_follows_the_pin() {
        let g = Gpio::with_pull(0);
        w(&g, IOINTSEL, 1);
        w(&g, MSKCLR, 1);
        assert!(!g.irq_asserted());
        g.set_input(0, true);
        assert!(g.irq_asserted());
        g.set_input(0, false);
        assert!(!g.irq_asserted());
    }

    #[test]
    fn state_round_trips() {
        use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
        let g = Gpio::with_pull(0x5);
        w(&g, INOUTSEL, 0xf0);
        w(&g, OUTDT, 0x30);
        g.set_input(1, true);
        let save = |d: &Gpio| {
            let mut shape = MachineShape::new();
            shape.add_device("g", CLASS_NAME).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut c = wr.chunk("g", CLASS_NAME, STATE_VERSION).unwrap();
                d.save(&mut c).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&g);
        let other = Gpio::with_pull(0x5);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("g", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        other.load(&mut chunk.reader()).unwrap();
        assert_eq!(save(&other), bytes);
        assert_eq!(r(&other, INDT), r(&g, INDT));
    }
}
