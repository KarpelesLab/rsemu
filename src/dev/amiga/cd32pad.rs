//! `amiga.cd32-pad`: the CD32's eleven-button controller on a nine-pin game
//! port, as four switches to ground and a shift register.
//!
//! ```text
//!   object pad "amiga.cd32-pad" { }
//!
//!   wire pad.up    -> denise.m1v      { pull = "up" }   # pin 1, FORW*
//!   wire pad.left  -> denise.m1vq     { pull = "up" }   # pin 3, LEFT*
//!   wire pad.down  -> denise.m1h      { pull = "up" }   # pin 2, BACK*
//!   wire pad.right -> denise.m1hq     { pull = "up" }   # pin 4, RIGH*
//!   wire pad.fire  -> cia_a.pa7       { pull = "up" }   # pin 6, out: red
//!   wire cia_a.pa7 -> pad.fire        { pull = "up" }   # pin 6, in: the clock
//!   wire pad.data  -> paula.potry     { pull = "up" }   # pin 9, out: blue, then the bits
//!   wire paula.potrx-out -> pad.load  { pull = "up" }   # pin 5, in: the latch
//! ```
//!
//! # What it is
//!
//! The *Amiga CD32 Developer Notes* (Revision 3, Commodore-Amiga Inc.)
//! describe the controller and how to read it, and nothing in between: "It has
//! six action buttons, one start button and four directional arrows", "It
//! plugs into the standard, 9-pin D connectors on the side of the system", and
//! `ReadJoyPort()` in `lowlevel.library` "is the only way to get Amiga CD 32
//! game controller information". There is no electrical description in any
//! Commodore document.
//!
//! What the pad *is* is not in doubt, because it is a nine-pin Amiga game port
//! and the manual describes that completely. Four direction switches to ground
//! on pins 1–4, read through Denise's counters (*Amiga Hardware Reference
//! Manual*, 3rd ed., Appendix A's `JOY0DAT` entry: pin 1 `FORW*`, pin 2
//! `BACK*`, pin 3 `LEFT*`, pin 4 `RIGH*`, "the joystick functions are all
//! active low at the connector pins"); a fire button on pin 6, which CIA-A's
//! port A reads (Appendix E: `PA7` is game port 1's); and pins 5 and 9, which
//! are `POTGO`'s and can be **driven as well as read** (Table 8-4, and chapter
//! 8's account of `OUT…` and `DAT…`).
//!
//! # The extra eight buttons
//!
//! A port with two button lines cannot carry eleven buttons, so the pad has a
//! **parallel-in, serial-out shift register** and the Amiga clocks it. Held
//! still, it is an ordinary two-button joystick: pin 6 is the red button and
//! pin 9 is the blue one. When the machine drives **pin 5 low** the register
//! latches, stops driving pin 6, and presents its bits on pin 9 one per rising
//! edge of pin 6 — which the machine now drives as a clock.
//!
//! ```text
//!   pin 5  JOYMODE   an output while the pad is read: low latches
//!   pin 6  CLOCK     the pad's red button, or the machine's clock
//!   pin 9  DATA      the pad's blue button, or the register's output
//! ```
//!
//! The order the bits come out in, with a pressed button reading **low**:
//!
//! ```text
//!   1  blue      2  red      3  yellow    4  green
//!   5  forward (right shoulder)           6  reverse (left shoulder)
//!   7  play/pause
//!   8  a one, and every clock after it a zero
//! ```
//!
//! That last pair is the point of the whole arrangement: a normal joystick
//! holds pin 9 at whatever its second button is doing however long you clock
//! it, and a pad drops the line and keeps it there, so `ReadJoyPort()` can
//! tell them apart without asking. The Developer Notes' `JP_TYPE_GAMECTLR` is
//! that distinction.
//!
//! **Provenance.** Every pin above is the *Amiga Hardware Reference Manual*'s.
//! The shift register, its bit order and the terminating one-then-zeros are
//! **not in any Commodore document**; they are the behaviour the pad has, as
//! recorded in the hardware descriptions and replacement-pad schematics that
//! circulate for it, and they are marked here as what they are. Nothing in
//! this file came from an emulator, an FPGA core or a ROM disassembly
//! (`ROADMAP.md` §1). The CD32's own ROM was not used to settle it either:
//! the boot screen never reads a pad, so there was nothing to watch.
//!
//! # Determinism
//!
//! Through the record/replay seam, exactly as [`super::mouse`] is: the pad is
//! a named host object, [`Pad`], opened by the device in `new(props)`;
//! [`pads::channel`] and [`pads::sink`] are its channel and sink, and a
//! payload is [`pads::RECORD_BYTES`] bytes — the [`buttons`] mask, little
//! endian.
//!
//! # Time
//!
//! The pad has none of its own. Every edge it responds to is one the machine
//! put on a wire, and its output changes in the same delivery, so it is not a
//! lazily-advanced device and registers no event. A real 74LS165 is the same:
//! combinational except for the clock somebody else drives.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{Drive, FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.cd32-pad";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

/// The host object a board gets when it does not name one.
pub const DEFAULT_PAD: &str = "pad0";

/// Which button is which bit of a report and of [`Pad::held`].
pub mod buttons {
    /// Pin 1, `FORW*`.
    pub const UP: u16 = 1 << 0;
    /// Pin 2, `BACK*`.
    pub const DOWN: u16 = 1 << 1;
    /// Pin 3, `LEFT*`.
    pub const LEFT: u16 = 1 << 2;
    /// Pin 4, `RIGH*`.
    pub const RIGHT: u16 = 1 << 3;
    /// The red button: pin 6 while the pad is not being clocked.
    pub const RED: u16 = 1 << 4;
    /// The blue button: pin 9 while the pad is not being clocked.
    pub const BLUE: u16 = 1 << 5;
    /// The green button.
    pub const GREEN: u16 = 1 << 6;
    /// The yellow button.
    pub const YELLOW: u16 = 1 << 7;
    /// The right shoulder.
    pub const FORWARD: u16 = 1 << 8;
    /// The left shoulder.
    pub const REVERSE: u16 = 1 << 9;
    /// The start button, marked play/pause.
    pub const PLAY: u16 = 1 << 10;
    /// Every button this pad has.
    pub const ALL: u16 =
        UP | DOWN | LEFT | RIGHT | RED | BLUE | GREEN | YELLOW | FORWARD | REVERSE | PLAY;
}

/// The pins this device drives, in order: the four directions, then pin 6 and
/// pin 9.
pub const OUTPUT_PINS: [&str; 6] = ["up", "down", "left", "right", "fire", "data"];

/// The pins it reads: pin 5, the latch, and pin 6, the clock — which is the
/// same connector pin `fire` drives, because it is one wire.
pub const INPUT_PINS: [&str; 2] = ["load", "fire"];

/// The shift register's order out, most significant first: the seven buttons
/// that are not the directions, then the one that marks the end of them.
const SHIFT_ORDER: [u16; 7] = [
    buttons::BLUE,
    buttons::RED,
    buttons::YELLOW,
    buttons::GREEN,
    buttons::FORWARD,
    buttons::REVERSE,
    buttons::PLAY,
];

// ---------------------------------------------------------------------------
// the pad
// ---------------------------------------------------------------------------

/// Everything the pad holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// Which buttons the host is holding down.
    held: u16,
    /// The level on pin 5. High is a joystick; low is a latched pad.
    load: bool,
    /// The level on pin 6, for finding its rising edges.
    clock: bool,
    /// The register's contents, shifted out of the top. Zeros arrive from
    /// below, which is the permanent low that says "this is a pad".
    shift: u16,
}

impl Default for State {
    fn default() -> State {
        State {
            held: 0,
            load: true,
            clock: true,
            shift: 0,
        }
    }
}

impl State {
    /// What the register holds the instant pin 5 goes low: the seven buttons
    /// active low, then a one.
    fn latch(&mut self) {
        let mut bits = 0u16;
        for (i, button) in SHIFT_ORDER.iter().enumerate() {
            if self.held & button == 0 {
                bits |= 1 << (15 - i);
            }
        }
        // The eighth bit is a one however the buttons stand; every bit after
        // it is a zero, which is what a shift with no input gives.
        bits |= 1 << (15 - SHIFT_ORDER.len());
        self.shift = bits;
    }

    /// Which of [`OUTPUT_PINS`] this pad is pulling low.
    fn pins(&self) -> [bool; 6] {
        let held = |b: u16| self.held & b != 0;
        let (fire, data) = if self.load {
            // A joystick: the two buttons, and nothing clever.
            (held(buttons::RED), held(buttons::BLUE))
        } else {
            // Latched: pin 6 is the machine's to drive, and pin 9 carries the
            // register's top bit, active low.
            (false, self.shift & 0x8000 == 0)
        };
        [
            held(buttons::UP),
            held(buttons::DOWN),
            held(buttons::LEFT),
            held(buttons::RIGHT),
            fire,
            data,
        ]
    }
}

#[derive(Debug, Default, Clone)]
struct Outputs {
    pins: [Option<WireSource>; 6],
}

/// A CD32 controller: its switches, its shift register, and the door button
/// reports come in through.
pub struct Pad {
    state: Mutex<State>,
    out: Mutex<Outputs>,
}

impl fmt::Debug for Pad {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Pad");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Default for Pad {
    fn default() -> Pad {
        Pad::new()
    }
}

impl Pad {
    /// A pad with nothing pressed.
    #[must_use]
    pub fn new() -> Pad {
        Pad {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            out: Mutex::with_rank(LockRank::WIRE, Outputs::default()),
        }
    }

    /// Hold exactly the buttons in `held` ([`buttons`] bits).
    ///
    /// The device end of the record/replay channel. A pad has no state of its
    /// own to lose, so a report is applied where it arrives.
    pub fn report(&self, held: u16) {
        self.update(|st| st.held = held & buttons::ALL);
    }

    /// The buttons held now.
    #[must_use]
    pub fn held(&self) -> u16 {
        self.state.lock().held
    }

    /// Whether pin 5 is holding the register latched.
    #[must_use]
    pub fn latched(&self) -> bool {
        !self.state.lock().load
    }

    /// The register's contents, most significant bit next out.
    #[must_use]
    pub fn shift(&self) -> u16 {
        self.state.lock().shift
    }

    /// Pin 5 moved.
    fn set_load(&self, high: bool) {
        self.update(|st| {
            if st.load && !high {
                // The falling edge is the latch.
                st.latch();
            }
            st.load = high;
        });
    }

    /// Pin 6 moved. A rising edge while the register is latched shifts it.
    fn set_clock(&self, high: bool) {
        self.update(|st| {
            if high && !st.clock && !st.load {
                st.shift <<= 1;
            }
            st.clock = high;
        });
    }

    fn update(&self, f: impl FnOnce(&mut State)) {
        let moved = {
            let mut st = self.state.lock();
            let before = st.pins();
            f(&mut st);
            before != st.pins()
        };
        if moved {
            self.refresh();
        }
    }

    /// Drive every output stage, holding no lock. Open collector: the pad
    /// pulls a pin low or lets the port's pull-up have it.
    fn refresh(&self) {
        let pins = self.state.lock().pins();
        let out = self.out.lock().clone();
        for (src, low) in out.pins.iter().zip(pins) {
            if let Some(src) = src {
                src.drive(if low { Drive::Low } else { Drive::HiZ });
            }
        }
    }
}

/// The build's named CD32 controllers.
pub mod pads {
    use super::Pad;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::record::{Channel, FnSink, InputSink};

    /// The kind a pad is filed under in a build's host objects.
    pub const KIND: HostKind = HostKind::door("amiga-cd32-pad", make_sink);

    /// The pad `name` refers to in `hosts`, creating it on first mention.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of object holds that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<Pad>> {
        hosts.open(KIND, name, Pad::new)
    }

    /// The device's side of [`open`].
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<Pad>> {
        props.host(KIND, name, Pad::new)
    }

    /// The pad called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<Pad>>> {
        hosts.get(KIND, name)
    }

    /// Every open name, in order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }

    /// Bytes per recorded report: the button mask as a little-endian `u16`.
    pub const RECORD_BYTES: usize = 2;

    /// One report as a payload.
    #[must_use]
    pub const fn encode(held: u16) -> [u8; RECORD_BYTES] {
        held.to_le_bytes()
    }

    /// The channel the pad called `name` reports on: `amiga-cd32-pad:pad0`.
    #[must_use]
    pub fn channel(name: &str) -> Channel {
        Channel::new(KIND, name)
    }

    /// The sink that applies recorded reports to `pad`, in order. A trailing
    /// partial report is discarded.
    #[must_use]
    pub fn sink(pad: &Arc<Pad>) -> Arc<dyn InputSink> {
        let pad = Arc::clone(pad);
        Arc::new(FnSink::new("amiga-cd32-pad", move |payload: &[u8]| {
            let (reports, _partial) = payload.as_chunks::<RECORD_BYTES>();
            for [lo, hi] in reports {
                pad.report(u16::from_le_bytes([*lo, *hi]));
            }
        }))
    }

    fn make_sink(object: &Arc<dyn core::any::Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
        Some(sink(&Arc::clone(object).downcast::<Pad>().ok()?))
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// One of the two pins the pad listens on.
#[derive(Debug)]
struct PadPin {
    pad: Arc<Pad>,
    /// True for pin 6, the clock; false for pin 5, the latch.
    clock: bool,
    inputs: FanIn,
}

impl WireSink for PadPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::And).is_high();
        if self.clock {
            self.pad.set_clock(high);
        } else {
            self.pad.set_load(high);
        }
    }
}

/// The `amiga.cd32-pad` device.
#[derive(Debug)]
pub struct Cd32Pad {
    pad: Arc<Pad>,
    pins: Mutex<Vec<Arc<PadPin>>>,
}

impl Cd32Pad {
    /// Validate `props` and open the pad they name.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a bad or unknown property, [`Error::Config`] if
    /// the name is held by something that is not a pad.
    pub fn new(props: &Props) -> Result<Cd32Pad> {
        let mut r = props.reader();
        let port = r.or_str("pad", DEFAULT_PAD)?.to_string();
        r.finish()?;
        Ok(Cd32Pad::with(pads::attach(props, &port)?))
    }

    /// A device around a pad the caller already holds.
    #[must_use]
    pub fn with(pad: Arc<Pad>) -> Cd32Pad {
        Cd32Pad {
            pad,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// The pad.
    #[must_use]
    pub fn pad(&self) -> &Arc<Pad> {
        &self.pad
    }
}

impl Device for Cd32Pad {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the wire graph brings the pins, and `connect`
        // drives each as it arrives.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // The machine's reset line does not reach a controller on a cable. The
        // buttons are where the player's thumbs are.
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let Some(pin) = OUTPUT_PINS.iter().position(|p| *p == port) else {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from(
                    "a CD32 pad drives `up`, `down`, `left`, `right` (connector pins 1-4), \
                     `fire` (pin 6) and `data` (pin 9)",
                ),
            });
        };
        self.pad.out.lock().pins[pin] = Some(source);
        self.pad.refresh();
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.pad.refresh();
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let clock = match port {
            "load" => false,
            "fire" => true,
            _ => return None,
        };
        let pin = Arc::new(PadPin {
            pad: Arc::clone(&self.pad),
            clock,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: u32::from(clock),
        })
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let st = *self.pad.state.lock();
        w.write_u16(st.held)?;
        w.write_bool(st.load)?;
        w.write_bool(st.clock)?;
        w.write_u16(st.shift)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let held = r.read_u16()?;
        if held & !buttons::ALL != 0 {
            return Err(Error::State(String::from(
                "amiga.cd32-pad: a button this pad does not have",
            )));
        }
        let load = r.read_bool()?;
        let clock = r.read_bool()?;
        let shift = r.read_u16()?;
        *self.pad.state.lock() = State {
            held,
            load,
            clock,
            shift,
        };
        self.pad.refresh();
        Ok(())
    }
}

impl Instance for Cd32Pad {}

/// The `amiga.cd32-pad` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a CD32 joypad on a nine-pin game port: four switches to ground, two buttons, and \
              a shift register the machine clocks for the other seven",
    properties: &[PropertySpec {
        name: "pad",
        kind: ValueKind::Str,
        required: false,
        summary: "the host pad this port is joined to (default `pad0`)",
    }],
    construct: |props| Ok(Box::new(Cd32Pad::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Cd32Pad::new(props)?)))
}

/// What the validator should know about `amiga.cd32-pad`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME).prop(PropSchema::new("pad", ValueKind::Str));
    for pin in OUTPUT_PINS {
        // Pin 6 is one wire with two jobs: the pad's red button on the way
        // out, the machine's clock on the way in.
        let dir = if pin == "fire" {
            PortDir::InOut
        } else {
            PortDir::Out
        };
        schema = schema.port(pin, dir);
    }
    schema.port("load", PortDir::In)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    /// A pad with its two input pins driven directly, which is what the wire
    /// graph does on a board.
    struct Port {
        pad: Arc<Pad>,
    }

    impl Port {
        fn new() -> Port {
            Port {
                pad: Arc::new(Pad::new()),
            }
        }

        fn load(&self, high: bool) {
            self.pad.set_load(high);
        }

        fn clock(&self, high: bool) {
            self.pad.set_clock(high);
        }

        /// Pin 6 as the machine sees it: low when the pad pulls it.
        fn fire_low(&self) -> bool {
            self.pad.state.lock().pins()[4]
        }

        /// Pin 9 as the machine sees it.
        fn data_low(&self) -> bool {
            self.pad.state.lock().pins()[5]
        }

        /// Take the bit pin 9 is presenting, then clock the register on.
        ///
        /// The line rests high — the port's pull-up has it — so a reader
        /// pulls it low and lets it rise, and the rise is the shift.
        fn shift_in(&self) -> bool {
            let bit = !self.data_low();
            self.clock(false);
            self.clock(true);
            bit
        }
    }

    /// Held still, the pad is a two-button joystick and nothing else.
    #[test]
    fn with_pin_five_high_it_is_an_ordinary_joystick() {
        let p = Port::new();
        assert!(!p.fire_low());
        assert!(!p.data_low());
        p.pad.report(buttons::RED);
        assert!(p.fire_low(), "pin 6 is the red button");
        assert!(!p.data_low());
        p.pad.report(buttons::BLUE);
        assert!(!p.fire_low());
        assert!(p.data_low(), "pin 9 is the blue button");
    }

    /// The four directions are switches to ground on pins 1 to 4, and they do
    /// not care about pin 5 at all.
    #[test]
    fn the_directions_are_four_switches() {
        let p = Port::new();
        for (bit, pin) in [
            (buttons::UP, 0),
            (buttons::DOWN, 1),
            (buttons::LEFT, 2),
            (buttons::RIGHT, 3),
        ] {
            p.pad.report(bit);
            let pins = p.pad.state.lock().pins();
            for (i, low) in pins.iter().enumerate().take(4) {
                assert_eq!(*low, i == pin, "button {bit:#x}, pin {i}");
            }
        }
        p.pad.report(buttons::UP);
        p.load(false);
        assert!(p.pad.state.lock().pins()[0], "still a switch when latched");
    }

    /// Pin 5 low latches, the pad lets go of pin 6, and the seven buttons
    /// come out of pin 9 in order, active low, followed by a one and then
    /// zeros for ever.
    #[test]
    fn latching_clocks_the_other_seven_buttons_out_of_pin_nine() {
        let p = Port::new();
        p.pad.report(buttons::YELLOW | buttons::PLAY | buttons::RED);
        p.load(false);
        assert!(!p.fire_low(), "the pad lets the machine have the clock");

        // Blue, red, yellow, green, forward, reverse, play — a pressed button
        // reads low, which is a `false` here.
        let want = [true, false, false, true, true, true, false];
        for (i, expect) in want.iter().enumerate() {
            assert_eq!(p.shift_in(), *expect, "bit {i}");
        }
        // The marker, then the permanent low that says this is a pad.
        assert!(p.shift_in(), "the eighth bit is a one");
        for i in 0..24 {
            assert!(!p.shift_in(), "clock {} past the marker", i + 9);
        }
    }

    /// A button pressed after the latch does not move the bits already in the
    /// register, which is what a parallel load means.
    #[test]
    fn the_register_holds_what_it_latched() {
        let p = Port::new();
        p.pad.report(0);
        p.load(false);
        p.pad.report(buttons::ALL);
        for i in 0..7 {
            assert!(
                p.shift_in(),
                "bit {i} is what was latched, not what is held"
            );
        }
    }

    /// Letting pin 5 go puts the two buttons back on the two pins.
    #[test]
    fn releasing_pin_five_makes_it_a_joystick_again() {
        let p = Port::new();
        p.pad.report(buttons::RED | buttons::BLUE);
        p.load(false);
        let _ = p.shift_in();
        p.load(true);
        assert!(p.fire_low());
        assert!(p.data_low());
    }

    /// Only a *rising* edge shifts, and only while pin 5 is low.
    #[test]
    fn a_clock_with_pin_five_high_shifts_nothing() {
        let p = Port::new();
        p.pad.report(0);
        p.load(false);
        let before = p.pad.shift();
        p.load(true);
        for _ in 0..4 {
            p.clock(false);
            p.clock(true);
        }
        assert_eq!(p.pad.shift(), before);
    }

    fn snapshot(d: &Cd32Pad) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("pad", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("pad", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(d, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn a_snapshot_round_trips_and_resumes_identically() {
        let saved = Cd32Pad::with(Arc::new(Pad::new()));
        saved.pad.report(buttons::GREEN | buttons::REVERSE);
        saved.pad.set_load(false);
        for _ in 0..2 {
            saved.pad.set_clock(false);
            saved.pad.set_clock(true);
        }

        let bytes = snapshot(&saved);
        let restored = Cd32Pad::with(Arc::new(Pad::new()));
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("pad", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();
        assert_eq!(snapshot(&restored), bytes);

        // And the rest of the register comes out the same on both.
        for _ in 0..6 {
            for pad in [&saved.pad, &restored.pad] {
                pad.set_clock(false);
                pad.set_clock(true);
            }
            assert_eq!(saved.pad.shift(), restored.pad.shift());
        }
        assert_eq!(snapshot(&restored), snapshot(&saved));
    }

    #[test]
    fn a_button_this_pad_does_not_have_is_refused_on_load() {
        let pad = Cd32Pad::with(Arc::new(Pad::new()));
        let mut bytes = snapshot(&pad);
        let at = bytes.len() - 6;
        bytes[at] = 0xFF;
        bytes[at + 1] = 0xFF;
        if let Ok(reader) = StateReader::new(&bytes)
            && let Ok(chunk) = reader.load("pad", CLASS_NAME, STATE_VERSION, &Migrations::new())
        {
            assert!(Device::load(&pad, &mut chunk.reader()).is_err());
        }
    }

    /// A report the host sends is the mask and nothing else.
    #[test]
    fn a_report_holds_exactly_what_it_names() {
        let pad = Pad::new();
        pad.report(0xFFFF);
        assert_eq!(pad.held(), buttons::ALL);
        pad.report(buttons::PLAY);
        assert_eq!(pad.held(), buttons::PLAY);
    }

    /// The recorded payload is the mask, little endian, and the sink puts it
    /// back.
    #[test]
    fn a_recorded_report_replays() {
        let pad = Arc::new(Pad::new());
        let sink = pads::sink(&pad);
        sink.deliver(&pads::encode(buttons::RED | buttons::PLAY));
        assert_eq!(pad.held(), buttons::RED | buttons::PLAY);
    }
}
