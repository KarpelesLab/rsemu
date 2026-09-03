//! The Game Boy's joypad register — `$FF00`.
//!
//! One register, and it is a matrix rather than a set of eight bits:
//!
//! ```text
//!   bit 7,6   not implemented, read as ones
//!   bit 5     0 selects the action buttons  (Start, Select, B, A)
//!   bit 4     0 selects the direction pad   (Down, Up, Left, Right)
//!   bit 3-0   the selected row, **0 meaning pressed**
//! ```
//!
//! Both select lines are outputs the program drives and both may be low at once,
//! in which case the two rows are wired together and the reads are ORed — in the
//! active-low sense, so a button held in *either* row pulls its column low. With
//! neither selected the low nibble reads as `$F`.
//!
//! The joypad interrupt fires when any selected line goes **low** — that is, on
//! a press, not a release, and only for a row the program has selected. This
//! device drives that as a *level* on [`IRQ_PIN`] and lets the CPU's edge
//! detector turn it into an `IF` bit, for the same reason the LCD's STAT line
//! does (`cpu::sm83`): a second button pressed while the first is still held
//! raises no second interrupt, and that falls out rather than being coded.
//!
//! The same line is what ends `STOP`.
//!
//! # Determinism
//!
//! Button state is a non-deterministic input crossing into the machine, and
//! `ROADMAP.md` §0 says every one of those goes through the record/replay seam
//! or it is a determinism bug. It does: the buttons live in a **named host
//! object**, [`GbPad`], which this device opens by name from `new(props)` the
//! way a UART opens a character port — so [`pads::channel`] is the channel a
//! recorder registers, [`pads::sink`] is what a recorded payload does, and a
//! board whose joypad the recorder does not know about is refused by
//! [`HostObjects::seal`](crate::core::hosts::HostObjects::seal) at build time.
//!
//! There is no second door. [`GbPad::set_pressed`] is the *device* side of the
//! seam — the thing the channel's sink calls, exactly as `CharPort::feed` is
//! for a keystroke — and a host reaches it by opening the pad by name, which is
//! the act the seal checks. A payload is one byte: the held mask, level rather
//! than edge, because that is what the matrix reads.
//!
//! # Sources
//!
//! [Pan Docs](https://gbdev.io/pandocs/) (CC0), *Joypad Input*. No emulator
//! source was consulted.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};

/// Where the register sits in the CPU's address space.
pub const REGISTER_BASE: u64 = 0xff00;

/// The name a `map` statement reaches it by.
pub const REGISTER_REGION: &str = "regs";

/// The joypad-interrupt output pin.
pub const IRQ_PIN: &str = "irq";

/// The host pad port a machine gets when its description names none.
pub const DEFAULT_PAD_PORT: &str = "gb-joypad";

/// One of the eight buttons.
///
/// The discriminants are the bit positions in [`GbJoypad::buttons`]: the
/// direction pad in the low nibble and the action buttons in the high one, each
/// in the order the hardware's four columns read out.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Button {
    /// The d-pad's right.
    Right,
    /// The d-pad's left.
    Left,
    /// The d-pad's up.
    Up,
    /// The d-pad's down.
    Down,
    /// The A button.
    A,
    /// The B button.
    B,
    /// Select.
    Select,
    /// Start.
    Start,
}

impl Button {
    /// Every button, in bit order.
    pub const ALL: [Button; 8] = [
        Button::Right,
        Button::Left,
        Button::Up,
        Button::Down,
        Button::A,
        Button::B,
        Button::Select,
        Button::Start,
    ];

    /// The bit this button occupies in the packed state.
    #[must_use]
    pub const fn bit(self) -> u8 {
        match self {
            Button::Right => 0,
            Button::Left => 1,
            Button::Up => 2,
            Button::Down => 3,
            Button::A => 4,
            Button::B => 5,
            Button::Select => 6,
            Button::Start => 7,
        }
    }

    /// The button's name, lowercase, as a front end would spell it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Button::Right => "right",
            Button::Left => "left",
            Button::Up => "up",
            Button::Down => "down",
            Button::A => "a",
            Button::B => "b",
            Button::Select => "select",
            Button::Start => "start",
        }
    }

    /// Look a button up by name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Button> {
        Button::ALL.into_iter().find(|b| b.name() == name)
    }
}

impl fmt::Display for Button {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

#[derive(Debug, Clone, Copy, Default)]
struct State {
    /// Which buttons are held, one bit each. **1 means pressed**, the opposite
    /// of what the register reads, because a bitmask nobody has to invert is
    /// harder to get wrong.
    pressed: u8,
    /// The two select lines, as bits 5 and 4 of the last write. Zero selects.
    select: u8,
}

impl State {
    /// The low nibble the program reads: zero for a pressed button in a
    /// selected row.
    fn nibble(&self) -> u8 {
        let mut low = 0x0f;
        // Bit 4 low selects the directions.
        if self.select & 0x10 == 0 {
            low &= !(self.pressed & 0x0f);
        }
        // Bit 5 low selects the action buttons.
        if self.select & 0x20 == 0 {
            low &= !((self.pressed >> 4) & 0x0f);
        }
        low
    }

    /// Whether any selected line is low — the interrupt condition, and what
    /// ends `STOP`.
    fn asserted(&self) -> bool {
        self.nibble() != 0x0f
    }

    /// `$FF00` as the guest reads it. Bits 7 and 6 are not implemented.
    fn read(&self) -> u8 {
        0xc0 | (self.select & 0x30) | self.nibble()
    }
}

/// The joypad as a device.
pub struct GbJoypad {
    pad: Arc<GbPad>,
    regs_region: RegionRef,
}

impl fmt::Debug for GbJoypad {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GbJoypad")
            .field("pad", &self.pad)
            .finish_non_exhaustive()
    }
}

impl Default for GbJoypad {
    fn default() -> Self {
        GbJoypad::new()
    }
}

/// The matrix itself: what is held, what is selected, and the request line.
///
/// This is the **host object** a build files under [`pads::KIND`] and the name
/// the machine description gave — the console's controller port rather than a
/// copy of it. The device holds it, the `$FF00` register block holds it, and a
/// host that presses a button opens it by name. One object rather than a device
/// handle plus a mirror of its state, because two would have to be kept in step
/// and nothing would check that they were.
///
/// The interrupt line lives here too, and has to: a press is what drives it,
/// and a press comes from out here.
pub struct GbPad {
    state: Mutex<State>,
    /// [`LockRank::WIRE`]: taken after the state lock is released, never with
    /// it held.
    irq: Mutex<Option<WireSource>>,
}

impl fmt::Debug for GbPad {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GbPad")
            .field("state", &self.state)
            .finish_non_exhaustive()
    }
}

impl Default for GbPad {
    fn default() -> GbPad {
        GbPad::new()
    }
}

impl GbPad {
    /// A pad with nothing held and neither row selected.
    #[must_use]
    pub fn new() -> GbPad {
        GbPad {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            irq: Mutex::with_rank(LockRank::WIRE, None),
        }
    }

    /// Which buttons are held, one bit each — see [`Button::bit`].
    #[must_use]
    pub fn buttons(&self) -> u8 {
        self.state.lock().pressed
    }

    /// `$FF00` as the guest reads it.
    #[must_use]
    pub fn read(&self) -> u8 {
        self.state.lock().read()
    }

    /// Press or release one button.
    ///
    /// The device end of the record/replay channel — see the module docs. The
    /// state lock is released before the request line is driven, because
    /// driving it is an outward call (`CLAUDE.md`, re-entrancy).
    pub fn set_pressed(&self, button: Button, pressed: bool) {
        let asserted = {
            let mut state = self.state.lock();
            if pressed {
                state.pressed |= 1 << button.bit();
            } else {
                state.pressed &= !(1 << button.bit());
            }
            state.asserted()
        };
        self.drive(asserted);
    }

    /// Set every button at once, as a packed mask. **1 is pressed.**
    ///
    /// What one recorded payload is: the whole held mask, level rather than
    /// edge, because a matrix has no notion of a transition.
    pub fn set_buttons(&self, pressed: u8) {
        let asserted = {
            let mut state = self.state.lock();
            state.pressed = pressed;
            state.asserted()
        };
        self.drive(asserted);
    }

    /// Connect the joypad-interrupt request line.
    pub fn attach_irq(&self, source: WireSource) {
        *self.irq.lock() = Some(source);
        let asserted = self.state.lock().asserted();
        self.drive(asserted);
    }

    /// Drive the request line, outside the state lock.
    fn drive(&self, asserted: bool) {
        let source = self.irq.lock().clone();
        if let Some(source) = source {
            source.set(Level::from_bool(asserted));
        }
    }
}

impl GbJoypad {
    /// A joypad with a private pad port: nothing held, neither row selected.
    #[must_use]
    pub fn new() -> GbJoypad {
        GbJoypad::with_pad(Arc::new(GbPad::new()))
    }

    /// A joypad reading `pad`, which the host may already hold.
    #[must_use]
    pub fn with_pad(pad: Arc<GbPad>) -> GbJoypad {
        // One state behind one lock, shared with the register block: the port
        // needs only the pad, and giving it a whole device handle would be an
        // ownership cycle through the region it is inside.
        let regs_region = Arc::new(MmioRegion::io(
            "gb.joypad.regs",
            1,
            Arc::new(JoypadPort {
                pad: Arc::clone(&pad),
            }) as Arc<dyn MemOps>,
        ));
        GbJoypad { pad, regs_region }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If an unknown property was given, or if another kind of host object
    /// already holds the pad port's name.
    pub fn from_props(props: &Props) -> Result<GbJoypad> {
        let mut r = props.reader();
        let name = r.or_str("pad", DEFAULT_PAD_PORT)?.to_string();
        r.finish()?;
        Ok(GbJoypad::with_pad(pads::attach(props, &name)?))
    }

    /// The pad port this joypad reads: the host end of the seam.
    #[must_use]
    pub fn pad(&self) -> &Arc<GbPad> {
        &self.pad
    }

    /// Which buttons are held, one bit each — see [`Button::bit`].
    #[must_use]
    pub fn buttons(&self) -> u8 {
        self.pad.buttons()
    }

    /// `$FF00` as the guest reads it.
    #[must_use]
    pub fn read(&self) -> u8 {
        self.pad.read()
    }
}

/// The build's named Game Boy pad ports.
///
/// The same shape as [`chardev::ports`](crate::host::chardev::ports) and the
/// NES's `nes.ports`: a *name* is the only thing that can travel from a machine
/// description into a device constructor, and both ends resolve it against the
/// build's own [`HostObjects`](crate::core::hosts::HostObjects).
///
/// ```text
/// machine file:  object pad "gb.joypad" { pad = "gb-joypad" }
/// device:        pads::attach(props, "gb-joypad")  ──┐
/// host:          pads::open(&hosts, "gb-joypad")   ──┴─► the same Arc<GbPad>
/// ```
pub mod pads {
    use super::GbPad;
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::record::{Channel, FnSink, InputSink};

    /// The kind a pad port is filed under in a build's [`HostObjects`].
    pub const KIND: HostKind = HostKind::door("pad", make_sink);

    /// The pad port `name` refers to in `hosts`, creating it on first mention.
    ///
    /// The **host** side of the rendezvous: called before anybody presses
    /// anything, or after the build to pick up what the device opened.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object already holds
    /// that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<GbPad>> {
        hosts.open(KIND, name, GbPad::new)
    }

    /// The pad port `name` refers to in the build these properties belong to.
    ///
    /// The **device** side, called from `new(props)`: acquiring a host object is
    /// allocation, not an outward action ([`core::hosts`](crate::core::hosts)
    /// argues the case). A `Props` that belongs to no build gets a private pad,
    /// so a device a unit test built directly still works and simply meets
    /// nobody.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<GbPad>> {
        props.host(KIND, name, GbPad::new)
    }

    /// The pad port called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<GbPad>>> {
        hosts.get(KIND, name)
    }

    /// Forget `name`, reporting whether there was one.
    pub fn close(hosts: &HostObjects, name: &str) -> bool {
        hosts.close(KIND, name)
    }

    /// Every open name, in order.
    #[must_use]
    pub fn names(hosts: &HostObjects) -> Vec<String> {
        hosts.names(KIND)
    }

    /// [`sink`], reached through the erased handle the host-object table holds.
    ///
    /// What [`KIND`] carries so that
    /// [`HostObjects::seal`](crate::core::hosts::HostObjects::seal) can wire
    /// this pad port to a recorder without the caller having to name it. `None`
    /// means something that is not a [`GbPad`] is filed under `pad` — two
    /// modules claiming one kind name, which the seal reports rather than
    /// guesses at.
    fn make_sink(object: &Arc<dyn core::any::Any + Send + Sync>) -> Option<Arc<dyn InputSink>> {
        Some(sink(&Arc::clone(object).downcast::<GbPad>().ok()?))
    }

    /// The record/replay channel the pad called `name` is pressed through.
    ///
    /// `pad:gb-joypad`, which is the same `(kind, name)` pair the host-object
    /// table files the pad under — so a board whose joypad has no channel is
    /// refused by [`HostObjects::seal`](crate::core::hosts::HostObjects::seal)
    /// naming this string.
    #[must_use]
    pub fn channel(name: &str) -> Channel {
        Channel::new(KIND, name)
    }

    /// The [`InputSink`] that applies a recorded payload to `pad`.
    ///
    /// One byte: the held mask, in [`Button::bit`](super::Button::bit) order. A
    /// longer payload is that mask changing more than once, and each byte is
    /// applied in turn — which is what a host batching two changes into one
    /// post means.
    ///
    /// No rewind hook: a pad holds a level rather than a queue, and the level
    /// is part of the machine snapshot a rewind restores.
    #[must_use]
    pub fn sink(pad: &Arc<GbPad>) -> Arc<dyn InputSink> {
        let pad = Arc::clone(pad);
        Arc::new(FnSink::new("pad", move |payload: &[u8]| {
            for byte in payload {
                pad.set_buttons(*byte);
            }
        }))
    }
}

/// The `$FF00` register.
struct JoypadPort {
    pad: Arc<GbPad>,
}

impl fmt::Debug for JoypadPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("JoypadPort").finish_non_exhaustive()
    }
}

impl MemOps for JoypadPort {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // Reading `$FF00` has no side effect at all, so a debug read needs no
        // special case.
        *byte = self.pad.state.lock().read();
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        // Only the two select bits are writable; the rest are inputs.
        self.pad.state.lock().select = *value & 0x30;
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// The properties `gb.joypad` takes.
static JOYPAD_PROPERTIES: &[PropertySpec] = &[PropertySpec {
    name: "pad",
    kind: ValueKind::Str,
    required: false,
    summary: "the host pad port buttons arrive through, by name (default \"gb-joypad\")",
}];

/// The `gb.joypad` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "gb.joypad",
    version: 1,
    summary: "Game Boy joypad matrix ($FF00): two selectable rows of four active-low lines",
    properties: JOYPAD_PROPERTIES,
    construct: |props| Ok(Box::new(GbJoypad::from_props(props)?) as Box<dyn Device>),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for GbJoypad {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The realize sweep: announce what the line idles at, which is low with
        // nothing held (`ROADMAP.md` §4.3).
        let asserted = self.pad.state.lock().asserted();
        self.pad.drive(asserted);
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        (name.is_empty() || name == REGISTER_REGION).then(|| Arc::clone(&self.regs_region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!("the joypad drives only `{IRQ_PIN}`"),
            });
        }
        self.pad.attach_irq(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == IRQ_PIN {
            let asserted = self.pad.state.lock().asserted();
            self.pad.drive(asserted);
        }
    }

    fn reset(&self, _kind: ResetKind) {
        // Which buttons a person is holding is not something a reset changes,
        // so only the select lines go back.
        self.pad.state.lock().select = 0;
        let asserted = self.pad.state.lock().asserted();
        self.pad.drive(asserted);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.pad.state.lock();
        w.write_u8(state.pressed)?;
        w.write_u8(state.select)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let state = State {
            pressed: r.read_u8()?,
            select: r.read_u8()?,
        };
        *self.pad.state.lock() = state;
        self.pad.drive(state.asserted());
        Ok(())
    }
}

impl crate::machine::Instance for GbJoypad {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| {
        Ok(Arc::new(GbJoypad::from_props(props)?))
    })
}

/// What the validator should know about `gb.joypad`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("pad", ValueKind::Str))
        .port(IRQ_PIN, PortDir::Out)
        .region(REGISTER_REGION)
}
