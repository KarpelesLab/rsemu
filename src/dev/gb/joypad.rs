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
//! or it is a determinism bug. That seam does not exist yet, so [`set_pressed`]
//! is the interim door and it is deliberately the *only* one: when the seam
//! lands, this is the single place that has to change.
//!
//! [`set_pressed`]: GbJoypad::set_pressed
//!
//! # Sources
//!
//! [Pan Docs](https://gbdev.io/pandocs/) (CC0), *Joypad Input*. No emulator
//! source was consulted.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
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
    shared: Arc<Shared>,
    irq: Mutex<Option<WireSource>>,
    regs_region: RegionRef,
}

impl fmt::Debug for GbJoypad {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GbJoypad")
            .field("state", &self.shared.state)
            .finish_non_exhaustive()
    }
}

impl Default for GbJoypad {
    fn default() -> Self {
        GbJoypad::new()
    }
}

/// What the register block shares with the device.
struct Shared {
    state: Mutex<State>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("state", &self.state)
            .finish()
    }
}

impl GbJoypad {
    /// A joypad with nothing held and neither row selected.
    #[must_use]
    pub fn new() -> GbJoypad {
        // One state behind one lock, shared with the register block: the port
        // needs only the state, and giving it a whole device handle would be an
        // ownership cycle through the region it is inside.
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
        });
        let regs_region = Arc::new(MmioRegion::io(
            "gb.joypad.regs",
            1,
            Arc::new(JoypadPort {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        GbJoypad {
            shared,
            irq: Mutex::with_rank(LockRank::WIRE, None),
            regs_region,
        }
    }

    /// Build one from machine-description properties. It takes none.
    ///
    /// # Errors
    ///
    /// If any property was given at all.
    pub fn from_props(props: &Props) -> Result<GbJoypad> {
        props.reader().finish()?;
        Ok(GbJoypad::new())
    }

    /// Which buttons are held, one bit each — see [`Button::bit`].
    #[must_use]
    pub fn buttons(&self) -> u8 {
        self.shared.state.lock().pressed
    }

    /// `$FF00` as the guest reads it.
    #[must_use]
    pub fn read(&self) -> u8 {
        self.shared.state.lock().read()
    }

    /// Press or release one button.
    ///
    /// The interim door for a non-deterministic input; see the module
    /// documentation.
    pub fn set_pressed(&self, button: Button, pressed: bool) {
        let asserted = {
            let mut state = self.shared.state.lock();
            if pressed {
                state.pressed |= 1 << button.bit();
            } else {
                state.pressed &= !(1 << button.bit());
            }
            state.asserted()
        };
        self.drive(asserted);
    }

    /// Set every button at once, as a packed mask.
    pub fn set_buttons(&self, pressed: u8) {
        let asserted = {
            let mut state = self.shared.state.lock();
            state.pressed = pressed;
            state.asserted()
        };
        self.drive(asserted);
    }

    /// Connect the joypad-interrupt request line.
    pub fn attach_irq(&self, source: WireSource) {
        *self.irq.lock() = Some(source);
        let asserted = self.shared.state.lock().asserted();
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

/// The `$FF00` register.
struct JoypadPort {
    shared: Arc<Shared>,
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
        *byte = self.shared.state.lock().read();
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        // Only the two select bits are writable; the rest are inputs.
        self.shared.state.lock().select = *value & 0x30;
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// The `gb.joypad` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "gb.joypad",
    version: 1,
    summary: "Game Boy joypad matrix ($FF00): two selectable rows of four active-low lines",
    properties: &[],
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
        let asserted = self.shared.state.lock().asserted();
        self.drive(asserted);
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
        self.attach_irq(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == IRQ_PIN {
            let asserted = self.shared.state.lock().asserted();
            self.drive(asserted);
        }
    }

    fn reset(&self, _kind: ResetKind) {
        // Which buttons a person is holding is not something a reset changes,
        // so only the select lines go back.
        self.shared.state.lock().select = 0;
        let asserted = self.shared.state.lock().asserted();
        self.drive(asserted);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u8(state.pressed)?;
        w.write_u8(state.select)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let state = State {
            pressed: r.read_u8()?,
            select: r.read_u8()?,
        };
        *self.shared.state.lock() = state;
        self.drive(state.asserted());
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
    use crate::machine::validate::{ClassSchema, PortDir};
    ClassSchema::new(CLASS.name)
        .port(IRQ_PIN, PortDir::Out)
        .region(REGISTER_REGION)
}
