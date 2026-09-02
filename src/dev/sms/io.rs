//! The Master System's I/O chip: the two controller ports, `$3E`, `$3F`, and
//! the console's own two buttons.
//!
//! Four addresses, decoded by two address lines and A0:
//!
//! ```text
//!   $3E  memory control    write-only: which chips answer the memory bus
//!   $3F  I/O port control  write-only: the TH/TR pins' direction and level
//!   $DC  I/O port A/B      read: port A's six lines, and port B's first two
//!   $DD  I/O port B/misc   read: the rest of port B, reset, and the TH pins
//! ```
//!
//! Every button line is **active low**: a zero means pressed. The chip supplies
//! pull-ups, so an unplugged port reads `$FF` and looks exactly like a pad with
//! nothing held — which is why a Master System never needs to know whether a
//! controller is connected.
//!
//! ```text
//!   $DC  bit 0-3  port A up, down, left, right
//!        bit 4-5  port A button 1 (TL), button 2 (TR)
//!        bit 6-7  port B up, down
//!   $DD  bit 0-1  port B left, right
//!        bit 2-3  port B button 1, button 2
//!        bit 4    the RESET button
//!        bit 5    unused, reads as one
//!        bit 6-7  port A TH, port B TH
//! ```
//!
//! # Pause is an NMI, and that is the whole design
//!
//! The **Pause** button is not in any register. It is wired straight to the
//! Z80's `/NMI`, so a game pauses by servicing `$0066` and nothing polls
//! anything. [`PAUSE_PIN`] is driven as a **level** and the core's pin latches
//! its edge — a net delivers changes, and a device that tried to deliver an
//! edge itself would have to invent one.
//!
//! **RESET** is different again: it is both a readable bit in `$DD` *and* wired
//! to the Z80's `/RESET`, so a game can notice the button without being reset
//! by it only because it is polling faster than the pulse. Both halves are here:
//! the bit, and [`RESET_PIN`].
//!
//! # `$3F` and the region check
//!
//! `$3F` sets each port's TH and TR pins to input or output and, when they are
//! outputs, what level they drive. It exists for the Light Phaser (which pulls
//! TH low on trigger, latching the VDP's H counter) and for the sports pad.
//!
//! Software also uses it to tell an export console from a Japanese one: drive
//! TH high, read `$DD`, drive it low, read again. An export machine feeds the
//! level back into bits 6 and 7; a Japanese one does not. That difference is
//! what [`Nationalisation`] selects, and it is the one behaviour in this file
//! that would most benefit from being checked against real hardware — it is
//! taken from SMS Power!'s description rather than measured.
//!
//! # `$3E`
//!
//! Six active-low enables for the chips on the memory bus. Only **bit 2**, the
//! I/O chip enable, has an effect here: with it set, `$DC` and `$DD` stop
//! answering and read as `$FF`. That is not a curiosity — it is the documented
//! way to reach the SDSC debug console at `$FC`/`$FD`. The BIOS, expansion and
//! card slots the other bits gate are not modelled, so their bits are recorded
//! and otherwise ignored.
//!
//! # Determinism
//!
//! Button state is a non-deterministic input crossing into the machine, and
//! `ROADMAP.md` §0 says every one of those goes through the record/replay seam
//! or it is a determinism bug. It does: the chip's state lives in a **named
//! host object**, [`SmsPads`], which this device opens by name from
//! `new(props)` the way a UART opens a character port — so [`pads::channel`] is
//! the channel a recorder registers, [`pads::sink`] is what a recorded payload
//! does, and a board whose pads the recorder does not know about is refused by
//! [`HostObjects::seal`](crate::core::hosts::HostObjects::seal) at build time.
//!
//! There is no second door. [`SmsPads::set_pressed`] and its neighbours are the
//! *device* side of the seam — what the channel's sink calls, exactly as
//! `CharPort::feed` is for a keystroke — and a host reaches them by opening the
//! pads by name, which is the act the seal checks.
//!
//! # Sources
//!
//! [SMS Power!'s development documents](https://www.smspower.org/Development/Documents),
//! the I/O port and peripheral pages. No emulator source of any licence was
//! consulted (`ROADMAP.md` §1).

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

/// The name a `map` statement reaches `$3E` and `$3F` by.
///
/// Two bytes: `$3E` at offset 0, `$3F` at offset 1. A board maps it with
/// `mirror()` across `$00`-`$3F`, because A0 is the only line decoded there.
pub const CONTROL_REGION: &str = "ctrl";

/// The name a `map` statement reaches `$DC` and `$DD` by.
///
/// Two bytes, mirrored across `$C0`-`$FF` for the same reason.
pub const PAD_REGION: &str = "pads";

/// The host pad port a machine gets when its description names none.
pub const DEFAULT_PAD_PORT: &str = "sms-pads";

/// The Pause button's output, wired to the core's `/NMI`.
pub const PAUSE_PIN: &str = "nmi";

/// The Reset button's output, wired to the core's `/RESET`.
pub const RESET_PIN: &str = "reset";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// Buttons
// ---------------------------------------------------------------------------

/// One of a control pad's six lines.
///
/// The discriminants are the bit order the chip reads them out in, which is
/// also the order they appear in `$DC` for port A.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Button {
    /// The d-pad's up.
    Up,
    /// The d-pad's down.
    Down,
    /// The d-pad's left.
    Left,
    /// The d-pad's right.
    Right,
    /// Button 1, the TL pin.
    One,
    /// Button 2, the TR pin.
    Two,
}

impl Button {
    /// Every button, in bit order.
    pub const ALL: [Button; 6] = [
        Button::Up,
        Button::Down,
        Button::Left,
        Button::Right,
        Button::One,
        Button::Two,
    ];

    /// The bit this button occupies in a port's packed state.
    #[must_use]
    pub const fn bit(self) -> u8 {
        match self {
            Button::Up => 0,
            Button::Down => 1,
            Button::Left => 2,
            Button::Right => 3,
            Button::One => 4,
            Button::Two => 5,
        }
    }

    /// The button's name, lowercase, as a front end would spell it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Button::Up => "up",
            Button::Down => "down",
            Button::Left => "left",
            Button::Right => "right",
            Button::One => "button1",
            Button::Two => "button2",
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

/// Which console this chip is in, for the `$3F` readback difference.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Nationalisation {
    /// A Western console: the TH output level appears in `$DD`.
    #[default]
    Export,
    /// A Japanese console: it does not.
    Japan,
}

impl Nationalisation {
    /// Look one up by the name a machine file writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Nationalisation> {
        match name {
            "export" => Some(Nationalisation::Export),
            "japan" => Some(Nationalisation::Japan),
            _ => None,
        }
    }

    /// The name a machine file writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Nationalisation::Export => "export",
            Nationalisation::Japan => "japan",
        }
    }
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
struct State {
    /// Which buttons are held on each port, one bit each. **1 means pressed**,
    /// the opposite of what the register reads, because a mask nobody has to
    /// invert is harder to get wrong.
    pads: [u8; 2],
    /// Whether Pause is held.
    pause: bool,
    /// Whether Reset is held.
    reset: bool,
    /// `$3E` as last written.
    memory_control: u8,
    /// `$3F` as last written.
    io_control: u8,
    nation: Nationalisation,
}

impl State {
    fn new(nation: Nationalisation) -> State {
        State {
            pads: [0; 2],
            pause: false,
            reset: false,
            // Every enable is active low and every chip is present at power-on.
            memory_control: 0,
            // Every pin an input, driving nothing.
            io_control: 0xff,
            nation,
        }
    }

    /// Whether the I/O chip answers at all. `$3E` bit 2 switches it out.
    fn io_enabled(&self) -> bool {
        self.memory_control & 0x04 == 0
    }

    /// The level a TH pin is at: what `$3F` drives when it is an output, and a
    /// pull-up when it is not.
    ///
    /// `$3F` bits 1 and 3 are the direction bits (0 output), bits 5 and 7 the
    /// levels.
    fn th_level(&self, port: usize) -> bool {
        let direction = 1u8 << (1 + port * 2);
        let level = 1u8 << (5 + port * 2);
        if self.io_control & direction == 0 {
            self.io_control & level != 0
        } else {
            true
        }
    }

    /// `$DC`.
    fn read_dc(&self) -> u8 {
        if !self.io_enabled() {
            return 0xff;
        }
        let a = self.pads[0] & 0x3f;
        let b = self.pads[1] & 0x03;
        !(a | (b << 6))
    }

    /// `$DD`.
    fn read_dd(&self) -> u8 {
        if !self.io_enabled() {
            return 0xff;
        }
        // Port B's remaining four lines, then Reset, then the unused bit.
        let b = (self.pads[1] >> 2) & 0x0f;
        let mut value = !b & 0x0f;
        value |= 0x20;
        if !self.reset {
            value |= 0x10;
        }
        if self.nation == Nationalisation::Export {
            if self.th_level(0) {
                value |= 0x40;
            }
            if self.th_level(1) {
                value |= 0x80;
            }
        }
        value
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

#[derive(Debug, Default)]
struct Links {
    pause: Option<WireSource>,
    reset: Option<WireSource>,
}

/// The chip's state: the two control pads, the console's own two buttons, the
/// control registers, and the pins the buttons drive.
///
/// This is the **host object** a build files under [`pads::KIND`] and the name
/// the machine description gave — the console's controller ports rather than a
/// copy of them. The device holds it, both register blocks hold it, and a host
/// that presses a button opens it by name. One object rather than a device
/// handle plus a mirror of its state, because two would have to be kept in step
/// and nothing would check that they were.
///
/// The Pause and Reset pins live here too, and have to: a press is what drives
/// them, and a press comes from out here.
pub struct SmsPads {
    state: Mutex<State>,
    links: Mutex<Links>,
}

impl fmt::Debug for SmsPads {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SmsPads")
            .field("state", &self.state)
            .finish_non_exhaustive()
    }
}

impl Default for SmsPads {
    fn default() -> SmsPads {
        SmsPads::new(Nationalisation::Export)
    }
}

impl SmsPads {
    /// A chip with nothing held and every pin an input.
    #[must_use]
    pub fn new(nation: Nationalisation) -> SmsPads {
        SmsPads {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(nation)),
            links: Mutex::with_rank(LockRank::WIRE, Links::default()),
        }
    }

    /// Which console this chip is in.
    #[must_use]
    pub fn nationalisation(&self) -> Nationalisation {
        self.state.lock().nation
    }

    /// Say which console this chip is in.
    ///
    /// The device calls it after opening the port by name: a host that opened
    /// the same name first got the default, and the machine description is the
    /// authority on which console this is.
    pub fn set_nationalisation(&self, nation: Nationalisation) {
        self.state.lock().nation = nation;
    }

    /// Press or release one button on `port` (0 or 1).
    ///
    /// The device end of the record/replay channel; see the module docs.
    pub fn set_pressed(&self, port: usize, button: Button, pressed: bool) {
        let mut state = self.state.lock();
        let mask = 1u8 << button.bit();
        if pressed {
            state.pads[port & 1] |= mask;
        } else {
            state.pads[port & 1] &= !mask;
        }
    }

    /// Set every button on `port` at once, as a packed mask. **1 is pressed.**
    pub fn set_buttons(&self, port: usize, pressed: u8) {
        self.state.lock().pads[port & 1] = pressed & 0x3f;
    }

    /// Which buttons are held on `port`.
    #[must_use]
    pub fn buttons(&self, port: usize) -> u8 {
        self.state.lock().pads[port & 1]
    }

    /// Hold or release the Pause button.
    ///
    /// It drives a level; the core's pin latches the rising edge, which is what
    /// makes one press one NMI however long it is held.
    pub fn set_pause(&self, held: bool) {
        self.state.lock().pause = held;
        self.settle();
    }

    /// A complete Pause press, for a caller that does not model the button's
    /// travel.
    pub fn pulse_pause(&self) {
        self.set_pause(true);
        self.set_pause(false);
    }

    /// Hold or release the Reset button.
    pub fn set_reset(&self, held: bool) {
        self.state.lock().reset = held;
        self.settle();
    }

    /// Drive both console buttons' pins, with no lock of this device held.
    fn drive(&self, pause: bool, reset: bool) {
        let (p, r) = {
            let links = self.links.lock();
            (links.pause.clone(), links.reset.clone())
        };
        if let Some(p) = p {
            p.set(Level::from_bool(pause));
        }
        if let Some(r) = r {
            r.set(Level::from_bool(reset));
        }
    }

    fn settle(&self) {
        let (pause, reset) = {
            let state = self.state.lock();
            (state.pause, state.reset)
        };
        self.drive(pause, reset);
    }
}

/// The Master System's I/O chip.
pub struct SmsIo {
    pads: Arc<SmsPads>,
    control_region: RegionRef,
    pad_region: RegionRef,
}

impl fmt::Debug for SmsIo {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SmsIo")
            .field("state", &self.pads.state)
            .finish_non_exhaustive()
    }
}

impl Default for SmsIo {
    fn default() -> Self {
        SmsIo::new(Nationalisation::Export)
    }
}

impl SmsIo {
    /// A chip with a private pad port: nothing held, every pin an input.
    #[must_use]
    pub fn new(nation: Nationalisation) -> SmsIo {
        SmsIo::with_pads(Arc::new(SmsPads::new(nation)))
    }

    /// A chip reading `pads`, which the host may already hold.
    #[must_use]
    pub fn with_pads(pads: Arc<SmsPads>) -> SmsIo {
        let control_region = Arc::new(MmioRegion::io(
            "sms.io.ctrl",
            2,
            Arc::new(ControlPorts {
                pads: Arc::clone(&pads),
            }) as Arc<dyn MemOps>,
        ));
        let pad_region = Arc::new(MmioRegion::io(
            "sms.io.pads",
            2,
            Arc::new(PadPorts {
                pads: Arc::clone(&pads),
            }) as Arc<dyn MemOps>,
        ));
        SmsIo {
            pads,
            control_region,
            pad_region,
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If `region` is not `export` or `japan`, or an unknown property was given.
    pub fn from_props(props: &Props) -> Result<SmsIo> {
        let mut r = props.reader();
        let name = r.or_str("region", "export")?;
        let nation = Nationalisation::from_name(name).ok_or_else(|| Error::Config {
            at: String::from("region"),
            message: alloc::format!("`{name}` is not a console region; use `export` or `japan`"),
        })?;
        let port = r.or_str("pads", DEFAULT_PAD_PORT)?.to_string();
        r.finish()?;
        let pads = pads::attach(props, &port)?;
        // A host that opened this name first got a default console; the machine
        // description is the authority on which one it is.
        pads.set_nationalisation(nation);
        Ok(SmsIo::with_pads(pads))
    }

    /// The pad port this chip's buttons come from: the host end of the seam.
    #[must_use]
    pub fn pads(&self) -> &Arc<SmsPads> {
        &self.pads
    }

    /// Which console this chip is in.
    #[must_use]
    pub fn nationalisation(&self) -> Nationalisation {
        self.pads.nationalisation()
    }

    /// Which buttons are held on `port`.
    #[must_use]
    pub fn buttons(&self, port: usize) -> u8 {
        self.pads.buttons(port)
    }

    /// `$DC` as the guest reads it.
    #[must_use]
    pub fn read_dc(&self) -> u8 {
        self.pads.state.lock().read_dc()
    }

    /// `$DD` as the guest reads it.
    #[must_use]
    pub fn read_dd(&self) -> u8 {
        self.pads.state.lock().read_dd()
    }

    /// `$3E` as it was last written.
    #[must_use]
    pub fn memory_control(&self) -> u8 {
        self.pads.state.lock().memory_control
    }

    /// `$3F` as it was last written.
    #[must_use]
    pub fn io_control(&self) -> u8 {
        self.pads.state.lock().io_control
    }

    /// Write `$3E` or `$3F` as the guest would — offset 0 or 1.
    pub fn write_control(&self, offset: u64, value: u8) {
        let mut state = self.pads.state.lock();
        if offset & 1 == 0 {
            state.memory_control = value;
        } else {
            state.io_control = value;
        }
    }

    /// Connect the Pause line.
    pub fn attach_pause(&self, source: WireSource) {
        self.pads.links.lock().pause = Some(source);
        self.pads.settle();
    }

    /// Connect the Reset line.
    pub fn attach_reset(&self, source: WireSource) {
        self.pads.links.lock().reset = Some(source);
        self.pads.settle();
    }
}

/// The build's named Master System pad ports.
///
/// The same shape as [`chardev::ports`](crate::host::chardev::ports): a *name*
/// is the only thing that can travel from a machine description into a device
/// constructor, and both ends resolve it against the build's own
/// [`HostObjects`](crate::core::hosts::HostObjects).
///
/// ```text
/// machine file:  object io "sms.io" { pads = "sms-pads" }
/// device:        pads::attach(props, "sms-pads")  ──┐
/// host:          pads::open(&hosts, "sms-pads")   ──┴─► the same Arc<SmsPads>
/// ```
pub mod pads {
    use super::{Nationalisation, SmsPads};
    use alloc::string::String;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    use crate::core::error::Result;
    use crate::core::hosts::{HostKind, HostObjects};
    use crate::core::props::Props;
    use crate::core::record::{Channel, FnSink, InputSink};

    /// The kind a pad port is filed under in a build's [`HostObjects`].
    pub const KIND: HostKind = HostKind::new("pad");

    /// How many bytes one recorded press is: port A, port B, the console's own
    /// two buttons.
    pub const RECORD_BYTES: usize = 3;

    /// Bit 0 of a payload's third byte: the Pause button.
    pub const PAUSE: u8 = 0x01;

    /// Bit 1 of a payload's third byte: the Reset button.
    pub const RESET: u8 = 0x02;

    /// The pad port `name` refers to in `hosts`, creating it on first mention.
    ///
    /// The **host** side of the rendezvous. A port created here is an export
    /// console until a device says otherwise, which the device does from its
    /// own constructor.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Config`] if another kind of host object already holds
    /// that name.
    pub fn open(hosts: &HostObjects, name: &str) -> Result<Arc<SmsPads>> {
        hosts.open(KIND, name, || SmsPads::new(Nationalisation::Export))
    }

    /// The pad port `name` refers to in the build these properties belong to.
    ///
    /// The **device** side, called from `new(props)`: acquiring a host object is
    /// allocation, not an outward action ([`core::hosts`](crate::core::hosts)
    /// argues the case). A `Props` that belongs to no build gets a private port.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn attach(props: &Props, name: &str) -> Result<Arc<SmsPads>> {
        props.host(KIND, name, || SmsPads::new(Nationalisation::Export))
    }

    /// The pad port called `name`, if it has been opened.
    ///
    /// # Errors
    ///
    /// As [`open`].
    pub fn get(hosts: &HostObjects, name: &str) -> Result<Option<Arc<SmsPads>>> {
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

    /// The record/replay channel the pads called `name` are pressed through.
    ///
    /// `pad:sms-pads`, which is the same `(kind, name)` pair the host-object
    /// table files the port under — so a board whose pads have no channel is
    /// refused by [`HostObjects::seal`](crate::core::hosts::HostObjects::seal)
    /// naming this string.
    #[must_use]
    pub fn channel(name: &str) -> Channel {
        Channel::new(KIND, name)
    }

    /// The [`InputSink`] that applies a recorded payload to `pads`.
    ///
    /// [`RECORD_BYTES`] bytes: the held mask of port A, the held mask of port B
    /// — both in [`Button::bit`](super::Button::bit) order — and then the
    /// console's own buttons, [`PAUSE`] and [`RESET`]. Level rather than edge,
    /// because that is what the lines are; a Pause *press* is two payloads, one
    /// with the bit set and one without, exactly as a person's thumb produces.
    ///
    /// A longer payload is that state changing more than once and each group is
    /// applied in turn; a short trailing group applies the bytes it has, so a
    /// host that only ever touches port A may post one byte.
    ///
    /// No rewind hook: these are levels rather than a queue, and the levels are
    /// part of the machine snapshot a rewind restores.
    #[must_use]
    pub fn sink(pads: &Arc<SmsPads>) -> Arc<dyn InputSink> {
        let pads = Arc::clone(pads);
        Arc::new(FnSink::new("pad", move |payload: &[u8]| {
            for group in payload.chunks(RECORD_BYTES) {
                if let Some(a) = group.first() {
                    pads.set_buttons(0, *a);
                }
                if let Some(b) = group.get(1) {
                    pads.set_buttons(1, *b);
                }
                if let Some(console) = group.get(2) {
                    pads.set_pause(console & PAUSE != 0);
                    pads.set_reset(console & RESET != 0);
                }
            }
        }))
    }
}

// ---------------------------------------------------------------------------
// The apertures
// ---------------------------------------------------------------------------

/// `$3E` and `$3F`, write-only.
struct ControlPorts {
    pads: Arc<SmsPads>,
}

impl fmt::Debug for ControlPorts {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ControlPorts").finish_non_exhaustive()
    }
}

impl MemOps for ControlPorts {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // Nothing drives the bus here; a Z80 board's pull-ups answer.
        *byte = 0xff;
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // Writing `$3E` can switch the cartridge out from under the guest.
            return Ok(());
        }
        let mut state = self.pads.state.lock();
        if offset & 1 == 0 {
            state.memory_control = *value;
        } else {
            state.io_control = *value;
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// `$DC` and `$DD`, read-only.
struct PadPorts {
    pads: Arc<SmsPads>,
}

impl fmt::Debug for PadPorts {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PadPorts").finish_non_exhaustive()
    }
}

impl MemOps for PadPorts {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // Reading a pad has no side effect at all, so a debug read needs no
        // special case: this is the aperture a monitor can look at freely.
        let state = self.pads.state.lock();
        *byte = if offset & 1 == 0 {
            state.read_dc()
        } else {
            state.read_dd()
        };
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let [_] = src else {
            return Err(BusError::BadAccess);
        };
        // Nothing on a Master System listens here. A Game Gear's `$06` stereo
        // register would, which is one of the differences that machine file
        // would carry.
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

/// The `sms.io` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "sms.io",
    version: 1,
    summary: "Master System I/O: two control pads at $DC/$DD, $3E/$3F, Pause as an NMI",
    properties: IO_PROPERTIES,
    construct: |props| Ok(Box::new(SmsIo::from_props(props)?) as Box<dyn Device>),
};

/// The properties `sms.io` takes.
static IO_PROPERTIES: &[PropertySpec] = &[
    PropertySpec {
        name: "region",
        kind: ValueKind::Str,
        required: false,
        summary: "console region, for the `$3F` readback: `export` or `japan`",
    },
    PropertySpec {
        name: "pads",
        kind: ValueKind::Str,
        required: false,
        summary: "the host pad port buttons arrive through, by name (default \"sms-pads\")",
    },
];

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for SmsIo {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The realize sweep: say what both lines idle at, which is low with
        // neither button held (`ROADMAP.md` §4.3).
        self.pads.settle();
        Ok(())
    }

    fn unrealize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        self.pads.drive(false, false);
        let mut links = self.pads.links.lock();
        links.pause = None;
        links.reset = None;
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            CONTROL_REGION => Some(Arc::clone(&self.control_region)),
            PAD_REGION => Some(Arc::clone(&self.pad_region)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            PAUSE_PIN => self.attach_pause(source),
            RESET_PIN => self.attach_reset(source),
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!(
                        "the I/O chip drives `{PAUSE_PIN}` and `{RESET_PIN}`, nothing else"
                    ),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == PAUSE_PIN || port == RESET_PIN {
            self.pads.settle();
        }
    }

    fn reset(&self, _kind: ResetKind) {
        // Which buttons a person is holding is not something a reset changes,
        // so only the two control registers go back.
        {
            let mut state = self.pads.state.lock();
            state.memory_control = 0;
            state.io_control = 0xff;
        }
        self.pads.settle();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.pads.state.lock();
        w.write_u32(STATE_VERSION)?;
        w.write_u8(state.pads[0])?;
        w.write_u8(state.pads[1])?;
        w.write_bool(state.pause)?;
        w.write_bool(state.reset)?;
        w.write_u8(state.memory_control)?;
        w.write_u8(state.io_control)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let version = r.read_u32()?;
        if version != STATE_VERSION {
            return Err(Error::State(alloc::format!(
                "the I/O chip's snapshot is version {version}, this build writes {STATE_VERSION}"
            )));
        }
        {
            let mut state = self.pads.state.lock();
            state.pads[0] = r.read_u8()?;
            state.pads[1] = r.read_u8()?;
            state.pause = r.read_bool()?;
            state.reset = r.read_bool()?;
            state.memory_control = r.read_u8()?;
            state.io_control = r.read_u8()?;
        }
        // The restored state implies levels nothing has announced.
        self.pads.settle();
        Ok(())
    }
}

impl crate::machine::Instance for SmsIo {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(SmsIo::from_props(props)?)))
}

/// What the validator should know about `sms.io`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("region", ValueKind::Str).values(&["export", "japan"]))
        .prop(PropSchema::new("pads", ValueKind::Str))
        .port(PAUSE_PIN, PortDir::Out)
        .port(RESET_PIN, PortDir::Out)
        .region(CONTROL_REGION)
        .region(PAD_REGION)
}
