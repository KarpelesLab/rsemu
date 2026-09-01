//! The Apple 1's MC6821 PIA: the keyboard and the display, at `$D010-$D013`.
//!
//! # Sources
//!
//! * *MC6821 Peripheral Interface Adapter* data sheet, for the register model:
//!   two ports, each with a data-direction register, an output register and a
//!   control register overlaid two per address.
//! * Applefritter, *Apple I Replica Creation*, chapter 7 ("Understanding the
//!   Apple I"), table 7.5, for what each of the four addresses is on this
//!   board and for the display's `DA`/`RDA` handshake.
//! * The Woz Monitor's published register equates and its two polling loops,
//!   for the software-visible contract — those are addresses and a status bit,
//!   which are facts about the hardware (`ROADMAP.md` §1, "facts versus
//!   expression"). No monitor source is reproduced here; see
//!   [`monitor`](super::monitor) for the one rsemu ships.
//!
//! # The register map
//!
//! ```text
//!   $D010  KBD    port A data     the key, with bit 7 set
//!   $D011  KBDCR  control A       bit 7 set while a key is waiting
//!   $D012  DSP    port B data     write a character; bit 7 set while busy
//!   $D013  DSPCR  control B
//! ```
//!
//! Four registers, but **six**, because a 6821 overlays each port's data
//! register on its data-direction register and bit 2 of the control register
//! picks between them. That is not pedantry here: the very first thing an
//! Apple 1 monitor does is store `$7F` to `$D012` while bit 2 of `$D013` is
//! still clear, which sets PB0-PB6 to outputs and leaves PB7 an input. A model
//! that skipped the DDRs would take that `$7F` for a character and print one.
//!
//! ## Port A — the keyboard
//!
//! The keyboard drives seven bits of ASCII onto PA0-PA6 and a strobe onto CA1;
//! **PA7 is strapped to +5 V**, so a key always reads with bit 7 set and
//! software compares against `$8D` for Return. The strobe sets CRA bit 7
//! (`IRQA1`), which is the "a key is waiting" flag, and reading `$D010` clears
//! it — which is why the polling loop is `LDA $D011 / BPL …` and not a read of
//! the data register.
//!
//! ## Port B — the display
//!
//! PB0-PB6 carry a character to the terminal section, which is a 40x24
//! character generator built on shift-register memory rather than a frame
//! buffer. It cannot take a character whenever it likes: the PIA raises `DA`
//! ("data available"), the video section answers `RDA` when the cursor
//! position next comes round, and only then is the character taken. `DA` is
//! wired back to PB7 so software can see it, and PB7 is an *input*, which is
//! why `$D012` reads back the busy flag and writes a character.
//!
//! The rate that handshake runs at is one character per video field — about 60
//! a second — and that is genuinely what the machine felt like. So the display
//! here is **paced by a clock domain**: the machine file gives this object a
//! 60 Hz clock, one tick releases at most one character, and a guest polling
//! PB7 waits exactly as long as it should. Set `paced = false` (or give it no
//! clock at all) and characters are released on the write, which is what a test
//! wants.
//!
//! # What is not modelled, and why
//!
//! * **CA2/CB2 as pins.** `DA` is CB2 in hardware; here it is state the display
//!   half owns. Nothing on this board observes CB2 except the video section
//!   this device already is.
//! * **The interrupt outputs.** `IRQA`/`IRQB` are not connected to the 6502 on
//!   an Apple 1, so CRA/CRB bit 0's interrupt enable changes nothing here. It
//!   is still stored and read back, because software writes it.
//! * **CRA/CRB bit 6** (`IRQA2`/`IRQB2`) reads as 0: nothing drives CA2 or CB2
//!   as an input on this board.
//! * **The wider `$D000-$DFFF` mirroring.** The PIA's `CS0` is A4 and its
//!   register selects are A0 and A1, so it answers all over the `$Dxxx` page.
//!   The machine file maps the sixteen bytes at `$D010` — the four registers
//!   repeated four times, which is what A0/A1-only decoding gives — and leaves
//!   the rest of the page on the open bus.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::host::chardev::{CharDevice, ports};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
const CLASS_NAME: &str = "apple1.pia";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The character port a machine file gets if it names none.
const DEFAULT_PORT: &str = "console";

/// How many bytes of address space the four registers occupy.
///
/// The board decodes A0 and A1 only, so the sixteen bytes at `$D010` are these
/// four repeated four times — the machine file writes that as `mirror(pia)`.
pub const REGISTER_COUNT: u64 = 4;

/// Bit 2 of a control register: 0 selects the data-direction register at the
/// data address, 1 selects the peripheral register.
const CR_DDR_ACCESS: u8 = 0x04;

/// Bits 6 and 7 of a control register are the interrupt flags. They are set by
/// the hardware and cleared by reading the data register; a write leaves them.
const CR_FLAGS: u8 = 0xc0;

/// CRA bit 7 — `IRQA1`, set by the keyboard strobe on CA1.
const CR_IRQ1: u8 = 0x80;

/// PB7, which the display's `DA` line drives: set while the terminal section
/// still owes us a character.
const PB_BUSY: u8 = 0x80;

/// PA7, strapped to +5 V on the Apple 1's keyboard connector.
const PA_STRAP: u8 = 0x80;

/// The Apple 1's backspace, which the keyboard sends for its "rub out" key.
const APPLE1_RUBOUT: u8 = 0x5f;

/// The 6821 as the Apple 1 wires it, plus the keyboard and display on the far
/// side of it.
///
/// Two-phase like every device (`ROADMAP.md` §4.4): [`Pia::new`] validates
/// properties, opens its character port and builds the region;
/// [`Device::realize`] does nothing, because a `map` statement places the
/// region and the realizer does that afterwards.
#[derive(Debug)]
pub struct Pia {
    regs: Arc<Registers>,
    region: RegionRef,
}

/// The four registers, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    port: Arc<dyn CharDevice>,
    /// The name the port was opened under, for `Debug` and for diagnostics.
    port_name: String,
    /// Whether the display waits for a clock tick before taking a character.
    paced: bool,
}

/// Everything the guest can see or change.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct State {
    /// Control register A. Bits 6-7 are flags the hardware sets.
    cra: u8,
    /// Control register B.
    crb: u8,
    /// Data direction A. The Apple 1's monitor never writes it, so it stays 0
    /// and port A is all inputs — which is correct for a keyboard.
    ddra: u8,
    /// Data direction B. Set to `$7F` by the monitor: PB0-PB6 out, PB7 in.
    ddrb: u8,
    /// Output register A. Nothing on this board reads it back; stored so a
    /// snapshot round-trips what the guest wrote.
    ora: u8,
    /// Output register B — the character the display is holding.
    orb: u8,
    /// The latched key, with bit 7 set. Meaningless unless `key_ready`.
    key: u8,
    /// `IRQA1`: a key arrived and nothing has read `$D010` since.
    key_ready: bool,
    /// `DA`: the display is holding a character the video section has not
    /// taken yet.
    busy: bool,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("port", &self.port_name).field("paced", &self.paced);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Pia {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`](crate::core::Error::Property) if a property is of the wrong kind, or if one this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Pia> {
        let mut r = props.reader();
        let port_name = r.or("port", String::from(DEFAULT_PORT))?;
        let paced = r.or("paced", true)?;
        r.finish()?;
        Ok(Pia::with_port(
            ports::attach(props, &port_name)?,
            port_name,
            paced,
        ))
    }

    /// Build one against a character device the caller already has.
    ///
    /// The route a test takes: it holds the other end of the port and does not
    /// have to go through the name table to find it.
    #[must_use]
    pub fn with_port(port: Arc<dyn CharDevice>, port_name: String, paced: bool) -> Pia {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            port,
            port_name,
            paced,
        });
        let region = Arc::new(Region::io(
            "pia",
            REGISTER_COUNT,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Pia { regs, region }
    }

    /// The name of the character port this device is attached to.
    #[must_use]
    pub fn port_name(&self) -> &str {
        &self.regs.port_name
    }

    /// Whether the display waits for a clock tick before taking a character.
    #[must_use]
    pub fn is_paced(&self) -> bool {
        self.regs.paced
    }

    /// Whether the display is holding a character the video section has not
    /// taken yet — bit 7 of `$D012`, as software sees it.
    #[must_use]
    pub fn display_busy(&self) -> bool {
        self.regs.state.lock().busy
    }

    /// Whether a key is waiting — bit 7 of `$D011`, as software sees it.
    ///
    /// Polls the character port first, exactly as a guest read of `$D011`
    /// would, so a caller that has just fed the port sees the key.
    #[must_use]
    pub fn key_waiting(&self) -> bool {
        self.regs.poll_keyboard();
        self.regs.state.lock().key_ready
    }
}

impl Registers {
    /// Latch one byte from the port, if the keyboard's latch is free.
    ///
    /// The port's lock is a leaf and this device's is `LockRank::DEVICE`, so
    /// taking one inside the other is the ranked order rather than a violation
    /// of it (`core::sync`). It has to be nested: a read of `$D011` must
    /// answer *now*, so there is nothing to defer.
    fn poll_keyboard(&self) {
        let mut state = self.state.lock();
        if state.key_ready {
            return;
        }
        let Some(byte) = self.port.read_byte() else {
            return;
        };
        state.key = keyboard_code(byte);
        state.key_ready = true;
        state.cra |= CR_IRQ1;
    }

    /// Hand the character the display is holding to the terminal section.
    ///
    /// Returns whether one was released. A port that will not take the byte
    /// leaves `DA` asserted, which stalls the guest — that is back pressure
    /// arriving as the hardware would deliver it, not a dropped character.
    fn release_character(&self) -> bool {
        let byte = {
            let state = self.state.lock();
            if !state.busy {
                return false;
            }
            // PB7 is an input pin, so only PB0-PB6 reach the video section.
            state.orb & !PB_BUSY
        };
        if !self.port.write_byte(byte) {
            return false;
        }
        self.state.lock().busy = false;
        true
    }

    /// Read one register. `debug` suppresses every side effect.
    fn read_register(&self, index: u8, debug: bool) -> u8 {
        if !debug {
            self.poll_keyboard();
        }
        let mut state = self.state.lock();
        match index {
            // $D010: DDRA, or the keyboard.
            0 => {
                if state.cra & CR_DDR_ACCESS == 0 {
                    return state.ddra;
                }
                if !debug {
                    // Reading the peripheral register clears the port's
                    // interrupt flags — the 6821's rule, and what makes the
                    // "key waiting" bit self-clearing.
                    state.cra &= !CR_FLAGS;
                    state.key_ready = false;
                }
                state.key
            }
            // $D011: CRA, whose bit 7 is the key-waiting flag.
            1 => state.cra,
            // $D012: DDRB, or port B — output bits read back, PB7 reads DA.
            2 => {
                if state.crb & CR_DDR_ACCESS == 0 {
                    return state.ddrb;
                }
                let inputs = if state.busy { PB_BUSY } else { 0 };
                (state.orb & state.ddrb) | (inputs & !state.ddrb)
            }
            // $D013: CRB.
            _ => state.crb,
        }
    }

    /// Write one register, reporting whether a character now wants releasing.
    fn write_register(&self, index: u8, value: u8) -> bool {
        let mut state = self.state.lock();
        match index {
            0 => {
                if state.cra & CR_DDR_ACCESS == 0 {
                    state.ddra = value;
                } else {
                    state.ora = value;
                }
            }
            // Bits 6 and 7 belong to the hardware; a write cannot set or clear
            // them (MC6821 data sheet).
            1 => state.cra = (state.cra & CR_FLAGS) | (value & !CR_FLAGS),
            2 => {
                if state.crb & CR_DDR_ACCESS == 0 {
                    state.ddrb = value;
                } else {
                    state.orb = value;
                    // DA goes high on the write and stays there until the video
                    // section takes the character.
                    state.busy = true;
                    return true;
                }
            }
            _ => state.crb = (state.crb & CR_FLAGS) | (value & !CR_FLAGS),
        }
        false
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        *byte = self.read_register((offset & 3) as u8, attrs.debug);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write to `$D012` would put a character on the screen,
            // and to `$D011` would change what the next read means. Neither is
            // something the core can make harmless, so it is refused rather
            // than guessed at (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        if self.write_register((offset & 3) as u8, *value) && !self.paced {
            // Unpaced: the video section is infinitely fast, so the character
            // is gone before the store instruction finishes and DA never reads
            // as set. That is the mode a test runs in.
            self.release_character();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // A 6821 is on an 8-bit bus. A 16-bit read of `$D010` is not a thing
        // that can happen, and accepting one would invent a byte order.
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

/// The 6821 as an Apple 1 wires it.
pub static PIA_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Apple 1 MC6821: keyboard at $D010/$D011, display at $D012/$D013",
    properties: &[
        PropertySpec {
            name: "port",
            kind: ValueKind::Str,
            required: false,
            summary: "the character port to attach to, by name (default \"console\")",
        },
        PropertySpec {
            name: "paced",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether the display takes one character per clock tick (default true)",
        },
    ],
    construct: |props| Ok(Box::new(Pia::new(props)?)),
};

impl Device for Pia {
    fn class(&self) -> &'static DeviceClass {
        &PIA_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // The 6821's RESET pin clears every register, and both kinds of reset
        // on this board pull it: there is no battery-backed anything here.
        *self.regs.state.lock() = State::default();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        w.write_u8(state.cra)?;
        w.write_u8(state.crb)?;
        w.write_u8(state.ddra)?;
        w.write_u8(state.ddrb)?;
        w.write_u8(state.ora)?;
        w.write_u8(state.orb)?;
        w.write_u8(state.key)?;
        w.write_bool(state.key_ready)?;
        w.write_bool(state.busy)
        // The port's queues are deliberately absent: what a user has typed and
        // not yet been read, and what the screen has shown, are the host's
        // state and not the machine's (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let state = State {
            cra: r.read_u8()?,
            crb: r.read_u8()?,
            ddra: r.read_u8()?,
            ddrb: r.read_u8()?,
            ora: r.read_u8()?,
            orb: r.read_u8()?,
            key: r.read_u8()?,
            key_ready: r.read_bool()?,
            busy: r.read_bool()?,
        };
        *self.regs.state.lock() = state;
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        // `""` for `map … = pia`, `"regs"` for anyone who prefers to say which.
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn is_runnable(&self) -> bool {
        // Not because it executes anything, but because the display's rate is
        // real and a tick of its clock domain is one character time. The
        // scheduler hands out those ticks; the alternative is a device reading
        // a clock, which nothing below `host/` may do (`CLAUDE.md`).
        self.regs.paced
    }

    fn run(&self, budget: Budget) -> Consumed {
        // At most one character per call, which is right however many ticks
        // this budget covers: the guest cannot write the next one until this
        // one has gone, so a backlog is impossible by construction.
        self.regs.release_character();
        // Poll the keyboard here as well as on a register read, so a guest that
        // reads `$D010` without polling `$D011` — legal, if unusual — still
        // sees keys.
        self.regs.poll_keyboard();
        Consumed::new(budget.ticks)
    }
}

impl Instance for Pia {}

/// What an Apple 1 keyboard would have put on PA0-PA7 for a host byte.
///
/// The keyboard is an upper-case ASCII keyboard: seven bits of data with PA7
/// strapped to +5 V, no lower case at all, and "rub out" rather than a
/// backspace. Translating here rather than in the backend is deliberate — this
/// is a property of the *keyboard*, and a 16550 on the same
/// [`CharDevice`](crate::host::chardev::CharDevice) must not inherit it.
fn keyboard_code(byte: u8) -> u8 {
    let ascii = match byte {
        // Both line endings are Return; a host sending CR LF would otherwise
        // enter two lines.
        b'\n' | b'\r' => 0x0d,
        // Backspace and delete both become the Apple 1's rub-out key.
        0x08 | 0x7f => APPLE1_RUBOUT,
        b'a'..=b'z' => byte.to_ascii_uppercase(),
        other => other & 0x7f,
    };
    ascii | PA_STRAP
}

/// Add [`PIA_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&PIA_CLASS)
}

/// Bind [`PIA_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Pia::new(props)?)))
}

/// What the validator should know about `apple1.pia`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("port", ValueKind::Str))
        .prop(PropSchema::new("paced", ValueKind::Bool))
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::host::chardev::CharPort;
    use alloc::string::ToString;
    use alloc::vec::Vec;

    /// A PIA with the far end of its port in hand, running flat out.
    fn wired() -> (Pia, Arc<CharPort>) {
        let port = Arc::new(CharPort::new());
        let pia = Pia::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            "test".to_string(),
            false,
        );
        (pia, port)
    }

    /// Read a register the way the address space would.
    fn peek(pia: &Pia, index: u64) -> u8 {
        let mut byte = [0u8; 1];
        pia.regs
            .read(index, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    fn peek_debug(pia: &Pia, index: u64) -> u8 {
        let mut byte = [0u8; 1];
        pia.regs
            .read(index, &mut byte, MemAttrs::DEBUG)
            .expect("a byte read is legal");
        byte[0]
    }

    fn poke(pia: &Pia, index: u64, value: u8) {
        pia.regs
            .write(index, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    /// Do what the Apple 1's monitor does at reset: DDRB, then both control
    /// registers.
    fn initialise(pia: &Pia) {
        poke(pia, 2, 0x7f);
        poke(pia, 1, 0xa7);
        poke(pia, 3, 0xa7);
    }

    #[test]
    fn before_the_control_registers_are_set_the_data_addresses_are_the_ddrs() {
        // The case a model without DDRs gets wrong: `$7F` to `$D012` at reset
        // sets the data directions and must not print a character.
        let (pia, port) = wired();
        poke(&pia, 2, 0x7f);
        assert!(port.drain().is_empty(), "$7F was taken for a character");
        assert_eq!(peek(&pia, 2), 0x7f, "reads back as DDRB");
        poke(&pia, 3, 0xa7);
        // Bits 6-7 belong to the hardware, so $A7 reads back as $27 until a
        // flag is set.
        assert_eq!(peek(&pia, 3), 0x27, "CRB, less the read-only flag bits");
        // Now $D012 is the port, and PB7 is an input reading DA — idle.
        assert_eq!(peek(&pia, 2) & PB_BUSY, 0);
    }

    #[test]
    fn a_character_written_to_the_display_reaches_the_port() {
        let (pia, port) = wired();
        initialise(&pia);
        poke(&pia, 2, 0xc1); // 'A' with bit 7 set, as an Apple 1 sends it
        // PB7 is an input pin, so only PB0-PB6 reach the video section.
        assert_eq!(port.drain(), b"A".to_vec());
        assert!(
            !pia.display_busy(),
            "unpaced: gone before the store returns"
        );
        // Bits 0-6 read back from the output register.
        assert_eq!(peek(&pia, 2), 0x41);
    }

    #[test]
    fn the_display_stays_busy_until_a_clock_tick_releases_the_character() {
        let port = Arc::new(CharPort::new());
        let pia = Pia::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            "test".to_string(),
            true,
        );
        initialise(&pia);
        poke(&pia, 2, 0xc8); // 'H'
        assert!(pia.display_busy());
        assert_eq!(peek(&pia, 2) & PB_BUSY, PB_BUSY, "software sees DA set");
        assert!(
            port.drain().is_empty(),
            "the video section has not taken it"
        );

        // One tick of the display's clock domain is one character time.
        let consumed = pia.run(Budget {
            until: crate::core::clock::GlobalTime::from_nanos(0),
            ticks: 1,
        });
        assert_eq!(consumed.ticks, 1, "the domain must advance");
        assert_eq!(port.drain(), b"H".to_vec());
        assert!(!pia.display_busy());
        assert_eq!(peek(&pia, 2) & PB_BUSY, 0);
    }

    #[test]
    fn a_key_sets_the_control_flag_and_reading_the_data_clears_it() {
        let (pia, port) = wired();
        initialise(&pia);
        assert_eq!(peek(&pia, 1) & CR_IRQ1, 0, "no key yet");

        port.feed(b"a");
        assert_eq!(peek(&pia, 1) & CR_IRQ1, CR_IRQ1, "bit 7 of $D011");
        // Upper case, with PA7 strapped high: 'a' arrives as $C1.
        assert_eq!(peek(&pia, 0), 0xc1);
        assert_eq!(peek(&pia, 1) & CR_IRQ1, 0, "reading $D010 cleared it");
    }

    #[test]
    fn the_keyboard_is_the_upper_case_one_the_apple_1_had() {
        assert_eq!(keyboard_code(b'a'), 0xc1);
        assert_eq!(keyboard_code(b'A'), 0xc1);
        assert_eq!(keyboard_code(b'\r'), 0x8d);
        assert_eq!(keyboard_code(b'\n'), 0x8d, "either line ending is Return");
        assert_eq!(keyboard_code(0x7f), 0xdf, "delete is rub out");
        assert_eq!(keyboard_code(0x08), 0xdf, "and so is backspace");
        assert_eq!(keyboard_code(b'.'), 0xae);
        // Bit 7 is strapped, so it is set whatever arrived.
        assert_eq!(keyboard_code(0xff) & PA_STRAP, PA_STRAP);
    }

    #[test]
    fn a_debug_access_changes_nothing() {
        // Invariant 5: a debugger read must not pop a FIFO or clear a flag.
        let (pia, port) = wired();
        initialise(&pia);
        port.feed(b"Z");
        // A debug read of $D011 does not even poll the port...
        assert_eq!(peek_debug(&pia, 1) & CR_IRQ1, 0);
        // ...and once the key is latched, a debug read of $D010 leaves it.
        assert_eq!(peek(&pia, 1) & CR_IRQ1, CR_IRQ1);
        assert_eq!(peek_debug(&pia, 0), 0xda);
        assert_eq!(peek(&pia, 1) & CR_IRQ1, CR_IRQ1, "still waiting");
        assert_eq!(peek(&pia, 0), 0xda);
        assert_eq!(peek(&pia, 1) & CR_IRQ1, 0);

        // A debug write is refused rather than guessed at.
        assert_eq!(
            pia.regs.write(2, &[0xc1], MemAttrs::DEBUG),
            Err(BusError::BadAccess)
        );
        assert!(port.drain().is_empty());
    }

    #[test]
    fn only_byte_accesses_are_accepted() {
        let (pia, _port) = wired();
        assert_eq!(
            pia.regs.read(0, &mut [0u8; 2], MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(
            pia.regs.write(0, &[0, 0], MemAttrs::DEFAULT),
            Err(BusError::BadAccess)
        );
        assert_eq!(pia.regs.constraints().min, Width::U8);
        assert_eq!(pia.regs.constraints().max, Width::U8);
    }

    #[test]
    fn a_port_that_will_not_take_the_byte_keeps_the_guest_waiting() {
        // Back pressure as the hardware would deliver it: DA stays set, so a
        // guest polling PB7 spins rather than losing the character.
        let port = Arc::new(CharPort::new());
        let pia = Pia::with_port(
            Arc::clone(&port) as Arc<dyn CharDevice>,
            "test".to_string(),
            true,
        );
        initialise(&pia);
        port.write(&alloc::vec![b'x'; crate::host::chardev::PORT_CAPACITY]);
        poke(&pia, 2, 0xc1);
        pia.run(Budget {
            until: crate::core::clock::GlobalTime::from_nanos(0),
            ticks: 1,
        });
        assert!(pia.display_busy(), "still holding it");
        let _ = port.drain();
        pia.run(Budget {
            until: crate::core::clock::GlobalTime::from_nanos(0),
            ticks: 1,
        });
        assert!(!pia.display_busy());
        assert_eq!(port.drain(), b"A".to_vec(), "and nothing was lost");
    }

    #[test]
    fn a_reset_clears_every_register() {
        let (pia, _port) = wired();
        initialise(&pia);
        pia.reset(ResetKind::Cold);
        assert_eq!(peek(&pia, 1), 0);
        assert_eq!(peek(&pia, 3), 0);
        // And bit 2 of the control registers is clear again, so $D012 is DDRB.
        assert_eq!(peek(&pia, 2), 0);
    }

    #[test]
    fn properties_are_checked() {
        assert!(Pia::new(&Props::new()).is_ok(), "everything has a default");
        let pia = Pia::new(&Props::new().with("port", "test.pia.props")).expect("a name");
        assert_eq!(pia.port_name(), "test.pia.props");
        assert!(pia.is_paced());

        let pia = Pia::new(&Props::new().with("paced", Value::Bool(false))).expect("unpaced");
        assert!(!pia.is_paced());
        assert!(!pia.is_runnable(), "nothing to schedule");

        let err = Pia::new(&Props::new().with("prot", "console"))
            .expect_err("a typo")
            .to_string();
        assert!(err.contains("prot") && err.contains("port"), "{err}");
    }

    #[test]
    fn the_whole_register_block_is_the_region() {
        let (pia, _port) = wired();
        let region = pia.region("").expect("the default region");
        assert_eq!(region.len(), REGISTER_COUNT);
        assert!(pia.region("regs").is_some());
        assert!(pia.region("keyboard").is_none());
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_state() {
        let (saved, port) = wired();
        initialise(&saved);
        port.feed(b"Q");
        assert!(saved.key_waiting());

        let mut shape = MachineShape::new();
        shape.add_device("pia", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("pia", CLASS_NAME, STATE_VERSION).unwrap();
            saved.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let (restored, _other) = wired();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("pia", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        // Every guest-visible register, read the way the guest would.
        let before: Vec<u8> = (0..4).map(|i| peek_debug(&saved, i)).collect();
        let after: Vec<u8> = (0..4).map(|i| peek_debug(&restored, i)).collect();
        assert_eq!(before, after);
        assert_eq!(restored.regs.state.lock().key, 0xd1);
    }

    #[test]
    fn the_class_is_registrable_and_describes_itself() {
        let mut registry = crate::core::Registry::new();
        register(&mut registry).expect("a fresh registry");
        let class = registry.get(CLASS_NAME).expect("registered");
        assert_eq!(class.version, STATE_VERSION);
        assert_eq!(class.properties.len(), 2);
        let device = (class.construct)(&Props::new().with("port", "test.pia.registry"))
            .expect("defaults are enough");
        assert_eq!(device.class().name, CLASS_NAME);
    }
}
