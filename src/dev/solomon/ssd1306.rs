//! The Solomon Systech SSD1306 / SSD1309 and the Sino Wealth SH1106: the
//! monochrome OLED controller on more hobby boards than anything else.
//!
//! 128×64 or 128×32 dots of white-on-black OLED, driven over 4-wire SPI, 3-wire
//! SPI or I²C, with the picture held in the **controller's own GDDRAM** — which
//! is the whole reason this device exists in the tree and is not another
//! [`st7272a`](crate::dev::sitronix::st7272a). Nothing in guest memory is the
//! screen; the guest *sends* the screen, a byte at a time, and the panel keeps
//! it. See [`crate::dev::lcd::panel`] for what that costs and what it buys.
//!
//! # Sources
//!
//! * Solomon Systech **SSD1306** Advance Information, Rev 1.1 (April 2008),
//!   cited as §. The SSD1309 is the same command set at 128×64 with a different
//!   charge pump, and is a [`Variant`] of this model rather than a file.
//! * Sino Wealth **SH1106** datasheet Rev 0.1, for the differences [`Variant`]
//!   names. It is a *different vendor's* part and not a Solomon one; it lives
//!   here because a driver treats it as an SSD1306 with two quirks, and
//!   splitting it into its own file would duplicate nine hundred lines to
//!   express a column offset.
//!
//! No emulator was consulted (`ROADMAP.md` §1).
//!
//! # GDDRAM, and the one thing everyone gets wrong
//!
//! §8.7: the display RAM is **page addressed**. A page is eight rows; a byte is
//! one column of one page, `D0` the topmost of its eight rows and `D7` the
//! bottom. 128 columns × 8 pages is a 128×64 screen.
//!
//! The SSD1306 has exactly 128 columns of RAM. **The SH1106 has 132**, and a
//! 128-pixel module wires the glass to `SEG2`..`SEG129` — so every address is
//! two columns further along than the same driver would put it on an SSD1306,
//! and a driver written for one produces a two-pixel-shifted, two-pixel-clipped
//! picture on the other. That is [`Variant::column_offset`], it is the single
//! most reported bug in every SSD1306 library, and
//! `the_sh1106_shifts_the_visible_window_two_columns` is the test.
//!
//! # Addressing modes (§10.1.3, `20h`)
//!
//! | Mode | After a byte | At the end of the column range | At the end of the page range |
//! | --- | --- | --- | --- |
//! | Page (`10`, the reset default) | column + 1 | back to column 0, **page unchanged** | — |
//! | Horizontal (`00`) | column + 1 | column ← start, page + 1 | page ← start |
//! | Vertical (`01`) | page + 1 | — | page ← start, column + 1 |
//!
//! Page mode wraps at the end of *RAM* (127, or 131 on an SH1106) rather than
//! at the `21h` column range, because §10.1.3's note says `21h` and `22h` are
//! "for horizontal or vertical addressing mode only". The SH1106 has no `20h`,
//! `21h` or `22h` at all and is always in page mode.
//!
//! # What a viewer sees
//!
//! §10.1.14 `40h`-`7Fh` start line, §10.1.20 `D3h` offset, §10.1.11 `A0h`/`A1h`
//! segment remap, §10.1.17 `C0h`/`C8h` COM scan direction, §10.1.16 `A8h`
//! multiplex ratio, §10.1.12 `A4h`/`A5h`, §10.1.13 `A6h`/`A7h`, §10.1.18
//! `AEh`/`AFh`:
//!
//! ```text
//!   screen row y  ──C0/C8──►  scan index  ──40h,D3h──►  RAM row  ──►  page,bit
//!   screen col x  ──offset──►  SEG index  ──A0h/A1h──►  RAM column
//! ```
//!
//! * Rows at or past the multiplex ratio are **not driven at all** and are
//!   black: `A8h` is how many COM lines exist, and a 128×32 module sets it to
//!   31.
//! * The start line and the offset are both a modulo-64 rotation of the same
//!   row counter, applied in opposite directions: `40h+n` starts the counter at
//!   RAM row `n`, `D3h` shifts the picture *down* by COM. At the reset
//!   multiplex of 64 they are exactly inverse, which is why so much firmware
//!   sets one and ignores the other.
//! * Contrast (`81h`) is the segment drive current, so it scales how bright a
//!   lit pixel is and does nothing to an unlit one. The model hands out that
//!   intensity as an RGB triple and says nothing about what colour the panel is
//!   — `dev/` names no colours (`host::display`).
//!
//! # `A1h`/`C8h` and which way up the glass is
//!
//! Every initialisation sequence in the world sends `A1h` and `C8h`, and the
//! reason is **not** that the picture wants mirroring — it is that the 0.96″ and
//! 1.3″ modules have their glass bonded 180° to the die's nominal `SEG`/`COM`
//! order, and those two commands cancel it. `SEG`-to-glass is a *board* fact
//! that no register can observe, so it is the [`Mount`] property, and a board
//! that buys the ordinary module writes `mount = "rotated"`. Without it, the
//! standard sequence would produce an upside-down picture here and the guest
//! would get the blame for the module's wiring.
//!
//! # Both transports, and what differs between them
//!
//! * **4-wire SPI** (§8.1.3): eight bits, MSB first, latched on the rising edge
//!   of `SCLK` — [`Mode::Mode0`] — and the `D/C̅` *pin* says whether the byte is
//!   a command or data. An ordinary [`WireSink`], because that is exactly what
//!   the pin is.
//! * **3-wire SPI** (§8.1.4): nine bits, and the **first** is `D/C̅`. The `D/C̅`
//!   pin is unused and tied low. Expressible with no extra wires at all, which
//!   is why `machines/oled-spi.machine` uses it.
//! * **I²C** (§8.1.5): the slave address is `0111100b` or `0111101b` depending
//!   on `SA0`, and after it comes a **control byte** — `Co` in bit 7, `D/C̅` in
//!   bit 6, the rest zero. `Co = 0` means "everything until the STOP is of this
//!   type", which is how a driver pushes a whole frame behind one `40h`;
//!   `Co = 1` means the next byte is one lone datum and another control byte
//!   follows it. **Reads are not supported over I²C** (§8.1.5.2 lists no read
//!   sequence and the part has no way to drive SDA with data), so this model
//!   NACKs a read address rather than inventing an answer.
//!
//! The command interpreter is one function and all three transports feed it, so
//! `the_i2c_transport_with_control_bytes_lands_in_the_same_ram` is a claim
//! about the *seam* rather than about a second copy of the command set.
//!
//! # What is recorded but inert
//!
//! `8Dh` charge pump, `D5h` clock divide, `D9h` pre-charge, `DAh` COM pins
//! hardware configuration and `DBh` VCOMH deselect are all analogue or timing
//! parameters. They are stored, because firmware writes them and a model that
//! forgot them would desynchronise nothing but would also lose state a snapshot
//! is supposed to carry, and they change no pixel.
//!
//! **The scroll commands `26h`-`2Fh` and `A3h` are parsed and recorded but do
//! not animate.** Their parameter counts are honoured exactly — six for
//! `26h`/`27h`, five for `29h`/`2Ah`, two for `A3h` — so a stream that uses them
//! stays in sync, which is the failure that would actually corrupt a picture.
//! What is missing is the motion: §10.2.3 scrolls by one column every `n`
//! *frames*, and this device has no frames (see [`crate::dev::lcd::panel`]), so
//! there is no honest tick to move it on. Anything that scrolls would be a made
//! up rate. `2Fh` therefore sets a flag a test can read and moves nothing, and
//! this paragraph is the promise that it is a gap rather than a claim.
//!
//! # Time
//!
//! There is none. This device has no clock domain, is not lazily advanced, and
//! registers no event: every guest-visible effect happens inside the byte that
//! caused it. The refresh that lights the glass runs on an internal RC
//! oscillator with no pin, no readable counter and no interrupt, so modelling
//! it would add a scheduler event nothing could observe.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::i2c::wires::{SlaveWires, SlaveWiresState, pin as i2c_line};
use crate::bus::i2c::{Ack, Address, Direction, I2cBus, I2cSlave, buses as i2c_buses};
use crate::bus::spi::{
    BitOrder, ChipSelect, Format, MAX_CHIP_SELECTS, Mode, SlavePins, SpiSlave, buses as spi_buses,
    pin as spi_pin,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{Level, WireId, WireSink, WireSource};
use crate::dev::lcd::panel::{Built, Panel, PanelClass};
use crate::machine::realize::Instance;

#[cfg(test)]
mod tests;

/// The class name a machine description writes.
const CLASS_NAME: &str = "solomon.ssd1306";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// Rows in one GDDRAM page (§8.7).
pub const PAGE_ROWS: u32 = 8;

/// How many rows of GDDRAM the row counter walks, whatever the multiplex ratio
/// is (§10.1.14: the start line selects "a value from 0 to 63").
pub const RAM_ROWS: u32 = 64;

/// The default panel: the 128×64 module (§1).
pub const DEFAULT_WIDTH: u64 = 128;
/// Ditto.
pub const DEFAULT_HEIGHT: u64 = 64;

/// The seven-bit I²C address with `SA0` low (§8.1.5).
pub const I2C_ADDRESS_SA0_LOW: u8 = 0x3c;
/// The same with `SA0` high.
pub const I2C_ADDRESS_SA0_HIGH: u8 = 0x3d;

/// The reset contrast, `7Fh` (§10.1.7).
const RESET_CONTRAST: u8 = 0x7f;
/// The reset clock divide / oscillator frequency, `80h` (§10.1.16).
const RESET_CLOCK: u8 = 0x80;
/// The reset pre-charge period, `22h` (§10.1.21).
const RESET_PRECHARGE: u8 = 0x22;
/// The reset COM pins configuration, `12h` (§10.1.22).
const RESET_COM_PINS: u8 = 0x12;
/// The reset VCOMH deselect level, `20h` (§10.1.23).
const RESET_VCOMH: u8 = 0x20;

/// The I²C control byte's `Co` bit (§8.1.5.1).
const CONTROL_CO: u8 = 1 << 7;
/// The I²C control byte's `D/C̅` bit.
const CONTROL_DC: u8 = 1 << 6;

// ---------------------------------------------------------------------------
// Variants
// ---------------------------------------------------------------------------

/// Which member of the family this is.
///
/// A real enumeration rather than the `pktkit` newtype pattern (`CLAUDE.md`):
/// every use below is an exhaustive `match` on a closed set of silicon, and a
/// fourth part is a considered addition rather than a constant somebody drops
/// in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum Variant {
    /// Solomon Systech SSD1306: 128 columns of GDDRAM, all three addressing
    /// modes, the scroll commands.
    #[default]
    Ssd1306,
    /// Solomon Systech SSD1309. Command-compatible with the SSD1306; the
    /// differences are the charge pump and the panel it is bonded to, neither
    /// of which a model can show.
    Ssd1309,
    /// Sino Wealth SH1106: **132 columns** of GDDRAM, page addressing only, no
    /// scroll, and a `30h`-`33h` pump-voltage command the Solomon parts do not
    /// have.
    Sh1106,
}

impl Variant {
    /// The spelling a machine description writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Variant> {
        match name {
            "ssd1306" => Some(Variant::Ssd1306),
            "ssd1309" => Some(Variant::Ssd1309),
            "sh1106" => Some(Variant::Sh1106),
            _ => None,
        }
    }

    /// The spelling a machine description writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Variant::Ssd1306 => "ssd1306",
            Variant::Ssd1309 => "ssd1309",
            Variant::Sh1106 => "sh1106",
        }
    }

    /// Every spelling, for the validator.
    pub const NAMES: &'static [&'static str] = &["ssd1306", "ssd1309", "sh1106"];

    /// How many columns of GDDRAM the part holds.
    #[must_use]
    pub const fn ram_columns(self) -> u32 {
        match self {
            Variant::Sh1106 => 132,
            _ => 128,
        }
    }

    /// Which `SEG` output the leftmost visible pixel of a 128-wide module hangs
    /// off.
    ///
    /// Two on an SH1106, whose 132 columns are centred on a 128-dot glass, and
    /// zero on the Solomon parts. The classic difference, and the one a driver
    /// written for the other part gets wrong.
    #[must_use]
    pub const fn column_offset(self) -> u32 {
        match self {
            Variant::Sh1106 => 2,
            _ => 0,
        }
    }

    /// Whether the part has `20h`/`21h`/`22h` and the scroll commands.
    ///
    /// The SH1106's command table has none of them: it is page addressed, full
    /// stop.
    #[must_use]
    pub const fn has_addressing_modes(self) -> bool {
        !matches!(self, Variant::Sh1106)
    }
}

// ---------------------------------------------------------------------------
// Addressing
// ---------------------------------------------------------------------------

/// How the address pointer advances after a data byte (§10.1.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AddrMode {
    /// `20h 00`: column first, then page.
    Horizontal,
    /// `20h 01`: page first, then column.
    Vertical,
    /// `20h 10`, and the reset default (§10.1.3: "Page addressing mode
    /// (RESET)").
    #[default]
    Page,
}

impl AddrMode {
    /// The two-bit encoding `20h` takes.
    const fn from_bits(bits: u8) -> AddrMode {
        match bits & 0b11 {
            0b00 => AddrMode::Horizontal,
            0b01 => AddrMode::Vertical,
            // `11` is "invalid" in §10.1.3's table; the part is page addressed
            // out of reset and there is nothing else for it to become.
            _ => AddrMode::Page,
        }
    }

    /// A stable code for the snapshot.
    const fn code(self) -> u8 {
        match self {
            AddrMode::Horizontal => 0,
            AddrMode::Vertical => 1,
            AddrMode::Page => 2,
        }
    }

    /// The inverse. An unknown code loads as the reset mode: a snapshot is
    /// untrusted input.
    const fn from_code(code: u8) -> AddrMode {
        match code {
            0 => AddrMode::Horizontal,
            1 => AddrMode::Vertical,
            _ => AddrMode::Page,
        }
    }
}

// ---------------------------------------------------------------------------
// Which transport a board wired up
// ---------------------------------------------------------------------------

/// The transport a particular board uses.
///
/// A property rather than something discovered, because it is a *board*
/// decision made with the `BS1`/`BS2` strapping pins at assembly time (§8.1,
/// Table 8-1) and the part cannot change it at runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Interface {
    /// 4-wire SPI: eight-bit words and a `D/C̅` pin (§8.1.3).
    #[default]
    Spi4,
    /// 3-wire SPI: nine-bit words whose first bit is `D/C̅` (§8.1.4).
    Spi3,
    /// I²C, with a control byte in front of every run of bytes (§8.1.5).
    I2c,
}

impl Interface {
    /// The spelling a machine description writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Interface> {
        match name {
            "spi4" => Some(Interface::Spi4),
            "spi3" => Some(Interface::Spi3),
            "i2c" => Some(Interface::I2c),
            _ => None,
        }
    }

    /// The spelling a machine description writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Interface::Spi4 => "spi4",
            Interface::Spi3 => "spi3",
            Interface::I2c => "i2c",
        }
    }

    /// Every spelling, for the validator.
    pub const NAMES: &'static [&'static str] = &["spi4", "spi3", "i2c"];

    /// Whether this is one of the two SPI framings.
    #[must_use]
    pub const fn is_spi(self) -> bool {
        matches!(self, Interface::Spi4 | Interface::Spi3)
    }
}

/// Which way round the glass is bonded to the die.
///
/// **This is a board fact, not a register.** The controller drives `SEG0`..
/// `SEG127` and `COM0`..`COM63`; which corner of the glass those land on is
/// decided by whoever built the module, and the datasheet's own block diagram
/// and Figure 10-1 draw `SEG` increasing to the right and `COM` downward, which
/// is [`Mount::Normal`].
///
/// The 0.96″ and 1.3″ modules everybody actually buys are bonded the other way
/// up — which is **the entire reason `A1h` and `C8h` exist**, and why every
/// driver's initialisation sequence contains both. A model that had no way to
/// say so would show those boards' output rotated by 180° and would be blaming
/// the guest for the module's wiring.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mount {
    /// `SEG0` and `COM0` at the top left: the die's own order.
    #[default]
    Normal,
    /// `SEG0` and `COM0` at the bottom right: the common module, whose firmware
    /// sends `A1h` and `C8h` to cancel it out.
    Rotated,
}

impl Mount {
    /// The spelling a machine description writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Mount> {
        match name {
            "normal" => Some(Mount::Normal),
            "rotated" => Some(Mount::Rotated),
            _ => None,
        }
    }

    /// The spelling a machine description writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Mount::Normal => "normal",
            Mount::Rotated => "rotated",
        }
    }

    /// Every spelling, for the validator.
    pub const NAMES: &'static [&'static str] = &["normal", "rotated"];
}

/// The pin names a machine description wires, beyond the bus's own.
pub mod pin {
    /// The data/command select, `D/C̅`. High is data (§8.1.3). 4-wire SPI only.
    pub const DC: &str = "dc";
    /// The reset input, `RES̅`. Active low (§8.5).
    pub const RES: &str = "res";

    /// Wire line for [`DC`]. Numbered past both bus front ends' lines so one
    /// device can host them without a collision.
    pub const DC_LINE: u32 = 16;
    /// Wire line for [`RES`].
    pub const RES_LINE: u32 = 17;
}

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

/// Everything the command set sets, and nothing the transports need.
///
/// Split out from the rest of the state so `E3h`-to-reset and the `RES̅` pin
/// can restore the whole lot in one assignment, which is what §8.5 says the
/// pin does.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Registers {
    /// `20h`: how the pointer advances.
    pub mode: AddrMode,
    /// The column the next data byte lands in, in RAM columns.
    pub column: u8,
    /// The page the next data byte lands in.
    pub page: u8,
    /// `21h`: the first and last column of the window, horizontal and vertical
    /// modes only.
    pub column_range: (u8, u8),
    /// `22h`: the first and last page of the window.
    pub page_range: (u8, u8),
    /// `40h`-`7Fh`: which RAM row the counter starts at.
    pub start_line: u8,
    /// `D3h`: how far down the picture is shifted, by COM.
    pub offset: u8,
    /// `81h`: segment drive current, and so how bright a lit dot is.
    pub contrast: u8,
    /// `A0h`/`A1h`: `true` when column 127 (or 131) is `SEG0`.
    pub segment_remap: bool,
    /// `C0h`/`C8h`: `true` when the COM scan is remapped, which mirrors the
    /// picture vertically.
    pub com_remap: bool,
    /// `A6h`/`A7h`: `true` when a RAM `0` is a lit dot.
    pub inverse: bool,
    /// `A4h`/`A5h`: `true` when the output ignores RAM and lights everything.
    pub entire_on: bool,
    /// `AEh`/`AFh`: `true` when the panel is lit at all.
    pub display_on: bool,
    /// `A8h`: the multiplex ratio, as the register holds it — one less than the
    /// number of driven COM lines.
    pub multiplex: u8,
    /// `8Dh`: the charge-pump setting. Recorded; inert.
    pub charge_pump: u8,
    /// `D5h`: clock divide and oscillator frequency. Recorded; inert.
    pub clock: u8,
    /// `D9h`: pre-charge period. Recorded; inert.
    pub precharge: u8,
    /// `DAh`: COM pins hardware configuration. Recorded; inert.
    pub com_pins: u8,
    /// `DBh`: VCOMH deselect level. Recorded; inert.
    pub vcomh: u8,
    /// `ADh` on the SH1106: DC-DC control. Recorded; inert.
    pub dc_dc: u8,
    /// `2Fh`/`2Eh`: whether scrolling is armed. Recorded; nothing moves — see
    /// the module docs.
    pub scrolling: bool,
    /// The last `26h`/`27h`/`29h`/`2Ah` parameter list, and `A3h`'s two.
    /// Recorded; inert.
    pub scroll: [u8; 6],
    /// `A3h`: the fixed and scrolling row counts. Recorded; inert.
    pub scroll_area: (u8, u8),
}

impl Default for Registers {
    fn default() -> Registers {
        Registers::new()
    }
}

impl Registers {
    /// Every register at the value §8.5's reset leaves it at.
    #[must_use]
    pub const fn new() -> Registers {
        Registers {
            mode: AddrMode::Page,
            column: 0,
            page: 0,
            column_range: (0, 127),
            page_range: (0, 7),
            start_line: 0,
            offset: 0,
            contrast: RESET_CONTRAST,
            segment_remap: false,
            com_remap: false,
            inverse: false,
            entire_on: false,
            // §8.5: "the display is in OFF state" after reset.
            display_on: false,
            multiplex: 63,
            charge_pump: 0x10,
            clock: RESET_CLOCK,
            precharge: RESET_PRECHARGE,
            com_pins: RESET_COM_PINS,
            vcomh: RESET_VCOMH,
            dc_dc: 0x8b,
            scrolling: false,
            scroll: [0; 6],
            scroll_area: (0, 64),
        }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The SSD1306 family as a device.
#[derive(Debug)]
pub struct Ssd1306 {
    shared: Arc<Shared>,
    /// The SPI bit-level front end. Present whatever the interface is, so the
    /// snapshot has one fixed layout; only an SPI board ever drives it.
    pins: Arc<SlavePins>,
    /// The I²C bit-level front end, on the same terms.
    wires: Arc<SlaveWires>,
    /// The I²C bus named by the machine file, joined at realize.
    i2c_bus: Option<Arc<I2cBus>>,
    /// The `dc` and `res` pins handed out by [`Device::sink`], kept alive
    /// because a net refers to its sinks weakly (`core::device`).
    control: Mutex<Vec<Arc<ControlSink>>>,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    variant: Variant,
    interface: Interface,
    /// Visible pixels across.
    width: u32,
    /// Visible pixels down.
    height: u32,
    /// Columns of GDDRAM, which is not always [`Shared::width`].
    ram_columns: u32,
    /// Where the leftmost visible pixel sits in `SEG` space.
    column_offset: u32,
    /// Which way round the glass is glued on.
    mount: Mount,
    /// The seven-bit I²C address, whatever the interface is.
    address: u8,
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// `ram_columns × pages` bytes of GDDRAM (§8.7).
    ram: Vec<u8>,
    /// How many times the picture has been able to change.
    generation: u64,
    /// The command set's registers.
    regs: Registers,
    /// The command waiting for parameters, and how many it still wants.
    pending: Option<(u8, u8)>,
    /// Parameters gathered so far. At most six (`26h`).
    args: [u8; 6],
    /// How many of `args` are filled.
    args_len: u8,
    /// The `D/C̅` pin, for 4-wire SPI. High is data.
    dc: Level,
    /// The `RES̅` pin. Low resets.
    ///
    /// It starts **high**, not at the low level a fresh net idles at: a board
    /// that ties `RES̅` to its power-on circuit must not hold this part in
    /// reset merely because the machine file did not name the pin.
    res: Level,
    /// The I²C transaction's state: whether the next byte is a control byte,
    /// and what the last one said.
    i2c: I2cPhase,
    /// How many bytes named a command the datasheet does not list. Diagnostics
    /// only.
    unlisted: u32,
}

/// Where an I²C transaction has got to (§8.1.5.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum I2cPhase {
    /// Not addressed.
    #[default]
    Idle,
    /// The next byte is a control byte.
    Control,
    /// Every byte until the STOP is of this type. `true` is data.
    Stream(bool),
    /// Exactly one byte of this type, then another control byte (`Co = 1`).
    One(bool),
}

impl I2cPhase {
    /// A stable code for the snapshot.
    const fn code(self) -> u8 {
        match self {
            I2cPhase::Idle => 0,
            I2cPhase::Control => 1,
            I2cPhase::Stream(false) => 2,
            I2cPhase::Stream(true) => 3,
            I2cPhase::One(false) => 4,
            I2cPhase::One(true) => 5,
        }
    }

    /// The inverse. An unknown code loads as idle: a snapshot is untrusted.
    const fn from_code(code: u8) -> I2cPhase {
        match code {
            1 => I2cPhase::Control,
            2 => I2cPhase::Stream(false),
            3 => I2cPhase::Stream(true),
            4 => I2cPhase::One(false),
            5 => I2cPhase::One(true),
            _ => I2cPhase::Idle,
        }
    }
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("variant", &self.variant)
            .field("interface", &self.interface)
            .field("width", &self.width)
            .field("height", &self.height);
        match self.state.try_lock() {
            // The RAM is the bulk of the state and never what the reader of a
            // failing test wants first.
            Some(state) => s.field("regs", &state.regs).finish(),
            None => s.field("regs", &"<in use>").finish(),
        }
    }
}

impl Ssd1306 {
    /// Validate `props` and build the controller.
    ///
    /// Properties:
    ///
    /// * `variant` — `ssd1306` (the default), `ssd1309` or `sh1106`.
    /// * `width`, `height` — the visible glass. 128 × 64 by default; the other
    ///   common module is 128 × 32, which also wants `multiplex` set by its
    ///   firmware.
    /// * `interface` — `spi4` (the default), `spi3` or `i2c`.
    /// * `mount` — `normal` (the default) or `rotated`, which is the common
    ///   module and the reason its firmware sends `A1h` and `C8h`.
    /// * `bus`, `cs` — the named SPI bus and chip select, for `spi4`/`spi3`.
    /// * `address` — the seven-bit I²C address, for `i2c`. `0x3c` by default,
    ///   which is `SA0` low (§8.1.5).
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property; [`Error::Config`] for an
    /// unknown variant or interface, a zero or oversized dimension, a height
    /// that is not a whole number of eight-row pages, a width wider than the
    /// part's GDDRAM, a chip select out of range, or a bus property that does
    /// not match the interface.
    pub fn new(props: &Props) -> Result<Ssd1306> {
        let mut r = props.reader();
        let variant_name = r.or("variant", String::from("ssd1306"))?;
        let interface_name = r.or("interface", String::from("spi4"))?;
        let mount_name = r.or("mount", String::from("normal"))?;
        let width: u64 = r.or("width", DEFAULT_WIDTH)?;
        let height: u64 = r.or("height", DEFAULT_HEIGHT)?;
        let bus_name = r.optional_str("bus")?.map(String::from);
        let cs: u64 = r.or("cs", 0)?;
        let address: u64 = r.or("address", u64::from(I2C_ADDRESS_SA0_LOW))?;
        r.finish()?;

        let bad = |message: String| Error::Config {
            at: String::from(CLASS_NAME),
            message,
        };
        let variant = Variant::from_name(&variant_name).ok_or_else(|| {
            bad(alloc::format!(
                "`variant` is `{variant_name}`; this model covers {:?}",
                Variant::NAMES
            ))
        })?;
        let interface = Interface::from_name(&interface_name).ok_or_else(|| {
            bad(alloc::format!(
                "`interface` is `{interface_name}`; the strapping pins choose one of {:?} (§8.1, \
                 Table 8-1)",
                Interface::NAMES
            ))
        })?;
        let mount = Mount::from_name(&mount_name).ok_or_else(|| {
            bad(alloc::format!(
                "`mount` is `{mount_name}`; a module's glass is bonded one of {:?} ways round, \
                 which is a board fact rather than a register",
                Mount::NAMES
            ))
        })?;
        if width == 0 || height == 0 {
            return Err(bad(alloc::format!(
                "a panel is {width}x{height}; both dimensions must be at least 1"
            )));
        }
        if height > u64::from(RAM_ROWS) {
            return Err(bad(alloc::format!(
                "`height` is {height}; the row counter walks {RAM_ROWS} rows (§10.1.14), so no \
                 member of this family drives more"
            )));
        }
        if !height.is_multiple_of(u64::from(PAGE_ROWS)) {
            return Err(bad(alloc::format!(
                "`height` is {height}; GDDRAM is eight-row pages (§8.7), so a panel is a whole \
                 number of them"
            )));
        }
        let ram_columns = u64::from(variant.ram_columns());
        let column_offset = u64::from(variant.column_offset());
        if width + column_offset > ram_columns {
            return Err(bad(alloc::format!(
                "`width` is {width} and a {} holds {ram_columns} columns of GDDRAM with the \
                 visible window starting at SEG{column_offset}",
                variant.name()
            )));
        }
        if address > 0x7f {
            return Err(bad(alloc::format!(
                "`address` is {address:#x}; an I²C target address is seven bits"
            )));
        }
        if interface.is_spi() && cs >= MAX_CHIP_SELECTS as u64 {
            return Err(bad(alloc::format!(
                "`cs` is {cs}; an SPI bus routes {MAX_CHIP_SELECTS} chip selects"
            )));
        }

        let pages = (height / u64::from(PAGE_ROWS)) as usize;
        // GDDRAM is the part's, not the panel's: an SH1106 holds 132 columns
        // whatever glass is bonded to it, and a driver that addresses column
        // 130 must find RAM there rather than a fault.
        let ram = alloc::vec![0u8; ram_columns as usize * 8];
        let mut regs = Registers::new();
        regs.column_range = (0, (ram_columns as u8).saturating_sub(1));
        regs.page_range = (0, (pages as u8).saturating_sub(1));
        regs.multiplex = (height as u8).saturating_sub(1);

        let shared = Arc::new(Shared {
            state: Mutex::with_rank(
                LockRank::DEVICE,
                State {
                    ram,
                    generation: 0,
                    regs,
                    pending: None,
                    args: [0; 6],
                    args_len: 0,
                    dc: Level::Low,
                    res: Level::High,
                    i2c: I2cPhase::Idle,
                    unlisted: 0,
                },
            ),
            variant,
            interface,
            width: width as u32,
            height: height as u32,
            ram_columns: ram_columns as u32,
            column_offset: column_offset as u32,
            mount,
            address: address as u8,
        });

        let pins = Arc::new(SlavePins::new(Arc::clone(&shared) as Arc<dyn SpiSlave>));
        let wires = Arc::new(SlaveWires::new(Arc::clone(&shared) as Arc<dyn I2cSlave>));

        // Opening a bus is allocation into this build's own host-object table
        // and nothing outside the machine can see it, which is why it is here;
        // *joining* one is the outward half and happens in `realize`
        // (`CLAUDE.md`, two-phase construction).
        let mut i2c_bus = None;
        if let Some(name) = bus_name {
            match interface {
                Interface::I2c => i2c_bus = Some(i2c_buses::attach(props, &name)?),
                _ => {
                    let bus = spi_buses::attach(props, &name)?;
                    // The SPI bus wants its slaves at construction: a controller
                    // only reaches them when the guest drives a transfer, so
                    // this is where `st7272a` does it too.
                    bus.attach(
                        ChipSelect(cs as u8),
                        Arc::clone(&shared) as Arc<dyn SpiSlave>,
                    )?;
                }
            }
        }

        Ok(Ssd1306 {
            shared,
            pins,
            wires,
            i2c_bus,
            control: Mutex::with_rank(LockRank::WIRE, Vec::new()),
        })
    }

    /// Which part this is.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.shared.variant
    }

    /// Which transport the board wired up.
    #[must_use]
    pub fn interface(&self) -> Interface {
        self.shared.interface
    }

    /// The visible glass, in pixels.
    #[must_use]
    pub fn size(&self) -> (u32, u32) {
        (self.shared.width, self.shared.height)
    }

    /// The seven-bit I²C address this part answers (§8.1.5).
    #[must_use]
    pub fn address(&self) -> Address {
        Address::Seven(self.shared.address)
    }

    /// The command set's registers, as they stand.
    #[must_use]
    pub fn registers(&self) -> Registers {
        self.shared.state.lock().regs
    }

    /// One byte of GDDRAM: `page` of 0..8, `column` of 0..[`Variant::ram_columns`].
    #[must_use]
    pub fn gddram(&self, page: u32, column: u32) -> Option<u8> {
        if column >= self.shared.ram_columns {
            // Not `ram.get`: the backing store is one flat vector, so a column
            // past the end of one page is a byte of the next, and answering
            // with it would be a silently wrong read rather than a miss.
            return None;
        }
        let state = self.shared.state.lock();
        let at = page as usize * self.shared.ram_columns as usize + column as usize;
        state.ram.get(at).copied()
    }

    /// Every byte of GDDRAM, page 0 first.
    #[must_use]
    pub fn contents(&self) -> Vec<u8> {
        self.shared.state.lock().ram.clone()
    }

    /// How many bytes named a command the datasheet does not list.
    #[must_use]
    pub fn unlisted_commands(&self) -> u32 {
        self.shared.state.lock().unlisted
    }

    /// This part as an SPI slave, for a test or an embedder that owns its own
    /// bus.
    #[must_use]
    pub fn spi_slave(&self) -> Arc<dyn SpiSlave> {
        Arc::clone(&self.shared) as Arc<dyn SpiSlave>
    }

    /// This part as an I²C target, on the same terms.
    #[must_use]
    pub fn i2c_slave(&self) -> Arc<dyn I2cSlave> {
        Arc::clone(&self.shared) as Arc<dyn I2cSlave>
    }

    /// This part as a picture.
    #[must_use]
    pub fn panel(&self) -> Arc<dyn Panel> {
        Arc::clone(&self.shared) as Arc<dyn Panel>
    }

    /// The panel's SPI pins, for a controller that drives them directly.
    #[must_use]
    pub fn pins(&self) -> &Arc<SlavePins> {
        &self.pins
    }

    /// The panel's I²C lines, ditto.
    #[must_use]
    pub fn wires(&self) -> &Arc<SlaveWires> {
        &self.wires
    }

    /// Drive the `D/C̅` pin without a wire, for a 4-wire SPI test.
    pub fn set_dc(&self, level: Level) {
        self.shared.state.lock().dc = level;
    }

    /// Feed one byte in, as the transport would.
    ///
    /// `data` is what the `D/C̅` pin or the control byte said. Public because
    /// every transport funnels into it and a test that wants to assert the
    /// *command set* should not have to pick one.
    pub fn feed(&self, data: bool, byte: u8) {
        self.shared.feed(data, byte);
    }
}

// ---------------------------------------------------------------------------
// The command set
// ---------------------------------------------------------------------------

impl Shared {
    /// One byte from any transport.
    fn feed(&self, data: bool, byte: u8) {
        let mut state = self.state.lock();
        if data {
            self.write_ram(&mut state, byte);
        } else {
            self.command(&mut state, byte);
        }
    }

    /// A data byte: into GDDRAM at the pointer, then advance it (§8.7, §10.1.3).
    fn write_ram(&self, state: &mut State, byte: u8) {
        let columns = self.ram_columns;
        let at = state.regs.page as usize * columns as usize + state.regs.column as usize;
        if let Some(slot) = state.ram.get_mut(at) {
            *slot = byte;
        }
        state.generation = state.generation.wrapping_add(1);

        let regs = &mut state.regs;
        let last_column = (columns - 1) as u8;
        match regs.mode {
            AddrMode::Page => {
                // §10.1.1: "the column address pointer is increased by one …
                // if it reaches the end, it is reset to 0 and the page address
                // is *not* changed". The end is the end of RAM, not `21h`'s
                // window: §10.1.3's note makes `21h` horizontal/vertical only.
                regs.column = if regs.column >= last_column {
                    0
                } else {
                    regs.column + 1
                };
            }
            AddrMode::Horizontal => {
                if regs.column >= regs.column_range.1 {
                    regs.column = regs.column_range.0;
                    regs.page = if regs.page >= regs.page_range.1 {
                        regs.page_range.0
                    } else {
                        regs.page + 1
                    };
                } else {
                    regs.column += 1;
                }
            }
            AddrMode::Vertical => {
                if regs.page >= regs.page_range.1 {
                    regs.page = regs.page_range.0;
                    regs.column = if regs.column >= regs.column_range.1 {
                        regs.column_range.0
                    } else {
                        regs.column + 1
                    };
                } else {
                    regs.page += 1;
                }
            }
        }
    }

    /// A command byte, or one of the parameters a previous one is waiting for.
    fn command(&self, state: &mut State, byte: u8) {
        if let Some((cmd, wanted)) = state.pending {
            let at = state.args_len as usize;
            if at < state.args.len() {
                state.args[at] = byte;
            }
            state.args_len += 1;
            if state.args_len >= wanted {
                let args = state.args;
                let len = state.args_len;
                state.pending = None;
                state.args_len = 0;
                self.apply(state, cmd, &args[..(len as usize).min(args.len())]);
            }
            return;
        }
        match self.parameters(byte) {
            0 => self.apply(state, byte, &[]),
            n => {
                state.pending = Some((byte, n));
                state.args_len = 0;
            }
        }
    }

    /// How many parameter bytes a command takes (§10.1, §10.2).
    ///
    /// The single most important function in the file for a stream that must
    /// not desynchronise: a command whose count is wrong swallows a pixel or
    /// leaves a parameter to be read as a command, and the picture after it is
    /// garbage rather than merely wrong.
    fn parameters(&self, cmd: u8) -> u8 {
        let solomon = self.variant.has_addressing_modes();
        match cmd {
            0x81 | 0x8d | 0xa8 | 0xd3 | 0xd5 | 0xd9 | 0xda | 0xdb => 1,
            // SH1106 `ADh`, DC-DC control mode set: one parameter. The Solomon
            // parts have no `ADh` and treat it as unlisted.
            0xad if !solomon => 1,
            0x20 if solomon => 1,
            0x21 | 0x22 if solomon => 2,
            0xa3 if solomon => 2,
            // §10.2.1/§10.2.2: `26h`/`27h` take A..F, six bytes, three of them
            // dummies. Getting this wrong is the classic desynchronisation.
            0x26 | 0x27 if solomon => 6,
            // §10.2.4: `29h`/`2Ah` take A..E, five bytes.
            0x29 | 0x2a if solomon => 5,
            _ => 0,
        }
    }

    /// Carry out a complete command.
    fn apply(&self, state: &mut State, cmd: u8, args: &[u8]) {
        let arg = args.first().copied().unwrap_or(0);
        let last_column = (self.ram_columns - 1) as u8;
        let pages = (self.height / PAGE_ROWS) as u8;
        // Whether the picture could have moved. Set by the arms that change the
        // output mapping; a pointer move or an inert analogue parameter leaves
        // it clear, so a host that draws only on change is not woken by a
        // driver setting its column address a thousand times.
        let mut visible = true;
        match cmd {
            // §10.1.1/§10.1.2: the column address, four bits at a time. Page
            // addressing only, and the SH1106's only way to set a column.
            0x00..=0x0f => {
                state.regs.column = (state.regs.column & 0xf0) | (cmd & 0x0f);
                visible = false;
            }
            0x10..=0x1f => {
                state.regs.column = (state.regs.column & 0x0f) | ((cmd & 0x0f) << 4);
                if state.regs.column > last_column {
                    state.regs.column = last_column;
                }
                visible = false;
            }
            // §10.1.3: memory addressing mode.
            0x20 if self.variant.has_addressing_modes() => {
                state.regs.mode = AddrMode::from_bits(arg);
                visible = false;
            }
            // §10.1.4: the column window, horizontal and vertical modes.
            0x21 if self.variant.has_addressing_modes() => {
                let lo = args.first().copied().unwrap_or(0).min(last_column);
                let hi = args.get(1).copied().unwrap_or(last_column).min(last_column);
                state.regs.column_range = (lo, hi);
                state.regs.column = lo;
                visible = false;
            }
            // §10.1.5: the page window.
            0x22 if self.variant.has_addressing_modes() => {
                let lo = args.first().copied().unwrap_or(0) & 0x07;
                let hi = args.get(1).copied().unwrap_or(pages - 1) & 0x07;
                state.regs.page_range = (lo, hi);
                state.regs.page = lo;
                visible = false;
            }
            // §10.2.1-§10.2.4: scroll setup. Recorded, inert — module docs.
            0x26 | 0x27 | 0x29 | 0x2a if self.variant.has_addressing_modes() => {
                for (slot, byte) in state.regs.scroll.iter_mut().zip(args) {
                    *slot = *byte;
                }
                visible = false;
            }
            0x2e if self.variant.has_addressing_modes() => {
                state.regs.scrolling = false;
                visible = false;
            }
            0x2f if self.variant.has_addressing_modes() => {
                state.regs.scrolling = true;
                visible = false;
            }
            // SH1106 `30h`-`33h`: pump output voltage. Recorded in the DC-DC
            // slot; inert.
            0x30..=0x33 if !self.variant.has_addressing_modes() => {
                state.regs.dc_dc = cmd;
                visible = false;
            }
            // §10.1.14: display start line.
            0x40..=0x7f => state.regs.start_line = cmd & 0x3f,
            // §10.1.7: contrast.
            0x81 => state.regs.contrast = arg,
            // §10.1.11: segment remap.
            0xa0 => state.regs.segment_remap = false,
            0xa1 => state.regs.segment_remap = true,
            // §10.1.12: entire display on.
            0xa4 => state.regs.entire_on = false,
            0xa5 => state.regs.entire_on = true,
            // §10.1.13: normal / inverse.
            0xa6 => state.regs.inverse = false,
            0xa7 => state.regs.inverse = true,
            // §10.2.5: vertical scroll area. Recorded; inert.
            0xa3 if self.variant.has_addressing_modes() => {
                state.regs.scroll_area = (
                    args.first().copied().unwrap_or(0),
                    args.get(1).copied().unwrap_or(64),
                );
                visible = false;
            }
            // §10.1.16: multiplex ratio. The datasheet's valid range is 16 to
            // 64 lines, i.e. a register value of 15 to 63; "invalid entries are
            // ignored" is the note under the table, so a smaller one is.
            0xa8 => {
                if arg >= 15 {
                    state.regs.multiplex = arg & 0x3f;
                }
            }
            // §10.1.9: charge pump. Recorded; inert.
            0x8d => {
                state.regs.charge_pump = arg;
                visible = false;
            }
            // SH1106 `ADh`: DC-DC control. Recorded; inert.
            0xad if !self.variant.has_addressing_modes() => {
                state.regs.dc_dc = arg;
                visible = false;
            }
            // §10.1.18: display off / on.
            0xae => state.regs.display_on = false,
            0xaf => state.regs.display_on = true,
            // §10.1.6: the page for page addressing.
            0xb0..=0xb7 => {
                state.regs.page = cmd & 0x07;
                visible = false;
            }
            // §10.1.17: COM output scan direction.
            0xc0 => state.regs.com_remap = false,
            0xc8 => state.regs.com_remap = true,
            // §10.1.20: display offset.
            0xd3 => state.regs.offset = arg & 0x3f,
            // §10.1.16/§10.1.21/§10.1.22/§10.1.23: analogue and timing.
            // Recorded; inert.
            0xd5 => {
                state.regs.clock = arg;
                visible = false;
            }
            0xd9 => {
                state.regs.precharge = arg;
                visible = false;
            }
            0xda => {
                state.regs.com_pins = arg;
                visible = false;
            }
            0xdb => {
                state.regs.vcomh = arg;
                visible = false;
            }
            // §10.1.24: NOP.
            0xe3 => visible = false,
            _ => {
                state.unlisted = state.unlisted.saturating_add(1);
                visible = false;
            }
        }
        if visible {
            state.generation = state.generation.wrapping_add(1);
        }
    }

    /// What §8.5's `RES̅` pulse does: every register back to its default.
    ///
    /// **GDDRAM survives.** The datasheet's power-on flow (§8.5, and the
    /// application note's initialisation sequence) has software clear the
    /// screen itself precisely because reset does not, and a model that
    /// blanked it would hide the flicker a real module shows.
    fn hardware_reset(&self, state: &mut State) {
        let mut regs = Registers::new();
        regs.column_range = (0, (self.ram_columns - 1) as u8);
        regs.page_range = (0, (self.height / PAGE_ROWS) as u8 - 1);
        regs.multiplex = (self.height - 1) as u8;
        state.regs = regs;
        state.pending = None;
        state.args_len = 0;
        state.i2c = I2cPhase::Idle;
        state.generation = state.generation.wrapping_add(1);
    }
}

// ---------------------------------------------------------------------------
// The picture
// ---------------------------------------------------------------------------

impl Panel for Shared {
    fn geometry(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    fn generation(&self) -> u64 {
        self.state.lock().generation
    }

    fn read_row(&self, y: u32, dst: &mut [[u8; 3]]) {
        if y >= self.height {
            return;
        }
        let state = self.state.lock();
        let regs = &state.regs;
        let width = (self.width as usize).min(dst.len());

        // The module's own wiring, before any register is consulted: a rotated
        // mount is glass glued on upside down, and no command can see it.
        let rotated = self.mount == Mount::Rotated;
        let y = if rotated { self.height - 1 - y } else { y };

        // §10.1.18: display off drives nothing at all, whatever RAM holds.
        // §10.1.16: only `multiplex + 1` COM lines exist, so a 128x32 module
        // driven at 64MUX by mistake still only lights 32 rows.
        let mux = u32::from(regs.multiplex) + 1;
        if !regs.display_on || y >= mux {
            dst[..width].fill([0, 0, 0]);
            return;
        }

        let lit = [regs.contrast, regs.contrast, regs.contrast];
        let dark = [0u8, 0, 0];

        // §10.1.12: "Entire display ON … Output ignores RAM content". There is
        // no RAM content left for `A7h` to invert, so `A5h` wins outright.
        if regs.entire_on {
            dst[..width].fill(lit);
            return;
        }

        // The row counter: `C0h`/`C8h` choose which end it starts at, `40h`
        // where in RAM it starts, `D3h` how far down the picture sits. Both
        // rotations are modulo 64 and in opposite directions — see the module
        // docs.
        let scan = if regs.com_remap { mux - 1 - y } else { y };
        let ram_row = (u32::from(regs.start_line) + scan + RAM_ROWS
            - u32::from(regs.offset) % RAM_ROWS)
            % RAM_ROWS;
        let page = ram_row / PAGE_ROWS;
        let bit = ram_row % PAGE_ROWS;
        let base = page as usize * self.ram_columns as usize;

        for (x, out) in dst[..width].iter_mut().enumerate() {
            // The glass hangs off `SEG(x + offset)`; `A1h` mirrors which RAM
            // column feeds a given SEG, which is why the offset is applied
            // first and the mirror second.
            let x = if rotated {
                self.width - 1 - x as u32
            } else {
                x as u32
            };
            let seg = x + self.column_offset;
            let column = if regs.segment_remap {
                self.ram_columns - 1 - seg
            } else {
                seg
            };
            let byte = state.ram.get(base + column as usize).copied().unwrap_or(0);
            let on = (byte >> bit) & 1 != 0;
            *out = if on != regs.inverse { lit } else { dark };
        }
    }
}

// ---------------------------------------------------------------------------
// The SPI face
// ---------------------------------------------------------------------------

impl SpiSlave for Shared {
    fn format(&self) -> Format {
        // §8.1.3: "D/C̅ … SDIN is shifted into an 8-bit shift register on every
        // rising edge of SCLK in the order of D7, D6, … D0" — Mode 0, MSB
        // first. §8.1.4's 3-wire framing is the same with a ninth bit in front.
        let bits = match self.interface {
            Interface::Spi3 => 9,
            _ => 8,
        };
        Format::new(Mode::Mode0, bits, BitOrder::MsbFirst)
    }

    fn select(&self, _selected: bool) {
        // Nothing commits on CS. Each byte is acted on as it completes (§8.1.3
        // describes no frame longer than a byte), and a multi-byte command's
        // parameters survive a chip select that goes away between them —
        // firmware that raises CS between a `81h` and its contrast byte is
        // doing something odd, but the part does not forget.
    }

    fn transfer(&self, mosi: u32) -> u32 {
        let (data, byte) = match self.interface {
            // §8.1.4: the first of nine bits is D/C̅.
            Interface::Spi3 => (mosi & 0x100 != 0, (mosi & 0xff) as u8),
            // §8.1.3: the pin says which it is, sampled with the byte. Read
            // into a local and released before `feed`, which takes the same
            // lock — this one is not re-entrant.
            _ => {
                let dc = self.state.lock().dc.is_high();
                (dc, (mosi & 0xff) as u8)
            }
        };
        self.feed(data, byte);
        // SDOUT does not exist on the SPI interface of this part; an undriven,
        // pulled-up MISO reads as ones.
        u32::MAX
    }

    fn peek(&self) -> u32 {
        u32::MAX
    }
}

// ---------------------------------------------------------------------------
// The I²C face
// ---------------------------------------------------------------------------

impl I2cSlave for Shared {
    fn address(&self, address: Address, dir: Direction) -> Ack {
        let Address::Seven(seven) = address else {
            // §8.1.5 gives one seven-bit address and no ten-bit form.
            return Ack::Nack;
        };
        if seven != self.address {
            return Ack::Nack;
        }
        if dir == Direction::Read {
            // §8.1.5.2: the I²C interface writes only. There is no read
            // sequence in the datasheet and no way for the part to drive SDA
            // with GDDRAM, so answering one would be invention.
            return Ack::Nack;
        }
        self.state.lock().i2c = I2cPhase::Control;
        Ack::Ack
    }

    fn write(&self, byte: u8) -> Ack {
        let next = {
            let mut state = self.state.lock();
            match state.i2c {
                I2cPhase::Idle => return Ack::Nack,
                // §8.1.5.1: `Co` in bit 7, `D/C̅` in bit 6, the rest zero.
                I2cPhase::Control => {
                    let data = byte & CONTROL_DC != 0;
                    state.i2c = if byte & CONTROL_CO != 0 {
                        I2cPhase::One(data)
                    } else {
                        I2cPhase::Stream(data)
                    };
                    return Ack::Ack;
                }
                I2cPhase::Stream(data) => data,
                I2cPhase::One(data) => {
                    state.i2c = I2cPhase::Control;
                    data
                }
            }
        };
        self.feed(next, byte);
        Ack::Ack
    }

    fn read(&self) -> u8 {
        // Never reached: `address` NACKs a read. An undriven, pulled-up SDA.
        0xff
    }

    fn stop(&self) {
        self.state.lock().i2c = I2cPhase::Idle;
    }

    fn peek(&self) -> u8 {
        0xff
    }
}

// ---------------------------------------------------------------------------
// The control pins
// ---------------------------------------------------------------------------

/// One of the panel's two control inputs.
struct ControlSink {
    shared: Arc<Shared>,
    line: u32,
}

impl fmt::Debug for ControlSink {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ControlSink")
            .field("line", &self.line)
            .finish()
    }
}

impl WireSink for ControlSink {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        let mut state = self.shared.state.lock();
        match self.line {
            pin::DC_LINE => state.dc = level,
            pin::RES_LINE => {
                let was = state.res;
                state.res = level;
                // §8.5: the part initialises while `RES̅` is low. Modelled on
                // the falling edge, which is when a board's power-on circuit
                // and a driver's GPIO pulse both do it.
                if was.is_high() && level.is_low() {
                    self.shared.hardware_reset(&mut state);
                }
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for Ssd1306 {
    fn class(&self) -> &'static DeviceClass {
        &SSD1306_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The one outward action: joining the I²C bus. The SPI bus wants its
        // slaves earlier, for the reason `new` gives.
        if let Some(bus) = &self.i2c_bus {
            bus.attach(Arc::clone(&self.shared) as Arc<dyn I2cSlave>)?;
        }
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            self.shared.hardware_reset(&mut state);
            // GDDRAM survives a board reset for the same reason it survives a
            // `RES̅` pulse: §8.5 does not clear it, and the initialisation
            // sequence every driver ships clears it itself.
        }
        self.pins.reset();
        self.wires.reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock();
        // **The framebuffer is architectural state**, and this is the line that
        // makes it so: a device-owned picture has no RAM device to be saved
        // with, unlike `lcd.scanout`'s, so it goes here or it is lost
        // (`crate::dev::lcd::panel`).
        w.write_bytes(&state.ram)?;
        w.write_u64(state.generation)?;
        let r = &state.regs;
        w.write_u8(r.mode.code())?;
        w.write_u8(r.column)?;
        w.write_u8(r.page)?;
        w.write_u8(r.column_range.0)?;
        w.write_u8(r.column_range.1)?;
        w.write_u8(r.page_range.0)?;
        w.write_u8(r.page_range.1)?;
        w.write_u8(r.start_line)?;
        w.write_u8(r.offset)?;
        w.write_u8(r.contrast)?;
        w.write_bool(r.segment_remap)?;
        w.write_bool(r.com_remap)?;
        w.write_bool(r.inverse)?;
        w.write_bool(r.entire_on)?;
        w.write_bool(r.display_on)?;
        w.write_u8(r.multiplex)?;
        w.write_u8(r.charge_pump)?;
        w.write_u8(r.clock)?;
        w.write_u8(r.precharge)?;
        w.write_u8(r.com_pins)?;
        w.write_u8(r.vcomh)?;
        w.write_u8(r.dc_dc)?;
        w.write_bool(r.scrolling)?;
        for byte in r.scroll {
            w.write_u8(byte)?;
        }
        w.write_u8(r.scroll_area.0)?;
        w.write_u8(r.scroll_area.1)?;
        // The half-finished command, so a snapshot taken between `81h` and its
        // parameter resumes rather than reading the contrast as a command.
        w.write_bool(state.pending.is_some())?;
        let (cmd, wanted) = state.pending.unwrap_or((0, 0));
        w.write_u8(cmd)?;
        w.write_u8(wanted)?;
        for byte in state.args {
            w.write_u8(byte)?;
        }
        w.write_u8(state.args_len)?;
        w.write_u8(state.i2c.code())?;
        w.write_u32(state.unlisted)?;
        drop(state);
        // Both bit-level front ends, always, so the chunk has one layout
        // whatever the board wired up.
        let (rx, tx, count, selected, sck, mosi, loaded) = self.pins.snapshot();
        w.write_u32(rx)?;
        w.write_u32(tx)?;
        w.write_u8(count)?;
        w.write_bool(selected)?;
        w.write_bool(sck)?;
        w.write_bool(mosi)?;
        w.write_bool(loaded)?;
        self.wires.snapshot().write(w)
        // `dc` and `res` are not saved: they are levels *other* devices drive,
        // and each restores its own state and drives them again
        // (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let ram = r.read_bytes()?.to_vec();
        let generation = r.read_u64()?;
        let mut regs = Registers::new();
        regs.mode = AddrMode::from_code(r.read_u8()?);
        regs.column = r.read_u8()?;
        regs.page = r.read_u8()?;
        regs.column_range = (r.read_u8()?, r.read_u8()?);
        regs.page_range = (r.read_u8()?, r.read_u8()?);
        regs.start_line = r.read_u8()?;
        regs.offset = r.read_u8()?;
        regs.contrast = r.read_u8()?;
        regs.segment_remap = r.read_bool()?;
        regs.com_remap = r.read_bool()?;
        regs.inverse = r.read_bool()?;
        regs.entire_on = r.read_bool()?;
        regs.display_on = r.read_bool()?;
        regs.multiplex = r.read_u8()?;
        regs.charge_pump = r.read_u8()?;
        regs.clock = r.read_u8()?;
        regs.precharge = r.read_u8()?;
        regs.com_pins = r.read_u8()?;
        regs.vcomh = r.read_u8()?;
        regs.dc_dc = r.read_u8()?;
        regs.scrolling = r.read_bool()?;
        for slot in &mut regs.scroll {
            *slot = r.read_u8()?;
        }
        regs.scroll_area = (r.read_u8()?, r.read_u8()?);
        // Both fields are always written, so both are always read: a
        // conditional decode would desynchronise the rest of the chunk.
        let has_pending = r.read_bool()?;
        let cmd = r.read_u8()?;
        let wanted = r.read_u8()?;
        let mut args = [0u8; 6];
        for slot in &mut args {
            *slot = r.read_u8()?;
        }
        let args_len = r.read_u8()?;
        let i2c = I2cPhase::from_code(r.read_u8()?);
        let unlisted = r.read_u32()?;
        let pins = (
            r.read_u32()?,
            r.read_u32()?,
            r.read_u8()?,
            r.read_bool()?,
            r.read_bool()?,
            r.read_bool()?,
            r.read_bool()?,
        );
        let bits = SlaveWiresState::read(r)?;

        {
            let mut state = self.shared.state.lock();
            if ram.len() == state.ram.len() {
                state.ram = ram;
            }
            state.generation = generation;
            state.regs = regs;
            state.pending = has_pending.then_some((cmd, wanted));
            state.args = args;
            state.args_len = args_len.min(args.len() as u8);
            state.i2c = i2c;
            state.unlisted = unlisted;
        }
        self.pins.restore(pins);
        self.wires.restore(bits);
        Ok(())
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let control = |line: u32| -> SinkPin {
            let pin = Arc::new(ControlSink {
                shared: Arc::clone(&self.shared),
                line,
            });
            // Kept, because a net refers to its sinks weakly.
            self.control.lock().push(Arc::clone(&pin));
            SinkPin {
                sink: pin as Arc<dyn WireSink>,
                line,
            }
        };
        match port {
            pin::RES => Some(control(pin::RES_LINE)),
            // `D/C̅` exists as a pin only on the 4-wire framing; on the 3-wire
            // one it is tied low and the bit travels in the word, and on I²C
            // there is no such pin at all (§8.1, Table 8-1).
            pin::DC if self.shared.interface == Interface::Spi4 => Some(control(pin::DC_LINE)),
            spi_pin::SCK_NAME if self.shared.interface.is_spi() => Some(SinkPin {
                sink: self.pins.sink(spi_pin::SCK),
                line: spi_pin::SCK,
            }),
            spi_pin::MOSI_NAME if self.shared.interface.is_spi() => Some(SinkPin {
                sink: self.pins.sink(spi_pin::MOSI),
                line: spi_pin::MOSI,
            }),
            spi_pin::CS_NAME if self.shared.interface.is_spi() => Some(SinkPin {
                sink: self.pins.sink(spi_pin::CS),
                line: spi_pin::CS,
            }),
            i2c_line::SCL_NAME if self.shared.interface == Interface::I2c => Some(SinkPin {
                sink: self.wires.sink(i2c_line::SCL, sources),
                line: i2c_line::SCL,
            }),
            i2c_line::SDA_NAME if self.shared.interface == Interface::I2c => Some(SinkPin {
                sink: self.wires.sink(i2c_line::SDA, sources),
                line: i2c_line::SDA,
            }),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            i2c_line::SCL_NAME if self.shared.interface == Interface::I2c => {
                self.wires.connect(i2c_line::SCL, source);
            }
            i2c_line::SDA_NAME if self.shared.interface == Interface::I2c => {
                self.wires.connect(i2c_line::SDA, source);
            }
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!(
                        "an SSD1306 on `{}` drives nothing: its SPI interface has no data output \
                         at all (§8.1.3), and only the I²C lines are open drain",
                        self.shared.interface.name()
                    ),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, _port: &str) {
        if self.shared.interface == Interface::I2c {
            self.wires.announce();
        }
    }
}

impl Instance for Ssd1306 {}

/// The `solomon.ssd1306` device class.
pub static SSD1306_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Solomon Systech SSD1306/SSD1309 and Sino Wealth SH1106 monochrome OLED: page-\
              addressed GDDRAM, SPI and I2C transports",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "ssd1306 (default), ssd1309, or sh1106 — 132 columns and a two-column offset",
        },
        PropertySpec {
            name: "interface",
            kind: ValueKind::Str,
            required: false,
            summary: "spi4 (default, a D/C pin), spi3 (nine-bit words), or i2c (§8.1, Table 8-1)",
        },
        PropertySpec {
            name: "mount",
            kind: ValueKind::Str,
            required: false,
            summary: "normal (default, SEG0/COM0 top left) or rotated (the common 180° module)",
        },
        PropertySpec {
            name: "width",
            kind: ValueKind::Uint,
            required: false,
            summary: "visible pixels across (default 128)",
        },
        PropertySpec {
            name: "height",
            kind: ValueKind::Uint,
            required: false,
            summary: "visible pixels down, a whole number of eight-row pages (default 64)",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the named SPI or I2C bus to attach to, whichever `interface` says",
        },
        PropertySpec {
            name: "cs",
            kind: ValueKind::Uint,
            required: false,
            summary: "which chip select on that SPI bus (default 0)",
        },
        PropertySpec {
            name: "address",
            kind: ValueKind::Uint,
            required: false,
            summary: "the seven-bit I2C address: 0x3c with SA0 low, 0x3d with it high (§8.1.5)",
        },
    ],
    construct: |props| Ok(Box::new(Ssd1306::new(props)?)),
};

/// The picture-owning half of the same class, for
/// [`host::display::panel`](crate::host::display::panel).
pub static SSD1306_PANEL: PanelClass = PanelClass {
    name: CLASS_NAME,
    construct: |props| {
        let device = Arc::new(Ssd1306::new(props)?);
        let panel = device.panel();
        Ok(Built {
            instance: device as Arc<dyn Instance>,
            panel,
        })
    },
};

/// Add [`SSD1306_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&SSD1306_CLASS)
}

/// Bind [`SSD1306_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Ssd1306::new(props)?)))
}

/// What the validator should know about `solomon.ssd1306`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(Variant::NAMES))
        .prop(PropSchema::new("interface", ValueKind::Str).values(Interface::NAMES))
        .prop(PropSchema::new("mount", ValueKind::Str).values(Mount::NAMES))
        .prop(PropSchema::new("width", ValueKind::Uint).range(1, 132))
        .prop(PropSchema::new("height", ValueKind::Uint).range(8, u64::from(RAM_ROWS)))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("cs", ValueKind::Uint).range(0, MAX_CHIP_SELECTS as u64 - 1))
        .prop(PropSchema::new("address", ValueKind::Uint).range(0, 0x7f))
        .port(spi_pin::SCK_NAME, PortDir::In)
        .port(spi_pin::MOSI_NAME, PortDir::In)
        .port(spi_pin::CS_NAME, PortDir::In)
        // Both I²C lines are open drain, so each is an input *and* an output.
        .port(i2c_line::SCL_NAME, PortDir::InOut)
        .port(i2c_line::SDA_NAME, PortDir::InOut)
        .port(pin::DC, PortDir::In)
        .port(pin::RES, PortDir::In)
}
