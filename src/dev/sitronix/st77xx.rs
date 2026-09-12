//! The Sitronix ST7789 / ST7789V and ST7735 / ST7735S: a TFT controller with
//! its **own frame memory**, driven over SPI.
//!
//! 240×320 (ST7789) or 132×162 (ST7735) of 18-bit colour, on every 1.3″ to
//! 2.8″ IPS module worth buying. Like [`crate::dev::solomon::ssd1306`] and
//! unlike [`st7272a`](super::st7272a), **the picture is in the controller**:
//! `CASET` and `RASET` set a window, `RAMWR` streams pixels into it, and
//! nothing in guest memory is the screen. [`crate::dev::lcd::panel`] is the
//! seam that follows from that, and the reasoning is written down there rather
//! than twice.
//!
//! # Sources
//!
//! Sitronix **ST7789V** datasheet v1.0, chapters 7 (command list), 8 (command
//! description) and 9 (the serial interface), cited as §. The ST7735S
//! datasheet v1.1 for the smaller part's memory geometry. Both use the
//! MIPI DCS command numbering, so one model with a [`Variant`] covers them —
//! which is what issue #25 asks for and what a driver assumes anyway.
//!
//! No emulator was consulted (`ROADMAP.md` §1).
//!
//! # Commands are not counted, they are *addressed*
//!
//! This is the architectural difference from an SSD1306 and it is worth saying
//! plainly, because getting it wrong produces a model that desynchronises where
//! silicon cannot. On this part, **`D/CX` decides**: a byte clocked in with
//! `D/CX` low is a command, one with it high is a parameter of whatever command
//! came last (§9.1). There is no parameter count anywhere in the protocol and
//! there is nothing to stay in sync with. A command that arrives halfway
//! through the previous command's parameters simply *replaces* it, and every
//! parameter register the previous one had already latched keeps its new value
//! — which is exactly what happens on the wire.
//!
//! So `PVGAMCTRL` (`E0h`) with fourteen parameters cannot desynchronise the
//! stream however this model treats its arguments, and
//! `a_panel_tuning_command_with_fourteen_parameters_does_not_desynchronise_the_stream`
//! passes for a *structural* reason rather than because a table has the right
//! number in it. The panel-tuning commands `B0h`-`E0h` are therefore accepted
//! and their parameters discarded, with no count to get wrong.
//!
//! # Frame memory and the write window
//!
//! §8.2.19-§8.2.22. `CASET` and `RASET` set inclusive start and end addresses;
//! `RAMWR` resets the pointer to the top left of that window and then advances
//! it — column first, wrapping to the next row at the column end and to the
//! window's first row at the row end. `WRMEMC` (`3Ch`) continues without
//! resetting, which is how a driver splits one window across several
//! transactions.
//!
//! **`MADCTL` (`36h`) turns the window**, not just the picture (§8.2.29). `MV`
//! exchanges the two axes, `MX` and `MY` reverse them. A driver that wants
//! landscape sets `MV` and then addresses `CASET` 0..319 and `RASET` 0..239 —
//! the window is in the *rotated* frame and the glass underneath it is still
//! physically portrait. That is why the model keeps one physical array and maps
//! each written pixel into it, rather than rotating on the way out:
//!
//! ```text
//!   (cx, cy) window  ──MV──►  (a, b)  ──MX,MY──►  (px, py) frame memory
//! ```
//!
//! `RGB` (bit 3) is applied on the way *out* instead, so `RAMRD` hands back
//! what `RAMWR` put in. The datasheet places the swap in the display data path
//! and does not say whether a read-back sees it; making the memory the
//! authority is the choice that keeps a write-then-read round trip honest, and
//! this sentence is where that choice is recorded.
//!
//! # Colour
//!
//! Frame memory is 18 bits — six per channel — whatever `COLMOD` says, and
//! `COLMOD` decides how an incoming stream is expanded into it (§8.2.33, and
//! §6's data-colour-coding tables):
//!
//! | `COLMOD` | Stream | Into 18 bits |
//! | --- | --- | --- |
//! | `55h` | 16-bit 5-6-5, two bytes MSB first | 5→6 and 5→6 by replicating the top bit |
//! | `66h` | 18-bit, three bytes, low two bits ignored | straight through |
//! | `53h` | 12-bit 4-4-4, three bytes per **two** pixels | 4→6 by replicating the top two bits |
//!
//! Expanding those six bits to the eight [`Panel::read_row`] hands out
//! replicates the high bits into the low ones — `0x3f` becomes `0xff` rather
//! than `0xfc`, the same convention
//! [`FbFormat::decode`](crate::dev::lcd::scanout::FbFormat::decode) uses, and
//! the one that keeps white white. `dev/` still names no colour: these are the
//! channel intensities the silicon drives, and what they look like is the
//! host's business.
//!
//! # What is modelled, and what is recorded
//!
//! *Modelled* — visible in a picture: the window and its wrap, `MADCTL`,
//! `COLMOD`, `INVON`/`INVOFF`, `DISPON`/`DISPOFF`, `SLPIN`/`SLPOUT`,
//! `PTLON`/`PTLAR` partial mode, `VSCRDEF`/`VSCSAD` vertical scroll,
//! `IDMON` idle mode's eight-colour reduction, and `SWRESET`.
//!
//! *Recorded and inert*: `GAMSET` and the whole `B0h`-`E0h` panel-tuning block
//! (porch, gate, power, gamma — analogue settings with no digital
//! consequence), `WRDISBV`/`WRCTRLD`/`WRCACE`/`WRCABCMB` (a backlight PWM duty
//! this model has no lamp for), and `TEON`/`TEOFF`.
//!
//! **The tearing-effect pin is not driven**, and that is a decision. `TE` is an
//! output that pulses once per internal refresh, and this device has no
//! refresh: it has no clock domain, no scheduler event and no observable frame
//! rate, for the reason [`crate::dev::lcd::panel`] gives. A port that never
//! changed level would be untested surface pretending to be a feature, so
//! `TEON` sets a flag a test can read and no pin exists. Giving the device a
//! clock is what would change that, and it would be a real addition rather than
//! a line.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

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
const CLASS_NAME: &str = "sitronix.st77xx";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// The command set (§7)
// ---------------------------------------------------------------------------

/// No operation (§8.2.1).
pub const NOP: u8 = 0x00;
/// Software reset (§8.2.2).
pub const SWRESET: u8 = 0x01;
/// Read display identification (§8.2.3).
pub const RDDID: u8 = 0x04;
/// Read display status (§8.2.4).
pub const RDDST: u8 = 0x09;
/// Sleep in (§8.2.12).
pub const SLPIN: u8 = 0x10;
/// Sleep out (§8.2.13).
pub const SLPOUT: u8 = 0x11;
/// Partial display mode on (§8.2.14).
pub const PTLON: u8 = 0x12;
/// Normal display mode on (§8.2.15).
pub const NORON: u8 = 0x13;
/// Display inversion off (§8.2.16).
pub const INVOFF: u8 = 0x20;
/// Display inversion on (§8.2.17).
pub const INVON: u8 = 0x21;
/// Gamma curve select (§8.2.18).
pub const GAMSET: u8 = 0x26;
/// Display off (§8.2.19).
pub const DISPOFF: u8 = 0x28;
/// Display on (§8.2.20).
pub const DISPON: u8 = 0x29;
/// Column address set (§8.2.21).
pub const CASET: u8 = 0x2a;
/// Row address set (§8.2.22).
pub const RASET: u8 = 0x2b;
/// Memory write (§8.2.23).
pub const RAMWR: u8 = 0x2c;
/// Memory read (§8.2.24).
pub const RAMRD: u8 = 0x2e;
/// Partial area (§8.2.25).
pub const PTLAR: u8 = 0x30;
/// Vertical scrolling definition (§8.2.26).
pub const VSCRDEF: u8 = 0x33;
/// Tearing effect line off (§8.2.27).
pub const TEOFF: u8 = 0x34;
/// Tearing effect line on (§8.2.28).
pub const TEON: u8 = 0x35;
/// Memory data access control (§8.2.29).
pub const MADCTL: u8 = 0x36;
/// Vertical scroll start address of RAM (§8.2.30).
pub const VSCSAD: u8 = 0x37;
/// Idle mode off (§8.2.31).
pub const IDMOFF: u8 = 0x38;
/// Idle mode on (§8.2.32).
pub const IDMON: u8 = 0x39;
/// Interface pixel format (§8.2.33).
pub const COLMOD: u8 = 0x3a;
/// Write memory continue (§8.2.34).
pub const WRMEMC: u8 = 0x3c;
/// Read memory continue (§8.2.35).
pub const RDMEMC: u8 = 0x3e;
/// Set tear scanline (§8.2.36).
pub const STE: u8 = 0x44;
/// Get scanline (§8.2.37).
pub const GSCAN: u8 = 0x45;
/// Write display brightness (§8.2.38).
pub const WRDISBV: u8 = 0x51;
/// Write CTRL display (§8.2.40).
pub const WRCTRLD: u8 = 0x53;
/// Write content adaptive brightness control (§8.2.42).
pub const WRCACE: u8 = 0x55;
/// Write CABC minimum brightness (§8.2.44).
pub const WRCABCMB: u8 = 0x5e;

/// `MADCTL` bit 7: row address order (§8.2.29).
pub const MADCTL_MY: u8 = 1 << 7;
/// `MADCTL` bit 6: column address order.
pub const MADCTL_MX: u8 = 1 << 6;
/// `MADCTL` bit 5: row/column exchange.
pub const MADCTL_MV: u8 = 1 << 5;
/// `MADCTL` bit 4: vertical refresh order. Not observable; recorded.
pub const MADCTL_ML: u8 = 1 << 4;
/// `MADCTL` bit 3: RGB/BGR order.
pub const MADCTL_RGB: u8 = 1 << 3;
/// `MADCTL` bit 2: horizontal refresh order. Not observable; recorded.
pub const MADCTL_MH: u8 = 1 << 2;

/// `COLMOD` 12-bit, 4-4-4 (§8.2.33).
pub const COLMOD_12BIT: u8 = 0x53;
/// `COLMOD` 16-bit, 5-6-5.
pub const COLMOD_16BIT: u8 = 0x55;
/// `COLMOD` 18-bit, 6-6-6.
pub const COLMOD_18BIT: u8 = 0x66;

// ---------------------------------------------------------------------------
// Variants
// ---------------------------------------------------------------------------

/// Which member of the family this is.
///
/// The difference the model cares about is the size of frame memory; the
/// command set is the same MIPI DCS numbering on all of them, which is why one
/// file covers the lot.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum Variant {
    /// ST7789 / ST7789V: 240 × 320 × 18 bits (§8.7).
    #[default]
    St7789,
    /// ST7735 / ST7735S: 132 × 162 × 18 bits, of which a module shows
    /// 128 × 160 or 80 × 160 through an offset.
    St7735,
}

impl Variant {
    /// The spelling a machine description writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Variant> {
        match name {
            "st7789" | "st7789v" => Some(Variant::St7789),
            "st7735" | "st7735s" => Some(Variant::St7735),
            _ => None,
        }
    }

    /// The spelling a machine description writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Variant::St7789 => "st7789",
            Variant::St7735 => "st7735",
        }
    }

    /// Every spelling, for the validator.
    pub const NAMES: &'static [&'static str] = &["st7789", "st7789v", "st7735", "st7735s"];

    /// Frame memory, in pixels: `(columns, rows)`.
    #[must_use]
    pub const fn memory(self) -> (u32, u32) {
        match self {
            Variant::St7789 => (240, 320),
            Variant::St7735 => (132, 162),
        }
    }

    /// The default visible glass, in pixels.
    #[must_use]
    pub const fn glass(self) -> (u32, u32) {
        match self {
            Variant::St7789 => (240, 320),
            Variant::St7735 => (128, 160),
        }
    }

    /// The three bytes `RDDID` answers with (§8.2.3).
    ///
    /// The ST7789V's are the datasheet's: manufacturer `85h`, version `85h`,
    /// module/driver `52h`. A board that needs another part's writes the `id`
    /// property.
    #[must_use]
    pub const fn id(self) -> [u8; 3] {
        match self {
            Variant::St7789 => [0x85, 0x85, 0x52],
            Variant::St7735 => [0x7c, 0x89, 0xf0],
        }
    }
}

/// Which SPI framing the board strapped (§9.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Interface {
    /// 4-line: eight-bit words and a `D/CX` pin.
    #[default]
    Spi4,
    /// 3-line (`SPI_3W`): nine-bit words whose first bit is `D/CX`.
    Spi3,
}

impl Interface {
    /// The spelling a machine description writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Interface> {
        match name {
            "spi4" => Some(Interface::Spi4),
            "spi3" => Some(Interface::Spi3),
            _ => None,
        }
    }

    /// The spelling a machine description writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Interface::Spi4 => "spi4",
            Interface::Spi3 => "spi3",
        }
    }

    /// Every spelling, for the validator.
    pub const NAMES: &'static [&'static str] = &["spi4", "spi3"];
}

/// The pin names a machine description wires, beyond SPI's own.
pub mod pin {
    /// The data/command select, `D/CX`. High is a parameter (§9.1). 4-line
    /// framing only.
    pub const DC: &str = "dc";
    /// The reset input, `RESX`. Active low (§8.1).
    pub const RES: &str = "res";

    /// Wire line for [`DC`], numbered past the SPI front end's own.
    pub const DC_LINE: u32 = 16;
    /// Wire line for [`RES`].
    pub const RES_LINE: u32 = 17;
}

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

/// Everything the command set sets.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Registers {
    /// `2Ah`: the inclusive column window.
    pub column: (u16, u16),
    /// `2Bh`: the inclusive row window.
    pub row: (u16, u16),
    /// `36h`, whole.
    pub madctl: u8,
    /// `3Ah`, whole.
    pub colmod: u8,
    /// `21h`/`20h`.
    pub inverted: bool,
    /// `29h`/`28h`.
    pub display_on: bool,
    /// `10h`/`11h`. Sleeping is black (§8.2.12).
    pub sleeping: bool,
    /// `12h`/`13h`.
    pub partial: bool,
    /// `39h`/`38h`: eight-colour idle mode (§8.2.32).
    pub idle: bool,
    /// `30h`: the partial area, inclusive.
    pub partial_area: (u16, u16),
    /// `33h`: top fixed, scrolling, bottom fixed.
    pub scroll: (u16, u16, u16),
    /// `37h`: the first row of frame memory shown in the scrolling area.
    pub scroll_start: u16,
    /// `35h`/`34h`, and `35h`'s mode parameter. Recorded; no pin.
    pub tearing: Option<u8>,
    /// `44h`: the scanline `TE` would pulse on. Recorded.
    pub tear_scanline: u16,
    /// `26h`. Recorded; inert.
    pub gamma_curve: u8,
    /// `51h`. Recorded; inert — there is no backlight here.
    pub brightness: u8,
    /// `53h`. Recorded; inert.
    pub ctrl_display: u8,
}

impl Default for Registers {
    fn default() -> Registers {
        Registers::new()
    }
}

impl Registers {
    /// Every register at the value §8.1's power-on sequence leaves it at.
    #[must_use]
    pub const fn new() -> Registers {
        Registers {
            column: (0, 0),
            row: (0, 0),
            madctl: 0,
            // §8.2.33: the reset value is 18-bit for the parallel interface and
            // 16-bit is what every SPI driver sets first; the datasheet's reset
            // table gives 066h.
            colmod: COLMOD_18BIT,
            inverted: false,
            display_on: false,
            // §8.2.12: the part comes up in sleep-in mode.
            sleeping: true,
            partial: false,
            idle: false,
            partial_area: (0, 0),
            scroll: (0, 0, 0),
            scroll_start: 0,
            tearing: None,
            tear_scanline: 0,
            gamma_curve: 0x01,
            brightness: 0,
            ctrl_display: 0,
        }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The ST77xx family as a device.
#[derive(Debug)]
pub struct St77xx {
    shared: Arc<Shared>,
    pins: Arc<SlavePins>,
    /// The `dc` and `res` pins handed out by [`Device::sink`], kept alive
    /// because a net refers to its sinks weakly (`core::device`).
    control: Mutex<Vec<Arc<ControlSink>>>,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    variant: Variant,
    interface: Interface,
    /// Frame memory, in pixels.
    mem_width: u32,
    mem_height: u32,
    /// The visible glass.
    width: u32,
    height: u32,
    /// Where the glass sits in frame memory.
    col_offset: u32,
    row_offset: u32,
    /// What `RDDID` answers.
    id: [u8; 3],
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// `mem_width × mem_height` pixels, three **six-bit** channels each: frame
    /// memory is 18 bits whatever `COLMOD` says (§8.7).
    ram: Vec<u8>,
    /// How many times the visible picture has been able to change.
    generation: u64,
    /// The command set's registers.
    regs: Registers,
    /// The command the last `D/CX`-low byte named, and how many parameters of
    /// it have arrived. There is no count to reach: see the module docs.
    cmd: u8,
    param: u16,
    /// The write pointer, in window coordinates.
    write: (u16, u16),
    /// The read pointer, ditto.
    read: (u16, u16),
    /// Bytes of a pixel gathered so far, for a format wider than a byte.
    pixel: [u8; 3],
    pixel_len: u8,
    /// What the part is driving on `SDO`, oldest first (§9.1's read frames).
    out: Vec<u8>,
    /// The `D/CX` pin, for the 4-line framing. High is a parameter.
    dc: Level,
    /// The `RESX` pin. Low resets.
    res: Level,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("variant", &self.variant)
            .field("width", &self.width)
            .field("height", &self.height);
        match self.state.try_lock() {
            // Frame memory is the bulk of the state and never what the reader
            // of a failing test wants first.
            Some(state) => s.field("regs", &state.regs).finish(),
            None => s.field("regs", &"<in use>").finish(),
        }
    }
}

impl St77xx {
    /// Validate `props` and build the controller.
    ///
    /// Properties:
    ///
    /// * `variant` — `st7789`/`st7789v` (the default) or `st7735`/`st7735s`.
    /// * `interface` — `spi4` (the default, a `D/CX` pin) or `spi3`.
    /// * `width`, `height` — the visible glass. The variant's own by default.
    /// * `col-offset`, `row-offset` — where the glass sits in frame memory. A
    ///   240×240 ST7789 module is `0, 0`; a 128×160 ST7735 module is `2, 1`;
    ///   an 80×160 one is `26, 1`.
    /// * `id` — the three bytes `RDDID` answers, as one number. The variant's
    ///   own by default.
    /// * `bus`, `cs` — the named SPI bus and chip select.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property; [`Error::Config`] for an
    /// unknown variant or interface, a zero dimension, glass that does not fit
    /// in frame memory at the given offset, an `id` above 24 bits, or a chip
    /// select out of range.
    pub fn new(props: &Props) -> Result<St77xx> {
        let mut r = props.reader();
        let variant_name = r.or("variant", String::from("st7789"))?;
        let interface_name = r.or("interface", String::from("spi4"))?;
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
                "`interface` is `{interface_name}`; §9.1 has {:?}",
                Interface::NAMES
            ))
        })?;
        let (mem_width, mem_height) = variant.memory();
        let (glass_width, glass_height) = variant.glass();
        let width: u64 = r.or("width", u64::from(glass_width))?;
        let height: u64 = r.or("height", u64::from(glass_height))?;
        let col_offset: u64 = r.or("col-offset", 0)?;
        let row_offset: u64 = r.or("row-offset", 0)?;
        let id: u64 = r.or("id", id_word(variant))?;
        let bus_name = r.optional_str("bus")?.map(String::from);
        let cs: u64 = r.or("cs", 0)?;
        r.finish()?;

        if width == 0 || height == 0 {
            return Err(bad(alloc::format!(
                "a panel is {width}x{height}; both dimensions must be at least 1"
            )));
        }
        if width + col_offset > u64::from(mem_width) || height + row_offset > u64::from(mem_height)
        {
            return Err(bad(alloc::format!(
                "a {} holds {mem_width}x{mem_height} pixels of frame memory (§8.7) and this glass \
                 is {width}x{height} at ({col_offset}, {row_offset}), which does not fit",
                variant.name()
            )));
        }
        if id > 0x00ff_ffff {
            return Err(bad(alloc::format!(
                "`id` is {id:#x}; `RDDID` answers with three bytes (§8.2.3)"
            )));
        }
        if cs >= MAX_CHIP_SELECTS as u64 {
            return Err(bad(alloc::format!(
                "`cs` is {cs}; an SPI bus routes {MAX_CHIP_SELECTS} chip selects"
            )));
        }

        let pixels = (mem_width as usize) * (mem_height as usize) * 3;
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(
                LockRank::DEVICE,
                State {
                    ram: alloc::vec![0u8; pixels],
                    generation: 0,
                    regs: reset_registers(mem_width, mem_height),
                    cmd: NOP,
                    param: 0,
                    write: (0, 0),
                    read: (0, 0),
                    pixel: [0; 3],
                    pixel_len: 0,
                    out: Vec::new(),
                    dc: Level::Low,
                    // An unwired `RESX` sits high: a board ties it to its
                    // power-on circuit, and a machine file that did not name
                    // the pin must not hold the part in reset.
                    res: Level::High,
                },
            ),
            variant,
            interface,
            mem_width,
            mem_height,
            width: width as u32,
            height: height as u32,
            col_offset: col_offset as u32,
            row_offset: row_offset as u32,
            id: [(id >> 16) as u8, (id >> 8) as u8, id as u8],
        });

        let pins = Arc::new(SlavePins::new(Arc::clone(&shared) as Arc<dyn SpiSlave>));
        // Opening a bus is allocation into this build's own host-object table;
        // the SPI bus wants its slaves at construction for the reason
        // `st7272a` gives.
        if let Some(name) = bus_name {
            let bus = spi_buses::attach(props, &name)?;
            bus.attach(
                ChipSelect(cs as u8),
                Arc::clone(&shared) as Arc<dyn SpiSlave>,
            )?;
        }

        Ok(St77xx {
            shared,
            pins,
            control: Mutex::with_rank(LockRank::WIRE, Vec::new()),
        })
    }

    /// Which part this is.
    #[must_use]
    pub fn variant(&self) -> Variant {
        self.shared.variant
    }

    /// The visible glass, in pixels.
    #[must_use]
    pub fn size(&self) -> (u32, u32) {
        (self.shared.width, self.shared.height)
    }

    /// Frame memory, in pixels.
    #[must_use]
    pub fn memory_size(&self) -> (u32, u32) {
        (self.shared.mem_width, self.shared.mem_height)
    }

    /// The command set's registers, as they stand.
    #[must_use]
    pub fn registers(&self) -> Registers {
        self.shared.state.lock().regs
    }

    /// The command the last `D/CX`-low byte named, and how many parameters have
    /// arrived since.
    ///
    /// A diagnostic, not a register: the part has no count to reach (see the
    /// module docs), and this is here so a test can say that fourteen
    /// parameters were accepted rather than silently swallowed as commands.
    #[must_use]
    pub fn in_progress(&self) -> (u8, u16) {
        let state = self.shared.state.lock();
        (state.cmd, state.param)
    }

    /// One pixel of frame memory, as three **six-bit** channels.
    ///
    /// Frame-memory coordinates, not glass ones, and not window ones: this is
    /// what `MADCTL` maps into.
    #[must_use]
    pub fn memory_pixel(&self, x: u32, y: u32) -> Option<[u8; 3]> {
        if x >= self.shared.mem_width || y >= self.shared.mem_height {
            return None;
        }
        let state = self.shared.state.lock();
        let at = ((y as usize) * self.shared.mem_width as usize + x as usize) * 3;
        Some([state.ram[at], state.ram[at + 1], state.ram[at + 2]])
    }

    /// This part as an SPI slave, for a test or an embedder that owns its bus.
    #[must_use]
    pub fn spi_slave(&self) -> Arc<dyn SpiSlave> {
        Arc::clone(&self.shared) as Arc<dyn SpiSlave>
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

    /// Drive the `D/CX` pin without a wire, for a 4-line test.
    pub fn set_dc(&self, level: Level) {
        self.shared.state.lock().dc = level;
    }

    /// Feed one byte in, as the transport would. `param` is what `D/CX` said.
    pub fn feed(&self, param: bool, byte: u8) {
        self.shared.feed(param, byte);
    }

    /// The next byte the part would drive on `SDO`, consuming it.
    #[must_use]
    pub fn read_byte(&self) -> u8 {
        self.shared.next_out()
    }
}

/// The variant's `RDDID` bytes as one number, for the property default.
const fn id_word(variant: Variant) -> u64 {
    let id = variant.id();
    ((id[0] as u64) << 16) | ((id[1] as u64) << 8) | (id[2] as u64)
}

/// The register file §8.1's power-on sequence leaves behind, with the windows
/// covering the whole of frame memory (§8.2.21: `XE` resets to the last column).
fn reset_registers(mem_width: u32, mem_height: u32) -> Registers {
    let mut regs = Registers::new();
    regs.column = (0, (mem_width - 1) as u16);
    regs.row = (0, (mem_height - 1) as u16);
    regs.partial_area = (0, (mem_height - 1) as u16);
    regs.scroll = (0, mem_height as u16, 0);
    regs
}

// ---------------------------------------------------------------------------
// The command interpreter
// ---------------------------------------------------------------------------

impl Shared {
    /// One byte from the transport. `param` is `D/CX`.
    fn feed(&self, param: bool, byte: u8) {
        let mut state = self.state.lock();
        if param {
            self.parameter(&mut state, byte);
        } else {
            self.command(&mut state, byte);
        }
    }

    /// The next byte to drive on `SDO`, consuming it.
    ///
    /// The queue is **refilled after the pop, never on demand from
    /// [`SpiSlave::peek`]**, and that is load-bearing rather than tidy. The two
    /// SPI link models ask different questions: [`SpiBus::transfer`] takes what
    /// [`SpiSlave::transfer`] returns, while [`SlavePins`] preloads its shift
    /// register from `peek` after every word, because a CPHA-0 controller
    /// samples the first bit on the *leading* edge. `peek` must have no side
    /// effect (`SpiSlave`'s own rule), so the only way both can see the same
    /// byte is for the queue never to be empty while a read is in flight.
    fn next_out(&self) -> u8 {
        let mut state = self.state.lock();
        let byte = if state.out.is_empty() {
            // Nothing to say: an undriven, pulled-up line.
            0xff
        } else {
            state.out.remove(0)
        };
        if state.out.is_empty() {
            self.refill(&mut state);
        }
        byte
    }

    /// A byte arriving with `D/CX` low: a new command (§9.1).
    ///
    /// **It replaces whatever was in progress.** There is no count to finish,
    /// and a parameter register the previous command had already latched keeps
    /// its new value — which is what the silicon does.
    fn command(&self, state: &mut State, cmd: u8) {
        state.cmd = cmd;
        state.param = 0;
        state.pixel_len = 0;
        state.out.clear();
        let mut visible = true;
        match cmd {
            NOP => visible = false,
            SWRESET => self.software_reset(state),
            RDDID => {
                // §9.1: a dummy clock first, then the three bytes.
                state.out.push(0x00);
                state.out.extend_from_slice(&self.id);
                visible = false;
            }
            RDDST => {
                state.out.push(0x00);
                let st = self.status(state);
                state.out.extend_from_slice(&st);
                visible = false;
            }
            GSCAN => {
                // §8.2.37: the scanline the part is currently driving. There is
                // no refresh here, so it is always zero, and that is a fact
                // this model states rather than a number it invents.
                state.out.push(0x00);
                state.out.push(0x00);
                state.out.push(0x00);
                visible = false;
            }
            SLPIN => state.regs.sleeping = true,
            SLPOUT => state.regs.sleeping = false,
            PTLON => state.regs.partial = true,
            NORON => state.regs.partial = false,
            INVOFF => state.regs.inverted = false,
            INVON => state.regs.inverted = true,
            DISPOFF => state.regs.display_on = false,
            DISPON => state.regs.display_on = true,
            IDMOFF => state.regs.idle = false,
            IDMON => state.regs.idle = true,
            TEOFF => {
                state.regs.tearing = None;
                visible = false;
            }
            RAMWR => {
                // §8.2.23: the pointer goes back to the top left of the window.
                state.write = (state.regs.column.0, state.regs.row.0);
                visible = false;
            }
            WRMEMC => visible = false, // §8.2.34: continue where it left off
            RAMRD => {
                state.read = (state.regs.column.0, state.regs.row.0);
                state.out.push(0x00); // the dummy clock of §9.1
                visible = false;
            }
            RDMEMC => {
                state.out.push(0x00);
                visible = false;
            }
            // Everything else takes parameters and does its work there.
            _ => visible = false,
        }
        if visible {
            state.generation = state.generation.wrapping_add(1);
        }
    }

    /// A byte arriving with `D/CX` high: a parameter of `state.cmd`.
    fn parameter(&self, state: &mut State, byte: u8) {
        let index = state.param;
        state.param = state.param.saturating_add(1);
        match state.cmd {
            RAMWR | WRMEMC => self.write_pixel_byte(state, byte),
            // §8.2.21/§8.2.22: XS[15:8] XS[7:0] XE[15:8] XE[7:0], latched one
            // byte at a time into a sixteen-bit register.
            //
            // **Nothing is clamped and nothing is reordered.** §8.2.21 makes
            // `XS ≤ XE ≤ 00EFh` the host's obligation, not the part's, and the
            // register is sixteen bits wide whatever `MADCTL` says the axis
            // means. A window outside frame memory is the guest's bug, and it
            // shows up as pixels that land nowhere — which is what
            // [`Shared::memory_index`] does with them — rather than as a clamp
            // this model invented and a picture that quietly looks fine.
            CASET | RASET => {
                let window = if state.cmd == CASET {
                    &mut state.regs.column
                } else {
                    &mut state.regs.row
                };
                match index {
                    0 => window.0 = (window.0 & 0x00ff) | (u16::from(byte) << 8),
                    1 => window.0 = (window.0 & 0xff00) | u16::from(byte),
                    2 => window.1 = (window.1 & 0x00ff) | (u16::from(byte) << 8),
                    3 => window.1 = (window.1 & 0xff00) | u16::from(byte),
                    _ => {}
                }
            }
            MADCTL if index == 0 => {
                state.regs.madctl = byte;
                state.generation = state.generation.wrapping_add(1);
            }
            COLMOD if index == 0 => state.regs.colmod = byte,
            // §8.2.25: PSL[15:0], PEL[15:0].
            PTLAR => {
                let area = &mut state.regs.partial_area;
                match index {
                    0 => area.0 = (area.0 & 0x00ff) | (u16::from(byte) << 8),
                    1 => area.0 = (area.0 & 0xff00) | u16::from(byte),
                    2 => area.1 = (area.1 & 0x00ff) | (u16::from(byte) << 8),
                    3 => area.1 = (area.1 & 0xff00) | u16::from(byte),
                    _ => {}
                }
                state.generation = state.generation.wrapping_add(1);
            }
            // §8.2.26: TFA[15:0], VSA[15:0], BFA[15:0].
            VSCRDEF => {
                let s = &mut state.regs.scroll;
                match index {
                    0 => s.0 = (s.0 & 0x00ff) | (u16::from(byte) << 8),
                    1 => s.0 = (s.0 & 0xff00) | u16::from(byte),
                    2 => s.1 = (s.1 & 0x00ff) | (u16::from(byte) << 8),
                    3 => s.1 = (s.1 & 0xff00) | u16::from(byte),
                    4 => s.2 = (s.2 & 0x00ff) | (u16::from(byte) << 8),
                    5 => s.2 = (s.2 & 0xff00) | u16::from(byte),
                    _ => {}
                }
                state.generation = state.generation.wrapping_add(1);
            }
            // §8.2.30: VSP[15:0].
            VSCSAD => {
                let s = &mut state.regs.scroll_start;
                match index {
                    0 => *s = (*s & 0x00ff) | (u16::from(byte) << 8),
                    1 => *s = (*s & 0xff00) | u16::from(byte),
                    _ => {}
                }
                state.generation = state.generation.wrapping_add(1);
            }
            TEON if index == 0 => state.regs.tearing = Some(byte),
            STE => {
                let n = &mut state.regs.tear_scanline;
                match index {
                    0 => *n = (*n & 0x00ff) | (u16::from(byte) << 8),
                    1 => *n = (*n & 0xff00) | u16::from(byte),
                    _ => {}
                }
            }
            GAMSET if index == 0 => state.regs.gamma_curve = byte,
            WRDISBV if index == 0 => state.regs.brightness = byte,
            WRCTRLD if index == 0 => state.regs.ctrl_display = byte,
            // §8.2.42/§8.2.44 and the whole B0h-E0h panel-tuning block: accepted
            // and discarded. There is no count to get wrong — see the module
            // docs — so a fourteen-parameter gamma table costs one line.
            _ => {}
        }
    }

    /// `01h`: everything back to its power-on value except frame memory.
    ///
    /// §8.2.2 says a software reset does not change frame memory, and the
    /// datasheet's own initialisation flow clears the screen afterwards
    /// precisely because of that.
    fn software_reset(&self, state: &mut State) {
        state.regs = reset_registers(self.mem_width, self.mem_height);
        state.write = (0, 0);
        state.read = (0, 0);
        state.pixel_len = 0;
        state.out.clear();
        state.generation = state.generation.wrapping_add(1);
    }

    /// The four bytes `09h` answers with (§8.2.4), as far as this model has
    /// facts for.
    fn status(&self, state: &State) -> [u8; 4] {
        let regs = &state.regs;
        let mut b1 = 0u8;
        // D[31:25] are the MADCTL bits, in the same order.
        b1 |= regs.madctl & (MADCTL_MY | MADCTL_MX | MADCTL_MV | MADCTL_ML | MADCTL_RGB);
        let mut b2 = 0u8;
        if regs.idle {
            b2 |= 1 << 6;
        }
        if regs.partial {
            b2 |= 1 << 5;
        }
        if !regs.sleeping {
            b2 |= 1 << 4;
        }
        let mut b3 = 0u8;
        if regs.inverted {
            b3 |= 1 << 5;
        }
        if regs.display_on {
            b3 |= 1 << 2;
        }
        [b1, b2, b3, 0]
    }

    /// Refill [`State::out`] from the read window, if a read command is running.
    fn refill(&self, state: &mut State) {
        if state.cmd != RAMRD && state.cmd != RDMEMC {
            return;
        }
        let (cx, cy) = state.read;
        let Some(at) = self.memory_index(state.regs.madctl, cx, cy) else {
            return;
        };
        // §8.2.24: "the read data is 18 bits", whatever `COLMOD` says about
        // writes. Six-bit channels are returned left-aligned in a byte, which
        // is how the parallel pins present them.
        let pixel = [state.ram[at], state.ram[at + 1], state.ram[at + 2]];
        state.out.push(pixel[0] << 2);
        state.out.push(pixel[1] << 2);
        state.out.push(pixel[2] << 2);
        state.read = self.advance(&state.regs, (cx, cy));
    }

    /// One byte of a pixel stream.
    fn write_pixel_byte(&self, state: &mut State, byte: u8) {
        let want = match state.regs.colmod & 0x77 {
            COLMOD_16BIT => 2,
            COLMOD_12BIT => 3,
            // 18-bit, and anything the datasheet does not define: three bytes.
            _ => 3,
        };
        state.pixel[state.pixel_len as usize] = byte;
        state.pixel_len += 1;
        if state.pixel_len < want {
            return;
        }
        let bytes = state.pixel;
        state.pixel_len = 0;
        match state.regs.colmod & 0x77 {
            COLMOD_16BIT => {
                // §6: RRRRRGGG GGGBBBBB, MSB first. 5→6 replicates the top bit.
                let v = (u16::from(bytes[0]) << 8) | u16::from(bytes[1]);
                let r = ((v >> 11) & 0x1f) as u8;
                let g = ((v >> 5) & 0x3f) as u8;
                let b = (v & 0x1f) as u8;
                self.store(state, [(r << 1) | (r >> 4), g, (b << 1) | (b >> 4)]);
            }
            COLMOD_12BIT => {
                // §6: three bytes carry *two* pixels — RRRRGGGG BBBBRRRR
                // GGGGBBBB. 4→6 replicates the top two bits.
                let widen = |v: u8| -> u8 { (v << 2) | (v >> 2) };
                let first = [
                    widen(bytes[0] >> 4),
                    widen(bytes[0] & 0x0f),
                    widen(bytes[1] >> 4),
                ];
                let second = [
                    widen(bytes[1] & 0x0f),
                    widen(bytes[2] >> 4),
                    widen(bytes[2] & 0x0f),
                ];
                self.store(state, first);
                self.store(state, second);
            }
            _ => {
                // §6: three bytes, the low two bits of each ignored.
                self.store(state, [bytes[0] >> 2, bytes[1] >> 2, bytes[2] >> 2]);
            }
        }
    }

    /// Put one pixel at the write pointer and advance it.
    fn store(&self, state: &mut State, pixel: [u8; 3]) {
        let (cx, cy) = state.write;
        if let Some(at) = self.memory_index(state.regs.madctl, cx, cy) {
            state.ram[at] = pixel[0];
            state.ram[at + 1] = pixel[1];
            state.ram[at + 2] = pixel[2];
            state.generation = state.generation.wrapping_add(1);
        }
        state.write = self.advance(&state.regs, (cx, cy));
    }

    /// The next window coordinate after `(cx, cy)` (§8.2.23).
    fn advance(&self, regs: &Registers, at: (u16, u16)) -> (u16, u16) {
        let (cx, cy) = at;
        if cx >= regs.column.1 {
            let cy = if cy >= regs.row.1 { regs.row.0 } else { cy + 1 };
            (regs.column.0, cy)
        } else {
            (cx + 1, cy)
        }
    }

    /// Where window coordinate `(cx, cy)` lands in frame memory, per `MADCTL`
    /// (§8.2.29), as a byte index.
    fn memory_index(&self, madctl: u8, cx: u16, cy: u16) -> Option<usize> {
        // `MV` exchanges the axes *before* the mirrors, which is what makes a
        // landscape driver's `CASET 0..319` address physical rows.
        let (a, b) = if madctl & MADCTL_MV != 0 {
            (u32::from(cy), u32::from(cx))
        } else {
            (u32::from(cx), u32::from(cy))
        };
        if a >= self.mem_width || b >= self.mem_height {
            return None;
        }
        let px = if madctl & MADCTL_MX != 0 {
            self.mem_width - 1 - a
        } else {
            a
        };
        let py = if madctl & MADCTL_MY != 0 {
            self.mem_height - 1 - b
        } else {
            b
        };
        Some(((py as usize) * self.mem_width as usize + px as usize) * 3)
    }
}

// ---------------------------------------------------------------------------
// The picture
// ---------------------------------------------------------------------------

/// Six bits to eight, replicating the high bits into the low ones so that
/// `0x3f` becomes `0xff` rather than `0xfc` — the convention
/// [`FbFormat::decode`](crate::dev::lcd::scanout::FbFormat::decode) uses, and
/// the one that keeps white white.
const fn widen6(v: u8) -> u8 {
    (v << 2) | (v >> 4)
}

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
        let dark = [0u8, 0, 0];

        // §8.2.12 and §8.2.19: sleeping and display-off are both black,
        // whatever frame memory holds.
        if regs.sleeping || !regs.display_on {
            dst[..width].fill(dark);
            return;
        }

        let line = self.row_offset + y;
        // §8.2.14: partial mode drives only the partial area; the rest of the
        // glass is not driven at all.
        if regs.partial {
            let (lo, hi) = regs.partial_area;
            if line < u32::from(lo) || line > u32::from(hi) {
                dst[..width].fill(dark);
                return;
            }
        }
        let source = if regs.partial {
            line
        } else {
            self.scrolled(regs, line)
        };

        for (x, out) in dst[..width].iter_mut().enumerate() {
            let col = self.col_offset + x as u32;
            let at = ((source as usize) * self.mem_width as usize + col as usize) * 3;
            let mut pixel = [state.ram[at], state.ram[at + 1], state.ram[at + 2]];
            // §8.2.32: idle mode drives eight colours — one bit per channel.
            if regs.idle {
                for c in &mut pixel {
                    *c = if *c >= 0x20 { 0x3f } else { 0 };
                }
            }
            // §8.2.17: inversion is a complement of the driven value.
            if regs.inverted {
                for c in &mut pixel {
                    *c = 0x3f - *c;
                }
            }
            // §8.2.29 bit 3: the panel's colour filter order. Applied here
            // rather than on the way in, so `RAMRD` returns what `RAMWR` wrote
            // — see the module docs.
            let (r, b) = if regs.madctl & MADCTL_RGB != 0 {
                (pixel[2], pixel[0])
            } else {
                (pixel[0], pixel[2])
            };
            *out = [widen6(r), widen6(pixel[1]), widen6(b)];
        }
    }
}

impl Shared {
    /// Which frame-memory row a display line shows, through `33h`/`37h`
    /// (§8.2.26, §8.2.30).
    fn scrolled(&self, regs: &Registers, line: u32) -> u32 {
        let (tfa, vsa, bfa) = (
            u32::from(regs.scroll.0),
            u32::from(regs.scroll.1),
            u32::from(regs.scroll.2),
        );
        // §8.2.26: "TFA + VSA + BFA must equal the number of lines". A
        // definition that does not is ignored rather than guessed at.
        if vsa == 0 || tfa + vsa + bfa != self.mem_height {
            return line;
        }
        if line < tfa || line >= tfa + vsa {
            return line;
        }
        let start = u32::from(regs.scroll_start);
        // §8.2.30: VSP is "the line in the frame memory written to the display
        // as the first line of the vertical scroll area", so the first scrolled
        // line shows row `VSP` and the rest follow, wrapping within the area.
        // A `VSP` outside the area — which §8.2.30 calls undefined — is folded
        // into it rather than left to index out of the scroll region.
        let rel = if start >= tfa {
            (start - tfa) % vsa
        } else {
            (vsa - (tfa - start) % vsa) % vsa
        };
        tfa + ((line - tfa + rel) % vsa)
    }
}

// ---------------------------------------------------------------------------
// The SPI face
// ---------------------------------------------------------------------------

impl SpiSlave for Shared {
    fn format(&self) -> Format {
        // §9.1: MSB first, sampled on the rising edge of SCL with SCL idling
        // low — mode 0. The 3-line framing is the same with `D/CX` in front.
        let bits = match self.interface {
            Interface::Spi3 => 9,
            Interface::Spi4 => 8,
        };
        Format::new(Mode::Mode0, bits, BitOrder::MsbFirst)
    }

    fn select(&self, selected: bool) {
        if !selected {
            // §9.1: a read frame ends with CSX, and a part-consumed answer does
            // not carry into the next transaction. The command itself survives:
            // `WRMEMC` after a chip select exists precisely so a driver can
            // split one window across transactions.
            let mut state = self.state.lock();
            state.out.clear();
        }
    }

    fn transfer(&self, mosi: u32) -> u32 {
        let (param, byte) = match self.interface {
            Interface::Spi3 => (mosi & 0x100 != 0, (mosi & 0xff) as u8),
            // Read into a local and released before `feed`, which takes the
            // same lock — it is not re-entrant.
            Interface::Spi4 => {
                let dc = self.state.lock().dc.is_high();
                (dc, (mosi & 0xff) as u8)
            }
        };
        // Full duplex: the answer is what was already in the shift register
        // when the transfer began, so it is taken *before* this byte lands.
        let out = self.next_out();
        self.feed(param, byte);
        u32::from(out)
    }

    fn peek(&self) -> u32 {
        let state = self.state.lock();
        u32::from(state.out.first().copied().unwrap_or(0xff))
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
                // §8.1: `RESX` low runs the same initialisation a `SWRESET`
                // does. Modelled on the falling edge, which is when a board's
                // power-on circuit and a driver's GPIO pulse both do it.
                if was.is_high() && level.is_low() {
                    self.shared.software_reset(&mut state);
                }
            }
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for St77xx {
    fn class(&self) -> &'static DeviceClass {
        &ST77XX_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            self.shared.software_reset(&mut state);
            state.cmd = NOP;
            state.param = 0;
            // Frame memory survives, for the reason §8.2.2 gives.
        }
        self.pins.reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock();
        // **The framebuffer is architectural state**: a device-owned picture
        // has no RAM device to be saved with, so it goes here or it is lost
        // (`crate::dev::lcd::panel`).
        w.write_bytes(&state.ram)?;
        w.write_u64(state.generation)?;
        let r = &state.regs;
        w.write_u16(r.column.0)?;
        w.write_u16(r.column.1)?;
        w.write_u16(r.row.0)?;
        w.write_u16(r.row.1)?;
        w.write_u8(r.madctl)?;
        w.write_u8(r.colmod)?;
        w.write_bool(r.inverted)?;
        w.write_bool(r.display_on)?;
        w.write_bool(r.sleeping)?;
        w.write_bool(r.partial)?;
        w.write_bool(r.idle)?;
        w.write_u16(r.partial_area.0)?;
        w.write_u16(r.partial_area.1)?;
        w.write_u16(r.scroll.0)?;
        w.write_u16(r.scroll.1)?;
        w.write_u16(r.scroll.2)?;
        w.write_u16(r.scroll_start)?;
        w.write_bool(r.tearing.is_some())?;
        w.write_u8(r.tearing.unwrap_or(0))?;
        w.write_u16(r.tear_scanline)?;
        w.write_u8(r.gamma_curve)?;
        w.write_u8(r.brightness)?;
        w.write_u8(r.ctrl_display)?;
        // The command in progress and the half-gathered pixel: a snapshot taken
        // between the two bytes of a 5-6-5 pixel has to resume, not restart.
        w.write_u8(state.cmd)?;
        w.write_u16(state.param)?;
        w.write_u16(state.write.0)?;
        w.write_u16(state.write.1)?;
        w.write_u16(state.read.0)?;
        w.write_u16(state.read.1)?;
        for byte in state.pixel {
            w.write_u8(byte)?;
        }
        w.write_u8(state.pixel_len)?;
        w.write_bytes(&state.out)?;
        drop(state);
        let (rx, tx, count, selected, sck, mosi, loaded) = self.pins.snapshot();
        w.write_u32(rx)?;
        w.write_u32(tx)?;
        w.write_u8(count)?;
        w.write_bool(selected)?;
        w.write_bool(sck)?;
        w.write_bool(mosi)?;
        w.write_bool(loaded)
        // `dc` and `res` are not saved: they are levels *other* devices drive,
        // and each restores its own state and drives them again
        // (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let ram = r.read_bytes()?.to_vec();
        let generation = r.read_u64()?;
        let mut regs = Registers::new();
        regs.column = (r.read_u16()?, r.read_u16()?);
        regs.row = (r.read_u16()?, r.read_u16()?);
        regs.madctl = r.read_u8()?;
        regs.colmod = r.read_u8()?;
        regs.inverted = r.read_bool()?;
        regs.display_on = r.read_bool()?;
        regs.sleeping = r.read_bool()?;
        regs.partial = r.read_bool()?;
        regs.idle = r.read_bool()?;
        regs.partial_area = (r.read_u16()?, r.read_u16()?);
        regs.scroll = (r.read_u16()?, r.read_u16()?, r.read_u16()?);
        regs.scroll_start = r.read_u16()?;
        // Both fields are always written, so both are always read: a
        // conditional decode would desynchronise the rest of the chunk.
        let has_te = r.read_bool()?;
        let te = r.read_u8()?;
        regs.tearing = has_te.then_some(te);
        regs.tear_scanline = r.read_u16()?;
        regs.gamma_curve = r.read_u8()?;
        regs.brightness = r.read_u8()?;
        regs.ctrl_display = r.read_u8()?;
        let cmd = r.read_u8()?;
        let param = r.read_u16()?;
        let write = (r.read_u16()?, r.read_u16()?);
        let read = (r.read_u16()?, r.read_u16()?);
        let mut pixel = [0u8; 3];
        for slot in &mut pixel {
            *slot = r.read_u8()?;
        }
        let pixel_len = r.read_u8()?;
        let out = r.read_bytes()?.to_vec();
        let pins = (
            r.read_u32()?,
            r.read_u32()?,
            r.read_u8()?,
            r.read_bool()?,
            r.read_bool()?,
            r.read_bool()?,
            r.read_bool()?,
        );

        {
            let mut state = self.shared.state.lock();
            if ram.len() == state.ram.len() {
                state.ram = ram;
            }
            state.generation = generation;
            state.regs = regs;
            state.cmd = cmd;
            state.param = param;
            state.write = write;
            state.read = read;
            state.pixel = pixel;
            state.pixel_len = pixel_len.min(3);
            state.out = out;
        }
        self.pins.restore(pins);
        Ok(())
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
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
            spi_pin::SCK_NAME => Some(SinkPin {
                sink: self.pins.sink(spi_pin::SCK),
                line: spi_pin::SCK,
            }),
            spi_pin::MOSI_NAME => Some(SinkPin {
                sink: self.pins.sink(spi_pin::MOSI),
                line: spi_pin::MOSI,
            }),
            spi_pin::CS_NAME => Some(SinkPin {
                sink: self.pins.sink(spi_pin::CS),
                line: spi_pin::CS,
            }),
            pin::RES => Some(control(pin::RES_LINE)),
            // `D/CX` is a pin only on the 4-line framing; on the 3-line one the
            // bit travels in the word and the pin is tied low (§9.1).
            pin::DC if self.shared.interface == Interface::Spi4 => Some(control(pin::DC_LINE)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != spi_pin::MISO_NAME {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!(
                    "an ST77xx drives only `{}` — on the real part it is `SDA` shared with `{}` on \
                     a 3-line bus, split here because a wire has fixed drivers",
                    spi_pin::MISO_NAME,
                    spi_pin::MOSI_NAME
                ),
            });
        }
        self.pins.connect_miso(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == spi_pin::MISO_NAME {
            self.pins.publish_miso();
        }
    }
}

impl Instance for St77xx {}

/// The `sitronix.st77xx` device class.
pub static ST77XX_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Sitronix ST7789/ST7735 TFT controller: its own frame memory, CASET/RASET windowed \
              writes, MADCTL, COLMOD",
    properties: &[
        PropertySpec {
            name: "variant",
            kind: ValueKind::Str,
            required: false,
            summary: "st7789 (default, 240x320 of frame memory) or st7735 (132x162)",
        },
        PropertySpec {
            name: "interface",
            kind: ValueKind::Str,
            required: false,
            summary: "spi4 (default, a D/CX pin) or spi3 (nine-bit words, §9.1)",
        },
        PropertySpec {
            name: "width",
            kind: ValueKind::Uint,
            required: false,
            summary: "visible pixels across (the variant's glass by default)",
        },
        PropertySpec {
            name: "height",
            kind: ValueKind::Uint,
            required: false,
            summary: "visible pixels down (the variant's glass by default)",
        },
        PropertySpec {
            name: "col-offset",
            kind: ValueKind::Uint,
            required: false,
            summary: "where the glass starts in frame memory: 2 on a 128x160 ST7735 module",
        },
        PropertySpec {
            name: "row-offset",
            kind: ValueKind::Uint,
            required: false,
            summary: "ditto, down: 1 on a 128x160 ST7735 module",
        },
        PropertySpec {
            name: "id",
            kind: ValueKind::Uint,
            required: false,
            summary: "the three bytes RDDID answers with (§8.2.3; default the variant's)",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the named SPI bus to attach to",
        },
        PropertySpec {
            name: "cs",
            kind: ValueKind::Uint,
            required: false,
            summary: "which chip select on that bus (default 0)",
        },
    ],
    construct: |props| Ok(Box::new(St77xx::new(props)?)),
};

/// The picture-owning half of the same class, for
/// [`host::display::panel`](crate::host::display::panel).
pub static ST77XX_PANEL: PanelClass = PanelClass {
    name: CLASS_NAME,
    construct: |props| {
        let device = Arc::new(St77xx::new(props)?);
        let panel = device.panel();
        Ok(Built {
            instance: device as Arc<dyn Instance>,
            panel,
        })
    },
};

/// Add [`ST77XX_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&ST77XX_CLASS)
}

/// Bind [`ST77XX_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(St77xx::new(props)?)))
}

/// What the validator should know about `sitronix.st77xx`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("variant", ValueKind::Str).values(Variant::NAMES))
        .prop(PropSchema::new("interface", ValueKind::Str).values(Interface::NAMES))
        .prop(PropSchema::new("width", ValueKind::Uint).range(1, 320))
        .prop(PropSchema::new("height", ValueKind::Uint).range(1, 320))
        .prop(PropSchema::new("col-offset", ValueKind::Uint).range(0, 319))
        .prop(PropSchema::new("row-offset", ValueKind::Uint).range(0, 319))
        .prop(PropSchema::new("id", ValueKind::Uint).range(0, 0x00ff_ffff))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("cs", ValueKind::Uint).range(0, MAX_CHIP_SELECTS as u64 - 1))
        .port(spi_pin::SCK_NAME, PortDir::In)
        .port(spi_pin::MOSI_NAME, PortDir::In)
        .port(spi_pin::CS_NAME, PortDir::In)
        .port(spi_pin::MISO_NAME, PortDir::Out)
        .port(pin::DC, PortDir::In)
        .port(pin::RES, PortDir::In)
}
