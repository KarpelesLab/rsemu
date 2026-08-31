//! The Sitronix ST7272A: a TFT panel you configure over SPI and feed over RGB.
//!
//! # What this part is, and what it is not
//!
//! **The ST7272A holds no picture.** It is a *panel driver* — 480 source
//! channels and 480 gate channels for a dual-gate 320RGB×240 TFT — and its
//! 3-wire SPI interface carries **register configuration only**: power, gamma,
//! contrast, brightness, scan direction, standby. Pixels arrive on a separate
//! parallel-RGB input (DCLK, HSYNC, VSYNC, DE and up to twenty-four data
//! lines). It is emphatically *not* an ST7735 or ST7789, which stream
//! framebuffer content over SPI into their own GRAM.
//!
//! This was checked rather than assumed, and here is the evidence from the
//! datasheet (Sitronix **ST7272A**, version 0.5, 2018/05):
//!
//! * **§5, Block Diagram (p.33).** The pixel path is `Data Shift → Data Latch
//!   → Level Shifter → DAC → 480 Source Buffer`, fed from `DR[7:0]`,
//!   `DG[7:0]`, `DB[7:0]`, `DCLK`, `HSYNC`, `VSYNC`, `DE`. `CS`, `SDA` and
//!   `SCL` feed a separate `Instruction Register` block. There is **no frame
//!   memory anywhere in the diagram** — "Data Latch" is the one-line source
//!   latch every TFT driver has, and the two "Buffer" blocks are the analogue
//!   source and gate output buffers.
//! * **§1 and §2 (pp.5-6).** "digital timing generator, source and gate driver,
//!   power supply circuit and **embedded serial communication interface for
//!   function setting**", and "support **dual gate panel resolution: 320RGB *
//!   240**".
//! * **§8.1, Register Summary (pp.46-48).** The complete register list is four
//!   tables — command table 1 (`10h`-`1Ch`), command table 2 (`40h`-`4Ah`),
//!   gamma (`20h`-`29h`, `30h`-`39h`) and OTP (`01h`-`6Ch`). **None of them is
//!   a memory-write register**: there is no `RAMWR`, no `CASET`/`RASET`, no
//!   address window, no auto-increment data port.
//! * **§7.1 (p.37).** "Each serial command consists of **16 bits** of data" —
//!   `R/W`, `A6..A0`, `D7..D0`. One byte per frame, to one address, with no
//!   streaming mode. Pixels physically cannot go through it.
//!
//! # The one thing that *is* double-buffered
//!
//! Not pixels — **the command registers**. §7.1(c): "the serial control block
//! is operational after power on reset, but **commands are established by the
//! VSYNC signal**. If command is transferred multiple times for the same
//! register, the **last command before the VSYNC signal is valid**."
//!
//! So a write lands in a shadow bank and the active bank is refreshed once per
//! frame. [`St7272a`] models exactly that: [`St7272a::shadow`] is what SPI
//! wrote, [`St7272a::active`] is what the panel is driving, and the panel is a
//! *lazily advanced* device (`ROADMAP.md` §4.2) on its own DCLK domain that
//! latches at each frame boundary. That is why it needs a `clock`.
//!
//! # Geometry
//!
//! The datasheet's part is 320RGB×240 (§2; and §7.3.4's timing table gives
//! `Thdisp = 320 DCLK`, `Tvdisp = 240 HSYNC`). `width` and `height` are
//! properties anyway, because a board may pair this driver with a differently
//! sized panel and because the model has no business hard-coding a number the
//! machine file can state.
//!
//! # Framing on the wire
//!
//! §7.1, with §9.3.3's timing:
//!
//! ```text
//!   CS   ‾‾\____________________________________________/‾‾
//!   SCL     _/‾\_/‾\_/‾\_ … 16 rising edges … _/‾\_/‾\_
//!   SDA      R/W A6  A5  A4 A3 A2 A1 A0 D7 D6 D5 D4 D3 D2 D1 D0
//! ```
//!
//! * 16 bits, MSB first: one direction bit, a 7-bit address, a data byte.
//! * "loaded one bit a time at the **rising edge** of serial clock SCL" with
//!   SCL idling low — CPOL 0, CPHA 0, i.e. [`Mode::Mode0`].
//! * §7.1(b): the frame runs from the falling edge of `CS` to the next rising
//!   edge; (d) "If less than 16 bits of SCL are input while CS is low, the
//!   transferred data is **ignored**"; (e) with more than 16, "the previous 16
//!   bits … before the rising edge of CS pulse are valid data".
//! * §7.1(h): "After power on reset or GRB reset, it is required **100ms
//!   delay** to begin SPI communication." Recorded in
//!   [`RESET_SETTLE_NANOS`]; not enforced, because a model that refused early
//!   commands would break firmware that works on real silicon by luck, and
//!   because nothing in the datasheet says what an early command *does*.
//!
//! **`SDA` is one bidirectional pin on the real part** and is modelled as
//! separate `mosi` and `miso` pins, because an rsemu wire has fixed drivers and
//! cannot be tri-stated. A 3-wire controller ties them together outside.
//!
//! # What is modelled, and what is recorded
//!
//! *Modelled* — these change [`St7272a::apply`]'s answer, so they are visible
//! in a picture:
//!
//! * `10h` `DISP`: standby blanks the panel; `GRB`: writing 0 resets every
//!   register to its default (§8.2.1, and the note under each table).
//! * `11h`-`16h` contrast, sub-contrast R/B, brightness, sub-brightness R/B,
//!   with the gains and offsets §8.2.2-§8.2.7 give.
//! * `19h` `SBGR` (swap red and blue), `HDIR`, `VDIR` (scan direction).
//! * `1Ch` BIST: with the `bist_en` pin high, the panel drives one of the flat
//!   patterns of §12.1 instead of its RGB input.
//!
//! *Recorded and read back but inert* — every one of these is an analogue or
//! timing parameter with no digital consequence a model can observe:
//! `17h`/`18h` blanking, `1Bh` auto-refresh, command table 2 (`40h`-`4Ah`:
//! GVDD, GVCL, AVDD/AVCL, VGH/VGL, source equalise, op-amp power, gate
//! timing), the twenty gamma registers, and the OTP block (`01h`-`05h`,
//! `60h`-`6Ch`). They are stored because firmware writes them, reads some of
//! them back, and would misbehave if they vanished.
//!
//! **OTP programming is not performed.** `60h`/`65h` are stored; no fuse is
//! blown, `66h`-`6Ch` keep their reset counts, and the `ENPROG` pin is not
//! modelled. A model that pretended to burn one-time fuses would be inventing
//! state the datasheet's flow (§13) cannot verify.
//!
//! # Where the pixels come from
//!
//! Nowhere, here. This device has no picture path and no
//! [`Scanout`](crate::host::display::Scanout) — it cannot have one, because it
//! owns no memory. Whatever drives the panel's RGB input owns the framebuffer
//! and therefore owns the picture. [`St7272a::apply`] is the panel's half of
//! that, ready for the display controller to call once `core::device` grows a
//! typed handle for one device to reach another; until then the two are
//! deliberately independent, and this file says so rather than reaching across
//! with a global.
//!
//! No emulator was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::bus::spi::{
    BitOrder, ChipSelect, Format, Mode, SlavePins, SpiSlave, buses, pin as spi_pin,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::LazyHandle;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::{Level, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
const CLASS_NAME: &str = "sitronix.st7272a";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many register addresses the 7-bit address field reaches.
pub const REGISTER_COUNT: usize = 128;

/// The panel geometry of the part the datasheet describes: 320RGB × 240
/// (§2, "support dual gate panel resolution: 320RGB * 240").
pub const DEFAULT_WIDTH: u64 = 320;
/// The other half of it.
pub const DEFAULT_HEIGHT: u64 = 240;

/// `Th`, one horizontal period in DCLK, typical (§7.3.4, parallel 24-bit).
pub const DEFAULT_HTOTAL: u64 = 371;
/// `Tv`, one vertical period in HSYNC, typical (§7.3.4).
pub const DEFAULT_VTOTAL: u64 = 260;

/// §7.1(h): "After power on reset or GRB reset, it is required 100ms delay to
/// begin SPI communication."
///
/// Recorded for a board that wants to model the wait; nothing here enforces it,
/// because the datasheet does not say what an early command does and inventing
/// an answer would be worse than accepting one.
pub const RESET_SETTLE_NANOS: u64 = 100_000_000;

/// The word framing §7.1 specifies: 16 bits, MSB first, sampled on the rising
/// edge of a clock that idles low.
pub const FRAME: Format = Format {
    mode: Mode::Mode0,
    bits: 16,
    order: BitOrder::MsbFirst,
};

/// Bit 15 of a command word: 1 reads, 0 writes (§7.1, "R/W: Read/Write mode
/// control bit").
const CMD_READ: u32 = 1 << 15;
/// Bits 14:8 of a command word: the register address, `A6..A0`.
const CMD_ADDR_SHIFT: u32 = 8;
/// And its mask, once shifted down.
const CMD_ADDR_MASK: u32 = 0x7f;

// ---------------------------------------------------------------------------
// Register addresses (§8.1)
// ---------------------------------------------------------------------------

/// `10h` — GRB and DISP control (§8.2.1).
pub const REG_GRB_DISP: u8 = 0x10;
/// `11h` — RGB contrast (§8.2.2).
pub const REG_CONTRAST: u8 = 0x11;
/// `12h` — red sub-contrast (§8.2.3).
pub const REG_SUB_CONTRAST_R: u8 = 0x12;
/// `13h` — blue sub-contrast (§8.2.4).
pub const REG_SUB_CONTRAST_B: u8 = 0x13;
/// `14h` — RGB brightness (§8.2.5).
pub const REG_BRIGHTNESS: u8 = 0x14;
/// `15h` — red sub-brightness (§8.2.6).
pub const REG_SUB_BRIGHTNESS_R: u8 = 0x15;
/// `16h` — blue sub-brightness (§8.2.7).
pub const REG_SUB_BRIGHTNESS_B: u8 = 0x16;
/// `17h` — HSYNC back porch (§8.2.8).
pub const REG_H_BLANKING: u8 = 0x17;
/// `18h` — VSYNC back porch (§8.2.9).
pub const REG_V_BLANKING: u8 = 0x18;
/// `19h` — display mode: scan direction, RGB swap, signal polarities (§8.2.10).
pub const REG_DISPLAY_MODE: u8 = 0x19;
/// `1Bh` — auto-refresh control (§8.2.11).
pub const REG_AUTO_REFRESH: u8 = 0x1b;
/// `1Ch` — BIST pattern selection (§8.2.12).
pub const REG_BIST: u8 = 0x1c;

/// `10h` bit 3: `GRB=0` resets all registers to their default value (§8.2.1).
const GRB_DISP_GRB: u8 = 1 << 3;
/// `10h` bit 0: `DISP=0` is standby, `1` is normal (§8.2.1).
const GRB_DISP_DISP: u8 = 1 << 0;

/// `19h` bit 6: vertical scan direction (§8.2.10).
const MODE_VDIR: u8 = 1 << 6;
/// `19h` bit 5: horizontal scan direction (§8.2.10).
const MODE_HDIR: u8 = 1 << 5;
/// `19h` bit 4: `SBGR=1` exchanges the red and blue data (§8.2.10).
const MODE_SBGR: u8 = 1 << 4;

/// `1Ch` bits 2:0: which BIST pattern (§8.2.12).
const BIST_PICSEL: u8 = 0x07;

/// One register's reset value and the bits software may change.
///
/// A single table drives the defaults, the write masks and "is this address a
/// register at all", so the three can never disagree — an address absent from
/// the table is one §8.1's note 3 covers: "Do not use instructions not listed
/// in these tables."
#[derive(Debug, Clone, Copy)]
struct RegDef {
    addr: u8,
    default: u8,
    /// Bits `D7..D0` that hold what is written. Zeroes are the tables' fixed
    /// `0` and `1` columns, which read back as their fixed value.
    writable: u8,
}

/// Every register §8.1 lists, in address order.
///
/// The `--` defaults of command table 2 and the gamma block are "OTP setting
/// according to parameters of system application, panel loading and display
/// quality" (§8.1, note 2), which is to say the value is programmed into the
/// part at the factory and the datasheet cannot state it. Zero is used, and
/// nothing in the model reads those registers back for anything but firmware's
/// own benefit.
const REGISTERS: &[RegDef] = &[
    // -- OTP table (§8.5) ---------------------------------------------------
    RegDef {
        addr: 0x01,
        default: 0x7f,
        writable: 0x7f,
    }, // ID1
    RegDef {
        addr: 0x02,
        default: 0x7f,
        writable: 0x7f,
    }, // ID2
    RegDef {
        addr: 0x03,
        default: 0x7f,
        writable: 0x7f,
    }, // ID3
    RegDef {
        addr: 0x04,
        default: 0x78,
        writable: 0x7f,
    }, // I2C ID
    RegDef {
        addr: 0x05,
        default: 0x40,
        writable: 0x7f,
    }, // VCOM offset
    // -- command table 1 (§8.2) --------------------------------------------
    RegDef {
        addr: 0x10,
        default: 0x08,
        writable: 0x09,
    },
    RegDef {
        addr: 0x11,
        default: 0x40,
        writable: 0xff,
    },
    RegDef {
        addr: 0x12,
        default: 0x40,
        writable: 0x7f,
    },
    RegDef {
        addr: 0x13,
        default: 0x40,
        writable: 0x7f,
    },
    RegDef {
        addr: 0x14,
        default: 0x40,
        writable: 0xff,
    },
    RegDef {
        addr: 0x15,
        default: 0x40,
        writable: 0x7f,
    },
    RegDef {
        addr: 0x16,
        default: 0x40,
        writable: 0x7f,
    },
    RegDef {
        addr: 0x17,
        default: 0x2b,
        writable: 0xff,
    },
    RegDef {
        addr: 0x18,
        default: 0x0c,
        writable: 0xff,
    },
    RegDef {
        addr: 0x19,
        default: 0x6d,
        writable: 0xff,
    },
    RegDef {
        addr: 0x1b,
        default: 0x0c,
        writable: 0x04,
    },
    RegDef {
        addr: 0x1c,
        default: 0x38,
        writable: 0x3f,
    },
    // -- gamma, positive polarity (§8.4) -----------------------------------
    RegDef {
        addr: 0x20,
        default: 0x00,
        writable: 0x1f,
    }, // VRF0P
    RegDef {
        addr: 0x21,
        default: 0x00,
        writable: 0x1f,
    }, // VOS0P
    RegDef {
        addr: 0x22,
        default: 0x00,
        writable: 0xff,
    }, // PFP0/PKP0
    RegDef {
        addr: 0x23,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x24,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x25,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x26,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x27,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x28,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x29,
        default: 0x00,
        writable: 0x1f,
    }, // PKP7
    // -- gamma, negative polarity ------------------------------------------
    RegDef {
        addr: 0x30,
        default: 0x00,
        writable: 0x1f,
    }, // VRF0N
    RegDef {
        addr: 0x31,
        default: 0x00,
        writable: 0x1f,
    }, // VOS0N
    RegDef {
        addr: 0x32,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x33,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x34,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x35,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x36,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x37,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x38,
        default: 0x00,
        writable: 0xff,
    },
    RegDef {
        addr: 0x39,
        default: 0x00,
        writable: 0x1f,
    }, // PKN7
    // -- command table 2 (§8.3) --------------------------------------------
    RegDef {
        addr: 0x40,
        default: 0x40,
        writable: 0x3f,
    }, // GVDD, bit 6 fixed 1
    RegDef {
        addr: 0x41,
        default: 0x00,
        writable: 0x7f,
    }, // GVCL
    RegDef {
        addr: 0x44,
        default: 0x40,
        writable: 0x3f,
    }, // AVDD/AVCL, bit 6 fixed 1
    RegDef {
        addr: 0x45,
        default: 0x00,
        writable: 0x0f,
    }, // VGH/VGL
    RegDef {
        addr: 0x46,
        default: 0x00,
        writable: 0xff,
    }, // source equalise
    RegDef {
        addr: 0x47,
        default: 0x00,
        writable: 0x07,
    }, // source op-amp power
    RegDef {
        addr: 0x49,
        default: 0x04,
        writable: 0x70,
    }, // gate timing 1, bit 2 fixed 1
    RegDef {
        addr: 0x4a,
        default: 0x00,
        writable: 0x77,
    }, // gate timing 2
    // -- OTP control (§8.5) -------------------------------------------------
    RegDef {
        addr: 0x60,
        default: 0x00,
        writable: 0x06,
    }, // INTVPP, OTPEN
    RegDef {
        addr: 0x65,
        default: 0x00,
        writable: 0xff,
    }, // OTPACK
    RegDef {
        addr: 0x66,
        default: 0x03,
        writable: 0x07,
    }, // VCOM offset program times
    RegDef {
        addr: 0x67,
        default: 0x03,
        writable: 0x07,
    }, // command 2
    RegDef {
        addr: 0x68,
        default: 0x03,
        writable: 0x07,
    }, // gamma
    RegDef {
        addr: 0x69,
        default: 0x03,
        writable: 0x07,
    }, // ID1
    RegDef {
        addr: 0x6a,
        default: 0x03,
        writable: 0x07,
    }, // ID2
    RegDef {
        addr: 0x6b,
        default: 0x03,
        writable: 0x07,
    }, // ID3
    RegDef {
        addr: 0x6c,
        default: 0x03,
        writable: 0x07,
    }, // I2C ID
];

/// The reset value of every address, `0` where §8.1 lists no register.
const DEFAULTS: [u8; REGISTER_COUNT] = build_defaults();
/// The writable-bit mask of every address, `0` where there is no register.
const WRITABLE: [u8; REGISTER_COUNT] = build_writable();
/// Whether each address is a register at all.
const KNOWN: [bool; REGISTER_COUNT] = build_known();

const fn build_defaults() -> [u8; REGISTER_COUNT] {
    let mut out = [0u8; REGISTER_COUNT];
    let mut i = 0;
    while i < REGISTERS.len() {
        out[REGISTERS[i].addr as usize] = REGISTERS[i].default;
        i += 1;
    }
    out
}

const fn build_writable() -> [u8; REGISTER_COUNT] {
    let mut out = [0u8; REGISTER_COUNT];
    let mut i = 0;
    while i < REGISTERS.len() {
        out[REGISTERS[i].addr as usize] = REGISTERS[i].writable;
        i += 1;
    }
    out
}

const fn build_known() -> [bool; REGISTER_COUNT] {
    let mut out = [false; REGISTER_COUNT];
    let mut i = 0;
    while i < REGISTERS.len() {
        out[REGISTERS[i].addr as usize] = true;
        i += 1;
    }
    out
}

// ---------------------------------------------------------------------------
// The register file
// ---------------------------------------------------------------------------

/// The panel's registers: what SPI wrote, and what the panel is driving.
///
/// Two banks, because §7.1(c) says so — a command takes effect at the next
/// VSYNC, and the last write to an address before that VSYNC is the one that
/// counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Registers {
    values: [u8; REGISTER_COUNT],
}

impl Default for Registers {
    fn default() -> Registers {
        Registers { values: DEFAULTS }
    }
}

impl Registers {
    /// Every register at its reset value.
    #[must_use]
    pub fn new() -> Registers {
        Registers::default()
    }

    /// Read one register. Addresses §8.1 does not list read as `0`.
    #[must_use]
    pub fn get(&self, addr: u8) -> u8 {
        self.values[usize::from(addr & 0x7f)]
    }

    /// Write one register, keeping the fixed bits of its table row.
    ///
    /// Returns whether the address is a register at all. §8.1's note 3 — "Do
    /// not use instructions not listed in these tables" — does not say what
    /// happens if you do, so an unlisted address is dropped rather than
    /// guessed at, and the caller is told so it can be counted.
    pub fn set(&mut self, addr: u8, value: u8) -> bool {
        let index = usize::from(addr & 0x7f);
        if !KNOWN[index] {
            return false;
        }
        let mask = WRITABLE[index];
        self.values[index] = (DEFAULTS[index] & !mask) | (value & mask);
        true
    }

    /// Whether `addr` is a register §8.1 lists.
    #[must_use]
    pub fn is_known(addr: u8) -> bool {
        KNOWN[usize::from(addr & 0x7f)]
    }

    /// Whether the panel is out of standby: `10h` bit 0 (§8.2.1).
    #[must_use]
    pub fn display_on(&self) -> bool {
        self.get(REG_GRB_DISP) & GRB_DISP_DISP != 0
    }

    /// Whether `10h`'s `GRB` bit is set, meaning normal operation (§8.2.1).
    #[must_use]
    pub fn grb(&self) -> bool {
        self.get(REG_GRB_DISP) & GRB_DISP_GRB != 0
    }

    /// Whether red and blue are exchanged: `19h` `SBGR` (§8.2.10).
    #[must_use]
    pub fn swap_rb(&self) -> bool {
        self.get(REG_DISPLAY_MODE) & MODE_SBGR != 0
    }

    /// Whether the horizontal scan runs left to right: `19h` `HDIR` (§8.2.10).
    ///
    /// `HDIR = 1` is "from left to right", which is the reset value.
    #[must_use]
    pub fn scan_left_to_right(&self) -> bool {
        self.get(REG_DISPLAY_MODE) & MODE_HDIR != 0
    }

    /// Whether the vertical scan runs top to bottom: `19h` `VDIR` (§8.2.10).
    #[must_use]
    pub fn scan_top_to_bottom(&self) -> bool {
        self.get(REG_DISPLAY_MODE) & MODE_VDIR != 0
    }

    /// Which BIST pattern `1Ch` selects (§8.2.12, and §12.1).
    ///
    /// `101`, `110` and `111` are all black, exactly as the table says.
    #[must_use]
    pub fn bist_pattern(&self) -> [u8; 3] {
        match self.get(REG_BIST) & BIST_PICSEL {
            0b001 => [0xff, 0xff, 0xff], // white
            0b010 => [0xff, 0x00, 0x00], // red
            0b011 => [0x00, 0xff, 0x00], // green
            0b100 => [0x00, 0x00, 0xff], // blue
            // 000, 101, 110 and 111 are all black.
            _ => [0x00, 0x00, 0x00],
        }
    }

    /// The contrast gain, in 1/64ths.
    ///
    /// §8.2.2: `00h` is gain 0, `40h` is gain 1, `FFh` is gain 3.984 — which is
    /// `255/64`, so the register *is* the numerator over 64 and no rounding is
    /// involved.
    #[must_use]
    pub fn contrast_q6(&self) -> u32 {
        u32::from(self.get(REG_CONTRAST))
    }

    /// The red sub-contrast gain, in 1/1024ths.
    ///
    /// §8.2.3: `00h` is 0.75, `40h` is 1, `7Fh` is 1.246. In 1/1024ths that is
    /// `768 + v × 508 / 127`, which hits 768, 1024 and 1276 exactly at those
    /// three points.
    #[must_use]
    pub fn sub_contrast_r_q10(&self) -> u32 {
        sub_contrast_q10(self.get(REG_SUB_CONTRAST_R))
    }

    /// The blue sub-contrast gain, in 1/1024ths (§8.2.4).
    #[must_use]
    pub fn sub_contrast_b_q10(&self) -> u32 {
        sub_contrast_q10(self.get(REG_SUB_CONTRAST_B))
    }

    /// The brightness offset, -64 to +191 (§8.2.5).
    #[must_use]
    pub fn brightness(&self) -> i32 {
        i32::from(self.get(REG_BRIGHTNESS)) - 64
    }

    /// The red sub-brightness offset, -64 to +63 (§8.2.6).
    #[must_use]
    pub fn sub_brightness_r(&self) -> i32 {
        i32::from(self.get(REG_SUB_BRIGHTNESS_R) & 0x7f) - 64
    }

    /// The blue sub-brightness offset, -64 to +63 (§8.2.7).
    #[must_use]
    pub fn sub_brightness_b(&self) -> i32 {
        i32::from(self.get(REG_SUB_BRIGHTNESS_B) & 0x7f) - 64
    }
}

/// §8.2.3's gain curve in 1/1024ths. Integer throughout: a colour pipeline is
/// not the time path, but this tree has no floats in it and adding the first
/// one here would be a poor reason.
fn sub_contrast_q10(value: u8) -> u32 {
    768 + u32::from(value & 0x7f) * 508 / 127
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The ST7272A as a device.
#[derive(Debug)]
pub struct St7272a {
    shared: Arc<Shared>,
    pins: Arc<SlavePins>,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    /// Visible pixels across.
    width: u32,
    /// Visible pixels down.
    height: u32,
    /// One frame in DCLK ticks: `htotal × vtotal` (§7.3.4).
    frame_ticks: u64,
    /// DCLK ticks simulated, published for the scheduler's lock-free question.
    ticks: AtomicU64,
    /// The tick the next VSYNC falls on.
    next_event: AtomicU64,
    /// The catch-up handle. The panel does not use it for register access —
    /// SPI reaches it through wires, not through a bus — but holding it keeps
    /// the seam symmetrical with every other lazy device.
    lazy: Mutex<Option<LazyHandle>>,
    /// The `grb`, `disp` and `bist_en` pins handed out by
    /// [`Device::sink`], kept alive because a net refers to its sinks weakly
    /// (`core::device`, §4.3's weak edge).
    control: Mutex<alloc::vec::Vec<Arc<ControlSink>>>,
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// DCLK ticks simulated.
    ticks: u64,
    /// What SPI has written since the last VSYNC (§7.1(c)).
    shadow: Registers,
    /// What the panel is driving.
    active: Registers,
    /// The command word of the frame in progress.
    ///
    /// §7.1(e): "If 16 bits or more of SCL are input while CS is low, the
    /// **previous 16 bits** of transferred data before the rising edge of CS
    /// pulse are valid data." So a frame carrying several words commits only
    /// the last, and this slot is overwritten rather than appended to.
    pending: Option<u32>,
    /// The `GRB` input pin, if a machine description wired one. Low resets.
    grb: Level,
    /// The `DISP` input pin, if wired. Low is standby.
    disp: Level,
    /// The `BIST_EN` input pin, if wired. High enables the test pattern.
    bist_en: Level,
    /// Which of the three above a `wire` statement actually named. An unwired
    /// pin sits at its inactive level rather than at the low level a fresh net
    /// idles at, because a panel on a board that ties `DISP` high must not come
    /// up in standby just because the machine file did not mention it.
    wired: u8,
    /// How many commands landed on an address §8.1 does not list. Diagnostics
    /// only; §8.1's note 3 says not to send them.
    unlisted: u32,
    /// How many frames have been latched.
    frames: u64,
}

/// `State::wired` bit for the `grb` pin.
const WIRED_GRB: u8 = 1 << 0;
/// `State::wired` bit for the `disp` pin.
const WIRED_DISP: u8 = 1 << 1;
/// `State::wired` bit for the `bist_en` pin.
const WIRED_BIST: u8 = 1 << 2;

impl Default for State {
    fn default() -> State {
        State {
            ticks: 0,
            shadow: Registers::new(),
            active: Registers::new(),
            pending: None,
            grb: Level::High,
            disp: Level::High,
            bist_en: Level::Low,
            wired: 0,
            unlisted: 0,
            frames: 0,
        }
    }
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("width", &self.width).field("height", &self.height);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

/// The pin names a machine description wires, beyond SPI's own.
pub mod pin {
    /// The global reset input, `GRB`. Active low (§6.1): "When GRB is 'L',
    /// internal initialization procedure is executed."
    pub const GRB: &str = "grb";
    /// The display-enable input, `DISP`. Low is standby (§6.1).
    pub const DISP: &str = "disp";
    /// The BIST enable input, `BIST_EN`. High runs the test pattern (§12).
    pub const BIST_EN: &str = "bist_en";

    /// Wire line for [`GRB`].
    pub const GRB_LINE: u32 = 16;
    /// Wire line for [`DISP`].
    pub const DISP_LINE: u32 = 17;
    /// Wire line for [`BIST_EN`].
    pub const BIST_EN_LINE: u32 = 18;
}

impl St7272a {
    /// Validate `props` and build the panel.
    ///
    /// Properties:
    ///
    /// * `width`, `height` — the panel's visible geometry. Default 320 × 240,
    ///   the part of §2.
    /// * `htotal`, `vtotal` — one horizontal period in DCLK and one vertical
    ///   period in HSYNC (§7.3.4). Their product is the frame in DCLK ticks,
    ///   which is when §7.1(c)'s command latch happens.
    /// * `bus`, `cs` — the named SPI bus to attach to, and the chip select.
    ///   Optional: a machine that only wires the pins up needs neither.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property, [`Error::Config`] for a
    /// zero dimension, a period shorter than the visible area, or a chip select
    /// out of range.
    pub fn new(props: &Props) -> Result<St7272a> {
        let mut r = props.reader();
        let width: u64 = r.or("width", DEFAULT_WIDTH)?;
        let height: u64 = r.or("height", DEFAULT_HEIGHT)?;
        let htotal: u64 = r.or("htotal", DEFAULT_HTOTAL)?;
        let vtotal: u64 = r.or("vtotal", DEFAULT_VTOTAL)?;
        let bus_name = r.optional_str("bus")?.map(String::from);
        let cs: u64 = r.or("cs", 0)?;
        r.finish()?;

        let bad = |message: String| Error::Config {
            at: String::from(CLASS_NAME),
            message,
        };
        if width == 0 || height == 0 {
            return Err(bad(alloc::format!(
                "a panel is {width}x{height}; both dimensions must be at least 1"
            )));
        }
        if width > u64::from(u32::MAX) || height > u64::from(u32::MAX) {
            return Err(bad(String::from(
                "a panel dimension is a pixel count, not an address",
            )));
        }
        if htotal < width || vtotal < height {
            return Err(bad(alloc::format!(
                "the total period {htotal}x{vtotal} is smaller than the visible {width}x{height}; \
                 §7.3.4's Th and Tv include the blanking, so they are never the smaller pair"
            )));
        }

        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            width: width as u32,
            height: height as u32,
            frame_ticks: htotal.saturating_mul(vtotal),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(htotal.saturating_mul(vtotal)),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
            control: Mutex::with_rank(LockRank::WIRE, alloc::vec::Vec::new()),
        });
        let pins = Arc::new(SlavePins::new(Arc::clone(&shared) as Arc<dyn SpiSlave>));

        // Attaching to the named bus is the *only* outward action construction
        // takes, and it is deferred to realize below rather than done here —
        // two-phase construction means nothing observable happens first
        // (`ROADMAP.md` §4.4). What happens here is opening the table entry,
        // which creates nothing anybody can see.
        let panel = St7272a { shared, pins };
        if let Some(name) = bus_name {
            let bus = buses::open(&name);
            if cs >= u64::from(crate::bus::spi::MAX_CHIP_SELECTS as u8) {
                return Err(bad(alloc::format!(
                    "`cs` is {cs}; an SPI bus routes {} chip selects",
                    crate::bus::spi::MAX_CHIP_SELECTS
                )));
            }
            bus.attach(
                ChipSelect(cs as u8),
                Arc::clone(&panel.shared) as Arc<dyn SpiSlave>,
            )?;
        }
        Ok(panel)
    }

    /// The panel's visible geometry.
    #[must_use]
    pub fn size(&self) -> (u32, u32) {
        (self.shared.width, self.shared.height)
    }

    /// One frame, in DCLK ticks (§7.3.4's `Th × Tv`).
    #[must_use]
    pub fn frame_ticks(&self) -> u64 {
        self.shared.frame_ticks
    }

    /// Frames latched since reset.
    #[must_use]
    pub fn frames(&self) -> u64 {
        self.shared.state.lock().frames
    }

    /// What SPI has written but VSYNC has not yet established (§7.1(c)).
    #[must_use]
    pub fn shadow(&self) -> Registers {
        self.shared.state.lock().shadow
    }

    /// What the panel is currently driving.
    #[must_use]
    pub fn active(&self) -> Registers {
        self.shared.state.lock().active
    }

    /// The panel's wire pins, for a controller that drives them directly.
    #[must_use]
    pub fn pins(&self) -> &Arc<SlavePins> {
        &self.pins
    }

    /// How many commands named an address §8.1 does not list.
    #[must_use]
    pub fn unlisted_commands(&self) -> u32 {
        self.shared.state.lock().unlisted
    }

    /// Whether the panel is lit at all: `DISP` high in both the pin and `10h`,
    /// and `GRB` not held low.
    #[must_use]
    pub fn is_displaying(&self) -> bool {
        let state = self.shared.state.lock();
        State::displaying(&state)
    }

    /// The flat colour the panel is driving instead of its RGB input, if BIST
    /// is running (§12, and §8.2.12's pattern table).
    ///
    /// `None` means the panel is showing what arrives on its RGB pins.
    #[must_use]
    pub fn bist_colour(&self) -> Option<[u8; 3]> {
        let state = self.shared.state.lock();
        (state.wired & WIRED_BIST != 0 && state.bist_en.is_high())
            .then(|| state.active.bist_pattern())
    }

    /// Put one incoming RGB888 pixel through the panel's own processing.
    ///
    /// This is the panel's half of the picture and the reason its registers are
    /// worth modelling: contrast, the two sub-contrasts, brightness, the two
    /// sub-brightnesses, `SBGR` and standby all land here, so a firmware that
    /// dims the display over SPI produces a visibly dimmer frame.
    ///
    /// Nothing calls it yet — see the module docs. The display controller that
    /// owns the framebuffer is the caller, and reaching it needs a typed
    /// device-to-device handle that `core::device` does not have.
    ///
    /// # Order of operations
    ///
    /// `SBGR` swap, then contrast (main × sub), then brightness (main + sub),
    /// then clamp. The datasheet's block diagram (§5) puts the whole colour
    /// adjustment between the data latch and the DAC without ordering the
    /// stages within it, so this order is *chosen*: gain before offset is what
    /// the register descriptions' units imply — a "gain" of 0 must produce
    /// black regardless of brightness, and it does.
    #[must_use]
    pub fn apply(&self, rgb: [u8; 3]) -> [u8; 3] {
        let state = self.shared.state.lock();
        if !State::displaying(&state) {
            // Standby drives nothing. §8.2.1: "DISP=0: standby mode".
            return [0, 0, 0];
        }
        if state.wired & WIRED_BIST != 0 && state.bist_en.is_high() {
            return state.active.bist_pattern();
        }
        State::process(&state.active, rgb)
    }

    /// Where a pixel at `(x, y)` of the incoming stream lands on the glass.
    ///
    /// `HDIR` and `VDIR` are scan directions (§8.2.10), so reversing one
    /// mirrors the picture. Separate from [`apply`](St7272a::apply) because a
    /// caller that walks a framebuffer wants to transform the coordinate once
    /// per pixel and the colour once per pixel, not both through one call.
    #[must_use]
    pub fn map_pixel(&self, x: u32, y: u32) -> (u32, u32) {
        let state = self.shared.state.lock();
        let x = if state.active.scan_left_to_right() {
            x
        } else {
            self.shared.width.saturating_sub(1).saturating_sub(x)
        };
        let y = if state.active.scan_top_to_bottom() {
            y
        } else {
            self.shared.height.saturating_sub(1).saturating_sub(y)
        };
        (x, y)
    }

    /// Establish the shadow bank, as VSYNC does (§7.1(c)).
    ///
    /// Public so a board that drives VSYNC from somewhere other than this
    /// panel's own clock can say when a frame starts.
    pub fn latch(&self) {
        let mut state = self.shared.state.lock();
        state.active = state.shadow;
        state.frames += 1;
        self.shared.publish(&state);
    }

    /// Run the panel until `target` DCLK ticks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

impl State {
    /// Whether anything is on the glass.
    fn displaying(state: &State) -> bool {
        if state.wired & WIRED_GRB != 0 && state.grb.is_low() {
            return false;
        }
        if state.wired & WIRED_DISP != 0 && state.disp.is_low() {
            return false;
        }
        state.active.display_on()
    }

    /// The colour pipeline of §8.2.2 to §8.2.7 and §8.2.10's `SBGR`.
    fn process(regs: &Registers, rgb: [u8; 3]) -> [u8; 3] {
        let [mut r, g, mut b] = rgb;
        if regs.swap_rb() {
            core::mem::swap(&mut r, &mut b);
        }
        let contrast = regs.contrast_q6();
        // Gain first: `CONTRAST = 00h` is "contrast gain=0" (§8.2.2), which has
        // to be black whatever brightness says, and applying the offset first
        // would leave a grey.
        let gain = |v: u8, sub_q10: u32| -> i32 {
            let scaled = u32::from(v) * contrast * sub_q10;
            // /64 for the main gain's Q6, /1024 for the sub gain's Q10.
            (scaled / (64 * 1024)) as i32
        };
        let plain = |v: u8| -> i32 { ((u32::from(v) * contrast) / 64) as i32 };
        let out = [
            gain(r, regs.sub_contrast_r_q10()) + regs.brightness() + regs.sub_brightness_r(),
            plain(g) + regs.brightness(),
            gain(b, regs.sub_contrast_b_q10()) + regs.brightness() + regs.sub_brightness_b(),
        ];
        [
            out[0].clamp(0, 255) as u8,
            out[1].clamp(0, 255) as u8,
            out[2].clamp(0, 255) as u8,
        ]
    }
}

impl Shared {
    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        // The next frame boundary, always strictly ahead of the present.
        let next = state
            .ticks
            .saturating_sub(state.ticks % self.frame_ticks)
            .saturating_add(self.frame_ticks);
        self.next_event
            .store(next.max(state.ticks.saturating_add(1)), Ordering::Relaxed);
    }

    /// Simulate forward, latching the shadow bank at every frame boundary.
    fn advance_to(&self, target: u64) {
        let mut state = self.state.lock();
        if target <= state.ticks {
            return;
        }
        let before = state.ticks / self.frame_ticks;
        let after = target / self.frame_ticks;
        state.ticks = target;
        if after > before {
            // §7.1(c): the last command before VSYNC is the valid one, so the
            // whole shadow bank becomes active in one step however many frames
            // were skipped.
            state.active = state.shadow;
            state.frames += after - before;
        }
        self.publish(&state);
    }

    /// Handle one 16-bit command word (§7.1).
    ///
    /// A read frame has already been answered by
    /// [`SpiSlave::partial`](crate::bus::spi::SpiSlave::partial) by the time
    /// this runs, and changes nothing.
    fn command(&self, word: u32) {
        let addr = ((word >> CMD_ADDR_SHIFT) & CMD_ADDR_MASK) as u8;
        if word & CMD_READ != 0 {
            return;
        }
        let mut state = self.state.lock();
        let data = (word & 0xff) as u8;
        if !state.shadow.set(addr, data) {
            state.unlisted = state.unlisted.saturating_add(1);
            return;
        }
        if addr == REG_GRB_DISP && data & GRB_DISP_GRB == 0 {
            // §8.2.1: "GRB=0: reset all registers to default value", and §8.1's
            // note 1 for each table. Both banks, immediately — this is a reset,
            // not a command waiting on VSYNC.
            state.shadow = Registers::new();
            state.active = Registers::new();
        }
    }
}

// ---------------------------------------------------------------------------
// The SPI face
// ---------------------------------------------------------------------------

impl SpiSlave for Shared {
    fn format(&self) -> Format {
        FRAME
    }

    /// After eight: `R/W` plus `A6..A0` come from the master, `D7..D0` from
    /// the panel on a read (§7.1, "Read Mode").
    fn turnaround(&self) -> Option<u8> {
        Some(8)
    }

    fn partial(&self, bits: u8, received: u32) -> Option<u32> {
        if bits != 8 {
            return None;
        }
        // Bit 7 of the eight received is `R/W`; a write frame is the master's
        // to drive all the way down and the panel answers nothing.
        if received & 0x80 == 0 {
            return None;
        }
        let addr = (received & CMD_ADDR_MASK) as u8;
        // From the shadow bank: it holds what the last write put there, which
        // is what firmware reading its own configuration back expects. §8.1's
        // TYPE column makes most of the map write-only anyway; command table 2
        // is the R/W part.
        Some(u32::from(self.state.lock().shadow.get(addr)))
    }

    fn select(&self, selected: bool) {
        // §7.1(b): "Command loading operation starts from the falling edge of
        // CS and is completed at the next rising edge of CS." So the command is
        // applied *here*, on deassertion — not when its sixteenth bit arrived.
        let word = {
            let mut state = self.state.lock();
            if selected {
                state.pending = None;
                None
            } else {
                state.pending.take()
            }
        };
        if let Some(word) = word {
            self.command(word);
        }
    }

    fn transfer(&self, mosi: u32) -> u32 {
        // Held, not applied: §7.1(e) makes the *last* whole word before the
        // rising edge of CS the valid one, so a longer frame overwrites this
        // rather than committing twice.
        self.state.lock().pending = Some(mosi);
        // What the panel was driving through the frame. The low byte of a read
        // is spliced in by `partial` above; the high half is the master's to
        // drive on the real part's single `SDA` pin, and reads as the pull-up
        // on the split pins this model uses.
        u32::MAX
    }

    fn peek(&self) -> u32 {
        u32::MAX
    }
}

// ---------------------------------------------------------------------------
// The control pins
// ---------------------------------------------------------------------------

/// One of the panel's three control inputs.
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
            pin::GRB_LINE => {
                let was = state.grb;
                state.grb = level;
                if was.is_high() && level.is_low() {
                    // §6.1: "When GRB is 'L', internal initialization procedure
                    // is executed", and §8.1's note 1: every register returns to
                    // its default.
                    state.shadow = Registers::new();
                    state.active = Registers::new();
                }
            }
            pin::DISP_LINE => state.disp = level,
            pin::BIST_EN_LINE => state.bist_en = level,
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for St7272a {
    fn class(&self) -> &'static DeviceClass {
        &ST7272A_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            let (ticks, wired, grb, disp, bist_en) = (
                state.ticks,
                state.wired,
                state.grb,
                state.disp,
                state.bist_en,
            );
            *state = State {
                ticks,
                wired,
                // The input levels belong to whatever drives them; a reset of
                // this device does not move another device's pin.
                grb,
                disp,
                bist_en,
                ..State::default()
            };
            self.shared.publish(&state);
        }
        self.pins.reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u64(state.frames)?;
        for addr in 0..REGISTER_COUNT {
            w.write_u8(state.shadow.values[addr])?;
        }
        for addr in 0..REGISTER_COUNT {
            w.write_u8(state.active.values[addr])?;
        }
        w.write_u32(state.unlisted)?;
        w.write_bool(state.pending.is_some())?;
        w.write_u32(state.pending.unwrap_or(0))?;
        // The bit-level shift register: a snapshot taken part-way through a
        // command has to resume mid-command, not restart it.
        let (rx, tx, count, selected, sck, mosi, loaded) = self.pins.snapshot();
        w.write_u32(rx)?;
        w.write_u32(tx)?;
        w.write_u8(count)?;
        w.write_bool(selected)?;
        w.write_bool(sck)?;
        w.write_bool(mosi)?;
        w.write_bool(loaded)
        // `grb`, `disp` and `bist_en` are not saved: they are levels *other*
        // devices drive, and each restores its own state and drives them again
        // (`ROADMAP.md` §4.5).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let ticks = r.read_u64()?;
        let frames = r.read_u64()?;
        let mut shadow = Registers::new();
        for addr in 0..REGISTER_COUNT {
            shadow.values[addr] = r.read_u8()?;
        }
        let mut active = Registers::new();
        for addr in 0..REGISTER_COUNT {
            active.values[addr] = r.read_u8()?;
        }
        let unlisted = r.read_u32()?;
        // Both fields are always written, so both are always read: a
        // conditional decode would desynchronise the rest of the chunk.
        let has_pending = r.read_bool()?;
        let pending_word = r.read_u32()?;
        let pending = has_pending.then_some(pending_word);
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
            state.ticks = ticks;
            state.frames = frames;
            state.shadow = shadow;
            state.active = active;
            state.unlisted = unlisted;
            state.pending = pending;
            self.shared.publish(&state);
        }
        self.pins.restore(pins);
        Ok(())
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        let control = |line: u32, bit: u8| -> SinkPin {
            self.shared.state.lock().wired |= bit;
            let pin = Arc::new(ControlSink {
                shared: Arc::clone(&self.shared),
                line,
            });
            // Kept, because a net refers to its sinks weakly.
            self.shared.control.lock().push(Arc::clone(&pin));
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
            pin::GRB => Some(control(pin::GRB_LINE, WIRED_GRB)),
            pin::DISP => Some(control(pin::DISP_LINE, WIRED_DISP)),
            pin::BIST_EN => Some(control(pin::BIST_EN_LINE, WIRED_BIST)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != spi_pin::MISO_NAME {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!(
                    "the ST7272A drives only `{}` — on the real part it is the same `SDA` pin as \
                     `{}`, split here because a wire has fixed drivers",
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

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes, and for one reason: §7.1(c)'s command latch happens at VSYNC, so
    /// the panel has to know where a frame boundary falls.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        St7272a::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        Some(self.shared.next_event.load(Ordering::Relaxed))
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for St7272a {}

/// The `sitronix.st7272a` device class.
pub static ST7272A_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Sitronix ST7272A TFT panel driver: SPI register configuration, RGB pixel input",
    properties: &[
        PropertySpec {
            name: "width",
            kind: ValueKind::Uint,
            required: false,
            summary: "visible pixels across (default 320, the part of §2)",
        },
        PropertySpec {
            name: "height",
            kind: ValueKind::Uint,
            required: false,
            summary: "visible pixels down (default 240)",
        },
        PropertySpec {
            name: "htotal",
            kind: ValueKind::Uint,
            required: false,
            summary: "one horizontal period in DCLK, blanking included (§7.3.4's Th)",
        },
        PropertySpec {
            name: "vtotal",
            kind: ValueKind::Uint,
            required: false,
            summary: "one vertical period in HSYNC (§7.3.4's Tv)",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the named SPI bus to attach to, for a transactional link",
        },
        PropertySpec {
            name: "cs",
            kind: ValueKind::Uint,
            required: false,
            summary: "which chip select on that bus (default 0)",
        },
    ],
    construct: |props| Ok(Box::new(St7272a::new(props)?)),
};

/// Add [`ST7272A_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&ST7272A_CLASS)
}

/// Bind [`ST7272A_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(St7272a::new(props)?)))
}

/// What the validator should know about `sitronix.st7272a`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("width", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .prop(PropSchema::new("height", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .prop(PropSchema::new("htotal", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .prop(PropSchema::new("vtotal", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(
            PropSchema::new("cs", ValueKind::Uint)
                .range(0, crate::bus::spi::MAX_CHIP_SELECTS as u64 - 1),
        )
        .port(spi_pin::SCK_NAME, PortDir::In)
        .port(spi_pin::MOSI_NAME, PortDir::In)
        .port(spi_pin::CS_NAME, PortDir::In)
        .port(spi_pin::MISO_NAME, PortDir::Out)
        .port(pin::GRB, PortDir::In)
        .port(pin::DISP, PortDir::In)
        .port(pin::BIST_EN, PortDir::In)
}

#[cfg(test)]
mod tests;
