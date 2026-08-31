//! The Master System's video display processor — a Sega 315-5124 / 315-5246.
//!
//! A TMS9918A with Sega's extensions bolted on: the same four legacy modes and
//! the same pin-out, plus **mode 4** — four bitplanes, 512 tiles, a 32-entry
//! colour RAM, per-tile flipping and priority, hardware scrolling, and a
//! programmable **line interrupt** the TMS never had.
//!
//! ```text
//!   16 KiB VRAM   patterns, name table, sprite attribute table
//!   32 B  CRAM    32 colours of --BBGGRR, two bits a gun
//!   11 registers  $00-$0A, written through the control port
//!   2 ports       $BE data, $BF control and status
//! ```
//!
//! # Two ports, one 16-bit command word
//!
//! Everything reaches the chip through two bytes of the Z80's **I/O** space,
//! not through memory: `$BE` is the data port and `$BF` the control port. A
//! control write is half a command — the chip keeps a latch, and the second
//! write completes a 16-bit word whose top two bits say what the other 14 mean:
//!
//! ```text
//!   code 0   VRAM read      address latched, one byte prefetched
//!   code 1   VRAM write
//!   code 2   register write register = high & $0F, value = the *first* byte
//!   code 3   CRAM write
//! ```
//!
//! The latch is shared state with real consequences: an interrupt handler that
//! touches the VDP in the middle of a two-byte command corrupts it, which is why
//! reading the status register — something an interrupt handler always does —
//! **clears the latch**, and why a data-port access does too.
//!
//! # Reading the status register has four side effects
//!
//! `$BF` read returns the status byte and then clears the frame-interrupt flag,
//! the sprite-overflow flag, the collision flag *and* the pending line
//! interrupt, and resets the control latch. That is how an interrupt is
//! acknowledged: there is no separate acknowledge register.
//!
//! Which makes this chip the textbook case for [`MemAttrs::debug`]
//! (`CLAUDE.md`): a monitor or a gdb `x/1b` that read `$BF` the ordinary way
//! would acknowledge an interrupt the guest had not seen and clear a flag it was
//! waiting on. Every side effect here is conditional on the access *not* being a
//! debug read — the address-latch advance on the data port included.
//!
//! # `/INT` is a level
//!
//! [`IRQ_PIN`] is high while
//! `(frame flag AND R1 bit 5) OR (line pending AND R0 bit 4)`, and it stays high
//! until the guest reads `$BF`. That is what the silicon does — the Z80's
//! `/INT` is level-sensitive and the chip holds it — and modelling it as a level
//! rather than a pulse is what makes a handler that returns without reading the
//! status register loop forever, exactly as it does on hardware.
//!
//! # Time
//!
//! **Lazily advanced** (`ROADMAP.md` §4.2) on the pixel clock — master ÷ 2 —
//! one tick per pixel, [`DOTS_PER_LINE`] to a line. Two boundaries a line
//! matter, and the device reports whichever comes first, so the scheduler never
//! runs past a moment the guest could observe:
//!
//! * **dot 0**: the line begins; if it is an active line it is rendered here,
//!   from the registers as they stand.
//! * **dot [`HBLANK_DOT`]**: the active display ends. The line counter runs, the
//!   frame flag is raised on the first blanked line, and `/INT` settles.
//!
//! Rendering *before* the interrupt is the whole point of splitting the line in
//! two: a line interrupt for line *n* must arrive after line *n* is drawn, so
//! that the handler's scroll write bends line *n+1*. Get that backwards and
//! every raster split in the library is off by one line.
//!
//! This device also asks to be caught up on **every CPU cycle**
//! ([`Device::sampled_every_cycle`]), because it drives `/INT` and the core
//! looks at that pin once an instruction.
//!
//! # What is not modelled
//!
//! Written down rather than discovered:
//!
//! * **The pixel pipeline.** A line is rendered in one go at dot 0. Per-*line*
//!   effects — scroll splits, palette splits, status bars — are correct because
//!   the interrupt fires after the line is drawn; a mid-*line* register change
//!   is not.
//! * **The H counter's TH latch.** `$7F` returns the live position rather than
//!   the value a light-phaser trigger latched. There is no light phaser.
//! * **The fifth-sprite number** in the status register's low five bits. They
//!   read as ones, which is what mode 4 does; the legacy modes would report a
//!   sprite index there.
//! * **The SMS2 sprite-zoom quirk**, where only the first four sprites are
//!   doubled vertically. Zoom doubles both axes for every sprite here.
//! * **VRAM and CRAM access timing.** A write during the active display is
//!   neither stolen nor delayed, and the fetch slot pattern is not simulated.
//!
//! # Sources
//!
//! [SMS Power!'s development documents](https://www.smspower.org/Development/Documents)
//! throughout — the VDP register descriptions, the mode table, the name-table
//! and sprite-attribute layouts, the line-interrupt algorithm, and the V and H
//! counter tables — plus the TMS9918A datasheet for the legacy modes and their
//! palette. No emulator source of any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};

/// Pixels in one scanline, borders and blanking included.
pub const DOTS_PER_LINE: u64 = 342;

/// The dot at which the active display ends and the line's bookkeeping runs.
///
/// 256 of the 342 pixels are picture; the rest is border and blanking. The line
/// counter and the frame flag are handled here rather than at dot 0 so that a
/// line interrupt lands *after* the line it belongs to has been drawn.
pub const HBLANK_DOT: u64 = 256;

/// Visible width, in pixels. Every mode this chip has is 256 wide except the
/// legacy text mode, which is 240 and centred in the same window.
pub const SCREEN_WIDTH: usize = 256;

/// The tallest picture the chip can produce: mode 4's 240-line variant.
pub const SCREEN_HEIGHT: usize = 240;

/// How many entries the framebuffer holds.
pub const FRAMEBUFFER_LEN: usize = SCREEN_WIDTH * SCREEN_HEIGHT;

/// How much video RAM the chip addresses.
pub const VRAM_LEN: usize = 0x4000;

/// How many colours colour RAM holds.
pub const CRAM_LEN: usize = 32;

/// How many registers exist. `$00`-`$0A`; `$0B`-`$0F` decode to nothing.
pub const REGISTER_COUNT: usize = 11;

/// The name a `map` statement reaches the data and control ports by.
///
/// Two bytes: offset 0 is the data port (`$BE`), offset 1 the control port
/// (`$BF`). A board maps it with `mirror()` across `$80`-`$BF`, because the only
/// address line the chip sees there is A0.
pub const PORT_REGION: &str = "ports";

/// The name a `map` statement reaches the V and H counters by.
///
/// Two bytes: offset 0 is `$7E` (V) and offset 1 is `$7F` (H). They are a
/// separate aperture because the *write* side of that address range belongs to
/// the sound chip — `split(vdp.counters, psg)` in the machine file.
pub const COUNTER_REGION: &str = "counters";

/// The interrupt request pin. A level, held until the guest reads `$BF`.
pub const IRQ_PIN: &str = "irq";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// Television region
// ---------------------------------------------------------------------------

/// Which television standard the chip is wired for.
///
/// It changes how many scanlines a frame has and where the V counter jumps, and
/// nothing else about the chip. The *frequency* difference lives in the machine
/// file's oscillator, where `ROADMAP.md` §4.2 says it belongs — this type never
/// names a hertz.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum TvRegion {
    /// 262 lines to the frame.
    #[default]
    Ntsc,
    /// 313 lines to the frame.
    Pal,
}

impl TvRegion {
    /// Look one up by the name a machine file writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<TvRegion> {
        match name {
            "ntsc" => Some(TvRegion::Ntsc),
            "pal" => Some(TvRegion::Pal),
            _ => None,
        }
    }

    /// The name a machine file writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            TvRegion::Ntsc => "ntsc",
            TvRegion::Pal => "pal",
        }
    }

    /// Scanlines in one frame.
    #[must_use]
    pub const fn lines_per_frame(self) -> u16 {
        match self {
            TvRegion::Ntsc => 262,
            TvRegion::Pal => 313,
        }
    }

    /// The V-counter run table for `height` active lines.
    ///
    /// The counter is eight bits but a frame has more lines than that, so the
    /// chip repeats a stretch of values: it counts up, jumps *backwards* once,
    /// and counts up again. Each entry is `(first value, how many)`, and the
    /// runs sum to [`lines_per_frame`](TvRegion::lines_per_frame).
    ///
    /// Source: SMS Power!, the VDP documentation's V-counter tables.
    const fn vcounter_runs(self, height: u16) -> &'static [(u8, u16)] {
        match (self, height) {
            (TvRegion::Ntsc, 192) => &[(0x00, 0xdb), (0xd5, 0x2b)],
            (TvRegion::Ntsc, 224) => &[(0x00, 0xeb), (0xe5, 0x1b)],
            // 240 lines on a 262-line field leaves no room to jump: the counter
            // simply wraps. The mode is out of specification on NTSC hardware,
            // and this is what it does rather than what it should do.
            (TvRegion::Ntsc, _) => &[(0x00, 0x100), (0x00, 0x06)],
            (TvRegion::Pal, 192) => &[(0x00, 0xf3), (0xba, 0x46)],
            (TvRegion::Pal, 224) => &[(0x00, 0x100), (0x00, 0x03), (0xca, 0x36)],
            (TvRegion::Pal, _) => &[(0x00, 0x100), (0x00, 0x0b), (0xd2, 0x2e)],
        }
    }
}

// ---------------------------------------------------------------------------
// Modes
// ---------------------------------------------------------------------------

/// Which of the chip's five display modes is selected.
///
/// The four mode bits are spread across two registers and are *not* in bit
/// order, which is a TMS9918A inheritance: `M1` is R1 bit 4, `M2` is R0 bit 1,
/// `M3` is R1 bit 3, and `M4` — Sega's addition — is R0 bit 2.
///
/// Source: SMS Power!, the VDP documentation's mode table.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VdpMode {
    /// TMS9918A Graphics I: 32x24 tiles, one colour pair per eight patterns.
    GraphicsI,
    /// TMS9918A Text: 40x24 characters of 6x8, two colours for the screen.
    Text,
    /// TMS9918A Graphics II: three banks of 256 patterns, a colour pair a row.
    GraphicsII,
    /// TMS9918A Multicolor: 64x48 blocks of 4x4 pixels.
    Multicolor,
    /// Sega mode 4, with `height` active lines: 192, 224 or 240.
    Mode4 {
        /// How many lines the active display has.
        height: u16,
    },
}

impl VdpMode {
    /// How many lines the active display has in this mode.
    #[must_use]
    pub const fn active_height(self) -> u16 {
        match self {
            VdpMode::Mode4 { height } => height,
            // Every legacy mode is 192 lines. The TMS9918A had no other.
            _ => 192,
        }
    }

    /// Whether this is one of Sega's mode-4 variants.
    #[must_use]
    pub const fn is_mode4(self) -> bool {
        matches!(self, VdpMode::Mode4 { .. })
    }
}

// ---------------------------------------------------------------------------
// The TMS9918A palette
// ---------------------------------------------------------------------------

/// The TMS9918A's fifteen colours, quantised to this chip's six-bit output.
///
/// Entry 0 is transparent and shows the backdrop; the rest are the datasheet's
/// RGB values rounded to two bits a gun, because the 315-5124 has **one** video
/// DAC and the legacy modes go through it just as mode 4 does. Two bits cannot
/// separate the TMS's dark blue from its light blue, and on a real Master System
/// they do not look separated either — that is a property of the part, not of
/// this table.
///
/// Packed `--BBGGRR`, the same encoding colour RAM uses.
///
/// Source: the TMS9918A datasheet's colour table.
const TMS_PALETTE: [u8; 16] = [
    0x00, // 0  transparent — painted as the backdrop
    0x00, // 1  black
    0x18, // 2  medium green
    0x1d, // 3  light green
    0x35, // 4  dark blue
    0x35, // 5  light blue
    0x16, // 6  dark red
    0x3d, // 7  cyan
    0x17, // 8  medium red
    0x17, // 9  light red
    0x1a, // 10 dark yellow
    0x2b, // 11 light yellow
    0x18, // 12 dark green
    0x26, // 13 magenta
    0x2a, // 14 grey
    0x3f, // 15 white
];

// ---------------------------------------------------------------------------
// The engine
// ---------------------------------------------------------------------------

/// One sprite selected for a line, in mode 4.
#[derive(Debug, Clone, Copy)]
struct Sprite {
    /// Screen X of the leftmost pixel. Signed, because the early-clock bit
    /// shifts a sprite off the left edge.
    x: i32,
    /// Address of the eight-pixel pattern row this line needs.
    pattern: usize,
    /// Whether the pattern row is doubled horizontally.
    zoom: bool,
}

/// Everything the chip remembers.
struct Engine {
    vram: Vec<u8>,
    cram: Vec<u8>,
    regs: [u8; REGISTER_COUNT],

    /// The 14-bit address register the data port walks.
    addr: u16,
    /// Which of the four command codes the last control word selected.
    code: u8,
    /// Whether a control write is half-finished.
    latch: bool,
    /// The first byte of a half-finished control word.
    first: u8,
    /// The data port's read buffer. A VRAM read returns the *previous* byte.
    buffer: u8,

    /// Frame flag in bit 7, sprite overflow in bit 6, collision in bit 5.
    status: u8,
    /// The line counter, reloaded from R10 when it underflows.
    line_counter: u8,
    /// Whether a line interrupt is waiting to be acknowledged.
    line_irq: bool,

    /// R9 as it stood when the frame began. Vertical scroll is latched once a
    /// frame, so a mid-frame write to R9 does nothing until the next one — the
    /// asymmetry with R8, which is read per line, and a common source of bugs.
    vscroll_latch: u8,

    region: TvRegion,
    /// Pixel within the line, `0..DOTS_PER_LINE`.
    dot: u64,
    /// Line within the frame.
    line: u16,
    /// Pixels since reset. The device's own tick.
    dots: u64,
    /// Frames completed since reset.
    frame: u64,

    /// Six-bit `--BBGGRR` per pixel: what the video DAC is handed.
    fb: Vec<u8>,
}

impl fmt::Debug for Engine {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Engine")
            .field("mode", &self.mode())
            .field("line", &self.line)
            .field("dot", &self.dot)
            .field("status", &self.status)
            .field("regs", &self.regs)
            .finish_non_exhaustive()
    }
}

impl Engine {
    fn new(region: TvRegion) -> Engine {
        let mut engine = Engine {
            vram: vec![0; VRAM_LEN],
            cram: vec![0; CRAM_LEN],
            // A machine with no BIOS starts the table pointers at zero, which is
            // a blank screen until the guest programs them. R10 = $FF is the
            // documented reset value: a line counter that never underflows.
            regs: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff],
            addr: 0,
            code: 0,
            latch: false,
            first: 0,
            buffer: 0,
            status: 0,
            line_counter: 0xff,
            line_irq: false,
            vscroll_latch: 0,
            region,
            dot: 0,
            line: 0,
            dots: 0,
            frame: 0,
            fb: vec![0; FRAMEBUFFER_LEN],
        };
        engine.line_start();
        engine
    }

    // -- decoding the mode bits ---------------------------------------------

    fn mode(&self) -> VdpMode {
        let m1 = self.regs[1] & 0x10 != 0;
        let m2 = self.regs[0] & 0x02 != 0;
        let m3 = self.regs[1] & 0x08 != 0;
        let m4 = self.regs[0] & 0x04 != 0;
        if m4 {
            // The tall variants need two more bits set, and which two decides
            // which height. Anything else is 192 lines.
            let height = match (m1, m2, m3) {
                (true, false, true) => 224,
                (false, true, true) => 240,
                _ => 192,
            };
            VdpMode::Mode4 { height }
        } else if m1 {
            VdpMode::Text
        } else if m2 {
            VdpMode::GraphicsII
        } else if m3 {
            VdpMode::Multicolor
        } else {
            VdpMode::GraphicsI
        }
    }

    fn active_height(&self) -> u16 {
        self.mode().active_height()
    }

    fn display_on(&self) -> bool {
        self.regs[1] & 0x40 != 0
    }

    /// The colour the border and every transparent pixel show.
    ///
    /// R7's low nibble indexes the **sprite** half of colour RAM, entries 16-31,
    /// not the background half.
    fn backdrop(&self) -> u8 {
        if self.mode().is_mode4() {
            self.cram[16 + (self.regs[7] & 0x0f) as usize] & 0x3f
        } else {
            TMS_PALETTE[(self.regs[7] & 0x0f) as usize]
        }
    }

    // -- the interrupt line -------------------------------------------------

    /// Whether `/INT` is asserted, as a pure function of the state.
    fn irq_line(&self) -> bool {
        let frame = self.status & 0x80 != 0 && self.regs[1] & 0x20 != 0;
        let line = self.line_irq && self.regs[0] & 0x10 != 0;
        frame || line
    }

    // -- the counters -------------------------------------------------------

    /// `$7E`: the line number, folded into eight bits by the region's runs.
    fn vcounter(&self) -> u8 {
        let runs = self.region.vcounter_runs(self.active_height());
        let mut line = self.line;
        for &(first, count) in runs {
            if line < count {
                return first.wrapping_add(line as u8);
            }
            line -= count;
        }
        // Unreachable while the runs sum to the frame height, which they do;
        // answering with the last value beats a panic on a device path.
        0xff
    }

    /// `$7F`: the pixel position, halved and folded the same way.
    ///
    /// 342 pixels give 171 counter values, which do not fit in a byte either:
    /// the counter runs `$00`-`$93` and then `$E9`-`$FF`.
    fn hcounter(&self) -> u8 {
        let half = (self.dot / 2) as u8;
        if half <= 0x93 {
            half
        } else {
            0xe9u8.wrapping_add(half - 0x94)
        }
    }

    // -- advancing ----------------------------------------------------------

    /// Pixels until the next moment the guest could observe a change.
    fn next_boundary(&self) -> u64 {
        if self.dot < HBLANK_DOT {
            HBLANK_DOT - self.dot
        } else {
            DOTS_PER_LINE - self.dot
        }
    }

    /// Start of a line: latch what is latched once a frame, then draw.
    fn line_start(&mut self) {
        if self.line == 0 {
            self.vscroll_latch = self.regs[9];
        }
        let height = self.active_height();
        if self.line < height {
            self.render_line(self.line);
        }
    }

    /// End of a line's active display: the line counter, and the frame flag.
    ///
    /// The counter runs on every active line **and on the first blanked one**,
    /// so a 192-line frame decrements it 193 times. Outside that stretch it is
    /// reloaded every line, which is why a line interrupt cannot be made to fire
    /// during vertical blanking.
    ///
    /// Source: SMS Power!, the VDP documentation's line-interrupt description.
    fn hblank(&mut self) {
        let height = self.active_height();
        if self.line <= height {
            if self.line_counter == 0 {
                self.line_counter = self.regs[10];
                self.line_irq = true;
            } else {
                self.line_counter -= 1;
            }
        } else {
            self.line_counter = self.regs[10];
        }
        if self.line == height {
            self.status |= 0x80;
        }
    }

    /// Move to the next line, wrapping the frame.
    fn next_line(&mut self) {
        self.line += 1;
        if self.line >= self.region.lines_per_frame() {
            self.line = 0;
            self.frame += 1;
        }
    }

    /// Run to `target`, jumping boundary to boundary rather than pixel to pixel:
    /// nothing between two boundaries is observable, and a per-pixel loop would
    /// cost ninety thousand iterations a frame to produce the same state.
    fn advance_to(&mut self, target: u64) {
        while self.dots < target {
            let step = (target - self.dots).min(self.next_boundary());
            self.dot += step;
            self.dots += step;
            if self.dot == HBLANK_DOT {
                self.hblank();
            } else if self.dot == DOTS_PER_LINE {
                self.dot = 0;
                self.next_line();
                self.line_start();
            }
        }
    }

    // -- the ports ----------------------------------------------------------

    /// `$BF` read: the status byte, and four side effects.
    fn read_status(&mut self) -> u8 {
        let value = self.peek_status();
        self.status &= 0x1f;
        self.line_irq = false;
        self.latch = false;
        value
    }

    /// `$BF` read with no side effect at all — a debug read, and what
    /// [`MemAttrs::debug`] exists for.
    fn peek_status(&self) -> u8 {
        // The low five bits are the TMS9918A's fifth-sprite number and read as
        // ones in mode 4. See the module's "not modelled" list.
        (self.status & 0xe0) | 0x1f
    }

    /// `$BF` write: half a command, or a whole one.
    fn write_control(&mut self, value: u8) {
        if !self.latch {
            self.first = value;
            self.latch = true;
            // The low byte reaches the address register immediately, before the
            // command word is complete. Software relies on it.
            self.addr = (self.addr & 0x3f00) | u16::from(value);
            return;
        }
        self.latch = false;
        self.code = (value >> 6) & 0x03;
        self.addr = ((u16::from(value) & 0x3f) << 8) | u16::from(self.first);
        match self.code {
            0 => {
                // A read command prefetches, so the *first* data-port read
                // returns the byte at the address rather than the one before.
                self.buffer = self.vram[self.addr as usize & 0x3fff];
                self.addr = (self.addr + 1) & 0x3fff;
            }
            2 => {
                let index = (value & 0x0f) as usize;
                if index < REGISTER_COUNT {
                    self.regs[index] = self.first;
                }
            }
            _ => {}
        }
    }

    /// `$BE` read: the buffered byte, then prefetch the next.
    fn read_data(&mut self) -> u8 {
        let value = self.buffer;
        self.buffer = self.vram[self.addr as usize & 0x3fff];
        self.addr = (self.addr + 1) & 0x3fff;
        self.latch = false;
        value
    }

    /// `$BE` write: VRAM or CRAM, then step the address.
    fn write_data(&mut self, value: u8) {
        if self.code == 3 {
            self.cram[self.addr as usize % CRAM_LEN] = value;
        } else {
            self.vram[self.addr as usize & 0x3fff] = value;
        }
        // A write fills the read buffer too, which is how a program reads back
        // what it just wrote without issuing a read command.
        self.buffer = value;
        self.addr = (self.addr + 1) & 0x3fff;
        self.latch = false;
    }

    // -- rendering ----------------------------------------------------------

    fn fb_row(&mut self, line: u16) -> &mut [u8] {
        let start = line as usize * SCREEN_WIDTH;
        &mut self.fb[start..start + SCREEN_WIDTH]
    }

    fn render_line(&mut self, line: u16) {
        if !self.display_on() {
            let colour = self.backdrop();
            self.fb_row(line).fill(colour);
            return;
        }
        match self.mode() {
            VdpMode::Mode4 { .. } => self.render_mode4(line),
            VdpMode::GraphicsI => self.render_tms_tiles(line, false),
            VdpMode::GraphicsII => self.render_tms_tiles(line, true),
            VdpMode::Multicolor => self.render_multicolor(line),
            VdpMode::Text => self.render_text(line),
        }
    }

    // -- mode 4 -------------------------------------------------------------

    /// Where the name table starts.
    ///
    /// In the 192-line mode it is `(R2 & $0E) << 10`. The taller modes need 32
    /// rows rather than 28, so the table grew to `$800` bytes and moved: bit 0
    /// of the field is ignored and `$700` is added.
    fn name_table_base(&self, height: u16) -> usize {
        if height == 192 {
            ((self.regs[2] as usize) & 0x0e) << 10
        } else {
            (((self.regs[2] as usize) & 0x0c) << 10) | 0x700
        }
    }

    fn render_mode4(&mut self, line: u16) {
        let height = self.active_height();
        let backdrop = self.backdrop();
        let mut row = [backdrop; SCREEN_WIDTH];
        // Which pixels a sprite may not cover, and which are opaque background.
        let mut bg_priority = [false; SCREEN_WIDTH];

        let nt_base = self.name_table_base(height);
        let rows = if height == 192 { 28u16 } else { 32u16 };
        let nt_height = rows * 8;

        // R8 scrolls the picture right; the top two rows can be exempted, which
        // is how a game pins a status bar while the world scrolls under it.
        let hscroll = if self.regs[0] & 0x40 != 0 && line < 16 {
            0
        } else {
            self.regs[8]
        };
        let vscroll = self.vscroll_latch;

        for x in 0..SCREEN_WIDTH {
            // The rightmost eight columns can be exempted from vertical scroll
            // for the same reason, turned the other way.
            let locked = self.regs[0] & 0x80 != 0 && x >= 192;
            let src_y = if locked {
                line
            } else {
                (line + u16::from(vscroll)) % nt_height
            };
            let src_x = (x as u8).wrapping_sub(hscroll);

            let col = (src_x / 8) as usize;
            let tile_row = (src_y / 8) as usize;
            let entry = nt_base + (tile_row * 32 + col) * 2;
            let low = self.vram[entry & 0x3fff];
            let high = self.vram[(entry + 1) & 0x3fff];
            let word = u16::from(low) | (u16::from(high) << 8);

            let tile = (word & 0x1ff) as usize;
            let hflip = word & 0x200 != 0;
            let vflip = word & 0x400 != 0;
            let palette = if word & 0x800 != 0 { 16 } else { 0 };
            let priority = word & 0x1000 != 0;

            let mut px = src_x % 8;
            let mut py = (src_y % 8) as u8;
            if hflip {
                px = 7 - px;
            }
            if vflip {
                py = 7 - py;
            }
            let index = self.tile_pixel(tile * 32 + py as usize * 4, px);

            row[x] = self.cram[palette + index as usize] & 0x3f;
            // A background pixel only wins against a sprite when it is both
            // flagged and opaque: colour 0 of either palette is see-through as
            // far as priority is concerned, however it is drawn.
            bg_priority[x] = priority && index != 0;
        }

        self.draw_mode4_sprites(line, &mut row, &bg_priority);

        // R0 bit 5 blanks the leftmost eight pixels with the backdrop, which is
        // what hides the column of garbage a horizontally scrolling name table
        // pushes in from the left.
        if self.regs[0] & 0x20 != 0 {
            row[..8].fill(backdrop);
        }
        self.fb_row(line).copy_from_slice(&row);
    }

    /// One four-bitplane pixel. `base` addresses the row's four bytes.
    fn tile_pixel(&self, base: usize, x: u8) -> u8 {
        let bit = 7 - x;
        let mut index = 0u8;
        for plane in 0..4 {
            let byte = self.vram[(base + plane) & 0x3fff];
            index |= ((byte >> bit) & 1) << plane;
        }
        index
    }

    /// Pick the sprites on this line, then paint them.
    ///
    /// The chip walks the table until it has eight, or hits the `$D0`
    /// terminator, or runs out of the sixty-four. A ninth sprite on the line
    /// sets the overflow flag and is dropped — and dropping it is why sprites
    /// flicker on this machine rather than the picture slowing down.
    fn draw_mode4_sprites(
        &mut self,
        line: u16,
        row: &mut [u8; SCREEN_WIDTH],
        bg_priority: &[bool; SCREEN_WIDTH],
    ) {
        let sat = ((self.regs[5] as usize) & 0x7e) << 7;
        let pattern_base = ((self.regs[6] as usize) & 0x04) << 11;
        let tall = self.regs[1] & 0x02 != 0;
        let zoom = self.regs[1] & 0x01 != 0;
        let shift = if self.regs[0] & 0x08 != 0 { 8 } else { 0 };
        let base_height: u16 = if tall { 16 } else { 8 };
        let height = if zoom { base_height * 2 } else { base_height };
        let terminates = self.active_height() == 192;

        let mut chosen: [Option<Sprite>; 8] = [None; 8];
        let mut count = 0usize;
        for i in 0..64usize {
            let y = self.vram[(sat + i) & 0x3fff];
            // Only the 192-line mode honours the terminator; the taller modes
            // have no spare Y value to spend on it.
            if y == 0xd0 && terminates {
                break;
            }
            let top = u16::from(y).wrapping_add(1) & 0xff;
            // A sprite whose top is near $FF wraps to the top of the screen,
            // which is how one is parked off-screen without being disabled.
            let delta = line.wrapping_sub(top) & 0xff;
            if delta >= height {
                continue;
            }
            if count == 8 {
                self.status |= 0x40;
                break;
            }
            let x = i32::from(self.vram[(sat + 0x80 + i * 2) & 0x3fff]) - shift;
            let mut tile = self.vram[(sat + 0x81 + i * 2) & 0x3fff] as usize;
            if tall {
                // The low bit of the index is ignored: an 8x16 sprite is a pair
                // of patterns and must start on an even one.
                tile &= 0xfe;
            }
            let py = if zoom { delta / 2 } else { delta };
            chosen[count] = Some(Sprite {
                x,
                pattern: pattern_base + tile * 32 + py as usize * 4,
                zoom,
            });
            count += 1;
        }

        // Lower-numbered sprites win, so the first to claim a pixel keeps it —
        // and a second sprite arriving at a claimed pixel is the collision the
        // status register reports.
        let mut claimed = [false; SCREEN_WIDTH];
        for sprite in chosen.iter().flatten() {
            for i in 0..8i32 {
                let index = self.tile_pixel(sprite.pattern, i as u8);
                if index == 0 {
                    continue;
                }
                let width = if sprite.zoom { 2 } else { 1 };
                for step in 0..width {
                    let x = sprite.x + i * width + step;
                    if !(0..SCREEN_WIDTH as i32).contains(&x) {
                        continue;
                    }
                    let x = x as usize;
                    if claimed[x] {
                        self.status |= 0x20;
                        continue;
                    }
                    claimed[x] = true;
                    if bg_priority[x] {
                        continue;
                    }
                    row[x] = self.cram[16 + index as usize] & 0x3f;
                }
            }
        }
    }

    // -- the TMS9918A legacy modes ------------------------------------------

    /// Graphics I and Graphics II.
    ///
    /// The difference is entirely in how the pattern and colour tables are
    /// addressed: Graphics I gives eight patterns one colour pair, Graphics II
    /// gives every pattern *row* its own pair and splits the screen into three
    /// banks of 256 patterns.
    fn render_tms_tiles(&mut self, line: u16, graphics_ii: bool) {
        let backdrop = self.backdrop();
        let mut row = [backdrop; SCREEN_WIDTH];
        let nt = ((self.regs[2] as usize) & 0x0f) << 10;
        let tile_row = (line / 8) as usize;
        let py = (line % 8) as usize;

        for col in 0..32usize {
            let name = self.vram[(nt + tile_row * 32 + col) & 0x3fff] as usize;
            let (pattern, colour) = if graphics_ii {
                // R4's low bits and R3's mask which of the three banks a third
                // of the screen may reach — a board with less video RAM wires
                // them so the banks overlap.
                let bank = (tile_row / 8) * 0x100;
                let pattern_base = ((self.regs[4] as usize) & 0x04) << 11;
                let pattern_mask = (((self.regs[4] as usize) & 0x03) << 8) | 0xff;
                let colour_base = ((self.regs[3] as usize) & 0x80) << 6;
                let colour_mask = (((self.regs[3] as usize) & 0x7f) << 3) | 0x07;
                (
                    pattern_base + ((bank + name) & pattern_mask) * 8 + py,
                    colour_base + ((bank + name) & colour_mask) * 8 + py,
                )
            } else {
                let pattern_base = ((self.regs[4] as usize) & 0x07) << 11;
                let colour_base = (self.regs[3] as usize) << 6;
                (pattern_base + name * 8 + py, colour_base + name / 8)
            };
            let bits = self.vram[pattern & 0x3fff];
            let pair = self.vram[colour & 0x3fff];
            for x in 0..8usize {
                let on = bits & (0x80 >> x) != 0;
                let index = if on { pair >> 4 } else { pair & 0x0f };
                row[col * 8 + x] = if index == 0 {
                    backdrop
                } else {
                    TMS_PALETTE[index as usize]
                };
            }
        }

        self.draw_tms_sprites(line, &mut row);
        self.fb_row(line).copy_from_slice(&row);
    }

    /// Multicolor: 4x4 blocks straight out of the pattern table.
    fn render_multicolor(&mut self, line: u16) {
        let backdrop = self.backdrop();
        let mut row = [backdrop; SCREEN_WIDTH];
        let nt = ((self.regs[2] as usize) & 0x0f) << 10;
        let pattern_base = ((self.regs[4] as usize) & 0x07) << 11;
        let tile_row = (line / 8) as usize;
        // Each name selects an eight-byte pattern; which of those bytes a line
        // uses rotates through the row every four lines.
        let offset = ((line / 4) % 2) as usize + ((tile_row % 4) * 2);
        for col in 0..32usize {
            let name = self.vram[(nt + tile_row * 32 + col) & 0x3fff] as usize;
            let byte = self.vram[(pattern_base + name * 8 + offset) & 0x3fff];
            for half in 0..2usize {
                let index = if half == 0 { byte >> 4 } else { byte & 0x0f };
                let value = if index == 0 {
                    backdrop
                } else {
                    TMS_PALETTE[index as usize]
                };
                let start = col * 8 + half * 4;
                row[start..start + 4].fill(value);
            }
        }
        self.draw_tms_sprites(line, &mut row);
        self.fb_row(line).copy_from_slice(&row);
    }

    /// Text: 40 columns of 6x8, two colours, and no sprites at all.
    fn render_text(&mut self, line: u16) {
        let backdrop = self.backdrop();
        let mut row = [backdrop; SCREEN_WIDTH];
        let nt = ((self.regs[2] as usize) & 0x0f) << 10;
        let pattern_base = ((self.regs[4] as usize) & 0x07) << 11;
        let fg = TMS_PALETTE[(self.regs[7] >> 4) as usize];
        let bg = TMS_PALETTE[(self.regs[7] & 0x0f) as usize];
        let tile_row = (line / 8) as usize;
        let py = (line % 8) as usize;
        // 40 columns of six pixels is 240, centred in a 256-pixel window: the
        // eight pixels either side stay backdrop, which is what the border is.
        for col in 0..40usize {
            let name = self.vram[(nt + tile_row * 40 + col) & 0x3fff] as usize;
            let bits = self.vram[(pattern_base + name * 8 + py) & 0x3fff];
            for x in 0..6usize {
                row[8 + col * 6 + x] = if bits & (0x80 >> x) != 0 { fg } else { bg };
            }
        }
        self.fb_row(line).copy_from_slice(&row);
    }

    /// The TMS9918A's sprites: four to a line, one colour each, optionally
    /// magnified, and 16x16 when R1 bit 1 says so.
    fn draw_tms_sprites(&mut self, line: u16, row: &mut [u8; SCREEN_WIDTH]) {
        let sat = ((self.regs[5] as usize) & 0x7f) << 7;
        let pattern_base = ((self.regs[6] as usize) & 0x07) << 11;
        let large = self.regs[1] & 0x02 != 0;
        let zoom = self.regs[1] & 0x01 != 0;
        let base = if large { 16u16 } else { 8 };
        let height = if zoom { base * 2 } else { base };

        let mut drawn = 0usize;
        let mut claimed = [false; SCREEN_WIDTH];
        for entry_index in 0..32usize {
            let entry = sat + entry_index * 4;
            let y = self.vram[entry & 0x3fff];
            if y == 0xd0 {
                break;
            }
            let top = u16::from(y).wrapping_add(1) & 0xff;
            let delta = line.wrapping_sub(top) & 0xff;
            if delta >= height {
                continue;
            }
            if drawn == 4 {
                self.status |= 0x40;
                break;
            }
            drawn += 1;
            let colour_byte = self.vram[(entry + 3) & 0x3fff];
            let colour = colour_byte & 0x0f;
            if colour == 0 {
                continue;
            }
            // Bit 7 of the colour byte is the early-clock bit: it shifts the
            // sprite 32 pixels left so it can walk off that edge.
            let x = i32::from(self.vram[(entry + 1) & 0x3fff])
                - if colour_byte & 0x80 != 0 { 32 } else { 0 };
            let name = self.vram[(entry + 2) & 0x3fff] as usize & if large { 0xfc } else { 0xff };
            let py = if zoom { delta / 2 } else { delta } as usize;
            let value = TMS_PALETTE[colour as usize];
            let width = if large { 16usize } else { 8 };
            for i in 0..width {
                // A 16x16 sprite is four 8x8 patterns in column-major order: its
                // right half is the pattern sixteen bytes further on.
                let half = (i / 8) * 16;
                let byte = self.vram[(pattern_base + name * 8 + half + (py % 8)) & 0x3fff];
                if byte & (0x80 >> (i % 8)) == 0 {
                    continue;
                }
                let scale = if zoom { 2i32 } else { 1 };
                for step in 0..scale {
                    let sx = x + i as i32 * scale + step;
                    if !(0..SCREEN_WIDTH as i32).contains(&sx) {
                        continue;
                    }
                    let sx = sx as usize;
                    if claimed[sx] {
                        self.status |= 0x20;
                        continue;
                    }
                    claimed[sx] = true;
                    row[sx] = value;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

struct Shared {
    engine: Mutex<Engine>,
    irq: Mutex<Option<WireSource>>,
    lazy: Mutex<Option<LazyHandle>>,
    /// [`Engine::dots`], republished lock-free for the scheduler.
    dots: AtomicU64,
    /// The tick of the next boundary, republished alongside.
    next_event: AtomicU64,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("engine", &self.engine)
            .finish_non_exhaustive()
    }
}

impl Shared {
    fn publish(&self, engine: &Engine) {
        self.dots.store(engine.dots, Ordering::Relaxed);
        self.next_event
            .store(engine.dots + engine.next_boundary(), Ordering::Relaxed);
    }

    /// Drive `/INT`, with no lock of this device held.
    fn drive(&self, level: bool) {
        let source = self.irq.lock().clone();
        if let Some(source) = source {
            source.set(Level::from_bool(level));
        }
    }

    /// Run `f` against the engine, then settle `/INT` outside the lock.
    ///
    /// The re-entrancy contract in one function (`ROADMAP.md` §4.4).
    fn with_engine<R>(&self, f: impl FnOnce(&mut Engine) -> R) -> R {
        let (result, irq) = {
            let mut engine = self.engine.lock();
            let result = f(&mut engine);
            self.publish(&engine);
            (result, engine.irq_line())
        };
        self.drive(irq);
        result
    }

    /// Catch the chip up before an access reads or changes it.
    fn sync(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        let _ = handle.sync(kind);
    }
}

/// The Master System's video display processor.
pub struct SmsVdp {
    shared: Arc<Shared>,
    port_region: RegionRef,
    counter_region: RegionRef,
}

impl fmt::Debug for SmsVdp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SmsVdp")
            .field("engine", &self.shared.engine)
            .finish_non_exhaustive()
    }
}

impl Default for SmsVdp {
    fn default() -> Self {
        SmsVdp::new(TvRegion::Ntsc)
    }
}

impl SmsVdp {
    /// A chip in its power-on state, wired for `region`.
    #[must_use]
    pub fn new(region: TvRegion) -> SmsVdp {
        let engine = Engine::new(region);
        let next = engine.dots + engine.next_boundary();
        let shared = Arc::new(Shared {
            engine: Mutex::with_rank(LockRank::DEVICE, engine),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            lazy: Mutex::new(None),
            dots: AtomicU64::new(0),
            next_event: AtomicU64::new(next),
        });
        let port_region = Arc::new(MmioRegion::io(
            "sms.vdp.ports",
            2,
            Arc::new(VdpPorts {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        let counter_region = Arc::new(MmioRegion::io(
            "sms.vdp.counters",
            2,
            Arc::new(VdpCounters {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        SmsVdp {
            shared,
            port_region,
            counter_region,
        }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// If `region` is not `ntsc` or `pal`, or an unknown property was given.
    pub fn from_props(props: &Props) -> Result<SmsVdp> {
        let mut r = props.reader();
        let name = r.or_str("region", "ntsc")?;
        let region = TvRegion::from_name(name).ok_or_else(|| Error::Config {
            at: String::from("region"),
            message: alloc::format!("`{name}` is not a television region; use `ntsc` or `pal`"),
        })?;
        r.finish()?;
        Ok(SmsVdp::new(region))
    }

    /// Which television standard this chip is wired for.
    #[must_use]
    pub fn tv_region(&self) -> TvRegion {
        self.shared.engine.lock().region
    }

    /// Connect the interrupt request line.
    pub fn attach_irq(&self, source: WireSource) {
        *self.shared.irq.lock() = Some(source);
        let level = self.shared.engine.lock().irq_line();
        self.shared.drive(level);
    }

    /// Connect the catch-up handle the ports sync through.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// Pixels executed since reset.
    #[must_use]
    pub fn dots(&self) -> u64 {
        self.shared.dots.load(Ordering::Relaxed)
    }

    /// Frames completed since reset.
    #[must_use]
    pub fn frame(&self) -> u64 {
        self.shared.engine.lock().frame
    }

    /// The line being drawn, and how far into it, as `(line, dot)`.
    #[must_use]
    pub fn position(&self) -> (u16, u64) {
        let engine = self.shared.engine.lock();
        (engine.line, engine.dot)
    }

    /// Which display mode is selected.
    #[must_use]
    pub fn mode(&self) -> VdpMode {
        self.shared.engine.lock().mode()
    }

    /// How many lines the active display has right now.
    #[must_use]
    pub fn active_height(&self) -> u16 {
        self.shared.engine.lock().active_height()
    }

    /// One register, 0-10, without catching up — for a test or a monitor.
    #[must_use]
    pub fn register(&self, index: usize) -> u8 {
        let engine = self.shared.engine.lock();
        engine.regs.get(index).copied().unwrap_or(0)
    }

    /// The status byte with no side effect, as a debug read sees it.
    #[must_use]
    pub fn peek_status(&self) -> u8 {
        self.shared.engine.lock().peek_status()
    }

    /// One byte of video RAM, ignoring the address register.
    #[must_use]
    pub fn peek_vram(&self, offset: usize) -> u8 {
        self.shared.engine.lock().vram[offset & 0x3fff]
    }

    /// Write one byte of video RAM directly — for a test or a monitor.
    pub fn poke_vram(&self, offset: usize, value: u8) {
        self.shared.engine.lock().vram[offset & 0x3fff] = value;
    }

    /// One colour-RAM entry.
    #[must_use]
    pub fn peek_cram(&self, index: usize) -> u8 {
        self.shared.engine.lock().cram[index % CRAM_LEN]
    }

    /// Write one colour-RAM entry directly.
    pub fn poke_cram(&self, index: usize, value: u8) {
        self.shared.engine.lock().cram[index % CRAM_LEN] = value;
    }

    /// Whether `/INT` is asserted.
    #[must_use]
    pub fn irq_line(&self) -> bool {
        self.shared.engine.lock().irq_line()
    }

    /// The `$7E` V counter.
    #[must_use]
    pub fn vcounter(&self) -> u8 {
        self.shared.engine.lock().vcounter()
    }

    /// The `$7F` H counter.
    #[must_use]
    pub fn hcounter(&self) -> u8 {
        self.shared.engine.lock().hcounter()
    }

    /// Read one of the two ports as the guest would — data at 0, control at 1.
    pub fn read_port(&self, offset: u64) -> u8 {
        self.shared.with_engine(|e| {
            if offset & 1 == 0 {
                e.read_data()
            } else {
                e.read_status()
            }
        })
    }

    /// Write one of the two ports as the guest would.
    pub fn write_port(&self, offset: u64, value: u8) {
        self.shared.with_engine(|e| {
            if offset & 1 == 0 {
                e.write_data(value);
            } else {
                e.write_control(value);
            }
        });
    }

    /// Look at the six-bit `--BBGGRR` framebuffer.
    ///
    /// It is always [`SCREEN_WIDTH`] x [`SCREEN_HEIGHT`]; a mode with fewer
    /// active lines leaves the tail of it holding whatever was there, and the
    /// scanout adapter crops to [`active_height`](SmsVdp::active_height).
    pub fn with_framebuffer<R>(&self, f: impl FnOnce(&[u8]) -> R) -> R {
        f(&self.shared.engine.lock().fb)
    }

    /// One pixel of the framebuffer, or `None` when it is off the screen.
    #[must_use]
    pub fn pixel(&self, x: usize, y: usize) -> Option<u8> {
        (x < SCREEN_WIDTH && y < SCREEN_HEIGHT)
            .then(|| self.shared.engine.lock().fb[y * SCREEN_WIDTH + x])
    }

    /// Run the chip to absolute tick `target`.
    pub fn advance_to(&self, target: u64) {
        self.shared.with_engine(|e| e.advance_to(target));
    }

    /// Run the chip forward by `dots` pixels.
    pub fn advance_by(&self, dots: u64) {
        let target = self.shared.dots.load(Ordering::Relaxed) + dots;
        self.advance_to(target);
    }
}

// ---------------------------------------------------------------------------
// The apertures
// ---------------------------------------------------------------------------

/// `$BE` and `$BF`.
struct VdpPorts {
    shared: Arc<Shared>,
}

impl fmt::Debug for VdpPorts {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("VdpPorts").finish_non_exhaustive()
    }
}

impl MemOps for VdpPorts {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        self.shared.sync(attrs);
        if attrs.debug {
            // Every read here has a side effect on real silicon, which is
            // exactly why a debugger must take the other path: no flag cleared,
            // no address advanced, no latch reset.
            let engine = self.shared.engine.lock();
            *byte = if offset & 1 == 0 {
                engine.buffer
            } else {
                engine.peek_status()
            };
            return Ok(());
        }
        *byte = self.shared.with_engine(|e| {
            if offset & 1 == 0 {
                e.read_data()
            } else {
                e.read_status()
            }
        });
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug *write* would be a guest action with no guest behind it.
            // Doing nothing beats corrupting the command latch.
            return Ok(());
        }
        self.shared.sync(attrs);
        self.shared.with_engine(|e| {
            if offset & 1 == 0 {
                e.write_data(*value);
            } else {
                e.write_control(*value);
            }
        });
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// `$7E` and `$7F`, read-only.
struct VdpCounters {
    shared: Arc<Shared>,
}

impl fmt::Debug for VdpCounters {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("VdpCounters").finish_non_exhaustive()
    }
}

impl MemOps for VdpCounters {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // Reading a counter has no side effect, so a debug read needs no special
        // case — but it still wants the chip caught up, or a monitor would
        // report the line the last guest access left it on.
        self.shared.sync(attrs);
        let engine = self.shared.engine.lock();
        *byte = if offset & 1 == 0 {
            engine.vcounter()
        } else {
            engine.hcounter()
        };
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let [_] = src else {
            return Err(BusError::BadAccess);
        };
        // Nothing drives these lines back. A board maps the write side of the
        // same addresses to the sound chip with `split()`, so this is only ever
        // reached when it did not.
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

/// The `sms.vdp` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "sms.vdp",
    version: 1,
    summary: "Sega 315-5124 VDP: mode 4 and the TMS9918A modes, 16 KiB VRAM, 32 colours",
    properties: &[PropertySpec {
        name: "region",
        kind: ValueKind::Str,
        required: false,
        summary: "television standard: `ntsc` (262 lines) or `pal` (313)",
    }],
    construct: |props| Ok(Box::new(SmsVdp::from_props(props)?) as Box<dyn Device>),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for SmsVdp {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The realize sweep: announce what the line idles at rather than leaving
        // a pin at whatever a constructor assumed (`ROADMAP.md` §4.3).
        let level = self.shared.engine.lock().irq_line();
        self.shared.drive(level);
        Ok(())
    }

    fn unrealize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        self.shared.drive(false);
        *self.shared.irq.lock() = None;
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            PORT_REGION => Some(Arc::clone(&self.port_region)),
            COUNTER_REGION => Some(Arc::clone(&self.counter_region)),
            // No empty-name default: two apertures, and a `map … = vdp` that
            // silently picked one would be a coin toss.
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!("the VDP drives only `{IRQ_PIN}`"),
            });
        }
        self.attach_irq(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == IRQ_PIN {
            let level = self.shared.engine.lock().irq_line();
            self.shared.drive(level);
        }
    }

    fn reset(&self, _kind: ResetKind) {
        let region = self.tv_region();
        self.shared.with_engine(|e| {
            // The device's own clock does **not** restart. The scheduler owns
            // it — `Machine::reset` does not rewind the clock domains — so a
            // device that zeroed its tick would then be told to advance to
            // wherever the scheduler already was, and would replay every dot in
            // between. Everything the guest can see goes back; the absolute
            // counter carries on.
            let dots = e.dots;
            *e = Engine::new(region);
            e.dots = dots;
        });
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let engine = self.shared.engine.lock();
        w.write_u32(STATE_VERSION)?;
        w.write_bytes(&engine.vram)?;
        w.write_bytes(&engine.cram)?;
        w.write_bytes(&engine.regs)?;
        w.write_u16(engine.addr)?;
        w.write_u8(engine.code)?;
        w.write_bool(engine.latch)?;
        w.write_u8(engine.first)?;
        w.write_u8(engine.buffer)?;
        w.write_u8(engine.status)?;
        w.write_u8(engine.line_counter)?;
        w.write_bool(engine.line_irq)?;
        w.write_u8(engine.vscroll_latch)?;
        w.write_u64(engine.dot)?;
        w.write_u16(engine.line)?;
        w.write_u64(engine.dots)?;
        w.write_u64(engine.frame)?;
        // The framebuffer is not derived state: a snapshot taken mid-frame and
        // restored must show the lines already drawn, not a blank screen.
        w.write_bytes(&engine.fb)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let version = r.read_u32()?;
        if version != STATE_VERSION {
            return Err(Error::State(alloc::format!(
                "the VDP's snapshot is version {version}, this build writes {STATE_VERSION}"
            )));
        }
        let vram = r.read_bytes()?;
        let cram = r.read_bytes()?;
        let regs = r.read_bytes()?;
        let addr = r.read_u16()?;
        let code = r.read_u8()?;
        let latch = r.read_bool()?;
        let first = r.read_u8()?;
        let buffer = r.read_u8()?;
        let status = r.read_u8()?;
        let line_counter = r.read_u8()?;
        let line_irq = r.read_bool()?;
        let vscroll_latch = r.read_u8()?;
        let dot = r.read_u64()?;
        let line = r.read_u16()?;
        let dots = r.read_u64()?;
        let frame = r.read_u64()?;
        let fb = r.read_bytes()?;
        if vram.len() != VRAM_LEN
            || cram.len() != CRAM_LEN
            || regs.len() != REGISTER_COUNT
            || fb.len() != FRAMEBUFFER_LEN
        {
            return Err(Error::State(String::from(
                "the VDP's snapshot has the wrong memory sizes",
            )));
        }
        let irq = {
            let mut engine = self.shared.engine.lock();
            engine.vram.copy_from_slice(vram);
            engine.cram.copy_from_slice(cram);
            engine.regs.copy_from_slice(regs);
            engine.addr = addr;
            engine.code = code;
            engine.latch = latch;
            engine.first = first;
            engine.buffer = buffer;
            engine.status = status;
            engine.line_counter = line_counter;
            engine.line_irq = line_irq;
            engine.vscroll_latch = vscroll_latch;
            engine.dot = dot;
            engine.line = line;
            engine.dots = dots;
            engine.frame = frame;
            engine.fb.copy_from_slice(fb);
            // The tick and the next boundary are derived from the snapshot and
            // must follow it, or the scheduler runs the chip from the wrong dot.
            self.shared.publish(&engine);
            engine.irq_line()
        };
        self.shared.drive(irq);
        Ok(())
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) --------------------------------

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.dots.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        SmsVdp::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        Some(self.shared.next_event.load(Ordering::Relaxed))
    }

    /// **True**, and it costs a catch-up per CPU cycle.
    ///
    /// This chip drives `/INT` and the Z80 samples that pin at the end of every
    /// instruction. Being caught up only when the ports are touched would move
    /// an interrupt by up to a quantum, which is visible: a game polling the V
    /// counter for a raster split would watch the line it wanted go past.
    fn sampled_every_cycle(&self) -> bool {
        true
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        SmsVdp::attach_lazy(self, handle);
    }
}

impl crate::machine::Instance for SmsVdp {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(SmsVdp::from_props(props)?)))
}

/// What the validator should know about `sms.vdp`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("region", ValueKind::Str).values(&["ntsc", "pal"]))
        .port(IRQ_PIN, PortDir::Out)
        .region(PORT_REGION)
        .region(COUNTER_REGION)
}
