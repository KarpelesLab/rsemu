//! An MC6845-derived CRTC, the character buffer, the character generator that
//! turns the two into pixels, and the VGA register file firmware programs on
//! its way to a text mode.
//!
//! # Sources
//!
//! * **Motorola MC6845 CRT Controller data sheet** — the eighteen registers and
//!   what each counts: R0/R1 the horizontal total and displayed *in
//!   characters*, R4/R5 the vertical total in character rows plus a scan-line
//!   adjust, R6 the vertical displayed, R7 the vertical sync position, R9 the
//!   maximum raster address (so the cell height), R10/R11 the cursor's start
//!   and end raster with the two blink-mode bits, R12/R13 the start address and
//!   R14/R15 the cursor address as 14-bit values in a 6-bit/8-bit pair, and
//!   R16/R17 the read-only light pen. Also the rule this file leans on twice:
//!   **R0-R11 are write-only and R12-R17 read**, and the vertical sync pulse is
//!   a fixed sixteen raster lines wide.
//! * **IBM Personal Computer Technical Reference**, the colour/graphics adapter
//!   section — the 6845 at 0x3d4/0x3d5, the mode control at 0x3d8 (bit 3 enables
//!   the video signal, bit 5 makes attribute bit 7 a blink rather than a bright
//!   background), the colour select at 0x3d9, the status register at 0x3da with
//!   its display-enable bit 0 and vertical-sync bit 3, the two-byte
//!   character/attribute cell, and the sixteen colours.
//! * **The OSDev wiki's VGA material** — the register file at 0x3c0-0x3cf: the
//!   attribute controller's single port with the index/data flip-flop that a
//!   read of 0x3da resets, the miscellaneous output at 0x3c2 (bit 0 picks the
//!   colour or monochrome address pair, bits 2-3 the dot clock), the sequencer,
//!   the 256x18-bit DAC, and the graphics controller.
//!
//! **No emulator source was consulted for any of it** (`CLAUDE.md`,
//! provenance). The font below is original — see `FONT_ASCII`.
//!
//! # Scope: text mode, deliberately
//!
//! This models **an 80x25 colour text mode** and nothing else: enough to watch
//! firmware talk and to use a DOS prompt. There is no graphics mode, no planar
//! memory, no latch/ALU path through the graphics controller, no CPU-visible
//! DAC-driven pixel pipeline. A mode nothing yet asks for would be untested
//! code, and this file is expected to grow: the character generator, the CRTC
//! timing and the scanout seam are all in place for a graphics mode to be added
//! *with* the guest that exercises it.
//!
//! The register files that exist but drive nothing yet — the sequencer, the
//! graphics controller, most of the attribute controller — are modelled as
//! honest latches, because firmware writes them all before it writes a single
//! character and a device that faulted on any of them would never get as far as
//! showing one.
//!
//! # The windows a machine file maps
//!
//! ```text
//!   crtc         2 bytes   0x3d4/0x3d5 (colour) or 0x3b4/0x3b5 (monochrome)
//!   status       1 byte    0x3da: display enable, vertical sync
//!   mode         2 bytes   0x3d8 mode control, 0x3d9 colour select
//!   vga         16 bytes   0x3c0-0x3cf: attribute, misc, sequencer, DAC, GC
//!   vram        32 KiB     RAM, mapped at 0xb8000
//! ```
//!
//! `crtc` answers at whichever pair the board decodes it at, because a board
//! that fits one adapter has one CRTC. A machine file that wants the
//! miscellaneous output's bit 0 to *choose* between the two pairs maps
//! `crtc-colour` and `crtc-mono` instead, and each answers only when that bit
//! selects it.
//!
//! # Time
//!
//! The device's clock domain is the **character clock** — the dot clock divided
//! by the width of a character cell — because every horizontal number the 6845
//! holds is counted in characters, so a tick is one increment of the horizontal
//! counter and every timing question below is integer division. The chip is
//! *lazily advanced* (`ROADMAP.md` §4.2): it holds its own tick, and a read of
//! the status register catches it up first, so a program polling 0x3da for
//! retrace sees the bit at the instant of its own `in` instruction rather than
//! at the end of the scheduler's quantum. Nothing here reads a host clock and
//! nothing here uses a float.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, RamStore, Region, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::host::display::{PixelFormat, Scanout, Surface, SurfaceInfo};
use crate::machine::realize::Instance;
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.video";

/// The snapshot chunk version. Bump with the encoding, never on its own.
///
/// v2 added the character buffer. v1 chunks left it out on the belief that the
/// machine snapshots RAM regions by itself; it does not — [`Machine::save`]
/// walks *devices*, and this adapter's `vram` is its own store rather than a
/// `ram` instance, so 32 KiB of guest-writable memory went missing on restore.
///
/// [`Machine::save`]: crate::machine::Machine::save
const STATE_VERSION: u32 = 2;

/// The CRTC's address and data registers: two bytes.
pub const CRTC_WINDOW_LEN: u64 = 2;

/// The status register: one byte.
pub const STATUS_WINDOW_LEN: u64 = 1;

/// The CGA mode control and colour select registers: two bytes.
pub const MODE_WINDOW_LEN: u64 = 2;

/// The VGA register file at 0x3c0-0x3cf: sixteen bytes.
pub const VGA_WINDOW_LEN: u64 = 16;

/// The character buffer: 32 KiB, which is the colour adapter's four text pages.
pub const VRAM_LEN: u64 = 32 * 1024;

/// How many registers an MC6845 has.
const CRTC_REGISTERS: usize = 18;

/// Attribute controller registers: sixteen palette entries and five others.
const ATTR_REGISTERS: usize = 21;

/// Sequencer registers: reset, clocking mode, map mask, character map select
/// and memory mode.
const SEQ_REGISTERS: usize = 5;

/// Graphics controller registers.
const GC_REGISTERS: usize = 9;

/// The width of the DAC's index, and so how many colours it holds.
const DAC_ENTRIES: usize = 256;

/// Components per DAC entry: red, green, blue, read and written in that order
/// through the single data port at 0x3c9.
const DAC_COMPONENTS: usize = 3;

/// The sentinel [`Shared::next_event`] holds when there is nothing pending.
const NO_EVENT: u64 = u64::MAX;

/// How many raster lines the vertical sync pulse lasts.
///
/// Fixed on the MC6845 — the data sheet gives no register for it, and the
/// variants that added one (the 6845-1, the HD6845S) put it in R3's high
/// nibble, which this model stores and ignores.
const VSYNC_LINES: u64 = 16;

// -- the CGA mode control register (0x3d8) ----------------------------------

/// Bit 3: enable the video signal. Clear blanks the screen.
const MODE_VIDEO_ENABLE: u8 = 0x08;
/// Bit 5: attribute bit 7 is blink rather than a bright background.
const MODE_BLINK: u8 = 0x20;

// -- the status register (0x3da) --------------------------------------------

/// Bit 0: *display enable* — set while the beam is in horizontal or vertical
/// blanking, which is when the guest may touch the buffer without snow.
const STATUS_DISPLAY_ENABLE: u8 = 0x01;
/// Bit 3: set for the whole vertical sync pulse.
const STATUS_VSYNC: u8 = 0x08;

// -- the miscellaneous output register (0x3c2) ------------------------------

/// Bit 0: set selects the colour address pair (0x3d4/0x3da), clear the
/// monochrome one (0x3b4/0x3ba).
const MISC_COLOUR: u8 = 0x01;

/// The 25.175 MHz crystal, which drives the 640-pixel-wide modes.
const DOT_CLOCK_25MHZ: u64 = 25_175_000;
/// The 28.322 MHz crystal, which drives the 720-pixel-wide text mode.
const DOT_CLOCK_28MHZ: u64 = 28_322_000;

/// What a write to each CRTC register keeps, from the data sheet's register
/// table: several of the eighteen are narrower than a byte, and software that
/// reads R12-R15 back expects the bits it could not set to be zero.
const CRTC_WRITE_MASK: [u8; CRTC_REGISTERS] = [
    0xff, // R0  horizontal total, in characters - 1
    0xff, // R1  horizontal displayed
    0xff, // R2  horizontal sync position
    0xff, // R3  sync widths: horizontal in the low nibble
    0x7f, // R4  vertical total, in character rows - 1
    0x1f, // R5  vertical total adjust, in raster lines
    0x7f, // R6  vertical displayed
    0x7f, // R7  vertical sync position
    0x03, // R8  interlace and skew
    0x1f, // R9  maximum raster address
    0x7f, // R10 cursor start raster, plus the two blink-mode bits
    0x1f, // R11 cursor end raster
    0x3f, // R12 start address, high
    0xff, // R13 start address, low
    0x3f, // R14 cursor address, high
    0xff, // R15 cursor address, low
    0x00, // R16 light pen, high: read-only
    0x00, // R17 light pen, low: read-only
];

/// The register values an 80x25 text mode with a 16-line cell needs, which is
/// what this device comes up in.
///
/// The 6845's own registers are undefined out of reset — the data sheet says
/// so, and real firmware programs all eighteen before enabling the video — but
/// a device whose default geometry is 0x0 has no defined picture at all, which
/// makes every test and every screenshot depend on boot order. So it starts
/// where firmware puts it: 100 characters by 449 lines of 16, of which 80x25
/// are displayed, and the sync pulse begins the line after the last displayed
/// one. That is the 720x400 70 Hz text mode of every VGA since 1987.
const CRTC_DEFAULTS: [u8; CRTC_REGISTERS] = [
    99,   // R0  100 characters per line
    80,   // R1  80 displayed
    82,   // R2  sync begins two characters after the last displayed one
    0x0f, // R3  sync widths
    27,   // R4  28 character rows
    1,    // R5  plus one raster line: 28 * 16 + 1 = 449
    25,   // R6  25 displayed
    25,   // R7  sync begins at row 25, i.e. raster line 400
    0,    // R8  no interlace
    15,   // R9  a 16-line cell
    14,   // R10 an underline cursor: the last two lines of the cell
    15,   // R11
    0, 0, // R12/R13 start address 0
    0, 0, // R14/R15 cursor at the top left
    0, 0, // R16/R17 light pen
];

// ---------------------------------------------------------------------------
// The character generator
// ---------------------------------------------------------------------------

/// The number of raster lines the font is drawn for.
const FONT_CELL_HEIGHT: usize = 16;

/// The first code point `FONT_ASCII` covers.
const FONT_FIRST: u8 = 0x20;

/// **An original 8x16 font, written for rsemu.**
///
/// It is *not* IBM's, not any ROM's, and not lifted from another emulator — the
/// letterforms were drawn here as 5x7 art and placed in the cell at column 1,
/// raster 4, with two more rasters below the baseline for descenders. Copying a
/// font out of a copyleft project would taint the crate as surely as copying
/// its code (`CLAUDE.md`, provenance), and a font ROM of unknown provenance is
/// somebody's copyrighted bitmap.
///
/// A machine that wants the *real* thing can be given a font ROM later: this
/// table is behind [`glyph`], which is the only thing that would change.
///
/// One entry per code point from `0x20` to `0x7e`, sixteen bytes each, the most
/// significant bit leftmost.
static FONT_ASCII: [[u8; FONT_CELL_HEIGHT]; 95] = [
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // ' '
    [
        0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '!'
    [
        0x00, 0x00, 0x00, 0x00, 0x28, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '"'
    [
        0x00, 0x00, 0x00, 0x00, 0x28, 0x28, 0x7c, 0x28, 0x7c, 0x28, 0x28, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '#'
    [
        0x00, 0x00, 0x00, 0x00, 0x10, 0x3c, 0x50, 0x38, 0x14, 0x78, 0x10, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '$'
    [
        0x00, 0x00, 0x00, 0x00, 0x60, 0x64, 0x08, 0x10, 0x20, 0x4c, 0x0c, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '%'
    [
        0x00, 0x00, 0x00, 0x00, 0x30, 0x48, 0x50, 0x20, 0x54, 0x48, 0x34, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '&'
    [
        0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // "'"
    [
        0x00, 0x00, 0x00, 0x00, 0x08, 0x10, 0x20, 0x20, 0x20, 0x10, 0x08, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '('
    [
        0x00, 0x00, 0x00, 0x00, 0x20, 0x10, 0x08, 0x08, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // ')'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x38, 0x7c, 0x38, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '*'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x7c, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '+'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x10, 0x20, 0x00, 0x00,
        0x00,
    ], // ','
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '-'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '.'
    [
        0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x08, 0x10, 0x20, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '/'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x4c, 0x54, 0x64, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '0'
    [
        0x00, 0x00, 0x00, 0x00, 0x10, 0x30, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '1'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x04, 0x08, 0x10, 0x20, 0x7c, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '2'
    [
        0x00, 0x00, 0x00, 0x00, 0x7c, 0x08, 0x10, 0x08, 0x04, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '3'
    [
        0x00, 0x00, 0x00, 0x00, 0x08, 0x18, 0x28, 0x48, 0x7c, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '4'
    [
        0x00, 0x00, 0x00, 0x00, 0x7c, 0x40, 0x78, 0x04, 0x04, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '5'
    [
        0x00, 0x00, 0x00, 0x00, 0x18, 0x20, 0x40, 0x78, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '6'
    [
        0x00, 0x00, 0x00, 0x00, 0x7c, 0x04, 0x08, 0x10, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '7'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x44, 0x38, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '8'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x44, 0x3c, 0x04, 0x08, 0x30, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '9'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // ':'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x30, 0x00, 0x30, 0x30, 0x00, 0x10, 0x20, 0x00, 0x00,
        0x00,
    ], // ';'
    [
        0x00, 0x00, 0x00, 0x00, 0x08, 0x10, 0x20, 0x40, 0x20, 0x10, 0x08, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '<'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '='
    [
        0x00, 0x00, 0x00, 0x00, 0x20, 0x10, 0x08, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '>'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x04, 0x08, 0x10, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '?'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x5c, 0x54, 0x5c, 0x40, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '@'
    [
        0x00, 0x00, 0x00, 0x00, 0x10, 0x28, 0x44, 0x44, 0x7c, 0x44, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'A'
    [
        0x00, 0x00, 0x00, 0x00, 0x78, 0x44, 0x44, 0x78, 0x44, 0x44, 0x78, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'B'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x40, 0x40, 0x40, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'C'
    [
        0x00, 0x00, 0x00, 0x00, 0x70, 0x48, 0x44, 0x44, 0x44, 0x48, 0x70, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'D'
    [
        0x00, 0x00, 0x00, 0x00, 0x7c, 0x40, 0x40, 0x78, 0x40, 0x40, 0x7c, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'E'
    [
        0x00, 0x00, 0x00, 0x00, 0x7c, 0x40, 0x40, 0x78, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'F'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x40, 0x5c, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'G'
    [
        0x00, 0x00, 0x00, 0x00, 0x44, 0x44, 0x44, 0x7c, 0x44, 0x44, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'H'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x10, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'I'
    [
        0x00, 0x00, 0x00, 0x00, 0x1c, 0x08, 0x08, 0x08, 0x08, 0x48, 0x30, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'J'
    [
        0x00, 0x00, 0x00, 0x00, 0x44, 0x48, 0x50, 0x60, 0x50, 0x48, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'K'
    [
        0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x7c, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'L'
    [
        0x00, 0x00, 0x00, 0x00, 0x44, 0x6c, 0x54, 0x54, 0x44, 0x44, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'M'
    [
        0x00, 0x00, 0x00, 0x00, 0x44, 0x44, 0x64, 0x54, 0x4c, 0x44, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'N'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x44, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'O'
    [
        0x00, 0x00, 0x00, 0x00, 0x78, 0x44, 0x44, 0x78, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'P'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x44, 0x44, 0x54, 0x48, 0x34, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'Q'
    [
        0x00, 0x00, 0x00, 0x00, 0x78, 0x44, 0x44, 0x78, 0x50, 0x48, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'R'
    [
        0x00, 0x00, 0x00, 0x00, 0x3c, 0x40, 0x40, 0x38, 0x04, 0x04, 0x78, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'S'
    [
        0x00, 0x00, 0x00, 0x00, 0x7c, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'T'
    [
        0x00, 0x00, 0x00, 0x00, 0x44, 0x44, 0x44, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'U'
    [
        0x00, 0x00, 0x00, 0x00, 0x44, 0x44, 0x44, 0x44, 0x44, 0x28, 0x10, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'V'
    [
        0x00, 0x00, 0x00, 0x00, 0x44, 0x44, 0x44, 0x54, 0x54, 0x6c, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'W'
    [
        0x00, 0x00, 0x00, 0x00, 0x44, 0x44, 0x28, 0x10, 0x28, 0x44, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'X'
    [
        0x00, 0x00, 0x00, 0x00, 0x44, 0x44, 0x28, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'Y'
    [
        0x00, 0x00, 0x00, 0x00, 0x7c, 0x04, 0x08, 0x10, 0x20, 0x40, 0x7c, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'Z'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x20, 0x20, 0x20, 0x20, 0x20, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '['
    [
        0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x20, 0x10, 0x08, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '\\'
    [
        0x00, 0x00, 0x00, 0x00, 0x38, 0x08, 0x08, 0x08, 0x08, 0x08, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // ']'
    [
        0x00, 0x00, 0x00, 0x00, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '^'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '_'
    [
        0x00, 0x00, 0x00, 0x00, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '`'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x04, 0x3c, 0x44, 0x3c, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'a'
    [
        0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x78, 0x44, 0x44, 0x44, 0x78, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'b'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x40, 0x40, 0x40, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'c'
    [
        0x00, 0x00, 0x00, 0x00, 0x04, 0x04, 0x3c, 0x44, 0x44, 0x44, 0x3c, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'd'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x7c, 0x40, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'e'
    [
        0x00, 0x00, 0x00, 0x00, 0x18, 0x20, 0x20, 0x78, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'f'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x44, 0x44, 0x3c, 0x04, 0x44, 0x38, 0x00, 0x00,
        0x00,
    ], // 'g'
    [
        0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x78, 0x44, 0x44, 0x44, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'h'
    [
        0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x30, 0x10, 0x10, 0x10, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'i'
    [
        0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x18, 0x08, 0x08, 0x08, 0x08, 0x48, 0x30, 0x00, 0x00,
        0x00,
    ], // 'j'
    [
        0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x48, 0x50, 0x60, 0x50, 0x48, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'k'
    [
        0x00, 0x00, 0x00, 0x00, 0x30, 0x10, 0x10, 0x10, 0x10, 0x10, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'l'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x54, 0x54, 0x54, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'm'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x44, 0x44, 0x44, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'n'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'o'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x44, 0x44, 0x78, 0x40, 0x40, 0x40, 0x00, 0x00,
        0x00,
    ], // 'p'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x44, 0x44, 0x3c, 0x04, 0x04, 0x04, 0x00, 0x00,
        0x00,
    ], // 'q'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x60, 0x40, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'r'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x40, 0x38, 0x04, 0x78, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 's'
    [
        0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x78, 0x20, 0x20, 0x24, 0x18, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 't'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x44, 0x44, 0x4c, 0x34, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'u'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x44, 0x44, 0x28, 0x10, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'v'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x54, 0x54, 0x54, 0x28, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'w'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x28, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'x'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x44, 0x44, 0x3c, 0x04, 0x44, 0x38, 0x00, 0x00,
        0x00,
    ], // 'y'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x08, 0x10, 0x20, 0x7c, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // 'z'
    [
        0x00, 0x00, 0x00, 0x00, 0x0c, 0x10, 0x10, 0x20, 0x10, 0x10, 0x0c, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '{'
    [
        0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '|'
    [
        0x00, 0x00, 0x00, 0x00, 0x60, 0x10, 0x10, 0x08, 0x10, 0x10, 0x60, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '}'
    [
        0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x4c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
    ], // '~'
];

/// The code points outside ASCII that a DOS program actually draws with: the
/// single and double box-drawing set, the shaded and half blocks, and a handful
/// of arrows. Sorted by code point, because [`glyph`] binary-searches it.
///
/// These are full-cell glyphs rather than 5x7 art — a frame only looks like a
/// frame if the strokes meet the cell edges and join up with the neighbouring
/// cell's.
static FONT_EXTRA: &[(u8, [u8; FONT_CELL_HEIGHT])] = &[
    (
        0x07,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x38, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0x10,
        [
            0x00, 0x00, 0x00, 0x00, 0x40, 0x60, 0x70, 0x78, 0x70, 0x60, 0x40, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0x11,
        [
            0x00, 0x00, 0x00, 0x00, 0x04, 0x0c, 0x1c, 0x3c, 0x1c, 0x0c, 0x04, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0x18,
        [
            0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x54, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0x19,
        [
            0x00, 0x00, 0x00, 0x00, 0x10, 0x10, 0x10, 0x10, 0x54, 0x38, 0x10, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0x1a,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x08, 0x7c, 0x08, 0x10, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0x1b,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x7c, 0x20, 0x10, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0xb0,
        [
            0x88, 0x00, 0x22, 0x00, 0x88, 0x00, 0x22, 0x00, 0x88, 0x00, 0x22, 0x00, 0x88, 0x00,
            0x22, 0x00,
        ],
    ),
    (
        0xb1,
        [
            0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55,
            0xaa, 0x55,
        ],
    ),
    (
        0xb2,
        [
            0xff, 0xaa, 0xff, 0xaa, 0xff, 0xaa, 0xff, 0xaa, 0xff, 0xaa, 0xff, 0xaa, 0xff, 0xaa,
            0xff, 0xaa,
        ],
    ),
    (
        0xb3,
        [
            0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
            0x10, 0x10,
        ],
    ),
    (
        0xb4,
        [
            0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xf0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
            0x10, 0x10,
        ],
    ),
    (
        0xb9,
        [
            0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0xf8, 0x20, 0xf8, 0x28, 0x28, 0x28, 0x28, 0x28,
            0x28, 0x28,
        ],
    ),
    (
        0xba,
        [
            0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x28,
            0x28, 0x28,
        ],
    ),
    (
        0xbb,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x20, 0xe8, 0x28, 0x28, 0x28, 0x28, 0x28,
            0x28, 0x28,
        ],
    ),
    (
        0xbc,
        [
            0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0xf8, 0x20, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0xbf,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
            0x10, 0x10,
        ],
    ),
    (
        0xc0,
        [
            0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0xc1,
        [
            0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0xc2,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
            0x10, 0x10,
        ],
    ),
    (
        0xc3,
        [
            0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1f, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
            0x10, 0x10,
        ],
    ),
    (
        0xc4,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0xc5,
        [
            0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xff, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
            0x10, 0x10,
        ],
    ),
    (
        0xc8,
        [
            0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x3f, 0x20, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0xc9,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x20, 0x2f, 0x28, 0x28, 0x28, 0x28, 0x28,
            0x28, 0x28,
        ],
    ),
    (
        0xca,
        [
            0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0xef, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0xcb,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xef, 0x28, 0x28, 0x28, 0x28, 0x28,
            0x28, 0x28,
        ],
    ),
    (
        0xcc,
        [
            0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0x3f, 0x08, 0x3f, 0x28, 0x28, 0x28, 0x28, 0x28,
            0x28, 0x28,
        ],
    ),
    (
        0xcd,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0xce,
        [
            0x28, 0x28, 0x28, 0x28, 0x28, 0x28, 0xef, 0x00, 0xef, 0x28, 0x28, 0x28, 0x28, 0x28,
            0x28, 0x28,
        ],
    ),
    (
        0xd9,
        [
            0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
    (
        0xda,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
            0x10, 0x10,
        ],
    ),
    (
        0xdb,
        [
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff,
        ],
    ),
    (
        0xdc,
        [
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff,
        ],
    ),
    (
        0xdd,
        [
            0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0,
            0xf0, 0xf0,
        ],
    ),
    (
        0xde,
        [
            0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,
            0x0f, 0x0f,
        ],
    ),
    (
        0xdf,
        [
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00,
        ],
    ),
];

/// What a code point with no glyph is drawn as: a hollow box, so that text
/// drawn in a character set this font does not cover is *visible* as missing
/// rather than silently blank.
static FONT_MISSING: [u8; FONT_CELL_HEIGHT] = [
    0x00, 0x00, 0x00, 0x00, 0x7c, 0x44, 0x44, 0x44, 0x44, 0x44, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00,
];

/// Nothing at all: the control codes, which a text mode leaves blank here.
static FONT_BLANK: [u8; FONT_CELL_HEIGHT] = [0; FONT_CELL_HEIGHT];

/// The sixteen raster lines of `code`.
#[must_use]
fn glyph(code: u8) -> &'static [u8; FONT_CELL_HEIGHT] {
    if (FONT_FIRST..0x7f).contains(&code) {
        return &FONT_ASCII[(code - FONT_FIRST) as usize];
    }
    match FONT_EXTRA.binary_search_by_key(&code, |(c, _)| *c) {
        Ok(i) => &FONT_EXTRA[i].1,
        // A control code is blank; a printable one this font does not reach is
        // a box, which is the difference between "nothing was written here" and
        // "something was, and we cannot draw it".
        Err(_) if code < FONT_FIRST || code == 0x7f => &FONT_BLANK,
        Err(_) => &FONT_MISSING,
    }
}

/// The sixteen colours of the colour/graphics adapter, in the DAC's six bits
/// per component.
///
/// These are the documented CGA colours — the three-level `0x00/0xAA/0xFF`
/// scheme with `0xAA5500` brown in place of dark yellow — written as the values
/// firmware loads into a VGA's DAC, so that six bits scaled back up by
/// [`expand6`] give exactly `0x00`, `0x55`, `0xAA` and `0xFF` again.
///
/// They are the *default* contents of the DAC rather than a hard-wired table:
/// once firmware programs the palette, what the guest sees is what the guest
/// asked for, which is the whole reason the DAC is modelled.
const CGA_PALETTE_6BIT: [[u8; 3]; 16] = [
    [0x00, 0x00, 0x00], // black
    [0x00, 0x00, 0x2a], // blue
    [0x00, 0x2a, 0x00], // green
    [0x00, 0x2a, 0x2a], // cyan
    [0x2a, 0x00, 0x00], // red
    [0x2a, 0x00, 0x2a], // magenta
    [0x2a, 0x15, 0x00], // brown
    [0x2a, 0x2a, 0x2a], // light grey
    [0x15, 0x15, 0x15], // dark grey
    [0x15, 0x15, 0x3f], // light blue
    [0x15, 0x3f, 0x15], // light green
    [0x15, 0x3f, 0x3f], // light cyan
    [0x3f, 0x15, 0x15], // light red
    [0x3f, 0x15, 0x3f], // light magenta
    [0x3f, 0x3f, 0x15], // yellow
    [0x3f, 0x3f, 0x3f], // white
];

/// Six bits of DAC to eight bits of host colour.
///
/// `(v << 2) | (v >> 4)` rather than `v << 2`, so that full scale stays full
/// scale: 0x3f becomes 0xff and not 0xfc.
#[inline]
#[must_use]
const fn expand6(v: u8) -> u8 {
    (v << 2) | (v >> 4)
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// The VGA register file: index latches and the arrays behind them.
///
/// Everything here is a latch that reads back correctly. Only some of it
/// reaches the picture — the attribute palette, the DAC, the sequencer's
/// character width and screen-off bits — and the rest exists because firmware
/// writes it and a fault would end the boot.
#[derive(Debug, Clone)]
struct Vga {
    /// Miscellaneous output (0x3c2 write, 0x3cc read).
    misc: u8,
    /// The motherboard enable register at 0x3c3.
    enable: u8,
    /// The attribute controller's index, including bit 5, the palette address
    /// source — clear means "I am programming the palette, blank the screen".
    attr_index: u8,
    /// The attribute controller's flip-flop: `false` when the next write to
    /// 0x3c0 is an index, `true` when it is data. Reset to `false` by a read of
    /// the status register, which is the single most-quoted VGA gotcha and
    /// which firmware depends on to resynchronise.
    attr_data_next: bool,
    attr: [u8; ATTR_REGISTERS],
    seq_index: u8,
    seq: [u8; SEQ_REGISTERS],
    gc_index: u8,
    gc: [u8; GC_REGISTERS],
    /// The DAC pixel mask at 0x3c6, which reads 0xff until something writes it.
    dac_mask: u8,
    dac_read: u8,
    dac_write: u8,
    /// Which of the three components the next access to 0x3c9 is.
    dac_read_sub: u8,
    dac_write_sub: u8,
    /// Whether the last index written was the read one (0x3c7), which is what
    /// the DAC state register reports.
    dac_reading: bool,
    dac: [[u8; 3]; DAC_ENTRIES],
}

impl Vga {
    /// The state firmware would leave behind after setting an 80x25 colour text
    /// mode, for the same reason [`CRTC_DEFAULTS`] exists.
    fn new() -> Vga {
        let mut attr = [0u8; ATTR_REGISTERS];
        // The palette registers default to the identity, so an attribute's
        // colour number is a DAC index until firmware says otherwise.
        let mut i = 0;
        while i < 16 {
            attr[i] = i as u8;
            i += 1;
        }
        // Mode control: line graphics enable (bit 2) and blink (bit 3), which
        // is what the BIOS writes for a text mode; colour plane enable is all
        // four planes.
        attr[16] = 0x0c;
        attr[18] = 0x0f;
        let mut dac = [[0u8; 3]; DAC_ENTRIES];
        let mut i = 0;
        while i < CGA_PALETTE_6BIT.len() {
            dac[i] = CGA_PALETTE_6BIT[i];
            i += 1;
        }
        Vga {
            // Colour address pair, 28.322 MHz clock select, 400-line: 0x67 is
            // the value the BIOS writes for mode 3.
            misc: 0x67,
            enable: 0x01,
            // Palette address source set: the screen is on.
            attr_index: 0x20,
            attr_data_next: false,
            attr,
            seq_index: 0,
            // Clocking mode 0: a nine-pixel character cell, which is what the
            // 720x400 text mode uses.
            seq: [0x03, 0x00, 0x03, 0x00, 0x02],
            gc_index: 0,
            gc: [0; GC_REGISTERS],
            dac_mask: 0xff,
            dac_read: 0,
            dac_write: 0,
            dac_read_sub: 0,
            dac_write_sub: 0,
            dac_reading: false,
            dac,
        }
    }
}

/// Everything the guest can see or change.
#[derive(Debug, Clone)]
struct State {
    /// Character clocks simulated. The authoritative copy; the atomic mirrors
    /// it for the scheduler's lock-free question.
    ticks: u64,
    /// The tick the frame being displayed began on. Kept rather than derived,
    /// because reprogramming the timing must not rewrite the past.
    frame_start: u64,
    /// Frames completed since reset.
    frames: u64,
    /// The CRTC's address register.
    crtc_index: u8,
    crtc: [u8; CRTC_REGISTERS],
    /// The CGA mode control register at 0x3d8.
    mode: u8,
    /// The CGA colour select register at 0x3d9.
    colour: u8,
    vga: Vga,
}

impl State {
    fn new() -> State {
        State {
            ticks: 0,
            frame_start: 0,
            frames: 0,
            crtc_index: 0,
            crtc: CRTC_DEFAULTS,
            // Bit 0 (80 columns), bit 3 (video on) and bit 5 (blink), which is
            // what the BIOS writes at 0x3d8 for an 80x25 colour text mode.
            mode: 0x29,
            colour: 0x00,
            vga: Vga::new(),
        }
    }

    /// Raster lines in a character cell: R9 counts from zero.
    fn cell_height(&self) -> u64 {
        u64::from(self.crtc[9] & 0x1f) + 1
    }

    /// Character clocks in one scan line, R0 counting from zero.
    fn chars_per_line(&self) -> u64 {
        u64::from(self.crtc[0]) + 1
    }

    /// Scan lines in one frame: whole character rows plus R5's adjust.
    fn lines_per_frame(&self) -> u64 {
        (u64::from(self.crtc[4] & 0x7f) + 1) * self.cell_height() + u64::from(self.crtc[5] & 0x1f)
    }

    /// Character clocks in one frame — the period everything below is modulo.
    fn ticks_per_frame(&self) -> u64 {
        self.chars_per_line() * self.lines_per_frame()
    }

    /// Displayed character columns (R1).
    fn columns(&self) -> u64 {
        u64::from(self.crtc[1])
    }

    /// Displayed character rows (R6).
    fn rows(&self) -> u64 {
        u64::from(self.crtc[6] & 0x7f)
    }

    /// The first scan line of the vertical sync pulse (R7, in character rows).
    fn vsync_start_line(&self) -> u64 {
        u64::from(self.crtc[7] & 0x7f) * self.cell_height()
    }

    /// The 14-bit refresh address the top left character comes from (R12/R13).
    fn start_address(&self) -> u64 {
        (u64::from(self.crtc[12] & 0x3f) << 8) | u64::from(self.crtc[13])
    }

    /// The 14-bit address the cursor sits on (R14/R15).
    fn cursor_address(&self) -> u64 {
        (u64::from(self.crtc[14] & 0x3f) << 8) | u64::from(self.crtc[15])
    }

    /// How many dots wide a character cell is.
    ///
    /// Sequencer register 1 bit 0 selects an eight-dot cell; clear — the reset
    /// state, and what a VGA text mode uses — is nine.
    fn char_width(&self) -> u32 {
        if self.vga.seq[1] & 0x01 != 0 { 8 } else { 9 }
    }

    /// The pixel clock in hertz, from the miscellaneous output's clock select.
    ///
    /// Selections 2 and 3 are the external and reserved inputs, which have no
    /// defined frequency; they keep the 28.322 MHz one rather than reporting a
    /// period of zero and leaving a host with no rate at all.
    fn dot_clock_hz(&self, override_hz: u64) -> u64 {
        if override_hz != 0 {
            return override_hz;
        }
        match (self.vga.misc >> 2) & 0x03 {
            0 => DOT_CLOCK_25MHZ,
            _ => DOT_CLOCK_28MHZ,
        }
    }

    /// Where in the frame the beam is, in character clocks.
    fn position(&self) -> u64 {
        let per = self.ticks_per_frame();
        if per == 0 {
            return 0;
        }
        (self.ticks - self.frame_start) % per
    }

    /// Whether the beam is inside the vertical sync pulse.
    fn in_vsync(&self) -> bool {
        let per = self.ticks_per_frame();
        if per == 0 {
            return false;
        }
        let chars = self.chars_per_line();
        let start = (self.vsync_start_line() * chars).min(per);
        let end = (start + VSYNC_LINES * chars).min(per);
        let pos = self.position();
        pos >= start && pos < end
    }

    /// The status register as a read would produce it.
    ///
    /// Both bits are a function of the tick, which is what makes them honest: a
    /// program that polls in a tight loop sees them change because the device's
    /// clock moved, not because it was read.
    fn status(&self) -> u8 {
        let per = self.ticks_per_frame();
        if per == 0 {
            // Nothing is being scanned, so nothing retraces. A guest polling
            // for a retrace that will never come has misprogrammed the chip,
            // and pretending otherwise would hide that.
            return 0;
        }
        let chars = self.chars_per_line();
        let pos = self.position();
        let line = pos / chars;
        let column = pos % chars;
        let mut value = 0;
        if column >= self.columns() || line >= self.rows() * self.cell_height() {
            value |= STATUS_DISPLAY_ENABLE;
        }
        if self.in_vsync() {
            value |= STATUS_VSYNC;
        }
        value
    }

    /// The next tick at which something visible changes: the sync pulse's two
    /// edges and the end of the frame.
    ///
    /// Strictly greater than [`State::ticks`], as [`Device::next_event_tick`]
    /// requires, because `position()` is always less than one frame.
    fn next_event(&self) -> u64 {
        let per = self.ticks_per_frame();
        if per == 0 {
            return NO_EVENT;
        }
        let chars = self.chars_per_line();
        let start = (self.vsync_start_line() * chars).min(per);
        let end = (start + VSYNC_LINES * chars).min(per);
        let pos = self.position();
        for candidate in [start, end, per] {
            if candidate > pos {
                return self.ticks + (candidate - pos);
            }
        }
        self.ticks + per
    }

    /// Pull `frame_start` back within one frame of the present.
    ///
    /// Reprogramming the timing can leave the beam beyond the end of the frame
    /// it is notionally in. Renormalising keeps `position()` inside one period,
    /// which is what lets [`State::next_event`] promise a tick in the future.
    fn renormalize(&mut self) {
        let per = self.ticks_per_frame();
        if per == 0 {
            self.frame_start = self.ticks;
            return;
        }
        let elapsed = self.ticks.saturating_sub(self.frame_start);
        if elapsed >= per {
            self.frame_start += (elapsed / per) * per;
        }
    }

    /// Whether the video signal is enabled at all.
    ///
    /// Three separate switches turn it off, and firmware uses all three: the
    /// CGA mode register's bit 3, the sequencer's screen-off bit, and the
    /// attribute controller's palette address source — which is clear exactly
    /// while the palette is being programmed.
    fn video_enabled(&self) -> bool {
        self.mode & MODE_VIDEO_ENABLE != 0
            && self.vga.seq[1] & 0x20 == 0
            && self.vga.attr_index & 0x20 != 0
    }

    /// Whether attribute bit 7 means blink rather than a bright background.
    ///
    /// Either register says so: the CGA's mode control bit 5 for firmware that
    /// programs the adapter as a CGA, and the attribute controller's mode
    /// control bit 3 for firmware that only touches the VGA side.
    fn blink_enabled(&self) -> bool {
        self.mode & MODE_BLINK != 0 || self.vga.attr[16] & 0x08 != 0
    }

    /// An attribute's colour number, through the attribute controller's palette
    /// and the colour select register, as a DAC index.
    ///
    /// The chain is the VGA's: the palette register supplies the low four bits
    /// and, unless mode control bit 7 says otherwise, bits 4-5 as well; the
    /// colour select register supplies bits 6-7 always. The DAC's pixel mask is
    /// applied last.
    fn dac_index(&self, colour: u8) -> u8 {
        let p = self.vga.attr[(colour & 0x0f) as usize] & 0x3f;
        let high = if self.vga.attr[16] & 0x80 != 0 {
            self.vga.attr[20] & 0x03
        } else {
            (p >> 4) & 0x03
        };
        let index = (p & 0x0f) | (high << 4) | ((self.vga.attr[20] & 0x0c) << 4);
        index & self.vga.dac_mask
    }

    /// One DAC entry as host RGB.
    fn rgb(&self, colour: u8) -> [u8; 3] {
        let entry = self.vga.dac[self.dac_index(colour) as usize];
        [expand6(entry[0]), expand6(entry[1]), expand6(entry[2])]
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    /// The character buffer, addressed by byte offset like all guest RAM.
    vram: Arc<RamStore>,
    /// Character clocks simulated, published for the scheduler's lock-free
    /// question — [`Device::current_tick`] runs under the scheduler's own slot
    /// lock and may not take one of ours.
    ticks: AtomicU64,
    /// Frames completed, published so a host can poll it without a lock.
    frames: AtomicU64,
    /// The tick of the next sync edge, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// A pixel clock a machine file names explicitly, overriding the clock
    /// select bits; `0` to follow them.
    dot_clock_hz: u64,
    /// The catch-up handle the status register syncs through.
    lazy: Mutex<Option<LazyHandle>>,
    /// The vertical sync output, if a machine wired one.
    vsync: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("dot_clock_hz", &self.dot_clock_hz);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    /// Publish what the scheduler and a host may ask for without a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.frames.store(state.frames, Ordering::Relaxed);
        self.next_event.store(state.next_event(), Ordering::Relaxed);
    }

    /// Drive the vertical sync pin. Never called with the state lock held —
    /// the re-entrancy contract is that outward calls happen after the critical
    /// section (`CLAUDE.md`, concurrency).
    fn drive_vsync(&self, level: Level) {
        let pin = self.vsync.lock().clone();
        if let Some(pin) = pin {
            pin.set(level);
        }
    }

    /// Bring the CRTC up to date before an access.
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5): a
    /// monitor that read 0x3da must not consume the retrace the guest is
    /// waiting for.
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
        // A refusal means catch-up is already running further up the stack. The
        // access still has to be answered, and answering it from where the
        // chip stands is the only defined thing to do.
        let _ = handle.sync(kind);
    }

    /// Run the beam forward to `target` character clocks.
    fn advance_to(&self, target: u64) {
        let changed = {
            let mut state = self.state.lock();
            if target <= state.ticks {
                return;
            }
            let before = state.in_vsync();
            let per = state.ticks_per_frame();
            match (target - state.frame_start).checked_div(per) {
                // The timing registers say nothing is being scanned, so no
                // frame is in progress and none completes.
                None => {
                    state.ticks = target;
                    state.frame_start = target;
                }
                Some(whole) => {
                    state.frames += whole;
                    state.frame_start += whole * per;
                    state.ticks = target;
                }
            }
            let after = state.in_vsync();
            self.publish(&state);
            if before == after { None } else { Some(after) }
        };
        if let Some(level) = changed {
            self.drive_vsync(Level::from_bool(level));
        }
    }

    /// Re-derive the published numbers after a register write.
    fn touched(&self, state: &mut State) {
        state.renormalize();
        self.publish(state);
    }
}

/// Which window an access arrived through, and — for the CRTC — whether the
/// miscellaneous output's address-pair bit has to select it first.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Window {
    /// The CRTC's address and data registers, answering wherever they are
    /// mapped.
    Crtc,
    /// The same pair, answering only while the colour pair is selected.
    CrtcColour,
    /// The same pair, answering only while the monochrome pair is selected.
    CrtcMono,
    /// The status register.
    Status,
    /// The CGA mode control and colour select registers.
    Mode,
    /// The VGA register file at 0x3c0-0x3cf.
    Vga,
}

/// One mapped window onto the device.
#[derive(Debug)]
struct Port {
    shared: Arc<Shared>,
    window: Window,
    /// How many bytes this window decodes, so a 16-bit access that would run
    /// off the end of it is refused rather than folded back inside.
    len: u64,
}

impl Port {
    /// Whether this window answers at all, given which address pair the
    /// miscellaneous output selects.
    fn decoded(&self, state: &State) -> bool {
        let colour = state.vga.misc & MISC_COLOUR != 0;
        match self.window {
            Window::CrtcColour => colour,
            Window::CrtcMono => !colour,
            _ => true,
        }
    }

    /// Read one register. `debug` suppresses every side effect.
    fn read_register(&self, offset: u64, debug: bool) -> u8 {
        let mut state = self.shared.state.lock();
        if !self.decoded(&state) {
            // Nothing is driving these lines: an ISA bus with nothing on it
            // reads ones.
            return 0xff;
        }
        match self.window {
            Window::Crtc | Window::CrtcColour | Window::CrtcMono => match offset & 1 {
                // On the MC6845 the address register is write-only. The VGA's
                // CRTC index reads back, and firmware saves and restores it, so
                // the latch is returned rather than open bus.
                0 => state.crtc_index,
                _ => {
                    let index = (state.crtc_index & 0x1f) as usize;
                    // R0-R11 are write-only; R12-R17 read. R16/R17 are the
                    // light pen latches, which nothing here ever loads.
                    match index {
                        12..=15 => state.crtc[index],
                        _ => 0,
                    }
                }
            },
            Window::Status => {
                let value = state.status();
                if !debug {
                    // The famous side effect: reading the status register puts
                    // the attribute controller back in its index state.
                    state.vga.attr_data_next = false;
                }
                value
            }
            Window::Mode => match offset & 1 {
                // Both are write-only on a real CGA. The latch is returned
                // because a monitor and a test both want to see what firmware
                // wrote, and no guest can tell the difference without hardware
                // that answers differently.
                0 => state.mode,
                _ => state.colour,
            },
            Window::Vga => Self::read_vga(&mut state, offset, debug),
        }
    }

    /// The VGA register file's read side.
    fn read_vga(state: &mut State, offset: u64, debug: bool) -> u8 {
        match offset & 0x0f {
            // The attribute controller's index, palette address source and all.
            0x0 => state.vga.attr_index,
            0x1 => {
                let index = (state.vga.attr_index & 0x1f) as usize;
                if index < ATTR_REGISTERS {
                    state.vga.attr[index]
                } else {
                    0
                }
            }
            // Input status 0. The switch sense bit reports what the DAC's
            // comparator says about the attached monitor, and nothing here has
            // one, so it reads as no monitor sensed.
            0x2 => 0,
            0x3 => state.vga.enable,
            0x4 => state.vga.seq_index,
            0x5 => {
                let index = state.vga.seq_index as usize;
                if index < SEQ_REGISTERS {
                    state.vga.seq[index]
                } else {
                    0
                }
            }
            0x6 => state.vga.dac_mask,
            // The DAC state register: 3 while the last index written was the
            // read one, 0 while it was the write one.
            0x7 => u8::from(state.vga.dac_reading) * 3,
            0x8 => state.vga.dac_write,
            0x9 => {
                let value =
                    state.vga.dac[state.vga.dac_read as usize][state.vga.dac_read_sub as usize];
                if !debug {
                    Self::step_dac_read(state);
                }
                value
            }
            // 0x3ca and 0x3cb are not decoded on this adapter.
            0xa | 0xb => 0xff,
            // The miscellaneous output's read address; 0x3c2 is its write one.
            0xc => state.vga.misc,
            0xd => 0xff,
            0xe => state.vga.gc_index,
            _ => {
                let index = state.vga.gc_index as usize;
                if index < GC_REGISTERS {
                    state.vga.gc[index]
                } else {
                    0
                }
            }
        }
    }

    /// Advance the DAC's read pointer one component, carrying into the index
    /// after the third — which is what lets firmware read 768 bytes in a loop.
    fn step_dac_read(state: &mut State) {
        state.vga.dac_read_sub += 1;
        if state.vga.dac_read_sub >= 3 {
            state.vga.dac_read_sub = 0;
            state.vga.dac_read = state.vga.dac_read.wrapping_add(1);
        }
    }

    /// Write one register.
    fn write_register(&self, offset: u64, value: u8) {
        let mut state = self.shared.state.lock();
        if !self.decoded(&state) {
            return;
        }
        match self.window {
            Window::Crtc | Window::CrtcColour | Window::CrtcMono => match offset & 1 {
                0 => state.crtc_index = value & 0x1f,
                _ => {
                    let index = (state.crtc_index & 0x1f) as usize;
                    if index < CRTC_REGISTERS {
                        state.crtc[index] = value & CRTC_WRITE_MASK[index];
                        self.shared.touched(&mut state);
                    }
                }
            },
            // Neither the status register nor 0x3db is writable.
            Window::Status => {}
            Window::Mode => match offset & 1 {
                0 => state.mode = value,
                _ => state.colour = value,
            },
            Window::Vga => {
                Self::write_vga(&mut state, offset, value);
                self.shared.touched(&mut state);
            }
        }
    }

    /// The VGA register file's write side.
    fn write_vga(state: &mut State, offset: u64, value: u8) {
        match offset & 0x0f {
            0x0 => {
                // One port, two meanings, alternating. The flip-flop is what a
                // read of the status register resets.
                if state.vga.attr_data_next {
                    let index = (state.vga.attr_index & 0x1f) as usize;
                    if index < ATTR_REGISTERS {
                        state.vga.attr[index] = value & attr_write_mask(index);
                    }
                } else {
                    state.vga.attr_index = value & 0x3f;
                }
                state.vga.attr_data_next = !state.vga.attr_data_next;
            }
            // 0x3c1 is the attribute controller's read port only.
            0x1 => {}
            0x2 => state.vga.misc = value,
            0x3 => state.vga.enable = value & 0x01,
            0x4 => state.vga.seq_index = value & 0x07,
            0x5 => {
                let index = state.vga.seq_index as usize;
                if index < SEQ_REGISTERS {
                    state.vga.seq[index] = value;
                }
            }
            0x6 => state.vga.dac_mask = value,
            0x7 => {
                state.vga.dac_read = value;
                state.vga.dac_read_sub = 0;
                state.vga.dac_reading = true;
            }
            0x8 => {
                state.vga.dac_write = value;
                state.vga.dac_write_sub = 0;
                state.vga.dac_reading = false;
            }
            0x9 => {
                // Six bits per component; the top two are not implemented in
                // the DAC and read back as zero.
                let index = state.vga.dac_write as usize;
                let sub = state.vga.dac_write_sub as usize;
                state.vga.dac[index][sub] = value & 0x3f;
                state.vga.dac_write_sub += 1;
                if state.vga.dac_write_sub >= 3 {
                    state.vga.dac_write_sub = 0;
                    state.vga.dac_write = state.vga.dac_write.wrapping_add(1);
                }
            }
            // Not decoded, and 0x3cc is the miscellaneous output's read address.
            0xa..=0xd => {}
            0xe => state.vga.gc_index = value & 0x0f,
            _ => {
                let index = state.vga.gc_index as usize;
                if index < GC_REGISTERS {
                    state.vga.gc[index] = value;
                }
            }
        }
    }
}

/// What a write to each attribute controller register keeps: the palette
/// entries are six bits, the plane enable and pixel panning four, the colour
/// select four.
fn attr_write_mask(index: usize) -> u8 {
    match index {
        0..=15 => 0x3f,
        18..=20 => 0x0f,
        _ => 0xff,
    }
}

impl Port {
    /// Whether an access of `len` bytes at `offset` is one this window answers.
    ///
    /// One byte, or two consecutive ones — see [`Port::constraints`].
    fn spans(&self, offset: u64, len: usize) -> bool {
        (1..=2).contains(&len) && offset.saturating_add(len as u64) <= self.len
    }
}

impl MemOps for Port {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if !self.spans(offset, dst.len()) {
            return Err(BusError::BadAccess);
        }
        // Before anything is read, not after: the status register's answer is a
        // function of the tick, so the tick has to be the one this access
        // happens on.
        self.shared.sync(attrs);
        for (i, byte) in dst.iter_mut().enumerate() {
            *byte = self.read_register(offset + i as u64, attrs.debug);
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if !self.spans(offset, src.len()) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write to the CRTC's data register would move the start
            // address and scroll the guest's screen, and one to 0x3c0 would
            // desynchronise the flip-flop firmware is counting on. Neither can
            // be made harmless (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        for (i, value) in src.iter().enumerate() {
            self.write_register(offset + i as u64, *value);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Byte **and word**, and the word is not a convenience: a VGA sits on a
        // 16-bit bus and its index/data pairs are laid out so that one
        // `OUT DX, AX` writes both halves — index in AL, datum in AH. That is
        // the idiom every VGA programming reference gives for the CRTC at
        // 0x3d4, the sequencer at 0x3c4 and the graphics controller at 0x3ce
        // (IBM's VGA documentation; FreeVGA, "Accessing the VGA Registers"),
        // and it is what a real video BIOS emits. It invents no order between
        // the halves: little-endian, low byte first, so the index is latched
        // before the datum that uses it — which is the order the pair exists
        // for.
        //
        // Naturally aligned, so a word access to the *data* half alone is
        // still refused: that would be a 16-bit cycle an 8-bit slave splits,
        // and nothing emits one.
        AccessConstraints::word(Width::U8, Endian::Little).with_widths(Width::U8, Width::U16)
    }
}

/// An MC6845-derived CRTC with a character generator and a VGA register file.
#[derive(Debug)]
pub struct Video {
    shared: Arc<Shared>,
    crtc: RegionRef,
    crtc_colour: RegionRef,
    crtc_mono: RegionRef,
    status: RegionRef,
    mode: RegionRef,
    vga: RegionRef,
    vram: RegionRef,
}

/// The whole of a store, as bytes a chunk can carry.
fn read_store(store: &RamStore) -> Result<alloc::vec::Vec<u8>> {
    let len = usize::try_from(store.len())
        .map_err(|_| Error::State(String::from("RAM larger than the host address space")))?;
    let mut buf = alloc::vec![0u8; len];
    store
        .read_at(0, &mut buf)
        .map_err(|e| Error::State(alloc::format!("cannot read the character buffer: {e}")))?;
    Ok(buf)
}

impl Video {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of
    /// the wrong kind, or if one this class does not know was given.
    pub fn new(props: &Props) -> Result<Video> {
        let mut r = props.reader();
        let dot_clock = r.or_range("dot-clock", 0, 0..=1_000_000_000)?;
        r.finish()?;
        Ok(Video::with_dot_clock(dot_clock))
    }

    /// One with default properties.
    #[must_use]
    pub fn default_device() -> Video {
        Video::with_dot_clock(0)
    }

    /// One whose pixel clock is `dot_clock_hz` rather than whichever crystal
    /// the miscellaneous output selects. `0` follows the register.
    #[must_use]
    pub fn with_dot_clock(dot_clock_hz: u64) -> Video {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::new()),
            vram: Arc::new(RamStore::new(VRAM_LEN)),
            ticks: AtomicU64::new(0),
            frames: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            dot_clock_hz,
            lazy: Mutex::with_rank(LockRank::LEAF, None),
            vsync: Mutex::with_rank(LockRank::LEAF, None),
        });
        shared.publish(&shared.state.lock());
        let port = |window, len| -> RegionRef {
            Arc::new(Region::io(
                CLASS_NAME,
                len,
                Arc::new(Port {
                    shared: Arc::clone(&shared),
                    window,
                    len,
                }) as Arc<dyn MemOps>,
            ))
        };
        let vram = Arc::new(Region::ram("pc.video.vram", Arc::clone(&shared.vram)));
        Video {
            crtc: port(Window::Crtc, CRTC_WINDOW_LEN),
            crtc_colour: port(Window::CrtcColour, CRTC_WINDOW_LEN),
            crtc_mono: port(Window::CrtcMono, CRTC_WINDOW_LEN),
            status: port(Window::Status, STATUS_WINDOW_LEN),
            mode: port(Window::Mode, MODE_WINDOW_LEN),
            vga: port(Window::Vga, VGA_WINDOW_LEN),
            vram,
            shared,
        }
    }

    /// The character buffer, for a host or a test that wants to write into it
    /// without going through a bus.
    #[must_use]
    pub fn vram(&self) -> &Arc<RamStore> {
        &self.shared.vram
    }

    /// A [`Scanout`] over this device, for a host that holds the concrete
    /// device and wants its picture.
    #[must_use]
    pub fn scanout(&self) -> VideoScanout {
        VideoScanout {
            shared: Arc::clone(&self.shared),
        }
    }

    /// Character clocks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// Frames completed since reset.
    #[must_use]
    pub fn frames(&self) -> u64 {
        self.shared.frames.load(Ordering::Relaxed)
    }

    /// Run the beam until `target` character clocks have passed in total.
    ///
    /// The catch-up entry point. Running backwards is a no-op, not an error.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }

    /// Connect the catch-up handle the status register syncs through (§4.2).
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// The status register as a read would produce it, advancing nothing.
    #[must_use]
    pub fn status(&self) -> u8 {
        self.shared.state.lock().status()
    }
}

/// The `pc.video` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "MC6845 CRTC with a text-mode character generator and a VGA register file",
    properties: &[PropertySpec {
        name: "dot-clock",
        kind: ValueKind::Uint,
        required: false,
        summary: "the pixel clock in Hz, overriding the clock select bits (default: follow them)",
    }],
    construct: |props| Ok(Box::new(Video::new(props)?)),
};

impl Device for Video {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        let level = {
            let mut state = self.shared.state.lock();
            *state = State::new();
            self.shared.publish(&state);
            state.in_vsync()
        };
        // The character buffer is deliberately not cleared: the chip has no
        // path to its own refresh RAM, and on a real machine what was on the
        // screen is still there until firmware writes over it.
        self.shared.drive_vsync(Level::from_bool(level));
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        let region = match name {
            "" | "crtc" | "regs" => &self.crtc,
            "crtc-colour" => &self.crtc_colour,
            "crtc-mono" => &self.crtc_mono,
            "status" => &self.status,
            "mode" => &self.mode,
            "vga" => &self.vga,
            "vram" => &self.vram,
            _ => return None,
        };
        Some(Arc::clone(region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "vsync" {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("a CRTC drives one pin, `vsync`"),
            });
        }
        *self.shared.vsync.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == "vsync" {
            let level = self.shared.state.lock().in_vsync();
            self.shared.drive_vsync(Level::from_bool(level));
        }
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. A read of the status register has to see the retrace bit as it was
    /// on the cycle of the read; a poll loop against a stale one never ends.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Video::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        Video::attach_lazy(self, handle);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u64(state.frame_start)?;
        w.write_u64(state.frames)?;
        w.write_u8(state.crtc_index)?;
        for byte in state.crtc {
            w.write_u8(byte)?;
        }
        w.write_u8(state.mode)?;
        w.write_u8(state.colour)?;
        w.write_u8(state.vga.misc)?;
        w.write_u8(state.vga.enable)?;
        w.write_u8(state.vga.attr_index)?;
        w.write_bool(state.vga.attr_data_next)?;
        for byte in state.vga.attr {
            w.write_u8(byte)?;
        }
        w.write_u8(state.vga.seq_index)?;
        for byte in state.vga.seq {
            w.write_u8(byte)?;
        }
        w.write_u8(state.vga.gc_index)?;
        for byte in state.vga.gc {
            w.write_u8(byte)?;
        }
        w.write_u8(state.vga.dac_mask)?;
        w.write_u8(state.vga.dac_read)?;
        w.write_u8(state.vga.dac_write)?;
        w.write_u8(state.vga.dac_read_sub)?;
        w.write_u8(state.vga.dac_write_sub)?;
        w.write_bool(state.vga.dac_reading)?;
        for entry in state.vga.dac {
            for component in entry {
                w.write_u8(component)?;
            }
        }
        // The character buffer. This *is* architectural state and it has to be
        // here: `Machine::save` walks devices, not regions, so the only RAM a
        // machine snapshots by itself is a `ram` device instance. This adapter's
        // `vram` is its own store, mapped from `region("vram")`, so nothing else
        // in the tree was ever going to write it — 32 KiB of guest-writable and
        // guest-*readable* memory (BIOS scroll routines read the text page back)
        // came out of a restore as zeroes, and neither a chunk diff nor a state
        // hash could see it, because it was absent from both sides.
        //
        // The rendered pixels stay out: those are derived from this buffer on
        // every capture.
        w.write_bytes(&read_store(&self.shared.vram)?)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new();
        state.ticks = r.read_u64()?;
        state.frame_start = r.read_u64()?;
        state.frames = r.read_u64()?;
        state.crtc_index = r.read_u8()?;
        for i in 0..CRTC_REGISTERS {
            state.crtc[i] = r.read_u8()?;
        }
        state.mode = r.read_u8()?;
        state.colour = r.read_u8()?;
        state.vga.misc = r.read_u8()?;
        state.vga.enable = r.read_u8()?;
        state.vga.attr_index = r.read_u8()?;
        state.vga.attr_data_next = r.read_bool()?;
        for i in 0..ATTR_REGISTERS {
            state.vga.attr[i] = r.read_u8()?;
        }
        state.vga.seq_index = r.read_u8()?;
        for i in 0..SEQ_REGISTERS {
            state.vga.seq[i] = r.read_u8()?;
        }
        state.vga.gc_index = r.read_u8()?;
        for i in 0..GC_REGISTERS {
            state.vga.gc[i] = r.read_u8()?;
        }
        state.vga.dac_mask = r.read_u8()?;
        state.vga.dac_read = r.read_u8()?;
        state.vga.dac_write = r.read_u8()?;
        state.vga.dac_read_sub = r.read_u8()?;
        state.vga.dac_write_sub = r.read_u8()?;
        state.vga.dac_reading = r.read_bool()?;
        for i in 0..DAC_ENTRIES {
            for c in 0..DAC_COMPONENTS {
                state.vga.dac[i][c] = r.read_u8()?;
            }
        }
        let vram = r.read_bytes()?;
        if vram.len() as u64 != VRAM_LEN {
            return Err(Error::State(alloc::format!(
                "snapshot has {} byte(s) of character buffer, this adapter has {VRAM_LEN}",
                vram.len()
            )));
        }
        self.shared.vram.write_at(0, vram).map_err(|e| {
            Error::State(alloc::format!("cannot restore the character buffer: {e}"))
        })?;
        if state.frame_start > state.ticks {
            return Err(Error::State(alloc::format!(
                "snapshot has a frame beginning at {} after the current tick {}",
                state.frame_start,
                state.ticks
            )));
        }
        // Both sub-indices select one of three DAC components and are used as
        // an array index unchecked on the 0x3c9 path, so a snapshot is the one
        // place they can arrive out of range. §4.5 asks for a diagnosis rather
        // than a crash, and every peer in this directory range-checks the
        // indices it restores.
        for (what, sub) in [
            ("read", state.vga.dac_read_sub),
            ("write", state.vga.dac_write_sub),
        ] {
            if usize::from(sub) >= DAC_COMPONENTS {
                return Err(Error::State(alloc::format!(
                    "snapshot has the DAC {what} sub-index at {sub}, past the \
                     {DAC_COMPONENTS} components of an entry"
                )));
            }
        }
        let level = {
            let mut current = self.shared.state.lock();
            *current = state;
            current.renormalize();
            self.shared.publish(&current);
            current.in_vsync()
        };
        self.shared.drive_vsync(Level::from_bool(level));
        Ok(())
    }
}

impl Instance for Video {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Video::new(props)?)))
}

/// What the validator should know about `pc.video`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::{PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("dot-clock", ValueKind::Uint).range(0, 1_000_000_000))
        .region("")
        .region("regs")
        .region("crtc")
        .region("crtc-colour")
        .region("crtc-mono")
        .region("status")
        .region("mode")
        .region("vga")
        .region("vram")
        .port("vsync", PortDir::Out)
}

// ---------------------------------------------------------------------------
// The scanout seam
// ---------------------------------------------------------------------------

/// A [`Scanout`] over a [`Video`]: the character buffer, drawn.
///
/// The colour conversion lives here rather than in the device proper, which is
/// the seam's rule — nothing that produces pixels names a colour
/// ([`crate::host::display`]). It sits in this file because the chain from an
/// attribute to RGB runs through the attribute controller's palette and the
/// DAC, both of which are guest state; a `host::display::pc` adapter would have
/// to reach back through the device for all of it. The palette table itself is
/// `CGA_PALETTE_6BIT`, and it is only the DAC's power-on contents.
#[derive(Debug, Clone)]
pub struct VideoScanout {
    shared: Arc<Shared>,
}

impl VideoScanout {
    /// Watch the device behind `video`.
    #[must_use]
    pub fn new(video: &Video) -> VideoScanout {
        video.scanout()
    }
}

/// How many frames the cursor is visible for, then hidden for, at the 6845's
/// "1/16 field rate" setting; the 1/32 setting is twice this. Data sheet, the
/// cursor control bits of R10.
const CURSOR_BLINK_FRAMES: u64 = 8;

/// How many frames a blinking character is visible for, then hidden for.
///
/// Sixteen on and sixteen off is the CGA's documented 1.875 Hz at a 60 Hz field
/// rate.
const TEXT_BLINK_FRAMES: u64 = 16;

impl Scanout for VideoScanout {
    fn info(&self) -> SurfaceInfo {
        let state = self.shared.state.lock();
        let width = state.columns() as u32 * state.char_width();
        let height = (state.rows() * state.cell_height()) as u32;
        SurfaceInfo::new(width, height, PixelFormat::RGBA8888)
    }

    fn frame_counter(&self) -> u64 {
        self.shared.frames.load(Ordering::Relaxed)
    }

    fn frame_period_ns(&self) -> u64 {
        let state = self.shared.state.lock();
        let per = state.ticks_per_frame();
        let dot_hz = state.dot_clock_hz(self.shared.dot_clock_hz);
        if per == 0 || dot_hz == 0 {
            return 0;
        }
        // characters per frame x dots per character x 1e9 / dots per second.
        // Exact integer arithmetic from the chip's own timing registers and its
        // own clock — never a wall-clock measurement, never a float
        // (`CLAUDE.md`, determinism).
        per.saturating_mul(u64::from(state.char_width()))
            .saturating_mul(1_000_000_000)
            / dot_hz
    }

    fn capture(&self, dst: &mut Surface) -> u64 {
        // The whole register file is copied and the lock dropped, so the
        // guest is not held out of its own registers for a frame's worth of
        // painting. The buffer itself is read afterwards, cell by cell: it is a
        // `RamStore`, which is atomic per byte and needs no lock at all.
        let state = self.shared.state.lock().clone();
        let width = state.columns() as u32 * state.char_width();
        let height = (state.rows() * state.cell_height()) as u32;
        dst.reshape(dst.format(), width, height);
        let serial = state.frames;

        if !state.video_enabled() {
            dst.fill([0, 0, 0]);
            dst.set_serial(serial);
            return serial;
        }

        let cell_height = state.cell_height();
        let char_width = state.char_width();
        let columns = state.columns();
        let blink = state.blink_enabled();
        let text_visible = (state.frames / TEXT_BLINK_FRAMES).is_multiple_of(2);
        // R10's two cursor-mode bits: steady, hidden, or blinking at one of two
        // rates (MC6845 data sheet, cursor start register).
        let cursor_visible = match (state.crtc[10] >> 5) & 0x03 {
            0 => true,
            1 => false,
            2 => (state.frames / CURSOR_BLINK_FRAMES).is_multiple_of(2),
            _ => (state.frames / (2 * CURSOR_BLINK_FRAMES)).is_multiple_of(2),
        };
        let cursor_at = state.cursor_address();
        let cursor_first = u64::from(state.crtc[10] & 0x1f);
        let cursor_last = u64::from(state.crtc[11] & 0x1f);
        // Line graphics: on a nine-dot cell the ninth column repeats the eighth
        // for the box-drawing range, so a frame's strokes join up. Attribute
        // controller mode control bit 2.
        let line_graphics = state.vga.attr[16] & 0x04 != 0;

        for row in 0..state.rows() {
            for column in 0..columns {
                // The refresh address is fourteen bits, and each character is
                // two bytes: code then attribute.
                let address = (state.start_address() + row * columns + column) & 0x3fff;
                let offset = address * 2;
                let code = self.shared.vram.read_u8(offset).unwrap_or(0);
                let attribute = self.shared.vram.read_u8(offset + 1).unwrap_or(0);

                let mut foreground = attribute & 0x0f;
                let background = if blink {
                    (attribute >> 4) & 0x07
                } else {
                    (attribute >> 4) & 0x0f
                };
                let blinking = blink && attribute & 0x80 != 0;
                if blinking && !text_visible {
                    // The cell keeps its background and loses its glyph, which
                    // is what blink does — it does not go black.
                    foreground = background;
                }
                let fg = state.rgb(foreground);
                let bg = state.rgb(background);

                let bitmap = glyph(code);
                let cursor_here = cursor_visible
                    && address == cursor_at
                    // The data sheet leaves a start beyond the end undefined;
                    // no cursor is the conservative reading, and it is what
                    // firmware that hides the cursor that way expects.
                    && cursor_first <= cursor_last;

                for line in 0..cell_height {
                    // The font is drawn for a sixteen-line cell. A shorter one
                    // samples it, which is a stopgap: a machine given a real
                    // font ROM would carry one bitmap per cell height.
                    let source = (line * FONT_CELL_HEIGHT as u64) / cell_height;
                    let bits = bitmap[source as usize];
                    let on_cursor = cursor_here && line >= cursor_first && line <= cursor_last;
                    let y = (row * cell_height + line) as u32;
                    for dot in 0..char_width {
                        let lit = if on_cursor {
                            true
                        } else if dot < 8 {
                            bits & (0x80 >> dot) != 0
                        } else if line_graphics && (0xc0..=0xdf).contains(&code) {
                            bits & 0x01 != 0
                        } else {
                            false
                        };
                        let x = column as u32 * char_width + dot;
                        dst.put(x, y, if lit { fg } else { bg });
                    }
                }
            }
        }
        dst.set_serial(serial);
        serial
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    /// Offsets within the CRTC window.
    const ADDRESS: u64 = 0;
    const DATA: u64 = 1;

    fn device() -> Video {
        Video::default_device()
    }

    fn port(video: &Video, window: Window) -> Port {
        Port {
            shared: Arc::clone(&video.shared),
            window,
            len: match window {
                Window::Crtc | Window::CrtcColour | Window::CrtcMono => CRTC_WINDOW_LEN,
                Window::Status => STATUS_WINDOW_LEN,
                Window::Mode => MODE_WINDOW_LEN,
                Window::Vga => VGA_WINDOW_LEN,
            },
        }
    }

    fn peek(video: &Video, window: Window, offset: u64) -> u8 {
        let mut byte = [0u8; 1];
        port(video, window)
            .read(offset, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        byte[0]
    }

    fn peek_debug(video: &Video, window: Window, offset: u64) -> u8 {
        let mut byte = [0u8; 1];
        port(video, window)
            .read(offset, &mut byte, MemAttrs::DEBUG)
            .expect("a byte read is legal");
        byte[0]
    }

    fn poke(video: &Video, window: Window, offset: u64, value: u8) {
        port(video, window)
            .write(offset, &[value], MemAttrs::DEFAULT)
            .expect("a byte write is legal");
    }

    fn crtc(video: &Video, index: u8, value: u8) {
        poke(video, Window::Crtc, ADDRESS, index);
        poke(video, Window::Crtc, DATA, value);
    }

    /// Write `text` at `address` (in characters) with attribute `attr`.
    fn write_text(video: &Video, address: u64, text: &str, attr: u8) {
        write_bytes(video, address, text.as_bytes(), attr);
    }

    /// The same, for code points a `str` cannot hold: the buffer takes bytes,
    /// and 0xdb is a block rather than half of a two-byte UTF-8 sequence.
    fn write_bytes(video: &Video, address: u64, codes: &[u8], attr: u8) {
        for (i, byte) in codes.iter().enumerate() {
            let offset = (address + i as u64) * 2;
            video.vram().write_u8(offset, *byte).unwrap();
            video.vram().write_u8(offset + 1, attr).unwrap();
        }
    }

    /// Hide the cursor, for a test comparing two pictures that would otherwise
    /// differ only by where it sits.
    fn hide_cursor(video: &Video) {
        crtc(video, 10, 0x20);
    }

    fn captured(video: &Video) -> Surface {
        let scanout = video.scanout();
        let mut surface = Surface::for_scanout(&scanout);
        scanout.capture(&mut surface);
        surface
    }

    #[test]
    fn the_address_register_selects_which_register_a_write_lands_in() {
        let video = device();
        crtc(&video, 12, 0x11);
        crtc(&video, 13, 0x22);
        let state = video.shared.state.lock();
        assert_eq!(state.crtc[12], 0x11);
        assert_eq!(state.crtc[13], 0x22);
        // And nothing else moved.
        assert_eq!(state.crtc[14], CRTC_DEFAULTS[14]);
    }

    #[test]
    fn registers_12_to_17_read_back_and_0_to_11_do_not() {
        let video = device();
        // R0-R11 are write-only on the 6845.
        crtc(&video, 0, 0x5a);
        poke(&video, Window::Crtc, ADDRESS, 0);
        assert_eq!(peek(&video, Window::Crtc, DATA), 0);
        assert_eq!(video.shared.state.lock().crtc[0], 0x5a, "but it was stored");

        for (index, value) in [(12u8, 0x0a), (13, 0xbc), (14, 0x01), (15, 0x23)] {
            crtc(&video, index, value);
            poke(&video, Window::Crtc, ADDRESS, index);
            assert_eq!(
                peek(&video, Window::Crtc, DATA),
                value & CRTC_WRITE_MASK[index as usize]
            );
        }

        // The light pen latches read as zero: nothing here ever loads them.
        poke(&video, Window::Crtc, ADDRESS, 16);
        assert_eq!(peek(&video, Window::Crtc, DATA), 0);
        poke(&video, Window::Crtc, DATA, 0xff);
        assert_eq!(
            peek(&video, Window::Crtc, DATA),
            0,
            "and a write is refused"
        );

        // Above 17 there is no register at all.
        poke(&video, Window::Crtc, ADDRESS, 20);
        assert_eq!(peek(&video, Window::Crtc, DATA), 0);
        // The index itself reads back, which firmware relies on.
        assert_eq!(peek(&video, Window::Crtc, ADDRESS), 20);
    }

    #[test]
    fn moving_the_start_address_scrolls_the_picture() {
        let video = device();
        // Two lines of text, one screen row apart. The cursor is hidden so the
        // two pictures below differ only by what scrolled.
        hide_cursor(&video);
        write_text(&video, 0, "TOP", 0x07);
        write_text(&video, 80, "NEXT", 0x07);
        let first = captured(&video);

        // Scrolling by one row is a start address of one line of characters —
        // which is the only thing DOS does to scroll.
        crtc(&video, 12, 0);
        crtc(&video, 13, 80);
        let scrolled = captured(&video);
        assert_ne!(first.hash(), scrolled.hash(), "the picture moved");

        // What was on row 1 is now on row 0: compare against a picture drawn
        // with "NEXT" at the top and nothing else.
        let reference = device();
        hide_cursor(&reference);
        write_text(&reference, 0, "NEXT", 0x07);
        let expect = captured(&reference);
        for y in 0..16u32 {
            for x in 0..(4 * 9) as u32 {
                assert_eq!(scrolled.get(x, y), expect.get(x, y), "at {x},{y}");
            }
        }
    }

    #[test]
    fn the_cursor_sits_where_r14_and_r15_name_it() {
        let video = device();
        // A cursor on the last two rasters of the cell at column 3, row 1. The
        // cursor takes the *cell's* foreground colour, so the cell needs one.
        let address = 80 + 3;
        write_bytes(&video, address, b" ", 0x07);
        crtc(&video, 14, (address >> 8) as u8);
        crtc(&video, 15, (address & 0xff) as u8);
        crtc(&video, 10, 14);
        crtc(&video, 11, 15);
        let surface = captured(&video);

        let x = 3 * 9 + 1;
        let top = 16;
        assert_eq!(
            surface.get(x, top + 14),
            Some([0xaa, 0xaa, 0xaa]),
            "the cursor's first raster is lit in the foreground colour"
        );
        assert_eq!(surface.get(x, top + 15), Some([0xaa, 0xaa, 0xaa]));
        assert_eq!(
            surface.get(x, top + 13),
            Some([0, 0, 0]),
            "and the raster above it is not"
        );

        // R10 bit 5 with bit 6 clear is "cursor non-display".
        crtc(&video, 10, 14 | 0x20);
        let hidden = captured(&video);
        assert_eq!(hidden.get(x, top + 14), Some([0, 0, 0]));
        assert_eq!(hidden.get(x, top + 15), Some([0, 0, 0]));
    }

    #[test]
    fn the_retrace_bit_follows_the_clock_and_a_debug_read_does_not_advance_it() {
        let video = device();
        let chars = 100u64;
        // Vertical sync begins at row 25, so raster line 400.
        let vsync_start = 400 * chars;

        assert_eq!(peek(&video, Window::Status, 0) & STATUS_VSYNC, 0);
        video.advance_to(chars * 10);
        assert_eq!(
            peek(&video, Window::Status, 0) & STATUS_VSYNC,
            0,
            "line 10 is displayed"
        );
        assert_eq!(
            peek(&video, Window::Status, 0) & STATUS_DISPLAY_ENABLE,
            0,
            "and the beam is inside the displayed columns"
        );

        video.advance_to(chars * 10 + 90);
        assert_eq!(
            peek(&video, Window::Status, 0) & STATUS_DISPLAY_ENABLE,
            STATUS_DISPLAY_ENABLE,
            "past column 80 is horizontal blanking"
        );

        video.advance_to(vsync_start + 5);
        assert_eq!(
            peek(&video, Window::Status, 0) & STATUS_VSYNC,
            STATUS_VSYNC,
            "inside the sixteen-line sync pulse"
        );
        video.advance_to(vsync_start + VSYNC_LINES * chars + 1);
        assert_eq!(
            peek(&video, Window::Status, 0) & STATUS_VSYNC,
            0,
            "and out the other side"
        );

        // A debug read advances nothing and consumes nothing.
        let before = video.ticks();
        let _ = peek_debug(&video, Window::Status, 0);
        assert_eq!(video.ticks(), before);
    }

    #[test]
    fn a_debug_read_of_the_status_register_leaves_the_flip_flop_alone() {
        let video = device();
        // Put the attribute controller in its data state.
        poke(&video, Window::Vga, 0x0, 0x00);
        assert!(video.shared.state.lock().vga.attr_data_next);
        let _ = peek_debug(&video, Window::Status, 0);
        assert!(
            video.shared.state.lock().vga.attr_data_next,
            "a debugger must not resynchronise the guest's flip-flop"
        );
        let _ = peek(&video, Window::Status, 0);
        assert!(!video.shared.state.lock().vga.attr_data_next);
    }

    #[test]
    fn the_attribute_flip_flop_alternates_between_index_and_data() {
        let video = device();
        // Index, then data.
        poke(&video, Window::Vga, 0x0, 0x02);
        poke(&video, Window::Vga, 0x0, 0x3f);
        assert_eq!(peek(&video, Window::Vga, 0x1), 0x3f);
        assert_eq!(peek(&video, Window::Vga, 0x0), 0x02, "the index reads back");

        // A status read resets it, so the next write is an index again.
        let _ = peek(&video, Window::Status, 0);
        poke(&video, Window::Vga, 0x0, 0x03);
        poke(&video, Window::Vga, 0x0, 0x11);
        assert_eq!(peek(&video, Window::Vga, 0x1), 0x11);
        assert_eq!(
            video.shared.state.lock().vga.attr[2],
            0x3f,
            "and the earlier register kept its value"
        );
    }

    #[test]
    fn the_dac_round_trips_every_entry_and_auto_increments() {
        let video = device();
        poke(&video, Window::Vga, 0x8, 0);
        for i in 0..DAC_ENTRIES {
            poke(&video, Window::Vga, 0x9, (i % 64) as u8);
            poke(&video, Window::Vga, 0x9, ((i + 1) % 64) as u8);
            poke(&video, Window::Vga, 0x9, ((i + 2) % 64) as u8);
        }
        // Writing 768 bytes wrapped the index right back round.
        assert_eq!(video.shared.state.lock().vga.dac_write, 0);

        poke(&video, Window::Vga, 0x7, 0);
        assert_eq!(peek(&video, Window::Vga, 0x7), 3, "the DAC is in read mode");
        for i in 0..DAC_ENTRIES {
            assert_eq!(peek(&video, Window::Vga, 0x9), (i % 64) as u8);
            assert_eq!(peek(&video, Window::Vga, 0x9), ((i + 1) % 64) as u8);
            assert_eq!(peek(&video, Window::Vga, 0x9), ((i + 2) % 64) as u8);
        }
        poke(&video, Window::Vga, 0x8, 0x10);
        assert_eq!(peek(&video, Window::Vga, 0x8), 0x10);
        assert_eq!(peek(&video, Window::Vga, 0x7), 0, "and back to write mode");

        // Only six bits per component survive.
        poke(&video, Window::Vga, 0x8, 5);
        poke(&video, Window::Vga, 0x9, 0xff);
        assert_eq!(video.shared.state.lock().vga.dac[5][0], 0x3f);
    }

    #[test]
    fn misc_output_bit_0_chooses_which_crtc_window_answers() {
        let video = device();
        // The colour pair is selected out of reset.
        poke(&video, Window::CrtcColour, ADDRESS, 12);
        assert_eq!(peek(&video, Window::CrtcColour, ADDRESS), 12);
        assert_eq!(
            peek(&video, Window::CrtcMono, ADDRESS),
            0xff,
            "the monochrome pair is not decoded"
        );
        poke(&video, Window::CrtcMono, ADDRESS, 3);
        assert_eq!(peek(&video, Window::CrtcColour, ADDRESS), 12, "and inert");

        // Clear bit 0 and the two swap over.
        poke(&video, Window::Vga, 0x2, 0x66);
        assert_eq!(peek(&video, Window::Vga, 0xc), 0x66, "0x3cc reads it back");
        assert_eq!(peek(&video, Window::CrtcColour, ADDRESS), 0xff);
        poke(&video, Window::CrtcMono, ADDRESS, 7);
        assert_eq!(peek(&video, Window::CrtcMono, ADDRESS), 7);
        // The undecoded window is what a board with one adapter maps, and it
        // answers either way.
        assert_eq!(peek(&video, Window::Crtc, ADDRESS), 7);
    }

    #[test]
    fn an_attributes_colours_land_in_the_right_pixels() {
        let video = device();
        hide_cursor(&video);
        // A space on a blue background with a white foreground: every pixel of
        // the cell is background, because a space has no lit dots.
        // Then a full block (0xdb), which is every pixel foreground, and an
        // asterisk, whose eighth and ninth columns are both blank.
        write_bytes(&video, 0, b" ", 0x1f);
        write_bytes(&video, 1, &[0xdb], 0x4e);
        write_bytes(&video, 2, b"*", 0x21);
        let surface = captured(&video);

        assert_eq!(surface.get(0, 0), Some([0x00, 0x00, 0xaa]), "blue ground");
        assert_eq!(
            surface.get(9, 5),
            Some([0xff, 0xff, 0x55]),
            "yellow on the block's first column"
        );
        // 0xdb is inside the box-drawing range, so the ninth column repeats the
        // eighth: that is what makes a frame's strokes meet.
        assert_eq!(surface.get(9 + 8, 5), Some([0xff, 0xff, 0x55]));
        // Outside that range the ninth column is background — here green.
        assert_eq!(surface.get(18 + 8, 5), Some([0x00, 0xaa, 0x00]));
        assert_eq!(
            surface.get(18 + 3, 5),
            Some([0x00, 0x00, 0xaa]),
            "the glyph"
        );
    }

    #[test]
    fn a_palette_change_is_visible_in_what_capture_produces() {
        let video = device();
        hide_cursor(&video);
        write_bytes(&video, 0, &[0xdb], 0x0f);
        assert_eq!(captured(&video).get(0, 0), Some([0xff, 0xff, 0xff]));

        // Reprogram DAC entry 15 — which is what colour 15 reaches through the
        // identity palette — to full red.
        poke(&video, Window::Vga, 0x8, 15);
        poke(&video, Window::Vga, 0x9, 0x3f);
        poke(&video, Window::Vga, 0x9, 0x00);
        poke(&video, Window::Vga, 0x9, 0x00);
        assert_eq!(captured(&video).get(0, 0), Some([0xff, 0x00, 0x00]));

        // And the attribute controller's palette redirects colour 15 somewhere
        // else again: entry 1 is blue. Selecting a palette register clears the
        // palette address source, which blanks the screen — that is what the
        // bit is for, and firmware sets it again when it has finished.
        poke(&video, Window::Vga, 0x0, 0x0f);
        poke(&video, Window::Vga, 0x0, 0x01);
        assert_eq!(
            captured(&video).get(0, 0),
            Some([0x00, 0x00, 0x00]),
            "blanked while the palette is being programmed"
        );
        poke(&video, Window::Vga, 0x0, 0x20);
        assert_eq!(captured(&video).get(0, 0), Some([0x00, 0x00, 0xaa]));
    }

    #[test]
    fn capturing_a_known_string_gives_a_stable_hash() {
        // The rendering regression: if a glyph, the layout or the colour chain
        // changes, this number changes with it (`ROADMAP.md` §12).
        let video = device();
        write_text(&video, 0, "rsemu 0.1 -- PC video", 0x0f);
        write_bytes(&video, 80, &[0xc9, 0xcd, 0xcd, 0xbb], 0x1e);
        let surface = captured(&video);
        assert_eq!(surface.width(), 720);
        assert_eq!(surface.height(), 400);
        assert_eq!(surface.hash(), 0xe736_85eb_ec2d_b0dd);
    }

    #[test]
    fn the_frame_period_comes_from_the_registers_and_the_clock() {
        let video = device();
        let scanout = video.scanout();
        // 100 characters x 449 lines x 9 dots at 28.322 MHz.
        let expect = 100 * 449 * 9 * 1_000_000_000 / DOT_CLOCK_28MHZ;
        assert_eq!(scanout.frame_period_ns(), expect);

        // A shorter frame is a shorter period, exactly.
        crtc(&video, 4, 25);
        let lines = 26 * 16 + 1;
        assert_eq!(
            scanout.frame_period_ns(),
            100 * lines * 9 * 1_000_000_000 / DOT_CLOCK_28MHZ
        );

        // The clock select bits move it too, and so does an eight-dot cell.
        poke(&video, Window::Vga, 0x2, 0x63);
        assert_eq!(
            scanout.frame_period_ns(),
            100 * lines * 9 * 1_000_000_000 / DOT_CLOCK_25MHZ
        );
        poke(&video, Window::Vga, 0x4, 1);
        poke(&video, Window::Vga, 0x5, 0x01);
        assert_eq!(
            scanout.frame_period_ns(),
            100 * lines * 8 * 1_000_000_000 / DOT_CLOCK_25MHZ
        );
        assert_eq!(scanout.info().width, 640, "and the picture narrows");

        // A machine file that names its own crystal overrides the bits.
        let fixed = Video::with_dot_clock(DOT_CLOCK_25MHZ);
        assert_eq!(
            fixed.scanout().frame_period_ns(),
            100 * 449 * 9 * 1_000_000_000 / DOT_CLOCK_25MHZ
        );
    }

    #[test]
    fn frames_advance_with_the_clock_and_bound_the_next_event() {
        let video = device();
        let per = 100 * 449;
        assert_eq!(video.frames(), 0);
        assert_eq!(video.scanout().frame_counter(), 0);
        video.advance_to(per * 3 + 7);
        assert_eq!(video.frames(), 3);
        // The next thing to happen is the start of the sync pulse.
        let next = Device::next_event_tick(&video).expect("a sync edge");
        assert!(next > video.ticks() && next <= per * 4);
        assert!(Device::is_lazy(&video));
    }

    #[test]
    fn a_snapshot_round_trip_is_byte_identical() {
        let saved = device();
        crtc(&saved, 12, 0x01);
        crtc(&saved, 13, 0x40);
        crtc(&saved, 9, 7);
        poke(&saved, Window::Mode, 0, 0x2d);
        poke(&saved, Window::Mode, 1, 0x3a);
        poke(&saved, Window::Vga, 0x2, 0x63);
        poke(&saved, Window::Vga, 0x0, 0x05);
        poke(&saved, Window::Vga, 0x0, 0x2a);
        poke(&saved, Window::Vga, 0x8, 3);
        poke(&saved, Window::Vga, 0x9, 0x11);
        poke(&saved, Window::Vga, 0x9, 0x22);
        poke(&saved, Window::Vga, 0xe, 4);
        poke(&saved, Window::Vga, 0xf, 0x0f);
        // The character buffer is architectural state too, and it is the one
        // piece of it a byte-identical comparison of two save images cannot
        // catch on its own: state that is in neither image agrees with itself.
        // So it is written here and read back below.
        saved.vram().write_u8(0, b'r').unwrap();
        saved.vram().write_u8(1, 0x0f).unwrap();
        saved.vram().write_u8(VRAM_LEN - 1, 0xa5).unwrap();
        saved.advance_to(123_456);

        let image = |video: &Video| {
            let mut shape = MachineShape::new();
            shape.add_device("vga", CLASS.name).unwrap();
            let mut w = StateWriter::new(shape);
            {
                let mut chunk = w.chunk("vga", CLASS.name, CLASS.version).unwrap();
                video.save(&mut chunk).unwrap();
            }
            w.to_vec().unwrap()
        };

        let bytes = image(&saved);
        let restored = device();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("vga", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        assert_eq!(image(&restored), bytes, "the two save images agree");
        assert_eq!(restored.ticks(), 123_456);
        assert_eq!(restored.frames(), saved.frames());
        assert_eq!(restored.status(), saved.status());
        assert_eq!(restored.vram().read_u8(0).unwrap(), b'r');
        assert_eq!(restored.vram().read_u8(1).unwrap(), 0x0f);
        assert_eq!(restored.vram().read_u8(VRAM_LEN - 1).unwrap(), 0xa5);
    }

    #[test]
    fn a_dac_sub_index_out_of_range_is_diagnosed_rather_than_panicked() {
        // The two sub-indices select one of three components and are used as an
        // array index unchecked on the 0x3c9 path, so a snapshot is the one
        // place they can arrive out of range (§4.5: a diagnosis, not a crash).
        let image = |video: &Video| {
            let mut shape = MachineShape::new();
            shape.add_device("vga", CLASS.name).unwrap();
            let mut w = StateWriter::new(shape);
            {
                let mut chunk = w.chunk("vga", CLASS.name, CLASS.version).unwrap();
                video.save(&mut chunk).unwrap();
            }
            w.to_vec().unwrap()
        };

        // Locate `dac_write_sub` without hard-coding an offset that the next
        // field added to this chunk would silently invalidate: it is the first
        // byte that moves when one DAC component is written.
        let plain = image(&device());
        let stepped = device();
        poke(&stepped, Window::Vga, 0x9, 0x2a);
        let at = plain
            .iter()
            .zip(image(&stepped).iter())
            .position(|(a, b)| a != b)
            .expect("writing a component moves the sub-index");

        let mut bytes = plain;
        bytes[at] = 7;
        let restored = device();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("vga", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        assert!(
            restored.load(&mut chunk.reader()).is_err(),
            "a DAC sub-index of 7 was accepted, and indexes an array of {DAC_COMPONENTS}"
        );
    }

    #[test]
    fn a_debug_write_is_refused_and_so_is_an_access_wider_than_the_bus() {
        let video = device();
        assert!(
            port(&video, Window::Crtc)
                .write(ADDRESS, &[0], MemAttrs::DEBUG)
                .is_err()
        );
        // Wider than sixteen bits is not a cycle this bus carries.
        assert!(
            port(&video, Window::Vga)
                .write(0, &[0u8; 4], MemAttrs::DEFAULT)
                .is_err()
        );
        // Nor is a sixteen-bit one that runs off the end of the window: the
        // CRTC pair is two bytes, and the second half of a word access at the
        // data register is somebody else's address.
        assert!(
            port(&video, Window::Crtc)
                .write(DATA, &[0u8; 2], MemAttrs::DEFAULT)
                .is_err()
        );
    }

    #[test]
    fn one_word_write_latches_the_index_and_the_datum() {
        // The idiom every VGA reference gives and every video BIOS emits:
        // `mov dx,3d4h; mov ax,(datum<<8)|index; out dx,ax`. Little-endian, so
        // the index is the low byte and is latched first — which is the whole
        // reason the pair is laid out this way.
        let video = device();
        port(&video, Window::Crtc)
            .write(ADDRESS, &[0x0c, 0x12], MemAttrs::DEFAULT)
            .expect("a word write to the index register is a VGA's own idiom");
        assert_eq!(
            video.shared.state.lock().crtc[0x0c],
            0x12,
            "R12, the start address high byte"
        );
        // And it reads back the same way: index in the low byte, datum in the
        // high one.
        let mut both = [0u8; 2];
        port(&video, Window::Crtc)
            .read(ADDRESS, &mut both, MemAttrs::DEFAULT)
            .expect("and a word read of the pair");
        assert_eq!(both, [0x0c, 0x12]);
    }

    #[test]
    fn the_regions_a_machine_file_maps_all_exist() {
        let video = device();
        for name in [
            "",
            "crtc",
            "crtc-colour",
            "crtc-mono",
            "status",
            "mode",
            "vga",
            "vram",
        ] {
            assert!(video.region(name).is_some(), "region {name}");
        }
        assert!(video.region("nonsense").is_none());
        assert_eq!(video.region("vram").expect("vram").len(), VRAM_LEN);
    }

    #[test]
    fn properties_are_checked_rather_than_ignored() {
        assert!(Video::new(&Props::new().with("dot-clock", 25_175_000u64)).is_ok());
        assert!(Video::new(&Props::new().with("dotclock", 1u64)).is_err());
    }

    #[test]
    fn a_code_point_with_no_glyph_is_visibly_missing() {
        // A control code is blank, an unmapped printable one is a box: the
        // difference between "nothing here" and "we cannot draw this".
        assert_eq!(glyph(0x01), &FONT_BLANK);
        assert_eq!(glyph(0xf0), &FONT_MISSING);
        assert_eq!(glyph(b'A'), &FONT_ASCII[(b'A' - FONT_FIRST) as usize]);
        // The box-drawing set a DOS installer frames its dialogues with.
        for code in [
            0xb3u8, 0xba, 0xc4, 0xcd, 0xda, 0xbf, 0xc0, 0xd9, 0xc9, 0xbb, 0xc8, 0xbc,
        ] {
            assert_ne!(glyph(code), &FONT_MISSING, "no glyph for {code:#04x}");
        }
    }
}
