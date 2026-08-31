//! The Game Boy's picture processing unit — the LCD controller, VRAM, OAM and
//! `$FF40`-`$FF4B`.
//!
//! A DMG frame is **70 224 crystal periods**: 154 lines of 456, of which the
//! first 144 are drawn and the last 10 are vertical blanking. Within a drawn
//! line the controller walks three modes, and which one it is in decides what
//! the CPU may touch:
//!
//! ```text
//!   dots   0- 79   mode 2  OAM scan     OAM unreadable
//!   dots  80-~251  mode 3  drawing      OAM *and* VRAM unreadable
//!   dots ~252-455  mode 0  H-blank      everything readable
//!   lines 144-153  mode 1  V-blank      everything readable
//! ```
//!
//! The `~` is the point. Mode 3 is **not** a fixed length: it is extended by up
//! to 7 dots for a non-zero `SCX & 7`, by 6 more when the window is switched on
//! part-way across the line, and by 6 to 11 for every object on the line. Mode 0
//! gets whatever is left. That variability is exactly what the accuracy suites
//! measure, and a PPU with a constant 172-dot mode 3 fails all of them
//! (`docs/platforms/game-boy.md`).
//!
//! # The interrupt lines are levels
//!
//! Two outputs: [`VBLANK_PIN`], high for the whole of mode 1, and [`STAT_PIN`],
//! high whenever **any** of the four conditions `STAT` enables is true. Both are
//! *levels*, and the CPU's pins latch their rising edges into `IF`
//! ([`crate::cpu::sm83`]). That is not a modelling convenience — it is what
//! hardware does, and it is where "STAT blocking" comes from: with both the
//! mode-0 and mode-2 interrupts enabled, the line never falls between them, so
//! the second raises nothing. Written as an edge on a wire, the behaviour is
//! free; written as "raise an interrupt when the mode changes", it is a bug that
//! takes a test ROM to find.
//!
//! # VRAM and OAM are inside this device
//!
//! Not RAM the machine file supplies. They are on the controller's side of the
//! bus and their accessibility depends on the mode, so the blocking rule lives
//! with the thing that knows the mode. Both are published as regions
//! ([`VRAM_REGION`], [`OAM_REGION`]) for a `map` statement to name.
//!
//! # OAM DMA lives here too
//!
//! `$FF46` is inside this register block on real hardware and the engine that
//! answers it is on this die. Writing it copies 160 bytes from `XX00` into OAM
//! over 160 machine cycles, one byte each. The source is read through the
//! **CPU's** address space, which is why this device is an [`Initiator`] and why
//! the machine file gives it `space = cpubus` — a device that could only respond
//! could not model it at all (`ROADMAP.md` §4.4).
//!
//! # Time
//!
//! **Lazily advanced** (`ROADMAP.md` §4.2), on the crystal's domain, one tick
//! per dot. [`GbPpu::next_event_tick`] reports the next tick at which the mode
//! or `LY` changes — which is every tick at which anything a program can read
//! out of `STAT` or `$FF44` changes — so a read landing between two events is
//! correct even though the device has not been dragged to the exact dot.
//!
//! # What is not modelled
//!
//! Written down rather than discovered:
//!
//! * **The pixel FIFO.** A line is rendered in one go when mode 3 begins, from
//!   the registers as they stand at that instant. Mid-scanline `SCX`/`SCY`/
//!   `LCDC` changes therefore do not bend the picture the way they do on
//!   hardware. Per-*line* effects — the status bars and parallax that a mode-0
//!   `STAT` interrupt produces — work correctly, because the interrupt fires
//!   before the next line is drawn.
//! * **The mode-3 penalty for objects** is Pan Docs' documented approximation,
//!   not a fetcher simulation.
//! * **`STAT`'s DMG write bug** — writing `STAT` briefly reads all conditions as
//!   true and can raise a spurious interrupt.
//! * **The first frame after the LCD is switched on** is a normal frame here;
//!   on hardware it is shorter and mode 0 is reported for its first line.
//!
//! # Sources
//!
//! [Pan Docs](https://gbdev.io/pandocs/) (CC0) — *LCDC*, *STAT*, *Rendering*,
//! *Pixel FIFO*, *OAM DMA Transfer*, *Palettes*. No emulator source was
//! consulted.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, Initiator, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
    RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU32, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};

/// How many crystal periods one scanline takes.
pub const DOTS_PER_LINE: u64 = 456;

/// How many lines a frame has, blanking included.
pub const LINES_PER_FRAME: u64 = 154;

/// How many crystal periods one frame takes: 456 x 154.
pub const DOTS_PER_FRAME: u64 = DOTS_PER_LINE * LINES_PER_FRAME;

/// The first line of vertical blanking.
pub const VBLANK_LINE: u8 = 144;

/// Visible width, in pixels.
pub const SCREEN_WIDTH: usize = 160;

/// Visible height, in pixels.
pub const SCREEN_HEIGHT: usize = 144;

/// How many entries the framebuffer holds.
pub const FRAMEBUFFER_LEN: usize = SCREEN_WIDTH * SCREEN_HEIGHT;

/// How long mode 2, the OAM scan, lasts. Fixed, unlike mode 3.
pub const OAM_SCAN_DOTS: u64 = 80;

/// The shortest mode 3 can be: no scroll, no window, no objects.
pub const MODE3_MIN_DOTS: u64 = 172;

/// How many bytes of video RAM the console has.
pub const VRAM_LEN: u64 = 0x2000;

/// How many bytes of object attribute memory it has: 40 objects of four bytes.
pub const OAM_LEN: u64 = 0xa0;

/// Where video RAM sits in the CPU's address space.
pub const VRAM_BASE: u64 = 0x8000;

/// Where object attribute memory sits.
pub const OAM_BASE: u64 = 0xfe00;

/// Where the register block sits.
pub const REGISTER_BASE: u64 = 0xff40;

/// How many registers it covers: `$FF40`-`$FF4B`.
pub const REGISTER_LEN: u64 = 12;

/// The name a `map` statement reaches video RAM by.
pub const VRAM_REGION: &str = "vram";

/// The name a `map` statement reaches object attribute memory by.
pub const OAM_REGION: &str = "oam";

/// The name a `map` statement reaches `$FF40`-`$FF4B` by.
pub const REGISTER_REGION: &str = "regs";

/// The vertical-blank interrupt output pin. High for the whole of mode 1.
pub const VBLANK_PIN: &str = "vblank";

/// The LCD status interrupt output pin. High while any enabled `STAT` condition
/// holds.
pub const STAT_PIN: &str = "stat";

/// How many machine cycles an OAM DMA takes: one per byte.
pub const DMA_BYTES: u64 = 160;

/// How many crystal periods each of those machine cycles is.
const CLOCKS_PER_MCYCLE: u64 = 4;

/// How many machine cycles pass between the write to `$FF46` and the first byte
/// of the transfer.
///
/// Two, and it is observable rather than an implementation detail. Gekkio's
/// `oam_dma_start` states the sequence a DMG runs and reports it verified on
/// every model of the family:
///
/// ```text
///   M = 0   the write to $FF46 happens
///   M = 1   nothing yet — OAM is still accessible
///   M = 2   the transfer starts, and OAM reads return $FF
/// ```
///
/// Which makes the blocked window `[W+2, W+161]` for a write on cycle `W`, and
/// that pair of numbers is what half of Gekkio's instruction-timing group is
/// built on: each of them aligns a memory access against the *end* of a
/// transfer, so an emulator whose window is two cycles early fails all of them
/// and one whose window is the wrong length fails half.
const DMA_START_DELAY: u64 = 2;

/// The four LCD modes, in the order a drawn line visits them.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Mode {
    /// Horizontal blanking. Everything is readable.
    HBlank,
    /// Vertical blanking. Everything is readable.
    VBlank,
    /// The OAM scan. OAM is not readable.
    OamScan,
    /// Drawing. Neither OAM nor VRAM is readable.
    Drawing,
}

impl Mode {
    /// The two bits `STAT` reports this mode as.
    #[must_use]
    pub const fn bits(self) -> u8 {
        match self {
            Mode::HBlank => 0,
            Mode::VBlank => 1,
            Mode::OamScan => 2,
            Mode::Drawing => 3,
        }
    }
}

/// `LCDC` (`$FF40`) bit names.
pub mod lcdc {
    /// Bit 0: on a DMG, whether the background and window are drawn at all.
    pub const BG_ENABLE: u8 = 0x01;
    /// Bit 1: whether objects are drawn.
    pub const OBJ_ENABLE: u8 = 0x02;
    /// Bit 2: objects are 8x16 rather than 8x8.
    pub const OBJ_TALL: u8 = 0x04;
    /// Bit 3: the background's tile map is at `$9C00` rather than `$9800`.
    pub const BG_MAP: u8 = 0x08;
    /// Bit 4: tile data is the unsigned `$8000` block rather than the signed
    /// `$8800` one.
    pub const TILE_DATA: u8 = 0x10;
    /// Bit 5: whether the window is drawn.
    pub const WINDOW_ENABLE: u8 = 0x20;
    /// Bit 6: the window's tile map is at `$9C00` rather than `$9800`.
    pub const WINDOW_MAP: u8 = 0x40;
    /// Bit 7: whether the LCD is running at all.
    pub const LCD_ENABLE: u8 = 0x80;
}

/// `STAT` (`$FF41`) bit names.
pub mod stat {
    /// Bit 3: raise the status interrupt on entering mode 0.
    pub const HBLANK_INT: u8 = 0x08;
    /// Bit 4: raise it on entering mode 1.
    pub const VBLANK_INT: u8 = 0x10;
    /// Bit 5: raise it on entering mode 2.
    pub const OAM_INT: u8 = 0x20;
    /// Bit 6: raise it while `LY` equals `LYC`.
    pub const LYC_INT: u8 = 0x40;
    /// Bit 2: set while `LY` equals `LYC`. Read-only.
    pub const LYC_EQUAL: u8 = 0x04;
    /// The bits a program may write.
    pub const WRITABLE: u8 = 0x78;
}

/// The controller's architectural state.
///
/// One struct rather than several, because nearly every field is read by the
/// renderer and a lock per group would be four locks taken in the same order
/// every time.
#[derive(Clone)]
struct Engine {
    /// Video RAM, `$8000`-`$9FFF`.
    vram: Vec<u8>,
    /// Object attribute memory, `$FE00`-`$FE9F`.
    oam: Vec<u8>,
    /// The finished picture: one shade, 0 (lightest) to 3 (darkest), per pixel.
    fb: Vec<u8>,

    lcdc: u8,
    /// Only the four enable bits are stored; the mode and the coincidence flag
    /// are computed.
    stat: u8,
    scy: u8,
    scx: u8,
    ly: u8,
    lyc: u8,
    bgp: u8,
    obp0: u8,
    obp1: u8,
    wy: u8,
    wx: u8,
    /// The last value written to `$FF46`, which reads back.
    dma_source: u8,

    /// Dots into the current line, 0..456.
    dot: u64,
    /// Dots since reset — this device's tick.
    dots: u64,
    /// Frames completed.
    frame: u64,
    /// How long mode 3 lasts on the line being drawn.
    mode3_len: u64,
    /// The window's own line counter, which advances only on lines where the
    /// window was actually drawn.
    window_line: u8,
    /// Whether the window has been drawn on this frame yet.
    window_active: bool,

    /// Bytes of the OAM transfer still to copy, or zero when idle.
    dma_remaining: u64,
    /// The dot at which the next transfer byte moves. One machine cycle apart.
    dma_next_dot: u64,
    /// The high byte of the address the transfer is reading from.
    dma_page: u8,
    /// The first dot at which object memory is blocked by a transfer.
    ///
    /// [`u64::MAX`] when none has ever run. Not simply the current transfer's
    /// start, because a write to `$FF46` while one is already running does not
    /// stop it (`oam_dma_restart`): the old transfer keeps the bus for the two
    /// cycles before the new one takes over, so the blocked window is
    /// continuous across the restart and this stays where the *first* of them
    /// put it.
    dma_block_from: u64,
    /// The last dot at which it is blocked: the dot the transfer's final byte
    /// moves on, which is still a blocked cycle.
    dma_block_until: u64,
}

impl fmt::Debug for Engine {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Engine")
            .field("lcdc", &self.lcdc)
            .field("ly", &self.ly)
            .field("dot", &self.dot)
            .field("mode", &self.mode())
            .finish_non_exhaustive()
    }
}

impl Engine {
    fn new() -> Engine {
        Engine {
            vram: vec![0; VRAM_LEN as usize],
            oam: vec![0; OAM_LEN as usize],
            fb: vec![0; FRAMEBUFFER_LEN],
            // The boot ROM leaves the LCD on with the background enabled and
            // the `$8000` tile block selected; a machine with no boot ROM has
            // to start somewhere, and here is where hardware would be.
            lcdc: lcdc::LCD_ENABLE | lcdc::BG_ENABLE | lcdc::TILE_DATA,
            stat: 0,
            scy: 0,
            scx: 0,
            ly: 0,
            lyc: 0,
            bgp: 0xfc,
            obp0: 0xff,
            obp1: 0xff,
            wy: 0,
            wx: 0,
            dma_source: 0xff,
            dot: 0,
            dots: 0,
            frame: 0,
            mode3_len: MODE3_MIN_DOTS,
            window_line: 0,
            window_active: false,
            dma_remaining: 0,
            dma_next_dot: 0,
            dma_page: 0,
            dma_block_from: u64::MAX,
            dma_block_until: 0,
        }
    }

    /// Start — or restart — an OAM transfer from page `page`.
    ///
    /// Called with `dots` standing on the cycle the write to `$FF46` was made
    /// on, which is what the two-cycle delay is measured from.
    fn arm_dma(&mut self, page: u8) {
        let start = self.dots + DMA_START_DELAY * CLOCKS_PER_MCYCLE;
        // A restart keeps the window open from wherever the transfer it is
        // displacing opened it: that one holds the bus until this one starts,
        // so there is no readable cycle in between.
        if !self.dma_blocking() {
            self.dma_block_from = start;
        }
        self.dma_block_until = start + (DMA_BYTES - 1) * CLOCKS_PER_MCYCLE;
        self.dma_page = page;
        self.dma_remaining = DMA_BYTES;
        self.dma_next_dot = start;
    }

    /// Whether a transfer owns the bus on the dot the controller stands on.
    fn dma_blocking(&self) -> bool {
        self.dots >= self.dma_block_from && self.dots <= self.dma_block_until
    }

    fn lcd_on(&self) -> bool {
        self.lcdc & lcdc::LCD_ENABLE != 0
    }

    /// Which mode the controller is in right now.
    fn mode(&self) -> Mode {
        if !self.lcd_on() {
            // With the LCD off the controller reports mode 0 and nothing is
            // blocked (Pan Docs, *LCDC*).
            return Mode::HBlank;
        }
        if self.ly >= VBLANK_LINE {
            return Mode::VBlank;
        }
        if self.dot < OAM_SCAN_DOTS {
            Mode::OamScan
        } else if self.dot < OAM_SCAN_DOTS + self.mode3_len {
            Mode::Drawing
        } else {
            Mode::HBlank
        }
    }

    /// `LY` as the guest reads it.
    ///
    /// The last line of the frame is the exception every accuracy suite tests:
    /// `LY` reads 153 for only the first four dots of line 153 and then reads
    /// **0** for the remaining 452, while the frame has not ended (Pan Docs,
    /// *Rendering*).
    fn visible_ly(&self) -> u8 {
        if !self.lcd_on() {
            return 0;
        }
        if self.ly == 153 && self.dot >= 4 {
            0
        } else {
            self.ly
        }
    }

    fn lyc_equal(&self) -> bool {
        self.lcd_on() && self.visible_ly() == self.lyc
    }

    /// `STAT` as the guest reads it: the stored enables, the coincidence flag,
    /// the mode, and bit 7 which is not implemented and reads as one.
    fn read_stat(&self) -> u8 {
        0x80 | (self.stat & stat::WRITABLE)
            | if self.lyc_equal() { stat::LYC_EQUAL } else { 0 }
            | self.mode().bits()
    }

    /// Whether the status interrupt line is asserted.
    ///
    /// The OR of every enabled condition — a *level*, which is what gives STAT
    /// blocking for free once the CPU's pin edge-detects it.
    fn stat_line(&self) -> bool {
        if !self.lcd_on() {
            return false;
        }
        let s = self.stat;
        (s & stat::LYC_INT != 0 && self.lyc_equal())
            || match self.mode() {
                Mode::HBlank => s & stat::HBLANK_INT != 0,
                Mode::VBlank => s & stat::VBLANK_INT != 0,
                Mode::OamScan => s & stat::OAM_INT != 0,
                Mode::Drawing => false,
            }
    }

    /// Whether the vertical-blank line is asserted.
    fn vblank_line(&self) -> bool {
        self.lcd_on() && self.ly >= VBLANK_LINE
    }

    fn vram_readable(&self) -> bool {
        self.mode() != Mode::Drawing
    }

    fn oam_readable(&self) -> bool {
        !matches!(self.mode(), Mode::Drawing | Mode::OamScan) && !self.dma_blocking()
    }

    // -- mode 3's length ----------------------------------------------------

    /// How long mode 3 will last on the line about to be drawn.
    ///
    /// Pan Docs, *Rendering*: 172 dots minimum, plus `SCX & 7` for the pixels
    /// discarded at the left edge, plus 6 when the window starts part-way
    /// across, plus a per-object penalty. The object penalty is Pan Docs' own
    /// approximation — 6 dots each, and up to 5 more for the first object
    /// landing in a given background tile — rather than a fetcher simulation.
    fn compute_mode3(&self) -> u64 {
        let mut len = MODE3_MIN_DOTS + u64::from(self.scx & 7);
        if self.window_visible_on_line() {
            len += 6;
        }
        if self.lcdc & lcdc::OBJ_ENABLE != 0 {
            let objects = self.objects_on_line();
            let mut last_tile = u16::MAX;
            for obj in &objects {
                let tile = (u16::from(obj.x).wrapping_add(u16::from(self.scx))) / 8;
                if tile == last_tile {
                    len += 6;
                } else {
                    len += 11 - u64::from(obj.x.wrapping_add(self.scx) & 7).min(5);
                    last_tile = tile;
                }
            }
        }
        // Mode 0 must still exist: hardware's mode 3 never runs past dot 369.
        len.min(DOTS_PER_LINE - OAM_SCAN_DOTS - 4)
    }

    fn window_visible_on_line(&self) -> bool {
        self.lcdc & lcdc::WINDOW_ENABLE != 0
            && self.lcdc & lcdc::BG_ENABLE != 0
            && self.ly >= self.wy
            && self.wx <= 166
    }

    // -- rendering ----------------------------------------------------------

    fn vram_byte(&self, addr: u16) -> u8 {
        self.vram[(addr as usize) & (VRAM_LEN as usize - 1)]
    }

    /// The two bits `palette` maps colour index `index` to.
    fn shade(palette: u8, index: u8) -> u8 {
        (palette >> (index * 2)) & 3
    }

    /// The colour index of one pixel of one tile.
    fn tile_pixel(&self, tile_addr: u16, x: u8, y: u8) -> u8 {
        let row = tile_addr + u16::from(y % 8) * 2;
        let lo = self.vram_byte(row);
        let hi = self.vram_byte(row + 1);
        let bit = 7 - (x % 8);
        ((hi >> bit) & 1) << 1 | ((lo >> bit) & 1)
    }

    /// Where a background or window tile's data starts.
    fn tile_address(&self, index: u8) -> u16 {
        if self.lcdc & lcdc::TILE_DATA != 0 {
            0x8000 + u16::from(index) * 16
        } else {
            // The `$8800` block is addressed by a *signed* index from `$9000`,
            // which is why tile 255 is below tile 0 rather than above it.
            (0x9000i32 + i32::from(index as i8) * 16) as u16
        }
    }

    /// Draw the whole of line `LY` into the framebuffer.
    ///
    /// Called once, when mode 3 begins. See the module documentation for what
    /// that costs in fidelity and what it does not.
    fn render_line(&mut self) {
        let ly = self.ly;
        if usize::from(ly) >= SCREEN_HEIGHT {
            return;
        }
        let row = usize::from(ly) * SCREEN_WIDTH;
        // The colour *index* of each background pixel, kept because object
        // priority is decided against the index rather than the shade.
        let mut bg_index = [0u8; SCREEN_WIDTH];

        let bg_on = self.lcdc & lcdc::BG_ENABLE != 0;
        let window_here = self.window_visible_on_line();
        let mut window_drawn = false;

        // The index is written into `bg_index` and the shade into `fb`, so the
        // loop walks the former by reference and reaches the latter by offset.
        #[allow(clippy::needless_range_loop)]
        for x in 0..SCREEN_WIDTH {
            let index = if !bg_on {
                // On a DMG, LCDC bit 0 clear blanks the background *and* the
                // window; objects still draw over the blank.
                0
            } else if window_here && (x as i16) >= i16::from(self.wx) - 7 {
                window_drawn = true;
                let wx = (x as i16 - (i16::from(self.wx) - 7)) as u16;
                let wy = u16::from(self.window_line);
                let map = if self.lcdc & lcdc::WINDOW_MAP != 0 {
                    0x9c00
                } else {
                    0x9800
                };
                let tile = self.vram_byte(map + (wy / 8) * 32 + (wx / 8));
                self.tile_pixel(self.tile_address(tile), wx as u8, wy as u8)
            } else {
                let bx = (x as u16 + u16::from(self.scx)) & 0xff;
                let by = (u16::from(ly) + u16::from(self.scy)) & 0xff;
                let map = if self.lcdc & lcdc::BG_MAP != 0 {
                    0x9c00
                } else {
                    0x9800
                };
                let tile = self.vram_byte(map + (by / 8) * 32 + (bx / 8));
                self.tile_pixel(self.tile_address(tile), bx as u8, by as u8)
            };
            bg_index[x] = index;
            self.fb[row + x] = Engine::shade(self.bgp, index);
        }

        if window_drawn {
            // The window's line counter advances only on lines it appeared on,
            // which is what lets a game move `WY` mid-frame without tearing.
            self.window_line = self.window_line.wrapping_add(1);
            self.window_active = true;
        }

        if self.lcdc & lcdc::OBJ_ENABLE == 0 {
            return;
        }
        let height: u8 = if self.lcdc & lcdc::OBJ_TALL != 0 {
            16
        } else {
            8
        };
        let mut objects = self.objects_on_line();
        // DMG priority: the smaller `x` wins, and OAM order breaks the tie. The
        // scan already produced OAM order, so a *stable* sort by `x` is exactly
        // the rule. Drawn back to front so the winner overwrites.
        objects.sort_by_key(|o| core::cmp::Reverse(o.x));
        for obj in objects {
            let mut line = ly.wrapping_add(16).wrapping_sub(obj.y);
            if obj.attrs & 0x40 != 0 {
                line = height - 1 - line;
            }
            let tile = if height == 16 {
                // A tall object's low tile bit is ignored; the two halves are
                // consecutive tiles.
                (obj.tile & 0xfe) + u8::from(line >= 8)
            } else {
                obj.tile
            };
            let addr = 0x8000 + u16::from(tile) * 16;
            let palette = if obj.attrs & 0x10 != 0 {
                self.obp1
            } else {
                self.obp0
            };
            for px in 0..8u8 {
                let sx = i16::from(obj.x) - 8 + i16::from(px);
                if sx < 0 || sx >= SCREEN_WIDTH as i16 {
                    continue;
                }
                let sx = sx as usize;
                let tx = if obj.attrs & 0x20 != 0 { 7 - px } else { px };
                let index = self.tile_pixel(addr, tx, line % 8);
                // Colour 0 is transparent for objects — always, whatever the
                // palette maps it to.
                if index == 0 {
                    continue;
                }
                // Attribute bit 7: the background wins wherever it is not
                // colour 0.
                if obj.attrs & 0x80 != 0 && bg_index[sx] != 0 {
                    continue;
                }
                self.fb[row + sx] = Engine::shade(palette, index);
            }
        }
    }

    /// The objects the OAM scan would select for the current line: the first ten
    /// in OAM order whose rows overlap it.
    fn objects_on_line(&self) -> Vec<Object> {
        let height: i16 = if self.lcdc & lcdc::OBJ_TALL != 0 {
            16
        } else {
            8
        };
        let ly = i16::from(self.ly);
        let mut out = Vec::with_capacity(10);
        for i in 0..40usize {
            let y = self.oam[i * 4];
            let top = i16::from(y) - 16;
            if ly < top || ly >= top + height {
                continue;
            }
            out.push(Object {
                y,
                x: self.oam[i * 4 + 1],
                tile: self.oam[i * 4 + 2],
                attrs: self.oam[i * 4 + 3],
            });
            // Ten per line, and the eleventh is simply not fetched.
            if out.len() == 10 {
                break;
            }
        }
        out
    }

    // -- the dot pipeline ---------------------------------------------------

    /// Advance one dot, returning whether a frame just ended.
    fn step_dot(&mut self) -> bool {
        self.dots += 1;
        if !self.lcd_on() {
            return false;
        }
        let entering_mode3 = self.ly < VBLANK_LINE && self.dot == OAM_SCAN_DOTS - 1;
        self.dot += 1;
        if entering_mode3 {
            // The line's registers are sampled here, once, and the line is
            // drawn from them. See the module documentation.
            self.mode3_len = self.compute_mode3();
            self.render_line();
        }
        if self.dot < DOTS_PER_LINE {
            return false;
        }
        self.dot = 0;
        self.ly += 1;
        if u64::from(self.ly) < LINES_PER_FRAME {
            if self.ly < VBLANK_LINE {
                self.mode3_len = MODE3_MIN_DOTS;
            }
            return false;
        }
        self.ly = 0;
        self.frame += 1;
        self.window_line = 0;
        self.window_active = false;
        true
    }

    /// The dot offset within the line at which the mode or `LY` next changes.
    ///
    /// `None` when the LCD is off, in which case nothing changes at all until a
    /// register write turns it back on.
    fn next_boundary(&self) -> Option<u64> {
        if !self.lcd_on() {
            return None;
        }
        let candidates: [u64; 4] = if self.ly < VBLANK_LINE {
            [
                OAM_SCAN_DOTS,
                OAM_SCAN_DOTS + self.mode3_len,
                DOTS_PER_LINE,
                DOTS_PER_LINE,
            ]
        } else if self.ly == 153 {
            // Line 153's `LY` changes from 153 to 0 four dots in, and a program
            // can see that.
            [4, DOTS_PER_LINE, DOTS_PER_LINE, DOTS_PER_LINE]
        } else {
            [DOTS_PER_LINE; 4]
        };
        candidates
            .into_iter()
            .filter(|c| *c > self.dot)
            .min()
            .map(|c| c - self.dot)
    }

    // -- registers ----------------------------------------------------------

    fn read_register(&self, index: u8) -> u8 {
        match index {
            0x00 => self.lcdc,
            0x01 => self.read_stat(),
            0x02 => self.scy,
            0x03 => self.scx,
            0x04 => self.visible_ly(),
            0x05 => self.lyc,
            0x06 => self.dma_source,
            0x07 => self.bgp,
            0x08 => self.obp0,
            0x09 => self.obp1,
            0x0a => self.wy,
            0x0b => self.wx,
            // The block is twelve bytes and the region is twelve bytes, so this
            // is unreachable; answering with the idle bus level is still the
            // only honest thing to do.
            _ => 0xff,
        }
    }

    /// Write a register. Returns the DMA page if this write started a transfer.
    fn write_register(&mut self, index: u8, value: u8) -> Option<u8> {
        match index {
            0x00 => {
                let was_on = self.lcd_on();
                self.lcdc = value;
                let now_on = self.lcd_on();
                if was_on && !now_on {
                    // Switching the LCD off resets the position: `LY` reads 0
                    // and the controller reports mode 0 (Pan Docs, *LCDC*).
                    self.ly = 0;
                    self.dot = 0;
                    self.window_line = 0;
                    self.window_active = false;
                } else if !was_on && now_on {
                    self.ly = 0;
                    self.dot = 0;
                    self.mode3_len = self.compute_mode3();
                }
            }
            0x01 => self.stat = value & stat::WRITABLE,
            0x02 => self.scy = value,
            0x03 => self.scx = value,
            // `LY` is read-only. A write is ignored rather than faulting: the
            // register is simply not connected to the bus in that direction.
            0x04 => {}
            0x05 => self.lyc = value,
            0x06 => {
                self.dma_source = value;
                return Some(value);
            }
            0x07 => self.bgp = value,
            0x08 => self.obp0 = value,
            0x09 => self.obp1 = value,
            0x0a => self.wy = value,
            0x0b => self.wx = value,
            _ => {}
        }
        None
    }
}

/// One entry of the object attribute table, as the scan produced it.
#[derive(Debug, Clone, Copy)]
struct Object {
    y: u8,
    x: u8,
    tile: u8,
    attrs: u8,
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// Where the engine lock sits in the ladder: **between** [`LockRank::BUS`] and
/// [`LockRank::DEVICE`].
///
/// The same squeeze the NES PPU documents, for the same two reasons. It is
/// reached from under a `BUS`-ranked lock, because sync-on-access fires from
/// inside `MemOps::read` while the CPU holds its own execution state at `BUS`.
/// And it must be *above* `DEVICE`, because an OAM DMA reaches the cartridge —
/// a `DEVICE`-ranked device — through the CPU's own address space.
///
/// So it goes in the gap the named ranks were spaced `0x1000` apart to leave.
pub const ENGINE_LOCK_RANK: LockRank = LockRank::new(0x4800);

struct Shared {
    engine: Mutex<Engine>,
    links: Mutex<Links>,
    lazy: Mutex<Option<LazyHandle>>,
    /// The CPU's address space, for OAM DMA. A DMA reads through the *bus*, not
    /// through some private port, which is why this device is an initiator.
    space: Mutex<Option<Arc<AddressSpace>>>,
    /// [`Engine::dots`], republished on every release of the engine lock.
    dots: AtomicU64,
    /// The tick of this device's own next event, republished alongside.
    next_event: AtomicU64,
    /// `u64::MAX` when there is no next event, which is what an LCD that is
    /// switched off has.
    requester: AtomicU32,
}

#[derive(Debug, Default)]
struct Links {
    vblank: Option<WireSource>,
    stat: Option<WireSource>,
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
        let event = match engine.next_boundary() {
            Some(delta) => engine.dots + delta,
            None => u64::MAX,
        };
        self.next_event.store(event, Ordering::Relaxed);
    }

    /// Drive both output lines, with no lock of this device held.
    fn drive(&self, vblank: bool, stat: bool) {
        let (v, s) = {
            let links = self.links.lock();
            (links.vblank.clone(), links.stat.clone())
        };
        if let Some(v) = v {
            v.set(Level::from_bool(vblank));
        }
        if let Some(s) = s {
            s.set(Level::from_bool(stat));
        }
    }

    /// Run `f` against the engine, then settle both lines outside the lock.
    ///
    /// The re-entrancy contract in one function (`ROADMAP.md` §4.4).
    fn with_engine<R>(&self, f: impl FnOnce(&mut Engine) -> R) -> R {
        let (result, vblank, stat) = {
            let mut engine = self.engine.lock();
            let result = f(&mut engine);
            self.publish(&engine);
            (result, engine.vblank_line(), engine.stat_line())
        };
        self.drive(vblank, stat);
        result
    }

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

/// The Game Boy's LCD controller.
pub struct GbPpu {
    shared: Arc<Shared>,
    vram_region: RegionRef,
    oam_region: RegionRef,
    regs_region: RegionRef,
}

impl fmt::Debug for GbPpu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("GbPpu")
            .field("engine", &self.shared.engine)
            .finish_non_exhaustive()
    }
}

impl Default for GbPpu {
    fn default() -> Self {
        GbPpu::new()
    }
}

impl GbPpu {
    /// A controller in its power-on state.
    #[must_use]
    pub fn new() -> GbPpu {
        let engine = Engine::new();
        let shared = Arc::new(Shared {
            engine: Mutex::with_rank(ENGINE_LOCK_RANK, engine),
            links: Mutex::with_rank(LockRank::WIRE, Links::default()),
            lazy: Mutex::new(None),
            space: Mutex::new(None),
            dots: AtomicU64::new(0),
            next_event: AtomicU64::new(OAM_SCAN_DOTS),
            requester: AtomicU32::new(0),
        });
        let vram_region = Arc::new(MmioRegion::io(
            "gb.ppu.vram",
            VRAM_LEN,
            Arc::new(VideoRam {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        let oam_region = Arc::new(MmioRegion::io(
            "gb.ppu.oam",
            OAM_LEN,
            Arc::new(ObjectRam {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        let regs_region = Arc::new(MmioRegion::io(
            "gb.ppu.regs",
            REGISTER_LEN,
            Arc::new(LcdPort {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        GbPpu {
            shared,
            vram_region,
            oam_region,
            regs_region,
        }
    }

    /// Build one from machine-description properties. It takes none yet.
    ///
    /// # Errors
    ///
    /// If any property was given at all.
    pub fn from_props(props: &Props) -> Result<GbPpu> {
        props.reader().finish()?;
        Ok(GbPpu::new())
    }

    /// Connect the CPU's address space, which an OAM DMA reads through.
    pub fn attach_space(&self, space: Arc<AddressSpace>) {
        *self.shared.space.lock() = Some(space);
    }

    /// Connect the vertical-blank request line.
    pub fn attach_vblank(&self, source: WireSource) {
        self.shared.links.lock().vblank = Some(source);
    }

    /// Connect the LCD status request line.
    pub fn attach_stat(&self, source: WireSource) {
        self.shared.links.lock().stat = Some(source);
    }

    /// Connect the catch-up handle the register block syncs through.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// Dots executed since reset.
    #[must_use]
    pub fn dots(&self) -> u64 {
        self.shared.dots.load(Ordering::Relaxed)
    }

    /// Frames completed since reset.
    #[must_use]
    pub fn frame(&self) -> u64 {
        self.shared.engine.lock().frame
    }

    /// The line being drawn, and how far into it, as `(LY, dot)`.
    #[must_use]
    pub fn position(&self) -> (u8, u64) {
        let engine = self.shared.engine.lock();
        (engine.ly, engine.dot)
    }

    /// Which mode the controller is in.
    #[must_use]
    pub fn mode(&self) -> Mode {
        self.shared.engine.lock().mode()
    }

    /// Read one register by index, 0-11, without catching up — for a test.
    #[must_use]
    pub fn read_register(&self, index: u8) -> u8 {
        self.shared.engine.lock().read_register(index)
    }

    /// Write one register by index, 0-11, without catching up — for a test.
    pub fn write_register(&self, index: u8, value: u8) {
        let page = self.shared.with_engine(|e| e.write_register(index, value));
        if let Some(page) = page {
            self.start_dma(page);
        }
    }

    /// Read one byte of video RAM, ignoring the mode's blocking — for a test or
    /// a monitor.
    #[must_use]
    pub fn peek_vram(&self, offset: u64) -> u8 {
        let engine = self.shared.engine.lock();
        engine.vram[(offset as usize) % engine.vram.len()]
    }

    /// Write one byte of video RAM, ignoring the mode's blocking.
    pub fn poke_vram(&self, offset: u64, value: u8) {
        let mut engine = self.shared.engine.lock();
        let len = engine.vram.len();
        engine.vram[(offset as usize) % len] = value;
    }

    /// Read one byte of object attribute memory, ignoring blocking.
    #[must_use]
    pub fn peek_oam(&self, offset: u64) -> u8 {
        let engine = self.shared.engine.lock();
        engine.oam[(offset as usize) % engine.oam.len()]
    }

    /// Write one byte of object attribute memory, ignoring blocking.
    pub fn poke_oam(&self, offset: u64, value: u8) {
        let mut engine = self.shared.engine.lock();
        let len = engine.oam.len();
        engine.oam[(offset as usize) % len] = value;
    }

    /// Borrow the framebuffer: [`SCREEN_WIDTH`] x [`SCREEN_HEIGHT`] shades,
    /// row-major from the top left, 0 lightest and 3 darkest.
    ///
    /// A callback rather than a slice because the buffer lives behind the engine
    /// lock, and handing out a borrow of it would either leak the guard or copy
    /// 23 kB nobody asked for.
    pub fn with_framebuffer<R>(&self, f: impl FnOnce(&[u8]) -> R) -> R {
        let engine = self.shared.engine.lock();
        f(&engine.fb)
    }

    /// One framebuffer pixel, or `None` off the screen.
    #[must_use]
    pub fn pixel(&self, x: usize, y: usize) -> Option<u8> {
        if x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT {
            return None;
        }
        Some(self.shared.engine.lock().fb[y * SCREEN_WIDTH + x])
    }

    /// Begin an OAM transfer from page `page`, i.e. `page << 8`.
    ///
    /// A write to `$FF46` while a transfer is already running restarts it, which
    /// is what hardware does and what `oam_dma_restart` tests.
    fn start_dma(&self, page: u8) {
        self.shared.engine.lock().arm_dma(page);
    }

    /// Run the controller until `target` dots have elapsed in total.
    ///
    /// The catch-up entry point (`ROADMAP.md` §4.2). Running backwards is not an
    /// error, it is a no-op.
    pub fn advance_to(&self, target: u64) {
        // An OAM transfer reads through the CPU's bus, so its reads happen with
        // **no lock of this device held** — otherwise a transfer whose source is
        // this device's own video RAM would meet its own engine lock. Every byte
        // due in this span is fetched first and applied below; the transfer and
        // the dot pipeline share the ticks but nothing else.
        let pending = self.pump_dma(target);
        let (vblank, stat) = {
            let mut engine = self.shared.engine.lock();
            for (index, byte) in pending {
                let len = engine.oam.len();
                engine.oam[(index as usize) % len] = byte;
            }
            while engine.dots < target {
                if !engine.lcd_on() {
                    // Nothing changes while the LCD is off, so there is nothing
                    // to step through.
                    engine.dots = target;
                    break;
                }
                engine.step_dot();
            }
            self.shared.publish(&engine);
            (engine.vblank_line(), engine.stat_line())
        };
        // Outward, and only once the critical section is released.
        self.shared.drive(vblank, stat);
    }

    /// Advance the OAM transfer up to `target`, returning the bytes to store.
    ///
    /// Reads are made through the CPU's address space with no lock held. A
    /// source inside video RAM is answered from this device's own array rather
    /// than through the bus, because on hardware the controller reaches VRAM
    /// directly and is not subject to its own blocking.
    fn pump_dma(&self, target: u64) -> Vec<(u64, u8)> {
        let (page, first, count) = {
            let engine = self.shared.engine.lock();
            if engine.dma_remaining == 0 || engine.dma_next_dot > target {
                return Vec::new();
            }
            // How many byte-cycles fall in `[next, target]`, inclusive: the same
            // convention the dot pipeline uses, where advancing to `T` means `T`
            // has happened.
            let span = target - engine.dma_next_dot;
            let count = (1 + span / CLOCKS_PER_MCYCLE).min(engine.dma_remaining);
            let first = DMA_BYTES - engine.dma_remaining;
            (engine.dma_page, first, count)
        };
        let space = self.shared.space.lock().clone();
        let attrs = MemAttrs::DEFAULT
            .with_requester(RequesterId(self.shared.requester.load(Ordering::Relaxed)));
        let mut out = Vec::with_capacity(count as usize);
        for i in 0..count {
            let index = first + i;
            let addr = (u64::from(page) << 8) | index;
            let byte = if (VRAM_BASE..VRAM_BASE + VRAM_LEN).contains(&addr) {
                self.shared.engine.lock().vram[(addr - VRAM_BASE) as usize]
            } else {
                match &space {
                    Some(space) => space.read(addr, Width::U8, attrs).unwrap_or(0xff) as u8,
                    None => 0xff,
                }
            };
            out.push((index, byte));
        }
        {
            let mut engine = self.shared.engine.lock();
            engine.dma_remaining -= count;
            engine.dma_next_dot += count * CLOCKS_PER_MCYCLE;
        }
        out
    }

    /// Run exactly `dots` more dots.
    pub fn advance_by(&self, dots: u64) {
        let target = self.shared.dots.load(Ordering::Relaxed) + dots;
        self.advance_to(target);
    }
}

// ---------------------------------------------------------------------------
// The three memory windows
// ---------------------------------------------------------------------------

/// `$8000`-`$9FFF`. Unreadable and unwritable during mode 3.
struct VideoRam {
    shared: Arc<Shared>,
}

impl fmt::Debug for VideoRam {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("VideoRam").finish_non_exhaustive()
    }
}

impl MemOps for VideoRam {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        self.shared.sync(attrs);
        let engine = self.shared.engine.lock();
        // A debug read sees through the blocking: a monitor showing the tile map
        // during mode 3 should show the tile map, not `$FF`.
        *byte = if attrs.debug || engine.vram_readable() {
            engine.vram[(offset as usize) % engine.vram.len()]
        } else {
            0xff
        };
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        self.shared.sync(attrs);
        let mut engine = self.shared.engine.lock();
        if !attrs.debug && !engine.vram_readable() {
            // Dropped, not faulted: the write really does go nowhere, and every
            // Game Boy program relies on that being harmless.
            return Ok(());
        }
        let len = engine.vram.len();
        engine.vram[(offset as usize) % len] = *value;
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// `$FE00`-`$FE9F`. Unreadable during modes 2 and 3, and during an OAM DMA.
struct ObjectRam {
    shared: Arc<Shared>,
}

impl fmt::Debug for ObjectRam {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ObjectRam").finish_non_exhaustive()
    }
}

impl MemOps for ObjectRam {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        self.shared.sync(attrs);
        let engine = self.shared.engine.lock();
        *byte = if attrs.debug || engine.oam_readable() {
            engine.oam[(offset as usize) % engine.oam.len()]
        } else {
            0xff
        };
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        self.shared.sync(attrs);
        let mut engine = self.shared.engine.lock();
        if !attrs.debug && !engine.oam_readable() {
            return Ok(());
        }
        let len = engine.oam.len();
        engine.oam[(offset as usize) % len] = *value;
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// `$FF40`-`$FF4B`.
struct LcdPort {
    shared: Arc<Shared>,
}

impl fmt::Debug for LcdPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("LcdPort").finish_non_exhaustive()
    }
}

impl MemOps for LcdPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        // `LY` and `STAT` are the whole reason this device is lazily advanced.
        self.shared.sync(attrs);
        *byte = self.shared.engine.lock().read_register(offset as u8);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // Writing `$FF46` starts a DMA and writing `LCDC` restarts the
            // frame; neither is something a debugger may cause by accident.
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        let page = self
            .shared
            .with_engine(|e| e.write_register(offset as u8, *value));
        if let Some(page) = page {
            self.shared.engine.lock().arm_dma(page);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

/// The `gb.ppu` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "gb.ppu",
    version: 2,
    summary: "Game Boy LCD controller: VRAM, OAM, $FF40-$FF4B, OAM DMA",
    properties: &[],
    construct: |props| Ok(Box::new(GbPpu::from_props(props)?) as Box<dyn Device>),
};

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for GbPpu {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        self.shared
            .requester
            .store(ctx.requester().0, Ordering::Relaxed);
        // The realize sweep: leave both wires driving what the state implies
        // (`ROADMAP.md` §4.3).
        let (vblank, stat) = {
            let engine = self.shared.engine.lock();
            (engine.vblank_line(), engine.stat_line())
        };
        self.shared.drive(vblank, stat);
        Ok(())
    }

    fn unrealize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        self.shared.drive(false, false);
        let mut links = self.shared.links.lock();
        links.vblank = None;
        links.stat = None;
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            VRAM_REGION => Some(Arc::clone(&self.vram_region)),
            OAM_REGION => Some(Arc::clone(&self.oam_region)),
            REGISTER_REGION => Some(Arc::clone(&self.regs_region)),
            // No empty-name default: this device publishes three apertures and
            // a `map … = ppu` that silently picked one would be a coin toss.
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            VBLANK_PIN => self.attach_vblank(source),
            STAT_PIN => self.attach_stat(source),
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!(
                        "the LCD controller drives `{VBLANK_PIN}` and `{STAT_PIN}`, nothing else"
                    ),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, port: &str) {
        let (vblank, stat) = {
            let engine = self.shared.engine.lock();
            (engine.vblank_line(), engine.stat_line())
        };
        match port {
            VBLANK_PIN | STAT_PIN => self.shared.drive(vblank, stat),
            _ => {}
        }
    }

    fn reset(&self, _kind: ResetKind) {
        self.shared.with_engine(|e| *e = Engine::new());
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let engine = self.shared.engine.lock();
        w.write_bytes(&engine.vram)?;
        w.write_bytes(&engine.oam)?;
        w.write_bytes(&engine.fb)?;
        for byte in [
            engine.lcdc,
            engine.stat,
            engine.scy,
            engine.scx,
            engine.ly,
            engine.lyc,
            engine.bgp,
            engine.obp0,
            engine.obp1,
            engine.wy,
            engine.wx,
            engine.dma_source,
            engine.window_line,
            engine.dma_page,
        ] {
            w.write_u8(byte)?;
        }
        w.write_bool(engine.window_active)?;
        w.write_u64(engine.dot)?;
        w.write_u64(engine.dots)?;
        w.write_u64(engine.frame)?;
        w.write_u64(engine.mode3_len)?;
        w.write_u64(engine.dma_remaining)?;
        w.write_u64(engine.dma_next_dot)?;
        w.write_u64(engine.dma_block_from)?;
        w.write_u64(engine.dma_block_until)?;
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let vram = r.read_bytes()?;
        let oam = r.read_bytes()?;
        let fb = r.read_bytes()?;
        if vram.len() as u64 != VRAM_LEN
            || oam.len() as u64 != OAM_LEN
            || fb.len() != FRAMEBUFFER_LEN
        {
            return Err(Error::State(String::from(
                "the LCD controller's snapshot has the wrong memory sizes",
            )));
        }
        let (vblank, stat) = {
            let mut engine = self.shared.engine.lock();
            engine.vram.copy_from_slice(vram);
            engine.oam.copy_from_slice(oam);
            engine.fb.copy_from_slice(fb);
            engine.lcdc = r.read_u8()?;
            engine.stat = r.read_u8()?;
            engine.scy = r.read_u8()?;
            engine.scx = r.read_u8()?;
            engine.ly = r.read_u8()?;
            engine.lyc = r.read_u8()?;
            engine.bgp = r.read_u8()?;
            engine.obp0 = r.read_u8()?;
            engine.obp1 = r.read_u8()?;
            engine.wy = r.read_u8()?;
            engine.wx = r.read_u8()?;
            engine.dma_source = r.read_u8()?;
            engine.window_line = r.read_u8()?;
            engine.dma_page = r.read_u8()?;
            engine.window_active = r.read_bool()?;
            engine.dot = r.read_u64()?;
            engine.dots = r.read_u64()?;
            engine.frame = r.read_u64()?;
            engine.mode3_len = r.read_u64()?;
            engine.dma_remaining = r.read_u64()?;
            engine.dma_next_dot = r.read_u64()?;
            engine.dma_block_from = r.read_u64()?;
            engine.dma_block_until = r.read_u64()?;
            // The dot counter and the next event both came from the snapshot;
            // the lock-free copies of them are derived state and must follow.
            self.shared.publish(&engine);
            (engine.vblank_line(), engine.stat_line())
        };
        // The restored state implies levels nothing has announced.
        self.shared.drive(vblank, stat);
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
        GbPpu::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        let event = self.shared.next_event.load(Ordering::Relaxed);
        (event != u64::MAX).then_some(event)
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        GbPpu::attach_lazy(self, handle);
    }
}

impl Initiator for GbPpu {
    fn requester(&self) -> RequesterId {
        RequesterId(self.shared.requester.load(Ordering::Relaxed))
    }
}

/// The machine layer's half: the controller needs the CPU's address space,
/// because an OAM DMA reads through it.
impl crate::machine::Instance for GbPpu {
    fn bind(&self, ctx: &crate::machine::BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "the LCD controller needs the CPU's address space to run an OAM DMA through: \
                 add `space = cpubus` to the object",
            ),
        })?;
        self.attach_space(Arc::clone(space));
        self.shared
            .requester
            .store(ctx.requester().0, Ordering::Relaxed);
        Ok(())
    }
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| Ok(Arc::new(GbPpu::from_props(props)?)))
}

/// What the validator should know about `gb.ppu`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir};
    ClassSchema::new(CLASS.name)
        .port(VBLANK_PIN, PortDir::Out)
        .port(STAT_PIN, PortDir::Out)
        .region(VRAM_REGION)
        .region(OAM_REGION)
        .region(REGISTER_REGION)
}
