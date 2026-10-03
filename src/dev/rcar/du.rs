//! The R-Car Display Unit (DU): a minimal model of the display controller
//! that turns a guest framebuffer into a picture.
//!
//! # Sources
//!
//! The Renesas *R-Car Series User's Manual: Hardware*, "Display Unit (DU)"
//! chapter, as the author knows it — the register names, offsets and the
//! handful of bit fields named below — and, as the **primary** source for
//! what a real guest does with it, a black-box trace of the register writes
//! Renesas' Linux 2.6.35 `rcarfb` driver makes on an R-Car H1 (R8A7779) head
//! unit. No kernel, emulator or boot loader source of any licence was
//! consulted (`ROADMAP.md` §1). Where this file had to choose, it says so, and
//! the choices are collected under *Uncertain* below.
//!
//! # What is modelled
//!
//! ```text
//!   0x00  DSYSR   display system control: DEN (bit 8), DRES (bit 9)
//!   0x04  DSMR    display mode (stored)
//!   0x08  DSSR    display status (read-only): FRM (14), VBK (11)
//!   0x0c  DSRCR   status clear: a 1 clears that DSSR bit (reads 0)
//!   0x10  DIER    interrupt enable, DSSR's layout: FRE (14), VBE (11)
//!   0x18  DPPR    plane priority: slot n (1..=8) is bits 4n-1 (DPEn, enable)
//!                 and 4n-4..4n-2 (DPSn, plane number minus one)
//!   0x11000 DORCR output routing: bit 0 set, display 1's planes come from
//!                 DS1PR instead of DPPR
//!   0x11020 DS1PR display 1's plane order: nibble k (k = 0 on top) is a
//!                 plane number, 1..=8, or 0 for none
//!   0x20, 0x34..0x3c  DEFR, DEFR2..DEFR4  (stored)
//!   0x40  HDSR  0x44 HDER  0x48 VDSR  0x4c VDER    the active window
//!   0x50  HCR   0x54 HSWR  0x58 VCR   0x5c VSPR    totals and syncs
//!   0x98  BPOR    background colour, 0x00RRGGBB (stored as written)
//!   0x100 + 0x100 × (n − 1): plane n, n = 1..=8
//!     +0x00 PnMR      mode: DDDF (1:0) pixel format, SPIM (14:12) blending
//!     +0x04 PnMWR     memory width, pixels per line of the source image
//!     +0x08 PnALPHAR  constant alpha (7:0) for SPIM = alpha blend
//!     +0x10 PnDSXR  +0x14 PnDSYR   displayed size
//!     +0x18 PnDPXR  +0x1c PnDPYR   position on the display
//!     +0x20 PnDSA0R               framebuffer address
//!     +0x30 PnSPXR  +0x34 PnSPYR   start pixel within the source image
//!     +0x90 PnDDCR4               EDF (2:0): 1 = ARGB8888, 2 = RGB888 in 32 bits
//! ```
//!
//! **Every other offset in the 256 KiB window is plain storage**: it reads
//! back what was written, so a driver that programs registers this model does
//! not interpret (`CPCR`, `DOOR`, `PnDSA1R`, `PnBTR`, `PnDDCR`, the "code"
//! registers whose upper half is a write key, the colour palettes, …) sees its
//! own values and nothing else. Byte, halfword and word accesses are accepted,
//! naturally aligned, and reach only the lanes they cover.
//!
//! # Composition
//!
//! A captured frame is the **active window** — `(HDER − HDSR) × (VDER − VDSR)`,
//! 800 × 480 for the traced driver — filled with `BPOR` and then with every
//! plane `DPPR` enables, in priority order. A plane places `PnDSXR × PnDSYR`
//! pixels at `(PnDPXR, PnDPYR)` of the active window, read from `PnDSA0R`
//! starting at source pixel `(PnSPXR, PnSPYR)` of an image `PnMWR` pixels
//! wide. Until the timing registers describe a window the geometry is the
//! `width` × `height` properties.
//!
//! `DSYSR.DEN` set and `DSYSR.DRES` clear is "displaying"; otherwise the frame
//! is black and no frames are counted. The framebuffer is read **when a frame
//! is captured**, from whatever the registers hold then — tearing is honest,
//! for the reasons `lcd.scanout` gives.
//!
//! Pixel formats, from `PnMR.DDDF` and `PnDDCR4.EDF` ([`PlaneFormat`]):
//! `DDDF = 1` RGB565, `DDDF = 2` ARGB1555, `EDF = 1` ARGB8888 and `EDF = 2`
//! RGB888 in a 32-bit container, all little-endian as an ARM guest stores
//! them. `DDDF = 0` is 8-bit palette indices and the palette is not modelled
//! (the index is shown as a grey level); `DDDF = 3` is YCbCr, which is not
//! modelled and is **decoded as RGB565** — a wrong picture, not a panic.
//!
//! Blending: `PnMR` bit 12 (`SPIM` bit 0) blends with `PnALPHAR[7:0]`, or
//! with the pixel's own alpha for ARGB8888, and ARGB1555's `A = 0` is
//! transparent — `SPIM` 1 from the kernel's frame buffer driver, 5 from the
//! navi's HMI. A plane without it draws opaque; colour keying is not
//! modelled.
//!
//! The navi's HMI never touches `DPPR`: its display service sets `DORCR` bit
//! 0 and orders the planes through `DS1PR`, a plane number per nibble with
//! the top in nibble 0 (read off its own `duc.so`, disassembled as data).
//!
//! # Why not `lcd.scanout`
//!
//! The generic engine's frame period is fixed at construction and it has one
//! layer at the origin; the DU's timing is guest-programmed (the traced driver
//! writes two different timing sets) and it composes up to eight positioned
//! planes over a background. So the DU owns its own lazily advanced frame
//! counter and composition, and hands out RGB888 rows exactly as the engine
//! does, which is all [`crate::host::display`] needs.
//!
//! # Time
//!
//! **Lazily advanced** on this device's clock domain, one tick per **dot
//! clock**. One frame is `(HCR + 1) × (VCR + 1)` dots — 1176 × 525 for the
//! traced driver — counted from the tick the display was enabled or the
//! totals last changed. At each frame boundary `DSSR.FRM` and `DSSR.VBK` are
//! set; both are placed on the same boundary, which is within a frame of the
//! manual's timing and exact for a guest that waits for "the next frame".
//! `HBK` and the raster interrupt are not modelled. The scheduler is only told
//! about a boundary when an enabled interrupt would change there.
//!
//! One level output, `irq`, high while `DSSR & DIER & (FRM | VBK)` is non-zero.
//!
//! # Uncertain
//!
//! * `DSYSR`'s reset value is taken to be `DRES` (`0x200`): the traced driver
//!   reads it and then writes `0x100`, which fits a read-modify-write clearing
//!   `DRES` and setting `DEN`.
//! * `DPPR` priority slot 1 is taken to be the **top** layer and slot 8 the
//!   bottom. The traced driver uses one plane, in slot 8, so it cannot tell.
//! * `PnDPXR`/`PnDPYR` are taken as relative to the active window's top-left.
//!   The traced driver writes 16 and 9 for a full-screen 800 × 480 plane, which
//!   under this reading shifts the picture by (16, 9). The `plane-origin-x` /
//!   `plane-origin-y` properties name the plane coordinate the window starts
//!   at, so a board can correct that without a code change if hardware shows
//!   the reading is wrong.
//! * `PnDSA0R` is always the displayed buffer; `PnDSA1R`'s role in the
//!   automatic buffer-switching modes (`PnMR.BM`) is not modelled.
//! * Byte order is little-endian with no `PnSWAPR` handling: the traced driver
//!   never writes `PnSWAPR`.

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::string::String;
use alloc::sync::{Arc, Weak};
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.du";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x4_0000;

/// How many planes the DU composes.
pub const PLANES: usize = 8;

/// The interrupt output.
pub const IRQ_PIN: &str = "irq";

/// Display system control.
pub const DSYSR: u32 = 0x00;
/// Display mode.
pub const DSMR: u32 = 0x04;
/// Display status (read-only).
pub const DSSR: u32 = 0x08;
/// Display status clear (write-only).
pub const DSRCR: u32 = 0x0c;
/// Display interrupt enable.
pub const DIER: u32 = 0x10;
/// Display plane priority.
pub const DPPR: u32 = 0x18;
/// Output routing: bit 0 hands the planes' order to [`DS1PR`].
pub const DORCR: u32 = 0x1_1000;
/// Display 1's plane order: nibble k (0 = top) is a plane number, 0 = none.
pub const DS1PR: u32 = 0x1_1020;
/// `PnMR` bit 12 (`SPIM` bit 0): blend.
const PNMR_BLEND: u32 = 1 << 12;
/// Horizontal display start.
pub const HDSR: u32 = 0x40;
/// Horizontal display end.
pub const HDER: u32 = 0x44;
/// Vertical display start.
pub const VDSR: u32 = 0x48;
/// Vertical display end.
pub const VDER: u32 = 0x4c;
/// Horizontal cycle (total dots minus one).
pub const HCR: u32 = 0x50;
/// Horizontal sync width.
pub const HSWR: u32 = 0x54;
/// Vertical cycle (total lines minus one).
pub const VCR: u32 = 0x58;
/// Vertical sync position.
pub const VSPR: u32 = 0x5c;
/// Background plane output colour.
pub const BPOR: u32 = 0x98;

/// Where plane 1's registers start.
pub const PLANE_BASE: u32 = 0x100;
/// The distance between one plane's registers and the next's.
pub const PLANE_STRIDE: u32 = 0x100;

/// Plane mode, relative to the plane's base.
pub const PNMR: u32 = 0x00;
/// Plane memory width in pixels.
pub const PNMWR: u32 = 0x04;
/// Plane constant alpha.
pub const PNALPHAR: u32 = 0x08;
/// Plane display size X.
pub const PNDSXR: u32 = 0x10;
/// Plane display size Y.
pub const PNDSYR: u32 = 0x14;
/// Plane display position X.
pub const PNDPXR: u32 = 0x18;
/// Plane display position Y.
pub const PNDPYR: u32 = 0x1c;
/// Plane display start address 0.
pub const PNDSA0R: u32 = 0x20;
/// Plane start position X in the source image.
pub const PNSPXR: u32 = 0x30;
/// Plane start position Y in the source image.
pub const PNSPYR: u32 = 0x34;
/// Plane data control 4: the extended data format.
pub const PNDDCR4: u32 = 0x90;

/// `DSYSR.DEN`: display enable.
pub const DSYSR_DEN: u32 = 1 << 8;
/// `DSYSR.DRES`: display reset.
pub const DSYSR_DRES: u32 = 1 << 9;
/// `DSSR.FRM`: a frame ended.
pub const DSSR_FRM: u32 = 1 << 14;
/// `DSSR.VBK`: vertical blanking began.
pub const DSSR_VBK: u32 = 1 << 11;
/// The status bits this model sets.
const DSSR_MODELLED: u32 = DSSR_FRM | DSSR_VBK;

/// No geometry larger than this is believed; a timing register full of
/// garbage must not make a capture allocate gigabytes.
const MAX_DIM: u32 = 4096;

/// Tick value meaning "no event".
const NEVER: u64 = u64::MAX;

/// The register offset of plane `n`'s (1-based) register `reg`.
#[must_use]
pub const fn plane_reg(n: u32, reg: u32) -> u32 {
    PLANE_BASE + PLANE_STRIDE * (n - 1) + reg
}

// ---------------------------------------------------------------------------
// Pixel formats
// ---------------------------------------------------------------------------

/// How a plane's framebuffer stores a pixel, as `PnMR.DDDF` and
/// `PnDDCR4.EDF` select it. The extensible-newtype pattern (`CLAUDE.md`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct PlaneFormat(pub u16);

impl PlaneFormat {
    /// Two bytes, little-endian `RRRRRGGG GGGBBBBB`.
    pub const RGB565: PlaneFormat = PlaneFormat(0);
    /// Two bytes, little-endian `ARRRRRGG GGGBBBBB`.
    pub const ARGB1555: PlaneFormat = PlaneFormat(1);
    /// Four bytes, little-endian `0xAARRGGBB`.
    pub const ARGB8888: PlaneFormat = PlaneFormat(2);
    /// Four bytes, little-endian `0xXXRRGGBB`: RGB888 in a 32-bit container.
    pub const XRGB8888: PlaneFormat = PlaneFormat(3);
    /// One byte, a palette index. The palette is not modelled.
    pub const INDEX8: PlaneFormat = PlaneFormat(4);

    /// What `PnMR` and `PnDDCR4` select. YCbCr (`DDDF = 3`) is not modelled
    /// and comes back as RGB565; see the module docs.
    #[must_use]
    pub const fn from_regs(pnmr: u32, pnddcr4: u32) -> PlaneFormat {
        match pnddcr4 & 7 {
            1 => return PlaneFormat::ARGB8888,
            2 => return PlaneFormat::XRGB8888,
            _ => {}
        }
        match pnmr & 3 {
            0 => PlaneFormat::INDEX8,
            2 => PlaneFormat::ARGB1555,
            _ => PlaneFormat::RGB565,
        }
    }

    /// How many bytes one pixel occupies.
    #[must_use]
    pub const fn bytes_per_pixel(self) -> u64 {
        match self {
            PlaneFormat::ARGB8888 | PlaneFormat::XRGB8888 => 4,
            PlaneFormat::INDEX8 => 1,
            _ => 2,
        }
    }

    /// One pixel from `bytes` as RGB888 and an alpha (255 opaque), before the
    /// plane's blending mode is applied.
    ///
    /// The 5- and 6-bit channels replicate their high bits into the low ones,
    /// so full scale is `0xff`, as `lcd.scanout` does it.
    #[must_use]
    pub fn decode(self, bytes: &[u8]) -> ([u8; 3], u8) {
        let five = |v: u16| {
            let v = (v & 0x1f) as u8;
            (v << 3) | (v >> 2)
        };
        match self {
            PlaneFormat::ARGB1555 => {
                let v = u16::from_le_bytes([bytes[0], bytes[1]]);
                let a = if v & 0x8000 != 0 { 0xff } else { 0 };
                ([five(v >> 10), five(v >> 5), five(v)], a)
            }
            PlaneFormat::ARGB8888 => ([bytes[2], bytes[1], bytes[0]], bytes[3]),
            PlaneFormat::XRGB8888 => ([bytes[2], bytes[1], bytes[0]], 0xff),
            PlaneFormat::INDEX8 => ([bytes[0]; 3], 0xff),
            // RGB565 and anything else.
            _ => {
                let v = u16::from_le_bytes([bytes[0], bytes[1]]);
                let g = ((v >> 5) & 0x3f) as u8;
                ([five(v >> 11), (g << 2) | (g >> 4), five(v)], 0xff)
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------

/// Everything the guest can see or change, and the frame anchor.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Regs {
    /// Every register, by word offset. A missing key reads zero; zero values
    /// are not stored, so the map (and a snapshot of it) is canonical.
    words: BTreeMap<u32, u32>,
    /// The tick the current run of frames is counted from.
    anchor: u64,
    /// Frames completed while displaying, since reset.
    frames: u64,
}

impl Regs {
    fn reset(tick: u64) -> Regs {
        let mut regs = Regs {
            words: BTreeMap::new(),
            anchor: tick,
            frames: 0,
        };
        regs.set(DSYSR, DSYSR_DRES);
        regs
    }

    fn get(&self, offset: u32) -> u32 {
        self.words.get(&offset).copied().unwrap_or(0)
    }

    fn set(&mut self, offset: u32, value: u32) {
        if value == 0 {
            self.words.remove(&offset);
        } else {
            self.words.insert(offset, value);
        }
    }

    fn displaying(&self) -> bool {
        self.get(DSYSR) & (DSYSR_DEN | DSYSR_DRES) == DSYSR_DEN
    }

    /// One frame in dots, or `None` until the totals are programmed.
    fn frame_ticks(&self) -> Option<u64> {
        let (hcr, vcr) = (self.get(HCR), self.get(VCR));
        if hcr == 0 || vcr == 0 {
            return None;
        }
        Some((u64::from(hcr) + 1).saturating_mul(u64::from(vcr) + 1))
    }

    /// The frame period, but only while frames are being counted.
    fn counting(&self) -> Option<u64> {
        if self.displaying() {
            self.frame_ticks()
        } else {
            None
        }
    }

    /// Whole frames from the anchor to `tick`.
    fn done(&self, period: u64, tick: u64) -> u64 {
        tick.saturating_sub(self.anchor) / period
    }

    /// The active window from the timing registers, if they describe one.
    fn active(&self) -> Option<(u32, u32)> {
        let w = self.get(HDER).checked_sub(self.get(HDSR))?;
        let h = self.get(VDER).checked_sub(self.get(VDSR))?;
        (w > 0 && h > 0 && w <= MAX_DIM && h <= MAX_DIM).then_some((w, h))
    }

    fn level(&self) -> bool {
        self.get(DSSR) & self.get(DIER) & DSSR_MODELLED != 0
    }

    /// The tick of the next frame boundary that would raise the interrupt.
    fn next_event(&self, now: u64) -> u64 {
        let Some(period) = self.counting() else {
            return NEVER;
        };
        let enabled = self.get(DIER) & DSSR_MODELLED;
        if enabled == 0 || self.get(DSSR) & enabled != 0 {
            return NEVER;
        }
        let next = self.done(period, now).saturating_add(1);
        self.anchor.saturating_add(next.saturating_mul(period))
    }
}

/// What one plane contributes, captured from the registers in one go.
#[derive(Debug, Clone, Copy)]
struct PlaneView {
    format: PlaneFormat,
    /// `PnMR` bit 12: blend.
    blend: bool,
    alpha: u8,
    mem_width: u64,
    width: u32,
    height: u32,
    x: i64,
    y: i64,
    base: u64,
    src_x: u64,
    src_y: u64,
}

/// A consistent view of everything a captured frame depends on.
#[derive(Debug, Clone)]
struct FrameView {
    on: bool,
    width: u32,
    height: u32,
    background: [u8; 3],
    /// Bottom first.
    planes: Vec<PlaneView>,
}

/// What the register block and the device share.
struct Shared {
    regs: Mutex<Regs>,
    irq: Mutex<Option<WireSource>>,
    lazy: Mutex<Option<LazyHandle>>,
    /// The address space the framebuffer lives in. **Weak**, for the reason
    /// `lcd.scanout`'s `Shared::bus` gives: the DU's own registers are mapped
    /// into that space, and a strong handle would close a cycle nothing
    /// breaks. Derived from the machine graph, never serialized.
    bus: Mutex<Option<Weak<AddressSpace>>>,
    requester: Mutex<RequesterId>,
    /// The tick reached, readable without the register lock.
    tick: AtomicU64,
    /// [`Regs::next_event`], republished with every change.
    next_event: AtomicU64,
    /// Frames completed, for a host that asks every redraw.
    frames: AtomicU64,
    /// The geometry shown until the timing registers describe a window.
    fallback: (u32, u32),
    /// The plane coordinate the active window's top-left corner is at.
    origin: (i64, i64),
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        match self.regs.try_lock() {
            Some(regs) => s.field("regs", &*regs),
            None => s.field("regs", &"<in use>"),
        };
        s.field("tick", &self.tick.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Shared {
    fn publish(&self, regs: &Regs) {
        let now = self.tick.load(Ordering::Relaxed);
        self.next_event
            .store(regs.next_event(now), Ordering::Relaxed);
        self.frames.store(regs.frames, Ordering::Relaxed);
    }

    /// Drive `irq`. Never called with the register lock held.
    fn drive(&self, level: bool) {
        let out = self.irq.lock().clone();
        if let Some(out) = out {
            out.set(Level::from_bool(level));
        }
    }

    fn advance_to(&self, target: u64) {
        let level = {
            let mut regs = self.regs.lock();
            let now = self.tick.load(Ordering::Relaxed);
            if target > now {
                if let Some(period) = regs.counting() {
                    let passed = regs.done(period, target) - regs.done(period, now);
                    if passed > 0 {
                        regs.frames = regs.frames.saturating_add(passed);
                        let dssr = regs.get(DSSR) | DSSR_MODELLED;
                        regs.set(DSSR, dssr);
                    }
                }
                self.tick.store(target, Ordering::Relaxed);
            }
            self.publish(&regs);
            regs.level()
        };
        self.drive(level);
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

    /// Write the bits `mask` of the word at `slot`, at tick `now`.
    fn write_register(regs: &mut Regs, slot: u32, value: u32, mask: u32, now: u64) {
        let before = (regs.displaying(), regs.frame_ticks());
        match slot {
            // Read-only: the status flags only change through DSRCR.
            DSSR => {}
            DSRCR => {
                let dssr = regs.get(DSSR) & !(value & mask);
                regs.set(DSSR, dssr);
            }
            _ => {
                let merged = (regs.get(slot) & !mask) | (value & mask);
                regs.set(slot, merged);
            }
        }
        // Enabling the display, or changing the totals, restarts the frame
        // count here: the sync generator starts a new frame.
        if (regs.displaying(), regs.frame_ticks()) != before {
            regs.anchor = now;
        }
    }

    fn view(&self) -> FrameView {
        let regs = self.regs.lock();
        let (width, height) = regs.active().unwrap_or(self.fallback);
        let bpor = regs.get(BPOR);
        let background = [(bpor >> 16) as u8, (bpor >> 8) as u8, bpor as u8];
        // The planes to draw, bottom first. With DORCR bit 0 set the display
        // takes its order from DS1PR, a plane number per nibble with slot 0
        // on top; otherwise from DPPR, slot 8 at the bottom (see *Uncertain*).
        let order: Vec<u32> = if regs.get(DORCR) & 1 != 0 {
            let ds1pr = regs.get(DS1PR);
            (0..8u32)
                .rev()
                .map(|k| (ds1pr >> (4 * k)) & 0xf)
                .filter(|n| (1..=8).contains(n))
                .collect()
        } else {
            let dppr = regs.get(DPPR);
            (1..=8u32)
                .rev()
                .filter(|slot| dppr & (1 << (4 * slot - 1)) != 0)
                .map(|slot| ((dppr >> (4 * slot - 4)) & 7) + 1)
                .collect()
        };
        let mut planes = Vec::new();
        for n in order {
            let r = |reg| regs.get(plane_reg(n, reg));
            let pnmr = r(PNMR);
            let width = r(PNDSXR).min(MAX_DIM);
            let mem_width = match r(PNMWR) {
                0 => u64::from(width),
                w => u64::from(w),
            };
            planes.push(PlaneView {
                format: PlaneFormat::from_regs(pnmr, r(PNDDCR4)),
                blend: pnmr & PNMR_BLEND != 0,
                alpha: r(PNALPHAR) as u8,
                mem_width,
                width,
                height: r(PNDSYR).min(MAX_DIM),
                x: i64::from(r(PNDPXR)) - self.origin.0,
                y: i64::from(r(PNDPYR)) - self.origin.1,
                base: u64::from(r(PNDSA0R)),
                src_x: u64::from(r(PNSPXR)),
                src_y: u64::from(r(PNSPYR)),
            });
        }
        FrameView {
            on: regs.displaying(),
            width,
            height,
            background,
            planes,
        }
    }
}

/// Mix `src` over `dst` by `alpha` out of 255, rounding.
fn blend(dst: [u8; 3], src: [u8; 3], alpha: u8) -> [u8; 3] {
    let a = u32::from(alpha);
    core::array::from_fn(|i| {
        ((u32::from(src[i]) * a + u32::from(dst[i]) * (255 - a) + 127) / 255) as u8
    })
}

/// Compose row `y` of `view` into `dst` (`view.width` long).
fn compose_row(
    view: &FrameView,
    bus: Option<&AddressSpace>,
    attrs: MemAttrs,
    y: u32,
    dst: &mut [[u8; 3]],
) {
    if !view.on {
        dst.fill([0, 0, 0]);
        return;
    }
    dst.fill(view.background);
    let Some(bus) = bus else {
        return;
    };
    let mut raw = Vec::new();
    for plane in &view.planes {
        let row = i64::from(y) - plane.y;
        if row < 0 || row >= i64::from(plane.height) {
            continue;
        }
        // The columns of the window this plane covers.
        let x0 = plane.x.max(0);
        let x1 = (plane.x + i64::from(plane.width)).min(dst.len() as i64);
        if x1 <= x0 {
            continue;
        }
        let count = (x1 - x0) as u64;
        let bpp = plane.format.bytes_per_pixel();
        let first = plane.src_x + (x0 - plane.x) as u64;
        let line = plane.src_y + row as u64;
        let addr = plane.base.wrapping_add(
            line.wrapping_mul(plane.mem_width)
                .wrapping_add(first)
                .wrapping_mul(bpp),
        );
        // One read per plane per line, not one per pixel.
        raw.clear();
        raw.resize((count * bpp) as usize, 0);
        if bus.read_bytes(addr, &mut raw, attrs).is_err() {
            continue;
        }
        for (i, out) in dst[x0 as usize..x1 as usize].iter_mut().enumerate() {
            let (rgb, own) = plane.format.decode(&raw[i * bpp as usize..]);
            let alpha = if !plane.blend {
                0xff
            } else if plane.format == PlaneFormat::ARGB8888 {
                own
            } else if own == 0 {
                0
            } else {
                plane.alpha
            };
            *out = match alpha {
                0xff => rgb,
                0 => *out,
                a => blend(*out, rgb, a),
            };
        }
    }
}

/// The byte-lane mask of an access of `len` bytes at `offset`.
fn lane_mask(offset: u64, len: usize) -> u32 {
    let bytes = if len >= 4 {
        0xffff_ffff
    } else {
        (1u32 << (len * 8)) - 1
    };
    bytes << ((offset & 3) * 8)
}

/// The register block.
struct Port {
    shared: Arc<Shared>,
}

impl fmt::Debug for Port {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Port").finish_non_exhaustive()
    }
}

impl MemOps for Port {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.is_empty() || dst.len() > 4 || offset >= REGISTER_WINDOW_LEN {
            return Err(BusError::BadAccess);
        }
        // Catch up first (a debug access observes without advancing), and
        // outside every lock this device owns. Reads have no side effects of
        // their own: DSSR is cleared only through DSRCR.
        self.shared.sync(attrs);
        let slot = (offset & !3) as u32;
        let value = match slot {
            DSRCR => 0,
            _ => self.shared.regs.lock().get(slot),
        };
        let bytes = (value >> ((offset & 3) * 8)).to_le_bytes();
        dst.copy_from_slice(&bytes[..dst.len()]);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.is_empty() || src.len() > 4 || offset >= REGISTER_WINDOW_LEN {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debugger write could clear a status flag or move the
            // framebuffer under the guest (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        let mut value = 0u32;
        for (i, byte) in src.iter().enumerate() {
            value |= u32::from(*byte) << (i * 8);
        }
        let shift = (offset & 3) * 8;
        let now = self.shared.tick.load(Ordering::Relaxed);
        let level = {
            let mut regs = self.shared.regs.lock();
            Shared::write_register(
                &mut regs,
                (offset & !3) as u32,
                value << shift,
                lane_mask(offset, src.len()),
                now,
            );
            self.shared.publish(&regs);
            regs.level()
        };
        self.shared.drive(level);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_natural_alignment(true)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An R-Car Display Unit.
pub struct Du {
    shared: Arc<Shared>,
    region: RegionRef,
}

impl fmt::Debug for Du {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Du")
            .field("shared", &self.shared)
            .finish_non_exhaustive()
    }
}

impl Du {
    /// Validate `props` and build the unit.
    ///
    /// Properties, all optional:
    ///
    /// * `width`, `height` — the geometry shown until the guest programs the
    ///   timing registers. Default 640 × 480.
    /// * `plane-origin-x`, `plane-origin-y` — the plane coordinate
    ///   (`PnDPXR`/`PnDPYR`) at which the active window starts. Default 0.
    ///
    /// The dot clock is the device's clock domain (`clock = …`) and the
    /// framebuffer's address space is `space = …`, both bound by the machine.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property, [`Error::Config`] for a
    /// zero or absurd fallback geometry.
    pub fn new(props: &Props) -> Result<Du> {
        let mut r = props.reader();
        let width: u64 = r.or("width", 640)?;
        let height: u64 = r.or("height", 480)?;
        let origin_x: u64 = r.or("plane-origin-x", 0)?;
        let origin_y: u64 = r.or("plane-origin-y", 0)?;
        r.finish()?;
        let max = u64::from(MAX_DIM);
        if width == 0 || height == 0 || width > max || height > max {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!(
                    "the fallback geometry is {width}x{height}; each side must be 1..={max}"
                ),
            });
        }
        if origin_x > max || origin_y > max {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: alloc::format!("a plane origin is a pixel position, at most {max}"),
            });
        }
        Ok(Du::build(
            (width as u32, height as u32),
            (origin_x as i64, origin_y as i64),
        ))
    }

    fn build(fallback: (u32, u32), origin: (i64, i64)) -> Du {
        let shared = Arc::new(Shared {
            regs: Mutex::with_rank(LockRank::DEVICE, Regs::reset(0)),
            irq: Mutex::with_rank(LockRank::WIRE, None),
            lazy: Mutex::new(None),
            bus: Mutex::with_rank(LockRank::WIRE, None),
            requester: Mutex::with_rank(LockRank::WIRE, RequesterId::ANONYMOUS),
            tick: AtomicU64::new(0),
            next_event: AtomicU64::new(NEVER),
            frames: AtomicU64::new(0),
            fallback,
            origin,
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::new(Port {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        Du { shared, region }
    }

    /// A register's value, without catching up — for a test or a monitor.
    #[must_use]
    pub fn reg(&self, offset: u32) -> u32 {
        self.shared.regs.lock().get(offset & !3)
    }

    /// The geometry a captured frame has: the active window, or the fallback.
    #[must_use]
    pub fn geometry(&self) -> (u32, u32) {
        self.shared
            .regs
            .lock()
            .active()
            .unwrap_or(self.shared.fallback)
    }

    /// Whether the display is on (`DEN` set, `DRES` clear).
    #[must_use]
    pub fn displaying(&self) -> bool {
        self.shared.regs.lock().displaying()
    }

    /// Frames completed while displaying. Lock-free.
    #[must_use]
    pub fn frames(&self) -> u64 {
        self.shared.frames.load(Ordering::Relaxed)
    }

    /// One frame in dot-clock ticks, `(HCR + 1) × (VCR + 1)`, or 0 before the
    /// totals are programmed.
    #[must_use]
    pub fn frame_ticks(&self) -> u64 {
        self.shared.regs.lock().frame_ticks().unwrap_or(0)
    }

    /// Whether `irq` is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.shared.regs.lock().level()
    }

    /// Compose the whole frame from one consistent view of the registers.
    ///
    /// Returns `(width, height, pixels)`, the pixels row-major RGB888. Black
    /// when the display is off; the background colour where no plane is, and
    /// where a plane's memory cannot be read.
    #[must_use]
    pub fn read_frame(&self) -> (u32, u32, Vec<[u8; 3]>) {
        let view = self.shared.view();
        let bus = self.shared.bus.lock().clone();
        let bus = bus.as_ref().and_then(Weak::upgrade);
        let attrs = self.attrs();
        let (w, h) = (view.width, view.height);
        let mut pixels = vec![[0u8; 3]; w as usize * h as usize];
        if w > 0 {
            for (y, row) in pixels.chunks_mut(w as usize).enumerate() {
                compose_row(&view, bus.as_deref(), attrs, y as u32, row);
            }
        }
        (w, h, pixels)
    }

    /// The attributes a framebuffer fetch carries: `debug`, because a capture
    /// is the host looking, not the guest making an access, and must not
    /// perturb whatever the plane happens to point at.
    fn attrs(&self) -> MemAttrs {
        MemAttrs {
            requester: *self.shared.requester.lock(),
            debug: true,
            ..MemAttrs::DEFAULT
        }
    }

    /// Run the unit until `tick` dots have passed in total.
    pub fn advance_to(&self, tick: u64) {
        self.shared.advance_to(tick);
    }
}

/// The `rcar.du` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Renesas R-Car Display Unit: composes up to eight planes from guest memory",
    properties: &[
        PropertySpec {
            name: "width",
            kind: ValueKind::Uint,
            required: false,
            summary: "pixels across until the guest programs HDSR/HDER (default 640)",
        },
        PropertySpec {
            name: "height",
            kind: ValueKind::Uint,
            required: false,
            summary: "lines down until the guest programs VDSR/VDER (default 480)",
        },
        PropertySpec {
            name: "plane-origin-x",
            kind: ValueKind::Uint,
            required: false,
            summary: "the PnDPXR value that is the active window's left edge (default 0)",
        },
        PropertySpec {
            name: "plane-origin-y",
            kind: ValueKind::Uint,
            required: false,
            summary: "the PnDPYR value that is the active window's top edge (default 0)",
        },
    ],
    construct: |props| Ok(Box::new(Du::new(props)?)),
};

impl Device for Du {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the space arrives in `Instance::bind`, and `irq`
        // idles low, which a fresh net already is.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // The tick is the clock domain's position and is not rewound.
        let now = self.shared.tick.load(Ordering::Relaxed);
        {
            let mut regs = self.shared.regs.lock();
            *regs = Regs::reset(now);
            self.shared.publish(&regs);
        }
        self.shared.drive(false);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: String::from(port),
                message: String::from("the DU drives one output, `irq`"),
            });
        }
        *self.shared.irq.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == IRQ_PIN {
            let level = self.shared.regs.lock().level();
            self.shared.drive(level);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let regs = self.shared.regs.lock().clone();
        w.write_u64(self.shared.tick.load(Ordering::Relaxed))?;
        w.write_u64(regs.anchor)?;
        w.write_u64(regs.frames)?;
        w.write_u32(regs.words.len() as u32)?;
        for (offset, value) in &regs.words {
            w.write_u32(*offset)?;
            w.write_u32(*value)?;
        }
        // The framebuffer is guest RAM and is saved by whoever owns it; the
        // address space and the wire are rebuilt by realize.
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let tick = r.read_u64()?;
        let mut regs = Regs::reset(0);
        regs.words.clear();
        regs.anchor = r.read_u64()?;
        regs.frames = r.read_u64()?;
        let count = r.read_u32()?;
        for _ in 0..count {
            let offset = r.read_u32()?;
            let value = r.read_u32()?;
            if offset & 3 != 0 || u64::from(offset) >= REGISTER_WINDOW_LEN {
                return Err(Error::Config {
                    at: String::from(CLASS_NAME),
                    message: alloc::format!("a snapshot names register {offset:#x}"),
                });
            }
            regs.set(offset, value);
        }
        let level = {
            let mut slot = self.shared.regs.lock();
            *slot = regs;
            self.shared.tick.store(tick, Ordering::Relaxed);
            self.shared.publish(&slot);
            slot.level()
        };
        self.shared.drive(level);
        Ok(())
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) --------------------------------

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.shared.advance_to(tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        let next = self.shared.next_event.load(Ordering::Relaxed);
        (next != NEVER).then_some(next)
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for Du {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "the DU is a bus master and needs the address space its framebuffers live in \
                 (`space = mem`)",
            ),
        })?;
        // Weak: see `Shared::bus`.
        *self.shared.bus.lock() = Some(Arc::downgrade(space));
        *self.shared.requester.lock() = ctx.requester();
        Ok(())
    }
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Du::new(props)?)))
}

/// What the validator should know about `rcar.du`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    let max = u64::from(MAX_DIM);
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("width", ValueKind::Uint).range(1, max))
        .prop(PropSchema::new("height", ValueKind::Uint).range(1, max))
        .prop(PropSchema::new("plane-origin-x", ValueKind::Uint).range(0, max))
        .prop(PropSchema::new("plane-origin-y", ValueKind::Uint).range(0, max))
        .region("")
        .region("regs")
        .port(IRQ_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests;
