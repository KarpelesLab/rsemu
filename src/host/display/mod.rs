//! The scanout seam: a guest surface a host can look at (`ROADMAP.md` §8).
//!
//! A display device produces pixels in whatever the silicon actually emits — a
//! 2C02 emits a 6-bit palette index and three emphasis bits, a VGA card emits
//! an 8-bit index into its DAC, a virtio-gpu emits ARGB already. **Converting
//! that to something a screen wants is the host's job**, not the device's, and
//! this module is where it happens. Nothing under `dev/` may name a colour.
//!
//! | Type | Role |
//! | --- | --- |
//! | [`PixelFormat`] | how bytes are laid out in a host surface |
//! | [`SurfaceInfo`] | geometry and preferred format, without the pixels |
//! | [`Surface`] | the frame buffer itself: `stride × height` bytes the host reads |
//! | [`Scanout`] | what a display device offers: "here is my latest frame" |
//!
//! # Shape
//!
//! ```text
//!   device (dev/)          seam (here)              host
//!   ─────────────          ───────────              ────
//!   NesPpu ──► Pixel(u16) ─► NesScanout ─► Surface ─► png::encode  → a file
//!                             (Scanout)              → wasm exports → a canvas
//! ```
//!
//! The device side is one small adapter per display device
//! ([`nes::NesScanout`] is the first, [`lcd::LcdScanout`] the second); the host
//! side never learns which machine
//! it is looking at. A Game Boy's LCD, a VGA card and a virtio-gpu each add an
//! adapter and nothing else changes.
//!
//! # Why a callback-free `Surface` rather than a borrowed slice
//!
//! The host owns the surface and hands it to [`Scanout::capture`] to be filled.
//! That keeps the device's own buffer behind its own lock (the PPU's is inside
//! the engine mutex), gives the host a buffer with a stable address across
//! frames — which is what a `<canvas>` upload and a `SharedArrayBuffer` both
//! want — and lets the host pick the byte order it needs without the device
//! knowing about any of it.
//!
//! # Example
//!
//! ```
//! use rsemu::host::display::{PixelFormat, Surface};
//!
//! let mut surface = Surface::new(PixelFormat::RGBA8888, 4, 2);
//! surface.fill([0, 0, 0]);
//! surface.put(1, 0, [0xff, 0x80, 0x00]);
//! assert_eq!(surface.get(1, 0), Some([0xff, 0x80, 0x00]));
//! assert_eq!(surface.pixels().len(), 4 * 2 * 4);
//! ```
//!
//! # Where the pixels go
//!
//! Two consumers exist today and neither of them is a window:
//!
//! * [`png`] captures a surface as a PNG, or a sequence of them as an APNG —
//!   headless, so CI and `docs/` get real screenshots of a real run.
//! * [`crate::wasm`] hands the surface's address to a `<canvas>`; `web/` is the
//!   page that draws it.
//!
//! **There is deliberately no native window.** `ROADMAP.md` §8 wants X11,
//! Wayland, Win32 and macOS backends eventually, and the dependency policy
//! rules out every GUI crate that would make them short (`CLAUDE.md`), so each
//! one is the wire protocol by hand: X11 means the connection handshake,
//! `.Xauthority` parsing, `MIT-MAGIC-COOKIE-1`, and `PutImage` per frame;
//! Wayland means passing a shared-memory file descriptor over a Unix socket,
//! which `std` cannot do at all without `unsafe` — and a seventh `unsafe`
//! subsystem is a design review rather than a commit (§0). It is a few hundred
//! lines of protocol that cannot be unit-tested into correctness, so it is its
//! own piece of work rather than a corner of this one. Until then a picture is
//! a PNG or a browser tab, and both are real.
//!
//! # Units
//!
//! Widths and heights are pixel counts and are `u32`: they are properties of a
//! *host* surface, not guest addresses, and every raster format on earth stores
//! them in 32 bits. Byte counts — [`Surface::stride`], [`Surface::len`] — are
//! `u64` as `CLAUDE.md` requires, and are converted to `usize` only where the
//! buffer is actually indexed.

pub mod palette;

#[cfg(feature = "dev-lcdc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-lcdc")))]
pub mod lcd;

#[cfg(feature = "dev-nes-ppu")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-ppu")))]
pub mod nes;

#[cfg(feature = "dev-pc-video")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-pc-video")))]
pub mod pc;

#[cfg(feature = "dev-sms")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-sms")))]
pub mod sms;

#[cfg(feature = "display-png")]
#[cfg_attr(docsrs, doc(cfg(feature = "display-png")))]
pub mod png;

#[cfg(test)]
mod tests;

use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

// ---------------------------------------------------------------------------
// Pixel formats
// ---------------------------------------------------------------------------

/// How a [`Surface`]'s bytes are laid out.
///
/// An extensible enumeration in the `pktkit` style rather than a Rust `enum`
/// (`CLAUDE.md`): a host backend that needs `RGB565` or a 30-bit format adds a
/// constant without breaking every `match` in the tree. The named constants are
/// **memory order**, not the word order of some particular endianness, because
/// a frame buffer is a byte array that gets memcpy'd into a canvas or a shared
/// buffer, and word order is a property of nobody's hardware here.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(transparent)]
pub struct PixelFormat(pub u16);

impl PixelFormat {
    /// Bytes `R`, `G`, `B`, `A`. What `ImageData` and WebGL want, so it is the
    /// browser demo's format and the default preference of every adapter here.
    pub const RGBA8888: PixelFormat = PixelFormat(0);

    /// Bytes `B`, `G`, `R`, `A`. What an X11 `ZPixmap` and a Win32 DIB want on
    /// a little-endian host.
    pub const BGRA8888: PixelFormat = PixelFormat(1);

    /// Bytes `R`, `G`, `B` with no padding. PNG's colour type 2.
    pub const RGB888: PixelFormat = PixelFormat(2);

    /// How many bytes one pixel occupies.
    #[inline]
    #[must_use]
    pub const fn bytes_per_pixel(self) -> u64 {
        match self {
            PixelFormat::RGB888 => 3,
            _ => 4,
        }
    }

    /// Whether this format has an alpha byte the host should fill in.
    #[inline]
    #[must_use]
    pub const fn has_alpha(self) -> bool {
        matches!(self, PixelFormat::RGBA8888 | PixelFormat::BGRA8888)
    }

    /// A short name for diagnostics.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            PixelFormat::RGBA8888 => "rgba8888",
            PixelFormat::BGRA8888 => "bgra8888",
            PixelFormat::RGB888 => "rgb888",
            _ => "unknown",
        }
    }

    /// Where the red, green and blue bytes sit within one pixel.
    ///
    /// The single place byte order is decided; every writer and reader below
    /// goes through it, so adding a format is one arm here and one in
    /// [`bytes_per_pixel`](PixelFormat::bytes_per_pixel).
    #[inline]
    const fn channel_offsets(self) -> [usize; 3] {
        match self {
            PixelFormat::BGRA8888 => [2, 1, 0],
            // RGBA8888, RGB888 and anything unknown: R, G, B in order. An
            // unknown format is treated as RGBA rather than panicking — a host
            // that invents a constant gets a wrong picture, not a crash in the
            // middle of a frame.
            _ => [0, 1, 2],
        }
    }
}

impl fmt::Display for PixelFormat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

// ---------------------------------------------------------------------------
// Surfaces
// ---------------------------------------------------------------------------

/// A surface's shape, without its pixels.
///
/// What a [`Scanout`] answers when asked what it produces, so a host can
/// allocate before the first frame exists.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SurfaceInfo {
    /// Visible width in pixels.
    pub width: u32,
    /// Visible height in pixels.
    pub height: u32,
    /// The format the device converts into most cheaply. A host may ask for a
    /// different one; every adapter here supports all of them.
    pub preferred_format: PixelFormat,
}

impl SurfaceInfo {
    /// A description of a `width × height` surface in `format`.
    #[must_use]
    pub const fn new(width: u32, height: u32, format: PixelFormat) -> SurfaceInfo {
        SurfaceInfo {
            width,
            height,
            preferred_format: format,
        }
    }
}

/// A host-side frame buffer: `stride × height` bytes in one [`PixelFormat`].
///
/// Rows are tightly packed (`stride == width × bytes_per_pixel`) and the buffer
/// is allocated once, so the pointer [`Surface::as_ptr`] returns is stable
/// until the surface is resized — which is what an embedder handing the address
/// to JavaScript relies on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Surface {
    format: PixelFormat,
    width: u32,
    height: u32,
    stride: u64,
    pixels: Vec<u8>,
    serial: u64,
}

impl Surface {
    /// A black (all-zero) surface.
    ///
    /// A zero-sized surface is legal and holds no bytes: a machine with no
    /// display is a thing a host has to render nothing for, not an error.
    #[must_use]
    pub fn new(format: PixelFormat, width: u32, height: u32) -> Surface {
        let stride = u64::from(width) * format.bytes_per_pixel();
        let len = stride * u64::from(height);
        Surface {
            format,
            width,
            height,
            stride,
            // A frame buffer is bounded by the host's own address space, so the
            // cast is the one place a pixel count becomes an index.
            pixels: vec![0u8; len as usize],
            serial: 0,
        }
    }

    /// A surface with no pixels at all.
    ///
    /// `const`, because a host that keeps its frame buffer in a `static` — the
    /// wasm module does — needs one before any machine exists.
    #[must_use]
    pub const fn empty() -> Surface {
        Surface {
            format: PixelFormat::RGBA8888,
            width: 0,
            height: 0,
            stride: 0,
            pixels: Vec::new(),
            serial: 0,
        }
    }

    /// A surface shaped for what `scanout` produces, in its preferred format.
    #[must_use]
    pub fn for_scanout(scanout: &dyn Scanout) -> Surface {
        let info = scanout.info();
        Surface::new(info.preferred_format, info.width, info.height)
    }

    /// The format the bytes are in.
    #[inline]
    #[must_use]
    pub const fn format(&self) -> PixelFormat {
        self.format
    }

    /// Width in pixels.
    #[inline]
    #[must_use]
    pub const fn width(&self) -> u32 {
        self.width
    }

    /// Height in pixels.
    #[inline]
    #[must_use]
    pub const fn height(&self) -> u32 {
        self.height
    }

    /// Bytes per row.
    #[inline]
    #[must_use]
    pub const fn stride(&self) -> u64 {
        self.stride
    }

    /// Total bytes in the buffer.
    #[inline]
    #[must_use]
    pub fn len(&self) -> u64 {
        self.pixels.len() as u64
    }

    /// Whether the surface holds no pixels at all.
    #[inline]
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pixels.is_empty()
    }

    /// This surface's shape.
    #[must_use]
    pub const fn info(&self) -> SurfaceInfo {
        SurfaceInfo::new(self.width, self.height, self.format)
    }

    /// Which frame these pixels are, as counted by whoever filled it.
    ///
    /// [`Scanout::capture`] sets it to the producing device's frame counter, so
    /// a host can tell a fresh frame from a repeat without comparing 180 kB.
    #[inline]
    #[must_use]
    pub const fn serial(&self) -> u64 {
        self.serial
    }

    /// Record which frame these pixels are.
    #[inline]
    pub const fn set_serial(&mut self, serial: u64) {
        self.serial = serial;
    }

    /// The raw bytes.
    #[inline]
    #[must_use]
    pub fn pixels(&self) -> &[u8] {
        &self.pixels
    }

    /// The raw bytes, mutably — for a producer filling the surface in bulk.
    #[inline]
    pub fn pixels_mut(&mut self) -> &mut [u8] {
        &mut self.pixels
    }

    /// The address of the first byte, for an embedder that reads the buffer out
    /// of exported memory (`ROADMAP.md` §11.5).
    ///
    /// Stable until the surface is resized or dropped.
    #[inline]
    #[must_use]
    pub fn as_ptr(&self) -> *const u8 {
        self.pixels.as_ptr()
    }

    /// Reshape to `format`/`width`/`height`, reallocating only if the byte
    /// count changes. The contents are left as they were where the size is
    /// unchanged, so a resize to the same shape is free and keeps the pointer.
    pub fn reshape(&mut self, format: PixelFormat, width: u32, height: u32) {
        let stride = u64::from(width) * format.bytes_per_pixel();
        let len = stride * u64::from(height);
        if self.format == format && self.width == width && self.height == height {
            return;
        }
        self.format = format;
        self.width = width;
        self.height = height;
        self.stride = stride;
        self.pixels.resize(len as usize, 0);
        self.pixels.shrink_to_fit();
    }

    /// Paint every pixel `rgb`.
    pub fn fill(&mut self, rgb: [u8; 3]) {
        let bpp = self.format.bytes_per_pixel() as usize;
        let offsets = self.format.channel_offsets();
        let alpha = self.format.has_alpha();
        for pixel in self.pixels.chunks_exact_mut(bpp) {
            pixel[offsets[0]] = rgb[0];
            pixel[offsets[1]] = rgb[1];
            pixel[offsets[2]] = rgb[2];
            if alpha {
                pixel[3] = 0xff;
            }
        }
    }

    /// Write one pixel, ignoring coordinates outside the surface.
    ///
    /// Out of range is a no-op rather than a panic: a device whose visible
    /// geometry disagrees with the host's surface by a scanline (PAL's blanked
    /// top line, for one) must not take the process down mid-frame.
    #[inline]
    pub fn put(&mut self, x: u32, y: u32, rgb: [u8; 3]) {
        let Some(offset) = self.offset_of(x, y) else {
            return;
        };
        let offsets = self.format.channel_offsets();
        let pixel = &mut self.pixels[offset..];
        pixel[offsets[0]] = rgb[0];
        pixel[offsets[1]] = rgb[1];
        pixel[offsets[2]] = rgb[2];
        if self.format.has_alpha() {
            pixel[3] = 0xff;
        }
    }

    /// Read one pixel back as `RGB`, or `None` outside the surface.
    #[inline]
    #[must_use]
    pub fn get(&self, x: u32, y: u32) -> Option<[u8; 3]> {
        let offset = self.offset_of(x, y)?;
        let offsets = self.format.channel_offsets();
        let pixel = &self.pixels[offset..];
        Some([pixel[offsets[0]], pixel[offsets[1]], pixel[offsets[2]]])
    }

    /// One row of bytes.
    #[must_use]
    pub fn row(&self, y: u32) -> Option<&[u8]> {
        if y >= self.height {
            return None;
        }
        let start = (u64::from(y) * self.stride) as usize;
        let end = start + self.stride as usize;
        Some(&self.pixels[start..end])
    }

    /// FNV-1a over the pixel bytes: the frame hash the regression method of
    /// `ROADMAP.md` §12 compares.
    ///
    /// The same function [`Machine::state_hash`](crate::machine::Machine::state_hash)
    /// uses, for the same reason — it is not cryptographic, it works in a
    /// dependency-free `no_std` build, and comparing two of them is the whole
    /// point.
    #[must_use]
    pub fn hash(&self) -> u64 {
        let mut h: u64 = 0xcbf2_9ce4_8422_2325;
        for b in &self.pixels {
            h ^= u64::from(*b);
            h = h.wrapping_mul(0x0000_0100_0000_01b3);
        }
        h
    }

    /// Byte offset of pixel `(x, y)`, or `None` if it is outside.
    #[inline]
    fn offset_of(&self, x: u32, y: u32) -> Option<usize> {
        if x >= self.width || y >= self.height {
            return None;
        }
        let bpp = self.format.bytes_per_pixel();
        Some((u64::from(y) * self.stride + u64::from(x) * bpp) as usize)
    }
}

// ---------------------------------------------------------------------------
// The scanout seam
// ---------------------------------------------------------------------------

/// A display device, as the host sees it.
///
/// `Send + Sync` like every device-facing trait (`ROADMAP.md` §0): the emulation
/// thread produces frames and the display thread captures them.
///
/// Implementors live beside the host, not beside the device: they hold whatever
/// handle the device offers (an `Arc<NesPpu>`, later a virtio-gpu resource) and
/// do the colour conversion the device deliberately does not.
pub trait Scanout: Send + Sync + fmt::Debug {
    /// What this device produces.
    fn info(&self) -> SurfaceInfo;

    /// Frames the device has completed since reset.
    ///
    /// Monotonic. A host that wants to draw only on change compares this with
    /// [`Surface::serial`].
    fn frame_counter(&self) -> u64;

    /// How long one frame lasts in virtual nanoseconds, or `0` if the device
    /// has no fixed rate (a VGA card between mode sets, say).
    ///
    /// Virtual, not real: it is how far a host advances the machine to get one
    /// more frame, and it is exact arithmetic from the device's own clock —
    /// never a wall-clock measurement (`CLAUDE.md`, determinism).
    fn frame_period_ns(&self) -> u64 {
        0
    }

    /// Copy the most recent frame into `dst`, reshaping it if the geometry
    /// changed, and return the frame counter that was captured.
    ///
    /// The destination's format is honoured: a host that wants `BGRA8888`
    /// gets it. Implementors go through [`Surface::put`] rather than laying
    /// bytes out themselves, so a new format is added in one place.
    fn capture(&self, dst: &mut Surface) -> u64;
}
