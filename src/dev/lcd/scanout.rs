//! A generic parallel-RGB scanout engine.
//!
//! # What it is
//!
//! The half of a display path that owns *where the pixels are*: a framebuffer
//! base address in some address space, a geometry, a pixel format, a stride,
//! and a frame period derived from a clock domain. It reads guest memory
//! directly and hands out RGB888 rows; [`crate::host::display`] turns those
//! into a [`Surface`](crate::host::display::Surface) a host can look at.
//!
//! Real hardware here is a display controller feeding a TFT panel over a
//! parallel RGB link — pixel clock, HSYNC, VSYNC, DE and up to twenty-four data
//! lines. **None of that is modelled, deliberately.** No guest can observe a
//! single one of those edges: it writes a framebuffer and programs a base
//! address, and everything after that is a geometry, a format and a frame
//! period. Simulating the link would cost a great deal and change nothing a
//! guest or a test could see. So the engine reads memory and the "link" is
//! three numbers.
//!
//! # Tearing is the honest answer
//!
//! The framebuffer is read **when a frame is captured**, from whatever base the
//! register holds at that moment. Nothing is buffered, nothing is stabilised.
//! That is what the hardware does: software that flips buffers without waiting
//! for VSYNC gets a torn frame on a real panel, and a model that quietly fixed
//! it would hide the guest's bug.
//!
//! # Why the register block here is not a real chip's
//!
//! It is rsemu's own, and it is labelled as such. The engine is deliberately
//! separable from it: a SoC's display controller is a *different* register
//! block over the same mechanism, and adding one should be a thin adapter that
//! pokes a base address, a geometry and an enable into [`Scanout`] rather than
//! a second copy of everything below. That split is the point of this module.
//!
//! # Time
//!
//! The frame period is `htotal × vtotal` ticks of the engine's clock domain —
//! the pixel clock — converted to virtual nanoseconds by exact integer
//! arithmetic from the domain's own rational frequency. Never a nominal 60 Hz,
//! and never a float (`CLAUDE.md`, determinism). The engine is a *lazily
//! advanced* device (`ROADMAP.md` §4.2) whose only per-frame work is to count
//! the frame, so a machine with nobody watching the display pays almost
//! nothing.
//!
//! **There is no VSYNC output pin and no frame interrupt**, and that is a
//! decision rather than an oversight: the firmware this was built for neither
//! waits on VSYNC nor takes a frame interrupt, so a wire nothing drives would
//! be untested surface. Both are small additions — a `WireSource` beside the
//! frame counter, raised from `advance_to` — and the frame boundary they would
//! be raised on is already computed here.

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
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
const CLASS_NAME: &str = "lcd.scanout";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes of address space the register block occupies.
pub const REGISTER_BYTES: u64 = 0x20;

/// `CTRL` bit 0: the engine is scanning out.
const CTRL_EN: u32 = 1 << 0;
/// Everything `CTRL` defines.
const CTRL_MASK: u32 = CTRL_EN;

// ---------------------------------------------------------------------------
// Pixel formats
// ---------------------------------------------------------------------------

/// How the *guest's* framebuffer stores a pixel.
///
/// Distinct from [`crate::host::display::PixelFormat`], which is how a *host*
/// surface stores one: this is what the engine has to decode, that is what the
/// host wants encoded, and conflating the two is how a display model ends up
/// with a byte order that is right on one machine and wrong on the next.
///
/// The extensible-newtype pattern (`CLAUDE.md`): a board that needs another
/// packing adds a constant and one arm in [`FbFormat::decode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
#[repr(transparent)]
pub struct FbFormat(pub u16);

impl FbFormat {
    /// Three bytes per pixel, red first. The common packing for an RGB panel,
    /// and what a 480×320 frame of 460,800 bytes is.
    pub const RGB888: FbFormat = FbFormat(0);
    /// Three bytes per pixel, blue first.
    pub const BGR888: FbFormat = FbFormat(1);
    /// Two bytes per pixel, little-endian, `RRRRRGGG GGGBBBBB`.
    pub const RGB565: FbFormat = FbFormat(2);
    /// Four bytes per pixel, little-endian `0xXXRRGGBB` — so bytes B, G, R, X.
    pub const XRGB8888: FbFormat = FbFormat(3);

    /// How many bytes one pixel occupies.
    #[must_use]
    pub const fn bytes_per_pixel(self) -> u64 {
        match self {
            FbFormat::RGB565 => 2,
            FbFormat::XRGB8888 => 4,
            // RGB888, BGR888, and anything unknown.
            _ => 3,
        }
    }

    /// The spelling a machine description writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            FbFormat::RGB888 => "rgb888",
            FbFormat::BGR888 => "bgr888",
            FbFormat::RGB565 => "rgb565",
            FbFormat::XRGB8888 => "xrgb8888",
            _ => "unknown",
        }
    }

    /// Parse that spelling.
    #[must_use]
    pub fn from_name(name: &str) -> Option<FbFormat> {
        match name {
            "rgb888" => Some(FbFormat::RGB888),
            "bgr888" => Some(FbFormat::BGR888),
            "rgb565" => Some(FbFormat::RGB565),
            "xrgb8888" => Some(FbFormat::XRGB8888),
            _ => None,
        }
    }

    /// Every spelling, for a validator's enumeration.
    pub const NAMES: &'static [&'static str] = &["rgb888", "bgr888", "rgb565", "xrgb8888"];

    /// One pixel from `bytes`, which must be at least
    /// [`bytes_per_pixel`](FbFormat::bytes_per_pixel) long.
    ///
    /// `RGB565`'s expansion replicates the high bits into the low ones, so
    /// `0x1f` becomes `0xff` rather than `0xf8` — the usual convention, and the
    /// one that keeps white white.
    #[must_use]
    pub fn decode(self, bytes: &[u8]) -> [u8; 3] {
        match self {
            FbFormat::BGR888 => [bytes[2], bytes[1], bytes[0]],
            FbFormat::RGB565 => {
                let v = u16::from_le_bytes([bytes[0], bytes[1]]);
                let r = ((v >> 11) & 0x1f) as u8;
                let g = ((v >> 5) & 0x3f) as u8;
                let b = (v & 0x1f) as u8;
                [
                    (r << 3) | (r >> 2),
                    (g << 2) | (g >> 4),
                    (b << 3) | (b >> 2),
                ]
            }
            FbFormat::XRGB8888 => [bytes[2], bytes[1], bytes[0]],
            // RGB888 and anything unknown: a wrong picture rather than a panic,
            // for the same reason `PixelFormat` does it.
            _ => [bytes[0], bytes[1], bytes[2]],
        }
    }
}

impl fmt::Display for FbFormat {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// A generic parallel-RGB scanout engine.
#[derive(Debug)]
pub struct Scanout {
    shared: Arc<Shared>,
    region: RegionRef,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    /// One frame in pixel-clock ticks: `htotal × vtotal`.
    frame_ticks: u64,
    /// One frame in virtual nanoseconds, resolved at bind time from the clock
    /// domain's exact rational frequency. Zero until then.
    frame_nanos: AtomicU64,
    /// Pixel-clock ticks simulated, published for the scheduler's lock-free
    /// question.
    ticks: AtomicU64,
    /// The tick the next frame boundary falls on.
    next_event: AtomicU64,
    /// Frames completed, published so a capture never has to take the lock.
    frames: AtomicU64,
    /// The address space the framebuffer lives in, and who we are on it.
    /// **Derived from the machine graph, never serialized** (invariant 3).
    bus: Mutex<Option<Arc<AddressSpace>>>,
    requester: Mutex<RequesterId>,
    /// The catch-up handle the register block syncs through.
    lazy: Mutex<Option<LazyHandle>>,
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// Pixel-clock ticks simulated.
    ticks: u64,
    /// Frames completed since reset.
    frames: u64,
    ctrl: u32,
    /// Where the framebuffer starts. A guest-physical address, so `u64`.
    base: u64,
    /// Bytes per row, or `0` for `width × bytes_per_pixel`.
    stride: u64,
    width: u32,
    height: u32,
    format: FbFormat,
    /// What the machine file configured, to return to on reset.
    reset_base: u64,
    reset_stride: u64,
    reset_width: u32,
    reset_height: u32,
    reset_format: FbFormat,
}

impl State {
    /// Bytes per row, resolving `0`.
    fn stride(&self) -> u64 {
        if self.stride != 0 {
            self.stride
        } else {
            u64::from(self.width) * self.format.bytes_per_pixel()
        }
    }
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Shared");
        s.field("frame_ticks", &self.frame_ticks);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Scanout {
    /// Validate `props` and build the engine.
    ///
    /// Properties:
    ///
    /// * `width`, `height` — the visible geometry. Required: a display
    ///   controller with no geometry has nothing to scan.
    /// * `base` — where the framebuffer starts. Default 0; a guest normally
    ///   programs it.
    /// * `stride` — bytes per row. Default 0, meaning `width × bpp`.
    /// * `format` — how the guest packs a pixel. Default `rgb888`.
    /// * `htotal`, `vtotal` — the total periods including blanking, in pixel
    ///   clocks and lines. Their product is the frame, which is what the frame
    ///   rate is computed from. Default to the visible geometry, which is a
    ///   panel with no blanking at all.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property or a missing required one,
    /// [`Error::Config`] for a zero dimension, an unknown `format`, or totals
    /// smaller than the visible area.
    pub fn new(props: &Props) -> Result<Scanout> {
        let mut r = props.reader();
        let width: u64 = r.require("width")?;
        let height: u64 = r.require("height")?;
        let base: u64 = r.or("base", 0)?;
        let stride: u64 = r.or("stride", 0)?;
        let format_name = r.optional_str("format")?.unwrap_or("rgb888");
        let htotal: u64 = r.or("htotal", width)?;
        let vtotal: u64 = r.or("vtotal", height)?;
        r.finish()?;

        let bad = |message: String| Error::Config {
            at: String::from(CLASS_NAME),
            message,
        };
        if width == 0 || height == 0 {
            return Err(bad(alloc::format!(
                "the scanout is {width}x{height}; both dimensions must be at least 1"
            )));
        }
        if width > u64::from(u32::MAX) || height > u64::from(u32::MAX) {
            return Err(bad(String::from(
                "a scanout dimension is a pixel count, not an address",
            )));
        }
        let format = FbFormat::from_name(format_name).ok_or_else(|| {
            bad(alloc::format!(
                "`format` is `{format_name}`; it must be one of {:?}",
                FbFormat::NAMES
            ))
        })?;
        if htotal < width || vtotal < height {
            return Err(bad(alloc::format!(
                "the total period {htotal}x{vtotal} is smaller than the visible {width}x{height}; \
                 the totals include the blanking, so they are never the smaller pair"
            )));
        }
        let frame_ticks = htotal.saturating_mul(vtotal);

        let state = State {
            ticks: 0,
            frames: 0,
            ctrl: 0,
            base,
            stride,
            width: width as u32,
            height: height as u32,
            format,
            reset_base: base,
            reset_stride: stride,
            reset_width: width as u32,
            reset_height: height as u32,
            reset_format: format,
        };
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, state),
            frame_ticks,
            frame_nanos: AtomicU64::new(0),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(frame_ticks),
            frames: AtomicU64::new(0),
            bus: Mutex::with_rank(LockRank::WIRE, None),
            requester: Mutex::with_rank(LockRank::WIRE, RequesterId::ANONYMOUS),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
        });
        let port = Arc::new(ScanoutPort {
            shared: Arc::clone(&shared),
        });
        let region = Arc::new(Region::io("lcdc", REGISTER_BYTES, port as Arc<dyn MemOps>));
        Ok(Scanout { shared, region })
    }

    /// The visible geometry the registers currently hold.
    #[must_use]
    pub fn geometry(&self) -> (u32, u32) {
        let state = self.shared.state.lock();
        (state.width, state.height)
    }

    /// How the guest packs a pixel.
    #[must_use]
    pub fn format(&self) -> FbFormat {
        self.shared.state.lock().format
    }

    /// Where the framebuffer starts, right now.
    #[must_use]
    pub fn base(&self) -> u64 {
        self.shared.state.lock().base
    }

    /// Bytes per row, with `0` resolved.
    #[must_use]
    pub fn stride(&self) -> u64 {
        self.shared.state.lock().stride()
    }

    /// Whether the engine is scanning out. A disabled controller shows black.
    #[must_use]
    pub fn enabled(&self) -> bool {
        self.shared.state.lock().ctrl & CTRL_EN != 0
    }

    /// Frames completed since reset. Lock-free: a host asks every redraw.
    #[must_use]
    pub fn frame(&self) -> u64 {
        self.shared.frames.load(Ordering::Relaxed)
    }

    /// One frame in pixel-clock ticks (`htotal × vtotal`).
    #[must_use]
    pub fn frame_ticks(&self) -> u64 {
        self.shared.frame_ticks
    }

    /// One frame in virtual nanoseconds, or `0` before the machine has bound a
    /// clock domain to this device.
    ///
    /// Exact integer arithmetic from the domain's own rational frequency —
    /// never a wall-clock measurement and never a nominal rate
    /// (`CLAUDE.md`, determinism).
    #[must_use]
    pub fn frame_period_nanos(&self) -> u64 {
        self.shared.frame_nanos.load(Ordering::Relaxed)
    }

    /// Read row `y` of the framebuffer as RGB888 triples.
    ///
    /// `dst` is filled to `min(dst.len(), width)`. Returns `false` — leaving
    /// `dst` black — when the engine is disabled, `y` is past the bottom, or
    /// the address space refuses the read, which is what a base pointing into a
    /// hole looks like.
    ///
    /// **Read at the moment it is called**, from whatever base the register
    /// holds. See the module docs: tearing is the hardware's behaviour and
    /// hiding it would hide the guest's bug.
    pub fn read_row(&self, y: u32, dst: &mut [[u8; 3]]) -> bool {
        for pixel in dst.iter_mut() {
            *pixel = [0, 0, 0];
        }
        let (base, stride, width, height, format, enabled) = {
            let state = self.shared.state.lock();
            (
                state.base,
                state.stride(),
                state.width,
                state.height,
                state.format,
                state.ctrl & CTRL_EN != 0,
            )
        };
        if !enabled || y >= height {
            return false;
        }
        let bus = self.shared.bus.lock().clone();
        let Some(bus) = bus else {
            return false;
        };
        let requester = *self.shared.requester.lock();
        let bpp = format.bytes_per_pixel();
        let count = (dst.len() as u64).min(u64::from(width));
        let row_addr = base.wrapping_add(u64::from(y).wrapping_mul(stride));

        // One read for the whole row rather than one per pixel: this is the
        // only hot loop in the device, and a per-pixel dispatch through the
        // address space would dominate it.
        let mut row = vec![0u8; (count * bpp) as usize];
        if bus
            .read_bytes(row_addr, &mut row, self.attrs(requester))
            .is_err()
        {
            return false;
        }
        for (i, pixel) in dst.iter_mut().take(count as usize).enumerate() {
            let at = i * bpp as usize;
            *pixel = format.decode(&row[at..]);
        }
        true
    }

    /// The attributes a scanout read carries.
    ///
    /// `debug` is set: a display controller reading a framebuffer must not pop
    /// a FIFO or advance a pointer in whatever it happens to be pointed at, and
    /// a host redrawing a window is not the guest making an access
    /// (`ROADMAP.md` §15, invariant 5). It is also what stops a capture from
    /// perturbing a machine a debugger has stopped.
    fn attrs(&self, requester: RequesterId) -> MemAttrs {
        MemAttrs {
            requester,
            debug: true,
            ..MemAttrs::DEFAULT
        }
    }

    /// Run the engine until `target` pixel-clock ticks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

impl Shared {
    /// Publish what the scheduler and a capture may ask for without a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.frames.store(state.frames, Ordering::Relaxed);
        let next = state
            .ticks
            .saturating_sub(state.ticks % self.frame_ticks)
            .saturating_add(self.frame_ticks);
        self.next_event
            .store(next.max(state.ticks.saturating_add(1)), Ordering::Relaxed);
    }

    /// Count whole frames as time passes.
    ///
    /// Nothing else happens here — no memory is read, no wire is driven — which
    /// is why a machine nobody is watching pays almost nothing for having a
    /// display.
    fn advance_to(&self, target: u64) {
        let mut state = self.state.lock();
        if target <= state.ticks {
            return;
        }
        let before = state.ticks / self.frame_ticks;
        let after = target / self.frame_ticks;
        state.ticks = target;
        if after > before && state.ctrl & CTRL_EN != 0 {
            state.frames += after - before;
        }
        self.publish(&state);
    }

    /// Bring the engine up to date before a register access.
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

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The memory-mapped registers.
///
/// | Offset | Name | Access | Meaning |
/// | --- | --- | --- | --- |
/// | `0x00` | `CTRL` | R/W | bit 0 `EN` |
/// | `0x04` | `BASE` | R/W | framebuffer base, low 32 bits |
/// | `0x08` | `BASE_HI` | R/W | framebuffer base, high 32 bits |
/// | `0x0c` | `STRIDE` | R/W | bytes per row, `0` for `WIDTH × bpp` |
/// | `0x10` | `WIDTH` | R/W | visible pixels across |
/// | `0x14` | `HEIGHT` | R/W | visible pixels down |
/// | `0x18` | `FORMAT` | R/W | an [`FbFormat`] code |
/// | `0x1c` | `FRAMES` | R | frames completed since reset |
struct ScanoutPort {
    shared: Arc<Shared>,
}

impl fmt::Debug for ScanoutPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ScanoutPort").finish_non_exhaustive()
    }
}

impl MemOps for ScanoutPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        // `FRAMES` is the only register whose value moves with time, and a
        // debug read of it must report where the engine stands rather than
        // advancing it — which is what `sync` does with `AccessKind::Debug`.
        self.shared.sync(attrs);
        let state = self.shared.state.lock();
        let value = match offset {
            0x00 => state.ctrl,
            0x04 => state.base as u32,
            0x08 => (state.base >> 32) as u32,
            0x0c => state.stride as u32,
            0x10 => state.width,
            0x14 => state.height,
            0x18 => u32::from(state.format.0),
            0x1c => state.frames as u32,
            _ => 0,
        };
        dst.copy_from_slice(&value.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write would move the framebuffer under the guest.
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        let value = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        let mut state = self.shared.state.lock();
        match offset {
            0x00 => state.ctrl = value & CTRL_MASK,
            0x04 => state.base = (state.base & 0xffff_ffff_0000_0000) | u64::from(value),
            0x08 => state.base = (state.base & 0x0000_0000_ffff_ffff) | (u64::from(value) << 32),
            0x0c => state.stride = u64::from(value),
            // A geometry of zero would make every capture empty; the register
            // takes the write but the floor is one, which is the same
            // "clamp, do not fault" rule the SPI word width follows.
            0x10 => state.width = value.max(1),
            0x14 => state.height = value.max(1),
            0x18 => state.format = FbFormat(value as u16),
            // `FRAMES` is read-only.
            _ => {}
        }
        self.shared.publish(&state);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for Scanout {
    fn class(&self) -> &'static DeviceClass {
        &SCANOUT_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the register block, and the
        // address space arrives later, in `Instance::bind` — which the realizer
        // runs *after* every region is mapped, so a bus master may reach
        // through its space from there. That is also where a missing `space` is
        // reported.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        let mut state = self.shared.state.lock();
        state.ctrl = 0;
        state.frames = 0;
        state.base = state.reset_base;
        state.stride = state.reset_stride;
        state.width = state.reset_width;
        state.height = state.reset_height;
        state.format = state.reset_format;
        self.shared.publish(&state);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u64(state.frames)?;
        w.write_u32(state.ctrl)?;
        w.write_u64(state.base)?;
        w.write_u64(state.stride)?;
        w.write_u32(state.width)?;
        w.write_u32(state.height)?;
        w.write_u16(state.format.0)
        // The framebuffer itself is **not** saved: it is ordinary guest memory
        // and belongs to whatever RAM device owns it, which snapshots it once
        // (`ROADMAP.md` §4.5). Nor is the address space — that is derived from
        // the machine graph and is rebuilt by realize (invariant 3).
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = self.shared.state.lock();
        state.ticks = r.read_u64()?;
        state.frames = r.read_u64()?;
        state.ctrl = r.read_u32()?;
        state.base = r.read_u64()?;
        state.stride = r.read_u64()?;
        state.width = r.read_u32()?;
        state.height = r.read_u32()?;
        state.format = FbFormat(r.read_u16()?);
        self.shared.publish(&state);
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. `FRAMES` has to report the frame the guest is actually in, and the
    /// frame counter a host polls has to move with virtual time rather than
    /// with the quantum boundary.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Scanout::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        Some(self.shared.next_event.load(Ordering::Relaxed))
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for Scanout {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "a scanout engine is a bus master and needs the address space its framebuffer \
                 lives in (`space = mem`)",
            ),
        })?;
        *self.shared.bus.lock() = Some(Arc::clone(space));
        *self.shared.requester.lock() = ctx.requester();
        Ok(())
    }
}

/// Resolve the frame period from a clock domain's exact rational frequency.
///
/// A device cannot reach the clock forest from `&self` — `BindCtx` carries the
/// `DomainId` but not the forest — so a host or a board hands the numbers in
/// once, after the machine is realized. `hz_num / hz_den` is the domain's
/// frequency in hertz, exactly; the period lands in virtual nanoseconds with no
/// float anywhere on the path.
///
/// # Errors
///
/// Nothing: an impossible rate leaves the period at zero, which
/// [`Scanout::frame_period_nanos`] already documents as "not known yet" and
/// which [`crate::host::display::Scanout::frame_period_ns`] documents as "no
/// fixed rate".
pub fn set_frame_rate(engine: &Scanout, hz_num: u64, hz_den: u64) {
    if hz_num == 0 {
        engine.shared.frame_nanos.store(0, Ordering::Relaxed);
        return;
    }
    let nanos = engine
        .shared
        .frame_ticks
        .saturating_mul(hz_den)
        .saturating_mul(1_000_000_000)
        / hz_num;
    engine.shared.frame_nanos.store(nanos, Ordering::Relaxed);
}

/// The `lcd.scanout` device class.
pub static SCANOUT_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a generic RGB scanout engine: reads a framebuffer out of an address space",
    properties: &[
        PropertySpec {
            name: "width",
            kind: ValueKind::Uint,
            required: true,
            summary: "visible pixels across",
        },
        PropertySpec {
            name: "height",
            kind: ValueKind::Uint,
            required: true,
            summary: "visible pixels down",
        },
        PropertySpec {
            name: "base",
            kind: ValueKind::Uint,
            required: false,
            summary: "where the framebuffer starts, if the guest does not program it",
        },
        PropertySpec {
            name: "stride",
            kind: ValueKind::Uint,
            required: false,
            summary: "bytes per row (default 0, meaning width x bytes-per-pixel)",
        },
        PropertySpec {
            name: "format",
            kind: ValueKind::Str,
            required: false,
            summary: "how the guest packs a pixel: rgb888, bgr888, rgb565, xrgb8888",
        },
        PropertySpec {
            name: "htotal",
            kind: ValueKind::Uint,
            required: false,
            summary: "one horizontal period in pixel clocks, blanking included",
        },
        PropertySpec {
            name: "vtotal",
            kind: ValueKind::Uint,
            required: false,
            summary: "one vertical period in lines, blanking included",
        },
    ],
    construct: |props| Ok(Box::new(Scanout::new(props)?)),
};

/// Add [`SCANOUT_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&SCANOUT_CLASS)
}

/// Bind [`SCANOUT_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Scanout::new(props)?)))
}

/// What the validator should know about `lcd.scanout`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(
            PropSchema::new("width", ValueKind::Uint)
                .required()
                .range(1, u64::from(u32::MAX)),
        )
        .prop(
            PropSchema::new("height", ValueKind::Uint)
                .required()
                .range(1, u64::from(u32::MAX)),
        )
        .prop(PropSchema::new("base", ValueKind::Uint))
        .prop(PropSchema::new("stride", ValueKind::Uint))
        .prop(PropSchema::new("format", ValueKind::Str).values(FbFormat::NAMES))
        .prop(PropSchema::new("htotal", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .prop(PropSchema::new("vtotal", ValueKind::Uint).range(1, u64::from(u32::MAX)))
        .region("")
        .region("regs")
}

/// Read a whole frame as RGB888 rows.
///
/// A convenience over [`Scanout::read_row`] for a caller that wants the lot —
/// the host adapter, and every test that asserts what is on the screen. The
/// result is `height` rows of `width` triples.
#[must_use]
pub fn read_frame(engine: &Scanout) -> Vec<Vec<[u8; 3]>> {
    let (width, height) = engine.geometry();
    let mut rows = Vec::with_capacity(height as usize);
    for y in 0..height {
        let mut row = vec![[0u8; 3]; width as usize];
        engine.read_row(y, &mut row);
        rows.push(row);
    }
    rows
}

#[cfg(test)]
mod tests;
