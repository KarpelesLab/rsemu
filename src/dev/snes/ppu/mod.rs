//! The SNES / Super Famicom picture processing unit (`dev-snes-ppu`): the
//! S-PPU, which is two chips — PPU1 (5C77) and PPU2 (5C78) — modelled as one
//! device because nothing outside them can tell where the seam is, except the
//! two version numbers and the two open-bus latches, which are kept apart.
//!
//! ```text
//!   64 KiB VRAM    tiles, tilemaps, mode-7 map and pixels
//!   512 B  CGRAM   256 colours of 0bbbbbgggggrrrrr
//!   544 B  OAM     128 sprites: a 512-byte low table, a 32-byte high table
//!   $2100-$213F    the registers, on the S-CPU's B-bus
//! ```
//!
//! # Shape
//!
//! | Item | Role |
//! | --- | --- |
//! | [`SnesPpu`] | the [`Device`]: lifecycle, snapshots, the pins, the lazy surface |
//! | [`SnesPpuBeam`] | the handle published as [`ExportId::SNES_PPU_BEAM`] for the S-CPU |
//! | [`Engine`] | memories, registers and beam; `regs.rs` and `render.rs` extend it |
//! | [`Region`] and the `timing` functions | the dot, line and frame geometry |
//!
//! # Wiring it
//!
//! * **Region** [`REGISTER_REGION`] (`regs`, also the empty name): 64 bytes,
//!   mapped by the board at `$2100` of the S-CPU's B-bus. Byte accesses only.
//!   A read of a register that drives nothing answers [`MemAttrs::bus`] — the
//!   S-CPU's own open-bus byte — and the eighteen write-only registers fullsnes
//!   lists answer PPU1's latch instead.
//! * **Output pins** [`VBLANK_PIN`] and [`HBLANK_PIN`]: levels, high during the
//!   blank. The S-CPU owns `RDNMI`, `HVBJOY` and the NMI itself; a rising
//!   `vblank` is the frame's NMI edge.
//! * **Input pin** [`EXTLATCH_PIN`]: `WRIO` bit 7 and controller port 2 pin 6.
//!   A falling edge latches the H/V counters; `$2137` latches only while it is
//!   high.
//! * **Export** [`ExportId::SNES_PPU_BEAM`]: an `Arc<`[`SnesPpuBeam`]`>`.
//! * **Clock**: the device's tick is **one master clock** — a dot is four or
//!   six of them, so no coarser unit is exact. [`add_clock_domain`] adds the
//!   ÷1 domain.
//! * **Properties**: `region` (`ntsc`, default, or `pal`), `ppu1-version`
//!   (default 1, the only 5C77 revision fullsnes knows) and `ppu2-version`
//!   (default 3, the last 5C78 revision and the one most consoles carry).
//!
//! # Time and rendering granularity
//!
//! **Lazily advanced** (`ROADMAP.md` §4.2) and caught up before every access,
//! like the NES PPU. Each line has four boundaries — dot 0, the H-blank fall at
//! dot 1, the picture's start at dot 22, the H-blank rise at dot 274 — and the
//! chip walks from one to the next rather than dot by dot.
//!
//! **A line is rendered whole, at dot 22**, from the registers as they stand —
//! the dot fullsnes and the wiki give for the first visible pixel. That is the
//! latch point that makes per-line effects exact: HDMA writes at H ≈ 278 of the
//! line before, and an H-IRQ handler that writes in H-blank, both land before
//! the line they target is drawn. What it gives up is a register change in the
//! middle of the visible part of a line (dots 22-277), which takes effect on the
//! next line rather than at the pixel it happened on. A dot renderer would buy
//! that at a large cost in a model that is otherwise exact at line granularity;
//! no commercial effect we know of needs it beyond a handful of raster demos.
//!
//! # What is not modelled
//!
//! * Mid-line register changes (above), and with them the CGRAM-write-during-
//!   rendering address corruption and OAM writes during rendering landing on
//!   the evaluation address. CGRAM and OAM writes during the active display
//!   simply land; VRAM writes during it are dropped, as on hardware.
//! * Garbage from `MPYx` while mode 7 is drawing, and from VRAM reads during
//!   the active display.
//! * Hires colour math on the even (sub-screen) half-dots: they show the sub
//!   screen as composed, without math.
//! * OBJ vertical flip of the rectangular sizes 6 and 7, which hardware does
//!   oddly; it is flipped over the full height here.
//! * Interlace in modes 5/6 shifts the BG line to `2V + field`; whether `VOFS`
//!   is also counted in interlaced lines is unverified.
//!
//! # Sources
//!
//! nocash **fullsnes** (the PPU, timing and status chapters) and the
//! **SNESdev wiki** (*PPU registers*, *Backgrounds*, *Tiles*, *Tilemaps*,
//! *Sprites*, *Offset-per-tile*, *Mode 7*, *Windows*, *Color math*, *Timing*).
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).
//! `docs/devices/snes-ppu.md` has the long form.

mod engine;
mod regs;
mod render;
pub mod timing;

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::clock::{ClockForest, ClockResult, DomainId};
use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Level, WireId, WireSink, WireSource};

pub use engine::{CGRAM_ENTRIES, Engine, FB_HEIGHT, FB_LEN, FB_WIDTH, OAM_LEN, VRAM_WORDS};
pub use regs::{
    MPYL, OPHCT, OPVCT, PPU1_OPEN_BUS, RDCGRAM, RDOAM, RDVRAMH, RDVRAML, Regs, SLHV, STAT77, STAT78,
};
pub use timing::Region;

/// Where the board maps the register block on the B-bus.
pub const REGISTER_BASE: u64 = 0x2100;
/// The register block's length: `$2100`-`$213F`.
pub const REGISTER_WINDOW_LEN: u64 = 0x40;
/// The name a `map` statement reaches the register block by.
pub const REGISTER_REGION: &str = "regs";
/// The vertical-blank output level.
pub const VBLANK_PIN: &str = "vblank";
/// The horizontal-blank output level.
pub const HBLANK_PIN: &str = "hblank";
/// The `EXTLATCH` input.
pub const EXTLATCH_PIN: &str = "extlatch";

/// The default `ppu1-version`: `STAT77` bits 0-3.
pub const DEFAULT_PPU1_VERSION: u8 = 1;
/// The default `ppu2-version`: `STAT78` bits 0-3.
pub const DEFAULT_PPU2_VERSION: u8 = 3;

/// The engine lock's rank: above [`LockRank::DEVICE`], below
/// [`LockRank::WIRE`].
///
/// Above `DEVICE` so the S-CPU's register block may ask the beam handle a
/// question while holding its own device lock — `HVBJOY` is exactly that.
/// The engine reaches nothing outward while held (the PPU's memories are its
/// own, and the wires are driven after release), so nothing has to rank above
/// it but the wire lock it never takes.
pub const ENGINE_LOCK_RANK: LockRank = LockRank::new(0x5c00);

/// Add the PPU's clock domain under `master`: master ÷ 1, because a dot is
/// four or six master clocks and only the master clock divides both.
///
/// # Errors
///
/// Whatever [`ClockForest::add_domain`] reports.
pub fn add_clock_domain(forest: &mut ClockForest, master: DomainId) -> ClockResult<DomainId> {
    forest.add_domain("ppu", master, 1, 1)
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

#[derive(Debug, Default)]
struct Links {
    vblank: Vec<WireSource>,
    hblank: Vec<WireSource>,
    /// What was last driven, so a catch-up per CPU cycle does not fan out a
    /// level nothing changed.
    driven: Option<(bool, bool)>,
}

struct Shared {
    engine: Mutex<Engine>,
    links: Mutex<Links>,
    lazy: Mutex<Option<LazyHandle>>,
    /// [`Engine::master`], republished for the lock-free lazy surface.
    master: AtomicU64,
    /// [`Engine::next_event`], likewise.
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
    fn publish(&self, e: &Engine) {
        self.master.store(e.master, Ordering::Relaxed);
        self.next_event.store(e.next_event(), Ordering::Relaxed);
    }

    /// Drive both levels, with no engine lock held. `force` re-announces
    /// levels that have not changed.
    fn drive(&self, levels: (bool, bool), force: bool) {
        let (vblank, hblank) = {
            let mut links = self.links.lock();
            if !force && links.driven == Some(levels) {
                return;
            }
            links.driven = Some(levels);
            (links.vblank.clone(), links.hblank.clone())
        };
        for s in &vblank {
            s.set(Level::from_bool(levels.0));
        }
        for s in &hblank {
            s.set(Level::from_bool(levels.1));
        }
    }

    /// Run `f` against the engine, then settle the pins outside the lock —
    /// the re-entrancy contract (`ROADMAP.md` §4.4).
    fn with_engine<R>(&self, f: impl FnOnce(&mut Engine) -> R) -> R {
        let (result, levels) = {
            let mut e = self.engine.lock();
            let result = f(&mut e);
            self.publish(&e);
            (result, (e.vblank, e.hblank))
        };
        self.drive(levels, false);
        result
    }

    fn advance_to(&self, target: u64) {
        loop {
            let (reached, levels) = {
                let mut e = self.engine.lock();
                let reached = e.run_to(target);
                self.publish(&e);
                (reached, (e.vblank, e.hblank))
            };
            // Every edge is driven on its own: a long budget holds several.
            self.drive(levels, false);
            if reached {
                return;
            }
        }
    }

    /// Catch the chip up to the present before an access (§4.2). A debug
    /// access moves no clock.
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
        // A refusal means catch-up is already running further up the stack;
        // answering from where the chip stands is the only defined thing.
        let _ = handle.sync(kind);
    }
}

// ---------------------------------------------------------------------------
// The beam handle
// ---------------------------------------------------------------------------

/// Where the beam is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BeamPosition {
    /// The H counter, `0..=339` (340 on the PAL long line).
    pub h: u16,
    /// The V counter, `0..lines`.
    pub v: u16,
    /// The interlace field, `STAT78` bit 7.
    pub field: bool,
    /// Master clocks into the current line, `0..1364` (1360 or 1368 on the
    /// short and long lines).
    pub master_in_line: u32,
}

/// The S-PPU as the S-CPU sees it, published as
/// [`ExportId::SNES_PPU_BEAM`].
///
/// Every method that answers a question about *now* catches the chip up first
/// (through the same lazy handle a register access uses), so the answer is at
/// the S-CPU's own position. Methods named `peek_*` do not. None of them may
/// be called with a lock ranked at or above [`ENGINE_LOCK_RANK`] held.
pub struct SnesPpuBeam {
    shared: Arc<Shared>,
}

impl fmt::Debug for SnesPpuBeam {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SnesPpuBeam").finish_non_exhaustive()
    }
}

impl SnesPpuBeam {
    fn now(&self) {
        self.shared.sync(MemAttrs::DEFAULT);
    }

    /// Catch the PPU up to the present.
    pub fn sync(&self) {
        self.now();
    }

    /// The beam position now.
    pub fn position(&self) -> BeamPosition {
        self.now();
        self.peek_position()
    }

    /// The beam position wherever the chip was last caught up to.
    pub fn peek_position(&self) -> BeamPosition {
        let e = self.shared.engine.lock();
        BeamPosition {
            h: e.h_dot(),
            v: e.v,
            field: e.field,
            master_in_line: e.line_pos,
        }
    }

    /// Latch the H/V counters now and raise `STAT78` bit 6, unconditionally —
    /// the S-CPU calls this for whatever its own documentation says latches
    /// (a light gun, or a `WRIO` write it decodes itself).
    pub fn latch_counters(&self) {
        self.now();
        self.shared.with_engine(Engine::latch_counters);
    }

    /// Drive `EXTLATCH` (`WRIO` bit 7). A 1→0 transition latches the counters,
    /// and `$2137` latches only while it is 1. Equivalent to the
    /// [`EXTLATCH_PIN`] input; use one or the other.
    pub fn set_extlatch(&self, level: bool) {
        self.now();
        self.shared.with_engine(|e| e.set_extlatch(level));
    }

    /// The television standard.
    pub fn region(&self) -> Region {
        self.shared.engine.lock().region()
    }

    /// Frames completed, counted at each start of vertical blank.
    pub fn frame(&self) -> u64 {
        self.now();
        self.shared.engine.lock().frame
    }

    /// Master clocks the PPU has run, at its own position (no catch-up).
    pub fn peek_master(&self) -> u64 {
        self.shared.master.load(Ordering::Relaxed)
    }

    /// The vertical-blank level now.
    pub fn vblank(&self) -> bool {
        self.now();
        self.shared.engine.lock().vblank
    }

    /// The horizontal-blank level now.
    pub fn hblank(&self) -> bool {
        self.now();
        self.shared.engine.lock().hblank
    }

    /// This frame's first vertical-blank line: 225, or 240 once overscan has
    /// been seen at line 225. (Before line 225 it says 225 even if overscan
    /// is on; the decision is made there.)
    pub fn vblank_line(&self) -> u16 {
        self.now();
        self.shared.engine.lock().vblank_line
    }

    /// Lines in the current frame.
    pub fn lines_in_frame(&self) -> u16 {
        self.now();
        self.shared.engine.lock().lines_in_frame
    }

    /// Whether the current frame is interlaced.
    pub fn interlace(&self) -> bool {
        self.now();
        self.shared.engine.lock().interlace_frame
    }

    /// Master clocks from now until the beam next reaches the start of dot `h`
    /// of line `v` — what an H/V timer IRQ is scheduled by. `None` if that
    /// position does not occur in the next two frames under the current
    /// `SETINI`. Zero if the beam is exactly there.
    pub fn master_until(&self, v: u16, h: u16) -> Option<u64> {
        self.now();
        self.shared.engine.lock().master_until(v, h)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The S-PPU.
pub struct SnesPpu {
    shared: Arc<Shared>,
    regs: RegionRef,
    beam: Arc<SnesPpuBeam>,
}

impl fmt::Debug for SnesPpu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SnesPpu")
            .field("engine", &self.shared.engine)
            .finish_non_exhaustive()
    }
}

impl Default for SnesPpu {
    fn default() -> Self {
        SnesPpu::new(Region::Ntsc, DEFAULT_PPU1_VERSION, DEFAULT_PPU2_VERSION)
    }
}

impl SnesPpu {
    /// A chip pair at power-on.
    #[must_use]
    pub fn new(region: Region, ppu1_version: u8, ppu2_version: u8) -> SnesPpu {
        let engine = Engine::new(region, ppu1_version & 0x0F, ppu2_version & 0x0F);
        let shared = Arc::new(Shared {
            master: AtomicU64::new(engine.master),
            next_event: AtomicU64::new(engine.next_event()),
            engine: Mutex::with_rank(ENGINE_LOCK_RANK, engine),
            links: Mutex::with_rank(LockRank::WIRE, Links::default()),
            lazy: Mutex::new(None),
        });
        let regs = Arc::new(MmioRegion::io(
            "snes.ppu.regs",
            REGISTER_WINDOW_LEN,
            Arc::new(PpuPort {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        let beam = Arc::new(SnesPpuBeam {
            shared: Arc::clone(&shared),
        });
        SnesPpu { shared, regs, beam }
    }

    /// Build one from machine-description properties.
    ///
    /// # Errors
    ///
    /// An unknown property, a region that is not `ntsc` or `pal`, or a
    /// version above 15.
    pub fn from_props(props: &Props) -> Result<SnesPpu> {
        let mut r = props.reader();
        let name = r.or_enum("region", Region::Ntsc.name(), Region::NAMES)?;
        let v1: u8 = r.or("ppu1-version", DEFAULT_PPU1_VERSION)?;
        let v2: u8 = r.or("ppu2-version", DEFAULT_PPU2_VERSION)?;
        r.finish()?;
        let region = Region::from_name(name)
            .ok_or_else(|| Error::Property(alloc::format!("unknown `region` `{name}`")))?;
        for (prop, v) in [("ppu1-version", v1), ("ppu2-version", v2)] {
            if v > 15 {
                return Err(Error::Property(alloc::format!(
                    "`{prop}` is four bits of a status register; {v} does not fit"
                )));
            }
        }
        Ok(SnesPpu::new(region, v1, v2))
    }

    /// The television standard. Not `region`, which is [`Device::region`].
    pub fn tv_region(&self) -> Region {
        self.shared.engine.lock().region()
    }

    /// The handle published as [`ExportId::SNES_PPU_BEAM`].
    pub fn beam(&self) -> Arc<SnesPpuBeam> {
        Arc::clone(&self.beam)
    }

    /// Connect the catch-up handle.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// Connect a source for the vertical-blank level. A pure store: the
    /// level is announced at realize, not here.
    pub fn attach_vblank(&self, source: WireSource) {
        let mut links = self.shared.links.lock();
        links.vblank.push(source);
        links.driven = None;
    }

    /// Connect a source for the horizontal-blank level. A pure store.
    pub fn attach_hblank(&self, source: WireSource) {
        let mut links = self.shared.links.lock();
        links.hblank.push(source);
        links.driven = None;
    }

    fn announce_levels(&self) {
        let levels = {
            let e = self.shared.engine.lock();
            (e.vblank, e.hblank)
        };
        self.shared.drive(levels, true);
    }

    /// Master clocks run since construction.
    pub fn master(&self) -> u64 {
        self.shared.master.load(Ordering::Relaxed)
    }

    /// Frames completed.
    pub fn frame(&self) -> u64 {
        self.shared.engine.lock().frame
    }

    /// Where the beam stands (no catch-up).
    pub fn position(&self) -> BeamPosition {
        self.beam.peek_position()
    }

    /// Run to absolute master tick `target`. Backwards is a no-op.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }

    /// Run `clocks` more master clocks.
    pub fn advance_by(&self, clocks: u64) {
        let target = self.master() + clocks;
        self.advance_to(target);
    }

    /// Read register `$21xx` as the guest would (no catch-up). `bus` is the
    /// S-CPU's open-bus byte.
    pub fn read_register(&self, offset: u8, bus: u8) -> u8 {
        self.shared
            .with_engine(|e| e.read_register(offset, bus, false))
    }

    /// Read register `$21xx` with no side effect, as a debugger does.
    pub fn peek_register(&self, offset: u8, bus: u8) -> u8 {
        self.shared.engine.lock().read_register(offset, bus, true)
    }

    /// Write register `$21xx` as the guest would (no catch-up).
    pub fn write_register(&self, offset: u8, value: u8) {
        self.shared.with_engine(|e| e.write_register(offset, value));
    }

    /// The output size of the last completed frame: 256 or 512 wide, 224,
    /// 239, 448 or 478 tall.
    pub fn output_size(&self) -> (u16, u16) {
        let e = self.shared.engine.lock();
        (e.out_width, e.out_height)
    }

    /// One pixel of the output picture, in [`output_size`](Self::output_size)
    /// coordinates: RGB555 with brightness applied.
    pub fn pixel(&self, x: usize, y: usize) -> Option<u16> {
        let e = self.shared.engine.lock();
        let (w, h) = (usize::from(e.out_width), usize::from(e.out_height));
        (x < w && y < h).then(|| e.fb[fb_index(x, y, w, h)])
    }

    /// Look at the raw [`FB_WIDTH`] × [`FB_HEIGHT`] framebuffer. A
    /// low-resolution line is stored doubled across each pair of columns, a
    /// non-interlaced one on both rows of its pair; [`fb_index`] maps output
    /// coordinates into it.
    pub fn with_framebuffer<R>(&self, f: impl FnOnce(&[u16]) -> R) -> R {
        f(&self.shared.engine.lock().fb)
    }

    /// Run `f` against the engine directly — tests and monitors.
    pub fn with_engine<R>(&self, f: impl FnOnce(&mut Engine) -> R) -> R {
        self.shared.with_engine(f)
    }
}

/// Where output pixel `(x, y)` of a `width` × `height` picture lives in the
/// framebuffer: a low-resolution pixel is the odd (main-screen) column of its
/// pair, a non-interlaced row the first of its pair.
#[must_use]
pub fn fb_index(x: usize, y: usize, width: usize, height: usize) -> usize {
    let col = if width <= 256 { 2 * x + 1 } else { x };
    let row = if height <= 239 { 2 * y } else { y };
    row * FB_WIDTH + col
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// `$2100`-`$213F`. Syncs before every access; a debug read has no effect.
struct PpuPort {
    shared: Arc<Shared>,
}

impl fmt::Debug for PpuPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PpuPort").finish_non_exhaustive()
    }
}

impl MemOps for PpuPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [byte] = dst else {
            return Err(BusError::BadAccess);
        };
        self.shared.sync(attrs);
        let index = (offset & 0x3F) as u8;
        *byte = if attrs.debug {
            self.shared
                .engine
                .lock()
                .read_register(index, attrs.bus, true)
        } else {
            self.shared
                .with_engine(|e| e.read_register(index, attrs.bus, false))
        };
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // Every write here moves a latch or an address; a debugger write
            // would be a guest action with no guest behind it.
            return Ok(());
        }
        self.shared.sync(attrs);
        let index = (offset & 0x3F) as u8;
        self.shared.with_engine(|e| e.write_register(index, *value));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}

/// The `EXTLATCH` input pin.
struct ExtLatchPin {
    shared: Arc<Shared>,
}

impl WireSink for ExtLatchPin {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        // Catch up first so the counters latch where the edge happened.
        self.shared.sync(MemAttrs::DEFAULT);
        self.shared.with_engine(|e| e.set_extlatch(level.is_high()));
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

/// Properties [`CLASS`] accepts.
static PROPERTIES: &[PropertySpec] = &[
    PropertySpec {
        name: "region",
        kind: ValueKind::Str,
        required: false,
        summary: "television standard: `ntsc` (262 lines) or `pal` (312); STAT78 bit 4",
    },
    PropertySpec {
        name: "ppu1-version",
        kind: ValueKind::Uint,
        required: false,
        summary: "5C77 version in STAT77 bits 0-3 (default 1)",
    },
    PropertySpec {
        name: "ppu2-version",
        kind: ValueKind::Uint,
        required: false,
        summary: "5C78 version in STAT78 bits 0-3 (default 3)",
    },
];

/// The `snes.ppu` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: "snes.ppu",
    version: engine::STATE_VERSION,
    summary: "SNES S-PPU (5C77 + 5C78): modes 0-7, sprites, windows, colour math, beam timing",
    properties: PROPERTIES,
    construct: |props| Ok(Box::new(SnesPpu::from_props(props)?) as Box<dyn Device>),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    reg.add(&CLASS)
}

impl Device for SnesPpu {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        self.announce_levels();
        Ok(())
    }

    fn unrealize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        self.shared.drive((false, false), true);
        let mut links = self.shared.links.lock();
        links.vblank.clear();
        links.hblank.clear();
        links.driven = None;
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        self.shared
            .with_engine(|e| e.reset(kind == ResetKind::Cold));
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.shared.engine.lock().save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let result = self.shared.with_engine(|e| e.load(r));
        // The restored levels are announced whether or not they differ from
        // what was driven before the restore.
        self.announce_levels();
        result
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        (name.is_empty() || name == REGISTER_REGION).then(|| Arc::clone(&self.regs))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            VBLANK_PIN => self.attach_vblank(source),
            HBLANK_PIN => self.attach_hblank(source),
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!("the S-PPU drives `{VBLANK_PIN}` and `{HBLANK_PIN}`"),
                });
            }
        }
        Ok(())
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        (port == EXTLATCH_PIN).then(|| SinkPin {
            sink: Arc::new(ExtLatchPin {
                shared: Arc::clone(&self.shared),
            }) as Arc<dyn WireSink>,
            line: 0,
        })
    }

    fn announce(&self, port: &str) {
        if port == VBLANK_PIN || port == HBLANK_PIN {
            self.announce_levels();
        }
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::SNES_PPU_BEAM).then(|| {
            Export::Opaque(Arc::clone(&self.beam) as Arc<dyn core::any::Any + Send + Sync>)
        })
    }

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.master.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        SnesPpu::advance_to(self, tick);
    }

    /// The next of the four in-line boundaries, which include every edge of
    /// both output levels.
    fn next_event_tick(&self) -> Option<u64> {
        Some(self.shared.next_event.load(Ordering::Relaxed))
    }

    /// Yes: `vblank` is the S-CPU's NMI source, and an NMI a quantum late is
    /// visible to any game that counts cycles in its handler.
    fn sampled_every_cycle(&self) -> bool {
        true
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        SnesPpu::attach_lazy(self, handle);
    }
}

impl crate::machine::Instance for SnesPpu {}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS.name, |props| {
        Ok(Arc::new(SnesPpu::from_props(props)?))
    })
}

/// What the validator should know about `snes.ppu`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS.name)
        .prop(PropSchema::new("region", ValueKind::Str).values(Region::NAMES))
        .prop(PropSchema::new("ppu1-version", ValueKind::Uint))
        .prop(PropSchema::new("ppu2-version", ValueKind::Uint))
        .port(VBLANK_PIN, PortDir::Out)
        .port(HBLANK_PIN, PortDir::Out)
        .port(EXTLATCH_PIN, PortDir::In)
        .region(REGISTER_REGION)
}
