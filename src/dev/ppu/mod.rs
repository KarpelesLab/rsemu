//! The NES / Famicom picture processing unit, RP2C02 (`dev-nes-ppu`).
//!
//! A cycle-accurate 2C02: 341 dots by 262 scanlines, the pre-render line, the
//! odd-frame skipped dot, the background fetch pipeline on its exact dots, the
//! two-phase sprite evaluation with its overflow bug, sprite 0 hit, and the
//! vblank/NMI race windows. It is written from the
//! [NESdev wiki](https://www.nesdev.org/wiki/PPU); the table below says which
//! page settles what.
//!
//! # Sources
//!
//! Everything here is written from the NESdev wiki, which documents the
//! hardware rather than anybody's emulation of it. The pages relied on, and
//! what each settles:
//!
//! | Page | What it fixes |
//! | --- | --- |
//! | [PPU rendering](https://www.nesdev.org/wiki/PPU_rendering) | the 341x262 dot grid, which fetch lands on which dot, shifter reloads, the odd-frame skip |
//! | [PPU scrolling](https://www.nesdev.org/wiki/PPU_scrolling) | `v`/`t`/`x`/`w`, the coarse-X and Y increments, the dot-257 and dot-280..304 copies, the tile and attribute address formulas |
//! | [PPU sprite evaluation](https://www.nesdev.org/wiki/PPU_sprite_evaluation) | the two-phase evaluation state machine and the overflow bug |
//! | [PPU registers](https://www.nesdev.org/wiki/PPU_registers) | every register bit, the `$2007` read buffer, OAMADDR quirks, the I/O latch |
//! | [PPU palettes](https://www.nesdev.org/wiki/PPU_palettes) | palette RAM mirroring and the backdrop override |
//! | [PPU frame timing](https://www.nesdev.org/wiki/PPU_frame_timing) | the vblank flag's set and clear dots and the `$2002` race windows |
//! | [NMI](https://www.nesdev.org/wiki/NMI) | `/NMI` as `vblank_flag AND nmi_output`, and its suppression |
//! | [PPU OAM](https://www.nesdev.org/wiki/PPU_OAM) | the sprite byte layout and the unimplemented attribute bits |
//!
//! # Shape
//!
//! | Type | Role |
//! | --- | --- |
//! | [`NesPpu`] | the [`Device`]: lifecycle, reset, snapshots, the links |
//! | [`PpuPort`] | the [`MemOps`] the CPU sees at `$2000`-`$3FFF` |
//! | [`Engine`] | the dot pipeline and every architectural bit |
//! | [`Pixel`] | one framebuffer entry: palette index plus emphasis |
//!
//! # Wiring a machine
//!
//! Until `RealizeCtx` grows accessors for spaces, wires and clocks (`ROADMAP.md`
//! §4.4), the connections are made with the `attach_*` methods below. They are
//! pure stores — nothing observable happens before
//! [`realize`](Device::realize), which is where the missing ones are reported.
//!
//! ```
//! use std::sync::Arc;
//!
//! use rsemu::core::clock::{ClockForest, Rational};
//! use rsemu::core::props::Props;
//! use rsemu::core::space::{AddressSpace, RamStore, Region, UnassignedPolicy};
//! use rsemu::dev::ppu::NesPpu;
//!
//! // The oscillator forest: one crystal, the CPU at ÷12 and the PPU at ÷4.
//! let mut forest = ClockForest::new();
//! let master = forest.add_oscillator("master", Rational::new(236_250_000, 11)?)?;
//! let cpu = forest.add_domain("cpu", master, 1, 12)?;
//! let dots = rsemu::dev::ppu::add_clock_domain(&mut forest, master)?;
//! // Exactly three dots per CPU cycle, by construction (ROADMAP.md §4.2).
//! assert_eq!(forest.convert_ticks(cpu, dots, 1)?, 3);
//!
//! // The PPU's own bus: $0000-$3FFF, pattern tables and nametables.
//! let mut vram = AddressSpace::new("ppu", 14).with_unassigned(UnassignedPolicy::ONES);
//! let chr = Arc::new(RamStore::new(0x2000));
//! vram.map(Arc::new(Region::ram("chr", chr)), 0x0000)?;
//! let nt = Arc::new(RamStore::new(0x1000));
//! vram.map(Arc::new(Region::ram("nametables", nt)), 0x2000)?;
//!
//! let ppu = NesPpu::new(&Props::new())?;
//! ppu.attach_bus(Arc::new(vram));
//! ppu.attach_clock(dots);
//! # Ok::<(), rsemu::Error>(())
//! ```
//!
//! # Time
//!
//! The PPU is a **lazily advanced** device (`ROADMAP.md` §4.2): it holds a dot
//! counter and the machine calls [`NesPpu::advance_to`] before dispatching any
//! access to it. Without that, every `$2002` read is thousands of dots stale and
//! the split-screen status bar in nearly every NES game is wrong.
//!
//! # OAM DMA
//!
//! `$4014` lives on the CPU side and the stall is the CPU's business. What the
//! PPU has to expose is the byte sink and the address register:
//! [`NesPpu::oam_dma_write`] delivers one byte exactly as a `$2004` write would,
//! and the OAM corruption behaviours (the glitched OAMADDR bump while rendering,
//! and the eight-byte copy when rendering starts with `OAMADDR >= 8`) live in
//! the engine where the timing that triggers them is.

mod engine;
mod regs;

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::sync::Arc;
use core::fmt;

use crate::core::clock::{ClockForest, ClockResult, DomainId};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult};
use crate::core::state::{ChunkReader, ChunkWriter};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};

pub use engine::{
    DEFAULT_DECAY_DOTS, DOTS_PER_FRAME, DOTS_PER_SCANLINE, Engine, EvalPhase, FRAMEBUFFER_LEN,
    NMI_ANNOUNCE_DOTS, PRE_RENDER_SCANLINE, Pixel, SCANLINES_PER_FRAME, SCREEN_HEIGHT,
    SCREEN_WIDTH, VBLANK_SCANLINE, WARMUP_DOTS,
};
pub use regs::{
    CTRL_BG_TABLE, CTRL_INCREMENT, CTRL_MASTER, CTRL_NAMETABLE, CTRL_NMI, CTRL_SPRITE_16,
    CTRL_SPRITE_TABLE, IoLatch, MASK_BG, MASK_BG_LEFT, MASK_EMPHASIS_B, MASK_EMPHASIS_G,
    MASK_EMPHASIS_R, MASK_GREYSCALE, MASK_RENDERING, MASK_SPRITE, MASK_SPRITE_LEFT, OAMADDR,
    OAMDATA, PPUADDR, PPUCTRL, PPUDATA, PPUMASK, PPUSCROLL, PPUSTATUS, SPRITE_ATTR_IMPLEMENTED,
    SPRITE_BEHIND, SPRITE_FLIP_X, SPRITE_FLIP_Y, SPRITE_PALETTE, STATUS_DRIVEN, STATUS_OVERFLOW,
    STATUS_SPRITE0, STATUS_VBLANK,
};

/// Where the register block sits in the CPU's address space.
pub const REGISTER_BASE: u64 = 0x2000;

/// How far the register block runs: `$2000`-`$3FFF`.
///
/// Eight registers mirrored every eight bytes for 8 KiB. The mirroring is inside
/// the device rather than expressed as 1024 alias regions, because it is one
/// `& 7` and a flat view with a thousand entries would be a poor trade
/// (`ROADMAP.md` §4.1).
pub const REGISTER_WINDOW_LEN: u64 = 0x2000;

/// The PPU dot domain's rate relative to the NES master crystal: master ÷ 4.
pub const DOT_DIVIDER: u64 = 4;

/// Add the PPU's clock domain under `master`, rated master ÷ 4.
///
/// The CPU is master ÷ 12 and the PPU master ÷ 4, so the PPU advances exactly
/// three dots per CPU cycle — forever, on every console ever made, because both
/// counters descend from one crystal. That ratio is what games depend on, not
/// the absolute frequency (`ROADMAP.md` §4.2).
///
/// # Errors
///
/// Whatever [`ClockForest::add_domain`] reports: an unknown parent, or no exact
/// common unit tick for the tree.
pub fn add_clock_domain(forest: &mut ClockForest, master: DomainId) -> ClockResult<DomainId> {
    forest.add_domain("ppu", master, 1, DOT_DIVIDER)
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

/// What the device and its memory port both hold.
struct Shared {
    /// The dot engine.
    ///
    /// Ranked [`LockRank::BUS`] rather than [`LockRank::DEVICE`] because the PPU
    /// is a bus master: it holds this across CHR fetches into the cartridge,
    /// which take a device lock of their own. A rank at `DEVICE` would make
    /// every pattern fetch a rank violation.
    engine: Mutex<Engine>,
    /// The NMI request line and the clock domain. Taken *after* `engine` and
    /// never held across the outward `set`.
    links: Mutex<Links>,
}

#[derive(Debug, Default)]
struct Links {
    nmi: Option<WireSource>,
    clock: Option<DomainId>,
}

impl Shared {
    /// Run `f` against the engine, then settle the NMI line outside the lock.
    ///
    /// This is the re-entrancy contract in one function: mutate own state in a
    /// short critical section, release it, *then* act outward
    /// (`ROADMAP.md` §4.4).
    fn with_engine<R>(&self, f: impl FnOnce(&mut Engine) -> R) -> R {
        let (result, nmi) = {
            let mut engine = self.engine.lock();
            let result = f(&mut engine);
            (result, engine.nmi_active())
        };
        self.drive_nmi(nmi);
        result
    }

    fn drive_nmi(&self, active: bool) {
        // Cloned out so the link lock is not held across the wire's fan-out.
        let source = {
            let links = self.links.lock();
            links.nmi.clone()
        };
        if let Some(source) = source {
            source.set(Level::from_bool(active));
        }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The RP2C02 picture processing unit.
///
/// Cloneable handles onto one chip: [`NesPpu::port`] hands the CPU-facing
/// register block to the address space while the machine keeps the device.
pub struct NesPpu {
    shared: Arc<Shared>,
}

impl fmt::Debug for NesPpu {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("NesPpu")
            .field("engine", &self.shared.engine)
            .finish()
    }
}

impl NesPpu {
    /// Validate properties and allocate. Performs no outward action.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Property`] for an unknown or ill-typed property.
    pub fn new(props: &Props) -> Result<NesPpu> {
        let mut r = props.reader();
        let warmup = r.or("warmup", true)?;
        let decay = r.or("open-bus-decay-dots", DEFAULT_DECAY_DOTS)?;
        r.finish()?;
        Ok(NesPpu {
            shared: Arc::new(Shared {
                engine: Mutex::with_rank(LockRank::BUS, Engine::new(warmup, decay)),
                links: Mutex::with_rank(LockRank::WIRE, Links::default()),
            }),
        })
    }

    /// Connect the PPU's own 14-bit address space: pattern tables at
    /// `$0000`-`$1FFF` and nametables at `$2000`-`$2FFF`, both supplied by the
    /// cartridge. Palette RAM is *not* on this bus; it is inside the chip.
    pub fn attach_bus(&self, space: Arc<AddressSpace>) {
        self.shared.engine.lock().bus = Some(space);
    }

    /// Connect the NMI request line.
    ///
    /// Driven **high when the NMI is requested**. That is the logical assertion,
    /// not the chip's active-low `/NMI` pin: [`crate::core::wire`] nets idle low,
    /// and inverting is a `wire.not` device's job (`ROADMAP.md` §4.3).
    pub fn attach_nmi(&self, source: WireSource) {
        self.shared.links.lock().nmi = Some(source);
    }

    /// Record which clock domain counts this PPU's dots.
    pub fn attach_clock(&self, domain: DomainId) {
        self.shared.links.lock().clock = Some(domain);
    }

    /// The clock domain [`attach_clock`](NesPpu::attach_clock) recorded.
    pub fn clock_domain(&self) -> Option<DomainId> {
        self.shared.links.lock().clock
    }

    /// The CPU-facing register block, ready to be wrapped in a
    /// [`Region::io`](crate::core::space::Region::io) of
    /// [`REGISTER_WINDOW_LEN`] bytes at [`REGISTER_BASE`].
    pub fn port(&self) -> Arc<PpuPort> {
        Arc::new(PpuPort {
            shared: Arc::clone(&self.shared),
        })
    }

    // -- time ---------------------------------------------------------------

    /// Dots executed since the last reset.
    pub fn dots(&self) -> u64 {
        self.shared.engine.lock().dots
    }

    /// Frames completed since the last reset.
    pub fn frame(&self) -> u64 {
        self.shared.engine.lock().frame
    }

    /// The position of the dot that will run next, as `(scanline, dot)`.
    pub fn position(&self) -> (u16, u16) {
        let engine = self.shared.engine.lock();
        (engine.scanline, engine.dot)
    }

    /// Run the pipeline until `target` dots have executed in total.
    ///
    /// The catch-up entry point (`ROADMAP.md` §4.2): the machine calls this
    /// before dispatching a CPU access to the register block, so a `$2002` read
    /// lands on the dot it really happened on. Running backwards is not an
    /// error, it is a no-op.
    pub fn advance_to(&self, target: u64) {
        loop {
            let (reached, nmi) = {
                let mut engine = self.shared.engine.lock();
                let entry = engine.nmi_active();
                let reached = engine.run_to(target, entry);
                (reached, engine.nmi_active())
            };
            // Outside the lock, every time the request level moved — a long
            // budget can contain both the assert and the deassert.
            self.shared.drive_nmi(nmi);
            if reached {
                return;
            }
        }
    }

    /// Run exactly `dots` more dots.
    pub fn advance_by(&self, dots: u64) {
        let target = self.shared.engine.lock().dots + dots;
        self.advance_to(target);
    }

    // -- ports --------------------------------------------------------------

    /// Read register `index` (0-7, i.e. `$2000` + `index`).
    pub fn read_register(&self, index: u8) -> u8 {
        self.shared.with_engine(|e| e.read_register(index, false))
    }

    /// Write register `index` (0-7).
    pub fn write_register(&self, index: u8, value: u8) {
        self.shared.with_engine(|e| e.write_register(index, value));
    }

    /// Deliver one OAM DMA byte, exactly as a `$2004` write would.
    ///
    /// The `$4014` register and the 513/514-cycle CPU stall belong to the CPU;
    /// this is the PPU half. Because it goes through the same path as `$2004`, a
    /// DMA that runs while rendering is enabled hits the same glitched OAMADDR
    /// bump hardware does — which is why a game is told to write `$00` to
    /// `$2003` first ([NESdev PPU registers](https://www.nesdev.org/wiki/PPU_registers)).
    pub fn oam_dma_write(&self, value: u8) {
        self.shared
            .with_engine(|e| e.write_register(regs::OAMDATA, value));
    }

    /// The current OAM address.
    pub fn oam_addr(&self) -> u8 {
        self.shared.engine.lock().oam_addr
    }

    /// Read one OAM byte without disturbing anything — for a monitor or a test.
    pub fn peek_oam(&self, addr: u8) -> u8 {
        self.shared.engine.lock().oam[usize::from(addr)]
    }

    /// Write one OAM byte directly, masking the three unimplemented attribute
    /// bits. Bypasses OAMADDR and the rendering interlock, so it is for machine
    /// setup and tests, not for a guest.
    pub fn poke_oam(&self, addr: u8, value: u8) {
        self.shared.engine.lock().write_oam(addr, value);
    }

    /// Read one palette entry through the mirroring rules.
    pub fn peek_palette(&self, addr: u16) -> u8 {
        self.shared.engine.lock().palette_read(addr)
    }

    /// Write one palette entry through the mirroring rules.
    pub fn poke_palette(&self, addr: u16, value: u8) {
        self.shared.engine.lock().palette_write(addr, value);
    }

    // -- output -------------------------------------------------------------

    /// Borrow the framebuffer: [`SCREEN_WIDTH`] x [`SCREEN_HEIGHT`] [`Pixel`]s,
    /// row-major from the top left.
    ///
    /// A callback rather than a slice because the buffer lives behind the engine
    /// lock, and handing out a borrow of it would either leak the guard or copy
    /// 60 kB nobody asked for.
    pub fn with_framebuffer<R>(&self, f: impl FnOnce(&[Pixel]) -> R) -> R {
        let engine = self.shared.engine.lock();
        f(&engine.fb)
    }

    /// One framebuffer pixel.
    pub fn pixel(&self, x: usize, y: usize) -> Option<Pixel> {
        if x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT {
            return None;
        }
        Some(self.shared.engine.lock().fb[y * SCREEN_WIDTH + x])
    }

    /// Run the engine directly. The escape hatch tests and the machine layer
    /// use; a guest never gets here.
    pub fn with_engine<R>(&self, f: impl FnOnce(&mut Engine) -> R) -> R {
        self.shared.with_engine(f)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

/// Properties [`NES_PPU_CLASS`] accepts.
static PPU_PROPERTIES: &[PropertySpec] = &[
    PropertySpec {
        name: "warmup",
        kind: ValueKind::Bool,
        required: false,
        summary: "honour the ~29658-CPU-cycle lockout on $2000/$2001/$2005/$2006 writes after reset",
    },
    PropertySpec {
        name: "open-bus-decay-dots",
        kind: ValueKind::Uint,
        required: false,
        summary: "how many PPU dots a bit of the I/O latch holds its charge",
    },
];

/// The device class, for [`crate::core::Registry`].
pub static NES_PPU_CLASS: DeviceClass = DeviceClass {
    name: "nes.ppu",
    version: 1,
    summary: "NES / Famicom picture processing unit (RP2C02)",
    properties: PPU_PROPERTIES,
    construct: |props| Ok(Box::new(NesPpu::new(props)?) as Box<dyn Device>),
};

/// Register this build's PPU class.
///
/// # Errors
///
/// [`crate::Error::Config`] if the name is already taken, which means two
/// features collided.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&NES_PPU_CLASS)
}

impl Device for NesPpu {
    fn class(&self) -> &'static DeviceClass {
        &NES_PPU_CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        if self.shared.engine.lock().bus.is_none() {
            return Err(ctx.error(
                "no PPU address space attached: call attach_bus with the $0000-$3FFF space \
                 the cartridge provides",
            ));
        }
        // Realize must leave every wire driving what its state implies, or a
        // freshly built machine comes up with the interrupt line wrong
        // (`ROADMAP.md` §4.3).
        let nmi = self.shared.engine.lock().nmi_active();
        self.shared.drive_nmi(nmi);
        Ok(())
    }

    fn unrealize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        self.shared.drive_nmi(false);
        self.shared.links.lock().nmi = None;
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        self.shared.with_engine(|e| match kind {
            ResetKind::Cold => e.reset_cold(),
            // A bus reset does not reach the PPU on a NES: it hangs off the CPU
            // directly, not off a bus that can be reset independently.
            ResetKind::Warm | ResetKind::Bus => e.reset_warm(),
        });
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.shared.engine.lock().save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let result = { self.shared.engine.lock().load(r) };
        // The restored state implies an NMI level that nothing has announced.
        let nmi = self.shared.engine.lock().nmi_active();
        self.shared.drive_nmi(nmi);
        result
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The CPU-facing `$2000`-`$3FFF` register block.
///
/// Byte accesses only, and no bulk transfers: the ports have side effects, so a
/// four-byte read would pop the `$2007` buffer four times. Mirroring is `& 7`.
///
/// **This port does not advance time.** The machine calls
/// [`NesPpu::advance_to`] first (`ROADMAP.md` §4.2); a port that caught up by
/// itself would need the scheduler's current time, which is not something a
/// [`MemOps`] implementation is handed.
pub struct PpuPort {
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
        let index = (offset & 7) as u8;
        *byte = self
            .shared
            .with_engine(|e| e.read_register(index, attrs.debug));
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [value] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debugger write to a port with side effects is not something the
            // core can make safe, so it is refused rather than guessed at.
            return Err(BusError::BadAccess);
        }
        let index = (offset & 7) as u8;
        self.shared.with_engine(|e| e.write_register(index, *value));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}
