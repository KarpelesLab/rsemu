//! The NES / Famicom picture processing unit (`dev-nes-ppu`).
//!
//! A cycle-accurate 2C02: 341 dots by 262 scanlines, the pre-render line, the
//! odd-frame skipped dot, the background fetch pipeline on its exact dots, the
//! two-phase sprite evaluation with its overflow bug, sprite 0 hit, and the
//! vblank/NMI race windows. It is written from the
//! [NESdev wiki](https://www.nesdev.org/wiki/PPU); the table below says which
//! page settles what.
//!
//! # Regions
//!
//! [`Region`] picks the chip: NTSC (RP2C02), PAL (RP2C07) or Dendy (UA6538).
//! It is a construction property — `region = "pal"` — never a `#[cfg]`, so one
//! build runs all three:
//!
//! | | NTSC | PAL | Dendy |
//! | --- | --- | --- | --- |
//! | Master ÷ CPU | 12 | 16 | 15 |
//! | Master ÷ dot | 4 | 5 | 5 |
//! | Scanlines | 262 | 312 | 312 |
//! | Post-render lines | 1 | 1 | 51 |
//! | VBlank at scanline | 241 | 241 | 291 |
//! | VBlank lines | 20 | 70 | 20 |
//! | Odd-frame skip | yes | no | no |
//!
//! Two things the framework was built for show up here. Neither master clock is
//! an integer number of hertz, and PAL runs **3.2 dots per CPU cycle** — which
//! is exact only because the forest counts master ticks rather than CPU cycles
//! (`ROADMAP.md` §4.2). There is deliberately no dots-per-CPU-cycle constant
//! anywhere in this module; see [`Region`] and [`add_clock_domain`].
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
//! A described machine needs none of this: `machines/nes-ntsc.machine` names
//! the object, maps `ppu.regs` at `$2000`, gives it `space = ppubus` and wires
//! `ppu.nmi -> cpu.nmi`, and the realizer does the rest through
//! [`Instance::bind`] and [`Device::attach_lazy`].
//!
//! Assembling one by hand — which is what the tests and this example do — means
//! calling the `attach_*` methods below. They are pure stores: nothing
//! observable happens before [`realize`](Device::realize).
//!
//! ```
//! use std::sync::Arc;
//!
//! use rsemu::core::clock::{ClockForest, Rational};
//! use rsemu::core::props::Props;
//! use rsemu::core::space::{AddressSpace, RamStore, Region as MmioRegion, UnassignedPolicy};
//! use rsemu::dev::ppu::{NesPpu, Region};
//!
//! // The oscillator forest: one crystal, the CPU at ÷12 and the PPU at ÷4.
//! let region = Region::Ntsc;
//! let (num, den) = region.master_clock();
//! let mut forest = ClockForest::new();
//! let master = forest.add_oscillator("master", Rational::new(num, den)?)?;
//! let cpu = forest.add_domain("cpu", master, 1, region.cpu_divider())?;
//! let dots = rsemu::dev::ppu::add_clock_domain(&mut forest, master, region)?;
//! // Exactly three dots per CPU cycle, by construction (ROADMAP.md §4.2).
//! assert_eq!(forest.convert_ticks(cpu, dots, 1)?, 3);
//!
//! // The PPU's own bus: $0000-$3FFF, pattern tables and nametables.
//! let vram = AddressSpace::new("ppu", 14).with_unassigned(UnassignedPolicy::ONES);
//! let chr = Arc::new(RamStore::new(0x2000));
//! let nt = Arc::new(RamStore::new(0x1000));
//! // One topology guard covers the whole batch (`core::space`).
//! {
//!     let mut topo = vram.topology();
//!     topo.map(Arc::new(MmioRegion::ram("chr", chr)), 0x0000)?;
//!     topo.map(Arc::new(MmioRegion::ram("nametables", nt)), 0x2000)?;
//! }
//!
//! let ppu = NesPpu::new(&Props::new().with("region", "ntsc"))?;
//! ppu.attach_bus(Arc::new(vram));
//! ppu.attach_clock(dots);
//! # Ok::<(), rsemu::Error>(())
//! ```
//!
//! # Time
//!
//! The PPU is a **lazily advanced** device (`ROADMAP.md` §4.2): it holds a dot
//! counter, and it is caught up before any access is dispatched to it. Without
//! that, every `$2002` read is thousands of dots stale and the split-screen
//! status bar in nearly every NES game is wrong.
//!
//! [`Device::is_lazy`] is how it says so, and the machine layer answers with a
//! [`LazyHandle`] through [`NesPpu::attach_lazy`]. The handle is what makes the
//! catch-up reachable at all: it fires from inside [`PpuPort::read`], which
//! takes `&self` and runs several frames below whoever owns the scheduler.
//! Two halves, and both are needed:
//!
//! * **Sampled** — the port syncs before it answers, so a `$2002` read lands on
//!   the dot it really happened on.
//! * **Scheduled** — the run loop bounds each quantum by
//!   [`Engine::next_event_dot`] and catches the chip up there, so vblank is
//!   raised on its own dot even though the CPU is spinning on a RAM flag and
//!   will not touch a PPU register until the NMI arrives.
//!
//! Two consequences for anyone extending this module. [`Device::current_tick`]
//! and [`Device::next_event_tick`] are asked with the scheduler's slot held at
//! [`LockRank::LEAF`], so they read atomics and **must not take the engine
//! lock**; every critical section that moves the engine republishes them. And
//! the engine lock is ranked between `BUS` and `DEVICE` — see
//! [`ENGINE_LOCK_RANK`], which is where the whole squeeze is written out.
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
mod region;
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
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::{BindCtx, Instance};

pub use engine::{
    DEFAULT_DECAY_DOTS, DOTS_PER_FRAME, DOTS_PER_SCANLINE, Engine, EvalPhase, FRAMEBUFFER_LEN,
    NMI_ANNOUNCE_DOTS, PRE_RENDER_SCANLINE, Pixel, SCANLINES_PER_FRAME, SCREEN_HEIGHT,
    SCREEN_WIDTH, VBLANK_SCANLINE, WARMUP_DOTS,
};
pub use region::{BORDER_BLACK, Geometry, RESET_LOCKOUT_CPU_CYCLES, Region};
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

/// The name of the region a `map` statement reaches the register block by.
///
/// Also what an empty region name resolves to, so
/// `map cpubus 0x2000 size 8K = ppu` is enough.
pub const REGISTER_REGION: &str = "regs";

/// The name of the `/NMI` output pin.
pub const NMI_PIN: &str = "nmi";

/// Add the PPU's dot domain under `master`, rated master ÷
/// [`Region::dot_divider`].
///
/// On NTSC the CPU is master ÷ 12 and the PPU master ÷ 4, so the PPU advances
/// exactly three dots per CPU cycle — forever, on every console ever made,
/// because both counters descend from one crystal. On PAL the dividers are 16
/// and 5, which is 3.2 dots per CPU cycle: not an integer, and *still exact*,
/// because the forest counts master ticks rather than CPU cycles
/// (`ROADMAP.md` §4.2). That is why this function takes the divider and no
/// dots-per-CPU-cycle figure exists anywhere in this module.
///
/// # Errors
///
/// Whatever [`ClockForest::add_domain`] reports: an unknown parent, or no exact
/// common unit tick for the tree.
pub fn add_clock_domain(
    forest: &mut ClockForest,
    master: DomainId,
    region: Region,
) -> ClockResult<DomainId> {
    forest.add_domain("ppu", master, 1, region.dot_divider())
}

// ---------------------------------------------------------------------------
// Shared state
// ---------------------------------------------------------------------------

/// Where the engine lock sits in the ladder: **between**
/// [`LockRank::BUS`] and [`LockRank::DEVICE`].
///
/// Not one of `core::sync`'s named ranks, and the reason is the whole of §4.2's
/// catch-up made concrete. The PPU is squeezed from both ends:
///
/// * It is reached **from under a `BUS`-ranked lock**. Sync-on-access fires
///   from inside `MemOps::read`, and the 6502 holds its own execution state at
///   `LockRank::BUS` across every access it issues. `BUS` again here would be
///   `BUS <= BUS` and the debug order check would fire on the first `$2002`
///   read of the first game — correctly, because on a threaded backend two bus
///   masters at one rank is a cycle waiting to happen.
/// * It is held **across CHR fetches into the cartridge**, which take
///   `DEVICE`-ranked locks of their own. The PPU is a bus master, so `DEVICE`
///   would make every pattern fetch a violation the other way.
///
/// So it goes between them, in the gap the named ranks were spaced `0x1000`
/// apart to leave. Nothing else claims this rank, and the invariant is the
/// ordinary one: a lock taken while it is held must rank above it.
pub const ENGINE_LOCK_RANK: LockRank = LockRank::new(0x4800);

/// What the device and its memory port both hold.
struct Shared {
    /// The dot engine. See [`ENGINE_LOCK_RANK`].
    engine: Mutex<Engine>,
    /// The NMI request line and the clock domain. Taken *after* `engine` and
    /// never held across the outward `set`.
    links: Mutex<Links>,
    /// The catch-up handle the machine layer hands over at realize time.
    ///
    /// Its own leaf-ranked lock rather than a field of `links`: it is read at
    /// the top of every guest access, and `links` is `WIRE`-ranked, which the
    /// engine lock would then be unable to nest under.
    lazy: Mutex<Option<LazyHandle>>,
    /// [`Engine::dots`], republished on every release of the engine lock.
    ///
    /// The scheduler asks a lazily-advanced device where it is *while holding
    /// its slot at [`LockRank::LEAF`]* — the rank nothing nests under — so
    /// [`Device::current_tick`] may not take a lock. An atomic is the answer,
    /// and it is the better one anyway: this is the hot path.
    dots: AtomicU64,
    /// [`Engine::next_event_dot`], republished alongside [`Shared::dots`] and
    /// read under the same constraint.
    next_event: AtomicU64,
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
            self.publish(&engine);
            (result, engine.nmi_active())
        };
        self.drive_nmi(nmi);
        result
    }

    /// Republish what the lock-free lazy surface reads.
    ///
    /// Called from inside every critical section that can move the dot counter
    /// or change what the chip's next self-driven event is — which is every one
    /// that takes the engine mutably, since `$2000` alone decides whether the
    /// vblank flag will raise `/NMI`.
    fn publish(&self, engine: &Engine) {
        self.dots.store(engine.dots, Ordering::Relaxed);
        self.next_event
            .store(engine.next_event_dot(), Ordering::Relaxed);
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

    /// Catch this chip up before an access is dispatched to it (§4.2).
    ///
    /// Takes no lock of its own beyond the handle's leaf, and holds none while
    /// the scheduler calls back into [`NesPpu::advance_to`] — which is what
    /// lets that call take the engine lock, reach the cartridge for a pattern
    /// fetch, and drive `/NMI`, all from inside a CPU access that is already
    /// holding the 6502's own `BUS`-ranked lock.
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
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
        // A refusal means catch-up for this chip is already running further up
        // the stack — the PPU reading its own registers through its own bus,
        // which no NES does. The access still has to be answered, and answering
        // it from where the chip stands is the only defined thing to do.
        let _ = handle.sync(kind);
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
    /// The `$2000`-`$3FFF` aperture, built once at construction.
    ///
    /// Built here rather than in [`NesPpu::region`] so that every `map`
    /// statement naming this device gets the *same* region: a fresh `Arc` per
    /// call would be a second identity for one piece of hardware, and
    /// `AddressSpace::rebase` identifies a mapping by exactly that.
    regs: RegionRef,
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
        let name = r.or_enum("region", Region::Ntsc.name(), Region::NAMES)?;
        let warmup = r.or("warmup", true)?;
        let decay = r.or("open-bus-decay-dots", DEFAULT_DECAY_DOTS)?;
        r.finish()?;
        // `or_enum` has already rejected everything else; this is the one place
        // the name becomes the variant, so it stays a checked conversion.
        let region = Region::from_name(name).ok_or_else(|| {
            crate::core::Error::Property(alloc::format!("unknown `region` `{name}`"))
        })?;
        let engine = Engine::new(region, warmup, decay);
        let next_event = engine.next_event_dot();
        let shared = Arc::new(Shared {
            engine: Mutex::with_rank(ENGINE_LOCK_RANK, engine),
            links: Mutex::with_rank(LockRank::WIRE, Links::default()),
            lazy: Mutex::new(None),
            dots: AtomicU64::new(0),
            next_event: AtomicU64::new(next_event),
        });
        let regs = Arc::new(MmioRegion::io(
            "nes.ppu.regs",
            REGISTER_WINDOW_LEN,
            Arc::new(PpuPort {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        Ok(NesPpu { shared, regs })
    }

    /// Which console this PPU is.
    ///
    /// Named `tv_region` rather than `region` because [`Device::region`] is the
    /// MMIO aperture lookup and the two would shadow each other on the concrete
    /// type.
    pub fn tv_region(&self) -> Region {
        self.shared.engine.lock().tv_region()
    }

    /// The frame geometry [`NesPpu::tv_region`] implies.
    pub fn geometry(&self) -> Geometry {
        self.shared.engine.lock().geometry()
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

    /// Connect the catch-up handle the register block syncs through (§4.2).
    ///
    /// The machine layer calls this from realize; a caller wiring a NES by hand
    /// registers the chip with `Scheduler::add_lazy_device` and passes the
    /// handle here. Without one the register block answers from wherever the
    /// chip happens to be standing, which is why the machine layer never
    /// leaves it unset.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// The dot the chip's own next self-driven event falls on — the catch-up
    /// bound of §4.2, and what a run loop clamps its quantum by.
    pub fn next_event_dot(&self) -> u64 {
        self.shared.next_event.load(Ordering::Relaxed)
    }

    /// The CPU-facing register block, ready to be wrapped in a
    /// [`Region::io`](crate::core::space::Region::io) of
    /// [`REGISTER_WINDOW_LEN`] bytes at [`REGISTER_BASE`].
    ///
    /// A machine described in the DSL does not need this: [`Device::region`]
    /// hands out the same window already wrapped.
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
                self.shared.publish(&engine);
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
        name: "region",
        kind: ValueKind::Str,
        required: false,
        summary: "console variant: `ntsc` (RP2C02), `pal` (RP2C07) or `dendy` (UA6538)",
    },
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
    summary: "NES / Famicom picture processing unit (RP2C02 / RP2C07 / UA6538)",
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

    /// # Why the PPU bus is not required here
    ///
    /// It used to be, and for a hand-wired machine that was the right place.
    /// The machine layer hands a device its address space at *bind* time, which
    /// runs after realize — a device may read through its space from `bind`,
    /// and every region has to be mapped before that is true — so a
    /// DSL-described PPU has no bus yet at this point. The check moved to
    /// [`Instance::bind`], which is where the machine layer is in a position to
    /// supply one and therefore where its absence is a real error.
    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
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
        let (result, nmi) = {
            let mut engine = self.shared.engine.lock();
            let result = engine.load(r);
            // The dot counter and the next event both came from the snapshot;
            // the lock-free copies of them are derived state and have to follow.
            self.shared.publish(&engine);
            let nmi = engine.nmi_active();
            (result, nmi)
        };
        // The restored state implies an NMI level that nothing has announced.
        self.shared.drive_nmi(nmi);
        result
    }

    // -- the connection surface (`ROADMAP.md` §4.4) --------------------------

    /// The register block, for `map cpubus 0x2000 size 8K = ppu`.
    ///
    /// One aperture, so the empty name and [`REGISTER_REGION`] both reach it.
    /// The 1024-fold mirroring inside `$2000`-`$3FFF` is the port's `& 7`, not
    /// a thousand alias regions (`ROADMAP.md` §4.1).
    fn region(&self, name: &str) -> Option<RegionRef> {
        (name.is_empty() || name == REGISTER_REGION).then(|| Arc::clone(&self.regs))
    }

    /// The `/NMI` output.
    ///
    /// Driven **high when the NMI is requested**, which is the logical
    /// assertion rather than the chip's active-low pin: nets idle low and
    /// inverting is a `wire.not` device's job (`ROADMAP.md` §4.3).
    ///
    /// The PPU has no wire *input* — nothing on a NES drives a pin of it — so
    /// [`Device::sink`] keeps its default.
    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != NMI_PIN {
            return Err(crate::core::Error::Config {
                at: alloc::string::String::from(port),
                message: alloc::format!("the PPU drives only `{NMI_PIN}`"),
            });
        }
        self.attach_nmi(source);
        Ok(())
    }

    /// Announce what `/NMI` idles at, for the realize sweep (§4.3).
    ///
    /// It idles low out of reset, but a machine restored from a snapshot inside
    /// vblank comes up asserting, and a net nobody announced to would be wrong.
    fn announce(&self, port: &str) {
        if port == NMI_PIN {
            let nmi = self.shared.engine.lock().nmi_active();
            self.shared.drive_nmi(nmi);
        }
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. The whole reason sync-on-access exists.
    ///
    /// Running the chip dot by dot in lockstep with the CPU would be far more
    /// expensive than running it in bursts, but a `$2002` read has to see the
    /// state at exactly the dot it happened on — sprite 0 and the vblank race
    /// included. So the chip keeps its own dot counter and whoever touches it
    /// catches it up first.
    fn is_lazy(&self) -> bool {
        true
    }

    /// Dots executed, read from the atomic rather than the engine: the
    /// scheduler asks this with its slot held at `LockRank::LEAF`.
    fn current_tick(&self) -> u64 {
        self.shared.dots.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        NesPpu::advance_to(self, tick);
    }

    /// See [`Engine::next_event_dot`]: the vblank edges, floored at the next
    /// scanline so a mid-quantum `$2002` read is never more than a line stale.
    fn next_event_tick(&self) -> Option<u64> {
        Some(self.shared.next_event.load(Ordering::Relaxed))
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        NesPpu::attach_lazy(self, handle);
    }
}

/// The machine layer's half: the PPU takes a clock domain and an address space
/// of its own, neither of which `Device` has a way to be told about.
impl Instance for NesPpu {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| crate::core::Error::Config {
            at: alloc::string::String::from(ctx.path()),
            message: alloc::string::String::from(
                "the PPU needs an address space of its own: add `space = ppubus` to the object \
                 and map the cartridge's pattern tables and the console's nametables into it",
            ),
        })?;
        self.attach_bus(Arc::clone(space));
        if let Some(domain) = ctx.domain() {
            self.attach_clock(domain);
        }
        Ok(())
    }
}

/// Bind [`NES_PPU_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(NES_PPU_CLASS.name, |props| {
        Ok(Arc::new(NesPpu::new(props)?))
    })
}

/// What the validator should know about `nes.ppu`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(NES_PPU_CLASS.name)
        .prop(PropSchema::new("region", ValueKind::Str).values(Region::NAMES))
        .prop(PropSchema::new("warmup", ValueKind::Bool))
        .prop(PropSchema::new("open-bus-decay-dots", ValueKind::Uint))
        // One output and no inputs: nothing on a NES drives a pin of the PPU.
        .port(NMI_PIN, PortDir::Out)
        .region(REGISTER_REGION)
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The CPU-facing `$2000`-`$3FFF` register block.
///
/// Byte accesses only, and no bulk transfers: the ports have side effects, so a
/// four-byte read would pop the `$2007` buffer four times. Mirroring is `& 7`.
///
/// **This port advances the chip before it answers** (`ROADMAP.md` §4.2). The
/// [`LazyHandle`] the machine layer attaches at realize time is what makes that
/// reachable: `read` takes `&self` and runs several frames below whoever owns
/// the scheduler, so there is no route back to it — the handle is the route.
/// Catch-up runs with no lock of this device held, and only then is the engine
/// locked to answer, which is why a pattern fetch during catch-up cannot meet
/// the access that triggered it.
///
/// A `MemAttrs::debug` access advances nothing at all: a monitor reading
/// `$2002` must not move the chip's clock any more than it may clear the vblank
/// flag (invariant 5).
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
        // First, and outside every lock this device owns.
        self.shared.sync(attrs);
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
        // A write is as time-sensitive as a read: `$2000` written one dot
        // either side of the vblank flag decides whether this frame's NMI
        // happens at all.
        self.shared.sync(attrs);
        let index = (offset & 7) as u8;
        self.shared.with_engine(|e| e.write_register(index, *value));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO.with_widths(Width::U8, Width::U8)
    }
}
