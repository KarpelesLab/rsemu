//! The RP2C02 dot engine: one `tick` is one PPU dot.
//!
//! The table of NESdev pages this is written from is in the parent module's
//! documentation; individual decisions cite the page they come from where they
//! are made.

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec;
use core::fmt;

use crate::core::error::Result;
use crate::core::space::{AddressSpace, MemAttrs};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::value::Width;

use super::region::{BORDER_BLACK, Geometry, Region};
use super::regs::*;

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------

/// Dots in one scanline, visible and blanking together.
///
/// 341 in every region: the differences between the 2C02, the 2C07 and the
/// UA6538 are all vertical, plus the master clocks each dot takes.
pub const DOTS_PER_SCANLINE: u16 = 341;
/// Visible pixels per scanline.
pub const SCREEN_WIDTH: usize = 256;
/// Scanlines the render pipeline draws, in every region.
///
/// PAL and Dendy show 239 of them, because their video border paints over the
/// top one ([`Geometry::picture_height`]); all three chips still *render* 240,
/// so the framebuffer is one shape everywhere and so is the snapshot.
pub const SCREEN_HEIGHT: usize = 240;
/// Framebuffer length, in pixels.
pub const FRAMEBUFFER_LEN: usize = SCREEN_WIDTH * SCREEN_HEIGHT;

/// Scanlines in one NTSC frame.
///
/// The other regions are in [`Geometry::scanlines_per_frame`]; this and the
/// constants below name the NTSC figure because it is the default region and
/// because most of the wiki's prose is written about the 2C02.
pub const SCANLINES_PER_FRAME: u16 = Region::Ntsc.geometry().scanlines_per_frame;
/// Dots in a full (even) NTSC frame. An odd frame with rendering enabled is one
/// dot shorter. See [`Geometry::dots_per_frame`].
pub const DOTS_PER_FRAME: u64 = Region::Ntsc.geometry().dots_per_frame;
/// The NTSC scanline on which the vertical blank flag is set (at dot 1).
/// See [`Geometry::vblank_scanline`].
pub const VBLANK_SCANLINE: u16 = Region::Ntsc.geometry().vblank_scanline;
/// The NTSC pre-render (dummy) scanline. See [`Geometry::pre_render_scanline`].
pub const PRE_RENDER_SCANLINE: u16 = Region::Ntsc.geometry().pre_render_scanline;

/// PPU dots the 2C02 ignores writes to `$2000`, `$2001`, `$2005` and `$2006`
/// after a reset.
///
/// The measured figure is ~29658 CPU cycles, and the CPU:PPU ratio is exactly
/// 3:1 because both descend from one crystal (`ROADMAP.md` §4.2), so the dot
/// count is exact even though the master frequency is irrational.
/// [NESdev PPU registers](https://www.nesdev.org/wiki/PPU_registers), PPUCTRL.
/// PAL's is not a whole number of dots; see [`Geometry::warmup_dots`].
pub const WARMUP_DOTS: u64 = Region::Ntsc.geometry().warmup_dots;

/// Default life of a charge on the PPU I/O latch, in dots.
///
/// Roughly 600 ms at the NTSC dot rate (~5.369 MHz). The wiki records that *at
/// least one* bit decays within 3-30 ms; full decay of every bit is much slower,
/// and 600 ms is the figure blargg's `ppu_open_bus` test is written against.
pub const DEFAULT_DECAY_DOTS: u64 = 3_221_591;

// ---------------------------------------------------------------------------
// Pixels
// ---------------------------------------------------------------------------

/// Dots between a `$2001` write and the pipeline seeing it.
///
/// "Toggling rendering takes effect approximately 3-4 dots after the write"
/// (NESdev, *PPU registers*); AccuracyCoin's OAM-corruption test measures the
/// consequence and accepts either 2 or 3 dots of delay from the *write cycle*,
/// which is this counter started at the dot the write lands on.
const MASK_WRITE_DELAY_DOTS: u8 = 3;

/// [`Engine::data_sm`]'s value the moment a `$2007` read is answered.
///
/// The engine is caught up to the dot **after** the CPU's read cycle — the 6502
/// publishes its cycle counter at the top of the cycle and latches the data bus
/// at the end of it, so the three dots the cycle occupied have all run by the
/// time the port answers. That dot is therefore the one M2 falls on, ALE is two
/// PPU cycles later and Read two after that: counting down one per dot from
/// here puts ALE at [`DATA_SM_ALE`] and the read at zero, four dots after the
/// access ends. (AccuracyCoin.asm's "PPU DATA State Machine" timing table, MIT,
/// (c) 2025 Chris Siebert.)
const DATA_SM_START: u8 = 5;

/// [`Engine::data_sm`] on the dot the state machine raises ALE.
const DATA_SM_ALE: u8 = 2;

/// Dots between the `$2006` write that loads `v` and `v` reaching the address
/// bus.
///
/// The same two PPU cycles the `$2007` state machine takes to get from M2
/// falling to its first output: the CPU's write is captured asynchronously and
/// only moves on a PPU clock edge, so `t` reaches `v` at *t2*, not during the
/// CPU's cycle. Counted down one per dot from here, so the load lands on the
/// second dot after the access — see [`Engine::v_delay`].
const ADDR_WRITE_DELAY_DOTS: u8 = 3;

/// One framebuffer entry: a palette index plus the colour-emphasis bits that
/// were in force when it was drawn.
///
/// Deliberately *not* an RGB value. The 2C02 emits an NTSC composite level, not
/// a colour, and the mapping from a 6-bit index to something a monitor shows is
/// a host-layer decision (`docs/devices/video-audio.md`). Emphasis has to
/// travel per pixel because `$2001` can change mid-scanline.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Pixel(pub u16);

impl Pixel {
    /// A pixel from a 6-bit palette index and 3 emphasis bits (`BGR`, as they
    /// sit in `$2001` bits 7-5, shifted down).
    #[inline]
    pub const fn new(index: u8, emphasis: u8) -> Pixel {
        Pixel(((emphasis as u16 & 0x07) << 6) | (index as u16 & 0x3f))
    }

    /// The palette index, 0-63.
    #[inline]
    pub const fn index(self) -> u8 {
        (self.0 & 0x3f) as u8
    }

    /// The emphasis bits, `0bBGR`.
    #[inline]
    pub const fn emphasis(self) -> u8 {
        ((self.0 >> 6) & 0x07) as u8
    }
}

// ---------------------------------------------------------------------------
// Sprite evaluation phases
// ---------------------------------------------------------------------------

/// Which step of the sprite-evaluation state machine is running.
///
/// Named after the numbered steps in
/// [NESdev PPU sprite evaluation](https://www.nesdev.org/wiki/PPU_sprite_evaluation).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvalPhase {
    /// Steps 1-2: range-check `OAM[n][0]` and copy in-range sprites.
    Copy,
    /// Step 3: the buggy ninth-sprite search that increments `n` *and* `m`.
    Overflow,
    /// Step 4: read `OAM[n][0]` and throw it away until h-blank.
    Idle,
}

impl EvalPhase {
    const fn to_bits(self) -> u8 {
        match self {
            EvalPhase::Copy => 0,
            EvalPhase::Overflow => 1,
            EvalPhase::Idle => 2,
        }
    }

    const fn from_bits(bits: u8) -> EvalPhase {
        match bits {
            0 => EvalPhase::Copy,
            1 => EvalPhase::Overflow,
            _ => EvalPhase::Idle,
        }
    }
}

// ---------------------------------------------------------------------------
// The memory cadence
// ---------------------------------------------------------------------------

/// Which half of a two-dot PPU memory access a dot is.
///
/// A 2C02 access takes **two** dots, because the chip multiplexes the low eight
/// address bits onto the same pins it reads data back on: the first dot drives
/// the address and strobes ALE, which opens an octal latch *on the board* and
/// holds those eight bits; the second dot performs the read, composing the
/// address from the six address-only pins as they stand on that dot and the
/// eight bits the latch is holding. That split is directly observable — see
/// [`Engine::ale_latch`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Drive the address, strobe ALE.
    Ale,
    /// Read, from the top six pins plus the octal latch.
    Read,
}

/// What the render pipeline is fetching on a given pair of dots.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FetchOp {
    /// The background nametable byte.
    Nt,
    /// The attribute byte for the tile being fetched.
    At,
    /// The background pattern's low bitplane.
    BgLo,
    /// The background pattern's high bitplane.
    BgHi,
    /// One of the two garbage nametable fetches inside a sprite slot.
    ///
    /// The sprite fetches do **not** read the attribute table: the slot is two
    /// nametable fetches followed by the two pattern planes ("Sprite fetch
    /// should not be performing attribute table fetches, rather it should
    /// perform two nametable fetches in a row" — AccuracyCoin.asm, MIT, (c)
    /// 2025 Chris Siebert).
    SpNt,
    /// A sprite pattern's low bitplane, for slot `.0`.
    SpLo(u8),
    /// A sprite pattern's high bitplane, for slot `.0`.
    SpHi(u8),
}

// ---------------------------------------------------------------------------
// The engine
// ---------------------------------------------------------------------------

/// Everything the picture unit contains, and the per-dot pipeline that drives
/// it.
///
/// This is the whole architectural state: nothing in here is a derived cache,
/// which is why all of it is snapshotted. The background shift registers look
/// like a cache and are not — a snapshot taken at dot 100 of a visible scanline
/// has to resume drawing the same pixels.
pub struct Engine {
    // -- position -----------------------------------------------------------
    /// Total dots executed since the last cold reset. The lazily-advanced
    /// device's clock (`ROADMAP.md` §4.2).
    pub(crate) dots: u64,
    /// Frames completed. Its parity chooses the odd-frame skip.
    pub(crate) frame: u64,
    /// The scanline about to be executed, 0 to
    /// [`Geometry::pre_render_scanline`].
    pub(crate) scanline: u16,
    /// The dot about to be executed, 0-340.
    pub(crate) dot: u16,

    // -- registers ----------------------------------------------------------
    pub(crate) ctrl: u8,
    pub(crate) mask: u8,
    pub(crate) status: u8,
    pub(crate) oam_addr: u8,
    /// Current VRAM address, 15 bits.
    pub(crate) v: u16,
    /// Temporary VRAM address / the scroll latch, 15 bits.
    pub(crate) t: u16,
    /// Fine X scroll, 3 bits.
    pub(crate) x: u8,
    /// The `$2005`/`$2006` write toggle.
    pub(crate) w: bool,
    /// The one-stage `$2007` read delay.
    pub(crate) read_buffer: u8,

    // -- buses --------------------------------------------------------------
    /// The CPU-facing I/O latch: open bus.
    pub(crate) latch: IoLatch,
    /// Last value seen on the PPU's own address bus, returned when a CHR fetch
    /// faults. The PPU bus floats too; this is its equivalent of open bus.
    pub(crate) bus_latch: u8,
    /// The **octal address latch**: the low eight bits of the PPU's address
    /// bus, held outside the chip.
    ///
    /// The 2C02 has fourteen address bits and eight data pins, and the low
    /// eight of the address share those eight pins. An access is therefore two
    /// dots: on the first, ALE is strobed and the low eight bits are captured
    /// into a 74-series octal latch on the cartridge board; on the second, the
    /// chip drives only the top six and reads, so the address that actually
    /// reaches memory is *the top six as they stand on the read dot* composed
    /// with *the eight bits the latch captured on the ALE dot*.
    ///
    /// Keeping the two halves apart is the entire subject of AccuracyCoin's
    /// "ALE + Read" and "Hybrid Addresses" tests: a `$2006` write landing
    /// between the two dots changes the top six and not the low eight, and the
    /// read goes to an address the chip never emitted.
    pub(crate) ale_latch: u8,
    /// `$2007`'s access state machine: dots left before its next phase, or 0.
    ///
    /// A `$2007` read does not fetch during the CPU's cycle. The read only
    /// *starts* a five-stage latch chain clocked off the PPU clock, which
    /// raises ALE two PPU cycles after M2 falls and Read two cycles after that
    /// — so the read buffer is filled four PPU cycles after the CPU access
    /// ends, in the middle of whatever the render pipeline is doing
    /// (AccuracyCoin.asm's "PPU DATA State Machine", MIT, (c) 2025 Chris
    /// Siebert). Counted down one per dot from [`DATA_SM_START`].
    pub(crate) data_sm: u8,
    /// The address the pending `$2007` access captured.
    pub(crate) data_sm_addr: u16,
    /// The `v` a second `$2006` write has loaded but not yet delivered.
    pub(crate) v_pending: u16,
    /// Dots left before [`Engine::v_pending`] becomes `v`.
    ///
    /// AccuracyCoin's "Hybrid Addresses" is the whole reason this is not
    /// immediate: it lands a `$2006` write between the two dots of a nametable
    /// fetch, so the top six address bits come from the *new* `v` and the low
    /// eight from the octal latch the *old* one strobed. That is only possible
    /// if `v` arrives on a dot boundary of the PPU's own clock rather than at
    /// the end of the CPU's write cycle. See [`ADDR_WRITE_DELAY_DOTS`].
    pub(crate) v_delay: u8,

    // -- memories -----------------------------------------------------------
    pub(crate) oam: [u8; 256],
    pub(crate) secondary_oam: [u8; 32],
    /// 32 bytes of 6-bit entries. Palette RAM is inside the PPU, not on the
    /// PPU bus, which is why a palette read is not buffered.
    pub(crate) palette: [u8; 32],

    // -- background pipeline ------------------------------------------------
    pub(crate) nt_latch: u8,
    /// The two attribute bits selected for the tile being fetched.
    pub(crate) at_latch: u8,
    pub(crate) bg_lo_latch: u8,
    pub(crate) bg_hi_latch: u8,
    pub(crate) bg_shift_lo: u16,
    pub(crate) bg_shift_hi: u16,
    pub(crate) at_shift_lo: u16,
    pub(crate) at_shift_hi: u16,

    // -- sprite evaluation (for the *next* scanline) ------------------------
    pub(crate) eval_phase: EvalPhase,
    /// Sprite index being examined, 0-63.
    pub(crate) eval_n: u8,
    /// Byte within that sprite, 0-3. The overflow bug lives in how this moves.
    pub(crate) eval_m: u8,
    /// Write cursor into secondary OAM.
    pub(crate) eval_sec: u8,
    /// Sprites copied so far, 0-8.
    pub(crate) eval_found: u8,
    /// The OAMADDR evaluation started from, so a misaligned OAMADDR reinterprets
    /// bytes as Y coordinates exactly as hardware does.
    pub(crate) eval_base: u8,
    /// The byte read on the odd dot, written on the even dot.
    ///
    /// Also what a `$2004` read sees during evaluation: the read line the
    /// sprite unit is driving is the primary-OAM read bus, and it holds its
    /// value across the odd/even pair.
    pub(crate) eval_latch: u8,
    /// Whether sprite 0 was among the sprites copied for the next scanline.
    pub(crate) sprite_zero_next: bool,
    /// Entries still to be read after an overflow hit, unconditionally.
    ///
    /// Step 3a of sprite evaluation: an in-range byte found with secondary OAM
    /// already full sets the overflow flag and then "reads the next 3 entries
    /// of OAM" — *without* range-checking them — after which the unit drops
    /// into step 4 ([NESdev PPU sprite evaluation]).
    pub(crate) eval_copy_left: u8,
    /// Whether the even dot just gone wrote secondary OAM rather than read it.
    ///
    /// The distinction is guest-visible: the OAM read line carries the byte
    /// being written on a write dot, and whatever secondary OAM answers with on
    /// a read dot. Recorded rather than derived because the phase can change on
    /// the very dot in question — the eighth sprite's last byte is written by a
    /// dot that leaves the unit in step 3.
    pub(crate) eval_wrote: bool,

    // -- sprite output registers (for the scanline being drawn) -------------
    pub(crate) sprite_pat_lo: [u8; 8],
    pub(crate) sprite_pat_hi: [u8; 8],
    pub(crate) sprite_attr: [u8; 8],
    /// The per-slot X **counter**, not the X coordinate.
    ///
    /// Loaded with the sprite's X during the fetch slots and counted down once
    /// per dot of the visible scanline while rendering is on. A slot whose
    /// counter has reached zero is outputting: its pattern registers shift one
    /// bit per dot and bit 7 is the pixel.
    ///
    /// Modelled as the counter it is rather than as a comparison against the
    /// pixel's column, because the two only agree while rendering stays on. A
    /// slot that reached zero and then had rendering taken away from it keeps
    /// its half-shifted pattern and finishes drawing when rendering returns,
    /// which is what AccuracyCoin's two "stale shift register" tests measure.
    pub(crate) sprite_x: [u8; 8],
    /// Which output units have stopped counting and started shifting.
    ///
    /// One bit per slot. Set when a slot's X counter reaches zero, and cleared
    /// — every slot at once — on **dot 339 of a rendered scanline**. That is
    /// the whole of the rule: "if the PPU is rendering on dot 339, then the
    /// shifter counters are set to counting; if rendering was not enabled on
    /// dot 339, the shifter counters will be in whatever state they were
    /// previously in, which is likely halted" (AccuracyCoin.asm, MIT, © 2025
    /// Chris Siebert). A frame that takes rendering away before dot 339 comes
    /// back with every unit already halted, which draws every sprite as though
    /// its X were zero.
    /// The secondary-OAM address the sprite unit is standing on.
    ///
    /// Not the same thing as the evaluation write cursor: it follows whichever
    /// phase of the scanline is using secondary OAM, and it is what seeds the
    /// OAM corruption when rendering is switched off mid-line.
    pub(crate) sec_addr: u8,
    /// A row of OAM waiting to be corrupted, and which row.
    ///
    /// Set when rendering is taken away mid-line; consumed on the first dot
    /// with rendering back on. See [`Engine::corrupt_oam`].
    pub(crate) corrupt_row: Option<u8>,
    pub(crate) sprite_halted: u8,
    pub(crate) sprite_active: u8,
    pub(crate) sprite_zero_active: bool,
    /// Latches held between the four secondary-OAM reads and the two pattern
    /// fetches of one sprite's 8-dot slot.
    pub(crate) sp_y_latch: u8,
    pub(crate) sp_tile_latch: u8,
    pub(crate) sp_attr_latch: u8,

    // -- flag pipelines -----------------------------------------------------
    /// Sprite 0 hit detected on the previous dot. The flag lags the pixel by one
    /// dot: "sprite 0 hit acts as if the image starts at cycle 2"
    /// (NESdev PPU rendering).
    pub(crate) sprite0_pending: bool,
    /// `$2001` as written, waiting out [`Engine::mask_delay`].
    pub(crate) mask_pending: u8,
    /// Dots left before a `$2001` write takes effect.
    ///
    /// "Toggling rendering takes effect approximately 3-4 dots after the
    /// write" (NESdev, *PPU registers*). It is not cosmetic: three of
    /// AccuracyCoin's tests switch rendering off at a named dot and measure
    /// what the pipeline was in the middle of, and the delay is the difference
    /// between the answer and the one before it.
    pub(crate) mask_delay: u8,
    /// The dot the vblank flag was last set on.
    pub(crate) vblank_set_dot: u64,
    /// A `$2002` read landed one dot before the vblank flag would be set, so it
    /// is not set at all this frame (NESdev PPU frame timing).
    pub(crate) suppress_vblank_set: bool,
    /// A `$2002` read landed on or just after the set, so `/NMI` never drops
    /// for long enough this frame.
    pub(crate) suppress_nmi: bool,
    /// The `/NMI` level as it stood **two** dots ago — what the CPU samples.
    /// See [`Engine::nmi_active`].
    pub(crate) nmi_out: bool,
    /// The first stage of that two-dot pipeline: the level one dot ago.
    pub(crate) nmi_delay: bool,
    /// Whether the pipeline ran on the previous dot.
    ///
    /// The shift registers' load pulse is a registered signal: it reloads on
    /// dot 9, 17, ... only if the pipeline was running on the dot before, which
    /// is the one that finished fetching the tile being loaded. Rendering
    /// switched back on *at* a reload dot therefore misses that reload, and the
    /// register keeps shifting its serial input in for another eight dots —
    /// which is the whole of AccuracyCoin's "BG Serial In".
    pub(crate) rendered_last: bool,

    // -- configuration ------------------------------------------------------
    /// Which console this is. Machine configuration, not architectural state,
    /// so it is not serialized: a snapshot never changes region.
    pub(crate) region: Region,
    /// [`Region::geometry`] of `region`, resolved once. Derived, never saved.
    pub(crate) geom: Geometry,
    /// Honour the ~29658-CPU-cycle write lockout after reset.
    pub(crate) warmup: bool,

    // -- output -------------------------------------------------------------
    pub(crate) fb: Box<[Pixel]>,

    // -- links --------------------------------------------------------------
    /// The PPU's own address space, `$0000`-`$3FFF`: pattern tables and
    /// nametables, both of which live on the cartridge.
    pub(crate) bus: Option<Arc<AddressSpace>>,
}

impl fmt::Debug for Engine {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // The framebuffer and the three memories are 61 kB of noise in a
        // backtrace; the position and the registers are what anyone wants.
        f.debug_struct("Engine")
            .field("region", &self.region)
            .field("dots", &self.dots)
            .field("frame", &self.frame)
            .field("scanline", &self.scanline)
            .field("dot", &self.dot)
            .field("ctrl", &self.ctrl)
            .field("mask", &self.mask)
            .field("status", &self.status)
            .field("v", &self.v)
            .field("t", &self.t)
            .field("x", &self.x)
            .field("w", &self.w)
            .finish_non_exhaustive()
    }
}

impl Engine {
    /// A powered-off engine. [`Engine::reset_cold`] gives it its reset state.
    pub fn new(region: Region, warmup: bool, decay_dots: u64) -> Engine {
        let mut engine = Engine {
            dots: 0,
            frame: 0,
            scanline: 0,
            dot: 0,
            ctrl: 0,
            mask: 0,
            status: 0,
            oam_addr: 0,
            v: 0,
            t: 0,
            x: 0,
            w: false,
            read_buffer: 0,
            latch: IoLatch::new(decay_dots),
            bus_latch: 0,
            ale_latch: 0,
            data_sm: 0,
            data_sm_addr: 0,
            v_pending: 0,
            v_delay: 0,
            oam: [0; 256],
            secondary_oam: [0xff; 32],
            palette: [0; 32],
            nt_latch: 0,
            at_latch: 0,
            bg_lo_latch: 0,
            bg_hi_latch: 0,
            bg_shift_lo: 0,
            bg_shift_hi: 0,
            at_shift_lo: 0,
            at_shift_hi: 0,
            eval_phase: EvalPhase::Copy,
            eval_n: 0,
            eval_m: 0,
            eval_sec: 0,
            eval_found: 0,
            eval_base: 0,
            eval_latch: 0,
            sprite_zero_next: false,
            eval_copy_left: 0,
            eval_wrote: false,
            sprite_pat_lo: [0; 8],
            sprite_pat_hi: [0; 8],
            sprite_attr: [0; 8],
            sprite_x: [0; 8],
            sec_addr: 0,
            corrupt_row: None,
            sprite_halted: 0,
            sprite_active: 0,
            sprite_zero_active: false,
            sp_y_latch: 0,
            sp_tile_latch: 0,
            sp_attr_latch: 0,
            mask_pending: 0,
            mask_delay: 0,
            sprite0_pending: false,
            vblank_set_dot: 0,
            suppress_vblank_set: false,
            suppress_nmi: false,
            nmi_out: false,
            nmi_delay: false,
            rendered_last: false,
            region,
            geom: region.geometry(),
            warmup,
            fb: vec![Pixel::default(); FRAMEBUFFER_LEN].into_boxed_slice(),
            bus: None,
        };
        engine.reset_cold();
        engine
    }

    /// Which console this engine is modelling.
    #[inline]
    pub const fn tv_region(&self) -> Region {
        self.region
    }

    /// The frame geometry [`Engine::tv_region`] implies.
    #[inline]
    pub const fn geometry(&self) -> Geometry {
        self.geom
    }

    /// Power-on state.
    ///
    /// OAM is *not* cleared: it is uninitialised DRAM on real hardware, and a
    /// game that reads it before writing it is reading garbage either way. What
    /// matters for determinism is that it is the *same* garbage every run, so it
    /// is zeroed once here and left alone by a warm reset.
    pub fn reset_cold(&mut self) {
        let warmup = self.warmup;
        let ttl = self.latch.ttl();
        self.dots = 0;
        self.frame = 0;
        self.scanline = 0;
        self.dot = 0;
        self.ctrl = 0;
        self.mask = 0;
        self.status = 0;
        self.oam_addr = 0;
        self.v = 0;
        self.t = 0;
        self.x = 0;
        self.w = false;
        self.read_buffer = 0;
        self.latch = IoLatch::new(ttl);
        self.bus_latch = 0;
        self.ale_latch = 0;
        self.data_sm = 0;
        self.data_sm_addr = 0;
        self.oam = [0; 256];
        self.secondary_oam = [0xff; 32];
        self.palette = [0; 32];
        self.warmup = warmup;
        self.reset_pipelines();
        self.fb.fill(Pixel::default());
    }

    /// A reset-line pulse: the same as a cold reset except that OAM, palette RAM
    /// and the frame buffer keep their contents, because nothing discharges
    /// them.
    pub fn reset_warm(&mut self) {
        self.dots = 0;
        self.frame = 0;
        self.scanline = 0;
        self.dot = 0;
        self.ctrl = 0;
        self.mask = 0;
        self.status = 0;
        self.v = 0;
        self.t = 0;
        self.x = 0;
        self.w = false;
        self.read_buffer = 0;
        self.reset_pipelines();
    }

    fn reset_pipelines(&mut self) {
        self.data_sm = 0;
        self.data_sm_addr = 0;
        self.v_pending = 0;
        self.v_delay = 0;
        self.nt_latch = 0;
        self.at_latch = 0;
        self.bg_lo_latch = 0;
        self.bg_hi_latch = 0;
        self.bg_shift_lo = 0;
        self.bg_shift_hi = 0;
        self.at_shift_lo = 0;
        self.at_shift_hi = 0;
        self.eval_phase = EvalPhase::Copy;
        self.eval_n = 0;
        self.eval_m = 0;
        self.eval_sec = 0;
        self.eval_found = 0;
        self.eval_base = 0;
        self.eval_latch = 0;
        self.sprite_zero_next = false;
        self.eval_copy_left = 0;
        self.eval_wrote = false;
        self.sprite_pat_lo = [0; 8];
        self.sprite_pat_hi = [0; 8];
        self.sprite_attr = [0; 8];
        self.sprite_x = [0; 8];
        self.sprite_active = 0;
        self.sprite_zero_active = false;
        self.sp_y_latch = 0;
        self.sp_tile_latch = 0;
        self.sp_attr_latch = 0;
        self.sprite0_pending = false;
        self.vblank_set_dot = 0;
        self.suppress_vblank_set = false;
        self.suppress_nmi = false;
        self.nmi_delay = false;
        self.rendered_last = false;
    }

    // -- helpers ------------------------------------------------------------

    /// Whether either rendering enable in `$2001` is set.
    #[inline]
    pub fn rendering_enabled(&self) -> bool {
        self.mask & MASK_RENDERING != 0
    }

    /// Whether the dot about to run is on a line the render pipeline is active
    /// on: the 240 visible lines plus the pre-render line.
    #[inline]
    fn render_line(&self) -> bool {
        self.scanline < self.geom.visible_scanlines
            || self.scanline == self.geom.pre_render_scanline
    }

    /// The level the `/NMI` request is at *right now*, before the output delay.
    ///
    /// `vblank_flag AND nmi_output`, with the frame's suppression applied
    /// ([NESdev NMI](https://www.nesdev.org/wiki/NMI)). Expressed as an
    /// active-high request rather than the chip's active-low pin, because
    /// [`crate::core::wire`] nets idle low and an inverter is a device
    /// (`ROADMAP.md` §4.3) if a machine wants the pin polarity.
    #[inline]
    fn nmi_raw(&self) -> bool {
        self.status & STATUS_VBLANK != 0 && self.ctrl & CTRL_NMI != 0 && !self.suppress_nmi
    }

    /// The level the CPU sees on `/NMI` — **two** dots behind the request.
    ///
    /// # Why two dots
    ///
    /// A 6502 samples `/NMI` on the φ1→φ2 boundary, one dot into its own
    /// three-dot cycle, and completes its bus access at the very end of that
    /// cycle. The scheduler catches this chip up to the dot *after* the cycle
    /// before the core looks, so the level the core must be shown is the one
    /// from two dots back: the end of the cycle's first dot. A `$2002` read,
    /// which happens after the sample, therefore cannot unmake a request the
    /// CPU has already seen — but one made in the same cycle still can.
    ///
    /// The whole of AccuracyCoin's vblank page measures this, and the two dots
    /// are what makes *all* of it agree at once. "NMI Suppression" fixes which
    /// side of the sample a `$2002` read falls on; "NMI at VBlank end" fixes
    /// the other edge, sweeping a `$2000` write across the dot the flag clears
    /// on and requiring an NMI from the write that lands on the dot before it —
    /// which a one-dot output cannot produce, because by the CPU's next sample
    /// the pin is back up. Publishing the level as it stood two dots ago, and
    /// sampling it at the cycle boundary, is that statement written down once.
    #[inline]
    pub fn nmi_active(&self) -> bool {
        self.nmi_out
    }

    /// Sprite height from `$2000` bit 5.
    #[inline]
    fn sprite_height(&self) -> u8 {
        if self.ctrl & CTRL_SPRITE_16 != 0 {
            16
        } else {
            8
        }
    }

    /// Whether `$2000`/`$2001`/`$2005`/`$2006` writes are accepted yet.
    #[inline]
    fn warm(&self) -> bool {
        !self.warmup || self.dots >= self.geom.warmup_dots
    }

    /// Read one byte from the PPU bus, or the bus latch if nothing answers.
    fn bus_read(&mut self, addr: u16, attrs: MemAttrs) -> u8 {
        let addr = u64::from(addr & 0x3fff);
        let fallback = self.bus_latch;
        let value = match self.bus.as_ref() {
            Some(space) => space
                .read(addr, Width::U8, attrs)
                .map_or(fallback, |v| v as u8),
            None => fallback,
        };
        if !attrs.debug {
            self.bus_latch = value;
        }
        value
    }

    fn bus_write(&mut self, addr: u16, value: u8, attrs: MemAttrs) {
        let addr = u64::from(addr & 0x3fff);
        if let Some(space) = self.bus.as_ref() {
            // A write into unmapped CHR space is normal on a cartridge with
            // CHR ROM: it is ignored, not a machine error.
            let _ = space.write(addr, Width::U8, u64::from(value), attrs);
        }
        if !attrs.debug {
            self.bus_latch = value;
        }
    }

    // -- palette ------------------------------------------------------------

    /// Fold a palette address onto its 32-byte storage.
    ///
    /// `$3F10`, `$3F14`, `$3F18` and `$3F1C` are not separate entries: they are
    /// the same storage as `$3F00`, `$3F04`, `$3F08` and `$3F0C`, so writing
    /// either updates both ([NESdev PPU palettes](https://www.nesdev.org/wiki/PPU_palettes)).
    /// The whole 32 bytes then repeat through `$3F00`-`$3FFF`.
    #[inline]
    pub const fn palette_index(addr: u16) -> usize {
        let a = (addr & 0x1f) as u8;
        // The four sprite entries whose low two bits are zero alias the
        // background ones; every other address stands alone.
        if a & 0x13 == 0x10 {
            (a & 0x0f) as usize
        } else {
            a as usize
        }
    }

    /// One palette entry, 6 bits.
    #[inline]
    pub fn palette_read(&self, addr: u16) -> u8 {
        self.palette[Self::palette_index(addr)] & 0x3f
    }

    /// Write one palette entry. Only 6 bits exist.
    #[inline]
    pub fn palette_write(&mut self, addr: u16, value: u8) {
        self.palette[Self::palette_index(addr)] = value & 0x3f;
    }

    // -- scroll arithmetic (NESdev PPU scrolling) ---------------------------

    fn increment_coarse_x(&mut self) {
        if self.v & 0x001f == 31 {
            self.v &= !0x001f;
            // Crossing a nametable horizontally toggles bit 10.
            self.v ^= 0x0400;
        } else {
            self.v += 1;
        }
    }

    fn increment_y(&mut self) {
        if self.v & 0x7000 != 0x7000 {
            self.v += 0x1000;
        } else {
            self.v &= !0x7000;
            let mut coarse_y = (self.v & 0x03e0) >> 5;
            if coarse_y == 29 {
                // Row 29 is the last tile row; 30 and 31 are attribute data, so
                // hardware wraps here and toggles the vertical nametable.
                coarse_y = 0;
                self.v ^= 0x0800;
            } else if coarse_y == 31 {
                // Reached only if software scrolled into the attribute rows.
                coarse_y = 0;
            } else {
                coarse_y += 1;
            }
            self.v = (self.v & !0x03e0) | (coarse_y << 5);
        }
    }

    fn copy_horizontal(&mut self) {
        self.v = (self.v & !0x041f) | (self.t & 0x041f);
    }

    fn copy_vertical(&mut self) {
        self.v = (self.v & !0x7be0) | (self.t & 0x7be0);
    }

    // -- the two-dot memory cadence -----------------------------------------

    /// The ALE half of an access: drive `addr` and strobe the octal latch.
    ///
    /// Only the low eight bits are captured. The top six are driven afresh on
    /// every dot and are *not* state — which is exactly why they can come from
    /// a different address than the low eight. See [`Engine::ale_latch`].
    fn ale(&mut self, addr: u16) {
        self.ale_latch = addr as u8;
    }

    /// The read half: top six bits from `addr` as it stands *now*, low eight
    /// from the octal latch.
    fn read_dot(&mut self, addr: u16) -> u8 {
        let full = (addr & 0x3f00) | u16::from(self.ale_latch);
        self.bus_read(full, MemAttrs::DEFAULT)
    }

    /// Which access the render pipeline is performing on `dot`, and which half.
    ///
    /// Every dot of a rendered scanline is one or the other — "every single PPU
    /// cycle is either setting up the address + latch, or reading from memory"
    /// (AccuracyCoin.asm, MIT, (c) 2025 Chris Siebert). Dots 1-256 and 321-336
    /// are the background's four fetches; 257-320 are eight sprite slots, each
    /// two *nametable* fetches and two pattern fetches; 337-340 is a further
    /// pair of nametable fetches; dot 0 re-drives the pattern address register.
    fn cadence_op(&self, dot: u16) -> Option<(Phase, FetchOp)> {
        match dot {
            0 => Some((Phase::Ale, FetchOp::BgHi)),
            1..=256 | 321..=336 => Some(match dot % 8 {
                1 => (Phase::Ale, FetchOp::Nt),
                2 => (Phase::Read, FetchOp::Nt),
                3 => (Phase::Ale, FetchOp::At),
                4 => (Phase::Read, FetchOp::At),
                5 => (Phase::Ale, FetchOp::BgLo),
                6 => (Phase::Read, FetchOp::BgLo),
                7 => (Phase::Ale, FetchOp::BgHi),
                _ => (Phase::Read, FetchOp::BgHi),
            }),
            257..=320 => {
                let slot = ((dot - 257) / 8) as u8;
                Some(match (dot - 257) % 8 {
                    0 => (Phase::Ale, FetchOp::SpNt),
                    1 => (Phase::Read, FetchOp::SpNt),
                    2 => (Phase::Ale, FetchOp::SpNt),
                    3 => (Phase::Read, FetchOp::SpNt),
                    4 => (Phase::Ale, FetchOp::SpLo(slot)),
                    5 => (Phase::Read, FetchOp::SpLo(slot)),
                    6 => (Phase::Ale, FetchOp::SpHi(slot)),
                    _ => (Phase::Read, FetchOp::SpHi(slot)),
                })
            }
            337 | 339 => Some((Phase::Ale, FetchOp::Nt)),
            338 | 340 => Some((Phase::Read, FetchOp::Nt)),
            _ => None,
        }
    }

    /// The address the pipeline wants for `op`, evaluated against `v` *now*.
    fn cadence_addr(&self, op: FetchOp, scanline: u16) -> u16 {
        match op {
            FetchOp::Nt | FetchOp::SpNt => 0x2000 | (self.v & 0x0fff),
            FetchOp::At => {
                0x23c0 | (self.v & 0x0c00) | ((self.v >> 4) & 0x38) | ((self.v >> 2) & 0x07)
            }
            FetchOp::BgLo => self.pattern_addr(false),
            FetchOp::BgHi => self.pattern_addr(true),
            FetchOp::SpLo(_) => self.sprite_pattern_addr(scanline, false),
            FetchOp::SpHi(_) => self.sprite_pattern_addr(scanline, true),
        }
    }

    /// File a byte the read half just produced into the latch that wanted it.
    fn cadence_store(&mut self, op: FetchOp, byte: u8) {
        match op {
            FetchOp::Nt | FetchOp::SpNt => self.nt_latch = byte,
            FetchOp::At => {
                // Bit 1 of coarse Y picks the row of the 2x2 quadrant grid, bit
                // 1 of coarse X the column.
                let shift = ((self.v >> 4) & 4) | (self.v & 2);
                self.at_latch = (byte >> shift) & 3;
            }
            FetchOp::BgLo => self.bg_lo_latch = byte,
            FetchOp::BgHi => self.bg_hi_latch = byte,
            FetchOp::SpLo(slot) => {
                let byte = if self.sp_attr_latch & SPRITE_FLIP_X != 0 {
                    byte.reverse_bits()
                } else {
                    byte
                };
                self.sprite_pat_lo[usize::from(slot)] = byte;
            }
            FetchOp::SpHi(slot) => {
                let byte = if self.sp_attr_latch & SPRITE_FLIP_X != 0 {
                    byte.reverse_bits()
                } else {
                    byte
                };
                self.sprite_pat_hi[usize::from(slot)] = byte;
            }
        }
    }

    /// Step `$2007`'s state machine by one dot and say what it is doing.
    fn data_sm_dot(&mut self) -> Option<Phase> {
        if self.data_sm == 0 {
            return None;
        }
        self.data_sm -= 1;
        match self.data_sm {
            DATA_SM_ALE => Some(Phase::Ale),
            0 => Some(Phase::Read),
            _ => None,
        }
    }

    /// One dot of the PPU's memory bus: the render cadence and `$2007`'s state
    /// machine, which contend for one set of pins.
    ///
    /// `cadence` is `None` when nothing is rendering, which is the ordinary
    /// case for a `$2007` read and the only one where the state machine gets
    /// the bus to itself. When both want it the arbitration is the ROM's, and
    /// every branch below cites which line of its commentary it comes from
    /// (AccuracyCoin.asm, MIT, (c) 2025 Chris Siebert).
    fn memory_dot(&mut self, cadence: Option<(Phase, FetchOp)>, scanline: u16) {
        let sm = self.data_sm_dot();
        match (cadence, sm) {
            (None, None) => {}
            // The pipeline alone.
            (Some((Phase::Ale, op)), None) => {
                let addr = self.cadence_addr(op, scanline);
                self.ale(addr);
            }
            (Some((Phase::Read, op)), None) => {
                let addr = self.cadence_addr(op, scanline);
                let byte = self.read_dot(addr);
                self.cadence_store(op, byte);
            }
            // The state machine alone.
            (None, Some(Phase::Ale)) => {
                let addr = self.data_sm_addr;
                self.ale(addr);
            }
            (None, Some(Phase::Read)) => {
                let addr = self.data_sm_addr;
                self.read_buffer = self.read_dot(addr);
            }
            // Both want ALE: "the background fetch takes priority, so the
            // address bus + latch are set up using the address for the
            // nametable read".
            (Some((Phase::Ale, op)), Some(Phase::Ale)) => {
                let addr = self.cadence_addr(op, scanline);
                self.ale(addr);
            }
            // ALE and Read together: the octal latch is transparent while the
            // pins are carrying data, so it does *not* take the address the
            // pipeline is driving — "notably, the octal latch is still $FF ...
            // I'm not sure why it wasn't updated to $03, but that's how it
            // appears to work out when both ALE and Read are set". The read
            // therefore goes to the pipeline's top six bits over the *stale*
            // low eight, and the byte that comes back is what the latch
            // settles on.
            (Some((Phase::Ale, op)), Some(Phase::Read)) => {
                let addr = self.cadence_addr(op, scanline);
                let byte = self.read_dot(addr);
                self.read_buffer = byte;
                self.ale_latch = byte;
            }
            // The pipeline reads while the state machine strobes ALE. The read
            // is one of the ones the ROM marks unstable and does not check; the
            // latch it leaves behind is the state machine's.
            (Some((Phase::Read, op)), Some(Phase::Ale)) => {
                let addr = self.cadence_addr(op, scanline);
                let byte = self.read_dot(addr);
                self.cadence_store(op, byte);
                let addr = self.data_sm_addr;
                self.ale(addr);
            }
            // "The Read line is synced between the read cadence and the state
            // machine, so the value read from the attribute table will also go
            // into the Read Buffer." One read, two destinations.
            (Some((Phase::Read, op)), Some(Phase::Read)) => {
                let addr = self.cadence_addr(op, scanline);
                let byte = self.read_dot(addr);
                self.cadence_store(op, byte);
                self.read_buffer = byte;
            }
        }
        // `v` moves at the *end* of the access, not during the CPU's cycle.
        // AccuracyCoin's "$2007 Stress Test" pins this down: its answer key
        // sweeps one `$2007` read across a whole scanline and reads back a
        // clean run of nametable bytes, which is only possible if the read the
        // state machine performs happens *before* the increment its own access
        // causes — otherwise every sample past the first would be a tile
        // further along than the key says.
        if sm == Some(Phase::Read) {
            self.increment_data_address();
        }
    }

    fn pattern_addr(&self, high: bool) -> u16 {
        let base = if self.ctrl & CTRL_BG_TABLE != 0 {
            0x1000
        } else {
            0x0000
        };
        let fine_y = (self.v >> 12) & 7;
        base | (u16::from(self.nt_latch) << 4) | fine_y | if high { 8 } else { 0 }
    }

    fn shift_background(&mut self) {
        // Not zeros: "shift registers each shift in a constant: logically 1 for
        // the high bitplane, 0 for the low bitplane" (NESdev, *PPU rendering*).
        // Shifted often enough without a reload — which is what switching
        // rendering off across the reload dot does — the high plane fills with
        // ones and a transparent nametable starts drawing pixel `%10`.
        self.bg_shift_lo <<= 1;
        self.bg_shift_hi = (self.bg_shift_hi << 1) | 1;
        self.at_shift_lo <<= 1;
        self.at_shift_hi <<= 1;
    }

    /// Load the just-fetched tile into the low half of the shift registers.
    ///
    /// The attribute bits are held in a latch that feeds an 8-bit shifter; a
    /// 16-bit shifter filled with a constant nibble is the same thing observed
    /// from the multiplexer, and it keeps one indexing rule for all four.
    fn reload_shifters(&mut self) {
        self.bg_shift_lo = (self.bg_shift_lo & 0xff00) | u16::from(self.bg_lo_latch);
        self.bg_shift_hi = (self.bg_shift_hi & 0xff00) | u16::from(self.bg_hi_latch);
        self.at_shift_lo =
            (self.at_shift_lo & 0xff00) | if self.at_latch & 1 != 0 { 0x00ff } else { 0 };
        self.at_shift_hi =
            (self.at_shift_hi & 0xff00) | if self.at_latch & 2 != 0 { 0x00ff } else { 0 };
    }

    // -- sprite evaluation --------------------------------------------------

    /// Is a sprite whose Y byte is `y` visible on the line evaluation is running
    /// for?
    ///
    /// Evaluation on scanline `n` fills secondary OAM for scanline `n + 1`, and
    /// sprites are drawn one line below their Y byte, so the two offsets cancel
    /// and the comparison is against the current scanline.
    #[inline]
    fn sprite_in_range(&self, y: u8, scanline: u16) -> bool {
        let delta = scanline.wrapping_sub(u16::from(y));
        delta < u16::from(self.sprite_height())
    }

    /// Dots 1-64: secondary OAM is cleared to `$FF`, one byte every two dots.
    fn secondary_clear_dot(&mut self, dot: u16) {
        if dot.is_multiple_of(2) {
            self.secondary_oam[(dot / 2 - 1) as usize] = 0xff;
        }
        if dot == 64 {
            self.eval_phase = EvalPhase::Copy;
            self.eval_n = 0;
            self.eval_m = 0;
            self.eval_sec = 0;
            self.eval_found = 0;
            self.eval_base = self.oam_addr;
            self.sprite_zero_next = false;
        }
    }

    /// The OAM byte evaluation is looking at, from the base OAMADDR it started
    /// with. A base that is not a multiple of four is what makes a misaligned
    /// OAMADDR reinterpret tile and attribute bytes as Y coordinates.
    #[inline]
    fn eval_oam_index(&self) -> usize {
        usize::from(
            self.eval_base
                .wrapping_add(self.eval_n.wrapping_mul(4))
                .wrapping_add(self.eval_m),
        )
    }

    /// Dots 65-256: odd dots read primary OAM, even dots act on what was read.
    fn sprite_eval_dot(&mut self, dot: u16, scanline: u16) {
        if !dot.is_multiple_of(2) {
            self.eval_latch = self.oam[self.eval_oam_index()];
            return;
        }
        // Only step 2 writes; steps 3 and 4 read secondary OAM instead, and
        // that read is what `$2004` sees on this dot.
        self.eval_wrote = self.eval_phase == EvalPhase::Copy;
        match self.eval_phase {
            EvalPhase::Copy => self.eval_copy(scanline),
            EvalPhase::Overflow => self.eval_overflow(scanline),
            EvalPhase::Idle => {
                // Step 4: the read happened, the write does not land.
                self.eval_n = self.eval_n.wrapping_add(1) & 63;
            }
        }
    }

    /// Step 4: read `OAM[n][0]` and throw it away.
    ///
    /// `m` is cleared on the way in, because step 4 reads the *`Y` byte* of
    /// each sprite however the previous step left the pointer — which is how
    /// AccuracyCoin's "$2004 Stress Test" sees the address step by four rather
    /// than continuing the diagonal it was on.
    fn enter_eval_idle(&mut self) {
        self.eval_phase = EvalPhase::Idle;
        self.eval_m = 0;
    }

    fn eval_copy(&mut self, scanline: u16) {
        let latch = self.eval_latch;
        if self.eval_found < 8 {
            self.secondary_oam[usize::from(self.eval_sec) & 31] = latch;
        }
        if self.eval_m == 0 {
            if self.sprite_in_range(latch, scanline) {
                if self.eval_n == 0 {
                    self.sprite_zero_next = true;
                }
                self.eval_m = 1;
                self.eval_sec = self.eval_sec.wrapping_add(1);
            } else {
                self.eval_n += 1;
                if self.eval_n == 64 {
                    self.eval_n = 0;
                    self.enter_eval_idle();
                }
            }
            return;
        }
        self.eval_sec = self.eval_sec.wrapping_add(1);
        self.eval_m += 1;
        if self.eval_m == 4 {
            self.eval_m = 0;
            self.eval_found += 1;
            self.eval_n += 1;
            if self.eval_n == 64 {
                self.eval_n = 0;
                self.enter_eval_idle();
            } else if self.eval_found == 8 {
                self.eval_phase = EvalPhase::Overflow;
            }
        }
    }

    /// Step 3, the sprite overflow bug.
    ///
    /// With eight sprites already found the hardware keeps range-checking, but
    /// on a *miss* it increments `n` **and** `m` — and `m` does not carry. So it
    /// walks OAM diagonally, treating tile, attribute and X bytes as Y
    /// coordinates, which is why the overflow flag both misses real overflows
    /// and invents ones that never happened
    /// ([NESdev PPU sprite evaluation](https://www.nesdev.org/wiki/PPU_sprite_evaluation)).
    fn eval_overflow(&mut self, scanline: u16) {
        if self.eval_copy_left > 0 {
            // The three entries after a hit are read, not range-checked: `m`
            // carries into `n` normally for each of them, and when the last is
            // done the unit falls into step 4. Range-checking them instead
            // would send the pointer off down a diagonal hardware never walks.
            self.eval_copy_left -= 1;
            self.advance_eval_pointer();
            if self.eval_copy_left == 0 {
                self.enter_eval_idle();
            }
            return;
        }
        if self.sprite_in_range(self.eval_latch, scanline) {
            self.status |= STATUS_OVERFLOW;
            self.eval_copy_left = 3;
            self.advance_eval_pointer();
            return;
        }
        self.eval_n += 1;
        self.eval_m = (self.eval_m + 1) & 3;
        if self.eval_n == 64 {
            self.eval_n = 0;
            self.enter_eval_idle();
        }
    }

    /// `m += 1`, carrying into `n` — the ordinary sprite-byte step.
    fn advance_eval_pointer(&mut self) {
        self.eval_m += 1;
        if self.eval_m == 4 {
            self.eval_m = 0;
            self.eval_n = (self.eval_n + 1) & 63;
        }
    }

    /// The pattern address for the sprite currently being fetched.
    fn sprite_pattern_addr(&self, scanline: u16, high: bool) -> u16 {
        let height = self.sprite_height();
        let mut row = (scanline.wrapping_sub(u16::from(self.sp_y_latch)) as u8) & (height - 1);
        if self.sp_attr_latch & SPRITE_FLIP_Y != 0 {
            row = (height - 1) - row;
        }
        let plane = if high { 8 } else { 0 };
        if height == 16 {
            // 8x16: bit 0 of the tile byte is the bank, and the bottom half is
            // the next tile up ([NESdev PPU OAM]).
            let bank = u16::from(self.sp_tile_latch & 1) << 12;
            let mut index = u16::from(self.sp_tile_latch & 0xfe);
            if row >= 8 {
                index += 1;
                row -= 8;
            }
            bank | (index << 4) | u16::from(row) | plane
        } else {
            let bank = if self.ctrl & CTRL_SPRITE_TABLE != 0 {
                0x1000
            } else {
                0x0000
            };
            bank | (u16::from(self.sp_tile_latch) << 4) | u16::from(row) | plane
        }
    }

    /// Where the sprite unit's secondary-OAM pointer is standing.
    ///
    /// Three phases, three answers: the clear walks 0-31 two dots at a time,
    /// evaluation uses its write cursor rounded *up* to a multiple of four, and
    /// the fetch slots step through the eight-dot cadence. Outside those it
    /// sits at zero.
    fn secondary_addr(&self, dot: u16) -> u8 {
        match dot {
            1..=64 => ((dot - 1) / 2) as u8,
            65..=256 => (self.eval_sec.wrapping_add(3)) & 0x1c,
            257..=320 => {
                let slot = ((dot - 257) / 8) as u8;
                let byte = ((dot - 257) % 8).min(3) as u8;
                (slot * 4 + byte) & 31
            }
            _ => 0,
        }
    }

    /// The row copy that a mid-frame rendering change leaves behind.
    ///
    /// Changing the OAM address while the PPU is accessing OAM corrupts the row
    /// it moves to: the old row is copied over the new one. Switching rendering
    /// *off* mid-line hands the address back from the sprite unit to `OAMADDR`
    /// and freezes wherever the sprite unit had got to; switching it back on
    /// hands it the other way, and on a 2C02G that second handover reliably
    /// copies OAM row 0 over the row the sprite unit was standing on (NESdev
    /// wiki, *Errata*: "on C, E, G and H PPUs, when rendering begins
    /// automatically on pre-render dot 0, the address changes from OAM1ADDR to
    /// OAM2ADDR mid-access and reliably copies the OAM1ADDR row to the OAM2ADDR
    /// row, regardless of alignment").
    ///
    /// Identical rows corrupt nothing, which the errata is explicit about and
    /// which is what makes the ordinary case — rendering enabled in vblank with
    /// the pointer at zero — silent.
    fn corrupt_oam(&mut self) {
        let Some(row) = self.corrupt_row.take() else {
            return;
        };
        // The row the address is handed *to* takes a copy of the row it was
        // handed *from*: `OAM1ADDR` is `OAMADDR`, `OAM2ADDR` is where the
        // sprite unit was standing. Identical rows corrupt nothing, which is
        // what makes the ordinary case — a frame starting with `OAMADDR` at
        // zero and the pointer at zero — silent.
        let dst = usize::from(row & 31) * 8;
        let src = usize::from(self.oam_addr & 0xf8);
        if dst == src {
            return;
        }
        for i in 0..8 {
            let byte = self.oam[(src + i) & 0xff];
            self.write_oam(((dst + i) & 0xff) as u8, byte);
        }
        self.secondary_oam[usize::from(row & 31)] = self.secondary_oam[0];
    }

    /// What `$2004` reads back while the sprite unit owns OAM.
    ///
    /// `OAMADDR` is not the answer during rendering. The sprite unit is driving
    /// the OAM read line for its own purposes, and a CPU read of `$2004`
    /// listens in on whatever that line happens to be carrying — which is a
    /// different thing on each of the scanline's four phases:
    ///
    /// * **dots 1-64**, the secondary-OAM clear: the read line is *forced*, so
    ///   every read comes back `$FF` (NESdev, *PPU sprite evaluation*).
    /// * **dots 65-256**, evaluation: the primary-OAM read latch, held across
    ///   the odd/even pair. This is the live evaluation pointer made visible,
    ///   and it is how a program can watch sprite evaluation happen.
    /// * **dots 257-320**, the sprite fetches: secondary OAM, following the
    ///   eight-dot slot cadence — and the fourth byte of a slot is read on five
    ///   of its eight dots, which is why an empty slot reads `$FF` five times
    ///   over.
    /// * **dots 321-340 and dot 0**: secondary OAM entry 0.
    ///
    /// `None` when the sprite unit is not driving the line at all, which is
    /// every dot with rendering off and every line that does not render.
    ///
    /// # The line is registered, so it answers for the dot before
    ///
    /// The sprite unit drives the read line out of a latch, and the CPU takes
    /// the data bus at the *end* of its read cycle — by which point the engine
    /// stands on the dot after the one the cycle occupied. So `$2004` answers
    /// with the line as the previous dot left it, and all four phase
    /// boundaries sit one dot later than the sprite unit's own.
    ///
    /// AccuracyCoin's "$2004 Stress Test" measures every dot of a scanline and
    /// is unambiguous about it: dot 1 still reads secondary OAM entry 0, the
    /// forced `$FF` runs dots 2-65, evaluation dots 66-257, the fetch slots
    /// dots 258-321. Its own preamble says why — "we're aiming for the END of
    /// the CPU read occurring on specific dots here; the data from address
    /// `$2004` can change mid-read, and it's the value at the end of the read
    /// that we care about" (AccuracyCoin.asm, MIT, (c) 2025 Chris Siebert).
    fn oam_read_bus(&self) -> Option<u8> {
        if !self.rendering_enabled() || !self.render_line() {
            return None;
        }
        let dot = if self.dot == 0 {
            DOTS_PER_SCANLINE - 1
        } else {
            self.dot - 1
        };
        Some(match dot {
            1..=64 => 0xff,
            65..=256 => {
                // Odd dots read primary OAM, even dots write that byte into
                // secondary OAM — and the line carries it either way. But once
                // secondary OAM is full (step 3) or `n` has wrapped all the way
                // round (step 4) there is nothing to write, so the unit *reads*
                // secondary OAM on the even dot instead and that is what the
                // line carries: "the PPU continues reading from OAM, but reads
                // from OAM2[OAM2Address] every other cycle".
                if !dot.is_multiple_of(2) || self.eval_wrote {
                    self.eval_latch
                } else {
                    self.secondary_oam[usize::from(self.eval_sec) & 31]
                }
            }
            257..=320 => {
                let slot = usize::from((dot - 257) / 8);
                // The slot reads Y, tile, attribute, X — and then keeps the X
                // byte on the line for the rest of the slot.
                let byte = usize::from((dot - 257) % 8).min(3);
                self.secondary_oam[(slot * 4 + byte) & 31]
            }
            _ => self.secondary_oam[0],
        })
    }

    /// Dots 257-320: the four secondary-OAM reads at the head of each of the
    /// eight sprite slots.
    ///
    /// The slot's two pattern fetches are the memory cadence's
    /// ([`Engine::cadence_op`]) and land on dots 4-7 of the slot, by which time
    /// all four bytes below have been latched. Every slot fetches, including
    /// the ones no sprite was found for — the dummy fetches are what a
    /// scanline-counting mapper watches the A12 line for, so skipping them
    /// would silently break MMC3 IRQ timing later. Only the slots that hold a
    /// real sprite are drawn.
    fn sprite_fetch_dot(&mut self, dot: u16) {
        let slot = usize::from((dot - 257) / 8);
        let base = slot * 4;
        match (dot - 257) % 8 {
            0 => self.sp_y_latch = self.secondary_oam[base],
            1 => self.sp_tile_latch = self.secondary_oam[base + 1],
            2 => {
                self.sp_attr_latch = self.secondary_oam[base + 2];
                self.sprite_attr[slot] = self.sp_attr_latch;
            }
            3 => self.sprite_x[slot] = self.secondary_oam[base + 3],
            _ => {}
        }
    }

    /// One dot of the eight sprite output units: count down, or shift.
    ///
    /// Runs on dots 1-256 of a **visible** scanline, rendering or not. The two
    /// halves are gated differently and AccuracyCoin's "Stale Sprite Shift
    /// Regs" separates them in consecutive subtests:
    ///
    /// * the X **counters** keep counting through forced blank — "disabling
    ///   rendering does not stop the sprite counters ... if a sprite should be
    ///   drawn at a specific X coordinate, disabling rendering before that dot
    ///   won't affect it";
    /// * the **shifters** do not — "the actual process of using the shift
    ///   register and drawing the sprite does get paused during Forced
    ///   Blanking".
    ///
    /// Neither moves during horizontal blanking, and a unit that has already
    /// stopped counting stays stopped until dot 339 of a *rendered* scanline
    /// puts it back — "if rendering was not enabled on dot 339, the shifter
    /// counters will be in whatever state they were previously in" — which is
    /// what lets a half-drawn sprite survive ten forced-blank scanlines and
    /// finish where it left off. (AccuracyCoin.asm, MIT, (c) 2025 Chris
    /// Siebert.)
    fn sprite_output_dot(&mut self, rendering: bool) {
        for slot in 0..8 {
            if self.sprite_halted & (1 << slot) != 0 {
                if rendering {
                    self.sprite_pat_lo[slot] <<= 1;
                    self.sprite_pat_hi[slot] <<= 1;
                }
            } else {
                self.sprite_x[slot] -= 1;
            }
        }
    }

    /// A unit whose counter has run out stops counting and starts drawing.
    ///
    /// Taken at the top of the dot, before the pixel: a sprite at X = 0 draws
    /// on column 0.
    fn sprite_arm(&mut self) {
        for slot in 0..8 {
            if self.sprite_x[slot] == 0 {
                self.sprite_halted |= 1 << slot;
            }
        }
    }

    // -- the pixel multiplexer ----------------------------------------------

    /// Draw the pixel at column `x` of `scanline`.
    fn output_pixel(&mut self, x: u16, scanline: u16) {
        let emphasis = (self.mask >> 5) & 7;
        let index = if self.rendering_enabled() {
            self.render_pixel(x)
        } else if self.v & 0x3f00 == 0x3f00 {
            // Rendering off with `v` pointing into palette RAM shows that entry
            // instead of the backdrop — the flicker a game gets for updating
            // palettes outside vblank (NESdev PPU palettes).
            self.palette_read(self.v)
        } else {
            self.palette_read(0)
        };
        let index = if self.mask & MASK_GREYSCALE != 0 {
            index & 0x30
        } else {
            index
        };
        // The 2C07's video border is forced black and intrudes on the top
        // scanline of the picture, which is the whole reason the PAL and Dendy
        // picture is 239 lines out of 240 rendered (NESdev cycle reference
        // chart, "Height of picture" and "Side and bottom borders"). The
        // pipeline above still runs — sprite 0 can hit on this line — because
        // the border is painted by the video output stage, not by the
        // multiplexer.
        let index = if scanline < self.geom.top_border_lines() {
            BORDER_BLACK
        } else {
            index
        };
        let offset = usize::from(scanline) * SCREEN_WIDTH + usize::from(x);
        self.fb[offset] = Pixel::new(index, emphasis);
    }

    fn render_pixel(&mut self, x: u16) -> u8 {
        // -- background --
        let bg_visible = self.mask & MASK_BG != 0 && (x >= 8 || self.mask & MASK_BG_LEFT != 0);
        let (bg_pixel, bg_palette) = if bg_visible {
            let bit = 15 - u16::from(self.x);
            let lo = (self.bg_shift_lo >> bit) & 1;
            let hi = (self.bg_shift_hi >> bit) & 1;
            let pa_lo = (self.at_shift_lo >> bit) & 1;
            let pa_hi = (self.at_shift_hi >> bit) & 1;
            (((hi << 1) | lo) as u8, ((pa_hi << 1) | pa_lo) as u8)
        } else {
            (0, 0)
        };

        // -- sprites: the lowest OAM index with an opaque pixel wins --
        let sp_visible =
            self.mask & MASK_SPRITE != 0 && (x >= 8 || self.mask & MASK_SPRITE_LEFT != 0);
        let mut sp_pixel = 0u8;
        let mut sp_palette = 0u8;
        let mut sp_behind = false;
        let mut sp_is_zero = false;
        if sp_visible {
            for slot in 0..usize::from(self.sprite_active) {
                // A slot is drawing exactly while its counter is at zero; the
                // pixel is the top of its shift registers.
                if self.sprite_halted & (1 << slot) == 0 {
                    continue;
                }
                let lo = (self.sprite_pat_lo[slot] >> 7) & 1;
                let hi = (self.sprite_pat_hi[slot] >> 7) & 1;
                let pixel = (hi << 1) | lo;
                if pixel == 0 {
                    continue;
                }
                sp_pixel = pixel;
                sp_palette = self.sprite_attr[slot] & SPRITE_PALETTE;
                sp_behind = self.sprite_attr[slot] & SPRITE_BEHIND != 0;
                sp_is_zero = slot == 0 && self.sprite_zero_active;
                break;
            }
        }

        // -- sprite 0 hit --
        //
        // Both layers opaque, both enabled, not clipped, and never at x = 255:
        // the hardware's own pipeline cannot report a hit on the last column
        // ([NESdev PPU registers], PPUSTATUS).
        if sp_is_zero
            && bg_pixel != 0
            && self.status & STATUS_SPRITE0 == 0
            && x != 255
            && (x >= 8 || (self.mask & MASK_BG_LEFT != 0 && self.mask & MASK_SPRITE_LEFT != 0))
        {
            self.sprite0_pending = true;
        }

        // -- priority --
        match (bg_pixel, sp_pixel) {
            (0, 0) => self.palette_read(0),
            (0, _) => self.palette_read(0x10 | (u16::from(sp_palette) << 2) | u16::from(sp_pixel)),
            (_, 0) => self.palette_read((u16::from(bg_palette) << 2) | u16::from(bg_pixel)),
            _ if sp_behind => self.palette_read((u16::from(bg_palette) << 2) | u16::from(bg_pixel)),
            _ => self.palette_read(0x10 | (u16::from(sp_palette) << 2) | u16::from(sp_pixel)),
        }
    }

    // -- the dot ------------------------------------------------------------

    /// Execute the dot at the current position and advance to the next.
    pub fn tick(&mut self) {
        // The output the CPU samples lags the request by two dots — see
        // [`Engine::nmi_active`]. Both stages move before the dot runs, so
        // after a catch-up to dot *d* the output holds the level as it stood at
        // the end of *d* − 2.
        self.nmi_out = self.nmi_delay;
        self.nmi_delay = self.nmi_raw();
        // A `$2001` write reaches the pipeline a few dots late.
        if self.mask_delay > 0 {
            self.mask_delay -= 1;
            if self.mask_delay == 0 {
                let was = self.rendering_enabled();
                self.mask = self.mask_pending;
                if was && !self.rendering_enabled() && self.render_line() {
                    // The sprite unit hands the OAM address back mid-access,
                    // and where it had got to is the seed for the row copy that
                    // happens when rendering comes back — see
                    // [`Engine::corrupt_oam`].
                    self.corrupt_row = Some(self.sec_addr);
                }
            }
        }
        // A `$2006` write reaches `v` on a PPU clock edge, not at the end of
        // the CPU's cycle — see [`ADDR_WRITE_DELAY_DOTS`].
        if self.v_delay > 0 {
            self.v_delay -= 1;
            if self.v_delay == 0 {
                self.v = self.v_pending;
            }
        }
        let scanline = self.scanline;
        let dot = self.dot;
        let rendering = self.rendering_enabled();
        let visible = scanline < self.geom.visible_scanlines;
        let pre_render = scanline == self.geom.pre_render_scanline;

        // The sprite 0 flag becomes visible one dot after the pixel that caused
        // it, so the earliest possible dot is 2 (NESdev PPU rendering).
        if self.sprite0_pending {
            self.status |= STATUS_SPRITE0;
            self.sprite0_pending = false;
        }

        // -- vblank edges --
        if scanline == self.geom.vblank_scanline && dot == 0 {
            // A read that lands here is "one PPU clock before" and has already
            // set `suppress_vblank_set`; anything earlier has not.
            self.suppress_nmi = false;
        }
        if scanline == self.geom.vblank_scanline && dot == 1 {
            if self.suppress_vblank_set {
                self.suppress_vblank_set = false;
            } else {
                self.status |= STATUS_VBLANK;
                self.vblank_set_dot = self.dots;
            }
        }
        if pre_render && dot == 1 {
            self.status &= !(STATUS_VBLANK | STATUS_SPRITE0 | STATUS_OVERFLOW);
            self.suppress_nmi = false;
            self.suppress_vblank_set = false;
        }

        // The OAM address changes hands at the top of the rendering region
        // every frame, whether or not anything switched rendering off — and
        // hands over to secondary-OAM address zero, so it is silent unless
        // `OAMADDR` is pointing at some other row.
        if pre_render && dot == 0 && rendering && self.corrupt_row.is_none() {
            self.corrupt_row = Some(0);
        }
        // The row copy itself, on the first dot rendering is back on for.
        if rendering && (visible || pre_render) {
            self.corrupt_oam();
        }

        if rendering && (visible || pre_render) {
            self.render_dot(scanline, dot, visible, pre_render);
        } else {
            // Nothing is driving the pipeline's half of the bus, so `$2007`'s
            // state machine has it to itself — the ordinary case.
            self.memory_dot(None, scanline);
        }

        // The X counters run whether or not the pipeline does, so this is
        // outside the rendering gate — see [`Engine::sprite_output_dot`].
        if visible && (1..=256).contains(&dot) {
            self.sprite_arm();
        }
        if visible && (1..=256).contains(&dot) {
            self.output_pixel(dot - 1, scanline);
        }
        // After the pixel: the units that drew it then advance, which is what
        // makes a sprite eight pixels wide starting at its own X.
        if visible && (1..=256).contains(&dot) {
            self.sprite_output_dot(rendering);
        }

        self.rendered_last = rendering && (visible || pre_render);
        self.advance_position(pre_render, rendering);
        self.dots += 1;
    }

    fn render_dot(&mut self, scanline: u16, dot: u16, visible: bool, pre_render: bool) {
        // -- background --
        if (2..=257).contains(&dot) || (322..=337).contains(&dot) {
            self.shift_background();
        }
        if dot % 8 == 1
            && ((9..=257).contains(&dot) || dot == 329 || dot == 337)
            && self.rendered_last
        {
            self.reload_shifters();
        }
        // The memory bus, before anything that moves `v`: the dot-257 fetch
        // reads through the *old* `v`, one dot before the horizontal reset.
        self.memory_dot(self.cadence_op(dot), scanline);
        // Coarse X moves at dots 8, 16, ... 256, 328 and 336 (NESdev PPU
        // scrolling) — after the read on that dot, which is the high bitplane
        // of the tile the increment is finishing.
        if dot % 8 == 0 && ((8..=256).contains(&dot) || dot == 328 || dot == 336) {
            self.increment_coarse_x();
        }
        if dot == 256 {
            self.increment_y();
        }
        if dot == 257 {
            self.copy_horizontal();
        }
        if pre_render && (280..=304).contains(&dot) {
            self.copy_vertical();
        }

        // -- sprites --
        self.sec_addr = self.secondary_addr(dot);
        if dot == 339 {
            // Every unit goes back to counting, and only here. See
            // [`Engine::sprite_halted`].
            self.sprite_halted = 0;
        }
        if (1..=64).contains(&dot) {
            self.secondary_clear_dot(dot);
        } else if visible && (65..=256).contains(&dot) {
            // Evaluation does not run on the pre-render line, which is why
            // sprites never appear on scanline 0.
            self.sprite_eval_dot(dot, scanline);
        }
        if dot == 257 {
            self.sprite_active = self.eval_found;
            self.sprite_zero_active = self.sprite_zero_next;
        }
        if (257..=320).contains(&dot) {
            // OAMADDR is held at zero throughout the sprite fetches
            // ([NESdev PPU registers], OAMADDR).
            self.oam_addr = 0;
            self.sprite_fetch_dot(dot);
        }
    }

    fn advance_position(&mut self, pre_render: bool, rendering: bool) {
        self.dot += 1;
        // The odd-frame skip: with rendering enabled, an odd 2C02 frame jumps
        // straight from (339, 261) to (0, 0), making it one dot shorter
        // (NESdev PPU frame timing). The 2C07 and the UA6538 do not do it —
        // the cycle reference chart gives both a flat 341 x 312 — so it is a
        // property of the region, not of the pipeline.
        if self.geom.odd_frame_skip
            && pre_render
            && self.dot == DOTS_PER_SCANLINE - 1
            && rendering
            && !self.frame.is_multiple_of(2)
        {
            self.dot = 0;
            self.scanline = 0;
            self.frame += 1;
            return;
        }
        if self.dot == DOTS_PER_SCANLINE {
            self.dot = 0;
            self.scanline += 1;
            if self.scanline == self.geom.scanlines_per_frame {
                self.scanline = 0;
                self.frame += 1;
            }
        }
    }

    /// The next dot count at which this chip's outputs can change on their own
    /// — the catch-up bound of `ROADMAP.md` §4.2.
    ///
    /// Two kinds of instant, and the smaller one wins:
    ///
    /// * **The vblank edges.** `/NMI` is raised when the flag is set at
    ///   (`vblank_scanline`, 1) and dropped when the flag is cleared at
    ///   (`pre_render_scanline`, 1). Those are the only dots at which the chip
    ///   drives a wire without anybody having touched it, so a run loop that
    ///   stops there delivers the NMI on the cycle it happened rather than
    ///   whenever the CPU next looks.
    /// * **The next scanline**, as a ceiling. Everything else the CPU can
    ///   sample — sprite 0 hit, sprite overflow, the pixel being drawn — moves
    ///   inside a line, so stopping at every line boundary bounds how stale a
    ///   `$2002` read taken mid-quantum can be. Stopping *more* often is never
    ///   wrong: catch-up is a floor on precision, never a ceiling.
    ///
    /// The result is always **strictly ahead** of where the chip stands, which
    /// `Device::next_event_tick` requires: a target the device is already on
    /// makes no progress and would stall catch-up where it is. No candidate is
    /// ever discarded for being close, though — the one that matters most is
    /// the one two dots away.
    pub fn next_event_dot(&self) -> u64 {
        // Distance from here to (`line`, `at`), always strictly ahead: catch-up
        // that returns a tick the device already stands on makes no progress.
        let ahead = |line: u16, at: u16| -> u64 {
            let here = u64::from(self.scanline) * DOTS_PER_SCANLINE as u64 + u64::from(self.dot);
            let there = u64::from(line) * DOTS_PER_SCANLINE as u64 + u64::from(at);
            let frame = self.geom.dots_per_frame;
            self.dots + 1 + (there + frame - here - 1) % frame
        };
        // `run_to(target)` has executed every dot below `target`, so the target
        // that *includes* the dot doing the work is one past it — and two past
        // *that*, because the `/NMI` output lags the request by two dots
        // ([`Engine::nmi_active`]) and a stop that did not run the following
        // dots would leave the wire still showing the old level. The flag is
        // set at (241, 1) and cleared at (pre-render, 1), and both move `/NMI`,
        // so neither is an instant a core may be let run past.
        let vblank_set = ahead(self.geom.vblank_scanline, 4);
        let vblank_clear = ahead(self.geom.pre_render_scanline, 4);
        // The line boundary is the ceiling: nothing may go stale by more than a
        // scanline, whatever else the chip is or is not about to do.
        let next_line = self.dots + u64::from(DOTS_PER_SCANLINE - self.dot);
        vblank_set.min(vblank_clear).min(next_line)
    }

    /// Run dots until `target` total dots have executed, or until the NMI
    /// request leaves `entry`.
    ///
    /// Stopping on an NMI change is what lets the caller drop its lock before
    /// driving the wire, as the re-entrancy contract requires
    /// (`ROADMAP.md` §4.4). Returns whether `target` was reached.
    pub fn run_to(&mut self, target: u64, entry: bool) -> bool {
        while self.dots < target {
            self.tick();
            if self.nmi_active() != entry {
                return self.dots >= target;
            }
        }
        true
    }

    // -- register file ------------------------------------------------------

    /// Read register `index` (`$2000` + `index`).
    ///
    /// A `debug` access has no side effects at all: no flag clear, no toggle
    /// reset, no buffer fill, no address increment (`ROADMAP.md` §15,
    /// invariant 5).
    pub fn read_register(&mut self, index: u8, debug: bool) -> u8 {
        let now = self.dots;
        match index & 7 {
            PPUSTATUS => {
                let mut status = self.status;
                // The three flags are not sampled together. The vblank bit is
                // latched when the read begins — M2 going high — while the two
                // sprite flags are taken at the end of it, about 1.875 PPU
                // clocks later on a revision-G part. All three clear on
                // pre-render dot 1, so a read that begins on the dot before
                // that reports vblank still set and the sprite flags already
                // gone, and AccuracyCoin's "$2002 flag timing" steps a read
                // across exactly that boundary to see it.
                if self.scanline == self.geom.pre_render_scanline && self.dot == 1 {
                    status &= !(STATUS_SPRITE0 | STATUS_OVERFLOW);
                }
                let value = (status & STATUS_DRIVEN) | (self.open_bus(debug) & !STATUS_DRIVEN);
                if !debug {
                    self.status &= !STATUS_VBLANK;
                    self.w = false;
                    self.apply_status_read_race();
                    self.latch.refresh(now, value, STATUS_DRIVEN);
                }
                value
            }
            OAMDATA => {
                let value = match self.oam_read_bus() {
                    Some(byte) => byte,
                    None => self.oam[usize::from(self.oam_addr)],
                };
                if !debug {
                    self.latch.refresh(now, value, 0xff);
                }
                value
            }
            PPUDATA => self.read_data(debug),
            // Every other port is write-only and answers with open bus.
            _ => self.open_bus(debug),
        }
    }

    /// The open-bus value a read observes, folding decay away only when the
    /// access is a real one.
    fn open_bus(&mut self, debug: bool) -> u8 {
        let now = self.dots;
        if debug {
            self.latch.peek(now)
        } else {
            self.latch.read(now)
        }
    }

    /// The `$2002` read races around the vblank flag's set
    /// ([NESdev PPU frame timing](https://www.nesdev.org/wiki/PPU_frame_timing)).
    ///
    /// Positions are named by the dot *about to run*, so "the flag is set at
    /// (241, 1)" means a read while `dot == 1` happens one PPU clock before the
    /// set, and a read while `dot == 2` happens on the same clock as far as the
    /// CPU is concerned.
    fn apply_status_read_race(&mut self) {
        if self.scanline != self.geom.vblank_scanline {
            return;
        }
        match self.dot {
            // One clock before: the flag never gets set, and no NMI happens.
            1 => {
                self.suppress_vblank_set = true;
                self.suppress_nmi = true;
            }
            // On the clock, or one after: the flag reads as set and is cleared,
            // but `/NMI` is pulled back up before the CPU can see the edge.
            2 | 3 => self.suppress_nmi = true,
            _ => {}
        }
    }

    fn read_data(&mut self, debug: bool) -> u8 {
        let now = self.dots;
        let addr = self.v & 0x3fff;
        let attrs = if debug {
            MemAttrs::DEBUG
        } else {
            MemAttrs::DEFAULT
        };
        if addr >= 0x3f00 {
            // Palette RAM is internal, so it answers immediately; the read
            // buffer still gets the nametable byte hiding under the mirror,
            // and the top two bits are open bus
            // ([NESdev PPU registers], PPUDATA).
            let mut entry = self.palette_read(addr);
            if self.mask & MASK_GREYSCALE != 0 {
                entry &= 0x30;
            }
            let value = (self.open_bus(debug) & 0xc0) | entry;
            if !debug {
                // Palette RAM is inside the chip, so the *value* is immediate;
                // the buffer still gets whatever is under the mirror, and it
                // gets it through the ordinary two-dot access.
                self.start_data_fetch(addr & 0x2fff, attrs);
                self.latch.refresh(now, value, 0x3f);
            }
            value
        } else {
            let value = self.read_buffer;
            if !debug {
                self.start_data_fetch(addr, attrs);
                self.latch.refresh(now, value, 0xff);
            }
            value
        }
    }

    /// Arm the `$2007` state machine to fill the read buffer from `addr`.
    ///
    /// The fetch happens four PPU cycles after the CPU's read cycle ends, not
    /// during it — see [`DATA_SM_START`]. A second `$2007` read arriving before
    /// the first has fired is not something hardware can produce (the shortest
    /// `$2007` read is four CPU cycles, twelve dots), so the stale one is just
    /// completed against its own address rather than modelled.
    fn start_data_fetch(&mut self, addr: u16, attrs: MemAttrs) {
        if self.data_sm != 0 {
            let stale = self.data_sm_addr;
            self.read_buffer = self.bus_read(stale, attrs);
            self.increment_data_address();
        }
        self.data_sm = DATA_SM_START;
        self.data_sm_addr = addr;
    }

    /// `$2007` moves `v` by 1 or 32 — except while rendering, where it shares
    /// the scroll counters and performs a coarse-X and a Y increment instead
    /// ([NESdev PPU registers], PPUDATA).
    fn increment_data_address(&mut self) {
        if self.rendering_enabled() && self.render_line() {
            self.increment_coarse_x();
            self.increment_y();
        } else {
            let step = if self.ctrl & CTRL_INCREMENT != 0 {
                32
            } else {
                1
            };
            self.v = (self.v + step) & 0x7fff;
        }
    }

    /// Write register `index` (`$2000` + `index`).
    ///
    /// Every write drives the I/O latch, including a write to a read-only port:
    /// the latch is board capacitance, not a register, so it does not care which
    /// port the CPU addressed.
    pub fn write_register(&mut self, index: u8, value: u8) {
        let now = self.dots;
        self.latch.refresh(now, value, 0xff);
        match index & 7 {
            PPUCTRL => {
                if self.warm() {
                    self.ctrl = value;
                    self.t = (self.t & 0xf3ff) | ((u16::from(value) & 3) << 10);
                }
            }
            PPUMASK => {
                if self.warm() {
                    // Not immediate: "toggling rendering takes effect
                    // approximately 3-4 dots after the write" (NESdev, *PPU
                    // registers*). Three of AccuracyCoin's tests switch
                    // rendering off at a named dot and then measure what the
                    // pipeline was in the middle of, so the delay is the
                    // difference between the right answer and the one before it.
                    self.mask_pending = value;
                    self.mask_delay = MASK_WRITE_DELAY_DOTS;
                }
            }
            PPUSTATUS => {}
            OAMADDR => self.oam_addr = value,
            OAMDATA => self.write_oam_data(value),
            PPUSCROLL => {
                if self.warm() {
                    if self.w {
                        self.t = (self.t & 0x8fff) | ((u16::from(value) & 0x07) << 12);
                        self.t = (self.t & 0xfc1f) | ((u16::from(value) & 0xf8) << 2);
                    } else {
                        self.t = (self.t & 0xffe0) | (u16::from(value) >> 3);
                        self.x = value & 7;
                    }
                    self.w = !self.w;
                }
            }
            PPUADDR => {
                if self.warm() {
                    if self.w {
                        self.t = (self.t & 0xff00) | u16::from(value);
                        self.v_pending = self.t;
                        self.v_delay = ADDR_WRITE_DELAY_DOTS;
                    } else {
                        // Bit 14 is cleared by the high write; `t` is 15 bits
                        // but only 14 reach the bus.
                        self.t = (self.t & 0x00ff) | ((u16::from(value) & 0x3f) << 8);
                    }
                    self.w = !self.w;
                }
            }
            _ => self.write_data(value),
        }
    }

    fn write_oam_data(&mut self, value: u8) {
        if self.rendering_enabled() && self.render_line() {
            // OAM is busy being evaluated, so the write is lost — but OAMADDR
            // still takes a glitched bump of its high six bits: it is the
            // *high six* that move, so the low two are cleared as well as
            // carried into ([NESdev PPU registers], OAMDATA, and AccuracyCoin's
            // "Address $2004 behavior" code A, which starts from an odd
            // OAMADDR precisely to tell the two apart).
            self.oam_addr = self.oam_addr.wrapping_add(4) & 0xfc;
            return;
        }
        self.write_oam(self.oam_addr, value);
        self.oam_addr = self.oam_addr.wrapping_add(1);
    }

    /// Store one OAM byte, masking the three attribute bits that do not exist.
    ///
    /// Masking on the way in rather than on the way out means sprite evaluation
    /// and `$2004` reads agree without either having to remember
    /// ([NESdev PPU OAM](https://www.nesdev.org/wiki/PPU_OAM)).
    pub fn write_oam(&mut self, addr: u8, value: u8) {
        let value = if addr & 3 == 2 {
            value & SPRITE_ATTR_IMPLEMENTED
        } else {
            value
        };
        self.oam[usize::from(addr)] = value;
    }

    fn write_data(&mut self, value: u8) {
        let addr = self.v & 0x3fff;
        if addr >= 0x3f00 {
            self.palette_write(addr, value);
        } else {
            self.bus_write(addr, value, MemAttrs::DEFAULT);
        }
        self.increment_data_address();
    }

    // -- snapshots ----------------------------------------------------------

    /// Serialize every architectural bit, mid-fetch pipeline included.
    pub fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u64(self.dots)?;
        w.write_u64(self.frame)?;
        w.write_u16(self.scanline)?;
        w.write_u16(self.dot)?;

        w.write_u8(self.ctrl)?;
        w.write_u8(self.mask)?;
        w.write_u8(self.status)?;
        w.write_u8(self.oam_addr)?;
        w.write_u16(self.v)?;
        w.write_u16(self.t)?;
        w.write_u8(self.x)?;
        w.write_bool(self.w)?;
        w.write_u8(self.read_buffer)?;

        self.latch.save(w)?;
        w.write_u8(self.bus_latch)?;

        w.write_all(&self.oam)?;
        w.write_all(&self.secondary_oam)?;
        w.write_all(&self.palette)?;

        w.write_u8(self.nt_latch)?;
        w.write_u8(self.at_latch)?;
        w.write_u8(self.bg_lo_latch)?;
        w.write_u8(self.bg_hi_latch)?;
        w.write_u16(self.bg_shift_lo)?;
        w.write_u16(self.bg_shift_hi)?;
        w.write_u16(self.at_shift_lo)?;
        w.write_u16(self.at_shift_hi)?;

        w.write_u8(self.eval_phase.to_bits())?;
        w.write_u8(self.eval_n)?;
        w.write_u8(self.eval_m)?;
        w.write_u8(self.eval_sec)?;
        w.write_u8(self.eval_found)?;
        w.write_u8(self.eval_base)?;
        w.write_u8(self.eval_latch)?;
        w.write_bool(self.sprite_zero_next)?;

        w.write_all(&self.sprite_pat_lo)?;
        w.write_all(&self.sprite_pat_hi)?;
        w.write_all(&self.sprite_attr)?;
        w.write_all(&self.sprite_x)?;
        w.write_u8(self.sprite_active)?;
        w.write_bool(self.sprite_zero_active)?;
        w.write_u8(self.sp_y_latch)?;
        w.write_u8(self.sp_tile_latch)?;
        w.write_u8(self.sp_attr_latch)?;

        w.write_bool(self.sprite0_pending)?;
        w.write_u64(self.vblank_set_dot)?;
        w.write_bool(self.suppress_vblank_set)?;
        w.write_bool(self.suppress_nmi)?;
        // Appended: everything the dot-exact pipeline gained. The prefix above
        // keeps the layout the previous chunk version wrote.
        w.write_bool(self.nmi_out)?;
        w.write_u8(self.sprite_halted)?;
        w.write_u8(self.sec_addr)?;
        w.write_bool(self.corrupt_row.is_some())?;
        w.write_u8(self.corrupt_row.unwrap_or(0))?;
        w.write_u8(self.mask_pending)?;
        w.write_u8(self.mask_delay)?;
        w.write_bool(self.warmup)?;
        // v3: the board's octal address latch and `$2007`'s state machine.
        w.write_u8(self.ale_latch)?;
        w.write_u8(self.data_sm)?;
        w.write_u16(self.data_sm_addr)?;
        w.write_u16(self.v_pending)?;
        w.write_u8(self.v_delay)?;
        w.write_u8(self.eval_copy_left)?;
        w.write_bool(self.eval_wrote)?;
        w.write_bool(self.rendered_last)?;
        w.write_bool(self.nmi_delay)?;

        w.write_seq_len(self.fb.len() as u64)?;
        for pixel in self.fb.iter() {
            w.write_u16(pixel.0)?;
        }
        Ok(())
    }

    /// Restore what [`Engine::save`] wrote.
    pub fn load(&mut self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.dots = r.read_u64()?;
        self.frame = r.read_u64()?;
        self.scanline = r.read_u16()?;
        self.dot = r.read_u16()?;

        self.ctrl = r.read_u8()?;
        self.mask = r.read_u8()?;
        self.status = r.read_u8()?;
        self.oam_addr = r.read_u8()?;
        self.v = r.read_u16()?;
        self.t = r.read_u16()?;
        self.x = r.read_u8()?;
        self.w = r.read_bool()?;
        self.read_buffer = r.read_u8()?;

        self.latch.load(r)?;
        self.bus_latch = r.read_u8()?;

        read_exact(r, &mut self.oam)?;
        read_exact(r, &mut self.secondary_oam)?;
        read_exact(r, &mut self.palette)?;

        self.nt_latch = r.read_u8()?;
        self.at_latch = r.read_u8()?;
        self.bg_lo_latch = r.read_u8()?;
        self.bg_hi_latch = r.read_u8()?;
        self.bg_shift_lo = r.read_u16()?;
        self.bg_shift_hi = r.read_u16()?;
        self.at_shift_lo = r.read_u16()?;
        self.at_shift_hi = r.read_u16()?;

        self.eval_phase = EvalPhase::from_bits(r.read_u8()?);
        self.eval_n = r.read_u8()?;
        self.eval_m = r.read_u8()?;
        self.eval_sec = r.read_u8()?;
        self.eval_found = r.read_u8()?;
        self.eval_base = r.read_u8()?;
        self.eval_latch = r.read_u8()?;
        self.sprite_zero_next = r.read_bool()?;

        read_exact(r, &mut self.sprite_pat_lo)?;
        read_exact(r, &mut self.sprite_pat_hi)?;
        read_exact(r, &mut self.sprite_attr)?;
        read_exact(r, &mut self.sprite_x)?;
        self.sprite_active = r.read_u8()?;
        self.sprite_zero_active = r.read_bool()?;
        self.sp_y_latch = r.read_u8()?;
        self.sp_tile_latch = r.read_u8()?;
        self.sp_attr_latch = r.read_u8()?;

        self.sprite0_pending = r.read_bool()?;
        self.vblank_set_dot = r.read_u64()?;
        self.suppress_vblank_set = r.read_bool()?;
        self.suppress_nmi = r.read_bool()?;
        self.nmi_out = r.read_bool()?;
        self.sprite_halted = r.read_u8()?;
        self.sec_addr = r.read_u8()?;
        let pending = r.read_bool()?;
        let row = r.read_u8()?;
        self.corrupt_row = pending.then_some(row);
        self.mask_pending = r.read_u8()?;
        self.mask_delay = r.read_u8()?;
        self.warmup = r.read_bool()?;
        self.ale_latch = r.read_u8()?;
        self.data_sm = r.read_u8()?;
        self.data_sm_addr = r.read_u16()?;
        self.v_pending = r.read_u16()?;
        self.v_delay = r.read_u8()?;
        self.eval_copy_left = r.read_u8()?;
        self.eval_wrote = r.read_bool()?;
        self.rendered_last = r.read_bool()?;
        self.nmi_delay = r.read_bool()?;

        let len = r.read_seq_len(2)? as usize;
        if len != FRAMEBUFFER_LEN {
            return Err(crate::core::Error::State(alloc::format!(
                "framebuffer is {len} pixels, expected {FRAMEBUFFER_LEN}"
            )));
        }
        for pixel in self.fb.iter_mut() {
            *pixel = Pixel(r.read_u16()?);
        }
        Ok(())
    }
}

/// Fill `dst` from the reader without allocating a temporary.
fn read_exact(r: &mut ChunkReader<'_>, dst: &mut [u8]) -> Result<()> {
    let bytes = r.take(dst.len())?;
    dst.copy_from_slice(bytes);
    Ok(())
}
