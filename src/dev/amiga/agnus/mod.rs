//! Agnus: the beam counters, the copper, the blitter and chip-RAM DMA.
//!
//! One class, `amiga.agnus`, for the part an A500 calls the 8370 (NTSC) or
//! 8371 (PAL) "Fat Agnus" and, with `revision = "ecs"`, for the Enhanced Chip
//! Set parts that replaced it ([`ecs`]). It is the chip every other chip's timing hangs off:
//! the video beam's position is its counter, the vertical and horizontal sync a
//! board feeds to the CIAs' `TOD` pins come out of it, the copper runs against
//! that counter, and every direct memory access into chip RAM is scheduled by
//! it.
//!
//! ```text
//!   object custom "amiga.custom" { }
//!   object agnus  "amiga.agnus"  {
//!     clock    = clk / 8          # the colour clock: one horizontal count
//!     custom   = custom           # the register space it attaches to
//!     ram      = chipram          # what its DMA channels address
//!     standard = "pal"            # or "ntsc"
//!     paula    = paula            # optional: whose disk and audio it serves
//!     video    = denise           # optional: who it pushes lines into
//!   }
//!   wire agnus.vsync -> cia_a.tod
//!   wire agnus.hsync -> cia_b.tod
//! ```
//!
//! # Time
//!
//! Agnus is a **lazily-advanced device on its own clock domain**, and one tick
//! of that domain is one count of the horizontal beam counter — one colour
//! clock, "3,579,545 Hz" on NTSC and "3,546,895 Hz" on PAL (chapter 2,
//! *Vertical Beam Position*). A500 crystals are 28.63636 MHz and 28.37516 MHz,
//! so `clk / 8` is that rate exactly, the processor's `clk / 4` is exactly
//! twice it, and the CIAs' E clock `clk / 40` is exactly a fifth of it. Every
//! relationship between them is an integer ratio inside one oscillator tree;
//! no rate here is ever a float, and no count is ever derived from a duration
//! (`CLAUDE.md`, *Determinism*).
//!
//! What varies with the beam — how many counts a line has, how many lines a
//! field — is counted, not computed from time: [`beam`] holds the counters and
//! the arithmetic that says how far away the next line and field are.
//!
//! The chip is caught up (`ROADMAP.md` §4.2) on every register access and at
//! every quantum boundary, and it tells the scheduler about every count on
//! which something outside it changes: a sync edge on a wire somebody has
//! connected, a copper `MOVE`, the end of a blit, the start of a field. Between
//! those it runs in long strides — a copper sitting in a `WAIT` costs nothing
//! until the line its comparison can first come true on.
//!
//! # What is modelled
//!
//! * **The beam counters** ([`beam`]): PAL and NTSC line counts, long and short
//!   fields with `LOF` toggling under `BPLCON0`'s `LACE`, NTSC's alternating
//!   long and short lines, `VPOSR`/`VHPOSR` and `VPOSW`/`VHPOSW`, and the Agnus
//!   identification in `VPOSR`'s bits 14–8.
//! * **Sync outputs**: `vsync`, `hsync` and `blit` pins, described below.
//! * **`DMACON`/`DMACONR`**: the set/clear write, the master and per-channel
//!   enables, `BBUSY` and `BZERO`.
//! * **The copper** ([`copper`]): `COP1LC`/`COP2LC`, both jump strobes,
//!   `COPCON`'s danger bit, `MOVE`, `WAIT` with its masks and `BFD`, `SKIP`,
//!   and the restart at the top of every field.
//! * **The blitter** ([`blitter`]): all four channels with pointers, modulos
//!   and data registers, first- and last-word masks, both shifters, the
//!   minterm generator, ascending and descending modes, inclusive and exclusive
//!   area fill with carry-in, the zero flag, the pipeline delay on D, line mode
//!   with texture and one-dot-per-row, and the speed table's timing. The ECS
//!   `BLTCON0L`, `BLTSIZV` and `BLTSIZH` work too.
//! * **Display DMA** ([`display`]): the bitplane fetch — `DIWSTRT`/`DIWSTOP`
//!   vertically, `DDFSTRT`/`DDFSTOP` horizontally, `BPLCON0`'s plane count and
//!   resolution, `BPLxPT` and both modulos — pushed a line at a time into a
//!   Denise's [`Video`]; and the eight sprite channels — `SPRxPT`, the vertical
//!   comparison against Agnus's half of `SPRxPOS`/`SPRxCTL`, control and data
//!   words written into Denise through the bus with [`Origin::dma`].
//! * **Paula's slots** ([`slots`]): `DSKPT` and the four `AUDxLC` registers,
//!   and on each line the disk read and write and the audio restart and fetch
//!   Paula asks for, served through its [`PaulaPort`] against chip RAM; and
//!   `VERTB` and `BLIT` requested in Paula on their counts.
//! * **The beam, for the other chips**: [`ChipDma::beam`] as atomics, and
//!   [`BeamSource`], which catches Agnus up first.
//! * **The Enhanced Chip Set** ([`ecs`]), with `revision = "ecs"`: an 8372A
//!   (`reach = 1M`, the default) or an 8375 (`reach = 2M`) — `VPOSR`'s ECS
//!   identification, `LOL` and `V10`/`V9`; `BEAMCON0` with the programmable
//!   beam (`HTOTAL`, `VTOTAL`, the sync and blank positions) and PAL/NTSC
//!   switching; `DIWHIGH`; the SuperHires fetch; the copper's wider `COPCON`
//!   rule; and a [`Raster`](denise::Raster) handed to Denise every field so
//!   her picture takes the programmed beam's shape.
//!
//! # What is register-only
//!
//! Written, held, snapshotted, and not acted on:
//!
//! * `REFPTR` ("writeable for test purposes only"), `COPINS` and `SPRHDAT`.
//! * On an original part (`revision = "ocs"`, the default and an A500's), every
//!   ECS register: `HTOTAL` through `VBSTOP`, `BEAMCON0`, `HSSTRT`, `VSSTRT`,
//!   `HCENTER` and `DIWHIGH`, and `BPLCON0`'s `SHRES`. `VPOSR` says so: an
//!   8370 or 8371 does not have them.
//! * On an ECS part, `HCENTER` and `BEAMCON0`'s polarity, redirection,
//!   light-pen and `DUAL` bits: none of them changes a count or a pin this
//!   model has.
//!
//! # The pins
//!
//! | pin | rises | falls | what it is for |
//! | --- | --- | --- | --- |
//! | `vsync` | line 0, count 0: the start of vertical blanking | line [`VSYNC_LINES`] | CIA-A's `TOD` (50/60 Hz) |
//! | `hsync` | count 0 of every line | count [`HSYNC_COUNTS`] | CIA-B's `TOD` |
//! | `blit` | the count a blit finishes on | the next count | a board without Paula; with one, `BLIT` goes through [`PaulaPort::request`] |
//!
//! All three are active high and a consumer should count **rising edges**. The
//! manual places vertical blanking's start at "line 0" and `VERTB` there
//! (chapter 7, *Vertical Blanking Interrupt*); it does not place the sync
//! pulses of the original chip set within the blanking intervals, or give their
//! widths — those arrived as programmable registers with ECS. So the rising
//! edges are on the counts the manual's counters wrap on, and the widths are
//! the broadcast standards' nominal ones (about 2.5 lines, about 4.7 µs)
//! rounded to whole lines and counts. Nothing a guest can read depends on
//! either width. On an ECS part `BEAMCON0`'s `VARHSYEN` and `VARVSYEN` move
//! both edges to `HSSTRT`/`HSSTOP` and `VSSTRT`/`VSSTOP` ([`ecs::Sync`]).
//!
//! # Sources
//!
//! *Amiga Hardware Reference Manual*, Commodore-Amiga Inc., 3rd edition:
//! chapter 2 (copper), chapter 6 (blitter), chapter 7 (*Beam Position
//! Detection*, *DMA Control*, *Interrupts*), Appendix A for bit layouts,
//! Appendix B for ownership, Appendix C for the ECS additions and the Agnus
//! identification values. Each submodule cites its sections. **No emulator
//! source of any licence was consulted** — every Amiga emulator is GPL, and
//! AROS is MPL-derived (`ROADMAP.md` §1).

pub mod beam;
pub mod blitter;
pub mod copper;
pub mod display;
pub mod ecs;
pub mod slots;

#[cfg(test)]
mod tests;

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::any::Any;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::AddressSpace;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

use self::beam::{Beam, Crossing, Standard, Timing};
use self::blitter::{Blitter, Memory};
use self::copper::{Copper, Move, Phase};
use self::display::SpriteDma;
use self::ecs::Revision;
use super::custom::{CustomBus, CustomChip, Driver, Origin};
use super::denise::{self, Video};
use super::dma::{self, BeamPosition, ChipDma, DmaChannel, merge_half};
use super::paula::{self, PaulaPort};
use super::regs::{ChipId, Reg};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.agnus";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 3;

/// The vertical sync output.
pub const VSYNC_PIN: &str = "vsync";
/// The horizontal sync output.
pub const HSYNC_PIN: &str = "hsync";
/// The blitter-finished output.
pub const BLIT_PIN: &str = "blit";

/// How many lines `vsync` stays high for, from line 0. See the module
/// documentation: a nominal width, not the manual's.
pub const VSYNC_LINES: u16 = 3;

/// How many counts `hsync` stays high for, from count 0. Likewise nominal.
pub const HSYNC_COUNTS: u16 = 17;

const PIN_VSYNC: u8 = 1 << 0;
const PIN_HSYNC: u8 = 1 << 1;
const PIN_BLIT: u8 = 1 << 2;

/// `DMACONR` bit 14: blitter busy.
const BBUSY: u16 = 1 << 14;
/// `DMACONR` bit 13: blitter zero.
const BZERO: u16 = 1 << 13;
/// `BPLCON0` bit 2: interlace.
const LACE: u16 = 1 << 2;
/// `COPCON` bit 1: the copper danger bit, "only bit 1 is currently in use".
const CDANG: u16 = 1 << 1;

// Offsets of every register this chip answers, as Appendix B places them.
const DMACONR: u16 = 0x002;
const VPOSR: u16 = 0x004;
const VHPOSR: u16 = 0x006;
const DSKPTH: u16 = 0x020;
const DSKPTL: u16 = 0x022;
const REFPTR: u16 = 0x028;
const VPOSW: u16 = 0x02a;
const VHPOSW: u16 = 0x02c;
const COPCON: u16 = 0x02e;
const BLTCON0: u16 = 0x040;
const BLTCON1: u16 = 0x042;
const BLTAFWM: u16 = 0x044;
const BLTALWM: u16 = 0x046;
const BLTCPTH: u16 = 0x048;
const BLTDPTL: u16 = 0x056;
const BLTSIZE: u16 = 0x058;
const BLTCON0L: u16 = 0x05a;
const BLTSIZV: u16 = 0x05c;
const BLTSIZH: u16 = 0x05e;
const BLTCMOD: u16 = 0x060;
const BLTDMOD: u16 = 0x066;
const BLTCDAT: u16 = 0x070;
const BLTADAT: u16 = 0x074;
const SPRHDAT: u16 = 0x078;
const COP1LCH: u16 = 0x080;
const COP1LCL: u16 = 0x082;
const COP2LCH: u16 = 0x084;
const COP2LCL: u16 = 0x086;
const COPJMP1: u16 = 0x088;
const COPJMP2: u16 = 0x08a;
const COPINS: u16 = 0x08c;
const DIWSTRT: u16 = 0x08e;
const DIWSTOP: u16 = 0x090;
const DDFSTRT: u16 = 0x092;
const DDFSTOP: u16 = 0x094;
const DMACON: u16 = 0x096;
const AUD0LCH: u16 = 0x0a0;
const AUD3LCL: u16 = 0x0d2;
const BPL1PTH: u16 = 0x0e0;
const BPL6PTL: u16 = 0x0f6;
const BPLCON0: u16 = 0x100;
const BPL1MOD: u16 = 0x108;
const BPL2MOD: u16 = 0x10a;
const SPR0PTH: u16 = 0x120;
const SPR7PTL: u16 = 0x13e;
const SPR0POS: u16 = 0x140;
const SPR7CTL: u16 = 0x17a;
const HTOTAL: u16 = 0x1c0;
const VBSTOP: u16 = 0x1ce;
const BEAMCON0: u16 = 0x1dc;
const DIWHIGH: u16 = 0x1e4;

/// How many ECS beam registers are held: `HTOTAL`…`VBSTOP` and
/// `BEAMCON0`…`DIWHIGH`.
const ECS_REGS: usize = 13;

/// "Nothing scheduled".
const NO_EVENT: u64 = u64::MAX;

// ---------------------------------------------------------------------------
// the engine
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// Counts simulated. The authoritative copy; an atomic mirrors it.
    ticks: u64,
    /// Fields completed.
    field: u64,
    beam: Beam,
    copper: Copper,
    blitter: Blitter,
    /// `DMACON` bits 10–0. [`ChipDma`] holds a published copy.
    dmacon: u16,
    copcon: u16,
    bplcon0: u16,
    diwstrt: u16,
    diwstop: u16,
    ddfstrt: u16,
    ddfstop: u16,
    bplpt: [u32; 6],
    bplmod: [u16; 2],
    sprpt: [u32; 8],
    sprpos: [u16; 8],
    sprctl: [u16; 8],
    refptr: u16,
    sprhdat: u16,
    copins: u16,
    ecs: [u16; ECS_REGS],
    /// `DIWHIGH` was written after the last `DIWSTRT` or `DIWSTOP`, so its top
    /// bits are in force. An ECS part only.
    diwhigh_on: bool,
    /// Which part this is. Configuration, not guest state: kept across a reset
    /// and a load like the wiring below.
    rev: Revision,
    /// Each sprite channel's place in its field.
    sprite: [SpriteDma; 8],
    /// `blit` is high on this count.
    blit_pulse: bool,
    /// Which pins a `wire` statement has connected. Wiring, not guest state,
    /// but it decides which counts are events, so it lives beside them.
    connected: u8,
    /// Whether a video chip is attached. Wiring, likewise.
    video: bool,
    /// Whether Paula is attached. Wiring, likewise.
    paula: bool,
    /// What the last count wants done outside this chip, in order. Always
    /// empty between two calls into the chip, so never snapshotted.
    outbox: Vec<Outward>,
}

/// An action a count produced that has to happen with no lock of this chip
/// held.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Outward {
    /// A register write through the custom bus: a copper `MOVE` or a sprite
    /// DMA word.
    Write {
        offset: u16,
        value: u16,
        from: Origin,
    },
    /// A finished line, for the video chip.
    Line {
        vpos: u16,
        clocks: u16,
        start: u16,
        planes: [Vec<u16>; 6],
    },
    /// A new field, for the video chip, and — from an ECS part — the raster
    /// it will be shown on.
    Field {
        lof: bool,
        raster: Option<denise::Raster>,
    },
    /// Paula's disk and audio slots on this line.
    Slot { at: u64 },
    /// `INTREQ` bits for Paula.
    Request { at: u64, bits: u16 },
}

impl State {
    fn new() -> State {
        State {
            ticks: 0,
            field: 0,
            beam: Beam::power_on(),
            copper: Copper::new(),
            blitter: Blitter::new(),
            dmacon: 0,
            copcon: 0,
            bplcon0: 0,
            diwstrt: 0,
            diwstop: 0,
            ddfstrt: 0,
            ddfstop: 0,
            bplpt: [0; 6],
            bplmod: [0; 2],
            sprpt: [0; 8],
            sprpos: [0; 8],
            sprctl: [0; 8],
            refptr: 0,
            sprhdat: 0,
            copins: 0,
            ecs: [0; ECS_REGS],
            diwhigh_on: false,
            rev: Revision::Ocs,
            sprite: [SpriteDma::Idle; 8],
            blit_pulse: false,
            connected: 0,
            video: false,
            paula: false,
            outbox: Vec::new(),
        }
    }

    /// A part at power-on: an ECS one with `BEAMCON0`'s strap in it.
    fn for_part(rev: Revision, std: Standard) -> State {
        let mut st = State::new();
        st.rev = rev;
        if rev.is_ecs() {
            st.ecs[ecs::BEAMCON0] = ecs::beamcon0_at_reset(std);
        }
        st
    }

    #[inline]
    fn lace(&self) -> bool {
        self.bplcon0 & LACE != 0
    }

    /// The raster the counters run through: the standard's, wired in, or
    /// whatever an ECS part's `BEAMCON0` makes it.
    #[inline]
    fn timing(&self, std: Standard) -> Timing {
        if self.rev.is_ecs() {
            ecs::timing(&self.ecs, self.lace())
        } else {
            Timing::from(std)
        }
    }

    #[inline]
    fn dma_on(&self, channel: DmaChannel) -> bool {
        self.dmacon & dma::DMAEN != 0 && self.dmacon & channel.0 == channel.0
    }

    /// Whether Paula is attached and one of its slotted channels is enabled.
    #[inline]
    fn slots_on(&self) -> bool {
        self.paula && self.dmacon & dma::DMAEN != 0 && self.dmacon & slots::SLOT_CHANNELS != 0
    }

    /// The levels on the connected pins, as a bit set.
    fn pins(&self) -> u8 {
        let mut pins = 0;
        if self.rev.is_ecs() {
            let sync = ecs::Sync::of(&self.ecs);
            if ecs::in_window(self.beam.vpos, sync.v_start, sync.v_stop) {
                pins |= PIN_VSYNC;
            }
            if ecs::in_window(self.beam.hpos, sync.h_start, sync.h_stop) {
                pins |= PIN_HSYNC;
            }
        } else {
            if self.beam.vpos < VSYNC_LINES {
                pins |= PIN_VSYNC;
            }
            if self.beam.hpos < HSYNC_COUNTS {
                pins |= PIN_HSYNC;
            }
        }
        if self.blit_pulse {
            pins |= PIN_BLIT;
        }
        pins & self.connected
    }

    fn dmaconr(&self) -> u16 {
        let mut value = self.dmacon;
        if self.blitter.busy {
            value |= BBUSY;
        }
        if self.blitter.zero {
            value |= BZERO;
        }
        value
    }

    fn position(&self) -> BeamPosition {
        BeamPosition {
            tick: self.ticks,
            field: self.field,
            vpos: self.beam.vpos,
            hpos: self.beam.hpos,
            lof: self.beam.lof,
        }
    }

    /// Counts from now until the next count that matters.
    ///
    /// With `for_event` false, the next count that has to be *simulated* one at
    /// a time: a stride may go this far. With it true, the next count on which
    /// something a guest or a scheduler can observe changes — which is what the
    /// scheduler is told, and is further away: a blit's words and a line's DMA
    /// are simulated count by count or line by line but change nothing outside
    /// until the blit ends, and a video chip takes finished lines whenever the
    /// chip is next caught up (it asks [`BeamSource`], which catches up first).
    fn horizon(&self, std: Standard, for_event: bool) -> u64 {
        let t = self.timing(std);
        // A new field restarts the copper and raises `vsync`.
        let mut horizon = self.beam.ticks_to_field(t);
        if self.blit_pulse {
            return 1;
        }
        let blitting = self.blitter.busy && self.dma_on(DmaChannel::BLITTER);
        if blitting {
            if !for_event {
                return 1;
            }
            horizon = horizon.min(self.blitter.ticks_to_finish().max(1));
        }
        if self.dma_on(DmaChannel::COPPER) {
            let copper = &self.copper;
            if copper.fetching() {
                // The fetch of IR1 has no effect outside; the IR2 cycle might.
                let cycles = match copper.phase {
                    Phase::Fetch1 if for_event => 2,
                    _ => 1,
                };
                horizon = horizon.min(self.copper_cycles_ahead(t, cycles));
            } else if copper.waiting()
                && !(copper.waits_for_blitter() && self.blitter.busy)
                && let Some(ticks) =
                    copper::ticks_until_reached(copper.ir1, copper.ir2, &self.beam, t)
            {
                horizon = horizon.min(ticks);
            }
        }
        if self.rev.is_ecs() {
            horizon = horizon.min(self.ecs_sync_horizon(t));
        } else {
            if self.connected & PIN_HSYNC != 0 {
                let edge = if self.beam.hpos < HSYNC_COUNTS {
                    u64::from(HSYNC_COUNTS - self.beam.hpos)
                } else {
                    self.beam.ticks_to_line(t)
                };
                horizon = horizon.min(edge);
            }
            if self.connected & PIN_VSYNC != 0 && self.beam.vpos < VSYNC_LINES {
                horizon = horizon.min(self.beam.ticks_to_line(t));
            }
        }
        if (!for_event && self.line_work()) || self.slots_on() {
            horizon = horizon.min(self.beam.ticks_to_line(t));
        }
        horizon.max(1)
    }

    /// Counts until the next edge on a connected sync pin of an ECS part,
    /// whose windows `BEAMCON0` may have moved anywhere in the line or the
    /// field. An edge the line never reaches is never coming.
    fn ecs_sync_horizon(&self, t: Timing) -> u64 {
        let mut horizon = u64::MAX;
        let sync = ecs::Sync::of(&self.ecs);
        let beam = &self.beam;
        if self.connected & PIN_HSYNC != 0 {
            let to_line = beam.ticks_to_line(t);
            for edge in [sync.h_start, sync.h_stop] {
                let ticks = if edge > beam.hpos && edge < beam.line_len(t) {
                    u64::from(edge - beam.hpos)
                } else if edge < t.line {
                    to_line + u64::from(edge)
                } else {
                    continue;
                };
                horizon = horizon.min(ticks);
            }
        }
        if self.connected & PIN_VSYNC != 0 {
            // Whole lines: an edge is at the start of the line the level
            // changes on. Line 0 of the next field is the field's own horizon.
            let now = ecs::in_window(beam.vpos, sync.v_start, sync.v_stop);
            let next = ecs::in_window(beam.vpos + 1, sync.v_start, sync.v_stop);
            if now != next {
                horizon = horizon.min(beam.ticks_to_line(t));
            }
        }
        horizon
    }

    /// Counts until the copper's `n`th cycle from now.
    fn copper_cycles_ahead(&self, t: Timing, n: u32) -> u64 {
        let mut beam = self.beam;
        let mut seen = 0;
        let mut ticks = 0;
        while seen < n {
            beam.advance(t, self.lace());
            ticks += 1;
            if beam.hpos & 1 == 0 {
                seen += 1;
            }
        }
        ticks
    }

    /// Move `n` counts with nothing happening on any of them. The caller has
    /// established that with [`horizon`](Self::horizon), so no field boundary
    /// is crossed.
    fn stride(&mut self, std: Standard, n: u64) {
        self.beam = self.beam.ahead(self.timing(std), self.lace(), n);
        self.ticks += n;
    }

    /// Arrive at the next count and do what happens there, queueing anything
    /// that has to leave the chip.
    fn tick(&mut self, std: Standard, mem: &mut Chip<'_>) {
        self.ticks += 1;
        self.blit_pulse = false;
        let leaving = self.beam;
        let t = self.timing(std);
        let crossing = self.beam.advance(t, self.lace());
        if crossing != Crossing::None {
            self.end_of_line(t, &leaving, mem);
            if crossing == Crossing::Field {
                self.field += 1;
                self.copper.jump(false);
                self.sprite = [SpriteDma::Idle; 8];
                if self.video {
                    let raster = self
                        .rev
                        .is_ecs()
                        .then(|| ecs::raster(&self.ecs, self.timing(std)));
                    self.outbox.push(Outward::Field {
                        lof: self.beam.lof,
                        raster,
                    });
                }
                if self.paula {
                    self.outbox.push(Outward::Request {
                        at: self.ticks,
                        bits: paula::int::VERTB,
                    });
                }
            }
            self.sprite_dma(std, mem);
            if self.slots_on() {
                self.outbox.push(Outward::Slot { at: self.ticks });
            }
        }
        if self.blitter.busy && self.dma_on(DmaChannel::BLITTER) && self.blitter.tick(mem) {
            self.blit_pulse = true;
            if self.paula {
                self.outbox.push(Outward::Request {
                    at: self.ticks,
                    bits: paula::int::BLIT,
                });
            }
        }
        if self.beam.hpos & 1 == 0 && self.dma_on(DmaChannel::COPPER) {
            let busy = self.blitter.busy;
            if let Some(Move { offset, value }) =
                self.copper.cycle(|addr| mem.read(addr), &self.beam, busy)
            {
                let danger = self.copcon & CDANG != 0;
                let ecs = self.rev.is_ecs();
                // The bus still sees and counts the refused write; the copper
                // stops behind it (`copper`'s module documentation).
                self.copper.halt_if_refused(offset, danger, ecs);
                self.outbox.push(Outward::Write {
                    offset,
                    value,
                    from: if ecs {
                        Origin::ecs_copper(danger)
                    } else {
                        Origin::copper(danger)
                    },
                });
            }
        }
    }

    /// Run toward `target`, stopping early — just after the count it happened
    /// on — at anything that needs an outward action: something in the outbox,
    /// or a change on a connected pin.
    fn run(&mut self, std: Standard, target: u64, mem: &mut Chip<'_>) {
        while self.ticks < target {
            let quiet = self.horizon(std, false);
            if quiet > 1 {
                let n = (quiet - 1).min(target - self.ticks);
                self.stride(std, n);
                continue;
            }
            let before = self.pins();
            self.tick(std, mem);
            if !self.outbox.is_empty() || self.pins() != before {
                return;
            }
        }
    }

    /// A register write. Pins are compared by the caller.
    fn write(&mut self, dma: &ChipDma, offset: u16, value: u16) {
        match offset {
            DSKPTH | DSKPTL => dma.set_disk_pointer(offset == DSKPTH, value),
            REFPTR => self.refptr = value,
            VPOSW => {
                self.beam.lof = value & 0x8000 != 0;
                // An ECS part's counter is eleven bits, and V10 and V9 are
                // written beside V8 as they are read (`ecs`).
                let high = if self.rev.is_ecs() { 7 } else { 1 };
                self.beam.vpos = (self.beam.vpos & 0xff) | ((value & high) << 8);
            }
            VHPOSW => {
                self.beam.vpos = (self.beam.vpos & 0x100) | (value >> 8);
                self.beam.hpos = value & 0xff;
            }
            COPCON => self.copcon = value & CDANG,
            BLTCON0 => self.blitter.con0 = value,
            BLTCON1 => self.blitter.con1 = value,
            BLTAFWM => self.blitter.afwm = value,
            BLTALWM => self.blitter.alwm = value,
            BLTCPTH..=BLTDPTL => {
                // C, B, A, D in address order, H then L.
                let index = usize::from((offset - BLTCPTH) / 4);
                let channel = [blitter::C, blitter::B, blitter::A, blitter::D][index];
                let high = (offset - BLTCPTH).is_multiple_of(4);
                self.blitter.ptr[channel] = merge_half(self.blitter.ptr[channel], high, value);
            }
            BLTSIZE => self.blitter.start_bltsize(value),
            BLTCON0L => self.blitter.con0 = (self.blitter.con0 & 0xff00) | (value & 0x00ff),
            BLTSIZV => self.blitter.sizv = value,
            BLTSIZH => self.blitter.start_bltsizh(value),
            BLTCMOD..=BLTDMOD => {
                let channel = [blitter::C, blitter::B, blitter::A, blitter::D]
                    [usize::from((offset - BLTCMOD) / 2)];
                self.blitter.modulo[channel] = value;
            }
            BLTCDAT..=BLTADAT => {
                let channel =
                    [blitter::C, blitter::B, blitter::A][usize::from((offset - BLTCDAT) / 2)];
                self.blitter.data[channel] = value;
            }
            SPRHDAT => self.sprhdat = value,
            COP1LCH | COP1LCL => {
                self.copper.cop1lc = merge_half(self.copper.cop1lc, offset == COP1LCH, value);
            }
            COP2LCH | COP2LCL => {
                self.copper.cop2lc = merge_half(self.copper.cop2lc, offset == COP2LCH, value);
            }
            COPJMP1 => self.copper.jump(false),
            COPJMP2 => self.copper.jump(true),
            COPINS => self.copins = value,
            // Appendix C: DIWHIGH "is written last in a sequence of setting the
            // display window"; "if it is not written, the old scheme for
            // DIWSTRT and DIWSTOP described above holds". So a write to either
            // of these puts the old scheme back until DIWHIGH is written again.
            DIWSTRT => {
                self.diwstrt = value;
                self.diwhigh_on = false;
            }
            DIWSTOP => {
                self.diwstop = value;
                self.diwhigh_on = false;
            }
            DDFSTRT => self.ddfstrt = value,
            DDFSTOP => self.ddfstop = value,
            DMACON => {
                let bits = value & dma::DMACON_WRITABLE;
                if value & dma::SETCLR != 0 {
                    self.dmacon |= bits;
                } else {
                    self.dmacon &= !bits;
                }
                dma.set_dmacon(self.dmacon);
            }
            AUD0LCH..=AUD3LCL => {
                let within = offset & 0x000f;
                if within <= 2 {
                    let ch = usize::from((offset - AUD0LCH) >> 4);
                    dma.set_audio_location(ch, within == 0, value);
                }
            }
            BPL1PTH..=BPL6PTL => {
                let plane = usize::from((offset - BPL1PTH) / 4);
                let high = (offset - BPL1PTH).is_multiple_of(4);
                self.bplpt[plane] = merge_half(self.bplpt[plane], high, value);
            }
            BPLCON0 => self.bplcon0 = value,
            BPL1MOD => self.bplmod[0] = value,
            BPL2MOD => self.bplmod[1] = value,
            SPR0PTH..=SPR7PTL => {
                let sprite = usize::from((offset - SPR0PTH) / 4);
                let high = (offset - SPR0PTH).is_multiple_of(4);
                self.sprpt[sprite] = merge_half(self.sprpt[sprite], high, value);
            }
            SPR0POS..=SPR7CTL => {
                let sprite = usize::from((offset - SPR0POS) / 8);
                match (offset - SPR0POS) % 8 {
                    0 => self.sprpos[sprite] = value,
                    2 => self.sprctl[sprite] = value,
                    // DATA and DATB are Denise's; the bus never sends them here.
                    _ => {}
                }
            }
            HTOTAL..=VBSTOP => self.ecs[usize::from((offset - HTOTAL) / 2)] = value,
            BEAMCON0..=DIWHIGH => {
                self.ecs[8 + usize::from((offset - BEAMCON0) / 2)] = value;
                if offset == DIWHIGH {
                    self.diwhigh_on = self.rev.is_ecs();
                }
            }
            // Anything else the table sends here is an Agnus row this model
            // has no behaviour for and nothing to hold; there are none today.
            _ => {}
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_u64(self.ticks)?;
        w.write_u64(self.field)?;
        w.write_u16(self.beam.vpos)?;
        w.write_u16(self.beam.hpos)?;
        w.write_bool(self.beam.lof)?;
        w.write_bool(self.beam.lol)?;

        let c = &self.copper;
        w.write_u32(c.cop1lc)?;
        w.write_u32(c.cop2lc)?;
        w.write_u32(c.pc)?;
        w.write_u8(c.phase.code())?;
        w.write_u16(c.ir1)?;
        w.write_u16(c.ir2)?;

        let b = &self.blitter;
        w.write_u16(b.con0)?;
        w.write_u16(b.con1)?;
        w.write_u16(b.afwm)?;
        w.write_u16(b.alwm)?;
        for p in b.ptr {
            w.write_u32(p)?;
        }
        for m in b.modulo {
            w.write_u16(m)?;
        }
        for d in b.data {
            w.write_u16(d)?;
        }
        w.write_u16(b.sizv)?;
        w.write_bool(b.busy)?;
        w.write_bool(b.zero)?;
        w.write_u32(b.height)?;
        w.write_u32(b.width)?;
        w.write_u32(b.row)?;
        w.write_u32(b.col)?;
        w.write_u16(b.prev_a)?;
        w.write_u16(b.prev_b)?;
        w.write_bool(b.fill)?;
        w.write_bool(b.pending.is_some())?;
        let (addr, word) = b.pending.unwrap_or((0, 0));
        w.write_u32(addr)?;
        w.write_u16(word)?;
        w.write_u32(b.credit)?;
        w.write_bool(b.dotted)?;

        for v in [
            self.dmacon,
            self.copcon,
            self.bplcon0,
            self.diwstrt,
            self.diwstop,
            self.ddfstrt,
            self.ddfstop,
            self.refptr,
            self.sprhdat,
            self.copins,
        ] {
            w.write_u16(v)?;
        }
        for p in self.bplpt {
            w.write_u32(p)?;
        }
        for m in self.bplmod {
            w.write_u16(m)?;
        }
        for p in self.sprpt {
            w.write_u32(p)?;
        }
        for v in self
            .sprpos
            .iter()
            .chain(self.sprctl.iter())
            .chain(self.ecs.iter())
        {
            w.write_u16(*v)?;
        }
        for sprite in self.sprite {
            w.write_u8(sprite.code())?;
        }
        w.write_bool(self.blit_pulse)?;
        w.write_bool(self.diwhigh_on)?;
        Ok(())
    }

    fn load(&mut self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.ticks = r.read_u64()?;
        self.field = r.read_u64()?;
        self.beam.vpos = r.read_u16()?;
        self.beam.hpos = r.read_u16()?;
        self.beam.lof = r.read_bool()?;
        self.beam.lol = r.read_bool()?;

        let c = &mut self.copper;
        c.cop1lc = r.read_u32()?;
        c.cop2lc = r.read_u32()?;
        c.pc = r.read_u32()?;
        c.phase = Phase::from_code(r.read_u8()?)
            .ok_or_else(|| Error::State(String::from("amiga.agnus: an unknown copper phase")))?;
        c.ir1 = r.read_u16()?;
        c.ir2 = r.read_u16()?;

        let b = &mut self.blitter;
        b.con0 = r.read_u16()?;
        b.con1 = r.read_u16()?;
        b.afwm = r.read_u16()?;
        b.alwm = r.read_u16()?;
        for p in &mut b.ptr {
            *p = r.read_u32()?;
        }
        for m in &mut b.modulo {
            *m = r.read_u16()?;
        }
        for d in &mut b.data {
            *d = r.read_u16()?;
        }
        b.sizv = r.read_u16()?;
        b.busy = r.read_bool()?;
        b.zero = r.read_bool()?;
        b.height = r.read_u32()?;
        b.width = r.read_u32()?;
        b.row = r.read_u32()?;
        b.col = r.read_u32()?;
        b.prev_a = r.read_u16()?;
        b.prev_b = r.read_u16()?;
        b.fill = r.read_bool()?;
        let pending = r.read_bool()?;
        let addr = r.read_u32()?;
        let word = r.read_u16()?;
        b.pending = pending.then_some((addr, word));
        b.credit = r.read_u32()?;
        b.dotted = r.read_bool()?;
        if b.busy && (b.width == 0 || b.height == 0 || b.row >= b.height) {
            return Err(Error::State(String::from(
                "amiga.agnus: a blit in progress with no rows left to run",
            )));
        }

        for slot in [
            &mut self.dmacon,
            &mut self.copcon,
            &mut self.bplcon0,
            &mut self.diwstrt,
            &mut self.diwstop,
            &mut self.ddfstrt,
            &mut self.ddfstop,
            &mut self.refptr,
            &mut self.sprhdat,
            &mut self.copins,
        ] {
            *slot = r.read_u16()?;
        }
        for p in &mut self.bplpt {
            *p = r.read_u32()?;
        }
        for m in &mut self.bplmod {
            *m = r.read_u16()?;
        }
        for p in &mut self.sprpt {
            *p = r.read_u32()?;
        }
        for v in self
            .sprpos
            .iter_mut()
            .chain(self.sprctl.iter_mut())
            .chain(self.ecs.iter_mut())
        {
            *v = r.read_u16()?;
        }
        for sprite in &mut self.sprite {
            *sprite = SpriteDma::from_code(r.read_u8()?).ok_or_else(|| {
                Error::State(String::from("amiga.agnus: an unknown sprite DMA state"))
            })?;
        }
        self.blit_pulse = r.read_bool()?;
        self.diwhigh_on = r.read_bool()?;
        Ok(())
    }
}

/// Chip RAM as the copper and blitter see it.
struct Chip<'a> {
    dma: &'a ChipDma,
    space: Option<&'a AddressSpace>,
}

impl Memory for Chip<'_> {
    #[inline]
    fn read(&mut self, addr: u32) -> u16 {
        self.dma.peek(self.space, addr)
    }

    #[inline]
    fn write(&mut self, addr: u32, value: u16) {
        self.dma.poke(self.space, addr, value);
    }
}

// ---------------------------------------------------------------------------
// what both halves of the device reach
// ---------------------------------------------------------------------------

/// The output pins, once wired.
#[derive(Debug, Default, Clone)]
struct Outputs {
    vsync: Option<WireSource>,
    hsync: Option<WireSource>,
    blit: Option<WireSource>,
}

struct Shared {
    std: Standard,
    /// Which part: original or Enhanced Chip Set, and the latter's reach.
    rev: Revision,
    /// `VPOSR` bits 14–8.
    id: u8,
    state: Mutex<State>,
    /// Counts simulated, for the scheduler's lock-free question.
    ticks: AtomicU64,
    /// The count of the next event, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The chip-RAM channels, shared with Paula and Denise.
    dma: Arc<ChipDma>,
    /// The custom-chip bus the copper writes through. `None` until bind.
    bus: Mutex<Option<Arc<CustomBus>>>,
    /// The video chip lines are pushed into, if one is attached.
    video: Mutex<Option<Arc<Video>>>,
    /// Paula, if attached.
    paula: Mutex<Option<PaulaPort>>,
    out: Mutex<Outputs>,
    lazy: Mutex<Option<LazyHandle>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("std", &self.std)
            .field("ticks", &self.ticks.load(Ordering::Relaxed))
            .field("next_event", &self.next_event.load(Ordering::Relaxed))
            .field("dma", &self.dma)
            .finish_non_exhaustive()
    }
}

impl Shared {
    /// Publish what the scheduler and the other chips may ask for without a
    /// lock.
    fn publish(&self, st: &State) {
        self.publish_counters(st);
        self.dma.set_beam(st.position());
    }

    /// Publish the tick and the next event, leaving the beam position alone.
    fn publish_counters(&self, st: &State) {
        self.ticks.store(st.ticks, Ordering::Relaxed);
        self.next_event
            .store(st.ticks + st.horizon(self.std, true), Ordering::Relaxed);
    }

    /// Drive the connected pins to `pins`, with no lock of this chip held.
    fn drive(&self, pins: u8) {
        let out = self.out.lock().clone();
        for (source, bit) in [
            (&out.vsync, PIN_VSYNC),
            (&out.hsync, PIN_HSYNC),
            (&out.blit, PIN_BLIT),
        ] {
            if let Some(source) = source {
                source.set(Level::from_bool(pins & bit != 0));
            }
        }
    }

    /// Catch up before an access, unless it is a debugger's, or this chip's own
    /// copper or DMA writing back into it from inside its own catch-up.
    fn sync(&self, from: Origin) {
        if from.debug || matches!(from.driver, Driver::Copper { .. } | Driver::Dma) {
            return;
        }
        let handle = self.lazy.lock().clone();
        if let Some(handle) = handle {
            // Refused only if catch-up for this chip is already further up the
            // stack; the access is then answered from where the chip stands.
            let _ = handle.sync(AccessKind::Guest);
        }
    }

    /// Simulate until `target` counts have passed in total.
    fn advance_to(&self, target: u64) {
        let space = self.dma.space();
        loop {
            let (actions, pins, moved, arrived) = {
                let mut st = self.state.lock();
                if st.ticks >= target {
                    return;
                }
                let before = st.pins();
                let mut mem = Chip {
                    dma: &self.dma,
                    space: space.as_deref(),
                };
                st.run(self.std, target, &mut mem);
                let pins = st.pins();
                let actions = core::mem::take(&mut st.outbox);
                let arrived = st.position();
                if let Some(leaving) = self.leaving(&actions, &arrived) {
                    // A line or a field is about to be handed to the video
                    // chip. Until it has been, the beam it can ask for stays on
                    // the line being handed over: Denise's contract is that a
                    // field is pushed before any position in it is reported.
                    self.publish_counters(&st);
                    self.dma.set_beam(leaving);
                } else {
                    self.publish(&st);
                }
                (actions, pins, pins != before, arrived)
            };
            self.deliver(actions, pins, moved, arrived);
        }
    }

    /// The last count of the line an outbox hands over, if it hands one over.
    fn leaving(&self, actions: &[Outward], arrived: &BeamPosition) -> Option<BeamPosition> {
        let (vpos, clocks) = actions.iter().find_map(|a| match a {
            Outward::Line { vpos, clocks, .. } => Some((*vpos, *clocks)),
            _ => None,
        })?;
        let new_field = actions.iter().any(|a| matches!(a, Outward::Field { .. }));
        Some(BeamPosition {
            tick: arrived.tick.saturating_sub(1),
            field: arrived.field.saturating_sub(u64::from(new_field)),
            vpos,
            hpos: clocks.saturating_sub(1),
            // `LOF` belongs to the field being left, and what is published is
            // still that field's.
            lof: if new_field {
                self.dma.beam().lof
            } else {
                arrived.lof
            },
        })
    }

    /// Carry out one count's outward actions, with no lock of this chip held:
    /// the video chip's lines and field first, because a position in the new
    /// line or field must not be reported to it before they arrive; then the
    /// pins; then the register writes, in the order they were made.
    fn deliver(&self, actions: Vec<Outward>, pins: u8, moved: bool, arrived: BeamPosition) {
        if actions
            .iter()
            .any(|a| matches!(a, Outward::Line { .. } | Outward::Field { .. }))
        {
            let video = self.video.lock().clone();
            if let Some(video) = video {
                for action in &actions {
                    match action {
                        Outward::Line {
                            vpos,
                            clocks,
                            start,
                            planes,
                        } => video.line(&denise::Line {
                            vpos: *vpos,
                            clocks: *clocks,
                            fetch: denise::Fetch {
                                start: *start,
                                planes: [
                                    &planes[0], &planes[1], &planes[2], &planes[3], &planes[4],
                                    &planes[5],
                                ],
                            },
                        }),
                        Outward::Field { lof, raster } => {
                            if let Some(raster) = raster {
                                video.raster(*raster);
                            }
                            video.field(*lof);
                        }
                        _ => {}
                    }
                }
            }
            // Handed over: the beam may now be reported where it is.
            self.dma.set_beam(arrived);
        }
        if moved {
            self.drive(pins);
        }
        if actions
            .iter()
            .any(|a| matches!(a, Outward::Slot { .. } | Outward::Request { .. }))
        {
            let paula = self.paula.lock().clone();
            if let Some(paula) = paula {
                for action in &actions {
                    match *action {
                        Outward::Request { at, bits } => paula.request(at, bits),
                        Outward::Slot { at } => slots::serve(&paula, &self.dma, at),
                        _ => {}
                    }
                }
            }
        }
        if actions.iter().any(|a| matches!(a, Outward::Write { .. })) {
            let bus = self.bus.lock().clone();
            if let Some(bus) = bus {
                for action in actions {
                    if let Outward::Write {
                        offset,
                        value,
                        from,
                    } = action
                    {
                        bus.write(offset, value, from);
                    }
                }
            }
        }
    }

    /// Apply a register write and settle the pins it moved.
    fn write(&self, offset: u16, value: u16) {
        let (pins, moved) = {
            let mut st = self.state.lock();
            let before = st.pins();
            st.write(&self.dma, offset, value);
            self.publish(&st);
            let pins = st.pins();
            (pins, pins != before)
        };
        if moved {
            self.drive(pins);
        }
    }

    fn read(&self, offset: u16) -> u16 {
        let st = self.state.lock();
        match offset {
            DMACONR => st.dmaconr(),
            VPOSR if st.rev.is_ecs() => {
                // Appendix C, Determining Chip Revisions: "LOF I6 I5 I4 I3 I2
                // I1 I0 LOL -- -- -- -- v10 v9 V8".
                let b = &st.beam;
                (u16::from(b.lof) << 15)
                    | (u16::from(self.id & 0x7f) << 8)
                    | (u16::from(b.lol) << 7)
                    | ((b.vpos >> 8) & 7)
            }
            VPOSR => st.beam.vposr() | (u16::from(self.id & 0x7f) << 8),
            VHPOSR => st.beam.vhposr(),
            // The table sends only Agnus's readable rows, and those are all.
            _ => 0,
        }
    }
}

/// Where the beam is, for a chip that needs to know from inside an access.
///
/// [`now`](Self::now) catches Agnus up to the present first — unless Agnus is
/// the one making the access, in which case catch-up is already running further
/// up the stack and the published position *is* the count the access is on. It
/// takes no lock the chip holds while running, so it may be called from inside
/// a copper `MOVE`; and catching up may push lines into the caller before it
/// returns.
///
/// It holds Agnus **weakly**: Agnus holds Denise's [`Video`] and Denise holds
/// this, and a strong reference either way round would be a cycle that frees
/// neither. A source whose Agnus is gone reports the top left of the field.
#[derive(Debug, Clone)]
pub struct BeamSource {
    shared: Weak<Shared>,
}

impl BeamSource {
    /// The beam, as of now.
    #[must_use]
    pub fn now(&self) -> BeamPosition {
        let Some(shared) = self.shared.upgrade() else {
            return BeamPosition::default();
        };
        shared.sync(Origin::cpu());
        shared.dma.beam()
    }
}

/// Denise asks for the beam through its own trait; this is that trait on the
/// one type that can answer.
impl denise::Beam for BeamSource {
    fn position(&self) -> denise::BeamPosition {
        let at = self.now();
        denise::BeamPosition {
            vpos: at.vpos,
            hpos: at.hpos,
        }
    }
}

/// The register block the custom bus delivers to.
#[derive(Debug)]
struct Registers {
    shared: Arc<Shared>,
}

impl CustomChip for Registers {
    fn which(&self) -> ChipId {
        ChipId::AGNUS
    }

    fn read(&self, reg: &Reg, from: Origin) -> u16 {
        self.shared.sync(from);
        // `DMACONR` is Paula's too, and the bus ORs the two answers: every bit
        // of it is Agnus's state, so Paula has nothing to add and should answer
        // zero.
        self.shared.read(reg.offset)
    }

    fn write(&self, reg: &Reg, value: u16, from: Origin) {
        self.shared.sync(from);
        self.shared.write(reg.offset, value);
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// Agnus.
#[derive(Debug)]
pub struct Agnus {
    shared: Arc<Shared>,
    custom_path: String,
    ram_path: String,
    paula_path: Option<String>,
    video_path: Option<String>,
}

impl Agnus {
    /// Validate `props` and build the chip.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `custom` or `ram` is missing, `paula` or `video` is
    /// not a link, `standard` is not `"pal"` or `"ntsc"`, `revision` is not
    /// `"ocs"` or `"ecs"`, `reach` is given to an original part or is not 1M or
    /// 2M, or a property nothing here accepts was given.
    pub fn new(props: &Props) -> Result<Agnus> {
        let mut r = props.reader();
        let custom_path = r.require_link("custom")?.as_str().to_string();
        let ram_path = r.require_link("ram")?.as_str().to_string();
        let paula_path = r.optional_link("paula")?.map(|l| l.as_str().to_string());
        let video_path = r.optional_link("video")?.map(|l| l.as_str().to_string());
        let standard = r.or_enum("standard", "pal", &["pal", "ntsc"])?;
        let revision = r.or_enum("revision", "ocs", &["ocs", "ecs"])?;
        let reach = if r.props().contains("reach") {
            Some(r.require_size("reach")?)
        } else {
            None
        };
        r.finish()?;
        let std = Standard::parse(standard).expect("or_enum checked it");
        let rev = match (revision, reach) {
            ("ocs", None) => Revision::Ocs,
            ("ocs", Some(_)) => {
                return Err(Error::Property(String::from(
                    "amiga.agnus: `reach` chooses between the ECS parts, 8372A (1M) and 8375 (2M); \
                     an original part addresses the chip RAM it is given",
                )));
            }
            (_, None) => Revision::Ecs { reach: ecs::MIB },
            (_, Some(reach)) if reach == ecs::MIB || reach == 2 * ecs::MIB => {
                Revision::Ecs { reach }
            }
            (_, Some(reach)) => {
                return Err(Error::Property(format!(
                    "amiga.agnus: `reach` is 1M (an 8372A) or 2M (an 8375), not {reach} bytes"
                )));
            }
        };
        let mut agnus = Agnus::part(rev, std);
        agnus.custom_path = custom_path;
        agnus.ram_path = ram_path;
        agnus.paula_path = paula_path;
        agnus.video_path = video_path;
        Ok(agnus)
    }

    /// An original chip-set part with nothing attached: no bus, no chip RAM,
    /// no wires.
    #[must_use]
    pub fn bare(std: Standard) -> Agnus {
        Agnus::part(Revision::Ocs, std)
    }

    /// Part `rev` strapped for `std`, with nothing attached.
    #[must_use]
    pub fn part(rev: Revision, std: Standard) -> Agnus {
        let shared = Arc::new(Shared {
            std,
            rev,
            id: ecs::agnus_id(rev, std),
            state: Mutex::with_rank(LockRank::DEVICE, State::for_part(rev, std)),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            dma: Arc::new(ChipDma::new()),
            // LEAF: each is held for one `Arc` clone.
            bus: Mutex::with_rank(LockRank::LEAF, None),
            video: Mutex::with_rank(LockRank::LEAF, None),
            paula: Mutex::with_rank(LockRank::LEAF, None),
            out: Mutex::with_rank(LockRank::WIRE, Outputs::default()),
            lazy: Mutex::with_rank(LockRank::LEAF, None),
        });
        shared.publish(&shared.state.lock());
        Agnus {
            shared,
            custom_path: String::new(),
            ram_path: String::new(),
            paula_path: None,
            video_path: None,
        }
    }

    /// The video standard.
    #[must_use]
    pub fn standard(&self) -> Standard {
        self.shared.std
    }

    /// The chip-RAM channels Paula and Denise hold.
    #[must_use]
    pub fn dma(&self) -> &Arc<ChipDma> {
        &self.shared.dma
    }

    /// Subscribe to a custom-chip bus: the register block attaches, and the
    /// copper writes through it. What `bind` does from a machine file.
    ///
    /// # Errors
    ///
    /// If the bus already has an Agnus.
    pub fn attach_bus(&self, bus: &Arc<CustomBus>) -> Result<()> {
        bus.attach(Arc::new(Registers {
            shared: Arc::clone(&self.shared),
        }))?;
        // Agnus is the chip that drives `D15`–`D0`, so it is the chip that
        // answers a read of a write-only register: hand the register space the
        // bus this chip's DMA cycles leave their words on (`dma::ChipDataBus`).
        bus.attach_data_bus(self.shared.dma.data_bus());
        *self.shared.bus.lock() = Some(Arc::clone(bus));
        Ok(())
    }

    /// Which part this is.
    #[must_use]
    pub fn revision(&self) -> Revision {
        self.shared.rev
    }

    /// Give the DMA channels chip RAM, in a private space at address zero.
    ///
    /// An original part drives as many address bits as the RAM needs. An ECS
    /// part drives its own: the pointers' high words have "five bits, was 3
    /// bits" (Appendix C, *Other ECS Modifications*), of which an 8372A wires
    /// up enough for 1 MiB and an 8375 for 2 MiB — so a smaller RAM repeats
    /// through the part's reach as it does through an original part's, and a
    /// larger one is a board that cannot be built.
    ///
    /// # Errors
    ///
    /// If the region cannot be mapped, or is larger than an ECS part reaches.
    pub fn attach_ram(&self, region: &crate::core::space::RegionRef) -> Result<()> {
        if let Revision::Ecs { reach } = self.shared.rev
            && region.len() > reach
        {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: format!(
                    "{} bytes of chip RAM is more than this ECS Agnus reaches ({reach}); \
                     `reach = 2M` is the 8375",
                    region.len()
                ),
            });
        }
        let space = AddressSpace::new(format!("{CLASS_NAME}.chip-ram"), 32);
        {
            let mut topo = space.topology();
            topo.map(Arc::clone(region), 0)?;
        }
        self.shared.dma.attach_ram(Arc::new(space), region.len());
        Ok(())
    }

    /// Connect one output pin by name.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] for a pin this chip does not drive.
    pub fn connect_pin(&self, port: &str, source: WireSource) -> Result<()> {
        let bit = {
            let mut out = self.shared.out.lock();
            match port {
                VSYNC_PIN => out.vsync = Some(source),
                HSYNC_PIN => out.hsync = Some(source),
                BLIT_PIN => out.blit = Some(source),
                _ => {
                    return Err(Error::Config {
                        at: String::from(port),
                        message: format!(
                            "Agnus drives `{VSYNC_PIN}`, `{HSYNC_PIN}` and `{BLIT_PIN}`"
                        ),
                    });
                }
            }
            match port {
                VSYNC_PIN => PIN_VSYNC,
                HSYNC_PIN => PIN_HSYNC,
                _ => PIN_BLIT,
            }
        };
        let pins = {
            let mut st = self.shared.state.lock();
            st.connected |= bit;
            self.shared.publish(&st);
            st.pins()
        };
        self.shared.drive(pins);
        Ok(())
    }

    /// Attach Denise: from the next line on, every line the beam leaves is
    /// fetched and handed over and every field announced, and Denise is given
    /// a [`BeamSource`] so a mid-line register write lands on its pixel.
    pub fn attach_video(&self, video: Arc<Video>) {
        video.connect_beam(Arc::new(self.beam_source()));
        *self.shared.video.lock() = Some(video);
        let mut st = self.shared.state.lock();
        st.video = true;
        self.shared.publish(&st);
    }

    /// Attach Paula: from now on its disk and audio slots are served every line
    /// they are enabled, and `VERTB` and `BLIT` are requested in it.
    pub fn attach_paula(&self, paula: PaulaPort) {
        *self.shared.paula.lock() = Some(paula);
        let mut st = self.shared.state.lock();
        st.paula = true;
        self.shared.publish(&st);
    }

    /// A handle that reports the beam position and catches the chip up first —
    /// what a video chip asks from inside a register write to place it on its
    /// pixel.
    #[must_use]
    pub fn beam_source(&self) -> BeamSource {
        BeamSource {
            shared: Arc::downgrade(&self.shared),
        }
    }

    /// Connect the catch-up handle register accesses sync through.
    pub fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }

    /// Simulate until `target` counts have passed since power-on.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }

    /// Counts simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// The beam, as of the last count simulated.
    #[must_use]
    pub fn beam(&self) -> BeamPosition {
        self.shared.dma.beam()
    }

    /// `DMACONR` as the processor would read it, without catching up.
    #[must_use]
    pub fn dmaconr(&self) -> u16 {
        self.shared.state.lock().dmaconr()
    }

    /// The copper's program counter.
    #[must_use]
    pub fn copper_pc(&self) -> u32 {
        self.shared.state.lock().copper.pc
    }

    /// Whether a blit is running.
    #[must_use]
    pub fn blitter_busy(&self) -> bool {
        self.shared.state.lock().blitter.busy
    }
}

impl Device for Agnus {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the bus and chip RAM arrive at bind, the pins with
        // the wire graph.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        let pins = {
            let mut st = self.shared.state.lock();
            let fresh = State {
                // The clock is the scheduler's and does not rewind (see
                // `sms.vdp`'s reset for the long form); the wiring is not
                // state.
                ticks: st.ticks,
                connected: st.connected,
                video: st.video,
                paula: st.paula,
                // A cold start puts the beam at the top of a field; a reset
                // pulse leaves the counters running, because nothing in the
                // manual says the reset line stops the video.
                beam: if kind == ResetKind::Cold {
                    Beam::power_on()
                } else {
                    st.beam
                },
                field: if kind == ResetKind::Cold { 0 } else { st.field },
                ..State::for_part(self.shared.rev, self.shared.std)
            };
            *st = fresh;
            self.shared.dma.reset();
            self.shared.publish(&st);
            st.pins()
        };
        self.shared.drive(pins);
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.shared.state.lock().save(w)?;
        for v in self.shared.dma.state() {
            w.write_u32(v)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let pins = {
            let mut st = self.shared.state.lock();
            let mut loaded = State {
                connected: st.connected,
                video: st.video,
                paula: st.paula,
                ..State::for_part(self.shared.rev, self.shared.std)
            };
            loaded.load(r)?;
            let mut dma_state = [0u32; 11];
            for v in &mut dma_state {
                *v = r.read_u32()?;
            }
            if (dma_state[0] as u16) != loaded.dmacon {
                return Err(Error::State(String::from(
                    "amiga.agnus: the snapshot's two copies of DMACON disagree",
                )));
            }
            *st = loaded;
            self.shared.dma.restore(dma_state);
            self.shared.publish(&st);
            st.pins()
        };
        self.shared.drive(pins);
        Ok(())
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::CHIP_DMA)
            .then(|| Export::Opaque(Arc::clone(&self.shared.dma) as Arc<dyn Any + Send + Sync>))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        self.connect_pin(port, source)
    }

    fn announce(&self, _port: &str) {
        let pins = self.shared.state.lock().pins();
        self.shared.drive(pins);
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Agnus::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        Agnus::attach_lazy(self, handle);
    }
}

impl Instance for Agnus {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let bus = ctx.export_as::<CustomBus>(&self.custom_path, ExportId::CUSTOM_BUS)?;
        self.attach_bus(&bus)?;
        let region = ctx.region(&self.ram_path, "").map_err(|e| Error::Config {
            at: ctx.path().to_string(),
            message: format!("`ram` has to name the chip RAM object Agnus's DMA addresses: {e}"),
        })?;
        self.attach_ram(&region)?;
        if let Some(path) = &self.paula_path {
            let port = ctx
                .export_as::<PaulaPort>(path, ExportId::PAULA)
                .map_err(|e| Error::Config {
                    at: ctx.path().to_string(),
                    message: format!("`paula` has to name an `amiga.paula`: {e}"),
                })?;
            self.attach_paula(PaulaPort::clone(&port));
        }
        if let Some(path) = &self.video_path {
            let video = ctx
                .export_as::<Video>(path, ExportId::AMIGA_VIDEO)
                .map_err(|e| Error::Config {
                    at: ctx.path().to_string(),
                    message: format!("`video` has to name an `amiga.denise`: {e}"),
                })?;
            self.attach_video(video);
        }
        Ok(())
    }
}

/// The `amiga.agnus` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Amiga Agnus: beam counters and video sync, the copper, the blitter, and chip-RAM DMA",
    properties: &[
        PropertySpec {
            name: "custom",
            kind: ValueKind::Link,
            required: true,
            summary: "the `amiga.custom` register space the chip attaches to",
        },
        PropertySpec {
            name: "ram",
            kind: ValueKind::Link,
            required: true,
            summary: "the chip RAM object every DMA channel addresses",
        },
        PropertySpec {
            name: "standard",
            kind: ValueKind::Str,
            required: false,
            summary: "\"pal\" (the default) or \"ntsc\": line counts, line lengths, VPOSR's identification",
        },
        PropertySpec {
            name: "revision",
            kind: ValueKind::Str,
            required: false,
            summary: "\"ocs\" (the default: an 8370/8371) or \"ecs\" (an Enhanced Chip Set part: BEAMCON0, the programmable beam, SuperHires fetch, DIWHIGH)",
        },
        PropertySpec {
            name: "reach",
            kind: ValueKind::Size,
            required: false,
            summary: "an ECS part's chip-RAM reach: 1M (the default, an 8372A) or 2M (an 8375)",
        },
        PropertySpec {
            name: "paula",
            kind: ValueKind::Link,
            required: false,
            summary: "the `amiga.paula` whose disk and audio DMA this chip serves and whose VERTB and BLIT it raises",
        },
        PropertySpec {
            name: "video",
            kind: ValueKind::Link,
            required: false,
            summary: "the `amiga.denise` this chip pushes each line and field into",
        },
    ],
    construct: |props| Ok(Box::new(Agnus::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Agnus::new(props)?)))
}

/// What the validator should know about `amiga.agnus`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("custom", ValueKind::Link))
        .prop(PropSchema::new("ram", ValueKind::Link))
        .prop(PropSchema::new("standard", ValueKind::Str).values(&["pal", "ntsc"]))
        .prop(PropSchema::new("revision", ValueKind::Str).values(&["ocs", "ecs"]))
        .prop(PropSchema::new("reach", ValueKind::Size))
        .prop(PropSchema::new("paula", ValueKind::Link))
        .prop(PropSchema::new("video", ValueKind::Link))
        .port(VSYNC_PIN, PortDir::Out)
        .port(HSYNC_PIN, PortDir::Out)
        .port(BLIT_PIN, PortDir::Out)
}
