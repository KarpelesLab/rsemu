//! Denise: the Amiga's video output chip — colour table, playfields, sprites,
//! priorities and collisions.
//!
//! One class, `amiga.denise`. It subscribes to the custom-chip register space
//! ([`custom`](super::custom)) for the registers Appendix B gives it, and it
//! turns bitplane words and sprite words into 12-bit RGB pixels.
//!
//! # What Denise does not own
//!
//! **The beam, and chip RAM.** Agnus counts the beam, fetches the bitplane and
//! sprite words by DMA, and decides which line is which. Denise's pins
//! (Appendix J, "Denise Pin Assignment") are the register address bus, the
//! data bus, the mouse inputs, the 7M and CCK clocks, `/CSYNC` and the RGB
//! outputs: no vertical counter, no memory bus. So this model does not count
//! the beam and does not read memory. It is **driven**, through the seam below,
//! and a test drives it from a synthetic source until Agnus exists.
//!
//! # The seam Agnus drives
//!
//! Denise publishes an [`Arc<Video>`](Video) as [`ExportId::AMIGA_VIDEO`].
//! Agnus names it (`video = denise`), fetches it with
//! [`BindCtx::export_as`], and from then on:
//!
//! 1. **Once per line**, when the beam leaves it, calls [`Video::line`] with a
//!    [`Line`]: the line's vertical position, its length in colour clocks, and
//!    the bitplane words it fetched for that line together with the horizontal
//!    beam position the first of them was fetched at. On the silicon this is
//!    the `STRHOR`/`STREQU`/`STRVBL` strobe Agnus puts on the register bus at
//!    the start of each line and the `BPLxDAT` DMA writes it makes as it
//!    fetches ("The parallel-to-serial conversion is triggered whenever
//!    bitplane #1 is written", Appendix A, `BPLxDAT`). A line at a time rather
//!    than a word at a time, because the second is a lock and a table walk per
//!    sixteen pixels per plane.
//! 2. **Once per field**, when the vertical counter wraps, calls
//!    [`Video::field`] with the new field's long-frame bit.
//! 3. **Sprite DMA stays register writes.** The sprite channel's `SPRxPOS`,
//!    `SPRxCTL`, `SPRxDATA` and `SPRxDATB` transfers go through the
//!    [`CustomBus`] with [`Origin::dma`], exactly as the `%` column says: they
//!    are Denise registers, their side effects (a `DATA` write arms the
//!    horizontal comparator, a `CTL` write disarms it — Appendix A,
//!    `SPRxDATA`) are Denise's, and manual-mode sprites written by the
//!    processor take the identical path. Vertical start and stop are Agnus's;
//!    Denise only ever compares horizontally.
//! 4. **Optionally**, hands Denise a [`Beam`] with [`Video::connect_beam`], so
//!    a register write that lands mid-line takes effect at the pixel the beam
//!    was at rather than at the start of the next line.
//!
//! Without a [`Beam`] every register write takes effect from the start of the
//! next line [`Video::line`] renders, which is exact for a copper list that
//! changes colours in the horizontal blank and one line late for a `WAIT` in
//! mid-screen.
//!
//! ## Coordinates
//!
//! The manual uses two horizontal scales, and the seam keeps both:
//!
//! * **`hpos`**, the beam counter, in colour clocks — `VHPOSR`'s H8–H1, "1/160th
//!   of the screen width", and the unit `DDFSTRT` is written in. [`Line::clocks`],
//!   [`Fetch::start`] and [`BeamPosition::hpos`] are in these.
//! * **`x`**, in low-resolution pixels, half a colour clock each — the unit of
//!   `DIWSTRT`/`DIWSTOP` ("The resolution of horizontal start and stop is one
//!   low resolution pixel", Chapter 3) and of a sprite's nine-bit `HSTART`.
//!
//! This model puts the pixel the beam is on at `x = 2 × hpos`, and the first
//! pixel of a word fetched at `hpos = D` at `x = 2D + 17` in low resolution and
//! `x = 2D + 9` in high resolution, before `BPLCON1`'s delay. Those two
//! constants are the manual's own arithmetic, Chapter 3, "Telling the System
//! How to Fetch and Display Data": `$81 / 2 − 8.5 = $38` and
//! `$81 / 2 − 4.5 = $3C`, the standard data-fetch starts for a window at `$81`.
//!
//! # The output
//!
//! [`Video::read_row`] hands out 12-bit `0RGB` words, four bits a gun — the
//! colour register's own encoding, which is what Denise's twelve RGB pins
//! carry. Expanding that to host bytes is `host::display::amiga`'s job, not a
//! device's. The picture is [`WIDTH`] columns of high-resolution pixels (a
//! low-resolution pixel is two) starting at [`OUTPUT_LEFT`], and two rows per
//! line from the end of vertical blank, so a non-interlaced field is
//! line-doubled and an interlaced frame weaves its two fields.
//!
//! ## Why Denise is not a `Panel`
//!
//! [`dev::lcd::panel`](crate::dev::lcd::panel) is for a controller that
//! **holds** a picture and refreshes it on an oscillator nothing can observe:
//! its `generation` counts content changes and deliberately is not a frame
//! count, and its host adapter reports no frame rate. Denise is the opposite on
//! both counts. It produces a field every 20 ms whether or not anything
//! changed, the guest sees that rate (vertical blank is its heartbeat), and a
//! host must advance the machine by exactly one field to get the next picture.
//! A `Panel` that bumped `generation` every field would break that trait's
//! contract, and one with a frame period of zero would leave the host guessing.
//! So Denise has its own `Scanout` in `host::display::amiga`: its frame counter
//! is [`Video::fields`], and its period is the last field's length in colour
//! clocks against the frequency of Denise's `clock` domain (its 7M pin).
//!
//! # Modelled, and register-only
//!
//! | | |
//! | --- | --- |
//! | `COLOR00`–`COLOR31` | modelled |
//! | `BPLCON0` | `HIRES`, `BPU`, `HOMOD`, `DBLPF`, `LACE` modelled; `COLOR`, `GAUD`, `LPEN`, `ERSY` latched only |
//! | `BPLCON1` | modelled: the playfield 1 and 2 delays |
//! | `BPLCON2` | modelled: `PF2PRI`, `PF2P`, `PF1P` |
//! | `DIWSTRT`, `DIWSTOP` | modelled, OCS range rules |
//! | single, dual, hold-and-modify, extra-half-brite | modelled |
//! | `SPRxPOS`, `SPRxCTL`, `SPRxDATA`, `SPRxDATB` | modelled: horizontal comparator, arming, attachment, fixed sprite priority |
//! | `CLXCON`, `CLXDAT` | modelled; `CLXDAT` clears on read, and not on a debugger's read |
//! | `JOY0DAT`, `JOY1DAT`, `JOYTEST` | the counters and `JOYTEST`'s write; **no mouse or joystick input yet** |
//! | `DMACON` | latched only — the manual does not say what Denise does with it |
//! | `BPL1DAT`–`BPL6DAT` | latched only; the bitplane words arrive through [`Video::line`] |
//! | `STREQU`, `STRVBL`, `STRHOR`, `STRLONG` | accepted and ignored; [`Video::line`] and [`Video::field`] carry what they mean |
//! | `BPLCON3`, `DIWHIGH`, `DENISEID` | ECS registers; this is the original 8362, so the first two latch and `DENISEID` reads the floating bus ("The original Denise (8362) does not have this register", Appendix C) |
//!
//! # What the manual leaves open, and what this model chose
//!
//! * **Which pixel a hold-and-modify pixel holds** at the start of a line. The
//!   hold register starts every line at `COLOR00` and then follows the
//!   serialized bits — outside the fetched words those are zero, which selects
//!   `COLOR00` again, so the choice is only visible on a line whose data starts
//!   before the window does.
//! * **Priority codes 5–7.** Table 7-2 defines `PF1P`/`PF2P` values 0–4. A
//!   sprite group `g` is in front of a playfield whose code is `c` when `g < c`,
//!   and that comparison is extended to 5–7 unchanged.
//! * **Sprite versus two playfields at once.** Chapter 7's example says
//!   playfield 2 is in front of playfield 1 "where playfield 2 is not blocked by
//!   sprites 0 through 3". So a playfield a sprite is in front of is removed
//!   first, and `PF2PRI` then picks between whatever playfields remain.
//! * **Collisions outside the display window** are not detected; nothing is
//!   displayed there for anything to collide with.
//! * **Interlaced field order.** A long field is woven into the even rows and a
//!   short field into the odd ones.
//! * **`CLXDAT` bit 15**, "not used", reads as zero.
//!
//! # Sources
//!
//! *Amiga Hardware Reference Manual*, Commodore-Amiga Inc., 3rd edition:
//! Chapter 3 ("Playfield Hardware") for the colour table, bitplane assignment,
//! the display window, data-fetch timing, dual playfields, scrolling, and
//! hold-and-modify and extra-half-brite colour selection (Tables 3-17 to 3-19);
//! Chapter 4 ("Sprite Hardware") for the sprite data structure, attachment and
//! the hardware details of arming and the horizontal comparator; Chapter 7
//! ("System Control Hardware") for video priorities (Table 7-2) and collision
//! detection (Tables 7-3 and 7-4); Appendix A for each register's bits;
//! Appendix B for the register table; Appendix C for the ECS register notes;
//! Appendix J for Denise's pins. **No emulator source of any licence was
//! consulted** (`ROADMAP.md` §1): every Amiga emulator the author is aware of is
//! GPL, and AROS is MPL-derived.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PropSchema};

use super::custom::{CustomBus, CustomChip, Origin};
use super::regs::{ChipId, Reg};

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.denise";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 1;

/// Columns in the picture, in high-resolution pixels.
pub const WIDTH: u32 = 800;

/// The low-resolution `x` of the picture's first column.
///
/// 400 low-resolution pixels from here reach `x = 464`, which holds everything
/// a data fetch can put on a line: Chapter 3's hardware limits, `DDFSTRT = $18`
/// and `DDFSTOP = $D8`, place pixels from `2 × $18 + 17 = 65` to
/// `2 × $D8 + 17 + 15 = 464`. The nominal window `$81`–`$1C1` sits inside it
/// with a border either side. The manual says horizontal blanking leaves 368
/// of those visible and does not say where they fall, so none are cut.
pub const OUTPUT_LEFT: u16 = 64;

/// How many words a line's fetch can carry per plane.
///
/// Chapter 3, Table 3-14: 25 in low resolution and 49 in high resolution
/// between the hardware limits. A few more are accepted and ignored past the
/// line's end.
pub const MAX_FETCH_WORDS: usize = 64;

/// Changes waiting for their pixel, past which the oldest is applied at once.
///
/// Only a connected [`Beam`] queues anything, and every [`Video::line`] drains
/// what is due, so a machine that reaches this is one whose beam source stopped
/// pushing lines. Bounded so that is a picture glitch rather than a leak.
const MAX_PENDING: usize = 4096;

// ---------------------------------------------------------------------------
// register offsets — Appendix B
// ---------------------------------------------------------------------------

const CLXDAT: u16 = 0x00e;
const JOY0DAT: u16 = 0x00a;
const JOY1DAT: u16 = 0x00c;
const JOYTEST: u16 = 0x036;
const DENISEID: u16 = 0x07c;
const DIWSTRT: u16 = 0x08e;
const DIWSTOP: u16 = 0x090;
const DMACON: u16 = 0x096;
const CLXCON: u16 = 0x098;
const BPLCON0: u16 = 0x100;
const BPLCON1: u16 = 0x102;
const BPLCON2: u16 = 0x104;
const BPLCON3: u16 = 0x106;
const BPL1DAT: u16 = 0x110;
const BPL6DAT: u16 = 0x11a;
const SPR0POS: u16 = 0x140;
const SPR7DATB: u16 = 0x17e;
const COLOR00: u16 = 0x180;
const COLOR31: u16 = 0x1be;
const DIWHIGH: u16 = 0x1e4;

// BPLCON0 bits, Appendix A.
const HIRES: u16 = 1 << 15;
const HOMOD: u16 = 1 << 11;
const DBLPF: u16 = 1 << 10;
const LACE: u16 = 1 << 2;

/// `SPRxCTL` bit 7, `ATT`: "Sprite attach control bit (odd sprites)".
const ATTACH: u16 = 1 << 7;

// ---------------------------------------------------------------------------
// the seam
// ---------------------------------------------------------------------------

/// Where the beam is, as the chip that counts it reports it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default, Hash)]
pub struct BeamPosition {
    /// The vertical position: `VPOSR`'s V8 and `VHPOSR`'s V7–V0.
    pub vpos: u16,
    /// The horizontal position in colour clocks: `VHPOSR`'s H8–H1.
    pub hpos: u16,
}

/// The beam counter, as Denise asks for it.
///
/// Implemented by Agnus and handed over with [`Video::connect_beam`].
///
/// # Contract
///
/// * [`position`](Beam::position) is called from inside
///   [`CustomChip::write`] — that is, from inside a copper `MOVE` Agnus itself
///   is executing, and from inside a processor store. It must not take any lock
///   Agnus holds while running the copper. A counter published in atomics is
///   the shape that satisfies this.
/// * It reports the beam *as of the access being made*. If that position is on
///   a line Agnus has not yet pushed with [`Video::line`], the change waits for
///   that line; if it is on a line already pushed, the change applies from the
///   start of the next one.
/// * Agnus must push [`Video::field`] before reporting a position in the new
///   field.
pub trait Beam: Send + Sync + fmt::Debug {
    /// The beam position now.
    fn position(&self) -> BeamPosition;
}

/// One line's bitplane data, as Agnus fetched it.
#[derive(Debug, Clone, Copy, Default)]
pub struct Fetch<'a> {
    /// The horizontal beam position, in colour clocks, at which the first word
    /// was fetched — `DDFSTRT` on a line where the fetch started on time.
    pub start: u16,
    /// The words fetched for bitplanes 1–6, in fetch order, leftmost first.
    ///
    /// A plane `BPLCON0` does not enable is ignored whatever is here, and so is
    /// anything past [`MAX_FETCH_WORDS`]. An empty slice is a plane with no data
    /// on this line: its pixels are zero.
    pub planes: [&'a [u16]; 6],
}

/// One line, from the chip that counts the beam.
#[derive(Debug, Clone, Copy, Default)]
pub struct Line<'a> {
    /// The line's vertical beam position.
    pub vpos: u16,
    /// The line's length in colour clocks — 227, or 228 for a long line
    /// (Appendix A, `STRLONG`: "lines with long counts (228)").
    pub clocks: u16,
    /// The bitplane words fetched for this line.
    pub fetch: Fetch<'a>,
}

/// The television standard, which decides only how many lines the picture has.
///
/// A closed set, so a real enum. Denise is the same silicon in both; what
/// differs is how many lines Agnus sends and where vertical blank ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Standard {
    /// 312 lines a field, 313 in a long one; blanking to line `$1D`.
    Pal,
    /// 262 lines a field, 263 in a long one; blanking to line `$15`.
    Ntsc,
}

impl Standard {
    /// The first line after vertical blank: Chapter 3, Table 3-13, "Vertical
    /// Blank Stop".
    #[must_use]
    pub const fn first_line(self) -> u16 {
        match self {
            Standard::Pal => 0x1d,
            Standard::Ntsc => 0x15,
        }
    }

    /// Lines after vertical blank in a long field: Table 3-13's displayable
    /// line counts, 567 = 283 + 284 interlaced PAL and 483 = 241 + 242 NTSC.
    #[must_use]
    pub const fn lines(self) -> u16 {
        match self {
            Standard::Pal => 284,
            Standard::Ntsc => 242,
        }
    }

    /// Rows in the picture: two per line.
    #[must_use]
    pub const fn height(self) -> u32 {
        self.lines() as u32 * 2
    }
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// Every register that shapes the picture, as of the pixel being drawn.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
struct Regs {
    color: [u16; 32],
    bplcon0: u16,
    bplcon1: u16,
    bplcon2: u16,
    bplcon3: u16,
    diwstrt: u16,
    diwstop: u16,
    diwhigh: u16,
    clxcon: u16,
    bpldat: [u16; 6],
    spr_pos: [u16; 8],
    spr_ctl: [u16; 8],
    spr_data: [u16; 8],
    spr_datb: [u16; 8],
    /// One bit per sprite: its horizontal comparator is enabled.
    armed: u8,
}

impl Regs {
    /// Whether a write to `offset` changes what a pixel looks like, and so has
    /// to land at a beam position rather than at once.
    fn timed(offset: u16) -> bool {
        matches!(
            offset,
            DIWSTRT
                | DIWSTOP
                | CLXCON
                | BPLCON0..=BPLCON3
                | BPL1DAT..=BPL6DAT
                | SPR0POS..=SPR7DATB
                | COLOR00..=COLOR31
                | DIWHIGH
        )
    }

    fn apply(&mut self, offset: u16, value: u16) {
        match offset {
            COLOR00..=COLOR31 => {
                // "Bits 15 - 12 Unused" (Table 3-3): Denise has twelve RGB
                // pins and nothing to put the top nibble on.
                self.color[usize::from((offset - COLOR00) / 2)] = value & 0x0fff;
            }
            BPLCON0 => self.bplcon0 = value,
            BPLCON1 => self.bplcon1 = value,
            BPLCON2 => self.bplcon2 = value,
            BPLCON3 => self.bplcon3 = value,
            DIWSTRT => self.diwstrt = value,
            DIWSTOP => self.diwstop = value,
            DIWHIGH => self.diwhigh = value,
            CLXCON => self.clxcon = value,
            BPL1DAT..=BPL6DAT => self.bpldat[usize::from((offset - BPL1DAT) / 2)] = value,
            SPR0POS..=SPR7DATB => {
                let rel = offset - SPR0POS;
                let i = usize::from(rel / 8);
                match rel % 8 {
                    0 => self.spr_pos[i] = value,
                    2 => {
                        // "Writing to the SPRxCTL register disables the
                        // sprite" (Appendix A, SPRxDATA).
                        self.spr_ctl[i] = value;
                        self.armed &= !(1 << i);
                    }
                    4 => {
                        // "Writing to the A buffer enables (arms) the sprite."
                        self.spr_data[i] = value;
                        self.armed |= 1 << i;
                    }
                    _ => self.spr_datb[i] = value,
                }
            }
            _ => {}
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        for c in self.color {
            w.write_u16(c)?;
        }
        for v in [
            self.bplcon0,
            self.bplcon1,
            self.bplcon2,
            self.bplcon3,
            self.diwstrt,
            self.diwstop,
            self.diwhigh,
            self.clxcon,
        ] {
            w.write_u16(v)?;
        }
        for v in self.bpldat {
            w.write_u16(v)?;
        }
        for table in [&self.spr_pos, &self.spr_ctl, &self.spr_data, &self.spr_datb] {
            for v in table {
                w.write_u16(*v)?;
            }
        }
        w.write_u8(self.armed)
    }

    fn load(r: &mut ChunkReader<'_>) -> Result<Regs> {
        let mut regs = Regs::default();
        for c in &mut regs.color {
            *c = r.read_u16()? & 0x0fff;
        }
        for v in [
            &mut regs.bplcon0,
            &mut regs.bplcon1,
            &mut regs.bplcon2,
            &mut regs.bplcon3,
            &mut regs.diwstrt,
            &mut regs.diwstop,
            &mut regs.diwhigh,
            &mut regs.clxcon,
        ] {
            *v = r.read_u16()?;
        }
        for v in &mut regs.bpldat {
            *v = r.read_u16()?;
        }
        for table in [
            &mut regs.spr_pos,
            &mut regs.spr_ctl,
            &mut regs.spr_data,
            &mut regs.spr_datb,
        ] {
            for v in table.iter_mut() {
                *v = r.read_u16()?;
            }
        }
        regs.armed = r.read_u8()?;
        Ok(regs)
    }
}

/// When a change happened: the field it was made in, and the beam position.
///
/// Ordered field first, so a sorted queue stays sorted across a field wrap.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Stamp {
    field: u64,
    vpos: u16,
    hpos: u16,
}

/// A register write waiting for its pixel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Change {
    at: Stamp,
    offset: u16,
    value: u16,
}

/// Everything behind the lock.
struct State {
    regs: Regs,
    pending: VecDeque<Change>,
    clxdat: u16,
    joy: [u16; 2],
    dmacon: u16,
    /// Fields begun since reset — the host's frame counter.
    fields: u64,
    /// The current field's long-frame bit.
    lof: bool,
    /// Colour clocks pushed so far in the current field.
    clocks: u64,
    /// Colour clocks in the last complete field.
    last_field_clocks: u64,
    /// The picture: `WIDTH × height` 12-bit RGB words.
    frame: Vec<u16>,
}

impl fmt::Debug for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Not the frame: nine hundred thousand words in a panic message helps
        // nobody.
        f.debug_struct("State")
            .field("regs", &self.regs)
            .field("pending", &self.pending.len())
            .field("clxdat", &self.clxdat)
            .field("fields", &self.fields)
            .field("lof", &self.lof)
            .finish_non_exhaustive()
    }
}

impl State {
    fn new(standard: Standard) -> State {
        State {
            regs: Regs::default(),
            pending: VecDeque::new(),
            clxdat: 0,
            joy: [0; 2],
            dmacon: 0,
            fields: 0,
            lof: true,
            clocks: 0,
            last_field_clocks: 0,
            frame: vec![0; WIDTH as usize * standard.height() as usize],
        }
    }

    /// Apply every queued change stamped before `at`.
    fn apply_before(&mut self, at: Stamp) {
        while let Some(change) = self.pending.front().copied() {
            if change.at >= at {
                break;
            }
            self.pending.pop_front();
            self.regs.apply(change.offset, change.value);
        }
    }
}

// ---------------------------------------------------------------------------
// the pixel pipeline
// ---------------------------------------------------------------------------

/// The per-line decisions `BPLCON0`, `BPLCON1` and the window make, recomputed
/// only when one of them changes.
#[derive(Debug, Clone, Copy)]
struct Setup {
    /// Planes enabled, 0–6.
    planes: usize,
    hires: bool,
    dual: bool,
    ham: bool,
    ehb: bool,
    /// Per plane, the high-resolution half-pixel of its first bit.
    start: [i32; 6],
    /// Inside the window vertically.
    inside_v: bool,
    hstart: u32,
    hstop: u32,
}

impl Setup {
    fn of(regs: &Regs, fetch_start: u16, vpos: u16) -> Setup {
        let con0 = regs.bplcon0;
        // "111 not used" (Table 3-5). Six is the most there are.
        let planes = usize::from((con0 >> 12) & 7).min(6);
        let hires = con0 & HIRES != 0;
        let dual = con0 & DBLPF != 0;
        // Chapter 3, "Hold-And-Modify Mode": HOMOD set, DBLPF clear, HIRES
        // clear, and five or six planes, or it is not active.
        let ham = con0 & HOMOD != 0 && !dual && !hires && planes >= 5;
        // Appendix A, BPLCON0: "0 = Extra Half Brite (EHB) if HAM=0 and BPU=6
        // and DBLPF=0".
        let ehb = con0 & HOMOD == 0 && !dual && planes == 6;

        // Chapter 3: DIWSTRT at $81 goes with DDFSTRT at $38 in low resolution
        // and $3C in high, "$81/2 - 8.5 = $38" and "$81/2 - 4.5 = $3C".
        let lead: i32 = if hires { 9 } else { 17 };
        let mut start = [0i32; 6];
        for (p, s) in start.iter_mut().enumerate() {
            // Odd planes (1, 3, 5, index 0, 2, 4) take playfield 1's delay in
            // BPLCON1 bits 3-0, even planes playfield 2's in bits 7-4. One
            // unit is one low-resolution pixel in either resolution: "In high
            // resolution mode, scrolling is in increments of 2 pixels."
            let delay = if p % 2 == 0 {
                regs.bplcon1 & 0xf
            } else {
                (regs.bplcon1 >> 4) & 0xf
            };
            *s = 2 * (2 * i32::from(fetch_start) + lead + i32::from(delay));
        }

        // Appendix A, DIWSTRT/DIWSTOP: start is restricted to H8=0 and V8=0;
        // stop to H8=1 and "V8=/=V7".
        let vstart = regs.diwstrt >> 8;
        let vstop_lo = regs.diwstop >> 8;
        let vstop = vstop_lo | if vstop_lo & 0x80 == 0 { 0x100 } else { 0 };
        Setup {
            planes,
            hires,
            dual,
            ham,
            ehb,
            start,
            inside_v: vstart <= vpos && vpos < vstop,
            hstart: u32::from(regs.diwstrt & 0xff),
            hstop: 0x100 | u32::from(regs.diwstop & 0xff),
        }
    }

    /// The six plane bits at high-resolution half-pixel `hx`, plane 1 in bit 0.
    #[inline]
    fn bits(&self, fetch: &Fetch<'_>, hx: i32) -> u8 {
        let mut bits = 0u8;
        for p in 0..self.planes {
            let rel = hx - self.start[p];
            if rel < 0 {
                continue;
            }
            let bit = if self.hires { rel } else { rel >> 1 } as usize;
            let words = &fetch.planes[p][..fetch.planes[p].len().min(MAX_FETCH_WORDS)];
            if let Some(word) = words.get(bit / 16) {
                bits |= (((word >> (15 - bit % 16)) & 1) as u8) << p;
            }
        }
        bits
    }
}

/// A sprite's serializer: the word pair it loaded at its comparator match, and
/// how many pixels are left to shift out.
#[derive(Debug, Clone, Copy, Default)]
struct Shifter {
    a: u16,
    b: u16,
    left: u8,
}

/// The frontmost sprite pixel: its group (0–3) and colour register.
///
/// Chapter 7: lower-numbered sprites are always in front, and for priority
/// sprites go in pairs. Chapter 4, "Attached Sprites": with `ATT` set on the
/// odd sprite the pair is one four-bit object, odd sprite high, selecting
/// registers 17–31; otherwise each selects "16 + 4k + value" from its pair's
/// four (Table 4-6). A value of zero is transparent either way.
#[inline]
fn front_sprite(pixels: &[u8; 8], regs: &Regs) -> Option<(u8, usize)> {
    for k in 0..4 {
        let even = pixels[2 * k];
        let odd = pixels[2 * k + 1];
        if regs.spr_ctl[2 * k + 1] & ATTACH != 0 {
            let value = (odd << 2) | even;
            if value != 0 {
                return Some((k as u8, 16 + usize::from(value)));
            }
        } else if even != 0 {
            return Some((k as u8, 16 + 4 * k + usize::from(even)));
        } else if odd != 0 {
            return Some((k as u8, 16 + 4 * k + usize::from(odd)));
        }
    }
    None
}

/// The collision bits one pixel raises (Tables 7-3 and 7-4).
#[inline]
fn collisions(bits: u8, pixels: &[u8; 8], clxcon: u16) -> u16 {
    let enabled = (clxcon >> 6) & 0x3f;
    let match_value = clxcon & 0x3f;
    // A disabled plane "cannot prevent collisions", so only the enabled ones
    // are compared, and a playfield with none enabled always matches.
    let mismatch = (u16::from(bits) ^ match_value) & enabled;
    let odd = mismatch & 0b01_0101 == 0;
    let even = mismatch & 0b10_1010 == 0;

    let mut groups = 0u8;
    for k in 0..4 {
        // "The even-numbered sprites always are included"; ENSP1/3/5/7 (bits
        // 12-15) OR in the odd one.
        let include_odd = clxcon & (1 << (12 + k)) != 0;
        if pixels[2 * k] != 0 || (include_odd && pixels[2 * k + 1] != 0) {
            groups |= 1 << k;
        }
    }

    let mut out = u16::from(odd && even);
    for k in 0..4 {
        if groups & (1 << k) != 0 {
            if odd {
                out |= 1 << (1 + k);
            }
            if even {
                out |= 1 << (5 + k);
            }
        }
    }
    // Bits 9-14: sprite group against sprite group, in the table's order.
    for (bit, (a, b)) in [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]
        .into_iter()
        .enumerate()
    {
        if groups & (1 << a) != 0 && groups & (1 << b) != 0 {
            out |= 1 << (9 + bit);
        }
    }
    out
}

/// Render one line into `st`. The heart of the chip.
fn render(standard: Standard, st: &mut State, line: &Line<'_>) {
    let field = st.fields;
    let vpos = line.vpos;
    st.apply_before(Stamp {
        field,
        vpos,
        hpos: 0,
    });
    st.clocks = st.clocks.wrapping_add(u64::from(line.clocks));

    // Which rows of the picture this line lands on. Interlace is decided at
    // the start of the line; a mid-line LACE change is not a picture anyone
    // has asked for.
    let first = standard.first_line();
    let rows: [Option<usize>; 2] = if vpos >= first && vpos - first < standard.lines() {
        let base = 2 * usize::from(vpos - first);
        if st.regs.bplcon0 & LACE != 0 {
            [Some(base + usize::from(!st.lof)), None]
        } else {
            [Some(base), Some(base + 1)]
        }
    } else {
        [None, None]
    };

    // The whole line, and never less than the picture's right edge: `x` is
    // where a pixel is *displayed*, and the serializer is 17 pixels behind the
    // fetch, so a word fetched at the `DDFSTOP` limit of `$D8` is displayed up
    // to `x = 464` on a line whose beam counter stops at 454. Capped at 1024
    // so a beam source that reports an absurd length cannot make a line cost
    // more than twice the longest real one.
    let span = (u32::from(line.clocks) * 2)
        .max(u32::from(OUTPUT_LEFT) + WIDTH / 2)
        .min(1024);
    let mut setup = Setup::of(&st.regs, line.fetch.start, vpos);

    // A line with no rows in the picture, outside the window, and nothing
    // queued against it draws nothing and cannot collide: skip the pixels.
    // Vertical blank is a tenth of every field.
    let queued_here = st
        .pending
        .front()
        .is_some_and(|c| c.at.field == field && c.at.vpos == vpos);
    if rows == [None, None] && !setup.inside_v && !queued_here {
        return;
    }

    let mut shifters = [Shifter::default(); 8];
    let mut hold = st.regs.color[0];
    let mut clx = 0u16;

    for x in 0..span {
        // Changes stamped on this line land at their pixel.
        let mut changed = false;
        while let Some(change) = st.pending.front().copied() {
            if change.at.field != field
                || change.at.vpos != vpos
                || u32::from(change.at.hpos) * 2 > x
            {
                break;
            }
            st.pending.pop_front();
            st.regs.apply(change.offset, change.value);
            changed = true;
        }
        if changed {
            setup = Setup::of(&st.regs, line.fetch.start, vpos);
        }
        let regs = &st.regs;

        // The horizontal comparators (Chapter 4, "Sprite Hardware Details"):
        // an armed sprite loads its buffers into its shifter when the beam
        // reaches HSTART, and shifts one bit out per low-resolution pixel.
        let mut pixels = [0u8; 8];
        if regs.armed != 0 || shifters.iter().any(|s| s.left != 0) {
            for (i, s) in shifters.iter_mut().enumerate() {
                let hstart =
                    (u32::from(regs.spr_pos[i] & 0xff) << 1) | u32::from(regs.spr_ctl[i] & 1);
                if regs.armed & (1 << i) != 0 && x == hstart {
                    *s = Shifter {
                        a: regs.spr_data[i],
                        b: regs.spr_datb[i],
                        left: 16,
                    };
                }
                if s.left != 0 {
                    // DATB is the high-order digit (Chapter 4, "Sprite Color
                    // Descriptor Words": the second word of a pair).
                    pixels[i] = (((s.b >> 15) & 1) << 1 | ((s.a >> 15) & 1)) as u8;
                }
            }
        }
        let sprite = front_sprite(&pixels, regs);
        let inside = setup.inside_v && setup.hstart <= x && x < setup.hstop;

        for sub in 0..2 {
            let hx = (2 * x + sub) as i32;
            let bits = setup.bits(&line.fetch, hx);

            // The hold register follows the serialized bits whether or not
            // anything is displayed over them.
            if setup.ham {
                let low = u16::from(bits & 0xf);
                hold = match (bits >> 4) & 3 {
                    0 => regs.color[usize::from(bits & 0xf)],
                    1 => (hold & 0xff0) | low,
                    2 => (hold & 0x0ff) | (low << 8),
                    _ => (hold & 0xf0f) | (low << 4),
                };
            }

            let colour = if !inside {
                regs.color[0]
            } else {
                clx |= collisions(bits, &pixels, regs.clxcon);
                let group = sprite.map(|s| u16::from(s.0));
                let sprite_colour = sprite.map(|s| regs.color[s.1]);
                // A playfield a sprite group is in front of is hidden at this
                // pixel (Table 7-2: group g is in front of code c when g < c).
                let blocked = |code: u16| group.is_some_and(|g| g < code);
                if setup.dual {
                    // Playfield 1 is planes 1, 3, 5; playfield 2 is 2, 4, 6.
                    let odd = (bits & 1) | ((bits >> 1) & 2) | ((bits >> 2) & 4);
                    let even = ((bits >> 1) & 1) | ((bits >> 2) & 2) | ((bits >> 3) & 4);
                    let show1 = odd != 0 && !blocked(regs.bplcon2 & 7);
                    let show2 = even != 0 && !blocked((regs.bplcon2 >> 3) & 7);
                    let pf2_first = regs.bplcon2 & (1 << 6) != 0;
                    match (show1, show2) {
                        (true, true) if pf2_first => regs.color[8 + usize::from(even)],
                        (true, _) => regs.color[usize::from(odd)],
                        (false, true) => regs.color[8 + usize::from(even)],
                        (false, false) => sprite_colour.unwrap_or(regs.color[0]),
                    }
                } else {
                    // "PF2P2 - PF2P0, bits 5-3, are the priority bits for
                    // normal (non-dual) playfields" (Chapter 7).
                    if bits != 0 && !blocked((regs.bplcon2 >> 3) & 7) {
                        if setup.ham {
                            hold
                        } else if setup.ehb && bits & 0x20 != 0 {
                            // "shifted to half-intensity by the sixth bitplane"
                            (regs.color[usize::from(bits & 0x1f)] >> 1) & 0x777
                        } else {
                            regs.color[usize::from(bits & 0x1f)]
                        }
                    } else {
                        sprite_colour.unwrap_or(regs.color[0])
                    }
                }
            };

            let col = hx - 2 * i32::from(OUTPUT_LEFT);
            if (0..WIDTH as i32).contains(&col) {
                for row in rows.into_iter().flatten() {
                    st.frame[row * WIDTH as usize + col as usize] = colour;
                }
            }
        }

        for s in &mut shifters {
            if s.left != 0 {
                s.a <<= 1;
                s.b <<= 1;
                s.left -= 1;
            }
        }
    }
    st.clxdat |= clx;
}

// ---------------------------------------------------------------------------
// the chip
// ---------------------------------------------------------------------------

/// Denise's register block, pixel pipeline and picture.
///
/// Shared three ways: the custom-chip bus holds it as a [`CustomChip`], Agnus
/// holds it as the [`ExportId::AMIGA_VIDEO`] handle, and a host holds it to read
/// the picture.
pub struct Video {
    standard: Standard,
    /// LEAF: nothing is called while it is held. The beam is asked *before*
    /// taking it, because asking may make Agnus push lines in here.
    state: Mutex<State>,
    beam: Mutex<Option<Arc<dyn Beam>>>,
    /// Weak, because the bus holds this chip and a strong reference back would
    /// be a cycle neither could ever free.
    bus: Mutex<Weak<CustomBus>>,
}

impl fmt::Debug for Video {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Video")
            .field("standard", &self.standard)
            .field("state", &*self.state.lock())
            .field("beam", &self.beam.lock().is_some())
            .finish_non_exhaustive()
    }
}

impl Video {
    /// A chip at power-on for `standard`.
    #[must_use]
    pub fn new(standard: Standard) -> Video {
        Video {
            standard,
            state: Mutex::with_rank(LockRank::LEAF, State::new(standard)),
            beam: Mutex::with_rank(LockRank::LEAF, None),
            bus: Mutex::with_rank(LockRank::LEAF, Weak::new()),
        }
    }

    /// The standard the picture is laid out for.
    #[must_use]
    pub fn standard(&self) -> Standard {
        self.standard
    }

    /// The picture's size: `(WIDTH, standard.height())`.
    #[must_use]
    pub fn geometry(&self) -> (u32, u32) {
        (WIDTH, self.standard.height())
    }

    /// Give Denise the beam counter, so a mid-line write lands mid-line.
    pub fn connect_beam(&self, beam: Arc<dyn Beam>) {
        *self.beam.lock() = Some(beam);
    }

    /// Render one line. See the module documentation for the contract.
    pub fn line(&self, line: &Line<'_>) {
        let mut st = self.state.lock();
        render(self.standard, &mut st, line);
    }

    /// A new field begins; `lof` is its long-frame bit (`VPOSR` bit 15).
    pub fn field(&self, lof: bool) {
        let mut st = self.state.lock();
        st.fields = st.fields.wrapping_add(1);
        st.last_field_clocks = st.clocks;
        st.clocks = 0;
        st.lof = lof;
    }

    /// Fields begun since reset.
    #[must_use]
    pub fn fields(&self) -> u64 {
        self.state.lock().fields
    }

    /// The last complete field's length in colour clocks, or zero before one
    /// has completed.
    #[must_use]
    pub fn field_clocks(&self) -> u64 {
        self.state.lock().last_field_clocks
    }

    /// Row `y` of the picture as 12-bit `0RGB` words. No side effects.
    ///
    /// `dst` is filled as far as it and the row go; a `y` past the bottom
    /// leaves it alone.
    pub fn read_row(&self, y: u32, dst: &mut [u16]) {
        if y >= self.standard.height() {
            return;
        }
        let st = self.state.lock();
        let at = y as usize * WIDTH as usize;
        let n = dst.len().min(WIDTH as usize);
        dst[..n].copy_from_slice(&st.frame[at..at + n]);
    }

    /// The collision register as it stands, without clearing it.
    #[must_use]
    pub fn peek_clxdat(&self) -> u16 {
        self.state.lock().clxdat
    }

    fn reset(&self) {
        let mut st = self.state.lock();
        *st = State::new(self.standard);
    }

    /// Where the beam is now, if Denise has been given one. Called with no
    /// lock held.
    fn now(&self) -> Option<BeamPosition> {
        let beam = self.beam.lock().clone();
        beam.map(|b| b.position())
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let st = self.state.lock();
        st.regs.save(w)?;
        w.write_u16(st.clxdat)?;
        w.write_u16(st.joy[0])?;
        w.write_u16(st.joy[1])?;
        w.write_u16(st.dmacon)?;
        w.write_u64(st.fields)?;
        w.write_bool(st.lof)?;
        w.write_u64(st.clocks)?;
        w.write_u64(st.last_field_clocks)?;
        w.write_seq_len(st.pending.len() as u64)?;
        for c in &st.pending {
            w.write_u64(c.at.field)?;
            w.write_u16(c.at.vpos)?;
            w.write_u16(c.at.hpos)?;
            w.write_u16(c.offset)?;
            w.write_u16(c.value)?;
        }
        // The picture is architectural state in the sense `dev::lcd::panel`
        // argues: nothing else saves it, and an interlaced frame keeps the
        // other field's rows from before the snapshot.
        let mut bytes = Vec::with_capacity(st.frame.len() * 2);
        for px in &st.frame {
            bytes.extend_from_slice(&px.to_le_bytes());
        }
        w.write_bytes(&bytes)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let regs = Regs::load(r)?;
        let clxdat = r.read_u16()?;
        let joy = [r.read_u16()?, r.read_u16()?];
        let dmacon = r.read_u16()?;
        let fields = r.read_u64()?;
        let lof = r.read_bool()?;
        let clocks = r.read_u64()?;
        let last_field_clocks = r.read_u64()?;
        let count = r.read_seq_len(16)?;
        if count > MAX_PENDING as u64 {
            return Err(Error::State(alloc::format!(
                "{CLASS_NAME}: {count} queued register changes, more than the {MAX_PENDING} a \
                 running chip ever holds"
            )));
        }
        let mut pending = VecDeque::with_capacity(count as usize);
        for _ in 0..count {
            let at = Stamp {
                field: r.read_u64()?,
                vpos: r.read_u16()?,
                hpos: r.read_u16()?,
            };
            pending.push_back(Change {
                at,
                offset: r.read_u16()?,
                value: r.read_u16()?,
            });
        }
        let bytes = r.read_bytes()?;
        let expected = WIDTH as usize * self.standard.height() as usize;
        if bytes.len() != expected * 2 {
            return Err(Error::State(alloc::format!(
                "{CLASS_NAME}: the snapshot's picture is {} bytes and a {:?} Denise's is {}",
                bytes.len(),
                self.standard,
                expected * 2
            )));
        }
        let frame = bytes
            .as_chunks::<2>()
            .0
            .iter()
            .map(|b| u16::from_le_bytes(*b) & 0x0fff)
            .collect();

        let mut st = self.state.lock();
        *st = State {
            regs,
            pending,
            clxdat,
            joy,
            dmacon,
            fields,
            lof,
            clocks,
            last_field_clocks,
            frame,
        };
        Ok(())
    }
}

impl CustomChip for Video {
    fn which(&self) -> ChipId {
        ChipId::DENISE
    }

    fn read(&self, reg: &Reg, from: Origin) -> u16 {
        match reg.offset {
            CLXDAT => {
                let mut st = self.state.lock();
                let value = st.clxdat & 0x7fff;
                // "its contents are automatically cleared to 0 after it is
                // read" (Chapter 7) — but not by a debugger, which must be able
                // to look without changing what the guest will see.
                if !from.debug {
                    st.clxdat = 0;
                }
                value
            }
            JOY0DAT => self.state.lock().joy[0],
            JOY1DAT => self.state.lock().joy[1],
            DENISEID => {
                // "The original Denise (8362) does not have this register, so
                // whatever value is left over on the bus from the last cycle
                // will be there" (Appendix C).
                let bus = self.bus.lock().upgrade();
                bus.map_or(0, |b| b.floating())
            }
            _ => 0,
        }
    }

    fn write(&self, reg: &Reg, value: u16, from: Origin) {
        let offset = reg.offset;
        if Regs::timed(offset) {
            // Ask the beam first, with nothing locked: asking may make Agnus
            // catch up and push lines into this very chip.
            let now = if from.debug { None } else { self.now() };
            let mut st = self.state.lock();
            match now {
                None => {
                    // No beam source (or a debugger): the change is in force
                    // from the next line rendered. Anything already queued
                    // goes first so the order of writes is kept.
                    let queued: Vec<Change> = st.pending.drain(..).collect();
                    for c in queued {
                        st.regs.apply(c.offset, c.value);
                    }
                    st.regs.apply(offset, value);
                }
                Some(pos) => {
                    let at = Stamp {
                        field: st.fields,
                        vpos: pos.vpos,
                        hpos: pos.hpos,
                    };
                    st.pending.push_back(Change { at, offset, value });
                    while st.pending.len() > MAX_PENDING {
                        if let Some(c) = st.pending.pop_front() {
                            st.regs.apply(c.offset, c.value);
                        }
                    }
                }
            }
            return;
        }
        match offset {
            JOYTEST => {
                // Appendix A, JOYTEST: Y7-Y2 and X7-X2 of all four counters;
                // the two low bits of each are the clock pins and stay.
                let mut st = self.state.lock();
                for counter in &mut st.joy {
                    *counter = (*counter & 0x0303) | (value & 0xfcfc);
                }
            }
            DMACON => {
                // Table 7-6: bit 15 says whether the others set or clear.
                let mut st = self.state.lock();
                let bits = value & 0x07ff;
                st.dmacon = if value & 0x8000 != 0 {
                    st.dmacon | bits
                } else {
                    st.dmacon & !bits
                };
            }
            // The strobes, and anything else Appendix B gives Denise that this
            // model has no behaviour for.
            _ => {}
        }
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// The `amiga.denise` device.
#[derive(Debug)]
pub struct Denise {
    video: Arc<Video>,
    custom: String,
}

impl Denise {
    /// A Denise with the given properties.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `custom` is missing, `standard` is not `"pal"`
    /// or `"ntsc"`, or a property nothing here accepts was given.
    pub fn new(props: &Props) -> Result<Denise> {
        let mut r = props.reader();
        let custom = r.require_link("custom")?.as_str().to_string();
        let standard = match r.or_enum("standard", "pal", &["pal", "ntsc"])? {
            "ntsc" => Standard::Ntsc,
            _ => Standard::Pal,
        };
        r.finish()?;
        Ok(Denise {
            video: Arc::new(Video::new(standard)),
            custom,
        })
    }

    /// The chip, for a host that wants the picture or a test standing in for
    /// Agnus.
    #[must_use]
    pub fn video(&self) -> &Arc<Video> {
        &self.video
    }
}

impl Device for Denise {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the register subscription is made from `bind`.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // "LPEN", "LACE" and "ERSY" are "reset on power up" (Appendix A,
        // BPLCON0); nothing else is said, and nothing in Denise is
        // battery-backed, so both kinds clear everything.
        self.video.reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        self.video.save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        self.video.load(r)
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::AMIGA_VIDEO).then(|| {
            Export::Opaque(Arc::clone(&self.video) as Arc<dyn core::any::Any + Send + Sync>)
        })
    }
}

impl Instance for Denise {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let bus =
            ctx.export_as::<CustomBus>(&self.custom, crate::core::device::ExportId::CUSTOM_BUS)?;
        bus.attach(Arc::clone(&self.video) as Arc<dyn CustomChip>)?;
        *self.video.bus.lock() = Arc::downgrade(&bus);
        Ok(())
    }
}

/// The `amiga.denise` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the Amiga's Denise video chip: colour table, playfields, sprites, collisions",
    properties: &[
        PropertySpec {
            name: "custom",
            kind: ValueKind::Link,
            required: true,
            summary: "the `amiga.custom` register space to subscribe to",
        },
        PropertySpec {
            name: "standard",
            kind: ValueKind::Str,
            required: false,
            summary: "`pal` (default) or `ntsc`: how many lines the picture has",
        },
    ],
    construct: |props| Ok(Box::new(Denise::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Denise::new(props)?)))
}

/// What the validator should know about `amiga.denise`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("custom", ValueKind::Link).required())
        .prop(PropSchema::new("standard", ValueKind::Str))
}

#[cfg(test)]
mod tests;
