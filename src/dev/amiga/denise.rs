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
//! by Agnus (`amiga.agnus`, `video = denise`), and a test may drive it from a
//! synthetic source instead.
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
//! That is the picture a hardwired beam gives. A programmable one — an ECS
//! Agnus's — hands over a [`Raster`] with every field ([`Video::raster`]), and
//! the field is laid out by it instead: its lines from the end of vertical
//! blanking, its columns between the horizontal blanks, and one row a line
//! rather than two for a 31 kHz line ([`DOUBLED_LINE`]). And while an 8373
//! has SuperHires on screen, its columns are SuperHires pixels, four to a
//! low-resolution one; the first SuperHires line of a field widens the picture
//! there and then, and a field with none narrows it back. So the host's
//! geometry — [`Video::geometry`] — is the field's, and can change at any
//! field; `host::display::amiga` reads picture and size together with
//! [`Video::copy_frame`].
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
//! | `JOY0DAT`, `JOY1DAT`, `JOYTEST` | modelled: four 8-bit counters clocked by quadrature on the eight mouse pins, low two bits the pins' state; see [below](#the-mouse-counters) |
//! | `DMACON` | latched only — the manual does not say what Denise does with it |
//! | `BPL1DAT`–`BPL6DAT` | latched only; the bitplane words arrive through [`Video::line`] |
//! | `STREQU`, `STRVBL`, `STRHOR`, `STRLONG` | accepted and ignored; [`Video::line`] and [`Video::field`] carry what they mean |
//! | `BPLCON3`, `DIWHIGH`, `DENISEID` | ECS registers. On an 8362 (`revision = "ocs"`, the default) the first two latch and `DENISEID` is not driven at all ("The original Denise (8362) does not have this register", Appendix C), so the bus answers off the chip data lines and leaves them floating — which is what makes Kickstart's seventeen-read stability test come out OCS. On an 8373 (`revision = "ecs"`) see the next rows |
//! | `DENISEID` (8373) | [`ECS_DENISEID`]: "$FC in the lower 8 bits" |
//! | `BPLCON0` (8373) | `SHRES` modelled: SuperHires, two pixels to a high-resolution one, colours through Appendix C's register encoding (`shr_colour`); `ENBPLCN3` modelled; `BPLHWRM`, `SPRHWRM` latched only |
//! | `BPLCON2` (8373) | `KILLEHB` modelled; `ZDBPSEL`, `ZDBPEN`, `ZDCTEN` latched only — genlock, and there is no genlock |
//! | `BPLCON3` (8373) | `BRDRBLNK` modelled: a black border; `BRDNTRAN` latched only, genlock again |
//! | `DIWHIGH` (8373) | modelled: the window's `H8` and `V10`–`V8` directly, once written after `DIWSTRT`/`DIWSTOP` |
//! | `SPRxCTL` (8373) | `SHSH1` modelled: a sprite half a low-resolution pixel later in SuperHires; `SHSH0` is "unimplemented" in the manual too |
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
//! * **Where a SuperHires word is first shown.** Chapter 3's fetch arithmetic
//!   puts the first pixel one fetch block and half a count after the fetch:
//!   `2D + 17` in low resolution, `2D + 9` in high. SuperHires fetches a block
//!   every two counts, so `2D + 5`. The manual gives no SuperHires formula.
//! * **Which of a SuperHires pair takes a register's top bits** (see
//!   `shr_colour`), and that a two-bit gun is shown repeated into four.
//! * **SuperHires with dual playfields**, which Appendix C calls
//!   "compatible": decoded as one playfield of the first two planes.
//!
//! # The mouse counters
//!
//! Appendix J gives Denise four mouse pins, `M0V`, `M0H`, `M1V` and `M1H`, and
//! Appendix A's `JOY0DAT` entry says what is on them: each carries two
//! connector pins "sampled (multiplexed) into the DENISE chip" at `CCK` and
//! `CCK*` — pin 1 (`FORW*`, `Y`) and pin 3 (`LEFT*`, `YQ`) on `M0V`, pin 2
//! (`BACK*`, `X`) and pin 4 (`RIGH*`, `XQ`) on `M0H`. The model does not
//! multiplex: it has the eight signals as eight input pins, `m0v`, `m0vq`,
//! `m0h`, `m0hq` and the same for port 1, each carrying its connector level.
//!
//! "After being sampled, these connector pin signals are used in quadrature to
//! clock the mouse counters. The LEFT and RIGHT joystick functions (active
//! high) are directly available on the Y1 and X1 bits … FORWARD … Y1 xor Y0".
//! With the connector active low, that fixes a counter's low two bits as
//! `(!Q, pin xor Q)`, which is the Gray-coded quadrature phase in binary. So a
//! transition is counted by comparing the phase the pins now show with the
//! counter's own low bits: one step up or one step down. A difference of two —
//! both signals moving between samples, which one wire at a time never does —
//! is counted as two up, because nothing says which way it went.
//!
//! Counting against the counter rather than against the last pins seen is what
//! lets a snapshot restore the counter without the pins' history: whatever
//! order the far end re-drives its pins in, the count comes back to the saved
//! value.
//!
//! # The AA chip set: Lisa
//!
//! `revision = "aga"` is **Lisa**, the A1200's and A4000's video chip. The
//! display half is modelled here — eight bitplanes, the 256-entry 24-bit
//! colour table with `BANK` and `LOCT`, HAM8, `BPLCON4`, the 35 ns scroll,
//! window and sprite positions, `SPRES`, 16/32/64-bit sprites and `CLXCON2`
//! — and `denise/aga.rs` has what the *Specification for the Advanced Amiga (AA) Chip
//! Set* settles, what it leaves open and what was chosen there. An 8362 and an
//! 8373 do not go near that module, and everything above this section is
//! still exactly what they do.
//!
//! What Lisa shares with the older parts is the seam: the same [`Line`] a
//! line, the same register writes, the same [`Beam`]. What she adds to it is
//! two things, both documented where they are declared:
//!
//! * [`Fetch::planes`] carries **eight** streams. A 32- or 64-bit `FMODE`
//!   needs nothing more — a wider fetch is more words in the same stream.
//! * [`Video::sprite_dma`] takes a sprite-data transfer too wide for the
//!   sixteen-bit register bus.
//!
//! The picture is **eight bits a gun** for every part ([`Video::copy_frame_rgb`]),
//! because Lisa's is; an 8362's and an 8373's guns reach it as `n × 17`, the
//! expansion the host adapter always made, so their pictures are unchanged
//! byte for byte and [`Video::read_row`] and [`Video::copy_frame`] still hand
//! out the twelve-bit words they always did.
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
//! Appendix J for Denise's pins. For Lisa, the *Specification for the Advanced
//! Amiga (AA) Chip Set* (Commodore-Amiga), whose sections `denise/aga.rs`
//! cites. **No
//! emulator source of any licence was consulted** (`ROADMAP.md` §1): every
//! Amiga emulator the author is aware of is GPL, and AROS is MPL-derived; nor
//! was any FPGA reimplementation of the chip set.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

use super::custom::{CustomBus, CustomChip, Origin};
use super::regs::{ChipId, Reg};

mod aga;

/// The class name a machine file writes.
pub const CLASS_NAME: &str = "amiga.denise";

/// Snapshot version for this class's chunk encoding.
const STATE_VERSION: u32 = 3;

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
/// between the hardware limits, and an 8373's SuperHires twice high
/// resolution's (Appendix C). A few more are accepted and ignored past the
/// line's end.
pub const MAX_FETCH_WORDS: usize = 128;

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

/// The eight mouse inputs, in line order: for port 0 then port 1, the
/// horizontal pair and then the vertical pair, each signal before its
/// quadrature partner. See [the mouse counters](self#the-mouse-counters).
pub const MOUSE_PINS: [&str; 8] = ["m0h", "m0hq", "m0v", "m0vq", "m1h", "m1hq", "m1v", "m1vq"];

/// A counter's low two bits for a pair of connector levels: `(!Q, pin xor Q)`.
#[inline]
#[must_use]
pub const fn quadrature_bits(pin_high: bool, q_high: bool) -> u8 {
    ((!q_high) as u8) << 1 | (pin_high ^ q_high) as u8
}
const DENISEID: u16 = 0x07c;
const DIWSTRT: u16 = 0x08e;
const DIWSTOP: u16 = 0x090;
const DMACON: u16 = 0x096;
const CLXCON: u16 = 0x098;
const BPLCON0: u16 = 0x100;
const BPLCON1: u16 = 0x102;
const BPLCON2: u16 = 0x104;
const BPLCON3: u16 = 0x106;
/// `BPLCON4`, "Bit plane control reg. (display masks)": AA only (AA
/// specification, §4, `BPLCON4`).
const BPLCON4: u16 = 0x10c;
/// `CLXCON2`, "Extended collision control": AA only. The specification's own
/// page for it misprints the address as `$10C`; its register list has `$10e`.
const CLXCON2: u16 = 0x10e;
const BPL1DAT: u16 = 0x110;
const BPL6DAT: u16 = 0x11a;
/// `BPL7DAT`: bitplane 7's parallel-to-serial buffer, AA only.
const BPL7DAT: u16 = 0x11c;
/// `BPL8DAT`: the last of the eight parallel-to-serial buffers an AA part has
/// (AA specification, §4, `BPLxDAT`).
const BPL8DAT: u16 = 0x11e;
const SPR0POS: u16 = 0x140;
const SPR7DATB: u16 = 0x17e;
const COLOR00: u16 = 0x180;
const COLOR31: u16 = 0x1be;
const DIWHIGH: u16 = 0x1e4;
/// `FMODE`, "Memory Fetch Mode": AA only, and Alice's register as much as
/// Lisa's (AA specification, §4, `FMODE`).
const FMODE: u16 = 0x1fc;

// BPLCON0 bits, Appendix A.
const HIRES: u16 = 1 << 15;
const HOMOD: u16 = 1 << 11;
const DBLPF: u16 = 1 << 10;
const LACE: u16 = 1 << 2;
// BPLCON0's ECS bits, Appendix C: "SHRES SuperHires 35ns pixel enable bit"
// and "ENBPLCN3 Enable new BLPCON3 register".
const SHRES: u16 = 1 << 6;
const ENBPLCN3: u16 = 1 << 0;
/// `BPLCON2` bit 9 on an ECS part: "KILLEHB Kill halfbrite" (Appendix C,
/// *Genlock Extensions*).
const KILLEHB: u16 = 1 << 9;
/// `BPLCON3` bit 5: "BRDRBLNK Border blank".
const BRDRBLNK: u16 = 1 << 5;
/// `SPRxCTL` bit 4 on an ECS part: "SHSH1 Start horizontal (SHR mode) 70ns
/// increment" (Appendix C, *SuperHires 70ns Sprite Positioning*).
const SHSH1: u16 = 1 << 4;

/// `SPRxCTL` bit 7, `ATT`: "Sprite attach control bit (odd sprites)".
const ATTACH: u16 = 1 << 7;

// ---------------------------------------------------------------------------
// AA bit definitions — *Specification for the Advanced Amiga (AA) Chip Set*
// (Commodore-Amiga), §4, the per-register pages. Nothing below is reachable
// from an 8362 or an 8373.
// ---------------------------------------------------------------------------

/// `BPLCON0` bit 4, `BPU3`: the fourth bitplane-use bit, so `BPU` counts
/// "0000-1000 (none thru 8 inclusive)" (AA specification, `BPLCON0`).
const BPU3: u16 = 1 << 4;

/// `BPLCON3` bit 9, `LOCT`: "Dictates that subsequent color palette values
/// will be written to a second 12-bit color palette, constituting the RGB low
/// order bits".
const LOCT: u16 = 1 << 9;

/// `BPLCON3` bit 1, `BRDSPRT`: "Enables sprites outside the display window.
/// disabled when ECSENA low."
const BRDSPRT: u16 = 1 << 1;

/// `FMODE` bit 15, `SSCAN2`: "Global enable for sprite scan-doubling."
const SSCAN2: u16 = 1 << 15;

/// What an ECS Denise answers at `DENISEID`: "The enhanced HighRes Denise
/// (8373) will return $FC in the lower 8 bits. The upper 8 bits are reserved"
/// (Appendix C, *Determining Chip Revisions*). **Choice:** the reserved byte
/// reads as ones.
pub const ECS_DENISEID: u16 = 0xfffc;

/// What an AA part answers at `DENISEID`, which the AA specification calls
/// `LISAID`: "Lisa returns hex (f8). The upper 8 bits of this [register are
/// reserved]" (AA specification, §4, `LISAID`). **Choice:** the reserved byte
/// reads as ones, as for [`ECS_DENISEID`].
pub const LISA_ID: u16 = 0xfff8;

/// A line at least this long is a 15 kHz line, drawn twice when not
/// interlaced so the picture keeps its shape; a shorter one — productivity
/// mode's 114 counts — is drawn once. Midway between the two families.
pub const DOUBLED_LINE: u16 = 170;

/// The most lines a picture holds, so a programmed `VTOTAL` cannot ask for a
/// frame buffer of absurd size. Twice the tallest real field.
pub const MAX_LINES: u16 = 1024;

/// Which Denise this is.
///
/// A closed set, so a real enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Revision {
    /// The original 8362.
    Ocs,
    /// The Enhanced Chip Set's 8373: `DENISEID`, `BPLCON3`, `DIWHIGH`,
    /// SuperHires, 70 ns sprite positions and `KILLEHB`. Its picture is laid
    /// out in SuperHires pixels, four to a low-resolution one.
    Ecs,
    /// The AA chip set's **Lisa**: eight bitplanes, a 256-entry 24-bit colour
    /// table, HAM8, `FMODE`, `BPLCON4` and 35 ns positioning throughout. Its
    /// picture is always laid out in 35 ns columns, four to a low-resolution
    /// pixel. See [the AA section](self#the-aa-chip-set-lisa).
    Aga,
}

impl Revision {
    /// Whether this part has the AA chip set's behaviour.
    #[must_use]
    #[inline]
    pub const fn is_aga(self) -> bool {
        matches!(self, Revision::Aga)
    }

    /// Whether this part has at least the Enhanced Chip Set's registers —
    /// `BPLCON3`, `DIWHIGH` and a driven `DENISEID`. Lisa has all of them.
    #[must_use]
    #[inline]
    pub const fn is_ecs(self) -> bool {
        matches!(self, Revision::Ecs | Revision::Aga)
    }
}

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
    /// The words fetched for bitplanes 1–8, in fetch order, leftmost first.
    ///
    /// A plane `BPLCON0` does not enable is ignored whatever is here, and so is
    /// anything past [`MAX_FETCH_WORDS`]. An empty slice is a plane with no data
    /// on this line: its pixels are zero.
    ///
    /// Planes 7 and 8 are Lisa's, and an 8362 or 8373 ignores them whatever
    /// `BPU` says. **A wider `FMODE` needs nothing new here**: a fetch of 16,
    /// 32 or 64 bits is that many consecutive pixels, most significant bit
    /// first — "the parallel to serial conversion is triggered whenever bit
    /// plane #1 is written, indicating the completion of all bit planes for
    /// that word (16/32/64 pixels). The MSB is output first, and is therefore
    /// always on the left" (AA specification, §4, `BPLxDAT`). So the wider
    /// fetch is the same word stream with more words per slot, and Alice puts
    /// each fetch's words here in the order the chip shifts them out.
    pub planes: [&'a [u16]; 8],
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

/// The part of the beam's raster a monitor shows, as the chip that counts the
/// beam reports it.
///
/// An original chip-set Agnus never reports one, and Denise lays its picture
/// out from its [`Standard`] with [`Raster::standard`]. An ECS Agnus, whose
/// beam is programmable, hands one over with [`Video::raster`] before every
/// [`Video::field`], and the new field is laid out by it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Raster {
    /// The picture's first line: the end of vertical blanking.
    pub first_line: u16,
    /// How many lines the picture has.
    pub lines: u16,
    /// The beam count of the picture's first column.
    pub first_clock: u16,
    /// How many colour clocks across the picture is.
    pub clocks: u16,
    /// A short line's length in colour clocks, which says whether the lines
    /// are 15 kHz ones or 31 kHz ones ([`DOUBLED_LINE`]).
    pub line_clocks: u16,
}

impl Raster {
    /// The original chip set's picture for `std`: from Table 3-13's end of
    /// vertical blank to the end of a long field, and [`WIDTH`] high-resolution
    /// pixels from [`OUTPUT_LEFT`].
    #[must_use]
    pub const fn standard(std: Standard) -> Raster {
        Raster {
            first_line: std.first_line(),
            lines: std.lines(),
            first_clock: OUTPUT_LEFT / 2,
            clocks: (WIDTH / 4) as u16,
            line_clocks: 227,
        }
    }

    /// The same raster with its size held to what a frame buffer can be.
    #[must_use]
    const fn bounded(self) -> Raster {
        let clocks = if self.clocks == 0 {
            1
        } else if self.clocks > 256 {
            256
        } else {
            self.clocks
        };
        let lines = if self.lines == 0 {
            1
        } else if self.lines > MAX_LINES {
            MAX_LINES
        } else {
            self.lines
        };
        Raster {
            lines,
            clocks,
            ..self
        }
    }
}

/// How a field's picture is laid out: which lines and columns of the beam it
/// shows, and at what size.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Layout {
    first_line: u16,
    lines: u16,
    /// The low-resolution `x` of the first column.
    left: u16,
    /// Low-resolution pixels across.
    span: u16,
    /// Columns per low-resolution pixel: two, a high-resolution pixel each,
    /// or four, a SuperHires pixel each, while an 8373 has SuperHires on
    /// screen.
    scale: u16,
    /// Rows per line: two when line-doubled or interlaced, one for a 31 kHz
    /// line drawn as it is.
    rows: u16,
}

impl Layout {
    /// `superhires` is whether the picture needs SuperHires columns.
    fn of(raster: Raster, superhires: bool, lace: bool) -> Layout {
        let raster = raster.bounded();
        Layout {
            first_line: raster.first_line,
            lines: raster.lines,
            left: 2 * raster.first_clock,
            span: 2 * raster.clocks,
            scale: if superhires { 4 } else { 2 },
            rows: if raster.line_clocks >= DOUBLED_LINE || lace {
                2
            } else {
                1
            },
        }
    }

    fn width(&self) -> u32 {
        u32::from(self.span) * u32::from(self.scale)
    }

    fn height(&self) -> u32 {
        u32::from(self.lines) * u32::from(self.rows)
    }
}

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------

/// Every register that shapes the picture, as of the pixel being drawn.
///
/// `color` is the 32 colour registers as an 8362 or an 8373 holds them —
/// twelve bits, four a gun. `palette` is Lisa's 256-entry table, which the
/// same 32 addresses reach a bank at a time; only an AA part maintains it, and
/// only an AA part reads it.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Regs {
    color: [u16; 32],
    /// Lisa's colour table: 256 entries of `0x00RR_GGBB`, with the genlock
    /// `T` bit in bit 24 (AA specification, §4, `COLORx`).
    palette: [u32; 256],
    bplcon0: u16,
    bplcon1: u16,
    bplcon2: u16,
    bplcon3: u16,
    /// `BPLCON4`: `BPLAM` in the high byte, `ESPRM` and `OSPRM` in the low.
    /// AA only.
    bplcon4: u16,
    /// `CLXCON2`: bitplanes 7 and 8's collision enables and match values.
    /// AA only.
    clxcon2: u16,
    /// `FMODE`: the bitplane and sprite fetch widths and the scan-double
    /// enables. AA only, and Alice's register too.
    fmode: u16,
    diwstrt: u16,
    diwstop: u16,
    diwhigh: u16,
    /// `DIWHIGH` was written after the last `DIWSTRT` or `DIWSTOP`: on an
    /// 8373 its top bits are in force (Appendix C, *Display Window
    /// Specification*: "If this register is written last in a sequence of
    /// setting the display window, it sets direct start and stop positions").
    diwhigh_on: bool,
    clxcon: u16,
    bpldat: [u16; 8],
    spr_pos: [u16; 8],
    spr_ctl: [u16; 8],
    /// The A buffers. Sixty-four bits wide because `FMODE`'s `SPR32` and
    /// `SPAGEM` make an AA sprite fetch 16, 32 or 64 bits ("Sprites are
    /// either 16, 32, or 64 bits wide", AA specification, §5). A 16-bit write
    /// through the register bus lands in the top of the word, which is where
    /// the shifter starts: "MSB first on the left".
    spr_data: [u64; 8],
    /// The B buffers, the same width.
    spr_datb: [u64; 8],
    /// One bit per sprite: its horizontal comparator is enabled.
    armed: u8,
}

impl Default for Regs {
    /// Everything zero — a hand-written `Default` only because `[u32; 256]`
    /// has none. An AA part's non-zero reset values are put on afterwards by
    /// [`Regs::power_on`]; an 8362 and an 8373 start at zero as they always
    /// did.
    fn default() -> Regs {
        Regs {
            color: [0; 32],
            palette: [0; 256],
            bplcon0: 0,
            bplcon1: 0,
            bplcon2: 0,
            bplcon3: 0,
            bplcon4: 0,
            clxcon2: 0,
            fmode: 0,
            diwstrt: 0,
            diwstop: 0,
            diwhigh: 0,
            diwhigh_on: false,
            clxcon: 0,
            bpldat: [0; 8],
            spr_pos: [0; 8],
            spr_ctl: [0; 8],
            spr_data: [0; 8],
            spr_datb: [0; 8],
            armed: 0,
        }
    }
}

/// `BPLCON3`'s AA reset value: `PF2OF1 = PF2OF0 = 1`, playfield 2's colour
/// offset 8 — which is where an 8362's second playfield already is, so an old
/// copper list that never writes `BPLCON3` keeps its colours.
const PF2OF_DEFAULT: u16 = 0b011 << 10;

/// `BPLCON4`'s AA reset value: `ESPRM4 = OSPRM4 = 1`, so a sprite's colours
/// are at 16–31 — again the 8362's fixed behaviour, made the default on
/// purpose.
const SPRM_DEFAULT: u16 = 0x11;

impl Regs {
    /// The register file an AA part comes out of reset with.
    ///
    /// "A RST_input pin has been added, which resets all the bits contained in
    /// registers that were new for ECS or LISA" (AA specification, §1), and
    /// the per-register pages print each new field's reset value beside it.
    /// The two that are not zero are [`PF2OF_DEFAULT`] and [`SPRM_DEFAULT`].
    fn power_on(rev: Revision) -> Regs {
        let mut regs = Regs::default();
        if rev.is_aga() {
            regs.bplcon3 = PF2OF_DEFAULT;
            regs.bplcon4 = SPRM_DEFAULT;
        }
        regs
    }

    /// Whether a write to `offset` changes what a pixel looks like, and so has
    /// to land at a beam position rather than at once.
    ///
    /// The AA-only registers are timed on an AA part and nothing at all on the
    /// older two, which have no such registers: the address map declares them
    /// because the decode is the map's, but an 8362 and an 8373 drop the
    /// write the way they drop every other address they own no behaviour at.
    fn timed(rev: Revision, offset: u16) -> bool {
        let aga_only = matches!(offset, BPLCON4 | CLXCON2 | FMODE | BPL7DAT..=BPL8DAT);
        if aga_only {
            return rev.is_aga();
        }
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

    /// Apply one register write. `rev` decides only what a `COLORxx` write
    /// means: an 8362 or an 8373 has one twelve-bit register per address,
    /// Lisa has a 256-entry table those addresses reach a bank at a time.
    fn apply(&mut self, rev: Revision, offset: u16, value: u16) {
        match offset {
            COLOR00..=COLOR31 if rev.is_aga() => self.write_palette(offset, value),
            COLOR00..=COLOR31 => {
                // "Bits 15 - 12 Unused" (Table 3-3): Denise has twelve RGB
                // pins and nothing to put the top nibble on.
                self.color[usize::from((offset - COLOR00) / 2)] = value & 0x0fff;
            }
            BPLCON0 => self.bplcon0 = value,
            BPLCON1 => self.bplcon1 = value,
            BPLCON2 => self.bplcon2 = value,
            BPLCON3 => self.bplcon3 = value,
            BPLCON4 => self.bplcon4 = value,
            CLXCON2 => self.clxcon2 = value,
            FMODE => self.fmode = value,
            DIWSTRT => {
                self.diwstrt = value;
                self.diwhigh_on = false;
            }
            DIWSTOP => {
                self.diwstop = value;
                self.diwhigh_on = false;
            }
            DIWHIGH => {
                self.diwhigh = value;
                self.diwhigh_on = true;
            }
            CLXCON => {
                self.clxcon = value;
                // "Contents of this register are reset by a write to CLXCON"
                // (AA specification, §4, `CLXCON2`), "so that old game
                // programs will be able to correctly detect collisions"
                // (§2, *Compatibility*).
                self.clxcon2 = 0;
            }
            BPL1DAT..=BPL8DAT => self.bpldat[usize::from((offset - BPL1DAT) / 2)] = value,
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
                        // A word through the register bus is the top of the
                        // buffer, because the shifter runs from the most
                        // significant bit: see `spr_data`.
                        self.spr_data[i] = u64::from(value) << 48;
                        self.armed |= 1 << i;
                    }
                    _ => self.spr_datb[i] = u64::from(value) << 48,
                }
            }
            _ => {}
        }
    }

    /// A whole sprite buffer from a wide transfer: `offset` is the `SPRxDATA`
    /// or `SPRxDATB` it loads, and `bits` the data, first pixel in bit 63.
    /// Loading the A buffer arms the sprite, as a register write to it does.
    fn load_sprite(&mut self, offset: u16, bits: u64) {
        let rel = offset.wrapping_sub(SPR0POS);
        let i = usize::from(rel / 8);
        match rel % 8 {
            4 if i < 8 => {
                self.spr_data[i] = bits;
                self.armed |= 1 << i;
            }
            6 if i < 8 => self.spr_datb[i] = bits,
            _ => {}
        }
    }

    /// One `COLORxx` write to Lisa's 256-entry table.
    ///
    /// "There are 32 of these registers (xx=00-31) and together with the
    /// banking bits they address the 256 locations in the color palette …
    /// When LOCT = 0 the 4 MSB of red, green and blue video data are selected
    /// along with the T bit for genlocks[;] the low order set of registers is
    /// also selected as well, so that the 4 bit values are automatically
    /// extended to 8 bits. This provides compatibility with old software. If
    /// the full range of palette values are desired, then LOCT can be set high
    /// and independant values for the 4 LSB of red, green and blue can be
    /// written. The low order color registers do not contain a transparency
    /// (T) bit." (AA specification, §4, `COLORx`.)
    ///
    /// So a `LOCT = 0` write of nibble `n` to a gun gives `n × 17`, which is
    /// the same expansion [`crate::host::display::amiga::rgb12_to_rgb888`]
    /// makes for an 8362 — an AA part loaded from an old copper list shows
    /// exactly the 8362's colours.
    fn write_palette(&mut self, offset: u16, value: u16) {
        // "BANK2,1,0 [select one] of 8 32 address banks", bits 15-13.
        let bank = usize::from(self.bplcon3 >> 13);
        let at = bank * 32 + usize::from((offset - COLOR00) / 2);
        let entry = &mut self.palette[at];
        let mut out = 0u32;
        if self.bplcon3 & LOCT != 0 {
            for shift in [16, 8, 0] {
                let nibble = u32::from((value >> (shift / 2)) & 0xf);
                out |= ((*entry >> shift) & 0xf0 | nibble) << shift;
            }
            // The low-order registers have no T bit, so the one already
            // latched stays.
            *entry = out | (*entry & (1 << 24));
        } else {
            for shift in [16, 8, 0] {
                let nibble = u32::from((value >> (shift / 2)) & 0xf);
                out |= (nibble << 4 | nibble) << shift;
            }
            *entry = out | (u32::from((value >> 15) & 1) << 24);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        for c in self.color {
            w.write_u16(c)?;
        }
        for c in self.palette {
            w.write_u32(c)?;
        }
        for v in [
            self.bplcon0,
            self.bplcon1,
            self.bplcon2,
            self.bplcon3,
            self.bplcon4,
            self.clxcon2,
            self.fmode,
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
        for table in [&self.spr_pos, &self.spr_ctl] {
            for v in table {
                w.write_u16(*v)?;
            }
        }
        for table in [&self.spr_data, &self.spr_datb] {
            for v in table {
                w.write_u64(*v)?;
            }
        }
        w.write_u8(self.armed)?;
        w.write_bool(self.diwhigh_on)
    }

    fn load(r: &mut ChunkReader<'_>) -> Result<Regs> {
        let mut regs = Regs::default();
        for c in &mut regs.color {
            *c = r.read_u16()? & 0x0fff;
        }
        for c in &mut regs.palette {
            // Twenty-four bits of colour and the genlock T bit; nothing is
            // defined above them.
            *c = r.read_u32()? & 0x01ff_ffff;
        }
        for v in [
            &mut regs.bplcon0,
            &mut regs.bplcon1,
            &mut regs.bplcon2,
            &mut regs.bplcon3,
            &mut regs.bplcon4,
            &mut regs.clxcon2,
            &mut regs.fmode,
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
        for table in [&mut regs.spr_pos, &mut regs.spr_ctl] {
            for v in table.iter_mut() {
                *v = r.read_u16()?;
            }
        }
        for table in [&mut regs.spr_data, &mut regs.spr_datb] {
            for v in table.iter_mut() {
                *v = r.read_u64()?;
            }
        }
        regs.armed = r.read_u8()?;
        regs.diwhigh_on = r.read_bool()?;
        Ok(regs)
    }
}

/// When a change happened: the field it was made in, and the beam position.
///
/// Ordered field first, so a sorted queue stays sorted across a field wrap.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord)]
struct Stamp {
    field: u64,
    vpos: u16,
    hpos: u16,
}

/// A register write waiting for its pixel.
///
/// `value` is sixty-four bits for one kind of change only: a wide sprite
/// transfer from [`Video::sprite_dma`], marked by [`WIDE`] in `offset`. Every
/// other change is a register word.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Change {
    at: Stamp,
    offset: u16,
    value: u64,
}

/// The mark in a [`Change`]'s offset for a wide sprite transfer: the rest of
/// the offset is the `SPRxDATA` or `SPRxDATB` it loads. Register offsets stop
/// at `$1FE`, so the bit is free.
const WIDE: u16 = 0x8000;

impl Change {
    /// Put this change into `regs`.
    fn apply_to(self, regs: &mut Regs, rev: Revision) {
        if self.offset & WIDE != 0 {
            regs.load_sprite(self.offset & !WIDE, self.value);
        } else {
            regs.apply(rev, self.offset, self.value as u16);
        }
    }
}

/// Everything behind the lock.
struct State {
    regs: Regs,
    pending: VecDeque<Change>,
    clxdat: u16,
    joy: [u16; 2],
    /// The eight mouse pins as last driven, bit `n` for [`MOUSE_PINS`]`[n]`,
    /// set when high. An input level: kept across a reset and a load.
    mouse_pins: u8,
    dmacon: u16,
    /// Fields begun since reset — the host's frame counter.
    fields: u64,
    /// The current field's long-frame bit.
    lof: bool,
    /// Colour clocks pushed so far in the current field.
    clocks: u64,
    /// Colour clocks in the last complete field.
    last_field_clocks: u64,
    /// The raster the current field is laid out by.
    raster: Raster,
    /// The raster the next field will be laid out by: the beam's last word.
    raster_next: Raster,
    /// The current field's layout: derived from `raster`, the revision and
    /// `LACE` at the field's start, and saved because the last cannot be
    /// derived afterwards.
    layout: Layout,
    /// A SuperHires line has been drawn in this field, so the next is laid
    /// out in SuperHires columns too.
    shres_seen: bool,
    /// The picture: `layout.width() × layout.height()` words of `0x00RR_GGBB`.
    ///
    /// Twenty-four bits because Lisa's guns are eight bits each. An 8362 and
    /// an 8373 put `n × 17` of each four-bit gun here, which is the exact
    /// expansion the host adapter used to make from the twelve-bit word, so
    /// their pictures come out byte for byte what they were —
    /// [`Video::read_row`] still hands out the twelve-bit form.
    frame: Vec<u32>,
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
    fn new(standard: Standard, rev: Revision) -> State {
        let raster = Raster::standard(standard);
        let layout = Layout::of(raster, rev.is_aga(), false);
        State {
            regs: Regs::power_on(rev),
            pending: VecDeque::new(),
            clxdat: 0,
            joy: [0; 2],
            // Nothing plugged in: every connector pin pulled up.
            mouse_pins: 0xff,
            dmacon: 0,
            fields: 0,
            lof: true,
            clocks: 0,
            last_field_clocks: 0,
            raster,
            raster_next: raster,
            layout,
            shres_seen: false,
            frame: vec![0; layout.width() as usize * layout.height() as usize],
        }
    }

    /// Lay the field that is starting out by `raster_next`: SuperHires
    /// columns if the field just ended showed any SuperHires line or
    /// `BPLCON0` asks for one now, high-resolution ones otherwise.
    ///
    /// A picture that only changes its columns is rescaled rather than
    /// cleared, and one that keeps its size keeps its pixels, so an
    /// interlaced frame still weaves; any other change starts a blank one.
    fn relayout(&mut self, rev: Revision) {
        self.raster = self.raster_next;
        // Lisa positions everything — scroll, window, sprites — on a 35 ns
        // grid in every resolution (AA specification, §2, *Horizontal
        // Comparators*), so its picture is always in 35 ns columns and never
        // rescales. An 8373 widens only while SuperHires is on screen.
        let superhires = rev.is_aga()
            || (rev == Revision::Ecs && (self.shres_seen || self.regs.bplcon0 & SHRES != 0));
        self.shres_seen = false;
        let layout = Layout::of(self.raster, superhires, self.regs.bplcon0 & LACE != 0);
        if layout == self.layout {
            return;
        }
        let same_but_columns = Layout {
            scale: self.layout.scale,
            ..layout
        } == self.layout;
        if same_but_columns {
            self.rescale(layout.scale);
            return;
        }
        self.frame = vec![0; layout.width() as usize * layout.height() as usize];
        self.layout = layout;
    }

    /// Change the picture's columns per low-resolution pixel to `scale`,
    /// keeping every row: two columns become four by repeating each, and four
    /// become two by keeping every other one.
    fn rescale(&mut self, scale: u16) {
        let old = self.layout;
        let new = Layout { scale, ..old };
        let (from, to) = (old.width() as usize, new.width() as usize);
        let mut frame = vec![0; to * new.height() as usize];
        for (dst, src) in frame
            .chunks_exact_mut(to)
            .zip(self.frame.chunks_exact(from))
        {
            for (i, px) in dst.iter_mut().enumerate() {
                *px = src[i * from / to];
            }
        }
        self.frame = frame;
        self.layout = new;
    }

    /// Count whatever the mouse pins moved: each counter against its own low
    /// two bits.
    fn clock_counters(&mut self) {
        for port in 0..2 {
            for (axis, shift) in [(0usize, 0u16), (1, 8)] {
                let base = port * 4 + axis * 2;
                let pin = self.mouse_pins & (1 << base) != 0;
                let q = self.mouse_pins & (1 << (base + 1)) != 0;
                let counter = ((self.joy[port] >> shift) & 0xff) as u8;
                let step = quadrature_bits(pin, q).wrapping_sub(counter) & 3;
                let counted = match step {
                    0 => counter,
                    3 => counter.wrapping_sub(1),
                    // One up, or both signals at once: two.
                    n => counter.wrapping_add(n),
                };
                self.joy[port] =
                    (self.joy[port] & !(0xff << shift)) | (u16::from(counted) << shift);
            }
        }
    }

    /// The counters' low bits brought into line with the pins, as at power-on.
    fn settle_counters(&mut self) {
        for port in 0..2 {
            for (axis, shift) in [(0usize, 0u16), (1, 8)] {
                let base = port * 4 + axis * 2;
                let pin = self.mouse_pins & (1 << base) != 0;
                let q = self.mouse_pins & (1 << (base + 1)) != 0;
                self.joy[port] = (self.joy[port] & !(3 << shift))
                    | (u16::from(quadrature_bits(pin, q)) << shift);
            }
        }
    }

    /// Apply every queued change stamped before `at`.
    fn apply_before(&mut self, rev: Revision, at: Stamp) {
        while let Some(change) = self.pending.front().copied() {
            if change.at >= at {
                break;
            }
            self.pending.pop_front();
            change.apply_to(&mut self.regs, rev);
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
    /// SuperHires, on an 8373: one bit per 35 ns quarter of a low-resolution
    /// pixel.
    shres: bool,
    dual: bool,
    ham: bool,
    ehb: bool,
    /// Per plane, the high-resolution half-pixel of its first bit — or, in
    /// SuperHires, the quarter-pixel.
    start: [i32; 6],
    /// Inside the window vertically.
    inside_v: bool,
    hstart: u32,
    hstop: u32,
    /// The border is black rather than `COLOR00`: `BRDRBLNK`.
    blank: bool,
}

impl Setup {
    fn of(regs: &Regs, fetch_start: u16, vpos: u16, rev: Revision) -> Setup {
        let ecs = rev == Revision::Ecs;
        let con0 = regs.bplcon0;
        // "111 not used" (Table 3-5). Six is the most there are.
        let planes = usize::from((con0 >> 12) & 7).min(6);
        // Appendix C: "HIRES Set it to zero if SHRES enabled". With both,
        // SuperHires wins; the manual does not say what the silicon does.
        let shres = ecs && con0 & SHRES != 0;
        let hires = !shres && con0 & HIRES != 0;
        let dual = con0 & DBLPF != 0;
        // Chapter 3, "Hold-And-Modify Mode": HOMOD set, DBLPF clear, HIRES
        // clear, and five or six planes, or it is not active. Appendix C: HAM
        // is "Incompatible w/ SuperHires mode".
        let ham = con0 & HOMOD != 0 && !dual && !hires && !shres && planes >= 5;
        // Appendix A, BPLCON0: "0 = Extra Half Brite (EHB) if HAM=0 and BPU=6
        // and DBLPF=0". An 8373's `KILLEHB` turns it off.
        let ehb = con0 & HOMOD == 0
            && !dual
            && planes == 6
            && !shres
            && !(ecs && regs.bplcon2 & KILLEHB != 0);

        // Chapter 3: DIWSTRT at $81 goes with DDFSTRT at $38 in low resolution
        // and $3C in high, "$81/2 - 8.5 = $38" and "$81/2 - 4.5 = $3C": the
        // serializer starts one fetch block and half a count after the block
        // began, eight counts in low resolution and four in high. SuperHires
        // fetches a block every two counts (four words to high resolution's
        // two), so by the same arithmetic its first pixel is 2.5 counts — five
        // low-resolution pixels — after the fetch. **Inference:** the manual
        // gives no SuperHires fetch formula.
        let lead: i32 = if shres {
            5
        } else if hires {
            9
        } else {
            17
        };
        // Half-pixels, or quarter-pixels in SuperHires.
        let unit: i32 = if shres { 4 } else { 2 };
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
            *s = unit * (2 * i32::from(fetch_start) + lead + i32::from(delay));
        }

        let (vstart, vstop, hstart, hstop) = if ecs && regs.diwhigh_on {
            // Appendix C, Display Window Specification: DIWHIGH's bits 10-8
            // and 13 are the stop's V10-V8 and H8, bits 2-0 and 5 the start's.
            let high = regs.diwhigh;
            (
                ((high & 7) << 8) | (regs.diwstrt >> 8),
                (((high >> 8) & 7) << 8) | (regs.diwstop >> 8),
                u32::from((high >> 5) & 1) << 8 | u32::from(regs.diwstrt & 0xff),
                u32::from((high >> 13) & 1) << 8 | u32::from(regs.diwstop & 0xff),
            )
        } else {
            // Appendix A, DIWSTRT/DIWSTOP: start is restricted to H8=0 and
            // V8=0; stop to H8=1 and "V8=/=V7".
            let vstop_lo = regs.diwstop >> 8;
            (
                regs.diwstrt >> 8,
                vstop_lo | if vstop_lo & 0x80 == 0 { 0x100 } else { 0 },
                u32::from(regs.diwstrt & 0xff),
                0x100 | u32::from(regs.diwstop & 0xff),
            )
        };
        // Appendix C, Genlock Extensions: BRDRBLNK, "Border blank", in the
        // BPLCON3 that BPLCON0's ENBPLCN3 enables.
        let blank = ecs && con0 & ENBPLCN3 != 0 && regs.bplcon3 & BRDRBLNK != 0;
        Setup {
            planes,
            hires,
            shres,
            dual,
            ham,
            ehb,
            start,
            inside_v: vstart <= vpos && vpos < vstop,
            hstart,
            hstop,
            blank,
        }
    }

    /// The six plane bits at high-resolution half-pixel `hx` — or, in
    /// SuperHires, at quarter-pixel `hx` — plane 1 in bit 0.
    #[inline]
    fn bits(&self, fetch: &Fetch<'_>, hx: i32) -> u8 {
        let mut bits = 0u8;
        for p in 0..self.planes {
            let rel = hx - self.start[p];
            if rel < 0 {
                continue;
            }
            let bit = if self.hires || self.shres {
                rel
            } else {
                rel >> 1
            } as usize;
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

/// One SuperHires pixel's colour from a register: Appendix C, *SuperHires Mode
/// and the Denise Color Registers*, "There are only two bits of red, green and
/// blue color resolution per hires pixel". Register `n` holds, in the top two
/// bits of each gun, the colour of pixel value `n & 3` and in the bottom two
/// that of `n >> 2` — the table's `COLOR01` is `gh ab`, colour 1's red over
/// colour 0's. So of a pair of SuperHires pixels (`left`, `right`), register
/// `left | right << 2` shows the left one through its top bits and the right
/// one through its bottom bits, and each shows its own colour.
///
/// **Choice:** which pixel of the pair takes the top bits. The manual gives
/// the encoding and not the silicon's index, and the other assignment shows
/// the same colours for any registers written as the table says. A two-bit
/// gun goes to a four-bit one by repeating it, which is what the table's own
/// `COLOR00`, `ab ab`, amounts to.
#[inline]
fn shr_colour(word: u16, top: bool) -> u16 {
    let mut out = 0;
    for shift in [8, 4, 0] {
        let nibble = (word >> shift) & 0xf;
        let gun = if top { nibble >> 2 } else { nibble & 3 };
        out |= ((gun << 2) | gun) << shift;
    }
    out
}

/// A twelve-bit `0RGB` colour register as the picture holds it: each four-bit
/// gun repeated into eight, `n × 17`.
///
/// The same expansion the host adapter always made from an 8362's word, moved
/// one step earlier so one picture buffer serves every part. It is also what
/// Lisa's own hardware does with a `LOCT = 0` write (AA specification, §4,
/// `COLORx`: "the 4 bit values are automatically extended to 8 bits"), so the
/// two paths agree by construction rather than by coincidence.
#[inline]
const fn rgb12(word: u16) -> u32 {
    let mut out = 0u32;
    let mut shift = 0;
    while shift < 24 {
        let nibble = ((word as u32) >> (shift / 2)) & 0xf;
        out |= (nibble << 4 | nibble) << shift;
        shift += 8;
    }
    out
}

/// The twelve-bit form of a picture word: the top nibble of each gun.
///
/// Exactly inverts [`rgb12`], so an 8362's or 8373's picture reads back as the
/// colour register that drew it. On Lisa it is the four most significant bits
/// of each eight, which is all twelve RGB pins ever carried.
#[inline]
const fn rgb12_of(pixel: u32) -> u16 {
    (((pixel >> 20) & 0xf) << 8 | ((pixel >> 12) & 0xf) << 4 | ((pixel >> 4) & 0xf)) as u16
}

/// Render one line into `st`. The heart of the chip.
fn render(rev: Revision, st: &mut State, line: &Line<'_>) {
    if rev.is_aga() {
        aga::render(st, line);
        return;
    }
    let field = st.fields;
    let vpos = line.vpos;
    st.apply_before(
        rev,
        Stamp {
            field,
            vpos,
            hpos: 0,
        },
    );
    st.clocks = st.clocks.wrapping_add(u64::from(line.clocks));

    // Which rows of the picture this line lands on. Interlace is decided at
    // the start of the line; a mid-line LACE change is not a picture anyone
    // has asked for.
    let layout = st.layout;
    let first = layout.first_line;
    let rows: [Option<usize>; 2] = if vpos >= first && vpos - first < layout.lines {
        let at = usize::from(vpos - first);
        if layout.rows == 1 {
            [Some(at), None]
        } else if st.regs.bplcon0 & LACE != 0 {
            [Some(2 * at + usize::from(!st.lof)), None]
        } else {
            [Some(2 * at), Some(2 * at + 1)]
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
        .max(u32::from(layout.left) + u32::from(layout.span))
        .min(1024);
    let mut setup = Setup::of(&st.regs, line.fetch.start, vpos, rev);

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

    // Columns in high-resolution half-pixels or SuperHires quarter-pixels, and
    // where the picture's first one is. The first SuperHires line of a field
    // laid out in half-pixels widens the picture there and then.
    if setup.shres {
        st.shres_seen = true;
        if st.layout.scale == 2 {
            st.rescale(4);
        }
    }
    let mut width = st.layout.width() as i32;
    let mut quarters = st.layout.scale == 4;
    let mut left = i32::from(st.layout.left) * i32::from(st.layout.scale);

    let mut shifters = [Shifter::default(); 8];
    let mut previous = [0u8; 8];
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
            change.apply_to(&mut st.regs, rev);
            changed = true;
        }
        if changed {
            setup = Setup::of(&st.regs, line.fetch.start, vpos, rev);
            if setup.shres {
                st.shres_seen = true;
                if st.layout.scale == 2 {
                    st.rescale(4);
                    width = st.layout.width() as i32;
                    quarters = true;
                    left = i32::from(st.layout.left) * 4;
                }
            }
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
                    // An 8362's and an 8373's buffers are sixteen bits: the
                    // top of the wide ones, which is where a register write
                    // puts its word.
                    *s = Shifter {
                        a: (regs.spr_data[i] >> 48) as u16,
                        b: (regs.spr_datb[i] >> 48) as u16,
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
        // An 8373 in SuperHires places a sprite with SHSH1 set 70 ns — half a
        // low-resolution pixel — later (Appendix C, *SuperHires 70ns Sprite
        // Positioning*): the first half of this pixel still shows its last.
        let mut early = pixels;
        if setup.shres {
            for (i, px) in early.iter_mut().enumerate() {
                if regs.spr_ctl[i] & SHSH1 != 0 {
                    *px = previous[i];
                }
            }
        }
        previous = pixels;
        let inside = setup.inside_v && setup.hstart <= x && x < setup.hstop;
        let border = if setup.blank { 0 } else { regs.color[0] };

        for sub in 0..2 {
            let hx = (2 * x + sub) as i32;
            let pixels = if sub == 0 { &early } else { &pixels };
            let sprite = front_sprite(pixels, regs);

            if setup.shres {
                // Two SuperHires pixels, whose colours come out of one register
                // between them (`shr_colour`).
                let q = 2 * hx;
                let bits = [setup.bits(&line.fetch, q), setup.bits(&line.fetch, q + 1)];
                let pair = usize::from(bits[0] & 3) | usize::from(bits[1] & 3) << 2;
                for (half, &b) in bits.iter().enumerate() {
                    let top = half == 0;
                    let colour = if !inside {
                        shr_colour(border, top)
                    } else {
                        clx |= collisions(b, pixels, regs.clxcon);
                        let blocked =
                            sprite.is_some_and(|s| u16::from(s.0) < (regs.bplcon2 >> 3) & 7);
                        match sprite {
                            Some((_, reg)) if b & 3 == 0 || blocked => {
                                // A sprite's colour is in the upper sixteen
                                // registers, encoded the same way, and both
                                // halves of the pair are the sprite.
                                let s = (reg - 16) & 3;
                                shr_colour(regs.color[16 + (s | s << 2)], top)
                            }
                            _ => shr_colour(regs.color[pair], top),
                        }
                    };
                    let col = q + half as i32 - left;
                    if (0..width).contains(&col) {
                        for row in rows.into_iter().flatten() {
                            st.frame[row * width as usize + col as usize] = rgb12(colour);
                        }
                    }
                }
                continue;
            }

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
                border
            } else {
                clx |= collisions(bits, pixels, regs.clxcon);
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

            // One column per half-pixel on an 8362, two on an 8373.
            if quarters {
                for q in [2 * hx, 2 * hx + 1] {
                    let col = q - left;
                    if (0..width).contains(&col) {
                        for row in rows.into_iter().flatten() {
                            st.frame[row * width as usize + col as usize] = rgb12(colour);
                        }
                    }
                }
            } else {
                let col = hx - left;
                if (0..width).contains(&col) {
                    for row in rows.into_iter().flatten() {
                        st.frame[row * width as usize + col as usize] = rgb12(colour);
                    }
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
    rev: Revision,
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
            .field("rev", &self.rev)
            .field("state", &*self.state.lock())
            .field("beam", &self.beam.lock().is_some())
            .finish_non_exhaustive()
    }
}

impl Video {
    /// An original 8362 at power-on for `standard`.
    #[must_use]
    pub fn new(standard: Standard) -> Video {
        Video::with_revision(standard, Revision::Ocs)
    }

    /// Part `rev` at power-on for `standard`.
    #[must_use]
    pub fn with_revision(standard: Standard, rev: Revision) -> Video {
        Video {
            standard,
            rev,
            state: Mutex::with_rank(LockRank::LEAF, State::new(standard, rev)),
            beam: Mutex::with_rank(LockRank::LEAF, None),
            bus: Mutex::with_rank(LockRank::LEAF, Weak::new()),
        }
    }

    /// The standard the picture is laid out for until a beam says otherwise.
    #[must_use]
    pub fn standard(&self) -> Standard {
        self.standard
    }

    /// Which part this is.
    #[must_use]
    pub fn revision(&self) -> Revision {
        self.rev
    }

    /// The picture's size, `(width, height)`: `(WIDTH, standard.height())` for
    /// an 8362 driven by an original Agnus, and whatever the current field's
    /// [`Raster`] makes it otherwise.
    #[must_use]
    pub fn geometry(&self) -> (u32, u32) {
        let st = self.state.lock();
        (st.layout.width(), st.layout.height())
    }

    /// The raster the current field is laid out by.
    #[must_use]
    pub fn current_raster(&self) -> Raster {
        self.state.lock().raster
    }

    /// Give Denise the beam counter, so a mid-line write lands mid-line.
    pub fn connect_beam(&self, beam: Arc<dyn Beam>) {
        *self.beam.lock() = Some(beam);
    }

    /// The raster the next field is shown on. A beam whose raster can change
    /// calls this before each [`field`](Self::field); one that never calls it
    /// leaves the picture laid out by [`Raster::standard`].
    pub fn raster(&self, raster: Raster) {
        self.state.lock().raster_next = raster.bounded();
    }

    /// Render one line. See the module documentation for the contract.
    pub fn line(&self, line: &Line<'_>) {
        let mut st = self.state.lock();
        render(self.rev, &mut st, line);
    }

    /// A new field begins; `lof` is its long-frame bit (`VPOSR` bit 15).
    pub fn field(&self, lof: bool) {
        let mut st = self.state.lock();
        st.fields = st.fields.wrapping_add(1);
        st.last_field_clocks = st.clocks;
        st.clocks = 0;
        st.lof = lof;
        st.relayout(self.rev);
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
    ///
    /// Twelve bits is an 8362's and an 8373's whole colour register, so for
    /// those two this is the picture exactly. On Lisa it is the top four bits
    /// of each eight-bit gun — the twelve pins the older parts had — and
    /// [`read_row_rgb`](Self::read_row_rgb) is the full picture.
    pub fn read_row(&self, y: u32, dst: &mut [u16]) {
        let st = self.state.lock();
        let (width, height) = (st.layout.width(), st.layout.height());
        if y >= height {
            return;
        }
        let at = y as usize * width as usize;
        let n = dst.len().min(width as usize);
        for (out, px) in dst[..n].iter_mut().zip(&st.frame[at..at + n]) {
            *out = rgb12_of(*px);
        }
    }

    /// Row `y` of the picture as `0x00RR_GGBB` words, eight bits a gun. No
    /// side effects.
    pub fn read_row_rgb(&self, y: u32, dst: &mut [u32]) {
        let st = self.state.lock();
        let (width, height) = (st.layout.width(), st.layout.height());
        if y >= height {
            return;
        }
        let at = y as usize * width as usize;
        let n = dst.len().min(width as usize);
        dst[..n].copy_from_slice(&st.frame[at..at + n]);
    }

    /// The whole picture, as 12-bit `0RGB` words row after row, and its size
    /// and field count — all as of one moment, which a host that reads row by
    /// row while the geometry can change would not get. No side effects.
    ///
    /// Twelve bits for the reason [`read_row`](Self::read_row) gives;
    /// [`copy_frame_rgb`](Self::copy_frame_rgb) is the full picture.
    pub fn copy_frame(&self, dst: &mut Vec<u16>) -> (u32, u32, u64) {
        let st = self.state.lock();
        dst.clear();
        dst.extend(st.frame.iter().map(|px| rgb12_of(*px)));
        (st.layout.width(), st.layout.height(), st.fields)
    }

    /// The whole picture as `0x00RR_GGBB` words, eight bits a gun, with its
    /// size and field count, all as of one moment. No side effects. What a
    /// host shows.
    pub fn copy_frame_rgb(&self, dst: &mut Vec<u32>) -> (u32, u32, u64) {
        let st = self.state.lock();
        dst.clear();
        dst.extend_from_slice(&st.frame);
        (st.layout.width(), st.layout.height(), st.fields)
    }

    /// One AA sprite-data DMA transfer, of the width `FMODE` selects.
    ///
    /// # The seam Alice needs
    ///
    /// `SPRxDATA` and `SPRxDATB` are sixteen-bit addresses on the register
    /// bus, and a processor write through [`CustomChip::write`] still reaches
    /// them — "they may be loaded by either processor at any time" (AA
    /// specification, §4, `SPRxDAT`). But with `FMODE`'s `SPR32` or `SPAGEM`
    /// set a sprite fetch is 32 or 64 bits and all of it arrives in one
    /// transfer, which no `u16` on that bus can carry. Alice makes that
    /// transfer here instead.
    ///
    /// `sprite` is 0–7, `b_buffer` picks `SPRxDATB` over `SPRxDATA`, and
    /// `bits` is the fetched data **left-justified**: the first pixel in bit
    /// 63, because "the MSB is output first on the left". A 16-bit `FMODE`
    /// therefore puts its word in bits 63–48, which is exactly what a
    /// register write does, so Alice may use either path at that width and get
    /// the same picture. Writing the A buffer arms the sprite and writing
    /// `SPRxCTL` disarms it, here as on the register bus.
    ///
    /// The transfer is timed exactly as a register write is: with a [`Beam`]
    /// connected it lands at the pixel the beam is at, in order with the
    /// `SPRxPOS` and `SPRxCTL` writes the same DMA slot made through the
    /// register bus — so a `CTL` write that disarms the sprite is followed,
    /// not overtaken, by the data that arms it again.
    ///
    /// Bitplane data needs nothing like this: a fetch of any width is that
    /// many consecutive pixels in [`Fetch::planes`].
    pub fn sprite_dma(&self, sprite: usize, b_buffer: bool, bits: u64) {
        if sprite >= 8 {
            return;
        }
        let offset = SPR0POS + 8 * sprite as u16 + if b_buffer { 6 } else { 4 };
        self.schedule(self.now(), offset | WIDE, bits);
    }

    /// Put a change in force now (no beam, or a debugger's write) or queue it
    /// for the pixel `now` names. The caller asks the beam with nothing
    /// locked, because asking may make Agnus push lines into this very chip.
    fn schedule(&self, now: Option<BeamPosition>, offset: u16, value: u64) {
        let mut st = self.state.lock();
        let st = &mut *st;
        let change = |at| Change { at, offset, value };
        match now {
            None => {
                // The change is in force from the next line rendered. Anything
                // already queued goes first so the order of writes is kept.
                for c in st.pending.drain(..) {
                    c.apply_to(&mut st.regs, self.rev);
                }
                change(Stamp::default()).apply_to(&mut st.regs, self.rev);
            }
            Some(pos) => {
                let at = Stamp {
                    field: st.fields,
                    vpos: pos.vpos,
                    hpos: pos.hpos,
                };
                st.pending.push_back(change(at));
                while st.pending.len() > MAX_PENDING {
                    if let Some(c) = st.pending.pop_front() {
                        c.apply_to(&mut st.regs, self.rev);
                    }
                }
            }
        }
    }

    /// The collision register as it stands, without clearing it.
    #[must_use]
    pub fn peek_clxdat(&self) -> u16 {
        self.state.lock().clxdat
    }

    fn reset(&self) {
        let mut st = self.state.lock();
        let pins = st.mouse_pins;
        *st = State::new(self.standard, self.rev);
        st.mouse_pins = pins;
        st.settle_counters();
    }

    /// The mouse counters, `[JOY0DAT, JOY1DAT]`, without a bus access.
    #[must_use]
    pub fn joy(&self) -> [u16; 2] {
        self.state.lock().joy
    }

    /// Set mouse input `line` ([`MOUSE_PINS`] order) to `high`, counting the
    /// transition.
    pub fn set_mouse_pin(&self, line: usize, high: bool) {
        if line >= MOUSE_PINS.len() {
            return;
        }
        let mut st = self.state.lock();
        let bit = 1u8 << line;
        let was = st.mouse_pins;
        st.mouse_pins = if high { was | bit } else { was & !bit };
        if st.mouse_pins != was {
            st.clock_counters();
        }
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
            w.write_u64(c.value)?;
        }
        // The chip's own strap and part, so a snapshot of one is not restored
        // into the other: the raster a beam handed over says how *this* field
        // is laid out, not which chip it was.
        w.write_bool(self.standard == Standard::Ntsc)?;
        w.write_u8(match self.rev {
            Revision::Ocs => 0,
            Revision::Ecs => 1,
            Revision::Aga => 2,
        })?;
        for raster in [st.raster, st.raster_next] {
            save_raster(w, raster)?;
        }
        w.write_u16(st.layout.rows)?;
        w.write_u16(st.layout.scale)?;
        w.write_bool(st.shres_seen)?;
        // The picture is architectural state in the sense `dev::lcd::panel`
        // argues: nothing else saves it, and an interlaced frame keeps the
        // other field's rows from before the snapshot.
        let mut bytes = Vec::with_capacity(st.frame.len() * 4);
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
        let count = r.read_seq_len(22)?;
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
            let (offset, value) = (r.read_u16()?, r.read_u64()?);
            // A register word, or a wide load of a sprite data buffer: nothing
            // else is ever queued.
            let sprite_data = matches!(offset & !WIDE, SPR0POS..=SPR7DATB)
                && ((offset & !WIDE) - SPR0POS) % 8 >= 4;
            let fits = if offset & WIDE != 0 {
                sprite_data
            } else {
                value <= 0xffff && offset < 0x200
            };
            if !fits {
                return Err(Error::State(alloc::format!(
                    "{CLASS_NAME}: a queued change of {value:#x} at {offset:#06x}"
                )));
            }
            pending.push_back(Change { at, offset, value });
        }
        let ntsc = r.read_bool()?;
        let rev = match r.read_u8()? {
            0 => Revision::Ocs,
            1 => Revision::Ecs,
            2 => Revision::Aga,
            other => {
                return Err(Error::State(alloc::format!(
                    "{CLASS_NAME}: revision {other} is not one this chip has"
                )));
            }
        };
        if ntsc != (self.standard == Standard::Ntsc) || rev != self.rev {
            return Err(Error::State(alloc::format!(
                "{CLASS_NAME}: the snapshot is of a {} {rev:?} Denise and this is a {:?} {:?} one",
                if ntsc { "NTSC" } else { "PAL" },
                self.standard,
                self.rev,
            )));
        }
        let raster = load_raster(r)?;
        let raster_next = load_raster(r)?;
        let rows = r.read_u16()?;
        let scale = r.read_u16()?;
        let shres_seen = r.read_bool()?;
        // Lisa's picture is always in 35 ns columns; an 8373's is in them
        // only while SuperHires is on screen; an 8362 has none.
        let scales: &[u16] = match self.rev {
            Revision::Ocs => &[2],
            Revision::Ecs => &[2, 4],
            Revision::Aga => &[4],
        };
        if !(1..=2).contains(&rows) || !scales.contains(&scale) {
            return Err(Error::State(alloc::format!(
                "{CLASS_NAME}: {rows} rows a line and {scale} columns a pixel; a {:?} Denise's \
                 picture has one or two, and one of {scales:?}",
                self.rev
            )));
        }
        let layout = Layout {
            rows,
            ..Layout::of(raster, scale == 4, false)
        };
        let bytes = r.read_bytes()?;
        let expected = layout.width() as usize * layout.height() as usize;
        if bytes.len() != expected * 4 {
            return Err(Error::State(alloc::format!(
                "{CLASS_NAME}: the snapshot's picture is {} bytes and its raster's is {}",
                bytes.len(),
                expected * 4
            )));
        }
        let frame = bytes
            .as_chunks::<4>()
            .0
            .iter()
            .map(|b| u32::from_le_bytes(*b) & 0x00ff_ffff)
            .collect();

        let mut st = self.state.lock();
        let mouse_pins = st.mouse_pins;
        *st = State {
            regs,
            pending,
            clxdat,
            joy,
            mouse_pins,
            dmacon,
            fields,
            lof,
            clocks,
            last_field_clocks,
            raster,
            raster_next,
            layout,
            shres_seen,
            frame,
        };
        Ok(())
    }
}

fn save_raster(w: &mut ChunkWriter<'_>, raster: Raster) -> Result<()> {
    for v in [
        raster.first_line,
        raster.lines,
        raster.first_clock,
        raster.clocks,
        raster.line_clocks,
    ] {
        w.write_u16(v)?;
    }
    Ok(())
}

fn load_raster(r: &mut ChunkReader<'_>) -> Result<Raster> {
    Ok(Raster {
        first_line: r.read_u16()?,
        lines: r.read_u16()?,
        first_clock: r.read_u16()?,
        clocks: r.read_u16()?,
        line_clocks: r.read_u16()?,
    }
    .bounded())
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
            DENISEID if self.rev == Revision::Ecs => ECS_DENISEID,
            DENISEID if self.rev.is_aga() => LISA_ID,
            // An 8362 never gets here: `drives` below tells the bus this part
            // has no such register, and the bus answers off the chip data
            // lines instead.
            DENISEID => 0,
            _ => 0,
        }
    }

    /// An 8362 does not drive `DENISEID`: "The original Denise (8362) does not
    /// have this register, so whatever value is left over on the bus from the
    /// last cycle will be there" (Appendix C, p. 299).
    ///
    /// Saying so here rather than answering with the bus is what makes that
    /// sentence an *observable* difference. A chip's answer is driven back
    /// onto the lines, so a part that read them and returned what it found
    /// would hold them steady — and a steady answer is exactly what an 8373
    /// gives. Not driving leaves the read a cycle nothing drove, which is what
    /// Commodore's own detection relies on.
    fn drives(&self, reg: &Reg) -> bool {
        reg.offset != DENISEID || self.rev.is_ecs()
    }

    fn write(&self, reg: &Reg, value: u16, from: Origin) {
        let offset = reg.offset;
        if Regs::timed(self.rev, offset) {
            // Ask the beam first, with nothing locked: asking may make Agnus
            // catch up and push lines into this very chip. A debugger's write
            // is in force at once.
            let now = if from.debug { None } else { self.now() };
            self.schedule(now, offset, u64::from(value));
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

/// One mouse input, as something a wire drives.
#[derive(Debug)]
struct MousePin {
    video: Arc<Video>,
    line: usize,
    inputs: FanIn,
}

impl WireSink for MousePin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::And).is_high();
        self.video.set_mouse_pin(self.line, high);
    }
}

/// The `amiga.denise` device.
#[derive(Debug)]
pub struct Denise {
    video: Arc<Video>,
    custom: String,
    /// The mouse inputs handed out, kept alive for the nets that hold them
    /// weakly.
    pins: Mutex<Vec<Arc<MousePin>>>,
}

impl Denise {
    /// A Denise with the given properties.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `custom` is missing, `standard` is not `"pal"`
    /// or `"ntsc"`, `revision` is not `"ocs"`, `"ecs"` or `"aga"`, or a
    /// property nothing here accepts was given.
    pub fn new(props: &Props) -> Result<Denise> {
        let mut r = props.reader();
        let custom = r.require_link("custom")?.as_str().to_string();
        let standard = match r.or_enum("standard", "pal", &["pal", "ntsc"])? {
            "ntsc" => Standard::Ntsc,
            _ => Standard::Pal,
        };
        let rev = match r.or_enum("revision", "ocs", &["ocs", "ecs", "aga"])? {
            "aga" => Revision::Aga,
            "ecs" => Revision::Ecs,
            _ => Revision::Ocs,
        };
        r.finish()?;
        Ok(Denise {
            video: Arc::new(Video::with_revision(standard, rev)),
            custom,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
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

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let line = MOUSE_PINS.iter().position(|p| *p == port)?;
        let pin = Arc::new(MousePin {
            video: Arc::clone(&self.video),
            line,
            inputs: FanIn::new(sources),
        });
        // A net holds its sinks weakly; the device is what keeps them.
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: line as u32,
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
    summary: "the Amiga's Denise/Lisa video chip: colour table, playfields, sprites, collisions",
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
        PropertySpec {
            name: "revision",
            kind: ValueKind::Str,
            required: false,
            summary: "`ocs` (default, an 8362), `ecs` (an 8373: DENISEID, BPLCON3, DIWHIGH, SuperHires) or `aga` (Lisa: eight bitplanes, 256 24-bit colours, HAM8, FMODE, BPLCON4)",
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
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("custom", ValueKind::Link).required())
        .prop(PropSchema::new("standard", ValueKind::Str))
        .prop(PropSchema::new("revision", ValueKind::Str).values(&["ocs", "ecs", "aga"]));
    for pin in MOUSE_PINS {
        schema = schema.port(pin, PortDir::In);
    }
    schema
}

#[cfg(test)]
mod tests;
