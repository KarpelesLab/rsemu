//! Which console the picture unit is: NTSC (2C02), PAL (2C07) or Dendy
//! (UA6538).
//!
//! A **construction property**, never a `#[cfg]`: one build of rsemu runs all
//! three, and a machine description picks one with `region = "pal"`. The chip
//! is a different part number in each case, and the differences are frame
//! geometry and clock dividers rather than anything the register file can see.
//!
//! # Source
//!
//! Every figure here comes from the [NESdev cycle reference
//! chart](https://www.nesdev.org/wiki/Cycle_reference_chart), cross-checked
//! against [PPU rendering](https://www.nesdev.org/wiki/PPU_rendering), which is
//! where the two sentences "Dendy PPUs render 51 post-render scanlines instead
//! of 1" and "PAL NES PPUs render 70 vblank scanlines instead of 20" live.
//!
//! # Why there is no "dots per CPU cycle" constant
//!
//! The chart quotes 3, 3.2 and 3 dots per CPU cycle, and 3.2 is not a number
//! this crate is allowed to hold (`CLAUDE.md`, "no floats in the time path").
//! It does not need to: both counters descend from one crystal, so the exact
//! statement is *master ÷ 16 for the CPU and master ÷ 5 for the dot clock*, and
//! the ratio falls out of the clock forest's integer arithmetic
//! (`ROADMAP.md` §4.2). Everything below that looks like a CPU-cycle figure is
//! therefore derived from [`Region::cpu_divider`] and [`Region::dot_divider`]
//! rather than stored.

use core::fmt;

use super::engine::DOTS_PER_SCANLINE;

/// CPU cycles the 2C02 ignores writes to `$2000`, `$2001`, `$2005` and `$2006`
/// for after a reset.
///
/// The measured figure for the 2C02 is ~29658 CPU cycles
/// ([NESdev PPU registers](https://www.nesdev.org/wiki/PPU_registers),
/// PPUCTRL). The wiki records no separate measurement for the 2C07 or the
/// UA6538, so the same *CPU-cycle* count is assumed for all three and converted
/// to dots through the dividers — see [`Geometry::warmup_dots`].
pub const RESET_LOCKOUT_CPU_CYCLES: u64 = 29_658;

/// The palette index the PAL video border is forced to.
///
/// `$0E` is black. The chart's "Side and bottom borders" row gives the 2C02's
/// border as palette entry `$3F00` but the 2C07's as "always black (`$0E`),
/// intruding on left and right 2 pixels and top 1 pixel of picture" — which is
/// exactly why the PAL and Dendy picture is 239 scanlines tall out of 240
/// rendered.
pub const BORDER_BLACK: u8 = 0x0E;

/// Which console variant a picture unit is.
///
/// A real enum rather than the `#[repr(transparent)]` newtype `CLAUDE.md`
/// prescribes for extensible enumerations, because exhaustiveness is genuinely
/// wanted here: [`Region::geometry`] has one arm per variant and a fourth
/// famiclone must not silently inherit NTSC's frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Region {
    /// Ricoh RP2C02, 60 Hz, 262 scanlines. The Famicom and the North American
    /// and Japanese NES.
    #[default]
    Ntsc,
    /// Ricoh RP2C07, 50 Hz, 312 scanlines. The European and Australian NES.
    Pal,
    /// UMC UA6538, the PAL famiclone chipset "Dendy" has come to name.
    ///
    /// A 50 Hz picture driven by a deliberately Famicom-compatible CPU rate:
    /// the chart notes the clone "is designed for compatibility with Famicom
    /// games, including games with CPU cycle counting mappers", which is why it
    /// divides the PAL crystal by 15 rather than 16 and pads the difference out
    /// with 51 post-render scanlines instead of 1.
    Dendy,
}

impl Region {
    /// Every name [`Region::from_name`] accepts, for `or_enum` and
    /// `rsemu describe`.
    pub const NAMES: &'static [&'static str] = &["ntsc", "pal", "dendy"];

    /// Parse the `region` property.
    pub fn from_name(name: &str) -> Option<Region> {
        match name {
            "ntsc" => Some(Region::Ntsc),
            "pal" => Some(Region::Pal),
            "dendy" => Some(Region::Dendy),
            _ => None,
        }
    }

    /// The name this variant is written with in a `.machine` file.
    pub const fn name(self) -> &'static str {
        match self {
            Region::Ntsc => "ntsc",
            Region::Pal => "pal",
            Region::Dendy => "dendy",
        }
    }

    /// The part number of the picture unit.
    pub const fn part_number(self) -> &'static str {
        match self {
            Region::Ntsc => "RP2C02",
            Region::Pal => "RP2C07",
            Region::Dendy => "UA6538",
        }
    }

    /// The board's master crystal, as an exact `(numerator, denominator)` in
    /// hertz.
    ///
    /// Neither is an integer number of hertz, which is the case
    /// `ROADMAP.md` §4.2 built rational oscillator literals for: NTSC is
    /// 236.25 MHz ÷ 11 = 21477272.72… Hz *by definition*, and PAL is
    /// 26.6017125 MHz by definition, i.e. 53203425/2 Hz. Neither value affects
    /// any ratio a game can observe — only the wall-clock rate.
    pub const fn master_clock(self) -> (u64, u64) {
        match self {
            Region::Ntsc => (236_250_000, 11),
            // Dendy is a PAL board: same crystal, different dividers.
            Region::Pal | Region::Dendy => (53_203_425, 2),
        }
    }

    /// Master clocks per CPU cycle.
    ///
    /// The chart explains PAL's 16: "the PAL CPU's master clock could have been
    /// divided by 15 to preserve the ratio between CPU and PPU speeds, but
    /// Nintendo chose to keep the Johnson counter structure, which always has
    /// an even period". The famiclone divides by 15 and gets NTSC's 3:1 ratio
    /// back.
    pub const fn cpu_divider(self) -> u64 {
        match self {
            Region::Ntsc => 12,
            Region::Pal => 16,
            Region::Dendy => 15,
        }
    }

    /// Master clocks per PPU dot.
    pub const fn dot_divider(self) -> u64 {
        match self {
            Region::Ntsc => 4,
            Region::Pal | Region::Dendy => 5,
        }
    }

    /// The frame this variant draws.
    pub const fn geometry(self) -> Geometry {
        // Both derived from the dividers rather than from a dots-per-CPU-cycle
        // figure, which is what keeps PAL's 16/5 exact.
        let cpu = self.cpu_divider();
        let dot = self.dot_divider();
        // PAL's lockout is 94905.6 dots, which no dot counter can hold; the
        // floor is the last dot that is certainly still inside the window.
        let warmup_dots = RESET_LOCKOUT_CPU_CYCLES * cpu / dot;
        let (scanlines_per_frame, picture_height, post_render_lines, vblank_lines, odd_frame_skip) =
            match self {
                Region::Ntsc => (262, 240, 1, 20, true),
                Region::Pal => (312, 239, 1, 70, false),
                Region::Dendy => (312, 239, 51, 20, false),
            };
        Geometry {
            scanlines_per_frame,
            visible_scanlines: VISIBLE_SCANLINES,
            picture_height,
            post_render_lines,
            vblank_scanline: VISIBLE_SCANLINES + post_render_lines,
            vblank_lines,
            pre_render_scanline: scanlines_per_frame - 1,
            odd_frame_skip,
            dots_per_frame: DOTS_PER_SCANLINE as u64 * scanlines_per_frame as u64,
            warmup_dots,
        }
    }
}

impl fmt::Display for Region {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Region::Ntsc => "NTSC",
            Region::Pal => "PAL",
            Region::Dendy => "Dendy",
        })
    }
}

/// Scanlines the render pipeline draws, in every region.
///
/// All three chips render 240 lines; PAL and Dendy differ only in that their
/// video border paints over the topmost one, which is what makes their
/// *picture* 239 tall. Keeping this fixed is what keeps
/// [`FRAMEBUFFER_LEN`](super::FRAMEBUFFER_LEN) — and therefore the snapshot
/// format — the same in every region.
const VISIBLE_SCANLINES: u16 = 240;

/// One region's frame, in scanlines and dots.
///
/// Every field is a plain fact from the cycle reference chart, resolved into
/// the scanline indices the engine compares against. One invariant ties them
/// together, and the tests check it for all three regions:
///
/// ```text
/// visible + post_render + vblank + 1 pre-render == scanlines_per_frame
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Geometry {
    /// Scanlines in one frame: picture, blanking and pre-render together.
    pub scanlines_per_frame: u16,
    /// Scanlines the render pipeline runs on. 240 everywhere.
    pub visible_scanlines: u16,
    /// Scanlines of picture a television actually shows.
    ///
    /// 239 on PAL and Dendy, because their border blanks the top rendered
    /// line; see [`BORDER_BLACK`].
    pub picture_height: u16,
    /// Blanking scanlines between the last rendered line and the NMI.
    ///
    /// 1, except on Dendy, where 51 of them absorb the difference between a
    /// 50 Hz picture and a Famicom-rate CPU.
    pub post_render_lines: u16,
    /// The scanline whose dot 1 sets the vblank flag and requests the NMI.
    pub vblank_scanline: u16,
    /// Vertical blanking scanlines after the NMI.
    pub vblank_lines: u16,
    /// The pre-render (dummy) scanline, whose dot 1 clears the status flags.
    ///
    /// Always the last one, and always exactly one scanline: the chart's
    /// "pre-render lines" row spans every column.
    pub pre_render_scanline: u16,
    /// Whether an odd frame with rendering enabled is one dot shorter.
    ///
    /// True only on NTSC: the chart gives 341 × 261 + 340.5 for the 2C02 and a
    /// flat 341 × 312 for the 2C07 and the UA6538.
    pub odd_frame_skip: bool,
    /// Dots in a frame with no skip applied.
    pub dots_per_frame: u64,
    /// Dots the chip ignores `$2000`/`$2001`/`$2005`/`$2006` writes for after
    /// a reset — [`RESET_LOCKOUT_CPU_CYCLES`] converted through the dividers.
    pub warmup_dots: u64,
}

impl Geometry {
    /// Rendered scanlines the video border paints over at the top of the
    /// picture: 0 on NTSC, 1 on PAL and Dendy.
    #[inline]
    pub const fn top_border_lines(&self) -> u16 {
        self.visible_scanlines - self.picture_height
    }
}
