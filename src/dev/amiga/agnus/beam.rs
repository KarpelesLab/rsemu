//! The beam counters: where the video beam is, and how far away the next line
//! and the next field are.
//!
//! # What the manual says
//!
//! *Amiga Hardware Reference Manual*, 3rd edition, chapter 2, "The WAIT
//! Instruction — Horizontal Beam Position" and "— Vertical Beam Position":
//!
//! * "One horizontal count takes one cycle of the system clock (processor is
//!   twice this)", at **3 579 545 Hz** on NTSC and **3 546 895 Hz** on PAL —
//!   the colour clock. So a board gives this chip `clock = <crystal> / 8`, and
//!   one tick of its domain is one count of the horizontal counter.
//! * "All lines are not the same length in NTSC. Every other line is a long
//!   line (228 color clocks, 0-$E3), with the others being 227 color clocks
//!   long. In PAL, they are all 227 long."
//! * "There are alternating long and short lines, there are also long and short
//!   fields (interlace only). In NTSC, the fields are 262, then 263 lines and in
//!   PAL, 312, then 313 lines."
//!
//! Chapter 7, "Using the Beam Position Counter", gives `VPOSR` bit 15 as `LOF`,
//! the long-frame bit, and bit 0 as `V8`, which "allows PAL line counts (313) to
//! appear"; Appendix A gives `LOF` as the "auto toggle control bit in
//! `BPLCON0`" — the `LACE` bit.
//!
//! # What is decided here, and why
//!
//! * **A field is 312 or 262 lines plus `LOF`.** `LOF` toggles at the start of
//!   each field while `BPLCON0`'s `LACE` is set, and holds otherwise.
//! * **`LOF` comes out of reset set**, so a non-interlaced display runs long
//!   fields: 313 lines on PAL, 263 on NTSC. The manual never states the reset
//!   value. It does place the last beam position of a field at "(226,262) NTSC
//!   (or (226,312) PAL)" (chapter 2, *Putting Together a Copper Instruction
//!   List*), which is the last line of a *long* field, and it gives 313 as the
//!   PAL line count `V8` exists for. Both read as long fields being the
//!   ordinary case.
//! * **Line length alternates on NTSC regardless of the field**, so the "four
//!   field repeating pattern" the manual lists falls out of one flip-flop that
//!   toggles every line rather than being a table.

/// The video standard: which line counts and which line lengths.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Standard {
    /// 312/313 lines, every line 227 counts, 3 546 895 Hz colour clock.
    Pal,
    /// 262/263 lines, lines alternating 227 and 228 counts, 3 579 545 Hz.
    Ntsc,
}

impl Standard {
    /// The property value a machine file writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Standard::Pal => "pal",
            Standard::Ntsc => "ntsc",
        }
    }

    /// Lines in a short field.
    #[must_use]
    pub const fn short_field(self) -> u16 {
        match self {
            Standard::Pal => 312,
            Standard::Ntsc => 262,
        }
    }

    /// The colour clock, in hertz: one horizontal count.
    ///
    /// Not used to time anything — the domain the machine file gives the chip
    /// is the clock — but a board test asserts that the domain is this rate.
    #[must_use]
    pub const fn colour_clock_hz(self) -> u64 {
        match self {
            Standard::Pal => 3_546_895,
            Standard::Ntsc => 3_579_545,
        }
    }

    /// The Agnus identification `VPOSR` carries in bits 14–8 for an original
    /// chip-set part of this standard.
    ///
    /// Appendix C, *Determining Chip Revisions*: "8361 (regular NTSC) or 8370
    /// (fat NTSC) = 10 for NTSC Agnus", "8367 (regular PAL) or 8371 (fat PAL) =
    /// 00 for PAL Agnus". An A500's Agnus is one of the two fat parts.
    #[must_use]
    pub const fn agnus_id(self) -> u8 {
        match self {
            Standard::Pal => 0x00,
            Standard::Ntsc => 0x10,
        }
    }

    /// Whether lines alternate between long and short.
    #[must_use]
    pub const fn long_lines(self) -> bool {
        matches!(self, Standard::Ntsc)
    }

    /// Parse a machine-file value.
    #[must_use]
    pub fn parse(value: &str) -> Option<Standard> {
        match value {
            "pal" => Some(Standard::Pal),
            "ntsc" => Some(Standard::Ntsc),
            _ => None,
        }
    }
}

/// How many counts a short line has.
pub const SHORT_LINE: u16 = 227;

/// The shape of the raster the counters run through: how long a line is, how
/// many lines a field has, and whether either alternates.
///
/// An original chip-set Agnus has one per [`Standard`], wired in. An ECS Agnus
/// derives it from `BEAMCON0` and, with `VARBEAMEN`, from `HTOTAL` and
/// `VTOTAL` (see [`ecs`](super::ecs)), which is how productivity mode's 31 kHz
/// lines are built. Everything that counts the beam takes one of these, and
/// `Timing::from(std)` is exactly the original chip set's numbers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    /// Counts in a short line.
    pub line: u16,
    /// Lines alternate between short and one count longer (NTSC's 227/228).
    pub alternate: bool,
    /// Lines in a short field.
    pub field: u16,
    /// A long field (`LOF` set) is one line longer. Always, for the hardwired
    /// counters, which is why a non-interlaced display runs 313-line fields;
    /// for a programmed `VTOTAL`, only while `LACE` is set (Appendix C, *Multi-
    /// Sync and Bi-Sync Monitors*: "the number of lines in a field(+1). The
    /// exception is if the INTERLACE bit is set").
    pub long_field: bool,
}

impl From<Standard> for Timing {
    fn from(std: Standard) -> Timing {
        Timing {
            line: SHORT_LINE,
            alternate: std.long_lines(),
            field: std.short_field(),
            long_field: true,
        }
    }
}

/// What crossing a count boundary did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Crossing {
    /// Still on the same line.
    None,
    /// Onto a new line of the same field.
    Line,
    /// Onto line 0 of a new field.
    Field,
}

/// The beam position and the two flip-flops that shape the next line and field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Beam {
    /// The vertical counter, `V8`–`V0`.
    pub vpos: u16,
    /// The horizontal counter, `H8`–`H1`: one count per colour clock.
    pub hpos: u16,
    /// `LOF`: this field is a long one.
    pub lof: bool,
    /// This line is a long one (NTSC only).
    pub lol: bool,
}

impl Default for Beam {
    fn default() -> Beam {
        Beam::power_on()
    }
}

impl Beam {
    /// The top left of a long field, on a short line.
    #[must_use]
    pub const fn power_on() -> Beam {
        Beam {
            vpos: 0,
            hpos: 0,
            lof: true,
            lol: false,
        }
    }

    /// Counts on the current line.
    #[must_use]
    #[inline]
    pub fn line_len(&self, t: impl Into<Timing>) -> u16 {
        let t = t.into();
        if t.alternate && self.lol {
            t.line + 1
        } else {
            t.line
        }
    }

    /// Lines in the current field.
    #[must_use]
    #[inline]
    pub fn field_len(&self, t: impl Into<Timing>) -> u16 {
        let t = t.into();
        t.field + (self.lof && t.long_field) as u16
    }

    /// Move one count. `lace` is `BPLCON0`'s interlace bit, which decides
    /// whether `LOF` toggles when a new field starts.
    ///
    /// A counter a guest has written past its end (`VHPOSW`) wraps at the next
    /// count rather than running on: nothing in the manual says what the
    /// silicon does there, and wrapping at the first boundary it reaches is the
    /// choice that cannot run a counter away.
    #[inline]
    pub fn advance(&mut self, t: impl Into<Timing>, lace: bool) -> Crossing {
        let t = t.into();
        self.hpos += 1;
        if self.hpos < self.line_len(t) {
            return Crossing::None;
        }
        self.hpos = 0;
        if t.alternate {
            self.lol = !self.lol;
        }
        self.vpos += 1;
        if self.vpos < self.field_len(t) {
            return Crossing::Line;
        }
        self.vpos = 0;
        if lace {
            self.lof = !self.lof;
        }
        Crossing::Field
    }

    /// Counts from here until the beam arrives at the start of the next line.
    ///
    /// Always at least one.
    #[must_use]
    pub fn ticks_to_line(&self, t: impl Into<Timing>) -> u64 {
        u64::from(self.line_len(t).saturating_sub(self.hpos).max(1))
    }

    /// Counts from here until the beam arrives at line 0 of the next field.
    ///
    /// Always at least one. Does not know whether `LOF` will toggle — it
    /// cannot matter, because the field the beam is in is already sized.
    #[must_use]
    pub fn ticks_to_field(&self, t: impl Into<Timing>) -> u64 {
        let t = t.into();
        let mut ticks = self.ticks_to_line(t);
        let remaining = self
            .field_len(t)
            .saturating_sub(self.vpos)
            .saturating_sub(1);
        let lines = u64::from(remaining);
        ticks += lines * u64::from(t.line);
        if t.alternate {
            // The lines after this one alternate, starting with the opposite of
            // this one's length.
            let longs = if self.lol {
                lines / 2
            } else {
                lines.div_ceil(2)
            };
            ticks += longs;
        }
        ticks
    }

    /// Where the beam is `n` counts from now, and whether it crossed into a new
    /// field on the way (and so which `LOF` it holds depends on `lace`).
    #[must_use]
    pub fn ahead(&self, t: impl Into<Timing>, lace: bool, n: u64) -> Beam {
        let t = t.into();
        let mut b = *self;
        let mut n = n;
        while n > 0 {
            let to_line = b.ticks_to_line(t);
            if n < to_line {
                b.hpos += n as u16;
                break;
            }
            // Land on the last count of the line, then take the boundary.
            b.hpos = b.line_len(t).saturating_sub(1);
            n -= to_line;
            b.advance(t, lace);
        }
        b
    }

    /// `VPOSR`'s beam bits: `LOF` and `V8`. The identification bits are the
    /// chip's, not the beam's.
    #[must_use]
    pub const fn vposr(&self) -> u16 {
        ((self.lof as u16) << 15) | ((self.vpos >> 8) & 1)
    }

    /// `VHPOSR`: `V7`–`V0` in the high byte, `H8`–`H1` in the low.
    #[must_use]
    pub const fn vhposr(&self) -> u16 {
        ((self.vpos & 0xff) << 8) | (self.hpos & 0xff)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn walk(std: Standard, lace: bool, mut b: Beam, n: u64) -> Beam {
        for _ in 0..n {
            b.advance(std, lace);
        }
        b
    }

    #[test]
    fn a_pal_field_is_313_lines_of_227_counts() {
        let b = Beam::power_on();
        assert_eq!(b.ticks_to_field(Standard::Pal), 313 * 227);
        let end = walk(Standard::Pal, false, b, 313 * 227 - 1);
        assert_eq!((end.vpos, end.hpos), (312, 226), "the manual's (226,312)");
        let next = walk(Standard::Pal, false, end, 1);
        assert_eq!((next.vpos, next.hpos, next.lof), (0, 0, true));
    }

    #[test]
    fn ntsc_lines_alternate_and_the_four_field_pattern_follows() {
        let std = Standard::Ntsc;
        let b = Beam::power_on();
        assert_eq!(b.line_len(std), 227);
        let second = walk(std, false, b, 227);
        assert_eq!(
            (second.vpos, second.hpos, second.line_len(std)),
            (1, 0, 228)
        );
        // 263 lines, starting short: 132 short and 131 long.
        assert_eq!(b.ticks_to_field(std), 263 * 227 + 131);
        // Interlaced: fields of 263 then 262, which is the manual's pattern.
        let f2 = walk(std, true, b, b.ticks_to_field(std));
        assert_eq!((f2.vpos, f2.hpos, f2.lof), (0, 0, false));
        assert_eq!(f2.field_len(std), 262);
        assert!(
            f2.lol,
            "a 263-line field starting short ends short, so this starts long"
        );
        let f3 = walk(std, true, f2, f2.ticks_to_field(std));
        assert!(f3.lof && f3.lol, "262 lines keep the phase");
    }

    #[test]
    fn looking_ahead_agrees_with_walking() {
        for std in [Standard::Pal, Standard::Ntsc] {
            for lace in [false, true] {
                let start = Beam {
                    vpos: 100,
                    hpos: 17,
                    lof: true,
                    lol: true,
                };
                for n in [0, 1, 209, 210, 211, 5000, 71_051, 200_000] {
                    assert_eq!(
                        start.ahead(std, lace, n),
                        walk(std, lace, start, n),
                        "{std:?} {n}"
                    );
                }
                let to = start.ticks_to_field(std);
                let there = walk(std, lace, start, to);
                assert_eq!((there.vpos, there.hpos), (0, 0));
                let short = walk(std, lace, start, to - 1);
                assert_ne!(short.vpos, 0);
            }
        }
    }

    #[test]
    fn the_register_views_split_v8_off() {
        let b = Beam {
            vpos: 0x12c,
            hpos: 0xe2,
            lof: true,
            lol: false,
        };
        assert_eq!(b.vposr(), 0x8001);
        assert_eq!(b.vhposr(), 0x2ce2);
    }
}
