//! The S-PPU's beam geometry: how long a dot, a line and a frame are.
//!
//! Everything here is a pure function of the region, the line number, the
//! interlace field and the interlace setting, so the S-CPU can ask the same
//! questions the PPU answers without holding its lock.
//!
//! # The line
//!
//! A line is **1364 master clocks** and the H counter runs `0..=339`: 340 dots,
//! 338 of four clocks and **two of six** — dots 323 and 327 (nocash fullsnes,
//! "SNES Timing H/V Counters"). The SNESdev wiki's *Timing* page counts the same
//! 1364 clocks as "336 dots of 4 clocks and 4 dots of 5"; the total, which is the
//! only thing a guest can observe without a light gun, agrees, and the brief this
//! module was written against asked for fullsnes's two long dots.
//!
//! Two lines a frame may differ (both from the wiki's *Timing* page, and both in
//! vertical blank, so they never move a visible pixel):
//!
//! * **short**: NTSC, interlace off, field 1, V = 240 — 1360 clocks, 340 dots
//!   of four, no long dots;
//! * **long**: PAL, interlace on, field 1, V = 311 — 1368 clocks, 341 dots,
//!   the long dots included (H reaches 340).
//!
//! # The frame
//!
//! 262 lines on NTSC and 312 on PAL, plus one in interlace mode on the frames
//! whose field bit is 0 (the wiki: "1 extra scanline will appear with each even
//! frame"; fullsnes: V reaches 262 "on alternate frames during interlace").
//! Which parity counts as "even" is the one reading here that is an inference:
//! field 0 is taken to be the even frame.
//!
//! V = 0 is never displayed. Lines 1..=224 — or 1..=239 with `SETINI` overscan
//! — carry the picture, and vertical blank begins on the line after (the wiki:
//! "V=225 or V=240, based on the SETINI overscan setting").

/// Master clocks in an ordinary line.
pub const MASTER_PER_LINE: u32 = 1364;

/// The two dots that are six master clocks long rather than four.
pub const LONG_DOTS: [u16; 2] = [323, 327];

/// Dots in an ordinary line: H runs `0..=339`.
pub const DOTS_PER_LINE: u16 = 340;

/// The dot at which the H-blank level falls (anomie's timing notes as
/// fullsnes repeats them: the `HVBJOY` H-blank flag clears at H = 1).
pub const HBLANK_END_DOT: u16 = 1;

/// The dot at which a visible line's picture begins: master clock 88 of the
/// line (the wiki's *Timing* page), H = 22 (fullsnes: "22-277 being visible").
///
/// The renderer draws the whole line here, from the registers as they stand —
/// see the module docs of [`super`] for why that point.
pub const RENDER_DOT: u16 = 22;

/// The dot at which the H-blank level rises (H = 274, the `HVBJOY` flag).
pub const HBLANK_START_DOT: u16 = 274;

/// The first line of vertical blank with overscan off.
pub const VBLANK_LINE: u16 = 225;

/// The first line of vertical blank with `SETINI` overscan on.
pub const VBLANK_LINE_OVERSCAN: u16 = 240;

/// Which television standard the console was built for.
///
/// A construction property (`region = "pal"`), never a `#[cfg]`. It changes
/// the frame's line count, which line is short or long, and `STAT78` bit 4;
/// the master-clock *frequency* lives in the machine file's oscillator, where
/// `ROADMAP.md` §4.2 puts it — [`Region::master_clock`] is only what that file
/// is expected to say.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum Region {
    /// 60 Hz, 262 lines (263 on an interlaced even frame).
    #[default]
    Ntsc,
    /// 50 Hz, 312 lines (313 on an interlaced even frame).
    Pal,
}

impl Region {
    /// The names a machine file may write.
    pub const NAMES: &'static [&'static str] = &["ntsc", "pal"];

    /// Look one up by the name a machine file writes.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Region> {
        match name {
            "ntsc" => Some(Region::Ntsc),
            "pal" => Some(Region::Pal),
            _ => None,
        }
    }

    /// The name a machine file writes.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Region::Ntsc => "ntsc",
            Region::Pal => "pal",
        }
    }

    /// The master clock as an exact `(numerator, denominator)` in hertz.
    ///
    /// NTSC is six times the colour subcarrier, `6 × 315/88 MHz` =
    /// 236 250 000 / 11 Hz ≈ 21.477 MHz; PAL is `24/5` of the PAL subcarrier,
    /// 4 433 618.75 Hz × 4.8 = 21 281 370 Hz (fullsnes, "SNES Timings").
    #[must_use]
    pub const fn master_clock(self) -> (u64, u64) {
        match self {
            Region::Ntsc => (236_250_000, 11),
            Region::Pal => (21_281_370, 1),
        }
    }

    /// Lines in a non-interlaced frame.
    #[must_use]
    pub const fn base_lines(self) -> u16 {
        match self {
            Region::Ntsc => 262,
            Region::Pal => 312,
        }
    }

    /// Lines in a frame with this interlace setting and field bit.
    #[must_use]
    pub const fn lines_per_frame(self, interlace: bool, field: bool) -> u16 {
        self.base_lines() + (interlace && !field) as u16
    }

    /// Master clocks in line `v`.
    #[must_use]
    pub const fn line_length(self, v: u16, field: bool, interlace: bool) -> u32 {
        match self {
            Region::Ntsc if !interlace && field && v == 240 => 1360,
            Region::Pal if interlace && field && v == 311 => 1368,
            _ => MASTER_PER_LINE,
        }
    }

    /// Whether line `v` has the two six-clock dots. Only the short line lacks
    /// them.
    #[must_use]
    pub const fn has_long_dots(self, v: u16, field: bool, interlace: bool) -> bool {
        self.line_length(v, field, interlace) != 1360
    }

    /// Dots in line `v`: 340, or 341 on the PAL long line.
    #[must_use]
    pub const fn dots_in_line(self, v: u16, field: bool, interlace: bool) -> u16 {
        if self.line_length(v, field, interlace) == 1368 {
            DOTS_PER_LINE + 1
        } else {
            DOTS_PER_LINE
        }
    }
}

/// The master clock within its line at which `dot` begins.
///
/// `dot` may be one past the last, which gives the line's length.
#[must_use]
pub const fn dot_start(dot: u16, long_dots: bool) -> u32 {
    let base = 4 * dot as u32;
    if long_dots {
        base + 2 * (dot > LONG_DOTS[0]) as u32 + 2 * (dot > LONG_DOTS[1]) as u32
    } else {
        base
    }
}

/// The dot that master clock `pos` of its line falls in.
#[must_use]
pub const fn dot_at(pos: u32, long_dots: bool) -> u16 {
    if !long_dots {
        return (pos / 4) as u16;
    }
    let first = dot_start(LONG_DOTS[0], true); // 1292
    let second = dot_start(LONG_DOTS[1], true); // 1310
    if pos < first {
        (pos / 4) as u16
    } else if pos < first + 6 {
        LONG_DOTS[0]
    } else if pos < second {
        LONG_DOTS[0] + 1 + ((pos - first - 6) / 4) as u16
    } else if pos < second + 6 {
        LONG_DOTS[1]
    } else {
        LONG_DOTS[1] + 1 + ((pos - second - 6) / 4) as u16
    }
}

/// One frame of the beam, pinned to the master-clock timeline.
///
/// Everything about where the beam is at a given master tick follows from
/// these five values, by integer arithmetic alone. That is what lets a second
/// chip on the same crystal — the 5A22, whose H/V timer and HDMA are beam
/// positions — schedule against its *own* tick rather than against wherever
/// the PPU happens to have been caught up to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Frame {
    /// The television standard.
    pub region: Region,
    /// The master tick at which dot 0 of line 0 begins.
    pub start: u64,
    /// The interlace field, `STAT78` bit 7.
    pub field: bool,
    /// Whether the frame is interlaced (`SETINI` bit 0 as latched at its
    /// start).
    pub interlace: bool,
    /// Lines in the frame.
    pub lines: u16,
}

impl Frame {
    /// Master clocks from the start of the frame to the start of line `v`.
    ///
    /// `v` may be [`Frame::lines`], which gives the frame's length.
    #[must_use]
    pub const fn line_start(&self, v: u16) -> u64 {
        let mut clocks = v as u64 * MASTER_PER_LINE as u64;
        // At most one line of a frame is not 1364 clocks: NTSC's short line
        // 240 or PAL's long line 311.
        if v > 240 && self.region.line_length(240, self.field, self.interlace) == 1360 {
            clocks -= 4;
        }
        if v > 311 && self.region.line_length(311, self.field, self.interlace) == 1368 {
            clocks += 4;
        }
        clocks
    }

    /// Master clocks in the frame.
    #[must_use]
    pub const fn length(&self) -> u64 {
        self.line_start(self.lines)
    }

    /// The master tick of the first clock *after* the frame: the next one's
    /// [`start`](Frame::start).
    #[must_use]
    pub const fn end(&self) -> u64 {
        self.start.saturating_add(self.length())
    }

    /// Whether master tick `tick` falls in this frame.
    #[must_use]
    pub const fn contains(&self, tick: u64) -> bool {
        tick >= self.start && tick < self.end()
    }

    /// The line master tick `tick` falls in, if it is in this frame.
    #[must_use]
    pub const fn line_at(&self, tick: u64) -> Option<u16> {
        if !self.contains(tick) {
            return None;
        }
        let into = tick - self.start;
        // The estimate is exact or one line early (a short line before it
        // moves every later boundary four clocks forward) or one late (a long
        // one), so correct it against the real boundaries.
        let mut v = (into / MASTER_PER_LINE as u64) as u16;
        if v >= self.lines {
            v = self.lines - 1;
        }
        while v > 0 && self.line_start(v) > into {
            v -= 1;
        }
        while v + 1 < self.lines && self.line_start(v + 1) <= into {
            v += 1;
        }
        Some(v)
    }

    /// The master tick at which dot `h` of line `v` begins, or `None` if this
    /// frame has no such line or that line no such dot.
    #[must_use]
    pub const fn tick_of(&self, v: u16, h: u16) -> Option<u64> {
        if v >= self.lines || h >= self.region.dots_in_line(v, self.field, self.interlace) {
            return None;
        }
        let long = self.region.has_long_dots(v, self.field, self.interlace);
        Some(self.start + self.line_start(v) + dot_start(h, long) as u64)
    }

    /// The frame that follows this one, given the interlace setting it will
    /// latch at its start.
    #[must_use]
    pub const fn next(&self, interlace: bool) -> Frame {
        let field = !self.field;
        Frame {
            region: self.region,
            start: self.end(),
            field,
            interlace,
            lines: self.region.lines_per_frame(interlace, field),
        }
    }

    /// The frame before this one, given the interlace setting it had — or
    /// `None` if that would start before tick zero.
    #[must_use]
    pub const fn prev(&self, interlace: bool) -> Option<Frame> {
        let field = !self.field;
        let mut frame = Frame {
            region: self.region,
            start: 0,
            field,
            interlace,
            lines: self.region.lines_per_frame(interlace, field),
        };
        match self.start.checked_sub(frame.length()) {
            Some(start) => {
                frame.start = start;
                Some(frame)
            }
            None => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_frame_places_every_line_where_summing_the_lines_does() {
        for region in [Region::Ntsc, Region::Pal] {
            for interlace in [false, true] {
                for field in [false, true] {
                    let frame = Frame {
                        region,
                        start: 1_000_000,
                        field,
                        interlace,
                        lines: region.lines_per_frame(interlace, field),
                    };
                    let mut sum = 0u64;
                    for v in 0..frame.lines {
                        assert_eq!(frame.line_start(v), sum, "{frame:?} line {v}");
                        let len = u64::from(region.line_length(v, field, interlace));
                        // Both ends of the line, and the tick before it.
                        assert_eq!(frame.line_at(frame.start + sum), Some(v));
                        assert_eq!(frame.line_at(frame.start + sum + len - 1), Some(v));
                        assert_eq!(frame.tick_of(v, 0), Some(frame.start + sum));
                        sum += len;
                    }
                    assert_eq!(frame.length(), sum);
                    assert_eq!(frame.line_at(frame.start - 1), None);
                    assert_eq!(frame.line_at(frame.end()), None);
                    assert_eq!(frame.tick_of(frame.lines, 0), None);
                    let next = frame.next(interlace);
                    assert_eq!(next.start, frame.end());
                    assert_eq!(next.prev(interlace), Some(frame));
                }
            }
        }
    }

    #[test]
    fn an_ordinary_line_is_1364_clocks_of_340_dots() {
        assert_eq!(dot_start(DOTS_PER_LINE, true), MASTER_PER_LINE);
        assert_eq!(dot_start(323, true) - dot_start(322, true), 4);
        assert_eq!(dot_start(324, true) - dot_start(323, true), 6);
        assert_eq!(dot_start(328, true) - dot_start(327, true), 6);
        assert_eq!(dot_start(DOTS_PER_LINE, false), 1360);
    }

    #[test]
    fn dot_at_inverts_dot_start() {
        for long in [false, true] {
            for dot in 0..=DOTS_PER_LINE {
                let start = dot_start(dot, long);
                assert_eq!(dot_at(start, long), dot, "dot {dot} long {long}");
                if dot > 0 {
                    assert_eq!(dot_at(start - 1, long), dot - 1);
                }
            }
        }
    }

    #[test]
    fn the_short_and_long_lines_are_where_the_wiki_puts_them() {
        assert_eq!(Region::Ntsc.line_length(240, true, false), 1360);
        assert_eq!(Region::Ntsc.line_length(240, false, false), 1364);
        assert_eq!(Region::Ntsc.line_length(240, true, true), 1364);
        assert_eq!(Region::Pal.line_length(311, true, true), 1368);
        assert_eq!(Region::Pal.line_length(311, true, false), 1364);
        assert_eq!(Region::Pal.dots_in_line(311, true, true), 341);
        assert_eq!(dot_start(341, true), 1368);
    }

    #[test]
    fn interlace_adds_a_line_on_even_frames() {
        assert_eq!(Region::Ntsc.lines_per_frame(false, false), 262);
        assert_eq!(Region::Ntsc.lines_per_frame(true, false), 263);
        assert_eq!(Region::Ntsc.lines_per_frame(true, true), 262);
        assert_eq!(Region::Pal.lines_per_frame(false, true), 312);
        assert_eq!(Region::Pal.lines_per_frame(true, false), 313);
    }
}
