//! The Enhanced Chip Set Agnus: what an 8372A or an 8375 does that an 8370 or
//! 8371 does not.
//!
//! Everything here is *Amiga Hardware Reference Manual*, 3rd edition, Appendix
//! C ("Enhanced Chip Set"), whose register table (*ECS Registers*) lists the
//! new and changed Agnus rows. What each one does in this model:
//!
//! | register | what it does here |
//! | --- | --- |
//! | `VPOSR` | the ECS identification in bits 14–8 ([`agnus_id`]), `LOL` in bit 7, and `V10`/`V9` in bits 2–1 — *Determining Chip Revisions* prints the register "LOF I6 … I0 LOL -- -- -- -- v10 v9 V8" |
//! | `VPOSW` | writes `V10`–`V8` |
//! | `BEAMCON0` | `PAL`, `VARBEAMEN`, `LOLDIS`, `VARHSYEN`, `VARVSYEN` and `VARVBEN` act; `HARDDIS` and the polarity, redirection, light-pen and `DUAL` bits are held, because the hardwired blanking they would switch off is not a picture this model cuts ([`raster`]) and nothing here drives the pins they shape |
//! | `HTOTAL`, `VTOTAL` | the line and field lengths under `VARBEAMEN` ([`timing`]) |
//! | `HSSTRT`, `HSSTOP`, `VSSTRT`, `VSSTOP` | the `hsync` and `vsync` pins under `VARHSYEN`/`VARVSYEN` ([`Sync`]) |
//! | `VBSTRT`, `VBSTOP`, `HBSTRT`, `HBSTOP` | the end of vertical blanking under `VARVBEN`, and the picture a monitor shows ([`raster`]) |
//! | `HCENTER` | held: it places an interlaced vertical sync half a line over, which no pin here resolves |
//! | `DIWHIGH` | the vertical display window's `V10`–`V8` for the bitplane fetch, once written after `DIWSTRT`/`DIWSTOP` |
//! | `BPLCON0` | `SHRES`, bit 6: four words to a fetch block rather than two |
//! | `COPCON` | the copper's wider permission (`regs::ecs_copper_may_write`) |
//! | `DSKPTH`, `BLTxPTH`, `COPxLCH`, `AUDxLCH` | five high bits rather than three: the chip-RAM mask is the part's reach, 1 or 2 MiB, rather than 512 KiB |
//!
//! # The two parts
//!
//! Appendix C, *Determining Chip Revisions*, lists "8368 (hr) or 8372 (fat-hr)
//! = 20 for PAL, 30 for NTSC" — the 1 MiB **8372A** of an A500 or A2000.
//! Commodore's later register notes for the AA chip set carry the same table
//! with a row more, "8372 (Fat-hr) (agnushr), rev 5 = 22 PAL, 31 NTSC"; the
//! 2 MiB part an A500+ and an A600 carry is the **8375** (part 318069-10 PAL,
//! -11 NTSC), the 2 MiB member of that family. That the 2 MiB part is the one
//! that answers `$22`/`$31` is a reading of those two documents rather than a
//! sentence in either; `graphics.library` tests only for "20 or 30" having bit
//! 5 set ("A value of 20 or 30 indicates that the enhanced Hires Agnus is
//! present"), which both values have.
//!
//! # `BEAMCON0` out of reset
//!
//! Appendix C: "the chips from the US factory are configured for NTSC mode. In
//! order to use them on a PAL system, you may have to reset the motherboard
//! jumpers". So PAL is a strap, and `BEAMCON0`'s `PAL` bit ("Programmable pal
//! mode enable") is where the programmable half of it lives. **Reading:** the
//! register comes out of reset with `PAL` equal to the strap — the machine
//! file's `standard` — and every other bit clear, which is an ECS Agnus that
//! counts exactly like the original part until software says otherwise.
//!
//! # The programmable beam
//!
//! *Multi-Sync and Bi-Sync Monitors*: `HTOTAL` is the "Highest number count in
//! horizontal line", in "280ns increments" — colour clocks — so a line is
//! `HTOTAL + 1` counts: the manual's own VGA figure, "114.0 colorclocks per
//! scan line", is `HTOTAL = 113`. `VTOTAL` "represents the number of lines in
//! a field(+1)", and under `LACE` "the number of lines in the long field (+2)
//! and the number of lines in the short field (+1)". Both take effect with
//! `VARBEAMEN` ("Variable beam counter comparator enable"; the ECS register
//! table: "Highest number count, horiz line (VARBEAMEN=1)").
//!
//! **Readings, where the manual is silent:**
//!
//! * NTSC's alternating long and short lines follow the `PAL` bit and
//!   `LOLDIS` ("Disable long line/short line toggle"), with a programmed line
//!   as with a hardwired one: a long line is one count longer.
//! * `VERTB` stays on line 0 and the copper still restarts there: the manual
//!   defines both by the vertical counter's wrap, not by a blanking register.
//! * Vertical sync, programmed, is whole lines: high from count 0 of line
//!   `VSSTRT` to count 0 of line `VSSTOP`. `HCENTER` would move it half a line
//!   in an interlaced field; nothing that listens to this pin can tell.
//!
//! # What a monitor shows
//!
//! An original chip-set Denise lays its picture out from a fixed table — Table
//! 3-13's blanking lines and a 400-pixel span. With a programmable beam that
//! table is wrong by a factor of two, so at every field an ECS Agnus hands
//! Denise the [`Raster`] its registers describe: the first line after vertical
//! blanking and the last before it, and the columns between horizontal blanks.
//! See [`raster`] for the rules.

use super::beam::{Standard, Timing};
use crate::dev::amiga::denise::Raster;

/// Which Agnus this is.
///
/// A closed set, so a real enum: two chip revisions, and the property that
/// names them is validated against exactly these.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Revision {
    /// An original chip-set part: 8361/8367 or the A500's 8370/8371.
    Ocs,
    /// An Enhanced Chip Set part reaching `reach` bytes of chip RAM: 1 MiB is
    /// an 8372A, 2 MiB an 8375.
    Ecs {
        /// How much chip RAM the part's address lines reach.
        reach: u64,
    },
}

impl Revision {
    /// Whether this is an Enhanced Chip Set part.
    #[must_use]
    #[inline]
    pub const fn is_ecs(self) -> bool {
        matches!(self, Revision::Ecs { .. })
    }
}

/// One mebibyte: an 8372A's reach.
pub const MIB: u64 = 1 << 20;

/// `VPOSR` bits 14–8 for this part and strap.
///
/// Appendix C, *Determining Chip Revisions*, for the original parts and the
/// 8372A; Commodore's AA register notes for the later row (see the module
/// documentation for why the 8375 is that row).
#[must_use]
pub const fn agnus_id(rev: Revision, std: Standard) -> u8 {
    match (rev, std) {
        (Revision::Ocs, _) => std.agnus_id(),
        (Revision::Ecs { reach }, Standard::Pal) if reach > MIB => 0x22,
        (Revision::Ecs { reach }, Standard::Ntsc) if reach > MIB => 0x31,
        (Revision::Ecs { .. }, Standard::Pal) => 0x20,
        (Revision::Ecs { .. }, Standard::Ntsc) => 0x30,
    }
}

// Indices into the chip's `ecs` register array: `HTOTAL`…`VBSTOP` in address
// order, then `BEAMCON0`…`DIWHIGH`.
pub(super) const HTOTAL: usize = 0;
pub(super) const HSSTOP: usize = 1;
pub(super) const HBSTRT: usize = 2;
pub(super) const HBSTOP: usize = 3;
pub(super) const VTOTAL: usize = 4;
pub(super) const VSSTOP: usize = 5;
pub(super) const VBSTRT: usize = 6;
pub(super) const VBSTOP: usize = 7;
pub(super) const BEAMCON0: usize = 8;
pub(super) const HSSTRT: usize = 9;
pub(super) const VSSTRT: usize = 10;
pub(super) const DIWHIGH: usize = 12;

// BEAMCON0's bits, Appendix C, *New BEAMCON0 Register*.
/// "Disable hardwired vertical/horizontal blank".
pub const HARDDIS: u16 = 1 << 14;
/// "Use VBSTRT/STOP disable hard window stop".
pub const VARVBEN: u16 = 1 << 12;
/// "Disable long line/short line toggle".
pub const LOLDIS: u16 = 1 << 11;
/// "Variable vertical sync enable".
pub const VARVSYEN: u16 = 1 << 9;
/// "Variable horizontal sync enable".
pub const VARHSYEN: u16 = 1 << 8;
/// "Variable beam counter comparator enable".
pub const VARBEAMEN: u16 = 1 << 7;
/// "Programmable pal mode enable".
pub const PAL: u16 = 1 << 5;

/// A horizontal register: `H8`–`H1`, one count each.
const H_BITS: u16 = 0x00ff;
/// A vertical register: `V10`–`V0`.
const V_BITS: u16 = 0x07ff;

/// `BEAMCON0` as it comes out of reset: the strap in `PAL`.
#[must_use]
pub const fn beamcon0_at_reset(std: Standard) -> u16 {
    match std {
        Standard::Pal => PAL,
        Standard::Ntsc => 0,
    }
}

/// The hardwired standard `BEAMCON0` selects.
#[must_use]
#[inline]
pub const fn standard(regs: &[u16; 13]) -> Standard {
    if regs[BEAMCON0] & PAL != 0 {
        Standard::Pal
    } else {
        Standard::Ntsc
    }
}

/// The raster the counters run through, from `BEAMCON0`, `HTOTAL` and
/// `VTOTAL`; `lace` is `BPLCON0`'s interlace bit.
#[must_use]
#[inline]
pub fn timing(regs: &[u16; 13], lace: bool) -> Timing {
    let beamcon0 = regs[BEAMCON0];
    let std = standard(regs);
    let alternate = std == Standard::Ntsc && beamcon0 & LOLDIS == 0;
    if beamcon0 & VARBEAMEN != 0 {
        Timing {
            line: (regs[HTOTAL] & H_BITS) + 1,
            alternate,
            field: (regs[VTOTAL] & V_BITS) + 1,
            long_field: lace,
        }
    } else {
        Timing {
            alternate,
            ..Timing::from(std)
        }
    }
}

/// The first line after vertical blanking: `VBSTOP` under `VARVBEN`, Table
/// 3-13's hardwired line for the standard `BEAMCON0` selects otherwise.
#[must_use]
pub fn vblank_end(regs: &[u16; 13]) -> u16 {
    if regs[BEAMCON0] & VARVBEN != 0 {
        regs[VBSTOP] & V_BITS
    } else {
        super::display::vblank_stop(standard(regs))
    }
}

/// Whether `pos` is inside a window that starts at `start` and stops at
/// `stop`, wrapping round the end of its line or field when `stop < start`.
#[must_use]
#[inline]
pub const fn in_window(pos: u16, start: u16, stop: u16) -> bool {
    if start <= stop {
        start <= pos && pos < stop
    } else {
        pos >= start || pos < stop
    }
}

/// Where the two sync pulses are.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Sync {
    /// `hsync` is high from this count…
    pub h_start: u16,
    /// …to this one.
    pub h_stop: u16,
    /// `vsync` is high from this line…
    pub v_start: u16,
    /// …to this one.
    pub v_stop: u16,
}

impl Sync {
    /// The windows `BEAMCON0` selects: programmed under `VARHSYEN` and
    /// `VARVSYEN`, the original chip set's nominal ones otherwise.
    #[must_use]
    pub fn of(regs: &[u16; 13]) -> Sync {
        let beamcon0 = regs[BEAMCON0];
        let (h_start, h_stop) = if beamcon0 & VARHSYEN != 0 {
            (regs[HSSTRT] & H_BITS, regs[HSSTOP] & H_BITS)
        } else {
            (0, super::HSYNC_COUNTS)
        };
        let (v_start, v_stop) = if beamcon0 & VARVSYEN != 0 {
            (regs[VSSTRT] & V_BITS, regs[VSSTOP] & V_BITS)
        } else {
            (0, super::VSYNC_LINES)
        };
        Sync {
            h_start,
            h_stop,
            v_start,
            v_stop,
        }
    }
}

/// The part of the raster a monitor shows, for Denise to lay its picture out
/// by.
///
/// * **Vertically**, from the end of vertical blanking ([`vblank_end`]) to
///   `VBSTRT` under `VARVBEN` when that is further down, and to the end of a
///   long field otherwise — which, hardwired, is Table 3-13's 284 PAL and 242
///   NTSC lines, the original chip set's picture exactly.
/// * **Horizontally**, hardwired, the original chip set's span: 200 counts
///   from count 32, the 400 low-resolution pixels from `x = 64` that
///   `denise::OUTPUT_LEFT` documents. Under `VARBEAMEN`, the counts between
///   `HBSTOP` and `HBSTRT` when those lie in order inside the line, and the
///   whole line otherwise. **Reading:** a monitor shows what is not blanked;
///   the manual gives the registers and not the picture.
#[must_use]
pub fn raster(regs: &[u16; 13], timing: Timing) -> Raster {
    let beamcon0 = regs[BEAMCON0];
    let first_line = vblank_end(regs);
    let field_end = timing.field + u16::from(timing.long_field);
    let vbstrt = regs[VBSTRT] & V_BITS;
    let last = if beamcon0 & VARVBEN != 0 && vbstrt > first_line {
        vbstrt.min(field_end)
    } else {
        field_end
    };
    let (first_clock, clocks) = if beamcon0 & VARBEAMEN != 0 {
        let (start, stop) = (regs[HBSTOP] & H_BITS, regs[HBSTRT] & H_BITS);
        if start < stop && stop <= timing.line {
            (start, stop - start)
        } else {
            (0, timing.line)
        }
    } else {
        (32, 200)
    };
    Raster {
        first_line,
        lines: last.saturating_sub(first_line).max(1),
        first_clock,
        clocks,
        line_clocks: timing.line,
    }
}

/// `DIWHIGH`'s vertical start and stop, `V10`–`V0`, from the three registers.
///
/// Appendix C, *Display Window Specification*: bits 10–8 are the stop's
/// `V10`–`V8` and bits 2–0 the start's. (The table prints "Vertical stop"
/// beside bits 2–0 as well; the start is the only register they can belong
/// to, and bit 5 beside them is the start's `H8`.)
#[must_use]
pub const fn window_lines(diwstrt: u16, diwstop: u16, diwhigh: u16) -> (u16, u16) {
    let start = ((diwhigh & 7) << 8) | (diwstrt >> 8);
    let stop = (((diwhigh >> 8) & 7) << 8) | (diwstop >> 8);
    (start, stop)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn regs() -> [u16; 13] {
        let mut r = [0u16; 13];
        r[BEAMCON0] = PAL;
        r
    }

    #[test]
    fn out_of_reset_an_ecs_part_counts_like_the_original() {
        for std in [Standard::Pal, Standard::Ntsc] {
            let mut r = [0u16; 13];
            r[BEAMCON0] = beamcon0_at_reset(std);
            assert_eq!(timing(&r, false), Timing::from(std), "{std:?}");
            assert_eq!(timing(&r, true), Timing::from(std), "{std:?}");
            let raster = raster(&r, timing(&r, false));
            let want = Raster::standard(match std {
                Standard::Pal => crate::dev::amiga::denise::Standard::Pal,
                Standard::Ntsc => crate::dev::amiga::denise::Standard::Ntsc,
            });
            assert_eq!(raster, want, "{std:?}");
        }
    }

    #[test]
    fn the_pal_bit_switches_the_hardwired_counts() {
        let mut r = regs();
        r[BEAMCON0] = 0;
        assert_eq!(timing(&r, false), Timing::from(Standard::Ntsc));
        r[BEAMCON0] = LOLDIS;
        assert!(!timing(&r, false).alternate, "LOLDIS stops the toggle");
    }

    #[test]
    fn varbeamen_takes_htotal_and_vtotal_as_highest_counts() {
        let mut r = regs();
        r[BEAMCON0] = PAL | VARBEAMEN;
        r[HTOTAL] = 113; // "VGA (525 lines, 114.0 colorclocks per scan line)"
        r[VTOTAL] = 524;
        let t = timing(&r, false);
        assert_eq!((t.line, t.field, t.long_field), (114, 525, false));
        assert!(timing(&r, true).long_field, "interlaced: long field +2");
    }

    #[test]
    fn the_ids_are_appendix_cs_and_the_aa_notes() {
        let one = Revision::Ecs { reach: MIB };
        let two = Revision::Ecs { reach: 2 * MIB };
        assert_eq!(agnus_id(Revision::Ocs, Standard::Pal), 0x00);
        assert_eq!(agnus_id(Revision::Ocs, Standard::Ntsc), 0x10);
        assert_eq!(agnus_id(one, Standard::Pal), 0x20);
        assert_eq!(agnus_id(one, Standard::Ntsc), 0x30);
        assert_eq!(agnus_id(two, Standard::Pal), 0x22);
        assert_eq!(agnus_id(two, Standard::Ntsc), 0x31);
    }

    #[test]
    fn windows_wrap_when_they_stop_before_they_start() {
        assert!(in_window(5, 3, 8));
        assert!(!in_window(8, 3, 8));
        assert!(in_window(250, 200, 10));
        assert!(in_window(2, 200, 10));
        assert!(!in_window(100, 200, 10));
    }

    #[test]
    fn a_programmed_raster_is_the_unblanked_part() {
        let mut r = regs();
        r[BEAMCON0] = PAL | VARBEAMEN | VARVBEN;
        r[HTOTAL] = 113;
        r[VTOTAL] = 524;
        r[HBSTOP] = 20;
        r[HBSTRT] = 110;
        r[VBSTOP] = 30;
        r[VBSTRT] = 510;
        let raster = raster(&r, timing(&r, false));
        assert_eq!(
            (
                raster.first_line,
                raster.lines,
                raster.first_clock,
                raster.clocks
            ),
            (30, 480, 20, 90)
        );
        assert_eq!(raster.line_clocks, 114);
    }

    #[test]
    fn diwhigh_carries_both_ends_top_bits() {
        assert_eq!(window_lines(0x2c81, 0x2cc1, 0x0100), (0x2c, 0x12c));
        assert_eq!(
            window_lines(0x1081, 0x00c1, 0x0200 | 0x0001),
            (0x110, 0x200)
        );
    }
}
