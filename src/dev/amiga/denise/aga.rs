//! Lisa: the AA chip set's video chip, the display half — the colour table
//! first.
//!
//! The 8362 and the 8373 are rendered by [`super::render`]; this is the whole
//! of `revision = "aga"`. It is a separate function rather than more branches
//! in that one because the two chips do not share a pixel grid: an 8362
//! decides everything on a 140 ns low-resolution pixel and an 8373 on a 70 ns
//! one, while **Lisa decides everything on a 35 ns one** — "All programmable
//! comparators with the exception of `VHPOSW` have 35nSec resolution:
//! `DIWHIGH`, `HBSTOP`, `SPRCTL`, `BPLCON1`" (*Specification for the Advanced
//! Amiga (AA) Chip Set*, Commodore-Amiga, §2, *Horizontal Comparators*). Every
//! position below is therefore in **quarters**: 35 ns each, four to a
//! low-resolution pixel, and the unit the picture's columns are in.
//!
//! # The source
//!
//! Everything here is from the *Specification for the Advanced Amiga (AA) Chip
//! Set*, cited by its section: §1 *Summary of new features for AA*, §2
//! *Explanation of new features*, §3 the register list, §4 the per-register
//! pages, §5 *New LISA Display & Sprite Modes*. Where it is silent, the
//! comment says so and says what was chosen instead. **No emulator source of
//! any licence was consulted**, and the same is true of every FPGA
//! reimplementation: the document was fetched on its own.
//!
//! # What is modelled so far
//!
//! | | |
//! | --- | --- |
//! | the 256-entry 24-bit colour table | modelled: `BANK`, `LOCT`, and the automatic four-to-eight-bit extension |
//! | an 8362's playfields through it | modelled: up to six planes, single and 3 + 3 dual playfield (playfield 2 at colour 8, as before), the old four-bit scroll, the window, `DIWHIGH` as on an 8373 |
//! | EHB, and `KILLEHB` | modelled: "EHB is invoked whenever SHRES = HIRES = HAMEN = DPF = 0 and BPU = 6" |
//! | `BPLCON0`'s `ECSENA` gate | modelled: it inhibits `BRDRBLNK` |
//! | `BPU3`, `BPLCON4`, `FMODE`, the eight-bit scroll, `PF2OF` | latched only, for now |
//! | HAM | not yet: `HAMEN` is ignored |
//! | sprites, collisions, `CLXCON2` | not yet: an AA part shows no sprites and detects no collisions |
//!
//! # Where a fetch's first pixel lands
//!
//! As for the 8373: the document gives no formula, so the 3rd-edition
//! manual's arithmetic is kept — one fetch block and half a colour clock after
//! the fetch. [`super::Setup`] has the derivation. That is an inference.

use super::{
    BRDRBLNK, DBLPF, ENBPLCN3 as ECSENA, Fetch, HIRES, HOMOD, KILLEHB, LACE, Line, MAX_FETCH_WORDS,
    Revision, SHRES, Stamp, State,
};

/// Quarters — 35 ns pixels — in one low-resolution pixel.
const QUARTERS: i32 = 4;

/// The per-line decisions, recomputed whenever a register that shapes them
/// changes.
#[derive(Debug, Clone, Copy)]
struct Setup {
    /// Planes enabled, 0–6.
    planes: usize,
    /// Quarters per bitplane pixel: 4 in `LORES`, 2 in `HIRES`, 1 in `SHRES`.
    step: i32,
    dual: bool,
    ehb: bool,
    /// Per plane, the quarter its first bit is displayed at.
    start: [i32; 8],
    /// Inside the window vertically.
    inside_v: bool,
    /// The window's horizontal edges, in quarters.
    hstart: i32,
    hstop: i32,
    /// `BRDRBLNK` and `ECSENA`: the border is black rather than colour 0.
    blank: bool,
}

impl Setup {
    fn of(regs: &super::Regs, fetch_start: u16, vpos: u16) -> Setup {
        let con0 = regs.bplcon0;
        // The 3rd-edition BPU: three bits and "111 not used". Six is the most
        // an 8362's playfield has.
        let planes = usize::from((con0 >> 12) & 7).min(6);
        // "SHRES Super hi-res mode (35ns pixel width)"; with both set
        // SuperHires wins, as on an 8373.
        let shres = con0 & SHRES != 0;
        let hires = !shres && con0 & HIRES != 0;
        let dual = con0 & DBLPF != 0;
        let step = if shres {
            1
        } else if hires {
            2
        } else {
            QUARTERS
        };
        // "As before, EHB is invoked whenever SHRES = HIRES = HAMEN = DPF = 0
        // and BPU = 6. Please note that starting with ECS DENISE there is a
        // bit in BPLCON2 which disables this mode (KILLEHB)."
        let ehb = con0 & HOMOD == 0
            && !dual
            && !hires
            && !shres
            && planes == 6
            && regs.bplcon2 & KILLEHB == 0;

        // Where the first fetched bit is displayed: the 3rd-edition manual's
        // arithmetic, in quarters, because the AA document gives no formula of
        // its own. See `super::Setup`.
        let lead: i32 = if shres {
            5
        } else if hires {
            9
        } else {
            17
        };
        let base = QUARTERS * (2 * i32::from(fetch_start) + lead);
        // The 3rd-edition four-bit delays, whole low-resolution pixels: odd
        // planes take playfield 1's, even planes playfield 2's.
        let delay = [
            QUARTERS * i32::from(regs.bplcon1 & 0xf),
            QUARTERS * i32::from((regs.bplcon1 >> 4) & 0xf),
        ];
        let mut start = [0i32; 8];
        for (p, s) in start.iter_mut().enumerate() {
            *s = base + delay[p % 2];
        }

        // `DIWHIGH` as an 8373 reads it: start V10-V8 at bits 2-0 and H10 at 5,
        // stop V10-V8 at 10-8 and H10 at 13.
        let (vstart, vstop, hstart, hstop) = if regs.diwhigh_on {
            let high = regs.diwhigh;
            (
                ((high & 7) << 8) | (regs.diwstrt >> 8),
                (((high >> 8) & 7) << 8) | (regs.diwstop >> 8),
                QUARTERS * (i32::from((high >> 5) & 1) << 8 | i32::from(regs.diwstrt & 0xff)),
                QUARTERS * (i32::from((high >> 13) & 1) << 8 | i32::from(regs.diwstop & 0xff)),
            )
        } else {
            let vstop_lo = regs.diwstop >> 8;
            (
                regs.diwstrt >> 8,
                vstop_lo | if vstop_lo & 0x80 == 0 { 0x100 } else { 0 },
                QUARTERS * i32::from(regs.diwstrt & 0xff),
                QUARTERS * (0x100 | i32::from(regs.diwstop & 0xff)),
            )
        };

        // "ECSENA … forces the following bits to their default low settings:
        // BRDRBLNK, BRDNTRAN, ZDCLKEN, EXTBLKEN, and BRDRSPRT" (§2).
        let enabled = con0 & ECSENA != 0;

        Setup {
            planes,
            step,
            dual,
            ehb,
            start,
            inside_v: vstart <= vpos && vpos < vstop,
            hstart,
            hstop,
            blank: enabled && regs.bplcon3 & BRDRBLNK != 0,
        }
    }

    /// The eight plane bits at quarter `q`, plane 1 in bit 0.
    #[inline]
    fn bits(&self, fetch: &Fetch<'_>, q: i32) -> u8 {
        let mut bits = 0u8;
        for p in 0..self.planes {
            let rel = q - self.start[p];
            if rel < 0 {
                continue;
            }
            let bit = (rel / self.step) as usize;
            let words = &fetch.planes[p][..fetch.planes[p].len().min(MAX_FETCH_WORDS)];
            if let Some(word) = words.get(bit / 16) {
                bits |= (((word >> (15 - bit % 16)) & 1) as u8) << p;
            }
        }
        bits
    }
}

/// Half intensity on eight-bit guns, for EHB: "The color register output
/// selected by 5 bitplanes is shifted to half intensity by the 6th bit plane"
/// (§4, `BPLCON0`).
#[inline]
const fn half(colour: u32) -> u32 {
    (colour >> 1) & 0x007f_7f7f
}

/// Render one line of an AA field.
pub(super) fn render(st: &mut State, line: &Line<'_>) {
    let field = st.fields;
    let vpos = line.vpos;
    st.apply_before(
        Revision::Aga,
        Stamp {
            field,
            vpos,
            hpos: 0,
        },
    );
    st.clocks = st.clocks.wrapping_add(u64::from(line.clocks));

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

    // Low-resolution pixels across, then quarters. The same bound the older
    // path uses and for the same reason: the serializer runs past the beam
    // counter's end, and a beam source that reports an absurd length must not
    // make a line cost more than twice the longest real one.
    let span = (u32::from(line.clocks) * 2)
        .max(u32::from(layout.left) + u32::from(layout.span))
        .min(1024) as i32
        * QUARTERS;
    let mut setup = Setup::of(&st.regs, line.fetch.start, vpos);

    let queued_here = st
        .pending
        .front()
        .is_some_and(|c| c.at.field == field && c.at.vpos == vpos);
    if rows == [None, None] && !setup.inside_v && !queued_here {
        return;
    }

    // Lisa's picture is always in quarters, so nothing here ever rescales.
    let width = layout.width() as i32;
    let left = i32::from(layout.left) * QUARTERS;

    for q in 0..span {
        let mut changed = false;
        while let Some(change) = st.pending.front().copied() {
            // A beam position is in colour clocks, two low-resolution pixels
            // and so eight quarters each.
            if change.at.field != field
                || change.at.vpos != vpos
                || i32::from(change.at.hpos) * 2 * QUARTERS > q
            {
                break;
            }
            st.pending.pop_front();
            st.regs.apply(Revision::Aga, change.offset, change.value);
            changed = true;
        }
        if changed {
            setup = Setup::of(&st.regs, line.fetch.start, vpos);
        }
        let regs = &st.regs;

        let inside = setup.inside_v && setup.hstart <= q && q < setup.hstop;
        let border = if setup.blank { 0 } else { regs.palette[0] };

        let bits = setup.bits(&line.fetch, q);

        let colour = if !inside {
            border
        } else {
            let background = regs.palette[0];
            if setup.dual {
                // As on an 8362: playfield 1 is planes 1, 3, 5 and playfield
                // 2 is planes 2, 4, 6, at colour 8.
                let odd = (bits & 1) | ((bits >> 1) & 2) | ((bits >> 2) & 4) | ((bits >> 3) & 8);
                let even =
                    ((bits >> 1) & 1) | ((bits >> 2) & 2) | ((bits >> 3) & 4) | ((bits >> 4) & 8);
                let show1 = odd != 0;
                let show2 = even != 0;
                let pf2_first = regs.bplcon2 & (1 << 6) != 0;
                let pf2 = || regs.palette[8 + usize::from(even)];
                match (show1, show2) {
                    (true, true) if pf2_first => pf2(),
                    (true, _) => regs.palette[usize::from(odd)],
                    (false, true) => pf2(),
                    (false, false) => background,
                }
            } else if bits != 0 {
                if setup.ehb && bits & 0x20 != 0 {
                    half(regs.palette[usize::from(bits & 0x1f)])
                } else {
                    regs.palette[usize::from(bits)]
                }
            } else {
                background
            }
        };

        let col = q - left;
        if (0..width).contains(&col) {
            // The genlock T bit rides in the palette entry and never reaches
            // the RGB pins.
            let colour = colour & 0x00ff_ffff;
            for row in rows.into_iter().flatten() {
                st.frame[row * width as usize + col as usize] = colour;
            }
        }
    }
}

#[cfg(test)]
mod tests;
