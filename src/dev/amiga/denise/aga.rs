//! Lisa: the AA chip set's video chip, the display half.
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
//! comment says so and says what was chosen instead — there is nothing here
//! that the document does not state or that a marked inference does not
//! explain. **No emulator source of any licence was consulted**, and the same
//! is true of every FPGA reimplementation: the document was fetched on its
//! own.
//!
//! # What Lisa takes from Alice
//!
//! The seam is [`super::Fetch`], documented where it is declared. In short:
//!
//! * **Bitplanes**: eight word streams, unchanged in shape. A wider `FMODE`
//!   needs nothing new, because "the parallel to serial conversion is
//!   triggered whenever bit plane #1 is written, indicating the completion of
//!   all bit planes for that word (16/32/64 pixels). The MSB is output first,
//!   and is therefore always on the left" (§4, `BPLxDAT`) — a fetch of any
//!   width is that many consecutive pixels, so the stream is the same stream
//!   with more words per fetch slot.
//! * **`FMODE` and `BPLCON4`** arrive as ordinary register writes; `FMODE` is
//!   Alice's register as well as Lisa's (§3: `FMODE 1FC W A D`).
//!
//! # What is modelled, and what is latched only
//!
//! | | |
//! | --- | --- |
//! | eight bitplanes, `BPU3` | modelled: `BPLCON0` bit 4, so `BPU` is "0000-1000 (none thru 8 inclusive)" |
//! | the 256-entry 24-bit colour table | modelled: `BANK`, `LOCT`, and the automatic four-to-eight-bit extension |
//! | HAM8, HAM6 | not yet: `HAMEN` is ignored |
//! | `BPLCON4`'s `BPLAM` | modelled; `ESPRM` and `OSPRM` latched, until there are sprites |
//! | `BPLCON3`'s `BANK`, `PF2OF`, `LOCT`, `BRDRBLNK` | modelled; `SPRES` and `BRDSPRT` latched, until there are sprites |
//! | `BPLCON0`'s `ECSENA` gate | modelled: it inhibits `BRDRBLNK`, `BRDNTRAN`, `ZDCLKEN`, `BRDSPRT` and `EXTBLKEN` |
//! | `BPLCON1`'s eight-bit scroll | modelled, both playfields, 35 ns granularity |
//! | 4+4 dual playfield with `PF2OF` | modelled |
//! | EHB, and `KILLEHB` | modelled: "EHB is invoked whenever SHRES = HIRES = HAMEN = DPF = 0 and BPU = 6" |
//! | `DIWHIGH`'s 70 ns and 35 ns window bits | modelled |
//! | sprites, collisions, `CLXCON2`, `FMODE`'s sprite bits | not yet: an AA part shows no sprites and detects no collisions |
//! | `FMODE`'s `BPL32`, `BPAGEM` | latched only, and deliberately: see [below](#what-fmodes-bitplane-bits-do-not-change) |
//! | `FMODE`'s `BSCAN2` | latched only — it selects between `BPL1MOD` and `BPL2MOD`, which is Alice's |
//! | `BPLCON2`'s `RDRAM`, `ZDBPEN`, `ZDBPSEL`, `ZDCTEN`, `SOGEN` | latched only: genlock, and reading the colour table back through a write-only address |
//! | `BPLCON3`'s `BRDNTRAN`, `ZDCLKEN`, `EXTBLKEN` | latched only: genlock and the `BLANK` pin, neither of which leaves the chip here |
//! | `BPLCON0`'s `BYPASS`, `UHRES` | latched only: eight-bit direct video out, and the external-logic pointers |
//! | the colour table's `T` bit | kept, so a snapshot round-trips it; nothing reads it, because nothing models the `ZD` pin |
//!
//! # What `FMODE`'s bitplane bits do not change
//!
//! `BPL32` and `BPAGEM` say how many bytes one bitplane fetch moves and
//! whether it is a double-`CAS` cycle (§4, `FMODE`, the first table). Both
//! facts are about the memory cycle Alice makes, and neither changes what Lisa
//! does with the result, for the reason `BPLxDAT` gives above. So Lisa latches
//! them — a snapshot has to carry them, and Alice reads the same register —
//! and the bit stream is the bit stream.
//!
//! The one place the width is visible from here is the **scroll range**: §5's
//! table gives 0–15 low-resolution pixels of scroll in `LORES` at 1× bandwidth
//! and 0–63 at 4×, which is exactly one fetch's worth each time. The document
//! gives ranges and no rule for a larger value, so **this model applies the
//! whole eight-bit delay** whatever `FMODE` says and shows whatever data is
//! there. That is an inference; a program that stays inside the table's range
//! cannot tell.
//!
//! # The other inferences, in one place
//!
//! * **Where a fetch's first pixel lands.** As for the 8373: the document
//!   gives no formula, so the 3rd-edition manual's arithmetic is kept — one
//!   fetch block and half a colour clock after the fetch — and the scroll runs
//!   from there. [`super::Setup`] has the derivation.
//! * **What `BPLAM` masks.** "Bits 15 thru 8 of `BPLCON4` comprise an 8 bit
//!   mask for the 8 bitplane address, XOR'ing the individual bits" (§2). The
//!   address that reaches the table is masked, in every mode: single
//!   playfield, either playfield of a dual one and EHB's five-bit address —
//!   and a playfield pixel whose value is zero, which is
//!   as much a bitplane colour address as any other, so inside the window the
//!   background is `COLOR(BPLAM)`. The border is not a bitplane pixel and
//!   stays colour 0. The document names no exception, and the register exists
//!   so "the copper [can] exchange color maps with a single instruction",
//!   which wants the mask to reach all of them.
//! * **`PF2OF` when playfield 1 has priority.** §4's `BPLCON3` page says the
//!   field determines the offset "when playfield 2 has priority in dual
//!   playfield mode", while §2 says flatly that it determines "second
//!   playfield's offset into the color table … since playfields in DPF mode
//!   can have up to 4 bitplanes". §2's reading is taken: it is playfield 2's
//!   offset, always. The other reading would make a playfield's colours depend
//!   on `PF2PRI`, which nothing else in the document suggests.
//!
//! # `BPU` out of range
//!
//! `BPU` is four bits and the document defines nine of the sixteen values.
//! Nine through fifteen are clamped to eight rather than blanking the display:
//! there are only eight planes to fetch, so eight is what the data can fill.

use super::{
    BPU3, BRDRBLNK, DBLPF, ENBPLCN3 as ECSENA, Fetch, HIRES, HOMOD, KILLEHB, LACE, Line,
    MAX_FETCH_WORDS, Revision, SHRES, Stamp, State,
};

/// Quarters — 35 ns pixels — in one low-resolution pixel.
const QUARTERS: i32 = 4;

/// Playfield 2's colour-table offset for each `PF2OF` code (§4, `BPLCON3`).
const PF2_OFFSET: [u8; 8] = [0, 2, 4, 8, 16, 32, 64, 128];

/// One playfield's eight-bit `BPLCON1` scroll, in quarters.
///
/// §4's `BPLCON1` page scatters the field: for playfield 1, `PF1H7` and
/// `PF1H6` are bits 11 and 10, `PF1H5`–`PF1H2` are bits 3–0, and `PF1H1` and
/// `PF1H0` are bits 9 and 8. Playfield 2's three groups sit four bits up in
/// each case. `PFyH0` is "LSB = 35ns SHRES pixel", so the assembled value is
/// already in quarters — and the old four-bit field, which the page renames
/// `PFyH5`–`PFyH2`, keeps its meaning of whole low-resolution pixels because
/// it now lands two bits up.
#[inline]
const fn scroll(bplcon1: u16, playfield2: bool) -> i32 {
    let v = if playfield2 { bplcon1 >> 4 } else { bplcon1 };
    let high = (v >> 10) & 3;
    let mid = v & 0xf;
    let low = (v >> 8) & 3;
    (high << 6 | mid << 2 | low) as i32
}

/// The per-line decisions, recomputed whenever a register that shapes them
/// changes.
#[derive(Debug, Clone, Copy)]
struct Setup {
    /// Planes enabled, 0–8.
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
    /// The `BPLAM` mask, already in place.
    bplam: u8,
    /// Playfield 2's colour-table offset, from `PF2OF`.
    pf2_offset: u8,
}

impl Setup {
    fn of(regs: &super::Regs, fetch_start: u16, vpos: u16) -> Setup {
        let con0 = regs.bplcon0;
        // "BPU2/1/0" at bits 14-12 and "BPU3" at bit 4, counting "0000-1000
        // (none thru 8 inclusive)". Nine and up cannot be fetched, so they are
        // eight.
        let planes = (usize::from((con0 >> 12) & 7) | usize::from(con0 & BPU3 != 0) << 3).min(8);
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
        // "PFI = odd, FP2 = even bit planes" (§4, BPLCON0): planes 1, 3, 5 and
        // 7 — indices 0, 2, 4 and 6 — take playfield 1's scroll.
        let delay = [scroll(regs.bplcon1, false), scroll(regs.bplcon1, true)];
        let mut start = [0i32; 8];
        for (p, s) in start.iter_mut().enumerate() {
            *s = base + delay[p % 2];
        }

        // "If this register is written, direct start & stop positions anywhere
        // on the screen" (§4, DIWHIGH), whose bits are: start V10-V8 at 2-0,
        // H0 at 3, H1 at 4, H10 at 5; stop V10-V8 at 10-8, H0 at 11, H1 at 12,
        // H10 at 13. An 8373's page gave the same H10 and V bits and left the
        // two sub-pixel ones "don't care", so an ECS program's window is
        // unchanged.
        let (vstart, vstop, hstart, hstop) = if regs.diwhigh_on {
            let high = regs.diwhigh;
            let quarter = |bits: u16| i32::from((bits >> 1) & 1) * 2 + i32::from(bits & 1);
            (
                ((high & 7) << 8) | (regs.diwstrt >> 8),
                (((high >> 8) & 7) << 8) | (regs.diwstop >> 8),
                QUARTERS * (i32::from((high >> 5) & 1) << 8 | i32::from(regs.diwstrt & 0xff))
                    + quarter(high >> 3),
                QUARTERS * (i32::from((high >> 13) & 1) << 8 | i32::from(regs.diwstop & 0xff))
                    + quarter(high >> 11),
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
            bplam: (regs.bplcon4 >> 8) as u8,
            pf2_offset: PF2_OFFSET[usize::from((regs.bplcon3 >> 10) & 7)],
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

    /// The colour-table address a bitplane value selects, `BPLAM` applied.
    #[inline]
    fn address(&self, index: u8) -> usize {
        usize::from(index ^ self.bplam)
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
            // A playfield pixel of value zero is still a bitplane colour
            // address, so `BPLAM` moves it too; the border is not one.
            let background = regs.palette[setup.address(0)];
            if setup.dual {
                // "4+4 bitplane dualplayfield is available in all 3
                // resolutions": playfield 1 is planes 1, 3, 5, 7 and
                // playfield 2 is planes 2, 4, 6, 8.
                let odd = (bits & 1) | ((bits >> 1) & 2) | ((bits >> 2) & 4) | ((bits >> 3) & 8);
                let even =
                    ((bits >> 1) & 1) | ((bits >> 2) & 2) | ((bits >> 3) & 4) | ((bits >> 4) & 8);
                let show1 = odd != 0;
                let show2 = even != 0;
                let pf2_first = regs.bplcon2 & (1 << 6) != 0;
                let pf2 = || regs.palette[setup.address(setup.pf2_offset.wrapping_add(even))];
                match (show1, show2) {
                    (true, true) if pf2_first => pf2(),
                    (true, _) => regs.palette[setup.address(odd)],
                    (false, true) => pf2(),
                    (false, false) => background,
                }
            } else if bits != 0 {
                if setup.ehb && bits & 0x20 != 0 {
                    half(regs.palette[setup.address(bits & 0x1f)])
                } else {
                    regs.palette[setup.address(bits)]
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
