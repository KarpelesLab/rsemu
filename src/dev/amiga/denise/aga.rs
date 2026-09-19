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
//! The seam is [`super::Fetch`] and [`Video::sprite_dma`](super::Video::sprite_dma),
//! and both are documented where they are declared. In short:
//!
//! * **Bitplanes**: eight word streams, unchanged in shape. A wider `FMODE`
//!   needs nothing new, because "the parallel to serial conversion is
//!   triggered whenever bit plane #1 is written, indicating the completion of
//!   all bit planes for that word (16/32/64 pixels). The MSB is output first,
//!   and is therefore always on the left" (§4, `BPLxDAT`) — a fetch of any
//!   width is that many consecutive pixels, so the stream is the same stream
//!   with more words per fetch slot.
//! * **Sprites**: [`Video::sprite_dma`](super::Video::sprite_dma), because a
//!   32- or 64-bit sprite fetch does not fit the sixteen-bit register bus.
//! * **`FMODE` and `BPLCON4`** arrive as ordinary register writes; `FMODE` is
//!   Alice's register as well as Lisa's (§3: `FMODE 1FC W A D`).
//!
//! # What is modelled, and what is latched only
//!
//! | | |
//! | --- | --- |
//! | eight bitplanes, `BPU3` | modelled: `BPLCON0` bit 4, so `BPU` is "0000-1000 (none thru 8 inclusive)" |
//! | the 256-entry 24-bit colour table | modelled: `BANK`, `LOCT`, and the automatic four-to-eight-bit extension |
//! | HAM8 and HAM6 in every resolution | modelled, §2 *Bitplanes* and its two control tables |
//! | `BPLCON4`'s `BPLAM`, `ESPRM`, `OSPRM` | modelled |
//! | `BPLCON3`'s `BANK`, `PF2OF`, `LOCT`, `SPRES`, `BRDRBLNK`, `BRDSPRT` | modelled |
//! | `BPLCON0`'s `ECSENA` gate | modelled: it inhibits `BRDRBLNK`, `BRDNTRAN`, `ZDCLKEN`, `BRDSPRT` and `EXTBLKEN` |
//! | `BPLCON1`'s eight-bit scroll | modelled, both playfields, 35 ns granularity |
//! | 4+4 dual playfield with `PF2OF` | modelled |
//! | EHB, and `KILLEHB` | modelled: "EHB is invoked whenever SHRES = HIRES = HAMEN = DPF = 0 and BPU = 6" |
//! | `DIWHIGH`'s 70 ns and 35 ns window bits | modelled |
//! | sprites: `SPRES`, 16/32/64-bit data, `ESPRM`/`OSPRM`, attachment in every resolution | modelled |
//! | `SPRxCTL`'s `SH1` and `SH0` | modelled: a sprite positioned to the quarter-pixel |
//! | `FMODE`'s `SSCAN2` | modelled: `SH10` leaves the horizontal comparison, because Alice is using it |
//! | `CLXCON2` | modelled: bitplanes 7 and 8 in collisions, and a `CLXCON` write clears it |
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
//! * **HAM6's low bits on an eight-bit gun.** §2 says of HAM8 that "the data
//!   is placed in 6 MSB. The 2 LSB are left unmodified"; it says nothing about
//!   HAM6, whose four bits are half of a gun. The same rule is applied — four
//!   bits into the four most significant, the rest held — because it is the
//!   rule the document states for the mode it describes.
//! * **What `BPLAM` masks.** "Bits 15 thru 8 of `BPLCON4` comprise an 8 bit
//!   mask for the 8 bitplane address, XOR'ing the individual bits" (§2). The
//!   address that reaches the table is masked, in every mode: single
//!   playfield, either playfield of a dual one, EHB's five-bit address and a
//!   HAM base register — and a playfield pixel whose value is zero, which is
//!   as much a bitplane colour address as any other, so inside the window the
//!   background is `COLOR(BPLAM)`. The border is not a bitplane pixel and
//!   stays colour 0. The document names no exception, and the register exists
//!   so "the copper [can] exchange color maps with a single instruction",
//!   which wants the mask to reach all of them.
//! * **HAM with seven planes.** §2 defines HAM8 at `BPU = 8` and HAM6 "as
//!   before"; `BPU = 7` with `HAMEN` is not described. Five, six or seven
//!   planes are taken as HAM6 here, and eight as HAM8.
//! * **`PF2OF` when playfield 1 has priority.** §4's `BPLCON3` page says the
//!   field determines the offset "when playfield 2 has priority in dual
//!   playfield mode", while §2 says flatly that it determines "second
//!   playfield's offset into the color table … since playfields in DPF mode
//!   can have up to 4 bitplanes". §2's reading is taken: it is playfield 2's
//!   offset, always. The other reading would make a playfield's colours depend
//!   on `PF2PRI`, which nothing else in the document suggests.
//! * **Sprite priority and collision grouping** are the 3rd-edition manual's,
//!   unchanged: `CLXDAT`'s bit assignments are reprinted identically in §4 and
//!   §2 says "CLXDAT is unchanged".
//!
//! # `BPU` out of range
//!
//! `BPU` is four bits and the document defines nine of the sixteen values.
//! Nine through fifteen are clamped to eight rather than blanking the display:
//! there are only eight planes to fetch, so eight is what the data can fill.

use super::{
    ATTACH, BPU3, BRDRBLNK, BRDSPRT, DBLPF, ENBPLCN3 as ECSENA, Fetch, HIRES, HOMOD, KILLEHB, LACE,
    Line, MAX_FETCH_WORDS, Revision, SHRES, SSCAN2, Stamp, State,
};

/// Quarters — 35 ns pixels — in one low-resolution pixel.
const QUARTERS: i32 = 4;

/// Playfield 2's colour-table offset for each `PF2OF` code (§4, `BPLCON3`).
const PF2_OFFSET: [u8; 8] = [0, 2, 4, 8, 16, 32, 64, 128];

/// A sprite's data width in pixels for `FMODE`'s `SPAGEM` and `SPR32`, which
/// §4 tabulates as a fetch "By 2 bytes", "By 4 bytes" or "By 8 bytes".
#[inline]
const fn sprite_pixels(fmode: u16) -> u8 {
    match (fmode >> 2) & 3 {
        0 => 16,
        3 => 64,
        // `SPR32` alone is a 32-bit bus; `SPAGEM` alone is two 16-bit cycles.
        // Both move four bytes, so both give a 32-pixel sprite.
        _ => 32,
    }
}

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
    ham6: bool,
    ham8: bool,
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
    /// `BRDSPRT` and `ECSENA`: sprites show outside the window.
    border_sprites: bool,
    /// The `BPLAM` mask, already in place.
    bplam: u8,
    /// Playfield 2's colour-table offset, from `PF2OF`.
    pf2_offset: u8,
    /// Quarters per sprite pixel, from `SPRES`.
    sprite_step: i32,
    /// A sprite's data width in pixels, from `FMODE`.
    sprite_pixels: u8,
    /// `SSCAN2`: `SH10` is Alice's scan-double flag, not part of the compare.
    sscan2: bool,
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
        // "This mode is invoked when BPU field in BPLCON0 is set to 8, and
        // HAMEN is set" (§2). The six-plane mode is "as before" — and now
        // "works in HIRES and SHRES resolutions" too.
        let ham = con0 & HOMOD != 0 && !dual;
        let ham8 = ham && planes == 8;
        let ham6 = ham && !ham8 && planes >= 5;
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
        // "SPRES1 and SPRES0 control sprite resolution": 00 the ECS defaults,
        // "LORES,HIRES=140ns, SHRES=70ns"; then 140 ns, 70 ns and 35 ns.
        let sprite_step = match (regs.bplcon3 >> 6) & 3 {
            0 if shres => 2,
            0 | 1 => QUARTERS,
            2 => 2,
            _ => 1,
        };
        Setup {
            planes,
            step,
            dual,
            ham6,
            ham8,
            ehb,
            start,
            inside_v: vstart <= vpos && vpos < vstop,
            hstart,
            hstop,
            blank: enabled && regs.bplcon3 & BRDRBLNK != 0,
            border_sprites: enabled && regs.bplcon3 & BRDSPRT != 0,
            bplam: (regs.bplcon4 >> 8) as u8,
            pf2_offset: PF2_OFFSET[usize::from((regs.bplcon3 >> 10) & 7)],
            sprite_step,
            sprite_pixels: sprite_pixels(regs.fmode),
            sscan2: regs.fmode & SSCAN2 != 0,
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

/// A sprite's serializer. Sixty-four bits because an AA sprite's data is up to
/// that wide, and left-justified because it shifts out "MSB first on the
/// left".
#[derive(Debug, Clone, Copy, Default)]
struct Shifter {
    a: u64,
    b: u64,
    /// Pixels still to shift out.
    left: u8,
    /// Quarters spent on the pixel being shown, against `sprite_step`.
    phase: i32,
}

/// The frontmost sprite pixel: its group (0–3) and colour-table address.
///
/// The pairing and the "lower-numbered sprites are always in front" rule are
/// the 3rd-edition manual's, unchanged. What AA adds is where the four-bit
/// value lands in a 256-entry table: "ESPRM7 thru ESPRM4 allow relocation of
/// the even sprite color map. OSPRM7 thru OSPRN4 allow relocation of the odd
/// sprite color map. In the case of attached sprites OSPRM bits are used."
/// (§2, *Sprites*.) With both fields at their reset value of `0001` the
/// addresses are 16–31 — the 8362's fixed ones.
#[inline]
fn front_sprite(pixels: &[u8; 8], regs: &super::Regs) -> Option<(u8, usize)> {
    let esprm = usize::from((regs.bplcon4 >> 4) & 0xf) << 4;
    let osprm = usize::from(regs.bplcon4 & 0xf) << 4;
    for k in 0..4 {
        let even = pixels[2 * k];
        let odd = pixels[2 * k + 1];
        if regs.spr_ctl[2 * k + 1] & ATTACH != 0 {
            let value = (odd << 2) | even;
            if value != 0 {
                return Some((k as u8, osprm | usize::from(value)));
            }
        } else if even != 0 {
            return Some((k as u8, esprm | (4 * k + usize::from(even))));
        } else if odd != 0 {
            return Some((k as u8, osprm | (4 * k + usize::from(odd))));
        }
    }
    None
}

/// The collision bits one pixel raises.
///
/// `CLXDAT`'s assignments and `CLXCON`'s are reprinted unchanged in §4, and §2
/// says "CLXDAT is unchanged". What AA adds is `CLXCON2`: "ENBP8 and ENBP7 are
/// the enable bits for bitplanes 8 and 7[;] MVBP8 and MVBP7 are their match
/// value bits", at bits 7, 6, 1 and 0. Its own note repeats the older one:
/// "Disable[d] bit planes cannot prevent collisions."
#[inline]
fn collisions(bits: u8, pixels: &[u8; 8], clxcon: u16, clxcon2: u16) -> u16 {
    let enabled = (clxcon >> 6) & 0x3f | ((clxcon2 >> 6) & 3) << 6;
    let match_value = clxcon & 0x3f | (clxcon2 & 3) << 6;
    let mismatch = (u16::from(bits) ^ match_value) & enabled;
    // "Playfield 1 is all odd numbered enabled bit planes. Playfield 2 is all
    // even numbered enabled bit planes" — now four of each.
    let odd = mismatch & 0b0101_0101 == 0;
    let even = mismatch & 0b1010_1010 == 0;

    let mut groups = 0u8;
    for k in 0..4 {
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

    let mut shifters = [Shifter::default(); 8];
    let mut hold = st.regs.palette[0];
    let mut clx = 0u16;

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
            change.apply_to(&mut st.regs, Revision::Aga);
            changed = true;
        }
        if changed {
            setup = Setup::of(&st.regs, line.fetch.start, vpos);
        }
        let regs = &st.regs;

        // The horizontal comparators. `SH10-SH3` are `SPRxPOS` bits 7-0 and
        // `SH2`, `SH1`, `SH0` are `SPRxCTL` bits 0, 4 and 3 — "Start horiz.
        // value, 140nS / 70nS / 35nS increment" (§4, SPRxCTL) — so the whole
        // comparison is in quarters. "If SSCAN2 bit in FMODE is set, then
        // disable SH10 horizontal coincidence detect. This bit is then free to
        // be used by ALICE as an individual scan double enable" (§4, SPRxPOS).
        let mut pixels = [0u8; 8];
        if regs.armed != 0 || shifters.iter().any(|s| s.left != 0) {
            for (i, s) in shifters.iter_mut().enumerate() {
                let hstart = i32::from(regs.spr_pos[i] & 0xff) << 3
                    | i32::from(regs.spr_ctl[i] & 1) << 2
                    | i32::from((regs.spr_ctl[i] >> 4) & 1) << 1
                    | i32::from((regs.spr_ctl[i] >> 3) & 1);
                // With SH10 out of the comparison, the comparator matches
                // wherever the other ten bits do — once every 1024 quarters.
                let hit = if setup.sscan2 {
                    q & 0x3ff == hstart & 0x3ff
                } else {
                    q == hstart
                };
                if regs.armed & (1 << i) != 0 && hit {
                    *s = Shifter {
                        a: regs.spr_data[i],
                        b: regs.spr_datb[i],
                        left: setup.sprite_pixels,
                        phase: 0,
                    };
                }
                if s.left != 0 {
                    // "The DATB bits are the 2SBs (worth 2) for the color
                    // registers[,] DATA bits are LSBs of the pixels."
                    pixels[i] = (((s.b >> 63) & 1) << 1 | ((s.a >> 63) & 1)) as u8;
                }
            }
        }

        let inside = setup.inside_v && setup.hstart <= q && q < setup.hstop;
        let sprite = front_sprite(&pixels, regs);
        let border = if setup.blank { 0 } else { regs.palette[0] };

        let bits = setup.bits(&line.fetch, q);

        // The hold register follows the serialized bits whether or not
        // anything is displayed over them, as on the older parts.
        if setup.ham8 {
            // "Bitplanes 1 and 2 are used as control bits analagous to the
            // function of bitplanes 5 and 6 in 6 bitplane HAM mode … Since
            // only 6 bitplanes are available for modify data, the data is
            // placed in 6 MSB. The 2 LSB are left unmodified" (§2).
            let data = u32::from(bits >> 2) << 2;
            hold = match bits & 3 {
                0 => regs.palette[setup.address(bits)],
                1 => (hold & 0x00ff_ff03) | data,
                2 => (hold & 0x0003_ffff) | data << 16,
                _ => (hold & 0x00ff_03ff) | data << 8,
            };
        } else if setup.ham6 {
            // The 3rd-edition table, reprinted in §2: planes 5 and 6 control,
            // planes 1-4 the data. Four bits into a gun's four most
            // significant, the rest held — see the module docs.
            let data = u32::from(bits & 0xf) << 4;
            hold = match (bits >> 4) & 3 {
                0 => regs.palette[setup.address(bits & 0xf)],
                1 => (hold & 0x00ff_ff0f) | data,
                2 => (hold & 0x000f_ffff) | data << 16,
                _ => (hold & 0x00ff_0fff) | data << 8,
            };
        }

        let colour = if !inside {
            // "BRDRSPRT, when high, allows sprites to be visible in border
            // areas" (§2, *Sprites*).
            match sprite {
                Some((_, reg)) if setup.border_sprites => regs.palette[reg],
                _ => border,
            }
        } else {
            clx |= collisions(bits, &pixels, regs.clxcon, regs.clxcon2);
            // A playfield pixel of value zero is still a bitplane colour
            // address, so `BPLAM` moves it too; the border is not one.
            let background = regs.palette[setup.address(0)];
            let group = sprite.map(|s| u16::from(s.0));
            let sprite_colour = sprite.map(|s| regs.palette[s.1]);
            let blocked = |code: u16| group.is_some_and(|g| g < code);
            if setup.dual {
                // "4+4 bitplane dualplayfield is available in all 3
                // resolutions": playfield 1 is planes 1, 3, 5, 7 and
                // playfield 2 is planes 2, 4, 6, 8.
                let odd = (bits & 1) | ((bits >> 1) & 2) | ((bits >> 2) & 4) | ((bits >> 3) & 8);
                let even =
                    ((bits >> 1) & 1) | ((bits >> 2) & 2) | ((bits >> 3) & 4) | ((bits >> 4) & 8);
                let show1 = odd != 0 && !blocked(regs.bplcon2 & 7);
                let show2 = even != 0 && !blocked((regs.bplcon2 >> 3) & 7);
                let pf2_first = regs.bplcon2 & (1 << 6) != 0;
                let pf2 = || regs.palette[setup.address(setup.pf2_offset.wrapping_add(even))];
                match (show1, show2) {
                    (true, true) if pf2_first => pf2(),
                    (true, _) => regs.palette[setup.address(odd)],
                    (false, true) => pf2(),
                    (false, false) => sprite_colour.unwrap_or(background),
                }
            } else if bits != 0 && !blocked((regs.bplcon2 >> 3) & 7) {
                if setup.ham8 || setup.ham6 {
                    hold
                } else if setup.ehb && bits & 0x20 != 0 {
                    half(regs.palette[setup.address(bits & 0x1f)])
                } else {
                    regs.palette[setup.address(bits)]
                }
            } else {
                sprite_colour.unwrap_or(background)
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

        for s in &mut shifters {
            if s.left != 0 {
                s.phase += 1;
                if s.phase >= setup.sprite_step {
                    s.a <<= 1;
                    s.b <<= 1;
                    s.left -= 1;
                    s.phase = 0;
                }
            }
        }
    }
    st.clxdat |= clx;
}

#[cfg(test)]
mod tests;
