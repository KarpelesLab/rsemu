//! Display DMA: the bitplane fetch and the sprite channels, and what Agnus
//! pushes into Denise.
//!
//! # Direction
//!
//! Denise has no memory bus and no vertical counter (Appendix J lists neither
//! among its pins), so it never asks for data. `BPLxDAT` and `SPRxDATA` are
//! Denise registers **Agnus writes by DMA** — Appendix B marks the first `&`,
//! "used by DMA channel only", and the second `%`. Agnus owns every pointer,
//! modulo and window register the fetch is steered by: `BPLxPT`, `BPL1MOD`,
//! `BPL2MOD`, `DDFSTRT`, `DDFSTOP`, `SPRxPT`, and its half of `DIWSTRT`,
//! `DIWSTOP`, `BPLCON0`, `SPRxPOS` and `SPRxCTL`.
//!
//! # The bitplane fetch
//!
//! Carried a **line at a time** rather than a word at a time: when the beam
//! leaves a line, Agnus hands Denise that line's words for every enabled plane
//! and the count its fetch began on, through
//! [`Video::line`](crate::dev::amiga::denise::Video::line), and when a field
//! begins it says so with
//! [`Video::field`](crate::dev::amiga::denise::Video::field) before any
//! position in the new field is reported. That is the contract `amiga.denise`
//! defines.
//!
//! What decides the words, from chapter 3:
//!
//! * **Vertically**, "the vertical bitplane DMA timing is identical to the
//!   display windows" (Appendix A, `DDFSTOP`): lines from `DIWSTRT`'s VSTART up
//!   to, not including, `DIWSTOP`'s VSTOP, whose ninth bit "is the complement of
//!   the next MSB" (chapter 3, *Setting Display Window Stopping Position*).
//! * **Horizontally**, from `DDFSTRT` to `DDFSTOP`, both using bits H8–H3
//!   (Appendix A), clamped to the hardware limits `$18` and `$D8` (table 3-14).
//!   The word count is counted in **eight-count blocks** from the block
//!   `DDFSTRT` falls in to the one `DDFSTOP` falls in, one word a block in
//!   low resolution and two in high. That agrees with chapter 3's formulas at
//!   the values the manual uses — twenty words for `$38`–`$D0`, forty for
//!   `$3C`–`$D4` — and departs from them elsewhere; see below.
//! * **Which planes**: `BPLCON0`'s `BPU` count, "000-110 (NONE through 6
//!   inclusive)".
//! * **The pointers** advance by two per word, and at the end of the line
//!   "the modulo is added to the bitplane pointers" — `BPL1MOD` for the odd
//!   planes, `BPL2MOD` for the even.
//!
//! Only `BPLEN` with `DMAEN` fetches. The pointers advance whether or not a
//! video chip is attached, so the guest-visible state of a board does not
//! depend on its wiring; the words are only read when somebody will look at
//! them.
//!
//! **Inference from firmware: the high-resolution word count.** Chapter 3's
//! "DDFSTRT = DDFSTOP − (4 × (word count − 2))" for high resolution, and table
//! 3-14's "49 words" at `$18`–`$D8` ("only one word is fetched at this limit"),
//! were this module's rule, and two unrelated ROMs that display correctly on
//! real machines contradict them:
//!
//! * Kickstart 2.04's graphics library opens the Workbench screen, 640 pixels
//!   of high resolution from a bitmap with 80 bytes a row, with `DDFSTRT $38`,
//!   `DDFSTOP $D8` and `BPL1MOD`/`BPL2MOD` of −4. So it counts on 84 bytes — 42
//!   words — being fetched a line; the formula with the `$D8` exception says
//!   41, and the desktop came out sheared a word a line.
//! * AROS opens an interlaced 640-pixel screen with `DDFSTRT $3C`,
//!   `DDFSTOP $D0` and modulos of 80 — so 40 words a line; the formula says 39,
//!   and it sheared the other way.
//!
//! Blocks of eight counts, aligned to eight, two words each in high resolution
//! give 42, 40, and the manual's own 40 for `$3C`–`$D4` and 20 for `$38`–`$D0`.
//! The fetch is plausibly the low-resolution machinery with two words to a
//! block, but that is a reading of the evidence, not a sentence in a manual,
//! and at `$18`–`$D8` it gives 50 where table 3-14 says 49.
//!
//! **Inference, and the one real simplification:** every word of a line is
//! fetched when the line ends, so a pointer or `BPLCON0` change the copper makes
//! part-way through a line's fetch applies to the whole of that line rather
//! than to the words after it. Copper lists set the pointers in vertical
//! blanking, which is why this matters so rarely.
//!
//! # The sprite channels
//!
//! Chapter 4, *Sprite Hardware Details* and *End-of-data Words*, per sprite:
//!
//! 1. The first two words at `SPRxPT` are written into `SPRxPOS` and `SPRxCTL`.
//!    **Inference:** this happens on the first line after vertical blanking —
//!    table 3-13's "Vertical Blank Stop", `$15` NTSC and `$1D` PAL — because
//!    the manual has the pointers written "during the vertical blanking
//!    interval before the first display of the sprite".
//! 2. "The sprite DMA channel will wait until the vertical beam counter value
//!    is the same as the data in the VSTART part of SPRxPOS."
//! 3. From then, two words a line go to `SPRxDATA` and `SPRxDATB`, "during a
//!    horizontal blanking interval" — here, on arrival at count 0.
//! 4. "When the vertical position of the beam counter is equal to the VSTOP
//!    value", the next two words go to `SPRxPOS` and `SPRxCTL` instead, and the
//!    channel waits again. "If the count of VSTOP − VSTART equals zero, no
//!    sprite output occurs" and the pair is taken as control words at once.
//!
//! Every one of those writes goes through the custom-chip bus with
//! [`Origin::dma`], so Denise's half of `SPRxPOS`/`SPRxCTL` sees them and
//! "writing to the sprite A data register enables the horizontal comparator"
//! on Denise's side as the manual says. Agnus's half of the same registers is
//! what the vertical comparison reads, so a processor write to `SPRxPOS` moves
//! the vertical start too.

use alloc::vec::Vec;

use super::beam::{Beam, Standard};
use super::blitter::Memory;
use super::{Chip, DmaChannel, Origin, Outward, State};

/// Where a sprite channel is in its field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpriteDma {
    /// Waiting for vertical blanking to end to fetch its first control words.
    Idle,
    /// Holding control words; waiting for the beam to reach VSTART.
    Waiting,
    /// Fetching a data pair every line until VSTOP.
    Active,
}

impl SpriteDma {
    /// The snapshot encoding.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            SpriteDma::Idle => 0,
            SpriteDma::Waiting => 1,
            SpriteDma::Active => 2,
        }
    }

    /// Decode [`code`](Self::code).
    #[must_use]
    pub const fn from_code(code: u8) -> Option<SpriteDma> {
        match code {
            0 => Some(SpriteDma::Idle),
            1 => Some(SpriteDma::Waiting),
            2 => Some(SpriteDma::Active),
            _ => None,
        }
    }
}

/// `BPLCON0` bit 15: high resolution.
const HIRES: u16 = 1 << 15;
/// `DDFSTRT`/`DDFSTOP` use H8–H3.
const DDF_BITS: u16 = 0x00fc;
/// Table 3-14's hardware limits.
const DDF_MIN: u16 = 0x18;
const DDF_MAX: u16 = 0xd8;

/// The first line after vertical blanking: table 3-13.
#[must_use]
pub const fn vblank_stop(std: Standard) -> u16 {
    match std {
        Standard::Pal => 0x1d,
        Standard::Ntsc => 0x15,
    }
}

impl State {
    /// How many bitplanes `BPLCON0` enables.
    pub(super) fn planes(&self) -> usize {
        usize::from((self.bplcon0 >> 12) & 7).min(6)
    }

    /// The horizontal fetch: the count it starts on and how many words.
    pub(super) fn fetch_window(&self) -> Option<(u16, u16)> {
        let start = (self.ddfstrt & DDF_BITS).max(DDF_MIN);
        let stop = (self.ddfstop & DDF_BITS).min(DDF_MAX);
        if stop < start {
            return None;
        }
        // Eight-count blocks, one word each in low resolution and two in high:
        // an inference from Kickstart 2.04 and AROS, which contradicts table
        // 3-14's "49 words"; see the module documentation.
        let blocks = ((stop & !7) - (start & !7)) / 8 + 1;
        let words = if self.bplcon0 & HIRES != 0 {
            blocks * 2
        } else {
            blocks
        };
        Some((start, words))
    }

    /// Whether line `vpos` is inside the display window vertically.
    pub(super) fn in_vertical_window(&self, vpos: u16) -> bool {
        let vstart = self.diwstrt >> 8;
        let raw = self.diwstop >> 8;
        let vstop = if raw & 0x80 == 0 { raw | 0x100 } else { raw };
        (vstart..vstop).contains(&vpos)
    }

    /// Whether line-level DMA work — bitplanes or sprites — is enabled, so a
    /// stride must not cross a line boundary.
    pub(super) fn line_work(&self) -> bool {
        self.video || self.dma_on(DmaChannel::BITPLANE) || self.dma_on(DmaChannel::SPRITE)
    }

    /// The beam is leaving `leaving`: fetch its bitplane words and, if a video
    /// chip is attached, queue the line for it.
    pub(super) fn end_of_line(&mut self, std: Standard, leaving: &Beam, mem: &mut Chip<'_>) {
        let mut planes: [Vec<u16>; 6] = Default::default();
        let window = self.fetch_window();
        let start = window.map_or(self.ddfstrt & DDF_BITS, |(s, _)| s);
        if self.dma_on(DmaChannel::BITPLANE)
            && self.in_vertical_window(leaving.vpos)
            && let Some((_, words)) = window
        {
            for (plane, out) in planes.iter_mut().enumerate().take(self.planes()) {
                let mut pointer = self.bplpt[plane];
                if self.video {
                    out.reserve(usize::from(words));
                    for _ in 0..words {
                        out.push(mem.read(pointer));
                        pointer = pointer.wrapping_add(2);
                    }
                } else {
                    pointer = pointer.wrapping_add(2 * u32::from(words));
                }
                // BPL1MOD for planes 1, 3, 5; BPL2MOD for 2, 4, 6.
                let modulo = i32::from(self.bplmod[plane % 2] as i16) as u32;
                self.bplpt[plane] = pointer.wrapping_add(modulo);
            }
        }
        if self.video {
            self.outbox.push(Outward::Line {
                vpos: leaving.vpos,
                clocks: leaving.line_len(std),
                start,
                planes,
            });
        }
    }

    /// A new line has begun: run each sprite channel's vertical comparison and
    /// fetch.
    pub(super) fn sprite_dma(&mut self, std: Standard, mem: &mut Chip<'_>) {
        if !self.dma_on(DmaChannel::SPRITE) {
            return;
        }
        let vpos = self.beam.vpos;
        for x in 0..8 {
            match self.sprite[x] {
                SpriteDma::Idle => {
                    if vpos == vblank_stop(std) {
                        self.sprite_control(x, mem);
                    }
                }
                SpriteDma::Waiting => {
                    if vpos == self.sprite_vstart(x) {
                        if vpos == self.sprite_vstop(x) {
                            self.sprite_control(x, mem);
                        } else {
                            self.sprite[x] = SpriteDma::Active;
                            self.sprite_data(x, mem);
                        }
                    }
                }
                SpriteDma::Active => {
                    if vpos == self.sprite_vstop(x) {
                        self.sprite_control(x, mem);
                    } else {
                        self.sprite_data(x, mem);
                    }
                }
            }
        }
    }

    /// `SV8`–`SV0`: `SPRxPOS` bits 15–8 and `SPRxCTL` bit 2.
    fn sprite_vstart(&self, x: usize) -> u16 {
        (self.sprpos[x] >> 8) | ((self.sprctl[x] >> 2) & 1) << 8
    }

    /// `EV8`–`EV0`: `SPRxCTL` bits 15–8 and bit 1.
    fn sprite_vstop(&self, x: usize) -> u16 {
        (self.sprctl[x] >> 8) | ((self.sprctl[x] >> 1) & 1) << 8
    }

    /// Fetch the next two words into `SPRxPOS` and `SPRxCTL`.
    fn sprite_control(&mut self, x: usize, mem: &mut Chip<'_>) {
        let (pos, ctl) = self.sprite_pair(x, mem);
        self.sprpos[x] = pos;
        self.sprctl[x] = ctl;
        self.sprite[x] = SpriteDma::Waiting;
        let base = 0x140 + 8 * x as u16;
        self.queue_dma_write(base, pos);
        self.queue_dma_write(base + 2, ctl);
    }

    /// Fetch the next two words into `SPRxDATA` and `SPRxDATB`.
    fn sprite_data(&mut self, x: usize, mem: &mut Chip<'_>) {
        let (data, datb) = self.sprite_pair(x, mem);
        let base = 0x140 + 8 * x as u16;
        self.queue_dma_write(base + 4, data);
        self.queue_dma_write(base + 6, datb);
    }

    fn sprite_pair(&mut self, x: usize, mem: &mut Chip<'_>) -> (u16, u16) {
        let pointer = self.sprpt[x];
        let first = mem.read(pointer);
        let second = mem.read(pointer.wrapping_add(2));
        self.sprpt[x] = pointer.wrapping_add(4);
        (first, second)
    }

    fn queue_dma_write(&mut self, offset: u16, value: u16) {
        self.outbox.push(Outward::Write {
            offset,
            value,
            from: Origin::dma(),
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_manuals_normal_fetch_windows() {
        let mut st = State::new();
        st.ddfstrt = 0x38;
        st.ddfstop = 0xd0;
        assert_eq!(
            st.fetch_window(),
            Some((0x38, 20)),
            "low resolution: 20 words"
        );
        st.bplcon0 = HIRES;
        st.ddfstrt = 0x3c;
        st.ddfstop = 0xd4;
        assert_eq!(st.fetch_window(), Some((0x3c, 40)), "high resolution: 40");
        st.bplcon0 = 0;
        st.ddfstrt = 0x00;
        st.ddfstop = 0xff;
        assert_eq!(
            st.fetch_window(),
            Some((0x18, 25)),
            "clamped to $18-$D8: \"a maximum of 25 words fetched in low resolution\""
        );
        st.bplcon0 = HIRES;
        assert_eq!(
            st.fetch_window(),
            Some((0x18, 50)),
            "50 in high, where table 3-14 says 49: the firmware evidence wins"
        );
    }

    #[test]
    fn a_high_resolution_fetch_counts_eight_count_blocks_as_the_firmware_expects() {
        let mut st = State::new();
        st.bplcon0 = HIRES;
        // Kickstart 2.04's Workbench screen: modulo -4 on an 80-byte row.
        st.ddfstrt = 0x38;
        st.ddfstop = 0xd8;
        assert_eq!(st.fetch_window(), Some((0x38, 42)));
        // AROS's interlaced screen: modulo 80 on an 80-byte row.
        st.ddfstrt = 0x3c;
        st.ddfstop = 0xd0;
        assert_eq!(st.fetch_window(), Some((0x3c, 40)));
    }

    #[test]
    fn the_vertical_window_takes_vstops_ninth_bit_from_its_eighth() {
        let mut st = State::new();
        st.diwstrt = 0x2c81;
        st.diwstop = 0x2cc1; // PAL: $12C
        assert!(!st.in_vertical_window(0x2b));
        assert!(st.in_vertical_window(0x2c));
        assert!(st.in_vertical_window(0x12b));
        assert!(!st.in_vertical_window(0x12c));
        st.diwstop = 0xf4c1; // NTSC: $F4
        assert!(st.in_vertical_window(0xf3));
        assert!(!st.in_vertical_window(0xf4));
    }

    #[test]
    fn bpu_counts_planes_and_stops_at_six() {
        let mut st = State::new();
        for (bpu, planes) in [(0, 0), (1, 1), (5, 5), (6, 6), (7, 6)] {
            st.bplcon0 = bpu << 12;
            assert_eq!(st.planes(), planes);
        }
    }
}
