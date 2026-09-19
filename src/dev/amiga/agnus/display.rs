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
//! On an ECS part three things change, all from Appendix C: `DIWHIGH`, once
//! written after `DIWSTRT`/`DIWSTOP`, gives the vertical window its `V10`–`V8`
//! directly; `BPLCON0`'s `SHRES` fetches four words a block; and a line the
//! programmable beam has made shorter than `DDFSTOP` fetches only the blocks
//! that start before it ends.
//!
//! On an AA part ([`aga`]) four more do: `BPLCON0`'s `BPU3` counts to eight
//! planes, which are fetched through `BPL7PT` and `BPL8PT`; `DDFSTRT` and
//! `DDFSTOP` decode `H2` as well, so a fetch starts on an even count rather
//! than a multiple of four; `FMODE` moves one, two or four words a transfer,
//! and the word count is rounded up to a whole one; and `BSCAN2` makes the
//! modulus the line's rather than the plane's. The sprite channel gains the
//! same widths, and `SSCAN2` skips its data fetch on a line of the wrong
//! parity — [`aga`]'s documentation quotes the sentences all of that comes
//! from.
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

use super::beam::{Beam, Standard, Timing};
use super::blitter::Memory;
use super::{Chip, DmaChannel, Origin, Outward, State, aga, ecs};

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
/// `BPLCON0` bit 6 on an ECS part: SuperHires (Appendix C, *SuperHires Mode*).
const SHRES: u16 = 1 << 6;
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
    /// How many bitplanes `BPLCON0` enables: six on an original or Enhanced
    /// Chip Set part, eight on Alice ([`aga::planes`]).
    pub(super) fn planes(&self) -> usize {
        aga::planes(self.rev, self.bplcon0)
    }

    /// Words moved by one bitplane transfer: one, always, until `FMODE`
    /// ([`aga::bitplane_words`]).
    pub(super) fn fetch_words(&self) -> u16 {
        if self.rev.is_aga() {
            aga::bitplane_words(self.fmode)
        } else {
            1
        }
    }

    /// The horizontal fetch: the count it starts on and how many words.
    pub(super) fn fetch_window(&self) -> Option<(u16, u16)> {
        // An AA part decodes one bit more of both registers, `H2` — two
        // colour clocks rather than four (`aga::DDF_BITS`).
        let bits = if self.rev.is_aga() {
            aga::DDF_BITS
        } else {
            DDF_BITS
        };
        let start = (self.ddfstrt & bits).max(DDF_MIN);
        let stop = (self.ddfstop & bits).min(DDF_MAX);
        if stop < start {
            return None;
        }
        // Eight-count blocks, one word each in low resolution and two in high:
        // an inference from Kickstart 2.04 and AROS, which contradicts table
        // 3-14's "49 words"; see the module documentation.
        let blocks = ((stop & !7) - (start & !7)) / 8 + 1;
        // A transfer is indivisible, so a window that does not divide by
        // `FMODE`'s width still moves the last one whole (`aga`, *`FMODE` and
        // the fetch*). Without `FMODE` this is the word count itself.
        let words = blocks * self.words_per_block();
        let f = self.fetch_words();
        Some((start, words.div_ceil(f) * f))
    }

    /// Words fetched per plane in each eight-count block: one in low
    /// resolution, two in high, and four in SuperHires — Appendix C,
    /// *SuperHires Mode*: a "35ns pixel display rate - twice the horizontal
    /// resolution of Hires mode", two planes of which "saturate DMA bandwidth
    /// as much as four Hires bitplanes".
    fn words_per_block(&self) -> u16 {
        if self.shres() {
            4
        } else if self.bplcon0 & HIRES != 0 {
            2
        } else {
            1
        }
    }

    /// `BPLCON0` bit 6, `SHRES`, on a part that has it.
    pub(super) fn shres(&self) -> bool {
        self.rev.is_ecs() && self.bplcon0 & SHRES != 0
    }

    /// Whether line `vpos` is inside the display window vertically.
    pub(super) fn in_vertical_window(&self, vpos: u16) -> bool {
        if self.diwhigh_on {
            let (vstart, vstop) =
                ecs::window_lines(self.diwstrt, self.diwstop, self.ecs[ecs::DIWHIGH]);
            return (vstart..vstop).contains(&vpos);
        }
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
    pub(super) fn end_of_line(&mut self, t: Timing, leaving: &Beam, mem: &mut Chip<'_>) {
        let mut planes: [Vec<u16>; 8] = Default::default();
        let clocks = leaving.line_len(t);
        let mut window = self.fetch_window();
        if self.rev.is_ecs()
            && let Some((start, words)) = window
        {
            // A programmed line can end before `DDFSTOP` does, and a fetch does
            // not outlive the line it is on: the blocks that start before the
            // counter wraps. (A hardwired line is 227 counts, past the `$D8`
            // limit, so this never shortens one.)
            let blocks = clocks.saturating_sub(start & !7).div_ceil(8);
            window = Some((start, words.min(blocks * self.words_per_block())));
        }
        let start = window.map_or(self.ddfstrt & DDF_BITS, |(s, _)| s);
        if self.dma_on(DmaChannel::BITPLANE)
            && self.in_vertical_window(leaving.vpos)
            && let Some((_, words)) = window
        {
            // `BSCAN2`: the modulus is the line's rather than the plane's -
            // "when scan-doubled both odd and even bitplanes use the same
            // modulus on a given line" (§2, *Bitplanes*).
            let scan2 = self.rev.is_aga() && self.fmode & aga::BSCAN2 != 0;
            let line_modulo = aga::scan_double_modulo(leaving.vpos, self.diwstrt);
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
                // BPL1MOD for planes 1, 3, 5, 7; BPL2MOD for 2, 4, 6, 8.
                let which = if scan2 { line_modulo } else { plane % 2 };
                let modulo = i32::from(self.bplmod[which] as i16) as u32;
                self.bplpt[plane] = pointer.wrapping_add(modulo);
            }
        }
        if self.video {
            self.outbox.push(Outward::Line {
                vpos: leaving.vpos,
                clocks,
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
        let first = if self.rev.is_ecs() {
            ecs::vblank_end(&self.ecs)
        } else {
            vblank_stop(std)
        };
        for x in 0..8 {
            match self.sprite[x] {
                SpriteDma::Idle => {
                    if vpos == first {
                        self.sprite_control(x, mem);
                    }
                }
                SpriteDma::Waiting => {
                    if vpos == self.sprite_vstart(x) {
                        if vpos == self.sprite_vstop(x) {
                            self.sprite_control(x, mem);
                        } else {
                            self.sprite[x] = SpriteDma::Active;
                            if self.sprite_fetches_now(x, vpos) {
                                self.sprite_data(x, mem);
                            }
                        }
                    }
                }
                SpriteDma::Active => {
                    if vpos == self.sprite_vstop(x) {
                        self.sprite_control(x, mem);
                    } else if self.sprite_fetches_now(x, vpos) {
                        self.sprite_data(x, mem);
                    }
                }
            }
        }
    }

    /// Whether sprite `x` moves data on this line: always, unless `SSCAN2`
    /// and its own `SH10` scan-double it and the parities disagree
    /// ([`aga::sprite_fetches_on`]). Only the *data* fetch is gated: a control
    /// fetch is what loads `SPRxPOS`, so the bits this asks about are not
    /// there yet, and the specification's parity note keeps a `VSTOP` line on
    /// the fetching side anyway.
    fn sprite_fetches_now(&self, x: usize, vpos: u16) -> bool {
        !self.rev.is_aga() || aga::sprite_fetches_on(self.fmode, self.sprpos[x], vpos)
    }

    /// Words moved by one sprite transfer ([`aga::sprite_words`]).
    fn sprite_transfer_words(&self) -> u16 {
        if self.rev.is_aga() {
            aga::sprite_words(self.fmode)
        } else {
            1
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

    /// Fetch the next two transfers into `SPRxPOS` and `SPRxCTL`.
    ///
    /// A wide `FMODE` widens these two as it widens the data pair — §4's
    /// table is the sprite channel's fetch increment and names no exception —
    /// so the control words are the first word of each transfer and the rest
    /// of it is skipped. **An inference**, and the one a sprite structure
    /// padded to the fetch width expects.
    fn sprite_control(&mut self, x: usize, mem: &mut Chip<'_>) {
        let f = self.sprite_transfer_words();
        let pos = self.sprite_transfer(x, f, mem)[0];
        let ctl = self.sprite_transfer(x, f, mem)[0];
        self.sprpos[x] = pos;
        self.sprctl[x] = ctl;
        self.sprite[x] = SpriteDma::Waiting;
        let base = 0x140 + 8 * x as u16;
        self.queue_dma_write(base, pos);
        self.queue_dma_write(base + 2, ctl);
    }

    /// Fetch the next two transfers into `SPRxDATA` and `SPRxDATB`.
    ///
    /// Sixteen bits go through the register bus, as they always have. A wider
    /// transfer does not fit a `u16`, so it goes through
    /// [`Video::sprite_dma`](crate::dev::amiga::denise::Video::sprite_dma)
    /// instead — left-justified, and queued in the outbox behind whatever this
    /// slot wrote through the bus.
    fn sprite_data(&mut self, x: usize, mem: &mut Chip<'_>) {
        let f = self.sprite_transfer_words();
        let base = 0x140 + 8 * x as u16;
        for (b_buffer, offset) in [(false, base + 4), (true, base + 6)] {
            let words = self.sprite_transfer(x, f, mem);
            if f == 1 {
                self.queue_dma_write(offset, words[0]);
            } else {
                self.outbox.push(Outward::SpriteData {
                    sprite: x,
                    b_buffer,
                    bits: aga::sprite_bits(&words[..usize::from(f)]),
                });
            }
        }
    }

    /// One sprite DMA transfer: `words` consecutive words from the channel's
    /// pointer, which then advances by that many.
    fn sprite_transfer(&mut self, x: usize, words: u16, mem: &mut Chip<'_>) -> [u16; 4] {
        let mut pointer = self.sprpt[x];
        let mut out = [0u16; 4];
        for slot in out.iter_mut().take(usize::from(words)) {
            *slot = mem.read(pointer);
            pointer = pointer.wrapping_add(2);
        }
        self.sprpt[x] = pointer;
        out
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
