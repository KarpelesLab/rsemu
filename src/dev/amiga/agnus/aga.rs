//! Alice: the AA chip set's Agnus, the 8374.
//!
//! `revision = "aga"` on `amiga.agnus`. Everything an 8375 does ([`super::ecs`])
//! and, on top of it, what the *Specification for the Advanced Amiga (AA) Chip
//! Set* (Commodore-Amiga, 06/07/91) gives the DMA half of the chip set. This
//! module holds the arithmetic; [`super::display`] and [`super`] call into it,
//! and every branch that is Alice's is behind [`Revision::is_aga`].
//!
//! # The source
//!
//! That specification, cited by its section: §1 *Summary of New features for
//! AA*, §2 *Explanation of new features*, §3 *List of Registers ordered by
//! Address*, §4 the per-register pages, §5 *New LISA Display Modes*. Where it
//! is silent the comment says so and says what was chosen instead. **No
//! emulator source of any licence was consulted**, nor any FPGA
//! reimplementation; the document was fetched on its own. Lisa's half of the
//! same document is read in [`crate::dev::amiga::denise::aga`], and that
//! module's ledger is worth reading beside this one.
//!
//! # What Alice does that an 8375 does not
//!
//! | | |
//! | --- | --- |
//! | eight bitplanes | `BPL7PT` `$0F8`/`$0FA` and `BPL8PT` `$0FC`/`$0FE`, and `BPLCON0`'s `BPU3` counting to eight |
//! | `FMODE` `$1FC` | the fetch widths: [`bitplane_words`] and [`sprite_words`], 1, 2 or 4 words a transfer for bitplanes and for sprites |
//! | `BSCAN2` | bitplane scan doubling: [`scan_double_modulo`] picks `BPL1MOD` or `BPL2MOD` by the line's parity rather than the plane's |
//! | `SSCAN2` | sprite scan doubling: [`sprite_scan_doubled`] and [`sprite_fetches_on`] |
//! | 2 MiB of chip RAM | 20-bit pointers, §3's preamble; [`Revision::reach`] |
//! | `DDFSTRT`/`DDFSTOP` | one bit more, `H2`: [`DDF_BITS`], two colour clocks rather than four |
//! | `VPOSR` | "8374(alice) = 22 PAL, 32 NTSC" (§4) |
//!
//! And what it does **not**: the blitter and the copper are unchanged. §4's
//! `BLTxPTH`/`BLTxPTL`, `BLTxMOD`, `BLTAFWM`/`BLTALWM`, `BLTxDAT`, `BLTCON0`,
//! `BLTCON1`, `BLTSIZE`, `BLTSIZH`/`BLTSIZV`, `COPCON`, `COPJMP1`/`COPJMP2`,
//! `COPxLC` and `COPINS` pages are the Enhanced Chip Set's pages word for word
//! — `BLTCON0L`, `BLTSIZV` and `BLTSIZH` still carry `h`, "new for HiRes chip
//! set", and `COPCON`'s rule is still "if 0, access to RGA>7E". Both engines
//! gain the wider pointer and nothing else. `UHRES` (`BPLHPT`, `SPRHPT`,
//! `BPLHMOD`, `SPRHSTRT`…) is held and not acted on, as it is on an 8375: it
//! drives external logic this board does not have.
//!
//! # `FMODE` and the fetch
//!
//! §4's `FMODE` page gives two identical tables, one for bitplanes and one for
//! sprites:
//!
//! ```text
//! BPAGEM BPL32   Bitplane Fetch Increment   Memory Cycle   Bus Width
//!    0     0     by 2 bytes (as before)     normal CAS        16
//!    0     1     by 4 bytes                 normal CAS        32
//!    1     0     by 4 bytes                 double CAS        16
//!    1     1     by 8 bytes                 double CAS        32
//! ```
//!
//! So one transfer moves one, two or four words, and the pointer advances by
//! that many. The two ways of moving four bytes differ only in how the memory
//! cycle is made, which is a fact about the RAM and not about the picture.
//!
//! **How many words a line, and the one inference.** §5's key says a mode
//! "needs 1x / 2x / 4x Bandwidth", and its scroll table gives one fetch's
//! worth of pixels per mode: `LORES` scrolls 0–15 low-resolution pixels at 1×,
//! 0–31 at 2× and 0–63 at 4×. A fetch is therefore 16, 32 or 64 *bitplane
//! pixels* however many words that takes, and the fetch window is the same
//! window it always was — `FMODE` buys bus cycles, not picture. The count of
//! words a line is what it was: this model's eight-count blocks, one word each
//! in `LORES`, two in `HIRES` and four in `SHRES` ([`super::display`] has the
//! firmware evidence for the block counting, which AA does not disturb).
//!
//! What `FMODE` does change is that a transfer is indivisible: the last one
//! moves a whole `f` words whether or not the window wanted them. So the word
//! count is **rounded up to a multiple of `f`** and the pointer advances by
//! twice that before the modulo. The document gives the increment and the
//! bandwidth and not this rule; **it is an inference**, and the alternative —
//! truncating to a multiple of `f` — would fetch fewer pixels than the window
//! displays. A program whose window is a whole number of transfers wide, which
//! is every program that gets the bandwidth right, cannot tell.
//!
//! # `BSCAN2`
//!
//! §2, *Bitplanes*: "BSCAN2 bit in FMODE enables bitplane scan-doubling. When
//! V0 bit of DIWSTRT matches V0 of vertical beam counter, BPL1MOD contains the
//! modulus for the display line, else BPL2MOD is used. When scan-doubled both
//! odd and even bitplanes use the same modulus on a given line, whereas in
//! normal mode odd bitplanes used BPL1MOD and even bitplanes used BPL2MOD. As
//! a result Dual Playfield screens will probably not display correctly when
//! scan-doubled." §4's `BPL1MOD` page says the same in other words: "Lines
//! whose LSBs of beam counter and DIWSTRT match are designated primary,
//! whereas lines whose LSBs don't match are designated alternate."
//!
//! `DIWSTRT`'s `V0` is bit 8 of that register. So the modulus is a property of
//! the *line* and not of the plane, which is [`scan_double_modulo`]. A screen
//! doubled this way sets `BPL2MOD` to the negative of a row, so the alternate
//! line fetches the same row again.
//!
//! # `SSCAN2`
//!
//! §2, *Sprites*: "SSCAN2 bit in FMODE enables sprite scan-doubling. When
//! enabled, individual SH10 bits in SPRxPOS registers control whether or not a
//! given sprite is to be scan-doubled. When V0 bit of SPRxPOS register matches
//! V0 bit of vertical beam counter, the given sprite's DMA is allowed to
//! proceed as before. If they don't match, then sprite DMA is disabled and
//! LISA reuses the sprite data from the previous line. When sprites are
//! scan-doubled, only the position and control registers need be modified by
//! the programmer; the data registers need no modification." And: "NOTE:
//! Sprite vertical start and stop positions must be of the same parity, i.e.
//! both odd or both even."
//!
//! `SH10` is `SPRxPOS` bit 0 — §4: "If SSCAN2 bit in FMODE is set, then
//! disable SH10 horizontal coincidence detect. This bit is then free to be
//! used by ALICE as an individual scan double enable." `SV0` is `SPRxPOS` bit
//! 8. On a line a doubled sprite skips, **nothing happens at all**: no fetch,
//! no pointer advance and no write into Lisa, who is still showing the
//! previous line's data because nothing overwrote it. The vertical comparisons
//! that start and stop the channel are not skipped — they are the beam's
//! business, not the fetch's, and the parity note exists precisely so that a
//! `VSTOP` line is never a skipped one.

use super::ecs::Revision;

/// `FMODE` bit 15: "global enable for sprite scan-doubling" (§4).
pub const SSCAN2: u16 = 1 << 15;
/// `FMODE` bit 14: "enables use of 2nd P/F modulus on an alternate line basis
/// to support bitplane scan-doubling" (§4).
pub const BSCAN2: u16 = 1 << 14;
/// `FMODE` bit 3: "Sprite Page Mode (double CAS)".
pub const SPAGEM: u16 = 1 << 3;
/// `FMODE` bit 2: "Sprite 32 Bit Wide Mode".
pub const SPR32: u16 = 1 << 2;
/// `FMODE` bit 1: "Bitplane Page Mode (double CAS)".
pub const BPAGEM: u16 = 1 << 1;
/// `FMODE` bit 0: "Bitplane 32 Bit Wide Mode".
pub const BPL32: u16 = 1 << 0;

/// `SPRxPOS` bit 0: `SH10`, and Alice's per-sprite scan-double enable under
/// `SSCAN2` (§4, `SPRxPOS`).
pub const SH10: u16 = 1 << 0;

/// `DDFSTRT` and `DDFSTOP` on an AA part: `H8`–`H2`, bits 7–1.
///
/// §4's page prints "USE X X X X X X X X H8 H7 H6 H5 H4 H3 H2 X" against bits
/// 15–0, one bit further down than the Enhanced Chip Set's `H8`–`H3`. `H1` is
/// one colour clock and is not decoded; `H2` is two, so an AA fetch can start
/// on an even count rather than only a multiple of four.
pub const DDF_BITS: u16 = 0x00fe;

/// Words moved by one bitplane transfer, from `FMODE`'s `BPAGEM` and `BPL32`.
///
/// §4's first `FMODE` table, read as words: "by 2 bytes", "by 4 bytes" twice
/// over and "by 8 bytes".
#[must_use]
#[inline]
pub const fn bitplane_words(fmode: u16) -> u16 {
    match fmode & (BPAGEM | BPL32) {
        0 => 1,
        x if x == BPAGEM | BPL32 => 4,
        _ => 2,
    }
}

/// Words moved by one sprite transfer, from `FMODE`'s `SPAGEM` and `SPR32`.
///
/// §4's second `FMODE` table, which is the first one with the sprite bits.
#[must_use]
#[inline]
pub const fn sprite_words(fmode: u16) -> u16 {
    bitplane_words(fmode >> 2)
}

/// Whether `BSCAN2` makes this line's modulus `BPL2MOD` rather than `BPL1MOD`.
///
/// `false` on a primary line — "lines whose LSBs of beam counter and DIWSTRT
/// match" — and `true` on an alternate one. `diwstrt` is the whole register;
/// its `V0` is bit 8.
#[must_use]
#[inline]
pub const fn scan_double_modulo(vpos: u16, diwstrt: u16) -> usize {
    ((vpos ^ (diwstrt >> 8)) & 1) as usize
}

/// Whether sprite `x` is scan-doubled: `SSCAN2` globally and `SH10` in its own
/// `SPRxPOS` (§2, *Sprites*).
#[must_use]
#[inline]
pub const fn sprite_scan_doubled(fmode: u16, sprpos: u16) -> bool {
    fmode & SSCAN2 != 0 && sprpos & SH10 != 0
}

/// Whether a scan-doubled sprite fetches on line `vpos`: "when V0 bit of
/// SPRxPOS register matches V0 bit of vertical beam counter". A sprite that is
/// not scan-doubled always fetches.
#[must_use]
#[inline]
pub const fn sprite_fetches_on(fmode: u16, sprpos: u16, vpos: u16) -> bool {
    !sprite_scan_doubled(fmode, sprpos) || (vpos ^ (sprpos >> 8)) & 1 == 0
}

/// The words of one sprite transfer, left-justified in a `u64` for
/// [`Video::sprite_dma`](crate::dev::amiga::denise::Video::sprite_dma).
///
/// "The MSB is output first, and is therefore always on the left" (§4,
/// `BPLxDAT`, and `SPRxDATA`'s page says the same of sprites: "serially
/// outputed to the display, MSB first on the left"), so the first word
/// fetched is the leftmost and goes in bits 63–48. A one-word transfer is
/// exactly what a register write to `SPRxDATA` puts there.
#[must_use]
pub fn sprite_bits(words: &[u16]) -> u64 {
    let mut bits = 0u64;
    for (i, word) in words.iter().enumerate().take(4) {
        bits |= u64::from(*word) << (48 - 16 * i);
    }
    bits
}

/// The plane count `BPLCON0` asks for, clamped to what the part has.
///
/// §4's `BPLCON0` page: "BPUx = Bit plane use code 0000-1000 (NONE thru 8
/// inclusive)", with `BPU3` at bit 4 and `BPU2`–`BPU0` at 14–12. An 8370 or an
/// 8375 has three bits and six planes.
#[must_use]
#[inline]
pub fn planes(rev: Revision, bplcon0: u16) -> usize {
    if rev.is_aga() {
        let bpu = ((bplcon0 >> 12) & 7) | ((bplcon0 >> 1) & 8);
        usize::from(bpu).min(8)
    } else {
        usize::from((bplcon0 >> 12) & 7).min(6)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fmodes_four_widths_are_the_specifications_table() {
        // "by 2 bytes (as before)", "by 4 bytes" twice, "by 8 bytes".
        assert_eq!(bitplane_words(0), 1);
        assert_eq!(bitplane_words(BPL32), 2);
        assert_eq!(bitplane_words(BPAGEM), 2);
        assert_eq!(bitplane_words(BPAGEM | BPL32), 4);
        // The sprite bits are the same table two bits up, and the bitplane
        // bits do not reach it.
        assert_eq!(sprite_words(0), 1);
        assert_eq!(sprite_words(SPR32), 2);
        assert_eq!(sprite_words(SPAGEM), 2);
        assert_eq!(sprite_words(SPAGEM | SPR32), 4);
        assert_eq!(sprite_words(BPAGEM | BPL32), 1, "bitplane bits, not these");
        assert_eq!(bitplane_words(SPAGEM | SPR32), 1, "and the other way");
    }

    #[test]
    fn bscan2_picks_the_modulus_by_the_lines_parity() {
        // DIWSTRT $2C81: V0 of $2C is 0, so even lines are primary.
        assert_eq!(scan_double_modulo(0x2c, 0x2c81), 0);
        assert_eq!(scan_double_modulo(0x2d, 0x2c81), 1);
        // A window starting on an odd line inverts it.
        assert_eq!(scan_double_modulo(0x2d, 0x2d81), 0);
        assert_eq!(scan_double_modulo(0x2c, 0x2d81), 1);
    }

    #[test]
    fn sscan2_needs_both_the_global_bit_and_the_sprites_own() {
        assert!(!sprite_scan_doubled(0, SH10));
        assert!(!sprite_scan_doubled(SSCAN2, 0));
        assert!(sprite_scan_doubled(SSCAN2, SH10));
        // SV0 = SPRxPOS bit 8. A doubled sprite whose start is on an even line
        // fetches on even lines only.
        let pos = 0x2c00 | SH10;
        assert!(sprite_fetches_on(SSCAN2, pos, 0x2c));
        assert!(!sprite_fetches_on(SSCAN2, pos, 0x2d));
        assert!(sprite_fetches_on(SSCAN2, pos, 0x2e));
        // Without SSCAN2 the same sprite fetches every line.
        assert!(sprite_fetches_on(0, pos, 0x2d));
    }

    #[test]
    fn a_sprite_transfer_is_left_justified_msb_first() {
        assert_eq!(sprite_bits(&[0x8001]), 0x8001_0000_0000_0000);
        assert_eq!(sprite_bits(&[0x1234, 0x5678]), 0x1234_5678_0000_0000);
        assert_eq!(
            sprite_bits(&[0x0123, 0x4567, 0x89ab, 0xcdef]),
            0x0123_4567_89ab_cdef
        );
    }

    #[test]
    fn bpu3_is_bplcon0_bit_4_and_only_alice_has_it() {
        let aga = Revision::Aga;
        let ecs = Revision::Ecs { reach: 2 << 20 };
        for bpu in 0..=8u16 {
            let bplcon0 = (bpu & 7) << 12 | (bpu & 8) << 1;
            assert_eq!(planes(aga, bplcon0), usize::from(bpu), "BPU = {bpu}");
        }
        // Nine through fifteen have only eight planes to fetch.
        for bpu in 9..16u16 {
            let bplcon0 = (bpu & 7) << 12 | (bpu & 8) << 1;
            assert_eq!(planes(aga, bplcon0), 8, "BPU = {bpu}");
        }
        // An ECS part ignores bit 4 and stops at six.
        assert_eq!(planes(ecs, 8 << 12 | 1 << 4), 0);
        assert_eq!(planes(ecs, 7 << 12 | 1 << 4), 6);
        assert_eq!(planes(ecs, 5 << 12), 5);
    }
}
