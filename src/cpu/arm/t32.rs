//! The pieces of the Thumb-2 (T32) encoding that do not depend on a profile.
//!
//! Both the A-profile core (`aprofile`, from v6T2) and the M-profile core
//! (`v7m`) execute T32, and a handful of its rules are pure functions of the
//! encoding with no machine state behind them: which first halfwords start a
//! 32-bit instruction, how a twelve-bit "modified immediate" expands, and how
//! the eight-bit `ITSTATE` walks through an `IT` block. They are written once
//! here rather than once per profile. Everything with a register file behind
//! it — which instructions exist, what a `CPSR` or an `EPSR` looks like — stays
//! in the profile.
//!
//! # Sources
//!
//! ARM DDI 0406C (ARMv7-A/R): A6.1 ("Thumb instruction set encoding", the
//! halfword that selects a 32-bit instruction), A6.3.2 (`ThumbExpandImm_C`),
//! A2.5.2 ("ITSTATE", with `InITBlock`, `LastInITBlock` and `ITAdvance`) and
//! A8.8.54 (`IT`). DDI 0403 (ARMv7-M) A5.1, A5.3.2 and A7.3 say the same
//! thing for the M profile. No emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

/// Whether a first halfword starts a 32-bit instruction (DDI 0406C A6.1):
/// bits 15..11 are `0b11101`, `0b11110` or `0b11111`.
///
/// On a part *without* Thumb-2 the same patterns are the ARMv4T/ARMv5T `BL`
/// and `BLX` halves, each a 16-bit instruction of its own; asking this
/// question is the caller's job only once it knows the part has Thumb-2.
#[inline]
#[must_use]
pub const fn is_32bit(first: u16) -> bool {
    matches!(first >> 11, 0b11101..=0b11111)
}

/// `ThumbExpandImm_C(imm12, carry_in)` (DDI 0406C A6.3.2).
///
/// Returns the expanded value and the carry it produces, or `None` where the
/// expansion leaves the carry flag alone. The two halves of the encoding are
/// genuinely different operations: with `imm12<11:10> == 00` a byte is
/// replicated into one of four patterns and no flag is touched; otherwise an
/// eight-bit value with its top bit forced set is rotated right by
/// `imm12<11:7>` — any amount from 8 to 31, odd ones included, which is why
/// A32's even-rotation immediate cannot represent every result — and bit 31
/// of the result becomes the carry.
///
/// The `imm12<9:8> != 00` patterns with a zero byte are UNPREDICTABLE; this
/// expands them as written (to zero), which is the reading that needs no
/// special case.
#[must_use]
pub const fn thumb_expand_imm(imm12: u32) -> (u32, Option<bool>) {
    if (imm12 >> 10) & 0b11 == 0 {
        let byte = imm12 & 0xff;
        let value = match (imm12 >> 8) & 0b11 {
            0b00 => byte,
            0b01 => (byte << 16) | byte,
            0b10 => (byte << 24) | (byte << 8),
            _ => (byte << 24) | (byte << 16) | (byte << 8) | byte,
        };
        (value, None)
    } else {
        let unrotated = 0x80 | (imm12 & 0x7f);
        let value = unrotated.rotate_right((imm12 >> 7) & 0x1f);
        (value, Some(value & 0x8000_0000 != 0))
    }
}

/// The eight-bit `IT` execution state, `IT[7:0]` (DDI 0406C A2.5.2).
///
/// `IT[7:5]` is the base condition of the block, and `IT[4:0]` the condition's
/// low bit for each remaining instruction followed by a terminating one. A
/// profile keeps the bits wherever its status register puts them — split
/// across `CPSR[26:25]` and `CPSR[15:10]` on the A profile — and converts at
/// the edge; the walk itself is the same everywhere.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct ItState(pub u8);

impl ItState {
    /// Outside any `IT` block.
    pub const NONE: ItState = ItState(0);

    /// The state an `IT <firstcond>, <mask>` instruction leaves behind
    /// (DDI 0406C A8.8.54: `ITSTATE.IT<7:0> = firstcond:mask`).
    #[must_use]
    pub const fn from_it(firstcond: u8, mask: u8) -> ItState {
        ItState(((firstcond & 0xf) << 4) | (mask & 0xf))
    }

    /// `InITBlock()`: the next instruction is inside an `IT` block.
    #[inline]
    #[must_use]
    pub const fn in_block(self) -> bool {
        self.0 & 0xf != 0
    }

    /// `LastInITBlock()`: the next instruction is the block's last.
    #[inline]
    #[must_use]
    pub const fn last_in_block(self) -> bool {
        self.0 & 0xf == 0b1000
    }

    /// The four-bit condition the next instruction executes under, when
    /// [`in_block`](ItState::in_block).
    #[inline]
    #[must_use]
    pub const fn cond(self) -> u8 {
        self.0 >> 4
    }

    /// `ITAdvance()`: the state after one instruction of the block has
    /// executed, or been skipped because its condition failed.
    #[inline]
    #[must_use]
    pub const fn advance(self) -> ItState {
        if self.0 & 0b111 == 0 {
            ItState(0)
        } else {
            ItState((self.0 & 0b1110_0000) | ((self.0 << 1) & 0b0001_1111))
        }
    }

    /// The `IT` mnemonic for a `firstcond`/`mask` pair: `IT`, `ITT`, `ITE`,
    /// `ITTEE` and so on (DDI 0406C A8.8.54). Each mask bit above the
    /// terminating one says `T` when it equals `firstcond<0>` and `E` when it
    /// does not.
    #[must_use]
    pub const fn mnemonic(firstcond: u8, mask: u8) -> &'static str {
        // Index by how many instructions follow the first (0..=3) and by the
        // then/else pattern of those, `T` = 0.
        const NAMES: [[&str; 8]; 4] = [
            ["IT", "IT", "IT", "IT", "IT", "IT", "IT", "IT"],
            ["ITT", "ITE", "ITT", "ITE", "ITT", "ITE", "ITT", "ITE"],
            [
                "ITTT", "ITET", "ITTE", "ITEE", "ITTT", "ITET", "ITTE", "ITEE",
            ],
            [
                "ITTTT", "ITETT", "ITTET", "ITEET", "ITTTE", "ITETE", "ITTEE", "ITEEE",
            ],
        ];
        let mask = mask & 0xf;
        if mask == 0 {
            return "IT";
        }
        let extra = 3 - mask.trailing_zeros() as usize;
        let low = firstcond & 1;
        // Bit `3 - i` of the mask belongs to instruction `i + 1`.
        let mut pattern = 0usize;
        let mut i = 0;
        while i < extra {
            if (mask >> (3 - i)) & 1 != low {
                pattern |= 1 << i;
            }
            i += 1;
        }
        NAMES[extra][pattern]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modified_immediates_expand_as_the_manual_tabulates() {
        // DDI 0406C A6.3.2's table, one row each.
        assert_eq!(thumb_expand_imm(0x0ab), (0xab, None));
        assert_eq!(thumb_expand_imm(0x1ab), (0x00ab_00ab, None));
        assert_eq!(thumb_expand_imm(0x2ab), (0xab00_ab00, None));
        assert_eq!(thumb_expand_imm(0x3ab), (0xabab_abab, None));
        // `1:bcdefgh` rotated right by 8 is 0x8000_0000 plus the low bits.
        assert_eq!(thumb_expand_imm(0x400), (0x8000_0000, Some(true)));
        // Rotation 9 is odd — not an A32 immediate.
        assert_eq!(thumb_expand_imm(0x4ff), (0x7f80_0000, Some(false)));
        assert_eq!(thumb_expand_imm(0xfff), (0x0000_01fe, Some(false)));
    }

    #[test]
    fn itstate_walks_a_block_and_ends_it() {
        // ITTE EQ: firstcond 0000, mask 0110 (T = 0, E = 1, then the stop bit).
        let mut it = ItState::from_it(0b0000, 0b0110);
        assert_eq!(ItState::mnemonic(0, 0b0110), "ITTE");
        let mut conds = alloc::vec::Vec::new();
        while it.in_block() {
            conds.push(it.cond());
            it = it.advance();
        }
        assert_eq!(conds, [0b0000, 0b0000, 0b0001]);
        assert_eq!(it, ItState::NONE);
        assert!(ItState::from_it(1, 0b1000).last_in_block());
    }

    #[test]
    fn it_mnemonics_follow_the_condition_parity() {
        assert_eq!(ItState::mnemonic(0b0001, 0b1000), "IT");
        // NE is odd: a mask bit of 1 now means Then.
        assert_eq!(ItState::mnemonic(0b0001, 0b1100), "ITT");
        assert_eq!(ItState::mnemonic(0b0001, 0b0100), "ITE");
        assert_eq!(ItState::mnemonic(0b0000, 0b0001), "ITTTT");
        assert_eq!(ItState::mnemonic(0b0000, 0b1111), "ITEEE");
        assert_eq!(ItState::mnemonic(0b0000, 0b0111), "ITTEE");
        assert_eq!(ItState::mnemonic(0b0000, 0b0011), "ITTTE");
    }
}
