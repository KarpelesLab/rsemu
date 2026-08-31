//! The Woz Monitor of 1976, re-plumbed onto a 65C51 ACIA.
//!
//! # Provenance
//!
//! **The monitor is public domain.** Its complete source was printed in the
//! *Apple-1 Operation Manual* (Apple Computer Company, 1976), a US work
//! published before 1978 and therefore under the 1909 Copyright Act, whose
//! notice requirement was strict: publication without notice put the work into
//! the public domain immediately. The manual carries no notice.
//! [`docs/platforms/apple1.md`] records the determination in full, with its
//! evidence and its caveats, because `ROADMAP.md` §1 requires provenance to be
//! auditable by someone who was not in the room.
//!
//! [`docs/platforms/apple1.md`]: https://github.com/KarpelesLab/rsemu/blob/master/docs/platforms/apple1.md
//!
//! **The bytes here were transcribed from that listing, not from anyone's
//! port.** Ben Eater's well-known ACIA adaptation exists and is CC-BY —
//! usable, with attribution — but it is a *different* program from this one:
//! it works in seven-bit ASCII, and its transmit path is a delay loop built on
//! `DEC A`, an instruction only the CMOS part has. Nothing here is derived from
//! it. Where a test below runs his image it does so as a *fixture*, fetched
//! and never committed, and says so.
//!
//! The transcription is checked rather than trusted. The manual prints an
//! address for every instruction, and the module test walks these bytes with
//! the crate's own 6502 disassembler and asserts that the instruction
//! boundaries land on exactly that list. An OCR slip in either column — and
//! the scan has several — fails it.
//!
//! # What was changed, and why each change is the size it is
//!
//! Three blocks of the original talk to the Apple 1's MC6821. Each is replaced
//! *in place* and padded with `NOP` to its original length, so every other
//! address in the page is the address the manual prints and the two can be read
//! side by side. That is worth three wasted bytes.
//!
//! | At | Was | Is |
//! | --- | --- | --- |
//! | `$FF02` (13 bytes) | `LDY #$7F`, then the PIA's data-direction and control setup | `LDY #$7F` kept, then the ACIA's control and command registers, then one `NOP` |
//! | `$FF29` (8 bytes) | `LDA KBDCR / BPL / LDA KBD` | `JSR GETKEY`, then five `NOP` |
//! | `$FFEF` (9 bytes) | `BIT DSP / BMI / STA DSP / RTS` | `JMP PUTC`, then six `NOP` |
//!
//! The two new subroutines are at `$FE00`, in the same 32 KiB EEPROM:
//!
//! ```text
//! FE00  AD 01 50  GETKEY: LDA $5001      ; the ACIA's status register
//! FE03  29 08             AND #$08       ; RDRF: a byte has arrived
//! FE05  F0 F9             BEQ GETKEY
//! FE07  AD 00 50          LDA $5000
//! FE0A  09 80             ORA #$80       ; the Apple 1 strapped PA7 high, and
//! FE0C  60                RTS            ;   the monitor compares against $8D
//!
//! FE0D  48        PUTC:   PHA
//! FE0E  AD 01 50  PUTC1:  LDA $5001
//! FE11  29 10             AND #$10       ; TDRE: room for another byte
//! FE13  F0 F9             BEQ PUTC1
//! FE15  68                PLA
//! FE16  29 7F             AND #$7F       ; the wire carries seven bits
//! FE18  8D 00 50          STA $5000
//! FE1B  09 80             ORA #$80       ; ECHO must return A untouched
//! FE1D  60                RTS
//! ```
//!
//! **`LDY #$7F` stays**, and it is not part of the PIA setup even though it sits
//! in the middle of it. `NOTCR` is entered straight from reset with `Y` at
//! `$7F`; its `INY` makes `Y` negative, the `BPL NEXTCHAR` below is therefore
//! not taken, and the fall-through into `ESCAPE` is what prints the `\` and the
//! carriage return you see at power-on. Drop that one instruction and the
//! monitor comes up silent and waiting, which looks exactly like a machine that
//! has not booted. It was found by dropping it.
//!
//! `ORA #$80` on the way in and `AND #$7F` on the way out is the whole of the
//! adaptation's cleverness, and it is what keeps the monitor's own code
//! byte-identical: every comparison in it — `$8D` for Return, `$9B` for Escape,
//! `$AE` for `.`, `$BA` for `:`, `$D2` for `R` — is against a character with
//! bit 7 set, because the Apple 1's keyboard strapped PA7 to +5 V. Restoring
//! that bit at the door means none of those constants has to move.
//!
//! One consequence is worth knowing before you type at it: **rub out is `_`**,
//! not backspace. `$DF` is `_` with bit 7 set, and that really is the key the
//! Apple 1 had. Backspace and delete are ordinary characters to this monitor.
//!
//! # Using it
//!
//! ```console
//! $ rsemu run beneater-6502 --monitor wozmon
//! \
//! FF00.FF0F
//!
//! FF00: D8 58 A0 7F A9 1F 8D 03
//! FF08: 50 A9 0B 8D 02 50 EA C9
//! 0300: AA BB CC
//!
//! 0300: 00
//! 0300.0302
//!
//! 0300: AA BB CC
//! ```
//!
//! `AAAA` examines one byte, `AAAA.BBBB` a range, `AAAA: xx yy` deposits, and
//! `AAAAR` runs. Two things in that transcript surprise everyone the first
//! time, and both are Woz's:
//!
//! * The `\` appears only at power-on and on Escape. A completed line just
//!   leaves you on a new one — there is no prompt.
//! * `0300: AA BB CC` answers `0300: 00`. The address is parsed while the
//!   monitor is still in examine mode, so it examines that byte *before* the
//!   `:` switches to store mode and the bytes after it go in.
//!
//! Line endings are bare carriage returns, as they were in 1976: this monitor
//! was written for a terminal that treated `$0D` as a new line and it has not
//! been taught otherwise. [`super::monitor`] — rsemu's own — is the one that
//! sends CR LF.
//!
//! **Zero page `$24`-`$2B` and `$0200`-`$027F` belong to the monitor**, the
//! second of those being its line buffer. Depositing into `$0200` works and
//! then reads back as whatever you last typed. The manual says so on the page
//! before the listing.
//!
//! # What it needs from the CPU
//!
//! Nothing a 1975 NMOS 6502 does not have. The monitor is Woz's original object
//! code and predates the CMOS part by a decade; the two subroutines above were
//! written to the same constraint deliberately, so this image runs on either
//! core. Eater's image does not — see the module docs above.

/// Where the monitor page sits, and what its RESET vector holds.
pub const WOZMON_BASE: u64 = 0xff00;

/// Where the two ACIA subroutines sit.
pub const WOZMON_HELPERS_BASE: u64 = 0xfe00;

/// The monitor's page: the 1976 object code with three blocks replaced.
///
/// `$FF00-$FFFF`, vectors included. The vectors are the manual's own —
/// NMI `$0F00`, RESET `$FF00`, IRQ `$0000` — and the first and last of those
/// point at RAM and at zero page respectively on this board. The monitor's
/// second instruction is `CLI`, so an interrupt taken here would run whatever
/// is there; nothing in this machine asserts one, and changing Woz's vectors to
/// hide that would be a worse kind of wrong than leaving them.
pub static WOZMON_PAGE: &[u8; 256] = &[
    0xd8, 0x58, 0xa0, 0x7f, 0xa9, 0x1f, 0x8d, 0x03, 0x50, 0xa9, 0x0b, 0x8d, 0x02, 0x50, 0xea, 0xc9,
    0xdf, 0xf0, 0x13, 0xc9, 0x9b, 0xf0, 0x03, 0xc8, 0x10, 0x0f, 0xa9, 0xdc, 0x20, 0xef, 0xff, 0xa9,
    0x8d, 0x20, 0xef, 0xff, 0xa0, 0x01, 0x88, 0x30, 0xf6, 0x20, 0x00, 0xfe, 0xea, 0xea, 0xea, 0xea,
    0xea, 0x99, 0x00, 0x02, 0x20, 0xef, 0xff, 0xc9, 0x8d, 0xd0, 0xd4, 0xa0, 0xff, 0xa9, 0x00, 0xaa,
    0x0a, 0x85, 0x2b, 0xc8, 0xb9, 0x00, 0x02, 0xc9, 0x8d, 0xf0, 0xd4, 0xc9, 0xae, 0x90, 0xf4, 0xf0,
    0xf0, 0xc9, 0xba, 0xf0, 0xeb, 0xc9, 0xd2, 0xf0, 0x3b, 0x86, 0x28, 0x86, 0x29, 0x84, 0x2a, 0xb9,
    0x00, 0x02, 0x49, 0xb0, 0xc9, 0x0a, 0x90, 0x06, 0x69, 0x88, 0xc9, 0xfa, 0x90, 0x11, 0x0a, 0x0a,
    0x0a, 0x0a, 0xa2, 0x04, 0x0a, 0x26, 0x28, 0x26, 0x29, 0xca, 0xd0, 0xf8, 0xc8, 0xd0, 0xe0, 0xc4,
    0x2a, 0xf0, 0x97, 0x24, 0x2b, 0x50, 0x10, 0xa5, 0x28, 0x81, 0x26, 0xe6, 0x26, 0xd0, 0xb5, 0xe6,
    0x27, 0x4c, 0x44, 0xff, 0x6c, 0x24, 0x00, 0x30, 0x2b, 0xa2, 0x02, 0xb5, 0x27, 0x95, 0x25, 0x95,
    0x23, 0xca, 0xd0, 0xf7, 0xd0, 0x14, 0xa9, 0x8d, 0x20, 0xef, 0xff, 0xa5, 0x25, 0x20, 0xdc, 0xff,
    0xa5, 0x24, 0x20, 0xdc, 0xff, 0xa9, 0xba, 0x20, 0xef, 0xff, 0xa9, 0xa0, 0x20, 0xef, 0xff, 0xa1,
    0x24, 0x20, 0xdc, 0xff, 0x86, 0x2b, 0xa5, 0x24, 0xc5, 0x28, 0xa5, 0x25, 0xe5, 0x29, 0xb0, 0xc1,
    0xe6, 0x24, 0xd0, 0x02, 0xe6, 0x25, 0xa5, 0x24, 0x29, 0x07, 0x10, 0xc8, 0x48, 0x4a, 0x4a, 0x4a,
    0x4a, 0x20, 0xe5, 0xff, 0x68, 0x29, 0x0f, 0x09, 0xb0, 0xc9, 0xba, 0x90, 0x02, 0x69, 0x06, 0x4c,
    0x0d, 0xfe, 0xea, 0xea, 0xea, 0xea, 0xea, 0xea, 0x00, 0x00, 0x00, 0x0f, 0x00, 0xff, 0x00, 0x00,
];

/// `GETKEY` and `PUTC`, the two subroutines the adaptation adds at `$FE00`.
pub static WOZMON_HELPERS: &[u8; 30] = &[
    0xad, 0x01, 0x50, 0x29, 0x08, 0xf0, 0xf9, 0xad, 0x00, 0x50, 0x09, 0x80, 0x60, 0x48, 0xad, 0x01,
    0x50, 0x29, 0x10, 0xf0, 0xf9, 0x68, 0x29, 0x7f, 0x8d, 0x00, 0x50, 0x09, 0x80, 0x60,
];

/// A 32 KiB EEPROM image: unprogrammed cells, the helpers, and the monitor.
pub static WOZMON_IMAGE: &[u8; 32_768] = &{
    let mut image = [0xffu8; 32_768];
    let mut i = 0;
    while i < WOZMON_HELPERS.len() {
        image[0x7e00 + i] = WOZMON_HELPERS[i];
        i += 1;
    }
    let mut i = 0;
    while i < WOZMON_PAGE.len() {
        image[0x7f00 + i] = WOZMON_PAGE[i];
        i += 1;
    }
    image
};

/// Every address the *Apple-1 Operation Manual*'s listing prints, in order.
///
/// The manual gives one per instruction from `$FF00` to the `RTS` at `$FFF7`.
/// Reproducing the column here is what makes [`the transcription checkable`];
/// the three patched blocks keep their original lengths precisely so that this
/// list still describes the bytes above.
///
/// [`the transcription checkable`]: self::tests::the_listing_walks_the_manuals_own_addresses
#[cfg(test)]
static MANUAL_ADDRESSES: &[u16] = &[
    0xff00, 0xff01, 0xff02, 0xff04, 0xff07, 0xff09, 0xff0c, 0xff0f, 0xff11, 0xff13, 0xff15, 0xff17,
    0xff18, 0xff1a, 0xff1c, 0xff1f, 0xff21, 0xff24, 0xff26, 0xff27, 0xff29, 0xff2c, 0xff2e, 0xff31,
    0xff34, 0xff37, 0xff39, 0xff3b, 0xff3d, 0xff3f, 0xff40, 0xff41, 0xff43, 0xff44, 0xff47, 0xff49,
    0xff4b, 0xff4d, 0xff4f, 0xff51, 0xff53, 0xff55, 0xff57, 0xff59, 0xff5b, 0xff5d, 0xff5f, 0xff62,
    0xff64, 0xff66, 0xff68, 0xff6a, 0xff6c, 0xff6e, 0xff6f, 0xff70, 0xff71, 0xff72, 0xff74, 0xff75,
    0xff77, 0xff79, 0xff7a, 0xff7c, 0xff7d, 0xff7f, 0xff81, 0xff83, 0xff85, 0xff87, 0xff89, 0xff8b,
    0xff8d, 0xff8f, 0xff91, 0xff94, 0xff97, 0xff99, 0xff9b, 0xff9d, 0xff9f, 0xffa1, 0xffa2, 0xffa4,
    0xffa6, 0xffa8, 0xffab, 0xffad, 0xffb0, 0xffb2, 0xffb5, 0xffb7, 0xffba, 0xffbc, 0xffbf, 0xffc1,
    0xffc4, 0xffc6, 0xffc8, 0xffca, 0xffcc, 0xffce, 0xffd0, 0xffd2, 0xffd4, 0xffd6, 0xffd8, 0xffda,
    0xffdc, 0xffdd, 0xffde, 0xffdf, 0xffe0, 0xffe1, 0xffe4, 0xffe5, 0xffe7, 0xffe9, 0xffeb, 0xffed,
    0xffef, 0xfff2, 0xfff4, 0xfff7,
];

/// The 1976 object code, unpatched, exactly as the manual prints it.
///
/// Test-only, and the reference both transcription tests are written against.
/// It costs 256 bytes in a test binary and it is what lets this file claim to
/// *be* the manual's listing rather than to resemble it.
#[cfg(test)]
static ORIGINAL_PAGE: &[u8; 256] = &[
    0xd8, 0x58, 0xa0, 0x7f, 0x8c, 0x12, 0xd0, 0xa9, 0xa7, 0x8d, 0x11, 0xd0, 0x8d, 0x13, 0xd0, 0xc9,
    0xdf, 0xf0, 0x13, 0xc9, 0x9b, 0xf0, 0x03, 0xc8, 0x10, 0x0f, 0xa9, 0xdc, 0x20, 0xef, 0xff, 0xa9,
    0x8d, 0x20, 0xef, 0xff, 0xa0, 0x01, 0x88, 0x30, 0xf6, 0xad, 0x11, 0xd0, 0x10, 0xfb, 0xad, 0x10,
    0xd0, 0x99, 0x00, 0x02, 0x20, 0xef, 0xff, 0xc9, 0x8d, 0xd0, 0xd4, 0xa0, 0xff, 0xa9, 0x00, 0xaa,
    0x0a, 0x85, 0x2b, 0xc8, 0xb9, 0x00, 0x02, 0xc9, 0x8d, 0xf0, 0xd4, 0xc9, 0xae, 0x90, 0xf4, 0xf0,
    0xf0, 0xc9, 0xba, 0xf0, 0xeb, 0xc9, 0xd2, 0xf0, 0x3b, 0x86, 0x28, 0x86, 0x29, 0x84, 0x2a, 0xb9,
    0x00, 0x02, 0x49, 0xb0, 0xc9, 0x0a, 0x90, 0x06, 0x69, 0x88, 0xc9, 0xfa, 0x90, 0x11, 0x0a, 0x0a,
    0x0a, 0x0a, 0xa2, 0x04, 0x0a, 0x26, 0x28, 0x26, 0x29, 0xca, 0xd0, 0xf8, 0xc8, 0xd0, 0xe0, 0xc4,
    0x2a, 0xf0, 0x97, 0x24, 0x2b, 0x50, 0x10, 0xa5, 0x28, 0x81, 0x26, 0xe6, 0x26, 0xd0, 0xb5, 0xe6,
    0x27, 0x4c, 0x44, 0xff, 0x6c, 0x24, 0x00, 0x30, 0x2b, 0xa2, 0x02, 0xb5, 0x27, 0x95, 0x25, 0x95,
    0x23, 0xca, 0xd0, 0xf7, 0xd0, 0x14, 0xa9, 0x8d, 0x20, 0xef, 0xff, 0xa5, 0x25, 0x20, 0xdc, 0xff,
    0xa5, 0x24, 0x20, 0xdc, 0xff, 0xa9, 0xba, 0x20, 0xef, 0xff, 0xa9, 0xa0, 0x20, 0xef, 0xff, 0xa1,
    0x24, 0x20, 0xdc, 0xff, 0x86, 0x2b, 0xa5, 0x24, 0xc5, 0x28, 0xa5, 0x25, 0xe5, 0x29, 0xb0, 0xc1,
    0xe6, 0x24, 0xd0, 0x02, 0xe6, 0x25, 0xa5, 0x24, 0x29, 0x07, 0x10, 0xc8, 0x48, 0x4a, 0x4a, 0x4a,
    0x4a, 0x20, 0xe5, 0xff, 0x68, 0x29, 0x0f, 0x09, 0xb0, 0xc9, 0xba, 0x90, 0x02, 0x69, 0x06, 0x2c,
    0x12, 0xd0, 0x30, 0xfb, 0x8d, 0x12, 0xd0, 0x60, 0x00, 0x00, 0x00, 0x0f, 0x00, 0xff, 0x00, 0x00,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_reset_vector_is_the_manuals_own() {
        assert_eq!(WOZMON_PAGE[0xfc], 0x00);
        assert_eq!(WOZMON_PAGE[0xfd], 0xff);
        assert_eq!(&WOZMON_PAGE[0xfa..0xfc], &[0x00, 0x0f], "NMI, as printed");
        assert_eq!(&WOZMON_PAGE[0xfe..], &[0x00, 0x00], "IRQ, as printed");
        assert_eq!(WOZMON_IMAGE[0x7ffc], 0x00);
        assert_eq!(WOZMON_IMAGE[0x7ffd], 0xff);
        assert_eq!(&WOZMON_IMAGE[0x7e00..0x7e1e], &WOZMON_HELPERS[..]);
    }

    /// The three patched blocks are the only thing that moved.
    ///
    /// Byte for byte against the manual's own object code, which is the claim
    /// the whole file rests on: outside `$FF02+13`, `$FF29+8` and `$FFEF+9`,
    /// this *is* the 1976 monitor.
    #[test]
    fn only_the_pia_blocks_were_replaced() {
        const PATCHED: [core::ops::Range<usize>; 3] = [0x02..0x0f, 0x29..0x31, 0xef..0xf8];
        for (i, (&now, &then)) in WOZMON_PAGE.iter().zip(ORIGINAL_PAGE).enumerate() {
            if PATCHED.iter().any(|r| r.contains(&i)) {
                continue;
            }
            assert_eq!(now, then, "${:04x} is not the manual's byte", 0xff00 + i);
        }
        // And the patches themselves, including the `LDY #$7F` that had to stay.
        assert_eq!(
            &WOZMON_PAGE[0x02..0x0e],
            &[
                0xa0, 0x7f, 0xa9, 0x1f, 0x8d, 0x03, 0x50, 0xa9, 0x0b, 0x8d, 0x02, 0x50
            ]
        );
        assert_eq!(WOZMON_PAGE[0x0e], 0xea, "one byte of padding");
        assert_eq!(&WOZMON_PAGE[0x29..0x2c], &[0x20, 0x00, 0xfe], "JSR GETKEY");
        assert_eq!(&WOZMON_PAGE[0xef..0xf2], &[0x4c, 0x0d, 0xfe], "JMP PUTC");
    }

    /// The transcription check: decoding the bytes must land on exactly the
    /// addresses the 1976 manual prints beside them.
    ///
    /// Two independently OCR'd columns of a fifty-year-old scan agreeing
    /// instruction by instruction is a far stronger statement than either one
    /// alone, and it is the reason this file can claim to *be* the manual's
    /// listing rather than to resemble it.
    #[cfg(feature = "cpu-mos6502")]
    #[test]
    fn the_listing_walks_the_manuals_own_addresses() {
        use crate::cpu::mos6502::disasm::disassemble_run;

        // The *unpatched* page, so this says something about the transcription
        // rather than about the surgery. What the surgery preserved is
        // `only_the_pia_blocks_were_replaced`'s business.
        let run = disassemble_run(WOZMON_BASE as u16, MANUAL_ADDRESSES.len(), |addr| {
            ORIGINAL_PAGE
                .get(usize::from(addr.wrapping_sub(WOZMON_BASE as u16)))
                .copied()
        });
        let decoded: alloc::vec::Vec<u16> = run
            .iter()
            .inspect(|d| assert!(!d.truncated, "truncated decode at ${:04x}", d.pc))
            .map(|d| d.pc)
            .collect();
        assert_eq!(decoded, MANUAL_ADDRESSES);
        // And the last of them really is the end of the code: `$FFF7 RTS`,
        // with the unused pair and the three vectors after it.
        assert_eq!(ORIGINAL_PAGE[0xf7], 0x60);
        assert_eq!(&ORIGINAL_PAGE[0xf8..0xfa], &[0x00, 0x00]);
    }

    /// Nothing here needs a CMOS part.
    ///
    /// The image has to run on the NMOS core as well as on a 65C02, and the one
    /// instruction that would break that — `DEC A`, `$3A`, which the NMOS part
    /// decodes as an undocumented `NOP` and so never terminates a delay loop
    /// built on it — is exactly the instruction Ben Eater's own image uses.
    #[test]
    fn no_65c02_only_opcode_appears() {
        for (i, &byte) in WOZMON_PAGE.iter().enumerate().take(0xf8) {
            assert_ne!(byte, 0x3a, "DEC A at ${:04x}", 0xff00 + i);
        }
        assert!(!WOZMON_HELPERS.contains(&0x3a));
    }
}
