//! RSMON/serial — rsemu's own monitor for a 6502 with an ACIA on it.
//!
//! # Why this exists
//!
//! The monitor everyone puts on this board is Ben Eater's adaptation of
//! Wozmon, published as a gist with **no licence, no copyright line and no
//! attribution**, and itself derived from Steve Wozniak's Apple 1 monitor,
//! whose status is also unclear. Two layers of unclear provenance is two more
//! than an MIT crate can absorb (`CLAUDE.md`, "Provenance"), so rsemu neither
//! vendors it nor copies from it. Point `--rom` at an image if you have one;
//! give no `--rom` and you get this.
//!
//! This is a rewrite of [`apple1::monitor`](crate::dev::apple1::monitor)'s
//! RSMON — ours, © Karpelès Lab Inc., MIT like the rest of the crate — moved
//! from `$FF00` to `$8000` and re-plumbed onto a
//! [`w65c51n`](super::acia). The serial version is *shorter* than the PIA one:
//! there is no `DA`/`RDA` handshake and no display busy bit, just two status
//! bits to poll.
//!
//! # What it does
//!
//! A hexadecimal examine/deposit monitor, one keystroke at a time, echoing as
//! it goes:
//!
//! ```text
//! RSMON
//! >8000                          four hex digits and Return
//! 8000: D8 A2 FF 9A A9 1F 8D 03
//! >                              Return again walks on eight bytes
//! 8008: 50 A9 0B 8D 02 50 A2 00
//! >0200:AA                       an address, a colon, a byte, Return
//! >0200                          and it is there
//! 0200: AA 00 00 00 00 00 00 00
//! >
//! ```
//!
//! Hexadecimal digits shift into a 16-bit address from the right, so four of
//! them replace it completely and there is no "start over" key to learn. `:`
//! switches to entering a byte; Return stores it and advances, so `:` `AA`
//! Return `:` `BB` Return fills consecutive bytes. Anything else is echoed and
//! ignored.
//!
//! Zero page `$00`/`$01` hold the address, `$02` the byte being entered and
//! `$03` the mode. Nothing else in RAM is touched but the stack, so the monitor
//! can examine and deposit anywhere above `$04`.
//!
//! Line endings are the one place this differs from the Apple 1 version by
//! choice rather than by wiring. The Apple 1's terminal treated a bare CR as a
//! new line; a serial terminal does not, so `CRLF` sends both characters and
//! the echoed CR that ends a typed line is completed with an LF before anything
//! is printed under it.
//!
//! # The listing
//!
//! Assembled at `$8000` and 261 bytes long. The socket is 32 KiB
//! ([`rom`](super::rom)), so everything between the end of this and the vectors
//! at `$FFFA` is unprogrammed `$FF`.
//!
//! It is reproduced here in full because [`RSMON`] is otherwise unreadable, and
//! because a test below walks the same bytes with the crate's own 6502
//! disassembler and checks that the instruction boundaries and the register
//! accesses are the ones written here.
//!
//! ```text
//! ACIA_DATA   = $5000     read: the byte that arrived; write: send one
//! ACIA_STATUS = $5001     bit 4 TDRE, bit 3 RDRF
//! ACIA_CMD    = $5002
//! ACIA_CTRL   = $5003
//! ADDRL       = $00       the 16-bit address, low byte
//! ADDRH       = $01       and high
//! BYTE        = $02       the byte being entered
//! MODE        = $03       0 examine, 1 deposit
//!
//! 8000  D8        RESET:  CLD
//! 8001  A2 FF             LDX #$FF
//! 8003  9A                TXS
//! 8004  A9 1F             LDA #$1F        ; 8-N-1, 19200, baud generator
//! 8006  8D 03 50          STA ACIA_CTRL
//! 8009  A9 0B             LDA #$0B        ; DTR low, RTS low, no interrupts
//! 800B  8D 02 50          STA ACIA_CMD
//! 800E  A2 00             LDX #$00
//! 8010  8A                TXA
//! 8011  85 00             STA ADDRL
//! 8013  85 01             STA ADDRH
//! 8015  85 03             STA MODE
//! 8017  BD FF 80  BANNER: LDA MSG,X
//! 801A  F0 06             BEQ NEWCMD
//! 801C  20 C3 80          JSR PUTC
//! 801F  E8                INX
//! 8020  D0 F5             BNE BANNER
//! 8022  20 D0 80  NEWCMD: JSR CRLF
//! 8025  A9 3E     PROMPT: LDA #'>'
//! 8027  20 C3 80          JSR PUTC
//! 802A  20 DA 80  MAIN:   JSR GETC        ; blocks, and echoes
//! 802D  29 7F             AND #$7F
//! 802F  C9 0D             CMP #$0D
//! 8031  F0 35             BEQ ENTER
//! 8033  C9 3A             CMP #':'
//! 8035  F0 27             BEQ SETDEP
//! 8037  20 E7 80          JSR HEXVAL      ; C=1 and A=nibble if hex
//! 803A  90 EE             BCC MAIN
//! 803C  A6 03             LDX MODE
//! 803E  D0 10             BNE DIGDAT
//! 8040  A2 04             LDX #$04        ; shift the nibble into the
//! 8042  06 00     SHLA:   ASL ADDRL       ;   16-bit address
//! 8044  26 01             ROL ADDRH
//! 8046  CA                DEX
//! 8047  D0 F9             BNE SHLA
//! 8049  05 00             ORA ADDRL
//! 804B  85 00             STA ADDRL
//! 804D  4C 2A 80          JMP MAIN
//! 8050  A2 04     DIGDAT: LDX #$04        ; or into the byte
//! 8052  06 02     SHLD:   ASL BYTE
//! 8054  CA                DEX
//! 8055  D0 FB             BNE SHLD
//! 8057  05 02             ORA BYTE
//! 8059  85 02             STA BYTE
//! 805B  4C 2A 80          JMP MAIN
//! 805E  A9 00     SETDEP: LDA #$00
//! 8060  85 02             STA BYTE
//! 8062  A9 01             LDA #$01
//! 8064  85 03             STA MODE
//! 8066  D0 C2             BNE MAIN        ; always: A is 1
//! 8068  A9 0A     ENTER:  LDA #$0A        ; the echoed CR wants its LF
//! 806A  20 C3 80          JSR PUTC
//! 806D  A5 03             LDA MODE
//! 806F  F0 11             BEQ DUMP
//! 8071  A5 02             LDA BYTE        ; deposit and advance
//! 8073  A0 00             LDY #$00
//! 8075  91 00             STA (ADDRL),Y
//! 8077  84 03             STY MODE
//! 8079  E6 00             INC ADDRL
//! 807B  D0 02             BNE ENT1
//! 807D  E6 01             INC ADDRH
//! 807F  4C 25 80  ENT1:   JMP PROMPT
//! 8082  A5 01     DUMP:   LDA ADDRH       ; "AAAA:" then eight bytes
//! 8084  20 B0 80          JSR PRBYTE
//! 8087  A5 00             LDA ADDRL
//! 8089  20 B0 80          JSR PRBYTE
//! 808C  A9 3A             LDA #':'
//! 808E  20 C3 80          JSR PUTC
//! 8091  A0 00             LDY #$00
//! 8093  A9 20     DUMPL:  LDA #' '
//! 8095  20 C3 80          JSR PUTC
//! 8098  B1 00             LDA (ADDRL),Y
//! 809A  20 B0 80          JSR PRBYTE
//! 809D  C8                INY
//! 809E  C0 08             CPY #$08
//! 80A0  D0 F1             BNE DUMPL
//! 80A2  A5 00             LDA ADDRL       ; address += 8
//! 80A4  18                CLC
//! 80A5  69 08             ADC #$08
//! 80A7  85 00             STA ADDRL
//! 80A9  90 02             BCC DMP1
//! 80AB  E6 01             INC ADDRH
//! 80AD  4C 22 80  DMP1:   JMP NEWCMD
//! 80B0  48        PRBYTE: PHA
//! 80B1  4A                LSR
//! 80B2  4A                LSR
//! 80B3  4A                LSR
//! 80B4  4A                LSR
//! 80B5  20 B9 80          JSR PRHEX
//! 80B8  68                PLA
//! 80B9  29 0F     PRHEX:  AND #$0F
//! 80BB  09 30             ORA #$30
//! 80BD  C9 3A             CMP #$3A
//! 80BF  90 02             BCC PUTC
//! 80C1  69 06             ADC #$06        ; C=1 here, so +7: '9'+1 -> 'A'
//! 80C3  48        PUTC:   PHA
//! 80C4  AD 01 50  PUTC1:  LDA ACIA_STATUS
//! 80C7  29 10             AND #$10        ; TDRE: room for another byte
//! 80C9  F0 F9             BEQ PUTC1
//! 80CB  68                PLA
//! 80CC  8D 00 50          STA ACIA_DATA
//! 80CF  60                RTS
//! 80D0  A9 0D     CRLF:   LDA #$0D
//! 80D2  20 C3 80          JSR PUTC
//! 80D5  A9 0A             LDA #$0A
//! 80D7  4C C3 80          JMP PUTC
//! 80DA  AD 01 50  GETC:   LDA ACIA_STATUS
//! 80DD  29 08             AND #$08        ; RDRF: a byte is waiting
//! 80DF  F0 F9             BEQ GETC
//! 80E1  AD 00 50          LDA ACIA_DATA
//! 80E4  4C C3 80          JMP PUTC        ; tail call: echoes and returns A
//! 80E7  C9 30     HEXVAL: CMP #'0'
//! 80E9  90 12             BCC HVBAD
//! 80EB  C9 3A             CMP #'9'+1
//! 80ED  90 0A             BCC HVDIG
//! 80EF  C9 41             CMP #'A'
//! 80F1  90 0A             BCC HVBAD
//! 80F3  C9 47             CMP #'F'+1
//! 80F5  B0 06             BCS HVBAD
//! 80F7  E9 06             SBC #$06        ; C=0 here, so -7
//! 80F9  29 0F     HVDIG:  AND #$0F
//! 80FB  38                SEC
//! 80FC  60                RTS
//! 80FD  18        HVBAD:  CLC
//! 80FE  60                RTS
//! 80FF  52 53 4D  MSG:    .byte "RSMON", 0
//!       4F 4E 00
//!
//! FFFA  00 80             .word RESET     ; NMI
//! FFFC  00 80             .word RESET     ; RESET
//! FFFE  00 80             .word RESET     ; IRQ
//! ```
//!
//! # Why nothing here touches the VIA
//!
//! The monitor needs a console and nothing else, and the 65C22 comes up with
//! both ports as inputs and both timers idle, which is harmless. A program
//! that wants the VIA configures it; a monitor that configured it for you would
//! be guessing at what is wired to the port headers.

/// Where [`RSMON`] is assembled, and the address A15 alone decodes the EEPROM
/// at.
pub const RSMON_BASE: u64 = 0x8000;

/// RSMON/serial: 261 bytes of 6502, assembled at `$8000`.
///
/// This is the code alone. [`RSMON_IMAGE`] is what a machine binds: this,
/// padded to the socket, with the vectors at the top.
pub static RSMON: &[u8] = &[
    0xd8, 0xa2, 0xff, 0x9a, 0xa9, 0x1f, 0x8d, 0x03, 0x50, 0xa9, 0x0b, 0x8d, 0x02, 0x50, 0xa2, 0x00,
    0x8a, 0x85, 0x00, 0x85, 0x01, 0x85, 0x03, 0xbd, 0xff, 0x80, 0xf0, 0x06, 0x20, 0xc3, 0x80, 0xe8,
    0xd0, 0xf5, 0x20, 0xd0, 0x80, 0xa9, 0x3e, 0x20, 0xc3, 0x80, 0x20, 0xda, 0x80, 0x29, 0x7f, 0xc9,
    0x0d, 0xf0, 0x35, 0xc9, 0x3a, 0xf0, 0x27, 0x20, 0xe7, 0x80, 0x90, 0xee, 0xa6, 0x03, 0xd0, 0x10,
    0xa2, 0x04, 0x06, 0x00, 0x26, 0x01, 0xca, 0xd0, 0xf9, 0x05, 0x00, 0x85, 0x00, 0x4c, 0x2a, 0x80,
    0xa2, 0x04, 0x06, 0x02, 0xca, 0xd0, 0xfb, 0x05, 0x02, 0x85, 0x02, 0x4c, 0x2a, 0x80, 0xa9, 0x00,
    0x85, 0x02, 0xa9, 0x01, 0x85, 0x03, 0xd0, 0xc2, 0xa9, 0x0a, 0x20, 0xc3, 0x80, 0xa5, 0x03, 0xf0,
    0x11, 0xa5, 0x02, 0xa0, 0x00, 0x91, 0x00, 0x84, 0x03, 0xe6, 0x00, 0xd0, 0x02, 0xe6, 0x01, 0x4c,
    0x25, 0x80, 0xa5, 0x01, 0x20, 0xb0, 0x80, 0xa5, 0x00, 0x20, 0xb0, 0x80, 0xa9, 0x3a, 0x20, 0xc3,
    0x80, 0xa0, 0x00, 0xa9, 0x20, 0x20, 0xc3, 0x80, 0xb1, 0x00, 0x20, 0xb0, 0x80, 0xc8, 0xc0, 0x08,
    0xd0, 0xf1, 0xa5, 0x00, 0x18, 0x69, 0x08, 0x85, 0x00, 0x90, 0x02, 0xe6, 0x01, 0x4c, 0x22, 0x80,
    0x48, 0x4a, 0x4a, 0x4a, 0x4a, 0x20, 0xb9, 0x80, 0x68, 0x29, 0x0f, 0x09, 0x30, 0xc9, 0x3a, 0x90,
    0x02, 0x69, 0x06, 0x48, 0xad, 0x01, 0x50, 0x29, 0x10, 0xf0, 0xf9, 0x68, 0x8d, 0x00, 0x50, 0x60,
    0xa9, 0x0d, 0x20, 0xc3, 0x80, 0xa9, 0x0a, 0x4c, 0xc3, 0x80, 0xad, 0x01, 0x50, 0x29, 0x08, 0xf0,
    0xf9, 0xad, 0x00, 0x50, 0x4c, 0xc3, 0x80, 0xc9, 0x30, 0x90, 0x12, 0xc9, 0x3a, 0x90, 0x0a, 0xc9,
    0x41, 0x90, 0x0a, 0xc9, 0x47, 0xb0, 0x06, 0xe9, 0x06, 0x29, 0x0f, 0x38, 0x60, 0x18, 0x60, 0x52,
    0x53, 0x4d, 0x4f, 0x4e, 0x00,
];

/// A 32 KiB EEPROM image: [`RSMON`], unprogrammed cells, and the vectors.
///
/// Built at compile time rather than written out, because 32768 hexadecimal
/// literals of which 32507 are `$FF` would be a worse thing to review than the
/// four lines that produce them.
pub static RSMON_IMAGE: &[u8; 32_768] = &{
    let mut image = [0xffu8; 32_768];
    let mut i = 0;
    while i < RSMON.len() {
        image[i] = RSMON[i];
        i += 1;
    }
    // NMI at $FFFA, RESET at $FFFC, IRQ at $FFFE: all three at the entry
    // point, because a monitor with no interrupt handler that took an
    // interrupt would otherwise run off into unprogrammed memory.
    let mut v = 0x7ffa;
    while v < 0x8000 {
        image[v] = (RSMON_BASE & 0xff) as u8;
        image[v + 1] = (RSMON_BASE >> 8) as u8;
        v += 2;
    }
    image
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_vectors_all_point_at_the_entry_point() {
        // The reset vector is what the machine actually follows, and $FFFC is
        // the top of this socket. Get it wrong and nothing runs at all.
        for vector in [0x7ffa, 0x7ffc, 0x7ffe] {
            let target = u16::from(RSMON_IMAGE[vector]) | (u16::from(RSMON_IMAGE[vector + 1]) << 8);
            assert_eq!(u64::from(target), RSMON_BASE, "vector at ${vector:04x}");
        }
    }

    #[test]
    fn the_image_is_the_code_then_unprogrammed_cells() {
        assert_eq!(RSMON.len(), 261);
        assert_eq!(&RSMON_IMAGE[..RSMON.len()], RSMON);
        assert_eq!(RSMON_IMAGE[RSMON.len()], 0xff, "and nothing after it");
        assert_eq!(&RSMON[255..], b"RSMON\0", "the banner ends the listing");
    }

    /// The listing decodes as one unbroken run of instructions, and reaches for
    /// nothing in the I/O window but the four ACIA registers.
    ///
    /// Uses the crate's own disassembler — generated from the same table the
    /// interpreter decodes with (`CLAUDE.md`, "CPU cores") — so this is a real
    /// decode rather than a scan for byte pairs that look like addresses. A
    /// listing whose instruction boundaries had drifted would not end where the
    /// message begins, and a typo'd `$5004` would be a register nothing answers
    /// and a guest that wedges on it.
    #[cfg(feature = "cpu-mos6502")]
    #[test]
    fn every_instruction_decodes_and_only_the_acia_is_touched() {
        use crate::cpu::mos6502::disasm::disassemble_run;

        let code_end = (RSMON_BASE as u16).wrapping_add(RSMON.len() as u16 - 6);
        let run = disassemble_run(RSMON_BASE as u16, RSMON.len(), |addr| {
            RSMON
                .get(usize::from(addr.wrapping_sub(RSMON_BASE as u16)))
                .copied()
        });

        let mut registers = alloc::vec::Vec::new();
        let mut end = RSMON_BASE as u16;
        for d in &run {
            assert!(!d.truncated, "truncated decode at ${:04x}", d.pc);
            if d.pc >= code_end {
                break;
            }
            end = d.pc.wrapping_add(u16::from(d.len));
            if d.len == 3 {
                let target = d.word();
                // The I/O half of the map: $4000-$7FFF.
                if (0x4000..0x8000).contains(&target) && !registers.contains(&target) {
                    registers.push(target);
                }
            }
        }
        assert_eq!(end, code_end, "the code does not end where MSG begins");
        registers.sort_unstable();
        assert_eq!(registers, [0x5000, 0x5001, 0x5002, 0x5003]);
    }
}
