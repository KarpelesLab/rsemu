//! RSMON — rsemu's own 256-byte Apple 1 monitor, in 6502 machine code.
//!
//! # Why this exists
//!
//! The Apple 1's own monitor is Steve Wozniak's, and its copyright status is
//! not clear: it has been passed around freely for decades, which is not a
//! licence. rsemu therefore treats it exactly like `nestest` and blargg's
//! ROMs — **fetch-only, never vendored** (`docs/testing/conformance-suites.md`)
//! — and ships this instead, so that `rsemu run apple1` demonstrates itself
//! with nothing to argue about. Point `--rom` at a Woz Monitor image and you
//! get the real thing; give no `--rom` and you get this.
//!
//! Written for rsemu, © Karpelès Lab Inc., MIT like the rest of the crate. It
//! is not a port, a translation or an abridgement of anything: the parts that
//! *do* match the Apple 1's own monitor are the ones the hardware dictates —
//! the register addresses, and the shape of a poll loop on a status bit that
//! only has one shape.
//!
//! # What it does
//!
//! A hexadecimal examine/deposit monitor, one keystroke at a time, echoing as
//! it goes:
//!
//! ```text
//! RSMON
//! >FF00                          four hex digits and Return
//! FF00: D8 A2 FF 9A A9 7F 8D 12
//! >                              Return again walks on eight bytes
//! FF08: D0 A9 A7 8D 11 D0 8D 13
//! >0300:AA                       an address, a colon, a byte, Return
//! >0300                          and it is there
//! 0300: AA 00 00 00 00 00 00 00
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
//! `$03` the mode. Nothing else in RAM is touched, so the monitor can examine
//! and deposit anywhere above `$04`.
//!
//! # The listing
//!
//! Assembled at `$FF00`, and exactly 250 bytes long, so the 6502's vectors sit
//! immediately after it at `$FFFA` with nothing left over. It is reproduced
//! here in full because [`RSMON`] is otherwise unreadable, and because a test
//! below walks the same bytes with the crate's own 6502 disassembler and
//! checks that the instruction boundaries and the register accesses are the
//! ones written here.
//!
//! ```text
//! KBD    = $D010          PIA port A: the keyboard, bit 7 always set
//! KBDCR  = $D011          PIA control A: bit 7 set when a key is waiting
//! DSP    = $D012          PIA port B: the display, bit 7 set while busy
//! DSPCR  = $D013          PIA control B
//!
//! FF00  D8         RESET:  CLD
//! FF01  A2 FF              LDX #$FF
//! FF03  9A                 TXS
//! FF04  A9 7F              LDA #$7F        ; DDRB: PB0-PB6 out, PB7 in
//! FF06  8D 12 D0           STA DSP
//! FF09  A9 A7              LDA #$A7        ; CRA/CRB: select the data
//! FF0B  8D 11 D0           STA KBDCR       ;   registers, CA1/CB1 on a
//! FF0E  8D 13 D0           STA DSPCR       ;   rising edge
//! FF11  A2 00              LDX #$00
//! FF13  8A                 TXA
//! FF14  85 00              STA $00         ; address low
//! FF16  85 01              STA $01         ; address high
//! FF18  85 03              STA $03         ; mode: 0 examine, 1 deposit
//! FF1A  BD F4 FF   BANNER: LDA MSG,X
//! FF1D  F0 06              BEQ NEWCMD
//! FF1F  20 C1 FF           JSR PUTC
//! FF22  E8                 INX
//! FF23  D0 F5              BNE BANNER
//! FF25  20 CC FF   NEWCMD: JSR CRLF
//! FF28  A9 3E      PROMPT: LDA #'>'
//! FF2A  20 C1 FF           JSR PUTC
//! FF2D  20 D1 FF   MAIN:   JSR GETC        ; blocks, and echoes
//! FF30  29 7F              AND #$7F
//! FF32  C9 0D              CMP #$0D
//! FF34  F0 35              BEQ ENTER
//! FF36  C9 3A              CMP #':'
//! FF38  F0 27              BEQ SETDEP
//! FF3A  20 DC FF           JSR HEXVAL      ; C=1 and A=nibble if hex
//! FF3D  90 EE              BCC MAIN
//! FF3F  A6 03              LDX $03
//! FF41  D0 10              BNE DIGDAT
//! FF43  A2 04              LDX #$04        ; shift the nibble into the
//! FF45  06 00      SHLA:   ASL $00         ;   16-bit address
//! FF47  26 01              ROL $01
//! FF49  CA                 DEX
//! FF4A  D0 F9              BNE SHLA
//! FF4C  05 00              ORA $00
//! FF4E  85 00              STA $00
//! FF50  4C 2D FF           JMP MAIN
//! FF53  A2 04      DIGDAT: LDX #$04        ; or into the byte
//! FF55  06 02      SHLD:   ASL $02
//! FF57  CA                 DEX
//! FF58  D0 FB              BNE SHLD
//! FF5A  05 02              ORA $02
//! FF5C  85 02              STA $02
//! FF5E  4C 2D FF           JMP MAIN
//! FF61  A9 00      SETDEP: LDA #$00
//! FF63  85 02              STA $02
//! FF65  A9 01              LDA #$01
//! FF67  85 03              STA $03
//! FF69  D0 C2              BNE MAIN        ; always: A is 1
//! FF6B  A5 03      ENTER:  LDA $03
//! FF6D  F0 11              BEQ DUMP
//! FF6F  A5 02              LDA $02         ; deposit and advance
//! FF71  A0 00              LDY #$00
//! FF73  91 00              STA ($00),Y
//! FF75  84 03              STY $03
//! FF77  E6 00              INC $00
//! FF79  D0 02              BNE ENT1
//! FF7B  E6 01              INC $01
//! FF7D  4C 28 FF   ENT1:   JMP PROMPT   ; the echoed Return ended the line
//! FF80  A5 01      DUMP:   LDA $01         ; "AAAA:" then eight bytes
//! FF82  20 AE FF           JSR PRBYTE
//! FF85  A5 00              LDA $00
//! FF87  20 AE FF           JSR PRBYTE
//! FF8A  A9 3A              LDA #':'
//! FF8C  20 C1 FF           JSR PUTC
//! FF8F  A0 00              LDY #$00
//! FF91  A9 20      DUMPL:  LDA #' '
//! FF93  20 C1 FF           JSR PUTC
//! FF96  B1 00              LDA ($00),Y
//! FF98  20 AE FF           JSR PRBYTE
//! FF9B  C8                 INY
//! FF9C  C0 08              CPY #$08
//! FF9E  D0 F1              BNE DUMPL
//! FFA0  A5 00              LDA $00         ; address += 8
//! FFA2  18                 CLC
//! FFA3  69 08              ADC #$08
//! FFA5  85 00              STA $00
//! FFA7  90 02              BCC DMP1
//! FFA9  E6 01              INC $01
//! FFAB  4C 25 FF   DMP1:   JMP NEWCMD
//! FFAE  48         PRBYTE: PHA
//! FFAF  4A                 LSR
//! FFB0  4A                 LSR
//! FFB1  4A                 LSR
//! FFB2  4A                 LSR
//! FFB3  20 B7 FF           JSR PRHEX
//! FFB6  68                 PLA
//! FFB7  29 0F      PRHEX:  AND #$0F
//! FFB9  09 30              ORA #$30
//! FFBB  C9 3A              CMP #$3A
//! FFBD  90 02              BCC PUTC
//! FFBF  69 06              ADC #$06        ; C=1 here, so +7: '9'+1 -> 'A'
//! FFC1  48         PUTC:   PHA
//! FFC2  2C 12 D0   PUTC1:  BIT DSP         ; the display's bit 7 is DA
//! FFC5  30 FB              BMI PUTC1
//! FFC7  68                 PLA
//! FFC8  8D 12 D0           STA DSP
//! FFCB  60                 RTS
//! FFCC  A9 0D      CRLF:   LDA #$0D
//! FFCE  4C C1 FF           JMP PUTC
//! FFD1  AD 11 D0   GETC:   LDA KBDCR
//! FFD4  10 FB              BPL GETC
//! FFD6  AD 10 D0           LDA KBD
//! FFD9  4C C1 FF           JMP PUTC        ; tail call: echoes and returns A
//! FFDC  C9 30      HEXVAL: CMP #'0'
//! FFDE  90 12              BCC HVBAD
//! FFE0  C9 3A              CMP #'9'+1
//! FFE2  90 0A              BCC HVDIG
//! FFE4  C9 41              CMP #'A'
//! FFE6  90 0A              BCC HVBAD
//! FFE8  C9 47              CMP #'F'+1
//! FFEA  B0 06              BCS HVBAD
//! FFEC  E9 06              SBC #$06        ; C=0 here, so -7
//! FFEE  29 0F      HVDIG:  AND #$0F
//! FFF0  38                 SEC
//! FFF1  60                 RTS
//! FFF2  18         HVBAD:  CLC
//! FFF3  60                 RTS
//! FFF4  52 53 4D   MSG:    .byte "RSMON", 0
//!       4F 4E 00
//! FFFA  00 FF              .word RESET     ; NMI
//! FFFC  00 FF              .word RESET     ; RESET
//! FFFE  00 FF              .word RESET     ; IRQ
//! ```

/// Where [`RSMON`] is assembled, and where the Apple 1 decodes its monitor ROM.
pub const RSMON_BASE: u64 = 0xff00;

/// RSMON, ready to bind to the `rom` media slot of `machines/apple1.machine`.
///
/// 256 bytes: 250 of code and message at `$FF00`, then the 6502's NMI, RESET
/// and IRQ vectors, all three pointing at `$FF00`. See the module docs for the
/// listing these bytes came from.
pub static RSMON: &[u8; 256] = &[
    0xd8, 0xa2, 0xff, 0x9a, 0xa9, 0x7f, 0x8d, 0x12, 0xd0, 0xa9, 0xa7, 0x8d, 0x11, 0xd0, 0x8d, 0x13,
    0xd0, 0xa2, 0x00, 0x8a, 0x85, 0x00, 0x85, 0x01, 0x85, 0x03, 0xbd, 0xf4, 0xff, 0xf0, 0x06, 0x20,
    0xc1, 0xff, 0xe8, 0xd0, 0xf5, 0x20, 0xcc, 0xff, 0xa9, 0x3e, 0x20, 0xc1, 0xff, 0x20, 0xd1, 0xff,
    0x29, 0x7f, 0xc9, 0x0d, 0xf0, 0x35, 0xc9, 0x3a, 0xf0, 0x27, 0x20, 0xdc, 0xff, 0x90, 0xee, 0xa6,
    0x03, 0xd0, 0x10, 0xa2, 0x04, 0x06, 0x00, 0x26, 0x01, 0xca, 0xd0, 0xf9, 0x05, 0x00, 0x85, 0x00,
    0x4c, 0x2d, 0xff, 0xa2, 0x04, 0x06, 0x02, 0xca, 0xd0, 0xfb, 0x05, 0x02, 0x85, 0x02, 0x4c, 0x2d,
    0xff, 0xa9, 0x00, 0x85, 0x02, 0xa9, 0x01, 0x85, 0x03, 0xd0, 0xc2, 0xa5, 0x03, 0xf0, 0x11, 0xa5,
    0x02, 0xa0, 0x00, 0x91, 0x00, 0x84, 0x03, 0xe6, 0x00, 0xd0, 0x02, 0xe6, 0x01, 0x4c, 0x28, 0xff,
    0xa5, 0x01, 0x20, 0xae, 0xff, 0xa5, 0x00, 0x20, 0xae, 0xff, 0xa9, 0x3a, 0x20, 0xc1, 0xff, 0xa0,
    0x00, 0xa9, 0x20, 0x20, 0xc1, 0xff, 0xb1, 0x00, 0x20, 0xae, 0xff, 0xc8, 0xc0, 0x08, 0xd0, 0xf1,
    0xa5, 0x00, 0x18, 0x69, 0x08, 0x85, 0x00, 0x90, 0x02, 0xe6, 0x01, 0x4c, 0x25, 0xff, 0x48, 0x4a,
    0x4a, 0x4a, 0x4a, 0x20, 0xb7, 0xff, 0x68, 0x29, 0x0f, 0x09, 0x30, 0xc9, 0x3a, 0x90, 0x02, 0x69,
    0x06, 0x48, 0x2c, 0x12, 0xd0, 0x30, 0xfb, 0x68, 0x8d, 0x12, 0xd0, 0x60, 0xa9, 0x0d, 0x4c, 0xc1,
    0xff, 0xad, 0x11, 0xd0, 0x10, 0xfb, 0xad, 0x10, 0xd0, 0x4c, 0xc1, 0xff, 0xc9, 0x30, 0x90, 0x12,
    0xc9, 0x3a, 0x90, 0x0a, 0xc9, 0x41, 0x90, 0x0a, 0xc9, 0x47, 0xb0, 0x06, 0xe9, 0x06, 0x29, 0x0f,
    0x38, 0x60, 0x18, 0x60, 0x52, 0x53, 0x4d, 0x4f, 0x4e, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff,
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_vectors_all_point_at_the_entry_point() {
        // The reset vector is what the machine actually follows, and $FFFC is
        // inside this ROM. Get it wrong and nothing runs at all.
        for vector in [0xfa, 0xfc, 0xfe] {
            let target = u16::from(RSMON[vector]) | (u16::from(RSMON[vector + 1]) << 8);
            assert_eq!(u64::from(target), RSMON_BASE, "vector at ${vector:02x}");
        }
    }

    #[test]
    fn the_code_stops_short_of_the_vectors() {
        // 250 bytes of code and message, then $FFFA. If the listing ever grows
        // past that it would overwrite the reset vector, which is a failure
        // mode worth naming rather than debugging.
        assert_eq!(RSMON[0xf3], 0x60, "HVBAD's RTS is the last opcode");
        assert_eq!(&RSMON[0xf4..0xfa], b"RSMON\0");
    }

    /// The listing decodes as one unbroken run of instructions, and reaches
    /// for nothing but the four registers the machine file maps.
    ///
    /// Uses the crate's own disassembler — generated from the same table the
    /// interpreter decodes with (`CLAUDE.md`, "CPU cores") — so this is a real
    /// decode rather than a scan for byte pairs that look like addresses. A
    /// listing whose instruction boundaries had drifted would land somewhere
    /// other than `$FFF4`, and a typo'd `$D014` would be a register nothing
    /// answers and a guest that wedges on it.
    #[cfg(feature = "cpu-mos6502")]
    #[test]
    fn every_instruction_decodes_and_only_the_pia_is_touched() {
        use crate::cpu::mos6502::disasm::disassemble_run;

        let run = disassemble_run(RSMON_BASE as u16, 256, |addr| {
            RSMON
                .get(usize::from(addr.wrapping_sub(RSMON_BASE as u16)))
                .copied()
        });

        let mut registers = alloc::vec::Vec::new();
        let mut end = RSMON_BASE as u16;
        for d in &run {
            assert!(!d.truncated, "truncated decode at ${:04x}", d.pc);
            if d.pc >= 0xfff4 {
                break;
            }
            end = d.pc.wrapping_add(u16::from(d.len));
            if d.len == 3 {
                let target = d.word();
                if (0xd000..0xe000).contains(&target) && !registers.contains(&target) {
                    registers.push(target);
                }
            }
        }
        assert_eq!(end, 0xfff4, "the code does not end where MSG begins");
        registers.sort_unstable();
        assert_eq!(registers, [0xd010, 0xd011, 0xd012, 0xd013]);
    }
}
