//! The `snes-ntsc` and `snes-pal` boards, end to end, with no corpus.
//!
//! One hand-assembled LoROM program exercises every seam the board file
//! draws at once:
//!
//! * the 65C816 boots in emulation mode from `$00FFFC` and switches to native;
//! * it writes the S-PPU (`INIDISP`, CGRAM) through the B-bus window at $2100;
//! * a **general-purpose DMA** moves four ROM bytes into WRAM through
//!   `WMDATA` ($2180), so the A-bus/B-bus split and the CPU stall are real;
//! * an **HDMA** table (one plain entry, one three-line repeat, then the
//!   terminator) writes `WMDATA` on lines 0 and 5-7 of every frame;
//! * the **NMI** handler counts frames and stores the automatic joypad read
//!   and APU port 0 (which the clean-room boot ROM has set to `$AA`);
//! * a **V-IRQ** on line 100 counts too.
//!
//! The source is written out beside the bytes so a reader can check each
//! opcode against its line.

#![cfg(feature = "machine-snes")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::dev::snes::input::{buttons, pads};
use rsemu::host::display::snes::capture;
use rsemu::machine::{Machine, catalog};

/// The program at `$8000`, ROM offset 0.
///
/// ```text
/// 8000 78           sei
/// 8001 18           clc
/// 8002 FB           xce             ; native, A/X/Y still 8-bit
/// 8003 A9 8F        lda #$8F
/// 8005 8D 00 21     sta $2100       ; INIDISP: forced blank
/// 8008 9C 21 21     stz $2121       ; CGADD 0
/// 800B A9 1F        lda #$1F
/// 800D 8D 22 21     sta $2122       ; colour 0 = $001F, red
/// 8010 9C 22 21     stz $2122
/// 8013 9C 81 21     stz $2181       ; WMADD = $000100
/// 8016 A9 01        lda #$01
/// 8018 8D 82 21     sta $2182
/// 801B 9C 83 21     stz $2183
/// 801E 9C 00 43     stz $4300       ; DMA0: mode 0, A -> B, increment
/// 8021 A9 80        lda #$80
/// 8023 8D 01 43     sta $4301       ; to $2180
/// 8026 9C 02 43     stz $4302
/// 8029 A9 90        lda #$90
/// 802B 8D 03 43     sta $4303       ; from $00:9000
/// 802E 9C 04 43     stz $4304
/// 8031 A9 04        lda #$04
/// 8033 8D 05 43     sta $4305       ; 4 bytes
/// 8036 9C 06 43     stz $4306
/// 8039 A9 01        lda #$01
/// 803B 8D 0B 42     sta $420B       ; MDMAEN: go
/// 803E 9C 81 21     stz $2181       ; WMADD = $000200, for the HDMA
/// 8041 A9 02        lda #$02
/// 8043 8D 82 21     sta $2182
/// 8046 9C 10 43     stz $4310       ; HDMA1: mode 0, direct, to $2180
/// 8049 A9 80        lda #$80
/// 804B 8D 11 43     sta $4311
/// 804E 9C 12 43     stz $4312
/// 8051 A9 91        lda #$91
/// 8053 8D 13 43     sta $4313       ; table at $00:9100
/// 8056 9C 14 43     stz $4314
/// 8059 A9 02        lda #$02
/// 805B 8D 0C 42     sta $420C       ; HDMAEN: channel 1
/// 805E A9 64        lda #100
/// 8060 8D 09 42     sta $4209       ; VTIME = 100
/// 8063 9C 0A 42     stz $420A
/// 8066 A9 0F        lda #$0F
/// 8068 8D 00 21     sta $2100       ; display on, full brightness
/// 806B A9 A1        lda #$A1
/// 806D 8D 00 42     sta $4200       ; NMI, V-IRQ, auto-joypad
/// 8070 58           cli
/// 8071 CB     idle: wai
/// 8072 80 FD        bra idle
/// 8074 AD 10 42 nmi: lda $4210      ; acknowledge
/// 8077 E6 10        inc $10         ; frames
/// 8079 AD 19 42     lda $4219       ; JOY1H
/// 807C 85 11        sta $11
/// 807E AD 40 21     lda $2140       ; APU port 0
/// 8081 85 12        sta $12
/// 8083 40           rti
/// 8084 AD 11 42 irq: lda $4211      ; acknowledge
/// 8087 E6 13        inc $13         ; V-IRQs
/// 8089 40           rti
/// ```
const PROGRAM: &[u8] = &[
    0x78, 0x18, 0xFB, 0xA9, 0x8F, 0x8D, 0x00, 0x21, 0x9C, 0x21, 0x21, 0xA9, 0x1F, 0x8D, 0x22, 0x21,
    0x9C, 0x22, 0x21, 0x9C, 0x81, 0x21, 0xA9, 0x01, 0x8D, 0x82, 0x21, 0x9C, 0x83, 0x21, 0x9C, 0x00,
    0x43, 0xA9, 0x80, 0x8D, 0x01, 0x43, 0x9C, 0x02, 0x43, 0xA9, 0x90, 0x8D, 0x03, 0x43, 0x9C, 0x04,
    0x43, 0xA9, 0x04, 0x8D, 0x05, 0x43, 0x9C, 0x06, 0x43, 0xA9, 0x01, 0x8D, 0x0B, 0x42, 0x9C, 0x81,
    0x21, 0xA9, 0x02, 0x8D, 0x82, 0x21, 0x9C, 0x10, 0x43, 0xA9, 0x80, 0x8D, 0x11, 0x43, 0x9C, 0x12,
    0x43, 0xA9, 0x91, 0x8D, 0x13, 0x43, 0x9C, 0x14, 0x43, 0xA9, 0x02, 0x8D, 0x0C, 0x42, 0xA9, 0x64,
    0x8D, 0x09, 0x42, 0x9C, 0x0A, 0x42, 0xA9, 0x0F, 0x8D, 0x00, 0x21, 0xA9, 0xA1, 0x8D, 0x00, 0x42,
    0x58, 0xCB, 0x80, 0xFD, 0xAD, 0x10, 0x42, 0xE6, 0x10, 0xAD, 0x19, 0x42, 0x85, 0x11, 0xAD, 0x40,
    0x21, 0x85, 0x12, 0x40, 0xAD, 0x11, 0x42, 0xE6, 0x13, 0x40,
];

/// The DMA's source, at `$9000`.
const DMA_DATA: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
/// The HDMA table, at `$9100`: one line-0 write, four quiet lines, a
/// three-line repeat, the end.
const HDMA_TABLE: [u8; 7] = [0x05, 0xAA, 0x83, 0x11, 0x22, 0x33, 0x00];

fn image() -> Vec<u8> {
    let mut rom = vec![0u8; 0x8000];
    rom[..PROGRAM.len()].copy_from_slice(PROGRAM);
    rom[0x1000..0x1004].copy_from_slice(&DMA_DATA);
    rom[0x1100..0x1107].copy_from_slice(&HDMA_TABLE);
    // A minimal header, so the map guess has something to agree with.
    rom[0x7FC0..0x7FD5].copy_from_slice(b"RSEMU SNES BOARD TEST");
    rom[0x7FD5] = 0x20;
    // Native NMI at $FFEA, native IRQ at $FFEE, emulation reset at $FFFC.
    rom[0x7FEA..0x7FEC].copy_from_slice(&0x8074u16.to_le_bytes());
    rom[0x7FEE..0x7FF0].copy_from_slice(&0x8084u16.to_le_bytes());
    rom[0x7FFC..0x7FFE].copy_from_slice(&0x8000u16.to_le_bytes());
    rom
}

fn boot(board: &str) -> (Machine, rsemu::host::display::snes::SnesScanout) {
    let entry = catalog::machine(board).expect("this build ships the board");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("cart", image());
    capture::install(&mut options).expect("a capture table");
    let pad = pads::open(&options.realize.hosts, "snes-pads").expect("the pad port");
    pad.set(0, buttons::B | buttons::START);
    let registry = catalog::registry().expect("a registry");
    let m = rsemu::machine::build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("{board} does not realize: {e}"));
    let scanout = capture::take(&options.realize.hosts).expect("an S-PPU");
    (m, scanout)
}

fn peek(m: &Machine, addr: u64) -> u8 {
    m.space("abus")
        .expect("the A-bus")
        .read(addr, Width::U8, MemAttrs::DEBUG)
        .unwrap_or_else(|e| panic!("{addr:#x}: {e:?}")) as u8
}

#[test]
fn the_program_runs_and_every_seam_answers() {
    for (board, hz) in [("snes-ntsc", 60), ("snes-pal", 50)] {
        let (mut m, scanout) = boot(board);
        // A tenth of a second: six NTSC frames, five PAL.
        m.run_for(GlobalTime::from_nanos(100_000_000))
            .expect("it runs");
        let frames = peek(&m, 0x10);
        assert!(
            (u8::try_from(hz / 10).unwrap() - 1..=u8::try_from(hz / 10).unwrap() + 1)
                .contains(&frames),
            "{board}: {frames} NMIs in 100 ms"
        );
        let irqs = peek(&m, 0x13);
        assert!(
            irqs.abs_diff(frames) <= 1,
            "{board}: {irqs} V-IRQs, {frames} NMIs"
        );
        assert_eq!(peek(&m, 0x11), 0x90, "{board}: B + Start in JOY1H");
        assert_eq!(
            peek(&m, 0x12),
            0xAA,
            "{board}: the IPL's ready byte on port 0"
        );
        // The GP DMA, through WMDATA.
        let dma: Vec<u8> = (0..4).map(|i| peek(&m, 0x7E_0100 + i)).collect();
        assert_eq!(dma, DMA_DATA, "{board}: GP DMA into WRAM");
        // HDMA: AA 11 22 33 per frame, frame after frame.
        let hdma: Vec<u8> = (0..8).map(|i| peek(&m, 0x7E_0200 + i)).collect();
        assert_eq!(
            hdma,
            [0xAA, 0x11, 0x22, 0x33, 0xAA, 0x11, 0x22, 0x33],
            "{board}: HDMA"
        );
        // A red backdrop, full brightness, in the middle of the picture.
        let pixel = scanout.ppu().pixel(128, 112).expect("a pixel");
        assert_eq!(pixel & 0x7FFF, 0x001F, "{board}: the backdrop is CGRAM 0");
    }
}

#[test]
fn a_running_snes_resumes_from_a_snapshot_exactly() {
    for board in ["snes-ntsc", "snes-pal"] {
        let (mut m, _scanout) = boot(board);
        // Mid-frame, with HDMA, the V-IRQ and NMI all ahead of it.
        m.run_for(GlobalTime::from_nanos(30_000_000)).expect("runs");
        let saved = m.save().expect("saves");
        let hash = m.state_hash().expect("hashes");
        m.run_for(GlobalTime::from_nanos(30_000_000))
            .expect("runs on");
        let straight = m.state_hash().expect("hashes");
        m.load(&saved).expect("loads");
        assert_eq!(m.state_hash().expect("hashes"), hash, "{board}: restored");
        m.run_for(GlobalTime::from_nanos(30_000_000))
            .expect("runs on again");
        assert_eq!(
            m.state_hash().expect("hashes"),
            straight,
            "{board}: a resumed run is the same run"
        );
    }
}
