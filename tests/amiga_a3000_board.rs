//! The `amiga-a3000` board, end to end, with a ROM built in this file.
//!
//! The one thing that proves the SCSI port on the *board* rather than in a rig
//! is a 68030 program doing what `scsi.device` does: point the level 2
//! autovector at a handler, enable the DMA controller's interrupt and Paula's
//! `PORTS`, reset the WD33C93A, load the command block with a `READ(10)`, and
//! run it with `Select-With-ATN-And-Transfer` straight into chip RAM. The
//! interrupt has to come the whole way — the controller's `INTRQ`, the DMAC's
//! `ISTR` gated by `INTENA`, the `INT2` net, Paula's `INTREQ`, the processor's
//! autovector — or the program waits forever and the test says so. And the
//! blocks have to arrive by bus mastering, at the address the Address Control
//! Register named.
//!
//! No Kickstart and no disk image from anywhere: the ROM is hand-assembled
//! from the MC68000 User's Manual's instruction formats, and the disk is bytes
//! made here. `tests/amiga_a3000.rs` is where real ones run.

#![cfg(feature = "machine-amiga-a3000")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// Where the program leaves its results in chip RAM.
const COUNT: u64 = 0x400; // interrupts taken
const DONE: u64 = 0x402; // $600D once the transfer completed
const SEEN: u64 = 0x404; // the last SCSI Status byte the handler read
const BUFFER: u64 = 0x1000; // where the DMA transfer lands

/// The handler's offset in the ROM.
const HANDLER: usize = 0x300;

/// How many 512-byte blocks the test disk holds.
const BLOCKS: usize = 64;

/// The block the program reads.
const LBA: u8 = 5;

/// `Select-With-ATN-And-Transfer` completed (datasheet §6.2.19, `0001 0110`).
const SAT_DONE: u8 = 0x16;

/// One `move.b #imm,(xxx).L` — the whole of this program's register access.
fn poke(words: &mut Vec<u16>, value: u8, at: u32) {
    words.extend_from_slice(&[0x13FC, u16::from(value), (at >> 16) as u16, at as u16]);
}

/// One `move.l #imm,(xxx).L`, for the DMAC's longword registers.
fn poke_long(words: &mut Vec<u16>, value: u32, at: u32) {
    words.extend_from_slice(&[
        0x23FC,
        (value >> 16) as u16,
        value as u16,
        (at >> 16) as u16,
        at as u16,
    ]);
}

/// `SASR := reg`, then `SCMD := value` — the two byte lanes of §2.4.1's
/// Table 2-5, at the addresses Kickstart itself uses.
fn wd(words: &mut Vec<u16>, reg: u8, value: u8) {
    poke(words, reg, 0x00DD_0049);
    poke(words, value, 0x00DD_0043);
}

/// A byte into the register the address register already names, which the
/// chip's auto-increment has moved on (§6.2.2).
fn wd_next(words: &mut Vec<u16>, value: u8) {
    poke(words, value, 0x00DD_0043);
}

/// A 512 KiB ROM: the reset longwords, the program at `$F80010` and the level
/// 2 handler at `$F80300`.
///
/// ```text
///   ; --- the program -------------------------------------------------------
///   ; It runs from the ROM's own window, because the first CIA write takes the
///   ; overlay — and the ROM at zero with it — away.
///   move.b #0,$00BFE001        ; CIA-A PRA: PA0 low …
///   move.b #1,$00BFE201        ; … and DDRA makes it an output: OVL drops
///   clr.w  $400.w              ; interrupts taken
///   clr.w  $402.w              ; the done flag
///   move.b #$FF,$404.w         ; the last status seen
///   move.l #$00F80300,$68.w    ; the level 2 autovector
///   move.w #$C008,$00DFF09A    ; INTENA: SET | INTEN | PORTS
///   move   #$2000,sr           ; and let them in
///
///   move.l #0,$00DD003C        ; SP_DMA
///   move.l #0,$00DD0018        ; CLR_INT
///   move.l #4,$00DD0008        ; CONTR: INTENA — and `MR-` left one waiting
///   cmpi.w #1,$400.w           ; so let the power-on reset interrupt in first
///   blt.s  *-2
///   ; the WD33C93A, through SASR at $00DD0049 and SCMD at $00DD0043
///   ; Own ID := $47 (a 12-15 MHz clock, divisor 3, this initiator is 7)
///   ; Command := $00 (Reset)
///   cmpi.w #2,$400.w           ; wait for its interrupt
///   blt.s  *-2
///
///   move.l #$1000,$00DD000C    ; ACR: where the blocks go
///   move.l #4,$00DD0008        ; CONTR: INTENA, DMADIR low — SCSI to memory
///   move.l #0,$00DD0010        ; ST_DMA
///   ; Control := $80 (DMA Mode), Target LUN := 0, Destination ID := 0,
///   ; Source ID := $80 (Enable Reselection), Transfer Count := 512,
///   ; CDB := READ(10) of one block at LBA 5, Command := $08
///   cmpi.w #3,$400.w           ; wait for the command to finish
///   blt.s  *-2
///   bra.s  *
///
///   ; --- the level 2 handler ----------------------------------------------
///   movem.l d0-d1,-(sp)
///   move.l #0,$00DD0018        ; CLR_INT, as the DMAC's own driver does
///   move.b #$17,$00DD0049      ; SASR := SCSI Status
///   move.b $00DD0043,d0        ; reading it is what clears INTRQ
///   move.b d0,$404.w
///   cmpi.b #$16,d0             ; Select-And-Transfer completed?
///   bne.s  .not
///   move.w #$600D,$402.w
///   .not:
///   addq.w #1,$400.w
///   move.w #$0008,$00DFF09C    ; INTREQ: let go of PORTS
///   movem.l (sp)+,d0-d1
///   rte
/// ```
fn kickstart() -> Vec<u8> {
    let mut main: Vec<u16> = Vec::new();
    // The overlay: the level first, so the pin never glitches high.
    poke(&mut main, 0x00, 0x00BF_E001);
    poke(&mut main, 0x01, 0x00BF_E201);
    main.extend_from_slice(&[0x4278, COUNT as u16]);
    main.extend_from_slice(&[0x4278, DONE as u16]);
    main.extend_from_slice(&[0x11FC, 0x00FF, SEEN as u16]);
    main.extend_from_slice(&[0x21FC, 0x00F8, 0x0300, 0x0068]);
    main.extend_from_slice(&[0x33FC, 0xC008, 0x00DF, 0xF09A]);
    main.extend_from_slice(&[0x46FC, 0x2000]);

    poke_long(&mut main, 0, 0x00DD_003C); // SP_DMA
    poke_long(&mut main, 0, 0x00DD_0018); // CLR_INT
    poke_long(&mut main, 4, 0x00DD_0008); // CONTR := INTENA
    // `MR-` left `INTRQ` asserted (§6.3.1), so enabling the DMAC's interrupt
    // is what lets that one in. Wait for it before issuing a command: §6.2.20
    // says one written while `INT` is set is ignored.
    main.extend_from_slice(&[0x0C78, 0x0001, COUNT as u16, 0x6DF8]);
    wd(&mut main, 0x00, 0x47); // Own ID
    wd(&mut main, 0x18, 0x00); // Command := Reset
    main.extend_from_slice(&[0x0C78, 0x0002, COUNT as u16, 0x6DF8]);

    poke_long(&mut main, BUFFER as u32, 0x00DD_000C); // ACR
    poke_long(&mut main, 4, 0x00DD_0008); // CONTR := INTENA
    poke_long(&mut main, 0, 0x00DD_0010); // ST_DMA
    wd(&mut main, 0x01, 0x80); // Control := DMA Mode
    wd(&mut main, 0x0F, 0x00); // Target LUN
    wd(&mut main, 0x15, 0x00); // Destination ID
    wd(&mut main, 0x16, 0x80); // Source ID: Enable Reselection
    wd(&mut main, 0x12, 0x00); // Transfer Count, most significant …
    wd_next(&mut main, 0x02); // … and the other two: $000200
    wd_next(&mut main, 0x00);
    wd(&mut main, 0x03, 0x28); // CDB: READ(10)
    for byte in [0x00, 0x00, 0x00, 0x00, LBA, 0x00, 0x00, 0x01, 0x00] {
        wd_next(&mut main, byte);
    }
    wd(&mut main, 0x18, 0x08); // Command := Sel w/ATN-And-Transfer
    main.extend_from_slice(&[0x0C78, 0x0003, COUNT as u16, 0x6DF8]);
    main.push(0x60FE);

    let handler: &[u16] = &[
        0x48E7,
        0xC000, //
        0x23FC,
        0x0000,
        0x0000,
        0x00DD,
        0x0018, //
        0x13FC,
        0x0017,
        0x00DD,
        0x0049, //
        0x1039,
        0x00DD,
        0x0043, //
        0x11C0,
        SEEN as u16, //
        0x0C00,
        u16::from(SAT_DONE), //
        0x6606,              //
        0x31FC,
        0x600D,
        DONE as u16, //
        0x5278,
        COUNT as u16, //
        0x33FC,
        0x0008,
        0x00DF,
        0xF09C, //
        0x4CDF,
        0x0003, //
        0x4E73,
    ];

    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0020_0000u32.to_be_bytes()); // SSP: top of 2 MiB
    image[4..8].copy_from_slice(&0x00F8_0010u32.to_be_bytes()); // PC: the ROM's own window
    for (base, code) in [(0x10usize, &main[..]), (HANDLER, handler)] {
        for (i, word) in code.iter().enumerate() {
            let at = base + 2 * i;
            image[at..at + 2].copy_from_slice(&word.to_be_bytes());
        }
    }
    assert!(
        0x10 + 2 * main.len() < HANDLER,
        "the program fits below the handler"
    );
    image
}

/// `BLOCKS` blocks whose every byte says which block it is, so a transfer that
/// lands on the wrong one is visible.
fn disk() -> Vec<u8> {
    let mut image = vec![0u8; BLOCKS * 512];
    for (n, block) in image.chunks_mut(512).enumerate() {
        block.fill(n as u8);
    }
    image[..4].copy_from_slice(b"RDSK");
    image
}

fn build(hd0: Vec<u8>) -> Machine {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("kickstart", kickstart());
    options.realize.media.insert("hd0", hd0);
    options.realize.media.insert("df0", Vec::new());
    let registry = catalog::registry().expect("a registry");
    let source = catalog::machine("amiga-a3000")
        .expect("this build ships amiga-a3000")
        .source;
    rsemu::machine::build("amiga-a3000", source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"))
}

fn peek(m: &Machine, at: u64, width: Width) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(at, width, MemAttrs::DEBUG)
        .expect("chip RAM")
}

/// Run until the program says it is done, or give up.
fn run(m: &mut Machine) {
    for _ in 0..200 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("it runs");
        if peek(m, DONE, Width::U16) == 0x600D {
            return;
        }
    }
}

#[test]
fn the_board_realizes_with_the_scsi_port_in_garys_place() {
    let m = build(disk());
    assert_eq!(m.name(), "amiga-a3000");
    for path in [
        "cpu", "chipram", "kick", "custom", "paula", "agnus", "denise", "rtc", "ramsey", "cia_a",
        "cia_b", "hd0", "wd0", "sdmac", "overlay", "df0", "kbd", "mouse",
    ] {
        assert!(m.device(path).is_some(), "no instance called `{path}`");
    }
}

#[test]
fn the_board_names_exactly_the_media_slots_it_documents() {
    use rsemu::machine::{ResolveOptions, resolve_file};
    let source = catalog::machine("amiga-a3000").expect("shipped").source;
    let resolved =
        resolve_file("amiga-a3000.machine", source, &ResolveOptions::new()).expect("it resolves");
    let mut slots: Vec<String> = resolved
        .objects
        .iter()
        .filter_map(|o| o.props.get("image"))
        .filter_map(|v| v.as_str().map(ToString::to_string))
        .collect();
    slots.sort();
    slots.dedup();
    // The Kickstart socket, the SCSI drive and the internal floppy. No `ext`:
    // this board has no extended-ROM window.
    assert_eq!(slots, ["df0", "hd0", "kickstart"]);
    assert_eq!(
        catalog::machine("amiga-a3000").expect("shipped").media,
        ["kickstart", "hd0", "df0"]
    );
}

#[test]
fn a_program_reads_a_block_off_the_scsi_bus_by_bus_mastering() {
    let mut m = build(disk());
    run(&mut m);
    assert_eq!(
        peek(&m, DONE, Width::U16),
        0x600D,
        "the transfer never completed; the last SCSI Status was {:#04x} after {} interrupt(s)",
        peek(&m, SEEN, Width::U8),
        peek(&m, COUNT, Width::U16),
    );
    assert_eq!(peek(&m, SEEN, Width::U8) as u8, SAT_DONE);
    // Every byte of the block, where the Address Control Register said.
    for i in 0..512 {
        assert_eq!(
            peek(&m, BUFFER + i, Width::U8) as u8,
            LBA,
            "byte {i} of the block"
        );
    }
    // And nothing either side of it.
    assert_eq!(peek(&m, BUFFER - 2, Width::U16), 0);
    assert_eq!(peek(&m, BUFFER + 512, Width::U16), 0);
}

#[test]
fn with_no_disk_the_selection_times_out_and_the_program_says_so() {
    let mut m = build(Vec::new());
    run(&mut m);
    assert_ne!(
        peek(&m, DONE, Width::U16),
        0x600D,
        "there is nothing on the bus to read from"
    );
    // §6.2.19's `0100 0010`: "A timeout occurred during a Select or Reselect
    // command." The interrupt still arrived, which is the point — an empty
    // cable is a report, not a hang.
    assert_eq!(peek(&m, SEEN, Width::U8) as u8, 0x42);
    assert!(peek(&m, COUNT, Width::U16) >= 2);
}
