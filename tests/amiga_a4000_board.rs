//! The `amiga-a4000` board, end to end, with a ROM built in this file.
//!
//! The one thing that proves the A4000's IDE port on the *board* rather than in
//! a rig is a 68040 program doing what `scsi.device` does: point the level 2
//! autovector at a handler, enable Paula's `PORTS`, clear the drive's `nIEN`,
//! send `IDENTIFY DEVICE`, wait for the interrupt, copy the block out of the
//! data register; then the same for `READ SECTORS`. The interrupt has to come
//! the whole way — the drive's `INTRQ`, the port's interrupt register at
//! `$00DD3020`, the `INT2` net, Paula's `INTREQ`, the processor's autovector —
//! or the program waits forever and the test says so.
//!
//! The second thing this file is for is the **map**: what an A4000 has that an
//! A1200 has not (a 32-bit space, 16 MiB of motherboard fast RAM behind Ramsey,
//! the battery-backed clock) and what it has not that an A1200 has (Gayle, its
//! identification register, and the Kickstart mirrors Gayle's ROM select makes).
//!
//! No Kickstart, no disk image from anywhere: the ROM is hand-assembled from
//! the MC68040 User's Manual's instruction formats — every instruction in it is
//! also an MC68000 one — and the disk is bytes made here.
//! `tests/amiga_a4000.rs` is where real ones run.

#![cfg(feature = "machine-amiga-a4000")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

const ROM: u64 = 0x00F8_0000;

/// Where the program leaves its results in chip RAM.
const COUNT: u64 = 0x400; // interrupts taken
const DONE: u64 = 0x402; // $600D once both blocks are copied
const SEEN: u64 = 0x404; // every interrupt-register value the handler saw, ANDed
const IDENTIFY_AT: u64 = 0x1000;
const SECTOR_AT: u64 = 0x2000;

/// The handler's offset in the ROM.
const HANDLER: usize = 0x200;

/// The A4000's task file: register *n*'s eight-bit half at `$00DD2022 + 4n`.
const fn cmd(n: u32) -> u32 {
    0x00DD_2022 + 4 * n
}

/// The sixteen-bit data register, which is the word at the bottom of slot 0.
const DATA: u32 = 0x00DD_2020;
/// Device Control on a write, Alternate Status on a read.
const DEVCTL: u32 = 0x00DD_3022 + 4 * 6;
/// The port's interrupt register, read as a word: bit 15 is `INTRQ`.
const INTREG: u32 = 0x00DD_3020;

/// One `move.b #imm,(xxx).L`.
fn poke(words: &mut Vec<u16>, value: u8, at: u32) {
    words.extend_from_slice(&[0x13FC, u16::from(value), (at >> 16) as u16, at as u16]);
}

/// A 512 KiB ROM: the reset longwords, the program at `$00F80010` and the
/// level 2 handler at `$00F80200`.
///
/// ```text
///   ; --- the program -------------------------------------------------------
///   ; It runs from the ROM's own window, because the first CIA write takes the
///   ; overlay — and the ROM at zero with it — away.
///   move.b #0,$00BFE001        ; CIA-A PRA: PA0 low …
///   move.b #1,$00BFE201        ; … and DDRA makes it an output: OVL drops
///   clr.w  $400.w              ; interrupts taken
///   clr.w  $402.w              ; the done flag
///   move.b #$FF,$404.w         ; every interrupt register seen, ANDed
///   move.l #$00F80200,$68.w    ; the level 2 autovector
///   move.w #$C008,$00DFF09A    ; INTENA: SET | INTEN | PORTS
///   move   #$2000,sr           ; and let them in
///   move.b #0,$00DD303A        ; Device Control: nIEN clear — INTRQ enabled
///   move.b #$A0,$00DD203A      ; Device: device 0
///   move.b #$EC,$00DD203E      ; IDENTIFY DEVICE
///   tst.w  $400.w         w1:
///   beq.s  w1
///   lea    $1000.w,a1
///   move.w #255,d0
///   move.w $00DD2020,(a1)+  l1:
///   dbra   d0,l1
///   move.b #$E0,$00DD203A      ; Device: LBA, device 0
///   move.b #1,$00DD202A        ; one sector
///   move.b #0,$00DD202E        ; LBA 7..0
///   move.b #0,$00DD2032        ; LBA 15..8
///   move.b #0,$00DD2036        ; LBA 23..16
///   move.b #$20,$00DD203E      ; READ SECTORS
///   cmpi.w #2,$400.w      w2:
///   blt.s  w2
///   lea    $2000.w,a1
///   move.w #255,d0
///   move.w $00DD2020,(a1)+  l2:
///   dbra   d0,l2
///   move.w #$600D,$402.w
///   bra.s  *
///
///   ; --- the level 2 handler ----------------------------------------------
///   move.w $00DD3020,d1        ; the port's interrupt register
///   move.b $00DD203E,d2        ; the drive's Status: reading it releases INTRQ
///   lsr.w  #8,d1               ; bit 15 down to bit 7
///   and.b  d1,$404.w
///   move.w #$0008,$00DFF09C    ; INTREQ: let go of PORTS
///   addq.w #1,$400.w
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
    main.extend_from_slice(&[0x21FC, 0x00F8, 0x0200, 0x0068]);
    main.extend_from_slice(&[0x33FC, 0xC008, 0x00DF, 0xF09A]);
    main.extend_from_slice(&[0x46FC, 0x2000]);

    poke(&mut main, 0x00, DEVCTL);
    poke(&mut main, 0xA0, cmd(6));
    poke(&mut main, 0xEC, cmd(7));
    main.extend_from_slice(&[0x4A78, COUNT as u16, 0x67FA]);
    main.extend_from_slice(&[0x43F8, IDENTIFY_AT as u16, 0x303C, 0x00FF]);
    main.extend_from_slice(&[0x32F9, (DATA >> 16) as u16, DATA as u16, 0x51C8, 0xFFF8]);

    poke(&mut main, 0xE0, cmd(6));
    poke(&mut main, 0x01, cmd(2));
    poke(&mut main, 0x00, cmd(3));
    poke(&mut main, 0x00, cmd(4));
    poke(&mut main, 0x00, cmd(5));
    poke(&mut main, 0x20, cmd(7));
    main.extend_from_slice(&[0x0C78, 0x0002, COUNT as u16, 0x6DF8]);
    main.extend_from_slice(&[0x43F8, SECTOR_AT as u16, 0x303C, 0x00FF]);
    main.extend_from_slice(&[0x32F9, (DATA >> 16) as u16, DATA as u16, 0x51C8, 0xFFF8]);
    main.extend_from_slice(&[0x31FC, 0x600D, DONE as u16]);
    main.push(0x60FE);

    let handler: &[u16] = &[
        0x3239,
        (INTREG >> 16) as u16,
        INTREG as u16, // move.w $00DD3020,d1
        0x1439,
        (cmd(7) >> 16) as u16,
        cmd(7) as u16, // move.b Status,d2
        0xE049,        // lsr.w #8,d1
        0xC338,
        SEEN as u16, // and.b d1,$404.w
        0x33FC,
        0x0008,
        0x00DF,
        0xF09C, // INTREQ: clear PORTS
        0x5278,
        COUNT as u16, // addq.w #1,$400.w
        0x4E73,       // rte
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
    image
}

/// 128 sectors whose first four bytes are `RDSK` and whose every other byte
/// says where it is: an HDF's shape, and no one's content.
fn disk() -> Vec<u8> {
    let mut image = vec![0u8; 128 * 512];
    for (i, byte) in image.iter_mut().enumerate() {
        *byte = (i as u8).wrapping_mul(7) ^ (i >> 8) as u8;
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
    let source = catalog::machine("amiga-a4000")
        .expect("this build ships amiga-a4000")
        .source;
    rsemu::machine::build("amiga-a4000", source, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"))
}

fn read(m: &Machine, addr: u64, width: Width) -> u64 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, width, MemAttrs::DEBUG)
        .expect("a mapped address")
}

fn bytes(m: &Machine, addr: u64, len: u64) -> Vec<u8> {
    (0..len)
        .map(|i| read(m, addr + i, Width::U8) as u8)
        .collect()
}

/// A read that says what it found on a floating bus: `$5A` is nobody's
/// register here.
fn floats(m: &Machine, addr: u64) -> bool {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEBUG.with_bus(0x5A))
        == Ok(0x5A)
}

#[test]
fn the_board_realizes_with_a_68040_and_the_a4000s_own_ide_port() {
    let m = build(disk());
    assert_eq!(m.name(), "amiga-a4000");
    for path in [
        "cpu", "chipram", "fastram", "kick", "custom", "paula", "agnus", "denise", "cia_a",
        "cia_b", "hd0", "ide", "ramsey", "rtc", "overlay", "df0", "kbd", "mouse",
    ] {
        assert!(m.device(path).is_some(), "no instance called `{path}`");
    }
    // There is no Gayle on this board, which is why `amiga.ide` exists.
    assert!(m.device("gayle").is_none());
}

#[test]
fn the_map_is_the_a4000s() {
    let mut m = build(Vec::new());

    // Out of reset the ROM is at zero as well as at $00F80000.
    assert_eq!(read(&m, 0, Width::U16), 0x0020);
    assert_eq!(read(&m, ROM + 4, Width::U16), 0x00F8);
    // And *only* at $00F80000: an A4000 has no Gayle, so none of Gayle's ROM
    // select mirrors is here. `machines/amiga-a1200.machine` maps all three.
    for mirror in [0x00E0_0000u64, 0x00A8_0000, 0x00B0_0000] {
        assert!(floats(&m, mirror + 4), "{mirror:#x} is not a ROM mirror");
    }
    // Nor Gayle's identification register at $00DE1000, nor its four
    // registers at $00DA8000, nor its IDE window at $00DA0000.
    for gayle in [0x00DE_1000u64, 0x00DA_8000, 0x00DA_2000, 0x00DA_201C] {
        assert!(
            floats(&m, gayle),
            "{gayle:#x} is Gayle's, and there is none"
        );
    }

    // Let the program run long enough to write CIA-A's PA0 and take the
    // overlay down; chip RAM is behind it until then.
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("it runs");
    let space = m.space("mem").expect("the memory space");
    assert_eq!(read(&m, 0, Width::U16), 0, "the ROM is no longer at zero");

    // 2 MiB of chip RAM, and nothing above it.
    space
        .write(0x001F_FFFC, Width::U32, 0xDEAD_BEEF, MemAttrs::DEFAULT)
        .expect("the top of chip RAM");
    assert_eq!(read(&m, 0x001F_FFFC, Width::U32), 0xDEAD_BEEF);
    assert!(floats(&m, 0x0020_0000));

    // Ramsey: the enhanced part, which is what an A4000 has (version $0F,
    // *The A3000+ System Specification* §2.2.2).
    assert_eq!(read(&m, 0x00DE_0043, Width::U8), 0x0F);
    // And its control register reads back what Kickstart's memory sizing
    // writes, which is the whole reason the chip is on the board at all.
    space
        .write(0x00DE_0003, Width::U8, 0x07, MemAttrs::DEFAULT)
        .expect("Ramsey's control register");
    assert_eq!(read(&m, 0x00DE_0003, Width::U8) & 0x07, 0x07);

    // 16 MiB of motherboard fast RAM at $07000000, and nothing above it.
    space
        .write(0x07FF_FFFC, Width::U32, 0x1234_5678, MemAttrs::DEFAULT)
        .expect("the top of the fast RAM");
    assert_eq!(read(&m, 0x07FF_FFFC, Width::U32), 0x1234_5678);
    assert_eq!(read(&m, 0x0700_0000, Width::U32), 0);
    assert!(floats(&m, 0x0800_0000), "the coprocessor slot is empty");
    // Zorro II autoconfig space and Zorro III space float: no cards.
    assert!(floats(&m, 0x00E8_0000));
    assert!(floats(&m, 0x4000_0000));

    // The battery-backed clock answers at $00DC0000, unlike an A1200's.
    assert!(!floats(&m, 0x00DC_0003), "the clock is on this board");

    // With no disk bound the IDE bay is empty and the command block floats.
    assert!(floats(&m, u64::from(cmd(7))));
    // As does the interrupt register's `INTRQ` bit, because nothing drives it.
    assert_eq!(read(&m, u64::from(INTREG), Width::U16) & 0x8000, 0);
}

#[test]
fn a_68040_program_reads_identify_and_a_sector_through_the_port_on_its_interrupt() {
    let image = disk();
    let mut m = build(image.clone());
    // Fifty milliseconds is more than a million processor cycles; the program
    // needs a few thousand, most of them the two 256-word copies.
    m.run_for(GlobalTime::from_nanos(50_000_000))
        .expect("it runs");

    assert_eq!(
        read(&m, DONE, Width::U16),
        0x600D,
        "the program never finished: {} interrupt(s) taken",
        read(&m, COUNT, Width::U16)
    );
    // Both waits ended on an interrupt, and every interrupt the handler took
    // had bit 15 of `$00DD3020` set: it came through this port, not from
    // anywhere else on INT2. Exactly two, one per command: both are PIO
    // data-in commands of a single DRQ block, and T13 ATA/ATAPI-6 §9.5 has the
    // drive interrupt when that block is ready and *not* when the host has
    // emptied it.
    assert_eq!(read(&m, COUNT, Width::U16), 2);
    assert_eq!(read(&m, SEEN, Width::U8) & 0x80, 0x80);
    // INTREQ's PORTS is clear again: the handler cleared it, and reading
    // Status let the port go.
    assert_eq!(read(&m, 0x00DF_F01E, Width::U16) & 0x0008, 0);

    // IDENTIFY, as the processor stored it: byte-swapped, so the model string
    // reads pairwise reversed at word 27.
    let identify = bytes(&m, IDENTIFY_AT, 512);
    assert_eq!(
        &identify[54..60],
        b"SRME U",
        "\"RSEMU \" a pair at a time, swapped"
    );
    // Word 60-61, the LBA capacity, low word first: 128 sectors.
    assert_eq!(&identify[120..124], &[128, 0, 0, 0]);

    // And sector 0, byte for byte as the image holds it — which is the byte
    // swap being right, because it is what makes `RDSK` come back as `RDSK`.
    assert_eq!(bytes(&m, SECTOR_AT, 512), &image[..512]);
    assert_eq!(&bytes(&m, SECTOR_AT, 4), b"RDSK");
}

#[test]
fn the_board_names_exactly_the_media_slots_it_documents() {
    let source = catalog::machine("amiga-a4000")
        .expect("this build ships amiga-a4000")
        .source;
    let mut slots: Vec<&str> = source
        .match_indices("image = \"")
        .map(|(at, _)| {
            let rest = &source[at + 9..];
            &rest[..rest.find('"').expect("a closing quote")]
        })
        .collect();
    slots.sort_unstable();
    slots.dedup();
    assert_eq!(slots, ["df0", "hd0", "kickstart"]);
    assert_eq!(
        catalog::machine("amiga-a4000").expect("an entry").media,
        ["kickstart", "hd0", "df0"]
    );
}
