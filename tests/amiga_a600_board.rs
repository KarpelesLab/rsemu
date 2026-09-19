//! The `amiga-a600` board, end to end, with a ROM built in this file.
//!
//! The one thing that proves Gayle's IDE port on the board rather than in a
//! rig is a 68000 program doing what `scsi.device` does: point the level 2
//! autovector at a handler, enable Gayle's IDE interrupt and Paula's `PORTS`,
//! send `IDENTIFY DEVICE`, wait for the interrupt, copy the block out of the
//! data register; then the same for `READ SECTORS`. The interrupt has to come
//! the whole way — the drive's `INTRQ`, Gayle's change latch and enable, the
//! `INT2` net, Paula's `INTREQ`, the processor's autovector — or the program
//! waits forever and the test says so.
//!
//! No Kickstart, no disk image from anywhere: the ROM is hand-assembled from
//! the MC68000 User's Manual's instruction formats, and the disk is bytes made
//! here. `tests/amiga_a600_hdf.rs` is where real ones run.

#![cfg(feature = "machine-amiga-a600")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

const ROM: u64 = 0xF8_0000;

/// Where the program leaves its results in chip RAM.
const COUNT: u64 = 0x400; // interrupts taken
const DONE: u64 = 0x402; // $600D once both blocks are copied
const SEEN: u64 = 0x404; // every change register value the handler saw, ANDed
const IDENTIFY_AT: u64 = 0x1000;
const SECTOR_AT: u64 = 0x2000;

/// The handler's offset in the ROM.
const HANDLER: usize = 0x200;

/// A 512 KiB ROM: the reset longwords, the program at `$F80010` and the level 2
/// handler at `$F80200`.
///
/// ```text
///   ; the program. It jumps straight to the ROM's own address, because the
///   ; first CIA write takes the overlay down and the ROM with it at zero.
///   13FC 0000 00BF E201   move.b #0,$BFE201        ; CIA-A DDRA: the overlay drops
///   11FC 00FF 0404        move.b #$FF,$404.w
///   4278 0400             clr.w  $400.w
///   21FC 00F8 0200 0068   move.l #$F80200,$68.w    ; level 2 autovector
///   13FC 0080 00DA A000   move.b #$80,$DAA000      ; Gayle: IDE change -> INT2
///   33FC C008 00DF F09A   move.w #$C008,$DFF09A    ; INTENA: SET|INTEN|PORTS
///   46FC 2000             move.w #$2000,sr
///   13FC 00A0 00DA 2018   move.b #$A0,$DA2018      ; Device: device 0
///   13FC 00EC 00DA 201C   move.b #$EC,$DA201C      ; IDENTIFY DEVICE
///   4A78 0400        w1:  tst.w  $400.w
///   67FA                  beq.s  w1
///   43F8 1000             lea    $1000.w,a1
///   303C 00FF             move.w #255,d0
///   32F9 00DA 2000   l1:  move.w $DA2000,(a1)+
///   51C8 FFF8             dbra   d0,l1
///   13FC 00E0 00DA 2018   move.b #$E0,$DA2018      ; Device: LBA, device 0
///   13FC 0001 00DA 2008   move.b #1,$DA2008        ; one sector
///   13FC 0000 00DA 200C   move.b #0,$DA200C        ; LBA 0
///   13FC 0000 00DA 2010   move.b #0,$DA2010
///   13FC 0000 00DA 2014   move.b #0,$DA2014
///   13FC 0020 00DA 201C   move.b #$20,$DA201C      ; READ SECTORS
///   0C78 0002 0400   w2:  cmpi.w #2,$400.w
///   6DF8                  blt.s  w2
///   43F8 2000             lea    $2000.w,a1
///   303C 00FF             move.w #255,d0
///   32F9 00DA 2000   l2:  move.w $DA2000,(a1)+
///   51C8 FFF8             dbra   d0,l2
///   31FC 600D 0402        move.w #$600D,$402.w
///   60FE                  bra.s  *
///
///   ; the level 2 handler, what `scsi.device`'s does in the order it does it
///   1239 00DA 9000        move.b $DA9000,d1        ; Gayle's change latches
///   1439 00DA 201C        move.b $DA201C,d2        ; the drive's Status: acknowledges
///   13FC 007C 00DA 9000   move.b #$7C,$DA9000      ; a 0 to bit 7 lets go
///   33FC 0008 00DF F09C   move.w #$0008,$DFF09C    ; INTREQ: clear PORTS
///   C338 0404             and.b  d1,$404.w
///   5278 0400             addq.w #1,$400.w
///   4E73                  rte
/// ```
fn kickstart() -> Vec<u8> {
    let main: &[u16] = &[
        0x13FC, 0x0000, 0x00BF, 0xE201, //
        0x11FC, 0x00FF, 0x0404, //
        0x4278, 0x0400, //
        0x21FC, 0x00F8, 0x0200, 0x0068, //
        0x13FC, 0x0080, 0x00DA, 0xA000, //
        0x33FC, 0xC008, 0x00DF, 0xF09A, //
        0x46FC, 0x2000, //
        0x13FC, 0x00A0, 0x00DA, 0x2018, //
        0x13FC, 0x00EC, 0x00DA, 0x201C, //
        0x4A78, 0x0400, 0x67FA, //
        0x43F8, 0x1000, 0x303C, 0x00FF, //
        0x32F9, 0x00DA, 0x2000, 0x51C8, 0xFFF8, //
        0x13FC, 0x00E0, 0x00DA, 0x2018, //
        0x13FC, 0x0001, 0x00DA, 0x2008, //
        0x13FC, 0x0000, 0x00DA, 0x200C, //
        0x13FC, 0x0000, 0x00DA, 0x2010, //
        0x13FC, 0x0000, 0x00DA, 0x2014, //
        0x13FC, 0x0020, 0x00DA, 0x201C, //
        0x0C78, 0x0002, 0x0400, 0x6DF8, //
        0x43F8, 0x2000, 0x303C, 0x00FF, //
        0x32F9, 0x00DA, 0x2000, 0x51C8, 0xFFF8, //
        0x31FC, 0x600D, 0x0402, //
        0x60FE,
    ];
    let handler: &[u16] = &[
        0x1239, 0x00DA, 0x9000, //
        0x1439, 0x00DA, 0x201C, //
        0x13FC, 0x007C, 0x00DA, 0x9000, //
        0x33FC, 0x0008, 0x00DF, 0xF09C, //
        0xC338, 0x0404, //
        0x5278, 0x0400, //
        0x4E73,
    ];
    let mut image = vec![0u8; 512 * 1024];
    image[0..4].copy_from_slice(&0x0010_0000u32.to_be_bytes()); // SSP: top of 1 MiB
    image[4..8].copy_from_slice(&0x00F8_0010u32.to_be_bytes()); // PC: the ROM's own window
    for (base, code) in [(0x10usize, main), (HANDLER, handler)] {
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
    let source = catalog::machine("amiga-a600")
        .expect("this build ships amiga-a600")
        .source;
    rsemu::machine::build("amiga-a600", source, &registry, &options)
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

#[test]
fn the_board_realizes_with_gayle_in_garys_place() {
    let m = build(disk());
    assert_eq!(m.name(), "amiga-a600");
    for path in [
        "cpu", "chipram", "kick", "custom", "paula", "agnus", "denise", "cia_a", "cia_b", "hd0",
        "gayle", "overlay", "df0", "kbd", "mouse",
    ] {
        assert!(m.device(path).is_some(), "no instance called `{path}`");
    }
}

#[test]
fn the_map_is_the_a600s() {
    let m = build(Vec::new());
    // Out of reset the ROM is at zero as well as at $F80000.
    assert_eq!(read(&m, 0, Width::U16), 0x0010);
    assert_eq!(read(&m, ROM + 4, Width::U16), 0x00F8);
    // Gayle's ROM select repeats the Kickstart at $E00000 and through
    // $A80000-$B7FFFF (Gayle Specification section 2.0).
    for mirror in [0xE0_0000u64, 0xA8_0000, 0xB0_0000] {
        assert_eq!(
            read(&m, mirror + 4, Width::U16),
            0x00F8,
            "{mirror:#x} should repeat the Kickstart"
        );
    }
    // No trapdoor RAM and no register mirror in bank 6, and no clock at
    // $DC0000: every one of them floats, and none faults.
    let space = m.space("mem").expect("the memory space");
    for addr in [0xC0_0000u64, 0xC0_F01C, 0xD0_0000, 0xDC_0000] {
        let floated = MemAttrs::DEBUG.with_bus(0x5A);
        assert_eq!(
            space.read(addr, Width::U8, floated),
            Ok(0x5A),
            "{addr:#x} should float on an A600"
        );
    }
    // Gayle's identification register answers at $DE1000.
    space
        .write(0xDE_1000, Width::U8, 0, MemAttrs::DEFAULT)
        .expect("a write");
    let id: Vec<u64> = (0..4)
        .map(|_| {
            space
                .read(0xDE_1000, Width::U8, MemAttrs::DEFAULT)
                .expect("a read")
                & 0x80
        })
        .collect();
    assert_eq!(id, [0x80, 0x80, 0, 0x80]);
    // With no disk bound the IDE bay is empty and the command block floats.
    let floated = MemAttrs::DEFAULT.with_bus(0x5A);
    assert_eq!(space.read(0xDA_201C, Width::U8, floated), Ok(0x5A));
}

#[test]
fn a_68000_program_reads_identify_and_a_sector_through_gayle_on_its_interrupt() {
    let image = disk();
    let mut m = build(image.clone());
    // Fifty milliseconds is some 350 000 processor cycles; the program needs a
    // few thousand, most of them the two 256-word copies.
    m.run_for(GlobalTime::from_nanos(50_000_000))
        .expect("it runs");

    assert_eq!(
        read(&m, DONE, Width::U16),
        0x600D,
        "the program never finished: {} interrupt(s) taken",
        read(&m, COUNT, Width::U16)
    );
    // Both waits ended on an interrupt, and every interrupt the handler took
    // had Gayle's IDE change bit set: it came through Gayle's latch, not from
    // anywhere else on INT2. Exactly two, one per command: both are PIO
    // data-in commands of a single DRQ block, and T13 ATA/ATAPI-6 §9.5 has the
    // drive interrupt when that block is ready and *not* when the host has
    // emptied it (DPIOI1:DI1, "The interrupt pending is not set on this
    // transition"). A third would be a completion interrupt ATA does not have.
    assert_eq!(read(&m, COUNT, Width::U16), 2);
    assert_eq!(read(&m, SEEN, Width::U8) & 0x80, 0x80);
    // INTREQ's PORTS is clear again: the handler cleared it, and Gayle let go.
    assert_eq!(read(&m, 0xDF_F01E, Width::U16) & 0x0008, 0);

    // IDENTIFY, as the 68000 stored it: byte-swapped, so the model string
    // reads pairwise reversed at word 27.
    let identify = bytes(&m, IDENTIFY_AT, 512);
    assert_eq!(
        &identify[54..60],
        b"SRME U",
        "\"RSEMU \" a pair at a time, swapped"
    );
    // Word 60-61, the LBA capacity, low word first: 128 sectors.
    assert_eq!(&identify[120..124], &[128, 0, 0, 0]);

    // And sector 0, byte for byte as the image holds it.
    assert_eq!(bytes(&m, SECTOR_AT, 512), &image[..512]);
}
