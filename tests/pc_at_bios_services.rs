//! Interrupt services of rsemu's own BIOS, called by a guest and checked
//! against the interface they are documented to have.
//!
//! Every test here boots `machines/pc-at.machine` on [`rsemu::fw::pcbios`],
//! off a diskette whose boot sector this file assembles with
//! [`rsemu::fw::asm16`], and reads the guest's answers back out of its memory.
//! Hermetic: nothing is downloaded and nothing but the firmware is under test.
//!
//! Each test is the regression test of one defect, and says which interface
//! sentence the old behaviour broke.
//!
//! # Sources
//!
//! Ralf Brown's Interrupt List for the `INT 10h`, `INT 13h` and `INT 16h`
//! register conventions; the *VBE Core Functions Standard* version 3.0 for
//! `AX=4F00h` and `AX=4F03h`. No firmware source and no emulator source was
//! consulted.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "dev-pc-hpet",
    feature = "fw-pcbios",
    feature = "machine-pc-at"
))]

use std::sync::Arc;

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::hosts::HostObjects;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::fw::asm16::{AX, Alu, Asm, DS, ES, Mem, SP, SS};
use rsemu::machine::{Machine, build};

/// Where the boot sector lands.
const BOOT: u16 = 0x7c00;
/// The block at `0x0500` every PC has left free since 1981.
const SCRATCH: u16 = 0x0500;
/// The guest writes this when it has finished.
const OFF_DONE: u16 = SCRATCH;
/// And this when it starts, so an answer cannot be mistaken for a boot that
/// never happened.
const OFF_STARTED: u16 = SCRATCH + 2;
/// Where the guest leaves what the services told it.
const R: u16 = SCRATCH + 0x10;

const STARTED: u16 = 0x1234;
const DONE: u16 = 0x600d;

/// How many sectors the fixed disk has: four cylinders of sixteen heads of 63
/// sectors, which is `ata.disk`'s own translation of that length.
const HD_SECTORS: usize = 4 * 16 * 63;
/// What every byte of the fixed disk's first sector holds.
const HD_FILL: u8 = 0x5a;

/// A fixed disk that is not bootable and whose first sector is recognisable.
fn fixed_disk() -> Vec<u8> {
    let mut image = vec![0u8; HD_SECTORS * 512];
    image[..510].fill(HD_FILL);
    image
}

/// Assemble a guest, put it on the diskette, boot `pc-at` on rsemu's own BIOS
/// and run until the guest says it has finished. The guest is left halted with
/// interrupts enabled, which is the state a PC waits for a key in.
fn run(name: &str, program: impl FnOnce(&mut Asm)) -> (Machine, Arc<HostObjects>) {
    let mut a = Asm::new(usize::from(BOOT) + 512, 0x00);
    a.seek(BOOT);
    a.cli();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT);
    a.sti();
    a.cld();
    a.movmi(Mem::abs(OFF_STARTED), STARTED);
    program(&mut a);
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movmi(Mem::abs(OFF_DONE), DONE);
    let spin = a.here_label();
    a.hlt();
    a.jmp(spin);
    assert!(
        a.here() <= BOOT + 510,
        "{name}: the boot sector is {} bytes and 510 is all a sector has",
        a.here() - BOOT
    );
    a.seek(BOOT + 510);
    a.db(&[0x55, 0xaa]);
    let mut floppy = a.finish()[usize::from(BOOT)..].to_vec();
    floppy.resize(1_474_560, 0);

    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options
        .realize
        .media
        .insert("bios", rsemu::fw::pcbios::image());
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("floppy", floppy);
    options.realize.media.insert("hd0", fixed_disk());
    options.realize.media.insert("hd1", Vec::new());
    options.realize.media.insert("cdrom", Vec::new());
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    m.reset(ResetKind::Cold);
    m.sweep();

    for _ in 0..8000 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek16(&m, OFF_DONE) == DONE {
            break;
        }
    }
    assert_eq!(
        peek16(&m, OFF_STARTED),
        STARTED,
        "{name}: the boot sector never ran"
    );
    assert_eq!(
        peek16(&m, OFF_DONE),
        DONE,
        "{name}: the guest never finished"
    );
    (m, Arc::clone(&options.realize.hosts))
}

fn peek(m: &Machine, at: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(at, Width::U8, MemAttrs::DEBUG)
        .unwrap_or(0xff) as u8
}

fn peek16(m: &Machine, at: u16) -> u16 {
    u16::from(peek(m, u64::from(at))) | (u16::from(peek(m, u64::from(at) + 1)) << 8)
}

/// Leave `AX` at `at` and the carry flag, as 0 or 1, in the word after it.
fn record(a: &mut Asm, at: u16) {
    a.pushf();
    a.movto(Mem::abs(at), AX);
    a.pop(AX);
    a.alui(Alu::AND, AX, 0x0001);
    a.movto(Mem::abs(at + 2), AX);
}

/// `INT 13h AH=42h`/`43h` with a block number the drive cannot be asked for.
///
/// The disk address packet carries a 64-bit block number (RBIL, table 00272:
/// "08h QWORD starting absolute block number") and the firmware reaches the
/// drive through `READ SECTOR(S)`/`WRITE SECTOR(S)`, which carry 28 bits. A
/// block number that does not fit must be refused: with the excess bits
/// dropped, block `1000_0000h` is block 0, and an extended *write* aimed past
/// the end of the disk lands on the master boot record.
#[test]
fn an_extended_transfer_beyond_28_bits_is_refused_rather_than_wrapped() {
    use rsemu::fw::asm16::{CX, DI, DX, SI};

    /// The disk address packet.
    const PACKET: u16 = 0x0640;
    /// Where the out-of-range read would land, pre-filled so a transfer shows.
    const READ_BUF: u16 = 0x0c00;
    /// What the out-of-range write would write.
    const WRITE_BUF: u16 = 0x0e00;
    /// Where block 0 is read back to at the end.
    const CHECK_BUF: u16 = 0x1000;

    let (m, _hosts) = run("edd range", |a| {
        for (buffer, fill) in [(READ_BUF, 0x1111u16), (WRITE_BUF, 0xeeee)] {
            a.movi(DI, buffer);
            a.movi(CX, 256);
            a.movi(AX, fill);
            a.rep();
            a.stosw();
        }
        a.movmi8(Mem::abs(PACKET), 0x10);
        a.movmi8(Mem::abs(PACKET + 1), 0x00);
        a.movmi(Mem::abs(PACKET + 2), 1);
        a.movmi(Mem::abs(PACKET + 6), 0x0000);

        // Block 1000_0000h: bit 28 set, everything below it clear.
        a.movmi(Mem::abs(PACKET + 4), READ_BUF);
        a.movmi32(Mem::abs(PACKET + 8), 0x1000_0000);
        a.movmi32(Mem::abs(PACKET + 12), 0);
        a.movi(AX, 0x4200);
        a.movi(DX, 0x0080);
        a.movi(SI, PACKET);
        a.int(0x13);
        record(a, R);

        // The same block, written.
        a.movmi(Mem::abs(PACKET + 4), WRITE_BUF);
        a.movi(AX, 0x4300);
        a.movi(DX, 0x0080);
        a.movi(SI, PACKET);
        a.int(0x13);
        record(a, R + 4);

        // Block 1_0000_0000h: only the high double word is non-zero.
        a.movmi(Mem::abs(PACKET + 4), READ_BUF);
        a.movmi32(Mem::abs(PACKET + 8), 0);
        a.movmi32(Mem::abs(PACKET + 12), 1);
        a.movi(AX, 0x4200);
        a.movi(DX, 0x0080);
        a.movi(SI, PACKET);
        a.int(0x13);
        record(a, R + 8);

        // And block 0 itself, which must still be what the disk came with.
        a.movmi(Mem::abs(PACKET + 4), CHECK_BUF);
        a.movmi32(Mem::abs(PACKET + 12), 0);
        a.movi(AX, 0x4200);
        a.movi(DX, 0x0080);
        a.movi(SI, PACKET);
        a.int(0x13);
        record(a, R + 12);
    });

    assert_eq!(peek16(&m, R + 2), 1, "a read of block 1000_0000h succeeded");
    assert_ne!(peek16(&m, R) >> 8, 0, "and it must say why in AH");
    assert_eq!(
        peek16(&m, READ_BUF),
        0x1111,
        "the refused read still moved a sector"
    );
    assert_eq!(peek16(&m, R + 6), 1, "a write to that block succeeded");
    assert_eq!(peek16(&m, R + 10), 1, "block 1_0000_0000h was read");
    assert_eq!(peek16(&m, R + 14), 0, "block 0 itself must read");
    assert_eq!(peek16(&m, R + 12) >> 8, 0);
    assert_eq!(
        peek16(&m, CHECK_BUF),
        u16::from_le_bytes([HD_FILL, HD_FILL]),
        "the out-of-range write landed on block 0"
    );
}

fn peek32(m: &Machine, at: u16) -> u32 {
    u32::from(peek16(m, at)) | (u32::from(peek16(m, at + 2)) << 16)
}

/// A service that saves its caller's registers must save all of each one.
///
/// `INT 13h` and `INT 10h` opened with a 16-bit `PUSHA` and then used `EAX`,
/// `EBX`, `ECX` and `EDX` as 32-bit scratch — the CHS translation multiplies
/// in 32 bits, the VBE services call the PCI BIOS with a class code in `ECX`
/// — so a caller came back with the upper halves of registers the interface
/// does not return anything in overwritten. A 32-bit real-mode loader that
/// keeps a block number in `EBX` across a read is the program that notices.
#[test]
fn the_disk_and_video_services_preserve_the_upper_register_halves() {
    use rsemu::fw::asm16::{BX, CX, DI, DX, SI};

    let (m, _hosts) = run("upper halves", |a| {
        // AH=02h: one sector, cylinder 0, head 0, sector 1 of the fixed disk.
        a.movi32(AX, 0xa5a5_0201);
        a.movi32(BX, 0x5a5a_0c00);
        a.movi32(CX, 0x1111_0001);
        a.movi32(DX, 0x2222_0080);
        a.movi32(SI, 0x3333_0000);
        a.movi32(DI, 0x4444_0000);
        a.int(0x13);
        for (slot, register) in [AX, BX, CX, DX, SI, DI].into_iter().enumerate() {
            a.movto32(Mem::abs(R + 4 * slot as u16), register);
        }
        // AX=4F01h: the mode information block for mode 101h, at 0000:0E00.
        a.movi32(AX, 0xa5a5_4f01);
        a.movi32(CX, 0x1111_0101);
        a.movi32(DX, 0x2222_0000);
        a.movi32(DI, 0x4444_0e00);
        a.int(0x10);
        for (slot, register) in [AX, CX, DX, DI].into_iter().enumerate() {
            a.movto32(Mem::abs(R + 24 + 4 * slot as u16), register);
        }
    });

    assert_eq!(peek32(&m, R), 0xa5a5_0001, "EAX after INT 13h AH=02h");
    assert_eq!(peek16(&m, 0x0c00), u16::from_le_bytes([HD_FILL, HD_FILL]));
    assert_eq!(peek32(&m, R + 4), 0x5a5a_0c00, "EBX after INT 13h AH=02h");
    assert_eq!(peek32(&m, R + 8), 0x1111_0001, "ECX after INT 13h AH=02h");
    assert_eq!(peek32(&m, R + 12), 0x2222_0080, "EDX after INT 13h AH=02h");
    assert_eq!(peek32(&m, R + 16), 0x3333_0000, "ESI after INT 13h AH=02h");
    assert_eq!(peek32(&m, R + 20), 0x4444_0000, "EDI after INT 13h AH=02h");

    assert_eq!(peek32(&m, R + 24), 0xa5a5_004f, "EAX after AX=4F01h");
    assert_eq!(peek32(&m, R + 28), 0x1111_0101, "ECX after AX=4F01h");
    assert_eq!(peek32(&m, R + 32), 0x2222_0000, "EDX after AX=4F01h");
    assert_eq!(peek32(&m, R + 36), 0x4444_0e00, "EDI after AX=4F01h");
}

/// `INT 13h AH=08h` on a drive with more than 1024 cylinders.
///
/// The function answers with the *maximum cylinder number* in ten bits — `CH`
/// and the top two bits of `CL` — so 1023 is the largest it can say. The
/// firmware took `cylinders - 1` and kept its low ten bits, which is a wrap
/// rather than a limit: a drive `IDENTIFY DEVICE` reports with 1025 cylinders
/// came back as a drive with one.
///
/// The cylinder count is the firmware's own copy of `IDENTIFY DEVICE` word 1,
/// in the Extended BIOS Data Area. The guest writes it there, because a disk
/// image big enough to have 1025 real cylinders is half a gigabyte.
#[test]
fn ah_08h_limits_the_cylinder_count_instead_of_wrapping_it() {
    use rsemu::fw::asm16::{AH, CX, DX};

    /// The firmware's EBDA, and the fixed disk's cylinder count inside it.
    const EBDA_SEGMENT: u16 = 0x9fc0;
    const EBDA_HD_CYLINDERS: u16 = 0x02;

    let cases: [(u16, u16); 4] = [(4, 3), (1024, 1023), (1025, 1023), (16383, 1023)];
    let (m, _hosts) = run("cylinder limit", |a| {
        for (slot, (cylinders, _)) in cases.into_iter().enumerate() {
            a.movi(AX, EBDA_SEGMENT);
            a.movsr(ES, AX);
            a.movmi(Mem::abs(EBDA_HD_CYLINDERS).seg(ES), cylinders);
            a.movi8(AH, 0x08);
            a.movi(DX, 0x0080);
            a.int(0x13);
            a.movto(Mem::abs(R + 2 * slot as u16), CX);
        }
    });

    for (slot, (cylinders, expected)) in cases.into_iter().enumerate() {
        let cx = peek16(&m, R + 2 * slot as u16);
        let reported = (cx >> 8) | ((cx & 0x00c0) << 2);
        assert_eq!(
            reported, expected,
            "a drive with {cylinders} cylinders reports {reported} as its last one"
        );
        assert_eq!(cx & 0x3f, 63, "and the sectors per track are untouched");
    }
}

/// The colour text page, as a physical address.
const TEXT_PAGE: u64 = 0xb8000;

/// The character and attribute of one cell of the text page.
fn cell(m: &Machine, row: u64, column: u64) -> (u8, u8) {
    let at = TEXT_PAGE + (row * 80 + column) * 2;
    (peek(m, at), peek(m, at + 1))
}

/// The services run with the direction flag *they* need, not the caller's.
///
/// `INT 10h` moves and fills the text page with `REP MOVSW` and `REP STOSW`
/// and never cleared the direction flag, so a caller that arrived with it set
/// — in the middle of a backward copy, say — had its three characters written
/// leftwards from the cursor and its scroll run off the top of the page.
/// `INT 13h` and `INT 15h` have always opened with `CLD` for this reason.
#[test]
fn int_10h_writes_forwards_whatever_the_callers_direction_flag_says() {
    use rsemu::fw::asm16::{BX, CX, DX};

    let (m, _hosts) = run("direction flag", |a| {
        // The cursor to row 5, column 10.
        a.movi(AX, 0x0200);
        a.movi(BX, 0x0000);
        a.movi(DX, 0x050a);
        a.int(0x10);
        // STD (SDM Vol. 2B, `STD`: FD), then three `A`s in yellow on blue.
        a.db(&[0xfd]);
        a.movi(AX, 0x0941);
        a.movi(BX, 0x001e);
        a.movi(CX, 3);
        a.int(0x10);
        a.cld();
    });

    for column in 10..13 {
        assert_eq!(
            cell(&m, 5, column),
            (b'A', 0x1e),
            "row 5 column {column} was not written"
        );
    }
    assert_ne!(cell(&m, 5, 9).0, b'A', "the run went leftwards");
}

/// `INT 10h AH=0Ah` writes the character and leaves the attribute alone.
///
/// RBIL, `INT 10h AH=0Ah`: "VIDEO - WRITE CHARACTER ONLY AT CURSOR POSITION",
/// with "BL = attribute (PCjr, Tandy 1000 only) or color (graphics mode)". On
/// an AT in a text mode `BL` is therefore not an attribute at all, and the
/// firmware used it as one: a caller with zero in `BL` got black on black.
#[test]
fn ah_0ah_writes_the_character_and_keeps_the_cells_attribute() {
    use rsemu::fw::asm16::{BX, CX, DX};

    let (m, _hosts) = run("write character only", |a| {
        a.movi(AX, 0x0200);
        a.movi(BX, 0x0000);
        a.movi(DX, 0x0600);
        a.int(0x10);
        // Three cells in yellow on blue with AH=09h...
        a.movi(AX, 0x0978);
        a.movi(BX, 0x001e);
        a.movi(CX, 3);
        a.int(0x10);
        // ...and the first two of them overwritten with AH=0Ah and BL=0.
        a.movi(AX, 0x0a79);
        a.movi(BX, 0x0000);
        a.movi(CX, 2);
        a.int(0x10);
    });

    assert_eq!(cell(&m, 6, 0), (b'y', 0x1e));
    assert_eq!(cell(&m, 6, 1), (b'y', 0x1e));
    assert_eq!(cell(&m, 6, 2), (b'x', 0x1e), "the count was overrun");
}
