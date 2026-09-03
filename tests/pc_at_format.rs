//! **A guest formats a diskette track through rsemu's own BIOS.**
//!
//! `INT 13h AH=05h` was the last diskette service [`rsemu::fw::pcbios`] did
//! not implement, and the reason given for that — "formatting is a different
//! command phase" — stopped being true once `pc.fdc` grew `FORMAT A TRACK`.
//! The chip's command reads four ID bytes per sector out of memory through the
//! 8237 during its execution phase and writes a whole track of empty sectors,
//! so the firmware's part is a seek, a DMA programming with a length that is
//! not 512, and six command bytes.
//!
//! The proof is before and after, on the *second* head of cylinder 0 — a track
//! nothing on this diskette uses, so a bug here cannot quietly destroy the
//! boot sector the test is running from:
//!
//! 1. read cylinder 0, head 1, sector 1 with `AH=02h` and keep a byte of it;
//! 2. build an address-field list — `C`, `H`, `R`, `N` for eighteen sectors —
//!    and format the track with `AH=05h`;
//! 3. read the same sector again.
//!
//! The image is a boot sector followed by zeros, so the first read is zeros
//! and the second has to be the format filler. Nothing but a `FORMAT A TRACK`
//! that actually reached the drive can turn one into the other.

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

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::fw::asm16::{AL, AX, Alu, Asm, BX, CX, Cc, DI, DS, DX, ES, Mem, SP, SS};
use rsemu::machine::{Machine, build};

/// Where the boot sector lands.
const BOOT: u16 = 0x7c00;
/// The block at `0x0500` every PC has left free since 1981.
const SCRATCH: u16 = 0x0500;
/// It ran at all.
const OFF_STARTED: u16 = SCRATCH;
/// It finished.
const OFF_DONE: u16 = SCRATCH + 2;
/// `AX` and the flags from the read before the format.
const OFF_READ_BEFORE: u16 = SCRATCH + 4;
/// `AX` and the flags from `AH=05h`.
const OFF_FORMAT: u16 = SCRATCH + 8;
/// `AX` and the flags from the read after it.
const OFF_READ_AFTER: u16 = SCRATCH + 12;

/// Where the address-field list is built: four bytes per sector.
const LIST: u16 = 0x0600;
/// Where the sector read before the format goes.
const BEFORE: u16 = 0x1000;
/// Where the sector read after it goes.
const AFTER: u16 = 0x1200;

/// How many sectors the track is formatted with — a 1.44 MB diskette's own
/// count, which is what the drive's geometry says and what the CMOS type byte
/// resolves to.
const SECTORS: u8 = 18;
/// The byte a freshly formatted sector's data field holds.
const FILLER: u8 = 0xf6;

/// What [`OFF_STARTED`] holds.
const STARTED: u16 = 0x0f05;
/// What [`OFF_DONE`] holds.
const DONE: u16 = 0x600d;

/// Save `AX` and the flags the last `INT 13h` returned.
fn record(a: &mut Asm, at: u16) {
    a.movto(Mem::abs(at), AX);
    a.pushf();
    a.pop(AX);
    a.movto(Mem::abs(at + 2), AX);
}

/// Read cylinder 0, head 1, sector 1 into `0000:buffer`.
fn read_one(a: &mut Asm, buffer: u16) {
    a.movi(AX, 0x0201);
    a.movi(CX, 0x0001);
    a.movi(DX, 0x0100);
    a.movi(BX, buffer);
    a.int(0x13);
}

/// Assemble the boot sector.
fn boot_sector() -> Vec<u8> {
    let mut a = Asm::new(usize::from(BOOT) + 512, 0x00);
    a.seek(BOOT);

    a.cli();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT);
    a.sti();
    a.movmi(Mem::abs(OFF_STARTED), STARTED);

    read_one(&mut a, BEFORE);
    record(&mut a, OFF_READ_BEFORE);

    // The address field list the controller reads during the execution phase:
    // cylinder, head, sector number and size code for each sector, in the
    // order they are to be laid down (µPD765A data sheet, `FORMAT A TRACK`).
    // Sector numbers from 1, which is what every PC format has done.
    a.movi(DI, LIST);
    a.movi(CX, u16::from(SECTORS));
    a.movi8(AL, 1);
    let fill = a.here_label();
    a.movmi8(Mem::di(0), 0); // C
    a.movmi8(Mem::di(1), 1); // H
    a.movto8(Mem::di(2), AL); // R
    a.movmi8(Mem::di(3), 2); // N, a 512-byte sector
    a.incm8(AL);
    a.alui(Alu::ADD, DI, 4);
    a.dec(CX);
    a.jcc(Cc::NE, fill);

    // AL sectors, cylinder in CH, head in DH, drive 0 in DL, the list at ES:BX.
    a.movi(AX, 0x0500 | u16::from(SECTORS));
    a.movi(CX, 0x0000);
    a.movi(DX, 0x0100);
    a.movi(BX, LIST);
    a.int(0x13);
    record(&mut a, OFF_FORMAT);

    read_one(&mut a, AFTER);
    record(&mut a, OFF_READ_AFTER);

    a.movmi(Mem::abs(OFF_DONE), DONE);
    let spin = a.here_label();
    a.hlt();
    a.jmp(spin);

    assert!(
        a.here() <= BOOT + 510,
        "the boot sector is {} bytes and 510 is all a sector has",
        a.here() - BOOT
    );
    a.seek(BOOT + 510);
    a.db(&[0x55, 0xaa]);

    let image = a.finish();
    image[usize::from(BOOT)..].to_vec()
}

/// A 1.44 MB diskette with that sector on it and zeros everywhere else.
fn diskette() -> Vec<u8> {
    let mut image = boot_sector();
    assert_eq!(image.len(), 512, "a boot sector is one sector");
    image.resize(1_474_560, 0);
    image
}

/// `machines/pc-at.machine` with rsemu's own BIOS and that diskette.
fn board() -> Machine {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options
        .realize
        .media
        .insert("bios", rsemu::fw::pcbios::image());
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("floppy", diskette());
    for slot in ["disk", "hd0", "hd1", "cd0", "cd1"] {
        options.realize.media.insert(slot, Vec::new());
    }
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options)
        .unwrap_or_else(|e| panic!("pc-at does not realize: {e}"));
    m.reset(ResetKind::Cold);
    m.sweep();
    m
}

/// A word of guest memory, read as a debugger reads.
fn peek16(m: &Machine, at: u16) -> u16 {
    m.space("mem")
        .expect("the memory space")
        .read(u64::from(at), Width::U16, MemAttrs::DEBUG)
        .unwrap_or(0) as u16
}

/// A byte of it.
fn peek8(m: &Machine, at: u16) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(u64::from(at), Width::U8, MemAttrs::DEBUG)
        .unwrap_or(0) as u8
}

/// What one `INT 13h` call answered.
fn status(m: &Machine, at: u16) -> (u8, bool) {
    ((peek16(m, at) >> 8) as u8, peek16(m, at + 2) & 1 != 0)
}

#[test]
fn a_guest_formats_a_track_and_reads_the_filler_back() {
    let mut m = board();
    for _ in 0..3000 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("the board runs");
        if peek16(&m, OFF_DONE) == DONE {
            break;
        }
    }
    assert_eq!(
        peek16(&m, OFF_STARTED),
        STARTED,
        "the boot sector never ran: `INT 19h` did not reach it"
    );
    assert_eq!(peek16(&m, OFF_DONE), DONE, "the guest did not finish");

    let (before_ah, before_cf) = status(&m, OFF_READ_BEFORE);
    assert!(
        !before_cf,
        "the read before the format failed: {before_ah:#04x}"
    );
    let (format_ah, format_cf) = status(&m, OFF_FORMAT);
    assert!(!format_cf, "`AH=05h` reported an error: {format_ah:#04x}");
    assert_eq!(format_ah, 0, "`AH=05h` did not return zero status");
    let (after_ah, after_cf) = status(&m, OFF_READ_AFTER);
    assert!(
        !after_cf,
        "the read after the format failed: {after_ah:#04x}"
    );

    // The image is a boot sector and then zeros, so the second head's first
    // sector was zeros; after the format it is the filler, all the way to the
    // end of the sector.
    for offset in [0u16, 1, 0x80, 0x1ff] {
        assert_eq!(
            peek8(&m, BEFORE + offset),
            0,
            "byte {offset:#x} was not zero before the format"
        );
        assert_eq!(
            peek8(&m, AFTER + offset),
            FILLER,
            "byte {offset:#x} is not the format filler"
        );
    }
}
