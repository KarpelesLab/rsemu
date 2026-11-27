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
