//! The disk services FreeDOS's installer needed, on rsemu's own firmware and
//! nothing downloaded.
//!
//! Installing an operating system asks `INT 13h` three things the boot of one
//! never does, and all three were missing or wrong. Each is fixed in
//! [`rsemu::fw::pcbios`], and each is checked here by a guest program this
//! file assembles — hermetic, so it runs in an ordinary `cargo test`, while
//! `tests/pc_at_freedos.rs` is the same claims with a real installer on the
//! other end and is gated on a download.
//!
//! # What is being claimed
//!
//! 1. **The diskette is tried before the fixed disk.** The AT's bootstrap
//!    loader reads the boot record from diskette drive A and only then from
//!    the fixed disk (*IBM Personal Computer AT Technical Reference*, the
//!    BIOS bootstrap loader). With both media bootable, the diskette's guest
//!    is the one that runs — which is what lets an installer partition a disk
//!    and reboot into its own installer again rather than into the empty
//!    partition it has just made.
//! 2. **The EDD fixed-disk subset is all there.** `INT 13h AH=41h` sets bit 0
//!    of `CX`, which EDD 1.1 defines as functions 42h, 43h, 44h, 47h and 48h.
//!    The bit was set with only 42h behind it, and FreeDOS's `FDISK` believed
//!    it: it called 48h for the disk's size, got carry, and reported "No
//!    fixed disks present" on a board with a disk in it. So the guest calls
//!    48h and checks its result table against what AH=08h says, then writes a
//!    sector with 43h and reads it back with 42h.
//! 3. **The diskette's change line is reported.** `AH=15h` answers 02h, a
//!    drive *with* a change line, and `AH=16h` reports it: 06h while the line
//!    is active, 00h once a seek with a diskette in the drive has cleared it
//!    (*IBM PC/AT Technical Reference*, the diskette adapter's digital input
//!    register, bit 7). That is how a DOS finds out that the diskette under
//!    its cached directory is not the one it cached, and it is what a person
//!    swapping the second disk of an installation set depends on.
//!
//! # Sources
//!
//! *IBM Personal Computer AT Technical Reference* (the bootstrap loader, the
//! diskette adapter); T13 D1386 *BIOS Enhanced Disk Drive Services* for the
//! 41h/42h/43h/48h ABI and its result table; Ralf Brown's Interrupt List for
//! the register conventions of 15h and 16h. No emulator source was consulted,
//! and no FreeDOS source: what the installer needed was learned from what it
//! *called*, by decoding the guest's own `INT` instructions.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "fw-pcbios",
    feature = "machine-pc-at"
))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::fw::asm16::{
    AH, AL, AX, Alu, Asm, BL, BX, CX, Cc, DH, DI, DL, DS, DX, ES, Mem, SI, SP, SS,
};
use rsemu::machine::realize::Bindings;
use rsemu::machine::{Machine, build};

/// Where the boot sector lands, and what its labels are relative to.
const BOOT_ADDRESS: u16 = 0x7c00;

/// The scratch block the guest leaves its answers in — below the stack, above
/// the BIOS Data Area, in the region every PC has left free since 1981.
const SCRATCH: u16 = 0x0500;

/// Which guest is running: the diskette's or the fixed disk's.
const MARKER: u16 = SCRATCH;
/// The drive the firmware said it booted from, out of `DL`.
const BOOT_DRIVE: u16 = SCRATCH + 2;
/// `AX`, `BX` and `CX` from `INT 13h AH=41h`.
const EDD_AX: u16 = SCRATCH + 4;
const EDD_BX: u16 = SCRATCH + 6;
const EDD_CX: u16 = SCRATCH + 8;
/// `AX` and the carry flag from `AH=48h`, and `CX`/`DX` from `AH=08h`.
const PARAMS_AX: u16 = SCRATCH + 10;
const PARAMS_CF: u16 = SCRATCH + 12;
const CHS_CX: u16 = SCRATCH + 14;
const CHS_DX: u16 = SCRATCH + 16;
/// `AX` from the extended write and the extended read that follows it.
const WRITE_AX: u16 = SCRATCH + 18;
const READ_AX: u16 = SCRATCH + 20;
/// The diskette's change line, as `AH=16h` last reported it, and how many
/// times the guest has been round its polling loop.
const CHANGE: u16 = SCRATCH + 22;
const LOOPS: u16 = SCRATCH + 24;
/// The mailbox the *test* writes: non-zero asks the guest to read a diskette
/// sector, which is the seek that clears the change line.
const READ_DISKETTE: u16 = SCRATCH + 26;
/// What that read answered.
const DISKETTE_AX: u16 = SCRATCH + 28;
/// `AH` from `AH=15h`, the drive type.
const DRIVE_TYPE: u16 = SCRATCH + 30;

/// The 26-byte result table `AH=48h` fills in.
const EDD_PARAMS: u16 = 0x0600;
/// The sixteen-byte disk address packet 42h and 43h are given.
const PACKET: u16 = 0x0640;
/// The sector the guest builds, writes and reads back.
const PATTERN: u16 = 0x0a00;
/// Where it reads it back to.
const READBACK: u16 = 0x0c00;
/// Where a diskette sector lands.
const DISKETTE_BUFFER: u16 = 0x0e00;

/// The LBA the extended write and read use: past the boot sector, and not a
/// round number, so a transfer that ignored the packet's LBA lands elsewhere.
const EDD_LBA: u32 = 37;

/// The first word of the pattern; word `k` is this plus `k`.
const PATTERN_SEED: u16 = 0x4321;

/// The marker the diskette's guest writes, and the fixed disk's.
const DISKETTE_MARKER: u16 = 0xfd00;
const FIXED_MARKER: u16 = 0x8000;

/// How big the fixed disk is, in sectors: four cylinders of sixteen heads of
/// 63 sectors, which is `ata.disk`'s own translation of that length.
const HD_SECTORS: u64 = 4 * 16 * 63;

/// The 512 bytes the guest generates and moves about.
fn pattern() -> Vec<u8> {
    (0..256u16)
        .flat_map(|k| PATTERN_SEED.wrapping_add(k).to_le_bytes())
        .collect()
}

// ---------------------------------------------------------------------------
// the guest
// ---------------------------------------------------------------------------

/// A boot sector that records `marker`, exercises the EDD subset on the fixed
/// disk if there is one, and then polls the diskette's change line for ever.
#[allow(clippy::too_many_lines)]
fn boot_sector(marker: u16) -> Vec<u8> {
    let mut a = Asm::new(usize::from(BOOT_ADDRESS) + 512, 0x00);
    a.seek(BOOT_ADDRESS);

    a.cli();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT_ADDRESS);
    a.sti();
    a.movmi(Mem::abs(MARKER), marker);
    a.movi(AX, 0);
    a.mov8(AL, DL);
    a.movto(Mem::abs(BOOT_DRIVE), AX);

    // -- the EDD installation check ----------------------------------------
    //
    // BX must arrive as 0x55AA and comes back byte-swapped; CX's bit 0 is the
    // fixed-disk access subset.
    a.movi8(AH, 0x41);
    a.movi(BX, 0x55aa);
    a.movi(DX, 0x0080);
    a.int(0x13);
    a.movto(Mem::abs(EDD_AX), AX);
    a.movto(Mem::abs(EDD_BX), BX);
    a.movto(Mem::abs(EDD_CX), CX);

    // -- AH=48h, get drive parameters --------------------------------------
    //
    // The caller declares the buffer's size in its first word; anything below
    // EDD 1.1's 1Ah must be refused rather than overrun, and 1Eh is what a
    // caller that would also accept a 2.0 table asks with.
    a.movi(DI, EDD_PARAMS);
    a.movi(CX, 0x1e / 2);
    a.movi(AX, 0);
    let clear = a.here_label();
    a.stosw();
    a.dec(CX);
    a.jcc(Cc::NE, clear);
    a.movmi(Mem::abs(EDD_PARAMS), 0x1e);
    a.movi8(AH, 0x48);
    a.movi(DX, 0x0080);
    a.movi(SI, EDD_PARAMS);
    a.int(0x13);
    a.movto(Mem::abs(PARAMS_AX), AX);
    a.movi(AX, 0);
    let no_carry = a.label();
    a.jcc(Cc::AE, no_carry);
    a.movi(AX, 1);
    a.bind(no_carry);
    a.movto(Mem::abs(PARAMS_CF), AX);

    // -- AH=08h, the same geometry the old way ------------------------------
    a.movi8(AH, 0x08);
    a.movi(DX, 0x0080);
    a.int(0x13);
    a.movto(Mem::abs(CHS_CX), CX);
    a.movto(Mem::abs(CHS_DX), DX);

    // -- the pattern, written with 43h and read back with 42h ---------------
    a.movi(DI, PATTERN);
    a.movi(CX, 256);
    a.movi(AX, PATTERN_SEED);
    let fill = a.here_label();
    a.stosw();
    a.inc(AX);
    a.dec(CX);
    a.jcc(Cc::NE, fill);

    // The disk address packet: size, reserved, one block, a far buffer
    // pointer, and a 64-bit LBA.
    a.movmi8(Mem::abs(PACKET), 0x10);
    a.movmi8(Mem::abs(PACKET + 1), 0x00);
    a.movmi(Mem::abs(PACKET + 2), 1);
    a.movmi(Mem::abs(PACKET + 4), PATTERN);
    a.movmi(Mem::abs(PACKET + 6), 0x0000);
    a.movmi32(Mem::abs(PACKET + 8), EDD_LBA);
    a.movmi32(Mem::abs(PACKET + 12), 0);
    a.movi(AX, 0x4300);
    a.movi(DX, 0x0080);
    a.movi(SI, PACKET);
    a.int(0x13);
    a.movto(Mem::abs(WRITE_AX), AX);

    a.movmi(Mem::abs(PACKET + 4), READBACK);
    a.movi(AX, 0x4200);
    a.movi(DX, 0x0080);
    a.movi(SI, PACKET);
    a.int(0x13);
    a.movto(Mem::abs(READ_AX), AX);

    // -- AH=15h, what kind of drive A is ------------------------------------
    a.movi8(AH, 0x15);
    a.movi(DX, 0x0000);
    a.int(0x13);
    a.movi(BX, 0);
    a.mov8(BL, AH);
    a.movto(Mem::abs(DRIVE_TYPE), BX);

    // -- and the change line, for ever --------------------------------------
    //
    // `AH=16h` on drive 0 every time round, and a read of the diskette
    // whenever the mailbox says to — which is the seek that clears the line.
    // Nothing here halts: the test wants an answer at an instant of its
    // choosing, not one interrupt later.
    let loop_top = a.here_label();
    a.incm(Mem::abs(LOOPS));
    a.movi8(AH, 0x16);
    a.movi(DX, 0x0000);
    a.int(0x13);
    a.movi(BX, 0);
    a.mov8(BL, AH);
    a.movto(Mem::abs(CHANGE), BX);

    let again = a.label();
    a.alui(Alu::CMP, Mem::abs(READ_DISKETTE), 0);
    a.jcc(Cc::E, again);
    a.movmi(Mem::abs(READ_DISKETTE), 0);
    a.movi(AX, 0x0201);
    a.movi(CX, 0x0001); // cylinder 0, sector 1
    a.movi8(DH, 0x00);
    a.movi8(DL, 0x00);
    a.movi(BX, DISKETTE_BUFFER);
    a.int(0x13);
    a.movto(Mem::abs(DISKETTE_AX), AX);
    a.bind(again);
    a.jmp(loop_top);

    assert!(
        a.here() <= BOOT_ADDRESS + 510,
        "the boot sector is {} bytes and a sector has 510",
        a.here() - BOOT_ADDRESS
    );
    a.seek(BOOT_ADDRESS + 510);
    a.db(&[0x55, 0xaa]);
    let image = a.finish();
    image[usize::from(BOOT_ADDRESS)..].to_vec()
}

/// A 1.44 MB diskette with `marker`'s boot sector on it.
fn diskette(marker: u16) -> Vec<u8> {
    let mut image = boot_sector(marker);
    image.resize(1_474_560, 0);
    image
}

/// A fixed-disk image with `marker`'s boot sector on it, or with none.
fn fixed_disk(marker: Option<u16>) -> Vec<u8> {
    let mut image = match marker {
        Some(marker) => boot_sector(marker),
        None => vec![0u8; 512],
    };
    image.resize((HD_SECTORS * 512) as usize, 0);
    image
}

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

fn board(
    floppy: Vec<u8>,
    hd0: Vec<u8>,
) -> (Machine, Arc<X86>, Arc<rsemu::core::hosts::HostObjects>) {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let mut b = Bindings::new();
    rsemu::machine::builtin::bind(&mut b).expect("ram and rom");
    rsemu::dev::pc::bind(&mut b).expect("the chipset");
    rsemu::dev::ata::bind(&mut b).expect("the hard disks");
    let kept = Arc::clone(&cpus);
    b.bind("cpu.x86", move |props| {
        let cpu = Arc::new(X86::from_props_defaulting(props, Variant::I80486)?);
        kept.push(&cpu);
        Ok(cpu)
    })
    .expect("nothing else in this table claims the name");

    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(b);
    options
        .realize
        .media
        .insert("bios", rsemu::fw::pcbios::image());
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("floppy", floppy);
    options.realize.media.insert("hd0", hd0);
    options.realize.media.insert("hd1", Vec::new());
    // The CD-ROM drive with no disc in it, which is what no bytes bound means.
    options.realize.media.insert("cdrom", Vec::new());

    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let cpu = cpus.take().expect("the constructor kept a handle");
    m.reset(ResetKind::Cold);
    m.sweep();
    (m, cpu, Arc::clone(&options.realize.hosts))
}

fn peek(m: &Machine, addr: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEBUG)
        .unwrap_or(0xff) as u8
}

fn peek16(m: &Machine, addr: u64) -> u16 {
    u16::from(peek(m, addr)) | (u16::from(peek(m, addr + 1)) << 8)
}

fn peek32(m: &Machine, addr: u64) -> u32 {
    u32::from(peek16(m, addr)) | (u32::from(peek16(m, addr + 2)) << 16)
}

/// Write the guest's mailbox. The only byte of guest memory this file writes,
/// and it is the guest's own scratch: there is no other way to ask a program
/// that is already running to do something at an instant of the test's
/// choosing.
fn poke16(m: &Machine, addr: u64, value: u16) {
    let space = m.space("mem").expect("the memory space");
    space
        .write(addr, Width::U16, u64::from(value), MemAttrs::DEBUG)
        .expect("the guest's scratch block is RAM");
}

/// Long enough for POST, the boot and a few thousand times round the guest's
/// polling loop.
const BOOTED: GlobalTime = GlobalTime::from_nanos(200_000_000);
/// Long enough for the loop to notice a mailbox or a swap.
const A_MOMENT: GlobalTime = GlobalTime::from_nanos(5_000_000);

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

/// With both media bootable, the **diskette** is the one that boots.
#[test]
fn the_diskette_is_tried_before_the_fixed_disk() {
    let (mut m, _cpu, _hosts) = board(diskette(DISKETTE_MARKER), fixed_disk(Some(FIXED_MARKER)));
    m.run_for(BOOTED).expect("the machine runs");
    assert_eq!(
        peek16(&m, u64::from(MARKER)),
        DISKETTE_MARKER,
        "the fixed disk booted although there was a bootable diskette in the drive"
    );
    assert_eq!(
        peek16(&m, u64::from(BOOT_DRIVE)),
        0x0000,
        "and the firmware told it which drive it came off"
    );

    // And with the diskette blank it falls through to the fixed disk, which is
    // the other half of the same rule: a diskette that is *in* the drive but
    // not bootable must not stop the boot.
    let (mut m, _cpu, _hosts) = board(vec![0u8; 1_474_560], fixed_disk(Some(FIXED_MARKER)));
    m.run_for(BOOTED).expect("the machine runs");
    assert_eq!(
        peek16(&m, u64::from(MARKER)),
        FIXED_MARKER,
        "a blank diskette stopped the boot instead of being declined"
    );
    assert_eq!(peek16(&m, u64::from(BOOT_DRIVE)), 0x0080);

    // And with **no diskette at all** — an empty drive, which is what a
    // machine that has just finished installing itself looks like. The
    // controller answers "not ready" rather than anything, and the bootstrap
    // has to treat that as "no boot record here" rather than waiting on it.
    let (mut m, _cpu, _hosts) = board(Vec::new(), fixed_disk(Some(FIXED_MARKER)));
    m.run_for(BOOTED).expect("the machine runs");
    assert_eq!(
        peek16(&m, u64::from(MARKER)),
        FIXED_MARKER,
        "an empty diskette drive stopped the boot"
    );
}

/// The EDD fixed-disk subset: the check, the parameter table, and a sector
/// out and back through the packet interface.
#[test]
fn the_edd_subset_answers_and_its_table_agrees_with_ah_08h() {
    let (mut m, _cpu, _hosts) = board(vec![0u8; 1_474_560], fixed_disk(Some(FIXED_MARKER)));
    m.run_for(BOOTED).expect("the machine runs");
    assert_eq!(peek16(&m, u64::from(MARKER)), FIXED_MARKER, "it booted");

    // AH=41h: version 1.1 in AH, the signature byte-swapped in BX, and the
    // fixed-disk subset claimed in CX bit 0.
    let (ax, bx, cx) = (
        peek16(&m, u64::from(EDD_AX)),
        peek16(&m, u64::from(EDD_BX)),
        peek16(&m, u64::from(EDD_CX)),
    );
    println!("pc-at edd: AH=41h -> ax={ax:#06x} bx={bx:#06x} cx={cx:#06x}");
    assert_eq!(ax >> 8, 0x21, "EDD 1.1");
    assert_eq!(bx, 0xaa55);
    assert_eq!(cx & 1, 1, "the fixed-disk access subset");

    // AH=48h: it answered, and its table says what AH=08h says. AH=08h packs
    // the cylinder count less one into CH and CL's top two bits, the sectors
    // per track into CL's low six, and the head count less one into DH.
    assert_eq!(
        peek16(&m, u64::from(PARAMS_CF)),
        0,
        "AH=48h came back with carry: this is what FDISK saw as `No fixed disks present`"
    );
    assert_eq!(
        peek16(&m, u64::from(PARAMS_AX)) >> 8,
        0,
        "and no error code"
    );
    assert_eq!(
        peek16(&m, u64::from(EDD_PARAMS)),
        0x1a,
        "the table's own size, which EDD 1.1 fixes at 1Ah"
    );
    assert_eq!(
        peek16(&m, u64::from(EDD_PARAMS + 2)) & 0x02,
        0x02,
        "the flag that says the geometry in it is valid"
    );
    let cylinders = peek32(&m, u64::from(EDD_PARAMS + 4));
    let heads = peek32(&m, u64::from(EDD_PARAMS + 8));
    let sectors = peek32(&m, u64::from(EDD_PARAMS + 12));
    let total = peek32(&m, u64::from(EDD_PARAMS + 16));
    let high = peek32(&m, u64::from(EDD_PARAMS + 20));
    let bytes = peek16(&m, u64::from(EDD_PARAMS + 24));
    println!(
        "pc-at edd: AH=48h -> {cylinders} cylinders, {heads} heads, {sectors} sectors, \
         {total} total, {bytes} bytes a sector"
    );
    let chs_cx = peek16(&m, u64::from(CHS_CX));
    let chs_dx = peek16(&m, u64::from(CHS_DX));
    let chs_cylinders = u32::from((chs_cx >> 8) & 0xff) | (u32::from((chs_cx & 0xc0) >> 6) << 8);
    assert_eq!(cylinders, chs_cylinders + 1, "AH=08h reports one less");
    assert_eq!(heads, u32::from((chs_dx >> 8) & 0xff) + 1);
    assert_eq!(sectors, u32::from(chs_cx & 0x3f));
    assert_eq!(bytes, 512);
    assert_eq!(high, 0, "the top half of a 64-bit sector count");
    assert_eq!(
        u64::from(total),
        HD_SECTORS,
        "the total is the drive's own addressable count, not the CHS product"
    );

    // AH=43h and AH=42h: the pattern went out through the packet interface and
    // came back, and it went to the LBA the packet named.
    assert_eq!(
        peek16(&m, u64::from(WRITE_AX)) >> 8,
        0,
        "the extended write"
    );
    assert_eq!(peek16(&m, u64::from(READ_AX)) >> 8, 0, "the extended read");
    let want = pattern();
    let back: Vec<u8> = (0..512)
        .map(|i| peek(&m, u64::from(READBACK) + i))
        .collect();
    assert_eq!(back, want, "what came back is not what went out");
}

/// The diskette's change line, swapped under a running guest.
#[test]
fn a_diskette_swap_raises_the_change_line_and_a_seek_clears_it() {
    let (mut m, _cpu, hosts) = board(diskette(DISKETTE_MARKER), fixed_disk(None));
    m.run_for(BOOTED).expect("the machine runs");
    assert_eq!(peek16(&m, u64::from(MARKER)), DISKETTE_MARKER, "it booted");
    assert!(peek16(&m, u64::from(LOOPS)) > 0, "the guest is polling");

    // AH=15h: type 2, a diskette drive *with* a change line. A firmware that
    // says type 1 tells a DOS to guess instead.
    assert_eq!(
        peek16(&m, u64::from(DRIVE_TYPE)),
        0x02,
        "the drive does not claim a change line"
    );

    // The firmware read the boot sector off this diskette, which is a seek
    // with a medium in the drive, so the line is down.
    assert_eq!(
        peek16(&m, u64::from(CHANGE)),
        0x00,
        "the line is active although nothing has been swapped"
    );

    // A person takes the diskette out and puts another one in.
    let drive = rsemu::dev::pc::fdc::drives::get(&hosts, "fd0")
        .expect("no other kind of host object claims the name")
        .expect("the controller filed itself in a drive")
        .clone();
    drive
        .insert("the second diskette", diskette(FIXED_MARKER))
        .expect("a 1.44 MB image");
    m.run_for(A_MOMENT).expect("the machine runs");
    assert_eq!(
        peek16(&m, u64::from(CHANGE)),
        0x06,
        "the guest was not told the diskette had been changed"
    );

    // And it stays active — the line is not a one-shot — until a read, whose
    // seek is what clears it.
    m.run_for(A_MOMENT).expect("the machine runs");
    assert_eq!(peek16(&m, u64::from(CHANGE)), 0x06, "still changed");

    poke16(&m, u64::from(READ_DISKETTE), 1);
    m.run_for(A_MOMENT).expect("the machine runs");
    assert_eq!(
        peek16(&m, u64::from(DISKETTE_AX)) >> 8,
        0,
        "the read of the new diskette failed"
    );
    assert_eq!(
        peek16(&m, u64::from(CHANGE)),
        0x00,
        "the seek did not clear the change line, so every later read would be \
         told the diskette had changed again"
    );

    // And what it read is the *new* diskette, byte for byte: the one that was
    // put in carries the other marker in its boot sector.
    let sector: Vec<u8> = (0..512)
        .map(|i| peek(&m, u64::from(DISKETTE_BUFFER) + i))
        .collect();
    assert_eq!(
        sector,
        boot_sector(FIXED_MARKER),
        "the drive answered with the diskette that was taken out"
    );
}
