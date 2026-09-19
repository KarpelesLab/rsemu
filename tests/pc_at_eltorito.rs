//! Booting the PC/AT board off a CD-ROM, on rsemu's own firmware.
//!
//! `tests/pc_at_cdrom.rs` proves the *drive*, with no firmware anywhere near
//! it. This proves the other half: that the El Torito boot record on a disc is
//! found, that the boot catalog it points at is walked, that the image it names
//! is loaded, and that what is loaded then runs and can still reach the disc it
//! came off.
//!
//! Every disc here is built by [`iso9660`] and written into a temporary
//! directory. **Nothing is vendored and nothing is fetched**, including the
//! thing that boots: the boot image is sixteen-bit code this file assembles
//! with `rsemu::fw::asm16`, the same assembler the firmware itself is written
//! in.
//!
//! # What each test claims
//!
//! * **No emulation.** The image is loaded at the catalog's load segment and
//!   entered with `DL` holding a drive number that works: the program prints a
//!   line, calls `INT 13h AH=4Bh` for the specification packet, reads its own
//!   disc's primary volume descriptor back through `AH=42h`, and prints what it
//!   found. Four claims in one program, because a boot that loads and then
//!   cannot read is a boot that cannot install anything.
//! * **Diskette emulation.** The image is a 1.44 MB diskette, it becomes
//!   `INT 13h` drive `00h`, and the program reads a sector out of the *middle*
//!   of it with an ordinary CHS `AH=02h` — which is the case that proves the
//!   four-virtual-sectors-to-a-logical-block arithmetic, because sector 1 alone
//!   would pass on an off-by-one.
//! * **The declines.** A catalog whose entry is not marked bootable, and one
//!   asking for hard-disk emulation, both fall through to `INT 18h` rather than
//!   running something.
//! * **The order.** A bootable diskette still wins over a bootable disc, which
//!   is what keeps an installer that reboots from running itself again.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "dev-ata-atapi",
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
use rsemu::fw::asm16::{AH, AL, AX, Alu, Asm, BX, CS, CX, Cc, DH, DL, DS, ES, Mem, SI, SP, SS};
use rsemu::machine::realize::Bindings;
use rsemu::machine::{Machine, build};

mod iso9660;

/// Where a boot image's code is assembled to run.
const BOOT_ADDRESS: u16 = 0x7c00;

/// The guest's scratch block: below the stack, above the BIOS Data Area.
const SCRATCH: u16 = 0x0500;
/// The drive number the firmware handed over in `DL`.
const BOOT_DRIVE: u16 = SCRATCH;
/// Whether `INT 13h AH=4Bh` answered, and what it said.
const SPEC_AX: u16 = SCRATCH + 2;
/// Whether the read-back succeeded, and its `AX`.
const READ_AX: u16 = SCRATCH + 4;

/// Where the nineteen-byte El Torito specification packet is asked for.
const SPEC_PACKET: u16 = 0x0600;
/// The sixteen-byte disk address packet `AH=42h` is given.
const DAP: u16 = 0x0640;
/// Where the block read back off the disc lands.
const READBACK: u16 = 0x0800;

/// What the one file on every disc here says.
const FILE_TEXT: &[u8] = b"rsemu el torito\n";

/// How many 512-byte virtual sectors the no-emulation image asks for. Four is
/// one whole logical block, which is what `mkisofs` writes and what every
/// no-emulation loader of the period was built to fit in.
const NO_EMULATION_SECTORS: u16 = 4;

// ---------------------------------------------------------------------------
// the boot images
// ---------------------------------------------------------------------------

/// Print the NUL-terminated string at `CS:SI` through `INT 10h AH=0Eh`.
///
/// The caller binds the entry label; this is only the body, so that the same
/// three boot images can each have their own `puts` at their own address.
fn emit_puts(a: &mut Asm) {
    let next = a.here_label();
    let done = a.label();
    a.mov8(AL, Mem::si(0).seg(CS));
    a.inc(SI);
    a.alui8(Alu::CMP, AL, 0);
    a.jcc(Cc::E, done);
    a.movi8(AH, 0x0e);
    a.movi(BX, 0x0007);
    a.int(0x10);
    a.jmp(next);
    a.bind(done);
    a.ret();
}

/// The no-emulation boot image.
///
/// Loaded at the catalog's load segment — `0x07c0` here, which is the
/// specification's default and puts the code at the same linear address a boot
/// sector occupies — and entered with `DL` holding the CD's drive number.
#[allow(clippy::too_many_lines)]
fn no_emulation_image() -> Vec<u8> {
    // Assembled at origin **zero**, not 7C00h, and that is the difference
    // between this image and a boot sector: El Torito loads it at the
    // catalog's load segment and enters it at `CS:0000`, so every `CS`-relative
    // reference in it — which is every string — is an offset from the start of
    // the image rather than from 7C00h. A copy of this file that assembled it
    // like a boot sector printed nothing and looked like a firmware bug.
    let mut a = Asm::new(2048, 0x00);

    let puts = a.label();
    let hello = a.label();
    let got_spec = a.label();
    let got_block = a.label();
    let parked = a.label();

    // The load segment is 07C0h and `CS` is therefore 07C0h, so every offset
    // here is relative to 7C00h only because the assembler was told to put it
    // there — the code itself runs at `CS:0000`. `jmp` past the data first, so
    // that the very first byte executed is an instruction whatever a loader
    // decides.
    a.cli();
    a.cld();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT_ADDRESS);
    a.sti();
    // The one register a boot image is entitled to find anything in.
    a.movto8(Mem::abs(BOOT_DRIVE), DL);
    a.movmi8(Mem::abs(BOOT_DRIVE + 1), 0);

    a.movi_label(SI, hello);
    a.call(puts);

    // -- INT 13h AH=4Bh, get the emulation status ----------------------------
    //
    // AL=01h asks for the status without terminating anything. What comes back
    // is the nineteen-byte specification packet: the media type, the drive
    // number, and the logical block the image was loaded from.
    a.movi(AX, 0x4b01);
    a.mov8(DL, Mem::abs(BOOT_DRIVE));
    a.movi(SI, SPEC_PACKET);
    a.int(0x13);
    a.movto(Mem::abs(SPEC_AX), AX);
    a.jcc(Cc::AE, got_spec);
    a.movmi(Mem::abs(SPEC_AX), 0xffff);
    a.bind(got_spec);

    // -- INT 13h AH=42h, read logical block 16 back --------------------------
    //
    // The primary volume descriptor, off the disc this program came from. A
    // no-emulation boot that cannot read its own disc has loaded a program with
    // nothing to do, which is the whole failure mode this asserts against.
    a.movmi8(Mem::abs(DAP), 0x10); // packet size
    a.movmi8(Mem::abs(DAP + 1), 0x00);
    a.movmi(Mem::abs(DAP + 2), 1); // one block
    a.movmi(Mem::abs(DAP + 4), READBACK); // buffer offset
    a.movmi(Mem::abs(DAP + 6), 0x0000); // buffer segment
    a.movmi32(Mem::abs(DAP + 8), 16); // the LBA, 64 bits of it
    a.movmi32(Mem::abs(DAP + 12), 0);
    a.movi(AX, 0x4200);
    a.mov8(DL, Mem::abs(BOOT_DRIVE));
    a.movi(SI, DAP);
    a.int(0x13);
    a.movto(Mem::abs(READ_AX), AX);
    a.jcc(Cc::AE, got_block);
    a.movmi(Mem::abs(READ_AX), 0xffff);
    a.bind(got_block);

    // Print the volume identifier out of what came back, which is the proof
    // that the bytes are the disc's rather than whatever was in memory.
    a.movi(SI, READBACK + 40);
    a.movi(CX, 10);
    let v_next = a.here_label();
    a.mov8(AL, Mem::si(0));
    a.inc(SI);
    a.movi8(AH, 0x0e);
    a.movi(BX, 0x0007);
    a.push(CX);
    a.push(SI);
    a.int(0x10);
    a.pop(SI);
    a.pop(CX);
    a.loop_(v_next);
    a.movi8(AL, b'\r');
    a.movi8(AH, 0x0e);
    a.int(0x10);
    a.movi8(AL, b'\n');
    a.movi8(AH, 0x0e);
    a.int(0x10);

    a.bind(parked);
    a.hlt();
    a.jmp(parked);

    a.bind(puts);
    emit_puts(&mut a);

    a.bind(hello);
    a.db(b"NOEMU BOOTED\r\n\0");

    a.finish()
}

/// A 1.44 MB diskette image whose boot sector reads a sector out of the middle
/// of the same image and prints what it found.
#[allow(clippy::too_many_lines)]
fn diskette_image() -> Vec<u8> {
    // The sector the boot sector reads back: cylinder 0, head 1, sector 5.
    // Against 18 sectors per track and two heads that is virtual sector
    // 18 + 4 = 22, which is the sixth quarter of the sixth logical block — so
    // an implementation that read the right block and the wrong quarter, or the
    // right quarter of the wrong block, fails here and cannot fail silently.
    const MARK_HEAD: u8 = 1;
    const MARK_SECTOR: u8 = 5;
    const MARK_LBA: usize = 18 + 4;

    let mut a = Asm::new(usize::from(BOOT_ADDRESS) + 512, 0x00);
    a.seek(BOOT_ADDRESS);
    let puts = a.label();
    let hello = a.label();
    let failed = a.label();
    let parked = a.label();

    a.cli();
    a.cld();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT_ADDRESS);
    a.sti();
    a.movto8(Mem::abs(BOOT_DRIVE), DL);
    a.movmi8(Mem::abs(BOOT_DRIVE + 1), 0);

    a.movi_label(SI, hello);
    a.call(puts);

    // AH=02h, one sector, cylinder 0 / head 1 / sector 5, into READBACK.
    a.movi(AX, 0x0201);
    a.movi(CX, u16::from(MARK_SECTOR)); // CH = cylinder 0, CL = sector
    a.movi8(DH, MARK_HEAD);
    a.mov8(DL, Mem::abs(BOOT_DRIVE));
    a.movi(BX, READBACK);
    a.int(0x13);
    a.movto(Mem::abs(READ_AX), AX);
    a.jcc(Cc::B, failed);
    // Print the sector's own text, which says which sector it is.
    a.movi(SI, READBACK);
    a.call(puts);
    a.jmp(parked);

    a.bind(failed);
    a.movmi(Mem::abs(READ_AX), 0xffff);
    a.bind(parked);
    a.hlt();
    a.jmp(parked);

    a.bind(puts);
    emit_puts(&mut a);
    a.bind(hello);
    a.db(b"FLOPPY BOOTED\r\n\0");

    let code = a.finish();
    let mut image = vec![0u8; 1_474_560];
    image[..512].copy_from_slice(&code[usize::from(BOOT_ADDRESS)..usize::from(BOOT_ADDRESS) + 512]);
    image[510] = 0x55;
    image[511] = 0xaa;
    // The sector the boot sector goes looking for.
    let text = b"SECTOR 0/1/5\r\n\0";
    image[MARK_LBA * 512..MARK_LBA * 512 + text.len()].copy_from_slice(text);
    image
}

/// A diskette that boots and says so, for the ordering test.
fn plain_diskette() -> Vec<u8> {
    let mut a = Asm::new(usize::from(BOOT_ADDRESS) + 512, 0x00);
    a.seek(BOOT_ADDRESS);
    let puts = a.label();
    let hello = a.label();
    let parked = a.label();
    a.cli();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT_ADDRESS);
    a.sti();
    a.movi_label(SI, hello);
    a.call(puts);
    a.bind(parked);
    a.hlt();
    a.jmp(parked);
    a.bind(puts);
    emit_puts(&mut a);
    a.bind(hello);
    a.db(b"REAL DISKETTE\r\n\0");
    let code = a.finish();
    let mut image = vec![0u8; 1_474_560];
    image[..512].copy_from_slice(&code[usize::from(BOOT_ADDRESS)..usize::from(BOOT_ADDRESS) + 512]);
    image[510] = 0x55;
    image[511] = 0xaa;
    image
}

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

fn bindings(cpus: &Arc<Captured<X86>>) -> Bindings {
    let mut b = Bindings::new();
    rsemu::machine::builtin::bind(&mut b).expect("ram and rom");
    rsemu::dev::pc::bind(&mut b).expect("the chipset");
    rsemu::dev::ata::bind(&mut b).expect("the drives");
    let kept = Arc::clone(cpus);
    b.bind("cpu.x86", move |props| {
        let cpu = Arc::new(X86::from_props_defaulting(props, Variant::I80486)?);
        kept.push(&cpu);
        Ok(cpu)
    })
    .expect("nothing else in this table claims the name");
    b
}

/// The shipped board with rsemu's own firmware, `disc` in the CD-ROM drive and
/// `floppy` in the diskette drive.
fn board(disc: Vec<u8>, floppy: Vec<u8>) -> (Machine, Arc<X86>) {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(bindings(&cpus));
    options
        .realize
        .media
        .insert("bios", rsemu::fw::pcbios::image());
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("floppy", floppy);
    options.realize.media.insert("hd0", Vec::new());
    options.realize.media.insert("hd1", Vec::new());
    options.realize.media.insert("cdrom", disc);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let cpu = cpus.take().expect("the constructor kept a handle");
    m.reset(ResetKind::Cold);
    m.sweep();
    (m, cpu)
}

fn peek(m: &Machine, addr: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEBUG)
        .unwrap_or(0xff) as u8
}

fn peek16(m: &Machine, addr: u16) -> u16 {
    u16::from(peek(m, u64::from(addr))) | (u16::from(peek(m, u64::from(addr) + 1)) << 8)
}

/// The colour text page, as lines of characters.
fn text_page(m: &Machine) -> Vec<String> {
    (0..25u64)
        .map(|row| {
            (0..80u64)
                .map(|col| {
                    let ch = peek(m, 0xb8000 + (row * 80 + col) * 2);
                    match ch {
                        0x20..=0x7e => ch as char,
                        _ => ' ',
                    }
                })
                .collect::<String>()
                .trim_end()
                .to_string()
        })
        .collect()
}

/// Whether the screen holds a line that is exactly `want`.
fn says(m: &Machine, want: &str) -> bool {
    text_page(m).iter().any(|line| line.trim() == want)
}

fn show(m: &Machine) -> String {
    text_page(m)
        .iter()
        .filter(|line| !line.is_empty())
        .map(|line| format!("  |{line}|"))
        .collect::<Vec<_>>()
        .join("\n")
}

/// Long enough for POST, the three boot attempts and the loaded program.
const BOOTED: GlobalTime = GlobalTime::from_nanos(400_000_000);

/// A disc, through a temporary file, so that what boots is what a host would
/// have handed the machine.
fn disc(name: &str, image: Vec<u8>) -> Vec<u8> {
    let path = iso9660::write_temp(name, &image);
    let back = std::fs::read(&path).expect("the disc image we just wrote");
    assert_eq!(back, image, "the image did not survive the round trip");
    let _ = std::fs::remove_file(&path);
    back
}

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

#[test]
fn a_no_emulation_disc_boots_and_can_read_itself() {
    let image = iso9660::bootable_disc(
        FILE_TEXT,
        &no_emulation_image(),
        iso9660::Emulation::None {
            // Zero, which El Torito says means 07C0h — the case worth taking,
            // because a firmware that loaded at segment zero would put the
            // image over the interrupt vector table.
            load_segment: 0,
            sectors: NO_EMULATION_SECTORS,
        },
        true,
    );
    let (mut m, _cpu) = board(disc("noemu", image), vec![0u8; 1_474_560]);
    m.run_for(BOOTED).expect("the machine runs");

    assert!(
        says(&m, "Booting from CD-ROM"),
        "the firmware never announced a CD boot:\n{}",
        show(&m)
    );
    assert!(
        says(&m, "NOEMU BOOTED"),
        "the boot image did not run:\n{}",
        show(&m)
    );

    // `DL` is the drive number the firmware assigned. There is no hard disk on
    // this board, so it is 0x80 — the first number a fixed disk would have had.
    assert_eq!(
        peek(&m, u64::from(BOOT_DRIVE)),
        0x80,
        "the wrong drive number reached the boot image"
    );

    // AH=4Bh answered, and the packet describes this disc.
    assert_eq!(
        peek16(&m, SPEC_AX) & 0xff00,
        0x0000,
        "INT 13h AH=4Bh failed: AX={:#06x}",
        peek16(&m, SPEC_AX)
    );
    assert_eq!(peek(&m, u64::from(SPEC_PACKET)), 0x13, "the packet's size");
    assert_eq!(peek(&m, u64::from(SPEC_PACKET + 1)), 0x00, "no emulation");
    assert_eq!(peek(&m, u64::from(SPEC_PACKET + 2)), 0x80, "the drive");
    let image_lba =
        u32::from(peek16(&m, SPEC_PACKET + 4)) | (u32::from(peek16(&m, SPEC_PACKET + 6)) << 16);
    assert_eq!(
        image_lba,
        iso9660::BOOT_IMAGE_BLOCK,
        "the packet points at the wrong block"
    );
    assert_eq!(
        peek16(&m, SPEC_PACKET + 0x0c),
        0x07c0,
        "the load segment the firmware substituted for zero"
    );
    assert_eq!(peek16(&m, SPEC_PACKET + 0x0e), NO_EMULATION_SECTORS);

    // And the program read its own disc back through the extensions.
    assert_eq!(
        peek16(&m, READ_AX) & 0xff00,
        0x0000,
        "INT 13h AH=42h failed: AX={:#06x}",
        peek16(&m, READ_AX)
    );
    assert!(
        says(&m, iso9660::VOLUME_ID),
        "the volume identifier the program read off its own disc is not on the screen:\n{}",
        show(&m)
    );
}

#[test]
fn a_diskette_emulation_disc_boots_as_drive_zero() {
    let image = iso9660::bootable_disc(
        FILE_TEXT,
        &diskette_image(),
        iso9660::Emulation::Diskette144,
        true,
    );
    // No diskette in the real drive, which is the ordinary case: the emulated
    // one is A: and there is nothing for it to displace.
    let (mut m, _cpu) = board(disc("floppy", image), Vec::new());
    m.run_for(BOOTED).expect("the machine runs");

    assert!(
        says(&m, "Booting from CD-ROM"),
        "the firmware never announced a CD boot:\n{}",
        show(&m)
    );
    assert!(
        says(&m, "FLOPPY BOOTED"),
        "the emulated diskette's boot sector did not run:\n{}",
        show(&m)
    );
    assert_eq!(
        peek(&m, u64::from(BOOT_DRIVE)),
        0x00,
        "an emulated diskette is drive 00h"
    );
    assert_eq!(
        peek16(&m, READ_AX) & 0xff00,
        0x0000,
        "the CHS read off the emulated diskette failed: AX={:#06x}",
        peek16(&m, READ_AX)
    );
    assert!(
        says(&m, "SECTOR 0/1/5"),
        "the sector that came back is not the one that was asked for:\n{}",
        show(&m)
    );
}

#[test]
fn a_catalog_that_is_not_bootable_is_declined() {
    let image = iso9660::bootable_disc(
        FILE_TEXT,
        &no_emulation_image(),
        iso9660::Emulation::None {
            load_segment: 0,
            sectors: NO_EMULATION_SECTORS,
        },
        false,
    );
    let (mut m, _cpu) = board(disc("notboot", image), Vec::new());
    m.run_for(BOOTED).expect("the machine runs");
    assert!(
        !says(&m, "NOEMU BOOTED"),
        "an entry marked not bootable was booted anyway:\n{}",
        show(&m)
    );
    assert!(
        says(&m, "No bootable device."),
        "the board did not fall through to INT 18h:\n{}",
        show(&m)
    );
}

#[test]
fn hard_disk_emulation_is_declined_rather_than_half_done() {
    // Media type 4. Loading the image and then having no `INT 13h` drive 80h
    // behind it would be worse than declining, which is the whole of the
    // argument `src/fw/pcbios/cdrom.rs` makes for leaving it out.
    let image = iso9660::bootable_disc(
        FILE_TEXT,
        &no_emulation_image(),
        iso9660::Emulation::HardDisk,
        true,
    );
    let (mut m, _cpu) = board(disc("harddisk", image), Vec::new());
    m.run_for(BOOTED).expect("the machine runs");
    assert!(
        says(&m, "No bootable device."),
        "a media type the firmware does not implement was not declined:\n{}",
        show(&m)
    );
}

#[test]
fn a_bootable_diskette_still_wins_over_a_bootable_disc() {
    // The order is load-bearing and the CD-ROM is last. An installer running
    // off a disc writes a boot record to the disk it is installing onto and
    // reboots; a CD ahead of the other two would run the installer again for
    // ever.
    let image = iso9660::bootable_disc(
        FILE_TEXT,
        &no_emulation_image(),
        iso9660::Emulation::None {
            load_segment: 0,
            sectors: NO_EMULATION_SECTORS,
        },
        true,
    );
    let (mut m, _cpu) = board(disc("order", image), plain_diskette());
    m.run_for(BOOTED).expect("the machine runs");
    assert!(
        says(&m, "REAL DISKETTE"),
        "the diskette in the drive did not boot:\n{}",
        show(&m)
    );
    assert!(
        !says(&m, "NOEMU BOOTED"),
        "the disc booted although a bootable diskette was in the drive:\n{}",
        show(&m)
    );
}

#[test]
fn a_drive_with_no_disc_in_it_does_not_stop_the_boot() {
    // The empty-tray case, which every board in the tree now has by default:
    // the firmware finds the drive, asks it for a boot record, is told there is
    // no medium, and carries on to say so.
    let (mut m, _cpu) = board(Vec::new(), Vec::new());
    m.run_for(BOOTED).expect("the machine runs");
    assert!(
        says(&m, "No bootable device."),
        "an empty CD-ROM drive wedged the bootstrap:\n{}",
        show(&m)
    );
}
