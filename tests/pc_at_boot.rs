//! The `pc-at` board booting on **rsemu's own firmware**, with nothing supplied
//! from outside.
//!
//! `tests/pc_at_firmware.rs` runs the *user's* BIOS image and is gated on
//! `RSEMU_BIOS`, so it skips in an ordinary `cargo test`. This is the other
//! half and the one that answers `ROADMAP.md` phase 6a: [`rsemu::fw::pcbios`]
//! assembles a 64 KiB legacy BIOS out of this repository, the board is built
//! with it in the `bios` socket, and a guest boots off the IDE drive. Nothing
//! is downloaded, nothing is vendored, and no environment variable turns it on.
//!
//! # What is being claimed
//!
//! Each assertion is a step of a real boot, in order, and every one of them was
//! false before the firmware existed:
//!
//! 1. The processor fetches `0xfffffff0`, takes the far jump and runs POST.
//! 2. POST fills the BIOS Data Area from the CMOS and the hardware.
//! 3. It finds the drive with `IDENTIFY DEVICE` and records its geometry.
//! 4. `INT 19h` reads cylinder 0, head 0, sector 1 into `0000:7c00`, sees the
//!    `0x55 0xAA` signature, and jumps there.
//! 5. **The boot sector runs** — and it is a real guest program that calls back
//!    into the firmware: `INT 10h` to print, `INT 11h`, `INT 12h`, `INT 13h`
//!    for the geometry *and* for a second read of its own, and `INT 15h` for
//!    the `E820` map. Everything it learns it leaves in low memory, where this
//!    test reads it.
//! 6. It **writes** — `INT 13h AH=03h` to the diskette and to the fixed disk,
//!    each read back afterwards. The fixed disk's is checked against the
//!    drive's own medium through `ata::bays`, which is the standard
//!    `tests/pc_at_ide.rs` sets.
//! 7. And it moves a block out to 8 MiB and back with `INT 15h AH=87h`, the
//!    one service that reaches above the first megabyte from real mode.
//!
//! The boot sector is assembled by [`rsemu::fw::asm16`], the same assembler the
//! firmware is built with — so the test guest is written the same way the
//! firmware is, and the assembler is exercised by something other than its own
//! unit tests.
//!
//! # And one that is not hermetic, on purpose
//!
//! [`freedos_boots_on_rsemus_own_firmware`] is `ROADMAP.md` phase 6a's gate. It
//! needs a FreeDOS boot diskette, which this repository will never contain, so
//! it is gated on `RSEMU_FREEDOS_FLOPPY` and skips cleanly without it — exactly
//! as `tests/pc_at_firmware.rs` is gated on `RSEMU_BIOS`.

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
use rsemu::fw::asm16::{AH, AL, AX, Alu, Asm, BX, CX, Cc, DH, DI, DL, DS, DX, ES, Mem, SI, SP, SS};
use rsemu::machine::Machine;
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

/// How big the test disk is, in sectors. Four cylinders of sixteen heads of 63
/// sectors is what `ata.disk`'s default translation covers, and the number is
/// the *product* rather than a round one on purpose: a disk whose size is not a
/// whole number of cylinders reports a geometry that does not cover it, and
/// then "the geometry the firmware read" and "the disk" are two different
/// claims that a test cannot tell apart.
const SECTORS: u64 = 4 * 16 * 63;

/// Where the boot sector lands, and therefore what its labels are relative to.
const BOOT_ADDRESS: u16 = 0x7c00;

/// Where the boot sector leaves what it learned. Below the stack, above the
/// BIOS Data Area, and inside the region every PC has left free for exactly
/// this since 1981.
const SCRATCH: u16 = 0x0500;

/// Where it asks `INT 15h` to put the first `E820` entry.
const E820_BUFFER: u16 = 0x0600;

/// Where it reads a second sector of its own.
const SECOND_SECTOR: u16 = 0x0800;

/// Where the boot sector builds the 512 bytes it writes to the diskette, and
/// then hands to `INT 15h AH=87h`.
const PATTERN: u16 = 0x0a00;

/// Where the sector it wrote is read back to.
const READBACK: u16 = 0x0c00;

/// Where `INT 15h AH=87h` brings the pattern back down to.
const XFER_BACK: u16 = 0x0e00;

/// Where the sector written to the *fixed disk* is read back to. Not
/// `XFER_BACK + 512`: the word immediately past the block move's landing zone
/// is the sentinel that says the move did not overrun, so it has to stay
/// untouched.
const HD_READBACK: u16 = 0x1200;

/// The one-based sector on cylinder 0, head 0 the guest writes to the fixed
/// disk. Sector 1 is the boot sector, so this is the fourth — and with sixteen
/// heads of 63 sectors it is LBA 3, which is [`HD_WRITE_LBA`].
const HD_WRITE_SECTOR: u8 = 4;

/// The LBA `INT 13h`'s CHS translation turns that into:
/// `(C x heads + H) x sectors + (S - 1)`, with C and H zero.
const HD_WRITE_LBA: u64 = HD_WRITE_SECTOR as u64 - 1;

/// The first word of the pattern; word `k` is this plus `k`. Not a constant
/// fill, so a transfer that moved the right number of bytes from the wrong
/// place fails instead of passing.
const PATTERN_SEED: u16 = 0x1234;

/// Where `INT 15h AH=87h` puts the pattern: 8 MiB, which is inside the 15 MiB
/// of extended memory the board fits and unreachable from real mode by any
/// other means. That is the whole point of the function.
const EXT_TARGET: u32 = 0x0080_0000;

/// The cylinder the guest writes a diskette sector to. Well away from track
/// zero, so the boot sector it came off is not the thing being overwritten.
const WRITE_CYLINDER: u8 = 5;
/// The head it writes to — the second one, because a transfer that ignored the
/// head would still land on a legal sector and pass.
const WRITE_HEAD: u8 = 1;
/// The one-based sector within that track.
const WRITE_SECTOR: u8 = 7;

/// The LBA the three above name on a 1.44 MB diskette: eighteen sectors per
/// track, two heads.
const WRITE_LBA: u64 =
    ((WRITE_CYLINDER as u64 * 2) + WRITE_HEAD as u64) * 18 + (WRITE_SECTOR as u64 - 1);

/// The word the boot sector writes last, so "it started" and "it finished" are
/// two different claims.
const DONE_MARKER: u16 = 0x600d;

/// One `INT 15h AH=87h` segment descriptor over a 64 KiB window at `base`.
///
/// The layout is the ordinary 80386 one — limit 15:0, base 23:0, the access
/// byte, then limit 19:16 with the flags and base 31:24 (Intel SDM Vol. 3A
/// §3.4.5). `0x93` is present, ring 0, a data segment that is writable and
/// already accessed, which is what the interface expects of the two the caller
/// fills in.
fn descriptor(base: u32) -> [u8; 8] {
    [
        0xff,
        0xff,
        base as u8,
        (base >> 8) as u8,
        (base >> 16) as u8,
        0x93,
        0x00,
        (base >> 24) as u8,
    ]
}

/// The 512 bytes the boot sector generates, writes and copies about.
fn pattern() -> Vec<u8> {
    (0..256u16)
        .flat_map(|k| PATTERN_SEED.wrapping_add(k).to_le_bytes())
        .collect()
}

/// What the boot sector prints.
const GREETING: &str = "rsemu boot sector on rsemu BIOS";

// ---------------------------------------------------------------------------
// the guest
// ---------------------------------------------------------------------------

/// Assemble the boot sector.
///
/// Assembled into an image that starts at zero and taken from `0x7c00`, so the
/// assembler's absolute labels are the addresses the sector actually runs at —
/// a sector assembled at origin zero and loaded at `0x7c00` would jump into the
/// interrupt vector table.
fn boot_sector(echo: bool) -> Vec<u8> {
    let mut a = Asm::new(usize::from(BOOT_ADDRESS) + 512, 0x00);
    a.seek(BOOT_ADDRESS);

    let message = a.label();
    let spin = a.label();

    // The firmware leaves DL holding the drive the sector came off, and nothing
    // else is promised. Segments and a stack first, because nothing else is.
    a.cli();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT_ADDRESS);
    a.sti();
    a.movto8(Mem::abs(SCRATCH), DL);

    // INT 10h AH=0Eh, one character at a time.
    a.movi_label(SI, message);
    let next = a.here_label();
    let printed = a.label();
    a.mov8(AL, Mem::si(0));
    a.inc(SI);
    a.alui8(Alu::CMP, AL, 0);
    a.jcc(Cc::E, printed);
    a.movi8(AH, 0x0e);
    a.movi(BX, 0x0007);
    a.int(0x10);
    a.jmp(next);
    a.bind(printed);

    // INT 12h: base memory in kilobytes.
    a.int(0x12);
    a.movto(Mem::abs(SCRATCH + 2), AX);

    // INT 11h: the equipment word.
    a.int(0x11);
    a.movto(Mem::abs(SCRATCH + 4), AX);

    // INT 13h AH=08h: the drive's geometry, as the firmware read it out of
    // `IDENTIFY DEVICE`.
    a.movi8(AH, 0x08);
    a.mov8(DL, Mem::abs(SCRATCH));
    a.int(0x13);
    a.movto(Mem::abs(SCRATCH + 6), CX);
    a.movto(Mem::abs(SCRATCH + 8), DX);

    // INT 15h AX=E820h: the first entry of the memory map.
    a.movi(AX, 0xe820);
    a.movi32(DX, 0x534d_4150);
    a.movi32(CX, 20);
    a.movi32(BX, 0);
    a.movi(DI, E820_BUFFER);
    a.int(0x15);
    a.movto(Mem::abs(SCRATCH + 10), BX);

    // INT 13h AH=02h: a read of its own, of the sector after this one. This is
    // the step that proves the firmware's disk service is usable by a guest
    // rather than only by its own bootstrap.
    a.movi(AX, 0x0201);
    a.movi(CX, 0x0002); // cylinder 0, sector 2 — the second sector of track 0
    a.movi8(DH, 0x00);
    a.mov8(DL, Mem::abs(SCRATCH));
    a.movi(BX, SECOND_SECTOR);
    a.int(0x13);
    a.movto(Mem::abs(SCRATCH + 12), AX);

    // -- the diskette, written and read back ---------------------------------
    //
    // A pattern generated here rather than carried as data, so 512 bytes of the
    // sector do not have to fit in the sector. Word `k` is `PATTERN_SEED + k`,
    // which the test recomputes.
    a.movi(DI, PATTERN);
    a.movi(CX, 256);
    a.movi(AX, PATTERN_SEED);
    let fill = a.here_label();
    a.stosw();
    a.inc(AX);
    a.dec(CX);
    a.jcc(Cc::NE, fill);

    // `INT 13h AH=03h`, one sector, to the cylinder/head/sector below. Always
    // drive 0 — the diskette exists in both board configurations, and in the
    // one that boots off the fixed disk this is the only thing that touches it.
    a.movi(AX, 0x0301);
    a.movi(CX, u16::from_be_bytes([WRITE_CYLINDER, WRITE_SECTOR]));
    a.movi(DX, u16::from_be_bytes([WRITE_HEAD, 0x00]));
    a.movi(BX, PATTERN);
    a.int(0x13);
    a.movto(Mem::abs(SCRATCH + 16), AX);

    // And straight back off the medium into a different buffer. This is the
    // check that matters: `pc.fdc` refills its sector buffer from the image on
    // every command, so a read that answers with the pattern is the image
    // answering, not the buffer the write left behind.
    a.movi(AX, 0x0201);
    a.movi(CX, u16::from_be_bytes([WRITE_CYLINDER, WRITE_SECTOR]));
    a.movi(DX, u16::from_be_bytes([WRITE_HEAD, 0x00]));
    a.movi(BX, READBACK);
    a.int(0x13);
    a.movto(Mem::abs(SCRATCH + 18), AX);

    // -- INT 15h AH=87h, out to extended memory and back ---------------------
    //
    // The one BIOS service that reaches above the first megabyte from real
    // mode. The pattern goes to `EXT_TARGET`, then comes back to a third
    // buffer, so a call that quietly copied nothing leaves that buffer holding
    // what it held before and the test sees it.
    let gdt = a.label();
    a.movi(AX, 0x8700);
    a.movi(CX, 256); // words, not bytes
    a.movi_label(SI, gdt);
    a.int(0x15);
    a.movto(Mem::abs(SCRATCH + 20), AX);

    // The same table with the two bases swapped end for end: the descriptors
    // are patched in place rather than duplicated, because a boot sector has
    // 512 bytes and two of these would not fit beside the code.
    a.movi_label(SI, gdt);
    a.movmi(Mem::si(0x12), (EXT_TARGET & 0xffff) as u16);
    a.movmi8(Mem::si(0x14), ((EXT_TARGET >> 16) & 0xff) as u8);
    a.movmi(Mem::si(0x1a), XFER_BACK);
    a.movmi8(Mem::si(0x1c), 0x00);
    a.movi(AX, 0x8700);
    a.movi(CX, 256);
    a.int(0x15);
    a.movto(Mem::abs(SCRATCH + 22), AX);

    // -- and the same on the fixed disk --------------------------------------
    //
    // `INT 13h AH=03h` with `DL = 0x80` goes down the ATA path instead: a CHS
    // triple translated to an LBA, `WRITE SECTOR(S)` into the command block and
    // `REP OUTSW` through the data port. On the board that boots off the
    // diskette there is no fixed disk and the firmware answers with carry,
    // which is a claim of its own and the test checks that instead.
    a.movi(AX, 0x0301);
    a.movi(CX, u16::from_be_bytes([0x00, HD_WRITE_SECTOR]));
    a.movi(DX, 0x0080);
    a.movi(BX, PATTERN);
    a.int(0x13);
    a.movto(Mem::abs(SCRATCH + 24), AX);

    a.movi(AX, 0x0201);
    a.movi(CX, u16::from_be_bytes([0x00, HD_WRITE_SECTOR]));
    a.movi(DX, 0x0080);
    a.movi(BX, HD_READBACK);
    a.int(0x13);
    a.movto(Mem::abs(SCRATCH + 26), AX);

    a.movmi(Mem::abs(SCRATCH + 14), DONE_MARKER);

    // The keyboard variant does not stop: it waits on `INT 16h` and echoes what
    // comes back through `INT 10h`, which is the whole path from a scan code on
    // the 8042's wire to a character on the text page.
    if echo {
        let key = a.here_label();
        a.movi8(AH, 0x00);
        a.int(0x16);
        a.movi8(AH, 0x0e);
        a.movi(BX, 0x0007);
        a.int(0x10);
        a.jmp(key);
    }

    a.bind(spin);
    a.hlt();
    a.jmp(spin);

    a.bind(message);
    a.db(GREETING.as_bytes());
    a.db(&[0x0d, 0x0a, 0x00]);

    // `INT 15h AH=87h`'s descriptor table: six eight-byte descriptors, of which
    // the caller fills in two. Entry 1 is the table's own descriptor and
    // entries 4 and 5 the BIOS code and stack segments — all three are the
    // firmware's to fill, and are left zero here precisely so that a firmware
    // which needed them would fail rather than appear to work (RBIL,
    // `INT 15h AH=87h`).
    a.bind(gdt);
    a.db(&[0u8; 16]);
    a.db(&descriptor(u32::from(PATTERN)));
    a.db(&descriptor(EXT_TARGET));
    a.db(&[0u8; 16]);

    // Nothing may have grown past the signature. The assembler would let it:
    // `seek` moves the cursor and the two bytes below would land on top of
    // whatever was there.
    assert!(
        a.here() <= BOOT_ADDRESS + 510,
        "the boot sector is {} bytes of code and data, and 510 is the whole of \
         what a sector has",
        a.here() - BOOT_ADDRESS
    );

    // The signature, which is the whole of what makes a sector bootable.
    a.seek(BOOT_ADDRESS + 510);
    a.db(&[0x55, 0xaa]);

    let image = a.finish();
    image[usize::from(BOOT_ADDRESS)..].to_vec()
}

/// What sector `lba` holds, apart from the first.
///
/// Every sector says which sector it is, so a read that lands one sector out
/// fails instead of passing on identical zeroes.
fn stamp(lba: u64) -> Vec<u8> {
    let mut out = vec![0u8; 512];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = (lba as u8) ^ (i as u8) ^ 0x5a;
    }
    out[0] = lba as u8;
    out[1] = (lba >> 8) as u8;
    out
}

/// A disk with the boot sector at LBA 0 and a stamp everywhere else.
fn disk_image(echo: bool) -> Vec<u8> {
    let mut out = boot_sector(echo);
    assert_eq!(out.len(), 512, "a boot sector is one sector");
    for lba in 1..SECTORS {
        out.extend_from_slice(&stamp(lba));
    }
    out
}

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// The `pc-at` board with rsemu's own BIOS in its socket and the test disk in
/// bay 0.
fn board(echo: bool) -> (Machine, Arc<X86>, Arc<rsemu::core::hosts::HostObjects>) {
    board_from(echo, None)
}

/// The same board with a diskette image supplied and both IDE bays empty — so
/// `INT 19h` tries the fixed disk, finds none, and falls through to the µPD765.
///
/// `floppy` is `None` for the fixed-disk boot, where the diskette is blank and
/// exists only so `INT 13h` has a second drive to talk to.
fn board_from(
    echo: bool,
    floppy: Option<Vec<u8>>,
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
    // An empty option-ROM socket by default: 64 KiB of zeroes has no
    // `0x55 0xAA`, which is exactly what the firmware's scan must survive.
    //
    // `RSEMU_VGABIOS` puts a real one in the **legacy** socket at `0xc0000`,
    // which is the path rsemu's own firmware takes — it does not enumerate PCI,
    // so the expansion-ROM BAR on `pc.vga-pci` is not how it finds video. That
    // makes the *ISA* build the right image here (`vgabios.bin` on a machine
    // with QEMU's, not `vgabios-stdvga.bin`), which is the opposite of what a
    // 440FX-era firmware wants. Nothing is vendored and the variable is
    // optional, so `cargo test` stays hermetic.
    options.realize.media.insert(
        "vgabios",
        std::env::var("RSEMU_VGABIOS")
            .ok()
            .map(|p| std::fs::read(&p).unwrap_or_else(|e| panic!("{p}: {e}")))
            .unwrap_or_default(),
    );
    if let Some(mut image) = floppy {
        // 1.44 MB: 80 cylinders of two heads of eighteen sectors, which is what
        // `pc.fdc` infers from the length and what the firmware's CMOS-driven
        // geometry expects.
        image.resize(1_474_560, 0);
        options.realize.media.insert("floppy", image);
        options.realize.media.insert("hd0", Vec::new());
    } else {
        options.realize.media.insert("floppy", vec![0u8; 1_474_560]);
        options.realize.media.insert("hd0", disk_image(echo));
    }
    options.realize.media.insert("hd1", Vec::new());

    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options)
        .unwrap_or_else(|e| panic!("the board does not realize: {e}"));
    let cpu = cpus.take().expect("the constructor kept a handle");
    m.reset(ResetKind::Cold);
    m.sweep();
    (m, cpu, options.realize.hosts)
}

/// One byte of guest memory, read as a debugger reads.
fn peek(m: &Machine, addr: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEBUG)
        .unwrap_or(0xff) as u8
}

/// A word of guest memory.
fn peek16(m: &Machine, addr: u64) -> u16 {
    u16::from(peek(m, addr)) | (u16::from(peek(m, addr + 1)) << 8)
}

/// A dword of guest memory.
fn peek32(m: &Machine, addr: u64) -> u32 {
    u32::from(peek16(m, addr)) | (u32::from(peek16(m, addr + 2)) << 16)
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

// ---------------------------------------------------------------------------
// the test
// ---------------------------------------------------------------------------

#[test]
fn the_board_boots_a_guest_on_rsemus_own_firmware() {
    let (mut m, cpu, hosts) = board(false);

    // One virtual second. POST and the boot need about nine milliseconds of it
    // at 25 MHz with an empty option-ROM socket; the rest of the budget is for
    // the `RSEMU_VGABIOS` case, where a real video BIOS's own initialisation
    // costs 145 ms before POST resumes. Either way the whole test is a fraction
    // of a second of host time, because the guest spends the remainder halted.
    m.run_for(GlobalTime::from_nanos(1_000_000_000))
        .expect("the machine runs");

    println!("pc-at boot: text page:");
    for line in text_page(&m) {
        if !line.trim().is_empty() {
            println!("  |{line}|");
        }
    }
    let regs = cpu.regs();
    println!(
        "pc-at boot: stopped at {:04x}:{:08x}, halted={}, {} cycles",
        regs.cs,
        regs.rip,
        cpu.is_halted(),
        cpu.cycles()
    );
    let (faults, last) = cpu.bus_faults();
    println!("pc-at boot: {faults} unanswered bus access(es), last at {last:08x}");

    // Who owns `INT 10h` at the end. With an empty socket it is this firmware;
    // with `RSEMU_VGABIOS` pointing at a legacy video BIOS the option-ROM scan
    // found it, checksummed it, entered it at `seg:0003`, and the ROM installed
    // its own — which is the whole of what option-ROM dispatch is for, and is
    // checked rather than assumed.
    let int10 = (peek16(&m, 0x10 * 4 + 2), peek16(&m, 0x10 * 4));
    println!("pc-at boot: INT 10h -> {:04x}:{:04x}", int10.0, int10.1);
    if std::env::var("RSEMU_VGABIOS").is_ok() {
        assert_ne!(
            int10.0, 0xf000,
            "a video option ROM was supplied and never took over INT 10h: the \
             scan did not find it, or its checksum did not come out"
        );
    } else {
        assert_eq!(int10.0, 0xf000, "nothing else should own INT 10h here");
    }

    // -- the firmware's own POST --------------------------------------------
    //
    // 639 KiB, not 640: the last kilobyte is the EBDA, and `INT 12h` has to
    // agree with `0040:0013` or a guest allocates over it.
    assert_eq!(
        peek16(&m, 0x413),
        639,
        "the BDA's memory size is not what POST read out of the CMOS"
    );
    assert_eq!(
        peek16(&m, 0x40e),
        0x9fc0,
        "the EBDA segment is not published"
    );
    assert_eq!(peek(&m, 0x449), 0x03, "the video mode was never set");
    assert_eq!(peek(&m, 0x475), 1, "POST did not find the fixed disk");
    assert_ne!(peek16(&m, 0x410), 0, "the equipment word is still zero");
    assert!(
        peek32(&m, 0x46c) > 0,
        "the tick count at 0040:006c never moved: the 8254 or the 8259A is not \
         programmed, or IRQ0 is masked"
    );

    // -- the boot ------------------------------------------------------------
    assert_eq!(
        peek16(&m, u64::from(SCRATCH) + 14),
        DONE_MARKER,
        "the boot sector did not run to the end; the text page above says how \
         far the firmware got"
    );
    assert_eq!(
        peek(&m, u64::from(SCRATCH)),
        0x80,
        "the sector was not handed the drive it came off in DL"
    );

    // -- what the guest asked the firmware -----------------------------------
    assert_eq!(
        peek16(&m, u64::from(SCRATCH) + 2),
        639,
        "INT 12h disagrees with the BDA"
    );
    assert_eq!(
        peek16(&m, u64::from(SCRATCH) + 4),
        peek16(&m, 0x410),
        "INT 11h disagrees with the BDA"
    );

    // INT 13h AH=08h packs the geometry: CH is the low byte of cylinders-1,
    // CL's top two bits its high two, CL's low six the sectors per track, and
    // DH is heads-1.
    let cx = peek16(&m, u64::from(SCRATCH) + 6);
    let dx = peek16(&m, u64::from(SCRATCH) + 8);
    let cylinders = u32::from(cx >> 8) | ((u32::from(cx) & 0xc0) << 2);
    let sectors = cx & 0x3f;
    let heads = u32::from(dx >> 8) + 1;
    println!(
        "pc-at boot: INT 13h AH=08h says {} cylinders, {heads} heads, {sectors} sectors, \
         {} drive(s)",
        cylinders + 1,
        dx & 0xff
    );
    assert_eq!(sectors, 63, "sectors per track");
    assert_eq!(heads, 16, "heads");
    assert_eq!(
        (cylinders + 1) * heads * u32::from(sectors),
        SECTORS as u32,
        "the geometry does not cover the disk"
    );
    assert_eq!(dx & 0xff, 1, "the drive count");

    // The first E820 entry: conventional memory, from zero, ending where the
    // EBDA starts.
    let base = peek32(&m, u64::from(E820_BUFFER));
    let length = peek32(&m, u64::from(E820_BUFFER) + 8);
    let kind = peek32(&m, u64::from(E820_BUFFER) + 16);
    println!("pc-at boot: E820[0] base={base:#010x} length={length:#010x} type={kind}");
    assert_eq!(base, 0, "the first E820 entry does not start at zero");
    assert_eq!(
        length,
        639 * 1024,
        "the first E820 entry is not base memory"
    );
    assert_eq!(kind, 1, "the first E820 entry is not usable memory");
    assert_eq!(
        peek16(&m, u64::from(SCRATCH) + 10),
        1,
        "E820 did not hand back a continuation index"
    );

    // The guest's own read, of a sector the firmware never touched.
    assert_eq!(
        peek16(&m, u64::from(SCRATCH) + 12) >> 8,
        0,
        "INT 13h AH=02h reported an error"
    );
    let want = stamp(1);
    let got: Vec<u8> = (0..512)
        .map(|i| peek(&m, u64::from(SECOND_SECTOR) + i))
        .collect();
    assert_eq!(
        got, want,
        "the sector the guest read is not the sector on the disk"
    );

    // The diskette write and the block move. Bay 0 holds the boot disk here, so
    // the diskette is blank: whatever the read-back answers with, it is not
    // something that was already on the medium.
    check_write_and_block_move(&m, &[0u8; 512]);

    // And the fixed-disk write, against the drive's **medium** — the standard
    // `tests/pc_at_ide.rs` sets, and reachable here because `ata::bays`
    // publishes the backing store as a host object.
    let drive = rsemu::dev::ata::bays::get(&hosts, "ide0-master")
        .expect("no other host object claimed the name")
        .expect("the channel and the drive both opened it")
        .drive()
        .expect("the master bay is populated");
    assert_eq!(
        peek16(&m, u64::from(SCRATCH) + 24) >> 8,
        0,
        "INT 13h AH=03h reported an error writing the fixed disk"
    );
    assert_eq!(
        peek16(&m, u64::from(SCRATCH) + 26) >> 8,
        0,
        "INT 13h AH=02h reported an error reading it back"
    );
    let mut on_disk = vec![0u8; 512];
    drive
        .read_media(HD_WRITE_LBA * 512, &mut on_disk)
        .expect("in range");
    assert_eq!(
        on_disk,
        pattern(),
        "the sector INT 13h AH=03h wrote never reached the drive's medium"
    );
    let mut neighbour = vec![0u8; 512];
    drive
        .read_media((HD_WRITE_LBA - 1) * 512, &mut neighbour)
        .expect("in range");
    assert_eq!(
        neighbour,
        stamp(HD_WRITE_LBA - 1),
        "the write landed on the wrong sector"
    );
    let read_back: Vec<u8> = (0..512)
        .map(|i| peek(&m, u64::from(HD_READBACK) + i))
        .collect();
    assert_eq!(
        read_back,
        pattern(),
        "and the guest read back what it wrote"
    );

    // -- what it printed -----------------------------------------------------
    let page = text_page(&m);
    assert!(
        page.iter().any(|line| line.contains("rsemu BIOS")),
        "the firmware's own banner never reached the text page"
    );
    assert!(
        page.iter().any(|line| line.contains(GREETING)),
        "the boot sector's greeting never reached the text page"
    );

    // An AT reads ones off an unterminated bus and never faults — except that
    // this board's `pc.pmc` answers `Protected` for an address inside a PAM
    // window with no region under it, instead of falling through to the
    // space's `unassigned = read-as-ones`. The option-ROM scan walks
    // `0xc0000-0xdffff`, and `0xd0000-0xdffff` is exactly that: a window the
    // bridge decodes and nothing populates. Sixteen 2 KiB steps, one word
    // each, is thirty-two bytes and no more. Bounded rather than asserted at
    // zero, so a fix on the device side does not fail this test.
    assert!(
        faults <= 32,
        "the memory map refused {faults} accesses, last at {last:08x}: more than \
         the option-ROM scan's own"
    );
    if faults > 0 {
        assert!(
            (0xd_0000..0xe_0000).contains(&last),
            "the last refused access at {last:08x} is not in the unpopulated \
             half of the option-ROM window"
        );
    }
}

/// The same boot, off the **diskette**, through the µPD765 and the 8237.
///
/// Both IDE bays are empty, so `INT 19h` tries `0x80`, `INT 13h` reports no
/// such drive, and the fallback takes over. Everything after that is a
/// different path from the fixed disk's: the digital output register, four
/// `SENSE INTERRUPT STATUS` commands to clear the reset's ready-changed
/// reports, `SPECIFY`, a `SEEK`, a DMA channel programmed for a memory write,
/// and a `READ DATA` per sector. The boot sector then reads a sector of its
/// own the same way.
#[test]
fn the_board_boots_off_the_diskette_too() {
    let (mut m, cpu, _hosts) = board_from(false, Some(disk_image(false)));
    m.run_for(GlobalTime::from_nanos(1_000_000_000))
        .expect("the machine runs");

    println!("pc-at boot (diskette): text page:");
    for line in text_page(&m) {
        if !line.trim().is_empty() {
            println!("  |{line}|");
        }
    }
    let regs = cpu.regs();
    println!(
        "pc-at boot (diskette): stopped at {:04x}:{:08x}, halted={}",
        regs.cs,
        regs.rip,
        cpu.is_halted()
    );

    assert_eq!(peek(&m, 0x475), 0, "no fixed disk should have been found");
    assert_eq!(
        peek16(&m, u64::from(SCRATCH) + 14),
        DONE_MARKER,
        "the diskette's boot sector did not run to the end"
    );
    assert_eq!(
        peek(&m, u64::from(SCRATCH)),
        0x00,
        "the sector was not told it came off drive 0"
    );

    // The geometry `INT 13h AH=08h` reports for a 1.44 MB unit, out of the CMOS
    // diskette-type byte rather than out of a constant in the firmware.
    let cx = peek16(&m, u64::from(SCRATCH) + 6);
    let dx = peek16(&m, u64::from(SCRATCH) + 8);
    println!(
        "pc-at boot (diskette): INT 13h AH=08h cylinders-1={}, sectors={}, heads-1={}, drives={}",
        cx >> 8,
        cx & 0x3f,
        dx >> 8,
        dx & 0xff
    );
    assert_eq!(cx & 0x3f, 18, "sectors per track");
    assert_eq!(dx >> 8, 1, "heads minus one");

    // The sector the guest read for itself, which went through the whole
    // controller-and-DMA path a second time.
    assert_eq!(
        peek16(&m, u64::from(SCRATCH) + 12) >> 8,
        0,
        "INT 13h AH=02h on the diskette reported an error"
    );
    let got: Vec<u8> = (0..512)
        .map(|i| peek(&m, u64::from(SECOND_SECTOR) + i))
        .collect();
    assert_eq!(
        got,
        stamp(1),
        "the sector the guest read off the diskette is not the one on the medium"
    );

    // The write, on the medium the machine actually booted from. The sector it
    // lands on held [`stamp`] before, which is what makes the read-back a claim
    // about the medium rather than about the controller's buffer: a write that
    // never reached the image would answer with that stamp.
    check_write_and_block_move(&m, &stamp(WRITE_LBA));

    // There is no fixed disk on this board, and the firmware says so rather
    // than pretending: `INT 13h AH=03h` with `DL = 0x80` comes back with carry
    // and a non-zero status. A BIOS that answered "done" for a drive that is
    // not there is the failure this catches.
    assert_ne!(
        peek16(&m, u64::from(SCRATCH) + 24) >> 8,
        0,
        "INT 13h AH=03h claimed to write a fixed disk this board does not have"
    );

    let page = text_page(&m);
    assert!(
        page.iter().any(|line| line.contains(GREETING)),
        "the boot sector's greeting never reached the text page"
    );
}

/// What the guest got out of `INT 13h AH=03h` and `INT 15h AH=87h`.
///
/// `before` is what the diskette sector held when the machine was built.
///
/// # Why a read-back is a check on the medium
///
/// `tests/pc_at_ide.rs` compares a written sector against the drive's backing
/// store rather than against anything the device model chose to answer with,
/// and that is the standard. It is reachable there because `ata::bays` publishes
/// the medium as a host object; `pc.fdc` publishes nothing, its `contents()` is
/// only reachable from whoever constructed it, and `Bindings::bind` refuses a
/// second binding for a class `rsemu::dev::pc::bind` has already claimed — so a
/// test cannot get a handle on the controller this board built.
///
/// What is left is still a claim about the medium rather than about a buffer,
/// for a reason specific to this model and worth writing down: `pc.fdc` rebuilds
/// its sector buffer from the image at the start of *every* transfer, so the
/// buffer a write filled is gone by the time the read that follows runs. A read
/// that answers with the pattern is the image answering. The two failures this
/// would actually catch — the 8237 programmed for the wrong direction, so the
/// "write" reads the disk into the guest's buffer instead, and a write that
/// stops at the controller and never reaches the image — both leave `before` on
/// the medium, and the assertion below is that `before` is not what came back.
fn check_write_and_block_move(m: &Machine, before: &[u8]) {
    let want = pattern();
    assert_ne!(
        want, before,
        "the sector already held the pattern, so reading it back proves nothing"
    );

    assert_eq!(
        peek16(m, u64::from(SCRATCH) + 16) >> 8,
        0,
        "INT 13h AH=03h reported an error writing the diskette"
    );
    assert_eq!(
        peek16(m, u64::from(SCRATCH) + 18) >> 8,
        0,
        "INT 13h AH=02h reported an error reading the sector back"
    );
    let got: Vec<u8> = (0..512).map(|i| peek(m, u64::from(READBACK) + i)).collect();
    assert_eq!(
        got, want,
        "the sector read back off the diskette is not the one the guest wrote: \
         the write never reached the medium"
    );

    // `INT 15h AH=87h`, out and back. `EXT_TARGET` is above the first megabyte,
    // so nothing in this boot sector could have put the pattern there by any
    // other route — it is the only thing here that needed protected mode.
    assert_eq!(
        peek16(m, u64::from(SCRATCH) + 20) >> 8,
        0,
        "INT 15h AH=87h reported an error moving the block up"
    );
    assert_eq!(
        peek16(m, u64::from(SCRATCH) + 22) >> 8,
        0,
        "INT 15h AH=87h reported an error moving it back"
    );
    let up: Vec<u8> = (0..512)
        .map(|i| peek(m, u64::from(EXT_TARGET) + i))
        .collect();
    assert_eq!(
        up, want,
        "INT 15h AH=87h did not land the block at {EXT_TARGET:#x}"
    );
    let down: Vec<u8> = (0..512)
        .map(|i| peek(m, u64::from(XFER_BACK) + i))
        .collect();
    assert_eq!(
        down, want,
        "INT 15h AH=87h did not bring the block back down"
    );
    // A word count is words, and a handler that read it as bytes would move
    // twice as much. Both landing zones have zeroed RAM after them.
    assert_eq!(
        peek16(m, u64::from(EXT_TARGET) + 512),
        0,
        "the block move ran past the end of the block"
    );
    assert_eq!(
        peek16(m, u64::from(XFER_BACK) + 512),
        0,
        "the block move back ran past the end of the block"
    );
}

/// A key typed at the 8042 comes back out of `INT 16h` as a character.
///
/// This is the half of the firmware that has nothing to do with the disk:
/// `INT 09h` takes the byte off port 0x60, decodes it against the set-1 table
/// and puts scan code and character in the BDA's ring; `INT 16h` blocks in a
/// `HLT` loop until one is there. None of it can be tested without a running
/// machine, an unmasked IRQ1 and a guest that asks.
#[test]
fn a_key_typed_at_the_8042_reaches_the_guest_through_int_16h() {
    use rsemu::host::chardev::ports;

    let (mut m, _cpu, hosts) = board(true);
    let keyboard = ports::open(&hosts, "keyboard").expect("the 8042 opened its port");

    // Let it boot first: the guest has to be sitting in `INT 16h` before a key
    // means anything, and the firmware's keyboard init has to have run.
    m.run_for(GlobalTime::from_nanos(20_000_000))
        .expect("the machine runs");
    assert_eq!(
        peek16(&m, u64::from(SCRATCH) + 14),
        DONE_MARKER,
        "the echo boot sector did not get as far as its keyboard loop"
    );

    // Set 2, which is what an AT keyboard sends and what `pc.kbc` carries:
    // `0x33` is the `H` key going down. The controller translates it to set 1
    // on the way to the output buffer, because POST turned translation on.
    //
    // **One key, not two, and that is a device bug rather than a firmware
    // one.** `pc.kbc`'s `read_data` clears `OBF` and immediately refills the
    // output buffer from the keyboard's queue *without* re-driving `IRQ1`, so
    // the line never falls between two bytes and an edge-triggered 8259A —
    // which is what `ICW1` asks for and what a PC has — never sees a second
    // edge. Measured on this test: after the first key the status port reads
    // `0x05` (a byte waiting) with the master's `IRR` at `0x00` (nothing
    // pending). Feed two keys here and the second never arrives. When that is
    // fixed, this test should feed a make/break pair for each of two keys and
    // assert both characters.
    keyboard.feed(&[0x33]);
    m.run_for(GlobalTime::from_nanos(40_000_000))
        .expect("the machine runs");

    let page = text_page(&m);
    println!("pc-at boot: text page after typing:");
    for line in &page {
        if !line.trim().is_empty() {
            println!("  |{line}|");
        }
    }
    assert!(
        page.iter().any(|line| line == "h"),
        "the key never came back out of INT 16h: the text page is {page:?}"
    );
}

/// A machine with nothing to boot parks in `INT 18h` **with interrupts on**.
///
/// The firmware's own comment there promises "interrupts on so a keystroke
/// still reaches the buffer", and until the `STI` was added it was not true:
/// `INT 18h` is entered by an `INT` instruction, which clears `IF`, and a `HLT`
/// with `IF` clear is the one halt an x86 cannot be woken from. The machine
/// looked alive and was not — no tick, no keystroke, nothing but `RESET`.
///
/// So this asserts the consequence rather than the instruction: a key typed at
/// the 8042 *after* the machine has parked reaches the BIOS keyboard buffer,
/// which can only happen if `INT 09h` ran, which can only happen if `IF` is
/// set.
#[test]
fn a_machine_with_nothing_to_boot_parks_with_interrupts_on() {
    use rsemu::host::chardev::ports;

    // A blank diskette and no fixed disk: `INT 19h` finds no signature on
    // either and falls through to `INT 18h`.
    let (mut m, cpu, hosts) = board_from(false, Some(Vec::new()));
    let keyboard = ports::open(&hosts, "keyboard").expect("the 8042 opened its port");
    m.run_for(GlobalTime::from_nanos(200_000_000))
        .expect("the machine runs");

    let page = text_page(&m);
    println!("pc-at boot (nothing bootable): text page:");
    for line in &page {
        if !line.trim().is_empty() {
            println!("  |{line}|");
        }
    }
    assert!(
        page.iter().any(|line| line.contains("No bootable device")),
        "the firmware did not reach INT 18h: the text page is {page:?}"
    );
    assert!(cpu.is_halted(), "and it parked rather than running on");
    assert_ne!(
        cpu.regs().eflags & 0x200,
        0,
        "parked with IF clear, which is a halt nothing but RESET ends"
    );

    // The BIOS Data Area's keyboard ring, at 0040:001a and 0040:001c. Empty
    // means head == tail; a key that arrived moves the tail.
    let head = peek16(&m, 0x41a);
    assert_eq!(peek16(&m, 0x41c), head, "the buffer starts empty");

    // `0x33` is `H` going down, in set 2, which is what an AT keyboard sends.
    keyboard.feed(&[0x33]);
    m.run_for(GlobalTime::from_nanos(40_000_000))
        .expect("the machine runs");
    assert_ne!(
        peek16(&m, 0x41c),
        head,
        "the keystroke never reached the buffer: the parked processor was \
         asleep with interrupts off"
    );
}

/// **FreeDOS boots** — `ROADMAP.md` phase 6a's gate, on rsemu's own firmware.
///
/// Gated on `RSEMU_FREEDOS_FLOPPY` naming a FreeDOS boot diskette, so an
/// ordinary `cargo test` skips it and stays hermetic.
/// `scripts/fetch-testdata.sh freedos` puts one in the ignored corpus directory
/// and prints the command:
///
/// ```text
/// scripts/fetch-testdata.sh freedos
/// RSEMU_FREEDOS_FLOPPY=testdata/freedos/x86BOOT.img \
///   cargo test --release --all-features --test pc_at_boot -- --nocapture freedos
/// ```
///
/// **Nothing is vendored.** FreeDOS is GPL-2.0 and the image never enters this
/// repository: running a program as an emulated guest is ordinary use and
/// creates no derivative work, while shipping it here would be redistribution
/// under its terms (`ROADMAP.md` §1). Its source was not read, and none of the
/// firmware was written against it — every function `INT 13h` and `INT 10h`
/// answer comes from Ralf Brown's Interrupt List. What the boot *did* do is
/// name which of them a real operating system actually calls, and
/// `INT 10h AH=08h` was added because of it.
///
/// # What a pass means
///
/// The whole chain, none of which this firmware had ever been asked for before:
/// POST, `INT 19h` declining an empty IDE bay and falling through to the
/// µPD765, FreeDOS's own boot sector loading a compressed kernel one sector at
/// a time through `INT 13h AH=02h`, the kernel decompressing and initialising,
/// `FDCONFIG.SYS`, and `COMMAND.COM` printing its banner and running the
/// startup file. The assertions are deliberately about *the guest owning the
/// machine* rather than about any particular version's wording, so a different
/// FreeDOS build still passes.
#[test]
fn freedos_boots_on_rsemus_own_firmware() {
    let Ok(path) = std::env::var("RSEMU_FREEDOS_FLOPPY") else {
        println!(
            "pc-at freedos: RSEMU_FREEDOS_FLOPPY is unset, so this test has nothing to \
             boot. `scripts/fetch-testdata.sh freedos` fetches one."
        );
        return;
    };
    let image = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));
    println!("pc-at freedos: booting {path} ({} bytes)", image.len());

    // Sixty virtual seconds. Measured rather than guessed: the kernel's
    // decompressor is a bit-at-a-time loop and `COMMAND.COM` reaches its banner
    // at about twenty, with the rest of the budget for the startup file. The
    // guest spends most of it halted waiting for the tick, so this costs single
    // digit seconds of host time.
    let ms: u64 = std::env::var("RSEMU_FREEDOS_MS")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(60_000);

    let (mut m, cpu, _hosts) = board_from(false, Some(image));
    let started = std::time::Instant::now();
    m.run_for(GlobalTime::from_nanos(ms * 1_000_000))
        .expect("the machine runs");

    let page = text_page(&m);
    println!("pc-at freedos: text page after {ms} virtual ms:");
    for line in &page {
        if !line.trim().is_empty() {
            println!("  |{line}|");
        }
    }
    let regs = cpu.regs();
    println!(
        "pc-at freedos: {:?} of host time; stopped at {:04x}:{:08x}, halted={}, \
         protected={}",
        started.elapsed(),
        regs.cs,
        regs.rip,
        cpu.is_halted(),
        cpu.sys().protected()
    );
    let vectors: Vec<String> = [0x08u64, 0x10, 0x13, 0x1c, 0x21, 0x2f]
        .iter()
        .map(|v| {
            format!(
                "{v:02x}->{:04x}:{:04x}",
                peek16(&m, v * 4 + 2),
                peek16(&m, v * 4)
            )
        })
        .collect();
    println!("pc-at freedos: vectors {}", vectors.join(" "));

    // POST still did its job: a guest that took the machine over did not have
    // to repair the BIOS data area first.
    assert_eq!(peek16(&m, 0x413), 639, "the BDA's memory size");

    // DOS is resident. `INT 21h` starts life pointing at this firmware's
    // "unknown function" stub in segment 0xf000, and only an operating system
    // moves it — so this one assertion covers the boot sector, the kernel load,
    // the decompression and the kernel's own initialisation, and it says
    // nothing about which DOS or which version.
    let int21 = peek16(&m, 0x21 * 4 + 2);
    assert_ne!(
        int21, 0xf000,
        "INT 21h still points into the BIOS: no DOS kernel installed itself. \
         The text page above says how far it got."
    );
    assert_ne!(
        int21, 0x0000,
        "INT 21h points at the interrupt vector table"
    );

    // And it got as far as a shell. `COMMAND.COM` is the first thing in the
    // sequence that prints its own name, so this distinguishes "the kernel
    // loaded" from "the system came up".
    assert!(
        page.iter()
            .any(|line| line.contains("FreeCom") || line.contains("FreeDOS")),
        "nothing on the text page names FreeDOS: the kernel loaded but the shell \
         never printed. The page is {page:?}"
    );

    // The same bound the other two tests hold: an AT reads ones off an
    // unterminated bus, and the only refusals on this board are the option-ROM
    // scan's own walk through the unpopulated half of its window.
    let (faults, last) = cpu.bus_faults();
    println!("pc-at freedos: {faults} unanswered bus access(es), last at {last:08x}");
    assert!(
        faults <= 32,
        "the memory map refused {faults} accesses, last at {last:08x}"
    );
}
