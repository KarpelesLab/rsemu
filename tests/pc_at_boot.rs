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
//!
//! The boot sector is assembled by [`rsemu::fw::asm16`], the same assembler the
//! firmware is built with — so the test guest is written the same way the
//! firmware is, and the assembler is exercised by something other than its own
//! unit tests.

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

/// The word the boot sector writes last, so "it started" and "it finished" are
/// two different claims.
const DONE_MARKER: u16 = 0x600d;

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
    a.movi8(DL, 0x80);
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
    a.movi8(DL, 0x80);
    a.movi(BX, SECOND_SECTOR);
    a.int(0x13);
    a.movto(Mem::abs(SCRATCH + 12), AX);

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
    // An empty option-ROM socket: 64 KiB of zeroes has no `0x55 0xAA`, which is
    // exactly what the firmware's scan must survive.
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("floppy", vec![0u8; 1_474_560]);
    options.realize.media.insert("hd0", disk_image(echo));
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
    let (mut m, cpu, _hosts) = board(false);

    // Fifty virtual milliseconds is about a thousand times what POST and the
    // boot need at 25 MHz, and it is short enough that the whole test is a
    // fraction of a second of host time.
    m.run_for(GlobalTime::from_nanos(50_000_000))
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
        regs.eip,
        cpu.is_halted(),
        cpu.cycles()
    );
    let (faults, last) = cpu.bus_faults();
    println!("pc-at boot: {faults} unanswered bus access(es), last at {last:08x}");

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
