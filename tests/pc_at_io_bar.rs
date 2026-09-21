//! **A guest moves a PCI I/O base address register and drives the device at the
//! address it chose.**
//!
//! This is the end of the thread `docs/platforms/pc-at.md` used to list under
//! "what is known to be missing". An I/O base address register moves a window
//! in the **I/O space**, and on this board a configuration write is an `OUT` to
//! `0xcfc` — which travels through that same space. So the retopology cannot
//! happen inside the write that asks for it, and it cannot happen at the next
//! configuration access either, because that is another `OUT` to `0xcfc`. What
//! places the window is the host bridge's scheduler drain, one round later.
//! `src/bus/pci/bar.rs` carries the argument; this is the evidence, from the
//! only place evidence can come from: a program running on the board.
//!
//! # What the guest does
//!
//! It is a boot sector assembled with [`rsemu::fw::asm16`], and it uses
//! **rsemu's own BIOS** for everything it knows about PCI — `INT 1Ah AH=B1h`,
//! the PCI BIOS interface, exactly as a DOS-era driver would:
//!
//! 1. `AX=B103h`, find by class code `010105h` — mass storage, IDE,
//!    programming interface 05h (native on both channels). It has been told
//!    nothing about where the controller is.
//! 2. `AX=B10Ah` to read the register the service says is BAR2, and `B10Dh` to
//!    write all ones and read the size mask back, which is §6.2.5.1's sizing
//!    protocol done through the firmware.
//! 3. `B10Dh` to place the command block at `0x300` and the control block at
//!    `0x308`, and `B10Ch` to set `COMMAND[0]`.
//! 4. **Poll** the status port at `0x307` until something answers. That loop is
//!    the honest shape of the deferral: for the rest of the scheduler round the
//!    port reads `0xff`, because the window is not in the map yet.
//! 5. Write two command-block registers and read them back, which is the
//!    unambiguous proof that the accesses reached a drive rather than an open
//!    bus — an open bus reads as ones and a latch reads back what was put in it.
//! 6. **Move it**: the same registers again, to `0x310` and `0x318`, and poll
//!    the new status port. Then check that the old one has gone dead and that
//!    the latches it wrote before the move are still there, because the drive
//!    never noticed that the board's decoder changed its mind.
//!
//! # The board
//!
//! `machines/pc-at.machine` with three edits, each of which is in the text
//! below rather than in the shipped file: the secondary IDE channel's two
//! fixed-port `map` statements are removed, a blank drive is fitted in its
//! master bay, and a `pc.ide-pci` function is added at `00:01.0` decoding that
//! channel. The *primary* channel is untouched, so everything the firmware
//! does with a hard disk is unaffected — and the secondary channel has no
//! address at all until the guest gives it one, which is what makes the
//! before-and-after readable.
//!
//! No accelerator: the guest never leaves real mode and talks only to
//! configuration space and I/O ports, so the interpreter runs it everywhere.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-apic",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "dev-pc-ide-pci",
    feature = "dev-pc-hpet",
    feature = "fw-pcbios",
    feature = "machine-pc-at"
))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::fw::asm16::{AL, AX, Asm, BX, CX, Cc, DI, DS, DX, Mem, SI, SP, SS};
use rsemu::machine::{Machine, build};

// ---------------------------------------------------------------------------
// the board's own numbers
// ---------------------------------------------------------------------------

/// Base class 01 (mass storage), sub-class 01 (IDE), programming interface 05
/// (native mode on both channels, no bus master) — what `pc.ide-pci` hardwires
/// at configuration offset 09h, and the only thing the guest is told.
const IDE_CLASS: u32 = 0x0001_0105;

/// Where the machine text below puts the function: device 1, **function 0**.
/// The PCI BIOS packs that into one byte as device in bits 7-3 and function in
/// bits 2-0 (*PCI BIOS* §4.3).
///
/// Function 0 and not 1, which is where a PIIX-lineage part puts its IDE
/// function, because a scan reads Header Type bit 7 of *function 0* to decide
/// whether to look at the other seven (Rev 2.1 §6.2.1) — so a lone function 1
/// on a board with no function 0 is a device no firmware ever finds.
const IDE_DEVFN: u8 = 1 << 3;

/// The register the secondary channel's command block answers at: BAR2.
const REG_BAR2: u16 = 0x18;
/// And its control block: BAR3.
const REG_BAR3: u16 = 0x1c;
/// The Command register.
const REG_COMMAND: u16 = 0x04;
/// `COMMAND[0]`, the I/O space enable (Rev 2.1 §6.2.2).
const COMMAND_IO: u16 = 0x0001;

/// Where the guest first puts the command block. `0x300`-`0x30f` is the
/// prototype-card range on a PC/AT and nothing on this board decodes it.
const FIRST_COMMAND: u16 = 0x0300;
/// And the control block. Its one decoded byte is at offset 2 of the window.
const FIRST_CONTROL: u16 = 0x0308;
/// Where it moves them.
const SECOND_COMMAND: u16 = 0x0310;
const SECOND_CONTROL: u16 = 0x0318;

/// Command-block offsets (ATA/ATAPI-6 §7).
const OFF_SECTOR_COUNT: u16 = 2;
const OFF_LBA_LOW: u16 = 3;
const OFF_STATUS: u16 = 7;

/// What the guest writes into the sector count and LBA low registers.
const LATCH_A: u8 = 0x5a;
const LATCH_B: u8 = 0x17;

// ---------------------------------------------------------------------------
// where the guest leaves what it found
// ---------------------------------------------------------------------------

/// Where the boot sector lands.
const BOOT: u16 = 0x7c00;
/// The block at `0x0500` every PC has left free since 1981.
const SCRATCH: u16 = 0x0500;

const OFF_STARTED: u16 = SCRATCH;
const OFF_DONE: u16 = SCRATCH + 2;
/// `BX` from `B103h`: the bus in `BH` and the device/function byte in `BL`.
const OFF_FOUND: u16 = SCRATCH + 4;
/// Whether `B103h` set carry.
const OFF_FOUND_FLAGS: u16 = SCRATCH + 6;
/// BAR2 as it reads out of reset.
const OFF_BAR_RESET: u16 = SCRATCH + 8;
/// The size mask BAR2 reports.
const OFF_BAR_MASK: u16 = SCRATCH + 12;
/// BAR2 read back after the guest placed it the first time.
const OFF_BAR_PLACED: u16 = SCRATCH + 16;
/// The status port at the first base, before the drain could have run.
const OFF_STATUS_EARLY: u16 = SCRATCH + 20;
/// How many times round the poll loop it took. Zero means it was never `0xff`.
const OFF_POLL_FIRST: u16 = SCRATCH + 22;
/// The status port at the first base, once something answered.
const OFF_STATUS_FIRST: u16 = SCRATCH + 24;
/// The alternate status register at `BAR3 + 2`.
const OFF_ALT_STATUS: u16 = SCRATCH + 26;
/// A byte of the control-block window that is *not* offset 2.
const OFF_CONTROL_HOLE: u16 = SCRATCH + 28;
/// The two latched command-block registers, read back.
const OFF_LATCH_A: u16 = SCRATCH + 30;
const OFF_LATCH_B: u16 = SCRATCH + 32;
/// How many times round the second poll loop.
const OFF_POLL_SECOND: u16 = SCRATCH + 34;
/// The status port at the *old* base after the move.
const OFF_STATUS_OLD: u16 = SCRATCH + 36;
/// The two latches, read back at the new base.
const OFF_LATCH_A2: u16 = SCRATCH + 38;
const OFF_LATCH_B2: u16 = SCRATCH + 40;

/// What [`OFF_STARTED`] holds.
const STARTED: u16 = 0xb101;
/// What [`OFF_DONE`] holds.
const DONE: u16 = 0x600d;

/// How many times the guest will look before giving up on a window appearing.
///
/// Generous on purpose: the bound is one scheduler round and this is a count of
/// `IN` instructions, so any value large enough to outlast a round will do. A
/// loop that runs out records its count and the test says so, rather than the
/// board hanging.
const POLL_LIMIT: u16 = 20000;

// ---------------------------------------------------------------------------
// the guest
// ---------------------------------------------------------------------------

/// `AX=B10Ah`, read a configuration Dword. Leaves the value in `ECX`.
fn read_dword(a: &mut Asm, register: u16) {
    a.movi(AX, 0xb10a);
    a.mov(BX, Mem::abs(OFF_FOUND));
    a.movi(DI, register);
    a.int(0x1a);
}

/// `AX=B10Dh`, write a configuration Dword.
fn write_dword(a: &mut Asm, register: u16, value: u32) {
    a.movi(AX, 0xb10d);
    a.mov(BX, Mem::abs(OFF_FOUND));
    a.movi(DI, register);
    a.movi32(CX, value);
    a.int(0x1a);
}

/// `AX=B10Ch`, write a configuration word.
fn write_word(a: &mut Asm, register: u16, value: u16) {
    a.movi(AX, 0xb10c);
    a.mov(BX, Mem::abs(OFF_FOUND));
    a.movi(DI, register);
    a.movi(CX, value);
    a.int(0x1a);
}

/// Read one I/O port into `AL` and store it as a word.
fn inb_to(a: &mut Asm, port: u16, at: u16) {
    a.movi(DX, port);
    a.in_al_dx();
    a.movi8(rsemu::fw::asm16::AH, 0);
    a.movto(Mem::abs(at), AX);
}

/// Write one I/O port.
fn outb(a: &mut Asm, port: u16, value: u8) {
    a.movi(DX, port);
    a.movi8(AL, value);
    a.out_dx_al();
}

/// Poll `port` until it stops reading as ones, recording how many attempts it
/// took at `at` — and zero if it never did.
///
/// **This loop is the deferral, seen from the guest.** The configuration write
/// that placed the window completed; the window is not in the map yet, so the
/// port reads `0xff` exactly as an undecoded port does, until the host bridge's
/// `Device::advance_to` runs at the end of the scheduler round.
fn poll_until_decoded(a: &mut Asm, port: u16, at: u16) {
    a.movmi(Mem::abs(at), 0);
    a.movi(SI, 0);
    let top = a.here_label();
    a.inc(SI);
    a.movi(DX, port);
    a.in_al_dx();
    a.alui8(rsemu::fw::asm16::Alu::CMP, AL, 0xff);
    let found = a.label();
    a.jcc(Cc::NE, found);
    a.alui(rsemu::fw::asm16::Alu::CMP, SI, POLL_LIMIT);
    a.jcc(Cc::B, top);
    // Ran out: leave the count at zero, which is what the test reads as "the
    // window never appeared".
    let out = a.label();
    a.jmp(out);
    a.bind(found);
    a.movto(Mem::abs(at), SI);
    a.bind(out);
}

/// Assemble the boot sector.
#[allow(clippy::too_many_lines)]
fn boot_sector() -> Vec<u8> {
    let mut a = Asm::new(usize::from(BOOT) + 512, 0x00);
    a.seek(BOOT);

    a.cli();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(SS, AX);
    a.movi(SP, BOOT);
    a.sti();
    a.movmi(Mem::abs(OFF_STARTED), STARTED);

    // -- find the controller by its class code ------------------------------
    a.movi(AX, 0xb103);
    a.movi32(CX, IDE_CLASS);
    a.movi(SI, 0);
    a.int(0x1a);
    a.movto(Mem::abs(OFF_FOUND), BX);
    a.pushf();
    a.pop(AX);
    a.movto(Mem::abs(OFF_FOUND_FLAGS), AX);

    // -- read BAR2 as it stands, then size it -------------------------------
    read_dword(&mut a, REG_BAR2);
    a.movto32(Mem::abs(OFF_BAR_RESET), CX);
    write_dword(&mut a, REG_BAR2, 0xffff_ffff);
    read_dword(&mut a, REG_BAR2);
    a.movto32(Mem::abs(OFF_BAR_MASK), CX);

    // -- place both windows and turn the decode on --------------------------
    write_dword(&mut a, REG_BAR2, u32::from(FIRST_COMMAND));
    write_dword(&mut a, REG_BAR3, u32::from(FIRST_CONTROL));
    read_dword(&mut a, REG_BAR2);
    a.movto32(Mem::abs(OFF_BAR_PLACED), CX);
    write_word(&mut a, REG_COMMAND, COMMAND_IO);

    // What the port reads *immediately*: the register moved, the map has not.
    inb_to(&mut a, FIRST_COMMAND + OFF_STATUS, OFF_STATUS_EARLY);

    // And what it reads once the host bridge has had a moment with no access
    // in flight.
    poll_until_decoded(&mut a, FIRST_COMMAND + OFF_STATUS, OFF_POLL_FIRST);
    // Select device 0, which is the bay the machine text below fits a drive in.
    outb(&mut a, FIRST_COMMAND + 6, 0xa0);
    inb_to(&mut a, FIRST_COMMAND + OFF_STATUS, OFF_STATUS_FIRST);
    inb_to(&mut a, FIRST_CONTROL + 2, OFF_ALT_STATUS);
    inb_to(&mut a, FIRST_CONTROL, OFF_CONTROL_HOLE);

    // -- two latches, which an open bus cannot fake -------------------------
    outb(&mut a, FIRST_COMMAND + OFF_SECTOR_COUNT, LATCH_A);
    outb(&mut a, FIRST_COMMAND + OFF_LBA_LOW, LATCH_B);
    inb_to(&mut a, FIRST_COMMAND + OFF_SECTOR_COUNT, OFF_LATCH_A);
    inb_to(&mut a, FIRST_COMMAND + OFF_LBA_LOW, OFF_LATCH_B);

    // -- the move -----------------------------------------------------------
    write_dword(&mut a, REG_BAR2, u32::from(SECOND_COMMAND));
    write_dword(&mut a, REG_BAR3, u32::from(SECOND_CONTROL));
    poll_until_decoded(&mut a, SECOND_COMMAND + OFF_STATUS, OFF_POLL_SECOND);
    inb_to(&mut a, FIRST_COMMAND + OFF_STATUS, OFF_STATUS_OLD);
    inb_to(&mut a, SECOND_COMMAND + OFF_SECTOR_COUNT, OFF_LATCH_A2);
    inb_to(&mut a, SECOND_COMMAND + OFF_LBA_LOW, OFF_LATCH_B2);

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

/// A 1.44 MB diskette with that sector on it.
fn diskette() -> Vec<u8> {
    let mut image = boot_sector();
    assert_eq!(image.len(), 512, "a boot sector is one sector");
    image.resize(1_474_560, 0);
    image
}

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// `machines/pc-at.machine` with the secondary IDE channel taken off the
/// board's fixed decoder and put behind a PCI function instead.
///
/// Three edits, and each is here rather than in the shipped file because the
/// shipped board is a 1984-lineage AT whose IDE ports are at `0x170` and
/// `0x376` by definition. What this text describes is the *other* way a PCI
/// controller may be configured, which is the thing under test.
fn board_text() -> String {
    let mut text = String::from(rsemu::dev::pc::PC_AT);
    for line in [
        "  map port 0x0170 size 0x0008 = ide1.regs      # IDE, secondary channel\n",
        "  map port 0x0376 size 0x0001 = ide1.ctl       # ...and its control block\n",
    ] {
        assert!(
            text.contains(line),
            "the shipped board no longer has this line: {line}"
        );
        text = text.replace(line, "");
    }
    // A drive that is certainly there, so a status read is unambiguous: an
    // ATAPI device's Status register reads zero *while it is present*
    // (ATA/ATAPI-6 §7.15.6.3), which is exactly as hard to tell from an open
    // bus as it sounds.
    let cd = "  object cd0 \"ata.cdrom\" { image = \"cdrom\", bay = \"ide1-master\" }\n";
    assert!(text.contains(cd), "the shipped board no longer fits a CD");
    text = text.replace(
        cd,
        "  object hd2 \"ata.disk\" { size = 1M, bay = \"ide1-master\" }\n\
         \x20 object idepci \"pc.ide-pci\" { bus = \"pci0\", device = 1, function = 0, \
         iospace = \"port\", secondary = ide1 }\n",
    );
    text
}

/// Build `text` with rsemu's own BIOS in its socket and the prober on the
/// diskette.
fn board(name: &str, text: &str) -> Machine {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's classes");
    options
        .realize
        .media
        .insert("bios", rsemu::fw::pcbios::image());
    options.realize.media.insert("vgabios", Vec::new());
    options.realize.media.insert("floppy", diskette());
    for slot in ["disk", "hd0", "hd1", "cdrom", "cd0", "cd1"] {
        options.realize.media.insert(slot, Vec::new());
    }
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut m = build(name, text, &registry, &options)
        .unwrap_or_else(|e| panic!("{name} does not realize: {e}"));
    m.reset(ResetKind::Cold);
    m.sweep();
    m
}

fn peek16(m: &Machine, at: u16) -> u16 {
    m.space("mem")
        .expect("the memory space")
        .read(u64::from(at), Width::U16, MemAttrs::DEBUG)
        .unwrap_or(0) as u16
}

fn peek32(m: &Machine, at: u16) -> u32 {
    u32::from(peek16(m, at)) | (u32::from(peek16(m, at + 2)) << 16)
}

/// One byte of the machine's I/O space, read as a debugger reads.
fn port(m: &Machine, at: u16) -> u8 {
    m.space("port")
        .expect("the I/O space")
        .read(u64::from(at), Width::U8, MemAttrs::DEBUG)
        .unwrap_or(0xdead) as u8
}

/// Everything the guest brought back.
#[derive(Debug)]
struct Found {
    found: u16,
    found_carry: bool,
    bar_reset: u32,
    bar_mask: u32,
    bar_placed: u32,
    status_early: u16,
    poll_first: u16,
    status_first: u16,
    alt_status: u16,
    control_hole: u16,
    latch_a: u16,
    latch_b: u16,
    poll_second: u16,
    status_old: u16,
    latch_a2: u16,
    latch_b2: u16,
}

fn run() -> (Machine, Found) {
    let mut m = board("pc-at-io-bar.machine", &board_text());
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
    assert_eq!(
        peek16(&m, OFF_DONE),
        DONE,
        "the guest did not finish driving the controller"
    );
    let found = Found {
        found: peek16(&m, OFF_FOUND),
        found_carry: peek16(&m, OFF_FOUND_FLAGS) & 1 != 0,
        bar_reset: peek32(&m, OFF_BAR_RESET),
        bar_mask: peek32(&m, OFF_BAR_MASK),
        bar_placed: peek32(&m, OFF_BAR_PLACED),
        status_early: peek16(&m, OFF_STATUS_EARLY),
        poll_first: peek16(&m, OFF_POLL_FIRST),
        status_first: peek16(&m, OFF_STATUS_FIRST),
        alt_status: peek16(&m, OFF_ALT_STATUS),
        control_hole: peek16(&m, OFF_CONTROL_HOLE),
        latch_a: peek16(&m, OFF_LATCH_A),
        latch_b: peek16(&m, OFF_LATCH_B),
        poll_second: peek16(&m, OFF_POLL_SECOND),
        status_old: peek16(&m, OFF_STATUS_OLD),
        latch_a2: peek16(&m, OFF_LATCH_A2),
        latch_b2: peek16(&m, OFF_LATCH_B2),
    };
    (m, found)
}

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

#[test]
fn the_firmware_finds_the_controller_by_its_class_code_alone() {
    let (_m, f) = run();
    assert!(!f.found_carry, "`B103h` answered: {f:?}");
    assert_eq!(
        f.found & 0xff,
        u16::from(IDE_DEVFN),
        "device 1, function 0 — where the machine text puts it"
    );
    assert_eq!(f.found >> 8, 0, "on bus 0, the only bus this board has");
}

#[test]
fn the_register_sizes_and_reads_back_as_an_io_bar() {
    let (_m, f) = run();
    // Rev 2.1 §6.2.5.1: bit 0 set marks an I/O register, and the address bits
    // below the window size are hardwired to zero. An eight-byte window
    // therefore sizes to `fffffff9`.
    assert_eq!(f.bar_reset, 0x0000_0001, "out of reset only the marker bit");
    assert_eq!(f.bar_mask, 0xffff_fff9);
    assert_eq!(
        f.bar_placed,
        u32::from(FIRST_COMMAND) | 1,
        "and it reads back the base the guest put in it, plus the marker"
    );
}

/// **The deferral, observed by the guest that caused it.**
#[test]
fn the_window_appears_a_round_after_the_write_and_not_in_it() {
    let (_m, f) = run();
    assert_eq!(
        f.status_early, 0xff,
        "the configuration write completed and the register moved, but the map \
         has not caught up yet — an undecoded port reads as ones"
    );
    assert_ne!(
        f.poll_first, 0,
        "and it did catch up: the guest saw the port start answering"
    );
    assert!(
        f.poll_first < POLL_LIMIT,
        "within the bound, which is one scheduler round: {f:?}"
    );
}

#[test]
fn the_drive_answers_through_the_window_the_guest_placed() {
    let (_m, f) = run();
    // ATA/ATAPI-6 §7.15: a fitted, idle drive has DRDY and DSC set and BSY
    // clear. The exact byte is the drive's business; what this file asserts is
    // that it is neither an open bus nor an empty cable.
    assert_ne!(f.status_first, 0xff, "not an open bus");
    assert_ne!(f.status_first, 0x00, "and not an empty cable");
    assert_eq!(
        f.alt_status, f.status_first,
        "the alternate status register at BAR3 + 2 is the same eight bits — \
         which is the *PCI IDE Controller Specification*'s offset 2, reached \
         from a base the guest chose"
    );
    assert_eq!(
        f.control_hole, 0xff,
        "and nothing else in the four-byte control window decodes"
    );
    // A latch reads back what was put in it; an open bus reads as ones.
    assert_eq!(f.latch_a, u16::from(LATCH_A));
    assert_eq!(f.latch_b, u16::from(LATCH_B));
}

/// **The move**, which is what separates "placed once" from "the mapping
/// follows the register".
#[test]
fn moving_the_register_moves_the_ports_and_the_drive_does_not_notice() {
    let (m, f) = run();
    assert_ne!(f.poll_second, 0, "the new base started answering");
    assert_eq!(
        f.status_old, 0xff,
        "and the old one went dead, so the window moved rather than being \
         mapped twice"
    );
    assert_eq!(
        f.latch_a2,
        u16::from(LATCH_A),
        "the drive kept the register it was given before the move: the board's \
         decoder changed its mind and the drive never heard about it"
    );
    assert_eq!(f.latch_b2, u16::from(LATCH_B));

    // And the same thing seen from outside the guest, through the machine's own
    // I/O space — which is the check that the guest is reporting the board
    // rather than reciting its own arithmetic.
    assert_eq!(port(&m, FIRST_COMMAND + OFF_SECTOR_COUNT), 0xff);
    assert_eq!(
        port(&m, SECOND_COMMAND + OFF_SECTOR_COUNT),
        LATCH_A,
        "a debug read of the port the BAR now names"
    );
    assert_eq!(port(&m, SECOND_CONTROL + 2), port(&m, SECOND_COMMAND + 7));
}

#[test]
fn the_primary_channel_is_where_it_always_was() {
    // The point of the board text's surgery is that it touches one channel. A
    // firmware that boots a hard disk off the primary channel must be unable to
    // tell any of this happened.
    let (m, _f) = run();
    // The secondary channel's fixed decode is gone, which is what the board
    // text took away; the primary's is untouched, which is what it did not.
    assert_eq!(port(&m, 0x0170), 0xff, "the secondary channel left 0x170");
    assert_eq!(port(&m, 0x0376), 0xff, "and 0x376");
    assert_ne!(
        port(&m, SECOND_COMMAND + 7),
        0xff,
        "while the window it moved to answers"
    );
}
