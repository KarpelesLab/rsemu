//! A CD-ROM on the PC/AT board, driven the way a driver drives one.
//!
//! **No firmware.** The image in the ROM socket is assembled by this file and
//! does nothing but talk to `0x170-0x177`: there is no `INT 13h` here, no POST,
//! and nothing from `src/fw/pcbios` is linked into the test at all. What is
//! being checked is the *device* — the packet handshake and the command set —
//! and a firmware in the middle would be a second thing that could be wrong.
//! `tests/pc_at_eltorito.rs` is the other half, where the firmware boots one.
//!
//! The guest is a sixteen-bit program that does exactly what `ATA/ATAPI-6`
//! §9.10 says a host does, in order, leaving its answers in a scratch block the
//! test then reads:
//!
//! 1. `IDENTIFY PACKET DEVICE`, which is an ordinary PIO data-in command and
//!    not a packet one — 256 words, and word 0 is what says this is a CD-ROM.
//! 2. `TEST UNIT READY`, which **fails**: a drive that has just been powered on
//!    owes the host a unit attention, and reporting it once is the behaviour
//!    being asserted.
//! 3. `REQUEST SENSE`, which says why and clears it.
//! 4. `TEST UNIT READY` again, which now succeeds.
//! 5. `INQUIRY`, `READ CD-ROM CAPACITY`, and two `READ(10)`s — one for the
//!    primary volume descriptor at logical block 16 and one for the file.
//!
//! The disc is an ISO 9660 image this test builds, writes into a temporary
//! directory and reads back. Nothing is vendored and nothing is fetched.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide",
    feature = "dev-ata-atapi",
    feature = "machine-pc-at"
))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::fw::asm16::{AL, AX, Asm, CH, CL, CX, Cc, DI, DS, DX, ES, Mem, SI, SP, SS, Shift};
use rsemu::machine::realize::Bindings;
use rsemu::machine::{Machine, build};

mod iso9660;

// ---------------------------------------------------------------------------
// where the guest leaves its answers
// ---------------------------------------------------------------------------

/// The CD-ROM's command block: the secondary channel, which is where
/// `machines/pc-at.machine` puts the drive.
const CD: u16 = 0x0170;

/// A progress counter the guest bumps at each step, so a test that finds the
/// wrong bytes can say *which* step never ran.
const STEP: u16 = 0x0500;
/// The completion status of each of the six packet commands, in order.
const STATUS_TUR1: u16 = 0x0502;
const STATUS_SENSE: u16 = 0x0503;
const STATUS_TUR2: u16 = 0x0504;
const STATUS_INQUIRY: u16 = 0x0505;
const STATUS_CAPACITY: u16 = 0x0506;
const STATUS_READ_PVD: u16 = 0x0507;
const STATUS_READ_FILE: u16 = 0x0508;
/// The interrupt reason register as the last command left it.
const REASON: u16 = 0x0509;
/// Where the guest assembles each command descriptor block before sending it.
const CDB: u16 = 0x0600;

/// The 512-byte `IDENTIFY PACKET DEVICE` response.
const IDENTIFY: u16 = 0x1000;
/// The eighteen-byte `REQUEST SENSE` response.
const SENSE: u16 = 0x1200;
/// The 36-byte `INQUIRY` response.
const INQUIRY: u16 = 0x1240;
/// The eight-byte `READ CD-ROM CAPACITY` response.
const CAPACITY: u16 = 0x1280;
/// The primary volume descriptor, logical block 16.
const PVD: u16 = 0x2000;
/// The one file's block.
const FILE: u16 = 0x2800;

/// What the guest writes to [`STEP`] once it has finished.
const FINISHED: u16 = 0x600d;

/// What the one file on the disc says.
const FILE_TEXT: &[u8] = b"a disc rsemu built for itself\n";

// ---------------------------------------------------------------------------
// the guest
// ---------------------------------------------------------------------------

/// Spin until `BSY` clears. Clobbers `AL` and `DX`.
fn wait_not_busy(a: &mut Asm) {
    a.movi(DX, CD + 7);
    let poll = a.here_label();
    a.in_al_dx();
    a.testi8(AL, 0x80);
    a.jcc(Cc::NE, poll);
}

/// Emit the whole of one packet command: ATA/ATAPI-6 §9.10, steps 1 to 4.
///
/// `bcl` is the byte count limit, `dest` where whatever comes back is put, and
/// `status_at` where the completion Status register is recorded. Nothing here
/// waits on `INTRQ`: `IDENTIFY PACKET DEVICE` word 0 reports microprocessor
/// DRQ, which is the promise that a host may poll.
fn packet(a: &mut Asm, cdb: &[u8], bcl: u16, dest: u16, status_at: u16) {
    for (i, byte) in cdb.iter().enumerate() {
        a.movmi8(Mem::abs(CDB + i as u16), *byte);
    }
    for i in cdb.len()..12 {
        a.movmi8(Mem::abs(CDB + i as u16), 0);
    }

    // Step 1: the device, the features, the byte count limit, then PACKET.
    a.movi(DX, CD + 6);
    a.movi8(AL, 0xa0);
    a.out_dx_al();
    wait_not_busy(a);
    a.movi(DX, CD + 1);
    a.movi8(AL, 0x00);
    a.out_dx_al();
    a.movi(DX, CD + 4);
    a.movi8(AL, bcl as u8);
    a.out_dx_al();
    a.movi(DX, CD + 5);
    a.movi8(AL, (bcl >> 8) as u8);
    a.out_dx_al();
    a.movi(DX, CD + 7);
    a.movi8(AL, 0xa0);
    a.out_dx_al();

    // Step 2 and 3: DRQ with C/D set, then six words of command packet.
    wait_not_busy(a);
    a.movi(SI, CDB);
    a.movi(CX, 6);
    a.movi(DX, CD);
    a.rep();
    a.outsw();

    // Step 4: every block the device offers, then completion.
    a.movi(DI, dest);
    let next = a.here_label();
    let done = a.label();
    wait_not_busy(a);
    a.movi(DX, CD + 7);
    a.in_al_dx();
    a.movto8(Mem::abs(status_at), AL);
    a.testi8(AL, 0x08);
    a.jcc(Cc::E, done);
    a.movi(DX, CD + 4);
    a.in_al_dx();
    a.mov8(CL, AL);
    a.movi(DX, CD + 5);
    a.in_al_dx();
    a.mov8(CH, AL);
    a.shift(Shift::SHR, CX, 1);
    a.movi(DX, CD);
    a.rep();
    a.insw();
    a.jmp(next);
    a.bind(done);
    // The interrupt reason as the command ended, which §9.10 says is C/D and
    // I/O both set.
    a.movi(DX, CD + 2);
    a.in_al_dx();
    a.movto8(Mem::abs(REASON), AL);
}

/// Bump the progress counter.
fn step(a: &mut Asm, n: u16) {
    a.movmi(Mem::abs(STEP), n);
}

/// A 128 KiB ROM image whose reset vector runs the program above.
///
/// It is a ROM because an x86 starts at `F000:FFF0` and there is nowhere else
/// to put the first instruction. It is not *firmware*: there is no interrupt
/// vector table, no BIOS Data Area and no service in it.
#[allow(clippy::too_many_lines)]
fn guest_rom() -> Vec<u8> {
    // Assembled as though it were the whole 64 KiB segment at F000, which is
    // what the labels are relative to; the bottom half is then dropped, exactly
    // as `pc.rom` aligns a 64 KiB image against the top of its window.
    let mut a = Asm::new(0x1_0000, 0xff);
    let entry = a.here_label();

    a.cli();
    a.cld();
    a.movi(AX, 0);
    a.movsr(DS, AX);
    a.movsr(ES, AX);
    a.movsr(SS, AX);
    a.movi(SP, 0x7c00);
    step(&mut a, 1);

    // -- IDENTIFY PACKET DEVICE ---------------------------------------------
    //
    // Not a packet command: an ordinary PIO data-in, 256 words, and the one
    // thing a driver can ask a packet device without sending it a packet.
    a.movi(DX, CD + 6);
    a.movi8(AL, 0xa0);
    a.out_dx_al();
    wait_not_busy(&mut a);
    a.movi(DX, CD + 7);
    a.movi8(AL, 0xa1);
    a.out_dx_al();
    wait_not_busy(&mut a);
    a.movi(DI, IDENTIFY);
    a.movi(CX, 256);
    a.movi(DX, CD);
    a.rep();
    a.insw();
    step(&mut a, 2);

    // -- the unit attention --------------------------------------------------
    packet(&mut a, &[0x00], 0, 0, STATUS_TUR1);
    step(&mut a, 3);
    packet(&mut a, &[0x03, 0, 0, 0, 18], 512, SENSE, STATUS_SENSE);
    step(&mut a, 4);
    packet(&mut a, &[0x00], 0, 0, STATUS_TUR2);
    step(&mut a, 5);

    // -- the command set -----------------------------------------------------
    packet(&mut a, &[0x12, 0, 0, 0, 36], 512, INQUIRY, STATUS_INQUIRY);
    step(&mut a, 6);
    packet(&mut a, &[0x25], 512, CAPACITY, STATUS_CAPACITY);
    step(&mut a, 7);
    // READ(10) of logical block 16, the primary volume descriptor. The byte
    // count limit is 512, so the 2048-byte block arrives in four goes — which
    // is the case a model with one block per DRQ would get wrong.
    packet(
        &mut a,
        &[0x28, 0, 0, 0, 0, 16, 0, 0, 1],
        512,
        PVD,
        STATUS_READ_PVD,
    );
    step(&mut a, 8);
    // And the file, in one go this time.
    packet(
        &mut a,
        &[0x28, 0, 0, 0, 0, iso9660::FILE_BLOCK as u8, 0, 0, 1],
        2048,
        FILE,
        STATUS_READ_FILE,
    );
    step(&mut a, FINISHED);

    let park = a.here_label();
    a.hlt();
    a.jmp(park);

    a.seek(0xfff0);
    a.jmpf_label(0xf000, entry);
    // The board's socket is 128 KiB and `pc.rom` aligns a short image against
    // the top of it, so a 64 KiB image is the top half — which is where `F000`
    // decodes and where the reset vector's far jump lands.
    a.finish()
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

/// A video option ROM header, which is all a scan looks at — and nothing here
/// scans, but the socket has to hold something plausible.
fn fake_vgabios(len: usize) -> Vec<u8> {
    let mut v = vec![0u8; len];
    v[0] = 0x55;
    v[1] = 0xaa;
    v[2] = (len / 512) as u8;
    v
}

/// The board with `disc` in the CD-ROM drive and the guest above in the ROM.
fn board(disc: Vec<u8>) -> (Machine, Arc<X86>) {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(bindings(&cpus));
    options.realize.media.insert("bios", guest_rom());
    options
        .realize
        .media
        .insert("vgabios", fake_vgabios(32 * 1024));
    options.realize.media.insert("floppy", Vec::new());
    options.realize.media.insert("hd0", Vec::new());
    options.realize.media.insert("hd1", Vec::new());
    options.realize.media.insert("cdrom", disc);
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut machine = match build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    let cpu = cpus.take().expect("the constructor kept a handle");
    machine.reset(ResetKind::Cold);
    machine.sweep();
    (machine, cpu)
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

fn bytes(m: &Machine, at: u16, len: usize) -> Vec<u8> {
    (0..len as u64)
        .map(|i| peek(m, u64::from(at) + i))
        .collect()
}

/// Long enough for every one of the eight steps, which between them are a few
/// thousand port accesses.
const RUN: GlobalTime = GlobalTime::from_nanos(50_000_000);

/// The disc every test here uses, read back out of the temporary directory it
/// was written to.
fn disc() -> Vec<u8> {
    let image = iso9660::data_disc(FILE_TEXT);
    let path = iso9660::write_temp("packet", &image);
    let back = std::fs::read(&path).expect("the disc image we just wrote");
    assert_eq!(back, image, "the image did not survive the round trip");
    let _ = std::fs::remove_file(&path);
    back
}

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

#[test]
fn a_guest_reads_an_iso_through_the_packet_interface() {
    let image = disc();
    let blocks = (image.len() / iso9660::BLOCK) as u32;
    let (mut m, _cpu) = board(image.clone());
    m.run_for(RUN).expect("the machine runs");

    assert_eq!(
        peek16(&m, STEP),
        FINISHED,
        "the guest stopped at step {}",
        peek16(&m, STEP)
    );

    // -- IDENTIFY PACKET DEVICE ---------------------------------------------
    let ident = bytes(&m, IDENTIFY, 512);
    let word0 = u16::from(ident[0]) | (u16::from(ident[1]) << 8);
    assert_eq!(word0 >> 14, 0b10, "word 0 does not say ATAPI");
    assert_eq!((word0 >> 8) & 0x1f, 5, "not the CD-ROM command packet set");
    assert_eq!(word0 & 0x80, 0x80, "not removable");
    assert_eq!(word0 & 3, 0, "not a twelve-byte command packet");
    assert_eq!(
        ident.iter().fold(0u8, |a, b| a.wrapping_add(*b)),
        0,
        "the IDENTIFY response does not checksum"
    );
    // Words 27-46, byte-swapped in pairs as ATA lays an ASCII field out.
    let model: String = ident[54..94]
        .chunks(2)
        .flat_map(|w| [w[1] as char, w[0] as char])
        .collect();
    assert_eq!(model.trim_end(), "RSEMU CD-ROM");

    // -- the unit attention, reported once ----------------------------------
    assert_eq!(
        peek(&m, u64::from(STATUS_TUR1)) & 0x01,
        0x01,
        "the first command after power-on did not report a unit attention"
    );
    let sense = bytes(&m, SENSE, 18);
    assert_eq!(sense[0], 0x70, "not current, fixed-format sense data");
    assert_eq!(sense[2] & 0x0f, 0x06, "not UNIT ATTENTION");
    assert_eq!((sense[12], sense[13]), (0x29, 0x00), "not RESET OCCURRED");
    assert_eq!(
        peek(&m, u64::from(STATUS_TUR2)) & 0x01,
        0x00,
        "the unit attention was reported twice"
    );
    // And that completion is C/D and I/O both set, which is what tells a driver
    // the register block holds a status rather than more data.
    assert_eq!(peek(&m, u64::from(REASON)) & 0x03, 0x03);

    // -- INQUIRY -------------------------------------------------------------
    assert_eq!(peek(&m, u64::from(STATUS_INQUIRY)) & 0x01, 0);
    let inquiry = bytes(&m, INQUIRY, 36);
    assert_eq!(inquiry[0] & 0x1f, 0x05, "peripheral device type: CD-ROM");
    assert_eq!(inquiry[1] & 0x80, 0x80, "removable");
    assert_eq!(inquiry[4], 31, "additional length");
    assert_eq!(&inquiry[8..13], b"RSEMU");
    assert_eq!(&inquiry[16..28], b"RSEMU CD-ROM");

    // -- READ CD-ROM CAPACITY ------------------------------------------------
    assert_eq!(peek(&m, u64::from(STATUS_CAPACITY)) & 0x01, 0);
    let capacity = bytes(&m, CAPACITY, 8);
    let last = u32::from_be_bytes([capacity[0], capacity[1], capacity[2], capacity[3]]);
    let size = u32::from_be_bytes([capacity[4], capacity[5], capacity[6], capacity[7]]);
    assert_eq!(last, blocks - 1, "the last block, not the count");
    assert_eq!(size, 2048);

    // -- READ(10) ------------------------------------------------------------
    assert_eq!(peek(&m, u64::from(STATUS_READ_PVD)) & 0x01, 0);
    let pvd = bytes(&m, PVD, 2048);
    let at = iso9660::PVD_BLOCK as usize * iso9660::BLOCK;
    assert_eq!(pvd, image[at..at + 2048], "the block came back wrong");
    assert_eq!(pvd[0], 1, "not a primary volume descriptor");
    assert_eq!(&pvd[1..6], b"CD001");
    assert_eq!(
        String::from_utf8_lossy(&pvd[40..72]).trim_end(),
        iso9660::VOLUME_ID
    );

    assert_eq!(peek(&m, u64::from(STATUS_READ_FILE)) & 0x01, 0);
    let file = bytes(&m, FILE, FILE_TEXT.len());
    assert_eq!(file, FILE_TEXT, "the file's own bytes did not come back");
}

#[test]
fn an_empty_drive_answers_and_says_there_is_no_disc() {
    // A CD-ROM with no disc in it is a CD-ROM, and the difference between that
    // and an empty cable is the whole reason the drive exists as an object
    // rather than as a property of an image.
    let (mut m, _cpu) = board(Vec::new());
    m.run_for(RUN).expect("the machine runs");
    assert_eq!(peek16(&m, STEP), FINISHED, "the guest did not finish");

    // The drive identified itself.
    let ident = bytes(&m, IDENTIFY, 512);
    let word0 = u16::from(ident[0]) | (u16::from(ident[1]) << 8);
    assert_eq!(
        word0 >> 14,
        0b10,
        "no drive answered IDENTIFY PACKET DEVICE"
    );

    // And then declined everything that needs a disc, with the sense that says
    // why rather than with an aborted command.
    for (what, at) in [
        ("TEST UNIT READY", STATUS_TUR2),
        ("READ CAPACITY", STATUS_CAPACITY),
        ("READ(10)", STATUS_READ_PVD),
    ] {
        assert_eq!(
            peek(&m, u64::from(at)) & 0x01,
            0x01,
            "{what} succeeded on an empty drive"
        );
    }
    // INQUIRY still answers: it describes the drive, not the disc.
    assert_eq!(peek(&m, u64::from(STATUS_INQUIRY)) & 0x01, 0);
    assert_eq!(bytes(&m, INQUIRY, 36)[0] & 0x1f, 0x05);
}
