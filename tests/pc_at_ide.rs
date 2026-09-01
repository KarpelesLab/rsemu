//! A hard disk on the PC/AT board, driven the way firmware drives one.
//!
//! `tests/pc_at_board.rs` proves the chipset fits together. This proves the one
//! thing that board could not do until now: boot from something other than a
//! floppy. Everything here goes through the I/O ports — `0x1f0-0x1f7` and
//! `0x3f6` — because that is all a BIOS has, and nothing here reaches into
//! either device's internals except to check the *medium*, which is the point:
//!
//! * `IDENTIFY DEVICE` returning 256 plausible words is the weakest claim, and
//!   is asserted first because everything else depends on the geometry it
//!   reports.
//! * A sector read into guest memory is a real one.
//! * A sector **written** through the ports and then checked against the drive's
//!   medium — not against the drive's own buffer, which would pass on a model
//!   that never touched a medium at all — is the one that proves the model.
//!
//! All three are here, in that order.

#![cfg(all(
    feature = "cpu-x86",
    feature = "dev-pc",
    feature = "dev-pc-video",
    feature = "dev-pc-floppy",
    feature = "dev-pc-ide"
))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::device::ResetKind;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::x86::{Variant, X86};
use rsemu::dev::ata::AtaDisk;
use rsemu::machine::build;
use rsemu::machine::realize::Bindings;

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// How big the disk in bay 0 is. 2 MiB is 4096 sectors, which the default
/// translation covers as 4 cylinders of 16 heads of 63 sectors.
const SECTORS: u64 = 4096;

/// A firmware image that is not firmware: recognisable bytes, so the board has
/// something to fetch from without anything needing to execute meaningfully.
fn fake_bios(len: usize) -> Vec<u8> {
    (0..len).map(|i| (i % 251) as u8).collect()
}

/// A video option ROM header, which is all a scan looks at.
fn fake_vgabios(len: usize) -> Vec<u8> {
    let mut v = vec![0u8; len];
    v[0] = 0x55;
    v[1] = 0xaa;
    v[2] = (len / 512) as u8;
    v
}

/// What sector `lba` holds on the test disk.
///
/// Every sector says which sector it is, so a transfer that lands one sector
/// out — the classic failure of a CHS/LBA translation — fails rather than
/// passing on identical zeroes.
fn stamp(lba: u64) -> Vec<u8> {
    let mut out = vec![0u8; 512];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = (lba as u8) ^ (i as u8) ^ 0xa5;
    }
    out[0] = lba as u8;
    out[1] = (lba >> 8) as u8;
    out[510] = 0x55;
    out[511] = 0xaa;
    out
}

fn disk_image() -> Vec<u8> {
    let mut out = Vec::with_capacity((SECTORS * 512) as usize);
    for lba in 0..SECTORS {
        out.extend_from_slice(&stamp(lba));
    }
    out
}

fn bindings(cpus: &Arc<Captured<X86>>) -> Bindings {
    let mut b = Bindings::new();
    rsemu::machine::builtin::bind(&mut b).expect("ram and rom");
    rsemu::dev::pc::bind(&mut b).expect("the chipset");
    rsemu::dev::ata::bind(&mut b).expect("the hard disks");
    let kept = Arc::clone(cpus);
    b.bind("cpu.x86", move |props| {
        let cpu = Arc::new(X86::from_props_defaulting(props, Variant::I80486)?);
        kept.push(&cpu);
        Ok(cpu)
    })
    .expect("nothing else in this table claims the name");
    b
}

/// The board with a 2 MiB drive in the primary master bay, its processor, and
/// the drive itself — reached the way a *host* reaches one, out of the build's
/// own drive bay rather than through a back door in the adapter.
fn board() -> (rsemu::machine::Machine, Arc<X86>, Arc<AtaDisk>) {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let mut options = rsemu::machine::BuildOptions::new()
        .with_classes(rsemu::machine::catalog::classes())
        .with_bindings(bindings(&cpus));
    options.realize.media.insert("bios", fake_bios(128 * 1024));
    options
        .realize
        .media
        .insert("vgabios", fake_vgabios(32 * 1024));
    options.realize.media.insert("floppy", Vec::new());
    options.realize.media.insert("hd0", disk_image());
    // The second bay is empty, which is what most PCs of the period had.
    options.realize.media.insert("hd1", Vec::new());
    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut machine = match build("pc-at.machine", rsemu::dev::pc::PC_AT, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the board does not realize: {e}"),
    };
    let cpu = cpus.take().expect("the constructor kept a handle");
    let bay = rsemu::dev::ata::bays::get(&options.realize.hosts, "ide0-master")
        .expect("no other host object claimed the name")
        .expect("the channel and the drive both opened it");
    let drive = bay.drive().expect("the master bay is populated");
    machine.reset(ResetKind::Cold);
    machine.sweep();
    (machine, cpu, drive)
}

// ---------------------------------------------------------------------------
// the ports
// ---------------------------------------------------------------------------

/// The primary channel's command block.
const CMD: u64 = 0x1f0;
/// The primary channel's control block: device control on a write, alternate
/// status on a read.
const CTL: u64 = 0x3f6;

fn outb(m: &rsemu::machine::Machine, port: u64, value: u8) {
    m.space("port")
        .expect("the I/O space")
        .write(port, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a decoded port");
}

fn inb(m: &rsemu::machine::Machine, port: u64) -> u8 {
    m.space("port")
        .expect("the I/O space")
        .read(port, Width::U8, MemAttrs::DEFAULT)
        .expect("a decoded port") as u8
}

/// A 16-bit `IN` from the data port, which is how a sector actually moves.
fn inw(m: &rsemu::machine::Machine) -> u16 {
    m.space("port")
        .expect("the I/O space")
        .read(CMD, Width::U16, MemAttrs::DEFAULT)
        .expect("the data port takes a word") as u16
}

fn outw(m: &rsemu::machine::Machine, value: u16) {
    m.space("port")
        .expect("the I/O space")
        .write(CMD, Width::U16, u64::from(value), MemAttrs::DEFAULT)
        .expect("the data port takes a word");
}

fn peek(m: &rsemu::machine::Machine, addr: u64) -> u8 {
    m.space("mem")
        .expect("the memory space")
        .read(addr, Width::U8, MemAttrs::DEFAULT)
        .expect("a mapped byte") as u8
}

fn poke(m: &rsemu::machine::Machine, addr: u64, value: u8) {
    m.space("mem")
        .expect("the memory space")
        .write(addr, Width::U8, u64::from(value), MemAttrs::DEFAULT)
        .expect("a mapped byte");
}

/// The eight command block registers, by the offset the AT decodes them at.
/// Offset 0 is the data port; the helpers above name it `CMD` directly,
/// because a word access there is not the same shape as a byte access to a
/// task file register.
const _DATA: u64 = 0;
const ERROR: u64 = 1;
const COUNT: u64 = 2;
const LBA_LOW: u64 = 3;
const LBA_MID: u64 = 4;
const LBA_HIGH: u64 = 5;
const DEVICE: u64 = 6;
const STATUS: u64 = 7;

const ST_DRDY: u8 = 0x40;
const ST_DSC: u8 = 0x10;
const ST_DRQ: u8 = 0x08;
const ST_BSY: u8 = 0x80;
const ST_ERR: u8 = 0x01;

/// Wait for the drive the way a driver does: spin on `BSY`, then on `DRQ`.
///
/// Bounded, because a model that never lowered `BSY` should fail the test
/// rather than hang the suite — and because the spin is the thing being
/// asserted, not the wait.
fn wait_for_drq(m: &rsemu::machine::Machine) {
    for _ in 0..1000 {
        let status = inb(m, CTL); // the alternate status: look, do not acknowledge
        if status & ST_BSY != 0 {
            continue;
        }
        assert_eq!(
            status & ST_ERR,
            0,
            "the drive reported an error: {status:#04x}"
        );
        if status & ST_DRQ != 0 {
            return;
        }
    }
    panic!("the drive never raised DRQ");
}

/// Read one sector at `lba` into guest memory at `dest`, the way an `INT 13h`
/// handler's LBA path does: program the command block, spin, then `rep insw`.
fn read_sector_to_memory(m: &rsemu::machine::Machine, lba: u32, dest: u64) {
    outb(m, CMD + DEVICE, 0xe0 | ((lba >> 24) as u8 & 0x0f));
    outb(m, CMD + COUNT, 1);
    outb(m, CMD + LBA_LOW, lba as u8);
    outb(m, CMD + LBA_MID, (lba >> 8) as u8);
    outb(m, CMD + LBA_HIGH, (lba >> 16) as u8);
    outb(m, CMD + STATUS, 0x20); // READ SECTOR(S)
    wait_for_drq(m);
    for i in 0..256u64 {
        let word = inw(m);
        poke(m, dest + i * 2, word as u8);
        poke(m, dest + i * 2 + 1, (word >> 8) as u8);
    }
}

/// The 256-word `IDENTIFY DEVICE` response, read through the ports.
fn identify(m: &rsemu::machine::Machine) -> Vec<u16> {
    outb(m, CMD + DEVICE, 0xa0);
    outb(m, CMD + STATUS, 0xec);
    wait_for_drq(m);
    (0..256).map(|_| inw(m)).collect()
}

/// An ATA ASCII field, unswapped back into something readable.
fn text(words: &[u16]) -> String {
    let mut out = String::new();
    for word in words {
        out.push((word >> 8) as u8 as char);
        out.push(*word as u8 as char);
    }
    out.trim_end().to_string()
}

// ---------------------------------------------------------------------------
// the tests
// ---------------------------------------------------------------------------

#[test]
fn the_board_decodes_both_channels_and_leaves_the_floppy_where_it_was() {
    // The primary channel shares the `0x3f0` neighbourhood with the floppy
    // adapter, and getting that split wrong is silent: one of the two answers
    // and the other does not.
    let (m, _cpu, _drive) = board();

    // A drive is there and says so.
    outb(&m, CMD + DEVICE, 0xa0);
    assert_eq!(
        inb(&m, CMD + STATUS),
        ST_DRDY | ST_DSC,
        "the master did not answer at 0x1f7"
    );

    // The secondary channel has nothing on its cable at all, so nothing drives
    // the bus and the ISA pull-ups win.
    assert_eq!(inb(&m, 0x177), 0xff, "an empty channel must read as ones");
    assert_eq!(inb(&m, 0x376), 0xff);

    // And the floppy controller still answers on both sides of the hole its
    // window now has at 0x3f6.
    outb(&m, 0x3f2, 0x0c); // out of reset
    assert_eq!(inb(&m, 0x3f4) & 0xc0, 0x80, "the uPD765's main status");
    let dir = inb(&m, 0x3f7);
    assert_eq!(dir & 0x80, 0x80, "the floppy's disk-change bit at 0x3f7");
}

#[test]
fn identify_device_agrees_with_the_disk_the_board_was_given() {
    // The weakest of the three claims, and the one everything else rests on: if
    // the geometry a BIOS reads here is not the geometry the drive decodes CHS
    // against, a disk that reads fine through one path is garbage through the
    // other.
    let (m, _cpu, drive) = board();
    let w = identify(&m);

    assert_eq!(w[0] & 0x8000, 0, "bit 15 clear means an ATA device");
    assert_eq!(text(&w[27..47]), "RSEMU HARDDISK");
    assert_eq!(
        u32::from(w[60]) | (u32::from(w[61]) << 16),
        SECTORS as u32,
        "the capacity the board's image gave it"
    );
    // The default translation, and the current one, which start equal.
    assert_eq!((w[1], w[3], w[6]), (4, 16, 63));
    assert_eq!((w[54], w[55], w[56]), (4, 16, 63));
    let geometry = drive.current_geometry();
    assert_eq!(
        (geometry.cylinders, geometry.heads, geometry.sectors),
        (w[54], w[55] as u8, w[56] as u8),
        "IDENTIFY and the drive must not be describing two different disks"
    );
    assert_eq!(inb(&m, CMD + STATUS), ST_DRDY | ST_DSC, "and it is done");
}

#[test]
fn a_sector_travels_from_the_disk_into_guest_memory() {
    // The real claim. Nothing here knows what is inside either device; it is
    // written the way an INT 13h handler is written, through the ports.
    let (m, _cpu, _drive) = board();
    const DEST: u64 = 0x0000_7c00; // where a BIOS puts a boot sector

    read_sector_to_memory(&m, 0, DEST);
    let expected = stamp(0);
    for offset in [0u64, 1, 2, 100, 255, 509, 510, 511] {
        assert_eq!(
            peek(&m, DEST + offset),
            expected[offset as usize],
            "byte {offset} of sector 0"
        );
    }
    assert_eq!(peek(&m, DEST + 510), 0x55, "the boot signature");
    assert_eq!(peek(&m, DEST + 511), 0xaa);

    // And a sector well inside the disk, so that "it read *a* sector" and "it
    // read *the* sector" are distinguishable.
    read_sector_to_memory(&m, 1234, DEST);
    assert_eq!(peek(&m, DEST), 1234u64 as u8);
    assert_eq!(peek(&m, DEST + 1), (1234u32 >> 8) as u8);
    assert_eq!(peek(&m, DEST + 100), stamp(1234)[100]);
}

#[test]
fn the_chs_path_and_the_lba_path_read_the_same_sector() {
    // A BIOS reads in CHS and an operating system reads in LBA, off one disk.
    // This is the test that says they agree, driven entirely through the ports
    // so that the geometry `IDENTIFY` reported and the geometry the drive
    // decodes against are the same one.
    let (m, _cpu, _drive) = board();
    let w = identify(&m);
    let (heads, spt) = (u32::from(w[55]), u32::from(w[56]));

    const LBA: u32 = 1000;
    let sector = LBA % spt + 1;
    let head = (LBA / spt) % heads;
    let cylinder = LBA / spt / heads;

    const A: u64 = 0x0000_8000;
    const B: u64 = 0x0000_8200;
    read_sector_to_memory(&m, LBA, A);

    outb(&m, CMD + DEVICE, 0xa0 | (head as u8 & 0x0f)); // no LBA bit: CHS
    outb(&m, CMD + COUNT, 1);
    outb(&m, CMD + LBA_LOW, sector as u8);
    outb(&m, CMD + LBA_MID, cylinder as u8);
    outb(&m, CMD + LBA_HIGH, (cylinder >> 8) as u8);
    outb(&m, CMD + STATUS, 0x20);
    wait_for_drq(&m);
    for i in 0..256u64 {
        let word = inw(&m);
        poke(&m, B + i * 2, word as u8);
        poke(&m, B + i * 2 + 1, (word >> 8) as u8);
    }

    for offset in 0..512u64 {
        assert_eq!(
            peek(&m, A + offset),
            peek(&m, B + offset),
            "CHS ({cylinder}/{head}/{sector}) and LBA {LBA} disagree at byte {offset}"
        );
    }
    assert_eq!(
        peek(&m, A),
        u64::from(LBA) as u8,
        "and both read the sector asked for"
    );
}

#[test]
fn a_sector_written_through_the_ports_reaches_the_image() {
    // The claim that proves the model rather than the buffer. The bytes go out
    // through port 0x1f0 and are then checked against the drive's **medium**,
    // which a device that never wrote one would fail — and then read back
    // through the ports as well, which a device that wrote the medium at the
    // wrong offset would fail.
    let (m, _cpu, drive) = board();
    const LBA: u32 = 2000;
    const SRC: u64 = 0x0000_9000;

    // Something recognisably not what is already there.
    let payload: Vec<u8> = (0..512u32).map(|i| (i * 7 + 3) as u8).collect();
    for (i, byte) in payload.iter().enumerate() {
        poke(&m, SRC + i as u64, *byte);
    }

    outb(&m, CMD + DEVICE, 0xe0);
    outb(&m, CMD + COUNT, 1);
    outb(&m, CMD + LBA_LOW, LBA as u8);
    outb(&m, CMD + LBA_MID, (LBA >> 8) as u8);
    outb(&m, CMD + LBA_HIGH, 0);
    outb(&m, CMD + STATUS, 0x30); // WRITE SECTOR(S)
    wait_for_drq(&m);
    for i in 0..256u64 {
        let lo = u16::from(peek(&m, SRC + i * 2));
        let hi = u16::from(peek(&m, SRC + i * 2 + 1));
        outw(&m, lo | (hi << 8));
    }
    let status = inb(&m, CMD + STATUS);
    assert_eq!(status & ST_ERR, 0, "the write failed: {status:#04x}");
    assert_eq!(status & ST_DRQ, 0, "and it wants no more data");

    // Against the image, not against the drive's own buffer.
    let mut on_disk = vec![0u8; 512];
    drive
        .read_media(u64::from(LBA) * 512, &mut on_disk)
        .expect("in range");
    assert_eq!(on_disk, payload, "the sector never reached the medium");

    // The neighbours are untouched, which catches a byte offset computed from
    // the wrong sector.
    let mut before = vec![0u8; 512];
    drive
        .read_media(u64::from(LBA - 1) * 512, &mut before)
        .expect("in range");
    assert_eq!(before, stamp(u64::from(LBA) - 1));

    // And it reads back through the ports.
    const DEST: u64 = 0x0000_9400;
    read_sector_to_memory(&m, LBA, DEST);
    for offset in 0..512u64 {
        assert_eq!(peek(&m, DEST + offset), payload[offset as usize]);
    }
}

#[test]
fn the_alternate_status_port_looks_and_the_status_port_acknowledges() {
    // `0x3f6` exists on real hardware precisely so that a driver — and a
    // debugger — can read the status without clearing the interrupt, and this
    // is that difference asserted from the guest's side of the bus.
    let (m, _cpu, drive) = board();
    outb(&m, CMD + DEVICE, 0xe0);
    outb(&m, CMD + COUNT, 1);
    outb(&m, CMD + LBA_LOW, 5);
    outb(&m, CMD + LBA_MID, 0);
    outb(&m, CMD + LBA_HIGH, 0);
    outb(&m, CMD + STATUS, 0x20);

    assert!(drive.irq_asserted(), "the read raised INTRQ");
    for _ in 0..8 {
        assert_ne!(inb(&m, CTL) & ST_DRQ, 0);
        assert!(
            drive.irq_asserted(),
            "reading 0x3f6 acknowledged an interrupt it must not touch"
        );
    }
    let _ = inb(&m, CMD + STATUS);
    assert!(!drive.irq_asserted(), "reading 0x1f7 must acknowledge");

    // A debug read of either does neither, which is what a monitor and the gdb
    // stub do to a running machine.
    outb(&m, CMD + STATUS, 0x20);
    assert!(drive.irq_asserted());
    let port = m.space("port").expect("the I/O space");
    for _ in 0..4 {
        let seen = port
            .read(0x1f7, Width::U8, MemAttrs::DEBUG)
            .expect("a decoded port") as u8;
        assert_eq!(seen, ST_DRDY | ST_DSC | ST_DRQ);
        assert!(drive.irq_asserted(), "a debug read of 0x1f7 acknowledged");
    }
    let first = port.read(CMD, Width::U16, MemAttrs::DEBUG).expect("legal");
    let again = port.read(CMD, Width::U16, MemAttrs::DEBUG).expect("legal");
    assert_eq!(first, again, "a debug read of 0x1f0 advanced the buffer");
}

#[test]
fn irq_14_reaches_the_processor_and_acknowledges_to_its_vector() {
    // The other half of a working disk: a driver that sleeps until the drive
    // says the data is ready. IRQ 14 is the slave 8259A's IR6, so this travels
    // ide0 → pic2 → pic1's IR2 → the processor's INTR pin, and the acknowledge
    // cycle comes back down both controllers to fetch 0x70 + 6.
    let (m, cpu, _drive) = board();
    assert!(!cpu.intr_asserted(), "nothing is pending at power on");

    // The master: vector base 0x08, a slave on IR2, 8086 mode, and only IR2
    // unmasked.
    outb(&m, 0x20, 0x11);
    outb(&m, 0x21, 0x08);
    outb(&m, 0x21, 0x04);
    outb(&m, 0x21, 0x01);
    outb(&m, 0x21, 0xfb);
    // The slave: vector base 0x70, cascade identity 2, 8086 mode, only IR6.
    outb(&m, 0xa0, 0x11);
    outb(&m, 0xa1, 0x70);
    outb(&m, 0xa1, 0x02);
    outb(&m, 0xa1, 0x01);
    outb(&m, 0xa1, 0xbf);

    outb(&m, CMD + DEVICE, 0xe0);
    outb(&m, CMD + COUNT, 1);
    outb(&m, CMD + LBA_LOW, 3);
    outb(&m, CMD + LBA_MID, 0);
    outb(&m, CMD + LBA_HIGH, 0);
    outb(&m, CMD + STATUS, 0x20);

    assert!(
        cpu.intr_asserted(),
        "the drive's INTRQ never reached the processor's pin"
    );
    assert_eq!(
        cpu.acknowledge(),
        0x76,
        "IRQ 14 is the slave's IR6, so 0x70 + 6"
    );
    assert!(!cpu.intr_asserted(), "the request is now in service");

    // And the driver's own acknowledge — reading the status register — takes
    // the line down, so the next command's interrupt is a fresh edge.
    let _ = inb(&m, CMD + STATUS);
    outb(&m, 0xa0, 0x20); // end of interrupt, slave
    outb(&m, 0x20, 0x20); // and master
    assert!(!cpu.intr_asserted());
}

#[test]
fn nien_keeps_the_line_down_without_losing_the_transfer() {
    // Every BIOS that polls rather than sleeping sets nIEN first, and a model
    // that let the interrupt through anyway would leave a spurious IRQ 14
    // pending in an 8259A nobody has programmed yet.
    let (m, cpu, drive) = board();
    outb(&m, 0x20, 0x11);
    outb(&m, 0x21, 0x08);
    outb(&m, 0x21, 0x04);
    outb(&m, 0x21, 0x01);
    outb(&m, 0x21, 0xfb);
    outb(&m, 0xa0, 0x11);
    outb(&m, 0xa1, 0x70);
    outb(&m, 0xa1, 0x02);
    outb(&m, 0xa1, 0x01);
    outb(&m, 0xa1, 0xbf);

    outb(&m, CTL, 0x02); // nIEN
    outb(&m, CMD + DEVICE, 0xe0);
    outb(&m, CMD + COUNT, 1);
    outb(&m, CMD + LBA_LOW, 8);
    outb(&m, CMD + LBA_MID, 0);
    outb(&m, CMD + LBA_HIGH, 0);
    outb(&m, CMD + STATUS, 0x20);

    assert!(!cpu.intr_asserted(), "nIEN did not hold the line down");
    assert!(!drive.irq_asserted());
    wait_for_drq(&m);
    let sector: Vec<u8> = (0..256).flat_map(|_| inw(&m).to_le_bytes()).collect();
    assert_eq!(sector, stamp(8), "and the data came through regardless");
}

#[test]
fn a_software_reset_through_0x3f6_leaves_the_ata_signature() {
    // Every driver's first act. The two cylinder bytes reading zero is what
    // says "ATA"; a packet device answers 0x14 / 0xeb there, which is how a
    // driver tells them apart — and this board has no packet device, on purpose.
    let (m, _cpu, _drive) = board();
    outb(&m, CTL, 0x04 | 0x02); // SRST, with interrupts off as a driver does
    assert_eq!(inb(&m, CTL) & ST_BSY, ST_BSY, "held in reset");
    outb(&m, CTL, 0x02);

    assert_eq!(inb(&m, CTL), ST_DRDY | ST_DSC);
    assert_eq!(inb(&m, CMD + ERROR), 0x01, "the diagnostic code");
    assert_eq!(inb(&m, CMD + COUNT), 0x01);
    assert_eq!(inb(&m, CMD + LBA_LOW), 0x01);
    assert_eq!(
        inb(&m, CMD + LBA_MID),
        0x00,
        "an ATA device, not a packet one"
    );
    assert_eq!(inb(&m, CMD + LBA_HIGH), 0x00);
}

#[test]
fn the_board_snapshots_and_restores_a_transfer_that_is_part_way_through() {
    // A transfer half way through its sector buffer is state, and the board's
    // hash has to agree about it. This stops in the middle of the second sector
    // of a three-sector read, which is the state a snapshot taken at rest
    // cannot reach.
    let (m, _cpu, _drive) = board();
    outb(&m, CMD + DEVICE, 0xe0);
    outb(&m, CMD + COUNT, 3);
    outb(&m, CMD + LBA_LOW, 60);
    outb(&m, CMD + LBA_MID, 0);
    outb(&m, CMD + LBA_HIGH, 0);
    outb(&m, CMD + STATUS, 0x20);
    wait_for_drq(&m);
    let mut taken: Vec<u8> = Vec::new();
    for _ in 0..(256 + 77) {
        taken.extend_from_slice(&inw(&m).to_le_bytes());
    }
    assert_eq!(&taken[..512], &stamp(60)[..]);

    let image = m.save().expect("the board saves");
    let (mut other, _, _) = board();
    other.load(&image).expect("the board loads");
    assert_eq!(
        other.state_hash().expect("a hash"),
        m.state_hash().expect("a hash"),
        "a restored board must be indistinguishable from the one it came from"
    );

    // And the restored board carries on from the 78th word of sector 61.
    let mut rest: Vec<u8> = Vec::new();
    for _ in 0..(256 - 77 + 256) {
        rest.extend_from_slice(&inw(&other).to_le_bytes());
    }
    assert_eq!(&rest[..(256 - 77) * 2], &stamp(61)[154..]);
    assert_eq!(&rest[(256 - 77) * 2..], &stamp(62)[..]);
    assert_eq!(inb(&other, CTL), ST_DRDY | ST_DSC, "and then it is done");
}

#[test]
fn an_empty_bay_is_a_bay_with_nothing_in_it() {
    // The second position on the primary cable has no drive, and the drive that
    // *is* there answers for it with zeroes — which is exactly how a driver
    // decides there is nothing at that address. Ones would look like a drive
    // that is permanently busy and never finishes booting.
    let (m, _cpu, _drive) = board();
    outb(&m, CMD + DEVICE, 0xb0); // DEV set: device 1
    assert_eq!(inb(&m, CMD + STATUS), 0x00, "no drive is fitted there");
    assert_eq!(inb(&m, CTL), 0x00);
    outb(&m, CMD + DEVICE, 0xa0);
    assert_eq!(inb(&m, CMD + STATUS), ST_DRDY | ST_DSC, "and one is here");
}
