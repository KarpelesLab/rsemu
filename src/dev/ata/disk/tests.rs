//! What a driver can actually do to an ATA drive.
//!
//! Everything here drives the model through the five methods a host adapter
//! has — [`AtaDisk::write_reg`], [`AtaDisk::read_reg`],
//! [`AtaDisk::read_alt_status`], [`AtaDisk::write_device_control`] and
//! [`AtaDisk::irq_asserted`] — because that is the whole of what is on the far
//! side of the cable and a test that reached past it would be testing something
//! no guest can do.
//!
//! The two claims that matter most are the ones a plausible-looking model gets
//! wrong: that a CHS command and an LBA command naming the same sector read the
//! same bytes, and that what a `WRITE SECTOR(S)` moved is on the **medium**
//! rather than only in the drive's own buffer.

use super::*;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

// ---------------------------------------------------------------------------
// rigs
// ---------------------------------------------------------------------------

/// A drive of `sectors` sectors at device 0, with LBA48 and a 16-sector
/// maximum multiple.
fn drive(sectors: u64) -> AtaDisk {
    let id = Identity::new(sectors, default_geometry(sectors), true, 16).expect("a valid drive");
    AtaDisk::with_identity(id, Position::Device0).expect("it fits in host memory")
}

/// A drive whose every sector says which sector it is, so a transfer that lands
/// one sector out fails rather than passing by luck.
fn stamped(sectors: u64) -> AtaDisk {
    let disk = drive(sectors);
    for lba in 0..sectors {
        disk.write_media(lba * SECTOR, &stamp(lba))
            .expect("in range");
    }
    disk
}

/// What sector `lba` holds on a [`stamped`] drive.
fn stamp(lba: u64) -> Vec<u8> {
    let mut out = alloc::vec![0u8; SECTOR as usize];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = (lba as u8) ^ (i as u8) ^ 0x5a;
    }
    out[0] = lba as u8;
    out[1] = (lba >> 8) as u8;
    out
}

/// Select device 0 in LBA mode, with `head` in the low four bits.
fn select_lba(disk: &AtaDisk, head: u8) {
    disk.write_reg(
        Reg::Device,
        u16::from(DEV_OBSOLETE | DEV_LBA | (head & DEV_HEAD)),
    );
}

/// Select device 0 in CHS mode.
fn select_chs(disk: &AtaDisk, head: u8) {
    disk.write_reg(Reg::Device, u16::from(DEV_OBSOLETE | (head & DEV_HEAD)));
}

/// The Status register, with the side effect it has on real hardware.
fn status(disk: &AtaDisk) -> u8 {
    disk.read_reg(Reg::Command, false) as u8
}

/// Empty the sector buffer through the data register, as `insw` would.
fn drain(disk: &AtaDisk, words: usize) -> Vec<u8> {
    let mut out = Vec::with_capacity(words * 2);
    for _ in 0..words {
        let word = disk.read_reg(Reg::Data, false);
        out.push(word as u8);
        out.push((word >> 8) as u8);
    }
    out
}

/// Fill the sector buffer through the data register, as `outsw` would.
fn fill(disk: &AtaDisk, bytes: &[u8]) {
    for pair in bytes.chunks(2) {
        let word = u16::from(pair[0]) | (u16::from(pair[1]) << 8);
        disk.write_reg(Reg::Data, word);
    }
}

/// One `IDENTIFY DEVICE` response, as words.
fn identify(disk: &AtaDisk) -> Vec<u16> {
    disk.write_reg(Reg::Command, u16::from(cmd::IDENTIFY));
    assert_ne!(
        disk.read_alt_status() & ST_DRQ,
        0,
        "IDENTIFY must raise DRQ"
    );
    let bytes = drain(disk, 256);
    bytes
        .chunks(2)
        .map(|p| u16::from(p[0]) | (u16::from(p[1]) << 8))
        .collect()
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
// IDENTIFY DEVICE
// ---------------------------------------------------------------------------

#[test]
fn identify_reports_the_drive_it_was_built_as() {
    // 4096 sectors is 2 MiB, which the default translation covers as
    // 4/16/63 = 4032 sectors — a translation that reaches less than the whole
    // drive, which is a property of CHS and not a defect.
    let disk = drive(4096);
    let w = identify(&disk);

    assert_eq!(w[0] & 0x8000, 0, "bit 15 clear means an ATA device");
    assert_eq!(w[1], 4, "default cylinders");
    assert_eq!(w[3], 16, "default heads");
    assert_eq!(w[6], 63, "default sectors per track");
    assert_eq!(w[54], 4, "the current translation starts at the default");
    assert_eq!(w[55], 16);
    assert_eq!(w[56], 63);
    assert_eq!(
        u32::from(w[60]) | (u32::from(w[61]) << 16),
        4096,
        "words 60-61 are the whole drive in 28-bit addressing"
    );
    assert_eq!(
        u64::from(w[100]) | (u64::from(w[101]) << 16),
        4096,
        "and words 100-103 are the same in 48-bit addressing"
    );
    assert_ne!(w[49] & (1 << 9), 0, "LBA is supported");
    assert_eq!(w[49] & (1 << 8), 0, "and DMA is not, because it is not");
    assert_ne!(w[83] & (1 << 10), 0, "the 48-bit Address feature set");
    assert_eq!(w[47] & 0xff, 16, "the largest READ/WRITE MULTIPLE block");
    assert_eq!(w[59], 0, "and multiple mode is off until SET MULTIPLE MODE");
}

#[test]
fn the_identify_strings_come_back_in_the_order_a_driver_prints_them() {
    // The classic: each ATA ASCII word holds its *first* character in the high
    // byte, so a model that writes them little-endian shows a driver
    // "SREMU AHDRSIDK".
    let disk = drive(4096);
    let w = identify(&disk);
    assert_eq!(text(&w[27..47]), "RSEMU HARDDISK");
    assert_eq!(text(&w[10..20]), "RSEMU00000000000001");
    assert_eq!(text(&w[23..27]), "1.0");
}

#[test]
fn the_identify_block_checksums_to_zero() {
    // Word 255 is a signature and a checksum, and a driver that validates it
    // rejects a block whose bytes do not sum to zero modulo 256.
    let disk = drive(4096);
    disk.write_reg(Reg::Command, u16::from(cmd::IDENTIFY));
    let bytes = drain(&disk, 256);
    assert_eq!(
        bytes[510], 0xa5,
        "the signature the checksum is valid under"
    );
    let sum = bytes.iter().fold(0u8, |a, b| a.wrapping_add(*b));
    assert_eq!(sum, 0);
}

#[test]
fn identify_packet_device_aborts_because_this_is_not_one() {
    // ATAPI is out of scope, and this is the falsifiable form of saying so:
    // the command a driver uses to ask "are you a packet device?" answers no.
    let disk = drive(4096);
    disk.write_reg(Reg::Command, u16::from(cmd::IDENTIFY_PACKET));
    let st = status(&disk);
    assert_ne!(st & ST_ERR, 0, "an aborted command sets ERR");
    assert_eq!(st & ST_DRQ, 0, "and hands over no data");
    assert_eq!(disk.read_reg(Reg::Feature, false) as u8, ERR_ABRT);
}

// ---------------------------------------------------------------------------
// addressing
// ---------------------------------------------------------------------------

#[test]
fn chs_and_lba_name_the_same_sector() {
    // The whole reason this device is hard to get right. A BIOS reads through
    // INT 13h in CHS and an operating system reads the same disk in LBA; if the
    // two disagree, one path reads a file and the other reads garbage, and
    // nothing but a test that does both catches it.
    let disk = stamped(4096);
    let geometry = disk.current_geometry();
    assert_eq!(
        (geometry.cylinders, geometry.heads, geometry.sectors),
        (4, 16, 63)
    );

    // LBA 1000 under 4/16/63: track 15, head 15, cylinder 0, and sector 56 —
    // because sectors are numbered from one, which is the off-by-one this
    // whole translation exists to get right.
    const LBA: u64 = 1000;
    let chs = Address::from_lba(LBA, &geometry).expect("it is inside the translation");
    assert_eq!(chs.to_lba(&geometry), Some(LBA), "the round trip closes");
    let Address::Chs {
        cylinder,
        head,
        sector,
    } = chs
    else {
        unreachable!("from_lba builds a CHS address")
    };
    assert_eq!((cylinder, head, sector), (0, 15, 56));

    // Read it in LBA.
    select_lba(&disk, ((LBA >> 24) & 0x0f) as u8);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, (LBA & 0xff) as u16);
    disk.write_reg(Reg::LbaMid, ((LBA >> 8) & 0xff) as u16);
    disk.write_reg(Reg::LbaHigh, ((LBA >> 16) & 0xff) as u16);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    let by_lba = drain(&disk, 256);

    // And in CHS.
    select_chs(&disk, head);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, u16::from(sector));
    disk.write_reg(Reg::LbaMid, cylinder & 0xff);
    disk.write_reg(Reg::LbaHigh, cylinder >> 8);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    let by_chs = drain(&disk, 256);

    assert_eq!(by_lba, stamp(LBA), "the LBA path read the wrong sector");
    assert_eq!(by_chs, by_lba, "CHS and LBA disagree about sector {LBA}");
}

#[test]
fn a_chs_sector_of_zero_is_an_address_no_translation_has() {
    // Sectors are numbered from one. A model that computed `sector` rather than
    // `sector - 1` would happily read something here.
    let disk = stamped(4096);
    select_chs(&disk, 0);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, 0);
    disk.write_reg(Reg::LbaMid, 0);
    disk.write_reg(Reg::LbaHigh, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    assert_ne!(status(&disk) & ST_ERR, 0);
    assert_eq!(disk.read_reg(Reg::Feature, false) as u8, ERR_IDNF);
}

#[test]
fn initialize_device_parameters_moves_the_translation_identify_reports() {
    // The command that makes a BIOS's arithmetic and the drive's agree. If
    // words 54-56 did not follow it, a BIOS would compute against one geometry
    // and the drive would decode against another — which is exactly the failure
    // that reads fine through one path and returns garbage through the other.
    let disk = stamped(4096);

    // Four heads and thirty-two sectors: 4096 / 128 = 32 cylinders.
    disk.write_reg(Reg::SectorCount, 32);
    disk.write_reg(Reg::Device, u16::from(DEV_OBSOLETE | 3)); // heads - 1
    disk.write_reg(Reg::Command, u16::from(cmd::INIT_DEVICE_PARAMS));
    assert_eq!(status(&disk) & ST_ERR, 0);

    let geometry = disk.current_geometry();
    assert_eq!(
        (geometry.cylinders, geometry.heads, geometry.sectors),
        (32, 4, 32)
    );
    let w = identify(&disk);
    assert_eq!((w[54], w[55], w[56]), (32, 4, 32), "words 54-56 follow it");
    assert_eq!((w[1], w[3], w[6]), (4, 16, 63), "and words 1/3/6 do not");
    assert_eq!(
        u32::from(w[57]) | (u32::from(w[58]) << 16),
        32 * 4 * 32,
        "words 57-58 are what the current translation addresses"
    );

    // And a CHS read now decodes against the new translation: cylinder 1,
    // head 0, sector 1 is LBA (1 * 4 + 0) * 32 + 0 = 128.
    select_chs(&disk, 0);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, 1);
    disk.write_reg(Reg::LbaMid, 1);
    disk.write_reg(Reg::LbaHigh, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    assert_eq!(drain(&disk, 256), stamp(128));
}

#[test]
fn the_48_bit_registers_are_two_deep_and_hob_chooses_which_half_reads_back() {
    // A drive big enough to *need* 48-bit addressing cannot be allocated in a
    // test — the medium is a flat `RamStore` — so the plumbing is asserted at a
    // low address instead: two writes fill the FIFO, the drive assembles a
    // 48-bit address out of both halves, and the Device Control register's HOB
    // bit is what reads the high one back.
    let disk = stamped(4096);
    select_lba(&disk, 0);
    // High half first, then low, which is the order the standard specifies.
    disk.write_reg(Reg::SectorCount, 0);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, 0); // bits 31:24
    disk.write_reg(Reg::LbaMid, 0); // bits 39:32
    disk.write_reg(Reg::LbaHigh, 0); // bits 47:40
    disk.write_reg(Reg::LbaLow, 200); // bits 7:0
    disk.write_reg(Reg::LbaMid, 0); // bits 15:8
    disk.write_reg(Reg::LbaHigh, 0); // bits 23:16

    // Before the command: HOB reads back the *previous* content.
    disk.write_device_control(CTL_HOB);
    assert_eq!(disk.read_reg(Reg::LbaLow, false) as u8, 0, "the high half");
    disk.write_device_control(0);
    assert_eq!(disk.read_reg(Reg::LbaLow, false) as u8, 200, "the low half");

    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS_EXT));
    assert_eq!(drain(&disk, 256), stamp(200));

    // And the pure translation, at an address no test can allocate.
    let far = Address::Lba48(0x0001_2345_6789);
    assert_eq!(
        far.to_lba(&Geometry {
            cylinders: 1,
            heads: 1,
            sectors: 1
        }),
        Some(0x0001_2345_6789)
    );
}

#[test]
fn a_read_past_the_end_reports_id_not_found_rather_than_reading_something() {
    let disk = stamped(64);
    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 4);
    disk.write_reg(Reg::LbaLow, 62);
    disk.write_reg(Reg::LbaMid, 0);
    disk.write_reg(Reg::LbaHigh, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    let st = status(&disk);
    assert_ne!(st & ST_ERR, 0, "a read off the end must fail");
    assert_eq!(st & ST_DRQ, 0, "and hand over nothing");
    assert_eq!(disk.read_reg(Reg::Feature, false) as u8, ERR_IDNF);
}

// ---------------------------------------------------------------------------
// the busy/DRQ dance
// ---------------------------------------------------------------------------

#[test]
fn a_read_announces_every_block_at_its_start() {
    let disk = stamped(4096);
    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 2);
    disk.write_reg(Reg::LbaLow, 10);
    disk.write_reg(Reg::LbaMid, 0);
    disk.write_reg(Reg::LbaHigh, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));

    // The first block is ready and announced before the write that started it
    // returned, which is what a zero-time model means and what makes every
    // spin-on-BSY loop terminate.
    assert_eq!(disk.read_alt_status() & ST_BSY, 0);
    assert_ne!(disk.read_alt_status() & ST_DRQ, 0);
    assert!(disk.irq_asserted(), "INTRQ at the start of the first block");

    // Reading the status register acknowledges it; the alternate status does
    // not, which is the entire reason the alternate status exists.
    assert_ne!(disk.read_alt_status() & ST_DRQ, 0);
    assert!(
        disk.irq_asserted(),
        "the alternate status acknowledges nothing"
    );
    status(&disk);
    assert!(!disk.irq_asserted(), "the status register acknowledges");

    assert_eq!(drain(&disk, 256), stamp(10));
    // The next sector is loaded and announced the moment the last word of the
    // previous one leaves.
    assert_ne!(disk.read_alt_status() & ST_DRQ, 0, "the second block");
    assert!(
        disk.irq_asserted(),
        "INTRQ at the start of the second block"
    );
    assert_eq!(drain(&disk, 256), stamp(11));

    assert_eq!(
        disk.read_alt_status(),
        ST_DRDY | ST_DSC,
        "and then it is done"
    );
    assert_eq!(disk.read_reg(Reg::SectorCount, false) as u8, 0);
}

#[test]
fn a_write_announces_every_block_at_its_end_and_the_first_one_not_at_all() {
    // The asymmetry that makes a model work with one driver and hang another.
    // A PIO data-out command raises DRQ with **no** interrupt and waits; the
    // interrupt comes when the block has been taken. A driver that waited for
    // an interrupt before writing the first block would hang on real hardware
    // too, which is why this must not be "helpfully" fixed.
    let disk = drive(4096);
    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 2);
    disk.write_reg(Reg::LbaLow, 20);
    disk.write_reg(Reg::LbaMid, 0);
    disk.write_reg(Reg::LbaHigh, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::WRITE_SECTORS));

    assert_ne!(
        disk.read_alt_status() & ST_DRQ,
        0,
        "ready for the first block"
    );
    assert!(!disk.irq_asserted(), "and no interrupt for it");

    fill(&disk, &stamp(20));
    assert!(disk.irq_asserted(), "INTRQ at the end of the first block");
    assert_ne!(disk.read_alt_status() & ST_DRQ, 0, "ready for the second");
    status(&disk);

    fill(&disk, &stamp(21));
    assert!(disk.irq_asserted(), "INTRQ at the end of the second block");
    assert_eq!(
        disk.read_alt_status(),
        ST_DRDY | ST_DSC,
        "and then it is done"
    );
}

#[test]
fn a_written_sector_is_on_the_medium_and_not_only_in_the_buffer() {
    // The claim that proves the model. Reading back what was just written
    // through the same buffer would pass on a device that never touched its
    // medium at all, so this looks at the medium directly — and then reads it
    // back through the protocol as well, so a medium write at the wrong offset
    // fails too.
    let disk = drive(4096);
    let payload = stamp(0x123);
    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, 0x23);
    disk.write_reg(Reg::LbaMid, 0x01);
    disk.write_reg(Reg::LbaHigh, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::WRITE_SECTORS));
    fill(&disk, &payload);
    assert_eq!(status(&disk) & ST_ERR, 0);

    let mut got = alloc::vec![0u8; SECTOR as usize];
    disk.read_media(0x123 * SECTOR, &mut got).expect("in range");
    assert_eq!(got, payload, "the sector did not reach the medium");
    // And the sector before it is untouched, which catches an off-by-one in
    // the byte offset.
    let mut before = alloc::vec![0u8; SECTOR as usize];
    disk.read_media(0x122 * SECTOR, &mut before)
        .expect("in range");
    assert!(before.iter().all(|b| *b == 0));

    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, 0x23);
    disk.write_reg(Reg::LbaMid, 0x01);
    disk.write_reg(Reg::LbaHigh, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    assert_eq!(drain(&disk, 256), payload);
}

#[test]
fn a_write_to_a_write_protected_drive_aborts_before_it_touches_anything() {
    let mut id = Identity::new(64, default_geometry(64), true, 16).expect("valid");
    id.read_only = true;
    let disk = AtaDisk::with_identity(id, Position::Device0).expect("it fits");
    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::WRITE_SECTORS));
    let st = status(&disk);
    assert_ne!(st & ST_ERR, 0);
    assert_eq!(st & ST_DRQ, 0, "and never asks for the data");
    assert_eq!(disk.read_reg(Reg::Feature, false) as u8, ERR_ABRT);
}

// ---------------------------------------------------------------------------
// multiple mode
// ---------------------------------------------------------------------------

#[test]
fn read_multiple_aborts_until_set_multiple_mode_has_run() {
    let disk = stamped(4096);
    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 4);
    disk.write_reg(Reg::LbaLow, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_MULTIPLE));
    assert_ne!(status(&disk) & ST_ERR, 0, "no block size has been agreed");
    assert_eq!(disk.read_reg(Reg::Feature, false) as u8, ERR_ABRT);
}

#[test]
fn set_multiple_mode_takes_a_power_of_two_and_nothing_else() {
    let disk = drive(4096);
    for bad in [0u16, 3, 17, 255] {
        disk.write_reg(Reg::SectorCount, bad);
        disk.write_reg(Reg::Command, u16::from(cmd::SET_MULTIPLE));
        assert_ne!(status(&disk) & ST_ERR, 0, "{bad} is not a block size");
        assert_eq!(disk.multiple(), 0, "and it must not have been taken");
    }
    disk.write_reg(Reg::SectorCount, 8);
    disk.write_reg(Reg::Command, u16::from(cmd::SET_MULTIPLE));
    assert_eq!(status(&disk) & ST_ERR, 0);
    assert_eq!(disk.multiple(), 8);
    assert_eq!(
        identify(&disk)[59],
        0x0108,
        "word 59 reports it, with bit 8"
    );
}

#[test]
fn read_multiple_moves_a_block_per_interrupt() {
    let disk = stamped(4096);
    disk.write_reg(Reg::SectorCount, 4);
    disk.write_reg(Reg::Command, u16::from(cmd::SET_MULTIPLE));
    assert_eq!(status(&disk) & ST_ERR, 0);

    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 6);
    disk.write_reg(Reg::LbaLow, 30);
    disk.write_reg(Reg::LbaMid, 0);
    disk.write_reg(Reg::LbaHigh, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_MULTIPLE));

    // Four sectors in one DRQ block, then the two that are left — a short last
    // block, which is what the standard says a residual count produces.
    assert!(disk.irq_asserted());
    status(&disk);
    let first = drain(&disk, 4 * 256);
    for lba in 30..34 {
        let at = (lba - 30) * SECTOR as usize;
        assert_eq!(&first[at..at + SECTOR as usize], &stamp(lba as u64)[..]);
    }
    assert!(disk.irq_asserted(), "the second block is announced");
    let second = drain(&disk, 2 * 256);
    assert_eq!(&second[..SECTOR as usize], &stamp(34)[..]);
    assert_eq!(&second[SECTOR as usize..], &stamp(35)[..]);
    assert_eq!(disk.read_alt_status(), ST_DRDY | ST_DSC);
}

#[test]
fn write_multiple_puts_a_whole_block_on_the_medium() {
    let disk = drive(4096);
    disk.write_reg(Reg::SectorCount, 4);
    disk.write_reg(Reg::Command, u16::from(cmd::SET_MULTIPLE));

    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 4);
    disk.write_reg(Reg::LbaLow, 40);
    disk.write_reg(Reg::LbaMid, 0);
    disk.write_reg(Reg::LbaHigh, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::WRITE_MULTIPLE));
    assert!(
        !disk.irq_asserted(),
        "still no interrupt for the first block"
    );
    let mut payload = Vec::new();
    for lba in 40..44 {
        payload.extend_from_slice(&stamp(lba));
    }
    fill(&disk, &payload);
    assert_eq!(status(&disk) & ST_ERR, 0);

    let mut got = alloc::vec![0u8; payload.len()];
    disk.read_media(40 * SECTOR, &mut got).expect("in range");
    assert_eq!(got, payload);
}

// ---------------------------------------------------------------------------
// MemAttrs::debug
// ---------------------------------------------------------------------------

#[test]
fn a_debug_read_neither_acknowledges_nor_advances() {
    // The two side effects a debugger must not cause, asserted one at a time
    // because a model that suppressed only one would still pass a test that
    // looked at the other.
    let disk = stamped(4096);
    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, 7);
    disk.write_reg(Reg::LbaMid, 0);
    disk.write_reg(Reg::LbaHigh, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));

    // Status, under debug: the same bits, and the interrupt still pending.
    let peeked = disk.read_reg(Reg::Command, true) as u8;
    assert_eq!(peeked, disk.read_alt_status());
    assert!(
        disk.irq_asserted(),
        "a debug read of status acknowledged it"
    );

    // Data, under debug: the same word twice, and the buffer where it was.
    let a = disk.read_reg(Reg::Data, true);
    let b = disk.read_reg(Reg::Data, true);
    assert_eq!(a, b, "a debug read of data advanced the buffer");

    // And the guest still gets the whole sector, from its first byte.
    assert_eq!(drain(&disk, 256), stamp(7));
}

// ---------------------------------------------------------------------------
// reset
// ---------------------------------------------------------------------------

#[test]
fn a_software_reset_leaves_the_ata_signature() {
    // 0x00 / 0x00 in the two cylinder bytes is what says "ATA"; a packet device
    // answers 0x14 / 0xeb there, and this is how a driver tells them apart.
    let disk = stamped(4096);
    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, 5);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    assert_ne!(disk.read_alt_status() & ST_DRQ, 0);

    disk.write_device_control(CTL_SRST);
    assert_eq!(disk.read_alt_status(), ST_BSY, "held in reset");
    assert!(!disk.irq_asserted());
    disk.write_device_control(0);

    assert_eq!(disk.read_alt_status(), ST_DRDY | ST_DSC);
    assert!(
        !disk.irq_asserted(),
        "a software reset asserts no interrupt"
    );
    assert_eq!(disk.read_reg(Reg::Feature, false) as u8, 0x01);
    assert_eq!(disk.read_reg(Reg::SectorCount, false) as u8, 0x01);
    assert_eq!(disk.read_reg(Reg::LbaLow, false) as u8, 0x01);
    assert_eq!(disk.read_reg(Reg::LbaMid, false) as u8, 0x00);
    assert_eq!(disk.read_reg(Reg::LbaHigh, false) as u8, 0x00);
}

#[test]
fn a_software_reset_keeps_what_the_host_configured_and_a_power_cycle_does_not() {
    // The one difference between the two that matters to a driver: SRST is a
    // bus-level reset and does not undo INITIALIZE DEVICE PARAMETERS or
    // SET MULTIPLE MODE, and a driver that reset the bus after configuring the
    // drive would otherwise silently lose both.
    let disk = drive(4096);
    disk.write_reg(Reg::SectorCount, 32);
    disk.write_reg(Reg::Device, u16::from(DEV_OBSOLETE | 3));
    disk.write_reg(Reg::Command, u16::from(cmd::INIT_DEVICE_PARAMS));
    disk.write_reg(Reg::SectorCount, 8);
    disk.write_reg(Reg::Command, u16::from(cmd::SET_MULTIPLE));

    disk.write_device_control(CTL_SRST);
    disk.write_device_control(0);
    assert_eq!(
        disk.current_geometry().heads,
        4,
        "SRST kept the translation"
    );
    assert_eq!(disk.multiple(), 8, "and the block size");

    disk.power_on_reset();
    assert_eq!(disk.current_geometry().heads, 16, "a power cycle did not");
    assert_eq!(disk.multiple(), 0);
}

#[test]
fn nien_holds_the_interrupt_off_without_losing_it() {
    let disk = stamped(4096);
    disk.write_device_control(CTL_NIEN);
    select_lba(&disk, 0);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, 1);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    assert_ne!(
        disk.read_alt_status() & ST_DRQ,
        0,
        "the data is still ready"
    );
    assert!(!disk.irq_asserted(), "nIEN gates the line");
    // Ungating it lets the pending request through, rather than having thrown
    // it away.
    disk.write_device_control(0);
    assert!(disk.irq_asserted());
}

// ---------------------------------------------------------------------------
// device selection
// ---------------------------------------------------------------------------

#[test]
fn a_drive_only_answers_when_the_dev_bit_names_it() {
    // Selection is the *drive's* decision, made by comparing the Device
    // register against the position it is jumpered to, which is what lets a
    // host adapter broadcast every write and know nothing about which bit it
    // was.
    let id = Identity::new(64, default_geometry(64), true, 16).expect("valid");
    let slave = AtaDisk::with_identity(id, Position::Device1).expect("it fits");
    assert!(!slave.is_selected(), "device 1 is not selected at power on");

    slave.write_reg(Reg::Device, u16::from(DEV_OBSOLETE | DEV_SELECT));
    assert!(slave.is_selected());
    // A write it should now take.
    slave.write_reg(Reg::SectorCount, 9);
    assert_eq!(slave.read_reg(Reg::SectorCount, false) as u8, 9);

    // Deselected, it ignores everything but the Device register.
    slave.write_reg(Reg::Device, u16::from(DEV_OBSOLETE));
    assert!(!slave.is_selected());
    slave.write_reg(Reg::SectorCount, 0x77);
    slave.write_reg(Reg::Device, u16::from(DEV_OBSOLETE | DEV_SELECT));
    assert_eq!(
        slave.read_reg(Reg::SectorCount, false) as u8,
        9,
        "a deselected drive took a write that was not addressed to it"
    );
}

// ---------------------------------------------------------------------------
// odds and ends a BIOS sends
// ---------------------------------------------------------------------------

#[test]
fn the_commands_old_firmware_still_sends_all_answer() {
    let disk = stamped(4096);
    select_lba(&disk, 0);
    for opcode in [
        cmd::RECALIBRATE,
        cmd::RECALIBRATE | 0x0f,
        cmd::SEEK,
        cmd::DIAGNOSTIC,
        cmd::FLUSH_CACHE,
        cmd::FLUSH_CACHE_EXT,
        cmd::STANDBY_IMMEDIATE,
        cmd::IDLE_IMMEDIATE,
        cmd::CHECK_POWER_MODE,
    ] {
        disk.write_reg(Reg::LbaLow, 1);
        disk.write_reg(Reg::LbaMid, 0);
        disk.write_reg(Reg::LbaHigh, 0);
        disk.write_reg(Reg::Command, u16::from(opcode));
        assert_eq!(
            status(&disk) & ST_ERR,
            0,
            "{opcode:#04x} should have succeeded"
        );
    }
    // And one that must not: NOP is specified to abort, name notwithstanding.
    disk.write_reg(Reg::Command, u16::from(cmd::NOP));
    assert_ne!(status(&disk) & ST_ERR, 0);
}

#[test]
fn read_native_max_address_reports_the_last_sector_not_the_count() {
    let disk = drive(4096);
    select_lba(&disk, 0);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_NATIVE_MAX));
    assert_eq!(status(&disk) & ST_ERR, 0);
    let lba = u32::from(disk.read_reg(Reg::LbaLow, false) as u8)
        | (u32::from(disk.read_reg(Reg::LbaMid, false) as u8) << 8)
        | (u32::from(disk.read_reg(Reg::LbaHigh, false) as u8) << 16)
        | (u32::from(disk.read_reg(Reg::Device, false) as u8 & DEV_HEAD) << 24);
    assert_eq!(lba, 4095, "the last addressable sector, which is count - 1");
}

#[test]
fn set_features_takes_a_pio_transfer_mode_and_refuses_a_dma_one() {
    let disk = drive(64);
    // 0x08 is PIO flow control mode 0, which this drive can do.
    disk.write_reg(Reg::Feature, 0x03);
    disk.write_reg(Reg::SectorCount, 0x08);
    disk.write_reg(Reg::Command, u16::from(cmd::SET_FEATURES));
    assert_eq!(status(&disk) & ST_ERR, 0);
    // 0x20 is multiword DMA mode 0, which it cannot: there is no DMA here and
    // saying otherwise would strand a driver waiting for a transfer.
    disk.write_reg(Reg::Feature, 0x03);
    disk.write_reg(Reg::SectorCount, 0x20);
    disk.write_reg(Reg::Command, u16::from(cmd::SET_FEATURES));
    assert_ne!(status(&disk) & ST_ERR, 0);
}

// ---------------------------------------------------------------------------
// geometry arithmetic
// ---------------------------------------------------------------------------

#[test]
fn a_default_geometry_is_one_identify_can_express() {
    for sectors in [
        1u64,
        62,
        63,
        1007,
        1008,
        4096,
        2 * 1024 * 1024,
        16383 * 16 * 63,
        1_000_000_000,
    ] {
        let g = default_geometry(sectors);
        assert!(g.is_valid(), "{sectors} produced {g:?}");
        assert!(u64::from(g.cylinders) <= MAX_IDENTIFY_CYLINDERS, "{g:?}");
        assert!(g.heads <= 16, "{g:?}");
        assert!(
            g.addressable() <= sectors,
            "{g:?} claims {} sectors on a drive of {sectors}",
            g.addressable()
        );
    }
}

#[test]
fn the_translation_round_trips_every_sector_it_can_name() {
    let geometry = Geometry {
        cylinders: 5,
        heads: 4,
        sectors: 17,
    };
    for lba in 0..geometry.addressable() {
        let chs = Address::from_lba(lba, &geometry).expect("inside the translation");
        assert_eq!(chs.to_lba(&geometry), Some(lba), "{chs:?}");
    }
}

// ---------------------------------------------------------------------------
// snapshots
// ---------------------------------------------------------------------------

/// Save `disk` into a one-device snapshot.
fn save_of(disk: &AtaDisk) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("hd", CLASS.name).expect("one device");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("hd", CLASS.name, CLASS.version).expect("a chunk");
        disk.save(&mut chunk).expect("the drive saves");
    }
    w.to_vec().expect("a snapshot")
}

#[test]
fn a_snapshot_taken_mid_sector_resumes_mid_sector() {
    // A transfer part way through its buffer *is* state. A model that saved the
    // command but not the buffer position would restore to a drive that reads
    // the sector again from its first word, which is a different machine.
    let saved = stamped(4096);
    select_lba(&saved, 0);
    saved.write_reg(Reg::SectorCount, 3);
    saved.write_reg(Reg::LbaLow, 50);
    saved.write_reg(Reg::LbaMid, 0);
    saved.write_reg(Reg::LbaHigh, 0);
    saved.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    let head = drain(&saved, 100);
    assert_eq!(head, stamp(50)[..200].to_vec());

    let first = save_of(&saved);

    let restored = drive(4096);
    let reader = StateReader::new(&first).expect("a snapshot we just wrote");
    let chunk = reader
        .load("hd", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk we just wrote");
    restored.load(&mut chunk.reader()).expect("it loads");

    assert_eq!(
        first,
        save_of(&restored),
        "the same state must save the same bytes"
    );

    // And it carries on where the other stopped, into the second and third
    // sectors, which is the part a byte comparison cannot see.
    assert_eq!(drain(&restored, 156), stamp(50)[200..].to_vec());
    assert_eq!(drain(&restored, 256), stamp(51));
    assert_eq!(drain(&restored, 256), stamp(52));
    assert_eq!(restored.read_alt_status(), ST_DRDY | ST_DSC);
}

#[test]
fn a_snapshot_carries_the_medium_and_the_configuration() {
    let saved = drive(4096);
    saved.write_reg(Reg::SectorCount, 32);
    saved.write_reg(Reg::Device, u16::from(DEV_OBSOLETE | 3));
    saved.write_reg(Reg::Command, u16::from(cmd::INIT_DEVICE_PARAMS));
    saved.write_reg(Reg::SectorCount, 8);
    saved.write_reg(Reg::Command, u16::from(cmd::SET_MULTIPLE));
    select_lba(&saved, 0);
    saved.write_reg(Reg::SectorCount, 1);
    saved.write_reg(Reg::LbaLow, 99);
    saved.write_reg(Reg::Command, u16::from(cmd::WRITE_SECTORS));
    fill(&saved, &stamp(99));

    let image = save_of(&saved);
    let restored = drive(4096);
    let reader = StateReader::new(&image).expect("a snapshot we just wrote");
    let chunk = reader
        .load("hd", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk");
    restored.load(&mut chunk.reader()).expect("it loads");

    assert_eq!(restored.current_geometry().heads, 4);
    assert_eq!(restored.multiple(), 8);
    let mut got = alloc::vec![0u8; SECTOR as usize];
    restored
        .read_media(99 * SECTOR, &mut got)
        .expect("in range");
    assert_eq!(got, stamp(99), "the medium came back too");
}

#[test]
fn a_corrupt_snapshot_is_refused_rather_than_believed() {
    let disk = drive(64);
    let image = save_of(&disk);
    let reader = StateReader::new(&image).expect("a snapshot we just wrote");
    let chunk = reader
        .load("hd", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk");
    // A drive of a different size cannot take it.
    let other = drive(128);
    assert!(other.load(&mut chunk.reader()).is_err());
}

// ---------------------------------------------------------------------------
// the machine-description object
// ---------------------------------------------------------------------------

#[test]
fn no_size_and_no_image_is_an_empty_bay() {
    let props = Props::new();
    assert!(
        AtaDisk::new(&props)
            .expect("an empty bay is not an error")
            .is_none(),
        "a machine file that names neither describes a cable position with \
         nothing plugged into it"
    );
}

#[test]
fn an_image_with_no_size_sets_the_capacity() {
    let mut props = Props::new();
    props.insert(
        "image",
        crate::core::props::Media::new("hd0", alloc::vec![0u8; 8 * 512]),
    );
    let disk = AtaDisk::new(&props)
        .expect("it builds")
        .expect("an image is a drive");
    assert_eq!(disk.identity().sectors, 8);
}

#[test]
fn a_size_that_is_not_a_whole_number_of_sectors_is_refused() {
    let mut props = Props::new();
    props.insert("size", crate::core::props::Value::Size(1000));
    assert!(AtaDisk::new(&props).is_err());
}

#[test]
fn a_partial_geometry_is_refused_rather_than_half_believed() {
    let mut props = Props::new();
    props.insert("size", crate::core::props::Value::Size(4096 * 512));
    props.insert("heads", crate::core::props::Value::Uint(8));
    assert!(
        AtaDisk::new(&props).is_err(),
        "`cylinders`, `heads` and `sectors` come as a set"
    );
}

#[test]
fn the_class_schema_and_the_property_list_describe_the_same_drive() {
    // Two descriptions of one class drift, so this asserts they have not.
    let schema = schema();
    for spec in CLASS.properties {
        assert!(
            schema.props.iter().any(|p| p.name == spec.name),
            "`{}` is in the class and not in the schema",
            spec.name
        );
    }
    assert_eq!(schema.props.len(), CLASS.properties.len());
}

#[test]
fn a_drive_answers_the_ata_signature_before_anything_has_reset_it() {
    // The very first thing a driver reads. A drive fresh out of the factory
    // has to carry the same signature a reset leaves, because a cold probe that
    // saw zeroes could not tell this drive from an empty cable position — and
    // this is the one place a model that only sets the signature in `reset`
    // gets it wrong.
    let disk = drive(4096);
    assert_eq!(disk.read_alt_status(), ST_DRDY | ST_DSC);
    assert_eq!(disk.read_reg(Reg::Feature, false) as u8, 0x01);
    assert_eq!(disk.read_reg(Reg::SectorCount, false) as u8, 0x01);
    assert_eq!(disk.read_reg(Reg::LbaLow, false) as u8, 0x01);
    assert_eq!(disk.read_reg(Reg::LbaMid, false) as u8, 0x00);
    assert_eq!(disk.read_reg(Reg::LbaHigh, false) as u8, 0x00);
}
