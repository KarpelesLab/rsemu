//! What a driver can actually do to an ATAPI CD-ROM.
//!
//! Everything here goes through the six methods a host adapter has, because
//! that is the whole of what is on the far side of the cable. The claims that
//! matter most are the ones a plausible-looking model gets wrong:
//!
//! * the reset signature and a Status register of **zero**, which together are
//!   how a driver tells this device from a hard disk before issuing anything;
//! * that `PACKET` raises `DRQ` **without** an interrupt and the data phase
//!   raises it **with** one, in that order and no other;
//! * that a packet data-in command interrupts once more than an ATA PIO data-in
//!   command of the same length does, because its completion is announced and
//!   an ATA read's is not;
//! * that the byte count limit actually limits, so a host with a 512-byte
//!   buffer can read a 2048-byte block in four goes.

use super::*;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

// ---------------------------------------------------------------------------
// rigs
// ---------------------------------------------------------------------------

/// What block `lba` holds on a stamped disc: bytes that say which block they
/// are, so a transfer that lands one block out fails rather than passing on
/// identical zeroes.
fn stamp(lba: u64) -> Vec<u8> {
    let mut out = alloc::vec![0u8; BLOCK as usize];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = (lba as u8) ^ (i as u8) ^ 0x3c;
    }
    out[0] = lba as u8;
    out[1] = (lba >> 8) as u8;
    out
}

/// A drive with a stamped disc of `blocks` logical blocks in it.
fn drive(blocks: u64) -> AtapiDrive {
    let store = RamStore::new(blocks * BLOCK);
    for lba in 0..blocks {
        store.write_at(lba * BLOCK, &stamp(lba)).expect("in range");
    }
    AtapiDrive::with_disc(Identity::new(), Position::Device0, Some(Arc::new(store)))
        .expect("a whole number of logical blocks")
}

/// A drive with nothing in it.
fn empty() -> AtapiDrive {
    AtapiDrive::with_disc(Identity::new(), Position::Device0, None).expect("an empty drive")
}

/// Select device 0, which is where every rig here is jumpered.
fn select(cd: &AtapiDrive) {
    cd.write_reg(Reg::Device, u16::from(DEV_OBSOLETE));
}

/// Clear the power-on unit attention the way a driver does: issue something,
/// take the `CHECK CONDITION`, and read the sense out.
fn clear_attention(cd: &AtapiDrive) {
    select(cd);
    let status = packet_command(cd, &cdb_test_unit_ready(), 0);
    assert_eq!(status.status & ST_CHK, ST_CHK, "the reset is reported once");
    let sense = packet_command_in(cd, &cdb_request_sense(18), 18);
    assert_eq!(sense.1[2] & 0x0f, sense_key::UNIT_ATTENTION);
    assert_eq!((sense.1[12], sense.1[13]), asc::RESET_OCCURRED);
}

/// How a command ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Ended {
    status: u8,
    reason: u8,
    interrupts: u32,
}

/// Deliver `cdb` and drain nothing: for commands with no data phase.
fn packet_command(cd: &AtapiDrive, cdb: &[u8], byte_count: u16) -> Ended {
    let (ended, data) = packet_command_in(cd, cdb, byte_count);
    assert!(data.is_empty(), "this command was not meant to return data");
    ended
}

/// Deliver `cdb` with a byte count limit of `byte_count`, drain whatever comes
/// back, and report how the command ended.
///
/// This is the §9.10 handshake written out once. Every assertion about the
/// protocol's *shape* lives here, so that a test about a command can be about
/// the command.
fn packet_command_in(cd: &AtapiDrive, cdb: &[u8], byte_count: u16) -> (Ended, Vec<u8>) {
    select(cd);
    let mut interrupts = 0u32;

    cd.write_reg(Reg::Feature, 0);
    cd.write_reg(Reg::LbaMid, u16::from(byte_count as u8));
    cd.write_reg(Reg::LbaHigh, u16::from((byte_count >> 8) as u8));
    cd.write_reg(Reg::Command, u16::from(cmd::PACKET));

    // Step 2: DRQ up, C/D set, I/O clear — and *no* interrupt, because word 0
    // reports microprocessor DRQ.
    assert_eq!(cd.read_alt_status() & ST_DRQ, ST_DRQ, "the packet phase");
    assert_eq!(
        cd.read_reg(Reg::SectorCount, true) as u8 & (IR_CD | IR_IO),
        IR_CD,
        "C/D set and I/O clear: give me a command packet"
    );
    assert!(!cd.irq_asserted(), "the packet phase does not interrupt");

    // Step 3: six words.
    let mut padded = cdb.to_vec();
    padded.resize(PACKET_BYTES, 0);
    for pair in padded.chunks(2) {
        cd.write_reg(Reg::Data, u16::from(pair[0]) | (u16::from(pair[1]) << 8));
    }

    // Step 4: data blocks, then completion.
    let mut out = Vec::new();
    loop {
        if cd.irq_asserted() {
            interrupts += 1;
        }
        let status = cd.read_reg(Reg::Command, false) as u8;
        if status & ST_DRQ == 0 {
            return (
                Ended {
                    status,
                    reason: cd.read_reg(Reg::SectorCount, true) as u8,
                    interrupts,
                },
                out,
            );
        }
        let reason = cd.read_reg(Reg::SectorCount, true) as u8;
        assert_eq!(reason & (IR_CD | IR_IO), IR_IO, "a data block is I/O only");
        let count = (cd.read_reg(Reg::LbaMid, true) as u8 as u16)
            | ((cd.read_reg(Reg::LbaHigh, true) as u8 as u16) << 8);
        assert!(count > 0, "a block the device announced has bytes in it");
        // The data register is sixteen bits wide, so an odd byte count — which
        // only the last block of a short response can have — costs the host a
        // whole word and a discarded high half. That is what the cable does.
        let before = out.len();
        for _ in 0..count.div_ceil(2) {
            let word = cd.read_reg(Reg::Data, false);
            out.push(word as u8);
            out.push((word >> 8) as u8);
        }
        out.truncate(before + usize::from(count));
    }
}

fn cdb_test_unit_ready() -> Vec<u8> {
    alloc::vec![packet::TEST_UNIT_READY, 0, 0, 0, 0, 0]
}

fn cdb_request_sense(len: u8) -> Vec<u8> {
    alloc::vec![packet::REQUEST_SENSE, 0, 0, 0, len, 0]
}

fn cdb_read10(lba: u32, blocks: u16) -> Vec<u8> {
    let l = lba.to_be_bytes();
    let n = blocks.to_be_bytes();
    alloc::vec![packet::READ_10, 0, l[0], l[1], l[2], l[3], 0, n[0], n[1], 0]
}

// ---------------------------------------------------------------------------
// what a driver sees before it says anything
// ---------------------------------------------------------------------------

#[test]
fn the_reset_signature_is_the_packet_one() {
    // ATA/ATAPI-6 §9.1. 0xEB14 in the two Byte Count registers is how a driver
    // knows this is not a hard disk, and the Status register of zero is the
    // other half of it: §7.15.6.3 says DRDY is not a bit a packet device has.
    let cd = empty();
    select(&cd);
    assert_eq!(cd.read_reg(Reg::SectorCount, true), 0x01);
    assert_eq!(cd.read_reg(Reg::LbaLow, true), 0x01);
    assert_eq!(cd.read_reg(Reg::LbaMid, true), u16::from(SIGNATURE_MID));
    assert_eq!(cd.read_reg(Reg::LbaHigh, true), u16::from(SIGNATURE_HIGH));
    assert_eq!(cd.read_alt_status(), 0x00);
}

#[test]
fn identify_device_aborts_and_leaves_the_signature_behind() {
    // §8.15: the symmetric half of `ata.disk` aborting IDENTIFY PACKET DEVICE.
    // A host that probed the wrong way round still learns what this is.
    let cd = drive(4);
    select(&cd);
    cd.write_reg(Reg::Command, u16::from(cmd::IDENTIFY));
    let status = cd.read_reg(Reg::Command, false) as u8;
    assert_eq!(status & ST_CHK, ST_CHK);
    assert_eq!(cd.read_reg(Reg::Feature, true) as u8 & ERR_ABRT, ERR_ABRT);
    assert_eq!(cd.read_reg(Reg::LbaMid, true), u16::from(SIGNATURE_MID));
    assert_eq!(cd.read_reg(Reg::LbaHigh, true), u16::from(SIGNATURE_HIGH));
}

#[test]
fn identify_packet_device_describes_a_twelve_byte_cd_rom() {
    let cd = drive(4);
    select(&cd);
    cd.write_reg(Reg::Command, u16::from(cmd::IDENTIFY_PACKET));
    assert!(cd.irq_asserted(), "a PIO data-in block is announced");
    assert_eq!(cd.read_reg(Reg::Command, false) as u8 & ST_DRQ, ST_DRQ);
    assert!(!cd.irq_asserted(), "reading Status acknowledged it");
    let mut words = Vec::new();
    for _ in 0..256 {
        words.push(cd.read_reg(Reg::Data, false));
    }
    assert_eq!(cd.read_alt_status() & ST_DRQ, 0, "one block and no more");
    assert!(
        !cd.irq_asserted(),
        "§9.5 DPIOI1:DI1 — a PIO data-in command has no completion interrupt"
    );

    // Word 0, field by field: ATAPI, the CD-ROM command packet set, removable,
    // microprocessor DRQ, a twelve-byte packet.
    assert_eq!(words[0] >> 14, 0b10, "an ATAPI device");
    assert_eq!((words[0] >> 8) & 0x1f, 5, "the CD-ROM command packet set");
    assert_eq!(words[0] & 0x80, 0x80, "removable media");
    assert_eq!((words[0] >> 5) & 3, 0, "microprocessor DRQ");
    assert_eq!(words[0] & 3, 0, "a twelve-byte command packet");
    // Word 82 bit 4 is the PACKET feature set, which is the other thing a
    // driver looks at.
    assert_eq!(words[82] & (1 << 4), 1 << 4);

    // And the response checksums, which is the cheapest way to catch a model
    // that scribbled off the end of its word array.
    let bytes: Vec<u8> = words.iter().flat_map(|w| w.to_le_bytes()).collect();
    assert_eq!(bytes[510], 0xa5);
    assert_eq!(bytes.iter().fold(0u8, |a, b| a.wrapping_add(*b)), 0);
}

// ---------------------------------------------------------------------------
// the packet handshake
// ---------------------------------------------------------------------------

#[test]
fn the_power_on_unit_attention_is_reported_once() {
    // SFF-8020i §9.3. A driver's first command fails, it reads the sense, and
    // the second one works — and a model that reported the attention forever
    // would hang every driver that retries.
    let cd = drive(4);
    clear_attention(&cd);
    let ended = packet_command(&cd, &cdb_test_unit_ready(), 0);
    assert_eq!(ended.status & ST_CHK, 0, "the second one is ready");
    assert_eq!(
        ended.reason & (IR_CD | IR_IO),
        IR_CD | IR_IO,
        "completion is C/D and I/O both set"
    );
    assert_eq!(ended.interrupts, 1, "one interrupt: the completion");
}

#[test]
fn a_data_in_command_interrupts_once_per_block_and_once_at_the_end() {
    // The one place the packet protocol differs from §9.5's PIO data-in: an
    // ATA read has no completion interrupt and an ATAPI one does. Counted,
    // because that is the only way to state it.
    let cd = drive(8);
    clear_attention(&cd);
    let (ended, data) = packet_command_in(&cd, &cdb_read10(2, 2), 2048);
    assert_eq!(ended.status & ST_CHK, 0);
    assert_eq!(data.len(), 4096);
    assert_eq!(data[..2048], stamp(2)[..]);
    assert_eq!(data[2048..], stamp(3)[..]);
    assert_eq!(ended.interrupts, 3, "two blocks plus the completion");
}

#[test]
fn the_byte_count_limit_limits() {
    // A host with a 512-byte buffer reads a 2048-byte block in four goes, and
    // the Byte Count registers report the *actual* size of each one.
    let cd = drive(8);
    clear_attention(&cd);
    let (ended, data) = packet_command_in(&cd, &cdb_read10(5, 1), 512);
    assert_eq!(ended.status & ST_CHK, 0);
    assert_eq!(data, stamp(5));
    assert_eq!(ended.interrupts, 5, "four blocks plus the completion");
}

#[test]
fn an_odd_byte_count_limit_is_rounded_down() {
    // §8.21.5: the data register is sixteen bits wide, so a device cannot
    // honour an odd limit and rounds down rather than transferring a half word.
    let cd = drive(8);
    clear_attention(&cd);
    let (ended, data) = packet_command_in(&cd, &cdb_read10(1, 1), 511);
    assert_eq!(ended.status & ST_CHK, 0);
    assert_eq!(data, stamp(1));
}

#[test]
fn a_debug_read_of_the_data_register_does_not_advance_the_buffer() {
    // `ROADMAP.md` §15 invariant 5, at the one register here that has a side
    // effect to suppress.
    let cd = drive(4);
    clear_attention(&cd);
    select(&cd);
    cd.write_reg(Reg::LbaMid, 0);
    cd.write_reg(Reg::LbaHigh, 8);
    cd.write_reg(Reg::Command, u16::from(cmd::PACKET));
    let mut cdb = cdb_read10(0, 1);
    cdb.resize(PACKET_BYTES, 0);
    for pair in cdb.chunks(2) {
        cd.write_reg(Reg::Data, u16::from(pair[0]) | (u16::from(pair[1]) << 8));
    }
    let peeked = cd.read_reg(Reg::Data, true);
    assert_eq!(cd.read_reg(Reg::Data, true), peeked, "still the first word");
    assert_eq!(
        cd.read_reg(Reg::Data, false),
        peeked,
        "and the host gets it"
    );
    assert_ne!(cd.read_reg(Reg::Data, true), peeked, "which then advanced");
}

#[test]
fn the_data_register_hands_back_nothing_during_the_command_packet_phase() {
    // `DRQ` is up while the drive is waiting for a command descriptor block,
    // and that block is the host's to fill: the Interrupt Reason register says
    // `I/O = 0`. A drive that handed it back would return a host its own
    // half-written packet.
    let cd = drive(4);
    clear_attention(&cd);
    select(&cd);
    cd.write_reg(Reg::LbaMid, 0);
    cd.write_reg(Reg::LbaHigh, 8);
    cd.write_reg(Reg::Command, u16::from(cmd::PACKET));
    assert_eq!(cd.read_alt_status() & ST_DRQ, ST_DRQ);
    assert_eq!(cd.read_reg(Reg::SectorCount, true) as u8 & IR_IO, 0);
    assert_eq!(cd.read_reg(Reg::Data, false), 0);
    // And the phase is undisturbed: the packet still goes in and still runs.
    let mut cdb = cdb_read10(1, 1);
    cdb.resize(PACKET_BYTES, 0);
    for pair in cdb.chunks(2) {
        cd.write_reg(Reg::Data, u16::from(pair[0]) | (u16::from(pair[1]) << 8));
    }
    assert_eq!(cd.read_alt_status() & ST_CHK, 0);
    assert_eq!(cd.read_reg(Reg::SectorCount, true) as u8 & IR_IO, IR_IO);
}

#[test]
fn a_debug_read_of_the_status_register_does_not_acknowledge() {
    let cd = drive(4);
    clear_attention(&cd);
    packet_command(&cd, &cdb_test_unit_ready(), 0);
    // The completion's interrupt is still up until a *real* status read.
    let cd2 = drive(4);
    clear_attention(&cd2);
    select(&cd2);
    cd2.write_reg(Reg::LbaMid, 0);
    cd2.write_reg(Reg::LbaHigh, 8);
    cd2.write_reg(Reg::Command, u16::from(cmd::PACKET));
    let mut cdb = cdb_test_unit_ready();
    cdb.resize(PACKET_BYTES, 0);
    for pair in cdb.chunks(2) {
        cd2.write_reg(Reg::Data, u16::from(pair[0]) | (u16::from(pair[1]) << 8));
    }
    assert!(cd2.irq_asserted());
    cd2.read_reg(Reg::Command, true);
    assert!(cd2.irq_asserted(), "a debug read acknowledges nothing");
    cd2.read_reg(Reg::Command, false);
    assert!(!cd2.irq_asserted(), "and a real one does");
}

#[test]
fn nien_gates_the_interrupt_line() {
    let cd = drive(4);
    clear_attention(&cd);
    cd.write_device_control(CTL_NIEN);
    select(&cd);
    cd.write_reg(Reg::LbaMid, 0);
    cd.write_reg(Reg::LbaHigh, 8);
    cd.write_reg(Reg::Command, u16::from(cmd::PACKET));
    let mut cdb = cdb_test_unit_ready();
    cdb.resize(PACKET_BYTES, 0);
    for pair in cdb.chunks(2) {
        cd.write_reg(Reg::Data, u16::from(pair[0]) | (u16::from(pair[1]) << 8));
    }
    assert!(!cd.irq_asserted(), "nIEN holds INTRQ off");
}

#[test]
fn a_software_reset_restores_the_signature() {
    let cd = drive(4);
    clear_attention(&cd);
    cd.write_device_control(CTL_SRST);
    assert_eq!(cd.read_alt_status(), ST_BSY, "held busy while SRST is up");
    cd.write_device_control(0);
    select(&cd);
    assert_eq!(cd.read_reg(Reg::LbaMid, true), u16::from(SIGNATURE_MID));
    assert_eq!(cd.read_reg(Reg::LbaHigh, true), u16::from(SIGNATURE_HIGH));
    assert!(!cd.irq_asserted(), "a software reset does not interrupt");
    // And it is a reason for a fresh unit attention, which is what tells the
    // host that whatever it had in flight is gone.
    let ended = packet_command(&cd, &cdb_test_unit_ready(), 0);
    assert_eq!(ended.status & ST_CHK, ST_CHK);
}

// ---------------------------------------------------------------------------
// the packet command set
// ---------------------------------------------------------------------------

#[test]
fn inquiry_says_removable_cd_rom() {
    let cd = drive(4);
    // INQUIRY is one of the two commands a unit attention does not block —
    // SFF-8020i §9.3 — which is why this test does not clear it first.
    let (ended, data) = packet_command_in(&cd, &alloc::vec![packet::INQUIRY, 0, 0, 0, 36, 0], 512);
    assert_eq!(
        ended.status & ST_CHK,
        0,
        "a unit attention does not block it"
    );
    assert_eq!(data.len(), 36);
    assert_eq!(data[0] & 0x1f, 0x05, "peripheral device type: CD-ROM");
    assert_eq!(data[1] & 0x80, 0x80, "removable");
    assert_eq!(data[4], 31, "additional length");
    assert_eq!(&data[8..13], b"RSEMU");
}

#[test]
fn an_allocation_length_shorter_than_the_response_truncates_it() {
    let cd = drive(4);
    let (_, data) = packet_command_in(&cd, &alloc::vec![packet::INQUIRY, 0, 0, 0, 5, 0], 512);
    assert_eq!(data.len(), 5);
}

#[test]
fn read_capacity_reports_the_last_block_and_2048() {
    let cd = drive(37);
    clear_attention(&cd);
    let (ended, data) = packet_command_in(
        &cd,
        &alloc::vec![packet::READ_CAPACITY, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        512,
    );
    assert_eq!(ended.status & ST_CHK, 0);
    assert_eq!(data.len(), 8);
    assert_eq!(be32(&data[0..4]), 36, "the last block, not the count");
    assert_eq!(be32(&data[4..8]), 2048);
}

#[test]
fn a_read_past_the_end_is_an_illegal_request() {
    let cd = drive(4);
    clear_attention(&cd);
    let ended = packet_command(&cd, &cdb_read10(4, 1), 2048);
    assert_eq!(ended.status & ST_CHK, ST_CHK);
    assert_eq!(
        cd.read_reg(Reg::Feature, true) as u8 >> 4,
        sense_key::ILLEGAL_REQUEST,
        "the sense key is in the Error register's top four bits"
    );
    let (_, sense) = packet_command_in(&cd, &cdb_request_sense(18), 512);
    assert_eq!(sense[2] & 0x0f, sense_key::ILLEGAL_REQUEST);
    assert_eq!((sense[12], sense[13]), asc::LBA_OUT_OF_RANGE);
}

#[test]
fn a_read_that_straddles_the_end_is_refused_whole() {
    let cd = drive(4);
    clear_attention(&cd);
    let ended = packet_command(&cd, &cdb_read10(3, 2), 2048);
    assert_eq!(ended.status & ST_CHK, ST_CHK, "not a short transfer");
}

#[test]
fn an_unknown_opcode_is_an_invalid_command_operation_code() {
    let cd = drive(4);
    clear_attention(&cd);
    let ended = packet_command(&cd, &alloc::vec![0xd0u8, 0, 0, 0, 0, 0], 512);
    assert_eq!(ended.status & ST_CHK, ST_CHK);
    let (_, sense) = packet_command_in(&cd, &cdb_request_sense(18), 512);
    assert_eq!(sense[2] & 0x0f, sense_key::ILLEGAL_REQUEST);
    assert_eq!((sense[12], sense[13]), asc::INVALID_COMMAND);
}

#[test]
fn an_empty_drive_says_medium_not_present() {
    let cd = empty();
    clear_attention(&cd);
    for cdb in [cdb_test_unit_ready(), cdb_read10(0, 1)] {
        let ended = packet_command(&cd, &cdb, 2048);
        assert_eq!(ended.status & ST_CHK, ST_CHK);
        let (_, sense) = packet_command_in(&cd, &cdb_request_sense(18), 512);
        assert_eq!(sense[2] & 0x0f, sense_key::NOT_READY);
        assert_eq!((sense[12], sense[13]), asc::MEDIUM_NOT_PRESENT);
    }
    // And INQUIRY still answers, because a drive with no disc is still a drive.
    let (ended, data) = packet_command_in(&cd, &alloc::vec![packet::INQUIRY, 0, 0, 0, 36, 0], 512);
    assert_eq!(ended.status & ST_CHK, 0);
    assert_eq!(data[0] & 0x1f, 0x05);
}

#[test]
fn read_toc_lists_one_data_track_and_the_lead_out() {
    let cd = drive(100);
    clear_attention(&cd);
    let (ended, data) = packet_command_in(
        &cd,
        &alloc::vec![packet::READ_TOC, 0, 0, 0, 0, 0, 0, 0, 200, 0],
        512,
    );
    assert_eq!(ended.status & ST_CHK, 0);
    // Header: length, first track, last track. Then two eight-byte descriptors.
    assert_eq!(be16(&data[0..2]) as usize, data.len() - 2);
    assert_eq!(data[2], 1, "first track");
    assert_eq!(data[3], 1, "last track");
    assert_eq!(data.len(), 4 + 16);
    assert_eq!(data[5], 0x14, "ADR 1, a data track");
    assert_eq!(data[6], 1, "track one");
    assert_eq!(be32(&data[8..12]), 0, "starts at block zero");
    assert_eq!(data[14], 0xaa, "the lead-out");
    assert_eq!(be32(&data[16..20]), 100, "at the end of the disc");
}

#[test]
fn read_toc_in_msf_adds_the_two_second_lead_in() {
    let cd = drive(100);
    clear_attention(&cd);
    let (_, data) = packet_command_in(
        &cd,
        &alloc::vec![packet::READ_TOC, 0x02, 0, 0, 0, 0, 0, 0, 200, 0],
        512,
    );
    // Block 0 is 00:02:00, which is the Red Book's 150-frame lead-in.
    assert_eq!((data[9], data[10], data[11]), (0, 2, 0));
    // Block 100 is 150 + 100 = 250 frames = 00:03:25.
    assert_eq!((data[17], data[18], data[19]), (0, 3, 25));
}

#[test]
fn mode_sense_reports_the_pages_it_has_and_refuses_the_rest() {
    let cd = drive(4);
    clear_attention(&cd);
    let (ended, data) = packet_command_in(
        &cd,
        &alloc::vec![packet::MODE_SENSE_10, 0, 0x2a, 0, 0, 0, 0, 0, 200, 0],
        512,
    );
    assert_eq!(ended.status & ST_CHK, 0);
    assert_eq!(be16(&data[0..2]) as usize, data.len() - 2);
    assert_eq!(data[3] & 0x80, 0x80, "write protected, by construction");
    assert_eq!(data[8], 0x2a, "the capabilities page");
    assert_eq!(data[9], 0x14);

    // Page 0Dh's two constants are what make an MSF address mean anything.
    let (_, cdp) = packet_command_in(
        &cd,
        &alloc::vec![packet::MODE_SENSE_10, 0, 0x0d, 0, 0, 0, 0, 0, 200, 0],
        512,
    );
    assert_eq!(cdp[13], 60, "seconds per minute");
    assert_eq!(cdp[15], 75, "frames per second");

    let ended = packet_command(
        &cd,
        &alloc::vec![packet::MODE_SENSE_10, 0, 0x1c, 0, 0, 0, 0, 0, 200, 0],
        512,
    );
    assert_eq!(ended.status & ST_CHK, ST_CHK, "a page this drive has not");
}

#[test]
fn mode_sense_6_has_the_same_pages_behind_a_shorter_header() {
    let cd = drive(4);
    clear_attention(&cd);
    let (ended, data) = packet_command_in(
        &cd,
        &alloc::vec![packet::MODE_SENSE_6, 0, 0x2a, 0, 200, 0],
        512,
    );
    assert_eq!(ended.status & ST_CHK, 0);
    assert_eq!(usize::from(data[0]), data.len() - 1);
    assert_eq!(data[2] & 0x80, 0x80);
    assert_eq!(data[4], 0x2a);
}

#[test]
fn a_locked_tray_refuses_to_eject() {
    let cd = drive(4);
    clear_attention(&cd);
    // PREVENT MEDIUM REMOVAL, then START/STOP UNIT with LoEj set and Start
    // clear, which is "open the tray".
    packet_command(&cd, &alloc::vec![packet::PREVENT_ALLOW, 0, 0, 0, 1, 0], 0);
    let ended = packet_command(
        &cd,
        &alloc::vec![packet::START_STOP_UNIT, 0, 0, 0, 0x02, 0],
        0,
    );
    assert_eq!(ended.status & ST_CHK, ST_CHK);
    let (_, sense) = packet_command_in(&cd, &cdb_request_sense(18), 512);
    assert_eq!((sense[12], sense[13]), asc::REMOVAL_PREVENTED);

    // ALLOW, and it is accepted again.
    packet_command(&cd, &alloc::vec![packet::PREVENT_ALLOW, 0, 0, 0, 0, 0], 0);
    let ended = packet_command(
        &cd,
        &alloc::vec![packet::START_STOP_UNIT, 0, 0, 0, 0x02, 0],
        0,
    );
    assert_eq!(ended.status & ST_CHK, 0);
}

#[test]
fn seek_validates_the_address_and_moves_nothing() {
    let cd = drive(4);
    clear_attention(&cd);
    let ended = packet_command(
        &cd,
        &alloc::vec![packet::SEEK_10, 0, 0, 0, 0, 3, 0, 0, 0, 0],
        0,
    );
    assert_eq!(ended.status & ST_CHK, 0);
    let ended = packet_command(
        &cd,
        &alloc::vec![packet::SEEK_10, 0, 0, 0, 0, 9, 0, 0, 0, 0],
        0,
    );
    assert_eq!(ended.status & ST_CHK, ST_CHK);
}

#[test]
fn read_12_reads_what_read_10_reads() {
    let cd = drive(8);
    clear_attention(&cd);
    let (_, ten) = packet_command_in(&cd, &cdb_read10(6, 1), 2048);
    let (_, twelve) = packet_command_in(
        &cd,
        &alloc::vec![packet::READ_12, 0, 0, 0, 0, 6, 0, 0, 0, 1, 0, 0],
        2048,
    );
    assert_eq!(ten, twelve);
    assert_eq!(ten, stamp(6));
}

#[test]
fn a_transfer_length_of_zero_is_a_successful_command_that_moves_nothing() {
    let cd = drive(8);
    clear_attention(&cd);
    let ended = packet_command(&cd, &cdb_read10(0, 0), 2048);
    assert_eq!(ended.status & ST_CHK, 0);
}

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------

#[test]
fn a_raw_2352_byte_image_is_refused_by_name() {
    // The failure this message exists to prevent: reading a raw rip as though
    // it were cooked hands a guest sixteen bytes of sync pattern where its boot
    // record should be, with no error anywhere.
    let store = RamStore::new(RAW_BLOCK * 10);
    let err = AtapiDrive::with_disc(Identity::new(), Position::Device0, Some(Arc::new(store)))
        .expect_err("a raw image is not a cooked one");
    let text = alloc::format!("{err}");
    assert!(text.contains("raw"), "{text}");
    assert!(text.contains("2352"), "{text}");
}

#[test]
fn a_disc_that_is_not_a_whole_number_of_blocks_is_refused() {
    let store = RamStore::new(BLOCK * 3 + 512);
    let err = AtapiDrive::with_disc(Identity::new(), Position::Device0, Some(Arc::new(store)))
        .expect_err("not a whole number of logical blocks");
    assert!(alloc::format!("{err}").contains("2048"));
}

#[test]
fn the_slave_position_answers_only_when_it_is_selected() {
    let cd = AtapiDrive::with_disc(
        Identity::new(),
        Position::Device1,
        Some(Arc::new(RamStore::new(BLOCK))),
    )
    .expect("a one-block disc");
    cd.write_reg(Reg::Device, u16::from(DEV_OBSOLETE));
    assert!(!cd.is_selected());
    cd.write_reg(Reg::Device, u16::from(DEV_OBSOLETE | DEV_SELECT));
    assert!(cd.is_selected());
}

// ---------------------------------------------------------------------------
// snapshots
// ---------------------------------------------------------------------------

fn save_of(cd: &AtapiDrive) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("cd", CLASS.name).expect("one device");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("cd", CLASS.name, CLASS.version).expect("a chunk");
        cd.save(&mut chunk).expect("the drive saves");
    }
    w.to_vec().expect("a snapshot")
}

#[test]
fn a_snapshot_taken_mid_block_resumes_mid_block() {
    let saved = drive(8);
    clear_attention(&saved);
    select(&saved);
    saved.write_reg(Reg::LbaMid, 0);
    saved.write_reg(Reg::LbaHigh, 8);
    saved.write_reg(Reg::Command, u16::from(cmd::PACKET));
    let mut cdb = cdb_read10(4, 2);
    cdb.resize(PACKET_BYTES, 0);
    for pair in cdb.chunks(2) {
        saved.write_reg(Reg::Data, u16::from(pair[0]) | (u16::from(pair[1]) << 8));
    }
    let mut head = Vec::new();
    for _ in 0..100 {
        let word = saved.read_reg(Reg::Data, false);
        head.push(word as u8);
        head.push((word >> 8) as u8);
    }
    assert_eq!(head, stamp(4)[..200]);

    let image = save_of(&saved);
    let restored = drive(8);
    let reader = StateReader::new(&image).expect("a snapshot we just wrote");
    let chunk = reader
        .load("cd", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk we just wrote");
    restored.load(&mut chunk.reader()).expect("it loads");

    assert_eq!(
        image,
        save_of(&restored),
        "the same state must save the same bytes"
    );

    // And it carries on where the other stopped, through the rest of block 4
    // and the whole of block 5.
    // Word at a time rather than block at a time, because the Byte Count
    // register reports the size of the *whole* block and this host is resuming
    // 200 bytes into one — which is exactly the state being tested.
    let mut rest = Vec::new();
    while restored.read_reg(Reg::Command, false) as u8 & ST_DRQ != 0 {
        let word = restored.read_reg(Reg::Data, false);
        rest.push(word as u8);
        rest.push((word >> 8) as u8);
    }
    let mut want = stamp(4)[200..].to_vec();
    want.extend_from_slice(&stamp(5));
    assert_eq!(rest, want);
}

#[test]
fn a_snapshot_carries_the_sense_and_the_tray_lock() {
    let saved = drive(4);
    clear_attention(&saved);
    packet_command(
        &saved,
        &alloc::vec![packet::PREVENT_ALLOW, 0, 0, 0, 1, 0],
        0,
    );
    packet_command(&saved, &cdb_read10(9, 1), 2048);

    let image = save_of(&saved);
    let restored = drive(4);
    let reader = StateReader::new(&image).expect("a snapshot we just wrote");
    let chunk = reader
        .load("cd", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk");
    restored.load(&mut chunk.reader()).expect("it loads");
    assert_eq!(image, save_of(&restored));

    // The sense survived: the failure the restored machine has not yet been
    // told about is still waiting for it.
    let (_, sense) = packet_command_in(&restored, &cdb_request_sense(18), 512);
    assert_eq!((sense[12], sense[13]), asc::LBA_OUT_OF_RANGE);
    // And so did the lock.
    let ended = packet_command(
        &restored,
        &alloc::vec![packet::START_STOP_UNIT, 0, 0, 0, 0x02, 0],
        0,
    );
    assert_eq!(ended.status & ST_CHK, ST_CHK);
}

// ---------------------------------------------------------------------------
// the taskfile door
// ---------------------------------------------------------------------------
//
// The second way in, which a Serial ATA adapter uses because it has no
// registers to write in order. The claim being tested is that it is the *same*
// drive underneath: every assertion below compares what the struct door
// produces against what the cable produces, so a second command set would show
// up as a difference rather than as a second set of green tests.

/// Run a packet command through the taskfile seam, the way `dev/ahci` does, and
/// hand back what came out and how many interrupts were taken.
fn taskfile_packet(cd: &AtapiDrive, command: &[u8], limit: u16, dma: bool) -> (Vec<u8>, u32, u8) {
    let tf = Taskfile {
        command: cmd::PACKET,
        feature: u16::from(dma),
        // The byte count limit is LBA Mid and LBA High, which is bits 23:8 of
        // the address a Register FIS carries (ATA/ATAPI-6 §8.21.4).
        lba: u64::from(limit) << 8,
        count: 0,
        device: 0,
    };
    let mut phase = TaskfileDevice::taskfile_start(cd, &tf);
    // §9.10 step 2: `DRQ` up over a command packet, and no interrupt for it.
    assert!(
        matches!(phase, Phase::Packet { bytes } if bytes == PACKET_BYTES as u64),
        "{phase:?}"
    );
    assert!(
        !TaskfileDevice::taskfile_acknowledge(cd),
        "PACKET does not interrupt"
    );

    // Sixteen bytes, as an AHCI command table's `ACMD` field is, of which this
    // drive takes twelve.
    let mut acmd = [0u8; 16];
    acmd[..command.len()].copy_from_slice(command);
    let mut at = 0usize;
    while let Phase::Packet { bytes } = phase {
        let want = core::cmp::min(bytes as usize, acmd.len() - at);
        let did = TaskfileDevice::taskfile_write(cd, &acmd[at..at + want]) as usize;
        assert!(did > 0);
        at += did;
        phase = TaskfileDevice::taskfile_phase(cd);
    }
    assert_eq!(at, PACKET_BYTES, "the drive took a twelve-byte packet");

    let mut out = Vec::new();
    let mut interrupts = 0;
    while let Phase::Data {
        out: o,
        dma: d,
        block,
    } = phase
    {
        assert!(!o, "this command set has no data-out phase");
        assert_eq!(
            d, dma,
            "the drive reports the protocol the PACKET asked for"
        );
        assert!(block <= u64::from(limit.max(1)) || limit == 0);
        if TaskfileDevice::taskfile_acknowledge(cd) {
            interrupts += 1;
        }
        let mut chunk = alloc::vec![0u8; block as usize];
        let got = TaskfileDevice::taskfile_read(cd, &mut chunk) as usize;
        out.extend_from_slice(&chunk[..got]);
        phase = TaskfileDevice::taskfile_phase(cd);
    }
    assert_eq!(phase, Phase::Done);
    if TaskfileDevice::taskfile_acknowledge(cd) {
        interrupts += 1;
    }
    (
        out,
        interrupts,
        TaskfileDevice::taskfile_registers(cd).status,
    )
}

/// The signature a Serial ATA port latches into `PxSIG` is the same four
/// registers a cable-driven host reads one at a time. `EB140101h` is how a
/// driver on either transport learns this is a packet device.
#[test]
fn the_taskfile_registers_carry_the_packet_signature() {
    let cd = empty();
    let regs = TaskfileDevice::taskfile_registers(&cd);
    assert_eq!(regs.status, 0, "DRDY is not a bit a packet device has");
    assert_eq!(regs.count & 0xff, 0x01);
    assert_eq!(regs.lba & 0xff, 0x01);
    assert_eq!((regs.lba >> 8) & 0xff, u64::from(SIGNATURE_MID));
    assert_eq!((regs.lba >> 16) & 0xff, u64::from(SIGNATURE_HIGH));
    // Which is exactly what the cable reports, register for register.
    select(&cd);
    assert_eq!(cd.read_reg(Reg::LbaMid, true), u16::from(SIGNATURE_MID));
    assert_eq!(cd.read_reg(Reg::LbaHigh, true), u16::from(SIGNATURE_HIGH));
}

/// One command, two doors, one answer. If the struct door were a second decode
/// this is where it would show.
#[test]
fn the_two_doors_give_the_same_answer_to_the_same_command() {
    let cable = drive(8);
    clear_attention(&cable);
    let (_, by_cable) = packet_command_in(&cable, &cdb_read10(3, 2), 512);

    let seam = drive(8);
    // The unit attention has to be taken on this drive too — through the seam,
    // which is the driver's first exchange on a Serial ATA port.
    let (_, _, status) = taskfile_packet(&seam, &cdb_test_unit_ready(), 512, false);
    assert_eq!(status & ST_CHK, ST_CHK, "the reset is reported once");
    let (_, _, _) = taskfile_packet(&seam, &cdb_request_sense(18), 512, false);
    let (by_seam, _, status) = taskfile_packet(&seam, &cdb_read10(3, 2), 512, false);

    assert_eq!(status & ST_CHK, 0);
    assert_eq!(by_seam.len(), 2 * BLOCK as usize);
    assert_eq!(by_seam, by_cable);
    assert_eq!(&by_seam[..BLOCK as usize], &stamp(3)[..]);
}

/// The interrupt count is the packet protocol's and not the transport's: a
/// packet data-in command of *n* blocks interrupts *n + 1* times, because its
/// completion is announced and an ATA PIO read's is not (§9.10 against §9.5).
/// Counted through the struct door, it has to come out the same.
#[test]
fn the_taskfile_door_takes_the_same_n_plus_one_interrupts() {
    let cd = drive(8);
    let _ = taskfile_packet(&cd, &cdb_test_unit_ready(), 512, false);
    let _ = taskfile_packet(&cd, &cdb_request_sense(18), 512, false);
    // One 2048-byte block at a 512-byte limit is four blocks on the link.
    let (data, interrupts, status) = taskfile_packet(&cd, &cdb_read10(1, 1), 512, false);
    assert_eq!(status & ST_CHK, 0);
    assert_eq!(data.len(), BLOCK as usize);
    assert_eq!(interrupts, 5, "four data blocks and one completion");
}

/// `PACKET` with the Features register's DMA bit is refused unless the drive
/// was configured for it, and accepted — reporting the DMA protocol per data
/// block — when it was. The adapter never reads an opcode to find this out.
#[test]
fn the_dma_bit_decides_the_protocol_the_drive_reports() {
    let store = RamStore::new(4 * BLOCK);
    let mut id = Identity::new();
    id.dma = true;
    let cd =
        AtapiDrive::with_disc(id, Position::Device0, Some(Arc::new(store))).expect("whole blocks");
    let _ = taskfile_packet(&cd, &cdb_test_unit_ready(), 512, true);
    let _ = taskfile_packet(&cd, &cdb_request_sense(18), 512, true);
    let (data, _, status) = taskfile_packet(&cd, &cdb_read10(0, 1), 2048, true);
    assert_eq!(status & ST_CHK, 0);
    assert_eq!(data.len(), BLOCK as usize);

    // A drive that was not configured for DMA aborts the command instead, which
    // is what keeps a driver from programming an engine that is not there.
    let plain = drive(4);
    let tf = Taskfile {
        command: cmd::PACKET,
        feature: 1,
        lba: 2048 << 8,
        count: 0,
        device: 0,
    };
    assert_eq!(TaskfileDevice::taskfile_start(&plain, &tf), Phase::Done);
    assert_ne!(
        TaskfileDevice::taskfile_registers(&plain).status & ST_CHK,
        0,
        "an unsupported DMA request is aborted"
    );
}

/// `IDENTIFY PACKET DEVICE` is an ATA command and never asks for a packet, so a
/// transport must not go looking for one.
#[test]
fn identify_packet_device_opens_a_data_phase_and_not_a_packet_one() {
    let cd = drive(4);
    let tf = Taskfile {
        command: cmd::IDENTIFY_PACKET,
        ..Taskfile::default()
    };
    let phase = TaskfileDevice::taskfile_start(&cd, &tf);
    assert_eq!(
        phase,
        Phase::Data {
            out: false,
            dma: false,
            block: 512,
        }
    );
    let mut block = alloc::vec![0u8; 512];
    assert_eq!(TaskfileDevice::taskfile_read(&cd, &mut block), 512);
    assert_eq!(TaskfileDevice::taskfile_phase(&cd), Phase::Done);
    let word0 = u16::from(block[0]) | (u16::from(block[1]) << 8);
    assert_eq!(word0 & 0xc000, 0x8000);
}

/// A command packet half delivered is state, and a snapshot taken there has to
/// carry it — along with whether the `PACKET` asked for DMA, which is the field
/// the chunk version was bumped for.
#[test]
fn a_snapshot_carries_a_half_delivered_packet_and_its_protocol() {
    let store = RamStore::new(4 * BLOCK);
    let mut id = Identity::new();
    id.dma = true;
    let saved = AtapiDrive::with_disc(id.clone(), Position::Device0, Some(Arc::new(store)))
        .expect("whole blocks");
    // Take the power-on unit attention first: it is owed to the first command
    // that is neither `INQUIRY` nor `REQUEST SENSE`, and a `READ(10)` that
    // collected it would end in `CHECK CONDITION` with no data phase at all.
    let _ = taskfile_packet(&saved, &cdb_test_unit_ready(), 512, true);
    let _ = taskfile_packet(&saved, &cdb_request_sense(18), 512, true);
    let tf = Taskfile {
        command: cmd::PACKET,
        feature: 1,
        lba: 2048 << 8,
        count: 0,
        device: 0,
    };
    assert!(matches!(
        TaskfileDevice::taskfile_start(&saved, &tf),
        Phase::Packet { .. }
    ));
    // Four of the twelve bytes. A `READ(10)` descriptor block is ten bytes
    // long and a twelve-byte packet carries it padded, which is what an
    // adapter reads out of a sixteen-byte `ACMD` field.
    let mut command = cdb_read10(1, 1);
    command.resize(PACKET_BYTES, 0);
    assert_eq!(TaskfileDevice::taskfile_write(&saved, &command[..4]), 4);
    let image = save_of(&saved);

    let store = RamStore::new(4 * BLOCK);
    let restored =
        AtapiDrive::with_disc(id, Position::Device0, Some(Arc::new(store))).expect("whole blocks");
    // Nothing is done to this one first: the snapshot carries the fact that the
    // attention has already been reported, which is part of what is being
    // checked.
    let reader = StateReader::new(&image).expect("a snapshot we just wrote");
    let chunk = reader
        .load("cd", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk");
    restored.load(&mut chunk.reader()).expect("it loads");
    assert_eq!(image, save_of(&restored));

    // It is still waiting for the other eight bytes, in the same phase.
    assert_eq!(
        TaskfileDevice::taskfile_phase(&restored),
        Phase::Packet { bytes: 8 }
    );
    assert_eq!(TaskfileDevice::taskfile_write(&restored, &command[4..]), 8);
    // And the DMA bit survived with it: the data phase reports the protocol the
    // `PACKET` before the snapshot asked for.
    assert_eq!(
        TaskfileDevice::taskfile_phase(&restored),
        Phase::Data {
            out: false,
            dma: true,
            block: BLOCK,
        }
    );
}

// ---------------------------------------------------------------------------
// the tray, opened from outside
// ---------------------------------------------------------------------------

/// A stamped disc of `blocks` logical blocks, as a medium a host can insert.
fn disc_medium(blocks: u64) -> Arc<dyn Medium> {
    let store = RamStore::new(blocks * BLOCK);
    for lba in 0..blocks {
        store.write_at(lba * BLOCK, &stamp(lba)).expect("in range");
    }
    Arc::new(store)
}

/// The sense a `REQUEST SENSE` reports right now, as `(key, asc, ascq)`.
fn sense_of(cd: &AtapiDrive) -> (u8, u8, u8) {
    let (_, sense) = packet_command_in(cd, &cdb_request_sense(18), 512);
    (sense[2] & 0x0f, sense[12], sense[13])
}

#[test]
fn a_disc_goes_in_and_comes_out_and_the_drive_says_which() {
    use crate::dev::medium::Removable;

    let cd = empty();
    assert_eq!(cd.bays().len(), 1, "one tray");
    assert_eq!(cd.bays()[0].name, "tray");
    assert!(cd.bays()[0].medium.is_none(), "an empty tray reads empty");

    cd.insert("tray", disc_medium(8), false)
        .expect("it goes in");
    let held = cd.bays()[0].medium.clone().expect("a disc is in it");
    assert_eq!(held.capacity, 8 * BLOCK);
    assert!(held.write_protected, "a CD has no write path at all");
    assert_eq!(cd.blocks(), 8);

    cd.eject("tray").expect("it comes out");
    assert!(cd.bays()[0].medium.is_none());
    assert_eq!(cd.blocks(), 0, "an empty tray has no capacity");

    // And a bay this drive does not have is named rather than ignored.
    let e = cd
        .insert("df0", disc_medium(8), false)
        .expect_err("no such bay");
    assert!(alloc::format!("{e}").contains("tray"), "{e}");
}

#[test]
fn a_swap_raises_the_unit_attention_the_command_set_prescribes() {
    use crate::dev::medium::Removable;

    // SFF-8020i §9.3: a medium change is reported once, as UNIT ATTENTION
    // with additional sense 28h/00h, on the next command that is neither
    // INQUIRY nor REQUEST SENSE.
    let cd = drive(8);
    clear_attention(&cd);
    assert_eq!(
        packet_command(&cd, &cdb_test_unit_ready(), 0).status & ST_CHK,
        0,
        "settled before the swap"
    );

    cd.insert("tray", disc_medium(16), false)
        .expect("a new disc");
    let ended = packet_command(&cd, &cdb_test_unit_ready(), 0);
    assert_eq!(ended.status & ST_CHK, ST_CHK, "the guest is told");
    assert_eq!(
        sense_of(&cd),
        (
            sense_key::UNIT_ATTENTION,
            asc::MEDIUM_CHANGED.0,
            asc::MEDIUM_CHANGED.1
        )
    );
    // Once, and then the drive is ready again on the new disc.
    assert_eq!(
        packet_command(&cd, &cdb_test_unit_ready(), 0).status & ST_CHK,
        0
    );
    let (_, data) = packet_command_in(&cd, &cdb_read10(15, 1), 2048);
    assert_eq!(data, stamp(15), "the new disc, and it is bigger");
}

#[test]
fn an_eject_is_told_the_same_way_and_then_the_medium_is_not_present() {
    use crate::dev::medium::Removable;

    let cd = drive(8);
    clear_attention(&cd);
    cd.eject("tray").expect("the tray opens");

    // The change first — a guest with a filesystem mounted has to be told the
    // disc left before it is told there is nothing there.
    let ended = packet_command(&cd, &cdb_test_unit_ready(), 0);
    assert_eq!(ended.status & ST_CHK, ST_CHK);
    assert_eq!(
        sense_of(&cd),
        (
            sense_key::UNIT_ATTENTION,
            asc::MEDIUM_CHANGED.0,
            asc::MEDIUM_CHANGED.1
        )
    );
    // And from then on, the empty-tray answer this drive has always given.
    for cdb in [cdb_test_unit_ready(), cdb_read10(0, 1)] {
        assert_eq!(packet_command(&cd, &cdb, 2048).status & ST_CHK, ST_CHK);
        assert_eq!(
            sense_of(&cd),
            (
                sense_key::NOT_READY,
                asc::MEDIUM_NOT_PRESENT.0,
                asc::MEDIUM_NOT_PRESENT.1
            )
        );
    }
}

#[test]
fn a_snapshot_round_trips_with_a_disc_and_without_one() {
    use crate::dev::medium::Removable;

    // Present.
    let cd = drive(8);
    clear_attention(&cd);
    let image = save_of(&cd);
    let restored = drive(8);
    let reader = StateReader::new(&image).expect("a snapshot we just wrote");
    let chunk = reader
        .load("cd", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk we just wrote");
    restored.load(&mut chunk.reader()).expect("it loads");
    assert_eq!(image, save_of(&restored));

    // Absent, and this one gets there by ejecting rather than by being built
    // empty — which is the state a swap actually produces.
    let opened = drive(8);
    clear_attention(&opened);
    opened.eject("tray").expect("the tray opens");
    let image = save_of(&opened);
    let target = drive(8);
    clear_attention(&target);
    target.eject("tray").expect("the tray opens");
    let reader = StateReader::new(&image).expect("a snapshot we just wrote");
    let chunk = reader
        .load("cd", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk we just wrote");
    target.load(&mut chunk.reader()).expect("it loads");
    assert_eq!(image, save_of(&target));
}

#[test]
fn looking_at_the_drive_does_not_consume_what_the_guest_has_not_read() {
    use crate::dev::medium::Removable;

    // `{:?}` is what the monitor's `device` command prints and `bays()` is
    // what its `media` command calls; neither may disturb the drive. The
    // claim worth making is about the pending sense, because that is the one
    // thing here a look could consume.
    let cd = drive(8);
    clear_attention(&cd);
    cd.eject("tray").expect("the tray opens");
    let _ = alloc::format!("{cd:?}");
    let _ = cd.bays();
    let ended = packet_command(&cd, &cdb_test_unit_ready(), 0);
    assert_eq!(ended.status & ST_CHK, ST_CHK, "still pending after a look");
    assert_eq!(
        sense_of(&cd),
        (
            sense_key::UNIT_ATTENTION,
            asc::MEDIUM_CHANGED.0,
            asc::MEDIUM_CHANGED.1
        )
    );
}
