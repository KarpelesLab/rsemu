//! The bus and the disk target, driven the way an initiator drives them:
//! select, walk the phases, move the bytes, let go.
//!
//! Nothing here knows what a WD33C93A is. The controller's half is
//! `src/dev/wd33c93/tests.rs`, and the two suites meeting in the middle is the
//! point of the split.

use super::*;
use crate::core::props::{Media, Props, Value};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::dev::scsi::disk::{BLOCK, CLASS_NAME, ScsiDisk};
use alloc::vec;
use alloc::vec::Vec;

/// The chunk version `scsi.disk` writes; private to that module, so the test
/// names it rather than importing it.
const STATE_VERSION: u32 = 1;

// ---------------------------------------------------------------------------
// rig
// ---------------------------------------------------------------------------

/// How many blocks the drive under test holds: small, and not a power of two
/// times the chunk size, so a multi-chunk transfer has a short tail.
const BLOCKS: u64 = 200;

/// A drive whose block *n* is filled with the byte `n`, so a read that lands
/// on the wrong block is visible at a glance.
fn drive() -> Arc<ScsiDisk> {
    let mut image = vec![0u8; (BLOCKS * BLOCK) as usize];
    for (n, block) in image.chunks_mut(BLOCK as usize).enumerate() {
        block.fill(n as u8);
    }
    let media = Media::new("hd0", image);
    let props = Props::new()
        .with("image", Value::Media(media))
        .with("id", Value::Uint(0));
    let device = DiskDevice::new(&props).expect("a drive");
    device.drive().expect("occupied")
}

/// Send `cdb` and read the whole `DATA IN` phase, the status byte and the
/// message, exactly as an initiator would. Returns `(data, status, message)`.
fn command(target: &dyn Target, cdb: &[u8]) -> (Vec<u8>, u8, u8) {
    assert!(target.select(true), "the target answers selection");
    assert_eq!(target.phase(), Phase::MessageOut);
    assert_eq!(target.write(&[message::IDENTIFY]), 1);
    assert_eq!(target.phase(), Phase::Command);
    assert_eq!(target.write(cdb), cdb.len());

    let mut data = Vec::new();
    while target.phase() == Phase::DataIn {
        let mut buf = [0u8; 64];
        let n = target.read(&mut buf);
        assert_ne!(n, 0, "a DATA IN phase that moves nothing never ends");
        data.extend_from_slice(&buf[..n]);
    }
    assert_eq!(target.phase(), Phase::Status);
    let mut status = [0u8; 1];
    assert_eq!(target.read(&mut status), 1);
    assert_eq!(target.phase(), Phase::MessageIn);
    let mut msg = [0u8; 1];
    assert_eq!(target.read(&mut msg), 1);
    assert_eq!(target.phase(), Phase::BusFree, "the target let go");
    (data, status[0], msg[0])
}

// ---------------------------------------------------------------------------
// the phases
// ---------------------------------------------------------------------------

#[test]
fn the_phase_signals_are_the_standards_three() {
    // X3.131-1994 §5.1, and the WD33C93A's `MCI` field, which is the same
    // three bits in the same order.
    assert_eq!(Phase::DataOut.mci(), Some(0b000));
    assert_eq!(Phase::DataIn.mci(), Some(0b001));
    assert_eq!(Phase::Command.mci(), Some(0b010));
    assert_eq!(Phase::Status.mci(), Some(0b011));
    assert_eq!(Phase::MessageOut.mci(), Some(0b110));
    assert_eq!(Phase::MessageIn.mci(), Some(0b111));
    assert_eq!(Phase::BusFree.mci(), None);
    for p in [Phase::DataIn, Phase::Status, Phase::MessageIn] {
        assert!(p.is_input(), "{p:?} moves bytes target → initiator");
    }
    for p in [Phase::DataOut, Phase::Command, Phase::MessageOut] {
        assert!(!p.is_input());
    }
}

#[test]
fn a_group_code_says_how_long_a_command_is() {
    // X3.131-1994 §7.1, Table 20.
    assert_eq!(cdb_len(0x00), Some(6)); // TEST UNIT READY
    assert_eq!(cdb_len(0x28), Some(10)); // READ(10)
    assert_eq!(cdb_len(0x5a), Some(10)); // MODE SENSE(10)
    assert_eq!(cdb_len(0xa8), Some(12)); // READ(12)'s group
    assert_eq!(cdb_len(0x60), None); // reserved
    assert_eq!(cdb_len(0xc0), None); // vendor specific
}

#[test]
fn an_inquiry_walks_selection_command_data_status_and_message() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    assert_eq!(target.phase(), Phase::BusFree);
    let (data, status, msg) = command(target, &[0x12, 0, 0, 0, 36, 0]);
    assert_eq!(status, status::GOOD);
    assert_eq!(msg, message::COMMAND_COMPLETE);
    assert_eq!(data.len(), 36);
    // §8.2.5, Table 45: a direct-access device that is not removable, SCSI-2,
    // response format 2, thirty-one more bytes.
    assert_eq!(data[0], 0x00);
    assert_eq!(data[1], 0x00);
    assert_eq!(data[2], 0x02);
    assert_eq!(data[3], 0x02);
    assert_eq!(data[4], 31);
    assert_eq!(&data[8..13], b"RSEMU");
    assert_eq!(&data[16..29], b"SCSI HARDDISK");
    assert_eq!(&data[32..35], b"1.0");
}

#[test]
fn a_read_10_moves_the_blocks_it_asked_for_in_order() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    // Three blocks from block 5 — and more than one refill of the internal
    // chunk is exercised by the long read below.
    let (data, status, _) = command(target, &[0x28, 0, 0, 0, 0, 5, 0, 0, 3, 0]);
    assert_eq!(status, status::GOOD);
    assert_eq!(data.len(), 3 * BLOCK as usize);
    for (i, block) in data.chunks(BLOCK as usize).enumerate() {
        assert!(
            block.iter().all(|&b| b == (5 + i) as u8),
            "block {} is not block {}",
            i,
            5 + i
        );
    }
}

#[test]
fn a_read_longer_than_one_chunk_is_still_one_stream() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    // 100 blocks is more than the 64 the target refills in one go, so the
    // seam between two medium accesses is inside the data phase.
    let (data, status, _) = command(target, &[0x28, 0, 0, 0, 0, 0, 0, 0, 100, 0]);
    assert_eq!(status, status::GOOD);
    assert_eq!(data.len(), 100 * BLOCK as usize);
    for (i, block) in data.chunks(BLOCK as usize).enumerate() {
        assert!(block.iter().all(|&b| b == i as u8), "block {i} is wrong");
    }
}

#[test]
fn a_read_6_addresses_twenty_one_bits_and_counts_zero_as_two_hundred_and_fifty_six() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    let (data, status, _) = command(target, &[0x08, 0x00, 0x00, 0x07, 2, 0]);
    assert_eq!(status, status::GOOD);
    assert_eq!(data.len(), 2 * BLOCK as usize);
    assert!(data[..BLOCK as usize].iter().all(|&b| b == 7));

    // §9.2.5: "a transfer length of zero indicates that 256 blocks shall be
    // transferred" — and this drive holds 200, so it is out of range.
    let (data, status, _) = command(target, &[0x08, 0, 0, 0, 0, 0]);
    assert!(data.is_empty());
    assert_eq!(status, status::CHECK_CONDITION);
}

#[test]
fn read_capacity_reports_the_last_block_not_the_count() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    let (data, status, _) = command(target, &[0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
    assert_eq!(status, status::GOOD);
    assert_eq!(data.len(), 8);
    // §9.2.7: the *logical block address* of the last block.
    assert_eq!(
        u32::from_be_bytes([data[0], data[1], data[2], data[3]]),
        (BLOCKS - 1) as u32
    );
    assert_eq!(
        u32::from_be_bytes([data[4], data[5], data[6], data[7]]),
        BLOCK as u32
    );
}

#[test]
fn a_write_10_lands_on_the_medium_and_reads_back() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    let payload: Vec<u8> = (0..BLOCK as usize).map(|i| (i * 3) as u8).collect();

    assert!(target.select(true));
    assert_eq!(target.write(&[message::IDENTIFY]), 1);
    assert_eq!(target.write(&[0x2a, 0, 0, 0, 0, 9, 0, 0, 1, 0]), 10);
    assert_eq!(target.phase(), Phase::DataOut);
    assert_eq!(target.write(&payload), payload.len());
    assert_eq!(target.phase(), Phase::Status);
    let mut status = [0u8; 1];
    target.read(&mut status);
    assert_eq!(status[0], status::GOOD);
    let mut msg = [0u8; 1];
    target.read(&mut msg);
    assert_eq!(target.phase(), Phase::BusFree);

    let (data, status, _) = command(target, &[0x28, 0, 0, 0, 0, 9, 0, 0, 1, 0]);
    assert_eq!(status, status::GOOD);
    assert_eq!(data, payload);
}

#[test]
fn a_command_this_drive_does_not_have_is_an_illegal_request() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    // FORMAT UNIT: a real command this model does not implement, which must
    // therefore be refused rather than quietly succeed.
    let (data, status, _) = command(target, &[0x04, 0, 0, 0, 0, 0]);
    assert!(data.is_empty());
    assert_eq!(status, status::CHECK_CONDITION);

    // And the sense data says exactly why (§8.2.14, Table 65).
    let (sense, status, _) = command(target, &[0x03, 0, 0, 0, 18, 0]);
    assert_eq!(status, status::GOOD);
    assert_eq!(sense.len(), 18);
    assert_eq!(sense[0], 0x70, "a current error in the fixed format");
    assert_eq!(sense[2] & 0x0f, sense::ILLEGAL_REQUEST);
    assert_eq!(sense[7], 10, "ten more bytes after this one");
    assert_eq!(sense[12], sense::ASC_INVALID_COMMAND);

    // §8.2.14: reading the sense clears it.
    let (again, _, _) = command(target, &[0x03, 0, 0, 0, 18, 0]);
    assert_eq!(again[2] & 0x0f, sense::NO_SENSE);
}

#[test]
fn a_block_past_the_end_is_refused_and_says_which_one() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    let lba = BLOCKS as u32 - 1;
    let b = lba.to_be_bytes();
    // One block at the last address is fine; two is one too many.
    let (_, status, _) = command(target, &[0x28, 0, b[0], b[1], b[2], b[3], 0, 0, 1, 0]);
    assert_eq!(status, status::GOOD);
    let (_, status, _) = command(target, &[0x28, 0, b[0], b[1], b[2], b[3], 0, 0, 2, 0]);
    assert_eq!(status, status::CHECK_CONDITION);
    let (s, _, _) = command(target, &[0x03, 0, 0, 0, 18, 0]);
    assert_eq!(s[2] & 0x0f, sense::ILLEGAL_REQUEST);
    assert_eq!(s[12], sense::ASC_LBA_OUT_OF_RANGE);
    assert_eq!(s[0] & 0x80, 0x80, "the INFORMATION field is valid");
    assert_eq!(u32::from_be_bytes([s[3], s[4], s[5], s[6]]), lba);
}

#[test]
fn mode_sense_carries_a_block_descriptor_and_the_pages_asked_for() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    // Page 04, rigid disk drive geometry (§9.3.3.7).
    let (data, status, _) = command(target, &[0x1a, 0, 0x04, 0, 0xff, 0]);
    assert_eq!(status, status::GOOD);
    assert_eq!(
        data[0] as usize,
        data.len() - 1,
        "the length counts itself out"
    );
    assert_eq!(data[3], 8, "a block descriptor");
    let blocks = u32::from_be_bytes([0, data[5], data[6], data[7]]);
    assert_eq!(u64::from(blocks), BLOCKS);
    let page = &data[12..];
    assert_eq!(page[0], 0x04);
    assert_eq!(page[1], 0x16);
    let (cylinders, heads, sectors) = disk.geometry();
    assert_eq!(
        u32::from_be_bytes([0, page[2], page[3], page[4]]),
        cylinders
    );
    assert_eq!(page[5], heads);
    // The geometry covers the medium and overshoots by less than a cylinder,
    // which is what rounding a block count up to whole cylinders means.
    let covered = u64::from(cylinders) * u64::from(heads) * u64::from(sectors);
    assert!(covered >= BLOCKS);
    assert!(covered - BLOCKS < u64::from(heads) * u64::from(sectors));

    // `DBD` leaves the descriptor out (§8.3.3).
    let (data, _, _) = command(target, &[0x1a, 0x08, 0x04, 0, 0xff, 0]);
    assert_eq!(data[3], 0);

    // Page 3F is every page, in ascending page-code order (§8.2.10).
    let (all, _, _) = command(target, &[0x1a, 0x08, 0x3f, 0, 0xff, 0]);
    assert_eq!(all[4], 0x01);
    assert_eq!(all[4 + 12], 0x03);
    assert_eq!(all[4 + 12 + 24], 0x04);
}

#[test]
fn an_allocation_length_truncates_and_a_short_one_still_ends_the_phase() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    let (data, status, _) = command(target, &[0x12, 0, 0, 0, 5, 0]);
    assert_eq!(status, status::GOOD);
    assert_eq!(data.len(), 5);
    // Zero bytes wanted is not an error and there is no data phase at all.
    let (data, status, _) = command(target, &[0x12, 0, 0, 0, 0, 0]);
    assert!(data.is_empty());
    assert_eq!(status, status::GOOD);
}

#[test]
fn an_unsupported_logical_unit_answers_inquiry_and_refuses_everything_else() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    assert!(target.select(true));
    // §6.6.7: `IDENTIFY` with LUN 3.
    assert_eq!(target.write(&[message::IDENTIFY | 3]), 1);
    assert_eq!(target.write(&[0x12, 0, 0, 0, 36, 0]), 6);
    let mut data = [0u8; 36];
    let n = target.read(&mut data);
    assert_eq!(n, 36);
    // §8.2.5: peripheral qualifier 3, device type 1F — "not here, and never
    // will be".
    assert_eq!(data[0], 0x7f);
    target.release();

    assert!(target.select(true));
    assert_eq!(target.write(&[message::IDENTIFY | 3]), 1);
    assert_eq!(target.write(&[0x00, 0, 0, 0, 0, 0]), 6);
    assert_eq!(target.phase(), Phase::Status);
    let mut status = [0u8; 1];
    target.read(&mut status);
    assert_eq!(status[0], status::CHECK_CONDITION);
}

#[test]
fn a_bus_reset_is_a_unit_attention_the_next_command_sees() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    target.bus_reset();
    let (s, status, _) = command(target, &[0x03, 0, 0, 0, 18, 0]);
    assert_eq!(status, status::GOOD);
    assert_eq!(s[2] & 0x0f, sense::UNIT_ATTENTION);
    assert_eq!(s[12], sense::ASC_RESET);
}

#[test]
fn letting_go_mid_command_leaves_the_target_ready_for_the_next_one() {
    let disk = drive();
    let target: &dyn Target = disk.as_ref();
    assert!(target.select(true));
    assert_eq!(target.write(&[message::IDENTIFY]), 1);
    // Half a command descriptor block, then the initiator drops the bus.
    assert_eq!(target.write(&[0x28, 0, 0]), 3);
    target.release();
    assert_eq!(target.phase(), Phase::BusFree);
    // And the next operation is unaffected.
    let (_, status, _) = command(target, &[0x00, 0, 0, 0, 0, 0]);
    assert_eq!(status, status::GOOD);
}

// ---------------------------------------------------------------------------
// the bus
// ---------------------------------------------------------------------------

#[test]
fn a_bus_holds_one_target_per_address() {
    let bus = Bus::new();
    assert!(bus.occupied().is_empty());
    let first: Arc<dyn Target> = drive();
    let second: Arc<dyn Target> = drive();
    assert!(bus.fit(3, Arc::clone(&first)).is_ok());
    assert!(bus.fit(3, Arc::clone(&second)).is_err(), "3 is taken");
    assert!(bus.fit(8, second).is_err(), "eight addresses, 0 through 7");
    assert_eq!(bus.occupied(), vec![3]);
    assert!(bus.target(3).is_some());
    assert!(bus.target(4).is_none());
    assert!(bus.remove(3).is_some());
    assert!(bus.occupied().is_empty());
}

#[test]
fn a_bus_reset_reaches_every_target_on_the_cable() {
    let bus = Bus::new();
    let a = drive();
    let b = drive();
    bus.fit(0, Arc::clone(&a) as Arc<dyn Target>).ok();
    bus.fit(1, Arc::clone(&b) as Arc<dyn Target>).ok();
    // Mid-command on one of them, to show `RST` clears the connection too.
    assert!(a.select(true));
    bus.reset();
    assert_eq!(Target::phase(a.as_ref()), Phase::BusFree);
    for target in [&a, &b] {
        let (s, _, _) = command(target.as_ref(), &[0x03, 0, 0, 0, 18, 0]);
        assert_eq!(s[2] & 0x0f, sense::UNIT_ATTENTION);
    }
}

#[test]
fn a_media_slot_with_no_bytes_is_an_empty_address() {
    let props = Props::new()
        .with("image", Value::Media(Media::new("hd0", Vec::new())))
        .with("id", Value::Uint(2));
    let device = DiskDevice::new(&props).expect("an empty bay is not an error");
    assert!(device.drive().is_none());
    assert!(device.bus().occupied().is_empty());
}

#[test]
fn two_drives_at_one_address_is_a_configuration_error() {
    let hosts = Arc::new(crate::core::hosts::HostObjects::new());
    let make = |id: u64| {
        Props::new()
            .with("image", Value::Media(Media::new("hd0", vec![0u8; 8 * 512])))
            .with("id", Value::Uint(id))
            .with_hosts(Arc::clone(&hosts))
    };
    DiskDevice::new(&make(1)).expect("the first fits");
    let clash = DiskDevice::new(&make(1)).expect_err("the second does not");
    assert!(alloc::format!("{clash}").contains("address 1"), "{clash}");
}

// ---------------------------------------------------------------------------
// snapshot
// ---------------------------------------------------------------------------

fn snapshot(d: &DiskDevice) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("hd0", CLASS_NAME).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("hd0", CLASS_NAME, STATE_VERSION).unwrap();
        crate::core::device::Device::save(d, &mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

fn device(id: u64) -> DiskDevice {
    let props = Props::new()
        .with(
            "image",
            Value::Media(Media::new("hd0", vec![0u8; 16 * 512])),
        )
        .with("id", Value::Uint(id));
    DiskDevice::new(&props).expect("a drive")
}

#[test]
fn a_snapshot_round_trips_to_identical_state() {
    let saved = device(0);
    let target = saved.drive().expect("a drive");
    // Stopped part way through a read, with sense data owing and a command
    // block half received — every field that is not the medium.
    {
        let t: &dyn Target = target.as_ref();
        let (_, _, _) = command(t, &[0x04, 0, 0, 0, 0, 0]);
        assert!(t.select(true));
        t.write(&[message::IDENTIFY]);
        t.write(&[0x28, 0, 0]);
    }
    target
        .load_image(512, &[1, 2, 3, 4])
        .expect("the medium takes it");
    let bytes = snapshot(&saved);

    let restored = device(0);
    assert_ne!(snapshot(&restored), bytes);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("hd0", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    crate::core::device::Device::load(&restored, &mut chunk.reader()).unwrap();
    assert_eq!(snapshot(&restored), bytes, "identical state");

    // And the restored target carries on from where the other one stopped.
    let back = restored.drive().expect("a drive");
    let t: &dyn Target = back.as_ref();
    assert_eq!(t.phase(), Phase::Command);
    assert_eq!(t.write(&[0, 0, 1, 0, 0, 1, 0]), 7);
    let mut data = [0u8; 512];
    assert_eq!(t.read(&mut data), 512);
    assert_eq!(&data[..4], &[1, 2, 3, 4]);
}

#[test]
fn a_snapshot_from_a_different_address_is_refused() {
    let bytes = snapshot(&device(0));
    let other = device(5);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load("hd0", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .unwrap();
    let e = crate::core::device::Device::load(&other, &mut chunk.reader())
        .expect_err("address 0 is not address 5");
    assert!(alloc::format!("{e}").contains("address"), "{e}");
}
