//! What a NOR part must refuse, and what a snapshot must carry.

use super::*;
use crate::core::props::Value;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

/// Two x16 parts on a 32-bit bus, four 4 KiB blocks, powering up unlocked so a
/// test that is not about locking does not have to unlock first.
fn flash(blocks: u64, block: u64) -> Cfi {
    let geom = Geometry::uniform(blocks * block, block, 4, 2).expect("a plausible part");
    Cfi::from_array(Arc::new(
        Array::with_options(geom, DEFAULT_MANUFACTURER, 0x8899, false, false).expect("fits"),
    ))
}

fn poke(cfi: &Cfi, offset: u64, value: u32) {
    cfi.array()
        .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a word write is a legal bus cycle");
}

/// The idiom a driver on a two-part bus uses: the same 16-bit command in both
/// halves of one 32-bit write.
fn command(cfi: &Cfi, offset: u64, cmd: u16) {
    poke(cfi, offset, (u32::from(cmd) << 16) | u32::from(cmd));
}

fn peek(cfi: &Cfi, offset: u64) -> u32 {
    let mut bytes = [0u8; 4];
    cfi.array()
        .read(offset, &mut bytes, MemAttrs::DEFAULT)
        .expect("a word read is a legal bus cycle");
    u32::from_le_bytes(bytes)
}

fn contents(cfi: &Cfi, offset: u64, len: usize) -> Vec<u8> {
    let mut out = alloc::vec![0u8; len];
    cfi.array().read_contents(offset, &mut out).expect("inside");
    out
}

/// Program one 32-bit word the way EDK II's driver does: setup, then the data.
fn word_program(cfi: &Cfi, offset: u64, value: u32) {
    command(cfi, offset, 0x0040);
    poke(cfi, offset, value);
}

fn erase_block(cfi: &Cfi, offset: u64) {
    command(cfi, offset, 0x0020);
    command(cfi, offset, 0x00d0);
}

fn unlock_block(cfi: &Cfi, offset: u64) {
    command(cfi, offset, 0x0060);
    command(cfi, offset, 0x00d0);
}

fn read_array(cfi: &Cfi) {
    command(cfi, 0, 0x00ff);
}

// -- the three properties that make it flash --------------------------------

#[test]
fn an_unwritten_part_reads_all_ones() {
    // Not zeroes. Firmware that finds zeroes concludes every byte has been
    // programmed and refuses to write anything more.
    let cfi = flash(4, 0x1000);
    assert_eq!(peek(&cfi, 0), 0xffff_ffff);
    assert_eq!(contents(&cfi, 0, 4), alloc::vec![0xff; 4]);
}

#[test]
fn a_program_can_only_clear_bits() {
    let cfi = flash(4, 0x1000);
    word_program(&cfi, 0, 0x0f0f_0f0f);
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0), 0x0f0f_0f0f);

    // Now try to put the high nibbles back. A memory would take it; flash
    // ANDs, so nothing changes at all.
    word_program(&cfi, 0, 0xf0f0_f0f0);
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0), 0x0000_0000, "0x0f0f & 0xf0f0 is zero");

    // And a second program that clears further bits is fine — this is exactly
    // what a fault-tolerant write depends on.
    word_program(&cfi, 4, 0xffff_00ff);
    word_program(&cfi, 4, 0xff00_00ff);
    read_array(&cfi);
    assert_eq!(peek(&cfi, 4), 0xff00_00ff);
}

#[test]
fn only_an_erase_puts_bits_back_and_it_takes_the_whole_block() {
    let cfi = flash(4, 0x1000);
    word_program(&cfi, 0x0000, 0);
    word_program(&cfi, 0x0ffc, 0);
    word_program(&cfi, 0x1000, 0);
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0x0000), 0);
    assert_eq!(peek(&cfi, 0x1000), 0);

    // Erase the first block, from an address in the middle of it.
    erase_block(&cfi, 0x0800);
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0x0000), 0xffff_ffff, "the whole block came back");
    assert_eq!(peek(&cfi, 0x0ffc), 0xffff_ffff);
    assert_eq!(peek(&cfi, 0x1000), 0, "and only that block");
}

#[test]
fn a_read_during_a_command_is_status_not_data() {
    let cfi = flash(4, 0x1000);
    word_program(&cfi, 0, 0x1234_5678);
    // The part has not been told to read the array again, so a read of the
    // address just programmed answers with two status registers, not data.
    assert_eq!(peek(&cfi, 0), 0x0080_0080, "SR.7 in each half");
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0), 0x1234_5678);
}

// -- the command set --------------------------------------------------------

#[test]
fn the_cfi_query_says_qry_and_the_intel_command_set() {
    let cfi = flash(4, 0x1000);
    command(&cfi, 0, 0x0098);
    // Query offsets are per-device word indices; on a four-byte bus that is
    // four bytes apart, and each device answers in its own halfword.
    assert_eq!(peek(&cfi, 0x10 * 4), 0x0051_0051, "Q");
    assert_eq!(peek(&cfi, 0x11 * 4), 0x0052_0052, "R");
    assert_eq!(peek(&cfi, 0x12 * 4), 0x0059_0059, "Y");
    assert_eq!(peek(&cfi, 0x13 * 4), 0x0001_0001, "Intel/Sharp extended");
    // Two 4 KiB blocks' worth per device: 8 KiB each, so 2^13.
    assert_eq!(peek(&cfi, 0x27 * 4), 0x000d_000d, "device size 2^13");
    assert_eq!(peek(&cfi, 0x2c * 4), 0x0001_0001, "one erase-block region");
    // Four blocks, reported as "count - 1"; 4096 bus bytes is 2048 per device,
    // stated in units of 256.
    assert_eq!(peek(&cfi, 0x2d * 4), 0x0003_0003);
    assert_eq!(peek(&cfi, 0x2f * 4), 0x0008_0008);
    // And the extended table right behind the single region descriptor.
    assert_eq!(peek(&cfi, 0x31 * 4), 0x0050_0050, "P");
    assert_eq!(peek(&cfi, 0x33 * 4), 0x0049_0049, "I");
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0x10 * 4), 0xffff_ffff, "back to the array");
}

#[test]
fn a_query_of_a_boot_block_part_describes_both_regions() {
    // Four 16 KiB blocks then sixty-three 64 KiB ones: the classic bottom-boot
    // layout, and the reason `Geometry` is a list.
    let geom = Geometry::new(
        alloc::vec![
            BlockRegion {
                count: 4,
                size: 16 * 1024
            },
            BlockRegion {
                count: 63,
                size: 64 * 1024
            },
        ],
        4,
        2,
    )
    .expect("a boot-block part");
    assert_eq!(geom.size(), 4 * 16 * 1024 + 63 * 64 * 1024);
    assert_eq!(geom.block_count(), 67);
    assert_eq!(geom.block_at(0), Some((0, 0, 16 * 1024)));
    assert_eq!(geom.block_at(0xffff), Some((3, 0xc000, 16 * 1024)));
    assert_eq!(geom.block_at(0x10000), Some((4, 0x10000, 64 * 1024)));

    let cfi = Cfi::from_array(Arc::new(
        Array::with_options(geom, DEFAULT_MANUFACTURER, 0, false, false).expect("fits"),
    ));
    command(&cfi, 0, 0x0098);
    assert_eq!(peek(&cfi, 0x2c * 4), 0x0002_0002, "two regions");
    assert_eq!(peek(&cfi, 0x2d * 4), 0x0003_0003, "four blocks");
    assert_eq!(peek(&cfi, 0x2f * 4), 0x0020_0020, "8 KiB each per device");
    assert_eq!(peek(&cfi, 0x31 * 4), 0x003e_003e, "sixty-three blocks");
    assert_eq!(peek(&cfi, 0x33 * 4), 0x0080_0080, "32 KiB each per device");
    // With two regions the extended table has been pushed out to 0x35.
    assert_eq!(peek(&cfi, 0x35 * 4), 0x0050_0050, "P");
    // And the query says where it is, rather than a driver guessing.
    assert_eq!(peek(&cfi, 0x15 * 4), 0x0035_0035);
}

#[test]
fn identifier_mode_reports_the_manufacturer_and_the_addressed_blocks_lock() {
    let cfi = flash(4, 0x1000);
    command(&cfi, 0x2000, 0x0090);
    assert_eq!(peek(&cfi, 0x2000), 0x0089_0089, "Intel, JEP106");
    assert_eq!(peek(&cfi, 0x2004), 0x8899_8899, "the device code");
    assert_eq!(peek(&cfi, 0x2008), 0, "block 2 is unlocked");
    command(&cfi, 0x2000, 0x0060);
    command(&cfi, 0x2000, 0x0001);
    command(&cfi, 0x2000, 0x0090);
    assert_eq!(peek(&cfi, 0x2008), 0x0001_0001, "and now it is locked");
    // A different block's lock bit is read at that block's own address.
    assert_eq!(peek(&cfi, 0x0008), 0, "block 0 was not touched");
}

#[test]
fn a_buffered_program_commits_only_on_the_confirm() {
    let cfi = flash(4, 0x1000);
    // Setup at the target, then the word count less one, then the data, then
    // the confirm at the device base — which is where the driver sends it.
    command(&cfi, 0x40, 0x00e8);
    assert_eq!(peek(&cfi, 0x40), 0x0080_0080, "the buffer is available");
    command(&cfi, 0x40, 0x0003);
    for i in 0..4u64 {
        poke(&cfi, 0x40 + i * 4, 0xaaaa_0000 | (i as u32));
    }
    // A read-array command where the confirm should be aborts the sequence
    // rather than being a command: the buffer is waiting for exactly one
    // cycle, and the one it got was not `0xd0`.
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0x40), 0x00b0_00b0, "command sequence error");
    command(&cfi, 0, 0x0050);
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0x40), 0xffff_ffff, "and nothing was programmed");

    // Again, and this time confirm it.
    command(&cfi, 0x40, 0x00e8);
    command(&cfi, 0x40, 0x0003);
    for i in 0..4u64 {
        poke(&cfi, 0x40 + i * 4, 0xaaaa_0000 | (i as u32));
    }
    command(&cfi, 0, 0x00d0);
    read_array(&cfi);
    for i in 0..4u64 {
        assert_eq!(peek(&cfi, 0x40 + i * 4), 0xaaaa_0000 | (i as u32));
    }
}

#[test]
fn a_locked_block_refuses_a_program_and_an_erase() {
    // Powering up locked is what an Intel P30 does, and the driver above
    // unlocks before every write because of it.
    let geom = Geometry::uniform(4 * 0x1000, 0x1000, 4, 2).expect("a plausible part");
    let cfi = Cfi::from_array(Arc::new(
        Array::with_options(geom, DEFAULT_MANUFACTURER, 0, true, false).expect("fits"),
    ));
    assert_eq!(cfi.array().is_locked(0, 0), Some(true));

    word_program(&cfi, 0, 0);
    assert_eq!(
        peek(&cfi, 0),
        0x0092_0092,
        "SR.7 ready, SR.4 program error, SR.1 locked"
    );
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0), 0xffff_ffff, "and nothing was programmed");

    // Clear the status, unlock, and it works.
    command(&cfi, 0, 0x0050);
    unlock_block(&cfi, 0);
    assert_eq!(cfi.array().is_locked(0, 0), Some(false));
    word_program(&cfi, 0, 0x1234_5678);
    assert_eq!(peek(&cfi, 0), 0x0080_0080, "no error this time");
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0), 0x1234_5678);

    // An erase of a block that is still locked is refused too, and says so
    // with SR.5 rather than SR.4.
    erase_block(&cfi, 0x1000);
    assert_eq!(peek(&cfi, 0x1000), 0x00a2_00a2, "SR.5 erase, SR.1 locked");
}

#[test]
fn a_locked_down_block_cannot_be_unlocked_which_is_what_read_only_means() {
    let geom = Geometry::uniform(2 * 0x1000, 0x1000, 4, 2).expect("a plausible part");
    let cfi = Cfi::from_array(Arc::new(
        Array::with_options(geom, DEFAULT_MANUFACTURER, 0, true, true).expect("fits"),
    ));
    unlock_block(&cfi, 0);
    assert_eq!(cfi.array().is_locked(0, 0), Some(true), "WP# is low");
    assert_eq!(peek(&cfi, 0), 0x0082_0082, "SR.1: the unlock was refused");
    command(&cfi, 0, 0x0050);
    word_program(&cfi, 0, 0);
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0), 0xffff_ffff, "and nothing can be written");
}

#[test]
fn an_unrecognised_command_is_a_command_sequence_error() {
    let cfi = flash(4, 0x1000);
    command(&cfi, 0, 0x00a5);
    // SR.4 and SR.5 together is how the Intel set spells "that sequence was
    // not a command".
    assert_eq!(peek(&cfi, 0), 0x00b0_00b0);
    // Clear Status Register takes them away and leaves the read mode alone.
    command(&cfi, 0, 0x0050);
    assert_eq!(peek(&cfi, 0), 0x0080_0080);
}

#[test]
fn an_erase_whose_confirm_is_missing_erases_nothing() {
    let cfi = flash(4, 0x1000);
    word_program(&cfi, 0, 0);
    command(&cfi, 0, 0x0020);
    command(&cfi, 0, 0x0070); // a status read where the confirm should be
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0), 0, "still programmed");
}

#[test]
fn each_part_on_the_bus_has_its_own_state_machine() {
    // A command sent to only the low halfword leaves the high one reading the
    // array, which is the whole reason a lane is not an implementation detail.
    let cfi = flash(4, 0x1000);
    cfi.array()
        .write(0, &0x0070u16.to_le_bytes(), MemAttrs::DEFAULT)
        .expect("a halfword write reaches one part");
    assert_eq!(
        peek(&cfi, 0),
        0xffff_0080,
        "status from the low part, array from the high one"
    );
}

// -- the framework contracts ------------------------------------------------

#[test]
fn a_debug_read_sees_the_array_and_a_debug_write_is_refused() {
    let cfi = flash(4, 0x1000);
    word_program(&cfi, 0, 0x1111_2222);
    // The part is in status mode, but a debugger asked what is *in* the flash.
    let mut bytes = [0u8; 4];
    cfi.array()
        .read(0, &mut bytes, MemAttrs::DEBUG)
        .expect("a debug read is always legal");
    assert_eq!(u32::from_le_bytes(bytes), 0x1111_2222);
    // And the state machine did not move: an ordinary read still says status.
    assert_eq!(peek(&cfi, 0), 0x0080_0080);

    // A debug write would advance the state machine — erasing a block by
    // looking at it is exactly what invariant 5 forbids.
    assert!(
        cfi.array()
            .write(0, &0x00ff_00ffu32.to_le_bytes(), MemAttrs::DEBUG)
            .is_err()
    );
    assert_eq!(peek(&cfi, 0), 0x0080_0080, "still in status mode");
}

#[test]
fn a_reset_returns_the_command_state_and_keeps_the_contents() {
    let cfi = flash(4, 0x1000);
    word_program(&cfi, 0, 0xdead_beef);
    command(&cfi, 0, 0x0098);
    assert!(!cfi.array().is_reading_array());

    cfi.reset(ResetKind::Cold);
    assert!(cfi.array().is_reading_array());
    // Flash is non-volatile. A cold reset that restored the factory image
    // would defeat the entire point of the device.
    assert_eq!(peek(&cfi, 0), 0xdead_beef);
    assert_eq!(cfi.array().status(0), Some(0x80));
}

#[test]
fn a_snapshot_carries_a_half_issued_erase() {
    // The state that only a real flash model has: a machine saved between the
    // erase setup and its confirm has an erase half-issued, and a loader that
    // dropped it would silently swallow the erase.
    let saved = flash(4, 0x1000);
    word_program(&saved, 0, 0x0f0f_0f0f);
    word_program(&saved, 0x2000, 0x0f0f_0f0f);
    command(&saved, 0x1000, 0x0060);
    command(&saved, 0x1000, 0x0001); // lock block 1
    command(&saved, 0x2000, 0x0020); // and set an erase going on block 2

    let bytes = snapshot(&saved);
    let restored = flash(4, 0x1000);
    restore(&restored, &bytes);
    assert_eq!(snapshot(&restored), bytes, "identical state");
    assert_eq!(restored.array().is_locked(0, 1), Some(true));

    // The restored machine finishes what the saved one had begun: one more
    // cycle, and the block it was aimed at goes.
    command(&restored, 0x2000, 0x00d0);
    read_array(&restored);
    assert_eq!(peek(&restored, 0x2000), 0xffff_ffff, "the erase completed");
    assert_eq!(peek(&restored, 0), 0x0f0f_0f0f, "and only that block");
}

#[test]
fn a_snapshot_carries_a_staged_write_buffer() {
    let saved = flash(4, 0x1000);
    command(&saved, 0x3000, 0x00e8);
    command(&saved, 0x3000, 0x0003); // four words to come
    poke(&saved, 0x3000, 0x1111_1111); // one of which has arrived

    let bytes = snapshot(&saved);
    let restored = flash(4, 0x1000);
    restore(&restored, &bytes);
    assert_eq!(snapshot(&restored), bytes, "identical state");

    // Three more words and a confirm, and the whole buffer lands — including
    // the word that was written before the snapshot.
    for i in 1..4u64 {
        poke(&restored, 0x3000 + i * 4, 0x1111_1111);
    }
    command(&restored, 0, 0x00d0);
    read_array(&restored);
    assert_eq!(peek(&restored, 0x3000), 0x1111_1111);
    assert_eq!(peek(&restored, 0x300c), 0x1111_1111);
    assert_eq!(peek(&restored, 0x3010), 0xffff_ffff, "and no further");
}

fn snapshot(cfi: &Cfi) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape
        .add_device("flash", CLASS.name)
        .expect("a fresh shape");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("flash", CLASS.name, CLASS.version)
            .expect("one chunk");
        cfi.save(&mut chunk).expect("the flash saves");
    }
    w.to_vec().expect("a snapshot")
}

fn restore(cfi: &Cfi, bytes: &[u8]) {
    let reader = StateReader::new(bytes).expect("a snapshot");
    let chunk = reader
        .load("flash", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    cfi.load(&mut chunk.reader()).expect("the flash loads");
}

#[test]
fn a_snapshot_from_a_differently_shaped_part_is_refused() {
    let big = flash(8, 0x1000);
    let bytes = snapshot(&big);
    let small = flash(4, 0x1000);
    let reader = StateReader::new(&bytes).expect("a snapshot");
    let chunk = reader
        .load("flash", CLASS.name, CLASS.version, &Migrations::new())
        .expect("the chunk is there");
    let e = small
        .load(&mut chunk.reader())
        .expect_err("32 KiB is not 16 KiB")
        .to_string();
    assert!(e.contains("32768") && e.contains("16384"), "{e}");
}

// -- construction -----------------------------------------------------------

#[test]
fn a_size_is_required_and_the_geometry_has_to_be_possible() {
    assert!(Cfi::new(&Props::new()).is_err(), "no size at all");

    let e = Cfi::new(
        &Props::new()
            .with("size", Value::Size(0x3000))
            .with("block", Value::Size(0x2000)),
    )
    .expect_err("0x3000 does not divide by 0x2000")
    .to_string();
    assert!(e.contains("does not divide"), "{e}");

    let e = Cfi::new(
        &Props::new()
            .with("size", Value::Size(0x4000))
            .with("width", 3u64),
    )
    .expect_err("a three-byte bus")
    .to_string();
    assert!(e.contains("1, 2, 4 or 8"), "{e}");

    // Four parts on a four-byte bus would be x8 each, which is legal; five
    // would not divide it at all.
    assert!(
        Geometry::uniform(0x4000, 0x1000, 4, 4).is_ok(),
        "four x8 parts"
    );
    assert!(Geometry::uniform(0x4000, 0x1000, 4, 5).is_err());

    // A typo'd property is an afternoon lost if it is silently ignored.
    let props = Props::new()
        .with("size", Value::Size(0x4000))
        .with("blcok", Value::Size(0x1000));
    assert!(Cfi::new(&props).is_err(), "unknown property");
}

#[test]
fn an_image_is_copied_in_and_the_rest_stays_erased() {
    let image = crate::core::props::Media::new("varstore", &b"\x01\x02\x03\x04"[..]);
    let cfi = Cfi::new(
        &Props::new()
            .with("size", Value::Size(0x2000))
            .with("block", Value::Size(0x1000))
            .with("image", image),
    )
    .expect("a flash with an image");
    assert_eq!(contents(&cfi, 0, 4), alloc::vec![1, 2, 3, 4]);
    assert_eq!(contents(&cfi, 4, 4), alloc::vec![0xff; 4]);
    // And a reset does not put the image back, because a reset does not put
    // the contents back at all.
    poke(&cfi, 0, 0x0000_0000);
    read_array(&cfi);
    cfi.reset(ResetKind::Cold);
    assert_eq!(contents(&cfi, 0, 4), alloc::vec![1, 2, 3, 4]);
}

#[test]
fn a_blocks_list_describes_a_part_whose_blocks_differ() {
    let cfi = Cfi::new(
        &Props::new()
            .with("size", Value::Size(4 * 0x1000 + 3 * 0x4000))
            .with(
                "blocks",
                Value::List(alloc::vec![
                    Value::Uint(4),
                    Value::Size(0x1000),
                    Value::Uint(3),
                    Value::Size(0x4000),
                ]),
            )
            .with("locked", false),
    )
    .expect("a boot-block part");
    assert_eq!(cfi.array().geometry().block_count(), 7);
    // Erasing in the small region takes 4 KiB with it; in the large one, 16.
    word_program(&cfi, 0x0000, 0);
    word_program(&cfi, 0x1000, 0);
    erase_block(&cfi, 0x0000);
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0x0000), 0xffff_ffff);
    assert_eq!(peek(&cfi, 0x1000), 0);

    word_program(&cfi, 0x4000, 0);
    word_program(&cfi, 0x7ffc, 0);
    word_program(&cfi, 0x8000, 0);
    erase_block(&cfi, 0x4000);
    read_array(&cfi);
    assert_eq!(peek(&cfi, 0x7ffc), 0xffff_ffff, "all 16 KiB of it");
    assert_eq!(peek(&cfi, 0x8000), 0, "and not the next one");
}

#[test]
fn a_write_that_is_not_a_whole_bus_word_is_refused() {
    let cfi = flash(2, 0x1000);
    // Half of a 16-bit part's word is not a bus cycle it can decode.
    assert!(
        cfi.array().write(1, &[0xff], MemAttrs::DEFAULT).is_err(),
        "misaligned"
    );
    assert!(
        cfi.array().write(0, &[], MemAttrs::DEFAULT).is_err(),
        "nothing at all"
    );
    // Past the end is a bus fault, not a wrap.
    assert!(
        cfi.array()
            .write(0x1ffe, &[0u8; 4], MemAttrs::DEFAULT)
            .is_err()
    );
    let mut bytes = [0u8; 4];
    assert!(
        cfi.array()
            .read(0x1ffe, &mut bytes, MemAttrs::DEFAULT)
            .is_err()
    );
}

#[test]
fn the_class_publishes_one_window_and_a_schema_that_matches_it() {
    let cfi = flash(2, 0x1000);
    assert!(cfi.region("").is_some());
    assert!(cfi.region("flash").is_some());
    assert!(cfi.region("bank0").is_none());
    assert_eq!(cfi.region("").expect("mapped").len(), 0x2000);

    let schema = schema();
    for prop in CLASS.properties {
        assert!(
            schema.props.iter().any(|p| p.name == prop.name),
            "the validator does not know `{}`",
            prop.name
        );
    }
}
