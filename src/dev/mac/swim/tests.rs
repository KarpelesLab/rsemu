//! The SWIM on its own: that it is the IWM while it is in IWM mode, that its
//! SuperDrive says so, and that the two cell rates come out at the two spindle
//! speeds.

use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use super::*;
use crate::core::device::Device;
use crate::core::props::Value;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::dev::mac::{gcr, mfm};

/// The soft switches are the IWM's, at the IWM's stride, and the mode register
/// is loaded the IWM's way — which is the whole shape of the older part and
/// the reason this device forwards rather than reimplements.
#[test]
fn the_register_file_is_the_iwms() {
    let swim = Swim::with_drives([true, false]);
    assert_eq!(REGISTER_SPAN, iwm::REGISTER_SPAN);
    // The mode register is loaded by a write while `Q7:Q6` is `11`.
    let _ = swim.peek(13); // Q6 on
    swim.poke(15, 0x17); // Q7 on, and the write lands in the mode register
    assert_eq!(swim.iwm().mode(), 0x17, "the write reached the IWM inside");
    // With `Q7` off again the status register mirrors the mode's low bits.
    let status = swim.peek(14);
    assert_eq!(status & 0x1f, 0x17, "{status:#04x}");
}

/// **The mechanism says it is a SuperDrive at `SEL` high, and only there.**
///
/// `CA2:CA1:CA0 = 101` is two drive registers, not one — address 10 with `SEL`
/// low and address 11 with it high — and Apple's IIGS note leaves both
/// unassigned on the 800K mechanism. Which of them a SuperDrive answers is
/// measured rather than read, and the measurement is a real Macintosh Classic
/// ROM's own behaviour: answering **both** makes it refuse to touch the
/// mechanism at all, and it sits on the insert-disk screen with the motor
/// never turning; answering only address 11 sends it into ISM mode to read the
/// 1.44 MB disk. `src/dev/mac/iwm.rs` has the table of all three outcomes.
///
/// So this asserts both halves: asserted at 11, and the pull-up at 10.
#[test]
fn the_drive_reports_itself_as_a_superdrive_at_sel_high_only() {
    // Set CA0 and CA2, clear CA1, then read the status register's `SENSE`.
    let sense = |read: &dyn Fn(u8) -> u8| -> bool {
        let _ = read(1); // CA0 on
        let _ = read(2); // CA1 off
        let _ = read(5); // CA2 on
        let _ = read(13); // Q6 on: the status register
        read(14) & 0x80 != 0
    };
    let swim = Swim::with_drives([true, false]);
    swim.set_sel(true);
    assert!(
        !sense(&|i| swim.peek(i)),
        "a SuperDrive pulls address 11 low, like every other line on the cable"
    );
    swim.set_sel(false);
    assert!(
        sense(&|i| swim.peek(i)),
        "and leaves address 10 to the pull-up, which is what the ROM requires"
    );

    let plus = Iwm::with_drives([true, false]);
    for sel in [false, true] {
        plus.set_sel(sel);
        assert!(
            sense(&|i| plus.peek(i)),
            "an 800K mechanism drives neither, which is what a Plus sees"
        );
    }
}

/// A 1.44 MB image goes in, and a GCR one still does.
#[test]
fn both_densities_go_into_the_drive() {
    let swim = Swim::with_drives([true, false]);
    assert!(!swim.has_disk(0));
    swim.insert(0, Disk::blank_mfm());
    assert_eq!(swim.density(0), Some(Density::Mfm));
    swim.insert(0, Disk::blank(2));
    assert_eq!(swim.density(0), Some(Density::Gcr));
    swim.eject(0);
    assert_eq!(swim.density(0), None);
}

/// The two cell rates, and the two spindle speeds they come out at.
///
/// One tick of this chip is one MFM cell. A GCR disk's cells are half that
/// rate, so it gets every second tick, and the arithmetic below is the whole
/// of why the two media turn at the speeds they do.
#[test]
fn a_gcr_disk_gets_every_second_tick_and_an_mfm_disk_every_one() {
    let swim = Swim::with_drives([true, false]);
    swim.insert(0, Disk::blank(2));
    swim.advance_to(1000);
    assert_eq!(swim.ticks(), 1000);
    assert_eq!(swim.iwm().ticks(), 500, "a GCR cell is two of this chip's");

    let swim = Swim::with_drives([true, false]);
    swim.insert(0, Disk::blank_mfm());
    swim.advance_to(1000);
    assert_eq!(swim.iwm().ticks(), 1000, "an MFM cell is one");

    // And the speeds that follow, with nothing else to get wrong: a cylinder
    // is its track's length and the spindle turns once per pass.
    let gcr_rpm = 500_000 * 60 / (gcr::SECTOR_CELLS * 12);
    assert_eq!(gcr_rpm, 394, "zone 0 of an 800K disk");
    let mfm_rpm = 1_000_000 * 60 / mfm::CELLS_PER_REVOLUTION;
    assert_eq!(mfm_rpm, usize::try_from(mfm::RPM).unwrap());
}

/// A 1.44 MB disk turns under the head and its cells go past it.
#[test]
fn an_mfm_disk_turns_under_the_head() {
    let swim = Swim::with_drives([true, false]);
    let mut image = alloc::vec![0u8; mfm::BYTES];
    for (block, chunk) in image.chunks_mut(512).enumerate() {
        chunk[..4].copy_from_slice(&(block as u32).to_be_bytes());
    }
    swim.insert(
        0,
        Disk::from_image_for(&image, Reader::Swim).expect("a SWIM reads one"),
    );
    // Motor on: the drive's control register `CA1:CA0:SEL = 100` with `CA2`
    // low, latched when `LSTRB` goes high.
    swim.set_sel(false);
    let _ = swim.peek(0); // CA0 off
    let _ = swim.peek(3); // CA1 on
    let _ = swim.peek(4); // CA2 off — motor on
    let _ = swim.peek(7); // LSTRB on: latch it
    let _ = swim.peek(6); // LSTRB off
    assert!(swim.motor(0), "the motor is running");

    // One revolution of cells. The chip's shifter is the IWM's GCR one, so
    // what it latches out of MFM is not sectors — see the module docs — but
    // the medium does move, which is what this asserts.
    let one = u64::try_from(mfm::CELLS_PER_REVOLUTION).unwrap();
    swim.advance_to(one);
    assert_eq!(swim.iwm().ticks(), one);
}

/// Save, load, and the image is identical.
#[test]
fn a_snapshot_round_trip_is_identical() {
    let a = Swim::with_drives([true, false]);
    a.insert(0, Disk::blank_mfm());
    let _ = a.peek(13);
    a.poke(15, 0x1f);
    a.advance_to(12_345);

    let image = |swim: &Swim| -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("swim", CLASS_NAME).expect("a fresh shape");
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("swim", CLASS_NAME, STATE_VERSION).expect("a chunk");
            Device::save(swim, &mut chunk).expect("a SWIM saves");
        }
        w.to_vec().expect("a complete image")
    };
    let first = image(&a);

    let b = Swim::with_drives([true, false]);
    b.insert(0, Disk::blank_mfm());
    let reader = StateReader::new(&first).expect("a well-formed image");
    let chunk = reader
        .load("swim", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    Device::load(&b, &mut chunk.reader()).expect("a SWIM loads");
    assert_eq!(image(&b), first, "the same chip, bit for bit");
    assert_eq!(b.ticks(), 12_345);
}

/// The class registers, its schema names the pin and the regions a machine
/// file may write, and `drives` is checked.
#[test]
fn the_class_is_registrable_and_its_schema_matches() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a fresh registry");
    let s = schema();
    assert_eq!(s.class, CLASS_NAME);
    let ports: Vec<&str> = s.ports.iter().map(|p| p.name.as_str()).collect();
    assert_eq!(ports, alloc::vec![SEL_PIN]);
    assert!(s.regions.contains(&String::from("regs")));

    let err = Swim::new(&Props::new().with("drives", Value::Uint(3)))
        .expect_err("three mechanisms")
        .to_string();
    assert!(err.contains("one or two"), "{err}");
}

/// And the region it hands the address space is the one a `map` places.
#[test]
fn the_region_is_the_span_the_board_maps() {
    let swim = Arc::new(Swim::with_drives([true, false]));
    let region = Device::region(&*swim, "").expect("a register file");
    assert_eq!(region.len(), REGISTER_SPAN);
    assert!(Device::region(&*swim, "nope").is_none());
}

/// The cached density is what the tick path reads, and it follows the medium
/// through every door the medium can change by.
///
/// It is derived state and is never serialized: `advance_to` runs once a
/// scheduler round — fifty thousand times a virtual second while a disk is
/// turning — and asking the drive would mean cloning a megabyte and a half of
/// disk every round.
#[test]
fn the_cached_density_follows_the_medium() {
    let swim = Swim::with_drives([true, false]);
    assert!(!swim.is_mfm(), "an empty drive is not MFM");
    swim.insert(0, Disk::blank_mfm());
    assert!(swim.is_mfm());
    swim.insert(0, Disk::blank(2));
    assert!(!swim.is_mfm(), "a GCR disk went in over it");
    swim.insert(0, Disk::blank_mfm());
    swim.eject(0);
    assert!(!swim.is_mfm(), "and the drive is empty again");

    // A reset leaves the disk in the drive, so the cache has to come back with
    // it rather than be assumed.
    swim.insert(0, Disk::blank_mfm());
    Device::reset(&swim, crate::core::device::ResetKind::Warm);
    assert!(
        swim.has_disk(0),
        "a disk is a thing in a slot, not a register"
    );
    assert!(swim.is_mfm(), "and the cache came back with it");
}

/// **Which of the two sync patterns a separator can actually find**, measured
/// over a whole track rather than argued.
///
/// The ISM's mark search runs cell by cell with no byte boundary to align to —
/// page 23: "The search for the mark byte is invisible to the software since it
/// is handled entirely by the SWIM chip" — so it can only look for a *bit
/// pattern*, and a pattern that also occurs at an odd alignment somewhere in
/// ordinary data is a false mark.
///
/// This walks every cell of a formatted track and counts where each of the two
/// patterns turns up. The result is the reason `Framer` syncs on `$4489` and
/// not on `$5224`, and it is printed so the numbers can be read rather than
/// taken on trust.
#[test]
fn only_the_a1_sync_is_unique_at_every_cell_alignment() {
    // Eighteen sectors of data that is not all one byte: a run of $00 or $FF
    // would not exercise the question at all.
    let sectors: Vec<mfm::Sector> = (1..=18u8)
        .map(|s| {
            let data: Vec<u8> = (0..512u32)
                .map(|i| (i.wrapping_mul(2_654_435_761) >> 13) as u8 ^ s)
                .collect();
            mfm::Sector::new(17, 1, s, &data)
        })
        .collect();
    let track = mfm::encode_track(&sectors);
    let a1 = mfm::sync_cells(mfm::SYNC_A1);
    let c2 = mfm::sync_cells(mfm::SYNC_C2);

    let (mut window, mut a1_hits, mut c2_hits) = (0u16, 0usize, 0usize);
    for cell in 0..track.len() {
        window = (window << 1) | u16::from(track.bit(cell));
        if cell < 15 {
            continue;
        }
        if window == a1 {
            a1_hits += 1;
        }
        if window == c2 {
            c2_hits += 1;
        }
    }
    // Three $A1s prefix each of the eighteen ID fields and each of the
    // eighteen data fields, and nothing else on the track carries the
    // pattern — at any alignment.
    assert_eq!(
        a1_hits,
        3 * 18 * 2,
        "the $A1 sync turns up somewhere other than where the format puts it: \
         {a1_hits} matches of $4489 against {c2_hits} of $5224 over one track"
    );
    // The index mark's $C2 is written three times, once a revolution — and
    // the pattern turns up more often than that, which is the measurement:
    // `$5224` is *not* unique at an arbitrary cell alignment, so a separator
    // that synced on it would report marks that are not there.
    assert!(
        c2_hits > 3,
        "if this ever comes out at 3 the separator could sync on $C2 safely: {c2_hits}"
    );
}

/// **The separator hands back a whole ID field and its CRC comes out zero.**
///
/// This is the end-to-end check of the MFM read path below the register file:
/// a 1.44 MB disk goes into the drive, the spindle turns, `ACTION` is set, and
/// the bytes that come out are `$A1 $A1 $A1 $FE C H R N` followed by the two
/// CRC bytes — with the chip's own generator reading **zero** once it has
/// absorbed them, which is what page 25's handshake bit 1 reports:
///
/// > The CRC error bit is cleared to zero if the CRC generated on the bytes up
/// > to and including the byte about to be read is zero (meaning all the bytes
/// > are correct).
///
/// A generator that came out non-zero was the defect that made Apple's ROM put
/// this disk back out again on every attempt: the separator was locking onto
/// the second or third `$A1` of the three, so the field it checksummed was not
/// the field the CRC was written over.
#[test]
fn the_separator_reads_an_id_field_and_its_crc_comes_out_zero() {
    let swim = Swim::with_drives([true, false]);
    let mut image = alloc::vec![0u8; mfm::BYTES];
    for (block, chunk) in image.chunks_mut(512).enumerate() {
        chunk[..4].copy_from_slice(&(block as u32).to_be_bytes());
    }
    swim.insert(
        0,
        Disk::from_image_for(&image, Reader::Swim).expect("a 1.44 MB image"),
    );
    let iwm = swim.iwm();
    // Drive 1, spindle on, head 0 — what the ISM's mode register would say.
    iwm.set_enables(Some(0), true, Some(false));
    iwm.set_mfm_framing(true, ism::CRC_SEED);

    // Walk a whole revolution a byte at a time, taking whatever the separator
    // frames, and stop once a mark has been followed by the ID address mark.
    let mut field: Vec<u8> = Vec::new();
    let mut tick = 0u64;
    for _ in 0..mfm::CELLS_PER_REVOLUTION {
        tick += 1;
        swim.advance_to(tick);
        while let Some((byte, mark)) = iwm.take_mfm() {
            if field.is_empty() && !mark {
                // Still in the gap: nothing has been found yet.
                continue;
            }
            field.push(byte);
        }
        // $A1 $A1 $A1 $FE and the four ID bytes, then the two CRC bytes.
        if field.len() >= 10 {
            break;
        }
    }
    assert_eq!(
        &field[..4],
        &[mfm::SYNC_A1, mfm::SYNC_A1, mfm::SYNC_A1, mfm::IDAM],
        "three sync bytes and the ID address mark, in that order: {field:02x?}"
    );
    assert_eq!(
        iwm.mfm_crc(),
        0,
        "the generator absorbed the field and its own CRC, so it should read \
         zero; the field was {field:02x?}"
    );
}

/// **A whole sector the ISM formats — ID field and all — is a sector the image
/// gets back.**
///
/// The layout is the one a Macintosh Classic really writes, taken off the wire
/// rather than out of a book: the trace of Mac OS 6.0.8 initializing a disk in
/// its external drive hands the write head, per sector,
///
/// ```text
///   101 x $4e   gap
///    12 x $00   sync
///     3 x $a1   through the *Mark* register
///     1 x $fe   the ID address mark
///       C H R N  the four ID bytes
///     1 write of the CRC register, which is two bytes
///    22 x $4e   gap 2
///    12 x $00   sync
///     3 x $a1   marks again
///     1 x $fb   the data address mark
///   512 x $f6   the format filler
///     1 write of the CRC register
/// ```
///
/// and nothing else. This is that, once, onto a blank; what it asserts is that
/// the sector comes back **out of the image**, which means the cells decoded
/// and both CRCs — which the *chip* supplied — held.
///
/// It exists because the real thing does not work yet: the Macintosh lays a
/// whole revolution down (194,625 cells of a 200,000-cell track), the chip
/// reports no error at all, and `Disk::absorb` recovers nothing.
/// `docs/platforms/mac-classic.md` has the measurement. A hermetic
/// reproduction is worth more to whoever picks this up than another ROM run.
#[test]
fn a_sector_the_chip_formats_comes_back_out_of_the_image() {
    let swim = Swim::with_drives([true, false]);
    swim.insert(0, Disk::blank_mfm());
    let iwm = swim.iwm();
    iwm.set_enables(Some(0), true, Some(false));

    let mut tick = 0u64;
    iwm.restart_write(ism::CRC_SEED);
    iwm.set_writing(true, true);
    let mut lay = |byte: u8, kind: iwm::WriteKind| {
        assert!(iwm.push_write(byte, kind), "the buffer had room");
        tick += 16;
        swim.advance_to(tick);
    };
    let (c, h, n) = (0u8, 0u8, mfm::SIZE_CODE);
    // A whole cylinder, which is what a format writes: eighteen sectors, each
    // an ID field and a data field, exactly as the trace has it.
    for r in 1..=mfm::SECTORS as u8 {
        for _ in 0..101 {
            lay(mfm::GAP_BYTE, iwm::WriteKind::Data);
        }
        for _ in 0..mfm::SYNC_BYTES {
            lay(0x00, iwm::WriteKind::Data);
        }
        for _ in 0..3 {
            lay(mfm::SYNC_A1, iwm::WriteKind::Mark);
        }
        for b in [mfm::IDAM, c, h, r, n] {
            lay(b, iwm::WriteKind::Data);
        }
        lay(0, iwm::WriteKind::CrcHigh);
        lay(0, iwm::WriteKind::CrcLow);
        for _ in 0..22 {
            lay(mfm::GAP_BYTE, iwm::WriteKind::Data);
        }
        for _ in 0..mfm::SYNC_BYTES {
            lay(0x00, iwm::WriteKind::Data);
        }
        for _ in 0..3 {
            lay(mfm::SYNC_A1, iwm::WriteKind::Mark);
        }
        lay(mfm::DAM, iwm::WriteKind::Data);
        for _ in 0..mfm::DATA_BYTES {
            lay(0xf6, iwm::WriteKind::Data);
        }
        lay(0, iwm::WriteKind::CrcHigh);
        lay(0, iwm::WriteKind::CrcLow);
    }
    // Every field the head laid down decodes, with no bad CRC anywhere: the
    // cells are checked before the image is, so a failure says which half.
    let cells = iwm.cylinder_cells();
    iwm.set_writing(false, true);
    let (found, bad) = mfm::decode_track(&cells);
    assert!(
        bad.is_empty(),
        "a field the head wrote did not check: {bad:?}"
    );
    assert_eq!(
        found.len(),
        mfm::SECTORS,
        "the head laid {} sectors down and {} decoded",
        mfm::SECTORS,
        found.len()
    );

    // And every one of them is in the image where the zone table puts it.
    let disk = iwm.disk(0).expect("a disk");
    let tally = iwm.write_tally();
    assert_eq!(tally.taken[0], mfm::SECTORS as u64, "{tally:?}");
    for r in 1..=mfm::SECTORS as u8 {
        let block = mfm::block_of(c, h, r).expect("a block for every sector");
        assert_eq!(
            disk.block(block).expect("the block"),
            &[0xf6u8; mfm::DATA_BYTES][..],
            "sector {r} did not come back out of the image"
        );
    }
}

/// **A data field the ISM writes is a data field the image gets back**, CRC and
/// all — and the CRC is the chip's own, not one the test computed.
///
/// This is the write path a Macintosh Classic really uses, driven the way Mac
/// OS 6.0.8 drives it: clear the FIFO with the read/write bit set to write,
/// set `ACTION`, then a sync field of `$00`s through the Data register, three
/// `$A1`s through the **Mark** register, the address mark, the 512 bytes, and
/// the two CRC bytes asked for through the **CRC** register rather than
/// computed by anybody. The sector then has to come off the medium through
/// `mfm::decode_track`, which checks the CRC it did not write.
///
/// That last part is what makes it worth the length: the generator's preset is
/// an inference (`iwm::Writer::crc`), and a field whose CRC was seeded a byte
/// early reads back bad. It did, before the preset moved to the first mark.
#[test]
fn a_field_the_chip_writes_reads_back_with_its_crc_clear() {
    let swim = Swim::with_drives([true, false]);
    swim.insert(0, Disk::blank_mfm());
    let iwm = swim.iwm();
    iwm.set_enables(Some(0), true, Some(false));

    // Read sector 1's ID field to the end of its CRC, which is exactly where a
    // driver about to write that sector's data leaves the head.
    iwm.set_mfm_framing(true, ism::CRC_SEED);
    let mut tick = 0u64;
    let mut field: Vec<u8> = Vec::new();
    while tick < mfm::CELLS_PER_REVOLUTION as u64 && field.len() < 10 {
        tick += 1;
        swim.advance_to(tick);
        while let Some((byte, mark)) = iwm.take_mfm() {
            if field.is_empty() && !mark {
                continue; // still in the gap
            }
            field.push(byte);
            if field.len() >= 10 {
                break;
            }
        }
    }
    assert_eq!(
        &field[..5],
        &[mfm::SYNC_A1, mfm::SYNC_A1, mfm::SYNC_A1, mfm::IDAM, 0],
        "three sync bytes, the ID address mark and cylinder 0: {field:02x?}"
    );
    let sector_number = field[6];
    iwm.set_mfm_framing(false, ism::CRC_SEED);

    // And write its data field where the format puts it: gap 2, the sync
    // field, three `$A1` marks, the data address mark, 512 bytes, and the two
    // CRC bytes the *chip* supplies.
    let data: Vec<u8> = (0..512u32).map(|i| (i * 7 + 3) as u8).collect();
    iwm.restart_write(ism::CRC_SEED);
    iwm.set_writing(true, true);
    let mut lay = |byte: u8, kind: iwm::WriteKind| {
        assert!(iwm.push_write(byte, kind), "the buffer had room");
        tick += 16; // one MFM byte is sixteen cells
        swim.advance_to(tick);
    };
    for _ in 0..mfm::GAP2 {
        lay(mfm::GAP_BYTE, iwm::WriteKind::Data);
    }
    for _ in 0..mfm::SYNC_BYTES {
        lay(0x00, iwm::WriteKind::Data);
    }
    for _ in 0..3 {
        lay(mfm::SYNC_A1, iwm::WriteKind::Mark);
    }
    lay(mfm::DAM, iwm::WriteKind::Data);
    for &byte in &data {
        lay(byte, iwm::WriteKind::Data);
    }
    lay(0, iwm::WriteKind::CrcHigh);
    lay(0, iwm::WriteKind::CrcLow);
    // Gap, so the splice at the end of the write does not land inside the
    // field that follows.
    for _ in 0..8 {
        lay(mfm::GAP_BYTE, iwm::WriteKind::Data);
    }
    iwm.set_writing(false, true);

    // And read it back **out of the image**, which is where a decoder that
    // checks the CRC has already had its say.
    let disk = iwm.disk(0).expect("a disk");
    let block = mfm::block_of(0, 0, sector_number).expect("sector 1 of cylinder 0 head 0");
    assert_eq!(
        disk.block(block).expect("the block"),
        &data[..],
        "the data field the chip wrote did not come back out of the image; a CRC seeded \
         anywhere but at the first mark byte is what that looks like"
    );
    // And nothing else on the cylinder moved.
    let others = (0..disk.blocks())
        .filter(|&n| n != block && disk.block(n) == Some(&data[..]))
        .count();
    assert_eq!(others, 0, "the write reached {others} other blocks");
}
