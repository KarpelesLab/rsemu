//! The MFM format on its own: the two marks against the hand arithmetic, the
//! CRC against its own definition, and a whole track written and read back.

use super::*;
use alloc::vec;

/// The two address marks are what the module docs derive, and that is the
/// point: they are arithmetic from the encoding rule, not a table copied from
/// anywhere.
#[test]
fn the_two_marks_are_what_the_rule_produces() {
    // `$A1 = 1010_0001` encoded normally, clock where two zeros meet.
    assert_eq!(cells(0xa1, false, None), 0x44a9);
    // And with the clock pulse between data bits 4 and 5 left out.
    assert_eq!(sync_cells(SYNC_A1), 0x4489);

    assert_eq!(cells(0xc2, false, None), 0x52a4);
    assert_eq!(sync_cells(SYNC_C2), 0x5224);

    // Any other byte is ordinary, because only those two are marks.
    assert_eq!(sync_cells(0x4e), cells(0x4e, false, None));
}

/// The encoding rule itself: a clock cell carries a pulse only where two zeros
/// meet, and never anywhere else.
#[test]
fn a_clock_pulse_goes_only_between_two_zeros() {
    // All ones: every data cell set, no clock cell ever.
    assert_eq!(cells(0xff, true, None), 0b01_01_01_01_01_01_01_01);
    // All zeros after a one: the first clock is suppressed by the preceding
    // one, the other seven are not.
    assert_eq!(cells(0x00, true, None), 0b00_10_10_10_10_10_10_10);
    // All zeros after a zero: every clock.
    assert_eq!(cells(0x00, false, None), 0b10_10_10_10_10_10_10_10);
}

/// CRC-16/CCITT, checked against the one value everybody's implementation
/// agrees on: `"123456789"` seeded with `$FFFF` is `$29B1`.
#[test]
fn the_crc_is_ccitt() {
    assert_eq!(crc16(0xffff, b"123456789"), 0x29b1);
    // And it is continuable, which is how a data field's CRC covers the three
    // sync bytes and the mark before the data.
    let seed = crc16(0xffff, b"1234");
    assert_eq!(crc16(seed, b"56789"), 0x29b1);
}

/// A track's geometry: eighteen sectors and their gaps fit inside one
/// revolution with the trailing gap to spare.
#[test]
fn a_track_is_one_revolution_long() {
    let data: Vec<u8> = (0..DATA_BYTES).map(|i| i as u8).collect();
    let sectors: Vec<Sector> = (1..=SECTORS as u8)
        .map(|s| Sector::new(0, 0, s, &data))
        .collect();
    let track = encode_track(&sectors);
    assert_eq!(
        track.len(),
        CELLS_PER_REVOLUTION,
        "a track is exactly one revolution of cells"
    );
    // 200,000 cells is 12,500 bytes of MFM; the eighteen sectors and their
    // gaps need 11,990 of them.
    let header = GAP4A + SYNC_BYTES + 3 + 1 + GAP1;
    let per_sector = SYNC_BYTES + 3 + 1 + 4 + 2 + GAP2 + SYNC_BYTES + 3 + 1 + DATA_BYTES + 2 + GAP3;
    let needed = header + SECTORS * per_sector;
    assert_eq!(needed, 11_990);
    assert!(
        needed * 16 < CELLS_PER_REVOLUTION,
        "and they fit: {needed} bytes in {}",
        CELLS_PER_REVOLUTION / 16
    );
}

/// **Every sector of a track survives the journey to cells and back**, ID
/// fields, CRCs and all.
#[test]
fn every_sector_of_a_track_writes_and_reads() {
    for (cyl, head) in [(0u8, 0u8), (17, 1), (79, 0), (79, 1)] {
        let sectors: Vec<Sector> = (1..=SECTORS as u8)
            .map(|s| {
                let data: Vec<u8> = (0..DATA_BYTES)
                    .map(|i| (i as u8).wrapping_mul(s).wrapping_add(cyl))
                    .collect();
                Sector::new(cyl, head, s, &data)
            })
            .collect();
        let track = encode_track(&sectors);
        let (found, bad) = decode_track(&track);
        assert!(bad.is_empty(), "cylinder {cyl} head {head}: {bad:?}");
        assert_eq!(
            found.len(),
            SECTORS,
            "cylinder {cyl} head {head}: found {} of {SECTORS}",
            found.len()
        );
        for want in &sectors {
            let got = found
                .iter()
                .find(|s| s.sector == want.sector)
                .unwrap_or_else(|| panic!("sector {} is missing", want.sector));
            assert_eq!(got, want, "sector {} came back changed", want.sector);
        }
    }
}

/// A single bit flipped in a data field is caught by the CRC rather than
/// handed over as data.
#[test]
fn a_flipped_bit_fails_the_crc() {
    let data = vec![0x5au8; DATA_BYTES];
    let track = encode_track(&[Sector::new(3, 1, 7, &data)]);
    let (found, bad) = decode_track(&track);
    assert_eq!(found.len(), 1);
    assert!(bad.is_empty());

    // Where the data field's first byte starts, in cells, worked out from the
    // constants rather than guessed: the track header, then everything of the
    // sector up to and including its data address mark.
    let header = GAP4A + SYNC_BYTES + 3 + 1 + GAP1;
    let pre_data = SYNC_BYTES + 3 + 1 + 4 + 2 + GAP2 + SYNC_BYTES + 3 + 1;
    let first_data_cell = (header + pre_data) * 16;
    for byte in [0usize, 9, 100, 511] {
        // The data cell of that byte's most significant bit: the odd cell of
        // the pair.
        let at = first_data_cell + byte * 16 + 1;
        let mut broken = Track::new();
        for n in 0..track.len() {
            broken.push_bit(if n == at { !track.bit(n) } else { track.bit(n) });
        }
        let (found, bad) = decode_track(&broken);
        assert!(
            found.is_empty(),
            "a flipped cell in data byte {byte} was handed over as good data"
        );
        assert!(!bad.is_empty(), "and it was not even reported: byte {byte}");
    }
}

/// Where a block sits, and back again, for every one of the 2,880.
#[test]
fn every_block_has_one_place_and_gets_back_from_it() {
    assert_eq!(BYTES, 1_474_560);
    assert_eq!(CYLINDERS * SIDES * SECTORS, 2_880);
    for block in 0..CYLINDERS * SIDES * SECTORS {
        let (c, h, s) = place(block).expect("a block inside the disk");
        assert!((1..=SECTORS as u8).contains(&s), "sectors are one-based");
        assert_eq!(block_of(c, h, s), Some(block), "block {block}");
    }
    assert_eq!(place(CYLINDERS * SIDES * SECTORS), None);
    assert_eq!(block_of(0, 0, 0), None, "sector 0 is not a sector");
    assert_eq!(block_of(0, 0, 19), None);
    assert_eq!(block_of(80, 0, 1), None);
}

/// An empty track decodes to nothing rather than panicking.
#[test]
fn an_unformatted_track_has_no_sectors() {
    let (found, bad) = decode_track(&Track::new());
    assert!(found.is_empty() && bad.is_empty());
    assert_eq!(read_byte(&Track::new(), 0), None);
}
