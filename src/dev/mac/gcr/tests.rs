//! The codec is its own oracle: everything here writes a track the way a
//! formatter would and reads it back the way a head and an IWM would, with
//! nothing in between that both halves could agree about wrongly.

use super::*;

/// The four rules leave exactly sixty-four patterns, and they leave out the
/// two bytes a track is searched for.
#[test]
fn the_rules_yield_exactly_sixty_four_disk_bytes() {
    let count = (0..=255u8).filter(|&c| is_disk_byte(c)).count();
    assert_eq!(count, 64, "the rules are not the right rules");
    assert_eq!(DISK_BYTES.len(), 64);
    assert_eq!(DISK_BYTES[0], 0x96, "the lowest is $96");
    assert_eq!(DISK_BYTES[63], 0xff, "and the highest is $FF");
    // Strictly ascending, so a payload's code and a code's payload agree.
    assert!(DISK_BYTES.windows(2).all(|w| w[0] < w[1]));
    for (i, &code) in DISK_BYTES.iter().enumerate() {
        assert_eq!(payload_of(code), Some(i as u8));
    }

    // The two marks are not disk bytes, which is why they can be marks: a
    // payload byte can never be mistaken for the start of a field.
    assert!(!is_disk_byte(0xd5), "$D5 is a mark, not a payload");
    assert!(!is_disk_byte(0xaa), "$AA is a mark, not a payload");
    assert_eq!(payload_of(0xd5), None);
    assert_eq!(payload_of(0xaa), None);
    // $96 and $AD *are* disk bytes and are still the third byte of a mark:
    // a mark is three bytes because its first two cannot occur, not its third.
    assert!(is_disk_byte(0x96) && is_disk_byte(0xad));

    // And the Guide's own rule holds of every one of them.
    for &code in &DISK_BYTES {
        assert_eq!(code & 0x80, 0x80, "{code:02x} has its top bit clear");
        let mut zeros = 0;
        for i in (0..8).rev() {
            if code & (1 << i) == 0 {
                zeros += 1;
                assert!(zeros <= 2, "{code:02x} has three zeros in a row");
            } else {
                zeros = 0;
            }
        }
    }
}

/// Three bytes to four six-bit values, and back.
#[test]
fn nibblization_round_trips_whatever_its_length() {
    for len in [0usize, 1, 2, 3, 4, 5, 6, 523, 524, 525] {
        let bytes: Vec<u8> = (0..len).map(|i| (i * 7 + 3) as u8).collect();
        let nibbles = nibblize(&bytes);
        assert!(
            nibbles.iter().all(|&n| n < 64),
            "a value wider than six bits at length {len}"
        );
        assert_eq!(denibblize(&nibbles), bytes, "length {len}");
    }
    // The number a whole sector becomes is the one the format names.
    assert_eq!(nibblize(&[0u8; SECTOR_BYTES]).len(), SECTOR_NIBBLES);
}

/// The patent's checksum scrambles and unscrambles, and a single bad byte is
/// caught — which is the point of it.
#[test]
fn the_checksum_scrambles_and_catches_a_changed_byte() {
    let sector: Vec<u8> = (0..SECTOR_BYTES).map(|i| (i * 31 + 17) as u8).collect();
    let (scrambled, sum) = Checksum::scramble(&sector);
    assert_eq!(scrambled.len(), SECTOR_BYTES);
    assert_ne!(scrambled, sector, "a scramble that changes nothing is none");

    let (back, check) = Checksum::unscramble(&scrambled);
    assert_eq!(back, sector);
    assert_eq!(check, sum);

    // Every single-byte change is caught.
    for at in [0usize, 1, 2, 3, 261, 522, 523] {
        let mut damaged = scrambled.clone();
        damaged[at] ^= 0x01;
        let (_, check) = Checksum::unscramble(&damaged);
        assert_ne!(check, sum, "a change at {at} went unnoticed");
    }

    // A sector of zeros still checksums to something, because C3 rotates
    // whatever is in it — and for an all-zero sector that is nothing, which is
    // the one case where the sums stay zero.
    let (_, zero) = Checksum::scramble(&[0u8; SECTOR_BYTES]);
    assert_eq!(zero.sums, [0, 0, 0]);
}

/// The checksum's four six-bit values carry all twenty-four bits.
#[test]
fn the_checksum_round_trips_through_its_four_nibbles() {
    for sums in [
        [0, 0, 0],
        [0xff, 0xff, 0xff],
        [0x12, 0x34, 0x56],
        [0xc0, 0x3f, 0x80],
    ] {
        let sum = Checksum { sums };
        let n = sum.nibbles();
        assert!(n.iter().all(|&v| v < 64));
        assert_eq!(Checksum::from_nibbles(n), sum, "{sums:02x?}");
    }
}

/// The five speed zones are the ones that add up to a Macintosh disk.
#[test]
fn the_five_speed_zones_make_four_hundred_kilobytes_a_side() {
    assert_eq!(sectors_on(0), 12);
    assert_eq!(sectors_on(15), 12);
    assert_eq!(sectors_on(16), 11);
    assert_eq!(sectors_on(47), 10);
    assert_eq!(sectors_on(63), 9);
    assert_eq!(sectors_on(64), 8);
    assert_eq!(sectors_on(MAX_TRACK), 8);
    // 800 sectors a side of 512 bytes is 400 KiB, which is the disk's name.
    assert_eq!(sectors_per_side(), 800);
    assert_eq!(sectors_per_side() * DATA_BYTES, 400 * 1024);
}

/// A self-sync run is ten bit cells a byte, not eight.
#[test]
fn a_self_sync_byte_is_ten_bits() {
    let mut track = Track::new();
    track.push_sync(36);
    assert_eq!(
        track.len(),
        360,
        "36 sync bytes are 45 octets of bit stream"
    );
    // And a shifter comes out of it aligned, whatever it was doing going in:
    // every ten bits it throws two away.
    assert_eq!(shift_bytes(&track, 1), vec![0xff; 36]);
}

/// One sector written and read back: the whole point.
#[test]
fn a_sector_goes_onto_a_track_and_comes_off_it() {
    let tag: Vec<u8> = (0..TAG_BYTES).map(|i| 0xa0 + i as u8).collect();
    let data: Vec<u8> = (0..DATA_BYTES).map(|i| (i * 13 + 5) as u8).collect();
    let sector = Sector::new(17, true, 5, FORMAT_800K, &tag, &data);
    let track = encode_track(core::slice::from_ref(&sector));

    let (read, bad) = decode_track(&track);
    assert!(bad.is_empty(), "the track did not read: {bad:?}");
    assert_eq!(read.len(), 1);
    let got = &read[0];
    assert_eq!(got.track, 17);
    assert!(got.side);
    assert_eq!(got.sector, 5);
    assert_eq!(got.format, FORMAT_800K);
    assert_eq!(got.tag(), &tag[..]);
    assert_eq!(got.data(), &data[..]);
    assert_eq!(got, &sector);
}

/// A whole cylinder of them, on both sides and in every speed zone.
#[test]
fn every_cylinder_of_a_disk_writes_and_reads() {
    for track_no in [0u8, 15, 16, 32, 48, 63, 64, MAX_TRACK] {
        for side in [false, true] {
            let n = sectors_on(track_no);
            let sectors: Vec<Sector> = (0..n)
                .map(|s| {
                    let data: Vec<u8> = (0..DATA_BYTES)
                        .map(|i| (i as u8) ^ s ^ track_no ^ u8::from(side))
                        .collect();
                    Sector::new(track_no, side, s, FORMAT_800K, &[s; TAG_BYTES], &data)
                })
                .collect();
            let track = encode_track(&sectors);
            let (read, bad) = decode_track(&track);
            assert!(bad.is_empty(), "track {track_no} side {side}: {bad:?}");
            assert_eq!(
                read.len(),
                usize::from(n),
                "track {track_no} side {side}: {} sectors read of {n}",
                read.len()
            );
            for (want, got) in sectors.iter().zip(&read) {
                assert_eq!(want, got, "track {track_no} side {side}");
            }
        }
    }
}

/// A cylinder above 63 needs the seventh track bit, which lives in the same
/// byte as the side — the one field of the address that is two things.
#[test]
fn the_seventh_track_bit_rides_with_the_side() {
    for track_no in [0u8, 63, 64, 79] {
        for side in [false, true] {
            let sector = Sector::new(track_no, side, 0, FORMAT_800K, &[], &[]);
            let track = encode_track(core::slice::from_ref(&sector));
            let (read, bad) = decode_track(&track);
            assert!(bad.is_empty());
            assert_eq!(read[0].track, track_no, "side {side}");
            assert_eq!(read[0].side, side, "track {track_no}");
        }
    }
}

/// A bit flipped in the data is caught by the checksum rather than handed back.
#[test]
fn a_damaged_sector_is_refused_rather_than_returned() {
    let sector = Sector::new(
        3,
        false,
        2,
        FORMAT_800K,
        &[1; TAG_BYTES],
        &[0x5a; DATA_BYTES],
    );
    let good = encode_track(core::slice::from_ref(&sector));

    // Flip one bit well inside the data field. The address field is 8 bytes
    // after a 45-octet sync run, so bit 3000 is past both of them.
    let mut damaged = Track::new();
    for n in 0..good.len() {
        damaged.push_bit(good.bit(n) ^ (n == 3_000));
    }
    let (read, bad) = decode_track(&damaged);
    assert!(read.is_empty(), "a damaged sector was handed back");
    assert!(!bad.is_empty(), "and nothing was reported");
}

/// **A cylinder is one [`SECTOR_CELLS`] slot per sector**, gaps included, and
/// that is what decides how fast the disk turns.
///
/// The sectors themselves are shorter than their slots; the difference is the
/// gap a formatter leaves, and leaving it out used to make every cylinder
/// 2.5 % short and the spindle 2.5 % fast — far enough outside the window a
/// Macintosh Plus ROM accepts that it would not read the disk at all. See
/// [`SECTOR_CELLS`] for the measurement, and `docs/platforms/mac-plus.md` for
/// how it was made.
#[test]
fn a_cylinder_is_one_slot_per_sector_and_turns_at_the_speed_that_implies() {
    // The IWM's own cell time in fast mode, which is what the board clocks
    // this at: two microseconds a cell.
    const CELLS_PER_SECOND: usize = 500_000;

    let sector = Sector::new(
        0,
        false,
        0,
        FORMAT_800K,
        &[0; TAG_BYTES],
        &[0x5a; DATA_BYTES],
    );
    let bare = {
        let mut t = Track::new();
        t.push_sector(&sector);
        t.len()
    };
    assert!(
        bare < SECTOR_CELLS,
        "a sector must fit in its slot with room for a gap: {bare} of {SECTOR_CELLS}"
    );

    for (zone, &sectors) in ZONE_SECTORS.iter().enumerate() {
        let track = (zone as u8) * ZONE_TRACKS;
        assert_eq!(sectors_on(track), sectors);
        let sectors_here: Vec<Sector> = (0..sectors)
            .map(|s| {
                Sector::new(
                    track,
                    false,
                    s,
                    FORMAT_800K,
                    &[0; TAG_BYTES],
                    &[0x5a; DATA_BYTES],
                )
            })
            .collect();
        let cells = encode_track(&sectors_here).len();
        assert_eq!(
            cells,
            usize::from(sectors) * SECTOR_CELLS,
            "cylinder {track} is not a whole number of sector slots"
        );
        // And the speed that implies, to a tenth of a revolution a minute.
        let rpm_tenths = (CELLS_PER_SECOND * 60 * 10 + cells / 2) / cells;
        let want = [3940, 4298, 4728, 5253, 5910][zone];
        assert_eq!(
            rpm_tenths,
            want,
            "cylinder {track} turns at {} rpm, not {}",
            rpm_tenths as f64 / 10.0,
            want as f64 / 10.0
        );
    }
}

/// The gap is written rather than erased, bar the cells that will not take
/// another self-sync byte.
#[test]
fn padding_lays_down_self_sync_and_then_bare_medium() {
    let mut track = Track::new();
    track.push_byte(0xd5);
    track.pad_to(8 + 25);
    assert_eq!(track.len(), 8 + 25);
    // Two whole self-sync bytes (ten cells each) and then five bare cells.
    for n in 8..8 + 8 {
        assert!(track.bit(n), "cell {n} should be a one");
    }
    for n in 8 + 8..8 + 10 {
        assert!(!track.bit(n), "cell {n} should be a zero");
    }
    for n in 8 + 20..8 + 25 {
        assert!(!track.bit(n), "cell {n} is the splice and carries nothing");
    }
    // A track already long enough is left exactly as it was.
    let before = track.clone();
    track.pad_to(4);
    assert_eq!(track, before);
}

/// A track with nothing on it reads as nothing rather than as noise.
#[test]
fn an_unformatted_cylinder_reads_as_empty() {
    let track = Track::new();
    assert!(track.is_empty());
    assert_eq!(track.len(), 0);
    assert!(!track.bit(0));
    let (read, bad) = decode_track(&track);
    assert!(read.is_empty() && bad.is_empty());
}
