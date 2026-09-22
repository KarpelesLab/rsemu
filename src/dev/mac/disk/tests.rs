//! The container's own tests. No byte of anybody's disk image is here: the
//! images these build are made on the spot, and the one test that reads a real
//! file reads the user's own in place and skips when it is not there.

use super::*;
use alloc::vec;

/// Wrap `data` in a DiskCopy 4.2 header, with the checksums computed.
fn dc42(name: &str, disk_format: u8, format_byte: u8, data: &[u8], tags: &[u8]) -> Vec<u8> {
    let mut out = vec![0u8; DC42_HEADER];
    out[0] = name.len().min(63) as u8;
    out[1..1 + name.len()].copy_from_slice(name.as_bytes());
    out[0x40..0x44].copy_from_slice(&(data.len() as u32).to_be_bytes());
    out[0x44..0x48].copy_from_slice(&(tags.len() as u32).to_be_bytes());
    out[0x48..0x4c].copy_from_slice(&Dc42::checksum(data).to_be_bytes());
    out[0x4c..0x50].copy_from_slice(&Dc42::tag_section_checksum(tags).to_be_bytes());
    out[0x50] = disk_format;
    out[0x51] = format_byte;
    out[0x52..0x54].copy_from_slice(&DC42_MAGIC.to_be_bytes());
    out.extend_from_slice(data);
    out.extend_from_slice(tags);
    out
}

/// A raw 800K image with a recognisable pattern in it.
fn raw(bytes: usize) -> Vec<u8> {
    (0..bytes)
        .map(|i| (i / DATA_BYTES) as u8 ^ (i % 251) as u8)
        .collect()
}

/// The two sizes a raw image may be, and what each becomes.
#[test]
fn a_raw_image_is_four_hundred_or_eight_hundred_kilobytes() {
    let disk = Disk::from_image(&raw(BYTES_800K)).expect("800K");
    assert_eq!(disk.sides(), 2);
    assert_eq!(disk.blocks(), 1600);

    let disk = Disk::from_image(&raw(BYTES_400K)).expect("400K");
    assert_eq!(disk.sides(), 1);
    assert_eq!(disk.blocks(), 800);

    // And nothing else is.
    let e = alloc::format!("{}", Disk::from_image(&raw(1024)).unwrap_err());
    assert!(e.contains("409,600") && e.contains("819,200"), "{e}");
}

/// The refusal a 1.44 MB image gets says what it is and why the drive cannot
/// take it, rather than failing somewhere further down.
#[test]
fn a_1440k_image_is_refused_by_name() {
    // Raw.
    let e = alloc::format!("{}", Disk::from_image(&vec![0u8; BYTES_1440K]).unwrap_err());
    assert!(e.contains("1.44 MB"), "{e}");
    assert!(e.contains("IWM"), "{e}");
    assert!(e.contains("SWIM"), "{e}");
    assert!(e.contains("800K"), "it says what to give it instead: {e}");

    // And in a container, which is how the ones in circulation come.
    let image = dc42(
        "System Startup",
        DISK_FORMAT_1440K,
        0x22,
        &vec![0u8; BYTES_1440K],
        &[],
    );
    let e = alloc::format!("{}", Disk::from_image(&image).unwrap_err());
    assert!(e.contains("System Startup"), "it names the disk: {e}");
    assert!(e.contains("diskFormat 3"), "{e}");
    assert!(e.contains("SWIM"), "{e}");

    // 720K MFM is refused the same way: also not a drive a Plus has.
    let image = dc42("PC disk", DISK_FORMAT_720K, 0x22, &vec![0u8; 737_280], &[]);
    let e = alloc::format!("{}", Disk::from_image(&image).unwrap_err());
    assert!(e.contains("720K"), "{e}");
}

/// **A SWIM takes the same image**, raw or in a container, and what it gets is
/// an MFM disk.
///
/// The refusal above is a fact about a Macintosh Plus's hardware and stays
/// exactly where it is; `Reader::Swim` is a different controller asking.
#[test]
fn a_swim_takes_a_1440k_image_and_gets_an_mfm_disk() {
    let mut image = vec![0u8; BYTES_1440K];
    for (block, chunk) in image.chunks_mut(512).enumerate() {
        chunk[..4].copy_from_slice(&(block as u32).to_be_bytes());
    }
    for bytes in [image.clone(), dc42("System Startup", DISK_FORMAT_1440K, 0x22, &image, &[])] {
        let disk = Disk::from_image_for(&bytes, Reader::Swim).expect("a SWIM reads one");
        assert_eq!(disk.density(), Density::Mfm);
        assert_eq!(disk.sides(), 2);
        assert_eq!(disk.blocks(), 2_880, "eighteen sectors on 160 tracks");
        // The two boot blocks are where an HFS volume puts them.
        assert_eq!(disk.block(0).unwrap()[..4], 0u32.to_be_bytes());
        assert_eq!(disk.block(1).unwrap()[..4], 1u32.to_be_bytes());
    }

    // And a SWIM still refuses an image that is not one of the three sizes.
    let e = alloc::format!(
        "{}",
        Disk::from_image_for(&vec![0u8; 1000], Reader::Swim).unwrap_err()
    );
    assert!(e.contains("1000 bytes"), "{e}");
}

/// **Every block of a 1.44 MB image survives the journey to MFM cells and
/// back**, through the disk rather than through `mfm` on its own: the block
/// mapping and the encoder have to agree, and they are the two halves that
/// could disagree.
#[test]
fn every_block_of_a_1440k_image_survives_the_journey_to_cells() {
    let mut image = vec![0u8; BYTES_1440K];
    for (block, chunk) in image.chunks_mut(512).enumerate() {
        chunk[..4].copy_from_slice(&(block as u32).to_be_bytes());
        for (i, byte) in chunk.iter_mut().enumerate().skip(4) {
            *byte = (block as u8).wrapping_mul(31).wrapping_add(i as u8);
        }
    }
    let disk = Disk::from_image_for(&image, Reader::Swim).expect("a SWIM reads one");
    let mut seen = 0usize;
    for cylinder in [0u8, 1, 40, 79] {
        for head in [0u8, 1] {
            let track = disk.mfm_track(cylinder, head);
            assert_eq!(track.len(), super::mfm::CELLS_PER_REVOLUTION);
            let (found, bad) = super::mfm::decode_track(&track);
            assert!(bad.is_empty(), "cylinder {cylinder} head {head}: {bad:?}");
            assert_eq!(found.len(), super::mfm::SECTORS);
            for sector in &found {
                assert_eq!(sector.cylinder, cylinder);
                assert_eq!(sector.head, head);
                let block = super::mfm::block_of(cylinder, head, sector.sector)
                    .expect("a sector on the disk");
                assert_eq!(
                    sector.data.as_slice(),
                    disk.block(block).expect("a block"),
                    "cylinder {cylinder} head {head} sector {} is block {block}",
                    sector.sector
                );
                seen += 1;
            }
        }
    }
    assert_eq!(seen, 8 * super::mfm::SECTORS);
    // A cylinder past the end of the disk is unformatted rather than a panic.
    assert!(disk.mfm_track(80, 0).is_empty());
    assert!(disk.mfm_track(0, 2).is_empty());
    // And a GCR disk has no MFM on it.
    assert!(Disk::blank(2).mfm_track(0, 0).is_empty());
    assert_eq!(Disk::blank_mfm().density(), Density::Mfm);
}

/// A container round-trips: the header is read, the blocks land where they
/// belong, and the tags come with them.
#[test]
fn a_diskcopy_container_is_read_with_its_tags() {
    let data = raw(BYTES_800K);
    let tags: Vec<u8> = (0..1600 * TAG_BYTES).map(|i| (i * 3) as u8).collect();
    let image = dc42("System Startup", DISK_FORMAT_800K, 0x22, &data, &tags);

    let header = Dc42::parse(&image)
        .expect("a header")
        .expect("one is there");
    assert_eq!(header.name, "System Startup");
    assert_eq!(header.data_size, BYTES_800K as u32);
    assert_eq!(header.tag_size, (1600 * TAG_BYTES) as u32);
    assert_eq!(header.disk_format, DISK_FORMAT_800K);
    assert_eq!(header.format_byte, 0x22);
    assert_eq!(header.blocks(), 1600);
    assert_eq!(header.data_checksum, Dc42::checksum(&data));
    assert_eq!(header.tag_checksum, Dc42::tag_section_checksum(&tags));
    assert!(header.describe().contains("800K GCR"));

    let disk = Disk::from_image(&image).expect("it reads");
    assert_eq!(disk.name(), "System Startup");
    assert_eq!(disk.blocks(), 1600);
    assert_eq!(disk.block(0).unwrap(), &data[..DATA_BYTES]);
    assert_eq!(
        disk.block(1599).unwrap(),
        &data[1599 * DATA_BYTES..1600 * DATA_BYTES]
    );
}

/// A raw image is not mistaken for a container and a container is not mistaken
/// for a raw image, even when the magic bytes line up by accident.
#[test]
fn the_two_containers_are_told_apart_by_more_than_the_magic() {
    let mut image = raw(BYTES_800K);
    // Put the magic exactly where a header would have it. The lengths cannot
    // then agree, so this is still a raw image.
    image[0x52] = 0x01;
    image[0x53] = 0x00;
    assert!(
        Dc42::parse(&image).is_err(),
        "a header whose lengths do not add up is a damaged container, and is said to be"
    );

    // A container whose lengths are right is one.
    let image = dc42("d", DISK_FORMAT_400K, 0x02, &raw(BYTES_400K), &[]);
    assert!(Dc42::parse(&image).unwrap().is_some());
    // And one that is too short for a header at all is not.
    assert!(Dc42::parse(&[0u8; 8]).unwrap().is_none());
}

/// Blocks are laid out cylinder, then side, then outward — and the zones are
/// what decide how many are on each.
#[test]
fn a_block_sits_where_the_speed_zones_put_it() {
    let disk = Disk::blank(2);
    assert_eq!(disk.block_of(0, false, 0), Some(0));
    assert_eq!(disk.block_of(0, false, 11), Some(11));
    assert_eq!(disk.block_of(0, true, 0), Some(12), "the other head");
    assert_eq!(disk.block_of(0, true, 11), Some(23));
    assert_eq!(disk.block_of(1, false, 0), Some(24), "the next cylinder");
    // Cylinder 16 starts a zone of eleven, after sixteen cylinders of twelve on
    // two sides.
    assert_eq!(disk.block_of(16, false, 0), Some(16 * 12 * 2));
    assert_eq!(disk.block_of(16, false, 10), Some(16 * 12 * 2 + 10));
    assert_eq!(disk.block_of(16, false, 11), None, "eleven, not twelve");
    // The last block of the disk.
    assert_eq!(disk.block_of(MAX_TRACK, true, 7), Some(1599));
    assert_eq!(disk.block_of(MAX_TRACK, true, 8), None);
    assert_eq!(disk.block_of(80, false, 0), None, "there is no cylinder 80");

    // A single-sided disk has no second head at all.
    let disk = Disk::blank(1);
    assert_eq!(disk.block_of(0, false, 0), Some(0));
    assert_eq!(disk.block_of(0, true, 0), None);
    assert_eq!(disk.block_of(MAX_TRACK, false, 7), Some(799));
}

/// The whole path, end to end and without a drive: an image becomes cylinders
/// of bits, and the bits become the blocks again.
#[test]
fn every_block_of_an_image_survives_the_journey_to_bits_and_back() {
    let data = raw(BYTES_800K);
    let tags: Vec<u8> = (0..1600 * TAG_BYTES).map(|i| (i * 7 + 1) as u8).collect();
    let disk =
        Disk::from_image(&dc42("t", DISK_FORMAT_800K, 0x22, &data, &tags)).expect("it reads");

    let mut seen = 0usize;
    for track in 0..=MAX_TRACK {
        for side in [false, true] {
            let bits = disk.track(track, side);
            let (sectors, bad) = gcr::decode_track(&bits);
            assert!(bad.is_empty(), "cylinder {track} side {side}: {bad:?}");
            assert_eq!(sectors.len(), usize::from(gcr::sectors_on(track)));
            for sector in &sectors {
                assert_eq!(sector.track, track);
                assert_eq!(sector.side, side);
                assert_eq!(sector.format, gcr::FORMAT_800K);
                let block = disk.block_of(track, side, sector.sector).expect("a block");
                assert_eq!(
                    sector.data(),
                    &data[block * DATA_BYTES..(block + 1) * DATA_BYTES],
                    "cylinder {track} side {side} sector {}",
                    sector.sector
                );
                assert_eq!(
                    sector.tag(),
                    &tags[block * TAG_BYTES..(block + 1) * TAG_BYTES]
                );
                seen += 1;
            }
        }
    }
    assert_eq!(seen, 1600, "every block of the disk");
}

/// A single-sided disk formats as 400K, and its second head has nothing on it.
#[test]
fn a_four_hundred_kilobyte_disk_has_one_side() {
    let disk = Disk::from_image(&raw(BYTES_400K)).expect("400K");
    let (sectors, bad) = gcr::decode_track(&disk.track(0, false));
    assert!(bad.is_empty());
    assert_eq!(sectors[0].format, gcr::FORMAT_400K);
    assert!(disk.track(0, true).is_empty(), "there is no second head");
}

/// The user's own images, read in place: the parser against a real file. The
/// test says what it found and skips when the directory is not set, and no byte
/// of either image is asserted on or kept.
#[test]
#[cfg(feature = "std")]
fn the_users_own_images_are_recognised_and_refused() {
    let Ok(dir) = std::env::var("RSEMU_MAC_DISK_DIR") else {
        std::println!(
            "mac.disk: set RSEMU_MAC_DISK_DIR to a directory of Macintosh disk images to \
             check the container parser against real files."
        );
        return;
    };
    let mut seen = 0;
    let Ok(entries) = std::fs::read_dir(&dir) else {
        std::println!("mac.disk: {dir} is not a directory; skipped");
        return;
    };
    let mut names: alloc::vec::Vec<_> = entries.flatten().map(|e| e.path()).collect();
    names.sort();
    for path in names {
        if !path.is_file() {
            continue;
        }
        let Ok(bytes) = std::fs::read(&path) else {
            continue;
        };
        seen += 1;
        let name = path.file_name().unwrap_or_default().to_string_lossy();
        match Dc42::parse(&bytes) {
            Ok(Some(header)) => {
                std::println!("mac.disk: {name} is {}", header.describe());
                // The checksums are arithmetic over bytes the test never keeps,
                // and they are what says the header is really describing this
                // file rather than being matched by accident.
                let data = &bytes[DC42_HEADER..DC42_HEADER + header.data_size as usize];
                assert_eq!(
                    Dc42::checksum(data),
                    header.data_checksum,
                    "{name}: dataChecksum"
                );
            }
            Ok(None) => std::println!("mac.disk: {name} is a raw image or not an image at all"),
            Err(e) => std::println!("mac.disk: {name}: {e}"),
        }
        match Disk::from_image(&bytes) {
            Ok(disk) => std::println!(
                "mac.disk: {name} loads: {} sides, {} blocks",
                disk.sides(),
                disk.blocks()
            ),
            Err(e) => std::println!("mac.disk: {name} is refused: {e}"),
        }
    }
    std::println!("mac.disk: looked at {seen} files in {dir}");
}
