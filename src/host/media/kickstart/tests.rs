//! Tests for the Kickstart reader.
//!
//! # Nothing here needs a real Kickstart
//!
//! A Kickstart ROM is Cloanto/Amiga-proprietary and a `rom.key` is licensed to
//! the person who bought it. Neither may be committed, and neither may be
//! fetched — `scripts/fetch-testdata.sh` states the rule for corpora that
//! cannot be redistributed, and it applies here with no free URL to soften it.
//!
//! So the logic is covered by images this file *builds*: [`build`] synthesizes a
//! header and a real footer with a real checksum, and the keyed tests XOR that
//! against a key made of nothing. CI exercises every path with no proprietary
//! byte anywhere near it.
//!
//! The two tests that do want real files read them **in place**, from wherever
//! the user keeps them, behind an environment variable, and skip with a printed
//! line when it is unset:
//!
//! ```text
//!   RSEMU_AMIGA_ROM_DIR=~/"Amiga Files/Shared/rom"      the ROM directory
//!   RSEMU_AMIGA_FOREVER_ISO=~/amiga-forever-dvd.iso     the disc image
//! ```

use alloc::format;
use alloc::vec;
use alloc::vec::Vec;

use super::*;

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// A synthetic ROM image of `len` bytes with a correct header and footer.
///
/// Not a Kickstart — the body is a counting pattern, not 68000 code — but it is
/// byte-for-byte the *shape* of one, which is all the reader looks at. The
/// checksum longword is solved for rather than assumed: whatever makes the
/// whole image sum to `$FFFF_FFFF`.
fn build(len: usize, id: u16, version: Option<(u16, u16)>) -> Vec<u8> {
    assert!(len >= 32 && len.is_multiple_of(4));
    let mut rom = vec![0u8; len];
    rom[0..2].copy_from_slice(&id.to_be_bytes());
    rom[2..4].copy_from_slice(&JMP_L.to_be_bytes());
    rom[4..8].copy_from_slice(&0x00F8_00D2u32.to_be_bytes());
    rom[8..12].copy_from_slice(&0x0000_FFFFu32.to_be_bytes());
    match version {
        Some((v, r)) => {
            rom[12..14].copy_from_slice(&v.to_be_bytes());
            rom[14..16].copy_from_slice(&r.to_be_bytes());
        }
        None => rom[12..16].copy_from_slice(&0xFFFF_FFFFu32.to_be_bytes()),
    }
    // Body: something that is not zero, so the checksum has work to do.
    for (i, byte) in rom[16..len - FOOTER].iter_mut().enumerate() {
        *byte = (i % 251) as u8;
    }
    // Footer: size, then the eight interrupt-acknowledge vector words.
    rom[len - 20..len - 16].copy_from_slice(&(len as u32).to_be_bytes());
    for n in 0..8u16 {
        let at = len - 16 + (n as usize) * 2;
        rom[at..at + 2].copy_from_slice(&(0x0018 + n).to_be_bytes());
    }
    // Solve for the checksum. The stored longword is part of the sum, so with
    // it at zero the image sums to S and the value that takes S to $FFFFFFFF is
    // !S — modulo the end-around carry, which is why this asserts afterwards
    // rather than trusting the algebra.
    let stored = !checksum(&rom);
    rom[len - FOOTER..len - 20].copy_from_slice(&stored.to_be_bytes());
    assert_eq!(checksum(&rom), u32::MAX, "fixture builder is wrong");
    rom
}

/// Wrap `rom` the way Cloanto does: the magic, then XOR with a cycling key.
fn wrap(rom: &[u8], key: &[u8]) -> Vec<u8> {
    let mut out = KEYED_MAGIC.to_vec();
    out.extend(
        rom.iter()
            .zip(key.iter().cycle())
            .map(|(b, k)| b ^ k)
            .collect::<Vec<u8>>(),
    );
    out
}

/// A key with no structure and an awkward length.
///
/// 1426 bytes because that is what a real `rom.key` is, and 512 KiB is not a
/// multiple of it — so every test that uses it exercises the wrap-around that a
/// key dividing the image would hide.
fn fake_key() -> Vec<u8> {
    (0..1426u32)
        .map(|i| (i.wrapping_mul(97) ^ 0x5A) as u8)
        .collect()
}

// ---------------------------------------------------------------------------
// The checksum
// ---------------------------------------------------------------------------

#[test]
fn the_sum_carries_end_around() {
    // Two longwords that overflow 32 bits: $FFFFFFFF + $00000002 is
    // $1_00000001, and the carry comes back in at bit 0 to make $00000002.
    let bytes = [0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x02];
    assert_eq!(checksum(&bytes), 2);
    // Without the end-around carry this would be 1, which is the bug this
    // asserts against.
    assert_ne!(checksum(&bytes), 1);
}

#[test]
fn a_built_image_verifies_at_both_sizes() {
    for len in [256 * 1024, 512 * 1024] {
        let rom = build(len, 0x1114, Some((40, 68)));
        let image = parse("fixture", rom, false).expect("a built image verifies");
        assert_eq!(
            image.checksum,
            Checksum::Verified(be32(&image.bytes, len - FOOTER))
        );
        assert_eq!(image.version, Some((40, 68)));
        assert_eq!(image.entry, 0x00F8_00D2);
        assert_eq!(image.bytes.len(), len);
    }
}

#[test]
fn one_flipped_bit_is_an_error_and_not_a_warning() {
    let mut rom = build(64 * 1024, 0x1111, Some((34, 5)));
    rom[0x4000] ^= 0x01;
    let e = parse("fixture", rom, false).unwrap_err();
    let Error::Config { message, .. } = e else {
        panic!("expected a config error");
    };
    assert!(message.contains("fails its own checksum"), "{message}");
    // The message has to carry both numbers, or it cannot be acted on.
    assert!(message.contains("$FFFFFFFF"), "{message}");
}

#[test]
fn a_truncated_image_is_refused_before_it_is_indexed() {
    // Every prefix of a good image, including the empty one. None may panic,
    // and none may be accepted: the footer's size longword cannot match.
    let rom = build(4096, 0x1111, Some((34, 5)));
    for len in (0..rom.len()).step_by(4) {
        let short = rom[..len].to_vec();
        if let Ok(image) = parse("fixture", short, false) {
            assert_eq!(
                image.checksum,
                Checksum::Absent,
                "a short image verified at {len}"
            );
        }
    }
}

#[test]
fn an_image_with_no_footer_is_accepted_and_says_so() {
    // An A1000 bootstrap ROM's shape: a valid header, and $FFFFFFFF where the
    // size longword would be. There is nothing to check, and refusing it would
    // reject a file Cloanto ships.
    let mut rom = build(8192, 0x1111, None);
    rom[8192 - 20..8192 - 16].copy_from_slice(&0xFFFF_FFFFu32.to_be_bytes());
    let image = parse("fixture", rom, false).expect("a footerless ROM still loads");
    assert_eq!(image.checksum, Checksum::Absent);
    assert!(!image.checksum.verified());
    assert!(
        image.describe().contains("no checksum footer"),
        "{}",
        image.describe()
    );
}

#[test]
fn the_identification_word_is_checked_on_its_high_byte_only() {
    // $1111 on a 512 KiB image and $1114 on a 256 KiB one both occur in
    // Cloanto's own set (Kickstart 36.16 for the A3000, and the A570 extended
    // ROM). Neither may be refused.
    for (len, id) in [(512 * 1024, 0x1111u16), (256 * 1024, 0x1114u16)] {
        let rom = build(len, id, Some((37, 175)));
        let image = parse("fixture", rom, false).expect("the low byte is not a size");
        assert_eq!(image.id, id);
    }
    // A wrong high byte is refused, and the message points at the likeliest
    // cause rather than just stating the number.
    let mut rom = build(4096, 0x1111, None);
    rom[0] = 0x12;
    let Error::Config { message, .. } = parse("fixture", rom, false).unwrap_err() else {
        panic!("expected a config error");
    };
    assert!(message.contains("rom.key"), "{message}");
}

#[test]
fn a_missing_jmp_is_refused() {
    let mut rom = build(4096, 0x1111, None);
    rom[2..4].copy_from_slice(&0x4E71u16.to_be_bytes()); // NOP
    let Error::Config { message, .. } = parse("fixture", rom, false).unwrap_err() else {
        panic!("expected a config error");
    };
    assert!(message.contains("JMP.L"), "{message}");
}

#[test]
fn a_length_that_is_not_a_whole_number_of_longwords_is_refused() {
    let mut rom = build(4096, 0x1111, None);
    rom.push(0);
    let Error::Config { message, .. } = parse("fixture", rom, false).unwrap_err() else {
        panic!("expected a config error");
    };
    assert!(message.contains("longword"), "{message}");
}

// ---------------------------------------------------------------------------
// AMIROMTYPE1
// ---------------------------------------------------------------------------

#[test]
fn the_keyed_wrapper_round_trips_through_a_key_that_does_not_divide_the_image() {
    let key = fake_key();
    for len in [256usize * 1024, 512 * 1024] {
        assert!(
            !len.is_multiple_of(key.len()),
            "the wrap-around must be exercised"
        );
        let rom = build(len, 0x1114, Some((45, 66)));
        let file = wrap(&rom, &key);
        assert!(is_keyed(&file));
        assert_eq!(file.len(), len + KEYED_MAGIC.len());

        let plain = decrypt("fixture", &file, &key).expect("decrypts");
        assert_eq!(plain, rom, "XOR with a cycling key is its own inverse");

        let image = decode("fixture", &file, Some(&key)).expect("decodes and verifies");
        assert!(image.keyed);
        assert!(image.checksum.verified());
        assert_eq!(image.bytes, rom);
        assert!(image.describe().contains("keyed"), "{}", image.describe());
    }
}

#[test]
fn a_plain_image_ignores_the_key_it_was_offered() {
    let rom = build(4096, 0x1111, Some((33, 180)));
    let image = decode("fixture", &rom, Some(&fake_key())).expect("decodes");
    assert!(!image.keyed);
    assert_eq!(image.bytes, rom);
}

#[test]
fn a_keyed_image_with_no_key_says_what_to_do_about_it() {
    let file = wrap(&build(4096, 0x1111, None), &fake_key());
    let Error::Config { message, .. } = decode("fixture", &file, None).unwrap_err() else {
        panic!("expected a config error");
    };
    assert!(message.contains("rom.key"), "{message}");
    assert!(message.contains("key="), "{message}");
}

#[test]
fn the_wrong_key_is_caught_by_the_header_rather_than_producing_a_ruined_board() {
    let rom = build(4096, 0x1111, None);
    let file = wrap(&rom, &fake_key());
    let mut wrong = fake_key();
    wrong[0] ^= 0xFF;
    let Error::Config { message, .. } = decode("fixture", &file, Some(&wrong)).unwrap_err() else {
        panic!("a key that is wrong in its first byte cannot produce a valid id word");
    };
    assert!(message.contains("identification word"), "{message}");
}

#[test]
fn the_wrong_key_deeper_in_is_caught_by_the_checksum() {
    // A key that agrees for the first longwords gets past the header, which is
    // exactly the case the checksum exists for.
    let rom = build(4096, 0x1111, None);
    let file = wrap(&rom, &fake_key());
    let mut wrong = fake_key();
    let last = wrong.len() - 1;
    wrong[last] ^= 0xFF;
    let Error::Config { message, .. } = decode("fixture", &file, Some(&wrong)).unwrap_err() else {
        panic!("expected the checksum to catch it");
    };
    assert!(message.contains("checksum"), "{message}");
}

#[test]
fn an_empty_key_is_refused_rather_than_applied_as_an_identity() {
    let file = wrap(&build(4096, 0x1111, None), &fake_key());
    let Error::Config { message, .. } = decrypt("fixture", &file, &[]).unwrap_err() else {
        panic!("expected a config error");
    };
    assert!(message.contains("empty"), "{message}");
}

#[test]
fn the_magic_alone_is_not_a_keyed_image() {
    assert!(!is_keyed(KEYED_MAGIC));
    assert!(!is_keyed(b"AMIROMTYPE"));
    assert!(!is_keyed(&[]));
    assert!(is_keyed(b"AMIROMTYPE1\x00"));
}

// ---------------------------------------------------------------------------
// The specification
// ---------------------------------------------------------------------------

#[test]
fn a_specification_is_a_source_then_options() {
    let spec = Spec::parse("/roms/kick.rom").unwrap();
    assert_eq!(spec.source, PathBuf::from("/roms/kick.rom"));
    assert_eq!(spec.rom, None);
    assert_eq!(spec.key, None);

    let spec = Spec::parse("/media/dvd.iso,rom=amiga-os-310-a1200.rom,key=/k/rom.key").unwrap();
    assert_eq!(spec.source, PathBuf::from("/media/dvd.iso"));
    assert_eq!(spec.rom.as_deref(), Some("amiga-os-310-a1200.rom"));
    assert_eq!(spec.key, Some(PathBuf::from("/k/rom.key")));
}

#[test]
fn an_unknown_option_names_the_ones_that_exist() {
    let Error::Config { message, .. } = Spec::parse("/roms/kick.rom,ro").unwrap_err() else {
        panic!("expected a config error");
    };
    assert!(message.contains("rom="), "{message}");
    assert!(message.contains("key="), "{message}");
}

#[test]
fn an_empty_specification_says_what_one_looks_like() {
    let Error::Config { message, .. } = Spec::parse("").unwrap_err() else {
        panic!("expected a config error");
    };
    assert!(message.contains("kickstart:"), "{message}");
}

// ---------------------------------------------------------------------------
// The files, on disk
// ---------------------------------------------------------------------------

/// A temporary directory this test owns, removed when it drops.
struct Scratch(PathBuf);

impl Scratch {
    fn new(tag: &str) -> Scratch {
        let dir =
            std::env::temp_dir().join(format!("rsemu-kickstart-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("a scratch directory");
        Scratch(dir)
    }

    fn write(&self, name: &str, bytes: &[u8]) -> PathBuf {
        let path = self.0.join(name);
        std::fs::write(&path, bytes).expect("writes");
        path
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

#[test]
fn a_keyed_rom_finds_the_key_beside_it() {
    let scratch = Scratch::new("beside");
    let key = fake_key();
    let rom = build(64 * 1024, 0x1111, Some((34, 5)));
    let path = scratch.write("kick.rom", &wrap(&rom, &key));
    scratch.write(KEY_NAME, &key);

    let image = open(&path.display().to_string()).expect("finds rom.key beside the ROM");
    assert_eq!(image.bytes, rom);
    assert!(image.keyed);
    assert!(image.checksum.verified());
}

#[test]
fn a_key_elsewhere_is_named_with_the_key_option() {
    let roms = Scratch::new("roms");
    let keys = Scratch::new("keys");
    let key = fake_key();
    let rom = build(64 * 1024, 0x1111, Some((34, 5)));
    let path = roms.write("kick.rom", &wrap(&rom, &key));
    let key_path = keys.write(KEY_NAME, &key);

    let spec = format!("{},key={}", path.display(), key_path.display());
    let image = open(&spec).expect("takes the key from `key=`");
    assert_eq!(image.bytes, rom);
}

#[test]
fn a_keyed_rom_with_no_key_anywhere_names_the_file_it_looked_for() {
    let scratch = Scratch::new("nokey");
    let path = scratch.write("kick.rom", &wrap(&build(4096, 0x1111, None), &fake_key()));
    let Error::Config { message, .. } = open(&path.display().to_string()).unwrap_err() else {
        panic!("expected a config error");
    };
    assert!(message.contains(KEY_NAME), "{message}");
    assert!(message.contains("key="), "{message}");
}

#[test]
fn rom_equals_on_something_that_is_not_a_disc_says_so() {
    let scratch = Scratch::new("notiso");
    let path = scratch.write("kick.rom", &build(4096, 0x1111, None));
    let spec = format!("{},rom=whatever", path.display());
    let Error::Config { message, .. } = open(&spec).unwrap_err() else {
        panic!("expected a config error");
    };
    assert!(message.contains("ISO 9660"), "{message}");
}

#[test]
fn a_file_that_is_not_there_is_reported_as_a_file() {
    let Error::Config { at, message } = open("/nonexistent/rsemu/kick.rom").unwrap_err() else {
        panic!("expected a config error");
    };
    assert_eq!(at, "/nonexistent/rsemu/kick.rom");
    assert!(message.contains("cannot be opened"), "{message}");
}

#[test]
fn an_iso_that_is_not_an_amiga_forever_disc_is_named_as_such() {
    // A volume descriptor set with the right magic and nothing else in it:
    // enough to be taken for an ISO, not enough to be a Kickstart source. What
    // matters is *which* error comes back, not that one does.
    let scratch = Scratch::new("wrongiso");
    let mut iso = vec![0u8; 32 * 2048];
    iso[16 * 2048] = 1;
    iso[16 * 2048 + 1..16 * 2048 + 6].copy_from_slice(b"CD001");
    iso[16 * 2048 + 6] = 1;
    let path = scratch.write("other.iso", &iso);

    let spec = format!("{},rom=amiga-os-310-a1200.rom", path.display());
    let Error::Config { message, .. } = open(&spec).unwrap_err() else {
        panic!("expected a config error");
    };
    assert!(
        message.contains("ISO 9660") || message.contains("Amiga Forever"),
        "{message}"
    );
}

// ---------------------------------------------------------------------------
// The user's own media, read in place
// ---------------------------------------------------------------------------

#[test]
fn the_users_rom_directory_decodes_and_verifies() {
    let Ok(dir) = std::env::var("RSEMU_AMIGA_ROM_DIR") else {
        println!(
            "kickstart: set RSEMU_AMIGA_ROM_DIR to an Amiga Forever `Shared/rom` directory to \
             check the reader against real images. Nothing is copied and nothing is vendored: \
             a Kickstart and a `rom.key` are licensed to whoever bought them."
        );
        return;
    };
    let dir = Path::new(&dir);
    let mut checked = 0usize;
    let mut footerless = 0usize;
    let entries = std::fs::read_dir(dir).unwrap_or_else(|e| panic!("{}: {e}", dir.display()));
    for entry in entries {
        let path = entry.expect("a directory entry").path();
        if path.extension().is_none_or(|e| e != "rom") {
            continue;
        }
        // A zero-length placeholder ships in that directory; it is not an image.
        if std::fs::metadata(&path).map(|m| m.len()).unwrap_or(0) == 0 {
            continue;
        }
        let name = path.display().to_string();
        match open(&name) {
            Ok(image) => {
                if image.checksum.verified() {
                    checked += 1;
                } else {
                    footerless += 1;
                }
            }
            // Expansion and bootstrap ROMs share the extension and are not
            // Kickstarts at all. They must be *refused*, clearly — never
            // accepted as a board's firmware.
            Err(Error::Config { .. }) => footerless += 1,
            Err(e) => panic!("{name}: {e}"),
        }
    }
    println!("kickstart: {checked} image(s) verified, {footerless} not Kickstarts");
    assert!(checked > 0, "no image in {} verified", dir.display());
}

#[test]
fn a_rom_comes_straight_out_of_the_users_disc_image() {
    let Ok(iso) = std::env::var("RSEMU_AMIGA_FOREVER_ISO") else {
        println!(
            "kickstart: set RSEMU_AMIGA_FOREVER_ISO to an Amiga Forever DVD image to check the \
             ISO 9660 path. The disc is read in place; nothing is extracted or copied."
        );
        return;
    };
    // No `rom=`: the error is the feature, so assert it lists what is there.
    let Error::Config { message, .. } = open(&iso).unwrap_err() else {
        panic!("a disc with no `rom=` should say which ROMs it holds");
    };
    assert!(message.contains("rom="), "{message}");
    assert!(message.contains(".rom"), "{message}");

    // And then the ROM itself, keyed, with the key taken off the same disc.
    let spec = format!("{iso},rom=amiga-os-310-a1200.rom");
    let image = open(&spec).unwrap_or_else(|e| panic!("{spec}: {e}"));
    assert!(image.keyed, "the disc's Kickstarts are AMIROMTYPE1 images");
    assert!(image.checksum.verified());
    assert_eq!(image.bytes.len(), 512 * 1024);
    assert_eq!(image.version, Some((40, 68)), "Kickstart 3.1 for the A1200");

    // The `.rom` suffix is a convenience, not a requirement.
    let short = format!("{iso},rom=amiga-os-310-a1200");
    let same = open(&short).unwrap_or_else(|e| panic!("{short}: {e}"));
    assert_eq!(same.bytes, image.bytes);

    // A name that is not on the disc lists what is.
    let missing = format!("{iso},rom=not-a-real-rom");
    let Error::Config { message, .. } = open(&missing).unwrap_err() else {
        panic!("expected a config error");
    };
    assert!(message.contains("not-a-real-rom"), "{message}");
}
