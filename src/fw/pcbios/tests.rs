//! What can be checked about the ROM without running it.
//!
//! The real proof is `tests/pc_at_boot.rs`, which boots a guest on it. These
//! are the properties a broken build would show here first.

use super::{BIOS_DATE, MODEL_BYTE, RESET_VECTOR, SEGMENT, SIZE, image};

#[test]
fn the_image_is_byte_identical_across_builds() {
    // The determinism rule is not decoration here: a machine's state hash
    // includes what the firmware wrote, so an image that varied would make
    // every `pc-at` regression test irreproducible.
    assert_eq!(image(), image());
    assert_eq!(image().len(), SIZE);
}

#[test]
fn the_reset_vector_is_a_far_jump_into_this_segment() {
    // An 80486 fetches `0xfffffff0`, which `pc.rom`'s top alignment puts here.
    // A firmware whose first instruction is not a far jump never leaves the
    // 16-byte window it starts in.
    let rom = image();
    let at = RESET_VECTOR as usize;
    assert_eq!(rom[at], 0xea, "the reset vector is not a far jump");
    let target = u16::from_le_bytes([rom[at + 1], rom[at + 2]]);
    let segment = u16::from_le_bytes([rom[at + 3], rom[at + 4]]);
    assert_eq!(segment, SEGMENT);
    assert!(
        (target as usize) < RESET_VECTOR as usize,
        "the far jump targets {target:#06x}, which is not code"
    );
}

#[test]
fn the_identification_bytes_are_where_software_looks_for_them() {
    let rom = image();
    assert_eq!(&rom[0xfff5..0xfffd], BIOS_DATE);
    assert_eq!(rom[0xfffe], MODEL_BYTE);
}

#[test]
fn the_whole_image_sums_to_zero() {
    // The convention every PC ROM follows, and the only thing the last byte is
    // for. A checksum that does not come out is the cheapest signal that the
    // image was truncated or patched after assembly.
    let sum = image().iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
    assert_eq!(sum, 0);
}

#[test]
fn the_code_fits_below_the_reset_vector() {
    // The assembler panics on an overflow, so this is about the *other* end:
    // the firmware must not have grown into the sixteen bytes at the top.
    let rom = image();
    let used = rom[..RESET_VECTOR as usize]
        .iter()
        .rposition(|&b| b != 0xff)
        .expect("the image is not empty");
    assert!(
        used < RESET_VECTOR as usize,
        "the code reaches {used:#06x}, which collides with the reset vector"
    );
    // Not a size limit, a sanity one: a firmware this small that suddenly
    // filled the socket would mean a runaway table.
    assert!(
        used < 0x4000,
        "the image is unexpectedly large: {used:#06x}"
    );
}
