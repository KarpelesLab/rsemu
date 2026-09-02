//! `rsemu run --screenshot` writes a PNG for every machine that has a display.
//!
//! The library-level scanout seam is covered device by device — `tests/spi_panel.rs`
//! captures an `lcd.scanout` engine, `tests/pc_at_board.rs` a VGA. What none of
//! those can see is the **binary's own wiring**, which is a separate list: one
//! `install_capture` arm per device family and one `take_scanout` arm to match.
//! A family missing from either list produces a machine that draws perfectly and
//! a `--screenshot` that says "this machine has no display" — which is exactly
//! what `spi-panel` did, because `lcd` was in neither.
//!
//! So this file runs the shipped binary, as a person would, and asserts a file
//! on disk. It is the only test that does.
//!
//! # Why the PNG is parsed rather than merely counted
//!
//! A zero-byte file and a valid one are both "a file exists". The signature and
//! the IHDR are eight and thirteen bytes of RFC-free, unambiguous structure
//! (PNG spec, §5.2 and §11.2.2), so checking the geometry costs nothing and
//! turns "something was written" into "the panel's picture was written".

#![cfg(all(feature = "cli", feature = "display-png"))]

use std::path::PathBuf;
use std::process::Command;

/// A scratch path nobody else in this run will pick.
///
/// The process id plus the caller's own name: `cargo test` runs these on
/// parallel harness threads and two tests writing one path would be a flake
/// that only ever appeared in CI.
#[allow(dead_code)]
fn scratch(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("rsemu-cli-{}-{name}", std::process::id()))
}

/// The width and height an IHDR declares, from a file that must be a PNG.
///
/// PNG spec §5.2: the eight-byte signature. §11.2.2: the IHDR is the first
/// chunk, and its data begins with two big-endian `u32`s.
#[allow(dead_code)]
fn png_geometry(bytes: &[u8]) -> (u32, u32) {
    assert!(
        bytes.starts_with(&[0x89, b'P', b'N', b'G', 0x0d, 0x0a, 0x1a, 0x0a]),
        "not a PNG: {:?}",
        &bytes[..bytes.len().min(16)]
    );
    assert_eq!(&bytes[12..16], b"IHDR", "the first chunk is the header");
    let width = u32::from_be_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]);
    let height = u32::from_be_bytes([bytes[20], bytes[21], bytes[22], bytes[23]]);
    (width, height)
}

/// Run the shipped binary and hand back its exit status and its stderr.
#[allow(dead_code)]
fn run(args: &[&str]) -> (bool, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_rsemu"))
        .args(args)
        .output()
        .expect("the binary this test was built alongside");
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

/// The board the defect was found on.
///
/// `spi-panel` has no console chip, so `rsemu run` takes the headless path and
/// reaches `--screenshot`; its display is `lcd.scanout`, the generic RGB engine,
/// which is the one whose frame rate lives in the clock forest rather than in
/// the device. That is why `take_scanout` had to grow the machine as an
/// argument, and this is what proves it did.
#[cfg(feature = "machine-spi-panel")]
#[test]
fn the_spi_panel_gets_a_screenshot() {
    let firmware = scratch("panel.bin");
    std::fs::write(&firmware, rsemu::dev::lcd::demo::PANEL_DEMO)
        .expect("the scratch directory is writable");
    let png = scratch("panel.png");
    let _ = std::fs::remove_file(&png);

    let (ok, stderr) = run(&[
        "run",
        "spi-panel",
        "--media",
        &format!("firmware={}", firmware.display()),
        "--screenshot",
        png.to_str().expect("a UTF-8 scratch path"),
        // Long enough for the demo to paint 76,800 pixels and enable the
        // engine, with room to spare.
        "--for",
        "200ms",
        "-q",
    ]);
    assert!(ok, "rsemu run spi-panel --screenshot failed: {stderr}");

    let bytes = std::fs::read(&png).expect("--screenshot wrote a file");
    // The panel's geometry, from the machine file: an ST7272A is 320RGB x 240.
    assert_eq!(png_geometry(&bytes), (320, 240));

    // And it is the firmware's picture rather than a blank frame. What a blank
    // one *is* comes from the crate's own encoder rather than from a number
    // written down here, so this keeps meaning what it says if the encoder's
    // filter choices ever change.
    let blank = {
        use rsemu::host::display::{PixelFormat, Surface, png};
        png::encode(&Surface::new(PixelFormat::RGB888, 320, 240)).expect("a blank frame encodes")
    };
    assert_ne!(bytes, blank, "the demo's gradient never reached the file");
    assert!(
        bytes.len() > blank.len(),
        "a gradient carries more than a flat colour: {} vs {} bytes",
        bytes.len(),
        blank.len()
    );

    let _ = std::fs::remove_file(&png);
    let _ = std::fs::remove_file(&firmware);
}

/// A minimal iNES image: one 16 KiB PRG bank, one 8 KiB CHR bank, and a reset
/// vector pointing at a branch to itself.
///
/// iNES header layout from the NESdev wiki, "INES": the magic, the two bank
/// counts, then ten bytes this cartridge leaves at zero — mapper 0, horizontal
/// mirroring, no trainer, which is an NROM board.
#[cfg(feature = "machine-nes")]
fn nrom() -> Vec<u8> {
    let mut rom = vec![0u8; 16 + 0x4000 + 0x2000];
    rom[..4].copy_from_slice(b"NES\x1a");
    rom[4] = 1; // one PRG bank
    rom[5] = 1; // one CHR bank
    // `JMP $8000` at the start of PRG, which is $8000 with this mapping.
    rom[16] = 0x4c;
    rom[17] = 0x00;
    rom[18] = 0x80;
    // The reset vector at $FFFC, which is the last-but-four byte of the bank.
    rom[16 + 0x3ffc] = 0x00;
    rom[16 + 0x3ffd] = 0x80;
    rom
}

/// The machines that already worked still do.
///
/// The `lcd` arm was appended to `take_scanout`'s list rather than inserted, so
/// a board with both a console video chip and a scanout engine still shows the
/// console. Nothing in the tree has both today; this asserts the half that can
/// be asserted — that adding the arm did not displace the ones above it.
#[cfg(feature = "machine-nes")]
#[test]
fn a_console_still_gets_its_screenshot() {
    let cart = scratch("nrom.nes");
    std::fs::write(&cart, nrom()).expect("the scratch directory is writable");
    let png = scratch("nes.png");
    let _ = std::fs::remove_file(&png);

    let (ok, stderr) = run(&[
        "run",
        "nes-ntsc",
        "--media",
        &format!("cart={}", cart.display()),
        "--screenshot",
        png.to_str().expect("a UTF-8 scratch path"),
        "--for",
        "50ms",
        "-q",
    ]);
    assert!(ok, "rsemu run nes-ntsc --screenshot failed: {stderr}");

    let bytes = std::fs::read(&png).expect("--screenshot wrote a file");
    assert_eq!(png_geometry(&bytes), (256, 240), "the 2C02's visible frame");

    let _ = std::fs::remove_file(&png);
    let _ = std::fs::remove_file(&cart);
}

/// A machine with no display at all still says so, rather than writing an empty
/// file or claiming success.
#[cfg(feature = "machine-z80-mini")]
#[test]
fn a_machine_with_no_display_says_so() {
    // `HALT`, so the board has something to fetch and stops there.
    let firmware = scratch("halt.bin");
    std::fs::write(&firmware, [0x76u8]).expect("the scratch directory is writable");
    let png = scratch("nothing.png");
    let _ = std::fs::remove_file(&png);
    let (ok, stderr) = run(&[
        "run",
        "z80-mini",
        "--media",
        &format!("firmware={}", firmware.display()),
        "--screenshot",
        png.to_str().expect("a UTF-8 scratch path"),
        "--for",
        "1ms",
        "-q",
    ]);
    assert!(!ok, "a screenshot that could not be taken is a failing run");
    assert!(stderr.contains("no display"), "{stderr}");
    assert!(!png.exists(), "and nothing was written");
    let _ = std::fs::remove_file(&firmware);
}
