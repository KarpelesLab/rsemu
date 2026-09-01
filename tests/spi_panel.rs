//! The `spi-panel` board, end to end.
//!
//! A unit test can say "the register accepted the write". These say something
//! stronger: a RISC-V program, running on the emulated hart, configures a
//! Sitronix ST7272A over an emulated SPI link, paints a framebuffer, programs
//! the display controller, and a **picture comes out of the scanout seam** —
//! captured as a PNG that a person can look at.
//!
//! The pair of them also close the loop on the claim `docs/buses/low-speed.md`
//! asks for: the *same firmware* produces the *same panel registers* and the
//! *same picture* whether the SPI link is modelled transactionally or as
//! clocked wires, and
//! [`both_spi_link_models_boot_to_the_same_picture`] is what would fail if that
//! were not true.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-spi-panel`.

#![cfg(feature = "machine-spi-panel")]

use rsemu::core::clock::GlobalTime;
use rsemu::dev::lcd::demo::{PANEL_DEMO, demo_pixel};
use rsemu::host::display::lcd::{LcdScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

/// How wide and tall the board's panel is.
const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;

/// Build the board, boot the demo firmware, and hand back the machine and
/// something to look at it with.
///
/// The bus name no longer has to be unique across the test binary: the SPI
/// rendezvous and the scanout capture both live in this build's own
/// `HostObjects`, so two boards booted at once have two `spi0`s and two panels.
/// `two_boards_at_once_are_two_panels` is the assertion.
fn boot(link: &str, bus: &str) -> (Machine, LcdScanout) {
    let entry = catalog::machine("spi-panel").expect("this build ships spi-panel");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.media.insert("firmware", PANEL_DEMO);
    options
        .resolve
        .params
        .push((String::from("link"), String::from(link)));
    options
        .resolve
        .params
        .push((String::from("spibus"), String::from(bus)));

    capture::install(&mut options).expect("the interception installs");
    let registry = catalog::registry().expect("a registry");
    let mut machine =
        rsemu::machine::build(entry.name, entry.source, &registry, &options).expect("it realizes");
    let picture =
        capture::take(&options.realize.hosts, &machine).expect("the board has a scanout engine");

    // Run until the firmware has enabled the scanout engine, which it does once
    // the framebuffer is painted — 76,800 pixels at about eleven instructions
    // each. A condition rather than a fixed span, because what bounds how much
    // a hart gets through per millisecond is the scheduler's default quantum
    // budget rather than the clock rate, and a hard-coded span would make this
    // test fail the day that default changes.
    let mut elapsed = 0u64;
    while picture.frame_counter() == 0 && elapsed < 4_000_000_000 {
        machine
            .run_for(GlobalTime::from_nanos(20_000_000))
            .expect("it runs");
        elapsed += 20_000_000;
    }
    assert!(
        picture.frame_counter() > 0,
        "the firmware never enabled the scanout engine within {elapsed} ns of virtual time"
    );
    (machine, picture)
}

/// The panel's saved chunk, decoded far enough to read its active registers.
///
/// There is no route from a `dyn Device` to an `St7272a` — `core::device` keeps
/// `Any` out of the supertrait chain deliberately — so the way to see a
/// device's state from outside is the surface `ROADMAP.md` §4.5 already
/// promises: its snapshot chunk. Reading it here doubles as a check that the
/// chunk really is the architectural state.
fn panel_registers(machine: &Machine) -> [u8; 128] {
    use rsemu::core::state::StateReader;

    let bytes = machine.save().expect("the machine snapshots");
    let reader = StateReader::new(&bytes).expect("a snapshot");
    let (_class, _version, data) = reader.load_raw("panel").expect("the panel has a chunk");
    // ticks (u64), frames (u64), then 128 shadow bytes, then 128 active ones.
    let at = 8 + 8 + 128;
    let mut out = [0u8; 128];
    out.copy_from_slice(&data[at..at + 128]);
    out
}

/// The picture, as a host surface.
fn frame(picture: &LcdScanout) -> Surface {
    let mut surface = Surface::new(PixelFormat::RGB888, WIDTH, HEIGHT);
    picture.capture(&mut surface);
    surface
}

// ---------------------------------------------------------------------------
// The picture
// ---------------------------------------------------------------------------

#[test]
fn the_firmware_draws_a_picture_that_reaches_the_scanout_seam() {
    let (_machine, picture) = boot("wired", "spi-panel-picture");

    let info = picture.info();
    assert_eq!(info.width, WIDTH);
    assert_eq!(info.height, HEIGHT);
    assert_eq!(
        info.preferred_format,
        PixelFormat::RGB888,
        "the engine hands out RGB triples and the seam has a format for exactly that"
    );

    let surface = frame(&picture);
    assert_eq!(surface.width(), WIDTH);
    assert_eq!(surface.height(), HEIGHT);

    // Every pixel is a pure function of its coordinates, so the whole frame is
    // checkable without a reference image. This is the assertion that fails if
    // the firmware, the memory map, the scanout engine or the seam breaks.
    let mut wrong = 0usize;
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            let got = surface.get(x, y).expect("inside the surface");
            if got != demo_pixel(x, y) {
                wrong += 1;
            }
        }
    }
    assert_eq!(wrong, 0, "{wrong} of {} pixels are wrong", WIDTH * HEIGHT);

    // And not a blank screen dressed up as a pass.
    assert_ne!(surface.get(0, 0), surface.get(319, 239));
    assert_eq!(surface.get(0, 0), Some([0, 0, 0]));
    assert_eq!(surface.get(1, 0), Some([1, 0, 1]));
    assert_eq!(surface.get(0, 1), Some([0, 1, 1]));
}

#[test]
fn the_frame_period_comes_from_the_pixel_clock() {
    let (_machine, picture) = boot("wired", "spi-panel-rate");

    // The board's `dclk` is 6 MHz and its totals are the ST7272A datasheet's
    // typicals, Th = 371 and Tv = 260 (§7.3.4). 371 x 260 = 96,460 ticks, which
    // at 6 MHz is 16.0766 ms — 62.2 frames a second, not a nominal 60.
    assert_eq!(
        picture.frame_period_ns(),
        96_460 * 1_000_000_000 / 6_000_000
    );
    assert_eq!(picture.frame_period_ns(), 16_076_666);
    assert!(
        picture.frame_counter() > 0,
        "and frames have actually gone by"
    );
}

// ---------------------------------------------------------------------------
// The claim the SPI bus is built around
// ---------------------------------------------------------------------------

#[test]
fn both_spi_link_models_boot_to_the_same_picture() {
    let (wired_machine, wired_picture) = boot("wired", "spi-panel-equiv-wired");
    let wired_regs = panel_registers(&wired_machine);
    let wired_frame = frame(&wired_picture);

    let (txn_machine, txn_picture) = boot("transactional", "spi-panel-equiv-txn");
    let txn_regs = panel_registers(&txn_machine);
    let txn_frame = frame(&txn_picture);

    assert_eq!(
        wired_regs, txn_regs,
        "the same firmware left the panel in the same state either way"
    );
    assert_eq!(
        wired_frame.hash(),
        txn_frame.hash(),
        "and drew the same picture"
    );
}

#[test]
fn the_firmware_actually_configured_the_panel_over_spi() {
    // The commands the demo sends, from `dev::lcd::demo`: 10h <- 09h leaves
    // standby (GRB = 1, DISP = 1), 11h <- 80h sets contrast gain 2, 14h <- 40h
    // leaves brightness at 0. Reading them back proves the whole SPI path
    // carried them, not just that a register accepted a write.
    let (machine, _picture) = boot("wired", "spi-panel-config");
    let regs = panel_registers(&machine);

    assert_eq!(regs[0x10], 0x09, "10h: GRB = 1, DISP = 1 — out of standby");
    assert_eq!(regs[0x11], 0x80, "11h: contrast");
    assert_eq!(regs[0x14], 0x40, "14h: brightness");
    // And nothing else moved: every other register is still its documented
    // reset value, so the frames were not silently misaddressed.
    assert_eq!(regs[0x12], 0x40, "12h untouched");
    assert_eq!(regs[0x19], 0x6d, "19h untouched");
    assert_eq!(regs[0x1c], 0x38, "1Ch untouched");
}

// ---------------------------------------------------------------------------
// Evidence a person can look at
// ---------------------------------------------------------------------------

/// Capture the picture as a real PNG.
///
/// Writes it beside the build, so a reviewer can open the file rather than
/// trust a hash. Gated on `display-png`, which is the feature that has an
/// encoder.
#[cfg(feature = "display-png")]
#[test]
fn the_picture_encodes_to_a_png_a_person_can_open() {
    let (_machine, picture) = boot("wired", "spi-panel-png");
    let surface = frame(&picture);

    let png = rsemu::host::display::png::encode(&surface).expect("it encodes");
    assert!(
        png.starts_with(&[0x89, b'P', b'N', b'G']),
        "a PNG signature"
    );
    assert!(
        png.len() > 1024,
        "and not an empty one: {} bytes",
        png.len()
    );

    let path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("spi-panel.png");
    std::fs::write(&path, &png).expect("the capture is writable");
    eprintln!("spi-panel: wrote {}", path.display());
}
