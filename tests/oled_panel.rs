//! The `oled-spi` board, end to end.
//!
//! A unit test can say "the command interpreter accepted the byte". This says
//! something stronger: a real initialisation sequence and a real frame go down
//! a **real SPI link** — through rsemu's SPI controller, one nine-bit word per
//! `DATA` write, paced by the scheduler, and in the `wired` case as individual
//! edges on `sck`, `mosi` and `cs0` — and a picture comes out of the scanout
//! seam that a person can look at.
//!
//! It is the first board in the tree whose picture is **not in guest memory**.
//! The SSD1306 holds its own GDDRAM; nothing dirties a page, nothing is mapped,
//! and the only route from the bytes to a `Surface` is the device-owned
//! framebuffer seam of `dev::lcd::panel` and `host::display::panel`. That seam
//! is what this file is really testing.
//!
//! # Why the host plays the part of the firmware
//!
//! `oled-spi` has no processor, deliberately — the board file argues it, and
//! `nvme-mini` and `ne2k-mini` are the same shape. So the register writes below
//! are made straight through the address space, which is exactly what a driver
//! does and exactly what `tests/nvme_board.rs` and `tests/pc_at_ide.rs` do. A
//! hart would have needed a firmware image and would have proved nothing more
//! about the panel.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-oled-spi`.

#![cfg(feature = "machine-oled-spi")]

use rsemu::core::clock::GlobalTime;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::host::display::panel::{PanelScanout, capture};
use rsemu::host::display::{PixelFormat, Scanout, Surface};
use rsemu::machine::{Machine, catalog};

/// The SPI controller's register block, as the board file maps it.
const SPI: u64 = 0x4000_0000;
const CTRL: u64 = SPI;
const CLKDIV: u64 = SPI + 0x04;
const CS: u64 = SPI + 0x08;
const STATUS: u64 = SPI + 0x0c;
const DATA: u64 = SPI + 0x10;

/// `CTRL`: enable, mode 0, MSB first, **nine-bit words** — the 3-wire framing of
/// datasheet §8.1.4, whose first bit is `D/C̅`.
const CTRL_9BIT: u64 = (8 << 8) | 1;

/// `STATUS` bit 0.
const BUSY: u64 = 1 << 0;

/// The panel's geometry, as the board file sets it.
const WIDTH: u32 = 128;
const HEIGHT: u32 = 64;

/// The initialisation sequence a 128×64 module's driver ships (§10.1, in order).
const INIT: &[u8] = &[
    0xae, 0xd5, 0x80, 0xa8, 0x3f, 0xd3, 0x00, 0x40, 0x8d, 0x14, 0x20, 0x00, 0xa1, 0xc8, 0xda, 0x12,
    0x81, 0xcf, 0xd9, 0xf1, 0xdb, 0x40, 0xa4, 0xa6, 0xaf,
];

/// The frame: a one-pixel checkerboard, lit where `x + y` is even. A GDDRAM
/// byte is one column of one page with `D0` at the top (§8.7).
fn checkerboard_byte(index: usize) -> u8 {
    if (index % 128).is_multiple_of(2) {
        0x55
    } else {
        0xaa
    }
}

/// Build the board and take a handle on its picture.
fn board(link: &str, bus: &str) -> (Machine, PanelScanout) {
    let entry = catalog::machine("oled-spi").expect("this build ships oled-spi");
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
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
    let machine =
        rsemu::machine::build(entry.name, entry.source, &registry, &options).expect("it realizes");
    let picture = capture::take(&options.realize.hosts).expect("the board has a panel");
    (machine, picture)
}

fn poke(m: &Machine, at: u64, value: u64) {
    m.space("mem")
        .expect("the board has one space")
        .write(at, Width::U32, value, MemAttrs::DEFAULT)
        .expect("inside the map");
}

fn peek(m: &Machine, at: u64) -> u64 {
    m.space("mem")
        .expect("the board has one space")
        .read(at, Width::U32, MemAttrs::DEBUG)
        .expect("inside the map")
}

/// Clock one nine-bit word out, and wait for it to land.
///
/// The effect of a transfer happens when it *finishes*, not when `DATA` is
/// written — the controller charges the scheduler `bits × 2 × (CLKDIV + 1)`
/// ticks either way — so the machine has to be run before the next word.
fn word(m: &mut Machine, word: u32) {
    poke(m, DATA, u64::from(word));
    let mut spins = 0;
    while peek(m, STATUS) & BUSY != 0 {
        m.run_for(GlobalTime::from_nanos(1_000)).expect("it runs");
        spins += 1;
        assert!(spins < 1_000, "a nine-bit transfer never completed");
    }
    // Pop the received word, clearing RXVALID. The SSD1306 drives nothing at
    // all on SPI (§8.1.3), so this is all-ones and only the flag matters.
    let _ = peek(m, DATA);
}

/// One command byte: `D/C̅` clear, in the ninth bit.
fn command(m: &mut Machine, byte: u8) {
    word(m, u32::from(byte));
}

/// One data byte: `D/C̅` set.
fn datum(m: &mut Machine, byte: u8) {
    word(m, 0x100 | u32::from(byte));
}

/// Initialise the panel and paint the checkerboard into its GDDRAM.
fn paint(m: &mut Machine) {
    poke(m, CTRL, CTRL_9BIT);
    poke(m, CLKDIV, 0);
    poke(m, CS, 1); // assert chip select 0 for the whole session
    for byte in INIT {
        command(m, *byte);
    }
    for i in 0..1024 {
        datum(m, checkerboard_byte(i));
    }
    poke(m, CS, 0);
}

/// The picture, as a host surface.
fn frame(picture: &PanelScanout) -> Surface {
    let mut surface = Surface::new(PixelFormat::RGB888, WIDTH, HEIGHT);
    picture.capture(&mut surface);
    surface
}

// ---------------------------------------------------------------------------
// The picture
// ---------------------------------------------------------------------------

#[test]
fn a_frame_clocked_down_the_spi_link_reaches_the_scanout_seam() {
    let (mut machine, picture) = board("wired", "oled-board-picture");

    let info = picture.info();
    assert_eq!(info.width, WIDTH);
    assert_eq!(info.height, HEIGHT);
    assert_eq!(info.preferred_format, PixelFormat::RGB888);
    // Nothing has been sent yet, and §8.5 leaves the display off out of reset.
    let dark = frame(&picture);
    assert!(
        dark.pixels().iter().all(|b| *b == 0),
        "a panel starts black"
    );

    paint(&mut machine);

    let surface = frame(&picture);
    // Every pixel is a pure function of its coordinates, so the whole frame is
    // checkable without a reference image. `mount = "rotated"` in the board
    // file is what makes the module's `A1h`/`C8h` cancel out, so the GDDRAM
    // byte at `(y / 8) * 128 + x` bit `y % 8` is the pixel at `(x, y)`.
    let mut wrong = 0usize;
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            let byte = checkerboard_byte((y as usize / 8) * 128 + x as usize);
            let on = (byte >> (y % 8)) & 1 != 0;
            let want = if on { [0xcf, 0xcf, 0xcf] } else { [0, 0, 0] };
            if surface.get(x, y) != Some(want) {
                wrong += 1;
            }
        }
    }
    assert_eq!(wrong, 0, "{wrong} of {} pixels are wrong", WIDTH * HEIGHT);

    // And not a blank screen dressed up as a pass: `81h CFh` in the sequence is
    // the contrast, which is the lit intensity.
    assert_eq!(surface.get(0, 0), Some([0xcf, 0xcf, 0xcf]));
    assert_eq!(surface.get(1, 0), Some([0, 0, 0]));
}

#[test]
fn the_panel_publishes_a_generation_rather_than_a_frame_rate() {
    let (mut machine, picture) = board("transactional", "oled-board-generation");

    // **There is no frame rate**, and that is the honest answer: an SSD1306
    // refreshes on an internal RC oscillator with no pin, no counter and no
    // interrupt, so a host that advanced the machine by a "frame" would be
    // advancing for nothing. `dev::lcd::panel` argues it.
    assert_eq!(picture.frame_period_ns(), 0);

    let before = picture.frame_counter();
    // Running the machine changes nothing: the picture moves when the guest
    // sends bytes and at no other time.
    machine
        .run_for(GlobalTime::from_nanos(50_000_000))
        .expect("it runs");
    assert_eq!(picture.frame_counter(), before, "time alone is not a frame");

    paint(&mut machine);
    let after = picture.frame_counter();
    assert!(after > before, "sending a frame moved the counter");

    // And a capture does not: a host that redraws on change would otherwise
    // redraw forever.
    let _ = frame(&picture);
    let _ = frame(&picture);
    assert_eq!(picture.frame_counter(), after);
}

#[test]
fn both_spi_link_models_paint_the_same_picture() {
    // The claim `docs/buses/low-speed.md` asks a machine file to make
    // explicitly, on a board whose device holds the picture: the same byte
    // stream through clocked wires and through whole-word calls is the same
    // screen.
    let (mut wired_machine, wired) = board("wired", "oled-board-equiv-wired");
    paint(&mut wired_machine);

    let (mut txn_machine, txn) = board("transactional", "oled-board-equiv-txn");
    paint(&mut txn_machine);

    assert_eq!(frame(&wired).hash(), frame(&txn).hash());
    assert_eq!(wired.frame_counter(), txn.frame_counter());
}

// ---------------------------------------------------------------------------
// The framebuffer is device state
// ---------------------------------------------------------------------------

#[test]
fn a_machine_snapshot_carries_the_panels_own_framebuffer() {
    // The consequence of a device-owned framebuffer that `lcd.scanout` does not
    // have: its pixels live in a RAM device and are saved with it, and these
    // have no such owner. If the panel's chunk did not hold them, a restored
    // machine would come up blank.
    let (mut machine, picture) = board("transactional", "oled-board-snapshot");
    paint(&mut machine);
    let painted = frame(&picture).hash();

    let bytes = machine.save().expect("the machine snapshots");

    let (mut restored_machine, restored) = board("transactional", "oled-board-snapshot-restore");
    assert_ne!(
        frame(&restored).hash(),
        painted,
        "a fresh board is not already showing the picture"
    );
    restored_machine.load(&bytes).expect("it restores");
    assert_eq!(frame(&restored).hash(), painted);
    assert_eq!(restored.frame_counter(), picture.frame_counter());
}

// ---------------------------------------------------------------------------
// Evidence a person can look at
// ---------------------------------------------------------------------------

/// Capture the picture as a real PNG, beside the build, so a reviewer can open
/// the file rather than trust a hash.
#[cfg(feature = "display-png")]
#[test]
fn the_picture_encodes_to_a_png_a_person_can_open() {
    let (mut machine, picture) = board("wired", "oled-board-png");
    paint(&mut machine);
    let surface = frame(&picture);

    let png = rsemu::host::display::png::encode(&surface).expect("it encodes");
    assert!(
        png.starts_with(&[0x89, b'P', b'N', b'G']),
        "a PNG signature"
    );

    let path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("oled-spi.png");
    std::fs::write(&path, &png).expect("the capture is writable");
    eprintln!("oled-spi: wrote {}", path.display());
}
