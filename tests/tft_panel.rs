//! The `tft-spi` board, end to end.
//!
//! The colour half of the same claim `tests/oled_panel.rs` makes: a real
//! initialisation sequence and a real windowed `RAMWR` go down a **real SPI
//! link** — through rsemu's SPI controller, one nine-bit word per `DATA` write,
//! paced by the scheduler, and in the `wired` case as individual edges — and a
//! picture comes out of the scanout seam.
//!
//! It also exercises the half of the ST77xx that the SSD1306 has no equivalent
//! for: **read-back**. `RDDID` and `RAMRD` answer on `SDO`, which on this board
//! is a real wire into the controller's `MISO` input, and the bytes come back
//! through `DATA` reads. A guest that can read its own frame memory is a guest
//! that can tell whether the link works, and nothing else in the display tree
//! could do that before.
//!
//! `tft-spi` has no processor, deliberately — the board file argues it — so the
//! register writes below are made straight through the address space, which is
//! what a driver does and what `tests/nvme_board.rs` does.
//!
//! Everything here needs a machine, so the whole file is gated on
//! `machine-tft-spi`.

#![cfg(feature = "machine-tft-spi")]

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

/// `CTRL`: enable, mode 0, MSB first, **nine-bit words** — the 3-line framing
/// of §9.1, whose first bit is `D/CX`.
const CTRL_9BIT: u64 = (8 << 8) | 1;
/// `STATUS` bit 0.
const BUSY: u64 = 1 << 0;

/// The glass, as the board file sets it.
const WIDTH: u32 = 240;
const HEIGHT: u32 = 240;

/// The square the test paints, in the top left.
const SQUARE: u16 = 32;

/// Build the board and take a handle on its picture.
fn board(link: &str, bus: &str) -> (Machine, PanelScanout) {
    let entry = catalog::machine("tft-spi").expect("this build ships tft-spi");
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

/// Clock one nine-bit word out and hand back what came in on `MISO`.
fn word(m: &mut Machine, out: u32) -> u8 {
    poke(m, DATA, u64::from(out));
    let mut spins = 0;
    while peek(m, STATUS) & BUSY != 0 {
        m.run_for(GlobalTime::from_nanos(1_000)).expect("it runs");
        spins += 1;
        assert!(spins < 1_000, "a nine-bit transfer never completed");
    }
    (peek(m, DATA) & 0xff) as u8
}

/// One command byte: `D/CX` clear, in the ninth bit.
fn command(m: &mut Machine, byte: u8) {
    word(m, u32::from(byte));
}

/// One parameter byte: `D/CX` set.
fn param(m: &mut Machine, byte: u8) {
    word(m, 0x100 | u32::from(byte));
}

/// A command and its parameters.
fn send(m: &mut Machine, byte: u8, args: &[u8]) {
    command(m, byte);
    for a in args {
        param(m, *a);
    }
}

/// Bring the link up and assert the chip select.
fn open(m: &mut Machine) {
    poke(m, CTRL, CTRL_9BIT);
    poke(m, CLKDIV, 0);
    poke(m, CS, 1);
}

/// Wake the panel, pick 16-bit colour, and paint a `SQUARE`×`SQUARE` block of
/// red in the top-left corner.
fn paint(m: &mut Machine) {
    open(m);
    command(m, 0x11); // SLPOUT
    send(m, 0x3a, &[0x55]); // COLMOD, 16-bit 5-6-5
    send(m, 0x36, &[0x00]); // MADCTL, unturned
    command(m, 0x29); // DISPON
    send(m, 0x2a, &[0, 0, 0, (SQUARE - 1) as u8]); // CASET 0..31
    send(m, 0x2b, &[0, 0, 0, (SQUARE - 1) as u8]); // RASET 0..31
    command(m, 0x2c); // RAMWR
    for _ in 0..u32::from(SQUARE) * u32::from(SQUARE) {
        // Full red in 5-6-5: F800h, MSB first.
        param(m, 0xf8);
        param(m, 0x00);
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

#[test]
fn a_windowed_ramwr_down_the_spi_link_reaches_the_scanout_seam() {
    let (mut machine, picture) = board("wired", "tft-board-picture");

    let info = picture.info();
    assert_eq!(info.width, WIDTH);
    assert_eq!(info.height, HEIGHT);
    assert_eq!(info.preferred_format, PixelFormat::RGB888);
    // §8.2.12 and §8.2.19: the part comes up asleep with the display off.
    let dark = frame(&picture);
    assert!(
        dark.pixels().iter().all(|b| *b == 0),
        "a panel starts black"
    );

    paint(&mut machine);

    let surface = frame(&picture);
    // 1Fh of red widened to eight bits is FFh, because the expansion replicates
    // the high bits — the convention that keeps white white.
    let red = Some([0xff, 0, 0]);
    let black = Some([0, 0, 0]);
    let mut wrong = 0usize;
    for y in 0..HEIGHT {
        for x in 0..WIDTH {
            let want = if x < u32::from(SQUARE) && y < u32::from(SQUARE) {
                red
            } else {
                black
            };
            if surface.get(x, y) != want {
                wrong += 1;
            }
        }
    }
    assert_eq!(wrong, 0, "{wrong} of {} pixels are wrong", WIDTH * HEIGHT);
    // The window's edges, exactly.
    assert_eq!(surface.get(31, 31), red);
    assert_eq!(surface.get(32, 31), black);
    assert_eq!(surface.get(31, 32), black);
}

#[test]
fn the_panel_answers_rddid_and_ramrd_back_down_the_miso_wire() {
    let (mut machine, _picture) = board("wired", "tft-board-readback");
    paint(&mut machine);
    open(&mut machine);

    // §8.2.3 with §9.1's read frame: one dummy clock, then three bytes. The
    // answer arrives on the *next* word, because SPI is full duplex and the
    // shift register was already loaded when each transfer began.
    command(&mut machine, 0x04); // RDDID
    assert_eq!(word(&mut machine, 0x1ff), 0x00, "the dummy byte");
    assert_eq!(word(&mut machine, 0x1ff), 0x85, "ST7789V manufacturer");
    assert_eq!(word(&mut machine, 0x1ff), 0x85);
    assert_eq!(word(&mut machine, 0x1ff), 0x52);

    // §8.2.24: `RAMRD` hands back eighteen bits per pixel whatever `COLMOD`
    // said about writes. The square was painted full red.
    send(&mut machine, 0x2a, &[0, 0, 0, 0]);
    send(&mut machine, 0x2b, &[0, 0, 0, 0]);
    command(&mut machine, 0x2e); // RAMRD
    assert_eq!(word(&mut machine, 0x1ff), 0x00, "the dummy byte");
    assert_eq!(
        word(&mut machine, 0x1ff),
        0xfc,
        "6 bits of red, left aligned"
    );
    assert_eq!(word(&mut machine, 0x1ff), 0x00);
    assert_eq!(word(&mut machine, 0x1ff), 0x00);
    poke(&machine, CS, 0);
}

#[test]
fn both_spi_link_models_paint_the_same_picture() {
    // The claim `docs/buses/low-speed.md` asks a machine file to make
    // explicitly, on a board whose device holds the picture.
    let (mut wired_machine, wired) = board("wired", "tft-board-equiv-wired");
    paint(&mut wired_machine);

    let (mut txn_machine, txn) = board("transactional", "tft-board-equiv-txn");
    paint(&mut txn_machine);

    assert_eq!(frame(&wired).hash(), frame(&txn).hash());
    assert_eq!(wired.frame_counter(), txn.frame_counter());
}

#[test]
fn a_machine_snapshot_carries_the_panels_own_frame_memory() {
    // The consequence of a device-owned framebuffer that `lcd.scanout` does not
    // have: its pixels live in a RAM device and are saved with it, and these
    // have no such owner.
    let (mut machine, picture) = board("transactional", "tft-board-snapshot");
    paint(&mut machine);
    let painted = frame(&picture).hash();

    let bytes = machine.save().expect("the machine snapshots");

    let (mut restored_machine, restored) = board("transactional", "tft-board-snapshot-restore");
    assert_ne!(frame(&restored).hash(), painted, "a fresh board is blank");
    restored_machine.load(&bytes).expect("it restores");
    assert_eq!(frame(&restored).hash(), painted);
    assert_eq!(restored.frame_counter(), picture.frame_counter());
}
