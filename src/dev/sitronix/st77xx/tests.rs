//! Tests for the ST77xx family, against the datasheet section each one cites.

use super::*;

use alloc::string::ToString;

use crate::bus::spi::exchange;
use crate::core::props::Value;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::dev::lcd::panel::read_frame;

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// A controller built from `props`, panicking on a bad description.
fn build(props: &[(&str, Value)]) -> St77xx {
    let mut p = Props::new();
    for (name, value) in props {
        p.insert(*name, value.clone());
    }
    St77xx::new(&p).expect("a valid ST77xx description")
}

/// The default part: a 240×320 ST7789 on the 4-line framing.
fn tft() -> St77xx {
    build(&[])
}

/// One command byte: `D/CX` low.
fn cmd(dev: &St77xx, byte: u8) {
    dev.feed(false, byte);
}

/// One parameter byte: `D/CX` high.
fn arg(dev: &St77xx, byte: u8) {
    dev.feed(true, byte);
}

/// A command and its parameters.
fn send(dev: &St77xx, byte: u8, args: &[u8]) {
    cmd(dev, byte);
    for a in args {
        arg(dev, *a);
    }
}

/// `SLPOUT` then `DISPON`: the two commands that make anything visible at all
/// (§8.2.13, §8.2.20).
fn wake(dev: &St77xx) {
    cmd(dev, SLPOUT);
    cmd(dev, DISPON);
}

/// Set the write window, in window coordinates (§8.2.21, §8.2.22).
fn window(dev: &St77xx, cols: (u16, u16), rows: (u16, u16)) {
    send(
        dev,
        CASET,
        &[
            (cols.0 >> 8) as u8,
            cols.0 as u8,
            (cols.1 >> 8) as u8,
            cols.1 as u8,
        ],
    );
    send(
        dev,
        RASET,
        &[
            (rows.0 >> 8) as u8,
            rows.0 as u8,
            (rows.1 >> 8) as u8,
            rows.1 as u8,
        ],
    );
}

/// One 5-6-5 pixel as the two bytes §6 puts on the wire, MSB first.
fn rgb565(r: u8, g: u8, b: u8) -> [u8; 2] {
    let v = (u16::from(r & 0x1f) << 11) | (u16::from(g & 0x3f) << 5) | u16::from(b & 0x1f);
    [(v >> 8) as u8, v as u8]
}

/// What a viewer sees at `(x, y)`.
fn pixel(dev: &St77xx, x: u32, y: u32) -> [u8; 3] {
    let (width, _) = dev.shared.geometry();
    let mut row = alloc::vec![[0u8; 3]; width as usize];
    dev.shared.read_row(y, &mut row);
    row[x as usize]
}

/// FNV-1a over every visible pixel: the frame hash `ROADMAP.md` §12 compares.
fn frame_hash(dev: &St77xx) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for row in read_frame(&*dev.shared) {
        for p in row {
            for byte in p {
                h ^= u64::from(byte);
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
    }
    h
}

/// Six bits out, as [`widen6`] produces them.
fn seen(r: u8, g: u8, b: u8) -> [u8; 3] {
    [widen6(r), widen6(g), widen6(b)]
}

// ---------------------------------------------------------------------------
// The window
// ---------------------------------------------------------------------------

#[test]
fn a_ten_pixel_ramwr_lands_in_the_caset_raset_window() {
    let dev = tft();
    wake(&dev);
    window(&dev, (10, 19), (5, 5));
    send(&dev, COLMOD, &[COLMOD_16BIT]);
    assert_eq!(dev.registers().column, (10, 19));
    assert_eq!(dev.registers().row, (5, 5));

    cmd(&dev, RAMWR);
    for i in 0..10u8 {
        let bytes = rgb565(i + 1, 0x20, 0x1f - i);
        arg(&dev, bytes[0]);
        arg(&dev, bytes[1]);
    }

    for i in 0..10u32 {
        let r = (i as u8) + 1;
        let b = 0x1f - i as u8;
        // 5→6 replicates the top bit; green is already six bits.
        let want = [(r << 1) | (r >> 4), 0x20, (b << 1) | (b >> 4)];
        assert_eq!(
            dev.memory_pixel(10 + i, 5),
            Some(want),
            "column {} of row 5",
            10 + i
        );
        assert_eq!(pixel(&dev, 10 + i, 5), seen(want[0], want[1], want[2]));
    }
    // And nothing outside the window moved.
    assert_eq!(dev.memory_pixel(9, 5), Some([0, 0, 0]));
    assert_eq!(dev.memory_pixel(20, 5), Some([0, 0, 0]));
    assert_eq!(dev.memory_pixel(10, 4), Some([0, 0, 0]));
    assert_eq!(dev.memory_pixel(10, 6), Some([0, 0, 0]));
}

#[test]
fn ramwr_wraps_to_the_next_row_at_the_window_edge_and_to_the_window_start_at_the_end() {
    let dev = tft();
    wake(&dev);
    send(&dev, COLMOD, &[COLMOD_16BIT]);
    window(&dev, (4, 6), (2, 3)); // three columns by two rows: six pixels
    cmd(&dev, RAMWR);

    let white = rgb565(0x1f, 0x3f, 0x1f);
    for _ in 0..6 {
        arg(&dev, white[0]);
        arg(&dev, white[1]);
    }
    for y in 2..=3 {
        for x in 4..=6 {
            assert_eq!(
                dev.memory_pixel(x, y),
                Some([0x3f, 0x3f, 0x3f]),
                "({x},{y})"
            );
        }
    }
    // §8.2.23: at the end of the window the pointer goes back to its *start*,
    // not to the next row of frame memory.
    assert_eq!(dev.memory_pixel(4, 4), Some([0, 0, 0]));
    assert_eq!(dev.memory_pixel(7, 2), Some([0, 0, 0]));

    // A seventh pixel overwrites the first.
    let red = rgb565(0x1f, 0, 0);
    arg(&dev, red[0]);
    arg(&dev, red[1]);
    assert_eq!(dev.memory_pixel(4, 2), Some([0x3f, 0, 0]));

    // §8.2.34: `WRMEMC` continues where the pointer is, `RAMWR` rewinds it.
    cmd(&dev, WRMEMC);
    arg(&dev, red[0]);
    arg(&dev, red[1]);
    assert_eq!(
        dev.memory_pixel(5, 2),
        Some([0x3f, 0, 0]),
        "WRMEMC continued"
    );
    cmd(&dev, RAMWR);
    let green = rgb565(0, 0x3f, 0);
    arg(&dev, green[0]);
    arg(&dev, green[1]);
    assert_eq!(dev.memory_pixel(4, 2), Some([0, 0x3f, 0]), "RAMWR rewound");
}

// ---------------------------------------------------------------------------
// MADCTL
// ---------------------------------------------------------------------------

/// Put one white pixel at window coordinate `(cx, cy)` with `madctl` in force.
fn dot(dev: &St77xx, madctl: u8, cx: u16, cy: u16) {
    send(dev, MADCTL, &[madctl]);
    window(dev, (cx, cx), (cy, cy));
    cmd(dev, RAMWR);
    let white = rgb565(0x1f, 0x3f, 0x1f);
    arg(dev, white[0]);
    arg(dev, white[1]);
}

#[test]
fn madctl_mv_swaps_the_axes_and_mx_my_mirror_them() {
    let white = [0x3f, 0x3f, 0x3f];

    // §8.2.29 with every bit clear: the window is frame memory, unturned.
    let plain = tft();
    wake(&plain);
    send(&plain, COLMOD, &[COLMOD_16BIT]);
    dot(&plain, 0, 3, 7);
    assert_eq!(plain.memory_pixel(3, 7), Some(white));

    // `MX` reverses the column order: window column 3 is physical column 236.
    let mx = tft();
    wake(&mx);
    send(&mx, COLMOD, &[COLMOD_16BIT]);
    dot(&mx, MADCTL_MX, 3, 7);
    assert_eq!(mx.memory_pixel(240 - 1 - 3, 7), Some(white));

    // `MY` reverses the row order.
    let my = tft();
    wake(&my);
    send(&my, COLMOD, &[COLMOD_16BIT]);
    dot(&my, MADCTL_MY, 3, 7);
    assert_eq!(my.memory_pixel(3, 320 - 1 - 7), Some(white));

    // `MV` exchanges them, which is what makes a landscape driver's
    // `CASET 0..319` address physical *rows*. Written at window (300, 100) —
    // a column address no unturned 240-wide part could accept at all.
    let mv = tft();
    wake(&mv);
    send(&mv, COLMOD, &[COLMOD_16BIT]);
    dot(&mv, MADCTL_MV, 300, 100);
    assert_eq!(mv.memory_pixel(100, 300), Some(white));

    // The same window without `MV` addresses nothing: §8.2.21 makes
    // `XS ≤ XE ≤ 00EFh` the *host's* obligation, so the register keeps the
    // sixteen bits it was given and the pixels land nowhere rather than being
    // folded back into the picture by a clamp this model invented.
    let narrow = tft();
    wake(&narrow);
    send(&narrow, COLMOD, &[COLMOD_16BIT]);
    dot(&narrow, 0, 300, 100);
    assert_eq!(narrow.registers().column, (300, 300));
    assert_eq!(narrow.memory_pixel(100, 300), Some([0, 0, 0]));
    assert_eq!(narrow.memory_pixel(239, 100), Some([0, 0, 0]));
}

#[test]
fn the_rgb_bit_swaps_red_and_blue_on_output() {
    let dev = tft();
    wake(&dev);
    send(&dev, COLMOD, &[COLMOD_16BIT]);
    window(&dev, (0, 0), (0, 0));
    cmd(&dev, RAMWR);
    let red = rgb565(0x1f, 0, 0);
    arg(&dev, red[0]);
    arg(&dev, red[1]);
    assert_eq!(pixel(&dev, 0, 0), seen(0x3f, 0, 0));

    // §8.2.29 bit 3 is the panel's colour filter order. Applied on the way out,
    // so frame memory — and therefore `RAMRD` — still says red.
    send(&dev, MADCTL, &[MADCTL_RGB]);
    assert_eq!(pixel(&dev, 0, 0), seen(0, 0, 0x3f));
    assert_eq!(dev.memory_pixel(0, 0), Some([0x3f, 0, 0]));
}

// ---------------------------------------------------------------------------
// COLMOD
// ---------------------------------------------------------------------------

#[test]
fn an_18_bit_pixel_stream_packs_three_bytes_per_pixel() {
    let dev = tft();
    wake(&dev);
    // §8.2.33 `66h`, and §6: three bytes, the low two bits of each ignored.
    send(&dev, COLMOD, &[COLMOD_18BIT]);
    window(&dev, (0, 1), (0, 0));
    cmd(&dev, RAMWR);
    for byte in [0xfc, 0x80, 0x04, 0x03, 0x7f, 0xff] {
        arg(&dev, byte);
    }
    assert_eq!(dev.memory_pixel(0, 0), Some([0x3f, 0x20, 0x01]));
    assert_eq!(dev.memory_pixel(1, 0), Some([0x00, 0x1f, 0x3f]));
    assert_eq!(pixel(&dev, 0, 0), seen(0x3f, 0x20, 0x01));
}

#[test]
fn a_12_bit_stream_carries_two_pixels_in_three_bytes() {
    let dev = tft();
    wake(&dev);
    // §8.2.33 `53h`, and §6: RRRRGGGG BBBBRRRR GGGGBBBB.
    send(&dev, COLMOD, &[COLMOD_12BIT]);
    window(&dev, (0, 1), (0, 0));
    cmd(&dev, RAMWR);
    for byte in [0xf0, 0x0f, 0x00] {
        arg(&dev, byte);
    }
    // 4→6 replicates the top two bits, so `Fh` is `3Fh` and `0h` is `00h`.
    assert_eq!(dev.memory_pixel(0, 0), Some([0x3f, 0x00, 0x00]));
    assert_eq!(dev.memory_pixel(1, 0), Some([0x3f, 0x00, 0x00]));
}

// ---------------------------------------------------------------------------
// Output modes
// ---------------------------------------------------------------------------

#[test]
fn invon_inverts_output_without_changing_memory_and_ramrd_proves_it() {
    let dev = tft();
    wake(&dev);
    send(&dev, COLMOD, &[COLMOD_16BIT]);
    window(&dev, (0, 0), (0, 0));
    cmd(&dev, RAMWR);
    let red = rgb565(0x1f, 0, 0);
    arg(&dev, red[0]);
    arg(&dev, red[1]);
    assert_eq!(pixel(&dev, 0, 0), seen(0x3f, 0, 0));

    cmd(&dev, INVON); // §8.2.17
    assert!(dev.registers().inverted);
    assert_eq!(pixel(&dev, 0, 0), seen(0, 0x3f, 0x3f));

    // §8.2.24: `RAMRD` reads *frame memory*, which inversion never touched.
    // Eighteen bits whatever `COLMOD` said about writes, left-aligned in a
    // byte, after one dummy clock (§9.1).
    window(&dev, (0, 0), (0, 0));
    cmd(&dev, RAMRD);
    assert_eq!(dev.read_byte(), 0x00, "the dummy byte of §9.1");
    assert_eq!(dev.read_byte(), 0x3f << 2);
    assert_eq!(dev.read_byte(), 0x00);
    assert_eq!(dev.read_byte(), 0x00);

    cmd(&dev, INVOFF);
    assert_eq!(pixel(&dev, 0, 0), seen(0x3f, 0, 0));
}

#[test]
fn sleep_in_and_display_off_are_black_and_the_frame_hash_says_so() {
    let dev = build(&[("height", Value::from(16u64))]);
    // §8.2.12: the part comes up sleeping, and §8.2.19 with the display off.
    assert!(dev.registers().sleeping);
    assert!(!dev.registers().display_on);
    let black = frame_hash(&dev);

    wake(&dev);
    send(&dev, COLMOD, &[COLMOD_16BIT]);
    window(&dev, (0, 239), (0, 15));
    cmd(&dev, RAMWR);
    let white = rgb565(0x1f, 0x3f, 0x1f);
    for _ in 0..240 * 16 {
        arg(&dev, white[0]);
        arg(&dev, white[1]);
    }
    let painted = frame_hash(&dev);
    assert_ne!(painted, black, "and the picture really did arrive");

    cmd(&dev, DISPOFF);
    assert_eq!(frame_hash(&dev), black);
    cmd(&dev, DISPON);
    assert_eq!(frame_hash(&dev), painted);

    cmd(&dev, SLPIN);
    assert_eq!(frame_hash(&dev), black, "sleeping is black too");
    cmd(&dev, SLPOUT);
    assert_eq!(frame_hash(&dev), painted, "and frame memory survived both");
}

#[test]
fn partial_mode_drives_only_the_partial_area() {
    let dev = build(&[("height", Value::from(32u64))]);
    wake(&dev);
    send(&dev, COLMOD, &[COLMOD_16BIT]);
    window(&dev, (0, 0), (0, 31));
    cmd(&dev, RAMWR);
    let white = rgb565(0x1f, 0x3f, 0x1f);
    for _ in 0..32 {
        arg(&dev, white[0]);
        arg(&dev, white[1]);
    }
    assert_eq!(pixel(&dev, 0, 0), seen(0x3f, 0x3f, 0x3f));

    // §8.2.25: PSL 8, PEL 15. §8.2.14: only those lines are driven.
    send(&dev, PTLAR, &[0, 8, 0, 15]);
    cmd(&dev, PTLON);
    assert_eq!(pixel(&dev, 0, 7), [0, 0, 0]);
    assert_eq!(pixel(&dev, 0, 8), seen(0x3f, 0x3f, 0x3f));
    assert_eq!(pixel(&dev, 0, 15), seen(0x3f, 0x3f, 0x3f));
    assert_eq!(pixel(&dev, 0, 16), [0, 0, 0]);

    cmd(&dev, NORON); // §8.2.15
    assert_eq!(pixel(&dev, 0, 7), seen(0x3f, 0x3f, 0x3f));
}

#[test]
fn vertical_scroll_moves_the_scroll_area_and_leaves_the_fixed_areas() {
    let dev = tft();
    wake(&dev);
    send(&dev, COLMOD, &[COLMOD_18BIT]);
    // One pixel per row, its red channel the row number, so a moved row is
    // identifiable rather than merely different.
    window(&dev, (0, 0), (0, 319));
    cmd(&dev, RAMWR);
    for y in 0..320u32 {
        arg(&dev, ((y % 64) << 2) as u8);
        arg(&dev, 0);
        arg(&dev, 0);
    }
    let row_of = |y: u32| -> u8 { pixel(&dev, 0, y)[0] >> 2 };
    assert_eq!(row_of(0), 0);
    assert_eq!(row_of(100), (100u32 % 64) as u8);

    // §8.2.26: TFA 8, VSA 304, BFA 8 — and they must add up to 320.
    send(&dev, VSCRDEF, &[0, 8, 1, 0x30, 0, 8]);
    assert_eq!(dev.registers().scroll, (8, 304, 8));
    // §8.2.30: the scroll area's first line shows frame-memory row VSP.
    send(&dev, VSCSAD, &[0, 40]);

    // The fixed areas did not move.
    assert_eq!(row_of(0), 0);
    assert_eq!(row_of(7), 7);
    assert_eq!(row_of(312), (312u32 % 64) as u8);
    assert_eq!(row_of(319), (319u32 % 64) as u8);
    // The scroll area starts at row 40 and wraps within [8, 312).
    assert_eq!(row_of(8), 40);
    assert_eq!(row_of(9), 41);
    // The last scrolled line wraps: 8 + ((311 - 8) + (40 - 8)) mod 304 = 39.
    assert_eq!(row_of(311), 39);

    // A definition whose three areas do not add up is ignored rather than
    // guessed at (§8.2.26).
    send(&dev, VSCRDEF, &[0, 8, 0, 8, 0, 8]);
    assert_eq!(row_of(8), 8);
}

#[test]
fn a_240x240_module_shows_rows_0_to_239_or_80_to_319_per_madctl_my() {
    // The 240×240 module everybody buys: the glass covers the top 240 of the
    // part's 320 gate lines, and §8.2.29's `MY` reverses the scan over all 320.
    let dev = build(&[("height", Value::from(240u64))]);
    wake(&dev);
    send(&dev, COLMOD, &[COLMOD_16BIT]);
    assert_eq!(dev.size(), (240, 240));
    assert_eq!(dev.memory_size(), (240, 320));

    // With `MY` clear, window rows 0..239 are the visible ones.
    dot(&dev, 0, 0, 0);
    assert_eq!(pixel(&dev, 0, 0), seen(0x3f, 0x3f, 0x3f));
    dot(&dev, 0, 0, 239);
    assert_eq!(pixel(&dev, 0, 239), seen(0x3f, 0x3f, 0x3f));

    let flipped = build(&[("height", Value::from(240u64))]);
    wake(&flipped);
    send(&flipped, COLMOD, &[COLMOD_16BIT]);
    // With `MY` set, window row 0 lands on frame-memory row 319, which no
    // gate line reaches — which is exactly the "my 180° rotation shows
    // nothing" every driver hits, and why they add 80 to the row address.
    dot(&flipped, MADCTL_MY, 0, 0);
    assert_eq!(pixel(&flipped, 0, 0), [0, 0, 0]);
    assert_eq!(pixel(&flipped, 0, 239), [0, 0, 0]);
    // Rows 80..319 are the visible band instead.
    dot(&flipped, MADCTL_MY, 0, 319);
    assert_eq!(pixel(&flipped, 0, 0), seen(0x3f, 0x3f, 0x3f));
    dot(&flipped, MADCTL_MY, 0, 80);
    assert_eq!(pixel(&flipped, 0, 239), seen(0x3f, 0x3f, 0x3f));
}

// ---------------------------------------------------------------------------
// Framing the command stream
// ---------------------------------------------------------------------------

#[test]
fn a_panel_tuning_command_with_fourteen_parameters_does_not_desynchronise_the_stream() {
    let dev = tft();
    wake(&dev);
    send(&dev, COLMOD, &[COLMOD_16BIT]);
    window(&dev, (0, 0), (0, 0));

    // `PVGAMCTRL` (E0h) with its fourteen parameters. **There is no count to
    // get wrong**: `D/CX` is what distinguishes a command from a parameter
    // (§9.1), so `RAMWR` is recognised whatever this one's arity is. The module
    // docs argue why issue #25's advice to "keep the parameter counts right"
    // does not apply to this part.
    send(
        &dev,
        0xe0,
        &[
            0xd0, 0x04, 0x0d, 0x11, 0x13, 0x2b, 0x3f, 0x54, 0x4c, 0x18, 0x0d, 0x0b, 0x1f, 0x23,
        ],
    );
    assert_eq!(dev.in_progress(), (0xe0, 14), "all fourteen were accepted");

    cmd(&dev, RAMWR);
    let red = rgb565(0x1f, 0, 0);
    arg(&dev, red[0]);
    arg(&dev, red[1]);
    assert_eq!(dev.memory_pixel(0, 0), Some([0x3f, 0, 0]));

    // And the other half of the same property: a command arriving part-way
    // through another's parameters simply replaces it, keeping whatever the
    // first had already latched. `CASET` gets two of its four bytes here, so
    // its start address moved and its end address did not.
    window(&dev, (0, 100), (0, 0));
    send(&dev, CASET, &[0, 5]);
    cmd(&dev, DISPON);
    assert_eq!(dev.registers().column, (5, 100));
}

#[test]
fn rddid_answers_the_variants_bytes_after_a_dummy_clock() {
    let dev = tft();
    // §8.2.3, and §9.1's read frame: one dummy clock, then three bytes.
    cmd(&dev, RDDID);
    assert_eq!(dev.read_byte(), 0x00);
    assert_eq!(dev.read_byte(), 0x85);
    assert_eq!(dev.read_byte(), 0x85);
    assert_eq!(dev.read_byte(), 0x52);
    assert_eq!(dev.read_byte(), 0xff, "and nothing after it");

    // A board may say otherwise.
    let other = build(&[("id", Value::from(0x0012_3456u64))]);
    cmd(&other, RDDID);
    assert_eq!(other.read_byte(), 0x00);
    assert_eq!(other.read_byte(), 0x12);
    assert_eq!(other.read_byte(), 0x34);
    assert_eq!(other.read_byte(), 0x56);

    // §8.2.4: `RDDST` reflects sleep, inversion and display-on.
    wake(&dev);
    cmd(&dev, INVON);
    cmd(&dev, RDDST);
    assert_eq!(dev.read_byte(), 0x00);
    let _madctl = dev.read_byte();
    assert_eq!(dev.read_byte() & (1 << 4), 1 << 4, "awake");
    let third = dev.read_byte();
    assert_eq!(third & (1 << 5), 1 << 5, "inverted");
    assert_eq!(third & (1 << 2), 1 << 2, "display on");
}

// ---------------------------------------------------------------------------
// Transports
// ---------------------------------------------------------------------------

#[test]
fn the_four_wire_framing_reads_the_dc_pin_and_answers_in_the_next_word() {
    let dev = build(&[("interface", Value::from("spi4"))]);
    let slave = dev.spi_slave();
    assert_eq!(slave.format().bits, 8);
    assert_eq!(slave.format().mode, Mode::Mode0);
    assert_eq!(slave.format().order, BitOrder::MsbFirst);

    dev.set_dc(Level::Low);
    exchange(&*slave, u32::from(RDDID));
    // Full duplex: the answer is what was in the shift register when the
    // transfer began, so the dummy comes back on the *next* word (§9.1).
    dev.set_dc(Level::High);
    assert_eq!(exchange(&*slave, 0xff), 0x00);
    assert_eq!(exchange(&*slave, 0xff), 0x85);
    assert_eq!(exchange(&*slave, 0xff), 0x85);
    assert_eq!(exchange(&*slave, 0xff), 0x52);

    // And a write goes the other way through the same pin.
    dev.set_dc(Level::Low);
    exchange(&*slave, u32::from(SLPOUT));
    assert!(!dev.registers().sleeping);
}

#[test]
fn the_three_wire_framing_carries_dc_in_the_ninth_bit() {
    let dev = build(&[("interface", Value::from("spi3"))]);
    let slave = dev.spi_slave();
    // §9.1's `SPI_3W`: nine bits, the first of them `D/CX`.
    assert_eq!(slave.format().bits, 9);

    exchange(&*slave, u32::from(SLPOUT));
    assert!(!dev.registers().sleeping);
    exchange(&*slave, u32::from(COLMOD));
    exchange(&*slave, 0x100 | u32::from(COLMOD_16BIT));
    assert_eq!(dev.registers().colmod, COLMOD_16BIT);
    // The pin is irrelevant here, which is why a board needs no wire for it.
    dev.set_dc(Level::High);
    exchange(&*slave, u32::from(DISPON));
    assert!(dev.registers().display_on);
}

// ---------------------------------------------------------------------------
// Reset and refusals
// ---------------------------------------------------------------------------

#[test]
fn a_software_reset_restores_the_registers_and_keeps_frame_memory() {
    let dev = tft();
    wake(&dev);
    send(&dev, COLMOD, &[COLMOD_16BIT]);
    send(&dev, MADCTL, &[MADCTL_MV | MADCTL_MX]);
    window(&dev, (10, 20), (30, 40));
    cmd(&dev, RAMWR);
    let white = rgb565(0x1f, 0x3f, 0x1f);
    arg(&dev, white[0]);
    arg(&dev, white[1]);
    let before = dev.memory_pixel(240 - 1 - 30, 10).expect("inside memory");
    assert_eq!(before, [0x3f, 0x3f, 0x3f]);

    cmd(&dev, SWRESET);
    // §8.2.2 leaves the part asleep with the display off and the windows over
    // the whole of frame memory…
    assert!(dev.registers().sleeping);
    assert!(!dev.registers().display_on);
    assert_eq!(dev.registers().column, (0, 239));
    assert_eq!(dev.registers().row, (0, 319));
    assert_eq!(dev.registers().madctl, 0);
    // …and does not clear frame memory, which is why every initialisation flow
    // paints the screen itself.
    assert_eq!(dev.memory_pixel(240 - 1 - 30, 10), Some(before));

    // The `RESX` pin does the same, on its falling edge (§8.1).
    wake(&dev);
    let sink = Device::sink(&dev, pin::RES, &[]).expect("the part has a RESX pin");
    assert_eq!(sink.line, pin::RES_LINE);
    sink.sink
        .set_level(WireId::new(1), pin::RES_LINE, Level::Low);
    assert!(dev.registers().sleeping);
    assert_eq!(dev.memory_pixel(240 - 1 - 30, 10), Some(before));
}

#[test]
fn a_description_that_cannot_be_a_module_is_refused() {
    let bad = |props: &[(&str, Value)]| -> String {
        let mut p = Props::new();
        for (name, value) in props {
            p.insert(*name, value.clone());
        }
        St77xx::new(&p)
            .expect_err("this description is not a part")
            .to_string()
    };
    assert!(bad(&[("variant", Value::from("st7796"))]).contains("variant"));
    assert!(bad(&[("interface", Value::from("i2c"))]).contains("interface"));
    // 240×320 of frame memory, so a 240×240 glass 100 rows down does not fit.
    assert!(
        bad(&[
            ("height", Value::from(240u64)),
            ("row-offset", Value::from(100u64)),
        ])
        .contains("frame memory")
    );
    assert!(bad(&[("id", Value::from(0x0100_0000u64))]).contains("three bytes"));
    // An ST7735 is 132×162, and its module is 128×160 two columns in.
    let small = build(&[
        ("variant", Value::from("st7735s")),
        ("col-offset", Value::from(2u64)),
        ("row-offset", Value::from(1u64)),
    ]);
    assert_eq!(small.memory_size(), (132, 162));
    assert_eq!(small.size(), (128, 160));
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

/// Everything a controller would write to a snapshot.
fn saved(dev: &St77xx) -> alloc::vec::Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("tft", ST77XX_CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("tft", ST77XX_CLASS.name, ST77XX_CLASS.version)
            .unwrap();
        dev.save(&mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

/// Load a controller from what [`saved`] produced.
fn restore(dev: &St77xx, bytes: &[u8]) {
    let reader = StateReader::new(bytes).unwrap();
    let chunk = reader
        .load(
            "tft",
            ST77XX_CLASS.name,
            ST77XX_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    dev.load(&mut chunk.reader()).unwrap();
}

#[test]
fn a_snapshot_round_trips_the_frame_memory_and_the_registers() {
    let dev = build(&[("height", Value::from(16u64))]);
    wake(&dev);
    send(&dev, COLMOD, &[COLMOD_16BIT]);
    send(&dev, MADCTL, &[MADCTL_MV]);
    window(&dev, (0, 15), (0, 15));
    cmd(&dev, RAMWR);
    for i in 0..100u8 {
        let bytes = rgb565(i & 0x1f, i & 0x3f, (!i) & 0x1f);
        arg(&dev, bytes[0]);
        arg(&dev, bytes[1]);
    }
    // Half a pixel: a snapshot taken between the two bytes of a 5-6-5 pixel
    // has to resume, not restart.
    arg(&dev, 0xff);

    let bytes = saved(&dev);
    let other = build(&[("height", Value::from(16u64))]);
    restore(&other, &bytes);

    // **Frame memory is the point.** `lcd.scanout` saves six registers and no
    // pixels because its pixels belong to the RAM device; this one has no such
    // owner, so the picture is in the chunk or it is gone.
    assert_eq!(other.registers(), dev.registers());
    assert_eq!(frame_hash(&other), frame_hash(&dev));
    for y in 0..16 {
        for x in 0..16 {
            assert_eq!(other.memory_pixel(x, y), dev.memory_pixel(x, y));
        }
    }

    // The half-written pixel completes identically on both.
    arg(&dev, 0xff);
    arg(&other, 0xff);
    assert_eq!(frame_hash(&other), frame_hash(&dev));
    assert_eq!(saved(&other), saved(&dev));
}

#[test]
fn a_snapshot_resumes_a_half_consumed_read() {
    let dev = tft();
    cmd(&dev, RDDID);
    assert_eq!(dev.read_byte(), 0x00);

    let bytes = saved(&dev);
    let other = tft();
    restore(&other, &bytes);

    // The restored part is still holding the two remaining identifier bytes.
    assert_eq!(other.read_byte(), 0x85);
    assert_eq!(other.read_byte(), 0x85);
    assert_eq!(other.read_byte(), 0x52);
}
