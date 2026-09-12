//! Tests for the SSD1306 family, against the datasheet section each one cites.

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
fn build(props: &[(&str, Value)]) -> Ssd1306 {
    let mut p = Props::new();
    for (name, value) in props {
        p.insert(*name, value.clone());
    }
    Ssd1306::new(&p).expect("a valid SSD1306 description")
}

/// The default part: a 128×64 SSD1306 on 4-wire SPI, glass the die's way up.
fn oled() -> Ssd1306 {
    build(&[])
}

/// One command byte, straight into the interpreter every transport shares.
fn cmd(dev: &Ssd1306, byte: u8) {
    dev.feed(false, byte);
}

/// One data byte.
fn data(dev: &Ssd1306, byte: u8) {
    dev.feed(true, byte);
}

/// Whether the pixel at `(x, y)` is lit, as a viewer sees it.
fn lit(dev: &Ssd1306, x: u32, y: u32) -> bool {
    let (width, _) = dev.shared.geometry();
    let mut row = alloc::vec![[0u8; 3]; width as usize];
    dev.shared.read_row(y, &mut row);
    row[x as usize] != [0, 0, 0]
}

/// FNV-1a over every visible pixel: the frame hash `ROADMAP.md` §12 compares.
fn frame_hash(dev: &Ssd1306) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for row in read_frame(&*dev.shared) {
        for pixel in row {
            for byte in pixel {
                h ^= u64::from(byte);
                h = h.wrapping_mul(0x0000_0100_0000_01b3);
            }
        }
    }
    h
}

/// The initialisation sequence every driver for a 128×64 module ships, and the
/// one issue #24 quotes: §10.1 in order, ending with the display on.
const INIT: &[u8] = &[
    0xae, // display off (§10.1.18)
    0xd5, 0x80, // clock divide / oscillator (§10.1.16)
    0xa8, 0x3f, // multiplex ratio 64 (§10.1.16)
    0xd3, 0x00, // display offset 0 (§10.1.20)
    0x40, // display start line 0 (§10.1.14)
    0x8d, 0x14, // charge pump on (§10.1.9)
    0x20, 0x00, // horizontal addressing (§10.1.3)
    0xa1, // segment remap (§10.1.11)
    0xc8, // COM scan remapped (§10.1.17)
    0xda, 0x12, // COM pins (§10.1.22)
    0x81, 0xcf, // contrast (§10.1.7)
    0xd9, 0xf1, // pre-charge (§10.1.21)
    0xdb, 0x40, // VCOMH deselect (§10.1.23)
    0xa4, // resume to RAM content (§10.1.12)
    0xa6, // normal, not inverse (§10.1.13)
    0xaf, // display on (§10.1.18)
];

/// The frame the tests paint: a one-pixel checkerboard, lit where `x + y` is
/// even. A byte is one column of one page with `D0` at the top (§8.7), so an
/// even column is `01010101` and an odd one `10101010`.
fn checkerboard_byte(index: usize) -> u8 {
    if (index % 128).is_multiple_of(2) {
        0x55
    } else {
        0xaa
    }
}

// ---------------------------------------------------------------------------
// The whole path
// ---------------------------------------------------------------------------

#[test]
fn the_init_sequence_and_a_full_frame_light_the_expected_pixels() {
    // The module the sequence is written for: `A1h`/`C8h` are in it because the
    // glass is bonded 180° to the die, not because the picture wants mirroring.
    let dev = build(&[("mount", Value::from("rotated"))]);
    for byte in INIT {
        cmd(&dev, *byte);
    }
    assert!(dev.registers().display_on);
    assert_eq!(dev.registers().mode, AddrMode::Horizontal);
    assert_eq!(dev.registers().contrast, 0xcf);
    assert_eq!(
        dev.unlisted_commands(),
        0,
        "every byte of INIT is a command"
    );

    let frame: alloc::vec::Vec<u8> = (0..1024).map(checkerboard_byte).collect();
    for byte in &frame {
        data(&dev, *byte);
    }

    // Exactly the claim issue #24 makes: the byte at `(y / 8) * 128 + x` of the
    // stream, bit `y % 8`, is the pixel at `(x, y)`.
    for y in 0..64u32 {
        for x in 0..128u32 {
            let byte = frame[(y as usize / 8) * 128 + x as usize];
            let want = (byte >> (y % 8)) & 1 != 0;
            assert_eq!(lit(&dev, x, y), want, "pixel ({x}, {y})");
        }
    }
    // And it really is the checkerboard, not a uniform field dressed up as one.
    assert!(lit(&dev, 0, 0));
    assert!(!lit(&dev, 1, 0));
    assert!(!lit(&dev, 0, 1));
    assert!(lit(&dev, 127, 63));
}

#[test]
fn a_frame_hash_of_the_checkerboard_is_stable_across_two_runs() {
    let paint = || {
        let dev = build(&[("mount", Value::from("rotated"))]);
        for byte in INIT {
            cmd(&dev, *byte);
        }
        for i in 0..1024 {
            data(&dev, checkerboard_byte(i));
        }
        dev
    };
    let first = paint();
    let second = paint();
    assert_eq!(frame_hash(&first), frame_hash(&second));
    // Two captures of the same panel agree too: `read_row` has no side effect,
    // which is the `MemAttrs::debug` rule applied to a picture.
    assert_eq!(frame_hash(&first), frame_hash(&first));
    assert_eq!(first.shared.generation(), second.shared.generation());
    // A capture must not move the generation, or a host that draws on change
    // draws forever.
    let before = first.shared.generation();
    let _ = frame_hash(&first);
    assert_eq!(first.shared.generation(), before);
}

// ---------------------------------------------------------------------------
// Addressing (§10.1.3)
// ---------------------------------------------------------------------------

#[test]
fn page_mode_wraps_the_column_within_the_page_and_does_not_advance_the_page() {
    let dev = oled();
    // Page mode is the reset default (§10.1.3: "Page addressing mode (RESET)"),
    // so this only sets the pointer: page 3, column 126.
    cmd(&dev, 0xb3);
    cmd(&dev, 0x0e); // lower nibble: column 0x_E
    cmd(&dev, 0x17); // higher nibble: column 0x7E = 126
    assert_eq!(dev.registers().page, 3);
    assert_eq!(dev.registers().column, 126);

    data(&dev, 0x11);
    data(&dev, 0x22);
    // §10.1.1: at the end of RAM the column resets to 0 and **the page address
    // is not changed**. That is the whole difference from horizontal mode.
    data(&dev, 0x33);
    assert_eq!(dev.registers().page, 3, "page mode never advances the page");
    assert_eq!(dev.registers().column, 1);
    assert_eq!(dev.gddram(3, 126), Some(0x11));
    assert_eq!(dev.gddram(3, 127), Some(0x22));
    assert_eq!(dev.gddram(3, 0), Some(0x33));
    assert_eq!(dev.gddram(4, 0), Some(0x00), "and page 4 is untouched");
}

#[test]
fn horizontal_mode_wraps_from_the_last_column_to_the_next_page_within_the_ranges() {
    let dev = oled();
    cmd(&dev, 0x20);
    cmd(&dev, 0x00); // horizontal
    cmd(&dev, 0x21);
    cmd(&dev, 4); // column 4..6
    cmd(&dev, 6);
    cmd(&dev, 0x22);
    cmd(&dev, 1); // page 1..2
    cmd(&dev, 2);
    // §10.1.4/§10.1.5: setting a range also resets the pointer to its start.
    assert_eq!(dev.registers().column, 4);
    assert_eq!(dev.registers().page, 1);

    // Six bytes exactly fill the window: three columns by two pages.
    for i in 0..6u8 {
        data(&dev, 0xa0 + i);
    }
    assert_eq!(dev.gddram(1, 4), Some(0xa0));
    assert_eq!(dev.gddram(1, 5), Some(0xa1));
    assert_eq!(dev.gddram(1, 6), Some(0xa2));
    // Column wrapped to the start of the window and the page advanced.
    assert_eq!(dev.gddram(2, 4), Some(0xa3));
    assert_eq!(dev.gddram(2, 6), Some(0xa5));
    // And at the end of the page range, back to the first page of it: the
    // seventh byte overwrites the first.
    assert_eq!(dev.registers().page, 1);
    assert_eq!(dev.registers().column, 4);
    data(&dev, 0xa6);
    assert_eq!(dev.gddram(1, 4), Some(0xa6));
    assert_eq!(
        dev.gddram(0, 4),
        Some(0x00),
        "outside the window, untouched"
    );
    assert_eq!(dev.gddram(1, 7), Some(0x00));
}

#[test]
fn vertical_mode_advances_the_page_first() {
    let dev = oled();
    cmd(&dev, 0x20);
    cmd(&dev, 0x01); // vertical
    cmd(&dev, 0x21);
    cmd(&dev, 10);
    cmd(&dev, 11);
    cmd(&dev, 0x22);
    cmd(&dev, 0);
    cmd(&dev, 1);

    for i in 0..4u8 {
        data(&dev, 0xb0 + i);
    }
    // Page first: (0,10), (1,10), then the column moves.
    assert_eq!(dev.gddram(0, 10), Some(0xb0));
    assert_eq!(dev.gddram(1, 10), Some(0xb1));
    assert_eq!(dev.gddram(0, 11), Some(0xb2));
    assert_eq!(dev.gddram(1, 11), Some(0xb3));
    // And at the end of both ranges, back to the start of the window.
    assert_eq!(dev.registers().page, 0);
    assert_eq!(dev.registers().column, 10);
    data(&dev, 0xb4);
    assert_eq!(
        dev.gddram(0, 10),
        Some(0xb4),
        "the fifth byte rewrote (0, 10)"
    );
}

// ---------------------------------------------------------------------------
// Output mapping
// ---------------------------------------------------------------------------

/// Put one lit dot at RAM `(page, column)`, bit `bit`.
fn dot(dev: &Ssd1306, page: u8, column: u8, bit: u8) {
    cmd(dev, 0xb0 | (page & 7));
    cmd(dev, column & 0x0f);
    cmd(dev, 0x10 | (column >> 4));
    data(dev, 1 << bit);
}

#[test]
fn segment_remap_mirrors_x_and_com_scan_direction_mirrors_y() {
    let dev = oled();
    cmd(&dev, 0xaf); // §10.1.18: the panel is off out of reset
    dot(&dev, 0, 3, 2); // RAM row 2, RAM column 3

    // §10.1.11 `A0h` and §10.1.17 `C0h` are both the reset state: the die's own
    // order, column 0 on SEG0 and COM0 scanned first.
    assert!(lit(&dev, 3, 2));

    cmd(&dev, 0xa1); // column 127 is SEG0
    assert!(!lit(&dev, 3, 2));
    assert!(lit(&dev, 124, 2), "127 - 3");

    cmd(&dev, 0xc8); // scan from COM[N-1]
    assert!(lit(&dev, 124, 61), "63 - 2");

    cmd(&dev, 0xa0);
    assert!(lit(&dev, 3, 61), "back to the unmirrored column");
    cmd(&dev, 0xc0);
    assert!(lit(&dev, 3, 2));

    // The same two commands on a rotated module cancel the bonding instead —
    // which is why every real initialisation sequence contains both.
    let module = build(&[("mount", Value::from("rotated"))]);
    cmd(&module, 0xaf);
    cmd(&module, 0xa1);
    cmd(&module, 0xc8);
    dot(&module, 0, 3, 2);
    assert!(
        lit(&module, 3, 2),
        "a rotated module with A1h/C8h is upright"
    );
}

#[test]
fn display_start_line_scrolls_the_output_without_touching_ram() {
    let dev = oled();
    cmd(&dev, 0xaf);
    dot(&dev, 0, 0, 0); // RAM row 0
    let ram = dev.contents();

    assert!(lit(&dev, 0, 0));
    // §10.1.14: `40h + n` starts the row counter at RAM row n, so the picture
    // moves *up* by n and wraps within 64 rows.
    cmd(&dev, 0x40 | 1);
    assert!(!lit(&dev, 0, 0));
    assert!(lit(&dev, 0, 63), "RAM row 0 is now the last scanned row");
    cmd(&dev, 0x40 | 63);
    assert!(lit(&dev, 0, 1));

    // §10.1.20: the offset shifts by COM, i.e. the other way.
    cmd(&dev, 0x40);
    cmd(&dev, 0xd3);
    cmd(&dev, 5);
    assert!(lit(&dev, 0, 5), "the picture moved down by five COM lines");

    assert_eq!(dev.contents(), ram, "and GDDRAM never moved");
}

#[test]
fn inverse_mode_and_entire_display_on_change_the_output_not_the_ram() {
    let dev = oled();
    cmd(&dev, 0xaf);
    dot(&dev, 0, 0, 0);
    let ram = dev.contents();
    assert!(lit(&dev, 0, 0));
    assert!(!lit(&dev, 1, 0));

    cmd(&dev, 0xa7); // §10.1.13: inverse
    assert!(!lit(&dev, 0, 0));
    assert!(lit(&dev, 1, 0));
    cmd(&dev, 0xa6);
    assert!(lit(&dev, 0, 0));

    // §10.1.12: "Entire display ON … Output ignores RAM content", so there is
    // nothing left for `A7h` to invert and `A5h` wins outright.
    cmd(&dev, 0xa5);
    assert!(lit(&dev, 0, 0));
    assert!(lit(&dev, 99, 40));
    cmd(&dev, 0xa7);
    assert!(lit(&dev, 99, 40), "A5h ignores RAM, and A7h inverts RAM");
    cmd(&dev, 0xa4);
    assert!(
        lit(&dev, 99, 40),
        "A4h hands it back, and A7h is still inverse"
    );
    cmd(&dev, 0xa6);
    assert!(!lit(&dev, 99, 40), "and now the picture is RAM again");

    assert_eq!(dev.contents(), ram, "none of it touched a byte of GDDRAM");
}

#[test]
fn display_off_is_black_and_on_restores_the_frame() {
    let dev = oled();
    cmd(&dev, 0xaf);
    for i in 0..1024 {
        // Page mode: one page at a time, so this does not depend on `20h`.
        if i % 128 == 0 {
            cmd(&dev, 0xb0 | (i / 128) as u8);
            cmd(&dev, 0x00);
            cmd(&dev, 0x10);
        }
        data(&dev, checkerboard_byte(i));
    }
    let painted = frame_hash(&dev);
    let ram = dev.contents();

    cmd(&dev, 0xae); // §10.1.18: display off
    for y in 0..64 {
        for x in 0..128 {
            assert!(!lit(&dev, x, y), "({x}, {y}) is lit with the display off");
        }
    }
    assert_eq!(dev.contents(), ram, "and GDDRAM survived");

    cmd(&dev, 0xaf);
    assert_eq!(frame_hash(&dev), painted, "the same picture came back");
}

#[test]
fn the_multiplex_ratio_blanks_the_rows_past_it() {
    let dev = oled();
    cmd(&dev, 0xaf);
    dot(&dev, 4, 0, 0); // RAM row 32
    assert!(lit(&dev, 0, 32));

    // §10.1.16: a 128×32 module drives 32 COM lines. Rows past them are not
    // driven at all.
    cmd(&dev, 0xa8);
    cmd(&dev, 31);
    assert!(!lit(&dev, 0, 32));
    assert_eq!(dev.registers().multiplex, 31);

    // "Invalid entries are ignored": the datasheet's range is 16 to 64 lines.
    cmd(&dev, 0xa8);
    cmd(&dev, 3);
    assert_eq!(
        dev.registers().multiplex,
        31,
        "a ratio below 16MUX is ignored"
    );
}

#[test]
fn contrast_scales_a_lit_dot_and_leaves_an_unlit_one_alone() {
    let dev = oled();
    cmd(&dev, 0xaf);
    dot(&dev, 0, 0, 0);
    let mut row = alloc::vec![[0u8; 3]; 128];
    dev.shared.read_row(0, &mut row);
    // §10.1.7: the reset contrast is 7Fh, and it is a segment *drive current*.
    assert_eq!(row[0], [0x7f, 0x7f, 0x7f]);
    assert_eq!(row[1], [0, 0, 0]);

    cmd(&dev, 0x81);
    cmd(&dev, 0xcf);
    dev.shared.read_row(0, &mut row);
    assert_eq!(row[0], [0xcf, 0xcf, 0xcf]);
    assert_eq!(
        row[1],
        [0, 0, 0],
        "an unlit dot draws no current either way"
    );
}

// ---------------------------------------------------------------------------
// The SH1106
// ---------------------------------------------------------------------------

#[test]
fn the_sh1106_shifts_the_visible_window_two_columns() {
    let sh = build(&[("variant", Value::from("sh1106"))]);
    cmd(&sh, 0xaf);
    // 132 columns of RAM, and the 128-dot glass hangs off SEG2..SEG129.
    assert_eq!(sh.variant().ram_columns(), 132);
    assert_eq!(sh.variant().column_offset(), 2);

    dot(&sh, 0, 4, 0);
    // RAM column 4 shows at x = 2, because the glass starts at SEG2. A driver
    // written for an SSD1306 is two pixels out on every column, which is the
    // single most reported bug in this family.
    assert!(lit(&sh, 2, 0));
    assert!(!lit(&sh, 4, 0));

    // And the two columns the glass does not reach hold bytes nobody sees.
    dot(&sh, 0, 0, 0);
    dot(&sh, 0, 1, 0);
    assert_eq!(sh.gddram(0, 0), Some(1));
    assert_eq!(sh.gddram(0, 1), Some(1));
    for x in 0..128 {
        let want = x == 2;
        assert_eq!(lit(&sh, x, 0), want, "column {x} of row 0");
    }
    // 132 columns of RAM exist even though 128 pixels do.
    assert_eq!(sh.gddram(0, 131), Some(0));
    assert_eq!(sh.gddram(0, 132), None);

    let ssd = oled();
    cmd(&ssd, 0xaf);
    dot(&ssd, 0, 0, 0);
    assert!(lit(&ssd, 0, 0), "on an SSD1306 column 0 is the left edge");

    // The SH1106 has no addressing-mode commands at all; `20h` is not in its
    // table, so it is counted rather than obeyed.
    cmd(&sh, 0x20);
    assert_eq!(sh.registers().mode, AddrMode::Page);
    assert!(sh.unlisted_commands() >= 1);
}

// ---------------------------------------------------------------------------
// Transports
// ---------------------------------------------------------------------------

#[test]
fn the_four_wire_spi_transport_reads_the_dc_pin_and_frames_eight_bits() {
    let dev = build(&[("interface", Value::from("spi4"))]);
    let slave = dev.spi_slave();
    assert_eq!(slave.format().bits, 8);
    assert_eq!(slave.format().mode, Mode::Mode0);
    assert_eq!(slave.format().order, BitOrder::MsbFirst);

    dev.set_dc(Level::Low);
    exchange(&*slave, 0xaf); // display on, as a command
    assert!(dev.registers().display_on);

    dev.set_dc(Level::High);
    exchange(&*slave, 0x5a); // the same byte value, now as data
    assert_eq!(dev.gddram(0, 0), Some(0x5a));
    assert_eq!(dev.unlisted_commands(), 0);
}

#[test]
fn the_three_wire_spi_transport_carries_dc_in_the_ninth_bit() {
    let dev = build(&[("interface", Value::from("spi3"))]);
    let slave = dev.spi_slave();
    // §8.1.4: nine bits, the first of them D/C̅.
    assert_eq!(slave.format().bits, 9);

    exchange(&*slave, 0xaf); // bit 8 clear: a command
    assert!(dev.registers().display_on);
    exchange(&*slave, 0x100 | 0x5a); // bit 8 set: data
    assert_eq!(dev.gddram(0, 0), Some(0x5a));
    // And the pin is irrelevant here, which is why the board needs no wire.
    dev.set_dc(Level::High);
    exchange(&*slave, 0xa5);
    assert!(dev.registers().entire_on);
}

#[test]
fn the_i2c_transport_with_control_bytes_lands_in_the_same_ram() {
    let dev = build(&[
        ("interface", Value::from("i2c")),
        ("mount", Value::from("rotated")),
    ]);
    let slave = dev.i2c_slave();
    assert_eq!(dev.address(), Address::Seven(0x3c));

    // §8.1.5.1: the slave address, then a control byte with `Co` clear and
    // `D/C̅` clear — every byte until the STOP is a command.
    assert_eq!(
        slave.address(Address::Seven(0x3c), Direction::Write),
        Ack::Ack
    );
    assert_eq!(slave.write(0x00), Ack::Ack);
    for byte in INIT {
        assert_eq!(slave.write(*byte), Ack::Ack);
    }
    slave.stop();

    // And a whole frame behind one `40h`, which is how every I²C driver does it.
    assert_eq!(
        slave.address(Address::Seven(0x3c), Direction::Write),
        Ack::Ack
    );
    assert_eq!(slave.write(0x40), Ack::Ack);
    for i in 0..1024 {
        assert_eq!(slave.write(checkerboard_byte(i)), Ack::Ack);
    }
    slave.stop();

    // The identical picture the SPI path produced, from the identical bytes.
    let spi = build(&[("mount", Value::from("rotated"))]);
    for byte in INIT {
        cmd(&spi, *byte);
    }
    for i in 0..1024 {
        data(&spi, checkerboard_byte(i));
    }
    assert_eq!(dev.contents(), spi.contents());
    assert_eq!(frame_hash(&dev), frame_hash(&spi));
}

#[test]
fn the_i2c_co_bit_takes_exactly_one_byte_before_another_control_byte() {
    let dev = build(&[("interface", Value::from("i2c"))]);
    let slave = dev.i2c_slave();
    slave.address(Address::Seven(0x3c), Direction::Write);
    // §8.1.5.1: `Co = 1` means "one byte of this type, then another control
    // byte". A stream that got this wrong would read the next control byte as
    // a pixel.
    slave.write(CONTROL_CO); // Co, command
    slave.write(0xaf);
    slave.write(CONTROL_CO | CONTROL_DC); // Co, data
    slave.write(0x3c);
    slave.write(0x00); // back to a command stream
    slave.write(0xa5);
    slave.stop();

    assert!(dev.registers().display_on);
    assert!(dev.registers().entire_on);
    assert_eq!(dev.gddram(0, 0), Some(0x3c));
    assert_eq!(dev.unlisted_commands(), 0);
}

#[test]
fn the_i2c_face_answers_only_its_own_address_and_refuses_a_read() {
    let dev = build(&[
        ("interface", Value::from("i2c")),
        ("address", Value::from(u64::from(I2C_ADDRESS_SA0_HIGH))),
    ]);
    let slave = dev.i2c_slave();
    assert_eq!(
        slave.address(Address::Seven(I2C_ADDRESS_SA0_LOW), Direction::Write),
        Ack::Nack,
        "SA0 is high on this board"
    );
    assert_eq!(
        slave.address(Address::Seven(I2C_ADDRESS_SA0_HIGH), Direction::Write),
        Ack::Ack
    );
    // §8.1.5.2 describes no read sequence and the part cannot drive SDA with
    // GDDRAM, so answering one would be invention.
    assert_eq!(
        slave.address(Address::Seven(I2C_ADDRESS_SA0_HIGH), Direction::Read),
        Ack::Nack
    );
    // A byte arriving with no address phase is refused rather than written.
    slave.stop();
    assert_eq!(slave.write(0x40), Ack::Nack);
}

// ---------------------------------------------------------------------------
// Framing the command stream
// ---------------------------------------------------------------------------

#[test]
fn a_scroll_setup_with_six_parameters_does_not_desynchronise_the_stream() {
    let dev = oled();
    cmd(&dev, 0xaf);
    // §10.2.1: `26h` takes A..F — six bytes, three of them dummies. A model
    // that took five would read `FFh` as a command and the picture after it
    // would be garbage rather than merely wrong.
    for byte in [0x26, 0x00, 0x00, 0x00, 0x07, 0x00, 0xff] {
        cmd(&dev, byte);
    }
    cmd(&dev, 0x2f); // activate
    assert!(dev.registers().scrolling);
    assert_eq!(
        dev.unlisted_commands(),
        0,
        "no parameter was read as a command"
    );

    // §10.2.4: `29h` takes five.
    for byte in [0x29, 0x00, 0x00, 0x00, 0x07, 0x01] {
        cmd(&dev, byte);
    }
    cmd(&dev, 0x2e);
    assert!(!dev.registers().scrolling);
    assert_eq!(dev.unlisted_commands(), 0);

    // And the stream is still in sync: a data byte lands where it should.
    dot(&dev, 2, 9, 3);
    assert_eq!(dev.gddram(2, 9), Some(1 << 3));
    assert_eq!(dev.unlisted_commands(), 0);
}

#[test]
fn a_command_the_datasheet_does_not_list_is_counted_and_ignored() {
    let dev = oled();
    cmd(&dev, 0xe3); // §10.1.24: NOP is listed and does nothing
    assert_eq!(dev.unlisted_commands(), 0);
    cmd(&dev, 0xfe);
    cmd(&dev, 0x34);
    assert_eq!(dev.unlisted_commands(), 2);
    assert_eq!(dev.registers(), Registers::new(), "and nothing else moved");
}

#[test]
fn the_reset_pin_restores_every_register_and_keeps_gddram() {
    let dev = oled();
    cmd(&dev, 0xaf);
    cmd(&dev, 0x81);
    cmd(&dev, 0x10);
    dot(&dev, 1, 1, 1);
    let ram = dev.contents();

    let sink = Device::sink(&dev, pin::RES, &[]).expect("the part has a RES pin");
    assert_eq!(sink.line, pin::RES_LINE);
    sink.sink
        .set_level(WireId::new(1), pin::RES_LINE, Level::Low);

    // §8.5 leaves the display off and every register at its default.
    assert_eq!(dev.registers(), Registers::new());
    assert!(!dev.registers().display_on);
    // But GDDRAM survives: the initialisation sequence every driver ships
    // clears the screen itself, precisely because reset does not.
    assert_eq!(dev.contents(), ram);
}

// ---------------------------------------------------------------------------
// Descriptions this model refuses
// ---------------------------------------------------------------------------

#[test]
fn a_description_that_cannot_be_a_module_is_refused() {
    let bad = |props: &[(&str, Value)]| -> String {
        let mut p = Props::new();
        for (name, value) in props {
            p.insert(*name, value.clone());
        }
        Ssd1306::new(&p)
            .expect_err("this description is not a part")
            .to_string()
    };
    assert!(bad(&[("variant", Value::from("ssd1331"))]).contains("variant"));
    assert!(bad(&[("interface", Value::from("spi5"))]).contains("interface"));
    assert!(bad(&[("mount", Value::from("sideways"))]).contains("mount"));
    // GDDRAM is eight-row pages, so a panel is a whole number of them.
    assert!(bad(&[("height", Value::from(60u64))]).contains("pages"));
    assert!(bad(&[("height", Value::from(96u64))]).contains("row counter"));
    // 132 columns, of which a 128-dot glass shows 128 from SEG2.
    assert!(
        bad(&[
            ("variant", Value::from("sh1106")),
            ("width", Value::from(132u64)),
        ])
        .contains("GDDRAM")
    );
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

/// Everything a controller would write to a snapshot.
fn saved(dev: &Ssd1306) -> alloc::vec::Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("oled", SSD1306_CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("oled", SSD1306_CLASS.name, SSD1306_CLASS.version)
            .unwrap();
        dev.save(&mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

/// Load a controller from what [`saved`] produced.
fn restore(dev: &Ssd1306, bytes: &[u8]) {
    let reader = StateReader::new(bytes).unwrap();
    let chunk = reader
        .load(
            "oled",
            SSD1306_CLASS.name,
            SSD1306_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    dev.load(&mut chunk.reader()).unwrap();
}

#[test]
fn a_snapshot_round_trips_the_framebuffer_and_the_registers() {
    let dev = build(&[("mount", Value::from("rotated"))]);
    for byte in INIT {
        cmd(&dev, *byte);
    }
    for i in 0..1024 {
        data(&dev, checkerboard_byte(i));
    }
    cmd(&dev, 0xd3);
    cmd(&dev, 7);
    // A command halfway through its parameters: a snapshot taken here has to
    // resume rather than read the contrast byte as a command.
    cmd(&dev, 0x81);

    let bytes = saved(&dev);
    let other = build(&[("mount", Value::from("rotated"))]);
    restore(&other, &bytes);

    // **The framebuffer is the point.** `lcd.scanout` saves six registers and
    // no pixels because its pixels belong to the RAM device; this one has no
    // such owner, so the picture is in the chunk or it is gone.
    assert_eq!(other.contents(), dev.contents());
    assert_eq!(other.registers(), dev.registers());
    assert_eq!(other.shared.generation(), dev.shared.generation());
    assert_eq!(frame_hash(&other), frame_hash(&dev));

    // And the half-finished command lands on the restored copy exactly as it
    // does on the original.
    cmd(&dev, 0x2a);
    cmd(&other, 0x2a);
    assert_eq!(other.registers().contrast, 0x2a);
    assert_eq!(other.registers(), dev.registers());
    assert_eq!(frame_hash(&other), frame_hash(&dev));

    // And their chunks are byte-identical, which is the "identical state hash"
    // `CLAUDE.md` asks for without a machine to take one from.
    assert_eq!(saved(&other), saved(&dev));
}

#[test]
fn a_snapshot_of_an_i2c_panel_resumes_mid_transaction() {
    let dev = build(&[("interface", Value::from("i2c"))]);
    let slave = dev.i2c_slave();
    slave.address(Address::Seven(0x3c), Direction::Write);
    slave.write(0x40); // a data stream is open

    let bytes = saved(&dev);
    let other = build(&[("interface", Value::from("i2c"))]);
    restore(&other, &bytes);

    // The restored panel is still inside the data stream: the next byte is a
    // pixel, not a control byte.
    other.i2c_slave().write(0x81);
    assert_eq!(other.gddram(0, 0), Some(0x81));
    assert_eq!(other.registers().contrast, RESET_CONTRAST);
}
