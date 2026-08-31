//! Tests for the ST7272A, against the datasheet section each one cites.

use super::*;

use alloc::string::ToString;
use alloc::vec::Vec;

use crate::bus::spi::{ChipSelect, SpiSlave, exchange};
use crate::core::props::Value;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::Wire;

/// A panel with the datasheet's own geometry and no bus attachment.
fn panel() -> St7272a {
    St7272a::new(&Props::new()).expect("a default ST7272A")
}

/// The 16-bit command word of §7.1 for a write.
fn write_cmd(addr: u8, data: u8) -> u32 {
    (u32::from(addr & 0x7f) << CMD_ADDR_SHIFT) | u32::from(data)
}

/// And for a read.
fn read_cmd(addr: u8) -> u32 {
    CMD_READ | (u32::from(addr & 0x7f) << CMD_ADDR_SHIFT)
}

/// One complete CS frame carrying `word`, through the word-level seam.
///
/// The chip select matters: §7.1(b) says the command is completed at the rising
/// edge of CS, so a bare `exchange` commits nothing.
fn send(panel: &St7272a, word: u32) -> u32 {
    panel.shared.select(true);
    let out = exchange(&*panel.shared, word);
    panel.shared.select(false);
    out
}

/// [`send`] followed by the VSYNC that establishes it (§7.1(c)).
fn command(panel: &St7272a, word: u32) -> u32 {
    let out = send(panel, word);
    panel.latch();
    out
}

// ---------------------------------------------------------------------------
// Framing
// ---------------------------------------------------------------------------

#[test]
fn the_frame_is_the_one_section_7_1_describes() {
    // "Each serial command consists of 16 bits of data which is loaded one bit
    // a time at the rising edge of serial clock SCL" — §7.1(a), with SCL shown
    // idling low, which is CPOL 0 / CPHA 0.
    assert_eq!(FRAME.bits, 16);
    assert_eq!(FRAME.mode, Mode::Mode0);
    assert_eq!(FRAME.order, BitOrder::MsbFirst);
    assert!(!FRAME.mode.cpol());
    assert!(!FRAME.mode.cpha());
    assert!(FRAME.mode.samples_on(crate::core::wire::Level::High));
}

#[test]
fn every_register_section_8_1_lists_starts_at_its_documented_default() {
    let regs = Registers::new();
    // Command table 1 (§8.1).
    assert_eq!(regs.get(0x10), 0x08);
    assert_eq!(regs.get(0x11), 0x40);
    assert_eq!(regs.get(0x12), 0x40);
    assert_eq!(regs.get(0x13), 0x40);
    assert_eq!(regs.get(0x14), 0x40);
    assert_eq!(regs.get(0x15), 0x40);
    assert_eq!(regs.get(0x16), 0x40);
    assert_eq!(regs.get(0x17), 0x2b);
    assert_eq!(regs.get(0x18), 0x0c);
    assert_eq!(regs.get(0x19), 0x6d);
    assert_eq!(regs.get(0x1b), 0x0c);
    assert_eq!(regs.get(0x1c), 0x38);
    // OTP table (§8.1).
    assert_eq!(regs.get(0x01), 0x7f);
    assert_eq!(regs.get(0x02), 0x7f);
    assert_eq!(regs.get(0x03), 0x7f);
    assert_eq!(regs.get(0x04), 0x78);
    assert_eq!(regs.get(0x05), 0x40);
    assert_eq!(regs.get(0x60), 0x00);
    for addr in 0x66..=0x6c {
        assert_eq!(regs.get(addr), 0x03, "{addr:#04x} program-times default");
    }
}

#[test]
fn the_default_of_10h_is_standby_which_firmware_has_to_leave() {
    // 08h is GRB=1, DISP=0 (§8.1's table and §8.2.1), so a panel that nobody
    // configures shows nothing. That is the hardware, and it is the reason
    // writing this register over SPI is visible in a picture.
    let panel = panel();
    assert!(!panel.is_displaying());
    command(&panel, write_cmd(REG_GRB_DISP, 0x09));
    assert!(panel.is_displaying());
}

#[test]
fn a_fixed_bit_reads_back_fixed_however_it_is_written() {
    // §8.1 draws the reserved columns as literal 0s and 1s: 1Bh has bit 3 = 1
    // and only AUTODL writable, 49h has bit 2 = 1.
    let mut regs = Registers::new();
    assert!(regs.set(0x1b, 0xff));
    assert_eq!(regs.get(0x1b), 0x0c, "bit 3 stays 1, bit 2 takes the write");
    assert!(regs.set(0x1b, 0x00));
    assert_eq!(regs.get(0x1b), 0x08);
    assert!(regs.set(0x49, 0x00));
    assert_eq!(regs.get(0x49), 0x04);
    assert!(regs.set(0x40, 0x00));
    assert_eq!(regs.get(0x40), 0x40, "40h bit 6 is a fixed 1");
}

#[test]
fn an_address_the_tables_do_not_list_is_dropped_and_counted() {
    // §8.1, note 3: "Do not use instructions not listed in these tables." It
    // does not say what happens if you do, so nothing is invented — the write
    // is discarded and the panel counts it for a diagnostic.
    let panel = panel();
    assert!(!Registers::is_known(0x0f));
    command(&panel, write_cmd(0x0f, 0xff));
    assert_eq!(panel.unlisted_commands(), 1);
    assert_eq!(panel.active().get(0x0f), 0);
}

// ---------------------------------------------------------------------------
// The VSYNC latch — §7.1(c)
// ---------------------------------------------------------------------------

#[test]
fn a_command_does_not_take_effect_until_vsync() {
    // §7.1(c): "commands are established by the VSYNC signal."
    let panel = panel();
    send(&panel, write_cmd(REG_CONTRAST, 0xff));
    assert_eq!(panel.shadow().get(REG_CONTRAST), 0xff, "SPI wrote it");
    assert_eq!(panel.active().get(REG_CONTRAST), 0x40, "the panel has not");
    panel.latch();
    assert_eq!(panel.active().get(REG_CONTRAST), 0xff);
}

#[test]
fn the_last_command_before_vsync_is_the_one_that_counts() {
    // §7.1(c): "If command is transferred multiple times for the same register,
    // the last command before the VSYNC signal is valid."
    let panel = panel();
    for value in [0x10u8, 0x20, 0x30] {
        send(&panel, write_cmd(REG_BRIGHTNESS, value));
    }
    panel.latch();
    assert_eq!(panel.active().get(REG_BRIGHTNESS), 0x30);
}

#[test]
fn the_latch_falls_on_a_frame_boundary_of_the_panels_own_clock() {
    // §7.3.4's typicals: Th = 371 DCLK, Tv = 260 HSYNC, so one frame is
    // 371 x 260 = 96,460 DCLK ticks.
    let panel = panel();
    assert_eq!(panel.frame_ticks(), DEFAULT_HTOTAL * DEFAULT_VTOTAL);
    send(&panel, write_cmd(REG_CONTRAST, 0x80));

    panel.advance_to(panel.frame_ticks() - 1);
    assert_eq!(panel.active().get(REG_CONTRAST), 0x40, "not yet");
    assert_eq!(panel.frames(), 0);
    panel.advance_to(panel.frame_ticks());
    assert_eq!(panel.active().get(REG_CONTRAST), 0x80, "on the boundary");
    assert_eq!(panel.frames(), 1);
}

#[test]
fn the_next_frame_boundary_is_always_ahead_of_the_present() {
    // The scheduler requires this strictly, or catch-up makes no progress and
    // the device stalls where it stands (`core::device`).
    let panel = panel();
    for tick in [0u64, 1, 96_459, 96_460, 96_461, 500_000] {
        panel.advance_to(tick);
        let next = Device::next_event_tick(&panel).expect("a panel always has a next frame");
        assert!(
            next > Device::current_tick(&panel),
            "at {tick}: {next} must be past the present"
        );
    }
}

// ---------------------------------------------------------------------------
// Reset — §8.2.1 and §6.1
// ---------------------------------------------------------------------------

#[test]
fn writing_grb_low_resets_every_register_at_once() {
    // §8.2.1: "GRB=0: reset all registers to default value", and §8.1's note 1
    // under each table. It is a reset, not a command, so it does not wait for
    // VSYNC.
    let panel = panel();
    command(&panel, write_cmd(REG_CONTRAST, 0xff));
    command(&panel, write_cmd(REG_BRIGHTNESS, 0xff));
    assert_eq!(panel.active().get(REG_CONTRAST), 0xff);

    send(&panel, write_cmd(REG_GRB_DISP, 0x00));
    assert_eq!(panel.active().get(REG_CONTRAST), 0x40, "without a VSYNC");
    assert_eq!(panel.active().get(REG_BRIGHTNESS), 0x40);
    assert_eq!(panel.shadow().get(REG_GRB_DISP), 0x08);
}

#[test]
fn the_grb_pin_resets_the_registers_too() {
    // §6.1: "Global reset pin. When GRB is 'L', internal initialization
    // procedure is executed."
    let panel = panel();
    command(&panel, write_cmd(REG_CONTRAST, 0xc0));
    let src = WireId::new(1);
    let pin = panel.sink(pin::GRB, &[src]).expect("a grb input");
    let wire = Wire::builder()
        .source(src)
        .sink(pin.sink, pin.line)
        .build_shared();
    wire.set(src, Level::High);
    assert_eq!(panel.active().get(REG_CONTRAST), 0xc0);
    wire.set(src, Level::Low);
    assert_eq!(panel.active().get(REG_CONTRAST), 0x40);
    assert!(!panel.is_displaying(), "GRB held low drives nothing");
}

#[test]
fn an_unwired_control_pin_sits_at_its_inactive_level() {
    // A fresh net idles low, and DISP low is standby. A panel on a board that
    // ties DISP high must not come up blank because the machine file did not
    // mention the pin.
    let panel = panel();
    command(&panel, write_cmd(REG_GRB_DISP, 0x09));
    assert!(panel.is_displaying(), "nothing is holding DISP or GRB low");
    assert_eq!(panel.bist_colour(), None, "BIST_EN is not wired either");
}

#[test]
fn the_disp_pin_blanks_the_panel_independently_of_the_register() {
    // §6.1: "DISP sets the display mode. L: Standby mode."
    let panel = panel();
    command(&panel, write_cmd(REG_GRB_DISP, 0x09));
    let src = WireId::new(1);
    let disp = panel.sink(pin::DISP, &[src]).expect("a disp input");
    let wire = Wire::builder()
        .source(src)
        .sink(disp.sink, disp.line)
        .build_shared();
    wire.set(src, Level::High);
    assert!(panel.is_displaying());
    wire.set(src, Level::Low);
    assert!(!panel.is_displaying());
    assert_eq!(panel.apply([200, 200, 200]), [0, 0, 0], "standby is black");
}

// ---------------------------------------------------------------------------
// The picture — §8.2.2 to §8.2.10, §12.1
// ---------------------------------------------------------------------------

/// A panel out of standby with default colour registers.
fn lit() -> St7272a {
    let panel = panel();
    command(&panel, write_cmd(REG_GRB_DISP, 0x09));
    panel
}

#[test]
fn the_default_colour_registers_pass_a_pixel_through_unchanged() {
    // CONTRAST 40h is "contrast gain=1", the sub-contrasts 40h are gain 1, and
    // the brightnesses 40h are 0 (§8.2.2-§8.2.7). Reset should therefore be a
    // pass-through, and if it is not, one of the six curves is wrong.
    let panel = lit();
    for value in [0u8, 1, 37, 128, 200, 254, 255] {
        let rgb = [value, value, value];
        assert_eq!(panel.apply(rgb), rgb, "{value} should pass through");
    }
    assert_eq!(panel.apply([12, 34, 56]), [12, 34, 56]);
}

#[test]
fn contrast_is_a_gain_in_sixty_fourths() {
    // §8.2.2: 00h is gain 0, 40h is gain 1, FFh is gain 3.984 = 255/64.
    let panel = lit();
    command(&panel, write_cmd(REG_CONTRAST, 0x00));
    assert_eq!(panel.apply([255, 255, 255]), [0, 0, 0], "gain 0 is black");
    command(&panel, write_cmd(REG_CONTRAST, 0x20));
    assert_eq!(panel.apply([100, 100, 100]), [50, 50, 50], "gain 1/2");
    command(&panel, write_cmd(REG_CONTRAST, 0xff));
    assert_eq!(
        panel.apply([100, 100, 100]),
        [255, 255, 255],
        "and it clamps"
    );
}

#[test]
fn brightness_is_an_offset_from_minus_sixty_four() {
    // §8.2.5: 00h is -64, 40h is 0, FFh is +191.
    let panel = lit();
    command(&panel, write_cmd(REG_BRIGHTNESS, 0x00));
    assert_eq!(panel.apply([100, 100, 100]), [36, 36, 36]);
    assert_eq!(
        panel.apply([10, 10, 10]),
        [0, 0, 0],
        "and it clamps at black"
    );
    command(&panel, write_cmd(REG_BRIGHTNESS, 0x60));
    assert_eq!(panel.apply([100, 100, 100]), [132, 132, 132]);
}

#[test]
fn gain_is_applied_before_offset_so_zero_contrast_is_black() {
    // The datasheet's block diagram (§5) does not order the two stages, so this
    // is a design decision written down: §8.2.2 calls CONTRAST=00h "contrast
    // gain=0", and a gain of zero that still leaves a grey would not be one.
    let panel = lit();
    command(&panel, write_cmd(REG_CONTRAST, 0x00));
    command(&panel, write_cmd(REG_BRIGHTNESS, 0x40));
    assert_eq!(panel.apply([255, 255, 255]), [0, 0, 0]);
}

#[test]
fn the_sub_contrast_curve_hits_its_three_documented_points() {
    // §8.2.3: 00h is 0.75, 40h is 1, 7Fh is 1.246. In 1/1024ths those are 768,
    // 1024 and 1276 — and 1276/1024 is 1.2461, which is the rounding the
    // datasheet itself printed.
    assert_eq!(sub_contrast_q10(0x00), 768);
    assert_eq!(sub_contrast_q10(0x40), 1024);
    assert_eq!(sub_contrast_q10(0x7f), 1276);
}

#[test]
fn the_sub_channels_move_red_and_blue_without_touching_green() {
    // §8.2.3, §8.2.4, §8.2.6, §8.2.7 — every one of them names red or blue, and
    // there is no green sub-anything, which is the check that matters.
    let panel = lit();
    command(&panel, write_cmd(REG_SUB_BRIGHTNESS_R, 0x50)); // +16
    command(&panel, write_cmd(REG_SUB_BRIGHTNESS_B, 0x30)); // -16
    assert_eq!(panel.apply([100, 100, 100]), [116, 100, 84]);

    let panel = lit();
    command(&panel, write_cmd(REG_SUB_CONTRAST_R, 0x00)); // gain 0.75
    assert_eq!(panel.apply([100, 100, 100]), [75, 100, 100]);
}

#[test]
fn sbgr_exchanges_red_and_blue() {
    // §8.2.10: "SBGR= 1: exchange, DR[7:0]->DB[7:0] and DB[7:0]->DR[7:0]".
    let panel = lit();
    assert_eq!(panel.apply([10, 20, 30]), [10, 20, 30]);
    // 19h defaults to 6Dh; set bit 4 and keep the rest.
    command(&panel, write_cmd(REG_DISPLAY_MODE, 0x7d));
    assert!(panel.active().swap_rb());
    assert_eq!(panel.apply([10, 20, 30]), [30, 20, 10]);
}

#[test]
fn the_scan_direction_bits_mirror_the_picture() {
    // §8.2.10: HDIR=1 is "from left to right", VDIR=1 is "from top to bottom",
    // and 6Dh has both set, so the reset state is the identity.
    let panel = lit();
    assert_eq!(panel.map_pixel(0, 0), (0, 0));
    assert_eq!(panel.map_pixel(319, 239), (319, 239));
    // Clear HDIR (bit 5) and VDIR (bit 6): 6Dh & !0x60 = 0x0d.
    command(&panel, write_cmd(REG_DISPLAY_MODE, 0x0d));
    assert_eq!(panel.map_pixel(0, 0), (319, 239));
    assert_eq!(panel.map_pixel(319, 239), (0, 0));
}

#[test]
fn bist_drives_the_flat_patterns_of_section_12() {
    // §8.2.12's PICSEL table: 000 black, 001 white, 010 red, 011 green,
    // 100 blue, and 101/110/111 all black.
    let panel = lit();
    let src = WireId::new(1);
    let bist = panel.sink(pin::BIST_EN, &[src]).expect("a bist_en input");
    let wire = Wire::builder()
        .source(src)
        .sink(bist.sink, bist.line)
        .build_shared();
    wire.set(src, Level::High);

    let expected = [
        (0b000u8, [0x00, 0x00, 0x00]),
        (0b001, [0xff, 0xff, 0xff]),
        (0b010, [0xff, 0x00, 0x00]),
        (0b011, [0x00, 0xff, 0x00]),
        (0b100, [0x00, 0x00, 0xff]),
        (0b101, [0x00, 0x00, 0x00]),
        (0b110, [0x00, 0x00, 0x00]),
        (0b111, [0x00, 0x00, 0x00]),
    ];
    for (picsel, colour) in expected {
        command(&panel, write_cmd(REG_BIST, 0x38 | picsel));
        assert_eq!(panel.bist_colour(), Some(colour), "PICSEL {picsel:#05b}");
        assert_eq!(
            panel.apply([1, 2, 3]),
            colour,
            "BIST replaces the RGB input, PICSEL {picsel:#05b}"
        );
    }

    wire.set(src, Level::Low);
    assert_eq!(panel.bist_colour(), None);
    assert_eq!(panel.apply([1, 2, 3]), [1, 2, 3]);
}

// ---------------------------------------------------------------------------
// Reading over SPI — §7.1's Read Mode
// ---------------------------------------------------------------------------

#[test]
fn a_read_frame_answers_in_its_own_low_byte() {
    // §7.1's Read Mode diagram: the master drives R/W and A6..A0, the panel
    // drives D7..D0 of the *same* sixteen-bit frame.
    let panel = panel();
    command(&panel, write_cmd(0x46, 0xa5));
    let word = send(&panel, read_cmd(0x46));
    assert_eq!(word & 0xff, 0xa5);
    assert_eq!(word >> 8, 0xff, "the master's half reads as the pull-up");
}

#[test]
fn a_read_frame_writes_nothing() {
    let panel = panel();
    command(&panel, write_cmd(REG_CONTRAST, 0x77));
    // A read of 11h whose low byte happens to hold a plausible value must not
    // land in the register.
    send(&panel, read_cmd(REG_CONTRAST) | 0x12);
    panel.latch();
    assert_eq!(panel.active().get(REG_CONTRAST), 0x77);
}

// ---------------------------------------------------------------------------
// On the wire, both ways
// ---------------------------------------------------------------------------

/// Clock one 16-bit frame into the panel's pins, MSB first, mode 0 — a GPIO
/// controller's job, done by hand.
fn bitbang(panel: &St7272a, word: u32) -> u32 {
    let pins = panel.pins();
    pins.drive(spi_pin::SCK, Level::Low);
    pins.drive(spi_pin::CS, Level::Low);
    let mut miso = 0u32;
    for bit in (0..16).rev() {
        pins.drive(spi_pin::MOSI, Level::from_bool(word >> bit & 1 != 0));
        if pins.miso_level().is_high() {
            miso |= 1 << bit;
        }
        pins.drive(spi_pin::SCK, Level::High);
        pins.drive(spi_pin::SCK, Level::Low);
    }
    pins.drive(spi_pin::CS, Level::High);
    miso
}

#[test]
fn a_bit_banged_frame_reaches_the_registers() {
    let panel = panel();
    bitbang(&panel, write_cmd(REG_GRB_DISP, 0x09));
    bitbang(&panel, write_cmd(REG_CONTRAST, 0x30));
    panel.latch();
    assert!(panel.is_displaying());
    assert_eq!(panel.active().get(REG_CONTRAST), 0x30);
}

#[test]
fn a_bit_banged_read_returns_the_same_word_a_transaction_does() {
    // The equivalence claim, at the device rather than the bus: the mid-word
    // turnaround of §7.1 has to look the same whether the frame arrived as one
    // call or as thirty-two edges.
    let panel = panel();
    bitbang(&panel, write_cmd(0x46, 0x5c));
    panel.latch();
    let banged = bitbang(&panel, read_cmd(0x46));
    let transacted = send(&panel, read_cmd(0x46));
    assert_eq!(banged & 0xff, 0x5c);
    assert_eq!(banged, transacted);
}

#[test]
fn a_short_frame_is_ignored() {
    // §7.1(d): "If less than 16 bits of SCL are input while CS is low, the
    // transferred data is ignored."
    let panel = panel();
    let pins = panel.pins();
    pins.drive(spi_pin::SCK, Level::Low);
    pins.drive(spi_pin::CS, Level::Low);
    let word = write_cmd(REG_CONTRAST, 0xff);
    for bit in (5..16).rev() {
        pins.drive(spi_pin::MOSI, Level::from_bool(word >> bit & 1 != 0));
        pins.drive(spi_pin::SCK, Level::High);
        pins.drive(spi_pin::SCK, Level::Low);
    }
    pins.drive(spi_pin::CS, Level::High);
    panel.latch();
    assert_eq!(panel.active().get(REG_CONTRAST), 0x40, "still the default");
}

#[test]
fn the_panel_attaches_to_a_named_bus_at_the_chip_select_it_is_given() {
    let name = "test-st7272a-attach";
    buses::close(name);
    let panel = St7272a::new(
        &Props::new()
            .with("bus", Value::Str(name.to_string()))
            .with("cs", Value::Uint(3)),
    )
    .expect("a panel on a named bus");
    let bus = buses::open(name);
    assert_eq!(bus.attached(), alloc::vec![ChipSelect(3)]);
    assert_eq!(bus.check_format(FRAME), None, "and it wants §7.1's framing");

    // And traffic through the bus reaches it.
    bus.select(Some(ChipSelect(3)));
    bus.transfer(write_cmd(REG_GRB_DISP, 0x09));
    bus.select(None);
    panel.latch();
    assert!(panel.is_displaying());
    buses::close(name);
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

#[test]
fn geometry_is_a_property_with_the_datasheets_part_as_the_default() {
    let panel = panel();
    assert_eq!(panel.size(), (320, 240));
    let other = St7272a::new(
        &Props::new()
            .with("width", Value::Uint(480))
            .with("height", Value::Uint(320))
            .with("htotal", Value::Uint(560))
            .with("vtotal", Value::Uint(340)),
    )
    .expect("a panel of another size");
    assert_eq!(other.size(), (480, 320));
    assert_eq!(other.frame_ticks(), 560 * 340);
}

#[test]
fn a_period_shorter_than_the_visible_area_is_refused() {
    let err = St7272a::new(&Props::new().with("htotal", Value::Uint(100)))
        .unwrap_err()
        .to_string();
    assert!(err.contains("blanking"), "{err}");
}

#[test]
fn a_zero_dimension_is_refused() {
    let err = St7272a::new(&Props::new().with("width", Value::Uint(0)))
        .unwrap_err()
        .to_string();
    assert!(err.contains("at least 1"), "{err}");
}

#[test]
fn the_panel_drives_only_miso() {
    let panel = panel();
    let err = panel
        .connect(
            "sck",
            crate::core::wire::WireSource::new(
                Wire::builder().source(WireId::new(1)).build_shared(),
                WireId::new(1),
            ),
        )
        .unwrap_err()
        .to_string();
    assert!(err.contains("miso"), "{err}");
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

#[test]
fn the_panel_round_trips_through_a_snapshot() {
    let panel = panel();
    command(&panel, write_cmd(REG_GRB_DISP, 0x09));
    command(&panel, write_cmd(REG_CONTRAST, 0x91));
    command(&panel, write_cmd(REG_DISPLAY_MODE, 0x7d));
    command(&panel, write_cmd(0x24, 0xab));
    // A write that has not been latched yet: both banks have to survive, or a
    // save taken between a command and its VSYNC loses it.
    send(&panel, write_cmd(REG_BRIGHTNESS, 0x22));
    panel.advance_to(150_000);

    let mut shape = MachineShape::new();
    shape.add_device("panel", ST7272A_CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("panel", ST7272A_CLASS.name, ST7272A_CLASS.version)
            .unwrap();
        panel.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let other = panel_like(&panel);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load(
            "panel",
            ST7272A_CLASS.name,
            ST7272A_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    other.load(&mut chunk.reader()).unwrap();

    assert_eq!(other.shadow(), panel.shadow());
    assert_eq!(other.active(), panel.active());
    assert_eq!(other.frames(), panel.frames());
    assert_eq!(Device::current_tick(&other), Device::current_tick(&panel));
    assert_eq!(other.unlisted_commands(), panel.unlisted_commands());
    assert_eq!(state_hash(&other), state_hash(&panel));

    // And the un-latched write still arrives on the next frame, on the restored
    // copy as on the original.
    other.advance_to(200_000);
    panel.advance_to(200_000);
    assert_eq!(other.active().get(REG_BRIGHTNESS), 0x22);
    assert_eq!(state_hash(&other), state_hash(&panel));
}

/// Everything a panel would write to a snapshot.
fn saved(panel: &St7272a) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("panel", ST7272A_CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("panel", ST7272A_CLASS.name, ST7272A_CLASS.version)
            .unwrap();
        panel.save(&mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

/// Load a panel from what [`saved`] produced.
fn restore(panel: &St7272a, bytes: &[u8]) {
    let reader = StateReader::new(bytes).unwrap();
    let chunk = reader
        .load(
            "panel",
            ST7272A_CLASS.name,
            ST7272A_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    panel.load(&mut chunk.reader()).unwrap();
}

/// A second panel of the same shape as `like`.
fn panel_like(like: &St7272a) -> St7272a {
    let (w, h) = like.size();
    St7272a::new(
        &Props::new()
            .with("width", Value::Uint(u64::from(w)))
            .with("height", Value::Uint(u64::from(h))),
    )
    .expect("a matching panel")
}

/// FNV-1a over a panel's saved chunk — the same comparison `Machine::state_hash`
/// makes, done on one device.
fn state_hash(panel: &St7272a) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in &saved(panel) {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

#[test]
fn a_snapshot_taken_mid_command_resumes_mid_command() {
    // The bit-level shift register is architectural state too: a save between
    // the eighth and ninth SCL edges has to come back with eight bits pending,
    // not with a fresh frame.
    let panel = panel();
    let pins = panel.pins();
    pins.drive(spi_pin::SCK, Level::Low);
    pins.drive(spi_pin::CS, Level::Low);
    let word = write_cmd(REG_CONTRAST, 0x5a);
    for bit in (8..16).rev() {
        pins.drive(spi_pin::MOSI, Level::from_bool(word >> bit & 1 != 0));
        pins.drive(spi_pin::SCK, Level::High);
        pins.drive(spi_pin::SCK, Level::Low);
    }

    let bytes = saved(&panel);
    let other = panel_like(&panel);
    restore(&other, &bytes);

    // Finish the frame on the *restored* panel.
    let pins = other.pins();
    for bit in (0..8).rev() {
        pins.drive(spi_pin::MOSI, Level::from_bool(word >> bit & 1 != 0));
        pins.drive(spi_pin::SCK, Level::High);
        pins.drive(spi_pin::SCK, Level::Low);
    }
    pins.drive(spi_pin::CS, Level::High);
    other.latch();
    assert_eq!(other.active().get(REG_CONTRAST), 0x5a);
}

#[test]
fn a_cold_reset_returns_the_registers_to_their_defaults() {
    let panel = panel();
    command(&panel, write_cmd(REG_CONTRAST, 0xff));
    command(&panel, write_cmd(REG_GRB_DISP, 0x09));
    panel.reset(ResetKind::Cold);
    assert_eq!(panel.active(), Registers::new());
    assert_eq!(panel.shadow(), Registers::new());
    assert!(!panel.is_displaying());
}

#[test]
fn only_the_last_whole_word_of_a_long_frame_is_valid() {
    // §7.1(e): "If 16 bits or more of SCL are input while CS is low, the
    // previous 16 bits of transferred data before the rising edge of CS pulse
    // are valid data." Three words in one CS frame commit one command.
    let panel = panel();
    panel.shared.select(true);
    exchange(&*panel.shared, write_cmd(REG_CONTRAST, 0x11));
    exchange(&*panel.shared, write_cmd(REG_BRIGHTNESS, 0x22));
    exchange(&*panel.shared, write_cmd(REG_CONTRAST, 0x33));
    panel.shared.select(false);
    panel.latch();

    assert_eq!(panel.active().get(REG_CONTRAST), 0x33, "the last word");
    assert_eq!(
        panel.active().get(REG_BRIGHTNESS),
        0x40,
        "the one in the middle never happened"
    );
}

#[test]
fn a_frame_that_never_ends_commits_nothing() {
    // The command completes "at the next rising edge of CS" (§7.1(b)), so a
    // controller that forgets to deassert leaves the panel exactly as it was.
    let panel = panel();
    panel.shared.select(true);
    exchange(&*panel.shared, write_cmd(REG_CONTRAST, 0x77));
    panel.latch();
    assert_eq!(panel.active().get(REG_CONTRAST), 0x40);
    assert_eq!(panel.shadow().get(REG_CONTRAST), 0x40);
}
