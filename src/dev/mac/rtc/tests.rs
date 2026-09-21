//! The clock chip's own tests: the frame shape a trace of a real ROM showed,
//! asserted one rule at a time.

use super::*;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use alloc::vec::Vec;

/// Build a command byte the way the trace says they are built: the direction in
/// bit 7, the address in bits 6-2, `01` below.
fn command(read: bool, addr: u8) -> u8 {
    (u8::from(read) << 7) | ((addr & 0x1f) << 2) | 0x01
}

/// Clock `byte` into the chip, most significant bit first, the way the
/// processor does: set the data line, raise the clock, lower it.
fn put(rtc: &Rtc, byte: u8) {
    for bit in (0..8).rev() {
        rtc.set_data(byte & (1 << bit) != 0);
        rtc.set_clk(true);
        rtc.set_clk(false);
    }
}

/// Take a byte out, reading the line while the clock is high.
fn get(rtc: &Rtc) -> u8 {
    let mut byte = 0u8;
    for _ in 0..8 {
        rtc.set_clk(true);
        byte = (byte << 1) | u8::from(!rtc.data_low());
        rtc.set_clk(false);
    }
    byte
}

/// One whole frame: enable, command, data, disable.
fn read_reg(rtc: &Rtc, addr: u8) -> u8 {
    rtc.set_enb(true);
    put(rtc, command(true, addr));
    let value = get(rtc);
    rtc.set_enb(false);
    value
}

fn write_reg(rtc: &Rtc, addr: u8, value: u8) {
    rtc.set_enb(true);
    put(rtc, command(false, addr));
    put(rtc, value);
    rtc.set_enb(false);
}

/// The trace's own arithmetic: the sixteen read commands really are four apart
/// and really do name `$10`-`$1F`.
#[test]
fn the_command_byte_is_a_direction_an_address_and_a_constant() {
    assert_eq!(command(true, 0x10), 0xc1);
    assert_eq!(command(true, 0x1f), 0xfd);
    assert_eq!(command(true, 0x08), 0xa1);
    assert_eq!(command(true, 0x00), 0x81);
    assert_eq!(command(false, 0x10), 0x41);
    assert_eq!(command(false, ADDR_TEST), 0x31);
    assert_eq!(command(false, ADDR_WRITE_PROTECT), 0x35);
    // And every one of them is four apart from the next, which is what made
    // the address field legible in the first place.
    for addr in 0..0x1fu8 {
        assert_eq!(command(true, addr + 1) - command(true, addr), 4);
    }
}

/// A flat battery reads `$FF` everywhere, which is what the traced ROM found
/// and why it wrote its own defaults over the top.
#[test]
fn a_flat_battery_reads_as_its_pull_ups() {
    let rtc = Rtc::bare();
    for addr in 0x10..=0x1f {
        assert_eq!(read_reg(&rtc, addr), 0xff, "parameter RAM ${addr:02x}");
    }
    for addr in 0x08..=0x0b {
        assert_eq!(read_reg(&rtc, addr), 0xff, "parameter RAM ${addr:02x}");
    }
}

/// Twenty bytes, and they are the twenty the addresses reach.
#[test]
fn parameter_ram_is_twenty_bytes_in_two_groups() {
    let rtc = Rtc::bare();
    write_reg(&rtc, ADDR_WRITE_PROTECT, 0x55);
    for (n, addr) in (0x08..=0x0b).chain(0x10..=0x1f).enumerate() {
        write_reg(&rtc, addr, n as u8);
    }
    write_reg(&rtc, ADDR_WRITE_PROTECT, 0xd5);
    for (n, addr) in (0x08..=0x0b).chain(0x10..=0x1f).enumerate() {
        assert_eq!(read_reg(&rtc, addr), n as u8, "parameter RAM ${addr:02x}");
    }
    let pram = rtc.pram();
    assert_eq!(pram.len(), PRAM_BYTES);
    assert_eq!(&pram[..4], &[0, 1, 2, 3]);
    assert_eq!(pram[19], 19);
}

/// The write-protect register is what brackets every write the ROM makes, and
/// it really does refuse them.
#[test]
fn write_protection_refuses_the_counter_and_parameter_ram() {
    let rtc = Rtc::bare();
    assert!(rtc.write_protected(), "the chip comes up protected");
    write_reg(&rtc, 0x10, 0x5a);
    assert_eq!(read_reg(&rtc, 0x10), 0xff, "the write was refused");
    write_reg(&rtc, 0x00, 0x5a);
    assert_eq!(rtc.seconds(), 0, "and so was the counter's");

    // `$55` is what the traced ROM sends to unlock.
    write_reg(&rtc, ADDR_WRITE_PROTECT, 0x55);
    assert!(!rtc.write_protected());
    write_reg(&rtc, 0x10, 0x5a);
    assert_eq!(read_reg(&rtc, 0x10), 0x5a);

    // `$D5` is what it sends to lock again: bit 7 is the one that matters.
    write_reg(&rtc, ADDR_WRITE_PROTECT, 0xd5);
    assert!(rtc.write_protected());
    write_reg(&rtc, 0x10, 0x00);
    assert_eq!(read_reg(&rtc, 0x10), 0x5a, "protected again");
}

/// The counter answers at `$00`-`$03` and again at `$04`-`$07`, least
/// significant byte first — which is what lets a ROM read it twice and compare.
#[test]
fn the_second_counter_answers_twice_over_least_significant_first() {
    let rtc = Rtc::at(0x1234_5678, [0xff; PRAM_BYTES]);
    assert_eq!(read_reg(&rtc, 0x00), 0x78);
    assert_eq!(read_reg(&rtc, 0x01), 0x56);
    assert_eq!(read_reg(&rtc, 0x02), 0x34);
    assert_eq!(read_reg(&rtc, 0x03), 0x12);
    for addr in 0x04..=0x07 {
        assert_eq!(read_reg(&rtc, addr), read_reg(&rtc, addr - 4));
    }
}

/// One second of the watch crystal is one second on the counter, and one edge
/// on the interrupt line for the VIA to latch.
#[test]
fn the_counter_takes_a_second_a_second_and_pulses_the_line() {
    let rtc = Rtc::at(1_000, [0xff; PRAM_BYTES]);
    assert_eq!(rtc.seconds(), 1_000);
    // Half a second: the line falls, and the counter takes its second there.
    rtc.advance_to(TICKS_PER_SECOND / 2);
    assert!(rtc.irq_low(), "the one-second output fell");
    assert_eq!(rtc.seconds(), 1_001);
    // And the other half brings it back up without counting again.
    rtc.advance_to(TICKS_PER_SECOND);
    assert!(!rtc.irq_low());
    assert_eq!(rtc.seconds(), 1_001);

    rtc.advance_to(TICKS_PER_SECOND * 61);
    assert_eq!(rtc.seconds(), 1_061, "a minute later");
}

/// The test register is a register: it takes a write and nothing else happens.
#[test]
fn the_test_register_is_written_and_ignored() {
    let rtc = Rtc::bare();
    write_reg(&rtc, ADDR_TEST, 0x00);
    assert_eq!(rtc.shared.state.lock().test, 0x00);
    write_reg(&rtc, ADDR_TEST, 0xa5);
    assert_eq!(rtc.shared.state.lock().test, 0xa5);
    // It is write-only, like the write-protect register.
    assert_eq!(read_reg(&rtc, ADDR_TEST), 0xff);
    assert_eq!(read_reg(&rtc, ADDR_WRITE_PROTECT), 0xff);
}

/// Raising the enable abandons whatever was in progress, which is what makes
/// a frame a frame.
#[test]
fn raising_the_enable_abandons_the_frame() {
    let rtc = Rtc::bare();
    write_reg(&rtc, ADDR_WRITE_PROTECT, 0x55);
    rtc.set_enb(true);
    put(&rtc, command(false, 0x10));
    // Four bits of the data byte, and then the processor changes its mind.
    for _ in 0..4 {
        rtc.set_data(true);
        rtc.set_clk(true);
        rtc.set_clk(false);
    }
    rtc.set_enb(false);
    assert_eq!(read_reg(&rtc, 0x10), 0xff, "the half-written byte went");
    assert!(!rtc.data_low(), "and the data line was let go");
}

/// Dates go in as text and come out as the counter the chip keeps.
#[test]
fn a_date_becomes_seconds_since_1904() {
    // The epoch itself.
    assert_eq!(parse_time("1904-01-01T00:00:00").unwrap(), 0);
    assert_eq!(parse_time("1904-01-01T00:00:01").unwrap(), 1);
    assert_eq!(parse_time("1904-01-02T00:00:00").unwrap(), 86_400);
    // 1904 is a leap year, so the first 29 February is 59 days in.
    assert_eq!(parse_time("1904-02-29T00:00:00").unwrap(), 59 * 86_400);
    // And the Unix epoch is the constant this file names.
    assert_eq!(unix_of(parse_time("1970-01-01T00:00:00").unwrap()), 0);
    assert_eq!(
        unix_of(parse_time("2026-01-01T00:00:00").unwrap()),
        1_767_225_600
    );

    for bad in [
        "",
        "2026-01-01",
        "2026-13-01T00:00:00",
        "2026-02-30T00:00:00",
        "1899-01-01T00:00:00",
        "2026-01-01T24:00:00",
        "2026-01-01T00:60:00",
        "xxxx-01-01T00:00:00",
    ] {
        assert!(parse_time(bad).is_err(), "{bad} should not parse");
    }
}

/// Properties are checked, including the hexadecimal parameter RAM.
#[test]
fn properties_are_checked() {
    use crate::core::props::Value;
    let with = |k: &str, v: &str| Props::new().with(k, Value::from(v));
    assert!(Rtc::new(&Props::new()).is_ok());
    assert!(Rtc::new(&with("time", "2000-06-15T12:30:45")).is_ok());
    assert!(Rtc::new(&with("time", "not a date")).is_err());
    assert!(Rtc::new(&with("pram", "a8000000cc0acc0a000000000002630000038800")).is_ok());
    assert!(Rtc::new(&with("pram", "a800")).is_err());
    assert!(Rtc::new(&with("pram", "zz".repeat(PRAM_BYTES).as_str())).is_err());
    assert!(Rtc::new(&with("nonsense", "1")).is_err());

    let rtc = Rtc::new(&with("pram", "000102030405060708090a0b0c0d0e0f10111213")).unwrap();
    assert_eq!(rtc.pram()[0], 0x00);
    assert_eq!(rtc.pram()[19], 0x13);
}

/// Invariant 6: `save` and `load` agree, and the restored chip is the same one
/// bit for bit — mid-frame included.
#[test]
fn a_snapshot_round_trips_to_an_identical_state_hash() {
    let saved = Rtc::at(0x1234_5678, [0xff; PRAM_BYTES]);
    write_reg(&saved, ADDR_WRITE_PROTECT, 0x55);
    write_reg(&saved, 0x11, 0xa5);
    saved.advance_to(TICKS_PER_SECOND * 3 + 7);
    // Stop with a command byte half in.
    saved.set_enb(true);
    for _ in 0..3 {
        saved.set_data(true);
        saved.set_clk(true);
        saved.set_clk(false);
    }

    let image = |rtc: &Rtc| -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("rtc", CLASS_NAME).expect("a fresh shape");
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("rtc", CLASS_NAME, STATE_VERSION).expect("a chunk");
            Device::save(rtc, &mut chunk).expect("a clock chip saves");
        }
        w.to_vec().expect("a complete image")
    };
    let first = image(&saved);

    let restored = Rtc::bare();
    let reader = StateReader::new(&first).expect("a well-formed image");
    let chunk = reader
        .load("rtc", CLASS_NAME, STATE_VERSION, &Migrations::new())
        .expect("the chunk is there");
    Device::load(&restored, &mut chunk.reader()).expect("a clock chip loads");
    assert_eq!(image(&restored), first, "the same chip, bit for bit");

    // Run both on and they stay identical: the counter was restored running.
    saved.advance_to(TICKS_PER_SECOND * 20);
    restored.advance_to(TICKS_PER_SECOND * 20);
    assert_eq!(image(&restored), image(&saved));
    assert_eq!(restored.seconds(), saved.seconds());
}

/// The class registers, describes itself, and the schema matches the pins the
/// device really answers on.
#[test]
fn the_class_is_registrable_and_its_schema_matches() {
    let mut registry = crate::core::Registry::new();
    register(&mut registry).expect("a fresh registry");
    assert!(register(&mut registry).is_err(), "twice is an error");

    let rtc = Rtc::bare();
    let schema = schema();
    for port in [DATA_PIN, CLK_PIN, ENB_PIN] {
        assert!(
            Device::sink(&rtc, port, &[WireId::new(1)]).is_some(),
            "{port} is a sink"
        );
    }
    assert!(Device::sink(&rtc, "nonsense", &[WireId::new(1)]).is_none());
    assert_eq!(schema.class, CLASS_NAME);
}
