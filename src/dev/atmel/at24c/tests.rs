//! Tests for the AT24C EEPROM model.
//!
//! Written against the Atmel **AT24C01D/02D** datasheet, section by section:
//! every assertion names the paragraph it is checking, so a disagreement is
//! either a bug here or a misreading of that paragraph and nothing else.

use super::*;

use alloc::vec;
use alloc::vec::Vec;

use crate::bus::i2c::wires::{MasterEvent, MasterOp, MasterWires, pin as line};
use crate::core::device::ResetKind;
use crate::core::props::{Props, Value};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId, WireSource};

/// A part with the given properties, on a bus nothing else can reach.
fn part(props: &[(&str, Value)]) -> (At24c, Arc<I2cBus>) {
    let mut p = Props::new();
    for (name, value) in props {
        p.insert(*name, value.clone());
    }
    let eeprom = At24c::new(&p).expect("it builds");
    let bus = Arc::new(I2cBus::new());
    bus.attach(Arc::clone(&eeprom.shared) as Arc<dyn I2cSlave>)
        .expect("room on the bus");
    (eeprom, bus)
}

/// The default part: an AT24C02D at `0x50`.
fn at24c02() -> (At24c, Arc<I2cBus>) {
    part(&[])
}

/// A byte or page write: START, device address, word address, data, STOP.
fn write(bus: &I2cBus, address: u8, word: u8, data: &[u8]) {
    assert_eq!(
        bus.start(Address::Seven(address), Direction::Write),
        Ack::Ack,
        "the device did not answer its own address"
    );
    assert_eq!(bus.write(word), Ack::Ack);
    for byte in data {
        assert_eq!(bus.write(*byte), Ack::Ack);
    }
    bus.stop();
}

/// A random read (§6.2): a dummy write for the address, a repeated START, then
/// `count` bytes with the last one refused.
fn read_at(bus: &I2cBus, address: u8, word: u8, count: usize) -> Vec<u8> {
    assert_eq!(
        bus.start(Address::Seven(address), Direction::Write),
        Ack::Ack
    );
    assert_eq!(bus.write(word), Ack::Ack);
    // §6.2: "the Data Byte and the Stop condition of the Byte Write must be
    // omitted to prevent the part from entering an internal write cycle."
    assert_eq!(
        bus.start(Address::Seven(address), Direction::Read),
        Ack::Ack
    );
    let mut out = Vec::new();
    for i in 0..count {
        let last = i + 1 == count;
        out.push(bus.read(if last { Ack::Nack } else { Ack::Ack }));
    }
    bus.stop();
    out
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

#[test]
fn the_default_part_is_an_at24c02_at_the_address_its_pins_select() {
    let (eeprom, _) = at24c02();
    // §4.1, Table 4-1: the device type identifier is 1010 and the three pins
    // follow it, so all-low is 0x50.
    assert_eq!(eeprom.address(), Address::Seven(0x50));
    assert_eq!(eeprom.size(), 256);
    assert_eq!(eeprom.page(), 8);
    let (seven, _) = part(&[("chip", Value::Uint(7))]);
    assert_eq!(seven.address(), Address::Seven(0x57));
}

#[test]
fn the_array_comes_out_of_the_box_erased() {
    // §7: "The AT24C01D/02D is delivered with the EEPROM array set to Logic 1,
    // resulting in FFh data in all locations."
    let (eeprom, _) = at24c02();
    assert!(eeprom.contents().iter().all(|b| *b == 0xff));
}

#[test]
fn a_part_bigger_than_one_word_address_byte_is_refused() {
    let mut p = Props::new();
    p.insert("size", Value::Uint(512));
    let err = At24c::new(&p).expect_err("512 bytes is a different part");
    let text = alloc::format!("{err}");
    assert!(text.contains("device address"), "{text}");
}

#[test]
fn a_page_that_does_not_divide_the_array_is_refused() {
    let mut p = Props::new();
    p.insert("page", Value::Uint(7));
    assert!(At24c::new(&p).is_err(), "a page is a power of two");
}

// ---------------------------------------------------------------------------
// Writing
// ---------------------------------------------------------------------------

#[test]
fn a_byte_write_lands_and_costs_the_self_timed_write_cycle() {
    let (eeprom, bus) = at24c02();
    write(&bus, 0x50, 0x10, &[0xa5]);
    // §5.1: the array is written "while the data is being programmed into the
    // nonvolatile EEPROM", which begins at the STOP.
    assert_eq!(eeprom.byte(0x10), Some(0xa5));
    assert!(eeprom.busy(), "the internal write cycle is running");

    // §5.3: "The device will not respond with an ACK while the write cycle is
    // ongoing." That one sentence is acknowledge polling.
    assert_eq!(
        bus.start(Address::Seven(0x50), Direction::Write),
        Ack::Nack,
        "an acknowledge-polling master must be told to wait"
    );
    bus.stop();

    eeprom.advance_to(DEFAULT_WRITE_TICKS - 1);
    assert!(eeprom.busy(), "tWR has not elapsed yet");
    eeprom.advance_to(DEFAULT_WRITE_TICKS);
    assert!(!eeprom.busy());
    assert_eq!(
        bus.start(Address::Seven(0x50), Direction::Write),
        Ack::Ack,
        "and now it answers, which is what ends the polling loop"
    );
    bus.stop();
}

#[test]
fn a_page_write_rolls_over_inside_its_own_page() {
    // §5.2: "When the incremented word address reaches the page boundary, the
    // address counter will 'roll-over' to the beginning of the same page.
    // Nevertheless, creating a roll-over event should be avoided since
    // previously loaded data in the page could become unintentionally altered."
    let (eeprom, bus) = at24c02();
    // Start at 0x16, two from the end of the page 0x10..0x18, and send four.
    write(&bus, 0x50, 0x16, &[0x11, 0x22, 0x33, 0x44]);
    assert_eq!(eeprom.byte(0x16), Some(0x11));
    assert_eq!(eeprom.byte(0x17), Some(0x22));
    assert_eq!(
        eeprom.byte(0x10),
        Some(0x33),
        "the counter wrapped to the start of the same page"
    );
    assert_eq!(eeprom.byte(0x11), Some(0x44));
    assert_eq!(
        eeprom.byte(0x18),
        Some(0xff),
        "and the next page was not touched"
    );
}

#[test]
fn a_dummy_write_starts_no_write_cycle() {
    // §6.2 depends on this: a random read sets the address with a write that
    // has no data byte, and if that began a tWR the read that follows it would
    // be refused.
    let (eeprom, bus) = at24c02();
    bus.start(Address::Seven(0x50), Direction::Write);
    bus.write(0x20);
    bus.stop();
    assert!(!eeprom.busy());
    assert_eq!(eeprom.word_address(), 0x20);
}

#[test]
fn write_protect_is_sampled_at_the_stop_and_blocks_the_whole_array() {
    // §5.5, Table 5-1: `WP` at VCC protects the full array. "If an attempt is
    // made to write to the device while the WP pin has been asserted, the device
    // will acknowledge the Device Address, Word address, and Data bytes but no
    // write cycle will occur when the Stop condition is issued."
    let (eeprom, bus) = at24c02();
    eeprom.shared.state.lock().wp = Level::High;

    assert_eq!(bus.start(Address::Seven(0x50), Direction::Write), Ack::Ack);
    assert_eq!(
        bus.write(0x00),
        Ack::Ack,
        "the word address is acknowledged"
    );
    assert_eq!(bus.write(0x5a), Ack::Ack, "and so is the data");
    bus.stop();

    assert_eq!(eeprom.byte(0x00), Some(0xff), "but nothing was written");
    assert!(!eeprom.busy(), "and no write cycle began");

    // §5.5 again: the *status at the STOP* is what decides, so a `WP` that goes
    // low before the STOP lets the same transfer through.
    assert_eq!(bus.start(Address::Seven(0x50), Direction::Write), Ack::Ack);
    bus.write(0x00);
    bus.write(0x5a);
    eeprom.shared.state.lock().wp = Level::Low;
    bus.stop();
    assert_eq!(eeprom.byte(0x00), Some(0x5a));
}

#[test]
fn an_unwired_write_protect_pin_is_the_level_the_datasheet_gives_it() {
    // Table 1-1, note 1: "If the A0, A1, A2, or WP pins are not driven, they are
    // internally pulled down to GND." So an unwired pin is *not* an invented
    // level — it is the one the part specifies, and §5.5's Table 5-1 makes a
    // grounded WP mean no protection.
    let (eeprom, bus) = at24c02();
    assert!(!eeprom.shared.state.lock().wp_wired);
    assert_eq!(eeprom.shared.state.lock().wp, Level::Low);
    write(&bus, 0x50, 0x00, &[0x42]);
    assert_eq!(eeprom.byte(0x00), Some(0x42));
}

// ---------------------------------------------------------------------------
// Reading
// ---------------------------------------------------------------------------

#[test]
fn a_random_read_needs_a_dummy_write_and_a_repeated_start() {
    let (eeprom, bus) = at24c02();
    write(&bus, 0x50, 0x40, &[0xde, 0xad, 0xbe, 0xef]);
    eeprom.advance_to(DEFAULT_WRITE_TICKS);
    assert_eq!(read_at(&bus, 0x50, 0x41, 3), vec![0xad, 0xbe, 0xef]);
}

#[test]
fn a_current_address_read_carries_on_where_the_last_one_stopped() {
    // §6.1: "The internal data word address counter maintains the last address
    // accessed during the last Read or Write operation, incremented by one."
    let (eeprom, bus) = at24c02();
    write(&bus, 0x50, 0x00, &[1, 2, 3, 4]);
    eeprom.advance_to(DEFAULT_WRITE_TICKS);
    assert_eq!(read_at(&bus, 0x50, 0x00, 2), vec![1, 2]);

    // No word address this time: straight to a read.
    assert_eq!(bus.start(Address::Seven(0x50), Direction::Read), Ack::Ack);
    assert_eq!(bus.read(Ack::Nack), 3, "the counter had moved to 0x02");
    bus.stop();
}

#[test]
fn a_sequential_read_rolls_over_the_whole_array() {
    // §6.1: "The address roll-over during read is from the last byte of the
    // last page to the first byte of the first page" — not within a page, which
    // is the write's rule and the difference that matters.
    let (eeprom, bus) = at24c02();
    write(&bus, 0x50, 0xf8, &[0, 0, 0, 0, 0, 0, 0, 0x7e]);
    eeprom.advance_to(DEFAULT_WRITE_TICKS);
    write(&bus, 0x50, 0x00, &[0x7f]);
    eeprom.advance_to(2 * DEFAULT_WRITE_TICKS);

    assert_eq!(read_at(&bus, 0x50, 0xff, 2), vec![0x7e, 0x7f]);
}

#[test]
fn a_nack_ends_the_read_and_the_next_address_starts_a_new_one() {
    // §6.1: a NACK "will force the device into standby mode".
    let (eeprom, bus) = at24c02();
    write(&bus, 0x50, 0x00, &[0x11, 0x22]);
    eeprom.advance_to(DEFAULT_WRITE_TICKS);
    // A random read (§6.2), so the counter starts where this test means it to
    // rather than where the page write left it.
    assert_eq!(bus.start(Address::Seven(0x50), Direction::Write), Ack::Ack);
    assert_eq!(bus.write(0x00), Ack::Ack);
    assert_eq!(bus.start(Address::Seven(0x50), Direction::Read), Ack::Ack);
    assert_eq!(bus.read(Ack::Nack), 0x11);
    assert_eq!(
        bus.read(Ack::Nack),
        0xff,
        "nothing is driving SDA any more, so a master clocking on reads the pull-up"
    );
    bus.stop();
}

#[test]
fn another_devices_address_is_not_answered() {
    // §4.1: "If a valid comparison is not made, the device will NACK and return
    // to a standby state."
    let (eeprom, bus) = at24c02();
    assert_eq!(bus.start(Address::Seven(0x51), Direction::Write), Ack::Nack);
    assert_eq!(bus.write(0x00), Ack::Nack);
    assert_eq!(bus.write(0xaa), Ack::Nack);
    bus.stop();
    assert_eq!(eeprom.byte(0x00), Some(0xff));
}

#[test]
fn the_word_address_is_masked_to_the_array_the_part_actually_has() {
    // §4.1, Table 4-2 note 1: "The A7 bit is a don't care bit for the
    // AT24C01D." Masking to the size is exactly that.
    let (eeprom, bus) = part(&[("size", Value::Uint(128))]);
    write(&bus, 0x50, 0x83, &[0x99]);
    assert_eq!(eeprom.byte(0x03), Some(0x99));
}

// ---------------------------------------------------------------------------
// Debug access, reset, snapshots
// ---------------------------------------------------------------------------

#[test]
fn a_debug_look_at_the_bus_moves_no_counter() {
    let (eeprom, bus) = at24c02();
    write(&bus, 0x50, 0x00, &[0x11, 0x22]);
    eeprom.advance_to(DEFAULT_WRITE_TICKS);
    bus.start(Address::Seven(0x50), Direction::Write);
    bus.write(0x00);
    bus.start(Address::Seven(0x50), Direction::Read);
    assert_eq!(bus.peek(), 0x11);
    assert_eq!(bus.peek(), 0x11);
    assert_eq!(eeprom.word_address(), 0, "and the counter did not move");
    bus.stop();
}

#[test]
fn a_reset_keeps_the_array_and_the_tick() {
    let (eeprom, bus) = at24c02();
    write(&bus, 0x50, 0x00, &[0xa5]);
    eeprom.advance_to(1_000);
    eeprom.reset(ResetKind::Cold);
    // An EEPROM is non-volatile: a power-on reset of the board is not an erase.
    assert_eq!(eeprom.byte(0x00), Some(0xa5));
    // And a lazily advanced device must not rewind its own tick, because
    // `Machine::reset` does not rewind clock domains (`ROADMAP.md` §4.2).
    assert_eq!(eeprom.ticks(), 1_000);
    assert_eq!(eeprom.word_address(), 0);
    assert!(!eeprom.busy());
}

#[test]
fn a_transfer_part_way_through_its_address_phase_round_trips() {
    // The part of `CLAUDE.md`'s save/load rule that is easy to miss: the
    // *protocol* state is state. This snapshot is taken with four bits of an
    // address byte clocked in, and the restored part has to finish that byte
    // rather than start a new one.
    let mut p = Props::new();
    let eeprom = At24c::new(&p).unwrap();
    let master = Arc::new(MasterWires::new());
    let slave = Arc::clone(eeprom.wires());
    let ids = [
        WireId::new(1),
        WireId::new(2),
        WireId::new(3),
        WireId::new(4),
    ];
    let scl = Wire::builder()
        .sources(&[ids[0], ids[2]])
        .sink(master.sink(line::SCL, &[ids[0], ids[2]]), line::SCL)
        .sink(slave.sink(line::SCL, &[ids[0], ids[2]]), line::SCL)
        .build_shared();
    let sda = Wire::builder()
        .sources(&[ids[1], ids[3]])
        .sink(master.sink(line::SDA, &[ids[1], ids[3]]), line::SDA)
        .sink(slave.sink(line::SDA, &[ids[1], ids[3]]), line::SDA)
        .build_shared();
    master.connect(line::SCL, WireSource::new(Arc::clone(&scl), ids[0]));
    master.connect(line::SDA, WireSource::new(Arc::clone(&sda), ids[1]));
    slave.connect(line::SCL, WireSource::new(Arc::clone(&scl), ids[2]));
    slave.connect(line::SDA, WireSource::new(Arc::clone(&sda), ids[3]));
    master.announce();
    slave.announce();

    // A START, then half of the address byte.
    master.submit(MasterOp::Start);
    while master.is_working() {
        master.tick();
    }
    master.submit(MasterOp::Write(0xa0));
    for _ in 0..9 {
        master.tick();
    }

    let mut shape = MachineShape::new();
    shape.add_device("eeprom", AT24C_CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("eeprom", AT24C_CLASS.name, AT24C_CLASS.version)
            .unwrap();
        eeprom.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    p = Props::new();
    let other = At24c::new(&p).unwrap();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load(
            "eeprom",
            AT24C_CLASS.name,
            AT24C_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    other.load(&mut chunk.reader()).unwrap();

    // The identical-state-hash property, checked where it is observable: the
    // two parts' saved chunks are byte for byte the same.
    let mut shape = MachineShape::new();
    shape.add_device("eeprom", AT24C_CLASS.name).unwrap();
    let mut w2 = StateWriter::new(shape);
    {
        let mut chunk = w2
            .chunk("eeprom", AT24C_CLASS.name, AT24C_CLASS.version)
            .unwrap();
        other.save(&mut chunk).unwrap();
    }
    assert_eq!(w2.to_vec().unwrap(), bytes);
    assert_eq!(other.wires().snapshot(), eeprom.wires().snapshot());
}

#[test]
fn an_image_fills_the_array_and_an_oversized_one_is_refused() {
    let mut p = Props::new();
    p.insert(
        "image",
        Value::Media(crate::core::props::Media::new("eeprom", &[1u8, 2, 3][..])),
    );
    let eeprom = At24c::new(&p).unwrap();
    assert_eq!(eeprom.byte(0), Some(1));
    assert_eq!(eeprom.byte(2), Some(3));
    assert_eq!(eeprom.byte(3), Some(0xff), "the rest stays erased");

    let mut p = Props::new();
    p.insert("size", Value::Uint(128));
    p.insert(
        "image",
        Value::Media(crate::core::props::Media::new("eeprom", vec![0u8; 200])),
    );
    assert!(At24c::new(&p).is_err());
}

#[test]
fn a_wired_master_writes_and_reads_the_same_bytes_a_transactional_one_does() {
    // The claim `docs/buses/low-speed.md` asks for, checked on a real device
    // rather than on a mock: the same firmware-shaped sequence through the
    // two link models leaves the same array behind.
    let (transactional, bus) = at24c02();
    write(&bus, 0x50, 0x08, &[0xca, 0xfe, 0xf0, 0x0d]);
    transactional.advance_to(DEFAULT_WRITE_TICKS);
    let by_call = transactional.contents();

    let p = Props::new();
    let wired = At24c::new(&p).unwrap();
    let master = Arc::new(MasterWires::new());
    let slave = Arc::clone(wired.wires());
    let ids = [
        WireId::new(1),
        WireId::new(2),
        WireId::new(3),
        WireId::new(4),
    ];
    let scl = Wire::builder()
        .sources(&[ids[0], ids[2]])
        .sink(master.sink(line::SCL, &[ids[0], ids[2]]), line::SCL)
        .sink(slave.sink(line::SCL, &[ids[0], ids[2]]), line::SCL)
        .build_shared();
    let sda = Wire::builder()
        .sources(&[ids[1], ids[3]])
        .sink(master.sink(line::SDA, &[ids[1], ids[3]]), line::SDA)
        .sink(slave.sink(line::SDA, &[ids[1], ids[3]]), line::SDA)
        .build_shared();
    master.connect(line::SCL, WireSource::new(Arc::clone(&scl), ids[0]));
    master.connect(line::SDA, WireSource::new(Arc::clone(&sda), ids[1]));
    slave.connect(line::SCL, WireSource::new(Arc::clone(&scl), ids[2]));
    slave.connect(line::SDA, WireSource::new(Arc::clone(&sda), ids[3]));
    master.announce();
    slave.announce();

    let script = [
        MasterOp::Start,
        MasterOp::Write(0xa0),
        MasterOp::Write(0x08),
        MasterOp::Write(0xca),
        MasterOp::Write(0xfe),
        MasterOp::Write(0xf0),
        MasterOp::Write(0x0d),
        MasterOp::Stop,
    ];
    for op in script {
        assert!(master.submit(op));
        let mut done = None;
        for _ in 0..64 {
            match master.tick() {
                MasterEvent::Working | MasterEvent::Stretched => {}
                other => {
                    done = Some(other);
                    break;
                }
            }
        }
        match done {
            Some(MasterEvent::Wrote(ack)) => assert_eq!(ack, Ack::Ack, "{op:?} was refused"),
            Some(MasterEvent::Started | MasterEvent::Stopped) => {}
            other => panic!("{op:?} ended as {other:?}"),
        }
    }
    wired.advance_to(DEFAULT_WRITE_TICKS);
    assert_eq!(wired.contents(), by_call);
    assert_eq!(wired.byte(0x0b), Some(0x0d));
}
