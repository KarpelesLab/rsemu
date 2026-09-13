//! Tests for the ST25DV dynamic NFC tag.
//!
//! Written against ST **DS10925 rev 9**, table by table: every assertion names
//! the paragraph it is checking, so a disagreement is either a bug here or a
//! misreading of that paragraph and nothing else. The RF half goes through
//! [`rf::Reader`] rather than through any private entry point, because the
//! reader door *is* the RF interface — there is no other way in, deliberately.

use super::*;

use alloc::vec;
use alloc::vec::Vec;

use crate::core::device::ResetKind;
use crate::core::props::{Props, Value};
use crate::core::record::{Channel, Recorder};
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

// ---------------------------------------------------------------------------
// Harness
// ---------------------------------------------------------------------------

/// A tag with the given properties, on a bus nothing else can reach, with its
/// reader door already bound.
fn tag(props: &[(&str, Value)]) -> (St25dv, Arc<I2cBus>) {
    let mut p = Props::new();
    for (name, value) in props {
        p.insert(*name, value.clone());
    }
    let part = St25dv::new(&p).expect("it builds");
    let bus = Arc::new(I2cBus::new());
    bus.attach(Arc::clone(&part.shared) as Arc<dyn I2cSlave>)
        .expect("room on the bus");
    // What `realize` does. A unit test holds the device directly, so it does
    // the outward half itself rather than running a machine.
    part.reader.bind(&part.shared);
    (part, bus)
}

/// The default part: an ST25DV64K.
fn st25dv() -> (St25dv, Arc<I2cBus>) {
    tag(&[])
}

/// Address the part for a write and send the two address bytes (§6.4).
fn open_write(bus: &I2cBus, seven: u8, at: u16) -> Ack {
    let ack = bus.start(Address::Seven(seven), Direction::Write);
    if !ack.is_ack() {
        return ack;
    }
    assert_eq!(bus.write((at >> 8) as u8), Ack::Ack, "address MSB");
    assert_eq!(bus.write(at as u8), Ack::Ack, "address LSB");
    Ack::Ack
}

/// A byte or sequential write. Answers the acknowledge of each data byte.
fn write(bus: &I2cBus, seven: u8, at: u16, data: &[u8]) -> Vec<Ack> {
    assert_eq!(open_write(bus, seven, at), Ack::Ack, "the part answered");
    let acks: Vec<Ack> = data.iter().map(|b| bus.write(*b)).collect();
    bus.stop();
    acks
}

/// A write every byte of which must be taken.
fn write_ok(bus: &I2cBus, seven: u8, at: u16, data: &[u8]) {
    for (i, ack) in write(bus, seven, at, data).into_iter().enumerate() {
        assert_eq!(ack, Ack::Ack, "byte {i} of a write that should be allowed");
    }
}

/// A random read (§6.5.1): a dummy write for the address, a repeated START,
/// then `count` bytes with the last one refused.
fn read_at(bus: &I2cBus, seven: u8, at: u16, count: usize) -> Vec<u8> {
    assert_eq!(open_write(bus, seven, at), Ack::Ack);
    assert_eq!(bus.start(Address::Seven(seven), Direction::Read), Ack::Ack);
    let mut out = Vec::new();
    for i in 0..count {
        let last = i + 1 == count;
        out.push(bus.read(if last { Ack::Nack } else { Ack::Ack }));
    }
    bus.stop();
    out
}

/// One byte, read the same way.
fn read_one(bus: &I2cBus, seven: u8, at: u16) -> u8 {
    read_at(bus, seven, at, 1)[0]
}

/// Present the I²C password (§6.6.1): address `0900h`, the password, `09h`,
/// the password again, STOP.
fn present_password(bus: &I2cBus, password: &[u8; 8]) {
    assert_eq!(open_write(bus, SYSTEM_ADDRESS, I2C_PWD_BASE), Ack::Ack);
    for byte in password {
        assert_eq!(bus.write(*byte), Ack::Ack);
    }
    assert_eq!(bus.write(0x09), Ack::Ack, "the validation code");
    for byte in password {
        assert_eq!(bus.write(*byte), Ack::Ack);
    }
    bus.stop();
}

/// Post one RF command straight at the reader, the way the record/replay sink
/// does — through the sink, so the test exercises the same path a recording
/// replays through.
fn rf_post(part: &St25dv, commands: &[rf::Command<'_>]) {
    let payload = rf::Command::encode_all(commands);
    rf::sink(&part.reader).deliver(&payload);
}

/// Bring the field up, which every RF command needs.
fn field_on(part: &St25dv) {
    rf_post(part, &[rf::Command::Field(true)]);
    part.reader.clear();
}

/// The one response a command produced, asserting there is exactly one.
fn one_reply(part: &St25dv) -> Vec<u8> {
    let mut replies = part.reader.drain();
    assert_eq!(replies.len(), 1, "exactly one response: {replies:?}");
    replies.pop().expect("one")
}

// ---------------------------------------------------------------------------
// Identity and construction
// ---------------------------------------------------------------------------

#[test]
fn the_uid_and_ic_ref_read_from_0x57_at_0x0018_match_the_part() {
    for (density, ic_ref, blocks) in [
        (Density::K4, 0x24u8, 128u64),
        (Density::K16, 0x26, 512),
        (Density::K64, 0x26, 2048),
    ] {
        let (part, bus) = tag(&[
            ("density", Value::Str(density.name().into())),
            ("uid", Value::Uint(0x01_0203_0405)),
        ]);
        // Table 83.
        assert_eq!(
            read_one(&bus, SYSTEM_ADDRESS, sys::IC_REF),
            ic_ref,
            "IC_REF for an ST25DV{density}"
        );
        // Table 79: MEM_SIZE is the block count *minus one*, LSB first.
        let size = read_at(&bus, SYSTEM_ADDRESS, sys::MEM_SIZE, 2);
        assert_eq!(
            u64::from(u16::from_le_bytes([size[0], size[1]])) + 1,
            blocks
        );
        // Table 81: BLK_SIZE is the block size minus one, so 03h.
        assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::BLK_SIZE), 0x03);

        // Table 85: over I²C the UID reads byte 0 (LSB) first, so `E0h` is at
        // `001Fh` and not at `0018h`. The issue that asked for this model had
        // it the other way round, which is the RF frame's order.
        let uid = read_at(&bus, SYSTEM_ADDRESS, sys::UID, 8);
        assert_eq!(uid[7], 0xe0, "byte 7 is E0h");
        assert_eq!(uid[6], 0x02, "byte 6 is the ST manufacturer code");
        assert_eq!(uid[5], ic_ref, "byte 5 is the ST product code");
        assert_eq!(&uid[..5], &[0x05, 0x04, 0x03, 0x02, 0x01]);
        assert_eq!(part.density(), density);
        assert_eq!(part.contents().len() as u64, density.bytes());
    }
}

#[test]
fn the_factory_configuration_is_the_one_the_tables_specify() {
    let (_, bus) = st25dv();
    // Table 26: FIELD_CHANGE_EN and GPO_EN are set out of the factory.
    assert_eq!(
        read_one(&bus, SYSTEM_ADDRESS, sys::GPO),
        gpo_bit::FIELD_CHANGE_EN | gpo_bit::GPO_EN
    );
    // Table 28: IT_TIME = 011b.
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::IT_TIME), 0b011);
    // Table 35: EH_MODE = 1, "EH on demand only", so EH_EN boots clear
    // (Table 38).
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::EH_MODE), 1);
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::EH_CTRL) & eh_bit::EH_EN,
        0
    );
    // Table 17: MB_WDG = 111b, and Table 15: MB_MODE = 0.
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::MB_WDG), 0b111);
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::MB_MODE), 0);
    // §4.4: GPO_CTRL_Dyn is a copy of GPO at power up.
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::GPO_CTRL),
        gpo_bit::FIELD_CHANGE_EN | gpo_bit::GPO_EN
    );
    // Table 19: MB_CTRL_Dyn is 00h.
    assert_eq!(read_one(&bus, USER_ADDRESS, dyn_reg::MB_CTRL), 0);
}

#[test]
fn an_unknown_density_or_gpo_style_is_refused_by_name() {
    let mut p = Props::new();
    p.insert("density", Value::Str("32K".into()));
    let text = alloc::format!("{}", St25dv::new(&p).expect_err("no such part"));
    assert!(text.contains("density"), "{text}");

    let mut p = Props::new();
    p.insert("gpo", Value::Str("push-pull".into()));
    let text = alloc::format!("{}", St25dv::new(&p).expect_err("no such output"));
    assert!(text.contains("open-drain"), "{text}");
}

// ---------------------------------------------------------------------------
// User memory over I²C
// ---------------------------------------------------------------------------

#[test]
fn user_memory_writes_at_0x53_read_back_and_the_device_nacks_during_tw() {
    let (part, bus) = st25dv();
    write_ok(&bus, USER_ADDRESS, 0x0010, &[0xde, 0xad, 0xbe, 0xef]);
    assert_eq!(part.byte(0x10), Some(0xde));
    assert_eq!(part.byte(0x13), Some(0xef));

    // §6.4.3: "During the internal write cycle, the device disconnects itself
    // from the bus" — which is the whole of acknowledge polling.
    assert!(part.busy());
    assert_eq!(
        bus.start(Address::Seven(USER_ADDRESS), Direction::Write),
        Ack::Nack,
        "an acknowledge-polling master must be told to wait"
    );
    bus.stop();
    // Four bytes from 0x10 are one four-byte page (§6.4.2), so one tW.
    part.advance_to(DEFAULT_WRITE_TICKS - 1);
    assert!(part.busy());
    part.advance_to(DEFAULT_WRITE_TICKS);
    assert!(!part.busy());
    assert_eq!(
        read_at(&bus, USER_ADDRESS, 0x0010, 4),
        vec![0xde, 0xad, 0xbe, 0xef]
    );
}

#[test]
fn a_sequential_write_costs_one_tw_per_four_byte_page() {
    let (part, bus) = st25dv();
    // §6.4.2's worked example: a write starting one byte into a page touches
    // one more page than its length divided by four.
    write_ok(&bus, USER_ADDRESS, 0x0002, &[0; 8]);
    part.advance_to(3 * DEFAULT_WRITE_TICKS - 1);
    assert!(part.busy(), "bytes 2..9 cover pages 0, 1 and 2");
    part.advance_to(3 * DEFAULT_WRITE_TICKS);
    assert!(!part.busy());
}

#[test]
fn a_257_byte_write_is_refused() {
    let (part, bus) = st25dv();
    assert_eq!(open_write(&bus, USER_ADDRESS, 0x0000), Ack::Ack);
    for i in 0..MAX_SEQUENTIAL_WRITE {
        assert_eq!(bus.write(i as u8), Ack::Ack, "byte {i} is within the limit");
    }
    // §6.4.2: "256 write occurrence have already been reached in the same
    // sequential write."
    assert_eq!(bus.write(0xff), Ack::Nack, "the 257th byte");
    bus.stop();
    // "If some bytes have been NotAck'ed, no internal programming is done
    // (0 byte written)."
    assert!(!part.busy(), "nothing was programmed");
    assert_eq!(part.byte(0), Some(0x00));
}

#[test]
fn a_sequential_read_stops_at_the_end_of_memory_rather_than_rolling_over() {
    let (_, bus) = tag(&[("density", Value::Str("4K".into()))]);
    // §6.5.3: "There is no roll over … ST25DVxxx returns only FFh after last
    // user memory byte address."
    let last = (Density::K4.bytes() - 2) as u16;
    assert_eq!(
        read_at(&bus, USER_ADDRESS, last, 4),
        vec![0x00, 0x00, 0xff, 0xff]
    );
}

// ---------------------------------------------------------------------------
// The I²C security session
// ---------------------------------------------------------------------------

#[test]
fn system_configuration_is_read_only_without_a_security_session() {
    let (part, bus) = st25dv();
    // §6.4.1: "Byte is in system memory and I2C security session is closed."
    assert!(!part.session_open());
    assert_eq!(
        write(&bus, SYSTEM_ADDRESS, sys::IT_TIME, &[0b111]),
        vec![Ack::Nack]
    );
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::IT_TIME), 0b011);
    // Reading it never needed one: §4.3, "Read accesses to the static
    // configuration register is always allowed, except for passwords."
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::GPO), 0x88);
}

#[test]
fn presenting_the_right_password_opens_the_session_and_a_wrong_one_does_not() {
    let secret = [0x01u8, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef];
    let (part, bus) = tag(&[("password", Value::Uint(0x0123_4567_89ab_cdef))]);

    present_password(&bus, &[0; 8]);
    assert!(
        !part.session_open(),
        "the wrong password closes the session"
    );
    assert_eq!(read_one(&bus, USER_ADDRESS, dyn_reg::I2C_SSO), 0x00);

    present_password(&bus, &secret);
    assert!(part.session_open());
    // §6.6.1: "the I2C security session is open, and the I2C_SSO_Dyn register
    // is set to 01h".
    assert_eq!(read_one(&bus, USER_ADDRESS, dyn_reg::I2C_SSO), 0x01);

    // And now the configuration is writable.
    write_ok(&bus, SYSTEM_ADDRESS, sys::IT_TIME, &[0b000]);
    part.advance_to(DEFAULT_WRITE_TICKS);
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::IT_TIME), 0b000);

    // A mismatched pair never even starts the comparison (§6.6.1), so the
    // session survives.
    assert_eq!(open_write(&bus, SYSTEM_ADDRESS, I2C_PWD_BASE), Ack::Ack);
    for byte in &secret {
        assert_eq!(bus.write(*byte), Ack::Ack);
    }
    assert_eq!(bus.write(0x09), Ack::Ack);
    for _ in 0..8 {
        assert_eq!(bus.write(0x00), Ack::Ack);
    }
    bus.stop();
    assert!(
        part.session_open(),
        "the two halves disagreed, so nothing ran"
    );

    present_password(&bus, &[0xff; 8]);
    assert!(!part.session_open());
}

#[test]
fn a_write_that_crosses_an_i2c_protected_area_boundary_is_refused() {
    let (part, bus) = st25dv();
    present_password(&bus, &[0; 8]);
    assert!(part.session_open(), "the factory password is zero");

    // Table 6: ENDA1 counts 32-byte steps over I²C, so ENDA1 = 0 puts the
    // border after byte 31. §4.2 insists the successors are already at the end
    // of memory, which is where they start.
    write_ok(&bus, SYSTEM_ADDRESS, sys::ENDA1, &[0x00]);
    part.advance_to(DEFAULT_WRITE_TICKS);
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::ENDA1), 0x00);

    // §6.4.2: "Byte is in user memory but does not belong to same area than
    // previous received byte (area border crossing is forbidden)."
    let acks = write(&bus, USER_ADDRESS, 30, &[1, 2, 3, 4]);
    assert_eq!(acks, vec![Ack::Ack, Ack::Ack, Ack::Nack, Ack::Nack]);
    assert_eq!(part.byte(30), Some(0), "a refused sequence writes nothing");

    // Area 2 write-protected by I2CSS (Table 52, code 01) with the session
    // shut: the write is inhibited.
    write_ok(&bus, SYSTEM_ADDRESS, sys::I2CSS, &[0b0000_0100]);
    part.advance_to(2 * DEFAULT_WRITE_TICKS);
    present_password(&bus, &[0xff; 8]);
    assert!(!part.session_open());
    assert_eq!(write(&bus, USER_ADDRESS, 32, &[0xaa]), vec![Ack::Nack]);
    assert_eq!(write(&bus, USER_ADDRESS, 0, &[0xaa]), vec![Ack::Ack]);
}

#[test]
fn an_enda_that_breaks_the_ordering_rule_is_nacked() {
    let (part, bus) = st25dv();
    present_password(&bus, &[0; 8]);
    // §4.2 spells the order out: with every ENDAi at the end of memory, ENDA1
    // goes first, then ENDA2 above it, then ENDA3 above that.
    write_ok(&bus, SYSTEM_ADDRESS, sys::ENDA1, &[0x10]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    write_ok(&bus, SYSTEM_ADDRESS, sys::ENDA2, &[0x40]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::ENDA2), 0x40);

    // "Successful ENDA3 programming condition: ENDA2 < ENDA3", so ENDA3 may
    // not be lowered onto ENDA2's own value…
    assert_eq!(
        write(&bus, SYSTEM_ADDRESS, sys::ENDA3, &[0x40]),
        vec![Ack::Nack],
        "ENDA2 < ENDA3 is not respected"
    );
    // …and ENDA1 may not be raised past ENDA2, nor written at all while its
    // successors are not both at the end of memory.
    assert_eq!(
        write(&bus, SYSTEM_ADDRESS, sys::ENDA1, &[0x50]),
        vec![Ack::Nack],
        "ENDA1's rule wants ENDA1 <= ENDA2 = ENDA3 = end of memory"
    );
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::ENDA1), 0x10);
}

// ---------------------------------------------------------------------------
// Dynamic registers
// ---------------------------------------------------------------------------

#[test]
fn the_read_only_dynamic_registers_are_nacked_and_the_writable_bits_are_masked() {
    let (_, bus) = st25dv();
    // Table 29: bits 0-6 of GPO_CTRL_Dyn are read-only, bit 7 is not, and no
    // password is needed for it.
    write_ok(&bus, USER_ADDRESS, dyn_reg::GPO_CTRL, &[0x00]);
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::GPO_CTRL),
        gpo_bit::FIELD_CHANGE_EN,
        "only GPO_EN moved"
    );
    // Table 12: `2001h`, I2C_SSO_Dyn, IT_STS_Dyn and MB_LEN_Dyn are read-only.
    for reg in [
        dyn_reg::RESERVED,
        dyn_reg::I2C_SSO,
        dyn_reg::IT_STS,
        dyn_reg::MB_LEN,
    ] {
        assert_eq!(write(&bus, USER_ADDRESS, reg, &[0xff]), vec![Ack::Nack]);
    }
    // Table 36: only EH_EN is writable.
    write_ok(&bus, USER_ADDRESS, dyn_reg::EH_CTRL, &[0xff]);
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::EH_CTRL),
        eh_bit::EH_EN | eh_bit::VCC_ON,
        "EH_EN took, and VCC_ON is a fact rather than a bit somebody wrote"
    );
}

#[test]
fn writing_eh_mode_to_zero_turns_energy_harvesting_on() {
    let (part, bus) = st25dv();
    present_password(&bus, &[0; 8]);
    // Table 38: "Writing 0 in EH_MODE at any time after boot will
    // automatically set EH_EN bit to 1."
    write_ok(&bus, SYSTEM_ADDRESS, sys::EH_MODE, &[0]);
    part.advance_to(DEFAULT_WRITE_TICKS);
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::EH_CTRL) & eh_bit::EH_EN,
        eh_bit::EH_EN
    );
    // §5.3.2: EH_ON follows EH_EN, but there is nothing to harvest without a
    // field — which is the whole of what this model says about the analogue
    // V_EH pin, and it says it honestly.
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::EH_CTRL) & eh_bit::EH_ON,
        0
    );
    field_on(&part);
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::EH_CTRL),
        eh_bit::EH_EN | eh_bit::EH_ON | eh_bit::FIELD_ON | eh_bit::VCC_ON
    );
}

// ---------------------------------------------------------------------------
// GPO, IT_TIME and field detect
// ---------------------------------------------------------------------------

#[test]
fn it_sts_dyn_clears_on_read_and_gpo_pulses_for_it_time() {
    let (part, bus) = st25dv();
    // Out of the factory: FIELD_CHANGE_EN and GPO_EN set, IT_TIME = 3.
    // Eq. (1): 301 µs − 3 × 37.65 µs = 188 µs, and the default `it-ticks` puts
    // that in microseconds.
    assert_eq!(part.gpo(), Drive::HiZ, "open drain idles high-Z");
    rf_post(&part, &[rf::Command::Field(true)]);
    assert_eq!(part.gpo(), Drive::Low, "the pulse pulls the pin to ground");

    // Table 32: the *status* bit for a rising field is bit 4, one above the
    // single FIELD_CHANGE_EN enable bit in Table 26. A debug read must not
    // clear it (`CLAUDE.md`, devices).
    assert_eq!(part.register(false, dyn_reg::IT_STS), it_bit::FIELD_RISING);
    assert_eq!(
        part.register(false, dyn_reg::IT_STS),
        it_bit::FIELD_RISING,
        "a debug read leaves it alone"
    );

    part.advance_to(187);
    assert_eq!(part.gpo(), Drive::Low, "188 µs has not elapsed");
    part.advance_to(188);
    assert_eq!(part.gpo(), Drive::HiZ, "and now it has");

    // "Once read the ITSTS_Dyn register is cleared (set to 00h)."
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::IT_STS),
        it_bit::FIELD_RISING
    );
    assert_eq!(read_one(&bus, USER_ADDRESS, dyn_reg::IT_STS), 0x00);
}

#[test]
fn the_it_time_register_moves_the_pulse_the_way_the_equation_says() {
    let (part, bus) = st25dv();
    present_password(&bus, &[0; 8]);
    // Eq. (1) at each end of the three-bit field: 301 µs and
    // 301 − 7 × 37.65 = 37.45 µs, which integer arithmetic on a hundredth of a
    // microsecond makes 37.
    for (it_time, expected) in [(0u8, 301u64), (7, 37)] {
        write_ok(&bus, SYSTEM_ADDRESS, sys::IT_TIME, &[it_time]);
        part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
        let state = part.shared.state.lock();
        assert_eq!(
            part.shared.pulse_ticks(&state),
            expected,
            "IT_TIME={it_time}"
        );
    }
}

#[test]
fn a_cmos_part_idles_low_and_drives_the_pulse_high() {
    let (part, _) = tag(&[("gpo", Value::Str("cmos".into()))]);
    // §5.2.1 defines the -JF part by inverting the -IE curve.
    assert_eq!(part.gpo(), Drive::Low);
    rf_post(&part, &[rf::Command::Field(true)]);
    assert_eq!(part.gpo(), Drive::High);
}

#[test]
fn gpo_en_silences_the_pin_without_silencing_the_status_register() {
    let (part, bus) = st25dv();
    // Table 33: either GPO_EN at zero leaves the pin idle, and GPO_CTRL_Dyn's
    // copy needs no password.
    write_ok(&bus, USER_ADDRESS, dyn_reg::GPO_CTRL, &[0]);
    rf_post(&part, &[rf::Command::Field(true)]);
    assert_eq!(part.gpo(), Drive::HiZ, "the output stage is off");
    // "Disabling GPO output … does not disable interruption report in
    // IT_STS_Dyn status register."
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::IT_STS),
        it_bit::FIELD_RISING
    );
}

#[test]
fn rf_sleep_hides_the_field_change_from_both_the_pin_and_the_status() {
    let (part, bus) = st25dv();
    write_ok(&bus, USER_ADDRESS, dyn_reg::RF_MNGT, &[rf_bit::RF_SLEEP]);
    rf_post(&part, &[rf::Command::Field(true)]);
    // Table 22: "GPO remains High-Z … IT_STS_Dyn register is not updated."
    assert_eq!(part.gpo(), Drive::HiZ);
    assert_eq!(read_one(&bus, USER_ADDRESS, dyn_reg::IT_STS), 0x00);
    // But the power-source flag still follows the field, because that is not
    // an interrupt.
    assert!(part.field());
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::EH_CTRL) & eh_bit::FIELD_ON,
        eh_bit::FIELD_ON
    );
    // And the tag is silent, which is the other half of RF sleep.
    rf_post(&part, &[rf::Command::Inventory]);
    assert_eq!(part.reader.pending(), 0);
}

#[test]
fn manage_gpo_forces_the_level_and_outranks_a_pulse() {
    let (part, bus) = st25dv();
    present_password(&bus, &[0; 8]);
    write_ok(
        &bus,
        SYSTEM_ADDRESS,
        sys::GPO,
        &[gpo_bit::RF_USER_EN | gpo_bit::GPO_EN],
    );
    part.advance_to(DEFAULT_WRITE_TICKS);
    field_on(&part);

    // Table 194: bit 7 clear and bit 0 clear is "set", which on an open-drain
    // part pulls the pin to ground.
    rf_post(&part, &[rf::Command::ManageGpo { value: 0x00 }]);
    assert_eq!(one_reply(&part), vec![0x00]);
    assert_eq!(part.gpo(), Drive::Low);
    // §5.2.1: "RF_USER is prevalent over all other GPO events", and it is a
    // level, so no amount of time releases it.
    part.advance_to(part.ticks() + 10_000);
    assert_eq!(part.gpo(), Drive::Low);

    rf_post(&part, &[rf::Command::ManageGpo { value: 0x01 }]);
    assert_eq!(one_reply(&part), vec![0x00]);
    assert_eq!(part.gpo(), Drive::HiZ, "bit 0 set releases it");

    // With RF_INTERRUPT disabled, a pulse request is refused (Table 196).
    rf_post(&part, &[rf::Command::ManageGpo { value: 0x80 }]);
    assert_eq!(
        one_reply(&part),
        vec![rf::ERROR_FLAG, rf::error::NOT_PROGRAMMED]
    );
}

#[test]
fn the_lpd_pin_takes_vcc_away_and_with_it_the_mailbox() {
    let (part, bus) = st25dv();
    enable_mailbox(&part, &bus);
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::EH_CTRL) & eh_bit::VCC_ON,
        eh_bit::VCC_ON
    );
    {
        // Table 37: VCC_ON is "0: No DC supply detected on VCC pin or Low Power
        // Down mode is forced (LPD is high)".
        part.shared.state.lock().lpd = true;
    }
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::EH_CTRL) & eh_bit::VCC_ON,
        0
    );
    // §5.1.2: "VCC supply source is mandatory to activate this feature."
    assert_eq!(
        write(&bus, USER_ADDRESS, MAILBOX_BASE, &[1]),
        vec![Ack::Nack]
    );
}

// ---------------------------------------------------------------------------
// The mailbox
// ---------------------------------------------------------------------------

/// Authorise fast transfer mode (`MB_MODE`) and enable it (`MB_EN`).
fn enable_mailbox(part: &St25dv, bus: &I2cBus) {
    present_password(bus, &[0; 8]);
    write_ok(bus, SYSTEM_ADDRESS, sys::MB_MODE, &[1]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    write_ok(bus, USER_ADDRESS, dyn_reg::MB_CTRL, &[mb_bit::MB_EN]);
    assert_eq!(
        read_one(bus, USER_ADDRESS, dyn_reg::MB_CTRL),
        mb_bit::MB_EN,
        "§5.1.2's state diagram: MB_CTRL_Dyn = 01h"
    );
}

#[test]
fn the_mailbox_cannot_be_enabled_unless_mb_mode_authorises_it() {
    let (_, bus) = st25dv();
    // Table 15: "0: Enabling fast transfer mode is forbidden."
    assert_eq!(
        write(&bus, USER_ADDRESS, dyn_reg::MB_CTRL, &[mb_bit::MB_EN]),
        vec![Ack::Nack]
    );
}

#[test]
fn an_i2c_message_put_in_the_mailbox_is_read_by_the_rf_side_and_clears_host_put_msg() {
    let (part, bus) = st25dv();
    enable_mailbox(&part, &bus);
    field_on(&part);

    write_ok(&bus, USER_ADDRESS, MAILBOX_BASE, &[0xca, 0xfe, 0xba, 0xbe]);
    // §5.1.2: "the message length is automatically set into MB_LEN_Dyn …
    // MB_LEN_Dyn contains the size of the message in byte, minus 1."
    assert_eq!(read_one(&bus, USER_ADDRESS, dyn_reg::MB_LEN), 3);
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::MB_CTRL),
        mb_bit::MB_EN | mb_bit::HOST_PUT_MSG | mb_bit::HOST_CURRENT_MSG,
        "§5.1.2's figure 11 calls this 43h"
    );

    // The reader asks how long it is, then takes it.
    rf_post(&part, &[rf::Command::ReadMessageLength]);
    assert_eq!(one_reply(&part), vec![0x00, 3]);
    rf_post(
        &part,
        &[rf::Command::ReadMessage {
            offset: 0,
            count: 4,
        }],
    );
    assert_eq!(one_reply(&part), vec![0x00, 0xca, 0xfe, 0xba, 0xbe]);

    // "HOST_PUT_MSG is cleared following a valid reading of the last message
    // byte, and mailbox is considered free (but message is not cleared)."
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::MB_CTRL),
        mb_bit::MB_EN | mb_bit::HOST_CURRENT_MSG,
        "§5.1.2's figure 11 calls this 41h"
    );
    assert_eq!(
        read_at(&bus, USER_ADDRESS, MAILBOX_BASE, 4),
        vec![0xca, 0xfe, 0xba, 0xbe]
    );
}

#[test]
fn an_rf_message_sets_rf_put_msg_pulses_gpo_and_is_read_by_i2c() {
    let (part, bus) = st25dv();
    present_password(&bus, &[0; 8]);
    // Table 26: enable the RF_PUT_MSG interrupt so the pin has something to do.
    write_ok(
        &bus,
        SYSTEM_ADDRESS,
        sys::GPO,
        &[gpo_bit::RF_PUT_MSG_EN | gpo_bit::GPO_EN],
    );
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    enable_mailbox(&part, &bus);
    field_on(&part);

    rf_post(&part, &[rf::Command::WriteMessage(&[1, 2, 3])]);
    assert_eq!(one_reply(&part), vec![0x00]);
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::MB_CTRL),
        mb_bit::MB_EN | mb_bit::RF_PUT_MSG | mb_bit::RF_CURRENT_MSG,
        "§5.1.2's figure 11 calls this 85h"
    );
    assert_eq!(read_one(&bus, USER_ADDRESS, dyn_reg::MB_LEN), 2);
    assert_eq!(part.gpo(), Drive::Low, "GPO pulsed for the message");
    // Table 32: the RF_PUT_MSG *status* bit is 20h, one above the 10h enable.
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::IT_STS) & it_bit::RF_PUT_MSG,
        it_bit::RF_PUT_MSG
    );

    // "A I2C reading operation will never clear HOST_PUT_MSG", and the mirror
    // of that: reading it from I²C clears RF_PUT_MSG, at the STOP.
    assert_eq!(read_at(&bus, USER_ADDRESS, MAILBOX_BASE, 3), vec![1, 2, 3]);
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::MB_CTRL),
        mb_bit::MB_EN | mb_bit::RF_CURRENT_MSG
    );
    // §5.1.2: "data out is set to FFh when the counter reaches the message
    // end", with no roll-over.
    assert_eq!(
        read_at(&bus, USER_ADDRESS, MAILBOX_BASE, 5),
        vec![1, 2, 3, 0xff, 0xff]
    );
}

#[test]
fn the_mailbox_rejects_a_write_while_a_message_is_pending() {
    let (part, bus) = st25dv();
    enable_mailbox(&part, &bus);
    field_on(&part);
    write_ok(&bus, USER_ADDRESS, MAILBOX_BASE, &[0xaa]);

    // §5.1.2: "Adding a message is only possible when fast transfer mode is
    // enabled and mailbox is free."
    assert_eq!(
        write(&bus, USER_ADDRESS, MAILBOX_BASE, &[0xbb]),
        vec![Ack::Nack]
    );
    rf_post(&part, &[rf::Command::WriteMessage(&[0xcc])]);
    assert_eq!(
        one_reply(&part),
        vec![rf::ERROR_FLAG, rf::error::NO_INFORMATION]
    );

    // And a write that does not start at the first mailbox location is refused
    // whatever the state: "A I2C write operation must start from the first
    // mailbox location, at address 2008h."
    rf_post(
        &part,
        &[rf::Command::ReadMessage {
            offset: 0,
            count: 1,
        }],
    );
    part.reader.clear();
    assert_eq!(
        write(&bus, USER_ADDRESS, MAILBOX_BASE + 1, &[0xbb]),
        vec![Ack::Nack]
    );
}

#[test]
fn the_mailbox_watchdog_clears_a_message_nobody_read() {
    let (part, bus) = st25dv();
    // Table 17: MB_WDG = 1 is 2^0 × 30 ms, which the default `wdg-ticks` makes
    // 30 000 ticks. It is a system register, so it has to be set *before* the
    // mailbox goes on and shuts the EEPROM write path (§6.4's caution).
    present_password(&bus, &[0; 8]);
    write_ok(&bus, SYSTEM_ADDRESS, sys::MB_WDG, &[1]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    enable_mailbox(&part, &bus);

    let started = part.ticks();
    write_ok(&bus, USER_ADDRESS, MAILBOX_BASE, &[0x11, 0x22]);
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::MB_CTRL) & mb_bit::HOST_PUT_MSG,
        mb_bit::HOST_PUT_MSG
    );
    part.advance_to(started + DEFAULT_WDG_TICKS - 1);
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::MB_CTRL) & mb_bit::HOST_PUT_MSG,
        mb_bit::HOST_PUT_MSG,
        "the watchdog has not fired"
    );
    part.advance_to(started + DEFAULT_WDG_TICKS);
    // §5.1.2: "when a time-out occurs, the mailbox is considered free, and the
    // HOST_MISS_MSG or RF_MISS_MSG bits is set … The data … is not cleared."
    let ctrl = read_one(&bus, USER_ADDRESS, dyn_reg::MB_CTRL);
    assert_eq!(ctrl & mb_bit::HOST_PUT_MSG, 0, "the mailbox is free again");
    assert_eq!(
        ctrl & mb_bit::RF_MISS_MSG,
        mb_bit::RF_MISS_MSG,
        "RF is the side that missed it"
    );
    assert_eq!(
        read_at(&bus, USER_ADDRESS, MAILBOX_BASE, 2),
        vec![0x11, 0x22]
    );
    // A watchdog with MB_WDG = 0 is infinite, which is the other branch.
    write_ok(&bus, USER_ADDRESS, dyn_reg::MB_CTRL, &[0]);
    write_ok(&bus, SYSTEM_ADDRESS, sys::MB_WDG, &[0]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    let state = part.shared.state.lock();
    assert_eq!(part.shared.wdg_duration(&state), None);
}

#[test]
fn enabling_the_mailbox_shuts_the_eeprom_write_path() {
    let (part, bus) = st25dv();
    enable_mailbox(&part, &bus);
    field_on(&part);
    // §6.4's caution: "I2C Writing data in user or system memory (EEPROM) …
    // transit via the 256-Bytes fast transfer mode's buffer. Consequently fast
    // transfer mode must be deactivated before starting any write operation."
    assert_eq!(write(&bus, USER_ADDRESS, 0x0000, &[0xaa]), vec![Ack::Nack]);
    assert_eq!(
        write(&bus, SYSTEM_ADDRESS, sys::IT_TIME, &[0]),
        vec![Ack::Nack]
    );
    // §5.1.2 says the RF side gets error 0Fh for the same reason.
    rf_post(
        &part,
        &[rf::Command::WriteBlocks {
            block: 0,
            data: &[1, 2, 3, 4],
        }],
    );
    assert_eq!(
        one_reply(&part),
        vec![rf::ERROR_FLAG, rf::error::NO_INFORMATION]
    );
}

// ---------------------------------------------------------------------------
// The RF face
// ---------------------------------------------------------------------------

#[test]
fn a_command_without_a_field_gets_no_answer_at_all() {
    let (part, _) = st25dv();
    rf_post(&part, &[rf::Command::Inventory]);
    assert_eq!(part.reader.pending(), 0, "a tag out of range says nothing");
    field_on(&part);
    rf_post(&part, &[rf::Command::Inventory]);
    let reply = one_reply(&part);
    // The DSFID, then the eight UID bytes as they travel.
    assert_eq!(reply.len(), 10);
    assert_eq!(reply[0], 0x00, "no error flag");
    assert_eq!(reply[9], 0xe0, "the UID's last byte on the wire is E0h");
}

#[test]
fn rf_reads_and_writes_blocks_and_the_write_costs_a_write_cycle() {
    let (part, bus) = st25dv();
    field_on(&part);
    rf_post(
        &part,
        &[rf::Command::WriteBlocks {
            block: 2,
            data: &[0xde, 0xad, 0xbe, 0xef],
        }],
    );
    assert_eq!(one_reply(&part), vec![0x00]);
    assert_eq!(part.byte(8), Some(0xde));
    // One block is one internal EEPROM page, so one tW — and the I²C side sees
    // it, which is the dual interface doing its job.
    assert!(part.busy());
    assert_eq!(
        bus.start(Address::Seven(USER_ADDRESS), Direction::Read),
        Ack::Nack
    );
    bus.stop();
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    assert!(!part.busy());

    rf_post(&part, &[rf::Command::ReadBlocks { block: 2, count: 1 }]);
    assert_eq!(one_reply(&part), vec![0x00, 0xde, 0xad, 0xbe, 0xef]);

    // A block past the end is "not available" (§7.6.5).
    rf_post(
        &part,
        &[rf::Command::ReadBlocks {
            block: 0xffff,
            count: 1,
        }],
    );
    assert_eq!(
        one_reply(&part),
        vec![rf::ERROR_FLAG, rf::error::NOT_AVAILABLE]
    );
}

#[test]
fn rf_reads_of_an_rf_protected_area_need_the_rf_password() {
    let (part, bus) = st25dv();
    present_password(&bus, &[0; 8]);
    // Table 44: PWD_CTRL_A1 = 01 (RF_PWD_1 opens area 1) and
    // RW_PROTECTION_A1 = 01 (write needs the session open).
    write_ok(&bus, SYSTEM_ADDRESS, sys::RFA1SS, &[0b0000_0101]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    field_on(&part);

    // Read is always allowed for every code the ST25DV defines — the RF
    // asymmetry is on the write side.
    rf_post(&part, &[rf::Command::ReadBlocks { block: 0, count: 1 }]);
    assert_eq!(one_reply(&part), vec![0x00, 0, 0, 0, 0]);

    rf_post(
        &part,
        &[rf::Command::WriteBlocks {
            block: 0,
            data: &[1, 2, 3, 4],
        }],
    );
    assert_eq!(one_reply(&part), vec![rf::ERROR_FLAG, rf::error::LOCKED]);

    // Present RF_PWD_1 — the factory value is zero — and the write lands.
    rf_post(
        &part,
        &[rf::Command::PresentPassword {
            number: 1,
            password: [0; 8],
        }],
    );
    assert_eq!(one_reply(&part), vec![0x00]);
    rf_post(
        &part,
        &[rf::Command::WriteBlocks {
            block: 0,
            data: &[1, 2, 3, 4],
        }],
    );
    assert_eq!(one_reply(&part), vec![0x00]);
    assert_eq!(part.byte(0), Some(1));

    // A wrong password closes the session again (§7.6.36).
    rf_post(
        &part,
        &[rf::Command::PresentPassword {
            number: 1,
            password: [0xff; 8],
        }],
    );
    assert_eq!(
        one_reply(&part),
        vec![rf::ERROR_FLAG, rf::error::NO_INFORMATION]
    );
    rf_post(
        &part,
        &[rf::Command::WriteBlocks {
            block: 0,
            data: &[9, 9, 9, 9],
        }],
    );
    assert_eq!(one_reply(&part), vec![rf::ERROR_FLAG, rf::error::LOCKED]);

    // And the field going away closes it too, because a session lives on the
    // carrier that powers it.
    rf_post(
        &part,
        &[
            rf::Command::PresentPassword {
                number: 1,
                password: [0; 8],
            },
            rf::Command::Field(false),
            rf::Command::Field(true),
            rf::Command::WriteBlocks {
                block: 0,
                data: &[7, 7, 7, 7],
            },
        ],
    );
    let replies = part.reader.drain();
    assert_eq!(replies.len(), 2, "two commands answered, two field events");
    assert_eq!(replies[1], vec![rf::ERROR_FLAG, rf::error::LOCKED]);
}

#[test]
fn the_configuration_registers_the_i2c_host_keeps_to_itself_have_no_rf_pointer() {
    let (part, bus) = st25dv();
    field_on(&part);
    // Table 11: I2CSS (0Bh) and LOCK_CCFILE (0Ch) say "No access" in the RF
    // column, which is what makes the I²C host the security master.
    for pointer in [0x0b_u8, 0x0c] {
        rf_post(&part, &[rf::Command::ReadConfiguration { pointer }]);
        assert_eq!(
            one_reply(&part),
            vec![rf::ERROR_FLAG, rf::error::NOT_AVAILABLE]
        );
    }
    rf_post(&part, &[rf::Command::ReadConfiguration { pointer: 0x01 }]);
    assert_eq!(one_reply(&part), vec![0x00, 0b011], "IT_TIME is readable");

    // Table 12 does the same for the dynamic registers: I2C_SSO_Dyn,
    // IT_STS_Dyn and MB_LEN_Dyn have no RF address.
    for pointer in [0x01_u8, 0x03, 0x04, 0x05, 0x07] {
        rf_post(&part, &[rf::Command::ReadDynamicConfiguration { pointer }]);
        assert_eq!(
            one_reply(&part),
            vec![rf::ERROR_FLAG, rf::error::NOT_AVAILABLE]
        );
    }
    // And GPO_CTRL_Dyn is readable from RF but not writable (Table 29).
    rf_post(
        &part,
        &[rf::Command::ReadDynamicConfiguration { pointer: 0 }],
    );
    assert_eq!(one_reply(&part), vec![0x00, 0x88]);
    rf_post(
        &part,
        &[rf::Command::WriteDynamicConfiguration {
            pointer: 0,
            value: 0,
        }],
    );
    assert_eq!(one_reply(&part), vec![rf::ERROR_FLAG, rf::error::LOCKED]);
    let _ = &bus;
}

#[test]
fn lock_cfg_lets_the_i2c_host_shut_rf_out_of_the_configuration() {
    let (part, bus) = st25dv();
    field_on(&part);
    // §4.3: an RF configuration write needs RF_PWD_0 presented *and*
    // LOCK_CFG = 0.
    rf_post(
        &part,
        &[rf::Command::WriteConfiguration {
            pointer: 0x01,
            value: 0,
        }],
    );
    assert_eq!(one_reply(&part), vec![rf::ERROR_FLAG, rf::error::LOCKED]);

    rf_post(
        &part,
        &[
            rf::Command::PresentPassword {
                number: 0,
                password: [0; 8],
            },
            rf::Command::WriteConfiguration {
                pointer: 0x01,
                value: 0b101,
            },
        ],
    );
    assert_eq!(part.reader.drain(), vec![vec![0x00], vec![0x00]]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::IT_TIME), 0b101);

    // The I²C host locks the configuration…
    present_password(&bus, &[0; 8]);
    write_ok(&bus, SYSTEM_ADDRESS, sys::LOCK_CFG, &[1]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    // …and the reader's session no longer helps.
    rf_post(
        &part,
        &[rf::Command::WriteConfiguration {
            pointer: 0x01,
            value: 0,
        }],
    );
    assert_eq!(one_reply(&part), vec![rf::ERROR_FLAG, rf::error::LOCKED]);
}

#[test]
fn rf_disable_answers_every_command_with_0f_and_stays_quiet_for_inventory() {
    let (part, bus) = st25dv();
    field_on(&part);
    write_ok(&bus, USER_ADDRESS, dyn_reg::RF_MNGT, &[rf_bit::RF_DISABLE]);
    // §5.4.2: "RF commands are interpreted but not executed. In case of a valid
    // command, ST25DVxxx will respond … with the error code 0Fh. The Inventory
    // command is not answered."
    rf_post(&part, &[rf::Command::ReadMessageLength]);
    assert_eq!(
        one_reply(&part),
        vec![rf::ERROR_FLAG, rf::error::NO_INFORMATION]
    );
    rf_post(&part, &[rf::Command::Inventory]);
    assert_eq!(part.reader.pending(), 0);
}

#[test]
fn an_unknown_command_code_is_not_recognized_and_ends_the_payload() {
    let (part, _) = st25dv();
    field_on(&part);
    // A code with no length cannot be skipped, so the tag answers 02h and the
    // rest of the payload is not guessed at — which keeps a replay's decision
    // identical every time.
    let mut payload = alloc::vec![0x55u8, 0x99, 0x99];
    payload.extend_from_slice(&rf::Command::Inventory.encode());
    rf::sink(&part.reader).deliver(&payload);
    assert_eq!(
        one_reply(&part),
        vec![rf::ERROR_FLAG, rf::error::NOT_RECOGNIZED]
    );
}

#[test]
fn a_truncated_command_at_the_end_of_a_payload_does_nothing() {
    let (part, _) = st25dv();
    field_on(&part);
    let mut payload = rf::Command::Inventory.encode();
    // Half a Read Multiple Blocks.
    payload.extend_from_slice(&[rf::code::READ_BLOCKS, 0x00]);
    rf::sink(&part.reader).deliver(&payload);
    assert_eq!(part.reader.pending(), 1, "only the Inventory ran");
    assert_eq!(part.reader.executed(), 2, "the field and the inventory");
}

// ---------------------------------------------------------------------------
// The host door
// ---------------------------------------------------------------------------

#[test]
fn the_reader_door_is_a_channel_the_seal_knows_how_to_wire() {
    use crate::core::hosts::HostObjects;

    let hosts = HostObjects::new();
    let reader = rf::open(&hosts, "nfc").expect("a fresh table takes one");
    assert!(Arc::ptr_eq(
        &reader,
        &rf::open(&hosts, "nfc").expect("the same name")
    ));
    assert_eq!(rf::names(&hosts), vec![alloc::string::String::from("nfc")]);
    assert_eq!(rf::channel("nfc").to_string(), "st25dv-rf:nfc");
    assert!(rf::KIND.is_door(), "a reader is host input");
    assert!(
        rf::KIND.sink_factory().is_some(),
        "and the seal can wire it without being told how"
    );

    // The seal wires it: that is the whole contract, from the `HostObjects`
    // side rather than from a machine's.
    let recorder = Arc::new(Recorder::recording());
    hosts
        .seal(Arc::clone(&recorder))
        .expect("a door with a sink seals");
    assert!(recorder.knows(&rf::channel("nfc")));

    // And an already-sealed table refuses a *new* name, which is what stops a
    // board from opening an unrecorded reader.
    let err = rf::open(&hosts, "other").expect_err("sealed");
    let text = alloc::format!("{err}");
    assert!(text.contains("st25dv-rf:other"), "{text}");
}

#[test]
fn a_reader_with_no_tag_behind_it_discards_what_it_is_given() {
    let reader = Arc::new(rf::Reader::new());
    assert!(!reader.is_bound());
    rf::sink(&reader).deliver(&rf::Command::Inventory.encode());
    assert_eq!(reader.pending(), 0);
    assert_eq!(reader.executed(), 0);
}

#[test]
fn a_payload_carries_several_commands_and_they_are_applied_in_order() {
    let (part, bus) = st25dv();
    enable_mailbox(&part, &bus);
    // One post, four commands: the field, a message, its length, and it back.
    rf_post(
        &part,
        &[
            rf::Command::Field(true),
            rf::Command::WriteMessage(&[0xaa, 0xbb]),
            rf::Command::ReadMessageLength,
            rf::Command::ReadMessage {
                offset: 1,
                count: 1,
            },
        ],
    );
    assert_eq!(
        part.reader.drain(),
        vec![vec![0x00], vec![0x00, 1], vec![0x00, 0xbb],]
    );
    assert_eq!(part.reader.executed(), 4);
}

// ---------------------------------------------------------------------------
// Snapshot
// ---------------------------------------------------------------------------

#[test]
fn eeprom_and_dynamic_state_survive_the_snapshot_and_the_field_does_not() {
    let (part, bus) = st25dv();
    // The EEPROM write comes first: §6.4's caution shuts that path the moment
    // fast transfer mode is enabled.
    present_password(&bus, &[0; 8]);
    write_ok(&bus, USER_ADDRESS, 0x0040, &[1, 2, 3, 4]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    enable_mailbox(&part, &bus);
    field_on(&part);
    rf_post(&part, &[rf::Command::WriteMessage(&[9, 8, 7])]);
    part.reader.clear();
    assert!(part.field());

    let mut shape = MachineShape::new();
    shape.add_device("tag", ST25DV_CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("tag", ST25DV_CLASS.name, ST25DV_CLASS.version)
            .unwrap();
        part.save(&mut chunk).unwrap();
    }
    let bytes = w.to_vec().unwrap();

    let (other, _) = st25dv();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load(
            "tag",
            ST25DV_CLASS.name,
            ST25DV_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    other.load(&mut chunk.reader()).unwrap();

    // The EEPROM and every dynamic register came across.
    assert_eq!(other.contents(), part.contents());
    assert_eq!(
        other.register(false, dyn_reg::MB_CTRL),
        part.register(false, dyn_reg::MB_CTRL)
    );
    assert!(other.session_open(), "the security session is tag state");
    // The field did not: it is the reader's, not the tag's.
    assert!(!other.field(), "a snapshot has no reader over it");
    assert_eq!(
        other.register(false, dyn_reg::EH_CTRL) & eh_bit::FIELD_ON,
        0
    );

    // The identical-state-hash property, checked where it is observable: save
    // the loaded part and the bytes must match, once the field is put back.
    rf::sink(other.reader()).deliver(&rf::Command::Field(true).encode());
    other.reader().clear();
    let mut shape = MachineShape::new();
    shape.add_device("tag", ST25DV_CLASS.name).unwrap();
    let mut w2 = StateWriter::new(shape);
    {
        let mut chunk = w2
            .chunk("tag", ST25DV_CLASS.name, ST25DV_CLASS.version)
            .unwrap();
        other.save(&mut chunk).unwrap();
    }
    assert_eq!(w2.to_vec().unwrap(), bytes);
}

#[test]
fn a_reset_restores_every_dynamic_register_from_its_static_image() {
    let (part, bus) = st25dv();
    present_password(&bus, &[0; 8]);
    write_ok(&bus, SYSTEM_ADDRESS, sys::EH_MODE, &[0]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    write_ok(&bus, USER_ADDRESS, dyn_reg::GPO_CTRL, &[0]);
    assert!(part.session_open());

    part.reset(ResetKind::Cold);
    // §4.4: "A dynamic configuration register updated by the application will
    // recover its default static value after a Power On Reset."
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::GPO_CTRL),
        gpo_bit::FIELD_CHANGE_EN | gpo_bit::GPO_EN
    );
    // Table 38: EH_MODE = 0 means EH_EN boots set.
    assert_eq!(
        read_one(&bus, USER_ADDRESS, dyn_reg::EH_CTRL) & eh_bit::EH_EN,
        eh_bit::EH_EN
    );
    assert!(!part.session_open(), "a session does not survive a reset");
    // The EEPROM does: it is an EEPROM.
    assert_eq!(read_one(&bus, SYSTEM_ADDRESS, sys::EH_MODE), 0);
}

#[test]
fn a_channel_and_a_recorder_carry_a_reader_session() {
    // The seam, at its smallest: post on the channel, deliver at an instant,
    // and the tag has moved. `tests/st25dv_replay.rs` makes the machine-level
    // claim; this one is about the wiring.
    let (part, _) = st25dv();
    let recorder = Arc::new(Recorder::recording());
    let channel: Channel = rf::channel("nfc");
    recorder
        .register(channel.clone(), rf::sink(&part.reader))
        .expect("a fresh recorder takes channels");
    recorder
        .post(&channel, &rf::Command::Field(true).encode())
        .expect("a registered channel");
    assert!(!part.field(), "nothing is delivered until the machine says");
    recorder
        .deliver(crate::core::clock::GlobalTime::from_nanos(1_000))
        .expect("a round boundary");
    assert!(part.field());
    assert_eq!(recorder.log().len(), 1);
}

#[test]
fn area_1_is_always_readable_from_rf_and_the_other_three_are_not() {
    let (part, bus) = st25dv();
    present_password(&bus, &[0; 8]);
    // Two areas: §4.2's ENDA1 = 0 puts the border after byte 31, so block 8 is
    // the first block of area 2.
    write_ok(&bus, SYSTEM_ADDRESS, sys::ENDA1, &[0x00]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    // Table 46: PWD_CTRL_A2 = 01 (RF_PWD_1) and RW_PROTECTION_A2 = 10, which is
    // "Read allowed if RF user security session is open".
    write_ok(&bus, SYSTEM_ADDRESS, sys::RFA2SS, &[0b0000_1001]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    // The same code in RFA1SS, which Table 44 still calls "Read always
    // allowed" — §4.2: "Area1 is always readable".
    write_ok(&bus, SYSTEM_ADDRESS, sys::RFA1SS, &[0b0000_1001]);
    part.advance_to(part.ticks() + DEFAULT_WRITE_TICKS);
    field_on(&part);

    rf_post(&part, &[rf::Command::ReadBlocks { block: 0, count: 1 }]);
    assert_eq!(one_reply(&part), vec![0x00, 0, 0, 0, 0], "area 1");
    rf_post(&part, &[rf::Command::ReadBlocks { block: 8, count: 1 }]);
    assert_eq!(
        one_reply(&part),
        vec![rf::ERROR_FLAG, rf::error::READ_PROTECTED],
        "area 2"
    );

    rf_post(
        &part,
        &[
            rf::Command::PresentPassword {
                number: 1,
                password: [0; 8],
            },
            rf::Command::ReadBlocks { block: 8, count: 1 },
        ],
    );
    assert_eq!(
        part.reader.drain(),
        vec![vec![0x00], vec![0x00, 0, 0, 0, 0]]
    );
}
