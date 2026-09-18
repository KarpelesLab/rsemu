//! Tests for the ATECC508A/608A/608B model.
//!
//! Written against the datasheets, section by section, and against the
//! standards the primitives come from: every assertion names what it is
//! checking, so a disagreement is either a bug here or a misreading of that
//! paragraph and nothing else.
//!
//! Two of them are worth pointing at before the rest.
//!
//! * [`sign_verifies_against_an_independent_witness_and_the_rfc_6979_vector`]
//!   does not ask the device whether its own signature is good. It checks the
//!   bytes with `purecrypto`'s verifier *and* against RFC 6979 A.2.5's
//!   published P-256/SHA-256 vector — because a consistently wrong curve would
//!   pass a device that only verifies itself.
//! * [`two_runs_with_the_same_seed_produce_the_same_random_and_the_same_key`]
//!   is the determinism claim the whole seed design exists for.

use super::*;

use alloc::vec;
use alloc::vec::Vec;

use crate::bus::i2c::wires::{MasterEvent, MasterOp, MasterWires, pin as line};
use crate::bus::swi::{BAUD, tokens_of};
use crate::core::device::{Deferred, ResetKind};
use crate::core::hosts::HostObjects;
use crate::core::props::{Media, Value};
use crate::core::space::RequesterId;
use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use crate::core::wire::{Wire, WireId, WireSource};

// ---------------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------------

/// The symmetric key this fixture puts in slot 0, so a test can compute what
/// the part is going to answer without asking the part.
const KEY0: [u8; 32] = [
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
    0x0f, 0x1e, 0x2d, 0x3c, 0x4b, 0x5a, 0x69, 0x78, 0x87, 0x96, 0xa5, 0xb4, 0xc3, 0xd2, 0xe1, 0xf0,
];

/// A configuration image with the slots the tests need.
///
/// * slot 0 — a symmetric secret: `IsSecret`, so a `Read` of it is refused.
/// * slots 1 and 2 — P-256 private keys (`KeyConfig.Private`), secret too.
/// * slot 3 — `WriteConfig` = never.
/// * slot 9 — a 72-byte public-key slot, readable and writable.
fn config_image() -> Vec<u8> {
    let mut config = vec![0u8; CONFIG_BYTES];
    let mut set_slot = |slot: usize, slot_config: u16, key_config: u16| {
        let at = SLOTCONFIG_OFFSET + slot * 2;
        config[at] = (slot_config & 0xff) as u8;
        config[at + 1] = (slot_config >> 8) as u8;
        let at = KEYCONFIG_OFFSET + slot * 2;
        config[at] = (key_config & 0xff) as u8;
        config[at + 1] = (key_config >> 8) as u8;
    };
    // KeyConfig: bit 0 `Private`, bits 4:2 `KeyType` — 4 is a P-256 key, 6 a
    // SHA/symmetric one (§2.2.6).
    set_slot(0, 0x0080, 6 << 2);
    set_slot(1, 0x0080, 0x0001 | (4 << 2));
    set_slot(2, 0x0080, 0x0001 | (4 << 2));
    // WriteConfig lives in bits 15:12; anything with bit 2 of it set forbids a
    // write outright (§2.2.5).
    set_slot(3, 0x4000, 6 << 2);
    set_slot(9, 0x0000, 4 << 2);
    config
}

/// A data image with [`KEY0`] in slot 0.
fn data_image() -> Vec<u8> {
    let mut data = vec![0u8; DATA_BYTES];
    data[0..32].copy_from_slice(&KEY0);
    data
}

/// A part with the given properties, on a bus nothing else can reach, realized
/// at the path `atecc` so its stream is seeded.
fn build(props: &[(&str, Value)]) -> (Atecc, Arc<I2cBus>) {
    let mut p = Props::new();
    for (name, value) in props {
        p.insert(*name, value.clone());
    }
    let dev = Atecc::new(&p).expect("it builds");
    dev.seed_from_path("atecc");
    let bus = Arc::new(I2cBus::new());
    bus.attach(dev.slave()).expect("room on the bus");
    (dev, bus)
}

/// A blank 608A: nothing locked, nothing configured.
fn blank() -> (Atecc, Arc<I2cBus>) {
    build(&[("watchdog-ticks", Value::Uint(0))])
}

/// A provisioned 608A: the fixture configuration, both zones locked.
///
/// `watchdog-ticks = 0` holds it awake for a whole test. That is a property of
/// the *model*, not of the part — no ATECC can disable its watchdog — and it is
/// here so that the sequences a command test needs (`Nonce`, `GenDig`, `MAC`)
/// are not interrupted by a timer whose own tests are below.
fn provisioned() -> (Atecc, Arc<I2cBus>) {
    build(&[
        ("config", Value::Media(Media::new("config", config_image()))),
        ("data", Value::Media(Media::new("data", data_image()))),
        ("lock", Value::Str("data".into())),
        ("watchdog-ticks", Value::Uint(0)),
    ])
}

/// The same fixture with only the configuration zone locked, which is the state
/// a part is in while it is being provisioned: `PrivWrite` still works.
fn half_locked() -> (Atecc, Arc<I2cBus>) {
    build(&[
        ("config", Value::Media(Media::new("config", config_image()))),
        ("data", Value::Media(Media::new("data", data_image()))),
        ("lock", Value::Str("config".into())),
        ("watchdog-ticks", Value::Uint(0)),
    ])
}

// ---------------------------------------------------------------------------
// The transport, as firmware drives it
// ---------------------------------------------------------------------------

/// The wake pulse: address `0x00`, which the part never acknowledges (§6.1).
fn wake(bus: &I2cBus) {
    assert_eq!(
        bus.start(Address::Seven(WAKE_ADDRESS), Direction::Write),
        Ack::Nack,
        "§6.1: the part does not acknowledge the wake token"
    );
    bus.stop();
}

/// Write a word address and, if `body` is not empty, what follows it (§7.1).
fn write_word(bus: &I2cBus, word: u8, body: &[u8]) {
    assert_eq!(
        bus.start(Address::Seven(DEFAULT_ADDRESS), Direction::Write),
        Ack::Ack,
        "the part did not answer its own address"
    );
    assert_eq!(bus.write(word), Ack::Ack);
    for byte in body {
        assert_eq!(bus.write(*byte), Ack::Ack);
    }
    bus.stop();
}

/// Read the response packet: the count byte, then the rest of it (§9.1.2).
fn read_packet(bus: &I2cBus) -> Vec<u8> {
    assert_eq!(
        bus.start(Address::Seven(DEFAULT_ADDRESS), Direction::Read),
        Ack::Ack,
        "the part has nothing to say"
    );
    let count = bus.read(Ack::Ack);
    let mut out = vec![count];
    for i in 1..usize::from(count) {
        let last = i + 1 == usize::from(count);
        out.push(bus.read(if last { Ack::Nack } else { Ack::Ack }));
    }
    bus.stop();
    out
}

/// Check a response packet's framing and hand back its body (§9.1.2).
fn body(packet: &[u8]) -> Vec<u8> {
    assert!(packet.len() >= 3, "a response is count, body and CRC");
    assert_eq!(
        usize::from(packet[0]),
        packet.len(),
        "the count byte covers the whole packet"
    );
    let crc = crc16(&packet[..packet.len() - 2]);
    assert_eq!(
        crc,
        packet[packet.len() - 2..],
        "the part CRCs what it sends"
    );
    packet[1..packet.len() - 2].to_vec()
}

/// Send a command, let it execute, and read the answer back.
fn call(dev: &Atecc, bus: &I2cBus, packet: &[u8]) -> Vec<u8> {
    write_word(bus, WORD_COMMAND, packet);
    assert!(dev.busy(), "§9.4: a command takes time");
    dev.advance_to(dev.ticks() + 1_000_000);
    body(&read_packet(bus))
}

/// The same, for a command whose answer is one status byte.
fn status(dev: &Atecc, bus: &I2cBus, packet: &[u8]) -> u8 {
    let out = call(dev, bus, packet);
    assert_eq!(out.len(), 1, "expected a status byte, got {out:02x?}");
    out[0]
}

// ---------------------------------------------------------------------------
// The packet layer
// ---------------------------------------------------------------------------

#[test]
fn the_crc_of_the_wake_token_is_the_datasheet_s_example() {
    // §9.1.3's own worked example, and the reason this function can be trusted
    // at all: the wake token is `04 11` followed by its CRC.
    assert_eq!(crc16(&[0x04, 0x11]), [0x33, 0x43]);
    assert_eq!(crc16(&[]), [0x00, 0x00]);
}

#[test]
fn a_wake_pulse_yields_04_11_33_43_and_nothing_before_it() {
    let (dev, bus) = blank();
    // §6.2: asleep, the part is not on the bus at all.
    assert_eq!(
        bus.start(Address::Seven(DEFAULT_ADDRESS), Direction::Read),
        Ack::Nack
    );
    bus.stop();
    assert!(!dev.awake());

    wake(&bus);
    assert!(dev.awake());
    assert_eq!(read_packet(&bus), WAKE_TOKEN.to_vec());
    // And the body of that packet is the "after wake" status of §9.3.
    assert_eq!(body(&WAKE_TOKEN), vec![STATUS_AFTER_WAKE]);
}

#[test]
fn a_packet_with_a_bad_crc_is_answered_0xff() {
    let (dev, bus) = blank();
    wake(&bus);
    let _ = read_packet(&bus);
    let mut packet = command(OP_INFO, 0x00, 0x0000, &[]);
    let last = packet.len() - 1;
    packet[last] ^= 0xff;
    // §9.1.3: a packet whose CRC does not check out is not executed.
    assert_eq!(status(&dev, &bus, &packet), STATUS_CRC);
}

#[test]
fn a_command_this_model_does_not_implement_says_so_rather_than_guessing() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    // `DeriveKey`, `KDF` and `SecureBoot` are the three the module docs admit
    // to. A parse error is the honest answer; an invented message layout would
    // not be.
    for op in [OP_DERIVEKEY, OP_KDF, OP_SECUREBOOT] {
        assert_eq!(
            status(&dev, &bus, &command(op, 0x00, 0x0000, &[0u8; 32])),
            STATUS_PARSE
        );
    }
}

// ---------------------------------------------------------------------------
// Identity
// ---------------------------------------------------------------------------

#[test]
fn info_revision_returns_the_part_s_revnum() {
    for (name, revnum) in [
        ("atecc508a", [0x00, 0x00, 0x50, 0x00]),
        ("atecc608a", [0x00, 0x00, 0x60, 0x02]),
        ("atecc608b", [0x00, 0x00, 0x60, 0x03]),
    ] {
        let (dev, bus) = build(&[
            ("part", Value::Str(name.into())),
            ("watchdog-ticks", Value::Uint(0)),
        ]);
        wake(&bus);
        let _ = read_packet(&bus);
        // §9.2, `Info` mode 0: the one command that is legal on a blank part
        // and identifies the silicon.
        assert_eq!(
            call(&dev, &bus, &command(OP_INFO, 0x00, 0x0000, &[])),
            revnum
        );
    }
}

#[test]
fn a_config_zone_read_of_word_0_shows_the_serial_number_prefix_01_23() {
    let (dev, bus) = blank();
    wake(&bus);
    let _ = read_packet(&bus);
    // §2.2.1: SN[0:1] is 01 23 on every part ever made, and SN[8] is 0xEE.
    let first = call(&dev, &bus, &command(OP_READ, 0x00, 0x0000, &[]));
    assert_eq!(&first[0..2], &[0x01, 0x23]);
    // Bytes 4..8 are RevNum, and 8..13 the rest of the serial number.
    let block = call(&dev, &bus, &command(OP_READ, 0x80, 0x0000, &[]));
    assert_eq!(&block[4..8], &[0x00, 0x00, 0x60, 0x02], "a 608A");
    assert_eq!(block[12], 0xee, "SN[8]");
    assert_eq!(dev.serial()[8], 0xee);
    assert_eq!(&dev.serial()[0..2], &[0x01, 0x23]);
    // §2.2.4: the address byte holds the seven-bit address in bits 7:1.
    assert_eq!(block[16], DEFAULT_ADDRESS << 1);
}

// ---------------------------------------------------------------------------
// The zones and their locks
// ---------------------------------------------------------------------------

#[test]
fn lock_config_with_a_wrong_crc_fails_and_with_the_right_one_flips_lockconfig_to_0x00() {
    let (dev, bus) = blank();
    wake(&bus);
    let _ = read_packet(&bus);
    assert_eq!(dev.config()[LOCKCONFIG_OFFSET], UNLOCKED);

    // §9.2, `Lock`: `param2` is a CRC-16 of the zone being frozen, so the host
    // and the part have to agree about what is in it.
    assert_eq!(
        status(&dev, &bus, &command(OP_LOCK, 0x00, 0x1234, &[])),
        STATUS_MISCOMPARE
    );
    assert_eq!(dev.config()[LOCKCONFIG_OFFSET], UNLOCKED, "still unlocked");

    let summary = u16::from_le_bytes(crc16(&dev.config()));
    assert_eq!(status(&dev, &bus, &command(OP_LOCK, 0x00, summary, &[])), 0);
    assert_eq!(dev.config()[LOCKCONFIG_OFFSET], LOCKED);
    // §2.2: and it only happens once.
    assert_eq!(
        status(&dev, &bus, &command(OP_LOCK, 0x00, summary, &[])),
        STATUS_EXEC
    );
}

#[test]
fn the_data_zone_cannot_be_locked_first_or_read_before_it_is() {
    let (dev, bus) = blank();
    wake(&bus);
    let _ = read_packet(&bus);
    // §2.2: the configuration zone locks first — the data zone's rules live in
    // it, and freezing the data under changeable rules would mean nothing.
    assert_eq!(
        status(&dev, &bus, &command(OP_LOCK, 0x81, 0x0000, &[])),
        STATUS_EXEC
    );
    // §2.1: and nothing may be read out of the data zone until it is locked.
    assert_eq!(
        status(&dev, &bus, &command(OP_READ, 0x82, 0x0000, &[])),
        STATUS_EXEC
    );
}

#[test]
fn reading_an_issecret_slot_is_an_execution_error_and_writing_a_never_slot_too() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);

    // §2.2.5: `SlotConfig.IsSecret` means the slot never leaves the part in
    // the clear. Slot 0 holds the symmetric key the MAC tests use.
    assert_eq!(
        status(&dev, &bus, &command(OP_READ, 0x82, 0x0000, &[])),
        STATUS_EXEC
    );
    // Slot 9 is not secret, so the same command against it works.
    let out = call(&dev, &bus, &command(OP_READ, 0x82, 0x0048, &[]));
    assert_eq!(out.len(), 32);

    // `WriteConfig` = never, on slot 3.
    let payload = [0xa5u8; 32];
    assert_eq!(
        status(&dev, &bus, &command(OP_WRITE, 0x82, 0x0018, &payload)),
        STATUS_EXEC
    );
    // And the same write into slot 9 lands.
    assert_eq!(
        status(&dev, &bus, &command(OP_WRITE, 0x82, 0x0048, &payload)),
        STATUS_OK
    );
    assert_eq!(&dev.slot(9).unwrap()[0..32], &payload);
}

#[test]
fn the_first_sixteen_config_bytes_and_the_lock_bytes_are_not_writable() {
    let (dev, bus) = blank();
    wake(&bus);
    let _ = read_packet(&bus);
    // §2.2: bytes 0 through 15 are the serial number, the revision and the
    // interface, and no host writes them.
    assert_eq!(
        status(&dev, &bus, &command(OP_WRITE, 0x00, 0x0000, &[1, 2, 3, 4])),
        STATUS_EXEC
    );
    // The lock bytes belong to `Lock`.
    assert_eq!(
        status(
            &dev,
            &bus,
            &command(OP_WRITE, 0x00, 0x0205, &[0x00, 0x00, 0xff, 0xff])
        ),
        STATUS_EXEC
    );
    // A configuration byte above the fixed head is writable while unlocked...
    assert_eq!(
        status(
            &dev,
            &bus,
            &command(OP_WRITE, 0x00, 0x0005, &[0xaa, 0xbb, 0xcc, 0xdd])
        ),
        STATUS_OK
    );
    assert_eq!(dev.config()[20], 0xaa);
    // ...and not after (§2.2: `UpdateExtra` is the only way in).
    let summary = u16::from_le_bytes(crc16(&dev.config()));
    assert_eq!(status(&dev, &bus, &command(OP_LOCK, 0x00, summary, &[])), 0);
    assert_eq!(
        status(
            &dev,
            &bus,
            &command(OP_WRITE, 0x00, 0x0005, &[0x11, 0x22, 0x33, 0x44])
        ),
        STATUS_EXEC
    );
    // `UpdateExtra` moves `UserExtra` off zero exactly once.
    assert_eq!(
        status(&dev, &bus, &command(OP_UPDATEEXTRA, 0x00, 0x0042, &[])),
        STATUS_OK
    );
    assert_eq!(dev.config()[USEREXTRA_OFFSET], 0x42);
    assert_eq!(
        status(&dev, &bus, &command(OP_UPDATEEXTRA, 0x00, 0x0043, &[])),
        STATUS_EXEC
    );
}

// ---------------------------------------------------------------------------
// Randomness, counters
// ---------------------------------------------------------------------------

#[test]
fn random_before_the_config_zone_is_locked_is_the_fixed_pattern_and_after_is_seeded() {
    let (dev, bus) = blank();
    wake(&bus);
    let _ = read_packet(&bus);
    // §9.2, `Random`: an unlocked part does not generate random numbers, it
    // answers `FFFF0000` over and over — which is what stops a provisioning
    // script from mistaking a blank part for a working one.
    let fixed = call(&dev, &bus, &command(OP_RANDOM, 0x00, 0x0000, &[]));
    assert_eq!(fixed.len(), 32);
    for chunk in fixed.chunks(4) {
        assert_eq!(chunk, [0xff, 0xff, 0x00, 0x00]);
    }

    let summary = u16::from_le_bytes(crc16(&dev.config()));
    assert_eq!(status(&dev, &bus, &command(OP_LOCK, 0x00, summary, &[])), 0);
    let first = call(&dev, &bus, &command(OP_RANDOM, 0x00, 0x0000, &[]));
    let second = call(&dev, &bus, &command(OP_RANDOM, 0x00, 0x0000, &[]));
    assert_ne!(first, fixed, "a locked part draws from the stream");
    assert_ne!(first, second, "and moves along it");
}

#[test]
fn counter_increments_and_saturates_at_2_097_151() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    let read = |dev: &Atecc| {
        let out = call(dev, &bus, &command(OP_COUNTER, 0x00, 0x0000, &[]));
        u32::from_le_bytes([out[0], out[1], out[2], out[3]])
    };
    let bump = |dev: &Atecc| call(dev, &bus, &command(OP_COUNTER, 0x01, 0x0000, &[]));

    assert_eq!(read(&dev), 0);
    assert_eq!(bump(&dev), 1u32.to_le_bytes());
    assert_eq!(bump(&dev), 2u32.to_le_bytes());
    assert_eq!(read(&dev), 2);
    // The other counter is its own.
    let other = call(&dev, &bus, &command(OP_COUNTER, 0x00, 0x0001, &[]));
    assert_eq!(other, 0u32.to_le_bytes());
    // There is no third one.
    assert_eq!(
        status(&dev, &bus, &command(OP_COUNTER, 0x00, 0x0002, &[])),
        STATUS_PARSE
    );

    // §9.2: the counter is 21 bits and monotonic. Rather than clock two million
    // commands through the bus, put it one short of the limit and check the
    // wall is there.
    dev.set_counter_for_test(0, COUNTER_MAX - 1);
    assert_eq!(bump(&dev), COUNTER_MAX.to_le_bytes());
    assert_eq!(
        status(&dev, &bus, &command(OP_COUNTER, 0x01, 0x0000, &[])),
        STATUS_EXEC,
        "at the limit the increment fails rather than wrapping"
    );
    assert_eq!(read(&dev), COUNTER_MAX);
}

// ---------------------------------------------------------------------------
// The digest commands
// ---------------------------------------------------------------------------

#[test]
fn sha_start_update_end_of_abc_matches_fips_180_4() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    // FIPS 180-4's own worked example: SHA-256("abc").
    const ABC: [u8; 32] = [
        0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22,
        0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00,
        0x15, 0xad,
    ];
    assert_eq!(
        status(&dev, &bus, &command(OP_SHA, 0x00, 0x0000, &[])),
        STATUS_OK
    );
    let digest = call(&dev, &bus, &command(OP_SHA, 0x02, 0x0003, b"abc"));
    assert_eq!(digest, ABC.to_vec());

    // And a message that needs a whole 64-byte block through `Update`.
    let block = [0x61u8; 64];
    assert_eq!(
        status(&dev, &bus, &command(OP_SHA, 0x00, 0x0000, &[])),
        STATUS_OK
    );
    assert_eq!(
        status(&dev, &bus, &command(OP_SHA, 0x01, 0x0040, &block)),
        STATUS_OK
    );
    let digest = call(&dev, &bus, &command(OP_SHA, 0x02, 0x0003, b"abc"));
    let mut expect = Vec::new();
    expect.extend_from_slice(&block);
    expect.extend_from_slice(b"abc");
    assert_eq!(digest, Sha256::digest(&expect).to_vec());
    // §9.2: the digest also lands in TempKey, which is what lets a host sign a
    // message longer than a nonce.
    assert_eq!(dev.temp_key_for_test(), Some(Sha256::digest(&expect)));
}

#[test]
fn nonce_then_gendig_then_mac_reproduces_the_datasheet_message_construction() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);

    // A pass-through nonce, so the test knows exactly what TempKey holds
    // (§9.2, `Nonce` mode 3).
    let nonce = [0x5au8; 32];
    assert_eq!(
        status(&dev, &bus, &command(OP_NONCE, 0x03, 0x0000, &nonce)),
        STATUS_OK
    );

    // `GenDig` on slot 0 folds the key into TempKey. §9.2's table:
    //   SHA-256(KeyValue ‖ Opcode ‖ Zone ‖ KeyID[2] ‖ SN[8] ‖ SN[0:1] ‖ 0×25
    //           ‖ TempKey)
    assert_eq!(
        status(&dev, &bus, &command(OP_GENDIG, 0x02, 0x0000, &[])),
        STATUS_OK
    );
    let serial = dev.serial();
    let mut msg = Vec::new();
    msg.extend_from_slice(&KEY0);
    msg.push(OP_GENDIG);
    msg.push(0x02);
    msg.extend_from_slice(&[0x00, 0x00]);
    msg.push(serial[8]);
    msg.extend_from_slice(&serial[0..2]);
    msg.extend_from_slice(&[0u8; 25]);
    msg.extend_from_slice(&nonce);
    assert_eq!(msg.len(), 96, "§9.2's GenDig message is 96 bytes");
    let expect_temp = Sha256::digest(&msg);
    assert_eq!(dev.temp_key_for_test(), Some(expect_temp));

    // And now a `MAC` with both halves taken from TempKey, whose message is
    // §9.2's 88-byte layout. Mode bit 2 has to agree with TempKey.SourceFlag,
    // which is "input" here because the nonce was a pass-through one.
    let mode = 0x07u8;
    let out = call(&dev, &bus, &command(OP_MAC, mode, 0x0000, &[]));
    let mut msg = Vec::new();
    msg.extend_from_slice(&expect_temp);
    msg.extend_from_slice(&expect_temp);
    msg.push(OP_MAC);
    msg.push(mode);
    msg.extend_from_slice(&[0x00, 0x00]);
    msg.extend_from_slice(&[0u8; 8]);
    msg.extend_from_slice(&[0u8; 3]);
    msg.push(serial[8]);
    msg.extend_from_slice(&[0u8; 4]);
    msg.extend_from_slice(&serial[0..2]);
    msg.extend_from_slice(&[0u8; 2]);
    assert_eq!(msg.len(), 88, "§9.2's MAC message is 88 bytes");
    assert_eq!(out, Sha256::digest(&msg).to_vec());

    // The source-flag check itself: claiming the nonce was internally
    // generated when it was not is refused.
    assert_eq!(
        status(&dev, &bus, &command(OP_MAC, 0x03, 0x0000, &[])),
        STATUS_EXEC
    );
}

#[test]
fn a_mac_over_a_challenge_uses_the_slot_key_and_checkmac_agrees_with_it() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    let serial = dev.serial();

    // Mode 0: the key from the slot, the challenge from the packet.
    let challenge = [0x33u8; 32];
    let out = call(&dev, &bus, &command(OP_MAC, 0x00, 0x0000, &challenge));
    let mut msg = Vec::new();
    msg.extend_from_slice(&KEY0);
    msg.extend_from_slice(&challenge);
    msg.push(OP_MAC);
    msg.push(0x00);
    msg.extend_from_slice(&[0x00, 0x00]);
    msg.extend_from_slice(&[0u8; 11]);
    msg.push(serial[8]);
    msg.extend_from_slice(&[0u8; 4]);
    msg.extend_from_slice(&serial[0..2]);
    msg.extend_from_slice(&[0u8; 2]);
    assert_eq!(out, Sha256::digest(&msg).to_vec());

    // `CheckMac` is the same message with the client's own copies of the
    // trailing fields. A client that computed it correctly gets 0x00 back and
    // one that did not gets the miscompare status of §9.3.
    let other = [0u8; 13];
    let mut msg = Vec::new();
    msg.extend_from_slice(&KEY0);
    msg.extend_from_slice(&challenge);
    msg.extend_from_slice(&other[0..4]);
    msg.extend_from_slice(&[0u8; 8]);
    msg.extend_from_slice(&other[4..7]);
    msg.push(serial[8]);
    msg.extend_from_slice(&other[7..11]);
    msg.extend_from_slice(&serial[0..2]);
    msg.extend_from_slice(&other[11..13]);
    let response = Sha256::digest(&msg);
    let mut data = Vec::new();
    data.extend_from_slice(&challenge);
    data.extend_from_slice(response.as_ref());
    data.extend_from_slice(&other);
    assert_eq!(
        status(&dev, &bus, &command(OP_CHECKMAC, 0x00, 0x0000, &data)),
        STATUS_OK
    );

    data[40] ^= 0x01;
    assert_eq!(
        status(&dev, &bus, &command(OP_CHECKMAC, 0x00, 0x0000, &data)),
        STATUS_MISCOMPARE
    );
}

#[test]
fn hmac_is_rfc_2104_over_the_mac_layout_and_the_608_does_not_have_it() {
    let (dev, bus) = build(&[
        ("part", Value::Str("atecc508a".into())),
        ("config", Value::Media(Media::new("config", config_image()))),
        ("data", Value::Media(Media::new("data", data_image()))),
        ("lock", Value::Str("data".into())),
        ("watchdog-ticks", Value::Uint(0)),
    ]);
    wake(&bus);
    let _ = read_packet(&bus);
    let nonce = [0x5au8; 32];
    assert_eq!(
        status(&dev, &bus, &command(OP_NONCE, 0x03, 0x0000, &nonce)),
        STATUS_OK
    );
    let out = call(&dev, &bus, &command(OP_HMAC, 0x04, 0x0000, &[]));
    let serial = dev.serial();
    let mut msg = Vec::new();
    msg.extend_from_slice(&[0u8; 32]);
    msg.extend_from_slice(&nonce);
    msg.push(OP_HMAC);
    msg.push(0x04);
    msg.extend_from_slice(&[0x00, 0x00]);
    msg.extend_from_slice(&[0u8; 11]);
    msg.push(serial[8]);
    msg.extend_from_slice(&[0u8; 4]);
    msg.extend_from_slice(&serial[0..2]);
    msg.extend_from_slice(&[0u8; 2]);
    assert_eq!(out, HmacSha256::mac(&KEY0, &msg).to_vec());

    // The 608 dropped the command (`KDF` took its place).
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    assert_eq!(
        status(&dev, &bus, &command(OP_HMAC, 0x04, 0x0000, &[])),
        STATUS_PARSE
    );
}

#[test]
fn a_random_nonce_hashes_the_datasheet_s_fifty_five_byte_message() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    let num_in = [0x11u8; 20];
    let rand_out = call(&dev, &bus, &command(OP_NONCE, 0x00, 0x0000, &num_in));
    assert_eq!(rand_out.len(), 32);
    // §9.2's `Nonce` table: TempKey = SHA-256(RandOut ‖ NumIn ‖ Opcode ‖ Mode
    // ‖ LSB(Param2)).
    let mut msg = Vec::new();
    msg.extend_from_slice(&rand_out);
    msg.extend_from_slice(&num_in);
    msg.push(OP_NONCE);
    msg.push(0x00);
    msg.push(0x00);
    assert_eq!(msg.len(), 55);
    assert_eq!(dev.temp_key_for_test(), Some(Sha256::digest(&msg)));
}

// ---------------------------------------------------------------------------
// The public-key commands
// ---------------------------------------------------------------------------

#[test]
fn genkey_then_sign_then_verify_round_trips_on_p256() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);

    // §9.2, `GenKey` mode 0x04: a new private key in slot 1, and its public key
    // on the wire as X ‖ Y.
    let public = call(&dev, &bus, &command(OP_GENKEY, 0x04, 0x0001, &[]));
    assert_eq!(public.len(), 64);
    // Mode 0x00 recomputes the public key from the stored private one, which
    // has to give the same answer.
    let again = call(&dev, &bus, &command(OP_GENKEY, 0x00, 0x0001, &[]));
    assert_eq!(again, public);

    // Something to sign: a digest in TempKey.
    let digest = Sha256::digest(b"the message the host wants signed");
    assert_eq!(
        status(
            &dev,
            &bus,
            &command(OP_NONCE, 0x03, 0x0000, digest.as_ref())
        ),
        STATUS_OK
    );
    let sig = call(&dev, &bus, &command(OP_SIGN, 0x80, 0x0001, &[]));
    assert_eq!(sig.len(), 64);

    // The device's own `Verify`, external mode: signature then public key.
    let mut data = sig.clone();
    data.extend_from_slice(&public);
    assert_eq!(
        status(&dev, &bus, &command(OP_VERIFY, 0x02, 0x0000, &data)),
        STATUS_OK
    );
    // A tampered signature is a miscompare, not a success.
    let mut bad = data.clone();
    bad[0] ^= 0x01;
    assert_eq!(
        status(&dev, &bus, &command(OP_VERIFY, 0x02, 0x0000, &bad)),
        STATUS_MISCOMPARE
    );

    // And `Verify` stored mode, against a public key written into slot 9. §2.1
    // pads X and Y to 36 bytes apiece with four leading zeros.
    let mut block0 = [0u8; 32];
    block0[4..32].copy_from_slice(&public[0..28]);
    let mut block1 = [0u8; 32];
    block1[0..4].copy_from_slice(&public[28..32]);
    block1[8..32].copy_from_slice(&public[32..56]);
    assert_eq!(
        status(&dev, &bus, &command(OP_WRITE, 0x82, 0x0048, &block0)),
        STATUS_OK
    );
    assert_eq!(
        status(&dev, &bus, &command(OP_WRITE, 0x82, 0x0148, &block1)),
        STATUS_OK
    );
    // The slot is 72 bytes, so the last eight are two four-byte writes rather
    // than a third block — which is itself the §2.1 geometry being checked.
    assert_eq!(
        status(
            &dev,
            &bus,
            &command(OP_WRITE, 0x02, 0x0248, &public[56..60])
        ),
        STATUS_OK
    );
    assert_eq!(
        status(
            &dev,
            &bus,
            &command(OP_WRITE, 0x02, 0x0249, &public[60..64])
        ),
        STATUS_OK
    );
    // And a 32-byte write that would run off the end of the slot is refused.
    assert_eq!(
        status(&dev, &bus, &command(OP_WRITE, 0x82, 0x0248, &[0u8; 32])),
        STATUS_PARSE
    );
    assert_eq!(
        status(&dev, &bus, &command(OP_VERIFY, 0x00, 0x0009, &sig)),
        STATUS_OK
    );
}

#[test]
fn sign_verifies_against_an_independent_witness_and_the_rfc_6979_vector() {
    // The point of this test: the device's own `Verify` is not evidence. A
    // consistently wrong curve would sign and verify happily with itself.
    //
    // So the signature is checked twice against something that is not this
    // model — once with `purecrypto`'s verifier directly, and once against the
    // published vector of **RFC 6979 A.2.5** (P-256, SHA-256, the message
    // "sample"), which also pins the *deterministic nonce*: the same key and
    // message must produce those exact bytes, not merely a valid signature.
    const X: [u8; 32] = [
        0xc9, 0xaf, 0xa9, 0xd8, 0x45, 0xba, 0x75, 0x16, 0x6b, 0x5c, 0x21, 0x57, 0x67, 0xb1, 0xd6,
        0x93, 0x4e, 0x50, 0xc3, 0xdb, 0x36, 0xe8, 0x9b, 0x12, 0x7b, 0x8a, 0x62, 0x2b, 0x12, 0x0f,
        0x67, 0x21,
    ];
    const R: [u8; 32] = [
        0xef, 0xd4, 0x8b, 0x2a, 0xac, 0xb6, 0xa8, 0xfd, 0x11, 0x40, 0xdd, 0x9c, 0xd4, 0x5e, 0x81,
        0xd6, 0x9d, 0x2c, 0x87, 0x7b, 0x56, 0xaa, 0xf9, 0x91, 0xc3, 0x4d, 0x0e, 0xa8, 0x4e, 0xaf,
        0x37, 0x16,
    ];
    const S: [u8; 32] = [
        0xf7, 0xcb, 0x1c, 0x94, 0x2d, 0x65, 0x7c, 0x41, 0xd4, 0x36, 0xc7, 0xa1, 0xb6, 0xe2, 0x9f,
        0x65, 0xf3, 0xe9, 0x00, 0xdb, 0xb9, 0xaf, 0xf4, 0x06, 0x4d, 0xc4, 0xab, 0x2f, 0x84, 0x3a,
        0xcd, 0xa8,
    ];

    let (dev, bus) = half_locked();
    wake(&bus);
    let _ = read_packet(&bus);

    // §9.2, `PrivWrite`: the host chooses the key, which is what makes a known
    // answer possible at all. Four leading zeros, then the 32-byte key.
    let mut payload = vec![0u8; 4];
    payload.extend_from_slice(&X);
    assert_eq!(
        status(&dev, &bus, &command(OP_PRIVWRITE, 0x00, 0x0001, &payload)),
        STATUS_OK
    );

    let digest = Sha256::digest(b"sample");
    assert_eq!(
        status(
            &dev,
            &bus,
            &command(OP_NONCE, 0x03, 0x0000, digest.as_ref())
        ),
        STATUS_OK
    );
    let sig = call(&dev, &bus, &command(OP_SIGN, 0x80, 0x0001, &[]));

    // The witness, one: RFC 6979 A.2.5's own r and s.
    assert_eq!(&sig[0..32], &R, "RFC 6979 A.2.5 r");
    assert_eq!(&sig[32..64], &S, "RFC 6979 A.2.5 s");

    // The witness, two: an implementation that is not this device's command
    // path, given the public key the device derived.
    let public = call(&dev, &bus, &command(OP_GENKEY, 0x00, 0x0001, &[]));
    let mut sec1 = [0u8; 65];
    sec1[0] = 0x04;
    sec1[1..65].copy_from_slice(&public);
    let key = EcdsaPublicKey::from_sec1(&sec1).expect("a point on P-256");
    let mut raw = [0u8; 64];
    raw.copy_from_slice(&sig);
    let signature = Signature::from_bytes(&raw);
    key.verify_prehash(digest.as_ref(), &signature)
        .expect("the signature verifies outside the device");
    // And does not verify over a different digest.
    let other = Sha256::digest(b"not sample");
    assert!(key.verify_prehash(other.as_ref(), &signature).is_err());
}

#[test]
fn ecdh_agrees_with_the_other_side_of_the_exchange() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    let device_public = call(&dev, &bus, &command(OP_GENKEY, 0x04, 0x0002, &[]));

    // The host's own key pair, from a fixed private key so the test is a known
    // answer rather than a coincidence.
    let host_private = [0x2bu8; 32];
    let host = EcdhPrivateKey::from_bytes(&host_private).expect("a legal scalar");
    let host_public = host.public_key().to_sec1();

    let shared = call(
        &dev,
        &bus,
        &command(OP_ECDH, 0x00, 0x0002, &host_public[1..65]),
    );
    // The other side of the exchange, computed here: X of d_host · Q_device.
    let mut sec1 = [0u8; 65];
    sec1[0] = 0x04;
    sec1[1..65].copy_from_slice(&device_public);
    let peer = EcdsaPublicKey::from_sec1(&sec1).expect("a point on P-256");
    assert_eq!(shared, host.diffie_hellman(&peer).unwrap().to_vec());
}

#[test]
fn a_key_command_against_a_slot_that_is_not_a_key_slot_is_refused() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    // Slot 9 is a public-key slot: `KeyConfig.Private` is clear, so there is no
    // private key in it to generate, sign with or exchange.
    for packet in [
        command(OP_GENKEY, 0x04, 0x0009, &[]),
        command(OP_SIGN, 0x80, 0x0009, &[]),
    ] {
        assert_eq!(status(&dev, &bus, &packet), STATUS_EXEC);
    }
}

#[test]
fn the_608_has_an_aes_engine_and_the_508a_does_not() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    // §9.2, `AES`: one block, under the key in a slot. The expected answer is
    // FIPS 197's own primitive, called here rather than by the device.
    let plain = [0x44u8; 16];
    let out = call(&dev, &bus, &command(OP_AES, 0x00, 0x0000, &plain));
    let mut key = [0u8; 16];
    key.copy_from_slice(&KEY0[0..16]);
    let mut expect = plain;
    Aes128::new(&key).encrypt_block(&mut expect);
    assert_eq!(out, expect.to_vec());
    // And back again.
    let round = call(&dev, &bus, &command(OP_AES, 0x01, 0x0000, &out));
    assert_eq!(round, plain.to_vec());

    let (dev, bus) = build(&[
        ("part", Value::Str("atecc508a".into())),
        ("config", Value::Media(Media::new("config", config_image()))),
        ("data", Value::Media(Media::new("data", data_image()))),
        ("lock", Value::Str("data".into())),
        ("watchdog-ticks", Value::Uint(0)),
    ]);
    wake(&bus);
    let _ = read_packet(&bus);
    assert_eq!(
        status(&dev, &bus, &command(OP_AES, 0x00, 0x0000, &plain)),
        STATUS_PARSE
    );
}

// ---------------------------------------------------------------------------
// Encrypted write
// ---------------------------------------------------------------------------

#[test]
fn an_encrypted_write_is_xored_with_the_session_key_and_carries_a_mac() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);

    // The session key: a pass-through nonce folded with slot 0's key, which is
    // what a host does before an encrypted write (§9.2, `Write`).
    let nonce = [0x77u8; 32];
    assert_eq!(
        status(&dev, &bus, &command(OP_NONCE, 0x03, 0x0000, &nonce)),
        STATUS_OK
    );
    assert_eq!(
        status(&dev, &bus, &command(OP_GENDIG, 0x02, 0x0000, &[])),
        STATUS_OK
    );
    let session = dev.temp_key_for_test().expect("GenDig left a session key");

    let plain = [0x5cu8; 32];
    let mode = 0xc2u8; // 32 bytes | encrypted | data zone
    let param2 = 0x0048u16; // slot 9, block 0
    let mut data: Vec<u8> = plain
        .iter()
        .zip(session.iter())
        .map(|(p, k)| p ^ k)
        .collect();
    let serial = dev.serial();
    let mut msg = Vec::new();
    msg.extend_from_slice(&session);
    msg.push(OP_WRITE);
    msg.push(mode);
    msg.extend_from_slice(&param2.to_le_bytes());
    msg.push(serial[8]);
    msg.extend_from_slice(&serial[0..2]);
    msg.extend_from_slice(&[0u8; 25]);
    msg.extend_from_slice(&plain);
    let mac = Sha256::digest(&msg);
    data.extend_from_slice(mac.as_ref());

    assert_eq!(
        status(&dev, &bus, &command(OP_WRITE, mode, param2, &data)),
        STATUS_OK
    );
    assert_eq!(&dev.slot(9).unwrap()[0..32], &plain);

    // A bit flipped in the ciphertext no longer matches the MAC, which is the
    // whole point of sending one.
    data[0] ^= 0x01;
    assert_eq!(
        status(&dev, &bus, &command(OP_WRITE, mode, param2, &data)),
        STATUS_MISCOMPARE
    );
}

// ---------------------------------------------------------------------------
// Power and the watchdog
// ---------------------------------------------------------------------------

#[test]
fn the_device_sleeps_after_the_watchdog_and_needs_a_new_wake() {
    // The default watchdog, this time: §6.3's tWATCHDOG, after which the part
    // puts *itself* to sleep with nobody talking to it. Firmware that does not
    // know this desyncs, which is why the model has it.
    let (dev, bus) = build(&[]);
    wake(&bus);
    assert!(dev.awake());
    let _ = read_packet(&bus);

    dev.advance_to(DEFAULT_WATCHDOG_TICKS - 1);
    assert!(dev.awake(), "still inside tWATCHDOG");
    dev.advance_to(DEFAULT_WATCHDOG_TICKS);
    assert!(!dev.awake(), "§6.3: the part sleeps by itself");
    assert_eq!(
        bus.start(Address::Seven(DEFAULT_ADDRESS), Direction::Write),
        Ack::Nack
    );
    bus.stop();

    // And a second wake brings it back, with the token again.
    wake(&bus);
    assert!(dev.awake());
    assert_eq!(read_packet(&bus), WAKE_TOKEN.to_vec());
}

#[test]
fn a_command_that_would_outlast_the_watchdog_is_refused_with_0xee() {
    let (dev, bus) = build(&[]);
    wake(&bus);
    let _ = read_packet(&bus);
    // §6.3: rather than be cut off mid-command, the part refuses one it cannot
    // finish. `GenKey` takes 115 ms, so park the clock 10 ms from the end.
    dev.advance_to(DEFAULT_WATCHDOG_TICKS - 10_000);
    write_word(&bus, WORD_COMMAND, &command(OP_GENKEY, 0x04, 0x0001, &[]));
    // Read it back inside the time that is left, which is the whole point of
    // being told rather than being cut off.
    dev.advance_to(dev.ticks() + 2_000);
    assert_eq!(body(&read_packet(&bus)), vec![STATUS_WATCHDOG]);
}

#[test]
fn exec_scale_stretches_the_datasheet_times_for_a_board_on_another_clock() {
    // §9.4's execution times are milliseconds, and this model spells them in
    // ticks of the board's own domain — the 1 MHz the defaults assume. A board
    // that clocks the part from 8 MHz says so once, here.
    let (dev, bus) = build(&[
        ("exec-scale", Value::Uint(8)),
        ("watchdog-ticks", Value::Uint(0)),
    ]);
    wake(&bus);
    let _ = read_packet(&bus);
    write_word(&bus, WORD_COMMAND, &command(OP_INFO, 0x00, 0x0000, &[]));
    // `Info` is one millisecond, so eight thousand ticks of an 8 MHz domain.
    dev.advance_to(7_999);
    assert!(dev.busy(), "one tick short");
    dev.advance_to(8_000);
    assert!(!dev.busy());
    assert_eq!(&body(&read_packet(&bus))[0..4], &dev.part().revnum());
}

#[test]
fn sleep_clears_tempkey_and_idle_keeps_it() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    let nonce = [0x21u8; 32];
    assert_eq!(
        status(&dev, &bus, &command(OP_NONCE, 0x03, 0x0000, &nonce)),
        STATUS_OK
    );
    assert_eq!(dev.temp_key_for_test(), Some(nonce));

    // §6.2: idle keeps the volatile registers...
    write_word(&bus, WORD_IDLE, &[]);
    assert!(!dev.awake());
    assert_eq!(dev.temp_key_for_test(), Some(nonce));
    wake(&bus);
    let _ = read_packet(&bus);
    assert_eq!(dev.temp_key_for_test(), Some(nonce));

    // ...and sleep does not.
    write_word(&bus, WORD_SLEEP, &[]);
    assert!(!dev.awake());
    assert_eq!(dev.temp_key_for_test(), None);
}

#[test]
fn the_part_nacks_while_a_command_runs_which_is_how_a_driver_polls() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    write_word(&bus, WORD_COMMAND, &command(OP_SIGN, 0x80, 0x0001, &[]));
    assert!(dev.busy());
    // §9.4: while the command runs the part answers nothing at all — the poll
    // loop a driver writes around exactly this.
    assert_eq!(
        bus.start(Address::Seven(DEFAULT_ADDRESS), Direction::Read),
        Ack::Nack
    );
    bus.stop();
    dev.advance_to(dev.ticks() + 60_000);
    assert!(!dev.busy(), "60 ms is the datasheet's maximum for Sign");
    assert_eq!(
        bus.start(Address::Seven(DEFAULT_ADDRESS), Direction::Read),
        Ack::Ack
    );
    bus.stop();
}

// ---------------------------------------------------------------------------
// Determinism
// ---------------------------------------------------------------------------

#[test]
fn two_runs_with_the_same_seed_produce_the_same_random_and_the_same_key() {
    // The claim the whole seed design exists for, and the one a state hash
    // depends on: the same machine file, run twice, deals the same numbers and
    // generates the same key pair.
    let run = |seed: u64, path: &str| {
        let mut p = Props::new();
        p.insert("config", Value::Media(Media::new("config", config_image())));
        p.insert("data", Value::Media(Media::new("data", data_image())));
        p.insert("lock", Value::Str("data".into()));
        p.insert("watchdog-ticks", Value::Uint(0));
        p.insert("seed", Value::Uint(seed));
        let dev = Atecc::new(&p).expect("it builds");
        dev.seed_from_path(path);
        let bus = Arc::new(I2cBus::new());
        bus.attach(dev.slave()).unwrap();
        wake(&bus);
        let _ = read_packet(&bus);
        let random = call(&dev, &bus, &command(OP_RANDOM, 0x00, 0x0000, &[]));
        let public = call(&dev, &bus, &command(OP_GENKEY, 0x04, 0x0001, &[]));
        (random, public, dev.serial())
    };

    let first = run(1, "board/atecc");
    let second = run(1, "board/atecc");
    assert_eq!(first, second, "same seed, same path, same everything");

    // A different instance of the same class on the same board is a different
    // stream, because the path goes into the mix (`core::rand::derive_seed`).
    let sibling = run(1, "board/atecc2");
    assert_ne!(first.0, sibling.0, "the random bytes differ");
    assert_ne!(first.1, sibling.1, "and so does the generated key");
    assert_ne!(first.2, sibling.2, "and the serial number");

    // As does a different board seed.
    let reseeded = run(7, "board/atecc");
    assert_ne!(first.0, reseeded.0);
    assert_ne!(first.1, reseeded.1);
}

#[test]
fn realize_is_what_mixes_the_path_in() {
    // The same claim through the real path: `Device::realize` is where the
    // instance name reaches the seed, because `new(props)` has no path.
    let mut p = Props::new();
    p.insert("seed", Value::Uint(1));
    let dev = Atecc::new(&p).expect("it builds");
    let before = dev.stream_seed();
    let hosts = HostObjects::new();
    let mut deferred = Deferred::new();
    let mut ctx = RealizeCtx::new("board/atecc", RequesterId(1), &mut deferred, &hosts);
    Device::realize(&dev, &mut ctx).expect("an ATECC realizes");
    assert_eq!(dev.board_seed(), 1);
    assert_ne!(dev.stream_seed(), before);
    assert_eq!(
        dev.stream_seed(),
        crate::core::rand::derive_seed(1, "board/atecc")
    );
}

#[test]
fn a_reset_rewinds_the_stream_so_a_rerun_deals_the_same_numbers() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    let first = call(&dev, &bus, &command(OP_RANDOM, 0x00, 0x0000, &[]));
    let second = call(&dev, &bus, &command(OP_RANDOM, 0x00, 0x0000, &[]));
    assert_ne!(first, second);

    dev.reset(ResetKind::Cold);
    // A reset is a power cycle: the part comes up asleep (§6).
    assert!(!dev.awake());
    wake(&bus);
    let _ = read_packet(&bus);
    assert_eq!(
        call(&dev, &bus, &command(OP_RANDOM, 0x00, 0x0000, &[])),
        first,
        "the stream went back to its seed"
    );
}

// ---------------------------------------------------------------------------
// Snapshots
// ---------------------------------------------------------------------------

#[test]
fn a_part_mid_session_round_trips_to_an_identical_chunk() {
    let (dev, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    // Something in every corner of the state: a stream position, a TempKey, a
    // generated key, a counter and a half-read response.
    let _ = call(&dev, &bus, &command(OP_RANDOM, 0x00, 0x0000, &[]));
    let _ = call(&dev, &bus, &command(OP_GENKEY, 0x04, 0x0001, &[]));
    let _ = call(&dev, &bus, &command(OP_COUNTER, 0x01, 0x0000, &[]));
    assert_eq!(
        status(&dev, &bus, &command(OP_NONCE, 0x03, 0x0000, &[0x9u8; 32])),
        STATUS_OK
    );
    write_word(&bus, WORD_COMMAND, &command(OP_INFO, 0x00, 0x0000, &[]));
    dev.advance_to(dev.ticks() + 1_000_000);
    assert_eq!(
        bus.start(Address::Seven(DEFAULT_ADDRESS), Direction::Read),
        Ack::Ack
    );
    assert_eq!(bus.read(Ack::Ack), 7, "the count byte of an Info response");
    bus.stop();

    let bytes = save(&dev);
    let (other, bus2) = provisioned();
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load(
            "atecc",
            ATECC_CLASS.name,
            ATECC_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    other.load(&mut chunk.reader()).unwrap();

    // The identical-state-hash property, checked where it is observable: the
    // two parts' saved chunks are byte for byte the same.
    assert_eq!(save(&other), bytes);
    // And the restored part finishes the response the first one was part way
    // through reading, rather than starting it again.
    let rest = {
        assert_eq!(
            bus2.start(Address::Seven(DEFAULT_ADDRESS), Direction::Read),
            Ack::Ack
        );
        let mut out = Vec::new();
        for i in 0..6 {
            out.push(bus2.read(if i == 5 { Ack::Nack } else { Ack::Ack }));
        }
        bus2.stop();
        out
    };
    assert_eq!(&rest[0..4], &other.part().revnum());
}

/// One device's chunk, on its own.
fn save(dev: &Atecc) -> Vec<u8> {
    let mut shape = MachineShape::new();
    shape.add_device("atecc", ATECC_CLASS.name).unwrap();
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("atecc", ATECC_CLASS.name, ATECC_CLASS.version)
            .unwrap();
        dev.save(&mut chunk).unwrap();
    }
    w.to_vec().unwrap()
}

// ---------------------------------------------------------------------------
// The other link
// ---------------------------------------------------------------------------

#[test]
fn a_wired_master_carries_the_same_packets_a_transactional_one_does() {
    // The claim `docs/buses/low-speed.md` asks for, on a device whose answer is
    // a cryptographic packet rather than a byte of RAM. The single-wire twin of
    // it is `the_swi_transport_carries_the_same_packets_as_i2c`, below: three
    // transports now reach one packet layer.
    let (transactional, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    let by_call = call(&transactional, &bus, &command(OP_INFO, 0x00, 0x0000, &[]));

    let (wired, _) = provisioned();
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

    // Drive one bus event to completion and say how it ended.
    let step = |op: MasterOp| -> MasterEvent {
        assert!(master.submit(op));
        for _ in 0..64 {
            match master.tick() {
                MasterEvent::Working | MasterEvent::Stretched => {}
                other => return other,
            }
        }
        panic!("{op:?} never finished");
    };
    let run = |ops: &[MasterOp]| {
        for op in ops {
            step(*op);
        }
    };
    // The wake pulse, bit by bit.
    run(&[
        MasterOp::Start,
        MasterOp::Write(WAKE_ADDRESS << 1),
        MasterOp::Stop,
    ]);
    // The command.
    let mut ops = vec![
        MasterOp::Start,
        MasterOp::Write(DEFAULT_ADDRESS << 1),
        MasterOp::Write(WORD_COMMAND),
    ];
    for byte in command(OP_INFO, 0x00, 0x0000, &[]) {
        ops.push(MasterOp::Write(byte));
    }
    ops.push(MasterOp::Stop);
    run(&ops);
    wired.advance_to(wired.ticks() + 1_000_000);

    // And the answer, read back the same way.
    run(&[MasterOp::Start, MasterOp::Write((DEFAULT_ADDRESS << 1) | 1)]);
    let mut packet = Vec::new();
    for i in 0..7 {
        let last = i == 6;
        match step(MasterOp::Read(if last { Ack::Nack } else { Ack::Ack })) {
            MasterEvent::Read(byte) => packet.push(byte),
            other => panic!("a read ended as {other:?}"),
        }
    }
    run(&[MasterOp::Stop]);
    assert_eq!(body(&packet), by_call);
}

// ---------------------------------------------------------------------------
// The single wire (DS40002249B §8)
// ---------------------------------------------------------------------------

/// Put this part on a single wire nothing else can reach.
///
/// The *same object* the I²C fixtures put on a bus, behind the other trait, so
/// a test can drive one device both ways.
fn wire(dev: &Atecc) -> Arc<SwiLink> {
    let link = Arc::new(SwiLink::new());
    link.attach(dev.swi()).expect("an empty wire");
    link
}

/// The wake pulse on one wire.
///
/// §7.1.1: "a data byte of 0x00 [transmitted] at a clock rate sufficiently slow
/// so that SDA is low for a minimum period of tWLO". At 7N1 a `0x00` is low for
/// eight bit times, so half the token rate makes 69 µs of it — over the 60 µs
/// of [`WAKE_LOW_NS`] — and the host puts its rate back afterwards.
fn swi_wake(link: &SwiLink) {
    link.set_baud(BAUD / 2).expect("a real rate");
    link.send(Token::WAKE);
    link.set_baud(BAUD).expect("a real rate");
}

/// Take the wake token the part answers a freshly woken transmit flag with.
fn swi_wake_token(link: &SwiLink) {
    link.flag(Flag::TRANSMIT);
    assert_eq!(
        link.read_group(),
        Some(WAKE_TOKEN.to_vec()),
        "§8.3.2 step 7: the part answers a 0x11 status after a wake"
    );
}

/// Send a command group, let it execute, and read the answer back.
///
/// The single-wire twin of [`call`], and deliberately the same shape: a flag,
/// the group, tEXEC, a transmit flag, the group back.
fn swi_call(dev: &Atecc, link: &SwiLink, packet: &[u8]) -> Vec<u8> {
    link.flag(Flag::COMMAND);
    link.write_group(packet);
    assert!(dev.busy(), "§9.4: a command takes time");
    dev.advance_to(dev.ticks() + 1_000_000);
    link.flag(Flag::TRANSMIT);
    body(&link.read_group().expect("the part answered"))
}

#[test]
fn the_swi_transport_carries_the_same_packets_as_i2c() {
    // The claim the whole front end exists to make. Two halves:
    //
    // 1. one device, two faces, one command — the packet layer is reached
    //    through either transport and answers the identical bytes;
    // 2. two identically seeded devices driven through a *sequence* that moves
    //    their state — `Random` and `GenKey` draw from the deterministic
    //    stream, `Nonce` and `MAC` carry `TempKey` between commands — one over
    //    two wires and one over one, with the transcripts compared. A front end
    //    that dropped or reordered a byte would pass (1) and fail this.
    let (dev, bus) = provisioned();
    let link = wire(&dev);

    wake(&bus);
    assert_eq!(read_packet(&bus), WAKE_TOKEN.to_vec());
    let over_i2c = call(&dev, &bus, &command(OP_INFO, 0x00, 0x0000, &[]));
    swi_wake(&link);
    swi_wake_token(&link);
    let over_swi = swi_call(&dev, &link, &command(OP_INFO, 0x00, 0x0000, &[]));
    assert_eq!(
        over_swi, over_i2c,
        "one device, one packet layer, two transports"
    );

    let script = [
        command(OP_RANDOM, 0x00, 0x0000, &[]),
        command(OP_GENKEY, 0x04, 0x0001, &[]),
        command(OP_NONCE, 0x03, 0x0000, &[0x9u8; 32]),
        command(OP_MAC, 0x01, 0x0000, &[]),
        command(OP_COUNTER, 0x01, 0x0000, &[]),
        command(OP_READ, 0x00, 0x0000, &[]),
    ];

    let (two_wire, bus) = provisioned();
    wake(&bus);
    let _ = read_packet(&bus);
    let by_i2c: Vec<Vec<u8>> = script.iter().map(|p| call(&two_wire, &bus, p)).collect();

    let (one_wire, _) = provisioned();
    let link = wire(&one_wire);
    swi_wake(&link);
    swi_wake_token(&link);
    let by_swi: Vec<Vec<u8>> = script
        .iter()
        .map(|p| swi_call(&one_wire, &link, p))
        .collect();

    assert_eq!(by_swi, by_i2c);
    // Not vacuously: the stream really did move, so these are not six copies
    // of one answer.
    assert_ne!(by_i2c[0], by_i2c[1]);
    assert_eq!(by_i2c[0].len(), 32, "`Random` is 32 bytes (§9.2)");
}

#[test]
fn a_wake_needs_a_low_pulse_longer_than_a_token_can_make() {
    let (dev, _bus) = blank();
    let link = wire(&dev);

    // §8.1: a part asleep "ignores all data tokens until [it receives] a legal
    // Wake token", and a token is not one however it is spelled.
    link.flag(Flag::TRANSMIT);
    assert_eq!(link.recv(), None);
    assert!(!dev.awake());

    // Nor is a 0x00 at the token rate: 34.7 µs of low against a 60 µs tWLO.
    assert!(Token::WAKE.low_ns(BAUD) < WAKE_LOW_NS);
    link.send(Token::WAKE);
    assert!(!dev.awake(), "§9.3.1: that pulse is too short to be a wake");

    // Slow the UART down and the same byte is one.
    assert!(Token::WAKE.low_ns(BAUD / 2) >= WAKE_LOW_NS);
    swi_wake(&link);
    assert!(dev.awake());
    swi_wake_token(&link);
    // §9.1.3's own worked example, arriving down the other transport.
    assert_eq!(body(&WAKE_TOKEN), vec![STATUS_AFTER_WAKE]);
}

#[test]
fn a_group_rides_the_wire_as_0x7f_and_0x7d_tokens_in_both_directions() {
    // The encoding itself, spelled out by hand rather than through the link's
    // helpers, so this test would fail if `bus::swi` and this front end agreed
    // with each other and disagreed with DS40002025A Table 5-1.
    let (dev, _bus) = provisioned();
    let link = wire(&dev);
    swi_wake(&link);
    swi_wake_token(&link);

    let packet = command(OP_INFO, 0x00, 0x0000, &[]);
    for token in tokens_of(Flag::COMMAND.0) {
        link.send(token);
    }
    for byte in &packet {
        // Least significant bit first (DS40002025A §5).
        for i in 0..8 {
            link.send(if (byte >> i) & 1 != 0 {
                Token::ONE
            } else {
                Token::ZERO
            });
        }
    }
    assert!(dev.busy(), "the count byte ended the group, not a STOP");
    dev.advance_to(dev.ticks() + 1_000_000);

    for token in tokens_of(Flag::TRANSMIT.0) {
        link.send(token);
    }
    let mut tokens = Vec::new();
    while let Some(token) = link.recv() {
        tokens.push(token);
    }
    assert!(
        tokens.iter().all(|t| *t == Token::ONE || *t == Token::ZERO),
        "the part drives nothing but 0x7f and 0x7d"
    );
    assert_eq!(tokens.len(), 7 * 8, "an `Info` response is seven bytes");
    let mut bytes = Vec::new();
    for chunk in tokens.chunks(8) {
        let mut byte = 0u8;
        for (i, token) in chunk.iter().enumerate() {
            byte |= u8::from(*token == Token::ONE) << i;
        }
        bytes.push(byte);
    }
    assert_eq!(&body(&bytes)[..4], &dev.part().revnum());
}

#[test]
fn each_of_the_four_flags_does_what_table_8_1_says() {
    let (dev, _bus) = provisioned();
    let link = wire(&dev);
    swi_wake(&link);

    // 0x88 Transmit: "wait for a bus turnaround time and then start
    // transmitting its response".
    swi_wake_token(&link);
    // And again — DS40002025A §5.2: "When valid data is in the output buffer,
    // the transmit flag may be repeatedly issued to the device to resend the
    // buffer to the system."
    swi_wake_token(&link);

    // 0x77 Command: "the system starts sending a command group to the device".
    let info = swi_call(&dev, &link, &command(OP_INFO, 0x00, 0x0000, &[]));
    assert_eq!(&info[..4], &dev.part().revnum());

    // Something in `TempKey` to tell idle from sleep by.
    assert_eq!(
        swi_call(&dev, &link, &command(OP_NONCE, 0x03, 0x0000, &[0x9u8; 32])),
        vec![STATUS_OK]
    );
    let temp_key = dev.temp_key_for_test().expect("`Nonce` loaded it");

    // 0xBB Idle: "the device goes into the idle mode ... It does not invalidate
    // the contents of the TempKey". The I/O buffer does go.
    link.flag(Flag::IDLE);
    assert!(!dev.awake());
    link.flag(Flag::TRANSMIT);
    assert_eq!(link.recv(), None, "idle answers nothing at all");
    swi_wake(&link);
    assert_eq!(
        dev.temp_key_for_test(),
        Some(temp_key),
        "§8.2: an idle flag keeps TempKey"
    );

    // 0xCC Sleep: "the low-power sleep mode, which causes a complete reset of
    // the device, including invalidation of the contents of the SRAM and all
    // volatile registers".
    link.flag(Flag::SLEEP);
    assert!(!dev.awake());
    swi_wake(&link);
    assert_eq!(
        dev.temp_key_for_test(),
        None,
        "§8.2: a sleep flag takes TempKey with it"
    );
    swi_wake_token(&link);

    // "All other values are reserved and must not be used." One that arrives
    // anyway leaves the part where it was rather than inventing a meaning.
    link.flag(Flag(0x5a));
    assert!(dev.awake());
    swi_wake_token(&link);
}

#[test]
fn a_malformed_token_stream_sleeps_the_part_after_the_io_timeout() {
    // §8.3.1: "Failure to send enough bits, or the transmission of an illegal
    // token ... will cause the device to enter the Sleep mode after the
    // tTIMEOUT-SWI interval." Note *after*: the part does not fail on the spot,
    // which is what lets a host recover by waiting.
    let (dev, _bus) = blank();
    let link = wire(&dev);
    swi_wake(&link);
    swi_wake_token(&link);

    link.send(Token(0x55));
    assert!(dev.awake(), "an illegal token does not fail on the spot");
    dev.advance_to(DEFAULT_SWI_TIMEOUT_TICKS - 1);
    assert!(dev.awake(), "still inside tTIMEOUT-SWI");
    dev.advance_to(DEFAULT_SWI_TIMEOUT_TICKS);
    assert!(!dev.awake(), "§8.3.1: the part sleeps by itself");

    // A group that stops half way through is the same failure: the count byte
    // promised seven bytes and one arrived.
    swi_wake(&link);
    let at = dev.ticks();
    link.flag(Flag::COMMAND);
    link.write_group(&[0x07]);
    dev.advance_to(at + DEFAULT_SWI_TIMEOUT_TICKS - 1);
    assert!(dev.awake());
    dev.advance_to(at + DEFAULT_SWI_TIMEOUT_TICKS);
    assert!(
        !dev.awake(),
        "§8.3.1: nor does it wait forever for the rest"
    );

    // And so is a byte that stops half way through.
    swi_wake(&link);
    let at = dev.ticks();
    link.send(Token::ONE);
    link.send(Token::ZERO);
    dev.advance_to(at + DEFAULT_SWI_TIMEOUT_TICKS);
    assert!(!dev.awake());

    // Whereas a part that is simply left alone between transactions keeps its
    // watchdog and nothing else: the timeout counter is not running.
    swi_wake(&link);
    swi_wake_token(&link);
    dev.advance_to(dev.ticks() + DEFAULT_SWI_TIMEOUT_TICKS * 4);
    assert!(dev.awake(), "nothing was half sent, so nothing timed out");
}

#[test]
fn the_watchdog_takes_the_part_on_the_single_wire_too() {
    // §6.3 is a property of the part, not of a transport: the same tWATCHDOG
    // that ends an I²C session ends this one, with nobody talking to the part.
    let (dev, _bus) = build(&[]);
    let link = wire(&dev);
    swi_wake(&link);
    swi_wake_token(&link);

    dev.advance_to(DEFAULT_WATCHDOG_TICKS - 1);
    assert!(dev.awake(), "still inside tWATCHDOG");
    dev.advance_to(DEFAULT_WATCHDOG_TICKS);
    assert!(!dev.awake(), "§6.3: the part sleeps by itself");
    link.flag(Flag::TRANSMIT);
    assert_eq!(link.recv(), None, "and answers nothing until it is woken");

    swi_wake(&link);
    assert!(dev.awake());
    swi_wake_token(&link);

    // And the other half of §6.3: a command that could not finish inside what
    // is left of the watchdog is refused rather than cut off. `GenKey` takes
    // 115 ms, and there are 10 left.
    dev.advance_to(dev.ticks() + DEFAULT_WATCHDOG_TICKS - 10_000);
    link.flag(Flag::COMMAND);
    link.write_group(&command(OP_GENKEY, 0x04, 0x0001, &[]));
    dev.advance_to(dev.ticks() + 2_000);
    link.flag(Flag::TRANSMIT);
    assert_eq!(
        body(&link.read_group().expect("the part answered")),
        vec![STATUS_WATCHDOG]
    );
}

#[test]
fn a_busy_part_ignores_the_single_wire_rather_than_nacking_it() {
    // §8.2: "When the device is busy executing a command, it ignores the SDA
    // pin and any flags that are sent by the system." There is no acknowledge
    // on this wire to say so with, so what a host sees is silence — and §8.3.2
    // is the procedure that follows from it: wait tEXEC, send the flag again.
    let (dev, _bus) = provisioned();
    let link = wire(&dev);
    swi_wake(&link);
    swi_wake_token(&link);

    link.flag(Flag::COMMAND);
    link.write_group(&command(OP_INFO, 0x00, 0x0000, &[]));
    assert!(dev.busy());
    link.flag(Flag::TRANSMIT);
    assert_eq!(link.recv(), None, "silence, not a NACK");
    assert!(!link.pending(), "and a monitor's look agrees");

    dev.advance_to(dev.ticks() + 1_000_000);
    // The swallowed flag has to be sent again: the part was not sampling the
    // pin, so it did not half-receive it either.
    link.flag(Flag::TRANSMIT);
    assert!(link.pending(), "a debug peek finds the answer");
    assert_eq!(
        &body(&link.read_group().expect("the part answered"))[..4],
        &dev.part().revnum()
    );
}

#[test]
fn the_interface_property_is_what_i2c_enable_reports() {
    // §2.2.4: configuration byte 14 is `I2C_Enable`, and bit 0 is what tells
    // firmware which part it is holding. Both faces answer either way — see
    // the module docs — so this byte is the whole of the difference.
    let (i2c, _) = build(&[("watchdog-ticks", Value::Uint(0))]);
    assert_eq!(i2c.interface(), Interface::I2c);
    assert_eq!(i2c.config()[14], 0x01);

    let (swi, _) = build(&[
        ("interface", Value::Str("swi".into())),
        ("watchdog-ticks", Value::Uint(0)),
    ]);
    assert_eq!(swi.interface(), Interface::Swi);
    assert_eq!(swi.config()[14], 0x00);

    let mut p = Props::new();
    p.insert("interface", Value::Str("spi".into()));
    Atecc::new(&p).expect_err("a part is ordered as `i2c` or `swi`");
}

#[test]
fn a_part_mid_group_on_the_single_wire_round_trips_to_an_identical_chunk() {
    // The framing of a half-received group is live state exactly as the I²C
    // phase is, so a snapshot has to carry it: this one is taken between two
    // tokens of one byte of one group.
    let (dev, _bus) = provisioned();
    let link = wire(&dev);
    swi_wake(&link);
    swi_wake_token(&link);

    let packet = command(OP_INFO, 0x00, 0x0000, &[]);
    link.flag(Flag::COMMAND);
    link.write_group(&packet[..3]);
    // And one token of the fourth byte, so the snapshot lands mid-byte.
    link.send(Token::of(packet[3] & 1 != 0));

    let bytes = save(&dev);
    let (other, _bus) = provisioned();
    let link = wire(&other);
    let reader = StateReader::new(&bytes).unwrap();
    let chunk = reader
        .load(
            "atecc",
            ATECC_CLASS.name,
            ATECC_CLASS.version,
            &Migrations::new(),
        )
        .unwrap();
    other.load(&mut chunk.reader()).unwrap();
    assert_eq!(save(&other), bytes, "byte for byte, framing included");

    // The restored part finishes the byte it was part way through rather than
    // starting it again, and then the group.
    for i in 1..8 {
        link.send(Token::of((packet[3] >> i) & 1 != 0));
    }
    link.write_group(&packet[4..]);
    assert!(other.busy(), "the group completed on its count byte");
    other.advance_to(other.ticks() + 1_000_000);
    link.flag(Flag::TRANSMIT);
    assert_eq!(
        &body(&link.read_group().expect("the part answered"))[..4],
        &other.part().revnum()
    );
}
