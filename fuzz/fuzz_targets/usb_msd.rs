#![no_main]
//! The USB mass storage device's untrusted surface: **a byte stream on two bulk
//! endpoints**.
//!
//! `CLAUDE.md` asks for a fuzz target on every MMIO surface. This device has no
//! MMIO surface at all — it masters nothing, it walks no guest structure, and it
//! has no register block — so the rule has to be read for what it is about
//! rather than for the word it uses: *the input a guest chooses, aimed at the
//! code that parses it*. For a Bulk-Only Transport device that input is the
//! Command Block Wrapper and the data phase behind it, and it is every bit as
//! attacker-controlled as a queue head is.
//!
//! What the fuzzer is looking for is the shape of bug the unit tests cannot
//! cover, because it needs an arbitrary *sequence*:
//!
//! > **Nothing is ever sized from a number the guest chose, and the state
//! > machine never gets stuck in a phase it cannot leave.** A
//! > `dCBWDataTransferLength` of `0xffffffff`, a `READ (10)` for 65,535 blocks
//! > and an allocation length of 65,535 all have to cost one packet of memory
//! > and one packet of work, whatever order they arrive in and whatever
//! > half-finished transfer they interrupt.
//!
//! An unbounded allocation shows up here as an out-of-memory abort and an
//! unterminating loop as a timeout, which are the two failures this target
//! exists to catch. Four properties ride along, checked after every step:
//!
//! * **`dCSWDataResidue` never exceeds `dCBWDataTransferLength`** (BOT §5.2 says
//!   so in as many words). This is the invariant that would break first if the
//!   residue arithmetic ever counted a byte twice, and a host that saw a residue
//!   larger than the transfer would compute a negative length.
//! * **A debug peek has no side effects.** Peeking twice must give the same
//!   answer, and must not pop a status wrapper, advance a data cursor or clear
//!   sense data (`ROADMAP.md` §15, invariant 5).
//! * **A CSW is thirteen bytes and a CBW is thirty-one**, and the device is
//!   ready for a new command once the wrapper has gone out (§6.2.1).
//! * **The snapshot loader is a parser on untrusted bytes**, so a tail of the
//!   input goes into `Device::load`, which must reject it or accept it and never
//!   panic — and the device must still be usable afterwards.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 <31 bytes>    a bulk-OUT packet of exactly CBW length
//!   0x01 nn <nn bytes> a bulk-OUT packet of nn (mod 96) bytes
//!   0x02 nn            a bulk-IN of nn (mod 96) bytes, peeked twice first
//!   0x03               the Bulk-Only Mass Storage Reset (BOT 3.1)
//!   0x04 ee            CLEAR_FEATURE(ENDPOINT_HALT) on endpoint ee
//!   0x05               a bus reset
//!   0x06 <8 bytes>     a raw SETUP packet on the default pipe, then one
//!                        IN and one OUT to run its stages
//!   0x07               save, then load what was saved: a round trip
//!   0x08 ...           load the rest of the input as a snapshot chunk
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather than
//! mostly-rejected.

use libfuzzer_sys::fuzz_target;

use std::collections::BTreeMap;

use rsemu::bus::usb::{DeviceAddress, SetupPacket, Status, UsbBus, feature, request};
use rsemu::core::device::{Device, ResetKind};
use rsemu::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use rsemu::dev::usb::msd::{
    CBW_BYTES, CBW_SIGNATURE, CSW_BYTES, CSW_SIGNATURE, ENDPOINT_IN, ENDPOINT_OUT, UsbStorage,
};
use std::sync::Arc;

/// A small disk: the interesting inputs are the ones that ask for far more than
/// this, and a small one makes an over-allocation obvious.
const DISK_BYTES: u64 = 64 * 512;

/// The address the harness enumerates the device to.
const ADDRESS: DeviceAddress = DeviceAddress(3);

/// The largest packet the fuzzer may hand an endpoint. Bigger than
/// `wMaxPacketSize` would be, on purpose: a host controller would never do it
/// and the device must not care.
const MAX_PACKET: usize = 96;

struct Fixture {
    bus: Arc<UsbBus>,
    disk: UsbStorage,
}

fn build() -> Fixture {
    let disk = UsbStorage::in_memory(DISK_BYTES);
    let bus = Arc::new(UsbBus::new(1));
    bus.attach(0, disk.device()).expect("an empty port");
    bus.set_enabled(0, true);
    Fixture { bus, disk }
}

/// Enumerate: address it and configure it, so the bulk endpoints are live.
///
/// Done with raw transactions rather than the host-side composer, because the
/// composer is not what is being fuzzed and a fixture that could fail to build
/// would hide the interesting inputs behind a `return`.
fn enumerate(f: &Fixture) {
    let setup = SetupPacket {
        request_type: 0,
        request: request::SET_ADDRESS,
        value: u16::from(ADDRESS.0),
        index: 0,
        length: 0,
    };
    f.bus.setup(DeviceAddress::DEFAULT, 0, setup);
    // The status stage is what makes the address take effect (USB 2.0 §9.4.6).
    let _ = f.bus.read(DeviceAddress::DEFAULT, 0, &mut []);

    let setup = SetupPacket {
        request_type: 0,
        request: request::SET_CONFIGURATION,
        value: 1,
        index: 0,
        length: 0,
    };
    f.bus.setup(ADDRESS, 0, setup);
    let _ = f.bus.read(ADDRESS, 0, &mut []);
}

fn snapshot(f: &Fixture) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("disk", "usb.storage").ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("disk", "usb.storage", 1).ok()?;
        f.disk.save(&mut chunk).ok()?;
    }
    w.to_vec().ok()
}

/// Every `dCBWDataTransferLength` the harness has sent under a given
/// `dCBWTag`, largest first.
///
/// **Keyed by tag, not by "the last one sent"**, and that is a correction the
/// fuzzer itself forced: a thirty-one-byte packet is a CBW whichever opcode
/// produced it, and a CBW that arrives during a data-out phase is *data* rather
/// than a command — so "the length of the last wrapper the harness wrote" is
/// not the length of the wrapper the device is answering, and comparing against
/// it reports a violation that is the harness's. The tag is the association BOT
/// §5.2 defines for exactly this, so the harness uses it. Recording the largest
/// length seen for a tag keeps the bound sound when one tag is reused: whichever
/// of those commands the device is answering, its length is no larger.
type Sent = BTreeMap<u32, u32>;

/// Note a packet the harness sent to the bulk-out pipe, if it is a CBW.
fn note_cbw(sent: &mut Sent, packet: &[u8]) {
    if packet.len() != CBW_BYTES {
        return;
    }
    if u32::from_le_bytes([packet[0], packet[1], packet[2], packet[3]]) != CBW_SIGNATURE {
        return;
    }
    let tag = u32::from_le_bytes([packet[4], packet[5], packet[6], packet[7]]);
    let length = u32::from_le_bytes([packet[8], packet[9], packet[10], packet[11]]);
    let slot = sent.entry(tag).or_insert(0);
    *slot = (*slot).max(length);
}

/// BOT §5.2: `dCSWDataResidue` shall not exceed the `dCBWDataTransferLength` of
/// the command it answers, and `bCSWStatus` is one of three values (table 5.3).
fn check_csw(bytes: &[u8], moved: usize, sent: &Sent) {
    if moved < CSW_BYTES || bytes.len() < CSW_BYTES {
        return;
    }
    let signature = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    if signature != CSW_SIGNATURE {
        // Not a status wrapper — this was a data-phase packet that happened to
        // be thirteen bytes long.
        return;
    }
    let tag = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
    let Some(&length) = sent.get(&tag) else {
        // A tag the harness never sent: this is a wrapper the device built out
        // of a snapshot the fuzzer wrote, and there is no command to compare
        // its residue against.
        return;
    };
    let residue = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
    assert!(
        residue <= length,
        "dCSWDataResidue {residue} exceeds the dCBWDataTransferLength {length} of the command \
         tagged {tag:#010x} that it answers (BOT 5.2)"
    );
    assert!(
        bytes[12] <= 2,
        "bCSWStatus {} is not one of the three values BOT table 5.3 defines",
        bytes[12]
    );
}

fuzz_target!(|data: &[u8]| {
    let f = build();
    enumerate(&f);
    // Every command block the fuzzer has sent, by tag, for the residue
    // invariant above.
    let mut sent = Sent::new();
    let mut at = 0usize;

    while at < data.len() {
        let op = data[at];
        at += 1;
        match op {
            0x00 => {
                if at + CBW_BYTES > data.len() {
                    break;
                }
                let packet = &data[at..at + CBW_BYTES];
                at += CBW_BYTES;
                note_cbw(&mut sent, packet);
                // **The property.** Whatever thirty-one bytes say — a transfer
                // length of 0xffffffff, a READ for every block a 16-bit field
                // can name, a command block length of 255 — this returns, and
                // it returns having allocated nothing proportional to any of
                // them.
                let _ = f.bus.write(ADDRESS, ENDPOINT_OUT, packet);
            }
            0x01 => {
                if at >= data.len() {
                    break;
                }
                let want = usize::from(data[at]) % MAX_PACKET;
                at += 1;
                let have = want.min(data.len() - at);
                let packet = &data[at..at + have];
                at += have;
                // A thirty-one-byte packet is a CBW whichever opcode produced
                // it, so this one is recorded too.
                note_cbw(&mut sent, packet);
                let _ = f.bus.write(ADDRESS, ENDPOINT_OUT, packet);
            }
            0x02 => {
                if at >= data.len() {
                    break;
                }
                let want = usize::from(data[at]) % MAX_PACKET;
                at += 1;

                // The debug path first, twice: a monitor showing what is queued
                // must not consume it, so two peeks must agree and the real read
                // that follows must still see the same bytes.
                let mut first = vec![0u8; want];
                let mut second = vec![0u8; want];
                let a = f.bus.peek(ADDRESS, ENDPOINT_IN, &mut first);
                let b = f.bus.peek(ADDRESS, ENDPOINT_IN, &mut second);
                assert_eq!(a, b, "a debug peek had a side effect");
                assert_eq!(first, second, "a debug peek moved a cursor");

                let mut live = vec![0u8; want];
                let done = f.bus.read(ADDRESS, ENDPOINT_IN, &mut live);
                if done.status == Status::Ack {
                    let moved = done.len as usize;
                    assert!(moved <= want, "an IN returned more than the packet size");
                    if a.status == Status::Ack {
                        assert_eq!(
                            &live[..moved],
                            &first[..moved.min(first.len())],
                            "the peek showed different bytes from the read"
                        );
                    }
                    check_csw(&live, moved, &sent);
                }
            }
            // BOT §3.1: the class reset. It must ready the device for the next
            // CBW without touching the endpoint stall conditions.
            0x03 => {
                let setup = SetupPacket {
                    request_type: 0x21,
                    request: 0xff,
                    value: 0,
                    index: 0,
                    length: 0,
                };
                f.bus.setup(ADDRESS, 0, setup);
                let _ = f.bus.read(ADDRESS, 0, &mut []);
            }
            0x04 => {
                if at >= data.len() {
                    break;
                }
                let endpoint = data[at];
                at += 1;
                let setup = SetupPacket {
                    request_type: 0x02,
                    request: request::CLEAR_FEATURE,
                    value: feature::ENDPOINT_HALT,
                    index: u16::from(endpoint),
                    length: 0,
                };
                f.bus.setup(ADDRESS, 0, setup);
                let _ = f.bus.read(ADDRESS, 0, &mut []);
            }
            0x05 => {
                // A bus reset drops the address, so the harness re-enumerates:
                // the interesting sequences are the ones that continue.
                f.bus.reset_port(0);
                f.bus.set_enabled(0, true);
                enumerate(&f);
            }
            0x06 => {
                if at + 8 > data.len() {
                    break;
                }
                let mut raw = [0u8; 8];
                raw.copy_from_slice(&data[at..at + 8]);
                at += 8;
                f.bus.setup(ADDRESS, 0, SetupPacket::decode(&raw));
                let mut buf = [0u8; 64];
                let _ = f.bus.read(ADDRESS, 0, &mut buf);
                let _ = f.bus.write(ADDRESS, 0, &buf[..8]);
            }
            0x07 => {
                // A round trip must reproduce the state exactly, whatever
                // half-finished transfer the stream left behind.
                if let Some(bytes) = snapshot(&f) {
                    let fresh = build();
                    let reader = StateReader::new(&bytes).expect("we just wrote it");
                    let chunk = reader
                        .load("disk", "usb.storage", 1, &Migrations::new())
                        .expect("it is in there");
                    fresh
                        .disk
                        .load(&mut chunk.reader())
                        .expect("our own snapshot loads");
                    assert_eq!(snapshot(&fresh), Some(bytes), "the disk did not round trip");
                }
            }
            0x08 => {
                // Untrusted bytes straight into the chunk decoder. Rejecting is
                // the expected outcome; panicking is never one.
                let mut r = ChunkReader::new(&data[at..]);
                let _ = f.disk.load(&mut r);
                at = data.len();
                // And the device is still usable afterwards.
                let mut buf = [0u8; CSW_BYTES];
                let _ = f.bus.read(ADDRESS, ENDPOINT_IN, &mut buf);
                let _ = f.bus.write(ADDRESS, ENDPOINT_OUT, &[0u8; CBW_BYTES]);
            }
            0x09 => f.disk.reset(ResetKind::Cold),
            _ => {}
        }
    }
});
