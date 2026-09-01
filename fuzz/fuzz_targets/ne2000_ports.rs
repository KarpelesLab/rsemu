#![no_main]
//! The NE2000's 32-port I/O surface, and the ring a guest programs behind it.
//!
//! `CLAUDE.md` asks for a fuzz target on every MMIO surface. This one earns it
//! twice over, because the register block is the smaller half of what a guest
//! controls: the four ring pointers (`PSTART`, `PSTOP`, `BNRY` and `CURR`) are
//! all writable, and the receive path does *page arithmetic* with them. A
//! driver bug sets `PSTOP` below `PSTART` by accident; a hostile guest sets
//! `PSTART` to `0x00`, `PSTOP` to `0x01` and `CURR` to `0xff` on purpose, and
//! then hands the card a 1514-byte frame to write into the two-page ring it
//! just described. The same is true of remote DMA: `RSAR` and `RBCR` are
//! sixteen bits each and address a space that is mostly not there.
//!
//! So this target pokes whatever the fuzzer says into the ports, hands the card
//! whatever frames it says, and advances it — and the properties are the ones
//! no unit test can assert across an arbitrary configuration:
//!
//! > **Nothing panics, and the receive path terminates.** However the ring is
//! > programmed, taking in a frame is bounded work: a wrap that failed to
//! > terminate would show up here as a `cargo fuzz -timeout`.
//!
//! Three more surfaces come along for free:
//!
//! * **A debug read has no side effects.** Every read is made twice with
//!   `MemAttrs::DEBUG` first and must answer identically both times and again
//!   for the guest read that follows. This is the rule the tally counters, the
//!   remote DMA window and the card's reset strap each break if it is written
//!   wrong (`ROADMAP.md` §15, invariant 5).
//! * **A debug write is refused.** Every register here does something — a write
//!   to `CR` transmits, a write to the data window advances a DMA, a write to
//!   the strap resets the card — so the device must refuse all of them.
//! * **The snapshot loader is a parser on untrusted bytes.** A tail of the
//!   input is handed to `Device::load`, which must reject it or accept it and
//!   never panic, and the card must still work afterwards.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 aa dd      write the port at offset (aa mod 0x20)
//!   0x01 aa         read it, guest and debug, and compare
//!   0x02 aa dd dd   write sixteen bits to the data window
//!   0x03 nn         advance nn*64 ticks of the card's clock domain
//!   0x04 nn ...     hand the link an nn-byte frame from the input
//!   0x05            cold reset
//!   0x06            save, then load what was saved: a round trip
//!   0x07 ...        load the rest of the input as a snapshot chunk
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly-rejected.

use std::sync::Arc;

use libfuzzer_sys::fuzz_target;

use rsemu::core::device::{Device, ResetKind};
use rsemu::core::space::{AddressSpace, MemAttrs};
use rsemu::core::state::{MachineShape, Migrations, Sink, StateReader, StateWriter};
use rsemu::core::value::Width;
use rsemu::dev::net::link::{MacAddr, NetLink, NetPort};
use rsemu::dev::net::ne2000::{CLASS, Ne2000, REGISTER_WINDOW_LEN};

/// Where the card is mapped. The conventional NE2000 base, and a non-zero one
/// so an offset bug shows up as a wrong address rather than as zero.
const BASE: u64 = 0x0300;

/// The card, the space it answers on, and the far end of its wire.
struct Rig {
    card: Ne2000,
    space: AddressSpace,
    port: Arc<NetPort>,
}

fn build() -> Rig {
    let port = Arc::new(NetPort::new());
    let card = Ne2000::with_link(
        Arc::clone(&port) as Arc<dyn NetLink>,
        String::from("fuzz"),
        MacAddr::new([0x52, 0x54, 0x00, 0x12, 0x34, 0x56]),
    );
    let space = AddressSpace::new("port", 16);
    space
        .topology()
        .map(card.region("").expect("the card has a register block"), BASE)
        .expect("32 ports fit in 64 KiB");
    Rig { card, space, port }
}

fn snapshot(card: &Ne2000) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("nic", CLASS.name).ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("nic", CLASS.name, CLASS.version).ok()?;
        card.save(&mut chunk).ok()?;
    }
    w.to_vec().ok()
}

fuzz_target!(|data: &[u8]| {
    let rig = build();
    let mut at = 0usize;
    // The card is lazy, so it may only ever be advanced forwards.
    let mut tick = 0u64;

    while at < data.len() {
        let op = data[at];
        at += 1;
        match op {
            0x00 => {
                if at + 2 > data.len() {
                    break;
                }
                let offset = BASE + u64::from(data[at]) % REGISTER_WINDOW_LEN;
                let value = u64::from(data[at + 1]);
                at += 2;
                // A refusal is a legal outcome; a panic is not.
                let _ = rig.space.write(offset, Width::U8, value, MemAttrs::DEFAULT);
                assert!(
                    rig.space
                        .write(offset, Width::U8, value, MemAttrs::DEBUG)
                        .is_err(),
                    "a debug write to {offset:#06x} was accepted"
                );
            }
            0x01 => {
                if at >= data.len() {
                    break;
                }
                let offset = BASE + u64::from(data[at]) % REGISTER_WINDOW_LEN;
                at += 1;
                // Debug first, twice: a debug read that changed anything would
                // disagree with itself.
                let first = rig.space.read(offset, Width::U8, MemAttrs::DEBUG);
                let second = rig.space.read(offset, Width::U8, MemAttrs::DEBUG);
                assert_eq!(
                    first.is_ok(),
                    second.is_ok(),
                    "two debug reads of {offset:#06x} disagreed about legality"
                );
                if let (Ok(a), Ok(b)) = (first, second) {
                    assert_eq!(a, b, "a debug read of {offset:#06x} had a side effect");
                    let live = rig
                        .space
                        .read(offset, Width::U8, MemAttrs::DEFAULT)
                        .expect("a guest read is legal wherever a debug read was");
                    assert_eq!(
                        live, a,
                        "a debug read of {offset:#06x} answered differently from the guest's"
                    );
                }
            }
            0x02 => {
                if at + 3 > data.len() {
                    break;
                }
                // The data window is the only place a 16-bit access is legal,
                // and only once `DCR.WTS` says so — so most of these are
                // refused, which is itself worth exercising.
                let offset = BASE + 0x10 + u64::from(data[at]) % 8;
                let value = u64::from(u16::from_le_bytes([data[at + 1], data[at + 2]]));
                at += 3;
                let _ = rig
                    .space
                    .write(offset, Width::U16, value, MemAttrs::DEFAULT);
                let _ = rig.space.read(offset, Width::U16, MemAttrs::DEFAULT);
            }
            0x03 => {
                if at >= data.len() {
                    break;
                }
                // Forwards only, and far enough that a queued frame comes due.
                tick += u64::from(data[at]) * 64 + 1;
                at += 1;
                rig.card.advance_to(tick);
            }
            0x04 => {
                if at >= data.len() {
                    break;
                }
                let len = usize::from(data[at]);
                at += 1;
                let end = (at + len).min(data.len());
                // Frames from the wire, at whatever tick the fuzzer has
                // reached. The card must survive a runt, a giant and a frame
                // for somebody else alike.
                rig.port.deliver_at(tick, &data[at..end]);
                at = end;
            }
            0x05 => rig.card.reset(ResetKind::Cold),
            0x06 => {
                // A round trip has to come back: whatever state the ports have
                // been driven into, saving and reloading it must reproduce the
                // same bytes.
                if let Some(bytes) = snapshot(&rig.card) {
                    let reader = StateReader::new(&bytes).expect("what we just wrote");
                    let chunk = reader
                        .load("nic", CLASS.name, CLASS.version, &Migrations::new())
                        .expect("the chunk we just wrote");
                    rig.card
                        .load(&mut chunk.reader())
                        .expect("our own snapshot loads");
                    assert_eq!(
                        snapshot(&rig.card).as_deref(),
                        Some(&bytes[..]),
                        "a save/load round trip changed the card's state"
                    );
                }
            }
            0x07 => {
                // The loader on bytes nobody wrote. Rejecting is the expected
                // outcome; panicking is not, and neither is being unusable
                // afterwards.
                let mut shape = MachineShape::new();
                if shape.add_device("nic", CLASS.name).is_ok() {
                    let mut w = StateWriter::new(shape);
                    if let Ok(mut chunk) = w.chunk("nic", CLASS.name, CLASS.version) {
                        let _ = chunk.write_bytes(&data[at..]);
                    }
                    if let Ok(bytes) = w.to_vec()
                        && let Ok(reader) = StateReader::new(&bytes)
                        && let Ok(chunk) =
                            reader.load("nic", CLASS.name, CLASS.version, &Migrations::new())
                    {
                        let _ = rig.card.load(&mut chunk.reader());
                    }
                }
                at = data.len();
                // Still alive: the registers still answer.
                let _ = rig.space.read(BASE, Width::U8, MemAttrs::DEFAULT);
            }
            _ => {}
        }
    }

    // Whatever happened, the card is still a card: it answers a read, it takes
    // a reset, and it advances.
    let _ = rig.space.read(BASE, Width::U8, MemAttrs::DEFAULT);
    rig.card.reset(ResetKind::Cold);
    rig.card.advance_to(tick + 1);
});
