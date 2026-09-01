#![no_main]
//! The dwc2 MMIO surface: the register block, and the FIFO windows behind it.
//!
//! `CLAUDE.md` asks for a fuzz target on "every MMIO surface". This controller's
//! surface is unusual, and that is why it gets its own target rather than
//! sharing the EHCI's. An EHCI is dangerous because it *reads guest memory*: a
//! linked list the guest built can close a circle. This one never reads memory
//! at all — everything it does comes out of registers and a FIFO the guest fills
//! a word at a time — so the hazards are different ones:
//!
//! > **A frame always ends, and nothing the guest writes makes this device
//! > allocate.** Every channel's transmit staging and the shared receive FIFO
//! > are capped by the programmed FIFO sizes, themselves capped by the RAM the
//! > part has; the frame is capped by a byte budget taken from the signalling
//! > rate and by a hard transaction count; and `HFIR.FRIVL` is clamped so that a
//! > frame interval a guest chose cannot make the host spin.
//!
//! An unbounded frame shows up here as a timeout, which is what `cargo fuzz`'s
//! `-timeout` reports. Unbounded growth shows up as an out-of-memory, which is
//! what `-rss_limit_mb` reports. Both are failures this target exists to catch.
//!
//! Three other surfaces come along for free:
//!
//! * **A debug read has no side effects** — and this device has two registers
//!   where that is a real question rather than a formality: `GRXSTSP` pops the
//!   receive FIFO when it is read, and a FIFO window read consumes the packet.
//!   Every read is repeated with `MemAttrs::DEBUG` and must answer identically
//!   twice over (`ROADMAP.md` §15, invariant 5).
//! * **A debug write is refused.** `HCINTn` is write-1-to-clear, `GRSTCTL`
//!   resets the core and `HCCHAR.CHENA` puts a transaction on the wire.
//! * **The snapshot loader is a parser on untrusted bytes.** A tail of the input
//!   is handed to `Device::load`, which must reject it or accept it and never
//!   panic — and the device must still be usable afterwards.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 aa dd dd dd dd   write the register at offset (aa*4 mod 0xe00)
//!   0x01 aa               read it, and compare against a debug read
//!   0x02 cc dd dd dd dd   push a word into channel (cc mod 16)'s FIFO window
//!   0x03 cc               read a word out of a FIFO window
//!   0x04 nn               advance nn frames
//!   0x05                  cold reset
//!   0x06                  save, then load what was saved: a round trip
//!   0x07 ...              load the rest of the input as a snapshot chunk
//!   0x08 xx yy bb         move the mouse
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly-rejected.

use libfuzzer_sys::fuzz_target;

use rsemu::bus::usb::{Speed, UsbBus};
use rsemu::core::device::{Device, ResetKind};
use rsemu::core::space::{MemAttrs, MemOps, RegionKind};
use rsemu::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use rsemu::dev::usb::dwc2::{Dwc2Controller, FIFO_BASE, FIFO_WINDOW, Params};
use rsemu::dev::usb::hid::HidMouse;
use std::sync::Arc;

/// A short frame, so the fuzzer's "advance" op is cheap. It is also the
/// smallest interval the model honours, which is the clamp being exercised.
const FRAME: u64 = 1000;

/// How much of the register block is reachable, past which the FIFO windows
/// start.
const REGISTERS: u64 = 0xe00;

struct Fixture {
    controller: Dwc2Controller,
    ops: Arc<dyn MemOps>,
    mouse: HidMouse,
}

fn build() -> Fixture {
    let bus = Arc::new(UsbBus::new(1));
    // Full speed, because that is what this transceiver is: a high-speed device
    // would never enable the port and half the target would be unreachable.
    let mouse = HidMouse::new_detached_at_speed(0x1234, 0x5678, Speed::Full);
    bus.attach(0, mouse.device()).expect("an empty port");

    let controller = Dwc2Controller::with_bus(
        Arc::clone(&bus),
        Params {
            channels: 8,
            fifo_words: 320,
            phy_ticks: 1,
            max_speed: Speed::Full,
            cid: 0x1000,
        },
    );

    let region = controller.region("").expect("the register block");
    let ops = match region.kind() {
        RegionKind::Io(ops) => Arc::clone(ops),
        _ => panic!("expected an io region"),
    };
    Fixture {
        controller,
        ops,
        mouse,
    }
}

fn snapshot(f: &Fixture) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("dwc2", "usb.dwc2").ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("dwc2", "usb.dwc2", 1).ok()?;
        f.controller.save(&mut chunk).ok()?;
    }
    w.to_vec().ok()
}

fn fifo_window(channel: u8) -> u64 {
    FIFO_BASE + u64::from(channel % 16) * FIFO_WINDOW
}

fuzz_target!(|data: &[u8]| {
    let f = build();
    let mut at = 0usize;

    while at < data.len() {
        let op = data[at];
        at += 1;
        match op {
            0x00 => {
                if at + 5 > data.len() {
                    break;
                }
                let offset = (u64::from(data[at]) * 4) % REGISTERS;
                let value =
                    u32::from_le_bytes([data[at + 1], data[at + 2], data[at + 3], data[at + 4]]);
                at += 5;
                // A refusal is a legal outcome; a panic is not.
                let _ = f.ops.write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT);
                assert!(
                    f.ops
                        .write(offset, &value.to_le_bytes(), MemAttrs::DEBUG)
                        .is_err(),
                    "a debug write to the dwc2 register block must be refused"
                );
            }
            0x01 => {
                if at >= data.len() {
                    break;
                }
                let offset = (u64::from(data[at]) * 4) % REGISTERS;
                at += 1;
                let mut live = [0u8; 4];
                let mut dbg = [0u8; 4];
                let ok = f.ops.read(offset, &mut live, MemAttrs::DEFAULT).is_ok();
                let dbg_ok = f.ops.read(offset, &mut dbg, MemAttrs::DEBUG).is_ok();
                assert_eq!(
                    ok, dbg_ok,
                    "a debug read and a guest read disagree about what is mapped"
                );
                if ok {
                    // `GRXSTSP` is the one that matters: a debug read of it must
                    // pop nothing, so two of them agree.
                    let mut again = [0u8; 4];
                    f.ops
                        .read(offset, &mut again, MemAttrs::DEBUG)
                        .expect("it read a moment ago");
                    assert_eq!(dbg, again, "a debug read had a side effect");
                }
            }
            0x02 => {
                if at + 5 > data.len() {
                    break;
                }
                let window = fifo_window(data[at]);
                let value =
                    u32::from_le_bytes([data[at + 1], data[at + 2], data[at + 3], data[at + 4]]);
                at += 5;
                // **The allocation property.** However many words the guest
                // pushes, the staging is capped by the FIFO it declared.
                let _ = f.ops.write(window, &value.to_le_bytes(), MemAttrs::DEFAULT);
            }
            0x03 => {
                if at >= data.len() {
                    break;
                }
                let window = fifo_window(data[at]);
                at += 1;
                let mut word = [0u8; 4];
                let _ = f.ops.read(window, &mut word, MemAttrs::DEFAULT);
                let mut peeked = [0u8; 4];
                let mut again = [0u8; 4];
                let _ = f.ops.read(window, &mut peeked, MemAttrs::DEBUG);
                let _ = f.ops.read(window, &mut again, MemAttrs::DEBUG);
                assert_eq!(peeked, again, "a debug FIFO read consumed the packet");
            }
            0x04 => {
                if at >= data.len() {
                    break;
                }
                let steps = u64::from(data[at]);
                at += 1;
                // **The termination property.** Whatever the channels and the
                // FIFO say, this returns.
                let now = f.controller.current_tick();
                f.controller.advance_to(now + steps * FRAME + FRAME);
            }
            0x05 => f.controller.reset(ResetKind::Cold),
            0x06 => {
                if let Some(bytes) = snapshot(&f) {
                    let fresh = build();
                    let reader = StateReader::new(&bytes).expect("we just wrote it");
                    let chunk = reader
                        .load("dwc2", "usb.dwc2", 1, &Migrations::new())
                        .expect("it is in there");
                    fresh
                        .controller
                        .load(&mut chunk.reader())
                        .expect("our own snapshot loads");
                    assert_eq!(
                        snapshot(&fresh),
                        Some(bytes),
                        "the controller did not round trip"
                    );
                }
            }
            0x07 => {
                // Untrusted bytes straight into the chunk decoder. Rejecting is
                // the expected outcome; panicking is never one.
                let mut r = ChunkReader::new(&data[at..]);
                let _ = f.controller.load(&mut r);
                at = data.len();
                // And the device is still usable.
                let now = f.controller.current_tick();
                f.controller.advance_to(now + FRAME);
            }
            0x08 => {
                if at + 3 > data.len() {
                    break;
                }
                f.mouse
                    .motion(data[at] as i8, data[at + 1] as i8, data[at + 2]);
                at += 3;
            }
            _ => {}
        }
    }
});
