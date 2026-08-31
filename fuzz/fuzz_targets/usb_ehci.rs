#![no_main]
//! The EHCI MMIO surface, and the linked lists a guest builds behind it.
//!
//! `CLAUDE.md` asks for a fuzz target on "every MMIO surface". This device
//! deserves one more than most, because its register block is the smaller half
//! of its attack surface: the controller is a **bus master that walks
//! guest-built linked lists**, and every pointer in a queue head, a transfer
//! descriptor and a periodic frame list comes from memory the guest wrote. A
//! queue head can link to itself, a descriptor can point at itself, a frame
//! list can close a circle, and a buffer pointer can address a page that is not
//! mapped. A driver bug does each of those by accident; a hostile guest does
//! them on purpose.
//!
//! So this target fills guest RAM with whatever the fuzzer says, pokes whatever
//! it says into the registers, and advances the controller — and the property
//! being tested is the one that cannot be unit-tested across an arbitrary
//! structure:
//!
//! > **Every walk terminates, and none of them panics.** The controller comes
//! > back from a microframe whatever the schedule says, because every loop over
//! > a guest pointer is bounded.
//!
//! An unbounded walk would show up here as a timeout, which is what
//! `cargo fuzz`'s `-timeout` reports, and is the failure this target exists to
//! catch.
//!
//! Three other surfaces come along for free:
//!
//! * **A debug read has no side effects.** Every register read is repeated with
//!   `MemAttrs::DEBUG` and must answer identically without advancing the frame
//!   counter (`ROADMAP.md` §15, invariant 5).
//! * **A debug write is refused.** `USBSTS` is write-1-to-clear, and a debugger
//!   that acknowledged an interrupt would be a debugger that changed the guest.
//! * **The snapshot loader is a parser on untrusted bytes.** A tail of the
//!   input is handed to `Device::load`, which must reject it or accept it and
//!   never panic — and the device must still be usable afterwards.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 aa dd dd dd dd   write the register at offset (aa*4 mod 0x100)
//!   0x01 aa               read it, and compare against a debug read
//!   0x02 aa aa dd dd dd dd  poke a dword into guest RAM at (aaaa mod size)
//!   0x03 nn               advance nn microframes
//!   0x04                  cold reset
//!   0x05                  save, then load what was saved: a round trip
//!   0x06 ...              load the rest of the input as a snapshot chunk
//!   0x07 xx yy bb         move the mouse
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly-rejected.

use libfuzzer_sys::fuzz_target;

use rsemu::bus::usb::UsbBus;
use rsemu::core::device::{Device, ResetKind};
use rsemu::core::space::{
    AddressSpace, MemAttrs, MemOps, RamStore, Region, RegionKind, RequesterId,
};
use rsemu::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use rsemu::core::value::Width;
use rsemu::dev::usb::ehci::{DEFAULT_CAPLENGTH, EhciController, Params};
use rsemu::dev::usb::hid::HidMouse;
use std::sync::Arc;

/// Where guest RAM starts. Not zero, so a null pointer in a descriptor is a
/// bus fault the controller has to survive rather than a plausible read.
const RAM_BASE: u64 = 0x1000;
/// How much of it there is.
const RAM_SIZE: u64 = 0xf000;
/// A short microframe, so the fuzzer's "advance" op is cheap.
const MICROFRAME: u64 = 8;
/// How much of the register block is reachable.
const REGISTERS: u64 = 0x100;

struct Fixture {
    controller: EhciController,
    space: Arc<AddressSpace>,
    ops: Arc<dyn MemOps>,
    mouse: HidMouse,
}

fn build() -> Fixture {
    let space = AddressSpace::new("mem", 32);
    {
        let mut topo = space.topology();
        topo.map(
            Region::ram("ram", Arc::new(RamStore::new(RAM_SIZE))),
            RAM_BASE,
        )
        .expect("the map fits");
    }
    let space = Arc::new(space);

    let bus = Arc::new(UsbBus::new(2));
    let mouse = HidMouse::new_detached(0x1234, 0x5678);
    bus.attach(0, mouse.device()).expect("an empty port");

    let controller = EhciController::with_bus(
        Arc::clone(&bus),
        Params {
            ports: 2,
            microframe_ticks: MICROFRAME,
            caplength: DEFAULT_CAPLENGTH,
            dual_role: false,
        },
    );
    controller.hcd().attach_space(&space, RequesterId(7));

    let region = controller.region("").expect("the register block");
    let ops = match region.kind() {
        RegionKind::Io(ops) => Arc::clone(ops),
        _ => panic!("expected an io region"),
    };
    Fixture {
        controller,
        space,
        ops,
        mouse,
    }
}

fn snapshot(f: &Fixture) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("ehci", "usb.ehci").ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("ehci", "usb.ehci", 1).ok()?;
        f.controller.save(&mut chunk).ok()?;
    }
    w.to_vec().ok()
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
                let value = u32::from_le_bytes([
                    data[at + 1],
                    data[at + 2],
                    data[at + 3],
                    data[at + 4],
                ]);
                at += 5;
                // A refusal is a legal outcome; a panic is not.
                let _ = f.ops.write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT);
                // A debug write must never be accepted: `USBSTS` is
                // write-1-to-clear and `USBCMD` starts the controller.
                assert!(
                    f.ops
                        .write(offset, &value.to_le_bytes(), MemAttrs::DEBUG)
                        .is_err(),
                    "a debug write to the EHCI register block must be refused"
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
                    // Reading twice must give the same answer: no register in
                    // this block is read-to-clear, and a debug read must not
                    // have advanced anything either.
                    let mut again = [0u8; 4];
                    f.ops
                        .read(offset, &mut again, MemAttrs::DEBUG)
                        .expect("it read a moment ago");
                    assert_eq!(dbg, again, "a debug read had a side effect");
                }
            }
            0x02 => {
                if at + 6 > data.len() {
                    break;
                }
                let addr = (u64::from(u16::from_le_bytes([data[at], data[at + 1]]))
                    % (RAM_SIZE - 4))
                    & !0x3;
                let value = u32::from_le_bytes([
                    data[at + 2],
                    data[at + 3],
                    data[at + 4],
                    data[at + 5],
                ]);
                at += 6;
                let _ = f.space.write(
                    RAM_BASE + addr,
                    Width::U32,
                    u64::from(value),
                    MemAttrs::DEBUG,
                );
            }
            0x03 => {
                if at >= data.len() {
                    break;
                }
                let steps = u64::from(data[at]);
                at += 1;
                // **The property.** Whatever the schedule in RAM says, this
                // returns: every walk over a guest-built list is bounded.
                let now = f.controller.current_tick();
                f.controller
                    .advance_to(now + steps * MICROFRAME + MICROFRAME);
            }
            0x04 => f.controller.reset(ResetKind::Cold),
            0x05 => {
                // A round trip must reproduce the register file exactly,
                // whatever half-configured state the stream left behind.
                if let Some(bytes) = snapshot(&f) {
                    let fresh = build();
                    let reader = StateReader::new(&bytes).expect("we just wrote it");
                    let chunk = reader
                        .load("ehci", "usb.ehci", 1, &Migrations::new())
                        .expect("it is in there");
                    fresh
                        .controller
                        .load(&mut chunk.reader())
                        .expect("our own snapshot loads");
                    assert_eq!(snapshot(&fresh), Some(bytes), "the controller did not round trip");
                }
            }
            0x06 => {
                // Untrusted bytes straight into the chunk decoder. Rejecting is
                // the expected outcome; panicking is never one.
                let mut r = ChunkReader::new(&data[at..]);
                let _ = f.controller.load(&mut r);
                at = data.len();
                // And the device is still usable.
                let now = f.controller.current_tick();
                f.controller.advance_to(now + MICROFRAME);
            }
            0x07 => {
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
