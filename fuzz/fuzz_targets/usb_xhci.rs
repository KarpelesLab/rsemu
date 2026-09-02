#![no_main]
//! The xHCI MMIO surface, and the rings and contexts a guest builds behind it.
//!
//! `CLAUDE.md` asks for a fuzz target on "every MMIO surface". This controller
//! deserves one more than most, for the same reason the EHCI does and then
//! some: its register block is the *smaller* half of its attack surface. The
//! controller is a bus master that walks a **Device Context Base Address
//! Array**, a **command ring**, thirty-one **transfer rings** per slot and an
//! **Event Ring Segment Table**, and every pointer in all of them comes from
//! memory the guest wrote.
//!
//! And a ring is a cycle *by construction*. A Link TRB may point at itself; a
//! ring may be nothing but Link TRBs; a Transfer Descriptor may chain
//! arbitrarily many TRBs; a command may make another command runnable; a
//! context may name a transfer ring that is the command ring. A driver bug does
//! each of those by accident and a hostile guest does them on purpose.
//!
//! So this target fills guest RAM with whatever the fuzzer says, pokes whatever
//! it says into the registers and the doorbells, and advances the controller —
//! and the property being tested is the one that cannot be unit-tested across
//! an arbitrary structure:
//!
//! > **Every walk terminates, and none of them panics.** A doorbell returns
//! > whatever the rings say, because every loop over a guest pointer is
//! > bounded: `MAX_LINK_HOPS`, `MAX_TRBS_PER_TD`, `MAX_WORK_ITEMS`,
//! > `MAX_PACKETS`.
//!
//! An unbounded walk shows up here as a timeout, which is what `cargo fuzz`'s
//! `-timeout` reports, and is the failure this target exists to catch.
//!
//! **The register block is mapped into the space the controller masters**, at
//! `REGS`, which is deliberate: it means the fuzzer can point a TRB's data
//! buffer, a context, the DCBAA or an ERST entry straight at the doorbell array
//! and re-enter the engine from inside its own DMA. Four bytes of anything are
//! a doorbell write. That is the hazard `dev-nvme` names for a PRP entry aimed
//! at `SQyTDBL`, and the answer is the same: the work is iterative, not
//! recursive.
//!
//! Three other surfaces come along for free:
//!
//! * **A debug read has no side effects.** Every register read is repeated with
//!   `MemAttrs::DEBUG` and must answer identically without consuming a TRB or
//!   advancing a dequeue pointer (`ROADMAP.md` §15, invariant 5).
//! * **A debug write is refused.** A doorbell has no harmless version, and
//!   `USBSTS`, `IMAN.IP`, `ERDP.EHB` and every `PORTSC` change bit are
//!   write-1-to-clear.
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
//!   0x00 aa aa dd dd dd dd   write the register at (aaaa mod 0x3000) & !3
//!   0x01 aa aa               read it, and compare against a debug read
//!   0x02 aa aa dd dd dd dd   poke a dword into guest RAM at (aaaa mod size)
//!   0x03 nn                  advance nn microframes
//!   0x04                     cold reset
//!   0x05                     save, then load what was saved: a round trip
//!   0x06 ...                 load the rest of the input as a snapshot chunk
//!   0x07 ss tt               ring doorbell ss with target tt
//!   0x08                     write a plausible initialisation sequence
//! ```
//!
//! Opcode `0x08` exists because an xHCI that has never had a `DCBAAP` or an
//! `ERSTBA` written does almost nothing, and a fuzzer would take a very long
//! time to guess sixteen dwords of set-up. It writes the registers the §4.2
//! initialisation sequence writes, pointing them at fixed RAM addresses the
//! fuzzer's `0x02` opcode can then fill with whatever it likes.
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
use rsemu::dev::usb::hid::HidMouse;
use rsemu::dev::usb::xhci::{Params, XhciController, offset};
use std::sync::Arc;

/// Where guest RAM starts. Not zero, so a null pointer in a context is a bus
/// fault the controller has to survive rather than a plausible read.
const RAM_BASE: u64 = 0x1000;
/// How much of it there is.
const RAM_SIZE: u64 = 0xf000;
/// Where the register block is mapped, in the same space the controller
/// masters — see the module docs.
const REGS: u64 = 0x10_0000;
/// A short microframe, so the fuzzer's "advance" op is cheap.
const MICROFRAME: u64 = 8;
/// How much of the register block is reachable.
const REGISTERS: u64 = 0x3000;

/// The addresses opcode `0x08` points the controller at.
const DCBAA: u64 = RAM_BASE + 0x000;
const ERST: u64 = RAM_BASE + 0x040;
const CMD_RING: u64 = RAM_BASE + 0x100;
const EVT_RING: u64 = RAM_BASE + 0x200;

struct Fixture {
    controller: XhciController,
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

    let controller = XhciController::with_bus(
        Arc::clone(&bus),
        Params {
            ports: 2,
            slots: 4,
            microframe_ticks: MICROFRAME,
        },
    );
    controller.xhci().attach_space(&space, RequesterId(7));

    let region = controller.region("").expect("the register block");
    {
        let mut topo = space.topology();
        topo.map(Arc::clone(&region), REGS).expect("the map fits");
    }
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
    shape.add_device("xhci", "usb.xhci").ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("xhci", "usb.xhci", 1).ok()?;
        f.controller.save(&mut chunk).ok()?;
    }
    w.to_vec().ok()
}

fn poke(f: &Fixture, addr: u64, value: u32) {
    let _ = f
        .space
        .write(addr, Width::U32, u64::from(value), MemAttrs::DEBUG);
}

fn reg(f: &Fixture, offset: u64, value: u32) {
    let _ = f.ops.write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT);
}

/// The §4.2 initialisation sequence, so the fuzzer does not have to guess it.
fn initialise(f: &Fixture) {
    let op = offset::OPERATIONAL;
    let ir0 = offset::RUNTIME + offset::INTERRUPTER0;
    // The Device Context Base Address Array, one Event Ring Segment Table entry
    // and a zeroed device context: enough that a doorbell has somewhere to go.
    poke(f, DCBAA + 8, (RAM_BASE + 0x400) as u32);
    poke(f, DCBAA + 12, 0);
    poke(f, ERST, EVT_RING as u32);
    poke(f, ERST + 4, 0);
    poke(f, ERST + 8, 16);
    poke(f, ERST + 12, 0);

    reg(f, op + 0x38, 4); // CONFIG: four device slots
    reg(f, op + 0x30, DCBAA as u32); // DCBAAP
    reg(f, op + 0x34, 0);
    reg(f, op + 0x18, CMD_RING as u32 | 1); // CRCR, RCS = 1
    reg(f, op + 0x1c, 0);
    reg(f, ir0 + 0x08, 1); // ERSTSZ
    reg(f, ir0 + 0x18, EVT_RING as u32); // ERDP
    reg(f, ir0 + 0x1c, 0);
    reg(f, ir0 + 0x10, ERST as u32); // ERSTBA
    reg(f, ir0 + 0x14, 0);
    reg(f, ir0 + 0x04, 0); // IMOD: no throttling
    reg(f, ir0 + 0x00, 2); // IMAN.IE
    reg(f, op + 0x00, 0x5); // USBCMD: RS | INTE
    // Acknowledge the attach, then reset the port, which is what enables it.
    reg(f, op + 0x400, (1 << 9) | (1 << 17));
    reg(f, op + 0x400, (1 << 9) | (1 << 4));
}

fuzz_target!(|data: &[u8]| {
    let f = build();
    let mut at = 0usize;

    while at < data.len() {
        let op = data[at];
        at += 1;
        match op {
            0x00 => {
                if at + 6 > data.len() {
                    break;
                }
                let offset =
                    (u64::from(u16::from_le_bytes([data[at], data[at + 1]])) % REGISTERS) & !0x3;
                let value = u32::from_le_bytes([
                    data[at + 2],
                    data[at + 3],
                    data[at + 4],
                    data[at + 5],
                ]);
                at += 6;
                // A refusal is a legal outcome; a panic is not.
                let _ = f.ops.write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT);
                // A debug write must never be accepted.
                assert!(
                    f.ops
                        .write(offset, &value.to_le_bytes(), MemAttrs::DEBUG)
                        .is_err(),
                    "a debug write to the xHCI register block must be refused"
                );
            }
            0x01 => {
                if at + 2 > data.len() {
                    break;
                }
                let offset =
                    (u64::from(u16::from_le_bytes([data[at], data[at + 1]])) % REGISTERS) & !0x3;
                at += 2;
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
                poke(&f, RAM_BASE + addr, value);
            }
            0x03 => {
                if at >= data.len() {
                    break;
                }
                let steps = u64::from(data[at]);
                at += 1;
                // **The property.** Whatever the rings in RAM say, this
                // returns: every walk over a guest-built structure is bounded.
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
                        .load("xhci", "usb.xhci", 1, &Migrations::new())
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
                if at + 2 > data.len() {
                    break;
                }
                // A doorbell, which is the one write that makes the controller
                // walk everything at once (§5.6).
                let index = u64::from(data[at] & 0x7);
                let target = u32::from(data[at + 1]);
                at += 2;
                let _ = f.ops.write(
                    offset::DOORBELL + index * 4,
                    &target.to_le_bytes(),
                    MemAttrs::DEFAULT,
                );
            }
            0x08 => initialise(&f),
            0x09 => {
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
