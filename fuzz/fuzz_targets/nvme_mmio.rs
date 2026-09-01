#![no_main]
//! The NVMe MMIO surface, and the queues and PRP chains a guest builds behind
//! it.
//!
//! `CLAUDE.md` asks for a fuzz target on "every MMIO surface", and this device
//! wants one more than most for the same reason the EHCI does: its register
//! block is the smaller half of its attack surface. The controller is a **bus
//! master that walks structures the guest wrote** — a 64-byte command fetched
//! out of a submission queue whose base the guest chose, a PRP entry, a PRP
//! List whose last entry may point at another list, a completion written back
//! to a queue whose base the guest also chose. A driver bug closes a ring by
//! accident; a hostile guest does it on purpose.
//!
//! So this target pokes whatever the fuzzer says into the registers and into
//! guest RAM, and the properties being checked are the ones no unit test can
//! cover across an arbitrary structure:
//!
//! > **Every walk terminates, and none of them panics.** A doorbell write
//! > returns whatever the queues say, because every loop over a guest pointer
//! > is bounded ([`MAX_PRP_LISTS`], and the completion-queue-full back
//! > pressure that stops the command loop).
//!
//! An unbounded walk shows up here as a timeout, which is what `cargo fuzz`'s
//! `-timeout` reports.
//!
//! Three more surfaces come along for free, and one of them is specific to this
//! device:
//!
//! * **The register block is mapped in the same address space the controller
//!   masters**, at an address a PRP entry can name. So the fuzzer can and will
//!   aim a transfer at the controller's own doorbells, which re-enters the
//!   write handler from inside itself. The engine is iterative rather than
//!   recursive, and a stack overflow here would say otherwise.
//! * **A debug read has no side effects**, and **a debug write is refused**:
//!   submitting a command is a register write, so there is no harmless subset.
//! * **The snapshot loader is a parser on untrusted bytes.** A tail of the
//!   input is handed to `Controller::load`, which must reject it or accept it
//!   and never panic — a completion queue with zero entries would divide by
//!   zero the first time a doorbell moved its head — and the controller must
//!   still be usable afterwards.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 aa dd dd dd dd   write a register: aa < 0x80 picks one of the
//!                         controller registers, aa >= 0x80 picks a doorbell
//!   0x01 aa               read it, and compare against a debug read
//!   0x02 aa aa dd dd dd dd  poke a dword into guest RAM at (aaaa mod size)
//!   0x03                  reset
//!   0x04                  save, then load what was saved: a round trip
//!   0x05                  bus master enable off, then on
//!   0x06 ...              load the rest of the input as a snapshot chunk
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly-rejected.

use libfuzzer_sys::fuzz_target;

use rsemu::core::space::{
    AddressSpace, MemAttrs, RamStore, Region, RequesterId, UnassignedPolicy,
};
use rsemu::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use rsemu::core::sync::{LockRank, Mutex};
use rsemu::core::value::Width;
use rsemu::core::wire::{Level, Wire, WireId, WireIdAllocator, WireSink, WireSource};
use rsemu::dev::ata::Medium;
use rsemu::dev::nvme::{Controller, Namespace, Params, REGISTER_LEN};
use std::sync::Arc;

/// Where guest RAM starts. Not zero, so a null pointer in a command is a bus
/// fault the controller has to survive rather than a plausible read.
const RAM_BASE: u64 = 0x1000;
/// How much of it there is.
const RAM_LEN: u64 = 0x20_0000;
/// Where the register block is mapped — inside the same space, so a PRP entry
/// can point at it.
const REGS: u64 = 0x1000_0000;
/// How big the namespace is.
const NS_LEN: u64 = 64 * 1024;

/// A sink that only remembers the level, so the interrupt has somewhere to go.
#[derive(Debug)]
struct Pin(Mutex<Level>);

impl WireSink for Pin {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        *self.0.lock() = level;
    }
}

struct Fixture {
    ctrl: Arc<Controller>,
    space: Arc<AddressSpace>,
}

fn build() -> Fixture {
    let store: Arc<dyn Medium> = Arc::new(RamStore::new(NS_LEN));
    let ns = Namespace::new(store, 9, false).expect("512-byte blocks");
    let ctrl = Arc::new(Controller::new(ns, Params::default()));

    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ONES));
    {
        let mut topo = space.topology();
        topo.map(
            Region::ram("ram", Arc::new(RamStore::new(RAM_LEN))),
            RAM_BASE,
        )
        .expect("the map fits");
        topo.map(
            Region::io(
                "nvme.regs",
                REGISTER_LEN,
                Arc::clone(&ctrl) as Arc<dyn rsemu::core::space::MemOps>,
            ),
            REGS,
        )
        .expect("the map fits");
    }
    ctrl.attach_space(&space, RequesterId(7));
    ctrl.set_master(true);

    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let pin: Arc<dyn WireSink> = Arc::new(Pin(Mutex::with_rank(LockRank::LEAF, Level::Low)));
    let wire = Wire::builder().source(id).sink(pin, 0).build_shared();
    ctrl.connect_irq(WireSource::new(wire, id));

    Fixture { ctrl, space }
}

/// Which register byte `sel` names: the controller registers below `0x80`, the
/// doorbell array above it.
fn offset_of(sel: u8) -> u64 {
    if sel < 0x80 {
        u64::from(sel % 0x10) * 4
    } else {
        0x1000 + u64::from(sel % 0x20) * 4
    }
}

fn snapshot(f: &Fixture) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("nvme", "nvme.controller").ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("nvme", "nvme.controller", 1).ok()?;
        f.ctrl.save(&mut chunk).ok()?;
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
                let offset = offset_of(data[at]);
                let value = u32::from_le_bytes([
                    data[at + 1],
                    data[at + 2],
                    data[at + 3],
                    data[at + 4],
                ]);
                at += 5;
                // **The property.** Whatever the queues in RAM say, this
                // returns: every walk over a guest-built structure is bounded,
                // and a transfer aimed at these very registers re-enters
                // iteratively rather than recursively.
                let _ = f.space.write(
                    REGS + offset,
                    Width::U32,
                    u64::from(value),
                    MemAttrs::DEFAULT,
                );
                // A debug write must never be accepted: `CC.EN` starts and
                // stops the controller and a doorbell runs a command.
                assert!(
                    f.space
                        .write(REGS + offset, Width::U32, u64::from(value), MemAttrs::DEBUG)
                        .is_err(),
                    "a debug write to the NVMe register block must be refused"
                );
            }
            0x01 => {
                if at >= data.len() {
                    break;
                }
                let offset = offset_of(data[at]);
                at += 1;
                let live = f.space.read(REGS + offset, Width::U32, MemAttrs::DEFAULT);
                let dbg = f.space.read(REGS + offset, Width::U32, MemAttrs::DEBUG);
                assert_eq!(
                    live.is_ok(),
                    dbg.is_ok(),
                    "a debug read and a guest read disagree about what is mapped"
                );
                if let (Ok(live), Ok(dbg)) = (live, dbg) {
                    assert_eq!(live, dbg, "a debug read answered differently");
                    let again = f
                        .space
                        .read(REGS + offset, Width::U32, MemAttrs::DEBUG)
                        .expect("it read a moment ago");
                    assert_eq!(dbg, again, "a debug read had a side effect");
                }
            }
            0x02 => {
                if at + 6 > data.len() {
                    break;
                }
                let addr = (u64::from(u16::from_le_bytes([data[at], data[at + 1]])) * 4)
                    % (RAM_LEN - 4);
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
                    MemAttrs::DEFAULT,
                );
            }
            0x03 => f.ctrl.reset(),
            0x04 => {
                // A round trip must reproduce the register file exactly,
                // whatever half-configured state the stream left behind.
                if let Some(bytes) = snapshot(&f) {
                    let fresh = build();
                    let reader = StateReader::new(&bytes).expect("we just wrote it");
                    let chunk = reader
                        .load("nvme", "nvme.controller", 1, &Migrations::new())
                        .expect("it is in there");
                    fresh
                        .ctrl
                        .load(&mut chunk.reader())
                        .expect("our own snapshot loads");
                    assert_eq!(
                        snapshot(&fresh),
                        Some(bytes),
                        "the controller did not round trip"
                    );
                }
            }
            0x05 => {
                // Bus mastering off and on again: a controller that may not
                // master the bus fetches nothing, and one that may picks up
                // whatever was left queued.
                f.ctrl.set_master(false);
                f.ctrl.run();
                f.ctrl.set_master(true);
                f.ctrl.run();
            }
            0x06 => {
                // Untrusted bytes straight into the chunk decoder. Rejecting is
                // the expected outcome; panicking is never one.
                let mut r = ChunkReader::new(&data[at..]);
                let _ = f.ctrl.load(&mut r);
                at = data.len();
                // And the device is still usable.
                f.ctrl.run();
                let _ = f
                    .space
                    .write(REGS + 0x1000, Width::U32, 1, MemAttrs::DEFAULT);
            }
            _ => {}
        }
    }
});
