#![no_main]
//! The virtio-MMIO surface, and the virtqueues a guest builds behind it.
//!
//! `CLAUDE.md` asks for a fuzz target on "every MMIO surface", and this device
//! earns one for the reason `nvme_mmio` does: the register block is the smaller
//! half of the attack surface. A virtio device is a **bus master that walks
//! structures the guest wrote** — a descriptor table, an available ring, a
//! chain that may loop or point anywhere, an indirect table, a used ring whose
//! base the guest also chose. Two defects this target would have found were
//! live in this tree until the disk moved onto the `Medium` seam:
//!
//! * a descriptor length is a `u32` and a chain holds up to a queue's worth of
//!   them, and the block device staged the whole claimed payload before
//!   checking it against the disk — so a chain claiming four gigabytes was a
//!   four-gigabyte allocation;
//! * the register block is mapped in the space the device masters, so a
//!   writable descriptor aimed at `QueueNotify` re-entered the engine from
//!   inside a transfer, and the engine was recursive. Four zero bytes of disk
//!   data are a notify of queue 0, which is what a blank sector holds.
//!
//! So the properties being checked are the ones no unit test covers across an
//! arbitrary structure:
//!
//! > **Every walk terminates, nothing panics, and no allocation is larger than
//! > the guest's own disk.** An unbounded walk shows up here as a timeout,
//! > which is what `cargo fuzz`'s `-timeout` reports; an unbounded allocation
//! > shows up as an OOM under `-rss_limit_mb`.
//!
//! Two more surfaces come along for free:
//!
//! * **A debug read has no side effects**, and **a debug write is refused** —
//!   writing `QueueNotify` performs I/O and writing `Status` resets the device,
//!   so there is no harmless subset to allow.
//! * **The snapshot loader is a parser on untrusted bytes.** A tail of the
//!   input goes to `load`, which must reject it or accept it and never panic,
//!   and the device must still be usable afterwards.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why a hand-decoded corpus is more stable):
//!
//! ```text
//!   0x00 aa dd dd dd dd   write a register: aa < 0x80 picks a transport
//!                         register, aa >= 0x80 picks a configuration byte
//!   0x01 aa               read it, and compare against a debug read
//!   0x02 aa aa dd dd dd dd  poke a dword into guest RAM at (aaaa mod size)
//!   0x03                  reset
//!   0x04                  save, then load what was saved: a round trip
//!   0x05 ...              load the rest of the input as a snapshot chunk
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly-rejected.
//!
//! # Provenance
//!
//! *Virtual I/O Device (VIRTIO) Version 1.2*, OASIS Standard, §4.2.2 for the
//! register offsets. No driver source of any licence was opened — `ROADMAP.md`
//! §1 names Linux's virtio drivers specifically.

use libfuzzer_sys::fuzz_target;

use rsemu::core::device::{Device, ResetKind};
use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region, RequesterId, UnassignedPolicy};
use rsemu::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use rsemu::core::sync::{LockRank, Mutex};
use rsemu::core::value::Width;
use rsemu::core::wire::{Level, Wire, WireId, WireIdAllocator, WireSink, WireSource};
use rsemu::dev::medium::Medium;
use rsemu::dev::virtio::{self, VirtioBlk, VirtioMmio};
use std::sync::Arc;

/// Where guest RAM starts. Not zero, so a null descriptor address is a bus
/// fault the device has to survive rather than a plausible read.
const RAM_BASE: u64 = 0x1000;
/// How much of it there is.
const RAM_LEN: u64 = 0x20_0000;
/// Where the register block is mapped — inside the same space, so a descriptor
/// can point at it.
const REGS: u64 = 0x1000_0000;
/// How big the disk is.
const DISK_LEN: u64 = 64 * 1024;

/// A sink that only remembers the level, so the interrupt has somewhere to go.
#[derive(Debug)]
struct Pin(Mutex<Level>);

impl WireSink for Pin {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        *self.0.lock() = level;
    }
}

struct Fixture {
    device: VirtioMmio,
    space: Arc<AddressSpace>,
}

fn build() -> Fixture {
    let store: Arc<dyn Medium> = Arc::new(RamStore::new(DISK_LEN));
    let blk = VirtioBlk::new(store, String::from("rsemu-fuzz"), false).expect("whole sectors");
    let device = VirtioMmio::new(
        Arc::new(blk) as Arc<dyn virtio::Backend>,
        &virtio::BLK_CLASS,
    );

    let space = Arc::new(AddressSpace::new("mem", 64).with_unassigned(UnassignedPolicy::ONES));
    {
        let mut topo = space.topology();
        topo.map(
            Region::ram("ram", Arc::new(RamStore::new(RAM_LEN))),
            RAM_BASE,
        )
        .expect("the map fits");
        topo.map(device.region("").expect("a register window"), REGS)
            .expect("the map fits");
    }
    device.attach_space(Arc::clone(&space), RequesterId(7));

    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let pin: Arc<dyn WireSink> = Arc::new(Pin(Mutex::with_rank(LockRank::LEAF, Level::Low)));
    let wire = Wire::builder().source(id).sink(pin, 0).build_shared();
    device
        .connect("irq", WireSource::new(wire, id))
        .expect("the one pin");

    Fixture { device, space }
}

/// Which byte of the window `sel` names: a transport register below `0x80`,
/// a configuration-space byte above it (§4.2.2).
fn offset_of(sel: u8) -> u64 {
    if sel < 0x80 {
        u64::from(sel % 0x40) * 4
    } else {
        0x100 + u64::from(sel % 0x20)
    }
}

fn snapshot(f: &Fixture) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("vio", virtio::BLK_CLASS_NAME).ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("vio", virtio::BLK_CLASS_NAME, 1).ok()?;
        f.device.save(&mut chunk).ok()?;
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
                let value =
                    u32::from_le_bytes([data[at + 1], data[at + 2], data[at + 3], data[at + 4]]);
                at += 5;
                // **The property.** Whatever the rings in RAM say, this
                // returns: every walk over a guest-built structure is bounded
                // by the queue size, every transfer is staged in fixed-size
                // chunks, and a chain aimed at these very registers re-enters
                // the engine iteratively rather than recursively.
                let _ = f.space.write(
                    REGS + offset,
                    Width::U32,
                    u64::from(value),
                    MemAttrs::DEFAULT,
                );
                // A debug write must never be accepted: `QueueNotify` performs
                // I/O and `Status` resets the device.
                assert!(
                    f.space
                        .write(REGS + offset, Width::U32, u64::from(value), MemAttrs::DEBUG)
                        .is_err(),
                    "a debug write to the virtio register block must be refused"
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
                let addr =
                    (u64::from(u16::from_le_bytes([data[at], data[at + 1]])) * 4) % (RAM_LEN - 4);
                let value =
                    u32::from_le_bytes([data[at + 2], data[at + 3], data[at + 4], data[at + 5]]);
                at += 6;
                let _ = f.space.write(
                    RAM_BASE + addr,
                    Width::U32,
                    u64::from(value),
                    MemAttrs::DEFAULT,
                );
            }
            0x03 => f.device.reset(ResetKind::Cold),
            0x04 => {
                // A round trip must reproduce the register file exactly,
                // whatever half-configured state the stream left behind.
                if let Some(bytes) = snapshot(&f) {
                    let fresh = build();
                    let reader = StateReader::new(&bytes).expect("we just wrote it");
                    let chunk = reader
                        .load("vio", virtio::BLK_CLASS_NAME, 1, &Migrations::new())
                        .expect("it is in there");
                    fresh
                        .device
                        .load(&mut chunk.reader())
                        .expect("our own snapshot loads");
                    assert_eq!(
                        snapshot(&fresh),
                        Some(bytes),
                        "the transport did not round trip"
                    );
                }
            }
            0x05 => {
                // Untrusted bytes straight into the chunk decoder. Rejecting is
                // the expected outcome; panicking is never one.
                let mut r = ChunkReader::new(&data[at..]);
                let _ = f.device.load(&mut r);
                at = data.len();
                // And the device is still usable.
                let _ = f
                    .space
                    .write(REGS + 0x050, Width::U32, 0, MemAttrs::DEFAULT);
            }
            _ => {}
        }
    }
});
