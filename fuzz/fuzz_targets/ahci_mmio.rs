#![no_main]
//! The AHCI MMIO surface, and the command lists, command tables and PRD tables
//! a guest builds behind it.
//!
//! `CLAUDE.md` asks for a fuzz target on "every MMIO surface", and this device
//! wants one more than most, for the same reason `nvme_mmio` does: its register
//! block is the smaller half of its attack surface. The adapter is a **bus
//! master that walks structures the guest wrote** — a 32-byte command header
//! whose base the guest chose, a command table whose base the header chose, a
//! command FIS whose length the header chose, and a Physical Region Descriptor
//! Table of up to 65,535 entries, every one of which names an address and a
//! length the guest picked. A driver bug points one of them at the wrong place;
//! a hostile guest points it at the adapter.
//!
//! So this target pokes whatever the fuzzer says into the registers and into
//! guest RAM, and the properties being checked are the ones no unit test can
//! cover across an arbitrary structure:
//!
//! > **Every walk terminates, and none of them panics.** A `PxCI` write returns,
//! > because every loop over a guest pointer is bounded — the PRD table, the
//! > data phase, and the number of commands one entry into the engine may run.
//!
//! An unbounded walk shows up here as a timeout, which is what `cargo fuzz`'s
//! `-timeout` reports.
//!
//! Four more surfaces come along for free, and two of them are specific to this
//! device:
//!
//! * **The register block is mapped in the same address space the adapter
//!   masters**, at an address a PRD can name. So the fuzzer can and will aim a
//!   data block at `PxCI` — or at `PxCLB`, moving the command list out from
//!   under a command that is running — which re-enters the write handler from
//!   inside itself. The engine is iterative rather than recursive, and a stack
//!   overflow here would say otherwise.
//! * **The drive is a real `AtaDisk` on a real medium**, so a taskfile the
//!   fuzzer assembled goes through the whole ATA command set. A command that
//!   leaves the drive half way through a data phase and an adapter that walks
//!   away from it would show up as the *next* command reading somebody else's
//!   sector.
//! * **A debug read has no side effects**, and **a debug write is refused**:
//!   writing `PxCI` runs a command, so there is no harmless subset.
//! * **The snapshot loader is a parser on untrusted bytes.** A tail of the input
//!   is handed to `Hba::load`, which must reject it or accept it and never
//!   panic, and the adapter must still be usable afterwards.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 aa dd dd dd dd   write a register: aa picks one of the generic host
//!                         control registers or one of port 0's or port 1's
//!   0x01 aa               read it, and compare against a debug read
//!   0x02 aa aa dd dd dd dd  poke a dword into guest RAM at (aaaa mod size)
//!   0x03                  reset
//!   0x04                  save, then load what was saved: a round trip
//!   0x05                  bus master enable off, then on
//!   0x06 ...              load the rest of the input as a snapshot chunk
//!   0x07                  write PxCI directly, which is the doorbell
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly-rejected.

use libfuzzer_sys::fuzz_target;

use rsemu::core::space::{
    AddressSpace, MemAttrs, MemOps, RamStore, Region, RequesterId, UnassignedPolicy,
};
use rsemu::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use rsemu::core::sync::{LockRank, Mutex};
use rsemu::core::value::Width;
use rsemu::core::wire::{Level, Wire, WireId, WireIdAllocator, WireSink, WireSource};
use rsemu::dev::ahci::hba::REGISTER_LEN;
use rsemu::dev::ahci::{Hba, MAX_PORTS};
use rsemu::dev::ata::bays::Bay;
use rsemu::dev::ata::disk::default_geometry;
use rsemu::dev::ata::{AtaDisk, Identity, Position};
use rsemu::dev::medium::Medium;
use std::sync::Arc;

/// Where guest RAM starts. Not zero, so a null pointer in a command header is a
/// bus fault the adapter has to survive rather than a plausible read.
const RAM_BASE: u64 = 0x1000;
/// How much of it there is.
const RAM_LEN: u64 = 0x20_0000;
/// Where the register block is mapped — inside the same space, so a PRD can
/// point at it.
const REGS: u64 = 0x1000_0000;
/// How many sectors the drive holds.
const SECTORS: u64 = 128;

/// A sink that only remembers the level, so the interrupt has somewhere to go.
#[derive(Debug)]
struct Pin(Mutex<Level>);

impl WireSink for Pin {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        *self.0.lock() = level;
    }
}

struct Fixture {
    hba: Arc<Hba>,
    space: Arc<AddressSpace>,
}

fn build() -> Fixture {
    let store: Arc<dyn Medium> = Arc::new(RamStore::new(SECTORS * 512));
    let mut id = Identity::new(SECTORS, default_geometry(SECTORS), true, 16).expect("an identity");
    id.dma = true;
    let drive = Arc::new(
        AtaDisk::with_medium(id, Position::Device0, store).expect("the medium fits"),
    );
    // Two bays, one of them empty: a port with nothing on it is a path the
    // adapter has to survive as much as a port with a drive.
    let occupied = Arc::new(Bay::new());
    occupied.fit(drive).expect("an empty bay");
    let hba = Arc::new(Hba::new(vec![
        (String::from("sata0"), occupied),
        (String::from("sata1"), Arc::new(Bay::new())),
    ]));

    let space = Arc::new(AddressSpace::new("mem", 32).with_unassigned(UnassignedPolicy::ONES));
    {
        let mut topo = space.topology();
        topo.map(
            Region::ram("ram", Arc::new(RamStore::new(RAM_LEN))),
            RAM_BASE,
        )
        .expect("the map fits");
        topo.map(
            Region::io("ahci.abar", REGISTER_LEN, Arc::clone(&hba) as Arc<dyn MemOps>),
            REGS,
        )
        .expect("the map fits");
    }
    hba.attach_space(&space, RequesterId(11));
    hba.reset();
    hba.set_master(true);

    let ids = WireIdAllocator::new();
    let id = ids.alloc();
    let pin: Arc<dyn WireSink> = Arc::new(Pin(Mutex::with_rank(LockRank::LEAF, Level::Low)));
    let wire = Wire::builder().source(id).sink(pin, 0).build_shared();
    hba.connect_irq(WireSource::new(wire, id));

    Fixture { hba, space }
}

/// Which register byte `sel` names: the generic host control block below `0x40`,
/// and one of the first two ports' register banks above it.
fn offset_of(sel: u8) -> u64 {
    if sel < 0x40 {
        u64::from(sel % 0x0c) * 4
    } else {
        let index = u64::from((sel >> 5) & 1).min(MAX_PORTS as u64 - 1);
        0x100 + index * 0x80 + u64::from(sel % 0x20) * 4
    }
}

fn snapshot(f: &Fixture) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("ahci", "ahci.hba").ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("ahci", "ahci.hba", 1).ok()?;
        f.hba.save(&mut chunk).ok()?;
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
                // **The property.** Whatever the command list in RAM says, this
                // returns: every walk over a guest-built structure is bounded,
                // and a transfer aimed at these very registers re-enters
                // iteratively rather than recursively.
                let _ = f.space.write(
                    REGS + offset,
                    Width::U32,
                    u64::from(value),
                    MemAttrs::DEFAULT,
                );
                // A debug write must never be accepted: `PxCI` runs a command,
                // `PxCMD.ST` stops the engine, and `PxIS` is write-1-to-clear.
                assert!(
                    f.space
                        .write(REGS + offset, Width::U32, u64::from(value), MemAttrs::DEBUG)
                        .is_err(),
                    "a debug write to the AHCI register block must be refused"
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
            0x03 => f.hba.reset(),
            0x04 => {
                // A round trip must reproduce the register file exactly,
                // whatever half-configured state the stream left behind.
                if let Some(bytes) = snapshot(&f) {
                    let fresh = build();
                    let reader = StateReader::new(&bytes).expect("we just wrote it");
                    let chunk = reader
                        .load("ahci", "ahci.hba", 1, &Migrations::new())
                        .expect("it is in there");
                    fresh
                        .hba
                        .load(&mut chunk.reader())
                        .expect("our own snapshot loads");
                    assert_eq!(snapshot(&fresh), Some(bytes), "the adapter did not round trip");
                }
            }
            0x05 => {
                // Bus mastering off and on again: a function that may not master
                // the bus fetches nothing, and one that may picks up whatever
                // was left in `PxCI`.
                f.hba.set_master(false);
                f.hba.run();
                f.hba.set_master(true);
                f.hba.run();
            }
            0x06 => {
                // Untrusted bytes straight into the chunk decoder. Rejecting is
                // the expected outcome; panicking is never one.
                let mut r = ChunkReader::new(&data[at..]);
                let _ = f.hba.load(&mut r);
                at = data.len();
                // And the device is still usable.
                f.hba.run();
                let _ = f
                    .space
                    .write(REGS + 0x138, Width::U32, 0xffff_ffff, MemAttrs::DEFAULT);
            }
            0x07 => {
                // The doorbell on its own, so a corpus entry that has built a
                // command list does not need six more bytes to ring it.
                let _ = f
                    .space
                    .write(REGS + 0x138, Width::U32, 0xffff_ffff, MemAttrs::DEFAULT);
            }
            _ => {}
        }
    }
});
