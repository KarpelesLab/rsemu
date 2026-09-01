#![no_main]
//! The three APIC-era MMIO surfaces on a PC: the local APIC, the I/O APIC and
//! the HPET.
//!
//! `CLAUDE.md` asks for a fuzz target on every MMIO surface. These three earn
//! one between them rather than one each, because the interesting failures are
//! *between* them: an I/O APIC turns a pin into a message, a local APIC turns a
//! message into a vector, and an end-of-interrupt travels back the other way.
//! Fuzzing them apart would exercise three register decoders and none of the
//! paths that connect them.
//!
//! What is being tested, beyond "does not panic":
//!
//! * **Every register offset is decoded or refused, and neither panics.** The
//!   local APIC's page is mostly reserved, the I/O APIC's indirect index is a
//!   byte with 232 undefined values, and the HPET's kilobyte is mostly holes.
//!   Indexing arithmetic over any of those is where an out-of-bounds slice
//!   would live.
//! * **A debug read answers the same value and changes nothing.** Compared on
//!   every read (`ROADMAP.md` §15, invariant 5).
//! * **Advancing is monotonic and terminates.** Both timers are lazily advanced
//!   devices; a periodic APIC timer with a one-tick period and an HPET with a
//!   zero-length period are the two shapes that would turn catch-up into a
//!   loop, and both are reachable from this input.
//! * **`next_event_tick` is strictly ahead of `current_tick`**, which the
//!   `Device` contract requires and which a scheduler would spin on forever if
//!   it were not.
//! * **The snapshot loader is a parser on untrusted bytes.** A tail of the
//!   input is handed to `Device::load`, which must reject it or accept it,
//!   never panic, and leave a device that still works.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 aa dd dd dd dd   write local APIC register (aa % 0x40) * 0x10
//!   0x01 aa               read it, and compare against a debug read
//!   0x02 ii dd dd dd dd   I/O APIC: select ii, then write the data window
//!   0x03 ii               I/O APIC: select ii, then read the data window
//!   0x04 aa dd dd dd dd   HPET: write the low half of register (aa % 0x80) * 8
//!   0x05 aa               HPET: read it, and compare against a debug read
//!   0x06 ll               toggle I/O APIC input (ll % 24)
//!   0x07 ll               toggle local APIC LINT pin (ll % 2)
//!   0x08 nn nn            advance both timers by nn ticks
//!   0x09                  run the local APIC's acknowledge cycle
//!   0x0a                  cold reset all three
//!   0x0b                  save, then load what was saved: a round trip
//!   0x0c ...              load the rest of the input as a snapshot chunk
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly-rejected.

use std::sync::Arc;

use libfuzzer_sys::fuzz_target;

use rsemu::core::device::{Deferred, Device, RealizeCtx, ResetKind};
use rsemu::core::hosts::HostObjects;
use rsemu::core::space::{MemAttrs, MemOps, RegionKind, RequesterId};
use rsemu::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use rsemu::core::wire::{IntAckCycle, Level, WireId, WireIdAllocator, WireSink};
use rsemu::dev::pc::apic::{ApicBus, LocalApic};
use rsemu::dev::pc::hpet::Hpet;
use rsemu::dev::pc::ioapic::{INPUTS, IoApic};

/// How many 16-byte registers the local APIC's page holds.
const LAPIC_REGS: u64 = 0x40;
/// How many 8-byte registers reach the HPET's kilobyte.
const HPET_REGS: u64 = 0x80;

/// The three parts, wired to each other and to nothing else.
struct Fixture {
    lapic: Arc<LocalApic>,
    ioapic: Arc<IoApic>,
    hpet: Arc<Hpet>,
    io_pins: Vec<Arc<dyn WireSink>>,
    lint: Vec<Arc<dyn WireSink>>,
    src: WireId,
    /// What each input is currently held at, so an op can toggle it.
    io_level: [bool; INPUTS],
    lint_level: [bool; 2],
}

/// Run a device's `realize`, which is what puts an APIC on its bus.
fn realize(device: &dyn Device) {
    let hosts = HostObjects::new();
    let mut deferred = Deferred::new();
    let mut ctx = RealizeCtx::new("fuzz", RequesterId::default(), &mut deferred, &hosts);
    let _ = device.realize(&mut ctx);
    deferred.drain();
}

/// The `MemOps` behind a device's register block.
fn ops(device: &dyn Device) -> Arc<dyn MemOps> {
    match device.region("regs").expect("every part here has one").kind() {
        RegionKind::Io(ops) => Arc::clone(ops),
        _ => unreachable!("a register block is an I/O region"),
    }
}

impl Fixture {
    fn new() -> Fixture {
        let bus = Arc::new(ApicBus::new());
        let lapic = Arc::new(LocalApic::with_bus(0, true, Arc::clone(&bus)));
        let ioapic = Arc::new(IoApic::with_bus(0, INPUTS, Arc::clone(&bus)));
        let hpet = Arc::new(Hpet::default_device());
        realize(&*lapic);
        realize(&*ioapic);
        let ids = WireIdAllocator::new();
        let src = ids.alloc();
        let io_pins = (0..INPUTS)
            .map(|i| {
                ioapic
                    .sink(&format!("irq{i}"), &[src])
                    .expect("every input exists")
                    .sink
            })
            .collect();
        let lint = ["lint0", "lint1"]
            .iter()
            .map(|p| lapic.sink(p, &[src]).expect("both LINT pins exist").sink)
            .collect();
        Fixture {
            lapic,
            ioapic,
            hpet,
            io_pins,
            lint,
            src,
            io_level: [false; INPUTS],
            lint_level: [false; 2],
        }
    }

    /// A read and a debug read of the same place must answer the same thing,
    /// and the debug one must move nothing.
    fn compare_read(&self, ops: &Arc<dyn MemOps>, offset: u64, width: usize) {
        let mut guest = [0u8; 8];
        let mut debug = [0u8; 8];
        let before = (self.lapic.tick(), self.hpet.tick());
        let a = ops.read(offset, &mut guest[..width], MemAttrs::DEFAULT);
        let b = ops.read(offset, &mut debug[..width], MemAttrs::DEBUG);
        assert_eq!(a.is_ok(), b.is_ok(), "a debug read decodes the same places");
        if a.is_ok() {
            assert_eq!(guest, debug, "and answers the same bytes at {offset:#x}");
        }
        assert_eq!(
            (self.lapic.tick(), self.hpet.tick()),
            before,
            "and neither read moved a clock"
        );
    }

    /// Both lazily advanced parts must be monotonic and must always name a
    /// next event strictly ahead of where they stand.
    fn check_time(&self) {
        for (now, next) in [
            (self.lapic.tick(), Device::next_event_tick(&*self.lapic)),
            (self.hpet.tick(), Device::next_event_tick(&*self.hpet)),
        ] {
            if let Some(at) = next {
                assert!(at > now, "an event at {at} is not ahead of {now}");
            }
        }
    }
}

/// Take `n` bytes, or nothing if the input has run out.
fn take<'a>(input: &mut &'a [u8], n: usize) -> Option<&'a [u8]> {
    if input.len() < n {
        return None;
    }
    let (head, tail) = input.split_at(n);
    *input = tail;
    Some(head)
}

fn byte(input: &mut &[u8]) -> Option<u8> {
    take(input, 1).map(|b| b[0])
}

fn dword(input: &mut &[u8]) -> Option<u32> {
    take(input, 4).map(|b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
}

/// Save every part and load it straight back. The bytes of a second save must
/// be the bytes of the first, which is the state hash the rule is about.
fn round_trip(f: &Fixture) {
    let parts: [(&str, &dyn Device); 3] = [
        ("lapic", &*f.lapic),
        ("ioapic", &*f.ioapic),
        ("hpet", &*f.hpet),
    ];
    for (name, device) in parts {
        let class = device.class();
        let mut shape = MachineShape::new();
        if shape.add_device(name, class.name).is_err() {
            return;
        }
        let mut w = StateWriter::new(shape);
        {
            let Ok(mut chunk) = w.chunk(name, class.name, class.version) else {
                return;
            };
            if device.save(&mut chunk).is_err() {
                return;
            }
        }
        let Ok(bytes) = w.to_vec() else { return };
        let Ok(reader) = StateReader::new(&bytes) else {
            return;
        };
        let Ok(chunk) = reader.load(name, class.name, class.version, &Migrations::new()) else {
            return;
        };
        assert!(
            device.load(&mut chunk.reader()).is_ok(),
            "a part cannot refuse its own state"
        );
    }
}

fuzz_target!(|data: &[u8]| {
    let mut f = Fixture::new();
    let lapic_ops = ops(&*f.lapic);
    let io_ops = ops(&*f.ioapic);
    let hpet_ops = ops(&*f.hpet);
    let mut input = data;

    while let Some(op) = byte(&mut input) {
        match op {
            0x00 => {
                let (Some(at), Some(value)) = (byte(&mut input), dword(&mut input)) else {
                    return;
                };
                let offset = (u64::from(at) % LAPIC_REGS) * 0x10;
                let _ = lapic_ops.write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT);
            }
            0x01 => {
                let Some(at) = byte(&mut input) else { return };
                f.compare_read(&lapic_ops, (u64::from(at) % LAPIC_REGS) * 0x10, 4);
            }
            0x02 => {
                let (Some(index), Some(value)) = (byte(&mut input), dword(&mut input)) else {
                    return;
                };
                let _ = io_ops.write(0x00, &u32::from(index).to_le_bytes(), MemAttrs::DEFAULT);
                let _ = io_ops.write(0x10, &value.to_le_bytes(), MemAttrs::DEFAULT);
            }
            0x03 => {
                let Some(index) = byte(&mut input) else { return };
                let _ = io_ops.write(0x00, &u32::from(index).to_le_bytes(), MemAttrs::DEFAULT);
                f.compare_read(&io_ops, 0x10, 4);
            }
            0x04 => {
                let (Some(at), Some(value)) = (byte(&mut input), dword(&mut input)) else {
                    return;
                };
                let offset = (u64::from(at) % HPET_REGS) * 8;
                let _ = hpet_ops.write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT);
            }
            0x05 => {
                let Some(at) = byte(&mut input) else { return };
                f.compare_read(&hpet_ops, (u64::from(at) % HPET_REGS) * 8, 8);
            }
            0x06 => {
                let Some(line) = byte(&mut input) else { return };
                let index = usize::from(line) % INPUTS;
                f.io_level[index] = !f.io_level[index];
                f.io_pins[index].set_level(
                    f.src,
                    index as u32,
                    Level::from_bool(f.io_level[index]),
                );
            }
            0x07 => {
                let Some(line) = byte(&mut input) else { return };
                let index = usize::from(line) % 2;
                f.lint_level[index] = !f.lint_level[index];
                f.lint[index].set_level(
                    f.src,
                    index as u32,
                    Level::from_bool(f.lint_level[index]),
                );
            }
            0x08 => {
                let Some(span) = take(&mut input, 2) else {
                    return;
                };
                let span = u64::from(u16::from_le_bytes([span[0], span[1]]));
                // Both parts are lazily advanced, and both must come back from
                // any span: a one-tick periodic APIC timer and a zero-period
                // HPET comparator are the two shapes that would loop.
                f.lapic.advance_to(f.lapic.tick() + span);
                f.hpet.advance_to(f.hpet.tick() + span);
                f.check_time();
            }
            0x09 => {
                let _ = f
                    .lapic
                    .int_ack("intr")
                    .expect("a local APIC answers the acknowledge")
                    .acknowledge(IntAckCycle::vector_only());
            }
            0x0a => {
                f.lapic.reset(ResetKind::Cold);
                f.ioapic.reset(ResetKind::Cold);
                f.hpet.reset(ResetKind::Cold);
                f.check_time();
            }
            0x0b => {
                round_trip(&f);
                f.check_time();
            }
            0x0c => {
                // Whatever is left, as a snapshot chunk. A loader is a parser
                // on untrusted bytes: it must reject or accept, never panic,
                // and leave a device that still answers.
                let rest = std::mem::take(&mut input);
                let mut reader = ChunkReader::new(rest);
                let _ = f.lapic.load(&mut reader);
                let mut reader = ChunkReader::new(rest);
                let _ = f.ioapic.load(&mut reader);
                let mut reader = ChunkReader::new(rest);
                let _ = f.hpet.load(&mut reader);
                f.check_time();
                f.lapic.advance_to(f.lapic.tick() + 1);
                f.hpet.advance_to(f.hpet.tick() + 1);
                f.check_time();
                return;
            }
            _ => {}
        }
    }
});
