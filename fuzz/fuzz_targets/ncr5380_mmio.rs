#![no_main]
//! The NCR 5380's eight registers, as a Macintosh Plus decodes them.
//!
//! `CLAUDE.md` asks for a fuzz target on every MMIO surface, and this one is a
//! good candidate for a reason particular to the part: **the 5380 leaves the
//! SCSI protocol in software**, so every bus signal is a bit a guest can set
//! in any order. `ASSERT ACK` with no `REQ` outstanding, `ASSERT SEL` with two
//! IDs on the data bus and `BSY` still asserted, a Start DMA register written
//! with `DMA MODE` clear, a pseudo-DMA byte taken in the middle of a command
//! phase — a driver reaches those states by mistake and a hostile guest
//! reaches them on purpose, and behind them is a real target with a real
//! medium.
//!
//! The properties are the ones no unit test can assert over an arbitrary
//! sequence:
//!
//! > **Nothing panics and nothing hangs.** However the signals are driven, one
//! > register access is bounded work. The phase machine inside
//! > [`rsemu::dev::scsi`] can move the target several phases in one access —
//! > an empty `DATA IN` becomes `STATUS` becomes `MESSAGE IN` — and a settle
//! > loop that failed to terminate would show up here as a timeout.
//!
//! > **The ranked lock order holds.** A debug build checks it on every
//! > acquisition, and this target drives the one path that nests three locks:
//! > the chip's state, the bus, and the target's own.
//!
//! Three more surfaces come along for free:
//!
//! * **A debug read has no side effects** (`ROADMAP.md` §15, invariant 5).
//!   Every read is made twice with `MemAttrs::debug` and must answer
//!   identically — which is the rule the Reset Parity/Interrupt register, the
//!   Current SCSI Data register and the Input Data Register each break if they
//!   are written wrong.
//! * **A debug write is refused.** Every write here does something to the
//!   cable, so there is no harmless version of one.
//! * **The snapshot loader is a parser on untrusted bytes.** A tail of the
//!   input is handed to `Device::load`, which must reject it or accept it and
//!   never panic, and the chip must still work afterwards.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 aa dd      write the register the offset aa decodes to
//!   0x01 aa         read it, guest and debug, and compare
//!   0x02 nn         nn+1 reads of the data register
//!   0x03 nn         nn+1 writes of the byte nn to the data register
//!   0x04            cold reset
//!   0x05            save, then load what was saved: a round trip
//!   0x06 ...        load the rest of the input as a snapshot chunk
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly-rejected.

use std::sync::Arc;

use libfuzzer_sys::fuzz_target;

use rsemu::core::device::{Device, ResetKind};
use rsemu::core::hosts::HostObjects;
use rsemu::core::props::{Media, Props, Value};
use rsemu::core::space::{AddressSpace, MemAttrs};
use rsemu::core::state::{MachineShape, Migrations, Sink, StateReader, StateWriter};
use rsemu::core::value::Width;
use rsemu::dev::ncr5380::{CLASS, Ncr5380};
use rsemu::dev::scsi::disk::DiskDevice;

/// The Macintosh Plus's decode: register selects sixteen bytes apart. The
/// widest decode any board in this tree uses, so it covers the narrow ones.
const STRIDE: u64 = 16;

/// How long the window is, and therefore how far an offset can reach.
const WINDOW: u64 = 8 * STRIDE;

/// How many blocks the target holds. Small: a fuzz run should spend its time in
/// the register file, not in a medium.
const BLOCKS: usize = 8;

/// The chip, the space it answers on, and the drive on its cable.
struct Rig {
    chip: Ncr5380,
    space: AddressSpace,
    /// Kept alive: the bus holds `Arc<dyn Target>` and this owns the medium.
    _disk: DiskDevice,
}

fn build() -> Rig {
    let hosts = Arc::new(HostObjects::new());
    let mut image = vec![0u8; BLOCKS * 512];
    for (n, block) in image.chunks_mut(512).enumerate() {
        block.fill(n as u8);
    }
    let disk = DiskDevice::new(
        &Props::new()
            .with("image", Value::Media(Media::new("hd0", image)))
            .with("bus", Value::Str(String::from("scsi0")))
            .with("id", Value::Uint(0))
            .with_hosts(Arc::clone(&hosts)),
    )
    .expect("a drive");
    let chip = Ncr5380::new(
        &Props::new()
            .with("bus", Value::Str(String::from("scsi0")))
            .with("id", Value::Uint(7))
            .with("stride", Value::Uint(STRIDE))
            .with_hosts(hosts),
    )
    .expect("a controller");
    let space = AddressSpace::new("scsi", 16);
    space
        .topology()
        .map(
            Device::region(&chip, "").expect("the chip has a window"),
            0,
        )
        .expect("the window fits");
    Rig {
        chip,
        space,
        _disk: disk,
    }
}

fn snapshot(chip: &Ncr5380) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("ncr", CLASS.name).ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("ncr", CLASS.name, CLASS.version).ok()?;
        Device::save(chip, &mut chunk).ok()?;
    }
    w.to_vec().ok()
}

fuzz_target!(|data: &[u8]| {
    let rig = build();
    let mut at = 0usize;

    while at < data.len() {
        let op = data[at];
        at += 1;
        match op {
            0x00 => {
                if at + 2 > data.len() {
                    break;
                }
                let offset = u64::from(data[at]) % WINDOW;
                let value = u64::from(data[at + 1]);
                at += 2;
                let _ = rig.space.write(offset, Width::U8, value, MemAttrs::DEFAULT);
                assert!(
                    rig.space
                        .write(offset, Width::U8, value, MemAttrs::DEBUG)
                        .is_err(),
                    "a debug write to {offset:#05x} was accepted"
                );
            }
            0x01 => {
                if at >= data.len() {
                    break;
                }
                let offset = u64::from(data[at]) % WINDOW;
                at += 1;
                let first = rig.space.read(offset, Width::U8, MemAttrs::DEBUG);
                let second = rig.space.read(offset, Width::U8, MemAttrs::DEBUG);
                assert_eq!(
                    first.is_ok(),
                    second.is_ok(),
                    "two debug reads of {offset:#05x} disagreed about legality"
                );
                if let (Ok(a), Ok(b)) = (first, second) {
                    assert_eq!(a, b, "a debug read of {offset:#05x} had a side effect");
                    let live = rig
                        .space
                        .read(offset, Width::U8, MemAttrs::DEFAULT)
                        .expect("a guest read is legal wherever a debug read was");
                    // The two are the same read everywhere except the data
                    // register under a started pseudo-DMA transfer, where the
                    // guest's access *is* the transfer and the debugger's
                    // deliberately hands over the stale latch instead.
                    let data_reg = (offset / STRIDE) & 7;
                    if data_reg != 0 && data_reg != 6 {
                        assert_eq!(
                            live, a,
                            "a debug read of {offset:#05x} answered differently from the guest's"
                        );
                    }
                }
            }
            0x02 => {
                if at >= data.len() {
                    break;
                }
                let count = u32::from(data[at]) + 1;
                at += 1;
                for _ in 0..count {
                    let _ = rig.space.read(0, Width::U8, MemAttrs::DEFAULT);
                }
            }
            0x03 => {
                if at >= data.len() {
                    break;
                }
                let byte = u64::from(data[at]);
                let count = u32::from(data[at]) + 1;
                at += 1;
                for _ in 0..count {
                    let _ = rig.space.write(1, Width::U8, byte, MemAttrs::DEFAULT);
                }
            }
            0x04 => Device::reset(&rig.chip, ResetKind::Cold),
            0x05 => {
                // A round trip has to come back: whatever state the signals
                // have been driven into, saving and reloading it must
                // reproduce the same bytes.
                if let Some(bytes) = snapshot(&rig.chip) {
                    let reader = StateReader::new(&bytes).expect("what we just wrote");
                    let chunk = reader
                        .load("ncr", CLASS.name, CLASS.version, &Migrations::new())
                        .expect("the chunk we just wrote");
                    Device::load(&rig.chip, &mut chunk.reader()).expect("our own snapshot loads");
                    assert_eq!(
                        snapshot(&rig.chip).as_deref(),
                        Some(&bytes[..]),
                        "a save/load round trip changed the chip's state"
                    );
                }
            }
            0x06 => {
                // The loader on bytes nobody wrote. Rejecting is the expected
                // outcome; panicking is not, and neither is being unusable
                // afterwards.
                let mut shape = MachineShape::new();
                if shape.add_device("ncr", CLASS.name).is_ok() {
                    let mut w = StateWriter::new(shape);
                    if let Ok(mut chunk) = w.chunk("ncr", CLASS.name, CLASS.version) {
                        let _ = chunk.write_bytes(&data[at..]);
                    }
                    if let Ok(bytes) = w.to_vec()
                        && let Ok(reader) = StateReader::new(&bytes)
                        && let Ok(chunk) =
                            reader.load("ncr", CLASS.name, CLASS.version, &Migrations::new())
                    {
                        let _ = Device::load(&rig.chip, &mut chunk.reader());
                    }
                }
                at = data.len();
                let _ = rig.space.read(0, Width::U8, MemAttrs::DEFAULT);
            }
            _ => {}
        }
    }

    // Whatever happened, the chip is still a chip: it answers a read and it
    // takes a reset.
    let _ = rig.space.read(0, Width::U8, MemAttrs::DEFAULT);
    Device::reset(&rig.chip, ResetKind::Cold);
});
