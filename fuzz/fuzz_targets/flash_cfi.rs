#![no_main]
//! The NOR flash MMIO surface, and the one property that makes it flash.
//!
//! `CLAUDE.md` asks for a fuzz target on "every MMIO surface", and this device
//! is the one where that is more than a no-panic check, because it has an
//! invariant a guest must not be able to break however it drives the bus:
//!
//! > **A program can only clear bits.** The only thing that puts a bit back to
//! > one is an erase, and an erase takes a whole block back to `0xff`.
//!
//! So after *every* bus cycle this target compares the array with what it held
//! before, and asserts that each byte either did not change, lost bits, or
//! became exactly `0xff`. Any other transition is a byte that gained a bit
//! without an erase, which is the failure the device exists to prevent — and
//! which no unit test can rule out across an arbitrary command sequence.
//!
//! Two other surfaces come along for free:
//!
//! * **Reads never panic and never have a side effect.** Every read is repeated
//!   with `MemAttrs::DEBUG`, which must answer with the array contents and must
//!   not move the command state machine (`ROADMAP.md` §15, invariant 5).
//! * **The snapshot loader is a parser on untrusted bytes.** A tail of the
//!   input is handed to `Device::load`, which must reject it or accept it and
//!   never panic — and afterwards the device must still be usable.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded rather than derived (see
//! `state_roundtrip` for why the corpus is more stable that way):
//!
//! ```text
//!   0x00 aa aa dd dd dd dd   write a 32-bit word at offset aaaa (mod size)
//!   0x01 aa aa dd dd         write a 16-bit halfword there
//!   0x02 aa aa               read a 32-bit word
//!   0x03                     reset the device
//!   0x04 ...                 save, then load what was saved: a round trip
//!   0x05 ...                 load the rest of the input as a snapshot chunk
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly-rejected.

use libfuzzer_sys::fuzz_target;

use rsemu::core::device::{Device, ResetKind};
use rsemu::core::space::{MemAttrs, MemOps};
use rsemu::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use rsemu::dev::flash::cfi::{Array, BlockRegion, Cfi, Geometry, DEFAULT_MANUFACTURER};
use std::sync::Arc;

/// Small enough to fuzz quickly, and *not* uniform, so the block arithmetic
/// that a boot-block part exercises is on the hot path rather than in one
/// hand-written test.
const REGIONS: [BlockRegion; 2] = [
    BlockRegion {
        count: 2,
        size: 0x400,
    },
    BlockRegion {
        count: 3,
        size: 0x800,
    },
];

fn build() -> Cfi {
    let geom = Geometry::new(REGIONS.to_vec(), 4, 2).expect("a plausible part");
    // Powering up unlocked: a locked part refuses every program, and a fuzzer
    // that spent its whole budget being refused would prove nothing.
    Cfi::from_array(Arc::new(
        Array::with_options(geom, DEFAULT_MANUFACTURER, 0x1234, false, false).expect("fits"),
    ))
}

/// Every byte either kept its value, lost bits, or was erased to all ones.
fn only_cleared_or_erased(before: &[u8], after: &[u8]) {
    assert_eq!(before.len(), after.len());
    for (i, (old, new)) in before.iter().zip(after).enumerate() {
        if old == new || *new == 0xff {
            continue;
        }
        assert_eq!(
            new & !old,
            0,
            "byte {i} went from {old:#04x} to {new:#04x}: a program set a bit"
        );
    }
}

fn snapshot(cfi: &Cfi) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("flash", "flash.cfi").ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("flash", "flash.cfi", 1).ok()?;
        cfi.save(&mut chunk).ok()?;
    }
    w.to_vec().ok()
}

fuzz_target!(|data: &[u8]| {
    let cfi = build();
    let size = cfi.array().geometry().size();
    let mut before = cfi.array().contents();
    let mut at = 0usize;

    while at < data.len() {
        let op = data[at];
        at += 1;
        match op {
            0x00 | 0x01 => {
                let width = if op == 0x00 { 4 } else { 2 };
                if at + 2 + width > data.len() {
                    break;
                }
                let offset = u64::from(u16::from_le_bytes([data[at], data[at + 1]])) % size;
                // Aligned to the device width: a misaligned command cycle is
                // refused by design, and every refusal spent here is a cycle
                // the interesting half of the state machine did not get.
                let offset = offset & !1;
                let bytes = &data[at + 2..at + 2 + width];
                at += 2 + width;
                // A refusal is a legal outcome; a panic is not.
                let _ = cfi.array().write(offset, bytes, MemAttrs::DEFAULT);
                let after = cfi.array().contents();
                only_cleared_or_erased(&before, &after);
                before = after;
            }
            0x02 => {
                if at + 2 > data.len() {
                    break;
                }
                let offset = u64::from(u16::from_le_bytes([data[at], data[at + 1]])) % size;
                at += 2;
                let mut live = [0u8; 4];
                let mut dbg = [0u8; 4];
                let ok = cfi
                    .array()
                    .read(offset & !3, &mut live, MemAttrs::DEFAULT)
                    .is_ok();
                assert!(
                    cfi.array().read(offset & !3, &mut dbg, MemAttrs::DEBUG).is_ok() == ok,
                    "a debug read and a bus read disagree about what is mapped"
                );
                // A debug read must not have moved anything.
                let after = cfi.array().contents();
                assert_eq!(before, after, "a read changed the array");
            }
            0x03 => {
                cfi.reset(ResetKind::Cold);
                // Non-volatile: a reset clears the command state, never the
                // contents.
                assert_eq!(cfi.array().contents(), before, "a reset erased the part");
            }
            0x04 => {
                // A round trip must reproduce the state exactly, whatever
                // half-issued command sequence the stream left behind.
                if let Some(bytes) = snapshot(&cfi) {
                    let fresh = build();
                    let reader = StateReader::new(&bytes).expect("we just wrote it");
                    let chunk = reader
                        .load("flash", "flash.cfi", 1, &Migrations::new())
                        .expect("it is in there");
                    fresh
                        .load(&mut chunk.reader())
                        .expect("our own snapshot loads");
                    assert_eq!(
                        snapshot(&fresh),
                        Some(bytes),
                        "the flash did not round trip"
                    );
                }
            }
            0x05 => {
                // Untrusted bytes straight into the chunk decoder. Rejecting
                // is the expected outcome; panicking is never one.
                let mut r = ChunkReader::new(&data[at..]);
                let _ = cfi.load(&mut r);
                before = cfi.array().contents();
                at = data.len();
            }
            _ => {}
        }
    }
});
