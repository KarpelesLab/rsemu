#![no_main]
//! The SWIM's sixteen-register window, both register sets and the switch
//! between them.
//!
//! `CLAUDE.md` asks for a fuzz target on every MMIO surface, and this one is
//! two surfaces behind one aperture. The chip powers up answering the IWM's
//! sixteen **soft switches**, where every access — read or write — moves an
//! internal line; four consecutive writes to the mode register with bit 6
//! going `1, 0, 1, 1` hand the same sixteen addresses to the **ISM** register
//! set instead, which has a FIFO, a two-byte auto-incrementing parameter RAM
//! pointer, an error register that a read clears, and four phase lines that
//! address the drive's own register file and can step a head or eject a disk.
//! Nothing in that sentence is a pure register.
//!
//! So this target pokes whatever the fuzzer says at the window, advances the
//! medium under the head, and asserts what has to hold for *any* sequence:
//!
//! > **Nothing panics, and the chip is still a chip afterwards.** However the
//! > registers have been driven, a read answers, a reset takes, and the head
//! > advances.
//!
//! Three properties ride along:
//!
//! * **A debug read has no side effects.** Every read is made twice with
//!   `MemAttrs::DEBUG` and must answer identically both times. An ISM register
//!   file is where this rule is hardest to keep: reading the data register
//!   pops the FIFO, reading the error register clears it, reading the
//!   parameter RAM advances its counter, and reading the handshake register
//!   tells the mechanism which head to use (`ROADMAP.md` §15, invariant 5).
//! * **A debug write is refused**, because every address in both register sets
//!   either moves a soft switch or loads a register.
//! * **The snapshot loader is a parser on untrusted bytes**, and the round
//!   trip of a state the fuzzer drove the chip into must reproduce itself.
//!
//! # Input encoding
//!
//! A stream of one-byte opcodes, hand-decoded so that a mutated corpus stays
//! productive:
//!
//! ```text
//!   0x00 rr dd      write register (rr mod 16)
//!   0x01 rr         read it, guest and debug, and compare
//!   0x02 nn         advance nn*256 MFM cells
//!   0x03            ask for ISM mode the way the document says
//!   0x04            cold reset
//!   0x05            put a 1.44 MB disk in, or take it out
//!   0x06            save, then load what was saved: a round trip
//!   0x07 ...        load the rest of the input as a snapshot chunk
//! ```

use std::sync::Arc;

use libfuzzer_sys::fuzz_target;

use rsemu::core::device::{Device, ResetKind};
use rsemu::core::space::{AddressSpace, MemAttrs};
use rsemu::core::state::{MachineShape, Migrations, Sink, StateReader, StateWriter};
use rsemu::core::value::Width;
use rsemu::dev::mac::disk::{Disk, Reader};
use rsemu::dev::mac::mfm;
use rsemu::dev::mac::swim::{REGISTER_SPAN, SWIM_CLASS, Swim};

/// Where the chip is mapped: the Macintosh's own window, so an offset bug
/// shows up as a wrong address rather than as zero.
const BASE: u64 = 0xC0_0000;

/// How far apart two of the sixteen registers are — the board puts the selects
/// on A9-A12.
const STRIDE: u64 = REGISTER_SPAN / 16;

fn snapshot(swim: &Swim) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("swim", SWIM_CLASS.name).ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("swim", SWIM_CLASS.name, SWIM_CLASS.version).ok()?;
        swim.save(&mut chunk).ok()?;
    }
    w.to_vec().ok()
}

/// A 1.44 MB image of numbered blocks. Nobody's disk, built on the spot.
fn image() -> Vec<u8> {
    let mut image = vec![0u8; mfm::BYTES];
    for (block, chunk) in image.chunks_mut(512).enumerate() {
        chunk[..4].copy_from_slice(&(block as u32).to_be_bytes());
    }
    image
}

fuzz_target!(|data: &[u8]| {
    let swim = Arc::new(Swim::with_drives([true, true]));
    let space = AddressSpace::new("mem", 24);
    space
        .topology()
        .map(swim.region("").expect("the chip has a register file"), BASE)
        .expect("the window fits");
    let mut at = 0usize;
    // The chip is lazy, so it may only ever be advanced forwards.
    let mut tick = 0u64;
    let mut has_disk = false;

    while at < data.len() {
        let op = data[at];
        at += 1;
        match op {
            0x00 => {
                if at + 2 > data.len() {
                    break;
                }
                let offset = BASE + u64::from(data[at] % 16) * STRIDE;
                let value = u64::from(data[at + 1]);
                at += 2;
                let _ = space.write(offset, Width::U8, value, MemAttrs::DEFAULT);
                assert!(
                    space
                        .write(offset, Width::U8, value, MemAttrs::DEBUG)
                        .is_err(),
                    "a debug write to {offset:#08x} was accepted"
                );
            }
            0x01 => {
                if at >= data.len() {
                    break;
                }
                let offset = BASE + u64::from(data[at] % 16) * STRIDE;
                at += 1;
                let first = space.read(offset, Width::U8, MemAttrs::DEBUG);
                let second = space.read(offset, Width::U8, MemAttrs::DEBUG);
                assert_eq!(
                    first.is_ok(),
                    second.is_ok(),
                    "two debug reads of {offset:#08x} disagreed about legality"
                );
                if let (Ok(a), Ok(b)) = (first, second) {
                    assert_eq!(a, b, "a debug read of {offset:#08x} had a side effect");
                }
                let _ = space.read(offset, Width::U8, MemAttrs::DEFAULT);
            }
            0x02 => {
                if at >= data.len() {
                    break;
                }
                tick += u64::from(data[at]) * 256 + 1;
                at += 1;
                swim.advance_to(tick);
            }
            0x03 => {
                // The documented way in: four consecutive loads of the IWM
                // mode register with bit 6 going 1, 0, 1, 1 (*SWIM Chip User's
                // Reference*, page 12). Worth an opcode of its own because a
                // fuzzer would take a very long time to find it, and every
                // interesting register in the chip is behind it.
                let mode = BASE + 15 * STRIDE;
                let _ = space.read(BASE + 13 * STRIDE, Width::U8, MemAttrs::DEFAULT);
                for value in [0x57u64, 0x17, 0x57, 0x57] {
                    let _ = space.write(mode, Width::U8, value, MemAttrs::DEFAULT);
                }
            }
            0x04 => swim.reset(ResetKind::Cold),
            0x05 => {
                has_disk = !has_disk;
                if has_disk {
                    if let Ok(disk) = Disk::from_image_for(&image(), Reader::Swim) {
                        swim.insert(0, disk);
                    }
                } else {
                    swim.eject(0);
                }
            }
            0x06 => {
                if let Some(bytes) = snapshot(&swim) {
                    if let Ok(reader) = StateReader::new(&bytes)
                        && let Ok(chunk) = reader.load(
                            "swim",
                            SWIM_CLASS.name,
                            SWIM_CLASS.version,
                            &Migrations::new(),
                        )
                    {
                        swim.load(&mut chunk.reader())
                            .expect("our own snapshot loads");
                        assert_eq!(
                            snapshot(&swim).as_deref(),
                            Some(&bytes[..]),
                            "a save/load round trip changed the chip's state"
                        );
                    }
                }
            }
            0x07 => {
                let mut shape = MachineShape::new();
                if shape.add_device("swim", SWIM_CLASS.name).is_ok() {
                    let mut w = StateWriter::new(shape);
                    if let Ok(mut chunk) =
                        w.chunk("swim", SWIM_CLASS.name, SWIM_CLASS.version)
                    {
                        let _ = chunk.write_bytes(&data[at..]);
                    }
                    if let Ok(bytes) = w.to_vec()
                        && let Ok(reader) = StateReader::new(&bytes)
                        && let Ok(chunk) = reader.load(
                            "swim",
                            SWIM_CLASS.name,
                            SWIM_CLASS.version,
                            &Migrations::new(),
                        )
                    {
                        let _ = swim.load(&mut chunk.reader());
                    }
                }
                at = data.len();
                let _ = space.read(BASE, Width::U8, MemAttrs::DEFAULT);
            }
            _ => {}
        }
    }

    // Whatever happened, the chip is still a chip.
    let _ = space.read(BASE, Width::U8, MemAttrs::DEFAULT);
    swim.reset(ResetKind::Cold);
    swim.advance_to(tick + 1);
});
