#![no_main]
//! A disk image, from a file nobody trusts.
//!
//! `CLAUDE.md` asks for a fuzz target on disk-image parsers by name, and this is
//! the one surface in the crate where a **file the user did not write** is
//! parsed before a single guest instruction runs: `rsemu run pc-at --drive
//! hd0=downloaded.qcow2` hands an arbitrary byte string to a header parser, a
//! refcount table walk and an L1/L2 cluster lookup. A malformed one must produce
//! an `Err`, never a panic, an unbounded allocation or a hang.
//!
//! The parsers themselves are `fstool`'s (`ROADMAP.md` §7.1 puts image formats
//! there rather than in rsemu, and QEMU's tree is not a source this project may
//! read — `CLAUDE.md` §1). Fuzzing them from here is still the right place: this
//! is where rsemu's *use* of them is, and the properties asserted below are
//! rsemu's contract rather than `fstool`'s.
//!
//! Four properties, in order of how much they would hurt:
//!
//! * **Opening arbitrary bytes never panics.** It returns a drive or an error.
//! * **The bounds check holds whatever the header claimed.** A capacity that
//!   came out of the fuzzed file is still the capacity every read and write is
//!   checked against, in `u64`, so no offset can wrap into range. A read that
//!   is refused *by that check* leaves the destination buffer untouched — a
//!   half-filled buffer handed to a guest is the silent corruption the
//!   three-way `MemResult` exists to prevent — and, the other way round, a
//!   sector that **is** on the drive never comes back as `BadAccess` however
//!   corrupt the image is, because that would be the drive lying about its own
//!   geometry.
//! * **Every failure is one of the three answers.** `Ok`, a bus fault, or
//!   `Retry` — never a silent `0xff`, never an `Option`.
//! * **The snapshot loader is a parser on untrusted bytes too.** A tail of the
//!   input is handed to `Device::load`, which must reject it or accept it and
//!   never panic, and the drive must still work afterwards.
//!
//! # Input encoding
//!
//! Hand-decoded rather than derived, so a corpus stays meaningful across
//! `arbitrary` versions — the argument `state_roundtrip` makes:
//!
//! ```text
//!   byte 0        flags: bit 0 read-only, bits 1..2 the snapshot policy
//!   byte 1        how many of the following bytes are the opcode stream
//!   ...           the opcode stream
//!   ...           everything after it is the image file, so a real qcow2 --
//!                 which is hundreds of kilobytes before a guest writes to it --
//!                 fits without a length field wide enough to be worth mutating
//!
//!   the opcode stream:
//!                   0x00 ss ss  read one sector at LBA ssss (mod capacity)
//!                   0x01 ss ss  write one sector there, filled with ss
//!                   0x02        flush
//!                   0x03 oo…    read 8 bytes at a raw 64-bit offset
//!                   0x04        save, then load what was saved: a round trip
//!                   0x05 ...    load the rest of the input as a snapshot chunk
//! ```
//!
//! Anything else is skipped, which keeps a mutated corpus productive rather
//! than mostly rejected.
//!
//! # Known finding: this target currently reproduces an upstream panic
//!
//! It found one within a few minutes of its first run, which is the argument
//! for its existence:
//!
//! > `fstool` 0.4.23, `src/block/qcow2/mod.rs:952` — `ensure_mapping` indexes
//! > `self.l1l2.l1[l1_idx]` without a bounds check. `l1l2.rs` deliberately does
//! > **not** require the header's `l1_size` to cover the image's virtual size
//! > (its comment says so), so a qcow2 declaring a non-zero `size` and
//! > `l1_size = 0` opens cleanly, reads fine — the read path uses `get` — and
//! > **panics on the guest's first write**. A crafted image plus one `WRITE
//! > SECTOR(S)` aborts the emulator.
//!
//! It is not fixable from here: guarding it in rsemu would mean parsing the
//! qcow2 header, which is exactly the parallel implementation `ROADMAP.md` §7.1
//! forbids. It is an `fstool` fix. Until it lands, `blk_image` is deliberately
//! **not** in `.github/workflows/fuzz.yml`'s smoke list — the list is curated,
//! not automatic — while `cargo fuzz build` still builds it, so the target
//! keeps doing its other job of catching an API drift. Adding one word to that
//! list is what turns it back on.

use libfuzzer_sys::fuzz_target;

use rsemu::core::device::Device;
use rsemu::core::error::BusError;
use rsemu::core::hosts::HostObjects;
use rsemu::core::props::{Media, Props, Value};
use rsemu::core::state::{ChunkReader, MachineShape, Migrations, StateReader, StateWriter};
use rsemu::dev::ata::disk::{self, DiskDevice, Reg, SECTOR, cmd};
use rsemu::dev::ata::{Medium, Snapshot};
use rsemu::dev::blk::{Image, ImageOptions};
use std::path::PathBuf;
use std::sync::Arc;

/// One scratch file per process, rewritten each iteration.
///
/// The format backends open by path — a qcow2 header parse is a `pread`, not a
/// slice — so there is no way to reach them without a file. One path, reused,
/// keeps the per-iteration cost to a single `write` on what is almost always
/// tmpfs.
fn scratch() -> PathBuf {
    let mut path = std::env::temp_dir();
    path.push(format!("rsemu-fuzz-blk-{}.img", std::process::id()));
    path
}

/// The drive class's snapshot chunk, for the round trip.
fn save_of(device: &DiskDevice) -> Option<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape.add_device("hd", disk::CLASS.name).ok()?;
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w.chunk("hd", disk::CLASS.name, disk::CLASS.version).ok()?;
        device.save(&mut chunk).ok()?;
    }
    w.to_vec().ok()
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 3 {
        return;
    }
    let flags = data[0];
    let want = usize::from(data[1]);
    let body = &data[2..];
    let split = want.min(body.len());
    let (mut ops, file) = body.split_at(split);
    if file.is_empty() {
        return;
    }

    let path = scratch();
    if std::fs::write(&path, file).is_err() {
        return;
    }

    let options = ImageOptions::new()
        .read_only(flags & 1 != 0)
        .snapshot(match (flags >> 1) & 3 {
            0 => Snapshot::Reference,
            1 => Snapshot::Capture,
            _ => Snapshot::Refuse,
        });

    // Property 1: arbitrary bytes are a drive or an error, never a panic.
    let Ok(image) = Image::open(&path, &options) else {
        return;
    };
    let capacity = image.capacity();
    // `Image::open` promises a non-zero whole number of sectors, whatever the
    // file claimed. If that ever stops holding, the sector arithmetic below
    // divides by zero and says so loudly.
    assert!(capacity >= SECTOR && capacity % SECTOR == 0, "{capacity}");

    // Through the machine-description object, because that is the path a run
    // takes and it is where the capacity, the write protection and the snapshot
    // policy are decided.
    let hosts = Arc::new(HostObjects::new());
    if rsemu::dev::blk::install(&hosts, "hd0", Arc::new(image)).is_err() {
        return;
    }
    let mut props = Props::new();
    props.insert("image", Value::Media(Media::new("hd0", Vec::new())));
    props.insert("size", Value::Uint(0));
    let props = props.with_hosts(hosts);
    let Ok(device) = DiskDevice::new(&props) else {
        return;
    };
    let Some(drive) = device.drive().cloned() else {
        return;
    };
    let sectors = capacity / SECTOR;

    let mut budget = 512u32;
    while let Some((&op, rest)) = ops.split_first() {
        ops = rest;
        if budget == 0 {
            break;
        }
        budget -= 1;
        match op {
            0x00 if ops.len() >= 2 => {
                let lba = u64::from(u16::from_le_bytes([ops[0], ops[1]])) % sectors;
                ops = &ops[2..];
                let mut got = vec![0xccu8; SECTOR as usize];
                // Property 2: a sector that *is* on the drive may still fail to
                // read — a corrupt L2 entry, a truncated file, a codec that is
                // not built in — but it must never come back as `BadAccess`,
                // which means "no such sector" and would be the drive lying
                // about its own geometry. `dev::blk::media_error` is what makes
                // that hold; this is the assertion that keeps it holding.
                if let Err(e) = drive.medium().read_at(lba * SECTOR, &mut got) {
                    assert_ne!(
                        e,
                        BusError::BadAccess,
                        "{lba} is inside a {sectors}-sector drive"
                    );
                }
            }
            0x01 if ops.len() >= 2 => {
                let lba = u64::from(u16::from_le_bytes([ops[0], ops[1]])) % sectors;
                let fill = ops[0];
                ops = &ops[2..];
                // May legitimately fail: a read-only image, a compressed qcow2
                // cluster, a backing chain. What it may not do is panic.
                let _ = drive.write_media(lba * SECTOR, &vec![fill; SECTOR as usize]);
            }
            0x02 => {
                let _ = drive.flush_media();
                // And through the guest's own barrier, which is a command that
                // has to answer whatever the host said.
                drive.write_reg(Reg::Command, u16::from(cmd::FLUSH_CACHE));
                let _ = drive.read_reg(Reg::Command, false);
            }
            0x03 if ops.len() >= 8 => {
                let mut raw = [0u8; 8];
                raw.copy_from_slice(&ops[..8]);
                ops = &ops[8..];
                let offset = u64::from_le_bytes(raw);
                let mut got = [0x5au8; 8];
                let inside = offset.checked_add(8).is_some_and(|end| end <= capacity);
                match drive.medium().read_at(offset, &mut got) {
                    // `BadAccess` is the drive's own bounds check and nothing
                    // else: it happens exactly when the range is off the end,
                    // and it leaves the destination untouched. A partially
                    // filled buffer is the silent corruption `MemResult` exists
                    // to rule out.
                    Err(BusError::BadAccess) => {
                        assert!(!inside, "{offset:#x} is inside a {capacity}-byte drive");
                        assert_eq!(got, [0x5au8; 8]);
                    }
                    // Any other error is the *image* failing, which it may do
                    // however corrupt it is — but only for a range that is
                    // actually on the drive.
                    Err(_) => assert!(inside, "{offset:#x} is past a {capacity}-byte drive"),
                    Ok(()) => assert!(inside),
                }
            }
            0x04 => {
                // Property 4a: save then load is a round trip, or an honest
                // refusal (a `Refuse` policy, an unreadable medium).
                if let Some(chunk) = save_of(&device) {
                    let Ok(reader) = StateReader::new(&chunk) else {
                        continue;
                    };
                    let Ok(loaded) = reader.load(
                        "hd",
                        disk::CLASS.name,
                        disk::CLASS.version,
                        &Migrations::new(),
                    ) else {
                        continue;
                    };
                    let _ = device.load(&mut loaded.reader());
                }
            }
            0x05 => {
                // Property 4b: an arbitrary chunk is rejected or accepted, and
                // the drive still works afterwards either way.
                let mut reader = ChunkReader::new(ops);
                let _ = device.load(&mut reader);
                ops = &[];
                let mut got = vec![0u8; SECTOR as usize];
                let _ = drive.read_media(0, &mut got);
            }
            _ => {}
        }
    }

    // Whatever happened, the drive still answers its command block.
    let _ = drive.read_alt_status();
    drive.write_reg(Reg::Command, u16::from(cmd::IDENTIFY));
    let _ = drive.read_reg(Reg::Data, true);

    // The three-way answer, once more, at the one address that can never be on
    // any medium.
    let mut past = [0u8; 8];
    assert!(matches!(
        drive.medium().read_at(u64::MAX - 3, &mut past),
        Err(BusError::BadAccess)
    ));
});
