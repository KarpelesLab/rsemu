//! A guest disk that is a **file**: the round trip, and what a snapshot of one
//! means.
//!
//! Everything the guest does here goes through the ATA command block —
//! `write_reg`, `read_reg`, the data register, `FLUSH CACHE` — because that is
//! the whole of what is on the far side of the cable. The standard
//! `tests/pc_at_ide.rs` holds is that a write is checked against the *medium*
//! rather than against the drive's own buffer; this goes one further and checks
//! it against the medium after the drive that wrote it has been dropped and the
//! image reopened from scratch, which is the only way to prove the bytes
//! actually left the process.
//!
//! Provenance: no image format is implemented in rsemu. qcow2, raw, DMG,
//! DiskCopy 4.2 and LUKS all come from `fstool` (Karpelès Lab, MIT), which
//! `ROADMAP.md` §7.1 names as the storage substrate. No QEMU source was opened.

#![cfg(all(feature = "dev-blk", feature = "dev-ata-disk"))]

use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use rsemu::core::device::Device;
use rsemu::core::hosts::HostObjects;
use rsemu::core::props::{Media, Props, Value};
use rsemu::core::state::{MachineShape, Migrations, StateReader, StateWriter};
use rsemu::dev::ata::disk::{
    self, AtaDisk, DEV_LBA, DiskDevice, Geometry, Identity, Position, Reg, SECTOR, cmd,
};
use rsemu::dev::ata::{Medium, Snapshot};
use rsemu::dev::blk::{Image, ImageOptions};

/// Bits 7 and 5 of the Device register are obsolete and read back as ones.
const DEV_OBSOLETE: u8 = 0xa0;

/// A scratch path nobody else is using, for this process and this call.
fn scratch(name: &str) -> PathBuf {
    static NEXT: AtomicU64 = AtomicU64::new(0);
    let mut path = std::env::temp_dir();
    path.push(format!(
        "rsemu-blk-ata-{}-{}-{name}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    path
}

/// What sector `lba` is stamped with, so a transfer that lands one sector out
/// fails rather than passing by luck.
fn stamp(lba: u64) -> Vec<u8> {
    let mut out = vec![0u8; SECTOR as usize];
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = (lba as u8) ^ (i as u8) ^ 0x5a;
    }
    out[0] = lba as u8;
    out[1] = (lba >> 8) as u8;
    out
}

/// A drive of `image`'s capacity, at device 0.
fn drive_on(image: Arc<Image>) -> AtaDisk {
    let sectors = image.capacity() / SECTOR;
    let id = Identity::new(sectors, geometry_for(sectors), true, 16).expect("a valid drive");
    AtaDisk::with_medium(id, Position::Device0, image).expect("the medium matches the identity")
}

/// A CHS translation the drive will accept for `sectors` sectors.
fn geometry_for(sectors: u64) -> Geometry {
    let cylinders = (sectors / (16 * 63)).clamp(1, 16383) as u16;
    Geometry {
        cylinders,
        heads: 16,
        sectors: 63,
    }
}

/// Select device 0, LBA mode, head bits zero.
fn select(disk: &AtaDisk) {
    disk.write_reg(Reg::Device, u16::from(DEV_OBSOLETE | DEV_LBA));
}

/// `WRITE SECTOR(S)` of one sector at `lba`, driven exactly as `outsw` would.
fn write_sector(disk: &AtaDisk, lba: u64, bytes: &[u8]) {
    select(disk);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, u16::from(lba as u8));
    disk.write_reg(Reg::LbaMid, u16::from((lba >> 8) as u8));
    disk.write_reg(Reg::LbaHigh, u16::from((lba >> 16) as u8));
    disk.write_reg(Reg::Command, u16::from(cmd::WRITE_SECTORS));
    assert_ne!(
        disk.read_alt_status() & disk::ST_DRQ,
        0,
        "the drive asked for the first block"
    );
    for pair in bytes.chunks(2) {
        let word = u16::from(pair[0]) | (u16::from(pair[1]) << 8);
        disk.write_reg(Reg::Data, word);
    }
    let status = disk.read_reg(Reg::Command, false) as u8;
    assert_eq!(
        status & disk::ST_ERR,
        0,
        "the write did not fail: {status:#x}"
    );
}

/// `READ SECTOR(S)` of one sector at `lba`, driven exactly as `insw` would.
fn read_sector(disk: &AtaDisk, lba: u64) -> Vec<u8> {
    select(disk);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, u16::from(lba as u8));
    disk.write_reg(Reg::LbaMid, u16::from((lba >> 8) as u8));
    disk.write_reg(Reg::LbaHigh, u16::from((lba >> 16) as u8));
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    let mut out = Vec::with_capacity(SECTOR as usize);
    for _ in 0..(SECTOR / 2) {
        let word = disk.read_reg(Reg::Data, false);
        out.push(word as u8);
        out.push((word >> 8) as u8);
    }
    out
}

/// `FLUSH CACHE`, and it must succeed.
fn flush(disk: &AtaDisk) {
    disk.write_reg(Reg::Command, u16::from(cmd::FLUSH_CACHE));
    let status = disk.read_reg(Reg::Command, false) as u8;
    assert_eq!(status & disk::ST_ERR, 0, "the flush failed: {status:#x}");
}

// ---------------------------------------------------------------------------
// the round trip
// ---------------------------------------------------------------------------

/// Write through the registers, flush, drop everything, reopen the file, read
/// the same bytes back — for a sparse raw image and for a qcow2.
#[test]
fn sectors_written_through_the_registers_survive_reopening_the_image() {
    for (name, opts) in [
        ("roundtrip.img", ImageOptions::new().create(2 << 20)),
        ("roundtrip.qcow2", ImageOptions::new().create(2 << 20)),
    ] {
        let path = scratch(name);
        let written: Vec<u64> = vec![0, 1, 63, 512, 4095];

        {
            let image = Arc::new(Image::open(&path, &opts).expect("created"));
            let disk = drive_on(image);
            for lba in &written {
                write_sector(&disk, *lba, &stamp(*lba));
            }
            // The guest's own durability barrier, and the whole reason the
            // medium has a `flush`: without it the bytes are the host's
            // problem rather than the file's.
            flush(&disk);
        }

        // A brand new `Image` over the same path — nothing of the first drive,
        // its buffers or its `RamStore`-shaped assumptions survives.
        let image = Arc::new(Image::open(&path, &ImageOptions::new()).expect("reopened"));
        let disk = drive_on(Arc::clone(&image));
        for lba in &written {
            assert_eq!(
                read_sector(&disk, *lba),
                stamp(*lba),
                "{name}: sector {lba} came back from the file"
            );
        }
        // And a sector nobody wrote reads as zero rather than as whatever the
        // host filesystem left in the hole.
        assert_eq!(read_sector(&disk, 2000), vec![0u8; SECTOR as usize]);

        let _ = std::fs::remove_file(&path);
    }
}

/// A qcow2 drive is *sparse*: five sectors written to a 2 MiB disk cost far
/// less than 2 MiB on the host. This is the thing the media-slot path cannot
/// do, and the reason `dev/blk` exists.
#[test]
fn a_qcow2_drive_costs_what_the_guest_wrote_rather_than_its_capacity() {
    let path = scratch("sparse.qcow2");
    let image = Arc::new(
        Image::open(&path, &ImageOptions::new().create(64 << 20)).expect("a 64 MiB qcow2"),
    );
    let disk = drive_on(Arc::clone(&image));
    assert_eq!(disk.identity().capacity(), 64 << 20);
    write_sector(&disk, 100_000, &stamp(7));
    flush(&disk);
    let on_disk = std::fs::metadata(&path).expect("stat").len();
    assert!(
        on_disk < (1 << 20),
        "a 64 MiB drive with one sector written cost {on_disk} bytes"
    );
    assert_eq!(read_sector(&disk, 100_000), stamp(7));
    let _ = std::fs::remove_file(&path);
}

// ---------------------------------------------------------------------------
// the register surface
// ---------------------------------------------------------------------------

/// A debugger must not move the drive. The Alternate Status register is the
/// door `MemAttrs::debug` opens (`ROADMAP.md` §15, invariant 5), and the
/// `debug` flag on `read_reg` is the same requirement on the command block —
/// neither may acknowledge the interrupt or advance the sector buffer, and
/// neither may touch the *file* either.
#[test]
fn a_debug_read_of_a_file_backed_drive_moves_nothing() {
    let path = scratch("debug.img");
    let image =
        Arc::new(Image::open(&path, &ImageOptions::new().create(1 << 20)).expect("created"));
    let disk = drive_on(Arc::clone(&image));
    write_sector(&disk, 3, &stamp(3));
    flush(&disk);

    select(&disk);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, 3);
    disk.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    assert!(disk.irq_asserted(), "a read announces its first block");

    // Alternate Status: the status without the acknowledgement.
    let alt = disk.read_alt_status();
    assert_ne!(alt & disk::ST_DRQ, 0);
    assert!(disk.irq_asserted(), "reading it did not acknowledge");

    // A debug read of the data register hands back the byte in the buffer and
    // does not consume it.
    let peeked = disk.read_reg(Reg::Data, true);
    assert_eq!(
        disk.read_reg(Reg::Data, true),
        peeked,
        "still the same word"
    );
    assert!(disk.irq_asserted(), "and still pending");

    // The real drain then starts where it always would have.
    let mut out = Vec::with_capacity(SECTOR as usize);
    for _ in 0..(SECTOR / 2) {
        let word = disk.read_reg(Reg::Data, false);
        out.push(word as u8);
        out.push((word >> 8) as u8);
    }
    assert_eq!(out, stamp(3), "the debugger stole nothing");
    let _ = std::fs::remove_file(&path);
}

/// A read-only image write protects the drive rather than accepting writes and
/// losing them, and the failure is `ABRT` — the command should not have been
/// issued — not `IDNF`, which would be a lie about the geometry.
#[test]
fn a_read_only_image_aborts_a_write_and_keeps_the_file_intact() {
    let path = scratch("ro.img");
    {
        let image =
            Arc::new(Image::open(&path, &ImageOptions::new().create(1 << 20)).expect("created"));
        let disk = drive_on(image);
        write_sector(&disk, 1, &stamp(1));
        flush(&disk);
    }

    let image = Arc::new(
        Image::open(&path, &ImageOptions::new().read_only(true)).expect("reopened read-only"),
    );
    // Through the machine-description object, because that is where a medium
    // that refuses writes turns into a drive that *says* it is write protected:
    // telling a guest it may write and then failing every write is worse than
    // an honest read-only drive.
    let device = device_on("hd0", Arc::clone(&image));
    let disk = device.drive().expect("a drive").clone();
    assert!(disk.identity().read_only, "IDENTIFY says so up front");
    select(&disk);
    disk.write_reg(Reg::SectorCount, 1);
    disk.write_reg(Reg::LbaLow, 1);
    disk.write_reg(Reg::Command, u16::from(cmd::WRITE_SECTORS));
    let status = disk.read_reg(Reg::Command, false) as u8;
    assert_ne!(status & disk::ST_ERR, 0, "the write was refused");
    assert_ne!(disk.read_reg(Reg::Feature, false) as u8 & disk::ERR_ABRT, 0);
    assert_eq!(read_sector(&disk, 1), stamp(1), "and the file is intact");
    let _ = std::fs::remove_file(&path);
}

// ---------------------------------------------------------------------------
// snapshots
// ---------------------------------------------------------------------------

/// Snapshot through the machine-description object, which is the surface a
/// machine snapshot actually uses.
fn save_of(device: &DiskDevice) -> rsemu::Result<Vec<u8>> {
    let mut shape = MachineShape::new();
    shape
        .add_device("hd", disk::CLASS.name)
        .expect("one device");
    let mut w = StateWriter::new(shape);
    {
        let mut chunk = w
            .chunk("hd", disk::CLASS.name, disk::CLASS.version)
            .expect("a chunk");
        device.save(&mut chunk)?;
    }
    w.to_vec()
}

fn load_into(device: &DiskDevice, image: &[u8]) -> rsemu::Result<()> {
    let reader = StateReader::new(image).expect("a snapshot we just wrote");
    let chunk = reader
        .load(
            "hd",
            disk::CLASS.name,
            disk::CLASS.version,
            &Migrations::new(),
        )
        .expect("the chunk");
    device.load(&mut chunk.reader())
}

/// The `ata.disk` object a machine description builds, over a host-supplied
/// image installed under media slot `slot`.
fn device_on(slot: &str, image: Arc<Image>) -> DiskDevice {
    let hosts = Arc::new(HostObjects::new());
    rsemu::dev::blk::install(&hosts, slot, image).expect("installed");
    let mut props = Props::new();
    props.insert("image", Value::Media(Media::new(slot, Vec::new())));
    props.insert("size", Value::Uint(0));
    let props = props.with_hosts(hosts);
    DiskDevice::new(&props).expect("a drive")
}

/// **What a snapshot of a file-backed disk means, asserted.**
///
/// It *references* the image: the chunk records what the medium is and the
/// drive's protocol state, `save` flushes the file so what is on disk matches
/// the moment the snapshot was taken, and the bytes stay in the file. Saving
/// twice around a load produces byte-identical chunks — the state hash the
/// device contract asks for — and the drive comes back with its geometry, its
/// multiple-block size and its mid-sector position intact.
#[test]
fn a_file_backed_drive_snapshots_by_reference_and_round_trips_identically() {
    let path = scratch("snapshot.qcow2");
    let image =
        Arc::new(Image::open(&path, &ImageOptions::new().create(4 << 20)).expect("created"));
    assert_eq!(image.snapshot(), Snapshot::Reference);
    let device = device_on("hd0", Arc::clone(&image));
    let saved = device.drive().expect("a drive").clone();

    // Put the drive somewhere interesting: a non-default CHS translation, a
    // multiple-block size, and a transfer stopped part way through a sector.
    saved.write_reg(Reg::SectorCount, 32);
    saved.write_reg(Reg::Device, u16::from(DEV_OBSOLETE | 3));
    saved.write_reg(Reg::Command, u16::from(cmd::INIT_DEVICE_PARAMS));
    saved.write_reg(Reg::SectorCount, 8);
    saved.write_reg(Reg::Command, u16::from(cmd::SET_MULTIPLE));
    write_sector(&saved, 77, &stamp(77));
    select(&saved);
    saved.write_reg(Reg::SectorCount, 1);
    saved.write_reg(Reg::LbaLow, 77);
    saved.write_reg(Reg::Command, u16::from(cmd::READ_SECTORS));
    let _ = saved.read_reg(Reg::Data, false);

    let first = save_of(&device).expect("it saves");
    // The chunk is a *reference*: it is a few hundred bytes, not four
    // megabytes. That is the claim, and it is checkable.
    assert!(
        first.len() < 4096,
        "a reference snapshot of a 4 MiB drive is {} bytes",
        first.len()
    );
    assert!(
        String::from_utf8_lossy(&first).contains("qcow2 "),
        "and it names the image it references"
    );

    // Restore into the same drive over the same image, which is what restoring
    // a machine snapshot does.
    load_into(&device, &first).expect("it loads");
    let restored = device.drive().expect("a drive").clone();
    assert_eq!(restored.current_geometry().heads, 4);
    assert_eq!(restored.multiple(), 8);

    // The state hash: save the restored drive and compare the bytes.
    let second = save_of(&device).expect("it saves");
    assert_eq!(first, second, "save -> load -> save is byte identical");

    // And the referenced bytes are still reachable, because they never left
    // the file.
    let mut got = vec![0u8; SECTOR as usize];
    restored
        .read_media(77 * SECTOR, &mut got)
        .expect("in range");
    assert_eq!(got, stamp(77));
    let _ = std::fs::remove_file(&path);
}

/// A reference snapshot restored against a *different* image is refused rather
/// than believed. This is the check that makes "the image is outside the
/// snapshot" a documented contract rather than an accident waiting to happen.
#[test]
fn a_reference_snapshot_will_not_load_against_another_image() {
    let one = scratch("one.img");
    let two = scratch("two.img");
    let opts = ImageOptions::new().create(1 << 20);
    let first = Arc::new(Image::open(&one, &opts).expect("created"));
    let second = Arc::new(Image::open(&two, &opts).expect("created"));

    let saved = device_on("hd0", first);
    let image = save_of(&saved).expect("it saves");
    let other = device_on("hd0", second);
    let err = load_into(&other, &image).expect_err("a different image");
    assert!(
        format!("{err}").contains("different medium"),
        "it says why: {err}"
    );
    let _ = std::fs::remove_file(&one);
    let _ = std::fs::remove_file(&two);
}

/// `snapshot=capture` puts the bytes in the chunk, for an image small enough
/// that a self-contained snapshot is what the user wants. Both policies are on
/// offer; what is not on offer is capturing sixteen gigabytes silently.
#[test]
fn a_capturing_file_backed_drive_puts_the_bytes_in_the_chunk() {
    let path = scratch("capture.img");
    let opts = ImageOptions::new()
        .create(64 * 512)
        .snapshot(Snapshot::Capture);
    let image = Arc::new(Image::open(&path, &opts).expect("created"));
    let device = device_on("hd0", Arc::clone(&image));
    let disk = device.drive().expect("a drive").clone();
    write_sector(&disk, 5, &stamp(5));

    let chunk = save_of(&device).expect("it saves");
    assert!(
        chunk.len() > 64 * 512,
        "a captured 32 KiB drive is at least 32 KiB of chunk, got {}",
        chunk.len()
    );

    // Overwrite the sector behind the drive's back, then restore: the captured
    // bytes come back, which is exactly what `Capture` promises and `Reference`
    // does not.
    disk.write_media(5 * SECTOR, &[0u8; SECTOR as usize])
        .expect("in range");
    load_into(&device, &chunk).expect("it loads");
    assert_eq!(read_sector(&disk, 5), stamp(5));
    let _ = std::fs::remove_file(&path);
}

/// `snapshot=refuse` fails loudly at `save`, which is the honest answer for a
/// medium a snapshot has no business either copying or referencing.
#[test]
fn a_refusing_drive_says_no_at_save_time() {
    let path = scratch("refuse.img");
    let opts = ImageOptions::new()
        .create(1 << 20)
        .snapshot(Snapshot::Refuse);
    let image = Arc::new(Image::open(&path, &opts).expect("created"));
    let device = device_on("hd0", image);
    let err = save_of(&device).expect_err("it refuses");
    assert!(format!("{err}").contains("refuses"), "{err}");
    let _ = std::fs::remove_file(&path);
}

// ---------------------------------------------------------------------------
// the wiring
// ---------------------------------------------------------------------------

/// The path `rsemu run pc-at --drive hd0=disk.qcow2` takes, without a PC: a
/// host installs an image under the media slot's name, and the `ata.disk`
/// object in the machine description picks it up with **no change to the
/// machine file and no change to the IDE adapter**. The media-slot path is
/// still there and still works — that is the other half of the claim.
#[test]
fn a_host_supplied_image_reaches_the_drive_through_the_media_slot() {
    let path = scratch("wired.qcow2");
    let hosts = Arc::new(HostObjects::new());
    let image =
        Arc::new(Image::open(&path, &ImageOptions::new().create(8 << 20)).expect("created"));
    rsemu::dev::blk::install(&hosts, "hd0", Arc::clone(&image)).expect("installed");

    // Exactly what `machines/pc-at.machine` writes: a media slot named `hd0`,
    // a bay, and a `size` of zero because the machine file does not know how
    // big the user's disk is.
    let mut props = Props::new();
    props.insert("image", Value::Media(Media::new("hd0", Vec::new())));
    props.insert("size", Value::Uint(0));
    props.insert("bay", Value::Str(String::from("ide0-master")));
    let props = props.with_hosts(Arc::clone(&hosts));

    let device = DiskDevice::new(&props).expect("a drive");
    let drive = device.drive().expect("the bay is not empty").clone();
    assert_eq!(drive.identity().capacity(), 8 << 20, "the image's capacity");
    assert_eq!(drive.medium().snapshot(), Snapshot::Reference);
    write_sector(&drive, 9, &stamp(9));
    flush(&drive);
    drop(device);

    let reopened = Arc::new(Image::open(&path, &ImageOptions::new()).expect("reopened"));
    let check = drive_on(reopened);
    assert_eq!(read_sector(&check, 9), stamp(9));

    // The same machine description with no image installed is the media-slot
    // path, unchanged: a `size` of zero and no bytes is still an empty bay.
    let plain = Props::new();
    let mut plain = plain;
    plain.insert("image", Value::Media(Media::new("hd0", Vec::new())));
    plain.insert("size", Value::Uint(0));
    let empty = DiskDevice::new(&plain).expect("a bay");
    assert!(empty.drive().is_none(), "no bytes and no image is no disk");

    let _ = std::fs::remove_file(&path);
}

/// A media slot bound to real bytes still builds a RAM-backed drive that
/// captures its contents — the `no_std` path, proven not to have regressed by
/// the existence of the file-backed one.
#[test]
fn the_media_slot_path_still_builds_a_capturing_ram_drive() {
    let mut props = Props::new();
    props.insert("image", Value::Media(Media::new("hd0", vec![0xa5u8; 4096])));
    let device = DiskDevice::new(&props).expect("a drive");
    let drive = device.drive().expect("the bay is not empty").clone();
    assert_eq!(drive.identity().capacity(), 4096);
    assert_eq!(drive.medium().snapshot(), Snapshot::Capture);
    assert_eq!(read_sector(&drive, 0), vec![0xa5u8; SECTOR as usize]);
}
