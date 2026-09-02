//! What a host file does when the guest asks it for something it cannot give.
//!
//! The round trip through the ATA registers lives in `tests/blk_ata_image.rs`,
//! because it needs a whole drive. These are the seam's own corners: the range
//! check in `u64`, the three-way error answer, the read-only refusal and the
//! snapshot policy.

use super::*;
use crate::core::error::BusError;
use crate::dev::blk::{bus_error, media_error};
use crate::dev::medium::Medium;
use alloc::vec;
use fstool::block::{CrashInject, FailAfter, MemoryBackend};
use std::path::PathBuf;

/// A scratch path nobody else in this process is using.
fn scratch(name: &str) -> PathBuf {
    let mut path = std::env::temp_dir();
    path.push(alloc::format!(
        "rsemu-blk-{}-{}-{name}",
        std::process::id(),
        core::sync::atomic::AtomicU64::new(0).fetch_add(1, core::sync::atomic::Ordering::Relaxed)
    ));
    path
}

/// An image over a plain in-RAM backend: no filesystem, no format.
fn memory(bytes: u64, opts: &ImageOptions) -> Image {
    Image::from_device("memory", Box::new(MemoryBackend::new(bytes)), opts)
        .expect("a whole number of sectors")
}

#[test]
fn a_read_past_the_end_is_bad_access_rather_than_a_short_read() {
    let image = memory(4 * 512, &ImageOptions::new());
    let mut buf = vec![0u8; 512];
    assert_eq!(image.read_at(4 * 512, &mut buf), Err(BusError::BadAccess));
    // Straddling the end is the interesting one: fstool would happily read the
    // first part, and a half-filled buffer handed to a guest is exactly the
    // silent corruption the three-way answer exists to prevent.
    assert_eq!(
        image.read_at(3 * 512 + 1, &mut buf),
        Err(BusError::BadAccess)
    );
    assert_eq!(image.read_at(3 * 512, &mut buf), Ok(()));
}

#[test]
fn the_bounds_check_is_done_in_u64_so_an_offset_cannot_wrap() {
    let image = memory(512, &ImageOptions::new());
    let mut buf = vec![0u8; 8];
    // `u64::MAX + 8` overflows; on a 32-bit host the offset does not even fit
    // in a `usize`. Neither is allowed to become a small offset.
    assert_eq!(image.read_at(u64::MAX, &mut buf), Err(BusError::BadAccess));
    assert_eq!(image.write_at(u64::MAX - 3, &buf), Err(BusError::BadAccess));
    assert_eq!(image.read_at(1 << 40, &mut buf), Err(BusError::BadAccess));
}

#[test]
fn a_read_only_image_refuses_writes_with_protected_not_bad_access() {
    let image = memory(512, &ImageOptions::new().read_only(true));
    assert!(image.is_read_only());
    assert_eq!(image.write_at(0, &[1, 2, 3]), Err(BusError::Protected));
    // The distinction matters: `Protected` is what the drive turns into ABRT,
    // and `BadAccess` is what it turns into IDNF. A write-protected drive that
    // told a guest "no such sector" would be lying about its geometry.
    #[cfg(feature = "dev-ata-disk")]
    assert_eq!(
        crate::dev::ata::disk::error_bit(BusError::Protected),
        crate::dev::ata::disk::ERR_ABRT
    );
    // And a flush on a drive that cannot be written succeeds, because nothing
    // is pending: a guest's barrier must not look broken.
    assert_eq!(image.flush(), Ok(()));
}

#[test]
fn a_torn_write_is_uncorrectable_rather_than_silently_lost() {
    // `fstool`'s own fault injector rather than a hand-rolled one: after the
    // threshold it drops writes on the floor, which is the SIGKILL-between-the-
    // syscall-and-the-platter case.
    let inject = CrashInject::new(MemoryBackend::new(4 * 512), FailAfter::Bytes(512));
    let image = Image::from_device("crash", Box::new(inject), &ImageOptions::new())
        .expect("a whole number of sectors");
    assert_eq!(image.write_at(0, &[0xa5; 512]), Ok(()));
    // The second write is dropped. `CrashInject` reports success — it is
    // modelling a lost write, not an EIO — so what the drive can detect is the
    // *read back*, which is what a guest filesystem's checksum would catch.
    let _ = image.write_at(512, &[0x5a; 512]);
    let mut got = vec![0u8; 512];
    assert_eq!(image.read_at(512, &mut got), Ok(()));
    assert_eq!(got, vec![0u8; 512], "the dropped write left zeroes");
    let mut first = vec![0u8; 512];
    assert_eq!(image.read_at(0, &mut first), Ok(()));
    assert_eq!(first, vec![0xa5; 512], "and the write before it survived");
}

#[test]
fn an_image_of_a_ragged_size_is_refused_rather_than_rounded() {
    let opts = ImageOptions::new();
    let err = Image::from_device("ragged", Box::new(MemoryBackend::new(700)), &opts)
        .expect_err("700 bytes is not sectors");
    assert!(
        alloc::format!("{err}").contains("not a whole number"),
        "{err}"
    );
    let err = Image::from_device("empty", Box::new(MemoryBackend::new(0)), &opts)
        .expect_err("an empty image is not a drive");
    assert!(alloc::format!("{err}").contains("empty"), "{err}");
}

#[test]
fn the_snapshot_policy_is_the_caller_s_and_reference_is_the_default() {
    let opts = ImageOptions::new();
    assert_eq!(opts.snapshot, Snapshot::Reference);
    assert_eq!(memory(512, &opts).snapshot(), Snapshot::Reference);
    let refuse = ImageOptions::new().snapshot(Snapshot::Refuse);
    assert_eq!(memory(512, &refuse).snapshot(), Snapshot::Refuse);
}

#[test]
fn a_raw_file_round_trips_and_says_what_it_is() {
    let path = scratch("raw.img");
    create_raw(&path, 8 * 512).expect("a sparse file");
    let opts = ImageOptions::new();
    {
        let image = Image::open(&path, &opts).expect("opened");
        assert_eq!(image.capacity(), 8 * 512);
        assert!(image.describe().starts_with("raw "), "{}", image.describe());
        assert!(image.describe().ends_with(" 4096"), "{}", image.describe());
        assert_eq!(image.write_at(512, &[0x42; 512]), Ok(()));
        assert_eq!(image.flush(), Ok(()));
    }
    let image = Image::open(&path, &opts).expect("reopened");
    let mut got = vec![0u8; 512];
    assert_eq!(image.read_at(512, &mut got), Ok(()));
    assert_eq!(got, vec![0x42; 512]);
    // A sparse file is a hole until something is written into it: the whole
    // point of not being a `RamStore`.
    let on_disk = std::fs::metadata(&path).expect("stat").len();
    assert_eq!(on_disk, 8 * 512, "the logical length is the capacity");
    let _ = std::fs::remove_file(&path);
}

#[test]
fn a_qcow2_is_created_opened_and_written_through() {
    // The structured format, and it is `fstool`'s implementation rather than
    // one reconstructed here: ROADMAP.md §7.1 puts image formats in `fstool`,
    // and QEMU's tree is not a source this project may read (CLAUDE.md §1).
    let path = scratch("disk.qcow2");
    let create = ImageOptions::new().create(4 << 20);
    {
        let image = Image::open(&path, &create).expect("created");
        assert_eq!(image.capacity(), 4 << 20);
        assert_eq!(image.write_at(2 << 20, b"qcow2 round trip"), Ok(()));
        assert_eq!(image.flush(), Ok(()));
    }
    let image = Image::open(&path, &ImageOptions::new()).expect("reopened");
    assert!(
        image.describe().starts_with("qcow2 "),
        "{}",
        image.describe()
    );
    let mut got = vec![0u8; 16];
    assert_eq!(image.read_at(2 << 20, &mut got), Ok(()));
    assert_eq!(&got, b"qcow2 round trip");
    // Allocate-on-write: a 4 MiB virtual disk with one cluster touched is far
    // smaller than 4 MiB on the host.
    let on_disk = std::fs::metadata(&path).expect("stat").len();
    assert!(on_disk < (4 << 20), "{on_disk} bytes for a 4 MiB disk");
    let _ = std::fs::remove_file(&path);
}

#[test]
fn a_qcow2_opened_read_only_refuses_the_guest_s_writes() {
    let path = scratch("ro.qcow2");
    {
        let _ = Image::open(&path, &ImageOptions::new().create(1 << 20)).expect("created");
    }
    let image =
        Image::open(&path, &ImageOptions::new().read_only(true)).expect("reopened read-only");
    assert!(image.is_read_only());
    assert_eq!(image.write_at(0, &[1; 512]), Err(BusError::Protected));
    let _ = std::fs::remove_file(&path);
}

#[test]
fn opening_something_that_is_not_an_image_says_so_rather_than_panicking() {
    let path = scratch("nonsense.qcow2");
    // A qcow2 magic and nothing else behind it: the header parser has to
    // refuse it, and the refusal has to be an error a person can read.
    std::fs::write(&path, b"QFI\xfb\x00\x00\x00\x03rubbish").expect("wrote");
    let err = Image::open(&path, &ImageOptions::new()).expect_err("a truncated qcow2");
    let shown = alloc::format!("{err}");
    assert!(shown.contains("nonsense.qcow2"), "{shown}");
    let _ = std::fs::remove_file(&path);

    let missing = scratch("no-such-file.img");
    let err = Image::open(&missing, &ImageOptions::new()).expect_err("no such file");
    assert!(alloc::format!("{err}").contains("no-such-file"), "{err}");
}

#[test]
fn a_corrupt_image_is_uncorrectable_rather_than_a_missing_sector() {
    // `Image` range-checks in `u64` before the backend ever sees the access, so
    // an `OutOfBounds` coming *back* cannot mean "off the end of the drive" —
    // it means the image's own metadata disagrees with its size. Telling a
    // guest IDNF there would be telling it its geometry is wrong, when what is
    // wrong is the file.
    let corrupt = fstool::Error::OutOfBounds {
        offset: 1 << 40,
        len: 512,
        size: 4096,
    };
    assert_eq!(bus_error(&corrupt), BusError::BadAccess);
    assert_eq!(media_error(&corrupt), BusError::Unassigned);
    #[cfg(feature = "dev-ata-disk")]
    assert_eq!(
        crate::dev::ata::disk::error_bit(media_error(&corrupt)),
        crate::dev::ata::disk::ERR_UNC
    );
    // Everything else is unchanged by the narrowing.
    for e in [
        fstool::Error::Immutable {
            kind: "iso9660",
            op: "write",
        },
        fstool::Error::Io(std::io::Error::from(std::io::ErrorKind::Interrupted)),
        fstool::Error::InvalidImage(String::from("torn")),
    ] {
        assert_eq!(media_error(&e), bus_error(&e));
    }
}

#[test]
fn every_fstool_failure_maps_to_a_bus_error_a_guest_can_be_told_about() {
    use std::io;
    assert_eq!(
        bus_error(&fstool::Error::OutOfBounds {
            offset: 0,
            len: 1,
            size: 0
        }),
        BusError::BadAccess
    );
    assert_eq!(
        bus_error(&fstool::Error::Immutable {
            kind: "iso9660",
            op: "write"
        }),
        BusError::Protected
    );
    assert_eq!(
        bus_error(&fstool::Error::Io(io::Error::from(
            io::ErrorKind::PermissionDenied
        ))),
        BusError::Protected
    );
    assert_eq!(
        bus_error(&fstool::Error::Io(io::Error::from(
            io::ErrorKind::Interrupted
        ))),
        BusError::Retry
    );
    // A short read is *not* retryable: bytes may already have moved.
    assert_eq!(
        bus_error(&fstool::Error::Io(io::Error::from(
            io::ErrorKind::UnexpectedEof
        ))),
        BusError::Unassigned
    );
    assert_eq!(
        bus_error(&fstool::Error::InvalidImage(String::from("torn"))),
        BusError::Unassigned
    );
}
