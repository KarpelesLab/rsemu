//! One promise, made by every device that has a [`Medium`]: **a run that ends
//! makes the guest's writes durable, and a medium that refuses says so.**
//!
//! `Device::flush` is what `rsemu run` calls on the way out, and it is the only
//! thing standing between `--drive hd0=disk.qcow2` and a qcow2 whose L2 tables
//! never reached the file. Each storage device has its own answer to it — NVM
//! Express's `Flush`, ATA's `FLUSH CACHE`, virtio's `VIRTIO_BLK_T_FLUSH` — and
//! each was written on its own, months apart, against a different
//! specification. Nothing until now asked whether they *agree*.
//!
//! The two halves of the promise are separate defects when they break, so they
//! are separate assertions here:
//!
//! * a device that never calls [`Medium::flush`] silently loses data, and
//! * a device that calls it and drops the error tells `rsemu run` the disk is
//!   safe when it is not, which is worse — the process exits `0`.
//!
//! Both are asked through the **machine**, not through a device handle, because
//! that is the path `rsemu run` takes: `Machine::flush` walks every realized
//! device and returns the first refusal.
//!
//! # Every board with a drive is in the table
//!
//! `usb.storage` was the exception when this file was written: it had no
//! `Device::flush` at all, inherited `core::device`'s no-op, and a file-backed
//! USB stick was synced only when the guest itself sent `SYNCHRONIZE CACHE`.
//! That gap was named here rather than skipped, and closing it needed one
//! method and the `usb-mini` row below — which is the shape a ledger entry
//! should have.

#![cfg(any(
    feature = "machine-nvme-mini",
    feature = "machine-ahci-mini",
    feature = "machine-riscv-virt",
    feature = "machine-usb-mini"
))]

use std::sync::Arc;

use rsemu::core::device::ResetKind;
use rsemu::core::error::BusError;
use rsemu::core::space::{MemResult, RamStore};
use rsemu::dev::medium::Medium;
use rsemu::machine::catalog::CatalogEntry;

/// How big the stand-in drive is. A whole number of 512-byte sectors, which
/// every device on the seam requires, and small enough that three of them cost
/// nothing.
const CAPACITY: u64 = 64 * 512;

/// A board, the media slot its machine file names, and the class behind it.
struct Board {
    entry: &'static CatalogEntry,
    slot: &'static str,
    /// Which device model actually holds the medium — not always the device the
    /// board is named for: `ahci-mini`'s HBA has no medium of its own and hands
    /// the taskfile to an `ata.disk`, which is the object that flushes.
    device: &'static str,
    /// Other media slots the board's own objects name, bound to nothing so that
    /// it realizes. Only `riscv-virt` has any: two flash banks.
    empty: &'static [&'static str],
}

/// Every shipped board in this build that puts a [`Medium`] behind a guest
/// storage controller.
const BOARDS: &[Board] = &[
    #[cfg(feature = "machine-nvme-mini")]
    Board {
        entry: &rsemu::machine::catalog::NVME_MINI,
        slot: "nvme0",
        device: "nvme.controller",
        empty: &[],
    },
    #[cfg(feature = "machine-ahci-mini")]
    Board {
        entry: &rsemu::machine::catalog::AHCI_MINI,
        slot: "sata0",
        device: "ata.disk",
        empty: &[],
    },
    #[cfg(feature = "machine-riscv-virt")]
    Board {
        entry: &rsemu::machine::catalog::RISCV_VIRT,
        slot: "disk",
        device: "virtio.blk",
        empty: &["flash0", "flash1", "firmware", "initrd"],
    },
    #[cfg(feature = "machine-usb-mini")]
    Board {
        entry: &rsemu::machine::catalog::USB_MINI,
        slot: "usb0",
        device: "usb.storage",
        empty: &["firmware"],
    },
];

/// A medium that takes every write and cannot make one durable: a full
/// filesystem, a failing disk, an `fsync` that came back `EIO`.
///
/// It counts the attempts, so "the flush failed" and "no flush was attempted"
/// are distinguishable — they are two different defects with the same symptom.
#[derive(Debug)]
struct NoSync {
    store: RamStore,
    attempts: std::sync::atomic::AtomicU32,
}

impl NoSync {
    fn new() -> NoSync {
        NoSync {
            store: RamStore::new(CAPACITY),
            attempts: std::sync::atomic::AtomicU32::new(0),
        }
    }

    fn attempts(&self) -> u32 {
        self.attempts.load(std::sync::atomic::Ordering::Relaxed)
    }
}

impl Medium for NoSync {
    fn capacity(&self) -> u64 {
        CAPACITY
    }

    fn read_at(&self, offset: u64, dst: &mut [u8]) -> MemResult {
        self.store.read_at(offset, dst)
    }

    fn write_at(&self, offset: u64, src: &[u8]) -> MemResult {
        self.store.write_at(offset, src)
    }

    fn flush(&self) -> MemResult {
        self.attempts
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        Err(BusError::Unassigned)
    }
}

/// A medium that takes writes and *can* make them durable, counting the same
/// thing — the control for the assertion above.
#[derive(Debug)]
struct Syncs {
    store: RamStore,
    attempts: std::sync::atomic::AtomicU32,
}

impl Syncs {
    fn new() -> Syncs {
        Syncs {
            store: RamStore::new(CAPACITY),
            attempts: std::sync::atomic::AtomicU32::new(0),
        }
    }

    fn attempts(&self) -> u32 {
        self.attempts.load(std::sync::atomic::Ordering::Relaxed)
    }
}

impl Medium for Syncs {
    fn capacity(&self) -> u64 {
        CAPACITY
    }

    fn read_at(&self, offset: u64, dst: &mut [u8]) -> MemResult {
        self.store.read_at(offset, dst)
    }

    fn write_at(&self, offset: u64, src: &[u8]) -> MemResult {
        self.store.write_at(offset, src)
    }

    fn flush(&self) -> MemResult {
        self.attempts
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        Ok(())
    }
}

/// Build `board` with `medium` in its media slot and hand back the machine.
///
/// The medium goes in the way `rsemu run … --drive slot=file` puts one there,
/// so what is proved is the path a person actually uses.
fn realize(board: &Board, medium: Arc<dyn Medium>) -> rsemu::machine::Machine {
    let mut options = rsemu::machine::catalog::build_options().expect("this build's options");
    rsemu::dev::medium::install(&options.realize.hosts, board.slot, medium)
        .expect("nothing else claimed the name");
    // Bound to no bytes: the medium above wins, and this is only how the machine
    // file's `image = "…"` finds a slot at all.
    options.realize.media.insert(board.slot, Vec::new());
    for slot in board.empty {
        options.realize.media.insert(*slot, Vec::new());
    }
    // No `disk`/`storage` parameter is set: a host-installed medium brings its
    // own capacity and the machine file's default size is ignored, which is
    // exactly what `--drive` relies on and is worth exercising here too.

    let registry = rsemu::machine::catalog::registry().expect("this build's registry");
    let mut machine =
        match rsemu::machine::build(board.entry.name, board.entry.source, &registry, &options) {
            Ok(m) => m,
            Err(e) => panic!("{} does not realize: {e}", board.entry.name),
        };
    machine.reset(ResetKind::Cold);
    machine.sweep();
    machine
}

/// The half that loses data quietly: a device that never asks the medium to
/// make anything durable.
#[test]
fn every_board_with_a_drive_flushes_its_medium_when_the_run_ends() {
    for board in BOARDS {
        let medium = Arc::new(Syncs::new());
        let machine = realize(board, Arc::clone(&medium) as Arc<dyn Medium>);
        assert_eq!(
            medium.attempts(),
            0,
            "{}: before the run ends",
            board.device
        );

        machine
            .flush()
            .unwrap_or_else(|e| panic!("{}: a medium that syncs refused: {e}", board.device));
        assert!(
            medium.attempts() >= 1,
            "{} on {}: the run ended and nothing asked the medium to make the \
             guest's writes durable — `rsemu run --drive` exits 0 and the image \
             is missing whatever was still in the host's page cache",
            board.device,
            board.entry.name
        );
    }
}

/// The half that loses data *loudly*, and is worse for it: a device that asks,
/// is refused, and reports success anyway.
#[test]
fn every_board_reports_a_medium_that_could_not_be_made_durable() {
    for board in BOARDS {
        let medium = Arc::new(NoSync::new());
        let machine = realize(board, Arc::clone(&medium) as Arc<dyn Medium>);

        let refused = machine.flush();
        assert!(
            medium.attempts() >= 1,
            "{}: nothing asked the medium at all",
            board.device
        );
        assert!(
            refused.is_err(),
            "{} on {}: the medium refused to make the guest's writes durable and \
             the machine reported success — `rsemu run` exits 0 over a disk that \
             did not get the data",
            board.device,
            board.entry.name
        );
    }
}

/// Not a tautology: [`BOARDS`] is the list this build can ask, and an empty one
/// would make both tests above pass by asking nothing.
#[test]
fn the_table_is_not_empty_in_a_build_that_has_a_storage_board() {
    assert!(!BOARDS.is_empty());
}
