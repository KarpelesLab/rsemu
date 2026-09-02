//! virtio: the transport-agnostic core, the MMIO transport, and two devices.
//!
//! `ROADMAP.md` §7 asks for "a transport-agnostic core (virtqueues, feature
//! negotiation) with PCI and MMIO transports", and this is the first half of
//! that: [`queue`] and [`Backend`] know nothing about MMIO, and [`mmio`] knows
//! nothing about block devices. A PCI transport slots in beside `mmio` without
//! either of the two device models changing.
//!
//! | Module | Covers |
//! | --- | --- |
//! | [`queue`] | split virtqueues: descriptor chains, available and used rings |
//! | [`mmio`] | the MMIO transport register block and the status handshake |
//! | [`blk`] | virtio-blk (device ID 2), on the `dev::ata::Medium` seam |
//! | [`rng`] | virtio-rng (device ID 4), deterministically seeded |
//!
//! # Source, and one prohibition
//!
//! Everything here is from *Virtual I/O Device (VIRTIO) Version 1.2*, OASIS
//! Standard — free, complete and normative. `ROADMAP.md` §1 names Linux's
//! virtio **drivers** as the most common way the provenance rule gets broken,
//! precisely because they are the obvious place to look when a device will not
//! probe. No driver source of any licence was opened for any part of this, and
//! the specification answered every question that came up.

pub mod blk;
pub mod mmio;
pub mod queue;
pub mod rng;

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec};
use crate::core::error::Result;
use crate::core::props::{Props, ValueKind};
use crate::core::space::RamStore;
use crate::core::state::{ChunkReader, ChunkWriter};
use crate::dev::ata::Medium;

use queue::{Descriptor, Queue};

pub use blk::VirtioBlk;
pub use mmio::VirtioMmio;
pub use rng::VirtioRng;

/// The `VendorID` register's value: ASCII `rsem`, little-endian.
///
/// The specification does not assign these, and no driver may condition on one
/// (§4.2.2), so it is only ever seen by a human reading a register dump.
pub const VENDOR_ID: u32 = 0x6d65_7372;

/// virtio-blk (§5.2).
pub const DEVICE_ID_BLOCK: u32 = 2;
/// virtio-rng (§5.4).
pub const DEVICE_ID_ENTROPY: u32 = 4;

/// The class name for a block device on the MMIO transport.
pub const BLK_CLASS_NAME: &str = "virtio.blk";
/// The class name for an entropy device on the MMIO transport.
pub const RNG_CLASS_NAME: &str = "virtio.rng";

/// What a virtio device *is*, with no transport in it.
///
/// A transport handles feature negotiation, the status handshake and the
/// rings; a backend answers three questions — what am I, what does my
/// configuration space say, and what do I do with a chain.
pub trait Backend: Send + Sync + fmt::Debug {
    /// The device type (§5): 2 for block, 4 for entropy.
    fn device_id(&self) -> u32;

    /// How many virtqueues it has.
    fn queue_count(&self) -> usize;

    /// The feature bits it offers, *not* including `VIRTIO_F_VERSION_1`, which
    /// every modern transport adds for itself.
    fn features(&self) -> u64 {
        0
    }

    /// Fill `dst` from the device configuration space at `offset`.
    ///
    /// Bytes past the end of the configuration read as zero, because a driver
    /// reads the whole space before it knows which features are on.
    fn config_read(&self, offset: u64, dst: &mut [u8]);

    /// Take a configuration-space write. Most devices have none.
    fn config_write(&self, offset: u64, src: &[u8]) {
        let (_, _) = (offset, src);
    }

    /// Do whatever one descriptor chain asks, returning how many bytes were
    /// written into it — the length that goes in the used ring (§2.7.8).
    fn handle(&self, queue: usize, q: &Queue<'_>, chain: &[Descriptor]) -> u32;

    /// Return to the state a `Status` write of zero implies.
    fn reset(&self);

    /// Serialize whatever of this device is architectural state.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let _ = w;
        Ok(())
    }

    /// Restore what [`save`](Backend::save) wrote.
    ///
    /// # Errors
    ///
    /// Whatever the reader refuses, or a snapshot that does not match this
    /// device's shape.
    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let _ = r;
        Ok(())
    }
}

/// The media slot a `virtio.blk` looks for a host medium under when its
/// machine description names none.
pub const DEFAULT_SLOT: &str = "disk";

/// Build a `virtio.blk` from machine-description properties.
///
/// # Where the bytes come from
///
/// The same two places an [`AtaDisk`](crate::dev::ata::AtaDisk)'s and an NVMe
/// namespace's do, and the machine file names neither of them directly. It
/// names a **media slot** (`image = "disk"`), and the run decides what is
/// behind that name:
///
/// * a [`Medium`] the host installed — what
///   `rsemu run riscv-virt --drive disk=root.qcow2` does — wins, and brings
///   its own capacity, so `size` and the media table are both ignored. A host
///   that named an image file did not also mean "and stamp these bytes over
///   the front of it";
/// * otherwise the media table's bytes, copied into a [`RamStore`] of `size`
///   bytes — or of the image's own length when there is no `size`.
///
/// **Both paths are supported and neither is a degraded version of the
/// other**: the media slot is what keeps this device `no_std` and what a wasm
/// build runs on, and the file is what keeps a 16 GiB disk out of host memory.
///
/// # Errors
///
/// [`Error::Property`](crate::core::Error::Property) if neither `size` nor
/// `image` nor a host medium supplied anything, or if a property this class
/// does not know was given; [`Error::Config`](crate::core::Error::Config) if
/// an installed medium's capacity is not a whole number of 512-byte sectors.
pub fn blk_from_props(props: &Props) -> Result<VirtioMmio> {
    let mut r = props.reader();
    let media = r.optional_media("image")?;
    let slot = media.map(crate::core::props::Media::name);
    let image = media.map(crate::core::props::Media::to_bytes);
    let size = r.or_size("size", 0)?;
    let serial = r.or("serial", String::from("rsemu-virtio"))?;
    let read_only = r.or("readonly", false)?;
    r.finish()?;

    // A medium the *host* installed, under the media slot's name if there is
    // one. It wins over the media table: a run that said
    // `--drive disk=root.qcow2` meant it.
    let supplied = match props.hosts() {
        Some(hosts) => {
            let name = slot.unwrap_or(DEFAULT_SLOT);
            crate::dev::ata::medium::get(hosts, name)?.and_then(|slot| slot.take())
        }
        None => None,
    };
    let bytes = match (&supplied, size, image.as_ref()) {
        (Some(medium), _, _) => medium.capacity(),
        // The larger of the two, which is what `size` has always meant here: a
        // media slot holds the *front* of the disk and `size` pads it out, so
        // an image longer than `size` is a bigger disk rather than an error.
        (None, size, Some(image)) => size.max(image.len() as u64),
        (None, size, None) => size,
    };
    if bytes == 0 {
        return Err(crate::core::Error::Property(String::from(
            "a `virtio.blk` needs a medium: give it a `size` (`size = 16M`) or an `image` \
             media slot, or both to pad an image out to a larger disk — or install one under \
             its media slot with `--drive disk=…`",
        )));
    }
    let media: Arc<dyn Medium> = match supplied {
        Some(medium) => medium,
        None => {
            // Rounded up rather than refused: a media slot holds whatever a
            // front end bound to it, and a short tail there is a ramdisk
            // image's rather than a misconfiguration.
            let bytes = bytes.next_multiple_of(blk::SECTOR_SIZE);
            let store = RamStore::new(bytes);
            if let Some(image) = image {
                RamStore::write_at(&store, 0, &image).map_err(|e| crate::core::Error::Config {
                    at: String::from(BLK_CLASS_NAME),
                    message: alloc::format!("the bound image did not fit: {e}"),
                })?;
            }
            Arc::new(store)
        }
    };

    Ok(VirtioMmio::new(
        Arc::new(VirtioBlk::new(media, serial, read_only)?) as Arc<dyn Backend>,
        &BLK_CLASS,
    ))
}

/// Build a `virtio.rng` from machine-description properties.
///
/// # Errors
///
/// [`Error::Property`](crate::core::Error::Property) if a property this class
/// does not know was given.
pub fn rng_from_props(props: &Props) -> Result<VirtioMmio> {
    let mut r = props.reader();
    let seed = r.or("seed", 0u64)?;
    r.finish()?;
    Ok(VirtioMmio::new(
        Arc::new(VirtioRng::new(seed)) as Arc<dyn Backend>,
        &RNG_CLASS,
    ))
}

/// The `virtio.blk` device class.
pub static BLK_CLASS: DeviceClass = DeviceClass {
    name: BLK_CLASS_NAME,
    version: 1,
    summary: "virtio block device on the MMIO transport, over a `dev::ata::Medium`",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "how large the disk is, as in `size = 16M`; ignored when a host \
                      installed a medium under the media slot",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot the disk is bound to; a host medium under that name \
                      wins, which is what `--drive disk=root.qcow2` installs",
        },
        PropertySpec {
            name: "serial",
            kind: ValueKind::Str,
            required: false,
            summary: "the serial number a `GET_ID` request reports",
        },
        PropertySpec {
            name: "readonly",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether writes are refused (default false)",
        },
    ],
    construct: |props| Ok(Box::new(blk_from_props(props)?) as Box<dyn Device>),
};

/// The `virtio.rng` device class.
pub static RNG_CLASS: DeviceClass = DeviceClass {
    name: RNG_CLASS_NAME,
    version: 1,
    summary: "virtio entropy device on the MMIO transport, deterministically seeded",
    properties: &[PropertySpec {
        name: "seed",
        kind: ValueKind::Uint,
        required: false,
        summary: "the generator's seed; the same seed gives the same bytes every run",
    }],
    construct: |props| Ok(Box::new(rng_from_props(props)?) as Box<dyn Device>),
};

/// Add both virtio classes to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is already claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&BLK_CLASS)?;
    registry.add(&RNG_CLASS)
}

/// Bind both virtio classes into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(BLK_CLASS_NAME, |props| Ok(Arc::new(blk_from_props(props)?)))?;
    bindings.bind(RNG_CLASS_NAME, |props| Ok(Arc::new(rng_from_props(props)?)))
}

/// What the validator should know about the virtio classes.
#[must_use]
pub fn schemas() -> Vec<crate::machine::validate::ClassSchema> {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    alloc::vec![
        ClassSchema::new(BLK_CLASS_NAME)
            .prop(PropSchema::new("size", ValueKind::Size))
            .prop(PropSchema::new("image", ValueKind::Media))
            .prop(PropSchema::new("serial", ValueKind::Str))
            .prop(PropSchema::new("readonly", ValueKind::Bool))
            .region("")
            .region("regs")
            .port("irq", PortDir::Out),
        ClassSchema::new(RNG_CLASS_NAME)
            .prop(PropSchema::new("seed", ValueKind::Uint))
            .region("")
            .region("regs")
            .port("irq", PortDir::Out),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use alloc::string::ToString;

    #[test]
    fn a_block_device_needs_a_medium() {
        let e = blk_from_props(&Props::new())
            .expect_err("no size and no image")
            .to_string();
        assert!(e.contains("medium"), "{e}");
        let disk = blk_from_props(&Props::new().with("size", Value::Size(4096)))
            .expect("a size is enough");
        assert_eq!(disk.backend().device_id(), DEVICE_ID_BLOCK);
    }

    #[test]
    fn a_medium_the_host_installed_wins_over_the_machine_files_size() {
        // `--drive disk=root.qcow2` in the small: the run installs a medium
        // under the slot the machine file names, and it brings its own
        // capacity. A run that named an image file meant it.
        use crate::core::hosts::HostObjects;
        use crate::core::space::RamStore;
        use crate::dev::ata::medium;

        let hosts = alloc::sync::Arc::new(HostObjects::new());
        let store: Arc<dyn Medium> = Arc::new(RamStore::new(8 * 512));
        medium::install(&hosts, "disk", store).expect("nothing else claimed it");

        let props = Props::new()
            .with("size", Value::Size(64 * 1024))
            .with_hosts(hosts);
        let disk = blk_from_props(&props).expect("a medium is enough");
        let backend = disk.backend();
        assert_eq!(backend.device_id(), DEVICE_ID_BLOCK);
        let mut config = [0u8; 8];
        backend.config_read(0, &mut config);
        assert_eq!(
            u64::from_le_bytes(config),
            8,
            "the medium's eight sectors, not the 128 `size` asked for"
        );
    }

    #[test]
    fn an_entropy_device_takes_a_seed_and_nothing_else() {
        let rng = rng_from_props(&Props::new().with("seed", 5u64)).expect("a seed is legal");
        assert_eq!(rng.backend().device_id(), DEVICE_ID_ENTROPY);
        assert!(rng_from_props(&Props::new().with("sed", 5u64)).is_err());
    }

    #[test]
    fn both_classes_register_and_bind() {
        let mut registry = crate::core::Registry::new();
        register(&mut registry).expect("fresh registry");
        assert!(registry.get(BLK_CLASS_NAME).is_some());
        assert!(registry.get(RNG_CLASS_NAME).is_some());
        let mut bindings = crate::machine::Bindings::new();
        bind(&mut bindings).expect("fresh bindings");
        assert_eq!(bindings.len(), 2);
        assert_eq!(schemas().len(), 2);
    }
}
