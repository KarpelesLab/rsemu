//! Device classes the description language ships with.
//!
//! Almost every device is a feature-gated model under [`dev`](crate::dev) or
//! [`cpu`](crate::cpu). A handful are not, because they are not models of
//! anything: `ram` is a block of bytes with an address, and every machine in
//! every catalog needs one. `ROADMAP.md` §5's own worked example opens with
//! `object wram "ram" { size = 2K }`, so a build that can parse that example
//! and then cannot realize it has a hole in it.
//!
//! `rom` is the same argument for read-only memory. Boards that ship one today
//! each carry their own class — `beneater.rom` under `dev-wdc`, `riscv.loader`
//! under `dev-riscv` — because each does something board-specific with the
//! image. A board that wants nothing board-specific, just "these bytes, at this
//! address, that the guest cannot write", had to invent a class to say so, and
//! that is ceremony of exactly the kind this module exists to avoid.
//!
//! These live here rather than in `dev/` for two reasons: they are always
//! compiled, like the language itself, and they are described entirely by
//! `core::space` — no device model is involved, and inventing a
//! `dev-memory` feature to hold one region would be ceremony.

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::registry::Registry;
use crate::core::space::{RamStore, Region, RegionRef, RomStore, RomWrite};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::machine::realize::{Bindings, Instance};
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine file writes.
const RAM_CLASS_NAME: &str = "ram";

/// And the one for read-only memory.
const ROM_CLASS_NAME: &str = "rom";

/// Read/write memory: `size` bytes, mapped wherever a `map` statement puts it.
///
/// Not a device model — there is no chip called "RAM" — but it is a device as
/// far as the machine graph is concerned: it publishes one region, it is
/// snapshotted, and a cold reset zeroes it.
#[derive(Debug)]
pub struct Ram {
    store: Arc<RamStore>,
    region: RegionRef,
}

impl Ram {
    /// Allocate `size` bytes.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `size` is missing, is not a byte count, or if a
    /// property nothing here accepts was given.
    pub fn new(props: &Props) -> Result<Ram> {
        let mut r = props.reader();
        let size = r.require_size("size")?;
        r.finish()?;
        if size == 0 {
            return Err(Error::Property(alloc::string::String::from(
                "property `size`: a ram object with no bytes in it cannot be mapped",
            )));
        }
        let store = Arc::new(RamStore::new(size));
        let region = Arc::new(Region::ram("ram", Arc::clone(&store)));
        Ok(Ram { store, region })
    }

    /// The backing store, for a test or a debugger that wants the bytes.
    #[must_use]
    pub fn store(&self) -> &Arc<RamStore> {
        &self.store
    }

    /// How many bytes it holds.
    #[must_use]
    pub fn len(&self) -> u64 {
        self.store.len()
    }

    /// Whether it holds none — it never does; `new` refuses a zero size.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.store.len() == 0
    }
}

impl Device for Ram {
    fn class(&self) -> &'static DeviceClass {
        &RAM_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region, and the
        // realizer does that after every device has realized.
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        // Power clears memory; a reset line does not. That is what makes a
        // "did we come from power-on?" check in a guest's reset handler work.
        if kind == ResetKind::Cold {
            let _ = self.store.fill(0, self.store.len(), 0);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let len = usize::try_from(self.store.len())
            .map_err(|_| Error::State(alloc::string::String::from("ram larger than this host")))?;
        let mut bytes = alloc::vec![0u8; len];
        self.store.read_at(0, &mut bytes)?;
        w.write_bytes(&bytes)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bytes: &[u8] = r.read_bytes()?;
        if bytes.len() as u64 != self.store.len() {
            return Err(Error::State(alloc::format!(
                "snapshot has {} byte(s) of ram, this object has {}",
                bytes.len(),
                self.store.len()
            )));
        }
        self.store.write_at(0, bytes)?;
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        // Only the whole thing: `map cpubus 0 size 2K = wram`, or
        // `mirror(wram)`. A named sub-window is what `alias()` is for.
        name.is_empty().then(|| Arc::clone(&self.region))
    }
}

impl Instance for Ram {}

// ---------------------------------------------------------------------------
// rom
// ---------------------------------------------------------------------------

/// Read-only memory: `size` bytes, optionally initialised from a media slot.
///
/// The generic counterpart to [`Ram`], and the reason a board with a firmware
/// image no longer needs a class of its own. `image` names a media slot the
/// caller binds (`--media firmware=fw.bin`), exactly as `rom = "cart"` does for
/// a NES cartridge; an unbound `image` is legal and leaves the ROM zeroed,
/// which is a board whose socket is empty.
///
/// A write is **dropped**, not faulted: that is what a ROM on a bus does, and a
/// guest that stores to one gets no bus error from real silicon either. A board
/// that wants a fault maps it into a space whose `unassigned` policy says so.
#[derive(Debug)]
pub struct Rom {
    store: Arc<RomStore>,
    region: RegionRef,
}

impl Rom {
    /// Allocate `size` bytes and copy in whatever `image` was bound to.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `size` is missing or a property nothing here
    /// accepts was given; [`Error::Config`] if the bound image does not fit.
    pub fn new(props: &Props) -> Result<Rom> {
        let mut r = props.reader();
        let size = r.require_size("size")?;
        let image = r
            .optional_media("image")?
            .map(crate::core::props::Media::to_bytes);
        r.finish()?;
        if size == 0 {
            return Err(Error::Property(alloc::string::String::from(
                "property `size`: a rom object with no bytes in it cannot be mapped",
            )));
        }
        let mut bytes = alloc::vec![0u8; usize::try_from(size).map_err(|_| {
            Error::Property(alloc::string::String::from(
                "property `size`: a rom larger than this host's address space",
            ))
        })?];
        if let Some(image) = image {
            if image.len() as u64 > size {
                return Err(Error::Config {
                    at: alloc::string::String::from(ROM_CLASS_NAME),
                    message: alloc::format!(
                        "the bound image is {} byte(s) and the rom is {size}",
                        image.len()
                    ),
                });
            }
            bytes[..image.len()].copy_from_slice(&image);
        }
        let store = Arc::new(RomStore::new(bytes));
        // `RomWrite::Ignore`: a store to a ROM is dropped on the bus, which is
        // what the hardware does and what a guest's own diagnostics expect.
        let region = Arc::new(Region::rom("rom", Arc::clone(&store), RomWrite::Ignore));
        Ok(Rom { store, region })
    }

    /// The backing store, for a test or a debugger that wants the bytes.
    #[must_use]
    pub fn store(&self) -> &Arc<RomStore> {
        &self.store
    }

    /// How many bytes it holds.
    #[must_use]
    pub fn len(&self) -> u64 {
        self.store.len()
    }

    /// Whether it holds none — it never does; `new` refuses a zero size.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.store.len() == 0
    }
}

impl Device for Rom {
    fn class(&self) -> &'static DeviceClass {
        &ROM_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Nothing to do: the contents never change.
    }

    // No `save`/`load`, deliberately. The contents are the image the caller
    // bound, and a snapshot that carried them would store the ROM in every save
    // state (`ROADMAP.md` §4.5) — the same reasoning `dev::wdc::rom` gives.

    fn region(&self, name: &str) -> Option<RegionRef> {
        name.is_empty().then(|| Arc::clone(&self.region))
    }
}

impl Instance for Rom {}

/// The `rom` device class.
pub static ROM_CLASS: DeviceClass = DeviceClass {
    name: ROM_CLASS_NAME,
    version: 1,
    summary: "read-only memory: a block of bytes from a media slot, writes dropped",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: true,
            summary: "how many bytes, as in `size = 64K`",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot holding the contents, as in `image = \"firmware\"`",
        },
    ],
    construct: |props| Ok(Box::new(Rom::new(props)?)),
};

/// What the validator should know about `rom`.
#[must_use]
pub fn rom_schema() -> ClassSchema {
    ClassSchema::new(ROM_CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size).required())
        .prop(PropSchema::new("image", ValueKind::Media))
        .region("")
}

/// The `ram` device class.
pub static RAM_CLASS: DeviceClass = DeviceClass {
    name: RAM_CLASS_NAME,
    version: 1,
    summary: "read/write memory: a block of bytes a `map` statement places",
    properties: &[PropertySpec {
        name: "size",
        kind: ValueKind::Size,
        required: true,
        summary: "how many bytes, as in `size = 2K`",
    }],
    construct: |props| Ok(Box::new(Ram::new(props)?)),
};

/// What the validator should know about `ram`.
#[must_use]
pub fn ram_schema() -> ClassSchema {
    ClassSchema::new(RAM_CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size).required())
        .region("")
}

/// Add every built-in class to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed one of the names.
pub fn register(registry: &mut Registry) -> Result<()> {
    registry.add(&RAM_CLASS)?;
    registry.add(&ROM_CLASS)?;
    super::combinator::register(registry)
}

/// Bind every built-in class into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if a name is already bound.
pub fn bind(bindings: &mut Bindings) -> Result<()> {
    bindings.bind(RAM_CLASS_NAME, |props| Ok(Arc::new(Ram::new(props)?)))?;
    bindings.bind(ROM_CLASS_NAME, |props| Ok(Arc::new(Rom::new(props)?)))?;
    super::combinator::bind(bindings)
}

/// Every built-in class's validator schema, in registration order.
#[must_use]
pub fn schemas() -> Vec<ClassSchema> {
    let mut out = alloc::vec![ram_schema(), rom_schema()];
    out.extend(super::combinator::schemas());
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Value;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use alloc::string::ToString;

    fn ram(size: u64) -> Ram {
        Ram::new(&Props::new().with("size", Value::Size(size))).expect("a size is all it takes")
    }

    #[test]
    fn a_size_is_required_and_must_be_usable() {
        assert!(Ram::new(&Props::new()).is_err(), "no size at all");
        let e = Ram::new(&Props::new().with("size", Value::Size(0)))
            .expect_err("zero bytes")
            .to_string();
        assert!(e.contains("no bytes"), "{e}");
        // A typo'd property is an afternoon lost if it is silently ignored.
        let props = Props::new().with("size", Value::Size(16)).with("sze", 1u64);
        assert!(Ram::new(&props).is_err(), "unknown property");
    }

    #[test]
    fn the_whole_block_is_the_only_region() {
        let wram = ram(2048);
        assert_eq!(wram.len(), 2048);
        assert!(!wram.is_empty());
        assert!(wram.region("").is_some());
        assert!(wram.region("bank0").is_none());
    }

    #[test]
    fn power_clears_memory_and_a_reset_line_does_not() {
        let wram = ram(64);
        wram.store().write_u8(3, 0xa5).unwrap();
        wram.reset(ResetKind::Warm);
        assert_eq!(wram.store().read_u8(3).unwrap(), 0xa5);
        wram.reset(ResetKind::Cold);
        assert_eq!(wram.store().read_u8(3).unwrap(), 0);
    }

    #[test]
    fn a_snapshot_round_trips_to_identical_bytes() {
        let saved = ram(256);
        for i in 0..256u64 {
            saved.store().write_u8(i, (i as u8) ^ 0x5a).unwrap();
        }

        let mut shape = MachineShape::new();
        shape.add_device("wram", RAM_CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("wram", RAM_CLASS.name, RAM_CLASS.version).unwrap();
            saved.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = ram(256);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load(
                "wram",
                RAM_CLASS.name,
                RAM_CLASS.version,
                &Migrations::new(),
            )
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();
        for i in 0..256u64 {
            assert_eq!(restored.store().read_u8(i).unwrap(), (i as u8) ^ 0x5a);
        }
    }

    #[test]
    fn a_snapshot_from_a_differently_sized_object_is_refused() {
        let big = ram(256);
        let mut shape = MachineShape::new();
        shape.add_device("wram", RAM_CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("wram", RAM_CLASS.name, RAM_CLASS.version).unwrap();
            big.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let small = ram(128);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load(
                "wram",
                RAM_CLASS.name,
                RAM_CLASS.version,
                &Migrations::new(),
            )
            .unwrap();
        let e = small
            .load(&mut chunk.reader())
            .expect_err("128 is not 256")
            .to_string();
        assert!(e.contains("256") && e.contains("128"), "{e}");
    }
}
