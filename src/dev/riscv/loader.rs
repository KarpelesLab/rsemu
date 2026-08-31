//! Putting a firmware or kernel image into guest memory.
//!
//! A RISC-V board has no mask ROM holding an operating system: the firmware is
//! whatever the person running the machine points at, and it lives in DRAM
//! before the first instruction executes. Real hardware achieves that with a
//! boot medium and a first-stage loader; an emulator achieves it by writing the
//! bytes in, and pretending otherwise would add a SPI flash model that nothing
//! would ever read twice.
//!
//! So this is a device with no registers. It publishes no region, drives no
//! wire and answers no access. What it does is **write its image into the
//! address space at reset**, which is the moment the machine says "everything
//! is back to how it starts".
//!
//! # Why reset and not realize
//!
//! A cold reset clears RAM (`machine::builtin`'s `ram` does exactly that), so
//! an image written at realize would be erased by the reset that ends realize.
//! `Machine::reset` runs devices in **declaration order**, so a loader
//! declared after the memory it writes into is guaranteed to run after that
//! memory has been cleared — which is the framework's documented way of saying
//! "reset me last", and why `machines/riscv-virt.machine` puts the loaders at
//! the bottom.
//!
//! # An empty image is not an error
//!
//! Binding zero bytes loads nothing. That is what lets one machine file carry a
//! kernel slot that a bare-metal run simply does not fill.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AddressSpace, MemAttrs, RequesterId};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "riscv.loader";

/// The address an image lands at when a machine file does not say.
pub const DEFAULT_ADDR: u64 = 0x8000_0000;

/// What the loader learned when the machine bound it.
#[derive(Debug, Default)]
struct Binding {
    space: Option<Arc<AddressSpace>>,
    requester: RequesterId,
    /// What the last load failed with, if it did. A `reset` cannot return an
    /// error, so the failure is kept here and surfaced by
    /// [`Loader::last_error`] — silence would give a machine that boots to
    /// zeroed memory with no explanation.
    error: Option<String>,
    /// How many times the image has been written in, for tests.
    loads: u64,
}

/// An image and the address it is written to.
#[derive(Debug)]
pub struct Loader {
    image: Arc<[u8]>,
    addr: u64,
    space_name: Option<String>,
    binding: Mutex<Binding>,
}

impl Loader {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if the `image` media slot is missing, or if a
    /// property this class does not know was given.
    pub fn new(props: &Props) -> Result<Loader> {
        let mut r = props.reader();
        let image = r.require_media("image")?.to_bytes();
        let addr = r.or_addr("addr", DEFAULT_ADDR)?;
        let space = r.optional_str("space")?.map(ToString::to_string);
        r.finish()?;
        Ok(Loader::from_image(image, addr, space))
    }

    /// Build one from bytes directly.
    #[must_use]
    pub fn from_image(image: impl Into<Arc<[u8]>>, addr: u64, space: Option<String>) -> Loader {
        Loader {
            image: image.into(),
            addr,
            space_name: space,
            binding: Mutex::with_rank(LockRank::DEVICE, Binding::default()),
        }
    }

    /// Where the image is written.
    #[must_use]
    pub fn addr(&self) -> u64 {
        self.addr
    }

    /// How many bytes it writes.
    #[must_use]
    pub fn len(&self) -> u64 {
        self.image.len() as u64
    }

    /// Whether there is nothing to write.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.image.is_empty()
    }

    /// What the last load failed with, if it did.
    #[must_use]
    pub fn last_error(&self) -> Option<String> {
        self.binding.lock().error.clone()
    }

    /// How many times the image has been written into memory.
    #[must_use]
    pub fn loads(&self) -> u64 {
        self.binding.lock().loads
    }

    /// Write the image into `space` now.
    ///
    /// # Errors
    ///
    /// Whatever the address space refuses — which for a firmware image almost
    /// always means the machine has less RAM than the image needs, or the
    /// image was aimed at an address nothing answers.
    pub fn load_into(&self, space: &AddressSpace, requester: RequesterId) -> Result<()> {
        if self.image.is_empty() {
            return Ok(());
        }
        let attrs = MemAttrs::DEFAULT
            .with_requester(requester)
            .with_privileged(true);
        // In page-sized pieces, so a failure names the address it happened at
        // rather than the start of a 30 MiB kernel.
        const CHUNK: usize = 4096;
        let mut at = self.addr;
        for piece in self.image.chunks(CHUNK) {
            space
                .write_bytes(at, piece, attrs)
                .map_err(|e| Error::Config {
                    at: CLASS_NAME.to_string(),
                    message: format!(
                        "cannot write the image at {at:#x} in space `{}`: {e} — the machine \
                         needs {} byte(s) of memory from {:#x}",
                        space.name(),
                        self.image.len(),
                        self.addr
                    ),
                })?;
            at += piece.len() as u64;
        }
        Ok(())
    }
}

/// The `riscv.loader` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "writes a media image into guest memory at reset: firmware, a kernel, a ramdisk",
    properties: &[
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: true,
            summary: "the image, as the name of a media slot (`image = \"firmware\"`)",
        },
        PropertySpec {
            name: "addr",
            kind: ValueKind::Addr,
            required: false,
            summary: "where the first byte lands (default 0x80000000)",
        },
        PropertySpec {
            name: "space",
            kind: ValueKind::Str,
            required: false,
            summary: "which address space to write into, if not the one the object declares",
        },
    ],
    construct: |props| Ok(Box::new(Loader::new(props)?)),
};

impl Device for Loader {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the image goes in at reset, once the memory it
        // lands in has been cleared. See the module docs.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        let (space, requester) = {
            let binding = self.binding.lock();
            (binding.space.clone(), binding.requester)
        };
        let Some(space) = space else {
            return;
        };
        let result = self.load_into(&space, requester);
        let mut binding = self.binding.lock();
        binding.loads += 1;
        binding.error = result.err().map(|e| e.to_string());
    }

    // No `save`/`load`: the image is the media the caller bound, and a snapshot
    // that carried it would store the firmware in every save state
    // (`ROADMAP.md` §4.5).
}

impl Instance for Loader {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = match &self.space_name {
            Some(name) => ctx.space_named(name).ok_or_else(|| Error::Config {
                at: ctx.path().to_string(),
                message: format!("no address space named `{name}`"),
            })?,
            None => ctx.space().ok_or_else(|| Error::Config {
                at: ctx.path().to_string(),
                message: String::from(
                    "a loader needs an address space to write into (`space = mem`)",
                ),
            })?,
        };
        // Fail here rather than at reset, where nothing could report it: a
        // machine whose firmware does not fit should not build at all.
        self.load_into(space, ctx.requester())?;
        let mut binding = self.binding.lock();
        binding.space = Some(Arc::clone(space));
        binding.requester = ctx.requester();
        Ok(())
    }
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Loader::new(props)?)))
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// What the validator should know about `riscv.loader`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("image", ValueKind::Media).required())
        .prop(PropSchema::new("addr", ValueKind::Addr))
        .prop(PropSchema::new("space", ValueKind::Str))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{RamStore, Region};
    use crate::core::value::Width;

    fn space_with_ram(len: u64) -> Arc<AddressSpace> {
        let space = AddressSpace::new("mem", 64);
        let ram = Arc::new(RamStore::new(len));
        space
            .topology()
            .map(Region::ram("ram", ram), DEFAULT_ADDR)
            .expect("a fresh space");
        Arc::new(space)
    }

    fn peek(space: &AddressSpace, addr: u64) -> u8 {
        space
            .read(addr, Width::U8, MemAttrs::DEBUG)
            .expect("mapped") as u8
    }

    #[test]
    fn the_image_lands_where_it_was_aimed() {
        let space = space_with_ram(0x1000);
        let loader = Loader::from_image(&b"\x01\x02\x03"[..], DEFAULT_ADDR, None);
        loader
            .load_into(&space, RequesterId::ANONYMOUS)
            .expect("three bytes fit");
        assert_eq!(peek(&space, DEFAULT_ADDR), 1);
        assert_eq!(peek(&space, DEFAULT_ADDR + 2), 3);
        assert_eq!(loader.len(), 3);
    }

    #[test]
    fn an_image_that_does_not_fit_says_where_and_how_big() {
        let space = space_with_ram(0x100);
        let loader = Loader::from_image(alloc::vec![0xaau8; 0x400], DEFAULT_ADDR, None);
        let e = loader
            .load_into(&space, RequesterId::ANONYMOUS)
            .expect_err("1 KiB into 256 bytes")
            .to_string();
        assert!(e.contains("1024"), "{e}");
        assert!(e.contains("80000000"), "{e}");
    }

    #[test]
    fn an_empty_image_loads_nothing_and_is_not_an_error() {
        let space = space_with_ram(0x100);
        let loader = Loader::from_image(&[][..], DEFAULT_ADDR, None);
        assert!(loader.is_empty());
        loader
            .load_into(&space, RequesterId::ANONYMOUS)
            .expect("nothing to do");
        assert_eq!(peek(&space, DEFAULT_ADDR), 0);
    }

    #[test]
    fn a_reset_writes_the_image_in_again() {
        // The whole reason the load happens at reset: a cold reset zeroes RAM,
        // and a machine that came back up with no firmware would be a puzzle.
        let space = space_with_ram(0x1000);
        let loader = Loader::from_image(&b"\xde\xad"[..], DEFAULT_ADDR, None);
        loader.binding.lock().space = Some(Arc::clone(&space));
        space
            .write(DEFAULT_ADDR, Width::U8, 0, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(peek(&space, DEFAULT_ADDR), 0);
        loader.reset(ResetKind::Cold);
        assert_eq!(peek(&space, DEFAULT_ADDR), 0xde);
        assert_eq!(loader.loads(), 1);
        assert_eq!(loader.last_error(), None);
    }

    #[test]
    fn a_loader_with_no_space_yet_resets_without_complaint() {
        // Reset runs on every device, including one whose machine failed to
        // bind it; it must not panic.
        let loader = Loader::from_image(&b"x"[..], DEFAULT_ADDR, None);
        loader.reset(ResetKind::Cold);
        assert_eq!(loader.loads(), 0);
    }

    #[test]
    fn a_media_slot_is_required() {
        let e = Loader::new(&Props::new())
            .expect_err("no image")
            .to_string();
        assert!(e.contains("image") && e.contains("media"), "{e}");
    }
}
