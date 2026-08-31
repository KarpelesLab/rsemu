//! The Apple 1's monitor ROM socket: 256 bytes at `$FF00`.
//!
//! On the board this is two 256x4 bipolar PROMs (A1 and A2) side by side,
//! decoded so that they answer the top page of the address space — which means
//! they hold the 6502's NMI, RESET and IRQ vectors as well as the monitor, and
//! that is why the machine boots at all.
//!
//! # Why this is not a generic `rom` class
//!
//! A generic one belongs beside `ram` in the language's own built-ins
//! ([`machine::builtin`](crate::machine::builtin)) and should be written there
//! when a second machine wants one. This class exists because it knows one
//! thing that a generic one could not: **the socket is 256 bytes**. An image
//! that does not fit is rejected by name here, rather than becoming a machine
//! that boots to a reset vector nobody wrote.

use alloc::boxed::Box;
use alloc::format;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{Region, RegionRef, RomStore, RomWrite};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
const CLASS_NAME: &str = "apple1.rom";

/// How many bytes the socket holds, and how much address space it answers.
pub const ROM_SIZE: u64 = 256;

/// What an unprogrammed cell reads as, and what a short image is padded with.
const ERASED: u8 = 0xff;

/// The monitor ROM, ready to be mapped at `$FF00`.
#[derive(Debug)]
pub struct MonitorRom {
    store: Arc<RomStore>,
    region: RegionRef,
}

impl MonitorRom {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if the `rom` media slot is missing or its image is
    /// empty or larger than the socket.
    pub fn new(props: &Props) -> Result<MonitorRom> {
        let mut r = props.reader();
        let image = r.require_media("rom")?.to_bytes();
        r.finish()?;
        MonitorRom::from_image(&image)
    }

    /// Build one from an image directly.
    ///
    /// A shorter image is padded to the socket's size with `$FF`, an
    /// unprogrammed cell — but the 6502's vectors live in the *last* six bytes
    /// of this page, so a short image is almost always a truncated one and the
    /// machine will not boot from it. It is accepted rather than refused
    /// because a monitor that only uses the low half of the socket is a legal
    /// thing to write.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if the image is empty or does not fit.
    pub fn from_image(image: &[u8]) -> Result<MonitorRom> {
        let len = image.len() as u64;
        if len == 0 || len > ROM_SIZE {
            return Err(Error::Property(format!(
                "property `rom`: the Apple 1's monitor socket holds {ROM_SIZE} bytes, and this \
                 image is {len}"
            )));
        }
        let mut bytes: Vec<u8> = Vec::with_capacity(ROM_SIZE as usize);
        bytes.extend_from_slice(image);
        bytes.resize(ROM_SIZE as usize, ERASED);
        let store = Arc::new(RomStore::new(bytes));
        // A write to ROM is ignored, not a bus error: there is no such line on
        // this board, and a stray `STA $FFxx` on real hardware does nothing.
        let region = Arc::new(Region::rom("monitor", Arc::clone(&store), RomWrite::Ignore));
        Ok(MonitorRom { store, region })
    }

    /// The bytes in the socket.
    #[must_use]
    pub fn bytes(&self) -> &[u8] {
        self.store.as_bytes()
    }

    /// The reset vector the machine will boot to.
    #[must_use]
    pub fn reset_vector(&self) -> u16 {
        let bytes = self.store.as_bytes();
        u16::from(bytes[0xfc]) | (u16::from(bytes[0xfd]) << 8)
    }
}

impl Device for MonitorRom {
    fn class(&self) -> &'static DeviceClass {
        &ROM_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Read-only memory has no state to return to.
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "monitor").then(|| Arc::clone(&self.region))
    }

    // No `save`/`load`: the contents are the media image the caller bound, and
    // a snapshot that carried them would be storing the ROM in every save
    // state (`ROADMAP.md` §4.5).
}

impl Instance for MonitorRom {}

/// The `apple1.rom` device class.
pub static ROM_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "the Apple 1's 256-byte monitor ROM socket, vectors included",
    properties: &[PropertySpec {
        name: "rom",
        kind: ValueKind::Media,
        required: true,
        summary: "the monitor image, as the name of a media slot (`rom = \"rom\"`)",
    }],
    construct: |props| Ok(Box::new(MonitorRom::new(props)?)),
};

/// Add [`ROM_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&ROM_CLASS)
}

/// Bind [`ROM_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(MonitorRom::new(props)?)))
}

/// What the validator should know about `apple1.rom`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("rom", ValueKind::Media).required())
        .region("")
        .region("monitor")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Media;
    use crate::dev::apple1::monitor::RSMON;
    use alloc::string::ToString;

    #[test]
    fn the_shipped_monitor_fills_the_socket_and_boots_to_itself() {
        let rom = MonitorRom::from_image(RSMON).expect("256 bytes exactly");
        assert_eq!(rom.bytes().len(), ROM_SIZE as usize);
        assert_eq!(rom.reset_vector(), 0xff00);
        assert_eq!(rom.region("").expect("mapped").len(), ROM_SIZE);
        assert!(rom.region("monitor").is_some());
        assert!(rom.region("basic").is_none());
    }

    #[test]
    fn a_short_image_is_padded_with_unprogrammed_cells() {
        let rom = MonitorRom::from_image(&[0xea, 0x4c]).expect("short but legal");
        assert_eq!(rom.bytes()[..2], [0xea, 0x4c]);
        assert_eq!(rom.bytes()[2], ERASED);
        assert_eq!(rom.reset_vector(), 0xffff, "and it would not boot");
    }

    #[test]
    fn an_image_that_does_not_fit_the_socket_says_so() {
        let e = MonitorRom::from_image(&[])
            .expect_err("nothing to program")
            .to_string();
        assert!(e.contains("256") && e.contains("0"), "{e}");
        let e = MonitorRom::from_image(&[0u8; 257])
            .expect_err("one byte too many")
            .to_string();
        assert!(e.contains("257"), "{e}");
    }

    #[test]
    fn the_media_slot_carries_the_image() {
        let props = Props::new().with("rom", Media::new("rom", RSMON.to_vec()));
        let device = (ROM_CLASS.construct)(&props).expect("a bound slot");
        assert_eq!(device.class().name, CLASS_NAME);

        // "missing required property" on its own does not tell the reader that
        // a *slot* is what is wanted.
        let e = (ROM_CLASS.construct)(&Props::new())
            .expect_err("no image")
            .to_string();
        assert!(e.contains("rom") && e.contains("media"), "{e}");
    }
}
