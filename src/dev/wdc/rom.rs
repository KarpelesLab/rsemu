//! The board's EEPROM socket: 32 KiB at `$8000`.
//!
//! On Ben Eater's breadboard this is one AT28C256, a 32 KiB parallel EEPROM,
//! whose `/CE` is driven straight from A15 — so it answers the whole top half
//! of the address space and holds the 6502's NMI, RESET and IRQ vectors as
//! well as the program. That is why the machine boots at all.
//!
//! # Why this is not a WDC part, and why it lives here anyway
//!
//! It is not a 65xx peripheral and this module is named for those. It is here
//! because a board needs a ROM socket and the description language has no
//! generic `rom` built-in yet: `machine::builtin` ships `ram` and nothing else.
//! When a second board wants one, a generic `rom` class belongs beside `ram`
//! there and this file collapses into a size property — the same argument
//! [`apple1::rom`](crate::dev::apple1::rom) makes for its own 256-byte socket.
//!
//! What a generic one could not know is the thing this class exists for: **the
//! socket is 32 KiB**. An image that does not fit is rejected by name here,
//! rather than becoming a machine that boots to a reset vector nobody wrote.

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
const CLASS_NAME: &str = "beneater.rom";

/// How many bytes the socket holds, and how much address space it answers.
pub const ROM_SIZE: u64 = 32 * 1024;

/// Where the socket is decoded: A15 high, and nothing else.
pub const ROM_BASE: u64 = 0x8000;

/// What an unprogrammed cell reads as, and what a short image is padded with.
const ERASED: u8 = 0xff;

/// The program EEPROM, ready to be mapped at `$8000`.
#[derive(Debug)]
pub struct ProgramRom {
    store: Arc<RomStore>,
    region: RegionRef,
}

impl ProgramRom {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if the `rom` media slot is missing or its image is
    /// empty or larger than the socket.
    pub fn new(props: &Props) -> Result<ProgramRom> {
        let mut r = props.reader();
        let image = r.require_media("rom")?.to_bytes();
        r.finish()?;
        ProgramRom::from_image(&image)
    }

    /// Build one from an image directly.
    ///
    /// A shorter image is padded to the socket's size with `$FF`, an
    /// unprogrammed cell — but the 6502's vectors live in the *last* six bytes
    /// of the socket, so a short image will not boot. It is accepted rather
    /// than refused because an assembler that emits only as far as its last
    /// `.byte` and leaves the vectors to a separate pass is an ordinary thing
    /// to have.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if the image is empty or does not fit.
    pub fn from_image(image: &[u8]) -> Result<ProgramRom> {
        let len = image.len() as u64;
        if len == 0 || len > ROM_SIZE {
            return Err(Error::Property(format!(
                "property `rom`: this board's EEPROM socket holds {ROM_SIZE} bytes, and this \
                 image is {len}"
            )));
        }
        let mut bytes: Vec<u8> = Vec::with_capacity(ROM_SIZE as usize);
        bytes.extend_from_slice(image);
        bytes.resize(ROM_SIZE as usize, ERASED);
        let store = Arc::new(RomStore::new(bytes));
        // A write to ROM is ignored, not a bus error: the EEPROM's /WE is tied
        // high on this board, so a stray `STA $9000` does nothing at all.
        let region = Arc::new(Region::rom("program", Arc::clone(&store), RomWrite::Ignore));
        Ok(ProgramRom { store, region })
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
        u16::from(bytes[0x7ffc]) | (u16::from(bytes[0x7ffd]) << 8)
    }
}

impl Device for ProgramRom {
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
        matches!(name, "" | "program").then(|| Arc::clone(&self.region))
    }

    // No `save`/`load`: the contents are the media image the caller bound, and
    // a snapshot that carried them would be storing the ROM in every save
    // state (`ROADMAP.md` §4.5).
}

impl Instance for ProgramRom {}

/// The `beneater.rom` device class.
pub static ROM_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "a 32 KiB program EEPROM at $8000, the 6502's vectors included",
    properties: &[PropertySpec {
        name: "rom",
        kind: ValueKind::Media,
        required: true,
        summary: "the program image, as the name of a media slot (`rom = \"rom\"`)",
    }],
    construct: |props| Ok(Box::new(ProgramRom::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(ProgramRom::new(props)?)))
}

/// What the validator should know about `beneater.rom`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("rom", ValueKind::Media).required())
        .region("")
        .region("program")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::props::Media;
    use crate::dev::wdc::monitor::RSMON_IMAGE;
    use alloc::string::ToString;

    #[test]
    fn the_shipped_monitor_fills_the_socket_and_boots_to_itself() {
        let rom = ProgramRom::from_image(RSMON_IMAGE).expect("32 KiB exactly");
        assert_eq!(rom.bytes().len(), ROM_SIZE as usize);
        assert_eq!(u64::from(rom.reset_vector()), ROM_BASE);
        assert_eq!(rom.region("").expect("mapped").len(), ROM_SIZE);
        assert!(rom.region("program").is_some());
        assert!(rom.region("basic").is_none());
    }

    #[test]
    fn a_short_image_is_padded_with_unprogrammed_cells() {
        let rom = ProgramRom::from_image(&[0xea, 0x4c]).expect("short but legal");
        assert_eq!(rom.bytes()[..2], [0xea, 0x4c]);
        assert_eq!(rom.bytes()[2], ERASED);
        assert_eq!(rom.reset_vector(), 0xffff, "and it would not boot");
    }

    #[test]
    fn an_image_that_does_not_fit_the_socket_says_so() {
        let e = ProgramRom::from_image(&[])
            .expect_err("nothing to program")
            .to_string();
        assert!(e.contains("32768") && e.contains(" 0"), "{e}");
        let e = ProgramRom::from_image(&[0u8; 32769])
            .expect_err("one byte too many")
            .to_string();
        assert!(e.contains("32769"), "{e}");
    }

    #[test]
    fn the_media_slot_carries_the_image() {
        let props = Props::new().with("rom", Media::new("rom", RSMON_IMAGE.to_vec()));
        let device = (ROM_CLASS.construct)(&props).expect("a bound slot");
        assert_eq!(device.class().name, CLASS_NAME);

        let e = (ROM_CLASS.construct)(&Props::new())
            .expect_err("no image")
            .to_string();
        assert!(e.contains("rom") && e.contains("media"), "{e}");
    }
}
