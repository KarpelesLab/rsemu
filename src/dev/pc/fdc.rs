//! A NEC uPD765 floppy disk controller.
//!
//! PLACEHOLDER. The register block answers with the value an unpopulated bus
//! returns and remembers nothing. It exists so that the machine file, the
//! catalog and the rest of the chipset compile while this chip is written.

use alloc::boxed::Box;
use alloc::sync::Arc;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Result};
use crate::core::props::Props;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::value::{Endian, Width};
use crate::machine::realize::Instance;
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.fdc";

/// How much address space the register block answers.
pub const REGISTER_WINDOW_LEN: u64 = 8;

/// The register block.
#[derive(Debug, Default)]
struct Registers;

impl MemOps for Registers {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        dst.fill(0xff);
        Ok(())
    }

    fn write(&self, _offset: u64, _src: &[u8], _attrs: MemAttrs) -> MemResult {
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

/// A NEC uPD765 floppy disk controller.
#[derive(Debug)]
pub struct Fdc765 {
    region: RegionRef,
}

impl Fdc765 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`](crate::core::Error::Property) if a property this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Fdc765> {
        props.reader().finish()?;
        Ok(Fdc765::default_device())
    }

    /// One with no properties set.
    #[must_use]
    pub fn default_device() -> Fdc765 {
        let regs = Arc::new(Registers);
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            regs as Arc<dyn MemOps>,
        ));
        Fdc765 { region }
    }
}

/// The `pc.fdc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "NEC uPD765 floppy disk controller",
    properties: &[],
    construct: |props| Ok(Box::new(Fdc765::new(props)?)),
};

impl Device for Fdc765 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {}

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }
}

impl Instance for Fdc765 {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Fdc765::new(props)?)))
}

/// What the validator should know about `pc.fdc`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME).region("").region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_unpopulated_bus_reads_as_ones() {
        let dev = Fdc765::default_device();
        let mut byte = [0u8; 1];
        Registers
            .read(0, &mut byte, MemAttrs::DEFAULT)
            .expect("a byte read is legal");
        assert_eq!(byte[0], 0xff);
        assert!(dev.region("").is_some());
    }
}

// Silence the unused import while this is a placeholder.
#[allow(unused_imports)]
use BusError as _BusError;
