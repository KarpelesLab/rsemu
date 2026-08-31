//! An Intel 8042 keyboard controller.
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
pub const CLASS_NAME: &str = "pc.kbc";

/// How much address space the register block answers.
pub const REGISTER_WINDOW_LEN: u64 = 1;

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

/// An Intel 8042 keyboard controller.
#[derive(Debug)]
pub struct Kbc8042 {
    region: RegionRef,
}

impl Kbc8042 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`](crate::core::Error::Property) if a property this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Kbc8042> {
        props.reader().finish()?;
        Ok(Kbc8042::default_device())
    }

    /// One with no properties set.
    #[must_use]
    pub fn default_device() -> Kbc8042 {
        let regs = Arc::new(Registers);
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            regs as Arc<dyn MemOps>,
        ));
        Kbc8042 { region }
    }
}

/// The `pc.kbc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "Intel 8042 keyboard controller, with the A20 gate",
    properties: &[],
    construct: |props| Ok(Box::new(Kbc8042::new(props)?)),
};

impl Device for Kbc8042 {
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

impl Instance for Kbc8042 {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Kbc8042::new(props)?)))
}

/// What the validator should know about `pc.kbc`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME).region("").region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_unpopulated_bus_reads_as_ones() {
        let dev = Kbc8042::default_device();
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
