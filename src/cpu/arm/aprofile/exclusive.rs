//! `arm.exclusive`: the global exclusive monitor several cores share.
//!
//! Not a device a guest can see — it has no registers — but the thing that
//! makes `LDREX`/`STREX` mean the same on four cores as on one. A machine
//! file names one object and points every core at it:
//!
//! ```text
//! object excl "arm.exclusive" { cores = 4 }
//! object cpu0 "cpu.arm" { …, monitor = excl }
//! object cpu1 "cpu.arm" { …, monitor = excl }
//! ```
//!
//! It is published as [`ExportId::EXCLUSIVE_MONITOR`] and attached at bind
//! time. See [`monitor`](super::monitor) for what it decides.

use alloc::boxed::Box;
use alloc::sync::Arc;

use super::monitor::SharedMonitor;
use crate::core::device::{
    Device, DeviceClass, Export, ExportId, PropertySpec, RealizeCtx, ResetKind,
};
use crate::core::error::Result;
use crate::core::props::{Props, ValueKind};
use crate::core::state::{ChunkReader, ChunkWriter};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "arm.exclusive";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The shared monitor, as a machine object.
#[derive(Debug)]
pub struct Exclusive {
    monitor: Arc<SharedMonitor>,
}

impl Exclusive {
    /// Build one with room for `cores` cores (default 8).
    ///
    /// # Errors
    ///
    /// [`Error::Property`](crate::core::error::Error::Property) for a bad
    /// count or a property it does not take.
    pub fn new(props: &Props) -> Result<Exclusive> {
        let mut r = props.reader();
        let cores = r.or_range("cores", 8u64, 1..=32)? as usize;
        r.finish()?;
        Ok(Exclusive {
            monitor: Arc::new(SharedMonitor::new(cores)),
        })
    }

    /// The monitor itself.
    #[must_use]
    pub fn monitor(&self) -> &Arc<SharedMonitor> {
        &self.monitor
    }
}

/// The class descriptor.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the global exclusive monitor the cores of a multi-core ARM machine share",
    properties: &[PropertySpec {
        name: "cores",
        kind: ValueKind::Uint,
        required: false,
        summary: "how many cores it tracks marks for (default 8)",
    }],
    construct: |props| Ok(Box::new(Exclusive::new(props)?)),
};

impl Device for Exclusive {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        self.monitor.clear();
    }

    fn export(&self, which: ExportId) -> Option<Export> {
        (which == ExportId::EXCLUSIVE_MONITOR).then(|| {
            Export::Opaque(Arc::clone(&self.monitor) as Arc<dyn core::any::Any + Send + Sync>)
        })
    }

    // The marks are not saved: a snapshot restores every core's *local*
    // monitor, and a store-exclusive that finds the global mark gone simply
    // fails and retries — always a permitted outcome (DDI 0406C A3.4.5).
    fn save(&self, _w: &mut ChunkWriter<'_>) -> Result<()> {
        Ok(())
    }

    fn load(&self, _r: &mut ChunkReader<'_>) -> Result<()> {
        self.monitor.clear();
        Ok(())
    }
}

impl Instance for Exclusive {}

/// Add the class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind the class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Exclusive::new(props)?)))
}

/// The validator schema.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME).prop(PropSchema::new("cores", ValueKind::Uint).range(1, 32))
}
