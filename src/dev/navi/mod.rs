//! Peers on the Toyota Alphard navigation unit's boards that are not SoC
//! peripherals: the chips the Aisin computer board talks to over its serial
//! lines, modelled as far as the SoC can observe them.
//!
//! | Module | Class | Covers |
//! | --- | --- | --- |
//! | [`psc`] | `navi.psc` | the base board MCU's power-supply-control protocol peer |

use alloc::vec::Vec;

use crate::core::error::Result;
use crate::machine::validate::ClassSchema;

pub mod psc;

pub use psc::Psc;

/// Add every class here to a registry.
///
/// # Errors
///
/// If something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    psc::register(registry)
}

/// Bind every class here into the machine graph.
///
/// # Errors
///
/// If one of the names is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    psc::bind(bindings)
}

/// Every class's validator schema.
#[must_use]
pub fn schemas() -> Vec<ClassSchema> {
    alloc::vec![psc::schema()]
}
