//! Peers on the Toyota Alphard navigation unit's boards that are not SoC
//! peripherals: the chips the Aisin computer board talks to over its serial
//! lines, modelled as far as the SoC can observe them.
//!
//! | Module | Class | Covers |
//! | --- | --- | --- |
//! | [`psc`] | `navi.psc` | the base board MCU's power-supply-control protocol peer |
//! | [`dcdcad`] | `navi.dcdcad` | the DC-DC/ADC monitor on HSPI channel 0 |
//! | [`kwiboot`] | `navi.kwiboot` | boots the product system from a `LOADING.KWI`, no flash needed |
//!
//! [`kwi`] reads the KWI container `kwiboot` boots from.

use alloc::vec::Vec;

use crate::core::error::Result;
use crate::machine::validate::ClassSchema;

pub mod dcdcad;
pub mod kwi;
pub mod kwiboot;
pub mod psc;

pub use dcdcad::Dcdcad;
pub use kwiboot::KwiBoot;
pub use psc::Psc;

/// Add every class here to a registry.
///
/// # Errors
///
/// If something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    psc::register(registry)?;
    kwiboot::register(registry)?;
    dcdcad::register(registry)
}

/// Bind every class here into the machine graph.
///
/// # Errors
///
/// If one of the names is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    psc::bind(bindings)?;
    kwiboot::bind(bindings)?;
    dcdcad::bind(bindings)
}

/// Every class's validator schema.
#[must_use]
pub fn schemas() -> Vec<ClassSchema> {
    alloc::vec![psc::schema(), dcdcad::schema(), kwiboot::schema()]
}
