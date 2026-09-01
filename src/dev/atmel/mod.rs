//! Atmel (now Microchip) parts.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`at24c`] | `dev-at24c` | the AT24C01D/02D I²C serial EEPROM |
//!
//! `no_std + alloc`, no dependencies.

#[cfg(feature = "dev-at24c")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-at24c")))]
pub mod at24c;

use crate::core::error::Result;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    #[cfg(feature = "dev-at24c")]
    at24c::register(registry)?;
    let _ = registry;
    Ok(())
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    #[cfg(feature = "dev-at24c")]
    at24c::bind(bindings)?;
    let _ = bindings;
    Ok(())
}

/// What the validator should know about every class here.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    #[cfg(feature = "dev-at24c")]
    return alloc::vec![at24c::schema()];
    #[cfg(not(feature = "dev-at24c"))]
    alloc::vec::Vec::new()
}
