//! Sitronix display drivers.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`st7272a`] | `dev-st7272a` | the ST7272A: a 320RGB×240 dual-gate TFT driver configured over 3-wire SPI |
//!
//! `no_std + alloc`, no dependencies, and nothing here names a colour space or
//! a host facility.

#[cfg(feature = "dev-st7272a")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-st7272a")))]
pub mod st7272a;

use crate::core::error::Result;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if something already claimed a name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    #[cfg(feature = "dev-st7272a")]
    st7272a::register(registry)?;
    let _ = registry;
    Ok(())
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    #[cfg(feature = "dev-st7272a")]
    st7272a::bind(bindings)?;
    let _ = bindings;
    Ok(())
}

/// What the validator should know about this module's classes.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![
        #[cfg(feature = "dev-st7272a")]
        st7272a::schema(),
    ]
}
