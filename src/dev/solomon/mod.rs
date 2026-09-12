//! Solomon Systech display controllers.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`ssd1306`] | `dev-ssd1306` | the SSD1306/SSD1309 monochrome OLED, and the Sino Wealth SH1106 that every driver treats as one |
//!
//! These are **smart panels**: the picture lives in the controller's own
//! GDDRAM, not in guest memory, which is what [`crate::dev::lcd::panel`] is for.
//!
//! `no_std + alloc`, no dependencies, and nothing here names a colour space or
//! a host facility.

#[cfg(feature = "dev-ssd1306")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-ssd1306")))]
pub mod ssd1306;

use crate::core::error::Result;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if something already claimed a name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    #[cfg(feature = "dev-ssd1306")]
    ssd1306::register(registry)?;
    let _ = registry;
    Ok(())
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    #[cfg(feature = "dev-ssd1306")]
    ssd1306::bind(bindings)?;
    let _ = bindings;
    Ok(())
}

/// What the validator should know about this module's classes.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    #[cfg(feature = "dev-ssd1306")]
    return alloc::vec![ssd1306::schema()];
    #[cfg(not(feature = "dev-ssd1306"))]
    alloc::vec::Vec::new()
}
