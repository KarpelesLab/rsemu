//! Sitronix display drivers.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`st7272a`] | `dev-st7272a` | the ST7272A: a 320RGB×240 dual-gate TFT driver configured over 3-wire SPI |
//! | [`st77xx`] | `dev-st77xx` | the ST7789/ST7789V and ST7735/ST7735S: a TFT controller with its own frame memory |
//!
//! **The two are not the same kind of part**, and the difference is the whole
//! reason they are two files. The ST7272A holds no picture: its SPI carries
//! register configuration and pixels arrive on a separate parallel-RGB link.
//! The ST77xx holds the picture, in frame memory a guest writes over the same
//! SPI it configures the part with — which is the seam
//! [`crate::dev::lcd::panel`] exists for. `st7272a.rs` sets out the datasheet
//! evidence for its half of that.
//!
//! `no_std + alloc`, no dependencies, and nothing here names a colour space or
//! a host facility.

#[cfg(feature = "dev-st7272a")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-st7272a")))]
pub mod st7272a;

#[cfg(feature = "dev-st77xx")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-st77xx")))]
pub mod st77xx;

use crate::core::error::Result;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if something already claimed a name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    #[cfg(feature = "dev-st7272a")]
    st7272a::register(registry)?;
    #[cfg(feature = "dev-st77xx")]
    st77xx::register(registry)?;
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
    #[cfg(feature = "dev-st77xx")]
    st77xx::bind(bindings)?;
    let _ = bindings;
    Ok(())
}

/// What the validator should know about this module's classes.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![
        #[cfg(feature = "dev-st7272a")]
        st7272a::schema(),
        #[cfg(feature = "dev-st77xx")]
        st77xx::schema(),
    ]
}
