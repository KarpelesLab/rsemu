//! Display controllers: the things that turn guest memory into a picture.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`scanout`] | `dev-lcdc` | a generic parallel-RGB scanout engine: geometry, pixel format, framebuffer base, frame period |
//! | [`demo`] | `dev-lcdc` | the `spi-panel` board's demo firmware, assembled at compile time |
//! | [`panel`] | `dev-ssd1306` | the seam a **smart panel** presents: a controller that owns its own framebuffer |
//!
//! Those are the two kinds of display device and they need different seams —
//! [`panel`] is where the difference is written down.
//!
//! `no_std + alloc`, no dependencies.

#[cfg(feature = "dev-ssd1306")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-ssd1306")))]
pub mod panel;

#[cfg(feature = "dev-lcdc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-lcdc")))]
pub mod demo;

#[cfg(feature = "dev-lcdc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-lcdc")))]
pub mod scanout;

use crate::core::error::Result;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if something already claimed a name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    #[cfg(feature = "dev-lcdc")]
    scanout::register(registry)?;
    let _ = registry;
    Ok(())
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    #[cfg(feature = "dev-lcdc")]
    scanout::bind(bindings)?;
    let _ = bindings;
    Ok(())
}

/// What the validator should know about this module's classes.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![
        #[cfg(feature = "dev-lcdc")]
        scanout::schema(),
    ]
}
