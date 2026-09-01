//! STM32 peripherals.
//!
//! One file per peripheral, one feature per file, as the rest of `dev/` is.
//! Nothing generic lives here: an ST register block is an ST register block,
//! and whatever is generic about it belongs in `core/` or in a `bus/` fabric
//! instead (`ROADMAP.md` §0, "generic first, specific second").
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`sdmmc`] | `dev-stm32-sdmmc` | the H7 family's SDMMC host controller, FIFO and internal DMA |
//!
//! # Which family
//!
//! ST reuses a peripheral name across families that share very little, so each
//! module here **names the family and the reference manual it was written
//! from**, at the top of the file, and says where the others differ. A model
//! that quietly averaged two families would be a model of no real part.

#[cfg(feature = "dev-stm32-sdmmc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-sdmmc")))]
pub mod sdmmc;

/// Add every STM32 class in this build to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> crate::core::error::Result<()> {
    #[cfg(feature = "dev-stm32-sdmmc")]
    sdmmc::register(registry)?;
    let _ = registry;
    Ok(())
}

/// Bind every STM32 class in this build into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::error::Result<()> {
    #[cfg(feature = "dev-stm32-sdmmc")]
    sdmmc::bind(bindings)?;
    let _ = bindings;
    Ok(())
}

/// What the validator should know about the STM32 classes in this build.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    #[allow(unused_mut)]
    let mut out = alloc::vec::Vec::new();
    #[cfg(feature = "dev-stm32-sdmmc")]
    out.extend([sdmmc::schema()]);
    out
}
