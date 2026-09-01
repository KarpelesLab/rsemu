//! STM32 peripherals.
//!
//! ST's STM32 line is not one machine but a family of them, and its peripheral
//! blocks are **reused across families with real differences between them** —
//! the F4's SPI and the H7's SPI share a name, a pin count and almost nothing
//! else. So every module here names the family it models and the reference
//! manual it was written from, in its own module documentation, and a board
//! that wants the other family's block gets another module rather than a
//! property.
//!
//! | Module | Feature | Family | Reference manual |
//! | --- | --- | --- | --- |
//! | [`spi`] | `dev-stm32-spi` | STM32F4 | RM0090 §28 |
//! | [`octospi`] | `dev-stm32-octospi` | STM32L4+ / H7A3 / H7B3 / L5 / U5 | RM0455, RM0456, AN5050 |
//!
//! That second row is worth reading twice: **RM0433's STM32H7 has a QUADSPI,
//! not an OCTOSPI**. The OCTOSPI manuals are RM0432 (L4+), RM0455 and RM0468
//! (H7A3/H7B3 and H723), RM0438 (L5) and RM0456 (U5). Reaching for RM0433
//! because it says "H7" is the first mistake this peripheral invites.
//!
//! Neither models a whole SoC: an rsemu device is a register block plus its
//! pins, and where it sits is a `map` statement in a machine file. Nothing
//! here writes down a base address.
//!
//! `no_std + alloc`, no dependencies. Written from ST's freely published
//! reference manuals; no emulator source of any licence was consulted
//! (`ROADMAP.md` §1).

#[cfg(feature = "machine-spi-flash")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-spi-flash")))]
pub mod demo;
#[cfg(feature = "dev-stm32-octospi")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-octospi")))]
pub mod octospi;
#[cfg(feature = "dev-stm32-spi")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-spi")))]
pub mod spi;

#[cfg(feature = "dev-stm32-octospi")]
pub use octospi::Octospi;
#[cfg(feature = "dev-stm32-spi")]
pub use spi::Stm32Spi;

/// Add every STM32 class to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is already claimed.
pub fn register(registry: &mut crate::core::Registry) -> crate::core::Result<()> {
    #[cfg(feature = "dev-stm32-spi")]
    spi::register(registry)?;
    #[cfg(feature = "dev-stm32-octospi")]
    octospi::register(registry)?;
    Ok(())
}

/// Bind every STM32 class into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::Result<()> {
    #[cfg(feature = "dev-stm32-spi")]
    spi::bind(bindings)?;
    #[cfg(feature = "dev-stm32-octospi")]
    octospi::bind(bindings)?;
    Ok(())
}

/// Every STM32 class's validator schema.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    let mut out = alloc::vec::Vec::new();
    #[cfg(feature = "dev-stm32-spi")]
    {
        out.push(spi::schema());
    }
    #[cfg(feature = "dev-stm32-octospi")]
    {
        out.push(octospi::schema());
    }
    out
}
