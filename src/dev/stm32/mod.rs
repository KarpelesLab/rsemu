//! STMicroelectronics STM32 peripherals.
//!
//! The chips an STM32F4-class microcontroller is built from, as separate
//! device classes a `.machine` file places at the addresses its reference
//! manual gives them. Nothing here knows what board it is on: every address is
//! a `map` statement and every per-instance difference — a GPIO port's reset
//! values, a USART's register layout — is a construction property.
//!
//! | Module | Class | Covers |
//! | --- | --- | --- |
//! | [`gpio`] | `st.gpio` | one general-purpose I/O port: `MODER`…`AFR`, the atomic `BSRR`, and the pin mux |
//! | [`usart`] | `st.usart` | a USART/UART on the character-device seam, in both the F4 and the F7/H7 register layouts |
//! | [`sdmmc`] | `st.sdmmc` | the H7 family's SDMMC host controller, its FIFO and its internal DMA |
//! | [`spi`] | `st.spi` | the F4 family's SPI/I2S master, RM0090 §28 |
//! | [`octospi`] | `st.octospi` | the L4+/H7A3/L5/U5 OCTOSPI, indirect and memory-mapped |
//!
//! # Which part
//!
//! [`machines/stm32f407.machine`] models an **STM32F407VG**, and the register
//! models here are written from that part's reference manual, ST
//! **RM0090**. Where a later family genuinely differs rather than merely
//! adding, the difference is a property rather than a second class — see
//! [`usart`], where it is the whole register map.
//!
//! Where a peripheral is a *different* peripheral between families rather than
//! a variant of one, it says so at the top of its own file and names the manual
//! it was written from: [`sdmmc`] is the H7's, RM0433, and is not the F4's
//! SDIO. A model that quietly averaged two families would be a model of no
//! real part.
//!
//! That applies with force to [`octospi`]: **RM0433's STM32H7 has a QUADSPI,
//! not an OCTOSPI.** The OCTOSPI manuals are RM0432 (L4+), RM0455 and RM0468
//! (H7A3/H7B3, H723), RM0438 (L5) and RM0456 (U5). Reaching for RM0433 because
//! it says "H7" is the first mistake this peripheral invites.
//!
//! `no_std + alloc`, no `unsafe`, no dependencies.
//!
//! [`machines/stm32f407.machine`]: https://github.com/KarpelesLab/rsemu/blob/master/machines/stm32f407.machine

#[cfg(feature = "dev-stm32")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32")))]
pub mod gpio;

#[cfg(feature = "dev-stm32")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32")))]
pub mod usart;

#[cfg(feature = "dev-stm32-sdmmc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-sdmmc")))]
pub mod sdmmc;

#[cfg(feature = "dev-stm32-i2c")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-i2c")))]
pub mod i2c;

#[cfg(feature = "dev-stm32-spi")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-spi")))]
pub mod spi;

#[cfg(feature = "dev-stm32-octospi")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-octospi")))]
pub mod octospi;

#[cfg(feature = "machine-spi-flash")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-spi-flash")))]
pub mod demo;

#[cfg(feature = "dev-stm32-octospi")]
pub use octospi::Octospi;
#[cfg(feature = "dev-stm32-spi")]
pub use spi::Stm32Spi;

use alloc::vec::Vec;

use crate::core::error::Result;
use crate::machine::validate::ClassSchema;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// If something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    #[cfg(feature = "dev-stm32")]
    gpio::register(registry)?;
    #[cfg(feature = "dev-stm32")]
    usart::register(registry)?;
    #[cfg(feature = "dev-stm32-sdmmc")]
    sdmmc::register(registry)?;
    #[cfg(feature = "dev-stm32-i2c")]
    i2c::register(registry)?;
    #[cfg(feature = "dev-stm32-spi")]
    spi::register(registry)?;
    #[cfg(feature = "dev-stm32-octospi")]
    octospi::register(registry)?;
    Ok(())
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// If one of the classes is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    #[cfg(feature = "dev-stm32")]
    gpio::bind(bindings)?;
    #[cfg(feature = "dev-stm32")]
    usart::bind(bindings)?;
    #[cfg(feature = "dev-stm32-sdmmc")]
    sdmmc::bind(bindings)?;
    #[cfg(feature = "dev-stm32-i2c")]
    i2c::bind(bindings)?;
    #[cfg(feature = "dev-stm32-spi")]
    spi::bind(bindings)?;
    #[cfg(feature = "dev-stm32-octospi")]
    octospi::bind(bindings)?;
    Ok(())
}

/// Every class's validator schema.
#[must_use]
pub fn schemas() -> Vec<ClassSchema> {
    #[allow(unused_mut)]
    let mut out: Vec<ClassSchema> = alloc::vec![];
    #[cfg(feature = "dev-stm32")]
    out.extend([gpio::schema(), usart::schema()]);
    #[cfg(feature = "dev-stm32-sdmmc")]
    out.push(sdmmc::schema());
    #[cfg(feature = "dev-stm32-i2c")]
    out.push(i2c::schema());
    #[cfg(feature = "dev-stm32-spi")]
    out.push(spi::schema());
    #[cfg(feature = "dev-stm32-octospi")]
    out.push(octospi::schema());
    out
}
