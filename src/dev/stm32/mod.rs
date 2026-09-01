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
    out
}
