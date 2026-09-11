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
//! | [`sdmmc`] | `stm32.sdmmc` | the H7 family's SDMMC host controller, its FIFO and its internal DMA |
//! | [`sdio`] | `stm32.sdio` | the F2/F4/F7 SDIO and the L1/L4 SDMMC1: the older block, its thirty-two-word FIFO and an **external** DMA request |
//! | [`spi`] | `stm32.spi` | an SPI master and slave in either generation by `variant`: RM0090 §28's F4 block, or RM0351 §42's with its FIFO and programmable `DS` |
//! | [`dma`] | `st.dma` | a DMA controller in either family layout: eight streams (RM0090 §10) or seven channels (RM0351 §11) |
//! | [`octospi`] | `st.octospi` | the L4+/H7A3/L5/U5 OCTOSPI, indirect and memory-mapped |
//! | [`tim`] | `st.tim` | a TIM timer in its basic, general-purpose or advanced form |
//! | [`exti`] | `st.exti` | the external interrupt/event controller: what turns a pin edge into an NVIC request |
//! | [`syscfg`] | `st.syscfg` | `EXTICR`, the pin multiplexer that decides which port drives each EXTI line |
//! | [`flash`] | `st.flash` | the embedded flash interface: wait states, the unlock keys, and a programmable array |
//! | [`crc`] | `st.crc` | the CRC calculation unit, fixed on an F4 and programmable from the F0/F3/F7/L4 on |
//! | [`iwdg`] | `st.iwdg` | the independent watchdog: a down-counter on the LSI that resets the board |
//! | [`wwdg`] | `st.wwdg` | the window watchdog, which also resets the board when a kick comes *early* |
//! | [`rcc`] | `st.rcc` | the reset and clock controller: ready bits, the PLL and prescaler tree, the peripheral gates, the backup domain |
//! | [`pwr`] | `st.pwr` | the power controller: `DBP`, voltage scaling and the F42x over-drive |
//! | [`rng`] | `st.rng` | the random number generator: `CR`/`SR`/`DR`, both error paths, and a stream the machine seeds |
//! | [`rtc`] | `st.rtc` | the real-time clock: a BCD calendar, both alarms, the wakeup timer and the thirty-two backup registers |
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
//! real part. [`sdio`] *is* that F4 block, from RM0090 §31 — two classes rather
//! than one with a `variant`, because `WAITRESP = 10b` means "no response" in
//! one and "short response, CRC not checked" in the other, and no property
//! reconciles that. The pair is the worked example of where the line falls.
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

#[cfg(feature = "dev-stm32-sdio")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-sdio")))]
pub mod sdio;

#[cfg(feature = "dev-stm32-dma")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-dma")))]
pub mod dma;

#[cfg(feature = "dev-stm32-i2c")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-i2c")))]
pub mod i2c;

#[cfg(feature = "dev-stm32-spi")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-spi")))]
pub mod spi;

#[cfg(feature = "dev-stm32-octospi")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-octospi")))]
pub mod octospi;

#[cfg(feature = "dev-stm32-exti")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-exti")))]
pub mod exti;
#[cfg(feature = "dev-stm32-tim")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-tim")))]
pub mod tim;

#[cfg(feature = "dev-stm32-exti")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-exti")))]
pub mod syscfg;

#[cfg(feature = "dev-stm32-crc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-crc")))]
pub mod crc;

#[cfg(feature = "dev-stm32-flash")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-flash")))]
pub mod flash;

#[cfg(feature = "dev-stm32-wdg")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-wdg")))]
pub mod iwdg;

#[cfg(feature = "dev-stm32-rcc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-rcc")))]
pub mod rcc;
#[cfg(feature = "dev-stm32-wdg")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-wdg")))]
pub mod wwdg;

#[cfg(feature = "dev-stm32-pwr")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-pwr")))]
pub mod pwr;

#[cfg(feature = "dev-stm32-rng")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-rng")))]
pub mod rng;

#[cfg(feature = "dev-stm32-rtc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32-rtc")))]
pub mod rtc;

#[cfg(feature = "machine-spi-flash")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-spi-flash")))]
pub mod demo;

#[cfg(feature = "dev-stm32-flash")]
pub use flash::Flash;
#[cfg(feature = "dev-stm32-octospi")]
pub use octospi::Octospi;
#[cfg(feature = "dev-stm32-pwr")]
pub use pwr::Pwr;
#[cfg(feature = "dev-stm32-rcc")]
pub use rcc::{ClockOutput, Clocks, Rcc};
#[cfg(feature = "dev-stm32-rtc")]
pub use rtc::Rtc;
#[cfg(feature = "dev-stm32-spi")]
pub use spi::Stm32Spi;
#[cfg(feature = "dev-stm32-tim")]
pub use tim::Tim;

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
    #[cfg(feature = "dev-stm32-dma")]
    dma::register(registry)?;
    #[cfg(feature = "dev-stm32-sdmmc")]
    sdmmc::register(registry)?;
    #[cfg(feature = "dev-stm32-sdio")]
    sdio::register(registry)?;
    #[cfg(feature = "dev-stm32-i2c")]
    i2c::register(registry)?;
    #[cfg(feature = "dev-stm32-spi")]
    spi::register(registry)?;
    #[cfg(feature = "dev-stm32-octospi")]
    octospi::register(registry)?;
    #[cfg(feature = "dev-stm32-tim")]
    tim::register(registry)?;
    #[cfg(feature = "dev-stm32-exti")]
    exti::register(registry)?;
    #[cfg(feature = "dev-stm32-exti")]
    syscfg::register(registry)?;
    #[cfg(feature = "dev-stm32-crc")]
    crc::register(registry)?;
    #[cfg(feature = "dev-stm32-flash")]
    flash::register(registry)?;
    #[cfg(feature = "dev-stm32-wdg")]
    iwdg::register(registry)?;
    #[cfg(feature = "dev-stm32-wdg")]
    wwdg::register(registry)?;
    #[cfg(feature = "dev-stm32-rcc")]
    rcc::register(registry)?;
    #[cfg(feature = "dev-stm32-pwr")]
    pwr::register(registry)?;
    #[cfg(feature = "dev-stm32-rng")]
    rng::register(registry)?;
    #[cfg(feature = "dev-stm32-rtc")]
    rtc::register(registry)?;
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
    #[cfg(feature = "dev-stm32-dma")]
    dma::bind(bindings)?;
    #[cfg(feature = "dev-stm32-sdmmc")]
    sdmmc::bind(bindings)?;
    #[cfg(feature = "dev-stm32-sdio")]
    sdio::bind(bindings)?;
    #[cfg(feature = "dev-stm32-i2c")]
    i2c::bind(bindings)?;
    #[cfg(feature = "dev-stm32-spi")]
    spi::bind(bindings)?;
    #[cfg(feature = "dev-stm32-octospi")]
    octospi::bind(bindings)?;
    #[cfg(feature = "dev-stm32-tim")]
    tim::bind(bindings)?;
    #[cfg(feature = "dev-stm32-exti")]
    exti::bind(bindings)?;
    #[cfg(feature = "dev-stm32-exti")]
    syscfg::bind(bindings)?;
    #[cfg(feature = "dev-stm32-crc")]
    crc::bind(bindings)?;
    #[cfg(feature = "dev-stm32-flash")]
    flash::bind(bindings)?;
    #[cfg(feature = "dev-stm32-wdg")]
    iwdg::bind(bindings)?;
    #[cfg(feature = "dev-stm32-wdg")]
    wwdg::bind(bindings)?;
    #[cfg(feature = "dev-stm32-rcc")]
    rcc::bind(bindings)?;
    #[cfg(feature = "dev-stm32-pwr")]
    pwr::bind(bindings)?;
    #[cfg(feature = "dev-stm32-rng")]
    rng::bind(bindings)?;
    #[cfg(feature = "dev-stm32-rtc")]
    rtc::bind(bindings)?;
    Ok(())
}

/// Every class's validator schema.
///
/// Every arm below is `cfg`-gated, so a build that enables exactly one of them
/// creates the vector and immediately pushes to it — which is what
/// `vec_init_then_push` objects to, and which no rewriting fixes while the set
/// of arms is a build configuration rather than a list. A build that enables
/// *none* never mutates the vector at all, which is `unused_mut`.
#[must_use]
#[allow(unused_mut, clippy::vec_init_then_push)]
pub fn schemas() -> Vec<ClassSchema> {
    let mut out: Vec<ClassSchema> = alloc::vec![];
    #[cfg(feature = "dev-stm32")]
    out.extend([gpio::schema(), usart::schema()]);
    #[cfg(feature = "dev-stm32-dma")]
    out.push(dma::schema());
    #[cfg(feature = "dev-stm32-sdmmc")]
    out.extend([sdmmc::schema()]);
    #[cfg(feature = "dev-stm32-sdio")]
    out.extend([sdio::schema()]);
    #[cfg(feature = "dev-stm32-i2c")]
    out.extend([i2c::schema()]);
    #[cfg(feature = "dev-stm32-spi")]
    out.extend([spi::schema()]);
    #[cfg(feature = "dev-stm32-octospi")]
    out.push(octospi::schema());
    #[cfg(feature = "dev-stm32-tim")]
    out.push(tim::schema());
    #[cfg(feature = "dev-stm32-exti")]
    out.extend([exti::schema(), syscfg::schema()]);
    #[cfg(feature = "dev-stm32-crc")]
    out.extend([crc::schema()]);
    #[cfg(feature = "dev-stm32-flash")]
    out.push(flash::schema());
    #[cfg(feature = "dev-stm32-wdg")]
    out.extend([iwdg::schema(), wwdg::schema()]);
    #[cfg(feature = "dev-stm32-rcc")]
    out.push(rcc::schema());
    #[cfg(feature = "dev-stm32-pwr")]
    out.push(pwr::schema());
    #[cfg(feature = "dev-stm32-rng")]
    out.push(rng::schema());
    #[cfg(feature = "dev-stm32-rtc")]
    out.push(rtc::schema());
    out
}
