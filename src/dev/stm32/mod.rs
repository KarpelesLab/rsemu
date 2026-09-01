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
//!
//! # Which part
//!
//! [`machines/stm32f407.machine`] models an **STM32F407VG**, and the register
//! models here are written from that part's reference manual, ST
//! **RM0090**. Where a later family genuinely differs rather than merely
//! adding, the difference is a property rather than a second class — see
//! [`usart`], where it is the whole register map.
//!
//! `no_std + alloc`, no `unsafe`, no dependencies.
//!
//! [`machines/stm32f407.machine`]: https://github.com/KarpelesLab/rsemu/blob/master/machines/stm32f407.machine

pub mod gpio;
pub mod usart;

use alloc::vec::Vec;

use crate::core::error::Result;
use crate::machine::validate::ClassSchema;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// If something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    gpio::register(registry)?;
    usart::register(registry)
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// If one of the classes is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    gpio::bind(bindings)?;
    usart::bind(bindings)
}

/// Every class's validator schema.
#[must_use]
pub fn schemas() -> Vec<ClassSchema> {
    alloc::vec![gpio::schema(), usart::schema()]
}
