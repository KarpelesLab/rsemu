//! Nintendo Super Famicom / SNES devices.
//!
//! | Module | Class | Covers |
//! | --- | --- | --- |
//! | [`cart`] | `snes.cart` | the cartridge: LoROM/HiROM/ExHiROM decoding, SRAM, header detection |

#[cfg(feature = "dev-snes-cart")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-snes-cart")))]
pub mod cart;

use alloc::vec::Vec;

use crate::core::error::Result;

/// Add every SNES class this build has to a registry.
///
/// # Errors
///
/// If something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    #[cfg(feature = "dev-snes-cart")]
    cart::register(registry)?;
    Ok(())
}

/// Bind every SNES class this build has into the machine graph.
///
/// # Errors
///
/// If one of the classes is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    #[cfg(feature = "dev-snes-cart")]
    cart::bind(bindings)?;
    Ok(())
}

/// What the validator should know about every SNES class this build has.
#[must_use]
#[allow(unused_mut, clippy::vec_init_then_push)]
pub fn schemas() -> Vec<crate::machine::validate::ClassSchema> {
    let mut out = Vec::new();
    #[cfg(feature = "dev-snes-cart")]
    out.push(cart::schema());
    out
}
