//! Atmel (now Microchip) parts.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`at24c`] | `dev-at24c` | the AT24C01D/02D I²C serial EEPROM |
//! | [`atecc`] | `dev-atecc` | the ATECC508A/608A/608B CryptoAuthentication secure element |
//!
//! `no_std + alloc`. [`at24c`] has no dependencies; [`atecc`] is the one thing
//! here that does — `purecrypto`, for SHA-256, HMAC, AES-128 and P-256, which
//! is what a secure element is made of.

#[cfg(feature = "dev-at24c")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-at24c")))]
pub mod at24c;

#[cfg(feature = "dev-atecc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-atecc")))]
pub mod atecc;

use crate::core::error::Result;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    #[cfg(feature = "dev-at24c")]
    at24c::register(registry)?;
    #[cfg(feature = "dev-atecc")]
    atecc::register(registry)?;
    let _ = registry;
    Ok(())
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    #[cfg(feature = "dev-at24c")]
    at24c::bind(bindings)?;
    #[cfg(feature = "dev-atecc")]
    atecc::bind(bindings)?;
    let _ = bindings;
    Ok(())
}

/// What the validator should know about every class here.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    // Written as a literal rather than pushed one at a time, because the set
    // is decided at compile time: every arm below is a `cfg`, so a build with
    // one feature on has a one-element vector and no branches at all.
    alloc::vec![
        #[cfg(feature = "dev-at24c")]
        at24c::schema(),
        #[cfg(feature = "dev-atecc")]
        atecc::schema(),
    ]
}
