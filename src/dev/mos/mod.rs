//! MOS Technology peripherals.
//!
//! | Module | Covers |
//! | --- | --- |
//! | [`cia`] | the MOS 8520 CIA: two ports, two timers, the 24-bit TOD counter and the shift register |
//!
//! The 8520 is here under a vendor name rather than a board name for the usual
//! reason: an Amiga has two of them and a CBM machine of that era has one or
//! more, and none of that is a property of the chip. Everything a *board* does
//! with it — where the sixteen registers land, what the port pins are called,
//! what drives the TOD pin — stays in the board, and [`cia`]'s module
//! documentation records what an Amiga in particular needs.
//!
//! # Provenance
//!
//! Every register and every bit is from the MOS 6526 and MOS 8520 data sheets
//! and the *Amiga Hardware Reference Manual*, cited in [`cia`] by section. No
//! emulator was consulted (`ROADMAP.md` §1): every Amiga emulator in existence
//! is GPL, and none of them was opened.

pub mod cia;

pub use cia::Cia;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is already claimed.
pub fn register(registry: &mut crate::core::Registry) -> crate::core::Result<()> {
    cia::register(registry)
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::Result<()> {
    cia::bind(bindings)
}

/// Every class's validator schema.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![cia::schema()]
}
