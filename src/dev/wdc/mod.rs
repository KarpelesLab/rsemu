//! Western Design Center 65xx peripherals, and the board rsemu ships them on.
//!
//! The 65C51 and the 65C22 are the two chips a homebrew 6502 almost always
//! has, and they are the reason a breadboard computer is a *computer* rather
//! than a 6502 running `NOP`. Neither is specific to any one machine — a
//! serial port is a serial port — so they live here under a vendor name rather
//! than a board name.
//!
//! | Module | Covers |
//! | --- | --- |
//! | [`acia`] | the W65C51N ACIA: four registers on `host::chardev` |
//! | [`via`] | the W65C22 VIA: two ports, both timers, IFR/IER |
//! | [`rom`] | the board's 32 KiB EEPROM socket at `$8000` |
//! | [`monitor`] | RSMON/serial, rsemu's own monitor for this board |
//! | [`wozmon`] | the 1976 Woz Monitor, re-plumbed onto the ACIA |
//!
//! # The machine
//!
//! `machines/beneater-6502.machine` puts it together. In short:
//!
//! ```text
//!   $0000-$3FFF  16 KiB of the 32 KiB SRAM
//!   $4000-$5FFF  the ACIA, four registers repeated
//!   $6000-$7FFF  the VIA, sixteen registers repeated
//!   $8000-$FFFF  the program EEPROM, vectors included
//! ```
//!
//! ```console
//! $ rsemu run beneater-6502
//! RSMON
//! >8000
//! 8000: D8 A2 FF 9A A9 1F 8D 03
//! >
//! ```
//!
//! # Provenance
//!
//! Every register and every bit here is from the WDC data sheets, cited by
//! table number in [`acia`] and [`via`]; the memory map is from the published
//! schematics of the board, cited in the machine file. No emulator was
//! consulted (`ROADMAP.md` §1).
//!
//! Two monitors ship, and neither is anyone else's port:
//!
//! * [`monitor`] — RSMON/serial, written for rsemu, MIT, and the default.
//! * [`wozmon`] — the 1976 Woz Monitor, transcribed from the *Apple-1
//!   Operation Manual* and re-plumbed onto the ACIA. That listing is **public
//!   domain**: it was published in 1976 without a copyright notice, which under
//!   the 1909 Act put it there immediately. `docs/platforms/apple1.md` records
//!   the determination and its caveats.
//!
//! Ben Eater's own ACIA port is a separate work, released under **CC-BY** along
//! with the rest of the code in his videos (<https://eater.net/6502>; the
//! statement is on the page itself, which is client-rendered and has to be read
//! in a browser rather than with `curl`). rsemu neither vendors nor derives from
//! it — [`wozmon`] came from the 1976 manual instead, so no attribution
//! obligation attaches to anything here. One env-var-gated test in `tests.rs`
//! runs his image as an unmodified *fixture*, downloaded and never committed,
//! and says so where it does.

pub mod acia;
pub mod monitor;
pub mod rom;
pub mod via;
pub mod wozmon;

// The machine-level tests need a 6502 to run on, so they come with
// `machine-beneater` rather than with the devices alone.
#[cfg(all(test, feature = "machine-beneater"))]
mod tests;

pub use acia::Acia;
pub use monitor::{RSMON, RSMON_BASE, RSMON_IMAGE};
pub use rom::ProgramRom;
pub use via::Via;
pub use wozmon::{WOZMON_BASE, WOZMON_IMAGE};

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is already claimed.
pub fn register(registry: &mut crate::core::Registry) -> crate::core::Result<()> {
    acia::register(registry)?;
    via::register(registry)?;
    rom::register(registry)
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::Result<()> {
    acia::bind(bindings)?;
    via::bind(bindings)?;
    rom::bind(bindings)
}

/// Every class's validator schema.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![acia::schema(), via::schema(), rom::schema()]
}
