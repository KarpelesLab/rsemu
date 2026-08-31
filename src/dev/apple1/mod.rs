//! The Apple 1 (1976): a 6502, some RAM, one PIA, and a monitor ROM.
//!
//! The smallest complete machine rsemu can run, and the first one a person can
//! interact with. There is no video timing to model and no sound at all: the
//! whole of the machine's I/O is four registers of an MC6821, with a
//! keyboard on one port and a 40x24 character terminal on the other.
//!
//! | Module | Covers |
//! | --- | --- |
//! | [`pia`] | the MC6821 at `$D010-$D013`, keyboard and display |
//! | [`rom`] | the 256-byte monitor ROM socket at `$FF00` |
//! | [`monitor`] | RSMON, rsemu's own 256-byte monitor ROM |
//!
//! # The machine
//!
//! `machines/apple1.machine` puts it together; `docs/platforms/apple1.md` is
//! the guided tour. In short:
//!
//! ```text
//!   $0000-$0FFF  4 KiB of RAM
//!   $D010-$D01F  the PIA, four registers repeated four times
//!   $FF00-$FFFF  the monitor ROM, vectors included
//! ```
//!
//! ```console
//! $ rsemu run apple1
//! RSMON
//! >FF00
//! FF00: D8 A2 FF 9A A9 7F 8D 12
//! >
//! ```
//!
//! # Provenance
//!
//! Everything here is written from hardware documentation — the MC6821 data
//! sheet and the Apple 1 hardware write-ups cited in [`pia`] — and the ROM in
//! [`monitor`] is rsemu's own. The Woz Monitor is *not* vendored: its copyright
//! status is unclear, so it is fetched, never committed, exactly like every
//! other conformance corpus (`CLAUDE.md`, and
//! `docs/testing/conformance-suites.md`).

pub mod monitor;
pub mod pia;
pub mod rom;

// The machine-level tests need a 6502 to run on, so they come with
// `machine-apple1` rather than with the device alone.
#[cfg(all(test, feature = "machine-apple1"))]
mod tests;

pub use monitor::{RSMON, RSMON_BASE};
pub use pia::Pia;
pub use rom::MonitorRom;

/// Add every Apple 1 class to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is already claimed.
pub fn register(registry: &mut crate::core::Registry) -> crate::core::Result<()> {
    pia::register(registry)?;
    rom::register(registry)
}

/// Bind every Apple 1 class into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::Result<()> {
    pia::bind(bindings)?;
    rom::bind(bindings)
}

/// Every Apple 1 class's validator schema.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![pia::schema(), rom::schema()]
}
