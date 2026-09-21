//! The Macintosh board: the address decode, the VIA, the video circuit, the
//! SCC and the IWM.
//!
//! | Module | Feature | What it is |
//! | --- | --- | --- |
//! | [`glue`] | `dev-mac` | the address decoder: the ROM overlay at zero and the RAM window above it |
//! | [`iwm`] | `dev-mac` | an Integrated Woz Machine and the 400K/800K drive on its cable |
//! | [`via`] | `dev-mac` | a 6522 on the board's A9-A12 register select |
//! | [`video`] | `dev-mac` | 512 × 342 one-bit pixels read straight out of main memory |
//! | [`scc`] | `dev-mac` | a Z8530, enough of it for a ROM to find no serial device |
//!
//! Everything here is written from *Guide to the Macintosh Family Hardware*
//! (Apple Computer, 2nd edition) and the parts' own data sheets — Synertek's
//! SY6522, Zilog's Z8530 and Apple's own IWM specification — and from
//! black-box traces of what a real ROM touches where those left a question
//! open. **No Macintosh emulator source was read, and the ROM was not
//! disassembled** (`ROADMAP.md` §1, `CLAUDE.md`). No byte of any Apple ROM is
//! in this repository.
//!
//! `docs/platforms/mac-plus.md` has the memory map, the boot ledger and what
//! is still missing.

pub mod glue;
pub mod iwm;
pub mod scc;
pub mod via;
pub mod video;

use crate::core::error::Result;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if something already claimed
/// one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    glue::register(registry)?;
    iwm::register(registry)?;
    scc::register(registry)?;
    via::register(registry)?;
    video::register(registry)
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if one is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    glue::bind(bindings)?;
    iwm::bind(bindings)?;
    scc::bind(bindings)?;
    via::bind(bindings)?;
    video::bind(bindings)
}

/// What the validator should know about this module's classes.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![
        glue::schema(),
        iwm::schema(),
        scc::schema(),
        via::schema(),
        video::schema(),
    ]
}
