//! The Macintosh board: the address decode, the VIA, the video circuit, the
//! SCC and the IWM.
//!
//! | Module | Feature | What it is |
//! | --- | --- | --- |
//! | [`adb`] | `dev-mac` | the Apple Desktop Bus transceiver on the VIA's shift register, and the keyboard and mouse on it |
//! | [`glue`] | `dev-mac` | the address decoder: the ROM overlay at zero and the RAM window above it |
//! | [`disk`] | `dev-mac` | the disk in the drive: a 400K or 800K image, raw or in a DiskCopy 4.2 container |
//! | [`gcr`] | `dev-mac` | Apple's 6-and-2 group-code recording: the bit stream on an 800K disk |
//! | [`iwm`] | `dev-mac` | an Integrated Woz Machine and the 400K/800K drive on its cable |
//! | [`keyboard`] | `dev-mac` | the keyboard on the VIA's shift register, and the four commands it answers |
//! | [`mouse`] | `dev-mac` | the one-button mouse: quadrature on the SCC's carrier detects and the VIA's port B |
//! | [`via`] | `dev-mac` | a 6522 on the board's A9-A12 register select |
//! | [`video`] | `dev-mac` | 512 × 342 one-bit pixels read straight out of main memory |
//! | [`rtc`] | `dev-mac` | the clock chip: a second counter, twenty bytes of parameter RAM, and the one-second interrupt |
//! | [`scc`] | `dev-mac` | a Z8530, enough of it for a ROM to find no serial device |
//! | [`sound`] | `dev-mac` | the pulse-width sound circuit: a byte a scan line out of main memory |
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

pub mod adb;
pub mod disk;
pub mod gcr;
pub mod glue;
pub mod iwm;
pub mod mfm;
pub mod keyboard;
pub mod mouse;
pub mod rtc;
pub mod scc;
pub mod sound;
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
    adb::register(registry)?;
    glue::register(registry)?;
    iwm::register(registry)?;
    keyboard::register(registry)?;
    mouse::register(registry)?;
    rtc::register(registry)?;
    scc::register(registry)?;
    sound::register(registry)?;
    via::register(registry)?;
    video::register(registry)
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if one is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    adb::bind(bindings)?;
    glue::bind(bindings)?;
    iwm::bind(bindings)?;
    keyboard::bind(bindings)?;
    mouse::bind(bindings)?;
    rtc::bind(bindings)?;
    scc::bind(bindings)?;
    sound::bind(bindings)?;
    via::bind(bindings)?;
    video::bind(bindings)
}

/// What the validator should know about this module's classes.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![
        adb::schema(),
        glue::schema(),
        iwm::schema(),
        keyboard::schema(),
        mouse::schema(),
        rtc::schema(),
        scc::schema(),
        sound::schema(),
        via::schema(),
        video::schema(),
    ]
}
