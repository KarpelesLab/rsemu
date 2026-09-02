//! The Game Boy's chips (`dev-gb`).
//!
//! Phase 4's *genericity proof* (`ROADMAP.md` §13): a machine that is not
//! 6502-shaped, built out of the same core the NES was, with no change to
//! `core::` to accommodate it. What that looks like in practice is worth stating,
//! because "no core API had to change" is a claim and these are the receipts:
//!
//! | The Game Boy needs | The core already had |
//! | --- | --- |
//! | `IF`/`IE` inside the CPU, mapped at `$FF0F` and `$FFFF` | a device may publish several named regions ([`Device::region`]) |
//! | STAT blocking — a level ORed from four conditions, one interrupt | wire *levels* plus an edge detector in the sink ([`core::wire`]) |
//! | `DIV` writes being audible | one device driving another's clock over an ordinary wire |
//! | A cartridge RTC on its own 32.768 kHz crystal | a second oscillator in the forest, cross-tree by construction ([`core::clock`]) |
//! | An OAM DMA that reads the CPU's own bus | [`Initiator`], and a per-master address space |
//! | `LY` and `STAT` read at an arbitrary cycle | sync-on-access, and a quantum bounded by the device's next event |
//! | VRAM that reads `$FF` during mode 3 | nothing at all: it is the device's own `MemOps` |
//!
//! [`Device::region`]: crate::core::Device::region
//! [`Initiator`]: crate::core::device::Initiator
//! [`core::wire`]: crate::core::wire
//! [`core::clock`]: crate::core::clock
//!
//! # The modules
//!
//! | Module | Covers |
//! | --- | --- |
//! | [`cart`] | the header, ROM-only, MBC1, MBC2, MBC3 (with its RTC) and MBC5 |
//! | [`ppu`] | the LCD controller, video RAM, object memory and the OAM DMA |
//! | [`timer`] | the divider and timer, and the 512 Hz clock the sound unit runs on |
//! | [`apu`] | four sound channels and the frame sequencer |
//! | [`joypad`] | the eight-button matrix at `$FF00` |
//! | [`serial`] | the link port, which is also how a test ROM reports its result |
//!
//! The CPU is [`crate::cpu::sm83`], under its own feature: a machine is a
//! *feature set*, and a build that wants only the chips can have only the chips.
//!
//! # The oscillators
//!
//! One crystal for the console, at **4.194304 MHz** — and unlike the NES's
//! 236250000/11 Hz or the Apple 1's 315/22 MHz, that is an exact power of two:
//! 2²². It is the case that makes the rational frequency literal look like
//! ceremony, and `machines/gameboy.machine` says so explicitly, because a design
//! decision only defends itself if the case *against* it is written down beside
//! the case for it.
//!
//! A cartridge with an MBC3 real-time clock carries a **second** crystal, a
//! 32.768 kHz watch can, with no fixed relationship to the first. `ROADMAP.md`
//! §4.2 is about exactly this: within one crystal's tree the ratios are exact
//! and guest-visible, and across trees exactness is not merely expensive but
//! meaningless. So the machine file declares both.
//!
//! # Sources
//!
//! [Pan Docs](https://gbdev.io/pandocs/) throughout, which is **CC0** — the rare
//! emulation reference that can be quoted verbatim into a source comment with no
//! attribution burden at all — plus Gekkio's *Game Boy: Complete Technical
//! Reference* for sub-instruction timing. No emulator source of any licence was
//! consulted (`ROADMAP.md` §1).

pub mod apu;
pub mod cart;
pub mod joypad;
pub mod ppu;
pub mod serial;
pub mod timer;

#[cfg(test)]
mod tests;

// The conformance runners read downloaded ROMs off the filesystem, so they exist
// only where there is one (`ROADMAP.md` §12).
#[cfg(all(test, feature = "std", feature = "machine-gameboy"))]
mod conformance;

pub use apu::GbApu;
pub use cart::{Cartridge, GbCart, Mapper};
pub use joypad::{Button, GbJoypad, GbPad};
pub use ppu::{GbPpu, Mode, SCREEN_HEIGHT, SCREEN_WIDTH};
pub use serial::GbSerial;
pub use timer::GbTimer;

use crate::core::error::Result;

/// The console's crystal, in hertz.
///
/// 4 194 304 = 2²². Note what that means for `ROADMAP.md` §4.2: every domain in
/// this machine descends from one oscillator with an integer divisor, so every
/// ratio in the machine is exact *and* the absolute frequency happens to be an
/// integer too. That is the exception, not the rule — see the module
/// documentation.
pub const MASTER_HZ: u64 = 4_194_304;

/// Register every Game Boy device class this build has.
///
/// # Errors
///
/// [`crate::Error::Config`] if a name is already taken, which means two features
/// collided.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    cart::register(reg)?;
    ppu::register(reg)?;
    timer::register(reg)?;
    apu::register(reg)?;
    joypad::register(reg)?;
    serial::register(reg)?;
    Ok(())
}

/// Bind every Game Boy device class into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    cart::bind(bindings)?;
    ppu::bind(bindings)?;
    timer::bind(bindings)?;
    apu::bind(bindings)?;
    joypad::bind(bindings)?;
    serial::bind(bindings)?;
    Ok(())
}

/// What the validator should know about every Game Boy class.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![
        cart::schema(),
        ppu::schema(),
        timer::schema(),
        apu::schema(),
        joypad::schema(),
        serial::schema(),
    ]
}
