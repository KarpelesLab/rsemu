//! The Sega Master System's chips (`dev-sms`).
//!
//! The second half of phase 4's *genericity proof* (`ROADMAP.md` §13). The Game
//! Boy was the first, and it was arguably an easy one: a memory-mapped console
//! with one bus, built from a CPU descended from the same family as the NES's.
//! This machine is deliberately not that.
//!
//! | The Master System needs | The core already had |
//! | --- | --- |
//! | A **separate I/O address space** the VDP and pads live in | a space per name, and `BindCtx::space_named` ([`crate::cpu::z80`]) |
//! | One address that is the sound chip on a write and the VDP's counters on a read | [`Region::split`], and `split()` in the DSL |
//! | Bank switching three 16 KiB windows | [`AddressSpace::rebase`]: an atomic store, no retopology |
//! | Pause as a **non-maskable** interrupt | a wire to the core's `nmi` pin, and its edge detector |
//! | `/INT` held until the guest reads a status register | wire *levels*, not pulses |
//! | Two television standards with different line counts | two oscillators in two machine files ([`core::clock`]) |
//! | A VDP that must not be disturbed by a debugger | [`MemAttrs::debug`], honoured on every port |
//!
//! [`Region::split`]: crate::core::space::Region::split
//! [`AddressSpace::rebase`]: crate::core::space::AddressSpace::rebase
//! [`MemAttrs::debug`]: crate::core::space::MemAttrs::debug
//! [`core::clock`]: crate::core::clock
//!
//! # The modules
//!
//! | Module | Covers |
//! | --- | --- |
//! | [`vdp`] | the 315-5124: mode 4, the TMS9918A modes, VRAM, CRAM, the line interrupt |
//! | [`psg`] | the SN76489: three square channels and one noise channel |
//! | [`mapper`] | Sega's standard cartridge mapper and its cartridge RAM |
//! | [`io`] | the two control pads, `$3E`/`$3F`, and the Pause and Reset buttons |
//! | [`sdsc`] | the SDSC debug console, which is how a test ROM reports headlessly |
//!
//! The CPU is [`crate::cpu::z80`], under its own feature: a machine is a
//! *feature set*, and a build that wants only the chips can have only the chips.
//!
//! # The oscillators
//!
//! One crystal, and which one depends on the television standard the console was
//! built for. Both are exact rationals rather than the rounded decimals every
//! reference prints, because a rounded frequency nails a rounding error into the
//! timeline (`ROADMAP.md` §4.2):
//!
//! ```text
//!   NTSC   945000000/88 Hz   = 3 x the NTSC colour subcarrier (315/88 MHz)
//!   PAL    10640685 Hz       = 12/5 x the PAL subcarrier (4433618.75 Hz)
//! ```
//!
//! Everything descends from it by an integer divisor — the Z80 by 3, the VDP's
//! pixel clock by 2 — so every ratio inside the console is exact by
//! construction. What differs between the two machines is the frequency *and*
//! the frame: 262 lines against 313. That is why `sms-ntsc` and `sms-pal` are
//! two files rather than one file with a flag, exactly as `nes-ntsc` and
//! `nes-pal` are.
//!
//! # Sources
//!
//! [SMS Power!'s development documents](https://www.smspower.org/Development/Documents)
//! throughout, plus the TMS9918A and SN76489 datasheets and Zilog's Z80 manual.
//! `docs/platforms/master-system.md` has the register. **No emulator source of
//! any licence was consulted** — not MAME, not higan, not Emulicious, not
//! anything derived from them (`ROADMAP.md` §1).

pub mod io;
pub mod mapper;
pub mod psg;
pub mod sdsc;
pub mod vdp;

#[cfg(test)]
mod tests;

// The conformance runner reads a downloaded ROM off the filesystem, so it exists
// only where there is one (`ROADMAP.md` §12).
#[cfg(all(test, feature = "std", feature = "machine-sms"))]
mod conformance;

// The conformance runner reads a downloaded ROM off the filesystem, so it exists
// only where there is one (`ROADMAP.md` §12).

pub use io::{Button, Nationalisation, SmsIo, SmsPads};
pub use mapper::SegaMapper;
pub use psg::SmsPsg;
pub use sdsc::SdscConsole;
pub use vdp::{SCREEN_HEIGHT, SCREEN_WIDTH, SmsVdp, TvRegion, VdpMode};

use crate::core::error::Result;

/// The NTSC console's crystal, as an exact rational number of hertz.
///
/// 945000000/88 = 10 738 636.36…, which is three times the NTSC colour
/// subcarrier. Every reference prints "10.738635 MHz"; that is the rounded
/// figure, and writing it down would fix a 1.4 Hz error in the timeline
/// forever.
pub const NTSC_MASTER_HZ: (u64, u64) = (945_000_000, 88);

/// The PAL console's crystal, in hertz.
///
/// 12/5 of the PAL subcarrier, 4 433 618.75 Hz, which comes out an exact
/// integer.
pub const PAL_MASTER_HZ: (u64, u64) = (10_640_685, 1);

/// What the master clock is divided by to clock the Z80.
pub const CPU_DIVIDER: u64 = 3;

/// What it is divided by to clock the VDP's pixel counter.
pub const DOT_DIVIDER: u64 = 2;

/// Register every Master System device class this build has.
///
/// # Errors
///
/// [`crate::Error::Config`] if a name is already taken, which means two features
/// collided.
pub fn register(reg: &mut crate::core::Registry) -> Result<()> {
    vdp::register(reg)?;
    psg::register(reg)?;
    mapper::register(reg)?;
    io::register(reg)?;
    sdsc::register(reg)?;
    Ok(())
}

/// Bind every Master System device class into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    vdp::bind(bindings)?;
    psg::bind(bindings)?;
    mapper::bind(bindings)?;
    io::bind(bindings)?;
    sdsc::bind(bindings)?;
    Ok(())
}

/// What the validator should know about every Master System class.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![
        vdp::schema(),
        psg::schema(),
        mapper::schema(),
        io::schema(),
        sdsc::schema(),
    ]
}
