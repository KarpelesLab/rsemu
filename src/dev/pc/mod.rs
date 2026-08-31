//! The chips an IBM PC/AT-class machine is built from.
//!
//! Everything here is a part somebody could buy: two 8259A interrupt
//! controllers, an 8254 timer, an 8042 keyboard controller, an MC146818 RTC,
//! two 8237A DMA controllers, a 6845-derived CRTC with a character generator in
//! front of it, and a µPD765 floppy controller. None of it is PC-specific in
//! itself — the PC is the *wiring*, and that lives in
//! `machines/pc-at.machine`, not in Rust.
//!
//! # Sources
//!
//! Every file cites its own, but the shared ones are:
//!
//! * *IBM Personal Computer AT Technical Reference* (1984) — the board: which
//!   chip answers which port, which IRQ each device lands on, and the system
//!   control ports that are not chips at all.
//! * The Intel component data sheets for the 8259A, 8253/8254, 8237A and 8042,
//!   and the Motorola MC146818 and MC6845 data sheets.
//! * Ralf Brown's Interrupt List, ports section, for the register-level
//!   behaviour the data sheets leave to the board.
//! * The OSDev wiki, for the same facts restated by people who have tested
//!   them.
//!
//! **No emulator source was consulted for any of it** (`CLAUDE.md`, provenance).
//!
//! # The board's I/O map
//!
//! The addresses are the AT's, and they are written once — in the machine file.
//! A device here knows only the offset within its own register block:
//!
//! ```text
//!   0x000-0x00f  DMA controller 1        (byte channels 0-3)
//!   0x020-0x021  interrupt controller 1  (master)
//!   0x040-0x043  8254 timer
//!   0x060,0x064  8042 keyboard controller
//!   0x061        system control port B   (speaker gate, timer 2 out, refresh)
//!   0x070-0x071  MC146818 RTC and CMOS   (plus the NMI mask, in bit 7 of 0x70)
//!   0x080-0x08f  DMA page registers
//!   0x092        system control port A   (A20 gate, fast reset)
//!   0x0a0-0x0a1  interrupt controller 2  (slave, cascaded onto IR2)
//!   0x0c0-0x0df  DMA controller 2        (word channels 4-7)
//!   0x3b4-0x3b5  CRTC, monochrome        }  one chip; which pair answers is
//!   0x3d4-0x3d5  CRTC, colour            }  a board-level decode
//!   0x3f0-0x3f7  floppy controller
//! ```
//!
//! # Conventions every chip here follows
//!
//! * State behind one [`Mutex`](crate::core::sync::Mutex) at
//!   [`LockRank::DEVICE`](crate::core::sync::LockRank::DEVICE); output pins at
//!   [`LEAF`](crate::core::sync::LockRank::LEAF), so a line can be driven with
//!   nothing else held. Never drive a wire with the state lock held — that is
//!   the re-entrancy contract, and an 8259A whose `INT` output re-enters its
//!   own port handler is exactly the case it exists for.
//! * `MemAttrs::debug` suppresses every side effect. A debugger that reads the
//!   8259A's IRR must not pop the 8042's output buffer or clear the RTC's
//!   interrupt flags.
//! * One [`DeviceClass`](crate::core::DeviceClass) per part, `save`/`load` for
//!   its architectural state, and a round-trip test beside it.

pub mod dma;
pub mod kbc;
pub mod pic;
pub mod pit;
pub mod rom;
pub mod rtc;
pub mod sysctl;

#[cfg(feature = "dev-pc-video")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-pc-video")))]
pub mod video;

#[cfg(feature = "dev-pc-floppy")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-pc-floppy")))]
pub mod fdc;

use alloc::vec::Vec;

use crate::core::error::Result;
use crate::core::registry::Registry;
use crate::machine::realize::Bindings;
use crate::machine::validate::ClassSchema;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is already claimed.
pub fn register(reg: &mut Registry) -> Result<()> {
    dma::register(reg)?;
    kbc::register(reg)?;
    pic::register(reg)?;
    pit::register(reg)?;
    rom::register(reg)?;
    rtc::register(reg)?;
    sysctl::register(reg)?;
    #[cfg(feature = "dev-pc-video")]
    video::register(reg)?;
    #[cfg(feature = "dev-pc-floppy")]
    fdc::register(reg)?;
    Ok(())
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is bound twice.
pub fn bind(b: &mut Bindings) -> Result<()> {
    dma::bind(b)?;
    kbc::bind(b)?;
    pic::bind(b)?;
    pit::bind(b)?;
    rom::bind(b)?;
    rtc::bind(b)?;
    sysctl::bind(b)?;
    #[cfg(feature = "dev-pc-video")]
    video::bind(b)?;
    #[cfg(feature = "dev-pc-floppy")]
    fdc::bind(b)?;
    Ok(())
}

/// What the validator should know about every class in this module.
#[must_use]
pub fn schemas() -> Vec<ClassSchema> {
    #[allow(unused_mut)]
    let mut out = alloc::vec![
        dma::schema(),
        kbc::schema(),
        pic::schema(),
        pit::schema(),
        rom::schema(),
        rtc::schema(),
        sysctl::schema(),
    ];
    #[cfg(feature = "dev-pc-video")]
    out.push(video::schema());
    #[cfg(feature = "dev-pc-floppy")]
    out.push(fdc::schema());
    out
}
