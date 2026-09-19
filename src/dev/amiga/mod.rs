//! Commodore Amiga: the memory map, and the seam the custom chips plug into.
//!
//! This module is the *board* and the chips on it. The board's classes are
//! always here with `dev-amiga`; each chip is a feature of its own:
//!
//! | class | what it is |
//! | --- | --- |
//! | [`custom`] | the register window at `$DFF000`, the appendix's table, and the subscription seam a chip attaches through |
//! | [`gary`] | the address decode: the `OVL` overlay at zero, bank 6 (an A501's slow RAM or the chip registers again) and a window at `$E0_0000` for AROS's second ROM half |
//! | `gayle` | `dev-amiga-gayle`: the A600's gate array — the IDE port's chip selects and byte swap in front of an `ata.disk`, the card and IDE interrupt registers at `$DA8000`, the identification register, and the overlay it clears on the first CIA write |
//! | [`cia_decode`] | one 8520's decode — register select on A8–A11, one byte lane of the data bus |
//! | `agnus` | `dev-amiga-agnus`: Agnus — the beam counters and sync, `DMACON`, the copper, the blitter, and every DMA transfer |
//! | [`paula`] | `dev-amiga-paula`: Paula — interrupts onto the 68000's levels, the disk controller, the UART, the four audio channels |
//! | [`floppy`] | `dev-amiga-floppy`: a floppy drive — the mechanism on the CIA ports, raw MFM cells for Paula |
//! | [`adf`] | `dev-amiga-floppy`: not a class — an ADF's sectors encoded as the MFM tracks `trackdisk.device` reads, and decoded back |
//! | `denise` | `dev-amiga-denise`: Denise — the colour table, playfields, sprites and collisions, driven a line at a time by whatever counts the beam |
//! | `keyboard` | `dev-amiga-keyboard`: the keyboard — Appendix G's `KCLK`/`KDAT` protocol, keyed from the host through the record/replay seam |
//! | `mouse` | `dev-amiga-mouse`: the mouse — host motion as quadrature transitions for Denise's counters, and three buttons |
//!
//! plus [`regs`], which is Appendix B of the hardware manual as data and is
//! what makes the first of those a decode rather than three scattered ones, and
//! [`dma`], the lock-free half of Agnus's chip-RAM DMA.
//!
//! # The map
//!
//! From Appendix D of the *Amiga Hardware Reference Manual*, for an A500 or
//! A2000, as far as `machines/amiga-a500.machine` describes it:
//!
//! | address | what |
//! | --- | --- |
//! | `$00_0000`–`$07_FFFF` | chip RAM (512 KiB; `$08_0000`–`$0F_FFFF` is the second 512 KiB on a 1 MiB machine) |
//! | `$BF_D000`–`$BF_DF00` | the 8520-B CIA, **even** byte addresses only |
//! | `$BF_E001`–`$BF_EF01` | the 8520-A CIA, **odd** byte addresses only |
//! | `$DF_F000`–`$DF_FFFF` | the custom chip registers |
//! | `$F8_0000`–`$FF_FFFF` | the system ROM |
//!
//! Two notes on that last row, because it is the one most often quoted wrong.
//! The appendix as printed says `$FC_0000`–`$FF_FFFF`, 256 KiB — it describes
//! the machine of its edition, whose Kickstart is 256 KiB. A 512 KiB Kickstart
//! occupies `$F8_0000` upwards, which is why the board takes the base as a
//! parameter and derives it from the image size rather than writing a constant
//! down.
//!
//! And at reset the ROM is *also* at zero. See [`gary`] for how that is
//! expressed and why it is a decoder rather than a pair of mappings.
//!
//! # What is here and what is not
//!
//! Deliberately absent from this module: where a disk image comes from (a
//! file, an Amiga Forever disc — `host::media::adf`), the serial port's host
//! end and the video output's, which are host adapters. What an image *means*
//! to the drive is here, in [`adf`]. The 8520s
//! are `mos.8520` in [`dev::mos`](crate::dev::mos). The custom register space
//! answers and counts what it could not route
//! ([`custom::CustomBus::unclaimed`]), which is a measurement of how much
//! chipset is still missing rather than a pretence that none is.
//!
//! # Provenance
//!
//! Everything here is written from the *Amiga Hardware Reference Manual*
//! (Commodore-Amiga Inc., 3rd edition) — Appendix B for the register table and
//! its legend, Appendix D for the address map — and from the M68000 User's
//! Manual for the reset sequence. **No Amiga emulator source was consulted**:
//! every one the author is aware of (UAE and its descendants, vAmiga) is GPL,
//! and AROS is MPL-derived weak copyleft. `ROADMAP.md` §1.

#[cfg(feature = "dev-amiga-agnus")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-amiga-agnus")))]
pub mod agnus;
pub mod cia_decode;
pub mod custom;
#[cfg(feature = "dev-amiga-denise")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-amiga-denise")))]
pub mod denise;
pub mod dma;
pub mod gary;
#[cfg(feature = "dev-amiga-gayle")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-amiga-gayle")))]
pub mod gayle;
#[cfg(feature = "dev-amiga-keyboard")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-amiga-keyboard")))]
pub mod keyboard;
#[cfg(feature = "dev-amiga-mouse")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-amiga-mouse")))]
pub mod mouse;
pub mod regs;

#[cfg(feature = "dev-amiga-paula")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-amiga-paula")))]
pub mod paula;

#[cfg(feature = "dev-amiga-floppy")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-amiga-floppy")))]
pub mod floppy;

#[cfg(feature = "dev-amiga-floppy")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-amiga-floppy")))]
pub mod adf;

use crate::core::error::Result;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// If something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    cia_decode::register(registry)?;
    custom::register(registry)?;
    #[cfg(feature = "dev-amiga-denise")]
    denise::register(registry)?;
    gary::register(registry)?;
    #[cfg(feature = "dev-amiga-gayle")]
    gayle::register(registry)?;
    #[cfg(feature = "dev-amiga-paula")]
    paula::register(registry)?;
    #[cfg(feature = "dev-amiga-floppy")]
    floppy::register(registry)?;
    #[cfg(feature = "dev-amiga-agnus")]
    agnus::register(registry)?;
    #[cfg(feature = "dev-amiga-keyboard")]
    keyboard::register(registry)?;
    #[cfg(feature = "dev-amiga-mouse")]
    mouse::register(registry)?;
    Ok(())
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// If one of the classes is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    cia_decode::bind(bindings)?;
    custom::bind(bindings)?;
    #[cfg(feature = "dev-amiga-denise")]
    denise::bind(bindings)?;
    gary::bind(bindings)?;
    #[cfg(feature = "dev-amiga-gayle")]
    gayle::bind(bindings)?;
    #[cfg(feature = "dev-amiga-paula")]
    paula::bind(bindings)?;
    #[cfg(feature = "dev-amiga-floppy")]
    floppy::bind(bindings)?;
    #[cfg(feature = "dev-amiga-agnus")]
    agnus::bind(bindings)?;
    #[cfg(feature = "dev-amiga-keyboard")]
    keyboard::bind(bindings)?;
    #[cfg(feature = "dev-amiga-mouse")]
    mouse::bind(bindings)?;
    Ok(())
}

/// What the validator should know about this module's classes.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    #[allow(unused_mut)]
    let mut schemas = alloc::vec![cia_decode::schema(), custom::schema(), gary::schema()];
    #[cfg(feature = "dev-amiga-gayle")]
    schemas.push(gayle::schema());
    #[cfg(feature = "dev-amiga-paula")]
    schemas.push(paula::schema());
    #[cfg(feature = "dev-amiga-floppy")]
    schemas.push(floppy::schema());
    #[cfg(feature = "dev-amiga-denise")]
    schemas.push(denise::schema());
    #[cfg(feature = "dev-amiga-agnus")]
    schemas.push(agnus::schema());
    #[cfg(feature = "dev-amiga-keyboard")]
    schemas.push(keyboard::schema());
    #[cfg(feature = "dev-amiga-mouse")]
    schemas.push(mouse::schema());
    schemas
}
