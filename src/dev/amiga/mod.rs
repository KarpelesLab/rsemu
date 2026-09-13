//! Commodore Amiga: the memory map, and the seam the custom chips plug into.
//!
//! This module is the *board*, not the chipset. It has three classes and none
//! of them is Agnus, Denise, Paula or an 8520:
//!
//! | class | what it is |
//! | --- | --- |
//! | [`custom`] | the register window at `$DFF000`, the appendix's table, and the subscription seam a chip attaches through |
//! | [`gary`] | the `OVL` overlay — whether the Kickstart ROM or chip RAM answers at address zero |
//! | [`cia_decode`] | one 8520's decode — register select on A8–A11, one byte lane of the data bus |
//!
//! plus [`regs`], which is Appendix B of the hardware manual as data and is
//! what makes the first of those a decode rather than three scattered ones.
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
//! Deliberately absent: Agnus, Denise, Paula, the 8520s themselves, the floppy, the
//! serial port, the video output. The custom register space answers and counts
//! what it could not route ([`custom::CustomBus::unclaimed`]), which is a
//! measurement of how much chipset is still missing rather than a pretence that
//! none is.
//!
//! # Provenance
//!
//! Everything here is written from the *Amiga Hardware Reference Manual*
//! (Commodore-Amiga Inc., 3rd edition) — Appendix B for the register table and
//! its legend, Appendix D for the address map — and from the M68000 User's
//! Manual for the reset sequence. **No Amiga emulator source was consulted**:
//! every one the author is aware of (UAE and its descendants, vAmiga) is GPL,
//! and AROS is MPL-derived weak copyleft. `ROADMAP.md` §1.

pub mod cia_decode;
pub mod custom;
pub mod gary;
pub mod regs;

use crate::core::error::Result;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// If something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    cia_decode::register(registry)?;
    custom::register(registry)?;
    gary::register(registry)?;
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
    gary::bind(bindings)?;
    Ok(())
}

/// What the validator should know about this module's classes.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![cia_decode::schema(), custom::schema(), gary::schema()]
}
