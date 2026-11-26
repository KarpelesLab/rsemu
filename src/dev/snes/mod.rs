//! Nintendo Super Famicom / SNES devices.
//!
//! | Module | Class | Covers |
//! | --- | --- | --- |
//! | [`cart`] | `snes.cart` | the cartridge: LoROM/HiROM/ExHiROM decoding, SRAM, header detection |
//! | [`wram`] | `snes.wram` | 128 KiB of work RAM and its `WMDATA`/`WMADD` port on the B-bus |
//! | [`dsp`] | — | the S-DSP: eight BRR voices, envelopes, noise, echo and FIR (`dev-snes-dsp`) |
//! | [`apu`] | `snes.apu` | the S-SMP: SPC700, audio RAM, timers, the CPU ports, the boot ROM (`dev-snes-apu`) |
//! | [`dsp_seam`] | — | what the S-SMP needs of the S-DSP, and the S-DSP of audio RAM |
//! | [`ppu`] | `snes.ppu` | the S-PPU (5C77 + 5C78): registers, VRAM/CGRAM/OAM, modes 0-7, sprites, windows, colour math, beam timing |

#[cfg(feature = "dev-snes-cart")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-snes-cart")))]
pub mod cart;

#[cfg(feature = "dev-snes-cart")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-snes-cart")))]
pub mod wram;

#[cfg(feature = "dev-snes-dsp")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-snes-dsp")))]
pub mod dsp_seam;

#[cfg(feature = "dev-snes-dsp")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-snes-dsp")))]
pub mod dsp;

#[cfg(feature = "dev-snes-apu")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-snes-apu")))]
pub mod apu;
#[cfg(feature = "dev-snes-ppu")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-snes-ppu")))]
pub mod ppu;

use alloc::vec::Vec;

use crate::core::error::Result;

/// Add every SNES class this build has to a registry.
///
/// # Errors
///
/// If something already claimed one of the names.
// The DSP has no class of its own; a DSP-only build has nothing to add.
#[cfg_attr(
    not(any(
        feature = "dev-snes-cart",
        feature = "dev-snes-ppu",
        feature = "dev-snes-apu"
    )),
    allow(unused_variables)
)]
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    #[cfg(feature = "dev-snes-cart")]
    cart::register(registry)?;
    #[cfg(feature = "dev-snes-cart")]
    wram::register(registry)?;
    #[cfg(feature = "dev-snes-ppu")]
    ppu::register(registry)?;
    #[cfg(feature = "dev-snes-apu")]
    apu::register(registry)?;
    Ok(())
}

/// Bind every SNES class this build has into the machine graph.
///
/// # Errors
///
/// If one of the classes is already bound.
// The DSP has no class of its own; a DSP-only build has nothing to add.
#[cfg_attr(
    not(any(
        feature = "dev-snes-cart",
        feature = "dev-snes-ppu",
        feature = "dev-snes-apu"
    )),
    allow(unused_variables)
)]
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    #[cfg(feature = "dev-snes-cart")]
    cart::bind(bindings)?;
    #[cfg(feature = "dev-snes-cart")]
    wram::bind(bindings)?;
    #[cfg(feature = "dev-snes-ppu")]
    ppu::bind(bindings)?;
    #[cfg(feature = "dev-snes-apu")]
    apu::bind(bindings)?;
    Ok(())
}

/// What the validator should know about every SNES class this build has.
#[must_use]
#[allow(unused_mut, clippy::vec_init_then_push)]
pub fn schemas() -> Vec<crate::machine::validate::ClassSchema> {
    let mut out = Vec::new();
    #[cfg(feature = "dev-snes-cart")]
    out.extend([cart::schema(), wram::schema()]);
    #[cfg(feature = "dev-snes-ppu")]
    out.push(ppu::schema());
    #[cfg(feature = "dev-snes-apu")]
    out.push(apu::schema());
    out
}
