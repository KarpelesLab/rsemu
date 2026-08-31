//! The console's own I/O: the controller ports and OAM DMA.
//!
//! Everything on the NES motherboard that is neither the CPU, the PPU, the APU
//! nor the cartridge. It is a short list, because there is very little else:
//!
//! | Module | Class | Decodes |
//! | --- | --- | --- |
//! | [`input`] | `nes.ports` | `$4016`/`$4017` — the two controller ports |
//! | [`dma`] | `nes.oamdma` | `$4014` — the sprite DMA unit |
//!
//! Both sit inside the RP2A03 package alongside the CPU and the APU, but
//! neither is part of either: `$4016` is a latch and two shift registers driven
//! straight off the connector pins, and `$4014` is a small state machine that
//! borrows the bus. Modelling them as their own devices is what lets a machine
//! description map exactly the bytes each one decodes — see
//! [`crate::dev::apu::WINDOWS`], which leaves precisely those holes.
//!
//! Source: the NESdev wiki, [Standard
//! controller](https://www.nesdev.org/wiki/Standard_controller),
//! [Controller port registers](https://www.nesdev.org/wiki/Controller_port_registers),
//! [Input devices](https://www.nesdev.org/wiki/Input_devices) and
//! [DMA](https://www.nesdev.org/wiki/DMA). Clean-room throughout: no emulator
//! source was consulted for either.
//!
//! # `no_std`
//!
//! `no_std + alloc`, no dependencies, no `unsafe`.

pub mod dma;
pub mod input;

pub use dma::{OAM_DMA_CLASS, OamDma};
pub use input::{DEFAULT_PAD_PORT, NesPorts, PORTS_CLASS, Pad, buttons, pads};

use crate::core::error::Result;

/// Add every class in this module to a registry.
///
/// Registration is explicit per feature rather than link-time magic
/// (`ROADMAP.md` §4.4); one call per module keeps
/// [`catalog`](crate::machine::catalog) to a line.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class name is already taken.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    input::register(registry)?;
    dma::register(registry)
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    input::bind(bindings)?;
    dma::bind(bindings)
}

/// What the validator should know about this module's classes.
#[must_use]
pub fn schemas() -> [crate::machine::validate::ClassSchema; 2] {
    [input::schema(), dma::schema()]
}
