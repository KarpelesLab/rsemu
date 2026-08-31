//! Device models.
//!
//! Everything here is feature-gated, one feature per device (`CLAUDE.md`,
//! "Crate shape"): a NES build links a cartridge, a 6502 and the NES chips, and
//! nothing else. The core ([`crate::core`]) is what they are all written
//! against, and no type in this tree may appear in a `core::` signature
//! (`ROADMAP.md` §0, "generic first, specific second").
//!
//! This module always compiles, and is empty in a build with no device
//! features enabled.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`apple1`] | `dev-apple1` | the Apple 1's MC6821, its monitor ROM socket, and RSMON |
//! | [`apu`] | `dev-nes-apu` | the RP2A03 audio half: channels, frame counter, DMC |
//! | [`cart`] | `dev-nes-cart` | cartridge images and the mappers that decode them |
//! | [`ppu`] | `dev-nes-ppu` | the RP2C02 picture unit: the per-dot pipeline |
//! | [`riscv`] | `dev-riscv` | the RISC-V `virt` board: CLINT, PLIC, 16550, virtio, and the device tree generator |
//!
//! Most of `dev/` is `no_std + alloc`. The two documented exceptions —
//! `dev/blk/*` and `dev/net/*`, which are `std` because `fstool` and `pktkit`
//! are — do not exist yet.

#[cfg(feature = "dev-apple1")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-apple1")))]
pub mod apple1;

#[cfg(feature = "dev-nes-apu")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-apu")))]
pub mod apu;

#[cfg(feature = "dev-nes-cart")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-cart")))]
pub mod cart;

#[cfg(feature = "dev-nes-ppu")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-ppu")))]
pub mod ppu;

#[cfg(feature = "dev-riscv")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-riscv")))]
pub mod riscv;
