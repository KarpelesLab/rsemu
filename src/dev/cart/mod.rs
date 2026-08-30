//! Cartridges: ROM images, the boards they sit on, and the mappers that decode
//! them.
//!
//! A cartridge is two things that are easy to conflate. The **image** is a file
//! format — a header followed by some ROM — and parsing one is untrusted-input
//! work. The **board** is hardware: address decoding, on-cartridge RAM, and the
//! nametable-mirroring wiring. This module keeps them apart. [`ines`] parses an
//! image into a [`Cartridge`]; one module per mapper turns that `Cartridge`
//! into a [`Device`](crate::core::Device) that maps regions into the machine's
//! address spaces ([`nrom`]).
//!
//! # Feature gating
//!
//! Everything below is behind `dev-nes-cart`. A Game Boy or Master System
//! cartridge would be a sibling module behind its own feature; nothing is
//! factored out for sharing yet, because inventing the shared abstraction
//! before the second implementation exists is how it comes out wrong.

#[cfg(feature = "dev-nes-cart")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-cart")))]
pub mod ines;

#[cfg(feature = "dev-nes-cart")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-cart")))]
pub mod nrom;

#[cfg(feature = "dev-nes-cart")]
pub use ines::{
    Cartridge, Chr, ConsoleKind, HeaderFormat, InesHeader, Mirroring, RomError, RomPart, TimingMode,
};

#[cfg(feature = "dev-nes-cart")]
pub use nrom::{CartMappings, Nrom};
