//! Device models.
//!
//! Everything here is feature-gated, one feature per device (`CLAUDE.md`,
//! "Crate shape"): a NES build links a cartridge and a 6502 and nothing else.
//! The core ([`crate::core`]) is what they are all written against, and no type
//! in this tree may appear in a `core::` signature (`ROADMAP.md` §0, "generic
//! first, specific second").
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`cart`] | `dev-nes-cart` | cartridge images and the mappers that decode them |
//!
//! Most of `dev/` is `no_std + alloc`. The two documented exceptions —
//! `dev/blk/*` and `dev/net/*`, which are `std` because `fstool` and `pktkit`
//! are — do not exist yet.

pub mod cart;
