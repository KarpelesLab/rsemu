//! Device models.
//!
//! Every device is one Cargo feature (`CLAUDE.md`, "Crate shape"), so a NES
//! build links a picture unit and nothing else. `core/` is always compiled;
//! everything in here is opt-in.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`ppu`] | `dev-nes-ppu` | the NES/Famicom picture processing unit (RP2C02) |

#[cfg(feature = "dev-nes-ppu")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-ppu")))]
pub mod ppu;
