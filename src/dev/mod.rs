//! Device models (`ROADMAP.md` §4.4, §7).
//!
//! Everything here is feature-gated, one Cargo feature per device, so a NES
//! build links a 6502 and nothing else. The module itself always compiles and
//! is empty in a build with no device features enabled.

#[cfg(feature = "dev-nes-apu")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-apu")))]
pub mod apu;
