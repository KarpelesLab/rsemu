//! rsemu — a multiplatform emulator built bottom-up on a generic framework.
//!
//! The crate is organised as one always-compiled emulation [`core`], with every
//! other component behind its own Cargo feature. See `ROADMAP.md` for the
//! architecture and `CLAUDE.md` for the rules this code is written under.
//!
//! # Status
//!
//! Most of the phase-1 core exists: address spaces and regions ([`core::space`]),
//! the oscillator forest and scheduler ([`core::clock`], [`core::sched`]), wires
//! ([`core::wire`]), the concurrency seam ([`core::sync`]), properties
//! ([`core::props`]), snapshots ([`core::state`]), and the machine-description
//! front end ([`machine`]).
//!
//! The first CPU core is in: `cpu::mos6502`, a cycle-accurate 6502 interpreter
//! behind the `cpu-mos6502` feature (enable it to see [`cpu`]).
//!
//! Not yet: the DSL resolver/validator/realizer, and the machine assembly
//! layer that hands a realizing device its address spaces, clocks and wires.
//!
//! # `no_std`
//!
//! The emulation core is `no_std + alloc`. `std` is a default feature; building
//! with `--no-default-features` must always work, and CI enforces it.

#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]

extern crate alloc;

pub mod core;
pub mod cpu;
pub mod dev;
pub mod machine;

#[cfg(feature = "wasm")]
#[cfg_attr(docsrs, doc(cfg(feature = "wasm")))]
pub mod wasm;

pub use crate::core::{Error, Result};

/// The crate version, as reported by `rsemu --version`.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// A short description of how this build was configured.
///
/// Because a machine is a feature set (`ROADMAP.md` §3), "which rsemu is this?"
/// is a real question with a build-specific answer. This is the honest one.
pub fn build_info() -> alloc::string::String {
    use alloc::string::String;
    use alloc::vec::Vec;

    let mut features: Vec<&str> = Vec::new();
    if cfg!(feature = "std") {
        features.push("std");
    }
    if cfg!(feature = "cli") {
        features.push("cli");
    }
    if cfg!(feature = "wasm") {
        features.push("wasm");
    }

    let mut s = String::from("rsemu ");
    s.push_str(VERSION);
    s.push_str(" [");
    s.push_str(&features.join(", "));
    s.push(']');
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_is_not_empty() {
        assert!(!VERSION.is_empty());
    }

    #[test]
    fn build_info_names_the_crate_and_its_features() {
        let info = build_info();
        assert!(info.starts_with("rsemu "));
        assert!(info.contains(VERSION));
        #[cfg(feature = "std")]
        assert!(info.contains("std"));
    }
}
