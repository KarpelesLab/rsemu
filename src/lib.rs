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
//! behind the `cpu-mos6502` feature (enable it to see [`cpu`]). With
//! `machine-nes`, [`machine::catalog`] ships a NES that a real cartridge boots
//! on: `rsemu run nes-ntsc --cart smb.nes`.
//!
//! With `machine-apple1`, [`machine::catalog`] also ships an Apple 1 — a 6502,
//! 4 KiB of RAM, an MC6821 and a 256-byte monitor ROM — which is the first
//! machine a person can actually type at: `rsemu run apple1`. It reaches the
//! terminal through [`host::chardev`], the character-stream seam a 16550 will
//! use next.
//!
//! The picture comes out through [`host::display`], the scanout seam: a device
//! emits whatever the silicon does — the 2C02 emits a palette index, not a
//! colour — and the host converts it, captures it as a PNG (`display-png`), or
//! hands it to a canvas. `web/` is the browser demo that does the last of those,
//! from the `demo` feature.
//!
//! With `gdb`, [`host::gdb`] speaks the GDB remote serial protocol over TCP, so
//! `rsemu debug apple1 --gdb :1234` is a guest a debugger can step through.
//!
//! Not yet: audio anywhere but inside the APU; a native window; the IR and JIT;
//! and the rest of the host layer (VNC, an interactive monitor console).
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
pub mod host;
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
    if cfg!(feature = "gdb") {
        features.push("gdb");
    }
    if cfg!(feature = "wasm") {
        features.push("wasm");
    }
    if cfg!(feature = "cpu-mos6502") {
        features.push("cpu-mos6502");
    }
    if cfg!(feature = "dev-nes-cart") {
        features.push("dev-nes-cart");
    }
    if cfg!(feature = "dev-nes-ppu") {
        features.push("dev-nes-ppu");
    }
    if cfg!(feature = "dev-nes-apu") {
        features.push("dev-nes-apu");
    }
    if cfg!(feature = "machine-nes") {
        features.push("machine-nes");
    }
    if cfg!(feature = "dev-apple1") {
        features.push("dev-apple1");
    }
    if cfg!(feature = "machine-apple1") {
        features.push("machine-apple1");
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
