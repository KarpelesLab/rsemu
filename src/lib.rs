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
//! The sound comes out through [`host::audio`], which is the same seam again: a
//! device emits what the silicon does — the RP2A03 emits an unsigned level out
//! of a non-linear DAC pair at 894 886.36… Hz — and the host centres it, applies
//! the console's own RC network, resamples it to 44.1 or 48 kHz with an exact
//! integer phase, and writes it to a `.wav` or hands it to WebAudio. Every float
//! in that path is an amplitude, never a time, so a machine's state hash does
//! not depend on whether anybody is listening.
//!
//! With `gdb`, [`host::gdb`] speaks the GDB remote serial protocol over TCP, so
//! `rsemu debug apple1 --gdb :1234` is a guest a debugger can step through.
//!
//! With `usermode`, [`usermode`] is level-3 execution: a program runs with **no
//! guest kernel under it**, its `ecall` leaves the core through
//! [`core::exec`], and something in Rust services it. rsemu supplies the
//! machine half — the exit, a memory map with no devices in it, a scheduling
//! contract for guest threads, and the record/replay funnel; the syscall
//! kernel is a downstream crate's (`ROADMAP.md` §2.1).
//!
//! Not yet: a native window or a native sound card — both need either a
//! GUI/audio dependency the policy forbids or a seventh `unsafe` subsystem the
//! ceiling forbids; the JIT backends, so everything is interpreted — the
//! translation IR they lower from is under [`ir`]; and the rest of the host
//! layer (VNC, an interactive monitor console).
//!
//! # `no_std`
//!
//! The emulation core is `no_std + alloc`. `std` is a default feature; building
//! with `--no-default-features` must always work, and CI enforces it.

#![cfg_attr(not(feature = "std"), no_std)]
#![cfg_attr(docsrs, feature(doc_cfg))]

extern crate alloc;

pub mod bus;
pub mod core;
pub mod cpu;
pub mod dev;
pub mod host;
pub mod machine;

#[cfg(feature = "float")]
#[cfg_attr(docsrs, doc(cfg(feature = "float")))]
pub mod float;

#[cfg(feature = "ir")]
#[cfg_attr(docsrs, doc(cfg(feature = "ir")))]
pub mod ir;

#[cfg(feature = "usermode")]
#[cfg_attr(docsrs, doc(cfg(feature = "usermode")))]
pub mod usermode;

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
    if cfg!(feature = "dev-pc") {
        features.push("dev-pc");
    }
    if cfg!(feature = "dev-pc-video") {
        features.push("dev-pc-video");
    }
    if cfg!(feature = "dev-pc-floppy") {
        features.push("dev-pc-floppy");
    }
    if cfg!(feature = "dev-ata-disk") {
        features.push("dev-ata-disk");
    }
    if cfg!(feature = "dev-blk") {
        features.push("dev-blk");
    }
    if cfg!(feature = "dev-pc-ide") {
        features.push("dev-pc-ide");
    }
    if cfg!(feature = "machine-pc-at") {
        features.push("machine-pc-at");
    }
    if cfg!(feature = "ir") {
        features.push("ir");
    }
    if cfg!(feature = "usermode") {
        features.push("usermode");
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
