//! The host-facing layer: where rsemu meets the machine it is running *on*
//! (`ROADMAP.md` §3, §8).
//!
//! Everything below this module is `no_std + alloc` and knows nothing about
//! files, terminals, windows or wall clocks. This is the one place in the tree
//! — along with `jit/` and `accel/` — that is allowed to.
//!
//! | Module | Needs `std` | Covers |
//! | --- | --- | --- |
//! | [`chardev`] | no | the character-stream seam: a byte pipe a device model can hold |
//! | [`terminal`] | yes | that seam, driven by the process's own stdin and stdout |
//! | [`display`] | no | the scanout seam: a guest surface, converted to host pixels |
//! | [`audio`] | no | the audio seam: a guest's samples, converted and rate-matched |
//! | [`input`] | no | the input seam: keys and pointer, at a virtual instant |
//! | `clock` | yes | the host's monotonic clock, which only the rate controller reads |
//! | [`signal`] | yes | `SIGINT`, `SIGTERM` and `SIGHUP`, turned into a flag a run loop reads |
//! | `listen` | yes | where a remote frontend binds, and the loopback-by-default rule |
//! | `gdb` | yes | the GDB remote serial protocol over TCP (§8) |
//! | `vnc` | yes | the RFB protocol over TCP: a screen and a keyboard (§8) |
//!
//! # Why the trait is not itself `std`
//!
//! A device model has to *hold* the far end of a character stream — the Apple 1
//! PIA does, and a 16550 will — and device models are `no_std`. So
//! [`chardev`] is `core + alloc` and always compiles, and only the backends
//! that actually touch the operating system ([`terminal`]) are behind `std`.
//! A `no_std` build gets the trait and the in-memory port, which is exactly
//! what a deterministic test or a wasm embedder wants.

pub mod audio;
pub mod chardev;
pub mod display;
pub mod input;

#[cfg(feature = "std")]
#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
pub mod clock;

#[cfg(feature = "std")]
#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
pub mod listen;

#[cfg(feature = "std")]
#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
pub mod signal;

#[cfg(feature = "std")]
#[cfg_attr(docsrs, doc(cfg(feature = "std")))]
pub mod terminal;

#[cfg(feature = "gdb")]
#[cfg_attr(docsrs, doc(cfg(feature = "gdb")))]
pub mod gdb;

#[cfg(feature = "vnc")]
#[cfg_attr(docsrs, doc(cfg(feature = "vnc")))]
pub mod vnc;
