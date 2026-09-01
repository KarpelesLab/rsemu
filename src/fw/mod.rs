//! Firmware rsemu ships: **guest** programs, built by the host build.
//!
//! Everything else in this crate is the emulator. This module is the one place
//! where the artefact is a program the *guest* runs — a ROM image, built out of
//! machine code that no host CPU here will ever execute. That is a new category
//! in this tree and it changes which rules apply, so the argument is written
//! down rather than assumed.
//!
//! # Why rsemu ships firmware at all
//!
//! It ships exactly one piece, and only because there is no other way to have
//! it. `ROADMAP.md` phase 6a names it: FreeDOS, Windows 95 and Windows XP all
//! need a **legacy** BIOS, the only permissively-licensed PC firmware in
//! existence is EDK II / OVMF, which is UEFI, and the legacy path everyone else
//! reaches for is GPL and therefore unreadable to us (§1). "Ship a GPL blob" is
//! not an option and "read one to learn how" is not either. So the BIOS is
//! written here, from the interrupt ABI as documented rather than from anyone's
//! implementation of it — see [`pcbios`] for the source register and what was
//! deliberately not consulted.
//!
//! This is **not** a licence to vendor firmware in general. `pc-at`'s `bios`
//! media slot still takes the user's own image and always will; this is a
//! default for the slot, not a replacement for it, and every other machine that
//! wants firmware still asks the user for it (`docs/platforms/pc-at.md`).
//!
//! # Which rules apply to guest code
//!
//! `CLAUDE.md` governs host code, and most of it applies unchanged because the
//! *builder* is ordinary host code: `no_std + alloc`, no dependencies, `Debug`
//! on public types, documented public items, tests beside the module. Three
//! rules mean something different here and one is suspended:
//!
//! - **`unsafe` is not involved.** The output is a `Vec<u8>`. Nothing here is a
//!   host pointer, so the six-subsystem ceiling is untouched.
//! - **Determinism is stricter, not looser.** The image is a build artefact
//!   that a guest's state hash depends on: the same source must produce the
//!   same bytes on every host. So there is no branch relaxation, no hash-map
//!   iteration, and no host clock — the BIOS date is a constant, not
//!   `SystemTime::now()`.
//! - **Guest arithmetic wraps by definition** applies to the *emitted* code, and
//!   the emitted code is 16-bit: every address computed in the firmware wraps at
//!   64 KiB whether or not the Rust that emitted it does.
//! - **"Sizes and offsets are `u64`" is suspended.** A 16-bit segment offset is
//!   `u16` because the guest's silicon says so; widening it would hide exactly
//!   the wrap the guest depends on.
//!
//! # How the image is built, and why not the other ways
//!
//! [`asm16`] is a 16-bit x86 assembler written in Rust, and [`pcbios`] is a
//! Rust program that calls it. `cargo test` builds the ROM; there is no build
//! script, no external assembler, no linker script and no second toolchain.
//!
//! The alternatives were considered and lost:
//!
//! - **A `no_std` crate compiled for a 16-bit target.** Rust has no 16-bit x86
//!   target — LLVM's `x86_16` has never been one — so this would mean 32-bit
//!   code with `.code16` directives, which needs an assembler and a linker
//!   script, i.e. exactly the external toolchain `ROADMAP.md` §0 forbids. It
//!   also needs a second `cargo` invocation from a build script, which the same
//!   rule forbids.
//! - **A vendored `.bin` built out of tree.** That is a binary blob in an MIT
//!   repository whose source nobody can rebuild from `cargo`. It fails the
//!   reproducibility rule and it is the shape of the problem we are avoiding.
//! - **Host callbacks — a "BIOS" that is really a device trapping `INT 13h`.**
//!   Tempting, and much less code: a few `HLT`-shaped stubs in ROM and all the
//!   logic in Rust reaching into the guest's registers. Rejected because it is
//!   not firmware. It would not survive the JIT or a KVM backend without a
//!   bespoke exit path, it cannot be handed to real hardware or to another
//!   emulator to check, and it makes the CPU core's register file part of a
//!   device's ABI. The whole point of phase 6a is a ROM image that boots a PC,
//!   and an image that only boots *this* PC is not that.
//!
//! What that choice costs is honest to state: the firmware's logic is 16-bit
//! assembly, expressed in Rust rather than in an assembler's syntax. Rust
//! supplies the labels, the constants, the tables and the type checking of the
//! operand shapes; it does not supply registers or control flow. Anyone
//! expecting `fn int13(...)` in Rust will be disappointed, and no design that
//! produces a real ROM could have delivered that.

pub mod asm16;
pub mod pcbios;
