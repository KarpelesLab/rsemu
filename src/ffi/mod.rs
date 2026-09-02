//! The C ABI (`ROADMAP.md` §2, phase 9).
//!
//! The third of the tri-modal shape `purecrypto` and `kataan` established: a
//! Rust library, a C library, and a binary. Embedding rsemu into somebody
//! else's program — a CI runner, a test harness, a hardware bring-up tool, a
//! front end written in something that is not Rust — is a supported use rather
//! than a fork, and this module is the whole of it.
//!
//! # Build
//!
//! The crate stays `rlib` by default so a `no_std` build is not made to grow an
//! allocator and a panic handler it does not want. The C library is produced on
//! demand, the way the siblings do it:
//!
//! ```sh
//! cargo rustc --lib --release --features ffi --crate-type staticlib
//! cargo rustc --lib --release --features ffi --crate-type cdylib
//! ```
//!
//! The header is `include/rsemu.h`. It is **generated** from [`abi`] rather
//! than written by hand, and a test compares the two; see [`header`].
//!
//! # The ABI in one screen
//!
//! ```c
//! rsemu_handle cfg = rsemu_config_new();
//! rsemu_config_machine(cfg, "nes-ntsc", NULL, 0);   /* NULL source: catalog */
//! rsemu_config_media(cfg, "cart", rom, rom_len);
//!
//! rsemu_handle vm;
//! if (rsemu_machine_new(cfg, &vm) != RSEMU_OK) { /* rsemu_last_error(cfg,…) */ }
//! rsemu_free(cfg);
//!
//! rsemu_machine_run_ns(vm, 1000000000ull);         /* one second of guest time */
//!
//! uint64_t hash;
//! rsemu_machine_state_hash(vm, &hash);
//! rsemu_free(vm);
//! ```
//!
//! That is `rsemu run nes-ntsc --cart smb.nes` with nothing left out, which is
//! the bar §2 sets: if the C ABI cannot express what the binary already does it
//! is not an embedding surface.
//!
//! # Conventions
//!
//! Chosen to match the siblings rather than to be locally optimal, because a
//! caller who has embedded one Karpelès Lab crate should not have to relearn
//! the rules for the next.
//!
//! * **Status codes.** Every fallible call returns [`RsemuStatus`], `0` for
//!   success and negative for failure. The numeric values are part of the ABI.
//!   `RSEMU_OK`, `RSEMU_NULL_POINTER` and `RSEMU_BUFFER_TOO_SMALL` are `0`,
//!   `-1` and `-2` in all three crates.
//! * **Output buffers belong to the caller.** A call that produces bytes takes
//!   `(out, out_len)` where `*out_len` is the capacity on entry and the actual
//!   length on return — *always written*, so `(NULL, &0)` is a length query
//!   and `RSEMU_BUFFER_TOO_SMALL` tells you how much to allocate. rsemu never
//!   hands the caller a pointer it has to free.
//! * **Bytes are pointer + length; identifiers are NUL-terminated.** A media
//!   image, a machine description and a snapshot are `(ptr, len)` and need no
//!   terminator; a slot name, a parameter name and a machine name are
//!   `const char *`. `(NULL, 0)` is a valid empty blob; `(NULL, n > 0)` is
//!   `RSEMU_NULL_POINTER`.
//! * **Text is UTF-8 and is not NUL-terminated on the way out.** `*out_len`
//!   carries the length. Text on the way *in* is validated; invalid UTF-8 is
//!   `RSEMU_INVALID_INPUT`, never a silent replacement character.
//! * **The one string rsemu owns** is [`rsemu_strerror`]'s, which points into
//!   static storage, is NUL-terminated, and must not be freed.
//!
//! # Handles
//!
//! [`RsemuHandle`] is `uint64_t`, not a pointer, and `0` is never valid. It
//! indexes a process-wide table of live objects; the ids come from a counter
//! that never repeats. This is the one place the C ABI **departs from the
//! siblings**, which hand out `Box::into_raw` pointers and document "do not
//! free twice", and the reason is what the two APIs are made of: a
//! `pc_hash_free` on a stale pointer corrupts a hash, an [`rsemu_free`] on a
//! stale pointer corrupts a whole emulated machine in the middle of a run, and
//! the failure surfaces a million guest instructions later.
//!
//! What the table buys, precisely:
//!
//! * **A double free is an error, not undefined behaviour.** The second
//!   [`rsemu_free`] finds no entry and returns `RSEMU_BAD_HANDLE`.
//! * **A use-after-free is an error too**, and for the same reason.
//! * **A forged or uninitialised handle is an error.** No caller-supplied
//!   integer is ever dereferenced, so there is no value of [`RsemuHandle`] that
//!   can make rsemu touch memory it does not own.
//! * **Type confusion is an error.** A config handle passed where a machine
//!   was wanted is `RSEMU_BAD_HANDLE`, because the table knows which is which.
//! * **Ids are never recycled**, so the ABA problem a pointer table would have
//!   does not exist: a `u64` counter incremented once per created object does
//!   not wrap in any run that finishes.
//!
//! What it costs is one lock and one map lookup per call, held for the lookup
//! and released before the machine runs. Against a call that advances virtual
//! time by milliseconds that is not measurable, and the two calls where it
//! might be — [`rsemu_machine_read`] and [`rsemu_machine_write`] — are debug
//! accesses, not the guest's own bus traffic.
//!
//! Handles are safe to use from several threads: calls on *one* handle
//! serialise on that object's lock, and different handles are independent. The
//! siblings' rule ("the caller must serialise with a `pthread_mutex_t`") is not
//! needed here.
//!
//! # Guest memory
//!
//! [`rsemu_machine_read`] and [`rsemu_machine_write`] **copy**. `CLAUDE.md` is
//! explicit that guest RAM is addressed by byte offset and never handed out as
//! a slice, so that it can live in a `SharedArrayBuffer`; a C ABI that returned
//! a pointer into guest RAM would break that for every target at once, and
//! would additionally hand out a pointer that the next remap, snapshot restore
//! or safe point is entitled to invalidate. Both calls use
//! [`MemAttrs::DEBUG`](crate::core::space::MemAttrs::DEBUG), so reading a
//! device register through this ABI does not pop a FIFO or clear a status bit
//! — the same contract the gdb stub and the monitor get.
//!
//! # Panics
//!
//! Every entry point runs its body inside `std::panic::catch_unwind`, and a
//! caught panic becomes `RSEMU_PANIC` rather than an unwind into C. Three
//! things are worth being precise about, because the folklore here is wrong in
//! two directions:
//!
//! * Since Rust 1.81 a panic that escapes an `extern "C"` function **aborts**;
//!   it is not undefined behaviour. The boundary is therefore sound with or
//!   without the guard. What the guard buys is that the embedder gets an error
//!   code and stays alive instead of losing the process.
//! * Under `-C panic=abort` the guard is inert — the abort happens at the
//!   panic site and `catch_unwind` never runs. Still sound, still fatal. An
//!   embedder that wants the error code must link an unwinding build, which is
//!   the default; `kataan`'s manifest documents the same constraint.
//! * `no_std` does not enter into it. Whether a panic unwinds is a property of
//!   the final artifact's panic strategy, not of whether this crate names
//!   `std` — a `#![no_std]` rlib linked into an unwinding binary unwinds. What
//!   is true is that `catch_unwind` lives in `std`, which is why the `ffi`
//!   feature implies `std` and why there is no `no_std` C ABI. The `no_std`
//!   build simply does not contain this module, and `--no-default-features`
//!   builds exactly as it did before.
//!
//! A machine whose call panicked is **poisoned**: its state was borrowed
//! mutably when the unwind started, so nothing may be assumed about it. Every
//! subsequent call on that handle returns `RSEMU_PANIC`; [`rsemu_last_error`]
//! and [`rsemu_free`] still work, so the embedder can report and clean up.
//!
//! # Errors
//!
//! `CLAUDE.md` mandates one crate-level [`Error`](crate::Error) and C has no
//! enums with payloads, so the mapping is split in two and loses nothing:
//! [`RsemuStatus`] carries the *class* — one code per `Error` variant, and one
//! per [`BusError`](crate::core::BusError) variant, so a caller can retry on
//! `RSEMU_BUS_RETRY` without matching on a string — and [`rsemu_last_error`]
//! carries the `Display` text, which for a configuration error is the
//! `file:line:col` and the caret §5 promises. The text is written into the
//! caller's buffer, so there is nothing to free.
//!
//! # What is deliberately not here
//!
//! No file paths and no file I/O: a machine description and a media image
//! arrive as bytes the embedder read, so this module has no encoding question
//! to get wrong and no sandbox to enforce. No display, audio, input, console,
//! gdb or record/replay surface — each of those is a `host/` seam with a shape
//! of its own, and guessing at their C spelling before anyone has embedded one
//! is how an ABI acquires functions it can never remove. No device
//! introspection beyond the address spaces, because the surface §4.5 already
//! promises for reading a device's state from outside is its snapshot chunk,
//! which [`rsemu_machine_save`] hands over whole.
//!
//! # `unsafe` in this module
//!
//! This is the C ABI, one of the six subsystems `ROADMAP.md` §0 sanctions to
//! opt back in, and the allow below is module-scoped. Every `unsafe` block
//! carries its own `// SAFETY:` argument naming who upholds the invariant;
//! in a C ABI the answer is nearly always "the caller", and saying so at each
//! site is the point of the rule.
//!
//! **This is not a seventh subsystem.** `src/wasm.rs` already carries the
//! allow, and it is the same sanctioned one: rsemu's C ABI has two boundaries,
//! a browser module with a single machine in a module-wide slot and this one,
//! with handles and several. Two files, one entry on §0's list of six, and the
//! count of *occupied* sites is unchanged.
//!
//! # Naming, and why the linker enforces it
//!
//! Both boundaries export `rsemu_*`, so a name used by one is unavailable to
//! the other. That is not a hazard to remember: `--all-features` builds both
//! into one artifact, and a duplicate `#[unsafe(no_mangle)]` is a hard error
//! there. The shape that keeps them apart is the siblings' own —
//! `<prefix>_<area>_<verb>`, as `pc_hash_update` and `pc_tls_cfg_set_alpn` are
//! — so this module owns `rsemu_config_*`, `rsemu_machine_*` and
//! `rsemu_catalog_*`, and `wasm` owns the bare verbs it claimed first.
//!
//! The prefix is `rsemu_` rather than the two-letter `pc_`/`kt_` the siblings
//! use, because `src/wasm.rs` established it for this crate first and one C ABI
//! wearing two prefixes would be worse than a long one.
#![allow(unsafe_code)]

pub mod abi;
mod common;
pub mod header;

#[cfg(test)]
mod tests;

// A glob rather than a list: `abi` is the ABI by definition, so an entry point
// that exists there and not here would be a re-export that went stale rather
// than a deliberate omission.
pub use crate::ffi::abi::*;
