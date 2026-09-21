//! The embedder seam: who turns a generated module into something callable.
//!
//! [`abi`](super::abi) is the *contract* — the import names, the signatures,
//! the frame layout, the status codes. This file is the *interface to whoever
//! honours it*. There are two implementations and they differ in one respect
//! only:
//!
//! | implementation | where | what a module is |
//! | --- | --- | --- |
//! | [`exec`](super::exec) | every build | bytes this crate decodes and steps |
//! | [`crate::wasm`]'s `jit_*` imports | `wasm32-unknown-unknown` | a `WebAssembly.Module` the host engine compiled |
//!
//! The second is the one this backend exists for. `docs/techniques/wasm-jit.md`
//! has the argument in full; the short form is that a wasm interpreter running
//! wasm generated from IR is slower than interpreting the IR, so the reference
//! executor is correctness evidence rather than a speed path, and the only
//! host that can make this backend pay is one whose engine compiles the module
//! once and runs it many times.
//!
//! # Why the seam is a trait and not a `cfg`
//!
//! Because `jit/wasm/` is `no_std + alloc` and contains no host call of any
//! kind, and because keeping it that way is what lets the whole backend —
//! encoder, lowering, executor — be compiled and tested on every target in the
//! matrix. A `#[cfg(target_arch = "wasm32")]` branch inside
//! [`Engine::run`](super::rt::Engine::run) would put the one code path that
//! matters somewhere only one CI job compiles. A trait puts it behind a
//! function pointer, so the *routing* is exercised by a test double on x86-64
//! (`super::rt`'s `embedder_routing` tests) and only the host call itself is
//! target-specific.
//!
//! # What crosses, and what does not
//!
//! Three calls, and every argument is a number:
//!
//! * [`compile`](Embedder::compile) takes the module's **bytes** and returns
//!   an opaque handle, or [`REFUSED`] if the host engine would not take them.
//! * [`enter`](Embedder::enter) takes a handle, the linear-memory slice
//!   holding the frame, and the [`Env`] the module's imports call back into.
//! * [`release`](Embedder::release) drops a handle on eviction.
//!
//! **No guest pointer and no `&mut [u8]` of guest RAM crosses**, which is the
//! rule `CLAUDE.md`'s "Targets" section states and the reason guest RAM stays
//! addressable by byte offset. The slice `enter` receives is the *temporary
//! frame* — this engine's own scratch buffer — and nothing else.
//!
//! # The frame is the embedder's to locate
//!
//! [`Engine`](super::rt::Engine) hands `enter` a `&mut [u8]` and never says
//! where in the imported memory it lives, because that is a question only the
//! embedder can answer. The browser one imports **rsemu's own exported
//! memory**, so the frame's wasm offset is simply the slice's address in this
//! module's linear address space and the `$frame` parameter is that number.
//! An embedder holding a *separate* `WebAssembly.Memory` would have to copy
//! the frame in and out around the call; none does, and none should, which is
//! why the browser one imports rsemu's memory rather than making its own.
//!
//! The corollary that matters for correctness: the slice `enter` passes on to
//! [`Env::call`] must be **the same bytes** the generated module sees at
//! `$frame + 0`, with index 0 at `$frame`. `Thunks`'s `frame` field is zero
//! for exactly that reason, and `rt`'s doc comment on it says what goes wrong
//! if the two ever stop agreeing.

use super::exec::Env;

/// The handle a host returns when it will not take a module.
///
/// Zero rather than an `Option` because it also crosses the C ABI in
/// [`crate::wasm`], where a nullable handle is a number and nothing else. A
/// refusal is not an error: the block is executed by the reference executor
/// instead, exactly as a block this backend refuses to *lower* is executed by
/// [`Interp`](crate::ir::Interp).
pub const REFUSED: u32 = 0;

/// A host that can instantiate a generated module and call into it.
///
/// `Sync` because [`install`] stores one in a `static` and every hart's
/// [`Engine`](super::rt::Engine) reads it; `Debug` because CLAUDE.md asks for
/// it on every public type and an `Engine` holding one has to print.
pub trait Embedder: Sync + core::fmt::Debug {
    /// Compile and instantiate `module`, returning a handle or [`REFUSED`].
    ///
    /// `module` is a complete wasm binary as [`abi`](super::abi) describes it:
    /// one exported function named `b`, four imported functions and one
    /// imported memory, all in module `e`. The host is expected to instantiate
    /// it immediately — a handle names an *instance*, not a module, because
    /// there is nothing an uninstantiated module can be asked to do and
    /// deferring the instantiation would only move its cost.
    fn compile(&self, module: &[u8]) -> u32;

    /// Call handle `handle`'s `b` export with this engine's frame.
    ///
    /// `mem` is the frame buffer, with index 0 at the `$frame` the module is
    /// entered with. `env` is what the module's four imports call back into;
    /// the host must route them by [`func`](super::abi::func) index and
    /// nothing else.
    ///
    /// `None` means *this did not run* — a handle the host has forgotten, or
    /// a host that has gone away — and the caller falls back to the reference
    /// executor. It is not a guest-visible condition and must not be one:
    /// returning `None` after the module has already called an import would
    /// run the block twice and charge its ticks twice.
    fn enter(&self, handle: u32, mem: &mut [u8], env: &mut dyn Env) -> Option<i64>;

    /// Drop `handle`; the module behind it is unreachable.
    ///
    /// Called on eviction and when an [`Engine`](super::rt::Engine) is
    /// dropped. Never called with [`REFUSED`].
    fn release(&self, handle: u32);
}

/// The installed embedder, or `None` on a host that has none.
///
/// A `static` rather than a constructor argument because the thing that builds
/// an [`Engine`](super::rt::Engine) is `cpu::riscv::engine`'s `Jit::new`,
/// which is `no_std` core code and has no business knowing what a host is. The
/// host installs its embedder once at start-up and every engine built
/// afterwards picks it up.
///
/// Read **once per engine**, in
/// [`Engine::with_capacity`](super::rt::Engine::with_capacity), and cached in
/// the engine — so this lock is taken a handful of times in a process's life
/// and never on the path a block runs down.
static INSTALLED: crate::core::sync::Global<Option<&'static dyn Embedder>> =
    crate::core::sync::Global::new(None);

/// Install the host's embedder, for every [`Engine`](super::rt::Engine) built
/// from now on.
///
/// Idempotent in the sense that installing twice keeps the second; engines
/// already built keep the first, which is why a host does this before it
/// builds a machine. Returns what was there.
pub fn install(embedder: &'static dyn Embedder) -> Option<&'static dyn Embedder> {
    INSTALLED.lock().replace(embedder)
}

/// The installed embedder, if a host installed one.
#[must_use]
pub fn installed() -> Option<&'static dyn Embedder> {
    *INSTALLED.lock()
}

/// Forget the installed embedder. Tests only — a host installs once.
#[cfg(test)]
pub(crate) fn uninstall() -> Option<&'static dyn Embedder> {
    INSTALLED.lock().take()
}
