//! The WebAssembly backend: [`Block`](crate::ir::Block) in, a
//! `WebAssembly.Module` out.
//!
//! `ROADMAP.md` §11.4, *"The JIT without `mmap`"*, is the brief:
//!
//! > wasm has no writable-then-executable memory, so the native code path is
//! > simply unavailable. **The JIT emits WebAssembly instead**: IR → wasm
//! > bytecode module → `WebAssembly.Module` → dispatched through a function
//! > table. […] a translation block is a wasm function, guest RAM is the
//! > shared linear memory, and helper calls are imports.
//!
//! All of that is here except the function table, and `docs/techniques/wasm-jit.md`
//! is why: a table dispatch is how an *embedder* re-enters a compiled block,
//! and this build re-enters it in Rust. The rest of §11.4 is kept as written —
//! one function per block, imports for everything that touches the host, a
//! bounded module table with eviction, and the interpreter as the fallback for
//! any block this backend refuses.
//!
//! **There are two things that can run a generated module.** [`exec`](crate::jit::wasm::exec) is a
//! wasm interpreter over the emitted subset and ships everywhere, so the
//! backend executes — and is hashed against `ir::Interp` — on every target.
//! [`Embedder`](crate::jit::wasm::embed::Embedder) is the seam a *host* engine plugs into, and
//! [`crate::wasm`] implements it for `wasm32-unknown-unknown`, where the
//! page's own `WebAssembly` compiles each module once. Which one ran is a
//! statistic ([`EngineStats::embedded`](crate::jit::wasm::rt::EngineStats::embedded))
//! and never a behaviour.
//!
//! # The six files
//!
//! | file | what it is | needs an embedder | `unsafe` |
//! | --- | --- | --- | --- |
//! | [`emit`](crate::jit::wasm::emit) | a wasm binary encoder over a `Vec<u8>` | no | no |
//! | [`abi`](crate::jit::wasm::abi) | the contract generated code is compiled against | no | no |
//! | [`compile`](mod@crate::jit::wasm::compile) | one IR block lowered to that encoder | no | no |
//! | [`exec`](crate::jit::wasm::exec) | the reference executor: a wasm interpreter over the emitted subset | no | no |
//! | [`embed`](crate::jit::wasm::embed) | the seam a host instantiates modules through | **it is one** | no |
//! | [`rt`](crate::jit::wasm::rt) | the module table, the imports, and the run | no | no |
//!
//! **No `unsafe` anywhere, and no host gate anywhere.** Both are worth saying
//! plainly, because both are differences from the two native backends rather
//! than omissions. `jit::x86` and `jit::arm64` opt into CLAUDE.md's second
//! sanctioned `unsafe` site — the JIT code buffer — because machine code has
//! to be reached through a raw pointer and has to reconstitute `&mut`
//! references to call back. A wasm module is a byte vector, entering it is a
//! safe function call, and an import's arguments are integers, so this backend
//! adds **no new `unsafe` and does not touch the existing seven**. And nothing
//! here executes host instructions, so unlike `jit::x86` — whose whole module
//! is `cfg`-gated to x86-64 Linux — every file compiles, and is tested, on
//! every target in the matrix.
//!
//! That survives the browser embedder. [`embed`](crate::jit::wasm::embed) is a
//! trait whose arguments
//! are a byte slice, a `&mut [u8]` and a `&mut dyn Env`; the raw pointer a
//! browser needs to route an import back into that `Env` is minted and
//! consumed entirely inside [`crate::wasm`], which is the `ffi` site CLAUDE.md
//! already sanctions. Not an eighth site, and nothing in `jit/` names one.
//!
//! # What §11.4 predicted, and what is now measurable
//!
//! §11.4 is unusually pessimistic about its own plan, and it is right to be:
//!
//! > **Block chaining is impossible here.** You cannot patch a jump in an
//! > instantiated wasm module, so every block exit returns through a
//! > `call_indirect` dispatcher — which removes the second-largest win in §9's
//! > list.
//!
//! Kept, and sharpened. There is no [`Chain`](crate::jit::Chain) for this
//! backend and there cannot be one: [`Engine::run`](crate::jit::wasm::rt::Engine::run) executes
//! exactly one block and returns, so a `goto_tb` costs a full return to
//! [`Dispatcher`](crate::jit::Dispatcher). What that loses is the *direct
//! link*; what it keeps is everything the block cache already does — the
//! keyed lookup, the patched exits at the cache level, the page-dirty
//! invalidation — because all of that is `no_std` Rust above the backend and
//! knows nothing about how a block executes. `docs/techniques/wasm-jit.md` has
//! the two alternatives that were considered (a superblock-shaped module, and
//! a module holding a whole trace with an internal dispatch loop) and why
//! neither is worth building before an embedder exists to measure against.
//!
//! # Invalidation without a patch
//!
//! The native backends invalidate by *unpatching* a predecessor's jump. There
//! is nothing to unpatch here, and there does not need to be: a module is only
//! ever reached through [`BlockCache`](crate::jit::BlockCache), so a block the
//! cache drops is a module nothing can reach. Three things drop one, and they
//! are the three the rest of `jit/` already has:
//!
//! * a **guest store** into the page the block was lifted from — the dirty
//!   log, drained at every block boundary;
//! * a **topology bump**, which flushes the whole cache;
//! * **eviction** from this backend's own module table, which bumps that
//!   slot's generation so the [`CodeRef`](crate::jit::CodeRef) naming it stops
//!   being live and the dispatcher compiles again.
//!
//! No third mechanism, and no window in which a stale module is reachable,
//! because the reachability question is answered one level up.
//!
//! # Determinism
//!
//! The claim is `ROADMAP.md` §0's: a compiled run and an interpreted one are
//! indistinguishable to the guest, *including* cycle counts. Two places in
//! this backend are where that would be lost, and both are handled rather than
//! hoped for.
//!
//! * **Shifts.** wasm reduces a shift count modulo the operand width (core
//!   specification §4.4.1); [`Interp`](crate::ir::Interp) takes the
//!   mathematical answer. The IR calls an out-of-range shift undefined, but
//!   *undefined* is not *whatever the host does* when the same block may run
//!   compiled on one pass and interpreted on the next — so
//!   [`compile`](mod@crate::jit::wasm::compile) selects the interpreter's answer
//!   explicitly.
//! * **Floating point.** There is no float instruction in the emitted subset
//!   at all, and both float types are refused. Guest FP is a helper call into
//!   `float::soft` (`ROADMAP.md` §9.1) precisely so a guest's NaN payloads and
//!   rounding never become the host's, and a wasm host is a host like any
//!   other.
//!
//! The evidence is `tests/riscv_virt_engines.rs`, which runs the same board
//! under `engine = "interp"`, `"jit"`, `"jit-host"` and `"jit-wasm"` and
//! asserts one hash at every checkpoint — including a snapshot taken under one
//! engine and carried on under another — and, since the embedder landed,
//! `web/check.mjs` §1c, which asserts the same thing with the page's own
//! engine running the modules. The first three only ever put
//! [`exec`](crate::jit::wasm::exec) behind them, so on their own they check
//! this project's reading of the core specification; the fourth is what
//! removes that qualifier.
//!
//! # Speed: two numbers, because there are two executors
//!
//! On a native host this backend is **slower than the IR interpreter**, and it
//! is meant to be: with no embedder there is nothing to run a module but
//! [`exec`](crate::jit::wasm::exec), so a block is interpreted twice over. In
//! a browser the page's own engine compiles each module once, which is what
//! the backend is for. The same guest for the same span, both ways:
//!
//! | engine | native ([`exec`](crate::jit::wasm::exec)) | in V8 |
//! | --- | --- | --- |
//! | `interp` | 1064 ms, 1.00× | 922 ms, 1.00× |
//! | `jit` | 517 ms, 2.06× | 840 ms, 1.10× |
//! | `jit-host` | 85 ms, 12.50× | — no `mmap` |
//! | `jit-wasm` | 1392 ms, **0.76×** | 613 ms, **1.50×** |
//!
//! So `ROADMAP.md` §11.4's question is answered: a wasm module *is* faster
//! than the IR interpreter in a browser, per basic block, with no chaining and
//! no superblocks — which is earlier than §11.4 expected it to pay off. The
//! native column is the floor rather than the backend, and what it buys is
//! that the translation is *executed* everywhere rather than merely encoded,
//! which is what lets the determinism gate above run on the x86-64 runner that
//! gates every commit — a claim `jit::arm64` explicitly cannot make.
//!
//! Both columns come from one function, [`crate::wasm::rsemu_jit_guest_run`],
//! so they are one workload rather than two with one name:
//! `benches/wasm_jit_embedder.rs` takes the left and `web/check.mjs` §1c takes
//! the right. `docs/techniques/wasm-jit.md` has the fixture, the caveats and
//! the three mechanisms that would move the number.

pub mod abi;
pub mod compile;
pub mod embed;
pub mod emit;
pub mod exec;
pub mod rt;

#[cfg(test)]
mod tests;

pub use compile::{Compiled, Refusal, compile, compiles};
pub use embed::{Embedder, REFUSED, install, installed};
pub use exec::{Env, ExecError, Program};
pub use rt::{DEFAULT_MODULES, Engine, EngineStats};
