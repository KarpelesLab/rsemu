//! The aarch64 host backend: [`Block`](crate::ir::Block) in, machine code out.
//!
//! `ROADMAP.md` §9 names "`aarch64` + `riscv64` backends" as a deliverable
//! after the x86-64 one, and this is the first half of it. Until it landed
//! rsemu had exactly **one** host backend, `cfg`-gated to x86-64 Linux, so
//! every Apple-silicon developer, every Arm server and this project's own
//! `aarch64 (weak memory)` CI job ran `engine = "jit-host"` and got the IR
//! interpreter — correct, and several times slower than the machine could go.
//!
//! # The five files, and which of them need an A64 machine
//!
//! | file | what it is | needs the host | `unsafe` |
//! | --- | --- | --- | --- |
//! | `emit` | an A64 assembler over a `Vec<u8>` | no | no |
//! | `compile` | one IR block lowered to that assembler, over the homes [`ir::linear_scan`](crate::ir::linear_scan) chose | no | no |
//! | `abi` | the context, the thunk table and the deferred bookkeeping | no | **yes** |
//! | `buf` | the W^X `mmap`/`mprotect` buffer, and A64's cache maintenance | **yes** | **yes** |
//! | `rt` | entering the code | **yes** | **yes** |
//!
//! **Three of the five compile everywhere**, and that is a deliberate
//! difference from [`jit::x86`](crate::jit::x86), whose whole module is gated
//! to its host. Encoding an instruction and lowering a block are arithmetic on
//! integers; only *executing* the result needs the machine. Splitting it that
//! way means the encoding assertions, the refusal coverage and the register
//! allocator's invariant are checked on **every** CI job that turns the
//! feature on, including the x86-64 ones — which matters a great deal for a
//! backend whose functional tests can only run on one runner.
//!
//! The two that opt into `unsafe` plus `abi` are one subsystem — *the JIT
//! code buffer*, CLAUDE.md's second sanctioned site — and this adds **no new
//! site**: it is the same subsystem implemented for a second host, with the
//! same three obligations (map the memory, cross the boundary, and — new here
//! — make the bytes visible to instruction fetch). Everything else in this
//! module, and all of `ir/`, is safe Rust.
//!
//! # What is different from the x86-64 backend, and what is not
//!
//! Not different, and this is the point of having an IR at all: the register
//! allocator ([`ir::linear_scan`](crate::ir::linear_scan)), the deferred
//! bookkeeping and its region rule, the write-through that makes exceptions
//! precise, the canonical masking contract, the refusal seam, the `Ctx`
//! layout and the thunk table. A reader who knows one backend knows the shape
//! of the other.
//!
//! Different, and each because of the architecture rather than taste:
//!
//! * **Cache maintenance.** A64's instruction cache is not architecturally
//!   coherent with its data cache, so writing bytes and then `mprotect`ing
//!   them is *not* enough. `buf` runs the `DC CVAU` / `DSB` / `IC IVAU` /
//!   `DSB` / `ISB` sequence DDI 0487 requires, over the range that was
//!   written, once per seal. x86-64 needs none of it.
//! * **The barrier.** [`Opcode::FENCE`](crate::ir::Opcode::FENCE) is `DMB ISH`
//!   here and `MFENCE` there. On x86-64 the fence was buying one reordering
//!   back (store-then-load); here it is buying all of them, because A64 is
//!   weakly ordered. `docs/techniques/memory-models.md` is where that argument
//!   lives, and the `aarch64` CI job exists because of it.
//! * **Ten allocatable registers against seven**, seven of them callee-saved
//!   against three. See `compile`'s register table.
//! * **No immediate is free.** A 64-bit constant is up to four `MOVZ`/`MOVK`
//!   and a memory offset is a scaled 12-bit field, so the lowerings shift
//!   twice where x86 masks once and this backend *refuses* a block with more
//!   than 4095 temporaries instead of reaching a frame slot with a `disp32`.
//! * **Branchless where x86 branches**: an out-of-range shift is a `CSEL`
//!   rather than a compare and two jumps, and a `brcond` on a one-bit
//!   selector is a single `TBNZ`.
//!
//! # Where this is not available
//!
//! `mmap`, `mprotect` and `SVC` mean nothing off Linux, and wasm has no
//! writable-then-executable memory at all (`ROADMAP.md` §11.4). So `buf` and
//! `rt` are `cfg`-gated to aarch64 Linux and turning the feature on
//! elsewhere costs a compile of the encoder and nothing else.
//!
//! **macOS is deliberately not attempted.** Apple silicon needs `MAP_JIT` at
//! `mmap` time, an entitlement on the binary, and
//! `pthread_jit_write_protect_np` to flip a *thread's* write protection rather
//! than a page's — a different model from W^X-by-`mprotect`, reached through a
//! libc function this crate's dependency policy does not allow it to link, and
//! with a per-thread state machine that the `CodeBuf` design here does not
//! express. It is a separate piece of work with its own design review, not a
//! `cfg` on this one. Until then an Apple-silicon build gets what it has
//! always had, the IR interpreter.
//!
//! # What has not been measured
//!
//! **Nothing.** There is no throughput number in this module and there will
//! not be one until it is taken on an aarch64 host: this backend was written
//! on an x86-64 machine, where it cannot execute a single instruction it
//! emits. `benches/jit_dispatch.rs` and `benches/a64_dispatch.rs` already have
//! the `+compiled` column that would answer it — they are gated on
//! `jit-x86` and would need the same `cfg` widened — and the honest state of
//! the question is that the mechanism is in place and the number is unknown.
//! Saying otherwise would be inventing it.

pub mod abi;
pub mod compile;
pub mod emit;

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
#[cfg_attr(docsrs, doc(cfg(target_arch = "aarch64")))]
pub mod buf;

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
#[cfg_attr(docsrs, doc(cfg(target_arch = "aarch64")))]
pub mod rt;

#[cfg(test)]
mod tests;

pub use abi::{Ctx, Event, Vtable};
pub use compile::{Compiled, Refusal, Regs, compile, compile_with, compiles};

#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
pub use buf::{CodeBuf, DEFAULT_CAPACITY};
#[cfg(all(target_os = "linux", target_arch = "aarch64"))]
pub use rt::{Engine, EngineStats};
