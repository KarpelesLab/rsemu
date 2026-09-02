//! The x86-64 host backend: [`Block`](crate::ir::Block) in, machine code out.
//!
//! `ROADMAP.md` §9 lists `x86_64` first among the backends, and §9.1 puts the
//! code generator *after* the software TLB and the block cache — which is why
//! this landed after them and not before. All three are now here, and the
//! third is the one that makes the first two worth having: a block executed
//! through [`ir::Interp`](crate::ir::Interp) spends most of its time in
//! opcode dispatch, so a chained cache in front of an interpreter saves
//! dispatches that were never the expensive part.
//!
//! # The four files
//!
//! | file | what it is | `unsafe` |
//! | --- | --- | --- |
//! | [`emit`] | an x86-64 assembler over a `Vec<u8>` | no |
//! | [`compile`](mod@compile) | one IR block lowered to that assembler | no |
//! | [`buf`] | the W^X `mmap`/`mprotect` code buffer | **yes** |
//! | [`rt`] | the context, the thunks, and entering the code | **yes** |
//!
//! The two that opt in are one subsystem — *the JIT code buffer*, §0's second
//! sanctioned site — split so that *mapping memory* and *crossing the
//! boundary* each state their invariants next to the code that keeps them.
//! Everything else in `jit/`, and all of `ir/`, is safe Rust.
//!
//! # What compiles, and what does not
//!
//! [`compiles`] is the list, and it is the union of what the RISC-V and x86
//! frontends actually emit plus the neighbours that cost nothing once their
//! family is in. A block containing anything else is **refused** and runs on
//! the interpreter, which is not a compromise: `ROADMAP.md` §9 asks for a
//! portable interpreter backend precisely *"so an unsupported host degrades in
//! speed rather than failing to run"*, and the same seam serves an unsupported
//! *block*. A [`Refusal`] always names what stopped it, because a backend
//! whose coverage is unmeasured is a backend whose coverage rots.
//!
//! Refused today: the atomics and `fence`, `call_helper`, `phi`, the divides,
//! `addc`/`subb`, `mulhsu`, anything wider than 64 bits, and both float types.
//! Every one of them is either a seam this backend deliberately does not
//! inline, or an op no frontend in the tree emits — and lowering an op nothing
//! emits would mean shipping code generation the differential harnesses cannot
//! reach, which is worse than shipping none.
//!
//! # The software TLB, inlined
//!
//! §9.1 says of the TLB that *"the fast path is inlined into generated code:
//! mask, compare, add, load"*, and that everything else about the JIT is
//! secondary to it. It is, for a load whose address the frontend hands over
//! unsegmented: [`Tlb::fast_set`](crate::jit::Tlb::fast_set) publishes the
//! entry array's base and mask, [`Tlb`](crate::jit::Tlb) precomputes a **host
//! address** per entry, and the generated sequence is a null check, an
//! alignment test, the index, the tag compare, an add, and the `mov`. A miss,
//! an uncacheable page, a misaligned address or a big-endian region branches
//! to the host's own path, which is the one that fills the entry.
//!
//! A host opts in by implementing [`FastMem`](crate::jit::FastMem), and what
//! it publishes is *its own* TLB — not a second one — so a hit and a miss
//! resolve through the same entries and a fill made either way is seen by
//! both. What the host still owes is the tick:
//! [`FastMem::note_fast_load`](crate::jit::FastMem::note_fast_load) is called
//! for every inlined access, and must charge exactly what that host's own
//! aligned access charges, or compiled and interpreted execution stop agreeing
//! on the cycle counter (`ROADMAP.md` §0).
//!
//! **Stores are not inlined**, and [`compile`](mod@compile)'s `store` says why: a store owes
//! the guest-physical dirty log the block cache drains for self-modifying code,
//! and the `RamStore`'s own dirty bitmap. Both are invisible from generated
//! code, so `RamStore::host_ptr` is read-only and stores keep the call.
//!
//! # Where this is not available
//!
//! wasm has no writable-then-executable memory (`ROADMAP.md` §11.4), and a
//! `syscall` instruction means nothing off Linux. So the whole module is
//! `cfg`-gated to `x86_64` Linux, exactly as `accel` is, and turning the
//! feature on elsewhere is harmless rather than a build break. §11.4's answer
//! for the browser is a *wasm* backend emitting a module, which is a different
//! file and is not this one.

pub mod buf;
pub mod compile;
pub mod emit;
pub mod rt;

#[cfg(test)]
mod tests;

pub use buf::{CodeBuf, DEFAULT_CAPACITY};
pub use compile::{Compiled, Refusal, compile, compiles};
pub use rt::{Ctx, Engine, EngineStats, Vtable};
