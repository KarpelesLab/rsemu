//! The translation runtime: the software TLB, the block cache, block chaining
//! and self-modifying-code detection.
//!
//! `ROADMAP.md` §9.1 closes with the list of *"the mechanisms that actually
//! produce speed"*, and is unusually direct about the order:
//!
//! > 1. **Software TLB** — per-CPU, direct-mapped (4096 entries), split by
//! >    access type, entry = `{ guest page tag, host addend | IO slot }`. […]
//! >    Everything else about the JIT is secondary to this.
//! > 2. **Translation block cache** keyed by `(guest PC, relevant CPU flags)`,
//! >    with **block chaining** […]
//! > 3. **Self-modifying code** — page dirty bitmap […]
//!
//! Those three are this module, and they are all reachable with
//! [`ir::Interp`](crate::ir::Interp) as the executor — a chained block cache in
//! front of the portable backend is a real result, where half a host code
//! generator is not. **There is no host code generation here yet.** When there
//! is, it slots in under [`Dispatcher`] as a different way to execute a
//! [`Block`](crate::ir::Block), and every mechanism here is already the shape
//! it needs.
//!
//! # Where this sits, and why it is `no_std`
//!
//! `ROADMAP.md` §0 puts `jit/` above the `std` line, alongside `host/` and
//! `accel/`, because emitting native code needs W^X `mmap` through raw
//! syscalls and emitting wasm needs an embedder import. **Nothing in this
//! round needs either**, so this module is `no_std + alloc` like the IR it
//! serves, and §11's bare-metal row — whose engine is the IR interpreter —
//! gets the TLB and the block cache too rather than being the one target that
//! runs everything cold. The `std` line moves when the first backend lands,
//! and it moves in the file that needs it.
//!
//! For the same reason there is **no `unsafe` here**. The one sanctioned
//! opt-in in this subsystem is the JIT *code buffer* (CLAUDE.md, "`unsafe`"),
//! and there is no code buffer yet. Guest RAM is still reached by byte offset
//! through [`RamStore`](crate::core::space::RamStore) — never as a
//! `&mut [u8]` — so the TLB's "host addend" is an addend into a store rather
//! than a host pointer, which is what keeps it working when guest RAM is a
//! `SharedArrayBuffer` (`ROADMAP.md` §11.2).
//!
//! # One answer to "stale", shared by both caches
//!
//! The hard part of a translation cache is never the lookup. Two counters
//! decide staleness here, and the TLB and the block cache consult the same
//! two rather than inventing separate answers:
//!
//! | Counter | Bumped by | Invalidates |
//! | --- | --- | --- |
//! | [`Epoch::translation`] | `SFENCE.VMA`, a `satp` write, an `mstatus` change that alters translation — `Csrs::translation_gen`, the counter `cpu::riscv::lift`'s `Origin::Paged` already folds into [`Block::key`](crate::ir::Block::key) | every [`Tlb`] entry; cached blocks are **keyed** on it, so a stale block is unreachable rather than wrong |
//! | [`Epoch::topology`] | [`AddressSpace::generation`](crate::core::space::AddressSpace::generation) — a map, unmap, remap, reprotect or replace | every [`Tlb`] entry **and** every cached block |
//!
//! The second row is the one that is easy to get wrong, and it is a real hole
//! the frontend's key does not cover: `Origin::Bare` contributes *nothing* to
//! [`Block::key`](crate::ir::Block::key), so a machine-mode block lifted
//! through a mapping that is later unmapped, remapped or shadowed keys
//! identically to one lifted after. The block cache therefore flushes on a
//! topology bump ([`BlockCache::sync`]) rather than trusting the key, and
//! `a_topology_change_invalidates_every_cached_block` is the test that fails
//! if that goes away. Widening the key would have worked equally well;
//! flushing was chosen because that counter is already what every other
//! derived cache in the crate invalidates on (`ROADMAP.md` §4.1), and because
//! a retopology is rare enough that throwing the cache away is free.
//!
//! Neither counter covers a **rebase** — a cartridge bank switch slides an
//! alias's offset ~15 000 times a second and deliberately bumps nothing
//! (`core::space`, "Two kinds of change"). A [`Tlb`] entry over a rebasable
//! leaf would go silently stale, so the TLB refuses to cache one at all and
//! marks the page uncacheable instead; see [`Tlb::fill`].
//!
//! # Determinism
//!
//! A cache hit and a cache miss are indistinguishable to the guest, including
//! in cycle accounting (`ROADMAP.md` §0). Nothing here calls
//! [`IrHost::charge`](crate::ir::IrHost::charge): the TLB caches the *host*
//! resolution of an address the guest's own MMU has already translated and
//! charged for, and the block cache caches the result of lifting, which the
//! guest cannot observe at all. The guest-visible TLB — the one whose misses
//! cost a walk and therefore ticks — is the core's own (`cpu::riscv::mmu`),
//! and it is a different object on purpose.
//!
//! No iteration order here reaches the guest: the block cache's page index is
//! a [`BTreeMap`](alloc::collections::BTreeMap), its bucket chains are in
//! insertion order, and eviction is FIFO.
//!
//! # Soundness under threads
//!
//! The TLB's fast path does **not** take the address space's read guard, which
//! is most of what it buys. That is sound exactly as far as the safe-point
//! protocol reaches: a retopology from another thread must stop the world
//! ([`SafePoint::request`](crate::core::sched::SafePoint::request)) so that
//! every CPU has left its block before the mapping moves, and every CPU calls
//! [`Tlb::sync`] and [`BlockCache::sync`] at its next block boundary. A
//! [`Dispatcher`] carrying an [`ExitFlag`](crate::core::sched::ExitFlag)
//! unwinds at that same boundary, which is the other half. Never a signal —
//! wasm has none.

mod cache;
mod dispatch;
mod tlb;

pub use cache::{BlockCache, BlockId, CacheStats, DEFAULT_CAPACITY, EXITS};
pub use dispatch::{
    DirtyPages, DispatchStats, Dispatcher, Frontend, Run, Stop, StoreLog, Translation,
};
pub use tlb::{Context, DEFAULT_ENTRIES, Epoch, PAGE_MASK, PAGE_SIZE, Tlb, TlbStats};
