# Design rules for rsemu

Read this before writing any code. [`ROADMAP.md`](ROADMAP.md) is the *what* and
the phase order; this file is the *how*. Where the two disagree, the roadmap's
non-negotiables (§0) win.

## Crate shape

- One crate, `rsemu`. Every component is a Cargo **feature**, not a separate
  crate: one per CPU core (`cpu-mos6502`), one per device (`dev-nvme`), one per
  bus (`bus-pci`), one per board (`machine-nes`). A NES build links a 6502 and
  nothing else.
- `src/core/` is never feature-gated — it is the framework and it always
  compiles. Everything else lives under `cpu/`, `dev/`, `boards/`, `ir/`,
  `jit/`, `accel/`, `machine/`, `host/`.
- `src/lib.rs` contains feature-gated re-exports and nothing else.
- Features are declared in `Cargo.toml` **with a comment explaining what the
  feature gets you** — see `compcol`'s manifest for the standard.

## Dependency policy

- Default build: `cargo tree` shows only `rsemu`. This is checked in CI.
- Permitted dependencies are first-party and feature-gated only: `pktkit`,
  `compcol`, `purecrypto`, `puremp`, `noroi`. Nothing else — no `serde`, no
  `libc`, no async runtime, no GUI toolkit.
- OS interaction is by raw syscall (the `purestd` pattern), not via `libc`.
- The two exceptions that break purity — macOS Hypervisor.framework and Windows
  WHPX — are opt-in features labelled as such in the README. Never silent.

## `no_std`

- `core/`, `ir/`, `cpu/`, `dev/`, and `machine/` are `no_std + alloc`. `std`
  only appears under `host/`, `jit/`, and `accel/`.
- CI builds `--no-default-features --features alloc`. A `std` leak into the
  emulation core is a build break, not a style nit.

## `unsafe`

- Crate-wide `unsafe_code = "deny"` (not `forbid`), so four subsystems can opt
  back in with a scoped `#[allow(unsafe_code)]`: the RAM host-pointer fast
  path, the JIT code buffer, the raw-syscall accel backends, and `ffi`.
- Every `unsafe` block carries a `// SAFETY:` comment stating the invariant and
  who upholds it. No exceptions, including in the hot path.

## Type conventions

- Guest addresses are `u64` (`GuestAddr`), guest-physical and guest-virtual are
  **distinct newtypes** — mixing them is the classic emulator bug.
- Sizes and offsets are `u64`, never `usize`. A 64-bit guest on a 32-bit host
  must still work.
- Extensible enumerations (device classes, memory-op codes, IR opcodes) are
  `#[repr(transparent)] pub struct Foo(pub u16)` with `pub const` variants —
  the `pktkit` `EtherType` pattern. Real enums only where exhaustiveness is
  genuinely wanted.
- Errors: one crate-level `Error` enum + `Result<T>`. Memory accesses return
  `MemResult`, which distinguishes *ok*, *bus fault*, and *retry* — never
  `Option`, never a silent `0xFF`.
- `#[derive(Debug)]` on every public type, or a manual impl when a field is not
  `Debug`.

## Concurrency

- No async. Devices are `Send + Sync` with synchronous methods.
- Shared mutable device state: `Mutex`/`RwLock` for cold paths, atomics for
  hot ones. Prefer designing the hot path to need neither.
- Background work is `std::thread::spawn`, and only under `host/`.
- The scheduler owns time. A device never sleeps, never reads the wall clock,
  and never spawns a thread to "tick" itself — it registers an event.

## Determinism

- No `HashMap` iteration order in anything that affects guest-visible state.
  Use `BTreeMap` or an insertion-ordered map.
- No floats in the time path. Rational integer arithmetic only.
- No wall-clock reads outside `host/` and the rate controller.
- Any non-deterministic input crossing into the machine goes through the
  record/replay seam, or it is a determinism bug.

## Devices

- Two-phase construction: `new(props)` validates and allocates; `realize(ctx)`
  performs every outward action. Nothing observable happens before realize.
- Every stateful device has a `save`/`load` pair **and a round-trip test** that
  asserts an identical state hash. Write the test with the device, not later.
- Every MMIO device honours `MemAttrs::debug` — a debugger read must not pop a
  FIFO, clear a status bit, or advance a pointer.
- Derived state (caches, TLBs, decoded tables, host pointers) is never
  serialized and is always invalidated by the topology generation counter.

## CPU cores

- Each core ships an interpreter first; the IR frontend comes later and is
  differentially tested against the interpreter forever. **The interpreter is
  the oracle.**
- Instruction tables are generated from a declarative description in the same
  file, not hand-written twice for decode and disassembly.
- Cycle accounting is per-access, driven through the bus, not a post-hoc table
  of instruction lengths — otherwise timing-sensitive software breaks in ways
  no unit test will catch.
- A core lands with its conformance suite (roadmap §9) and a known-failures
  ledger that only ever shrinks.

## Testing

- `#[cfg(test)] mod tests` at the bottom of each file, or `tests.rs` beside a
  module directory.
- Machine-level regression: run deterministically for N virtual units, assert
  the final state hash and periodic framebuffer hashes.
- Conformance corpora are downloaded by a script into an ignored directory and
  gated behind an env var — never committed, never required for `cargo test`.
- `fuzz/` targets for the `.machine` parser, disk-image parsers, and every MMIO
  surface.

## Style

- Comments explain *why*. Match the surrounding density.
- Public items are documented; `missing_docs = "warn"`.
- Hot-path accessors get `#[inline]`; nothing else does.
- Keep the module tree flat enough to navigate: a device is one file until it
  genuinely isn't.
