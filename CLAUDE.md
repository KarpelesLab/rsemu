# Design rules for rsemu

Read this before writing any code. [`ROADMAP.md`](ROADMAP.md) is the *what* and
the phase order; this file is the *how*. Where the two disagree, the roadmap's
non-negotiables (§0) win.

## Provenance — the rule that outranks every other rule

rsemu is MIT. **MIT code can never absorb GPL'd code**, and no amount of
paraphrasing changes that. Taint is not fixable by deleting a file afterwards:
the history and everything derived from it are contaminated.

- **Never open the QEMU source tree.** Not the code, not the headers, not the
  comments, not the commit messages, not the mailing list, not anything derived
  from it (Unicorn Engine, forks, vendored copies). "Just to see how they did
  it" is precisely the forbidden act.
- Same for every other copyleft source — GPL, LGPL, AGPL, SSPL, CDDL. Bochs,
  DOSBox, MAME, VICE, Dolphin, PCSX2, Nestopia, higan are all off limits.
  **Check a project's licence before opening it.** LGPL permits linking, not
  copying source into an MIT crate.
- **Work from hardware documentation**: datasheets, ISA manuals, the NESdev
  wiki, Pan Docs, service manuals, papers. These describe the machine we are
  emulating rather than someone's emulation of it, and they are better sources
  anyway.
- **Permissive code is fine with attribution** (MIT/BSD/Apache/ISC). `../gones`
  is ours and MIT; its PPU/APU lineage is Michael Fogleman's MIT emulator —
  carry his copyright notice into any ported file.
- **Black-box use of a GPL program is fine.** Run it, benchmark it, diff its
  trace. Reading its source is not.
- **Hardware behaviour is fact; an implementation of it is expression.** A cycle
  count from a datasheet is free. The identical number lifted from a GPL
  emulator's table is not.
- **Cite your source** for any non-obvious algorithm — which manual, which
  section — in the commit message or a comment.
- This applies to tool-generated code. Origin is a property of the code, not of
  how it reached the editor.
- If you have previously read forbidden source for a subsystem, say so and let
  someone else write it. That is hygiene, not blame.

Roadmap §1 has the long form. When in doubt, ask **before** reading.

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
  `compcol`, `purecrypto`, **`fstool`**, `puremp`, `noroi`. Nothing else — no
  `serde`, no `libc`, no async runtime, no GUI toolkit.
- The empty-`cargo tree` rule holds for *default* features. Several siblings
  pull external crates under optional features, so CI checks the feature-enabled
  tree too.
- OS interaction is by raw syscall (the `purestd` pattern), not via `libc`.
- The two exceptions that break purity — macOS Hypervisor.framework and Windows
  WHPX — are opt-in features labelled as such in the README. Never silent.

## `no_std`

- `core/`, `ir/`, `cpu/`, `machine/` and most of `dev/` are `no_std + alloc`.
  `std` appears under `host/`, `jit/`, `accel/` — and, as documented exceptions,
  `dev/blk/*` and `dev/net/*`, because `fstool` and `pktkit` are `std` crates.
  Those two are feature-gated so a `no_std` build excludes them.
- CI builds `--no-default-features --features alloc`. A `std` leak into the
  emulation core is a build break, not a style nit.

## `unsafe`

- Crate-wide `unsafe_code = "deny"` (not `forbid`), so **six** subsystems can
  opt back in with a scoped `#[allow(unsafe_code)]`: the RAM host-pointer fast
  path, the JIT code buffer, the raw-syscall accel backends, `ffi`, the
  `core::sync` `single` backend (`RefCell` is not `Sync`), and per-CPU execution
  state (a lock per instruction cannot hit the throughput gate). Six is the
  ceiling; a seventh is a design review, not a commit.
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

- No async. Devices are `Send + Sync` with synchronous methods — from the first
  commit, not once threading "is needed".
- **Everything goes through `core::sync`.** Nothing under `core/`, `cpu/`,
  `dev/`, `machine/` or `ir/` may name `std::sync`, `std::thread`, or the host
  clock. One `std::sync::Mutex` in a device model breaks `no_std`, both wasm
  targets, and the `fullrust` build simultaneously. `host/`, `jit/` and `accel/`
  may use `std` directly.
- **Submit jobs, never spawn threads.** Background work goes to the seam's task
  pool. wasm cannot create a worker synchronously from arbitrary code, and
  thread count belongs to the machine configuration anyway.
- Shared mutable device state: `Mutex`/`RwLock` for cold paths, atomics for
  hot ones. Prefer designing the hot path to need neither.
- **Follow the re-entrancy contract, not a blanket no-locks rule.** Mutate your
  own state in a short critical section, release it, *then* make any outward
  call — DMA, wire change, remap, a call into a sibling — or push the action
  onto the handler's deferred queue. "Never hold a lock across a call into
  another device" was unimplementable: NES OAM DMA and a BAR-moving config write
  both require exactly that. Respect the ranked lock order in `core::sync`.
- The scheduler owns time. A device never sleeps, never reads the wall clock,
  and never spawns a thread to "tick" itself — it registers an event.
- Stopping the world (TLB shootdown, remap, snapshot, reset) uses the
  safe-point protocol: a generation counter plus a per-CPU exit flag checked at
  block boundaries. Never a signal — wasm has none.

## Targets

- Build the matrix, not just your host: native, `no_std`,
  `wasm32-wasip1-threads`, `wasm32-wasip1`, and `wasm32-unknown-unknown` with
  and without threads. CI does this every commit; if you break wasm you find out
  immediately, which is the point.
- **Threaded `wasm32-unknown-unknown` needs a nightly** (`-Z build-std`) —
  stable's precompiled std lacks `+atomics`, so `--shared-memory` fails at link.
  That job is the project's only nightly and no shipping artifact uses it;
  `wasm32-wasip1-threads` is the stable threaded target CI gates on. Roadmap
  §11.1.
- The non-threaded browser configuration is a supported target, not a fallback.
  It shares a code path with the deterministic test runner, so it stays honest.
- Guest RAM is addressed by byte offset, never by handing out `&mut [u8]`, so it
  can live in a `SharedArrayBuffer`. Do not "simplify" that API.

## Determinism

- No `HashMap` iteration order in anything that affects guest-visible state.
  Use `BTreeMap` or an insertion-ordered map.
- No floats in the time path. Ratios *within* an oscillator tree are exact
  integer arithmetic; the *cross-tree* timeline is fixed-point plus a residual
  accumulator. Never `f64` seconds — and never route an intra-tree relationship
  through absolute time, which throws away the exactness the design exists to
  preserve.
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
  file, not hand-written twice for decode and disassembly. That generator also
  emits the **disassembler**, which gdb and the monitor both need — it is not a
  separate project.
- Cycle accounting is per-access, driven through the bus, not a post-hoc table
  of instruction lengths — otherwise timing-sensitive software breaks in ways
  no unit test will catch.
- A core lands with its conformance suite (roadmap §12) and a known-failures
  ledger that only ever shrinks.

## Testing

- `#[cfg(test)] mod tests` at the bottom of each file, or `tests.rs` beside a
  module directory.
- Machine-level regression: run deterministically for N virtual units, assert
  the final state hash and periodic framebuffer hashes.
- Conformance corpora are downloaded by a script into an ignored directory and
  gated behind an env var — never committed, never required for `cargo test`.
  This is a licensing rule as much as a size one: several suites are copyleft
  (`kvm-unit-tests` is GPLv2). Running a GPL binary as an emulated guest is
  ordinary use; shipping it in our repo is redistribution under its terms.
  Confirm the licence of any fixture before vendoring it.
- `fuzz/` targets for the `.machine` parser, disk-image parsers, and every MMIO
  surface.

## Arithmetic

- Guest arithmetic wraps by definition. Use `wrapping_*` / `checked_*` /
  `overflow_checks` deliberately and say which you meant; never rely on the
  profile, or debug builds will panic exactly where release silently wraps.
- Guest addresses are computed in the guest's width, then widened. Widening
  first and masking later hides wrap bugs.

## Commit messages

Release-plz builds the changelog and picks the version bump from these, so the
prefix is machine-read, not decoration. **Conventional Commits**, matching the
sibling crates:

```
feat(cpu-arm): Thumb interworking and the v5 DSP extensions
fix(space): reject a rebase that slides off its target
docs: record why PAL needs its own machine file
test(z80): assert MEMPTR through BIT n,(HL)
chore: bump the fuzz corpus
```

- **Types**: `feat` (new capability), `fix` (defect), `docs`, `test`, `refactor`,
  `perf`, `build`, `ci`, `chore`. Anything else is invisible to the changelog.
- **Scope** is the subsystem, in parentheses: a core module (`space`, `clock`,
  `sched`, `wire`, `device`, `props`, `state`, `sync`, `registry`), `machine`, a
  CPU (`cpu-6502`, `cpu-arm`, …), a device (`dev-ppu`, `dev-apu`, `dev-cart`),
  or `wasm`. Optional, but a changelog line without one rarely says enough.
- **Breaking changes take `!`** — `feat(space)!: topology moves behind a guard`.
  Pre-1.0 that is a minor bump, not a major one, which is exactly why it must be
  marked: nothing else distinguishes it.
- The subject is a sentence fragment in the imperative, lower case after the
  colon, no trailing period. The body explains *why*, as always.
- `chore: release vX.Y.Z` belongs to release-plz. Don't write it by hand.

The version bump follows: `feat` → minor, `fix` → patch, `!` → major (minor
while pre-1.0). A defect fixed under a `chore:` prefix ships silently.

## Style

- Comments explain *why*. Match the surrounding density.
- Public items are documented; `missing_docs = "warn"`.
- Hot-path accessors get `#[inline]`; nothing else does.
- Keep the module tree flat enough to navigate: a device is one file until it
  genuinely isn't.
