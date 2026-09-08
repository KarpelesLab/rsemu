# WebAssembly and the browser target

Consumed by: `jit/wasm`, `host/` wasm shim, from in CI. See
`ROADMAP.md` §11 for the target matrix and the design.

## Specifications

| Source | Covers |
| --- | --- |
| [WebAssembly Core Specification](https://webassembly.github.io/spec/core/) | The instruction set, module format, validation, and execution semantics — needed to *emit* wasm from the IR |
| [Threads proposal](https://github.com/WebAssembly/threads) | Shared memory, atomic instructions, `memory.atomic.wait`/`notify`, and the wasm memory model |
| [Threads-enabled core spec](https://webassembly.github.io/threads/core/) | The core specification with the threads proposal merged |

## Browser platform

The relevant web APIs — `WebAssembly.Module`/`Instance`/`Memory`,
`SharedArrayBuffer`, `Atomics`, Web Workers, `requestAnimationFrame`,
`performance.now()`, the File System Access API — are specified by WHATWG/W3C
and documented on MDN. Two constraints drive the whole design:

1. **`SharedArrayBuffer` requires cross-origin isolation** (COOP/COEP headers).
   Often unavailable, which is why the non-threaded configuration is a supported
   target and not a fallback.
2. **`Atomics.wait` is forbidden on the main thread.** Emulation therefore runs
   in a worker, always, with the main thread doing display and input only.

## In-house prior art

| Project | Relevance |
| --- | --- |
| [`fstool`](https://github.com/KarpelesLab/fstool) | A complete disk/filesystem toolchain shipping as a client-side browser app — the pattern rsemu follows for its demo page |
| [`kataan`](https://github.com/KarpelesLab/kataan) | A WebAssembly engine *and* a JIT, both ours and MIT — the closest existing reference for emitting and running wasm |
| [`purecrypto`](https://github.com/KarpelesLab/purecrypto) | The embedder-supplied host-import convention (`wasm32-unknown-unknown` with no bundled JS runtime) |

## Implementation notes

- **No `mmap` means no native code path.** The JIT emits wasm modules instead;
  synchronous `new WebAssembly.Module()` is permitted inside a worker.
- Per-module instantiation cost means only superblocks are worth compiling.
  Measure this at and cut the backend if the numbers say so — the IR
  interpreter is always the fallback.
- Guest RAM lives in the shared linear memory, so generated code addresses it
  with plain loads and stores.
- Virtual time is computed internally, so a browser session replays
  bit-identically under a native debugger.

## Running the threaded target, and the two things it found

`ROADMAP.md` §11 calls `wasm32-wasip1-threads` the primary threaded wasm target
and says CI gates on it. Until the `wasm-threads` job it gated on `cargo build`:
every wasm target has been *compiled* on every commit since the first one, the
`crosshost` job added *execution* for `wasm32-wasip1`, and the one target where
the concurrency design can be observed rather than type-checked ran nothing.

**The runtime is [wasmer](https://github.com/wasmerio/wasmer)** (MIT), driven by
`scripts/wasi-threads-run.sh` as cargo's target runner. Three of the four
obvious alternatives do not work, which is why the choice is written down:

| Runtime | wasi-threads |
| --- | --- |
| wasmtime ≥ 47 | **removed.** `-S threads` is a hard error; the removal landed with the `wasmtime-remove-wasi-threads` RFC |
| wasmtime 46.x | accepts the flag, then refuses a module with a shared memory (`shared memory support is disabled for this engine`) |
| `node:wasi` — what `scripts/wasi-run.mjs` uses for the non-threaded target | never had it |
| WAMR `iwasm` (published Linux binary) | rejects the module before threads are reached (`SIMD compatibility check failed`) |
| wasmer 7.x | runs it as shipped: verified overlapping OS threads and a 4.36× speed-up on four CPU-bound ones |

`scripts/check.sh wasm-threads` runs about 2400 tests there. Two constraints
shape how, and neither is a bug in the emulator:

- **`--test-threads=1`.** Every wasm target is `panic = "abort"`, so libtest
  cannot catch a failing test. Above one thread the panic message is printed
  from a spawned libtest thread whose stderr the runtime drops — the run still
  aborts, with no message and no test name. It buys no wall time either (111.6 s
  against 113.8 s for the same suite), because the tests are allocation-bound
  behind one `dlmalloc` lock. The threading that matters here is the threads the
  *tests* spawn.
- **`-C link-arg=--max-memory=4294967296`.** See below.

### Shared memory has a maximum, and the default is 1 GiB

A shared linear memory must declare a maximum — the non-threaded
`wasm32-wasip1` memory declares none and can grow to the whole 32-bit space.
rustc links this target with `--max-memory=1073741824`, 16384 pages. That is not
enough to snapshot `machine-a64-mini`: it has 128 MiB of guest RAM,
`Machine::save` materialises the state into a `Vec<u8>`, and the vector's
doubling asks for 268 438 836 bytes on top of the machine itself. It aborts in
`handle_alloc_error`. The CI leg raises the cap to the wasm32 ceiling (65536
pages, 4 GiB) and the test passes.

The default is what a browser build gets, so state it plainly: **as linked
today, a threaded rsemu in a `SharedArrayBuffer` cannot save a machine with
128 MiB of guest RAM.** A browser build that wants to needs the same link
argument, and a `SharedArrayBuffer` cannot be grown past the module's declared
maximum after instantiation.

### `core::sync` selects `single` on wasm, threads or no threads

The backend selection in `src/core/sync.rs` is
`cfg(all(feature = "std", not(target_family = "wasm")))`. `wasm32-wasip1-threads`
has a working `std::thread` and a shared memory, and still gets the zero-worker,
non-blocking `single` backend — the `wasm-atomics` backend that file names as an
EXTENSION POINT and that ROADMAP.md §11's target matrix lists does not exist
yet. Running the suite is what turned that from a to-do into two observed
failures:

- `Pool` reports **zero workers**, so `tests/parallel_threading.rs`'s first
  assertion fails with "this host refused a worker thread". The rest of that
  file passes — `parallel` mode *runs* on this target, it just runs every
  runnable on the calling thread.
- `single::Mutex::lock` treats a contended acquisition as the deadlock it would
  be under one thread and panics. `core::space::buslock::BusLock` is such a
  `Mutex`, and two guest cores on two real host threads contend for it, so
  `tests/a64_lse_atomicity.rs`, `tests/riscv_amo_atomicity.rs` and
  `tests/x86_bus_lock.rs` abort inside `BusLock::acquire`. In a threaded browser
  build that is two emulated cores taking a bus lock and killing the page.

### The extension point cannot be a `cfg` on stable

`src/core/sync.rs`'s EXTENSION POINT comment proposes that the `wasm-atomics`
backend claim `target_family = "wasm"` with `target_feature = "atomics"`. That
cfg does not exist on stable. Measured, not assumed:

```console
$ rustc --print cfg --target wasm32-wasip1        | sort > a
$ rustc --print cfg --target wasm32-wasip1-threads | sort > b
$ diff a b        # no output: the two targets are cfg-identical
```

`atomics` is an *unstable* target feature, so stable rustc never emits it as a
cfg — not for `wasm32-wasip1-threads`, and not even for `wasm32-unknown-unknown`
built with `-C target-feature=+atomics` (it warns and omits the cfg; only a
nightly sets `target_feature="atomics"`, which is the toolchain the
`wasm-threads-browser` job already needs for `-Z build-std`). `target_has_atomic
= "ptr"` cannot stand in either: every wasm32 target sets it, threads or no
threads, because it describes how atomics *lower*, not whether the host has more
than one thread.

What does distinguish them is asking. `std::thread::Builder::spawn` returns a
plain error on the non-threaded target and succeeds on the threaded one — no
panic either way, so a pool can probe once at construction:

| Target | `std::thread::Builder::new().spawn(..)` |
| --- | --- |
| `wasm32-wasip1` (Node's WASI) | `Err(Os { code: 58, kind: Unsupported, message: "Not supported" })` |
| `wasm32-wasip1-threads` (wasmer) | `Ok(..)`, and the join returns the value |

A Cargo feature would work too and is more explicit, at the cost of a build that
can be configured wrong. Either way the choice belongs in `core::sync`, and it
is a `src/` change with a review attached — the point of writing it down here is
that "just key on `target_feature`" is not available.
