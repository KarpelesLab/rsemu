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

### The threaded wasm build declares itself, because stable cannot detect it

Until this was run, `core::sync`'s backend selection was `cfg(all(feature =
"std", not(target_family = "wasm")))`, so `wasm32-wasip1-threads` — a working
`std::thread` and a shared linear memory — got the zero-worker, non-blocking
`single` backend. Running the suite turned a to-do into two observed failures:

- `Pool` reported **zero workers**, so `tests/parallel_threading.rs`'s
  `every_runnable_is_inside_its_run_call_on_a_thread_of_its_own` failed with
  "this host refused a worker thread". The other eleven tests in that file
  passed only because `parallel` mode degenerated to running every runnable on
  the calling thread.
- `single::Mutex::lock` treats a contended acquisition as the deadlock it would
  be under one thread and panics. `core::space::buslock::BusLock` is such a
  `Mutex`, so two guest cores on two real host threads aborted inside
  `BusLock::acquire` — five tests across `a64_lse_atomicity`,
  `riscv_amo_atomicity`, `x86_bus_lock` and `parallel_threading`. **In a
  threaded browser build that is two emulated cores killing the page.**

All five pass now, and the stage's skip list went from ten rows to four.

#### There is no `wasm-atomics` module and there does not need to be

`src/core/sync.rs` named one as an EXTENSION POINT and `ROADMAP.md` §4.7's table
still lists the name. What the name describes is real; a separate module was
not. On a threaded wasm target `std::sync::Mutex` **is** `memory.atomic.wait32`
and `std::thread::spawn` **is** `wasi:thread-spawn`, so `core::sync`'s
`native_std` backend compiled for that target already was the wasm-atomics
backend. Writing a second one would have reimplemented std's futex to arrive at
the same two instructions. What was missing was never the code; it was a `cfg`
that could reach it.

#### The `cfg` cannot exist on stable

The old extension-point comment proposed keying on `target_family = "wasm"` plus
`target_feature = "atomics"`. That cfg does not exist on stable. Measured, not
assumed:

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

#### So the build says so: the `wasm-threads` feature

`core::sync` selects its threaded backend on `all(feature = "std",
any(not(target_family = "wasm"), feature = "wasm-threads"))`. Off wasm the first
disjunct is already true, so the feature is inert there and native is
untouched — the same object code, and the feature sweep gains one build that
changes nothing.

**Why a feature and not the runtime probe.** A probe answers the question
perfectly:

| Target | `std::thread::Builder::new().spawn(..)` |
| --- | --- |
| `wasm32-wasip1` (Node's WASI, and wasmer) | `Err(Os { code: 58, kind: Unsupported, message: "Not supported" })` |
| `wasm32-wasip1-threads` (wasmer) | `Ok(..)`, and the join returns the value |

— but it answers it too late to be the selector. `Mutex`, `RwLock`, `Once`,
`Pool` and `Handle` are *types*, chosen when the crate is compiled, and the two
candidates differ in more than a policy bit: one blocks and the other panics,
one owns `std::thread` workers and the other is `no_std` and has no threads to
own. Selecting between them at run time means an enum around every primitive and
a branch on every acquisition — including `BusLock`, which is on the guest's
memory path. A feature costs nothing and moves the decision to where the
information already is.

**Why a build script is not the automatic version of it, either.** `TARGET` in a
build script would separate `wasm32-wasip1` from `wasm32-wasip1-threads` with no
chance of misconfiguration. It would not separate the threaded browser build
from the non-threaded one: both are `wasm32-unknown-unknown`, differing only in
`-Z build-std -C target-feature=+atomics`. Half the problem solved, the feature
still needed, and two mechanisms answering one question.

#### The probe survives as a tripwire

A wrong feature setting is not symmetric, and the harmless direction is harmless
by construction. Claim threads on a host that has none and `Pool::new` spawns,
gets `Err`, keeps zero workers and runs jobs inline — which is what `single`
does. Measured rather than reasoned: `--features std,wasm-threads` builds for
`wasm32-wasip1` and `wasm32-unknown-unknown`, and on `wasm32-wasip1` under Node
every `core::sync` test passes except the `threaded` module, whose whole job is
to assert that `Pool::new(4).workers() == 4`. So the emulator degrades and
`cargo test` still says the build was configured wrong.

The other direction is the defect that shipped: a host that really does hand
back a thread, running the backend that panics on contention. So
`core::sync`'s `a_threaded_wasm_host_must_not_be_running_single` spawns one
thread, and if the spawn succeeds it asserts the build is not on `single`.
`Builder::spawn` returns an error rather than panicking on every target, so the
probe is safe to run anywhere; it lives in a test rather than in `Pool` because
CI is the enforcement and a shipped build should not spend a thread proving a
`cfg`.

Note that this is a probe *at pool construction time*, which is the one moment
`ROADMAP.md` §4.7 and `CLAUDE.md` already sanction: the embedder builds the pool
up front with a worker count from the machine configuration, precisely because a
Web Worker cannot be created synchronously from arbitrary code. Spawning there
is not a device model quietly making a thread — it is the sanctioned
construction point doing the thing it exists to do, and its refusal is data
(`Pool::workers() == 0`) rather than an error a caller has to handle.

#### One test left the ledger a different way

`core::sync`'s `a_panicking_job_is_reported_at_join_and_the_pool_survives` is
now gated `#[cfg(panic = "unwind")]` rather than skipped. Its module never
compiled for a wasm target before, and `catch_unwind` catches nothing under
`panic = "abort"`: the test is inapplicable there, not failing. That is a
different thing from the three conformance-harness skips, which are `quietly(..)`
wrappers inside code that must keep compiling for every target.
