# WebAssembly and the browser target

Consumed by: `jit/wasm`, `host/` wasm shim, from phase 0 in CI. See
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
  Measure this at phase 6 and cut the backend if the numbers say so — the IR
  interpreter is always the fallback.
- Guest RAM lives in the shared linear memory, so generated code addresses it
  with plain loads and stores.
- Virtual time is computed internally, so a browser session replays
  bit-identically under a native debugger.
