// Run a `wasm32-wasip1` test binary under Node's WASI, as cargo's target runner.
//
// Why this exists
// ---------------
//
// CI builds all three wasm targets every commit and, until this file, ran none
// of them: `tests/record_replay.rs` said so in as many words — "there is no
// wasm test runner in this repository" — and named the gap it left, that phase
// 9's "replayed bit-identically on a different host" had never been asked of a
// 32-bit address space with a different code generator. `scripts/check.sh
// crosshost` now asks it, and this is what it asks with.
//
// Node rather than wasmtime because `web/` already assumes Node and because a
// developer who has one usually does not have the other. Either would do: this
// is twelve lines around `node:wasi`, and the stage skips itself when Node is
// absent.
//
// This file is for `wasm32-wasip1` *only*. `node:wasi` implements preview 1 and
// nothing else, so a `wasm32-wasip1-threads` module — which imports
// `wasi.thread-spawn` and a shared memory — will not instantiate under it. That
// target has its own runner, `scripts/wasi-threads-run.sh`, and its own runtime
// for the reasons written there.
//
// Usage — cargo calls it, nothing else needs to:
//
//   CARGO_TARGET_WASM32_WASIP1_RUNNER="node scripts/wasi-run.mjs" \
//     cargo test --target wasm32-wasip1 ... -- --test-threads=1
//
// `--test-threads=1` is not optional. `wasm32-wasip1` (the one without
// `-threads`) has no `std::thread`, so libtest's default of one thread per test
// aborts the process before the first test runs.
//
// Preopens `/` so a test can exchange a file with another host — which is what
// `tests/crosshost_snapshot.rs` does with a save state. Absolute host paths
// therefore work as written; a relative one resolves against the guest's
// working directory, which is `/`.

import { WASI } from 'node:wasi';
import { readFile } from 'node:fs/promises';

const argv = process.argv.slice(2);
if (argv.length === 0) {
  console.error('usage: node scripts/wasi-run.mjs <module.wasm> [args...]');
  process.exit(2);
}

const wasi = new WASI({
  version: 'preview1',
  args: argv,
  env: process.env,
  preopens: { '/': '/' },
});

const module_ = await WebAssembly.compile(await readFile(argv[0]));
const instance = await WebAssembly.instantiate(module_, wasi.getImportObject());
// A libtest failure aborts rather than returning, and an abort surfaces as a
// thrown `RuntimeError` that leaves node with a non-zero status. Either way the
// exit code is the verdict, which is what `scripts/check.sh` gates on.
process.exitCode = wasi.start(instance);
