#!/usr/bin/env sh
# Run a `wasm32-wasip1-threads` test binary under a WASI runtime that has real
# threads, as cargo's target runner.
#
# Why this exists and why it is not `scripts/wasi-run.mjs`
# -------------------------------------------------------
#
# `wasi-run.mjs` runs the *other* wasip1 target under Node's `node:wasi`, and
# Node's WASI has no thread support at all: it implements preview-1 and nothing
# else, so a module importing `wasi.thread-spawn` fails to instantiate. That is
# why CI built `wasm32-wasip1-threads` on every commit from the first one and
# ran it never — ROADMAP.md §11 calls this "the primary threaded wasm target"
# and "what CI gates on", and until this script the gate was a compile.
#
# The runtime is **wasmer**, and the choice was made by trying them, not by
# reading a table:
#
#   wasmtime ≥ 47   removed wasi-threads outright. `-S threads` is a hard error
#                   ("the `-Sthreads` flag is no longer supported"); the removal
#                   landed in 47.0.0 with the `wasmtime-remove-wasi-threads`
#                   RFC. 46.x still accepts the flag but then refuses to
#                   instantiate a module with a shared memory ("shared memory
#                   support is disabled for this engine"), and it is a version
#                   whose successor deleted the feature — not something to hang
#                   a gate on.
#   node:wasi       no threads, as above.
#   WAMR `iwasm`    the published Linux binary rejects the module before it gets
#                   as far as threads ("SIMD compatibility check failed") and
#                   the option its message names is not in that build's `--help`.
#   wasmer 7.x      runs it as shipped. Verified: four libtest threads, a
#                   two-thread rendezvous that only completes if both are live
#                   at once, and a 4.36x speed-up on four CPU-bound threads —
#                   these are host OS threads, not a scheduler pretending.
#
# wasmer is MIT, so running it is ordinary tool use and nothing in CLAUDE.md's
# provenance rule is in play. We do not read its source; we run it.
#
# Usage — cargo calls it, nothing else needs to:
#
#   CARGO_TARGET_WASM32_WASIP1_THREADS_RUNNER=scripts/wasi-threads-run.sh \
#     cargo test --target wasm32-wasip1-threads ... -- --test-threads=1
#
# `--test-threads=1` and why it is *not* the same concession `wasi-run.mjs`
# makes: that file passes it because `wasm32-wasip1` has no `std::thread` and
# libtest's default aborts. Here `std::thread` works. The reason is different
# and worth writing down, because it is the first thing anyone will want to
# change: the target is `panic = "abort"`, so libtest cannot catch a failing
# test, and at `--test-threads > 1` the panic message is printed from a spawned
# libtest thread whose stderr the runtime drops — the run still aborts, but with
# no message and no test name, which is a strictly worse failure report. It buys
# nothing either: the same suite takes 111.6 s at one libtest thread and 113.8 s
# at four, because the tests are allocation-bound behind one `dlmalloc` lock.
# The threading this leg actually exercises is the threads the *tests* spawn —
# `tests/a64_lse_atomicity.rs`, `tests/riscv_amo_atomicity.rs` and
# `tests/parallel_threading.rs` each drive two or more guest cores on real host
# threads, and those do run in parallel.
#
# `--volume /:/` preopens the host root, the same reach `wasi-run.mjs` gives
# with its `preopens`, so `tests/crosshost_snapshot.rs` can exchange a save
# state through an absolute path. `--forward-host-env` carries the `RSEMU_*`
# variables that select those directories.
#
# Point RSEMU_WASI_THREADS_RUNTIME at a wasmer that is not on `$PATH`.

set -eu

runtime="${RSEMU_WASI_THREADS_RUNTIME:-}"
if [ -z "$runtime" ]; then
  runtime=$(command -v wasmer || true)
fi
if [ -z "$runtime" ]; then
  echo "wasi-threads-run.sh: no wasmer on \$PATH and RSEMU_WASI_THREADS_RUNTIME is unset" >&2
  echo "  https://github.com/wasmerio/wasmer/releases — or set the variable to one you have" >&2
  exit 127
fi

if [ "$#" -eq 0 ]; then
  echo "usage: wasi-threads-run.sh <module.wasm> [args...]" >&2
  exit 2
fi

# `--` so the module path and everything after it stay positional: a libtest
# flag like `--test-threads=1` is otherwise parsed by wasmer's own clap.
exec "$runtime" run --volume /:/ --forward-host-env -- "$@"
