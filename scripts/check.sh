#!/usr/bin/env bash
# Run what CI runs, locally, and say plainly which parts failed.
#
# Two problems this exists for.
#
# The first is that "I ran the tests" and "CI is green" were different claims
# and nobody could tell which one they had. The gates live in
# `.github/workflows/ci.yml` and were reachable only by pushing. Every stage
# below is the same command that workflow runs, so a green run here means the
# same thing.
#
# The second is legibility. A sweep over a hundred features prints tens of
# thousands of lines, and a failure two thirds of the way up scrolls off; a
# `std`-only construct reached master twice because a summary line was not
# read. So: nothing here parses test output — a step's verdict is its exit
# status and nothing else — and the summary is printed last, marked, and
# repeated as an exit code. `test result: ok. N passed; M failed` has the
# failure count in field *six*, and a gate that counted field seven let a red
# build through once already. Do not count fields. Use `$?`.
#
# One failure shape that is never a code failure.
#
# If a stage fails with `could not execute process ... (No such file or
# directory)` naming a test binary, or a test reports `never executed`, two
# builders shared one target directory and one deleted the other's binaries
# mid-run. A real regression never has that shape. It has happened three times:
# two `check.sh` runs started in one worktree, an unscoped `pkill` that killed a
# sibling agent's build, and an orphaned run that survived a `pkill -f` because
# it had been invoked as `bash ./scripts/check.sh` and the pattern did not
# match. Do not chase it as a defect -- give each concurrent run its own
# `CARGO_TARGET_DIR`, and kill by working directory
# (`readlink /proc/$pid/cwd`) rather than by command-line pattern, which misses
# a relative invocation and can match somebody else's.
#
# Usage:
#   scripts/check.sh              the per-commit set: fast, test, wasm, combos
#   scripts/check.sh --all        everything, including the full feature sweep
#   scripts/check.sh fast test    named stages only
#   scripts/check.sh --list       what the stages are
#
# Stages:
#   fast     fmt, both clippy configurations, rustdoc, the dependency policy
#   test     --all-features, the default build, --no-default-features
#   wasm     all three wasm targets, plus the browser cdylib
#   combos   the derived no_std feature-*combination* builds (see below)
#   crosshost  the replay gate on a second architecture (32-bit)
#   sweep    every feature on its own — long; CI runs it on its own job
#   fuzz     `cargo fuzz build` (needs a nightly and cargo-fuzz)
#
# `combos` is the one that is not a copy of an existing CI step. Cargo features
# are additive, so `--all-features` compiles every conjunction of them — but it
# also turns on `std`, and a one-at-a-time sweep never turns two features on at
# once, so code gated on a *pair* of `no_std` features had never been compiled
# without `std`. `scripts/feature-matrix.py` derives those pairs from the tree
# rather than from a list somebody maintains; `feature-matrix.py plan` explains
# what it found.

set -uo pipefail

cd "$(dirname "$0")/.."

# One-shot builds never reuse an incremental cache, and on a machine running
# several worktrees the caches were the single largest thing on the disk —
# larger than every compiled artifact put together. Off here; CI sets the same.
export CARGO_INCREMENTAL="${CARGO_INCREMENTAL:-0}"
export CARGO_TERM_COLOR="${CARGO_TERM_COLOR:-always}"
export RUSTFLAGS="${RUSTFLAGS:--D warnings}"

STAGES=(fast test wasm combos crosshost sweep fuzz)
DEFAULT_STAGES=(fast test wasm combos)

bold() { printf '\033[1m%s\033[0m\n' "$*"; }

# A failed build on a full disk is not a failed build. This project has already
# once read a hardware fault as memory pressure; a linker that ran out of space
# reads as a compile error just as convincingly, so the number is printed
# before anything runs and again beside any failure.
disk_free() { df -h . | awk 'NR==2 {print $4 " free of " $2 " (" $5 " used)"}'; }
disk_free_gb() { df -Pk . | awk 'NR==2 {print int($4 / 1048576)}'; }

# Every feature set has its own metadata hash, so no sweep iteration can ever
# reuse the previous one's artifacts — they only accumulate. A hundred of them
# is about 85 GB in one target directory, which is most of how a checkout here
# reached half a terabyte. Dropping just this crate's artifacts between
# iterations therefore costs nothing that would have been reused, and turns the
# sweep's footprint from O(features) into O(1). Set RSEMU_SWEEP_KEEP=1 to keep
# them (for bisecting a sweep failure, where the second run should be fast).
#
# Only the sweep does this locally. The five combination builds are few enough
# that the ~4 GB is not worth throwing away the working tree's own artifacts
# with it; CI cleans after those too, because a fresh runner has nothing to
# preserve and fourteen gigabytes to spend.
sweep_clean() {
  [ -n "${RSEMU_SWEEP_KEEP:-}" ] || cargo clean -p rsemu >/dev/null 2>&1 || true
}

# A build that ran out of disk fails like a build that is broken. This project
# has already once read a hardware fault as memory pressure; stopping with a
# clear message beats producing a hundred lines of misleading compiler output.
DISK_FLOOR_GB="${RSEMU_DISK_FLOOR_GB:-20}"
disk_guard() {
  local free
  free=$(disk_free_gb)
  if [ "$free" -lt "$DISK_FLOOR_GB" ]; then
    bold "STOPPING: only ${free} GB free, floor is ${DISK_FLOOR_GB} GB."
    echo "A failure from here would be about the disk, not about the code."
    echo "Set RSEMU_DISK_FLOOR_GB to override."
    exit 2
  fi
}

RESULTS=()
FAILED=0

# Each verdict is printed twice: once here, right after the step, so a long
# run says what happened as it happens, and once in the summary at the end, so
# it is all in one place. The summary alone was not enough — that is precisely
# the line that got scrolled past.
record() {
  RESULTS+=("$1")
  bold "    <-- $1"
}

run() {
  local name="$1"; shift
  bold "==> $name"
  printf '    %s\n' "$*"
  if "$@"; then
    record "ok    $name"
  else
    local rc=$?
    record "FAIL  $name (exit $rc, disk: $(disk_free))"
    FAILED=$((FAILED + 1))
  fi
}

# ---------------------------------------------------------------------------

stage_fast() {
  run "fmt" cargo fmt --all --check
  run "clippy --all-features" \
    cargo clippy --all-targets --all-features -- -D warnings
  # The second configuration, and the one that matters for ROADMAP §0: clippy
  # with std absent. A lint that only fires in the emulation core is invisible
  # to the run above, which has every feature on.
  run "clippy --no-default-features" \
    cargo clippy --all-targets --no-default-features -- -D warnings
  run "rustdoc" env RUSTDOCFLAGS="-D warnings" \
    cargo doc --no-deps --all-features
  # The same script CI's `deps` job runs, so the two answers cannot differ.
  run "dependency policy" ./scripts/deps-policy.sh
}

stage_test() {
  run "test --all-features" cargo test --all-features
  run "test (default features)" cargo test
  # `std` is a default feature, so dropping the defaults is the whole no_std
  # gate. Both the build and the tests: `cargo build` never compiles a
  # `#[cfg(test)]` block, and two of the three `std` leaks this project has
  # shipped were inside one.
  run "build --no-default-features" cargo build --no-default-features
  run "test --no-default-features" cargo test --no-default-features
}

stage_wasm() {
  local t
  for t in wasm32-unknown-unknown wasm32-wasip1 wasm32-wasip1-threads; do
    if ! rustc --print target-libdir --target "$t" >/dev/null 2>&1; then
      record "skip  wasm $t (target not installed: rustup target add $t)"
      continue
    fi
    run "wasm $t" cargo build --target "$t" --no-default-features --features wasm
  done
  if rustc --print target-libdir --target wasm32-unknown-unknown >/dev/null 2>&1; then
    # The non-threaded browser build is a supported target, not a fallback
    # (ROADMAP.md §11), and `demo` is the only feature set the page loads.
    run "wasm demo cdylib" cargo rustc --crate-type cdylib \
      --target wasm32-unknown-unknown --no-default-features --features demo --release
  fi
}

# The derived combination builds. Failures are collected rather than fatal, so
# one broken pair does not hide the other four.
stage_combos() {
  local sets rc=0 set_
  sets=$(python3 scripts/feature-matrix.py merged) || {
    record "FAIL  combos (feature-matrix.py failed)"; FAILED=$((FAILED+1)); return; }
  bold "==> feature combinations"
  python3 scripts/feature-matrix.py plan | sed 's/^/    /'
  for set_ in $sets; do
    disk_guard
    printf '    -- %s\n' "$set_"
    cargo test --no-default-features --features "$set_" || {
      record "FAIL  combo $set_ (disk: $(disk_free))"
      FAILED=$((FAILED + 1)); rc=1; }
  done
  [ "$rc" -eq 0 ] && record "ok    feature combinations"
  return 0
}

# Every feature on its own. Long, and every failure is collected: the point of
# a sweep is the list of what is broken, not the first thing that is.
stage_sweep() {
  local f rc=0
  bold "==> feature sweep (one feature at a time)"
  for f in $(python3 scripts/feature-matrix.py features); do
    disk_guard
    printf '    -- %s\n' "$f"
    cargo test --no-default-features --features "$f" || {
      record "FAIL  feature $f (disk: $(disk_free))"
      FAILED=$((FAILED + 1)); rc=1; }
    sweep_clean
  done
  [ "$rc" -eq 0 ] && record "ok    feature sweep"
  return 0
}

# Phase 9's gate is "a recorded session replayed bit-identically on a *different
# host*", and every other stage here runs on this one. `tests/record_replay.rs`
# pins the recording and the resulting state hash as constants, so replaying
# them under a second target is the whole test: same source, same constants,
# different `usize`, different ABI, different code generator.
#
# i686 rather than aarch64 because it is the one installed target this machine
# can also *run* -- a cross-compiled aarch64 binary needs an emulator that is
# not assumed here. Skipped, not failed, when the target or its 32-bit runtime
# is absent: a developer without them has lost nothing the CI matrix
# (ubuntu/macos/windows) does not already check.
stage_crosshost() {
  local t=i686-unknown-linux-gnu
  if ! rustc --print target-libdir --target "$t" >/dev/null 2>&1; then
    record "skip  crosshost (target not installed: rustup target add $t)"
    return 0
  fi
  local feats="std,machine-apple1"
  if ! cargo build --target "$t" --no-default-features --features "$feats" \
       >/dev/null 2>&1; then
    record "skip  crosshost ($t does not link here: 32-bit runtime missing?)"
    return 0
  fi
  run "crosshost replay ($t)" \
    cargo test --target "$t" --no-default-features --features "$feats" \
    --test record_replay
}

stage_fuzz() {
  if ! cargo +nightly fuzz --version >/dev/null 2>&1; then
    record "skip  fuzz (needs a nightly toolchain and cargo-fuzz)"
    return 0
  fi
  # Build only. A campaign is `fuzz/README.md`'s command, not a gate.
  run "fuzz build" env RUSTUP_TOOLCHAIN=nightly cargo fuzz build
}

# ---------------------------------------------------------------------------

want=()
case "${1:-}" in
  --list) printf '%s\n' "${STAGES[@]}"; exit 0 ;;
  --all)  want=("${STAGES[@]}") ;;
  "")     want=("${DEFAULT_STAGES[@]}") ;;
  -*)     echo "unknown option $1" >&2; exit 2 ;;
  *)      want=("$@") ;;
esac

bold "rsemu check: ${want[*]}"
echo "disk: $(disk_free)"
echo

for s in "${want[@]}"; do
  case " ${STAGES[*]} " in
    *" $s "*) "stage_$s" ;;
    *) echo "unknown stage $s (try --list)" >&2; exit 2 ;;
  esac
done

echo
bold "================ check summary ================"
printf '%s\n' "${RESULTS[@]}"
echo "disk: $(disk_free)"
if [ "$FAILED" -ne 0 ]; then
  bold "CHECK FAILED: $FAILED step(s) above are marked FAIL"
  exit 1
fi
bold "CHECK OK"
