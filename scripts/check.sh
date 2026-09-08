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
#   long     the engine-divergence long run (needs a fetched arm64 kernel for
#            its real leg; nightly in CI, not in --all)
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

STAGES=(fast test wasm combos crosshost sweep fuzz long)
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
# host*", and every other stage here runs on this one. Two artefacts have to
# cross that boundary and they cross it differently.
#
# A **recording** crosses as a constant: `tests/record_replay.rs` pins the bytes
# and the resulting state hash in the source, so replaying them under a second
# target is the whole test -- same source, same constants, different `usize`,
# different ABI, different code generator.
#
# A **save state** cannot: it is the machine's whole RAM, and a constant that
# large would be unreadable and rewritten by every unrelated change to a board.
# So `tests/crosshost_snapshot.rs` sends it through a file. The foreign target
# writes each board's snapshot and the snapshot of the same machine after it has
# run on; this host loads the first into a board that has only ever been reset,
# runs it the same span, and must produce the second byte for byte. Running both
# machines forward is the part that can see a field `save` never wrote.
#
# Two second hosts, for two different reasons:
#
#   i686        a 32-bit `usize` on the same ISA family. The one installed
#               target this machine can also *run* -- a cross-compiled aarch64
#               binary needs an emulator that is not assumed here.
#   wasm32      a 32-bit address space, a different code generator, and no
#               native ABI at all, under Node's WASI (`scripts/wasi-run.mjs`).
#               CI built this target every commit and ran it never until the
#               `crosshost` job, which runs this stage as written; the
#               `machine::` and `core::state::` unit tests run here too, which
#               is how `every_shipped_machine_resumes_from_its_own_snapshot`
#               reaches wasm.
#
# Each leg is skipped rather than failed when its target or runtime is absent: a
# developer without them has lost nothing the CI matrix (ubuntu/macos/windows)
# does not already check. `RSEMU_CROSSHOST_REQUIRED` turns those skips into
# failures, and CI sets it — see `crosshost_absent` below.
#
# The feature set is every board `tests/crosshost_snapshot.rs` knows how to
# build without a corpus -- eight guest architectures, no drive on any of them,
# which is deliberate: a drive whose medium snapshots by *reference* writes a
# canonical host path into its chunk and cannot cross a host boundary at all.
CROSSHOST_FEATURES="std,machine-apple1,machine-nes,machine-beneater,machine-z80-mini,machine-m68k-mini,machine-mips-mini,machine-a64-mini,machine-arm926,machine-stm32f407,machine-spi-flash,machine-spi-panel"

# On a developer's machine a missing target or runtime is a skip, and that is
# right: the rest of the CI matrix still checks everything they can check here.
# On a runner it is a failure. The workflow installs every prerequisite itself
# (`.github/workflows/ci.yml`, job `crosshost`), so a leg that skips there means
# the provisioning broke — and a job that is green because it ran nothing is
# worse than no job at all, because it reports a gate as held that nobody is
# holding. CI sets RSEMU_CROSSHOST_REQUIRED=1; nothing else does, so the local
# gate is exactly as strong, and as skippable, as it was.
CROSSHOST_REQUIRED="${RSEMU_CROSSHOST_REQUIRED:-}"
crosshost_absent() {
  if [ -n "$CROSSHOST_REQUIRED" ]; then
    record "FAIL  $1 -- RSEMU_CROSSHOST_REQUIRED is set, so this had to run"
    FAILED=$((FAILED + 1))
  else
    record "skip  $1"
  fi
}

# Load the save states in `$1` into this host and run them on.
crosshost_read() {
  RSEMU_SNAPSHOT_READ_DIR="$1" \
    cargo test --no-default-features --features "$CROSSHOST_FEATURES" \
    --test crosshost_snapshot
}

stage_crosshost() {
  local t=i686-unknown-linux-gnu
  local out probe
  out="$(pwd)/target/crosshost"
  # The link probe's output is discarded locally, where "i686 does not link
  # here" is the whole answer a developer needs; it is kept when the leg is
  # required, where the linker's own error is the only thing that says which
  # package the runner is missing.
  probe=/dev/null
  [ -n "$CROSSHOST_REQUIRED" ] && probe=/dev/stderr

  if ! rustc --print target-libdir --target "$t" >/dev/null 2>&1; then
    crosshost_absent "crosshost i686 (target not installed: rustup target add $t)"
  elif ! cargo build --target "$t" --no-default-features \
         --features "$CROSSHOST_FEATURES" >"$probe" 2>&1; then
    crosshost_absent "crosshost i686 ($t does not link here: 32-bit runtime missing?)"
  else
    run "crosshost replay ($t)" \
      cargo test --target "$t" --no-default-features \
      --features "$CROSSHOST_FEATURES" --test record_replay
    rm -rf "$out/i686"
    run "crosshost snapshot written by $t" \
      env RSEMU_SNAPSHOT_WRITE_DIR="$out/i686" \
      cargo test --target "$t" --no-default-features \
      --features "$CROSSHOST_FEATURES" --test crosshost_snapshot
    run "crosshost snapshot from $t loaded here" crosshost_read "$out/i686"
  fi

  local w=wasm32-wasip1
  if ! rustc --print target-libdir --target "$w" >/dev/null 2>&1; then
    crosshost_absent "crosshost wasm (target not installed: rustup target add $w)"
    return 0
  fi
  if ! command -v node >/dev/null 2>&1; then
    crosshost_absent "crosshost wasm (no node; scripts/wasi-run.mjs needs one)"
    return 0
  fi
  # `wasm32-wasip1` has no `std::thread`, so libtest's default of a thread per
  # test aborts before the first one runs.
  export CARGO_TARGET_WASM32_WASIP1_RUNNER="node --no-warnings=ExperimentalWarning $(pwd)/scripts/wasi-run.mjs"
  run "crosshost replay ($w)" \
    cargo test --target "$w" --no-default-features \
    --features "$CROSSHOST_FEATURES" --test record_replay -- --test-threads=1
  run "crosshost unit tests ($w)" \
    cargo test --target "$w" --no-default-features \
    --features "$CROSSHOST_FEATURES" --lib -- --test-threads=1 \
    core::state:: machine::
  rm -rf "$out/wasm"
  run "crosshost snapshot written by $w" \
    env RSEMU_SNAPSHOT_WRITE_DIR="$out/wasm" \
    cargo test --target "$w" --no-default-features \
    --features "$CROSSHOST_FEATURES" --test crosshost_snapshot -- --test-threads=1
  unset CARGO_TARGET_WASM32_WASIP1_RUNNER
  run "crosshost snapshot from $w loaded here" crosshost_read "$out/wasm"
}

# The long engine-divergence run: the interpreter against each translated
# engine, quantum by quantum, over a real guest.
#
# Not in the default set and not in --all, because the honest version of it
# needs a fetched Linux kernel and minutes of wall time. Two defects lived past
# `tests/a64_engines.rs`'s forty quanta *and* past twenty seconds of that boot
# (docs/testing/long-run.md); the shortest thing that would have caught either
# is this, and nothing was running it. `.github/workflows/long-run.yml` runs
# this stage nightly, which is the answer to "an opt-in nobody opts into".
#
# With no kernel the run still does something — the synthetic workloads are in
# the ordinary test suite and this stage just makes them longer — but the real
# gate skips, and `RSEMU_LONGRUN_REQUIRED` turns that skip into a failure the
# way `RSEMU_CROSSHOST_REQUIRED` does. CI sets it.
#
# `cpu-x86-lift` and `dev-pc` are the synthetic x86 leg: the frontend, and the
# 8254 and 8259A its workload needs to be interrupted by. It builds its own
# board rather than naming a `machine-*` feature, because every shipped x86
# board starts in real mode and real mode is outside the lifted subset — see
# the file. `machine-pc64` is the *kernel* leg on the same core, which does
# want a shipped board, because the whole point of it is a guest nobody
# designed for this.
LONGRUN_FEATURES="machine-arm64-virt,cpu-arm-a64-lift,machine-riscv-virt,cpu-riscv-lift,cpu-x86-lift,dev-pc,machine-pc64,jit,jit-x86"
stage_long() {
  local secs kernel initrd
  secs="${RSEMU_LONGRUN_SECONDS:-120}"
  kernel="${RSEMU_ARM64_KERNEL:-testdata/arm64/linux}"
  initrd="${RSEMU_ARM64_INITRD:-testdata/arm64/initramfs.cpio}"

  # The synthetic workloads, lengthened. No fixture, so this leg always runs.
  #
  # Its own budget rather than `$secs`: the synthetic guests are *designed*
  # around the mechanisms that broke and reach every one of them inside a
  # second, so a hundred and twenty of them would be a hundred and twenty
  # seconds of the same thing. What wants the long budget is the kernel, which
  # is the only leg that can find something nobody designed for.
  #
  # It is 30 rather than 120 for the A64 and RISC-V legs and for the x86 one
  # alike: at 30 guest seconds the x86 workload alone is about three minutes,
  # which is where the whole stage's synthetic half sits.
  local syn="${RSEMU_LONGRUN_SYNTHETIC_SECONDS:-30}"
  run "long synthetic (${syn}s of guest time)" \
    env RSEMU_LONGRUN_SECONDS="$syn" \
    cargo test --release --features "$LONGRUN_FEATURES" --test engine_longrun

  if [ ! -s "$kernel" ]; then
    if [ -n "${RSEMU_LONGRUN_REQUIRED:-}" ]; then
      record "FAIL  long kernel boot -- RSEMU_LONGRUN_REQUIRED is set, so this had to run"
      FAILED=$((FAILED + 1))
    else
      record "skip  long kernel boot (no $kernel: scripts/fetch-testdata.sh arm64-linux arm64-initramfs)"
    fi
  else
    run "long kernel boot (${secs}s of guest time)" \
      env RSEMU_ARM64_KERNEL="$kernel" RSEMU_ARM64_INITRD="$initrd" \
          RSEMU_LONGRUN_SECONDS="$secs" \
      cargo test --release --features "$LONGRUN_FEATURES" \
        --test engine_longrun -- --ignored --nocapture --test-threads=1 \
        a_real_arm64_linux_boot_agrees_across_the_engines
  fi

  # A missing AArch64 kernel used to end the stage, which would now silently
  # take the x86 gate with it.
  stage_long_x86
}

# The same gate on the x86 core: `pc64`, a stock bzImage, both engines, quantum
# by quantum.
#
# Its own function and its own budget, because `pc64` measures time differently
# from `arm64-virt`. That board runs a 100 MHz processor and has no firmware, so
# the bzImage decompresses itself from the reset vector: at 120 guest seconds
# the last thing the guest has printed is "KASLR disabled" and it is still in
# the decompressor, where 120 of `arm64-virt` is well past both defects that
# gate exists for. So RSEMU_X86_LONGRUN_SECONDS is the knob.
#
# 900 rather than something smaller is measured rather than chosen. The
# calibration run re-introduced `admit`'s interrupt-shadow refusal and this leg
# caught it at **quantum 1 280 242, 635.04 s of guest time** — the first `STI`
# the kernel executes is that far in, because everything up to
# `local_irq_enable` runs with interrupts off. The same run at 600 guest seconds
# passes with that defect planted. docs/testing/long-run.md has the table.
#
# It costs about sixteen minutes for the two engines together, which is what the
# nightly's ninety-minute budget is sized for.
#
# RSEMU_LONGRUN_SECONDS deliberately does not drive it. The two boards' guest
# seconds are not comparable budgets, and one variable driving both would make
# whichever is the shorter of them meaningless.
stage_long_x86() {
  local kernel initrd secs
  kernel="${RSEMU_X86_KERNEL:-testdata/x86/bzImage}"
  initrd="${RSEMU_X86_INITRD:-testdata/x86/initramfs-x86.cpio}"
  secs="${RSEMU_X86_LONGRUN_SECONDS:-900}"

  if [ ! -s "$kernel" ]; then
    if [ -n "${RSEMU_LONGRUN_REQUIRED:-}" ]; then
      record "FAIL  long x86 kernel boot -- RSEMU_LONGRUN_REQUIRED is set, so this had to run"
      FAILED=$((FAILED + 1))
    else
      record "skip  long x86 kernel boot (no $kernel: scripts/fetch-testdata.sh x86-linux initramfs-x86)"
    fi
    return 0
  fi
  # An absent initramfs is not fatal: with no root the kernel panics, which is
  # still a complete boot and still hundreds of guest seconds of varied x86-64
  # to compare. `machines/pc64.machine` says why an initramfs is the only root
  # this board can be given at all.
  [ -s "$initrd" ] || initrd=""
  run "long x86 kernel boot (${secs}s of guest time)" \
    env RSEMU_X86_KERNEL="$kernel" RSEMU_X86_INITRD="$initrd" \
        RSEMU_LONGRUN_SECONDS="$secs" \
    cargo test --release --features "$LONGRUN_FEATURES" \
      --test engine_longrun -- --ignored --nocapture --test-threads=1 \
      a_real_x86_linux_boot_agrees_across_the_engines
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
  # Deliberately not `long`: it wants a fetched kernel and minutes of wall
  # time, and `--all` is what somebody runs before a commit.
  --all)  want=(fast test wasm combos crosshost sweep fuzz) ;;
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
