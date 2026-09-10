# Reference host and benchmark workloads

Several phase gates in [`../ROADMAP.md`](../ROADMAP.md) §13 carry numbers — ≥ 100
MIPS, within 2× of QEMU, ≥ 80 % of native, 60 fps at a 99th-percentile frame
time. A number without a machine and a workload is not a gate, so both live
here, in version control, changed by pull request.

> **Status:** the **versus-QEMU** row is chosen, measured and published —
> [`testing/benchmarks.md`](testing/benchmarks.md) has the workload set, the
> method, the noise floor and the ratio, and the answer is that phase 8's gate
> is **not met by an order of magnitude**. That is the point of filling a table
> in: an unmeasured gate cannot be failed. The frame-rate row is still unfilled —
> the three NES titles have not been named — and a gate that cites an
> unpopulated table has not been met either way.

## Reference host

The machine the published figures were taken on. Figures from any other machine
are informative, never gating. When the reference host is replaced, re-measure
every published figure in the same commit — a mixed-provenance benchmark table
is worse than none.

| Field | Value |
| --- | --- |
| CPU | AMD Ryzen Threadripper 9970X — 32 cores / 64 threads, 1.22–5.49 GHz, 128 MiB L3 |
| Memory | 125 GiB |
| OS / kernel | Linux 6.18.41-gentoo-x86_64 |
| Rust toolchain | 1.98.0 (88d9e12ae 2026-08-18) — the pinned version, `rust-toolchain.toml` |
| CPU governor | `powersave`, `amd_pstate` active, boost enabled |
| Mitigations | as configured by the distribution; changing them changes the numbers |

**Two honesty notes about this particular host**, both of which belong in the
register rather than in a footnote on one page:

* The governor is `powersave`, not `performance`. That is what the machine is
  configured with, and a figure taken under one governor and labelled as the
  other is worse than a slower figure.
* It is a **shared development machine** and the published figures were taken
  while several other builds were running (load average ~50 on 64 cores). Every
  measurement here must therefore be interleaved and reported as a minimum over
  repetitions, with its run-to-run spread beside it — which is what
  [`testing/benchmarks.md`](testing/benchmarks.md)'s harness does. Absolute
  seconds from this host are pessimistic; ratios between two sides that were
  interleaved through the same load are not.

## Workloads

Committed fixtures, pinned by content hash, downloaded like any other corpus
(never vendored — see [`testing/conformance-suites.md`](testing/conformance-suites.md)).

| Gate | Workload | Metric | State |
| --- | --- | --- | --- |
| Versus QEMU | boot, hash, awk and gzip on an `arm64-virt` and a `pc64` Linux guest, run black-box under both (§1) | wall-clock ratio | **measured: about 20× on `arm64-virt`, about 110× on `pc64`, against a gate of 2×** — [`testing/benchmarks.md`](testing/benchmarks.md) |
| CPU throughput | `coremark`, RV64GC, single hart | retired guest instructions/second, counted by the interpreter's own counter | not yet run |
| Acceleration | the same CPU-bound workload, accel vs. native | percentage of native | `tests/kvm_native_ratio.rs` |
| Console frame rate | three named commercial NES titles | emulated fps, and 99th-percentile frame time | titles not yet chosen |

The three NES titles are named here once chosen, so "a real game runs at 60 fps"
cannot quietly become "the easiest game we could find".

The versus-QEMU row deliberately does **not** use `coremark`. The argument is on
[`testing/benchmarks.md`](testing/benchmarks.md): a cross toolchain per guest
buys a score to quote, where the busybox already inside the fetched initramfs
buys four workloads of different shapes for no new fixture — and, decisively,
the *same binary* runs under both emulators, which is the property a comparison
needs and a per-architecture rebuild does not have.

## Method

- Report the median of 5 runs plus the interquartile range. A single number
  hides variance, and variance is where regressions live.
- **Interleave the sides**; never run all of A and then all of B. On a machine
  that drifts, A-then-B measures the drift.
- Warm up before measuring; report cold-start separately when it matters.
- Record the rsemu commit, the toolchain version, and the feature set — a
  machine is a feature set (§3), so the build is part of the measurement.
- Publish the raw numbers alongside the ratio. Ratios drift silently when the
  baseline changes.
- **Name the denominator.** A round here was rejected for calling 1.27× "27 %
  faster" of the wrong baseline; a ratio without the thing it is a ratio *of* is
  not a result.
