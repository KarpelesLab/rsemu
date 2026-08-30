# Reference host and benchmark workloads

Several phase gates in [`../ROADMAP.md`](../ROADMAP.md) §13 carry numbers — ≥ 100
MIPS, within 2× of QEMU, ≥ 80 % of native, 60 fps at a 99th-percentile frame
time. A number without a machine and a workload is not a gate, so both live
here, in version control, changed by pull request.

> **Status:** not yet chosen. The first phase with a performance gate is the NES
> milestone; this file must be filled in before that gate can be claimed, and a
> gate that cites an unpopulated table has not been met.

## Reference host

| Field | Value |
| --- | --- |
| CPU | *TBD* — exact model, base/boost clock, core count |
| Memory | *TBD* — capacity, speed, channels |
| OS / kernel | *TBD* |
| Rust toolchain | *TBD* — pinned version used for published figures |
| CPU governor | `performance`, turbo state recorded |
| Mitigations | recorded as configured; changing them changes the numbers |

Figures from any other machine are informative, never gating. When the reference
host is replaced, re-measure every published figure in the same commit — a
mixed-provenance benchmark table is worse than none.

## Workloads

Committed fixtures, pinned by content hash, downloaded like any other corpus
(never vendored — see [`testing/conformance-suites.md`](testing/conformance-suites.md)).

| Gate | Workload | Metric |
| --- | --- | --- |
| CPU throughput | `coremark`, RV64GC, single hart | retired guest instructions/second, counted by the interpreter's own counter |
| Versus QEMU | the same workload set, run black-box under both (§1) | wall-clock ratio |
| Acceleration | the same CPU-bound workload, accel vs. native | percentage of native |
| Console frame rate | three named commercial NES titles | emulated fps, and 99th-percentile frame time |

The three NES titles are named here once chosen, so "a real game runs at 60 fps"
cannot quietly become "the easiest game we could find".

## Method

- Report the median of 5 runs plus the interquartile range. A single number
  hides variance, and variance is where regressions live.
- Warm up before measuring; report cold-start separately when it matters.
- Record the rsemu commit, the toolchain version, and the feature set — a
  machine is a feature set (§3), so the build is part of the measurement.
- Publish the raw numbers alongside the ratio. Ratios drift silently when the
  baseline changes.
