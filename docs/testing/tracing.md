# Tracing and profiling output

`ROADMAP.md` phase 9 names "tracing/profiling output" in one clause and nothing
implemented it. This is that, and this file is what it emits, why those things
and not others, what it costs, and what is specified but not yet wired.

```
rsemu run pc64 --media kernel=bzImage --for 900s --headless --trace all=boot.trace
```

Nothing here needs a fetched fixture to try, though. Eight bytes of RV64 —
`addi a0, a0, 1` and a branch back to it — are a guest:

```
printf '\x13\x05\x15\x00\x6f\xf0\xdf\xff' > loop.bin
rsemu run riscv-virt --media firmware=loop.bin --for 100ms --headless \
    -p engine=jit-host --trace all
```

```
# rsemu-trace 1
# machine         riscv-virt
# guest-ns        99999999
# threading       deterministic
# channels        sched,cpu,clock
# state-hash      0xe49df53d39dbf916
# cpu0            engine=jit-host
clock.clint.ticks               1000000
clock.cpu0.ticks                1000000
clock.uart.ticks                  11520
cpu.blocks                         7902
cpu.compiled                       7902
cpu.cpu0.blocks                    7902
cpu.cpu0.compiled                  7902
cpu.cpu0.cycles                 1000001
cpu.cpu0.retired-total           500000
sched.budgets                       200
sched.events                          0
sched.quanta                        104
sched.quanta.empty                    4
sched.quanta.idle                     4
sched.span-ns                  99999999
sched.span-ns.log2.00                 4
sched.span-ns.log2.20               100
sched.ticks                     1011520
```

Half a million guest instructions in 7 902 blocks, every one of them compiled to
host code; a hundred scheduler rounds of a millisecond each plus four the
headless loop declined at its slice boundaries. None of that was printable
before.

## Why this exists

The tree has needed exactly this repeatedly and improvised it every time, and
each improvisation was a private patch that was reverted afterwards. Three from
the last five rounds:

* `Host::new` instrumented by hand to print `(pc, cycles, edge, pending)` on
  every round, to find the *one* line in two guest seconds where `pending` was
  true — the defect
  [`long-run.md`](long-run.md) records as "the edge computed across an entry
  walk".
* Block entries, chained entries, translations and invalidations counted by
  hand, to prove a workload really stressed the seams it claimed. That is why
  [`long-run.md`](long-run.md) can say "982 618 of 1 220 450 block entries in a
  guest second are chained" and "106 058 translations thrown away against
  106 089 made" as *prose*: there was no way to print them.
* Per-function host-instruction counts, reached for through callgrind, which
  cannot see guest-level structure at all — it can tell you `Exec::step` cost
  36 818 446 752 instructions and not that 98.7% of the guest retired inside a
  block.

Every published figure in [`../platforms/pc64.md`](../platforms/pc64.md) §"the
block-fraction headline" and [`../platforms/arm64-virt.md`](../platforms/arm64-virt.md)
§"mechanism counters" was obtained that way: read out of a debugger, or out of
an `eprintln!` that no longer exists.

## What it emits, and why those things

**Counters, not an event stream.** Every one of the improvisations above was a
*count*; not one of them was a stream. That decides the design on its own. A
record per block entry is hundreds of millions of records for a single run —
a cost the block-entry path cannot carry and an output nobody reads — while a
total answers the same question in eight bytes. Where a distribution rather
than a total is wanted (how long a scheduler round ran) there is a
power-of-two histogram, which is still counters.

**Most of the numbers already existed.** `jit::DispatchStats` has counted block
entries, chained entries, lookups and translations since the block cache
landed; `cpu::x86::JitStats` and `cpu::arm::a64::JitStats` add
retired-inside-a-block against interpreted. What did not exist was any way to
get them *out of a process*: they are reachable from a `&X86`, and a built
machine hands back `Arc<dyn Device>`. So the first thing this subsystem does is
not add counters but open a door to the ones already there — which is also why
the `cpu` channel costs the block-entry path nothing at all.

### The channels

| `--trace` | What it reports | Where it comes from |
| --- | --- | --- |
| `sched` | scheduler rounds: how many, how long each was, how many gave a runnable no budget, how many events came due, and a power-of-two histogram of round length in nanoseconds | one hook at the end of `Machine::advance_to` |
| `cpu` | per processor: blocks entered, blocks run as host code, blocks reached by following a patched exit, distinct blocks lifted, translations thrown away, guest instructions retired inside a block against interpreted one at a time — and, on A64, inlined-TLB load and store counts | each core's own `jit_stats`, read once when the run ends |
| `clock` | per-clock-domain tick totals | `ClockForest::ticks`, read once when the run ends |
| `all` | every channel above | |

### The rows

`sched`:

| Row | Meaning |
| --- | --- |
| `sched.quanta` | rounds run |
| `sched.quanta.idle` | rounds in which no runnable was given a budget: a boundary `run_until` **declined** because the deadline fell inside the round, or a machine with nothing runnable |
| `sched.quanta.empty` | rounds that advanced virtual time by nothing |
| `sched.budgets` | runnable budgets issued — `budgets / quanta` is the machine's runnable count |
| `sched.ticks` | ticks consumed by runnables, summed across domains (a volume of work, deliberately not converted to a time) |
| `sched.events` | events dispatched out of rounds |
| `sched.span-ns` | virtual nanoseconds the rounds covered — equal to the run, which is a useful self-check |
| `sched.span-ns.log2.NN` | how many rounds needed *NN* bits of nanoseconds. Bucket 20 is about a millisecond; bucket 10 about a microsecond |

`cpu`, under `cpu.<instance path>.` and again as a machine-wide total under
`cpu.`:

| Row | Meaning |
| --- | --- |
| `blocks` | translated blocks entered |
| `compiled` | of those, entered as host code rather than interpreted IR |
| `chained` | of those, reached by following a patched exit rather than a cache lookup |
| `translated` | distinct blocks lifted |
| `invalidated` | translations thrown away because the guest wrote into the page they came from. A64 splits this into `invalidated.in-block` and `invalidated.interpreted`, because those are separate mechanisms and a single total lets one of them stop working while the other holds the number up |
| `retired` | guest instructions that retired **inside** a block |
| `interpreted` | guest instructions the interpreter executed, one per call |
| `retired.permille` | `retired` per thousand of `retired + interpreted` — the documents' "99.3%" is this divided by ten. Integer, because the determinism rule has no room for a float in anything a run produces |
| `fast-loads` / `fast-stores` | (A64) compiled accesses served from an inlined software-TLB probe |

A board whose processors keep none of these — a 6502, an accelerated core, a
build with no translation runtime — gets the header line `# cpu  no processor
in this machine keeps translation statistics` and **no rows**. "Zero blocks
executed" and "nothing here counts blocks" are different facts, and a column of
zeroes reads as "the JIT is broken".

### What was considered and left out

* **A per-block-entry event stream.** See above: the cost and the volume both
  rule it out, and every question anybody actually asked was a total.
* **A wall-clock figure.** Deliberately absent — see *Determinism* below.
* **MMIO by region.** Wanted, specified below, and not implementable from
  outside `core::space`: the flat view carries an `Arc<dyn MemOps>` and no name.
* **A reason for each quantum boundary** (allowance spent, timer edge, exit
  flag, declined boundary). Also wanted, also specified below: the scheduler has
  no "why did this round end" type, and `sched.quanta.idle` is the part that
  *is* observable from a `QuantumReport`.

## The format

Two whitespace-separated columns, sorted by name, under a `#` header. That is
the whole specification; there is no parser, because there is nothing to parse.

* A person reads it as a table.
* A script reads one row with a single awk pattern on the first field, or
  totals a family with a prefix match.
* `diff` between two traces is the answer to "what did that change do".

It costs no dependency, which the policy requires: no `serde`, no external
format, no encoder. Rendering is thirty lines in `core::trace`, and it is
`no_std + alloc`, so a wasm embedder gets a trace out of a browser too.

Names are dotted and hierarchical (`cpu.cpu0.chained`), so the prefix is the
channel and the tail is the quantity. Histogram buckets are zero-padded
(`log2.09`, `log2.23`) so that the sorted order is the numeric order.

## Determinism

**A trace must not change what the guest does or when.** Three rules make that
structural rather than aspirational:

1. **Nothing in the trace path reads a clock.** The one live hook is *handed*
   the span the scheduler already computed, so there is no time source in the
   subsystem to perturb. `CLAUDE.md`'s "no wall-clock reads outside `host/`" is
   satisfied by there being nothing to read.
2. **Nothing in it is readable by the guest.** The counters are not device
   state, are in no snapshot chunk, and are not in `Machine::state_hash`.
3. **Nothing in it allocates or locks on a path a guest can feel.** A counter is
   a relaxed `fetch_add` on a static; the table that allocates is built once,
   after the run.

Two tests assert the consequences rather than the rules, in `tests/cli_trace.rs`:

* `tracing_does_not_change_what_the_guest_does` — the same workload run with
  tracing off and with every channel on reaches the **identical state hash**,
  and the whole summary is identical besides. That is not a proxy for the
  property; `Machine::state_hash` is the whole machine.
* `a_trace_of_a_deterministic_run_is_itself_deterministic` — two traces of one
  workload are **byte-identical**. That is why there is no timestamp and no
  wall-clock figure anywhere in the output: host time is callgrind's and
  `perf`'s business, and a file with a timestamp in it cannot be diffed.

The first test also guards something narrower. The `cpu` channel reaches a
core's statistics by *replacing that class's constructor*, because there is no
route from a `dyn Device` to a concrete core — so the constructor call is
written twice, once in the core's `bind` and once in `host::trace::install`. A
drift between them builds a subtly different machine, and the state hash is what
notices.

### A parallel or accelerated run

`--trace sched` and `--trace clock` work under any threading mode. The header
says `state-hash  not reproducible under this threading mode` rather than
printing a number, exactly as `rsemu run`'s summary does. `--trace cpu` with
`--accel` is **refused**: an accelerated processor runs on the host's own
silicon and translates nothing, so it keeps none of these counters, and a table
of zeroes would be a wrong answer rather than no answer. `--trace all` with
`--accel` leaves `cpu` out and says so on stderr — `all` is a wildcard and means
"every channel this run can report", while a channel somebody *named* is a flag
that cannot be honoured.

## Cost

The design's whole virtue is that the hot paths are not touched. The `cpu` and
`clock` channels read a `u64` once, after the guest has stopped. The `sched`
channel's hook is per *scheduler round* — a few thousand per guest second, not
the hundreds of millions per run that block entry is.

Measured with callgrind, on this host, `--release`:

### The block-entry and store paths are untouched

`benches/x86_dispatch --smoke` — the block-entry and memory workload
[`../platforms/pc64.md`](../platforms/pc64.md) uses for its own callgrind
comparisons — built `--features cli,machine-apple1,machine-pc64,cpu-x86-lift,jit,jit-x86`
with and without `trace`, whole-program `Ir`:

| build | host instructions | vs. control |
| --- | --- | --- |
| without the `trace` feature (control) | **72 365 358 429** | — |
| with the `trace` feature | **72 365 361 713** | +3 284, **+0.0000045%** |

Three thousand instructions out of seventy-two billion is process start-up: the
two binaries are not byte-identical, and neither is their dynamic-loader work.
The emulation is the same emulation, because the bench never reaches
`Machine::advance_to` and there is no hook anywhere else. That is the design's
whole claim, measured: **the `cpu` channel adds nothing to the block-entry path
because it does not touch it** — the counters it reports were already being
incremented there, and it reads them once when the guest has stopped.

### The one hook, compiled in and switched off

`rsemu run apple1 --for 10s --headless`, which is 10 994 scheduler rounds
(`sched.quanta`), same three-way comparison:

| build | host instructions | vs. control | per round |
| --- | --- | --- | --- |
| without the `trace` feature (control) | **5 067 326 448** | — | — |
| with the feature, nothing enabled | **5 067 392 222** | +65 774, **+0.0013%** | +6.0 |
| with the feature, `--trace sched` | **5 068 330 696** | +1 004 248, **+0.0198%** | +91.3 |

Six host instructions per *scheduler round* is the price of the feature being
compiled in — and that figure is an upper bound on the hook itself, because the
binary also grew the flag's own parsing. Ninety-one per round is the price of
actually counting: five relaxed `fetch_add`s, a histogram, a sum over the
round's runnables, and a share of building and writing the file.

**Without the feature the cost is exactly zero**, not nearly zero: every
function in `core::trace` has an empty body, so the call site and everything it
guards is deleted. That is the compile-time gate `CLAUDE.md` asks for when "off"
is not free, and it is what makes it safe to put a hook on a hot path later —
the MMIO hook specified below is the first one that would need it.

## Reproducing a measurement that was taken by hand

The x86 synthetic leg of [`long-run.md`](long-run.md) is the workload whose
figures were read out of a debugger. Running it for one guest second and
collecting the same channels this document describes:

| [`long-run.md`](long-run.md) says | the trace says |
| --- | --- |
| "982 618 of **1 220 450 block entries** in a guest second are chained" | `cpu.blocks 1220450`, `cpu.chained 982618` |
| "**106 058 translations thrown away** against **106 089** made" | `cpu.invalidated 106058`, `cpu.translated 106089` |
| "**98.7%** of that guest's instructions retire inside a block" | `cpu.retired.permille 987` (8 367 119 retired against 106 500 interpreted) |
| "one guest second of that board is **24 818 quanta**" | `sched.quanta 49636`, which is 24 818 twice — see below |

Every figure agrees exactly. What took a private patch and a reverted commit is
now four rows of a file.

> The doubled round count is the design's documented behaviour rather than a
> defect, and it is worth seeing once: the lockstep harness runs *two* machines
> in one process — an interpreted oracle and the engine under test — and the
> `sched` counters are process-global, so `sched.quanta` was 49 636 and
> `sched.span-ns` two guest seconds. The `cpu` rows are unaffected, because
> those are read off a specific core rather than pushed into a static. `rsemu
> run` runs one machine per process and does not have the problem;
> `core::trace::reset()` is what a caller with two of them uses.

## The counters are process-global

Deliberately, and it is the one property that surprises. A hook belongs wherever
the event happens — inside `jit::Dispatcher::run`, inside a lifted block's store
path — and in none of those places is there a `Machine` in hand to attribute the
count to. A per-machine table would be a table only the outermost hooks could
reach, which is the opposite of what this is for.

Two consequences, both real:

* A process running two machines gets their sum for the `sched` channel.
  `core::trace::reset()` exists for a caller that wants them apart, and `rsemu
  run` runs one machine per process.
* **A test that enables a channel a hook writes to must own its process.**
  `cargo test` runs a target's tests on several threads, and dozens of tests in
  this crate run a machine, so a unit test that switched `sched` on would count
  theirs as well as its own. That is why every exact assertion about `sched`
  lives in `tests/cli_trace.rs`, which runs the shipped binary, and why the unit
  tests use a channel no hook feeds.

## Hooks that are specified but not applied

Each of these needs a change inside a file this subsystem does not own. They are
written out precisely enough to apply as-is.

### 1. A reason for each quantum boundary — `src/core/sched.rs`

`sched.quanta.idle` conflates a *declined boundary* with *a machine with nothing
runnable*, because a `QuantumReport` cannot tell them apart, and neither
"allowance spent" nor "a timer edge" is visible at all. The scheduler already
chooses between exactly three candidate end-instants in `Scheduler::natural_target`
and takes a fourth path in `Scheduler::decline_round`. Give that choice a name:

```rust
/// Why a scheduler round ended.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ended(pub u16);

impl Ended {
    /// The round reached the quantum grid point: the allowance was spent.
    pub const ALLOWANCE: Ended = Ended(0);
    /// A queued event came due before the grid point.
    pub const EVENT: Ended = Ended(1);
    /// A lazily advanced device's own deadline came first.
    pub const LAZY: Ended = Ended(2);
    /// `run_until` declined a boundary the deadline fell inside.
    pub const DECLINED: Ended = Ended(3);
    /// A runnable raised its exit flag (stop-the-world, a debugger, `SIGINT`).
    pub const EXIT: Ended = Ended(4);
}
```

* `natural_target` returns `(GlobalTime, Ended)` instead of `GlobalTime` — the
  three arms already exist, one per candidate, and each names its own constant.
* `QuantumReport` grows `pub ended: Ended`.
* `decline_round` sets `Ended::DECLINED`; `close_round` takes the value
  `natural_target` returned, except that a round cut short by a raised exit flag
  reports `Ended::EXIT`.

Then `core::trace::quantum_report` counts `Counter(8 + report.ended.0)` — the
slots are already reserved — and the rows appear as `sched.ended.allowance`,
`sched.ended.event`, `sched.ended.lazy`, `sched.ended.declined`,
`sched.ended.exit`. No other file changes.

### 2. MMIO by region — `src/core/space/flat.rs`

A device MMIO count needs an identity at the dispatch site and there is none:
`FlatTarget::Io` carries only `Arc<dyn MemOps>`, and the human-readable name
lives on `Region`, one layer up and unreachable from the flat view.

The change is to thread the identity in at *build* time, where the name is in
hand and the cost is paid once:

* `FlatTarget::Io(Arc<dyn MemOps>)` becomes `FlatTarget::Io(Arc<dyn MemOps>,
  RegionId)`, where `RegionId` is the existing `MappingId` or a dense index
  assigned when the flat view is built. A dense `u16` index is preferable: the
  counter array is indexed by it directly.
* `flat::leaf_target` and the `RegionKind::Io` arm of the `Piece` construction
  are the two places that build one, and both have the `RegionRef` in hand.
* At `flat.rs`'s `FlatTarget::Io` arms in `read`, `write` and `write_value`, one
  line each:
  ```rust
  trace::add(Channel::MMIO, Counter(MMIO_BASE + id.0 * 2 + is_write as u16), 1);
  ```
* `host::trace` renders those against a name table the flat view publishes
  alongside, so the rows read `mmio.pit0.read` rather than `mmio.7.read`.

`Channel::MMIO` and the counter slots it needs do not exist yet and are the
easy half: [`Channel`] and [`Counter`] are `#[repr(transparent)]` newtypes with
`pub const` variants precisely so that adding one breaks no `match`, and
`SLOTS` is a compile-time constant over zero-initialised static data. Per-region
slots want more than the forty-eight there are now; a second array indexed by
region, sized from the flat view, is the shape.

[`Channel`]: ../../src/core/trace.rs
[`Counter`]: ../../src/core/trace.rs

This is the only hook in this document that touches a path with a per-access
cost, and it is the reason it is specified rather than applied: it wants the
same callgrind measurement the store path already has in
[`../platforms/pc64.md`](../platforms/pc64.md), taken by whoever owns
`core::space`.

### 3. RISC-V's retired-versus-interpreted split — `src/cpu/riscv/engine.rs`

`cpu::riscv::Jit::stats()` returns `(blocks, compiled)` where x86 and A64 return
a struct with `retired` and `interpreted` in it, so the `cpu` channel's most
useful row is missing on RISC-V. The count is already computed: `Run::insns`
is folded into `minstret` at `engine.rs`'s `exec.st.csrs.minstret =
exec.st.csrs.minstret.wrapping_add(run.insns as u64)`. Add `retired: u64` and
`interpreted: u64` to `Jit`, accumulate `run.insns` into the first at that same
line and one per `interpret()` call into the second, and widen `jit_stats` to a
struct in the shape of the other two. `host::trace::cpus`'s RISC-V arm then
loses its `retired-total` fallback and reads the same rows as the other cores.

### 4. Per-mechanism block counters — `src/jit/dispatch.rs`

`DispatchStats` already has `looked_up`, `resyncs` and `smc`, and
`jit::CacheStats` has eleven more (`hits`, `misses`, `flushes`, `links`,
`stale_links`, `filtered`, `evictions`, `unlinks`). None of them reaches the
`cpu` channel, because the per-core `JitStats` structs do not carry them. No
*hook* is needed — the counters exist — only a widening of
`cpu::x86::engine::Stats` and its A64 twin to carry the dispatcher's and the
cache's own totals through. That is a change in three files, all owned by the
CPU cores, and it is worth doing the next time one of them is opened: the eleven
cache counters are exactly what a "why is this workload translating so much"
question wants.

## See also

* [`long-run.md`](long-run.md) — the harness whose hand-taken numbers this
  reproduces, and the two engine defects it caught.
* [`../platforms/pc64.md`](../platforms/pc64.md) — the block-fraction and
  interpreter-entry tables, and the callgrind method this file follows.
* [`../bench-host.md`](../bench-host.md) — the reference host and the workloads
  the phase gates are measured on. Wall clock lives there; guest structure lives
  here.
