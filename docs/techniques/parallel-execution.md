# Parallel guest execution, and what "correct" means without a state hash

Consumed by: `core/sched` (the dispatched round), `machine/` (which mode a
board asks for), `tests/parallel_threading.rs`,
`tests/parallel_smp_boards.rs`, `tests/memory_model_litmus.rs` (whose `machine`
module is the litmus half of the same argument), phase 8. Companion to
[`memory-models.md`](memory-models.md), which is about the *ordering* rules a
parallel run has to preserve; this file is about what a parallel run is allowed
to be, and how anyone ever knows it is right.

## The contradiction, stated plainly

`ROADMAP.md` §0 makes determinism a non-negotiable: run for N virtual units,
hash the state, compare. Every golden in this tree, every conformance ledger,
record/replay and rewind all rest on that one sentence.

`ROADMAP.md` §4.2 also wants SMP boards whose processors execute at the same
instant, because a two-core board whose cores take turns is not an SMP board —
it is a single-core board with extra bookkeeping, and no atomicity bug, no
barrier omission and no lock-free algorithm in a guest kernel can be observed
on it.

Those two cannot both be properties of one run. Two host threads inside two
guest cores interleave at instants the *host's* scheduler picks, so a run
records a sample of a distribution and not a value.

## The reconciliation: determinism is a property of the mode

The project's premise is not "every run is reproducible". It is **"a
reproducible run is always available, and it is the default"**. That is what
[`ThreadingMode`](../../src/core/sched.rs) encodes, and
`ThreadingMode::is_deterministic` is the one predicate everything that depends
on reproducibility asks, rather than each caller re-deciding what "deterministic
enough" means.

| | `deterministic` | `parallel` | `accel` |
| --- | --- | --- | --- |
| host threads | one | one per runnable | one per vCPU, in hardware |
| finest interleaving | one whole guest instruction | the host's, inside a round | the host's silicon |
| virtual time comes from | the board's oscillators | the board's oscillators | the wall clock |
| `Machine::state_hash` | a number | **refused** | **refused** |
| record / replay | yes | **refused** | **refused** |
| the default | **yes** | opt-in | `--accel <backend>` |

The refusals are the load-bearing part and they are structural, not advisory:
`Machine::state_hash` returns an error outside a deterministic mode, so a
conformance suite, a frame-hash golden or a replay trace **cannot** be blessed
against a parallel run by accident, because the call that would produce the
number does not produce one. `Machine::nondeterministic_state_hash` is the
escape hatch for the one legitimate use — both sides of a comparison coming
from a single run, such as a snapshot taken and restored inside it — and its
name is its documentation.

`Machine::set_recorder` refuses for a narrower and more interesting reason,
worth repeating here because "parallel is non-deterministic" is not it. A
parallel round *does* join every job before it returns, so the round boundaries
the record/replay seam timestamps against are reproducible under `parallel`
too. What is not reproducible is what happens **inside** a round. No input log
can recover that, so a recording taken from a parallel run would replay into a
different machine while looking entirely valid.

## So what does `parallel` promise?

Everything except the interleaving, and the list is worth being exact about
because "non-deterministic" is usually read as "anything may happen".

**Guest time is unchanged.** Budgets, the event queue, every tree's tick
counters and the quantum grid all come from the same absolute grid the
deterministic mode uses. A machine does not run *faster in guest time* under
`parallel`, only in host time. A board's declared frequencies still mean what
they say; that is the difference between this mode and `accel`, where virtual
time is slaved to the wall.

**Every round is a rendezvous.** One job per runnable goes to the `core::sync`
task pool at the start of a round; the round ends when every one of them has
been joined. That join is both the barrier and the happens-before edge that
makes the next round's bookkeeping see everything the last round's runnables
wrote. So the machine's state *at a round boundary* is a legal state of the
board — not a torn snapshot of one — and everything that runs between rounds
(event dispatch, lazy-device catch-up, the deferred queue) runs with nothing
executing.

**Stopping the world still works.** `SafePoint` is a generation counter and a
per-runnable `ExitFlag` checked at block boundaries — never a host signal,
because wasm has none. A snapshot taken under `Machine::stop_the_world`
restores and continues, and `tests/parallel_threading.rs` asserts it on real
CPU interpreters.

**No threads are spawned behind anyone's back.** Work is submitted to the
seam's pool, and a pool with zero workers runs jobs inline. That is the
no-threads browser build and every `no_std` host, and it is a supported
configuration rather than a fallback (§11.3).

## What "correct" means here, since it cannot mean "the hash matched"

A parallel run is checked by **refutable architectural claims** instead of by a
golden. The distinction that makes this work, and it is the same one
`tests/riscv_amo_atomicity.rs` and `tests/a64_lse_atomicity.rs` both turn on:

> **A lost update is not a reordering.** It is a value that *no* interleaving of
> the two guest programs could have produced. So any host that runs the two
> threads at once shows it, the assertion can be an equality rather than a
> printed count, and a run that fails is a defect rather than a bad sample.

That gives four kinds of check, in descending order of how much they prove.

1. **Equalities that no interleaving permits.** Two cores each performing *N*
   atomic increments of one word must leave `2N`. Two store-conditionals
   against one reservation granule must not both succeed. A naturally aligned
   load must return a value that was in memory at some instant. These are
   gates: any violation is a bug, and the test asserts equality.
2. **Outcomes a barrier forbids.** The store-buffer litmus test — both loads
   returning zero — is what `MFENCE`, `DMB` and `FENCE` exist to prevent. These
   are **not** gates on an x86-64 host, and `tests/memory_model_litmus.rs` says
   so at length: a host strong enough to hide the reordering hides it before
   and after the fix. They gate on a weak host, which is why an AArch64 CI leg
   exists. On a *machine* they are weaker still, and the measurements below say
   by how much: an interpreted guest instruction between the store and the load
   is about as wide as the host's whole store-buffer window, so the machine-level
   rows check that the barrier is executed rather than discriminate between
   having one and not.
3. **A real guest kernel that boots and stays up.** An SMP kernel's own
   spinlocks, IPIs, per-CPU areas and RCU are an atomics stress suite somebody
   else wrote and debugged. See the measurements below.
4. **A witness that the cores actually collided.** Every one of the above is
   worthless in a run where the host scheduler never let the two threads
   overlap, so the tests carry a *plain* counter beside the atomic one — three
   instructions on a word the architecture promises nothing about. A run in
   which the plain counter reaches its full `2N` is a run whose atomic result
   is not evidence, and the test says so out loud rather than passing quietly.

**How a regression is caught without a state hash**, in one sentence: the
deterministic mode keeps the goldens, and the parallel mode is gated on
equalities plus a collision witness — so a change that breaks atomicity fails a
`==`, and a change that breaks timing fails a hash in the other mode.

## What a machine file may say

```
machine "smp-parallel" {
  threading parallel
  …
}
```

`threading` is a statement about the **hardware**: these processors execute at
the same instant, the way an SMP board's do and the way a NES's CPU and PPU do
not — there, one drives the oscillator tree and the other is computed from it.
It takes `deterministic` or `parallel` and nothing else. `accel` is refused
with a message, because that word does not describe a board at all: it says the
host's own silicon is executing the guest, and `rsemu run … --accel <backend>`
is what selects it.

Three things it deliberately cannot say:

* **A worker count.** How many host threads to spend is a property of the run.
  `--threading parallel:4` sets it; the file has no spelling for it.
* **The last word.** `--threading` on the command line, and
  `RealizeOptions::threading` in Rust, override the file. The regression suite
  depends on being able to put any board back into `deterministic`, and
  `tests/parallel_smp_boards.rs` asserts that it can.
* **Anything, in a shipped board.** No file in `machines/` declares a
  threading mode, and that is a decision rather than an omission — see below.

A library caller that builds a file-selected `parallel` machine still gets
`SchedulerConfig::workers == 0`, which runs jobs inline: `realize` will not
spawn threads a caller did not ask for. `rsemu run` sizes the pool itself.

## Why the shipped SMP boards do not declare it

`pc-at-smp`, `q35-linux-smp`, `arm64-virt-smp` and `riscv-virt-smp` are real
two-processor boards and they stay `deterministic` by default. The argument is
short: a state hash is refused outside a deterministic mode, so a shipped board
that declared `parallel` would take *itself* out of the regression suite for
everybody who builds it — including the snapshot round-trip, the reproducibility
check and every future golden. That is a permanent cost paid for a default
nobody asked for.

What the boards get instead is honesty and a one-line switch. `--threading
parallel` runs any of them concurrently today; a user's own copy of the file
takes one added line. `machines/tests/smp-parallel.machine` is the in-tree
board that *does* declare it, and it exists so the path is exercised by the
suite rather than only documented.

## The degenerate configuration every one of those boards has

All four shipped `-smp` files put both processors on **one** oscillator, and so
does the fixture, deliberately. That configuration is outside the clock model,
in *both* threading modes, and `Scheduler::ticks_until_after` has said so all
along: a tree has one unit counter and every domain on it is a divider of that
counter, so one counter cannot express one processor halted while the other
runs. Two cores want two roots, which is also what §4.2 says the hardware has —
"as many roots as the real board has crystals".

What keeps it working is not a fix, it is a cap. The parallel round hands out
every budget before anything runs, so it *reserves*: each runnable on a tree is
offered the span its predecessors could not have used, and the first one would
take all of it were it not for `SchedulerConfig::max_ticks_per_quantum`. On a
100 MHz board with a 1 ms quantum the round's span is 100 000 ticks and the cap
is 10 000, so both processors get 10 000 and both run.

Raise the cap past the round's whole span and **the second processor is handed
a budget of zero, every round, for ever.** The deterministic mode survives the
same configuration for a different reason — it rotates its round-robin cursor,
so the two processors alternate whole rounds — and the parallel round has no
cursor to rotate.

`tests/parallel_smp_boards.rs::one_oscillator_and_no_tick_cap_starves_the_second_hart`
asserts exactly that, so the limitation is a fact in the suite rather than a
paragraph, and a fix has to come here and delete both.

## Measured

Everything below is an x86-64 Linux host, `--release`, and is a measurement
rather than a promise.

**A real SMP kernel boots on both processors under `parallel`.** Debian's
`arm64` installer kernel on `machines/arm64-virt-smp.machine`, `--threading
parallel`, 1 GiB, with a busybox ramdisk:

```text
[    0.022103] smp: Bringing up secondary CPUs ...
[    0.022611] Detected PIPT I-cache on CPU1
[    0.022637] CPU1: Booted secondary processor 0x0000000001 [0x410fd034]
[    0.022746] smp: Brought up 1 node, 2 CPUs
```

The same run in `deterministic` reaches the same four lines at `0.022736`
rather than `0.022746` — a ten-microsecond difference in guest time that is
precisely the non-determinism this mode buys, and the reason a hash from it
would be a sample.

**And it is slower.** 90 seconds of virtual time on that board: **137 s** of
wall in `deterministic`, **187 s** in `parallel`. That is not a surprise and it
is not a regression — `ThreadingMode::Parallel`'s own documentation has a table
saying two runnables at the default `max_ticks_per_quantum` are *slower* in
this mode, because a round costs a dispatch per runnable and the barrier is per
round. A two-processor board wants a larger cap before it asks for parallelism;
the mode's speedup is a four-processor-and-up proposition.

**`AMOADD` loses nothing through a whole machine.** `tests/parallel_smp_boards.rs`,
two RV64 harts, 20 000 atomic increments each:

```text
two harts, 20000 increments each: the plain counter lost 9596 of 40000,
the atomic one lost 0
```

The first number is the witness: nearly a quarter of the *plain*
read-modify-writes were lost to the other hart landing inside them, so the two
programs demonstrably overlapped and the zero beside it means something. This
is `tests/riscv_amo_atomicity.rs`'s claim moved from two hand-spawned host
threads onto a board a user can run, which is a strictly stronger statement:
the machine file, the realizer, the scheduler's dispatch, the address space and
the RAM store are all in the picture now.

**The litmus tests run against a machine now, and they are a blunter
instrument there.** `tests/memory_model_litmus.rs`'s `machine` module runs two
litmus shapes as guest programs on two boards that declare `threading
parallel` — `machines/tests/smp-parallel.machine` (two RV64 harts) and
`machines/tests/smp-parallel-a64.machine` (two Neoverse-N1-class cores, added
for this). Ten rows: store-buffering and message-passing, each with the
architecture's barrier, with `STLR`/`LDAR` where the architecture has it, and
with neither.

The harness is *in the guest*, and it has to be. A host-thread litmus stops
both threads after every round and resets the flags; a machine cannot be
stopped that finely without spending a whole quantum per litmus round, so both
processors instead run a self-refereeing program that paces itself against the
other with monotone counters, writes one byte per round saying what it
observed, and parks. Rust joins the two records afterwards. Nothing in the
guest ever reads a word the other processor wrote in order to decide an
outcome, which is the one thing a memory-model test may not do.

| row, 20 000 rounds, x86-64 release | forbidden outcome | rounds where one load was stale |
| --- | --- | --- |
| rv, SB, `fence rw,rw` | **0** | 111–1 698 |
| rv, SB, neither | **0** | 1 050–8 365 |
| a64, SB, `DMB ISH` | **0** | 889–2 763 |
| a64, SB, `STLR`/`LDAR` | **0** | 3 793–6 230 |
| a64, SB, neither | **0** | 4 376–6 463 |
| rv, MP, `fence w,w` / `fence r,r` | **0** | — |
| rv, MP, neither | **0** | — |
| a64, MP, `DMB ISH` | **0** | — |
| a64, MP, `STLR`/`LDAR` | **0** | — |
| a64, MP, neither | **0** | — |

Ranges over 31 runs — 25 sequential and two batches of six in parallel, 310
row-runs in all — every one of which passed. The collision witness (a plain
non-atomic increment both processors perform every round) lost 1 447–19 656 of
40 000 across every row and every run, and never zero, so no run in that set
was vacuous. Fifty to a hundred and thirty machine rounds per row-run, so the
scheduler's rendezvous falls inside the litmus loop hundreds of times rather
than around it.

**The unfenced arm is zero too, and that is the finding.** On x86-64 the same
is true of the host-thread rows, so it says nothing there; what is new is that
it stays true of the *machine* rows for a reason that will not go away on a
weakly ordered host. Between the guest's store and the guest's load sit a whole
interpreted instruction and the scheduler's per-access cycle accounting —
`tests/memory_model_costs.rs` puts the host's store-buffer window at about forty
nanoseconds, and one interpreted guest instruction is the same order. So the
machine-level store-buffer rows are a *check that the machine still executes
the barrier*, not a discriminator between having one and not. The AArch64 CI
job says so in its own comment rather than claiming the pair it cannot have.

The third column is the closest thing to a signal: with a barrier, far fewer
rounds saw *either* load stale — 111–1 698 against 1 050–8 365 on RISC-V — which
is the fence delaying the load until the other processor's store is visible.
The ranges nearly touch for the A64 acquire/release pair, so it is suggestive
rather than a discriminator, and nothing gates on it.

**MP was the shape that found something, and what it found was the torn load.**
Message-passing needs no tight rendezvous — the reader is already spinning when
the writer stores — so it is the row that could plausibly fail on a machine, and
on the first release run of twenty thousand rounds the fenced RISC-V row
reported one violation on an *x86-64* host, where store-store reordering cannot
happen at all. It was `RamStore`'s missing single-copy atomicity: the reader
spins on the flag while the writer stores it, and a four-byte load that mixes
the old word with the new returns a value larger than either. Eleven
occurrences in two million RISC-V rounds and thirteen in two million A64 ones
(25 invocations of the search, four runs of 20 000 apiece), and **every single
recorded pair was a byte-carry boundary** —

```text
flag=8959  payload=8704   (0x22ff under 0x2200)
flag=4095  payload=3840   (0x0fff under 0x0f00)
flag=19967 payload=19712  (0x4dff under 0x4d00)
```

— each the low byte of the previous round's value wearing the high bytes of
this one. Only one round in 256 crosses such a boundary, so the rate among
rounds that could tear is about 1 500 per million.

The rows that gate were made immune to it rather than being made to depend on
it: the reader compares the payload against its own round number, which the
ping-pong pins to the flag value in every untorn case and which a torn flag
cannot inflate. The sensitive comparison survives as
`a_torn_flag_load_is_visible_through_a_whole_machine`, `#[ignore]`d, which is
the first reproduction of that defect through a whole board rather than through
two hand-spawned threads over an `AddressSpace`.

## What does not work yet

Named, so that nothing here reads as finished.

* **Single-copy atomicity is not kept.** `RamStore` is a `Vec<AtomicU8>` and
  every access is a byte loop, so a naturally aligned four-byte load racing a
  four-byte store can return a mixture of the old and the new word — a value
  all three architectures forbid outright (*Intel SDM* vol. 3 §9.1.1, ARM DDI
  0487 B2.2.1, RISC-V Unprivileged ISA §1.4).
  `tests/smp_single_copy_atomicity.rs` catches it 117–361 times in sixty
  thousand loads, and
  `tests/memory_model_litmus.rs::machine::rv::a_torn_flag_load_is_visible_through_a_whole_machine`
  now catches it through a whole board — four times in 1.6 million rounds, with
  the torn values recorded. This is the largest known gap and it is reachable
  only in this mode.
* **`LDAR`/`STLR` on a64 still issue an ordinary load and store.**
  `core::space::BusLock` carries the argument for the shape of the fix.
* **No litmus run on a weak host has been *seen*.** The rows now exist and both
  legs execute them — the `aarch64 (weak memory)` job runs the whole of
  `tests/memory_model_litmus.rs`, the machine rows included, and
  `scripts/check.sh wasm-threads` runs the same file on
  `wasm32-wasip1-threads`. Every measurement above is from an x86-64 host, so
  what the gate is still waiting on is a green run of those two legs and the
  counts they print. Read the counts, not the tick: the machine-level
  store-buffer rows are expected to read zero in every arm on both hosts, for
  the sensitivity reason above.
* **The threaded-wasm leg carries only the A64 machine rows.**
  `scripts/check.sh`'s `WASM_THREADS_FEATURES` has `machine-a64-mini` and no
  RISC-V feature at all, so `machine::rv` compiles out there and
  `tests/riscv_amo_atomicity.rs` is already empty on that target for the same
  reason. The A64 half of the gate is covered in a threaded browser build; the
  RISC-V half is not, and closing it is one feature name.
* **The atomicity suites still spawn their own threads.**
  `tests/a64_lse_atomicity.rs`, `tests/riscv_amo_atomicity.rs` and
  `tests/smp_single_copy_atomicity.rs` construct two `Exec`s over one
  `AddressSpace` and run them on `std::thread`s. That is a sharper instrument —
  it pins both threads in a tight loop and maximises collisions — and it is
  worth keeping for that reason; but only `tests/parallel_smp_boards.rs` makes
  the claim about a whole machine, and it makes it for RISC-V `AMOADD` only.
  `tests/memory_model_litmus.rs`'s `machine` module is the *ordering* half of
  the same move; the atomicity half has not been made.
* **One oscillator, two processors** — see above. Every shipped `-smp` board is
  in this configuration.
* **No `parallel` board is in the frame-hash regression**, and cannot be: the
  goldens are state hashes.
