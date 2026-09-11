# rsemu against QEMU: the published benchmark suite

`ROADMAP.md` phase 8's gate is two claims, and until this page existed the tree
held neither:

> **Gate:** published benchmark suite; **within 2× of QEMU wall-clock** on the
> committed workload set, on the reference host (**black-box comparison only** —
> running it as a measuring instrument, never reading it, §1).

Every performance number this project had before this page was **self-referential**:
callgrind against our own previous baseline, and host instructions counted
against our own last count. That instrument is excellent at attributing a change
and incapable of answering *is this fast?* — and the difference is not
academic. A self-referential suite has twice let a figure into this tree that
priced the wrong thing: once a translation limit tuned against a cost that
turned out to live in the runtime rather than in the guest, and once a
frame-budget percentage quoted in two files for a design nobody had built. Both
are the same mistake, and neither is catchable by measuring harder against
yourself. An external reference is the fix. This is it.

**The headline, with the denominator named** — because a previous round had a
figure rejected for calling 1.27× "27 % faster" of the wrong baseline. On the
reference host below, rsemu's host JIT drives an `arm64-virt` Linux guest from
reset through a fixed workload in **about twenty times the wall-clock seconds
that `qemu-system-aarch64 -accel tcg` takes to drive the same guest through the
same work** (23.8× comparing each side's fastest run, 19.3× comparing medians),
and a `pc64` x86-64 guest in **about a hundred and ten times what
`qemu-system-x86_64 -accel tcg` takes** (109.9× and 110.8×).

The gate wants 2×. rsemu is an order of magnitude away on AArch64 and two orders
away on x86-64. That is the number the roadmap's gate needs, and an honest bad
one is worth more than a flattering one: the distance is now measurable rather
than assumed.

**And it is not the distance phase 8's original work list was aimed at**, which
is the second thing this measurement cost the roadmap. That list —
superblocks, cross-block guest-register allocation, tier-2 feedback
recompilation, memory-op fusion —
improves the code the JIT *generates*, and the three callgrind profiles taken
beside this measurement put that code at **7.32% of a RISC-V boot, 8.02% of an
AArch64 one and 8.22% of an x86-64 one**. Free code generation, on every core,
does not close a tenth of a 20× gap. `ROADMAP.md` phase 8 has been re-ordered
around that: the runtime items — x86's missing inlined memory path, the
deferred-charge replay, the scheduler's round boundary, and block length, which
is the only lever anyone has yet pulled — come first, and the code-generation
items are kept where their 8% puts them. *"What has to change for the gate to
be met"*, at the bottom of this page, is the short form.

## Provenance

QEMU is GPLv2 and its source is permanently off limits (`ROADMAP.md` §1,
`CLAUDE.md`). **Running** it is not, and §1 says so in the same paragraph that
forbids reading it: *"Using a GPL emulator as a measuring instrument is fine;
reading its source is not. Where this roadmap compares performance against QEMU
(§13), that is black-box benchmarking and nothing more."*

[`scripts/bench-vs-qemu.sh`](../../scripts/bench-vs-qemu.sh) starts a process,
reads what the guest prints on its serial line, and stops a clock. It reads
`--help` and a man page and nothing else. Nothing on this page or in that script
was informed by how QEMU achieves its numbers, and nothing may be — if a number
here makes you curious about the mechanism behind it, that curiosity is the
thing §1 forbids acting on. Measure our own side instead;
[`benches/a64_linux_boot.rs`](../../benches/a64_linux_boot.rs) already says where
rsemu's time goes.

## The reference host

Also recorded in [`../bench-host.md`](../bench-host.md), which is the register
these figures belong to.

| Field | Value |
| --- | --- |
| CPU | AMD Ryzen Threadripper 9970X, 32 cores / 64 threads, 1.22–5.49 GHz |
| Memory | 125 GiB |
| OS / kernel | Linux 6.18.41-gentoo-x86_64 |
| Rust | 1.98.0 (88d9e12ae 2026-08-18), the pinned toolchain |
| rsemu | 0.0.4, `--release`, features `cli,machine-arm64-virt,cpu-arm-a64-lift,jit,jit-x86,machine-pc64,cpu-x86-lift` |
| QEMU | 10.2.3, the distribution build, `-accel tcg` |
| CPU governor | `powersave`, `amd_pstate` active, boost enabled |
| Mitigations | as configured by the distribution; changing them changes the numbers |

**These figures were taken on a busy machine** — five other build jobs were
running, one-minute load average between 47 and 51 on 64 cores. That is stated
here rather than hidden because it is the single largest caveat on the page. It
does not invalidate the ratio: both sides were interleaved through the same
load, the reported figure is a minimum over repetitions, and the run-to-run
spread the harness prints is the evidence (see *the noise floor*, below). It
does mean the absolute seconds are pessimistic for everyone, and that a
re-measurement on a quiet host is the first thing to do when one is available.

## The committed workload set

Five workloads, on two guest architectures. A workload earns its place by
exercising something the others do not, and by being **reproducible, bounded and
dominated by guest execution** rather than by host I/O.

| Workload | What it is | What it measures | Why it earns its place |
| --- | --- | --- | --- |
| **boot** | reset to `/init`, on a stock Debian kernel | MMU bring-up, exception vectors, the whole initcall list — thousands of distinct basic blocks executed once each | The least synthetic workload there is, and the only one where *translation cost* dominates translated-code quality. A JIT that is brilliant on hot loops and slow to translate shows up here and nowhere else |
| **hash** | `sha256sum` of 8 MiB already in memory | a tight integer kernel: no branch worth predicting, working set inside L2 | As close to "how good is the generated code" as a real program gets. It is the phase where rsemu's ratio is worst, which makes it the most useful one |
| **awk** | 300 000 arithmetic iterations through busybox `awk` | an interpreter loop: branchy, pointer-chasing, re-entering a handful of blocks millions of times | Where block chaining and cross-block register allocation are worth something — two of the four items on phase 8's list |
| **gzip** | deflate over the busybox binary | table lookups and unaligned accesses at a rate the others do not reach | The phase that prices the software TLB, which is 18 % of the profile in `benches/a64_linux_boot.rs` |
| **total** | reset to the last marker | all of the above plus the shell that sequences them | The number the gate is actually about. Reported separately because a suite that only published per-phase figures could quietly drop the phase it did worst on |

Two guests run all five:

* **`arm64-virt`** against `qemu-system-aarch64 -machine virt`. The closer of the
  two comparisons by a distance: one core, a GICv2, a PL011, PSCI, virtio-MMIO,
  and `-cpu cortex-a53` is exactly what `machines/arm64-virt.machine`'s `part`
  parameter says. The boards are not byte-identical — rsemu's has two populated
  virtio-MMIO transports where QEMU's `virt` lays out empty ones — and **the
  guest cannot tell**: neither side's boot log mentions virtio at all, because
  the Debian kernel builds `virtio_mmio` as a module and this initramfs never
  loads it. That is a black-box observation from the two logs the harness
  keeps, which is the only kind available here and the right kind.
* **`pc64`** against `qemu-system-x86_64 -machine microvm`. `microvm` rather than
  `pc` because `pc64` has no PCI, no APIC and no ACPI, and neither does
  `microvm`; it is still not the same board, which is why this is the weaker leg.

### What is not in the set, and why

* **RISC-V.** `riscv-virt` is the furthest-along board in the tree and the
  natural third leg, and there is **no `qemu-system-riscv64` on the reference
  host**. The harness has no RISC-V case rather than a case that would silently
  skip; adding one is a few lines once the instrument is installed.
* **CoreMark and Dhrystone.** Both were considered and neither is here. They
  would need a cross toolchain per guest and a fixture per architecture, and
  what they would add over `hash` is a score to quote rather than a phase to
  attribute. The busybox already inside the fetched initramfs gives four
  workloads with different shapes for no new fixture at all — and the same
  binary runs under both emulators, which is the property that matters.
* **A frame-rate workload.** `ROADMAP.md`'s phase-3 gate has one and
  [`benches/frame_time.rs`](../../benches/frame_time.rs) is it. QEMU does not
  emulate a NES, so there is nothing to compare against; that gate is not this
  gate.
* **SMP.** One vCPU on both sides. Phase 8's SMP half is a correctness gate
  (litmus tests, `kvm-unit-tests` atomics), measured elsewhere.

## The method

### "The same work", without equating two timelines

This is the hard part of the comparison and the place a benchmark of this shape
usually goes wrong. rsemu is driven by `--for <virtual duration>`; QEMU without
`-icount` has no virtual timeline at all. Declaring that *n* rsemu virtual
seconds equal *m* QEMU seconds would be a modelling decision, and a wrong one
would silently become the answer.

So neither is used. **The guest says when it is done.** Both emulators get the
same kernel, the same initramfs, the same command line and a `/bench` init that
prints a marker line either side of each phase. The harness timestamps the
markers *on the host* as the bytes arrive and stops the process at the last one.
What is compared is host wall-clock seconds to drive one guest from reset to a
fixed point in its own execution — which needs no equivalence between the two
timelines and cannot be gamed by either side's idea of a second.

That choice turned out to be load-bearing rather than merely careful, and the
reason it did is worth keeping: **it found a defect.**

`--for` did **not** mean what a reader would assume, on either board. At
`--for 10s`, `rsemu run arm64-virt` reported its core as having taken
100 000 000 ticks of a clock the machine file rates at 1 GHz — a hundredth of
the elapsed span — and `rsemu run pc64` reported 201 600 000 of a 100 MHz one,
about a fifth. Every *other* clocked device in the same two summaries matched
its declared oscillator exactly (the PL011 at `uartclk / 16` = 1.5 MHz, the
8254 at 105/88 MHz, the MC146818 at 32 768 Hz, the 16550 at 115 200 Hz), so it
was specific to the processor objects and stable across spans. The guest felt
it: sixty seconds of `--for` on `arm64-virt` got a Linux kernel to a printk
timestamp of 0.3 s.

It was the domain genuinely running slow. `SchedulerConfig::max_ticks_per_
quantum` capped a scheduler round at **ten thousand** processor ticks whatever
the board declared, against a 1 ms quantum that owes a 1 GHz core a million,
and because a budget is recomputed from the tree's absolute position every
round, what a round could not spend became a backlog the next round re-capped.
A processor's effective rate was *ten thousand × rounds per guest second* and
the `osc` statement had no bearing on it.
[`../techniques/execution-budgets.md`](../techniques/execution-budgets.md) is
the whole argument and the fix; both boards' cores now advance at exactly their
declared rate.

Two things follow for this page. The ratios below are **unaffected**, which is
the point of measuring host wall-clock to a marker in the guest's own output: a
benchmark built on "n virtual seconds equals m real ones" would have silently
inherited the defect, and this one could not. And every `--for` on the rsemu
side of `scripts/bench-vs-qemu.sh` is a *bound* rather than a plan — it only
has to be more virtual time than the workload needs — so the bounds are now
generous by a factor of a hundred rather than tight, which costs nothing.

The guest also prints a digest of what each phase produced — the sha256 prefix,
the awk sum, the compressed size — and **a run whose digest disagrees with the
others is refused**. That is what makes it the same work rather than two
programs with the same name. Every figure below was taken from runs that agreed
on `2daeb1f36095b44b/899997/…`.

### Interleaving, minima, and the noise floor

`benches/a64_linux_boot.rs` already says what this host does to a stopwatch: it
drifts several nanoseconds per instruction under concurrent load and whole
seconds under a build. Three things follow, and the harness implements all three
rather than recommending them:

* Runs are **interleaved**, never A-then-B. One repetition runs every side once,
  and the side that goes first rotates, so a machine that gets slower over the
  afternoon charges each side equally.
* The reported figure is the **minimum** over repetitions — the run least
  interfered with — with the median and the max beside it.
* The sides are **never run concurrently**. Two emulators on one host measure
  the memory system, not each other.

**The noise floor is the spread column**, and it is the harness reporting on
itself: the same side, the same guest, the same work, repetitions apart. On this
host under load it is what the tables below say — and a ratio quoted without it
is not a measurement. Two consequences worth stating: a phase whose spread is
larger than the difference you are arguing about has not measured that
difference, and the reason no phase in this suite is sized to finish in under
about a second is that the marker latency and the noise floor would then be the
same size as the phase.

### The fairness decisions

**Compare against plain QEMU, `-icount` QEMU, or both? Both, and publish both.**
The argument:

rsemu always does per-access cycle accounting — it is how the bus charges time
(`ROADMAP.md` §4.2), it is what the cross-engine state hash is over (§0), and it
is not switchable. QEMU does that accounting only under `-icount`. Comparing
rsemu against plain QEMU therefore compares an emulator that is counting against
one that is not, and comparing it against `-icount` compares against a mode
almost nobody runs. Neither alone is the honest number, so both are published.

**What it costs is measured, and two numbers get confused here.** The *replay*
of deferred charges and boundaries (`flush_thunk`) is about a fifth of rsemu's
host instructions — 19.3 % of `benches/a64_linux_boot.rs`'s profile before the
store-ends-a-block change and 21.66 % after it — but that is the whole
bookkeeping row, not what the accuracy promise costs, because a translator with
no promise still keeps *some* budget. The number that is only about the promise
is the counterfactual in `src/ir`'s decision 2: a scratch build accounting per
**region** instead of per guest instruction ran the same boot for **11.9 %
fewer host instructions**. That is the figure to quote at this comparison, and
it is a lower bound — the per-access ticks charged inside an access are not
removed by it. So:

* **`qemu`** is what a person gets by typing `qemu-system-…`. It is the number
  that answers "how much slower is rsemu than the thing people actually use?",
  and it is the number the gate should be read against.
* **`qemu-icount`** (`-icount shift=0,sleep=off`) is the nearer thing to what
  rsemu is doing, and it is the number that answers "how much of the gap is the
  accounting?"

The rest, decided once and applied to every run:

* **KVM is off on both sides**, explicitly. TCG against rsemu's JIT is the
  comparison the gate is about; against KVM it would be a comparison of two host
  CPUs. (`tests/kvm_native_ratio.rs` is where the accelerated comparison lives.)
* **One vCPU on both sides.** rsemu's default threading is deterministic and
  single-threaded; QEMU gets `-smp 1`.
* **The same guest kernel, the same initramfs, the same command line.** The
  x86 leg passes `auto-kernel-cmdline=off` because `microvm` otherwise appends
  arguments of its own, and the two sides would stop having the same command
  line without anyone noticing.
* **The rsemu engine is `jit-host`**, the fastest of the three. `interp` is
  measured once below, for scale.

### Where it is still not fair

Stated here rather than left for a reader to find:

* **Idle costs plain QEMU and nobody else.** Under `-icount …,sleep=off` and
  under rsemu, a guest waiting on a timer costs no host time — the scheduler
  moves virtual time to the next event. Plain QEMU waits in real seconds. That
  flatters rsemu, it is confined almost entirely to **boot** (the other three
  phases are compute), and it is a second reason to read the per-phase table
  rather than only the total.
* **The x86 boards are not the same board.** `microvm` is the closest QEMU
  machine to `pc64` and it is not the same device set. The x86 ratio is the
  weaker of the two and should be read as an order of magnitude, not as three
  significant figures.
* **Marker latency.** A marker is timestamped when its byte reaches the harness,
  not when the guest stored it, so every phase boundary carries one UART's worth
  of latency on each side.
* **The host was busy.** See the reference-host section.
* **One QEMU build.** The distribution's, with whatever it was configured with.
  A comparison against a QEMU built for this host would be a different number,
  and probably a slightly worse one for us.

## The numbers

Taken with `scripts/bench-vs-qemu.sh` on the reference host, in one sitting,
under the load the reference-host section describes.

Every cell is **minimum / median** over the repetitions, in seconds, with the
run-to-run **spread** — `(max − min) / min` — in brackets. The ratio columns are
likewise minimum ÷ minimum, then median ÷ median. Read the spread first: it is
the harness reporting its own noise floor on this host, and it is large.

### `arm64-virt`, nine repetitions

`qemu-system-aarch64 -machine virt -accel tcg -cpu cortex-a53 -m 1G -smp 1`
against `rsemu run arm64-virt -p ram=1G -p engine=jit-host`, same kernel, same
initramfs, same `earlycon=pl011,0x9000000 console=ttyAMA0 rdinit=/bench`.

| phase | rsemu `jit-host` | `qemu` | `qemu -icount` | **rsemu ÷ qemu** | ÷ icount |
| --- | --- | --- | --- | --- | --- |
| boot | 22.85 / 25.32 (25 %) | 1.39 / 1.86 (99 %) | 1.44 / 1.86 (191 %) | **16.4× / 13.6×** | 15.9× / 13.6× |
| hash | 10.21 / 11.20 (74 %) | 0.09 / 0.10 (101 %) | 0.29 / 0.48 (126 %) | **112.2× / 115.5×** | 35.1× / 23.5× |
| awk | 24.68 / 27.30 (39 %) | 1.04 / 1.32 (89 %) | 1.43 / 2.11 (149 %) | **23.7× / 20.6×** | 17.3× / 12.9× |
| gzip | 5.50 / 5.95 (74 %) | 0.34 / 0.37 (81 %) | 0.46 / 0.60 (150 %) | **16.1× / 15.9×** | 12.0× / 9.9× |
| **total** | 69.11 / 72.41 (24 %) | 2.90 / 3.74 (76 %) | 3.88 / 5.23 (96 %) | **23.8× / 19.3×** | 17.8× / 13.8× |

### `pc64`, three repetitions

`qemu-system-x86_64 -machine microvm,acpi=off,pit=on,pic=on,rtc=on,isa-serial=on
-accel tcg -cpu qemu64 -m 257M -smp 1` against `rsemu run pc64
-p engine=jit-host`. Three repetitions rather than nine because one rsemu
repetition of this leg is about eight minutes.

| phase | rsemu `jit-host` | `qemu` | `qemu -icount` | **rsemu ÷ qemu** | ÷ icount |
| --- | --- | --- | --- | --- | --- |
| boot | 122 / 128 (8 %) | 2.02 / 2.10 (35 %) | 9.61 / 11.50 (21 %) | **60.6× / 60.8×** | 12.7× / 11.1× |
| hash | 43.74 / 43.84 (4 %) | 0.10 / 0.10 (60 %) | 0.23 / 0.24 (44 %) | **455.6× / 429.8×** | 187.7× / 182.7× |
| awk | 225 / 261 (16 %) | 1.55 / 1.81 (51 %) | 4.03 / 4.22 (17 %) | **145.3× / 144.6×** | 55.9× / 61.9× |
| gzip | 37.88 / 45.83 (28 %) | 0.22 / 0.27 (69 %) | 0.29 / 0.32 (13 %) | **172.2× / 169.7×** | 132.5× / 144.6× |
| **total** | 439 / 475 (11 %) | 4.00 / 4.28 (39 %) | 14.24 / 16.41 (19 %) | **109.9× / 110.8×** | 30.8× / 28.9× |

### The same arm64 workload on the interpreter, one repetition

Not part of the gate — the interpreter is the oracle, not the product. It is
here because it is the only number on the page that says what the host JIT is
worth measured through this same instrument, and because a suite that can only
measure one configuration cannot be trusted about that one. One repetition, so
minimum and median are the same run and the spread column is structurally zero
— it is a scale marker, not a measurement with an error bar.

| phase | rsemu `interp` | `qemu` | `qemu -icount` | **rsemu ÷ qemu** | ÷ icount |
| --- | --- | --- | --- | --- | --- |
| boot | 145 / 145 (0 %) | 2.90 / 2.90 (0 %) | 2.10 / 2.10 (0 %) | **50.1× / 50.1×** | 69.3× / 69.3× |
| hash | 112 / 112 (0 %) | 0.18 / 0.18 (0 %) | 0.36 / 0.36 (0 %) | **621.6× / 621.6×** | 309.1× / 309.1× |
| awk | 187 / 187 (0 %) | 2.04 / 2.04 (0 %) | 1.49 / 1.49 (0 %) | **91.7× / 91.7×** | 125.8× / 125.8× |
| gzip | 68.14 / 68.14 (0 %) | 0.68 / 0.68 (0 %) | 0.55 / 0.55 (0 %) | **100.6× / 100.6×** | 124.6× / 124.6× |
| **total** | 526 / 526 (0 %) | 5.87 / 5.87 (0 %) | 4.57 / 4.57 (0 %) | **89.7× / 89.7×** | 115.1× / 115.1× |

## Reading it

**The gate is 2×. The measurement is 19–24× on AArch64 and about 110× on
x86-64.** Six things the tables say that the headline does not.

**1. The two statistics disagree on AArch64 and agree on x86-64, and that is
the noise talking.** `arm64-virt`'s total is 23.8× on minima and 19.3× on
medians; `pc64`'s is 109.9× and 110.8×. The arm64 runs are seconds long on the
QEMU side, where a loaded host moves a four-second run by most of its own
length; the x86 runs are minutes on one side and seconds on the other, and the
ratio is so large that the noise cannot reach it. **Quote `arm64-virt` as
"about twenty times" and `pc64` as "about a hundred and ten times"**, and do not
put a second significant figure on the first of those.

**2. The noise floor is worse for the faster side, precisely because it is
faster.** rsemu's spreads here run 4–28 % on totals; QEMU's run 35–99 %, and
one `qemu-icount` boot spread reached 191 %. That is not QEMU being erratic —
it is a four-second measurement on a machine with fifty other things running,
against a seventy-second one that averages its own interference out. The
consequence is a real limitation of this suite and the first thing to fix:
**size the workload so the fast side also takes tens of seconds.** It was not
done here because the factor of four to eight it would take turns the x86 leg
from the twenty-five minutes it costs today into two or three hours, and a
benchmark nobody runs measures nothing.

What that noise is worth, concretely: a **single-repetition** run of the
`arm64-virt` leg taken minutes after the nine-repetition one above reported
**13.65×** total, because its one QEMU sample happened to be a slow one
(5.69 s against the nine-run minimum of 2.91 s). One repetition of this suite
can flatter rsemu by most of a factor of two. `--reps 1` is for checking that
the harness runs, not for quoting.

**3. `hash` is the extreme and the least reliable cell on the page.** 112× on
AArch64 and 456× on x86-64 — and QEMU does that phase in about 0.1 s, which is
inside its own spread. The direction is not in doubt (even against QEMU's
*worst* run of it, rsemu is more than fifty times slower on AArch64) but the
figure is not a measurement of three digits. It is also the phase whose shape
most directly prices generated code: one tight integer loop, no MMU pressure,
no branch worth predicting. That it is the worst phase is the finding; its
exact value is not.

**4. `-icount` explains most of the x86 gap and almost none of the AArch64
one.** Turning on QEMU's per-instruction accounting costs it 3.6× on `pc64`
(4.00 s → 14.24 s) and *nothing* on `arm64-virt` (2.90 s → 3.88 s on minima,
and the icount side is sometimes the faster of the two because `sleep=off` skips
the boot's idle). So the honest reading of "how much of the gap is the
accounting rsemu cannot switch off?" is **architecture-dependent**: on x86-64 it
is most of the difference between 110× and 31×, and on AArch64 it is a few per
cent. Anyone tempted to attribute rsemu's distance to cycle accounting should
look at the AArch64 column first.

**5. The host JIT is worth about seven and a half times, measured here.** The
same arm64 workload on `engine=interp` totals 526 s against `jit-host`'s 69.1 s
best and 72.4 s median — **7.6× and 7.3×**. The roadmap's own figure for the
same pair, taken a different way over twenty seconds of guest time, is 5.61×.
Two independent methods landing within thirty per cent of each other, on a
workload that is mostly not boot, is the kind of cross-check this page exists to
make possible; the gap between them is most of what "twenty seconds of boot" and
"a boot plus three compute kernels" measure differently.

**6. rsemu's AArch64 *interpreter* is 89.7× its QEMU; rsemu's x86-64 *host
JIT* is 109.9× its QEMU.** Those are two different guests against two different
QEMU binaries, so it is not a like-for-like ratio and must not be quoted as one.
It is still the sharpest thing in the tables, because the comparison that
matters is against each side's own baseline: on AArch64 the JIT moves rsemu from
89.7× to 23.8×, and on x86-64 the JIT lands at 109.9× — further from its own
QEMU than the AArch64 *interpreter* is from that one. Phase 8 should treat the
x86 core as a different problem from the A64 one rather than as the same problem
further behind.

## Running it

```sh
scripts/fetch-testdata.sh arm64-linux arm64-initramfs x86-linux initramfs-x86
scripts/bench-vs-qemu.sh                        # both guests, 5 repetitions
scripts/bench-vs-qemu.sh --guest arm64 --reps 3
scripts/bench-vs-qemu.sh --engine interp        # the oracle, for scale
scripts/bench-vs-qemu.sh --sides rsemu,qemu     # skip the icount leg
```

or as a `check.sh` stage, which is the same thing with the house's skip
behaviour:

```sh
scripts/check.sh qemu                                  # both guests, 5 reps
RSEMU_BENCH_GUEST=arm64 RSEMU_BENCH_REPS=3 scripts/check.sh qemu
```

`RSEMU_BENCH_GUEST=arm64` exists because the two legs cost wildly different
amounts — the arm64 one is minutes, the x86 one is an hour — and a knob that
makes the cheap half runnable is the difference between a stage that gets used
and a stage that gets skipped.

**Do not edit `scripts/bench-vs-qemu.sh` while a run of it is in progress.**
bash reads a script lazily by byte offset, so inserting a line near the top
while the run loop near the bottom is executing moves every later offset and the
interpreter lands mid-statement at the next seek. It cost this page one complete
five-repetition arm64 leg: every sample was collected and the report that would
have aggregated them died with a syntax error on a line that was not wrong. If a
run dies that way, check the file's mtime before the line it names.

It is **not** in `check.sh`'s default set and not in `--all`: one repetition of
the arm64 leg is a minute and a half and the x86 leg is far more, so it is a
stage somebody runs on purpose. `cargo test` never runs it and never needs a
fixture for it.

Without a `qemu-system` on `PATH`, or without a fetched kernel, it **skips
loudly** and exits 0 — the same contract `crosshost` and `long` have.
`RSEMU_BENCH_REQUIRED=1` turns those skips into failures, for a runner that
installed both on purpose.

**The stage never fails the build.** It prints the ratio and exits 0 either way,
because the gate is a judgement about a number and a script that failed at 2.01×
would be asserting a precision nobody has. The gate is held by this page being
re-measured and re-read, not by an exit status.

## What has to change for the gate to be met

The gate is 2×. `ROADMAP.md` phase 8 is the ordered list and this is the short
form of it, from the tables above and from the three boot profiles taken beside
them — [`benches/a64_linux_boot.rs`](../../benches/a64_linux_boot.rs), which
measured **350 host instructions per guest instruction** before the
block-length work and 287 after it, of which the code the JIT generated was
twenty-five; [`benches/riscv_linux_boot.rs`](../../benches/riscv_linux_boot.rs)
at 310 and then 219; and
[`benches/x86_linux_boot.rs`](../../benches/x86_linux_boot.rs) at 652 and then
508. Each of those is one workload over one span, and each says which — twenty
guest seconds of boot on A64, twenty on RISC-V that reach `ftrace: allocating`,
and a hundred and twenty on x86 that are the kernel's own self-decompressor,
because a nine-hundred-second boot does not finish under callgrind.

1. **Not the generated code.** It is **7.32%** of the RISC-V boot, **8.02%** of
   the AArch64 one and **8.22%** of the x86-64 one. The other ninety-two are
   the machinery around it, and no amount of better code generation moves a
   ratio whose numerator is mostly not code generation. Phase 8's original list
   was entirely code generation; it has been re-ordered behind the runtime
   items for exactly this reason.
2. **x86 publishes no inlined memory path, and that is the largest single item
   on any core**: 22.34% of `pc64`'s profile and 146 host instructions per guest
   instruction go through the address space, because `cpu::x86::engine`'s
   `FastMem` is empty and every access takes the call. A64 serves most of its
   accesses from an inlined probe. This is most of why the x86 leg of the tables
   above is five times the AArch64 one.
3. **The deferred-charge replay, which is the largest row left on two of three
   cores** — and of which 11.9% of a boot is the accuracy promise §0 requires,
   measured against a per-region counterfactual rather than estimated. A sixth
   of that is implementation and is recoverable; the rest is the promise.
4. **Block length.** 6.44 guest instructions per block on A64 before the
   store-ends-a-block change and 10.80 after, 5.51 → 15.91 on RISC-V, 5.21 →
   12.17 on x86, worth −14.6%, −29.3% and −22.2% of the host instructions.
   Every per-block cost is divided by that number, and superblocks are the item
   on phase 8's list that attacks it directly.
5. **The x86 core is a different problem from the A64 one.** 110× against 20×
   is not a tuning gap — and the sharper form of it is that rsemu's *AArch64
   interpreter* is 90× QEMU while rsemu's *x86-64 host JIT* is 110×. Phase 8
   should not treat the two cores as the same problem at different depths.
6. **A quieter host, and a longer workload.** The two limitations of this
   measurement are in the tables: it was taken under a load average near fifty,
   and the QEMU side of `arm64-virt` runs for four seconds where its own
   run-to-run spread is most of a second. Both are fixable and neither changes
   an order of magnitude.

And what none of it measured, because the profiles are boots and this suite's
worst phases are not: **there is no profile of `hash`, `awk` or `gzip` on any
core.** The attribution above says where a *boot* spends its host instructions;
`hash` is 112× on AArch64 and 456× on x86-64 and nobody has yet taken a
callgrind profile of it. Anyone looking for the next item for phase 8 should
take that profile rather than reason from these.

Whoever closes any of this: re-run this page's command, replace the tables, and
say in the commit which workload moved. A ratio that changed without a named
workload is the self-referential number this page exists to replace.
