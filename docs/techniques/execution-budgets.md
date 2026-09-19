# Execution budgets: what a scheduler round hands a processor

Consumed by: `core/sched` (both rounds), every CPU core's `Runnable::run`,
`machines/*.machine` (whose `osc` statements this decides the meaning of),
`tests/engine_longrun.rs` and `scripts/check.sh`'s `long` stage (whose budgets
are written in guest time), `benches/*_linux_boot.rs`. Companion to
[`parallel-execution.md`](parallel-execution.md), which is about what happens
when two runnables share a round; this file is about how much a round hands out
in the first place.

`ROADMAP.md` §4.2 states the contract in one sentence: *"a CPU is never
'stepped one instruction' by the scheduler; it is handed a budget ('run until
virtual time T or 10 000 ticks, whichever first') and reports back how much it
consumed."* This file is about the second half of that parenthesis, which was
a literal constant in the code for a long time and should not have been.

## How a budget is derived

A round has a **target**, and the target is a pure function of virtual time and
machine state: the next point on the quantum grid, or the next queued event, or
the next event a lazily-advanced device has of its own, whichever is soonest
(`Scheduler::natural_target`). Everything else follows from it.

For each runnable, in turn:

1. **`Scheduler::ticks_until`** asks how many ticks of the runnable's own
   clock domain fit between where it actually stands and the target. *Actually stands*, not where the target's own conversion says it
   should: a runnable stops on a tick boundary (§4.2, "stop at the cycle
   boundary before"), so a tree lags the exact conversion by whatever fraction
   of a tick the last round could not spend, and a round that was declined
   outright leaves a whole round's worth behind. Measuring from the real
   position is what hands those back.
2. **`SchedulerConfig::max_ticks_per_quantum`**, if a caller set one, lowers it.
   It is `None` by default.

The runnable reports what it consumed and its domain moves on by that. A
runnable on a crystal another runnable shares measures from its **own**
position, not the tree's — see *[Two runnables on one
crystal](#two-runnables-on-one-crystal)*.

Two properties fall out, and both are load-bearing:

* **A budget is recomputed from the tree's absolute position every round**, so
  the rounding in any cross-tree step is bounded by one tick and cannot
  accumulate.
* **Ticks a round could not spend are handed out by the round that ends up
  owning them.** `Scheduler::decline_round` depends on this — it is what makes
  `Machine::run_for` additive (§11.6), and `tests/run_for_additive.rs` is the
  board-level statement of it.

## The constant that used to be step 2, and what it cost

`max_ticks_per_quantum` was **10 000**, against a `DEFAULT_QUANTUM` of **1 ms**.

A 1 GHz core is owed a million ticks by a millisecond. It got ten thousand —
one percent — and because a budget is recomputed from the tree's absolute
position, the 990 000 it did not get became a backlog that the next round
re-capped. The core's effective rate was therefore
*`max_ticks_per_quantum` × rounds per virtual second* and **the board's `osc`
statement had no bearing on it at all**.

That is measurable from outside, and it was measured three times independently
before anyone changed it. On the tree as it stood:

```console
$ rsemu run arm64-virt --media kernel=… --initrd … --for 10s --headless --trace clock
ran to 10000000000 ns of virtual time
  cpu      100000000 ticks      # a 1 GHz oscillator owes ten seconds 10 000 000 000
  uart     15000000 ticks       # 24 MHz / 16, for ten seconds: exact
```

| board | core | ticks in 10 guest seconds | share of what the machine file declares |
| --- | --- | --- | --- |
| `arm64-virt` | 1 GHz | 100 000 000 | 1/100 |
| `riscv-virt` | 1 GHz | 100 000 000 | 1/100 |
| `pc64` | 100 MHz | 201 600 000 | 1/4.96 |

Every *other* clocked device in those summaries was exact — the PL011 at
1.5 MHz, the CLINT's `mtime` at 10 MHz, the 8254, the MC146818, the 16550 —
because nothing but a `Runnable` is bounded by a budget. So the defect was not
"the machine runs slow"; it was "**the processor runs slow against every clock
on its own board**", which a guest feels directly: sixty seconds of `--for` on
`arm64-virt` reached a printk timestamp of 0.3 s.

The same command after the change, and it is the shortest statement of what
this was worth:

```console
$ rsemu run arm64-virt --media kernel=… --initrd … --for 1s --capture console
[    0.689553] evm: HMAC attrs: 0x1
ran to 1000000000 ns of virtual time
  cpu      1000000000 ticks
  uart     1500000 ticks
```

One second of `--for`, one billion ticks of a 1 GHz core, and a kernel whose own
printk clock reads 0.69 s — the rest being the idle the boot actually spends.
Sixty seconds used to reach 0.3.

`pc64`'s 1/4.96 rather than 1/10 is the arithmetic working exactly as
described. Its rounds are cut short by its lazily-advanced devices, and a round
cut short is a round that still hands out up to the full cap — so *more* rounds
per guest second meant *more* ticks per guest second. The cap made a board's
processor speed a function of how many devices interrupted its scheduler.

### What it cost in host instructions

Round-boundary work is real, so the cap also meant paying for roughly a hundred
times more scheduler rounds per unit of guest work than necessary. The size of
that was measured rather than assumed, under callgrind, with **guest work held
constant**: the cap was varied and the virtual span divided by the same factor,
so every run in a column retires the same guest instructions to the digit
(15 548 846 on `arm64-virt`, 11 464 564 on `riscv-virt`).

```console
$ valgrind --tool=callgrind --smc-check=all-non-file --cache-sim=no \
      target/release/deps/a64_linux_boot-… --nanos $((2000000000 * 10000 / CAP)) --cap $CAP --reps 1
```

`--smc-check=all-non-file` is not optional: the generated code is an anonymous
mapping this process writes and then executes.

| cap | `a64_linux_boot` | vs 10 000 | `riscv_linux_boot` | vs 10 000 |
| --- | --- | --- | --- | --- |
| 10 000 | 5 123 812 894 | — | 3 477 740 409 | — |
| 20 000 | 5 084 830 486 | −0.76% | 3 414 757 576 | −1.81% |
| 50 000 | 5 038 555 330 | −1.66% | 3 365 277 255 | −3.23% |
| 100 000 | 5 027 442 153 | −1.88% | 3 339 249 126 | −3.98% |
| 200 000 | 5 013 993 979 | −2.15% | 3 307 628 887 | −4.89% |
| 1 000 000 | 5 003 904 798 | **−2.34%** | 3 290 609 674 | **−5.38%** |

A million is where the curve stops: a 1 GHz core on a 1 ms quantum cannot use
more than a million ticks a round, so `--cap 1000000` and no cap at all produce
the identical state hash and the identical instruction count.

Net of `RamStore::fill` — 1 481 880 746 and 833 552 748 host instructions,
identical in every run, zeroing the board's guest RAM at reset — the emulation
itself falls by **3.29%** and **7.08%**. Per round: 1 980 fewer rounds saved
119 908 096 and 187 130 735 host instructions, so **a scheduler round costs
about 61 000 host instructions on `arm64-virt` and 95 000 on `riscv-virt`**.

**That curve is a result that kills a hypothesis.** Three separate profiles had
found the dispatch loop dominating a boot — 29.5% of a RISC-V one, 18.6% of an
a64 one — and round boundaries were the suspected multiplier. They are not: a
round boundary is 2-7% of a boot, because there are three orders of magnitude
more *block* entries than *round* entries (1.5 million against 2 000 in the
window above). The per-block costs are where that 18.6% lives and the
block-length work in `cpu::arm::a64::lift` is what moves it.

What did come out of the profile is one row worth chasing on its own:
`EventQueue::advance_to` was 43 043 776 host instructions — 1.24% of the whole
`riscv-virt` run and 23% of everything the cap's removal saved — at 2 000
rounds, which is **21 500 host instructions per round**. The timing wheel
sweeps `WHEEL_LEVELS × WHEEL_SLOTS` cells on any round long enough to cross a
level, whether or not a single event is queued. Removing the cap amortised it a
hundredfold and left it standing.

## Two runnables on one crystal

The cap was doing a second job, and that one is real: **making sure the
second runnable on a crystal is scheduled at all.** A tree has one unit
counter, and the round used to measure every budget from it: offer the first
of two runnables the whole span, advance the counter by what it consumed, and
the second finds nothing left. All four shipped `-smp` boards are in that
configuration; on a 100 MHz board the cap of 10 000 against a 100 000-tick
round was what left the second processor anything.

### The share, and why it was wrong

`Scheduler::tree_shares` replaced the cap with *the span the tree has left,
divided by however many runnables still have a turn in this round*: the first
of two got half, the second the other half, and the counter moved once. Both
processors ran with no cap at all — but each at **half** the rate its board
declares, and a board with three runnables on a crystal at a third. The
premise was that two runnables on one crystal spend one span between them.
The hardware does not work that way: a crystal clocks every chip on it at the
same time, so a 68000 on an A500 retires `clk / 4` cycles a second whatever
else is on the crystal, and so does each of two CPUs sharing one oscillator.
Measured over 200 ms of virtual time before the change, as cycles each
runnable *consumed* against what its clock owes:

| board | runnables on one crystal | each executed |
| --- | --- | --- |
| `amiga-a500` | 68000, Paula (a serial poll) | 50.2 %, 49.8 % |
| `arm64-virt-smp`, `riscv-virt-smp`, `pc-at-smp`, `pc-apic`, `q35-linux-smp` | two CPUs | 50.0 % each |
| `stm32f407` | Cortex-M4, two DMA controllers | 33.3 % each |

Every other board in `machines/` has at most one runnable per crystal and ran
at 100 %.

### Each runnable has a position of its own

`Scheduler::advance_runnable` is what replaced the share. Every runnable is
offered the whole span from **its own position** to the round's target, and
moves on by what it consumed *without moving anyone else*
(`ClockForest::advance_alone`). The tree's counter then stands at the
**slowest** running runnable on it (`ClockForest::advance_tree_to`), and a
runnable that is ahead keeps its lead in its own domain — `max(tree, own)`, in
units, so no phase inside a tick is lost — until the tree catches up.

The counter standing at the slowest is the invariant that makes this correct
rather than merely fast. The counter is what every lazily-advanced device on
the crystal is published at, so no device is ever published ahead of a
processor that has yet to reach it. And during a round, a runnable on a shared
crystal arms **no** live view, and nothing arms a device that sits on one
(`Scheduler::arm_live_cursors`): two processors that each execute the round
from its start, one after the other, would otherwise let the second read a
device the first had already carried past it. That is the rule the parallel
round has always applied to a tree driven by more than one runnable, so the
two modes agree on it. What it costs is resolution inside a round for the
devices those processors reach — `riscv-virt-smp`'s CLINT is caught up at
round boundaries and at its own events rather than at each hart's cycle.

It does **not** cost resolution in a register that is a pure function of time.
Catch-up is what the rule restricts, because catch-up moves a device; reading a
free-running counter moves nothing, and the answer at a reader's own position
depends on that reader alone. So every runnable also carries a *read view* for
the length of a `run` call — the exact tick its own position corresponds to in
each such device's domain (`Scheduler::read_view`, `TickCursor::tick_in`,
`LazyHandle::reader_tick`) — and `mtime`, the `time` CSR that shadows it, and
anything else of that shape are read there. Comparators and interrupts are
untouched: they still fire at the device's own events, where the scheduler
delivers them.

That was not an optimisation. Without it every printk timestamp Linux wrote on
`riscv-virt-smp` was a whole millisecond, the length of a round, because a
guest's clock could not move inside one; a `udelay` spun to the next round and
anything that measured an interval between two nearby reads measured zero.

**Which boards that reaches, and which were never affected.** A guest's clock
is the board's, so the answer is per architecture rather than per scheduler
rule:

| board | what a guest reads time from | inside a round |
| --- | --- | --- |
| `riscv-virt`, `riscv-virt-smp` | the `time` CSR and `MTIME`, both the CLINT's `mtime` | now the reader's own position, on both |
| `arm64-virt`, `arm64-virt-smp` | `CNTPCT_EL0`/`CNTVCT_EL0` | never affected: the generic timer is the core's own cycle counter divided by an integer, per processor, inside the core |
| `pc-at-smp`, `pc-apic`, `q35-linux-smp` | the TSC | never affected: `State::cycles`, per processor |
| every x86 board | the HPET, the ACPI PM timer, the 8254 and the local APIC's current count | now the reader's own position, on one processor and on two |

The last row was a *different* defect with a different cause, and it is worth
keeping separate: `cpu.x86` published no `TickCursor` position at all, so those
devices were caught up to a round boundary on a one-processor board exactly as
on a two-processor one. A guest latching the 8254 in a loop read the same pair
of counts and then a jump of up to 1 193 — one millisecond — on `q35` and
`pc-apic` alike, and the HPET, the PM timer and the APIC's current count stood
still for a round and jumped the same way
(`tests/x86_counter_resolution.rs`). It took two pieces, one per cause:

* **The core publishes its position** (`cpu::x86::Exec::publish_position`),
  at the four bus entry points every engine reaches, so a device reached from
  an access is caught up to the cycle the access is on. That is what fixes a
  one-processor board, through ordinary catch-up.
* **The four devices answer a read at the reader's position**, through the read
  view, because on a crystal two processors share catch-up stops at the round
  boundary by design — the same rule that made `riscv-virt-smp`'s CLINT need
  one.

The number the core publishes is **not** its cycle counter, and that is the
part most likely to be got wrong next time. x86's only cycle counter is the
TSC, which a guest overwrites with `WRMSR`, a hypervisor hand-over replaces,
and a `HLT` stops while the forest goes on counting the budget it consumed. A
live position is not capped at the round, so publishing the TSC would let one
`WRMSR` carry every device on the board arbitrarily far into the future. The
scheduler therefore records where each runnable's domain stands before every
`run` call (`TickCursor::anchor`), and the core publishes that plus the debt it
carries plus what it has charged since — the scheduler's own accounting,
re-anchored every call, which no guest instruction can reach
(`cpu::x86::exec::Position`).

Moving the published position moved something besides reads, and on purpose:
on a one-processor board **a write** now lands at the writer's cycle too, so a
timer armed mid-round starts counting where the instruction that armed it ran.
Before, it started where the round began and fired up to a round early:
measured from the arming instruction to the handler on `q35`, for a
half-millisecond alarm (12 500 cycles), APIC 3 615 → 12 646, HPET 3 450 →
12 646, 8254 3 532 → 12 662. On a crystal two processors share, nothing is
caught up inside a round and a write still lands at the round's start —
unchanged, and still up to a round early (`pc-apic`: 3 659, 3 494 and 3 565
cycles, before and after). Answering the *arming* write at the writer's
position there is the remaining piece; for the HPET and the 8254 it moves a
comparator's origin, which a read never does, and it wants a change of its own.

What a Linux guest made of it. **Every figure names its kernel and how the
machine was driven**, because both change the answer and a figure without them
is not a figure: `rsemu run … --for 150s` is one `run_for` call, and
`tests/x86boot` — which is what `cargo test` runs — drives the same board in
one-millisecond calls. `before` is the base of this work with neither change;
`after` is both. Each board's core is declared at 100 MHz, so 100.000 is the
right answer everywhere.

| board | kernel | driven | before | after |
| --- | --- | --- | --- | --- |
| `pc64` | Debian installer (`testdata/x86/bzImage`) | `tests/pc64_linux.rs`, 1 ms calls | `tsc: Detected 99.470 MHz`, `lpj=397880` | **`100.002 MHz`**, `lpj=400008` |
| `pc64` | Gentoo `6.6.67` | `tests/pc64_linux.rs`, 1 ms calls | `97.530 MHz`, `lpj=325100` | **`100.004 MHz`**, `lpj=333346` |
| `pc64` | Debian installer | `rsemu run`, one call | `96.868 MHz`, `lpj=387472` | **`99.990 MHz`**, `lpj=399960` |
| `pc64` | Gentoo `6.6.67` | `rsemu run`, one call | `96.780 MHz`, `lpj=322600` | **`100.004 MHz`**, `lpj=333346` |
| `q35-linux` | Gentoo `6.6.67` | `rsemu run`, one call | `99.641 MHz`, `lpj=332136`, refined to `99.952 MHz`, then *"Marking TSC unstable due to clocksource watchdog"* — skewed −691 803 ns over the HPET's 480 ms — and `Switched to clocksource hpet` | **`100.004 MHz`**, `lpj=333346`, refined to `99.999 MHz`, and the TSC stays the clocksource |
| `q35-linux-smp` | Gentoo `6.6.67` | `rsemu run`, one call | `97.915 MHz`, `lpj=326383` | **`99.999 MHz`**, `lpj=333330` |

Two things in that table are worth naming rather than leaving to the reader.

**The Gentoo kernel now reads the same under both driving patterns** —
`100.004 MHz`, `lpj=333346`, to the digit — where before it read `97.530`
sliced and `96.780` whole. That is the additivity §11.6 claims, arriving at the
guest: how a caller cut its run had been worth 0.8% of the guest's idea of its
own clock.

**Each change alone was not enough, and one alone made a configuration worse.**
Publishing the core's position without *"a declined round ages nobody"* left
the sliced cells at `90.003` and `111.649` MHz — worse than the base, because
live reads inside a round expose a passive crystal that had aged through the
fragments at every call boundary, where the old round-grained reads had
averaged it away. That is why the two land as separate commits with the
scheduler one first.

The watchdog line in the `q35-linux` row is the read defect seen from inside: a
TSC that counts every cycle, checked against an HPET that moved once per round,
disagrees, and the kernel believes the HPET. `q35-linux-smp` still marks
`tsc-early` unstable at 7.5 s, before and after alike, skewed by about 100 ms
in 508 ms — which is not a round's worth of anything, and is the open defect
below rather than this one.

## Open: a halted processor's time-stamp counter stops

`X86::run_budget` consumes its whole budget when `HLT` has stopped the core —
it must, or the scheduler never reaches the timer that would wake it — but it
charges no cycles. `State::cycles` is what `RDTSC` reads, so a guest's TSC
stands still while the board's clocks go on, and the *Intel SDM* volume 3B
§17.17.1 says the opposite: an invariant TSC "will run at a constant rate in
all ACPI P-, C-. and T-states".

Measured on `pc-apic`, a guest halting twenty times and waking on the 8254:
the HPET moved 199 744 ticks — 499 360 cycles of that board's 25 MHz core —
while the guest's time-stamp counter moved **2 926**, under one per cent of
them (`an_idle_processors_time_stamp_counter_stops_which_is_an_open_defect` in
`tests/x86_counter_resolution.rs`, which asserts today's wrong answer as a
ledger entry so that fixing it trips the test).

It is plausibly behind the `tsc-early` watchdog skew on `q35-linux-smp` —
same direction, same order — but that is **unproven**: charging the TSC
through `HLT` as an experiment left that boot's console byte-identical, so
whatever the kernel measured there did not include a halt. Neither change in
this work caused it or cured it.

The cost, measured under callgrind on `publishing_cost_workload` in
`tests/x86_counter_resolution.rs` — an interpreted load/add/store loop on `q35`
that reads no clock, so both builds execute the same guest instructions to the
same state hash, `0xad8ee3eed4080384` — is 294 587 629 → 297 447 645 host
instructions for the scheduled run, **+0.97%**: +0.72% in the core, publishing
at the loop's two data accesses and advancing the origin once per step, and
+0.26% in the scheduler, which now builds read views because four devices ask
for them. Instruction fetches do not publish (`Exec::fetch_read`); the first
cut did, and cost +6.4%.

*"A declined round ages nobody"* costs nothing measurable on the same
workload — 294 589 389 host instructions at the base against 294 587 629 with
it, which is noise — because it removes work rather than adding any: a
declined round now converts no trees at all.

Three things follow:

* **Every runnable executes the rate its board declares**, on every board,
  in both modes; the table above reads 100 % in every row.
* **A tree with one runnable is unchanged to the bit.** The runnable moves,
  the tree follows it to the same unit, and every committed frame hash of a
  uniprocessor board is identical.
* **A slow runnable on a fast crystal needs no floor.** The share needed one:
  an 8042 at 1 193 Hz beside an 8086 (`tests/vnc_input.rs`) was offered half
  of the 1.19 ticks a 1 ms round owes it, which is zero, and a one-tick floor
  was the repair. Offered its own clock's whole span, it gets every tick in
  the round that owes it.

A lead survives a snapshot: `save_clocks` writes the domain's tick count as
always and, only when some domain has one, a trailing section with the lead in
units. A machine with at most one runnable per crystal never has one, so its
bytes — and every committed state hash — are what they were.

### What the counter still cannot say

A runnable that *stops consuming* holds its crystal back, since the tree
stands at the slowest runnable on it — exactly as a tree with one runnable has
always stood wherever that runnable stopped. Every core here consumes its whole
budget when halted, waiting for an interrupt or switched off, so what this
takes is a core that returns nothing for ever.

## The job the cap was never doing

It looks like a latency bound — "how late can an interrupt or an exit flag be
noticed" — and it is not, in either direction:

* **An exit.** Every core tests its `ExitFlag` at each block boundary
  (`ROADMAP.md` §4.7's safe-point protocol) and `jit::dispatch` tests it at
  each *linked* boundary too, so a stop lands within one block however long the
  budget is. `jit::dispatch`'s
  `a_raised_exit_flag_stops_within_one_block_however_long_the_block_is` is that
  claim's own gate.
* **A queued interrupt.** A queued event pulls the round's target in through
  `natural_target`, so no budget can run past it.
* **An interrupt raised mid-round.** `TickCursor` is that path: a runnable
  publishes its own tick as it goes and a lazily-advanced device is caught up
  from inside a cycle.

The knob that *does* bound a round in time is `SchedulerConfig::quantum`, and
it always was.

## Guest seconds are now what they say

The consequence for anything that budgets in guest time: **a guest second of a
1 GHz board is a hundred times the work it used to be**, and 4.96 times on
`pc64`. Every budget in the tree written in guest seconds was divided by that
ratio when the cap went, so each covers the same guest work in the same wall
clock:

| where | was | is |
| --- | --- | --- |
| `check.sh long`, arm64 kernel leg | 120 s | 1 200 ms |
| `check.sh long`, synthetic legs | 30 s | 300 ms |
| `check.sh long`, x86 kernel leg | 900 s | 200 000 ms |
| `engine_longrun` a64/riscv synthetic default | 2 s | 20 ms |
| `engine_longrun` x86 synthetic default | 6 000 quanta | 1 200 quanta |
| `benches/{a64,riscv}_linux_boot` default | 20 s | 200 000 000 ns |

`RSEMU_LONGRUN_MS` and `RSEMU_X86_LONGRUN_MS` are the knobs; the `_SECONDS`
spellings still work and still mean whole seconds, so an old command line does
what it always said and simply costs a hundred times the wall clock.

`scripts/check.sh long` with those budgets, all three legs green on this host
(contended, so these are upper bounds):

| leg | wall |
| --- | --- |
| synthetic, 300 ms, three architectures, two engines each | 40.7 s |
| `arm64-virt` kernel, 1 200 ms — 1 200 quanta, `jit` 154 s and `jit-host` 117 s | 273.2 s |
| `pc64` kernel, 200 000 ms — 440 082 quanta, `jit` 418 s and `jit-host` 310 s | 729.4 s |

The x86 leg's own documentation budgeted "about sixteen minutes for the two
engines together" at 900 guest seconds, and 200 covers **more** guest work than
that did — 2.0e10 processor ticks against 1.81e10 — in twelve.

The bench default is the sharpest check available that the two spans really are
the same run: `benches/a64_linux_boot` at its new 200 000 000 ns default
reports the same **154 233 793** guest instructions retired and the same 10.8
per block that `docs/testing/` records for the old `--seconds 20`.

One number is *not* preserved and is worth naming: a leg's coverage of
**guest-time-driven events** falls with its guest-time budget. The synthetic
x86 workload's 2 800 timer interrupts become about 560, because the 8254 fires
on guest time while the code rewrites and page-table walks it also counts fire
per pass. `engine_longrun`'s `DEFAULT_QUANTA` is a fifth rather than a tenth of
what it was for exactly that reason, and it is a trade rather than a wash.

## A declined round ages nobody

A caller's deadline that falls inside a round **declines** it rather than
splitting it (`Scheduler::run_quantum_until`), because a boundary invented by
how a caller sliced its run is a boundary the unsliced run never had — §11.6
and `tests/run_for_additive.rs` are the long form. Nothing executes in that
fragment.

So nothing may *age* in it either, and that half was missing. Virtual time
moved to the deadline and every **crystal no runnable drives** was carried
along with it — a Game Boy cartridge's real-time clock, a CLINT's `mtime`, a
PC's 8254 — while the processors stayed where the last whole round left them.
A passive crystal therefore ran ahead of the processors by up to a round every
time a caller stopped, and a guest that reads one of those counters against
its own cycle count was told that time had passed while it did nothing.

Measured before the rule changed:

| | | |
| --- | --- | --- |
| `riscv-virt`, 100 ms, run whole | `cpu0` at 99 ms | the CLINT at 99.9999 ms |
| `q35`, 20 ms in 1 ms calls | the 8254 at 20.000 ms | `cpu0` at 19.531 ms |

The second row is what a guest sees: the same program measuring the 8254
against its own time-stamp counter read **20.418 cycles per tick** where the
two crystals fix 20.952 — 2.6% — and the error grew the more finely the caller
sliced, because each call ends on a declined fragment. `tests/x86boot` drives
a kernel in one-millisecond calls, so this reached every Linux boot the test
suite runs: a calibration that measures across one such fragment is wrong by
the fragment over its own window, which for a 5 ms calibration window is ten
per cent.

`decline_round` now moves virtual time and nothing else — on a machine that
has a runnable. With **no** runnable at all it still carries the passive trees,
because there is then no processor for them to run ahead of and a declined
round is the only way any clock on such a machine ever moves: a bare device on
a bench, like the timer-and-a-RAM board `dev::stm32::tim` asks for half a
millisecond at a time, would otherwise stand still for ever.

The fragment is repaid to everyone together, because the next round's close
converts every passive tree from absolute time (`advance_undriven_trees`), so a
tree lands on exactly the tick it would have had and no drift accumulates —
`tests/run_for_additive.rs`'s
`every_crystal_stands_at_the_same_instant_when_a_run_returns` is the
assertion, exact integer arithmetic on the declared rationals with a tolerance
of one tick of each crystal compared.

Two Game Boy state hashes moved and no picture did
(`tests/goldens/frame-hashes.txt`): `machines/gameboy.machine` is the only
shipped board with a crystal nothing drives — `osc rtc = 32768 Hz`, the MBC3
cartridge clock — and a domain's tick count is in the state hash, so the
checkpoints at 15 and 60 frames record an RTC that no longer ages through the
frame boundaries. The frame hashes at all four checkpoints are byte-identical,
which is the right answer: the cartridge clock drives no pixel.
