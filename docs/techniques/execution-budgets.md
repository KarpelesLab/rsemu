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

1. **`Scheduler::ticks_until_after`** asks how many ticks of the runnable's own
   clock domain fit between where its oscillator tree actually stands and the
   target. *Actually stands*, not where the target's own conversion says it
   should: a runnable stops on a tick boundary (§4.2, "stop at the cycle
   boundary before"), so a tree lags the exact conversion by whatever fraction
   of a tick the last round could not spend, and a round that was declined
   outright leaves a whole round's worth behind. Measuring from the real
   position is what hands those back.
2. **`Scheduler::take_share`** divides that by however many runnables still
   have a turn on the same tree in this round, and counts itself down — see
   *[The share](#the-share)*. A tree with one runnable divides by one.
3. **`SchedulerConfig::max_ticks_per_quantum`**, if a caller set one, lowers it
   further. It is `None` by default.

The runnable reports what it consumed, the forest advances by that, and the
next runnable on the same tree sees the position the first one left.

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

## The share

The cap was doing a second job, and that one is real: **bounding a runnable's
share so its siblings are scheduled at all.**

A tree has one unit counter and every domain on it is a divider of that
counter, so two runnables on one oscillator are spending one span between them.
Offer the first one the whole span and the second is offered nothing. All four
shipped `-smp` boards are in exactly that configuration, and so was the cap's
only defence of them: on a 100 MHz board a round's span was 100 000 ticks and
the cap was 10 000, so both processors got 10 000 and both ran. Raise the
constant past the span and the second processor was starved for ever, which
`tests/parallel_smp_boards.rs` asserted as a fact rather than describing.

`Scheduler::tree_shares` replaces that with the quantity it was standing in
for. A round offers each runnable **the span its tree has left, divided by
however many runnables still have a turn in this round**, counting itself down:
the first of two gets half, the second then finds half the span gone and one
sharer left, so it gets the other half. A gated domain is not counted.

Three things follow:

* **Two processors on one crystal both run with no cap at all.**
* **The deterministic and dispatched rounds agree by construction** — both
  divide the same span the same way — rather than by accident of ordering.
* **A tree with one runnable divides by one**, so the bound is exactly the
  span and nothing is rounded away. That is every board in `machines/` except
  the four `-smp` ones, which is why removing the constant moved exactly two
  committed state hashes in `tests/goldens/frame-hashes.txt` and left every
  other one identical to the bit.

And one rule that is not obvious until a board needs it: **a share that rounds
down to nothing still gets one tick**, whenever the span itself is not zero.
Dividing starves a slow domain otherwise, which is a different bug from the one
the share fixes — `tests/vnc_input.rs` is the board that found it, an 8086 at
4.77 MHz and an 8042 at 1 193 Hz on one crystal, where the controller's tick is
838 µs against a 1 ms round, so half a round is six tenths of one tick and the
keyboard never moved a byte. The span stays the ceiling, so the runnable that
takes that tick leaves the next one nothing and the round-robin's rotation
gives that one its turn next round — which is what happened before any share
bound existed.

### What it does not fix

Each of two processors on one crystal still executes at **half** the rate its
board declares. One counter cannot say that one of them halted while the other
ran, and dividing is the best a single counter can do. That wants two
oscillators, which is what `machines/tests/heterogeneous.machine` has and what
§4.2's "as many roots as the real board has crystals" asks for.
[`parallel-execution.md`](parallel-execution.md) says what has to be measured
before the four shipped files change.

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
