# Clock control: how a device re-rates a clock domain

Consumed by: `core/clock` (`ClockControl`, `ClockForest::set_ratings`),
`core/sched` (`Scheduler::apply_clock_requests`), `core/device`
(`Device::attach_clock_control`), `machine/builtin` (the `clock` class),
`dev/stm32/rcc`, and any future PLL. Companion to
[`execution-budgets.md`](execution-budgets.md), which is about how much a round
hands out; this file is about what happens when the *rate* the round is
measured in changes underneath it.

`ROADMAP.md` §4.2 gives every domain an exact rating against its parent and
`ClockForest::set_rating` has always been able to change one. What did not
exist was a route to it from a device: `RealizeCtx` and `BindCtx` hand out a
`DomainId` and no forest, and a device learns the new rate from inside
`MemOps::write`, several frames below whoever owns the scheduler. So `st.rcc`
computed a perfectly exact 168 MHz and **nothing in the machine ran any
faster**.

## The shape

A device holds an `Arc<ClockControl>` — handed to it by the machine layer at
registration, through `Device::attach_clock_control`, the same way a lazily
advanced device gets its `LazyHandle`. It calls `request_ratio(domain, hz)` and
returns. The scheduler drains the queue at a boundary.

It is a queue and not a call for a reason that is not plumbing. Re-rating
rescales a whole tree's unit position and every domain's `units_per_tick`, and
those numbers are the input to the ratio table `Scheduler::build_ratios`
caches, to the live cursors a parallel round arms in its workers, and to every
budget already handed out. Changing them under a running round is a torn read
of the time model, and under `ThreadingMode::Parallel` a real data race.

## The rule, in full

1. **A request is applied at the scheduling boundary at or after it, never
   inside a round.** There are four such boundaries and they are all the same
   instant-with-nothing-in-flight: the head of a round (`run_quantum_bounded`),
   the tail of one (`close_round`, `close_round_slaved`), the tail of
   `Machine::run_quantum`/`advance_to` after lazily advanced devices have been
   caught up, and `Machine::reset`.
2. **A request carries no timestamp**, several requests for one domain coalesce
   to the last, and the batch is sorted by `DomainId` before it is applied. The
   result is therefore a function of *what* was asked for and not of where in
   the round it was asked, nor of the order two devices wrote their registers
   in. That is the property the determinism gate needs: two runs that program
   the same PLL at different cycles of the same round are in the same state
   afterwards, tick counters and all. `where_in_a_round_the_write_happened_changes_nothing`
   is the test.
3. **Counters are continuous.** `ClockForest::set_ratings` materializes every
   counter in each changed subtree *before* any rating moves, so ticks counted
   at the old rate stay counted at the old rate and counting continues at the
   new one. No tick is lost; none is duplicated. `ticks()` reads the same value
   on both sides of the change.
4. **The sub-tick fraction in flight is discarded** for the re-rated subtree,
   and *only* for it. A domain three quarters of the way through a tick starts
   a fresh edge at the new rate, which is what a PLL that relocks does. A
   domain that is not below a re-rated one keeps its phase exactly — its
   anchor and its `units_per_tick` are both scaled by the same integer factor,
   so `(units − base_unit) % units_per_tick` is preserved.
5. **A request that would not change the rating is dropped**, because applying
   it would perform rule 4's phase reset for nothing. A device is free to ask
   on every register write, and `st.rcc` does.
6. **A refusal is a refusal.** If the forest cannot represent the batch — the
   tree's internal lcm does not fit in 64 bits — the forest is left exactly as
   it was and `SchedError::Clock` comes out of `run_quantum`. Nothing degrades
   to an approximation.

## Why the batch is one call and not a loop

A tree's unit multiplier `A` is the lcm of its domains' ratio numerators and is
**monotonically non-decreasing**, which is what makes each rescale an exact
integer multiplication of the unit position. The cost of that is that every
intermediate state a sequence of single re-ratings passes through leaves its
numerator in `A` for ever — including states the hardware never had. Re-rating
`PCLK1` before `SYSCLK` on a part that has just switched to its PLL folds in
the lcm of a ratio that never existed, and the tree's unit rate, and with it
the headroom in its `u64` unit counter, pays for it permanently.
`ClockForest::set_ratings` applies the whole batch and recomputes each affected
tree once, so `A` takes the lcm of the ratios the machine *ends up with*.

The headroom is still finite, and this is the seam's known limit: a guest that
repeatedly programs PLL ratios with coprime numerators grows `A` without bound
and eventually gets `ClockError::Overflow` on the unit position. Real firmware
configures its tree once or twice. An F407 at HSI 16 MHz and then PLL 168 MHz
off an 8 MHz can reaches `A = 42`, a 336 MHz unit tick, and about 1700 years of
`u64`.

## What it does not do

* **It does not gate.** An output whose source has stopped leaves its domain at
  the rate it had. Stopping a domain is `ClockForest::set_gated`, and it goes
  with the peripheral clock-enable half of the same problem.
* **It does not reparent.** A rating is measured against the domain's *parent*,
  so a controller cannot say which crystal an output came from. A board with
  one declared high-speed oscillator models its internal RC and its PLL as
  exact ratios of that oscillator: the rate a guest measures is exactly right,
  and the physical independence of two cans is not modelled. Saying that
  properly means `reparent` across trees at the moment `SWS` changes.
* **It does not move events already posted.** `EventQueue` holds absolute
  instants; `Scheduler::schedule_at_tick` converts a tick to one at posting
  time and the result stays where it was converted to. Nothing in the tree
  posts a tick-anchored event — every clocked device is either a `Runnable` or
  a `LazyDevice` — and a lazily advanced device needs nothing at all, because
  `Scheduler::lazy_deadline` asks it for its next event **in its own ticks**
  and reconverts through the live forest every round. A device that both posts
  queue events in domain ticks and sits on a re-rated domain would have to
  repost them; when one exists, the anchor belongs in the event and in the
  snapshot chunk beside it.

## Writing a board that uses it

A machine file names a clock domain by naming an object, which left `SYSCLK`,
`HCLK` and `PCLK1` — nodes of the tree that are not peripherals — with nothing
to be named by. `machine/builtin`'s `clock` class is a node and nothing else:
no registers, no state, no ports.

```text
osc hse = 8000000 Hz

object sysclk "clock" { clock = hse * 2 }   # the reset rate: HSI, 16 MHz
object hclk   "clock" { clock = sysclk }
object pclk1  "clock" { clock = hclk / 4 }
object pclk2  "clock" { clock = hclk / 2 }

object rcc "st.rcc" {
  clock = hse, variant = "f4", hse = 8000000,
  sysclk = "sysclk", hclk = "hclk", pclk1 = "pclk1", pclk2 = "pclk2"
}

object cpu  "cpu.arm.v7m" { clock = hclk, … }
object tim2 "st.tim"      { clock = pclk1 * 2, … }
```

Two things this asks of the author, neither of which the device can check:

* **`hse` has to be the truth.** It is the reference every rating is measured
  against, and it already has to agree with the board's `osc hse`.
* **The chain has to be the chain.** Each output is rated against the nearest
  output above it that this instance was given, falling through to `hse`. A
  board that names only `pclk1` gets `pclk1 / hse`, which is right if that is
  how it wired the domain and wrong if it is not.

An output the board does not name drives nothing, which is every board in
`machines/` today: they keep the fixed ratios their machine file declares.
Rewriting one is a behavioural change and should be read as one — a part comes
out of reset on its internal RC, so a board that hands `SYSCLK` to `st.rcc`
runs at 16 MHz until its firmware configures the PLL, where before it ran at
168 MHz from the first instruction.
