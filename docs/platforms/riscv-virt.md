# RISC-V `virt` board

Consumed by: `boards/riscv-virt` — the first machine that boots a real
operating system.

## Why this board

It is the smallest credible target that boots upstream Linux: a RISC-V hart, a
CLINT, a PLIC, a 16550 UART, and virtio-mmio devices. No PCI, no ACPI, no
legacy. Everything it needs is specified in freely available documents, and the
guest discovers the topology from a device tree we generate — so there is no
hidden convention to reverse-engineer.

## Components and their specifications

| Component | Source |
| --- | --- |
| Hart (RV64GC) | [`../cpu/riscv.md`](../cpu/riscv.md) |
| CLINT (timer + software interrupts) | RISC-V privileged spec; `mtime`/`mtimecmp` semantics |
| PLIC (external interrupts) | [RISC-V PLIC specification](https://github.com/riscv/riscv-plic-spec) |
| SBI (firmware interface) | [riscv-sbi-doc](https://github.com/riscv-non-isa/riscv-sbi-doc) |
| Firmware | [OpenSBI](https://github.com/riscv-software-src/opensbi) — **BSD-2-Clause**, readable and usable |
| 16550 UART | [`../devices/network-input.md`](../devices/network-input.md) and the National Semiconductor PC16550D datasheet |
| virtio-mmio | [`../buses/virtio.md`](../buses/virtio.md) |
| Device tree | [Devicetree Specification](https://www.devicetree.org/specifications/) |

## Implementation notes

- rsemu **generates** the device tree from the realized machine graph and passes
  it to firmware. That is a genuine test of the machine model: if the DTB can be
  produced mechanically from the topology, the topology is well-formed.
- Boot chain: our firmware load → OpenSBI (M-mode) → kernel (S-mode). SBI calls
  are the ABI between them.
- Networking comes from `pktkit` behind virtio-net; storage from `fstool`
  behind virtio-blk. Neither needs board-specific code.
- The hart's `time` CSR reads the CLINT's `mtime`, through `timer = clint` in
  the machine file. `time` is architecturally a *view* of the platform timer,
  not a counter the hart owns, and before that line existed `rdtime` read zero
  — which every kernel that takes its clocksource from it turns into an
  immediately-expired deadline and a live-lock. The wiring is
  [`Device::export`](../../src/core/device.rs), and it is named explicitly
  rather than searched for.

## Choosing an execution engine

`cpu.riscv` takes three, and this board makes it a parameter so one file runs
all of them:

```console
$ rsemu run riscv-virt --media firmware=fw_payload.bin -p engine=jit
```

- **`interp`** — the interpreter, and the oracle everything else is measured
  against. The default.
- **`jit`** — the translation runtime in [`src/jit`](../../src/jit): guest
  instructions lifted into IR blocks by
  [`cpu::riscv::lift`](../../src/cpu/riscv/lift.rs), cached under
  `(pc, physical page)`, executed by the portable IR backend. Runs everywhere
  the crate does.
- **`jit-host`** — the same runtime with the **host code generator** attached,
  so blocks are lowered to machine code by
  [`jit::x86`](../../src/jit/x86) on x86-64 Linux in a build with `jit-x86`.
  Anywhere else it falls back to `jit`'s backend and answers identically.

All three are **indistinguishable to the guest**, cycle counts included, and
that is asserted rather than hoped: `tests/riscv_virt_engines.rs` runs this
board on each and compares `Machine::state_hash` at ten checkpoints, then moves
a snapshot between engines in both directions.

It holds on a real guest too. OpenSBI plus a Debian riscv64 kernel and a busybox
initramfs, 512 MiB, one binary, `--headless` so nothing is rate-limited to the
wall clock — over sixty seconds of virtual time, which is the boot, and over
four minutes, which is well past the shell prompt:

| `engine` | 60 s of guest time | 240 s of guest time |
| --- | --- | --- |
| `interp` | 25.1 s | 122.3 s |
| `jit` | 19.8 s (**1.27×**) | 103.7 s (**1.18×**) |
| `jit-host` | 10.3 s (**2.44×**) | 56.8 s (**2.15×**) |

Every cell is the median of three interleaved runs, and in each sweep all three
engines — and the binary from before the last change, run in the same sweep —
finished on one state hash: `0x887e28c90a99e82b` at sixty seconds and
`0xb561134639a875b9` at four minutes. Those numbers belong to *this* invocation,
because a state hash names a stopping point as much as a machine: `--for 240s`
stops at a virtual instant, and the environment harness's `RSEMU_RISCV_QUANTA`
stops after a count of quanta, so the two do not hash alike and neither is more
correct.

The host code generator used to *lose* to the portable one on this guest, at
0.50×, and none of the five things that fixed it was the code it emits.
**Blocks are chained** — 86% of them are reached by following a patched exit,
where the count was previously zero in every run. **A compile stopped costing
144 µs**, which is what two `mprotect` calls over a 256 MiB code buffer had
been costing before `jit::x86::buf` learned to flip a page-sized window
instead. **A guest load stopped costing a call**: the software TLB's fast path
is inlined into generated code now that a hart publishes a `MemPlan`, so 97.3%
of compiled loads are a mask, a compare, an add and a `mov` rather than a trip
through the hart's translation and PMP. **A guest store stopped costing one
too** — 99.8% of them, over a second set whose entries were admitted on write
permission and filled by a walk that set the page's dirty bit, with one thunk
left to pay the tick, the store's dirty bitmap, the reservation and the
self-modifying-code check. And **the PMP scan stopped being asked sixteen
entries at a time**: `pmp_allows` was 265 host instructions a call over 5.5
million calls — a fifth of all emulation — and it is now memoized over the span
its answer is provably constant on.

The last two were measured the way this file insists on, with the old binary
and the new one interleaved in one three-rep sweep over the same guest:

| `engine` | 240 s before | 240 s after | |
| --- | --- | --- | --- |
| `interp` | 155.1 s | 122.3 s | **1.27×** |
| `jit` | 118.6 s | 103.7 s | **1.14×** |
| `jit-host` | 69.1 s | 56.8 s | **1.22×** |

The **interpreter** gains most, and that is the point about where the PMP scan
lived: on the path every engine takes, not on the JIT's. It is also why
`jit-host`'s headline ratio *falls* from 2.28× to 2.15× while the engine itself
got 22% faster — the control moved too, and a ratio against a moving control is
the wrong number to quote on its own. Under callgrind, which is
host-CPU-independent, `Hart::advance` taken inclusively — emulation and nothing
else — went from 7.37 G host instructions to 5.25 G over three seconds of this
boot, **28.7% fewer**.
[`src/cpu/riscv/engine.rs`](../../src/cpu/riscv/engine.rs) has the reasoning and
the measurements behind every one of those claims, including what it costs to
keep the engines identical, and `src/jit/fast.rs` has the argument for why a
*paged* hart may publish a plan at all — and what a **store** plan promises on
top of a load's.

**Measure the interpreter in the same sweep as the engines it is the control
for.** This is a shared machine, and a 150-second run of the same binary
varied by 12% between sweeps run twenty minutes apart — larger than most of the
effects being measured. Each table above is the median of an interleaved
three-rep sweep, and a before-and-after taken from two different sittings is
not evidence. Interleave the *binaries* too when the change is to one engine:
the 13% the inlined fast path is worth was measured with the old and the new
`jit-host` in the same sweep. An earlier sweep of the same two binaries, on the
host CPU this machine had before, read anywhere from 3.6% to 9.4% depending on
what else was running — same code, same guest, same instruction counts. When a
machine is busy, prefer the median of the *per-rep ratios* to the ratio of the
medians: they agree here (13.8% against 12.9%) and the first degrades more
gracefully. They agreed again on the sweep above — 1.268 against 1.268 for the
interpreter, 1.215 against 1.218 for `jit-host` — which is what a quiet machine
looks like, and is worth recording so a sweep where they *disagree* is read as
the warning it is.

A build without `cpu-riscv-lift` and `jit` **refuses** both JIT values with a
message saying which features it wants, rather than interpreting quietly — an
engine that silently is not the one you asked for is how a JIT stays unmeasured
for a year. A build that *has* them but is not x86-64 Linux, or that lacks
`jit-x86`, is a different case and **falls back**: `jit-host` runs the same
blocks from the same cache on the portable backend. The first is a configuration
error and the second is a portability property, so they are treated differently
on purpose.

### The CLINT can raise `mtip` in the middle of a block

> **Reachability, and what changed.** The mechanism below is fixed and tested,
> and for two rounds a *guest on this board* could not reach it: a comparator is
> only crossed where the CLINT is caught up, and while
> `Scheduler::arm_live_cursors` armed nothing across two oscillator trees (see
> *`rdtime` and a load of `mtime` return the same number*) the only catch-up
> inside a round was the `mtimecmp` write's own `republish`. Measured on the
> synthetic board in `tests/engine_longrun.rs` at the time: **1 999 timer
> interrupts in 2 000 quanta**, exactly one each, with the seam knob on and off
> alike. So `a_load_that_raises_an_interrupt_is_taken_where_
> the_interpreter_takes_it`, which builds a device that raises
> unconditionally, was the only coverage this seam had — worth knowing before
> anybody reads `IrHost::load`'s hand-back as dead code.
>
> The cross-tree arming has landed, so a comparator can now be crossed inside a
> round and the count above is the thing to re-measure. Nobody has: the number
> quoted is the old one and is labelled as such rather than quietly refreshed.

A fourth instance of the class `docs/testing/long-run.md` records for A64 —
*where a quantum ends*, not what an instruction computes — and the first one
found on this board. It was fixed in `IrHost::load`.

`cpu::riscv::engine`'s module docs used to say that within a block nothing can
raise an interrupt, and enumerated why: every CSR write, `MRET`, `WFI` and
`SFENCE.VMA` is outside the lifted subset and ends the block; a store ends the
block by construction; and *"what is left is a load from a device that raises
an interrupt as a side effect of being read, which nothing on a `virt` board
does"*. The enumeration was right and the last clause was wrong. The CLINT is
**lazily advanced** (`ROADMAP.md` §4.2): `MemOps::read` catches it up to the
reading hart's live position before answering, and `Registers::republish`
drives `mtip` for every comparator that catch-up crossed.

Most of the time that is invisible, and for a good reason:
`Scheduler::natural_target` ends a round on the soonest event a lazily-advanced
device has of its own, so an expiring comparator *is* a quantum boundary and
both engines see it in the same place. The window is a comparator the guest
moves **into the round that is already running** — which is precisely what a
timer handler does, `mtimecmp = mtime + interval`. The round's target was
chosen when the round began and does not move, so the next `mtime` read crosses
the new comparator, `mtip` rises between two instructions of a lifted trace,
and the block runs on to its natural end: up to `lift::MAX_INSNS` instructions
past where `Exec::step` would have taken the trap, with `mepc` naming the wrong
instruction and the two engines on different cycle counts thereafter.

`a_load_that_raises_an_interrupt_is_taken_where_the_interpreter_takes_it` in
`src/cpu/riscv/engine.rs` is the regression test. It puts a one-register device
that asserts `mtip` when read four instructions from the end of a merged trace;
before the fix the interpreter had `x5 = 0` and both translated engines `x5 =
39`.

**What was looked for and is not reachable.** The hypothesis this was opened on
was a *sibling hart's IPI* — a store into another hart's `MSIP` landing inside
this hart's quantum on an SMP board. Under `ThreadingMode::Deterministic`,
which is the only mode with a state hash, it cannot happen: the sibling runs in
its own quantum, so the write lands while this hart is between quanta and both
engines see it at the next `admit`. Under `ThreadingMode::Parallel` it can, and
there is no oracle to diverge from — that mode gives up reproducibility by
construction and `Machine::state_hash` refuses in it. The fix covers the `MSIP`
case incidentally, because a hart reading anywhere in the CLINT window asks the
same question.

**The walk window, which was argued away and then closed.** `admit` asks
`Exec::pending_interrupt` and *then* charges the entry fetch translation, which
on a TLB miss is a walk — the gap A64's `admit` closes from the other side,
where the walk's own *ticks* cross a comparator. This paragraph used to say the
gap was empty here "because a walk reads page-table entries and this board puts
page tables in DRAM", and that is a statement about what a *guest* does, not
about what the board or the core permits. `satp` is a guest-written register;
`mmu::root` masks its `PPN` field and range-checks nothing; a descriptor read
goes out through `AddressSpace::read` with `MemAttrs::debug` clear. Point
`satp` at `0x0200_0000` and the walk reads the **CLINT**, whose
`AccessConstraints` are `U32`..`U64` on natural alignment — exactly the width
of an Sv39 descriptor — and which is lazily advanced, so answering catches it
up to this hart and drives `mtip` for every comparator that crossed.

So it is closed rather than argued about. `admit` re-asks
`Exec::pending_interrupt` after the translation, gated on `Exec::used` having
changed, which is true only when a walk actually happened; `Admitted::leave`
carries the answer to both of `admit`'s call sites — `advance`'s prologue and
`Frontend::enter`, which is a chained block's entry translation and just as
unwatched — and `Host::hand_back` retires the allowance there too. The
regression test is
`a_walk_that_raises_an_interrupt_is_taken_where_the_interpreter_takes_it`: an
Sv39 root table mapped over a device, a loop whose `sfence.vma` makes both of
its pages cold on every pass, and a sweep over which walk raises. Before it,
`mepc` read `0x4` where the interpreter read `0xc`.

**What it cost.** Callgrind, over `benches/jit_dispatch --smoke`, with
`Exec::step` as the control row — the interpreter shares no code with any of
this, and callgrind counts instructions rather than time, so a control row that
does not move says the instrument is exact rather than merely quiet.
`Hart::advance` inclusive **2 083 019 034 -> 2 084 481 466, +0.070%**;
`Exec::step` inclusive 12 655 697 632 both sides, to the instruction. Two
orders of magnitude under the ~2.5% wall-clock noise floor this host was
measured at last round.

**One exit of `admit` is still open**, and it is the same window at a third
door: a known-unliftable PC on a cold page. The walk happens, raises, and then
`Unlifted::holds` hands the instruction to `Exec::step`, whose own first act is
to take the pending interrupt — so the instruction never runs and `mepc` names
it rather than its successor, where an interpreted hart would have run it
because its `step` looked at the wire *before* its fetch charged the walk. Same
reachability: the walk still has to read a device. Closing it needs `Exec` to
offer one step with the interrupt check already discharged, which is a change
to the interpreter rather than to the engine, and the interpreter is the
oracle.

**What it cost.** Nothing in `IrHost::spent`, which is the function asked at
every guest instruction boundary of every block: `Host::hand_back` retires the
run's tick allowance instead of adding a second field to that comparison, so
`spent` is textually unchanged. What is left to pay is one call to the
interpreter's own `Exec::pending_interrupt` per load that reaches
`IrHost::load` — 2.7% of compiled loads on `jit-host`, all of them on `jit`.
Measured under callgrind, because the host was carrying a load average of
thirty and the wall-clock spread between two runs of the *unmodified* binary
was larger than the effect: `Hart::advance` taken inclusively over the whole of
`benches/jit_dispatch --smoke` went from 1 038 017 036 host instructions to
1 039 145 936, **+0.11%**. A variant carrying a separate `bool` measured
1 037 957 044, *below* the baseline, so the two bracket it and the honest
reading is that the cost is under the instrument's resolution on this mix.

That measurement needed a new table. `benches/jit_dispatch`'s original table
drives `Dispatcher` from a host of its own, so it cannot see
`cpu::riscv::engine::Host` at all — neither the budget seam nor this one — and
a number taken from it would have measured nothing. It now has a second table
in the shape `benches/a64_dispatch` already had: `Hart::run_budget` across six
quanta, with the `interp` row as a control that shares no code with the change.

### `rdtime` and a load of `mtime` return the same number

A suspicion raised while the section above was being written, and settled by
measurement rather than by reading: the two ways a guest can ask for the
platform timer look like they should disagree.

* A load of `0x0200_bff8` enters `Registers::read`, which calls
  `Registers::sync` and catches the block up before answering.
* `csrr t0, time` never touches the bus. The hart holds a copy of
  `Registers::mtime_cell`, sampled once per `Hart::step`, and that cell is
  written only by `Registers::republish` — on an advance or on a guest write.

`rdtime_and_a_memory_mapped_mtime_read_agree` in `src/dev/riscv/tests.rs` runs
a loop that reads both back to back, 125 000 times, and keeps the largest
difference and a count of every occasion `time` went backwards. The assertion is
that the gap never exceeds one tick and that `time` never goes backwards —
which is what the architecture licenses.

**They now differ by one tick, and that is the fix working.** The largest gap
over those 125 000 round trips is exactly **1**, and the backwards count is
zero. It used to be zero and zero, which looked like agreement and was really a
counter standing still: within a round there was nothing for `sync` to catch up
to, for two separate reasons that landed a round apart.

* The hart published nothing. It publishes a `TickCursor` position now —
  `Hart::attach_cursor` keeps both halves and `Exec::publish_position`
  publishes `State::cycles` before every access that leaves for the address
  space, which is the one publication point `interp`, `jit` and `jit-host` all
  reach identically.
* `mtime` counts on the `rtc` crystal, a separate oscillator tree from `core`,
  and `Scheduler::arm_live_cursors` armed a live view only across slots sharing
  a root — so the CLINT's slot was skipped whatever the hart published. It is
  armed now, through a ratio built from the two domains' declared rational
  frequencies. See *The remaining half* below for why that is not the thing
  §4.2 forbids.

A `csrr time` and a load of `mtime` one instruction apart are therefore
separated by real ticks of a 10 MHz counter, and the load answers with the next
one. One tick is the bound; nothing here can produce two.

**What publishing the position cost.** Callgrind, whole process, this board
with a hand-written RV64 loop in its firmware slot — thirteen instructions of
which two touch memory, fifteen bus accesses a lap — for two virtual seconds,
which is 20 000 000 bus accesses. The state hash is `0xa542803012fbfd38`
before and after on all three engines, so the two binaries are running the same
guest instruction for instruction and the difference is entirely emulation
overhead:

| `engine` | before | after | delta |
| --- | --- | --- | --- |
| `interp` | 12 680 455 433 | 12 991 963 786 | **+2.46%** |
| `jit` | 9 547 432 602 | 9 554 538 973 | +0.074% |
| `jit-host` | 3 405 887 145 | 3 406 566 898 | +0.020% |

Two functions move and every other line of the profile is identical to the
instruction: `Exec::step` by +175 000 014, which is `publish_position` at
**8.75 host instructions per bus access** — an `Arc` deref, a relaxed store, the
relaxed load of `TickCursor`'s own deadline and the compare — and
`Hart::step_to_exit` by +130 000 010, which is the cursor reaching `Exec` and
the register pressure that comes with it.

The spread between the engines is the whole design in one table. A translated
block fetches nothing per instruction and the compiled fast path serves its RAM
loads through `note_fast_load`, which charges a tick without going near
`Exec::read_at` — so the two engines that matter for a boot pay essentially
nothing, and the interpreter, which fetches every instruction through the same
door a device would come in by, pays for all of them. Skipping fetches would
recover most of it and is deliberately not done: a guest in a long arithmetic
stretch would then publish nothing at all, and `TickCursor::set`'s deadline —
the mechanism that delivers a lazily-advanced device's *own* event mid-round —
would have nothing to fire on.

`benches/jit_dispatch` was run interleaved either side of this, three sittings
each, and is **not** the number reported: its untouched control columns —
`cached`, `+superblock`, `+compiled`, which drive the dispatcher from the
bench's own `IrHost` and never enter `Exec` — moved by up to +18% and −22%
between sittings on this host, an order of magnitude above the ~2.5% floor it
was calibrated at. Its `interp` rows do agree with the table above, at −2.3% to
−5.8% across every budget; its `jit chain` row shows −16%, consistently and in
both sittings, against a callgrind delta of +0.074% for that engine, which is
code layout rather than work.

**The remaining half was a scheduler change, and it has landed.**
`ROADMAP.md` §4.2 does not forbid relating two trees; it says how — "reciprocal
multiply + a per-root residual accumulator", with the error bounded below one
unit and non-accumulating. What it forbids is routing an *intra*-tree
relationship that way, and that path is untouched:
`core::sched::tests::an_intra_tree_ratio_is_still_exact_with_another_crystal_present`
puts a second crystal in a NES forest and asserts the PPU is still exactly three
dots a cycle. `core::sched::Ratio` carries one pair of numbers for both cases,
built from the domains' units-per-tick within a tree and from their declared
rational frequencies across two, reduced once per topology change rather than
once per round.

`engine_longrun`'s `the_clint_advances_while_the_hart_is_running` reports
**800** distinct `mtime` values over eight quanta instead of seven — the hundred
per round the numbers below predict — the largest `time`/`mtime` gap is one
`rtc` tick, and `riscv_virt_engines`'s cross-engine hashes are identical. The
test is no longer `#[ignore]`d; its assertion was written against the fixed
behaviour while the bug was still there, so it turned green without an edit.

**What it cost.** Callgrind, whole process, `benches/frame_time --only
riscv-virt --frames 6`: 7 640 656 394 → 7 640 862 542, **+0.0027%**. The
conversion itself is one `u64` multiply and one divide on the access path, the
same two instructions the intra-tree path always ran; the reduction and the
frequency lookups happen in `Scheduler::build_ratios`, which is keyed on a
topology epoch and does not run per round at all.

**What the board should not do instead.** Putting `mtime` on the core's tree —
`clock = core / 100`, which is exact — would make the intra-tree path do the
work today and needs no scheduler change at all. It is the wrong trade. The
divider ties the platform timer's *rate* to the core clock, so `-p` or a PLL
that re-rates `core` silently changes what `mtime` counts at while the device
tree's `timebase-frequency` goes on saying 10 MHz, and every guest delay is
wrong by that factor. It also only works because 1 GHz / 10 MHz happens to be
an integer: a board with a real 32.768 kHz can could not express it at all.
`mtime` is architecturally a counter that "increments at a constant rate", and
on real hardware that rate comes from a different crystal — which is exactly
what the `osc rtc` line says.

This used to qualify one clause of the section above, and no longer does.
Before the cross-tree arming a *read* of the CLINT advanced it by nothing: the
loop above took about 312 `mtime` loads per round and every one of them returned
the identical value, so the window that section describes was opened here only
by the `mtimecmp` **write**'s own `republish` — `Registers::write` syncs and
republishes unconditionally, and republishing raises `mtip` for any comparator
the current `mtime` is already past — rather than by a later read catching the
block up across the new comparator. A read now advances the chip to the hart's
live position, so the read route is open too. The fix and its regression test
were unaffected either way; the test uses a device of its own that raises a line
when read, which is a shape a board is free to have.

**The bound.** `mtime` *was* a staircase to the guest, one step per scheduler
round: with no comparator armed a round runs to
`SchedulerConfig::max_ticks_per_quantum`, and the step measured on this board
was exactly 10 000 `rtc` ticks — one millisecond — every round, for as long as
the machine ran. That is what the cross-tree arming removed. A guest that loads
`mtime` now sees it advance inside the round, and the residual step is the one
`rtc` tick a single 100-core-tick interval is worth. `csrr time` still lags by
up to that one tick, because it reads `Registers::mtime_cell` as `republish`
last left it rather than syncing — see *Nothing was changed in the CSR path*.

**The architecture allows exactly this.** *Volume II: Privileged Architecture*,
"Machine Timer Registers": "When `mtime` changes, it is guaranteed to be
reflected in `time` and `timeh` eventually, but not necessarily immediately."
And *Volume I*'s `Zicntr` chapter licenses the staircase from the other side,
in the note under "the real-time clocks of all harts must be synchronized to
within one tick": it is "acceptable for this example implementation to only
update the real-time clock at, say, a frequency of 100 MHz with increments of
10 ticks. As long as software cannot observe this seeming violation of the
above synchronization requirement, and software always observes time across
harts to be monotonically nondecreasing, then this implementation is
compliant." What the spec does *not* forgive is time going backwards, and that
is the assertion the test is built around.

Cross-hart agreement comes free on `riscv-virt-smp`:
`Scheduler::publish_lazy_positions` runs at round close and nowhere else, so
both harts in a round read one cell holding one value — identical rather than
merely within a tick.

**Nothing was changed in the CSR path, deliberately.** The obvious repair —
have the CSR read sync the device the way a load does — would put a device
catch-up on the instruction Linux runs most often, since `rdtime` is the
userspace clocksource through the vDSO. While the CLINT was stranded on its own crystal it also bought nothing, the sync
it would perform being one that advanced nothing. Now that the cross-tree
arming has landed it would buy exactly **one tick** of freshness — the gap
`rdtime_and_a_memory_mapped_mtime_read_agree` measures — at the price of a
lazy-device catch-up on the instruction Linux runs most often. That is the
trade, and it is still not worth taking: one tick of a 10 MHz counter is 100 ns,
below anything the vDSO's callers can act on, and the architecture licenses
exactly this lag.

**The one latent hazard, and the second one it turned into.** `Csrs::mtime` is
sampled once per `Hart::advance`, which is once per *block* under a translating
engine. What a `rdtime` reads is fresh only because `cpu::riscv::lift` admits no
CSR instruction, so a block always ends before one; lift a CSR read and `time`
freezes for the length of a block.
`a_guest_write_to_mtime_is_visible_to_the_very_next_rdtime` stores to the
memory-mapped `mtime` and reads `time` two instructions later, on every engine
the build has, so that day is caught.

The second hazard is the same sample seen from the *snapshot* rather than from
a guest instruction, and it is real the moment `mtime` can move inside a round:
a start-of-`advance` sample leaves the cache holding the cell as of the last
instruction under `interp` and as of the start of the last **block** under a
JIT, which is one guest state hashing two ways. `cpu::riscv::resample_timer` takes a
second sample after each run unit, which both engines reach on the same guest
instruction because both finish a budget there. It was written for the
cross-tree change above and verified against it — without it
`every_engine_hashes_to_the_same_machine_at_every_checkpoint` fails at the first
quantum with `mtime 0x63` interpreted against `0x62` translated. It was a no-op
until that change landed, since a cell that only moves at round close is the
same value at both ends of a run unit; it is load-bearing now, and that test is
what says so.

**The neighbours.** `cycle` and `instret` are read-only shadows of `mcycle` and
`minstret` (Volume II, `mcounteren`), which are hart-local and have no
memory-mapped alias at all — there is no second route for them to disagree
with, and the two engines were checked to report identical values for the same
program. The x86 `RDTSC` path has the same shape and the same immunity:
`prot.rs::rdtsc` reads `state.cycles`, the core's own counter, and
`IA32_TIME_STAMP_COUNTER` reads that same field, so the two cannot part
company either.

## Booting something real

Everything below is fetched, never committed
(`scripts/fetch-testdata.sh riscv linux`), and gated behind environment
variables so an ordinary `cargo test` skips it.

```console
$ export TD=testdata/riscv
$ RSEMU_RISCV_FIRMWARE=$TD/fw_jump.bin \
  RSEMU_RISCV_PAYLOAD=0x80200000:$TD/linux \
  RSEMU_RISCV_RAM=1G RSEMU_RISCV_QUANTA=8000000 \
  cargo test --release --all-features firmware_from_the --lib -- --nocapture
```

OpenSBI's `fw_jump` runs at `0x80000000` and hands control to `0x80200000` in
S-mode, which is where a RISC-V `Image` expects to be.

## Booting to a shell

A kernel with no root filesystem panics in `prepare_namespace`, correctly. Give
it a ramdisk and it does not:

```console
$ scripts/fetch-testdata.sh linux initramfs
$ RSEMU_RISCV_FIRMWARE=$TD/fw_jump.bin \
  RSEMU_RISCV_PAYLOAD=0x80200000:$TD/linux \
  RSEMU_RISCV_INITRD=$TD/initramfs.cpio \
  RSEMU_RISCV_BOOTARGS='console=ttyS0 earlycon=sbi' \
  RSEMU_RISCV_RAM=512M RSEMU_RISCV_QUANTA=2000000 \
  RSEMU_RISCV_INPUT='rsemu# =>uname -a\n' \
  RSEMU_RISCV_STOP_AT='GNU/Linux' \
  cargo test --release --all-features firmware_from_the --lib -- --nocapture
```

`initramfs.cpio` is **built** by the fetch script, not downloaded: one
statically linked riscv64 busybox out of Debian's own package, a `/dev/console`
node, and a ten-line `/init`. The `newc` cpio writer is forty lines of shell in
`scripts/fetch-testdata.sh`, so the fixture needs no cross toolchain and no
`cpio(1)`, and every entry is written with mtime 0 so the archive is
reproducible.

The board carries the ramdisk the way a real one does. A `riscv.loader` writes
it into DRAM at `initrd_addr`, and the boot ROM puts
`/chosen/linux,initrd-start` and `linux,initrd-end` in the generated tree — the
same media slot named twice, so the length is read from the bytes in both
places and only the address is written down more than once.

`RSEMU_RISCV_INPUT` types at the guest: one `marker=>text` step per line, fed
when the guest has printed `marker`. That is what makes the console
*bidirectional* rather than write-only, and matching on output rather than on
elapsed time keeps the run deterministic.

## Booting with the virtio disk

The Debian kernel builds `virtio_mmio` and `virtio_blk` as modules, so an
unadorned initramfs never claims the `virtio.blk` this board provides.
`initramfs-virtio` is the same archive with those two modules in `/lib/modules`
and an `insmod` loop at the top of `/init`; it resolves the kernel package from
the fetched image's own version banner, because a module whose vermagic
disagrees is refused at load time.

```console
$ scripts/fetch-testdata.sh linux initramfs-virtio
$ RSEMU_RISCV_INITRD=$TD/initramfs-virtio.cpio RSEMU_RISCV_DISK=$TD/disk.img …
```

`--disk` / `RSEMU_RISCV_DISK` binds the `disk` media slot, which is the front of
the disk; the `storage` parameter pads it out with zeroes. That is the
media-slot path: bytes, copied into a `RamStore`, `no_std`, and what a wasm
build runs on.

## Booting off a qcow2

The other path backs the same slot with a host **file**. `virtio.blk` stores
its bytes behind `dev::medium::Medium`, the seam an ATA drive's platter and an
NVMe namespace already use, so `--drive` works here for the reason it works
there and no image format is parsed in rsemu — sparse raw, qcow2, DMG and LUKS
all come from `fstool`.

```console
$ rsemu run riscv-virt --media firmware=fw.bin --drive disk=root.qcow2,new=64M
```

The medium brings its own capacity, so `storage` is ignored and the guest sees
the image's size; a 16 GiB disk costs 16 GiB of *disk* and nothing in host
memory until the guest touches it. Guest writes go into the file, so the next
run is a reboot of the last one — and a machine snapshot **references** the
image (flushing it first) rather than copying it, which is what
[`storage.md`](../buses/storage.md) argues at length.

`RSEMU_RISCV_DRIVE` is the same thing for the test harness, with
`RSEMU_RISCV_DRIVE_NEW=<size>` to create the image and `RSEMU_RISCV_DRIVE_RO`
to open it read-only (which the device reports as `VIRTIO_BLK_F_RO`, so the
guest finds out before it tries):

```console
$ RSEMU_RISCV_FIRMWARE=$TD/fw_jump.bin \
  RSEMU_RISCV_PAYLOAD=0x80200000:$TD/linux \
  RSEMU_RISCV_INITRD=$TD/initramfs-virtio.cpio \
  RSEMU_RISCV_DRIVE=$TD/root.qcow2 RSEMU_RISCV_DRIVE_NEW=64M \
  RSEMU_RISCV_RAM=512M RSEMU_RISCV_QUANTA=6000000 \
  RSEMU_RISCV_BOOTARGS='console=ttyS0 earlycon=sbi' \
      cargo test --release --all-features firmware_from_the --lib -- --nocapture
…
[  225.340000] virtio_blk virtio0: [vda] 131072 512-byte logical blocks (67.1 MB/64.0 MiB)
```

64 MiB rather than the board's `storage = 16M`, which is the whole point: the
guest is reading the image's geometry, not the machine file's. Two runs against
that image are a write and a reboot:

```console
rsemu# echo rsemu-qcow2-round-trip | dd of=/dev/vda bs=512 count=1 conv=sync,fsync
WROTE-OK
… second run, same qcow2, no RSEMU_RISCV_DRIVE_NEW …
rsemu# head -c 22 /dev/vda
rsemu-qcow2-round-trip
```

**`fsync`, not `sync`.** A guest write is durable in the image when the guest
asks for it to be — `VIRTIO_BLK_T_FLUSH`, which is what `dd conv=fsync` and
`fsync(2)` on the device produce — and `sync(1)` alone is not that. Linux
writes a bare block device's dirty pages back on `sync(2)` but issues the
device cache flush from `blkdev_fsync`, so with `sync` alone the data cluster
reaches the qcow2 and the L2 entry that finds it does not; the next open sees a
hole. That is the flush contract working as specified rather than a defect, but
it is sharp, and nothing yet flushes an image when a *run* ends.

## Booting UEFI

`edk2-riscv-code.fd` from EDK2's `OvmfPkg/RiscVVirt` (BSD-2-Clause-Patent) is
built for the board's NOR flash at `0x20000000`, with the variable store in a
second bank at `0x22000000`. Both are **real `flash.cfi` devices** in
`machines/riscv-virt.machine` — parallel NOR with the Intel/Sharp command set,
a CFI query structure, per-block erase and the bit-clearing-only program
semantics that fault-tolerant write depends on. Staging RAM under those windows
gets the firmware as far as the DXE dispatcher and no further, because
`VirtNorFlashDxe` will not install `gEfiVariableWriteArchProtocolGuid` against
memory.

The one splice that remains is the trampoline, and it has nothing to do with
the flash: `fw_jump.bin` has its hand-off address compiled in at
`0x80200000`, so eight bytes there — `lui t0, 0x20000` then `jr t0` — bridge
OpenSBI to the flash base, leaving `a0` (hart id) and `a1` (device tree) as
OpenSBI set them. OpenSBI's `fw_dynamic` takes the next stage's address at run
time and would need no trampoline at all.

```console
$ printf '\xb7\x02\x00\x20\x67\x80\x02\x00' > $TD/tramp.bin
$ cp /usr/share/qemu/edk2-riscv-vars.fd $TD/vars.fd     # a writable copy
$ RSEMU_RISCV_FIRMWARE=$TD/fw_jump.bin \
  RSEMU_RISCV_PAYLOAD=0x80200000:$TD/tramp.bin \
  RSEMU_RISCV_FLASH0=/usr/share/qemu/edk2-riscv-code.fd \
  RSEMU_RISCV_FLASH1=$TD/vars.fd \
  RSEMU_RISCV_FLASH1_OUT=$TD/vars.fd \
  RSEMU_RISCV_RAM=512M RSEMU_RISCV_QUANTA=6000000 \
  cargo test --release --all-features firmware_from_the --lib -- --nocapture
```

## Two harts

[`machines/riscv-virt-smp.machine`](../../machines/riscv-virt-smp.machine) is
this board with a second hart. It is a **separate file** rather than a `param`
on the first, for the reason `arm64-virt-smp` is separate from `arm64-virt`:
the description language declares objects and cannot be told how many to make,
so a one-hart run of a two-hart file would be a board with a spare hart parked
in firmware. `riscv-virt` is unchanged, down to the byte in its device tree,
and it is still the file `tests/riscv_virt_engines.rs` runs its three-engine
equivalence gate on.

### What had to change, and it was less than on the other two architectures

**No device model changed** — not a line of `clint.rs`, `plic.rs`, `dt.rs` or
`boot.rs`, and nothing in `src/cpu/riscv/` or `src/core/`. The diff under
`src/` is a catalog entry, module documentation and tests. That is not modesty
about the work; it is the architectural fact this board exists to demonstrate,
and it is worth stating next to what the other two boards needed.

`arm64-virt-smp` and `pc-at-smp` both had to teach a controller *who was
asking*. A GICv2 banks its low 32 interrupt ids — one address, N registers
(IHI 0048 §4.1.3) — and an x86 local APIC has one architectural page shared by
every processor. Both are demultiplexed on `MemAttrs::requester`, resolved from
the machine file's `processors = [...]` through `BindCtx::peer`.

RISC-V has no such register. Both of its controllers are indexed by hart id in
the *address*:

| | where hart `h`'s copy lives | source |
| --- | --- | --- |
| software interrupt | `MSIP + 4·h` | ACLINT specification, MSWI register map |
| timer comparator | `MTIMECMP + 8·h`, i.e. `0x4000 + 8·h` | ACLINT specification, MTIMER register map |
| interrupt enables | `0x2000 + 0x80·c` for context `c` | PLIC specification §3 |
| threshold and claim | `0x200000 + 0x1000·c` | PLIC specification §3 |

A hart reaches its own registers because it knows its own `mhartid`, which the
boot ROM has always put in `a0`. `mtime` is the one genuinely shared register,
and it is shared on real hardware too. So `harts = 2` on the CLINT and on the
PLIC is the entire model change, and both blocks were written that way in their
first commit — `Clint::with_harts` allocates a comparator and an `msip` bit per
hart, `Plic::build` allocates `harts × 2` contexts, and both refuse a snapshot
whose hart count disagrees. There is **no `processors` property on this board
and no `BindCtx::peer` call anywhere in it.**

The device tree needed nothing either. [`dt.rs`](../../src/dev/riscv/dt.rs)
takes the hart count from `riscv.boot` and already emitted one `cpu@N` node
with its own `riscv,cpu-intc` phandle per hart, and one phandle-and-cause pair
per hart in the CLINT's and the PLIC's `interrupts-extended` — causes 3 and 7
for machine software and machine timer, 11 and 9 for machine and supervisor
external (Privileged Architecture, the interrupt cause table).

What the file does add is eight wires instead of four: `clint.mtip1`,
`clint.msip1`, `plic.meip1` and `plic.seip1` into the second hart. And an
**IPI needs no mechanism at all** — on this architecture an interprocessor
interrupt *is* a store of 1 to a sibling's `msip` word, delivered as cause 3.
`a_store_to_the_other_harts_msip_is_an_interprocessor_interrupt` in
`src/dev/riscv/tests.rs` is that in twenty instructions: hart 0 writes
`0x02000004`, hart 1 is in `wfi` and lands in `mtvec`.

The CLINT's re-entrancy was already right, and this is where a board like this
usually breaks. A CPU holds a `BUS`-ranked lock across the accesses it issues,
so delivering an IPI — a device reaching *another hart* from inside a guest
access — must not take a lock above it. `Registers::write` mutates `msip` under
the state lock, releases it, and only then calls `drive_msip`, which takes the
output pins at `LockRank::LEAF`. That contract is `CLAUDE.md`'s and it was
being kept before there was a second hart to keep it for.

### How the second hart starts: SBI HSM, not a spin table

This is the one design decision the board file makes, and it is different from
`arm64-virt-smp`'s.

Both harts come out of reset at `0x1000` — that is what a reset vector is — and
the boot ROM's five instructions send both of them to `0x80000000` with
`mhartid` in `a0`. There is no parking loop in the ROM, because on this
architecture the firmware already provides one. OpenSBI picks one hart to
initialise the platform on — its own banner names it, `Boot HART ID : 0` —
and holds the others until something asks the **Hart State Management**
extension to start them: `sbi_hart_start(hartid, start_addr, opaque)`, SBI
specification v2.0 §9. Linux's `smp: Bringing up secondary CPUs` is that call.

What the board has to supply for it is exactly what the CLINT already supplied,
because **HSM's wake-up is an IPI**: a hart waiting to be started is woken by a
write to its own `msip` word. So the interprocessor path above is on the
bring-up path and not only on the reschedule one, and `IPI1: 4620 2007` in the
`/proc/interrupts` further down is the same wire doing the ordinary job
afterwards.

The alternative was a boot ROM that parks secondaries by hart id, the way
`arm.boot`'s reset vector parks everything but affinity 0 on a release table.
It was rejected for two reasons. This board's documented boot chain **is**
OpenSBI, so SBI HSM is present in every guest anybody boots on it, and a kernel
would use it in preference to anything else on offer. And a parking loop would
be a second, rsemu-specific bring-up protocol invented here rather than read
out of a specification — exactly the sort of thing the AArch64 board had to
resort to only because PSCI `CPU_ON` needs a route to a *sibling* core that
`cpu.arm.a64` has not got. `cpu.riscv` needs no such route: hart 0 starts hart 1
by writing a register through the ordinary address space, which is what the
hardware does.

Nothing in `riscv.boot` therefore grew a `secondary` property, and nothing in
`src/cpu/riscv/` was asked for.

### The reservation set is global now, and Linux's spinlocks are built on it

**Was**: `cpu::riscv`'s `reservation` was private per-hart state — broken by
*this* hart's stores, its AMOs and its traps, and by nothing a sibling did. So
an `sc.d` the architecture requires to fail could succeed and the sibling's
update was lost, which is the identical defect `docs/platforms/arm64-virt.md`
records for `arm64-virt-smp` from the identical cause. The board still booted
to a shell, because a kernel's spinlocks are uncontended almost always and two
harts rarely reach the same lock inside one scheduler quantum — luck about
timing rather than a property of the model.

**Is**: `core::space::ExclusiveMonitor` is Volume I's reservation set, living
on the `AddressSpace` because a space is one coherence domain. Each hart claims
a slot when it is given the space; `State::reservation` stays as the *local*
half, holding the virtual address the `SC` compares, and an `SC` commits only
if both agree. Every store that reaches `SpaceView::write_span` — a guest
store, a DMA burst, a `LOCK`ed x86 access on a heterogeneous board — breaks any
reservation covering the bytes it touches, keyed on the **physical** address,
because two harts contending for one lock reach it through their own `satp`.
The compiled fast path does not go through the space at all, so
`Exec::note_fast_store` tells the monitor itself.

The granule is eight bytes (`RESERVATION_SHIFT`). Volume I requires the
reservation set to contain at least the naturally aligned `XLEN`-bit word the
`LR` read; larger would be legal and would cost forward progress.

`usermode::proof`'s `a_siblings_store_breaks_this_cores_reservation` is the
regression, on both architectures at once, and it is worth knowing why RISC-V
did not find this on its own: LLVM emits a single `amoadd.d` for an atomic add
where AArch64 without `FEAT_LSE` needs an `lr`/`sc` pair, so the threaded guest
that lost 32 038 of 40 000 increments on AArch64 landed all 40 000 here. Same
source, same defect, different compiler output. `cmpxchg` is `lr`/`sc` on both.

### The reservation set was necessary and not sufficient: three windows inside one instruction

The section above closes the window *between* an `lr` and its `sc`. It says
nothing about the windows *inside* each of those instructions, and inside an
AMO, and there were three. All three are the same shape — this hart issues a
guest atomic as several separate accesses through `AddressSpace`, and the only
lock held across them is the hart's own `BUS`-ranked session mutex, which is
per hart and excludes nothing on a sibling. Every one of them is the AArch64
defect verbatim; that architecture found them first and the argument below is
the same argument checked against a different manual.

What the manual asks for, and the two shapes ask for different things:

* **AMOs.** *Volume I*, "Atomic Memory Operations": the instruction
  "atomically" loads the value at `rs1`, places it in `rd`, applies the
  operator and stores the result back. There is no status register with which
  to report a failure and no retry loop in the guest to catch one.
* **`LR`/`SC`.** RVWMO's **atomicity axiom**: if `r` and `w` are the paired
  load and store of an aligned `LR`/`SC` in hart *h*, and `s` is the store to
  byte *x* whose value `r` returns, then `s` precedes `w` in the global memory
  order and there is **no store from a hart other than *h* to byte *x*** in
  between. Two harts whose `sc`s both succeed against one word is exactly that
  forbidden store.

`tests/riscv_amo_atomicity.rs` is the instrument: two harts, two host threads,
each incrementing one word 60 000 times, once with `amoadd.w` and once with an
`lr.w`/`sc.w` loop, beside a second word in the same reservation granule
incremented with a plain `lw`, `addi` and `sw` as the witness that the two
really overlapped. A lost update is not a reordering — it is a value no
interleaving of the two programs could have produced — so unlike almost
everything else in this area it can be *asserted* on an x86-64 host rather than
printed.

Each row below is a *configuration*, not an isolated window: the fixes are
peeled off one at a time from the bottom up, so a row measures everything the
rows beneath it still leave open. 20 runs sequential plus 36 six-way parallel
for the first two rows, 36 six-way parallel for the rest — a loaded host, which
is what widens these windows; an idle one misses the narrow ones entirely.

| configuration | lost of 120 000 | runs that lost any |
| --- | --- | --- |
| `amoadd.w`, no bus lock anywhere | 3 911 – 13 616 | 56 of 56 |
| `lr`/`sc`, no bus lock anywhere and the granule claimed after the read | 38 – 4 343 | 56 of 56 |
| `lr`/`sc`, the bus lock on the `sc` alone, granule still claimed after the read | 7 – 493 | 36 of 36 |
| `lr`/`sc`, the bus lock on the `sc` alone, granule claimed **before** the read | 1 – 4 | 23 of 36 |
| everything, as the tree stands | 0 | 0 of 172 |

An AMO's read-to-write window and an `sc`'s check-to-store window — and, as the
last row showed, the load-reserved as well — are closed by
`AddressSpace::bus_lock`, taken by `Exec::lock_bus` before the
instruction issues a data access and held until it ends: the span
`cpu::x86::exec` already uses for a `LOCK` prefix, and for the same second
reason, because a bus lock fences at *both* ends and a guard that opened after
the first access would put the barrier inside the instruction it is meant to
bracket. It is the bus lock rather than the reservation because an AMO lands on
x86's side of the line `core::space::BusLock` opens with — a reservation is
licensed to fail spuriously and is paid for by the guest's retry loop, and
`amoadd.w` has no status register and no retry loop, so a spurious clear would
make it return a wrong answer rather than go round again.

The `lr`'s own claim-after-read is not a lock's problem. `ExclusiveMonitor`
walks *live* slots, so a sibling's store landing after an `lr`'s read but
before it registers its reservation clears nothing, and the `sc` that follows commits a value that was
already stale. The fix is to claim the granule before the read issues rather
than after it returns (`Exec::reserve_then_read`), which costs nothing at all:
the same translation, the same read, in the other order. What it changes is that
an `lr` which faults leaves the reservation cleared instead of holding what it
held before — Volume I licenses an `SC` to fail for reasons other than a store
to the reservation set, and the alternative would be a reservation on a word the
hart never loaded.

The last window is the one that survives all three of those fixes, and it is
why the load-reserved takes the bus lock at all. `SpaceView::write_span` breaks
reservations **before** it transfers rather than after, on purpose: a store that
then faults has still broken them, which is a licensed spurious clear, and
clearing afterwards would mean threading a split transfer's outcome back out of
its loop. The consequence is that a committing `sc` has a window *inside
itself*, between telling the monitor and writing the bytes:

```text
hart 0: sc.w [bus held]              hart 1: lr.w [no lock]
  reservation holds
  note_store: walks the live slots,
    and hart 1 has none yet
                                       reserve: the slot goes live
                                       read [a0] -> 5, not written yet
  write [a0] <- 6
[bus released]
                                     sc.w: nothing broke the reservation,
                                     so it commits 6 — hart 0's update is gone
```

Claiming the granule first does not help: the claim lands *after* the sibling
looked. The `sc` holding the bus does not help either, because the `lr` on the
other side was holding nothing. What closes it is the load-reserved taking the
bus lock too — not to make a write indivisible, since it has none, but so that
claiming the granule and reading it are one transaction against a sibling in
the act of storing. That is what hardware gives for free: the reservation is
registered by the same coherent access that returns the data.

The measurement is the point of the last two rows together. Claiming the
granule before the read cuts the loss from 7–493 per run to 1–4 and the
rate from 36 of 36 to 23 of 36 — two orders of magnitude, and **not** a fix.
An earlier round of the AArch64 work argued that a load with no write had
nothing to gain from a bus lock and was wrong; this port did not repeat the
argument, and the 23-of-36 row is what it would have cost. Read those rows as
failure *rates*: a defect worth one update in 120 000 is exactly the shape a
green run hides.

There is a second way to close the fourth, in `core::space` rather than here:
move `note_store` after the transfer. It would also close a residual this does
not — a *plain* `sw` racing an `lr` can still have its `note_store` run before
the `lr` claims the granule and its bytes land after the `lr` has read, leaving
a reservation that should have been broken. That is `BusLock`'s plain-store
residual reaching the monitor, it is narrower than the case above (a plain store
racing a reserved word is a data race in the guest's own terms), and the fault
trade-off is real, so it is written down rather than taken. It is also somebody
else's file.

#### Which mode, and what it costs

Reachable only under `ThreadingMode::Parallel`, which is opt-in
(`--threading parallel`) and which no board in `machines/` selects — though a
machine file now can, with a `threading` statement; see
[`../techniques/parallel-execution.md`](../techniques/parallel-execution.md).
The argument that `Deterministic` is safe is structural rather than statistical:
one host thread cannot interleave inside an instruction, and the test's
one-instruction-quantum run — finer than any quantum the scheduler hands out —
loses nothing before any of the fixes or after them. The JIT does not widen it
either: `cpu::riscv::lift` excludes the whole `A` extension, so no atomic is
ever inside a block a budget could leave part-way.

Measured on one hart with nothing contending, release build, as nanoseconds per
loop iteration — the instruction under test plus the `addi` and `bne` that
drive it, which is the ~166 ns bottom row. Best of four runs on an idle host:

| | before | after | delta |
| --- | --- | --- | --- |
| `amoadd.w x0, a1, (a0)` | 199 ns | 219 ns | +20 |
| `amoadd.w.aqrl`, the same encoding with both ordering bits | 199 ns | 219 ns | +20 |
| `lr.w` + `sc.w`, uncontended | 330 ns | 371 ns | +41 |
| plain `lw` | 184 ns | 186 ns | +2 |
| plain `sw` | 181 ns | 183 ns | +2 |
| the loop alone, as the control | 165 ns | 167 ns | +2 |

So about +19 ns for each instruction that now takes the bus, which is one
uncontended mutex with its two fences (`core::space::BusLock` prices its own
acquire at ~13 ns); the pair pays it twice. The three control rows move by 2 ns,
which is this harness's noise and the honest floor on reading the others.

`aqrl` costs the same as the relaxed encoding in *both* columns, for two
different reasons. Before, the ordering bits were decoded and ignored. After,
the guard's two `SeqCst` fences bracket the whole instruction, which is
strictly stronger than either bit asks for — the same over-approximation
`cpu::x86::exec` makes for every `LOCK`-prefixed instruction, and the price of
using one mechanism for indivisibility rather than two.

The ordinary load and store paths are untouched, which is the point of the
control rows: nothing outside the `A` extension reads the bus lock, so a board
that executes no atomic pays nothing. What a real guest pays is one mutex on an
instruction it executes thousands of times a second, not millions — and RISC-V
leans on the cheaper of the two shapes, since LLVM spells an atomic add as one
`amoadd` where an Armv8.0 part needs a pair.

So: the boot below is evidence about bring-up, per-hart timer and external
interrupt delivery, IPIs, and now the atomics too. What it is still not
evidence about is *ordering*. This core executes one instruction at a time and
completes every access before the next, so a guest that depends on a weak
memory model being weak has nothing here to disagree with — `FENCE` executes a
host `fence(SeqCst)` (`docs/techniques/memory-models.md`), but a fence on a
host that was already ordering the accesses changes no outcome this board can
produce.

### What a kernel does with it

Debian's `riscv64` kernel — the same image as the single-hart gate, behind the
same OpenSBI `fw_jump.bin` — on `riscv-virt-smp`:

```console
$ export TD=testdata/riscv
$ RSEMU_RISCV_MACHINE=riscv-virt-smp \
  RSEMU_RISCV_FIRMWARE=$TD/fw_jump.bin \
  RSEMU_RISCV_PAYLOAD=0x80200000:$TD/linux \
  RSEMU_RISCV_INITRD=$TD/initramfs.cpio \
  RSEMU_RISCV_BOOTARGS='console=ttyS0 earlycon=sbi' \
  RSEMU_RISCV_RAM=512M RSEMU_RISCV_QUANTA=8000000 \
  RSEMU_RISCV_INPUT='rsemu# =>cat /proc/interrupts; nproc; head -3 /proc/stat\n' \
  RSEMU_RISCV_STOP_AT='cpu1 ' \
  cargo test --release --all-features firmware_from_the --lib -- --nocapture
```

`RSEMU_RISCV_MACHINE` is the only difference from the single-hart invocation,
which is the point: same firmware, same kernel, same ramdisk, same script.

OpenSBI sizes the board off the generated tree and finds both harts (its own
banner, with the lines this section is about kept and the rest elided):

```text
Platform Name               : rsemu riscv-virt-smp
Platform HART Count         : 2
Platform IPI Device         : aclint-mswi
Platform Timer Device       : aclint-mtimer @ 10000000Hz
Platform HSM Device         : ---
…
Standard SBI Extensions     : ipi,pmu,srst,hsm,rfnc,time,base,legacy,dbcn
…
Domain0 Boot HART           : 0
Domain0 HARTs               : 0*,1*
```

`Platform HSM Device : ---` is not a gap. That line reports a platform-specific
hart power controller; OpenSBI's *generic* HSM implementation — the one that
parks a hart until `sbi_hart_start` arrives and wakes it with an IPI — needs no
device behind it, and `hsm` is in the extension list below it. `0*,1*` is the
domain saying both harts are assigned to it and both may boot.

And the kernel:

```text
[    0.000000] Machine model: rsemu riscv-virt-smp
[    4.379000] smp: Bringing up secondary CPUs ...
[    4.481000] smp: Brought up 1 node, 2 CPUs
[    6.786000] cpu1: Ratio of byte access time to unaligned word access is 4.00, unaligned accesses are fast
[    6.817998] cpu0: Ratio of byte access time to unaligned word access is 1.99, unaligned accesses are fast
```

The last two lines are what makes this evidence rather than a claim about a
device tree. The unaligned-access probe runs **on each hart**, times a copy
loop there, and prints from that hart — so `cpu1:` is a line hart 1 printed
about work hart 1 did. The two ratios differ because the harts measured at
different points in the same virtual timeline, which is what a real pair of
cores does too.

One more line says the same thing from the other direction, and it is a
complaint rather than a status report:

```text
[   57.142009] rcu: INFO: rcu_sched self-detected stall on CPU
[   57.145000] rcu: 	1-....: (5249 ticks this GP) …
[   57.147000] rcu: 	(t=5250 jiffies g=-1131 q=2 ncpus=2)
[   57.150000] CPU: 1 UID: 0 PID: 1 Comm: swapper/0 Not tainted …
[   57.152000] epc : keccakf_round+0x352/0x4f8
[   57.154000]  ra : crypto_sha3_final+0xf2/0x1c0
```

`ncpus=2`, and PID 1 — the kernel's own init thread — is running on **hart 1**
when it stalls. This is the same artifact the single-hart boot has and the same
cause: virtual time here is derived from bus accesses, a crypto self-test makes
a great many of them, and the kernel decides 5250 jiffies went by without a
grace period. It is not an SMP defect; what is new is only that the scheduler
put the thread on the second hart, which nothing but a running second hart can
do. The kernel warns and carries on to a shell.

And it runs userspace on both. Typed at the busybox prompt, three minutes of
host time into the run:

```text
rsemu# cat /proc/interrupts; nproc; head -3 /proc/stat
           CPU0       CPU1
 10:      53635      58007  RISC-V INTC   5 Edge      riscv-timer
 12:         64          0  SiFive PLIC  10 Edge      ttyS0
IPI0:        64         67  Rescheduling interrupts
IPI1:      4620       2007  Function call interrupts
IPI2:         0          0  CPU stop interrupts
…
2
cpu  38 0 18424 7173 0 0 38 0 0 0
cpu0 25 0 6883 5036 0 0 19 0 0 0
cpu1 13 0 11540 2136 0 0 19 0 0 0
```

Five separate claims, each checked by a different column:

* **`nproc` → 2.** Userspace agrees with the kernel.
* **The timer row is per hart** — 53 635 against 58 007. That is `riscv-timer`
  on `RISC-V INTC 5`, the *supervisor* timer, which arrives because OpenSBI
  answers `sbi_set_timer` by programming that hart's own `mtimecmp` in the
  CLINT and the CLINT drives that hart's own `mtip` wire. Two harts, two
  comparators, two wires, two counts.
* **The IPI rows are non-zero in both directions.** 64/67 rescheduling and
  4620/2007 function-call interrupts are `msip` writes each hart made to the
  other's word, and each one is a device driving a wire into a sibling hart
  from inside a guest store — the re-entrancy case the CLINT's lock discipline
  exists for.
* **`ttyS0` counts 64 on CPU0 and 0 on CPU1**, which is the PLIC doing the
  opposite thing correctly: an external interrupt is offered to the contexts
  that enabled it, and Linux enabled it on one.
* **`/proc/stat`'s `cpu1` line has 11 540 jiffies of system time and only
  2 136 idle** — more system time than `cpu0`. The second hart is not parked
  after bring-up; it is where most of the kernel work went.

178 seconds of host time from reset to that output, under the interpreter, in
a release build — on a machine running six other builds at the time, so read it
as an order of magnitude and not as a measurement. This page's own rule about
interleaving a sweep applies: the 143 s the single-hart boot quotes above was
taken in a different sitting and the two are **not** a before-and-after.

### What the hermetic tests cover

Five of them, in `src/dev/riscv/tests.rs`, none of which needs a download:

* `both_harts_run_and_each_one_knows_which_it_is` — one image, both harts
  enter it, `bne a0, x0` sends them to different halves, and hart 0 cannot
  reach its `poweroff` unless hart 1 wrote the handshake word.
* `the_single_hart_board_runs_only_hart_zero` — the control. The same program
  on `riscv-virt` never finishes, which is what says the two board files are
  genuinely different rather than both being SMP.
* `a_store_to_the_other_harts_msip_is_an_interprocessor_interrupt` — hart 1
  sets `mtvec`, enables `mie.MSIE` and `mstatus.MIE` and waits in `wfi`; hart 0
  stores 1 to `0x02000004`; hart 1 lands in its handler, clears the word (the
  only way `msip` clears — there is no acknowledge bit) and answers.
* `a_two_hart_machine_snapshots_and_restores_to_the_same_state_hash` — two
  comparators, two `msip` bits, four PLIC contexts and two harts' register
  files, out and back.
* `the_generated_tree_describes_both_harts` — `cpu@0`, `cpu@1`, and eight cells
  of `interrupts-extended` on each of the CLINT and the PLIC.

## How far each guest gets

Written down rather than rounded up, because a precisely located stopping point
is the only kind that is useful.

### Linux

**Linux 6.12 (Debian riscv64 installer kernel)** runs its whole boot: every
initcall, the driver model, and the console handover off the SBI earlycon onto
this board's own 16550A —

```
[  110.961106] 10000000.serial: ttyS0 at MMIO 0x10000000 (irq = 12, base_baud = 115200) is a 16550A
[  110.969106] printk: legacy console [ttyS0] enabled
[  110.975106] printk: legacy bootconsole [sbi0] disabled
```

— and, given a ramdisk, **reaches a shell prompt that echoes what is typed at
it**:

```text
[  222.376146] Run /init as init process

rsemu initramfs on Linux 6.12.94+deb13-riscv64 riscv64

BusyBox v1.37.0 (Debian 1:1.37.0-6+b8) built-in shell (ash)
Enter 'help' for a list of built-in commands.

/bin/sh: can't access tty; job control turned off
rsemu# uname -a
Linux (none) 6.12.94+deb13-riscv64 #1 SMP Debian 6.12.94-1 (2026-06-20) riscv64 GNU/Linux
```

`uname -a` on the second-to-last line is the *echo* of the nine bytes the
harness fed to the port; the line under it is the reply. Nothing echoes it but
the guest's own terminal line discipline, so that pair is the console proving
it carries bytes in both directions. **143 seconds of host time** from reset to
that prompt, under the interpreter, on one core.

With `initramfs-virtio` — the same archive plus the kernel's own
`virtio_mmio.ko` and `virtio_blk.ko` — Linux claims the board's virtio disk and
reads and writes it:

```text
[  284.552146] virtio_blk virtio0: 1/0/0 default/read/poll queues
[  284.876146] virtio_blk virtio0: [vda] 32768 512-byte logical blocks (16.8 MB/16.0 MiB)
rsemu# head -c 34 /dev/vda
rsemu virtio-blk fixture, sector 0
rsemu# printf "vda-%s" roundtrip-ok > /w && dd if=/w of=/dev/vda bs=512 seek=1 && sync && dd if=/dev/vda bs=512 skip=1 count=1 | head -c 16
vda-roundtrip-ok
```

The first read is the host's `disk.img` arriving through the virtqueue; the
second command writes a sector and reads it back, so the descriptor ring is
exercised in both directions by Linux's own driver rather than by ours. 205
seconds of host time, the extra minute being module relocation and probe.

Without a ramdisk the kernel still panics in `prepare_namespace`, and that
remains the correct end of a boot nobody gave a root filesystem to.

Two observations from these runs that are ours, not the kernel's:

- `jitterentropy` trips the soft-lockup watchdog during `jent_entropy_init`
  (`BUG: soft lockup - CPU#0 stuck for 22s!`, and again at 44s). Virtual time
  here is derived from bus accesses, and jitterentropy's calibration loop makes
  a great many of them; the kernel warns, taints itself `[L]=SOFTLOCKUP`, and
  carries on. It costs real time as well as virtual: the initcall spans t=59s
  to t=105s of the 212 virtual seconds before `/init`, a little over a fifth of
  the boot — call it **half a minute of the two and a half wall-clock
  minutes**. Worth knowing before this becomes a CI fixture, and worth
  measuring against a kernel built without the module before assuming a
  command-line switch would skip it.
- Time is virtual throughout, so the timestamps above measure emulated seconds,
  not patience.

**Its ASID probe found a real bug in our `satp`.** An early initcall discovers
`ASIDLEN` the way the privileged specification suggests — write all ones to
`satp.ASID`, read back which bits stick:

```asm
csrr  a1, satp          # keep MODE and PPN
lui   a5, 65535
slli  a5, a5, 32        # 0xffff << 44 -- the ASID field
or    a5, a5, a1
csrw  satp, a5
csrr  a4, satp          # this fetch is the one that used to fault
```

`satp.PPN` is 44 bits under Sv39 and `ASID` sits directly on top of it. Masking
`PPN` any wider folds ASID bits into the root page table's address, so the
instant that `csrw` retired the whole address space moved and the *next* fetch
took an instruction access fault — as did the fetch of the trap handler, and so
on forever. The guest stayed live, ping-ponging through OpenSBI's M-mode trap
entry on the timer, which is what made it look like an SBI problem. It was not:
every SBI call in the trace is a well-formed `sbi_set_timer` (EID `0x54494d45`,
FID 0) being answered correctly. With the field masked to its real width the
kernel prints `ASID allocator using 16 bits (65536 entries)` and carries on.

`RSEMU_RISCV_STOP_AT` ends the run at the first line containing it — firmware
that reaches a prompt does not stop by itself — and `RSEMU_RISCV_FLASH1_OUT`
writes the variable bank back out when it does. Pointing `FLASH1_OUT` at the
same file `FLASH1` read is a reboot.

### EDK2/UEFI

To a shell, in about two minutes of host time under the interpreter:

```text
[Bds]Booting EFI Internal Shell
UEFI Interactive Shell v2.2
EDK II
UEFI v2.70 (EDK II, 0x00010000)
Shell>
```

`gEfiVariableWriteArchProtocolGuid` (`6441F818-6362-4E44-B570-7DBA31DD2453`)
installs, and "discovered but not loaded" falls from **47 drivers to 13** — the
thirteen left are the network stack and the PCI drivers, whose depexes this
board genuinely does not satisfy.

**A variable written in one run is there in the next**, and the variable store
is where that is visible rather than inferred. UEFI's store is an append-only
log in flash, because appending is the only thing a part that can merely clear
bits is able to do:

| | bytes programmed in the 256 KiB store | log ends at |
| --- | --- | --- |
| the image as shipped | 97 | `0x000063` |
| after one run | 2086 | `0x000857` |
| after a second run from that image | 2158 | `0x00089f` |

The second run **continued** the log rather than restarting it: it read run
one's `BootOrder`, `Boot0000`-`Boot0002`, `ConIn`, `ConOut`, `PlatformLang` and
`Timeout` out of the flash and appended only what had changed. A store that had
come up blank would have ended at `0x857` again.

The firmware finds both banks the same way it finds everything else here — the
generated device tree carries a `cfi-flash` node per bank, and EDK2's
`FdtNorFlashQemuLib` walks them, skips the one overlapping its own firmware
volume, and makes the other `PcdFlashNvStorageVariableBase`. Nothing is written
down twice: the addresses in the tree come out of the `map` statements.
