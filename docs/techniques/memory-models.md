# Memory consistency models

Consumed by: `ir/` (atomic ops and barrier lowering), `cpu/*` frontends, phase
9. This is where parallel emulation goes wrong, and the failures are
load-dependent, host-specific, and nearly impossible to debug after the fact —
which is why the rules are fixed in advance.

## The problem

When guest CPUs run on parallel host threads, guest memory ordering must be
preserved on a host whose ordering rules are different. Three cases:

- **Guest weaker than host** (e.g. RISC-V or ARM guest on x86 host): nothing to
  emit *for the guest's ordinary loads and stores*. The host is already
  stricter than the guest's baseline requires.
- **Guest stronger than host** (e.g. **x86-TSO guest on AArch64 or wasm**): the
  frontend lifter **must** insert barriers. Miss one and the guest sees
  reorderings its ISA promises cannot happen.
- **A guest barrier instruction is neither case.** It is the guest asking for
  something stronger than *its own* baseline, so "weaker than or equal to the
  host" does not answer it: an `MFENCE` on an x86 host still has to be a host
  fence, because the host's baseline is not stronger than what `MFENCE` asks
  for. That row used to read "nothing to emit" and it cost the tree a year of
  no-op barriers; see below.

`ROADMAP.md` §4.7 assigns this responsibility explicitly to the frontend lifter:
the core provides atomic primitives, the lifter owns the ordering.

## Sources

| Topic | Source |
| --- | --- |
| Rigorous models for x86, ARM, POWER, RISC-V | [Peter Sewell's group, Cambridge — relaxed memory concurrency](https://www.cl.cam.ac.uk/~pes20/weakmemory/) — the `x86-TSO` paper and the ARM/POWER tutorials are the standard references, with formal models and litmus tests |
| x86 ordering rules | Intel SDM Volume 3, "Memory Ordering" **[browser]** |
| ARM ordering rules | Arm ARM (DDI 0487), the memory model chapter **[browser]** |
| RISC-V RVWMO | [riscv-isa-manual](https://github.com/riscv/riscv-isa-manual) Volume 1, Chapter 14 — **CC-BY-4.0** |
| WebAssembly threads | [WebAssembly threads proposal](https://github.com/WebAssembly/threads) — the wasm memory model, relevant to the threaded browser target |
| Rust's model | The `core::sync::atomic` documentation; Rust follows the C++20 model |

## Litmus testing

The Cambridge group publishes **litmus tests** — small multi-threaded programs
with a defined set of allowed outcomes. Running them as guest programs under
parallel translated execution is the only practical way to gain confidence that
barrier lowering is right, and it belongs in the SMP-emulation gate alongside the
atomics stress suite.

The mode those guest programs have to run in, and what a run in it is checked
by when a state hash is not available, is
[`parallel-execution.md`](parallel-execution.md). The short version, because it
decides how a litmus result should be read: a *lost update* is a value no
interleaving could produce, so it is a gate on any host; a missing *barrier* is
not, because a host strong enough to hide the reordering hides it before and
after the fix. Litmus tests gate on a weak host, which is what the AArch64 CI
leg is for.

## Implementation notes

- Guest atomic instructions lower to host atomics through the IR's atomic ops
  (`cmpxchg`, `fetch_*`, `xchg`, fences).
- Load-linked/store-conditional guests (ARM, RISC-V, PowerPC) do not map
  directly onto compare-and-swap hosts; the standard approaches (address
  monitors, or CAS with an ABA-tolerant scheme) each have documented failure
  cases. Decide deliberately and write the reasoning down.
- Each frontend gets its own memory-model conformance suite (`ROADMAP.md` §12).

## Where this stands today, measured

Four things are kept and two are not. Everything below is reachable **only**
under `ThreadingMode::Parallel`, which is opt-in (`--threading parallel`, or a
`threading` statement in a machine file); no board in `machines/` selects it,
`Deterministic` is the default, and `usermode`'s `ThreadSet` runs every guest
thread on one host thread by design. Under one host thread the finest
interleaving there is is one whole instruction, so none of it can be observed —
which is why the tree has got this far without it mattering.

**Kept.** The load-reserved pair, by `core::space::ExclusiveMonitor`: a store by
any observer to the reservation granule clears the slot, so a store-conditional
that raced anything fails and the guest retries. The x86 locked
read-modify-write, by `core::space::BusLock`, against *another* locked one.

**Kept, as of this round: the A64 atomics, against each other.** The paragraph
above was true of the *interval* between a load-reserved and its
store-conditional and not of the instructions themselves, and the difference
cost updates. `cpu::arm::a64` issued a `FEAT_LSE` atomic as a separate load and
store with nothing held across them, checked a `STXR`'s monitor and then stored
as two acts, and claimed a `LDXR`'s reservation *after* the read had already
returned — so a sibling's store in any of those three windows went unnoticed.
There was a fourth, and it is the one worth remembering: because
`SpaceView::write_span` breaks reservations *before* it transfers, a committing
`STXR` has a window inside itself in which a sibling's `LDXR` — holding no lock
— could claim the granule and read the pre-store value. Two cores, two host
threads, 120 000 increments of one word: `tests/a64_lse_atomicity.rs` lost
3 410–8 850 through the first window, 318 through the second, 46 through the
third, 1–3 through the fourth in **33 of 60 runs on a loaded host**, and loses
none now over 126.

The first, second and fourth are closed by `AddressSpace::bus_lock`, held across
the whole instruction as x86's `LOCK` already was — a `FEAT_LSE` atomic is
pessimistic and unconditional, which is exactly the case `core::space::BusLock`
exists for and the case the optimistic monitor cannot serve; and a
load-exclusive takes it not to make a write indivisible, having none, but so
that claiming the granule and reading it are one transaction against a sibling
in the act of storing. The third is closed by taking the reservation before the
read rather than after it, which costs nothing.
`docs/platforms/arm64-virt.md` has the interleavings, the reasoning and the
price (+14 ns an instruction that takes the bus; nothing on the ordinary load
and store path).

Two things generalise from it. **`note_store`-before-transfer was a window in
every core that has a monitor**, and it has since been closed in `core::space`
rather than in any core, by telling the monitor on **both** sides of the
transfer. That was not the trade the round that found it expected: clearing on
a store that *completed* is required by both manuals ("The `sc` **must** fail if
a store to the reservation set from another hart can be observed to occur
between the `lr` and `sc`"; "Any successful write to the marked block by any
other observer … is **guaranteed** to clear the marking"), while clearing on a
store that *faulted* is only ever permitted — so the first call keeps the
licence and the second keeps the requirement, and there is no trade at all.
`core::space::monitor`, "The transfer is the window", has the interleaving and
the citations; what is left after it is "Not kept: a plain store against another
master's atomic" below, which is where all three of these objects' residuals are
now written down once. And **fixing one instruction of a pair is not
fixing the pair**: the `STXR` half was locked a round before the `LDXR` half,
and the defect that survived cost one update in 120 000, which is exactly the
size a single green run hides.

**Kept, as of this round: the RISC-V `A` extension, against itself.** All four
windows were in `cpu::riscv::exec` verbatim, and the port is line for line:
`Exec::lock_bus` across an AMO, across an `SC` and across an `LR`, and
`Exec::reserve_then_read` claiming the granule before the read issues. What
differs is the manual rather than the mechanism. An AMO is Volume I's
"atomically load … apply … store the result back", with no status register and
no retry loop, so it is on x86's side of `BusLock`'s opening distinction for the
same reason `FEAT_LSE` is; the pair is governed by RVWMO's **atomicity axiom**,
which forbids any store from another hart between a paired `LR`'s load and its
`SC`'s store — and two harts that both *check* before either *stores* commit
exactly that. `tests/riscv_amo_atomicity.rs` is the twin instrument: 3 911–13 616
lost of 120 000 through the AMO window, 38–4 343 through the pair's, in 56 of 56
runs, and none over 172 after. It also prices the trap: claiming the granule
before the read, with the `LR` still unlocked, cuts the pair from 7–493 lost per
run to 1–4 and the rate from 36 of 36 to 23 of 36 — necessary, two orders of
magnitude, and **not** a fix. `docs/platforms/riscv-virt.md` has the table and
the cost (+19 ns an instruction that takes the bus; +2 ns, which is the
harness's noise, on the ordinary load and store path).

This is the one thing in this section a test on an x86-64 host can gate, and
the reason is worth keeping: a lost update is not a reordering. It is a value
no interleaving could have produced, so no amount of host strength hides it and
the assertion can be an equality rather than a printed count.

**Not kept: single-copy atomicity.** `RamStore` is a `Vec<AtomicU8>` and every
access to it is a byte loop, so a naturally aligned four-byte load racing a
naturally aligned four-byte store can return a mixture of the old and the new
word — a value all three architectures forbid (*Intel SDM* volume 3 §9.1.1, ARM
DDI 0487 B2.2.1, RISC-V Unprivileged ISA §1.4).
`tests/smp_single_copy_atomicity.rs` catches it 117–361 times in sixty thousand
loads, and catches the read half of a `LOCK XADD` torn the same way 90–138 times
with the bus lock held throughout. That second form is x86's; the tearing itself
belongs to every architecture here, because an ordinary aligned load has no
backstop on any of them.

It follows the **store**, and it is narrower than "engine-dependent". An earlier
version of this paragraph said "a store the JIT inlines is one host instruction
of the guest's width and does not tear", which is true and misleads twice. A
*load* the JIT inlines still comes back torn if the racing store went through
the byte loop, because no reader can un-tear a store made in four pieces — so
the guarantee follows whichever core is storing. And `jit::x86` is the *host*
backend: inlining happens only where a core publishes `FastMem::store_plan`,
which AArch64 and RISC-V do unconditionally and **x86 does only in long mode**
(`cpu::x86::lift::Lifter::flat`, and `docs/platforms/pc64.md`'s "The inlined
memory path" for what it refuses). So an x86 guest's stores tear wherever a plan
does not reach — below long mode, through `FS` or `GS`, in a read-modify-write,
and in the interpreter, which is the configuration
`tests/smp_single_copy_atomicity.rs` builds its cores in.

`core::space::store`'s "What per-byte atomicity is not" has three candidate
shapes, all re-derivable from `tests/memory_model_costs.rs`, and why none was
taken. The one correction worth repeating here is the price: "+12% of the store
path" divided by `SpaceView::write_span` (≈19 ns when that was written, ≈25
before it, and ≈12 since the span loop stopped being inlined into every value
store), which is a sixth of an interpreted store instruction (≈117 ns) rather
than a path a guest executes. The
byte loop is ~1% of that instruction and full conformance costs +2–3% of it.
Both fixes are affordable at that denominator; the question is the memory model,
not the nanoseconds.

**Not kept: a plain store against another master's atomic.** This is the
authoritative account; `core::space::BusLock`, `core::space::ExclusiveMonitor`
and `SpaceView::write_span` each described a face of it from their own end, and
now point here instead.

One rule explains all of it: **a plain store takes no lock and asks no
question.** Every mechanism that makes a guest's atomic indivisible is a
protocol between participants that opt in — `AddressSpace::bus_lock` for a
`LOCK` prefix, a `FEAT_LSE` atomic and an AMO; `ExclusiveMonitor` for the
load-reserved pair. An ordinary `STR`/`sd`/`mov` joins neither, so it can land
inside another master's multi-event operation, and both objects' residuals are
that one sentence seen from two ends.

There are two windows, and the smaller one is the one the previous three rounds
were looking at.

```text
core 0: plain STR [x] <- 6         core 1: ldxr / stxr on [x]

                                     ldxr: reserve [x], read [x] -> 5
                                     stxr: monitor holds, so commit
                                           — the answer is now given  ─┐
  note_store: clears the                                              │
    reservation, too late                                             │ (A)
  write [x] <- 6                                                      │
  note_store                                                          │
                                     stxr: write [x] <- 6, from 5    ─┘
                                     core 0's store is gone
```

* **(A) between the `SC`'s decision and the `SC`'s write** — `BusLock`'s
  residual. The `STXR` holds the bus lock across both, but core 0 does not take
  it, so nothing keeps the two apart. The gap is a whole `SpaceView::write`,
  tens of nanoseconds. This is the dominant one by an order of magnitude.
* **(B) between a store's bytes and the `note_store` that follows them** — the
  monitor's residual, and the one `core::space::monitor`'s "the transfer is the
  window" used to state. A reservation taken after the leading `note_store`
  survives it; if the transfer then crosses the granule and the `SC` consults
  the monitor before the trailing `note_store`, it commits against a granule
  that has just been written. For a value store that window is a couple of
  nanoseconds and the pair cannot fit inside it. For a **span** transfer — a DMA
  burst, a ROM load, `write_bytes` — it is as wide as the rest of the burst.
  `core::space::tests`'
  `a_store_conditional_inside_a_burst_can_still_see_a_written_granule` reaches
  it on purpose rather than racing for it, and is the pin that says it is real.

**An affected program is well defined, and is required to work.** This is not a
case where only an already-erroneous guest is hurt, and an earlier draft of
`BusLock`'s residual said the opposite ("a plain store racing a locked
read-modify-write on the same word is a data race in the guest's own terms")
— that claim was wrong and is withdrawn. All three manuals forbid the outcome,
naming a *store*, not another atomic:

* RISC-V Unprivileged ISA, RVWMO, the **atomicity axiom**: "If `r` and `w` are
  paired load and store operations generated by aligned LR and SC instructions
  in a hart `h`, `s` is a store to byte `x`, and `r` returns a value written by
  `s`, then `s` must precede `w` in the global memory order, and **there can be
  no store from a hart other than `h`** to byte `x` following `s` and preceding
  `w` in the global memory order." Its `Zalrsc` chapter says the same
  operationally — "The SC must fail if a store to the reservation set from
  another hart can be observed to occur between the LR and SC" — and, which
  settles the DMA question, "The SC must fail if **a write from some other
  device** to the bytes accessed by the LR can be observed to occur between the
  LR and SC."
* Arm DDI 0487 B2.9.2: "Any successful write to the marked block by any other
  observer in the shareability domain of the memory location is guaranteed to
  clear the marking."
* *Intel SDM* volume 3 §9.1.2.2: the `LOCK#` signal "ensures that the processor
  has exclusive use of any shared memory while the signal is asserted" — against
  every other access, not only against another locked one.

And the guest source that compiles to it is ordinary, which is the part that
matters. `AtomicU64::store(v, Relaxed)` lowers to a plain `str`/`sd`/`mov` on
all three; `compare_exchange_weak` lowers to `LDXR`/`STXR` on pre-`FEAT_LSE`
AArch64 and to `lr.d`/`sc.d` on RISC-V without `Zacas`. Two threads, one
storing and one compare-exchanging the same atomic, is race-free C++ and
race-free Rust, and the model requires exactly what the manuals do: "Atomic
read-modify-write operations shall always read the last value (in the
modification order) written before the write associated with the
read-modify-write operation" (C++ `[atomics.order]`; Rust follows the C++20
model). A guest doing that gets a lost update here.

**Measured.** Two host threads over one `AddressSpace`, one storing a plain
aligned doubleword and one running the `LR`/`SC` pair as `cpu::arm::a64::exec`
issues it, 20 000 rounds. A lost update is detected soundly — the writer
publishes each value only after its store has returned, and the reserver reads
the word back after committing — and torn reads are excluded by writing repeated
bytes, so `RamStore`'s byte loop is not counted as this. **112 to 2 416 lost of
814 to 4 786 committed store-conditionals, in 20 of 20 sequential runs and 6 of
6 of a parallel batch**, with the burst arm and the value-store arm both losing
in every run. Of those, at most 2–13% had a store in flight at the instant of
the decision, which is the ceiling on window (B)'s share; the rest is (A).

**What closing it costs.** The sound fix is the obvious one — every store takes
the bus lock — and it works: the same reproducer with the writer holding
`AddressSpace::bus_lock` loses **zero, in 52 of 52 arms**. The price, on the
store path that a value store reaches in ~12.7 ns:

| | 1 B | 2 B | 4 B | 8 B |
| --- | --- | --- | --- | --- |
| `AddressSpace::write` today | 13.41 | 12.71 | 12.67 | 13.06 ns |
| … taking the bus lock | 30.74 | 30.94 | 31.00 | 31.29 ns |
| … with only the "already held?" branch | 12.72 | 12.90 | 12.91 | 14.15 ns |

That is **+18 ns, +144%** uncontended, and uncontended is not the case that
decides it. Aggregate store throughput, one master per host thread, each
storing into its own page so that the *only* thing shared is the lock:

| masters | today | with a bus lock on every store |
| --- | --- | --- |
| 1 | 67.6 M stores/s | 29.5 |
| 2 | 40.1 | 12.5 |
| 4 | 31.2 | 4.7 |
| 8 | 32.2 | 3.5 |

A **9× collapse at eight masters**, which is the whole of what parallel
execution is for. The bus lock is affordable on `LOCK` because
`docs/platforms/pc64.md` counts 0.21 M locked instructions in nine hundred guest
seconds of a Linux boot; a store happens billions of times in the same run, and
the two are four orders of magnitude apart.

**The reentrancy story does not change that arithmetic.** `STXR`, `LOCK` and
`LDADD` already hold the bus lock while they store, so a naive acquire in
`write_span` deadlocks, and the fix is to let a caller that holds it through —
the third row of the table above says that check is free. But it only exempts
the stores made *by* locked instructions, which are the 0.21 M; every plain
store, which is all of the traffic and all of the exposure, still takes the
lock. Reentrancy makes the change *possible*, not affordable. It is also not
portable: the check needs to know whether *this* caller holds the bus, and
`core::sync`'s own rank tracker documents that a hosted `no_std` build and
threaded wasm have threads and no thread-local storage at all, so a thread-local
depth counter has nowhere to live on two supported targets. Evidence would have
to be carried on `MemAttrs` instead — `RequesterId` cannot supply it, as
`MemAttrs::exclusive` records — which is a bus-wide API change to buy a
9× slowdown.

**A third shape, refused for correctness rather than cost.** Commit the `SC`
with a compare-exchange of the word instead of a check followed by a store: the
cost lands on the `SC` rather than on the store path. It is unsound. An `SC`
must fail on *any* intervening store, and a compare-exchange succeeds when the
value was written and restored — the ABA case this file's "Implementation notes"
already names, and the axiom above says "no store", not "no change of value". It
would also need `RamStore` to be single-copy atomic first, and it cannot serve an
MMIO operand at all.

**So it stays open, deliberately, and the boundary is here rather than in three
places.** What is bought by leaving it is the store path: 12.7 ns and linear
scaling. What is paid is that a guest which races a plain store against another
master's atomic on the same granule can lose that store, under
`ThreadingMode::Parallel` only — no board in `machines/` selects it,
`Deterministic` round-robins on one host thread where the finest grain is a whole instruction,
and `Accel` never reaches either object because the host's silicon performs the
guest's read-modify-write. The one exception to the `Deterministic` claim is
`BusLock`'s: a lazily advanced device is caught up from inside the access that
dispatches to it, so a locked read-modify-write whose operand is *MMIO* can have
another master's write land inside it even on one thread. A RAM operand cannot.

Closing it is a design review, not a patch, and the number it has to beat is the
9× above.

**Kept, as of this round: barriers.** Every data barrier now executes one host
`fence(SeqCst)` — `DSB`/`DMB` in `cpu::arm::a64::exec`, `FENCE` in
`cpu::riscv::exec`, `LFENCE`/`MFENCE`/`SFENCE` in `cpu::x86::fpexec`, and
`IrHost::fence`'s default, which used to be an empty body under the comment
*"a no-op on a host with one thread of guest execution"*. What stays a no-op
stays one on an architectural reason rather than a convenient one: A64's `ISB`
and RISC-V's `FENCE.I` order their own PE's instruction *fetch* and no data
access another observer can see (Arm DDI 0487, `ISB`; RISC-V Zifencei), and
`cpu::arm::v7m`'s barriers are on a uniprocessor with no second observer to
order against.

`tests/memory_model_litmus.rs` is the reproducer, and what it found is not what
the argument above predicted.

- Over two bare relaxed `AtomicU8`s — the primitive the previous round measured
  — the store-buffer outcome appears tens to hundreds of times in 200 000
  rounds, and a `fence(SeqCst)` removes it. That much held.
- **Over `RamStore` it was already zero, before any of this.** Every write to
  the store ends in `mark_dirty`, which sets a bit in an `AtomicU64` — and it
  used to set that bit with an unconditional `fetch_or`, a *relaxed*
  read-modify-write, which on x86-64 is a `lock or`, and a locked instruction is
  a full barrier (*Intel SDM* volume 3 §9.2.5). So every guest store to RAM was
  draining the host's store buffer, on the interpreter and inside a translated
  block alike: `jit::Tlb::note_fast_store` marks the same bitmap after an
  inlined store. `tests/memory_model_costs.rs` prices the two identically —
  3.96 ns for a store plus a `SeqCst` fence, 3.97 ns for a store plus the
  `fetch_or`.
- Over guest instructions, two `cpu::x86` cores on two host threads: zero
  either way, for a second and independent reason — a whole interpreted
  instruction separates the guest's store from the guest's load, and the host's
  window closes at about forty nanoseconds.

None of that was a reason to leave the barrier a no-op. The accident was
x86-only (`fetch_or(Relaxed)` on AArch64 orders nothing), it covered only what
follows a store, and it did nothing between two loads. What it meant is that the
*exposure* was smaller than the previous round's measurement implied, and that
the honest reason to fence is portability rather than a live defect on this
host.

**And it is gone now, which is the cleanest possible demonstration of the
point.** That `lock or` was the largest single component of a guest store —
3.6 ns of 25.4 — bought for an ordering nobody had asked it for. `mark_dirty`
tests the bit before setting it, so in the steady state there is no locked
instruction; the litmus file's `RamStore` row has moved from **0** to tens per
200 000, level with the unfenced control, while the guest-instruction row stayed
at zero because the interpreter's own overhead was always what closed *that*
window. The guarantee is unaffected because the guarantee was never here: it is
`IrHost::fence`, `A64::host_fence` and `jit::x86::compile`'s `mfence`. The one
thing the accident really was propping up — a dirty log read while a vCPU runs —
is answered by the safe-point protocol instead, argued in full on
`RamStore::mark_dirty`.

**How often a real guest pays.** An arm64 Linux boot executes 479 000 `DSB`/`DMB`
in 876 million instructions — one in 1 800 — plus another 213 000 `ISB`, which
is why `ISB` is excluded rather than lumped in. A RISC-V Linux boot executes
`FENCE` about once in 100 000 instructions. An x86-64 Linux 6.6 guest executes
**`LFENCE` tens of thousands of times and `MFENCE` once**: `smp_mb()` on x86-64
is `lock addl $0,-4(%rsp)`, not `MFENCE`, so the barrier an x86 guest really
leans on is the `LOCK` prefix — which reaches `AddressSpace::bus_lock`, a mutex,
whose acquire/release did not forbid store-then-load *through* the critical
section, because acquire and release are each one-way. That gap was
`core::space`'s rather than `cpu/`'s and it is closed: the bus lock emits a
`SeqCst` fence once it is held and another before it is given back, which is
what makes a locked instruction the full barrier the SDM says it is. It could
not be gated by a test on an x86-64 host — the mutex's own `lock cmpxchg` masks
the difference — so it is gated by the architecture, and `core::space::BusLock`
carries the argument.

**And now it is gated by a test, on a host that can fail it.** CI grew an
`ubuntu-24.04-arm` job (`.github/workflows/ci.yml`, `aarch64`), and
`tests/memory_model_litmus.rs` grew the rows the three doc comments above said
belonged there: a whole `BusLock` transaction between the store and the load,
the *same transaction with the two fences deleted* beside it, and two
`cpu.arm.a64` cores running the litmus as guest code with `STLR`/`LDAR`, with
`DMB ISH`, and with neither. The fenced rows assert zero, which is sound
everywhere and therefore proves nothing on its own; what the weakly ordered
runner supplies is the unfenced half of each pair, which on x86-64 is also zero
and there says nothing at all. `RSEMU_WEAK_MEMORY_REQUIRED=1` is what stops the
job passing vacuously: it asserts that the bare-relaxed control reorders
something *and* that the relaxed `fetch_or` does too, the second being exactly
the accident this section is about — so a runner that quietly became x86-64
fails rather than reporting green. One row still has no test and cannot get one
on any runner: `jit::x86`'s `MFENCE` lowering is gated on
`target_arch = "x86_64"`, so on the only host where the row would mean anything
the backend does not exist.

**What the lifters do.** The RISC-V and x86 frontends do not lift a fence at
all — the block ends at one and the interpreter runs it. `a64::lift` does:
`classify`'s `Op::Dsb | Op::Dmb` arm produces one `Opcode::FENCE`, and
`jit::x86` lowers that to an `MFENCE` (*Intel SDM* volume 2B; `0F AE F0`).

That arm was `return None` until the lowering existed, and the reason was the
backend rather than the frontend: `jit::x86::compiles` refused `FENCE`, and
`jit::dispatch` does not *remember* a refusal — it re-attempts the compilation
every time the block is reached. Measured over the same 120 s of an arm64 Linux
boot on `jit-host`, `FENCE` emitted into the trace was **+4.9%**, `FENCE` alone
in a block of its own **+2.8%**, and leaving the barrier outside the lifted
subset — one dispatcher round trip and one interpreted instruction per barrier,
0.05% of the stream — **+1.3%**, inside the run-to-run noise. Every one of those
numbers is a *refusal* cost; none of them survives the backend accepting the op.
`a64::lift`'s `nothing_this_frontend_emits_is_an_op_the_host_backend_refuses` is
the standing invariant that made the order of the two changes matter.

**Why `MFENCE` and not nothing.** x86-TSO already gives store-store and
load-load ordering, so a guest barrier needing only those two would compile to
no instruction. `IrHost::fence`'s contract is a `SeqCst` host fence precisely
because that is the one ordering that also forbids **store-then-load**, which is
the reordering x86 does perform and the one the litmus test above measures.
`MFENCE` is the only one of the three x86 fence instructions that drains the
store buffer against a later load; `SFENCE` and `LFENCE` would be the no-op the
weakening amounts to. A64's `DSB`/`DMB` scope and type fields are not read: one
full fence is never weaker than the barrier the guest asked for, and narrowing
it is an optimisation whose soundness argument would have to be diffed against
the interpreter.

**What the compiled path does not do** is call `IrHost::fence`. Generated code
performs the barrier itself, the way an inlined load performs an access
`IrHost::load` would otherwise have made. The default body is a `SeqCst` host
fence, which on an x86-64 host *is* that instruction, and every `IrHost` in the
tree takes the default — so nothing observable differs today — but a host that wanted a guest barrier to *do*
something (count it, record it, replay it) would need a seam that does not exist
yet, and would have to be given one before it could rely on the call.

`jit::x86::compile`'s `plan` makes a fence a **region boundary**, which is the
one entry in that predicate that is not a call site. Without it the fence's
write to `Ctx::committed` would be overwritten by the replay of an `insn_start`
that architecturally precedes it — reporting a later `Retry` fault restartable
where `Interp` reports it not — and a barrier could be performed on behalf of a
guest instruction a spent tick allowance then unwound.
