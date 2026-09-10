# The long run: catching an engine divergence that needs seconds to appear

`ROADMAP.md` §0 asks for *a bit-identical state hash across the interpreter and
the JIT for the same guest*, and `CLAUDE.md` makes the interpreter the oracle.
Three tests assert that on every commit — [`tests/a64_engines.rs`],
[`tests/riscv_virt_engines.rs`] and [`tests/x86_engines.rs`] — and each of them
runs **forty quanta of a six-instruction loop**.

In September 2026 two defects in the A64 translating engine went through all
three, and through a twenty-second Linux boot as well. One first parted from
the interpreter at **15.04 s** of guest time; the other at **23.46 s**, where it
desynchronised the guest permanently.
[`docs/platforms/arm64-virt.md`](../platforms/arm64-virt.md) has the diagnosis.
Neither changed what an instruction computes; both changed *where a quantum
ends*, so `State::debt` and `ELR_EL1` were the columns that parted and a short
run could not see either. Both were found by hand, once, by somebody who
happened to look.

This page is the operating manual for the thing that now looks automatically.

## What it is

[`tests/longrun/mod.rs`] builds two machines from one description differing only
in `engine`, advances them **one quantum at a time in lockstep**, and compares
three things in order of cost:

| tier | every | what it covers | what it costs |
| --- | --- | --- | --- |
| clock | quantum | the two schedulers stand on the same instant | a comparison |
| device fingerprint | quantum | every device whose snapshot chunk is small enough to re-serialise — the CPU, the GIC, the UART, the virtio transports | one `save` per side, ~900 bytes on `arm64-virt` |
| full state hash | `hash_every` quanta, **and once at the end whatever the budget** | RAM, and any device too large for the tier above | a walk of all of RAM |

The device list is *probed*, not written down: each device is saved once at the
start and kept if its chunk is under 64 KiB, so RAM and framebuffers fall out by
their size and a board the file has never seen gets the right answer.

Failure names the first quantum on which anything parts, the guest time it
happened at, and — for `cpu.arm.a64`, `cpu.riscv` and `cpu.x86`, which have
field decoders — the register that moved:

```text
arm64-virt: engine=jit-host left the interpreter at quantum 23119, 23.118999 s of guest time.
    Device `cpu` (cpu.arm.a64):
        spsr_el1       0x00000000a0000005 interpreted   0x0000000040000005 translated
        elr_el1        0xffff8000803c9918 interpreted   0xffff8000803c9950 translated
        cntv_cval      0x0000000000e04a44 interpreted   0x0000000000e04a47 translated
```

That is the difference between this and a final-hash comparison. A final hash
says "something differs somewhere in 120 seconds", which is what the last round
had and spent a day bisecting. This says *which quantum, and which column*. It
also catches a divergence that **self-corrects** — the 15.04 s defect did, so a
run that only compared at the end would have finished on one hash with the
window closed unseen.

## Where it runs

| test | fixture | cost | when |
| --- | --- | --- | --- |
| `the_harness_names_the_quantum_a_planted_divergence_appears_on` | none | ~0.1 s | every `cargo test` |
| `a_synthetic_a64_workload_agrees_across_the_engines` | none | ~1.7 s | every `cargo test` |
| `a_synthetic_riscv_workload_agrees_across_the_engines` | none | ~3.1 s | every `cargo test` |
| `a_tlbi_in_the_loop_agrees_across_the_engines` | none | ~0.4 s | every `cargo test` — see below for what it found |
| `a_synthetic_x86_workload_agrees_across_the_engines` | none | ~1.4 s | every `cargo test` |
| `the_guest_carries_the_addresses_and_the_windows_this_file_names` | none | ~0 s | every `cargo test` — the x86 guest's hand-assembled bytes against the constants that describe them |
| `the_guest_carries_the_windows_and_the_addresses_this_file_names` | none | ~0 s | every `cargo test` — the same, for the RISC-V guest |
| `a_real_arm64_linux_boot_agrees_across_the_engines` | an `Image` | minutes | `--ignored`; nightly in CI |
| `a_real_x86_linux_boot_agrees_across_the_engines` | a `bzImage` | minutes | `--ignored`; nightly in CI |
| `the_clint_advances_while_the_hart_is_running` | none | ~0 s | every `cargo test` — a defect this file found, fixed in two halves; see below |

```sh
# the whole target, as CI runs it on every commit
cargo test --release --test engine_longrun

# longer, locally
RSEMU_LONGRUN_SECONDS=30 cargo test --release --test engine_longrun

# the real gate — two kernels now, one per core
scripts/fetch-testdata.sh arm64-linux arm64-initramfs x86-linux initramfs-x86
scripts/check.sh long
```

`scripts/check.sh long` is the stage; [`.github/workflows/long-run.yml`] runs it
on a nightly schedule with `RSEMU_LONGRUN_REQUIRED=1`, which turns "no kernel,
skipping" into a failure the way `RSEMU_CROSSHOST_REQUIRED` does for the
crosshost job. It is **not** in `scripts/check.sh --all`: `--all` is what
somebody runs before a commit, and this wants a download and minutes of wall
time.

Nightly rather than per-pull-request is a judgement, and it is the one the fuzz
job already made for the same reason: a 48 MiB fetch and a quarter of an hour
across two boards is not worth thirty times a day, and what this catches is a
property of the tree rather than of an individual commit. Per-commit coverage is
the four synthetic runs, which cost 3.4 s of wall time together because they run
beside one another.

| Variable | Effect |
| --- | --- |
| `RSEMU_LONGRUN_SECONDS` | guest seconds to compare (2 for the A64 and RISC-V synthetics, 30 for either kernel leg, 120 in CI). The x86 *synthetic* leg is capped at 6 000 quanta instead when this is unset — see below — and setting it lifts the cap |
| `RSEMU_LONGRUN_ENGINES` | comma-separated; default `jit,jit-host`. `interp` is the control — an interpreter against itself must always agree |
| `RSEMU_ARM64_KERNEL`, `RSEMU_ARM64_INITRD` | the AArch64 fixture, as in `tests/a64_linux.rs` |
| `RSEMU_ARM64_RAM` | that board's DRAM; default 512M |
| `RSEMU_X86_KERNEL`, `RSEMU_X86_INITRD` | the x86-64 fixture: a `bzImage` and an initramfs, as in `tests/pc64_linux.rs` |
| `RSEMU_X86_RAM` | `pc64`'s extended memory; default 256M |
| `RSEMU_X86_CMDLINE` | the kernel command line, replacing the one below. `nokaslr` is not optional on this board |
| `RSEMU_X86_LONGRUN_SECONDS` | (`check.sh`) the x86 kernel leg's own budget; default 900. **Not** `RSEMU_LONGRUN_SECONDS`, for the reason under "Two kernels, two budgets" |
| `RSEMU_LONGRUN_REQUIRED` | (`check.sh`) a missing kernel is a failure rather than a skip |
| `RSEMU_X86_LONGRUN_SEAMS` | the x86 synthetic leg's bisecting knob: a comma-separated subset of `timer,invlpg,smc,shadow,flags,int` to keep. Absent, everything |
| `RSEMU_RISCV_LONGRUN_SEAMS` | the same for the RISC-V leg: `timer,clint,sfence,csr,amo,ecall` |

## When the fixture is absent

The kernel test prints the two commands that would make it run and returns. It
does not fall back to something weaker while claiming to be the same test —
`docs/testing/README.md`'s rule for every corpus in this tree.

What still runs is the synthetic workload, and it is worth being precise about
what that is worth.

The synthetic A64 guest is *designed* around the two mechanisms that broke: the
MMU is on, its data window walks 64 pages so the software TLB is under real
pressure, it leaves its own page for an `MRS` the frontend does not lift, and
the generic timer fires inside a sixteen-instruction chain of lifted code. It
reaches both mechanisms in **under a second** of guest time, where the kernel
needed fifteen and twenty-three. That makes it an excellent *regression* test.

It is a poor *discovery* test, and that is the honest finding: it can only
exercise the mechanisms its author already thought of. The two September defects
were found because a real kernel does things nobody designed for, and a
synthetic guest that reached fifteen seconds of genuinely varied behaviour would
be a kernel. **The real gate only runs where the fixture exists.** Both exist
because they are different claims.

The x86 guest is designed the same way and against a longer list, because
`cpu::x86::engine` documents more seams than the A64 one does — "Generalising
past A64" below has the table, seam by seam. The RISC-V guest is now the same
kind of thing on a third core. The same caveat applies to all three in the same
words: each exercises what its author read off `engine.rs`.

**There is an x86 leg of the kernel gate now**, and it is the answer to the
paragraph that used to stand here. `pc64` with a stock `bzImage` in its slot,
both engines, quantum by quantum — the section after next has what it costs and
what it caught.

## Two kernels, two budgets

`RSEMU_LONGRUN_SECONDS` drives the AArch64 gate and `RSEMU_X86_LONGRUN_SECONDS`
drives the x86 one, and that is deliberate rather than untidy. A guest second
does not mean the same thing on the two boards:

* `arm64-virt` runs a 1 GHz core and reaches a shell inside twenty guest
  seconds. 120 is well past both defects the gate exists for.
* `pc64` runs a **100 MHz** processor and has no firmware, so the `bzImage`
  decompresses itself from the reset vector. Measured on the Debian installer
  kernel: at 120 guest seconds the last thing the guest has printed is
  `KASLR disabled: 'nokaslr' on cmdline` and it is still in the decompressor;
  `Linux version` does not appear until somewhere between 400 and 600.

So `RSEMU_LONGRUN_SECONDS=120` on both would have made the x86 leg a test of
`REP MOVS` and nothing else. **900** is the default in `scripts/check.sh`, and
that number is measured rather than chosen: the calibration below re-introduced
a defect that needs an `STI`, and the first `STI` a Linux kernel executes on
this board is at 635 guest seconds. The nightly's `workflow_dispatch` can raise
either budget independently.

## The instrument is calibrated

A test that cannot fail is worse than no test, so the harness was made to fail
on purpose, three ways.

`the_harness_names_the_quantum_a_planted_divergence_appears_on` runs on every
commit and plants one divergence per tier: a system register nothing reads
(caught on the quantum it was planted on, and named), a word in a RAM page the
guest never touches (caught by the periodic hash and *only* then, which is what
says both tiers are load-bearing), and a machine one quantum ahead (reported as
a clock divergence, not as a state one).

And the two defects this exists for were re-introduced, one at a time, in a
scratch tree:

| defect re-introduced | `tests/a64_engines.rs` | synthetic (no `TLBI`) | kernel gate |
| --- | --- | --- | --- |
| the block that did not notice its own timer (`IrHost::spent`) | **passed** | **failed at quantum 17 — 0.017 s**, naming `elr_el1` `0x100c` against `0x4` | **failed at quantum 23119 — 23.119 s**, naming `elr_el1` and `spsr_el1` |
| the declined chained boundary (`advance`'s `Stop::Declined` arm) | **passed** | passed | **failed at quantum 14097 — 14.097 s**, naming `debt` 3 against 2, `pc` and `cycles` |
| the edge computed across an entry walk (then `engine::leave_at`, now `Admitted::leave`) | **passed** | passed | not reached in 40 s — but the synthetic **with** the `TLBI` **failed at quantum 417 — 0.417 s**, naming `elr_el1` `0x1014` against `0x4`, `pc` and `cycles` |

The old test passes in all three rows, which is the claim the last round made
and it is correct. The second row is why the kernel run cannot be replaced: a declined
chained boundary needs a **cold instruction-fetch translation**, and
`mmu::Tlb` keeps fetch, load and store entries in three separate 256-entry sets
— so no amount of data-side pressure evicts a code page's fetch entry, and the
only ways to get one are a `TLBI` or a guest that executes from 257 pages.

### The same, on x86

The x86 leg was calibrated the same way, against `tests/x86_engines.rs` — which
is the per-commit test it has to be worth more than. Four defects, one at a
time, each a line of `src/cpu/x86/engine.rs` reverted afterwards:

| defect re-introduced | `tests/x86_engines.rs` | `a_synthetic_x86_workload…` |
| --- | --- | --- |
| `admit` stops refusing on `State::int_shadow`, so a block may be entered inside an `STI` shadow | **passed** — 5 tests, all green | **failed at quantum 2**, naming `int shadow` `0` interpreted against `1` translated |
| `FLAGS` back to `Flags::Elide` | failed, 4 of 5 | **failed at quantum 1**, naming `eflags` `0x217` against `0x206` |
| `close_bus` removed — a block makes no fetches, so it leaves whatever its last data access put on the bus | failed, 4 of 5 | **failed at quantum 1**, naming `open bus` `0xcc` against `0xff` |
| `IrHost::spent` compares `>` rather than `>=`, so a block leaves one instruction late | failed, 4 of 5 | **failed at quantum 4**, naming `eip`/`rip` `0xa0c8` against `0xa0ce`, `cycles`, `open bus` and **`debt` 0 against 7** |

The first row is the one this leg exists for, and it is the x86 analogue of the
A64 table's second: `tests/x86_engines.rs` runs a loop with no `STI` in it, so
the interrupt shadow it would need never exists and the test is green on a core
that enters blocks inside one. The workload here has a two-instruction `CLI`
window every thirty-second pass precisely because `admit`'s refusal list says
`int_shadow` — and running it again with `RSEMU_X86_LONGRUN_SEAMS` naming every
seam *but* `shadow` passes for its whole six thousand quanta, which is the
negative that says the `STI` is what catches it rather than anything else in the
loop.

The last row is the shape both September A64 defects had — nothing computes a
different answer, a quantum ends one instruction later — and the columns it
names are the same ones: the program counter, the cycle count and `State::debt`.

### And on a real x86-64 kernel

The `pc64` leg was calibrated against the synthetic one above, which is the test
*it* has to be worth more than. The defect re-introduced is the first row of
that table — `admit` stops refusing on `State::int_shadow` — because it is the
one the synthetic leg exists for and the one `tests/x86_engines.rs` cannot see
at all.

| | verdict |
| --- | --- |
| `tests/x86_engines.rs` | **passed** — 5 tests, all green, 0.43 s |
| `a_synthetic_x86_workload…` | **failed at quantum 2**, 0.001005 s, naming `int shadow` `0` interpreted against `1` translated |
| `pc64`, 30 guest seconds | passed — the decompressor runs with interrupts off and never executes an `STI` |
| `pc64`, 400 guest seconds | passed — still in the decompressor |
| `pc64`, 600 guest seconds | passed — the kernel is only at `[    0.000000] NR_IRQS`, and everything up to `local_irq_enable` runs with interrupts off, so it has still not executed an `STI` |
| **`pc64`, 900 guest seconds** | **failed at quantum 1 280 242 — 635.040039 s of guest time**, naming `int shadow` `0` interpreted against `1` translated |

That table is the honest shape of the trade and it is worth reading in both
directions. The synthetic leg finds this defect in a millisecond of guest time
and the kernel needs minutes of wall clock to reach the first `STI` — so for a
mechanism somebody has already thought of, the designed workload wins by three
orders of magnitude, exactly as the A64 table says. What the kernel leg buys is
the other half: it is running instructions, and reaching states, that nobody
wrote down.

That 635-second figure is why `scripts/check.sh`'s default for this leg is
**900** and not the 600 it was first written with. The budget is not a taste;
it is the measured distance to the first `STI` a Linux kernel executes on a
board with no firmware, and a gate set below it would have been green on a core
that enters blocks inside an interrupt shadow.

**On `master` it has found nothing.** Nine hundred guest seconds of that kernel
— 1 946 548 quanta — agree quantum for quantum against `jit-host`, and six
hundred agree against `jit` and `jit-host` both. The two machines also printed
byte-identical console output, which the lockstep loop cannot see for itself: a
drained console is a host object rather than device state, so only a byte still
sitting in the 16550's transmitter is in the per-quantum fingerprint. The test
compares the drained text as a fourth tier and asserts the engine under test
retired more instructions inside blocks than outside — 1.52 billion against 6.5
million at 900 seconds — because a boot that agreed at every checkpoint says
nothing if the guest stopped early, and two processors stopped in the same place
agree on every hash they are asked for.

### And on the RISC-V board

Same method, against `tests/riscv_virt_engines.rs`:

| defect re-introduced | `tests/riscv_virt_engines.rs` | `a_synthetic_riscv_workload…` |
| --- | --- | --- |
| `IrHost::spent` compares `>` rather than `>=`, so a block leaves one instruction late | **passed** — 3 tests, all green | **failed at quantum 4 — 0.003999 s**, naming `pc` `0x80001184` against `0x80001188`, `cycles`, **`debt` 0 against 2**, `minstret` and `mcycle` |

It is the same row as the last of the x86 table and the same shape as both
September A64 defects — nothing computes a different answer, a quantum ends one
instruction later — and it names the same columns: the program counter, the
cycle count and `State::debt`.

That run is also why `tests/longrun/mod.rs` has a **third** field decoder now.
The first time it was made to fail it said *"first difference at byte 512, as a
big-endian word 0x8411008000000000 against 0x8811008000000000"* — the program
counter, one instruction apart, in the wrong byte order. Correct, useless, and
exactly the weakness the fallback is documented to have. `decode_riscv` reads
`Hart::save`'s chunk field by field and names the integer registers the way the
guest's own assembly names them, so a report says `s2 (x18)` rather than sending
the reader to chapter 25 of the manual.

## What it found on its first run: the generic timer across a `TLBI`

`a_tlbi_in_the_loop_agrees_across_the_engines` was committed `#[ignore]`d,
because it failed on the `master` it was committed to. It is now un-`#[ignore]`d
and it is the regression test: the ledger has shrunk by one. This is the third
instance of the class the two September defects belong to, and the first one
that a machine rather than a person found.

What it reported:

* At quantum 417 (0.417 s of guest time, the 52nd timer interrupt) the
  interpreter has `ELR_EL1 = 0x1014` and both translated engines have `0x4`:
  the translated core ran two more guest instructions before noticing a timer
  the interpreter took immediately, and carried one extra cycle for it.
* It needs the generic timer **and** the `TLBI`. The same workload with either
  one alone agrees for six thousand quanta.
* `DSB`, `ISB` and `DSB; ISB` in the same slots are all fine, so the instruction
  is `TLBI` and not the barriers around it.
* `jit` and `jit-host` produce the *same* wrong answer, with only the
  interpreter on the other side — the signature of a frontend defect rather
  than a code-generator one.
* A 40-second `arm64-virt` Linux boot does **not** hit it, which is why it is
  worth having a synthetic workload as well as a kernel: Linux does `TLBI`
  constantly, but a timer edge has to land on one, and a loop that flushes on
  half its passes gets there in half a second.

Every one of those five lines survived being checked, and together they name
the defect almost exactly. `engine::admit` asks `Exec::pending_interrupt` and
*then* charges the entry translation, which on a TLB miss is a walk; the generic
timer's count is this core's own tick counter divided down, so the walk can
cross a comparator the check a moment before found un-crossed. `Exec::timer_edge`
reports `u64::MAX` for a comparator already crossed — correct for its own
question, since an asserting output cannot rise again, and the wrong edge for a
run, which had to leave at its next boundary rather than never. It takes a cold
*instruction-fetch* translation to open the window, and `mmu::Tlb`'s three
separate sets mean a `TLBI` is the only thing on this core that produces one:
the same fact the table above gives for the declined-boundary defect, arrived at
from the other direction. `engine::leave_at` was the fix; the question has
since moved into `admit` as `Admitted::leave`, which also covers a *chained*
block's entry translation and a line a walk's **reads** raise rather than its
ticks. [`docs/platforms/arm64-virt.md`](../platforms/arm64-virt.md) has the
long form.

The instructive part is the bisect, not the fix. Five properties, each cheap to
test by re-running the same harness with one thing changed, took a
`0x03b29d3a…`-against-`0x7d0edfc4…` hash mismatch down to a named function
before anybody read a line of `engine.rs` — and the one that mattered most was
the negative: `DSB`, `ISB` and both together are *fine*. That is what says the
instruction is doing something to the translation regime rather than to the
ordering, and it is the difference between reading `timer_edge` and reading
`admit`.

## Cost, measured

On a 64-core Zen host, `--release`, `arm64-virt` with 512 MiB and a Debian
arm64 kernel, `pc64` with 256 MiB and a Debian amd64 one:

| run | wall time |
| --- | --- |
| the whole `engine_longrun` target, default settings | 3.4 s (the four synthetic legs run beside each other) |
| the synthetic x86 leg on its own, default settings | 1.4 s |
| the synthetic x86 leg, one guest second, each engine | 2.9 s |
| the synthetic x86 leg, twenty guest seconds, both engines | 117 s |
| the synthetic RISC-V leg on its own, default settings (2 guest seconds) | 3.7 s (1.9 s `jit`, 1.5 s `jit-host`) |
| `arm64-virt`, 40 guest seconds, interpreter against `jit-host` | 43.2 s |
| `arm64-virt`, 40 guest seconds, interpreter against `jit` | 60.4 s |
| **`arm64-virt`, 120 guest seconds, both engines — what the nightly runs** | **319 s** (179 s `jit`, 138 s `jit-host`) |
| `pc64`, 300 guest seconds, interpreter against `jit-host` | 131.6 s |
| `pc64`, 900 guest seconds, interpreter against `jit-host` | 413.0 s |
| `pc64`, 600 guest seconds, both engines | 567 s (323 s `jit`, 244 s `jit-host`) |
| **`pc64`, 900 guest seconds, both engines — what the nightly runs** | **about 16 minutes** |
| `scripts/check.sh long`'s synthetic half, 30 guest seconds each | 165 s — unchanged by the new RISC-V board, which runs beside the x86 leg that is the critical path |

`pc64` is about 2 016 quanta per guest second against `arm64-virt`'s 1 000 and
the synthetic x86 board's 24 818, and 900 guest seconds of it is 1 946 548
quanta, 292 million translated block entries and 1.52 **billion** instructions
retired inside them against 6.5 million interpreted — 99.6% of the stream. That
is the number that says what a lockstep kernel run is worth: it is not a test of
the interpreter with a JIT beside it, it is 1.5 billion instructions of compiled
code checked one quantum at a time against the oracle.

The x86 leg is the one that is **capped in quanta rather than in guest
seconds**, and the reason is in the second row of that table: one guest second
of that board is 24 818 quanta and about three seconds of wall time per engine,
which is twice what the rest of the target costs put together, and effectively
all of it is repetition — the workload reaches every seam it was written for
inside the first two hundred quanta. Six thousand quanta is 0.24 s of guest
time, 46 000 passes round the loop, 2 900 code rewrites, 700 `INVLPG`s and 2 800
timer interrupts. `RSEMU_LONGRUN_SECONDS` removes the cap, which is what
`scripts/check.sh long` and the nightly do.

The interpreter is the floor in every lockstep row — it is roughly 0.95 s of
wall time per guest second on this host on its own — so the comparison costs
about what the oracle costs, and the per-quantum fingerprint is lost in the
noise beside it. `hash_every` is the knob that is *not* free: the full hash
walks all of RAM, so the kernel run takes one every 20 000 quanta and the
synthetics every 2 000.

## Generalising past A64

The harness is core-agnostic already: it talks to `Machine`, `DeviceEntry` and
`StateWriter` and knows nothing about any instruction set. Pointing it at
`riscv-virt` cost forty lines and pointing it at x86 cost about the same.

What does **not** generalise for free is the two things that give a run its
teeth. The x86 and RISC-V legs are what that claim looks like paid in full,
twice:

* **a field decoder.** `decode_a64` turns a chunk into named registers, which is
  what makes a failure say `debt 3 against 2` instead of `first difference at
  byte 812`. Each core needs its own, matched to its `save`; the fallback is a
  byte offset and is much weaker. `decode_x86` is the second one. `cpu.x86`'s
  chunk is a prefix and four appended blocks — the gdb i386 core block, then
  long mode, then floating point, then the multiprocessor state — with a
  **length-prefixed prefetch queue** in the middle of it, so nothing after that
  queue can be read at a fixed offset and a `ChunkReader` is the only way in.
  It names the 32-bit views and the 64-bit ones both, because `save` writes
  both: a divergence in the upper half of a register shows as `rax` with no
  `eax` beside it, which is exactly what `engine::narrow_state_is_clean` exists
  to prevent. `decode_riscv` is the third, and its own chunk moves for a
  different reason again — the exclusive reservation is an `Option`. It names
  the integer registers by their ABI names with the number beside them, `s2
  (x18)`, because that is how the guest's own assembly names them and a report
  that says only `x18` sends the reader to chapter 25 of the manual.
* **a workload designed around the engine's seams.** RISC-V and x86 share the
  first of A64's seams — a boundary the frontend declines — and **not** the
  second: neither core has a timer of its own counted off its own cycle
  counter, so nothing on either can cross a comparator inside `admit`'s entry
  walk the way A64's generic timer can. What they have instead is a *device* on
  the other side of a load. The CLINT, the local APIC and the HPET are all
  lazily advanced, so a guest load catches the chip up to the core's live
  position and a comparator crossed there raises the wire **between two
  instructions of a lifted block** — where the interpreter would have taken the
  trap at the next one. Both cores had that divergence and both are fixed in
  `IrHost::load`; `docs/platforms/riscv-virt.md` and `docs/platforms/pc64.md`
  have the diagnosis, and the regression tests live beside the engines rather
  than here because each needs a device that raises on read.

  The RISC-V leg **is** that board now — "The RISC-V board" below — and
  building it turned up why the seam above cannot be reached from a guest on
  `riscv-virt` at all. Neither leg is a plain loop any more: each is written
  seam by seam against what its `engine.rs` does, and each row below is a
  paragraph of that file's own documentation turned into guest code.

| seam in `cpu::x86::engine` | what the x86 workload does about it |
| --- | --- |
| `MAX_INSNS` = 32, `CHAIN` = 16 | twenty-four consecutive lifted instructions, so an `advance` is a chain: 982 618 of 1 220 450 block entries in a guest second are chained |
| `FLAGS = Flags::Eager` | every one of those writes flags and five read them back — `ADC`, `SBB`, `SETB`, `CMOVZ` and the `Jcc`s |
| `IrHost::spent` | that run has no store in it, so nothing but the tick allowance can end a quantum inside it |
| `admit`'s exclusion list | `PUSHFQ`/`POPFQ` every eighth pass, `CLI`/`STI` every thirty-second — `STI` leaves the interrupt shadow `admit` refuses on |
| `Smc::HostGuard` | every sixteenth pass the guest **rewrites its own immediate**, from the same page it is executing: 106 058 translations thrown away against 106 089 made. (The counts were taken while the policy under paging was still `Smc::EndBlock`; what the seam is called changed, what the guest does about it did not.) |
| the entry translation in `admit` | `INVLPG` on the running code page every sixty-fourth pass — the x86 analogue of the `TLBI` above, and `paging::Buffers::Split` means it is the only thing that cools a fetch translation |
| an interrupt from inside a chain | an 8254 into a master 8259A into `INTR`, at 83.8 µs against a quantum of at most 100 µs: 11 922 interrupts in a guest second |
| a synchronous entry from inside a chain | `INT 0x30` every two hundred and fifty-sixth pass |
| the data-side walk and its accessed and dirty bits | a store and a load to a different 4 KiB page every pass, sixty-four of them |

98.7% of that guest's instructions retire inside a block, so what is being
compared really is a translated core.

**It has found nothing on `master`.** Twenty guest seconds — 3.85 M passes round
the loop, 240 000 timer interrupts, 240 000 code rewrites and 60 000 `INVLPG`s —
agree quantum for quantum under both `jit` and `jit-host`. That is the honest
result and it is worth writing down beside the A64 leg's, which failed the first
time it was run.

### The x86 board is built here, and why it is not `pc64`

Every shipped x86 board is a board for software this repository does not contain
— `pc-at` and `q35` want a firmware, `pc64` and `q35-linux` want a `bzImage` —
and all four start in **real mode**, which `lift::World::of` refuses by
construction. A board that spent its first hundred thousand instructions there
would compare two interpreters and pass. `tests/x86_engines.rs` already makes
that argument and builds its own machine; this leg builds the same machine plus
the one thing that file deliberately has not got, an **asynchronous interrupt
source**, because the seam under test is a timer edge arriving in the middle of
a chain and a machine with nothing that interrupts cannot reach it. The chips
are `pc64`'s own, at `pc64`'s own clock rates, so a count means the same thing
on both.

## The RISC-V board

The old leg was `tests/riscv_virt_engines.rs`'s twelve-instruction RV64I loop
pointed at this harness, and both previous rounds said so in its own doc
comment. It is now a board: **Sv39 paging over four-kibibyte identity pages**, a
machine-mode trap handler, the CLINT arming its own comparator, and a
supervisor-mode loop on a page of its own. Machine-mode setup and the handler
are at `0x80000000`; the loop is one page above it, so `SFENCE.VMA` cools
*its* fetch translation and leaves the handler's alone.

| seam in `cpu::riscv::engine` | what the RISC-V workload does about it |
| --- | --- |
| `lift::MAX_INSNS` = 64, `CHAIN` = 16 | sixty-four consecutive lifted ALU instructions, so one `advance` is a chain: 6.07 block entries per pass |
| *"a store still ends the block"* | a store and a load to a different 4 KiB page every pass, sixty-four of them |
| `IrHost::load` and a **lazily-advanced device** | `ld` of the CLINT's `mtime` from inside a lifted block, every pass |
| `admit`'s exclusion list | `SFENCE.VMA` every sixteenth pass, a supervisor CSR round trip every eighth, `amoadd.d` every thirty-second, `ECALL` every two hundred and fifty-sixth — every one outside the lifted subset, so every one a declined boundary |
| the entry translation in `admit` | that same `SFENCE.VMA`, the only thing on this hart that cools a **fetch** translation |
| the data-side walk, its accessed and dirty bits | the Sv39 tables leave `A` and `D` clear, so the first touch of each page after each flush takes the walk's update |

`mtime` is folded into the first of the sixty-four ALU instructions on purpose:
a divergence about *when* a block noticed the timer becomes an arithmetic
difference in a named register on the next pass, rather than something only
`debt` and `pc` carry.

Measured over two guest seconds: 117 053 passes round the loop and 710 140 block
entries, so 7 315 `SFENCE.VMA`s, 14 631 CSR round trips, 3 657 atomics and 457
`ECALL`s, plus 1 999 timer interrupts — identical counts under `jit` and
`jit-host`, and under the interpreter that is the oracle for both.

**It has found nothing on `master`** — under `jit` and `jit-host` alike, quantum
for quantum. What it *did* find is a floor under the seam it was written for,
and that is the next section.

### What the RISC-V board found: `mtime` did not move while the hart ran

`the_clint_advances_while_the_hart_is_running` was committed `#[ignore]`d
because it **failed on the `master` it was committed to**, the way
`a_tlbi_in_the_loop_agrees_across_the_engines` did. It is not an engine
divergence — both engines agreed — which is why it was filed rather than fixed
where it was found. It is green now, and both halves of the fix are worth
recording because the shape recurs.

A guest reads `mtime` in a tight loop and counts distinct values. Over eight
quanta it should see several hundred: a hart on this board gets
`SchedulerConfig::max_ticks_per_quantum` — ten thousand — of a 1 GHz domain per
round, which is 10 µs, and `mtime` counts at 10 MHz. **It saw seven. One per
quantum.** It sees **800** now, which is the hundred a round predicts.

`riscv.clint` is a lazily-advanced device and its `Registers::read` calls `sync`
before answering, precisely so a guest load catches the chip up to the core's
live position. Two separate things stopped that from happening, and the fix
needed both:

* **The hart published nothing.** `Scheduler::arm_live_cursors` builds each
  lazy device's live view on the running runnable's `TickCursor`, and
  `Hart::attach_cursor` used to keep only that cursor's **exit flag** and drop
  the position half — saying so in as many words: *"this hart does not publish
  its own position — nothing on a RISC-V board here is sampled inside an
  instruction the way a PPU is"*. It keeps both now, and
  `Exec::publish_position` publishes `State::cycles` before every access that
  leaves for the address space.
* **The CLINT is on a second crystal.** `machines/riscv-virt.machine` hangs
  `mtime` off `osc rtc`, and `arm_live_cursors` used to arm a live view only
  across slots sharing a root — so the CLINT's slot was skipped whatever the
  hart published. `Live` now carries a ratio built from the two domains'
  declared rational frequencies where the two trees differ, which is what
  `ROADMAP.md` §4.2 prescribes for independent crystals: reciprocal multiply
  plus a per-root residual, error below one tick and non-accumulating because
  the base is re-anchored from the forest every round. The intra-tree path is
  untouched, and
  `core::sched::tests::an_intra_tree_ratio_is_still_exact_with_another_crystal_present`
  is that claim's own gate.

Two consequences followed from the defect, and the second is why it mattered
here rather than only as a clock-resolution nit:

* `rdtime` and `mtime` were quantised to the scheduler's grid — a millisecond —
  on every RISC-V board in this tree.
* The seam `cpu::riscv::engine` documents at length was **unreachable from a
  guest**. Its window is a comparator crossed inside a running round, and on
  that board a comparator was only ever crossed in `close_round`, which is a
  quantum boundary and where both engines agree by construction. The workload
  above took **1 999 timer interrupts in 2 000 quanta** — exactly one each,
  with the `clint` seam on and with it off alike, which is the same statement
  from the other side.

So `engine::tests::a_load_that_raises_an_interrupt_is_taken_where_the_interpreter_takes_it`,
which builds a device that raises unconditionally, was the **only** coverage
that seam had, and no guest on a shipped RISC-V board could reach it. Worth
knowing before somebody reads `IrHost::load`'s hand-back as dead code — and
worth re-measuring now that a comparator can be crossed mid-round.

**The lesson worth keeping is about the test, not the defect.** Its assertion
was written against the fixed behaviour rather than against the bug, so it
turned green on the commit that landed the second half and needed no edit. An
`#[ignore]`d test that asserts the *current wrong* number has to be rewritten by
the person who fixes it, which is exactly when nobody wants to be arguing about
what the right number was.

### Bisecting a RISC-V failure

`RSEMU_RISCV_LONGRUN_SEAMS` is the x86 knob on the second core, and it works the
same way — the seams named are kept and the rest are replaced by `addi x0, x0,
0` **in place**, so every branch displacement in the loop is untouched and a
bisect changes one property at a time.

```sh
# everything but the SFENCE.VMA
RSEMU_RISCV_LONGRUN_SEAMS=timer,clint,csr,amo,ecall \
    cargo test --release --test engine_longrun -- riscv

# the timer alone, with no CLINT read and nothing outside the lifted subset
RSEMU_RISCV_LONGRUN_SEAMS=timer cargo test --release --test engine_longrun -- riscv
```

### Bisecting an x86 failure

`RSEMU_X86_LONGRUN_SEAMS` names the seams to keep and turns the rest into
`NOP`s in place, leaving every branch displacement alone. It is what produced
the negative in the calibration table above, and it is the same move the A64
leg's `DSB`/`ISB` controls made:

```sh
# everything but the STI window
RSEMU_X86_LONGRUN_SEAMS=timer,invlpg,smc,flags,int \
    cargo test --release --test engine_longrun -- x86

# the timer alone, with no self-modifying code and no INVLPG
RSEMU_X86_LONGRUN_SEAMS=timer cargo test --release --test engine_longrun -- x86
```

[`tests/a64_engines.rs`]: ../../tests/a64_engines.rs
[`tests/riscv_virt_engines.rs`]: ../../tests/riscv_virt_engines.rs
[`tests/x86_engines.rs`]: ../../tests/x86_engines.rs
[`tests/longrun/mod.rs`]: ../../tests/longrun/mod.rs
[`.github/workflows/long-run.yml`]: ../../.github/workflows/long-run.yml
