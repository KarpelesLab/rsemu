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
happened at, and — for `cpu.arm.a64` and `cpu.x86`, which have field decoders —
the register that moved:

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
| `a_synthetic_riscv_workload_agrees_across_the_engines` | none | ~1.3 s | every `cargo test` |
| `a_tlbi_in_the_loop_agrees_across_the_engines` | none | ~0.4 s | every `cargo test` — see below for what it found |
| `a_synthetic_x86_workload_agrees_across_the_engines` | none | ~1.4 s | every `cargo test` |
| `the_guest_carries_the_addresses_and_the_windows_this_file_names` | none | ~0 s | every `cargo test` — the x86 guest's hand-assembled bytes against the constants that describe them |
| `a_real_arm64_linux_boot_agrees_across_the_engines` | a kernel | minutes | `--ignored`; nightly in CI |

```sh
# the whole target, as CI runs it on every commit
cargo test --release --test engine_longrun

# longer, locally
RSEMU_LONGRUN_SECONDS=30 cargo test --release --test engine_longrun

# the real gate
scripts/fetch-testdata.sh arm64-linux arm64-initramfs
scripts/check.sh long
```

`scripts/check.sh long` is the stage; [`.github/workflows/long-run.yml`] runs it
on a nightly schedule with `RSEMU_LONGRUN_REQUIRED=1`, which turns "no kernel,
skipping" into a failure the way `RSEMU_CROSSHOST_REQUIRED` does for the
crosshost job. It is **not** in `scripts/check.sh --all`: `--all` is what
somebody runs before a commit, and this wants a download and minutes of wall
time.

Nightly rather than per-pull-request is a judgement, and it is the one the fuzz
job already made for the same reason: a 36 MiB fetch and five minutes on one
board is not worth thirty times a day, and what this catches is a property of
the tree rather than of an individual commit. Per-commit coverage is the four
synthetic runs, which cost about five seconds of CPU and 3.7 s of wall time
together.

| Variable | Effect |
| --- | --- |
| `RSEMU_LONGRUN_SECONDS` | guest seconds to compare (2 for the A64 and RISC-V synthetics, 30 for the kernel, 120 in CI). The x86 leg is capped at 6 000 quanta instead when this is unset — see below — and setting it lifts the cap |
| `RSEMU_LONGRUN_ENGINES` | comma-separated; default `jit,jit-host`. `interp` is the control — an interpreter against itself must always agree |
| `RSEMU_ARM64_KERNEL`, `RSEMU_ARM64_INITRD` | the fixture, as in `tests/a64_linux.rs` |
| `RSEMU_ARM64_RAM` | the board's DRAM; default 512M |
| `RSEMU_LONGRUN_REQUIRED` | (`check.sh`) a missing kernel is a failure rather than a skip |
| `RSEMU_X86_LONGRUN_SEAMS` | the x86 leg's bisecting knob: a comma-separated subset of `timer,invlpg,smc,shadow,flags,int` to keep. Absent, everything |

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
past A64" below has the table, seam by seam. The same caveat applies to it in
the same words: it exercises what its author read off `engine.rs`, and there is
no x86 leg of the kernel gate. `pc64` boots a Linux kernel on either engine and
`docs/platforms/pc64.md` records nine hundred guest seconds of it, but nothing
runs that *in lockstep*, so a divergence there is still found by comparing final
hashes. Pointing this harness at `pc64` is the obvious next thing and it is not
done.

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
| the edge computed across an entry walk (`engine::leave_at`) | **passed** | passed | not reached in 40 s — but the synthetic **with** the `TLBI` **failed at quantum 417 — 0.417 s**, naming `elr_el1` `0x1014` against `0x4`, `pc` and `cycles` |

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
from the other direction. `engine::leave_at` is the fix.
[`docs/platforms/arm64-virt.md`](../platforms/arm64-virt.md) has the long form.

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
arm64 kernel:

| run | wall time |
| --- | --- |
| the whole `engine_longrun` target, default settings | 3.7 s (3.4 s of it without the x86 leg, which runs beside the others) |
| the x86 leg on its own, default settings | 1.4 s |
| the x86 leg, one guest second, each engine | 2.9 s |
| the x86 leg, twenty guest seconds, both engines | 117 s |
| 40 guest seconds, interpreter against `jit-host` | 43.2 s |
| 40 guest seconds, interpreter against `jit` | 60.4 s |
| **120 guest seconds, both engines — what the nightly runs** | **319 s** (179 s `jit`, 138 s `jit-host`) |

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
teeth, and the x86 leg is what that claim now looks like paid in full:

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
  to prevent.
* **a workload designed around the engine's seams.** `tests/engine_longrun.rs`'s
  RISC-V leg is still the plain RV64I loop the other tests use, and it still
  says so. The x86 one is not: it is written seam by seam against what
  `cpu::x86::engine` does, and each row of the table below is a paragraph of
  that file's own documentation turned into guest code.

| seam in `cpu::x86::engine` | what the x86 workload does about it |
| --- | --- |
| `MAX_INSNS` = 32, `CHAIN` = 16 | twenty-four consecutive lifted instructions, so an `advance` is a chain: 982 618 of 1 220 450 block entries in a guest second are chained |
| `FLAGS = Flags::Eager` | every one of those writes flags and five read them back — `ADC`, `SBB`, `SETB`, `CMOVZ` and the `Jcc`s |
| `IrHost::spent` | that run has no store in it, so nothing but the tick allowance can end a quantum inside it |
| `admit`'s exclusion list | `PUSHFQ`/`POPFQ` every eighth pass, `CLI`/`STI` every thirty-second — `STI` leaves the interrupt shadow `admit` refuses on |
| `Smc::EndBlock` | every sixteenth pass the guest **rewrites its own immediate**, from the same page it is executing: 106 058 translations thrown away against 106 089 made |
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
