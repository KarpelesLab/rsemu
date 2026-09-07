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
happened at, and — for `cpu.arm.a64`, which has a field decoder — the register
that moved:

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
| `a_tlbi_in_the_loop_agrees_across_the_engines` | none | ~0.4 s | `--ignored` — **a known defect**, see below |
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
the tree rather than of an individual commit. Per-commit coverage is the two
synthetic runs, which cost about three seconds together.

| Variable | Effect |
| --- | --- |
| `RSEMU_LONGRUN_SECONDS` | guest seconds to compare (2 for the synthetics, 30 for the kernel, 120 in CI) |
| `RSEMU_LONGRUN_ENGINES` | comma-separated; default `jit,jit-host`. `interp` is the control — an interpreter against itself must always agree |
| `RSEMU_ARM64_KERNEL`, `RSEMU_ARM64_INITRD` | the fixture, as in `tests/a64_linux.rs` |
| `RSEMU_ARM64_RAM` | the board's DRAM; default 512M |
| `RSEMU_LONGRUN_REQUIRED` | (`check.sh`) a missing kernel is a failure rather than a skip |

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

The old test passes in both rows, which is the claim the last round made and it
is correct. The second row is why the kernel run cannot be replaced: a declined
chained boundary needs a **cold instruction-fetch translation**, and
`mmu::Tlb` keeps fetch, load and store entries in three separate 256-entry sets
— so no amount of data-side pressure evicts a code page's fetch entry, and the
only ways to get one are a `TLBI` or a guest that executes from 257 pages.

## Known defect: the generic timer across a `TLBI`

`a_tlbi_in_the_loop_agrees_across_the_engines` is `#[ignore]`d because it fails
today, on unmodified `master`. This harness found it the first time it was
pointed at anything, and it is a third instance of the class the two September
defects belong to.

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

Un-`#[ignore]` that test the day it is fixed; it is then the regression test,
and the ledger has shrunk by one.

## Cost, measured

On a 64-core Zen host, `--release`, `arm64-virt` with 512 MiB and a Debian
arm64 kernel:

| run | wall time |
| --- | --- |
| the whole `engine_longrun` target, default settings | 3.0 s |
| 40 guest seconds, interpreter against `jit-host` | 43.2 s |
| 40 guest seconds, interpreter against `jit` | 60.4 s |
| **120 guest seconds, both engines — what the nightly runs** | **319 s** (179 s `jit`, 138 s `jit-host`) |

The interpreter is the floor in every lockstep row — it is roughly 0.95 s of
wall time per guest second on this host on its own — so the comparison costs
about what the oracle costs, and the per-quantum fingerprint is lost in the
noise beside it. `hash_every` is the knob that is *not* free: the full hash
walks all of RAM, so the kernel run takes one every 20 000 quanta and the
synthetics every 2 000.

## Generalising past A64

The harness is core-agnostic already: it talks to `Machine`, `DeviceEntry` and
`StateWriter` and knows nothing about any instruction set. Pointing it at
`riscv-virt` cost forty lines, and pointing it at x86 would cost about the same
(the board `tests/x86_engines.rs` builds, lifted into a `board(engine, tag)`).

What does **not** generalise for free is the two things that give the A64 run
its teeth:

* **a field decoder.** `decode_a64` turns a chunk into named registers, which is
  what makes a failure say `debt 3 against 2` instead of `first difference at
  byte 812`. Each core needs its own, matched to its `save`; the fallback is a
  byte offset and is much weaker.
* **a workload designed around the engine's seams.** RISC-V and x86 have the
  same two seams — a boundary the frontend declines, and a per-core timer
  reached from inside a block — and nobody has yet written the guest that
  provokes them. `tests/engine_longrun.rs`'s RISC-V leg is the plain RV64I loop
  the other tests use, and it says so.

[`tests/a64_engines.rs`]: ../../tests/a64_engines.rs
[`tests/riscv_virt_engines.rs`]: ../../tests/riscv_virt_engines.rs
[`tests/x86_engines.rs`]: ../../tests/x86_engines.rs
[`tests/longrun/mod.rs`]: ../../tests/longrun/mod.rs
[`.github/workflows/long-run.yml`]: ../../.github/workflows/long-run.yml
