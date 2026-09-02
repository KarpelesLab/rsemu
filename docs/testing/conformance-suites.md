# Conformance suites and test corpora

Consumed by: every CPU core and every machine. `ROADMAP.md` §0 says accuracy is
measured, never asserted — this file is the list of things that measure it.

## The rule: run them, never vendor them

Test corpora are **downloaded at test time into a git-ignored directory and
never committed** (`CLAUDE.md`, Testing). Two independent reasons:

1. **Licensing.** Several suites are copyleft and some have no licence at all.
   *Executing* a GPL binary as an emulated guest is ordinary use and creates no
   derivative work; *shipping* it in this repository is redistribution under its
   terms.
2. **Size.** Full instruction-level corpora run to gigabytes.

The suites are gated behind an environment variable so `cargo test` works
offline with no corpus present.

## Running them

[`README.md`](README.md) in this directory is the operating manual: the fetch
script, the environment variables, the bring-up order, and what each gate
requires. The harness itself is [`tests/conformance/`](../../tests/conformance),
and three of the suites below have runners today — SingleStepTests 65x02,
`nestest`, and AccuracyCoin.

## Licences — verified

Checked against each project's own licence file. **Re-verify before vendoring
anything**; this table records a point in time.

| Suite | Covers | Licence | May we vendor? |
| --- | --- | --- | --- |
| [SingleStepTests/65x02](https://github.com/SingleStepTests/65x02) | 6502/65C02, 10 000 vectors per opcode with full bus-cycle traces | **MIT** ✅ | Yes, with attribution |
| [SingleStepTests/z80](https://github.com/SingleStepTests/z80) | Z80, same format | **MIT** ✅ | Yes, with attribution |
| [SingleStepTests/8088](https://github.com/SingleStepTests/8088) | 8088/8086, same format | **MIT** ✅ | Yes, with attribution |
| [Gekkio/mooneye-test-suite](https://github.com/Gekkio/mooneye-test-suite) | Game Boy acceptance tests — timing, PPU, interrupts | **MIT** ✅ | Yes, with attribution |
| [riscv-tests](https://github.com/riscv-software-src/riscv-tests) | RISC-V ISA tests | **BSD-3-Clause** (UC Regents) ✅ | Yes, with attribution |
| [riscv-arch-test](https://github.com/riscv-non-isa/riscv-arch-test) | RISC-V architectural certification — one signature per test, diffed against a reference model | **BSD-3-Clause** ✅ (© RISC-V International — verified against upstream `COPYING.BSD` and the `SPDX-License-Identifier` line on all 2 088 files of `riscv-test-suite` at tag 3.9.1) | Yes, with attribution — but it is *built*, not vendored: `scripts/fetch-testdata.sh riscv-arch-test` |
| [sail-riscv](https://github.com/riscv/sail-riscv) | The RISC-V formal model, used as the reference that `riscv-arch-test` signatures are diffed against | **BSD-2-Clause** ✅ (verified against the `copyright` file in the release tarball) | Permissive, so readable — but it is only ever **run**, as a downloaded binary |
| [barotto/test386.asm](https://github.com/barotto/test386.asm) | 80386 CPU test ROM | **GPL-3.0** ⛔ | **No.** Download and run only |
| [OpenSBI](https://github.com/riscv-software-src/opensbi) | RISC-V M-mode firmware | **BSD-2-Clause** ✅ | Yes — readable *and* usable |
| [EDK II / OVMF](https://github.com/tianocore/edk2) | UEFI firmware | BSD-2-Clause-Patent ✅ | Readable; taken as a prebuilt rather than vendored (building it needs a C toolchain). `scripts/fetch-testdata.sh edk2` copies the RISC-V build out of the local `qemu` firmware package |
| [kvm-unit-tests](https://gitlab.com/kvm-unit-tests/kvm-unit-tests) | Atomics, barriers, interrupt controllers — the parallel-execution stress suite | **GPL-2.0** ⛔ | **No.** Download and run only |
| Linux (Debian riscv64 installer kernel, and the matching `linux-image` for its `virtio_mmio` and `virtio_blk` modules) | The `riscv-virt` boot-to-shell gate | **GPL-2.0** ⛔ | **No.** Download and run only — and its source is off limits to this project, not merely unvendorable |
| [busybox](https://www.busybox.net) (Debian's `busybox-static`, riscv64) | The userland `/init` execs, so the boot reaches a prompt | **GPL-2.0** ⛔ | **No.** Download and run only. The `newc` cpio archive built around it by `scripts/fetch-testdata.sh` is ours; the binary inside it is not |
| [busybox](https://www.busybox.net) (upstream's own 1.35.0 x86-64 static build, against musl) | The same job on `pc64`, which reaches a shell prompt over its 16550 | **GPL-2.0** ⛔ | **No.** Download and run only. musl rather than Debian's glibc build for a reason the fetch script records: glibc's static startup runs `PUNPCKLDQ`, which the x86 core advertises through `CPUID` and does not decode |
| [FreeDOS 1.3](https://www.freedos.org) (the floppy edition's `144m/x86BOOT.img`) | The `pc-at` boot-to-DOS gate, on rsemu's own BIOS — `ROADMAP.md` phase 6a | **GPL-2.0** ⛔ for the kernel; the distribution as a whole is a mix | **No.** Download and run only — and its source is off limits, like Linux's |
| [retrio/gb-test-roms](https://github.com/retrio/gb-test-roms) (blargg) | Game Boy CPU, timing, sound | **No licence file** ⛔ | **No.** Provenance unclear — run only |
| blargg's NES test ROMs | NES CPU, PPU, APU | Freely circulated, licence unclear ⛔ | **No.** Run only |
| `nestest` + reference log | 6502 trace comparison — the fastest way to first-boot a 6502 | Licence unclear ⛔ | **No.** Run only |
| Woz Monitor (Apple-1 Operation Manual, 1976) | The Apple 1's 256-byte monitor | **Public domain** ✅ — published pre-1978 with no copyright notice; see [`../platforms/apple1.md`](../platforms/apple1.md) | Yes |
| [Ben Eater's Wozmon port](https://eater.net/6502) (`wozmon.s`, `wozmon.bin`) | The same monitor on a 65C51 ACIA | **CC-BY** ✅ — eater.net/6502 releases all video code under Creative Commons Attribution | Yes, **with attribution**: credit Ben Eater, name the licence, note any modification |
| [AccuracyCoin](https://github.com/100thCoin/AccuracyCoin) | 141 NES accuracy tests on one NROM cart, with per-test error codes | **MIT** ✅ (© 2025 Chris Siebert — verified upstream; the copy in `../gones` predates the licence file) | Yes, with attribution |
| `zexall` / `zexdoc` | Z80 exerciser | Verify before use ⚠️ | Verify first |
| [ZEXALL-SMS](https://www.smspower.org/Homebrew/ZEXALL-SMS) | The same exerciser on a Master System, reporting through the SDSC debug console at `$FC`/`$FD` — the only SMS test ROM that reports as *text* rather than by drawing | **GPL-2.0** ⛔ (the licence file ships inside the archive) | **No.** Download and run only |
| [SMS VDP Test](https://www.smspower.org/Homebrew/SMSVDPTest-SMS) (FluBBa) | Master System VDP registers, latches, collision, interrupt timing | No licence file ⛔ | **No** — and not automatable either: it reports on screen only, with no documented pass/fail location |
| [SMS Test Suite](https://github.com/sverx/SMSTestSuite) (sverx) | Master System video patterns, pads, paddle, BIOS checksums | No licence file ⛔ | **No** — and it needs buttons pressed, so it is a manual suite |
| [Berkeley TestFloat](http://www.jhauser.us/arithmetic/TestFloat.html) | IEEE 754 arithmetic — the standard cross-check for a soft-float implementation | BSD-style ⚠️ — the project page says "release 3 and later have a U.C. Berkeley open-source license"; the exact text is in the release's `COPYING.txt` and has **not** been verified here | Not vendored, and not wired up — see below |

The MIT-licensed SingleStepTests corpora are the ones we could legitimately
vendor — and they are also the most valuable, since they test at bus-cycle
granularity. Even so, size argues for downloading them.

### Floating point has no fetchable corpus, and why

`src/float` is checked against IEEE 754-2019 directly (directed vectors, and an
exact-integer proof that each result is the correctly rounded one), against the
host FPU where the host is an oracle, and against binary64 for the x87 path at
`PC = 53`. There is no downloaded corpus behind any of that, and the reason is
worth writing down rather than rediscovering.

The obvious candidate is **Berkeley TestFloat**, which is what everyone else
uses. Three things stop it being a `fetch-testdata.sh` entry today:

* It **generates** its cases by running a program rather than shipping vectors,
  so "download a corpus" is really "build a C program", which needs a toolchain
  the default test path does not have (`riscv-arch-test` is the one existing
  exception, and it says so).
* The program it compares against **is Berkeley SoftFloat** — TestFloat's
  reference implementation is that library. Running it is black-box use and is
  fine; but its source is a soft-float implementation, and this subsystem was
  written without opening *any* soft-float implementation's source, permissive
  or not. That is hygiene beyond what the licence requires, and it is cheaper to
  keep than to re-establish.
* Its licence is a BSD-style U.C. Berkeley one — almost certainly usable — but
  "almost certainly" is not the standard this table holds itself to, and nothing
  currently depends on the answer.

The shape it would take when it is wanted: build the generator **outside** this
repository, dump its vector files into `$RSEMU_TESTDATA/testfloat/`, and add an
env-gated runner that reads them. Nothing about `src/float` would change.

### AArch64 has no fetchable instruction corpus either, and why

`cpu/arm/a64` ships directed tests written from DDI 0487 and a board test that
runs a real program, and **no downloaded corpus at all**. That is not an
oversight, and the search behind it is worth recording so it is not repeated.

* **SingleStepTests has no AArch64 suite.** The organisation covers 65x02, z80,
  8088/8086, 80286, 80386, 68000, SPC700, V20 and r3000 — every one of them a
  processor somebody could put on a bench with a logic analyser. AArch64 parts
  are not single-steppable that way, and no corpus in that format exists for
  one.
* **Arm's Architecture Validation Suite is not public.** It is licensed to
  implementers under agreement. There is no version of it we can fetch, and no
  substitute Arm publishes.
* **`rems-project/sail-arm`** is the formal model, generated from Arm's ASL. It
  is permissively licensed and could legitimately be *run* — but running it
  means building a Sail/OCaml toolchain, which is further outside the default
  test path than `riscv-arch-test`'s clang requirement, and it produces a model
  to differentially test against rather than a corpus to download.
* **`kvm-unit-tests` has an arm64 target**, and it is GPL-2.0: run-only, like
  every other row above marked so, and it gates a *machine* rather than a core.
* **Built test binaries** are the realistic next step, exactly as
  `riscv-arch-test` is built rather than vendored: `clang` targets
  `aarch64-unknown-none` directly, and a small ELF per feature signalling pass
  or fail through a `BRK` would cover the instruction set and the exception
  model together. That needs the level-3 exit seam the core already has, plus a
  loader, plus a toolchain in CI.

Until then the honest description is the one `ROADMAP.md` §0 uses: a core with
no suite is **untested**, not done. What exists in its place is (a) directed
tests per instruction family written against the manual, with the
`SP`/`XZR` distinction, the carry rule on subtraction, `DecodeBitMasks` and the
translation walk each covered by a test that would fail if the rule were
guessed; (b) a board test that assembles a real program — a loop, a stack, a
supervisor call through the guest's own vector table, and a three-level
translation-table hierarchy the guest builds and then executes through; and
(c) a decode-table self-check that no two rows accept the same word and that
every row is reachable.

## What each milestone is gated on

Phase numbers live in [`../../ROADMAP.md`](../../ROADMAP.md) §13 and are not
repeated here — they drifted once already.

| Milestone | Suites |
| --- | --- |
| AArch64 (`a64-mini`) | `a64-tests` — the suite this repository *builds* rather than fetches, because no usable AArch64 corpus exists. See below |
| NES | SingleStepTests 65x02 (documented opcodes 100 %; the analog unstable ones ledgered separately), `nestest` trace-identical, blargg `cpu_instrs` + `instr_timing`, AccuracyCoin (licence permitting) |
| Game Boy / SMS | `mooneye-test-suite` acceptance, blargg GB, `zexall` |
| RISC-V `virt` | `riscv-tests`, `riscv-arch-test` (in-tree runner, Sail as the reference — no RISCOF), Linux boot to shell |
| PC | `test386.asm`, SingleStepTests 8088, then real-OS boots (FreeDOS → Win 3.11 → Win 95 → Linux → Win XP) |
| SMP emulation | `kvm-unit-tests` atomics/barriers, plus memory-model litmus tests ([`../techniques/memory-models.md`](../techniques/memory-models.md)) |

## The AArch64 exception: a corpus we generate

There is no AArch64 corpus this project can use. SingleStepTests has no
AArch64 repository; Arm's Architecture Validation Suite is licensed to
implementers; `rems-project/sail-arm` is permissive but is a *model* needing a
Sail/OCaml toolchain rather than a downloadable corpus; and `kvm-unit-tests`
has an arm64 target but is GPL-2.0 and is a machine gate rather than a core
one.

So `scripts/fetch-testdata.sh a64-tests` **builds** a suite instead. The guests
are `tests/a64/*.rs` — ours, MIT — compiled by `rustc --target
aarch64-unknown-none`, which needs no C toolchain because that target's linker
is the `rust-lld` inside the Rust toolchain. Each runs to a `BRK #0` and
reports through `x0`–`x3`; `src/cpu/arm/a64/conformance.rs` is the runner.

Generating a corpus removes the *licensing* problem completely and does not by
itself remove the *evidence* problem, so two of the five guests take their
expected values from somewhere that is not this project:

- **`rustc`'s constant evaluator as a floating-point oracle.** Each expectation
  in `fp_arith.rs` and `fp_convert.rs` is computed on the host at compile time
  by `rustc_apfloat` — a port of LLVM's `APFloat`, sharing no code with
  `src/float` — and computed again at run time by the guest, where
  `core::hint::black_box` forces real `FADD`/`FCVTZS` instructions. IEEE 754
  §5.1 makes those operations correctly rounded and therefore unique, so a
  disagreement means one of two independent implementations is wrong.
- **LLVM's instruction selector as an encoding generator.** The guests are
  ordinary Rust and nobody here chose which instructions they contain.

`fp_rules.rs` and the second half of `memory.rs` are directed tests
transcribed from DDI 0487 rather than conformance evidence, and they say so.

## Framework-level testing

Not third-party corpora — these are ours, and they catch more than anything
above:

- **Machine-level regression**: run deterministically for N virtual units,
  assert the final state hash and periodic framebuffer hashes.
- **Snapshot round-trip identity**, per device, no exceptions.
- **Differential**: interpreter vs JIT vs accel over randomized instruction
  streams. The interpreter is the oracle (`ROADMAP.md` §15).
- **Sync-backend equivalence**: identical state hash under `single`,
  `native-std`, and `wasm-atomics`.
- **Fuzzing**: the `.machine` parser, disk-image parsers, every MMIO surface.
