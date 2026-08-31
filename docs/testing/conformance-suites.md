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
| [riscv-arch-test](https://github.com/riscv-non-isa/riscv-arch-test) | RISC-V architectural compliance, driven by [RISCOF](https://github.com/riscv-software-src/riscof) | Not auto-detected — **check the in-tree licence** ⚠️ | Verify first |
| [barotto/test386.asm](https://github.com/barotto/test386.asm) | 80386 CPU test ROM | **GPL-3.0** ⛔ | **No.** Download and run only |
| [OpenSBI](https://github.com/riscv-software-src/opensbi) | RISC-V M-mode firmware | **BSD-2-Clause** ✅ | Yes — readable *and* usable |
| [EDK II / OVMF](https://github.com/tianocore/edk2) | UEFI firmware | BSD-2-Clause-Patent ✅ | Readable; taken as a prebuilt rather than vendored (building it needs a C toolchain). `scripts/fetch-testdata.sh edk2` copies the RISC-V build out of the local `qemu` firmware package |
| [kvm-unit-tests](https://gitlab.com/kvm-unit-tests/kvm-unit-tests) | Atomics, barriers, interrupt controllers — the parallel-execution stress suite | **GPL-2.0** ⛔ | **No.** Download and run only |
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

The MIT-licensed SingleStepTests corpora are the ones we could legitimately
vendor — and they are also the most valuable, since they test at bus-cycle
granularity. Even so, size argues for downloading them.

## What each milestone is gated on

Phase numbers live in [`../../ROADMAP.md`](../../ROADMAP.md) §13 and are not
repeated here — they drifted once already.

| Milestone | Suites |
| --- | --- |
| NES | SingleStepTests 65x02 (documented opcodes 100 %; the analog unstable ones ledgered separately), `nestest` trace-identical, blargg `cpu_instrs` + `instr_timing`, AccuracyCoin (licence permitting) |
| Game Boy / SMS | `mooneye-test-suite` acceptance, blargg GB, `zexall` |
| RISC-V `virt` | `riscv-tests`, `riscv-arch-test` via RISCOF, Linux boot to shell |
| PC | `test386.asm`, SingleStepTests 8088, then real-OS boots (FreeDOS → Win 3.11 → Win 95 → Linux → Win XP) |
| SMP emulation | `kvm-unit-tests` atomics/barriers, plus memory-model litmus tests ([`../techniques/memory-models.md`](../techniques/memory-models.md)) |

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
