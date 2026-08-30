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
| [kvm-unit-tests](https://gitlab.com/kvm-unit-tests/kvm-unit-tests) | Atomics, barriers, interrupt controllers — the parallel-execution stress suite | **GPL-2.0** ⛔ | **No.** Download and run only |
| [retrio/gb-test-roms](https://github.com/retrio/gb-test-roms) (blargg) | Game Boy CPU, timing, sound | **No licence file** ⛔ | **No.** Provenance unclear — run only |
| blargg's NES test ROMs | NES CPU, PPU, APU | Freely circulated, licence unclear ⛔ | **No.** Run only |
| `nestest` + reference log | 6502 trace comparison — the fastest way to first-boot a 6502 | Licence unclear ⛔ | **No.** Run only |
| AccuracyCoin | NES accuracy test ROM | **No licence file present** in `../gones` ⛔ | **No** — flagged in `ROADMAP.md` §1 |
| `zexall` / `zexdoc` | Z80 exerciser | Verify before use ⚠️ | Verify first |

The MIT-licensed SingleStepTests corpora are the ones we could legitimately
vendor — and they are also the most valuable, since they test at bus-cycle
granularity. Even so, size argues for downloading them.

## What each phase is gated on

| Phase | Machine | Suites |
| --- | --- | --- |
| 4 | NES | SingleStepTests 65x02 (100 %), `nestest` trace-identical, blargg `cpu_instrs` + `instr_timing`, AccuracyCoin |
| 5 | Game Boy / SMS | mooneye acceptance, blargg GB, `zexall` |
| 6 | RISC-V `virt` | `riscv-tests`, `riscv-arch-test` via RISCOF, Linux boot to shell |
| 7 | PC | `test386.asm`, SingleStepTests 8088, then real-OS boots (FreeDOS → Win 3.11 → Win 95 → Linux → Win XP) |
| 9 | Parallel execution | `kvm-unit-tests` atomics/barriers, plus memory-model litmus tests ([`../techniques/memory-models.md`](../techniques/memory-models.md)) |

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
