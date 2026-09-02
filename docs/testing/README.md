# Running the conformance suites

`ROADMAP.md` §0: accuracy is **measured, never asserted**. This page is how you
run the things that measure it. [`conformance-suites.md`](conformance-suites.md)
is the catalogue and the licence table; this is the operating manual.

The harness lives in [`tests/conformance/`](../../tests/conformance) and is one
Cargo test target, `conformance`.

## Two commands

```sh
scripts/fetch-testdata.sh --all
RSEMU_CONFORMANCE=1 cargo test --test conformance -- --nocapture
```

Without either, `cargo test` still passes: every suite prints why it is skipping
and returns. That is a rule, not a convenience — corpora are downloaded into a
git-ignored directory and are never required for a build (`CLAUDE.md`, Testing).

## Bring-up order

The three suites need increasing amounts of machine. Make them pass in this
order; skipping ahead means debugging a CPU bug through a PPU.

| # | Suite | Needs | Catches |
| --- | --- | --- | --- |
| 1 | **SingleStepTests/65x02** | a CPU and 64 KiB of RAM | per-cycle bus behaviour of one instruction: dummy reads, write ordering, flag edge cases |
| 2 | **nestest** | a CPU, work RAM, a cartridge | cumulative drift over 8 991 instructions — cycle counts and control flow that per-instruction vectors cannot see |
| 3 | **AccuracyCoin** | the whole NES: CPU, PPU, APU, DMA, cartridge, clock domains | interaction and timing *between* components |

AccuracyCoin is emphatically **not** a CPU test. Of its 67 documented sections
roughly 22 are CPU, 24 PPU, 10 APU, 7 DMA and 3 controller; the ones that matter
most — "NMI Suppression", "Sprite 0 Hit behavior", "DMC DMA Bus Conflicts",
"DMA + $2007 Read", "OAM Corruption", "Open Bus" — are about exact alignment
between the CPU and the PPU or APU. Its runner reports per test and degrades
gracefully, so it is informative long before all of that exists, but it cannot
be finished early.

## Gate 1 — SingleStepTests/65x02

MIT, © Tom Harte and contributors, <https://github.com/SingleStepTests/65x02>.
10 000 vectors per opcode; each is an initial register file and sparse RAM, a
final register file and sparse RAM, and the full cycle-by-cycle bus trace.

```sh
# The NES variant (decimal mode disabled) — about 1.1 GB.
scripts/fetch-testdata.sh sst-65x02 --variant nes6502

# Or, while bringing one addressing mode up, a handful of opcodes:
scripts/fetch-testdata.sh sst-65x02 --opcodes a9,ad,bd,b9,a1,b1

RSEMU_CONFORMANCE=1 cargo test --test conformance sst_65x02 -- --nocapture
```

What the runner checks per vector, in order of how often it is the thing that is
wrong:

1. every bus access, in cycle order, with its address, its data and whether it
   was a read or a write;
2. that the cycle count the core *returns* equals the number of accesses it
   *made* — a table-driven timing model passes the trace and fails this;
3. the register file, with a flag-by-flag diff on `P`;
4. the whole 64 KiB of memory, not just the addresses the vector lists, so a
   stray write anywhere is caught.

A core that panics or runs away is reported as a failed vector with context, not
as a dead test binary.

Useful knobs:

| Variable | Effect |
| --- | --- |
| `RSEMU_SST_OPCODES=a9,ad` | run only these opcodes. Ranges work: `a0-af`. |
| `RSEMU_TESTDATA=/path` | corpus root (default `<repo>/testdata`) |

Reports land in `target/conformance/sst-nes6502.txt`: one block per failing
opcode, with a full diff for the first five failing vectors of each.

**Gate (`ROADMAP.md` §13):** 100 % of the documented opcodes, with the
analog-unstable ones (`ANE`, `LXA`, `SHA`/`SHX`/`SHY`/`SHS`) ledgered separately
against the suite's chosen constants.

## Gate 2 — nestest

Licence unclear for both the ROM and the reference log, so both are **fetch-only**
— never commit them, never attach them to a release.

```sh
scripts/fetch-testdata.sh nestest
RSEMU_CONFORMANCE=1 cargo test --test conformance nestest_trace -- --nocapture
```

Entered in automated mode at `PC = $C000` with `P = $24`, `S = $FD`,
`A = X = Y = 0` and 7 cycles already charged for the reset, then compared line by
line against `nestest.log`. Before any of that, the runner asserts that the log's
first line describes the ROM that was fetched — two artefacts from different
sources would make every later comparison meaningless.

Compared every run: `PC`, `A`, `X`, `Y`, `P`, `SP`, the cumulative cycle count,
and the instruction bytes at `PC`. On divergence you get the eight preceding
reference lines, the expected and actual state side by side, a field-by-field
list of what differs (with the cycle drift signed), and the next three reference
lines for orientation.

Not compared by default:

* **The disassembly column.** It is Nintendulator's formatting convention, not
  something the hardware specifies. `RSEMU_NESTEST_DISASM=1` turns it on once the
  core has a disassembler and you want to match the text exactly.

  **rsemu's does not, deliberately, and strict mode therefore fails on line 1.**
  Of the 8 991 lines, 5 370 differ: 5 344 in presentation only (lower-case hex,
  Nintendulator's leading `*` on undocumented encodings, and its resolved-operand
  annotations such as `STX $00 = 00` — which are execution results a
  disassembler cannot produce), 21 in `ISC` against `ISB`, and 5 in `USBC`
  against `*SBC` for `$eb`. None differ in mnemonic plus addressing mode. The
  flag stays as an exact-text check for a core that has chosen that convention;
  rsemu has not, and contorting `cpu::mos6502::disasm` to match one tracer's
  output would be the wrong trade.
* **The `PPU:` columns.** They need a PPU; this bus has none.

Afterwards the runner reads `$02` and `$03`, where the ROM leaves result codes
for the documented and unofficial opcodes. `00 00` means it agreed with itself
too.

**Gate:** trace-identical to the end of the log, and both result codes zero.

## Gate 3 — AccuracyCoin

MIT, © 2025 Chris Siebert, <https://github.com/100thCoin/AccuracyCoin>. 141
asserting tests plus 5 information-only "DRAW" pages on one NROM cartridge.

```sh
scripts/fetch-testdata.sh accuracycoin
RSEMU_CONFORMANCE=1 cargo test --test conformance accuracycoin -- --nocapture
```

Driven headlessly with no rendering and no human: boot, press Start at the menu,
wait for the ROM's own completion flag, read 141 result bytes out of work RAM.
[`accuracycoin.md`](accuracycoin.md) documents exactly how, and why each byte
means what it means.

**Gate:** every test passes, licence permitting (`ROADMAP.md` §13). Until the
machine exists the runner reports each test as "never ran", which is a useful
progress meter rather than a single red light.

## Gate 4 — riscv-arch-test

BSD-3-Clause, © RISC-V International,
<https://github.com/riscv-non-isa/riscv-arch-test>, pinned at tag 3.9.1. The
official architectural certification tests: each one exercises an instruction
across generated corner cases and writes every result into a **signature**
region, and conformance means our signature equals a reference model's, byte
for byte.

```sh
scripts/fetch-testdata.sh riscv-arch-test
RSEMU_CONFORMANCE=1 cargo test --release --all-features \
    --test conformance -- --nocapture riscv
```

This suite is **built, not downloaded** — upstream ships assembly — so the
fetch script needs `clang` and a RISC-V linker (`lld`, or the `rust-lld` that
comes with rustup) and takes a few minutes. That is why it is not in `--all`.
What lands in `testdata/riscv-arch-test` is one linked ELF per test plus the
reference signature for it, so running the suite afterwards needs neither a
toolchain nor a reference model.

Upstream drives all of this with RISCOF (and, after 3.9.1, a successor
framework built on Python, Ruby, `uv` and a UDB gem). **Neither is used.** The
two things they do are done here instead:

* *test selection* — each test's `RVTEST_CASE(...)` macro carries an ISA
  regular expression and a few named DUT parameters; `arch_test_defines` in
  `scripts/fetch-testdata.sh` evaluates them against the ISA string rsemu's
  hart advertises, and the winning case's `def X=Y` clauses become `-D` flags.
* *the reference* — the [Sail RISC-V model](https://github.com/riscv/sail-riscv)
  (BSD-2-Clause), run as a downloaded binary with `--test-signature`, and
  configured by `scripts/riscv-arch-test/sail-config.json` to describe the same
  hart rsemu does. Sail is run, never read.

`scripts/riscv-arch-test/` holds the four inputs that are ours: `model_test.h`
(the device-under-test macros the suite asks every implementer to write),
`link.ld`, and the two reference-model configuration overrides.

**Configure the model to match the hart, or the answer is meaningless.** The
first run of this suite had three failures — `rv64-privilege/ecall`,
`rv64-privilege/ebreak` and `rv64-C/cebreak-01` — that turned out to be one bit
in `misa`: Sail's default configuration enables the hypervisor extension, and the suite's
own trap handler stores a wider trap signature when `misa.H` is set. Nothing
was wrong with rsemu. `sail-config.json` records that, and the ledger's header
says to reach for the configuration before reaching for the ledger.

Useful knobs:

| Variable | Effect |
| --- | --- |
| `RSEMU_ARCH_TEST_ONLY=rv64-I/add,privilege` | run only the tests whose name contains one of these |
| `RSEMU_TESTDATA=/path` | corpus root (default `<repo>/testdata`) |

The report in `target/conformance/riscv-arch-test.txt` names every differing
signature word by index and by byte offset from `begin_signature`, which points
straight at the store that produced it.

**181 of 181** as of 2026-09-01, over 94 152 signature words and 475 424
retired instructions, with an empty ledger. The runner prints those last two
numbers and asserts they are non-zero: a ratio on its own cannot tell a clean
run from a run that measured nothing, which is the mistake this directory has
already made once.

**Gate (`ROADMAP.md` §13):** phase 5, alongside booting Linux on `riscv-virt`.

## AArch64 — `a64-tests`, the suite we build

MIT, ours: the guests are `tests/a64/*.rs`. **Nothing is downloaded**, because
there is nothing usable to download — SingleStepTests has no AArch64
repository, Arm's Architecture Validation Suite is licensed to implementers,
`rems-project/sail-arm` is a model rather than a corpus, and
`kvm-unit-tests`'s arm64 target is GPL-2.0 *and* a machine gate rather than a
core one.

```sh
scripts/fetch-testdata.sh a64-tests
RSEMU_A64_TESTS=testdata/a64-tests \
    cargo test --all-features a64_conformance -- --nocapture
```

The only tool is `rustc` with the `aarch64-unknown-none` target installed
(`rustup target add aarch64-unknown-none`). That target's linker is the
`rust-lld` inside the toolchain, so there is no C toolchain anywhere in this
path — which is why, unlike `riscv-arch-test`, this suite is cheap enough to
run anywhere.

Each guest runs to a `BRK #0` and reports through `x0`–`x3`: zero for success,
otherwise a case number, what it produced, what it should have produced, and a
subtest tag. The core is run with `BREAKPOINT`, `FAULT` and `SYSCALL` armed, so
an unexpected `UNDEFINED` **leaves the core** and the runner prints the
faulting address, `ESR_EL1`, and the instruction disassembled — which is how
the missing Advanced SIMD `MOVI Dd, #0` in an early build of `fp_rules` was
diagnosed in one line.

**Generating a corpus removes the licensing problem completely. It does not by
itself remove the evidence problem**, so four of the six guests take their
expectations from outside this project:

| Guest | Expectations come from |
| --- | --- |
| `fp_arith` | `rustc`'s constant evaluator (`rustc_apfloat`, a port of LLVM's `APFloat`) — an independent IEEE-754 implementation. Correctly-rounded operations are unique, so a disagreement means one of the two is wrong. Thirty-odd hand-chosen operand pairs |
| `fp_random` | the same oracle over a **generated** sweep: a const-evaluated LCG shaped so zeros, subnormals, infinities and NaNs each appear a few percent of the time. 8 000 `binary64` vectors and 4 000 `binary32` ones — still two orders of magnitude short of a SingleStepTests corpus, which is why the file says so |
| `fp_convert` | the same, for `as` casts: Rust's float→int cast saturates with NaN→0, which is exactly `FCVTZS`/`FCVTZU`, so LLVM emits the bare instruction |
| `integer` | `rustc` again for the arithmetic — but the value is the *instruction selection*, which nobody here chose: `SDIV`+`MSUB`, `UMULH`+`MADD`+`ADCS`, `RBIT`+`CLZ`, and several hundred instructions of `compiler_builtins` for a `u128` divide |
| `memory` | properties (a byte written is the byte), plus DDI 0487 B2.9 for the exclusive-monitor cases |
| `fp_rules` | **ours**, transcribed from DDI 0487: the rounding modes, `FPSR`'s sticky flags, `FCMP`'s four-way `NZCV`, `FPMulAdd`'s NaN order. Directed tests that happen to run in a guest, not conformance evidence — the file says so at the top |

| Variable | Effect |
| --- | --- |
| `RSEMU_A64_TESTS=/path` | where the built guests are; without it the test prints how to build them and passes |
| `RSEMU_A64_TESTS_ONLY=fp_arith,memory` | run only the guests whose name contains one of these |

**6 of 6** as of the commit that added floating point, with an empty ledger,
over 243 949 charged bus accesses. The runner prints that number per guest and
asserts the total is non-zero: a pass count on its own cannot tell a clean run
from a guest whose body was optimised away.

The ledger is a `const` list in `src/cpu/arm/a64/conformance.rs` rather than a
file under `ledgers/`, because this suite's runner lives in the crate beside
the core it measures; it is enforced both ways round like every other one here.

**Gate (`ROADMAP.md` §13):** phases 6–8, alongside the ARM JIT frontend.

## The known-failures ledger

Each suite has a ledger under
[`tests/conformance/ledgers/`](../../tests/conformance/ledgers). It **only ever
shrinks**, and the runner enforces both halves of that:

* a failure that is not in the ledger fails the suite;
* a ledger entry whose test now passes *also* fails the suite, telling you which
  line to delete.

Without the second half a ledger quietly becomes a list of things that used to
be broken, and nobody can tell which entries are load-bearing.

Format — one entry per line, `#` starts a comment, and the `# why` note is
mandatory. Suites whose unit is an opcode are keyed by one:

```text
8b                # ANE #imm: unstable, depends on an analog magic constant
ab :: ab 5c 21    # one specific vector, by its upstream name
```

and suites whose unit is a named test are keyed by its name:

```text
rv64-I/add-01     # why
```

`LEDGERS` in [`tests/conformance/main.rs`](../../tests/conformance/main.rs)
says which parser each file uses, and `every_ledger_parses` asserts that the
table and the directory listing agree in both directions — so a ledger file no
suite reads cannot sit there looking load-bearing.

Staleness is only checked for tests that actually ran, so narrowing a run with
`RSEMU_SST_OPCODES` or `RSEMU_ARCH_TEST_ONLY` does not condemn the rest of the
file.

Every ledger is empty today. That is the strictest state, and it is the state
to keep them in: an entry is an admission, not a checkbox.

## What the harness needs from a CPU and a machine

Neither exists yet. The harness is written against two small interfaces:

* [`cpu-interface.md`](cpu-interface.md) — the four methods a 6502 core must
  expose, and the traps. Read this before writing the core, not after.
* [`tests/conformance/machine.rs`](../../tests/conformance/machine.rs) — the
  three methods a realized NES must expose for AccuracyCoin.

Each has exactly one seam (`new_cpu`, `new_nes`) that returns `None` today.
Binding a real implementation there switches every suite from "skipped" to
"running" at once.

## Not yet wired

blargg's `cpu_instrs` and `instr_timing` are on the phase-3 gate
(`ROADMAP.md` §13) and have no runner here. They report by writing a status byte
and an ASCII message to `$6000..`, which needs the machine layer, so they belong
with AccuracyCoin rather than before it. Their provenance is also unclear
(`conformance-suites.md`), so whoever adds them should confirm a source before
adding a URL to the fetch script.
