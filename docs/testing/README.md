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
mandatory:

```text
8b                # ANE #imm: unstable, depends on an analog magic constant
ab :: ab 5c 21    # one specific vector, by its upstream name
```

Staleness is only checked for tests that actually ran, so narrowing a run with
`RSEMU_SST_OPCODES` does not condemn the rest of the file.

All three ledgers start empty. That is the strictest state and the right one for
a core nobody has measured yet.

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
