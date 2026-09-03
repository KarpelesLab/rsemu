# `rsemu` fuzz targets

`cargo-fuzz` targets for the two parsers in rsemu that read input a user can
hand us: the `.machine` description front end and the snapshot container.

This is a **separate crate, detached from the parent workspace** — `fuzz/Cargo.toml`
carries its own `[workspace]` line for exactly that reason. Nothing in the
repo root builds it, `cargo test` at the root does not run it, and
`libfuzzer-sys` never appears in `rsemu`'s dependency tree. It is the one place
in the repo where an external dependency is acceptable, on the same terms as
the sibling crates `compcol` and `purecrypto`: nothing here ships.

## Why these targets

`CLAUDE.md` (Testing) requires `fuzz/` targets for the `.machine` parser, disk
image parsers and every MMIO surface. Of those, the parser and the snapshot
reader exist today; the rest arrive with the code they cover.

`ROADMAP.md` §13, phase 2, states the gate:

> the parser fuzz target survives **1 CPU-hour from a seeded corpus** with zero
> crashes and zero timeouts (unbounded fuzzing is never "clean"; a stated budget
> is).

That is `machine_parser`, run for 3600 seconds against `corpus/machine_parser`.
See [The phase-2 gate](#the-phase-2-gate) for the exact command.

| Target            | What it drives                                     | Property |
|-------------------|----------------------------------------------------|----------|
| `machine_parser`  | `machine::lexer` → `machine::parser` → `Diagnostic::render`, plus constant folding | never panics |
| `state_decoder`   | `core::state::StateReader` over arbitrary bytes     | never panics; canonical encoding |
| `state_roundtrip` | `StateWriter` → `StateReader`, structured input     | writer/reader are inverses |
| `flash_cfi`       | `dev::flash::cfi` — the NOR flash MMIO surface and its snapshot chunk | a program only ever clears bits |
| `ir_verify`       | `ir::verify` over arbitrary blocks, then `ir::eliminate_dead_code` and `ir::Liveness` | never panics; a block the verifier accepts survives elimination and still verifies |
| `riscv_lift`      | `cpu::riscv::lift` → `ir::verify` → `ir::Interp`, against `cpu::riscv`'s interpreter | the lifted block and the interpreter agree on registers, PC, ticks, memory and faults |
| `record_log`      | `core::record::InputLog` — a recorded session, plus the replay path that reads it | never panics; canonical encoding; a replay cursor always advances |
| `journal_log`     | `usermode::journal::Journal` — the second seam's recording, and the snapshot form that embeds it | never panics; canonical encoding; the two forms agree |

### `machine_parser`

Bytes become source text (lossily, so every input is a parse rather than a
discard) and go through the front end. The claim being tested is written down
in `src/machine/parser.rs`: nesting is capped at `MAX_DEPTH`, which bounds both
parser recursion *and* the recursion in dropping the tree, so no input can
overflow the stack.

Both branches matter. The error path runs `Diagnostic::render`, which maps a
span back to `file:line:col` and slices the offending line out of the input to
hang a caret under it — byte arithmetic over untrusted text with multi-byte
characters in it, and it runs on every input the happy path rejects. The
success path walks the tree, evaluating every constant expression through
`Expr::eval_rational` and `OscDecl::frequency_hz`, which do exact `i128`
rational arithmetic on literals the input chose.

### `state_decoder`

`core::state` states its own contract: the reader "never panics, never indexes
without a bounds check, never trusts a length field it has not compared against
the bytes actually remaining, and never allocates proportional to a claimed
count". This drives `StateReader::new` over arbitrary bytes, then every
accessor over whatever parsed, then the typed `ChunkReader` decoders over each
chunk payload.

It also asserts the module's **canonical-form** claim: a snapshot has exactly
one valid encoding, so decoding the fuzzer's bytes and re-encoding them must
reproduce them byte for byte. If the reader ever accepts two spellings of one
snapshot, this finds the pair — and a state hash stops being an identity.

### `state_roundtrip`

The differential. A no-panic check is blind to the failure that actually loses
a save state: a field written one way and read back another, which crashes
nothing and produces a subtly wrong machine a million cycles later. Structured
input builds a `MachineShape` and a set of chunks whose payloads are a
fuzzer-chosen sequence of typed values — every integer width signed and
unsigned, bools, byte arrays, strings, sequences — writes the snapshot, reads
it back, and compares structure and values element by element. It also checks
that emitting the same state twice is byte-identical, which is what
`ROADMAP.md` §0's hash-the-state regression method rests on.

Structured input is decoded by hand (`Gen`) rather than through `arbitrary`'s
derive, so a dependency bump cannot silently reinterpret every committed seed.

### `flash_cfi`

The first per-device target, and the reason `CLAUDE.md` asks for one on every
MMIO surface. A NOR flash has an invariant no guest may break however it drives
the bus: **a program can only clear bits**, and the only thing that puts one
back is an erase, which takes a whole block to `0xff`. The target replays a
fuzzer-chosen command sequence and, after every bus cycle, asserts that each
byte of the array either did not change, lost bits, or became exactly `0xff`.
No unit test can rule that out across an arbitrary command sequence; this can.

It also checks that a debug read answers without moving the state machine, that
a reset clears the command state and not the contents, that a save/load round
trip reproduces the bytes exactly — including a half-issued erase or a staged
write buffer — and that the chunk decoder survives arbitrary bytes.

### `ir_verify`

The third parser in the tree, and the one whose input nobody types: `ir::verify`
reads a block a **frontend** built, and a frontend turns attacker-controlled
guest bytes into IR. A lifter bug — an operand read from the wrong slot, a
temporary numbered off the end of its table, a carry that came out `i32` — hands
the verifier a block no hand-written test resembles, and the verifier's whole
purpose is to *name* the defect there rather than let a backend miscompile it.

The target builds a block from the fuzzer's bytes, dumps it (the `Display` runs
on malformed blocks too — it is what a differential failure gets reported as),
verifies it, and then runs dead-code elimination over it whether or not the
verifier accepted it. That last part is deliberate: a pass is ordinarily run
after the verifier, but nothing in the type system says so, and a pass that
panics on a block the verifier would have rejected is a pass that panics the day
someone reorders the pipeline.

The pass is fuzzed alongside because it is the one component here whose *job* is
to delete things. Flags are ordinary temporaries in this IR (`src/ir/mod.rs`,
decision 1), which is a debt until DCE removes the parity and half-carry
temporaries nothing reads — and a DCE that also removes a store, a charge, a
6502 dummy read or the temporary an `insn_start` names is a silent miscompile:
no crash, a guest that diverges a million cycles later. So each iteration also
asserts that no effect went away, that boundary records and temporary numbering
are untouched, that what survived is either an effect or a value something
needs, and that a second run changes nothing.

Input is a five-byte header and a stream of one-byte selectors, hand-decoded;
the header comment in the target has the table. The generator is biased towards
blocks that verify — a boundary is opened first, a carry in is usually `i1`, a
terminator normally ends the block — because a uniformly random block dies on
the first rule it meets and the agreement between the verifier and the pass
would then never be tested. Every bias is still reachable in the wrong
direction, so the rejection arms stay live.

### `riscv_lift`

The other half of the IR's correctness, and the half no amount of verification
reaches. `ir_verify` asks whether a malformed block is *named* rather than
miscompiled; this asks whether a **well-formed** block means the same thing the
guest's own interpreter does. `CLAUDE.md` ("CPU cores") is the rule:

> Each core ships an interpreter first; the IR frontend comes later and is
> differentially tested against the interpreter **forever**. The interpreter is
> the oracle.

One generated RV64I program runs twice — once through `Hart::step`, once
through `lift` → `verify` → `ir::Interp` — and every column either engine
exposes is compared: the integer register file, the PC, the tick count, the
block's own static tick column, guest RAM byte for byte, and whether the two
agreed about taking a fault. Each catches a different class of bug, and only
the first is obvious: a miscounted `charge` is invisible in the registers and
fails the phase-5 state-hash gate a million cycles later, a store lifted at the
wrong width writes the right register and the wrong memory, and a mis-computed
address usually faults where the interpreter did not.

The comparison itself is `cpu::riscv::differential::compare`, shared with
`tests/riscv_lift_differential.rs` so that a crash found here pastes straight
into an offline regression that needs no fuzzer.

Input is a four-byte header and five bytes per instruction, hand-decoded; the
target's header comment has the table. The `form`/`fields` pair goes to
`differential::synthesize`, which turns any two numbers into an encoding
*inside the lifted subset* — the same bias `ir_verify` uses and for the same
reason. A uniformly random 32-bit word is almost never one of the ninety-odd
encodings this frontend lifts, so an unbiased target would spend its whole
budget proving that `lift` stops cleanly at an unsupported opcode and would
never compare two engines at all. The out-of-subset arms stay reachable
anyway: `JALR` on a core without `C`, an out-of-range shift amount, and a
branch to a four-byte-misaligned target all come back as "nothing lifted".

The header's first byte picks between three cores — bare RV64I, one that traps
misaligned accesses, and one with `C` — because the misalignment policy is in
every memory op's `Align` *and* in the block's cache key, and `C` decides both
whether a 16-bit halfword is an instruction and whether `JALR` is in the subset.

## Setup (one-time)

```sh
cargo install cargo-fuzz
rustup toolchain install nightly   # cargo-fuzz needs nightly for -Z flags
```

The repo pins stable in `rust-toolchain.toml`, so **every command below needs
an explicit `+nightly`**; the rustup proxy's `+toolchain` overrides the
toolchain file for the whole build.

## Running a target

From the repo root (cargo-fuzz finds `./fuzz` on its own):

```sh
cargo +nightly fuzz build                       # all three targets
cargo +nightly fuzz list
cargo +nightly fuzz run machine_parser -- -max_total_time=60
```

Runs pick up `fuzz/corpus/<target>/` by default. **That is the seed corpus and
it is committed**, so a default run will grow it with libFuzzer's own
discoveries. To keep the committed seeds curated, point the writable corpus
somewhere else and pass the seeds as a second, read-only input:

```sh
mkdir -p /tmp/rsemu-corpus/machine_parser
cargo +nightly fuzz run machine_parser \
    /tmp/rsemu-corpus/machine_parser fuzz/corpus/machine_parser \
    -- -max_total_time=60
```

libFuzzer writes new units to the *first* corpus directory listed.

## The phase-2 gate

```sh
mkdir -p /tmp/rsemu-corpus/machine_parser
cargo +nightly fuzz run machine_parser \
    /tmp/rsemu-corpus/machine_parser fuzz/corpus/machine_parser \
    -- -max_total_time=3600 -timeout=10 -print_final_stats=1
```

The gate is met when that exits 0: **zero crashes and zero timeouts** over one
CPU-hour, starting from the committed seeds. `-timeout=10` makes "timeout"
mean something — without it libFuzzer's default is 1200 s per input, which no
run would ever reach. `-print_final_stats=1` prints the execution count to
record alongside the result; a run that managed only a few thousand executions
has not tested anything, however green it looks.

Anything the run finds lands in `fuzz/artifacts/machine_parser/`.

## Seed corpora

Seeds are committed because the gate is stated in terms of a *seeded* corpus.
They are hand-written, not fuzzer output.

`corpus/machine_parser/` — valid `.machine` files, one deliberately invalid:

| File | Why |
|---|---|
| `nes.machine` | `ROADMAP.md` §5's worked example, verbatim: the language's acceptance test |
| `minimal.machine` | the smallest legal file |
| `templated.machine` | the phase-2 fixture: `include`, `template`, indexed instantiation, `param` |
| `heterogeneous.machine` | two CPU classes, two spaces, one shared RAM region, differing endianness |
| `expressions.machine` | every literal form, radix, size/duration suffix and operator |
| `comments.machine` | comments in every position, optional separators |
| `deep.machine` | nesting, well inside `MAX_DEPTH`, for the mutator to push past it |
| `errors.machine` | **invalid on purpose** — seeds the diagnostic renderer, with multi-byte characters where a caret placed by byte offset would go wrong |

`corpus/state_decoder/` — snapshots written by `StateWriter` (so they are valid
by construction), plus four malformed derivatives:

| File | Why |
|---|---|
| `empty.snap` | the smallest legal snapshot: empty shape, no chunks |
| `one-chunk.snap` | the `core::state` doc example |
| `nes-shaped.snap` | five devices, three regions, features, arches, chunks using every typed encoder, including a scheduler chunk (§4.5: the scheduler is architectural state) |
| `unicode.snap` | multi-byte and empty strings in every string position |
| `bad-magic.snap`, `truncated-header.snap`, `truncated-chunks.snap`, `trailing.snap` | the four rejection paths, so the mutator starts from them |

`corpus/ir_verify/` — inputs to that target's own `Gen` decoder, so every file
is a translation block written in the selector encoding the target documents:

| File | Why |
|---|---|
| `parity` | the case the pass exists for: an `XOR` that is stored, and a popcount flag nothing reads, which elimination removes |
| `parity-live` | the same flag named live at the next guest instruction boundary, where it must survive — the pass's subtle failure, and the only difference between the two files |
| `dummy-read` | a volatile load beside a plain one at the same address: the 6502 dummy read stays, the plain load goes |
| `effects` | `brcond`, a helper call and an atomic, none of whose results anything reads and none of which may be removed |
| `malformed` | **invalid on purpose** — a temporary used before it is assigned, one that was never allocated, and a second terminator after the first |

`corpus/state_roundtrip/` — inputs to that target's own `Gen` decoder, not
snapshots: `empty`, `widths` (every integer width at its extremes),
`nes-shaped`, `unicode`, and `short-truncated` — five bytes that run the
generator off the end of its input, which is where the harness's first version
had its own out-of-bounds bug.

## Triaging a crash

`cargo fuzz` writes the failing input to `fuzz/artifacts/<target>/crash-<sha>`
and prints the reproduce command. Then:

```sh
cargo +nightly fuzz run  <target> fuzz/artifacts/<target>/crash-<sha>   # reproduce
cargo +nightly fuzz tmin <target> fuzz/artifacts/<target>/crash-<sha>   # minimize
cargo +nightly fuzz fmt  <target> fuzz/artifacts/<target>/crash-<sha>   # show the decoded input
```

`fmt` is the useful one for `state_roundtrip`, whose input is structured: it
prints the `Debug` of what the harness decoded rather than the raw bytes.

Then, in order:

1. **Decide whose bug it is.** A panic inside `rsemu` is a finding. A panic
   inside a `fuzz_targets/` frame is a harness bug — the input decoder running
   off the end of a short input is the classic one — and fixing the harness is
   not fixing the crash.
2. **Turn it into a unit test** in the module that owns the code, beside the
   rest of its tests (`CLAUDE.md`, Testing). The fuzz corpus is not a
   regression suite; a `#[test]` is.
3. **Commit the minimised input** as a corpus seed, so the mutator keeps that
   region of the input space warm.

## Adding a target

1. Write `fuzz_targets/<name>.rs`, modeled on the closest existing one.
2. Add a `[[bin]]` entry to `fuzz/Cargo.toml`.
3. Seed `corpus/<name>/` with inputs that are valid for whatever it parses.
   A target with no seeds spends its first CPU-hour rediscovering the file
   format.

Next in line, as the code lands: the disk-image parsers (§7.1), every MMIO
register surface (§4.1), and the JSON projection of the machine language once
`rsemu convert` exists — the JSON side is a second parser for the same AST and
inherits the same obligations.

## When CI runs this

Not on every commit. `.github/workflows/fuzz.yml` runs the sixty-second smoke
daily and on manual dispatch, because what that budget actually detects is a
target that stopped compiling against a changed API — a property of the tree
rather than of one commit. **Run it by hand from the Actions tab, or locally
with the command above, before merging a change to a fuzzed surface**: the
`.machine` parser, a disk-image parser, or an MMIO register block. The daily
run is drift detection; the real campaign is still the command at the top of
this file.
