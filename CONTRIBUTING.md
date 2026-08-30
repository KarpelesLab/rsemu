# Contributing to rsemu

Thanks for wanting to help. One rule comes before all the others, and a patch
that breaks it cannot be accepted no matter how good it is.

## 1. Provenance: where your code came from

rsemu is **MIT licensed**. MIT is permissive and the GPL is copyleft, and the
compatibility runs one way only: **GPL'd projects may absorb MIT code, but MIT
projects may never absorb GPL'd code.** Paraphrasing does not launder it —
code written by studying a GPL implementation is a derivative work of that
implementation regardless of how different it looks.

This matters more than it might seem. Taint cannot be repaired by deleting a
file: once contaminated code is merged, the history and everything built on top
of it are affected, and the project becomes undistributable under its own
licence. So the rule is preventative, not corrective.

### Do not read QEMU. At all.

QEMU is GPLv2 and is **permanently off limits** — not the source, not the
headers, not the in-tree comments or documentation, not commit messages, not
mailing-list patches, and not anything derived from it (Unicorn Engine, forks,
copies vendored inside other emulators).

*"I only looked to understand the concept"* is exactly the act this forbids.

If you have previously read QEMU source covering a subsystem you would like to
work on, **please tell us and pick a different subsystem.** This is standard
clean-room practice and it protects you as much as the project — nobody is
being accused of anything.

### The same applies to every copyleft source

The rule is the licence, not the name. GPL, LGPL, AGPL, SSPL, CDDL, MPL-2.0 and
EPL sources are all off limits, including the emulators people reach for by
reflex: Bochs, DOSBox, MAME, VICE, Dolphin, PCSX2, Nestopia, higan. **Check a
project's licence before you open it.**

Two *documentation* licences get a different answer rather than a ban. **GFDL**
(the GDB manual, the Multiboot spec) and **CC-BY-SA** (several hardware wikis)
are fine to implement from — that is what a published specification is for — but
do not copy their prose into our source or docs. Read it, write your own words,
cite the section.

LGPL is the one people get wrong: it permits *linking*, not *copying source
into an MIT crate*. Treat it exactly like GPL here.

## 2. What you should use instead

The prohibition is narrow, and everything below covers essentially all real
work:

- **Hardware documentation** — datasheets, ISA manuals (Intel SDM, ARM ARM,
  RISC-V specs), service manuals, schematics, errata. This is the primary
  source, and it is a better one: it describes the machine, not somebody's
  guess at it.
- **Community hardware documentation** — the NESdev wiki, Pan Docs, OSDev wiki,
  hardware test write-ups. Respect each site's licence for verbatim text; the
  underlying facts are free.
- **Papers and textbooks** on binary translation, JIT compilation, register
  allocation, memory models.
- **Permissively licensed code** — MIT, BSD, Apache-2.0, ISC, public domain —
  with its copyright notice and licence text retained. Attribution is required,
  not optional.
- **Real hardware.** Measuring the actual machine settles arguments no secondary
  source can.
- **Black-box observation of any program, GPL included.** Run QEMU, benchmark
  it, compare traces against it. Using a GPL program as a measuring instrument
  creates no derivative work. Reading its source does.

### Facts versus expression

The distinction that decides most real cases: **hardware behaviour is fact; an
implementation of that behaviour is expression.** An opcode's cycle count from a
datasheet is a fact you may use freely. The identical number copied out of a GPL
emulator's timing table is expression obtained from a forbidden source. Take
facts from primary sources and the question never arises.

### Test ROMs and conformance suites

Several carry copyleft licences (`kvm-unit-tests` is GPLv2) and some have
unclear provenance. rsemu **downloads test corpora at test time into an ignored
directory and never commits them**. Running a GPL binary as an emulated guest is
ordinary use; shipping it in this repository would be redistribution under its
terms. Confirm the licence of any fixture before proposing to vendor it.

### Tool-assisted code

The same rule applies to code produced with an AI assistant. If a tool emits
recognizable GPL code, it is still GPL code — origin is a property of the code,
not of how it reached your editor. Review what you submit.

### Cite your sources

Any non-obvious algorithm should name its source — which manual, which section —
in the commit message or a comment. Provenance has to be auditable years later
by someone who was not in the room.

## 3. Everything else

- Read [`CLAUDE.md`](CLAUDE.md) for the design rules (crate shape, `no_std`, the
  `sync` seam, determinism, error handling, testing) and
  [`ROADMAP.md`](ROADMAP.md) for the architecture and phase order.
- Dependency policy is strict: the default build must have an empty
  `cargo tree`. Only first-party Karpelès Lab crates, feature-gated.
- `cargo fmt` and `cargo clippy -D warnings` must be clean, across the whole
  target matrix — native, `no_std`, and both wasm configurations.
- New devices ship with a snapshot round-trip test; new CPU cores ship with
  their conformance suite.
- By submitting a patch you confirm you have the right to license it under MIT
  and that it complies with §1 above.

Questions about whether a source is safe to consult are always welcome — ask
**before** reading, not after.
