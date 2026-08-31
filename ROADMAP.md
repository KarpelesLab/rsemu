# rsemu Roadmap — a pure-Rust emulator, built from the bottom up

`rsemu` is an **emulator** — the thing you point at a ROM or a disk image and
run. It is *built* on a generic framework, and that framework is what gets
written first, because bottom-up is the order that produces an emulator worth
having: address spaces, clock domains, wires, devices, buses and a translation
IR; then CPU cores, PCI, USB, storage and NICs on top of them; then machines
described by a config file rather than compiled in.

The framework is the means. The end is a binary that emulates a NES, a Game Boy,
a RISC-V board and a PC — and that a stranger can point at a machine file
describing four heterogeneous CPUs sharing one RAM region across three bus
fabrics without patching rsemu to allow it.

Starting low costs time before the first ROM boots. It buys the thing every
emulator that started at the top eventually wishes it had: **one** memory model,
**one** clock, **one** snapshot format, **one** debugger, shared by every
machine that will ever be added.

This roadmap defines the architecture, the phase order, and the acceptance gate
for each phase. It is written to be executed top-to-bottom; every phase ships
something a person can actually run (§2).

> **Status (2026-08-31).** Phases 0-2 are done and phase 3 is most of the way
> there. ~88k lines, 1,223 tests, one crate in `cargo tree`, `unsafe` confined
> to two of the six sanctioned sites.
>
> **Five CPU cores**, each with an independently-run conformance number rather
> than an assertion: MOS 6502 **2,560,000/2,560,000** (with full bus traces),
> Z80 **1,604,000/1,604,000** plus `zexall` 67/67, RISC-V RV64GC **409/409**,
> 8086/8088 **2,974,160/3,007,000** (the gap is one microcode residue in the
> undefined flags after `IMUL`/`DIV`), and ARMv5TE (no usable public corpus for
> v5 — see §12).
>
> **Three machines that run**: `nes-ntsc` and `nes-pal` — AccuracyCoin boots,
> raises NMI and draws its menu — and `apple1`, which is interactive over a
> terminal. Plus the DSL front-to-back: parse, resolve, validate, realize, run.
>
> Ordering deviated from the plan deliberately, and it was the right call.
> ARM was pulled forward for a downstream crate; Z80, x86 and RISC-V were built
> in parallel once the 6502 proved the shape. What did *not* deviate: every core
> ships its suite, and the framework was finished before the emulation started.
>
> What remains for phase 3 is small and known: a controller at `$4016`, OAM DMA
> at `$4014`, cartridge-driven nametable mirroring, and dot-exact rather than
> scanline-exact intra-quantum staleness (§4.2). Phase 5's IR and JIT have not
> been started.
---

## 0. Non-negotiables

These are decided. Do not relitigate them mid-implementation.

- **Pure Rust, no foreign code.** No C, no `bindgen`, no vendored assembly, no
  build scripts that invoke a compiler. The dependency budget is *first-party
  Karpelès Lab crates only* (§14) — and even those stay feature-gated so the
  core builds with an empty `cargo tree`. **That holds for default features**;
  several siblings pull external crates under optional ones (tokio, mio, rustls,
  RustCrypto, libc), so CI checks the *feature-enabled* dependency tree, not
  just the default.
- **`unsafe` is quarantined.** Crate-wide `unsafe_code = "deny"` (not `forbid`).
  Exactly **six** subsystems may opt back in with a scoped
  `#[allow(unsafe_code)]` + a `// SAFETY:` comment: the RAM host-pointer fast
  path, the JIT code buffer (W^X `mmap`/`mprotect`), the raw-syscall accel
  backends (KVM ioctls), the C ABI module, the **`core::sync` `single` backend**
  (its lock is an `UnsafeCell` with a `Sync` impl justified by that backend being
  single-threaded by construction — `RefCell` is *not* `Sync`, so there is no
  safe way to satisfy the `Send + Sync` bound), and **per-CPU execution state**
  (the register file lives behind an `UnsafeCell` guarded by an
  exclusive-execution token from the scheduler; a lock per instruction cannot
  reach the phase-5 throughput gate). Everything else is safe Rust. **Six is the
  ceiling** — a seventh is a design review, not a commit.
- **Determinism is a first-class mode, not an afterthought.** A machine run in
  deterministic mode must produce a bit-identical state hash across runs, hosts,
  and — for the same guest architecture — across the interpreter and the JIT.
  Record/replay, save states, rewind, and the entire regression suite are built
  on this. Speed is never traded for determinism without a flag.
- **Accuracy is measured, never asserted.** Every CPU core ships with a
  published conformance suite (§12) and a known-failures ledger that only ever
  shrinks. A core with no suite is not "done", it is "untested".
- **Generic first, specific second.** If a device model needs a mechanism the
  core does not have, the mechanism gets added to the core generically — never
  special-cased in the device. The NES PPU must not appear in a `core::` type
  signature.
- **`no_std` + `alloc` core stays buildable.** The emulation core (memory,
  clock, devices, IR, interpreters) never touches `std` — including for
  threading, which goes through the `core::sync` seam (§4.7). **Two documented
  exceptions:** `dev/blk/*` and `dev/net/*` are `std`, because `fstool` and
  `pktkit` are `std` crates (`fstool::BlockDevice` is `std::io::Read + Write +
  Seek`). They are feature-gated so a `no_std` build simply excludes them. Host I/O, JIT,
  accel and frontends live above the `std` line. CI builds both.
- **Multithreaded by design.** Every core type is `Send + Sync` from phase 1,
  and threading is a configuration, never an assumption baked into a device.
  **Guest-visible determinism is a property of the `deterministic` threading
  mode, not of the thread count** — parallel guest execution is inherently
  non-reproducible (§16), and claiming otherwise would set an unreachable gate.
  What *must* hold regardless of thread count: background work (compilation,
  device I/O, encoding) never changes guest-visible results.
- **The browser is a first-class target.** rsemu builds and runs on wasm with
  *and without* threads, from phase 0, in CI (§11). No `mmap`, no OS threads, no
  signals, no monotonic clock — a constraint that keeps the core portable rather
  than one that limits it. One documented exception to "stable toolchain": the
  threaded **browser** job needs a nightly, for a reason outside our control
  (§11.1).
- **MIT licensed, and clean-room.** GPL/LGPL/AGPL sources are off limits —
  **QEMU above all**, permanently and in its entirety. Work from hardware
  documentation, not from somebody else's emulator. Read §1 before writing
  anything; a tainted contribution cannot be undone by deleting the file.
- `edition = "2024"`, `rustfmt` + `clippy` clean under `-D warnings`. **Stable
  toolchain for every shipping artifact**; the one nightly-pinned CI job is
  named and justified in §11.1.

---

## 1. Licensing and provenance — read this before writing a line

rsemu is **MIT licensed**. MIT is a permissive licence and is **one-way
incompatible with the GPL family**: GPL'd code can absorb MIT code, but MIT code
can never absorb GPL'd code. There is no exception, no "just for reference", and
no amount of paraphrasing that launders it. A single tainted contribution makes
the project undistributable under its own licence and is not fixable by deleting
the file later — the history and everything derived from it are contaminated.

### QEMU is specifically and permanently off limits

**Do not read, open, clone, grep, quote, adapt, translate, or consult the QEMU
source tree.** QEMU is GPLv2. This prohibition covers the whole artifact, not
just `.c` files:

- source files, headers, and build system
- comments and documentation inside the tree
- commit messages, mailing-list patches, and code review threads
- any project derived from it (Unicorn Engine, libvirt's QEMU-specific code,
  forks, and vendored copies inside other emulators)

"I only looked at it to understand the concept" is exactly the act this rule
forbids, because the resulting code is a derivative work of what you read
whether or not it looks similar. If you have previously read QEMU source for a
given subsystem, **say so and let someone else write that subsystem.** That is
not a judgment; it is ordinary clean-room hygiene and it protects you too.

### Other copyleft sources, same rule

The rule is the licence, not the name. Any GPL, LGPL, AGPL, SSPL or CDDL source
is off limits. Emulator projects people reach for by reflex — Bochs, DOSBox,
MAME, VICE, Dolphin, PCSX2, Nestopia, higan — are copyleft and are all
forbidden. **Verify a project's licence before you open it**, not after.

LGPL deserves its own sentence because it is routinely misread: LGPL permits
*linking*, not *copying source into an MIT crate*. It is as forbidden here as
GPL.

### What you may absolutely use

The rule above is narrow on purpose. This is the other half, and it is where
essentially all legitimate work happens. **[`docs/`](docs/) is the curated,
link-checked register of these sources**, organized by subsystem — start there
rather than searching:

- **Hardware documentation.** Datasheets, ISA manuals (Intel SDM, ARM ARM,
  RISC-V ISA specs), chip service manuals, schematics, errata sheets. This is
  the *primary* source and should be the first thing you reach for anyway — it
  describes the machine we are emulating, not somebody's emulation of it.
- **Community reverse-engineering documentation**: the NESdev wiki, Pan Docs,
  OSDev wiki, hardware-test write-ups. These document *facts about hardware*.
  Respect each site's licence for verbatim text; the facts themselves are free.
- **Academic papers and textbooks** on dynamic binary translation, JIT
  compilation, register allocation, and memory models.
- **Permissively licensed code** — MIT, BSD, Apache-2.0, ISC, public domain —
  used *with* its copyright notice and licence text retained. Attribution is not
  optional just because the licence is easy.
- **Real hardware.** Measuring a real console or PC is unimpeachable and often
  more accurate than any secondary source.
- **Black-box observation of any program, including GPL ones.** Running QEMU,
  benchmarking it, comparing its output to ours, or diffing an execution trace
  creates no derivative work. Using a GPL emulator as a *measuring instrument*
  is fine; reading its source is not. Where this roadmap compares performance
  against QEMU (§13), that is black-box benchmarking and nothing more.
- **Our own code.** `../gones` is MIT (© Mark Karpelès), and the PPU/APU lineage
  inside it derives from Michael Fogleman's MIT-licensed NES emulator. The
  phase-4 port is therefore clean — but the Fogleman copyright notice travels
  with that code and must be preserved in the ported files.

### Facts versus expression

The line that matters in practice: **hardware behaviour is fact, somebody's
implementation of it is expression.** An opcode's cycle count taken from a
datasheet is a fact and may be used freely. The same number copied out of a GPL
emulator's timing table is expression you obtained from a forbidden source —
even though the number is identical. Get facts from primary sources so the
provenance question never arises.

### Test corpora: run them, never vendor them

Conformance suites and test ROMs carry their own licences, several of them
copyleft (`kvm-unit-tests` is GPLv2), and some are of unclear provenance
entirely. rsemu therefore **downloads test corpora at test time into an ignored
directory and never commits them** (§12). *Executing* a GPL binary as an
emulated guest is ordinary use and creates no derivative work; *shipping* it in
our repository would be redistribution under its terms. Confirm a fixture's
licence before vendoring it.

**`AccuracyCoin` is settled — and it is MIT.** An earlier revision here said it
had no licence and was run-only. That was true of the *copy* in `../gones`,
which predates the licence file, and wrong about the project:
[100thCoin/AccuracyCoin](https://github.com/100thCoin/AccuracyCoin) is MIT,
© 2025 Chris Siebert (verified against the upstream `LICENSE`). It may therefore
be used *and* redistributed with its notice — a rare and valuable thing for a
conformance ROM. Take it from upstream rather than from `../gones`, and do not
copy `nesasm.exe`, a third-party Windows binary of unstated provenance that the
local tree also carries.

The lesson generalises: **check the upstream, not the copy in front of you.** A
vendored tree missing a licence file says nothing about the project's licence.

### Practical discipline

1. **Cite the source** for any non-obvious algorithm, in the commit message or a
   comment: which manual, which section. Provenance must be auditable years
   later by someone who was not there.
2. **Tool-generated code is subject to the same rule.** An AI assistant that
   reproduces recognizable GPL code has not cleaned it. Origin is a property of
   the code, not of the keyboard it arrived through.
3. **Do not adopt another project's internal jargon** in our public API or
   module names. It is bad naming in its own right, and it makes an
   independently-written subsystem look derived when it is not.
4. **No file is named after a forbidden project's file**, no comment is
   translated from one, and no constant table is copied from one unless the
   values are independently obtainable hardware facts (§1, above).
5. When in doubt, ask before reading — not after.

The [`docs/`](docs/) index also records what is **deliberately excluded** and
why, so a forbidden source does not get added later as an apparent oversight.

---

## 2. What rsemu is when it's finished

The framework is judged by the emulator it produces, so the product surface is
specified here rather than discovered in the last phase.

### The binary

```console
$ rsemu run nes.machine --cart smb.nes              # a machine file + its media
$ rsemu run q35.machine -p ram=8G --disk win.qcow2 --accel kvm
$ rsemu run --machine gb --rom tetris.gb            # catalog shorthand
$ rsemu machines                                    # what this build can emulate
$ rsemu devices                                     # every registered device class
$ rsemu describe pci.nvme                           # class, properties, defaults
$ rsemu convert nes.machine --json                  # tooling projection
$ rsemu record session.trace -- run nes.machine --cart smb.nes
$ rsemu replay session.trace                        # bit-identical, on any host
$ rsemu debug q35.machine --gdb :1234               # gdbstub attached to the guest
```

Save states, rewind, screenshots, VNC display, and the monitor console are
properties of the framework (§4.5, §8), so every machine gets them the day it
exists — not once someone writes per-machine plumbing.

### The machine catalog

`machines/` ships description files as **data**: consoles, boards, and PC
chipsets, each a readable file a user can copy and modify. Adding a machine
that rsemu already has the components for requires no Rust and no rebuild.
This is the test of whether §5 succeeded.

### The library and the C ABI

The same tri-modal model proven in `purecrypto` and `kataan`: a Rust library, a
C library (`ffi`), and a standalone binary. Embedding rsemu into someone else's
application — a test harness, a CI runner, a game front-end, a hardware
bring-up tool — is a supported use, not a fork.

### Every phase ships a usable emulator

The phase plan (§13) is ordered so that value lands long before the framework is
"finished":

| After | Someone can actually… |
| --- | --- |
| Phase 3 | play NES games, with save states and a debugger |
| Phase 4 | play Game Boy and Master System games on the same binary |
| Phase 5 | boot a RISC-V Linux to a shell and debug the kernel over gdb |
| Phase 6 | run a PC — DOS, Win95, Linux, XP — with disks, USB and networking |
| Phase 7 | run that PC at near-native speed under KVM |
| Phase 9 | drive all of it over VNC, record and replay sessions, embed it in something else |

Phases 1–2 are the only ones with no user-facing artifact. That is the price of
starting low, and it is paid once — which is only true because phase 3 carries a
**minimum host slice** (window, input, audio, gdbstub) rather than deferring all
of `host/` to the end. A framework with no way to see or hear its output has not
shipped anything, whatever the gate says.

---

## 3. Crate shape

One crate, `rsemu`, with **one Cargo feature per component** — the `compcol`
model, scaled up. A machine is then a feature set: `--features "cpu-mos6502,
machine-nes"` produces a binary that can emulate a NES and nothing else, with
no dead device models linked in.

```
src/
  lib.rs              # feature-gated re-exports, nothing else
  core/               # THE FRAMEWORK — no feature gates, always compiled
    value.rs          #   widths, endianness, typed access
    space.rs          #   AddressSpace, MemRegion, FlatView, dispatch tables
    ram.rs rom.rs     #   backing stores
    clock.rs          #   oscillator forest; exact within a tree, bounded across
    sched.rs          #   event queue, execution budgets, quantum, threading modes
    sync.rs           #   portability seam: locks, atomics, task pool (4 backends)
    wire.rs           #   IRQ / GPIO lines, splitters, combiners
    device.rs         #   Device trait, lifecycle, composition
    props.rs          #   dynamic property values + typed extraction
    registry.rs       #   by-name construction (the config's entry point)
    state.rs          #   versioned snapshot reader/writer
    reset.rs bus.rs   #   reset trees, generic Bus trait
    error.rs trace.rs #   diagnostics, structured tracing
  machine/            # the description language: lexer, parser, resolver, realizer
  ir/                 # translation IR: ops, builder, passes, verifier
  jit/                # backends: x86_64, aarch64, riscv64, wasm, portable interpreter
  accel/              # kvm, (hvf), (whpx) — execution engines that aren't ours
  cpu/                # one module + one feature per core: mos6502, z80, sm83, …
  dev/                # one module + one feature per device: pci/, usb/, blk/, net/, …
  boards/             # one module + one feature per built-in machine
  host/               # std-only: display, audio, input, gdbstub, VNC, CLI
                      #   + the wasm shim (worker pool, imports, ring buffers)
machines/             # shipped .machine description files (data, not code)
```

**Why one crate.** Cross-component invariants (snapshot versioning, the
determinism contract, the IR) change together; splitting them across crates
means a version-skew matrix nobody will maintain. **Escape hatch:** if
full-feature compile time exceeds ~90 s, split *only* `jit/` and `host/` into
sibling crates — those have the fewest inbound edges. Do not split `cpu/` or
`dev/`; they are the whole point of the feature system.

---

## 4. The generic core

This is the part that must be right. Everything else is replaceable.

### 4.1 Address spaces and memory

The single most important abstraction. Modelled as a **region tree flattened
into a dispatch table**: a tree because that is how real hardware composes
(a chipset contains a bridge contains a device, each with its own window), and a
flattened dispatch table because a tree walk per access would be ruinous. The
tree is what the machine file describes and what a human reasons about; the flat
view is a derived cache, rebuilt whenever the topology changes.

```rust
pub trait MemOps: Send + Sync {
    fn read (&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult;
    fn write(&self, offset: u64, src: &[u8],     attrs: MemAttrs) -> MemResult;
    fn constraints(&self) -> AccessConstraints;  // min/max width, alignment, endianness
}

pub enum Region {
    Ram   { store: Arc<RamStore>, len: u64 },       // host-backed, direct pointer
    Rom   { store: Arc<RomStore>, len: u64 },       // reads direct, writes → policy
    Io    { ops: Arc<dyn MemOps>, len: u64 },       // MMIO, always a call
    Alias { target: RegionRef, offset: u64, len: u64 },   // mirrors, windows
    Container { children: Vec<Mapping> },           // { region, base, priority }
}
```

Requirements the design must satisfy from day one, because retrofitting any of
them is a rewrite:

| Requirement | Why it exists |
| --- | --- |
| **Overlapping regions with priority** | PCI BAR over RAM; NES cartridge mappers; boot ROM shadowing |
| **Aliases / mirrors** | NES `$0000-$07FF` mirrored 4×; SNES banks; ARM alias windows |
| **Per-master address spaces** | CPU view ≠ DMA view ≠ GPU view. The user's "unholy" configs live here |
| **`MemAttrs`** — requester ID, secure/non-secure, user/priv, exclusive, **debug** | IOMMU translation, TrustZone, and a debugger read that must not pop a FIFO |
| **Access-width constraints** | A 32-bit-only register must reject a byte write, not silently accept it. Region-level constraints are the coarse filter; a register block with per-register rules enforces its own, so `AccessConstraints` is a *fast reject*, not the whole guarantee |
| **Per-region endianness** | Big-endian device on a little-endian bus is normal, not exotic |
| **Fallible access** | Unmapped read returns a bus fault the CPU can turn into an exception, not `0xFF` guesswork. Per-space "unassigned" policy: fault / read-as-ones / read-as-zeros / log. `MemResult` is `Ok` / `Fault` / `Retry`, where **`Retry` is only legal before any side effect or partial transfer** — a retry that re-runs a half-completed multi-byte access is a correctness bug, so the dispatcher rejects it after first commit |
| **Topology generation counter** | Every cache (TLBs, JIT translation blocks, direct pointers) invalidates on remap |
| **Dirty-page tracking** | Framebuffer refresh, self-modifying-code detection, live snapshot |

**Two kinds of change, and only one is expensive.** This distinction is
load-bearing. An MMC3 cartridge rebanks on nearly every scanline — ~15 000 times
a second — and PC firmware reprograms every BAR twice during enumeration. If each
of those rebuilt a flat view and invalidated every translation block, the NES
would be a slideshow and the PC would take minutes to boot.

| | **Rebase** | **Retopology** |
| --- | --- | --- |
| What changed | an alias's `offset` slides; the region *set* is identical | regions added, removed, resized, re-prioritized |
| Examples | cartridge bank switching; any fixed aperture whose *contents* slide | BAR programming (the mapping **moves**), enable/disable, hotplug, ROM shadowing toggle, realize |
| Cost | one atomic store per affected dispatch entry | full flatten + rebuild |
| Generation counter | untouched | bumped |
| JIT invalidation | only if bytes under a page holding translations changed | all caches for the affected range |

Bank switching must be rebase-shaped, which means `Alias` holds its `offset` in
an atomic cell rather than as a frozen enum payload. Design this in phase 1;
phase 3 depends on it.

**Topology is reached through a guard, not `&mut self`.** The first
implementation took `&mut self` for `map`/`unmap`/`remap`, which made the
rebase/retopology split borrow-checker enforced — and also made §4.7's BAR
write from inside an MMIO handler *impossible*, because a space shared with
devices can never be borrowed mutably again. So the mutable half sits behind one
`core::sync` lock at `TOPOLOGY`, with `SpaceView` (read guard, carries `rebase`)
and `TopologyGuard` (write guard, the only route to `map`/`unmap`/`remap`).
The distinction survives; the impossibility does not.

Two consequences that must be designed around rather than discovered:

- **The access path takes the lock non-blocking.** `TOPOLOGY` sits above `BUS`,
  and a CPU holds `BUS` across every access, so a blocking read guard there
  would close a deadlock cycle — a retopology may take `BUS` locks *underneath*
  `TOPOLOGY`. An access that meets a retopology in flight therefore returns
  `BusError::Retry`. **Known hazard:** a 6502 has no bus-retry input; it returns
  the open-bus latch and counts a fault. So until the §4.7 safe-point protocol
  exists, a retopology racing a guest access silently yields open bus rather
  than stalling. Safe-points are what make this unreachable, and they are not
  built yet.
- **A cross-space retopology is two steps, not one.** Two guards at the same
  rank cannot be held together, so mapping a cartridge into the CPU and PPU
  spaces is sequential and non-atomic. The alternative — a rank per space —
  would mean no ladder at all.

**A BAR *address* change is not a rebase** — an earlier revision of this table
said it was, and that was wrong. Moving a mapping changes the addresses in the
sorted flat view and invalidates every cache keyed on the old address, so it is
a retopology. The genuinely cheap case is the other one: a *fixed* aperture
whose contents slide underneath it, which is exactly what a cartridge mapper
does and exactly why the NES needs this. Enforce the distinction in the type
system rather than by convention — a rebase should not be able to reach
topology.

**Dispatch.** On retopology, the container tree is flattened into a sorted,
non-overlapping `FlatView`. Lookup is two-level: a page-granular dispatch table
(dense `Vec` for the low 4 GiB, radix trie above) yields either a **host pointer
+ length** (the RAM fast path — no virtual call, no bounds walk) or a `FlatView`
index (the slow path).

Two costs to budget rather than discover. A dense page table over the low 4 GiB
is ~10⁶ entries × 16 B = **16 MiB per address space**, and §4.1 mandates a
separate space per bus master — a dozen masters is ~200 MiB, and on `wasm32` it
is 16 MiB out of a 32-bit linear memory. So the dense table is **opt-in per
space**, chosen by the machine file or by a heuristic on the space's realized
extent; small spaces use the sorted flat view directly. And **page granularity
cannot express every mapping** — the NES maps APU registers at `$4000` for 32
bytes, PC I/O ports are byte-granular — so an entry must be able to say
"sub-page: consult the flat view". Which is also the honest reason the NES gets
no benefit from a dense table at all.

**The software TLB is unconditional**, not an MMU-only feature; a CPU without an
MMU (6502, SM83) gets an identity-mapping one. This is not ceremony. The write
path needs a writable bit for dirty tracking to work at all — framebuffer
refresh, self-modifying-code detection, watchpoints — and a bare host-pointer
write sets no dirty bit and can carry no hook. Host signals are forbidden (wasm
has none, §4.7), so a software check on the write path is the only mechanism
available. Reads may still go straight through a host pointer. See §9.

*Note on gones.* The `memory.Bus` in `../gones` OR-combines the results of every
handler mapped at an address and logs a "bus conflict". That is the correct
model for an open-bus system like the NES and the wrong default for PCI. In
rsemu this becomes a per-container `CombinePolicy { Priority, WiredOr, WiredAnd,
Conflict }` — the NES keeps its behaviour, everyone else gets deterministic
priority.

### 4.2 Time, clocks, and scheduling

Generalizes the `../gones` master-clock-plus-dividers model — which is the right
shape — to a **clock domain forest**: one tree per physical oscillator. That
shape is the whole design, because it is what decides where exactness is
meaningful.

- **Clock domains.** `ClockDomain { parent, mul: u64, div: u64 }`. A domain's
  root is an **oscillator** — a declared crystal, not just "a frequency".
  Domains can be reparented, re-rated and gated at runtime: a PLL, a guest
  reprogramming a divider, a power-managed peripheral, a CPU that halts.
- A machine has as many roots as the real board has crystals. One for a Game
  Boy. Two for a SNES (the 21.47 MHz master and the SPC700's own ~24.576 MHz
  can). A dozen for a PC.

#### Exactness is a property of sharing a crystal

The two things people want from emulated time are not two *modes* to choose
between — they are two *physical situations*, and which one applies is decided
by the machine's topology, not by a preference.

**Within one oscillator's tree, the ratios are exact and guest-visible.** On the
NES, the CPU is master ÷ 12 and the PPU is master ÷ 4. Both counters descend
from the same crystal, so the PPU advances exactly 3 dots per CPU cycle —
forever, with no drift, on every console ever made. Games depend on this
absolutely: they alter memory based on the current scanline, and sometimes on
the pixel position within a scanline. A one-dot error is a visibly wrong frame.

Note what this means: **the CPU:PPU relationship is exact regardless of what we
believe the master frequency to be.** It is 3:1 because of the divisors, not
because of the hertz. Getting the absolute frequency slightly wrong makes the
console run imperceptibly fast or slow; getting the *ratio* wrong breaks the
game. Only the second one matters, and it costs nothing to be exact about.

**Across independent oscillators, exactness is not merely expensive — it is
meaningless.** Two crystals have independent tolerances (±20–100 ppm typical),
independent temperature coefficients, and an arbitrary power-on phase. Real
hardware does not have a fixed phase relationship between them, it varies
between individual units, and therefore no correct software can depend on one.
Emulating such a relationship "exactly" would be emulating a precision the
hardware never had. The SNES is the standard example: its CPU and its SPC700
audio unit run from separate cans, real consoles vary audibly, and both
locking them and not locking them break different games — because the truth is
that the relationship is genuinely loose.

#### What this makes the implementation

| | Within one oscillator tree | Across oscillator trees |
| --- | --- | --- |
| Representation | per-domain `u64` tick counters, **authoritative architectural state** | a global fixed-point timeline (2⁻⁶⁴ s), used only for ordering |
| Ratio arithmetic | exact integer multiply/divide over the divisors — small numbers | reciprocal multiply + a per-root **residual accumulator** |
| Error | **none, by construction** | bounded < 1 unit, non-accumulating, and physically justified |
| Cost | `u64` mul/div on values like 12 and 4 | ticks→time is a `u64` mul + shift; **time→ticks is a 128×64 product and a 192-bit division** — `core` has no `u256`, so this needs a multi-limb helper, and the inverse must be the true inverse of the forward map or a tree can never reach its own reported time |
| Runtime re-rating | recompute the tree's small internal lcm | free |

The critical simplification: **the unit is derived per tree, from the rates
inside that tree — never across trees.** This is what makes the exact path
cheap, and it dissolves the failure mode of a global lcm, where adding one
ordinary 32.768 kHz RTC crystal multiplies the unit by 10⁵ and overflows.

The derivation, stated correctly: with each domain's rate reduced to
`root × aᵢ/bᵢ`, the unit rate is `root × A` where **`A = lcm(aᵢ)` — an lcm over
the rate *numerators*** — and domain *i* advances one tick per
`kᵢ = (A/aᵢ) × bᵢ` units. For the NES both numerators are 1, so `A = 1`, the
unit *is* the master tick, and `k_cpu = 12`, `k_ppu = 4` — giving exactly 3 PPU
dots per CPU cycle by construction.

> An earlier revision of this paragraph said "lcm(12, 4) = 12, i.e. the master
> tick", taking the lcm over the *divisors*. That is wrong and points the wrong
> way: a unit of 12 master ticks cannot represent a 4-master-tick PPU period at
> all. The conclusion was right for the NES only because its numerators happen
> to be 1, which is exactly what made the error invisible.

Two consequences worth stating:

- **Absolute frequencies may be irrational and it does not matter.** The NES
  master is 236250000/11 Hz = 21477272.72… and the PC's PIT is 105000000/88 Hz.
  Neither is an integer. The DSL therefore takes **rational frequency literals**
  (`osc master = 236250000/11 Hz`), and that value is used only for cross-tree
  conversion and wall-clock rate control — never for the intra-tree ratios that
  games actually depend on.
- **Per-domain tick counters are the state that gets snapshotted**, not a
  derived absolute time. Restoring is exact because the counters are exact; the
  global timeline is recomputed from them.

#### Overriding the default

Topology decides the default, but a machine may override it with a stated
reason:

- **Lock two oscillators** into an exact declared ratio (`lock spc700 = master *
  a / b`). Useful for regression determinism, and for the SNES case where a
  chosen fixed ratio is the pragmatic answer even though hardware is loose.
- **Relax a tree to best-effort** where a subtree's precision is irrelevant and
  its rate is awkward — a USB frame timer inside an otherwise exact machine.

Both are declarations in the machine file. Neither ever happens silently: if a
tree's internal lcm cannot be computed (a guest programs a PLL to an arbitrary
ratio at runtime), realize — or the write handler — **fails with an error naming
the domains**, and the file must say what to do instead. A timing model that
quietly degrades is worse than one that refuses.

#### Precision is orthogonal to determinism

Both paths are integer-only and both are bit-reproducible. Cross-tree
best-effort is *deterministic* — the residual accumulator makes it a pure
function of the tick counts — it simply does not claim a precision the hardware
lacks. Non-determinism enters only from the *source* of time: the `accel`
threading mode below slaves virtual time to the host clock. That is a separate,
explicitly-labelled choice.

The oscillator topology is part of the machine's identity and is recorded in the
snapshot header, since queued deadlines are meaningless without it.

#### Scheduling

- **Event queue.** A hierarchical timing wheel for the dense near term plus a
  binary heap for far-future events. Events carry a monotonically increasing
  sequence number so ties break deterministically.
- **Execution budgets.** A CPU is never "stepped one instruction" by the
  scheduler; it is handed a budget ("run until virtual time T or 10 000 ticks,
  whichever first") and reports back how much it consumed. This is what makes
  JIT block execution and cycle accounting coexist.
- **Sync-on-access (catch-up).** The event queue handles *scheduled* behaviour —
  the PPU raises NMI at a known dot. It cannot handle *sampled* behaviour: the
  6502 reads `$2002` at an arbitrary cycle and the PPU must be at exactly that
  dot, sprite-0 flag and vblank race included. So a device may declare itself
  **lazily advanced**: it holds a current tick, and the address space calls
  `advance_to(now)` before dispatching any access to it.

  This is what makes execution budgets safe. Without it, a 10 000-tick budget
  means every status-register read is thousands of cycles stale and the
  split-screen status bar in almost every NES game is wrong. `../gones` has this
  (`clock/listener.go`); the generalization here originally dropped it, and it
  would have resurfaced as an unexplainable phase-3 rendering bug.

  Catch-up is bounded by the device's own next scheduled event, so it never
  simulates past a point where its behaviour would change. A `MemAttrs::debug`
  access advances nothing.
- **A budget rarely lands on a tick boundary, and that has to be decided.** An
  event at PPU dot 82181 falls two-thirds of the way through a CPU cycle. The
  rule: **stop at the cycle boundary before, never drag a domain mid-cycle** —
  dragging permanently shifts a domain's phase against its own crystal, which is
  precisely the exactness §4.2 exists to protect. An event dispatcher that needs
  a device *on* a particular tick asks for it explicitly. This was unaddressed
  until the implementation forced the question.
- **Threading modes**, selectable per machine:
  - `deterministic` — one host thread, round-robin over CPUs with a fixed
    quantum. Required for record/replay and the regression suite.
  - `parallel` — thread per CPU with a rendezvous barrier per quantum. Fast,
    non-deterministic, the default for interactive use.
  - `accel` — CPUs run in hardware (§10); virtual time is slaved to the host
    clock and the scheduler becomes a deadline service.
- **Rate control.** `realtime` (throttle to wall clock, with catch-up limits and
  frame pacing), `unbounded` (as fast as possible), `fixed-ratio` (2× slow for
  debugging).

### 4.3 Wires: interrupts and GPIO

```rust
pub trait WireSink: Send + Sync {
    fn set_level(&self, src: WireId, line: u32, level: Level);
}
pub struct Wire { /* per-source level state + fan-out */ }
```

**The `src` is not optional.** Without it a sink cannot implement wired-OR: when
the APU deasserts IRQ while the cartridge still asserts it, a sink that only
knows "someone said low" drops a line that should stay high. That is the classic
shared-interrupt bug, and it is unfixable after the fact because the information
was never passed. Either the sink tracks which sources are asserting, or every
fan-in must go through an explicit `wire.or` device — and then a machine file
that wires two sources to one sink is a **resolver error**, not a silent
mis-wiring. rsemu does the former, and the DSL's implicit fan-in (§5) is sugar
that the resolver expands into an explicit combiner.

**Realize must sweep the graph.** An undriven wire sits low, which contradicts
an inverter's idle-high output, so a freshly realized *or freshly restored*
machine is inconsistent until every gate drives what its inputs imply. Reset
therefore walks wire sources in topological order and announces their levels.
Skip it and interrupt lines come up wrong on some machines and only on some
paths — the worst class of bug to find later.

**One edge of every wire cycle must be weak.** A real IRQ/ack loop is cyclic,
and a graph wired with strong references is necessarily acyclic, so the realize
protocol states which edge is weak: the machine owns devices, and a wire merely
refers to them.

**Which cycles the resolver rejects**, since "reject wire cycles" and "a real
IRQ/ack loop is cyclic" plainly conflict: a cycle is an error only when *every*
device in it is **combinational** — forwarding levels with no state
(`wire.not`, `wire.or`, `wire.and`, `wire.split`). A cycle through a
**sequential** device (`wire.level-to-edge`, and any device model, which is
sequential by default) is a legitimate handshake and is accepted. That is
exactly the condition under which the realize sweep above has a topological
order, so the cycle check and the sweep ordering are **one computation** rather
than two rules that could disagree.

Level and edge semantics both, with the *edge detector as a device* rather than
a flag, so it snapshots correctly. Ships with the standard combinators as
ordinary devices: `wire.split`, `wire.or`, `wire.and`, `wire.not`,
`wire.level-to-edge`. Interrupt controllers (i8259, APIC, GIC, PLIC, NES NMI
line) are then just devices with wire sinks and sources — the core knows nothing
about "interrupts".

### 4.4 Devices, properties, registry

```rust
pub trait Device: Send + Sync {
    fn class(&self) -> &'static DeviceClass;
    fn realize(&self, ctx: &mut RealizeCtx) -> Result<()>;   // wire up, map regions
    fn unrealize(&self, ctx: &mut RealizeCtx) -> Result<()>; // unmap, unwire, cancel
    fn reset(&self, kind: ResetKind);                        // Cold | Warm | Bus
    fn save(&self, w: &mut StateWriter) -> Result<()>;
    fn load(&self, r: &mut StateReader) -> Result<()>;
}

/// A device that performs its own accesses: DMA engines, bus masters,
/// host controllers, and every CPU. Handed out at realize time.
pub trait Initiator {
    fn space(&self) -> &AddressSpace;   // *its* view, not the CPU's
    fn id(&self) -> RequesterId;        // travels in MemAttrs for IOMMU/ACS
}
```

**Devices must be able to initiate.** A device that can only *respond* cannot
model NES OAM DMA (`$4014`), the DMC sample fetch, an 8237, any PCI bus master,
virtio descriptor fetch, a USB controller walking transfer rings, or AHCI/NVMe
queue processing — which is to say, two devices in phase 3 and most of them from
phase 6 on. `RealizeCtx` hands a device an `Initiator` bound to the address
space its DMA actually traverses, which is where §4.1's per-master spaces stop
being theoretical.

- **Two-phase construction.** `new(props)` validates properties and allocates;
  `realize(ctx)` performs every outward action (mapping regions, connecting
  wires, attaching to buses). Nothing observable happens before realize, so the
  config resolver can build the whole graph and fail cleanly.
- **Composition.** Devices own child devices. A `pc.q35` device instantiates its
  own chipset children; the config only names the top level unless it wants to
  reach in.
- **The connection surface is on `Device` itself** — `region`, `sink`,
  `connect`, `announce`, `combinational`, `is_runnable`, `run`, `event`, all
  defaulted. It cannot be a second trait beside it: there is no route from a
  `dyn Device` to another trait object without `Any` in the supertrait chain,
  and the machine layer's first attempt cost a second registration table that
  nothing kept in step with the registry. Defaults are chosen so silence is
  safe — in particular a device is **sequential** until it says otherwise,
  because claiming to be combinational when you are not turns a legitimate
  IRQ/ack handshake into a machine the resolver rejects.
- **Property system** (`core::props`): a small dynamic `Value` — int, uint,
  bool, string, size (`512M`), address, duration, list, map, and **link**
  (a reference to another object) — with typed extraction and precise error
  messages. Deliberately not `serde`: the dependency policy forbids it, the
  value set is small, and the error messages matter more than the generality.
- **Registry** (`core::registry`): by-name construction. `compcol::factory` is
  the precedent for the **naming convention only** — it is a compile-time
  `match` over feature-gated arms with no registration API, where rsemu needs a
  mutable registry. `registry::create("pci.nvme", props)`. Registration is explicit per
  feature (`#[cfg(feature = "dev-nvme")] reg.add(NVME_CLASS);`) — no
  link-time-magic crate. The registry is also the introspection surface:
  `rsemu devices` / `rsemu describe pci.nvme` prints classes, properties,
  defaults and bus requirements, and the doc generator reads the same data.

### 4.5 State: snapshots, replay, rewind

Built in phase 1, not bolted on later.

- **Format.** Chunked and versioned: a machine header (structural fingerprint,
  feature set, guest arch list), then one chunk per device instance **keyed by
  instance path, carrying the class name and class version as attributes**.
  The version must *not* be part of the key: if it were, bumping a class
  version would make the old chunk unfindable and the migration chain below
  unreachable — the class is verified on load and the version feeds migration.
  Loading a snapshot into a differently-shaped machine fails with a diff, not a
  crash.
- **Content.** Devices serialize *architectural* state only. Derived caches
  (TLBs, translation blocks, flattened views, host pointers) are rebuilt on
  load. A device whose `save`/`load` round-trip does not reproduce an identical
  state hash fails its own unit test.
- **The scheduler is architectural state**, and is easy to forget. The event
  queue, every domain's tick counter, the cross-tree residual accumulators, and
  the tie-break sequence counter all go in the snapshot. Re-deriving events by
  asking devices to re-register loses sub-tick phase, and every timer then fails
  its own round-trip test.
- **Guest RAM** is the bulk of any snapshot and needs its own format decision,
  not an afterthought: page-indexed with a dirty-log-driven incremental mode, so
  that rewind (§4.5) and live snapshot cost proportional to what changed rather
  than to RAM size. An 8 GiB phase-7 guest makes this the entire cost model —
  and `compcol`'s zstd encoder is currently ~0.15× reference speed on
  incompressible data, which guest RAM largely is, so compression is opt-in and
  measured rather than assumed.
- **Storage is snapshotted with the machine or not at all.** A machine snapshot
  taken while a write-back cache holds dirty blocks, without a matching disk
  snapshot, restores to a corrupt guest filesystem. The atomicity rule and the
  cache-flush contract are decided *before* the first storage controller is
  written (§7.1), not after.
- **Migration across class versions.** A version field with no migration
  mechanism is decoration. Each class may register upgrade functions
  `vN -> vN+1`, and the test is a *cross-version* load from a committed
  fixture — a round-trip test never exercises it. Save states are a headline
  feature of a decade-scale project; the format will change.
- **Machine identity** is a structural fingerprint (device classes, instance
  paths, region layout), not a hash of the config text. A hash gives a boolean
  where §4.5 promises a diff, and invalidates every snapshot when someone edits
  a comment or passes `-p ram=8G`.
- **Layering.** Compression via `compcol` (zstd), integrity via `purecrypto`
  (BLAKE3), optional encryption via `purecrypto` — all feature-gated; the raw
  format works with zero dependencies.
- **Record/replay.** In deterministic mode, log every non-deterministic input
  (host clock reads, RNG draws, network/serial/input events) against a virtual
  timestamp. Replay reinjects them. This yields, for free: reproducible bug
  reports, CI regression fixtures, and **rewind** (periodic snapshot + replay
  forward to an earlier point).

### 4.6 Execution engines

The core does not know what a CPU *is* beyond:

```rust
pub trait Cpu: Device {
    fn run(&self, budget: Budget) -> Consumed;
    fn interrupt(&self, req: InterruptReq);
    fn regs(&self) -> RegView<'_>;               // gdb, monitor, tests
    fn mmu(&self) -> Option<&dyn Mmu>;           // guest-virt → guest-phys
}
```

A core may implement `run` by interpreting, by translating through the IR
(§9), or by entering hardware (§10). The choice is a per-CPU config property
(`engine = "interp" | "jit" | "kvm"`), and **all engines for one guest
architecture must agree instruction-for-instruction** — enforced by differential
testing (§12), which is the only thing that keeps a JIT honest.

### 4.7 Concurrency: the `sync` seam and shared guest memory

Threading is designed in at phase 1, not added at phase 8. Retrofitting
`Send + Sync`, a shareable RAM store, and a safe-point protocol onto a core that
assumed one thread is a rewrite — and the wasm target makes the usual shortcut
(`std::thread::spawn` wherever convenient) unavailable anyway.

**Four independent axes of parallelism.** Only the first changes guest-visible
semantics; the other three must be invisible.

| Axis | What runs in parallel | Guest-visible? |
| --- | --- | --- |
| **Multi-CPU execution** (parallel translated execution) | one thread per guest CPU | **Yes** — needs a memory model and safe points |
| **Background compilation** | JIT tier-up while the interpreter runs the same block | No |
| **Device / host offload** | disk I/O, VNC encode, audio resample, snapshot compression | No, *provided* results land at a virtual time derived from the guest clock |
| **Data-parallel helpers** | framebuffer conversion, hashing, `compcol` compression | No |

#### The `sync` seam

`core::sync` is a portability seam. **No code under `core/`, `cpu/`, `dev/`,
`machine/` or `ir/` ever names `std::thread` or `std::sync` directly.** The seam
exports `Mutex`, `RwLock`, `Condvar`, `Atomic*`, `Once`, and a task pool, with
four compile-time backends selected by target and feature:

| Backend | Primitives | Where |
| --- | --- | --- |
| `native-std` | `std::sync` + `std::thread` | ordinary hosted builds |
| `native-raw` | futex / `WaitOnAddress` by raw syscall | libc-free (`fullrust`) and `no_std` hosted builds |
| `wasm-atomics` | shared linear memory + `Atomics.wait`/`notify` in Web Workers | `wasm32-*` with the threads proposal |
| `single` | locks compile to borrow-checked no-ops; the pool runs jobs inline | no-threads wasm, bare metal, and the deterministic test runner |

Because the API is identical across all four, a device is written once and works
on every target. `single` is not a degraded mode to be tolerated — it is the
**reference semantics**, and CI asserts that a machine produces the same state
hash under `single` and under `native-std`.

**Jobs, not threads.** The seam exposes a *task pool* (`pool.submit(job) ->
Handle`), never `spawn`. This is forced by wasm — a worker cannot be created
synchronously from arbitrary code; the embedder builds the pool up front and
hands it in — and it is better design regardless: thread count becomes a machine
property, work is schedulable, and nothing deep in a device model can quietly
create an OS thread.

#### Shared guest memory

- `RamStore` is addressed by **byte offset, not by `&mut [u8]`**, precisely so
  it can be shared across worker threads without handing out aliasing slices.
  This is the reason for the API shape; do not "simplify" it.
- Native: one allocation behind an `Arc`, with the host-pointer fast path as one
  of the four sanctioned `unsafe` sites (§0).
- Wasm with threads: the allocation must live inside the module's **shared**
  `WebAssembly.Memory` (a `SharedArrayBuffer`), so the same offset arithmetic
  and the same generated-code load/store sequences work unchanged.
- **Guest memory model.** Guest atomic instructions lower to host atomics
  through the IR's atomic ops. Where the guest model is weaker than the host's,
  nothing is emitted; where it is stronger (x86-TSO guest on an AArch64 or
  wasm host), **the frontend lifter inserts the barriers** — the core provides
  the primitives, the lifter owns the ordering. Getting this wrong produces bugs
  that appear only under load on one host architecture, so it is a documented
  per-frontend responsibility with its own test suite (§12).

#### Safe points and stop-the-world

TLB shootdown, memory-topology change, snapshot, reset, and single-step all
require every CPU thread to be quiescent. The protocol is a **generation counter
plus a per-CPU exit flag checked at translation-block boundaries** — never a
host signal, because wasm has none and signals are miserable on Windows.
A CPU that must stop unwinds to the scheduler at the next block edge; the
requester waits on the pool's barrier.

#### Locking discipline

- **The re-entrancy contract, which replaces the naive "never hold a lock"
  rule.** That rule was unimplementable: an MMIO write to a DMA controller's GO
  register *must* issue reads while the handler is running, and a PCI config
  write that moves a BAR remaps memory from inside the device's own write path.
  Forbidding outward calls under a lock forbids the phase-3 machine.

  The contract instead: a device mutates its own state inside a **short critical
  section that must be released before any outward call**. Anything outward — a
  DMA burst, a wire change, a remap, a call into a sibling — happens after the
  release, or is pushed onto the handler's **deferred-action queue** and run by
  the caller once the handler returns. The queue preserves ordering, is drained
  before the access completes, and makes re-entrancy explicit instead of
  accidental. A device that re-enters itself through it gets a diagnosable
  error, not a deadlock under `native-std` and a panic under `single`.
- A ranked lock order is documented in `core::sync` and asserted in debug
  builds; hot paths use atomics rather than locks.
- In `deterministic` threading mode, guest CPUs are serialized, but background
  work is still permitted — it just must deliver results through the event queue
  at a virtual time computed from the guest clock, never from the host's.
  Determinism constrains *when results become visible*, not *where work happens*.

---

## 5. The machine description language

The framework's user interface. It must express arbitrary graphs — including
heterogeneous CPUs sharing memory, multiple disjoint address spaces, and
recursive bus fabrics — and it must produce good errors, because most people
will meet rsemu through a syntax error.

**Format:** a purpose-built declarative DSL (`.machine`), hand-parsed with span
tracking, plus a **lossless JSON projection** for tooling. One AST, two
syntaxes: `rsemu convert` round-trips either direction. JSON alone is
rejected — it cannot carry comments, and comments in a machine file are how the
next person learns why a mirror exists.

```
machine "nes" {
  param region = "ntsc"

  # One crystal, so every domain below is exactly related to every other.
  # The literal is rational because the real frequency is not an integer;
  # it affects wall-clock rate only, never the CPU:PPU ratio.
  osc master = 236250000/11 Hz           # 21477272.72… — NTSC colorburst × 6

  space cpubus  { width = 16, unassigned = open-bus }
  space ppubus  { width = 14, unassigned = open-bus }

  object wram "ram" { size = 2K }        # instance `wram`, class `ram`

  object cpu "mos6502" {
    clock  = master / 12                 # PPU advances exactly 3 dots per cycle
    space  = cpubus
    engine = "interp"
  }
  object ppu "nes.ppu" { clock = master / 4, space = ppubus }
  object apu  "nes.apu"  { clock = master / 12 }
  object cart "nes.cart" { space = cpubus }   # the mapper drives an IRQ below

  map cpubus 0x0000 size 0x2000 = mirror(wram)      # 2K mirrored 4×
  map cpubus 0x2000 size 0x2000 = mirror(ppu.regs)
  map cpubus 0x4000 size 0x0020 = apu.regs

  wire ppu.nmi   -> cpu.nmi
  wire apu.irq   -> cpu.irq
  wire cart.irq  -> cpu.irq                          # wired-OR: see §4.3
}
```

> This example did not resolve until the resolver was written against it. It had
> `object ram "wram"` — instance `ram`, class `wram` — and then `mirror(wram)`,
> which names the *class*; and it wired `cart.irq` without ever declaring a
> cartridge. Both are fixed above and both are pinned by golden tests in
> `src/machine/tests.rs`, so the document cannot drift from the grammar again.
> A worked example nobody has executed is a plausible-looking guess.

Required language features, all driven by "any remotely possible configuration":

- **`param`** with defaults and CLI/env override (`rsemu run nes.machine -p ram=4M`).
- **`include`** with a search path, so `pc-q35.machine` can pull in
  `pci-common.machine`.
- **`template`** — parameterized reusable subsystems, instantiated N times.
  This is how you get four identical CPU complexes, or two PCI segments.
- **Loops / indexed instantiation** for `for i in 0..4 { object cpu$i … }`.
- **Explicit edges.** Memory maps and wires are *statements*, not properties
  buried inside objects. The graph must be readable by scanning the file.
- **Multiple address spaces and multiple CPUs of different classes** sharing
  regions — the motivating case, and therefore a test fixture from day one
  (`machines/tests/heterogeneous.machine`: a 6502 and a RISC-V core sharing one
  RAM region through two spaces with different endianness).

**Pipeline:** lex → parse (spans preserved) → resolve (names, links, params,
includes; detect cycles) → validate (does this device class exist? does it take
this property? is this bus type compatible?) → realize (construct, wire, map) →
run. Errors carry file:line:col and a caret, always.

---

## 6. CPU cores

Each core is a feature. The order below is chosen so that each one proves a
*new mechanism* in the framework rather than adding another opcode table.

| Core | Proves | Phase |
| --- | --- | --- |
| **MOS 6502** (+ illegal opcodes, 2A03 variant) | Cycle-accurate interpretation, bus timing, the whole core is exercised end to end | 3 |
| **SM83** (Game Boy) and **Z80** | That the framework is not 6502-shaped; different interrupt model, I/O space | 4 |
| **RISC-V rv64gc** (+ rv32) | MMU + software TLB, privilege levels, atomics, FPU, and the IR/JIT path. Smallest ISA that boots real Linux | 5 |
| **x86**: i386 → x86-64 (real/protected/long mode, SSE) | The hard one: segmentation, variable-length decode, self-modifying code, paging quirks | 6 |
| **ARM**: ARMv7-A, ARMv8-A AArch64 | Second major JIT frontend; validates IR generality | 6–8 |
| **ARMv6Z** (ARM1176JZF-S) | Additive over v5TE: media SIMD, LDREX/STREX, VMSAv6, TrustZone. Raspberry Pi 1 / Zero; see §6.1 | planned |
| **ARMv7E-M** (Cortex-M4/M7) | Thumb-2 only, a wholly different exception model; see §6.1 | planned |
| Later: 68000, MIPS, PowerPC, SuperH, 8080, 65816, V850 | Breadth; each is a weekend once the IR is stable | post-8 |

### 6.1 ARM: one module or several?

Planned, not built. **ARMv7E-M gets its own module, `src/cpu/armv7m/`, beside
`src/cpu/arm/` rather than inside it** — and that is a different answer from the
one the 6502 got, for a reason worth stating.

The 6502's three parts share one architectural model: NMOS, RP2A03 and W65C02S
differ in about a tenth of the instruction table and in nothing else, so
`Variant` as a construction property was right and one table macro serves all
three. ARMv5TE and ARMv7E-M do not share a model:

| | ARMv5TE | ARMv7E-M |
| --- | --- | --- |
| A32 (ARM state) | the bulk of the core | **does not exist** |
| T32 (32-bit Thumb-2) | does not exist | the bulk of the core |
| Conditional execution | a field in every A32 instruction | `IT` blocks |
| Privilege / modes | seven modes, banked registers, SPSR | Handler/Thread, MSP/PSP, no banking |
| Exception entry | mode switch, banked LR, vector *instructions* | automatic register stacking, `EXC_RETURN`, vector *addresses* |
| Interrupt controller | external, whatever the SoC provides | NVIC, architecturally specified |
| System registers | CP15 coprocessor | memory-mapped SCB / SysTick / MPU |

A `Variant` flag across that is `#[cfg]` wearing a different hat: nearly every
function would branch on it, and a Cortex-M build would link ~1,900 lines of A32
decode it can never execute. That breaks the crate-shape rule (§3) directly — a
NES build links a 6502 and nothing else, and a Cortex-M build should link no
ARM state.

**The target layout.** `arm/` becomes the family, and the cut is **by profile,
not by version**:

```
src/cpu/arm/
  mod.rs       — family types and re-exports; always compiles, links nothing
  common/      — shifter, flag rules, DSP semantics, Thumb-1        (LATER)
  aprofile/    — A32 + Thumb, banked modes, CP15, an MMU
                 Arch::{V5TE, V6, V6Z, V7A}                  (cpu-arm-aprofile)
  v7m/         — Thumb-2 only, Handler/Thread, NVIC              (cpu-arm-v7m)
```

An earlier revision of this section put ARMv5TE in a `v5te/` directory. That was
wrong the moment ARMv6 entered the plan, and the correction is worth recording
rather than quietly applying: **ARMv6 is additive over ARMv5TE, while ARMv7E-M
is a different machine.** Counting major areas, v6 shares all ten of v5TE's —
A32 plus Thumb, seven banked modes, CPSR/SPSR, CP15, the exception model, the
shifter, the DSP extensions, the multiply family, `LDM`/`STM` with the S-bit,
condition codes on every instruction — and adds ten of its own. v7E-M shares
none of those first ten.

So v5TE, v6, v6Z and later v7-A belong in **one core with an `Arch`
construction property**, exactly as the 6502's three parts do, because the
alternative is three near-copies drifting apart. A directory named `v5te/` that
also implements v6 and v7-A would be lying, so the directory is named for the
profile.

**ARM1176JZF-S (ARMv6Z)** is the concrete target, and the ten additions are:
the SIMD media instructions (`UADD8`, `SADD16`, `USAT`/`SSAT`, `SEL`, `PKHBT`/
`PKHTB`, `USAD8`), `REV`/`REV16`/`REVSH` and the extend-with-rotate family,
`LDREX`/`STREX` (with the v6K byte/halfword/doubleword forms and `CLREX`),
`CPS` and `SETEND`, architectural unaligned access under `SCTLR.U`/`A`, the
**VMSAv6 MMU** — supersections, ASIDs, TEX remap, and a genuinely different
descriptor format from v5's — `WFI`/`WFE`/`SEV`/`YIELD` as real instructions
rather than hints, **TrustZone** (the `Z`: Monitor mode, and secure/non-secure
banking of much of CP15), **VFPv2** (the `F`), and Jazelle (the `J`, trivial on
real silicon and fine to stub as such — say so).

The MMU and TrustZone are the substantial parts; the instruction additions sit
in existing encoding gaps and are mostly mechanical. The natural machine to aim
at is the **Raspberry Pi 1 / Zero (BCM2835)** — a PL011 UART, the system timer,
the mailbox and GPIO — which is to ARMv6 what `virt` is to RISC-V: a real board
with real firmware to boot rather than a synthetic one.

**Conformance.** No `SingleStepTests` corpus for ARMv6 either, so the same three
substitutes as §6.1's v7-M discussion apply, with one addition that is stronger
here: **our own ARMv5TE core is a near-complete oracle**, because v6 is a
superset. Every v5TE instruction must behave identically unless the architecture
says otherwise, and the divergences are enumerable (unaligned access, `SWP`
deprecated in favour of `LDREX`/`STREX`, the CP15 register map). Differential
testing against a core that passes 2,200,000 corpus vectors covers far more of
ARMv6 than it does of ARMv7E-M. The ARM7TDMI corpus also still validates the
shared v4T subset directly.

### 6.1.1 Planning for many variants: an extension lattice, not a version ladder

We will implement a lot of ARM. That changes the design, and the change is worth
stating before the second core lands rather than after the fifth.

**ARM versions are not a linear chain, and `if version >= V6` is a bug.** The
architecture is a lattice of *optional extensions* that appear, become
mandatory, and occasionally disappear along paths that do not nest:

- **Thumb (T)** is optional in v4, mandatory from v6.
- **DSP (E)** arrives in v5TE, but plain ARMv5 (ARM9TDMI) does not have it —
  so a v5 part can lack what an earlier-numbered part with an extension has.
- **Jazelle (J)**, **Security/TrustZone (Z)**, **VFP (F)**, **NEON**,
  **hardware divide**, **LPAE**, **Virtualization (H)** and **multiprocessing
  (MP)** are each independently present or absent within a single version.
  ARM1176JZF-S has Z and VFPv2; ARM1136J-S has neither, and both are ARMv6.
- **Thumb-2** appears in v6T2 — *after* v6 and v6K, which lack it.
- The **M profile** has Thumb-2 but no A32 at all, so it is not "v7 minus
  things"; it is a different branch of the lattice entirely.

Encoding a lattice as an ordered enum forces every decode site to hard-code the
part number it was thinking of, and the bug surfaces as an instruction that
silently exists on a core that never had it. So:

```rust
pub struct Arch {
    profile: Profile,        // A | R | M
    version: Version,        // V4, V4T, V5TE, V6, V6K, V6T2, V7, V8 …
    ext:     Extensions,     // independently selectable, below
    mem:     MemModel,       // None | VMSAv5 | VMSAv6 | VMSAv7{lpae} | PMSA{v6,v7,v8}
}

pub struct Extensions {
    pub thumb: bool, pub thumb2: bool, pub dsp: bool, pub media: bool,
    pub jazelle: bool, pub security: bool, pub virt: bool,
    pub vfp: Option<VfpVersion>, pub neon: bool,
    pub idiv_thumb: bool, pub idiv_arm: bool, pub mp: bool, pub excl: ExclKind,
}
```

This is not a new idea in this crate: **`cpu/riscv` already does exactly this**,
with a `Config` carrying independently selectable `m`/`a`/`f`/`d`/`c`/`s`/`u`
rather than an "RV64GC or not" flag. ARM's need is stronger, not weaker.

**Named presets are the public surface.** Nobody should assemble an
`Extensions` by hand to get a real chip; the constants carry the part numbers,
and a machine file names one:

```rust
Arch::ARM7TDMI      // v4T:   thumb
Arch::ARM926EJ_S    // v5TE:  thumb dsp jazelle,           VMSAv5
Arch::ARM1136J_S    // v6:    …media,                      VMSAv6
Arch::ARM1176JZF_S  // v6Z:   …security vfp(V2),           VMSAv6   ← Pi 1 / Zero
Arch::CORTEX_A8     // v7-A:  …thumb2 neon vfp(V3) idiv_t, VMSAv7
Arch::CORTEX_R5     // v7-R:  …                            PMSAv7
Arch::CORTEX_M0     // v6-M:  thumb2(subset),              PMSAv6
Arch::CORTEX_M4F    // v7E-M: thumb2 dsp vfp(V4-SP),       PMSAv7
```

Adding a part is then a `const` and its conformance evidence, not a module.

**Decode is gated per entry, not per core.** §6's rule is that instruction
tables are generated from a declarative description in the same file; each entry
gains a *requirement* — the extension set it needs — and the generator emits a
table filtered against the configured `Arch` at construction time. An
instruction that is absent must trap as UNDEFINED, exactly as the silicon does;
"we decoded it anyway because the core supports v7" is a conformance failure and
also the way real guests probe for features. Because the requirement lives
beside the semantics, adding an extension cannot silently light it up on parts
that never had it, and the disassembler the same generator emits stays honest
about which core it is disassembling for.

**Where the profiles split.** Extensions handle variation *within* a profile.
The A/R and M profiles differ in the parts an extension flag cannot express —
register banking versus a stack-pointer pair, CP15 versus a memory-mapped SCB,
CPSR versus xPSR, an eight-entry vector table versus a relocatable NVIC vector
array — so those stay separate cores (`aprofile/`, `v7m/`) sharing `common/`.
A/R and M is one boundary, and one is the number we should be prepared to
defend; a third appears only if AArch64 lands, which shares even less.

**The order to build in**, each step reusing the last rather than copying it:
v5TE (done) → v6/v6K → v6Z (ARM1176JZF-S, the Raspberry Pi target) → v6T2 and
v7-A. VFP and NEON ride on §9.1's soft-float, which already exists for RISC-V's
`f`/`d` and is the reason none of this needs a host FPU.

**What is genuinely shared, and when to factor it.** The barrel shifter and its
flag rules, the DSP (E) semantics (`QADD`, `SMLAxy`, `SMUAD`, the SIMD
add/sub family), and the 16-bit Thumb-1 encodings, which ARMv7-M inherits
almost wholesale. Perhaps a third of the work.

**Do not extract that up front.** Factoring shared code out of one
implementation, before the second exists to disagree with it, is guessing at
the seam — and the guess is usually wrong in the direction that hurts. Build
`armv7m` standalone, let the duplication become real and visible, then extract
against two working consumers. The cost of waiting is some duplicated
arithmetic for one release; the cost of guessing is an abstraction both cores
have to fight.

**Conformance is the open problem.** There is no `SingleStepTests` corpus for
ARMv7-M, and Arm's own architecture validation suite is not public — so the
approach that produced trustworthy numbers for the other five cores is
unavailable. Three substitutes, in descending order of what they prove:

1. **Differential against our own ARMv5TE core on the shared Thumb-1 subset.**
   That core passes 2,200,000 corpus vectors, which makes it a real oracle for
   the overlap rather than a peer opinion. This is the one worth building.
2. **Built test binaries.** `clang` targets `thumbv7em-none-eabi` directly, so a
   corpus can be assembled the way `riscv-tests` is — a small ELF per feature
   signalling pass or fail. Covers T32 and the exception model, which (1) cannot.
3. **Real firmware.** Booting a CMSIS or Zephyr image proves integration, not
   correctness, but it finds the things unit tests never do.

Absent (1) and (2), an ARMv7E-M core would be self-validated, and §0 is explicit
that a core with no suite is untested rather than done. Plan for the corpus
before the core.


Every core provides both an **interpreter** and (from phase 5 onward) an **IR
frontend**, and the two are differentially tested against each other forever.

---

## 7. Buses and devices

Generic `Bus` trait (attach/detach, enumeration, address routing, hotplug),
with concrete fabrics as features:

- **PCI / PCIe** — config space, BAR sizing and mapping into address spaces,
  capabilities, MSI/MSI-X, bridges, multiple segments, SR-IOV later. PCI is the
  hardest test of the region-priority model; if BARs map cleanly, the memory
  design is right.
- **USB** — host controller ↔ device model, endpoints, transfer queues; UHCI,
  EHCI, xHCI; HID, mass storage, hub, serial, audio. Optionally bridged to real
  hardware via the existing `usbmagic` work.
- **Low-speed fabrics** — I2C/SMBus, SPI, 1-Wire, GPIO controllers, MDIO.
- **Storage transports** — IDE/ATA, AHCI, NVMe, SCSI, SD/MMC, virtio-blk.
- **virtio** — transport-agnostic core (virtqueues, feature negotiation) with
  PCI and MMIO transports: blk, net, rng, console, balloon, gpu, 9p/fs.
- **Interrupt controllers, timers, RTC, DMA controllers, UARTs** — the
  unglamorous majority.

### 7.1 Storage

**Largely solved by [`fstool`](https://github.com/KarpelesLab/fstool).** It
already provides the `BlockDevice` trait (`Read + Write + Seek + Send`), file /
memory / sliced backends, **qcow2**, DMG, MBR/GPT/APM partition tables, and
read-write implementations of ext2/3/4, FAT12/16/32, exFAT, NTFS, XFS, HFS+,
F2FS, littlefs, SquashFS and ISO9660. Emulated storage controllers sit directly
on `fstool::BlockDevice` rather than on a parallel rsemu invention.

What rsemu adds on top: the remaining image formats (`vmdk`, `vhdx`, `vdi`),
copy-on-write overlays and image snapshots tied to machine snapshots (§4.5),
discard/TRIM, and a write-back cache whose flush contract survives snapshotting.

What this buys the user directly: `rsemu run --disk-from-dir ./rootfs` builds a
bootable image on the fly; the monitor can inspect and edit a guest disk without
booting it; and CI fixtures generate their own FAT/ext4 boot media with no
external tools and no `mkfs`. `fstool`'s `crash_inject` block device also gives
guest-filesystem robustness testing for free.

### 7.2 Networking

**Solved by `pktkit`.** Every emulated NIC (`e1000`, `rtl8139`, `virtio-net`,
`ne2000`, …) is a `pktkit::L2Device`; the config then attaches it to a
`pktkit::L2Hub`, a `slirp` NAT stack, a TUN/TAP device, or a WireGuard/OpenVPN
tunnel with no rsemu-side code. This is a large, already-finished chunk of the
project that most emulators have to write themselves.

---

## 8. Host-facing layer (`host/`, std only)

- **Display** — a framebuffer/scanout abstraction; guest surface → host window.
  Backends: raw framebuffer, X11/Wayland (reusing `x11anywhere` protocol work),
  Win32, macOS, plus headless PNG capture for CI.
- **Remote display** — a built-in **VNC** server (and later SPICE, given the
  existing `spice` / `shells-spice` work). This is the highest-value frontend:
  it costs no GUI dependencies, works over the network, and doubles as the CI
  screenshot mechanism.
- **Audio** — mixer with resampling and a virtual-time-anchored clock; backends
  ALSA/PulseAudio/CoreAudio/WASAPI via raw syscalls where possible.
- **Input** — keyboard/mouse/gamepad with guest-scancode translation tables.
- **Console/monitor** — a `noroi` TUI: device tree, memory map dump (the
  descendant of gones' `Bus::String()`), register views, breakpoints, trace
  control.
- **gdbstub** — the GDB remote serial protocol over TCP: registers, memory,
  breakpoints/watchpoints, multi-CPU as threads, `qXfer` target descriptions.
  Debugging a guest kernel is a headline feature, not a nicety.

---

## 9. The translation IR and JIT

The performance story. Design it once, correctly; every guest and every host
pays for mistakes here.

**IR shape** — deliberately small and low-level: ~60 architecture-neutral ops
over typed temporaries (`i32 i64 i128 f32 f64 v128`), SSA within a translation
block, helper calls for anything messy (rare instructions, MMIO, exceptions).
The op set is chosen so that the *common* case of every target ISA lowers to one
or two host instructions, and everything else becomes a helper call rather than
a new op. Design it from the ISA manuals of the guests and hosts we target
(§1) — the op list below is derived from what those instruction sets actually
need, and it is short because breadth belongs in helpers.

- Data: `mov ext trunc bswap deposit extract`
- Arith: `add sub mul div rem neg`, `add2 sub2 mulu2 muls2` (carry chains)
- Logic/shift: `and or xor not andc orc eqv nand nor shl shr sar rotl rotr`
- Bit: `clz ctz popcount`
- Compare/branch: `setcond movcond brcond`
- Memory: `ld st` carrying a `MemOp { size, sign, endianness, alignment, index }`
- Atomics: `cmpxchg fetch_{add,and,or,xor} xchg`, plus fence
- Control: `goto_tb exit_tb lookup_and_goto call_helper`
- SSA glue: `phi` (required — superblocks span branches)
- Vector ops added with the ARM/x86 SIMD work, not before

**Precise exceptions, designed in before a line of the IR is written.** This is
the part that eats binary-translation projects, and it constrains the op set,
the register allocator, and block layout — so it cannot be retrofitted. When a
load faults halfway through a translated block, the guest must observe *exactly*
the architectural state its ISA specifies at that instruction: the right PC, the
right registers, and nothing from instructions that had not yet retired.

The design: the IR carries an explicit **`insn_start` marker** at every guest
instruction boundary, recording the guest PC and the live guest-register→
temporary mapping at that point. The register allocator emits a compact
side-table per translation block keyed by host code offset. On a fault, the
runtime looks up the offset, materializes the architectural state from the
recorded mapping, and delivers the exception at the correct guest PC.

Two policies follow and must be stated per architecture, because they differ:
whether a faulting instruction **restarts** or **resumes** (`rep movsb`, ARM
`LDM`, and a misaligned store spanning a page boundary where only the second
half faults are the cases that decide it), and whether stores are permitted to
retire before an instruction is known to complete. Getting the second wrong
gives a guest torn memory that no real CPU would produce.

**Floating point is helper calls in tier 1, and that is a decision, not an
omission.** The `f32`/`f64` temporaries exist so the IR can *carry* FP values;
the arithmetic goes through helpers into the soft-float implementation (§9.1).
Native FP instruction selection is a tier-2 optimization, available only where
the host can be proven bit-identical to the guest — which is rarer than it
sounds.

**Pipeline.** Guest ISA → frontend lifter → IR → passes (constant folding, copy
propagation, dead-code elimination, liveness, memory-op fusion) → register
allocation (linear scan) → host backend.

**Backends.** `x86_64` first (the dev machine), then `aarch64` and `riscv64`,
then **`wasm`** (§11.4) for the browser, plus a **portable IR interpreter
backend** so an unsupported host degrades in speed rather than failing to run.
Native code buffers are W^X: `mmap` RW → emit → `mprotect` RX, via raw syscalls,
no libc — the `purestd`/`kataan::jit` pattern. The wasm backend has no such
buffer; it emits a module and instantiates it.

**Compilation runs off the emulation thread.** Translation is submitted to the
`core::sync` task pool (§4.7) while the interpreter keeps executing the same
block; the compiled entry is published with a single atomic store. This is the
cheapest large win in the whole JIT and it is only available if the core was
`Send + Sync` from the start — which is the argument for §4.7 landing in
phase 1.

### 9.1 Soft-float — a named deliverable, not an assumption

`ROADMAP.md` §0 claims bit-identical state hashes across hosts and in a browser.
**Guest floating point executed on host floating point does not deliver that**,
and the plan previously assumed it away. x86 and AArch64 differ in NaN payload
propagation and in default flush-to-zero behaviour; wasm canonicalizes NaNs;
and x87's 80-bit extended precision — which DOS and Win9x guests need — exists
on no other host at all.

So cross-host determinism requires a **software IEEE-754 implementation**:
binary32/binary64 with all four rounding modes, correct subnormal handling,
sticky exception flags (`MXCSR`/`FPSR`/`fcsr` semantics per guest), NaN payload
propagation rules, and — for phase 7 — an x87 80-bit extended path with its
own precision-control quirks.

This is a multi-week subproject that phases 6 and 7 both depend on, and no
sibling crate provides it: `puremp` is MPFR-class arbitrary precision with
caller-chosen precision and *no* fixed binary32/64 format, no bounded exponent,
and no IEEE-754 status flags — it cannot emulate a guest FPU. It is scheduled
in phase 6 alongside the RISC-V `F`/`D` extensions, which are its first
consumer.

A host-FP fast path may exist for interactive use where reproducibility is not
required. It is a flag, it is off in deterministic mode, and it is off in CI.

**The mechanisms that actually produce speed** (all in phase 6–9):

1. **Software TLB** — per-CPU, direct-mapped (4096 entries), split by access
   type, entry = `{ guest page tag, host addend | IO slot }`. The fast path is
   inlined into generated code: mask, compare, add, load. Everything else about
   the JIT is secondary to this.
2. **Translation block cache** keyed by `(guest PC, relevant CPU flags)`, with
   **block chaining** (patch the exit jump directly to the successor).
3. **Self-modifying code** — page dirty bitmap; a guest write into a page with
   translations invalidates them. x86 makes this mandatory.
4. **Superblocks / traces** — merge across direct branches, keep guest registers
   in host registers across block boundaries within a trace.
5. **Tier 2, feedback-driven** — hot loops get a second compile with better
   allocation and specialization on observed values. Mirrors the tiering already
   proven in `kataan`.
6. **parallel translation** — parallel translated execution with a correct memory model
   (atomics lowered to host atomics, cross-CPU TLB shootdown).

---

## 10. Hardware acceleration

Two distinct meanings, both in scope, tracked separately.

**Virtualization accel** — run guest code natively when guest ISA == host ISA:

- **KVM** (Linux) — reachable with raw `ioctl` syscalls only, so it fits the
  no-foreign-code rule exactly. The primary target: `/dev/kvm`, vCPU fd, the
  `kvm_run` shared page, MMIO/PIO exits routed back into the address-space
  layer, irqfd/ioeventfd, dirty-log-based live snapshot.
- **Hypervisor.framework** (macOS) and **WHPX** (Windows) — both require
  linking a system library, which *breaks the pure-Rust rule*. Ship them as
  explicitly-marked opt-in features and say so in the README rather than
  quietly compromising the charter.
- The accel backend is a `Cpu` implementation like any other, so a config can
  mix an accelerated x86 CPU with an interpreted co-processor in one machine.

**Host GPU acceleration** — scanout upload, scaling and shader filters for
display; later a virtio-gpu/virgl path so guest 3D reaches host 3D. Kept behind
a hard interface boundary; it must never become a build requirement.

---

## 11. Execution targets: native and WebAssembly

rsemu runs natively **and in a browser**. The browser is not a stunt target: it
is the distribution mechanism that needs no install, the demo that makes the
project legible, and — because it removes `mmap`, OS threads, signals and the
monotonic clock all at once — the constraint that keeps the core honest. The
sibling [`fstool`](https://github.com/KarpelesLab/fstool) already ships this way
(a full disk/filesystem toolchain running client-side at
`karpeleslab.github.io/fstool/`), so the pattern is proven in-house.

**Every target below is built in CI from phase 0.** A target that is not built
every commit is a target that does not work; wasm rots faster than anything else.

| Target | Threads | Execution engine | Time source | Storage |
| --- | --- | --- | --- | --- |
| `x86_64` / `aarch64` / `riscv64` Linux, macOS, Windows | `native-std` or `native-raw` | native JIT + KVM/HVF/WHPX | monotonic clock | host files |
| `*-linux-fullrust` (libc-free) | `native-raw` | native JIT, KVM | raw `clock_gettime` | raw syscalls |
| `wasm32-wasip1-threads` | `wasm-atomics` | wasm JIT or IR interpreter | WASI clock | WASI fs |
| `wasm32-unknown-unknown` **+ threads** ⚠️ **nightly** | `wasm-atomics` (Web Workers) | **wasm JIT** or IR interpreter | `performance.now()` import | in-memory / IndexedDB / File System Access |
| `wasm32-unknown-unknown`, no threads | `single` | wasm JIT or IR interpreter | `performance.now()` import | same |
| `wasm32-wasip1` | `single` (threads when the host offers them) | wasm JIT or IR interpreter | WASI `clock_time_get` | WASI preview-1 fs |
| bare metal `no_std` | `single` | IR interpreter | board timer | none |

### 11.1 The one nightly job, and why

**Threaded `wasm32-unknown-unknown` cannot be built on stable**, and this was
verified rather than assumed. Shared linear memory requires `core`/`alloc`/`std`
compiled with `+atomics,+bulk-memory`; the precompiled std that ships with the
target is not, so the link fails:

```
rust-lld: error: --shared-memory is disallowed by std-….rcgu.o
          because it was not compiled with 'atomics' or 'bulk-memory' features.
```

Rebuilding std needs `-Z build-std`, which is nightly-only. So §0's "stable
toolchain" and "threaded browser from phase 0" cannot both hold unqualified, and
the resolution is explicit rather than discovered on day one:

- **`wasm32-wasip1-threads` is the primary threaded wasm target.** It ships a
  precompiled atomics-enabled std, builds on stable, and exercises every line of
  the `wasm-atomics` sync backend. This is what CI gates on.
- **The threaded browser job is pinned to a dated nightly**, uses `-Z
  build-std`, and is the *only* nightly in the project. It is allowed to be the
  one job that can break on a toolchain bump.
- **No shipping artifact is built on nightly.** The non-threaded browser build,
  which is the one that must work everywhere anyway (§11.4), is stable.
- If a future stable ships an atomics-enabled `wasm32-unknown-unknown` std, the
  pin is deleted and nothing else changes.

### 11.2 The browser, with threads

Requires cross-origin isolation (COOP/COEP) for `SharedArrayBuffer`. The JS shim
creates the worker pool and the shared `WebAssembly.Memory` up front and hands
both to rsemu — which is exactly why the `sync` seam exposes a pool rather than
`spawn` (§4.7).

- **Emulation never runs on the main thread.** `Atomics.wait` is forbidden
  there, and a blocked main thread freezes the page. The main thread does
  display and input only; it talks to the emulation worker through lock-free
  ring buffers in shared memory.
- Guest RAM lives in the shared linear memory, so worker threads and generated
  code address it with the same offsets they would natively (§4.7).

### 11.3 The browser, without threads

COOP/COEP is often unavailable (a GitHub Pages default, an embedded iframe, a
corporate proxy), so **this configuration must work, not merely compile**: the
`single` backend, the `best-effort` time base, and execution sliced per
`requestAnimationFrame` so the page stays responsive. Guest CPUs are
round-robined cooperatively, which is the same code path the deterministic test
runner uses — it gets exercised constantly rather than only in demos.

### 11.4 The JIT without `mmap`

wasm has no writable-then-executable memory, so the native code path is simply
unavailable. **The JIT emits WebAssembly instead**: IR → wasm bytecode module →
`WebAssembly.Module` (synchronous instantiation is permitted inside a worker) →
dispatched through a function table. This is a real backend alongside `x86_64`
/`aarch64`/`riscv64` (§9), and it is cheap to build precisely because the IR
already exists — a translation block is a wasm function, guest RAM is the shared
linear memory, and helper calls are imports.

**Block chaining is impossible here.** You cannot patch a jump in an
instantiated wasm module, so every block exit returns through a `call_indirect`
dispatcher — which removes the second-largest win in §9's list. Combined with
the instantiation cost below, the realistic expectation is that the wasm backend
wins only on long-running superblocks, and may not win at all. The plan already
says decide with numbers; this is the specific reason to expect the answer might
be "ship the IR interpreter".

Costs to plan for: per-module instantiation overhead makes tiny blocks a loss,
so the wasm backend only tiers up superblocks; module count is bounded with an
LRU eviction of cold code; and the portable IR interpreter is always the
fallback, so a browser with no `WebAssembly.Module` budget still runs.

### 11.5 Host imports

Follows `purecrypto`'s browser convention — an embedder-supplied import object,
not a bundled JS runtime: `rsemu.now`, `rsemu.random_get`, `rsemu.compile`
(bytes → module handle), `rsemu.log`. Under WASI the same functions bind to
preview-1 imports instead. Nothing else crosses the boundary.

### 11.6 What determinism buys here

Virtual time is computed entirely inside the emulator, so a deterministic run
produces the *same state hash in a browser as on a Linux host*. A user can
record a session in the browser demo, attach the trace to a bug report, and it
replays bit-identically under a native debugger. That is a genuinely unusual
property and it falls straight out of §0 — but only if nothing in `core/` ever
reads the host clock (§15).

### 11.7 Deliverable

A static browser demo page — the `fstool` `web/` + GitHub Pages pattern —
shipping from phase 3: load a ROM, play it, take a save state, all client-side
with nothing uploaded.

---

## 12. Validation

The credibility of the whole project. Each core lands *with* its suite.

| Target | Suite |
| --- | --- |
| 6502 | Tom Harte `SingleStepTests/65x02` (10k vectors/opcode, MIT), `nestest.log` trace diff, blargg `cpu_instrs`/`instr_timing` |
| NES, whole machine | `AccuracyCoin` (MIT, §1) — a *machine* gate, not a CPU one; see the bring-up order below |
| Z80 / SM83 | `zexall`/`zexdoc`, SingleStepTests z80 (MIT), `Gekkio/mooneye-test-suite` acceptance (MIT — **not** `mooneye-gb`, which is the emulator, not the suite), blargg GB suites |
| x86 | `test386.asm`, SingleStepTests 8088/80286/80386, then real-OS boots: FreeDOS → Win 3.11 → Win 95 → Linux → Win XP |
| RISC-V | `riscv-tests`, `riscv-arch-test` via RISCOF, Linux boot on `virt` |
| ARM | SingleStepTests ARM7TDMI, Linux boot on `virt` |
| Framework | Snapshot round-trip identity per device; replay determinism; region-priority/alias unit matrix; DSL parser corpus incl. error-message goldens |
| Threading | Identical state hash under `single` / `native-std` / `wasm-atomics`; safe-point protocol under stress; ranked-lock-order assertions; guest-atomics conformance per frontend (a TSO guest on a weakly-ordered host is the case that finds the bugs) |
| Targets | Every row of §11 built in CI; the browser build runs the machine-level regression suite headlessly under both threaded and non-threaded configurations |
| Cross-cutting | **Differential**: interpreter vs JIT vs accel on randomized instruction streams; **fuzzing** (`fuzz/`) on the DSL parser, disk-image parsers, and every MMIO surface |

**Bring-up order, because AccuracyCoin is last.** It is not a CPU suite: its 67
sections break down roughly as 22 CPU-only, 24 PPU, 10 APU, 7 DMA and 3
controller, and the ones that bite — NMI suppression, sprite-0-hit timing, DMC
DMA bus conflicts, `DMA + $2007`, OAM corruption, open bus — are precisely about
*interaction* between components at exact cycle offsets. None can pass until a
CPU, a PPU, an APU, DMA and a cartridge run together in a realized machine on
correctly-related clock domains.

1. **SingleStepTests 65x02** — pure CPU, no machine. The bring-up gate, and what
   a CPU author iterates against.
2. **`nestest`** — CPU plus a minimal bus, trace-compared. Needs a cartridge, no
   rendering.
3. **AccuracyCoin** — the whole-machine gate, and the real definition of "the NES
   works". Its runner reports per-test results while the machine is still
   incomplete, rather than being all-or-nothing.

Machine-level regression: run a machine deterministically for N virtual seconds
and assert the final state hash plus periodic framebuffer hashes. Cheap, brutal,
catches nearly everything.

---

## 13. Phase plan

Each phase ends in something that **runs and is measured**, and from phase 3
onward in something a person can *use* (§2). No phase is "framework only" —
generic code with no consumer is generic code that is wrong, and a framework
that never becomes an emulator was never validated.

### Phase 0 — Scaffolding
Repo skeleton, `Cargo.toml` feature scaffold, `CLAUDE.md` design rules, CI
(fmt, clippy `-D warnings`, `no_std` build, `--all-features` build, test, and
**a feature-combination sweep** — Rust features are additive-only, so a
`dev-nvme` that silently needs `bus-pci` passes `--all-features` forever and
breaks for the first user who picks a narrow set; CI builds each feature alone
plus a sampled subset),
`LICENSE`, dependency-policy check (`cargo tree` on default features must show
only `rsemu`), and the **full target matrix in CI from the first commit** —
native, `no_std`, `wasm32-unknown-unknown` with and without threads,
`wasm32-wasip1` (§11).
**Gate:** CI green on an empty crate across every target; policy check in place
and enforced. Adding wasm on day one costs an afternoon; adding it at phase 6
costs a refactor of everything.

### Phase 1 — The core kernel
`core/`: value/endianness, address spaces + regions + flat view + dispatch,
RAM/ROM stores, **clock domain forest** (exact integer ratios within an
oscillator tree, bounded fixed-point + residual across trees), scheduler +
event queue, **the
`core::sync` seam with its `single` and `native-std` backends plus the task
pool**, shareable `RamStore`, safe-point protocol, wires, device trait +
lifecycle + composition, props, registry, snapshot reader/writer, reset trees,
error/trace.
**Gate:** a synthetic machine (RAM + a counter device + a stub CPU) built in
Rust runs deterministically for 10¹² ticks; **two domains in one oscillator
tree hold their exact integer ratio over the whole run** (asserted, not
sampled), and cross-tree drift is shown non-accumulating — argued analytically
from the residual accumulator and spot-checked at 10¹² ticks; a tree whose
internal lcm cannot be computed is **refused with an error naming the
domains**;
snapshot → restore → continue produces a bit-identical state hash; the
region-priority/alias/attrs unit matrix is complete and green; the same machine
yields an identical state hash under the `single` and `native-std` sync
backends; `no_std` and both wasm builds pass.

### Phase 2 — The machine description language
Lexer, parser with spans, resolver (params, includes, templates, links),
validator, realizer; JSON projection and round-trip; `rsemu machines` /
`devices` / `describe` / `convert`; error-message golden tests.
**Gate:** the phase-1 synthetic machine is described *entirely* by a `.machine`
file with zero Rust glue; `machines/tests/heterogeneous.machine` (two different
CPU classes, two spaces, one shared RAM region, differing endianness) realizes
and runs; **a fixture that instantiates a `template` four times inside a loop,
from an `include`d file, with `param` overrides** — `template`, `include` and
indexed instantiation are the three hardest features in §5 and the three most
likely to be quietly deferred, and nothing else here touches them; the parser
fuzz target survives **1 CPU-hour from a seeded corpus** with zero crashes and
zero timeouts (unbounded fuzzing is never "clean"; a stated budget is).

### Phase 3 — First real machine: NES
MOS 6502 interpreter (documented + illegal opcodes, cycle-accurate bus timing),
NES PPU/APU/mappers/input, ported from `../gones` onto the generic core (see §1
on the attribution audit that must happen first).

Plus the **minimum host slice**, without which none of this is usable and §2's
"every phase ships something" is false: a framebuffer sink with a native window
and a headless PNG path, keyboard input, audio out, and **the gdbstub**. The
gdbstub is here rather than at the end because it is the highest-leverage tool
for every later phase — building x86 protected mode without it is a
self-inflicted wound — and it costs roughly two weeks.

**Gate:** SingleStepTests 65x02 100 % on documented opcodes, with the analog
unstable ones (`ANE`, `LAX #imm`, `SHA`/`SHX`/`SHY`/`TAS`) ledgered separately
against the suite's chosen constants; `nestest.log` trace-identical; blargg
`cpu_instrs` + `instr_timing` pass; AccuracyCoin passes; three named commercial titles hold 60 emulated fps with 99th-percentile
frame times under 16.6 ms on the reference host, with a headless frame-hash
regression; a human can play one with sound and a controller and attach gdb to
the 6502; the whole machine is one `.machine` file; and it runs **in a browser**
from the demo page (§11.7),
threaded and non-threaded, with the same frame hashes as the native build.
**This is the phase that proves the framework — expect to change core APIs here,
and do it now rather than later.**

### Phase 4 — Genericity proof: Game Boy + Master System
SM83 and Z80 cores, GB PPU/APU, SMS VDP/PSG.
**Gate:** `mooneye-test-suite` acceptance, blargg GB suites, `zexall` clean.
Plus a *falsifiable* genericity test, because "no core API may need to change"
is a claim anyone can satisfy by relabelling a change as a bugfix: **`git diff
--stat src/core/` between the phase-3 and phase-4 tags is under 50 lines, and
every hunk carries a written justification in its commit message.** If it comes
out larger, that is real information about phase 1 and belongs in the record
rather than in an argument.

### Phase 5 — IR, JIT, and the first real OS
IR + verifier + passes, x86-64 backend, **wasm backend** (§11.4), portable
interpreter backend, **background compilation on the task pool**, software TLB,
TB cache + chaining, SMC detection. RISC-V rv64gc frontend + interpreter.
`virt` machine: CLINT, PLIC, 16550 UART, virtio-mmio (blk, net via `pktkit`).
**Gate:** boots an upstream Linux kernel to a shell prompt; `riscv-arch-test`
green; interpreter-vs-JIT differential clean over a randomized corpus;
≥ 100 MIPS single-core on the dev machine; save/restore works *across* an
engine switch.

### Phase 6 — Buses and the PC
> **This phase is split**, because as written it held PCI+PCIe, all of USB, the
> legacy device set, three storage controllers, VGA, disk formats *and* the
> entire x86 frontend from i386 through long mode — gated on four operating
> systems booting. That is most of the project in one box with no intermediate
> progress signal for a year. It runs as **6a** (buses, legacy devices, i386
> real/protected mode → FreeDOS boots), **6b** (long mode, SSE, x87 soft-float,
> AHCI/NVMe, q35 → a modern Linux distro boots), **6c** (Win95, then XP).
>
> **Firmware is a named deliverable here — it was previously missing entirely.**
> FreeDOS, Win95 and XP all need a *legacy* BIOS (XP has no UEFI support), while
> the only permissively-licensed firmware available to us is EDK II / OVMF
> (BSD-2-Clause-Patent), which is UEFI; its legacy CSM path historically used
> SeaBIOS, which is GPL and unreadable to us (§1). So **6a owns a minimal
> in-house legacy BIOS in Rust**: INT 10h/13h/15h/16h, the PCI BIOS interface,
> option-ROM dispatch, and ACPI/SMBIOS table publication. 6b uses EDK II as a
> fetched prebuilt, never vendored (building it needs a C toolchain, which §0
> forbids in-tree). If the in-house BIOS slips, **6c drops out of the gate**
> rather than quietly becoming "ship a GPL blob".

PCI/PCIe, USB (UHCI/EHCI/xHCI + HID/storage/hub), i8259/APIC/IOAPIC/HPET/PIT/RTC,
IDE/AHCI/NVMe, VGA + a modern display device, disk image formats, x86 frontend
(i386 → x86-64, long mode, SSE), `i440fx` and `q35` machines.
**Gate:** FreeDOS, Windows 95, a current Linux distro, and Windows XP all boot
to a desktop from a `.machine` file and a disk image; USB storage and HID work;
`test386.asm` and the x86 SingleStepTests pass.

### Phase 7 — Hardware acceleration
KVM backend (raw ioctls), MMIO/PIO exit routing, irqfd/ioeventfd, dirty logging;
opt-in HVF/WHPX behind clearly-labelled non-pure features.

**SMP accel needs phase 8's foundations, so they move here.** A KVM guest with
more than one vCPU *is* parallel guest execution on host threads: it needs the
safe-point protocol, cross-vCPU TLB shootdown, and a coherent shared-RAM story
on day one. Either those land in phase 7 or phase 7 ships uniprocessor-only —
and uniprocessor KVM makes the "phase-6 machines boot" gate much weaker, since
XP and modern Linux both want SMP. They land here.

Snapshot compatibility across an engine switch also requires an
**engine-independent architectural CPU-state model**: for x86-64 that is the
full MSR set, the XSAVE area, LAPIC/x2APIC state, and the TSC offset. That is
substantial and is a named deliverable of this phase, not a property that
emerges.

**Gate:** the phase-6 machines boot under KVM **with ≥ 2 vCPUs**; snapshots
taken under KVM restore under the JIT and vice versa; an accelerated guest
reaches **≥ 80 % of native** on the same CPU-bound workload, on the reference
host.

### Phase 8 — Performance
Superblocks, cross-block guest-register allocation, tier-2 feedback-driven
recompilation, `aarch64` + `riscv64` backends, **SMP emulation on both native
threads and wasm workers** with a correct memory model, memory-op fusion.
**Gate:** published benchmark suite; **within 2× of QEMU wall-clock** on the
committed workload set, on the reference host (**black-box comparison only** —
running it as a measuring instrument, never reading it, §1). 2× is the number;
if it proves wrong, change it in a commit that says why rather than leaving it
unstated. SMP emulation passes a stress suite (`kvm-unit-tests` atomics/barriers)
plus the Cambridge **litmus tests** for each guest/host memory-model pair, with
no violations, on native threads *and* in a threaded browser build.

### Phase 9 — Frontends, remote, and debugging depth
VNC (then SPICE) server, local windowing backends, audio, gamepad, `noroi`
monitor TUI, gdbstub, record/replay + rewind UI, tracing/profiling output,
C ABI (`ffi`) so rsemu is embeddable the way `purecrypto` and `kataan` are.
**Gate:** a guest debugged end-to-end over gdb; a recorded session replayed
bit-identically on a different host; a rewind demo.

**Continuous tracks** (not phases — they run alongside from their first need):
documentation and per-device docs generated from the registry; the fuzz corpus;
the known-failures ledger; and the machine library under `machines/`.

---

## 14. Reused Karpelès Lab crates

| Crate | Used for | Feature-gated |
| --- | --- | --- |
| [`pktkit`](https://github.com/KarpelesLab/pktkit-rs) | Networking: NIC models are `L2Device`s and `L2Hub` wires them together. **slirp and WireGuard are `L3Device`, not L2** — an in-crate `L2Adapter` (ARP/NDP/DHCP) sits between, so no rsemu code is needed but the config surface is two layers, not one. OpenVPN is server-only; TAP is Linux-only. v0.1.1 with an explicitly unstable API: substantial and useful, **not finished** | yes |
| [`fstool`](https://github.com/KarpelesLab/fstool) | The storage substrate: `BlockDevice`, qcow2, DMG, MBR/GPT (RW; **APM is read-only**), and **read-write** ext2/3/4, FAT, exFAT, NTFS, XFS, HFS+, littlefs. **SquashFS and ISO9660 are `Immutable`** (format-and-flush only), as is a reopened F2FS image. **qcow2 backing files, snapshots and encryption hard-error on open**, and compression is read-only — so the CoW-overlay mechanism §7.1 wants is *fstool work*, not rsemu-on-top work. Also the proof that a KLB crate of this shape ships to the browser | yes |
| [`compcol`](https://github.com/KarpelesLab/compcol) | Snapshot compression (and, under `fstool`, every filesystem codec). Its zstd encoder is self-described as partial and benchmarks at ~0.15× reference speed on incompressible data — which guest RAM largely is — so snapshot compression is opt-in and measured, never assumed | yes |
| [`purecrypto`](https://github.com/KarpelesLab/purecrypto) | TLS for remote display; AES-XTS and PBKDF2/Argon2 as the **primitives** a disk-encryption layer is built from. It does **not** ship LUKS or qcow2 crypto — verified, zero hits — so those are rsemu-side work. On TPM: purecrypto has an external-*signer* seam; the actual TPM 2.0 stack is the separate `purecrypto-tpm` crate | yes |
| [`puremp`](https://github.com/KarpelesLab/puremp) | Exact `Rational` over arbitrary-precision `Int`, for clock arithmetic if `u128` proves insufficient. **Not usable for guest FP**: MPFR-class with caller-chosen precision, no fixed binary32/64 format, no bounded exponent, no IEEE-754 status flags — see §9.1 | yes, and only if needed |
| [`oxideav-png`](https://github.com/OxideAV/oxideav-png) | PNG and APNG encode/decode for framebuffer capture — headless CI screenshots, the frame-hash regression, and docs. With `default-features = false` it drops `oxideav-core` and its only remaining edge is `compcol`, already permitted. Beats hand-rolling a writer: real PNG, and APNG makes recorded sequences free | yes |
| [`noroi`](https://github.com/KarpelesLab/noroi) | A generic curses-style TUI library; the monitor/debugger UI on top is entirely rsemu work. Least mature crate in the set (v0.1.0, Unix TTY only), and **its backend links `libc` directly** — which conflicts with §0's raw-syscall rule and will not link on `*-linux-fullrust`. Optional and non-blocking | yes |
| [`purestd`](https://github.com/KarpelesLab/purestd) / [`fullrust`](https://github.com/KarpelesLab/fullrust) | The raw-syscall **idiom**, and a libc-free build target. It has anonymous `mmap` but **no `mprotect`, no `ioctl`, no `PROT_EXEC`** (verified) — the JIT and KVM syscalls are ours to write. `kataan` is the crate that actually does raw-syscall W^X today | pattern + optional target |
| `../gones` (Go) | Behavioural reference for the 6502/NES port and the clock-divider model | reference only |
| `kataan` (Rust) | Reference for **raw-syscall W^X emission** — real, and the right thing to copy — and for snapshot/mmap design. Its "tiers" are type-specialization, not baseline→optimizing; a baseline tier, OSR and deopt are unstarted there, so it is *not* a precedent for §9's tiering. x86-64 Linux only | reference only |

---

## 15. Design invariants to hold under pressure

Recorded here because each will be tempting to violate around phase 5–6.

1. **No device type appears in a `core::` signature.** If the core needs to know
   about PCI, the abstraction is wrong.
2. **No floats in the time path.** Ever. Intra-tree ratios are exact integer
   arithmetic; the cross-tree timeline is fixed-point with a residual
   accumulator. An `f64` seconds value anywhere near the scheduler is a bug.
   Corollary: **never make a cross-tree conversion where an intra-tree one
   exists.** Going through absolute time to relate the NES CPU and PPU would
   discard the exactness the whole design is built to preserve.
3. **Caches are derived state.** A TLB, a translation block, a flat view, and a
   host pointer must all be reconstructible from architectural state alone, and
   must all be invalidated by the topology generation counter.
4. **Nothing under `core/`, `cpu/`, `dev/`, `machine/` or `ir/` names
   `std::thread`, `std::sync`, or the host clock directly.** The scheduler's
   rate controller and the `accel` mode genuinely need wall time, and both live
   in `core/` — so they take a `HostClock` trait implemented above the `std`
   line and injected at construction. Injected, never called by name: that keeps
   the wasm and `no_std` builds compiling and keeps the clock mockable, which is
   what makes deterministic replay testable in the first place. The `sync` seam and the
   scheduler exist so that the browser build is a recompile rather than a port.
   A single `std::sync::Mutex` in a device model breaks `no_std`, wasm, and the
   `fullrust` target at once.
5. **`MemAttrs::debug` must be honoured by every MMIO device.** A monitor read
   that pops a FIFO is a bug that eats hours.
6. **Every device that has state has a snapshot round-trip test.** No exceptions
   for "simple" devices; simple devices are where the missing field hides.
7. **The interpreter is the oracle.** When the JIT disagrees with the
   interpreter, the JIT is wrong until proven otherwise, and the disagreement
   becomes a regression fixture.
8. **A machine is data.** If emulating a new board requires Rust, ask why the
   DSL could not express it, and fix the DSL.

---

## 16. Known risks

- **Scope.** This is a decade-scale project whose yardstick — measured
  black-box, per §1 — is QEMU. The phase
  gates exist so that value lands early: phase 3 is a shippable NES emulator,
  phase 5 a shippable RISC-V VM, phase 6 a shippable PC emulator.
- **Compile time** at `--all-features` in one crate. Mitigated by the feature
  discipline; escape hatch in §3.
- **The purity rule vs. the host.** GPU, HVF, and WHPX cannot be reached without
  foreign code. The answer is explicit, labelled opt-in features — never a
  silent compromise.
- **Determinism vs. SMP emulation.** Parallel guest execution is fundamentally at
  odds with bit-reproducibility. Resolution: they are different modes; the
  regression suite only ever runs deterministic mode.
- **Cross-origin isolation.** The threaded browser build needs COOP/COEP, which
  is not always obtainable. Mitigated by making the non-threaded configuration a
  supported, CI-tested target rather than a fallback nobody runs — but it is
  slower, and that gap should be measured and published, not hidden.
- **wasm JIT economics.** Per-module instantiation cost means the wasm backend
  only pays off on superblocks; if measurement says otherwise, the honest
  outcome is that the browser ships the IR interpreter and the wasm backend is
  cut. Decide with numbers at phase 5, not with hope at phase 0.
- **Guest memory models.** A TSO guest on a weakly-ordered host is where
  parallel emulation goes wrong, and the failures are load-dependent and
  host-specific. This is why the barrier responsibility is pinned to the
  frontend lifter with its own suite (§12) rather than left implicit.
- **x86 is a tar pit.** Segmentation, SMC, and the paging corner cases have
  consumed larger teams. Phase 6 is the long one; treat its estimate with
  suspicion.
