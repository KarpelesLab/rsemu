# rsemu Roadmap — a generic, pure-Rust emulator framework

`rsemu` is a **machine emulator framework**, not an emulator. The deliverable is
a set of generic mechanisms — address spaces, clock domains, wires, devices,
buses, a translation IR — plus concrete components built on those mechanisms
(CPU cores, PCI, USB, storage, NICs), plus a **machine description language**
that wires arbitrary topologies together at runtime. Whether the described
machine is a NES, a q35 PC, or four heterogeneous CPUs sharing one RAM region
through three different bus fabrics is the config file's problem, not the
framework's.

This roadmap defines the architecture, the phase order, and the acceptance gate
for each phase. It is written to be executed top-to-bottom; every phase ends in
something that runs and is measured.

> **Status (2026-08-30):** nothing implemented. This document is the plan.

---

## 0. Non-negotiables

These are decided. Do not relitigate them mid-implementation.

- **Pure Rust, no foreign code.** No C, no `bindgen`, no vendored assembly, no
  build scripts that invoke a compiler. The dependency budget is *first-party
  Karpelès Lab crates only* (§14) — and even those stay feature-gated so the
  core builds with an empty `cargo tree`.
- **`unsafe` is quarantined.** Crate-wide `unsafe_code = "deny"` (not `forbid`).
  Exactly four subsystems may opt back in with a scoped
  `#[allow(unsafe_code)]` + a `// SAFETY:` comment: the RAM host-pointer fast
  path, the JIT code buffer (W^X `mmap`/`mprotect`), the raw-syscall accel
  backends (KVM ioctls), and the C ABI module. Everything else is safe Rust.
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
  clock, devices, IR, interpreters) never touches `std`. Host I/O, threads,
  JIT, accel and frontends live above the `std` line. CI builds both.
- **MIT licensed**, `edition = "2024"`, stable toolchain, `rustfmt` + `clippy`
  clean under `-D warnings`.

---

## 1. Crate shape

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
    clock.rs          #   clock domain tree, exact rational virtual time
    sched.rs          #   event queue, execution budgets, quantum, threading modes
    wire.rs           #   IRQ / GPIO lines, splitters, combiners
    device.rs         #   Device trait, lifecycle, composition
    props.rs          #   dynamic property values + typed extraction
    registry.rs       #   by-name construction (the config's entry point)
    state.rs          #   versioned snapshot reader/writer
    reset.rs bus.rs   #   reset trees, generic Bus trait
    error.rs trace.rs #   diagnostics, structured tracing
  machine/            # the description language: lexer, parser, resolver, realizer
  ir/                 # translation IR: ops, builder, passes, verifier
  jit/                # backends: x86_64, aarch64, riscv64, portable interpreter
  accel/              # kvm, (hvf), (whpx) — execution engines that aren't ours
  cpu/                # one module + one feature per core: mos6502, z80, sm83, …
  dev/                # one module + one feature per device: pci/, usb/, blk/, net/, …
  boards/             # one module + one feature per built-in machine
  host/               # std-only: display, audio, input, gdbstub, VNC, CLI
machines/             # shipped .machine description files (data, not code)
```

**Why one crate.** Cross-component invariants (snapshot versioning, the
determinism contract, the IR) change together; splitting them across crates
means a version-skew matrix nobody will maintain. **Escape hatch:** if
full-feature compile time exceeds ~90 s, split *only* `jit/` and `host/` into
sibling crates — those have the fewest inbound edges. Do not split `cpu/` or
`dev/`; they are the whole point of the feature system.

---

## 2. The generic core

This is the part that must be right. Everything else is replaceable.

### 2.1 Address spaces and memory

The single most important abstraction. Modelled as a **region tree flattened
into a dispatch table**, which is the design QEMU converged on after a decade of
alternatives; it is worth adopting deliberately rather than rediscovering.

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
| **Access-width constraints** | A 32-bit-only register must reject a byte write, not silently accept it |
| **Per-region endianness** | Big-endian device on a little-endian bus is normal, not exotic |
| **Fallible access** | Unmapped read returns a bus fault the CPU can turn into an exception, not `0xFF` guesswork. Per-space "unassigned" policy: fault / read-as-ones / read-as-zeros / log |
| **Topology generation counter** | Every cache (TLBs, JIT translation blocks, direct pointers) invalidates on remap |
| **Dirty-page tracking** | Framebuffer refresh, self-modifying-code detection, live snapshot |

**Dispatch.** On topology change, the container tree is flattened into a sorted,
non-overlapping `FlatView`. Lookup is two-level: a page-granular dispatch table
(dense `Vec` for the low 4 GiB, radix trie above) yields either a **host pointer
+ length** (the RAM fast path — no virtual call, no bounds walk) or a `FlatView`
index (the slow path). Guest-virtual→host translation for CPUs with an MMU adds
a per-CPU **softmmu TLB** in front of this (§9).

*Note on gones.* The `memory.Bus` in `../gones` OR-combines the results of every
handler mapped at an address and logs a "bus conflict". That is the correct
model for an open-bus system like the NES and the wrong default for PCI. In
rsemu this becomes a per-container `CombinePolicy { Priority, WiredOr, WiredAnd,
Conflict }` — the NES keeps its behaviour, everyone else gets deterministic
priority.

### 2.2 Time, clocks, and scheduling

Generalizes the `../gones` master-clock-plus-dividers model, which is the right
shape, to a tree with exact arithmetic.

- **Clock domain tree.** `ClockDomain { parent, mul: u64, div: u64 }`, root = a
  named frequency. Domains can be reparented and gated at runtime (a PLL, a
  power-managed peripheral, a CPU that halts).
- **Exact virtual time.** The global timeline unit is `1/L` seconds where
  `L = lcm(all root frequencies)`, computed at realize time and held in `u128`.
  For real machines (21477272, 1789773, 44100, 48000, …) `L` fits comfortably;
  if it overflows, fall back to femtoseconds and *log the bounded drift* rather
  than silently accumulating it. Every domain converts to and from this unit by
  exact integer multiply/divide — **no floats anywhere in the time path.**
- **Event queue.** A hierarchical timing wheel for the dense near term plus a
  binary heap for far-future events. Events carry a monotonically increasing
  sequence number so ties break deterministically.
- **Execution budgets.** A CPU is never "stepped one instruction" by the
  scheduler; it is handed a budget ("run until virtual time T or 10 000 ticks,
  whichever first") and reports back how much it consumed. This is what makes
  JIT block execution and cycle accounting coexist.
- **Threading modes**, selectable per machine:
  - `deterministic` — one host thread, round-robin over CPUs with a fixed
    quantum. Required for record/replay and the regression suite.
  - `parallel` — thread per CPU with a rendezvous barrier per quantum. Fast,
    non-deterministic, the default for interactive use.
  - `accel` — CPUs run in hardware (§10); virtual time is driven by the host
    clock and the scheduler becomes a deadline service.
- **Rate control.** `realtime` (throttle to wall clock, with catch-up limits and
  frame pacing), `unbounded` (as fast as possible), `fixed-ratio` (2× slow for
  debugging).

### 2.3 Wires: interrupts and GPIO

```rust
pub trait WireSink: Send + Sync { fn set_level(&self, line: u32, level: Level); }
pub struct Wire { /* level state + fan-out */ }
```

Level and edge semantics both, with the *edge detector as a device* rather than
a flag, so it snapshots correctly. Ships with the standard combinators as
ordinary devices: `wire.split`, `wire.or`, `wire.and`, `wire.not`,
`wire.level-to-edge`. Interrupt controllers (i8259, APIC, GIC, PLIC, NES NMI
line) are then just devices with wire sinks and sources — the core knows nothing
about "interrupts".

### 2.4 Devices, properties, registry

```rust
pub trait Device: Send + Sync {
    fn class(&self) -> &'static DeviceClass;
    fn realize(&self, ctx: &mut RealizeCtx) -> Result<()>;   // wire up, map regions
    fn reset(&self, kind: ResetKind);                        // Cold | Warm | Bus
    fn save(&self, w: &mut StateWriter) -> Result<()>;
    fn load(&self, r: &mut StateReader) -> Result<()>;
}
```

- **Two-phase construction.** `new(props)` validates properties and allocates;
  `realize(ctx)` performs every outward action (mapping regions, connecting
  wires, attaching to buses). Nothing observable happens before realize, so the
  config resolver can build the whole graph and fail cleanly.
- **Composition.** Devices own child devices. A `pc.q35` device instantiates its
  own chipset children; the config only names the top level unless it wants to
  reach in.
- **Property system** (`core::props`): a small dynamic `Value` — int, uint,
  bool, string, size (`512M`), address, duration, list, map, and **link**
  (a reference to another object) — with typed extraction and precise error
  messages. Deliberately not `serde`: the dependency policy forbids it, the
  value set is small, and the error messages matter more than the generality.
- **Registry** (`core::registry`): by-name construction, `compcol::factory`
  style. `registry::create("pci.nvme", props)`. Registration is explicit per
  feature (`#[cfg(feature = "dev-nvme")] reg.add(NVME_CLASS);`) — no
  link-time-magic crate. The registry is also the introspection surface:
  `rsemu list-devices` / `rsemu describe pci.nvme` prints classes, properties,
  defaults and bus requirements, and the doc generator reads the same data.

### 2.5 State: snapshots, replay, rewind

Built in phase 1, not bolted on later.

- **Format.** Chunked and versioned: a machine header (config hash, feature
  set, guest arch list), then one chunk per device instance keyed by
  `(instance path, class name, class version)`. Loading a snapshot into a
  differently-shaped machine fails with a diff, not a crash.
- **Content.** Devices serialize *architectural* state only. Derived caches
  (TLBs, translation blocks, flattened views, host pointers) are rebuilt on
  load. A device whose `save`/`load` round-trip does not reproduce an identical
  state hash fails its own unit test.
- **Layering.** Compression via `compcol` (zstd), integrity via `purecrypto`
  (BLAKE3), optional encryption via `purecrypto` — all feature-gated; the raw
  format works with zero dependencies.
- **Record/replay.** In deterministic mode, log every non-deterministic input
  (host clock reads, RNG draws, network/serial/input events) against a virtual
  timestamp. Replay reinjects them. This yields, for free: reproducible bug
  reports, CI regression fixtures, and **rewind** (periodic snapshot + replay
  forward to an earlier point).

### 2.6 Execution engines

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

---

## 3. The machine description language

The framework's user interface. It must express arbitrary graphs — including
heterogeneous CPUs sharing memory, multiple disjoint address spaces, and
recursive bus fabrics — and it must produce good errors, because most people
will meet rsemu through a syntax error.

**Format:** a purpose-built declarative DSL (`.machine`), hand-parsed with span
tracking, plus a **lossless JSON projection** for tooling. One AST, two
syntaxes: `rsemu machine convert` round-trips either direction. JSON alone is
rejected — it cannot carry comments, and comments in a machine file are how the
next person learns why a mirror exists.

```
machine "nes" {
  param  region = "ntsc"
  clock  master = 21477272 Hz            # NTSC colorburst × 6

  space cpubus  { width = 16, unassigned = open-bus }
  space ppubus  { width = 14, unassigned = open-bus }

  object ram "wram" { size = 2K }

  object cpu "mos6502" {
    clock  = master / 12
    space  = cpubus
    engine = "interp"
  }
  object ppu "nes.ppu" { clock = master / 4, space = ppubus }
  object apu "nes.apu" { clock = master / 12 }

  map cpubus 0x0000 size 0x2000 = mirror(wram)      # 2K mirrored 4×
  map cpubus 0x2000 size 0x2000 = mirror(ppu.regs)
  map cpubus 0x4000 size 0x0020 = apu.regs

  wire ppu.nmi   -> cpu.nmi
  wire apu.irq   -> cpu.irq
  wire cart.irq  -> cpu.irq                          # wired-OR, declared once
}
```

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

## 4. CPU cores

Each core is a feature. The order below is chosen so that each one proves a
*new mechanism* in the framework rather than adding another opcode table.

| Core | Proves | Phase |
| --- | --- | --- |
| **MOS 6502** (+ illegal opcodes, 2A03 variant) | Cycle-accurate interpretation, bus timing, the whole core is exercised end to end | 3 |
| **SM83** (Game Boy) and **Z80** | That the framework is not 6502-shaped; different interrupt model, I/O space | 4 |
| **RISC-V rv64gc** (+ rv32) | MMU/softmmu, privilege levels, atomics, FPU, and the IR/JIT path. Smallest ISA that boots real Linux | 5 |
| **x86**: i386 → x86-64 (real/protected/long mode, SSE) | The hard one: segmentation, variable-length decode, self-modifying code, paging quirks | 6 |
| **ARM**: ARMv7-A, ARMv8-A AArch64 | Second major JIT frontend; validates IR generality | 6–8 |
| Later: 68000, MIPS, PowerPC, SuperH, 8080, 65816, V850 | Breadth; each is a weekend once the IR is stable | post-8 |

Every core provides both an **interpreter** and (from phase 5 onward) an **IR
frontend**, and the two are differentially tested against each other forever.

---

## 5. Buses and devices

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

### 5.1 Storage

Disk image layer: `raw`, `qcow2` (compression via `compcol`, encryption via
`purecrypto`), `vmdk`, `vhdx`, `vdi`, plus a copy-on-write overlay, snapshots,
discard/TRIM, and a write-back cache with a flush contract that survives
snapshotting. Read-only ISO/UDF via the existing `iso9660` work.

### 5.2 Networking

**Solved by `pktkit`.** Every emulated NIC (`e1000`, `rtl8139`, `virtio-net`,
`ne2000`, …) is a `pktkit::L2Device`; the config then attaches it to a
`pktkit::L2Hub`, a `slirp` NAT stack, a TUN/TAP device, or a WireGuard/OpenVPN
tunnel with no rsemu-side code. This is a large, already-finished chunk of the
project that most emulators have to write themselves.

---

## 6. Host-facing layer (`host/`, std only)

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

## 7. The translation IR and JIT

The performance story. Design it once, correctly; every guest and every host
pays for mistakes here.

**IR shape** — TCG-like, deliberately: ~60 architecture-neutral ops over typed
temporaries (`i32 i64 i128 f32 f64 v128`), SSA within a translation block,
helper calls for anything messy (rare instructions, MMIO, exceptions).

- Data: `mov ext trunc bswap deposit extract`
- Arith: `add sub mul div rem neg`, `add2 sub2 mulu2 muls2` (carry chains)
- Logic/shift: `and or xor not andc orc eqv nand nor shl shr sar rotl rotr`
- Bit: `clz ctz popcount`
- Compare/branch: `setcond movcond brcond`
- Memory: `ld st` carrying a `MemOp { size, sign, endianness, alignment, index }`
- Atomics: `cmpxchg fetch_{add,and,or,xor} xchg`, plus fence
- Control: `goto_tb exit_tb lookup_and_goto call_helper`
- Vector ops added with the ARM/x86 SIMD work, not before

**Pipeline.** Guest ISA → frontend lifter → IR → passes (constant folding, copy
propagation, dead-code elimination, liveness, memory-op fusion) → register
allocation (linear scan) → host backend.

**Backends.** `x86_64` first (the dev machine), then `aarch64` and `riscv64`,
plus a **portable IR interpreter backend** so an unsupported host degrades in
speed rather than failing to run. Code buffers are W^X: `mmap` RW → emit →
`mprotect` RX, via raw syscalls, no libc — the `purestd`/`kataan::jit` pattern.

**The mechanisms that actually produce speed** (all in phase 5–8):

1. **Softmmu TLB** — per-CPU, direct-mapped (4096 entries), split by access
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
6. **MTTCG** — parallel translated execution with a correct memory model
   (atomics lowered to host atomics, cross-CPU TLB shootdown).

---

## 8. Hardware acceleration

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

## 9. Validation

The credibility of the whole project. Each core lands *with* its suite.

| Target | Suite |
| --- | --- |
| 6502 | Tom Harte `SingleStepTests/65x02` (10k vectors/opcode), `nestest.log` trace diff, blargg `cpu_instrs`/`instr_timing`, `AccuracyCoin` (already vendored in `../gones`) |
| Z80 / SM83 | `zexall`/`zexdoc`, SingleStepTests z80, mooneye-gb acceptance, blargg GB suites |
| x86 | `test386.asm`, SingleStepTests 8088/80286/80386, then real-OS boots: FreeDOS → Win 3.11 → Win 95 → Linux → Win XP |
| RISC-V | `riscv-tests`, `riscv-arch-test` via RISCOF, Linux boot on `virt` |
| ARM | SingleStepTests ARM7TDMI, Linux boot on `virt` |
| Framework | Snapshot round-trip identity per device; replay determinism; region-priority/alias unit matrix; DSL parser corpus incl. error-message goldens |
| Cross-cutting | **Differential**: interpreter vs JIT vs accel on randomized instruction streams; **fuzzing** (`fuzz/`) on the DSL parser, disk-image parsers, and every MMIO surface |

Machine-level regression: run a machine deterministically for N virtual seconds
and assert the final state hash plus periodic framebuffer hashes. Cheap, brutal,
catches nearly everything.

---

## 10. Phase plan

Each phase ends in something that **runs and is measured**. No phase is
"framework only" — generic code with no consumer is generic code that is wrong.

### Phase 0 — Scaffolding
Repo skeleton, `Cargo.toml` feature scaffold, `CLAUDE.md` design rules, CI
(fmt, clippy `-D warnings`, `no_std` build, `--all-features` build, test),
`LICENSE`, dependency-policy check (`cargo tree` on default features must show
only `rsemu`).
**Gate:** CI green on an empty crate; policy check in place and enforced.

### Phase 1 — The core kernel
`core/`: value/endianness, address spaces + regions + flat view + dispatch,
RAM/ROM stores, clock tree with exact rational time, scheduler + event queue,
wires, device trait + lifecycle + composition, props, registry, snapshot
reader/writer, reset trees, error/trace.
**Gate:** a synthetic machine (RAM + a counter device + a stub CPU) built in
Rust runs deterministically for 10⁹ ticks; snapshot → restore → continue
produces a bit-identical state hash; the region-priority/alias/attrs unit matrix
is complete and green; `no_std` build passes.

### Phase 2 — The machine description language
Lexer, parser with spans, resolver (params, includes, templates, links),
validator, realizer; JSON projection and round-trip; `rsemu list-devices` /
`describe` / `convert`; error-message golden tests.
**Gate:** the phase-1 synthetic machine is described *entirely* by a `.machine`
file with zero Rust glue; `machines/tests/heterogeneous.machine` (two different
CPU classes, two spaces, one shared RAM region, differing endianness) realizes
and runs; the parser fuzz target runs clean.

### Phase 3 — First real machine: NES
MOS 6502 interpreter (documented + illegal opcodes, cycle-accurate bus timing),
NES PPU/APU/mappers/input, ported from `../gones` onto the generic core.
**Gate:** SingleStepTests 65x02 100 %; `nestest.log` trace-identical; blargg
`cpu_instrs` + `instr_timing` pass; AccuracyCoin passes; a real game runs at
60 fps with a headless frame-hash regression; the whole machine is one
`.machine` file. **This is the phase that proves the framework — expect to
change core APIs here, and do it now rather than later.**

### Phase 4 — Genericity proof: Game Boy + Master System
SM83 and Z80 cores, GB PPU/APU, SMS VDP/PSG.
**Gate:** mooneye-gb acceptance suite, blargg GB suites, `zexall` clean. **No
core API may need to change to accommodate these** — if one does, it was a
phase-1 design bug and the fix belongs in the core, not the board.

### Phase 5 — IR, JIT, and the first real OS
IR + verifier + passes, x86-64 backend, portable interpreter backend, softmmu
TLB, TB cache + chaining, SMC detection. RISC-V rv64gc frontend + interpreter.
`virt` machine: CLINT, PLIC, 16550 UART, virtio-mmio (blk, net via `pktkit`).
**Gate:** boots an upstream Linux kernel to a shell prompt; `riscv-arch-test`
green; interpreter-vs-JIT differential clean over a randomized corpus;
≥ 100 MIPS single-core on the dev machine; save/restore works *across* an
engine switch.

### Phase 6 — Buses and the PC
PCI/PCIe, USB (UHCI/EHCI/xHCI + HID/storage/hub), i8259/APIC/IOAPIC/HPET/PIT/RTC,
IDE/AHCI/NVMe, VGA + a modern display device, disk image formats, x86 frontend
(i386 → x86-64, long mode, SSE), `i440fx` and `q35` machines.
**Gate:** FreeDOS, Windows 95, a current Linux distro, and Windows XP all boot
to a desktop from a `.machine` file and a disk image; USB storage and HID work;
`test386.asm` and the x86 SingleStepTests pass.

### Phase 7 — Hardware acceleration
KVM backend (raw ioctls), MMIO/PIO exit routing, irqfd/ioeventfd, dirty logging;
opt-in HVF/WHPX behind clearly-labelled non-pure features.
**Gate:** the phase-6 machines boot under KVM; snapshots taken under KVM restore
under the JIT and vice versa; near-native CPU benchmark on an accelerated guest.

### Phase 8 — Performance
Superblocks, cross-block guest-register allocation, tier-2 feedback-driven
recompilation, `aarch64` + `riscv64` backends, MTTCG with a correct memory
model, memory-op fusion.
**Gate:** published benchmark suite; within a stated factor of QEMU on the same
workloads; MTTCG passes a stress suite (`kvm-unit-tests` atomics/barriers) with
no memory-model violations.

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

## 11. Reused Karpelès Lab crates

| Crate | Used for | Feature-gated |
| --- | --- | --- |
| [`pktkit`](https://github.com/KarpelesLab/pktkit-rs) | Everything networking: NIC models are `L2Device`s; hubs, NAT/slirp, TUN/TAP, WireGuard, OpenVPN come free | yes |
| [`compcol`](https://github.com/KarpelesLab/compcol) | qcow2/vmdk compression, snapshot compression, compressed ROM/disk images | yes |
| [`purecrypto`](https://github.com/KarpelesLab/purecrypto) | Disk/snapshot encryption (LUKS, qcow2 crypto), emulated TPM/crypto devices, TLS for remote display | yes |
| [`puremp`](https://github.com/KarpelesLab/puremp) | Exact rational clock arithmetic *if* `u128` proves insufficient; guest FPU corner cases | yes, and only if needed |
| [`noroi`](https://github.com/KarpelesLab/noroi) | Monitor/debugger TUI | yes |
| [`purestd`](https://github.com/KarpelesLab/purestd) / [`fullrust`](https://github.com/KarpelesLab/fullrust) | The raw-syscall pattern for JIT `mmap`/`mprotect` and KVM ioctls; a libc-free build target | pattern + optional target |
| `../gones` (Go) | Behavioural reference for the 6502/NES port and the clock-divider model | reference only |
| `kataan` (Rust) | Reference for JIT tiering, W^X emission, and snapshot/mmap design | reference only |

---

## 12. Design invariants to hold under pressure

Recorded here because each will be tempting to violate around phase 5–6.

1. **No device type appears in a `core::` signature.** If the core needs to know
   about PCI, the abstraction is wrong.
2. **No floats in the time path.** Ever. Rational integer arithmetic only.
3. **Caches are derived state.** A TLB, a translation block, a flat view, and a
   host pointer must all be reconstructible from architectural state alone, and
   must all be invalidated by the topology generation counter.
4. **`MemAttrs::debug` must be honoured by every MMIO device.** A monitor read
   that pops a FIFO is a bug that eats hours.
5. **Every device that has state has a snapshot round-trip test.** No exceptions
   for "simple" devices; simple devices are where the missing field hides.
6. **The interpreter is the oracle.** When the JIT disagrees with the
   interpreter, the JIT is wrong until proven otherwise, and the disagreement
   becomes a regression fixture.
7. **A machine is data.** If emulating a new board requires Rust, ask why the
   DSL could not express it, and fix the DSL.

---

## 13. Known risks

- **Scope.** This is a decade-scale project measured against QEMU. The phase
  gates exist so that value lands early: phase 3 is a shippable NES emulator,
  phase 5 a shippable RISC-V VM, phase 6 a shippable PC emulator.
- **Compile time** at `--all-features` in one crate. Mitigated by the feature
  discipline; escape hatch in §1.
- **The purity rule vs. the host.** GPU, HVF, and WHPX cannot be reached without
  foreign code. The answer is explicit, labelled opt-in features — never a
  silent compromise.
- **Determinism vs. MTTCG.** Parallel translated execution is fundamentally at
  odds with bit-reproducibility. Resolution: they are different modes; the
  regression suite only ever runs deterministic mode.
- **x86 is a tar pit.** Segmentation, SMC, and the paging corner cases have
  consumed larger teams. Phase 6 is the long one; treat its estimate with
  suspicion.
