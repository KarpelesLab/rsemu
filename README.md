# rsemu

**A multiplatform emulator in pure Rust, built from the bottom up.**

rsemu is an emulator — the thing you point at a ROM or a disk image and run.
It is *built* on a generic framework, and that framework comes first: address
spaces, clock domains, wires, devices, buses and a translation IR, then CPU
cores, PCI, USB, storage and NICs on top of them, then machines described by a
config file rather than compiled in.

If you want a NES, you write a `.machine` file. If you want four heterogeneous
CPUs sharing one RAM region across three bus fabrics, you write a stranger
`.machine` file. Nothing in rsemu needs patching to allow either.

Starting low costs time before the first ROM boots. It buys what every emulator
that started at the top eventually wishes it had: one memory model, one clock,
one snapshot format, one debugger, shared by every machine ever added.

## Design principles

- **Pure Rust, no foreign code.** No C, no FFI, no vendored assembly, no build
  scripts. The default build has an empty `cargo tree`; dependencies are
  first-party Karpelès Lab crates only, and every one is feature-gated.
- **`unsafe` is quarantined.** `unsafe_code = "deny"` crate-wide. Only the RAM
  host-pointer fast path, the JIT code buffer, the raw-syscall accel backends,
  and the C ABI opt back in — each scoped, each with a safety comment.
- **Determinism is a mode, not an accident.** Deterministic runs are
  bit-reproducible across hosts and across execution engines, which is what
  makes save states, record/replay, rewind, and the regression suite possible.
- **Two time bases.** `exact` represents every clock ratio with zero error
  (rational integer arithmetic) for cycle-accurate machines; `best-effort`
  uses fixed-point with a residual accumulator — cheaper, tolerant of runtime
  clock changes, bounded non-accumulating drift. Both are integer-only and
  both are deterministic; picking one is a declaration, never a silent
  fallback.
- **Accuracy is measured.** Every CPU core ships with a published conformance
  suite and a known-failures ledger that only ever shrinks.
- **Generic first.** A device that needs a new mechanism gets it added to the
  core generically. No device type ever appears in a `core::` signature.
- **`no_std` + `alloc` core.** Host I/O, JIT, acceleration and frontends live
  above the `std` line.
- **Multithreaded by design.** Guest CPUs, background JIT compilation, and
  device I/O can all run in parallel — with the same state hash whether the
  machine runs on one thread or many. All of it goes through one portability
  seam, so a device is written once.
- **Runs in the browser.** `wasm32-unknown-unknown` with *and* without threads
  is a CI target from the first commit: Web Workers over shared memory, a JIT
  that emits WebAssembly instead of native code, and no `mmap`, signals, or
  host clock anywhere in the core.
- **One crate, one feature per component.** A NES build links a 6502 and
  nothing else.

## Status

Planning. Nothing is implemented yet. The plan is ordered so that value lands
early: phase 3 is a usable NES emulator, phase 5 a RISC-V machine that boots
Linux, phase 6 a PC that boots DOS through Windows XP.

**Read [`ROADMAP.md`](ROADMAP.md)** — it contains the architecture (memory,
time, devices, state, IR), the machine description language, the phase plan
with acceptance gates, and the design invariants.

## Built on

[`pktkit`](https://github.com/KarpelesLab/pktkit-rs) (all networking),
[`compcol`](https://github.com/KarpelesLab/compcol) (image + snapshot
compression), [`purecrypto`](https://github.com/KarpelesLab/purecrypto)
(disk/snapshot encryption, emulated crypto devices),
[`fstool`](https://github.com/KarpelesLab/fstool) (block devices, qcow2,
partition tables, and a dozen read-write filesystems),
[`noroi`](https://github.com/KarpelesLab/noroi) (monitor TUI).

## License

MIT — see [LICENSE](LICENSE).
