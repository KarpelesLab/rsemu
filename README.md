# rsemu

[![CI](https://github.com/KarpelesLab/rsemu/actions/workflows/ci.yml/badge.svg)](https://github.com/KarpelesLab/rsemu/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/rsemu.svg)](https://crates.io/crates/rsemu)
[![docs.rs](https://img.shields.io/docsrs/rsemu)](https://docs.rs/rsemu)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

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
- **Time follows the crystals.** A machine is a *forest* of clock domains, one
  tree per oscillator. Within a tree, ratios are exact integers — the NES PPU
  advances exactly 3 dots per CPU cycle, forever, because both descend from one
  crystal, and games depend on that absolutely. Across independent oscillators
  the relationship is bounded rather than exact, because on real hardware it is
  genuinely loose: separate crystals drift, and no correct software can depend
  on their phase. Both paths are integer-only and deterministic.
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

Scaffolding. Nothing is emulated yet — what exists is the crate skeleton, the
error and value types, and a CI matrix that builds and tests every target from
the first commit. The plan is ordered so that value lands early: the NES
milestone is a usable emulator, then a RISC-V machine that boots Linux, then a
PC that boots DOS through Windows XP.

## Build

```sh
cargo build              # library + the rsemu binary
cargo test --all-features
cargo run -- --version

cargo build --no-default-features   # no_std core, as CI checks it
```

WebAssembly — no `wasm-bindgen`; the module is instantiated directly and
strings cross as a pointer/length pair read from exported memory:

```sh
cargo rustc --crate-type cdylib --target wasm32-unknown-unknown \
    --no-default-features --features wasm --release
cp target/wasm32-unknown-unknown/release/rsemu.wasm web/
python3 -m http.server -d web 8080     # then open http://localhost:8080/
```

See [`web/README.md`](web/README.md). MSRV is 1.88, pinned by a CI job so it
stays a checked claim.

**Read [`ROADMAP.md`](ROADMAP.md)** — it contains the architecture (memory,
time, devices, state, IR), the machine description language, the phase plan
with acceptance gates, and the design invariants.

## Built on

[`pktkit`](https://github.com/KarpelesLab/pktkit-rs) (all networking),
[`compcol`](https://github.com/KarpelesLab/compcol) (image + snapshot
compression), [`purecrypto`](https://github.com/KarpelesLab/purecrypto)
(disk/snapshot encryption, emulated crypto devices),
[`fstool`](https://github.com/KarpelesLab/fstool) (block devices, qcow2,
partition tables, and read-write ext/FAT/exFAT/NTFS/XFS/HFS+),
[`noroi`](https://github.com/KarpelesLab/noroi) (monitor TUI).

## License and provenance

MIT — see [LICENSE](LICENSE).

rsemu is written **clean-room from hardware documentation**. MIT cannot absorb
GPL'd code, so copyleft sources are off limits to contributors — **the QEMU
source tree above all**, along with Bochs, DOSBox, MAME, VICE, Dolphin, PCSX2
and every other GPL/LGPL emulator. We work from datasheets, ISA manuals, the
NESdev wiki, Pan Docs and real hardware; permissively licensed code is welcome
with its attribution intact. Benchmarking against a GPL emulator is fine —
that is black-box use, not derivation.

[`docs/`](docs/) is the curated register of primary sources — ISA manuals,
platform specs, PCI/USB/virtio, OSDev resources and conformance suites — each
annotated with what it authoritatively answers and whether it is safe to quote.

See [CONTRIBUTING.md](CONTRIBUTING.md) before your first patch, and
[`ROADMAP.md` §1](ROADMAP.md) for the full policy.
