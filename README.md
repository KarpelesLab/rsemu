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

Early, but it runs things.

**Eight CPU cores.** Where a public corpus exists the number is measured, not
claimed; where one does not, the row says what stands in for it rather than
quietly leaving the impression of a number.

| Core | Suite | Result |
| --- | --- | --- |
| MOS 6502 / RP2A03 | SingleStepTests 65x02 | **2,560,000 / 2,560,000** incl. bus traces |
| WDC 65C02S | SingleStepTests 65x02 | **2,530,025 / 2,540,000** |
| Zilog Z80 | SingleStepTests z80, zexall | **1,604,000 / 1,604,000**, 67/67 |
| RISC-V RV64GC | riscv-tests | **409 / 409** |
| RISC-V RV64GC | riscv-arch-test 3.9.1, signatures diffed against the Sail model | **181 / 181** — I, M, A, F, D, C, Zifencei, privilege. Which extension suites are *not* run, and why, is listed in [`docs/testing/README.md`](docs/testing/README.md) |
| Intel 8086/8088 | SingleStepTests 8088 | **2,974,160 / 3,007,000** |
| Motorola 68000 | SingleStepTests 680x0 | runner in-tree; fetch the corpus to reproduce |
| Sharp SM83 (Game Boy) | blargg, mooneye | `cpu_instrs` 11/11; mooneye acceptance 22/66 |
| ARMv5TE | ARM7TDMI corpus (v4T subset) | no public v5 corpus exists — see §12 |
| ARMv7E-M | differential vs. our own ARMv5TE | 83,597 encodings identical, 13,683 divergences *asserted* |

Every corpus is fetched by `scripts/fetch-testdata.sh`, never vendored, and
gated behind an environment variable — a licensing rule as much as a size one.

**Six machines you can run.** `nes-ntsc` and `nes-pal` boot a cartridge, raise
NMI and render — AccuracyCoin draws its menu. `gameboy` runs blargg's suite.
`beneater-6502` and `apple1` are interactive over your terminal:

```console
$ cargo run --features machine-apple1 -- run apple1
RSMON
>FF00
FF00: D8 A2 FF 9A A9 7F 8D 12
```

`riscv-virt` is the one that boots real system software. OpenSBI 1.6 runs
completely on a device tree **generated from the realized machine** — addresses
from the actual mappings, interrupt numbers from the wire graph — and Linux
6.12 riscv64 boots all the way to `prepare_namespace`, running every initcall
and moving its console off the SBI earlycon onto our own 16550A before it
panics for want of a root filesystem nobody gave it. **EDK2/UEFI boots to its
shell** — `UEFI Interactive Shell v2.2`, at a `Shell>` prompt — out of two CFI
NOR flash banks the board maps and the generated device tree describes, and a
variable written in one run is read back in the next. Where each stops is
written down in `docs/platforms/riscv-virt.md` rather than rounded up.

A seventh, `pc-at`, is a complete IBM PC/AT chipset — cascaded 8259As, 8254,
MC146818, 8042, two 8237As, MC6845/VGA text mode, µPD765A — with a
user-supplied BIOS path in the QEMU style (`--bios`, `--vgabios`). No firmware
is shipped and none will be.

Beside them are four synthetic boards, each the smallest machine that exercises
one thing: `spi-panel` (a display path over SPI), `arm926` (an ARM926EJ-S with a
parameterised peripheral aperture, the starting point for a downstream SoC),
`z80-mini` (the Z80's separate 64 KiB I/O space) and `m68k-mini` (a 68000 on a
big-endian map). They model no products; they exist so those subsystems have
somewhere real to run.

The framework underneath is complete: address spaces with priority and
mirroring, an oscillator forest with exact intra-tree ratios, wires, devices,
snapshots, a typed export seam so one device can hand another a handle, and a
`.machine` description language that goes parse → resolve → validate → realize
→ run. There is a **gdb stub** (`rsemu debug apple1 --gdb :1234`) and a
**browser build** at <https://karpeleslab.github.io/rsemu/>.

Not started: the IR and JIT, so everything is interpreted; hardware
acceleration; and i386 protected mode, without which no stock PC BIOS runs. See
[`ROADMAP.md`](ROADMAP.md).

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
# the minimal module: the ABI boundary and nothing else
cargo rustc --crate-type cdylib --target wasm32-unknown-unknown \
    --no-default-features --features wasm --release

# the demo the browser page runs — adds the machines it offers
cargo rustc --crate-type cdylib --target wasm32-unknown-unknown \
    --no-default-features --features demo --release
cp target/wasm32-unknown-unknown/release/rsemu.wasm web/public/
cd web && npm ci && npm run build && python3 -m http.server -d dist 8080
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
