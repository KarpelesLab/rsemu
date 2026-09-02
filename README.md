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
- **Multithreaded by design.** Guest CPUs, background JIT compilation and
  device I/O can all run in parallel, through one portability seam, so a device
  is written once. Threading is a *mode*, and the modes differ in what they
  promise: `deterministic` runs the guests on one thread and is
  bit-reproducible, and background work never changes what it computes;
  `parallel` gives every CPU a thread and gives up reproducibility for speed,
  because two guests racing through memory is not a reproducible thing. Asking
  a `parallel` machine for a state hash is an error rather than a number.
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
| Zilog Z80 | SingleStepTests z80, zexall | **1,604,000 / 1,604,000**, 67/67; **77/77** again inside a Master System |
| RISC-V RV64GC | riscv-tests | **409 / 409** |
| RISC-V RV64GC | riscv-arch-test 3.9.1, signatures diffed against the Sail model | **181 / 181** — I, M, A, F, D, C, Zifencei, privilege. Which extension suites are *not* run, and why, is listed in [`docs/testing/README.md`](docs/testing/README.md) |
| Intel 8086/8088 | SingleStepTests 8088 | **2,974,160 / 3,007,000** |
| Motorola 68000 | SingleStepTests 680x0 | runner in-tree; fetch the corpus to reproduce |
| Sharp SM83 (Game Boy) | blargg, mooneye | `cpu_instrs` 11/11; mooneye acceptance 22/66 |
| ARMv5TE | ARM7TDMI corpus (v4T subset) | no public v5 corpus exists — see §12 |
| ARMv7E-M | differential vs. our own ARMv5TE | 83,597 encodings identical, 13,683 divergences *asserted* |

Every corpus is fetched by `scripts/fetch-testdata.sh`, never vendored, and
gated behind an environment variable — a licensing rule as much as a size one.

**Eight machines you can run.** `nes-ntsc` and `nes-pal` boot a cartridge, raise
NMI and render — AccuracyCoin draws its menu. `gameboy` runs blargg's suite.
`sms-ntsc` and `sms-pal` are a Sega Master System: a Z80 with a **separate I/O
address space**, a 315-5124 VDP and an SN76489 living in it, Sega's bank-switched
mapper, and a Pause button wired to /NMI. `beneater-6502` and `apple1` are
interactive over your terminal:

```console
$ cargo run --features machine-apple1 -- run apple1
RSMON
>FF00
FF00: D8 A2 FF 9A A9 7F 8D 12
```

`riscv-virt` is the one that boots real system software. OpenSBI 1.6 runs
completely on a device tree **generated from the realized machine** — addresses
from the actual mappings, interrupt numbers from the wire graph — and Linux
6.12 riscv64 **boots to a shell prompt that echoes what is typed at it**,
running every initcall, moving its console off the SBI earlycon onto our own
16550A, and reaching busybox on an initramfs `scripts/fetch-testdata.sh`
builds — two and a half minutes of host time under the interpreter. Load the
kernel's own `virtio_mmio` and `virtio_blk` and it drives the board's virtio
disk as well. **EDK2/UEFI boots to its shell** — `UEFI Interactive Shell v2.2`,
at a `Shell>` prompt — out of two CFI NOR flash banks the board maps and the
generated device tree describes, and a variable written in one run is read back
in the next. Where each stops is written down in
`docs/platforms/riscv-virt.md` rather than rounded up.

`stm32f407` is a microcontroller rather than a computer: an **STM32F407VGT6**,
the part on ST's own STM32F4 Discovery board — a Cortex-M4 out of flash aliased
at zero, six GPIO ports as instances of one class, and USART2 on your terminal.
It is where an M-profile core answers the question the other boards never ask,
because a Cortex-M's interrupt controller is *inside the core*: a peripheral
drives `cpu.irq38` directly, and 38 is USART2's row in the part's vector table,
written in the machine file where the part is chosen rather than in any device
model.

A seventh, `pc-at`, is a complete IBM PC/AT chipset — cascaded 8259As, 8254,
MC146818, 8042, two 8237As, MC6845/VGA text mode, µPD765A, an 82441FX host
bridge with the PAM registers that shadow the BIOS, and a PCI display adapter
whose expansion ROM BAR is where a firmware written this century looks for its
video BIOS — with user-supplied firmware paths in the QEMU style (`--bios`,
`--vgabios`). A real BIOS completes POST on it, runs the video option ROM, sets
a text mode and boots a diskette.

**No third-party firmware is shipped and none will be** — but there is now one
of our own. `rsemu run pc-at --hd0 disk.img` boots with nothing supplied,
because `src/fw/pcbios` is a minimal legacy BIOS written here: POST, the BIOS
Data Area, option-ROM dispatch, `INT 10h`/`11h`/`12h`/`13h`/`15h`/`16h`/`19h`/
`1Ah`, and a bootstrap that reads the first sector and jumps to it. It exists
because FreeDOS, Windows 95 and Windows XP all need a *legacy* BIOS and every
one anybody could reach for is GPL. There is no assembler in this repository
and Rust cannot target 16-bit x86, so the ROM is **emitted**: `src/fw/asm16` is
a 16-bit x86 assembler in Rust and the firmware is a Rust program that calls
it, which makes `cargo build` the whole build.

Beside them are four synthetic boards, each the smallest machine that exercises
one thing: `spi-panel` (a display path over SPI), `arm926` (an ARM926EJ-S with CP15,
the VMSAv5 MMU and a parameterised peripheral aperture, the starting point for
a downstream SoC),
`z80-mini` (the Z80's separate 64 KiB I/O space) and `m68k-mini` (a 68000 on a
big-endian map). They model no products; they exist so those subsystems have
somewhere real to run. `ne2k-mini` is the newest of them: a Z80 with an
**NE2000 Ethernet card** on its port bus, whose ROM is a real driver — it runs
the DP8390's initialisation procedure, builds a frame in card memory through
the remote DMA window, transmits it, and takes the receive interrupt in mode 1.
`nvme-mini` is the one after it, and it exists for the opposite reason: an
**NVM Express controller** is the first device here that reads and writes guest
memory *itself*. A driver builds a submission queue, a completion queue and a
list of Physical Region Pages in the board's own RAM, writes one doorbell, and
the controller fetches the command, walks the chain, moves the data to or from
the disk image, posts a completion with its phase tag and holds its interrupt
line down until the driver acknowledges it. The board is RAM, a host bridge, an
8259A and the controller — nothing else, so a failure on it is a failure in the
device. `ahci-mini` is its twin for the *other* way a modern machine reaches a
disk: a **Serial ATA host bus adapter**, which is a bus master over an ordinary
ATA drive — the same drive object the PC/AT hangs off its IDE cable, with the
same command set behind it. That reuse is the point of the work rather than a
side effect: an AHCI port carries an ATA command, so the drive grew a *taskfile*
seam — the command block as a struct, loaded into the same registers and
dispatched by the same code a port write reaches — and `src/dev/pc/ide.rs` did
not change by one line.

`usb-mini` is the third way to a disk and the one that reuses the most: a **USB
mass storage device** speaking Bulk-Only Transport over a SCSI command set, on
the **EHCI host controller that was finished before it existed**. Bulk-Only is
two bulk endpoints and the default pipe, and the controller already walked bulk
queue heads — so an RV32 program on this board enumerates the disk, pushes a
Command Block Wrapper out of an endpoint and pulls a sector and a Command Status
Wrapper back in, and the sector that lands in its RAM is the sector on the
medium. The disk's bytes are the same `Medium` an ATA drive, an AHCI port and an
NVMe namespace read, so `--drive usb0=disk.qcow2` works here for the same reason
and through the same media slot. Its completion interrupt is the one that is not
polled: it travels a wire into a PLIC and the guest takes a real trap for it.

The framework underneath is complete: address spaces with priority and
mirroring, an oscillator forest with exact intra-tree ratios, wires, devices,
snapshots, a typed export seam so one device can hand another a handle, and a
`.machine` description language that goes parse → resolve → validate → realize
→ run. There is a **gdb stub** (`rsemu debug apple1 --gdb :1234`) — driven end to end
by a **real `gdb` binary** in `tests/gdb_real_client.rs`, which attaches, reads
registers, writes a program into guest RAM, sets a breakpoint, hits it and steps
— and a **browser build** at <https://karpeleslab.github.io/rsemu/>.

A machine is also **watchable over the network**. `rsemu run pc-at --vnc :5900`
serves the display over RFB (RFC 6143) and takes keyboard and pointer events
back from whoever connects; there is no GUI dependency because there is no GUI —
a socket, a framebuffer and a scan-code table are the whole of it. Input crosses
into the machine the way every non-deterministic input has to — through the
record/replay seam: a keystroke is *posted* whenever the human produced it, and
the machine delivers it at the top of a scheduling round and logs it against
that round's instant. So `--record-input` and `--replay-input` reproduce a
session bit for bit, which `tests/vnc_input.rs` asserts by comparing state
hashes — against a run nobody typed at, which reaches a different one.

There is **sound**, too. The audio seam mirrors the display one: a device emits
what the silicon does — the RP2A03 emits an unsigned level out of a non-linear
DAC pair at 894 886.36… Hz — and the host applies the board's own RC network,
resamples with an exact integer phase, and either writes a `.wav`
(`rsemu run nes-ntsc --cart game.nes --for 5s --record-audio game.wav`) or hands
it to WebAudio in the browser. A headless capture is bounded by what the
device's ring can hold, because a headless run visits the host only once; a
`--vnc` session drains that ring every frame and so records a run of any
length. Every float in that path is an amplitude, never a
duration, so a machine's state hash does not depend on whether anybody is
listening. There is no native sound-card backend for the same reason there is no
native window: ALSA is an `ioctl` protocol and the alternative to `libc` is a
seventh `unsafe` subsystem, which the ceiling of six forbids.

The **translation IR** has landed — the architecture-neutral op set, typed SSA
blocks, the guest-instruction-boundary markers that make a mid-block fault
deliverable at the right PC with the right cycle count, a verifier, liveness
and dead-code elimination, and a portable interpreter backend that needs no
`unsafe` and runs on every target including bare metal. A RISC-V frontend lifts
an RV64I subset into it, and the whole path is tested end to end: guest bytes
in, IR out, verified, executed, with a tick count asserted equal to what the
interpreter charges for the same program. Not started: the **host** JIT
backends, so everything still executes interpreted; hardware acceleration; and
the rest of the pass pipeline. See [`ROADMAP.md`](ROADMAP.md).

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
