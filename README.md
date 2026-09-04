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
- **`unsafe` is quarantined.** `unsafe_code = "deny"` crate-wide, and exactly
  **seven** subsystems opt back in with a scoped allow: the RAM host-pointer
  fast path, the JIT code buffer, the raw-syscall accel backend, the C and wasm
  ABIs, `core::sync`'s single-threaded backend, per-CPU execution state, and the
  host signal disposition. Seven is the ceiling and every block carries a
  `// SAFETY:` comment. `CLAUDE.md` records why the seventh was granted, as the
  worked example of how an eighth would have to be argued.
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
  is a CI target from the first commit, and no `mmap`, signals, or host clock
  appear anywhere in the core. The whole demo below runs interpreted: the JIT
  has one host backend and it is x86-64, so a **wasm backend is designed and not
  written** (`ROADMAP.md` §11.4) — wasm has no writable-then-executable memory
  and needs a different mechanism from the other two.
- **One crate, one feature per component.** A NES build links a 6502 and
  nothing else.

## Status

Early, but it runs things.

**Ten CPU cores.** Where a public corpus exists the number is measured, not
claimed; where one does not, the row says what stands in for it rather than
quietly leaving the impression of a number.

| Core | Suite | Result |
| --- | --- | --- |
| MOS 6502 / RP2A03 | SingleStepTests 65x02 | **2,560,000 / 2,560,000** incl. bus traces |
| WDC 65C02S | SingleStepTests 65x02 | **2,530,025 / 2,540,000** — the gap is the decimal half of two opcodes, argued in `cpu::mos6502::conformance` |
| Zilog Z80 | SingleStepTests z80, zexall | **1,604,000 / 1,604,000**, `zexall` 67/67; and **79/79** again as a `zexall.sms` cartridge inside an assembled Master System |
| RISC-V RV64GC | riscv-tests | **409 / 409** |
| RISC-V RV64GC | riscv-arch-test 3.9.1, signatures diffed against the Sail model | **181 / 181** — I, M, A, F, D, C, Zifencei, privilege. Which extension suites are *not* run, and why, is listed in [`docs/testing/README.md`](docs/testing/README.md) |
| Intel 8086/8088 | SingleStepTests 8088 | **2,974,160 / 3,007,000** |
| Intel 80386 | the 8088 corpus, replayed on a 386 | **2,650,981 / 3,007,000** — there is no hardware corpus for a 386, so every disagreement is traced to a documented difference between the parts and an opcode failing outside that list fails the test |
| MIPS I / R3000A | SingleStepTests r3000 | **55,000 / 55,000**, empty ledger. Those vectors were generated by another emulator's interpreter, so `cpu::mips::conformance` calls it a peer opinion rather than an oracle |
| Motorola 68000 | SingleStepTests 680x0 | all **124** instruction files pass all three checks |
| Sharp SM83 (Game Boy) | blargg, mooneye | blargg `cpu_instrs` and `instr_timing` **12/12** on the assembled machine, empty ledger; Gekkio's acceptance suite **59/66**, with the other seven ledgered and argued — three of them need a boot ROM we cannot ship |
| ARMv5TE | ARM7TDMI corpus (v4T subset) | no public v5 corpus exists — see §12 |
| ARMv7E-M | differential vs. our own ARMv5TE | every one of the 65,536 halfwords, twice: 83,597 encodings identical, 13,683 divergences *classified and asserted* rather than skipped |
| AArch64 (A64) | a suite rsemu **builds**, because none exists to download | **9 / 9** guests, empty ledger, 393,763 charged bus accesses. Four of them take their expectations from `rustc`'s own const-evaluator — an independent IEEE-754 implementation — rather than from us; `fp_rules` and the timer are directed tests transcribed from DDI 0487 and are checked by mutation instead, which [`docs/testing/README.md`](docs/testing/README.md) says out loud rather than counting them as conformance evidence |

Every corpus is fetched by `scripts/fetch-testdata.sh`, never vendored, and
gated behind an environment variable — a licensing rule as much as a size one.

**Thirty-one machine files**, and `machines/` is where they live: a machine is
described rather than compiled in. Which of them exists in a given binary is a
feature set, and `rsemu machines` lists what *your* build has.

Sixteen are consoles, computers and microcontrollers; the other fifteen are
synthetic boards that exist so a subsystem has somewhere real to run.

`nes-ntsc` and `nes-pal` pass **AccuracyCoin 141/141** — the whole-machine gate, run headlessly, with an
empty known-failures ledger. `gameboy` runs blargg's suite on the assembled
machine. `sms-ntsc` and `sms-pal` are a Sega Master System: a Z80 with a
**separate I/O address space**, a 315-5124 VDP and an SN76489 living in it,
Sega's bank-switched mapper, and a Pause button wired to /NMI. `beneater-6502`
and `apple1` are interactive over your terminal:

```console
$ cargo run --features machine-apple1 -- run apple1
RSMON
>FF00
FF00: D8 A2 FF 9A A9 7F 8D 12
```

### The boards that boot software this project did not write

There are seven, on three architectures. [`docs/README.md`](docs/README.md)
has a table comparing them and a page per board behind it, and those pages
record **where each one stops** rather than where it gets to. That is the useful
half: `docs/platforms/q35-linux.md` has driven three rounds of work, and two of
the four obstacles it named turned out to be *refuted* rather than fixed — their
real causes were somewhere else entirely — which is a thing a success report
cannot tell you.

Two caveats apply to every operating-system claim below, and they should be read
into all of them. **None of these boots runs in CI**: each is gated behind an
environment variable naming a kernel or a firmware image, because rsemu ships
neither and never will (see *License and provenance*), and the images are
whatever was on the machine that measured them — a distribution's own kernel, a
distribution's own OVMF — rather than an artifact pinned here. What *is* in CI
is a hermetic test per board that drives the same hardware with a guest this
repository builds; the boards' own firmware paths, where the firmware is
rsemu's, are in CI outright.

`riscv-virt` was the first and is still the furthest. OpenSBI 1.6 runs
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

`arm64-virt` is the newest, and the first AArch64 board here. A
Cortex-A53-class core, a **GICv2**, a **PL011**, a power controller for where
`PSCI_SYSTEM_OFF` lands, and a **device tree generated from the realized
machine** the same way the RISC-V board's is — addresses out of the map
statements, the UART's interrupt number out of the wire graph. Point it at
Debian's own `arm64` installer kernel and an initramfs and it **boots to a
busybox shell**, and `poweroff -f` typed at that shell reaches `PSCI_SYSTEM_OFF`
through an `SMC` and stops the machine, which is what the test asserts. About
three minutes of host time for 1.12 seconds of guest time, interpreted, in a
release build.

`arm64-virt-smp` is that board with a **second core**, and the same kernel
reports `smp: Brought up 1 node, 2 CPUs` on it. Three different problems had to
be solved for that: the GIC's banked registers now answer per
`MemAttrs::requester` — the machine file names the processors and the ids are
resolved when the machine binds, the same seam the local APIC's architectural
page uses — each core's generic timer is wired into its own bank of the
distributor, and the boot ROM's reset vector reads `MPIDR_EL1` and parks
everything but the boot processor on a release table. At the shell,
`/proc/interrupts` shows each processor's own timer count and the
interprocessor interrupts that went between them; `/proc/stat` shows the second
one running tasks.

`docs/platforms/arm64-virt.md` has the ledger, and it is long: PSCI `CPU_ON` is
not implemented, so the second core comes up off a spin table and cannot be
turned off again — servicing one means reaching a *sibling* core from inside
the one executing the `SMC`, and that page says exactly what the core would
need; no RTC, so `date` starts at the epoch; `GICC_CTLR.EOImode` unimplemented;
the `AT S1E1R` family unimplemented; `CLIDR_EL1` zero, so the guest sees no
caches. And one honest lie: the board asserts `psci = "smc"` on a core with no
EL3, which is the single place it tells a guest something its own
identification registers deny.

The x86 boards are five, and they divide by what starts first. `pc-at` is a
complete IBM PC/AT chipset — cascaded 8259As, 8254, MC146818, 8042, two 8237As,
MC6845/VGA text mode, µPD765A, an 82441FX host bridge with the PAM registers
that shadow the BIOS, and a PCI display adapter
whose expansion ROM BAR is where a firmware written this century looks for its
video BIOS — with user-supplied firmware paths in the QEMU style (`--bios`,
`--vgabios`). A real BIOS completes POST on it, runs the video option ROM, sets
a text mode and boots a diskette.

`pc-at-smp` is the same board with **two processors**, and it differs by five
lines — a second `cpu.x86`, a second local APIC, and one changed mapping. Both
the MP configuration table and the ACPI MADT carry a single local-APIC address,
because on silicon the register block is on the processor's own die; so
`0xfee00000` decodes to a *window* that demultiplexes on which processor is
asking, and each one reaches its own APIC through the one address an operating
system uses on both.

**No third-party firmware is shipped and none will be** — but there is now one
of our own. `rsemu run pc-at --hd0 disk.img` boots with nothing supplied,
because `src/fw/pcbios` is a minimal legacy BIOS written here: POST, the BIOS
Data Area, option-ROM dispatch, `INT 10h`/`11h`/`12h`/`13h`/`15h`/`16h`/`19h`/
`1Ah`, and a bootstrap that reads the first sector and jumps to it. It exists
because FreeDOS, Windows 95 and Windows XP all need a *legacy* BIOS and every
one anybody could reach for is GPL. There is no assembler in this repository
and Rust cannot target 16-bit x86, so the ROM is **emitted**: `src/fw/asm16` is
a 16-bit x86 assembler in Rust and the firmware is a Rust program that calls
it, which makes `cargo build` the whole build. On that firmware `pc-at` **boots
FreeDOS 1.3 to its installer prompt** — the board sizes 16 MiB of RAM, shadows
itself out of ROM into RAM through the 82441FX's PAM registers, enumerates PCI,
maps and runs a video card's option ROM off an expansion-ROM BAR, reads a
diskette through the µPD765 and the 8237, and jumps to `0000:7c00`, where
FreeDOS's own boot sector takes over. Where it stops is written down: the
installer cannot be driven past its first keystroke, because `pc.kbc` delivers
one and then goes silent (`docs/platforms/pc-at.md`).

The other four x86 boards are modern. `q35` is the chipset — an 82Q35 (G)MCH
with **ECAM** as well as the `0xcf8` pair, an ICH9 with the `PIRQ[A-H]` routers,
the PAM file, and **ACPI tables generated from the realized machine** rather
than written down. A third-party PC firmware POSTs on it to a boot prompt, and
rsemu's own BIOS boots a guest off its IDE drive.

`pc64` and `q35-linux` skip the firmware entirely: `x86.linuxboot` writes a
`bzImage` into memory and enters it at its own 32-bit entry point, so the first
thing that runs after the reset vector is the kernel's decompressor. Both reach
a shell. `pc64` is the smaller claim and the sharper instrument — eight objects,
no APIC, no PCI, no video — and it proves the core survives early boot on a
machine with nothing on it. `q35-linux` is the whole chain: a **stock Gentoo
6.6.67 kernel**, unmodified and never read here, finds an RSDP by scanning a
window nothing staged, reads six generated tables under it, assigns base
addresses out of a `_CRS` the board generates, routes a PCI interrupt through
`_PRT` and an I/O APIC redirection entry, binds its own **NVMe** driver to the
controller at `00:04.0`, and busybox reads forty bytes off the namespace with
`head -c 40 /dev/nvme0n1`. On **the board's own default command line** — that is
the part worth the sentence, because it used to need three extra words and every
one of them was hiding a defect here.

`q35-uefi` is the same chipset with the ROM socket replaced by **two banks of
parallel NOR flash** below 4 GiB, which is the layout every split OVMF build is
compiled for. A real OVMF runs SEC out of flash, sizes memory in PEI from the
CMOS, decompresses `FVMAIN`, and reaches the **DXE dispatcher** — and stops
there, on `MOV RAX, CR8`, an instruction this core raises `#UD` for. It is the
weakest claim in this section, and `docs/platforms/q35-uefi.md` is the reason
you should read that rather than this: the board has no video and no `0x402`
debug port, so the firmware prints *nothing at all*, and "reaches DXE" is
inferred from register state rather than read off a console.

`stm32f407` is a microcontroller rather than a computer: an **STM32F407VGT6**,
the part on ST's own STM32F4 Discovery board — a Cortex-M4 out of flash aliased
at zero, six GPIO ports as instances of one class, and USART2 on your terminal.
It is where an M-profile core answers the question the other boards never ask,
because a Cortex-M's interrupt controller is *inside the core*: a peripheral
drives `cpu.irq38` directly, and 38 is USART2's row in the part's vector table,
written in the machine file where the part is chosen rather than in any device
model.

Beside them are the fifteen synthetic boards, each the smallest
machine that exercises one thing: `spi-panel` (a display path over SPI),
`spi-flash` (an RV32 program that programs a Winbond part through an OCTOSPI
window and then *executes out of it*), `arm926` (an ARM926EJ-S with CP15, the
VMSAv5 MMU and a parameterised peripheral aperture, the starting point for a
downstream SoC), `a64-mini` (where an AArch64 guest builds its own three-level
translation tables and turns the MMU on), `z80-mini` (the Z80's separate 64 KiB
I/O space), `m68k-mini` (a 68000 on a big-endian map), `mips-mini` (an R3000A
whose board maps *physical* addresses, so it is the processor that turns the
kseg1 reset vector into ROM) and `pc-apic` (two x86s, two local APICs and an I/O
APIC — the interrupt path an SMP PC needs, and the board the KVM backend runs
on). They model no products; they exist so those subsystems have somewhere real
to run. `ne2k-mini` is the newest of them: a Z80 with an
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

`xhci-mini` is that board with the controller swapped and nothing else changed,
which is what makes the comparison worth anything. An **xHCI** is shaped like
NVMe rather than like EHCI: the driver builds a Device Context Base Address
Array, a command ring, an event ring with a segment table and one transfer ring
per endpoint, and hands them over a doorbell at a time — with the **Cycle bit**
in each Transfer Request Block as the ownership flag, so a ring is a cycle by
construction and every walk over one is bounded. The RV32 program on this board
resets the root port, issues Enable Slot and Address Device, reads the device
descriptor over the default pipe, configures two bulk endpoints and then moves
the same CBW/data/CSW triples to the same disk — checked against the same
`Medium::read_at` as the EHCI board's, so "the bytes came back" cannot be
satisfied by a controller echoing its own buffers. Its completions arrive as
event TRBs, and acknowledging one is three writes in the order the specification
fixes; the test counts the guest's traps and asserts fifteen, because the wrong
order measures thirty.

`hub-mini` is `usb-mini` again with one object inserted: a **USB hub** between
the controller and the disk. A hub is the first device here whose interesting
half is not what it says about itself but *where other things are* — and it is
not a router, because the address on the wire is flat and a hub never looks at
it. What makes a device behind one reachable is the host powering, resetting and
enabling the port it is on, through class requests it addresses to the hub like
to anything else; so the hub's downstream ports are simply a **second named
bus**, and the disk behind it is an ordinary object whose `bus` is that name.
Neither mentions the other. The RV32 program enumerates the hub, reads its
descriptor, powers a port, watches the connection appear *because* of the power,
resets it, watches it enable — and then addresses and reads a disk that is on no
root port at all, and moves a sector each way over its bulk endpoints, checked
against the same `Medium::read_at`. What a hub here still cannot do is carry a
*slow* device to a high-speed controller: that is the transaction translator,
which is a second and larger deliverable, and the port says so by not enabling
rather than by pretending.

`xhci-pci-mini` is the first board here with a **screen and a mouse at once**,
and the reason it could not exist before is one line of `dev/`: every USB
controller in this tree was MMIO-attached only, so a PC guest — which finds a
host controller by enumerating the bus for class code `0C0330h`, not by knowing
an address — could never have found one. `usb.xhci-pci` is that attachment: the
same xHCI engine behind a Type 00h configuration header, its register block on a
64-bit base address register the guest sizes and places itself, and `INTA#` onto
the fabric's shared level-triggered net. A driver enumerates it, enables Memory
Space and Bus Master, addresses a HID boot mouse and pulls a report off its
interrupt endpoint — a report that a VNC client's `PointerEvent` put there,
through the input seam that until now had nowhere to deliver one. Bus Master
Enable is not decoration: with `COMMAND[2]` clear the controller fetches
*nothing*, and the test asserts it.

The framework underneath is complete: address spaces with priority and
mirroring, an oscillator forest with exact intra-tree ratios, wires, devices,
snapshots, a typed export seam so one device can hand another a handle, and a
`.machine` description language that goes parse → resolve → validate → realize
→ run. There is a **gdb stub** (`rsemu debug apple1 --gdb :1234`) — driven end to end
by a **real `gdb` binary** in `tests/gdb_real_client.rs`, which attaches, reads
registers, writes a program into guest RAM, sets a breakpoint, hits it and steps
— and a **browser build** at <https://karpeleslab.github.io/rsemu/>.

That page is not a screenshot. Seven machines are in it, four of which boot on
an image the 3 MB module carries, so there is something to press before there is
anything to open: rsemu's own monitors, the public-domain Woz Monitor of 1976,
an RV32 board painting a gradient through a real SPI display path, and **a
PC/AT posting on rsemu's own BIOS**. You can type at that PC — one export hands
a key transition to the same keysym→scan-code table the VNC server uses — and
open a diskette image into its drive, which boots under the firmware the module
assembled. Nothing is uploaded: a file you pick is read in the page, and a save
state is a file the tab writes. [`web/README.md`](web/README.md) records what
each machine costs in bytes and why the boards that are *not* there are not,
which is the more useful half of that document.

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

Those two flags are not the frontend's. `--record-input` seals the board's
**host-object table** for the build, so every door a device opens — a console,
a controller, a network port — is wired to the recorder before the machine has
executed an instruction, and a board with an input that nothing can record
*refuses to build* rather than producing a log with a stream quietly missing
from it. `printf 'E000.E00F\r' | rsemu run apple1 --for 1s --record-input
session.trace` records a terminal session, and `rsemu run apple1 --for 1s
--replay-input session.trace` types it again on a machine with nothing on its
stdin, arriving at the same state hash.

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

### Three ways to execute a guest

**The interpreter is the oracle**, always: every other engine is differentially
tested against it, and `tests/riscv_virt_engines.rs` asserts identical state
hashes at ten checkpoints across all three plus a snapshot restored *across* an
engine switch.

The **translation IR** landed first — the architecture-neutral op set, typed SSA
blocks, the guest-instruction-boundary markers that make a mid-block fault
deliverable at the right PC with the right cycle count, a verifier, liveness and
dead-code elimination, and a portable interpreter backend that needs no `unsafe`
and runs on every target including bare metal. **RISC-V and x86 both have
frontends now**, and both cores take an `engine` property with three values:
`interp`, `jit` (the portable backend), and `jit-host` (native code).

There is **one host backend and it is x86-64 Linux**. On it, over a 900-second
Linux boot on `pc64`, medians of three interleaved runs:

| Engine | RISC-V (`riscv-virt`, 240 s of guest time) | x86 (`pc64`, 900 s) |
| --- | --- | --- |
| `interp` | 122.3 s | 279.9 s |
| `jit` | 103.7 s (1.18×) | 206.6 s (1.35×) |
| `jit-host` | **56.8 s (2.15×)** | **105.8 s (2.65×)** |

All three produce byte-identical guest output — 710 console lines on the x86
run, with the same `CS:RIP`, `CR2`, `CR3`, `CR4`, `EFER` and flags at the end.
**87.6% of the x86 guest's instructions retire inside a translated block**
(1,573,762,719 of 1,797,400,802), and 99.8% of compiled RISC-V stores write
guest RAM inline rather than through a call. The RISC-V headline *fell* from
2.28× to 2.15× along the way, because the interpreter it is measured against got
**1.27× faster** (155.1 s → 122.3 s) and the control moved; the numbers and that
argument are in
[`docs/platforms/riscv-virt.md`](docs/platforms/riscv-virt.md) and
[`docs/platforms/pc64.md`](docs/platforms/pc64.md). No aarch64 backend, and **no
wasm backend** — the browser runs interpreted.

Two caveats on all of the above. The machine files still write `engine =
"interp"` as a literal rather than a `param`, so on the x86 boards the engine is
not yet selectable from the command line and those figures were taken through a
test-harness override. And no number in this repository sits on the declared
reference host, because [`docs/bench-host.md`](docs/bench-host.md) has not been
filled in — by the project's own rule that makes every one of them informative
rather than gating.

**Hardware acceleration** is real, and it is KVM on Linux x86-64. `q35-linux`
boots that same stock Gentoo 6.6.67 kernel to a busybox shell in **about ten
seconds of wall clock (9-12 s measured)** against **973 s** interpreted, and
**279 of the accelerated run's 347 console lines are byte-identical** to the
interpreted run's, in the same order, once the printk timestamp is removed — the
68 that differ are the ones that describe the *host* processor, its mitigations,
its XSAVE list and its TLB geometry. On a pure-execution workload the ratio to
native is **99.7%, 99.9% and 101.2%** across three runs, against a phase-7 gate
of 80%. That pair of boots is not on the board's own default line, though: an
accelerated run still needs `no_timer_check`, and the reason is in the scheduler
rather than the board. Two more things it is not yet: `engine = "kvm"` is not a machine-file value
and there is no `--kvm` flag, so acceleration is reached through the API rather
than a command line; and `ThreadingMode::Accel` is unimplemented, so an
accelerated board runs `Parallel` and is not reproducible. HVF and WHPX are
roadmap entries with no code behind them.

**Level 3 — user-mode execution** — has its proof: a static musl Rust binary,
built by `scripts/fetch-testdata.sh` for `riscv64gc-unknown-linux-musl` and
never committed, runs through musl's own `_start` and `__libc_start_main` —
`AT_PHDR`, thread-local storage, a `brk` heap — reaches `main`, prints, and
**exits 0**, with no syscall refused. The run is then replayed with the entropy
source replaced by a panicking guard, and produces identical output and an
identical tick count. Forty syscalls, and a hard rule: *a level-3 guest may be
told about itself, and may not be told about the host* — there is no filesystem,
`mmap` is anonymous-only, and there is no `--allow` flag and none planned. rsemu
builds the machine half only; the kernel half is
[`nixvm`](https://github.com/KarpelesLab/nixvm)'s (`ROADMAP.md` §2.1), which is
why all of this lives under `#[cfg(test)]` and none of it is public API.
[`docs/system/usermode-abi.md`](docs/system/usermode-abi.md) has the syscall
table and the differential traces against a host `strace`.

See [`ROADMAP.md`](ROADMAP.md) for what remains.

## Build

```sh
cargo build              # library + the rsemu binary
cargo test --all-features
cargo run -- --version

cargo build --no-default-features   # no_std core, as CI checks it

scripts/check.sh         # everything CI gates on, per commit
scripts/check.sh --all   # plus the full per-feature sweep (long)
```

`scripts/check.sh` is the whole CI workflow as one command, so "I ran the
tests" and "CI is green" stop being different claims. It prints one marked
summary at the end and exits non-zero if any stage failed; `--list` names the
stages and any of them runs alone.

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

The C ABI — `rsemu run` as twenty-three `extern "C"` functions, so a program
that is not written in Rust can build a machine, run it for an amount of
virtual time, read and write its memory, snapshot it and hash it:

```sh
cargo rustc --lib --release --features ffi --crate-type staticlib
cargo rustc --lib --release --features ffi --crate-type cdylib
```

The header is [`include/rsemu.h`](include/rsemu.h). It is **generated** from
`src/ffi/abi.rs` and compared against it by `cargo test --features ffi`, so it
cannot drift; regenerate it with `RSEMU_UPDATE_HEADER=1`. There is no cbindgen
— the dependency policy has no room for one.

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
partition tables, and read-write ext/FAT/exFAT/NTFS/XFS/HFS+).

[`noroi`](https://github.com/KarpelesLab/noroi) is *not* among them. It was
listed here for a monitor TUI that has not been built and is not planned: the
commands such a UI would carry already answer over GDB's `monitor`, and noroi's
`std` layer is Linux-only by construction, so it could not be in the
`--all-features` build our macOS and Windows jobs test. The reasoning is in
[`docs/system/debug-protocols.md`](docs/system/debug-protocols.md).

## License and provenance

MIT — see [LICENSE](LICENSE).

rsemu is written **clean-room from hardware documentation**. MIT cannot absorb
GPL'd code, so copyleft sources are off limits to contributors — **the QEMU
source tree above all**, along with Bochs, DOSBox, MAME, VICE, Dolphin, PCSX2
and every other GPL/LGPL emulator. We work from datasheets, ISA manuals, the
NESdev wiki, Pan Docs and real hardware; permissively licensed code is welcome
with its attribution intact. Benchmarking against a GPL emulator is fine —
that is black-box use, not derivation. **TianoCore EDK II is readable**: it is
BSD-2-Clause-Patent, so it is a reference rather than a hazard, and it is the
one substantial open firmware implementation that is not copyleft. SeaBIOS,
coreboot and QEMU's firmware are not.

**No guest image is shipped and none will be.** Kernels, BIOS images, cartridges
and conformance corpora are fetched by `scripts/fetch-testdata.sh` into an
ignored directory and gated behind an environment variable — running a GPL
binary as an emulated guest is ordinary use, while redistributing one from this
repository would not be. The one firmware here is `src/fw/pcbios`, which is
ours.

[`docs/`](docs/) is the curated register of primary sources — ISA manuals,
platform specs, PCI/USB/virtio, OSDev resources and conformance suites — each
annotated with what it authoritatively answers and whether it is safe to quote.
It also carries [the board table](docs/README.md) and a page per board under
`docs/platforms/`, each of which is a ledger of what is still in the way.

See [CONTRIBUTING.md](CONTRIBUTING.md) before your first patch, and
[`ROADMAP.md` §1](ROADMAP.md) for the full policy.
