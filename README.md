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
  scripts. The default build has an empty `cargo tree` — literally just
  `rsemu`, and CI gates on it — and every dependency is feature-gated. Turning
  *everything* on costs eleven crates, seven of them first-party Karpelès Lab;
  the other four are one transitive chain and the *Built on* section names it
  rather than rounding it away.
- **`unsafe` is quarantined.** `unsafe_code = "deny"` crate-wide, and exactly
  **seven** subsystems *may* opt back in with a scoped allow: the RAM
  host-pointer fast path, the JIT code buffer, the raw-syscall accel backend,
  the C and wasm ABIs, `core::sync`'s single-threaded backend, per-CPU execution
  state, and the host signal disposition. **Five of the seven have actually
  needed it** — the RAM fast path and per-CPU execution state still have not.
  Seven is the ceiling and every block carries a `// SAFETY:` comment.
  `CLAUDE.md` records why the seventh was granted, as the worked example of how
  an eighth would have to be argued.
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
  appear anywhere in the core. wasm has no writable-then-executable memory, so
  it needs a different mechanism from the other two backends, and it has one:
  the **wasm backend** (`ROADMAP.md` §11.4) lowers an IR block to a
  `WebAssembly.Module` rather than to machine code and reaches the same state
  hash as the interpreter on the same guest. **The browser embedder is built**:
  three imports, four exports and forty lines in
  [`web/src/jit.js`](web/src/jit.js), measured at **1.50× the IR interpreter**
  in V8 on an RV64I guest — per basic block, with no chaining and no
  superblocks, which is more than `ROADMAP.md` §11.4 expected.
  [`docs/techniques/wasm-jit.md`](docs/techniques/wasm-jit.md) is the design
  note.
- **One crate, one feature per component.** A NES build links a 6502 and
  nothing else.

## Status

Early, but it runs things.

**Ten CPU cores**, and a floating-point coprocessor beside the 680x0. Where a
public corpus exists the number is measured, not claimed; where one does not,
the row says what stands in for it rather than quietly leaving the impression of
a number.

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
| Motorola 68000 | SingleStepTests 680x0 | all **124** instruction files pass all three checks — **1,000,058** vectors, state, cycles and bus trace |
| Motorola 68010 / 68020 / 68030 / 68040 | the 68000 corpus, replayed on each | no public corpus exists for any of them, so every 68000 vector is run through each later model and **every disagreement must be a documented difference** — the 68010's format-`$8` frames, the 68020's misaligned operands and scaled index — or the test fails: **774,984** identical on the 68010 in state, cycles *and* trace, **731,457** on the 68EC020, the 68030 and the 68040 in state, nothing unexplained. [`docs/cpu/m68k.md`](docs/cpu/m68k.md) has the classification |
| Motorola 68881 / 68882 | 133 `(argument, result)` pairs computed by GNU `bc -l` at `scale=110` | all **133 / 133** bit-identical — *correctly rounded*, not merely within an ulp, which is stricter than the part itself. M68881UM §4.3.2 bounds a real coprocessor's transcendentals at 64 units in the last place typically and 4096 at worst, and an error pattern that loose is not something to reproduce; `src/float` is integer arithmetic rounded once, so the answer is the same on every host and in a browser |
| Sharp SM83 (Game Boy) | blargg, mooneye | blargg `cpu_instrs` and `instr_timing` **12/12** on the assembled machine, empty ledger; Gekkio's acceptance suite **59/66**, with the other seven ledgered and argued — three of them need a boot ROM we cannot ship |
| ARMv5TE | ARM7TDMI corpus (v4T subset) | no public v5 corpus exists — see §12 |
| ARMv7E-M | differential vs. our own ARMv5TE | every one of the 65,536 halfwords, twice: 83,597 encodings identical, 13,683 divergences *classified and asserted* rather than skipped |
| AArch64 (A64) | a suite rsemu **builds**, because none exists to download | **9 / 9** guests, empty ledger, 393,763 charged bus accesses. Four of them take their expectations from `rustc`'s own const-evaluator — an independent IEEE-754 implementation — rather than from us; `fp_rules` and the timer are directed tests transcribed from DDI 0487 and are checked by mutation instead, which [`docs/testing/README.md`](docs/testing/README.md) says out loud rather than counting them as conformance evidence |

Every corpus is fetched by `scripts/fetch-testdata.sh`, never vendored, and
gated behind an environment variable — a licensing rule as much as a size one.

**Forty-five machine files**, and `machines/` is where they live: a machine is
described rather than compiled in. Which of them exists in a given binary is a
feature set, and `rsemu machines` lists what *your* build has.

Twenty-eight are consoles, computers and microcontrollers; the other seventeen
are synthetic boards that exist so a subsystem has somewhere real to run.

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

There are sixteen, on four architectures — counting a board and its
two-processor twin as one, which is what they are: an `-smp` file is the same
board with a second `cpu` object and a table told about it. **Nine of the
sixteen are 680x0** — eight Amigas and a Macintosh Plus — five are x86, and
RISC-V and AArch64 have one board each. **Four of those twins exist** —
`riscv-virt-smp`, `arm64-virt-smp`, `q35-linux-smp` and `pc-at-smp` — and
**three of them boot a real kernel onto both processors, with no accelerator**;
the fourth, `pc-at`'s, runs rsemu's own boot sector and no operating system,
though the single-processor `pc-at` installs FreeDOS 1.3 onto a hard disk and
boots off it.
[`docs/README.md`](docs/README.md) has a table comparing them and a page per
board behind it, and those pages record **where each one stops** rather than
where it gets to. That is the useful half: `docs/platforms/q35-linux.md` has
driven three rounds of work, and two of
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

`riscv-virt-smp` is that board with a **second hart**, and the same kernel
reports `smp: Brought up 1 node, 2 CPUs` on it and then **runs userspace on
both** — `nproc` says 2, `/proc/interrupts` has a column per hart with the
timer counted separately in each and interprocessor interrupts going both
directions, and `/proc/stat` gives the second hart more system time than the
first. It needed less than either of
the other two multiprocessor boards, and for a reason that is architectural
rather than lucky: a GICv2 *banks* its low interrupt ids and an x86 local APIC
shares one page between processors, so both had to be taught to demultiplex on
`MemAttrs::requester` — while RISC-V gives every hart its own **address** for
`msip`, `mtimecmp` and its PLIC context. So the CLINT and the PLIC were already
per-hart, the device tree generator already emitted a node per hart, and an
**IPI is a store to a sibling's `msip` word** rather than a mechanism. The
second hart is started through **SBI HSM** — the firmware's own hart state
machine — which is the RISC-V spelling of what `arm64-virt-smp` now does with
PSCI `CPU_ON`.

`arm64-virt` is the first AArch64 board here. A
Cortex-A53-class core, a **GICv2**, a **PL011**, a power controller for where
`PSCI_SYSTEM_OFF` lands, and a **device tree generated from the realized
machine** the same way the RISC-V board's is — addresses out of the map
statements, the UART's interrupt number out of the wire graph. Point it at
Debian's own `arm64` installer kernel and an initramfs and it **boots to a
busybox shell**, and `poweroff -f` typed at that shell reaches `PSCI_SYSTEM_OFF`
through an `SMC` and stops the machine, which is what the test asserts. About
three minutes of host time for 1.12 seconds of guest time, interpreted, in a
release build.

It has a **disk** now, too — two `virtio` MMIO windows, the same transport and
the same device models the RISC-V board uses, moved out from under `dev/riscv/`
into `src/dev/virtio` so that both boards reach them. Give it a root image and
the kernel loads its own `virtio_mmio`, `virtio_blk` and `ext4` modules,
**mounts `/dev/vda` as an ext4 root** and runs the shell out of it — which is
the difference between a machine that boots and a machine you can put a
filesystem on.

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

**Guest atomics are kept, on all three architectures.** Each was broken in its
own way and each is now closed. `core::space::monitor` is a **global exclusive
monitor** on the address space, hooked at the single funnel every guest store
passes through, so an ordinary store by any observer breaks a covering
reservation: `LDXR`/`STXR` and `LR`/`SC` do what the architecture requires.
x86 has no reservation to break — a `LOCK`ed read-modify-write is
unconditional and has no status flag to report a failure through — so it takes
a **bus lock** (`LockRank::BUS_LOCK`, above `BUS` and below every bus fabric)
held across the read and the write of one instruction. The two compose without
knowing about each other: a locked write still goes out through the same funnel,
so it breaks reservations on its way past.

The evidence is a number that used to come out wrong. An AArch64
`AtomicU32::fetch_add` loop over two cores landed **32,038 of 40,000**; it lands
40,000 now, and reverting the one check reproduces 32,038 exactly. Two x86
interpreters on two host threads running `lock xadd` land **40,000 of 40,000**,
against 34,271 with the bus lock removed. Both are hermetic tests.

**Two residuals, recorded rather than papered over**, and
[`docs/techniques/memory-models.md`](docs/techniques/memory-models.md) measures
both rather than asserting them.

*Single-copy atomicity is not kept.* `RamStore` is a `Vec<AtomicU8>` and every
access to it is a byte loop, so a naturally aligned four-byte load racing a
naturally aligned four-byte store can come back a mixture of the old and the new
word — a value all three architectures forbid (*Intel SDM* vol. 3 §9.1.1, ARM
DDI 0487 B2.2.1, RISC-V Unprivileged ISA §1.4).
`tests/smp_single_copy_atomicity.rs` catches it 117–361 times in sixty thousand
loads, and catches the read half of a `LOCK XADD` torn the same way with the bus
lock held throughout — so locked-against-*plain* is one case of a wider gap
rather than the whole of it.

*And this is atomicity, not ordering.* Ordering used to be the other half of
the gap and is now covered: `DMB`/`DSB`, RISC-V `FENCE` and
`MFENCE`/`LFENCE`/`SFENCE` each retire as a host `SeqCst` fence, `jit::x86`
compiles `Opcode::FENCE` to an `MFENCE`, and the bus lock an x86 `LOCK` prefix
takes fences at both ends — which is the one that matters most, because
`smp_mb()` on x86-64 is `lock addl $0,-4(%rsp)` rather than `MFENCE`, so a
Linux guest leans on the prefix and executes `MFENCE` exactly once in a boot.
None of that was harmless to omit merely because guest and host share an
architecture: a barrier is the guest asking for something stronger than its own
baseline, and an x86 host's store buffer performs exactly the store-then-load
reordering an x86 guest's `MFENCE` paid to remove. `tests/memory_model_costs.rs`
produces the outcome `MFENCE` forbids tens to hundreds of times in 200 000
rounds, and never once when about forty nanoseconds separate the store from the
load. What is *not* covered is `LDAR`/`STLR` on a64, which still issue an
ordinary load and store; `core::space::BusLock` carries the argument for the
shape of the fix.

**Both are reachable only under `ThreadingMode::Parallel`**, which is not the
default: `Deterministic` runs every guest on one host thread, where the finest
interleaving there is is one whole instruction and none of this can be observed.
That is what makes them documented boundaries rather than live defects, and it
is the sentence every SMP claim above should be read with. Separately, under
`--accel kvm` the host's own silicon does the atomic, which is why an
accelerated SMP boot is not evidence about this tree either way.

**The four boards above really do run their processors at once when asked to.**
`--threading parallel` is one flag on any of them, and a machine file can now
declare the mode itself — `threading parallel`, a claim about the hardware
rather than about the run, which `--threading` still overrides. Debian's arm64
kernel on `arm64-virt-smp` prints `smp: Brought up 1 node, 2 CPUs` in that mode
as readily as in the deterministic one, at `0.022746` against `0.022736`, and
that ten-microsecond difference is exactly the reproducibility being given up.
It is also *slower* on a two-processor board — 90 s of virtual time costs 187 s
of wall in `parallel` against 137 s in `deterministic` — because the rendezvous
costs a dispatch per runnable per round and two runnables do not repay it;
`ThreadingMode::Parallel`'s own table says where the crossover is.

No board in `machines/` declares the mode, and that is a decision rather than an
oversight: a state hash is refused outside a deterministic mode, so a shipped
board that declared `parallel` would take itself out of the regression suite for
everybody who builds it. `machines/tests/smp-parallel.machine` is the in-tree
board that does declare it, and `tests/parallel_smp_boards.rs` runs two RV64
harts through 20 000 atomic increments each with no update lost — beside a
*plain* counter that loses 7 700–10 800 of 40 000, which is what proves the two
harts genuinely collided rather than taking turns.
[`docs/techniques/parallel-execution.md`](docs/techniques/parallel-execution.md)
is the whole argument: what the mode promises, what a machine file may say, what
a run with no state hash is checked by instead, and a named list of what still
does not work.

`docs/platforms/arm64-virt.md` has the ledger, and it is long: PSCI
`CPU_SUSPEND` is refused rather than implemented (`CPU_ON`, `CPU_OFF` and
`AFFINITY_INFO` are real, and are how the second core comes up); no RTC, so
`date` starts at the epoch; `GICC_CTLR.EOImode` unimplemented;
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
system uses on both. What has actually run on it is **rsemu's own boot sector**,
not an operating system: it walks the MP configuration table for the application
processor's APIC id, sends the INIT/Start-Up pair the *MultiProcessor
Specification* §B.4 describes, and the processor that starts enters protected
mode and reads its own id back as `1`. That is the whole claim, and the SMP
caveat above applies to it as much as to the others.

**No third-party firmware is shipped and none will be** — but there is now one
of our own. `rsemu run pc-at --hd0 disk.img` boots with nothing supplied,
because `src/fw/pcbios` is a minimal legacy BIOS written here: POST, the BIOS
Data Area, option-ROM dispatch, `INT 10h`/`11h`/`12h`/`13h`/`15h`/`16h`/`19h`/
`1Ah`, and a bootstrap that reads the first sector and jumps to it. It exists
because FreeDOS, Windows 95 and Windows XP all need a *legacy* BIOS and every
one anybody could reach for is GPL. There is no assembler in this repository
and Rust cannot target 16-bit x86, so the ROM is **emitted**: `src/fw/asm16` is
a 16-bit x86 assembler in Rust and the firmware is a Rust program that calls
it, which makes `cargo build` the whole build. On that firmware `pc-at`
**installs FreeDOS 1.3 onto a hard disk and boots off it** — the board sizes
16 MiB of RAM, shadows itself out of ROM into RAM through the 82441FX's PAM
registers, enumerates PCI, maps and runs a video card's option ROM off an
expansion-ROM BAR, reads a diskette through the µPD765 and the 8237, and jumps
to `0000:7c00`, where FreeDOS's own boot sector takes over. From there a person
answers the installer: `FDISK` partitions a blank IDE drive, the machine
reboots off the diskette, `FORMAT` writes FAT16, and the installer unpacks 114
archive volumes off five diskettes that are swapped under the running guest.
The diskette then comes out, and the disk image the install wrote boots on its
own to `C:\>`, where `VER` answers `FreeDOS 1.3`. Every keystroke goes in
through the VNC input seam as set-2 scan codes on the 8042's port, and every
answer is read off the guest's own text page; nothing is vendored, and the
diskettes are fetched. What that took is written down, including the three
`INT 13h` defects it found — the boot order, the EDD subset and the diskette's
change line (`docs/platforms/pc-at.md`).

The boot off that disk loads **JemmEx**, the memory manager `FDCONFIG.SYS`
names, and so it runs in **virtual-8086 mode under paging**: five drivers load
high and `MEM` reports 173 KiB of upper memory free. The core did not have the
mode, and getting there also turned up a privilege-level defect that had been
sitting under this board since it existed — the window between `MOV CR0`
setting `PE` and the far jump after it is privilege 0 whatever the `CS`
selector's low two bits say, and this core read them.

**That board has a screen now, and not only a text page.** `pc.video` took a
`model` property: `6845` is the text-mode CRTC it always was — bit for bit, and
the two models' text pages are asserted to hash the same — and `vga` is the
adapter proper. Four 64 KiB planes behind the `A0000` window, the sequencer's
map mask, the graphics controller's four write modes and two read modes,
odd/even and chain 4; and a scanout that follows the CRT controller's address
sequence rather than a mode number, so mode 12h is 640×480 because the registers
say so and an unchained 320×240 comes out 640×480 for the same reason. The
firmware programs them — modes 0Dh, 0Eh, 10h, 12h, 13h and 03h out of IBM's own
register tables — and **VBE 2.0** sits on top: nine linear modes from 640×480 to
1024×768 at 8, 16 and 32 bits a pixel, whose `PhysBasePtr` the BIOS reads out of
the display adapter's **BAR0** through its own PCI interface. The extension
registers the linear mode hides behind are *ours*, specified in
[`docs/devices/pc-video.md`](docs/devices/pc-video.md), and deliberately not
Bochs' DISPI ports, whose only specification is a GPL program's source. `4F04h`,
`4F06h`, `4F07h` and `4F08h` answer `AH=01h` rather than pretending.

**And a disc drive.** `ata.cdrom` is a **packet device** on the same cable as
the hard disk — `PACKET` with the byte-count limit and the C/D + I/O handshake
of ATA/ATAPI-6 §9.10, the `0xEB14` signature a driver detects it by, and the
SFF-8020i command subset a boot and an install need. It is a sibling of the disk
rather than a flag on it, because a CD-ROM is a different command set and not a
variant of `READ SECTOR(S)`; what the two share is the *cable*, which is now a
trait rather than a type. The **disc itself has no bus in it**, so one model
hangs off an IDE cable and off an **AHCI** port without either knowing about the
other. `src/fw/pcbios` boots from one: **El Torito 1.0** — the boot record
volume descriptor at logical block 17, the catalog's validation entry and its
`55h AAh` key, media type 0 (no emulation) and types 1, 2 and 3 (diskette
emulation, where the image becomes `INT 13h` drive 00h and the board's own
diskette moves to 01h). Type 4 is **declined** rather than half-implemented,
because loading a hard-disk image with no drive 80h behind it is worse than
falling through to `INT 18h`; and the disc goes *last* in `INT 19h`'s order, for
the same reason the rest of that order was fixed — an installer that writes a
boot record and reboots would otherwise run itself again for ever.

**Media can be changed while the machine runs**, on every board rather than on
one. `dev::medium::Removable` is the door — which bays a device has, what is in
each, whether it is write protected, and put-this-in / take-that-out — published
through the export seam, so a host asks the *machine* rather than knowing five
device types, and `rsemu monitor` reaches every drive on the board through it.
Five devices implement it and **each raises the change signal its own part
raises**, because a swap the guest does not see corrupts the filesystem it has
mounted: `DSKCHG` in the PC diskette adapter's digital input register, `CHNG*`
on the Amiga drive connector at CIA-A `PRA` bit 2, a pending `28h 00h` UNIT
ATTENTION on `ata.cdrom` going out as well as in. Two of the five have no signal
to raise and say so where the door is rather than burying it: an SD card's
write-protect notch is a switch on the socket and not something a card can
report, and **`amiga.cd` reaches the guest not at all** — Akiko carries no
disc-change message, none was invented, so a CD32's tray opens and closes
without the machine being told.

Two timing defects underneath all of that turned out to be the scheduler's
rather than the guest's. `RDTSC` **stood still through `HLT`**, because a halted
core consumes its whole budget and was charged none of it — where *Intel SDM*
vol. 3B §17.17.1 gives an invariant counter one that runs "at a constant rate in
all ACPI P-, C-. and T-states", and `HLT` is C1. The remaining allowance is now
charged in one addition rather than by spinning a halted processor, and
`CPUID.80000007H:EDX[8]` reports the invariant bit because the core keeps it.
Then with two processors on `q35-linux-smp` the kernel measured them against
each other, found **22 cycles of TSC warp** and turned the clocksource off: a
processor is charged whole instructions, so the last one of a round runs past
its grant, and two runnables on one crystal each execute a round from their own
position. A counter and a timer comparator are now read and armed where the
processor **stands**, not where its round ended.

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

`q35-linux-smp` is that board with **two processors** — the same five lines
`pc-at-smp` adds to `pc-at`, plus a MADT that is told there are two — and it is
the one SMP board here on which a real kernel does real SMP work: the same stock
Gentoo kernel prints `smp: Brought up 1 node, 2 CPUs` **1.7 seconds** into a
`--accel kvm` run and `nproc` says `2` at a shell **2.8 seconds** in, on the
board's own command line. Read that number with the SMP caveat above — under
KVM the atomics are the host's silicon, so that boot says nothing about
rsemu's. **The interpreted boot does**, and it now happens: the same board with
no accelerator prints the same line, reaches a shell, and answers `nproc` with
`2` and `/proc/interrupts` with a column of live counts per processor. It took
one flag bit to get there — `EFLAGS.ID`, which has no job other than letting
software discover `CPUID`, and which only ever mattered to the processor that
boots second, because a Start-Up leaves that one in real mode and the code
waiting for it there asks.
`docs/platforms/q35-linux.md` has the hunt.

`q35-uefi` is the same chipset with the ROM socket replaced by **two banks of
parallel NOR flash** below 4 GiB, which is the layout every split OVMF build is
compiled for. A real OVMF runs SEC out of flash, sizes memory in PEI from the
CMOS, decompresses `FVMAIN`, dispatches DXE, and **reaches an interactive
`UEFI Interactive Shell v2.2` — a `Shell>` prompt that executes what is typed at
it**, in 367.2 seconds of guest time. Three x86-core defects were between it and
that prompt and all three are fixed: `MOV RAX, CR8` raised `#UD`, a long-mode
`FXSAVE` frame was mis-aligned, and `RDMSR(IA32_PLATFORM_ID)` raised `#GP`.
**Variables survive a reboot** — `setvar` in one run reads back in the next, and
one boot to the shell leaves **5,799 programmed bytes** in the variable bank
where the shipped `OVMF_VARS.fd` had 127.

And **an operating system now follows it**. The
board grew an **NVM Express controller at `00:04.0`** — the same part
`q35-linux` uses, chosen because EDK II's `NvmExpressDxe` binds on a class code
and polls its completion queues, so a namespace is reachable with nothing on
this board wired for it. OVMF enumerates it, `FS0:` maps a FAT volume on it,
`startup.nsh` is read off that volume and executed, and BDS starts
`\EFI\BOOT\BOOTX64.EFI` from it rather than from the firmware volume. Put a
stock Gentoo 6.6.67 `bzImage` on that volume — a modern one *is* a PE/COFF EFI
application — and the **firmware, not a loader**, reads it and its initramfs off
that namespace, `LoadImage`/`StartImage`s it, hands it a memory map and a system
table, and the kernel comes up on COM1, runs `/init` and answers `uname -srm`.

It also **describes itself** now, through **`fw_cfg`** at `0x510`. Under UEFI
nothing scans for an RSDP — the kernel takes it from the EFI configuration
table, which holds whatever the *firmware* installed, and an OVMF build has
exactly one source for that. So `q35.fwcfg` hands over the same tables
`src/dev/q35/acpi.rs` generates from the realized machine, packaged the way
`QemuFwCfgAcpi.c` expects: a blob whose pointer fields are offsets, an RSDP, and
a 128-byte-command loader script that allocates, relocates and checksums. Eight
tables get installed, and the kernel that used to print `ACPI MADT or MP tables
are not detected` and fall back to virtual wire mode now prints `ACPI: Using
ACPI (MADT) for SMP configuration information` and `APIC: Switch to symmetric
I/O mode setup`. Wiring the HPET's `LEG_RT_CNF` multiplexer was the other half
of that, and the board panicked `IO-APIC + timer doesn't work!` in between.

Every byte of all of it is read off the **16550 at `0x3f8`**, which is this
board's only console: it has no video adapter, because EDK II's `QemuVideoDxe`
binds three PCI identifications and none of them is ours, and no `0x402` debug
port. The whole boot is 2,156,716 ms of guest time — minutes of host time — and
`docs/platforms/q35-uefi.md` is the ledger of what is still in the way.

`stm32f407` is a microcontroller rather than a computer: an **STM32F407VGT6**,
the part on ST's own STM32F4 Discovery board — a Cortex-M4F out of flash aliased
at zero, six GPIO ports as instances of one class, USART2 on your terminal, and
the peripherals a startup file actually talks to: the clock and power
controllers, fourteen timers, two DMA controllers, the EXTI pin-interrupt mux
and both watchdogs. It is where an M-profile core answers the question the other
boards never ask, because a Cortex-M's interrupt controller is *inside the
core*: a peripheral drives `cpu.irq38` directly, and 38 is USART2's row in the
part's vector table, written in the machine file where the part is chosen rather
than in any device model. Fifty-odd such rows are written there now, each with
its name from the manual beside it, and a test makes the handlers say which
number they were reached through — because a vector wired to the wrong core pin
does not fail, it quietly runs somebody else's code.

Its input device is a **4x4 matrix keypad** on PE0-PE7, and it is there because
it is the one thing on this board that an ordinary wire could not carry. A
matrix keypad has no register block: sixteen switches, eight pins, and firmware
that drives one row low at a time and reads the columns back through the
pull-ups. That needs a net where *nobody driving* is a state distinct from
*somebody driving low*, so a driver presents a tri-state `Drive` rather than a
level, `OTYPER` and `PUPDR` decide what a pad actually presents, `IDR` reads
the **pin** rather than the port's intention, and the machine file says
`wire gpioe.p0 -> keypad.row0 { pull = "up" }` to put the resistor on the
copper where it belongs. Before that, a row configured input-with-pull-up read
low with nothing attached and firmware saw every key held. The pad has no series
diodes, which is the truth about a cheap membrane part: press three corners of a
rectangle and a test watches the fourth ghost, through the wire model rather
than through a special case.

The two DMA controllers master a **second address space**, which is how the
board says the thing a comment cannot: an F4's core-coupled memory is on the
Cortex-M4's own bus and no DMA reaches it, so `dmabus` has SRAM and the
peripherals in it and no CCM, and a stream pointed at `0x10000000` raises a
transfer error exactly as the silicon does.

`amiga-a500` is the one that boots a **desktop**. A
68000, the full custom chipset — Agnus with the copper, the blitter and the
beam, Denise, Paula, two 8520s, Gary's address decode — and a 3.5-inch drive on
the CIA ports. Point it at a Kickstart and it draws what an A500 with an empty
drive draws: **1.3's hand holding a disk, 2.04's and 3.1's check mark with the
disk sliding into the drive**. Put a Workbench disk in DF0 and **1.3 and 2.04
boot to their desktops** — title bar, icons, pointer — off the real ADF, read
in place. Then **a person can use it**: through the same input seam a VNC
client drives, a test double-clicks the disk icon, opens a Shell and types
`echo hello` into it, on both releases. The four audio channels reach a host
`.wav` in stereo, 0 and 3 left and 1 and 2 right, the way the machine is wired.
**AROS boots its own boot disk to the Workbook desktop** as well, given the
extended-ROM socket the board offers and more memory than Commodore ever sold
in one — the machine file says so where it offers it. `amiga-a500plus` is the
same board with the **Enhanced Chip Set** — an ECS Agnus with `BEAMCON0` and a
programmable beam, an 8373 Denise with SuperHires — 1 MiB of chip RAM and the
battery-backed clock; Kickstart 2.04 and 3.1 find the ECS chips there, and
Workbench 2.04 boots on it. `amiga-a600` puts **Gayle** and an IDE hard disk in
Gary's place, and `amiga-a1200` puts the **AA chip set** — Alice and Lisa,
eight bitplanes, a 256-entry 24-bit colour table, HAM8 — around a 68EC020 with
2 MiB of chip RAM: **Kickstart 3.1 boots Workbench 3.1 on it off the hard disk
and off a floppy**, and `graphics.library` reads back the AA bits in
`GfxBase->ChipRevBits0` where the A600 beside it reads only the ECS pair.
`amiga-cd32` is the **CD32**: the same AA chip set and 68EC020 with no floppy,
no IDE port and no keyboard, a two-part 3.1 ROM, **Akiko** at `$B80000` — its
chunky-to-planar corner turn, the CD-ROM controller's registers and the two
wires of the machine's EEPROM — a CD-ROM drive on the `cd0` slot and the
eleven-button joypad on a controller port. With nothing in the tray it runs the
animated boot screen, which is what a real one does. `amiga-a3000` is the
**68030** machine: a 25 MHz 68030 with its 68882, ECS, 2 MiB of chip RAM and
4 MiB of 32-bit motherboard fast RAM in a 32-bit address space, **Ramsey** — the
memory controller whose one readable register Kickstart spins on, and whose
window the fast RAM must be **top-aligned in**, because the *A3000+ System
Specification* §2.2 builds it down from `$07FFFFFF` and the ROM's sizing
routine walks down from `$07F7FFF0`, so a half-populated board filled from the
*bottom* of that window is a board whose ROM concludes it has no fast RAM at
all — and **SCSI** where the A600 has IDE: a `scsi.disk` target on a named bus,
a Western Digital **WD33C93A** initiator, and Commodore's **Super DMAC**, which
masters memory and which the SCSI chip's two registers are mapped inside.
**Kickstart 3.1 boots Workbench 3.1 off the SCSI disk to its desktop, and 2.04
boots Workbench 2.1**, with the guest itself reporting an ECS chip set, a 68030
with a 68882, an `ExecBase` relocated into the fast RAM and a `DH0` handler
task for the partition it found in the Rigid Disk Block; with the bus empty
both reach the insert-disk screen instead. What that boot cost is the ledger's
most useful column: **eight defects**, three of them in Ramsey's register and
the Super DMAC's decode — the chip's two registers are *byte lanes* and not
longwords — and five between the WD33C93A and Commodore's `scsi.device`, the
last of which froze the machine 109 successful SCSI commands in, with `DH0`
already mounted, because an interrupt queued behind a `Select` was delivered
only when the host read a register that only an interrupt makes a host read.
`amiga-a4000` is the **68040** machine: the AA chip set on the A3000's 32-bit
board, a 25 MHz 68040 with its on-chip FPU, 2 MiB of chip RAM, **16 MiB of
motherboard fast RAM** behind Ramsey — `exec` relocates `ExecBase` into it —
and the A4000's own **IDE port**, which is not Gayle's and is at an address
Commodore's published documentation does not print, so it was found by
recording what the ROM touches on a board that answers nowhere else.
**Kickstart 3.1 boots Workbench 3.1 on it off the hard disk**, with the guest
reporting AA chips and a 68040. `amiga-a4000t` is that board in a **tower**,
with an **NCR 53C710 SCSI I/O Processor** on the motherboard beside the IDE
port — a chip that is a *processor*, fetching its own SCRIPTS instructions out
of guest memory and mastering the bus for them, so the board needs no DMA
controller between it and the memory at all. It is what
`amiga-os-310-a4000t.rom` has been asking for: on an `amiga-a4000` that ROM
finds the IDE drive and then stops, and here it **boots Workbench 3.1 off a
SCSI disk** — and off the IDE port too, because a real A4000T has both. Where
the chip is, which way round its byte lanes are, which byte of `DSP` starts it
and how a selection spells an address were all settled by recording what the
ROM touches; `docs/platforms/amiga.md`, "A4000T", has the traces.

`mac-plus` is the newest, and the first Apple machine here: a **Macintosh
Plus**, a 68000 at 7.8336 MHz with the `ROMOVERLAY` overlay that puts the ROM
under the vector table until the VIA's `PA4` drops it, a 6522 and an IWM on the
same A9-A12 decode, a Z8530 in two windows because the board decodes the
read/write direction from the address, and a 512 × 342 one-bit screen with no
video chip behind it at all — a counter walks main memory in step with the
beam, and the screen buffer hangs a fixed distance below the top of *installed*
memory, which is why the ROM has to size memory before it can draw anything.
Point it at a real Plus ROM and it chimes, sizes its megabyte, runs its memory
test, finds its clock chip's battery flat and writes twenty bytes of parameter
RAM back, finishes the keyboard's Model Number handshake and settles into
asking it for a key every quarter second, draws the **grey Macintosh desktop
with the arrow cursor**, and puts the **blinking insert-disk icon** in the
middle of it, with the 60.15 Hz tick chain running and nothing faulting — and
from there it **polls the drive** six to eight times a second, which is the
loop that notices a disk. Put an 800K image in and it starts the motor, checks
the spindle against the drive's tachometer, **reads cylinder 0 and decodes the
two boot blocks off it** — Apple's own code finding them whole in memory is the
independent proof that this encoder's 6-and-2 GCR is right — finds no system on
them and puts the disk back out. `--record-audio` gets the **startup chime**,
which is a pulse-width byte a scan line out of a buffer in main memory and not a
sound chip at all, and `--vnc` gets a **mouse** whose pointer goes where it is
put: two quadrature pulse trains an axis, X1 and Y1 on the SCC's carrier
detects and X2 and Y2 on the VIA's port B. What it does **not** do is boot,
and the only thing missing is Apple system software on an 800K image — ledger
item 1. `docs/platforms/mac-plus.md` has the ledger and the black-box traces
that got it this far: the one that found the ROM drawing that icon through a
pointer near the top of the four-megabyte window, which is how it was settled
that main memory *repeats*; the clock chip's whole undocumented command
encoding read off the wire; the spindle speed measured by sweeping it until the
ROM accepted a disk; the word at the level-3 autovector that proves the board
priority-encodes its two interrupts, which the first mouse to move found by
livelocking the machine; and why a Plus cannot read a 1.44 MB disk whatever
else is built.

`mac-classic` is the **Macintosh Classic** (1990), and it is here for one
reason: a Plus has an IWM and an 800K drive, and the system software people
actually have is on 1.44 MB disks. A Classic reads those, so it is the shortest
path to a Macintosh that boots — and once one boots, its own Finder can author
an 800K disk for the Plus, which is the only thing that writes HFS resource
forks correctly. It is the same compact-Macintosh shape with three differences
that matter: a **512 KiB ROM** whose own checksum only covers the first half of
it, the **Apple Desktop Bus** where a Plus has its serial keyboard, and a
**SWIM** where a Plus has an IWM. Apple's own Classic ROM runs it to the
**insert-disk screen** — the grey desktop, the arrow cursor, the blinking floppy
with a question mark — having sized memory, initialised the SCC, written its
parameter RAM back, reset the Apple Desktop Bus and walked all sixteen bus
addresses asking each for register 3 (finding the keyboard at 2 and the mouse at
3, which is where a Macintosh leaves them), probed the drive, and seen that the
mechanism is a **SuperDrive**. A 1.44 MB image goes into that drive, becomes IBM
MFM cells — `A1A1A1` sync with the missing clock derived from the encoding rule
rather than quoted, ID and data address marks, CRC-16/CCITT — and turns at 300
rpm under a head the ROM can step, with all 2,880 blocks making the round trip
through the encoder. What is left is **one chip**: the ROM asks the SWIM for
**ISM mode**, and no document available to this project states that register
file, so there is none here rather than an invented one. Two real defects fell
out on the way and both are fixed: the ROM re-asserts the overlay 5.4 seconds
into startup, which on a board that reads the pin as the decode puts the ROM
back over the machine's own vector table mid-instruction, and it makes one word
access to the VIA, which has to *complete* on a machine that has no bus-error
timeout at all. `docs/platforms/mac-classic.md` has every trace, the exact
sequence the ROM writes asking for ISM mode, and the instruments that found it.

**Not one byte of any of that is in this repository, and none ever will be.**
Kickstart is Cloanto's, Workbench is Commodore's and the Macintosh ROM is
Apple's; the tests read the user's
own **Amiga Forever** files in place, decoding its keyed `AMIROMTYPE1` images
with the `rom.key` beside them, and **skip with a printed reason** when the
environment variable naming that directory is unset — so `cargo test` passes for
somebody who owns none of it. What the goldens assert is *our rendering*: a hash
of the picture Denise produced, at a fixed virtual instant, plus that nothing
faulted. No Amiga emulator's source was opened for any of it — not UAE, not
WinUAE, not vAmiga, not AROS's own tree — and
[`docs/platforms/amiga.md`](docs/platforms/amiga.md) cites the *Amiga Hardware
Reference Manual* chapter by chapter for what it does instead, including the
handful of behaviours inferred from what the firmware itself demanded.

Beside them are the seventeen synthetic boards, each the smallest
machine that exercises one thing: `spi-panel` (a display path over SPI, with a
panel whose pixels are in guest RAM), `oled-spi` and `tft-spi` (an SSD1306 and
an ST7789, the first two devices here that hold their *own* framebuffer and are
filled by commands on a bus rather than by guest stores),
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

**An I/O base address register places a window now**, and why it could not
before is the reason the fix is general rather than a special case. A
configuration write on a port-mechanism board is an `OUT` to `0xcfc`: it travels
through the I/O space, and an I/O BAR moves a window *in that same space* — so
placing it means taking `TOPOLOGY` while `TOPOLOGY` is held for reading, which
`core::sync`'s ladder refuses, and no retry escapes it because the retry comes
down the same route. A q35's ECAM reaches the identical dead end from the other
side with a *memory* BAR, and that already had an answer: the function reports
the retopology it owes, and a device with a clock domain drains the flag from
`Device::advance_to` — the one moment with no access in flight. So what decides
whether a placement is deferred is never the kind of register, only whether the
access and the window are in the same space, and a guest sees the latch move at
once with the old mapping standing until the drain. The first thing that needed
it is a **PCI IDE controller in native mode**, which declares a four-byte I/O
window per channel and decodes exactly one byte of it, at offset 2.

The framework underneath is complete: address spaces with priority and
mirroring, an oscillator forest with exact intra-tree ratios, wires, devices,
snapshots, a typed export seam so one device can hand another a handle, and a
`.machine` description language that goes parse → resolve → validate → realize
→ run. There is a **gdb stub** (`rsemu debug apple1 --gdb :1234`) — driven end to
end by a **real `gdb` binary** in `tests/gdb_real_client.rs`, on an x86 guest
*and* an AArch64 one, which attaches, reads registers, writes a program into
guest RAM, sets a breakpoint, hits it and steps. That same `gdb` **`load`s an
image into flash**: the memory map declares a programmable array's real erase
geometry and `vFlashErase`/`vFlashWrite`/`vFlashDone` program it, through the
loader's door the device opens — a ROM has none, so `load` into one still fails,
and `core::space` still refuses a write to a read-only mapping. An **SMP board's
processors are threads**: `pc-at-smp` answers `qfThreadInfo` with two, each with its own
register file, address space and watchpoints. And a **debugger's write into
guest code invalidates the compiled blocks over it**, so a patch you set through
gdb is the code that runs — on x86 and RISC-V; `cpu.arm.a64` is the documented
exception and `docs/system/debug-protocols.md` says so.

Beside it there is a **monitor console** — `rsemu monitor apple1` — which
answers the questions a debugger has no packet for: the device tree and one
device's whole current state, every clock domain's exact rate and tick count,
the wire graph and what each net settles at, the scheduler's event queue, the
`--trace` counters *live*, `rewind`, and — through the removable-media seam
above — `media`, `insert` and `eject` over every drive on the board, taking the
same file specifications `--media` does. It advances the machine only through
the same `Machine::run_until` a headless run uses, which is additive — so a
session that types `run 2s` lands on the same state hash as
`rsemu run apple1 --for 2s --headless`, and `tests/cli_monitor.rs` asserts it.
Every read it makes sets `MemAttrs::debug`, so fifteen commands' worth of
inspection leave that hash where they found it — the other half of the same
test. [`docs/system/monitor.md`](docs/system/monitor.md).

There is also a **browser build** at <https://karpeleslab.github.io/rsemu/>.

That page is not a screenshot. Seven machines are in it — nine catalog entries,
because the NES and the Master System each ship an NTSC and a PAL file — and
four of them boot on an image the 3.10 MB module carries, so there is something
to press before there is anything to open: rsemu's own monitors, the
public-domain Woz Monitor of 1976,
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
native window: ALSA is an `ioctl` protocol and the alternative to `libc` is an
eighth `unsafe` subsystem, which the ceiling of seven forbids.

**A run can say what it did**, in numbers rather than prose. `rsemu run pc64
--media kernel=bzImage --for 200s --headless --trace all=boot.trace` writes a
plain two-column table — `cpu.retired.permille 987`, `sched.quanta 24818`,
`mmio.pia.read 1461001` — that `awk` reads with one pattern and `diff` compares
between two runs. **Counters, not an event stream**: every measurement this
project has improvised was a count, and a record per block entry would be
hundreds of millions of records the hot path cannot carry. Four channels —
`sched`, `cpu`, `clock`, and `mmio` per device aperture — and there is no
timestamp and no wall-clock figure anywhere in the output, deliberately, so two
traces of one deterministic run are byte-identical and a diff between them is a
real regression test. `tests/cli_trace.rs` asserts that, and asserts the
property the whole facility stands on: a traced run reaches the **same state
hash** as an untraced one. A flag that cannot be honoured is refused rather than
ignored — `--trace cpu` with `--accel` says why an accelerated processor keeps
none of those counters, while `--trace all` drops `cpu` from the wildcard and
says so on stderr, because `all` means "every channel this run can report".
Without the `trace` feature every hook has an empty body and the whole thing
costs zero; with it compiled in, the numbers — including the callgrind and
wall-clock pair that decided the MMIO hook was safe to put on a guest access
path at all — are in
[`docs/testing/tracing.md`](docs/testing/tracing.md).

### Three ways to execute a guest

**The interpreter is the oracle**, always: every other engine is differentially
tested against it, and `tests/riscv_virt_engines.rs` asserts identical state
hashes at ten checkpoints across all three plus a snapshot restored *across* an
engine switch.

The **translation IR** landed first — the architecture-neutral op set, typed SSA
blocks, the guest-instruction-boundary markers that make a mid-block fault
deliverable at the right PC with the right cycle count, a verifier, liveness and
dead-code elimination, and a portable interpreter backend that needs no `unsafe`
and runs on every target including bare metal. **Three architectures have
frontends now** — RISC-V, x86 and AArch64 — and each of those cores takes an
`engine` property with three values: `interp`, `jit` (the portable backend), and
`jit-host` (native code). RISC-V takes a fourth, `jit-wasm`, which lowers each
block to a WebAssembly module instead.

**AArch64 has a frontend** means an AArch64 *guest* is lowered to host code —
which is not the same claim as having an aarch64 code generator. There are
three backends: `jit::x86` (x86-64 Linux), `jit::arm64` (aarch64 Linux, written
on an x86-64 machine and therefore never yet executed) and `jit::wasm`
(`engine = "jit-wasm"`, a `WebAssembly.Module` per block). **Every number in the
table below was taken on x86-64 Linux under `jit-host`**, so it says nothing
about the other two. Each row below is the median of interleaved
runs of a real Linux boot on that board, and every run in a row finished on one
state hash:

| Engine | RISC-V (`riscv-virt`, 240 s of guest time) | x86 (`pc64`, 900 s) | AArch64 (`arm64-virt`, 20 s) |
| --- | --- | --- | --- |
| `interp` | 122.3 s | 276.5 s | 18.94 s |
| `jit` | 103.7 s (1.18×) | 184.3 s (1.50×) | 10.42 s (1.82×) |
| `jit-host` | **56.8 s (2.15×)** | **86.4 s (3.20×)** | **3.37 s (5.61×)** |

All three engines produce byte-identical guest output — 653 console lines on the
x86 run, ending 900,000 virtual milliseconds in at the same `CS:RIP`, `CR2`,
`CR3`, `CR4`, `EFER` and flags, having executed the same blocks from the same translations.

**A block now leaves on its tick allowance instead of being refused for a bound
it might have exceeded.** A block used to have to prove its *worst case* fitted
what remained of the scheduler quantum, so the last part of every quantum
admitted nothing — and two independent profiles found that guard, not the
unlifted encodings, was the dominant cost of interpretation. With
`IrHost::spent` asked at each instruction boundary, the guard's share is zero:

| | before | after |
|---|---|---|
| x86 retired in blocks | 97.3% | **99.3%** (12,248,632 interpreted, was 48,592,900) |
| AArch64 retired in blocks | 97.5% | **99.4%** (6,012,343 interpreted, was 26,356,169) |

What is left on x86 is the exclusion list plus 0.35 M interrupt shadows and
pins. On AArch64 it is 6.0 M outside the lifted subset, 1,614 first-sighting
lifts and 292 interrupts. 99.8% of compiled RISC-V stores still write guest RAM
inline rather than through a call (1,749,886 of 1,753,140). The RISC-V
headline *fell* from 2.28× to 2.15× along the way, because the interpreter it is
measured against got **1.27× faster** (155.1 s → 122.3 s) and the control moved;
the numbers and that argument are in
[`docs/platforms/riscv-virt.md`](docs/platforms/riscv-virt.md),
[`docs/platforms/pc64.md`](docs/platforms/pc64.md) and
[`docs/platforms/arm64-virt.md`](docs/platforms/arm64-virt.md).

**The wasm backend runs in a browser engine now, and it wins.** `engine =
"jit-wasm"` lowers each IR block to a WebAssembly module and executes it, on
every target, reaching the same state hash as the interpreter at every
checkpoint of `tests/riscv_virt_engines.rs`. On a native host the only thing
that can run one of those modules is a reference wasm interpreter, which makes
`jit-wasm` the *slowest* of the four engines there and is the point: the
translation is executed and hashed everywhere rather than only where a browser
is. On `wasm32-unknown-unknown` the page's own engine compiles them, through
three imports and four exports ([`web/src/jit.js`](web/src/jit.js)), and the
same guest for the same span comes out:

| Engine | native (reference executor) | in V8 (node 26) |
| --- | --- | --- |
| `interp` | 1064 ms, 1.00× | 922 ms, 1.00× |
| `jit` | 517 ms, 2.06× | 840 ms, 1.10× |
| `jit-host` | **85 ms, 12.50×** | — (there is no `mmap` in a browser) |
| `jit-wasm` | 1392 ms, 0.76× | **613 ms, 1.50×** |

`ROADMAP.md` §11.4 expected the backend to win "only on long-running
superblocks, and may not win at all"; it wins per *basic block*, with no
chaining and no superblocks. It is well short of `jit-host`'s 12.5×, and
`docs/techniques/wasm-jit.md` says which three mechanisms would move it and why
none was worth building before this number existed. Both columns come from one
function — `benches/wasm_jit_embedder.rs` and `web/check.mjs` §1c call it — so
they are one workload rather than two with one name, and the browser column is
asserted to hash identically to the interpreter's on every commit.

Two caveats on all of the above. `engine` is a `param` on the seven boards that
run third-party system software — `riscv-virt`, `arm64-virt`, `arm64-virt-smp`,
`pc64`, `q35-linux`, `q35-linux-smp` and `q35-uefi` — so
`rsemu run -p engine=jit-host` picks it from the command line there. It is
**still a literal** on `pc-at`, `pc-at-smp`, `pc-apic`, `q35` and `a64-mini`,
which are therefore still interpreted whatever you pass. And the table above is
**informative rather than gating**: [`docs/bench-host.md`](docs/bench-host.md)
now names the reference host, but these ratios were not taken under the
discipline the benchmark harness applies — interleaved sides, a minimum over
repetitions, a run-to-run spread beside every figure. One set of numbers in
this repository was, and it is the next paragraph.

**And rsemu has been measured against something other than itself.** Every
figure above compares rsemu with rsemu, which can say a change made things
faster and can never say whether the result is fast.
[`docs/testing/benchmarks.md`](docs/testing/benchmarks.md) is the external
comparison phase 8's gate asks for: the same kernel, the same initramfs and the
same five workloads under rsemu and under `qemu-system-…`, timed by markers the
*guest* prints, with QEMU run as a measuring instrument and never read (that is
a licence rule, not a preference — see below). The answer is **about twenty
times QEMU's wall clock on `arm64-virt` and about a hundred and ten on
`pc64`**, against a gate of 2×, and the interesting part is where the distance
lives: the code the JIT *generates* is **under a tenth** of a boot's host
instructions on all three cores — 7.32%, 8.02% and 8.22% where it has been
split out. So what the gate measures is the runtime *around* the generated
code: the address space, the deferred-charge replay, the block boundary, the
scheduler round. `ROADMAP.md` phase 8 is that list, in the order the profiles
put it.

**Hardware acceleration** is real, and it is KVM on Linux x86-64.
`rsemu run q35-linux --media kernel=bzImage --accel kvm` boots that same stock
Gentoo 6.6.67 kernel to a busybox shell in **2.4 seconds of wall clock**
against **978 s** interpreted — 2,826 seconds of guest time either way — **on
the board's own default command line**: the `no_timer_check` this paragraph
used to carry is gone, and so is the defect it was hiding. **282 of the
accelerated run's 346 console lines are byte-identical** to the interpreted
run's, in the same order, once the printk timestamp is removed; the 62 that
differ are the ones that describe the *host* processor, its mitigations, its
XSAVE list and its TLB geometry. On a pure-execution workload the ratio to
native is **99.7%, 99.9% and 101.2%** across three runs, against a phase-7 gate
of 80%.

What made the command line honest is `ThreadingMode::Accel`, which is now
implemented: a scheduler round's elapsed virtual time is **read off the host
clock** rather than counted out of the board's oscillators, and a periodic
per-thread timer bounds a guest that takes no exits at all. Before it, virtual
time did not advance while a vCPU was inside `KVM_RUN`, so a kernel calibrating
its time-stamp counter against a board timer concluded it was on a
**176,273 MHz** processor; it now reports **3,992.968 MHz** on a 3,993,994 kHz
host. That is also what unblocked `q35-linux-smp`, the two-processor version of
the same board, on which the kernel prints `smp: Brought up 1 node, 2 CPUs` and
`nproc` says `2`.

`--accel` is a **host** flag rather than a machine-file value, deliberately:
`engine = "interp"` and `engine = "jit"` are two implementations of the same
processor and their state hashes match, while a vCPU answers `CPUID` from the
host's silicon, cannot be replayed, and only exists on Linux/x86-64 — so a
board that named it would be a board that does not build on a Mac. The file is
used verbatim either way and what is accelerated is the *run*. It is not
reproducible: no state hash, no `--record-input`. HVF and WHPX are roadmap
entries with no code behind them.

**Level 3 — user-mode execution** — has its proof, on **two architectures**: a
static musl Rust binary, built by `scripts/fetch-testdata.sh` for
`riscv64gc-unknown-linux-musl` and `aarch64-unknown-linux-musl` and never
committed, runs through musl's own `_start` and `__libc_start_main` —
`AT_PHDR`, thread-local storage, a `brk` heap — reaches `main`, prints, and
**exits 0**, with no syscall refused. The same `hello` makes the same
twenty-five calls in the same order on both. The run is then replayed with the
entropy source replaced by a panicking guard, and produces identical output and
an identical tick count.

**Threaded `std` Rust guests run too**: `clone`, `futex` `WAIT`/`WAKE`,
`set_tid_address` and `CLONE_CHILD_CLEARTID` — which together are the whole of
`pthread_join` — carry four workers hammering one atomic and three threads on a
condition variable, written with no knowledge of the emulator. It is also what
found the tree's largest correctness defect: **the exclusive monitor was
core-local**, so an AArch64 `AtomicU32::fetch_add` loop landed 32,038 of 40,000
increments. It lands 40,000 now (see the SMP section above), and the same guest
is what proves it.

**And a whole C library runs, on both architectures.** The same `hello`, linked
against **glibc** instead of statically against musl and run under that
library's own `ld.so`: a `PT_INTERP` is read, the interpreter is mapped and
entered with no relocations applied, it opens and relocates the objects its
`DT_NEEDED` names, resolves the ifuncs glibc picks its `memcpy` and `strlen`
with, and transfers control — 59 syscalls on aarch64 and 62 on riscv64.
Threaded, it is 205 and 204, and it found something musl never asks for:
glibc's `pthread_create` tries **`clone3`** first and falls back to the
five-register `clone` on `-ENOSYS`.

**Software nobody here wrote runs on it too** — three programs picked for what
they ask of the ABI rather than for fame, all permissive, all cross-built
unmodified at a pinned version by `scripts/fetch-testdata.sh` and never
committed: **SQLite 3.45** (public domain), **Lua 5.4.7** (MIT) and **sbase**
(MIT). SQLite makes the same ninety-three calls in the same order on both
architectures and prints the same rows; sbase's `sha256sum`, `wc` and `cksum`
are diffed against the *host's* over the same bytes, which is what makes it
different in kind from a test that only checks a program did not crash. They
are what found four holes, every one of them in the consumer's half:
`mstatus.FS` left Off, and missing `readv`, `pread64` and `fcntl`.

**Fifty-two syscall numbers are dispatched** — `hello` makes 25 of them on
either architecture and the threaded musl guest 166 — and the hard rule is
unchanged: *a level-3 guest may be told about itself, and may not be told about
the host*. What that means has been sharpened rather than relaxed. There is
still no `--allow` flag and none planned, and the property is now **mechanical**
rather than argued: *nothing that services a syscall links `std`*, which a CI
feature-combination job builds on every commit. A guest reaches files through a
**stage** — a map from guest path to bytes the harness fixes before the guest
exists — so a program that is *told* a path runs and a program that
*discovers* paths does not: `ls` fails with `lstat /work: No such file or
directory`, because there is no `getdents64` and no notion of a directory, and
that is the answer rather than a gap. rsemu
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
scripts/check.sh qemu    # rsemu against QEMU — minutes to an hour, on purpose,
                         # and in neither of the two sets above
```

`scripts/check.sh` is the whole CI workflow as one command, so "I ran the
tests" and "CI is green" stop being different claims. It prints one marked
summary at the end and exits non-zero if any stage failed; `--list` names the
stages and any of them runs alone.

`fuzz/` is a separate crate — detached from the workspace, so `libfuzzer-sys`
never appears in `rsemu`'s dependency tree — and it carries **twenty-nine
targets**: the `.machine` parser, the snapshot container and the migrations that
read an old one, the qcow2/DfuSe/ADF image parsers, three lifter differentials
against the interpreters that are their oracles, and every MMIO surface that has
acquired one. None of it is a merge gate, deliberately: thirty seconds a target
finds nothing a real campaign would, so a daily job runs them for *drift* and
[`fuzz/README.md`](fuzz/README.md) has the command for the hour-long phase-2
gate. That job now reports **every** failing target rather than the first, which
is not a tidiness fix — it was `set -e` and died on `blk_image`, fourth of
twenty-nine alphabetically, with three further targets red behind it for weeks:
one of them a remotely reachable panic in the RFB server and one a reference
cycle leaking a whole address space.

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

The default `cargo tree` is exactly `rsemu`. An **`--all-features` build carries
eleven crates besides it, and seven of those are Karpelès Lab and MIT**:
[`pktkit`](https://github.com/KarpelesLab/pktkit-rs) (all networking),
[`fstool`](https://github.com/KarpelesLab/fstool) (block devices, qcow2,
partition tables, and read-write ext/FAT/exFAT/NTFS/XFS/HFS+),
[`compcol`](https://github.com/KarpelesLab/compcol) (image + snapshot
compression), [`purecrypto`](https://github.com/KarpelesLab/purecrypto),
`oxideav-png` (`--screenshot`), and `intl` and `charcode`, which
`fstool` reaches for. The other four are one chain — `fstool → uuid (v4) →
getrandom → libc, cfg-if` — so `libc` arrives through *randomness* rather than
through an `ioctl`, and making `uuid`'s `v4` optional upstream would take all
four out at once. That list was **33 crates two releases of `fstool` ago, then
23, and is now 11**; `Cargo.toml` records what each step dropped.

`purecrypto` is the newest of them, and it arrived for a reason the dependency
policy did not predict. It is named there for **disk and snapshot encryption**,
and for that it is still a seam and not a dependency: `src/core/state.rs`
carries the integrity and encryption fields in the container header and says, at
the point where the encrypted path would be, that this build does not implement
it. What actually links the crate is `dev-atecc` — a **Microchip ATECC508A/608**
crypto authentication chip on I²C or on a single wire — which needs SHA-256,
HMAC, AES-128 and P-256 to answer its own command set honestly. Every "random"
byte and every generated key on that part comes from the board's `seed`, so two
runs deal the same keys and hash the same.

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
