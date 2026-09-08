# rsemu documentation index

This directory is a **register of primary sources** — the specifications,
datasheets and hardware documentation rsemu is implemented *from*. It is an
annotated bibliography, not a copy of the documents.

## Why an index and not the documents themselves

Almost every specification here is copyrighted by its publisher and cannot be
redistributed from an MIT repository. Vendoring the Intel SDM or the PCIe base
spec would create exactly the licensing problem [`../CONTRIBUTING.md`](../CONTRIBUTING.md)
exists to prevent. So this directory records **what to read, where to get it,
what it authoritatively answers, and whether it is safe to quote** — and each
contributor fetches their own copies.

Downloaded documents belong in `docs/local/`, which is git-ignored. The same
rule as test corpora: fetch it, don't ship it.

## How to use this

1. Find the subsystem you're implementing below.
2. Read the **primary** source first — the datasheet or the ISA manual. It
   describes the hardware; everything else describes someone's understanding of
   it.
3. Cite what you used, in the commit message or a comment: document, revision,
   section. Provenance must be auditable years later.

## The provenance rule, in one paragraph

rsemu is MIT and **cannot absorb GPL'd code**. Do not read the QEMU source tree,
and do not read any other copyleft emulator (Bochs, DOSBox, MAME, VICE, Dolphin,
PCSX2, Nestopia, higan). Work from hardware documentation instead. Permissively
licensed code (MIT/BSD/Apache/ISC) is usable with its attribution retained.
Black-box use of a GPL program — running it, benchmarking it, diffing its trace
— creates no derivative work and is fine. Full policy:
[`../ROADMAP.md` §1](../ROADMAP.md).

**Facts versus expression:** a cycle count from a datasheet is a fact and is
free. The identical number copied from a GPL emulator's timing table is
expression obtained from a forbidden source. Take facts from primary sources and
the question never arises.

## Index

### CPU architectures
| | |
| --- | --- |
| [`cpu/6502.md`](cpu/6502.md) | MOS 6502, RP2A03, 65C02, 65816 |
| [`cpu/z80-sm83.md`](cpu/z80-sm83.md) | Zilog Z80, Sharp SM83 (Game Boy), Intel 8080 |
| [`cpu/x86.md`](cpu/x86.md) | 8086 → i386 → x86-64, SSE/AVX, paging, segmentation |
| [`cpu/arm.md`](cpu/arm.md) | ARMv5TE, ARMv7-A, ARMv7-M / Cortex-M, ARMv8-A / AArch64, GIC |
| [`cpu/riscv.md`](cpu/riscv.md) | RV32/RV64, privileged spec, SBI |
| [`cpu/mips.md`](cpu/mips.md) | MIPS I / R3000A, the LR33300, and why MIPS32 is the wrong CP0 manual |
| [`cpu/other.md`](cpu/other.md) | 68000, PowerPC, SuperH, V850 |

### Machines
| | |
| --- | --- |
| [`platforms/apple1.md`](platforms/apple1.md) | Apple 1: the MC6821, the clock, and the monitor-ROM licence question |
| [`platforms/nes.md`](platforms/nes.md) | NES / Famicom |
| [`platforms/game-boy.md`](platforms/game-boy.md) | Game Boy, Game Boy Color |
| [`platforms/master-system.md`](platforms/master-system.md) | Sega Master System, Game Gear |
| [`platforms/ibm-pc.md`](platforms/ibm-pc.md) | IBM PC/AT and modern PC chipsets |
| [`platforms/pc-at.md`](platforms/pc-at.md) | The `pc-at` machine rsemu builds, its firmware slots, and what is missing |
| [`platforms/q35.md`](platforms/q35.md) | The `q35` board: an 82Q35 (G)MCH, an ICH9, ECAM, and the generated ACPI tables |
| [`platforms/pc64.md`](platforms/pc64.md) | The `pc64` board: a long-mode PC with no firmware, and a Linux kernel entered directly |
| [`platforms/q35-linux.md`](platforms/q35-linux.md) | The `q35-linux` board: that chipset with no firmware, a kernel entered directly, and an NVMe disk |
| [`platforms/q35-uefi.md`](platforms/q35-uefi.md) | The `q35-uefi` board: that chipset with two NOR flash banks below 4 GiB, and what a real OVMF does on it |
| [`platforms/riscv-virt.md`](platforms/riscv-virt.md) | The RISC-V `virt` board |
| [`platforms/arm64-virt.md`](platforms/arm64-virt.md) | The AArch64 `virt` board: a GICv2, a PL011, PSCI, and a generated device tree |
| [`platforms/stm32f407.md`](platforms/stm32f407.md) | The `stm32f407` microcontroller board, its peripherals, and how an M-profile interrupt is wired |

#### Which board boots what, and how far

The pages above are one per board and each is a ledger of what is still in the
way; this table is the thing you cannot get by reading seven of them, which is
how they compare. **Every row is a measurement, not a gate**: no operating-system
boot here runs in CI, because each needs a kernel or a firmware image rsemu does
not ship (§1), and each is behind an environment variable that names one. The
hermetic tests beside them — which do run on every `cargo test` — drive the same
hardware with a guest this repository builds.

| Board | CPU | Boots | How far, and what is in the way |
| --- | --- | --- | --- |
| [`riscv-virt`](platforms/riscv-virt.md) | RV64GC | OpenSBI 1.6, Linux 6.12, EDK II | **furthest of any board here.** Linux to a shell that echoes typed input, on a generated DTB, with the console handed over to our own 16550A and a virtio disk mounted; EDK II to an interactive `Shell>` prompt out of two CFI NOR banks, with a variable written in one run read back in the next. `riscv-virt-smp` is the same board with a **second hart**, started through **SBI HSM** rather than a spin table, and the same kernel prints `smp: Brought up 1 node, 2 CPUs` and then runs userspace on both — `nproc` says 2, `/proc/interrupts` has a column per hart with IPIs going both ways, `/proc/stat` gives hart 1 more system time than hart 0. It needed the least of the three, because RISC-V gives every hart its own *address* for `msip`, `mtimecmp` and its PLIC context |
| [`arm64-virt`](platforms/arm64-virt.md) | AArch64 | Debian's arm64 kernel | a busybox shell, and `poweroff -f` typed at it stopping the machine through PSCI; with a root image it mounts **`/dev/vda` as an ext4 root** off virtio-mmio and runs the shell from it. `arm64-virt-smp` is the same board with a second core, on which the kernel prints `smp: Brought up 1 node, 2 CPUs` and `/proc/stat` shows CPU1 running tasks — started by PSCI **`CPU_ON`**, so it can be switched off again, which a spin table has no mechanism for (`secondary = "spin-table"` still selects the generated release table). Still open: `CPU_SUSPEND` refused rather than implemented, no RTC, no `EOImode`, no `AT S1E1R` |
| [`pc-at`](platforms/pc-at.md) | i386-class | FreeDOS 1.3, on firmware this repo assembles | `COMMAND.COM`'s banner and then the installer's own `Do you want to proceed [Y,N]?`, which is a live prompt: a scan code fed to the 8042 puts an `N` on the line. **But it cannot be driven past that first keystroke** — `pc.kbc` delivers one and then goes silent. `pc-at-smp` adds a second processor, but what runs on it is rsemu's own boot sector reading its APIC id back as `1` — **no operating system has run on two processors here** |
| [`q35`](platforms/q35.md) | x86-64 | a user-supplied PC firmware; rsemu's own BIOS | a firmware boot prompt, and a guest booted off IDE under our BIOS. **No operating system on the third-party path**; no SMP, no SMM, and S5 does not power off |
| [`pc64`](platforms/pc64.md) | x86-64 | a stock Linux `bzImage`, entered directly | a shell that echoes typed input, on an initramfs — no PCI, so no other root is possible. Needs `cryptomgr.notests` on the command line |
| [`q35-linux`](platforms/q35-linux.md) | x86-64 | the same, plus the chipset and a disk | a shell **on the board's own default command line**, reading bytes off an NVMe namespace through the kernel's own driver and a level-triggered interrupt. Its ledger is empty; **two of the four obstacles it named were refuted rather than fixed**, which is why the page is worth reading. `q35-linux-smp` is the same board with a second processor and one of the three here on which a real kernel does real SMP work — `smp: Brought up 1 node, 2 CPUs` at 1.7 s and `nproc` saying `2` at a shell at 2.8 s, under KVM (see the caveat below). Also the only board measured **under KVM**: the same boot in **2.4 seconds** of wall clock against **978** interpreted, 282 of 346 console lines byte-identical — and on the same command line either way, because the `no_timer_check` this row used to name is gone along with the defect it hid |
| [`q35-uefi`](platforms/q35-uefi.md) | x86-64 | a distribution's OVMF, out of NOR flash, and then **Linux** | an interactive **`UEFI Interactive Shell v2.2`** that runs what is typed at it, read off the 16550 at `0x3f8`; **variables that survive a reboot** (5,799 programmed bytes in the store where the shipped image had 127); and, since the board grew an **NVMe controller at `00:04.0`**, `FS0:` mapping a FAT volume on it, `startup.nsh` read off that volume and executed, and a Gentoo 6.6.67 `bzImage` entered through its **EFI stub** with an initramfs, reaching `Run /init as init process` and answering `uname -srm`. The board also describes itself now, through **`fw_cfg`**: eight ACPI tables the firmware relocates, checksums and installs, so the kernel logs `ACPI: Using ACPI (MADT) for SMP configuration information` and `APIC: Switch to symmetric I/O mode setup` where it used to fall back to virtual wire mode. Still **no video**, so serial is the whole console — but the board now has a **`0x402` debug console**, and EDK II's own `DEBUG()` log comes out of it: 1,648 lines from `SecCoreStartupWithStack` to the shell, captured headless with `--capture debug=boot.log`; 2,156,716 ms of guest time end to end, which is minutes of host time |

#### The caveat every SMP row is subject to

Five machine files declare two processors — `riscv-virt-smp`, `arm64-virt-smp`,
`q35-linux-smp`, `pc-at-smp` and the synthetic `pc-apic` — and **their guest
atomics are kept**. This was the tree's largest correctness defect until
recently, and it was closed in two pieces because the two families of
architecture break in different ways.

`core::space::monitor` is a **global exclusive monitor** on the address space,
hooked at the single funnel every guest store, DMA burst and ROM load passes
through, so an ordinary store by any observer breaks a covering reservation
without the storing master knowing the monitor exists. Each core keeps its
architectural reservation as before (`cpu::arm::a64`'s `State::exclusive`,
`cpu::riscv`'s `reservation`) and a store-conditional now needs both. Granules
are per architecture: 16 bytes on AArch64 (DDI 0487 B2.9, the smallest that
holds a 128-bit `LDXP`), 8 on RISC-V (the naturally aligned XLEN word the
reservation set must contain).

x86 needed a different primitive, and the reasoning is worth keeping: a
reservation is *optimistic* and architecturally licensed to fail spuriously,
which is what buys the monitor its lock-free table — but `LOCK CMPXCHG`/`XADD`/
`XCHG` are *unconditional*, with no status flag and no retry loop in the guest
to catch a failure, so building them on a spuriously-clearing monitor would
produce a wrong answer rather than a retry. Instead a **bus lock**
(`LockRank::BUS_LOCK`, immediately above `BUS` and below every bus fabric) is
held across the read and the write of one locked instruction. The two compose
without knowing about each other: a locked write still leaves through the store
funnel, so it breaks reservations on the way past.

**The evidence is numbers that used to come out wrong.** An AArch64
`AtomicU32::fetch_add` loop over two cores landed 32,038 of 40,000; it lands
40,000 now, and reverting the one check reproduces 32,038 exactly. Two x86
interpreters on two host threads running `lock xadd` land 40,000 of 40,000,
against 34,271 with the bus lock removed.

**Two things remain open and a green SMP row still does not cover them**, and
[`techniques/memory-models.md`](techniques/memory-models.md) is where both are
measured rather than asserted.

* **Single-copy atomicity is not kept.** `RamStore` is a `Vec<AtomicU8>` and
  every access to it is a byte loop, so a naturally aligned four-byte load
  racing a naturally aligned four-byte store can return a mixture of the old
  and the new word — a value all three architectures forbid (*Intel SDM* vol. 3
  §9.1.1, ARM DDI 0487 B2.2.1, RISC-V Unprivileged ISA §1.4).
  `tests/smp_single_copy_atomicity.rs` catches it 117–361 times in sixty
  thousand loads, and catches the read half of a `LOCK XADD` torn the same way
  with the bus lock held throughout. So locked-against-*plain* is a special
  case of a wider gap rather than the whole of it.
* **This is *atomicity*, not *ordering*.** Ordering was the other half and is
  now covered: `DMB`/`DSB`, RISC-V `FENCE` and `MFENCE`/`LFENCE`/`SFENCE` each
  retire as a host `SeqCst` fence, `jit::x86` compiles `Opcode::FENCE` to an
  `MFENCE`, and the bus lock behind an x86 `LOCK` prefix fences at both ends.
  Omitting them was never harmless just because guest and host share an
  architecture: an x86 host's store buffer performs exactly the store-then-load
  reordering an x86 guest's `MFENCE` paid to remove, and
  `tests/memory_model_costs.rs` produces the forbidden outcome tens to hundreds
  of times in 200 000 rounds. `LDAR`/`STLR` on a64 are the one barrier pair
  still issuing an ordinary access.

**Both are reachable only under `ThreadingMode::Parallel`**, which is opt-in
(`--threading parallel`), which **no machine file selects**, and which is not
the default — `Deterministic` runs every guest on one host thread, where the
finest interleaving there is is one whole instruction. That is what makes these
documented boundaries rather than live defects, and it is the sentence that
keeps every SMP row above honest. Separately, under `--accel kvm` the host's
silicon performs the atomic, so an accelerated SMP boot remains evidence about
the host rather than about this tree.

Boards with no page here — `pc-apic`, `spi-flash`, `spi-panel`, `arm926`,
`a64-mini`, `mips-mini`, `z80-mini`, `m68k-mini`, `ne2k-mini`, `nvme-mini`,
`ahci-mini`, `usb-mini`, `xhci-mini`, `xhci-pci-mini`, `hub-mini` — are
synthetic: the smallest machine that exercises one subsystem, described in
`machines/*.machine` and in the `Cargo.toml` comment on their feature. They boot
a guest this repository builds, and there is nothing about them a platform page
would answer that the machine file does not.

### Buses
| | |
| --- | --- |
| [`buses/pci.md`](buses/pci.md) | PCI, PCI Express, config space, MSI |
| [`buses/usb.md`](buses/usb.md) | USB 1.1/2.0/3.x, UHCI/EHCI/xHCI, device classes |
| [`buses/storage.md`](buses/storage.md) | ATA/ATAPI, AHCI, NVMe, SCSI, SD/MMC |
| [`buses/virtio.md`](buses/virtio.md) | virtio, virtqueues, PCI and MMIO transports |
| [`buses/low-speed.md`](buses/low-speed.md) | I²C/SMBus, SPI, 1-Wire, GPIO |

### Devices
| | |
| --- | --- |
| [`devices/interrupts-timers.md`](devices/interrupts-timers.md) | 8259, APIC/IOAPIC, PIT, HPET, RTC, PLIC/CLINT |
| [`devices/video-audio.md`](devices/video-audio.md) | VGA/VBE, framebuffers, sound hardware |
| [`devices/network-input.md`](devices/network-input.md) | NIC models, PS/2, HID |

### System software interfaces
| | |
| --- | --- |
| [`system/firmware-boot.md`](system/firmware-boot.md) | BIOS, UEFI, ACPI, SMBIOS, device tree, boot protocols |
| [`system/usermode-abi.md`](system/usermode-abi.md) | Level 3: ELF loading, the auxiliary vector, the syscall ABI, and the sandbox policy |
| [`system/debug-protocols.md`](system/debug-protocols.md) | GDB remote serial protocol, DWARF |
| [`system/remote-display.md`](system/remote-display.md) | RFB/VNC, SPICE |

### Techniques
| | |
| --- | --- |
| [`techniques/binary-translation.md`](techniques/binary-translation.md) | DBT, JIT, register allocation |
| [`techniques/memory-models.md`](techniques/memory-models.md) | x86-TSO, ARM/POWER relaxed models, barrier lowering |
| [`techniques/virtualization.md`](techniques/virtualization.md) | KVM, Hypervisor.framework, WHPX |
| [`techniques/webassembly.md`](techniques/webassembly.md) | Wasm core, threads, browser APIs |

### Testing
| | |
| --- | --- |
| [`testing/README.md`](testing/README.md) | How to run each suite, the bring-up order, and what each gate requires |
| [`testing/conformance-suites.md`](testing/conformance-suites.md) | Every suite, **with its licence** and whether it may be vendored |
| [`testing/cpu-interface.md`](testing/cpu-interface.md) | The four methods a 6502 core must expose for the harness to drive it |
| [`testing/accuracycoin.md`](testing/accuracycoin.md) | Reading AccuracyCoin's 141 results out of RAM with no screen and no controller |
| [`testing/long-run.md`](testing/long-run.md) | The interpreter against each translated engine, quantum by quantum, over a real guest — the gate two engine defects went through |

## Deliberately excluded

Listed so nobody adds them later thinking they were an oversight. Each is a
resource an emulator author would reach for by reflex, and each is off limits:

| Excluded | Licence | Note |
| --- | --- | --- |
| QEMU — source, headers, in-tree docs, commit messages, mailing list | GPLv2 | Permanently forbidden. Black-box benchmarking only |
| Unicorn Engine | GPLv2 | Derived from QEMU |
| Bochs, DOSBox, MAME, VICE, Dolphin, PCSX2, Nestopia, higan | GPL/LGPL | Copyleft |
| **Linux kernel source and in-tree `Documentation/`** | GPLv2 | See below — this one catches people |
| SeaBIOS, coreboot | GPL | Use EDK II (BSD-2-Clause-Patent) instead |

### The Linux kernel trap

Reading a Linux *driver* to learn how a device behaves is the most common way
this rule gets broken, because the driver is easy to find and the spec is not.
It is still GPLv2 and still forbidden. Read the specification the driver
implements — for virtio that is the OASIS standard, for NVMe the NVM Express
spec, for xHCI the Intel specification.

**On the KVM ABI.** The kernel's userspace ABI headers under `include/uapi/`
carry `GPL-2.0 WITH Linux-syscall-note`, and everyone relies on that exception.
But a project that elsewhere insists no paraphrasing launders a copyleft source
should not lean on one exception being obviously broad enough. **Use our own
doctrine instead: the KVM ABI is a *fact*.** Transcribe the ioctl numbers and
structure layouts you need from the documented API
([`techniques/virtualization.md`](techniques/virtualization.md)); do not copy
the header text. That needs no exception at all, and it is the same rule that
governs everything else here. Either way it covers the ABI only — never
drivers, `Documentation/`, or the rest of the tree.

## Link status

Every URL in this directory was checked at the time of writing. A few
publishers (Intel, Arm, PCI-SIG, UEFI Forum, DMTF, SD Association) block
automated requests, so those links are marked **[browser]** — they work
normally, just not from `curl`.
