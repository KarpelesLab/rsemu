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
| [`platforms/stm32f407.md`](platforms/stm32f407.md) | The `stm32f407` microcontroller board, its peripherals, and how an M-profile interrupt is wired |

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
