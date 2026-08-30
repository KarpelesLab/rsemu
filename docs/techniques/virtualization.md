# Hardware virtualization interfaces

Consumed by: `accel/`. Running guest code natively when guest ISA ==
host ISA.

## KVM (Linux)

| Source | Covers | Licence note |
| --- | --- | --- |
| [KVM API documentation](https://www.kernel.org/doc/html/latest/virt/kvm/api.html) | The complete ioctl interface: VM and vCPU creation, memory slots, the `kvm_run` shared structure, exit reasons, irqfd/ioeventfd, dirty logging | See below |
| Linux `include/uapi/linux/kvm.h` | The structure and constant definitions | `GPL-2.0 WITH Linux-syscall-note` |

**The licensing point that makes this possible:** Linux's userspace ABI headers
under `include/uapi/` carry an explicit exception —
`GPL-2.0 WITH Linux-syscall-note` — permitting their use in non-GPL programs.
That is what allows an MIT project to speak the KVM ABI. **The exception covers
the UAPI headers only.** Kernel drivers, `Documentation/` beyond the ABI
description, and the rest of the tree remain off limits.

KVM is reachable with raw `ioctl` syscalls alone, so the Linux accel backend
fits the no-foreign-code rule exactly.

## Hypervisor.framework (macOS)

| Source | Covers |
| --- | --- |
| [Apple Hypervisor framework documentation](https://developer.apple.com/documentation/hypervisor) | vCPU creation, memory mapping, VM exits, on both Apple silicon and Intel Macs |

**This breaks the purity rule**: it is a system framework reached through a C
ABI, not a syscall interface. It ships as an explicitly-labelled opt-in feature
(`ROADMAP.md` §10) rather than as a silent compromise.

## WHPX (Windows)

| Source | Covers |
| --- | --- |
| [Windows Hypervisor Platform API](https://learn.microsoft.com/en-us/virtualization/api/hypervisor-platform/hypervisor-platform) | Partition and vCPU management, memory mapping, exit handling |

Same caveat as Hypervisor.framework: a DLL import, therefore opt-in and labelled.

## Implementation notes

- An accel backend is a `Cpu` implementation like any other, so a machine file
  can mix an accelerated CPU with an interpreted co-processor.
- MMIO and PIO exits route **back into the address-space layer** — the same
  regions the interpreter and JIT use. This is the payoff for having one memory
  model.
- Time under accel is host-driven, which forfeits determinism. That is a
  documented property of the mode, not a bug (`ROADMAP.md` §4.2).
- Snapshots must remain compatible across an accel/JIT switch; that is an
  explicit gate on the acceleration phase.
