# Interrupt controllers, timers, and clocks

Consumed by: `dev/intc/*`, `dev/timer/*`, phases 6–7. In rsemu these are
ordinary devices with wire sinks and sources — the core knows nothing about
"interrupts" (`ROADMAP.md` §4.3).

## PC devices

| Device | Source |
| --- | --- |
| 8259A PIC | Intel 8259A datasheet ([bitsavers](https://bitsavers.org/)); [OSDev: 8259 PIC](https://wiki.osdev.org/8259_PIC) |
| Local APIC / IOAPIC | Intel SDM Volume 3 (the authoritative source), plus the 82093AA IOAPIC datasheet; [OSDev: APIC](https://wiki.osdev.org/APIC) |
| 8254 PIT | Intel 8253/8254 datasheet; [OSDev: PIT](https://wiki.osdev.org/Programmable_Interval_Timer) |
| MC146818 RTC / CMOS | Motorola MC146818 datasheet; [OSDev: CMOS](https://wiki.osdev.org/CMOS) |
| HPET | Intel *IA-PC HPET Specification*; [OSDev: HPET](https://wiki.osdev.org/HPET) |

## Non-PC

| Device | Source |
| --- | --- |
| RISC-V CLINT | Privileged spec — `mtime` / `mtimecmp` |
| RISC-V PLIC | [RISC-V PLIC specification](https://github.com/riscv/riscv-plic-spec) |
| ARM GIC v2/v3/v4 | Arm IHI 0069 **[browser]** |
| NES / Game Boy interrupt lines | The platform documentation — these machines have wires, not controllers |

## Implementation notes

- **Level versus edge is a device property, not a flag on the wire.** Model the
  edge detector as a device so that it snapshots correctly.
- Timers are the most common source of guest-visible timing bugs. Every timer
  registers events on its clock domain; none of them reads the host clock. A
  timer that "catches up" by sampling wall time will break record/replay
  silently.
- The 8259's cascade wiring, the IOAPIC's redirection table, and the APIC's
  priority/EOI handling are each small state machines worth unit-testing
  independently of any guest.
- These devices are also where `MemAttrs::debug` earns its keep — a monitor
  reading an interrupt controller's status register must not acknowledge an
  interrupt.
