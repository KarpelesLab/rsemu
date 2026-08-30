# IBM PC, PC/AT, and modern PC chipsets

Consumed by: `boards/i440fx`, `boards/q35` — the long phase.

## The legacy machine

| Source | Covers |
| --- | --- |
| [bitsavers.org](https://bitsavers.org/) | IBM PC, XT and AT **Technical Reference** manuals — including the original BIOS listings and full schematics. The primary source for the legacy PC |
| [minuszerodegrees.net](https://minuszerodegrees.net/) | Detailed hardware analysis, card documentation, and scans of the original machines **[browser]** (blocks `curl`) |
| [PCjs](https://www.pcjs.org/) | Hosts a large collection of scanned original documentation and manuals |
| ~~Bochs `PORTS.LST`~~ | **Withdrawn** — documentation inside a copyleft tree (`../../ROADMAP.md` §1). The IBM AT Technical Reference on bitsavers is the primary source for the port map |

## The modern machine

Emulated PC chipsets are modelled on real silicon, so the datasheets are the
specification:

- **i440FX** — Intel 440FX PCIset datasheet (host bridge) plus the **PIIX3/PIIX4**
  datasheet (PCI-to-ISA bridge, IDE, USB, power management).
- **Q35 / ICH9** — Intel 3 Series chipset and ICH9 datasheets.

Search Intel's document library or bitsavers by part number. These describe
config-space layout, BAR behaviour, and the legacy device wiring that firmware
expects to find.

## System-software interfaces

Boot firmware, ACPI, SMBIOS and the interfaces an OS actually probes are in
[`../system/firmware-boot.md`](../system/firmware-boot.md). The legacy device
set (8259, PIT, RTC, APIC) is in
[`../devices/interrupts-timers.md`](../devices/interrupts-timers.md).

## Implementation notes

- Guest OSes probe far more than they document. The practical development loop
  is: boot a real OS, watch what it touches, implement that, repeat — with the
  datasheet open, never another emulator's source.
- **A20 gating, the 8042 keyboard controller, and CMOS/RTC layout** are all
  load-bearing for DOS-era boots and are documented in the AT Technical
  Reference.
- Firmware choice matters: **EDK II / OVMF is BSD-2-Clause-Patent** and may be
  read and used. **SeaBIOS is GPL** — treat it as a binary you may run, never a
  source you may read.

## ⚠ Do not consult

Bochs, DOSBox and QEMU are all copyleft and all forbidden, including for "just
checking which port that is". `PORTS.LST` above answers most such questions and
is a fact table rather than an implementation.
