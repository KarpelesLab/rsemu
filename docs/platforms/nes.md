# NES / Famicom

Consumed by: `boards/nes`, phase 4 — the first real machine, and therefore the
phase that proves the framework.

## Primary

The [**NESdev wiki**](https://www.nesdev.org/wiki/Nesdev_Wiki) is the reference.
It is community reverse-engineering documentation of hardware behaviour, built
from decades of measurement against real consoles, and it is unusually rigorous
about distinguishing measured fact from inference.

| Page | Covers |
| --- | --- |
| [CPU](https://www.nesdev.org/wiki/CPU) | RP2A03 specifics, memory map, DMA stalls |
| [PPU](https://www.nesdev.org/wiki/PPU) | Picture unit: registers, scrolling, sprite evaluation, the exact per-cycle pipeline |
| [APU](https://www.nesdev.org/wiki/APU) | Audio: pulse, triangle, noise, DMC channels; frame counter |
| [Mapper](https://www.nesdev.org/wiki/Mapper) | Cartridge mappers — bank switching, IRQ generation, extra RAM |

Check the wiki's own licence before copying text verbatim; the facts it records
are free regardless.

## What makes the NES a good first target

- Small enough to finish (a 6502, a PPU, an APU, and a mapper).
- **Timing-brutal**: the PPU and CPU run at a fixed 3:1 ratio off one master
  clock, and real games depend on cycle-exact behaviour of both. It exercises
  the clock-domain tree and the `exact` time base properly, which a more
  forgiving machine would not.
- Excellent conformance suites exist, so "done" is measurable.
- Open-bus behaviour and cartridge mappers exercise memory-region priority,
  mirroring, and runtime remapping — the parts of `ROADMAP.md` §4.1 that are
  hardest to get right.

## Prior work we may use

[`../../../gones`](https://github.com/MagicalTux/gones) is **MIT** (© Mark
Karpelès) and its PPU/APU lineage derives from Michael Fogleman's MIT-licensed
NES emulator. Both are permissive, so the port is clean — **carry Fogleman's
copyright notice into any file derived from that code.**

## Validation

`nestest` trace comparison, blargg's test ROMs, and AccuracyCoin. Licences and
the vendoring rules are in
[`../testing/conformance-suites.md`](../testing/conformance-suites.md).
