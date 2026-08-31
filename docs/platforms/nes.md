# NES / Famicom

Consumed by: `boards/nes` — the first real machine, and therefore the phase
that proves the framework (`../../ROADMAP.md` §13).

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

## Regional variants — three machines, not one

Verified against NESdev's [Cycle reference chart](https://www.nesdev.org/wiki/Cycle_reference_chart).
Each is a separate `.machine` file; the differences are far more than a
frequency.

| | NTSC (2C02) | PAL (2C07) | Dendy (UA6538) |
| --- | --- | --- | --- |
| Master clock | 236250000/11 Hz | 26.6017125 MHz = 53203425/2 Hz | like PAL |
| CPU divider | ÷12 | ÷16 | ÷15 |
| Master clocks per PPU dot | 4 | 5 | 5 |
| PPU dots per CPU cycle | 3 | **3.2** | 3 |
| Scanlines per frame | 262 | 312 | 312 |
| Odd-frame dot skip | yes | **no** | no |
| Picture height | 240 | 239 | 239 |
| Post-render blanking | 1 line | 1 line | **51 lines** |
| VBlank after NMI | 20 lines | 70 lines | like NTSC |
| APU frame counter | 60 Hz | 50 Hz | 59 Hz |
| Frame rate | 60.0988 Hz | 50.0070 Hz | like PAL |

Two of these numbers are worth pausing on, because the framework was designed
for exactly them (`ROADMAP.md` §4.2):

- **Neither master clock is an integer number of hertz.** NTSC is 236.25 MHz ÷
  11 by definition; PAL is 26.6017125 MHz by definition. This is why frequency
  literals in the DSL are *rational*, and why writing `21477272 Hz` would be
  wrong rather than merely imprecise.
- **PAL runs 3.2 PPU dots per CPU cycle** — 16:5, not an integer. Nintendo kept
  the Johnson counter's even period and divided by 16 rather than 15. An
  emulator that counts CPU cycles cannot represent that ratio exactly and must
  fudge it; counting *master ticks* makes it exact by construction, because both
  domains descend from one crystal. Dendy is the counterexample that proves the
  point: same crystal, ÷15, and the ratio returns to a clean 3.
  **Never introduce a "dots per CPU cycle" constant** — derive from the
  dividers, or PAL is wrong the moment someone writes `3`.

## Validation

`nestest` trace comparison, blargg's test ROMs, and AccuracyCoin. Licences and
the vendoring rules are in
[`../testing/conformance-suites.md`](../testing/conformance-suites.md).
