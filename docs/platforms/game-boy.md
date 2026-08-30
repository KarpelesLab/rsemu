# Game Boy / Game Boy Color

Consumed by: `boards/gameboy` — the genericity proof. If any core API
has to change to accommodate the Game Boy, that was a core-design bug and the
fix belongs in the core.

## Primary

| Source | Covers | Licence |
| --- | --- | --- |
| [**Pan Docs**](https://gbdev.io/pandocs/) | The complete machine: memory map, PPU modes and timing, APU, timers, interrupts, MBC cartridge controllers, CGB extensions | **CC0** (verified) — no restrictions on quoting or adapting |
| [Game Boy: Complete Technical Reference](https://gekkio.fi/files/gb-docs/gbctr.pdf) | Hardware-measured timing at sub-instruction granularity; the source for T-cycle exact behaviour | |
| [gbdev.io](https://gbdev.io/) | Portal to the wider documentation set and toolchain | |

Pan Docs being CC0 is worth noting explicitly: it is one of the very few
emulation references that can be quoted verbatim into source comments with no
attribution burden at all.

## Implementation notes

- The CPU is an **SM83**, not a Z80 and not an 8080 — see
  [`../cpu/z80-sm83.md`](../cpu/z80-sm83.md).
- PPU mode timing varies with sprite count and window state; the "mode 3
  extension" behaviour is what the accuracy suites actually test.
- MBC1's multicart wiring and MBC3's RTC are the two mappers that need care.
- The APU's frame sequencer is driven off the divider register, so `DIV` writes
  have audible side effects — a good test of the clock-domain design.

## Validation

Mooneye test suite (MIT — verified) and blargg's Game Boy ROMs. See
[`../testing/conformance-suites.md`](../testing/conformance-suites.md).
