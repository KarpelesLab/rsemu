# Sega Master System / Game Gear

Consumed by: `boards/sms`, phase 5. Shares the Z80 core with nothing else we
build, but reuses it — which is exactly the point of doing it.

## Primary

[**SMS Power! development documents**](https://www.smspower.org/Development/Documents)
is the reference collection: VDP (video), PSG (audio), memory mapping and the
cartridge mappers, controller ports, and the Game Gear differences.

## Implementation notes

- The **VDP** is a TMS9918A derivative with Sega extensions — mode 4, the
  extended palette, and line interrupts.
- The **SN76489 PSG** is shared with several other machines, so build it as a
  reusable device rather than an SMS-specific one.
- Game Gear is the same machine with a different screen crop, an extended
  palette and a stereo control port — it should be a variant machine file, not
  a second board. That is a good test of whether the `.machine` DSL's
  `template`/`param` support is pulling its weight.
