# Super Nintendo — the `snes-ntsc` and `snes-pal` machines

Consumed by: `machines/snes-ntsc.machine`, `machines/snes-pal.machine`,
`src/dev/snes/`, `src/cpu/w65c816/`, `src/cpu/spc700/`.

```sh
rsemu run snes-ntsc --media cart=game.sfc
rsemu run snes-pal  --media cart=game.sfc
```

The cartridge goes in the `cart` slot: a `.sfc`/`.smc` image, with or
without a 512-byte copier header, decoded LoROM, HiROM or ExHiROM by its
internal header. The window, VNC and `--screenshot` show the S-PPU's picture
at 256 or 512 wide and 224, 239, 448 or 478 tall; `--record-audio` writes
the S-DSP's 32 kHz stereo, resampled. Keys: the arrows, `Z` B, `X` A, `A` Y,
`S` X, `Q` L, `W` R, Return Start, Shift Select.

## The board

| Part | Class | Doc |
| --- | --- | --- |
| 5A22's 65C816 core | `cpu.w65c816` | [`../cpu/65816.md`](../cpu/65816.md) — passes all 5 080 400 SingleStepTests/65816 vectors, bus traces included |
| 5A22's peripherals: NMI/IRQ timers, DMA/HDMA, multiply/divide, joypads, bus speed, DRAM refresh | `snes.scpu` | `src/dev/snes/scpu.rs` |
| S-PPU (5C77 + 5C78) | `snes.ppu` | [`../devices/snes-ppu.md`](../devices/snes-ppu.md) |
| Audio unit: SPC700, 64 KiB ARAM, timers, ports, boot ROM | `snes.apu` | [`../cpu/spc700.md`](../cpu/spc700.md) — all 256 000 SingleStepTests/spc700 vectors |
| S-DSP | inside `snes.apu` | [`../devices/snes-dsp.md`](../devices/snes-dsp.md) |
| 128 KiB WRAM and its B-bus port | `snes.wram` | `src/dev/snes/wram.rs` |
| Cartridge: LoROM/HiROM/ExHiROM, battery SRAM | `snes.cart` | `src/dev/snes/cart.rs` |

### Clocks

Two crystals, as on the console: the master (21.477 MHz NTSC, 21.281 MHz PAL)
clocks the CPU, the S-CPU's peripherals and the PPU, all counting master
clocks; the audio unit's 24.576 MHz resonator, divided by 24, clocks the
SPC700. Every CPU cycle is priced by `snes.scpu` — 6, 8 or 12 master clocks by
address, 6 for FastROM with `MEMSEL` set — and DMA, HDMA and the 40-clock DRAM
refresh of every line are charged to the CPU as held cycles through its `/RDY`
gate. HDMA runs from the S-CPU's own beam-timed events, so it lands on its dot
even while the CPU sits in `WAI`.

### The memory map

The board file's `map` statements are fullsnes's memory map: in every system
bank (`$00–$3F`, `$80–$BF`) LowRAM at `$0000`, the B-bus at `$2100` (PPU
`$2100–$213F`, APU ports `$2140–$217F`, WRAM port `$2180–$2183`), the joypad
ports at `$4016`, the S-CPU at `$4200` and `$4300`, the cartridge from
`$6000`; banks `$40–$7D` and `$C0–$FF` are the cartridge's and `$7E–$7F` are
WRAM. The A-bus is `unassigned = open-bus` and the 65C816 answers an undriven
read with its data-bus latch, so the cartridge's holes read as the hardware's
do.

## The boot ROM

The SPC700's 64-byte IPL ROM is Nintendo's code and is **not shipped**.
`snes.apu` boots a clean-room replacement written from the documented upload
protocol, which is all a cartridge talks to. Software that reads the real
ROM's bytes, or jumps into the middle of it, needs a dump: the `ipl` property
takes one (`object apu "snes.apu" { clock = apuosc / 24, ipl = "ipl" }` and
`--media ipl=ipl.rom`).

## Sources

nocash **fullsnes** and the **SNESdev wiki** for the machine; the **WDC
W65C816S datasheet** for the CPU; the Sony SPC700 documentation as fullsnes
and the wiki give it. The SingleStepTests corpora (MIT) are the CPU oracles,
fetched by `scripts/fetch-testdata.sh sst-65816` and `sst-spc700`. bsnes/higan,
snes9x, Mesen, ares and every other emulator were **not** consulted
(`CLAUDE.md`, Provenance).

## Proof

- `tests/snes_board.rs` — on both boards, one hand-assembled program: native
  mode, CGRAM, a GP DMA into WRAM through `WMDATA`, an HDMA table with a repeat
  run, NMI with the automatic joypad read, a V-IRQ, the APU's ready byte; the
  backdrop pixel; a mid-run snapshot round trip.
- Each component's unit tests, and the two CPU conformance suites.

## What is not modelled, stated plainly

- **Enhancement chips**: Super FX, SA-1, DSP-1…4, Cx4, S-DD1, SPC7110, OBC1,
  S-RTC, ST010/011/018, MSU-1. A cartridge that needs one boots into whatever
  its code does without it.
- **The multitap, mouse, Super Scope** and every controller but the standard
  pad; `JOY3`/`JOY4` read zero.
- **Mid-line PPU effects**: the S-PPU draws each line whole at its first
  visible dot, so a register write made in the visible part of a line lands a
  line late (H-blank writes and HDMA are on time). See the PPU doc's list.
- **The 5A22's multiplier and divider are instant**; real silicon needs 8 and
  16 CPU cycles, and a read in between sees a partial result.
- **The H/V IRQ fires at the start of dot `HTIME`**, not the few dots later
  the hardware does, and DMA's clock alignment is folded into fixed overheads.
- **The two crystals do not drift.** Each is exact and nominal; real consoles'
  APU clocks run a little fast and vary.
- **Battery SRAM is kept in snapshots, not in a `.srm` file**: a save survives
  a snapshot, not a fresh `rsemu run`.
