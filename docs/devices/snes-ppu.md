# `snes.ppu` — the SNES S-PPU

The Super Famicom / SNES picture processing unit: PPU1 (5C77) and PPU2 (5C78),
modelled as one device. Feature `dev-snes-ppu`, code in `src/dev/snes/ppu/`,
host adapter in `src/host/display/snes.rs`.

## Provenance

Written from documentation only:

- nocash **fullsnes** — "SNES PPU Registers", "SNES PPU Timers and Status",
  the sprite, BG and colour-math chapters, and the timing chapter.
- The **SNESdev wiki** (snes.nesdev.org) — *PPU registers*, *Backgrounds*,
  *Tiles*, *Tilemaps*, *Sprites*, *Offset-per-tile*, *Mode 7*, *Windows*,
  *Color math*, *Timing*.

No emulator source of any licence was opened: not bsnes/higan/ares, Mesen,
snes9x, MAME, or anything derived from them.

## Connection surface

| Kind | Name | Notes |
| --- | --- | --- |
| class | `snes.ppu` | snapshot version 1 |
| region | `regs` (also the empty name) | 64 bytes; map at `$2100` of the B-bus; byte accesses only |
| output pin | `vblank` | level, high from line 225 (240 with overscan) dot 0 until line 0 dot 0 |
| output pin | `hblank` | level, high from dot 274 until dot 1 of the next line |
| input pin | `extlatch` | `WRIO` bit 7 / port 2 pin 6; a falling edge latches H/V |
| export | `ExportId::SNES_PPU_BEAM` (19) | `Opaque(Arc<SnesPpuBeam>)` |
| prop | `region` | `ntsc` (default) or `pal` |
| prop | `ppu1-version` | `STAT77` bits 0-3, default 1 |
| prop | `ppu2-version` | `STAT78` bits 0-3, default 3 |
| clock | master ÷ 1 | `add_clock_domain`; the device's tick is one master clock |

### The beam handle (`SnesPpuBeam`)

Methods that answer about *now* catch the PPU up through the lazy handle
first; `peek_*` do not. Never call with a lock ranked at or above
`ENGINE_LOCK_RANK` (`0x5c00`, between `DEVICE` and `WIRE`) held.

| Method | Returns |
| --- | --- |
| `sync()` | catch up |
| `position()` / `peek_position()` | `BeamPosition { h, v, field, master_in_line }` |
| `latch_counters()` | latch H/V unconditionally, set `STAT78` bit 6 |
| `set_extlatch(bool)` | drive `EXTLATCH`; 1→0 latches, `$2137` latches only while 1 |
| `region()` | `Region::Ntsc` / `Region::Pal` |
| `frame()` | frames completed (counted at each vblank start) |
| `peek_master()` | master clocks the PPU has run |
| `vblank()`, `hblank()` | the levels |
| `vblank_line()` | 225 or 240 (decided at line 225) |
| `lines_in_frame()`, `interlace()` | the current frame's geometry |
| `master_until(v, h)` | master clocks until the beam reaches dot `h` of line `v` |

`timing` also exports the pure geometry (`dot_start`, `dot_at`,
`Region::line_length`, `Region::lines_per_frame`) for an S-CPU that wants to
compute without the lock.

## Timing

| | NTSC | PAL |
| --- | --- | --- |
| master clock | 236 250 000 / 11 Hz | 21 281 370 Hz |
| line | 1364 clocks, H 0..=339 | same |
| long dots | 323 and 327, 6 clocks each | same |
| short line | V 240, field 1, interlace off: 1360 clocks, no long dots | — |
| long line | — | V 311, field 1, interlace on: 1368 clocks, H reaches 340 |
| lines | 262 (+1 interlaced, field 0) | 312 (+1 interlaced, field 0) |

The SNESdev wiki counts the same 1364 clocks as four five-clock dots rather than
two six-clock ones; the total agrees and this model follows fullsnes. That the
extra interlace line falls on field 0 is an inference from "each even frame".

## Rendering

Per line, at dot 22 (master clock 88, the first visible pixel), from the
registers as they stand. HDMA and H-blank writes therefore land on the line
they target; a write in the visible part of a line takes effect a line late.
The framebuffer is 512 × 478 RGB555 with brightness applied (`c · b / 15`);
low-resolution lines are stored doubled, non-interlaced lines on both rows of
their pair, and the output size of the last completed frame is reported as
256/512 × 224/239/448/478.

Modelled: modes 0-7 with their depths and priority tables (BG3 priority in mode
1), 8×8/16×16 tiles, the four tilemap sizes, offset-per-tile in 2/4/6, hires 5/6
(sub screen on even columns, main on odd) and pseudo-hires, mode 7's matrix with
the `& !63` origin truncation, screen-over modes and EXTBG, direct colour,
sprites with all eight `OBSEL` sizes, the 32-sprite / 34-tile limits with the
reverse fetch order and the X = −256 quirk, priority rotation, OBJ interlace,
mosaic, both windows with their logic and the per-screen masks, colour math
(add/subtract, half, fixed colour, sub-screen backdrop, clip-to-black and
prevent regions, OBJ palettes 4-7 only), forced blank, interlace.

Not modelled: mid-line register changes, CGRAM/OAM write corruption during
rendering, `MPYx` and VRAM-read garbage during rendering, colour math on hires
even columns, the odd vertical flip of rectangular sprites, and whether `VOFS`
counts interlaced lines in modes 5/6.

## Registers

The two-write latches (`BGnHOFS`, `BGnVOFS`, the mode-7 registers), the VRAM
prefetch and `VMAIN` remap, the CGRAM and OAM flip-flops and the OAM reload at
vblank, `$2137`/`$213C`/`$213D`/`$213F` with their flip-flops, and the open bus
— PPU1's latch for the eighteen write-only registers fullsnes lists, the S-CPU's
own bus byte (`MemAttrs::bus`) for the rest — are described in
`src/dev/snes/ppu/regs.rs`. Debug reads change nothing; debug writes are
dropped.
