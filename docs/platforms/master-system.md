# Sega Master System / Game Gear

Consumed by: [`src/dev/sms/`](../../src/dev/sms), [`machines/sms-ntsc.machine`](../../machines/sms-ntsc.machine)
and [`machines/sms-pal.machine`](../../machines/sms-pal.machine). Shares the Z80
core with nothing else we build, but reuses it — which is exactly the point of
doing it (`ROADMAP.md` §13).

## Primary

[**SMS Power! development documents**](https://www.smspower.org/Development/Documents)
is the reference collection and the source for everything below unless another
is named: the VDP registers and mode table, the video timing and the V/H counter
tables, the PSG's register format and noise taps, the memory map and Sega's
standard mapper, the I/O port assignments, and the peripheral pinout.

Supporting primary sources:

* the **TMS9918A** datasheet — the four legacy modes, the sprite attribute
  table, and the colour table this VDP quantises to six bits;
* the **SN76489** datasheet — the attenuation steps and the counter chain;
* Zilog **UM0080**, the Z80 user manual — the instruction set, and the I/O
  machine cycle's automatic wait state.

**No emulator source of any licence was consulted** for any of this. MAME,
higan, Emulicious, BizHawk, Meka, Kega and everything derived from them are off
limits (`ROADMAP.md` §1).

## What is implemented

| Part | Class | Covers |
| --- | --- | --- |
| VDP (315-5124 / 315-5246) | `sms.vdp` | mode 4 at 192/224/240 lines, the four TMS9918A modes, 16 KiB VRAM, 32-entry CRAM, the line interrupt, the status register's three flags and their read-clears-them behaviour, the V and H counters |
| PSG (SN76489) | `sms.psg` | three square channels, the noise channel and its shift register, the four attenuators, the two-byte register protocol |
| Cartridge | `sms.mapper` | Sega's standard mapper: three 16 KiB slots, the fixed first kilobyte, cartridge RAM and its bank bit |
| I/O | `sms.io` | the two control pads at `$DC`/`$DD`, `$3E` and `$3F`, the Reset button, and Pause as an **NMI** |
| Debug console | `sms.sdsc` | [SMS Power!'s SDSC specification](https://www.smspower.org/Development/SDSCDebugConsoleSpecification), `$FC`/`$FD` — how a test ROM reports headlessly |

The oscillators are exact rationals rather than the rounded decimals every
reference prints, because a rounded frequency nails a rounding error into the
timeline (`ROADMAP.md` §4.2):

```text
  NTSC   945000000/88 Hz   3 x the NTSC colour subcarrier, 315/88 MHz
  PAL    10640685 Hz       12/5 x the PAL subcarrier, 4433618.75 Hz
```

The Z80 is master ÷ 3 and the VDP's pixel counter is master ÷ 2, so every ratio
inside the console is exact by construction — two pixels to three CPU cycles,
forever.

## What is deliberately not implemented

Written down rather than discovered:

* **The VDP's pixel pipeline.** A line is rendered in one go at its first dot.
  Per-*line* raster effects are correct, because a line interrupt is raised
  after the line it belongs to is drawn; a mid-*line* register change is not.
* **The H counter's TH latch**, and therefore the Light Phaser.
* **The BIOS.** Export consoles shipped with one; it is Sega's copyrighted code
  and rsemu ships none, so `$3E`'s BIOS-enable bit is recorded and not acted on.
  A cartridge boots directly, which is what a Japanese Mark III does.
* **The Codemasters, Korean and 4 Pak mappers**, the FM sound board, and the
  3-D glasses.
* **The Game Gear.** It is the same machine with a different screen crop, a
  12-bit palette written through a two-byte CRAM latch, and a stereo control
  port at `$06`. It should be a variant machine file, not a second board — a
  good test of whether the DSL's `template`/`param` support is pulling its
  weight.

## Conformance

The Master System's test-ROM ecosystem is small, and the honest summary is that
**almost none of it can be automated**. FluBBa's *SMS VDP Test* and sverx's *SMS
Test Suite* report on screen — the second wants buttons pressed — and neither
has a documented pass/fail memory location, so a harness for either would have
to hash a framebuffer against a picture nobody has published.

The exception is **ZEXALL-SMS**, Maxim's port of Frank Cringle's Z80 instruction
exerciser, which writes its verdict to the SDSC debug console a character at a
time. `scripts/fetch-testdata.sh sms` fetches it, and
[`src/dev/sms/conformance.rs`](../../src/dev/sms/conformance.rs) runs it against
the **shipped machine** rather than a synthetic bus — a stronger claim than the
Z80's own 67/67, because it also exercises the mapper, the interrupt wire and
the scheduler's interleaving of three lazily-advanced devices.

It is GPL-2.0, so it is fetched at test time and never committed. Its own README
warns that the longest single test is well over an hour of emulated time; the
tests are ordered fastest-first so a truncated run is still informative, and the
runner reports a truncated run as truncated rather than as a pass.
