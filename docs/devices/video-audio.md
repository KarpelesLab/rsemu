# Video and audio devices

Consumed by: `dev/display/*`, `dev/audio/*`. Display and audio *host* backends
are separate (`ROADMAP.md` §8); this file is about the emulated hardware.

## PC video

| Device | Source |
| --- | --- |
| MDA / CGA / EGA / VGA | IBM PC and PS/2 Technical References ([bitsavers](https://bitsavers.org/)) — original register-level documentation |
| VGA registers | [OSDev: VGA Hardware](https://wiki.osdev.org/VGA_Hardware) — consolidated register reference |
| VBE | The VESA BIOS Extension Core Functions Standard 2.0 — an interface standard: what a *caller* sees, and nothing about how a card is driven |
| The card's own linear-mode registers | [`pc-video.md`](pc-video.md) — **ours**. Not Bochs' "DISPI" ports, whose only specification is a GPL program's source (`../../CLAUDE.md`, provenance) |
| virtio-gpu | [`../buses/virtio.md`](../buses/virtio.md) |

Both halves are implemented and the order above is the order it happened in
reverse: `pc.video`'s VGA model is the full register emulation — planar
memory, the four write modes, the attribute controller, the CRT controller's
own address sequence — and the linear framebuffer sits behind it for the
guests that want one, through a PCI BAR and rsemu's own extension registers.
[`pc-video.md`](pc-video.md) specifies the latter; `docs/platforms/pc-at.md`
records what has been measured running both.

## Console video

| Machine | Source |
| --- | --- |
| NES PPU | [NESdev PPU](https://www.nesdev.org/wiki/PPU) — per-cycle pipeline |
| Game Boy PPU | [Pan Docs](https://gbdev.io/pandocs/) (CC0) |
| SMS VDP | [SMS Power! documents](https://www.smspower.org/Development/Documents) |

## Audio

| Device | Source |
| --- | --- |
| NES APU | [NESdev APU](https://www.nesdev.org/wiki/APU) |
| Game Boy APU | [Pan Docs](https://gbdev.io/pandocs/) |
| SN76489 PSG | Texas Instruments datasheet; SMS Power! documents |
| AC'97 / Intel HDA | Intel specifications **[browser]** |
| Sound Blaster / OPL | Creative and Yamaha (YM3812/YMF262) datasheets |

## Implementation notes

- **Audio is a clock-domain problem before it is a DSP problem.** The sample
  clock is a domain like any other; resampling to the host rate happens in the
  host layer, anchored to virtual time. Never let the host audio callback drive
  guest timing.
- Video output is a scanout surface plus dirty tracking. Note that the
  region-level dirty bitmap (§4.1) is **not** what drives it and never has
  been: `VideoScanout::capture` re-renders every character cell each frame and
  the VNC server does a `memcmp` per row (`docs/system/remote-display.md`).
  An audit found the bitmap has no production consumer at all.
- Both are natural fits for the frame-hash regression method: render N frames
  deterministically, hash them, compare.
