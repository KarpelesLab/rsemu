# Video and audio devices

Consumed by: `dev/display/*`, `dev/audio/*`. Display and audio *host* backends
are separate (`ROADMAP.md` §8); this file is about the emulated hardware.

## PC video

| Device | Source |
| --- | --- |
| MDA / CGA / EGA / VGA | IBM PC and PS/2 Technical References ([bitsavers](https://bitsavers.org/)) — original register-level documentation |
| VGA registers | [OSDev: VGA Hardware](https://wiki.osdev.org/VGA_Hardware) — consolidated register reference |
| VBE / Bochs VBE extension | [OSDev: Bochs VBE Extensions](https://wiki.osdev.org/Bochs_VBE_Extensions) — the de-facto simple framebuffer interface every guest supports. This is an **interface specification**, documented independently of any implementation |
| virtio-gpu | [`../buses/virtio.md`](../buses/virtio.md) |

A plain linear framebuffer plus the VBE interface gets a modern guest to a
usable display quickly; full VGA register emulation (planar modes, the
attribute controller, CRTC timing) is needed for DOS-era software.

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
- Video output is a scanout surface plus dirty tracking — the region-level dirty
  bitmap (§4.1) exists partly for this.
- Both are natural fits for the frame-hash regression method: render N frames
  deterministically, hash them, compare.
