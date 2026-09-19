# `pc.video` — the VGA, and rsemu's display extension registers

Consumed by `src/dev/pc/video.rs`, `src/dev/pc/video/vga.rs`,
`src/dev/pc/video/scan.rs` and `src/fw/pcbios/vbe.rs`. The *hardware* this
models is IBM's, and the sources for it are listed in each of those files; this
page is the part that is **ours** — the small register interface a linear
framebuffer is set up through, and the VBE services the in-house BIOS builds on
it.

## Why an interface of our own

VBE 2.0 tells a guest how to *ask* for a linear framebuffer mode
(`INT 10h AX=4F02h` with bit 14 of `BX` set) and what the answer looks like; it
says nothing about how the video BIOS and the card talk to each other. Every
SVGA chip answered that differently, in extended registers of its own, and the
one interface every emulator implements — Bochs' "DISPI" ports at `0x1ce`/
`0x1cf` — has **no specification outside a GPL program's source**, which
`CLAUDE.md` puts out of reach. Reimplementing it from a description would also
be an interface whose only authority is that source.

So rsemu's display adapter has extension registers of its own design, specified
here, and `src/fw/pcbios` implements VBE 2.0 over them. The pair is
self-contained: our firmware, our card, one document.

Two consequences worth stating:

- A guest driver written for somebody else's card finds nothing at `0x1ce`.
  That is correct — this card is not that card — and the VBE services are the
  portable interface a guest is expected to use.
- A **third-party** video BIOS in the `vgabios` socket still works exactly as
  before: it drives the standard VGA register file, which is fully modelled,
  and reports no linear modes because it does not find the extension registers
  it was built for.

## Where the registers live

In the **sequencer's index space**, at indices `E0h`-`EFh`, reached through the
ordinary index/data pair at `0x3c4`/`0x3c5`:

```text
  mov dx, 0x3c4
  mov ax, (value << 8) | index      ; the usual one-OUT idiom
  out dx, ax
```

That is where SVGA vendors put their extended registers, and it means a board
needs no new address decode: a machine file that already maps `0x3c0`-`0x3cf`
has them. The index register decodes all eight bits on the VGA model (indices
`05h`-`DFh` read as zero and ignore writes), and the 6845 model decodes three
bits as it always did, so a machine with `model = "6845"` has no extension
registers at all — which is how the firmware detects the card.

`SR E0h` is a **lock**. Out of reset, and after any write of a value other than
the key, registers `E1h`-`EFh` read as zero and ignore writes; writing `72h`
(`'r'`) unlocks them, and `SR E0h` then reads `01h`. Locking again does not
undo what the other registers hold — it only stops further writes — because a
mode that is running must keep running.

| Index | Access | Meaning |
| --- | --- | --- |
| `E0h` | R/W | lock: write `72h` to unlock, anything else to lock; reads `01h`/`00h` |
| `E1h` | R | identification: `52h` (`'R'`) |
| `E2h` | R | interface revision: `01h` |
| `E3h` | R/W | control — bit 0: linear mode enable. Other bits reserved, read zero |
| `E4h`, `E5h` | R/W | width in pixels, low byte first |
| `E6h`, `E7h` | R/W | height in scan lines, low byte first |
| `E8h` | R/W | bits per pixel: 8, 15, 16, 24 or 32 |
| `E9h`, `EAh` | R/W | pitch: bytes from one line to the next, low byte first |
| `EBh`, `ECh`, `EDh` | R/W | display start: the byte offset of the top left pixel, 24 bits |
| `EEh` | R/W | bank: which 64 KiB of video memory the window at `A0000h` shows |
| `EFh` | R | video memory in 256 KiB units |

### What the linear mode does

While `E3h` bit 0 is set:

- The **scanout** reads video memory as packed pixels — `start + y × pitch +
  x × bytes-per-pixel` — instead of running the CRT controller's address
  sequence over the planes. A pitch of zero means `width × bytes-per-pixel`.
  The CRT controller still provides the *timing*, so the frame rate is whatever
  the mode set programmed into it, and the attribute controller's palette is
  out of the path.
- The window at `A0000h` becomes a plain 64 KiB aperture on video memory at
  `bank × 64 KiB`, with none of the planar pipeline — no latches, no map mask,
  no write modes. This is what VBE's window (`AX=4F05h`) moves, and it is how a
  real-mode program without a linear aperture reaches the picture.
- Geometry the model will not scan out — a zero dimension, a depth that is not
  in the table, or more than 4096 × 4096 pixels — produces an empty surface
  rather than an allocation. A guest cannot make the host allocate gigabytes by
  writing nonsense into sixteen-bit registers.

Pixel formats are VBE 2.0's, little-endian in memory:

| Depth | Layout |
| --- | --- |
| 8 | one byte, an index into the VGA's own DAC (so the DAC ports are the palette interface) |
| 15 | `0RRRRRGG GGGBBBBB` as a 16-bit word |
| 16 | `RRRRRGGG GGGBBBBB` as a 16-bit word |
| 24 | three bytes: blue, green, red |
| 32 | four bytes: blue, green, red, unused |

### The linear framebuffer's address

Video memory is also a **region**, `lfb`, which a board hands to the display
adapter's PCI function (`pc.vga-pci`'s `framebuffer` property) to decode behind
**BAR0** — a 16 MiB prefetchable memory window, sized as the aperture with the
card's actual memory at the bottom of it. So a guest finds the framebuffer the
way it finds any PCI resource: class code `030000`, base address register 0.

`PhysBasePtr` in VBE's mode information block is that BAR's base, read out of
configuration space. Nothing in the device hardwires an address.

## The VBE services

`src/fw/pcbios/vbe.rs` implements, over the registers above:

| Function | What it does |
| --- | --- |
| `AX=4F00h` | controller information: the `VESA` signature, version 2.0, the OEM string, the mode list, and total memory from `SR EFh` |
| `AX=4F01h` | mode information: geometry, depth, pitch, the colour masks, and `PhysBasePtr` from BAR0 |
| `AX=4F02h` | set mode: a VGA mode number goes to `INT 10h AH=00h`; a VBE mode programs the CRT controller for the timing and the extension registers for the picture |
| `AX=4F03h` | the current mode |
| `AX=4F05h` | the window position, in 64 KiB banks (`SR EEh`) |

**Not implemented, and they return `AH=01h` rather than pretending:**
`AX=4F04h` (save and restore state), `AX=4F06h` (the scan line length),
`AX=4F07h` (the display start) and `AX=4F08h` (the DAC width). The registers
they would move are all there — pitch, start and bank are `SR E9h`-`EEh` — so
each is a short routine; none is written on a guess, and the first guest that
needs one is the reason to add it.

The mode list is generated from a table in the ROM: 640x480, 800x600 and
1024x768, each at 8, 16 (5:6:5) and 32 bits a pixel, which is 3 MiB at the
largest and fits the 4 MiB the boards give the adapter.
`docs/platforms/pc-at.md` records what has been measured running them.
