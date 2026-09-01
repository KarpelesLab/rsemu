# Other architectures

Candidates for after the IR stabilises (`ROADMAP.md` §6, "later"). Each is a
short project once the IR and the framework exist; the work is a frontend lifter
plus a machine description.

| Architecture | Primary documentation | Notes |
| --- | --- | --- |
| Motorola 68000 | *M68000 Family Programmer's Reference Manual* and the 68000 User's Manual — NXP hosts current PDFs (search "M68000PRM"); [bitsavers](https://bitsavers.org/) has the originals | Amiga, Atari ST, Genesis, early Macs |
| MIPS | see [`mips.md`](mips.md) — the R3000A core is implemented | PlayStation, N64, routers |
| PowerPC | *PowerPC Architecture Book* I–III; NXP hosts the classic PPC manuals | Mac, GameCube/Wii, embedded |
| SuperH (SH-2/SH-4) | Renesas SH-2 and SH-4 hardware manuals | Saturn, Dreamcast |
| 65816 | WDC W65C816S datasheet + *Programming the 65816* | SNES, Apple IIGS |
| V850 | Renesas/NEC V850 architecture manuals | Automotive; see `../ghidra_v850` |
| Z80 / 8080 | see [`z80-sm83.md`](z80-sm83.md) | |

## General

[bitsavers.org](https://bitsavers.org/) and its
[Internet Archive mirror](https://archive.org/details/bitsavers) hold scanned
original manuals for essentially every processor of the 1970s–90s, which is
often the *only* remaining primary source. [Ken Shirriff's blog](https://www.righto.com/)
carries die-level analyses that settle undocumented-behaviour arguments no
manual answers.
