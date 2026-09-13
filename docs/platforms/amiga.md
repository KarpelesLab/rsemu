# Commodore Amiga 500

Consumed by: `dev/amiga`, `dev/mos`, `machines/amiga-a500.machine`,
`tests/amiga_a500_board.rs`.

A 68000, 512 KiB of chip RAM, a Kickstart ROM, two 8520 CIAs and three custom
chips — Agnus, Denise and Paula. **What exists today is the memory map and its
decoders, both 8520s, Paula, and the internal floppy drive; Agnus and Denise are
not there yet.** This page is the ledger of what the board decided, what it had
to leave open, and what the missing chips will need from it.

## Primary sources

| Source | Covers |
| --- | --- |
| [*Amiga Hardware Reference Manual*, 3rd edition](https://archive.org/details/amiga-hardware-reference-manual-3rd-edition) (Commodore-Amiga Inc.) | Appendix B, the custom-chip register summary in address order and its legend; Appendix A, every register's bits; Appendix D, the system memory maps; Appendix F, the CIA addresses, chip selects and clocks; Appendix E, the port signal assignments and the disk connector; chapter 7, interrupts; chapter 8, the disk controller, the drive lines and the UART; chapter 5, audio |
| MC68000 User's Manual (Motorola) | The reset sequence, and the instruction encodings the test ROMs are hand-assembled from |

**Nothing else.** Every Amiga emulator the project is aware of is copyleft and
is listed under *Deliberately excluded* in [`../README.md`](../README.md), and
so is AROS's source: AROS is something rsemu may run, never something it reads.

## What the manual pins down

| | |
| --- | --- |
| Chip RAM | `$00_0000`–`$07_FFFF`, and `$08_0000`–`$0F_FFFF` on a 1 MiB machine |
| Custom chips | `$DF_F000`–`$DF_FFFF`; the register table fills offsets `$000`–`$1E4` |
| CIA-A | `$BFEr01`, register `r` = 0–F; "selected when A12 is low, A13 high" |
| CIA-B | `$BFDr00`; "selected when A12 is high, A13 low" |
| System ROM | `$FC_0000`–`$FF_FFFF` in the edition's own map — see below |
| `OVL` | CIA-A `PA0`, "memory overlay bit" |
| CIA timers | the 68000's E clock, one tenth of the processor clock: 709.379 kHz PAL, 715.909 kHz NTSC |
| CIA TOD | CIA-A: a 50/60 Hz event, "VSync or line tick"; CIA-B: horizontal sync |
| CIA interrupts | CIA-A "can generate interrupt INT2", CIA-B INT6 |
| Interrupt levels | `INT2*` sets `PORTS` (level 2), `INT6*` sets `EXTER` (level 6); `TBE`, `DSKBLK`, `SOFT` 1; `COPER`, `VERTB`, `BLIT` 3; `AUD0`–`AUD3` 4; `RBF`, `DSKSYN` 5 (chapter 7; Appendix A, `INTENA`) |
| Floppy lines | CIA-B `PB0`–`PB7`: `STEP*`, `DIR`, `SIDE*`, `SEL0*`–`SEL3*`, `MTR*`; CIA-A `PA2`–`PA5`: `CHNG*`, `WPRO*`, `TK0*`, `RDY*`; the index pulse on CIA-B's `/FLAG` (Table 8-5, Appendix E) |

## What the board decided, and why

**The overlay is a decoder, not a remap.** `amiga.gary` owns one region at
address zero and forwards every access to the Kickstart ROM or to chip RAM by
the level on its `ovl` pin. Swapping two mappings instead would have to happen
inside the CIA write that clears the bit — an access already holding the space's
topology lock for reading — and the deferred alternative lands a scheduler
quantum late. `st.syscfg`'s boot alias made the same choice for the same reason.
The price is that chip RAM is reached through a call rather than a host pointer;
the escape hatch, a safe-point retopology the first time `OVL` goes low, is
written down in `src/dev/amiga/gary.rs` and has not been needed yet.

**The decoders reach their targets by name.** `BindCtx::region` returns the
region a named object publishes. The syscfg route — find the target by the
address it is mapped at — cannot work here twice over: chip RAM's only address
is the one the overlay itself claims, and a CIA's register block is not mapped
anywhere until something decodes it.

**The custom-chip space is one decode with subscribers.** `amiga.custom` holds
Appendix B as a table and each chip attaches to it, rather than each chip
mapping its own scatter of windows. Twenty-one registers belong to more than
one chip (`DMACON` to all three, every sprite's `POS` and `CTL` to Agnus and
Denise), and one table is the only place that fact can live once.

**Interrupts go through Paula, and Paula alone drives the processor.** Each
CIA's `irq` is wired to Paula's `int2` or `int6`, never to the processor, and
Paula encodes the highest enabled request onto `ipl0`–`ipl2`. The external
lines are level-sensitive: a clear of `PORTS` does not stick while CIA-A is
still requesting, because on the machine the line is shared with the expansion
bus and software acknowledges the CIA before it acknowledges Paula.

**Agnus and Paula meet at a Rust seam, not at a register.** The disk and audio
DMA move words through addresses the processor never reads (`DSKDATR` is an
early-read dummy, `AUDxDAT` is DMA-only) and pointers Agnus owns, so Paula
publishes `PaulaPort` as `ExportId::PAULA`. `src/dev/amiga/paula.rs` lists
exactly what Agnus must call; every call carries a tick of the shared colour
clock and catches Paula up to it first.

**A drive is its own device, on the CIA ports.** `amiga.floppy` takes `MTR*`,
`SEL*`, `SIDE*`, `DIR` and `STEP*` as wires and answers on `RDY*`, `TK0*`,
`WPRO*`, `CHNG*` and `INDEX*`, open-collector, exactly as Appendix E's connector
describes. The bit cells are the one thing too fast for a wire: Paula asks the
drive for them through `DiskDrive`, holding its own lock, which is why the
drive's lock ranks above `DEVICE`.

**The CIA decode is the board's.** `mos.8520` publishes sixteen registers back
to back; `amiga.cia-decode` spreads them 256 bytes apart and puts them on one
byte lane. One decoder per chip, because the A12/A13 selects never pick both.

## Open, and not guessed at

| | Why it is open | What happens instead |
| --- | --- | --- |
| **`$1FE`** | Widely called the copper's `NO-OP`, but in neither Appendix A nor B of this edition, which ends at `DIWHIGH` (`$1E4`) | An unclaimed offset: a write is dropped and counted |
| **Byte access to a custom register** | Every entry is a word and the manual says nothing about a single data strobe | Refused by the region's access constraints |
| **Reading a write-only custom register** | The manual does not say | The last word driven onto the bus — **a placeholder** that Agnus's DMA cycles will replace; `CustomBus::unclaimed` counts every use |
| **`$DFF200`–`$DFFFFF`** | Appendix D gives a 4 KiB window, the table fills 512 bytes, and whether the rest mirrors is not stated | Only 512 bytes mapped; the rest faults. `mirror(custom)` is a one-word change |
| **A0–A7 in a CIA window** | The notation gives one hex digit of register select; nothing says whether the low byte is decoded | Not decoded — `$BFE003` is register 0 |
| **The ROM base** | The edition's map is `$FC_0000`, a 256 KiB Kickstart; 512 KiB images start at `$F8_0000` | `rom-base` and `kickstart-size` are parameters, defaulting to 512 KiB |

| **Paula's disk bit cell** | "Two microseconds per bit cell" is 7.094 colour clocks, and Paula has no other clock | 7 colour clocks (14 slow): 1.974 µs PAL. The drive spins at the same rate, so a track reads back as written |
| **The drive's spindle speed and index width** | Not in the manual | 100,000 cells a revolution (300 rpm at 2 µs), an index pulse of 1,000 cells |
| **`SERDATR`'s `TBE`** | Chapter 8 says "not a mirror" of `INTREQ`'s; Appendix A says "mirror" | The chapter: the transmit buffer's own state |
| **`MSBSYNC`, precompensation** | One sentence each, and precomp is analogue | Stored and not acted on |
| **`DSKSYNC` out of reset** | The manual does not say | Zero — which matches an idle, zero read line on every cell, so `DSKSYN` is requested until software loads a sync word |
| **Audio's state diagram** | Figure 5-8's arrows are not legible in the available scan | The chapter's prose: see `paula.rs` |
| **Disk images** | ADF is AmigaDOS's format, not the hardware's | The drive's `image` slot takes a raw MFM dump of its own layout; nothing encodes a file system yet |

## What each chip will need

* **Agnus, Denise** — implement `CustomChip` on their register block, take a
  `custom = <object>` property, and call `CustomBus::attach` from `bind`. The
  copper writes through the same bus with `Origin::copper(danger)`.
  `src/dev/amiga/custom.rs` has the contract in full.
* **Agnus, for Paula** — a `paula = paula` link and `PaulaPort` from
  `ExportId::PAULA`, on the same `clk / 8` clock: disk read and write words in
  its DMA slots against `DSKPT`, audio `restart`/`fetch` requests against
  `AUDxLC` and its own pointers, and `request` with `VERTB` and `BLIT`. The
  board then needs one line for it. `DMACONR` is Agnus's to answer: Paula drives
  none of its bits.
* **Agnus, for the CIAs** — the TOD inputs: CIA-A's `tod` from vertical sync,
  CIA-B's from horizontal sync. Two `wire` lines.
