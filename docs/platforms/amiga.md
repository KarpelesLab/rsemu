# Commodore Amiga 500

Consumed by: `dev/amiga`, `dev/mos`, `host/display/amiga.rs`,
`host/input/amiga.rs`, `bin/rsemu.rs`, `machines/amiga-a500.machine`,
`machines/tests/amiga-denise.machine`, `machines/tests/agnus-board.machine`,
`tests/amiga_a500_board.rs`, `tests/amiga_denise_board.rs`,
`tests/agnus_board.rs`, `tests/amiga_a500_chipset.rs`,
`tests/amiga_a500_input.rs`, `tests/amiga_adf.rs`,
`tests/amiga_a500_kickstart.rs`; and for the A600 (its own section),
`machines/amiga-a600.machine`, `src/dev/amiga/gayle.rs`,
`tests/amiga_a600_board.rs`, `tests/amiga_a600_hdf.rs`; and for the A3000 (its
own section), `machines/amiga-a3000.machine`, `src/dev/scsi/`,
`src/dev/wd33c93.rs`, `src/dev/amiga/sdmac.rs`, `src/dev/amiga/ramsey.rs`,
`tests/amiga_a3000_board.rs`, `tests/amiga_a3000.rs`; and for the A4000 (its
own section), `machines/amiga-a4000.machine`, `src/dev/amiga/ide.rs`,
`tests/amiga_a4000_board.rs`, `tests/amiga_a4000.rs`; and for the ECS section,
`machines/amiga-a500plus.machine`, `src/dev/amiga/rtc.rs` and
`tests/amiga_a500plus.rs`; and for the AA section, `src/dev/amiga/denise/aga.rs`
and `tests/amiga_lisa.rs`.

A 68000, 512 KiB of chip RAM, a Kickstart ROM, two 8520 CIAs and three custom
chips — Agnus, Denise and Paula. **All of them are on the A500 board now**: the
memory map and its decoders, both 8520s, Paula, the internal floppy drive,
Agnus and Denise, with Agnus counting the beam and pushing lines into Denise —
and the keyboard and mouse, so a person at `rsemu run amiga-a500 --vnc` sees
the picture, types, and points. This page is the ledger of what the board
decided, what it had to leave open, and where each chip meets the others.

## Primary sources

| Source | Covers |
| --- | --- |
| [*Amiga Hardware Reference Manual*, 3rd edition](https://archive.org/details/amiga-hardware-reference-manual-3rd-edition) (Commodore-Amiga Inc.) | Appendix B, the custom-chip register summary in address order and its legend; Appendix A, every register's bits; Appendix D, the system memory maps; Appendix F, the CIA addresses, chip selects and clocks; Appendix E, the port signal assignments and the disk connector; chapter 7, interrupts; chapter 8, the disk controller, the drive lines and the UART; chapter 5, audio. For Denise: Chapter 3 (playfields, the display window, data-fetch timing, dual playfields, scrolling, hold-and-modify, extra-half-brite), Chapter 4 (sprites), Chapter 7 (video priorities, collision detection), Appendix A (register bits), Appendix C (the ECS notes, including the display window's chip column) and Appendix J (Denise's pins) |
| The same manual as it appears on the Amiga Developer CD 2.1 | Chapter 2 (the copper, the beam counters' ranges and clocks), chapter 3 (display window and data fetch), chapter 4 (sprite DMA), chapter 6 (the blitter), chapter 7 (DMA control, beam position, interrupts), Appendix A (bit layouts), Appendix C (ECS, Agnus identification) — what `amiga.agnus` is written from, and the copy `regs.rs` was checked against row by row |
| MC68000 User's Manual (Motorola, M68000UM/AD rev. 8) | The reset sequence, and the instruction encodings the test ROMs are hand-assembled from; Table 3-1 and §5.1 for what a byte access drives onto the bus |
| *A500/A2000 Technical Reference Manual* (Commodore) | Table 6-1 (Fat Agnus's pins: `UDS*`/`LDS*` used only for DRAM; `A1`–`A8` onto the register-address bus), §7.3 (the A2000 PAL equations Gary replaced: the chip data buffers, `/ROME`, and `/RGAE` for bank 6), the note on memory at `$C00000` and the clock section's "Clock Warning" |
| *A500/A2000 Gary Specification* (Commodore; the copy in Dave Haynie's A2000 documents) | Gary's pins, its bank decode on `A17`–`A23`, `ERAM` (`$C0_0000`–`$C7_FFFF`, "expansion RAM") and the `NEXP` pin a trapdoor card grounds; `NROM` covering `$E0_0000`–`$E7_FFFF` |
| A500 schematics #312511-02 rev. 5 and #312511-03 rev. 6A/7 (Commodore) | Sheet 2: the chip data bus buffers (`U10`–`U13`) and their shared enables; sheet 3: the ROM socket `U6` |
| The same *Hardware Reference Manual*, for input | Appendix G, "Keyboard Interface" (pp. 357-364): the protocol, timing, handshake, resync, power-up sequence, special codes and the matrix table with every key's legend; chapter 8, "The Keyboard" (pp. 251-254) and "Reading Mouse/Trackball Controllers" and "Mouse Buttons" (pp. 229-233); Table 8-4 (`POTGO`); Appendix A, `JOY0DAT` and `JOYTEST` (pp. 281-282); Appendix E, CIA port assignments; Appendix F, the 8520's serial port and "Bidirectional Feature" |
| *Specification for the Advanced Amiga (AA) Chip Set* (Commodore-Amiga; the typed-in AmigaGuide edition, "Pandora Chipset Documentation") | §1 the summary of new features, §2 their explanation (bitplanes, HAM8, EHB, dual playfields, sprites, the colour lookup table, collision, the horizontal comparators, compatibility), §3 the register list by address, §4 each new or changed register's bits (`BPLCON0`–`BPLCON4`, `CLXCON2`, `COLORx`, `DIWHIGH`, `FMODE`, `LISAID`, `SPRxPOS`/`CTL`/`DAT`, `BPLxDAT`), §5 Lisa's display and sprite modes and the scroll ranges — what Lisa (`revision = "aga"`) is written from. Fetched as the document alone |
| [*Amiga ROM Kernel Reference Manual: Devices*, 3rd edition](http://amigadev.elowar.com/read/ADCD_2.1/Devices_Manual_guide/node015B.html) (Commodore-Amiga Inc.), Appendix C | The floppy's track and sector layout, the MFM encoding and its odd/even split, and the boot block's type and checksum — what `src/dev/amiga/adf.rs` and `src/host/media/adf.rs` are written from. See *Disks* below for the one thing it leaves out |

**Nothing else.** Every Amiga emulator the project is aware of is copyleft and
is listed under *Deliberately excluded* in [`../README.md`](../README.md), and
so is AROS's source: AROS is something rsemu may run, never something it reads.

## What the manual pins down

| | |
| --- | --- |
| Chip RAM | `$00_0000`–`$07_FFFF`, and `$08_0000`–`$0F_FFFF` on a 1 MiB machine |
| Slow RAM | `$C0_0000`–`$D7_FFFF`, "Internal expansion (slow) memory (on some systems)"; an A501 card is 512 KiB at `$C0_0000` |
| Custom chips | `$DF_F000`–`$DF_FFFF`; the register table fills offsets `$000`–`$1FE` |
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

**An empty address completes and floats.** The space's `unassigned` policy is
`open-bus`. Appendix D marks the gaps "Reserved. Do not use", not faulting;
Appendix K has the bus controller drive `/DTACK` for a slave that does not
answer and `/BERR` only for a bus collision or DMA error; and the MC68000 user's
manual (§5.4) makes `/BERR` external circuitry a board may omit. Kickstart 2.04
and AROS both probe `$F00000` while the overlay is still up, and under a
`fault` policy that probe double-faulted the processor.

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
mapping its own scatter of windows. Twenty-three registers belong to more than
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

**The four audio channels reach a host as stereo.** Paula's channels are two
pairs — 0 and 3 to the left output, 1 and 2 to the right (chapter 5) — and with
`record` set it integrates each pair's `sample × volume` sum over 32 colour
clocks and queues the result, which `host::audio::amiga` converts to a host
rate. 32 colour clocks is a choice, not a fact: the chip has no output sample
clock, and the manual's minimum period of 124 bounds a channel at about
28.6 kHz (PAL), so a frame rate of 110 840.46875 Hz carries everything it can
produce. The board's two output filters — the fixed RC low-pass and the
switchable "LED" one on CIA-A's `PA1` — are **not** modelled: they are on the
board rather than in the chip, they differ between models, the switchable one
cannot be a fixed `Pole` in the audio seam, and the Hardware Reference Manual
gives neither corner frequency. `src/dev/amiga/paula.rs` has the long form.

**Every word of a block plays once, in order.** Agnus serves each channel one
audio slot a line (chapter 6's time-slot allocation) and a word lasts at least
248 colour clocks against a 227-count line, so a channel never has two words
outstanding and a block is exactly `AUDxLEN` words; `tests/amiga_a500_audio_dma.rs`
asserts both on a hand-assembled tone, sample by sample, with the guest counting
its own block interrupts. It did not hold at first, and neither chip was the
cause: a scheduler round that ended on Agnus's next line could leave the
colour-clock domain a fraction of a tick short of it, and that line then went
undelivered until the next quantum boundary. Measured under Kickstart 2.04,
Agnus fell behind by more than a line 5 526 times in eight seconds, by up to a
whole millisecond (3 547 colour clocks). Paula, caught up first, crossed two
word boundaries before the slot came, and about a fifth of a steady tone's
words played twice. `Scheduler::sync_lazy_devices` now delivers the event a
round ended on; since then no device on this board is ever more than a line
behind at a round boundary, and the same fix puts `VERTB`, the copper, the
blitter's interrupt and the CIA timers back on their own counts rather than up
to a millisecond late.

What is left open is **starting**: Figure 5-8's diagram is not legible in the
copy this was written from, and at a period shorter than a line the channel's
first word boundary can come before its first slot. The model then plays the
stale buffer for that boundary and counts it, so the first block is one word
short. From the second block on the stream is exact at every period.

**The 68000 gets its whole clock.** It used to run at half: 6 204 233 cycles
executed in 1.74 virtual seconds, 50.2 % of what `clk / 4` owes, measured
from the processor's own count. Paula was a runnable — only so that it could
poll the host serial port once a round — on the processor's own crystal, and
a round divided a crystal's span between the runnables on it, so the poll took
half of every round with nothing executing. The chips all kept time; only the
processor was slow.

Paula executes nothing, and it is not a runnable any more. The poll happens at
the end of every catch-up that moves the chip (`Shared::advance_to`), which
`Scheduler::sync_lazy_devices` reaches at least once a round and every guest
access reaches besides, so a waiting byte is taken no later than it was and a
refused one is still retried every round. `tests/amiga_a500_cpu_rate.rs`
runs a two-instruction loop out of a synthetic ROM for 100 ms and asserts the
68000 retired `clk / 4` of it to within one iteration: **709 380 cycles of
709 379 owed** (100.00 %), where the same test on the old board reads 356 292
(50.23 %). The scheduler's own rule changed too — two runnables on one crystal
now each execute its whole rate (`docs/techniques/execution-budgets.md`) —
but the A500 has one runnable on its crystal now, so that change leaves this
board's hashes exactly where this one put them.

What moved: the two animated insert-disk screens (2.04 at 28 s, 3.1 at 12 s),
which show the same picture with the disk at another point of its slide
because the ROM reached the screen sooner; nothing else. The Workbench boots
reach **the same desktops, bit for bit, much sooner**: Workbench 2.04's
picture stops changing at 42 s rather than about 60, and Workbench 1.3's at
68 s rather than about 88, so the tests now stop at 45 s and 72 s instead of
62 s and 90 s. Over the *same* virtual span the host pays about 6 % more for
twice the processor work (12.6-13.0 s → 13.5-13.7 s for 62 s of 2.04, 20.0 s
→ 21.2 s for 90 s of 1.3).

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
| **Reading a write-only custom register** | Appendix B says which registers are readable and nothing about the rest; chapter 6 describes the arbitration, not the bus's electrical state | The word the last chip-bus cycle left on `D15`–`D0` (`dma::ChipDataBus`), **once**: that read is itself a cycle nothing drove, so it takes the word and leaves the lines floating. Agnus's DMA, a register write and a register read a chip answers all drive them; a **refresh** slot is `RAS`-only and transfers no data, so it changes nothing. A board with no DMA engine reads zero. `CustomBus::unclaimed` counts every use. See `dma.rs` for which parts are inference and which are choice |
| **`$DFF200`–`$DFFFFF`** | Appendix D gives a 4 KiB window, the table fills 512 bytes, and whether the rest mirrors is not stated | Only 512 bytes mapped; the rest floats like any empty address. `mirror(custom)` is a one-word change |
| **A0–A7 in a CIA window** | The notation gives one hex digit of register select; nothing says whether the low byte is decoded | Not decoded — `$BFE003` is register 0 |

| **Paula's disk bit cell** | "Two microseconds per bit cell" is 7.094 colour clocks, and Paula has no other clock | 7 colour clocks (14 slow): 1.974 µs PAL. The drive spins at the same rate, so a track reads back as written |
| **The drive's spindle speed and index width** | Not in the manual | 100,000 cells a revolution (300 rpm at 2 µs), an index pulse of 1,000 cells |
| **`SERDATR`'s `TBE`** | Chapter 8 says "not a mirror" of `INTREQ`'s; Appendix A says "mirror" | The chapter: the transmit buffer's own state |
| **`MSBSYNC`, precompensation** | One sentence each, and precomp is analogue | Stored and not acted on |
| **`DSKSYNC` out of reset** | The manual does not say | Zero — which matches an idle, zero read line on every cell, so `DSKSYN` is requested until software loads a sync word |
| **Audio's state diagram** | Figure 5-8's arrows are not legible in the available scan | The chapter's prose: see `paula.rs` |
| **The sector checksum's arithmetic** | Appendix C of the RKRM *Devices* names both fields and gives no formula | An XOR of the region's MFM longwords, data cells only — pinned down against Kickstart's own `trackdisk.device`; see *Disks* |

## Disks

DF0 takes an **ADF** — AmigaDOS's 1760 sectors back to back, 901 120 bytes —
and encodes it into the raw MFM tracks a drive head presents, because an Amiga
never reads a sector: `trackdisk.device` has Paula DMA a whole track into chip
RAM and finds the sectors in software. `src/dev/amiga/adf.rs` has the codec.

**The track**, from the RKRM *Devices*, Appendix C ("Commodore-Amiga Disk
Format", "MFM Track Encoding"): a gap, then eleven sectors with no gaps between
them. Each sector is `$00 $00` (MFM `$AAAA $AAAA`), two `$A1` sync bytes with a
missing clock (`$4489 $4489`), a longword of format `$FF`, track, sector and
sectors-until-the-gap, sixteen bytes of OS recovery info, a longword header
checksum, a longword data checksum, and 512 bytes of data — 1088 bytes of MFM.
Each field is encoded as a block, **all its odd bits first, then all its even
bits**, each data bit behind a clock bit that is set only between two zeroes.
The manual's `$4489` pins the cell order: data in the `$5555` cells. Eleven
sectors are 11 968 bytes of the drive's 12 500-byte revolution; the gap is the
other 532, written first, the order of the manual's "first-ever write".

**The checksums.** Appendix C does not give the arithmetic, and neither does any
other Commodore document found (the 1.3 RKRM *Libraries and Devices*, the
AmigaOS wiki's trackdisk chapter). What is used: the XOR of the region's
longwords **as encoded on the disk**, masked to the data cells, `$5555_5555` —
the header sum over the format longword and the recovery info, the data sum over
the 512 bytes. That it is an XOR of 32-bit chunks is how it is publicly described
(techtravels.org, 2010, a hardware project); which chunks and which mask were
established **black-box against Kickstart**: 2.04 boots a Workbench ADF encoded
this way, and refuses the same disk with one cell flipped in every data sum, or
in every header sum (`tests/amiga_adf.rs`, behind `RSEMU_AMIGA_ROM_DIR` and
`RSEMU_AMIGA_ADF_DIR`). No emulator's source or output was consulted.

**Where a write goes** is decided by the run, never by the board, and on the
line `--hd0` and `--drive hd0=` already draw for a hard disk:

| Given as | The guest's writes |
| --- | --- |
| `--media df0=disk.adf`, `--media df0=adf:…` | Land in the session's tracks and its snapshots. The file is never touched: it may be the user's only copy of a Workbench disk, and an ADF inside a disc image cannot be written at all |
| `--drive df0=disk.adf` (`,ro` to protect it) | Go back to the file. A written track is decoded into sectors when the head leaves it, when the motor stops, and at every flush; each sector that decodes goes to its ADF offset. One that does not — a track written in some other format — cannot be said in an ADF, so the file keeps what it had and the flush fails naming it |

**The media syntax** follows `kickstart:`:

```
rsemu run <board> --media df0=disk.adf                         a plain path works
rsemu run <board> --media df0=adf:disk.adf                     checked, with a note
rsemu run <board> --media df0=adf:<dvd.iso>,disk=<name>        out of Amiga Forever
rsemu run <board> --drive df0=disk.adf                          writes go to the file
```

`adf:` (feature `media-adf`) checks the length, refuses a high-density image by
name, and says whether the boot block will boot (RKRM *Devices*, Appendix C:
`DOS` type, "an additive carry wraparound sum of 0xffffffff"). `disk=` resolves
in `/Amiga Files/Shared/adf/`, `.adf` optional; a disc with no `disk=` lists
what it holds. A raw MFM dump of the drive's own layout (160 × 12 500 bytes)
still works, told apart by length.

**An empty drive is the default.** A named media slot must be bound, so
`rsemu run` and the wasm front end bind `df0` to no bytes when nobody names a
disk — the same list, and the same argument, as a PC's `floppy` — and
`amiga.floppy` reads no bytes as no disk. The shipped `amiga-a500.machine`
names the slot, so every test that builds the board binds it — to no bytes for
an empty drive.

### How far a real disk gets

**Kickstart 2.04 boots the Workbench 2.04 disk to its desktop**, both files
read in place: the boot block runs, AmigaDOS reads the root block on cylinder
40, the head works across the disk for about thirty-five virtual seconds, the shell
window prints "Amiga Release 2. Kickstart 37.175, Workbench 37.67", and
`LoadWB` opens the Workbench window with its Ram Disk and Workbench2.0 icons.
`tests/amiga_a500_kickstart.rs` hashes that picture at 45 s. Workbench 1.3 on
the same Kickstart reaches its own desktop the same way. No write reaches
either disk.

The alert that used to end this run at about thirty-eight seconds —
`$81000005`, `AN_MemCorrupt` — was the blitter's, not the drive's: see *Real
Kickstarts* below.

**Kickstart 1.3 with a disk in the drive does not start the motor at all**, and
sits at its insert-disk screen. 2.04 on the same board and the same disk boots
it, so the drive answers the lines trackdisk drives; whatever 1.3 asks for
first, it does not get. That is the next disk question.

Two things stood between Kickstart and the drive:

* **Paula's disk queue lost words.** Agnus serves the disk slot once a line
  (227 counts) and a `FAST` word is 112, so a third word sometimes finishes
  before the slot; a two-word read queue dropped it, which shifted every
  sector after it and failed every checksum on every disk. Reads now wait for
  Agnus however many there are, and a write keeps six words in hand, because a
  one-word queue ran dry just as the slot arrived and left a hole in the track.
* **The 8520's one-shot start.** "In one-shot mode, a write to timer-high ...
  will transfer the timer latch to the counter and initiate counting regardless
  of the start bit" (Appendix F). `timer.device` calibrates against the TOD
  clock with exactly that write, and without it Kickstart waited forever.
* **Byte access to a custom register.** Kickstart 2.04 reads `$DFF07D` as a
  byte and 2.05 `$DFF006`, and the decode refused a byte access, which was a
  bus error and a dead-end alert `$80000002` before `trackdisk.device` touched
  a drive. `amiga.custom` answers them now — see *A byte access to a custom
  register* below — and the test-local window that stood in for it is gone.

## An address with nothing at it floats; it does not fault

A real Kickstart 2.04 and the AROS ROM both halted on this board before their
first chip register. Each sums its own image and then makes a **word read at
`$F0_0000`** with `OVL` still up; under `unassigned = fault` that became a bus
error whose vector came out of the ROM overlay, and the processor double-faulted.

| Source | What it says |
| --- | --- |
| HRM Appendix D, p. 314 | `$F0_0000`–`$FB_FFFF` "Reserved. Do not use."; likewise `$10_0000`–`$1F_FFFF`, `$A0_0000`–`$BE_FFFF`, `$C0_0000`–`$DF_EFFF` around slow RAM and the clock, `$E0_0000`–`$E7_FFFF`. The A3000 map (p. 315) calls `$F0_0000` "Diagnostic ROM (Reserved)" |
| MC68000UM §5.4 | `BERR` is asserted by "external circuitry" a board may provide |
| HRM Appendix K, p. 397, `/DTACK` | "If a Zorro II slave does nothing, this /DTACK will be driven by the bus controller with no wait states" |
| HRM Appendix K, pp. 393–394, `/BERR` | driven by the controller on "a detected bus collision or DMA error" |
| HRM Appendix E, 86-pin expansion connector | the A500 carries the same `/DTACK`, `/OVR` and `RDY` as the A2000 |

So on an A500 every Reserved range, every unfitted memory range (`$08_0000` on a
512 KiB machine, the clock) and every empty autoconfig slot completes
its cycle with nothing driving the data bus, and nothing documented raises
`/BERR` for an empty address. No manual gives a floating value.
**`unassigned = open-bus`** says exactly that; the 68000 core has no data-bus
latch, so it reads zero, which is also what makes each of Kickstart's probes —
diagnostic ROM, autoconfig, the second half of chip RAM — conclude "nothing
fitted". `tests/amiga_a500_unassigned.rs` walks every range and runs the
user's own ROMs in place behind `RSEMU_AMIGA_ROM_DIR`.

Slow RAM was on that list and should not have been: bank 6 is decoded whether
or not a card is fitted. See the next section.

## Bank 6: the trapdoor RAM, or the chip registers again

`$C0_0000`–`$D7_FFFF` used to float like a hole. It is not one. What Gary does
there is fixed by three Commodore documents between them:

| Source | What it says |
| --- | --- |
| *Gary Specification*, address decoding and `NEXP` | `$C0_0000`–`$DF_FFFF` is `BANK6`; `$C0_0000`–`$C7_FFFF` is `ERAM`, "expansion RAM". `NEXP` "is externally pulled up. The expansion ram card grounds this line. This signal is used in the generation of NRAME" |
| TRM §7.3, the A2000 `PALEN` equations | `/RGAE`, "Amiga chip register address decode", is asserted for `$C0_0000`–`$CF_FFFF`, `$D0_0000`–`$D7_FFFF` and `$DC_0000`–`$DF_FFFF` |
| TRM Table 6-1, Fat Agnus | "the processor uses A1 to A8 to access one of the device registers" — nothing above `A8` reaches the register-address bus |
| TRM, clock section, "Clock Warning" | "The addresses used by the real time clock chip access the custom chip registers without the memory expansion/real time clock module" |
| TRM, on the A500 | "memory at $C00000 is 'slow' RAM (the processor is locked out by the custom chips)" |

So:

* **No card** (`-p slow-ram=0`, the default): all 1.5 MiB of the bank is the
  chip registers, repeated every 512 bytes. A word written to `$C0_F09A` is a
  write to `INTENA`, and `$C0_001C` reads `INTENAR`. Nothing there floats.
* **An A501** (`-p slow-ram=512K`): the card's 512 KiB answers
  `$C0_0000`–`$C7_FFFF`, and the megabyte above it is the registers as before.
  The RAM is Gary's, not a `ram` object's, because a stock board has none and
  an object cannot have no bytes; it is saved in Gary's snapshot chunk and
  cleared by a cold reset. No DMA pointer reaches it, and bus contention — the
  "slow" — is not modelled on this board at all.
* **Anything else is refused.** Gary's `ERAM` is 512 KiB; bigger trapdoor
  cards brought their own decode, which this board does not model.

The same `/RGAE` term covers `$DC_0000`–`$DF_FFFF`, but on an A500 the clock
that shares that range arrives on the same card, and which of the two answers
there with a card fitted is not in any of these documents. So that range is
left as it was: `$DF_F000`–`$DF_F1FF` is the register file and the rest
floats.

Kickstart's own probe agrees with both halves. With no card it now finds the
registers repeating where it used to find a floating bus, and still concludes
there is no RAM: **no Amiga golden moved**, not by a bit. With an A501,
Kickstart 1.3's Workbench title bar reads "889256 free memory" instead of
"365000" — 524 256 bytes more, the card less a 32-byte header — and AROS puts
`ExecBase` at `$C0_0560`, in the card, which is what the TRM says the RAM is
for ("when ExecBase is transferred to $C00000"). `tests/amiga_a500_unassigned.rs`
asserts the decode with synthetic firmware; `tests/amiga_a500_kickstart.rs`
the free-memory count, behind `RSEMU_AMIGA_ROM_DIR` and `RSEMU_AMIGA_ADF_DIR`.

## `$E0_0000`: a window for AROS's second ROM half

**A real A500 has no ROM socket at `$E0_0000`.** Gary's `NROM` term does
decode `$E0_0000`–`$E7_FFFF`, but it selects the one Kickstart part — which,
carrying `A1`–`A18`, would repeat itself there. That repeat is **not**
modelled: an empty `ext` slot must leave the board exactly what it was, and it
does (every golden above is unchanged).

The window exists because AROS's Amiga ROM comes in two 512 KiB halves, and the
second — the graphics library among it — is built to answer at `$E0_0000`.
`amiga.gary` publishes an `ext` region there, bound to the `ext` media slot:

* **no bytes** (what `rsemu run`, the wasm front end and every test bind when
  nobody names one) is an empty container, so every access falls through to
  the space's open bus exactly as before;
* **an image** is a read-only ROM, repeated through the window if smaller (a
  power of two up to 512 KiB), writes dropped.

`kickstart:` decodes it the way it decodes the main ROM — keyed or plain, out of
a file or an Amiga Forever disc image — so a keyed extended ROM works in this
slot too (`amiga-os-130-a570-ext.rom`, 256 KiB and keyed, decodes and mirrors
through the window).

With it, both ROMs go straight from the `$F0_0000` probe to CIA-A. Kickstart
2.04's next refused access was a **byte** read of `$DF_F07D`; see the next
section.

## A byte access to a custom register

Kickstart 2.04 reads `$DF_F07D` as a byte — the low half of `DENISEID`, which
an original Denise does not have. Appendix B lists only words, so the decode
used to refuse it. What the hardware does is settled by three documents:

| Source | What it says |
| --- | --- |
| HRM Appendix J, pin allocation | Denise and Paula have `D15`–`D0` and `RGA8`–`RGA1` and **no** `UDS`, `LDS` or `R/W` pin |
| TRM Table 6-1, Fat Agnus | `LDS*`/`UDS*` are "enabled only during a processor DRAM access" and pick `CASL*`/`CASU*`; a register access is `AS*` + `RGEN*` with `A1`–`A8` |
| A500 schematic #312511-02, sheet 2 | CPU→chip data through two 74LS244s (`U12` upper, `U10` lower) whose enables are both Gary's `_OEB`; chip→CPU through two 74LS373s (`U13`, `U11`) sharing `_OEL` and `_LATCH`. TRM §7.3's A2000 PAL enables the same buffers from the register decode alone |
| MC68000UM Table 3-1 | a byte **write** with only `LDS` drives bits 7–0 on `D15`–`D8` *and* `D7`–`D0`; with only `UDS`, bits 15–8 on both halves ("a result of current implementation") |
| MC68000UM §5.1.1 | on a byte **read** "the processor internally positions the byte appropriately" |

So, in `amiga.custom`:

* **A byte read** is a full word read of the register, with all of a word
  read's side effects (the chip cannot know only half was wanted), and the
  processor keeps the upper byte at an even address or the lower at an odd one.
  `$DF_F07D` on this board therefore answers the low half of the chip data
  bus, which is Appendix C's "whatever value is left over on the bus from the
  last cycle" (p. 299) — Agnus's last DMA cycle, not the processor's own last
  write.
* **A byte write** stores **the same byte in both halves** of the register,
  at either address: `MOVE.B #$20,$DFF09B` writes `$2020`. No half is kept.
* A word at an odd offset, and anything wider than a word, are still refused.

## A 256 KiB Kickstart

`-p kickstart-size=256K` used to fail to build: `amiga.gary` refused a ROM
smaller than its window at zero. On the board the ROM **repeats**:

| Source | What it says |
| --- | --- |
| TRM §7.3, A2000 PAL `/ROME` | the ROM is selected for a read at `$F8_0000`–`$FF_FFFF`, and at `$00_0000`–`$07_FFFF` while `OVL` is high — `A19`–`A23` only |
| A500 schematic #312511-03 rev. 6A/7, sheet 3 | `U6` has `A0`–`A16` on `A1`–`A17` and the processor's `A18` on pin 1; `/CS` grounded, `/OE` on `_ROMEN`. Rev. 5 fits a "128K × 16" part in the same socket |
| HRM Appendix D and p. 223 | Kickstart 1.x lives at `$FC_0000`; "an additional copy of the system ROM responds starting at memory location $00000000" |

A 256 KiB part has no pin for `A18`, so it answers in both halves of the
512 KiB select — which is the only way 1.x, built for `$FC_0000` (`A18` high),
can also serve the reset vector at `$00_0000` (`A18` low). `amiga.gary` now
repeats the ROM through the first 512 KiB of its window and floats beyond that
(a 1 MiB chip-RAM board with `OVL` up: the PAL selects neither ROM nor RAM
there). The board's own ROM mapping wants the same shape:

```
map mem 0xF80000 size 512K = mirror(kick) { endian = "big" }
```

in place of `map mem rom-base size kickstart-size = kick`, and `param rom-base`
is gone. That is what the shipped board now says, so `-p kickstart-size=256K`
is the whole of a Kickstart 1.x configuration.

## Real Kickstarts, as far as each gets

`tests/amiga_a500_kickstart.rs`, behind `RSEMU_AMIGA_ROM_DIR`, runs the user's
ROMs in place on the shipped board and checks a hash of Denise's picture;
with `RSEMU_AMIGA_ADF_DIR` as well it boots the Workbench 1.3 and 2.04 disks.
`RSEMU_AMIGA_FRAME_DIR` writes the frames out as PNGs.

| ROM | Reaches | What is on screen |
| --- | --- | --- |
| Kickstart 1.3 (34.5, 256 KiB) | its insert-disk screen | White background; a black-outlined hand holding a blue-violet 3½" disk with a grey shutter; the label reads "AMIGA Workbench" upside down, as the disk is held; "V1.3" beside it. Static. No pointer |
| Kickstart 2.04 (37.175) | its insert-disk screen | Dark purple background; the rainbow check mark top left; "2.0 Roms (37.175) / Copyright © 1985-1991 / Commodore-Amiga, Inc. / All Rights Reserved" in salmon; a salmon drive with a black slot, and a blue disk with a grey shutter and white label below it, animating into the drive. No pointer. The colours are the ones the ROM's own copper list loads |
| Kickstart 3.1 (40.063, A500/A600/A2000) | its insert-disk screen | The same picture with "3.1 ROM 40.063 / Copyright © 1985-1993 / Commodore-Amiga, Inc. / All Rights Reserved." |
| Kickstart 2.04 + the Workbench 2.04 disk | the Workbench desktop, 42 s | A grey 640-pixel high-resolution screen; a black screen title bar reading "Copyright © 1985-1991 Commodore-Amiga, Inc. All Rights Reserved" with the red pointer over its first letters; below it the blue-titled "Workbench" window, its Ram Disk and Workbench2.0 icons, both scroll bars and the sizing gadget. The busy pointer appears while the disk is read. Nothing is out of place |
| Kickstart 1.3 + the Workbench 1.3 disk | the Workbench desktop, 68 s | A plain blue 640-pixel high-resolution screen; a white screen title bar reading "Workbench release." and "365000 free memory", with the red pointer over its first letters; the RAM DISK and Workbench1.3 icons down the right-hand edge. The AmigaDOS shell the startup-sequence opens, then `[CLI 2]` — `LoadWB` — on the way |
| Kickstart 1.3 + Workbench 1.3 + an A501 (`-p slow-ram=512K`) | the Workbench desktop, 72 s | The same desktop, the title bar reading "889256 free memory" |
| AROS (2025-04-22), both ROM halves, its boot disk, `-p chip-ram=1M -p slow-ram=512K` | the `Workbook` desktop, 80 s | See *AROS* below |

### AROS

AROS runs on the shipped board from its own boot disk, with its second ROM
half in the `ext` window:

```
rsemu run amiga-a500 -p chip-ram=1M -p slow-ram=512K \
    --media kickstart=kickstart:<rom dir>/aros-20250422.rom \
    --media ext=kickstart:<rom dir>/aros-20250422-ext.rom \
    --media df0=adf:<adf dir>/aros-20250422-boot.adf --vnc :5900
```

Or all three straight out of the Amiga Forever disc image, nothing extracted:
`--media kickstart=kickstart:<dvd.iso>,rom=aros-20250422`,
`--media ext=kickstart:<dvd.iso>,rom=aros-20250422-ext` and
`--media df0=adf:<dvd.iso>,disk=aros-20250422-boot`. Add `--headless --for 90s
--screenshot aros.png` for a picture without a VNC client; the requester in
step 3 then stays up, because nobody is there to answer it. What it does, measured
in `tests/amiga_a500_kickstart.rs`:

1. Its serial log reports a "1MiB ROM" in two regions, `$E0_0000` and
   `$F8_0000`, and finds three memories: the card at `$C0_0000` (type
   `$1705`), chip RAM from `$400` to `$10_0000`, and the two ROMs.
2. It boots the disk: the grey screen, a blue-framed "AROS" shell window with
   the copyright, licence, version and build-date lines.
3. By 55 s a **"System requester"** sits over it: `Please insert volume "AROS
   Live CD" in any drive`, `Retry` and `Cancel`. The disk's startup-sequence
   asks `If EXISTS "AROS Live CD:"` — the CD Amiga Forever pairs this disk
   with, for which the board has no drive. That is AROS asking, not the
   hardware stopping, and it waits for a person.
4. **Cancel** — Left-Amiga+B on the keyboard, which is what the test types —
   and the startup-sequence carries on to `LoadWB` and `EndCLI`. By 80 s the
   shell window is gone and the **`Workbook` desktop** is up: a title bar
   reading "Workbook 1.0  Chip: 634k, Fast: 0k, Any: 634k" (the card's RAM is
   counted as `Fast` and is full: AROS allocates from it first), the "AROS
   Kickstart" disk icon and the "RAM Disk" icon, the red pointer. It does not
   change after that, except that the title bar is cleared and redrawn about
   every thirty seconds.

**How much memory it needs.** With 512 KiB in all AROS runs out before it
draws anything (the grey screen it has always stopped at here). With 1 MiB in
all — `chip-ram=1M`, or 512 KiB of chip and an A501 — it gets as far as
"Workbook 1.0" in the title bar and the shell window never closes. That is not
the hardware either: read through the RKRM's `ExecBase`, `Task` and
`MemHeader` layouts, every task is waiting on exactly the signals it waits on
in the run that succeeds, and chip RAM has 127 KiB free in pieces of at most
26 KiB. The run with 512 KiB more reaches the desktop with the same tasks in
the same states. So `chip-ram=1M` plus an A501 is the configuration the test
uses; it is not one Commodore sold (an A500 with a 1 MiB Agnus used the
trapdoor for the second half of chip RAM), and the page says so rather than
pretend.

**What stood in the way before.** Without the window, the main ROM alone
raised "graphics.library could not open library hidd" (`$C2038002`) over and
over, because the graphics code is in the other half. A scratch board with
that half mapped used to stop at a plain grey screen whenever the keyboard was
wired — the "lost wake-up" earlier revisions of this page chased through the
8520's serial port and the keyboard's handshake. It was the 68000 running at
half its clock (see *The 68000 gets its whole clock*): with its whole clock
AROS boots with the keyboard wired and reads keys through it, as step 4 shows.
No defect in the chips was found on the way to the desktop. The one board
defect found on the way was bank 6's decode (above): it floated where Gary
decodes either the card's RAM or the chip registers, so there was no card to
give AROS the memory it needs.

What it took to get the Kickstarts there, beyond the byte access and the ROM
mirror:

* **CIA one-shot start** (`src/dev/mos/cia.rs`, landed with the ADF work):
  "In one-shot mode, a write to timer-high ... will transfer the timer latch to
  the counter and initiate counting regardless of the start bit" (HRM Appendix
  F; the TRM says the same). 2.04's and 3.1's graphics library times its
  genlock probe that way, and `timer.device` calibrates against TOD with it;
  without it Kickstart waited forever on a grey screen. The same write also
  raises a toggle-mode `PB6`/`PB7`, because "the toggle output is set high
  whenever the timer is started".
* **Blitter line mode steps D by `BLTCMOD`** (`src/dev/amiga/agnus/blitter.rs`).
  Kickstart's line routine loads `BLTCMOD`, `BLTCPT` and `BLTDPT` and never
  `BLTDMOD`; after an area fill left `BLTDMOD` at −4, stepping D by it walked a
  line up through an allocation header and exec stopped 3.1 with
  `AN_MemCorrupt`. Appendix A has software load both modulos alike, so no
  conforming program can tell the difference.
* **The high-resolution bitplane fetch counts eight-count blocks**
  (`src/dev/amiga/agnus/display.rs`). **Inference from firmware, against the
  manual.** Chapter 3's high-resolution formula and table 3-14's "49 words"
  said 41 words for the `DDFSTRT $38`, `DDFSTOP $D8` window Kickstart 2.04's
  Workbench screen uses, and the screen's own modulos of −4 on an 80-byte row
  say it expects 42; AROS's `$3C`–`$D0` screen says 40 where the formula says
  39. The desktop came out sheared a word a line, AROS's picture sheared the
  other way. Counting eight-count blocks, two words each in high resolution,
  gives 42 and 40 and the manual's own 40 and 20 at the standard windows — and
  50, not 49, at the `$18`–`$D8` limit, which is where it departs from the
  book. Both pictures are square with it.
* **The head steps on the trailing edge of `STEP*`**
  (`src/dev/amiga/floppy.rs`). Appendix E names no edge — the edge belongs to
  the drive, and an Amiga's is an ordinary Shugart-compatible 3.5-inch
  mechanism, whose interface has said since the SA400 that "the access motion
  is initiated on the trailing edge of the step pulse". It matters because the
  two Kickstarts drive the line differently: 2.04 asserts `SEL0*` and *then*
  pulses `STEP*` inside the selected window, where either edge would do, but
  1.3 deselects between pulses and asserts `SEL0*`, `DIR` and `STEP*` in one
  `PRB` write, so its leading edge lands on the instant of selection. Stepping
  on the leading edge dropped every 1.x step: the head never left cylinder 0,
  the change flop — "reset when drive is selected and the head stepped, but
  only if a disk is installed" — was never reset, `trackdisk` read the drive as
  empty and never started the motor, and 1.3 sat on its insert-disk screen with
  a disk in DF0.
* **A refused copper `MOVE` halts the copper** until its next restart
  (`src/dev/amiga/agnus/copper.rs`). Inference from firmware: 1.3 loads a
  `View` with no copper list, which sends the copper into `ExecBase`, and AROS
  leaves `COP2LC` at zero; both "lists" begin with a `MOVE` to `$000`, and a
  copper that carried on went on to clear `INTENA` bits and stop the system.

## What each chip will need

## How the chips meet

* **Agnus, Denise** — implement `CustomChip` on their register block, take a
  `custom = <object>` property, and call `CustomBus::attach` from `bind`. The
  copper writes through the same bus with `Origin::copper(danger)`.
  `src/dev/amiga/custom.rs` has the contract in full.
* **The 8520s** — `wire cia_a.pa0 -> gary.ovl`, an `amiga.cia-decode` per chip,
  their interrupts through Paula, and their TOD inputs from Agnus:
  `wire agnus.vsync -> cia_a.tod`, `wire agnus.hsync -> cia_b.tod`.
* **Agnus** — `paula = paula` and `video = denise`; see *Agnus* below.

## Where the table departs from Appendix B

| Register | Appendix B | The table | Why |
| --- | --- | --- | --- |
| `DIWSTRT` `$08E`, `DIWSTOP` `$090` | `A` | `A D` | Appendix C, "Display Window Specification", prints both `W A D`; the window's horizontal resolution is one low-resolution pixel, which only the chip that serializes pixels can compare against. Without it Denise could not clip its own output |

`SPRHDAT` at `$078` and `NO-OP(NULL)` at `$1FE` are in Appendix B and were
missing from the first transcription; both are rows now. A write to `NO-OP` is
dropped and is **not** counted as unclaimed, because it is the address copper
lists pad with.

## Denise

`amiga.denise` is the colour table, single and dual playfields, hold-and-modify
and extra-half-brite, `BPLCON1` scrolling, the eight sprites with attachment and
`BPLCON2` priority, and `CLXCON`/`CLXDAT` collisions. `src/dev/amiga/denise.rs`
has the full list of what is modelled and what only latches.

**It is driven, not clocked.** Denise has no vertical counter and no memory bus
(Appendix J), so the chip that counts the beam pushes lines into it through the
`ExportId::AMIGA_VIDEO` handle:

| Call | When | Carries |
| --- | --- | --- |
| `Video::line(&Line)` | as the beam leaves each line | `vpos`; the line's length in colour clocks; the bitplane words fetched for planes 1–6 and the `hpos` the first was fetched at |
| `Video::field(lof)` | as the vertical counter wraps | the new field's long-frame bit |
| `Video::connect_beam(Arc<dyn Beam>)` | once, at bind | a lock-free `position()` so a mid-line register write lands mid-line |

Sprite DMA is **not** part of that seam: `SPRxPOS`/`CTL`/`DATA`/`DATB` arrive as
register writes with `Origin::dma()`, because arming and disarming are their
side effects and manual-mode sprites take the identical path.

The coordinate rules are the manual's: a pixel on screen at `x = 2 × hpos`, a
word fetched at `hpos = D` first displayed at `x = 2D + 17` (low resolution) or
`2D + 9` (high), from Chapter 3's "$81/2 − 8.5 = $38" and "$81/2 − 4.5 = $3C".

**It is its own `Scanout`, not a `Panel`.** A panel counts content changes and
has no frame rate; Denise emits a field every 20 ms and the host must step by
one. `host::display::amiga` reports `Video::fields()` as the frame counter and a
frame period of the last field's colour clocks × 2 ticks of Denise's `clock`
domain (its 7M pin).

**On the A500** Denise is `object denise "amiga.denise" { custom = custom,
clock = clk / 4 }`, and Agnus names it with `video = denise`.

| Open | What this model does |
| --- | --- |
| Which pixel hold-and-modify holds at the start of a line | `COLOR00`, then whatever the serialized bits say |
| `PF1P`/`PF2P` values 5–7 (Table 7-2 defines 0–4) | the same `group < code` comparison |
| A sprite in front of one playfield and behind the other | a playfield a sprite is in front of is removed, then `PF2PRI` picks (Chapter 7's example) |
| Collisions outside the display window | not detected |
| Interlaced field order | long field on even rows, short on odd |
| Where horizontal blanking falls in the 368 visible pixels | not cut; the picture is 400 low-resolution pixels from `x = 64` |
| The mouse inputs' `CCK`/`CCK*` multiplexing | not modelled: eight input pins, one per connector signal; see *Keyboard and mouse* |

## Agnus

`amiga.agnus`, feature `dev-amiga-agnus`. The module documentation in
`src/dev/amiga/agnus/` is the long form; this is what a board author needs.

**Time is the colour clock.** One tick of the chip's domain is one count of
the horizontal beam counter — "3,546,895 Hz" PAL, "3,579,545 Hz" NTSC
(chapter 2) — so an A500 gives it `clock = clk / 8` of its 28.37516 or
28.63636 MHz crystal. The processor is `clk / 4` and the CIAs' E clock
`clk / 40`: integer ratios in one tree, and the line and field lengths are
counted rather than derived from a duration.

**Counters.** PAL fields of 312 or 313 lines of 227 counts; NTSC 262 or 263
with lines alternating 227 and 228. `LOF` toggles under `BPLCON0`'s `LACE`
and holds otherwise, and comes out of reset set — the manual gives no reset
value, but its last beam position "(226,312)" and "PAL line counts (313)" are
both a long field's. `VPOSR` carries the Agnus identification from Appendix C:
`$00` PAL, `$10` NTSC.

**Pins.**

| pin | rising edge | wire it to |
| --- | --- | --- |
| `vsync` | line 0, count 0 — "start of vertical blank" | CIA-A's `tod` |
| `hsync` | count 0 of each line | CIA-B's `tod` |
| `blit` | the count a blit finishes on (one count wide) | nothing on an A500: Paula hears `BLIT` through its seam |

The widths (three lines, seventeen counts) are nominal: the original chip set's
sync placement is not in the manual.

**What it drives into the other chips.**

* The copper writes through `amiga.custom` with `Origin::copper(cdang)`.
* Neither Paula nor Denise reads chip RAM; Agnus pushes into both.
* **Denise** (`video = denise`, `ExportId::AMIGA_VIDEO`): each line's
  bitplane words through `Video::line` as the beam leaves the line, each field
  through `Video::field` before any position in it is reported, sprite control
  and data words written into `SPRxPOS`/`SPRxCTL`/`SPRxDATA`/`SPRxDATB` through
  the bus with `Origin::dma()`, and Agnus's beam handed over with
  `Video::connect_beam` so a mid-line write lands on its pixel.
* **Paula** (`paula = paula`, `ExportId::PAULA`): on each line's first count,
  while `DMAEN` and a disk or audio enable are on, the slot through
  `PaulaPort` — disk words stored at or fetched from `DSKPT`, audio restarts
  from `AUDxLC` and fetches — and `VERTB` and `BLIT` requested on their counts.
* `ChipDma` (`ExportId::CHIP_DMA`, nine) holds what those need without Agnus's
  lock — chip RAM, `DMACON`, `DSKPT`, `AUDxLC` — and is published for tests
  and monitors.

**Where the manual left a choice, and what was chosen.**

| | Choice | Why |
| --- | --- | --- |
| Copper cycle parity | Even horizontal counts | The manual's own loop example waits for `$E2`, the last count of a PAL line |
| Copper restart | Line 0, count 0 | Appendix A's `COPINS`: "at the beginning of each vertical blank time" |
| A `MOVE` to `$00`–`$3E`, or to `$40`–`$7E` without `CDANG` | The copper halts until `COPJMP1`, `COPJMP2` or the next field | Nothing in the manual says; Kickstart 1.3 and AROS both send the copper into memory that starts with such a `MOVE` and boot on real machines. See *Real Kickstarts* above |
| Line-mode D pointer | Steps by `BLTCMOD`, as C does | Appendix A loads both modulos alike; Kickstart's own line routine loads only `BLTCMOD` |
| First sprite control fetch | The first line after table 3-13's vertical blank | The pointers are written "during the vertical blanking interval before the first display" |
| Line-mode texture and `ONEDOT` | `BSH` counts down; the first dot of a row is kept | The manual gives register set-up, not the stepping |
| Bitplane fetch | All of a line's words as it ends | Pointer changes mid-fetch apply to the whole line |
| How many words a high-resolution line fetches | Eight-count blocks, two words each — 42 for `$38`–`$D8`, where chapter 3's formula says 41 | Kickstart 2.04's Workbench screen and AROS both display square only this way; see *Real Kickstarts* |
| Contention | None: no slot is lent or stolen, `BLTPRI` is stored only | The arbitration figure is a sketch, not a timing |
| The copper's write permission | Appendix B's `*`/`~` columns, the original chip set's rule | Appendix C gives ECS a wider one, which `revision = "ecs"` follows; an A500 Agnus is not ECS. See *ECS* below |

## ECS: the Enhanced Chip Set, and the A500+

The A500+, the A600 and the A3000 carry the **Enhanced Chip Set**: an ECS
Agnus (the 1 MiB 8372A, or the 2 MiB 8375) and the 8373 Denise. Both classes
take a `revision` property, `"ocs"` (the default, and every board that existed
before) or `"ecs"`; the Agnus also takes `reach = 1M` (an 8372A, the default)
or `2M` (an 8375). With `ocs` nothing changed: every A500 golden — the
Kickstart, Workbench and AROS screens of `tests/amiga_a500_kickstart.rs`, both
sessions of `tests/amiga_a500_workbench.rs`, the chipset and Denise board
hashes and Denise's own golden field — is the same hash it was.

`machines/amiga-a500plus.machine` (feature `machine-amiga-a500plus`) is the
A500 with an 8375 and an 8373, 1 MiB of chip RAM (`-p chip-ram=2M` is the
trapdoor card that makes it 2 MiB), no `ext` window, and its battery-backed
clock. Kickstart 2.04 is its own ROM.

### Sources

| Source | Covers |
| --- | --- |
| *Amiga Hardware Reference Manual*, 3rd edition, Appendix C ("Enhanced Chip Set") | Everything below unless a row says otherwise: *Determining Chip Revisions* (`VPOSR`'s layout and identifications, `DENISEID`), *SuperHires Mode* and its colour-register table, *SuperHires 70ns Sprite Positioning*, *Multi-Sync and Bi-Sync Monitors* (`HTOTAL`, `VTOTAL`, the sync and blank registers), *New BEAMCON0 Register*, *Display Window Specification* (`DIWHIGH`), *Genlock Extensions* (`BPLCON2`/`BPLCON3`), *Other ECS Modifications*, *Interpretational Differences* (`COPCON`), and the *ECS Registers* table |
| Commodore's register notes for the AA chip set (the `VPOSR` identification table as transcribed at amiga-dev.wikidot.com) | "8372 (Fat-hr) (agnushr), rev 5 = 22 PAL, 31 NTSC" — the identification this model gave the 2 MiB part until Kickstart 3.1 showed the PAL value is the AA specification's `$21`; see [the A1200](#the-guest-sees-aa-and-that-is-a-test-rather-than-a-claim) |
| *MSM6242B* data sheet (Oki Semiconductor) | The battery-backed clock: register table, the functional description of every register, Tables 1 and 2 |
| *ROM Kernel Reference Manual* structure layouts (`exec/execbase.h`, `exec/nodes.h`, `graphics/gfxbase.h`) | Where a test finds `GfxBase->ChipRevBits0` in guest RAM |

### What an ECS Agnus does (`src/dev/amiga/agnus/ecs.rs`)

| | |
| --- | --- |
| `VPOSR` | "LOF I6 … I0 LOL -- -- -- -- v10 v9 V8": `$20` PAL / `$30` NTSC for an 8372A, `$21` / `$31` for an 8375; `LOL`; `V10`/`V9`. `VPOSW` writes `V10`–`V8` |
| `BEAMCON0` | Out of reset `PAL` follows the strap ("the chips from the US factory are configured for NTSC mode … reset the motherboard jumpers") and the rest is clear, so an ECS Agnus counts like its original until told otherwise. `PAL` switches the hardwired counts between 312/313 × 227 and 262/263 × 227/228; `LOLDIS` stops NTSC's long/short toggle |
| Programmable beam | `VARBEAMEN`: `HTOTAL` is the highest count of a line and `VTOTAL` the highest line of a field ("VGA (525 lines, 114.0 colorclocks per scan line)" is `HTOTAL = 113`, `VTOTAL = 524`), with a long field one line longer under `LACE`. Counted, never timed: a field is an integer of colour clocks |
| Sync pins | `VARHSYEN`/`VARVSYEN` move `hsync` to `HSSTRT`–`HSSTOP` and `vsync` to lines `VSSTRT`–`VSSTOP`, so the CIAs' TOD counters count a programmed beam's lines and fields |
| Blanking | `VARVBEN`: sprite DMA starts at `VBSTOP` instead of Table 3-13's line |
| `DIWHIGH` | The vertical fetch window's `V10`–`V8`, once written after `DIWSTRT`/`DIWSTOP` ("If this register is written last in a sequence …"); a later `DIWSTRT`/`DIWSTOP` puts the old scheme back |
| SuperHires fetch | `SHRES` fetches four words per eight-count block, twice high resolution's |
| `COPCON` | "In the ECS, if this bit is set, the Copper can access all of the Amiga chip registers. If this bit is clear, the Copper can access the address range from $DFF03E through $DFF07E" — a second rule in `regs`, and an `ecs` flag on the copper's `Origin` |
| Chip RAM | The pointers' five high bits: an 8372A reaches 1 MiB and an 8375 2 MiB. A smaller RAM repeats through the reach as it does on an original part; a larger one is a build error |
| The raster | Every field, a `denise::Raster`: the first line after vertical blanking and the last before it, and the counts between `HBSTOP` and `HBSTRT` under `VARBEAMEN` (the whole line if those are not in order). Hardwired, it is exactly the original picture |

Held and not acted on: `HCENTER`; `BEAMCON0`'s `HARDDIS`, `LPENDIS`, `CSCBEN`,
`DUAL`, `VARCSYEN`, `BLANKEN` and the three polarity bits. None of them moves a
count or a pin this model has.

### What an 8373 does (`src/dev/amiga/denise.rs`)

| | |
| --- | --- |
| `DENISEID` | `$FFFC`: "$FC in the lower 8 bits"; the reserved upper byte reads as ones here. An 8362 still answers with the chip data bus, which moves with the DMA |
| SuperHires | `SHRES`: 35 ns pixels, two to a high-resolution one. Colours through Appendix C's table: register *n* holds colour `n & 3` in the top two bits of each gun and colour `n >> 2` in the bottom two, so a pair of pixels (*a*, *b*) is register `a | b << 2`, *a* through the top bits and *b* through the bottom; a two-bit gun is shown repeated into four. Sprites the same way through the upper sixteen |
| 70 ns sprites | `SPRxCTL`'s `SHSH1` places a sprite half a low-resolution pixel later in SuperHires |
| `KILLEHB` | Six planes without half-brite |
| `BPLCON3` | `BRDRBLNK`, once `BPLCON0`'s `ENBPLCN3` enables the register: a black border |
| `DIWHIGH` | The window's `H8` and `V10`–`V8` directly, on the same written-last rule as Agnus |
| Genlock | `ZDBPSEL`, `ZDBPEN`, `ZDCTEN`, `BRDNTRAN`: latched only. There is no genlock |

### The picture follows the beam

A hardwired beam's picture is what it always was: 800 high-resolution columns
from `x = 64`, two rows a line from Table 3-13's end of blanking. An ECS
Agnus's `Raster` lays each field out instead, and the host adapter follows:

* **Rows**: two a line (line-doubled, or woven when interlaced) for a 15 kHz
  line; one for a 31 kHz one. The boundary is `denise::DOUBLED_LINE`, 170
  counts, midway between the two families.
* **Columns**: high-resolution pixels, or — while an 8373 has SuperHires on
  screen — SuperHires pixels, four to a low-resolution one. The first
  SuperHires line of a field widens the picture there and then (the rows
  already drawn are repeated, not lost); a field with none narrows it back.
  So a Workbench on an A500+ is 800 × 568, like an A500's, and a SuperHires
  screen is 1600 wide.
* **The frame period** is still the last field's colour clocks × 2 ticks of
  Denise's 7M domain, an exact rational: a productivity field of 525 × 114
  counts on a PAL crystal is 16 873 913 ns. `Video::copy_frame` gives the host
  the picture, its size and its field count in one moment, because the size
  can now change between fields.

Choices, where Appendix C gives the registers and not the picture: the
columns shown under `VARBEAMEN` are the unblanked ones (`HBSTOP`–`HBSTRT`); the
first pixel of a SuperHires word fetched at `D` is at `x = 2D + 5`, chapter 3's
fetch arithmetic (one block and half a count) carried to a two-count block;
SuperHires with dual playfields is decoded as one playfield of two planes.

### The battery-backed clock (`src/dev/amiga/rtc.rs`, `dev-amiga-rtc`)

An Oki MSM6242B at `$DC_0000`, which Appendix D gives the clock and which an
A500 has only on an A501 card. Its address pins are on `A2`–`A5` and its data
pins on `D0`–`D3`, so register *n* is the byte at `$DC_0003 + 4n` and the
64-byte block repeats through the window — confirmed black-box: booting
Workbench 2.04, Kickstart reads `CF`, sets `HOLD`, reads `S1`…`W` a byte each
at exactly those addresses, and clears `HOLD`, the data sheet's own protocol.
BCD date and time with leap years, `HOLD`/`BUSY`, the 30-second adjust, the
interrupt flag and its four periods, `REST`, `STOP` and 12/24 hours. It
starts at `-p time` (default `2026-01-01T00:00:00`) and counts only its own
32 768 Hz crystal, never the host's clock. `STD.P` is not wired; `TEST`'s
fast count is not modelled.

### Kickstart finds the ECS chips

`tests/amiga_a500plus.rs` reads `GfxBase->ChipRevBits0` out of guest RAM —
`ExecBase` at 4, `LibList` at `$17A`, each node's `ln_Name` at 10, and
`gb_ChipRevBits0` at `$EC` — after every ROM run. It is `$03`,
`GFXF_HR_AGNUS | GFXF_HR_DENISE`, for all three. Kickstart 2.04 also writes
`BEAMCON0 = $0020` (PAL, nothing variable) and a `DIWHIGH` into every field's
copper list (watched black-box).

| ROM, on the A500+ | Reaches | `ChipRevBits0` | What is on screen |
| --- | --- | --- | --- |
| Kickstart 2.04 (37.175) | its insert-disk screen, 28 s | `$03` | The purple screen, the rainbow check mark, "2.0 Roms (37.175) / Copyright © 1985-1991 / Commodore-Amiga, Inc. / All Rights Reserved", the salmon drive and the blue disk mid-animation. **Bit for bit the A500's picture**: the explicit window graphics.library sets through `DIWHIGH` is the one the original scheme gave |
| Kickstart 3.1 (40.063) | its insert-disk screen, 12 s | `$03` | The same with "3.1 ROM 40.063 / Copyright © 1985-1993"; bit for bit the A500's |
| Kickstart 2.04 + Workbench 2.04 | the desktop, 45 s | `$03` | The grey 640-pixel desktop, the copyright in the screen title bar under the red pointer, the "Workbench" window with the Ram Disk and Workbench2.0 icons; bit for bit the A500's. The clock was read on the way |

**ScreenMode, through a person's hands.** The same test file opens the
Workbench2.0 disk, the Prefs drawer and ScreenMode on both boards, through the
input seam a VNC client drives. The 500+'s screen title reads "811288 graphics
mem" (the A500's "287248": Exec found the megabyte), and its ScreenMode window
lists PAL:Hires, PAL:SuperHires, PAL:Hires-Interlaced and
PAL:SuperHires-Interlaced with **"Max Size 16368 x 16384"** — the big blits,
"provided for all graphics functions if the ECS Agnus is present". The A500's
window lists **PAL:Hires and PAL:Hires-Interlaced only**, and **"Max Size
1008 x 1024"**, the original blitter's. Productivity is not on either list;
the session installs no monitor driver, and whether the stock disk would bring
one up is not what it checks.

**How Kickstart tells the two Denises apart, and what it cost to get right.**
Watched black-box, Kickstart 2.04 reads `$DFF07C` **seventeen times in a row**
with no other access between them. That is a stability test, and it is the one
Appendix C sets up: an 8373 answers `$FFFC` every time, while on an 8362 "the
original Denise (8362) does not have this register, so whatever value is left
over on the bus from the last cycle will be there" — sixteen lines nobody is
driving, which do not read the same twice.

While `amiga.custom` kept a placeholder word — the last word *written*, and so
stable — graphics.library saw seventeen equal answers, concluded there was an
8373, and set `GFXF_HR_DENISE`: the A500's `ChipRevBits0` came out `$02` and
its ScreenMode offered SuperHires modes an 8362 cannot produce. Modelling the
lines was not enough on its own, twice over: a word that only Agnus's DMA
drove was still `$0000` at that point in the boot, seventeen times; and Denise
*claimed* `$07C` and answered with the lines, which drove the same value
straight back onto them. Both are fixed by saying what the hardware says — an
8362 does not drive `$07C` at all (`CustomChip::drives`), and a read nothing
drives takes the lines rather than leaving them. The A500 now reports `$00`,
neither ECS chip; the A500+ still reports `$03`.

And ROM-free, a hand-assembled program programs a productivity beam
(`HTOTAL = 113`, `VTOTAL = 524`, blanking to 90 counts × 480 lines) and a
two-plane SuperHires window with `DIWHIGH`: the host frame is 720 × 480, the
pixels are the four colours the encoded registers give (white, green, red,
dark blue, repeating a SuperHires pixel each), and the frame period is
16 873 913 ns. The same program on the A500 leaves the field at 313 × 227
counts and the picture at 800 × 568, and shows the planes in low resolution.

### Not modelled

* Genlock, and so `BRDNTRAN`, `ZDBPSEL`/`ZDBPEN`/`ZDCTEN` and `BEAMCON0`'s
  redirection and polarity bits.
* `HCENTER`'s half-line vertical sync in an interlaced field.
* The A2024 and the light pen.
* ECS sprite vertical positions beyond line 511 (the manual gives no `SV9`).
* The host pointer's scale on a SuperHires or a 31 kHz picture.
  `host::input::amiga` moves the mouse a count per framebuffer pixel, which is
  one high-resolution pixel on every picture an A500 draws; on a SuperHires
  picture a framebuffer pixel is half of one, so the guest's pointer moves
  twice as far as the host's cursor there. A Workbench in high resolution — an
  A500+'s default — is unaffected.

## Keyboard and mouse

`amiga.keyboard` (`dev-amiga-keyboard`) and `amiga.mouse` (`dev-amiga-mouse`),
each its own device on its own microsecond clock (`osc periph = 1000000 Hz`),
because both are hardware at the end of a cable timed in microseconds rather
than divisions of the Amiga's crystal.

**The keyboard is its protocol, not its microcontroller.** Appendix G on the
wire: each code rotated so the up/down flag goes last ("6-5-4-3-2-1-0-7"),
active low, KDAT set 20 µs before a 20 µs KCLK pulse and held 20 µs after.
The computer's handshake — "pulsing the SP line low then high" — is latched
from the rising edge of the last clock; with none within 143 ms the keyboard
clocks out single ones until one comes, then sends `$F9` and the code again.
Power-up is the same sync, then `$FD`, the keys held, `$FE`, and the Caps Lock
LED off. Caps Lock sends only when pushed, with the up/down bit saying what the
LED did. Ten codes wait in the type-ahead buffer; an eleventh is lost and `$FA`
follows. No keyboard firmware is modelled or read.

| Where the manual is silent | This model |
| --- | --- |
| When the handshake latch is armed | The last clock's rising edge — when the 8520 interrupts — and the line is sampled as the keyboard lets go of KDAT, to catch a pulse that began while it held a one |
| When the next byte starts | When the handshake pulse has *ended*; starting under it would read the computer's own pulse as a one |
| Self-test | Passes, instantly |
| Power-up sync rate | 143 ms, the resync rule's |
| Which key an overflow loses | The one that arrived to a full buffer |
| How fast host movements reach the controller | One every 5 ms; a host's burst waits rather than overflowing (see *Input*) |
| Host key repeat | A down for a key already down is not a transition, and is dropped |
| Keys moved during power-up sync | Not queued; the power-up stream reports what is held |
| Reset warning, hard reset | Not modelled: "some A1000 and A2000 keyboards", hard reset "valid for all keyboards except the Amiga 500" |

**The 8520's SP and CNT are open drain in output mode**, as Appendix F's
"Bidirectional Feature" says, so the keyboard and CIA-A share KDAT on one
resolved net. **SP's output latch is low out of reset** and holds the last bit
shifted out afterwards ("SP will remain at the level of the last data bit
transmitted"). That is a reading of the data sheet's "all other registers are
reset to zero" rather than a sentence of it, and it is what makes turning
CIA-A's serial port to output — which never transmits a byte — pull KDAT low
for the handshake. **Kickstart confirms it**: 1.3 turns the port round to
answer the keyboard, receives the power-up stream as `SDR` `$04` and `$02`
(`$FD`, `$FE`), and a key typed at its insert-disk screen arrives as `$7F` and
`$7E` (`$40` down and up), each acknowledged. AROS does the same; see *Real
Kickstarts* for what happens to it next.

**The mouse is quadrature transitions, not counter writes.** Host motion
becomes one Gray-code step at a time on the connector's X, XQ, Y and YQ pins,
200 µs apart (5 000 counts a second — 25 in/s at the manual's 200 counts an
inch, 100 counts a PAL field, under the 127 a once-a-field reader can tell from
a wrap), and Denise counts them. Three reasons, all the manual's: the counters
wrap and software reads deltas, so a large host jump written straight in would
read the wrong way; bits 1 and 0 of each counter "may be read to determine the
state of these two clock pins", which a written counter has nowhere to keep;
and a joystick is the same four pins (Table 8-3), so one needs no change to
Denise. Denise decodes a counter's low bits as `(!Q, pin xor Q)` from Appendix
A's joystick table and counts each change against the counter's own low bits,
so a snapshot restores the count whatever order the pins are re-driven in.

| Line | Where | Source |
| --- | --- | --- |
| Left button | CIA-A **PA6**, active low | Appendix E: "PA6..game port 0, pin 6 (fire button\*)"; chapter 8: "Port 1 uses bit 6" (its ports are numbered from 1) |
| Right button | pin 9, `POTGO`/`POTGOR` bit 10 `DATLY` | Chapter 8, "Mouse Buttons"; Table 8-4 |
| Middle button | pin 5, bit 8 `DATLX` | The same |

A button change waits for the motion posted before it, so a click lands where
the pointer was sent. Paula's `POTGOR` now reads a pot pin pulled low as zero
whether the pin is an output or an input ("set both OUT… and DAT… to 1.
Reading POTINP will produce a 0 if the button is pressed").

**Input crosses the record/replay seam** as named host objects —
`amiga-keyboard:keyboard` (one byte a movement, the raw code with bit 7 for a
release) and `amiga-mouse:mouse` (two little-endian `i16` deltas and a button
byte) — and a sealed build refuses either without a channel.
`rsemu run --vnc` records the frontend's keysyms and pointer positions instead
and translates downstream (`host::input::amiga`), so a recording replays through
the same translation.

**The keymap is positional, from the USA keyboard** in Appendix G's matrix
table. A shifted character brings its own shift when the client sent none.
`Control_R` is Ctrl; the Super and Meta keys are the Amiga keys; `Help` and
`Insert` are Help; the Num-Lock-off keypad keys are their keypad positions.
`Home`, `End`, `Page_Up`, `Page_Down`, `F11`, `F12`, `Print`, `Scroll_Lock`,
`Pause`, `Num_Lock` and `Menu` have no Amiga key and send nothing. The keypad's
`(` and `)` and the international `$2B` and `$30` have no PC keysym.

**The picture reaches a person through `--vnc`.** `rsemu run` installs Denise's
capture and serves its `Scanout`; without `--vnc` the terminal attaches to
Paula's UART, the board's one character port.

## Input: a person at the Workbench

`tests/amiga_a500_workbench.rs` uses the booted desktops the way a person
does, through the same seam a VNC client's events cross (`input:vnc` on the
machine's recorder, a `Feed`, `AmigaKeyboardSink` and `AmigaMouseSink`), at
fixed virtual instants, reading nothing back but Denise's pictures. On 2.04 it
double-clicks the Workbench2.0 icon, double-clicks Shell in the window that
opens, types `echo hello` and finds `hello` under it, then replays its own
recording to the same state hash. On 1.3 it does the same with
`echo "Hi, A500!"`, whose shifted characters the keymap supplies. Both need
`RSEMU_AMIGA_ROM_DIR` and `RSEMU_AMIGA_ADF_DIR` and are worth `--release`.
Run by hand with a small RFB client, `rsemu run amiga-a500 --media
kickstart=kickstart:<rom> --media df0=<adf> --vnc 127.0.0.1:5977` does the
same end to end.

**A count is one framebuffer pixel.** Black-box, both Kickstarts at default
preferences move the pointer one high-resolution pixel across and one
interlaced line down per count, with no acceleration at the mouse's 5 000
counts a second. The sink used two pixels a count and the pointer went half as
far as the host's cursor. What a relative mouse cannot share with an absolute
pointer is *position*: the first event only establishes one, Intuition stops
the pointer at the screen's edges while the host carries on, and it confines
the pointer while a window is dragged so the window stays on screen (the
full-screen "Workbench" window cannot move, so dragging it freezes the pointer
— Intuition's rule, not a lost count). The test homes by sweeping past the
screen's top-left corner; a person does the same by eye.

**A host's burst waits for the keyboard.** A VNC client's paste, or a
frontend a slice behind, delivers many key movements at one instant; taken
straight into the ten-code type-ahead buffer the twelfth was lost, often a
release, and Workbench repeated that key until the next was pressed. Host
movements now wait in a backlog and enter the controller one every 5 ms —
faster than anyone types, several times slower than a code crosses the cable
and is answered — so the buffer and `$FA` keep their Appendix G meaning for a
computer that stops answering. Double-click, the right-button menus, dragging
icons, screens and windows, Caps Lock and the operating system's key repeat all
worked without a change.

## A600

`machines/amiga-a600.machine` is the A500's chipset around **Gayle**
(`amiga.gayle`, `src/dev/amiga/gayle.rs`, feature `dev-amiga-gayle`) with an
IDE hard disk, and it boots Workbench from that disk with no floppy:

```
rsemu run amiga-a600 --media kickstart=kickstart:<rom dir>/amiga-os-310-a600.rom \
    --media hd0=<hdf dir>/workbench-311.hdf --vnc :5900
```

`hd0` takes a whole disk — an HDF that begins `RDSK`, with its Rigid Disk
Block — which is what Kickstart's `scsi.device` looks for. `--media` copies it
into the drive; `--drive hd0=` (a `dev-blk` build) writes the guest's changes
back to the file. The drive is `ata.disk`, the same object `pc-at` hangs off
`pc.ide`, unchanged: Gayle is the host adapter, and the split
`src/dev/ata/mod.rs` states holds — `gayle.rs` contains no ATA opcode, no
`IDENTIFY` word index and no status bit, and the drive no register offset.

### Sources

| Source | Covers |
| --- | --- |
| *GAYLE — Gate array for A300/A500+ — Specification*, Commodore, July 10 1991 (a draft; the copy on amigawiki.org) | The pin list (section 1.3: no address pin below `A12`, eight data pins); the ROM and overlay (2.0); the CIA selects (5.0); the RTC select (6.0); the IDE chip selects against `A12`/`A13` (7.0) and the drive address lines on `A2`–`A4` (7.3); reset clears every register (10.0); the four registers at `$DA8000`–`$DAB000` bit by bit (19.0); the memory map (17.0) |
| *A600 System Schematics*, Commodore, schematic #315987 rev. C | Sheet 2: Gayle's data pins on `D15`–`D8`; sheet 7: CIA-A's `PA0` unconnected; sheet 12: the IDE connector `CN16` — `_IDE_CS(1)`/`(2)` on pins 37/38, `A2`/`A3`/`A4` on `DA0`/`DA1`/`DA2`, `_IDE_IRQ` on 31, and the data bus marked "WARNING: BYTE SWAPPED"; `_RTC_CS` on the expansion header; p. 1-1, the A600's parts |
| *A1200/A1200HD Advanced Amiga 1200 System Functional Specification* rev. 1.6, Commodore-Amiga | The 44-pin IDE header's pin names (A3.1), the memory map (5.0) |
| MC68000 User's Manual (M68000UM/AD rev. 8), Table 3-1 | A byte write drives both halves of the data bus |
| Black-box: Kickstart 3.1 (40.063) and 2.05 (37.350) | What they read and write at `$DA0000`–`$DAFFFF` and `$DE1000`, and what they wait on — the identification register, and which CIA write drops the overlay |

No emulator source, no AROS source and no Kickstart disassembly was consulted.

### Gayle's register map, as modelled

| Address | What | Notes |
| --- | --- | --- |
| `$DA0000`–`$DA0FFF`, `$DA2000`–`$DA2FFF` | the drive's command block (`CS1FX-`) | register *n* at `+4n`: data `$DA2000` (16 bits), error/features `$DA2004`, sector count `$DA2008`, sector/LBA low `$DA200C`, cylinder/LBA mid `$DA2010`, cylinder/LBA high `$DA2014`, device/head `$DA2018`, status/command `$DA201C`. `A13` is timing only; `A1` is not decoded |
| `$DA1000`–`$DA1FFF`, `$DA3000`–`$DA3FFF` | the control block (`CS3FX-`) | `$DA3018`: alternate status / device control. The rest floats |
| `$DA4000`–`$DA7FFF` | nothing selected (7.0, "None") | floats |
| `$DA8000` | status | 7 IDE `INTRQ`, 6 card detect, 5 BVD2, 4 BVD1, 3 write enable, 2 BSY/IRQ — each readable, and forced high by writing a 1; 1 digital audio enable, 0 card disable, plain |
| `$DA9000` | change | bits 7–2 latch a change of the matching status line and hold until a 0 is written (a 1 leaves them); 1–0 plain |
| `$DAA000` | enable | 7 IDE → `INT2`, 6 card detect → `INT6`, 5/4 BVD, 3 WR → `INT2`, 2 BSY; 1 and 0 choose `INT6` over `INT2` for BVD and BSY |
| `$DAB000` | configuration | bits 3–0 read back; 7–4, the page registers the draft calls unimplemented, read 0 |
| `$DE1000` | identification | a write restarts it; each read returns the next bit of `$D0` in bit 7 |
| `$BFD000`, `$BFE000` | the two CIA selects | passed through to `amiga.cia-decode`; the first write to either drops the overlay |

Every register is on the even byte (Gayle's eight data pins are the 68000's
`D15`–`D8`) and fills a 4 KiB page, because Gayle sees nothing below `A12`;
the odd byte floats. The IDE port's eight-bit registers are on the even byte
too, and its data word arrives low byte first — the schematic's byte swap —
so an HDF's sectors are in memory in the order the file holds them.

**The IDE interrupt** is Gayle's change latch: `INTRQ` rises, bit 7 of
`$DA9000` latches, and with bit 7 of `$DAA000` set Gayle pulls `INT2`, the net
CIA-A's `/IRQ` is on, and Paula raises `PORTS`. Kickstart's handler reads
`$DA9000`, reads the drive's status (which drops `INTRQ` — a second change,
latched in the same bit), and writes `$7C` to `$DA9000` to let go.

**PCMCIA is out of scope.** With no card, every card line reads negated, which
is what the specification says an empty slot looks like; forcing one through
`$DA8000` still latches a change and interrupts, because that is Gayle's own
logic. The card's windows at `$600000`–`$A5FFFF` are not decoded and float.

### What the board changes from the A500

| | A500 | A600, and why |
| --- | --- | --- |
| Overlay | CIA-A `PA0` into Gary | Gayle's own, dropped by the first CIA write: `PA0` is not connected on the A600 (sheet 7) |
| Chip RAM | 512 KiB | 1 MiB (p. 1-1: "512KB or 1MB internal"; the A600 sold with 1 MiB) |
| Bank 6 (`$C00000`–`$D7FFFF`) | an A501, or the chip registers repeating | floats: no trapdoor slow RAM exists for the A600, and Gayle selects the chip registers at `$DFF000`–`$DFF1FF` only (section 4.0) |
| `$DC0000` | floats (the clock comes on the A501) | floats: the RTC select goes to the expansion header, where an A601 puts a clock |
| `$E00000` | the AROS-only `ext` window | the Kickstart again — see below |
| IDE, Gayle registers, ID | — | `$DA0000`, `$DA8000`, `$DE1000` |

**The ROM answers at `$F80000`, `$E00000` and `$A80000`–`$B7FFFF`.** The
draft specification's ROM select covers all three (section 2.0, and 17.0's
map), and the A600's ROM is a 256K×16 part with no pin for `A19`, so the
Kickstart repeats through each. Black-box it changes nothing tested here:
Kickstart 3.1 boots Workbench 3.1 to a bit-identical frame at the same moment
with the two mirrors mapped or without them. They are mapped because the chip
decodes them, not because anything was seen to need them.

**The chips are ECS**, as a real A600's are: an 8375 Agnus (`reach = 2M`, the
part, whatever the board has soldered on it — an A600 has 1 MiB) and an 8373
Super Denise. The board was built on the OCS models because the ECS ones did
not exist yet, and moved when they landed. Two of its four goldens moved with
it and both pictures were looked at: the insert-disk screen is the same screen
with the disk at a different point of its slide, and the Workbench 1.3 hard
disk draws the same desktop. The Workbench 3.1 and 2.1 desktops did not move
at all.

### Defects found on the way

| Stopped at | Why | Fix |
| --- | --- | --- |
| Kickstart 3.1 looping in its first second, nothing on screen | The overlay was dropped only by a write to CIA-B, as the draft says ("the first write to CIA1 (address range of $BFD000 to $BFDFFF)"). Kickstart's first CIA write is to CIA-A (`$BFE001`), and it uses chip RAM at zero long before it first writes CIA-B (`$BFD200`); with the ROM still over the vector table it never got further. On an A600 the CIA-A write can only reach the overlay through Gayle, so the shipped part must negate it there | Either CIA's first write drops it. Test: `the_overlay_is_up_out_of_reset_and_the_first_write_to_either_cia_drops_it` |
| The insert-disk screen, `$DA0000` never touched | Kickstart writes `$DE1000`, reads it four times, and uses the IDE port only if bit 7 reads 1, 1, 0, 1. Tried against `$0`, `$5`, `$8`, `$9`, `$A`, `$C`, `$E`, `$F` (3.1) and `$0` (2.05): all skip the port; the next four bits (`$D0`, `$D1`, `$DF`) change nothing | The register shifts out `$D0`. Test: `the_id_register_shifts_out_d_msb_first_after_a_write` |

**Found here, fixed in the drive — the drive's, not Gayle's.** `ata.disk`
used to raise `INTRQ` again when the host emptied the last block of a PIO read.
ATA's PIO data-in protocol announces each block with an interrupt *before* it
is transferred and has no completion interrupt (T13 ATA/ATAPI-6 §9.5,
DPIOI1:DI1, and §6.3's "except a PIO data-in command"). On the A600 it cost one
extra level 2 interrupt per read, which `scsi.device` handled — its handler
found the drive idle and let go — so nothing here stopped. It had been left
because `ahci` read that interrupt as the last PIO Setup FIS's `I` bit; the
adapter now samples the bit before each block, where Serial ATA 2.6 §10.3.10
sends the FIS, and the drive raises exactly §6.3's interrupts on both doors.
`tests/amiga_a600_board.rs` asserts exactly two interrupts for its
`IDENTIFY DEVICE` and one-sector read (three before the fix). None of the four
`tests/amiga_a600_hdf.rs` goldens moved, and each frame was looked at again:
the two Workbench desktops, Workbench 1.3 and the insert-disk screen, as the
table below describes them.

### How far each disk gets

`tests/amiga_a600_hdf.rs`, behind `RSEMU_AMIGA_ROM_DIR` and
`RSEMU_AMIGA_HDF_DIR` (Amiga Forever's `Shared/rom` and `Shared/hdf`), boots
the user's files in place and checks a frame hash; each frame was looked at.

| ROM + disk | Reaches | What is on screen |
| --- | --- | --- |
| Kickstart 3.1 (40.063) + `workbench-311.hdf` | the Workbench 3.1 desktop, 12 s | Black while the ROM finds Gayle and the drive and AmigaDOS runs the startup-sequence; then the grey 640-pixel Workbench screen, "Copyright © 1985-1993 Commodore-Amiga, Inc. All Rights Reserved." in its title bar, the blue-framed "Workbench" window with the Ram Disk icon and the hard-disk icon "Workbench3.1", and the red pointer |
| Kickstart 2.05 (37.350, the A600's own) + `workbench-211.hdf` | the Workbench 2.1 desktop, 13 s | White while the ROM boots; at 9 s the AmigaDOS shell window, "Amiga Release 2.1.1. Kickstart 37.350, Workbench 38.36"; then the grey desktop with "Copyright © 1985-1992 …" in a black title bar, the "Workbench" window, Ram Disk and "Workbench2.1" |
| Kickstart 3.1 + `workbench-135.hdf` | the Workbench 1.3 desktop, 12 s | A blue screen, the 3.1 copyright in the title bar, Ram Disk and the "Workbench1.3" icon, no window |
| Kickstart 3.1, empty bay | its insert-disk screen, 20 s | Black for 19 s while `scsi.device` keeps selecting a drive that is not there and reading a status nothing drives; then the A500's insert-disk animation, bit for bit its frame |

Kickstart's probe, as traced: it writes `$00` then `$A0` to device/head, runs
an echo test on the cylinder-low register (`$12`, `$34`), writes device control
at `$DA3018`, issues `RECALIBRATE`, `IDENTIFY DEVICE`, `INITIALIZE DEVICE
PARAMETERS` (16 heads, 63 sectors), `SET MULTIPLE MODE` (16), and reads with
`READ SECTORS` and `READ MULTIPLE` in CHS mode; it also selects device 1 once,
and reads zeroes from the empty position, as a drive answers for its absent
slave.

**The empty-bay delay is open.** What an A600 with no drive reads from its IDE
status register is whatever its unbuffered data bus floats to; the board here
floats like every other empty address (the last value on the bus). Whether a
real machine without "HD" waits as long before the insert-disk screen is not in
any document at hand.

### Tests

* `src/dev/amiga/gayle/tests.rs` (ROM-free): the decode; the byte swap on
  `IDENTIFY` (the model string reads pairwise swapped at the 68000's
  addresses) and on `READ SECTORS` and `WRITE SECTORS` (an `RDSK` image lands
  in memory, and memory lands on the medium, in order); the interrupt through
  the status, change and enable registers onto `INT2`, and `nIEN`; the empty
  card slot, forced lines, and the `INT2`/`INT6` level bits; the
  identification sequence; the overlay on either CIA; `MemAttrs::debug`
  popping nothing; a snapshot round trip.
* `tests/amiga_a600_board.rs` (ROM-free): the board realizes, its map is the
  A600's, and a hand-assembled 68000 program sends `IDENTIFY DEVICE` and `READ
  SECTORS` through Gayle, waiting on the level 2 interrupt each time, and gets
  the image's bytes.
* `tests/amiga_a600_hdf.rs` (the user's ROMs and HDFs): the four rows above.

## A3000

`machines/amiga-a3000.machine` is a 68030 at 25 MHz with a 68882 in the
coprocessor socket, the Enhanced Chip Set, 2 MiB of chip RAM in a **32-bit**
address space, the motherboard's battery-backed clock — and **SCSI** where the
A600 has IDE:

```
rsemu run amiga-a3000 --media kickstart=kickstart:<rom dir>/amiga-os-310-a3000.rom \
    --media hd0=<hdf dir>/workbench-311.hdf --vnc :5900
```

`hd0` is a whole disk with a Rigid Disk Block, the same image the A600 boots
from its IDE port: the RDB is a partition table, not a cable.

The port is **three objects**, because the machine has three things, and that
split is the point of the work:

| Object | Class | Feature | What it is |
| --- | --- | --- | --- |
| `hd0` | `scsi.disk` | `dev-scsi` | the target: a phase machine and a SCSI command set, backed by `dev::medium` |
| `wd0` | `wd.33c93` | `dev-wd33c93` | the initiator: a Western Digital WD33C93A |
| `sdmac` | `amiga.sdmac` | `dev-amiga-sdmac` | Commodore's DMA controller, which masters memory and which the SCSI chip's two registers are mapped *inside* |

The first two meet on a named bus (`bus = "scsi0"`), the way an `ata.disk` and
its adapter meet in a named drive bay, so **adding a CD-ROM later is a new
target rather than a new controller**. The falsifiable form of the split is in
`src/dev/scsi/mod.rs`: `dev/scsi` contains no controller register name and
`dev/wd33c93.rs` contains no SCSI command opcode.

### Sources

| Source | Covers |
| --- | --- |
| *WD33C93A SCSI Bus Interface Controller — Data Sheet and Application Notes*, Western Digital, November 1990 | §6.1 the register map; §6.2 every register bit by bit; §6.2.1 Auxiliary Status; §6.2.2 the two addresses and the auto-increment; §6.2.19 the whole interrupt code table; §6.3 the two resets; §7.1 the command list; §7.4/§7.5 the Level I and simple Level II commands; §7.6.1 `Select-And-Transfer` and its Command Phase values |
| *The A3000+ System Specification*, Commodore-Amiga | §2.1 Fat Gary's registers; §2.2 Ramsey (Table 2-2); §2.4 the DMAC, §2.4.1 its register map (Table 2-5) and control/interrupt bits (Table 2-6) |
| *Small Computer System Interface-2*, X3.131-1994 | §5.1 the bus phases; §6.6 the messages; §7 the command structure, status byte and sense data; §8 and §9 the commands a direct-access device implements |
| *Amiga Hardware Reference Manual*, 3rd edition, Appendix D | the memory map the board's other windows keep |
| Black-box: Kickstart 3.1 (40.068) and 2.04 (37.175), both A3000 images | which addresses the ROM touches at `$00DD0000` and `$00DE0000`, in what order, and what it waits on |

No emulator source, no FPGA core, no AROS source and no published Kickstart
disassembly was consulted. The one place this page quotes guest instructions —
Ramsey's spin loop, below — is rsemu's own disassembler printing what the guest
was executing when it stopped.

### The register map, as modelled

| Address | What | Notes |
| --- | --- | --- |
| `$00000000`–`$001FFFFF` | chip RAM, behind the overlay | 2 MiB; `OVL` is CIA-A's `PA0` into Gary, as on an A500 |
| `$00BFD000`, `$00BFE000` | CIA-B and CIA-A | the A500's decode, unchanged |
| `$00DC0000` | the battery-backed clock | soldered on this board rather than on a trapdoor card |
| `$00DD0000` | `DAWR` | the `DACK` width. Not in Table 2-5 — the enhanced part dropped it — but Kickstart writes `3` there before anything else, so the A3000's has it where its A2091 ancestor does |
| `$00DD0004` | `WTC` | obsolete, and read/write, which is how software tells this part from the enhanced one |
| `$00DD0008` | `CONTR` | bit 8 `DMAENA`, bit 4 `PREST`, bit 2 `INTENA`, bit 1 `DMADIR` |
| `$00DD000C` | `ACR` | the DMA address, rounded down to an even word |
| `$00DD0010`, `$00DD0014`, `$00DD0018`, `$00DD003C` | `ST_DMA`, `FLUSH`, `CLR_INT`, `SP_DMA` | strobes: they act on any access and drive no data |
| `$00DD001C` | `ISTR` | bits 7/6/5 the controller's line, bit 4 the same gated by `INTENA`, bit 1 `FF`, bit 0 `FE` |
| `$00DD0041`, `$00DD0049` | the WD33C93A's `SASR` | byte lane 1 |
| `$00DD0043`, `$00DD0047` | the WD33C93A's `SCMD` | byte lane 3 |
| `$00DE0003` | Ramsey control | the three mode bits Kickstart spins on |
| `$00DE0043` | Ramsey version | `$0D`, the A3000's own part |
| `$00DFF000` | the custom chip registers | |
| `$00F80000` | Kickstart | 512 KiB |

Everything else floats, for the reason the A500 file gives.

**`CONTR`'s bits are bit numbers, not masks.** Table 2-6 lists `DMAENA` 8,
`PREST` 4, `INTENA` 2 and `DMADIR` 1 in a column whose `ISTR` half is
unambiguously bit *numbers* — 7, 6, 5, 4, 1, 0 for six bits. Reading the
`CONTR` half as masks instead puts `PREST` where `INTENA` is, and Kickstart's
"enable the interrupt now that the command is issued" becomes "reset the SCSI
chip the instant it has been told to select". Which reading is right was
settled by watching the ROM: it writes `$0C`, reads back `$04`, writes `$00`
during setup, and writes `$04` again immediately after putting
`Select-with-ATN` in the Command register.

**`SASR` and `SCMD` are two byte lanes, not two longwords.** §2.4.1 calls the
mapping "a little strangely … based on 68030 behavior, rather than 68030
specifications, so properly designed 68040 cards could not access these
registers as bytes", and every row of Table 2-5 falls out of one rule: lane 1
(`$…1`, `D23`–`D16`) is `SASR` and lane 3 (`$…3`, `D7`–`D0`) is `SCMD`. That
also explains `$00DD0043`, which the table does not list and which both A3000
Kickstarts use for `SCMD`. Read as longwords instead, the table contradicts the
ROM — `$00DD0040`–`$…43` would have to be `SASR` *and* `SCMD` at once.

### What the controller implements, and what it does not

All three levels of the datasheet's command set are there. **Level I**:
`Reset`, `Abort`, `Assert ATN`, `Negate ACK`, `Disconnect`, `Set IDI`.
**Simple Level II**: `Select-With-ATN`, `Select-Without-ATN` and
`Transfer Info`, with the host walking the phases itself and reading the `MCI`
field of each interrupt. **Combination Level II**: `Select-And-Transfer`, where
the chip's own microprocessor runs selection, message out, command, data,
status and message in and raises one interrupt, updating the Command Phase
register at each step so that a termination says where it stopped. Both data
paths: polled I/O through the Data register with `DBR` in Auxiliary Status, and
a `DmaPort` seam the board's DMA controller fills in.

Not implemented, and refused rather than faked: every **target-role** command
(`Reselect`, `Reselect-And-Transfer`, `Wait-For-Select-And-Receive`,
`Send-Status-And-Command-Complete`, `Send-Disconnect-Message`, the four
`Receive` and four `Send` commands) and `Translate Address`, all of which
answer the invalid-command interrupt. **Synchronous transfer** is not modelled:
the Synchronous Transfer register reads back what was written and every
transfer is asynchronous, because nothing here has a transfer *rate*.
**Parity** is never reported, because no byte on this bus was carried by a
wire.

**Disconnection and reselection are not modelled**, and that is a decision
rather than an omission: a SCSI-2 target need not implement disconnection
(§6.6.10), so a target that completes every command in the connection the
initiator opened is a conforming target. Nothing in this tree therefore ever
reselects, and the controller's reselection paths are written from the
datasheet and exercised against a synthetic target rather than against
`scsi.disk`.

`scsi.disk` implements the direct-access command set an operating system of
1990 issues — `TEST UNIT READY`, `REQUEST SENSE` in the fixed format,
`INQUIRY` with vital product data pages `00` and `80`, `MODE SELECT(6)`/`(10)`,
`MODE SENSE(6)`/`(10)` with pages `01`, `03`, `04` and `3F`, `READ CAPACITY`,
`READ(6)`/`(10)`, `WRITE(6)`/`(10)`, `SEEK`, `VERIFY`, `SYNCHRONIZE CACHE`,
`READ DEFECT DATA`, `START STOP UNIT`, `RESERVE`/`RELEASE`, `SEND DIAGNOSTIC`
and `PREVENT ALLOW MEDIUM REMOVAL`. Everything else is `CHECK CONDITION` with
`ILLEGAL REQUEST` / *invalid command operation code*, which is how a driver
finds out what a drive has. `FORMAT UNIT`, linked commands and tagged queueing
are among the refusals.

### Ramsey, and why a memory controller is on this board at all

Everything Ramsey *does* — DRAM refresh, static-column page detection, 68030
burst cycles — is invisible to an emulator whose RAM answers in no time. What
is not invisible is that **Kickstart waits for its control register to read
back what it wrote**:

```text
    LEA     $00DE0003.l,A4
    LEA     $07F7FFF0.l,A3          ; the top of the Fast RAM window
    MOVEQ   #$7,D2
    CMPI.B  #$7F,$40(A4)            ; $00DE0043: no Ramsey?
    BEQ     done
    …
    MOVE.B  D0,(A4)                 ; set WRAP | BURST | PAGE DETECT
  wait:
    MOVE.B  (A4),D1
    AND.B   D2,D1
    CMP.B   D2,D1
    BNE     wait
```

With `$00DE0003` floating that loop never ends and the machine never reaches
`exec`. `src/dev/amiga/ramsey.rs` is two registers and nothing else.

### The ROM, and the "ROM tower"

This board maps a 512 KiB Kickstart at `$00F80000` and **needs no ROM tower**:
no second socket, no `$00E00000` window (that is the A500 board's, and only
AROS fills it) and no write-enabled RAM shadow of `$00F80000`.

The earliest A3000s shipped with a **bootstrap ROM** instead — Kickstart 1.4 or
a 2.0 beta, "SuperKickstart" — which is not an operating system: it finds a
Kickstart *partition* on the SCSI disk, copies the real Kickstart into RAM at
`$00F80000` and jumps to it. `amiga-os-140-a3000.rom` is such an image. Booting
one needs an HDF carrying that partition, which the Amiga Forever Workbench
images do not have, so this board is not tested with it; a machine file for a
bootstrap-ROM A3000 would need the RAM shadow. A shipped Kickstart — 2.04
(37.175) or 3.1 (40.068), both 512 KiB — needs none of that machinery, and both
are what the tests use.

### No motherboard fast RAM, and why

A real A3000 has 1 to 16 MiB of 32-bit RAM at `$07000000` behind Ramsey. This
board has none, which is a machine Commodore sold (no SIMMs fitted) and is
**not** what the board would ship with if fitting it worked. It does not:

* With RAM mapped anywhere in that window, Kickstart 3.1 and 2.04 both relocate
  `ExecBase` into it — `AttnFlags` grows `AFF_ADDR32` and `ExecBase` moves to
  `$0700xxxx`, so the relocation itself works — and then, about half a second
  later, take an **unexpected `CHK` exception** (`AT_DeadEnd | 6`, the
  vector-6 stub) and reboot in a loop, forever.
* The same crash happens with 4 MiB top-aligned and with the whole 16 MiB
  window populated, and with `unassigned = read-as-ones` as well as
  `open-bus`, so it is neither a partially-populated window nor phantom RAM
  found by a blind probe on a floating bus.
* It is independent of SCSI: an A3000 with fast RAM and an **empty** SCSI bus
  crashes identically.

That leaves the 68030 core, and `src/cpu/m68k/` is not this work's to change.
Written down here so the next person starts from the evidence rather than from
the beginning.

### How far each ROM gets

`tests/amiga_a3000.rs`, behind `RSEMU_AMIGA_ROM_DIR` and `RSEMU_AMIGA_HDF_DIR`,
boots the user's files in place and checks a frame hash; each frame was looked
at.

| ROM + disk | Reaches | What is on screen |
| --- | --- | --- |
| Kickstart 3.1 (40.068), empty SCSI bus | its insert-disk screen | a dark purple field; the Amiga check-mark in its blue-to-red gradient, four lines of orange text — "3.1 ROM   40.068 / Copyright © 1985-1993 / Commodore-Amiga, Inc. / All Rights Reserved." — and, to the right, the diskette held below the drive slot, mid-animation |
| Kickstart 2.04 (37.175), empty SCSI bus | the same screen | "2.0 Roms (37.175) / Copyright © 1985-1991 / …", the diskette a little further into the same slide |
| Kickstart 3.1 + `workbench-311.hdf` | `scsi.device` finds the drive, and stops — see below | black |
| Kickstart 2.04 + `workbench-211.hdf` | the same | white, 2.0's blank boot screen |

The two empty-bus rows are the whole chain working: `scsi.device` initialises,
resets the controller, scans all eight bus addresses and finds nothing, and the
boot goes on through `intuition.library` and `console.device` to the screen the
machine draws when it has no disk. `GfxBase->ChipRevBits0` reads `$03`
(`GFXF_HR_AGNUS | GFXF_HR_DENISE`, and **not** `GFXB_AA_ALICE`) and
`ExecBase->AttnFlags` reads `$8037` — `AFF_68010 | AFF_68020 | AFF_68030 |
AFF_68881 | AFF_68882` — so the guest itself agrees this is an ECS board with a
68030 and a 68882 in it. Both are asserted rather than described.

### Where it stops, with a disk on the bus

With `workbench-311.hdf` at SCSI address 0 the boot gets **as far as the bus
scan finding the drive** and no further. Traced at the register level, what the
ROM does is:

1. `DAWR := 3`, `SP_DMA`, `CLR_INT`, the `WTC` read/write test (so it knows
   which DMAC it has), `CONTR := INTENA`.
2. `Own ID := $4F`, `Command := Reset` → interrupt `$01` (reset, advanced
   features enabled); `Own ID := $47`, `Reset` again → `$00`.
3. `Control := 0` (polled I/O), `Timeout := $2C`, `Synchronous Transfer :=
   $40`, `Source ID := $80` (Enable Reselection).
4. `Destination ID := n`, `Command := $06` (`Select-With-ATN`),
   `CONTR := INTENA`, then poll `ISTR`.

With nothing at address *n* the interrupt is `$42` — selection timeout — and
the ROM moves to the next address and eventually finishes. With the drive
there, the interrupt is `$11` (§6.2.19: "a Select command completed
successfully"), then, on the next poll, `$86` (§7.5.6's service-required
interrupt naming the `MESSAGE OUT` phase the target requests because `ATN` was
asserted). The driver reads both, writes the Synchronous Transfer register
again, and its bus-handler task goes to `Wait()` for a signal that never comes.
**Nothing further is written to `$00DD0000` at all** — not in 30 seconds of
guest time — so whatever it is waiting for is not an access this model could
have answered differently.

Ruled out by experiment, each one tried against both ROMs:

| Tried | Result |
| --- | --- |
| No service-required interrupt after the select (only `$11`) | the same stall |
| Command phase instead of message out (`$82`) | the same stall |
| `ISTR` reporting only `INT_S` rather than `INT_F | INT_S | E_INT` | the same stall |
| Delivering the queued interrupt 3 and 20 polls later | the same stall |
| Not touching the Command Phase register on a plain `Select` | the same stall |
| Releasing the bus on a `Reset` command (a real defect, and fixed) | the same stall |

Kickstart 2.04 takes the same path and, on seeing `$11`, reads `Own ID`,
re-initialises the chip and reconfigures it — and then stops at exactly the
point where the empty-bus run issues its next `Select`. So both drivers reach
"there is a device here" and stall in the per-unit bring-up that follows, which
is software this work is not permitted to read.

### Defects found on the way

| Stopped at | Why | Fix |
| --- | --- | --- |
| A tight loop at `$670` in chip RAM, black screen, `exec` never reached | Kickstart's memory sizing sets three bits of Ramsey's control register at `$00DE0003` and spins until they read back. Nothing answered there | `src/dev/amiga/ramsey.rs`, and `machines/amiga-a3000.machine` maps it. Test: `the_control_register_reads_back_what_kickstart_spins_on` |
| `scsi.device` writing register numbers and data to the same address | `SASR` and `SCMD` were decoded as whole longwords, following Table 2-5's addresses literally. They are two *byte lanes* | `scsi_lane` in `src/dev/amiga/sdmac.rs`. Test: `the_scsi_chips_two_registers_are_on_two_byte_lanes` |
| The controller being reset the instant it was told to select | Table 2-6's second column read as masks rather than bit numbers, which put `PREST` at `$04` where `INTENA` is | the `CONTR` constants. Test: `every_dmac_register_is_the_longword_table_2_5_puts_it_at` |
| The driver reading `$11` and finding `INT` still set, so unable to issue the next command (§6.2.20) | both the select's completion interrupt and the first `REQ`'s service interrupt were raised at once, and the status read revealed the second in the same bus cycle | `Chip::poll_pending`: the first is asserted when the chip decides it, the rest arrive when the host next asks — which a board does by reading its own interrupt status register |
| A target left holding the bus after the controller was reset | §6.3.2's "all SCSI bus signals are reset to the negated state" was not modelled | `Chip::release_bus`. Test: `a_reset_lets_go_of_the_bus_so_the_next_selection_starts_clean` |

### Tests

* `src/dev/scsi/tests.rs` (ROM-free): the phase signals against X3.131 §5.1;
  the group-code lengths of §7.1; `INQUIRY`, `READ(6)`, `READ(10)` across more
  than one internal chunk, `WRITE(10)` read back off the medium,
  `READ CAPACITY`, `MODE SENSE` with and without a block descriptor, an
  allocation length truncating, an unsupported logical unit, sense data and
  its clearing, a block past the end, a bus reset's unit attention, the bus
  rendezvous, and a snapshot round trip.
* `src/dev/wd33c93/tests.rs` (ROM-free): the address register's
  auto-increment and its three exceptions; an unavailable register reading all
  ones; both resets and what each keeps; a command ignored while `INT` is set;
  a target-role command refused; selection and its timeout; **the same
  `INQUIRY` driven phase by phase with `Transfer Info`** and **a `READ(10)`
  driven in one go by `Select-And-Transfer` through the DMA seam**; an
  unexpected phase terminating where it stands; `MemAttrs::debug` moving
  nothing; and a snapshot round trip.
* `src/dev/amiga/sdmac/tests.rs` (ROM-free): every longword of Table 2-5; a
  narrow access behaving as a whole longword; the two byte lanes; `ISTR` as a
  live view gated by `INTENA`; `CLR_INT`; a data phase reaching memory at the
  address `ACR` named, and moving nothing while DMA is stopped; `debug`;
  a snapshot round trip.
* `src/dev/amiga/ramsey/tests.rs` (ROM-free): the two registers, the read-back
  the ROM waits on, the read-only version, `debug`, and a snapshot.
* `tests/amiga_a3000_board.rs` (ROM-free): the board realizes; it names
  exactly the media slots it documents; and a hand-assembled 68030 program
  resets the controller, issues a `READ(10)` with `Select-And-Transfer` and
  gets the block into chip RAM by bus mastering, waiting on the level 2
  interrupt the whole way — with an empty bus it gets the `$42` timeout
  instead, which is a report rather than a hang.
* `tests/amiga_a3000.rs` (the user's ROMs and HDFs): the four rows above, plus
  `ChipRevBits0` and `AttnFlags` read out of guest RAM.

## AA: Lisa, the display half

`amiga.denise` with `revision = "aga"` is **Lisa**, the AA chip set's video
chip. She is driven by hand in `src/dev/amiga/denise/aga/tests.rs` and
`tests/amiga_lisa.rs` — written before Alice existed, the way
`amiga_denise_board.rs` drove Denise before Agnus did — and by Alice on the
[A1200](#a1200) below. `src/dev/amiga/denise/aga.rs` is the ledger of what the
AA specification settles and what it leaves open; this is the summary.

**An 8362 and an 8373 are untouched, and that is checked, not asserted.**
Every Amiga golden — `amiga_a500_kickstart`, `amiga_a500_workbench`,
`amiga_a500plus`, `amiga_a600_hdf` against the user's ROMs, disks and hard
disk, and `amiga_a500_chipset`, `amiga_denise_board`, `agnus_board` and
Denise's own golden field without them — is the hash it was. The one change
they share is that the picture is now eight bits a gun: the older parts'
four-bit guns go into it as `n × 17`, the expansion the host adapter always
made, so their host bytes and their twelve-bit `read_row`/`copy_frame` words
are what they were.

### What Lisa does

| | |
| --- | --- |
| Eight bitplanes | `BPLCON0`'s `BPU3` (bit 4): "0000-1000 (none thru 8 inclusive)"; nine to fifteen are clamped to eight |
| The colour table | 256 entries of 24 bits and a `T` bit, reached 32 at a time through `BPLCON3`'s `BANK`. A `LOCT = 0` write sets each gun to `n × 17` and the `T` bit; a `LOCT = 1` write sets the low nibbles only |
| `LISAID` | `$00F8`: the specification's `$F8`, and bits 9–8 low — the board's fetch is four times wide. Those two bits are what `graphics.library` sizes its display database from; see [the A1200](#what-lisaid-bits-9-and-8-are) |
| HAM8, and HAM6 everywhere | Planes 1 and 2 control, planes 3–8 are the six high bits of the modified gun, and the two low bits are held; a base register is one of 64, the plane address with the control bits at `00`. HAM6 works in every resolution too |
| `BPLCON4` | `BPLAM` XOR'ed with every bitplane colour address; `ESPRM`/`OSPRM` the high four bits of even, odd and attached sprites' colours, reset to `0001` |
| Dual playfield, EHB | 4 + 4 planes, playfield 2 at `PF2OF`'s offset (reset 8). EHB only when `SHRES = HIRES = HAMEN = DPF = 0` and `BPU = 6`, and `KILLEHB` still kills it |
| 35 ns everywhere | `BPLCON1`'s eight-bit scroll per playfield, `DIWHIGH`'s `H1`/`H0`, `SPRxCTL`'s `SH1`/`SH0`. The picture is always 35 ns columns: 1600 across for a standard PAL field |
| Sprites | `SPRES` (the ECS default, 140, 70 or 35 ns, whatever the playfield's resolution), 16/32/64-bit data by `FMODE`'s `SPR32`/`SPAGEM`, attachment in every resolution, `BRDSPRT` behind `ECSENA`, and `SSCAN2` taking `SH10` out of the comparison |
| `CLXCON2` | Planes 7 and 8 in collisions; a `CLXCON` write clears it |

### The seam Alice needs, and what she does with it

Alice landed on the other side of it and needed nothing changed; the four
points below are as they were written, and `src/dev/amiga/agnus/aga.rs` is her
half.


1. **Bitplanes**: `denise::Fetch::planes` is eight streams now. A 32- or
   64-bit fetch is that many consecutive pixels — "the parallel to serial
   conversion is triggered whenever bit plane #1 is written, indicating the
   completion of all bit planes for that word (16/32/64 pixels). The MSB is
   output first" (§4, `BPLxDAT`) — so Alice puts one, two or four words a
   fetch slot into the same stream, in shift order. `Fetch::start` is still
   the colour clock of the first fetch; Lisa places its first pixel by the
   3rd-edition arithmetic — one fetch block and half a count later — with the
   block stretched by a wide `FMODE`, [below](#where-a-wide-fetch-is-first-shown).
2. **Sprites**: `Video::sprite_dma(sprite, b_buffer, bits)` for a 32- or
   64-bit sprite fetch, left-justified in a `u64`. It is timed exactly as a
   register write is — stamped with the `Beam`'s position and queued behind
   the `SPRxPOS`/`SPRxCTL` writes the same DMA slot made — so the `CTL`
   write that disarms a sprite cannot overtake the data that re-arms it. A
   16-bit fetch may still go through the register bus; the two agree.
3. **Registers**: `FMODE` (`$1FC`, `A D`) reaches both chips as an ordinary
   write, as do `BPLCON4`, `CLXCON2` and the extended `BPLCON1`/`BPLCON3`/
   `DIWHIGH`. `regs.rs` declares the Lisa rows; the Alice rows (bitplane 7 and
   8 pointers) are Alice's to add.
4. **Scan doubling**: `BSCAN2` is Alice's (it picks the modulus); Lisa only
   latches it. `SSCAN2` is both chips': Lisa drops `SH10` from the compare and
   Alice uses the bit as a per-sprite enable.

### Where the document is silent

Each is marked in `aga.rs` as an inference: where a fetch's first pixel lands
(the 3rd-edition arithmetic, with the block a wide `FMODE` stretches measured
off Kickstart 3.1 — [below](#where-a-wide-fetch-is-first-shown)); that HAM6's four bits go to a gun's
top four and its low four are held, as HAM8's are; that `BPLAM` masks every
bitplane colour address including a zero pixel inside the window, and not the
border; that five to seven planes with `HAMEN` are HAM6; that `PF2OF` is
playfield 2's offset whatever `PF2PRI` says (§2 and §4 read differently); and
that a scroll larger than §5's range for the fetch width is applied whole.
`RDRAM`, genlock (`ZD`, `BRDNTRAN`, `ZDCLKEN`), `EXTBLKEN`, `BYPASS` and
`UHRES` are latched only.

### Pictures

`tests/amiga_lisa.rs` paints three whole fields through the public API and the
real host adapter, asserts their pixels, and writes `lisa-256.png`,
`lisa-ham8.png` and `lisa-sprites.png` to `RSEMU_AMIGA_FRAME_DIR` when built
with `display-png`: a 16 × 16 chart of 256 24-bit colours, a HAM8 gradient of
some eight thousand colours with the HAM fringe at its left edge, and one
64-pixel sprite at 140, 70 and 35 ns over a 256-colour background.

## AA: Alice, the DMA half

`amiga.agnus` with `revision = "aga"` is **Alice**, the 8374: everything an
8375 does ([*ECS*](#ecs-the-enhanced-chip-set-and-the-a500)) and the AA
additions on top. `src/dev/amiga/agnus/aga.rs` is the ledger — what the
*Specification for the Advanced Amiga (AA) Chip Set* settles, what it leaves
open, and which sentence each behaviour comes from; this is the summary.

**An 8370, an 8371, an 8372A and an 8375 are untouched, and that is checked,
not asserted.** `FMODE` is decoded on every part, because the decode is a
property of the address map, and acted on by none but Alice; the bitplane 7 and
8 pointers are held by Alice alone; and `BPLCON0`'s `BPU3` is a bit only she
has. Two unit tests say so by name, and all eight Amiga goldens —
`amiga_a500_kickstart`, `amiga_a500_workbench`, `amiga_a500plus` and
`amiga_a600_hdf` against the user's ROMs and disks, and `amiga_a500_chipset`,
`amiga_denise_board`, `agnus_board` and `amiga_lisa` without them — are the
hashes they were.

### What Alice does

| | |
| --- | --- |
| Eight bitplanes | `BPL7PT` `$0F8`/`$0FA` and `BPL8PT` `$0FC`/`$0FE` (§3), fetched when `BPLCON0`'s `BPU3` at bit 4 counts past six: "0000-1000 (NONE thru 8 inclusive)" (§4) |
| `FMODE`'s fetch widths | §4's table: a bitplane or sprite transfer moves 2, 4 or 8 bytes, "normal CAS" or "double CAS", on a 16- or 32-bit bus. One, two or four words a transfer for each |
| The sprite seam | 16 bits still go through the register bus; 32 and 64 go to Lisa through `Video::sprite_dma`, left-justified, queued behind the `SPRxPOS`/`SPRxCTL` writes of the same DMA slot |
| `BSCAN2` | The modulus becomes the *line's* rather than the plane's: `BPL1MOD` when `DIWSTRT`'s `V0` matches the beam counter's, `BPL2MOD` when it does not (§2, *Bitplanes*) |
| `SSCAN2` | With a sprite's own `SH10` set, its data fetch is skipped on a line of the wrong parity and "LISA reuses the sprite data from the previous line" (§2, *Sprites*) |
| 2 MiB of chip RAM | "PTL,PTH=20 bit Pointer that addresses DMA data … (old chips- 18 bits)" (§3) — twenty bits of address from bit 1 is 2 MiB, so Alice's reach is not a property: `reach = 2M` may be written and nothing else |
| `DDFSTRT`, `DDFSTOP` | One bit more, `H2` — "H8 H7 H6 H5 H4 H3 H2 X" against bits 7–0 (§4) — so a fetch starts on an even colour clock rather than a multiple of four |
| `VPOSR` | `$22` PAL and `$32` NTSC: "8374(alice)" (§4) |

**The blitter and the copper are unchanged**, and that is a finding rather than
an omission: §4's `BLTxPT`, `BLTxMOD`, `BLTAFWM`/`BLTALWM`, `BLTxDAT`,
`BLTCON0`, `BLTCON1`, `BLTSIZE`, `BLTSIZH`/`BLTSIZV`, `COPCON`, `COPxLC`,
`COPJMP1`/`COPJMP2` and `COPINS` pages are the Enhanced Chip Set's pages word
for word — `BLTCON0L`, `BLTSIZV` and `BLTSIZH` still carry `h`, "new for HiRes
chip set", and `COPCON`'s rule is still "if 0, access to RGA>7E". Both engines
gain the wider pointer and nothing else. `UHRES` — `BPLHPT`, `SPRHPT`,
`BPLHMOD`, `SPRHSTRT` and the rest — is held and not acted on, as on an 8375:
it drives external logic no board here has.

### Where the document is silent

Each is marked in `aga.rs` as an inference.

* **How many words a line a wide `FMODE` fetches.** §5's key says a mode "needs
  1x / 2x / 4x Bandwidth" and its scroll table gives one fetch's worth of
  pixels — 16, 32 or 64 bitplane pixels — so `FMODE` buys bus cycles, not
  picture, and the word count a line is the window's. What the document does
  not give is the rounding: a transfer is indivisible, so the count is rounded
  **up** to a multiple of the width here, and the pointer advances by twice
  that before the modulo. Truncating instead would fetch fewer pixels than the
  window displays. A program whose window is a whole number of transfers wide
  cannot tell.
* **The width of a sprite's *control* fetch.** §4's table is the sprite
  channel's fetch increment and names no exception, so `SPRxPOS` and `SPRxCTL`
  are each the first word of a transfer of the same width and the rest of it is
  skipped — which is what a sprite structure padded to the fetch width expects.
* **What a scan-doubled sprite does at the ends of its run.** Only the *data*
  fetch is gated by the parity: a control fetch is what loads `SPRxPOS`, so the
  bits the gate asks about are not there yet, and §2's note that "sprite
  vertical start and stop positions must be of the same parity" keeps a `VSTOP`
  line on the fetching side anyway.
* **`VPOSR` used to collide with the 8375's.** The AA specification's own
  copy of the identification list reads "8372(fat-hr) (agnushr), rev. 5 = 21
  PAL, 31 NTSC", one less than the `$22` this tree gave the 8375 off a
  differently-transcribed copy of the same table. The ROM settled which is
  right: Kickstart 3.1 sets `GFXF_AA_ALICE` from bit 1 of the identification
  ([the A1200](#the-guest-sees-aa-and-that-is-a-test-rather-than-a-claim)), so
  at `$22` an A600 booting off its hard disk read `ChipRevBits0 = $07`, an ECS
  board claiming Alice. The 8375 now answers the specification's `$21`, whose
  bit 1 is clear and which keeps every row's PAL and NTSC `$10` apart; every
  golden is unchanged, and that A600 reads `$03`.

### Tests

* `src/dev/amiga/agnus/aga.rs` (ROM-free): the four `FMODE` widths for
  bitplanes and for sprites, `BSCAN2`'s modulus choice, `SSCAN2`'s parity gate,
  a transfer left-justified in a `u64`, and `BPU3`.
* `src/dev/amiga/agnus/tests.rs` (ROM-free): Alice's `VPOSR`; the bitplane 7
  and 8 pointers held by her and by no older part; eight planes fetched with
  their own pointers and both modulos; `BPU = 8` putting colour 128 on a real
  Lisa's screen; each `FMODE` width's word count and the rounding; a 64-bit
  fetch drawing what four 16-bit fetches draw; `H2` in `DDFSTRT`; `BSCAN2` with
  and without; a wide sprite's six transfers; `SSCAN2` skipping half of them;
  2 MiB addressed through a twenty-bit pointer; and a snapshot round trip.
* `tests/amiga_alice.rs` (ROM-free, on the shipped A1200): a hand-assembled
  68EC020 program and a copper list put eight bitplanes out of the **second**
  megabyte on screen as twenty sawtooth ramps of colours 128–143, draw the
  identical picture at all three `FMODE` widths, and place a 64-pixel sprite
  Alice fetched four words a transfer. `alice-256.png` and `alice-sprite.png`
  in `RSEMU_AMIGA_FRAME_DIR`.

## A1200

`machines/amiga-a1200.machine` is the A600's Gayle, CIAs, Paula and DF0 around
the AA chip set, with a **68EC020 at 14.19 MHz** and **2 MiB of chip RAM**, and
it boots Workbench 3.1 off the hard disk:

```
rsemu run amiga-a1200 --media kickstart=kickstart:<rom dir>/amiga-os-310-a1200.rom \
    --media hd0=<hdf dir>/workbench-311.hdf --vnc :5900
```

### Sources

| Source | Covers |
| --- | --- |
| *Functional Specification for the Advanced Amiga Chip Set (AA)*, Commodore-Amiga, 06/07/91, ed. R. Raible | Everything both AA chips do: §1 the summary, §2 the explanations, §3 the register list, §4 the per-register pages, §5 the new Lisa modes |
| *A1200 System Schematics Service Addendum*, Commodore, 1992 | The parts: "ALICE (AA AGNUS)", "LISA (AA DENISE)", "BUDGIE (ASIC)", "ROM 512KX16", "DRAM 256KX16" and its "OPTIONAL" pair, "TTL 28-37512 MHZ PAL" |
| *GAYLE — Gate array for A300/A500+ — Specification*, Commodore, July 10 1991 | The same chip as the A600's, doing the same decoding: see the A600 section |
| MC68020/MC68EC020 User's Manual | The EC020: the 68020 with twenty-four address pins and no dynamic bus sizing |
| *Amiga ROM Kernel Reference Manual: Libraries*, 3rd ed. | The structure layouts read out of guest memory: `ExecBase`'s library list, `GfxBase` (`ChipRevBits0`, `ActiView`, the copper lists), `View`, `ViewPort`, `RasInfo`, `BitMap`, and the display database's `QueryHeader`, `DisplayInfo` and `DimensionInfo` with their `DTAG_` identifiers, `DIPF_` flags and `ModeID` keys |
| Black-box: Kickstart 3.1 (40.068, the A1200's own) | What it reads, writes and waits on; `GfxBase->ChipRevBits0`; how far each disk gets; the display database it builds for each `LISAID`, and the copper lists it programs for each resolution and bandwidth |

No Amiga emulator source, no FPGA reimplementation of any Amiga chip, no AROS
source and no Kickstart disassembly was consulted. The AA specification was
fetched on its own, from a document archive rather than from any project's
repository.

### What the board changes from the A600

| | A600 | A1200 |
| --- | --- | --- |
| Processor | 68000 at `clk / 4` | **68EC020** at `clk / 2` — 14.18758 MHz PAL, exactly twice the A600's |
| Chips | 8375 Agnus, 8373 Denise | **Alice** and **Lisa**, `revision = "aga"` on both |
| Chip RAM | 1 MiB | **2 MiB**, which is exactly what a twenty-bit pointer reaches |
| Everything else | — | unchanged: the same Gayle object, the same decode, the same CIAs, Paula, DF0, keyboard and mouse |

`mem` is 24 bits wide because that is how many address pins the part has, so
the trapdoor's 32-bit local bus at `$08000000` is off the map by construction,
as it is on a machine with an empty trapdoor. Bank 6 and `$DC0000` float, as
on the A600.

### How far each disk gets

`tests/amiga_a1200.rs`, behind `RSEMU_AMIGA_ROM_DIR`, `RSEMU_AMIGA_HDF_DIR` and
`RSEMU_AMIGA_ADF_DIR`, boots the user's files in place and checks a frame hash;
each frame was looked at.

| ROM + disk | Reaches | What is on screen |
| --- | --- | --- |
| Kickstart 3.1 (40.068) + `workbench-311.hdf` | the Workbench 3.1 desktop, 12 s | Black while the ROM finds Gayle and the drive and AmigaDOS runs the startup-sequence; then the grey 640-pixel Workbench screen — **1280 of Lisa's 35 ns columns** — "Copyright © 1985-1993 Commodore-Amiga, Inc. All Rights Reserved." in its title bar, the blue-framed "Workbench" window with the Ram Disk icon and the hard-disk icon "Workbench3.1", and the red pointer |
| Kickstart 3.1 + `amiga-os-310-workbench.adf` in DF0, bay empty | the same desktop, about 70 s | Black while `trackdisk.device` reads the disk track by track; then the same screen and window with a **floppy** icon labelled "Workbench3.1" |
| Kickstart 3.1, both drives empty | its insert-disk screen, 45 s | The processor is *stopped* for thirty-one seconds while `scsi.device` waits on a drive that is not there; then the purple screen, the gradient check mark, "3.1 ROM 40.068", and the drive with the disk part-way into the slot. Bit for bit the A600's screen at twice the width, so the picture is not by itself evidence of the chip set — the ROM version is |

### The guest sees AA, and that is a test rather than a claim

`GfxBase->ChipRevBits0`, read out of guest memory the way
`tests/amiga_a500plus.rs` reads it for the Enhanced Chip Set — exec's library
list walked from `ExecBase` by name, `gb_ChipRevBits0` at offset `$EC` of the
library base:

| Board | `ChipRevBits0` |
| --- | --- |
| A1200, once the ROM has a boot device | **`$1F`** — `GFXF_HR_AGNUS`, `GFXF_HR_DENISE`, `GFXF_AA_ALICE`, `GFXF_AA_LISA` and bit 4 |
| A1200 with no boot device at all | `$13` — the AA pair is never set; the ROM sets it at 1.3 s on a board that boots, ten seconds before AmigaDOS mounts anything |
| A600, same ROM family, same Gayle and CIAs and Paula | `$03` |

**Which chip each bit comes from was measured**, by running the A1200's own
machine source with one chip swapped for its Enhanced Chip Set part, and then
by sweeping each chip's identification word:

| Board | `ChipRevBits0` |
| --- | --- |
| Alice + Lisa | `$13` → `$1F` |
| Alice + 8373 Denise | `$03` → `$07` |
| 8375 Agnus + Lisa | `$13` → `$1F` |
| Alice answering `$20`, `$21`, `$30` or `$31` in `VPOSR` | `$1B` |
| Alice answering `$22`, `$23`, `$32` or `$33` | `$1F` |

So bit 3 (`GFXF_AA_LISA`) and bit 4 are `LISAID`'s, and **bit 2
(`GFXF_AA_ALICE`) is bit 1 of `VPOSR`'s Agnus identification**. An earlier
reading of the first three rows had it "not `VPOSR`", because the 8375 swap
moved nothing and neither did Alice at `$23`; both had bit 1 set, because this
tree's 8375 answered `$22`. It answers the AA specification's `$21` now
(`src/dev/amiga/agnus/ecs.rs`, `agnus_id`): at `$22` the A600 booting
Workbench off its hard disk read `$07` — the ROM's later chip test, the one
that sets the AA pair on the A1200, found "Alice" on an ECS board — and it
reads `$03`, which `the_ecs_board_booting_workbench_still_finds_no_alice`
holds.

### What `LISAID` bits 9 and 8 are

`ChipRevBits0` at `$1F` did not make the machine an AA machine to
`graphics.library`. Driven through the input seam, ScreenMode Preferences on
the A1200 offered **"Maximum Colors: 16"** for PAL:High Res, the A600's
answer, where an AA machine offers 256. What decides it was found black-box,
in this order:

1. **The display database, read out of chip RAM.** The *ROM Kernel Reference
   Manual: Libraries* gives `DisplayInfo` and `DimensionInfo` a `QueryHeader`
   whose `StructID` is `DTAG_DISP` (`$80000000`) or `DTAG_DIMS`
   (`$80001000`), so a scan of guest memory finds every record. The A1200's
   `DisplayInfo` records were already AA — `PaletteRange` 65535, eight bits a
   gun, thirty-six modes the A600 lacks (HAM and EHB in high and super-high
   resolution; none carries `DIPF_IS_WB`, so Workbench's list is unchanged by
   them) — but **every `DimensionInfo` was the Enhanced Chip Set's**:
   `MaxDepth` 5 in low resolution, 4 in high and 2 in super-high, the same
   records the A600 has. ScreenMode's "Maximum Colors" is `1 << MaxDepth`.
2. **What the processor reads.** A recorder on the custom-register bus showed
   that while the database is built (0.78–0.80 s) the processor reads only
   `VPOSR`, `VHPOSR`, `INTENAR` and `INTREQR`, and before it nothing of the
   chips but those, `DMACONR`, `JOY0DAT` and **nineteen reads of `LISAID`**.
   Swapping Alice for an 8375, halving chip RAM, a 68000 in the EC020's place,
   every `VPOSR` identification above, and writing `$1F` into
   `ChipRevBits0` before the database is built all left `MaxDepth` where it
   was. Kickstart 3.0 for the A1200 and the CD32's 3.1 behave the same.
3. **`LISAID`, swept.** This model answered `$FFF8`: the specification's `$F8`
   and "the upper 8 bits of this register are reserved", read as ones, as for
   an 8373. Answering other words and reading the database back:

| `LISAID` | bits 9–8 | `MaxDepth` lores / hires / shres |
| --- | --- | --- |
| `$FFF8`, `$0FF8`, `$07F8`, `$03F8` | `11` | 5 / 4 / 2 |
| `$FEF8`, `$05F8`, `$01F8` | `01` | 8 / 8 / 4 |
| `$06F8`, `$02F8` | `10` | 8 / 8 / 4 |
| `$00F8`, `$04F8`, `$08F8` … `$80F8`, `$F8F8` | `00` | 8 / 8 / 8 |

**Bits 9 and 8 are the board's fetch bandwidth, and nothing else in the upper
byte matters.** They are §4's `FMODE` pair `BPAGEM`/`BPL32` read active low:
`11` is one times, a 16-bit bus with normal `CAS`; `01` and `10` two times;
`00` four, 32 bits and double `CAS`. The depths are §5's "needs 1x / 2x / 4x
Bandwidth" key. The ROM also chooses its `FMODE` from them — `$0000`, `$0002`
for `01`, `$0001` for `10`, `$0003` for `00` — so they are the whole of how it
learns what the board can fetch.

An A1200 is four 256K × 16 DRAMs on a 32-bit bus with page mode, which is
`FMODE $000F`'s four times, so **Lisa answers `$00F8`**. An 8373 drives none of
its upper byte and answers `11`, one times, which is right for a chip without
`FMODE`: the ECS constant was right because its reserved byte reads as ones,
and Lisa's was wrong for the same reason. Bits 15–10 move nothing and are zero.
`src/dev/amiga/denise.rs`, `LISA_ID`, carries the table;
`lisaids_upper_bits_say_the_fetch_is_four_times_and_an_8373s_say_one` is the
ROM-free test.

ScreenMode now offers **"Maximum Colors: 256"** on the A1200 and still 16 on
the A600 beside it. The mode list is the A600's on both, as it should be:
this install's `Devs/Monitors` holds only `PAL` and `NTSC`, and DblPAL,
Multiscan and the rest sit unused in `Storage/Monitors`, where Workbench 3.1
puts them.

### Where a wide fetch is first shown

Answering four times made the ROM fetch its Workbench screen with
`FMODE $0003`, which it had never done here, and the desktop came out sixteen
high-resolution pixels to the left with the start of the next plane-row's
data at its right edge. Lisa placed a fetch's first pixel by the 3rd-edition
manual's arithmetic — one fetch block and half a count after the fetch, eight
counts in low resolution, four in high — whatever `FMODE` said.

The AA specification says which way that must move — "the parallel to serial
conversion is triggered whenever bit plane #1 is written, indicating the
completion of all bit planes for that word (16/32/64 pixels)" (§4, `BPLxDAT`),
and a wider group completes later — but not by how much. **The ROM knows**,
because it programs its screens for the silicon's delay. With `LISAID` set to
each bandwidth, ScreenMode's "Use" reopened the Workbench in low, high and
super-high resolution, and each copper list was read out of chip RAM beside the
screen's `BitMap` (through `GfxBase->ActiView`, its `ViewPort` and `RasInfo`).
All nine keep `DDFSTRT $38`, `DIWSTRT $xx81` and `BPLCON1 0`, and set the
bitplane pointer some words before the bitmap so that its first pixel is at
the window's edge:

| | `FMODE 0` | `FMODE 1` or `2` | `FMODE 3` |
| --- | --- | --- | --- |
| lores | pointer +0: block 8 | +0: 8 | +0: 8 |
| hires | back 1 word: block 4 | +0: 8 | +0: 8 |
| shres | back 3 words: block 2 | back 2: 4 | +0: 8 |

The one-times column is the manual's arithmetic and this model's own earlier
SuperHires inference, which is how the method was checked; every modulo the ROM
chose also matches the word counts Alice already fetched. The rest is **the
one-times block times `FMODE`'s factor, no longer than eight counts**, which is
what `denise/aga.rs`'s `fetch_block` now does. With it, the four-times
desktop is the one-times desktop to the pixel — `GOLDEN_WB311_HD` and
`GOLDEN_WB311_DF0` did not move — and
`a_wide_fmode_delays_the_first_pixel_by_what_kickstart_3_1_programs_for` holds
Lisa to all twelve cells without a ROM.

What moved: the ScreenMode session's volume, Prefs and ScreenMode pictures. A
pixel diff of each against the frame before puts every changed pixel in the
title bar's free-memory figure (1,822,912 → 1,822,400 graphics mem: the
four-times screen takes 512 bytes more), the "Maximum Colors" number, and the
Colors slider's knob, which is narrower because the slider now runs to 256.

### Open, and not guessed at

* **Why the block stops at eight counts.** Every screen the ROM opens has
  `DDFSTRT` a multiple of eight, and a second reading fits the same nine: a
  block of the full stretched length on a grid aligned to it, with the fetch
  rounding `DDFSTRT` down to that grid. The two differ only for a `DDFSTRT`
  off the grid, which the ROM never programs. The bound is taken because it
  changes nothing but the delay; a program that fetches wide from an unaligned
  `DDFSTRT` is the test that would settle it.
* **The empty-bay delay**, as on the A600: what a real machine without a drive
  reads from its IDE status register is whatever its unbuffered bus floats to,
  and how long a real A1200 waits is not in any document at hand.

## CD32

`machines/amiga-cd32.machine` is the A1200's AA chip set and 68EC020 with the
floppy and the IDE port taken away, **Akiko** in their place, a CD-ROM drive
behind it and a joypad on the second controller port. It runs the animated boot
screen a real CD32 shows with nothing in the tray:

```
rsemu run amiga-cd32 --media kickstart=kickstart:<rom dir>/amiga-os-310-cd32.rom \
    --media ext=kickstart:<rom dir>/amiga-os-310-cd32-ext.rom --vnc :5900
```

### Sources

| Source | Covers |
| --- | --- |
| *Amiga CD32 Developer Notes*, Revision 3, Commodore-Amiga Inc. | Everything Commodore published about this machine: "2 Megabytes of 32-bit Chip RAM", "14MHz 68EC020 CPU", "Top loading double speed CD-ROM drive", "Akiko is a 160-pin PQFP … It includes the CD-ROM control logic and the system timers", "very fast chunky-to-planar conversion hardware", "The EEPROM is 8K bits", the pad's "six action buttons, one start button and four directional arrows", and `ReadJoyPort()`, `CD_READ` and `nonvolatile.library` as the ways to reach all three |
| *Functional Specification for the Advanced Amiga Chip Set (AA)*, Commodore-Amiga, 06/07/91 | Alice and Lisa, as on the A1200 |
| *Amiga Hardware Reference Manual*, 3rd ed. | The nine-pin game port: pins 1–4 on Denise's counters, pin 6 on CIA-A's `PA7`, pins 5 and 9 on `POTGO`, and `OUT…`/`DAT…` making the last two outputs |
| ECMA-130 (CD-ROM frames) and ECMA-119 (ISO 9660) | What a disc image is: the 2352-byte frame, its sync pattern and its Mode 1 and Mode 2 Form 1 layouts, the 2048-byte logical sector a file system sees, the 150-frame lead-in |
| NXP UM10204 (I²C) and the 24C08 data sheet | What the part on Akiko's two wires does with them |
| Black-box: the user's own CD32 ROM, 40.60 and its extended half | Akiko's whole register file — see below |

**Commodore published no hardware reference for Akiko**, so every register
address and bit below came from watching what the ROM does on the bus. No
Amiga emulator source, no FPGA reimplementation, no AROS source and no
Kickstart disassembly was consulted.

### What the board changes from the A1200

| | A1200 | CD32 |
| --- | --- | --- |
| Storage | Gayle's IDE port and DF0 | **neither**; a CD-ROM drive behind Akiko |
| ROM | one 512 KiB part | **two**: `kickstart` at `$F8_0000` and `ext` at `$E0_0000`, through the same `amiga.gary` window the A500 board has for AROS |
| Chip RAM | 2 MiB, half-populated optional | 2 MiB, soldered |
| Input | keyboard and mouse | **no keyboard connector at all**; a joypad on port 1, a mouse on port 0 |
| New | — | `amiga.akiko` at `$B8_0000`, `amiga.cd`, `amiga.cd32-pad` |

Everything else — the crystal, the processor's divisor, Alice, Lisa, Paula, the
CIAs and their decode, the `OVL` overlay — is the A1200's.

### Akiko, measured

`src/dev/amiga/akiko.rs` is the ledger. With an empty tray the ROM touches
sixty-four bytes at `$B8_0000` and nothing else:

| | What | How it is known |
| --- | --- | --- |
| `$00` | `$C0CACAFE` | Black-box: the ROM reads the word at `$B8_0002` once and `$CAFE` is what lets it go on |
| `$04`, `$08` | interrupt request and enable, bits 31–24 | Black-box: written once, then read as a pair for ever |
| `$10`, `$14` | two pointers into chip RAM (`$0001_0000`, `$001F_E400`) | Black-box; **their layout is not known** and this model stores them and walks nothing |
| `$18`, `$1C` | ring indices, the chip's and software's | Black-box |
| `$24`, `$25` | configuration; `$25` is written and read back as a presence test | Black-box |
| `$30` | the EEPROM's two wires: bit 31 SCL, bit 30 SDA, bit 15 drive SCL, bit 14 drive SDA | **Derived**: the ROM's first four writes are a textbook I²C start condition and the eight bits after it are `$A0`, a 24Cxx's write address, under that reading and under no other |
| `$38` | the chunky-to-planar corner turn | **Measured**, below |

The EEPROM is the check on all of it. With nothing answering on the wires, the
ROM's five virtual seconds of boot contain **207 005** accesses to this
register file, almost all of them retries of a transfer that never completes.
With a 24C08 on them it is **311**: the transfer happens once and the boot goes
on.

### The corner turn, and why its orientation is not a guess

Thirty-two 8-bit chunky pixels in, eight 32-bit planar longwords out — a square
bit matrix written by rows and read by columns:

```
plane[p] bit (31 - i)  =  (chunky[i] >> p) & 1        i = 0..31, p = 0..7
```

Which end of the longword is the first pixel, and which plane comes out first,
would be inferences — except that **the CD32's ROM proves the converter on its
way up** and will not go on if it is wrong. It writes eight longwords of
`$5555_0000` and reads the result back:

| The model | What the ROM does |
| --- | --- |
| Pixel 0 in the most significant byte, plane 0 first | reads **four** longwords — `$CCCCCCCC`, 0, `$CCCCCCCC`, 0 — and goes on to the next thing it does, which is to write `$80` into `$25` |
| Plane 7 first | reads **one** longword, the 0 that is plane 7, and stops; `$25` is never touched |
| Pixel 0 in the least significant bit | reads **one** longword of `$33333333` and stops in the same place |

So the transform, its pixel order and its plane order are all measured. The one
inference left is that a single byte pointer serves both sides and wraps at
thirty-two, which the trace cannot distinguish from two pointers that both
start at zero.

### What does not work yet, and why it is written down rather than guessed

**No disc reaches the guest.** `amiga.cd` is a complete mechanism — it finds
its sectors, tells 2048-byte ISO 9660 user data from 2352-byte raw frames by
the frame's sync pattern, lifts Mode 1 and Mode 2 Form 1 user data out of a
frame, and answers a one-track table of contents — and Akiko holds it and can
say whether a disc is in the tray. What is missing is the road between them:
the message format `cd.device` and the controller pass commands through is in
no Commodore document, and **with an empty tray the ROM never sends one**, so
there was nothing to watch. A plausible invention would agree with no real disc
and no real game, so there is none here.

For the same reason Akiko has **no interrupt pin**: which of the processor's
levels a CD interrupt reaches was not determined either, and the ROM polls
`$04` regardless, so the boot does not depend on it.

Also not modelled: Akiko's system timers (named in one sentence of the
Developer Notes, touched by nothing), CD audio, subcode and so CD+G, multi
session, and Mode 2 Form 2's 2324-byte sectors.

### The joypad

`src/dev/amiga/cd32pad.rs`. Held still it is an ordinary two-button joystick —
four switches to ground on pins 1–4, red on pin 6, blue on pin 9. When the
machine drives **pin 5 low** (`POTGO`'s `OUTRX`, published on Paula's new
`potrx-out` wire) the pad's shift register latches, the pad lets go of pin 6,
and the machine clocks the other seven buttons out of pin 9 one per rising
edge: blue, red, yellow, green, forward, reverse, play, then a one and then
zeros for ever. That terminating low is how `ReadJoyPort()` tells a pad from a
joystick.

The pins are the hardware manual's. **The shift register, its bit order and the
terminating one-then-zeros are in no Commodore document** and are marked as
such in the module: they are the pad's known behaviour, and the CD32's own ROM
could not settle them because the boot screen never reads a pad.

Paula gained four output wires for this — `potlx-out`, `potly-out`,
`potrx-out`, `potry-out` — because `POTGO` could already make a pot pin an
output and nothing could see the level. A pin it does not drive is released and
the source drives high, leaving the net's pull to decide.

**No front end presses it yet.** The pad is a named host object with a
record/replay door of its own (`amiga-cd32-pad:pad0`), so a recorded payload
replays and a test drives it directly; what is not written is a mapping from a
person's keyboard or gamepad to eleven buttons, the way `host::input::amiga`
maps keys to an `amiga.keyboard` and a pointer to an `amiga.mouse`. That
mapping is a choice about a host's controls rather than a fact about the
machine, and it is left until somebody has a control to map.

### How far it gets

`tests/amiga_cd32.rs`, behind `RSEMU_AMIGA_ROM_DIR`, boots the user's two ROM
halves in place and checks a frame hash; each frame was looked at.

| ROM + disc | Reaches | What is on screen |
| --- | --- | --- |
| Kickstart 3.1 (40.60) + its extended half (40.60), empty tray | **the animated boot screen**, 6 s | Black for five seconds while Kickstart sizes memory, builds exec's lists, and `cd.device` finds Akiko, proves the corner turn, talks to the EEPROM and finds the drive empty — a *different* black from 4 s, where the frame hash moves and `ChipRevBits0` reaches `$1F`. At 6 s: a starfield on black and, across the middle, a grey compact disc seen almost edge-on with a black hub, a white ring and four rainbow diffraction streaks. At 12 s: a band of deep purple sky across the top third with "AMIGA CD" over it in dark red and silver serif capitals, a rainbow highlight through the "CD", and "32" raised to its right with a small "TM". The disc turns, and the sky is a ribbon that sweeps down behind it and back up as its colours cycle — at 14 s it is a teal and green aurora along the bottom — so a golden is one exact virtual instant |
| The same with a disc in the tray | the same frame, hash for hash | Nothing changes, for the reason above |

The processor is **stopped** from 6 s on: the animation is copper and blitter
work with the processor waiting on the interrupt that drives it, which is what
an idle Amiga looks like.

`GfxBase->ChipRevBits0` reads **`$1F`** from 4 s, two seconds before anything
is drawn — `GFXF_HR_AGNUS`,
`GFXF_HR_DENISE`, `GFXF_AA_ALICE`, `GFXF_AA_LISA` and bit 4 — so the guest's
own `graphics.library` found Alice and Lisa. Both ROM halves identify
themselves as **40.60** through their header words at `$F8_000C` and
`$E0_000C` — both halves carry the same version and revision — and
`ExecBase`'s `lib_Version` is 40.

Amiga Forever ships no CD32-bootable disc image: its `Shared` directories hold
ROMs, ADFs and HDFs, and the only ISO in the product is its own installer DVD.
Nothing here went looking for a game.

## A4000

`machines/amiga-a4000.machine` is the A1200's AA chip set on the A3000's 32-bit
board, with a **68040** on it, **16 MiB of motherboard fast RAM** behind Ramsey,
and an IDE port that is not Gayle's. It boots Workbench 3.1 off the same Rigid
Disk Block whole disk the A600 and the A1200 boot from.

### Primary sources

| Source | Covers |
| --- | --- |
| *Specification for the Advanced Amiga (AA) Chip Set*, Commodore-Amiga 06/07/91 | Alice and Lisa, as the AA sections above read it |
| *Amiga Hardware Reference Manual*, 3rd ed., Appendix D (p. 315) | the 32-bit map: `$0400_0000`–`$07FF_FFFF` "Motherboard Fast RAM", `$0800_0000`–`$0FFF_FFFF` "Coprocessor Slot Expansion", `$FF00_0000` Zorro III configuration |
| *The A3000+ System Specification*, §2.2 (Table 2-2) | Ramsey's two registers, and that version `$0E` or later is the enhanced part an A4000 has |
| *MC68040 User's Manual* | the processor, its on-chip FPU and which instructions it traps rather than computes (`src/cpu/m68k/fpu.rs`) |
| ATA-1 (X3.221-1994), §7 and §9 | the task file's register order, the drive-present probe, and `INTRQ`'s release on a Status read |
| Black-box: Kickstart 3.1 (40.068), 3.X and the A4000T 3.1, all A4000 images | **where the IDE port is**, what the ROM polls, and what the A4000T's ROM wants that this board has not |

The last row is the important one, and the next section is what it means. No
Kickstart was disassembled: what follows is rsemu's own recorder printing the
bus cycles a running machine made, which is the same instrument
`src/dev/amiga/ramsey.rs` and the A600's Gayle identification register were
settled with.

### Finding the IDE port

Commodore's A4000 documentation to hand prints no address for it. So the board
was built with everything *but* the port — the chip set, the CIAs, Ramsey, the
clock, the ROM — and the whole of `$00D8_0000`–`$00DB_FFFF`,
`$00DD_0000`–`$00DD_FFFF` and `$00E8_0000`–`$00EF_FFFF` given to a recorder that
answers zero and writes down every cycle. `amiga-os-310-a4000.rom` finished
autoconfig (34 byte reads, at every even address from `$00E8_0000` to
`$00E8_0042`, finding nothing) and then made exactly three accesses, four times
over:

```text
  W.B  $00DD_203A      ; device 0, then device 1
  R.B  $00DD_2032
  R.B  $00DD_203E
```

That is ATA-1 §9.1's drive-present probe — Device/Head written, Cylinder Low
and Status read — and it fixes the geometry: the three offsets from
`$00DD_2020` are `$12`, `$1A` and `$1E`, which is `4n + 2` for *n* = 4, 6 and 7,
the ATA-1 §7 register numbers of exactly those three. So **`A4`–`A2` are the
drive's `DA2`–`DA0`** and **`A12` is the chip select**, which is the A600's
wiring at a different base.

With the task file modelled the ROM went two steps further and showed the rest:

```text
  W.B  $00DD_2032  $12   R.B $00DD_2032  $12    ; a scratch register, twice
  W.B  $00DD_2032  $34   R.B $00DD_2032  $34
  W.B  $00DD_303A  $00                          ; Device Control: nIEN clear
  W.B  $00DD_203E  $10                          ; RECALIBRATE
  R.W  $00DD_3020  $8000                        ; <- the interrupt register
  R.B  $00DD_203E  $50
  W.B  $00DD_203E  $EC                          ; IDENTIFY DEVICE
  R.W  $00DD_3020  $8000
  R.B  $00DD_203E  $58                          ; DRDY | DSC | DRQ
  R.W  $00DD_2020  x256                         ; <- the data register
```

Three things fall out of those last lines, and two of them were wrong in the
first model:

* **The interrupt register is at `$00DD_3020`, read as a *word*, and only bit
  15 matters.** That is the control block's register-0 slot, which ATA-1 §7.2
  leaves to no drive at all.
* **It is the drive's `INTRQ`, not a latch in front of it.** Modelled as a latch
  that only a write clears, the boot stops dead: the ROM reads `$8000`, reads
  Status — which releases `INTRQ` (ATA-1 §9.5) — and reads `$00DD_3020` again,
  204 216 times in six seconds, because it never writes there. Made the line
  itself, the same ROM reads it once, goes on to `IDENTIFY DEVICE`, and boots.
* **`A1` is not decoded, and the port is byte-swapped.** The eight-bit registers
  are read at the *even* addresses `$…32`, `$…3A`, `$…3E` and the sixteen-bit
  data register as a word at `$00DD_2020` — four bytes lower in the same slot.
  One decode covers both only if the four bytes of a slot are all the same
  register and `A0` alone picks the half of the data bus, with `D15`–`D8`
  carrying the drive's `DD7`–`DD0`. That is Gayle's arrangement exactly, and it
  is also the arrangement that makes `RDSK` come back as `RDSK`: with the swap
  the other way round the Rigid Disk Block would read `DRKS` and nothing would
  mount.

`src/dev/amiga/ide.rs` is the model. It is a class of its own rather than a
property on `amiga.gayle` because an A4000 has no Gayle: a moved-base Gayle
would still carry a PCMCIA slot this board has not, an identification register
its ROM never reads, and — fatally — an interrupt that only arrives once
software writes Gayle's enable register at `$00DA_A000`, which an A4000
Kickstart never does. The decode itself is reproduced from `gayle.rs` rather
than imported, so that an A4000 build links no PCMCIA model.

### The map

| address | what |
| --- | --- |
| `$0000_0000`–`$001F_FFFF` | chip RAM, and the Kickstart behind the overlay until CIA-A's `PA0` goes low |
| `$00BF_D000` / `$00BF_E000` | CIA-B (even lane) and CIA-A (odd lane) |
| `$00DC_0000` | the battery-backed clock, which an A1200 has not |
| `$00DD_2020`–`$00DD_203F` | the IDE command block (`CS1FX-`) |
| `$00DD_3020` | the port's interrupt register |
| `$00DD_303A` | Alternate Status, and Device Control on a write (`CS3FX-`) |
| `$00DE_0003` / `$00DE_0043` | Ramsey's control register and its version, which reads **`$0F`** here and `$0D` on an A3000 |
| `$00DF_F000` | the custom chip registers |
| `$00F8_0000` | Kickstart, 512 KiB, **and nowhere else**: no Gayle means none of its ROM-select mirrors at `$00E0_0000` or `$00A8_0000` |
| `$0700_0000`–`$07FF_FFFF` | motherboard fast RAM, 16 MiB |

Everything else floats: Zorro II space and its autoconfig at `$00E8_0000`, the
coprocessor slot at `$0800_0000`, Zorro III space above `$4000_0000`, and
Gayle's whole world — `$00DA_0000`, `$00DA_8000`, `$00DE_1000`.
`tests/amiga_a4000_board.rs` asserts every row of that table and every one of
those holes, with a ROM built in the file.

### Motherboard fast RAM, and the A3000's `CHK`

The A3000 section above records, as an open bug, that fitting RAM anywhere in
the `$0700_0000` window makes both A3000 Kickstarts relocate `ExecBase` into it
and then take an unexpected `CHK` exception about half a second later and
reboot forever. **That does not reproduce on this tree.** Every row below was
run on `master` at `ebaa03e4`, for 15 to 30 virtual seconds, watching the
program counter, the reset-pulse count, the bus-fault count and `ExecBase`:

| board | processor | ROM | fast RAM | result |
| --- | --- | --- | --- | --- |
| `amiga-a4000` | 68040 | 3.1 (40.068) A4000 | 16 MiB at `$0700_0000` | boots Workbench 3.1 off the IDE port; `ExecBase` `$0700_07F8` |
| `amiga-a4000` | 68030 | 3.1 (40.068) A4000 | 16 MiB at `$0700_0000` | the same, idle at the same address |
| `amiga-a3000` | 68030 + 68882 | 3.1 (40.068) A3000 | 16 MiB at `$0700_0000` | insert-disk screen; `ExecBase` `$0700_07F8`; 0 resets, 0 faults |
| `amiga-a3000` | 68030 + 68882 | 3.1 (40.068) A3000 | 4 MiB at `$07C0_0000` | the same; `ExecBase` `$07C0_07F8` |
| `amiga-a3000` | 68030 + 68882 | 2.04 (37.175) A3000 | 16 MiB at `$0700_0000` | the same |
| `amiga-a3000` | 68030 + 68882 | 2.04 (37.175) A3000 | 4 MiB at `$0700_0000` | insert-disk screen, `ExecBase` **in chip RAM** — see below |

The A3000 rows were run by taking `machines/amiga-a3000.machine`'s own shipped
source, appending a `ram` object and one `map` statement to it, and building
that; the shipped file is unchanged and that is its section's work to do, not
this one's. What is recorded here is the evidence.

The last row is not a failure: it is where the ROM looks. Kickstart's
memory-sizing routine starts at **`$07F7_FFF0`** and works down — the listing is
in `src/dev/amiga/ramsey.rs`, printed by rsemu's disassembler — so a window
filled from the *bottom* with less than 16 MiB has nothing at the address the
ROM probes, and the ROM concludes there is none. A part-populated board has to
be mapped so that it **ends** at `$07FF_FFFF`. `machines/amiga-a4000.machine`
defaults to the full 16 MiB for exactly that reason, and says so where
`fast-ram` is declared.

### How far each ROM gets

`tests/amiga_a4000.rs`, behind `RSEMU_AMIGA_ROM_DIR` and `RSEMU_AMIGA_HDF_DIR`,
boots the user's files in place and checks a frame hash; each frame was looked
at.

| ROM + disk | Reaches | What is on screen |
| --- | --- | --- |
| Kickstart 3.1 (40.068) + `workbench-311.hdf` | **the Workbench 3.1 desktop**, 30 s | 1600×568 in AGA. A light grey backdrop; along the top, "Copyright © 1985-1993 Commodore-Amiga, Inc. All Rights Reserved." with the screen's depth gadgets at the right; below it the open "Workbench" window in its blue bordering, holding the "Ram Disk" icon and, under it, the "Workbench3.1" hard-disk icon; scroll bars and arrows down the right and along the bottom; the red arrow pointer at the top left, where the mouse has not moved |
| Kickstart 3.1, empty bay | its insert-disk screen, and it takes until about 35 s to draw it | A dark purple field; the Amiga check-mark in its blue-to-red gradient above four lines of orange text — "3.1 ROM   40.068 / Copyright © 1985-1993 / Commodore-Amiga, Inc. / All Rights Reserved." — and, to the right, the drive slot with the diskette below it, mid-animation |
| `amiga-os-3x0-a4000.rom` + `workbench-311.hdf` | the same desktop | Identical but for the title bar, which reads "Copyright © 1985-2017 Cloanto Corporation and its licensors." |
| `amiga-os-310-a4000t.rom` + `workbench-311.hdf` | finds the drive, identifies it, and stops | **black** — see below |

`GfxBase->ChipRevBits0` reads **`$1F`** on both booted runs — `GFXF_HR_AGNUS`,
`GFXF_HR_DENISE`, `GFXF_AA_ALICE`, `GFXF_AA_LISA` and bit 4 — and `$13` at the
insert-disk screen, which is the same "the AA pair appears only once the ROM has
a boot device" the A1200 section records, reproduced on a different board.
`ExecBase->AttnFlags` reads **`$807F`** — `AFF_68010 | AFF_68020 | AFF_68030 |
AFF_68040 | AFF_68881 | AFF_68882 | AFF_FPU40` — and `ExecBase` itself is at
`$0700_07F8`, in the motherboard fast RAM. All three are asserted rather than
described.

### The A4000T's ROM wants a chip this board has not

`amiga-os-310-a4000t.rom` gets as far as anything does on the IDE port: it
probes both device positions, runs the scratch-register test, issues
`RECALIBRATE` and then `IDENTIFY DEVICE`, reads all 256 words back — the
identification says `RSEMU` where the model string is — and goes on polling the
drive about eleven times a second for the rest of the run. Then it stops, with
`ExecBase` in fast RAM at `$0700_0810` and a black screen, and it is still there
at 75 s.

The recorder says why. Besides the IDE port, that ROM writes and reads
**`$00DD_0040`–`$00DD_00EE`** — a register file that is neither the IDE port's
nor Ramsey's — and then reads `$00DD_0062` 388 times and gives up. An A4000T has
an **NCR 53C710** SCSI controller on the motherboard in exactly that space,
where an A3000 has its DMAC and WD33C93A, and this board has nothing there. So
that row is the board being honest about what it is: the same disk, the same
port and the same drive boot under both of the other two ROMs. A machine file
for an A4000T would need that controller, which is a separate part and a
separate job.

### Open, and not guessed at

* **How much of `$00DD_2000`–`$00DD_3FFF` the board really decodes.** The model
  answers only where `A5` is high and `A11`–`A6` are low, because that is the
  only place the ROM goes; whether a real A4000 aliases the task file across the
  rest of each page is not established, and nothing tested here can tell.
* **What the port does with a byte access at an odd address.** The ROM never
  makes one. The model puts `D7`–`D0` there, which is Gayle's rule and the
  A600's schematic's.
* **The 53C710 at `$00DD_0040`.** Identified by what the A4000T's ROM does with
  it and by what Commodore put on that board, not by a register table; nothing
  here models it.
* **Whether the A3000's `CHK` was ever the board's.** It does not reproduce, and
  the sweep above is as far as this work can take it without the A3000's own
  file, which belongs to that section.
