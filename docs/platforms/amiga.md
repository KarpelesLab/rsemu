# Commodore Amiga 500

Consumed by: `dev/amiga`, `dev/mos`, `host/display/amiga.rs`,
`host/input/amiga.rs`, `bin/rsemu.rs`, `machines/amiga-a500.machine`,
`machines/tests/amiga-denise.machine`, `machines/tests/agnus-board.machine`,
`tests/amiga_a500_board.rs`, `tests/amiga_denise_board.rs`,
`tests/agnus_board.rs`, `tests/amiga_a500_chipset.rs`,
`tests/amiga_a500_input.rs`, `tests/amiga_adf.rs`,
`tests/amiga_a500_kickstart.rs`; and for the A600 (the last section),
`machines/amiga-a600.machine`, `src/dev/amiga/gayle.rs`,
`tests/amiga_a600_board.rs`, `tests/amiga_a600_hdf.rs`; and for the ECS section,
`machines/amiga-a500plus.machine`, `src/dev/amiga/rtc.rs` and
`tests/amiga_a500plus.rs`.

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
| Commodore's register notes for the AA chip set (the `VPOSR` identification table as transcribed at amiga-dev.wikidot.com) | "8372 (Fat-hr) (agnushr), rev 5 = 22 PAL, 31 NTSC" — the identification this model gives the 2 MiB part |
| *MSM6242B* data sheet (Oki Semiconductor) | The battery-backed clock: register table, the functional description of every register, Tables 1 and 2 |
| *ROM Kernel Reference Manual* structure layouts (`exec/execbase.h`, `exec/nodes.h`, `graphics/gfxbase.h`) | Where a test finds `GfxBase->ChipRevBits0` in guest RAM |

### What an ECS Agnus does (`src/dev/amiga/agnus/ecs.rs`)

| | |
| --- | --- |
| `VPOSR` | "LOF I6 … I0 LOL -- -- -- -- v10 v9 V8": `$20` PAL / `$30` NTSC for an 8372A, `$22` / `$31` for an 8375; `LOL`; `V10`/`V9`. `VPOSW` writes `V10`–`V8` |
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

**The chips are OCS.** The A600 has the ECS 8375 Agnus and 8373 Denise; the
board moves to them when the ECS models land, and every golden below moves
with it.

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
