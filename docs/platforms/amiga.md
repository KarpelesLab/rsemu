# Commodore Amiga 500

Consumed by: `dev/amiga`, `dev/mos`, `host/display/amiga.rs`,
`host/input/amiga.rs`, `bin/rsemu.rs`, `machines/amiga-a500.machine`,
`machines/tests/amiga-denise.machine`, `machines/tests/agnus-board.machine`,
`tests/amiga_a500_board.rs`, `tests/amiga_denise_board.rs`,
`tests/agnus_board.rs`, `tests/amiga_a500_chipset.rs`,
`tests/amiga_a500_input.rs`, `tests/amiga_adf.rs`,
`tests/amiga_a500_kickstart.rs`.

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
| *A500/A2000 Technical Reference Manual* (Commodore) | Table 6-1 (Fat Agnus's pins: `UDS*`/`LDS*` used only for DRAM), §7.3 (the A2000 PAL equations Gary replaced: the chip data buffers and `/ROME`), the 8520 section |
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
(50.23 %).

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
| **Reading a write-only custom register** | The manual does not say | The last word driven onto the bus — **a placeholder** that Agnus's DMA cycles will replace; `CustomBus::unclaimed` counts every use |
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
512 KiB machine, slow RAM, the clock) and every empty autoconfig slot completes
its cycle with nothing driving the data bus, and nothing documented raises
`/BERR` for an empty address. No manual gives a floating value.
**`unassigned = open-bus`** says exactly that; the 68000 core has no data-bus
latch, so it reads zero, which is also what makes each of Kickstart's probes —
diagnostic ROM, slow RAM, autoconfig, the second half of chip RAM — conclude
"nothing fitted". `tests/amiga_a500_unassigned.rs` walks every range and runs
the user's own ROMs in place behind `RSEMU_AMIGA_ROM_DIR`.

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
  `$DF_F07D` on this board therefore answers the low half of the floating bus,
  which is Appendix C's "whatever value is left over on the bus from the last
  cycle" (p. 299).
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
| AROS (2025-04-22 main ROM) | an alert on the serial port | See below |

**AROS** is blocked by the board, not a chip. Its main ROM alone raises
"graphics.library could not open library hidd" (`$C2038002`) over and over:
the graphics code is in `aros-…-ext.rom`, which wants to be at `$E0_0000`, and
an A500 has no socket there, so the shipped board does not map one. With that
ROM mapped (a scratch board, not committed) AROS loads intuition, then **runs
out of chip memory** on a 512 KiB board (`AvailMem` down to 16 bytes, grey
screen, idle). With `-p chip-ram=1M` it draws its boot picture — a cat's eyes
in the dark, 640 pixels of high resolution, square since the fetch fix below.

That left one AROS question, and the 68000's whole clock answered it: on
that scratch board (extended ROM at `$E0_0000`, `-p chip-ram=1M`), with
`amiga.keyboard` wired to CIA-A, AROS used to stop at a plain grey screen
(`COLOR00` `$AAA`, no bitplane DMA), and without it the picture appeared —
until the scheduler started delivering the event a round ends on (the audio
paragraph above has why), since when it stopped there with or without the
keyboard. Its serial log ended at `romtaginit done` either way. It now draws
the eyes and the logo with the keyboard wired. The shipped board is no
witness: without the extended ROM it draws nothing with or without the
keyboard. What was established on the way:

* **The keyboard's side of the wire checks out** against Appendix G and the
  8520 data sheet: eight positive `CNT` edges a byte, 60 µs a bit, the code
  rotated and inverted into `SDR`, the handshake latched. Kickstart 1.3 and
  AROS both receive `$FD` and `$FE` intact, and 1.3 receives typed keys.
* **What sets it off is timing, not content.** AROS enables the `SP`
  interrupt, reads `ICR` and `SDR`, and pulses `KDAT` low for about a
  millisecond (`CRA` bit 6, timed on CIA-A's timer B) — before its input
  handling is ready. The keyboard, still resynchronising after power-up, takes
  that pulse as the handshake it has been waiting for and sends its power-up
  stream about 1.5 ms later. Any code delivered then wedges AROS: `$FD`, `$FE`,
  `$40` and `$FF` alike. The same codes delivered half a second later — the
  keyboard started after the pulse, or its reply held back 500 ms — and AROS
  boots to "Waiting for bootable media", keyboard working. 143 ms is not
  enough.
* **The wedge is a lost wake-up.** At 60 s nothing is ready: the Exec
  Bootstrap Task is waiting on signal `$00080000` and `input.device` on
  `$003F0000` (read through the RKRM's `ExecBase` and `Task` layouts).
* **It was not the crystal's speed, but it was the processor's.** A crystal
  four times faster wedged the same way, and a keyboard clock 5 % off did too —
  but both kept the 68000 at the half of its clock it used to get on this
  board (see *The 68000 gets its whole clock* above). Given its whole clock,
  which it now has, AROS draws its eyes and its logo **with the keyboard**:
  the same scratch board (the shipped file with `chip-ram = 1M` and the
  extended ROM mapped at `$E00000`), 60 s, reaches the eyes-and-logo picture
  rather than the flat grey it stopped at. So the keyboard was one way of
  moving an interrupt into a window a half-speed processor left open. Taking DF0 off the board, or only its `TK0*`
  wire, also avoids it (AROS's path then differs); taking the mouse off does
  not.
* **Ruled out:** resetting the 8520's shift counter when `CRA` bit 6 flips,
  the one-shot timer-high start, and a spurious level-seven interrupt (Paula
  could flicker its `IPL` pins through seven; fixed, but it never fired here).
  The interrupt levels the processor takes are the same with and without the
  keyboard, apart from the keyboard's own four at level 2.

So the open question is what on this board makes a code arriving in that
window cost a wake-up. Holding the keyboard's reply back would hide it, but a
real keyboard answers just as fast, so that would be a workaround, not a fix.

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
| The copper's write permission | Appendix B's `*`/`~` columns, the original chip set's rule | Appendix C gives ECS a wider one; an A500 Agnus is not ECS |

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
