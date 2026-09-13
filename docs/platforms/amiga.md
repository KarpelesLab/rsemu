# Commodore Amiga 500

Consumed by: `dev/amiga`, `dev/mos`, `host/display/amiga.rs`,
`host/input/amiga.rs`, `bin/rsemu.rs`, `machines/amiga-a500.machine`,
`machines/tests/amiga-denise.machine`, `machines/tests/agnus-board.machine`,
`tests/amiga_a500_board.rs`, `tests/amiga_denise_board.rs`,
`tests/agnus_board.rs`, `tests/amiga_a500_chipset.rs`,
`tests/amiga_a500_input.rs`.

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
| MC68000 User's Manual (Motorola) | The reset sequence, and the instruction encodings the test ROMs are hand-assembled from |
| The same *Hardware Reference Manual*, for input | Appendix G, "Keyboard Interface" (pp. 357-364): the protocol, timing, handshake, resync, power-up sequence, special codes and the matrix table with every key's legend; chapter 8, "The Keyboard" (pp. 251-254) and "Reading Mouse/Trackball Controllers" and "Mouse Buttons" (pp. 229-233); Table 8-4 (`POTGO`); Appendix A, `JOY0DAT` and `JOYTEST` (pp. 281-282); Appendix E, CIA port assignments; Appendix F, the 8520's serial port and "Bidirectional Feature" |

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
| **Byte access to a custom register** | Every entry is a word and the manual says nothing about a single data strobe | Refused by the region's access constraints |
| **Reading a write-only custom register** | The manual does not say | The last word driven onto the bus — **a placeholder** that Agnus's DMA cycles will replace; `CustomBus::unclaimed` counts every use |
| **`$DFF200`–`$DFFFFF`** | Appendix D gives a 4 KiB window, the table fills 512 bytes, and whether the rest mirrors is not stated | Only 512 bytes mapped; the rest floats like any empty address. `mirror(custom)` is a one-word change |
| **Kickstart 1.x's 256 KiB ROM** | `-p kickstart-size=256K -p rom-base=0xFC0000` fails to build: `amiga.gary` sizes the overlay from `chip-ram` (512 KiB), which is larger than the ROM it forwards to | Not supported yet; the overlay would need to mirror a smaller ROM across its window |
| **A0–A7 in a CIA window** | The notation gives one hex digit of register select; nothing says whether the low byte is decoded | Not decoded — `$BFE003` is register 0 |
| **The ROM base** | The edition's map is `$FC_0000`, a 256 KiB Kickstart; 512 KiB images start at `$F8_0000` | `rom-base` and `kickstart-size` are parameters, defaulting to 512 KiB |

| **Paula's disk bit cell** | "Two microseconds per bit cell" is 7.094 colour clocks, and Paula has no other clock | 7 colour clocks (14 slow): 1.974 µs PAL. The drive spins at the same rate, so a track reads back as written |
| **The drive's spindle speed and index width** | Not in the manual | 100,000 cells a revolution (300 rpm at 2 µs), an index pulse of 1,000 cells |
| **`SERDATR`'s `TBE`** | Chapter 8 says "not a mirror" of `INTREQ`'s; Appendix A says "mirror" | The chapter: the transmit buffer's own state |
| **`MSBSYNC`, precompensation** | One sentence each, and precomp is analogue | Stored and not acted on |
| **`DSKSYNC` out of reset** | The manual does not say | Zero — which matches an idle, zero read line on every cell, so `DSKSYN` is requested until software loads a sync word |
| **Audio's state diagram** | Figure 5-8's arrows are not legible in the available scan | The chapter's prose: see `paula.rs` |
| **Disk images** | ADF is AmigaDOS's format, not the hardware's | The drive's `image` slot takes a raw MFM dump of its own layout; nothing encodes a file system yet |

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
2.04's next refused access is a **byte** read of `$DF_F07D` (the "byte access to
a custom register" row below), which it survives because its vectors are in
chip RAM by then.

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
| A `MOVE` the seam refuses | The copper carries on | Nothing says it stops |
| First sprite control fetch | The first line after table 3-13's vertical blank | The pointers are written "during the vertical blanking interval before the first display" |
| Line-mode texture and `ONEDOT` | `BSH` counts down; the first dot of a row is kept | The manual gives register set-up, not the stepping |
| Bitplane fetch | All of a line's words as it ends | Pointer changes mid-fetch apply to the whole line |
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
for the handshake. **It is not yet confirmed against Kickstart**: neither
Kickstart 2.04 nor AROS reaches keyboard initialisation on this board yet.

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
