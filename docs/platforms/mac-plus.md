# Macintosh Plus (1986)

Consumed by: `dev/mac`, `machines/mac-plus.machine`, `host/display/mac.rs`.

A 68000 at 7.8336 MHz, one to four megabytes of memory, 128 KiB of ROM, a 6522
VIA, a Z8530 SCC, an IWM and its 800K drive, a clock chip with twenty bytes of
battery-backed RAM, a keyboard on the VIA's shift register, an NCR 5380 for
SCSI, and a 512 × 342 one-bit screen that is **read out of main memory** by a
counter rather than owned by a video chip. rsemu's first Apple machine, and the
first board here whose framebuffer is somebody else's RAM.

## Primary sources

| Source | Covers |
| --- | --- |
| *Guide to the Macintosh Family Hardware*, 2nd edition (Apple Computer, Addison-Wesley 1990) | The whole machine: chapter 3 for the address map, the overlay and the clock chip's three wires, chapter 7 for the keyboard's protocol and its four commands, chapter 9 for GCR and the disk interface, the VIA chapter's port-assignment tables, the video raster, and the drive's register file |
| US patent **4,564,941**, "Error detection system", Apple Computer Inc. (filed 1983, granted 1986) | The three-byte interleaved checksum on a 400K/800K disk sector: the rotation, the carry chain, and the scrambling of the data with it |
| *Synertek SY6522 / Rockwell R6522 Versatile Interface Adapter* data sheet | The chip: sixteen registers, two ports, two timers, the shift register, the interrupt flag/enable pair |
| *Zilog Z8030/Z8530 SCC* technical manual | The one register pointer, the thirty-two registers per channel, `RR0`-`RR3`, the reset commands |
| Apple, *IWM Specification* (1982) | The sixteen soft switches, the four register pairs `Q7:Q6` selects, the mode register, the write handshake |
| Black-box register traces of a real ROM | Everything the documents leave ambiguous — see "How the ambiguities were settled" |

**No Macintosh emulator source was read and the ROM was never disassembled**
(`ROADMAP.md` §1, `CLAUDE.md`). Mini vMac, vMac, Basilisk II and MAME are all
copyleft and all off limits. No byte of any Apple ROM or disk image is in this
repository; the tests read the user's own files in place and skip, saying so,
when they are not there.

## The ROM file, and its trailing 7,504 bytes

A Macintosh Plus ROM is **131,072 bytes**. The file this board was developed
against is **138,576** — 7,504 bytes longer — and the extra bytes are not part
of the ROM:

* The first longword of a Macintosh ROM is its own checksum: the sum of every
  16-bit word after it, modulo 2³². Over the **first 131,072 bytes** of this
  file that arithmetic comes out at `$4D1F8172`, which is exactly the longword
  stored there. So the ROM starts at offset zero, is 128 KiB long, and is the
  last Plus revision — `$4D1F8172` is the published identifier for it.
* The remaining 7,504 bytes fail to be a continuation of anything. They are
  **84 % zero** — 6,276 of 7,504; what is not zero is a sparse table of 58-byte records holding
  small 32-bit values, a hexadecimal-digit lookup table
  (`ABCDEFabcdef9876543210` against `0a 0b 0c 0d 0e 0f …`), a character
  classification table, and a ramp that walks `0f00, 0f10, 0f20 … 0fff` up and
  back down. That is the **data segment of a compiled program**, not 68000 code
  and not ROM: it has the shape of a C runtime's tables, and nothing in it is
  referenced by any address the machine can generate.

The honest conclusion is that the file is the ROM with something else
concatenated onto it — a dump tool's own image, most likely — and the socket
has nowhere to put it. `tests/mac_plus.rs` takes the first 128 KiB, checks the
ROM's own checksum over them, says how many bytes it left behind, and refuses
the file if the checksum does not hold. That check is arithmetic over bytes the
test never keeps; the sum is a fact about the format rather than any of its
contents.

## The memory map

```text
  $00 0000 - $3F FFFF   memory, repeating; or the ROM while the overlay is up
  $40 0000 - $4F FFFF   ROM, repeating every 128 KiB
  $50 0000 - $57 FFFF   nothing
  $58 0000 - $5F FFFF   SCSI (an NCR 5380) — not modelled; floats
  $60 0000 - $7F FFFF   memory while the overlay is up; nothing afterwards
  $80 0000 - $9F FFFF   the SCC, read
  $A0 0000 - $BF FFFF   the SCC, written
  $C0 0000 - $DF FFFF   the IWM
  $E0 0000 - $E7 FFFF   nothing
  $E8 0000 - $EF FFFF   the VIA
  $F0 0000 - $FF FFFF   the phase-read space — not modelled; floats
```

Everything the board does not claim **completes and floats**: the space's
`unassigned` policy is `open-bus`. A Macintosh Plus has no bus-error timeout on
those ranges, and the MC68000 user's manual (§5.4) leaves `/BERR` to external
circuitry a board may omit. A `fault` policy turns the ROM's first probe of a
chip that is not fitted into an exception the startup code has nowhere to take.

### The overlay

`ROMOVERLAY` is the VIA's **`PA4`**, and it is asserted at power-on because
port A comes out of reset as all inputs and the pin is pulled up. While it is
asserted the ROM answers at `$000000` as well as at `$400000`, and memory
appears at `$600000`; once software clears it, memory is at zero and `$600000`
is not decoded at all.

```text
  window          overlay asserted   overlay cleared
  $00 0000        the ROM            main memory
  $60 0000        main memory        nothing
```

That is the only reason the machine starts: the processor's first two fetches
are the longwords at `$000000` and `$000004`, and the second is a program
counter pointing into `$40xxxx` — so the overlay puts the ROM under the vector
table, and the same part answers at its own address afterwards.

`src/dev/mac/glue.rs` is the decoder. It is a decoder rather than two mappings
swapped, for the reason `amiga.gary` gives: the write that clears the overlay
arrives *through* the address space, so calling `AddressSpace::topology` from
inside it would deadlock against the read guard the access already holds, and
deferring it would apply a whole quantum late.

### Main memory repeats — and *that* is load-bearing

**The one thing in this map that had to be found rather than read**, and it was
got backwards once. Main memory answers again every `ram` bytes all the way to
`$3F FFFF`, because the DRAM is given only the address lines its own depth
needs and a board with less than four megabytes on it leaves `A20`/`A21` out of
the decode.

The evidence is the boot screen. The ROM draws the insert-disk icon through a
pointer of **`$3F CB5E`** — a constant near the top of the four-megabyte
window, not a number derived from `ScrnBase`. A register trace catches `A2`
becoming `$3FCB5E` at `$4007DE`, with `A4` holding `$400FA2` — a pointer into
the ROM — and with no register and no word of low memory holding anything it
could have been computed from, so the number comes out of the ROM itself and
not out of the machine. Folded, that address is `MemTop - $5900 + $245E` on
**every** power-of-two size — `$0F CB5E`
on a 1 MiB board, `$1F CB5E` on 2 MiB, `$3F CB5E` on 4 MiB — which is the
middle of the screen, every time, and 512K, 1 MiB, 2 MiB and 4 MiB boards all
reach the same picture. Unfolded it is in nothing at all on anything
but a 4 MiB machine, and the ROM draws its icon into the void.

That was this board's hang. The picture was the bare grey desktop and the
processor sat in a two-byte loop at `$4006E8`; it had in fact drawn the whole
insert-disk screen, into memory that was not there.

The worry that put the opposite claim here to begin with — that a folded window
makes every machine look like a 4 MiB one, because the ROM sizes memory by
writing at the top of each candidate size and reading it back — does not
survive contact with the ROM. It still writes `MemTop = $00100000` and
`ScrnBase = $000FA700` on the stock board, and `$00400000` / `$003FA700` with
`-p ram=4M`: its sizing pass writes markers at two addresses and compares them,
which is exactly the test an alias fails. Both numbers are asserted in
`tests/mac_plus.rs`, because they are the decoder's behaviour reported back by
the guest.

### The two chips on A9-A12

The VIA and the IWM both have their register selects on **A9-A12**, so each has
sixteen registers 512 bytes apart and each block repeats every 8 KiB through
the window its chip select decodes. The published bases line up with that and
are the check: `VIA` is `$EFE1FE` with `vBufB` at offset `$0000` and `vBufA` at
`$1E00`, so port B is register 0 and port A is register **15**, the
no-handshake address — which is exactly what `(offset >> 9) & 15` gives.

`$EFE1FE` is even and `$DFE1FF` is odd because the two chips sit on opposite
byte lanes of the 68000's word bus. Both models accept byte accesses only and
ignore `A0`: a word access would read the chip and the floating other half, and
there is no value to invent for it.

The SCC is decoded differently again — `A1` picks the channel and `A2` picks
data over control — and it has **two windows** because the board has no
read/write pin for it and decodes the direction from the address instead.

## The devices

| Class | What it models | What it does not |
| --- | --- | --- |
| `mac.glue` | the overlay at zero, the `$600000` window, and memory repeating through the whole four-megabyte window | nothing else; it has no registers |
| `mac.via` | a whole 6522: both ports pin by pin, both timers with the one-shot, free-run and PB7 modes, all eight shift-register modes, `ACR`/`PCR`, and the interrupt flag/enable pair with its read-to-clear and its SET/CLEAR write | PCR's pulse and handshake output modes on `CA2`/`CB2`, which nothing on a Macintosh uses |
| `mac.video` | 512 × 342 one-bit pixels read out of main memory at capture time, the screen buffer hanging below the top of memory with `PAGE2` picking which of the two, and the vertical and horizontal blanking outputs | the cycles it steals from the processor: this board's 68000 runs at its full rate |
| `mac.keyboard` | the Guide's clock/data protocol, its bit timing, the four commands of Table 7-4 and a type-ahead buffer | a host keymap — `Keyboard::key` takes the Guide's own transition code — and the separate keypad's `$79` prefix |
| `mac.rtc` | the four-byte second counter, twenty bytes of parameter RAM, the write-protect and test registers, the three-wire serial interface and the one-second interrupt | the battery: parameter RAM lives and dies with the machine. The 256-byte chip of later models, and its two-byte extended command |
| `mac.scc` | the register pointer and all thirty-two registers, `RR0`-`RR3`, the reset commands, and the two carrier detects | any serial traffic, the baud-rate generator, the DPLL, `/WREQ` |
| `mac.iwm` | the sixteen soft switches, the mode and status registers, the write handshake, the drive's sixteen status lines and four controls, and the **read** data path: a disk shifted past the head a bit cell at a time | **writing.** A byte written to the data register is kept and goes nowhere, so a disk is read-only however its tab is set |
| `mac.gcr` | Apple's 6-and-2 encoding: the sixty-four disk bytes, the self-sync run, both field marks, the patent's three-byte checksum and the five speed zones | the 400K drive's PWM speed control, which an 800K mechanism ignores |
| `mac.disk` | a raw 400K/800K image or a DiskCopy 4.2 container, with its tags, and the block-to-cylinder mapping the zones decide | writing back, and every other container (`.dart`, `.sit`, a nibble image) |

Not modelled at all: the **mouse** (two quadrature phases on the SCC's carrier
detects and two more on the VIA's `PB4`/`PB5`), the **sound** (the PWM buffer
the VIA's `PB7` gates), and **SCSI** (the NCR 5380 at `$580000`).

## How far a real ROM gets

`tests/mac_plus.rs`, on the stock 1 MiB board with the user's own ROM and
nothing in the drive:

| Virtual time | What happens |
| --- | --- |
| 0 – 25 ms | the VIA is set up: `DDRA = $7F`, `DDRB = $87`, `PCR = 0`, `IER = $82` — vertical blanking enabled — and `PA4` is driven low, so the overlay goes and memory is at zero. The clock chip is asked for parameter RAM `$10` before any of that, 131 φ2 ticks in |
| 25 – 725 ms | the sound buffer is filled a byte per word and `/SNDENB` is asserted: **the startup chime**, 43 frames long, timed by polling `IFR` for the blanking flag some 6,600 times per 25 ms |
| 0.7 – 6 s | the memory test: alternating write and read passes over the whole megabyte, several patterns deep |
| ~6 s | the ROM finds 1 MiB, writes `MemTop`, `BufPtr` and `ScrnBase`, initialises the SCC (32 register writes), exercises the IWM (all sixteen switches, including a mode-register load), reads all twenty bytes of parameter RAM and the clock twice over, finds the battery flat, and **writes its own defaults back** — unlocking the write-protect register with `$55` and locking it again with `$D5` around them |
| 6.9 s | the first keyboard transaction: `ACR = $18`, `SR = $00` to pull the data line low, then `ACR = $1C`, `SR = $16` — Model Number. The keyboard answers `$03` |
| 7.3 s onward | the desktop is painted grey, `_HideCursor` runs, the **insert-disk icon** is drawn in the middle of the screen, `_ShowCursor` puts the arrow back, and the processor parks in a two-byte loop at `$4006E8`. From then on it is **steady state**: the 60.15 Hz tick chain runs, `Ticks` at `$16A` counts up, `IFR` is cleared 60 times a second, and the keyboard is asked `$10` — Inquiry — every 0.25 second and answers `$7B`, Null. Which is exactly the cadence chapter 7 describes |

`Time` at `$20C` holds the date the clock chip was given plus however long the
machine has been on, which is the check that the counter's byte order is right:
with `time = "2026-01-01T00:00:00"` it reads `$E57B698D` twelve seconds in.

**What the picture shows at that point**: the Macintosh's **50 % grey
desktop** — a one-pixel checkerboard, 87,337 black pixels of 175,104 — with the
**arrow cursor** drawn over it about fifteen pixels in from the left and
fourteen down, and the **insert-disk icon** in the middle: a white floppy disk
with a black outline, a shutter across its top with a small oval in it, and a
large `?` in a box on its face, in a 32 × 32 area whose top left corner is
pixel (240, 145) and whose outline runs from row 145 to row 176. Seven hundred
and sixty of those 1,024 pixels are white, where the bare desktop would be
exactly half. Nothing else is on it, and it does not change again in thirty
virtual seconds. The 4 MiB board reaches the identical picture — same frame
hash — about twenty-six seconds in, because its memory test is four times as
long.

No access faults, the processor never double-faults, and the video circuit
produces 60 frames a virtual second throughout.

### `$4006E8`, and what it turned out to be

For a long time the picture was the bare grey desktop and this section said the
ROM was *waiting* for something. It was not. It had **already drawn the
insert-disk screen**, into memory that was not there.

The processor lives at `$4006E8`, in a two-byte loop, for 99.2 % of sampled
instants. `SR = $2004`: supervisor, interrupt mask zero. Stepping the machine
200 ns at a time for a virtual second finds **exactly one** vector ever taken —
25, the level-1 autovector, at `$401A42`, about seventy times a second (sixty
vertical blankings plus the keyboard's shift register and the clock chip's
one-second interrupt). Vector 26, the SCC's, is never taken; nor is any other.
The loop itself makes no bus access at all, so nothing can release it but a
handler rewriting its return address, and no handler does: over three virtual
seconds the only bytes that change anywhere in the megabyte are `Ticks`, the low
byte of `Time`, and one keyboard variable.

What it is, is the end of the boot sequence. A ring buffer of the last few
thousand program counters before it settles gives the run-up: a delay loop
polling `Ticks` at `$4007D4`, `_HideCursor` (trap `$A852`, found by looking the
handler address up in the dispatch table the ROM built in RAM at `$C00`), the
icon drawn a row at a time, `_ShowCursor` (`$A853`), two returns, and the loop.
The icon's pointer is `$3FCB5E` — see "Main memory repeats", above — and with
the window folded that is the middle of the screen.

Things ruled out by measurement rather than by argument, each of which cost a
device or a probe:

* **The keyboard is not it.** It completes the Guide's whole handshake — Model
  Number answered, then Inquiry every quarter second.
* **The clock chip is not it.** Parameter RAM is read, found invalid, written
  with the ROM's own defaults and read back — `03 88 00 4c a8 00 00 00 cc 0a cc
  0a 00 00 00 00 00 02 63 00` — and the date reaches `Time`.
* **SCSI is not it.** The space's own unassigned-access counter records
  **zero** accesses to `$500000`, `$580000` or `$F00000` in eight virtual
  seconds. Not three writes: none — the "three writes" this file used to record
  is not reproducible. A ROM waiting on a 5380 would be reading one.
* **The sound is not it.** After the chime the VIA's `ACR` is `$0C` — timer 1 in
  one-shot mode, not free-running — both timers read zero, and `IER = $87`
  enables `CA2`, `CA1` and the shift register and nothing else. There is no
  timer interrupt to wait for and `/SNDENB` is off.
* **The mouse is not it either**, though it is closer than it looks: the ROM
  *has* armed the path — channel A and B both have `WR1 = $01` (external/status
  interrupts on), `WR9 = $0A` (master interrupt enable on) and `WR15 = $08`
  (carrier detect among the external statuses) — but it reads the chip exactly
  twice in eight seconds and never again, and moving a carrier detect does not
  get the processor out of the loop. It does something worse; see below.

### The defects this turned up, and the one still open

* **The window fold.** Above. Fixed, and it is what put the icon on screen.
* **`--screenshot` and `--vnc` never worked on this board.** `mac.video` was in
  neither of the two lists the `rsemu` binary keeps — `install_capture` and
  `take_scanout` — so every `rsemu run mac-plus --screenshot` answered "this
  machine has no display" while `machines/mac-plus.machine` advertised
  `--vnc :5900`. The library tests could not see it, because they install the
  capture table themselves. Fixed, with a case in `tests/cli_screenshot.rs`
  that runs the shipped binary against rsemu's own ten-byte stub ROM — the
  second board that file's reason for existing has caught.
* **A carrier-detect transition locks the machine up.** With the SCC configured
  as the ROM leaves it, driving either `DCD` input makes the chip assert, the
  processor takes vector 26 or 27, and it never comes back: sampling 200 000
  instants afterwards finds **every one** of them at `$401A84` or `$401AB4`, the
  level-2 and level-3 handler entries, and none in the idle loop. The handler
  reads `RR0` on channel B about 740 times in the 200 ms after a single
  transition — no other register, and a transparent tap over both SCC windows
  catches no write in the first two dozen accesses, so it is not obvious that
  anything issues *Reset Ext/Status Interrupts*. `MTemp` at `$828` does move, from
  `(15,15)` to `(16,14)`, so the quadrature is being decoded; the machine simply
  never leaves interrupt level again. Whether the fault is in `mac.scc`'s latch
  or in what the board does with `/INT` is not yet settled, and it is the thing
  to settle before `mac.mouse` is written.

A counting stub has to be **transparent** or it changes what it measures. The
first one here filled a read with `$FF` instead of `attrs.bus`, which is what
the space's `open-bus` policy delivers, and the ROM went off the rails into
floating memory within a second. The value on a floating bus is load-bearing on
this board. The probes that found the fold wrap the decoder's own `MemOps` and
forward every access unchanged, which is the only kind worth writing.

## Booting a disk

The path is there and is tested end to end without one: an image becomes
cylinders of bit cells, the cells are shifted past the drive's head, and the
bytes the IWM's data register hands over decode back into the sectors that went
on. `src/dev/mac/iwm/tests.rs` does exactly that *through the chip* — cylinders
0, 17 and 79, both heads, motor and stepper and all — and
`src/dev/mac/disk/tests.rs` does it for all 1,600 blocks of an 800K image
without the chip.

**What is unproven without a real 800K image** is whether Apple's ROM agrees
with this encoder about the low-level bit assignments: which two bits of each
byte go where in a 6-and-2 group, and which of the three sums scrambles which
byte. The encoder and the decoder here are each other's oracle, so they would
agree with each other even if both were wrong in the same way. The only thing
that settles it is a ROM reading a track this encoder wrote — and the ROM stops
at the insert-disk icon without ever asking whether a disk is there, so even
with an image in hand that test cannot run. Both halves of that are honest and
both are recorded here.

```sh
rsemu run mac-plus --media macrom=Mac-Plus.ROM --floppy System-Startup.dsk
```

takes a raw 400K or 800K image or a DiskCopy 4.2 container. A **1.44 MB image
is refused by name**:

```text
mac.disk: `System Startup` is a DiskCopy 4.2 container of a 1.44 MB disk
(diskFormat 3, dataSize 1474560); a Macintosh Plus has an IWM and an 800K
double-density drive, and 1.44 MB needs the SWIM controller and high-density
media that arrived with the Macintosh SE FDHD in 1989. Give it a 400K or 800K
image instead
```

That is the hardware, not a limitation of this board: chapter 9 of the Guide
lists the 800K drive interface and the FDHD interface as two different
interfaces with two different controllers, and a board that read a 1.44 MB disk
would not be a Plus. The two images this was developed against are both
`diskFormat = 3`; `RSEMU_MAC_DISK_DIR` points `src/dev/mac/disk/tests.rs` at a
directory of them, and it prints what it makes of each and checks the
container's own `dataChecksum` — arithmetic over bytes it never keeps.

## The ledger: what to build next, in the order it is likely to matter

1. **Why the insert-disk screen never ends.** The icon is up, and the ROM then
   sits at `$4006E8` for ever: it does not blink the icon, it does not poll the
   drive for a disk, and putting one in changes nothing. While it waits, the
   only thing it says to the controller is one read of the `ENABLE off` switch
   twice a second — and the chip is left at `switches = $25`, `Q7:Q6 = 00`, so
   that read returns the data register rather than any status line and the ROM
   is not asking whether a disk is there. A transparent tap over the
   controller's whole window records the identical eight accesses in four
   virtual seconds with a disk in the drive and without one. Something is
   supposed to take
   it out of that loop and nothing does. The `$17D6` element on `VBLQueue` is
   walked every blanking interval and its count at `$17E0` is read and written
   back unchanged, which is a task that is never armed: what arms it is the
   next thing to find.
2. **The carrier-detect lock-up** (above). A `DCD` transition puts the machine
   into the level-2/3 handlers permanently. Settle that before writing
   `mac.mouse`, because the mouse is the thing that will drive those pins.
3. **The NCR 5380.** Lower down the list than it was: the ROM makes **zero**
   accesses to `$580000` in eight virtual seconds, so nothing is waiting on it.
   `src/dev/scsi` has the bus, the `Target` trait and a disk when the ROM gets
   far enough to look.
4. **Writing to a disk.** The read path is here; the write path is the same
   machinery backwards, plus the IWM's write handshake meaning something and a
   way to get the bytes back into the image.
5. **A host keymap.** `mac.keyboard` takes the Guide's own transition codes and
   nothing turns a keysym into one. Figure 7-6 has the table; the OCR of it in
   circulation is not reliable enough to transcribe and it wants a clean scan.
6. **The mouse and the sound**, which are the last two things on the board with
   nothing behind them.

## How the ambiguities were settled

Black-box tracing, which is the tool `CLAUDE.md` names and the only one
available: which addresses the ROM touches, in what order, what it writes, what
it reads back, what it waits on. Five things came out of it that no document
stated plainly.

* **The clock chip's whole command encoding.** The Guide gives the three wires
  and sends the reader to *Inside Macintosh* for the rest, which is not a
  hardware document. Reconstructing the serial interface from what the ROM
  writes to `ORB` and `DDRB` gives frames of one command byte and one data
  byte, and the command bytes fall out as a direction in bit 7, an address in
  bits 6-2 and a constant `01` below. The addresses that appear are `$10`-`$1F`
  and `$08`-`$0B` — **twenty bytes**, which is the number the Guide gives for
  this chip's parameter RAM, and nothing else about the trace would have
  produced exactly twenty. `src/dev/mac/rtc.rs` has the whole argument.
* **Which shift-register mode the keyboard uses.** Six of the 6522's eight are
  ruled out by one sentence of chapter 7 — the clock line "is driven only by
  the keyboard" — and the ROM confirms the remaining two by writing `ACR = $1C`
  and then `ACR = $0C`, which are shift-out and shift-in under the external CB1
  clock.

* **Memory repeats through its window** (above). This one was settled **twice**,
  and the first answer was wrong. It was first read off `MemTop` — a data
  structure the ROM builds in memory, which is data rather than code — and an
  early folding decoder that made a 1 MiB machine report `$00400000` was taken
  as proof that the hardware cannot fold. What that really showed was that
  *that* decoder folded wrongly. The second pass watched the address registers
  instead of low memory, caught the ROM loading `$3FCB5E` into `A2` at
  `$4007DE` and drawing its boot icon through it, and the fold fell out of the
  arithmetic: that constant is the middle of the screen on every power-of-two
  memory size **only** if the window wraps. The lesson worth keeping is that a
  low-memory global says what the ROM *decided*, and a register trace says what
  it is about to *do*; the second is the stronger evidence when they disagree.
* **The hang after the chime was the IWM.** With `$C00000` unmapped the ROM
  polled it 119,109 times and then sat in a ten-instruction loop at `$400104`
  for ever, touching nothing. Per-region access counters (`Channel::MMIO`) were
  what showed the polling; the loop itself made no bus access at all, which is
  what made the unmapped window invisible until a counting stub was put over
  it.
* **The vertical blanking net needs a `pull`.** The realize sweep refreshes a
  *resolved* net so every sink learns its idle level, and a per-sink net has no
  level of its own — so a driver whose output already matches a fresh source's
  default announces nothing, and the VIA came up holding the pull-up its pin has
  with nothing wired to it. A snapshot round trip found it: a restore *does*
  deliver the level, so the restored machine and the built one disagreed about
  one bit of `IFR`.

## Running it

```sh
rsemu run mac-plus --media macrom=Mac-Plus.ROM
rsemu run mac-plus -p ram=4M --media macrom=Mac-Plus.ROM
rsemu run mac-plus --media macrom=Mac-Plus.ROM --vnc :5900
rsemu run mac-plus --media macrom=Mac-Plus.ROM --for 15s --screenshot boot.png
rsemu run mac-plus --media macrom=Mac-Plus.ROM --floppy System-Startup.dsk
rsemu run mac-plus --media macrom=Mac-Plus.ROM -p rtcdate=1986-01-16T09:00:00
```

The ROM image must be exactly 128 KiB; trim a longer file first (see above).
The drive takes a raw 400K or 800K image or a DiskCopy 4.2 container, and an
empty or unbound `floppy` slot is an empty drive.

The tests read the user's own ROM in place and skip, printing why, when it is
not there:

```sh
RSEMU_MAC_ROM_DIR=~/retro/macintosh_plus/Macintosh-ROMs \
RSEMU_MAC_FRAME_DIR=/tmp/frames \
  cargo test --features machine-mac-plus,display-png --test mac_plus -- --nocapture
```

`RSEMU_MAC_TRACE=1` prints the processor's state once a virtual second, which
is how to find where a ROM stopped. `RSEMU_MAC_DISK_DIR` points
`src/dev/mac/disk/tests.rs` at a directory of disk images and it says what it
makes of each one. `tests/mac_plus_board.rs` needs nothing of anybody's: it
assembles the board around rsemu's own ten-byte stub and checks that every chip
answers where the Guide puts it.
