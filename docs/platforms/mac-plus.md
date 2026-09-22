# Macintosh Plus (1986)

Consumed by: `dev/mac`, `machines/mac-plus.machine`, `host/display/mac.rs`.

A 68000 at 7.8336 MHz, one to four megabytes of memory, 128 KiB of ROM, a 6522
VIA, a Z8530 SCC, an IWM and its 800K drive, a clock chip with twenty bytes of
battery-backed RAM, a keyboard on the VIA's shift register, an NCR 5380 for
SCSI, and a 512 × 342 one-bit screen that is **read out of main memory** by a
counter rather than owned by a video chip. rsemu's first Apple machine, and the
first board here whose framebuffer is somebody else's RAM.

A real ROM runs it to the **blinking insert-disk icon**, polling the drive six
to eight times a second; put an 800K image in and it spins the drive up,
**reads the track**, decodes the two boot blocks out of it, finds no system on
them and puts the disk back out. What it cannot do yet is boot, and that wants
Apple system software on an 800K image — ledger item 1.

## Primary sources

| Source | Covers |
| --- | --- |
| *Guide to the Macintosh Family Hardware*, 2nd edition (Apple Computer, Addison-Wesley 1990) | The whole machine: chapter 3 for the address map, the overlay and the clock chip's three wires, chapter 7 for the keyboard's protocol and its four commands, chapter 9 for GCR and the disk interface, the VIA chapter's port-assignment tables, and the video raster. **It does not carry the drive's register file** — chapter 9's tables are connector signal assignments and circuit diagrams, nothing more, and an earlier draft of this file said otherwise at some cost |
| Neil Parker, *Controlling the 3.5 Drive Hardware on the Apple IIGS*, version 1.00 (February 1994) | The Sony mechanism's **sixteen one-bit status registers** and its control registers, addressed by `CA2`, `CA1`, `CA0` and `SEL`, with the polarity of each. The same mechanism hangs off a Macintosh Plus, and this is the only published listing of it. Its own summary of the polarities is the thing to remember: "the settings of most of these bits are *backwards*: 0 means yes and 1 means no" |
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
| `mac.scc` | the register pointer and all thirty-two registers, `RR0`-`RR3`, the reset commands, `WR9`'s master interrupt enable, and the two carrier detects | any serial traffic, the baud-rate generator, the DPLL, `/WREQ` |
| `mac.iwm` | the sixteen soft switches, the mode and status registers, the write handshake, the drive's sixteen status lines and its control registers, and the **read** data path: a disk shifted past the head a bit cell at a time, with each byte the shifter latches named as a scheduler event so a guest polling the data register cannot miss one | **writing.** A byte written to the data register is kept and goes nowhere, so a disk is read-only however its tab is set. The 400K drive's **PWM speed input** — the mechanism here turns at whatever rate its track length implies and nothing the computer writes changes it |
| `mac.gcr` | Apple's 6-and-2 encoding: the sixty-four disk bytes, the self-sync run, both field marks, the patent's three-byte checksum, the five speed zones and the **gap a formatter leaves**, which is what decides how fast the disk turns (`SECTOR_CELLS`) | the 400K drive's PWM speed control, which an 800K mechanism ignores |
| `mac.disk` | a raw 400K/800K image or a DiskCopy 4.2 container, with its tags, and the block-to-cylinder mapping the zones decide | writing back, and every other container (`.dart`, `.sit`, a nibble image) |
| `mac.mouse` | the one-button mouse: two quadrature pulse trains an axis, `X1`/`Y1` on the SCC's carrier detects and `X2`/`Y2` on the VIA's `PB4`/`PB5`, the switch on `PB3`, and the host seam and record/replay door a person moves it through | the second button a later mouse has; and acceleration, which is the *ROM*'s and is worked around rather than modelled (below) |
| `mac.sound` | the pulse-width circuit: the high byte of each of 370 words in a buffer below the top of memory, one a scan line, the `SNDENB` gate, the three volume bits and `SNDPG2`'s two buffers | the reconstruction filter on the board, whose corner the Guide does not give; and the disk-speed byte beside each sample, which an 800K mechanism ignores |

Not modelled at all: **SCSI** (the NCR 5380 at `$580000`).

## How far a real ROM gets

`tests/mac_plus.rs`, on the stock 1 MiB board with the user's own ROM and
nothing in the drive:

| Virtual time | What happens |
| --- | --- |
| 0 – 25 ms | the VIA is set up: `DDRA = $7F`, `DDRB = $87`, `PCR = 0`, `IER = $82` — vertical blanking enabled — and `PA4` is driven low, so the overlay goes and memory is at zero. The clock chip is asked for parameter RAM `$10` before any of that, 131 φ2 ticks in |
| 25 – 725 ms | the sound buffer is filled a byte per word and `/SNDENB` is asserted: **the startup chime**, 43 frames long, timed by polling `IFR` for the blanking flag some 6,600 times per 25 ms. `mac.sound` plays it — 601.5 Hz, and `--record-audio` writes it out |
| 0.7 – 6 s | the memory test: alternating write and read passes over the whole megabyte, several patterns deep |
| ~6 s | the ROM finds 1 MiB, writes `MemTop`, `BufPtr` and `ScrnBase`, initialises the SCC (32 register writes), exercises the IWM (all sixteen switches, including a mode-register load), reads all twenty bytes of parameter RAM and the clock twice over, finds the battery flat, and **writes its own defaults back** — unlocking the write-protect register with `$55` and locking it again with `$D5` around them |
| 6.9 s | the first keyboard transaction: `ACR = $18`, `SR = $00` to pull the data line low, then `ACR = $1C`, `SR = $16` — Model Number. The keyboard answers `$03` |
| 7.3 s onward | the desktop is painted grey, `_HideCursor` runs, the **insert-disk icon** is drawn in the middle of the screen, `_ShowCursor` puts the arrow back. The 60.15 Hz tick chain runs, `Ticks` at `$16A` counts up, `IFR` is cleared 60 times a second, and the keyboard is asked `$10` — Inquiry — every 0.25 second and answers `$7B`, Null. Which is exactly the cadence chapter 7 describes |
| 7 s onward | the ROM **probes the drive**: the drive-installed line, the number of sides, then the motor on. With a disk in the slot it spins up, checks the spindle speed against the tachometer, **reads cylinder 0** and decodes blocks 0 and 1 out of it — they are in memory by about 10 s — finds no system on them and at about 13 s **puts the disk back out**. With nothing in the slot it goes straight to the insert-disk loop |
| 13 s onward | **the icon blinks** — the floppy with the question mark alternating with the plain floppy, about a second each way — and the drive's *disk in place* line is read six to eight times a second, for ever. That is the loop that notices a disk, and it is the thing that was missing |

`Time` at `$20C` holds the date the clock chip was given plus however long the
machine has been on, which is the check that the counter's byte order is right:
with `time = "2026-01-01T00:00:00"` it reads `$E57B698D` twelve seconds in.

**What the picture shows**: the Macintosh's **50 % grey desktop** — a
one-pixel checkerboard, about 87,300 black pixels of 175,104 — with the
**arrow cursor** drawn over it about fifteen pixels in from the left and
fourteen down, and the **insert-disk icon** in the middle: a white floppy disk
with a black outline and a shutter across its top with a small oval in it, in a
32 × 32 area whose top left corner is pixel (240, 145) and whose outline runs
from row 145 to row 176.

It **blinks**, which it did not use to. The icon alternates between the plain
floppy (799 of those 1,024 pixels white) and the same floppy with a large `?`
in a box on its face (760 white), about a second each way, where the bare
desktop would be exactly half. Both phases are goldens in `tests/mac_plus.rs`
and `the_insert_disk_icon_blinks` is the test that asserts the alternation
rather than either picture.

A third picture — the floppy with a **cross** through it (739 white), the
Macintosh's unreadable-disk icon — is what an 800K image in the drive used to
produce, and it is now a *fault indication* rather than the ordinary case: the
ROM draws it when it cannot get a track off the disk. A disk this board reads
and finds no system on goes back to the blinking question mark instead, which
is what a real Macintosh does with one.

The 4 MiB board reaches the same pictures about twenty-six seconds in, because
its memory test is four times as long.

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
  twice in eight seconds and never again, and moving a carrier detect did not
  get the processor out of the loop. It did something worse; see below.

**And the answer, in the end, was the drive.** The processor was not parked
because the boot had finished; it was parked because the ROM had asked the
drive whether a drive was there, been told no, and had nothing left to do. See
"The drive's register file was invented", below. `$4006E8` is now a loop the
ROM passes through rather than one it stays in.

### The drive's register file was invented

The one that mattered, and the reason the machine sat on the insert-disk screen
for three agents' worth of investigation.

`src/dev/mac/iwm.rs` used to cite the *Guide to the Macintosh Family Hardware*,
chapter 9, for "the drive's own register file: sixteen readable status lines
and four writable controls". **Chapter 9 has no such table.** Its tables are
signal assignments for the twenty-pin and DB-19 connectors and its figures are
circuit diagrams; the mechanism's internal registers are not in the book. The
eight low addresses in that invented table happened to be right, because they
are the classic 400K drive's lines and are widely quoted; the eight high ones
were not.

What settles it is Apple's own note — Neil Parker, *Controlling the 3.5 Drive
Hardware on the Apple IIGS* (1994), "Accessing Disk Drive Status and Control
Bits". The same Sony mechanism hangs off a Macintosh Plus. Its table, as
`CA2:CA1:CA0:SEL`:

```text
   0   0   0   0   step direction          1 = outward, toward track 0
   0   0   0   1   disk in place           0 = a disk is in the drive
   0   0   1   0   disk is stepping        0 = the head is moving
   0   0   1   1   disk locked             0 = write protected
   0   1   0   0   motor on                0 = the spindle is turning
   0   1   0   1   track 0                 0 = the head is over track 0
   0   1   1   0   disk switched           0 = the user ejected a disk
   0   1   1   1   tachometer              60 pulses a revolution
   1   0   0   0   lower head's read line  and selects that head
   1   0   0   1   upper head's read line  and selects that head
   1   0   1   x   (unassigned)
   1   1   0   0   number of sides         1 = double sided
   1   1   0   1   disk ready for reading  0 = ready
   1   1   1   x   drive installed         0 = a drive is connected
```

and the control registers, addressed by `CA1:CA0:SEL` with `CA2` as the data:

```text
   0   0   0   CA2 = 0 step inward,  CA2 = 1 step outward
   0   0   1                         CA2 = 1 reset the disk-switched flag
   0   1   0   CA2 = 0 one step
   1   0   0   CA2 = 0 motor on,     CA2 = 1 motor off
   1   1   0                         CA2 = 1 eject
```

Four things in the old model were wrong, and the measurements that say so:

* **Drive installed**, `CA2:CA1:CA0 = 111`, was unassigned, so it answered the
  cable's pull-up — which is "no drive". A transparent tap over the controller
  counts the ROM reading that one address **82 times** while it works out what
  is on the cable, more than every other status line put together. Answering it
  asserted low is the single change that starts the motor. That is also why a
  previous agent found that reporting "no drive at all" changed nothing: the
  board was *already* reporting no drive, at the address the ROM uses.
* **Number of sides** and **disk ready** were at 10 and 11; they are at 12 and
  13. With them moved, the ROM reads 12 during its probe and waits on 13 before
  it will look for a sector's address field, which is exactly what the note
  says the firmware does.
* **Disk switched** was inverted: the line reads *low* once a disk has been
  ejected, not high. `Iwm::insert` also used to set the flag, which made a
  machine that powered on with a disk already in the drive report that the user
  had just ejected one.
* **Eject** fired on `CA2 = 0`; it is `CA2 = 1`. And the control address is
  `CA1:CA0:SEL`, not `CA1:CA0` — `SEL` is what separates "set the step
  direction" from "reset the disk-switched flag". With the old decode the four
  eject strobes the ROM issues during its probe did nothing at all.

The note's table puts *drive installed* at `SEL` on (address 15) and the
Macintosh Plus ROM reads it at `SEL` off (address 14). The mechanism has one
such line and no way to make it depend on `SEL`, so both halves answer it here.
That is the one place the model goes beyond what the note says, and it is
written down rather than buried.

A fifth defect came out of the same reading. Apple's note says that *reading*
the lower or upper head's line is what configures the drive to use that head —
not merely having the `CA` lines sitting at that address. The model latched the
side on the switch movement, so a ROM walking the sixteen switches on its way
to somewhere else left the drive on the upper head. It now latches on a read of
the status register at that address, and a debug read still changes nothing.

### The tachometer loop and the 2.5 %

The last two defects in the read path, and they were two rather than one.

For three sessions the ROM started the motor and then spent five or six virtual
seconds reading **one** status line — the tachometer, at
`CA2:CA1:CA0:SEL = 0111` — a hundred and forty-seven thousand times a second
from a twelve-instruction loop at `$418B0E`-`$418B36` at interrupt level 3,
reading the data register only forty to a hundred times a second where a
turning 800K disk delivers sixty-two thousand. Then it ejected the disk and
drew the floppy with a cross through it.

**What the loop is** came out of one instrument: a transparent tap over the
controller recording every access as `(cell, switch, value)` and then collapsing
runs of the same switch into one line. The whole thing is one shape, repeated
about a hundred times:

```text
  t=3866333 idx= 1 R val=fe          set CA0
  t=3866333 idx= 3 R val=00          set CA1
  t=3866333 idx= 4 R val=00          clear CA2   — the tachometer's address
  t=3866333 idx=13 R val=bf          set Q6      — read the status register
  t=3866356 idx=14 x5688  cells=19248  senseflips=32
  t=3885610 idx=12 R val=cf          clear Q6
  t=3885671 idx= 1 R val=b3          and round again
```

The reads at switches 1, 3, 4 and 12 are what the "forty to a hundred data
register reads a second" were: every IWM address is a soft switch, so *setting*
`CA0` is a read, and with `Q7:Q6` at `00` it returns the data register as a side
effect. The ROM is not reading the disk in them and never was. The loop is
reads of switch 14 — `Q7` off, with `Q6` already on, so the status register —
and it ends on the **thirty-second transition** of the tachometer, every single
time, having polled about 5,814 times to see them.

**What it is doing with that** came out of the second instrument: diffing the
whole megabyte every forty milliseconds, which is one pass of the loop. In the
steady state exactly three things move — a 24-bit accumulator at `$07FB31`
climbing by `$716E` a pass, a counter beside it at `$07FB37` going 3, 2, 1, 0,
and a retry count at `$001803` walking 8, 7, 6, 5, 4 as the accumulator resets.
That is a **speed measurement, averaged over three samples**, against a count
that starts at eight and walks down; where it stops was not watched all the way,
but the ROM gives up a second or so later. The tachometer is both the thing being measured and the clock the
measurement is timed against, which is why sweeping its rate "only changed how
long the timeout took": it changes both ends at once.

**So it is a speed check with a tolerance**, and the previous session's
retraction was half right and half wrong. The *Guide* is right that "the
double-sided disk drives have internal speed control circuitry and do not use
the disk-speed control signal", so the PWM bytes the ROM writes into the sound
buffer go nowhere — but the ROM still measures, and on real hardware it passes
because the drive's own control holds it at the right speed. Here it failed
because the model's disk turned at the wrong speed and no amount of PWM was
ever going to move it.

**The window, measured.** Sweeping the rate the model turns at and watching for
the ROM to leave the loop and start reading the data register in earnest — the
metric is the longest unbroken run of data-register reads, which goes from 23
to seventy thousand — puts the acceptance window for a twelve-sector cylinder
at:

```text
  385.0 rpm   refused
  386.0 rpm   read
  ...
  401.0 rpm   read
  401.5 rpm   refused
```

That is two per cent either side of a centre of about 393.5, and **394 rpm** is
the figure quoted everywhere for an 800K mechanism's outermost zone. The number
is no longer a recollection: Apple's ROM is what says it.

**And that is what the 2.5 % was.** The IWM shifts a cell every two
microseconds in fast mode, so 500,000 cells a second, and 394 rpm on twelve
sectors is `500000 * 60 / 394 / 12` = 6,345 cells a sector. This encoder laid
down 6,186 — exactly as long as the sector itself, with **no gap** — so every
cylinder was 2.5 % short and every spindle 2.5 % fast, at 404 rpm, three
revolutions a minute outside the window. `gcr::SECTOR_CELLS` is now 6,345 and
`Track::pad_to` fills the difference with the self-sync a formatter writes.

The other four zones follow from that one number with nothing else to get
wrong, because constant linear density is the whole point of a zoned disk:
a cylinder is its sector count times one slot, so it turns at 394 × 12 / *n*.

```text
  zone   cylinders   sectors   cells a revolution   rpm
    0      0 - 15       12           76,140         394.0
    1     16 - 31       11           69,795         429.8
    2     32 - 47       10           63,450         472.8
    3     48 - 63        9           57,105         525.3
    4     64 - 79        8           50,760         591.0
```

which are the speeds quoted for an 800K mechanism, arrived at rather than
copied. **They are derived, not measured**: only zone 0's was put to the ROM,
because a ROM that cannot boot a disk never steps off cylinder 0. Ledger item 1.

### The chip named no event, so the ROM lost one byte in three

Fixing the speed got the ROM into its read loop and no further: it found ten
address-field prologues on the track and could not decode a sector out of any
of them. Recording the bytes the data register actually handed over says why.
Cylinder 0's first address field is eleven bytes on the medium — the `$D5 $AA
$96` prologue, five disk bytes, the `$DE $AA` epilogue and an `$FF` — and for
sector 0 of a double-sided disk those five are `96 96 96 d9 d9`, since
`DISK_BYTES[0]` is `$96` and the format byte `$22` encodes as `$D9`. What the
ROM was handed was

```text
  d5 aa 96  96 d9 d9  aa ff
```

— the prologue, and then one byte in three missing: two of the three `$96`s
and the `$DE`.

The cause is not in the disk and not in the ROM. Recording the chip's own cell
counter against each access shows it advancing in jumps of **0, 6 or 16 cells**
and nothing in between, where a byte is eight cells: 86 % of the ROM's polls
saw no time pass at all and then sixteen cells went by at once, taking two
bytes with them and leaving the second.

`LazyDevice` is exact when the access can be answered at the cycle it happened,
and §4.2 gives two ways to get there: the runnable publishes a live cursor, or
the device **names its next internal event** so the scheduler cuts the round
there (`Scheduler::lazy_deadline` → `natural_target`). The 68000 core does
neither for it — `M68k::run_budget` steps instructions and touches no
`TickCursor` — and `mac.iwm` returned `None`, on the reasoning, written into
the source, that "naming a per-byte event would wake the scheduler fifty
thousand times a second to compute what the next read computes anyway". The
next read computes it *at the position the round reached*, which is the whole
defect.

So the latch is now named as what it is: an internal event, past which a read of
the data register answers differently. `Shared::publish_latch` works out the
cell the shifter will next complete a byte on — arithmetic, not simulation: a
register already holding a one needs only the shifts that carry its highest one
up to bit 7, and an empty one waits for the next one on the medium and then
eight more cells — and publishes it in an atomic, because `next_event_tick` is
asked under the scheduler's own leaf lock and may not take one of ours. It
costs a round per byte, about fifty thousand a second, and only while a disk is
actually turning under a head; the whole `mac_plus` suite still runs in about
five seconds of wall clock in a release build.

With both fixed the same field comes back whole — a later trace caught
`d5 aa 96 96 9b 96 d9 d7 de aa`, prologue, five nibbles and epilogue, for
whichever sector happened to be under the head — and the ROM reads the track
and hands back blocks 0 and 1.

### Two smaller defects found on the way

* **A restored snapshot put the head in the wrong place.** `Iwm::load` wrote
  the restored cell counter into the chip's state and not into the atomic the
  scheduler reads, so a restore left `current_tick` at zero with the state
  somewhere else entirely. It was invisible while the chip named no events; it
  is not invisible now.
* **`insert`, `eject` and `set_sel` changed the medium under the head without
  saying so.** Each now goes through `Shared::invalidate`, which throws the
  cached cylinder away *and* re-announces the next byte's cell.

### The sound circuit, and the rate it runs at

A Macintosh Plus has no sound chip. It has a **pulse-width modulator** fed one
byte per horizontal scan line out of a buffer in main memory, so the whole
device is a second bus master with no registers — the video circuit's shape
exactly, and for the same reason.

Everything about it that is a number was either read out of the *Guide* or
measured through a real ROM:

| Fact | Where from |
| --- | --- |
| one byte a scan line | the Guide's sound chapter |
| the sound byte is the **high** half of each word, the disk-speed byte the low half | the Guide, same chapter; confirmed by the trace below, where the low halves stay `$00` right through the chime |
| the main buffer is `MemTop - $0300`, the alternate `MemTop - $5F00` | the Guide's chapter 3 memory map — and the main one is corroborated by `mac.video`'s own arithmetic: the 896 bytes between the screen buffer's end at `MemTop - $0380` and the top of memory are exactly this buffer and its slack |
| `PA0`-`PA2` volume, `PA3` `SNDPG2` (0 = alternate), `PB7` `SNDENB` (**0 = enabled**) | the Guide's VIA port-assignment tables |
| `$80` is silence | **measured**: after the chime the ROM fills all 370 words with `$80` and leaves them there |
| the volume ladder | **nowhere.** The Guide gives the three bits and not the resistors, so the model is linear in the setting, volume 0 being silence — which is what the Sound control panel does with it. It is written down rather than presented as a measurement |

**The rate is the horizontal line rate**, which `mac.video` already defines:
704 dot clocks of the board's one 15.6672 MHz crystal, so

```text
  15 667 200 / 704  =  244 800 / 11  =  22 254.5454… Hz
```

— not a whole number of hertz, which is what `StreamInfo`'s rational is for.
The machine file gives the circuit `clk / 704`, so **one tick of its clock is
one sample** and `host::audio::mac` reads the rate back out of the clock forest
rather than writing it down twice. 370 samples is exactly one frame of the
raster, which is why `ticks % 370` is the index into the buffer.

**What the ROM puts in it.** A transparent dump of the buffer, frame by frame,
through the chime:

```text
    16 ms  SNDENB=1  vol=7  all 370 words $00
    32 ms  SNDENB=0  vol=7  bytes $06…$FA
   400 ms  SNDENB=0  vol=7  bytes $0E…$C9   — decaying
   688 ms  SNDENB=0  vol=7  bytes $20…$C8
   704 ms  SNDENB=1  vol=7  all 370 words $80
   928 ms  SNDENB=1          $6D $DB $B6 …  — the memory test's patterns
```

which is the gate doing its job twice over: the chime is 30 ms to 690 ms, and
the memory test's patterns that follow are **not** played because `SNDENB` went
back up. A model without the gate would screech through the whole memory test.

And the waveform is periodic with a period of **37 samples**, ten of which fill
the 370-word buffer exactly — which is the design: a waveform whose period
divides 370 is seamless across a frame boundary, so the ROM never has to do
anything at 60.15 Hz but refill. 22 254.5454 / 37 is **601.5 Hz**, and the
recorded WAV measures 602.2 Hz by autocorrelation, which is the whole path —
buffer, gate, volume, resampler, file — agreeing with the bytes in memory.

```sh
rsemu run mac-plus --media macrom=Mac-Plus.ROM --for 2s --record-audio boot.wav
```

`tests/cli_record_audio.rs::a_macintosh_records_its_startup_chime` is the
assertion, and it measures the tone rather than hashing the file.

**Nothing is produced unless somebody is listening.** `record` is off in the
machine file and `--record-audio` switches it on, as `amiga.paula`'s is — with
one reason more than Paula has. This circuit **names a scheduler event per
sample**, because a byte has to be read at the tick its line scanned it and a
round boundary anywhere else would read some of a frame's samples out of the
next frame's waveform. That is the defect `mac.iwm` had, recorded above. It
costs 22 254 rounds a virtual second, a run with no listener pays none of it,
and the recorded and unrecorded runs reach the same state hash — which is the
check that the flag is not guest-visible.

### The mouse

Two quadrature pulse trains an axis and a switch to ground. The *Guide*'s VIA
port-assignment tables give three of the five wires — "PB3 mouse switch
(0 = button down)", "PB4 mouse X2", "PB5 mouse Y2" — and the other two are the
SCC's carrier detects, which is what that chip's two spare inputs are for on
this board.

Everything else about it was measured, because no document says any of it:

| Fact | How |
| --- | --- |
| **channel A is the horizontal axis**, channel B the vertical | a transition on `DCDA` moves the low word of `MTemp` at `$828` and one on `DCDB` moves the high word, and a QuickDraw `Point` is `{vertical, horizontal}` |
| **one count is one X1 transition**, so a full quadrature cycle carries two counts and not four | the SCC's external/status condition is a change either way, and the ROM reads `X2` on the VIA for the direction; `MTemp` moves by one per `DCD` edge |
| the sense of each pair — and **the two axes turn opposite ways** | the phase walking `00 → 01 → 11 → 10` counts `MTemp`'s horizontal coordinate *down* and its vertical coordinate *up*. Which way round a wheel's encoder is mounted is not in any book |
| **one count is one pixel** | 39 counts to the right moved `Mouse` at `$830` from 15 to 54 and moved the arrow's mark in the picture from x = 15 to x = 54 |
| the arrow's mark — the first four-pixel run of ink in its shape — sits at `(h, v + 3)` for a hot spot of `(h, v)` | the same measurement, and it is how the test finds the pointer |
| `MBState` at `$172` is `$00` with the button down and `$80` with it up | pressing it |

**One count is one pixel only because the device is deliberately slow**, and
that is the interesting part. The ROM applies its own mouse scaling:

```text
  counts in one 60.15 Hz tick    1   3   5    6    7    8   16   32
  MTemp moves by                 1   3   5   12   14   16   32   64
```

Six or more in one tick is doubled. A guest that accelerates cannot be pointed
at anything by a host whose cursor is *absolute* — the guest's pointer runs
ahead, pins at an edge, and the two never agree again — so `mac.mouse` delivers
at 208 counts a second, which is 3.5 a tick. The same boundary shows up from
the other side by sweeping the rate rather than the burst size: 100 counts at
294 a second arrive as 118, and at 208 a second they arrive as 100.

And it caps the **distance** rather than each axis's own rate: both axes moving
step at half rate each, because the ROM's threshold is on the two together.
Without that, 100 counts on one axis came through exactly and 100 on each came
through as 180.

The cost is a pointer that crosses the screen in about two and a half seconds.
`-p mousestep=200` gives the speed back and takes the acceleration with it,
which is what a real Macintosh does to a real mouse.

**It is not exact to the count.** 400 counts on one axis move `Mouse` by 398,
at every rate from 2 400 ticks a step down to 6 000 and not at all at 12 000:
the guest counts *interrupts*, and an edge arriving while the processor is in
the level-2 handler with the VIA also waiting is an edge nothing counts. One in
two hundred, and it does not cancel. A sweep into a screen edge puts the two
ends back together — the ROM clamps the pointer to the screen and the host's
cursor stops at the same place — which is what a person does without thinking
about it and what the test does deliberately before each placement.

**What the picture shows.** The insert-disk screen with the arrow cursor
wherever it was sent: send the pointer to (470, 100) and the arrow is drawn in
the upper right, its mark at (469, 103). `RSEMU_MAC_FRAME_DIR` has it.

### The interrupt wiring was a livelock, and the ROM is what says so

The first mouse to move on this board stopped it dead, and the cause was not in
the mouse.

The machine file used to wire the VIA's `/IRQ` straight to `IPL0` and the SCC's
`/INT` straight to `IPL1`, with a comment asserting that "both at once really is
level 3. That is the hardware, not a simplification." Both at once *was* level
3, and level 3 is fatal:

* Vector 27, the level-3 autovector, is at `$6C` in the table the ROM builds in
  RAM, and it points at `$401AB4`. The word there is `$4E73` — `RTE`, per the
  MC68000 user's manual's instruction encodings. The whole handler is "return".
* The level-2 handler, at `$401A84`, runs its entire length at `SR = $2200` —
  mask 2 — which sampling `SR` through it shows. It never raises the mask.

So: the SCC asks, the processor enters the level-2 handler at mask 2, the VIA
asks while it is in there, `IPL` goes to 3, the level-3 exception is taken,
`RTE` returns to mask 2 with level 3 still asserted, and it is taken again. For
ever, with the same six bytes pushed and popped in place. Measured with a mouse
moving: `PC` pinned at `$401AB4` across five thousand samples, `SR = $2300`,
`A7` never moving off `$07FBD4`, the frame there reading `$2200 / $00401A88`,
`Ticks` stopped, and the SCC still asserting. It took about 135 carrier-detect
transitions at 4 ms intervals to hit, and toggling `DCD` directly — no mouse
device involved — wedged it at four of seven rates tried.

An `RTE` at that vector is only a safe thing for Apple to have shipped if level
3 **cannot be asserted**. So the board priority-encodes, which is also what
`cpu.m68k`'s own documentation expects of a board that drives more than one
`IPL` pin:

```text
  SCC   VIA   IPL1  IPL0   level
   -     -     0     0       0
   -     x     0     1       1     the VIA
   x     -     1     0       2     the SCC
   x     x     1     0       2     the SCC, with the VIA still waiting
```

`mac.glue` does it, because that object *is* the board's glue. The VIA's
request is not lost: it is still asserted when the level-2 handler clears the
SCC, and the level falls to 1 rather than to 0. Nothing about the boot changes
— with no mouse the SCC never asks — and every golden in `tests/mac_plus.rs`
held across the change, which is the check that it did not.

The lesson is the one this file keeps relearning, in a new place: **a claim
about the hardware needs evidence, and the ROM is evidence.** "There is no
priority encoder" was written down with confidence and nothing behind it, and
it survived three agents because nothing on this board had ever raised two
interrupts at once.

### The defects this turned up

* **The window fold.** Above. Fixed, and it is what put the icon on screen.
* **`--screenshot` and `--vnc` never worked on this board.** `mac.video` was in
  neither of the two lists the `rsemu` binary keeps — `install_capture` and
  `take_scanout` — so every `rsemu run mac-plus --screenshot` answered "this
  machine has no display" while `machines/mac-plus.machine` advertised
  `--vnc :5900`. The library tests could not see it, because they install the
  capture table themselves. Fixed, with a case in `tests/cli_screenshot.rs`
  that runs the shipped binary against rsemu's own ten-byte stub ROM — the
  second board that file's reason for existing has caught.
* **Two interrupts at once locked the machine up**, which is the one above's
  bigger brother and took a mouse to find. "The interrupt wiring was a
  livelock", above.
* **A carrier-detect transition locked the machine up.** Fixed, and it was
  neither the latch nor the board: it was an early `return`.

  `Reset Ext/Status Interrupts` is command 2 in bits 5-3 of a control write
  with the register pointer at zero. `Shared::write` handled it inside the
  `if pointer == 0` branch and then returned from the function — *before* the
  line at the bottom that re-announces `/INT` on its wire. So the chip's own
  state said it had stopped asking and the wire said it had not, and a
  Macintosh runs `/INT` straight into `IPL1`: the handler did everything the
  Z8530 manual asks of it, returned, and was entered again, for ever.

  A trace of the seven accesses the handler makes says it plainly. Read `RR0`;
  write `WR0 = $02` (Reset Ext/Status); read `RR2` — **`$02`, the status code
  for "channel B external/status change"**, so the vector was right; read `RR0`
  again; `WR0 = $0F` (point high to `WR15`); read `RR15`; `WR0 = $10` (Reset
  Ext/Status again, which is the manual's own advice). After that the chip's
  `ext_ip` is clear and `RR2` reads `$06`, "no interrupt pending". Everything
  the ROM could see was right; only the pin was wrong.

  Every path out of the write now falls through to the refresh, and the
  regression watches the **net** rather than the registers — which is the
  point, because `Scc::irq` used to read the state and so could not see it.
  The old test passed the whole time.
* **The SCC ignored `WR9`'s master interrupt enable.** Found while fixing the
  above: the chip pulled `/INT` whenever a channel had a pending bit and `WR1`
  enabled the condition, with no reference to MIE. The manual gates the pin on
  all three, and `WR9` is the one register the two channels share.
  `Scc::irq` now reports the pin rather than the pending bits.

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

That used to be everything, and it was not enough: the encoder and the decoder
here are each other's oracle, so they would agree with each other even if both
were wrong in the same way about which two bits of a byte go where in a 6-and-2
group or about which of the three sums scrambles which byte.

**Apple's ROM now settles it.** Put an 800K image of 1,600 numbered blocks in
the drive, leave the ROM alone for twelve virtual seconds, and **blocks 0 and 1
of the image are sitting whole in the machine's memory** — the boot blocks,
which is what a Macintosh reads first and all it needs in order to decide there
is no system on the disk. Those 512 bytes cannot be there unless Apple's own
code found the address field, found the data field, denibblized it and checked
the patent's three-byte checksum over it.
`tests/mac_plus.rs::the_rom_reads_a_track_and_decodes_a_sector` is the
assertion; it names the blocks it found so a failure says *which*.

The board **does not boot** and cannot without Apple system software on an
800K image. Nothing of the kind is in this repository and nothing of the kind
will be: the two images this was developed against are 1.44 MB and a Plus
refuses them by name, correctly.

Forging a signature is enough to watch the ROM take a disk seriously. Set bytes
0 and 1 of block 0 to `LK` — the boot block's identifier — and the ROM stops
ejecting: the disk stays in and the motor stays on for at least another twenty
virtual seconds, where a disk without it is put back out at about thirteen.
What it does with the rest of that block is somebody's operating system's
business, so this is an observation about the read path rather than a test, and
it is not one of them.

One consequence, and it is the honest limit on the section above: the ROM only
ever reads **cylinder 0** of a disk it cannot boot, so the four inner zones'
rotation speeds are *derived* rather than measured. "The tachometer loop and
the 2.5 %", above, says from what.

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

1. **Booting one.** The board reads a disk and cannot yet start from one, and
   the only thing missing is an 800K image with Apple system software on it.
   That cannot come from here: the two images this was developed against are
   1.44 MB, which a Plus refuses by name and correctly, and no byte of anybody's
   disk goes in this repository. What *is* here is everything under it — see
   "The tachometer loop and the 2.5 %", above, for how the last two defects in
   the read path were found and what they were.

   The next thing a person with a real 800K system disk would find out is
   whether the **inner zones** turn at the right speed. Cylinder 0's rotation
   is measured against the ROM; the other four are derived from it, and a ROM
   that cannot boot never steps off cylinder 0, so nothing has exercised them
   against Apple's code. A disk that boots would.
2. ~~**The mouse.**~~ Built: `mac.mouse`, and
   `tests/mac_plus.rs::the_pointer_goes_where_it_is_put` finds the arrow in the
   picture where the pointer was sent. See "The mouse", below — and note what
   building it turned up, which was a livelock in the board's interrupt
   wiring that had been latent since the board existed.
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
6. ~~**The sound.**~~ Built: `mac.sound`, and the chime comes out of
   `rsemu run mac-plus --record-audio boot.wav`. See "The sound circuit", above.
   The buffer it reads is also where the ROM writes the disk-speed byte, which
   goes nowhere on an 800K mechanism, so this is for the noise rather than for
   the instrument — which is what it turned out to be worth, because the chime
   is the one thing on this board a person can *hear* is right.

## How the ambiguities were settled

Black-box tracing, which is the tool `CLAUDE.md` names and the only one
available: which addresses the ROM touches, in what order, what it writes, what
it reads back, what it waits on. Everything below came out of it, and no
document states any of it plainly.

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
* **Which address the ROM tests for "is there a drive".** Counting the drive
  register addresses a real ROM reads is what found it: one address, read
  eighty-two times during the startup probe, against twenty-four and twelve for
  the next two and nothing at all for the other thirteen. A histogram of
  *which register* is asked for is a much better instrument on this chip than a
  count of accesses, because every one of the IWM's sixteen addresses is a soft
  switch and the ROM walks through addresses it does not mean to read.

  The general lesson is the one this file keeps relearning: **check what the
  cited source actually says.** The drive register table had a chapter
  reference attached to it and the chapter does not contain a table. Three
  agents worked around the consequences without going back to look.
* **Whether a device's *pin* moved, not just its state.** The carrier-detect
  lock-up was invisible to every register-level test because every register was
  right. A device that publishes through a wire needs at least one test that
  reads the wire.
* **What the ROM *accepts*, swept finely.** The spindle speed was settled by
  running the machine at one rate after another and watching a single number —
  the longest unbroken run of data-register reads — go from 23 to seventy
  thousand. A previous sweep of the same parameter found nothing because it
  doubled each time and the window it was looking for is four per cent wide.
  When a ROM is checking a quantity against a tolerance, the step size is the
  experiment.
* **Collapse the trace before reading it.** Half a million register accesses is
  not a trace anybody can read. Folding runs of the same switch into one line
  with a count, a cell span and how many times the sensed bit flipped turned
  the tachometer storm into eight lines that said what the loop was — and the
  five accesses on either side of it, which had been dismissed as "a trickle of
  data-register reads", turned out to be the loop setting up the `CA` lines.

## Running it

```sh
rsemu run mac-plus --media macrom=Mac-Plus.ROM
rsemu run mac-plus -p ram=4M --media macrom=Mac-Plus.ROM
rsemu run mac-plus --media macrom=Mac-Plus.ROM --vnc :5900   # keyboard aside, the mouse works
rsemu run mac-plus --media macrom=Mac-Plus.ROM --for 15s --screenshot boot.png
rsemu run mac-plus --media macrom=Mac-Plus.ROM --for 2s --record-audio boot.wav
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
