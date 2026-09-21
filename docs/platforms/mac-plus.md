# Macintosh Plus (1986)

Consumed by: `dev/mac`, `machines/mac-plus.machine`, `host/display/mac.rs`.

A 68000 at 7.8336 MHz, one to four megabytes of memory, 128 KiB of ROM, a 6522
VIA, a Z8530 SCC, an IWM and its 800K drive, a clock chip with twenty bytes of
battery-backed RAM, a keyboard on the VIA's shift register, an NCR 5380 for
SCSI, and a 512 × 342 one-bit screen that is **read out of main memory** by a
counter rather than owned by a video chip. rsemu's first Apple machine, and the
first board here whose framebuffer is somebody else's RAM.

A real ROM runs it to the **blinking insert-disk icon**, polling the drive six
to eight times a second; put an 800K image in and it spins the drive up, takes
two runs at reading it, and puts it back out with the unreadable-disk cross.
What it does not do yet is get a track off the disk — the last piece is ledger
item 1.

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
| `mac.iwm` | the sixteen soft switches, the mode and status registers, the write handshake, the drive's sixteen status lines and its control registers, and the **read** data path: a disk shifted past the head a bit cell at a time | **writing.** A byte written to the data register is kept and goes nowhere, so a disk is read-only however its tab is set. The 400K drive's **PWM speed input** — the mechanism here turns at whatever rate its track length implies and nothing the computer writes changes it |
| `mac.gcr` | Apple's 6-and-2 encoding: the sixty-four disk bytes, the self-sync run, both field marks, the patent's three-byte checksum and the five speed zones | the 400K drive's PWM speed control, which an 800K mechanism ignores |
| `mac.disk` | a raw 400K/800K image or a DiskCopy 4.2 container, with its tags, and the block-to-cylinder mapping the zones decide | writing back, and every other container (`.dart`, `.sit`, a nibble image) |

Not modelled at all: the **mouse** (two quadrature phases on the SCC's carrier
detects and two more on the VIA's `PB4`/`PB5`) — buildable now, see the ledger
— the **sound** (the PWM buffer the VIA's `PB7` gates), and **SCSI** (the NCR
5380 at `$580000`).

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
| 7.3 s onward | the desktop is painted grey, `_HideCursor` runs, the **insert-disk icon** is drawn in the middle of the screen, `_ShowCursor` puts the arrow back. The 60.15 Hz tick chain runs, `Ticks` at `$16A` counts up, `IFR` is cleared 60 times a second, and the keyboard is asked `$10` — Inquiry — every 0.25 second and answers `$7B`, Null. Which is exactly the cadence chapter 7 describes |
| 7 s onward | the ROM **probes the drive**: the drive-installed line, the number of sides, then the motor on. With a disk in the slot it spins up, and at about 14 s it gives up on it, **puts it back out** and draws the floppy with a **cross** through it — the unreadable-disk icon. With nothing in the slot it goes straight to the insert-disk loop |
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

Put an 800K image in the drive and a third picture appears: the floppy with a
**cross** through it (739 white), the Macintosh's unreadable-disk icon, drawn
at about 14 s after the ROM has spun the drive up, failed to get anything off
it and ejected it.

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

**What is unproven without a real 800K image** is whether Apple's ROM agrees
with this encoder about the low-level bit assignments: which two bits of each
byte go where in a 6-and-2 group, and which of the three sums scrambles which
byte. The encoder and the decoder here are each other's oracle, so they would
agree with each other even if both were wrong in the same way.

The ROM now gets *close enough to ask*. It probes the drive, starts the motor,
and takes two runs at the disk before it ejects it — but in those runs it reads
only forty to a hundred bytes a second out of the data register, where a head
over a spinning 800K disk delivers sixty-two thousand. It is not reading the
track. It is in a **speed servo** instead, and that is the thing now standing
between this board and a boot: see ledger item 1.

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

1. **The drive's speed, and the PWM servo the ROM runs against it.** This is
   what stands between the board and a boot, and it is the successor to "why
   the insert-disk screen never ends" — that one is answered and fixed.

   With the register file right the ROM probes the drive, starts the motor and
   then spends five to six virtual seconds reading **one** status line in a
   tight loop: the tachometer, at `CA2:CA1:CA0:SEL = 0111`, a hundred and
   forty-seven thousand times a second, from a twelve-instruction loop at
   `$418B0E`-`$418B36` running at interrupt level 3. Between bursts it reads
   the data register forty to a hundred times a second — a trickle, not a track
   read — and after two attempts it ejects the disk and draws the cross.

   **It is servoing the speed.** The low byte of each word in the sound buffer
   is the 400K drive's PWM speed control, and watching `MemTop - $300` through
   the wait shows it moving: `ff ff ff …` before the motor starts, then
   `36 2d 2d 36 2d 36 2d 2d`, then `01 20 20 20 20 01 20 20`, then flat `20`.
   The ROM measures, corrects, measures again — and this drive's speed is a
   property of how many bit cells our encoder put on the cylinder, so nothing
   it writes changes anything and the loop never converges.

   **The tachometer rate is not the answer**, which is worth recording because
   it is the obvious first guess. The model turns track 0 at 404 rpm where a
   real 800K mechanism turns it at 394, and both were tried, along with a sweep
   of the whole apparent range from 101 rpm to 1,616 rpm. Every rate behaves
   the same: the only thing that changes is how long the timeout takes, because
   the timeout is counted in tachometer transitions. At no rate does the ROM
   read more than about eighty-five bytes a second.

   So the next question is **why a Macintosh Plus ROM runs the 400K servo
   against a drive that reports itself double-sided**. Telling it single-sided
   (the "number of sides" line low) changes nothing, and neither does moving
   the unassigned `CA2:CA1:CA0 = 101` line. Two shapes are worth testing next:
   that the Plus ROM always runs the servo and a real 800K drive simply reads
   in range on the first measurement — in which case the target rate is the
   whole of it and the sweep above was measuring the wrong quantity — or that
   something else the ROM reads before it starts (`$401A`-something in low
   memory, or the parameter RAM the clock chip hands back) is what picks the
   400K path.

   The alternative, if that turns out to be a dead end, is to make the cylinder
   the right *length*. A cylinder here is exactly as long as the sectors on it
   — `src/dev/mac/gcr.rs` lays down twelve sectors of 6,186 bit cells and
   stops — with no trailing gap before sector 0 comes round again, which a real
   formatter leaves. So this disk's revolution is 74,232 cells and its rotation
   rate is whatever that works out at against the bit clock, rather than the
   drive's actual speed.

   **Doing that needs a source this board does not yet have.** The zone
   rotation speeds for an Apple 800K mechanism are quoted in various places as
   394, 429, 472, 525 and 590 rpm, and at 500 kbit/s those give revolutions of
   76,142 / 69,930 / 63,559 / 57,143 / 50,847 cells — each about 2.5 % longer
   than what this encoder produces, which is a suspiciously consistent gap and
   is the right shape for an answer. But chapter 9 of the *Guide* does not give
   those figures and neither does Apple's IWM note, and writing a table of five
   numbers into `gcr.rs` on the strength of recollection is **exactly** the
   mistake that cost this board the drive's register file. Find the figures in
   a document first. Until then the tachometer reports the rotation the model
   actually has, which is at least not a lie.
2. **The mouse.** Now buildable: a carrier-detect transition no longer locks
   the machine up, `MTemp` at `$828` moves on both axes — `(15,15)` to
   `(15,14)` for channel A and to `(16,15)` for channel B — and the machine
   goes straight back to its idle loop and keeps counting `Ticks`.
   `tests/mac_plus.rs` asserts exactly that. What is left is a `mac.mouse`
   device driving two phases onto the SCC's carrier detects and two onto the
   VIA's `PB4`/`PB5`, plus the button on `PB3`.
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
6. **The sound**, the last thing on the board with nothing behind it — and now
   also the thing the disk's speed servo writes into, so a `mac.sound` that
   reads the PWM buffer would have a second reason to exist.

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
