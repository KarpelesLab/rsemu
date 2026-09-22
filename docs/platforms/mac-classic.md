# Macintosh Classic (1990)

Consumed by: `dev/mac`, `machines/mac-classic.machine`, `host/display/mac.rs`.

A 68000 at 7.8336 MHz, one to four megabytes of memory, **512 KiB of ROM**, a
6522 VIA, a Z8530 SCC, a clock chip with twenty bytes of battery-backed RAM,
the **Apple Desktop Bus** on the VIA's shift register, an NCR 5380 for SCSI, a
**SWIM** and a SuperDrive that reads 1.44 MB media, and the same 512 × 342
one-bit screen a Plus has, read out of main memory by a counter.

rsemu's second Apple machine, and it exists for one reason: a Macintosh Plus
has an IWM and an 800K drive, and the system software people actually have is
on 1.44 MB disks. A Classic reads those. So the Classic is the shortest path to
a Macintosh that boots — and once one boots, its own Finder can author an 800K
disk for the Plus, which is the only thing that writes HFS resource forks and
Finder info correctly (`docs/upstream/fstool-hfs-resource-fork-write.md`).

**Where it is now.** A real Classic ROM runs to the **insert-disk screen** —
the grey desktop, the arrow cursor, the blinking floppy with a question mark —
having reset the Apple Desktop Bus, walked all sixteen bus addresses, found the
keyboard and the mouse, probed the drive, seen that the mechanism is a
SuperDrive, and asked the SWIM for **ISM mode**. ISM mode is not modelled, so
it cannot be handed a sector off the 1.44 MB disk in the slot. **That is the
next thing in the way and the first**, and it is ledger item 1 — what is behind
it is not known, because nothing has got past it to find out.

## Primary sources

| Source | Covers |
| --- | --- |
| *Guide to the Macintosh Family Hardware*, 2nd edition (Apple Computer, Addison-Wesley 1990) | The compact-Macintosh shape this board inherits from `mac-plus`: the address map and the overlay, the VIA's port assignments, the clock chip's three wires, the video raster. **It was not available to this work for the SWIM's ISM registers or for the ADB transceiver's link protocol**, and this file says so wherever a number came from somewhere else instead |
| Neil Parker, *Controlling the 3.5 Drive Hardware on the Apple IIGS*, version 1.00 (February 1994) | The Sony mechanism's sixteen one-bit status registers and its control registers, addressed by `CA2`, `CA1`, `CA0` and `SEL`, with the polarity of each. It leaves `CA2:CA1:CA0 = 101` unassigned; that is where a SuperDrive answers here, and `src/dev/mac/iwm.rs` says so as an extension rather than a reading |
| IBM System 34 / ECMA-147 double-density MFM | The 1.44 MB track layout: `A1A1A1` sync, the ID and data address marks, gap lengths, CRC-16/CCITT. `src/dev/mac/mfm.rs` **derives** the two missing-clock patterns from the encoding rule rather than quoting them, and asserts the derivation in its tests |
| *Synertek SY6522 / Rockwell R6522 Versatile Interface Adapter* data sheet | The chip, and in particular the eight shift-register modes — which is what identifies the ADB link's clock as the transceiver's |
| **Black-box register traces of a real Classic ROM** | Everything else. Every one is written down below, with the trace |

**No Macintosh emulator source was read and the ROM was never disassembled**
(`ROADMAP.md` §1, `CLAUDE.md`). Mini vMac, vMac, Basilisk II, SheepShaver,
Executor and MAME are all off limits. No byte of any Apple ROM or disk image is
in this repository; the tests read the user's own files in place and skip,
saying so, when they are not there.

## The ROM file, and the checksum that only covers half of it

A Macintosh Classic ROM file is **524,288 bytes**, and the socket takes all of
them. The first longword of a Macintosh ROM is its own checksum — the sum of
every 16-bit word after it, modulo 2³² — and on this file that arithmetic comes
out over the **first 262,144 bytes**: `$A49F9914`, exactly the longword stored
there, and the published identifier for a Macintosh Classic. Over the whole
512 KiB it comes out at `$78A05CEA`, which is nothing.

So the checksum reaches half the part. The upper half is **not** a disk image
and not padding: it begins with a `00 01 02 … 0F` table, it carries no HFS boot
block or master directory block anywhere in it, and it is only 4 % zero against
the lower half's 11 %.

What it is *for* was **not** established here, and the honest statement is that
every program counter any trace in this file caught was below ROM offset
`$40000` — the furthest was `$435A1E`, which is offset `$35A1E` — so nothing
measured says the ROM executes up there. `tests/mac_classic.rs` prints the
arithmetic and uses the whole 512 KiB either way, and the check is over bytes it
never keeps.

## The memory map

```text
  $00 0000 - $3F FFFF   memory, repeating; or the ROM while the overlay is up
  $40 0000 - $4F FFFF   ROM, repeating every 512 KiB
  $50 0000 - $57 FFFF   nothing
  $58 0000 - $5F FFFF   SCSI (an NCR 5380) — not modelled; floats
  $60 0000 - $7F FFFF   memory while the overlay is up; nothing afterwards
  $80 0000 - $9F FFFF   the SCC, read
  $A0 0000 - $BF FFFF   the SCC, written
  $C0 0000 - $DF FFFF   the SWIM
  $E0 0000 - $E7 FFFF   nothing
  $E8 0000 - $EF FFFF   the VIA
  $F0 0000 - $FF FFFF   the phase-read space — not modelled; floats
```

**It is the Plus's map.** That is a measurement, not an assumption: the first
thing this board did was run with the Plus's decode and a 512 KiB ROM, and in
the first four virtual seconds the ROM made **zero** accesses to anything the
board does not claim, **204,667** to the VIA at `$EFE1FE`, **384** to the SCC
across both its windows (352 reads in the read window, 32 writes in the write
one), and eleven to the disk controller at `$DFE1FF`. Later, past the ADB
handshake and with a disk in the slot, the controller's count goes to 246. A
map that was wrong anywhere would have shown up in the unassigned counter, and
it reads zero at every point that was measured.

Everything the board does not claim **completes and floats**: the space's
`unassigned` policy is `open-bus`. A compact Macintosh has no bus-error timeout
on those ranges, and this turned out to matter more than it does on a Plus —
see "A word access to the VIA", below.

### The overlay is cleared once and cannot be put back

**The first of the two defects this board found**, and the one that killed it.

`ROMOVERLAY` is the VIA's `PA4`, and the Classic ROM clears it the way a Plus's
does: `DDRA = $18`, `ORA = $EF` (`PA4` high), `ORA = $69` (`PA4` low), seven
accesses into the boot. Memory is then at zero, the ROM sizes it, writes
`MemTop`, `ScrnBase` and `ROMBase`, and builds its exception vector table down
there.

And then, **5.404 virtual seconds in**, while it is working through the disk
controller, it writes `ORA = $D9` — `PA4` **high** again.

On a board that reads the pin as the decode, the ROM is now back over the
machine's own vector table, mid-instruction. What that looks like, from a tap
that verifies every write by reading it back:

```text
  !! 0x07fb58 written 809d, reads back 80a1     the stack has gone read-only
  !! 0x07fb54 written 2004, reads back 0348     and reads back the ROM's bytes
  ...
  0x07fb56 W 0043 / 0x07fb58 W 5578             a return address of $00435578
  0x000cb3 R ff
  0x07fb56 R 2d5c / 0x07fb58 R 80a1             comes back as $2D5C80A1
  <a group-0 exception frame>
  0x00000c R 0034 / 0x00000e R 4efa             vector 3 out of the ROM header
  0x344efa ...                                  and off it goes
```

The frame says it exactly: instruction register `$4E75`, which is `RTS`; access
address `$2D5C80A1`, which is odd; so an **address error**, whose vector at `$C`
is now the ROM's own header longword `$00344EFA`. Two exceptions later the
processor is executing at `$0276309C` — the word at ROM offset 8 with something
after it — and never comes back.

So `mac.glue` grew `overlay = "latching"`: cleared once is cleared until a
reset. A reset still asserts it, because the processor is about to fetch a reset
vector out of whatever answers at zero. The default is `level`, the Plus's, so
nothing about that board changed.

**Which of two hardware stories this is, the measurement does not settle.**
Either the glue latches the bit, or `PA4` is not the overlay at all on a Classic
and something else clears it — the ROM drives `PA4` and `PA5` together around
the disk accesses, and `PA5` is the drive register file's `SEL`, so a second
purpose for `PA4` is not far-fetched. Both stories agree that the overlay goes
away and does not come back, and that is what is modelled.
`tests/mac_classic.rs::the_overlay_is_cleared_once_and_stays_cleared` asserts
it through the address space, because what matters is what answers at zero.

### A word access to the VIA

**The second defect**, found on the way to the first and fixed before it.

`mac.via` accepted byte accesses only, on the argument that "a word access
would read the chip and the floating other half, which is not something to
invent a value for". A Macintosh Classic ROM makes exactly one such access — a
word through a pointer of `$EF_E1FC`, in the same phase as the overlay write —
and refusing it raised a bus error the ROM took through the vector table its own
memory test had just overwritten.

The argument was wrong in its premise rather than its conclusion: **a compact
Macintosh has no bus-error timeout**, `/DTACK` comes from the address decoder
for everything the board claims, and the MC68000 user's manual (§5.4) leaves
`/BERR` to external circuitry a board may omit. So an access to a chip that *is*
fitted cannot fault, whatever its width. The chip drives the even byte — it sits
on the high lane, which is why `$EFE1FE` is even — and the odd half is
`attrs.bus`, the value the space's own open-bus policy delivers, not a value
invented in the device. Anything wider than a word is still refused: a longword
covers two copies of the same register and reading one twice has side effects.

The Plus's behaviour is byte-identical, because its ROM never makes such an
access.

### `PB6` is an output here, and `H4` is not wired

A Plus writes `DDRB = $87` and polls `PB6` for the video circuit's `H4`. The
Classic ROM writes `DDRB = $C7` and then `$F7`, both of which make `PB6` an
**output**, so `machines/mac-classic.machine` does not wire `video.hblank` to
it. A board that drove a signal onto a pin the computer is driving would be a
net with two drivers and no arbiter.

## The Apple Desktop Bus, and it is what the boot waits for

A Plus has a serial keyboard on the VIA's shift register with the Guide's own
four-command protocol. A Classic has **ADB**, and without it the boot stops
dead. The measurement is unambiguous: with the Plus's board and a Classic ROM,
the last thing the ROM ever does is

```text
  via r12 W 00     PCR
  via r2  R c7
  via r2  W f7     DDRB: PB3 is port B's only input
  via r11 W 00
  via r11 W 1c     ACR: mode 111, shift out under an external clock on CB1
  via r10 W 00     SR: the byte the computer wants the transceiver to have
  via r0  R 7f
  via r0  W 4f     ORB: PB5 = 0, PB4 = 0
```

and then, for ever, nothing but the vertical-blanking handler — `IFR`, `IER`,
`IFR = $02`, sixty times a second.

Three things follow with no room for doubt.

* **The clock is the transceiver's.** `ACR = $1C` is the 6522's only shift-out
  mode that takes its clock from CB1 as an *input*; the computer cannot be
  generating it. That is the same argument `src/dev/mac/keyboard.rs` makes for
  the Plus's keyboard, from the same data sheet.
* **A write to the state lines starts a transfer**, because it is the last thing
  the computer does before it waits.
* **`PB3` is the transceiver's attention line and `PB4`/`PB5` are the
  computer's state lines**, because `DDRB = $F7` leaves `PB3` as the only input
  and `ORB = $4F` drives the other two low.

What each state value means was settled by answering one interpretation and
watching what the ROM did next. With `(ST1, ST0) = 0` taken as "the command
byte, computer to transceiver", the ROM's own behaviour falls straight out —
it writes a command, switches `ACR` to shift-in, reads `SR` to arm the counter,
sets state 1, takes a byte, sets state 2, takes another, and sets state 3:

```text
  ACR = $1c ; SR = <command> ; ORB = $4f     state 0: the command goes out
  ACR = $0c ; SR read (arms it) ; ORB = $5f  state 1: the even byte comes back
  SR read ; ORB = $6f                        state 2: the odd byte
  ACR = $00                                  and done
```

repeated sixteen times with commands `$0F, $1F, $2F, … $FF` — **Talk register 3
of every bus address 0 to 15** — after a first command of `$00`, `SendReset`.
That is what confirms the command byte's fields are the Apple Desktop Bus ones:
address in bits 7-4, command in 3-2, register in 1-0. Then `$3C`: Talk 0 of
address 3, the mouse's data register.

### The one that cost an hour: an unanswered Talk still clocks the link

A transceiver that stayed quiet when no device held the address left the ROM
waiting on a shift-register interrupt that never came — at **address 0**, the
first address of its own scan, so the scan never happened.

The transceiver clocks the byte either way. It simply drives nothing, the
pulled-up bus reads as `$FF`, and that is how the computer tells an empty
address from an occupied one: no device's register 3 reads `$FF $FF`. It is also
what the hardware physically does, which is the argument for it rather than the
measurement — the measurement is that the ROM's scan then runs to completion and
reads `$62 $01` at address 2 and `$63 $01` at address 3.

### What is inferred rather than measured

Two numbers, and they are commented as inferences in `src/dev/mac/adb.rs`:

* **The link's bit rate.** 50 µs a bit, 25 of them with the clock low. No
  document here gives it and the measurement cannot: the ROM waits for the
  shift register's interrupt and does not care how long it took. What *is*
  measured is that Apple's ROM accepts these.
* **The keyboard's and the mouse's device handler identifiers**, both 1. The
  ROM asks for register 3 and does not act differently on what comes back.

## The SWIM, and where the 1.44 MB path stops

`mac.swim` is a **superset of the IWM** and is written as one: it owns a
`mac.iwm` with SuperDrive mechanisms on its cable and forwards every access to
it, rather than carrying a second copy of the sixteen soft switches, the drive
register file and the GCR shifter. What it adds is the SuperDrive's own status
line and the ability to take a 1.44 MB image.

Its clock is 1 MHz where the IWM's is 500 kHz, because **one tick is one MFM
cell**: MFM spends two cells on a data bit, so the 500 kbit/s data rate every
high-density 3.5-inch drive runs at is a 1 MHz cell rate. An Apple GCR disk's
cells are half that, so the GCR side of the chip gets every second tick — exact
integer arithmetic inside one oscillator rather than two whose ratio would have
to be written down somewhere else. The two spindle speeds then need no further
number at all:

```text
  GCR, zone 0:   500,000 cells/s / 76,140 cells a revolution = 394 rpm
  MFM:         1,000,000 cells/s / 200,000 cells a revolution = 300 rpm
```

### What the ROM does with a 1.44 MB disk in the slot

246 accesses to the controller's window in thirty virtual seconds. The
interesting ones, from the tap, folded:

```text
  r14 R 00 / r13 R 00 / r8 R 00        the chip reset, read back
  r15 W 17                             the IWM mode register: $17, as a Plus does
  r14 R 17 / r8 R 17 / r13 R 17        and read back
  ...
  r15 W 57                             the mode register again: $57
  r15 W 17                                                      $17
  r15 W 57  x2                                                  $57, twice
  r4  W f5                             switch 4, and the write loads it: $75
  r12 R c0 / r14 R 00 / r9 R 00 ...    and then it reads the file
  r13 R b5 / r14 R b5                  the status register: SENSE high
  r13 R 35 / r14 R 35                  and low — the drive's own lines
  r6  W 80  x25                        and finally, over and over, switch 6
```

`$57` and `$75` both have **bit 6** set, and a Macintosh Plus ROM never writes
that bit at all — it writes `$17` once and nothing else. So this is the Classic
asking its controller for something a Plus's does not have, and it is the ISM
mode switch. `tests/mac_classic.rs::the_rom_asks_the_swim_for_ism_mode` asserts
the sequence so it cannot be lost.

### Why there is no ISM register table here

Because no document available to this work states one, and `CLAUDE.md` forbids
the two other ways of finding out: reading a Macintosh emulator's source, and
disassembling Apple's ROM.

An ISM register file invented to fit the trace would be exactly the mistake that
cost this board's sibling three sessions — `docs/platforms/mac-plus.md`, "The
drive's register file was invented", where sixteen addresses were written down
with a chapter reference and the chapter had no such table, eight of them made
up and five of them wrong. So there is no guess here. What is here is the trace,
the arithmetic, and the honest statement that the chip cannot yet hand the ROM a
sector.

### What *is* here, and is tested

The whole of the medium, which is the half of the problem a format specification
settles:

* `src/dev/mac/mfm.rs` lays down a 1.44 MB track — 80 cylinders × 2 heads × 18
  sectors × 512 bytes, `A1A1A1` sync with the missing clock, ID and data address
  marks, CRC-16/CCITT, and the IBM gap lengths — and reads it back. Both address
  marks are **generated from the encoding rule**, and the tests assert them
  against the hand arithmetic:

  ```text
    $A1 = 1010_0001 encoded normally -> 01 00 01 00 10 10 10 01 = $44A9
          clock between data bits 4 and 5 suppressed            = $4489
    $C2 = 1100_0010 encoded normally -> 01 01 00 10 10 10 01 00 = $52A4
          clock between data bits 3 and 4 suppressed            = $5224
  ```

* Eighteen sectors and their gaps are 11,990 bytes against the 12,500 a
  revolution holds, so a track pads to exactly `CELLS_PER_REVOLUTION` and the
  spindle turns at 300 rpm — the same reasoning `gcr::SECTOR_CELLS` uses for an
  800K disk, where **how fast the disk turns is the track's length**.
* `src/dev/mac/disk.rs` takes a 1.44 MB image, raw or in a DiskCopy 4.2
  container, when a `Reader::Swim` asks for it, and every one of its 2,880
  blocks makes the round trip to MFM cells and back through the disk — which is
  the test that the block mapping and the encoder agree, since those are the two
  halves that could disagree.
* A Macintosh **Plus** still refuses the same image by name, unchanged. That is
  a fact about a Plus's hardware, not a limitation of this model, and turning it
  into a silent success would be worse than the error.

## How far a real ROM gets

`tests/mac_classic.rs`, on the stock 1 MiB board with the user's own ROM:

| Virtual time | What happens |
| --- | --- |
| 0 – 25 ms | the VIA is set up: `DDRA = $18`, `ORA = $EF` then `$69` — `PA4` low, so the overlay goes and memory is at zero — then `DDRA = $7F`, `ORB = $C7`, `DDRB = $C7`, and the clock chip is asked for parameter RAM |
| 25 ms – 5 s | the memory test: alternating write and read passes at interrupt level 7, several patterns deep, with the test pattern on screen. Four times as long on a 4 MiB board |
| ~5 s | the ROM finds 1 MiB, writes `MemTop = $00100000`, `ScrnBase = $000FA700` and `ROMBase = $00400000`, initialises the SCC (32 register writes in both windows), reads the clock chip and writes its own parameter RAM back, and drives `PA4` high again — which is why the overlay has to latch |
| ~5 s | the Apple Desktop Bus: `DDRB = $F7`, `ACR = $1C`, `SendReset`, then **Talk register 3 of every address 0 to 15**, finding the keyboard at 2 and the mouse at 3, then Talk 0 of the mouse |
| ~5 s | the disk controller: the chip is reset, the mode register loaded with `$17`, then the `$57`/`$17`/`$57`/`$57`/`$75` sequence that asks for ISM mode, then the drive's status lines, then switch 6 over and over |
| 6 s onward | the desktop is painted grey, the arrow cursor goes in the top left, and the **insert-disk icon** — the floppy with a question mark — is drawn in the middle. The 60.15 Hz tick chain runs, `Ticks` at `$16A` counts up, and the ROM settles there |

`Time` at `$20C` holds the date the clock chip was given plus however long the
machine has been on, which is the check that the counter's byte order is right:
with `rtcdate = "2026-01-01T00:00:00"` it reads a little above `$E57B6980`.

**What the picture shows**: the Macintosh's 50 % grey desktop — a one-pixel
checkerboard, 87,337 black pixels of 175,104 — with the arrow cursor about
fifteen pixels in from the left and fourteen down, and the insert-disk icon in a
32 × 32 area whose top left corner is pixel (240, 145). 760 of those 1,024
pixels are white where a bare desktop would be 512.

The frame hash is the **same** one `mac-plus` produces for the same phase of the
same blink, and that is not a coincidence worth explaining away: it is Apple's
own icon drawn by Apple's own code into the same place on the same 512 × 342
screen, by two ROMs four years apart.

No access faults, the processor never double-faults, the video circuit produces
60 frames a virtual second throughout, and the space's unassigned counter reads
**zero**.

## The ledger: what to build next, in the order it is likely to matter

1. **ISM mode.** It is the next thing between this board and a booting
   Macintosh, and the only one anything has been able to see; what is behind it
   is unknown, because nothing has got past it. Everything *under* it is here
   and tested: the disk goes in, it
   becomes MFM cells, it turns at 300 rpm under a head the ROM can step, and the
   ROM has already asked for the mode. What is missing is the chip's own register
   file behind that request — the MFM separator, the sector-search engine, the
   parameter registers and the handshake — and it needs either a document that
   states it or a great deal more black-box work than this session had. **Do not
   invent one**; see "Why there is no ISM register table here".

   The measured starting point is exactly this: the ROM writes `$57`, `$17`,
   `$57`, `$57` to the IWM mode register and then `$75`, and every one of those
   has bit 6 set where a Plus's `$17` does not. A chip that switched on that
   sequence and then answered *something* would immediately show, in the same
   tap, which registers the ROM reads next and what it waits for — which is how
   the ADB link in this file was worked out.
2. **Writing to a disk.** The read path is here; the write path is the same
   machinery backwards, plus the controller's write handshake meaning something
   and a way to get the bytes back into the image. A Macintosh that boots wants
   to write to its disk almost immediately.
3. **The NCR 5380.** The ROM makes **zero** accesses to `$580000` in thirty
   virtual seconds, so nothing is waiting on it — but a Classic with an internal
   hard disk is the configuration most of them shipped in, and `src/dev/scsi`
   has the bus, the `Target` trait and a disk.
4. **A host keymap, and the mouse.** `mac.adb` carries a keyboard at address 2
   and a mouse at address 3 and will report a key transition or a movement
   through Talk 0, but nothing turns a host keysym into an ADB key code and
   nothing yet drives the mouse from `host/input`. The Plus's `mac.keyboard`
   cannot be reused for the codes: it speaks the Guide's Table 7-6 transition
   codes, which are a different encoding from ADB's, so one queue cannot serve
   both boards.
5. **The sound**, which the Classic has in the same place a Plus does and which
   neither board models.

## How the ambiguities were settled

Black-box tracing, which is the tool `CLAUDE.md` names and the only one
available. The instruments, in the order they earned their keep:

* **A transparent tap over a device's aperture**, mapped at a higher priority
  than the board's own and forwarding every access unchanged.
  `tests/mac_classic.rs` carries it. Two things about it are load-bearing and
  both were learnt the hard way: a tap must answer with what the device answers
  (a stub that fills a read with `$FF` changes what the ROM does, because the
  value on a floating bus matters on this board), and **a tap must carry the
  mapping's byte order** — a mapping's endianness is not a property of the
  region it points at, so a tap built on `AccessConstraints::ANY` swapped every
  word the processor fetched and the board never started.
* **Fold the trace before reading it.** Half a million accesses is not a trace
  anybody can read. Collapsing runs of the identical access into one line with a
  count turned two hundred thousand VIA accesses into a few hundred lines, and the ADB
  sequence was visible in the last eight of them.
* **A per-register histogram beats a count of accesses.** "The ROM touched the
  VIA two hundred thousand times" says nothing; "it wrote `ACR` seven times with `$00`, `$1C`
  and `$0C`, and `SR` once" says the whole protocol.
* **Verify writes by reading them back.** The overlay defect was invisible in
  the access log — every access looked ordinary — and obvious the moment the tap
  started comparing each write against an immediate debug read of the same
  address. A log of what the processor *asked for* cannot tell a lying log from
  memory somebody else owns.
* **A program-counter histogram, sampled every 200 ns.** Two addresses out of
  20,000 samples is a two-instruction loop, and a two-instruction loop that
  makes no bus access at all cannot be waiting for anything — which is what said
  the ROM had *decided* to stop rather than been left waiting.
* **A ring buffer of the last few hundred distinct program counters**, dumped
  the first time the counter leaves the map. That is what caught `$0276309C`
  and, with a search of the whole address space for that value finding it
  nowhere, said it had been computed rather than read — which pointed at the
  exception frame rather than at a table.
* **Read the exception frame.** The 68000's group-0 frame carries the faulting
  access address and the instruction register. `$4E75` and an odd address named
  the defect in one line, after two hours of looking at everything else.
* **Compare against the sibling board.** Every "is this the Classic or is this
  us?" question was answered against `mac-plus`, whose own page records what a
  Plus ROM does: it writes `DDRB = $87` and polls `PB6` where the Classic writes
  `$C7` and drives it, and `$17` to the mode register where the Classic writes
  `$57`. It cannot be re-asserting `PA4` either, because `mac-plus` runs on a
  *level* overlay and its goldens have not moved. Two ROMs over one board is
  the cheapest differential test there is.

## Running it

```sh
rsemu run mac-classic --media macrom=Classic.ROM
rsemu run mac-classic --media macrom=Classic.ROM --floppy System-Startup.img
rsemu run mac-classic -p ram=4M --media macrom=Classic.ROM
rsemu run mac-classic --media macrom=Classic.ROM --vnc :5900
rsemu run mac-classic --media macrom=Classic.ROM --for 12s --screenshot boot.png
rsemu run mac-classic --media macrom=Classic.ROM -p rtcdate=1990-10-15T09:00:00
```

The ROM image must be exactly 512 KiB. The drive takes a raw 400K, 800K or
1.44 MB image or a DiskCopy 4.2 container of one, and an empty or unbound
`floppy` slot is an empty drive.

The tests read the user's own files in place and skip, printing why, when they
are not there. `RSEMU_MAC_DISK_DIR` is the variable `mac-plus` already
established for a directory of Macintosh disk images — `src/dev/mac/disk/tests.rs`
reads the same one — rather than a second name for the same directory:

```sh
RSEMU_MAC_ROM_DIR=~/retro/macintosh_plus/Macintosh-ROMs \
RSEMU_MAC_DISK_DIR=~/retro/macintosh_plus \
RSEMU_MAC_FRAME_DIR=/tmp/frames \
  cargo test --features machine-mac-classic,display-png --test mac_classic -- --nocapture
```

`RSEMU_MAC_TRACE=1` prints the processor's state once a virtual second, which is
how to find where a ROM stopped. **Three of the nine tests need nothing of
anybody's**: they assemble the board around rsemu's own ten-byte stub and around
a 1.44 MB image of numbered blocks built on the spot, and they are what `cargo
test` runs in CI.
