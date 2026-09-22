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

**Where it is now.** It **boots**. A real Classic ROM with the user's own
Mac OS 6.0.8 system disk in the drive reaches the **Finder desktop**: the
memory test to about five virtual seconds, the happy Mac at ten, "Welcome to
Macintosh" at fifteen, the menu bar by sixty, and by seventy a desktop with the
startup volume's icon in the top right corner and the Trash in the bottom
right. With an empty drive it draws the insert-disk screen instead, which is
the right answer to nothing to boot from.

Getting there took **ISM mode**, which the previous session measured the
request for and correctly refused to invent a register file behind. The
register file is Apple's, and it is written down: the *SWIM Chip User's
Reference*, revision 1.5. Three things then stood between the document and the
Finder, and all three were measurements rather than readings — which half of a
drive status line says "SuperDrive", how the separator locks onto a sync field,
and how the head gets chosen when the ROM never drives the chip's own
head-select pin. All three are below, with what was measured.

## Primary sources

| Source | Covers |
| --- | --- |
| **Apple Computer, *SWIM Chip User's Reference*, revision 1.5 (11 January 1988)**, with the *SWIM Chip Specification* of 29 September 1987 beside it | **Both register sets.** Page 10's `L7`/`L6`/`MotorOn` table is the IWM's six registers and is what says that `Write Data` and `Set Mode` share an address and are told apart by the drive enable. And **the ISM register set**, the whole of it: the sixteen addresses and A3 as their read/write line, every bit of the mode, setup, phase, error and handshake registers, the parameter RAM and its auto-increment counter, the Trans-Space machine's MFM encoding rule, the correction machine, and the four writes that switch the chip out of IWM mode. `src/dev/mac/swim/ism.rs` quotes the sentence and the page beside every one of them. **Read the page images, not the OCR** — the OCR prints `xOOl` for `x001` and would have made every address in the file a guess |
| *Guide to the Macintosh Family Hardware*, 2nd edition (Apple Computer, Addison-Wesley 1990) | The compact-Macintosh shape this board inherits from `mac-plus`: the address map and the overlay, the VIA's port assignments, the clock chip's three wires, the video raster. **It was not available to this work for the SWIM's ISM registers or for the ADB transceiver's link protocol**, and this file says so wherever a number came from somewhere else instead |
| Neil Parker, *Controlling the 3.5 Drive Hardware on the Apple IIGS*, version 1.00 (February 1994) | The Sony mechanism's sixteen one-bit status registers and its control registers, addressed by `CA2`, `CA1`, `CA0` and `SEL`, with the polarity of each. It leaves `CA2:CA1:CA0 = 101` unassigned, and a SuperDrive answers at the `SEL`-high half of it — which is a **measurement** against Apple's ROM and not a reading, and `src/dev/mac/iwm.rs` carries the three outcomes that settled it. Also the IWM's four registers and the write loop that ends `BVS WLAST ;wait until last data underruns` |
| **Apple Computer, *Software Control of the Disk II or IWM Controller*, 26 April 1984 (revision 1, 10 May 1984)** | The write path: the `Q6`/`Q7` register table, the load-and-shift sequence — "The STA instruction loads the contents of the accumulator into the controller's data shift register… Shifting out the data serially to the disk drive requires Q6L and Q7H" — and the **forty** clock cycles a self-sync byte gets against a data byte's thirty-two, which is where the extra two cells come from |
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

### ISM mode: the document, and the three things it does not say

The register file is Apple's **SWIM Chip User's Reference, revision 1.5
(11 January 1988)**, with the *SWIM Chip Specification* of 29 September 1987
beside it. `src/dev/mac/swim/ism.rs` carries the quotation and the page next to
every register, bit and rule, because the alternative — a chapter reference to
a chapter that turns out to have no such table — is what
`docs/platforms/mac-plus.md` records under "The drive's register file was
invented".

**The mode switch is `1, 0, 1, 1`.** Page 12:

> The *ISM/IWM* bit selects which register set will be used. To select the ISM
> set, you must write to the GCR mode register **four times in a row** with this
> bit set to "1", "0", "1","1", respectively. This somewhat torturous route is
> set up to prevent unintentional intrusions into the ISM world by existing
> software. After the switch, all further accesses to the SWIM will then be
> routed to the ISM register set until you clear bit 6 in the ISM mode register.

So the sequence the previous session measured and could not account for — `$57`,
`$17`, `$57`, `$57`, four consecutive loads of the mode register, bit 6 going
1, 0, 1, 1 — **is the document's own, verbatim**. There was never a tension to
resolve: the fourth write is a "1" and the ROM writes a "1". (A brief that said
the document reads `1, 0, 1, 0` was quoting it wrong; the OCR on archive.org
also reads `1, 0, 1, 1`, and the page image settles it.)

The address map is page 26's table, `IWM State/ISM Register Mapping`, with one
correction its own author made **by hand on the scan**: the printed row for
address 10 says "Read CRC" with `CRC` struck through, and the register's own
section on page 24 heads itself `ERROR Register  R  [1010]`. The per-register
headings are the authority — they are also what says which registers answer at
*both* halves of the address space, by writing `x` for an address bit they do
not care about.

```text
   0/8   DATA        R/W [x000]   (ACTION=1);  8 is CORRECTION when ACTION=0
   1/9   MARK        R/W [x001]
   2     CRC         W   [0010]   (ACTION=1);  IWM Config when ACTION=0
   3/11  PARAMETER RAM  R/W [x011]
   4/12  PHASE       R/W [x100]   reset 11110000
   5/13  SETUP       R/W [x101]   reset 00000000
   6/7   MODE        W   [011x]   6 clears bits, 7 sets them; reset 00000000
   10    ERROR       R   [1010]   reset 00000000; a read clears it
   14    STATUS      R   [1110]   the mode register read back
   15    HANDSHAKE   R   [1111]
```

**The proof that it was read right is the ROM's own parameter RAM.** Having
switched, Apple's ROM loads sixteen bytes into register 3, and among them are
`$41`, `$97` and `$57` — which are MULT = 65, Late/Normal and Early/Normal
exactly as printed on **page 17** for a 15.6672 MHz FCLK. The ROM is loading
the document's own table into the register this file put at address 3.

#### What the document does not say, and had to be measured

**1. Where a SuperDrive answers.** Apple's IIGS note leaves
`CA2:CA1:CA0 = 101` unassigned, and the previous session answered *both* halves
of it — addresses 10 and 11 — from one flag, reasoning that the mechanism has
one such line and no way to make it depend on `SEL`. That is true of *drive
installed*; it is not true here, and it is what kept the board off the Finder.
The differential, same board, same disk, twenty virtual seconds:

| address 10 | address 11 | what Apple's ROM does |
| --- | --- | --- |
| pull-up | pull-up | drives the mechanism as an IWM: spins it up, steps to track 79, reads GCR. A plain 800K drive, and the path a Plus uses |
| pull-up | **asserted** | switches the controller into ISM mode, loads page 17's parameter table, and reads the 1.44 MB disk |
| **asserted** | **asserted** | never touches the mechanism at all — 279 accesses in twenty seconds, no motor, no step, the insert-disk icon for ever |

So **address 11 is "this is a SuperDrive"** and address 10 is a different line
that a SuperDrive does not assert. What address 10 is *for* is not established
and `src/dev/mac/iwm.rs` does not guess: it reads as the cable's pull-up, which
is what the ROM requires and what an unassigned line does.

**2. How the separator locks.** Two decisions, both measured:

* **It syncs on `$A1`'s `$4489` and not on `$C2`'s `$5224`.** A mark search has
  no byte boundary to align to, so it can only look for a bit pattern — and
  `$5224` is not unique at an arbitrary cell offset.
  `swim::tests::only_the_a1_sync_is_unique_at_every_cell_alignment` walks a
  formatted track and finds `$4489` **108** times, which is three per ID field
  and three per data field and nothing else, against **192** hits of `$5224` on
  a track that carries three. Syncing on the second made the chip report an
  index mark eighteen times a revolution, and the ROM — which reads a mark and
  then the address mark behind it — got `$C2` where a sector's `$A1` should
  have been and started over, for ever. Nothing is lost by dropping it: an ID
  or data field is prefixed by `$A1` and only by `$A1`.
* **It takes the *first* of the three `$A1`s**, by requiring sixteen cells of
  `$00` behind the mark. The field's CRC covers all three sync bytes, so a
  separator that locked onto the second or the third seeded its generator a
  byte into the field and the handshake register's bit 1 came back set on every
  sector on the disk — and the ROM put the disk straight back out. The document
  describes the chip locking this way while describing the correction machine,
  page 19: *"The CSM looks for 32 pairs of minimum cells which coincidently
  show up in a run of zero bytes, such as a sync field. After that it looks to
  see if the first non-minimum cell belongs to a mark byte. If not, it starts
  looking for minimum cells again."*

**3. How the head gets chosen.** Page 23 makes `HDSEL` drive its pin only when
the Setup register's bit 0 says so, and a Classic ROM **never sets that bit** —
it writes `$20` to Setup and nothing else. What it does instead is drive the
phase lines to `CA2:CA1:CA0 = 100` and read the handshake register, because
selecting the head is the *drive's* rule and not the controller's: Apple's note
says "Instantaneous data from lower head. Reading this bit configures the drive
to do I/O with the lower head". So a read of the handshake's `SENSE` bit picks
the head, exactly as a read of an IWM's status register does. Until it did, the
ROM read cylinder 0 head 0 for ever and never reached the catalogue.

#### What is inferred rather than quoted

Four, and each is commented as an inference where it is written:

* **"Four times in a row" means four consecutive loads of the mode register**,
  with reads of other addresses allowed in between. The stricter reading also
  accepts the ROM's sequence, which has nothing at all between its four, so
  nothing measured distinguishes them.
* **The CRC generator's seed is `$FFFF`.** Page 23 says the Clear FIFO bit
  "initializes the CRC generator with its starting value" and that "this value
  is different for reading or writing", without giving either. `$FFFF` is the
  preset the format specifies, and it is *checkable* rather than assumed: a
  field followed by its own CRC leaves the generator at zero only for the right
  preset, which is what the handshake register's bit 1 reports and what
  `the_separator_reads_an_id_field_and_its_crc_comes_out_zero` asserts.
* **The generator is fed where bytes are framed**, not where the processor
  reads them. Page 25 says the bit reports the CRC "on the bytes up to and
  including the byte about to be read" and is "usually checked when the second
  CRC byte is about to be read from the FIFO" — by then the generator must have
  absorbed *both* CRC bytes, and only one tapped at the medium has.
* **A bus write names its register with the low three address bits**, A3 being
  documented as "the read/write line for the registers" and so saying nothing
  further about a write. Reads decode all four bits, because four registers —
  CORRECTION, ERROR, STATUS and HANDSHAKE — exist only in the high half.

Addresses 6 and 7 have no documented read function; this model answers zero and
**completes the access**, because a compact Macintosh has no bus-error timeout
and a chip that is fitted may not fault.

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
| ~5 s | the disk controller: the chip is reset, the mode register loaded with `$17`, then `$57`, `$17`, `$57`, `$57` — the four writes that ask for **ISM mode**, bit 6 going 1, 0, 1, 1 — and then, in ISM mode, `$F5`, `$F6`, `$F7` into the phase register with each read back, which is the ROM satisfying itself that this is a SWIM. It then clears mode bit 6 and goes back to being an IWM while it looks at the mechanism |
| 6 – 9 s | the drive: `installed`, `sides` and the SuperDrive line on both cable positions. Finding a SuperDrive on drive 1 is what sends the ROM down the high-density path; finding an 800K mechanism there instead sends it down the IWM's, and it reads GCR perfectly well that way |
| 9 – 20 s | ISM mode again, and this time for real: sixteen bytes of parameter RAM — `$1B`, `$41`, … `$97`, `$57`, page 17's own table — `Setup = $20`, `MotorOn` and drive 1, the Clear FIFO toggle, `ACTION`, and the boot blocks come off cylinder 0. The **happy Mac** is on the screen by ten seconds |
| 15 s | "Welcome to Macintosh", and the System loading behind it: the head steps out across the disk and the ROM reads whole sectors, `$A1 $A1 $A1 $FE C H R N` and the field behind each |
| 60 s | the Finder's **menu bar** — the Apple, *File*, *Edit*, *View*, *Special* |
| 69 s | the startup volume's icon lands in the top right corner, and the ROM strobes the drive register file's eject (ledger item 1) |
| 70 s onward | the desktop is finished and stops changing: every frame from here to two virtual minutes hashes the same |

`Time` at `$20C` holds the date the clock chip was given plus however long the
machine has been on, which is the check that the counter's byte order is right:
with `rtcdate = "2026-01-01T00:00:00"` it reads a little above `$E57B6980`.

**What the picture shows at seventy seconds**: the menu bar across the top of
the screen, white but for the Apple and the four menu titles — 9,165 of its
10,240 pixels lit, where the bare desktop below is *exactly* half; the arrow
cursor in the top left; the startup volume's floppy icon with **System
Startup** under it in the top right corner, 65.1 % white in a box that measures
50.0 % on the insert-disk screen and 49.9 % on "Welcome to Macintosh"; the
Trash in the bottom right; and the Macintosh's one-pixel checkerboard between
them, 82,909 black pixels of 175,104.

**With an empty drive** it is the insert-disk screen instead: the grey desktop,
the arrow cursor, and the blinking floppy in a 32 × 32 area whose top left
corner is pixel (240, 145).

No access faults, the processor never double-faults, the video circuit produces
60 frames a virtual second throughout, and the space's unassigned counter reads
**zero**. Two virtual minutes of it cost about twenty seconds of wall time,
which is faster than the machine being emulated.

## The write path

**A disk can be written now**, and the thing that proves it is not a test this
project wrote: **Mac OS 6.0.8 writes to its own startup volume while it brings
the Finder up**, and a block of the image comes back changed.

The protocol, from the trace, is the document's:

```text
  ISM wMode0  W 18     clear ACTION and the read/write bit
  ISM wMode1  W 10     set the read/write bit to *write*
  ISM wMode1  W 01     Clear FIFO high
  ISM wMode0  W 01     and low: the FIFO is empty and the CRC is seeded
  ISM wData   W 00 x2  prime it
  ISM wMode1  W 08     ACTION: the head starts laying cells down
  ISM rHandshake R da  and from here it is poll, write, poll, write
  ISM wData   W 00     the sync field
  ISM wMark   W a1 x3  three marks - a byte with a clock pulse missing
  ISM wData   W fb     the data address mark, then 512 bytes
  ISM wCRC    W ff     the *chip's* CRC, two bytes, which nobody computed
  ISM wData   W 4e x4  the gap, so the splice lands in it
  ISM wMode0  W 18     and out of write mode
```

`src/dev/mac/iwm.rs`'s `Writer` is the whole of it: a two-byte buffer, a
shifter that spends eight cells on a GCR byte and sixteen on an MFM one, and a
head that lays a cell down every tick where the read path takes one up. The
cells go into the *same* cached cylinder the read path shifts past the head,
and the cylinder is decoded back into the image by the *same* decoder the read
path is tested against — so a write that did not come out as a readable field
with a good CRC is not absorbed at all, and the image keeps what it had.

Three things in it are worth naming.

* **What the head lays down when the processor is late is an inference**, and
  it is where a self-sync byte comes from. Apple's *Software Control of the
  Disk II or IWM Controller* (1984) spends **forty** 6502 cycles on a `$FF`
  sync byte where a data byte gets **thirty-two**; eight cycles are two bit
  cells, and what is on the disk is eight ones and two zeros. So a byte
  boundary with an empty buffer writes a cell with no transition in it. Neil
  Parker's IIGS note confirms that an underrun is the *ordinary* end of a
  write — `BIT Q6 / BVS WLAST ;wait until last data underruns` — rather than a
  fault.
* **The CRC generator is preset at the first mark byte of a field**, which is
  also an inference, and it is the one defect that cost real time. A generator
  running from the Clear FIFO toggle absorbs the sync field of `$00`s the
  processor writes in front of the marks, so every field on the disk reads back
  bad — which is exactly what happened: Apple's code wrote, our decoder
  rejected every field, and **zero** blocks changed while the Finder quietly
  went wrong. The read side's own rule is the argument: `super::mfm` says the
  CRC "covers the three sync bytes as data — `$A1 $A1 $A1` — then the address
  mark, then the field", and on the read side that falls out of where framing
  begins. On the write side the chip is *told* which bytes are marks, which is
  what the ISM's Write Mark register is for.
* **A CRC byte is whatever the generator holds when the head reaches it**, not
  when the processor asked for it: bytes handed over earlier may still be in
  the buffer, and the CRC has to cover them. So the buffer holds a *kind* per
  slot rather than only a value.

**The register decode had to be corrected to get there**, and the correction is
Apple's own table, *SWIM Chip User's Reference* page 10:

| L7 | L6 | MotorOn | Register (State Name) |
| --- | --- | --- | --- |
| 0 | 0 | 0 | Read All Ones |
| 0 | 0 | 1 | Read Data |
| 0 | 1 | X | Read Status |
| 1 | 0 | X | Read Write-Handshake |
| 1 | 1 | 0 | Set Mode |
| 1 | 1 | 1 | Write Data |

> Reading from a register must be done from a "0" state (L7=0, L6=0 or
> MotorOn=0). Writing to a register must be done from a "1" state (L7=1, L6=1
> or MotorOn=1).

This model used to put `Write Data` at `L7=1, L6=0` — which is the *handshake*
— and the mode register at `[11X]` whatever `MotorOn` was. `MotorOn` is the
third address bit, and it is the same latch the sixteen soft switches call
`ENABLE`. Apple's IIGS note says the same thing from the software side: "the
write to the mode register will fail unless the drive is fully deactivated".

**The boot is eight virtual seconds longer** than it was, and that is the write
path being real: a sector takes the time a sector takes. The desktop at the end
hashes the same.

**Nothing writes to anybody's file.** The medium lives in the `Disk` the media
slot handed the drive, and `rsemu run` never writes a floppy image back out.
A disk that *has* been written is carried in the snapshot, because a restore
cannot rebuild it from the machine file the way an untouched one is rebuilt.

## The second drive, and what a Macintosh does with a blank disk in it

The route this file opens with — a Macintosh that boots authors an 800K disk
for a Plus — needs the machine to have somewhere to write, so `mac.swim` and
`mac.iwm` take an `image2` property for the second mechanism on the cable.
**No shipped machine file names it**, and that is `machine::realize`'s rule
rather than a decision here: a slot a board names and nothing binds is an
error, so `image2 = "floppy2"` in `mac-classic.machine` would mean every test
that assembles the board has to bind zero bytes for it, and one of them belongs
to another subsystem. A test that wants a disk in the external drive puts it
there through `Swim::insert`, which is what the property does anyway.

With `-p drives=2` and 819,200 zero bytes in the second drive —
formatted cells with no HFS volume on them — Mac OS 6.0.8 boots, mounts its own
startup volume, and at about ninety virtual seconds puts up

```text
  This disk is improperly formatted for use in this drive. Do you want to
  initialize it?                                   [ Eject ]   [ Initialize ]
```

which is the machine offering to do exactly what this route needs — and the
pointer can now click it. `a_blank_disk_in_the_second_drive` drives the whole
dialogue:

```text
  This disk is improperly formatted for use in this drive. Do you want to
  initialize it?                                   [ Eject ]   [ Initialize ]
      -> This process will erase all information on this disk.
                                                   [ Cancel ]  [ Erase ]
      -> Please name this disk:  [ Untitled ]      [ OK ]
      -> Formatting disk...
      -> Initialization failed!                    [ OK ]
```

**Three clicks land, the Macintosh steps the head to cylinder 40 and starts
laying a format down, and then it gives up.** Two defects were found on the way
to that and both are fixed; a third is where this pass ends.

**The Clear FIFO toggle owns the write buffer, and nothing else does.**
*SWIM Chip User's Reference*, page 23: "Toggling the clear FIFO bit high then
low clears the FIFO to begin a read or write operation, and initializes the CRC
generator with its starting value." A Macintosh toggles Clear FIFO, **then
primes the FIFO with two bytes**, and only then sets `ACTION`. This model
emptied the buffer when `ACTION` rose, threw those two bytes away, and started
every write with the head already late — and one underrun is all it takes,
because page 24 says "When any of the bits is set, the Error bit in the
Handshake register will also be set" and the formatter polls that bit **9,837
times a track without ever reading the ERROR register that would clear it**
(the trace counts zero reads of address `1010`). With the toggle owning the
buffer, the underrun count over a whole format is **0** and the error register
reads `$00` throughout.

The ERROR register itself was already read-to-clear, which page 24 requires —
"The register is cleared by either reading it or resetting the chip" — and a
debug read already left it alone. What was wrong beside it is that the ISM took
the IWM's underrun flag as a **level**: page 11 makes that flag sticky for the
*IWM's* handshake register, so a level put the ERROR bit straight back the
instant the processor cleared it. It is an edge now, and the two registers keep
their own rules.

**The first copy of a sector on a track wins.** `super::gcr`'s decoder has had
that rule all along; `super::mfm`'s had not. A head that lays a field down
somewhere other than where the old one was leaves *both* on the medium, and
taking the later one hands back exactly the bytes the write was meant to
replace. `swim::tests::a_sector_the_chip_formats_comes_back_out_of_the_image`
is the hermetic reproduction: it writes a **whole eighteen-sector cylinder**
through the write head in the Macintosh's own layout, taken off the wire rather
than out of a book —

```text
  101 x $4e   gap          12 x $00  sync      3 x $a1  through Write Mark
    1 x $fe   the ID address mark, then C H R N, then one write of Write CRC
   22 x $4e   gap 2        12 x $00  sync      3 x $a1  marks again
    1 x $fb   the data address mark, 512 x $f6, one write of Write CRC
```

— and asserts that every field decodes with no bad CRC and every sector lands
in the image. It does: 194,400 cells, eighteen sectors, nothing bad. **The
write path can format a track.**

**A fourth defect was between that and the machine, and the diff is what found
it.** `Iwm::last_unabsorbed` keeps the cells of a flush that could make nothing
of them — the only evidence of what the head actually laid down, since the
cache is rebuilt from the image the moment the head moves — and
`a_blank_disk_in_the_second_drive` compares them with what this project's own
encoder makes of the same track. That diff answers in one run whether a
divergence is a shift, an inversion, a doubling or a different value.

It was none of them. The cells were **right**: 114 `$A1` marks at every
alignment, 9,217 `$F6`, 2,329 `$4E`, 5,254 `$00` syncs — a proper IBM track. The
decoder rejected all nineteen fields it found, every one `Bad::Id`, and every
stored CRC ended in **`$4E`**. The gap byte. The field was **one byte short**:
the decoder read the first CRC byte and then the gap behind it.

**A write to the ISM's CRC register is one register write and two bytes on the
medium**, and the processor paces itself off a handshake register that counts a
*two*-byte FIFO (page 25: "In write mode, it indicates that 2 bytes can be
written to the FIFO"). So a processor that has filled the FIFO exactly as it
was invited to, and then asks for the CRC, is asking for two bytes there is no
room for — and the second was dropped on every field of every sector. The two
halves of the generator are now staged **behind** the FIFO the processor
counts; `Writer::space` still reports two, which is what the handshake register
must say, and nothing the processor does can consume the chip's own pair.

With that, a Macintosh initializing a blank in its external drive goes from one
track and nothing absorbed to **178 cylinders flushed and 3,126 sectors taken**
— more than a whole disk's worth — with no underrun and the ERROR register
`$00` throughout.

**It still says "Initialization failed!"**, now at cylinder 40 rather than
after one track, and that is where this pass stops. Two things about it are
worth writing down rather than guessing at.

* The failure is now *late* — after the medium has been written — so it is a
  different question from the four above, most likely the verify pass or the
  volume the Macintosh writes after the format.
* **The blank is formatted as 1.44 MB MFM whatever drive it is in**, which a
  Macintosh Plus cannot read — and **putting a plain 800K mechanism on the
  external port does not change it**, which is a measured negative result
  rather than a guess. `mac.swim` takes `external = "dd"` for exactly that
  configuration, which a real Classic could have and which ought to settle the
  question without inventing a signal; the mechanism then answers the
  SuperDrive line with the cable's pull-up, the ROM reads it (`rHandshake R
  1a`, `SENSE` high — "not a SuperDrive"), and the System **still** drives the
  disk through the ISM in MFM. It never writes the Setup register **at all**
  after the boot — zero writes of register 5 in twenty thousand accesses — so
  bit 2 stays clear ("Setting the bit selects GCR mode; clearing it selects the
  normal operating mode", page 22) and so does bit 6 ("This bit must be set for
  GCR operation"), and it never drops to the IWM register set either: **zero**
  IWM accesses for drive 2 against 20,591 ISM ones.

  With an 800K cylinder under it — 76,140 cells for zone 0 — a 1.44 MB MFM
  track laps itself: the head laid **215,991** cells into it, nearly three
  revolutions, each overwriting the last, and nothing decodes.

  So *how a Macintosh is told to format 800K GCR* is the open question, and it
  is not the drive's own SuperDrive line. The real density line remains
  unestablished and this sidesteps rather than answers it.

## The ledger: what to build next, in the order it is likely to matter

1. ~~**The eject at 69 seconds.**~~ **Settled, and it was the mechanism.** The
   trace now prints `SEL`, the selected drive and the motor beside every phase
   write, and the two readings the ledger offered are no longer symmetric:

   ```text
     ISM wPhase  W f3  drv1 sel1 tach        motor1 disk1   CA0, CA1
     ISM wPhase  W f7  drv1 sel0 installed   motor1 disk1   and CA2, and SEL goes low
     ISM wPhase  W ff  drv1 sel0 installed   motor1 disk0   LSTRB rises: eject
     ISM wPhase  W f7  drv1 sel0 installed                  and falls
   ```

   **`SEL` is low**, so the reading that `SEL` is not the VIA's `PA5` on a
   Classic is refuted twice over: the pin is measurably low at the strobe, and
   it has to be `PA5` anyway or the head could not be chosen, because `RDDATA0`
   and `RDDATA1` differ only in that line and this machine reads both sides of
   its disk. The address really is the drive register file's eject with `CA2`
   as its one.

   What settles it is the register write *immediately before*: `wMode1 W 82`,
   which is `MotorOn` and `ENBL1` — the ROM turns the spindle on and then asks
   to eject, and nothing stops it first. A Sony mechanism will not throw a disk
   out from under a turning spindle. So `Mechanism::control` performs the eject
   only when the motor is stopped, and the guest agrees: the startup volume
   stays mounted, the Finder goes on reading *and writing* it for another
   virtual minute, and the desktop is the same picture it was.

   That the interlock exists is an **inference**; what is measured is that the
   guest does not accept the eject. `tests/mac_classic.rs` asserts the disk is
   still in the drive at the end of the boot.
2. ~~**Writing to a disk.**~~ **Built**, and Apple's own code is what proves
   it. See "The write path", below.
3. ~~**The pointer.**~~ **Built, and it lands where it is put.**
   `tests/mac_classic.rs::the_pointer_goes_where_it_is_put` boots to the Finder
   and drives the pointer to four places on the screen — (350, 158), (100, 40),
   (470, 300), (12, 300) — and `Mouse` at `$830`, the ROM's own low-memory
   global, holds **exactly** each of them.

   `src/host/input/mac.rs` grew `MacAdbSink`, the Apple Desktop Bus counterpart
   of the sink a Plus's quadrature mouse has. Getting a count from it to the
   guest took four measurements and each is written where it belongs.

   **The system does not poll the bus, and pulling the attention line does not
   make it.** Read through the VIA with a debugger once Mac OS 6.0.8 is up:

   ```text
     ORB  $7f    PB5:PB4 = 1:1, state 3 — idle
     DDRB $f7    PB3, the attention line, is the only input
     ACR  $0c    mode 011 — shift *in* under an external clock on CB1
     IER  $a7    and the shift-register interrupt is enabled
   ```

   and then it touches `ORB`, `SR` and `ACR` **not once** for the next virtual
   minute, whatever the attention line does — holding it low ten thousand times
   longer changes nothing. The computer is not watching a pin. It is sitting in
   shift-in mode waiting for a **byte**.

   **And the byte is the command of a reply, not a doorbell.** The instrument
   that settled it is the one `tests/mac_plus.rs` earned on the other machine:
   count the stages rather than the ends — reports in, polls, answers, bytes
   out — and the one that drops is the answer. Here none of them dropped. One
   report, one poll, the mouse answering it, **ten bytes out and every one of
   the ten the pull-up**; and the pointer crept one pixel up and left per
   report, which is `$FF $FF` read as (−1, −1). A five-microsecond trace of the
   transceiver's own state showed why: given a byte with nothing behind it the
   computer drives states 1 and 2 *five times over*, reading `$FF` each time,
   before giving up. It is not answering a doorbell with a transaction — it is
   collecting a reply it believes has already been polled. So the transceiver
   polls the device itself and hands over the command it used.

   **A reply is spent once it has been read**, or the computer's own follow-up
   Talk reads the same movement a second time and the pointer travels twice as
   far.

   **Movement accumulates until it is read.** A report that arrives while the
   link is busy used to be dropped: the pointer arrived twenty-seven pixels
   short and fourteen of a hundred and twenty reports had never been answered.
   The mouse now counts, and an announcement the link was too busy for is
   *owed* and goes out when it next falls idle. With both, 531 of 532 reports
   are answered.

   **The budget is shared between the axes**, because the ROM's threshold is on
   the two together: a diagonal of four and four is over it where four on one
   axis alone is not, and a pointer sent diagonally arrived at twice the
   distance and pinned in a corner.

   What is left over is Apple's own loss — the cursor task reads `MTemp`,
   scales it and writes it back at interrupt mask 0 — so the seam offers
   `MacAdbSink::resync`, which takes the guest's `Mouse` global as the truth.
   A Plus has no cheap way to ask and sweeps into a screen corner instead.

   Nothing wires `MacAdbSink` into the VNC or CLI front ends yet, so
   `rsemu run mac-classic` behaves exactly as it did.
4. **A host keymap.** `mac.adb` carries a keyboard at address 2
   and a mouse at address 3 and will report a key transition or a movement
   through Talk 0, and `mac.mouse` now drives the pointer — but nothing turns a
   host keysym into an ADB key code. The Plus's `mac.keyboard` cannot be reused
   for the codes: it speaks the *Guide*'s Table 7-6 transition codes, which are
   a different encoding from ADB's.
5. **The NCR 5380.** `src/dev/ncr5380.rs` exists and the Plus's `$580000`
   window is modelled; a Classic with an internal hard disk is the
   configuration most of them shipped in, and it is the way off a floppy.
6. **The sound**, which a Classic has in the same place a Plus does.

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
how to find where a ROM stopped. **Three of the eleven tests need nothing of
anybody's**: they assemble the board around rsemu's own ten-byte stub and around
a 1.44 MB image of numbered blocks built on the spot, and they are what `cargo
test` runs in CI.

### The instrument

`trace_the_controller` is `#[ignore]`d because it asserts nothing; it is the
tool the three measurements above came out of, and it has four knobs:

```sh
RSEMU_MAC_DISK_KIND=real|1440k|800k|none   # what is in the drive; 1440k is the default
RSEMU_MAC_SECONDS=120                      # how long to run; 12 by default
RSEMU_MAC_DRIVES=2                         # an external drive on the cable as well
RSEMU_MAC_UNFOLDED=1                       # every access, not runs collapsed with a count

RSEMU_MAC_ROM_DIR=… RSEMU_MAC_DISK_DIR=… RSEMU_MAC_DISK_KIND=real \
  cargo test --release --all-features --test mac_classic \
    trace_the_controller -- --ignored --nocapture
```

It prints the controller's whole conversation with each access **named** —
which register set answered, which ISM register, and which of the mechanism's
sixteen status lines the phase lines and `SEL` were addressing — then the
busiest memory addresses of the last virtual second, which is how to tell a
processor waiting on something from one that has decided to stop. Swapping
`RSEMU_MAC_DISK_KIND` between `800k` and `1440k` is the differential that found
the SuperDrive's status line: the two traces were **identical for 239
accesses** while the ROM refused to touch the drive, which is what said the
medium was not what it was deciding on.
