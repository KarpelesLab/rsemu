# Macintosh Plus (1986)

Consumed by: `dev/mac`, `machines/mac-plus.machine`, `host/display/mac.rs`.

A 68000 at 7.8336 MHz, one to four megabytes of memory, 128 KiB of ROM, a 6522
VIA, a Z8530 SCC, an IWM and its 800K drive, an NCR 5380 for SCSI, and a
512 × 342 one-bit screen that is **read out of main memory** by a counter
rather than owned by a video chip. rsemu's first Apple machine, and the first
board here whose framebuffer is somebody else's RAM.

## Primary sources

| Source | Covers |
| --- | --- |
| *Guide to the Macintosh Family Hardware*, 2nd edition (Apple Computer, Addison-Wesley 1990) | The whole machine: chapter 3 for the address map and the overlay, the VIA chapter's port-assignment tables, the video raster, the disk interface and the drive's register file |
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
  **83 % zero**; what is not zero is a sparse table of 58-byte records holding
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
  $00 0000 - $3F FFFF   memory, or the ROM while the overlay is up
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

### Main memory does not repeat — and that is load-bearing

**The one thing in this map that had to be found rather than read.** The ROM
sizes memory by writing at the top of each candidate size and reading it back;
a decoder that folded the address so that a 1 MiB machine answered at
`$100000` made *every* machine look like a 4 MiB one. The ROM then put its
screen buffer at `$3FA700`, where there is no memory, and looped in its memory
test for ever — a write pass of 1.7 virtual seconds followed by a read pass of
1.1, over and over, with nothing else happening on the board.

With memory answering only where it is and the rest of its window floating, the
same ROM writes `MemTop = $00100000` and `ScrnBase = $000FA700` on the stock
board, and `$00400000` / `$003FA700` with `-p ram=4M`. Both are asserted in
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
| `mac.glue` | the overlay at zero, the `$600000` window, and memory that answers only where it is | nothing else; it has no registers |
| `mac.via` | a whole 6522: both ports pin by pin, both timers with the one-shot, free-run and PB7 modes, `ACR`/`PCR`, and the interrupt flag/enable pair with its read-to-clear and its SET/CLEAR write | **the shift register does not shift.** `SR` is stored and read back, and `IFR` bit 2 never sets — which is the keyboard path |
| `mac.video` | 512 × 342 one-bit pixels read out of main memory at capture time, the screen buffer hanging below the top of memory with `PAGE2` picking which of the two, and the vertical and horizontal blanking outputs | the cycles it steals from the processor: this board's 68000 runs at its full rate |
| `mac.scc` | the register pointer and all thirty-two registers, `RR0`-`RR3`, the reset commands, and the two carrier detects | any serial traffic, the baud-rate generator, the DPLL, `/WREQ` |
| `mac.iwm` | the sixteen soft switches, the mode and status registers, the write handshake, and the drive's own sixteen status lines and four controls | **the data path.** `RDDATA` is a line that never moves, so the drive spins and steps and never delivers a sector |

Not modelled at all: the **clock chip** (the real-time clock and PRAM on the
VIA's `PB0`-`PB2`, and the one-second interrupt on `CA2`), the **keyboard**,
the **mouse**, the **sound** (the PWM buffer the VIA's `PB7` gates), and
**SCSI** (the NCR 5380 at `$580000`).

## How far a real ROM gets

`tests/mac_plus.rs`, on the stock 1 MiB board with the user's own ROM:

| Virtual time | What happens |
| --- | --- |
| 0 – 25 ms | the VIA is set up: `DDRA = $7F`, `DDRB = $87`, `PCR = 0`, `IER = $82` — vertical blanking enabled — and `PA4` is driven low, so the overlay goes and memory is at zero |
| 25 – 725 ms | the sound buffer is filled a byte per word and `/SNDENB` is asserted: **the startup chime**, 43 frames long, timed by polling `IFR` for the blanking flag some 6,600 times per 25 ms |
| 0.7 – 6 s | the memory test: alternating write and read passes over the whole megabyte, several patterns deep |
| ~6 s | the ROM finds 1 MiB, writes `MemTop`, `BufPtr` and `ScrnBase`, initialises the SCC (32 register writes), exercises the IWM (all sixteen switches, including a mode-register load), and talks to the clock chip |
| 7 s onward | **steady state**: the picture stops changing and the machine idles, with the 60.15 Hz tick chain running — `Ticks` at `$16A` counting up, `IFR` cleared 60 times a second — the keyboard retried two or three times a second on the VIA's shift register, and the drive polled once every half second |

**What the picture shows at that point**: the Macintosh's **50 % grey desktop**,
exactly 87,585 black pixels of 175,104, with the **arrow cursor drawn in the
top left corner**. Nothing else is on it, and it does not change again in thirty
virtual seconds. Every row is plain grey except the top thirty and the bottom
five.

No access faults, the processor never double-faults, and the video circuit
produces 60 frames a virtual second throughout.

### What it is not

It is **not the insert-disk screen**. The floppy-with-a-question-mark, and the
happy Macintosh before it, are not drawn. The machine is idle rather than
stuck in a loop of its own — the main thread is parked in a two-byte loop at
`$4006E8` and everything that happens, happens in the blanking interrupt — so
the ROM is *waiting* for something this board does not give it.

## The ledger: what to build next, in the order it is likely to matter

1. **The VIA's shift register, and a keyboard.** The only thing the idle
   machine does repeatedly that is not the tick chain is a keyboard
   transaction: six writes to `SR` and eight to `ACR` a second, for ever. A
   Macintosh Plus keyboard is a synchronous serial device that supplies its own
   clock on `CB1` and its data on `CB2`, and the VIA shifts against it. The
   model here does not shift at all, so `IFR` bit 2 never sets and every
   transaction times out. This is the most likely thing the ROM is waiting for
   and the cheapest to test.
2. **The clock chip.** The real-time clock and its twenty bytes of PRAM hang
   off `PB0`-`PB2` as a bit-banged three-wire interface, and its one-second
   output is the VIA's `CA2`. The ROM does talk to it during startup — 1,828
   VIA writes in one 25 ms window, which is the bit-banging — and gets nothing
   back. Driving `CA2` at 1 Hz from a test changed nothing, so the one-second
   interrupt alone is not the blocker; PRAM's contents may still be.
3. **The IWM's data path**, which is what makes a disk readable at all: the
   400K/800K format is Apple's 6-and-2 GCR, 80 tracks in five speed zones of
   12, 11, 10, 9 and 8 sectors, 524 bytes a sector (512 of data and 12 of tag),
   with an address field of `D5 AA 96` and a data field of `D5 AA AD`. The
   drive's registers and the head position are already here; what is missing is
   the bit stream and the encoder.
4. **The NCR 5380**, so the ROM's SCSI probe finds a bus rather than a floating
   one. `src/dev/scsi` already has the bus, the `Target` trait and a disk.
5. **Sound**, which is 370 bytes a frame out of the same buffer the disk-speed
   byte lives in, and the mouse, which arrives as two quadrature phases on the
   SCC's carrier detects and two more on the VIA's `PB4`/`PB5`.

## How the ambiguities were settled

Black-box tracing, which is the tool `CLAUDE.md` names and the only one
available: which addresses the ROM touches, in what order, what it writes, what
it reads back, what it waits on. Three things came out of it that no document
stated plainly.

* **Memory must not repeat** (above). Found by reading back `MemTop` — a
  data structure the ROM builds in memory, which is data rather than code — and
  seeing `$00400000` on a machine with a megabyte in it.
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

## Booting the user's disk images, and why this machine cannot

The two images in question are **DiskCopy 4.2** containers: an 84-byte header
(a 64-byte Pascal name — `System Startup` — then `dataSize`, `tagSize`, two
checksums, `diskFormat`, `formatByte` and the magic `$0100`) followed by the
data fork. Both read `dataSize = $00168000` — **1,474,560 bytes** — with
`diskFormat = 3`, which is 1.44 MB, and `formatByte = $22`. The file length,
1,474,644, is exactly the header plus the data, so nothing is being
mis-identified.

**A Macintosh Plus cannot read them, and that is the hardware.** A Plus has an
IWM and an 800K double-density drive. 1.44 MB needs the **SWIM** controller and
high-density media, both of which arrived with the Macintosh SE FDHD in 1989 —
three years after this machine. No amount of work on this board makes a Plus
read a 1.44 MB disk; a board that did would not be a Plus.

There are three real options, and they are not equally good.

1. **Find or make an 800K image of the same system, and finish the IWM.**
   System 6.0.8 shipped on 800K disks and those images exist; a 1.44 MB image
   can also be re-laid-out onto 800K media if what is on it fits, which for a
   *System Startup* disk it does. This is the option that finishes the machine
   the user asked for, and the work is item 3 in the ledger above — the 6-and-2
   GCR encoder and the drive's bit stream — plus items 1 and 2 to get the ROM
   as far as looking for a disk at all. **This is the recommendation.**
2. **Build a Macintosh SE FDHD or a Classic** and boot the images there. The
   user has `Classic.ROM` (512 KiB) in the same directory, and a Macintosh
   Classic is a 68000 machine with the same video, the same VIA and a SWIM —
   so most of `dev/mac` is reused and the new work is the SWIM and the ASC.
   The SWIM's IWM-compatible mode is the same sixteen switches this board
   already has; its MFM mode is new. It is more work than option 1 and it
   ends with a different machine than the one that was asked for.
3. **Neither**: go on with the Plus and read nothing. The insert-disk screen is
   a real milestone and it needs no disk at all.

The other ROMs in that directory — `LC.ROM`, `LC-II.ROM`, `Mac-IIcx.ROM`,
`Color-Classic.ROM` and the rest — are all 68020/68030 machines with slots,
different video and a different glue chip. They are a *third* board, not a
variation on this one.

## Running it

```sh
rsemu run mac-plus --media macrom=Mac-Plus.ROM
rsemu run mac-plus -p ram=4M --media macrom=Mac-Plus.ROM
rsemu run mac-plus --media macrom=Mac-Plus.ROM --vnc :5900
```

The ROM image must be exactly 128 KiB; trim a longer file first (see above).

The tests read the user's own ROM in place and skip, printing why, when it is
not there:

```sh
RSEMU_MAC_ROM_DIR=~/retro/macintosh_plus/Macintosh-ROMs \
RSEMU_MAC_FRAME_DIR=/tmp/frames \
  cargo test --features machine-mac-plus,display-png --test mac_plus -- --nocapture
```

`RSEMU_MAC_TRACE=1` prints the processor's state once a virtual second, which
is how to find where a ROM stopped. `tests/mac_plus_board.rs` needs nothing of
anybody's: it assembles the board around rsemu's own ten-byte stub and checks
that every chip answers where the Guide puts it.
