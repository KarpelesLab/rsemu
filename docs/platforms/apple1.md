# Apple 1

Consumed by: `machines/apple1.machine` and `src/dev/apple1/`, behind the
`machine-apple1` feature.

```console
$ rsemu run apple1
RSMON
>FF00
FF00: D8 A2 FF 9A A9 7F 8D 12
>
```

The smallest complete machine rsemu ships, and the first one a person can type
at. A 6502, 4 KiB of RAM, one MC6821, and 256 bytes of monitor ROM — no video
timing, no sound, no cartridge. Everything the machine does for you it does
through four registers.

## Why it is here

Not for its own sake. It is the smallest thing that forces the parts of the
framework a headless regression test never touches:

- a **character-stream seam** between a device model and the host
  (`src/host/chardev.rs`), which is what a 16550 UART needs on the RISC-V board
  and what a PS/2 controller needs on the PC;
- a **device whose clock is not the CPU's**, and whose rate a guest can observe
  by polling a status bit;
- a **run loop that is not `run_for(1s)`** — one that pumps a terminal, holds
  virtual time to real time, and hands the keyboard over.

And it produces a visible result on the day it lands, which `ROADMAP.md` §2 asks
of every phase.

## Primary sources

| Source | Covers |
| --- | --- |
| *MC6821 Peripheral Interface Adapter* data sheet (Motorola) | the register model: two ports, each with a data-direction register, an output register and a control register overlaid two per address |
| Applefritter, [*Apple I Replica Creation*, ch. 7](https://www.applefritter.com/replica/chapter7) | what each of the four addresses is on this board, and the display's `DA`/`RDA` handshake. Table 7.5 is the register map |
| Ken Shirriff, [*Inside the Apple-1's shift-register memory*](http://www.righto.com/2022/04/inside-apple-1s-shift-register-memory.html) | the terminal section: why the display is a delay line and not a frame buffer, and why it takes about one character per video field |
| The Woz Monitor's published register equates and polling loops | the software-visible contract — addresses and a status bit, which are facts about the hardware (`ROADMAP.md` §1, "facts versus expression"). No monitor *source* is used or reproduced |

## The register map

Verified against the data sheet and Applefritter table 7.5 before anything was
written. The Woz Monitor's `BIT $D012 / BMI` and `LDA $D011 / BPL` loops are the
same contract seen from the other side.

| Address | Name | Read | Write |
| --- | --- | --- | --- |
| `$D010` | `KBD` | the key, **bit 7 always set**; clears the flag in `$D011` | `DDRA` while `$D011` bit 2 is clear |
| `$D011` | `KBDCR` | control A; **bit 7 set while a key is waiting** (`IRQA1`, from the CA1 strobe) | control A; bits 6-7 are read-only |
| `$D012` | `DSP` | port B: bits 0-6 read back the last character, **bit 7 is `DA` — set while the display is busy** | `DDRB` while `$D013` bit 2 is clear, otherwise a character |
| `$D013` | `DSPCR` | control B | control B; bits 6-7 are read-only |

Four addresses, **six** registers. A 6821 overlays each port's data register on
its data-direction register, and bit 2 of the control register picks between
them. That is not pedantry: the first thing an Apple 1 monitor does is store
`$7F` to `$D012` while bit 2 of `$D013` is still clear, setting PB0-PB6 to
outputs and leaving PB7 an input. A model without the DDRs takes that `$7F` for
a character and prints one.

Two board details the data sheet cannot tell you:

- **PA7 is strapped to +5 V**, so a key always reads with bit 7 set — which is
  why monitors compare against `$8D` for Return.
- **`DA` is wired back to PB7**, so software can poll the display's busy line as
  an ordinary port bit.

### Decoding

`CS0` is A4 and the register selects are A0 and A1, so the PIA answers all over
the `$Dxxx` page. `machines/apple1.machine` maps the sixteen bytes at `$D010` —
the four registers repeated four times, which is what A0/A1-only decoding gives
— and leaves the rest of the page on the open bus, which reads the same either
way.

## The clock

The Apple 1 runs its 6502 at **1.022727… MHz**, from a 14.31818 MHz crystal
divided by 14. The crystal is exactly four times the NTSC colour burst, and the
colour burst is 315/88 MHz, so:

```text
  crystal  = 4 x 315/88 MHz = 315/22 MHz  = 14318181.81… Hz
  CPU      = crystal / 14   = 45/44 MHz   =  1022727.27… Hz
```

Neither is an integer number of hertz, which is why `machines/apple1.machine`
writes `osc master = 315000000/22 Hz` and `clock = master / 14` rather than a
rounded decimal (`ROADMAP.md` §4.2). The ratio itself is an integer, so the
relationship stays exact however long the machine runs.

The display is a **separate oscillator at 60 Hz**, and deliberately so. The
terminal section is timed off the same crystal, but the exact divisor is not
something these sources state; inventing a plausible one would nail a guess into
the timeline, where an honest second tree costs only the fixed-point cross-tree
conversion the design already has. `-p pace=false` releases every character on
the store instruction instead, which is what a test wants.

## The monitor ROM, and the licence question

**rsemu ships its own.** `RSMON` (`src/dev/apple1/monitor.rs`) is 250 bytes of
6502 plus the vectors: a hexadecimal examine/deposit monitor that echoes what
you type. It is ours, MIT, committed, and reproduced in full as a commented
listing beside the bytes — a test disassembles those bytes with the crate's own
6502 disassembler and checks that they say what the listing says.

**The Woz Monitor is not shipped and never will be.** Steve Wozniak's monitor
has been passed around freely for decades, which is not a licence, and its
copyright status is not clear. It is therefore under the same rule as `nestest`
and blargg's ROMs (`../testing/conformance-suites.md`): **fetch-only, never
vendored, never committed, never attached to a release.** Running it as an
emulated guest is ordinary use; redistributing it is not ours to do.

If you have a copy you may use:

```console
$ rsemu run apple1 --rom wozmon.bin
$ RSEMU_APPLE1_ROM=wozmon.bin cargo test --all-features woz -- --nocapture
```

`scripts/fetch-testdata.sh wozmon` will place one for you if you give it a
`--wozmon-url`; it has no default and will not pick a mirror on your behalf. The
test that needs it skips cleanly when the variable is unset, so `cargo test`
offline stays green.

## Using it

```console
$ rsemu run apple1                     # RSMON, at the machine's real speed
$ rsemu run apple1 -p pace=false       # ...without waiting for the display
$ rsemu run apple1 -p ram=8K           # the expansion the machine took
$ rsemu run apple1 --rom wozmon.bin    # somebody else's monitor
$ rsemu run apple1 --headless --for 5s # no terminal, just a state hash
```

The terminal goes into raw mode if the host has `stty` and stdin is a terminal
— pure `std`, no `libc` (`src/host/terminal.rs`) — and **Ctrl-C stops the
machine**. Without raw mode it falls back to line buffering, says so, and
everything still works a line at a time.

RSMON itself: four hex digits and Return dumps eight bytes; Return on its own
walks forward; `:` then two hex digits and Return deposits a byte and advances.
The Apple 1's keyboard was upper case only, so lower case is folded up on the
way in and backspace arrives as the machine's rub-out key.

## What is not modelled

- **The cassette interface** at `$C100` and **Apple BASIC** at `$E000`. Both are
  expansion cards rather than board features, and neither is needed to boot a
  monitor and type at it.
- **The 6821's interrupt outputs.** `IRQA`/`IRQB` are not connected to the 6502
  on this board, so the enable bits are stored and read back and do nothing
  else.
- **CA2/CB2 as pins.** `DA` is CB2 in hardware; here it is state the display
  half owns, and nothing on this board observes CB2 except the video section
  the device already is.
- **The screen.** There are 40 columns and 24 lines on a real one, and scrolling
  it is the terminal section's job. Here the characters go to your terminal,
  which has its own opinion about how wide it is.
