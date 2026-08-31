# Apple 1 (1976)

Consumed by: `dev/apple1`, `machines/apple1.machine`.

A 6502, 4 KiB of RAM, an MC6821 PIA driving a keyboard and a character
display, and 256 bytes of monitor ROM. rsemu's smallest interactive machine,
and the one that forced the `host::chardev` seam a UART will later reuse.

## Primary sources

| Source | Covers |
| --- | --- |
| [Apple-1 Operation Manual (1976)](https://archive.org/details/apple1manual) | The whole machine: specifications, memory map, the PIA addresses, and **the complete Woz Monitor source listing** |
| MC6821 data sheet (Motorola) | The PIA's register model — four addresses, six registers |
| *Apple I Replica Creation*, ch. 7 tbl. 7.5 | What the four addresses are on this board, and the `DA`/`RDA` handshake |
| [Ken Shirriff, Apple 1 shift-register memory](https://www.righto.com/) | Why the display runs at ~60 characters/second |

## What the manual pins down

Now that the manual is usable as a primary source, these come from it directly
rather than from inference:

| | |
| --- | --- |
| CPU | MOS Technology 6502, **1.023 MHz** clock, **0.960 MHz** effective "including refresh waits" |
| Video | 40 characters/line, 24 lines, automatic scrolling; 15734 Hz line rate, **60.05 Hz** frame rate; 5×7 character matrix |
| Display memory | dynamic shift registers, 1K × 7 |
| RAM | 4 KiB supplied, 8 KiB on-board capacity (4096/2104 dynamic) |

And the PIA, quoted from its "PIA Internal Registers" table:

- `$D010` **KBD** — "High order bit equals 1."
- `$D011` **KBD CR** — "High order bit indicates 'key ready'. Reading key
  clears flag. Rising edge of KBD sets flag."
- `$D012` **DSP** — "Lower seven bits are data output, high order bit is
  'display ready' input."
- `$D013` **DSP CR**

**One polarity worth pinning down, because the prose and the code appear to
disagree.** The manual's prose reads as though 1 means ready. Its own object
code says otherwise, and the object code wins: `ECHO` at `$FFEF` is

```
FFEF:  2C 12 D0     BIT $D012
FFF2:  30 FB        BMI $FFEF     ; $30 is BMI, not $10 BPL
FFF4:  8D 12 D0     STA $D012
```

`BMI` loops **while bit 7 is set**, so **set means busy** and the routine
proceeds when it clears. rsemu implements that, and the disagreement is
recorded here rather than left for the next reader to rediscover — the scanned
prose at this point is OCR-damaged (a character is missing from the
parenthetical), which is very likely the whole explanation.

## The clock

The manual states it directly: a 6502 at **1.023 MHz**, with an *effective*
cycle frequency of **0.960 MHz** "including refresh waits". The first number is
the crystal chain — 4 × NTSC colour burst = 315/22 MHz, ÷14 = **45/44 MHz** —
and the second is what the CPU actually achieves once DRAM refresh steals
cycles. rsemu models the first; the refresh stall is not modelled, and that is
a real ~6 % difference worth knowing before anyone benchmarks against the
manual's figure.

## The Woz Monitor: public domain, and why

**Determination: the 1976 listing is in the public domain, and rsemu may ship
it.** The reasoning, recorded here because `ROADMAP.md` §1 requires provenance
to be auditable years later by someone who was not in the room:

1. The complete monitor source was **published** in the Apple-1 Operation
   Manual, a 1976 Apple Computer Company publication (the Computer History
   Museum catalogues it as such).
2. US works published before 1 January 1978 fall under the **1909 Copyright
   Act**, whose notice requirement was strict: publication without the required
   notice injected the work into the public domain immediately. The 1976 Act's
   cure provisions for omitted notice did not exist yet and do not apply
   retroactively.
3. **The manual carries no copyright notice.** Verified against the scan at
   `archive.org/details/apple1manual`: searching its full text for
   `copyright`, `(c)`, `©` and `all rights reserved` returns nothing. Its
   masthead reads "APPLE COMPUTER COMPANY, 770 Welch Road, Palo Alto, Calif.
   94304" — the pre-Cupertino address, confirming the 1976 printing — and the
   only rights-adjacent text in it is a hardware warranty.
4. Others reproducing the manual have reached the same conclusion.

**Caveats, stated rather than buried.** This rests on OCR of one scan: a notice
rendered as a small glyph, or printed on a cover not included in that scan,
would not appear in the text search. And this is a reading of copyright law by
engineers, not advice from a lawyer. It is recorded as the project's
determination, with its evidence, so that it can be re-examined rather than
inherited as folklore.

**Later adaptations are separate works, and their own licence governs.** Ben
Eater's well-known port to a 65C51 ACIA is the case in point. The
[gist](https://gist.github.com/beneater/8136c8b7f2fd95ccdd4562a498758217)
itself carries no licence header. The licence is stated on the page that
publishes it: <https://eater.net/6502> says **all code used in his videos is
released under a Creative Commons Attribution (CC-BY) licence**, and lists
`wozmon.s` and a prebuilt `wozmon.bin` among the files it covers. (The site is
client-rendered, so read it in a browser; `curl` returns only the app shell.)

CC-BY is permissive and redistributable alongside MIT **provided attribution
travels with it**: credit Ben Eater, name the licence, and say the file was
modified if it was. Creative Commons themselves advise against CC licences for
software, but that is advice about fitness, not a defect in the grant — it is a
valid permission and we may rely on it.

So both layers are usable: the 1976 listing is public domain, and Eater's
modifications are CC-BY. Attribution is required for the latter and costs
nothing.

## rsemu's own monitor

`dev::apple1::RSMON` is 250 bytes of 6502 written for rsemu, MIT, committed
with its assembly listing, and used as the default ROM so the machine
demonstrates itself with nothing third-party at all. `--rom` takes any other
image.
