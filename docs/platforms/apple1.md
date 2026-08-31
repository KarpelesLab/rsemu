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

**What this does *not* cover.** Later adaptations are separate works. Ben
Eater's well-known port to a 65C51 ACIA
([gist](https://gist.github.com/beneater/8136c8b7f2fd95ccdd4562a498758217))
carries no licence, no copyright line and no attribution; whatever originality
his changes contain is his, unlicensed. So: **take the original listing from
the 1976 manual, not somebody's adaptation of it.** The same goes for any
modernised or relocated variant.

## rsemu's own monitor

`dev::apple1::RSMON` is 250 bytes of 6502 written for rsemu, MIT, committed
with its assembly listing, and used as the default ROM so the machine
demonstrates itself with nothing third-party at all. `--rom` takes any other
image.
