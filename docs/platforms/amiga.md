# Commodore Amiga 500

Consumed by: `dev/amiga`, `machines/amiga-a500.machine`,
`tests/amiga_a500_board.rs`.

A 68000, 512 KiB of chip RAM, a Kickstart ROM, two 8520 CIAs and three custom
chips — Agnus, Denise and Paula. **What exists today is the memory map and the
decoders that make it work, and none of the chips.** This page is the ledger of
what the map decided, what it had to leave open, and what the chips will need
from it.

## Primary sources

| Source | Covers |
| --- | --- |
| [*Amiga Hardware Reference Manual*, 3rd edition](https://archive.org/details/amiga-hardware-reference-manual-3rd-edition) (Commodore-Amiga Inc.) | Appendix B, the custom-chip register summary in address order and its legend; Appendix D, the system memory maps; the 8520 appendix, the CIA addresses, port bits, chip selects and clocks |
| MC68000 User's Manual (Motorola) | The reset sequence, and the instruction encodings the test ROMs are hand-assembled from |

**Nothing else.** Every Amiga emulator the project is aware of is copyleft and
is listed under *Deliberately excluded* in [`../README.md`](../README.md), and
so is AROS's source: AROS is something rsemu may run, never something it reads.

## What the manual pins down

| | |
| --- | --- |
| Chip RAM | `$00_0000`–`$07_FFFF`, and `$08_0000`–`$0F_FFFF` on a 1 MiB machine |
| Custom chips | `$DF_F000`–`$DF_FFFF`; the register table fills offsets `$000`–`$1E4` |
| CIA-A | `$BFEr01`, register `r` = 0–F; "selected when A12 is low, A13 high" |
| CIA-B | `$BFDr00`; "selected when A12 is high, A13 low" |
| System ROM | `$FC_0000`–`$FF_FFFF` in the edition's own map — see below |
| `OVL` | CIA-A `PA0`, "memory overlay bit" |
| CIA timers | the 68000's E clock, one tenth of the processor clock: 709.379 kHz PAL, 715.909 kHz NTSC |
| CIA TOD | CIA-A: a 50/60 Hz event, "VSync or line tick"; CIA-B: horizontal sync |
| CIA interrupts | CIA-A "can generate interrupt INT2", CIA-B INT6 |

## What the board decided, and why

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
mapping its own scatter of windows. Twenty-one registers belong to more than
one chip (`DMACON` to all three, every sprite's `POS` and `CTL` to Agnus and
Denise), and one table is the only place that fact can live once.

**The CIA decode is the board's.** `mos.8520` publishes sixteen registers back
to back; `amiga.cia-decode` spreads them 256 bytes apart and puts them on one
byte lane. One decoder per chip, because the A12/A13 selects never pick both.

## Open, and not guessed at

| | Why it is open | What happens instead |
| --- | --- | --- |
| **`$1FE`** | Widely called the copper's `NO-OP`, but in neither Appendix A nor B of this edition, which ends at `DIWHIGH` (`$1E4`) | An unclaimed offset: a write is dropped and counted |
| **Byte access to a custom register** | Every entry is a word and the manual says nothing about a single data strobe | Refused by the region's access constraints |
| **Reading a write-only custom register** | The manual does not say | The last word driven onto the bus — **a placeholder** that Agnus's DMA cycles will replace; `CustomBus::unclaimed` counts every use |
| **`$DFF200`–`$DFFFFF`** | Appendix D gives a 4 KiB window, the table fills 512 bytes, and whether the rest mirrors is not stated | Only 512 bytes mapped; the rest faults. `mirror(custom)` is a one-word change |
| **A0–A7 in a CIA window** | The notation gives one hex digit of register select; nothing says whether the low byte is decoded | Not decoded — `$BFE003` is register 0 |
| **The ROM base** | The edition's map is `$FC_0000`, a 256 KiB Kickstart; 512 KiB images start at `$F8_0000` | `rom-base` and `kickstart-size` are parameters, defaulting to 512 KiB |

## What each chip will need

* **Agnus, Denise, Paula** — implement `CustomChip` on their register block,
  take a `custom = <object>` property, and call `CustomBus::attach` from `bind`.
  The copper writes through the same bus with `Origin::copper(danger)`.
  `src/dev/amiga/custom.rs` has the contract in full.
* **The 8520s** — `wire cia_a.pa0 -> gary.ovl`, and an `amiga.cia-decode` per
  chip. The CIA interrupts wait for Paula, which owns `INTENA`/`INTREQ`; the TOD
  inputs wait for Agnus's beam counters. The exact statements are in the
  comment block at the top of `machines/amiga-a500.machine`.
