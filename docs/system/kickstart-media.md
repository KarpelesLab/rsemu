# Amiga Kickstart ROMs, from the media the user already owns

Implemented by: `src/host/media/kickstart.rs`, feature `media-kickstart`.
Reached by: `--media <slot>=kickstart:<source>`.

A Kickstart ROM is not redistributable. rsemu therefore ships none, fetches
none, and keeps none: it **points at the user's own legally purchased media and
reads it in place**, on every run. Nothing is copied out of an Amiga Forever
installation, nothing is cached, and no `rom.key` ever touches this repository.

## What a user types

```
rsemu run <machine> --media firmware=kickstart:<file>
rsemu run <machine> --media firmware=kickstart:<file>,key=<file>
rsemu run <machine> --media firmware=kickstart:<dvd.iso>,rom=<name>
```

| Form | What it reads |
| --- | --- |
| `kickstart:kick31.rom` | A plain or `AMIROMTYPE1`-keyed ROM. A keyed one is decoded with the `rom.key` **beside it**, which is where Amiga Forever puts it |
| `kickstart:kick31.rom,key=/elsewhere/rom.key` | The same, with the key somewhere else |
| `kickstart:amiga-forever-dvd.iso,rom=amiga-os-310-a1200` | Straight out of the DVD image, key included, nothing extracted by hand. The `.rom` suffix is optional |
| `kickstart:amiga-forever-dvd.iso` | Fails, and lists the ROMs the disc holds |

The scheme list is closed: a specification that is not `kickstart:…` is a plain
file path, so `C:\roms\fw.bin` is a file and not a source called `C`.

Every run prints one line naming what it decoded, because nothing else can:

```
rsemu: firmware: Kickstart 40.68, 512 KiB, keyed, checksum verified
```

An ADF disk comes off the same disc the same way, as
`--media df0=adf:<dvd.iso>,disk=<name>`; `docs/platforms/amiga.md` (*Disks*)
has that source, and why the disk it produces is a copy.

## Media decode, not a loader device

`dfu.loader` (`src/dev/dfuse.rs`) is the precedent that decides this, and it
decides it the *other* way — which is why it is worth writing down.

A DfuSe file is a list of pieces each carrying its own absolute address. A media
slot is `{name, bytes}` and no address travels with it, so the thing that
understands DfuSe has to be a device that writes into an `AddressSpace`.

A Kickstart carries **no addresses at all**. Once the wrapper is off it is one
flat 256 KiB or 512 KiB image, and everything about where it goes belongs to the
board:

* it sits at `$F8_0000` (512 KiB parts) or `$FC_0000` (256 KiB) — a decode in
  Gary, not a field in the file;
* it is *also* visible at address 0 out of reset, because `OVL` in CIA-A's port A
  comes up set and the decoder mirrors ROM over Chip RAM until Kickstart clears
  it. The same bytes are at two addresses at once, which no file format could
  describe. That is a `map` and a wire.

A loader device would buy nothing and cost a realize, a snapshot and a reset —
and it would put the overlay's policy inside a *format parser*, where it cannot
see the CIA. So the container decodes on the host side, the plain bytes go into
a media slot, and an Amiga board maps that slot twice and wires `OVL` to the
second mapping.

The second reason is smaller and just as decisive: `rom.key` is a second file,
supplied by the user, that is not guest-visible in any way. A device would need
a media slot for it, making `rom.key` part of the machine description of every
Amiga. It is not part of the machine.

## The ROM header and footer

Big-endian throughout. From the *Amiga Hardware Reference Manual*
(Commodore-Amiga) for the address map, the `OVL` overlay and the
interrupt-acknowledge read; confirmed field by field against 42 images.

```
  header
    0   u16  ROM identification word, high byte $11
    2   u16  $4EF9, a 68000 JMP.L
    4   u32  its operand: the absolute entry point
   12   u16  version    $FFFF on 1.0 and 1.1, which predate the convention
   14   u16  revision

  footer, the last 24 bytes
  -24   u32  checksum
  -20   u32  the size of the image, in bytes
  -16   8xu16  $0018..$001F, the vector numbers read back during an
               interrupt-acknowledge cycle ($19..$1F are the 68000's
               autovectors for levels 1 to 7)
```

**The identification word does not encode the size.** `$1111` on 256 KiB and
`$1114` on 512 KiB is the common case and looks like an encoding, but two images
in Cloanto's own set say otherwise: Kickstart 36.16 for the A3000 is 512 KiB with
`$1111`, and the A570 extended ROM is 256 KiB with `$1114`. Only the high byte is
checked; the size comes from the footer and from the file.

### The checksum

A 32-bit **one's-complement sum with end-around carry** over every big-endian
longword, the stored checksum included. A good image sums to `$FFFF_FFFF`. Same
arithmetic as an IP header checksum (RFC 1071) widened to 32 bits, which is what
makes it two 68000 instructions: `ADD.L` then `ADDX.L #0`.

**A checksum that is present and wrong is an error, never a warning.** A board
that boots a half-downloaded Kickstart does not fail; it misbehaves somewhere
else entirely.

### When there is no checksum

Not everything with a `.rom` extension is a Kickstart. An A1000 bootstrap ROM, a
CDTV extended ROM, an A590 controller ROM and a Picasso IV board ROM carry
`$FFFF_FFFF` or a stray value where the size longword belongs, and no checksum
anywhere. The discriminator that holds across all 42 images is **the size
longword at `-20` equals the length of the image**: every image where that holds
verifies, and every image where it does not carries no checksum at all.

Of the 42 files Amiga Forever 11 ships, 30 verify, 10 are refused as not
Kickstarts at all, and one is a zero-length placeholder.

## `AMIROMTYPE1`

Eleven bytes of ASCII `AMIROMTYPE1`, then the ROM XORed byte for byte with
`rom.key`, the key repeating from the start whenever it runs out. It is an
obfuscation and a licence check, not cryptography, and it is its own inverse.
Cloanto's own published description of the format; confirmed by decoding a real
keyed ROM with a real key and checking the result against the checksum above.

A real key is 1426 bytes and a Kickstart is 256 or 512 KiB, so the key does not
divide the image — the wrap-around is exercised on every real decode, and the
tests use a synthetic key of the same awkward length so CI exercises it too.

A wrong key is caught twice over: usually by the identification word, and
otherwise by the checksum. Neither failure can reach a board.

## Reading from the disc image

`fstool`'s `iso9660` module does the filesystem. Amiga Forever keeps everything
in one place, on the disc and in an installed copy alike:

```
/Amiga Files/Shared/rom/<name>.rom
/Amiga Files/Shared/rom/rom.key
```

so `rom=<name>` resolves there, and the key rides on the same disc — a user with
the ISO has the key, so `,key=` is an override rather than a requirement. A name
containing `/` is taken as a whole path into the volume instead.

Whether a source is a disc at all is settled by a two-syscall probe for `CD001`
at byte 32769 (ECMA-119 §6.7.1, §8.1.2: the volume descriptor set begins at
logical sector 16, and the standard identifier is at offset 1 of a descriptor).
A 1.7 GB ISO must not be read into memory to find one 512 KiB file inside it, and
a 512 KiB `.rom` must not be handed to a filesystem reader.

**The ROM directory is the authority on whether this is the right disc; the
volume id is the diagnostic.** Checking the id alone would reject a future disc
that reorganised nothing; checking neither would report "no such file" for a
Debian image. So a disc with no `/Amiga Files/Shared/rom` is refused by name:

```
rsemu: other.iso: is an ISO 9660 volume named `Debian`, but it has no
`/Amiga Files/Shared/rom` directory, so it is not an Amiga Forever disc.
```

## Testing without the user's files

`cargo test` needs no proprietary byte. The tests **build** ROM images —
a real header, a real footer, a checksum solved for rather than assumed — and
XOR them against a synthetic key, so CI covers the checksum, the wrapper, the
key wrap-around, every rejection path and the specification parser with nothing
licensed anywhere near it. `scripts/fetch-testdata.sh` states the rule for
corpora that cannot be redistributed; it applies here with no free URL to soften
it, so there is not even a fetch script.

Two tests want real files, read them **in place**, and skip with a printed line
when the variable is unset — never silently:

```
RSEMU_AMIGA_ROM_DIR=~/"Amiga Files/Shared/rom"
RSEMU_AMIGA_FOREVER_ISO=~/amiga-forever-dvd.iso
```

## Provenance

Written from the *Amiga Hardware Reference Manual* (Commodore-Amiga), Cloanto's
published description of the `AMIROMTYPE1` format, and ECMA-119 for ISO 9660.

**No Amiga emulator source was opened.** WinUAE, FS-UAE, UAE, vAmiga and Hatari
are GPL. AROS is under the MPL-derived AROS Public License — weak copyleft, and
therefore a **binary we run, never a source we read**; the AROS ROM in Amiga
Forever is a perfectly good black-box test fixture and that is all it is here.
