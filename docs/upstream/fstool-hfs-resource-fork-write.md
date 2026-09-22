# fstool: classic HFS can be created and read, but a resource fork cannot be written

**Status:** open upstream as of `fstool` 0.4.29. No workaround in rsemu — the
gap is what stops `machines/mac-plus.machine` booting an operating system.

This is a specification for a change to `fstool`, which is a *separate
repository*. It is written down here rather than committed there — rsemu's
working rule is that we do not push to sibling repos, so an upstream fix becomes
a written spec and somebody carries it across deliberately.

## Why rsemu cares

`mac-plus` runs Apple's ROM to the insert-disk icon, spins the drive, reads a
track and decodes sectors — Apple's own code verifies our GCR encoding
(`tests/mac_plus.rs::the_rom_reads_a_track_and_decodes_a_sector`). What it
cannot do is boot, because that needs Apple system software on an **800K**
image and a Macintosh Plus has an IWM and an 800K drive: 1.44 MB needs the SWIM
controller that arrived with the SE FDHD, and `mac.disk` refuses such an image
by name.

The obvious route is to build an 800K disk from a 1.44 MB one the user already
owns — their media, their machine, nothing redistributed. `fstool` gets
impressively close:

| | |
|---|---|
| create a classic HFS volume | **works** — `fstool create --type hfs --output d.img --size 819200` (the `--type` help text omits `hfs` from its list, which is itself a small bug) |
| read a classic HFS volume | **works** — `fstool ls`, `fstool info`, `fstool cat` |
| read a **resource fork** | **works** — `fstool cat --rsrc` |
| list typed resources | **works** — `fstool resources`, with `--extract` |
| **write** a resource fork | **missing** |
| set type/creator (Finder info) | **missing** |

## Why the missing half makes the whole thing useless

A classic Macintosh file keeps its content in the resource fork. Measured on
the user's own *System Startup* disk (System 6.0.8, HFS, 1,435 KiB used of
1,437 KiB):

```
/System Folder/System   data fork    860 bytes
/System Folder/Finder   data fork      0 bytes
```

Copying those two files in with `fstool add` produces an 860-byte `System` and
an empty `Finder`: the right names, none of the content. The ROM would find a
volume, read its boot blocks and hand over to nothing.

Type and creator matter as much. The boot blocks name the System and Finder
files, and the Mac identifies a system file by its type `ZSYS` — a file copied
in without Finder info is not a system file however complete its forks are.

## What is wanted

1. `fstool add --rsrc <IMAGE> <HOST_SRC> <FS_DEST>` — write the host file's
   bytes to the destination's **resource** fork, creating the file if needed,
   as the mirror of `cat --rsrc`. That alone is enough for a two-pass copy
   (data fork, then resource fork).
2. A way to set **Finder info** on an HFS file — four-byte type and creator at
   minimum (`--type ZSYS --creator MACS`), since without them the result is not
   bootable.
3. Optionally, an AppleDouble/AppleSingle-aware `add` so one command carries
   both forks and the Finder info, which is how such files travel between
   non-Mac hosts.

## How to tell it works

Round-trip on a real volume: copy `System` and `Finder` out of a 1.44 MB HFS
image with `cat` and `cat --rsrc`, into a fresh 800K HFS volume, set the types,
copy the source volume's first 1024 bytes (the boot blocks) verbatim, and boot
it on `mac-plus`. The machine should leave the insert-disk icon and run the
system. rsemu's side of that is already proven; only the writing half is
missing.
