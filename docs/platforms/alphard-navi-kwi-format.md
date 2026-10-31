# alphard-navi — the KWI container format

Byte-level reference for the `LOADING.KWI` files on the Alphard/Prado map SD
(`HD14/EXE/<model>/LOADING.KWI`, and the backup `HD10/BKPRG/EXE/LOADING.KWI`).
A KWI is the vendor's packaging of one model's full system — splash images,
parameter blocks, the ext2 root, the config descriptor, and the XIP kernel.
See [alphard-navi-kwi-boot.md](alphard-navi-kwi-boot.md) for how it is loaded.

Offsets below are from the HC59 image and were confirmed identical in layout on
HC60; all multi-byte container fields are **big-endian** unless noted (the PCRD
descriptor and the ext2 payload are little-endian).

## File shape

```
0x00000000  KWI header (32 bytes)
0x00000020  reserved region: manifest + splash BMPs + pmb + USRCONF
            + the PCRD boot descriptor + PCRD marker/digest
0x00158002  ext2 rootfs (raw, little-endian ext2), 255 MiB
            … followed by text config (smng process table, …) and
            the raw uncompressed XIP kernel …
EOF         (HC59: 0x1047c800 = 273,139,712 bytes)
```

Every member's file offset is its manifest address plus the manifest's end
(see below). The ext2 root always begins at file `0x158002`.

## KWI header (0x00..0x20)

```
0x00  u32 BE   0x00000100                              format/version
0x04  [12]     0f 56 a3 00 3c 3c 8a 00 07 00 01 66     fixed signature ("aisin-aw" v14)
0x10  u32 BE   0x00000100
0x14  u32 BE   0x00000001
0x18  char[4]  "HC59" / "HC60" / …                     model code (selects the unit)
0x1c  …        0x00000000
```

The 12-byte signature at `0x04` is what `kwifs.out` checks before mounting (it
carries the same bytes in its `.rodata`). The header is content-independent —
identical across models apart from the code at `0x18` — so there is **no
whole-image checksum in the container**; integrity is enforced later by the
PCRD `kernel_cksum` CRC at boot, not here.

A map/manifest-only KWI (`HD0/ALLDATA.KWI`, ~6 KiB) uses a different signature
(`0f 67 88 00 3c 47 22 00 03 00 07 22`) and is not a system image.

## The partition manifest

A packed table that lists the logical partitions of the assembled image. In the
HC59 file it sits at ~`0x4e000` (inside the reserved region). Each entry:

```
u32 BE  addr        logical address in the assembled image
u32 BE  size        byte length
u16 BE  namelen
u8[namelen] name    ASCII, no NUL
```

Entries are packed back-to-back and **2-byte aligned** (one pad byte if a name
ends on an odd offset). A 0x1e-byte header precedes entry 0 (at file
`0x4e010` in every image seen — all eleven models and the backup):

```
+0x00  00 00 00 1c ff ff ff ff 00 00 00 00 00 00 00 1c   fixed; how a reader finds it
+0x10  u32 BE  0x7a
+0x14  u32 BE  manifest length, header included (0x96)
+0x18  u32 BE  total assembled size (0x1042e420)
+0x1c  u16 BE  length of the entry table (0x78)
+0x1e  entries
```

**The logical image starts where the manifest ends** (`0x4e010 + 0x96 =
0x4e0a6`, where the first splash's `BM` sits), so a member's file offset is
simply `manifest end + addr`. That holds for every member, and it is how
`navi.kwiboot` finds them — no `MOD_HEAD` parsing needed:

| member | file offset (HC59) |
| --- | --- |
| `pmb` | `0x117e02` — this *is* the PCRD boot descriptor |
| `rootfs` | `0x118002` — the 256 KiB `PCRD` block, then the ext2 at `0x158002` |
| `USRCONF` | `0x10058002` |
| `xipImage` | `0x10064bb2` |

The six partitions (HC59; HC60 identical except `xipImage`/`rootfs` sizes):

| name | logical addr | size | meaning |
| --- | --- | --- | --- |
| `grp_dat_13cy_prgup.bmp` | `0x00000000` | `0x00064d24` | program-update splash |
| `loading.bmp` | `0x00064d24` | `0x00065038` | boot splash |
| `pmb` | `0x000c9d5c` | `0x00000200` | parameter block (512 B) |
| `rootfs` | `0x000c9f5c` | `0x0ff40000` | ext2 XIP root (255 MiB) |
| `USRCONF` | `0x10009f5c` | `0x0000cbb0` | user config (~51 KiB) |
| `xipImage` | `0x10016b0c` | `0x00417914` | raw XIP kernel (~4.1 MiB) |

Addresses are **contiguous** (each = previous addr + previous size) and the
last ends at `0x1042e420`, the total. This is the *logical* assembled image, a
separate address space from both the file layout and the DRAM load addresses.
Do not confuse it with either:

* file offsets — those are the logical address plus the manifest's end
  (`0x4e0a6`), as above;
* DRAM addresses — at load time `rootfs` goes to `0x68000000` and `xipImage` to
  `0x62f80000` per the PCRD descriptor, not to their logical addrs.

## The MOD_HEAD directory

`kwifs.out` resolves member → file offset through a directory it calls
`MOD_HEAD`, exposed as `module_header.bin`. Its member-name table (from
`kwifs.out` `.rodata`) maps logical keys to file names:

```
MOD_HEAD       module_header.bin
GRP1_FILE      grpdata1.bin
GRP2_FILE      grpdata2.bin
RU_FILE        H2O-R.bin
PMB1_FILE      pmb1.bin        PMB1_MIR_FILE  pmb1_mir.bin
PMB2_FILE      pmb2.bin        PMB2_MIR_FILE  pmb2_mir.bin
PMB3_FILE      pmb3.bin        PMB3_MIR_FILE  pmb3_mir.bin
PMB4_FILE      pmb4.bin        PMB4_MIR_FILE  pmb4_mir.bin
```

(plus the `rootfs`/`xipImage`/`USRCONF`/`pmb` names the PWI uses). The `_mir`
entries are mirror copies of the parameter blocks. The exact on-disk layout of
`MOD_HEAD` (how it stores each member's file offset and length) is only
partially decoded; a loader that needs an arbitrary member should parse it, but
a boot loader does not — see next.

## What a boot loader actually needs

To stage a boot (per the boot doc) only two members matter, and both are
reachable without the directory:

* **`rootfs`** — raw little-endian **ext2**, always at file **`0x158002`**
  (superblock magic `0x53ef` at `0x158002 + 0x438`). The real filesystem is
  `blocks_count × block_size` = `65280 × 4096` = `0x0ff00000` bytes, i.e.
  `0x158002 .. 0x10058002`. Load it at DRAM `0x68000000`.
* **`xipImage`** — the raw, uncompressed ARM kernel (banner `Linux version
  2.6.35.14+` at ~`0x10346d46` in HC59). It is the trailing member; its length
  is the manifest/PCRD `kernel_size` (`0x00417914` HC59 / `0x004178e0` HC60).
  Between the ext2 end (`0x10058002`) and the kernel is a text config block
  (`smng` process table etc.), so the kernel does **not** start exactly at the
  ext2 end — resolve its file offset via `MOD_HEAD` or by locating the kernel
  head. Load it at DRAM `0x62f80000`.

The remaining boot inputs — kernel load/entry, sizes, CRC, and the command
line — are all in the **PCRD boot descriptor** (file `0x117e02`, little-endian),
documented in the boot doc. Its last 20 bytes are USRCONF's placement and the
version tag:

```
+0x1ec  u32 LE  USRCONF load address (0x6010c000)
+0x1f0  u32 LE  USRCONF length (= the manifest's)
+0x1f4  u32 LE  USRCONF's 32-bit additive sum of little-endian words
+0x1f8  char[8] the version tag (VC5910KA), which the PCRD block repeats
```

The kernel's CRC is CRC-32 as zlib computes it; checked against all eleven
models.

## Cross-model check

HC59 vs HC60, from the two images:

| field | HC59 | HC60 |
| --- | --- | --- |
| header sig / layout | identical | identical |
| model code @0x18 | `HC59` | `HC60` |
| manifest format + addrs | identical | identical |
| `xipImage` size | `0x00417914` | `0x004178e0` |
| PCRD magic/load/rootfs addrs/cmdline | identical | identical |
| PCRD `kernel_cksum` | `0x19a823f5` | `0x58795fcf` |
| version tag @ PCRD marker | `VC5910KA` | `VC6010KA` |
| PCRD digest high 16 bytes | constant | constant (same) |

Conclusion: the container is a stable, model-generic format. A reader written
against these offsets handles all 11 models; only payload bytes, sizes, and the
per-model CRC/digest differ.
