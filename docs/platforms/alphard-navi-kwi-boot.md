# alphard-navi — the KWI boot and load path

How the Alphard/Prado navigation computer goes from power-on to the running
product OS, and specifically how `osloader` stages a `LOADING.KWI` into memory
and hands it to U-Boot for the final XIP boot. This is the companion to
[alphard-navi.md](alphard-navi.md), which covers the board and the rsemu
machine model; here the focus is the **load-into-memory recipe** so the loader
can be driven (or reproduced) without caring which car it is.

Everything below was reverse-engineered from:

* the HC59 NOR dump (`S29JL064J`, 8 MiB) — U-Boot, the PWI config region, the
  recovery uImage, and the recovery CramFS root with the resident `osloader`,
  `kwifs.out`, `pmng`, `smng`;
* the HC59 and HC60 `HD14/EXE/<model>/LOADING.KWI` map-SD images.

**It is model-generic.** Every structure here is byte-identical in layout
between HC59 and HC60 — only the per-model payload (sizes, checksums, the
`%ID%` string) differs. The 11 models (`HC02 HC11 HC59 HC60 HD70 HD72 HE38
HF06 HF93 HH40 HJ66`) are the same system built for different head units, and
the resident NOR half selects among them by one variable. So one loader boots
them all; HC60 needs no HC60-specific NOR.

## The chain in one line

```
NOR U-Boot (EXCIOS M0017) → bootm 40000 → recovery kernel (uImage @ NOR 0x40000)
  → /sbin/init = /vns/bin/pmng → osloader
  → osloader mounts the map media, picks HD14/EXE/<ID>/LOADING.KWI,
    mounts it with kwifs.out (FUSE at /kwi), reads the members, and
    stages kernel + rootfs + USRCONF into DRAM via /dev/mem, builds the
    OSMEM shared area, writes the PCRD boot descriptor, sets a flag in SRAM,
    and warm-reboots
  → U-Boot (warm path) reads the PCRD descriptor, CRC-checks the staged kernel,
    and XIP-boots it at 0x62f80000 — or, if the check fails, boots the recovery
    kernel again
  → full product system (XIP kernel, ext2 root in RAM) → smng → exesvr
```

One kernel switch happens, and it is a **warm reboot**, not a kexec: osloader
leaves the staged image in DRAM (which survives the reset) plus a flag in
on-chip SRAM, and U-Boot's warm path boots what is already in memory.

## NOR layout (the resident half)

8 MiB Spansion S29JL064J at address 0, AMD/CFI command set. The map SD never
carries any of this; it is factory-resident and only a physical dump obtains
it.

| offset | size | contents |
| --- | --- | --- |
| `0x000000` | 256 KiB | U-Boot 2011.03-svn, Aisin fork **EXCIOS M0017** (banner `CPU : R-CarM1A`, `BOARD: M1DEBUG`). First word is `ea000013` (ARM reset branch). |
| `0x020018` | ~51 KiB | **PWI config region** — a tab/newline text blob: the `smng` process table followed by the `OSLOADER_*` and platform keys (see below). |
| `0x040000` | 0x2f1380 | **Recovery uImage** — a real U-Boot legacy image: magic `0x27051956`, `load=ep=0x80008000`, os=Linux, arch=ARM, type=kernel, **uncompressed**, name `Linux-2.6.35.14+`. |
| `0x340000` | ~3 MiB | **Recovery root** — CramFS (magic `0x28cd3d45`), 359 files. `/sbin/init → /vns/bin/pmng`; carries `osloader`, `kwifs.out`, `smng`, `biflib.so`. |

U-Boot's compiled-in boot command is the recovery path:

```
bootcmd = printenv; setenv bootargs ${console} root=/dev/null \
          rootflags=physaddr=0x340000 mem=256M ro; bootm 40000
```

So cold boot always brings up the **recovery** kernel (XIP CramFS root in NOR
at `0x340000`, 256 MiB view), whose only job is to run `osloader` and load the
real system.

## The PWI config region

A plain text blob at NOR `0x020018`, read by both U-Boot and the userland
`osloader` (osloader reads it through `read_flashrom_data` in `libftm.so` /
`biflib.so`). Lines are `KEY<TAB>value`. The loader-relevant keys (HC59 dump,
verbatim):

```
OSLOADER_DEVICE_INFO_0          /dev/mmcblk1p1 vfat
OSLOADER_KWI_PATH_0             HD14/EXE/%ID%
OSLOADER_KWI_PATH_1             HD10/BKPRG/EXE
OSLOADER_PMB_PATH               pmb
OSLOADER_KERNEL_PATH            xipImage
OSLOADER_ROOTFS_PATH            rootfs
OSLOADER_USRCONF_PATH           USRCONF
OSLOADER_LOADING_GRP_PATH       loading.bmp
OSLOADER_INIT_GRP_PATH          /vns/osloader/init.bmp
OSLOADER_ERROR_GRP_PATH         /vns/osloader/error.bmp
OSLOADER_NOMEDIA_ERRDISP_TIME   3000
OSLOADER_NOMEDIA_POLL_TIME      100
OSLOADER_UPDATE_LOAD_SIZE       5242880
OSLOADER_LOADING_NORMAL_GRP_PATH  loading.bmp
OSLOADER_LOADING_PRGUP_GRP_PATH   grp_dat_13cy_prgup.bmp
OSLOADER_LOADING_MAPUP_GRP_PATH   grp_dat_13cy_mapup.bmp
```

Platform keys in the same blob that fix parts of the memory map (useful when
building/validating the OSMEM area and the display):

```
ERR_LOG_AREA         0x6024E000 0x1000
ICCOM_LOG_BACKUP_ADDR 0x60100000
ICCOM_LOG_SHARE_ADDR 0x8E7FE000
VP_FRONT_VRAM        0x8E800000 0x280000
VP_REAR_VRAM         0x8EA80000 0x280000      (dual display)
TK_EVT_MEM_SIZE      3145728
TK_{TSK,FLG,SEM,EVQ,MTX,EXT}_TBL_SIZE …       (T-Kernel/uITRON object tables)
```

`%ID%` in `OSLOADER_KWI_PATH_0` is the per-model selector (`HC59`, `HC60`, …).
It is a U-Boot variable; changing it is what makes the same resident NOR load a
different model's system. **This is the lowest-effort way to boot HC60 in the
emulator with the HC59 NOR: set `%ID%`/the model var to `HC60` and present the
HC60 map media.** The genuine HC60 KWI then passes U-Boot's CRC check unchanged.

## osloader — the loader

`/vns/bin/osloader` (ARM EABI5, ~26 KiB, Thumb-2, dynamically linked → it runs
as ordinary Linux userland under the recovery kernel; it is **not** a
pre-kernel bootloader). Observed behaviour, from its strings, imports, and
literal pools:

1. Read the PWI from NOR (`read_flashrom_data`).
2. Mount the OS media named by `OSLOADER_DEVICE_INFO_0` — **`/dev/mmcblk1p1`,
   vfat** — at `/vns/osloader/media`. This is the internal eMMC, not the
   removable map SD; the map card is copied to it (see the SD/update note in
   the platform doc). The map card itself is `/dev/sda` / SDHI3.
3. Build the KWI path from `OSLOADER_KWI_PATH_0` with `%ID%` substituted
   (`HD14/EXE/HC60/LOADING.KWI`), falling back to `OSLOADER_KWI_PATH_1`
   (`HD10/BKPRG/EXE/LOADING.KWI`).
4. Mount that KWI with **`kwifs.out`** (FUSE) at `/vns/osloader/kwi` → `/kwi`,
   exposing the container's members as files (see below).
5. Read the members named by the PWI — `rootfs`, `xipImage` (`KERNEL_PATH`),
   `USRCONF`, `pmb` — and write them into DRAM through **`/dev/mem`**
   (`open64`/`mmap`/`write`). osloader's literal pools contain `0x60000000`
   (OSMEM/DRAM base), `0x68000000` (rootfs target), and the PCRD magic
   `0x27051956`; the kernel/rootfs target addresses themselves come from the
   PCRD descriptor it writes.
6. Build the **OSMEM** shared area at `0x60000000` and stage **USRCONF** with
   the descriptor + checksums the product kernel's `bif_init` expects (this is
   the data the product kernel reads back out of surviving DRAM after the warm
   reboot).
7. Draw the right splash (`libScd.so` / `SCD_drawBmp` on `/dev/psc_ex0`):
   `init.bmp` → `loading.bmp` (normal) / `grp_dat_13cy_prgup.bmp` (program
   update) / `grp_dat_13cy_mapup.bmp` (map update); `error.bmp` on failure.
8. Set the warm-reboot flag in on-chip SRAM (`0xfe790000` block; the flag the
   platform doc places at `0xfe794000`) and trigger the warm reboot
   (`execvp`/`reboot`; no kexec).

osloader does **not** itself verify the program image — there is no
sig/sha/rsa/crc over the KWI inside osloader. The integrity gate is U-Boot's,
at step "U-Boot warm path" below.

## kwifs.out — the KWI as a filesystem

`/vns/bin/kwifs.out` (ARM EABI5, ~23 KiB, links `libfuse.so.2`, FUSE 2.6). It
`open64`s the `LOADING.KWI`, validates the 12-byte KWI signature
(`0f 56 a3 00 3c 3c 8a 00 07 00 01 66`), reads the container's `MOD_HEAD`
directory, and presents each member as a file under `/kwi` via an internal
`pfs_open` (seeking with `lseek64`). The member name table it carries:

| logical key | exposed file |
| --- | --- |
| `MOD_HEAD` | `module_header.bin` |
| `GRP1_FILE` / `GRP2_FILE` | `grpdata1.bin` / `grpdata2.bin` |
| `RU_FILE` | `H2O-R.bin` |
| `PMB1_FILE`..`PMB4_FILE` (+ `_MIR`) | `pmb1.bin`..`pmb4.bin` (+ `_mir`) |

plus `rootfs`, `xipImage`, `USRCONF`, `pmb` (the names the PWI uses). The file
offsets of members come from the `MOD_HEAD` directory, **not** from the
manifest's logical addresses — see [alphard-navi-kwi-format.md](alphard-navi-kwi-format.md)
for why (the reserved region packs the small members out of logical order).

A direct-boot loader does not need FUSE: it can read the manifest and the two
members that matter (`rootfs`, `xipImage`) straight out of the file — `rootfs`
is a raw ext2 at a fixed offset; see the format doc.

## The PCRD boot descriptor — the handoff structure

This is the contract between osloader and U-Boot, and the single most useful
structure for driving the boot. osloader writes it; U-Boot reads it on the warm
path. In the KWI it lives in the reserved region at **file offset `0x117e02`**
(immediately followed at `0x118002` by the `PCRD` marker block). Little-endian,
32-byte header then the kernel command line:

| off | field | HC59 | HC60 | note |
| --- | --- | --- | --- | --- |
| 0x00 | magic | `0x27051956` | `0x27051956` | same value as a uImage magic |
| 0x04 | header crc | `0x9e2b886c` | `0x28e7d110` | per-model |
| 0x08 | load_base | `0x62e00000` | `0x62e00000` | base of the staged preamble |
| 0x0c | **kernel_load** | `0x62f80000` | `0x62f80000` | where the XIP kernel runs |
| 0x10 | kernel_size | `0x00417914` | `0x004178e0` | = manifest `xipImage` size |
| 0x14 | kernel_cksum | `0x19a823f5` | `0x58795fcf` | CRC U-Boot verifies |
| 0x18 | **rootfs_load** | `0x68000000` | `0x68000000` | where the ext2 root lands |
| 0x1c | rootfs_size | `0x0ff40000` | `0x0ff40000` | = manifest `rootfs` size |
| 0x20 | cmdline | (see below) | identical | NUL-terminated ASCII |

Kernel command line (identical HC59/HC60):

```
console=ttyS0,115200 root=/dev/pcrd rootflags=xip lpj=963379 mem=674M quiet \
pcrd=0x68000000,xip ro max_loop=21
```

`root=/dev/pcrd` + `pcrd=0x68000000,xip` = the product root is the ext2 at
`0x68000000`, mounted execute-in-place, read-only, by the kernel's built-in
`pcrd` driver (no initramfs). `console=ttyS0` is vestigial — the real hardware
has no live UART (see the platform doc).

At `0x118002`, right after the descriptor's 0x200 slot, is the `PCRD` marker
block: the ASCII model/version tag (`VC5910KA` for HC59, `VC6010KA` for HC60)
immediately before `PCRD`, then a 32-byte digest. The digest's **low 16 bytes
are per-model; the high 16 bytes are a constant trailer** shared across models
(`…21b54f5d 0d1c9707 8845ed58 a4ec3502 d0175ced`) — treat it as a 16-byte image
digest plus a 16-byte fixed tag.

## U-Boot — the warm-boot path and the integrity gate

U-Boot's `bootm` is a standard legacy-image boot: it validates the uImage
header checksum and the data CRC before it will start a kernel ("Verifying
Checksum", "Bad Data CRC", "Bad Magic Number", "Could not find kernel entry
point!"). The PCRD descriptor reuses the uImage magic precisely so the staged
kernel is bootable this way, and the `kernel_cksum` field is the CRC U-Boot
checks. That check is the **"a modified image is refused and the recovery
kernel boots instead"** behaviour: tamper with the staged kernel and the CRC
mismatches, so U-Boot falls back to the recovery path. A genuine, unmodified
KWI (any model's) passes.

The `OSLOADER_*` keys and `HD14/EXE/%ID%` also appear inside the U-Boot image —
they are the compiled-in defaults for the PWI env, which is why both U-Boot and
userland agree on paths and the model variable.

## Memory map of a staged boot

DRAM is 1 GiB at `0x60000000` (R-Car gen1 base). The staged layout the loader
produces:

| address | region |
| --- | --- |
| `0x60000000` | **OSMEM** shared area (osloader → kernel `bif_init`/`initOSsysMemLib`) |
| `0x60100000` | ICCOM inter-core log backup (PWI `ICCOM_LOG_BACKUP_ADDR`) |
| `0x6024e000` | error-log area (PWI `ERR_LOG_AREA`, 0x1000) |
| `0x62e00000` | staged preamble base (PCRD `load_base`) |
| `0x62f80000` | **XIP kernel** (PCRD `kernel_load`); runs in place |
| `0x68000000` | **ext2 root**, read-only XIP (`root=/dev/pcrd`) |
| `0x8e7fe000` | ICCOM log share |
| `0x8e800000` | front-display VRAM (0x280000); `0x8ea80000` rear-display VRAM |
| `0xfe790000` | on-chip SRAM (64 KiB); warm-reboot flag at `0xfe794000` |

`mem=674M` from `0x60000000` ends about `0x89a00000`, so the VRAM planes at
`0x8e800000` sit in reserved space above the kernel's RAM.

Recovery context differs: the recovery kernel is loaded by `bootm` from NOR
`0x40000` to `0x80008000`, `mem=256M`, root = the NOR CramFS XIP at physical
`0x340000`.

## Driving it in rsemu (two options)

**Option B — reuse the HC59 NOR, change the model.** Boot the real HC59 NOR,
set the U-Boot model variable (`%ID%`) to the target (`HC60`), and give the
machine that model's map media. The resident `osloader` does everything above
with genuine code; the genuine KWI passes U-Boot's CRC. Lowest effort, highest
fidelity. Caveat: a few NOR/PWI params are model-specific (the `pmb` blocks,
SCAC security config, display setup); a mismatch shows up as a specific driver
complaint, not a silent hang.

**Option A — synthesize the handoff (no NOR needed for any model).** This is
implemented: the `navi.kwiboot` device (`src/dev/navi/kwiboot.rs`), selected by
giving the machine a KWI instead of a flash image:

```
rsemu run alphard-navi --media kwi=HD14/EXE/HC59/LOADING.KWI --drive sd=card.img
```

It reproduces the state the real chain leaves at the instant U-Boot jumps,
which was captured by running the HC59 NOR with the HC59 card in rsemu and
stopping on the product kernel's first instruction (3.76 s of virtual time):

1. **Members** go where the boot descriptor (`pmb`) says: the whole `rootfs`
   member — `PCRD` block first, the ext2 0x40000 bytes in — at `rootfs_load`
   (`0x68000000`), `xipImage` at `kernel_load` (`0x62f80000`), and `USRCONF`
   at the address in `pmb` +0x1ec (`0x6010c000`).
2. **ATAGs** at `load_base + 0x100` (`0x62e00100`): `CORE` (all zero), `MEM`
   from `load_base` to the end of DRAM (`0x3d200000` bytes at `0x62e00000`),
   and `CMDLINE` = the `pmb` command line.
3. **OSMEM** records at `0x60000000` — see the next section.
4. **Entry**: `r0 = 0`, `r1 = 0xd3a` (machine type 3386, BOCK-W), `r2` = the
   ATAGs, PC = `kernel_load`, SVC mode, MMU and caches off. A four-word stub
   mapped over the reset vector does this; the kernel's own entry code was
   seen to keep exactly these `r1`/`r2` at 1 ms, identical between the two
   boots.

Entering the kernel directly skips U-Boot's CRC gate, so the loader checks
the CRC itself (and USRCONF's sum) and refuses a modified image, as U-Boot
would.

## The OSMEM area as the chain leaves it

The product kernel finds its records through a table compiled into it (HC59
XIP VA `0x7f378988`, 16-byte rows: offset, record size, data size, kind).
Records are a value and its complement where the kind is `0x101`. Observed at
the jump, and what `navi.kwiboot` writes:

| offset | contents | source |
| --- | --- | --- |
| `0x000` | `OS M0017` | constant (also in U-Boot) |
| `0x010`–`0x034` | records 1–5: `0, 1, 1, 1, 0`, each with its complement | constant — but see record 4 |
| `0x130`–`0x144` | `0, 0, 1/~1, 0/~0` | constant |
| `0x150` | `ff 00 00 00 00 ff ff ff`, the version tag (`VC5910KA`) and its complement, `0xff` to `0x278`, `PMB_DATA` | tag from `pmb` +0x1f8 |
| `0x400`–`0x42c` | six `0/~0` pairs | constant |
| `0x460` | `KWI `, the 32-bit word sum of the next 24 bytes, the path `/HD14/EXE/HC59` NUL-padded to 24 | model code from the KWI header |
| `0x4b0` | `CONF`, the word sum of the next three words, USRCONF's word sum, length and address, then 256 × `0xff` | `pmb` +0x1ec..0x1f8 |
| `0x1000` | magic `0x0a375fe6` (a U-Boot literal), CRC-32 of bytes 8..len, len (`0x1c` + command line), `load_base`, `kernel_load`, `kernel_size`, `kernel_crc`, the command line | `pmb` |
| `0x2010` | `OS L`, `0`, `1`, `0`, `0x55555555` | constant |
| `0x4030`, `0xb040`, `0xb140` | small constant records | constant |
| `0x24e800` | `0001 fffe`, `0`, `0x12345678`, `0xedcba987` (error-log header) | constant |

Not reproduced: the recovery system's process table at `0x2040`–`0x3180`
(it names `osloader`) and an error-log counter at `0x24e000`. Writing them
made no difference.

**Record 4 (`+0x28`) says where USRCONF is.** The real chain leaves `1`, and
the product kernel's `CONFMEM_init` then maps the NOR at physical `0x20000`
and expects a `USRCONF ` block there (the "PWI config region" is that block's
text, after a 0x18-byte header). With no flash that check fails and the kernel
calls `requestHardReset(3)` about a second in. `0` makes it use the RAM copy
the `CONF` record describes, so `navi.kwiboot` writes `0`. The NOR's block on
the HC59 unit is an older `HC02100B` copy of the same text; the KWI's has
every key it has and a few more.

## Open items

* **The PCRD digest** at `0x11800a` — whether anything checks it, and what
  produces the 16-byte per-image half. Neither boot path in rsemu needs it.
* **What else the product system reads from the NOR.** With `navi.kwiboot`
  and no flash image the NOR reads as erased; anything in userland that reads
  an MTD partition gets `0xff`.
