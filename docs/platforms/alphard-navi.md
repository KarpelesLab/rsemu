# alphard-navi — the Toyota Alphard navigation computer

```
rsemu run alphard-navi --media flash=S29JL064J.bin --media sd=map.img --for 10s --screenshot shot.png
```

The head unit in a 2015 Toyota Alphard (GGH30), Toyota part 86100-58182, is a
Panasonic chassis carrying an **Aisin AW computer board**, 99370-00649. The
board is a Renesas R-Car Gen1 (the SoC is marked R8A77791) with 1 GiB of DDR3
on two channels and an 8 MiB Spansion **S29JL064J** NOR flash (IC400) on the
local bus at address zero. `machines/alphard-navi.machine` is that board.

The NOR image is proprietary and is not in this tree; the `flash` slot takes
a dump of it. Without one the board runs a `B .` at the reset vector. The `sd`
slot takes an image of the map SD card (read-only is fine); without one the
socket is empty and the unit runs its no-card path.

## What is on the flash

| offset | contents |
| --- | --- |
| `0x000000` | U-Boot 2011.03 (Aisin fork, "EXCIOS M0017") |
| `0x040000` | uImage: Linux 2.6.35.14, load and entry `0x80008000`, uncompressed |
| `0x340000` | CramFS: the recovery root (`rootflags=physaddr=0x340000`) |

The boot command is `bootm 40000` with
`console=ttyS0,115200 root=/dev/null rootflags=physaddr=0x340000 mem=256M ro`.
The recovery root's `init` is Aisin's process manager `/vns/bin/pmng`, which
starts `osloader`; osloader's job is to load the full system from the map SD
card and warm-reboot into it, leaving a flag in on-chip SRAM at `0xfe794000`.

## How the map was learnt

Renesas's R-Car hardware manual is not public, and the Linux and U-Boot trees
that describe the SoC are GPL and are not read here (CLAUDE.md, provenance).
So every address, interrupt number and clock rate in the machine file was
learnt by **running the flash image and watching what it touched**:

* a scratch harness mapped a logging register file under every unmodelled
  address and printed each access with the program counter that made it;
* the kernel's own static I/O table and `struct resource` tables were read out
  of the binary image as data;
* the kernel's `kallsyms` table was decoded, so backtraces and program counters
  have names.

Findings that are not obvious from the part numbers:

* **The kernel's machine is `BOCK-W`**, Renesas's name for its R-Car *M1A*
  reference board, and U-Boot's banner says `R-CarM1A`. The owner reports the
  same U-Boot fork is used across products; the map below is the one this
  firmware uses, whatever the silicon's marketing name.
* **There is no console.** `printk` in this production kernel is a four-
  instruction stub that returns without formatting anything, and no driver
  registers `ttyS0` — the only serial driver is Aisin's `scif_iif`, which moves
  data to the sub-processors over the SCIFs by DMA. The real unit therefore
  prints nothing on any UART. A bring-up harness can restore the log by
  pointing `printk` (`0x8020c5b4`) at the intact `vprintk` (`0x8003d0cc`) in
  guest RAM after U-Boot has copied the kernel.
* **Any fatal user-mode fault reboots the unit.** `__do_user_fault` calls
  `v5plus_except_reset`, which ends in `requestHardReset`: a flag in SRAM, bit
  30 of GPIO bank 0 (a reset request to the sub-processor), and a `B .`. Setting
  the kernel's `user_debug` (`0x802f9764`) to 31 makes it print the faulting
  registers first.
* **The unit is only alive while the base board says so.** The base board's
  MN103 microcontroller talks to the kernel over SCIF3 with a packet protocol
  the kernel calls PSC (below), and after START it must send a STAT notice at
  least every 500 ms: `psc_ltc_BreakCycleNti` checks, and a missed notice
  calls `psc_tif_ResetNavi`. A second, independent guard is the external
  watchdog on GPIO 156 (bank 4 pin 28), toggled from TMU1 channel 0's
  interrupt (every 20 ms) by `touch_v5plus_watchdog` for as long as the
  kernel's soft-watchdog counter (3000, reloaded by
  `touch_v5plus_soft_watchdog`) has not run out.
* The kernel assumes a **60.288 MHz** peripheral clock: it programs TMU0 for a
  1 ms tick with `TCOR = 0x3ae0` at Pφ/4. The machine file runs the TMUs at that
  rate so kernel time and machine time agree.

## Memory map

| address | block | here |
| --- | --- | --- |
| `0x00000000` | NOR, 8 MiB | `flash.cfi`, AMD command set |
| `0x18100000` | a device on an LBSC chip select | placeholder |
| `0x60000000` | DDR3, 1 GiB (two channels) | `ram` |
| `0xf0000000` | Cortex-A9 private region: SCU, GIC CPU interface, timers, GIC distributor | `arm.a9mpcore`, `arm.gic` (v1) |
| `0xf0100000` | L2C-310 | `arm.l2c310` |
| `0xfe700000` | interrupt mask registers | placeholder |
| `0xfe780000` | interrupt controller block | placeholder, with the HPB-DMAC status mirror at `0xfe782000` |
| `0xfe790000` | on-chip SRAM, 64 KiB | `ram` |
| `0xfe800000`, `0xfec00000` | DDR controllers | placeholder |
| `0xff800000` | local bus controller | placeholder |
| `0xffc08000` | HPB-DMAC channels, common registers at `0xffc09000` | `rcar.hpbdmac` |
| `0xffc40000` | GPIO banks 0–6 (IDs 173–179) | `rcar.gpio` |
| `0xffc70000` | I2C 0–2 (IDs 111, 114, 112) | placeholder |
| `0xffc80000` | clock pulse generator | placeholder |
| `0xffcc0000` | power control | placeholder |
| `0xffd80000`–`0xffd82000` | TMU0–2 | `rcar.tmu` |
| `0xffe40000` | SCIF0 | `rcar.scif` |
| `0xffe41000` | SCIF1–5 (sub-processor links; SCIF3 is the PSC link) | `rcar.scif` |
| `0xffe4c000` | SDHI0, SDHI2, SDHI3 (IDs 136, 139, 138); SDHI1 unused | `rcar.sdhi` |
| `0xfff80000` | Display Unit (ID 63) | `rcar.du` |
| `0xfffc0000` | pin function controller | placeholder |

A placeholder is a plain `ram` object: it reads back what was written and does
nothing else. That carries the boot code's write-then-read-back sequences but
is not a model; the power controller's SGX status poll, for one, times out
there ("power on SGX error!"), exactly as it would on a board whose power domain
never came up.

Interrupts confirmed so far: TMU0 channel 0 is GIC ID 64 (the tick), TMU1
channel 0 is ID 68, SCIF*n* is 120+*n*, GPIO bank *n* is 173+*n*. The
HPB-DMAC's channels are virtual IRQs 260+*n* that the kernel demultiplexes
from twelve group lines (IDs 142–153) by reading the status words at
`0xfe782104` and `0xfe7820f0`.

## The PSC link

The base board's MCU owns power, the fans and a bank of I/O the SoC cannot
reach — including the SD slot's supply, which the kernel switches with a PORTW
command. Frames are `01 CMD LEN payload CS` with
`CS = (CMD + LEN + Σpayload) & 0xff`; the host opens with the sync frame
`0f 00 01 00 01`. The MCU acknowledges every frame (`01 06 00 06`) and answers
VERG, PORTR and FANRPMGET with VERD, PORTD and FANRPMD; START is answered with
STAT and turns on the cyclic STAT notice. `navi.psc` is that peer, on a
`bus::uart` link to SCIF3, which runs by DMA (HPB-DMAC channel 6 transmit, 7
receive). Its `log` port prints one line per command.

## The SD card

The map card is in the SDHI3 socket; card detect is GPIO 90 (bank 2 pin 26),
and userspace also watches GPIO 92 (bank 2 pin 28). Data moves by DMA on
HPB-DMAC channel 39 in 16-bit units, one scatter-gather segment per register
set load. The card is FAT32 on an MBR partition; osloader looks for
`HD14/EXE/<model>/LOADING.KWI`, where the model (`HC59` on this unit) is a
U-Boot variable.

## Status

| stage | state |
| --- | --- |
| U-Boot: pin setup, DDR init from SRAM, relocation, `bootm` | runs |
| Kernel: MMU, GIC, TMU tick and clocksource, calibration, drivers | runs |
| Kernel: NOR via CFI (4 partitions), framebuffer, I2C, SDHI probe, VFP | runs |
| Kernel: CramFS root mounted, `init` started | runs |
| Userspace: `pmng` (Thumb-2) loads `tab_dd.ko`, exports its GPIOs, starts `osloader` | runs |
| PSC link to the base-board MCU: sync, START, VERG, PORTW (SD slot power), cyclic STAT | runs, against `navi.psc` |
| Map SD card: enumeration, SCR, partition table, FAT32 reads by DMA | runs |
| `osloader` reads `HD14/EXE/HC59/LOADING.KWI` into RAM (xipImage at `0x62f80000`, rootfs at `0x68000000`) and requests the hot reboot | runs; the loaded image matches the file byte for byte |
| U-Boot checks the loaded image and boots the XIP kernel from it | runs (a patched image is refused and the recovery kernel boots instead) |
| XIP kernel | takes a fault during an initcall; its handler logs the context and requests a reset through GPIO 0 bit 30 |
| Sub-processor link (`cis`, over HSPI) | fails: `cis:trans NG(-512)` |
| I2C | placeholder: `i2c-1 Fatal error Cancel timeout` |

The vendor binaries under `/vns` (`pmng`, `osloader`, `smng`) are ARMv7
**Thumb-2**; glibc and busybox are ARMv6 ARM/Thumb-1 with VFPv2.

Twenty guest seconds take a little over three minutes of host time on an
M-series Mac with the interpreter; the splash is up within the first ten, and
the loaded kernel starts after roughly seventy.

What would move it further: whatever the XIP kernel's faulting initcall wants
(and the reset request on GPIO 0 bit 30 wired to the MCU, so it reboots rather
than waiting for the watchdog); the I2C controllers; and the HSPI controller
with a model of the sub-processor at its other end.
