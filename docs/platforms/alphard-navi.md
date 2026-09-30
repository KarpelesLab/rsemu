# alphard-navi — the Toyota Alphard navigation computer

```
rsemu run alphard-navi --media flash=S29JL064J.bin --for 10s --screenshot shot.png
```

The head unit in a 2015 Toyota Alphard (GGH30), Toyota part 86100-58182, is a
Panasonic chassis carrying an **Aisin AW computer board**, 99370-00649. The
board is a Renesas R-Car Gen1 (the SoC is marked R8A77791) with 1 GiB of DDR3
on two channels and an 8 MiB Spansion **S29JL064J** NOR flash (IC400) on the
local bus at address zero. `machines/alphard-navi.machine` is that board.

The NOR image is proprietary and is not in this tree; the `flash` slot takes
a dump of it. Without one the board runs a `B .` at the reset vector.

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
| `0xfe780000` | on-chip SRAM | `ram` |
| `0xfe800000`, `0xfec00000` | DDR controllers | placeholder |
| `0xff800000` | local bus controller | placeholder |
| `0xffc40000` | GPIO banks 0–6 | placeholder |
| `0xffc70000` | I2C 0–2 (IDs 111, 114, 112) | placeholder |
| `0xffc80000` | clock pulse generator | placeholder |
| `0xffcc0000` | power control | placeholder |
| `0xffd80000`–`0xffd82000` | TMU0–2 | `rcar.tmu` |
| `0xffe40000` | SCIF0 | `rcar.scif` |
| `0xffe41000` | SCIF1–5 (sub-processor links) | placeholder |
| `0xffe4c000` | SDHI0–3 (IDs 136, 138, 139) | placeholder |
| `0xfff80000` | Display Unit (ID 63) | `rcar.du` |
| `0xfffc0000` | pin function controller | placeholder |

A placeholder is a plain `ram` object: it reads back what was written and does
nothing else. That carries the boot code's write-then-read-back sequences but
is not a model; the power controller's SGX status poll, for one, times out
there ("power on SGX error!"), exactly as it would on a board whose power domain
never came up.

Interrupts confirmed so far: TMU0 channel 0 is GIC ID 64 (the tick), TMU1
channel 0 is ID 68.

## Status

| stage | state |
| --- | --- |
| U-Boot: pin setup, DDR init from SRAM, relocation, `bootm` | runs |
| Kernel: MMU, GIC, TMU tick and clocksource, calibration, drivers | runs |
| Kernel: NOR via CFI (4 partitions), framebuffer, I2C, SDHI probe, VFP | runs |
| Kernel: CramFS root mounted, `init` started | runs |
| Userspace: `pmng` (Thumb-2) loads `tab_dd.ko`, exports its GPIOs, starts `osloader` | runs |
| `osloader` draws its splash: 地図ディスクを確認しています / しばらくお待ち下さい ("checking the map disc, please wait") | runs, and waits |
| Sub-processor link (`cis`, over HSPI) | fails: `cis:trans NG(-512)`, userspace logs `[PSC Debug] Wakeup-NG` |
| Map SD card (SDHI) | not modelled: placeholder |

The vendor binaries under `/vns` (`pmng`, `osloader`, `smng`) are ARMv7
**Thumb-2**; glibc and busybox are ARMv6 ARM/Thumb-1 with VFPv2.

Twenty guest seconds take a little over three minutes of host time on an
M-series Mac with the interpreter; the splash is up within the first ten.

What would move it further: an SDHI model with a card image, so osloader has a
map disc to check; and the HSPI controller with a model of the sub-processor at
its other end, which answers the wake-up handshake.
