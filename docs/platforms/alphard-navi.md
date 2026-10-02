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
* **There is no console.** `printk` in both production kernels is a four-
  instruction stub that returns without formatting anything, and no driver
  registers `ttyS0` — the only serial driver is Aisin's `scif_iif`, which moves
  data to the sub-processors over the SCIFs by DMA. The real unit therefore
  prints nothing on any UART. The machine file carries two `linux.printk`
  taps (a debugging aid, not part of the board) that patch each kernel's
  `printk` to call its intact `vprintk` and copy the log ring to a `klog`
  port. Read it with `--console klog`, or with `--capture klog`
  alongside `--window`:

  ```
  rsemu run alphard-navi --media flash=S29JL064J.bin --drive sd=map.img --window --capture klog
  ```
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
| `0x18300000` | the SCAC security device (16-bit, big-endian ID at offset 0: `0xa001`) | placeholder; reads absent ("sec device not present"), which the system tolerates |
| `0x60000000` | DDR3, 1 GiB (two channels) | `ram` |
| `0xf0000000` | Cortex-A9 private region: SCU, GIC CPU interface, timers, GIC distributor | `arm.a9mpcore`, `arm.gic` (v1) |
| `0xf0100000` | L2C-310 | `arm.l2c310` |
| `0xfce00000` | PowerVR SGX (ID 89) | `pvr.sgx`: the firmware's handshake, no rendering |
| `0xfe700000` | interrupt mask registers | placeholder |
| `0xfe780000` | interrupt controller block | placeholder, with the HPB-DMAC status mirror at `0xfe782000` |
| `0xfe790000` | on-chip SRAM, 64 KiB | `ram` |
| `0xfe800000`, `0xfec00000` | DDR controllers | placeholder |
| `0xff800000` | local bus controller | placeholder |
| `0xffc08000` | HPB-DMAC channels, common registers at `0xffc09000` | `rcar.hpbdmac` |
| `0xffc40000` | GPIO banks 0–6 (IDs 173–179) | `rcar.gpio` |
| `0xffc50000` | video capture (the SD kernel's `vc_*` driver) | placeholder |
| `0xffc70000` | I2C 0–2 (IDs 111, 114, 112) | `rcar.i2c` |
| `0xffc80000` | clock pulse generator | placeholder |
| `0xffcc0000` | unidentified; read-modify-written by the CAN driver's reset | placeholder |
| `0xffd80000`–`0xffd82000` | TMU0–2 | `rcar.tmu` |
| `0xffd85000` | SYSC power domains | `rcar.sysc` |
| `0xffe40000` | SCIF0 | `rcar.scif` |
| `0xffe41000` | SCIF1–5 (sub-processor links; SCIF3 is the PSC link) | `rcar.scif` |
| `0xffe4c000` | SDHI0, SDHI2, SDHI3 (IDs 136, 139, 138); SDHI1 unused | `rcar.sdhi` |
| `0xfff80000` | Display Unit (ID 63) | `rcar.du` |
| `0xfffc0000` | pin function controller | placeholder |
| `0xfffd1000` | CAN channel 1 (ID 116) | `rcar.can`: a silent bus |
| `0xfffc9000` | IEBus controller (ID 140) | `rcar.iebus`: a silent bus |
| `0xfe6cf000`, `0xfe700040` | secondary-core reset control, boot address | `rcar.rst` |

A placeholder is a plain `ram` object: it reads back what was written and does
nothing else. That carries the boot code's write-then-read-back sequences but
is not a model, and every model in the table above began as one of these
until a driver's poll on it timed out: the SGX power domain ("power on SGX
error!" until `rcar.sysc`), the CAN controller's mode handshake (below).

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

A genuine map card is password locked. Both kernels unlock it with CMD42
before reading — a one-block DMA write of the flags byte, a length byte and a
sixteen-byte password, at the length CMD16 set — and the full system checks
it again after it loads its own SD driver. `sd.card` models the lock; run the
machine with `-p map-password=...` to present a locked card, or leave it empty
for an unlocked one, which the unit also reads. The password is not in this
tree.

The map card is in the SDHI3 socket; card detect is GPIO 90 (bank 2 pin 26),
and userspace also watches GPIO 92 (bank 2 pin 28). Data moves by DMA on
HPB-DMAC channel 39 in 16-bit units, one scatter-gather segment per register
set load, in both directions: the driver reprograms the channel's DCR for a
write. In the full system the SD hosts are exposed to a user-space driver
through UIO, and `tmio_mmc` is loaded as a module about eighteen guest seconds
in. The card is FAT32 on an MBR partition; osloader looks for
`HD14/EXE/<model>/LOADING.KWI`, where the model (`HC59` on this unit) is a
U-Boot variable.

## Status

| stage | state |
| --- | --- |
| U-Boot: pin setup, DDR init from SRAM, relocation, `bootm` | runs |
| Recovery kernel: MMU, GIC, TMU, NOR via CFI, framebuffer, I2C, HSPI, SDHI, VFP | runs |
| Recovery userspace: `pmng` starts `osloader`, which draws its splash | runs |
| PSC link to the base-board MCU: sync, START, VERG, PORTW (SD slot power), cyclic STAT, reset requests | runs, against `navi.psc` |
| Map SD card: enumeration, partition table, FAT32 reads by DMA | runs |
| `osloader` loads `HD14/EXE/HC59/LOADING.KWI` (xipImage at `0x62f80000`, rootfs at `0x68000000`) and asks for the hot reboot | runs; the loaded image matches the file byte for byte |
| U-Boot checks the loaded image and boots its XIP kernel | runs (a modified image is refused and the recovery kernel boots instead) |
| Full system: XIP kernel, ext2 root in RAM, init scripts, udev, vendor modules, I2C devices, PSC wake-up | runs; it shows 「プログラム読込み中」 with its progress bar |
| PowerVR SGX | `pvr.sgx` stand-in: the driver initialises; nothing is rendered |
| DC-DC monitor on HSPI channel 0 | `navi.dcdcad`: `DCDC Version 2` |
| CAN channel 1 | `rcar.can`, a silent bus: the channel starts and its readers park |
| Full system past the driver load: application layer, HMI screen layers (`MAPPARENT` among them) | runs, on four cores, without a reset for as long as it has been run (three guest minutes); the screen is black, because the HMI draws through OpenGL ES and `pvr.sgx` renders nothing |
| Four Cortex-A9s | both kernels bring up four CPUs: `rcar.rst` releases the secondaries at the boot address, the GIC's SGIs are always on, and the cores share `arm.exclusive` |
| IEBus (AVC-LAN) | `rcar.iebus`, a silent bus: frames are delivered, nothing arrives |
| Sub-processor links (`cis`, HSPI channels 1 and 2) | nothing behind them |

The vendor binaries under `/vns` (`pmng`, `osloader`, `smng`) are ARMv7
**Thumb-2**; glibc and busybox are ARMv6 ARM/Thumb-1 with VFPv2.

With a map card the full system starts about six guest seconds in. Sixteen
guest seconds take about four minutes of host time on an M-series Mac with
the interpreter. Run it with the card file-backed (`--drive`, so the 32 GB
image is not read into memory; add `,ro` to write-protect it):

```
rsemu run alphard-navi --media flash=S29JL064J.bin --drive sd=map.img --for 16s --headless --screenshot shot.png
```

## The watchdogs

Two, and a reboot with nothing in the kernel log is one of them:

* **The sub-processor's**, outside the SoC: the TMU1 interrupt toggles GPIO
  4.28, and silence on the pin for two seconds resets the board
  (`watchdog.pin`; its `log` port, `--capture wdt=...`, says when it fires).
* **Aisin's software watchdog**, inside the kernel, which decides whether that
  pin keeps toggling. `WDP_start` loads a countdown of `WDT_CYCLCHK` × 50
  TMU1 ticks (60 × 50 at 50 Hz: a minute) and starts `WDP_task`, a real-time
  kernel thread that reloads it every `WDT_INTERVAL` ms (5000). At zero the
  tick handler calls `requestHardReset` and stops kicking. Anything that keeps
  `WDP_task` off the CPU for a minute — a real-time user thread spinning
  because a device answers at once where hardware would block — resets the
  board a minute later. The CAN reader `LCAN02` (SCHED_FIFO 74) did exactly
  that until `rcar.can` let the channel start.

The monitor finds these faster than a trace: it is deterministic, so a script
can run to just before the reset and sample.

```
printf 'run 60s\nx 816dd0bc 4\nregs\nquit\n' |
  rsemu run alphard-navi --media flash=... --drive sd=... -p map-password=... --headless --mon
```

`0x816dd0bc` is CPU 0's countdown in the SD system's kernel (VA); a value that
only ever falls is a starved `WDP_task`. A sample in SVC mode names the
running task: its `thread_info` is `sp & ~0x1fff`, the task pointer at +12,
and the task's name at +0x1dc.

## The I2C and SPI buses

The I2C parts sit where the kernel's own `i2cfs` table puts them (33
entries; every transfer goes through it): bus 0 the Apple MFi coprocessor
(0x10); bus 1 the AK7734 DSP (0x18), the two ADV7186 decoders (0x60/0x61,
each with ten sub-map addresses), the CXD4905 GVIF (0x27), the rear-camera
ADV7180 (0x20) and the USB hub (0x2c); bus 2 the RTC (0x51), the touch panel
(0x5c) and the EEPROM (0x54–0x57). Each is an `i2c.regfile` stand-in that
answers and reads back. The kernel's board-info table lists the same parts on
bus 0 and generates no traffic.

The HSPI's three channels are the driver's 0, 1 and 2 at `0xfffc7000`,
`0xfffc8000` and `0xfffc6000` (GIC 105–107): channel 0 to the DC-DC/ADC
monitor, 1 and 2 the CIS links to the base-board sub-CPUs, which raise GPIO
134 and 146 when they have a 64-byte frame.

## What would move it further

* Rendering: the HMI's OpenGL ES output, which needs the SGX to draw.
* The SCAC security chip (`sammc` polls its status and times out every few
  seconds).
* The CIS peers on HSPI channels 1 and 2.
* Behaviour behind the I2C stand-ins the full system checks (the decoders'
  status registers, the RTC's time, the EEPROM's contents).
* The full system's own displays past the loading screen: the HMI draws
  through OpenGL ES on the SGX, which `pvr.sgx` initialises but does not
  render for.
* USB: the host bridge at `0xffe70800` must report its PLL locked (`0x808`
  bits 31 and 30) or the kernel drops both host controllers.
