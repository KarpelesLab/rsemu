# `q35-linux` — the q35 chipset with a kernel in the socket, and a disk to find

Consumed by [`machines/q35-linux.machine`](../../machines/q35-linux.machine),
[`tests/q35_linux.rs`](../../tests/q35_linux.rs) and
[`src/dev/q35`](../../src/dev/q35). The chipset is [`q35`](q35.md)'s and the
entry is [`pc64`](pc64.md)'s; neither of those pages is repeated here.

## Why a third board and not a flag on one of the two

[`q35`](q35.md) has the chipset — ECAM, the PAM file at `0x90`, the ICH9's
`PIRQ` routers, ACPI tables generated from the realized machine — and therefore
a PCI bus with somewhere to put a disk controller. What it has is a **firmware**
socket, and what runs on it starts in real mode at `0xfffffff0`.

[`pc64`](pc64.md) has the other half: `x86.linuxboot` writes a `bzImage` into
memory and enters it at its own 32-bit entry point. What it has not got is a
PCI bus, so an initramfs is the only root a kernel on it can be given.

They cannot be one board, and the reason is one address: `x86.linuxboot` maps
its reset stub at the top of the address space, which is exactly where the
firmware socket is. A board with both would have two things decoding
`0xfffffff0`. So this is the third — `q35`'s chipset, `pc64`'s entry, and an
**NVM Express controller** at `00:04.0`.

Three routes were considered and this is the second of them.

1. **A boot loader on the disk image.** Firmware POSTs, `INT 13h` loads a boot
   sector, the boot sector loads a kernel. Closest to a real machine, and the
   right answer eventually — Windows 95 and XP (`ROADMAP.md` 6c) have no other
   route. It was rejected *for this step* because the loader would have to be
   written here (every boot loader in circulation is GPL, and §1 forbids
   reading one), it would be several hundred instructions of 16-bit code that
   re-implements what `src/dev/linuxboot.rs` already does correctly, and a
   failure in it would be a failure in a boot loader rather than in the board.
2. **This board.** Cheapest, and it turns out to cost nothing in honesty: the
   ACPI tables on a q35 are **not published by firmware** here. They are a
   device with a region, and the machine file maps that region inside ACPI
   §5.2.5.1's own RSDP search window — so a kernel entered by `x86.linuxboot`
   scans `0xe0000`-`0xfffff` and finds them with nothing having staged them.
   `x86.linuxboot` did not have to learn to publish anything.
3. **Reusing `nvme-mini`'s shape on `pc64`** — a `pc.pmc` host bridge and an
   NVMe controller, no chipset. Rejected because a kernel needs `_PRT` to route
   a PCI interrupt and `_PRT` needs a south bridge with routers in it, which is
   the thing `q35` has and `pc-at` does not.

## Why NVMe, and not the three other controllers in this tree

Because of what a *stock distribution kernel* has built in, which decides this
far more than which controller is easiest to wire. A distribution builds nearly
every storage driver as a module, and a module lives on the root that has not
been mounted yet.

Measured against a stock Gentoo `6.6.67` build's own `modules.builtin`, which is
the kernel saying what it linked in:

| Driver | In that kernel |
| --- | --- |
| `drivers/nvme/host/nvme.ko` | **built in** — and self-contained: an NVMe namespace becomes `/dev/nvme0n1` through blk-mq with no SCSI layer under it |
| `drivers/ata/libata.ko` | built in, but `libahci` and `ahci` are modules — and `sd_mod` is a module too, so even a bound AHCI adapter would publish no block device |
| `drivers/scsi/scsi_mod.ko` | built in; `sd_mod` is not |
| virtio, of any transport | absent from the list entirely |

So NVMe is the one controller a kernel that has never heard of rsemu can drive
off a board like this. A kernel with a different set built in is a different
answer, and the machine file is one `object` away from either of the others.

## What is here that `q35` has not, and what is gone

Added: `x86.linuxboot`, and `nvme.controller` at device 4.

Gone: the firmware socket, the video adapter and its option ROM, the 8042, the
two 8237As and both IDE channels. Two of those absences are load bearing rather
than tidying:

* **A20 is deliberately not wired.** A net with a driver comes up low and a low
  `a20` pin shuts the gate; on `q35` the firmware opens it during POST and there
  is no firmware here to. An unwired pin is a permanently connected address
  line, which is the machine a kernel entered in protected mode wants. The same
  argument [`pc64`](pc64.md) makes.
* **No video**, so `x86.linuxboot` leaves `screen_info` zeroed and the 16550 is
  the console — which is what `console=ttyS0` names.

`nokaslr` is on the default command line for `pc64`'s reason exactly: the
decompressor's entropy loop spins on the 8254's null-count bit, which nothing
has ever cleared on a machine with no firmware to have programmed counter 0.
Measured on `q35`, a firmware clears it; measured here, there is no firmware.

## The interrupt the disk arrives on

Nothing is wired for it, and that is the point. The controller drives its own
`INTA#`; `src/bus/pci` rotates it by device number onto one of the bus's four
interrupt nets; the ICH9 collects those nets as `PIRQ[A-D]`; the board's
`pirq-routes` say where a POST would have steered them; and `q35.acpi`'s `_PRT`
is generated from the routers the bridge is holding. Device 4 with `INTA#`
swizzles to `PIRQ[A]`, which these routes put on IRQ11. `tests/q35_intx.rs`
runs that whole path on a smaller board.

## Two things the board had to grow, and what each was worth

### `sci-en`: an ACPI mode nothing could put the machine into

`q35.acpi` publishes an FADT with `SMI_CMD` zero, which ACPI §5.2.9 defines as
*this system does not support ACPI mode transitions* — correct, because there is
no SMI path on this board and no firmware to answer on one. An ICH9 also powers
up with `PM1_CNT.SCI_EN` clear (§13.8.3.3), meaning an enabled ACPI event would
raise `SMI#`. An operating system that reads both has been told two
contradictory things: there is no way into ACPI mode, and we are not in it.

`q35.lpc` therefore takes a **`sci-en`** property, the third of the same kind of
firmware stand-in `pm-base` and `pirq-routes` already are, and this board sets
it. The silicon's own reset value is still the default, so `machines/q35.machine`
is unchanged.

### `_CRS`: the difference between "enumerated" and "has a driver"

This was `q35.md`'s known gap 3 and it turned out to be the whole blocker. A
host bridge's `_CRS` is the list of windows it *produces* — the bus numbers, the
I/O ports and the physical addresses that belong to the bus below it. An
operating system allocates every base address register out of those windows, so
a bridge that declares none has declared that nothing downstream may be given an
address. The kernel said so:

```text
pci 0000:00:04.0: [1234:1122] type 00 class 0x010802
pci 0000:00:04.0: reg 0x10: [mem 0x00000000-0x00001fff 64bit]
pci_bus 0000:00: root bus resource [bus 00-ff]
...
pci 0000:00:04.0: BAR 0: no space for [mem size 0x00002000 64bit]
pci 0000:00:04.0: BAR 0: failed to assign [mem size 0x00002000 64bit]
```

The controller was enumerated and then had nowhere to live, so no driver ever
bound. `src/dev/q35/acpi.rs` now generates a `_CRS`, and every edge of it is
read out of the realized machine rather than written down — the same discipline
the rest of that file already had:

| Window | Where its edges come from |
| --- | --- |
| bus numbers | 0-255; everything is on bus 0 and there is no Type 1 header to divide the range with |
| I/O | 0-0xffff, with the hole where the 0xcf8 pair was found in the I/O space |
| memory | the megabyte above the top of RAM, up to the lowest of the APIC and HPET pages, with the ECAM window cut out of it |

and the ECAM window is then declared again on a `PNP0C02` motherboard device,
which is what a firmware does with it and what makes an operating system willing
to *use* the window rather than falling back to `0xcf8`. The kernel says both
halves out loud:

```text
PCI: MMCONFIG at [mem 0xe0000000-0xefffffff] reserved as ACPI motherboard resource
pci_bus 0000:00: root bus resource [io  0x0000-0x0cf7 window]
pci_bus 0000:00: root bus resource [io  0x0d00-0xffff window]
pci_bus 0000:00: root bus resource [mem 0x10100000-0xdfffffff window]
pci_bus 0000:00: root bus resource [mem 0xf0000000-0xfebfffff window]
pci_bus 0000:00: root bus resource [bus 00-ff]
pci 0000:00:04.0: BAR 0: assigned [mem 0x10100000-0x10101fff 64bit]
pci 0000:00:1f.0: quirk: [io  0x0600-0x067f] claimed by ICH6 ACPI/GPIO/TCO
```

That last line is the ICH9's ACPI block, at the address `pm-base` put it, being
recognised by name.

## What it was measured doing

Every line below is a byte the guest itself wrote to its own serial port,
running a stock distribution kernel — 12.7 MB, `6.6.67`, `SMP PREEMPT_DYNAMIC`
— that this project has never built, patched or read (`ROADMAP.md` §1).

```text
scripts/fetch-testdata.sh initramfs-x86

RSEMU_KERNEL=/boot/vmlinuz \
RSEMU_INITRD=testdata/x86/initramfs-x86.cpio \
RSEMU_KERNEL_CMDLINE='console=ttyS0,115200 nokaslr cryptomgr.notests nolapic' \
RSEMU_KERNEL_MS=2500000 \
RSEMU_KERNEL_INPUT='rsemu# =>cat /proc/partitions\n
nvme0n1=>head -c 40 /dev/nvme0n1\n' \
RSEMU_KERNEL_STOP_AT='LBA 0' \
    cargo test --release --features machine-q35-linux --test q35_linux -- --nocapture
```

**That command does not pass today**, and the section after this one says why:
it types at a shell prompt the run does not reach. It is written out in full
anyway, because it is the command that will pass when it does, and because
everything quoted below came out of it.

The board's own half of the boot, which is the part this page is about:

```text
[    0.000000] ACPI: RSDP 0x00000000000E0000 000024 (v02 RSEMU )
[    0.000000] ACPI: XSDT 0x00000000000E09E0 000044 (v01 RSEMU  RSEMUQ35 …)
[    0.000000] ACPI: FACP 0x00000000000E08E0 0000F4 (v03 RSEMU  RSEMUQ35 …)
[    0.000000] ACPI: DSDT 0x00000000000E0080 000772 (v02 RSEMU  RSEMUQ35 …)
[    0.000000] ACPI: APIC 0x00000000000E0800 00005A (v06 RSEMU  RSEMUQ35 …)
[    0.000000] ACPI: MCFG 0x00000000000E0860 00003C (v01 RSEMU  RSEMUQ35 …)
[    0.000000] ACPI: HPET 0x00000000000E08A0 000038 (v01 RSEMU  RSEMUQ35 …)
[    0.000000] ACPI: PM-Timer IO Port: 0x608
[    0.000000] ACPI: HPET id: 0x8086a201 base: 0xfed00000
[    0.000000] tsc: PIT calibration matches HPET. 1 loops
[    0.000000] ACPI: PCI Root Bridge [PCI0] (domain 0000 [bus 00-ff])
[    0.000000] PCI: Using ACPI for IRQ routing
[    0.000000] hpet0: at MMIO 0xfed00000, IRQs 2, 8, 0
[    0.000000] hpet0: 3 comparators, 64-bit 10.000000 MHz counter
[    0.000000] serial8250: ttyS0 at I/O 0x3f8 (irq = 4, base_baud = 115200) is a 16550A
[    0.000000] nvme nvme0: pci function 0000:00:04.0
[    0.000000] nvme 0000:00:04.0: enabling device (0000 -> 0002)
```

Eight claims about *this* work are in those lines and each is the board
answering rather than a constant: the RSDP found by a scan of a window nothing
staged, and six tables under it; `PM-Timer IO Port: 0x608` is `PMBASE` plus
ICH9 Table 13-11's offset; `HPET id` and the comparator count are a debug read
of the HPET's own capabilities register, travelling through the HPET table;
`PIT calibration matches HPET. 1 loops` is two independent clocks in this
machine agreeing with each other at the first attempt; the root bridge and its
IRQ routing are the DSDT; and the last two lines are the kernel's own NVMe
driver binding to a controller it found by class code.

The run above carries `nolapic`, so the kernel does not print the I/O APIC's
own lines. Without it, the MADT is read in full and the machine gets four lines
further before it panics — `IOAPIC[0]: apic_id 1, version 17, address
0xfec00000, GSI 0-23` and both of the interrupt source overrides the board's
wiring generates. The section after next has that log.

**Userspace was reached on this board, and is not reached today**, and the
difference between those two sentences is the whole of the next section — so it
is worth being exact about which run each claim comes from.

Before `_CRS` existed, the NVMe controller was enumerated and then given no
address, so no driver ever bound to it and nothing waited on one. That run
reached `Run /init as init process` at 341 virtual seconds and then a busybox
shell — `pc64`'s result reproduced on a much larger board, over a PCI bus, an
ACPI namespace and an APIC-capable interrupt tree.

With `_CRS` the driver *does* bind, and the boot now stops waiting for it:
`async_synchronize_full()` never returns, so `Freeing unused kernel image` and
`Run /init` do not happen. That is a strictly better place to be stuck — the
gap it exposes is one this board was built to find — but it is a stop, and the
line above would be a lie if it were left standing on its own.

## Where it stops, and what is in the way

**The disk is enumerated, given an address, and claimed by the kernel's own
driver — and the probe does not finish.** After
`nvme 0000:00:04.0: enabling device`, the machine idles: the guest sits in
`sti; hlt` inside `default_idle` and `async_synchronize_full()` never returns,
so `Freeing unused kernel image` and `Run /init` never happen. There is no
`/dev/nvme0n1` and nothing has been read off the medium.

Three things are in the way, in the order they matter, and **none of them is in
this board's own files** — all three are device models this board is the first
guest to exercise from a real driver.

1. **`CC.IOSQES`, `CC.IOCQES` and `CC.SHN` are one bit too high in
   `src/dev/nvme/ctrl.rs`, so the controller refuses the configuration every
   current kernel writes.** This is the whole of it, and it is four constants.

   The board's own post-mortem prints where the guest left the controller:

   ```text
   q35-linux: nvme 00:04.0 command=0x2000006 bar0=0x10100000 interrupt line=0x00 pin=0x01
   q35-linux:   cap=00400020_010103ff vs=0x10400 cc=0x00460001 csts=0x00000002 intms=0x00000000
   q35-linux:   aqa=0x001f001f asq=0x10074000 acq=0x10075000
   ```

   Everything there is what it should be except the last word of `csts`:
   `0x2` is `CSTS.CFS` — controller fatal status — with `CSTS.RDY` clear. The
   driver placed `BAR0`, enabled memory decode and bus mastering, built a
   32-entry admin queue pair in this board's RAM at two page-aligned addresses,
   and wrote `CC = 0x00460001`; the controller answered that it could not
   accept that.

   The *NVM Express* base specification's `CC` puts `MPS` at bits **10:07**,
   `AMS` at **13:11**, `SHN` at **15:14**, `IOSQES` at **19:16** and `IOCQES`
   at **23:20**. `ctrl.rs` has `CC_MPS_SHIFT` 7 over five bits,
   `CC_SHN_SHIFT` 15, `CC_IOSQES_SHIFT` 17 and `CC_IOCQES_SHIFT` 21 — every
   field from `MPS` up shifted by one — and `CC_MASK` `0x01ff_f8f1` where the
   specification's is `0x00ff_f8f1`.

   The guest's own value settles which layout is right. Read with the
   specification's positions, `0x00460001` is `IOSQES` 6 — 64-byte submission
   entries — and `IOCQES` 4 — 16-byte completion entries, which are the only
   two sizes NVMe defines. Read with `ctrl.rs`'s they are 3 and 2, and
   `Controller::enable`'s `iosqes == 6 && iocqes == 4` is false.

   `tests/nvme_board.rs` does not see it because it drives the register file
   from Rust and writes the same layout it reads.
   `the_controller_comes_ready_for_the_configuration_a_current_kernel_writes`
   in [`tests/q35_linux.rs`](../../tests/q35_linux.rs) reproduces it in five
   seconds and is `#[ignore]`d with this as its reason — un-ignore it with the
   shifts.
2. **A local APIC timer in periodic mode delivers nothing.** With the board's
   default command line the kernel programs the local APIC timer as the tick
   and then stops dead immediately after
   `APIC delta adjusted to PM-Timer: 624999 (600000)`. `tests/pc_apic.rs`
   exercises the APIC timer as a **one-shot** only.
3. **An edge-triggered I/O APIC redirection entry delivers nothing.** In
   symmetric I/O mode the kernel routes IRQ0 to I/O APIC input 2 — the MADT's
   own interrupt source override — and finds no tick:

   ```text
   ..TIMER: vector=0x30 apic1=0 pin1=2 apic2=-1 pin2=-1
   ..MP-BIOS bug: 8254 timer not connected to IO-APIC
   ...trying to set up timer (IRQ0) through the 8259A ...
   ..... (found apic 0 pin 2) ...
   ....... failed.
   ...trying to set up timer as Virtual Wire IRQ...
   ..... failed.
   ...trying to set up timer as ExtINT IRQ...
   ..... failed :(.
   Kernel panic - not syncing: IO-APIC + timer doesn't work!
   ```

   `tests/pc_apic.rs`'s only I/O APIC delivery is **level-triggered** (its
   redirection entry sets bit 15); an ISA source through a `_PRT` or an
   interrupt source override is edge-triggered, and that is the case with no
   coverage.

(2) and (3) are why every measurement above was taken with **`nolapic`** on the
command line, which leaves the machine in PIC mode — the 8259A driving `INTR`
through the IMCR, exactly as `pc64` runs. `nolapic` is deliberately **not** in
the machine file's default `cmdline`: the board really does have both APICs and
really does publish a MADT describing them, and a default that hid two device
defects would put them somewhere nobody looks. When they are fixed the default
command line starts working with no change here.

## Sources

[`q35.md`](q35.md)'s and [`pc64.md`](pc64.md)'s, plus:

* *ACPI Specification* revision **6.5** — §5.2.9 (`SMI_CMD`), §6.2.13 (`_PRT`),
  §6.4.2.9 and §6.4.3.5 (the resource descriptors `_CRS` is made of), §9.15
  (`PNP0C02`).
* *NVM Express* base specification for the controller.

**No emulator source and no firmware source was consulted** (`ROADMAP.md` §1).
The kernel image and the busybox binary are *data*: they are loaded and run,
never read as source, and never vendored.
