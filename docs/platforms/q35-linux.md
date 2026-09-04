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

## Three things the board had to grow, and what each was worth

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

### The legacy replacement route: a gate between three chips

`pc.hpet` advertises `LEG_RT_CAP`, so `hpet_enable()` takes it: it registers the
HPET as the global clock event, sets `LEG_RT_CNF`, and — this is the half that
matters — **never calls `pit_timer_init()`**, because `hpet_time_init()` programs
the 8254 only when `hpet_enable()` fails. From that point the machine's tick is
supposed to arrive from HPET comparator 0 on IRQ0, which is the 8259A's `IR0`
and I/O APIC input 2, with the 8254 disconnected (*IA-PC HPET Specification*
rev 1.0a §2.3.5).

This board wired `hpet0.t0` to `ioapic.irq16` and left `pc.hpet`'s `legacy`
output pin unconnected, so while that bit was set **nothing drove IRQ0 at all**
— counter 0 of the 8254 had never been loaded, and comparator 0 was on a
different input. The kernel's complaint was therefore literally accurate:

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

Every one of those three fallbacks is another way of asking for IRQ0, so all
three had to fail. Adding `hpet=disable` proved it: Linux then falls back to
`pit_timer_init`, programs the 8254 itself, and the same run gets through
`..TIMER: vector=0x30 apic1=0 pin1=2` with no `MP-BIOS bug` line.

`LEG_RT_CNF` is **not a register in any of the three chips it moves**. It is a
multiplexer on the board, and a device may not reach across a pin and
disconnect a sibling — so `pc.hpet` reports the bit on a `legacy` output pin
(announced from state on the realize sweep and after a restore, dropped by
reset) and the machine file builds the multiplexer out of the combinators
`ROADMAP.md` §4.3 ships:

| `LEG_RT_CNF` | IRQ0 / I/O APIC input 2 | IRQ8 / I/O APIC input 8 |
| --- | --- | --- |
| clear | the 8254's counter 0 | the RTC's periodic interrupt |
| set | the HPET's comparator 0 | the HPET's comparator 1 |

and while it is set, comparators 0 and 1 stop driving the inputs their
`Tn_INT_ROUTE_CNF` names — which is why `machines/q35-linux.machine` has one
`wire.not` and *five* `wire.and`s and not three. Two sources on one net is
§4.3's wired-OR and it is the right shape: at most one side of a multiplexer is
ever enabled, so the resolve never has to arbitrate.

Nothing in `src/dev` changed for this, and that is the point worth keeping.
The pin was already there and `tests/wire_combinators.rs` already drove it
through a gate on a toy board
(`an_hpets_legacy_route_gates_a_timer_off_its_line`); what was missing was six
objects and a wire block in one machine file.

`the_legacy_replacement_route_moves_irq0_from_the_8254_to_the_hpet` in
[`tests/q35_linux.rs`](../../tests/q35_linux.rs) asks the shipped board the same
question hermetically, in five seconds rather than five minutes, so that the
wiring cannot be deleted quietly: it programs the 8254's counter 0 and watches
the master 8259A's request register, which latches whether or not the line is
masked, then sets the bit and asserts that the 8254 stops reaching IRQ0 and
comparator 0 starts. Checked against the machine file as it was before the
gate, the middle assertion fails — which is the negative control that says the
test is testing the board rather than agreeing with itself.

## What it was measured doing

Every line below is a byte the guest itself wrote to its own serial port,
running a stock distribution kernel — 12.7 MB, `6.6.67`, `SMP PREEMPT_DYNAMIC`
— that this project has never built, patched or read (`ROADMAP.md` §1).

```text
scripts/fetch-testdata.sh initramfs-x86

RSEMU_KERNEL=/boot/vmlinuz \
RSEMU_INITRD=testdata/x86/initramfs-x86.cpio \
RSEMU_KERNEL_MS=3000000 \
RSEMU_KERNEL_INPUT='rsemu# =>head -c 40 /dev/nvme0n1\n' \
RSEMU_KERNEL_STOP_AT='LBA 0' \
    cargo test --release --features machine-q35-linux --test q35_linux -- --nocapture
```

**There is no `RSEMU_KERNEL_CMDLINE` on that command**, and that is the single
biggest change on this page. The board's own default line —
`console=ttyS0,115200 earlyprintk=ttyS0,115200 nokaslr` — is what the kernel
gets, and it carries none of the three words this page used to need:

| Word | What it was hiding | What happened to it |
| --- | --- | --- |
| `nolapic` | an acknowledge cycle answered by an idle 8259A instead of the local APIC that was actually asserting | fixed in `src/dev/pc` |
| `noapic` | the missing legacy replacement gate, filed against the I/O APIC | fixed in this board's machine file, above |
| `hpet=disable` | the same gate, from the other side | likewise |

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
[    0.000000] IOAPIC[0]: apic_id 1, version 17, address 0xfec00000, GSI 0-23
[    0.000000] ACPI: INT_SRC_OVR (bus 0 bus_irq 0 global_irq 2 dfl dfl)
[    0.000000] ACPI: INT_SRC_OVR (bus 0 bus_irq 9 global_irq 9 high level)
[    0.000000] ACPI: HPET id: 0x8086a201 base: 0xfed00000
[    0.000000] APIC: Switch to symmetric I/O mode setup
[    0.029999] ..TIMER: vector=0x30 apic1=0 pin1=2 apic2=-1 pin2=-1
[    0.049999] tsc: PIT calibration matches HPET. 1 loops
[  139.044607] PCI: Using ACPI for IRQ routing
[  141.657744] hpet0: at MMIO 0xfed00000, IRQs 2, 8, 0
[  141.677162] hpet0: 3 comparators, 64-bit 10.000000 MHz counter
[  306.737476] serial8250: ttyS0 at I/O 0x3f8 (irq = 4, base_baud = 115200) is a 16550A
[  319.301970] nvme nvme0: pci function 0000:00:04.0
[  319.317778] nvme 0000:00:04.0: enabling device (0000 -> 0002)
[  322.201066] rtc_cmos rtc_cmos: setting system clock to 2026-01-01T00:12:23 UTC
[  322.237990] nvme nvme0: 1/0/0 default/read/poll queues
[ 2076.993539] Freeing unused kernel image (initmem) memory: 3912K
[ 2374.773912] Run /init as init process
```

Each of those is the board answering rather than a constant: the RSDP found by
a scan of a window nothing staged, and six tables under it; `PM-Timer IO Port:
0x608` is `PMBASE` plus ICH9 Table 13-11's offset; `HPET id` and the comparator
count are a debug read of the HPET's own capabilities register travelling
through the HPET table; both interrupt source overrides are generated from the
board's own wiring, and the first of them is IRQ0's offset to global system
interrupt 2; `PIT calibration matches HPET` is two independent clocks in this
machine agreeing at the first attempt; the root bridge and its IRQ routing are
the DSDT; `setting system clock to 2026-01-01T00:12:23` is the board's `time`
parameter plus twelve minutes of its own emulated wall clock, read back out of
the RTC by a driver; and the four lines after `Using ACPI for IRQ routing` are
the kernel's own NVMe driver binding to a controller it found by class code,
taking its **level-triggered interrupt through the I/O APIC**, and building the
queue it reads a disk with.

**The two lines that matter most are the ones that are not there.** After
`..TIMER: vector=0x30 apic1=0 pin1=2` there is no `..MP-BIOS bug: 8254 timer
not connected to IO-APIC` and no `Kernel panic - not syncing: IO-APIC + timer
doesn't work!`. `check_timer()` found its tick on I/O APIC input 2, delivered
by HPET comparator 0 through the gate, and went on.

`hpet0: at MMIO 0xfed00000, IRQs 2, 8, 0` is the same fact from the kernel's
side: with `LEG_RT_CNF` set it expects comparator 0 on 2 and comparator 1 on 8,
and that is exactly what the multiplexer now puts there.

### What `noapic` was for, and why it is no longer a second mode

This page used to carry a second run here: the same kernel, the same board and
the same controller with `noapic` added, which reached userspace while the
default line did not.

```text
RSEMU_KERNEL_CMDLINE='console=ttyS0,115200 nokaslr cryptomgr.notests noapic'
```

That run is worth keeping in the record, because **it is what located the last
defect**. With the I/O APIC out of the picture — the 8259A pair driving `INTR`
through the IMCR — the NVMe probe finished and busybox came up, which said the
gap was neither the controller model nor the driver nor a machine merely too
slow. The 8259A has no polarity bit; the I/O APIC has one, and was applying it.
The next section has the rest.

It is no longer a *mode*. The default line reaches the same place:

```text
[ 2374.773912] Run /init as init process

rsemu initramfs on Linux 6.6.67-gentoo-x86_64 x86_64
rsemu# head -c 40 /dev/nvme0n1
rsemu q35-linux nvme namespace, LBA 0
```

That last line is **forty bytes off the medium**, read by busybox through a
block device the kernel's own driver created. It is the board's whole stack
answering at once: PCI enumeration, an MSI-less **level-triggered interrupt
through the I/O APIC**, an admin queue pair, an I/O queue pair, `Identify` for
the controller and for the namespace, and a read command whose data landed
where the PRP list said. Nothing here is a constant in a test.

## Where it stopped, and what was in the way

**Nothing now.** This page has named four obstacles over its life and all four
are gone: two were refuted rather than fixed (their real causes were the 8259A
acknowledge defect and the missing HPET route), one was a controller defect,
and the last was a single bit in an I/O APIC redirection entry.

### Fixed: the namespace is published

This section used to say `/dev/nvme0n1` did not exist because the model
refused `Identify` with `CNS = 03h`, the Namespace Identification Descriptor
list, with status `0x2` *Invalid Field in Command*.

It was not alone. The controller reports `VS = 1.4.0`, and NVM Express 1.4
§5.15.2.2 also makes `SUBNQN` mandatory for any controller claiming 1.2.1 or
later — that field read as zeroes, and the kernel said so out loud
(`missing or invalid SUBNQN field`). One claim, two fields not backed by it.
Both are answered now (`src/dev/nvme/ctrl.rs`), the namespace carries a UUID
derived from the identity the controller already publishes, and the run above
is the result.

### Fixed: the polarity bit an I/O APIC must not apply

On the default command line the boot used to reach
`nvme 0000:00:04.0: enabling device` and stop. Sampled every 28 virtual seconds
for the rest of the run, the guest was always at the same instruction:

```text
watchdog: BUG: soft lockup - CPU#0 stuck for 678s! [kworker/u2:0:11]
 <TASK>
 RIP: 0010:_raw_spin_unlock_irqrestore+0x1c/0x30
 __setup_irq+0x450/0x690
 request_threaded_irq+0xe4/0x170
 ? __pfx_nvme_irq+0x10/0x10
 pci_request_irq+0xa5/0xf0
 queue_request_irq+0x6f/0x80
 nvme_pci_enable+0x40a/0x570
 nvme_probe+0x33d/0x8a0
```

`_raw_spin_unlock_irqrestore+0x1c` is the `sti` at the end of `__setup_irq`'s
critical section. **`noapic` was the control**: the same board, the same image
and the same controller got through `request_threaded_irq` and on to
`Run /init` with the I/O APIC out of the picture, which is what made this a
finding rather than a guess.

`tests/q35_linux.rs`'s `report_apics` is what settled it — the redirection
table and the local APIC's request registers, read out of the machine with
`MemAttrs::DEBUG` after the run:

```text
q35-linux:   irq9  01000000_00008820 vector=0x20 level high open
q35-linux:   irq11 01000000_0000e822 vector=0x22 level low  open remote-irr
q35-linux:   isr=00000000_..._00000000
q35-linux:   irr=00000000_..._00000004_00000000
```

Input 11 is where this board's `pirq-routes` put device 4's `INTA#`. The entry
is level-triggered, **active low**, unmasked, holding remote IRR, and the local
APIC has its vector requested with nothing in service. That is an interrupt
storm caught mid-cycle, and it starts the instant `irq_startup()` unmasks the
entry.

The bit is the whole defect. An operating system programs *every* PCI interrupt
level-triggered and active low, because PCI Local Bus 3.0 §2.2.6 defines
`INTA#`-`INTD#` that way; and `src/dev/pc/ioapic.rs` was exclusive-oring that
polarity bit into the level its input net resolved to (82093AA §3.2.4 bit 13,
`INTPOL`). But a `core::wire` net carries an **assertion**, not a voltage: a
fresh fan-in holds every source low, `Resolve::Or` resolves an idle net low,
and `ROADMAP.md` §4.3 states outright that an undriven wire sits low. Every
driver in the tree agrees — `q35.lpc` reads `PIRQ[n]#`, an active-low pin on
the silicon, as asserted when its net is *high*.

So the exclusive-or did not model an active-low input; it made one impossible.
The idle line read as asserted, the entry latched remote IRR and sent, the
processor took the vector and ended it, the end-of-interrupt cleared remote IRR,
the condition was still true by inspection, and it sent again — for ever. The
`sti` that let the first one in was the last instruction that guest ever nearly
retired. The 8259A has no polarity bit, which is exactly why `noapic` booted.

The polarity bit is now recorded and reported and not applied; the same is true
of bit 13 of a local APIC's LINT entries, where an idle `ExtINT` LINT0 reading
as asserted would have been the same failure with an 8259A on the far side. A
board that genuinely needs an inverted input says so once, with the `wire.not`
§4.3 ships — which is where an inversion belongs, on the board rather than in
every part that has a polarity bit.

The entry after the fix, from the run at the top of this page:

```text
q35-linux:   irq11 01000000_0000a822 vector=0x22 level low  open
```

Same entry, same polarity bit reading back, no remote IRR — and a shell.

Three tests hold it. `tests/pc_apic.rs`'s
`a_level_entry_programmed_active_low_waits_for_its_device` drives a real guest
through the whole lifecycle and counts interrupts: **zero** before its device
asserts and **one** after, where the old code counted 64 and 441.
`tests/q35_linux.rs` writes the entry the kernel wrote, on the shipped board,
with the controller idle, and asserts the local APIC stays quiet. And
`src/dev/pc/ioapic.rs` has the unit test that used to assert the opposite.

### What the soft lockups are, and are not

They are **not** the defect, and they appear on both command lines throughout
the boot — one of them lands in `ptdump_walk_pgd`, which is `CONFIG_DEBUG_WX`
walking the kernel page tables and is simply a lot of work. The guest believes
it is a `24.899 MHz` processor (`osc cpu` is 100 MHz; the TSC counts charged
cycles), and its own tick keeps running while the interpreter grinds, so any
stretch of expensive kernel work reads as a stall to the watchdog. Reading one
of these as a hang has already cost this page one wrong diagnosis; the test for
"stuck" is whether the sampled `RIP` moves.

### The ledger this page used to carry, and what became of it

Three items were filed here. **Two of them were wrong**, and both were wrong in
the same way: a delivery path was blamed for a route that did not exist.

1. **`CC.IOSQES`, `CC.IOCQES` and `CC.SHN` one bit too high in
   `src/dev/nvme/ctrl.rs`.** Correct, and **fixed**. The specification puts
   `MPS` at 10:07, `AMS` at 13:11, `SHN` at 15:14, `IOSQES` at 19:16 and
   `IOCQES` at 23:20; every field from `MPS` up had been shifted by one, so the
   controller answered `CSTS.CFS` to the `CC = 0x00460001` every current kernel
   writes. The board's post-mortem now prints `csts=0x00000001` — `RDY` — for
   the same guest write.
2. **"A local APIC timer in periodic mode delivers nothing."** **Refuted.** The
   timer delivers; `tests/pc_apic.rs` covers periodic mode now and it passes.
   The defect the symptom belonged to was an **acknowledge cycle**: an idle
   8259A answered an interrupt acknowledge that belonged to the local APIC that
   was actually asserting, so the vector the processor took was the 8259A's
   spurious one.
3. **"An edge-triggered I/O APIC redirection entry delivers nothing."**
   **Refuted.** Edge-triggered entries deliver, including the 8254 on input 2,
   and `tests/pc_apic.rs` drives that case. What was missing was the legacy
   replacement gate, three sections up: nothing was driving IRQ0 at all, so
   there was no interrupt for a working redirection entry to deliver.

The lesson is worth keeping with the ledger. Both refuted items were filed from
a *panic message* rather than from a probe of the path, and both blamed a
device model for something the board file never wired. `tests/pc_apic.rs`
exists partly because of that: it drives each delivery path from Rust, on a
board whose wiring is written down beside it, so "the device does not deliver"
and "the board does not route" stop being the same observation.

The fourth item — the polarity bit — is the one that was filed *from a probe*,
and it is the shortest entry on this page for that reason. `report_apics` read
the redirection table and the local APIC's request registers out of the stopped
machine, one line said `level low ... remote-irr` with the vector requested and
nothing in service, and that was the diagnosis. Nothing was inferred from a
backtrace. It is also worth noting what the backtrace *would* have said: the
frame the earlier reading leaned on, `nvme_wait_ready`, is `?`-prefixed — an
unreliable leftover stack word, not a live frame — and this controller sets
`CSTS.RDY` synchronously and never polls.

## The same board under KVM

The run above is the *interpreted* one: 2,826 seconds of guest time and about
sixteen minutes of wall clock. `tests/kvm_q35_linux.rs` is the same machine
file, the same kernel, the same initramfs and the same namespace signature,
with the engine underneath `cpu0` replaced by a KVM vCPU
(`Bindings::replace`, `src/accel/cpu.rs`), and it reaches the same line —
`rsemu q35-linux nvme namespace, LBA 0`, read off the emulated controller by
the kernel's own driver — in **about three seconds**. Roughly three hundredfold
on the clock a person watches, **on this board's own command line**: nothing is
added to it.

Guest time and wall clock are the same three seconds, and that is the point
rather than a coincidence. The accelerated run is in `ThreadingMode::Accel`,
where the scheduler reads a round's elapsed virtual time off the host clock
instead of taking it from what the runnables claimed — `ROADMAP.md` §4.2's
*"virtual time is slaved to the host clock"*. See
[below](#the-word-the-command-line-no-longer-needs) for the two kernel checks
that used to make this impossible.

```text
RSEMU_KERNEL=/boot/vmlinuz \
RSEMU_INITRD=testdata/x86/initramfs-x86.cpio \
RSEMU_KERNEL_INPUT='rsemu# =>head -c 40 /dev/nvme0n1\n' \
RSEMU_KERNEL_STOP_AT='LBA 0' \
    cargo test --release --features accel-kvm,machine-q35-linux \
               --test kvm_q35_linux -- --nocapture
```

Nothing about the board changes. The memory map, the ACPI tables, the PCI
enumeration, the `PIRQ` swizzle, the I/O APIC redirection entry and the NVMe
controller are the same objects doing the same work; what changes is who
executes the guest's instructions between two of their accesses. About 132,000
of those accesses leave hardware over a whole boot — 69,000 port accesses and
63,000 MMIO — and every one is served by this board's own device models.

The two runs were compared line for line, on *literally* the same command line
now that `no_timer_check` is gone: **282 of the accelerated run's 346 console
lines are byte-identical to the interpreted run's**, in the same order, and
every milestone from the RSDP scan to `rsemu q35-linux nvme namespace, LBA 0`
appears in both. The 62 that differ are all downstream of *who the processor
is* — the model line, the speculative-execution mitigations, the `XSAVE` list,
the PMU, the TLB geometry, the BogoMIPS a correct calibration produces —
because an accelerated `cpu.x86` reports the host's silicon rather than the
`variant` this file declares. Not one of them is a device answering
differently, and none of them is a timekeeping failure. (The interpreted run
also prints five `soft lockup` backtraces and the 636 lines of stack that go
with them; that is its own artefact, the guest's watchdog noticing that its
interpreter is slow.)

Two things had to exist first, and both are in `src/accel/`:

* **a `CPUID` table.** A vCPU that has never been given one answers every leaf
  with zeros, and this kernel checks the long-mode bit before it does anything
  else. `KVM_GET_SUPPORTED_CPUID` filtered through `accel::kvm::board_cpuid`
  is the processor's identity, and the filter is where the board gets a say:
  x2APIC and the TSC-deadline timer come *out*, because `pc.lapic` implements
  neither, and so do the host's paravirtual leaves, because a guest taking its
  time from `kvmclock` is a guest reading the host's clock while every other
  clock on this board is virtual time.
* **an interpreter for the reset vector.** A hypervisor cannot fetch through an
  MMIO exit, and this board's reset vector is a *device region* — sixteen
  synthesised bytes of `x86.linuxboot` at `0xfffffff0`, because what lives
  there is a loader rather than a ROM image. The far jump out of it runs on the
  shell interpreter that every accelerated `cpu.x86` carries; everything after
  it runs on silicon. One instruction, measured.

### The word the command line no longer needs

`no_timer_check` used to be on it, and it was a statement about the *scheduler*
rather than about this board:

> Virtual time did not advance while a vCPU was inside `KVM_RUN`.

A scheduler round ends when every runnable returns, and an accelerated
processor returns when the guest exits — so a guest that ran without exiting
held the round, and the board's clocks stood still for as long as it took. A
delay loop is exactly such a guest. Two of this kernel's checks were that one
fact:

| what the kernel printed | what it did |
| --- | --- |
| `hpet: Counter not counting. HPET disabled` | read `HPET_COUNTER`, spun 200,000 TSC cycles, read it again — both reads inside one round |
| `..MP-BIOS bug: 8254 timer not connected to IO-APIC`, then a panic | `timer_irq_works()`: read `jiffies`, spin ~40 ms, read `jiffies` — the delay loop takes no exits, so no tick arrived |

Neither was a defect in the interrupt tree this page spent four obstacles
getting right. Two changes removed both, and only one of them is in `accel/`:

* **`ThreadingMode::Accel`, in `src/core/sched.rs`.** A round's elapsed virtual
  time is read off the injected `HostClock` rather than derived from what the
  runnables reported — an accelerated core can only report its whole budget, so
  deriving it made the board's clocks run at a rate set by the quantum. And
  because this engine's slice under that mode is **one guest exit long**, every
  access the guest makes to a device sees the wall as of that access:
  `hpet_counting()` now reads a counter that has moved.
* **A preemption interval, in `src/accel/preempt.rs`.** Slaving time to the wall
  is not enough on its own — a guest spinning in `RDTSC` and `PAUSE` takes no
  exits at all, so the round cannot end and the tick that is *due* cannot be
  delivered. The vCPU's own thread asks the kernel for a periodic signal, whose
  delivery is what makes `KVM_RUN` return `EINTR`; that module works through
  `immediate_exit`, the safe-point exit flag, an `ioctl` from another thread and
  VMX's notify window, and why none of them can do it.

What the kernel says now is the measurement that matters, because a guest's own
view of the clock is the honest test of whether the clock is right:

```text
[    0.033333] tsc: using HPET reference calibration
[    0.036666] tsc: Detected 3992.942 MHz processor
[    0.736685] hpet0: at MMIO 0xfed00000, IRQs 2, 8, 0
[    0.737097] hpet0: 3 comparators, 64-bit 10.000000 MHz counter
```

3,992.942 MHz against a host that is 3,993,994 kHz — the last digits move run
to run, because it is a measurement. Before this it reported a
`176273.643 MHz processor` — it was measuring a real time-stamp counter against
a board whose clocks only moved when it stopped running, so every delay it
computed was wrong by about forty-four times. `src/accel/mod.rs` has the full
list of what acceleration still costs.

## Sources

[`q35.md`](q35.md)'s and [`pc64.md`](pc64.md)'s, plus:

* *ACPI Specification* revision **6.5** — §5.2.9 (`SMI_CMD`), §6.2.13 (`_PRT`),
  §6.4.2.9 and §6.4.3.5 (the resource descriptors `_CRS` is made of), §9.15
  (`PNP0C02`).
* *NVM Express* base specification for the controller.

**No emulator source and no firmware source was consulted** (`ROADMAP.md` §1).
The kernel image and the busybox binary are *data*: they are loaded and run,
never read as source, and never vendored.
