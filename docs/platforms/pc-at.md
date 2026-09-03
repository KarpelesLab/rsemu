# `pc-at` — the machine rsemu builds, and the firmware it does not ship

Consumed by: `machines/pc-at.machine` and `src/dev/pc`. The source *register*
for the platform is [`ibm-pc.md`](ibm-pc.md); this page is about the board rsemu
actually assembles and what is known to be missing from it.

## The firmware is the user's — and there is now one of our own to fall back on

rsemu ships **no third-party** BIOS and no third-party video BIOS, and will not.
A PC's firmware is somebody else's copyrighted binary; **running one as an
emulated guest is ordinary use whatever its licence, and redistributing it is
not** (`../../ROADMAP.md` §1). So the machine takes both through media slots,
exactly as `riscv-virt` takes its firmware:

```console
$ rsemu run pc-at --hd0 disk.img                   # rsemu's own BIOS
$ rsemu run pc-at --bios /usr/share/qemu/bios.bin  # or the user's
$ rsemu run pc-at --bios bios.bin --media vgabios=vgabios.bin --media floppy=boot.img
```

What changed in the first line is that [`src/fw/pcbios`](../../src/fw/pcbios) is
a **minimal legacy BIOS written here**, and the `bios` slot defaults to it — see
[the section below](#the-in-house-bios). That is a default, not a replacement:
`--bios` still wins, and every other firmware this board might run still comes
from the user.

**A third-party video BIOS has to be a PCI one.** A 440FX-era firmware finds video by
enumerating the bus for class code `030000` and taking that card's option ROM off
its expansion ROM base address register — and it validates the **PCI Data
Structure** inside the image against the card's vendor and device id before it
runs it. A video BIOS built for an ISA card carries no such structure (its
pointer at offset `0x18` is zero) and no amount of chipset will make one load
this way. On a machine with QEMU's images installed that means
`vgabios-stdvga.bin`, not `vgabios.bin` — the latter is the ISA build. If the
ids do not match, the firmware finds the card, maps the ROM, and silently
declines to run it; measured, both ways, in the section below.

`--bios` and `--vgabios` are conveniences over `--media bios=…`, and so are
`--hd0` and `--hd1`, which fill the two drive bays on the primary IDE channel;
the mechanism is the media table and nothing else. An unbound `hd0` or `hd1` is
an **empty bay**, not an error — a PC with no hard disk is an ordinary PC, and a
drive bound that way costs its whole capacity in host memory the moment it
exists, because the media table is bytes.

A build with `dev-blk` has the other option, which is the one you want for a
disk of any size:

```console
$ rsemu run pc-at --bios bios.bin --drive hd0=disk.qcow2
$ rsemu run pc-at --bios bios.bin --drive hd0=fresh.qcow2,new=8G
```

`--drive` backs the same media slot with the **file** rather than with a copy of
its bytes: the capacity comes from the image, the guest's writes go to the file,
and sparse raw, qcow2, DMG, DiskCopy 4.2 and LUKS are all understood (through
`fstool` — no image format is parsed in rsemu). Nothing in
`machines/pc-at.machine` changes, because the machine file names a media slot
and the *run* decides what is behind that name. A machine snapshot then
**references** the image rather than copying it; `docs/buses/storage.md` has the
argument. `pc.rom` is the socket the ROMs land in, and
its `align` property is the one thing about it that is not obvious: a **system
BIOS is top-aligned**, because an x86 fetches its first instruction from the top
of its address space, and an **option ROM is bottom-aligned**, because firmware
finds one by scanning upward for `0x55 0xaa`.

Note for anyone reaching for SeaBIOS: it is **LGPL**. Run the binary, never read
the source. Same for the Bochs BIOS. EDK II / OVMF is BSD-2-Clause-Patent and
may be read.

## What the board is

An IBM PC/AT, wired the way the 1984 Technical Reference wires it, described
entirely in `machines/pc-at.machine`. Nothing in `src/dev/pc` knows a PC
address; every one of them is written once, in that file.

| Component | Class | Where | Specification |
| --- | --- | --- | --- |
| CPU | `cpu.x86` (80486) | — | [`../cpu/x86.md`](../cpu/x86.md) |
| Interrupt controllers (2, cascaded) | `pc.pic` | 0x20, 0xa0, ELCR at 0x4d0/1 | Intel 8259A data sheet |
| Timer | `pc.pit` | 0x40-0x43 | Intel 8254 data sheet |
| Keyboard controller, A20, reset | `pc.kbc` | 0x60, 0x64 | Intel 8042 data sheet |
| System control ports | `pc.sysctl` | 0x61, 0x92, 0xcf9 | AT Technical Reference, Intel chipset data sheets |
| RTC and CMOS | `pc.rtc` | 0x70-0x71 | Motorola MC146818 data sheet |
| DMA (2, byte and word) | `pc.dma` | 0x00-0x0f, 0xc0-0xdf, pages at 0x80 | Intel 8237A data sheet |
| Display | `pc.video` | 0x3b4/0x3d4, 0x3c0-0x3cf, 0x3da | MC6845 data sheet, VGA register set |
| Floppy controller | `pc.fdc` | 0x3f0-0x3f5, 0x3f7 | NEC µPD765A data sheet |
| IDE channels (2) | `pc.ide` | 0x1f0-0x1f7 + 0x3f6, 0x170-0x177 + 0x376 | AT Technical Reference, fixed-disk adapter |
| Hard disks (2 bays) | `ata.disk` | — (on the cable) | T13 ATA/ATAPI-6 |
| Firmware sockets | `pc.rom` | 0xc0000, 0xe0000 (+ a high alias) | — |
| PCI host bridge, and RAM shadowing | `pc.pmc` | 0xcf8-0xcff | Intel 82441FX data sheet |
| PCI display adapter, and its video BIOS | `pc.vga-pci` | 00:02.0, expansion ROM BAR | PCI Local Bus Spec Rev 2.1 §6.2.5.2 |

Six oscillators, because the board has six cans: the CPU clock, the 8254's
105/88 MHz — not an integer number of hertz, which is why the description
language takes rational frequency literals — the RTC's 32.768 kHz, the VGA dot
clock, the 8042's own crystal, and the floppy data separator. The IDE channels
have none: a drive's controller is on the drive, and this model's is zero-time,
so there is nothing on this side of the cable for a clock to pace. Only ratios
*within* a tree are
guest-visible (`../../ROADMAP.md` §4.2).

The system ROM is mapped **twice**, at `0xe0000` and again just below 4 GiB. A
386 fetches its first instruction from `0xfffffff0` and only reaches the low
window after firmware's first far jump; both windows are one chip.

## Two things the core grew for this board

Both are generic mechanisms in `core::wire`, not PC special cases
(`../../ROADMAP.md` §0, "generic first, specific second"):

- **`IntAck`** — the reverse half of a vectored interrupt. A wire carries a
  level; an acknowledge cycle carries a *vector* back the other way. Without it
  an 8259A never learns its request was taken, so it cannot move it from pending
  to in-service and end-of-interrupt has nothing to clear. It is also how the
  master asks the *slave* for the vector during the second INTA pulse, since the
  slave's `INT` and the master's `IR2` are one net — the same cycle, handed on.
  The cycle carries what the processor presents while it asks (nothing, on an
  8086; the interrupt level, on a 68000) and a controller may decline one that
  is not its own, which is what lets several controllers share one processor.
- **`DmaPeripheral`** — the data half of a DMA request line. `DRQ` carries a
  level, but the transfer it asks for moves bytes over `DACK` and `IOR`/`IOW`.
  Without it an 8237 can be programmed and can never transfer.

Both attach along a net: the driver offers, the sink is handed a `Weak`. They
exist because `BindCtx` cannot reach a sibling device's handle — see below.

## A20 is **open** at power-on, and it took the reset vector to prove it

The A20 gate exists so that an 8086's megabyte wrap survives, and it is easy to
read that as "the gate starts shut". It does not, and the board itself is the
proof: a 286 fetches its first instruction from `0xfffff0` and a 386 or 486
from `0xfffffff0`, and **bit 20 is set in both**. A gate closed at power-on
would turn the first into `0xeffff0` and the second into `0xffeffff0`, neither
of which an AT decodes as ROM. No such machine could reach its own reset
vector.

What actually happens is in the 8042's data sheet: `RESET` puts ports 1 and 2
into the input mode, and P2 is quasi-bidirectional with pull-ups, so every
output-port line comes up high. Bit 0 is the system reset line, active low, so
high means *not resetting*; bit 1 is the A20 gate, so high means *open*.
Firmware closes it during POST — which is exactly why an AT BIOS contains a
step that does so, and would not need one if the hardware handed it over shut.

Two things follow for rsemu, and both were bugs:

- `pc.kbc`'s output port comes out of reset as `OP_RESET | OP_A20`, not
  `OP_RESET` alone. One latch, one reset, one power-up state.
- The x86 core does **not** touch its A20 mask on reset. The gate is chipset
  logic, not a processor feature; a `RESET` on the processor does not move it.
  Driving it shut from there was inventing a level for an input pin, and no
  driver could correct it, because a source re-announcing the level it already
  sits at is not a change and `Wire::set` delivers changes. The board came up
  with its 8042 holding the gate open and the core masking bit 20 anyway.

## What a real firmware binary asks for

Measured black-box against one, two ways: scanning it for the immediate operands
of its I/O instructions, and reading its diagnostic strings. Both are mechanical
observations of an opaque file — the same class of thing as watching which
addresses a program puts on a bus, or reading what a program prints. **No source
was read**, and none may be: the common x86 firmwares are LGPL.

The ports it touches, in descending order of how often: the **RTC and CMOS** by a
wide margin, then the **8042**, the **8259A** pair, the **8254**, the **8237**
pair with their page latches, port **0x92**, the **ELCR** at 0x4d0/0x4d1, the
floppy controller at 0x3f0-0x3f7, PCI configuration at 0xcf8/0xcfc, the reset
control port at 0xcf9, and a paravirtual configuration channel at 0x510/0x511.

Its own error messages say which of those it can do without. It reports finding
that channel rather than requiring it; it has a path for "no PCI VGA devices
found" and one for "no APIC"; it scans 0xc0000 for a **legacy option ROM**,
which is exactly what this board offers; and it warns rather than stops when it
cannot find a host bridge to unlock RAM for shadowing — which it no longer has
to, since the board grew one. Three of its checks are
against this board's 8042 — a self test that must answer 0x55, an interface test
that must answer 0x00, and a keyboard self test that must answer 0xaa — and
`pc.kbc` answers all three.

So the missing pieces below are ranked by that evidence, not by guesswork.

## How far a real firmware image gets

Measured, not asserted: `tests/pc_at_firmware.rs` runs the user's own image on
the assembled board and prints what it observed. It is gated on `RSEMU_BIOS`
and skips without it, so `cargo test` stays hermetic and nothing is vendored.

That test maps a **log port at 0x402**, and the board does not — it is an
instrument, not a chip. Firmware built for emulated machines writes its
progress there a character at a time,
and reading what a program prints is the most ordinary black-box observation
there is (`../../ROADMAP.md` §1). It is the single most useful instrument on
this board and it costs one `MemOps`.

What one such image does today, in order:

1. Fetches its reset vector from `0xfffffff0`, takes the far jump into the low
   ROM window, sets `CR0.PE`, loads a GDT and runs 32-bit code.
2. Prints its banner and its build string.
3. Finds the host bridge at `00:00.0`, unlocks `0xc0000-0xfffff` through its
   PAM registers, copies itself into the RAM that appears there and
   write-protects the copy.
4. Sizes RAM from the CMOS — **`RamSize: 0x01000000`**, which is the 640 KiB
   plus 15 MiB the machine file declares — and relocates 44 KiB of init code
   into high memory.
5. Enumerates PCI, finds **two** devices, sizes and places their windows —
   `PCI: map device bdf=00:02.0 bar 6, addr febf0000, size 00010000 [mem]` is
   the display adapter's expansion ROM — and says
   **`PCI: Using 00:02.0 for primary VGA`**. No APIC.
6. Copies its PIR, MPTABLE and SMBIOS tables *into the f-segment*, which is
   only possible because that segment is now RAM.
7. Scans for a video option ROM, finds the one behind that BAR, copies it into
   the RAM it made at `0xc0000` and far-calls it: **`Running option rom at
   c000:0003`**, then the video BIOS's own banner, then **`set VGA mode 3`**.
   From here everything the firmware prints reaches the screen as well as the
   log port.
8. Initialises the PS/2 keyboard, programs counter 0 of the 8254 in mode 2 and
   the 8259A pair (`0xb8`/`0x8e`), and takes timer interrupts at 18 Hz for as
   long as it is left running.
9. Builds a five-entry e820 map, offers its boot menu, and boots. With the
   drive empty it tries the floppy, tries the hard disk and settles into
   `sti; hlt` with **"No bootable device. Retrying in 60 seconds."** With a
   bootable diskette in it (`RSEMU_FLOPPY`) it says **"Booting from
   0000:7c00"** and hands over — and the sector runs, including through
   `INT 10h`, which is the video BIOS the firmware just installed.

That is a complete POST and a complete boot, with a picture. The BIOS data area
agrees: equipment word `0x0027` — bit 5 set, "colour 80x25" — the CRT mode byte
at `0x449` holding **3**, base memory 639 KiB at `0x413` (the EBDA takes the
other kilobyte), the tick count at `0x46c` climbing. The text page holds what
the firmware printed:

```text
|SeaBIOS (version rel-1.17.0-0-gb52ca86e094d-prebuilt.qemu.org)|
|Press ESC for boot menu.|
|Booting from Floppy...|
```

The floppy read went through the µPD765 and the 8237 on channel 2, so that path
is exercised by a real driver rather than only by its own unit tests.

### What that took, and what it cost

Four things, and three of them were surprises.

**The host bridge.** `pc.pmc` is an Intel 82441FX, and steps 3 and 4 above are
the whole reason it exists. Its module docs quote the datasheet section that
fixes the PAM encoding.

**The log port's signature.** The test's sink at `0x402` used to answer reads
with `0xff`, and that was fine for exactly as long as shadowing did not work.
A firmware built for an emulated machine *probes* that port and stores a zero
in its own f-segment if it dislikes the answer — and while the f-segment was a
ROM socket the store was swallowed, so the log kept working **by accident**.
The first run after shadowing landed printed the banner and then nothing at
all. The sink now answers `0xe9`, which is Bochs's convention for its debug
console, and the log came back. It is worth stating plainly: a working feature
made a working instrument stop, because the instrument had been relying on a
missing feature.

**A PCI VGA function and its expansion ROM BAR.** The firmware used to report
"No VGA found" and never set a mode, and the explanation was already written
down here: it sets PAM1-PAM5 to read/write, which turns `0xc0000-0xdffff` into
blank DRAM, so the legacy `vgarom` socket underneath is invisible by the time
anything scans for a signature. A 440FX-era firmware gets its video ROM off a
**PCI card's expansion ROM BAR**, and the board now has one: `pc.vga-pci` at
`00:02.0`, class code `030000`, with the `vgabios` image behind a 64 KiB ROM
window that `bus/pci`'s base address registers place where the firmware asks.

**A video BIOS writes its registers a word at a time.** Not predicted. With the
option ROM running, the first run showed **243 unanswered bus accesses, last at
`0x3d4`**: `pc.video` declared itself byte-only, and a video BIOS programs the
CRTC, the sequencer and the graphics controller with one `OUT DX, AX` per
register — index in `AL`, datum in `AH`. That is not a shortcut; it is what the
register pairs are laid out for, and what every VGA reference gives as *the*
idiom. The device now accepts a word at a naturally aligned pair and applies it
low byte first, which latches the index before the datum that uses it. Bus
faults are back to zero, and the assertion in `tests/pc_at_firmware.rs` that
they stay there is what would catch the next such gap.

**The video BIOS has to be for this card.** Also not predicted, and the most
useful thing measured here. A firmware that loads a ROM off a base address
register checks the image's **PCI Data Structure** — signature `PCIR`, then a
vendor and a device id — against the card it came off. Three runs, all
black-box:

| Image | `PCIR` | Card ids | Result |
| --- | --- | --- | --- |
| `vgabios.bin` (the ISA build) | none | 1234:1111 | mapped, never run |
| `vgabios-stdvga.bin` | 1234:1111 | 1013:00b8 | mapped, never run |
| `vgabios-stdvga.bin` | 1234:1111 | 1234:1111 | **`Running option rom at c000:0003`**, mode 3 |

A Cirrus image against a card declaring Cirrus's ids runs and then correctly
gives up — "cirrus init / Failed to initialize VGA hardware" — because the card
is not a Cirrus. So `vendor-id` and `device-id` on the `vgacard` object are
load-bearing, and the machine file says so.

### What is still not there

**A graphics mode.** `pc.video` is text-mode only and says so, and the video
BIOS agrees: it reports "No VBE DISPI interface detected, falling back to
stdvga" and sets mode 3. A Cirrus image, whose ids the machine file can be told
to match, gets as far as `cirrus init` and then correctly refuses — "Failed to
initialize VGA hardware" — because the card is not a Cirrus. Which is the right
answer, and a good demonstration that the id fields are load-bearing rather
than decoration.

## The in-house BIOS

`ROADMAP.md` phase 6a names it and says why: FreeDOS, Windows 95 and Windows XP
all need a *legacy* BIOS, the only permissively licensed PC firmware in
existence is EDK II / OVMF (which is UEFI), and every legacy alternative anyone
reaches for is GPL or LGPL and therefore unreadable to us. Shipping a blob was
not an option and reading one to learn how was not either, so it is written from
the interrupt ABI as documented.

### How the image is built

There is no assembler in this repository and no C toolchain, because §0 forbids
both. Rust cannot target 16-bit x86 either. So the firmware is **emitted**:
[`src/fw/asm16.rs`](../../src/fw/asm16.rs) is a 16-bit x86 assembler written in
Rust, [`src/fw/pcbios/`](../../src/fw/pcbios) is a Rust program that calls it,
and `rsemu::fw::pcbios::image()` returns 65,536 bytes. `cargo build` is the
whole build; the same source produces the same bytes on every host, which is
asserted rather than assumed. The alternatives that lost — a `.code16` crate
needing an external linker script, a vendored `.bin`, and a "BIOS" that is
really a host callback trapping `INT 13h` — are argued in `src/fw/mod.rs`.

### What it does, measured

`tests/pc_at_boot.rs` runs it on this board with **no environment variable and
nothing downloaded**, so it is part of an ordinary `cargo test`. Quoted from a
run:

```text
  |rsemu BIOS, 639K base, 15360K extended|
  |Booting.|
  |Booting from 0000:7c00|
  |rsemu boot sector on rsemu BIOS|
  pc-at boot: INT 13h AH=08h says 4 cylinders, 16 heads, 63 sectors, 1 drive(s)
  pc-at boot: E820[0] base=0x00000000 length=0x0009fc00 type=1
```

Option-ROM dispatch is measured the same way, with `RSEMU_VGABIOS` pointing at a
real video BIOS. rsemu's own firmware does **not** enumerate PCI, so it finds
video the way a pre-PCI machine does — a scan of `0xc0000` upward for `0x55
0xaa` — which makes the **ISA** build the right image here, the opposite of what
the 440FX-era firmware above wants. With `vgabios.bin` in the legacy socket the
scan finds it, checksums it, enters it at `seg:0003`, and the vector moves:

```text
  pc-at boot: INT 10h -> c000:53ef        (empty socket: f000:047f)
```

The ROM's own initialisation costs about 145 ms of virtual time, and POST
resumes afterwards and boots as before — so the whole text page above is then
drawn by somebody else's video BIOS through our `INT 19h`.

The sequence is: reset at `0xfffffff0`, the far jump, the 8259A pair, the 8254,
the 146818, the BIOS Data Area, the EBDA and the `E820` table built into it, the
option-ROM scan over `0xc0000-0xdffff`, the 8042 brought up with translation on,
`IDENTIFY DEVICE` on the primary IDE channel, and `INT 19h` reading cylinder 0,
head 0, sector 1 to `0000:7c00` and jumping there. The guest that lands is a
real program that calls back in: `INT 10h`, `INT 11h`, `INT 12h`, `INT 15h`
(`E820`) and `INT 13h` for its own second read, all of which answer.

The same test boots the same sector off the **diskette**, with both IDE bays
empty so `INT 19h` falls through to it — a different path end to end, through
the µPD765 and DMA channel 2 rather than the IDE command block.

The guest also *writes*: `INT 13h AH=03h` to the diskette and to the fixed disk,
each read back afterwards. The fixed disk's is compared against the drive's own
**medium** through `ata::bays`, which is the standard `tests/pc_at_ide.rs` sets;
the diskette's cannot be, because `pc.fdc` publishes no host object and
`Bindings::bind` refuses a second binding for a class `dev::pc::bind` has already
claimed — so it is a read-back instead, which is still a claim about the medium
because that controller rebuilds its sector buffer from the image at the start of
every transfer. An `Fdc765` medium accessor reachable from a realized machine
would close that gap. And the guest moves 512 bytes out to 8 MiB and back with
`INT 15h AH=87h`, the one service that reaches above the first megabyte from
real mode.

### The tables an operating system enumerates the board through

POST is not enough to make a second processor usable: something has to *say*
it is there. The ROM carries the structures that do, as data, at `0xf8000` —
inside the BIOS segment, which is the third of the three places the
*MultiProcessor Specification* §4 allows and the second of ACPI §5.2.5.1's two:

| At | Structure |
| --- | --- |
| `0xf8000` | the MP floating pointer, `_MP_` |
| `0xf8010` | the MP configuration table, `PCMP` |
| after it | the ACPI RSDP, then the DSDT, MADT, FADT, XSDT and RSDT |
| after those | the SMBIOS entry point, `_SM_`, and its structure table |

**Every field in them is read out of the machine description**, which is what
makes them worth having rather than a decoration: the processors are the
board's `cpu.x86` objects, each APIC ID comes from the `pc.lapic` wired to that
processor, the local and I/O APIC addresses are where the file maps them, and
the interrupt entries come from the board's own double wiring — `pit0.out0`
goes to `pic1.ir0` *and* `ioapic.irq2`, which is exactly the statement "ISA IRQ0
is global system interrupt 2" that an operating system needs and that no
firmware can invent. `src/fw/pcbios/platform.rs` does the reading;
`src/fw/pcbios/tables.rs` does the byte layout.

`tests/pc_at_tables.rs` is a guest that walks them: it searches the segment on
16-byte boundaries for `_MP_`, follows the pointer, checks the configuration
table's checksum, steps through `ENTRY COUNT` entries, and then does the same
through the RSDP and the RSDT to the MADT. On the shipped board it reports one
processor from both tables. On the same file with a second processor added it
reports **two**, from the same firmware source — and given the *stock* image it
reports one, which is the assertion that the tables come from the machine and
not from a constant. `tests/kvm_pc_at_smp.rs` goes one further: its boot sector
takes the application processor's local APIC ID out of the configuration table
and sends the Start-Up to that, so the processor that runs is the processor the
table named.

What is *not* published is as deliberate. There is no FACS and the FADT
declares `HW_REDUCED_ACPI`, because an AT has no ACPI hardware register
interface at all — no PM1 block, no power management timer, no SCI, no SMI
command port — and revision 5 introduced that flag precisely so a table would
not have to claim registers at address zero. The DSDT is an empty `\_SB`
scope for the same reason, and there is no MCFG (this board's configuration
space is the `0xcf8` port pair) and no HPET table (nothing describes a
processor with it).

### The one page every processor has to see differently

One thing the tables say is true of every real machine and not yet true of
this model, and it is worth knowing before an operating system finds it: both
tables have room for **one** local APIC address (*MP* §4.2's `ADDRESS OF LOCAL
APIC`, *ACPI* §5.2.12's Local Interrupt Controller Address), because on real
silicon every processor reaches its own APIC at the same physical address.
rsemu models each local APIC as a separate device with its own mapping, so a
second processor's lands somewhere else — `0xfef00000` in
`machines/pc-apic.machine` and in `tests/kvm_pc_at_smp.rs`. An application
processor that reads its own APIC ID through the architectural `0xfee00000`
therefore reads the bootstrap processor's. Enumerating and starting the second
processor works; code running on it that programs "its" APIC is programming the
first one.

**This is why no `pc-at-smp.machine` is shipped.** `tests/kvm_pc_at_smp.rs`
patches this file's text at run time to add the second processor, and says so
where it does it. A board file would be a promise, and until the page is
per-processor it would be a promise this model does not keep for any operating
system that actually schedules on the second core.

#### What it would take, since "a per-processor alias" is not one thing

The obvious reading — teach `core::space` a mapping whose target depends on who
is asking — is the expensive one and is not needed. Decode there is strictly
`address → FlatEntry`: `RegionKind` is `Ram | Rom | Io | Alias | Container`,
`FlatTarget` is `Ram | Rom | Io`, and nothing on the lookup path branches on
the initiator, so a *mapping* keyed on the processor would mean a new region
kind, a fourth flat target, and a branch on the hot path for something almost
no board wants.

It does not have to be a mapping, because **the initiator is already carried**.
`MemAttrs` has a `RequesterId` field, `src/machine/realize.rs` allocates every
object a distinct one at realize time, `cpu.x86` stamps its own on every access
it makes, and both of KVM's exit paths rebuild the attributes with the vCPU's.
That value reaches `MemOps::read` and `MemOps::write` at the leaf, unchanged,
on the interpreter and under KVM alike. Nothing in the tree reads it back
today; a per-processor APIC window would be its **first** consumer, which is
precisely the "per-master filter" the field's own documentation says it exists
for. So the window can be an ordinary device with an ordinary region that
demultiplexes on `attrs.requester`, and `core::space` needs no change at all.

Two smaller things are genuinely missing, and they are the whole of the work:

1. **A device cannot learn a *peer's* requester id.** `BindCtx` answers
   `requester()` for itself and `export(path, which)` for a neighbour's
   published handle, but there is nothing that answers "what id does `cpu1`
   stamp on its accesses?" — and a machine file must never write the number
   down, because it is allocated by declaration order. The `peers` slice inside
   `BindCtx` already carries it; this is an accessor, not a mechanism.
2. **The firmware's survey has to keep working.** `platform.rs` derives the one
   local APIC address in both tables from *where the bootstrap `pc.lapic`'s own
   `regs` region is mapped*. Put the architectural page on a different object
   and that survey reports the wrong address or fails outright — and
   `tests/pc_at_tables.rs`'s negative control, the one that proves the tables
   follow the machine, is exactly what catches it. Either the survey learns
   about the window class, or the window is declared *over* a mapping the
   bootstrap APIC still owns (`map … = target { priority = 1 }` is spellable
   today).

There is a third option and it may be the honest one: **decode the page on the
processor.** On real silicon the APIC aperture is on the die and never reaches
the bus at all, which is the actual reason every processor sees a different
thing at one address. `cpu.x86` already holds the `LocalController` the wire
seam handed it and already implements `IA32_APIC_BASE` — as *reported, not
obeyed*, because moving the window is a retopology and a device does not get to
do that to itself. A decode of `[APIC_BASE, +0x1000)` ahead of the bus would
make the MSR obeyed, give every processor its own page at `0xfee00000`, and
take the second page off the board entirely; the cost is one compare on the
MMIO path and a board question moving into a CPU core.

### FreeDOS boots

`ROADMAP.md` phase 6a's gate, and it is met. `tests/pc_at_boot.rs` boots a
**FreeDOS 1.3 diskette** on this board with rsemu's own firmware in the socket,
gated on `RSEMU_FREEDOS_FLOPPY` so an ordinary `cargo test` stays hermetic;
`scripts/fetch-testdata.sh freedos` fetches one into the ignored corpus
directory. **Nothing is vendored** — FreeDOS is GPL-2.0, running it as an
emulated guest is ordinary use, shipping it here would not be, and its source
was not read (`ROADMAP.md` §1). Quoted from a run:

```text
  |FreeCom version 0.85a - WATCOMC - XMS_Swap [Jul 10 2021 19:28:06]|
  |Welcome to the FreeDOS 1.3 installation program.|
  |Do you want to proceed [Y,N]?|
  pc-at freedos: 5.8s of host time; stopped at 2241:0000375f, halted=true
  pc-at freedos: vectors 08->0070:000f 10->f000:047f 13->f000:09c9 21->00d8:129a
```

Sixty seconds of virtual time end to end, six of host time: FreeDOS's own boot
sector loads a compressed kernel a sector at a time through `INT 13h AH=02h`,
printing a dot as it goes; the kernel decompresses and initialises,
`FDCONFIG.SYS` runs, `COMMAND.COM` prints its banner, and the installer draws a
screen and waits at a prompt. Feeding one scan code to the 8042 at that point
puts an `N` on the line after the prompt, so it is a live prompt rather than a
picture; the committed test does not do that, because a second keystroke does
not arrive (see the `pc.kbc` note in `tests/pc_at_boot.rs`) and half a
conversation is not worth asserting. `INT 21h` moving out of segment `0xf000`
is the assertion that says a kernel installed itself, and it is independent of
which DOS.

Two things the boot taught the firmware, neither predicted:

- **`INT 10h AH=08h`, read the character and attribute under the cursor, is
  called over four hundred times in the first twenty virtual seconds** — and
  was not implemented, so it answered with whatever `AX` happened to hold. It
  is now. That count is measured, by single-stepping the boot and decoding
  every `CD` opcode the guest executed.
- **`INT 15h` is not called once** over that same window — not `E820`, not
  `E801`, not `AH=88h`, not `AH=87h`. A 16-bit DOS sizes memory from the BDA
  and reaches extended memory through its own XMS driver, so the whole
  memory-map interface is for the operating systems that come after this one.
  `AH=87h` was implemented anyway, because a DOS extender is the next thing to
  run here and block move is the service it asks for; it is exercised by the
  hermetic test's own boot sector rather than by FreeDOS.

### The PCI BIOS interface

`INT 1Ah AH=B1h`, the last of phase 6a's firmware list, and the service a
DOS-era driver or a PCI option ROM uses to find a function on the bus without
knowing how a configuration cycle is generated on this board.

Implemented, over configuration mechanism #1: `B101h` installation check,
`B102h` find by vendor and device, `B103h` find by class code, and
`B108h`/`B109h`/`B10Ah` and `B10Bh`/`B10Ch`/`B10Dh` — read and write
configuration space as a byte, a word or a Dword, with the register number
range-checked and alignment-checked so a misaligned word gets
`BAD_REGISTER_NUMBER` rather than the wrong two bytes.

**POST probes for the mechanism rather than being told about it.**
`CONFIG_ADDRESS` is a Dword register whose bits 30-24 read as zero, so writing
the enable bit alone and reading back exactly the enable bit alone identifies a
live window (*PCI Local Bus* §3.7.4.1); a board with no host bridge answers
with ones and has no PCI BIOS. The probe puts the latch back to zero
afterwards, so the byte at `0xcf9` — the reset control register, which this
board's bridge passes through to `pc.sysctl` — is not left sitting behind an
enabled configuration cycle. That is the one thing about this board a
firmware can find out by *asking* rather than by reading the machine
description, and it is why the answer is a probe and not a
[`Platform`](../../src/fw/pcbios/platform.rs) field.

`tests/pc_at_pci_bios.rs` is the evidence: a boot sector calls `B101h`, checks
the `'PCI '` signature, finds the board's display adapter *twice* — once by the
vendor and device identification `machines/pc-at.machine` gives it and once by
its class code — reads its configuration space through the service and then
**reads the same registers itself through `0xcf8`/`0xcfc`**, which is the check
that the service is reporting the bus rather than reciting a constant. It also
writes the host bridge's latency timer through `B10Bh` and sees the change
through the ports. The negative control is the same firmware on the same board
with the `0xcf8` window unmapped: every function, the installation check
included, comes back with carry set.

Refused, each with `FUNC_NOT_SUPPORTED` and a reason rather than an omission:
`B106h` generate special cycle, because the fabric has no special-cycle path
and the write would be a master abort no device sees; `B10Eh`/`B10Fh`, the
`$PIR` interrupt routing table, because this board has no south bridge and
therefore no routing to report; and the 32-bit BIOS32 service directory,
because `src/fw/asm16.rs` emits 16-bit code and a `_32_` structure is found by
a search, so publishing one that could not be entered would be worse than
publishing none. The scan covers bus 0 and `B101h` says so in `CL`: a bus
beyond 0 lives behind a PCI-to-PCI bridge whose bus numbers firmware assigns at
POST, and nothing in the tree models one.

### What it does not do

- **No `INT 10h AH=11h`**, the character-generator group. `AL=30h` answers with
  a pointer to *the font being displayed*, and on this board that font is
  `pc.video`'s — the adapter draws the glyphs and there is no font in the ROM
  at all. A copy here would answer with a different font from the one on the
  screen, which is a worse answer than carry, and it would put a font's
  provenance inside the firmware where `pc.video`'s original one is already
  argued. FreeDOS calls it once while booting and does not mind.
- **Text mode only**, because `pc.video` is a text-mode CRTC. `INT 10h AH=00h`
  records a graphics mode and changes nothing.
- **`INT 10h AH=06h` scrolls the whole screen** when the line count is
  non-zero; the rectangle is honoured for a clear (`AL=0`), which is what
  programs use it for.
- **US layout, base scan codes only.** Extended (`E0`-prefixed) keys are
  dropped rather than half-decoded, and the lock states are not tracked.

### Which boards actually reach this firmware

Worth writing down, because it is easy to assume more than is true:

- **`pc-at`** — the only board `rsemu run` offers it to. `builtin_bios` in
  `src/bin/rsemu.rs` matches the machine's stem, and `pc-at` is the only stem
  in the match; the image is assembled from *that* description, so a user who
  edits their copy gets tables that describe their copy.
- **`q35`** — only from `tests/q35_board.rs`, which puts the image in the
  socket explicitly. `machines/q35.machine`'s firmware comment says the slot
  "defaults to rsemu's own 64 KiB image", and on the command line it does not:
  the default is offered per machine stem and `q35` is not one of them. Adding
  it is a board decision rather than a firmware one, which is why it is
  recorded here rather than taken.
- **`pc-apic` does not reach it at all.** `machines/pc-apic.machine` has no
  builtin and `tests/pc_apic.rs` assembles its own protected-mode stub by hand;
  neither goes near `src/fw/pcbios`.

**On `q35` there are two valid RSDPs, and the right one wins.** The board's
`q35.acpi` device maps its generated tables at `0xe0000`, and this firmware
lays its own at `0xf8000`. ACPI §5.2.5.1 has OSPM search the EBDA's first
kilobyte and then `0xE0000`-`0xFFFFF` on 16-byte boundaries and take the first
valid structure, which is the device's — and that is the one that should win,
because it describes the machine *as realized*, with an MCFG for the ECAM
window and a `_PRT` read out of the bridge's routing registers, while this
firmware's set describes `pc-at`. The MP table at `0xf8000` has no competitor
and is found; a legacy operating system on a q35 therefore gets an MP table
from the ROM and an ACPI set from the board, which is the correct pairing
rather than a coincidence.

### Formatting a diskette

`INT 13h AH=05h` is implemented, and the reason it was not — "formatting is a
different command phase" — stopped being true once `pc.fdc` grew `FORMAT A
TRACK`. The chip reads four ID bytes per sector out of memory through the same
8237 channel the data would use, so the firmware's part is a seek (the command
formats the track the head is *on*, and takes no cylinder parameter), a DMA
programming whose length is four bytes per sector rather than 512, and six
command bytes.

`tests/pc_at_format.rs` runs it on the second head of cylinder 0 — a track this
diskette does not use, so a bug there cannot destroy the boot sector the test is
running from — and reads the same sector before and after: zeros become the
`0xF6` filler, which nothing but a format that reached the drive can do.

### Two things the board does that the firmware found

Both are device-side, both are measured by `tests/pc_at_boot.rs`, and neither is
a firmware bug:

- **A PAM window with nothing under it faults instead of reading as ones.**
  `pc.pmc` maps thirteen shadow windows across `0xc0000-0xfffff`. Where a
  window has no permission and no region beneath it — `0xd0000-0xdffff` on this
  board — a read returns `Err(Protected)` rather than falling through to the
  space's `unassigned = read-as-ones`. An option-ROM scan walks exactly that
  range, so the CPU's bus-fault counter climbs by 32 during a normal POST. A
  440FX with a window set to "ROM read" and no ROM there is an ISA bus with
  pull-ups, which reads as ones.
- **`pc.kbc` delivers one keystroke and then goes quiet.** `read_data` clears
  `OBF` and immediately refills the output buffer from the keyboard's queue
  without re-driving `IRQ1`, so the line never falls between two bytes and the
  edge-triggered 8259A never sees a second edge. Measured: after one key the
  status port reads `0x05` (a byte waiting) with the master's `IRR` at `0x00`.
  On real hardware the line drops when the buffer is read and rises when the
  next byte arrives a serial frame later. The fix is a `refresh()` after the
  clear and a deferred refill.

## What is known to be missing

- **PCI I/O BARs that decode.** The register is complete — firmware can size
  and place one — but `Bars::install` refuses to *map* one, and says why: a
  configuration cycle travels through the I/O space, so the order-exempt
  try-lock that makes a memory BAR move from inside a configuration write
  cannot help there. Nothing in the tree has an I/O BAR yet, so the deferred
  action that would be the escape is not written on a guess.
- **Everything else on the bus.** A host bridge and a display adapter. No
  south bridge, so no PCI IDE, no PCI interrupt routing and no `PIRQ` swizzle;
  no bridges, so no bus but bus 0.
- **`0x510`/`0x511`.** A firmware built for another emulator reads its whole
  configuration — memory map, boot order, SMBIOS and ACPI tables — from a
  paravirtual interface at those ports. Its strings show it *detects* the
  interface rather than requiring it, so the fallback path exists — and it is
  now measured: with nothing there the image falls back to the CMOS for its
  memory map and keeps going.
- **A graphics mode.** Setting mode 3 writes the sequencer, the graphics
  controller, the attribute controller and the DAC, and `pc.video` implements
  that register file — a real video BIOS drives all of it and reaches a text
  console. What it does *not* implement is any graphics mode, deliberately, so
  a guest that asks for one gets a register file that latches and a screen that
  does not change.
- **The keyboard takes raw set-2 scan codes** on a character port. Mapping a
  terminal's keystrokes to scan codes is a host concern and belongs in `host/`.
- **No serial port, no parallel port, no APIC, no ACPI.** The firmware finds
  and reports all four absences and carries on.
- **No ATAPI.** The IDE channels carry `ata.disk` and nothing else; `IDENTIFY
  PACKET DEVICE` aborts, which is what a non-packet device does. A CD-ROM is a
  separate command set on the same transport, not a flag on this one.
- **No busmastering IDE DMA.** PIO only, which is what an AT's cable does; the
  DMA modes arrived with PCI and would need a busmaster on a fabric this board
  does not have.

## Framework gaps this board ran into

- **`BindCtx` cannot reach a sibling device's handle.** Every device-to-device
  relationship here had to travel along a wire instead. That worked, and
  arguably produced a better model — the acknowledge cycle and the DMA handshake
  really are properties of the net. But there are now *two* interfaces riding on
  a net with a hook pair each; a third should unify them.
- **A device cannot read the machine's memory map.** The RTC has to be told the
  memory sizes that the `ram` objects already state, so the machine file writes
  each number twice. The RISC-V board solves the same problem for its device
  tree by reading the realized machine; a PC's CMOS is that question asked in
  1984.
- **`Device::sink` returns `Option`**, so a typo'd input pin name is
  indistinguishable from an unknown one, while `connect` gets a real error
  naming the valid pins.
