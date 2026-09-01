# `pc-at` — the machine rsemu builds, and the firmware it does not ship

Consumed by: `machines/pc-at.machine` and `src/dev/pc`. The source *register*
for the platform is [`ibm-pc.md`](ibm-pc.md); this page is about the board rsemu
actually assembles and what is known to be missing from it.

## The firmware is the user's, always

rsemu ships no BIOS and no video BIOS, and will not. A PC's firmware is somebody
else's copyrighted binary; **running one as an emulated guest is ordinary use
whatever its licence, and redistributing it is not** (`../../ROADMAP.md` §1). So
the machine takes both through media slots, exactly as `riscv-virt` takes its
firmware:

```console
$ rsemu run pc-at --bios /usr/share/qemu/bios.bin
$ rsemu run pc-at --bios bios.bin --media vgabios=vgabios.bin --media floppy=boot.img
```

`--bios` and `--vgabios` are conveniences over `--media bios=…`, and so are
`--hd0` and `--hd1`, which fill the two drive bays on the primary IDE channel;
the mechanism is the media table and nothing else. An unbound `hd0` or `hd1` is
an **empty bay**, not an error — a PC with no hard disk is an ordinary PC, and a
drive costs its whole capacity in host memory the moment it exists. `pc.rom` is the socket they land in, and
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
5. Enumerates PCI, finds the one device there is, allocates its windows, and
   reports no VGA and no APIC.
6. Copies its PIR, MPTABLE and SMBIOS tables *into the f-segment*, which is
   only possible because that segment is now RAM.
7. Scans for a video option ROM, finds none, initialises the PS/2 keyboard,
   programs counter 0 of the 8254 in mode 2 and the 8259A pair (`0xb8`/`0x8e`),
   and takes timer interrupts at 18 Hz for as long as it is left running.
8. Builds a five-entry e820 map, offers its boot menu, and boots. With the
   drive empty it tries the floppy, tries the hard disk and settles into
   `sti; hlt` with **"No bootable device. Retrying in 60 seconds."** With a
   bootable diskette in it (`RSEMU_FLOPPY`) it says **"Booting from
   0000:7c00"** and hands over — and the sector runs: a twenty-eight byte boot
   sector that writes to `0xb8000` puts its string on the text page, which the
   test prints.

That is a complete POST and a complete boot. The BIOS data area agrees:
equipment word `0x0007`, base memory 639 KiB at `0x413` (the EBDA takes the
other kilobyte), the tick count at `0x46c` climbing. The floppy read went
through the µPD765 and the 8237 on channel 2, so that path is exercised by a
real driver rather than only by its own unit tests.

### What that took, and what it cost

Two things, and the second was a surprise.

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

### What is still not there

**No video.** The firmware reports "No VGA found" and never sets a video mode,
so nothing *it* prints reaches the screen — the text page only ever holds what
a guest wrote there directly. That is not a bug in `pc.video` and it is not new
— it happened before shadowing too — but it is
now *explained*: the firmware sets PAM1-PAM5 to read/write, which turns
`0xc0000-0xdffff` into blank DRAM, and it does not copy an ISA-style option
ROM into it first because a 440FX-era machine gets its video ROM off a PCI
card's expansion-ROM BAR. So this board's `vgarom` socket — a legacy ISA ROM
at `0xc0000` — is invisible to a firmware that knows what a 440FX is. Getting
a video BIOS in front of this firmware needs base address registers and a PCI
VGA function, not a bigger ROM socket.

## What is known to be missing

- **PCI base address registers.** `bus/pci` has configuration space, the
  `0xcf8`/`0xcfc` mechanism and master aborts, and no BARs: a BAR is a mapping
  that *moves*, from inside a configuration write, and no function in the tree
  has one yet. It is the next thing this board needs, because a video BIOS
  arrives on a PCI card's expansion-ROM BAR.
- **Everything else on the bus.** One host bridge and nothing behind it. No
  south bridge, so no PCI IDE, no PCI interrupt routing and no `PIRQ` swizzle;
  no PCI VGA; no bridges, so no bus but bus 0.
- **`0x510`/`0x511`.** A firmware built for another emulator reads its whole
  configuration — memory map, boot order, SMBIOS and ACPI tables — from a
  paravirtual interface at those ports. Its strings show it *detects* the
  interface rather than requiring it, so the fallback path exists — and it is
  now measured: with nothing there the image falls back to the CMOS for its
  memory map and keeps going.
- **A video BIOS needs a real VGA**, not a text-mode CRTC: setting even mode 3
  writes the sequencer, the graphics controller, the attribute controller and
  the DAC. `pc.video` implements that register file; what it does *not* implement
  is any graphics mode, deliberately.
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
