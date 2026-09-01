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
cannot find a host bridge to unlock RAM for shadowing. Three of its checks are
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
3. Prints **"Unable to unlock ram - bridge not found"** — see below.
4. Sizes RAM from the CMOS, and gets **zero**, because it stores the answer in
   a variable in the `f` segment and that segment is ROM.
5. Prints "No space for init relocation." and stops on `cli; hlt; jmp .-1`.

Step 4 is the blocker and step 3 is its cause. The firmware expects the
`0xc0000-0xfffff` window to become **writable RAM** once it has asked a PCI
host bridge to unlock it, and this board has neither the bridge nor the
shadow. The same image on the same core against flat RAM — `cpu::x86::firmware`
— runs twenty million instructions without complaint precisely because
everything there is RAM, which is the control experiment for this claim.

With a writable copy of the image laid over that window as an experiment
(`RSEMU_SHADOW=1` in the same test, which is **not** a board and says so), the
firmware goes considerably further: RAM sizing succeeds, it relocates its init
code into high memory, programs the 8259A pair (master mask `0xb8`, slave
`0x8e`) and counter 0 of the 8254 in mode 2, takes **timer interrupts at 18 Hz
for as long as it is left running**, and fills the BIOS data area — equipment
word `0x0007`, base memory 639 KiB at `0x413`, the tick count at `0x46c`
climbing. It then waits in the BIOS's own `sti; hlt; cli` idle without setting
a video mode or loading a boot sector, and what it is waiting for is not yet
known. It is the same with and without a video BIOS in the socket, so it is
stuck *before* the option-ROM scan.

## What is known to be missing

- **RAM shadowing, and the host bridge that controls it.** The system BIOS
  window is a ROM socket: `pc.rom` swallows writes, which is what a board with
  no shadow control does. Current firmware needs the window to be writable and
  asks a PCI host bridge's PAM registers for it. This is the first thing
  standing between this board and a boot, and it needs `bus-pci` and a host
  bridge before it needs anything else.
- **PCI.** Firmware probes `0xcf8`/`0xcfc` for a host bridge. With nothing
  mapped there the reads return ones and the probe concludes there is no PCI —
  measured: it says so and carries on.
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
- **No serial port, no parallel port, no PCI, no APIC, no ACPI.**
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
