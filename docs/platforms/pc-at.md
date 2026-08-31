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

`--bios` and `--vgabios` are conveniences over `--media bios=…`; the mechanism
is the media table and nothing else. `pc.rom` is the socket they land in, and
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
| CPU | `cpu.i8086` | — | [`../cpu/x86.md`](../cpu/x86.md) |
| Interrupt controllers (2, cascaded) | `pc.pic` | 0x20, 0xa0, ELCR at 0x4d0/1 | Intel 8259A data sheet |
| Timer | `pc.pit` | 0x40-0x43 | Intel 8254 data sheet |
| Keyboard controller, A20, reset | `pc.kbc` | 0x60, 0x64 | Intel 8042 data sheet |
| System control ports | `pc.sysctl` | 0x61, 0x92, 0xcf9 | AT Technical Reference, Intel chipset data sheets |
| RTC and CMOS | `pc.rtc` | 0x70-0x71 | Motorola MC146818 data sheet |
| DMA (2, byte and word) | `pc.dma` | 0x00-0x0f, 0xc0-0xdf, pages at 0x80 | Intel 8237A data sheet |
| Display | `pc.video` | 0x3b4/0x3d4, 0x3c0-0x3cf, 0x3da | MC6845 data sheet, VGA register set |
| Floppy controller | `pc.fdc` | 0x3f0-0x3f7 | NEC µPD765A data sheet |
| Firmware sockets | `pc.rom` | 0xc0000, 0xe0000 (+ a high alias) | — |

Five oscillators, because the board has five cans: the CPU clock, the 8254's
105/88 MHz — not an integer number of hertz, which is why the description
language takes rational frequency literals — the RTC's 32.768 kHz, the VGA dot
clock, and the floppy data separator. Only ratios *within* a tree are
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

## What is known to be missing

- **The CPU is not bound into the machine graph.** `cpu.i8086` is registered but
  has no `Instance` impl, no `bind`, no input pins and no `schema`, so a machine
  file cannot give it an address space or wire an interrupt to it. Until it does,
  `pc-at` is shipped as data and checked by `dev::pc`'s tests rather than listed
  in the catalog. What is needed is enumerated in `src/dev/pc/mod.rs`'s tests.
- **The core is 8086/8088 real-mode only**, and any current PC firmware needs at
  least a 386: control-register moves, `lgdt`/`lidt`, the 32-bit prefixes,
  protected mode, and `cpuid`.
- **PCI.** Firmware probes `0xcf8`/`0xcfc` for a host bridge. With nothing
  mapped there the reads return ones and the probe should conclude there is no
  PCI — but that is a claim to test, not to assume.
- **`0x510`/`0x511`.** A firmware built for another emulator reads its whole
  configuration — memory map, boot order, SMBIOS and ACPI tables — from a
  paravirtual interface at those ports. Its strings show it *detects* the
  interface rather than requiring it, so the fallback path exists; how far it
  goes is unmeasured, and measuring it needs the 386 core.
- **A video BIOS needs a real VGA**, not a text-mode CRTC: setting even mode 3
  writes the sequencer, the graphics controller, the attribute controller and
  the DAC. `pc.video` implements that register file; what it does *not* implement
  is any graphics mode, deliberately.
- **The keyboard takes raw set-2 scan codes** on a character port. Mapping a
  terminal's keystrokes to scan codes is a host concern and belongs in `host/`.
- **No serial port, no parallel port, no IDE, no PCI, no APIC, no ACPI.**

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
