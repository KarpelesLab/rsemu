# PCI and PCI Express

Consumed by: `bus/pci`. PCI is the hardest test of the memory-region
model — if BARs map cleanly through the priority/alias machinery of
`ROADMAP.md` §4.1, that design is right.

## Primary

| Source | Covers | Access |
| --- | --- | --- |
| PCI Local Bus Specification 3.0 | Configuration space, BAR sizing, interrupt routing, bridges | PCI-SIG **[browser]**, membership required |
| PCI Express Base Specification | The modern fabric: TLPs, capabilities, MSI/MSI-X, ARI/SR-IOV | PCI-SIG **[browser]**, membership required |

PCI-SIG specifications are **paywalled behind membership**, which is a real
practical obstacle. Fortunately config-space layout and BAR behaviour — the
parts an emulator implements — are also documented in every chipset and device
datasheet, which are free.

## Freely available and sufficient for most work

| Source | Covers |
| --- | --- |
| [OSDev: PCI](https://wiki.osdev.org/PCI) | Config space layout, enumeration, BAR decoding, the header types. Enough to implement a working PCI host bridge |
| [OSDev: PCI Express](https://wiki.osdev.org/PCI_Express) | ECAM (memory-mapped config), extended config space |
| Device datasheets | Every PCI device's datasheet documents its own config space, BARs and capabilities — this is the authoritative source for the device you are modelling |

## Implementation notes

- **BARs are the interesting part**: sizing (write all-ones, read back the
  mask), enable/disable via the command register, and remapping at runtime.
  Each mapped BAR is a region inserted into an address space at a priority above
  RAM, and it moves. This is exactly the case the topology generation counter
  exists for.
- **Config space is a separate address space**, not part of memory. Model it as
  one, with the ECAM window as an alias into it for PCIe.
- Interrupt routing goes through two eras: legacy INTx (with the swizzle across
  bridges) and MSI/MSI-X (a memory write to an address the OS programs). Both
  are needed; MSI is much easier to model correctly. **INTx is implemented**:
  `bus::pci::Intx` is a function's pin, `bus::pci::swizzle` is the rotation by
  device number (PCI-to-PCI Bridge 1.1 §9.1), and the fabric resolves the four
  shared, level-sensitive, open-drain nets and hands them to whatever registered
  as its `IntxSink` — an ICH9's `PIRQ` routers, on the q35 board. The set of
  asserting functions is kept rather than a level per net, because "the line
  stays down until the last driver lets go" is the whole difficulty. MSI is not
  implemented: no function in the tree has the capability yet.
- Bus mastering means a device performs DMA — through **its own address space**,
  not the CPU's. See the per-master address space requirement in §4.1.

## ⚠ Do not consult

Linux's `drivers/pci` and any GPL emulator's PCI implementation. Use the device
datasheet for the device, and OSDev for the bus mechanics.
