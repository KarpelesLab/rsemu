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
| PCI BIOS Specification 2.1 | `INT 1Ah AH=B1h`: the real-mode service a DOS-era driver or an option ROM finds a function and reads its config space through. Openly published, unlike the Local Bus specification, and what `src/fw/pcbios/pci.rs` is written from |
| Ralf Brown's Interrupt List, `INT 1A/AH=B1h` | The same ABI as software in the field actually calls it — a catalogue of interfaces, with no implementation in it |
| Device datasheets | Every PCI device's datasheet documents its own config space, BARs and capabilities — this is the authoritative source for the device you are modelling |

## Implementation notes

- **BARs are the interesting part**: sizing (write all-ones, read back the
  mask), enable/disable via the command register, and remapping at runtime.
  Each mapped BAR is a region inserted into an address space at a priority above
  RAM, and it moves. This is exactly the case the topology generation counter
  exists for.
- **A function has two spaces, not one.** Bit 0 of a base address register says
  which, so `Bars::install` takes a `BarSpaces` — a memory space for memory BARs
  and the expansion ROM, an I/O space for I/O BARs — and `Bars::sync` makes one
  pass per space. The two topology guards are taken **sequentially and never
  nested**: the same lock rank twice is the violation `core::space` names, so a
  write that moves windows in both spaces can leave one placed and the other
  deferred. A register that decodes into a space the caller did not supply is
  refused by name at install, because the alternative is a register firmware can
  size and place behind which nothing ever answers.
- **Which window is deferred depends on the route to configuration space, not on
  the kind of BAR.** A retopology cannot happen inside an access that is
  travelling through the space being retopologised — the access holds that
  space's topology lock for reading, the blocking guard would be `TOPOLOGY`
  twice, and the order-exempt try-lock fails. So:

  | route | memory BAR | I/O BAR |
  | --- | --- | --- |
  | ports `0xcf8`/`0xcfc` (the access is in the I/O space) | placed at once | **deferred** |
  | ECAM (the access is in the memory space) | **deferred** | placed at once |

  "Deferred" cannot mean "retry at the next configuration access", because that
  access takes the same route and fails the same way. It means the function
  reports `PciFunction::retopology_owed`, `PciBus` remembers that something on
  the fabric owes a retopology, and a device with a clock domain drains it from
  `Device::advance_to` — `dev::pc::pmc` on a 440FX board, `dev::q35::mch` on a
  q35. The bound is one scheduler round. What is deferred is "make the map agree
  with the registers", not "apply this base", so two moves in one round collapse
  into one retopology at the second base and the intermediate base never decodes.
  `src/bus/pci/bar.rs`'s module docs carry the whole argument, including what a
  guest observes in between.
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
