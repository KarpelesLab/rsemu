# Firmware, boot, and platform description

Consumed by: `boards/*`. This is the layer where a machine stops being
a pile of devices and becomes something an OS will boot on.

## Specifications

| Source | Covers | Access |
| --- | --- | --- |
| [UEFI Forum specifications](https://uefi.org/specifications) | The **UEFI** specification and the **ACPI** specification (both published here) | **[browser]** — free download |
| [DMTF SMBIOS](https://www.dmtf.org/standards/smbios) | System management BIOS tables: what the guest reads to learn the machine's identity | **[browser]** — free |
| [Devicetree Specification](https://www.devicetree.org/specifications/) | The DTB format and standard bindings — how non-PC machines describe themselves | Free |
| [Multiboot](https://www.gnu.org/software/grub/manual/multiboot/multiboot.html) | The Multiboot boot protocol | GNU manual (GFDL) — a protocol specification |
| [OSDev: UEFI](https://wiki.osdev.org/UEFI) / [ACPI](https://wiki.osdev.org/ACPI) / [Multiboot](https://wiki.osdev.org/Multiboot) | Practical orientation | Free |

## Firmware implementations

| Project | Licence | Usable? |
| --- | --- | --- |
| [EDK II / OVMF](https://github.com/tianocore/edk2) | BSD-2-Clause-Patent | **Yes** — permissive; may be read and used with attribution |
| SeaBIOS | GPL | **No** — run it as a binary if needed, never read the source |
| coreboot | GPL | **No** |
| [OpenSBI](https://github.com/riscv-software-src/opensbi) | BSD-2-Clause (verified) | **Yes** |

This is one of the places where the licence difference has a real engineering
consequence: EDK II and OpenSBI are open to us, the GPL firmwares are not.

## ACPI, in practice

The PC machines must **generate** ACPI tables (DSDT, FADT, MADT, MCFG) describing
the topology they realized, exactly as the RISC-V board generates a device tree.
Both are the same idea: a machine that can describe itself mechanically from its
realized graph is a machine whose model is well-formed. If a table cannot be
generated from the topology, the topology is missing information.

AML (the ACPI bytecode in the DSDT) is the awkward part — it must be produced,
and guests are unforgiving about malformed tables. The ACPI specification at
uefi.org defines the language.

## Implementation notes

- Boot order: firmware load → firmware init → boot device selection → OS. Each
  step is a place a machine file may want to intervene, so make them explicit
  rather than hard-coded.
- Guests probe far more than they document; the practical loop is boot, observe,
  implement — with the specification open.
