//! The q35 chipset: an Intel 82Q35 (G)MCH and an ICH9, and the ACPI tables the
//! pair has to publish before a modern operating system will look at them.
//!
//! # What is here and what is deliberately not
//!
//! A q35 board is mostly parts rsemu already has. The two 8259As, the 8254, the
//! MC146818, the 8042, the two 8237As, the local APIC, the I/O APIC and the
//! HPET are all in [`crate::dev::pc`] and `machines/q35.machine` instantiates
//! them by name, exactly as `machines/pc-at.machine` does. **Nothing in this
//! module reimplements any of them**, and that is the point of the module
//! rather than an economy: what makes a q35 a q35 is the *chipset around* those
//! parts, and it is four things.
//!
//! | Module | What it is |
//! | --- | --- |
//! | [`ecam`] | configuration space reached by reading memory — the PCI Express mechanism |
//! | [`mch`] | the north bridge: `PCIEXBAR`, which places the ECAM window, and `PAM`, which shadows the BIOS |
//! | [`lpc`] | the south bridge: PCI interrupt routing, and `PMBASE`, which places the ACPI register block |
//! | [`pm`] | that register block: `PM1_STS`/`PM1_EN`/`PM1_CNT` and the 3.579545 MHz power-management timer |
//! | [`acpi`] | the tables, **generated from the realized machine** |
//! | [`aml`] | the byte encoder the DSDT is built with |
//!
//! # How much PCI Express this is
//!
//! Exactly one mechanism: [`ecam`], the memory window in which the address is
//! the configuration address. That is enough to enumerate a q35 and it is what
//! the `MCFG` table exists to announce.
//!
//! What is **not** here, and what each would cost:
//!
//! * **Root ports.** A real q35 puts its slots behind ICH9 root ports, which
//!   are Type 1 (bridge) configuration headers that forward cycles to a
//!   secondary bus. [`crate::bus::pci`] has no Type 1 header and nothing that
//!   forwards, and its module docs say so. Everything on this board is on bus 0,
//!   which is a legal PCI Express topology (the root complex's own integrated
//!   endpoints live there) and is what an enumeration finds. A device that
//!   genuinely needs to be behind a bridge — hot-plug, a second segment — needs
//!   the Type 1 header first, and that belongs in `bus::pci` rather than here.
//! * **Express capability structures.** A function with no PCI Express
//!   Capability is a *conventional* PCI function as far as software is
//!   concerned, which is what every device in this tree currently is. Adding
//!   one is per-device work, not chipset work.
//! * **Link state, AER, MSI/MSI-X.** None of them is on the path from power-on
//!   to a kernel finding its root device.
//!
//! # Sources
//!
//! Each file cites its own; the shared ones are:
//!
//! * *Intel 3 Series Express Chipset Family Datasheet*, order number
//!   316966-002 — the (G)MCH: Table 5-1's register map, §5.1.16 `PCIEXBAR`,
//!   §5.1.18-§5.1.24 `PAM0`-`PAM6`.
//! * *Intel I/O Controller Hub 9 (ICH9) Family Datasheet*, order number
//!   316972-004 — the south bridge: Table 13-1's register map, §13.1.13-§13.1.19
//!   for `PMBASE`, `ACPI_CNTL` and the `PIRQ` routers, Table 13-11 and
//!   §13.8.3.x for the ACPI register block.
//! * *ACPI Specification*, revision 6.5 (UEFI Forum, openly published) —
//!   chapter 5 for every table this module writes.
//! * *PCI Express Base Specification* for the Enhanced Configuration Access
//!   Mechanism.
//! * *IA-PC HPET Specification*, revision 1.0a, §3.2.4 for the `HPET` table.
//!
//! **No emulator source and no firmware source was consulted for any of it**
//! (`CLAUDE.md`, provenance).

pub mod acpi;
pub mod aml;
pub mod ecam;
pub mod lpc;
pub mod mch;
pub mod pm;

use alloc::vec::Vec;

use crate::core::error::Result;
use crate::core::registry::Registry;
use crate::machine::realize::Bindings;
use crate::machine::validate::ClassSchema;

/// Add every class in this module to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if a name is already claimed.
pub fn register(reg: &mut Registry) -> Result<()> {
    mch::register(reg)?;
    lpc::register(reg)?;
    acpi::register(reg)?;
    Ok(())
}

/// Bind every class in this module into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if a name is bound twice.
pub fn bind(b: &mut Bindings) -> Result<()> {
    mch::bind(b)?;
    lpc::bind(b)?;
    acpi::bind(b)?;
    Ok(())
}

/// What the validator should know about every class in this module.
#[must_use]
pub fn schemas() -> Vec<ClassSchema> {
    alloc::vec![mch::schema(), lpc::schema(), acpi::schema()]
}

/// The machine description this chipset was written for, compiled in so that a
/// build which can realize it always ships one that parses.
///
/// It is data, not code: a user copies `machines/q35.machine` and edits it.
pub const Q35: &str = include_str!("../../../machines/q35.machine");

#[cfg(test)]
mod tests;
