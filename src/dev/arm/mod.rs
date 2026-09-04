//! The AArch64 `virt` board: the devices a core needs to boot an operating
//! system.
//!
//! `ROADMAP.md` §6 makes booting real system software the point of a CPU core,
//! and `machines/a64-mini.machine` is honest that it cannot: it is RAM, a core
//! and a timer, with no interrupt controller, no console and no way for a
//! guest to stop the machine. This module is what turns the same core into a
//! board — the smallest credible thing an AArch64 kernel can be pointed at:
//! a GIC, a PL011, a firmware interface and a generated device tree.
//!
//! | Module | Covers |
//! | --- | --- |
//! | [`gic`] | the GICv2 distributor and CPU interface: enable, priority, claim, end |
//! | [`pl011`] | the ARM PrimeCell UART, on the character-device seam |
//! | [`power`] | where `PSCI_SYSTEM_OFF` lands, as a named host signal |
//! | [`dt`] | the device tree *generator*, which walks the realized machine |
//! | [`boot`] | the reset vector, and where the generated tree lands |
//! | [`loader`] | putting a kernel or a ramdisk into guest memory |
//!
//! PSCI itself is **not** here, and could not be: `SMC` and `HVC` are
//! instructions, so the handler is in
//! [`cpu::arm::a64::psci`](crate::cpu::arm::a64::psci) where the register file
//! is. What is here is [`power`], the board's answer to the two requests that
//! have to leave the core.
//!
//! # The board
//!
//! `machines/arm64-virt.machine` puts it together. The layout is the
//! conventional one for an AArch64 `virt` board, and it is conventional rather
//! than required: the guest learns every address from the device tree we
//! generate, so nothing outside this repository fixes these numbers. They are
//! the familiar ones because familiar is easier to debug.
//!
//! ```text
//!   0x0000_0000  boot ROM: the reset vector, then the generated DTB
//!   0x0800_0000  GIC distributor
//!   0x0801_0000  GIC CPU interface
//!   0x0900_0000  PL011 UART
//!   0x4000_0000  DRAM
//! ```
//!
//! # How far it boots
//!
//! Measured, not asserted (`ROADMAP.md` §0), by `tests/a64_linux.rs`:
//!
//! * **A bare-metal program** written into DRAM prints through the PL011,
//!   programs the GIC out of the numbers in its own generated device tree,
//!   takes a generic-timer interrupt that leaves the core and comes back
//!   through the distributor, and stops the machine with a PSCI call. No
//!   firmware, no operating system, no fetched fixture — `tests.rs` beside
//!   this file.
//! * **Linux** (a Debian `arm64` 6.12 `Image`, fetched by
//!   `scripts/fetch-testdata.sh arm64-linux`) boots to a **busybox shell**:
//!   it parses the generated tree (`Hardware name: rsemu arm64-virt (DT)`),
//!   brings up the GIC and the architected timer, drives the PL011 as
//!   `ttyAMA0`, unpacks an initramfs, reaches `Run /init as init process` and
//!   gets a prompt. Typing `poweroff` at that prompt stops the machine through
//!   PSCI.
//!
//! `docs/platforms/arm64-virt.md` has the transcript, the two core bugs the
//! boot found, and the ledger of what is still in the way.
//!
//! # What this module is not
//!
//! The DTB *format* is not here. It is [`dev::fdt`](crate::dev::fdt), behind
//! `dev-fdt`, and this module's [`dt`] is one of the two generators that write
//! through it — the other being [`dev::riscv::dt`](crate::dev::riscv::dt).
//! Chapter 5 of the Devicetree Specification describes a container that knows
//! nothing about the architecture inside it; which nodes go in the container
//! is entirely this board's business.
//!
//! virtio is not here either: [`dev::virtio`](crate::dev::virtio), behind
//! `dev-virtio`, which is what this board's disk is. What *is* board-specific
//! about a virtio device — the `virtio,mmio` node [`dt`] describes it with —
//! is a `dev-arm` block inside that module, exactly as `dev::arm::pl011`'s
//! node is written here.
//!
//! [`power`]'s `Signal` is still `dev::riscv::syscon`'s twin, and is still
//! filed under its own [`HostKind`] — `power`, not `signal` — deliberately: a
//! kind's identity is its *name alone*, so two modules sharing a name must
//! agree about the type stored under it, and these two do not (one carries an
//! exit code a syscon can report and PSCI has no way to express).
//!
//! [`HostKind`]: crate::core::hosts::HostKind
//!
//! # Provenance
//!
//! Written from the *ARM Generic Interrupt Controller Architecture
//! Specification v2.0* (IHI 0048), the *PrimeCell UART (PL011) Technical
//! Reference Manual* (DDI 0183), the *Arm Power State Coordination Interface*
//! (DEN 0022), the *Arm Architecture Reference Manual for A-profile* (DDI
//! 0487) and the *Devicetree Specification* v0.4. Each module names the
//! sections it used. Where a fact was needed that only the Linux kernel's own
//! GPL-2.0 documentation states — the `Image` header layout and the boot
//! register hand-off — it was taken instead from the permissive
//! implementations that agree with it: the ARM boot-wrapper (BSD-3-Clause),
//! Trusted Firmware-A (BSD-3-Clause), TianoCore EDK II (BSD-2-Clause-Patent),
//! crosvm (BSD-3-Clause), Zephyr and Apache NuttX (Apache-2.0). No emulator
//! source of any licence was consulted and no Linux driver was read.

pub mod boot;
pub mod dt;
pub mod gic;
pub mod loader;
pub mod pl011;
pub mod power;

// The board-level tests need a core and the machine layer to run on, so they
// come with `machine-arm64-virt` rather than with the devices alone.
#[cfg(all(test, feature = "machine-arm64-virt"))]
mod tests;

pub use boot::BootRom;
pub use gic::Gic;
pub use loader::Loader;
pub use pl011::Pl011;
pub use power::Power;

/// Add every board class to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is already claimed.
pub fn register(registry: &mut crate::core::Registry) -> crate::core::Result<()> {
    gic::register(registry)?;
    pl011::register(registry)?;
    power::register(registry)?;
    boot::register(registry)?;
    loader::register(registry)
}

/// Bind every board class into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::Result<()> {
    gic::bind(bindings)?;
    pl011::bind(bindings)?;
    power::bind(bindings)?;
    boot::bind(bindings)?;
    loader::bind(bindings)
}

/// Every board class's validator schema.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![
        gic::schema(),
        pl011::schema(),
        power::schema(),
        boot::schema(),
        loader::schema(),
    ]
}
