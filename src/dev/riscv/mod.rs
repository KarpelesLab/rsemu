//! The RISC-V `virt` board: the devices a hart needs to boot an operating
//! system.
//!
//! `ROADMAP.md` §6 picks RISC-V as the first architecture to boot real system
//! software, and `docs/platforms/riscv-virt.md` picks this board because it is
//! the smallest credible thing that does: a hart, a timer, an interrupt
//! controller, a serial port, and virtio for storage. No PCI, no ACPI, no
//! legacy — and every part of it specified in a document anybody can download.
//!
//! Neither the console nor virtio is **in** this module, and for the same
//! reason. The serial port is a 16550, which is nobody's board's chip, and it
//! lives in [`dev::uart::ns16550`](crate::dev::uart) where `pc64` and `q35`
//! reach it without linking a PLIC; virtio is an OASIS standard that names no
//! architecture, and it lives in [`dev::virtio`](crate::dev::virtio) where an
//! AArch64 board reaches it without linking a CLINT. What *is* board-specific
//! about each of them — the `ns16550a` and `virtio,mmio` nodes [`dt`]
//! describes them with — is a `dev-riscv` block in those files, exactly as
//! [`dev::flash::cfi`](crate::dev::flash)'s `cfi-flash` node is.
//!
//! | Module | Covers |
//! | --- | --- |
//! | [`clint`] | `mtime`, per-hart `mtimecmp`, software interrupts |
//! | [`plic`] | the platform-level interrupt controller: priority, enable, claim |
//! | [`syscon`] | the system controller a guest powers itself off through |
//! | [`dt`] | the device tree *generator*, which walks the realized machine |
//! | [`boot`] | the reset vector, and where the generated tree lands |
//! | [`loader`] | putting a firmware or kernel image into guest memory |
//!
//! # The board
//!
//! `machines/riscv-virt.machine` puts it together. The layout below is the
//! conventional one for a RISC-V `virt` board, and it is conventional rather
//! than required: the guest learns every address from the device tree we
//! generate, so nothing outside this repository fixes these numbers. They are
//! the familiar ones because familiar is easier to debug.
//!
//! ```text
//!   0x0000_1000  boot ROM: the reset vector, then the generated DTB
//!   0x0010_0000  system controller (poweroff, reboot)
//!   0x0200_0000  CLINT
//!   0x0c00_0000  PLIC
//!   0x1000_0000  16550 UART
//!   0x1000_1000  virtio-mmio (dev::virtio), one 4 KiB window each
//!   0x2000_0000  NOR flash bank 0: the firmware
//!   0x2200_0000  NOR flash bank 1: the UEFI variable store
//!   0x8000_0000  DRAM
//! ```
//!
//! The two flash banks are [`crate::dev::flash::cfi`] rather than anything in
//! this module: a CFI part is not a RISC-V device, and the board simply maps
//! two of them. What *is* board-specific is that they appear in the generated
//! tree as `cfi-flash` nodes, which is how a UEFI build finds its variable
//! store.
//!
//! # How far it boots
//!
//! Measured, not asserted (`ROADMAP.md` §0), and the runners are in
//! [`tests`](self#modules)'s sibling `tests.rs`:
//!
//! * **A bare-metal program** written straight to `0x80000000` prints to the
//!   console, takes a CLINT timer interrupt at `mtvec`, receives a keystroke
//!   through the UART, the PLIC and `meip`, and stops the machine through the
//!   system controller. No firmware, no operating system, no fetched fixture.
//! * **OpenSBI** (`fw_jump.bin`, BSD-2-Clause, fetched by
//!   `scripts/fetch-testdata.sh opensbi`) runs to completion on the generated
//!   device tree: it finds the timer as `aclint-mtimer @ 10000000Hz`, the
//!   console as `uart8250`, both `syscon-poweroff` and `syscon-reboot`, builds
//!   its domain from our memory map, and jumps to `0x80200000` in S-mode.
//! * **EDK2/UEFI** (`OvmfPkg/RiscVVirt`, BSD-2-Clause-Patent) boots out of the
//!   two NOR banks to `UEFI Interactive Shell v2.2` at a `Shell>` prompt:
//!   `gEfiVariableWriteArchProtocolGuid` installs against the flash, the DXE
//!   dispatcher completes, and BDS enumerates its boot options. A variable
//!   written in one run is in the store the next — the append-only variable log
//!   continues where the previous run left it rather than restarting, which is
//!   only possible on a part where a program clears bits and an erase costs a
//!   block. `docs/platforms/riscv-virt.md` has the numbers.
//! * **Linux** (a 6.12 `riscv64` Image behind OpenSBI) enters, parses the tree
//!   — `Hardware name: rsemu riscv-virt (DT)` — sets up memory, the RISC-V
//!   INTC, the SBI IPI and timer extensions and a 10 MHz clocksource, and
//!   prints its whole early log. It then live-locks in the timer path, and the
//!   reason is known and is not on this side of the boundary: the hart's `time`
//!   CSR is a field nothing advances, so `rdtime` reads zero and every deadline
//!   the kernel computes is already in the past. [`clint`] documents the gap
//!   and supplies the half of the fix that belongs to a CLINT.
//!
//! # Provenance
//!
//! Written from the RISC-V Privileged Architecture (CC-BY-4.0), the RISC-V PLIC
//! specification, the ACLINT specification, the National Semiconductor PC16550D
//! data sheet, the OASIS VIRTIO 1.2 specification and the Devicetree
//! Specification. Each module names the sections it used. No emulator source of
//! any licence was consulted, and in particular no virtio *driver* — which
//! `ROADMAP.md` §1 calls out as the most common way the rule gets broken.

pub mod boot;
pub mod clint;
pub mod dt;
pub mod loader;
pub mod plic;
pub mod syscon;

// The board-level tests need a hart and the machine layer to run on, so they
// come with `machine-riscv-virt` rather than with the devices alone.
#[cfg(all(test, feature = "machine-riscv-virt"))]
mod tests;

pub use boot::BootRom;
pub use clint::Clint;
pub use loader::Loader;
pub use plic::Plic;
pub use syscon::Syscon;

/// Add every board class to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is already claimed.
pub fn register(registry: &mut crate::core::Registry) -> crate::core::Result<()> {
    clint::register(registry)?;
    plic::register(registry)?;
    syscon::register(registry)?;
    boot::register(registry)?;
    loader::register(registry)
}

/// Bind every board class into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::Result<()> {
    clint::bind(bindings)?;
    plic::bind(bindings)?;
    syscon::bind(bindings)?;
    boot::bind(bindings)?;
    loader::bind(bindings)
}

/// Every board class's validator schema.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![
        clint::schema(),
        plic::schema(),
        syscon::schema(),
        boot::schema(),
        loader::schema(),
    ]
}
