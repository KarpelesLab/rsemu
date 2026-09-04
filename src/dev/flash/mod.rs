//! Flash memory.
//!
//! Flash is the one kind of storage a board maps straight into its address
//! space and then *writes through the same window it executes from*, which is
//! why it belongs here rather than behind a block-device seam: the guest's view
//! of it is a memory map with a command protocol layered on top, not a disk.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`cfi`] | `dev-flash-cfi` | parallel NOR flash: the CFI query and the Intel/Sharp command set |
//! | [`spinor`] | `dev-flash-spinor` | serial NOR flash: a Winbond W25Q on the SPI bus |
//!
//! The two share their *semantics* — a program clears bits, an erase costs a
//! whole granule, and a read during a command answers with something other
//! than the array — and share nothing else, because a parallel part is an
//! address window and a serial part is a frame on four wires.
//!
//! `no_std + alloc`, no dependencies. The contents arrive through a **media
//! slot** — the same seam a firmware image or a NES cartridge comes in on — and
//! never through `fstool` directly: a NOR part's contents are a flat image with
//! no partition table, no filesystem and no sector geometry, and parsing one
//! here would drag `std` into a `no_std` device to gain nothing (`CLAUDE.md`,
//! "`no_std`").
//!
//! [`cfi`] also takes a [`Medium`](crate::dev::medium::Medium) bound to that
//! same slot name, because **a part a guest writes is a storage device**. That
//! is what makes a UEFI variable survive a reboot rather than a run:
//! `--drive flash1=OVMF_VARS.fd` puts the bank's bytes in a host file, and
//! `Device::flush` writes them back when the run ends. The seam is
//! `dev::medium`, so a snapshot obeys the same [`Snapshot`] policy every drive
//! in the tree does.
//!
//! [`Snapshot`]: crate::dev::medium::Snapshot

#[cfg(feature = "dev-flash-cfi")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-flash-cfi")))]
pub mod cfi;
#[cfg(feature = "dev-flash-spinor")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-flash-spinor")))]
pub mod spinor;

#[cfg(feature = "dev-flash-cfi")]
pub use cfi::Cfi;
#[cfg(feature = "dev-flash-spinor")]
pub use spinor::SpiNor;

/// Add every flash class to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is already claimed.
pub fn register(registry: &mut crate::core::Registry) -> crate::core::Result<()> {
    // At least one of the two is on, or this module would not be compiled.
    #[cfg(feature = "dev-flash-cfi")]
    cfi::register(registry)?;
    #[cfg(feature = "dev-flash-spinor")]
    spinor::register(registry)?;
    Ok(())
}

/// Bind every flash class into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::Result<()> {
    #[cfg(feature = "dev-flash-cfi")]
    cfi::bind(bindings)?;
    #[cfg(feature = "dev-flash-spinor")]
    spinor::bind(bindings)?;
    Ok(())
}

/// Every flash class's validator schema.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    let mut out = alloc::vec::Vec::new();
    #[cfg(feature = "dev-flash-cfi")]
    {
        out.push(cfi::schema());
    }
    #[cfg(feature = "dev-flash-spinor")]
    {
        out.push(spinor::schema());
    }
    out
}
