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
//!
//! `no_std + alloc`, no dependencies. The backing store is a **media slot**,
//! the same seam a firmware image or a NES cartridge arrives through, rather
//! than `fstool`: the contents of a NOR part are a flat image with no partition
//! table, no filesystem and no sector geometry, and binding one through the
//! disk-image crate would drag `std` into a `no_std` device to gain nothing
//! (`CLAUDE.md`, "`no_std`").

pub mod cfi;

pub use cfi::Cfi;

/// Add every flash class to a registry.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if a name is already claimed.
pub fn register(registry: &mut crate::core::Registry) -> crate::core::Result<()> {
    cfi::register(registry)
}

/// Bind every flash class into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::core::Result<()> {
    cfi::bind(bindings)
}

/// Every flash class's validator schema.
#[must_use]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    alloc::vec![cfi::schema()]
}
