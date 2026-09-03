//! USB host controllers and USB device models.
//!
//! The bus itself — devices, endpoints, transfers, descriptors, enumeration —
//! is [`crate::bus::usb`], and nothing here is in its signatures. What lives in
//! this module is the two ends that hang off it: the **host controllers** that
//! turn a schedule in guest memory into transactions, and the **device models**
//! that answer them.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`ehci`] | `dev-usb-ehci` | a generic EHCI host controller: the register file of EHCI 1.0 and the QH/qTD schedule walker that DMA-reads it out of guest RAM |
//! | [`dwc2`] | `dev-usb-dwc2` | a Synopsys DesignWare USB 2.0 OTG controller — STM32's OTG_FS — with host channels and a shared FIFO instead of a schedule in guest memory, **in both roles**: host, and device, where the guest is the peripheral |
//! | [`chipidea`] | `dev-usb-chipidea` | the ChipIdea/ARC dual-role variant of the same controller: a `+0x140` operational offset, an `ID` register and a `USBMODE` role select |
//! | [`hid`] | `dev-usb-hid` | a USB HID boot-protocol mouse: the smallest device that proves the stack |
//! | [`hub`] | `dev-usb-hub` | a USB 2.0 hub: downstream ports, the class requests of §11.24.2 and a status change endpoint — the device that makes the bus a tree |
//! | [`xhci`] | `dev-usb-xhci` | an xHCI host controller: rings and contexts in guest RAM instead of linked lists, a command ring, an event ring with an ERST, and one interrupter |
//! | [`xhci::pci`] | `dev-usb-xhci-pci` | the same controller as a **PCI function**: class code `0C0330h`, its register block behind a base address register, `INTA#` onto the fabric's shared net — the attachment that lets a PC board carry a USB port at all |
//! | [`msd`] | `dev-usb-msd` | a USB mass storage device: Bulk-Only Transport and a SCSI command set over two bulk endpoints, backed by the same [`Medium`](crate::dev::medium::Medium) an ATA drive or an NVMe namespace reads |
//!
//! # The layering, and why it is the point
//!
//! [`chipidea`] contains **no schedule walker, no qTD decoding and no DMA**.
//! All of that is [`ehci::Hcd`], which is the controller as EHCI 1.0 defines
//! it; the ChipIdea module is a register map that hands the same engine the
//! same writes at different offsets, plus the handful of registers that are the
//! vendor's own. That is the test of whether the split is real: the next
//! controller that embeds an EHCI core — and there are many — is another
//! register map and nothing else.
//!
//! [`xhci`] is the other half of the same claim, from the far side. It shares
//! *nothing* with [`ehci`] — rings and contexts instead of linked lists, a Cycle
//! bit instead of an Active bit, an event ring the controller produces instead
//! of a status field it writes back — and it still needed no change to
//! [`crate::bus::usb`], because the seam there is a transaction and a
//! transaction is a transaction whatever built the schedule. [`msd`]'s disk
//! answers both without knowing which is asking.
//!
//! # Both directions, and what that cost the fabric
//!
//! [`dwc2`]'s device mode is the first thing in this tree where the *guest* is
//! the peripheral: a host somewhere else issues a `GET_DESCRIPTOR` and guest
//! firmware answers it out of its own endpoint FIFO. It is a
//! [`UsbDevice`](crate::bus::usb::UsbDevice) and **not** a
//! [`Peripheral`](crate::bus::usb::Peripheral), because a `Peripheral` answers
//! USB 2.0 §9.4 inside the emulator and that is precisely the job the guest is
//! there to do. What [`crate::bus::usb`] had to grow for it was two additive
//! things — a start-of-frame broadcast and a host-side transfer composer — plus
//! one comment that had become untrue. All three are argued where they live.
//!
//! Everything here is `no_std + alloc` and names no host facility.

#[cfg(feature = "dev-usb-chipidea")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-usb-chipidea")))]
pub mod chipidea;

#[cfg(feature = "dev-usb-dwc2")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-usb-dwc2")))]
pub mod dwc2;

#[cfg(feature = "dev-usb-ehci")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-usb-ehci")))]
pub mod ehci;

#[cfg(feature = "dev-usb-hid")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-usb-hid")))]
pub mod hid;

#[cfg(feature = "dev-usb-hub")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-usb-hub")))]
pub mod hub;

#[cfg(feature = "dev-usb-msd")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-usb-msd")))]
pub mod msd;

#[cfg(feature = "dev-usb-xhci")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-usb-xhci")))]
pub mod xhci;

/// Add every USB class this build has to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> crate::Result<()> {
    #[cfg(feature = "dev-usb-ehci")]
    ehci::register(registry)?;
    #[cfg(feature = "dev-usb-chipidea")]
    chipidea::register(registry)?;
    #[cfg(feature = "dev-usb-dwc2")]
    dwc2::register(registry)?;
    #[cfg(feature = "dev-usb-hid")]
    hid::register(registry)?;
    #[cfg(feature = "dev-usb-hub")]
    hub::register(registry)?;
    #[cfg(feature = "dev-usb-msd")]
    msd::register(registry)?;
    #[cfg(feature = "dev-usb-xhci")]
    xhci::register(registry)?;
    #[cfg(feature = "dev-usb-xhci-pci")]
    xhci::pci::register(registry)?;
    let _ = registry;
    Ok(())
}

/// Bind every USB class this build has into the machine graph.
///
/// # Errors
///
/// As [`register`].
pub fn bind(bindings: &mut crate::machine::Bindings) -> crate::Result<()> {
    #[cfg(feature = "dev-usb-ehci")]
    ehci::bind(bindings)?;
    #[cfg(feature = "dev-usb-chipidea")]
    chipidea::bind(bindings)?;
    #[cfg(feature = "dev-usb-dwc2")]
    dwc2::bind(bindings)?;
    #[cfg(feature = "dev-usb-hid")]
    hid::bind(bindings)?;
    #[cfg(feature = "dev-usb-hub")]
    hub::bind(bindings)?;
    #[cfg(feature = "dev-usb-msd")]
    msd::bind(bindings)?;
    #[cfg(feature = "dev-usb-xhci")]
    xhci::bind(bindings)?;
    #[cfg(feature = "dev-usb-xhci-pci")]
    xhci::pci::bind(bindings)?;
    let _ = bindings;
    Ok(())
}

/// What the validator should know about this build's USB classes.
// One `#[cfg]`-gated push per class, which is what the lint is complaining
// about: a `vec![]` literal cannot carry an attribute on one of its elements,
// so the push form is the only one that expresses "this class exists only in
// some builds". `machine::catalog` carries the same note for the same reason.
#[must_use]
#[allow(unused_mut, clippy::vec_init_then_push)]
pub fn schemas() -> alloc::vec::Vec<crate::machine::validate::ClassSchema> {
    let mut out = alloc::vec::Vec::new();
    #[cfg(feature = "dev-usb-ehci")]
    out.push(ehci::schema());
    #[cfg(feature = "dev-usb-chipidea")]
    out.push(chipidea::schema());
    #[cfg(feature = "dev-usb-dwc2")]
    out.push(dwc2::schema());
    #[cfg(feature = "dev-usb-hid")]
    out.push(hid::schema());
    #[cfg(feature = "dev-usb-hub")]
    out.push(hub::schema());
    #[cfg(feature = "dev-usb-msd")]
    out.push(msd::schema());
    #[cfg(feature = "dev-usb-xhci")]
    out.push(xhci::schema());
    #[cfg(feature = "dev-usb-xhci-pci")]
    out.push(xhci::pci::schema());
    out
}
