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
    out
}
