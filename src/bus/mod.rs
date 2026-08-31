//! Bus fabrics.
//!
//! One module and one Cargo feature per fabric (`CLAUDE.md`, "Crate shape"), the
//! same rule `dev/` and `cpu/` follow. A fabric is the thing that carries
//! traffic between a controller and the peripherals hanging off it; the
//! peripherals themselves live under [`crate::dev`].
//!
//! This module always compiles, and is empty in a build with no bus features
//! enabled.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`spi`] | `bus-spi` | SPI: chip selects, the four modes, word size, bit order, full duplex — modelled transactionally *or* as clocked wires |
//! | [`usb`] | `bus-usb` | USB: devices, endpoints, the four transfer types, descriptors, enumeration, speeds and ports — controller-agnostic |
//!
//! Everything here is `no_std + alloc` and names no host facility.
//!
//! # What is deliberately *not* here: a pixel bus
//!
//! A TFT panel is fed over a parallel RGB link — pixel clock, HSYNC, VSYNC, DE
//! and twenty-four data lines — and there is no `bus/rgb` module modelling it,
//! on purpose. **No guest can observe a single one of those edges.** The guest
//! writes a framebuffer and programs a display controller; everything after
//! that is a geometry, a pixel format and a frame period. Simulating the link
//! would cost a great deal and change nothing observable, so
//! [`crate::dev::lcd`] reads the framebuffer straight out of the address space
//! and builds a [`crate::host::display::Surface`] from it. "Parallel RGB" is a
//! fact about the hardware, not a thing this tree models.
//!
//! SPI is the opposite case, and that is why it *is* here: a guest absolutely
//! can bit-bang SPI through GPIO and watch every edge.
//!
//! # Why a bus is not a `Device`
//!
//! `ROADMAP.md` §4 lists a generic `core::bus` module, and it does not exist
//! yet. Until it does, a fabric here is a plain object reached through a
//! **named rendezvous table** — [`spi::buses`], modelled on
//! [`crate::host::chardev::ports`], which solves the same problem for character
//! streams: two devices constructed independently have to find each other, and
//! a machine description can only hand them a *name*.
//!
//! That is a seam, and it is marked as one. When `core::bus` lands, the table
//! becomes the fabric's registry and every device-facing signature here stays
//! as it is.

#[cfg(feature = "bus-spi")]
#[cfg_attr(docsrs, doc(cfg(feature = "bus-spi")))]
pub mod spi;

#[cfg(feature = "bus-usb")]
#[cfg_attr(docsrs, doc(cfg(feature = "bus-usb")))]
pub mod usb;
