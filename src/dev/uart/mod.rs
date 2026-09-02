//! Serial ports.
//!
//! One chip so far, and one feature per chip as everywhere else under
//! [`dev`](crate::dev):
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`ns16550`] | `dev-uart-ns16550` | a National Semiconductor 16550 with FIFOs, on the character-device seam |
//!
//! A serial port belongs to no board. This module exists because the 16550
//! spent its first year under `dev/riscv/`, where the `virt` board that first
//! wanted one had put it, and a PC that wanted a console had to link a PLIC, a
//! CLINT and virtio to get it — which is exactly what `CLAUDE.md`'s crate-shape
//! rule forbids. The class names did not change when it moved, so no machine
//! file did either.
//!
//! Not everything that shifts bytes down a wire lives here: a peripheral that
//! is part of an SoC stays with the SoC, because its register file is that
//! part's and not a standard. `dev::stm32`'s USART and `dev::wdc`'s W65C51N
//! ACIA are both that, and both stay where they are.

#[cfg(feature = "dev-uart-ns16550")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-uart-ns16550")))]
pub mod ns16550;

#[cfg(feature = "dev-uart-ns16550")]
pub use ns16550::Uart16550;
