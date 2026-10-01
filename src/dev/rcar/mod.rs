//! Renesas on-chip peripherals of the SH-4A / R-Car lineage.
//!
//! Renesas carried a handful of blocks from the SH-3 and SH-4A into R-Car
//! almost unchanged — the same register offsets, the same flag semantics — so
//! they are modelled once here, as classes a board places at the addresses its
//! own manual gives, rather than under any one SoC. Nothing here knows which
//! SoC it is on; the R-Car H1 (R8A7779) board is the first user.
//!
//! | Module | Class | Covers |
//! | --- | --- | --- |
//! | [`scif`] | `rcar.scif` | the SCIF (16-byte FIFOs) and, by `variant`, the HSCIF (128-byte FIFOs), on the character-device seam |
//! | [`tmu`] | `rcar.tmu` | one TMU: three 32-bit down-counters on the prescaled peripheral clock, lazily advanced |
//! | [`gpio`] | `rcar.gpio` | one GPIO bank: 32 pins as wires both ways, the output latch, edge and level interrupts |
//! | [`i2c`] | `rcar.i2c` | one I2C controller: a master on a named I2C bus, interrupt-driven |
//! | [`hspi`] | `rcar.hspi` | one HSPI channel: an SPI master with 8-byte FIFOs, by programmed I/O, on a named SPI bus |
//! | [`hpbdmac`] | `rcar.hpbdmac` | the HPB-DMAC: 44 request-paced peripheral DMA channels with double-buffered register sets |
//! | [`sdhi`] | `rcar.sdhi` | the SD host interface: command engine, one-block buffer, DMA requests, driving a `sd.card` through a socket |
//! | [`sysc`] | `rcar.sysc` | the SYSC power domains: switches that complete at once, with status and completion bits |
//! | [`du`] | `rcar.du` | the R-Car Display Unit: timing registers, up to eight planes composed from guest memory, a frame counter on the dot clock |
//!
//! The SCIF and TMU are written from the Renesas hardware manuals (the
//! SH7780's SCIF and TMU chapters, and the R-Car SCIF/HSCIF chapters), cited
//! by section in each file; the DU from the R-Car manual's DU chapter and a
//! black-box trace of a real driver's register writes, as [`du`] records. No
//! emulator, kernel or boot loader source was consulted.

use alloc::vec::Vec;

use crate::core::error::Result;
use crate::machine::validate::ClassSchema;

pub mod du;
pub mod gpio;
pub mod hpbdmac;
pub mod hspi;
pub mod i2c;
pub mod scif;
pub mod sdhi;
pub mod sysc;
pub mod tmu;

pub use du::Du;
pub use gpio::Gpio;
pub use hpbdmac::HpbDmac;
pub use hspi::Hspi;
pub use i2c::I2c;
pub use scif::Scif;
pub use sdhi::Sdhi;
pub use sysc::Sysc;
pub use tmu::Tmu;

/// Add every class here to a registry.
///
/// # Errors
///
/// If something already claimed one of the names.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    gpio::register(registry)?;
    hpbdmac::register(registry)?;
    hspi::register(registry)?;
    i2c::register(registry)?;
    scif::register(registry)?;
    sdhi::register(registry)?;
    sysc::register(registry)?;
    tmu::register(registry)?;
    du::register(registry)
}

/// Bind every class here into the machine graph.
///
/// # Errors
///
/// If one of the names is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    gpio::bind(bindings)?;
    hpbdmac::bind(bindings)?;
    hspi::bind(bindings)?;
    i2c::bind(bindings)?;
    scif::bind(bindings)?;
    sdhi::bind(bindings)?;
    sysc::bind(bindings)?;
    tmu::bind(bindings)?;
    du::bind(bindings)
}

/// Every class's validator schema.
#[must_use]
pub fn schemas() -> Vec<ClassSchema> {
    alloc::vec![
        gpio::schema(),
        hpbdmac::schema(),
        hspi::schema(),
        i2c::schema(),
        scif::schema(),
        sdhi::schema(),
        sysc::schema(),
        tmu::schema(),
        du::schema()
    ]
}
