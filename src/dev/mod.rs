//! Device models.
//!
//! Everything here is feature-gated, one feature per device (`CLAUDE.md`,
//! "Crate shape"): a NES build links a cartridge, a 6502 and the NES chips, and
//! nothing else. The core ([`crate::core`]) is what they are all written
//! against, and no type in this tree may appear in a `core::` signature
//! (`ROADMAP.md` §0, "generic first, specific second").
//!
//! This module always compiles, and is empty in a build with no device
//! features enabled.
//!
//! | Module | Feature | Covers |
//! | --- | --- | --- |
//! | [`apple1`] | `dev-apple1` | the Apple 1's MC6821, its monitor ROM socket, and RSMON |
//! | [`ata`] | `dev-ata-disk` | an ATA hard disk: the command block, the command set, CHS and LBA |
//! | [`apu`] | `dev-nes-apu` | the RP2A03 audio half: channels, frame counter, DMC |
//! | [`cart`] | `dev-nes-cart` | cartridge images and the mappers that decode them |
//! | [`flash`] | `dev-flash-cfi`, `dev-flash-spinor` | NOR flash: parallel (CFI) and serial (W25Q on SPI) |
//! | [`nes`] | `dev-nes-io` | the console's own I/O: controller ports, OAM DMA |
//! | [`net`] | `dev-net`, `dev-ne2000`, `net-pktkit` | the network seam, an NE2000 card, and the `pktkit` bridge |
//! | [`pc`] | `dev-pc` | an IBM PC/AT board's chips: 8259A, 8254, 8042, MC146818, 8237A, the firmware socket |
//! | [`ppu`] | `dev-nes-ppu` | the RP2C02 picture unit: the per-dot pipeline |
//! | [`lcd`] | `dev-lcdc` | a generic RGB scanout engine: framebuffer in, `Scanout` out |
//! | [`sitronix`] | `dev-st7272a` | the ST7272A TFT panel driver: SPI register configuration, no pixel path |
//! | [`stm32`] | `dev-stm32` | STM32 peripherals: a GPIO port and a USART |
//! | [`riscv`] | `dev-riscv` | the RISC-V `virt` board: CLINT, PLIC, 16550, virtio, and the device tree generator |
//! | [`sd`] | `dev-sd-card` | an SD memory card: the command set, the state machine, the registers |
//! | [`usb`] | `dev-usb-*` | USB host controllers — a generic EHCI, the ChipIdea/ARC variant over it, and a Synopsys dwc2 that shares nothing with either — and a HID mouse |
//! | [`wdc`] | `dev-wdc` | the W65C51N ACIA and W65C22 VIA, and a 6502 board's ROM |
//!
//! Most of `dev/` is `no_std + alloc`, and that includes all of [`net`] except
//! one file: `dev/net/pktkit.rs` is the `std` half of the exception
//! `ROADMAP.md` §0 grants `dev/net/*`, because `pktkit` is a `std` crate. The
//! other documented exception, `dev/blk/*` for `fstool`, does not exist yet.

#[cfg(feature = "dev-at24c")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-at24c")))]
pub mod atmel;

#[cfg(feature = "dev-apple1")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-apple1")))]
pub mod apple1;

#[cfg(feature = "dev-nes-apu")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-apu")))]
pub mod apu;

#[cfg(feature = "dev-ata-disk")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-ata-disk")))]
pub mod ata;

#[cfg(feature = "dev-nes-cart")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-cart")))]
pub mod cart;

#[cfg(feature = "dev-gb")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-gb")))]
pub mod gb;

#[cfg(any(feature = "dev-flash-cfi", feature = "dev-flash-spinor"))]
#[cfg_attr(
    docsrs,
    doc(cfg(any(feature = "dev-flash-cfi", feature = "dev-flash-spinor")))
)]
pub mod flash;

#[cfg(feature = "dev-nes-io")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-io")))]
pub mod nes;

#[cfg(feature = "dev-net")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-net")))]
pub mod net;

#[cfg(feature = "dev-pc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-pc")))]
pub mod pc;

#[cfg(feature = "dev-nes-ppu")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-nes-ppu")))]
pub mod ppu;

#[cfg(feature = "dev-lcdc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-lcdc")))]
pub mod lcd;

#[cfg(feature = "dev-st7272a")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-st7272a")))]
pub mod sitronix;

#[cfg(feature = "dev-sms")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-sms")))]
pub mod sms;

#[cfg(any(
    feature = "dev-stm32",
    feature = "dev-stm32-sdmmc",
    feature = "dev-stm32-spi",
    feature = "dev-stm32-octospi",
    feature = "dev-stm32-i2c",
    feature = "machine-spi-flash"
))]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-stm32")))]
pub mod stm32;

#[cfg(feature = "dev-riscv")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-riscv")))]
pub mod riscv;

#[cfg(feature = "dev-sd-card")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-sd-card")))]
pub mod sd;

#[cfg(any(
    feature = "dev-usb-ehci",
    feature = "dev-usb-chipidea",
    feature = "dev-usb-dwc2",
    feature = "dev-usb-hid"
))]
#[cfg_attr(
    docsrs,
    doc(cfg(any(
        feature = "dev-usb-ehci",
        feature = "dev-usb-chipidea",
        feature = "dev-usb-dwc2",
        feature = "dev-usb-hid"
    )))
)]
pub mod usb;

#[cfg(feature = "dev-wdc")]
#[cfg_attr(docsrs, doc(cfg(feature = "dev-wdc")))]
pub mod wdc;
