//! What this build can emulate: its device classes and its shipped machines.
//!
//! A machine is a feature set (`ROADMAP.md` §3), so "which classes exist?" and
//! "which machines can this binary run?" are build-specific questions with
//! honest answers. This module is where they are answered, once, for the CLI,
//! the wasm shim, the tests and `rsemu describe` alike — three copies of a
//! registration list would drift apart on the first new device.
//!
//! # Registration is explicit
//!
//! One `#[cfg(feature = …)]` arm per component, calling that component's own
//! `register` / `bind` / `schema` (§4.4). No link-time magic, no inventory
//! crate: a class that is not named here is not in the build, and that is
//! visible by reading the file.
//!
//! # Three tables, not one
//!
//! * [`registry`] — construction, and the `rsemu devices` listing. The table
//!   of record.
//! * [`bindings`] — the classes that take part in the memory map and the wire
//!   graph. See [`Bindings`] for why this is still separate.
//! * [`classes`] — what the *validator* checks a machine file against. It
//!   cannot be derived from the registry: `DeviceClass` declares a class's
//!   properties but not its pins or its mappable regions, so a table built
//!   from the registry alone would reject `map cpubus 0x8000 = cart.prg`.
//!
//! [`registry`]: registry()
//! [`bindings`]: bindings()
//! [`classes`]: classes()
//!
//! # The machine catalog
//!
//! `machines/*.machine` ships as **data** — a user copies one and edits it —
//! and the files this build knows how to realize are also compiled in, so
//! `rsemu run nes-ntsc` works from any directory and a wasm build has them
//! without a filesystem. A path on the command line still wins: the catalog is
//! a default, not a jail.

use alloc::string::String;
use alloc::vec::Vec;

use crate::core::error::Result;
use crate::core::registry::Registry;
use crate::machine::builtin;
use crate::machine::realize::Bindings;
use crate::machine::validate::ClassTable;
use crate::machine::{BuildOptions, Machine};

/// One machine description this build ships.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CatalogEntry {
    /// The name `rsemu run <name>` takes, and the file's stem under
    /// `machines/`.
    pub name: &'static str,
    /// One line for `rsemu machines`.
    pub summary: &'static str,
    /// Media slots the machine will not realize without, as
    /// `rsemu run … --<slot> <file>` spells them.
    pub media: &'static [&'static str],
    /// The description text itself.
    pub source: &'static str,
}

/// The NTSC NES, when this build has a 6502 and a cartridge to put in it.
#[cfg(feature = "machine-nes")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-nes")))]
pub static NES_NTSC: CatalogEntry = CatalogEntry {
    name: "nes-ntsc",
    summary: "Nintendo Entertainment System / Famicom, NTSC (RP2C02 at 60 Hz)",
    media: &["cart"],
    source: include_str!("../../machines/nes-ntsc.machine"),
};

/// The PAL NES: the same board with a different crystal and different
/// dividers.
///
/// Its own file rather than a parameter of the NTSC one. The region changes the
/// oscillator, both clock dividers and the PPU's frame geometry at once, and
/// §4.2 makes the oscillator topology part of the machine's identity — a
/// snapshot records it, and two machines that differ in it are not the same
/// machine wearing a flag.
#[cfg(feature = "machine-nes")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-nes")))]
pub static NES_PAL: CatalogEntry = CatalogEntry {
    name: "nes-pal",
    summary: "Nintendo Entertainment System, PAL (RP2C07 at 50 Hz, 312 scanlines)",
    media: &["cart"],
    source: include_str!("../../machines/nes-pal.machine"),
};

/// The Apple 1, when this build has a 6502 and the board's chips.
///
/// The `rom` slot has a default nothing else does: `rsemu run apple1` with no
/// `--rom` binds [`RSMON`](crate::dev::apple1::RSMON), rsemu's own monitor, so
/// the machine demonstrates itself without a ROM of unclear provenance.
#[cfg(feature = "machine-apple1")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-apple1")))]
pub static APPLE1: CatalogEntry = CatalogEntry {
    name: "apple1",
    summary: "Apple 1 (1976): 6502, 4K RAM, MC6821 keyboard and display",
    media: &["rom"],
    source: include_str!("../../machines/apple1.machine"),
};

/// Ben Eater's 6502 breadboard computer, when this build has a 6502 and the
/// board's chips.
///
/// The `rom` slot has a default: `rsemu run beneater-6502` with no `--rom`
/// binds [`RSMON_IMAGE`](crate::dev::wdc::RSMON_IMAGE), rsemu's own monitor,
/// and `--monitor wozmon` binds the 1976 Woz Monitor instead.
#[cfg(feature = "machine-beneater")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-beneater")))]
pub static BENEATER_6502: CatalogEntry = CatalogEntry {
    name: "beneater-6502",
    summary: "Ben Eater's 6502 breadboard computer: 1 MHz, 16K RAM, 65C51 serial, 65C22",
    media: &["rom"],
    source: include_str!("../../machines/beneater-6502.machine"),
};

/// The RISC-V `virt` board, when this build has a hart and the board's chips.
///
/// The first machine rsemu boots real system software on. The `firmware` slot
/// takes whatever should be at `0x80000000`: a bare-metal image, or OpenSBI,
/// or OpenSBI with a kernel payload attached. The device tree the guest is
/// handed is generated from the realized machine rather than shipped
/// (`docs/platforms/riscv-virt.md`).
#[cfg(feature = "machine-riscv-virt")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-riscv-virt")))]
pub static RISCV_VIRT: CatalogEntry = CatalogEntry {
    name: "riscv-virt",
    summary: "RISC-V `virt`: RV64GC hart, CLINT, PLIC, 16550, virtio-MMIO, NOR flash, DTB",
    // Only `firmware` is needed to come up. The rest are listed because they
    // are what a particular guest wants — the NOR banks for a UEFI build, the
    // ramdisk for a kernel that has to find a root filesystem, the disk image
    // for one that reads a real one — and unbound means the empty version of
    // each: blank flash, no initrd, a disk of zeroes. `rsemu run` binds them
    // empty so nobody has to say so.
    media: &["firmware", "flash0", "flash1", "initrd", "disk"],
    source: include_str!("../../machines/riscv-virt.machine"),
};

/// The Game Boy, when this build has an SM83 and the console's chips.
///
/// Phase 4's genericity proof (`ROADMAP.md` §13): a machine that is not
/// NES-shaped, realized by the same core.
#[cfg(feature = "machine-gameboy")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-gameboy")))]
pub static GAMEBOY: CatalogEntry = CatalogEntry {
    name: "gameboy",
    summary: "Nintendo Game Boy (DMG, 1989): SM83 at 4.194304 MHz, 160x144 LCD",
    media: &["cart"],
    source: include_str!("../../machines/gameboy.machine"),
};

/// The NTSC Master System, when this build has a Z80 and the console's chips.
///
/// The other half of phase 4's genericity proof (`ROADMAP.md` §13): a Z80 with
/// a **separate I/O address space**, a VDP and control pads that live in it
/// rather than in memory, and one address that is the sound chip on a write and
/// the VDP's counters on a read.
#[cfg(feature = "machine-sms")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-sms")))]
pub static SMS_NTSC: CatalogEntry = CatalogEntry {
    name: "sms-ntsc",
    summary: "Sega Master System (NTSC): Z80 at 3.58 MHz, 315-5124 VDP, SN76489, 262 lines",
    media: &["cart"],
    source: include_str!("../../machines/sms-ntsc.machine"),
};

/// The PAL Master System.
///
/// A second file rather than a parameter, for the same reason `nes-pal` is: the
/// region changes the oscillator *and* the frame — 313 lines against 262 — and
/// `ROADMAP.md` §4.2 makes that part of the machine's identity.
#[cfg(feature = "machine-sms")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-sms")))]
pub static SMS_PAL: CatalogEntry = CatalogEntry {
    name: "sms-pal",
    summary: "Sega Master System (PAL): Z80 at 3.55 MHz, 315-5246 VDP, SN76489, 313 lines",
    media: &["cart"],
    source: include_str!("../../machines/sms-pal.machine"),
};

/// The `spi-panel` board, when this build has a hart, the SPI bus and the
/// display devices.
///
/// A synthetic board rather than a product: the smallest machine that exercises
/// a whole display path, and the place the SPI bus and the ST7272A are actually
/// run. The `firmware` slot takes whatever should be at `0x00000000`, where the
/// hart resets to.
#[cfg(feature = "machine-spi-panel")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-spi-panel")))]
pub static SPI_PANEL: CatalogEntry = CatalogEntry {
    name: "spi-panel",
    summary: "a minimal SoC-shaped board: RV32, SPI, a Sitronix ST7272A panel and a scanout engine",
    media: &["firmware"],
    source: include_str!("../../machines/spi-panel.machine"),
};

/// The `spi-flash` board, when this build has a hart, the SPI bus and the
/// serial-flash devices.
///
/// A synthetic board rather than a product: the smallest machine that drives a
/// serial flash *both* ways — indirectly, a byte at a time through an STM32 SPI
/// master, and by executing straight out of an OCTOSPI memory-mapped window.
/// The `firmware` slot takes whatever should be at the hart's reset vector.
#[cfg(feature = "machine-spi-flash")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-spi-flash")))]
pub static SPI_FLASH: CatalogEntry = CatalogEntry {
    name: "spi-flash",
    summary: "a serial-flash board: RV32, an STM32 SPI master, an OCTOSPI window and a W25Q part",
    media: &["firmware"],
    source: include_str!("../../machines/spi-flash.machine"),
};

/// A bare ARM926EJ-S SoC skeleton, when this build has an A-profile core.
///
/// A synthetic board rather than a product: a boot ROM at the reset vector,
/// DRAM, and one peripheral aperture, with every address a parameter because an
/// ARM9 SoC's memory map belongs to the SoC and not to the architecture. It is
/// the starting point a downstream part copies and edits — the immediate one
/// being a Conexant DigiColor CX92755-class SoC, whose peripherals sit at
/// `0xf0000000`. The `firmware` slot takes whatever should be at `0x00000000`,
/// where the core resets to.
#[cfg(feature = "machine-arm926")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-arm926")))]
pub static ARM926: CatalogEntry = CatalogEntry {
    name: "arm926",
    summary: "a bare ARM926EJ-S SoC skeleton: ARMv5TE core, boot ROM, DRAM, one peripheral window",
    media: &["firmware"],
    source: include_str!("../../machines/arm926.machine"),
};

/// The PC/AT, when this build has an x86 core and the board's chips.
///
/// Held out of the catalog until now for one reason: the x86 core was registered
/// but not bound, so a machine file could not hand it an address space or wire
/// an interrupt to it. It can, so the board is here.
///
/// **No firmware is shipped and none will be.** The `bios` slot takes an image
/// the user supplies; `vgabios` and `floppy` likewise. A PC with no BIOS is a
/// board that realizes and executes open bus, which is a useful thing to be
/// able to look at and not a machine that boots.
///
/// `hd0` and `hd1` are the two drive bays on the primary IDE channel, and an
/// unbound one is an empty bay rather than an error: a PC with no hard disk is
/// an ordinary PC.
#[cfg(feature = "machine-pc-at")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-pc-at")))]
pub static PC_AT: CatalogEntry = CatalogEntry {
    name: "pc-at",
    summary: "IBM PC/AT-class board: x86, 8259As, 8254, MC146818, 8042, 8237s, VGA, floppy, IDE",
    media: &["bios", "vgabios", "floppy", "hd0", "hd1"],
    source: include_str!("../../machines/pc-at.machine"),
};

/// The same lineage stripped to its interrupt path: an x86, the two 8259As, a
/// local APIC, an I/O APIC and an HPET, and almost nothing else.
///
/// `pc-at` carries all of that too, so this is not "the board with an APIC" —
/// it is the board with *only* the parts an APIC question involves. No video,
/// no disks, no PCI, no DMA: a machine for `tests/pc_apic.rs` and
/// `tests/pc_apic_smp.rs` to reason about a redirection entry on, where a
/// failure names the interrupt controller rather than whatever else happened to
/// be on the bus. The one structural difference from `pc-at` is that the master
/// 8259A's `INT` reaches `LINT0` directly rather than through an IMCR, so this
/// board is in virtual-wire mode from the first instruction — which is the
/// state an APIC test wants and the state a DOS-era firmware cannot boot in.
///
/// **No firmware is shipped.** The `bios` slot takes the user's own image.
#[cfg(feature = "machine-pc-apic")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-pc-apic")))]
pub static PC_APIC: CatalogEntry = CatalogEntry {
    name: "pc-apic",
    summary: "PC/AT interrupt path with the APIC fitted: x86, 8259As, local APIC, I/O APIC, HPET",
    media: &["bios"],
    source: include_str!("../../machines/pc-apic.machine"),
};

/// An STM32F407VGT6 microcontroller, when this build has a Cortex-M core.
///
/// A real part rather than a synthetic board: the microcontroller on ST's own
/// STM32F4 Discovery. It is where an M-profile core is exercised through a
/// machine file, and where the answer to "how does a peripheral raise an
/// interrupt on a core whose NVIC is *inside* it" is written down — one line,
/// `wire usart2.irq -> cpu.irq38`, with the number out of RM0090's vector
/// table. The `firmware` slot takes the flash image the core boots from.
#[cfg(feature = "machine-stm32f407")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-stm32f407")))]
pub static STM32F407: CatalogEntry = CatalogEntry {
    name: "stm32f407",
    summary: "STM32F407VGT6: Cortex-M4 at 168 MHz, 1 MiB flash, three SRAM banks, six GPIO ports, USART2",
    // Only `firmware` — the flash image, which the core fetches its initial
    // `SP` and `PC` out of through the boot alias at zero.
    media: &["firmware"],
    source: include_str!("../../machines/stm32f407.machine"),
};

/// A minimal Z80 board, when this build has a Z80.
///
/// A synthetic board rather than a product: ROM at the reset vector, RAM above
/// it, and a second address space for the 64 KiB of ports `IN` and `OUT` reach.
/// It is where the Z80's separate I/O space is actually exercised through a
/// machine file. The `firmware` slot takes whatever should be at `0x0000`.
#[cfg(feature = "machine-z80-mini")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-z80-mini")))]
pub static Z80_MINI: CatalogEntry = CatalogEntry {
    name: "z80-mini",
    summary: "a minimal Z80 board: ROM, RAM, and the separate 64 KiB port space IN and OUT reach",
    media: &["firmware"],
    source: include_str!("../../machines/z80-mini.machine"),
};

/// A minimal Z80 board with an NE2000 on its port bus, when this build has
/// both.
///
/// A synthetic board rather than a product: it exists so that `net.ne2000` has
/// somewhere a real driver can run, executing `IN` and `OUT` through the
/// machine's own port space and taking the card's interrupt on `/INT`. The
/// `firmware` slot takes whatever should be at `0x0000`.
#[cfg(feature = "machine-ne2k-mini")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-ne2k-mini")))]
pub static NE2K_MINI: CatalogEntry = CatalogEntry {
    name: "ne2k-mini",
    summary: "a minimal Z80 board with an NE2000 Ethernet card on its port bus",
    media: &["firmware"],
    source: include_str!("../../machines/ne2k-mini.machine"),
};

/// A minimal PCI board with a Serial ATA host bus adapter on it, when this
/// build has one.
///
/// A synthetic board rather than a product: RAM, a host bridge for the
/// configuration ports, an 8259A for the completion interrupt, an AHCI adapter,
/// and one ATA drive in its port 0 bay. It exists so that `ahci.hba` has
/// somewhere a driver can reach it the way a driver does — through
/// configuration cycles, an `ABAR` it places itself, and a command list in the
/// board's own RAM. The `sata0` slot takes the drive's contents.
#[cfg(feature = "machine-ahci-mini")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-ahci-mini")))]
pub static AHCI_MINI: CatalogEntry = CatalogEntry {
    name: "ahci-mini",
    summary: "a minimal PCI board with an AHCI adapter and one SATA drive: RAM, a host bridge, \
              an 8259A",
    media: &["sata0"],
    source: include_str!("../../machines/ahci-mini.machine"),
};

/// A minimal PCI board with an NVM Express controller on it, when this build
/// has one.
///
/// A synthetic board rather than a product: RAM, a host bridge for the
/// configuration ports, an 8259A for the completion interrupt, and the
/// controller. It exists so that `nvme.controller` has somewhere a driver can
/// reach it the way a driver does — through configuration cycles, a base
/// address register it places itself, and queues in the board's own RAM. The
/// `nvme0` slot takes the namespace's contents.
#[cfg(feature = "machine-nvme-mini")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-nvme-mini")))]
pub static NVME_MINI: CatalogEntry = CatalogEntry {
    name: "nvme-mini",
    summary: "a minimal PCI board with an NVM Express controller: RAM, a host bridge, an 8259A",
    media: &["nvme0"],
    source: include_str!("../../machines/nvme-mini.machine"),
};

/// The smallest board a real USB storage driver can run on, when this build has
/// an EHCI and a USB disk.
///
/// A synthetic board rather than a product: a RISC-V hart, RAM, a PLIC, an EHCI
/// host controller and a `usb.storage` device in its one root port. It exists so
/// that `usb.storage` has somewhere a driver can reach it the way a driver does
/// — reset the port, enumerate over the default pipe, then push a Command Block
/// Wrapper out of a bulk endpoint and pull a sector and a Command Status Wrapper
/// back in, with the controller's completion interrupt travelling a real wire
/// into a real interrupt controller.
///
/// **It has a processor and `nvme-mini` does not**, and the difference is the
/// device: an NVMe command completes inside the doorbell write that submitted
/// it, while an EHCI's schedule runs on its own clock a microframe at a time, so
/// the claim worth making about one needs a program executing. The `firmware`
/// slot takes that program; the `usb0` slot takes the disk's contents.
#[cfg(feature = "machine-usb-mini")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-usb-mini")))]
pub static USB_MINI: CatalogEntry = CatalogEntry {
    name: "usb-mini",
    summary: "a minimal board with an EHCI and a USB mass storage device: a RISC-V hart, RAM, \
              a PLIC",
    media: &["firmware", "usb0"],
    source: include_str!("../../machines/usb-mini.machine"),
};

/// A minimal MC68000 board, when this build has a 68000.
///
/// A synthetic board rather than a product: a big-endian 24-bit space, ROM at
/// zero holding the exception vector table, and RAM above it. The `firmware`
/// slot takes whatever should be at `0x000000` — whose first two longwords are
/// the reset stack pointer and the reset program counter.
#[cfg(feature = "machine-m68k-mini")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-m68k-mini")))]
pub static M68K_MINI: CatalogEntry = CatalogEntry {
    name: "m68k-mini",
    summary: "a minimal MC68000 board: 8 MHz, a big-endian 24-bit space, ROM at the vectors, RAM",
    media: &["firmware"],
    source: include_str!("../../machines/m68k-mini.machine"),
};

/// A minimal R3000A board, when this build has a MIPS core.
///
/// A synthetic board rather than a product: a 32-bit **physical** space, a
/// boot ROM at `0x1FC0_0000` where the `kseg1` reset vector points, and RAM at
/// physical zero — which the processor's own segment map makes visible at
/// `0x0000_0000`, `0x8000_0000` and `0xA000_0000` alike. The `firmware` slot
/// takes whatever should answer at `0xBFC0_0000`.
#[cfg(feature = "machine-mips-mini")]
#[cfg_attr(docsrs, doc(cfg(feature = "machine-mips-mini")))]
pub static MIPS_MINI: CatalogEntry = CatalogEntry {
    name: "mips-mini",
    summary: "a minimal MIPS R3000A board: 25 MHz, a physical 32-bit space, a kseg1 boot ROM, RAM",
    media: &["firmware"],
    source: include_str!("../../machines/mips-mini.machine"),
};

/// Every machine this build can realize, in catalog order.
// One `#[cfg]`-gated push per shipped machine, which is what the lint is
// complaining about: a `vec![]` literal cannot carry an attribute on one of its
// elements, so the push form is the only one that expresses "this entry exists
// only in some builds".
#[allow(unused_mut, clippy::vec_init_then_push)]
#[must_use]
pub fn machines() -> Vec<&'static CatalogEntry> {
    let mut out: Vec<&'static CatalogEntry> = Vec::new();
    #[cfg(feature = "machine-apple1")]
    out.push(&APPLE1);
    #[cfg(feature = "machine-arm926")]
    out.push(&ARM926);
    #[cfg(feature = "machine-beneater")]
    out.push(&BENEATER_6502);
    #[cfg(feature = "machine-gameboy")]
    out.push(&GAMEBOY);
    #[cfg(feature = "machine-m68k-mini")]
    out.push(&M68K_MINI);
    #[cfg(feature = "machine-mips-mini")]
    out.push(&MIPS_MINI);
    #[cfg(feature = "machine-ne2k-mini")]
    out.push(&NE2K_MINI);
    #[cfg(feature = "machine-ahci-mini")]
    out.push(&AHCI_MINI);
    #[cfg(feature = "machine-nvme-mini")]
    out.push(&NVME_MINI);
    #[cfg(feature = "machine-pc-at")]
    out.push(&PC_AT);
    #[cfg(feature = "machine-pc-apic")]
    out.push(&PC_APIC);
    #[cfg(feature = "machine-nes")]
    out.push(&NES_NTSC);
    #[cfg(feature = "machine-nes")]
    out.push(&NES_PAL);
    #[cfg(feature = "machine-riscv-virt")]
    out.push(&RISCV_VIRT);
    #[cfg(feature = "machine-sms")]
    out.push(&SMS_NTSC);
    #[cfg(feature = "machine-sms")]
    out.push(&SMS_PAL);
    #[cfg(feature = "machine-spi-panel")]
    out.push(&SPI_PANEL);
    #[cfg(feature = "machine-spi-flash")]
    out.push(&SPI_FLASH);
    #[cfg(feature = "machine-stm32f407")]
    out.push(&STM32F407);
    #[cfg(feature = "machine-usb-mini")]
    out.push(&USB_MINI);
    #[cfg(feature = "machine-z80-mini")]
    out.push(&Z80_MINI);
    out
}

/// One shipped machine by name, with or without its `.machine` suffix.
#[must_use]
pub fn machine(name: &str) -> Option<&'static CatalogEntry> {
    let stem = name.strip_suffix(".machine").unwrap_or(name);
    machines().into_iter().find(|m| m.name == stem)
}

/// Every device class this build can construct.
///
/// # Errors
///
/// [`Error::Config`](crate::core::Error::Config) if two features claimed one
/// class name, which is a bug in this file rather than in a machine
/// description.
pub fn registry() -> Result<Registry> {
    let mut reg = Registry::new();
    builtin::register(&mut reg)?;
    #[cfg(feature = "cpu-mos6502")]
    crate::cpu::mos6502::register(&mut reg)?;
    #[cfg(feature = "cpu-sm83")]
    crate::cpu::sm83::register(&mut reg)?;
    #[cfg(feature = "dev-gb")]
    crate::dev::gb::register(&mut reg)?;
    #[cfg(feature = "dev-sms")]
    crate::dev::sms::register(&mut reg)?;
    #[cfg(feature = "dev-nes-cart")]
    crate::dev::cart::nrom::register(&mut reg)?;
    #[cfg(feature = "dev-nes-io")]
    crate::dev::nes::register(&mut reg)?;
    #[cfg(feature = "dev-nes-ppu")]
    crate::dev::ppu::register(&mut reg)?;
    #[cfg(feature = "dev-nes-apu")]
    crate::dev::apu::register(&mut reg)?;
    #[cfg(feature = "dev-at24c")]
    crate::dev::atmel::register(&mut reg)?;
    #[cfg(feature = "dev-apple1")]
    crate::dev::apple1::register(&mut reg)?;
    #[cfg(feature = "cpu-arm-aprofile")]
    crate::cpu::arm::aprofile::register(&mut reg)?;
    #[cfg(feature = "cpu-arm-v7m")]
    crate::cpu::arm::v7m::register(&mut reg)?;
    #[cfg(feature = "cpu-z80")]
    crate::cpu::z80::register(&mut reg)?;
    #[cfg(feature = "cpu-m68k")]
    crate::cpu::m68k::register(&mut reg)?;
    #[cfg(feature = "cpu-mips")]
    crate::cpu::mips::register(&mut reg)?;
    #[cfg(feature = "cpu-riscv")]
    crate::cpu::riscv::register(&mut reg)?;
    #[cfg(feature = "dev-riscv")]
    crate::dev::riscv::register(&mut reg)?;
    #[cfg(any(feature = "dev-flash-cfi", feature = "dev-flash-spinor"))]
    crate::dev::flash::register(&mut reg)?;
    #[cfg(feature = "dev-sd-card")]
    crate::dev::sd::register(&mut reg)?;
    #[cfg(feature = "dev-ata-disk")]
    crate::dev::ata::register(&mut reg)?;
    #[cfg(feature = "dev-wdc")]
    crate::dev::wdc::register(&mut reg)?;
    #[cfg(feature = "dev-ne2000")]
    crate::dev::net::ne2000::register(&mut reg)?;
    #[cfg(feature = "dev-ahci")]
    crate::dev::ahci::register(&mut reg)?;
    #[cfg(feature = "dev-nvme")]
    crate::dev::nvme::register(&mut reg)?;
    #[cfg(feature = "cpu-x86")]
    crate::cpu::x86::register(&mut reg)?;
    #[cfg(feature = "dev-pc")]
    crate::dev::pc::register(&mut reg)?;
    #[cfg(feature = "bus-spi")]
    crate::bus::spi::controller::register(&mut reg)?;
    #[cfg(feature = "dev-st7272a")]
    crate::dev::sitronix::register(&mut reg)?;
    #[cfg(any(
        feature = "dev-stm32",
        feature = "dev-stm32-sdmmc",
        feature = "dev-stm32-spi",
        feature = "dev-stm32-octospi",
        feature = "dev-stm32-i2c",
        feature = "machine-spi-flash"
    ))]
    crate::dev::stm32::register(&mut reg)?;
    #[cfg(feature = "dev-lcdc")]
    crate::dev::lcd::register(&mut reg)?;
    #[cfg(any(
        feature = "dev-usb-ehci",
        feature = "dev-usb-chipidea",
        feature = "dev-usb-dwc2",
        feature = "dev-usb-hid",
        feature = "dev-usb-msd"
    ))]
    crate::dev::usb::register(&mut reg)?;
    Ok(reg)
}

/// Every class that takes part in the memory map and the wire graph.
///
/// # The PPU and the APU
///
/// Both are **lazily advanced** (`ROADMAP.md` §4.2): they hold their own tick
/// and are caught up by whoever accesses them, through
/// [`LazyHandle`](crate::core::sched::LazyHandle). They were kept out of this
/// table until a class could *declare* that, because a PPU that is mapped and
/// wired and then never advanced reports VBlank clear forever — a worse machine
/// than one with no PPU at all, where the open bus at least reads as ones.
///
/// [`Device::is_lazy`](crate::core::Device::is_lazy) is that declaration, and
/// [`realize`](mod@crate::machine::realize) registers such a device on its clock
/// domain and hands it back the handle its own `MemOps` syncs through. So they
/// are bound.
///
/// # Errors
///
/// As [`registry`].
pub fn bindings() -> Result<Bindings> {
    let mut b = Bindings::new();
    builtin::bind(&mut b)?;
    #[cfg(feature = "cpu-mos6502")]
    crate::cpu::mos6502::bind(&mut b)?;
    #[cfg(feature = "cpu-sm83")]
    crate::cpu::sm83::bind(&mut b)?;
    #[cfg(feature = "dev-gb")]
    crate::dev::gb::bind(&mut b)?;
    #[cfg(feature = "dev-sms")]
    crate::dev::sms::bind(&mut b)?;
    #[cfg(feature = "dev-nes-cart")]
    crate::dev::cart::nrom::bind(&mut b)?;
    #[cfg(feature = "dev-nes-io")]
    crate::dev::nes::bind(&mut b)?;
    #[cfg(feature = "dev-nes-ppu")]
    crate::dev::ppu::bind(&mut b)?;
    #[cfg(feature = "dev-nes-apu")]
    crate::dev::apu::bind(&mut b)?;
    #[cfg(feature = "dev-at24c")]
    crate::dev::atmel::bind(&mut b)?;
    #[cfg(feature = "dev-apple1")]
    crate::dev::apple1::bind(&mut b)?;
    #[cfg(feature = "cpu-arm-aprofile")]
    crate::cpu::arm::aprofile::bind(&mut b)?;
    #[cfg(feature = "cpu-arm-v7m")]
    crate::cpu::arm::v7m::bind(&mut b)?;
    #[cfg(feature = "cpu-z80")]
    crate::cpu::z80::bind(&mut b)?;
    #[cfg(feature = "cpu-m68k")]
    crate::cpu::m68k::bind(&mut b)?;
    #[cfg(feature = "cpu-mips")]
    crate::cpu::mips::bind(&mut b)?;
    #[cfg(feature = "cpu-riscv")]
    crate::cpu::riscv::bind(&mut b)?;
    #[cfg(feature = "dev-riscv")]
    crate::dev::riscv::bind(&mut b)?;
    #[cfg(any(feature = "dev-flash-cfi", feature = "dev-flash-spinor"))]
    crate::dev::flash::bind(&mut b)?;
    #[cfg(feature = "dev-sd-card")]
    crate::dev::sd::bind(&mut b)?;
    #[cfg(feature = "dev-ata-disk")]
    crate::dev::ata::bind(&mut b)?;
    #[cfg(feature = "dev-wdc")]
    crate::dev::wdc::bind(&mut b)?;
    #[cfg(feature = "dev-ne2000")]
    crate::dev::net::ne2000::bind(&mut b)?;
    #[cfg(feature = "dev-ahci")]
    crate::dev::ahci::bind(&mut b)?;
    #[cfg(feature = "dev-nvme")]
    crate::dev::nvme::bind(&mut b)?;
    #[cfg(feature = "cpu-x86")]
    crate::cpu::x86::bind(&mut b)?;
    #[cfg(feature = "dev-pc")]
    crate::dev::pc::bind(&mut b)?;
    #[cfg(feature = "bus-spi")]
    crate::bus::spi::controller::bind(&mut b)?;
    #[cfg(feature = "dev-st7272a")]
    crate::dev::sitronix::bind(&mut b)?;
    #[cfg(any(
        feature = "dev-stm32",
        feature = "dev-stm32-sdmmc",
        feature = "dev-stm32-spi",
        feature = "dev-stm32-octospi",
        feature = "dev-stm32-i2c",
        feature = "machine-spi-flash"
    ))]
    crate::dev::stm32::bind(&mut b)?;
    #[cfg(feature = "dev-lcdc")]
    crate::dev::lcd::bind(&mut b)?;
    #[cfg(any(
        feature = "dev-usb-ehci",
        feature = "dev-usb-chipidea",
        feature = "dev-usb-dwc2",
        feature = "dev-usb-hid",
        feature = "dev-usb-msd"
    ))]
    crate::dev::usb::bind(&mut b)?;
    Ok(b)
}

/// What the validator checks a machine file against.
#[must_use]
pub fn classes() -> ClassTable {
    let mut table = ClassTable::new();
    for schema in builtin::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "cpu-mos6502")]
    table.insert(crate::cpu::mos6502::schema());
    #[cfg(feature = "cpu-sm83")]
    table.insert(crate::cpu::sm83::schema());
    #[cfg(feature = "dev-gb")]
    for schema in crate::dev::gb::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "dev-sms")]
    for schema in crate::dev::sms::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "dev-nes-cart")]
    table.insert(crate::dev::cart::nrom::schema());
    #[cfg(feature = "dev-nes-io")]
    for schema in crate::dev::nes::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "dev-nes-ppu")]
    table.insert(crate::dev::ppu::schema());
    #[cfg(feature = "dev-nes-apu")]
    table.insert(crate::dev::apu::schema());
    #[cfg(feature = "dev-at24c")]
    for schema in crate::dev::atmel::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "dev-apple1")]
    for schema in crate::dev::apple1::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "cpu-arm-aprofile")]
    table.insert(crate::cpu::arm::aprofile::schema());
    #[cfg(feature = "cpu-arm-v7m")]
    table.insert(crate::cpu::arm::v7m::schema());
    #[cfg(feature = "cpu-z80")]
    table.insert(crate::cpu::z80::schema());
    #[cfg(feature = "dev-ne2000")]
    table.insert(crate::dev::net::ne2000::schema());
    #[cfg(feature = "dev-ahci")]
    table.insert(crate::dev::ahci::schema());
    #[cfg(feature = "dev-nvme")]
    table.insert(crate::dev::nvme::schema());
    #[cfg(feature = "cpu-m68k")]
    table.insert(crate::cpu::m68k::schema());
    #[cfg(feature = "cpu-mips")]
    table.insert(crate::cpu::mips::schema());
    #[cfg(feature = "cpu-riscv")]
    table.insert(crate::cpu::riscv::schema());
    #[cfg(feature = "dev-riscv")]
    for schema in crate::dev::riscv::schemas() {
        table.insert(schema);
    }
    #[cfg(any(feature = "dev-flash-cfi", feature = "dev-flash-spinor"))]
    for schema in crate::dev::flash::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "dev-sd-card")]
    for schema in crate::dev::sd::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "dev-ata-disk")]
    for schema in crate::dev::ata::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "dev-wdc")]
    for schema in crate::dev::wdc::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "bus-spi")]
    table.insert(crate::bus::spi::controller::schema());
    #[cfg(feature = "dev-st7272a")]
    for schema in crate::dev::sitronix::schemas() {
        table.insert(schema);
    }
    #[cfg(any(
        feature = "dev-stm32",
        feature = "dev-stm32-sdmmc",
        feature = "dev-stm32-spi",
        feature = "dev-stm32-octospi",
        feature = "dev-stm32-i2c",
        feature = "machine-spi-flash"
    ))]
    for schema in crate::dev::stm32::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "cpu-x86")]
    for schema in crate::cpu::x86::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "dev-pc")]
    for schema in crate::dev::pc::schemas() {
        table.insert(schema);
    }
    #[cfg(feature = "dev-lcdc")]
    for schema in crate::dev::lcd::schemas() {
        table.insert(schema);
    }
    #[cfg(any(
        feature = "dev-usb-ehci",
        feature = "dev-usb-chipidea",
        feature = "dev-usb-dwc2",
        feature = "dev-usb-hid",
        feature = "dev-usb-msd"
    ))]
    for schema in crate::dev::usb::schemas() {
        table.insert(schema);
    }
    table
}

/// [`BuildOptions`] wired to this build's classes and bindings.
///
/// The caller adds media and parameter overrides; everything else about "what
/// this binary knows" is already here.
///
/// # Errors
///
/// As [`registry`].
pub fn build_options() -> Result<BuildOptions> {
    Ok(BuildOptions::new()
        .with_classes(classes())
        .with_bindings(bindings()?))
}

/// Build a shipped machine by catalog name.
///
/// `media` binds the slots the description names: for the NES that is
/// `[("cart", &image)]`, which is what `--cart smb.nes` becomes.
///
/// # Errors
///
/// If the name is not in this build's catalog, or anything
/// [`build`](crate::machine::build) refuses — including a media slot the
/// caller did not bind.
pub fn build_catalog(name: &str, media: &[(&str, &[u8])]) -> Result<Machine> {
    build_catalog_with_hosts(name, media).map(|(machine, _)| machine)
}

/// Build a shipped machine and keep the host objects its devices opened.
///
/// The same build as [`build_catalog`], with the other end of every character
/// port, pad and signal the machine asked for — see
/// [`core::hosts`](crate::core::hosts). `build_catalog` drops that table, which
/// is right for a caller that only drives the machine and reads its snapshot,
/// and useless to one that wants to type at it.
///
/// # Errors
///
/// As [`build_catalog`].
pub fn build_catalog_with_hosts(
    name: &str,
    media: &[(&str, &[u8])],
) -> Result<(Machine, alloc::sync::Arc<crate::core::hosts::HostObjects>)> {
    let entry = machine(name).ok_or_else(|| unknown(name))?;
    let mut options = build_options()?;
    for (slot, bytes) in media {
        options.realize.media.insert(*slot, *bytes);
    }
    let machine = crate::machine::build(entry.name, entry.source, &registry()?, &options)?;
    Ok((machine, options.realize.hosts))
}

/// The error for a machine this build does not ship.
fn unknown(name: &str) -> crate::core::Error {
    let mut message = String::from("no machine named `");
    message.push_str(name);
    message.push_str("` in this build; it has ");
    let names: Vec<&str> = machines().into_iter().map(|m| m.name).collect();
    if names.is_empty() {
        message.push_str("none (enable a `machine-*` feature)");
    } else {
        for (i, n) in names.iter().enumerate() {
            if i != 0 {
                message.push_str(", ");
            }
            message.push('`');
            message.push_str(n);
            message.push('`');
        }
    }
    crate::core::Error::Config {
        at: String::from("catalog"),
        message,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn the_registry_and_the_bindings_agree() {
        let reg = registry().expect("no class name collides");
        let bound = bindings().expect("no binding collides");
        // A class with a binding but no registry entry is invisible to
        // `rsemu devices` and to the validator, which is exactly the drift
        // the registry-is-the-table-of-record rule exists to prevent.
        for class in bound.classes() {
            assert!(
                reg.get(class).is_some(),
                "`{class}` is bound but not registered"
            );
        }
        assert!(reg.get("ram").is_some(), "the language's own class");
    }

    #[test]
    fn every_shipped_machine_realizes() {
        // The catalog's whole claim. A machine file that no longer parses, or
        // names a class this build dropped, fails here rather than in front of
        // a user.
        for entry in machines() {
            let media: Vec<(&str, &[u8])> = entry
                .media
                .iter()
                .map(|slot| (*slot, fixture(entry.name, slot)))
                .collect();
            match build_catalog(entry.name, &media) {
                Ok(machine) => assert_eq!(machine.name(), entry.name),
                Err(e) => panic!("{}: {e}", entry.name),
            }
        }
    }

    #[test]
    fn an_unknown_machine_lists_what_there_is() {
        let e = build_catalog("megadrive", &[])
            .expect_err("no megadrive")
            .to_string();
        assert!(e.contains("megadrive"), "{e}");
    }

    /// The CPU's architectural state, read back out of a snapshot.
    ///
    /// There is no route from a `dyn Device` to a `Mos6502` — `core::device`
    /// keeps `Any` out of the supertrait chain deliberately — so the way to
    /// see a core's registers from outside is the surface §4.5 already
    /// promises: its snapshot chunk. Reading it here doubles as a check that
    /// the chunk really is the architectural state, and it pins the layout to
    /// the class version so a bump cannot silently change what this decodes.
    // Gated on the *core*, but every caller is gated on a machine, so a build
    // with `cpu-mos6502` and no board leaves these unused. Enumerating the
    // boards in a `cfg` here would go stale the next time one lands, which is
    // the same rot the feature sweep itself had.
    #[cfg(feature = "cpu-mos6502")]
    #[allow(dead_code)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct CpuState {
        a: u8,
        x: u8,
        y: u8,
        s: u8,
        p: u8,
        pc: u16,
        cycles: u64,
        halted: bool,
        reset_pending: bool,
        faults: u64,
        last_fault: u16,
    }

    #[cfg(feature = "cpu-mos6502")]
    #[allow(dead_code)]
    fn cpu_state(machine: &Machine, path: &str) -> CpuState {
        use crate::core::state::{Migrations, Source, StateReader};
        let class = &crate::cpu::mos6502::CLASS;
        let bytes = machine.save().expect("a machine saves");
        let reader = StateReader::new(&bytes).expect("well formed");
        let chunk = reader
            .load(path, class.name, class.version, &Migrations::new())
            .expect("a chunk per device, keyed by instance path");
        let mut r = chunk.reader();
        let mut byte = || r.read_u8().expect("the chunk is not truncated");
        let (a, x, y, s, p) = (byte(), byte(), byte(), byte(), byte());
        let pc = r.read_u16().expect("pc");
        let cycles = r.read_u64().expect("cycles");
        let halted = r.read_bool().expect("halted");
        let reset_pending = r.read_bool().expect("reset_pending");
        let _pending_interrupt = r.read_u8().expect("pending");
        let _open_bus = r.read_u8().expect("open bus");
        let faults = r.read_u64().expect("faults");
        let last_fault = r.read_u16().expect("last fault");
        CpuState {
            a,
            x,
            y,
            s,
            p,
            pc,
            cycles,
            halted,
            reset_pending,
            faults,
            last_fault,
        }
    }

    /// One CPU-visible byte, read the way a debugger would.
    #[cfg(feature = "machine-nes")]
    fn peek(machine: &Machine, addr: u64) -> u8 {
        use crate::core::space::MemAttrs;
        use crate::core::value::Width;
        machine
            .space("cpubus")
            .expect("cpubus")
            .read(addr, Width::U8, MemAttrs::DEBUG)
            .expect("open bus answers everything") as u8
    }

    /// The reset vector is fetched from the cartridge and executed from.
    ///
    /// No corpus and no environment variable: the fixture is a generated NROM
    /// image whose vector points at `$C000` and whose only instruction is
    /// `JMP $C000`, so the CPU's program counter after any amount of running
    /// is exactly one known number. That makes this the test that fails when
    /// the memory map, the vector fetch or the scheduler wiring breaks —
    /// [`a_real_cartridge_boots_and_executes`] then says how far real software
    /// gets.
    ///
    /// [`a_real_cartridge_boots_and_executes`]: self::tests::a_real_cartridge_boots_and_executes
    #[cfg(feature = "machine-nes")]
    #[test]
    fn the_reset_vector_is_fetched_and_executed() {
        let mut machine =
            build_catalog("nes-ntsc", &[("cart", MINIMAL_NROM)]).expect("a minimal cart");

        // NROM-128: 16 KiB of PRG answers at $8000 *and* at $C000, because A14
        // is not connected. Both windows must show the same byte.
        assert_eq!(peek(&machine, 0xfffc), 0x00);
        assert_eq!(peek(&machine, 0xfffd), 0xc0);
        assert_eq!(peek(&machine, 0x8000), 0x4c, "JMP at the low window");
        assert_eq!(peek(&machine, 0xc000), 0x4c, "and at the high one");

        let before = cpu_state(&machine, "cpu");
        assert!(before.reset_pending);

        machine
            .run_for(crate::core::clock::GlobalTime::from_nanos(100_000))
            .expect("runs");

        let after = cpu_state(&machine, "cpu");
        assert!(!after.reset_pending, "the reset sequence ran");
        assert_eq!(
            after.pc, 0xc000,
            "the cpu is not executing the reset vector's target"
        );
        // The reset sequence pushes nothing but decrements S three times, from
        // the power-on 0 — so $FD, and it sets I.
        assert_eq!(after.s, 0xfd);
        assert_ne!(after.p & crate::cpu::mos6502::flags::I, 0);
        assert!(after.cycles >= 7 + 3, "reset plus at least one JMP");
        assert_eq!(after.faults, 0, "every access is answered");

        // 2 KiB of work RAM answers four times over $0000-$1FFF, which is the
        // one thing in this machine the cartridge does not provide.
        machine
            .space("cpubus")
            .expect("cpubus")
            .write(
                0x0003,
                crate::core::value::Width::U8,
                0xa5,
                crate::core::space::MemAttrs::DEFAULT,
            )
            .expect("wram is writable");
        for base in [0x0000u64, 0x0800, 0x1000, 0x1800] {
            assert_eq!(peek(&machine, base + 3), 0xa5, "mirror at {base:#06x}");
        }
    }

    /// A machine built from a `.machine` file snapshots and restores.
    #[cfg(feature = "machine-nes")]
    #[test]
    fn a_running_nes_round_trips_through_a_snapshot() {
        let mut machine =
            build_catalog("nes-ntsc", &[("cart", MINIMAL_NROM)]).expect("a minimal cart");
        machine
            .run_for(crate::core::clock::GlobalTime::from_nanos(100_000))
            .expect("runs");
        let saved = machine.save().expect("saves");
        let hash = machine.state_hash().expect("hashes");

        // Into a second machine built from the same description, which is the
        // case that matters: a save state is loaded by a fresh process.
        let mut restored =
            build_catalog("nes-ntsc", &[("cart", MINIMAL_NROM)]).expect("a minimal cart");
        assert_ne!(restored.state_hash().expect("hashes"), hash);
        restored.load(&saved).expect("loads");
        assert_eq!(restored.state_hash().expect("hashes"), hash);
        assert_eq!(cpu_state(&restored, "cpu"), cpu_state(&machine, "cpu"));

        // And it keeps running identically from there — the point of a
        // deterministic snapshot, and what the cycle debt is carried for.
        let span = crate::core::clock::GlobalTime::from_nanos(1_000_000);
        machine.run_for(span).expect("runs");
        restored.run_for(span).expect("runs");
        assert_eq!(
            restored.state_hash().expect("hashes"),
            machine.state_hash().expect("hashes")
        );
    }

    /// A whole NES boots from a real cartridge and retires instructions.
    ///
    /// The phase-3 milestone in one test: a `.machine` file, a real ROM bound
    /// to a media slot, a realized machine, and a 6502 fetching its reset
    /// vector out of PRG ROM and executing from it — through the scheduler,
    /// the address space and the region tree, with no hand-wiring anywhere.
    ///
    /// Gated on `RSEMU_NES_TEST_ROM`, like every other corpus (`CLAUDE.md`):
    /// point it at an iNES image. AccuracyCoin is the one this was written
    /// against — MIT, © 2025 Chris Siebert — but any NROM cartridge works.
    /// Without the variable the test passes trivially, so `cargo test` offline
    /// stays green.
    #[cfg(all(feature = "machine-nes", feature = "std"))]
    #[test]
    fn a_real_cartridge_boots_and_executes() {
        let Ok(path) = std::env::var("RSEMU_NES_TEST_ROM") else {
            println!("SKIP: set RSEMU_NES_TEST_ROM to an iNES image to run this");
            return;
        };
        let image = std::fs::read(&path).expect("RSEMU_NES_TEST_ROM is readable");
        let mut machine = match build_catalog("nes-ntsc", &[("cart", &image)]) {
            Ok(m) => m,
            Err(e) => panic!("{path}: {e}"),
        };

        // The machine came up cold, so the CPU owes a reset sequence and has
        // not fetched anything yet.
        let before = cpu_state(&machine, "cpu");
        assert!(before.reset_pending, "a cold machine owes a reset");
        assert_eq!(before.cycles, 0);

        // The reset vector, as the cartridge holds it. `$FFFC` is inside PRG
        // ROM, so this already proves the cart is mapped where the file says.
        let vector = u16::from(peek(&machine, 0xfffc)) | (u16::from(peek(&machine, 0xfffd)) << 8);
        assert!(
            vector >= 0x8000,
            "a reset vector of {vector:#06x} is not in cartridge space; is the ROM mapped?"
        );

        // A frame's worth of virtual time. Deterministic: the same span always
        // retires the same number of cycles, whatever the host is doing.
        let frame = crate::core::clock::GlobalTime::from_nanos(16_639_267);
        machine.run_for(frame).expect("the machine runs");

        let after = cpu_state(&machine, "cpu");
        let domain = machine
            .device("cpu")
            .and_then(crate::machine::machine::DeviceEntry::domain)
            .expect("the cpu has a clock domain");
        let ticks = machine.clocks().ticks(domain).expect("a tick count");

        println!(
            "nes-ntsc + {path}:\n  \
             reset vector ${vector:04x}\n  \
             {} cpu cycles in one frame ({ticks} domain ticks)\n  \
             {}\n  \
             {} refused access(es){}",
            after.cycles,
            regs_line(&after),
            after.faults,
            if after.faults == 0 {
                ""
            } else {
                " — the memory map has a hole the open-bus policy did not cover"
            },
        );

        assert!(!after.reset_pending, "the reset sequence must have run");
        assert!(
            after.cycles > 20_000,
            "only {} cycles in a frame; the cpu is not running",
            after.cycles
        );
        // The scheduler must not have been overrun or starved: one NTSC frame
        // is 29780.5 CPU cycles, and the debt mechanism keeps the core's own
        // count within one instruction of its domain's.
        assert!(
            after.cycles.abs_diff(ticks) <= 7,
            "cpu counted {} cycles but its domain advanced {ticks}",
            after.cycles
        );
        assert!(
            !after.halted,
            "a JAM opcode froze the core at ${:04x}",
            after.pc
        );
        // Every access is answered: RAM, the cartridge, or the open bus the
        // real console has. A refusal means the address space itself said no.
        assert_eq!(
            after.faults, 0,
            "bus fault at ${:04x} after {} cycles",
            after.last_fault, after.cycles
        );

        // -- and now the part the PPU makes possible -----------------------
        //
        // Two seconds of virtual time is far more than any NES ROM's init
        // needs: AccuracyCoin runs its whole first page of CPU tests and draws
        // its menu well inside one.
        for _ in 0..120 {
            machine.run_for(frame).expect("the machine runs");
        }
        let after = cpu_state(&machine, "cpu");
        let ppu_domain = machine
            .device("ppu")
            .and_then(crate::machine::machine::DeviceEntry::domain)
            .expect("the ppu has a clock domain");
        let dots = machine.clocks().ticks(ppu_domain).expect("a dot count");
        let ticks = machine.clocks().ticks(domain).expect("a tick count");
        println!(
            "  after 121 frames: {} PPU dots, {}\n  \
             {} tiles written to the first nametable",
            dots,
            regs_line(&after),
            nametable_tiles(&machine)
        );

        // Exactly three dots per CPU cycle, forever, by construction: both
        // counters descend from one crystal (`ROADMAP.md` §4.2). If this ever
        // drifts the split-screen effects in half the NES library break.
        assert_eq!(dots, ticks * 3, "the dot clock is not three times the CPU");
        assert_eq!(after.faults, 0, "bus fault at ${:04x}", after.last_fault);
        assert!(
            !after.halted,
            "a JAM opcode froze the core at ${:04x}",
            after.pc
        );

        // The ROM has finished its init, run its tests and drawn a screen.
        // Reaching this needs `$2002` to report vblank at the dot it is read
        // on, `$2006`/`$2007` to reach the nametables through the PPU's own
        // bus, and the NMI to arrive: an unadvanced PPU parks the wait loop at
        // the top of the reset path with a blank screen behind it.
        let tiles = nametable_tiles(&machine);
        assert!(
            tiles > 64,
            "only {tiles} non-blank tiles in the first nametable; the ROM never \
             drew anything"
        );
    }

    /// How many of the first nametable's 960 tiles are not the blank one.
    ///
    /// Read through the PPU's own address space with debug attributes, so
    /// counting them disturbs nothing.
    #[cfg(all(feature = "machine-nes", feature = "std"))]
    fn nametable_tiles(machine: &Machine) -> usize {
        use crate::core::space::MemAttrs;
        use crate::core::value::Width;
        let space = machine.space("ppubus").expect("ppubus");
        (0..960u64)
            .filter(|i| {
                let tile = space
                    .read(0x2000 + i, Width::U8, MemAttrs::DEBUG)
                    .unwrap_or(0);
                // `$24` is the blank tile in most ASCII-ish NES tile sets, and
                // `$00` is an unwritten nametable.
                tile != 0x24 && tile != 0x00
            })
            .count()
    }

    #[cfg(feature = "cpu-mos6502")]
    #[allow(dead_code)]
    fn regs_line(s: &CpuState) -> alloc::string::String {
        alloc::format!(
            "A:{:02x} X:{:02x} Y:{:02x} P:{:02x} SP:{:02x} PC:{:04x}",
            s.a,
            s.x,
            s.y,
            s.p,
            s.s,
            s.pc
        )
    }

    /// The whole point of the exercise: vblank is reported and the NMI fires.
    ///
    /// No corpus and no environment variable — the cartridge is generated here,
    /// so this runs on every `cargo test` and is the regression gate for
    /// sync-on-access end to end (`ROADMAP.md` §4.2). It proves three separate
    /// things, and each of them was broken before the PPU could declare itself
    /// lazily advanced:
    ///
    /// 1. **`$2002` reports vblank.** The ROM does what every NES game's init
    ///    does — polls `$2002` until bit 7 goes high, twice. With an unadvanced
    ///    PPU that loop never ends; with an unmapped one it falls straight
    ///    through on open bus, which looks like success and is not.
    /// 2. **The chip advances with nobody looking at it.** After enabling the
    ///    NMI the ROM sits in `JMP *` and touches no PPU register ever again.
    ///    Sync-on-access alone would leave it there forever.
    /// 3. **The NMI lands once per frame.** Not twice, which is what a level
    ///    re-announced at every catch-up boundary would produce, and not never.
    #[cfg(feature = "machine-nes")]
    #[test]
    fn vblank_is_reported_and_the_nmi_fires_once_a_frame() {
        let image = nmi_rom();
        let mut machine = build_catalog("nes-ntsc", &[("cart", &image)]).expect("a cart");
        // Six frames of virtual time. The first is spent in the warm-up
        // lockout, so `$2000` is only accepted from the second.
        let frame = crate::core::clock::GlobalTime::from_nanos(16_639_267);
        for _ in 0..6 {
            machine.run_for(frame).expect("runs");
        }

        assert_eq!(
            peek(&machine, 0x0001),
            1,
            "the vblank wait loop never ended"
        );
        let nmis = peek(&machine, 0x0000);
        // Frame 0 is the lockout, and the two `$2002` wait loops each cost a
        // frame — a poll that lands on the dot the flag would be set on reads
        // it clear and suppresses the set for that whole frame, which is the
        // hardware behaviour the vblank tests exist to pin. So three are
        // certain and six are the ceiling; what the assertion is really about
        // is that the count is neither zero nor doubled.
        assert!(
            (3..=6).contains(&nmis),
            "{nmis} NMIs in six frames; one per vblank is the answer"
        );

        // Another three frames: exactly three more.
        for _ in 0..3 {
            machine.run_for(frame).expect("runs");
        }
        assert_eq!(peek(&machine, 0x0000), nmis + 3);
    }

    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
    ///
    /// The catch-up hook sits at the top of the PPU's own `MemOps::read`, which
    /// is exactly where it would be easiest to move the chip's clock on a
    /// monitor read. It must not.
    #[cfg(all(feature = "machine-nes", feature = "dev-nes-ppu"))]
    #[test]
    fn a_debug_read_of_2002_advances_no_clock() {
        use crate::core::space::MemAttrs;
        use crate::core::value::Width;

        let mut machine = build_catalog("nes-ntsc", &[("cart", MINIMAL_NROM)]).expect("a cart");
        machine
            .run_for(crate::core::clock::GlobalTime::from_nanos(1_000_000))
            .expect("runs");
        // Catch-up did happen: the chip is standing exactly on its domain's
        // tick. Without that this test would pass vacuously.
        let domain = machine
            .device("ppu")
            .and_then(crate::machine::machine::DeviceEntry::domain)
            .expect("the ppu has a clock domain");
        let before = ppu_dots(&machine);
        assert_eq!(before, machine.clocks().ticks(domain).expect("ticks"));
        assert!(before > 0);

        for _ in 0..64 {
            let _ = peek(&machine, 0x2002);
        }
        assert_eq!(
            ppu_dots(&machine),
            before,
            "a debug read moved the dot clock"
        );

        // A debug *write* to a port with side effects is refused outright,
        // which is the same invariant seen from the other side.
        assert!(
            machine
                .space("cpubus")
                .expect("cpubus")
                .write(0x2000, Width::U8, 0x80, MemAttrs::DEBUG)
                .is_err()
        );
        assert_eq!(ppu_dots(&machine), before);
    }

    /// The PPU's dot counter, out of its snapshot chunk.
    #[cfg(all(feature = "machine-nes", feature = "dev-nes-ppu"))]
    fn ppu_dots(machine: &Machine) -> u64 {
        use crate::core::state::{Migrations, Source, StateReader};
        let class = &crate::dev::ppu::NES_PPU_CLASS;
        let bytes = machine.save().expect("a machine saves");
        let reader = StateReader::new(&bytes).expect("well formed");
        let chunk = reader
            .load("ppu", class.name, class.version, &Migrations::new())
            .expect("a chunk per device");
        chunk.reader().read_u64().expect("dots come first")
    }

    /// A cartridge whose init waits for two vblanks, enables the NMI and then
    /// does nothing at all — the shape of every NES game's reset path.
    ///
    /// `$00` counts NMIs and `$01` counts completions of the wait loop.
    #[cfg(feature = "machine-nes")]
    fn nmi_rom() -> alloc::vec::Vec<u8> {
        let mut image = alloc::vec![0u8; 16 + 16384 + 8192];
        image[..4].copy_from_slice(b"NES\x1a");
        image[4] = 1; // 16 KiB of PRG, which answers at $8000 and at $C000
        image[5] = 1; // 8 KiB of CHR
        let prg = &mut image[16..16 + 16384];
        let code: &[u8] = &[
            0x78, // $C000  SEI
            0xad, 0x02, 0x20, // $C001  LDA $2002   reset the address latch
            0xad, 0x02, 0x20, // $C004  LDA $2002
            0x10, 0xfb, //       $C007  BPL $C004   wait for vblank
            0xad, 0x02, 0x20, // $C009  LDA $2002
            0x10, 0xfb, //       $C00C  BPL $C009   and again
            0xee, 0x01, 0x00, // $C00E  INC $0001   the wait loop ended
            0xa9, 0x80, //       $C011  LDA #$80
            0x8d, 0x00, 0x20, // $C013  STA $2000   enable the NMI
            0x4c, 0x16, 0xc0, // $C016  JMP $C016   and never look again
        ];
        prg[..code.len()].copy_from_slice(code);
        // The handler, at $C020: count it and return. It deliberately does not
        // read $2002 — the request stays asserted for the whole of vblank, and
        // the CPU's edge latch is what must keep that from firing twice.
        prg[0x20..0x23].copy_from_slice(&[0xe6, 0x00, 0x40]); // INC $00 ; RTI
        // NMI $C020, RESET $C000, IRQ $C020.
        prg[0x3ffa..0x4000].copy_from_slice(&[0x20, 0xc0, 0x00, 0xc0, 0x20, 0xc0]);
        image
    }

    /// Something plausible to bind to a media slot, so the catalog can be
    /// realized without a corpus.
    fn fixture(machine: &str, slot: &str) -> &'static [u8] {
        match (machine, slot) {
            // Two machines take a slot called `cart` and they are not the same
            // kind of cartridge at all, which is why this is keyed by both.
            #[cfg(feature = "machine-gameboy")]
            ("gameboy", "cart") => minimal_gb(),
            (_, "cart") => MINIMAL_NROM,
            // Each board's default monitor: rsemu's own, committed precisely so
            // that this needs no download and no licence question. The two are
            // different sizes — a 256-byte PROM socket against a 32 KiB EEPROM
            // — so the slot name alone does not decide it.
            #[cfg(feature = "machine-apple1")]
            ("apple1", "rom") => crate::dev::apple1::RSMON,
            #[cfg(feature = "machine-beneater")]
            ("beneater-6502", "rom") => crate::dev::wdc::RSMON_IMAGE,
            // A firmware image that fits in eight bytes: `wfi` then a branch
            // back to it, so the catalog's "every shipped machine realizes"
            // check needs no download and no toolchain. `dev::riscv::tests`
            // supplies the programs that actually do something.
            #[cfg(feature = "machine-riscv-virt")]
            ("riscv-virt", "firmware") => &[0x73, 0x00, 0x50, 0x10, 0x6f, 0xf0, 0xdf, 0xff],
            // Both NOR banks come up erased when nothing is put in them, which
            // is a board with blank parts soldered on — the state a factory
            // ships and the state a UEFI build initialises for itself.
            #[cfg(feature = "machine-riscv-virt")]
            ("riscv-virt", "flash0" | "flash1") => &[],
            // No ramdisk, which is what a bare-metal or disk-rooted boot has.
            // The `initrd` loader writes nothing for an empty image and the
            // boot ROM leaves `/chosen` without the two `linux,initrd-*`
            // properties, so a machine with this slot unbound is the machine
            // that existed before the slot did.
            #[cfg(feature = "machine-riscv-virt")]
            ("riscv-virt", "initrd") => &[],
            // And no disk image, which leaves the `size` in the machine file
            // to supply a blank one — a board with an unwritten disk in it.
            #[cfg(feature = "machine-riscv-virt")]
            ("riscv-virt", "disk") => &[],
            // The board's own demo: it configures the panel over SPI, paints a
            // gradient and enables the scanout engine. Assembled at compile
            // time by `dev::lcd::demo`, so it needs no toolchain either.
            #[cfg(feature = "machine-spi-panel")]
            ("spi-panel", "firmware") => crate::dev::lcd::demo::PANEL_DEMO,
            #[cfg(feature = "machine-spi-flash")]
            ("spi-flash", "firmware") => crate::dev::stm32::demo::SPI_FLASH_DEMO,
            // `B .` — the ARM branch-to-self, which is the whole four-byte
            // program needed to prove the board realizes and the core fetches.
            // `tests/arm926_board.rs` supplies the one that does something.
            #[cfg(feature = "machine-arm926")]
            ("arm926", "firmware") => &[0xfe, 0xff, 0xff, 0xea],
            // The two words a Cortex-M4 fetches out of reset — an initial
            // stack pointer at the top of SRAM and a reset vector, with bit 0
            // set because there is no ARM state to interwork to — followed by
            // `B .`, the two-byte branch to itself at 0x08. Everything needed
            // to prove the board realizes and the core fetches; the program
            // that does something is in `tests/stm32f407_board.rs`.
            #[cfg(feature = "machine-stm32f407")]
            ("stm32f407", "firmware") => &[
                0x00, 0x00, 0x02, 0x20, // SP = 0x20020000, the top of SRAM2
                0x09, 0x00, 0x00, 0x00, // PC = 0x00000008 | 1
                0xfe, 0xe7, // b .
            ],
            // `jal x0, 0` — the RV32 jump-to-itself, which is the whole
            // four-byte program needed to prove the board realizes and the hart
            // fetches. `tests/usb_msd.rs` supplies the one that enumerates a
            // disk and moves a sector.
            #[cfg(feature = "machine-usb-mini")]
            ("usb-mini", "firmware") => &[0x6f, 0x00, 0x00, 0x00],
            // No default disk contents, which leaves the `capacity` in the
            // machine file to supply a blank one — exactly as `nvme-mini` and
            // `ahci-mini` get, and a run that means something else says
            // `--drive usb0=disk.img`.
            #[cfg(feature = "machine-usb-mini")]
            ("usb-mini", "usb0") => &[],
            // `JR -2` — the Z80 branch-to-self, which is the whole two-byte
            // program needed to prove the board realizes and the core fetches.
            // `tests/z80_mini_board.rs` supplies the one that does something.
            #[cfg(feature = "machine-z80-mini")]
            ("z80-mini", "firmware") => &[0x18, 0xfe],
            // The same two bytes, for the same reason: this board realizes and
            // fetches with them, and the driver that actually talks to the
            // NE2000 lives in `tests/ne2000_board.rs`.
            #[cfg(feature = "machine-ne2k-mini")]
            ("ne2k-mini", "firmware") => &[0x18, 0xfe],
            // No default namespace contents, which leaves the `disk` size in
            // the machine file to supply a blank one — a board with an
            // unwritten drive in it, exactly as `riscv-virt` gets.
            #[cfg(feature = "machine-nvme-mini")]
            ("nvme-mini", "nvme0") => &[],
            // The same, and for the same reason: the `disk` size in the machine
            // file supplies a blank drive, and a run that means something else
            // says `--drive sata0=disk.img`.
            #[cfg(feature = "machine-ahci-mini")]
            ("ahci-mini", "sata0") => &[],
            // The two longwords a 68000 fetches out of reset — a stack pointer
            // at the top of the board's RAM and a program counter at $000008 —
            // followed by `BRA .-0`, the two-byte branch to itself. Everything
            // needed to prove the board realizes and the core fetches; the
            // program that does something is in `tests/m68k_mini_board.rs`.
            // `j .` and its delay slot — the MIPS branch to itself, which is
            // two words rather than one because every transfer of control on
            // this architecture drags the instruction after it along. The
            // target field is 0xbfc00000 >> 2, masked to 26 bits; the top four
            // bits of the address come from the delay slot's program counter.
            // Everything needed to prove the board realizes and the core
            // fetches out of kseg1; the program that does something is in
            // `tests/mips_mini_board.rs`.
            #[cfg(feature = "machine-mips-mini")]
            ("mips-mini", "firmware") => &[
                0x00, 0x00, 0xf0, 0x0b, // j 0xbfc00000
                0x00, 0x00, 0x00, 0x00, // nop, in the delay slot
            ],
            #[cfg(feature = "machine-m68k-mini")]
            ("m68k-mini", "firmware") => &[
                0x00, 0x20, 0x00, 0x00, // SSP = $00200000
                0x00, 0x00, 0x00, 0x08, // PC  = $00000008
                0x60, 0xfe, // BRA .
            ],
            // No firmware is shipped for the PC and none ever will be, so what
            // this board gets is the right *shape*: a socket-sized image of
            // zeroes, which realizes and executes open bus. `tests/pc_at_board`
            // is where the board is actually exercised.
            #[cfg(feature = "machine-pc-at")]
            ("pc-at", "bios") => blank(128 * 1024),
            #[cfg(feature = "machine-pc-at")]
            ("pc-at", "vgabios") => blank(32 * 1024),
            #[cfg(feature = "machine-pc-at")]
            ("pc-at", "floppy") => blank(1_474_560),
            // Both IDE bays empty, which is what no bytes bound means: a PC
            // with no hard disk is an ordinary PC, and a drive would cost this
            // test its whole capacity in host memory. `tests/pc_at_ide` is
            // where a populated bay is exercised.
            #[cfg(feature = "machine-pc-at")]
            ("pc-at", "hd0" | "hd1") => &[],
            #[cfg(feature = "machine-pc-apic")]
            ("pc-apic", "bios") => blank(128 * 1024),
            (m, other) => panic!("no fixture for `{m}`'s media slot `{other}`"),
        }
    }

    /// A run of zeroes of a given length, leaked once per length asked for.
    ///
    /// A media fixture has to outlive the machine that binds it, and the sizes
    /// wanted here are a socket's, not a constant's — so the array cannot be a
    /// `static`. Leaking a handful of buffers in a test process is the honest
    /// trade against threading a lifetime through the whole fixture table.
    #[cfg(any(feature = "machine-pc-at", feature = "machine-pc-apic"))]
    fn blank(len: usize) -> &'static [u8] {
        use crate::core::sync::Global;
        use alloc::collections::BTreeMap;

        // `Global`, not `Mutex`: this is a `static`, so libtest's threads reach
        // it as readily as one machine build does (`core::sync`). It also keeps
        // `std::sync` out of `machine/`, which CLAUDE.md forbids outright.
        static CACHE: Global<BTreeMap<usize, &'static [u8]>> = Global::new(BTreeMap::new());
        let mut cache = CACHE.lock();
        if let Some(image) = cache.get(&len).copied() {
            return image;
        }
        let image: &'static [u8] = alloc::vec![0u8; len].leak();
        cache.insert(len, image);
        image
    }

    /// The smallest legal Game Boy image: two 16 KiB banks, a correct header
    /// checksum, and a one-instruction program (`JR -2`, the smallest program
    /// that neither ends nor wanders). Generated, never vendored.
    #[cfg(feature = "machine-gameboy")]
    fn minimal_gb() -> &'static [u8] {
        use crate::core::sync::Global;

        // Built once and leaked rather than written out as a `static` array,
        // because the header checksum is computed over the image and the
        // generator is where that lives. `Global` for the same reason as
        // `blank` below.
        static IMAGE: Global<Option<&'static [u8]>> = Global::new(None);
        let mut slot = IMAGE.lock();
        if let Some(image) = *slot {
            return image;
        }
        let image: &'static [u8] =
            crate::dev::gb::cart::synthetic_image(2, 0x00, 0x00, &[0x18, 0xfe]).leak();
        *slot = Some(image);
        image
    }

    /// The smallest legal NROM image: an iNES header, 16 KiB of PRG, 8 KiB of
    /// CHR. Generated, never vendored.
    static MINIMAL_NROM: &[u8] = &{
        let mut image = [0u8; 16 + 16384 + 8192];
        image[0] = b'N';
        image[1] = b'E';
        image[2] = b'S';
        image[3] = 0x1a;
        image[4] = 1; // 16 KiB of PRG
        image[5] = 1; // 8 KiB of CHR
        // A reset vector at $C000 — the 16 KiB of PRG answers at both $8000
        // and $C000 — holding `JMP $C000`, so the program counter after any
        // amount of running is exactly one known number.
        image[16 + 0x3ffc] = 0x00;
        image[16 + 0x3ffd] = 0xc0;
        image[16] = 0x4c;
        image[17] = 0x00;
        image[18] = 0xc0;
        image
    };
}
