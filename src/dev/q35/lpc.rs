//! The q35 south bridge: an ICH9 LPC interface bridge at D31:F0.
//!
//! # What this part is
//!
//! The other half of a q35. Everything a PC has that is not memory or PCI
//! Express hangs off it, and the ICH9 datasheet says so in chapter 13's first
//! paragraph:
//!
//! > The LPC bridge function of the ICH9 resides in PCI Device 31:Function 0.
//! > This function contains many other functional units, such as DMA and
//! > Interrupt controllers, Timers, Power Management, System Management, GPIO,
//! > RTC, and LPC Configuration Registers.
//!
//! **rsemu already has those functional units.** The two 8259As, the 8254, the
//! MC146818, the 8042, the two 8237As, the local and I/O APICs and the HPET are
//! all in [`crate::dev::pc`], they are all separate objects in a machine
//! description, and `machines/q35.machine` wires them exactly as
//! `machines/pc-at.machine` does. Re-implementing them behind this function
//! would be describing the same silicon twice.
//!
//! So what is left for this file is what a q35 adds *around* them, and it is
//! three things:
//!
//! 1. **A PCI function to find.** Class code `060100h` at `00:1f.0`, which is
//!    how software recognises the chipset at all.
//! 2. **PCI interrupt routing.** `PIRQ[A-H]` come in and are steered onto ISA
//!    interrupt lines by eight registers. That is the job the `pc-at` board has
//!    no answer for, and this is the south bridge.
//!
//!    Where the eight inputs come from is worth being precise about, because
//!    they come from two places. Four of them are the **fabric's own interrupt
//!    nets**: a function drives its `INTA#`-`INTD#` pin, [`crate::bus::pci`]
//!    swizzles it by device number onto one of four nets, and this function
//!    registers as that fabric's [`IntxSink`] — which is what `bus = "pci0"` on
//!    this object means, and why a q35 board needs no wire between a card and
//!    its router. All eight are also ordinary wire sinks a machine file may
//!    drive, for a board with something else to hang on one, and a pin driven
//!    from both is the wired-OR of the two: `PIRQ` is a shared, level-sensitive
//!    line and modelling that is the whole difficulty.
//! 3. **The ACPI register block**, decoded at whatever `PMBASE` says
//!    ([`super::pm`]).
//!
//! # The registers, from the datasheet
//!
//! *Intel I/O Controller Hub 9 (ICH9) Family Datasheet*, order number
//! 316972-004, Table 13-1 (`LPC Interface PCI Register Address Map`):
//!
//! ```text
//!   00h-01h  VID           Vendor Identification            8086h
//!   02h-03h  DID           Device Identification            see below
//!   04h-05h  PCICMD        PCI Command                      0007h
//!   06h-07h  PCISTS        PCI Status                       0210h
//!     09h    PI            Programming Interface              00h
//!     0Ah    SCC           Sub Class Code                     01h
//!     0Bh    BCC           Base Class Code                    06h
//!     0Eh    HEADTYP       Header Type                        80h
//!   40h-43h  PMBASE        ACPI Base Address            00000001h
//!     44h    ACPI_CNTL     ACPI Control                       00h
//!   60h-63h  PIRQ[n]_ROUT  PIRQ[A-D] Routing Control    80808080h
//!   68h-6Bh  PIRQ[n]_ROUT  PIRQ[E-H] Routing Control    80808080h
//!   F0h-F3h  RCBA          Root Complex Base Address    00000000h
//! ```
//!
//! ## The device identification is a documented gap
//!
//! §13.1.2 does not give one:
//!
//! > Device ID — RO. This is a 16-bit value assigned to the Intel ICH9 LPC
//! > bridge. Refer to the Intel I/O Controller Hub (ICH9) Family Specification
//! > Update for the value of the Device ID Register.
//!
//! That Specification Update (316973) could not be retrieved from any public
//! mirror while this was written, so **this file does not state a number.**
//! `device-id` is a required property and `machines/q35.machine` writes it,
//! with the same caveat beside it. Nothing functional depends on the value —
//! an operating system finds the chipset by class code and finds its ACPI
//! hardware through the FADT — and inventing one and calling it a datasheet
//! fact would be worse than making a board say out loud what it is claiming to
//! be.
//!
//! ## `PIRQ[n]_ROUT`, §13.1.17 and §13.1.19
//!
//! Eight identical bytes, default `80h`:
//!
//! > 7    Interrupt Routing Enable (IRQEN) — R/W.
//! >      0 = The corresponding PIRQ is routed to one of the ISA-compatible
//! >          interrupts specified in bits\[3:0\].
//! >      1 = The PIRQ is not routed to the 8259.
//! > 6:4  Reserved
//! > 3:0  IRQ Routing — R/W. (ISA compatible.)
//! >      0000b Reserved   1000b Reserved
//! >      0001b Reserved   1001b IRQ9
//! >      0010b Reserved   1010b IRQ10
//! >      0011b IRQ3       1011b IRQ11
//! >      0100b IRQ4       1100b IRQ12
//! >      0101b IRQ5       1101b Reserved
//! >      0110b IRQ6       1110b IRQ14
//! >      0111b IRQ7       1111b IRQ15
//!
//! Note the sense of bit 7: `80h` out of reset means **not** routed, so a board
//! powers up with no PCI interrupt reaching a controller until firmware says
//! where each one goes. That is why the datasheet adds "BIOS must program this
//! bit to 0 during POST for any of the PIRQs that are being used".
//!
//! Eleven output pins, one per legal destination, and an input contributes to
//! whichever its register names. Two `PIRQ`s routed to one `IRQ` is ordinary —
//! that is what sharing a PCI interrupt *is* — so the outputs are the OR of
//! everything routed to them.
//!
//! ## `PMBASE` and `ACPI_CNTL`, §13.1.13 and §13.1.14
//!
//! > 15:7  Base Address — R/W. This field provides 128 bytes of I/O space for
//! >       ACPI, GPIO, and TCO logic. This is placed on a 128-byte boundary.
//! >    0  Resource Type Indicator (RTE) — RO. Hardwired to 1 to indicate I/O
//! >       space.
//!
//! > 7    ACPI Enable (ACPI_EN) — R/W. […] 1 = Decode of the I/O range pointed
//! >      to by the ACPI base register is enabled […]
//! > 2:0  SCI IRQ Select (SCI_IRQ_SEL) — R/W. […] 000b IRQ9, 001b IRQ10,
//! >      010b IRQ11, 011b Reserved, 100b IRQ20, 101b IRQ21
//!
//! The `SCI` is therefore **five output pins**, at most one of them high, and
//! the register picks which. That is the same shape
//! [`crate::dev::pc::imcr`] uses for the same reason — the unselected pin is
//! driven *low* rather than left alone, or switching the selection with the SCI
//! asserted would strand a level high for ever.
//!
//! # Moving the ACPI window from inside a configuration write
//!
//! `PMBASE` and `ACPI_EN` move a window in the **I/O** space, and
//! [`crate::bus::pci::bar`] documents that as the one case the order-exempt
//! try-lock cannot serve — *when the configuration cycle also travels through
//! the I/O space*, which through `0xcf8`/`0xcfc` it does.
//!
//! Through **ECAM** it does not. A q35's configuration cycles arrive as memory
//! accesses ([`super::ecam`]), so the I/O space is not the space the access is
//! inside, the try-lock succeeds, and the window moves on the instant the
//! guest asked for it. That is the concrete reason this board can decode an
//! ACPI block at all and `pc-at` could not have.
//!
//! Both paths are still handled: a `0xcfc` write sets the `stale` flag and the
//! window is placed at the next configuration access of any kind, at reset, and
//! after a snapshot load — the pattern `pmc` and `bar` both use.
//!
//! # What is not modelled
//!
//! `GPIOBASE`/`GC`, `SIRQ_CNTL` and the whole serial IRQ machine,
//! `LPC_I/O_DEC`, `LPC_EN` and the generic decode ranges, the firmware hub
//! selects, `BIOS_CNTL`, the feature-detection capability, and the root complex
//! register block `RCBA` points at. `RCBA` itself is latched and read back
//! because firmware writes it early and reads it back to check, and a register
//! that master-aborted there would stop a POST for no reason; nothing decodes
//! what it names. `LPC_IBDF` — which bus:device:function the I/O APIC answers
//! at — is not modelled either, because this board's I/O APIC is not on the PCI
//! bus at all.
//!
//! # Sources
//!
//! *Intel I/O Controller Hub 9 (ICH9) Family Datasheet*, order number
//! 316972-004: chapter 13's opening, Table 13-1, and §13.1.1-§13.1.19 and
//! §13.1.35 for the individual registers. *PCI Local Bus Specification* Rev 2.1
//! §6.1, §6.2 and Appendix D for the Type 00h header and the class code.
//!
//! No emulator source was consulted (`CLAUDE.md`, provenance).

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::bus::pci::{Bdf, ConfigSpace, INTX_LINES, IntxSink, PciBus, PciFunction, buses, config};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::props::{Props, Value, ValueKind};
use crate::core::sched::LazyHandle;
use crate::core::space::{
    AddressSpace, Mapping, MappingId, MemAttrs, MemOps, Perms, Region, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::core::{Error, Result};
use crate::machine::SinkPin;
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

use super::pm::{self, AcpiBlock, SciSink};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "q35.lpc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// Where the LPC bridge lives: D31:F0 (chapter 13's first sentence).
pub const LPC_DEVICE: u8 = 31;

/// Configuration offset of `PMBASE` (§13.1.13).
const PMBASE: u16 = 0x40;
/// `PMBASE`'s writable address field: bits 15:7, a 128-byte boundary.
const PMBASE_MASK: u32 = 0x0000_ff80;
/// `PMBASE[0]`: hardwired to 1 to indicate I/O space.
const PMBASE_IO: u32 = 1;

/// Configuration offset of `ACPI_CNTL` (§13.1.14).
const ACPI_CNTL: u16 = 0x44;
/// `ACPI_CNTL[7]`: the ACPI I/O range decodes.
const ACPI_EN: u8 = 0x80;
/// `ACPI_CNTL[2:0]`: which interrupt the SCI appears on.
const SCI_IRQ_SEL: u8 = 0x07;
/// Every `ACPI_CNTL` bit a write keeps; 6:3 are reserved.
const ACPI_CNTL_MASK: u8 = ACPI_EN | SCI_IRQ_SEL;

/// Configuration offset of `PIRQ[A-D]_ROUT` (§13.1.17).
const PIRQ_ABCD: u16 = 0x60;
/// Configuration offset of `PIRQ[E-H]_ROUT` (§13.1.19).
const PIRQ_EFGH: u16 = 0x68;
/// Every `PIRQ[n]_ROUT` bit a write keeps; 6:4 are reserved.
const PIRQ_MASK: u8 = 0x8f;
/// `PIRQ[n]_ROUT[7]`: *not* routed to the 8259. Set out of reset.
const PIRQ_DISABLE: u8 = 0x80;

/// Configuration offset of `RCBA` (§13.1.35).
const RCBA: u16 = 0xf0;
/// `RCBA`'s writable bits: the base at 31:14 and the enable at 0.
const RCBA_MASK: u32 = 0xffff_c000 | 1;

/// How many `PIRQ` inputs there are: A through H.
pub const PIRQS: usize = 8;

/// The ISA interrupt each `IRQ Routing` encoding names, or `None` for a
/// reserved one (§13.1.17's table).
const fn routed_irq(encoding: u8) -> Option<u8> {
    match encoding & 0xf {
        0b0011 => Some(3),
        0b0100 => Some(4),
        0b0101 => Some(5),
        0b0110 => Some(6),
        0b0111 => Some(7),
        0b1001 => Some(9),
        0b1010 => Some(10),
        0b1011 => Some(11),
        0b1100 => Some(12),
        0b1110 => Some(14),
        0b1111 => Some(15),
        _ => None,
    }
}

/// The eleven interrupts a `PIRQ` can be routed to, in order.
pub const ROUTABLE: [u8; 11] = [3, 4, 5, 6, 7, 9, 10, 11, 12, 14, 15];

/// The `PIRQ[n]_ROUT` byte that routes a `PIRQ` to ISA interrupt `irq`, or
/// `None` if §13.1.17's table has no encoding for it.
///
/// The inverse of [`routed_irq`], and the two are asserted to agree over the
/// whole byte in this module's tests — a routing table with a hole in one
/// direction only is exactly the bug that puts an interrupt somewhere nobody
/// is listening.
pub(super) const fn route_encoding(irq: u8) -> Option<u8> {
    match irq {
        3 => Some(0b0011),
        4 => Some(0b0100),
        5 => Some(0b0101),
        6 => Some(0b0110),
        7 => Some(0b0111),
        9 => Some(0b1001),
        10 => Some(0b1010),
        11 => Some(0b1011),
        12 => Some(0b1100),
        14 => Some(0b1110),
        15 => Some(0b1111),
        _ => None,
    }
}

/// The interrupt each `SCI_IRQ_SEL` encoding names, or `None` for the reserved
/// one (§13.1.14's table).
const fn sci_irq(encoding: u8) -> Option<u8> {
    match encoding & 0x7 {
        0b000 => Some(9),
        0b001 => Some(10),
        0b010 => Some(11),
        0b100 => Some(20),
        0b101 => Some(21),
        _ => None,
    }
}

/// The five interrupts an SCI can appear on, in order.
pub const SCI_LINES: [u8; 5] = [9, 10, 11, 20, 21];

/// The configuration offset of `PIRQ[index]_ROUT`, A through H.
///
/// Two disjoint runs of four (§13.1.17 and §13.1.19), which is the kind of
/// detail a second reader of the register file would get subtly wrong.
#[must_use]
pub const fn pirq_rout(index: usize) -> u16 {
    if index < 4 {
        PIRQ_ABCD + index as u16
    } else {
        PIRQ_EFGH + (index as u16 - 4)
    }
}

/// The ISA interrupt a `PIRQ[n]_ROUT` byte routes its input to, or `None` if it
/// routes nowhere — bit 7 set, or a reserved encoding (§13.1.17).
#[must_use]
pub const fn pirq_destination(byte: u8) -> Option<u8> {
    if byte & PIRQ_DISABLE != 0 {
        return None;
    }
    routed_irq(byte)
}

/// Where the ACPI window went.
#[derive(Debug, Clone)]
struct Placed {
    space: Arc<AddressSpace>,
    id: Option<MappingId>,
}

/// The bridge's configuration registers, its routing, and the ACPI block.
struct Registers {
    /// The 256 bytes of configuration space. At [`LockRank::DEVICE`], released
    /// before anything outward.
    config: Mutex<ConfigSpace>,
    /// `PMBASE`'s latch: bits 15:7 plus the hardwired indicator, which
    /// [`ConfigSpace`]'s per-byte mask cannot express. [`LockRank::LEAF`].
    pmbase: Mutex<u32>,
    /// `RCBA`'s latch, for the same reason. [`LockRank::LEAF`].
    rcba: Mutex<u32>,
    /// The level each `PIRQ` input is being driven at **by a wire**.
    /// [`LockRank::LEAF`], and read out before any pin is driven.
    pirq_in: Mutex<[bool; PIRQS]>,
    /// The level each of the fabric's four interrupt nets is at.
    ///
    /// Separate from `pirq_in` because they are separate drivers of the same
    /// four pins and a shared line must know which of its drivers let go —
    /// `ROADMAP.md` §4.3's argument for why `set_level` carries a source, one
    /// level up. On this chipset the nets land on `PIRQ[A-D]` with no
    /// rotation: an ICH9 can steer an internal function's `INTx#` onto any of
    /// the eight through the `D<n>IR` registers in the root complex register
    /// block `RCBA` names, and that block is not modelled, so the identity
    /// mapping is what this part does. Derived from what the functions on the
    /// bus are driving, so never serialized. [`LockRank::LEAF`].
    pirq_bus: Mutex<[bool; INTX_LINES as usize]>,
    /// The eleven ISA outputs, in [`ROUTABLE`] order. [`LockRank::LEAF`].
    irq_out: Mutex<[Option<WireSource>; ROUTABLE.len()]>,
    /// The five SCI outputs, in [`SCI_LINES`] order. [`LockRank::LEAF`].
    sci_out: Mutex<[Option<WireSource>; SCI_LINES.len()]>,
    /// The ACPI register block this function decodes at `PMBASE`.
    acpi: Arc<AcpiBlock>,
    /// The region that block answers through, kept so it can be mapped.
    acpi_region: RegionRef,
    /// The I/O space the ACPI window goes in, and where it went. `None` until
    /// [`Instance::bind`]. [`LockRank::LEAF`].
    placed: Mutex<Option<Placed>>,
    /// Set when a retopology could not happen when it was asked for. Derived
    /// state: never serialized, and a load re-applies unconditionally.
    stale: Mutex<bool>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        match self.pmbase.try_lock() {
            Some(v) => s.field("pmbase", &*v),
            None => s.field("pmbase", &"<in use>"),
        };
        s.field("acpi", &self.acpi)
            .field("placed", &self.placed.try_lock().map(|p| p.is_some()))
            .finish()
    }
}

/// Clear the reserved bits of the registers whose whole bytes are writable.
///
/// [`ConfigSpace`]'s write mask is per byte, and `ACPI_CNTL` and the eight
/// `PIRQ[n]_ROUT` bytes each have holes in them (§13.1.14, §13.1.17). Applied
/// after every write and after every snapshot load, so neither route can leave
/// a bit set that the hardware never holds.
fn mask_reserved(c: &mut ConfigSpace) {
    let acpi = c.byte(ACPI_CNTL) & ACPI_CNTL_MASK;
    c.set_byte(ACPI_CNTL, acpi);
    for i in 0..4u16 {
        let abcd = c.byte(PIRQ_ABCD + i) & PIRQ_MASK;
        c.set_byte(PIRQ_ABCD + i, abcd);
        let efgh = c.byte(PIRQ_EFGH + i) & PIRQ_MASK;
        c.set_byte(PIRQ_EFGH + i, efgh);
    }
}

/// Drive one output, with no lock held.
fn drive(out: &Option<WireSource>, level: Level) {
    if let Some(out) = out {
        out.set(level);
    }
}

impl Registers {
    /// The header this part hardwires, from Table 13-1 and §13.1.1-§13.1.10.
    fn fresh_config(device_id: u16, revision: u8, pirq: &[u8; PIRQS]) -> ConfigSpace {
        let mut c = ConfigSpace::new();
        c.hardwire(config::VENDOR_ID, u32::from(config::VENDOR_INTEL), 2);
        c.hardwire(config::DEVICE_ID, u32::from(device_id), 2);
        // Table 13-1: PCICMD default 0007h — I/O space, memory space and bus
        // master, all set, because the LPC bridge decodes all three.
        c.hardwire(config::COMMAND, 0x0007, 2);
        // Table 13-1: PCISTS default 0210h. The capability list bit is *not*
        // set: §13.1.12's CAPP points at a feature-detection capability this
        // model does not implement, and claiming a list a traversal would then
        // walk into would be worse than having none.
        c.hardwire(config::STATUS, 0x0200, 2);
        c.hardwire(config::REVISION_ID, u32::from(revision), 1);
        // §13.1.6-§13.1.8: PI 00h, SCC 01h, BCC 06h — an ISA bridge.
        c.hardwire(config::CLASS_CODE, 0x00, 1);
        c.hardwire(config::CLASS_CODE + 1, 0x01, 1);
        c.hardwire(config::CLASS_CODE + 2, u32::from(config::CLASS_BRIDGE), 1);
        // §13.1.10: HEADTYP 80h — the basic format, and multi-function, which
        // it truthfully is: a real ICH9 has D31:F1 through F6 as well.
        c.hardwire(config::HEADER_TYPE, 0x80, 1);

        // Table 13-1's R/W registers, minus the two with bit-level masks.
        c.allow(ACPI_CNTL, 1);
        c.allow(PIRQ_ABCD, 4);
        c.allow(PIRQ_EFGH, 4);
        // Table 13-1: both PIRQ runs default to 80808080h, and the sense of
        // bit 7 makes that *not routed* rather than routed to interrupt zero.
        // A board powers up with no PCI interrupt reaching a controller at all,
        // which is why §13.1.17 adds "BIOS must program this bit to 0 during
        // POST for any of the PIRQs that are being used".
        //
        // A board may state a different power-up value with `pirq-routes`, and
        // `CLASS`'s summary says why that is a stand-in for firmware rather
        // than a correction to the datasheet.
        for i in 0..4usize {
            c.set_byte(PIRQ_ABCD + i as u16, pirq[i]);
            c.set_byte(PIRQ_EFGH + i as u16, pirq[i + 4]);
        }
        c
    }

    /// The `PIRQ[n]_ROUT` byte for input `index`, A-H.
    fn pirq_route(config: &ConfigSpace, index: usize) -> u8 {
        config.byte(pirq_rout(index))
    }

    /// The level each ISA output should be at, in [`ROUTABLE`] order.
    fn irq_levels(&self) -> [Level; ROUTABLE.len()] {
        let mut inputs = *self.pirq_in.lock();
        let nets = *self.pirq_bus.lock();
        // Wired-OR with the fabric's own nets: a `PIRQ` pin driven by a card
        // through the bus and by a wire from elsewhere on the board is one
        // shared line, and it stays asserted while either holds it.
        for (slot, net) in nets.iter().enumerate() {
            inputs[slot] |= *net;
        }
        let config = self.config.lock();
        let mut high = [false; ROUTABLE.len()];
        for (index, asserted) in inputs.iter().enumerate() {
            if !asserted {
                continue;
            }
            let route = Registers::pirq_route(&config, index);
            if route & PIRQ_DISABLE != 0 {
                // "1 = The PIRQ is not routed to the 8259" (§13.1.17).
                continue;
            }
            let Some(irq) = routed_irq(route) else {
                // A reserved encoding routes nowhere, which is the only
                // answer a table with holes in it can give.
                continue;
            };
            if let Some(slot) = ROUTABLE.iter().position(|i| *i == irq) {
                // Wired-OR: two PIRQs on one IRQ is what sharing a PCI
                // interrupt means.
                high[slot] = true;
            }
        }
        high.map(Level::from_bool)
    }

    /// The level each SCI output should be at, in [`SCI_LINES`] order.
    ///
    /// At most one is high: the unselected ones are driven **low** rather than
    /// left alone, for the reason [`crate::dev::pc::imcr`] gives.
    fn sci_levels(&self) -> [Level; SCI_LINES.len()] {
        let selected = sci_irq(self.config.lock().byte(ACPI_CNTL) & SCI_IRQ_SEL);
        let asserted = self.acpi.sci_asserted();
        SCI_LINES.map(|line| Level::from_bool(asserted && selected == Some(line)))
    }

    /// Put every output pin where the registers say it belongs, with no state
    /// lock held while a wire is driven.
    fn drive_outputs(&self) {
        let irq = self.irq_levels();
        let sci = self.sci_levels();
        let outs = self.irq_out.lock().clone();
        for (out, level) in outs.iter().zip(irq) {
            drive(out, level);
        }
        let outs = self.sci_out.lock().clone();
        for (out, level) in outs.iter().zip(sci) {
            drive(out, level);
        }
    }

    /// One `PIRQ` input moved.
    fn pirq_level(&self, index: usize, level: Level) {
        {
            let mut inputs = self.pirq_in.lock();
            let asserted = level == Level::High;
            if inputs[index] == asserted {
                return;
            }
            inputs[index] = asserted;
        }
        self.drive_outputs();
    }

    /// Where the ACPI window decodes, or `None` if `ACPI_EN` is clear.
    fn acpi_window(&self) -> Option<u64> {
        if self.config.lock().byte(ACPI_CNTL) & ACPI_EN == 0 {
            return None;
        }
        let base = *self.pmbase.lock() & PMBASE_MASK;
        Some(u64::from(base))
    }

    /// Bring the ACPI window into line with the registers. Reports whether it
    /// could be done.
    fn retopo(&self, blocking: bool) -> bool {
        let Some(placed) = self.placed.lock().clone() else {
            // Not bound yet. Not stale either: `bind` places it correctly.
            return true;
        };
        let want = self.acpi_window();
        let guard = if blocking {
            Some(placed.space.topology())
        } else {
            placed.space.try_topology()
        };
        let Some(mut topo) = guard else {
            *self.stale.lock() = true;
            return false;
        };
        let mut id = placed.id;
        if let Some(existing) = id {
            match want {
                // A move, which is what `remap` is for. A base the space
                // cannot drive takes the window out entirely: that is a
                // chipset decoding an address the machine does not have.
                Some(base) => {
                    if topo.remap(existing, base).is_err() {
                        let _ = topo.unmap(existing);
                        id = None;
                    }
                }
                None => {
                    let _ = topo.unmap(existing);
                    id = None;
                }
            }
        }
        if id.is_none()
            && let Some(base) = want
        {
            id = topo
                .map_with(
                    Mapping::new(Arc::clone(&self.acpi_region), base)
                        .with_priority(1)
                        .with_perms(Perms::RW),
                )
                .ok();
        }
        drop(topo);
        *self.placed.lock() = Some(Placed {
            space: Arc::clone(&placed.space),
            id,
        });
        *self.stale.lock() = false;
        true
    }

    /// Claim the I/O space and place the ACPI window if it decodes.
    ///
    /// **Retopology**, and legal here: `bind` runs with no access in flight.
    fn install(&self, space: &Arc<AddressSpace>) {
        *self.placed.lock() = Some(Placed {
            space: Arc::clone(space),
            id: None,
        });
        self.retopo(true);
    }
}

impl PciFunction for Registers {
    fn config_read(&self, offset: u16, dst: &mut [u8], _attrs: MemAttrs) {
        // No `debug` branch: nothing here is cleared or advanced by a read.
        self.config.lock().read(offset, dst);
        let pmbase = (*self.pmbase.lock() | PMBASE_IO).to_le_bytes();
        let rcba = self.rcba.lock().to_le_bytes();
        for (i, slot) in dst.iter_mut().enumerate() {
            let at = offset.saturating_add(i as u16);
            if (PMBASE..PMBASE + 4).contains(&at) {
                *slot = pmbase[usize::from(at - PMBASE)];
            } else if (RCBA..RCBA + 4).contains(&at) {
                *slot = rcba[usize::from(at - RCBA)];
            }
        }
        if *self.stale.lock() {
            self.retopo(false);
        }
    }

    fn config_write(&self, offset: u16, src: &[u8], attrs: MemAttrs) {
        if attrs.debug {
            // A debug write could move the ACPI window or re-route a live
            // interrupt. `ConfigPorts` and `Ecam` both refuse one before it
            // gets here; this is the second lock on the door.
            return;
        }
        let touches = |first: u16, len: u16| {
            offset < first + len && offset.saturating_add(src.len() as u16) > first
        };
        let mut window_moved = false;
        if touches(PMBASE, 4) {
            let mut latch = self.pmbase.lock();
            let mut bytes = latch.to_le_bytes();
            for (i, byte) in src.iter().enumerate() {
                let at = offset.saturating_add(i as u16);
                if (PMBASE..PMBASE + 4).contains(&at) {
                    bytes[usize::from(at - PMBASE)] = *byte;
                }
            }
            let updated = u32::from_le_bytes(bytes) & PMBASE_MASK;
            window_moved |= updated != *latch;
            *latch = updated;
        }
        if touches(RCBA, 4) {
            let mut latch = self.rcba.lock();
            let mut bytes = latch.to_le_bytes();
            for (i, byte) in src.iter().enumerate() {
                let at = offset.saturating_add(i as u16);
                if (RCBA..RCBA + 4).contains(&at) {
                    bytes[usize::from(at - RCBA)] = *byte;
                }
            }
            *latch = u32::from_le_bytes(bytes) & RCBA_MASK;
        }
        {
            let mut c = self.config.lock();
            c.write(offset, src);
            // The reserved bits of the registers whose bytes are writable.
            // `ConfigSpace`'s mask is per byte, and these three have holes in
            // them (§13.1.14, §13.1.17).
            mask_reserved(&mut c);
        }
        window_moved |= touches(ACPI_CNTL, 1);
        if window_moved || *self.stale.lock() {
            self.retopo(false);
        }
        // The routing and the SCI selection both live in bytes this write may
        // have touched, and re-driving a pin that did not move costs a level
        // comparison inside `core::wire`.
        self.drive_outputs();
    }
}

impl IntxSink for Registers {
    fn intx_changed(&self, line: u8, level: Level) {
        {
            let mut nets = self.pirq_bus.lock();
            let Some(slot) = nets.get_mut(line as usize) else {
                return;
            };
            let asserted = level == Level::High;
            if *slot == asserted {
                return;
            }
            *slot = asserted;
        }
        // The lock is released first: driving these outputs reaches an 8259A,
        // an I/O APIC and a processor.
        self.drive_outputs();
    }
}

impl SciSink for Registers {
    fn sci_changed(&self) {
        // The block computes the condition; which of five pins it appears on
        // is `ACPI_CNTL[2:0]`'s answer, which is this register file's.
        self.drive_outputs();
    }
}

/// One `PIRQ` input pin.
#[derive(Debug)]
struct InputPin {
    regs: Arc<Registers>,
    index: usize,
    inputs: FanIn,
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // Wired-OR: two functions sharing one `PIRQ` net is the normal case.
        self.regs
            .pirq_level(self.index, self.inputs.resolve(Resolve::Or));
    }
}

/// The ICH9 LPC interface bridge.
#[derive(Debug)]
pub struct Lpc {
    regs: Arc<Registers>,
    bus: Arc<PciBus>,
    at: Bdf,
    device_id: u16,
    revision: u8,
    /// The value `PMBASE` and `ACPI_CNTL` come out of reset holding. See
    /// [`CLASS`]'s property summary.
    reset_pmbase: u32,
    reset_acpi_en: bool,
    /// The eight `PIRQ[n]_ROUT` bytes the bridge comes out of reset holding.
    /// See [`CLASS`]'s property summary for why a board may want to state one.
    reset_pirq: [u8; PIRQS],
    /// The name of the address space the ACPI window goes in.
    iospace: String,
    /// The sinks handed out by [`Device::sink`], kept alive here: a net holds
    /// only a weak reference to a sink.
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl Lpc {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property this class does not know, a missing
    /// `device-id`, or a value outside its width; [`Error::Config`] if the
    /// `bus` name is already open as something else, or if `pm-base` is not on
    /// a 128-byte boundary.
    pub fn new(props: &Props) -> Result<Lpc> {
        let mut r = props.reader();
        let bus_name = r.or_str("bus", "pci0")?.to_string();
        let device = r.or_range(
            "device",
            u64::from(LPC_DEVICE),
            0..=u64::from(crate::bus::pci::MAX_DEVICE),
        )?;
        // Required, and the module docs say why: §13.1.2 defers the value to a
        // Specification Update this work could not obtain, so the number is the
        // board's claim rather than this file's.
        let device_id = r.require_range("device-id", 0..=0xffffu64)?;
        let revision = r.or_range("revision", 0u64, 0..=255)?;
        let iospace = r.or_str("iospace", "port")?.to_string();
        let pm_base = r.or_size("pm-base", 0)?;
        let routes = r.optional_list("pirq-routes")?.map(<[Value]>::to_vec);
        r.finish()?;
        if pm_base > 0xffff || pm_base % u64::from(pm::BLOCK_LEN as u32) != 0 {
            return Err(Error::Config {
                at: CLASS_NAME.to_string(),
                message: alloc::format!(
                    "`pm-base` is PMBASE's reset value and PMBASE places 128 bytes of I/O space \
                     on a 128-byte boundary (ICH9 datasheet 316972-004 §13.1.13), so {pm_base:#x} \
                     cannot be one"
                ),
            });
        }
        let mut pirq = [PIRQ_DISABLE; PIRQS];
        if let Some(routes) = routes {
            if routes.len() > PIRQS {
                return Err(Error::Config {
                    at: CLASS_NAME.to_string(),
                    message: alloc::format!(
                        "`pirq-routes` is one ISA interrupt per PIRQ input and there are {PIRQS} \
                         of them (PIRQ[A-H]), so a list of {} cannot be one",
                        routes.len()
                    ),
                });
            }
            for (slot, value) in pirq.iter_mut().zip(routes.iter()) {
                let irq = value.to_uint("pirq-routes")?;
                if irq == 0 {
                    // The datasheet's own power-up value: not routed at all.
                    continue;
                }
                let encoding = u8::try_from(irq).ok().and_then(route_encoding);
                let Some(encoding) = encoding else {
                    return Err(Error::Config {
                        at: CLASS_NAME.to_string(),
                        message: alloc::format!(
                            "§13.1.17's IRQ Routing field can name {ROUTABLE:?} and nothing else, \
                             so a PIRQ cannot come up routed to IRQ{irq}; 0 leaves it unrouted, \
                             which is the datasheet's own default"
                        ),
                    });
                };
                *slot = encoding;
            }
        }
        let bus = buses::attach(props, &bus_name)?;
        let at = Bdf::new(0, device as u8, 0)?;
        Ok(Lpc::with_bus(
            bus,
            at,
            device_id as u16,
            revision as u8,
            pm_base as u32,
            pirq,
            iospace,
        ))
    }

    /// The same device, built from a fabric handle a test already has.
    #[must_use]
    pub fn with_bus(
        bus: Arc<PciBus>,
        at: Bdf,
        device_id: u16,
        revision: u8,
        pm_base: u32,
        pirq: [u8; PIRQS],
        iospace: String,
    ) -> Lpc {
        let acpi = pm::block();
        let acpi_region: RegionRef = Arc::new(Region::io(
            ACPI_REGION,
            pm::BLOCK_LEN,
            Arc::clone(&acpi) as Arc<dyn MemOps>,
        ));
        let reset_acpi_en = pm_base != 0;
        let mut config = Registers::fresh_config(device_id, revision, &pirq);
        if reset_acpi_en {
            config.set_byte(ACPI_CNTL, ACPI_EN);
        }
        let regs = Arc::new(Registers {
            config: Mutex::with_rank(LockRank::DEVICE, config),
            pmbase: Mutex::with_rank(LockRank::LEAF, pm_base & PMBASE_MASK),
            rcba: Mutex::with_rank(LockRank::LEAF, 0),
            pirq_in: Mutex::with_rank(LockRank::LEAF, [false; PIRQS]),
            pirq_bus: Mutex::with_rank(LockRank::LEAF, [false; INTX_LINES as usize]),
            irq_out: Mutex::with_rank(LockRank::LEAF, [const { None }; ROUTABLE.len()]),
            sci_out: Mutex::with_rank(LockRank::LEAF, [const { None }; SCI_LINES.len()]),
            acpi,
            acpi_region,
            placed: Mutex::with_rank(LockRank::LEAF, None),
            stale: Mutex::with_rank(LockRank::LEAF, false),
        });
        // The block tells the bridge when its SCI condition moves, because the
        // pin it appears on is `ACPI_CNTL`'s choice and therefore the bridge's.
        // Weak, because the bridge owns the block.
        regs.acpi
            .set_sink(Arc::downgrade(&regs) as Weak<dyn SciSink>);
        Lpc {
            regs,
            bus,
            at,
            device_id,
            revision,
            reset_pmbase: pm_base & PMBASE_MASK,
            reset_acpi_en,
            reset_pirq: pirq,
            iospace,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Where this bridge sits on its fabric.
    #[must_use]
    pub fn address(&self) -> Bdf {
        self.at
    }

    /// The ACPI register block this function decodes.
    #[must_use]
    pub fn acpi(&self) -> &Arc<AcpiBlock> {
        &self.regs.acpi
    }

    /// Where the ACPI window currently decodes, or `None`.
    #[must_use]
    pub fn acpi_base(&self) -> Option<u64> {
        self.regs.acpi_window()
    }

    /// The `PIRQ[n]_ROUT` byte for input `index`, A-H.
    #[must_use]
    pub fn pirq_route(&self, index: usize) -> u8 {
        Registers::pirq_route(&self.regs.config.lock(), index)
    }

    /// Map the ACPI window into `space`. **Retopology.**
    pub fn attach_space(&self, space: &Arc<AddressSpace>) {
        self.regs.install(space);
    }
}

/// The name the ACPI register block's region carries, and the name
/// [`super::acpi`] looks it up by when it fills in the FADT's block pointers.
pub const ACPI_REGION: &str = "q35.lpc.acpi";

/// The `q35.lpc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an Intel ICH9 LPC interface bridge: PCI interrupt routing and the ACPI register block",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the PCI fabric it answers on (default `pci0`)",
        },
        PropertySpec {
            name: "device",
            kind: ValueKind::Uint,
            required: false,
            summary: "the device number it answers at (default 31, which is the part's own)",
        },
        PropertySpec {
            name: "device-id",
            kind: ValueKind::Uint,
            required: true,
            summary: "the device identification. Required and not defaulted: the ICH9 datasheet \
                      §13.1.2 defers the value to a Specification Update, so the board states it",
        },
        PropertySpec {
            name: "revision",
            kind: ValueKind::Uint,
            required: false,
            summary: "the revision identification byte (default 0)",
        },
        PropertySpec {
            name: "iospace",
            kind: ValueKind::Str,
            required: false,
            summary: "the address space the ACPI window is decoded in (default `port`)",
        },
        PropertySpec {
            name: "pirq-routes",
            kind: ValueKind::List,
            required: false,
            summary: "the ISA interrupt each of PIRQ[A-H] comes out of reset routed to, as a \
                      list of up to eight — a stand-in for the POST §13.1.17 requires (\"BIOS \
                      must program this bit to 0 during POST for any of the PIRQs that are being \
                      used\") on a board with no firmware that does it; 0, and the default, is \
                      the datasheet's own 80h, which routes nothing",
        },
        PropertySpec {
            name: "pm-base",
            kind: ValueKind::Size,
            required: false,
            summary: "where PMBASE comes out of reset pointing, with ACPI_EN set — a stand-in for \
                      the firmware initialisation this board does not have; 0 is the datasheet's \
                      own default, which decodes nothing",
        },
    ],
    construct: |props| Ok(Box::new(Lpc::new(props)?)),
};

/// The error for a pin name this device does not have.
fn unknown_pin(port: &str) -> Error {
    Error::Config {
        at: port.to_string(),
        message: String::from(
            "the LPC bridge takes `pirqa`-`pirqh` in, and drives `irq3`-`irq15` (the eleven \
             §13.1.17 allows) and `sci9`, `sci10`, `sci11`, `sci20`, `sci21` out",
        ),
    }
}

impl Device for Lpc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The one outward action: announcing itself onto the fabric.
        self.bus
            .attach(self.at, Arc::clone(&self.regs) as Arc<dyn PciFunction>)?;
        // And claiming the fabric's four interrupt nets, which is the other
        // half of what makes this a south bridge: a function's `INTx#` reaches
        // the bus, the bus swizzles it onto one of `INTA#`-`INTD#`, and those
        // arrive here as `PIRQ[A-D]`. Weak, and announced immediately — a
        // card may already be asserting (`ROADMAP.md` §4.3's realize sweep).
        self.bus
            .set_intx_sink(Arc::downgrade(&self.regs) as Weak<dyn IntxSink>);
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut c = self.regs.config.lock();
            *c = Registers::fresh_config(self.device_id, self.revision, &self.reset_pirq);
            if self.reset_acpi_en {
                c.set_byte(ACPI_CNTL, ACPI_EN);
            }
        }
        *self.regs.pmbase.lock() = self.reset_pmbase;
        *self.regs.rcba.lock() = 0;
        // The `PIRQ` *input* levels stay: they are what the fabric is driving
        // onto the pins, and resetting a router does not reach across a wire
        // and change what feeds it — the argument `pc.imcr`'s reset makes.
        self.regs.acpi.reset();
        self.regs.retopo(true);
        self.regs.drive_outputs();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        // The ACPI block is placed by `PMBASE`, not by a `map` statement, so a
        // machine file never names it. It is answerable anyway, because a test
        // and a debugger both want at it.
        matches!(name, "acpi").then(|| Arc::clone(&self.regs.acpi_region))
    }

    fn is_lazy(&self) -> bool {
        // The PM timer counts this device's own clock domain.
        true
    }

    fn current_tick(&self) -> u64 {
        self.regs.acpi.tick()
    }

    fn advance_to(&self, tick: u64) {
        self.regs.acpi.advance_to(tick);
        // `TMROF_STS` may have come up, and the SCI is this device's pin
        // rather than the block's.
        self.regs.drive_outputs();
        // And a moment with no access in flight, which is the only place an
        // owed retopology of the I/O space can land when the configuration
        // write that owed it arrived through `0xcfc`. `super::mch`'s
        // `advance_to` carries the argument; this is its mirror image, and it
        // costs nothing here because this device is already clocked.
        if *self.regs.stale.lock() {
            self.regs.retopo(false);
        }
    }

    fn next_event_tick(&self) -> Option<u64> {
        self.regs.acpi.next_event_tick()
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        self.regs.acpi.set_lazy(handle);
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let index = match port {
            "pirqa" => 0,
            "pirqb" => 1,
            "pirqc" => 2,
            "pirqd" => 3,
            "pirqe" => 4,
            "pirqf" => 5,
            "pirqg" => 6,
            "pirqh" => 7,
            _ => return None,
        };
        let pin = Arc::new(InputPin {
            regs: Arc::clone(&self.regs),
            index,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: index as u32,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if let Some(rest) = port.strip_prefix("irq")
            && let Ok(irq) = rest.parse::<u8>()
            && let Some(slot) = ROUTABLE.iter().position(|i| *i == irq)
        {
            self.regs.irq_out.lock()[slot] = Some(source);
            return Ok(());
        }
        if let Some(rest) = port.strip_prefix("sci")
            && let Ok(irq) = rest.parse::<u8>()
            && let Some(slot) = SCI_LINES.iter().position(|i| *i == irq)
        {
            self.regs.sci_out.lock()[slot] = Some(source);
            return Ok(());
        }
        Err(unknown_pin(port))
    }

    fn announce(&self, _port: &str) {
        // Every output is idle low out of reset, but a snapshot loaded before
        // the sweep can leave any of them high, and which one is the routing
        // registers' business — so no pin can be announced without the others.
        self.regs.drive_outputs();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bytes(self.regs.config.lock().bytes())?;
        w.write_u32(*self.regs.pmbase.lock())?;
        w.write_u32(*self.regs.rcba.lock())?;
        // The `PIRQ` input levels, for the reason `pc.imcr` saves its own: a
        // snapshot that dropped them would restore a machine with an interrupt
        // pending that nobody is being told about.
        let inputs = *self.regs.pirq_in.lock();
        for asserted in inputs {
            w.write_bool(asserted)?;
        }
        for word in self.regs.acpi.save_state() {
            w.write_u64(word)?;
        }
        w.write_u64(self.regs.acpi.tick())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let config: &[u8] = r.read_bytes()?;
        let pmbase = r.read_u32()?;
        let rcba = r.read_u32()?;
        let mut inputs = [false; PIRQS];
        for slot in &mut inputs {
            *slot = r.read_bool()?;
        }
        let mut words = [0u64; 6];
        for word in &mut words {
            *word = r.read_u64()?;
        }
        let tick = r.read_u64()?;
        {
            let mut c = self.regs.config.lock();
            *c = Registers::fresh_config(self.device_id, self.revision, &self.reset_pirq);
            c.restore(config);
            // Masked exactly as a guest write is, so a hand-written snapshot
            // cannot install bits the hardware could never hold.
            mask_reserved(&mut c);
        }
        *self.regs.pmbase.lock() = pmbase & PMBASE_MASK;
        *self.regs.rcba.lock() = rcba & RCBA_MASK;
        *self.regs.pirq_in.lock() = inputs;
        self.regs.acpi.load_state(words, tick);
        // The window is a function of the registers, so it is rebuilt rather
        // than saved (`CLAUDE.md`: derived state is never serialized).
        self.regs.retopo(true);
        self.regs.drive_outputs();
        Ok(())
    }
}

impl Instance for Lpc {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx
            .space_named(&self.iospace)
            .ok_or_else(|| Error::Config {
                at: String::from(ctx.path()),
                message: alloc::format!(
                    "the LPC bridge decodes the ACPI register block in the I/O space PMBASE names, \
                 and this machine has no space called `{}`: name it with `iospace = \"…\"`",
                    self.iospace
                ),
            })?;
        self.attach_space(space);
        Ok(())
    }
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Lpc::new(props)?)))
}

/// What the validator should know about `q35.lpc`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(
            PropSchema::new("device", ValueKind::Uint)
                .range(0, u64::from(crate::bus::pci::MAX_DEVICE)),
        )
        .prop(PropSchema::new("device-id", ValueKind::Uint).range(0, 0xffff))
        .prop(PropSchema::new("revision", ValueKind::Uint).range(0, 255))
        .prop(PropSchema::new("iospace", ValueKind::Str))
        .prop(PropSchema::new("pm-base", ValueKind::Size))
        .prop(PropSchema::new("pirq-routes", ValueKind::List))
        .region("acpi");
    for name in [
        "pirqa", "pirqb", "pirqc", "pirqd", "pirqe", "pirqf", "pirqg", "pirqh",
    ] {
        schema = schema.port(name, PortDir::In);
    }
    for irq in ROUTABLE {
        schema = schema.port(alloc::format!("irq{irq}"), PortDir::Out);
    }
    for irq in SCI_LINES {
        schema = schema.port(alloc::format!("sci{irq}"), PortDir::Out);
    }
    schema
}
