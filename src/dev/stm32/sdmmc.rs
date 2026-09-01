//! The STM32H7 **SDMMC** host controller.
//!
//! # Which family, and why this one
//!
//! **The H7's SDMMC, from ST's RM0433 (`STM32H742/743/753/750`), §55.** Not the
//! F4's SDIO, which is a different peripheral wearing a similar register map,
//! and the difference is not cosmetic:
//!
//! * The H7's SDMMC has an **internal DMA** — `IDMACTRLR`, `IDMABSIZER`,
//!   `IDMABASE0R`, `IDMABASE1R` — so the controller is a bus master in its own
//!   right and moves a block into guest memory by itself. The F4's SDIO has
//!   none; it raises a request line and an external DMA2 stream does the work.
//!   Nothing in this tree models a DMA2 yet, so an F4 model would be the FIFO
//!   path and a dangling request, and "SDMMC transfers are usually DMA-driven"
//!   would be a sentence in a comment rather than a code path under test.
//! * `SDMMC_CMDR` gained `CMDTRANS` and `CMDSTOP`: on the H7 the command state
//!   machine starts and stops the *data* state machine, where on the F4
//!   software sets `DCTRL.DTEN` itself. Both are accepted here, because a
//!   driver may still set `DTEN` directly.
//! * `WAITRESP = 10b` means "short response, no CRC check" on the H7 (it is
//!   what `R3` and `R4` use). On the F4 it means "no response". A driver
//!   written for one will be told the wrong thing by the other, which is
//!   exactly why this file names its family.
//! * The FIFO is **sixteen** words deep here, thirty-two on the F4, and the
//!   status register's bits 12 to 21 are laid out differently.
//!
//! Everything below is the H7 layout. An F4 SDIO is a sibling file when
//! somebody has a DMA2 to hang off it, not a property of this one.
//!
//! # What it talks to
//!
//! [`crate::dev::sd::SdCard`], through the four calls that model's
//! documentation argues for. **There is not one SD command index anywhere in
//! this file**, which is the check on whether the split is in the right place:
//! a controller forwards an index and an argument and reacts to the *shape* of
//! what comes back, and the moment it needs to know that seventeen means "read
//! a block", the card model is missing something.
//!
//! # The interrupt
//!
//! **One output pin called `irq`, and no interrupt number anywhere in this
//! file.** A Cortex-M's NVIC is inside the core, so there is no
//! interrupt-controller object to sit between a peripheral and the CPU: a board
//! wires the peripheral's pin straight to the core's, and the core publishes
//! `irq0` … `irq239` for external interrupt *n* at exception *n + 16*.
//!
//! ```text
//!   wire sdmmc1.irq -> cpu.irq49
//! ```
//!
//! Forty-nine is the SDMMC1 global interrupt's vector position on an H7, and it
//! is a fact about the *part* rather than about this peripheral — the same
//! block wired into a different device would be a different number. So it
//! belongs in the `.machine` file where the part is chosen, and this device
//! neither knows it nor has a property for it.
//!
//! The line is a **level**: it is `STA & MASK` reduced to "any bit set", driven
//! after every register access. A driver clears it by clearing the flag through
//! `ICR` or by masking it in `MASKR`, which is what the silicon requires and
//! what makes a handler that forgets both loop forever, as it would on hardware.
//!
//! # Time
//!
//! **Deliberately zero**, and this is the decision most worth arguing.
//!
//! * A command completes inside the bus cycle that writes `CMDR`: the response
//!   registers are already loaded and `STA.CMDREND` already set by the time the
//!   write returns. A driver polling `STA` sees it on its first read; a driver
//!   using the interrupt takes it at the next instruction boundary, because the
//!   `irq` output is driven from inside the write.
//! * A **DMA** transfer completes inside the write that starts it. All of
//!   `DLEN` bytes are in guest memory, `DCNTR` is zero and `DATAEND` is set
//!   before the guest's store instruction retires.
//! * A **FIFO** transfer is paced by the guest instead, which is the honest
//!   thing to do and not a compromise. The card streams into the sixteen-word
//!   FIFO as the guest drains it, so the FIFO's occupancy — and therefore
//!   `RXFIFOHF`, `RXFIFOF`, `RXFIFOE` and `TXFIFOE` — is always the real depth
//!   of a real FIFO, never a pretend-infinite buffer. `DCNTR` counts the bytes
//!   the card has not yet handed over, exactly as the silicon's does.
//!
//! Two flags are consequently **unreachable**, and that is a claim rather than
//! an omission: `RXOVERR` cannot happen because the card never runs ahead of
//! the reader, and `TXUNDERR` cannot happen because the writer never runs ahead
//! of the card. On real silicon both are the driver being too slow, which is a
//! property of the wall clock and not of the guest. `DTIMER` is likewise
//! decoded and stored but never expires: `DTIMEOUT` is raised when the *card*
//! has nothing to say, which is the case that actually matters.
//!
//! Giving this device a clock domain is a small change — `CLKCR.CLKDIV` is
//! already decoded and the divider is already there — and would buy a
//! transfer's real duration. Nothing yet asks for it. What is *not* deferred is
//! the seam: the card is asked [`SdCard::is_busy`] rather than assumed idle, so
//! `STA.BUSYD0` comes from the card and starts telling the truth the moment the
//! card does.
//!
//! # Sources
//!
//! * **ST, RM0433**, *STM32H742, STM32H743/753 and STM32H750 Value line
//!   advanced Arm-based 32-bit MCUs*, §55 "SDMMC controller (SDMMC)": the
//!   register map of §55.10, the command and data state machines of §55.5, the
//!   status flags of §55.8.10 and the internal DMA of §55.6.
//! * The SD Association's *Physical Layer Simplified Specification* for what
//!   the other end of the link does — but only through
//!   [`crate::dev::sd`], never here.
//!
//! No emulator source of any licence was consulted, and no operating system's
//! MMC subsystem was read.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::dev::sd::card::{Data, Reply, SdCard};
use crate::dev::sd::slots::{self, Slot};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "stm32.sdmmc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes of address space the register block occupies.
///
/// The 1 KiB aperture an STM32 peripheral gets in the memory map. Only the
/// first `0xc0` bytes decode to anything; the rest reads zero, which is what a
/// reserved word on this part does.
pub const REGISTER_BYTES: u64 = 0x400;

/// How many 32-bit words the data FIFO holds. Sixteen on the H7.
pub const FIFO_WORDS: usize = 16;

/// Where this controller's register lock sits in the ranked order.
///
/// Below [`LockRank::DEVICE`], because the lock **is** held
/// across the call into the card: moving a block is one step, and dropping the
/// register lock in the middle would let a second access interleave with a
/// half-updated `DCNTR`. That is the same argument `bus::usb`'s `HCD_RANK` and
/// `bus::spi`'s `SHIFTER_RANK` make. The ladder is written out on
/// [`crate::dev::sd::slots::SLOT_RANK`].
pub const REGISTER_RANK: LockRank = LockRank::new(0x4d00);

/// The rank of this controller's smaller cells — the interrupt output, the
/// address space its DMA masters and its identity on it.
///
/// Above the register lock because `run_idma` reaches them while it holds it,
/// and below the interrupt wire because the interrupt cell is held across the
/// call that drives it.
const CELL_RANK: LockRank = LockRank::new(0x4f00);

/// The pin names a machine description wires.
pub mod pin {
    /// The interrupt request the NVIC sees: `STA & MASK` reduced to a level.
    pub const IRQ: &str = "irq";
}

// ---------------------------------------------------------------------------
// The register map (RM0433 §55.10)
// ---------------------------------------------------------------------------

const R_POWER: u64 = 0x00;
const R_CLKCR: u64 = 0x04;
const R_ARGR: u64 = 0x08;
const R_CMDR: u64 = 0x0c;
const R_RESPCMDR: u64 = 0x10;
const R_RESP1R: u64 = 0x14;
const R_RESP4R: u64 = 0x20;
const R_DTIMER: u64 = 0x24;
const R_DLENR: u64 = 0x28;
const R_DCTRL: u64 = 0x2c;
const R_DCNTR: u64 = 0x30;
const R_STAR: u64 = 0x34;
const R_ICR: u64 = 0x38;
const R_MASKR: u64 = 0x3c;
const R_ACKTIMER: u64 = 0x40;
const R_IDMACTRLR: u64 = 0x50;
const R_IDMABSIZER: u64 = 0x54;
const R_IDMABASE0R: u64 = 0x58;
const R_IDMABASE1R: u64 = 0x5c;
const R_FIFOR: u64 = 0x80;
const R_FIFOR_END: u64 = R_FIFOR + (FIFO_WORDS as u64) * 4;

// -- POWER ------------------------------------------------------------------

/// `POWER` bits 1:0. `11b` is the only value that powers the card.
const POWER_PWRCTRL: u32 = 0x3;
/// The `PWRCTRL` encoding for "power on".
const POWER_ON: u32 = 0x3;
/// Everything `POWER` defines: `PWRCTRL`, `VSWITCH`, `VSWITCHEN`, `DIRPOL`.
const POWER_MASK: u32 = 0x1f;

// -- CLKCR ------------------------------------------------------------------

/// `CLKCR` bits 9:0, the clock divider.
const CLKCR_CLKDIV: u32 = 0x3ff;
/// Everything `CLKCR` defines up to `SELCLKRX`.
const CLKCR_MASK: u32 = 0x003f_93ff;

// -- CMDR -------------------------------------------------------------------

/// `CMDR` bits 5:0, the command index.
const CMD_INDEX: u32 = 0x3f;
/// `CMDR` bit 6: the CPSM starts the DPSM once the response arrives.
const CMD_TRANS: u32 = 1 << 6;
/// `CMDR` bit 7: the command stops a transfer in flight.
const CMD_STOP: u32 = 1 << 7;
/// `CMDR` bits 9:8, how long a response to wait for.
const CMD_WAITRESP_SHIFT: u32 = 8;
const CMD_WAITRESP_MASK: u32 = 0x3;
/// `CMDR` bit 12: enable the command state machine.
const CMD_CPSMEN: u32 = 1 << 12;
/// Everything `CMDR` defines.
const CMD_MASK: u32 = 0x0001_ffff;

/// `WAITRESP = 00b`: no response at all, and `CMDSENT` when the command is out.
pub const WAITRESP_NONE: u32 = 0b00;
/// `WAITRESP = 01b`: a 48-bit response whose CRC is checked.
pub const WAITRESP_SHORT: u32 = 0b01;
/// `WAITRESP = 10b`: a 48-bit response whose CRC is **not** checked, which is
/// what `R3` and `R4` need because they carry none. The F4's SDIO gives this
/// encoding a different meaning; see the module note.
pub const WAITRESP_SHORT_NOCRC: u32 = 0b10;
/// `WAITRESP = 11b`: a 136-bit response.
pub const WAITRESP_LONG: u32 = 0b11;

/// How many bits the command state machine waits for.
///
/// The two-bit `WAITRESP` field with its two short encodings collapsed, which
/// is what makes the response-shape match below exhaustive rather than needing
/// an unreachable arm for a value the mask forbids.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Shape {
    /// Nothing. The CPSM reports `CMDSENT` and hears whatever the card said as
    /// silence.
    None,
    /// 48 bits, with or without the CRC check.
    Short,
    /// 136 bits.
    Long,
}

impl Shape {
    fn of(waitresp: u32) -> Shape {
        match waitresp & CMD_WAITRESP_MASK {
            WAITRESP_NONE => Shape::None,
            WAITRESP_SHORT | WAITRESP_SHORT_NOCRC => Shape::Short,
            _ => Shape::Long,
        }
    }
}

// -- DCTRL ------------------------------------------------------------------

/// `DCTRL` bit 0: enable the data state machine.
const DCTRL_DTEN: u32 = 1 << 0;
/// `DCTRL` bit 1: `0` is controller to card, `1` is card to controller.
const DCTRL_DTDIR: u32 = 1 << 1;
/// `DCTRL` bits 7:4, the base-two logarithm of the block size.
const DCTRL_DBLOCKSIZE_SHIFT: u32 = 4;
const DCTRL_DBLOCKSIZE_MASK: u32 = 0xf;
/// `DCTRL` bit 13: reset the FIFO. Self-clearing.
const DCTRL_FIFORST: u32 = 1 << 13;
/// Everything `DCTRL` defines.
const DCTRL_MASK: u32 = 0x3fff;

// -- STA / ICR / MASK (RM0433 §55.10.11) ------------------------------------

/// Command response CRC failed.
const STA_CCRCFAIL: u32 = 1 << 0;
/// Data block CRC failed.
const STA_DCRCFAIL: u32 = 1 << 1;
/// Command response timeout.
const STA_CTIMEOUT: u32 = 1 << 2;
/// Data timeout.
const STA_DTIMEOUT: u32 = 1 << 3;
/// Transmit FIFO underrun.
const STA_TXUNDERR: u32 = 1 << 4;
/// Receive FIFO overrun.
const STA_RXOVERR: u32 = 1 << 5;
/// Command response received, CRC passed.
const STA_CMDREND: u32 = 1 << 6;
/// Command sent, no response required.
const STA_CMDSENT: u32 = 1 << 7;
/// Data end: `DCNTR` reached zero.
const STA_DATAEND: u32 = 1 << 8;
/// Data transfer held.
const STA_DHOLD: u32 = 1 << 9;
/// Data block sent or received, CRC passed.
const STA_DBCKEND: u32 = 1 << 10;
/// Data transfer aborted by `CMD12`.
const STA_DABORT: u32 = 1 << 11;
/// The data state machine is running. Read-only, not clearable.
const STA_DPSMACT: u32 = 1 << 12;
/// The command state machine is running. Read-only, not clearable.
const STA_CPSMACT: u32 = 1 << 13;
/// The transmit FIFO is half empty or better.
const STA_TXFIFOHE: u32 = 1 << 14;
/// The receive FIFO is half full or better.
const STA_RXFIFOHF: u32 = 1 << 15;
/// The transmit FIFO is full.
const STA_TXFIFOF: u32 = 1 << 16;
/// The receive FIFO is full.
const STA_RXFIFOF: u32 = 1 << 17;
/// The transmit FIFO is empty.
const STA_TXFIFOE: u32 = 1 << 18;
/// The receive FIFO is empty.
const STA_RXFIFOE: u32 = 1 << 19;
/// The card is holding DAT0 low.
const STA_BUSYD0: u32 = 1 << 20;
/// DAT0 was released. Clearable.
const STA_BUSYD0END: u32 = 1 << 21;
/// An SDIO interrupt arrived on DAT1.
const STA_SDIOIT: u32 = 1 << 22;
/// The boot acknowledgement was wrong.
const STA_ACKFAIL: u32 = 1 << 23;
/// The boot acknowledgement timed out.
const STA_ACKTIMEOUT: u32 = 1 << 24;
/// The voltage switch finished.
const STA_VSWEND: u32 = 1 << 25;
/// The clock is stopped for a voltage switch.
const STA_CKSTOP: u32 = 1 << 26;
/// An internal DMA transfer error.
const STA_IDMATE: u32 = 1 << 27;
/// An internal DMA buffer transfer completed.
const STA_IDMABTC: u32 = 1 << 28;

/// The bits `ICR` clears. Everything else in `STA` is computed, not latched.
const ICR_MASK: u32 = STA_CCRCFAIL
    | STA_DCRCFAIL
    | STA_CTIMEOUT
    | STA_DTIMEOUT
    | STA_TXUNDERR
    | STA_RXOVERR
    | STA_CMDREND
    | STA_CMDSENT
    | STA_DATAEND
    | STA_DHOLD
    | STA_DBCKEND
    | STA_DABORT
    | STA_BUSYD0END
    | STA_SDIOIT
    | STA_ACKFAIL
    | STA_ACKTIMEOUT
    | STA_VSWEND
    | STA_CKSTOP
    | STA_IDMATE
    | STA_IDMABTC;

/// The bits that live in the latch rather than being derived from the FIFO.
const STA_LATCHED: u32 = ICR_MASK;

/// The bits computed on every read from the FIFO's occupancy, the data state
/// machine and the card, rather than latched.
const STA_DERIVED: u32 = STA_DPSMACT
    | STA_CPSMACT
    | STA_TXFIFOHE
    | STA_RXFIFOHF
    | STA_TXFIFOF
    | STA_RXFIFOF
    | STA_TXFIFOE
    | STA_RXFIFOE
    | STA_BUSYD0;

// A bit is one or the other and never both: a derived bit that `ICR` could
// clear would come straight back, and a latched bit that a read recomputed
// would never be clearable. The one place this could go wrong silently is a
// typo in a shift, so it is checked at compile time.
const _: () = assert!(STA_LATCHED & STA_DERIVED == 0);
const _: () = assert!(STA_LATCHED | STA_DERIVED == 0x1fff_ffff);

/// Everything `MASKR` can enable.
///
/// Every clearable flag, plus the four FIFO **level** flags — which are not
/// clearable, because they are a level rather than an event, and *are*
/// interrupt sources: a driver moving a block through the FIFO under interrupt
/// enables `RXFIFOHFIE` and does nothing else. Leaving them out would silently
/// drop that write and the interrupt would never fire.
///
/// Not maskable: `DPSMACT` and `CPSMACT` (bits 12 and 13) and `BUSYD0` (bit
/// 20), which are states rather than requests, and the two "FIFO full" flags.
const MASK_MASK: u32 = ICR_MASK | STA_TXFIFOHE | STA_RXFIFOHF | STA_TXFIFOE | STA_RXFIFOE;

// -- IDMA -------------------------------------------------------------------

/// `IDMACTRLR` bit 0: the internal DMA moves the data.
const IDMA_EN: u32 = 1 << 0;
/// `IDMACTRLR` bit 1: double-buffer mode, alternating between the two bases.
const IDMA_BMODE: u32 = 1 << 1;
/// `IDMACTRLR` bit 2: which buffer is in use. Read-only to software.
const IDMA_BACT: u32 = 1 << 2;
/// Everything software may write into `IDMACTRLR`.
const IDMA_WRITABLE: u32 = IDMA_EN | IDMA_BMODE;

/// `IDMABSIZER` bits 12:5, the buffer size in units of eight double words.
const IDMABSIZE_SHIFT: u32 = 5;
const IDMABSIZE_MASK: u32 = 0xff;
/// How many bytes one unit of `IDMABNDT` is: eight 32-bit transfers.
const IDMABSIZE_UNIT: u32 = 32;

/// `DLENR` bits 24:0.
const DLEN_MASK: u32 = 0x01ff_ffff;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// The data state machine, while it is running.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Dpsm {
    /// Card to controller.
    to_host: bool,
    /// Bytes the card still owes the FIFO, or still expects from it. This is
    /// `DCNTR`.
    left: u32,
    /// The block size the transfer is counted in.
    block: u32,
    /// Bytes left in the block being moved, for `DBCKEND`.
    block_left: u32,
    /// Whether any byte has moved yet.
    ///
    /// The DPSM can legitimately be armed before the card has anything to say:
    /// the F4-style sequence sets `DCTRL.DTEN` and *then* sends the read
    /// command, and on real silicon the data state machine simply waits on DAT.
    /// So a card with nothing to give is "not yet", while a card that stops
    /// mid-transfer is a data timeout, and this is what tells them apart.
    started: bool,
}

/// Everything the register block holds.
#[derive(Debug)]
struct Regs {
    power: u32,
    clkcr: u32,
    arg: u32,
    cmd: u32,
    respcmd: u32,
    resp: [u32; 4],
    dtimer: u32,
    dlen: u32,
    dctrl: u32,
    acktimer: u32,
    idmactrl: u32,
    idmabsize: u32,
    idmabase: [u32; 2],
    /// The latched half of `STA`. The FIFO level bits are computed.
    sta: u32,
    mask: u32,
    /// The data FIFO, oldest word first.
    fifo: VecDeque<u32>,
    /// The transfer in flight, if any.
    dpsm: Option<Dpsm>,
}

impl Regs {
    fn reset() -> Regs {
        Regs {
            power: 0,
            clkcr: 0,
            arg: 0,
            cmd: 0,
            respcmd: 0,
            resp: [0; 4],
            dtimer: 0,
            dlen: 0,
            dctrl: 0,
            acktimer: 0,
            idmactrl: 0,
            idmabsize: 0,
            idmabase: [0; 2],
            sta: 0,
            mask: 0,
            fifo: VecDeque::with_capacity(FIFO_WORDS),
            dpsm: None,
        }
    }

    fn powered(&self) -> bool {
        self.power & POWER_PWRCTRL == POWER_ON
    }

    /// `STA` as a guest reads it: the latch, plus the bits derived from the
    /// FIFO and the two state machines.
    fn status(&self, card_busy: bool) -> u32 {
        let mut sta = self.sta & STA_LATCHED;
        let level = self.fifo.len();
        if level == 0 {
            sta |= STA_TXFIFOE | STA_RXFIFOE;
        }
        if level >= FIFO_WORDS {
            sta |= STA_TXFIFOF | STA_RXFIFOF;
        }
        if level >= FIFO_WORDS / 2 {
            sta |= STA_RXFIFOHF;
        }
        if level <= FIFO_WORDS / 2 {
            sta |= STA_TXFIFOHE;
        }
        if self.dpsm.is_some() {
            sta |= STA_DPSMACT;
        }
        if card_busy {
            sta |= STA_BUSYD0;
        }
        // CPSMACT is never set: a command completes inside the write to CMDR.
        sta
    }

    fn dcount(&self) -> u32 {
        self.dpsm.map_or(0, |d| d.left)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The STM32H7 SDMMC host controller.
pub struct Sdmmc {
    shared: Arc<Shared>,
    region: RegionRef,
}

impl fmt::Debug for Sdmmc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Sdmmc")
            .field("slot", &self.shared.slot_name)
            .finish_non_exhaustive()
    }
}

/// What both halves of the device reach.
struct Shared {
    regs: Mutex<Regs>,
    /// The socket the card is in. Always present; possibly empty.
    slot: Arc<Slot>,
    slot_name: String,
    /// The interrupt output, once a `wire` statement connects it.
    irq: Mutex<Option<WireSource>>,
    /// The address space the internal DMA traverses, and who we are on it.
    /// `None` until `bind`, and legitimately `None` forever on a board that
    /// only uses the FIFO path.
    bus: Mutex<Option<Arc<AddressSpace>>>,
    requester: Mutex<RequesterId>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("slot", &self.slot_name)
            .finish_non_exhaustive()
    }
}

impl Sdmmc {
    /// Validate `props` and allocate the controller.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is missing or of the wrong kind.
    pub fn new(props: &Props) -> Result<Sdmmc> {
        let mut r = props.reader();
        let slot_name = r.or_str("slot", crate::dev::sd::DEFAULT_SLOT)?.to_string();
        r.finish()?;
        // Acquiring the socket is allocation, not an outward action: the card
        // may not have been constructed yet, and whichever end runs first
        // creates the rendezvous (`core::hosts`).
        let slot = slots::attach(props, &slot_name)?;
        Ok(Sdmmc::with_slot(slot, slot_name))
    }

    /// Build a controller around a socket that already exists.
    #[must_use]
    pub fn with_slot(slot: Arc<Slot>, slot_name: String) -> Sdmmc {
        let shared = Arc::new(Shared {
            regs: Mutex::with_rank(REGISTER_RANK, Regs::reset()),
            slot,
            slot_name,
            irq: Mutex::with_rank(CELL_RANK, None),
            bus: Mutex::with_rank(CELL_RANK, None),
            requester: Mutex::with_rank(CELL_RANK, RequesterId::ANONYMOUS),
        });
        let port = Arc::new(Port {
            shared: Arc::clone(&shared),
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_BYTES,
            port as Arc<dyn MemOps>,
        ));
        Sdmmc { shared, region }
    }

    /// The socket this controller is wired to.
    #[must_use]
    pub fn slot(&self) -> &Arc<Slot> {
        &self.shared.slot
    }

    /// Connect the interrupt request line.
    pub fn attach_irq(&self, source: WireSource) {
        *self.shared.irq.lock() = Some(source);
        self.shared.refresh_irq();
    }

    /// Give the internal DMA an address space to master, and an identity on it.
    pub fn attach_bus(&self, space: Arc<AddressSpace>, requester: RequesterId) {
        *self.shared.bus.lock() = Some(space);
        *self.shared.requester.lock() = requester;
    }

    /// `STA` as a guest would read it, without disturbing anything.
    #[must_use]
    pub fn status(&self) -> u32 {
        let busy = self.shared.card_busy();
        self.shared.regs.lock().status(busy)
    }

    /// `CLKCR.CLKDIV`: the divider from the kernel clock to `SDMMC_CK`.
    ///
    /// Decoded and stored but not otherwise used, because this device has no
    /// clock domain — see the module note on time. It is the number a timed
    /// version would count a transfer in, and it is read back so a driver that
    /// programs it and checks sees what it wrote.
    #[must_use]
    pub fn clock_divider(&self) -> u32 {
        self.shared.regs.lock().clkcr & CLKCR_CLKDIV
    }
}

impl Shared {
    fn card(&self) -> Option<Arc<SdCard>> {
        self.slot.card()
    }

    fn card_busy(&self) -> bool {
        self.card().is_some_and(|c| c.is_busy())
    }

    /// Drive `irq` from `STA & MASK`.
    ///
    /// Called with **no** lock held: driving a wire reaches into whatever is
    /// listening, and the re-entrancy contract says to release first
    /// (`CLAUDE.md`, "Concurrency").
    fn refresh_irq(&self) {
        let asserted = {
            let busy = self.card_busy();
            let regs = self.regs.lock();
            regs.status(busy) & regs.mask & MASK_MASK != 0
        };
        if let Some(irq) = self.irq.lock().as_ref() {
            irq.set(Level::from(asserted));
        }
    }

    // -- the command state machine -----------------------------------------

    /// Run one command, as a write to `CMDR` with `CPSMEN` set does.
    ///
    /// The card is passed in rather than looked up, because the socket ranks
    /// *above* the register lock: it is read once, before the registers are
    /// touched, and released. `None` is an empty socket.
    fn run_command(&self, regs: &mut Regs, card: Option<&SdCard>) {
        let index = (regs.cmd & CMD_INDEX) as u8;
        let waitresp = (regs.cmd >> CMD_WAITRESP_SHIFT) & CMD_WAITRESP_MASK;
        let arg = regs.arg;

        if !regs.powered() {
            // No clock on the bus, so nothing answers. This is also what an
            // empty socket looks like, and a driver cannot tell them apart on
            // real hardware either.
            regs.sta |= STA_CTIMEOUT;
            return;
        }
        let Some(card) = card else {
            regs.sta |= STA_CTIMEOUT;
            return;
        };

        // CMDSTOP means this command aborts a transfer in flight. The card is
        // told by the command itself; this is the controller's own half.
        if regs.cmd & CMD_STOP != 0 && regs.dpsm.is_some() {
            Self::finish(regs);
            regs.fifo.clear();
            regs.sta |= STA_DABORT;
        }

        let reply = card.command(index, arg);
        match (Shape::of(waitresp), reply) {
            (Shape::None, _) => {
                // The CPSM did not wait for anything, so whatever the card
                // said went unheard. That is the silicon's behaviour and it is
                // why a driver must set WAITRESP correctly.
                regs.sta |= STA_CMDSENT;
            }
            (_, Reply::None) => regs.sta |= STA_CTIMEOUT,
            (Shape::Long, Reply::Long(words)) => {
                // R2 has no command index; the corresponding field is all ones
                // and that is what RESPCMDR latches.
                regs.respcmd = CMD_INDEX;
                regs.resp = words;
                regs.sta |= STA_CMDREND;
            }
            (Shape::Long, Reply::Short { .. }) => {
                // 48 bits arrived where 136 were expected: the CPSM keeps
                // waiting and eventually gives up.
                regs.sta |= STA_CTIMEOUT;
            }
            // Both short encodings, WAITRESP 01b and 10b. The only difference
            // is whether the CPSM checks the CRC, and this card never sends a
            // bad one.
            (Shape::Short, Reply::Short { index, value, .. }) => {
                regs.respcmd = u32::from(index) & CMD_INDEX;
                regs.resp[0] = value;
                regs.sta |= STA_CMDREND;
            }
            (Shape::Short, Reply::Long(_)) => {
                // 136 bits arrived where 48 were expected, so the bits the CPSM
                // sampled as a CRC are not one.
                regs.sta |= STA_CCRCFAIL;
            }
        }

        // On the H7 the CPSM starts the DPSM, rather than software setting
        // DTEN itself (RM0433 §55.5.4). Both routes end up here.
        if regs.cmd & CMD_TRANS != 0 && regs.sta & STA_CMDREND != 0 {
            self.start_data(regs, card);
        } else {
            // The other route: `DTEN` was set first and the data state machine
            // has been waiting on DAT for the card to start talking. It has
            // now been asked to.
            self.pump(regs, card);
        }
    }

    // -- the data state machine --------------------------------------------

    /// Start a transfer, as `DTEN` or a `CMDTRANS` command does.
    fn start_data(&self, regs: &mut Regs, card: &SdCard) {
        let len = regs.dlen & DLEN_MASK;
        let to_host = regs.dctrl & DCTRL_DTDIR != 0;
        let shift = (regs.dctrl >> DCTRL_DBLOCKSIZE_SHIFT) & DCTRL_DBLOCKSIZE_MASK;
        // DBLOCKSIZE is the base-two logarithm, capped at 14 (16 KiB).
        let block = 1u32 << shift.min(14);
        if len == 0 {
            Self::finish(regs);
            return;
        }
        regs.fifo.clear();
        regs.dpsm = Some(Dpsm {
            to_host,
            left: len,
            block,
            block_left: block.min(len),
            started: false,
        });
        self.pump(regs, card);
    }

    /// Move whatever the transfer in flight can move right now.
    ///
    /// Idempotent and re-entrant-safe: it is called when the transfer is armed,
    /// when a command completes, and on every FIFO access, and it does nothing
    /// when there is nothing to do. That is what lets both the H7 sequence
    /// (`CMDTRANS` starts the DPSM) and the older one (`DTEN` first, command
    /// second) work without either being special-cased.
    fn pump(&self, regs: &mut Regs, card: &SdCard) {
        let Some(dpsm) = regs.dpsm else { return };
        if regs.idmactrl & IDMA_EN != 0 {
            self.run_idma(regs, card);
        } else if dpsm.to_host {
            self.fill_fifo(regs, card);
        } else {
            self.drain_fifo(regs, card);
        }
    }

    /// Pull bytes from the card into the FIFO until one or the other is done.
    fn fill_fifo(&self, regs: &mut Regs, card: &SdCard) {
        while regs.fifo.len() < FIFO_WORDS {
            let Some(dpsm) = regs.dpsm else { return };
            if dpsm.left == 0 {
                return;
            }
            let run = dpsm.left.min(4) as usize;
            let mut word = [0u8; 4];
            if card.read_data(&mut word[..run]) == Data::Ended {
                Self::stalled(regs);
                return;
            }
            regs.fifo.push_back(u32::from_le_bytes(word));
            Self::advance(regs, run as u32);
        }
    }

    /// Push whatever the FIFO holds at the card.
    fn drain_fifo(&self, regs: &mut Regs, card: &SdCard) {
        while !regs.fifo.is_empty() {
            let Some(dpsm) = regs.dpsm else { return };
            if dpsm.left == 0 {
                return;
            }
            let word = regs.fifo.pop_front().expect("not empty");
            let run = dpsm.left.min(4) as usize;
            if card.write_data(&word.to_le_bytes()[..run]) == Data::Ended {
                Self::stalled(regs);
                return;
            }
            Self::advance(regs, run as u32);
        }
    }

    /// End the data path, clearing `DCTRL.DTEN` as the hardware does.
    ///
    /// RM0433 §55.10.8: `DTEN` "is cleared by hardware when the data transfer
    /// completes". A model that left it set would refuse to arm the *next*
    /// transfer, because software writes `DTEN` again and the bit never made an
    /// edge.
    fn finish(regs: &mut Regs) {
        regs.dpsm = None;
        regs.dctrl &= !DCTRL_DTEN;
    }

    /// The card had nothing to say.
    ///
    /// Before the first byte that is the ordinary state of a data path waiting
    /// on DAT for a command that has not been sent yet, and the DPSM stays
    /// armed. After it, the card has stopped mid-transfer, which is silence
    /// past `DTIMER`.
    fn stalled(regs: &mut Regs) {
        if regs.dpsm.is_some_and(|d| d.started) {
            Self::finish(regs);
            regs.sta |= STA_DTIMEOUT;
        }
    }

    /// Charge `moved` bytes to the transfer, raising `DBCKEND` and `DATAEND`
    /// where the counters say to.
    fn advance(regs: &mut Regs, moved: u32) {
        let Some(dpsm) = regs.dpsm.as_mut() else {
            return;
        };
        dpsm.started = true;
        dpsm.left -= moved;
        dpsm.block_left -= moved.min(dpsm.block_left);
        if dpsm.block_left == 0 {
            regs.sta |= STA_DBCKEND;
            let block = dpsm.block;
            dpsm.block_left = block.min(dpsm.left);
        }
        if dpsm.left == 0 {
            regs.sta |= STA_DATAEND;
            Self::finish(regs);
        }
    }

    /// Move the whole transfer through the internal DMA.
    ///
    /// The controller is a bus master here: it reaches guest memory through the
    /// address space a `space =` statement bound, under its own
    /// [`RequesterId`], exactly as any other DMA engine in this tree does.
    fn run_idma(&self, regs: &mut Regs, card: &SdCard) {
        let Some(dpsm) = regs.dpsm else { return };
        let space = self.bus.lock().clone();
        let Some(space) = space else {
            // IDMAEN with no address space bound is a machine-file mistake, and
            // the flag that says so already exists.
            Self::finish(regs);
            regs.sta |= STA_IDMATE;
            return;
        };
        let attrs = MemAttrs {
            requester: *self.requester.lock(),
            privileged: true,
            ..MemAttrs::DEFAULT
        };
        let double = regs.idmactrl & IDMA_BMODE != 0;
        let buffer_bytes = ((regs.idmabsize >> IDMABSIZE_SHIFT) & IDMABSIZE_MASK) * IDMABSIZE_UNIT;
        // In double-buffer mode the transfer alternates between the two bases
        // every IDMABSIZER bytes; in single-buffer mode it is one contiguous
        // run at IDMABASE0R and IDMABSIZER is not consulted at all.
        if double && buffer_bytes == 0 {
            Self::finish(regs);
            regs.sta |= STA_IDMATE;
            return;
        }
        let mut which = usize::from(regs.idmactrl & IDMA_BACT != 0);
        let mut at = regs.idmabase[if double { which } else { 0 }];
        let mut in_buffer = 0u32;
        // A chunk is one block, so DBCKEND lands where the silicon puts it and
        // a large DLEN does not allocate a large buffer.
        let chunk = dpsm.block.min(dpsm.left).max(1) as usize;
        let mut buf = alloc::vec![0u8; chunk];

        while regs.dpsm.is_some_and(|d| d.left > 0) {
            let dpsm = regs.dpsm.expect("still running");
            let mut run = (dpsm.left as usize).min(chunk);
            if double {
                run = run.min((buffer_bytes - in_buffer) as usize);
            }
            let slice = &mut buf[..run];
            // Which end failed decides which flag is raised, so the two are
            // kept apart rather than folded into one `ok`: a card that has
            // nothing to say is a data path still waiting on DAT, while an
            // address space that refuses is an IDMA transfer error.
            let (card_ok, space_ok) = if dpsm.to_host {
                let card_ok = card.read_data(slice) != Data::Ended;
                let space_ok = !card_ok || space.write_bytes(u64::from(at), slice, attrs).is_ok();
                (card_ok, space_ok)
            } else {
                let space_ok = space.read_bytes(u64::from(at), slice, attrs).is_ok();
                let card_ok = !space_ok || card.write_data(slice) != Data::Ended;
                (card_ok, space_ok)
            };
            if !card_ok {
                Self::stalled(regs);
                break;
            }
            if !space_ok {
                Self::finish(regs);
                regs.sta |= STA_IDMATE;
                break;
            }
            at += run as u32;
            in_buffer += run as u32;
            Self::advance(regs, run as u32);
            if double && in_buffer == buffer_bytes {
                regs.sta |= STA_IDMABTC;
                which ^= 1;
                at = regs.idmabase[which];
                in_buffer = 0;
            }
        }
        if double {
            regs.idmactrl = (regs.idmactrl & !IDMA_BACT) | ((which as u32) << 2);
        }
    }

    // -- the register block ------------------------------------------------

    fn read_register(&self, offset: u64, debug: bool) -> u32 {
        if (R_FIFOR..R_FIFOR_END).contains(&offset) {
            return self.read_fifo(debug);
        }
        let busy = self.card_busy();
        let regs = self.regs.lock();
        match offset {
            R_POWER => regs.power,
            R_CLKCR => regs.clkcr,
            R_ARGR => regs.arg,
            R_CMDR => regs.cmd,
            R_RESPCMDR => regs.respcmd,
            R_RESP1R..=R_RESP4R => regs.resp[((offset - R_RESP1R) / 4) as usize],
            R_DTIMER => regs.dtimer,
            R_DLENR => regs.dlen,
            R_DCTRL => regs.dctrl & !DCTRL_FIFORST,
            R_DCNTR => regs.dcount(),
            R_STAR => regs.status(busy),
            // ICR reads back the bits it would clear, which is what the
            // reference manual's reset value and access column say.
            R_ICR => regs.sta & ICR_MASK,
            R_MASKR => regs.mask,
            R_ACKTIMER => regs.acktimer,
            R_IDMACTRLR => regs.idmactrl,
            R_IDMABSIZER => regs.idmabsize,
            R_IDMABASE0R => regs.idmabase[0],
            R_IDMABASE1R => regs.idmabase[1],
            // Reserved. Zero, which is what this part answers — and the whole
            // register file is otherwise decoded, so a hole here is a hole in
            // the manual rather than a hole in the model.
            _ => 0,
        }
    }

    /// Pop the FIFO, or peek at it for a debugger.
    ///
    /// The rule this device exists to demonstrate: a debugger reading `FIFOR`
    /// must not consume a word, and must not let the card stream another one in
    /// behind it (`ROADMAP.md` §15, invariant 5).
    fn read_fifo(&self, debug: bool) -> u32 {
        if debug {
            return self.regs.lock().fifo.front().copied().unwrap_or(0);
        }
        let card = self.card();
        let mut regs = self.regs.lock();
        let word = regs.fifo.pop_front().unwrap_or(0);
        if let Some(card) = card.as_deref() {
            // Draining made room, so the card streams the next words in. This
            // is what keeps the modelled FIFO exactly sixteen words deep.
            self.pump(&mut regs, card);
        }
        word
    }

    fn write_register(&self, offset: u64, value: u32) {
        if (R_FIFOR..R_FIFOR_END).contains(&offset) {
            self.write_fifo(value);
            return;
        }
        let card = self.card();
        let mut regs = self.regs.lock();
        match offset {
            R_POWER => {
                let was_on = regs.powered();
                regs.power = value & POWER_MASK;
                if was_on && !regs.powered() {
                    // Taking the supply away resets the card, which is exactly
                    // what this register is for: a driver that cannot get a
                    // card to answer power-cycles it and starts again.
                    if let Some(card) = card.as_deref() {
                        card.power_cycle();
                    }
                    Self::finish(&mut regs);
                    regs.fifo.clear();
                }
            }
            R_CLKCR => regs.clkcr = value & CLKCR_MASK,
            R_ARGR => regs.arg = value,
            R_CMDR => {
                regs.cmd = value & CMD_MASK;
                if value & CMD_CPSMEN != 0 {
                    self.run_command(&mut regs, card.as_deref());
                }
            }
            R_DTIMER => regs.dtimer = value,
            R_DLENR => regs.dlen = value & DLEN_MASK,
            R_DCTRL => {
                if value & DCTRL_FIFORST != 0 {
                    regs.fifo.clear();
                }
                regs.dctrl = value & DCTRL_MASK & !DCTRL_FIFORST;
                if value & DCTRL_DTEN != 0 && regs.dpsm.is_none() {
                    if let Some(card) = card.as_deref() {
                        self.start_data(&mut regs, card);
                    } else {
                        regs.sta |= STA_DTIMEOUT;
                    }
                }
            }
            R_ICR => regs.sta &= !(value & ICR_MASK),
            R_MASKR => regs.mask = value & MASK_MASK,
            R_ACKTIMER => regs.acktimer = value,
            R_IDMACTRLR => {
                regs.idmactrl = (regs.idmactrl & IDMA_BACT) | (value & IDMA_WRITABLE);
            }
            R_IDMABSIZER => regs.idmabsize = value & (IDMABSIZE_MASK << IDMABSIZE_SHIFT),
            R_IDMABASE0R => regs.idmabase[0] = value & !0x3,
            R_IDMABASE1R => regs.idmabase[1] = value & !0x3,
            // RESPCMDR, RESPnR, DCNTR and STAR are read-only; reserved words
            // swallow the write. Neither is an error on this bus.
            _ => {}
        }
    }

    fn write_fifo(&self, value: u32) {
        let card = self.card();
        let mut regs = self.regs.lock();
        if regs.dpsm.is_none_or(|d| d.to_host) {
            // Nothing is expecting data. On real silicon the word goes into a
            // FIFO the DPSM will never read, which is indistinguishable from
            // dropping it.
            return;
        }
        if regs.fifo.len() < FIFO_WORDS {
            regs.fifo.push_back(value);
        }
        if let Some(card) = card.as_deref() {
            self.pump(&mut regs, card);
        }
    }
}

// ---------------------------------------------------------------------------
// The bus port
// ---------------------------------------------------------------------------

/// What an address space dispatches to.
struct Port {
    shared: Arc<Shared>,
}

impl fmt::Debug for Port {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Port").finish_non_exhaustive()
    }
}

impl MemOps for Port {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        let value = self.shared.read_register(offset, attrs.debug);
        dst.copy_from_slice(&value.to_le_bytes());
        if !attrs.debug {
            self.shared.refresh_irq();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write would send a command, move a block or clear a
            // status bit; none of those can be made harmless.
            return Err(BusError::BadAccess);
        }
        self.shared
            .write_register(offset, u32::from_le_bytes([src[0], src[1], src[2], src[3]]));
        self.shared.refresh_irq();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for Sdmmc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the register block and a
        // `wire` statement connects the interrupt.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut regs = self.shared.regs.lock();
            *regs = Regs::reset();
        }
        // The card is its own device and resets itself; this controller must
        // not reach across and do it a second time, because a board reset is
        // delivered to every device exactly once.
        self.shared.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let regs = self.shared.regs.lock();
        w.write_u32(regs.power)?;
        w.write_u32(regs.clkcr)?;
        w.write_u32(regs.arg)?;
        w.write_u32(regs.cmd)?;
        w.write_u32(regs.respcmd)?;
        for word in regs.resp {
            w.write_u32(word)?;
        }
        w.write_u32(regs.dtimer)?;
        w.write_u32(regs.dlen)?;
        w.write_u32(regs.dctrl)?;
        w.write_u32(regs.acktimer)?;
        w.write_u32(regs.idmactrl)?;
        w.write_u32(regs.idmabsize)?;
        w.write_u32(regs.idmabase[0])?;
        w.write_u32(regs.idmabase[1])?;
        w.write_u32(regs.sta & STA_LATCHED)?;
        w.write_u32(regs.mask)?;
        // A partly-filled FIFO is state: the words in it have already left the
        // card and nothing else holds them.
        w.write_seq_len(regs.fifo.len() as u64)?;
        for word in &regs.fifo {
            w.write_u32(*word)?;
        }
        match regs.dpsm {
            None => w.write_bool(false)?,
            Some(d) => {
                w.write_bool(true)?;
                w.write_bool(d.to_host)?;
                w.write_bool(d.started)?;
                w.write_u32(d.left)?;
                w.write_u32(d.block)?;
                w.write_u32(d.block_left)?;
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut regs = Regs::reset();
        regs.power = r.read_u32()?;
        regs.clkcr = r.read_u32()?;
        regs.arg = r.read_u32()?;
        regs.cmd = r.read_u32()?;
        regs.respcmd = r.read_u32()?;
        for slot in &mut regs.resp {
            *slot = r.read_u32()?;
        }
        regs.dtimer = r.read_u32()?;
        regs.dlen = r.read_u32()?;
        regs.dctrl = r.read_u32()?;
        regs.acktimer = r.read_u32()?;
        regs.idmactrl = r.read_u32()?;
        regs.idmabsize = r.read_u32()?;
        regs.idmabase[0] = r.read_u32()?;
        regs.idmabase[1] = r.read_u32()?;
        regs.sta = r.read_u32()? & STA_LATCHED;
        regs.mask = r.read_u32()?;
        let words = r.read_seq_len(4)?;
        if words > FIFO_WORDS as u64 {
            return Err(Error::State(alloc::format!(
                "the snapshot holds {words} FIFO word(s) and this controller holds {FIFO_WORDS}"
            )));
        }
        for _ in 0..words {
            regs.fifo.push_back(r.read_u32()?);
        }
        regs.dpsm = if r.read_bool()? {
            let to_host = r.read_bool()?;
            let started = r.read_bool()?;
            let left = r.read_u32()?;
            let block = r.read_u32()?;
            let block_left = r.read_u32()?;
            if block == 0 || !block.is_power_of_two() || block_left > block || left == 0 {
                return Err(Error::State(alloc::format!(
                    "a snapshot transfer of {left} byte(s) in {block}-byte blocks is not one this \
                     controller can hold"
                )));
            }
            Some(Dpsm {
                to_host,
                left,
                block,
                block_left,
                started,
            })
        } else {
            None
        };
        *self.shared.regs.lock() = regs;
        self.shared.refresh_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != pin::IRQ {
            return Err(Error::Config {
                at: String::from(port),
                message: alloc::format!(
                    "an SDMMC controller drives `{}` and nothing else",
                    pin::IRQ
                ),
            });
        }
        self.attach_irq(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.refresh_irq();
    }
}

impl Instance for Sdmmc {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        // Optional on purpose: a board that only ever uses the FIFO path needs
        // no address space, and requiring one would make every such machine
        // file carry a statement that means nothing. A machine that enables
        // IDMAEN without one is told so through STA.IDMATE, which is the flag
        // the part already has for it.
        if let Some(space) = ctx.space() {
            *self.shared.bus.lock() = Some(Arc::clone(space));
            *self.shared.requester.lock() = ctx.requester();
        }
        Ok(())
    }
}

/// The `stm32.sdmmc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the STM32H7 SDMMC host controller: the register block, the FIFO and the internal DMA",
    properties: &[PropertySpec {
        name: "slot",
        kind: ValueKind::Str,
        required: false,
        summary: "the named card slot this controller drives (default `sd0`)",
    }],
    construct: |props| Ok(Box::new(Sdmmc::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Sdmmc::new(props)?)))
}

/// What the validator should know about `stm32.sdmmc`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("slot", ValueKind::Str))
        .port(pin::IRQ, PortDir::Out)
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests;
