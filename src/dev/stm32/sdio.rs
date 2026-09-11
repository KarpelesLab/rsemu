//! The STM32 **SDIO** host controller — the F2/F4/F7 block, and the L1/L4's
//! `SDMMC` of the same IP.
//!
//! # Which family, and why a second file
//!
//! **ST's RM0090 (`STM32F405/415`, `F407/417`, `F427/437`, `F429/439`), §31.**
//! [`crate::dev::stm32::sdmmc`] is the *other* one — the H7's SDMMC of RM0433
//! §55 — and its header spends a page on why the two cannot be one model with a
//! `variant` property. Everything it says there is still true, so this file
//! only records what is on the other side of each of those sentences:
//!
//! * **No internal DMA.** This controller is not a bus master and takes no
//!   `space`. Data leaves through the FIFO, either a word at a time under the
//!   CPU or by an **external** DMA2 stream that this block asks for service
//!   with a request line. That line is the [`pin::DMA`] output below, and it is
//!   the reason this file could not be written when `sdmmc.rs` was: there was
//!   no DMA controller to hang it off.
//! * **`SDIO_CMD` has no `CMDTRANS` and no `CMDSTOP`.** Bits 6 and 7 are
//!   `WAITRESP`, so the command state machine never starts the data state
//!   machine: software sets `DCTRL.DTEN` itself, always. A driver written for
//!   the H7 that relies on `CMDTRANS` sets `WAITRESP = 01b` here by accident.
//! * **`WAITRESP = 10b` means *no response*** (RM0090 §31.9.4), where on the
//!   H7 it means "short response, CRC not checked". This is the incompatibility
//!   that costs the most, because it fails *quietly*: a driver that sends
//!   `ACMD41` with `10b` — the right thing on an H7 — gets `CMDSENT`, an
//!   untouched `RESP1` and an OCR of zero, and concludes the card never powered
//!   up. See [`Shape::of`], which is four lines and is the whole of it.
//! * **There is no "short, no CRC" encoding at all**, which is why `R3` and
//!   `R4` — whose CRC field is all ones because they carry none (Physical Layer
//!   §4.9.4) — are sent with `WAITRESP = 01b` and come back with
//!   **`CCRCFAIL` instead of `CMDREND`**. That is not a failure, and every
//!   STM32 SD driver clears it and carries on; a model that reported `CMDREND`
//!   would be one whose `ACMD41` loop never terminates on real firmware. See
//!   [`Shared::run_command`].
//! * **The FIFO is thirty-two words**, not sixteen, and the half thresholds are
//!   at eight words either way rather than at the midpoint: `TXFIFOHE` means
//!   "at least eight words can be written", `RXFIFOHF` "at least eight words
//!   are in it" (§31.9.11).
//! * **`STA` bits 11 to 21 are a different register.** `CMDACT`, `TXACT` and
//!   `RXACT` sit at 11/12/13 where the H7 has `DABORT`/`DPSMACT`/`CPSMACT`, and
//!   **`TXDAVL` and `RXDAVL` at 20 and 21 exist only here** — the H7 puts
//!   `BUSYD0` and `BUSYD0END` there. `RXDAVL` is what a CPU-driven read loop
//!   polls, so it is the flag a ported driver misses first.
//! * **Every one of `STA`'s twenty-four bits is maskable** (§31.9.13 gives an
//!   `IE` for each, `CMDACTIE` and `RXDAVLIE` included), where the H7 refuses
//!   to let three of them raise an interrupt.
//! * `SDIO_FIFOCNT` (0x48) has no counterpart on the H7 at all.
//!
//! The register block is otherwise the same on the L1 and L4, where the same IP
//! is called `SDMMC1` (ST RM0351 §47). One class covers both; nothing below
//! differs between them.
//!
//! # The DMA request, which is the point of this device
//!
//! [`pin::DMA`] is an **output**, a level, wired in the board file to the input
//! pin of whichever DMA unit the part's request matrix puts this peripheral on:
//!
//! ```text
//!   wire sdio.dma -> dma2.req3     # RM0090 Table 43: DMA2 stream 3, channel 4
//! ```
//!
//! `st.dma` then performs an ordinary bus read or write **at `CPAR`**, which
//! the driver has pointed at this block's `FIFO` aperture, and that access is
//! what moves the word — exactly as on the silicon, where the controller has no
//! data path to the peripheral either. So the whole seam is the line, and the
//! two devices share nothing else. Two things follow that a board must get
//! right, and both are mistakes a machine file can make silently: the SDIO's
//! register block has to be mapped in *the space the DMA controller masters*,
//! not only in the core's, and `CPAR` has to point at the FIFO aperture rather
//! than at `DCTRL`.
//!
//! The level is high while the FIFO can service a beat: a word is in it on a
//! card-to-host transfer, or there is room and words still owed on a
//! host-to-card one, and `DCTRL.DMAEN` is set. It is recomputed and driven
//! after every register access, **including the DMA controller's own** — which
//! is what makes the transfer converge: each beat pops a word, the card streams
//! the next one in behind it, and the line drops on the beat that empties the
//! FIFO for the last time.
//!
//! **Why not the `RXFIFOHF`/`TXFIFOHE` thresholds the manual names.** On the
//! part those exist to amortise a *burst*: ST's own configuration gives the
//! stream `PBURST = INCR4`, so one request buys four words and `DLEN`'s
//! four-word multiple makes the tail come out even. `st.dma` models no
//! bursts — one request buys one beat — so a half-full threshold here would do
//! two things the silicon does not: hold the first beat back for eight words,
//! and strand a tail of fewer than eight that no further request would ever
//! collect. Following `RXDAVL` and FIFO room instead moves the same words, in
//! the same order, at the same points in the stream. When `st.dma` grows
//! bursts, [`Regs::dma_request`] is the one function to revisit.
//!
//! Nothing outward happens while the register lock is held: both levels are
//! computed under it, the lock is dropped, and only then are the wires driven
//! (`CLAUDE.md`, "Concurrency").
//!
//! # The interrupt
//!
//! One output pin, `irq`, driven from `STA & MASK` reduced to a level, with no
//! interrupt number anywhere in this file — the argument is
//! [`crate::dev::stm32::sdmmc`]'s and it is unchanged. On an F407 the board
//! writes `wire sdio.irq -> cpu.irq49`, 49 being SDIO's position in RM0090
//! Table 62.
//!
//! # Time
//!
//! As in the H7 model, **zero**, and for the same reasons: a command completes
//! inside the write to `CMD`, and a FIFO transfer is paced by whoever drains
//! it, so the FIFO's occupancy is always the real depth of a real
//! thirty-two-word FIFO. `TXUNDERR` and `RXOVERR` are consequently
//! **unreachable**, which is a claim rather than an oversight: both are the
//! driver losing a race against the card, and there is no race to lose while
//! the card only moves a word when somebody asks for one. A driver that reads
//! the FIFO arbitrarily slowly loses nothing here and would lose nothing on the
//! part either — the card stops clocking DAT when the FIFO fills. Giving this
//! block a clock domain is what makes the two flags reachable, and
//! `CLKCR.CLKDIV` is already decoded for the day that happens.
//!
//! `DTIMER` is the exception, because one value of it *is* distinguishable
//! without a clock: **zero**. The data timeout counter is loaded from `DTIMER`
//! when the DPSM starts waiting on DAT (§31.9.6), so a driver that arms a
//! transfer having never programmed `DTIMER` gets `DTIMEOUT` immediately and no
//! data — which is the mistake the register exists to catch and the one a model
//! that ignored it would hide. Any non-zero value is "long enough", and a card
//! that goes quiet part way through is what raises `DTIMEOUT` otherwise.
//!
//! # Sources
//!
//! * **ST, RM0090** rev 21, §31 "Secure digital input/output interface (SDIO)":
//!   the register map of §31.9, the command and data state machines of §31.4,
//!   the FIFO of §31.4.4 and the status flags of §31.9.11. Table 43 for which
//!   DMA stream carries the request and Table 62 for the vector position — both
//!   facts about the part, so both live in the board file.
//! * **ST, RM0351** for the L4's `SDMMC1`, §47, which is the same block.
//! * The SD Association's *Physical Layer Simplified Specification* for what
//!   the other end of the link does — but only through [`crate::dev::sd`],
//!   never here. There is not one SD command index in this file.
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
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::dev::sd::card::{Data, Reply, SdCard};
use crate::dev::sd::slots::{self, Slot};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "stm32.sdio";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes of address space the register block occupies.
///
/// The 1 KiB aperture an STM32 peripheral gets in the memory map. Only the
/// first 0x100 bytes decode to anything; the rest reads zero, which is what a
/// reserved word on this part does.
pub const REGISTER_BYTES: u64 = 0x400;

/// How many 32-bit words the data FIFO holds. **Thirty-two** here, sixteen on
/// the H7 (RM0090 §31.4.4).
pub const FIFO_WORDS: usize = 32;

/// The occupancy either half-threshold flag is measured against.
///
/// Eight words, and it is eight rather than half the depth: `TXFIFOHE` is "at
/// least 8 words can be written into the FIFO" and `RXFIFOHF` is "at least 8
/// words are in the FIFO" (§31.9.11). On a thirty-two-word FIFO that is a
/// quarter and a quarter, not the midpoint the names suggest.
const HALF_WORDS: usize = 8;

/// Where this controller's register lock sits in the ranked order.
///
/// The same rung [`crate::dev::stm32::sdmmc`] uses, and for the same reason:
/// the lock **is** held across the call into the card, because moving a block
/// is one step and dropping the lock half way would let a second access see a
/// half-updated `DCOUNT`. The ladder is written out on
/// [`crate::dev::sd::slots::SLOT_RANK`].
pub const REGISTER_RANK: LockRank = LockRank::new(0x4d00);

/// The rank of this controller's two output cells.
///
/// Above the register lock, and taken only after it has been dropped: both
/// wires are driven from outside every critical section.
const CELL_RANK: LockRank = LockRank::new(0x4f00);

/// The pin names a machine description wires.
pub mod pin {
    /// The interrupt request the NVIC sees: `STA & MASK` reduced to a level.
    pub const IRQ: &str = "irq";
    /// The DMA request: high while the FIFO wants service and `DMAEN` is set.
    ///
    /// Wired to one of a DMA controller's `req` inputs. See the module note.
    pub const DMA: &str = "dma";
}

// ---------------------------------------------------------------------------
// The register map (RM0090 §31.9)
// ---------------------------------------------------------------------------

const R_POWER: u64 = 0x00;
const R_CLKCR: u64 = 0x04;
const R_ARG: u64 = 0x08;
const R_CMD: u64 = 0x0c;
const R_RESPCMD: u64 = 0x10;
const R_RESP1: u64 = 0x14;
const R_RESP4: u64 = 0x20;
const R_DTIMER: u64 = 0x24;
const R_DLEN: u64 = 0x28;
const R_DCTRL: u64 = 0x2c;
const R_DCOUNT: u64 = 0x30;
const R_STA: u64 = 0x34;
const R_ICR: u64 = 0x38;
const R_MASK: u64 = 0x3c;
const R_FIFOCNT: u64 = 0x48;
const R_FIFO: u64 = 0x80;
const R_FIFO_END: u64 = R_FIFO + (FIFO_WORDS as u64) * 4;

// -- POWER (§31.9.1) --------------------------------------------------------

/// `POWER` bits 1:0, which is everything the register has.
const POWER_PWRCTRL: u32 = 0x3;
/// The `PWRCTRL` encoding for "power on". `01b` is reserved and `10b` is the
/// power-up state in which the clock is not yet running.
const POWER_ON: u32 = 0x3;

// -- CLKCR (§31.9.2) --------------------------------------------------------

/// `CLKCR` bits 7:0, the clock divider. Eight bits here; ten on the H7.
const CLKCR_CLKDIV: u32 = 0xff;
/// `CLKCR` bits 12:11, the data bus width.
const CLKCR_WIDBUS_SHIFT: u32 = 11;
const CLKCR_WIDBUS_MASK: u32 = 0x3;
/// Everything `CLKCR` defines, up to `HWFC_EN` at 14.
const CLKCR_MASK: u32 = 0x7fff;

// -- CMD (§31.9.4) ----------------------------------------------------------

/// `CMD` bits 5:0, the command index.
const CMD_INDEX: u32 = 0x3f;
/// `CMD` bits 7:6, how long a response to wait for.
///
/// **Where the H7 puts `CMDTRANS` and `CMDSTOP`.** The named incompatibility.
const CMD_WAITRESP_SHIFT: u32 = 6;
const CMD_WAITRESP_MASK: u32 = 0x3;
/// `CMD` bit 10: enable the command state machine.
const CMD_CPSMEN: u32 = 1 << 10;
/// Everything `CMD` defines, up to `CE-ATACMD` at 14.
const CMD_MASK: u32 = 0x7fff;

/// `WAITRESP = 00b`: no response, and `CMDSENT` when the command is out.
pub const WAITRESP_NONE: u32 = 0b00;
/// `WAITRESP = 01b`: a 48-bit response.
///
/// The *only* short encoding this block has, which is why `R3` and `R4` are
/// sent with it and answered with `CCRCFAIL` — see the module note.
pub const WAITRESP_SHORT: u32 = 0b01;
/// `WAITRESP = 10b`: **no response**, identically to `00b` (§31.9.4).
///
/// On the H7 this encoding means "short response, CRC not checked". A driver
/// carrying that assumption across families is told nothing and reads zero out
/// of `RESP1`.
pub const WAITRESP_NONE_ALT: u32 = 0b10;
/// `WAITRESP = 11b`: a 136-bit response.
pub const WAITRESP_LONG: u32 = 0b11;

/// How many bits the command state machine waits for.
///
/// Four encodings, three shapes, and **two of the four mean nothing at all** —
/// which is the difference from the H7 in the only form that matters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Shape {
    /// Nothing. The CPSM reports `CMDSENT` and hears whatever the card said as
    /// silence.
    None,
    /// 48 bits.
    Short,
    /// 136 bits.
    Long,
}

impl Shape {
    fn of(waitresp: u32) -> Shape {
        match waitresp & CMD_WAITRESP_MASK {
            // `10b` lands here, not on `Short`. RM0090 §31.9.4's WAITRESP
            // table: "00: No response, expect CMDSENT flag / 01: Short
            // response, expect CMDREND or CCRCFAIL flag / 10: No response,
            // expect CMDSENT flag / 11: Long response, expect CMDREND or
            // CCRCFAIL flag".
            WAITRESP_NONE | WAITRESP_NONE_ALT => Shape::None,
            WAITRESP_SHORT => Shape::Short,
            _ => Shape::Long,
        }
    }
}

// -- DCTRL (§31.9.9) --------------------------------------------------------

/// `DCTRL` bit 0: enable the data state machine.
const DCTRL_DTEN: u32 = 1 << 0;
/// `DCTRL` bit 1: `0` is controller to card, `1` is card to controller.
const DCTRL_DTDIR: u32 = 1 << 1;
/// `DCTRL` bit 2: `0` is block, `1` is stream or multibyte.
const DCTRL_DTMODE: u32 = 1 << 2;
/// `DCTRL` bit 3: the DMA request output is enabled.
const DCTRL_DMAEN: u32 = 1 << 3;
/// `DCTRL` bits 7:4, the base-two logarithm of the block size.
const DCTRL_DBLOCKSIZE_SHIFT: u32 = 4;
const DCTRL_DBLOCKSIZE_MASK: u32 = 0xf;
/// Everything `DCTRL` defines, up to `SDIOEN` at 11.
const DCTRL_MASK: u32 = 0x0fff;

/// `DLEN` bits 24:0.
const DLEN_MASK: u32 = 0x01ff_ffff;

// -- STA / ICR / MASK (§31.9.11 to §31.9.13) --------------------------------

/// Command response CRC failed. Also what a CRC-less `R3` looks like.
const STA_CCRCFAIL: u32 = 1 << 0;
/// Data block CRC failed.
const STA_DCRCFAIL: u32 = 1 << 1;
/// Command response timeout: 64 `SDIO_CK` with nothing on CMD.
const STA_CTIMEOUT: u32 = 1 << 2;
/// Data timeout: `DTIMER` elapsed with nothing on DAT.
const STA_DTIMEOUT: u32 = 1 << 3;
/// Transmit FIFO underrun.
const STA_TXUNDERR: u32 = 1 << 4;
/// Receive FIFO overrun.
const STA_RXOVERR: u32 = 1 << 5;
/// Command response received, CRC passed.
const STA_CMDREND: u32 = 1 << 6;
/// Command sent, no response required.
const STA_CMDSENT: u32 = 1 << 7;
/// Data end: `DCOUNT` reached zero.
const STA_DATAEND: u32 = 1 << 8;
/// A start bit was not seen on every data line in wide-bus mode.
const STA_STBITERR: u32 = 1 << 9;
/// Data block sent or received, CRC passed.
const STA_DBCKEND: u32 = 1 << 10;
/// The command state machine is running. Read-only, not clearable.
///
/// **Bit 11**, where the H7 has `DABORT`.
const STA_CMDACT: u32 = 1 << 11;
/// The data state machine is transmitting. Read-only.
const STA_TXACT: u32 = 1 << 12;
/// The data state machine is receiving. Read-only.
const STA_RXACT: u32 = 1 << 13;
/// At least [`HALF_WORDS`] words can be written into the FIFO.
const STA_TXFIFOHE: u32 = 1 << 14;
/// At least [`HALF_WORDS`] words are in the FIFO.
const STA_RXFIFOHF: u32 = 1 << 15;
/// The transmit FIFO is full.
const STA_TXFIFOF: u32 = 1 << 16;
/// The receive FIFO is full.
const STA_RXFIFOF: u32 = 1 << 17;
/// The transmit FIFO is empty.
const STA_TXFIFOE: u32 = 1 << 18;
/// The receive FIFO is empty.
const STA_RXFIFOE: u32 = 1 << 19;
/// Data available in the transmit FIFO. **No counterpart on the H7.**
const STA_TXDAVL: u32 = 1 << 20;
/// Data available in the receive FIFO.
///
/// **No counterpart on the H7**, and the flag a CPU-driven read loop polls.
const STA_RXDAVL: u32 = 1 << 21;
/// An SD I/O interrupt arrived on DAT1.
const STA_SDIOIT: u32 = 1 << 22;
/// A CE-ATA command completion signal was received.
const STA_CEATAEND: u32 = 1 << 23;

/// The bits `ICR` clears (§31.9.12). Everything else in `STA` is computed.
const ICR_MASK: u32 = STA_CCRCFAIL
    | STA_DCRCFAIL
    | STA_CTIMEOUT
    | STA_DTIMEOUT
    | STA_TXUNDERR
    | STA_RXOVERR
    | STA_CMDREND
    | STA_CMDSENT
    | STA_DATAEND
    | STA_STBITERR
    | STA_DBCKEND
    | STA_SDIOIT
    | STA_CEATAEND;

/// The bits that live in the latch rather than being derived.
const STA_LATCHED: u32 = ICR_MASK;

/// The bits computed on every read from the FIFO's occupancy and the two state
/// machines, rather than latched.
const STA_DERIVED: u32 = STA_CMDACT
    | STA_TXACT
    | STA_RXACT
    | STA_TXFIFOHE
    | STA_RXFIFOHF
    | STA_TXFIFOF
    | STA_RXFIFOF
    | STA_TXFIFOE
    | STA_RXFIFOE
    | STA_TXDAVL
    | STA_RXDAVL;

// A bit is one or the other and never both, and between them they are the whole
// of the twenty-four `STA` defines. A typo in a shift is the one way this goes
// wrong silently, so it is checked at compile time.
const _: () = assert!(STA_LATCHED & STA_DERIVED == 0);
const _: () = assert!(STA_LATCHED | STA_DERIVED == 0x00ff_ffff);

/// Everything `MASK` can enable.
///
/// **All twenty-four**, which is a real difference from the H7 rather than a
/// simplification: §31.9.13 gives `SDIO_MASK` an interrupt-enable bit for every
/// `STA` bit, `CMDACTIE`, `TXACTIE`, `RXACTIE`, `TXFIFOFIE` and `RXDAVLIE`
/// included. The H7 refuses three of them.
const MASK_MASK: u32 = STA_LATCHED | STA_DERIVED;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

/// The data state machine, while it is running.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Dpsm {
    /// Card to controller.
    to_host: bool,
    /// Stream or multibyte rather than block, from `DCTRL.DTMODE`. There are no
    /// block boundaries in that mode, so there is no `DBCKEND`.
    stream: bool,
    /// Bytes the card still owes the FIFO, or still expects from it. This is
    /// `DCOUNT`.
    left: u32,
    /// The block size the transfer is counted in.
    block: u32,
    /// Bytes left in the block being moved, for `DBCKEND`.
    block_left: u32,
    /// Whether any byte has moved yet.
    ///
    /// The DPSM is armed before the command is sent on this block — always,
    /// because there is no `CMDTRANS` — and on real silicon it simply waits on
    /// DAT. So a card with nothing to give is "not yet", while a card that
    /// stops mid-transfer is a data timeout, and this is what tells them apart.
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
    /// The latched half of `STA`. The rest is computed.
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
            sta: 0,
            mask: 0,
            fifo: VecDeque::with_capacity(FIFO_WORDS),
            dpsm: None,
        }
    }

    fn powered(&self) -> bool {
        self.power & POWER_PWRCTRL == POWER_ON
    }

    /// `STA` as a guest reads it: the latch, plus the eleven derived bits.
    ///
    /// The FIFO is **one** FIFO on the die, so the transmit and receive views of
    /// its occupancy come off the same counter and both are meaningful
    /// whichever way the transfer is going. A model that gated them on
    /// direction would report `RXFIFOE` as clear during a write, which no part
    /// does.
    fn status(&self) -> u32 {
        let mut sta = self.sta & STA_LATCHED;
        let level = self.fifo.len();
        if level == 0 {
            sta |= STA_TXFIFOE | STA_RXFIFOE;
        } else {
            sta |= STA_TXDAVL | STA_RXDAVL;
        }
        if level >= FIFO_WORDS {
            sta |= STA_TXFIFOF | STA_RXFIFOF;
        }
        if level >= HALF_WORDS {
            sta |= STA_RXFIFOHF;
        }
        if FIFO_WORDS - level >= HALF_WORDS {
            sta |= STA_TXFIFOHE;
        }
        if let Some(dpsm) = self.dpsm {
            sta |= if dpsm.to_host { STA_RXACT } else { STA_TXACT };
        }
        // CMDACT is never set: a command completes inside the write to CMD.
        sta
    }

    /// `DCOUNT`: bytes the card has not yet handed over, or taken.
    fn dcount(&self) -> u32 {
        self.dpsm.map_or(0, |d| d.left)
    }

    /// `FIFOCNT`: words that have still to pass through the FIFO *port*.
    ///
    /// §31.9.14 — "remaining number of words to be written to or read from the
    /// FIFO", loaded from `DLEN` when `DTEN` is set. It is not `DCOUNT / 4`: on
    /// a read the words already streamed into the FIFO still have to be read
    /// out of it, and on a write the words already in it have already been
    /// written into it. A data length that is not a multiple of four counts its
    /// last one to three bytes as a whole word, which the manual says in as
    /// many words and [`u32::div_ceil`] does here.
    fn fifocnt(&self) -> u32 {
        let Some(dpsm) = self.dpsm else { return 0 };
        let words = dpsm.left.div_ceil(4);
        let held = self.fifo.len() as u32;
        if dpsm.to_host {
            words + held
        } else {
            words.saturating_sub(held)
        }
    }

    /// Whether the DMA request line wants service.
    ///
    /// See the module note on why this follows `RXDAVL` and FIFO room rather
    /// than the `RXFIFOHF`/`TXFIFOHE` thresholds the manual names.
    ///
    /// The direction comes from `DCTRL.DTDIR` and not from the transfer,
    /// because the receiving case **outlives** it: `DCOUNT` reaching zero means
    /// the *card* has finished, and at that moment the last words of the block
    /// are still in the FIFO with nobody but the DMA to collect them. A request
    /// that went down with the DPSM would strand them — thirty-two words of a
    /// 128-word block, which is exactly the shape of a driver bug report that
    /// says "the last part of every read is garbage".
    fn dma_request(&self) -> bool {
        if self.dctrl & DCTRL_DMAEN == 0 {
            return false;
        }
        if self.dctrl & DCTRL_DTDIR != 0 {
            !self.fifo.is_empty()
        } else {
            self.dpsm.is_some() && self.fifo.len() < FIFO_WORDS && self.fifocnt() > 0
        }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// The STM32 SDIO host controller.
pub struct Sdio {
    shared: Arc<Shared>,
    region: RegionRef,
}

impl fmt::Debug for Sdio {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Sdio")
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
    /// The DMA request output, once a `wire` statement connects it.
    ///
    /// Legitimately `None` forever on a board that only ever uses the CPU path,
    /// which is why nothing here refuses `DMAEN` when nobody is listening: on
    /// the part the pin is bonded to a DMA controller that may simply not be
    /// enabled, and the symptom is the one the silicon gives — a transfer that
    /// never advances.
    drq: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared")
            .field("slot", &self.slot_name)
            .finish_non_exhaustive()
    }
}

impl Sdio {
    /// Validate `props` and allocate the controller.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is missing or of the wrong kind.
    pub fn new(props: &Props) -> Result<Sdio> {
        let mut r = props.reader();
        let slot_name = r.or_str("slot", crate::dev::sd::DEFAULT_SLOT)?.to_string();
        r.finish()?;
        // Acquiring the socket is allocation, not an outward action: the card
        // may not have been constructed yet, and whichever end runs first
        // creates the rendezvous (`core::hosts`).
        let slot = slots::attach(props, &slot_name)?;
        Ok(Sdio::with_slot(slot, slot_name))
    }

    /// Build a controller around a socket that already exists.
    #[must_use]
    pub fn with_slot(slot: Arc<Slot>, slot_name: String) -> Sdio {
        let shared = Arc::new(Shared {
            regs: Mutex::with_rank(REGISTER_RANK, Regs::reset()),
            slot,
            slot_name,
            irq: Mutex::with_rank(CELL_RANK, None),
            drq: Mutex::with_rank(CELL_RANK, None),
        });
        let port = Arc::new(Port {
            shared: Arc::clone(&shared),
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_BYTES,
            port as Arc<dyn MemOps>,
        ));
        Sdio { shared, region }
    }

    /// The socket this controller is wired to.
    #[must_use]
    pub fn slot(&self) -> &Arc<Slot> {
        &self.shared.slot
    }

    /// Connect the interrupt request line.
    pub fn attach_irq(&self, source: WireSource) {
        *self.shared.irq.lock() = Some(source);
        self.shared.refresh_outputs();
    }

    /// Connect the DMA request line.
    pub fn attach_drq(&self, source: WireSource) {
        *self.shared.drq.lock() = Some(source);
        self.shared.refresh_outputs();
    }

    /// `STA` as a guest would read it, without disturbing anything.
    #[must_use]
    pub fn status(&self) -> u32 {
        self.shared.regs.lock().status()
    }

    /// `CLKCR.CLKDIV`: the divider from `SDIOCLK` to `SDIO_CK`.
    ///
    /// Decoded and stored but not otherwise used, because this device has no
    /// clock domain — see the module note on time. It is read back, so a driver
    /// that programs it and checks sees what it wrote.
    #[must_use]
    pub fn clock_divider(&self) -> u32 {
        self.shared.regs.lock().clkcr & CLKCR_CLKDIV
    }

    /// How many data lines `CLKCR.WIDBUS` selects: 1, 4 or 8.
    ///
    /// The card learns its own width from `ACMD6` and not from here, so this is
    /// what the *controller* thinks — which is the half a driver can get wrong
    /// on its own.
    #[must_use]
    pub fn bus_width(&self) -> u32 {
        match (self.shared.regs.lock().clkcr >> CLKCR_WIDBUS_SHIFT) & CLKCR_WIDBUS_MASK {
            0b01 => 4,
            0b10 => 8,
            // `11b` is reserved; the block behaves as if one line were
            // selected, which is also the reset value's behaviour.
            _ => 1,
        }
    }

    /// Whether the DMA request line is currently asking for service.
    #[must_use]
    pub fn dma_requesting(&self) -> bool {
        self.shared.regs.lock().dma_request()
    }
}

impl Shared {
    fn card(&self) -> Option<Arc<SdCard>> {
        self.slot.card()
    }

    /// Drive `irq` from `STA & MASK`, and `dma` from the FIFO.
    ///
    /// Called with **no** lock held: driving a wire reaches into whatever is
    /// listening, and the re-entrancy contract says to release first
    /// (`CLAUDE.md`, "Concurrency"). Both levels are computed in one pass so the
    /// two outputs can never disagree about the same register image.
    fn refresh_outputs(&self) {
        let (irq, drq) = {
            let regs = self.regs.lock();
            (
                regs.status() & regs.mask & MASK_MASK != 0,
                regs.dma_request(),
            )
        };
        if let Some(wire) = self.irq.lock().as_ref() {
            wire.set(Level::from(irq));
        }
        if let Some(wire) = self.drq.lock().as_ref() {
            wire.set(Level::from(drq));
        }
    }

    // -- the command state machine -----------------------------------------

    /// Run one command, as a write to `CMD` with `CPSMEN` set does.
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
            // real hardware either. §31.4.2: the CPSM gives up after 64
            // `SDIO_CK` and sets `CTIMEOUT`.
            regs.sta |= STA_CTIMEOUT;
            return;
        }
        let Some(card) = card else {
            regs.sta |= STA_CTIMEOUT;
            return;
        };

        let reply = card.command(index, arg);
        match (Shape::of(waitresp), reply) {
            (Shape::None, _) => {
                // The CPSM did not wait for anything, so whatever the card said
                // went unheard. This is the arm a driver ported from an H7
                // falls into with `ACMD41`.
                regs.sta |= STA_CMDSENT;
            }
            (_, Reply::None) => regs.sta |= STA_CTIMEOUT,
            (Shape::Long, Reply::Long(words)) => {
                // R2 has no command index; the corresponding field is all ones
                // and that is what RESPCMD latches. The CRC7 the card puts in
                // the last byte is a real one, so the check passes.
                regs.respcmd = CMD_INDEX;
                regs.resp = words;
                regs.sta |= STA_CMDREND;
            }
            (Shape::Long, Reply::Short { .. }) => {
                // 48 bits arrived where 136 were expected: the CPSM keeps
                // waiting and eventually gives up.
                regs.sta |= STA_CTIMEOUT;
            }
            (Shape::Short, Reply::Short { index, value, .. }) => {
                regs.respcmd = u32::from(index) & CMD_INDEX;
                regs.resp[0] = value;
                // **`R3` reports `CCRCFAIL`, not `CMDREND`.** The card drives
                // all ones where the CRC7 belongs, because `R3` carries none
                // (Physical Layer §4.9.4), and this block has no "short, no
                // CRC" `WAITRESP` encoding to be told so with — so the CPSM
                // checks a CRC that was never sent and reports a mismatch, with
                // the response still latched. Every STM32 SD driver clears the
                // flag and reads `RESP1` anyway.
                //
                // The response is recognised the way the silicon's CPSM
                // recognises it: the echoed index field is all ones too, which
                // is what `R2` and `R3` have in common and what no ordinary
                // response has.
                if u32::from(index) & CMD_INDEX == CMD_INDEX {
                    regs.sta |= STA_CCRCFAIL;
                } else {
                    regs.sta |= STA_CMDREND;
                }
            }
            (Shape::Short, Reply::Long(_)) => {
                // 136 bits arrived where 48 were expected, so the bits the CPSM
                // sampled as a CRC are not one.
                regs.sta |= STA_CCRCFAIL;
            }
        }

        // No `CMDTRANS` on this block: if a transfer is armed it has been
        // waiting on DAT since the write to `DCTRL`, and the command it was
        // waiting for has now gone out.
        self.pump(regs, card);
    }

    // -- the data state machine --------------------------------------------

    /// Start a transfer, as a write to `DCTRL` with `DTEN` set does.
    fn start_data(&self, regs: &mut Regs, card: &SdCard) {
        let len = regs.dlen & DLEN_MASK;
        let to_host = regs.dctrl & DCTRL_DTDIR != 0;
        let stream = regs.dctrl & DCTRL_DTMODE != 0;
        let shift = (regs.dctrl >> DCTRL_DBLOCKSIZE_SHIFT) & DCTRL_DBLOCKSIZE_MASK;
        // DBLOCKSIZE is the base-two logarithm, capped at 14 (16 KiB).
        let block = 1u32 << shift.min(14);
        if len == 0 {
            Self::finish(regs);
            return;
        }
        if regs.dtimer == 0 {
            // §31.9.6: the data timeout counter is loaded from `DTIMER` when
            // the DPSM starts waiting, so zero has already expired before the
            // first byte. A driver that never programmed the register finds out
            // here rather than reading a block it was not entitled to.
            Self::finish(regs);
            regs.sta |= STA_DTIMEOUT;
            return;
        }
        regs.fifo.clear();
        regs.dpsm = Some(Dpsm {
            to_host,
            stream,
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
    /// when there is nothing to do. That is what lets the `DTEN`-first sequence
    /// this block requires work without the command path knowing about it.
    fn pump(&self, regs: &mut Regs, card: &SdCard) {
        let Some(dpsm) = regs.dpsm else { return };
        if dpsm.to_host {
            Self::fill_fifo(regs, card);
        } else {
            Self::drain_fifo(regs, card);
        }
    }

    /// Pull bytes from the card into the FIFO until one or the other is done.
    fn fill_fifo(regs: &mut Regs, card: &SdCard) {
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
    ///
    /// The word is **taken out only once the card has accepted it**, which
    /// matters here in a way it does not on the H7: there is no `CMDTRANS`, so
    /// software may fill the FIFO before the write command has gone out, and on
    /// this block it routinely does — a DMA stream starts pulling from memory
    /// the moment `DMAEN` and `DTEN` are set. A pop-then-offer loop would drop
    /// every one of those words on the floor.
    fn drain_fifo(regs: &mut Regs, card: &SdCard) {
        while let Some(&word) = regs.fifo.front() {
            let Some(dpsm) = regs.dpsm else { return };
            if dpsm.left == 0 {
                return;
            }
            let run = dpsm.left.min(4) as usize;
            if card.write_data(&word.to_le_bytes()[..run]) == Data::Ended {
                Self::stalled(regs);
                return;
            }
            regs.fifo.pop_front();
            Self::advance(regs, run as u32);
        }
    }

    /// End the data path, clearing `DCTRL.DTEN` as the hardware does.
    ///
    /// The DPSM leaves the Idle state on `DTEN` and returns to it when the
    /// transfer ends (§31.4.3). Clearing the bit with it is what lets software
    /// arm the *next* transfer: a driver writes `DTEN` again, and a bit that
    /// never went low makes no edge.
    fn finish(regs: &mut Regs) {
        regs.dpsm = None;
        regs.dctrl &= !DCTRL_DTEN;
    }

    /// The card had nothing to say.
    ///
    /// Before the first byte that is the ordinary state of a data path waiting
    /// on DAT for a command that has not been sent yet — which on this block is
    /// *every* transfer, because `DTEN` always precedes the command — and the
    /// DPSM stays armed. After it, the card has stopped mid-transfer, which is
    /// silence past `DTIMER`.
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
            // Stream and multibyte transfers have no block boundaries, so there
            // is nothing for `DBCKEND` to mark (§31.4.3).
            if !dpsm.stream {
                regs.sta |= STA_DBCKEND;
            }
            let block = dpsm.block;
            let left = dpsm.left;
            dpsm.block_left = block.min(left);
        }
        if dpsm.left == 0 {
            regs.sta |= STA_DATAEND;
            Self::finish(regs);
        }
    }

    // -- the register block ------------------------------------------------

    fn read_register(&self, offset: u64, debug: bool) -> u32 {
        if (R_FIFO..R_FIFO_END).contains(&offset) {
            return self.read_fifo(debug);
        }
        let regs = self.regs.lock();
        match offset {
            R_POWER => regs.power,
            R_CLKCR => regs.clkcr,
            R_ARG => regs.arg,
            R_CMD => regs.cmd,
            R_RESPCMD => regs.respcmd,
            R_RESP1..=R_RESP4 => regs.resp[((offset - R_RESP1) / 4) as usize],
            R_DTIMER => regs.dtimer,
            R_DLEN => regs.dlen,
            R_DCTRL => regs.dctrl,
            R_DCOUNT => regs.dcount(),
            R_STA => regs.status(),
            // ICR reads back the bits it would clear, which is what the
            // reference manual's reset value and access column say.
            R_ICR => regs.sta & ICR_MASK,
            R_MASK => regs.mask,
            R_FIFOCNT => regs.fifocnt(),
            // Reserved. Zero, which is what this part answers.
            _ => 0,
        }
    }

    /// Pop the FIFO, or peek at it for a debugger.
    ///
    /// A debugger reading `FIFO` must not consume a word, and must not let the
    /// card stream another one in behind it (`ROADMAP.md` §15, invariant 5).
    fn read_fifo(&self, debug: bool) -> u32 {
        if debug {
            return self.regs.lock().fifo.front().copied().unwrap_or(0);
        }
        let card = self.card();
        let mut regs = self.regs.lock();
        let word = regs.fifo.pop_front().unwrap_or(0);
        if let Some(card) = card.as_deref() {
            // Draining made room, so the card streams the next words in. This
            // is what keeps the modelled FIFO exactly thirty-two words deep.
            self.pump(&mut regs, card);
        }
        word
    }

    fn write_register(&self, offset: u64, value: u32) {
        if (R_FIFO..R_FIFO_END).contains(&offset) {
            self.write_fifo(value);
            return;
        }
        let card = self.card();
        let mut regs = self.regs.lock();
        match offset {
            R_POWER => {
                let was_on = regs.powered();
                regs.power = value & POWER_PWRCTRL;
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
            R_ARG => regs.arg = value,
            R_CMD => {
                regs.cmd = value & CMD_MASK;
                if value & CMD_CPSMEN != 0 {
                    self.run_command(&mut regs, card.as_deref());
                }
            }
            R_DTIMER => regs.dtimer = value,
            R_DLEN => regs.dlen = value & DLEN_MASK,
            R_DCTRL => {
                regs.dctrl = value & DCTRL_MASK;
                if value & DCTRL_DTEN != 0 && regs.dpsm.is_none() {
                    if let Some(card) = card.as_deref() {
                        self.start_data(&mut regs, card);
                    } else {
                        Self::finish(&mut regs);
                        regs.sta |= STA_DTIMEOUT;
                    }
                }
            }
            R_ICR => regs.sta &= !(value & ICR_MASK),
            R_MASK => regs.mask = value & MASK_MASK,
            // RESPCMD, RESPn, DCOUNT, STA and FIFOCNT are read-only; reserved
            // words swallow the write. Neither is an error on this bus.
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
            // This is also the path a DMA stream's beat arrives on, so the
            // request line is recomputed here and the transfer converges.
            self.shared.refresh_outputs();
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
        self.shared.refresh_outputs();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for Sdio {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the register block and two
        // `wire` statements connect the outputs. This controller is not a bus
        // master, so unlike `stm32.sdmmc` it takes no address space — which is
        // why there is no `Instance::bind` body below.
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
        self.shared.refresh_outputs();
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
        w.write_u32(regs.sta & STA_LATCHED)?;
        w.write_u32(regs.mask)?;
        // A partly-filled FIFO is state: the words in it have already left the
        // card and nothing else holds them. Up to thirty-two of them, and a
        // transfer caught mid-block keeps whichever the guest had not read yet.
        w.write_seq_len(regs.fifo.len() as u64)?;
        for word in &regs.fifo {
            w.write_u32(*word)?;
        }
        match regs.dpsm {
            None => w.write_bool(false)?,
            Some(d) => {
                w.write_bool(true)?;
                w.write_bool(d.to_host)?;
                w.write_bool(d.stream)?;
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
            let stream = r.read_bool()?;
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
                stream,
                left,
                block,
                block_left,
                started,
            })
        } else {
            None
        };
        *self.shared.regs.lock() = regs;
        self.shared.refresh_outputs();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            pin::IRQ => {
                self.attach_irq(source);
                Ok(())
            }
            pin::DMA => {
                self.attach_drq(source);
                Ok(())
            }
            other => Err(Error::Config {
                at: String::from(other),
                message: alloc::format!(
                    "an SDIO controller drives `{}` and `{}` and nothing else",
                    pin::IRQ,
                    pin::DMA
                ),
            }),
        }
    }

    fn announce(&self, _port: &str) {
        self.shared.refresh_outputs();
    }
}

impl Instance for Sdio {}

/// The `stm32.sdio` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the STM32 F2/F4/F7 SDIO host controller: the v1 register block, its \
              thirty-two-word FIFO and an external DMA request",
    properties: &[PropertySpec {
        name: "slot",
        kind: ValueKind::Str,
        required: false,
        summary: "the named card slot this controller drives (default `sd0`)",
    }],
    construct: |props| Ok(Box::new(Sdio::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Sdio::new(props)?)))
}

/// What the validator should know about `stm32.sdio`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("slot", ValueKind::Str))
        .port(pin::IRQ, PortDir::Out)
        .port(pin::DMA, PortDir::Out)
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests;
