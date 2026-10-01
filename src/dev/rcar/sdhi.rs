//! The SDHI: Renesas's SD host interface.
//!
//! One SD host controller of the SH-Mobile / R-Car lineage: a command engine
//! that sends one command and latches its response, a one-block data buffer
//! behind a port register, a pair of status registers whose bits the driver
//! clears by writing zeros, and DMA request lines for an external DMA
//! controller to move the data. It talks to an [`SdCard`] through a
//! [`slots::Slot`] rendezvous — the controller owns no storage, the card
//! does, exactly as on the board.
//!
//! # Register map
//!
//! Sixteen-bit registers at their native two-byte spacing (R-Car Gen1 wires
//! the block with no address shift):
//!
//! | Offset | Name | Here |
//! | --- | --- | --- |
//! | `0x00` | `SD_CMD` | writing it issues the command |
//! | `0x02` | `SD_PORTSEL` | stored |
//! | `0x04`, `0x06` | `SD_ARG0`, `SD_ARG1` | the 32-bit argument |
//! | `0x08` | `SD_STOP` | `STP` aborts a transfer, `SEC` enables `SD_SECCNT` and the automatic `CMD12` |
//! | `0x0a` | `SD_SECCNT` | blocks in a multiple-block transfer |
//! | `0x0c`–`0x1a` | `SD_RSP10`…`SD_RSP76` | the response, bits 127:8 of it, low halfword first |
//! | `0x1c` | `SD_INFO1` | `RSPEND`, `ACEND`, card removed/inserted, card detect, write protect |
//! | `0x1e` | `SD_INFO2` | the error bits, `DAT0` level, `BRE`/`BWE`, `SCLKDIVEN`, `CBSY` |
//! | `0x20`, `0x22` | `SD_INFO1_MASK`, `SD_INFO2_MASK` | a set bit masks the interrupt |
//! | `0x24` | `SD_CLK_CTRL` | stored; the card clock is not modelled |
//! | `0x26` | `SD_SIZE` | the block length |
//! | `0x28` | `SD_OPTION` | stored (bus width, timeouts) |
//! | `0x2c`, `0x2e` | `SD_ERR_STS1`, `SD_ERR_STS2` | the error detail; a response timeout sets bit 0 of the second |
//! | `0x30` | `SD_BUF0` | the data port, 16- or 32-bit |
//! | `0x34`–`0x38` | `SDIO_MODE`, `SDIO_INFO1`, its mask | stored; no SDIO card exists here |
//! | `0xd8` | `CC_EXT_MODE` | bit 1: data moves by DMA request rather than by `BRE`/`BWE` |
//! | `0xe0` | `SOFT_RST` | bit 0 clear holds the controller in reset |
//! | `0xe2` | `VERSION` | a fixed value |
//! | `0xe4`, `0xe6` | `HOST_MODE`, `SDIF_MODE` | stored |
//!
//! Any other offset in the window is storage that reads back what was
//! written, so a driver's write-then-verify of a register this model does not
//! interpret still works; it is listed nowhere as modelled.
//!
//! # Time
//!
//! **Zero**, the card model's choice ([`crate::dev::sd::card`]): a command
//! completes inside the write to `SD_CMD`, and a block is in the buffer by the
//! time `RSPEND` is visible. `SCLKDIVEN` therefore always reads set — the
//! divider is never busy — and `CBSY` never does.
//!
//! # DMA
//!
//! With `CC_EXT_MODE` bit 1 set the controller asks for data rather than
//! interrupting for it: `rx-dreq` is high while the buffer holds bytes the
//! host has not read, `tx-dreq` while it has room for bytes the host has not
//! written. A DMA controller wired to those lines reads or writes `SD_BUF0`
//! through the bus like any other master, which is what the silicon does, and
//! this model sees an ordinary access.
//!
//! # Sources
//!
//! The SDHI chapter of the Renesas SH-Mobile and R-Car hardware manuals
//! (register names, the write-zero-to-clear status convention, the `SD_CMD`
//! field layout), the SD Association's Physical Layer Simplified
//! Specification for what the card answers, and a black-box trace of the
//! Alphard navi kernel's driver for which registers it uses and how
//! (`docs/platforms/alphard-navi.md`). No kernel or emulator source was
//! consulted.

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::Width;
use crate::core::wire::{Level, WireSource};
use crate::dev::sd::card::{Data, Reply, SdCard};
use crate::dev::sd::slots::{self, Slot};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.sdhi";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x100;

/// The interrupt output.
pub const IRQ_PIN: &str = "irq";
/// The receive DMA request.
pub const RX_DREQ_PIN: &str = "rx-dreq";
/// The transmit DMA request.
pub const TX_DREQ_PIN: &str = "tx-dreq";
/// Either request: for a board whose DMA controller serves the host on one
/// channel in both directions, reprogrammed per transfer.
pub const DREQ_PIN: &str = "dreq";

// -- registers -----------------------------------------------------------------

const SD_CMD: u64 = 0x00;
const SD_ARG0: u64 = 0x04;
const SD_ARG1: u64 = 0x06;
const SD_STOP: u64 = 0x08;
const SD_SECCNT: u64 = 0x0a;
const SD_RSP_FIRST: u64 = 0x0c;
const SD_RSP_LAST: u64 = 0x1a;
const SD_INFO1: u64 = 0x1c;
const SD_INFO2: u64 = 0x1e;
const SD_INFO1_MASK: u64 = 0x20;
const SD_INFO2_MASK: u64 = 0x22;
const SD_SIZE: u64 = 0x26;
const SD_ERR_STS1: u64 = 0x2c;
const SD_ERR_STS2: u64 = 0x2e;
const SD_BUF0: u64 = 0x30;
const CC_EXT_MODE: u64 = 0xd8;
const SOFT_RST: u64 = 0xe0;
const VERSION: u64 = 0xe2;

// -- SD_CMD fields ---------------------------------------------------------------

/// Bits 10:8: the response type, or zero for "decide from the index".
const CMD_RSP_SHIFT: u16 = 8;
/// Bit 11: a data phase follows.
const CMD_DATA: u16 = 1 << 11;
/// Bit 12: the data phase reads from the card.
const CMD_READ: u16 = 1 << 12;
/// Bit 13: more than one block.
const CMD_MULTI: u16 = 1 << 13;

// -- SD_STOP -------------------------------------------------------------------

/// Abort the transfer in progress.
const STOP_STP: u16 = 1 << 0;
/// `SD_SECCNT` is valid, and the controller issues `CMD12` after the last block.
const STOP_SEC: u16 = 1 << 8;

// -- SD_INFO1 ------------------------------------------------------------------

/// The response has been received.
const INFO1_RSPEND: u16 = 1 << 0;
/// The data transfer, or a command with no data, has ended.
const INFO1_ACEND: u16 = 1 << 2;
/// A card was removed.
const INFO1_RM: u16 = 1 << 3;
/// A card was inserted.
const INFO1_IS: u16 = 1 << 4;
/// Card detect: a card is in the socket. A level, not a flag.
const INFO1_SDCD: u16 = 1 << 5;
/// The write-protect switch reads "writable". A level, not a flag.
const INFO1_SDWP: u16 = 1 << 7;
/// The bits a write can clear.
const INFO1_FLAGS: u16 = INFO1_RSPEND | INFO1_ACEND | INFO1_RM | INFO1_IS;
/// The bits that can interrupt.
const INFO1_IRQ: u16 = INFO1_RSPEND | INFO1_ACEND | INFO1_RM | INFO1_IS;

// -- SD_INFO2 ------------------------------------------------------------------

/// Command error: an index or argument the controller refused.
const INFO2_CMDE: u16 = 1 << 0;
/// A data read ran past what the card would give.
const INFO2_ENDE: u16 = 1 << 2;
/// Data timeout.
const INFO2_DTO: u16 = 1 << 3;
/// Response timeout.
const INFO2_RSPTO: u16 = 1 << 6;
/// `DAT0` is high: the card is not busy. A level.
const INFO2_DAT0: u16 = 1 << 7;
/// The buffer holds data to read.
const INFO2_BRE: u16 = 1 << 8;
/// The buffer has room for data to write.
const INFO2_BWE: u16 = 1 << 9;
/// The clock divider may be changed. A level; always set here.
const INFO2_SCLKDIVEN: u16 = 1 << 13;
/// Illegal access: a buffer access with no transfer.
const INFO2_ILA: u16 = 1 << 15;
/// The bits a write can clear.
const INFO2_FLAGS: u16 = INFO2_CMDE
    | 0b10
    | INFO2_ENDE
    | INFO2_DTO
    | 0b11_0000
    | INFO2_RSPTO
    | INFO2_BRE
    | INFO2_BWE
    | INFO2_ILA;
/// The bits that can interrupt.
const INFO2_IRQ: u16 = INFO2_FLAGS;
/// The levels, which reflect the controller rather than latch.
const INFO2_LEVELS: u16 = INFO2_DAT0 | INFO2_SCLKDIVEN;

/// `CC_EXT_MODE` bit 1: data moves by DMA request.
const EXT_DMA: u16 = 1 << 1;

/// The register lock's place in the SD ladder (`dev::sd`'s `SLOT_RANK`):
/// below the socket, which is looked up first, and above the card's own
/// state, which a command reaches with the registers held.
const REGS_RANK: LockRank = LockRank::new(0x4d00);

/// What `VERSION` reads. The lineage's controllers report an IP revision in the
/// low byte; this is a plausible one and nothing in the traced driver reads it.
const VERSION_VALUE: u16 = 0x0010;

/// The mask registers' reset values: everything masked.
const INFO1_MASK_RESET: u16 = 0x031d;
const INFO2_MASK_RESET: u16 = 0x8b7f;

/// The direction of a data phase.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Dir {
    Read,
    Write,
}

/// A data phase in progress: one block staged in `buf`.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Transfer {
    dir: Dir,
    /// Bytes of the current block: for a read, what the card gave and the
    /// host has yet to take from `pos`; for a write, what the host has given.
    buf: Vec<u8>,
    pos: usize,
    /// Blocks still to move after the current one, or `None` if unbounded
    /// (an open-ended multiple-block transfer ended by `CMD12`).
    left: Option<u32>,
    /// Whether the controller issues `CMD12` itself after the last block.
    auto_stop: bool,
}

/// Everything the guest can see.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    cmd: u16,
    arg: u32,
    stop: u16,
    seccnt: u16,
    /// Bits 127:8 of the last response, as four 32-bit words, low first.
    rsp: [u32; 4],
    info1: u16,
    info2: u16,
    info1_mask: u16,
    info2_mask: u16,
    size: u16,
    err1: u16,
    err2: u16,
    ext_mode: u16,
    soft_rst: u16,
    /// Registers this model stores without interpreting, by offset.
    other: BTreeMap<u16, u16>,
    xfer: Option<Transfer>,
}

impl State {
    fn new() -> State {
        State {
            cmd: 0,
            arg: 0,
            stop: 0,
            seccnt: 0,
            rsp: [0; 4],
            info1: 0,
            info2: 0,
            info1_mask: INFO1_MASK_RESET,
            info2_mask: INFO2_MASK_RESET,
            size: 0x200,
            err1: 0,
            err2: 0,
            ext_mode: 0,
            soft_rst: 1,
            other: BTreeMap::new(),
            xfer: None,
        }
    }
}

struct Registers {
    slot: Arc<Slot>,
    slot_name: String,
    state: Mutex<State>,
    irq: Mutex<Option<WireSource>>,
    rx_dreq: Mutex<Option<WireSource>>,
    tx_dreq: Mutex<Option<WireSource>>,
    dreq: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Registers")
            .field("slot", &self.slot_name)
            .finish_non_exhaustive()
    }
}

/// One SDHI channel.
#[derive(Debug)]
pub struct Sdhi {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Sdhi {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind, or one this
    /// class does not know was given.
    pub fn new(props: &Props) -> Result<Sdhi> {
        let mut r = props.reader();
        let slot_name = r.or("slot", String::from("sd0"))?;
        r.finish()?;
        Ok(Sdhi::with_slot(
            slots::attach(props, &slot_name)?,
            slot_name,
        ))
    }

    /// Build one against a socket the caller already has.
    #[must_use]
    pub fn with_slot(slot: Arc<Slot>, slot_name: String) -> Sdhi {
        let regs = Arc::new(Registers {
            slot,
            slot_name,
            state: Mutex::with_rank(REGS_RANK, State::new()),
            irq: Mutex::with_rank(LockRank::LEAF, None),
            rx_dreq: Mutex::with_rank(LockRank::LEAF, None),
            tx_dreq: Mutex::with_rank(LockRank::LEAF, None),
            dreq: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            "rcar.sdhi",
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Sdhi { regs, region }
    }

    /// Whether the interrupt output is currently asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.regs.outputs().0
    }
}

/// Which data phase, if any, a command in "normal" mode (`SD_CMD` bits 13:8
/// clear) carries: the table the controller decides by when the driver lets
/// it. Keyed by (application command, index).
fn implied_data(app: bool, index: u8) -> Option<(Dir, bool)> {
    match (app, index) {
        (false, 17) | (false, 6) | (true, 13) | (true, 51) | (true, 22) | (false, 30) => {
            Some((Dir::Read, false))
        }
        (false, 18) => Some((Dir::Read, true)),
        (false, 24) | (false, 27) | (false, 42) => Some((Dir::Write, false)),
        (false, 25) => Some((Dir::Write, true)),
        _ => None,
    }
}

/// The length of the data block a command moves, when the command fixes it
/// regardless of `SD_SIZE` (the SD status, the SCR, the switch status).
fn fixed_len(app: bool, index: u8) -> Option<usize> {
    match (app, index) {
        (false, 6) | (true, 13) => Some(64),
        (true, 51) => Some(8),
        (true, 22) => Some(4),
        _ => None,
    }
}

impl Registers {
    /// The card in the socket. Looked up *before* the register lock is
    /// taken — the socket ranks above it (`dev::sd::slot::SLOT_RANK`) — and
    /// handed down to whatever needs it.
    fn card(&self) -> Option<Arc<SdCard>> {
        self.slot.card()
    }

    /// `flag` (BRE or BWE) if the buffer is served by programmed I/O; nothing
    /// in DMA mode, where the same condition is a DMA request instead. A
    /// driver that left BWE unmasked for a DMA write would otherwise take a
    /// PIO interrupt in the middle of it — the navi's does, and its DMA
    /// completion then finds the request already finished.
    fn buffer_flag(state: &State, flag: u16) -> u16 {
        if state.ext_mode & EXT_DMA != 0 {
            0
        } else {
            flag
        }
    }

    /// The card-detect and write-protect levels, from the socket.
    fn levels(card: Option<&Arc<SdCard>>) -> u16 {
        if card.is_some() {
            INFO1_SDCD | INFO1_SDWP
        } else {
            INFO1_SDWP
        }
    }

    /// (interrupt, rx request, tx request), from the state.
    fn outputs_of(state: &State) -> (bool, bool, bool) {
        let irq = (state.info1 & INFO1_IRQ & !state.info1_mask) != 0
            || (state.info2 & INFO2_IRQ & !state.info2_mask) != 0;
        let dma = state.ext_mode & EXT_DMA != 0;
        let (rx, tx) = match &state.xfer {
            Some(t) if dma && t.dir == Dir::Read => (t.pos < t.buf.len(), false),
            Some(t) if dma && t.dir == Dir::Write => (false, true),
            _ => (false, false),
        };
        (irq, rx, tx)
    }

    fn outputs(&self) -> (bool, bool, bool) {
        Self::outputs_of(&self.state.lock())
    }

    /// Drive the three outputs from the state. Called with no lock held.
    fn refresh(&self) {
        let (irq, rx, tx) = self.outputs();
        for (pin, level) in [
            (&self.irq, irq),
            (&self.rx_dreq, rx),
            (&self.tx_dreq, tx),
            (&self.dreq, rx || tx),
        ] {
            let out = pin.lock().clone();
            if let Some(out) = out {
                out.set(if level { Level::High } else { Level::Low });
            }
        }
    }

    /// Issue the command `SD_CMD` names.
    fn issue(&self, state: &mut State, card: Option<&Arc<SdCard>>, value: u16) {
        state.cmd = value;
        if state.soft_rst & 1 == 0 {
            return;
        }
        let index = (value & 0x3f) as u8;
        let app = (value >> 6) & 3 == 1;
        let rsp = (value >> CMD_RSP_SHIFT) & 7;
        state.xfer = None;
        let Some(card) = card else {
            // Nobody on the CMD line: a timeout, unless nothing was expected.
            if rsp != 3 && index != 0 {
                state.info2 |= INFO2_RSPTO;
                state.err2 |= 1;
            } else {
                state.info1 |= INFO1_RSPEND | INFO1_ACEND;
            }
            return;
        };
        let reply = card.command(index, state.arg);
        match reply {
            Reply::None => {
                // Silence. A command with no response (`CMD0`, or an explicit
                // "none" type) has ended; anything else timed out.
                if index == 0 || rsp == 3 {
                    state.rsp = [0; 4];
                    state.info1 |= INFO1_RSPEND | INFO1_ACEND;
                } else {
                    state.info2 |= INFO2_RSPTO;
                    state.err2 |= 1;
                }
                return;
            }
            Reply::Short { value, .. } => {
                state.rsp = [value, 0, 0, 0];
            }
            Reply::Long(words) => {
                // The 128-bit register with its CRC byte shifted out: bits
                // 127:8 of the response, low word first.
                let wide = (u128::from(words[0]) << 96)
                    | (u128::from(words[1]) << 64)
                    | (u128::from(words[2]) << 32)
                    | u128::from(words[3]);
                let shifted = wide >> 8;
                state.rsp = [
                    shifted as u32,
                    (shifted >> 32) as u32,
                    (shifted >> 64) as u32,
                    (shifted >> 96) as u32,
                ];
            }
        }
        state.info1 |= INFO1_RSPEND;

        let data = if value & CMD_DATA != 0 {
            Some((
                if value & CMD_READ != 0 {
                    Dir::Read
                } else {
                    Dir::Write
                },
                value & CMD_MULTI != 0,
            ))
        } else if rsp == 0 {
            implied_data(app, index)
        } else {
            None
        };
        let Some((dir, multi)) = data else {
            // No data phase: the access has ended with the response.
            state.info1 |= INFO1_ACEND;
            return;
        };
        let counted = state.stop & STOP_SEC != 0;
        let blocks = if multi && counted {
            Some(u32::from(state.seccnt.max(1)))
        } else if multi {
            None
        } else {
            Some(1)
        };
        let mut t = Transfer {
            dir,
            buf: Vec::new(),
            pos: 0,
            left: blocks.map(|n| n - 1),
            auto_stop: multi && counted,
        };
        let len = fixed_len(app, index).unwrap_or(usize::from(state.size.max(1)));
        match dir {
            Dir::Read => {
                if !Self::fill(card, &mut t, len) {
                    state.info2 |= INFO2_DTO;
                    return;
                }
                state.info2 |= Self::buffer_flag(state, INFO2_BRE);
            }
            Dir::Write => {
                t.buf.reserve(len);
                state.info2 |= Self::buffer_flag(state, INFO2_BWE);
            }
        }
        state.xfer = Some(t);
    }

    /// Stage the next `len` bytes of a read from the card.
    fn fill(card: &SdCard, t: &mut Transfer, len: usize) -> bool {
        t.buf.clear();
        t.buf.resize(len, 0);
        t.pos = 0;
        card.read_data(&mut t.buf) == Data::Moved
    }

    /// The host has consumed (read) or supplied (write) a whole block: move
    /// to the next one or end the transfer.
    fn block_done(&self, state: &mut State, card: Option<&Arc<SdCard>>) {
        let Some(mut t) = state.xfer.take() else {
            return;
        };
        if t.dir == Dir::Write {
            let ok = card
                .as_ref()
                .is_some_and(|c| c.write_data(&t.buf) != Data::Ended);
            if !ok {
                state.info2 |= INFO2_DTO;
                return;
            }
        }
        let more = match t.left {
            Some(0) => false,
            Some(n) => {
                t.left = Some(n - 1);
                true
            }
            None => true,
        };
        if !more {
            if t.auto_stop
                && let Some(card) = &card
            {
                // The controller's own CMD12; its response lands in the
                // response registers, as the manual describes for the
                // automatic stop.
                if let Reply::Short { value, .. } = card.command(12, 0) {
                    state.rsp = [value, 0, 0, 0];
                }
            }
            state.info2 &= !(INFO2_BRE | INFO2_BWE);
            state.info1 |= INFO1_ACEND;
            return;
        }
        let len = usize::from(state.size.max(1));
        match t.dir {
            Dir::Read => {
                let Some(card) = card else {
                    state.info2 |= INFO2_DTO;
                    return;
                };
                if !Self::fill(card, &mut t, len) {
                    state.info2 |= INFO2_DTO;
                    return;
                }
                state.info2 |= Self::buffer_flag(state, INFO2_BRE);
            }
            Dir::Write => {
                t.buf.clear();
                t.pos = 0;
                state.info2 |= Self::buffer_flag(state, INFO2_BWE);
            }
        }
        state.xfer = Some(t);
    }

    /// Read `n` bytes from the data port.
    fn read_buf(&self, state: &mut State, card: Option<&Arc<SdCard>>, n: usize) -> u32 {
        let Some(t) = state.xfer.as_mut().filter(|t| t.dir == Dir::Read) else {
            state.info2 |= INFO2_ILA;
            return 0;
        };
        let mut value = 0u32;
        for i in 0..n {
            if t.pos < t.buf.len() {
                value |= u32::from(t.buf[t.pos]) << (8 * i);
                t.pos += 1;
            }
        }
        if t.pos >= t.buf.len() {
            state.info2 &= !INFO2_BRE;
            self.block_done(state, card);
        }
        value
    }

    /// Write `n` bytes to the data port.
    fn write_buf(&self, state: &mut State, card: Option<&Arc<SdCard>>, value: u32, n: usize) {
        let len = usize::from(state.size.max(1));
        let Some(t) = state.xfer.as_mut().filter(|t| t.dir == Dir::Write) else {
            state.info2 |= INFO2_ILA;
            return;
        };
        for i in 0..n {
            if t.buf.len() < len {
                t.buf.push((value >> (8 * i)) as u8);
            }
        }
        if t.buf.len() >= len {
            state.info2 &= !INFO2_BWE;
            self.block_done(state, card);
        }
    }

    /// Read a sixteen-bit register (the data port is handled by the caller).
    fn read_reg(&self, state: &State, card: Option<&Arc<SdCard>>, offset: u64) -> u16 {
        match offset {
            SD_CMD => state.cmd,
            SD_ARG0 => state.arg as u16,
            SD_ARG1 => (state.arg >> 16) as u16,
            SD_STOP => state.stop,
            SD_SECCNT => state.seccnt,
            SD_RSP_FIRST..=SD_RSP_LAST => {
                let half = ((offset - SD_RSP_FIRST) / 2) as usize;
                (state.rsp[half / 2] >> (16 * (half % 2))) as u16
            }
            SD_INFO1 => (state.info1 & INFO1_FLAGS) | Self::levels(card),
            SD_INFO2 => (state.info2 & !INFO2_LEVELS) | INFO2_LEVELS,
            SD_INFO1_MASK => state.info1_mask,
            SD_INFO2_MASK => state.info2_mask,
            SD_SIZE => state.size,
            SD_ERR_STS1 => state.err1,
            SD_ERR_STS2 => state.err2,
            CC_EXT_MODE => state.ext_mode,
            SOFT_RST => state.soft_rst,
            VERSION => VERSION_VALUE,
            _ => state.other.get(&(offset as u16)).copied().unwrap_or(0),
        }
    }

    /// Write a sixteen-bit register (the data port is handled by the caller).
    fn write_reg(&self, state: &mut State, card: Option<&Arc<SdCard>>, offset: u64, value: u16) {
        match offset {
            SD_CMD => self.issue(state, card, value),
            SD_ARG0 => state.arg = (state.arg & 0xffff_0000) | u32::from(value),
            SD_ARG1 => state.arg = (state.arg & 0x0000_ffff) | (u32::from(value) << 16),
            SD_STOP => {
                state.stop = value & !STOP_STP;
                if value & STOP_STP != 0 && state.xfer.take().is_some() {
                    state.info2 &= !(INFO2_BRE | INFO2_BWE);
                    state.info1 |= INFO1_ACEND;
                }
            }
            SD_SECCNT => state.seccnt = value,
            SD_RSP_FIRST..=SD_RSP_LAST => {}
            // Write-zero-to-clear: a 1 leaves a flag as it was.
            SD_INFO1 => state.info1 &= value | !INFO1_FLAGS,
            SD_INFO2 => state.info2 &= value | !INFO2_FLAGS,
            SD_INFO1_MASK => state.info1_mask = value,
            SD_INFO2_MASK => state.info2_mask = value,
            SD_SIZE => state.size = value & 0x3ff,
            SD_ERR_STS1 | SD_ERR_STS2 | VERSION => {}
            CC_EXT_MODE => state.ext_mode = value,
            SOFT_RST => {
                if value & 1 == 0 {
                    // Reset asserted: the command engine and the buffer go
                    // back to idle; the masks and the stored registers stay.
                    state.xfer = None;
                    state.info1 = 0;
                    state.info2 = 0;
                    state.err1 = 0;
                    state.err2 = 0;
                }
                state.soft_rst = value;
            }
            _ => {
                state.other.insert(offset as u16, value);
            }
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let n = dst.len();
        if !matches!(n, 1 | 2 | 4) {
            return Err(BusError::BadAccess);
        }
        let card = self.card();
        let card = card.as_ref();
        let value = {
            let mut state = self.state.lock();
            if offset & !3 == SD_BUF0 {
                if attrs.debug {
                    // A debugger's look at the port pops nothing.
                    state
                        .xfer
                        .as_ref()
                        .map_or(0, |t| t.buf.get(t.pos).copied().map_or(0, u32::from))
                } else {
                    self.read_buf(&mut state, card, n)
                }
            } else if n == 4 {
                u32::from(self.read_reg(&state, card, offset))
                    | (u32::from(self.read_reg(&state, card, offset + 2)) << 16)
            } else {
                let half = self.read_reg(&state, card, offset & !1);
                u32::from(if offset & 1 != 0 { half >> 8 } else { half })
            }
        };
        dst.copy_from_slice(&value.to_le_bytes()[..n]);
        if !attrs.debug && offset & !3 == SD_BUF0 {
            self.refresh();
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let n = src.len();
        if !matches!(n, 1 | 2 | 4) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        let mut value = 0u32;
        for (i, b) in src.iter().enumerate() {
            value |= u32::from(*b) << (8 * i);
        }
        let card = self.card();
        let card = card.as_ref();
        {
            let mut state = self.state.lock();
            if offset & !3 == SD_BUF0 {
                self.write_buf(&mut state, card, value, n);
            } else if n == 4 {
                // A 32-bit write covers two registers; the high one second,
                // so an argument or a status pair lands in order.
                self.write_reg(&mut state, card, offset, value as u16);
                self.write_reg(&mut state, card, offset + 2, (value >> 16) as u16);
            } else if n == 2 {
                self.write_reg(&mut state, card, offset & !1, value as u16);
            } else {
                let old = self.read_reg(&state, card, offset & !1);
                let merged = if offset & 1 != 0 {
                    (old & 0x00ff) | ((value as u16) << 8)
                } else {
                    (old & 0xff00) | (value as u16 & 0xff)
                };
                self.write_reg(&mut state, card, offset & !1, merged);
            }
        }
        self.refresh();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO
            .with_widths(Width::U8, Width::U32)
            .with_natural_alignment(true)
    }
}

/// The `rcar.sdhi` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Renesas SDHI: an SD host controller with a one-block buffer and DMA requests",
    properties: &[PropertySpec {
        name: "slot",
        kind: ValueKind::Str,
        required: false,
        summary: "the card socket this controller drives, by name (default \"sd0\")",
    }],
    construct: |props| Ok(Box::new(Sdhi::new(props)?)),
};

impl Device for Sdhi {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.regs.state.lock() = State::new();
        self.regs.refresh();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let pin = match port {
            IRQ_PIN => &self.regs.irq,
            RX_DREQ_PIN => &self.regs.rx_dreq,
            TX_DREQ_PIN => &self.regs.tx_dreq,
            DREQ_PIN => &self.regs.dreq,
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: String::from("an SDHI drives `irq`, `rx-dreq`, `tx-dreq` and `dreq`"),
                });
            }
        };
        *pin.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.regs.refresh();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        for half in [
            state.cmd,
            state.stop,
            state.seccnt,
            state.info1,
            state.info2,
            state.info1_mask,
            state.info2_mask,
            state.size,
            state.err1,
            state.err2,
            state.ext_mode,
            state.soft_rst,
        ] {
            w.write_u16(half)?;
        }
        w.write_u32(state.arg)?;
        for word in state.rsp {
            w.write_u32(word)?;
        }
        w.write_seq_len(state.other.len() as u64)?;
        for (k, v) in &state.other {
            w.write_u16(*k)?;
            w.write_u16(*v)?;
        }
        match &state.xfer {
            None => w.write_bool(false)?,
            Some(t) => {
                w.write_bool(true)?;
                w.write_bool(t.dir == Dir::Read)?;
                w.write_bytes(&t.buf)?;
                w.write_u32(t.pos as u32)?;
                w.write_bool(t.left.is_some())?;
                w.write_u32(t.left.unwrap_or(0))?;
                w.write_bool(t.auto_stop)?;
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::new();
        state.cmd = r.read_u16()?;
        state.stop = r.read_u16()?;
        state.seccnt = r.read_u16()?;
        state.info1 = r.read_u16()?;
        state.info2 = r.read_u16()?;
        state.info1_mask = r.read_u16()?;
        state.info2_mask = r.read_u16()?;
        state.size = r.read_u16()?;
        state.err1 = r.read_u16()?;
        state.err2 = r.read_u16()?;
        state.ext_mode = r.read_u16()?;
        state.soft_rst = r.read_u16()?;
        state.arg = r.read_u32()?;
        for word in &mut state.rsp {
            *word = r.read_u32()?;
        }
        let count = r.read_seq_len(4)?;
        for _ in 0..count {
            let k = r.read_u16()?;
            let v = r.read_u16()?;
            state.other.insert(k, v);
        }
        if r.read_bool()? {
            let dir = if r.read_bool()? {
                Dir::Read
            } else {
                Dir::Write
            };
            let buf = r.read_bytes()?.to_vec();
            let pos = r.read_u32()? as usize;
            let bounded = r.read_bool()?;
            let left = r.read_u32()?;
            let auto_stop = r.read_bool()?;
            state.xfer = Some(Transfer {
                dir,
                buf,
                pos,
                left: bounded.then_some(left),
                auto_stop,
            });
        }
        *self.regs.state.lock() = state;
        self.regs.refresh();
        Ok(())
    }
}

impl Instance for Sdhi {}

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind this class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Sdhi::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("slot", ValueKind::Str))
        .region("")
        .region("regs")
        .port(IRQ_PIN, PortDir::Out)
        .port(RX_DREQ_PIN, PortDir::Out)
        .port(TX_DREQ_PIN, PortDir::Out)
        .port(DREQ_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests;
