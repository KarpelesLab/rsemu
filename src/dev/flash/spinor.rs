//! SPI NOR flash: a Winbond **W25Q**-class serial part, on the SPI bus.
//!
//! This is the same silicon as [`super::cfi`] with a different front door. The
//! three properties that make a NOR part a *device* rather than memory with
//! another name are unchanged, and a model that drops any one of them lets
//! firmware succeed at something the chip would have refused:
//!
//! 1. **A program can only clear bits.** Writing `0xf0` over `0x0f` leaves
//!    `0x00`. Every fault-tolerant-write scheme depends on it.
//! 2. **Setting bits takes an erase, and an erase takes a whole granule** — a
//!    4 KiB sector, a 32 KiB half block, a 64 KiB block, or the chip.
//! 3. **The array is not always what a read returns.** After `05h` the part
//!    answers with its status register until the chip select rises, and
//!    firmware polls exactly that window.
//!
//! What is different is the *interface*. A CFI part is a parallel bus: an
//! address and a data word arrive together and a command is a write cycle.
//! A serial part has neither — it has one chip select, one clock and one data
//! line, and **a command is a frame**: an opcode byte, some address bytes, some
//! dummy bytes, then a data stream, all delimited by the chip select. So the
//! interesting state here is a *position in a frame*, and the interesting rule
//! is the datasheet's own:
//!
//! > If the chip select is driven high before the instruction is complete, the
//! > instruction is not executed.
//!
//! which is why every mutating command is **staged** while the frame runs and
//! applied in [`SpiSlave::select`] when CS rises, never when its last byte
//! lands. A snapshot taken mid-frame therefore carries a page-program buffer
//! that has been latched and not committed, exactly as the silicon would hold
//! it.
//!
//! # This is an [`SpiSlave`], not a bus device
//!
//! It has no memory-mapped region of its own. It answers whatever clocks it —
//! the generic [`spi.controller`](crate::bus::spi::controller), an
//! [`stm32.spi`](crate::dev::stm32::spi), an
//! [`stm32.octospi`](crate::dev::stm32::octospi) in either of its modes, or a
//! guest bit-banging GPIO pins — because [`SlavePins`] gives the word-level
//! model its bit-level front end for free (`docs/buses/low-speed.md`).
//!
//! # One data line, and what that costs
//!
//! `bus::spi` models **one** MOSI and one MISO, which is what SPI is. The dual
//! and quad commands (`3Bh`, `BBh`, `6Bh`, `EBh`) exist on this part because
//! everything real uses them, and they are decoded here: the opcode is
//! accepted, the address, mode and dummy phases are consumed, the `QE` bit
//! gates the quad ones, and the **byte stream is exactly right**. What is not
//! modelled is that their address and data phases take a half or a quarter of
//! the clocks — the seam has no notion of how many lines a phase uses, so
//! every command here costs single-line time.
//!
//! The dummy phases follow from converting the datasheet's *clock* counts into
//! *bytes* at the line count each phase uses, which is the only conversion
//! that keeps a controller and this part agreeing about where the data starts
//! (§8.1.3, Instruction Set Table 2):
//!
//! | Command | Datasheet | Bytes here |
//! | --- | --- | --- |
//! | `0Bh`, `3Bh`, `6Bh` | 8 dummy clocks on one line | 1 |
//! | `BBh` | 4 mode clocks on two lines, no dummy | 1 |
//! | `EBh` | 2 mode clocks on four lines, then 4 dummy clocks | 3 |
//!
//! **`M7-M0` has no effect on this part.** The `JV` generation dropped the
//! `FV`'s continuous-read latch: the datasheet's only statements about the
//! mode byte are the three notes saying it "should be set to `Fxh`", and there
//! is no paragraph anywhere letting a following frame omit its opcode. It is
//! consumed and discarded here, and a model that latched it would be inventing
//! an `FV` behaviour this part does not have.
//!
//! # Time
//!
//! **Deliberately zero**, as [`super::cfi`] chose and for one more reason
//! besides. A program or an erase completes inside the frame that commits it,
//! so `BUSY` reads clear the moment firmware polls it.
//!
//! * The polling loop firmware actually writes — `05h` until bit 0 clears —
//!   terminates immediately here instead of after the datasheet's 400 ms, and
//!   nothing else a guest can do tells the difference.
//! * A busy window needs a scheduler event, and an event needs a clock domain.
//!   The part has no clock input worth the name: `SCK` is the *master's*, and
//!   the internal program/erase timer is an on-die oscillator no board wires
//!   and no device tree describes. Giving this device a domain would mean
//!   inventing a crystal for it.
//! * And a serial slave is reached **from inside its controller's own
//!   catch-up** — [`SpiSlave::transfer`] runs under [`Device::advance_to`] — so
//!   a busy window would have to arm an event from a re-entrant position the
//!   lazy seam deliberately refuses. That is not a thing to work around
//!   quietly.
//!
//! The status register still reports what a real part would, and a guest that
//! needs a real busy window should say so and this device will grow a domain.
//!
//! # Sources
//!
//! * **Winbond W25Q128JV** *3V 128M-bit serial flash memory with dual/quad SPI*,
//!   **revision F (27 March 2018)**, cross-checked against revision H — the
//!   instruction set tables (§8.1.2, §8.1.3), the identification bytes
//!   (§8.1.1), the three status registers (§7.1) and their protection table
//!   (§7.1.9), the page-program wrap rule (§8.2.13), the erase instructions
//!   (§8.2.15-§8.2.18), the "instruction is not executed" framing rule (§8 and
//!   §8.2.13), what a busy part answers (§8.2.6), deep power-down (§8.2.21),
//!   the `66h`/`99h` reset (§8.2.37) and the power-up state (§6.5). The
//!   W25Q64JV/W25Q32JV/W25Q16JV differ only in the density and the capacity
//!   byte, which are properties here.
//!
//!   Two of that datasheet's facts are easy to get backwards and are asserted
//!   in the tests: the legacy device id `90h` returns is **one less** than the
//!   capacity byte `9Fh` returns (`17h` against `18h`), and the memory-type
//!   byte is an **ordering option rather than a density** — `40h` is the
//!   `-IQ`/`-JQ` part whose `QE` bit is fixed set, `70h` the `-IM`/`-JM` part
//!   whose `QE` powers up clear and is programmable (§11, Ordering
//!   Information).
//! * **JEDEC JEP106** for the manufacturer identifier, `EFh` (bank 1) for
//!   Winbond.
//!
//! No emulator source of any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::spi::{
    BitOrder, ChipSelect, Format, MAX_CHIP_SELECTS, Mode, SlavePins, SpiSlave, buses,
    pin as spi_pin,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::RamStore;
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{WireId, WireSource};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "flash.spinor";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// Winbond's JEDEC manufacturer identifier (JEP106, bank 1 code `EFh`).
pub const WINBOND: u8 = 0xef;

/// The memory-type byte a W25Q…JV returns in standard SPI: `40h`.
pub const TYPE_W25Q: u8 = 0x40;

/// The programming page, in bytes. Every W25Q part uses 256.
pub const PAGE: u64 = 256;

/// The smallest erase granule: the 4 KiB sector of `20h`.
pub const SECTOR: u64 = 4 * 1024;

/// The 32 KiB half block of `52h`.
pub const HALF_BLOCK: u64 = 32 * 1024;

/// The 64 KiB block of `D8h`.
pub const BLOCK: u64 = 64 * 1024;

/// The size a machine file gets when it does not say: 16 MiB, a W25Q128.
pub const DEFAULT_SIZE: u64 = 16 * 1024 * 1024;

// ---------------------------------------------------------------------------
// the instruction set (W25Q128JV §8.1)
// ---------------------------------------------------------------------------

/// Write Enable: sets `WEL`, without which nothing can be programmed or erased.
const CMD_WRITE_ENABLE: u8 = 0x06;
/// Volatile SR Write Enable: `01h` after this writes the volatile status bits.
const CMD_VOLATILE_SR_WRITE_ENABLE: u8 = 0x50;
/// Write Disable: clears `WEL`.
const CMD_WRITE_DISABLE: u8 = 0x04;
/// Read Status Register-1.
const CMD_READ_STATUS1: u8 = 0x05;
/// Read Status Register-2.
const CMD_READ_STATUS2: u8 = 0x35;
/// Read Status Register-3.
const CMD_READ_STATUS3: u8 = 0x15;
/// Write Status Register-1. One or two data bytes; the second is SR-2.
const CMD_WRITE_STATUS1: u8 = 0x01;
/// Write Status Register-2.
const CMD_WRITE_STATUS2: u8 = 0x31;
/// Write Status Register-3.
const CMD_WRITE_STATUS3: u8 = 0x11;
/// Read Data: address, then the array at full speed.
const CMD_READ: u8 = 0x03;
/// Fast Read: address, eight dummy clocks, then the array.
const CMD_FAST_READ: u8 = 0x0b;
/// Fast Read Dual Output.
const CMD_FAST_READ_DUAL_OUT: u8 = 0x3b;
/// Fast Read Dual I/O: address on two lines, then a mode byte.
const CMD_FAST_READ_DUAL_IO: u8 = 0xbb;
/// Fast Read Quad Output. Needs `QE`.
const CMD_FAST_READ_QUAD_OUT: u8 = 0x6b;
/// Fast Read Quad I/O: address on four lines, a mode byte, then dummies.
const CMD_FAST_READ_QUAD_IO: u8 = 0xeb;
/// Page Program: address, then up to a page of data.
const CMD_PAGE_PROGRAM: u8 = 0x02;
/// Quad Input Page Program. Same bytes, four lines.
const CMD_QUAD_PAGE_PROGRAM: u8 = 0x32;
/// Sector Erase, 4 KiB.
const CMD_SECTOR_ERASE: u8 = 0x20;
/// Block Erase, 32 KiB.
const CMD_HALF_BLOCK_ERASE: u8 = 0x52;
/// Block Erase, 64 KiB.
const CMD_BLOCK_ERASE: u8 = 0xd8;
/// Chip Erase.
const CMD_CHIP_ERASE: u8 = 0xc7;
/// The alternate encoding of Chip Erase.
const CMD_CHIP_ERASE_ALT: u8 = 0x60;
/// Read JEDEC ID: manufacturer, memory type, capacity.
const CMD_JEDEC_ID: u8 = 0x9f;
/// Read Manufacturer / Device ID, with a three-byte address.
const CMD_DEVICE_ID: u8 = 0x90;
/// The dual-I/O encoding of the same.
const CMD_DEVICE_ID_DUAL: u8 = 0x92;
/// The quad-I/O encoding of the same.
const CMD_DEVICE_ID_QUAD: u8 = 0x94;
/// Read Unique ID: four dummy bytes, then a 64-bit factory serial.
const CMD_UNIQUE_ID: u8 = 0x4b;
/// Read SFDP Register (JESD216).
const CMD_READ_SFDP: u8 = 0x5a;
/// Power-down.
const CMD_POWER_DOWN: u8 = 0xb9;
/// Release Power-down / Device ID.
const CMD_RELEASE_POWER_DOWN: u8 = 0xab;
/// Enable Reset. Only `99h` immediately after it resets the part.
const CMD_ENABLE_RESET: u8 = 0x66;
/// Reset Device.
const CMD_RESET: u8 = 0x99;
/// Enter 4-byte address mode, on parts larger than 128 Mbit.
const CMD_ENTER_4B: u8 = 0xb7;
/// Exit 4-byte address mode.
const CMD_EXIT_4B: u8 = 0xe9;

// -- status register bits (W25Q128JV §7.1) ----------------------------------

/// SR-1 bit 0, `BUSY`: an erase or program is in progress. See the module docs
/// for why this model never leaves it set.
const SR1_BUSY: u8 = 1 << 0;
/// SR-1 bit 1, `WEL`: a write is enabled. Cleared by every completed write.
const SR1_WEL: u8 = 1 << 1;
/// SR-1 bits 4:2, `BP2..BP0`: how much of the array is protected.
const SR1_BP_SHIFT: u32 = 2;
/// And their mask, once shifted down.
const SR1_BP_MASK: u8 = 0x07;
/// SR-1 bit 5, `TB`: the protected range is at the bottom rather than the top.
const SR1_TB: u8 = 1 << 5;
/// SR-1 bit 6, `SEC`: the protected range is counted in 4 KiB sectors.
const SR1_SEC: u8 = 1 << 6;
/// SR-1 bit 7, `SRP` (S7): with `WP#` low, the status register itself is
/// locked. The datasheet calls it `SRP`, not `SRP0` — `SRP0`/`SRP1` is other
/// vendors' spelling, and §7.1.1's companion bit is `SRL` in SR-2.
const SR1_SRP: u8 = 1 << 7;
/// The SR-1 bits `01h` may change. `BUSY` and `WEL` are the part's own.
const SR1_WRITABLE: u8 = SR1_BP_MASK << SR1_BP_SHIFT | SR1_TB | SR1_SEC | SR1_SRP;

/// SR-2 bit 0, `SRL` (S8): status-register lock.
const SR2_SRL: u8 = 1 << 0;
/// SR-2 bit 1, `QE` (S9): quad enable. Without it `6Bh`, `EBh` and `32h` are
/// refused.
const SR2_QE: u8 = 1 << 1;
/// SR-2 bit 6, `CMP` (S14): complement the protected range.
const SR2_CMP: u8 = 1 << 6;
/// The SR-2 bits `31h` may change: `SRL`, `QE`, the three security-register
/// lock bits and `CMP`. `SUS` (S15) is the part's own status.
const SR2_WRITABLE: u8 = 0x7f;

/// SR-3 bit 0, `ADS`: the part is in 4-byte address mode. Read-only; `B7h` and
/// `E9h` move it.
///
/// **Not a W25Q128JV bit.** That part is 24-bit addressed and its §7.1 shows
/// S16 and S17 reserved; `ADS`/`ADP` belong to the 256 Mbit and larger family
/// members. They are here because `size` may name one of those, and a part
/// configured for three-byte addressing simply never sets them.
const SR3_ADS: u8 = 1 << 0;
/// SR-3 bit 1, `ADP`: the mode the part powers up in. See [`SR3_ADS`].
const SR3_ADP: u8 = 1 << 1;
/// SR-3 bit 2, `WPS` (S18): write protection selection — the individual block
/// locks rather than the `BP` scheme. Stored and read back; this model always
/// applies the `BP` scheme, and says so in the class summary.
const SR3_WPS: u8 = 1 << 2;
/// SR-3 bits 6:5, `DRV1`/`DRV0` (S22, S21): output drive strength. `11` is the
/// factory default and means 25% (§7.1.6).
const SR3_DRV: u8 = 3 << 5;
/// The SR-3 bits `11h` may change: the drive strength and `WPS`. `ADS` is a
/// status rather than a setting, and the rest is reserved.
const SR3_WRITABLE: u8 = SR3_DRV | SR3_WPS;

/// What an undriven, pulled-up MISO reads as, and what this part presents when
/// it has nothing to say.
const IDLE_BYTE: u8 = 0xff;

fn config(message: String) -> Error {
    Error::Config {
        at: CLASS_NAME.to_string(),
        message,
    }
}

// ---------------------------------------------------------------------------
// where a frame is
// ---------------------------------------------------------------------------

/// Which part of the current frame the next byte belongs to.
///
/// This is the whole difference between a serial part and a parallel one: a
/// command is a *position*, and the position is what a snapshot has to carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// Waiting for the instruction byte.
    Opcode,
    /// Collecting address bytes.
    Address,
    /// Consuming mode or dummy bytes.
    Dummy,
    /// Streaming data, in whichever direction [`Stream`] says.
    Data,
    /// An instruction this part does not implement. Everything to the next
    /// rising edge of CS is discarded, which is what the silicon does.
    Ignored,
}

impl Phase {
    const fn tag(self) -> u8 {
        match self {
            Phase::Opcode => 0,
            Phase::Address => 1,
            Phase::Dummy => 2,
            Phase::Data => 3,
            Phase::Ignored => 4,
        }
    }

    fn from_tag(tag: u8) -> Result<Phase> {
        match tag {
            0 => Ok(Phase::Opcode),
            1 => Ok(Phase::Address),
            2 => Ok(Phase::Dummy),
            3 => Ok(Phase::Data),
            4 => Ok(Phase::Ignored),
            other => Err(Error::State(format!("{other} is not an SPI flash phase"))),
        }
    }
}

/// What the data phase of the current frame carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stream {
    /// Nothing; the part drives the idle level.
    None,
    /// The array, from `addr` upwards, wrapping at the end of the part.
    Array,
    /// One status register, repeated for as long as the master clocks.
    Status(u8),
    /// Manufacturer, memory type, capacity — repeating (`9Fh`).
    JedecId,
    /// Manufacturer and device id, alternating (`90h`).
    DeviceId,
    /// The 64-bit factory serial (`4Bh`).
    UniqueId,
    /// The SFDP window. See [`Stream::Sfdp`]'s use below: this part reports no
    /// table.
    Sfdp,
    /// Incoming page-program data.
    Program,
    /// Incoming status-register bytes.
    StatusIn,
    /// The device id alone, repeated — what `ABh` answers with after its three
    /// dummy bytes. Distinct from [`Stream::DeviceId`], which alternates with
    /// the manufacturer because `90h` does.
    JustDeviceId,
}

impl Stream {
    const fn tag(self) -> u8 {
        match self {
            Stream::None => 0,
            Stream::Array => 1,
            Stream::Status(_) => 2,
            Stream::JedecId => 3,
            Stream::DeviceId => 4,
            Stream::UniqueId => 5,
            Stream::Sfdp => 6,
            Stream::Program => 7,
            Stream::StatusIn => 8,
            Stream::JustDeviceId => 9,
        }
    }

    fn from_tag(tag: u8, which: u8) -> Result<Stream> {
        match tag {
            0 => Ok(Stream::None),
            1 => Ok(Stream::Array),
            2 => Ok(Stream::Status(which)),
            3 => Ok(Stream::JedecId),
            4 => Ok(Stream::DeviceId),
            5 => Ok(Stream::UniqueId),
            6 => Ok(Stream::Sfdp),
            7 => Ok(Stream::Program),
            8 => Ok(Stream::StatusIn),
            9 => Ok(Stream::JustDeviceId),
            other => Err(Error::State(format!("{other} is not an SPI flash stream"))),
        }
    }
}

/// A command that has been received in full and will take effect when the chip
/// select rises — and only then.
///
/// The datasheet's rule, and the reason this type exists rather than the
/// commands applying where their last byte lands: an instruction interrupted by
/// CS going high **is not executed**. A snapshot in the middle of one restores
/// a part with the same half-issued command, which is what the silicon holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Staged {
    /// Nothing to do.
    None,
    /// `06h` — set `WEL`.
    WriteEnable,
    /// `50h` — set `WEL` for a volatile status-register write. Modelled
    /// identically: this part has no separate non-volatile status latch.
    VolatileWriteEnable,
    /// `04h` — clear `WEL`.
    WriteDisable,
    /// `02h`/`32h` — program the staged page at this base.
    Program {
        /// The start of the 256-byte page the frame addressed.
        base: u64,
    },
    /// `20h`/`52h`/`D8h`/`C7h` — erase this span, aligned down.
    Erase {
        /// Where the granule starts.
        base: u64,
        /// How long it is. The whole part, for a chip erase.
        span: u64,
    },
    /// `01h`/`31h`/`11h` — take the staged bytes into the status registers.
    WriteStatus {
        /// Which register the opcode named, 1 to 3.
        first: u8,
    },
    /// `B9h`.
    PowerDown,
    /// `ABh`.
    ReleasePowerDown,
    /// `66h` — arm the reset. Only `99h` immediately after it does anything.
    EnableReset,
    /// `99h`.
    Reset,
    /// `B7h`/`E9h` — enter or leave 4-byte address mode.
    AddressBytes {
        /// 3 or 4.
        bytes: u8,
    },
}

impl Staged {
    const fn tag(self) -> u8 {
        match self {
            Staged::None => 0,
            Staged::WriteEnable => 1,
            Staged::VolatileWriteEnable => 2,
            Staged::WriteDisable => 3,
            Staged::Program { .. } => 4,
            Staged::Erase { .. } => 5,
            Staged::WriteStatus { .. } => 6,
            Staged::PowerDown => 7,
            Staged::ReleasePowerDown => 8,
            Staged::EnableReset => 9,
            Staged::Reset => 10,
            Staged::AddressBytes { .. } => 11,
        }
    }
}

/// The 256-byte page-program latch.
///
/// Held as an accumulator rather than a list of latched bytes so that its size
/// is bounded whatever the master clocks: the datasheet says a program that
/// runs past the end of a page **wraps to the start of the same page**, so a
/// master that clocks a megabyte writes the same 256 bytes over and over. AND
/// is idempotent and commutes, so accumulating is not merely a compression of
/// the byte list — it is the same answer.
#[derive(Clone)]
struct PageLatch {
    /// What each byte of the page will be ANDed with.
    data: [u8; PAGE as usize],
    /// Which of them the frame actually latched. An untouched byte is left
    /// alone, which is what a short page program does.
    touched: [bool; PAGE as usize],
    /// Where the next byte lands. Wraps within the page.
    at: u8,
    /// How many bytes have been latched, saturating. Only "was there at least
    /// one" matters, and a `bool` would read as if it did not count.
    latched: u32,
}

impl fmt::Debug for PageLatch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PageLatch")
            .field("at", &self.at)
            .field("latched", &self.latched)
            .finish_non_exhaustive()
    }
}

impl PageLatch {
    fn new() -> PageLatch {
        PageLatch {
            data: [0xff; PAGE as usize],
            touched: [false; PAGE as usize],
            at: 0,
            latched: 0,
        }
    }

    fn reset(&mut self, start: u8) {
        self.data = [0xff; PAGE as usize];
        self.touched = [false; PAGE as usize];
        self.at = start;
        self.latched = 0;
    }

    fn push(&mut self, byte: u8) {
        let at = usize::from(self.at);
        self.data[at] &= byte;
        self.touched[at] = true;
        self.at = self.at.wrapping_add(1);
        self.latched = self.latched.saturating_add(1);
    }
}

// ---------------------------------------------------------------------------
// the part
// ---------------------------------------------------------------------------

/// Everything the guest can see or change.
#[derive(Debug, Clone)]
struct State {
    phase: Phase,
    stream: Stream,
    /// The byte the part is presenting on MISO *now*. Full duplex: what
    /// [`SpiSlave::transfer`] returns is what was already here when the word
    /// began, never a reply to it.
    out: u8,
    /// The address the frame is working at.
    addr: u64,
    /// How many address bytes have arrived.
    got: u8,
    /// How many mode or dummy bytes are still to come.
    dummy: u8,
    /// The granule an erase opcode named, waiting for its address. Zero when
    /// the frame is not an erase — which is also why an erase whose address
    /// never finished stages nothing.
    span: u64,
    /// How far into a repeating identifier response the master has clocked.
    count: u64,
    sr1: u8,
    sr2: u8,
    sr3: u8,
    /// Deep power-down. Only `ABh` is answered while it is set.
    powered_down: bool,
    /// `66h` was the last completed frame, so `99h` will reset the part.
    reset_armed: bool,
    /// What the rising edge of CS will do.
    staged: Staged,
    /// Status-register bytes latched by `01h`/`31h`/`11h`.
    sr_in: [u8; 3],
    /// How many of them arrived.
    sr_in_len: u8,
    /// The page-program latch.
    page: PageLatch,
}

impl State {
    fn new(addr_bytes: u8, qe_fixed: bool) -> State {
        State {
            phase: Phase::Opcode,
            stream: Stream::None,
            out: IDLE_BYTE,
            addr: 0,
            got: 0,
            dummy: 0,
            span: 0,
            count: 0,
            sr1: 0,
            // A `-IQ`/`-JQ` part has `QE` fixed set; a `-IM`/`-JM` part powers
            // up with it clear (§11, and §7.1.4's note).
            sr2: if qe_fixed { SR2_QE } else { 0 },
            // The drive strength powers up at `11`, 25% (§7.1.6). `ADP`
            // records the mode the part powers up in and `ADS` reports the one
            // it is in, so a part configured for four-byte addressing powers
            // up already there.
            sr3: SR3_DRV
                | if addr_bytes == 4 {
                    SR3_ADS | SR3_ADP
                } else {
                    0
                },
            powered_down: false,
            reset_armed: false,
            staged: Staged::None,
            sr_in: [0; 3],
            sr_in_len: 0,
            page: PageLatch::new(),
        }
    }

    /// How many address bytes a frame carries right now.
    const fn addr_bytes(&self) -> u8 {
        if self.sr3 & SR3_ADS != 0 { 4 } else { 3 }
    }

    /// Start a fresh frame, keeping everything that survives a chip select.
    fn begin_frame(&mut self) {
        self.phase = Phase::Opcode;
        self.stream = Stream::None;
        self.addr = 0;
        self.got = 0;
        self.dummy = 0;
        self.span = 0;
        self.count = 0;
        self.staged = Staged::None;
        self.sr_in_len = 0;
        self.out = IDLE_BYTE;
    }
}

/// Everything both halves of the device reach.
struct Shared {
    /// The contents. A [`RamStore`] for the same reasons [`super::cfi`] uses
    /// one: byte addressed, `Sync` without `unsafe`, never handed out as a
    /// slice, so it can live in a `SharedArrayBuffer`.
    array: Arc<RamStore>,
    size: u64,
    /// How this part frames a word on the wire.
    format: Format,
    /// Manufacturer, memory type, capacity — what `9Fh` answers with.
    jedec: [u8; 3],
    /// What `90h` and `ABh` answer with.
    device_id: u8,
    /// The factory serial `4Bh` reports.
    unique_id: u64,
    /// Whether the part powers up in 4-byte address mode.
    addr_bytes: u8,
    /// Whether `QE` is fixed set, as it is on the `-IQ`/`-JQ` ordering option
    /// the `40h` memory-type byte names.
    qe_fixed: bool,
    /// `WP#` tied low: the status register cannot be written while `SRP0` is
    /// set, which is how a board makes its protection stick.
    write_protect: bool,
    state: Mutex<State>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("SpiNor");
        s.field("size", &self.size).field("jedec", &self.jedec);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Shared {
    /// One byte of the array. Off the end reads as erased, which is what a
    /// part with fewer address pins than the master drove would return from
    /// somewhere else in the array; wrapping is the modelled behaviour and
    /// this is only the belt.
    fn byte(&self, addr: u64) -> u8 {
        self.array.read_u8(addr % self.size.max(1)).unwrap_or(0xff)
    }

    /// The protected span the status register currently describes, if any.
    ///
    /// The W25Q's status-register memory protection: `BP2..BP0` select a power
    /// of two fraction of the array — one sixty-fourth at `BP = 1`, doubling
    /// to the whole part at `BP = 7` — `TB` puts it at the bottom instead of
    /// the top, `SEC` counts it in 4 KiB sectors instead of 64 KiB blocks, and
    /// `CMP` inverts whatever the other four decided.
    fn protection(&self, state: &State) -> Option<(u64, u64)> {
        let bp = (state.sr1 >> SR1_BP_SHIFT) & SR1_BP_MASK;
        let sec = state.sr1 & SR1_SEC != 0;
        let bottom = state.sr1 & SR1_TB != 0;
        let complement = state.sr2 & SR2_CMP != 0;
        let len = if bp == 0 {
            0
        } else if sec {
            // Sector granularity tops out at a 64 KiB block: BP 5, 6 and 7 all
            // protect the same amount.
            (SECTOR << (bp - 1)).min(BLOCK).min(self.size)
        } else {
            (self.size >> (7 - u32::from(bp))).min(self.size)
        };
        let plain = if len == 0 {
            None
        } else if bottom {
            Some((0, len))
        } else {
            Some((self.size - len, len))
        };
        if !complement {
            return plain;
        }
        // The complement of nothing is everything, and of everything nothing;
        // otherwise it is the rest of the array, which is contiguous because
        // the plain range always touches one end.
        match plain {
            None => Some((0, self.size)),
            Some((_, len)) if len >= self.size => None,
            Some((0, len)) => Some((len, self.size - len)),
            Some((base, _)) => Some((0, base)),
        }
    }

    /// Whether `[base, base + span)` may be programmed or erased.
    fn writable(&self, state: &State, base: u64, span: u64) -> bool {
        match self.protection(state) {
            None => true,
            Some((p_base, p_len)) => base >= p_base + p_len || base + span <= p_base,
        }
    }

    /// Present the byte the part will drive during the next word.
    fn present(&self, state: &mut State) {
        state.out = match state.stream {
            Stream::Array => self.byte(state.addr),
            // `BUSY` never sets: see the module docs.
            Stream::Status(1) => state.sr1 & !SR1_BUSY,
            Stream::Status(2) => state.sr2,
            Stream::Status(3) => state.sr3,
            Stream::Status(_) => IDLE_BYTE,
            Stream::JedecId => self.jedec[(state.count % 3) as usize],
            Stream::DeviceId => {
                if state.count.is_multiple_of(2) {
                    self.jedec[0]
                } else {
                    self.device_id
                }
            }
            Stream::UniqueId => (self.unique_id >> (8 * (7 - (state.count % 8)))) as u8,
            // No SFDP table. A part that has one answers JESD216 DWORDs here,
            // and inventing DWORDs this model has not verified against the
            // standard would be worse than saying there is none: a driver that
            // reads all-ones falls back to the JEDEC id, which this part
            // answers correctly.
            Stream::Sfdp => IDLE_BYTE,
            Stream::JustDeviceId => self.device_id,
            Stream::None | Stream::Program | Stream::StatusIn => IDLE_BYTE,
        };
    }

    /// One word has been exchanged: fold `mosi` into the frame.
    fn step(&self, state: &mut State, mosi: u8) {
        match state.phase {
            Phase::Opcode => self.opcode(state, mosi),
            Phase::Address => {
                state.addr = (state.addr << 8) | u64::from(mosi);
                state.got += 1;
                if state.got >= state.addr_bytes() {
                    self.addressed(state);
                }
            }
            Phase::Dummy => {
                // The first dummy byte of a dual/quad I/O read is the `M7-M0`
                // mode byte. On this generation it is consumed and has no
                // effect — see the module docs for why latching it would be
                // inventing an `FV` behaviour this part does not have.
                let _ = mosi;
                if state.dummy > 0 {
                    state.dummy -= 1;
                }
                if state.dummy == 0 {
                    state.phase = Phase::Data;
                    state.count = 0;
                    self.present(state);
                }
            }
            Phase::Data => match state.stream {
                Stream::Program => {
                    state.page.push(mosi);
                    if !matches!(state.staged, Staged::Program { .. }) {
                        state.staged = Staged::Program {
                            base: state.addr & !(PAGE - 1),
                        };
                    }
                }
                Stream::StatusIn => {
                    let at = usize::from(state.sr_in_len);
                    if at < state.sr_in.len() {
                        state.sr_in[at] = mosi;
                        state.sr_in_len += 1;
                    }
                }
                _ => {
                    state.count = state.count.saturating_add(1);
                    if matches!(state.stream, Stream::Array | Stream::Sfdp) {
                        state.addr = (state.addr + 1) % self.size.max(1);
                    }
                    self.present(state);
                }
            },
            Phase::Ignored => {}
        }
    }

    /// The instruction byte of a fresh frame.
    fn opcode(&self, state: &mut State, cmd: u8) {
        // Deep power-down answers exactly one instruction, which is how
        // firmware that forgot to wake the part sees the fault it caused.
        if state.powered_down && cmd != CMD_RELEASE_POWER_DOWN {
            state.phase = Phase::Ignored;
            return;
        }
        // `99h` resets only when `66h` was the frame before it.
        let armed = state.reset_armed;
        state.reset_armed = false;
        let writes_enabled = state.sr1 & SR1_WEL != 0;
        match cmd {
            CMD_WRITE_ENABLE => {
                state.staged = Staged::WriteEnable;
                state.phase = Phase::Ignored;
            }
            CMD_VOLATILE_SR_WRITE_ENABLE => {
                state.staged = Staged::VolatileWriteEnable;
                state.phase = Phase::Ignored;
            }
            CMD_WRITE_DISABLE => {
                state.staged = Staged::WriteDisable;
                state.phase = Phase::Ignored;
            }
            CMD_READ_STATUS1 | CMD_READ_STATUS2 | CMD_READ_STATUS3 => {
                state.stream = Stream::Status(match cmd {
                    CMD_READ_STATUS1 => 1,
                    CMD_READ_STATUS2 => 2,
                    _ => 3,
                });
                state.phase = Phase::Data;
                state.count = 0;
                self.present(state);
            }
            CMD_WRITE_STATUS1 | CMD_WRITE_STATUS2 | CMD_WRITE_STATUS3 => {
                if !writes_enabled {
                    state.phase = Phase::Ignored;
                    return;
                }
                state.stream = Stream::StatusIn;
                state.phase = Phase::Data;
                state.sr_in_len = 0;
                state.staged = Staged::WriteStatus {
                    first: match cmd {
                        CMD_WRITE_STATUS1 => 1,
                        CMD_WRITE_STATUS2 => 2,
                        _ => 3,
                    },
                };
            }
            CMD_READ => start_read(state, 0),
            CMD_FAST_READ | CMD_FAST_READ_DUAL_OUT => start_read(state, 1),
            CMD_FAST_READ_QUAD_OUT => {
                if state.sr2 & SR2_QE == 0 {
                    state.phase = Phase::Ignored;
                } else {
                    start_read(state, 1);
                }
            }
            // The I/O forms carry a mode byte where the dummy would be, and
            // `EBh` two more bytes of dummy clocks after it. Byte counts, not
            // clock counts: the module docs give the conversion and what this
            // one-line fabric costs.
            CMD_FAST_READ_DUAL_IO => start_read(state, 1),
            CMD_FAST_READ_QUAD_IO => {
                if state.sr2 & SR2_QE == 0 {
                    state.phase = Phase::Ignored;
                } else {
                    start_read(state, 3);
                }
            }
            CMD_PAGE_PROGRAM | CMD_QUAD_PAGE_PROGRAM => {
                if !writes_enabled {
                    state.phase = Phase::Ignored;
                    return;
                }
                if cmd == CMD_QUAD_PAGE_PROGRAM && state.sr2 & SR2_QE == 0 {
                    state.phase = Phase::Ignored;
                    return;
                }
                state.stream = Stream::Program;
                state.phase = Phase::Address;
            }
            CMD_SECTOR_ERASE | CMD_HALF_BLOCK_ERASE | CMD_BLOCK_ERASE => {
                if !writes_enabled {
                    state.phase = Phase::Ignored;
                    return;
                }
                state.stream = Stream::None;
                state.phase = Phase::Address;
                state.span = erase_span(cmd).unwrap_or(SECTOR);
            }
            CMD_CHIP_ERASE | CMD_CHIP_ERASE_ALT => {
                if !writes_enabled {
                    state.phase = Phase::Ignored;
                    return;
                }
                state.staged = Staged::Erase {
                    base: 0,
                    span: self.size,
                };
                state.phase = Phase::Ignored;
            }
            CMD_JEDEC_ID => {
                state.stream = Stream::JedecId;
                state.phase = Phase::Data;
                state.count = 0;
                self.present(state);
            }
            CMD_DEVICE_ID | CMD_DEVICE_ID_DUAL | CMD_DEVICE_ID_QUAD => {
                // Three address bytes even on a four-byte-address part: `90h`
                // takes a fixed 24-bit address, and the datasheet's is 000000h.
                state.stream = Stream::DeviceId;
                state.phase = Phase::Address;
                // Force three, whatever `ADS` says.
                state.got = state.addr_bytes().saturating_sub(3);
            }
            CMD_UNIQUE_ID => {
                state.stream = Stream::UniqueId;
                state.phase = Phase::Dummy;
                state.dummy = 4;
            }
            CMD_READ_SFDP => {
                state.stream = Stream::Sfdp;
                state.phase = Phase::Address;
                state.got = state.addr_bytes().saturating_sub(3);
                state.dummy = 1;
            }
            CMD_POWER_DOWN => {
                state.staged = Staged::PowerDown;
                state.phase = Phase::Ignored;
            }
            CMD_RELEASE_POWER_DOWN => {
                state.staged = Staged::ReleasePowerDown;
                // Three dummy bytes, then the device id — or nothing at all if
                // the master raises CS straight away, which is the ordinary
                // wake-up. Both are the same frame.
                state.stream = Stream::JustDeviceId;
                state.phase = Phase::Dummy;
                state.dummy = 3;
            }
            CMD_ENABLE_RESET => {
                state.staged = Staged::EnableReset;
                state.phase = Phase::Ignored;
            }
            CMD_RESET => {
                state.staged = if armed { Staged::Reset } else { Staged::None };
                state.phase = Phase::Ignored;
            }
            CMD_ENTER_4B | CMD_EXIT_4B => {
                state.staged = Staged::AddressBytes {
                    bytes: if cmd == CMD_ENTER_4B { 4 } else { 3 },
                };
                state.phase = Phase::Ignored;
            }
            _ => state.phase = Phase::Ignored,
        }
    }

    /// The address phase finished.
    fn addressed(&self, state: &mut State) {
        state.addr %= self.size.max(1);
        match state.stream {
            Stream::Array | Stream::Sfdp => {
                if state.dummy > 0 {
                    state.phase = Phase::Dummy;
                } else {
                    state.phase = Phase::Data;
                    state.count = 0;
                    self.present(state);
                }
            }
            Stream::DeviceId => {
                state.phase = Phase::Data;
                state.count = 0;
                self.present(state);
            }
            Stream::Program => {
                state.phase = Phase::Data;
                state.page.reset((state.addr % PAGE) as u8);
            }
            // An erase: the address *is* the whole instruction, so it is
            // staged the moment the last address byte lands — and not before,
            // which is what makes a frame cut short stage nothing.
            _ => {
                if state.span > 0 {
                    state.staged = Staged::Erase {
                        base: state.addr & !(state.span - 1),
                        span: state.span,
                    };
                }
                state.phase = Phase::Ignored;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// the SPI face
// ---------------------------------------------------------------------------

impl SpiSlave for Shared {
    fn format(&self) -> Format {
        self.format
    }

    fn select(&self, selected: bool) {
        let mut state = self.state.lock();
        if selected {
            state.begin_frame();
            self.present(&mut state);
            return;
        }
        // The rising edge. Everything that changes the part happens here, and
        // nowhere else.
        let staged = core::mem::replace(&mut state.staged, Staged::None);
        self.commit(&mut state, staged);
        state.phase = Phase::Opcode;
        state.stream = Stream::None;
        state.out = IDLE_BYTE;
    }

    fn transfer(&self, mosi: u32) -> u32 {
        let mut state = self.state.lock();
        // Full duplex: what goes out is what was already in the shift register
        // when this word began, which is what `present` last put there.
        let presented = state.out;
        self.step(&mut state, mosi as u8);
        u32::from(presented)
    }

    fn peek(&self) -> u32 {
        u32::from(self.state.lock().out)
    }
}

impl Shared {
    /// Apply what the frame staged, at the rising edge of the chip select.
    fn commit(&self, state: &mut State, staged: Staged) {
        match staged {
            Staged::None => {}
            Staged::WriteEnable | Staged::VolatileWriteEnable => state.sr1 |= SR1_WEL,
            Staged::WriteDisable => state.sr1 &= !SR1_WEL,
            Staged::Program { base } => {
                if state.page.latched > 0 && self.writable(state, base, PAGE) {
                    for i in 0..PAGE as usize {
                        if !state.page.touched[i] {
                            continue;
                        }
                        let at = base + i as u64;
                        let Ok(old) = self.array.read_u8(at) else {
                            continue;
                        };
                        // The rule the device exists to enforce: a program
                        // clears bits and never sets one.
                        let _ = self.array.write_u8(at, old & state.page.data[i]);
                    }
                }
                state.sr1 &= !SR1_WEL;
                state.page.reset(0);
            }
            Staged::Erase { base, span } => {
                if self.writable(state, base, span) {
                    let _ = self.array.fill(base, span.min(self.size - base), 0xff);
                }
                state.sr1 &= !SR1_WEL;
            }
            Staged::WriteStatus { first } => {
                // §7.1.1's two locks. `SRP` set with `WP#` low is hardware
                // protection; `SRL` set is the power-supply lock-down, which
                // nothing but a power cycle lifts — and a power cycle is
                // `Device::reset`, which rebuilds this state from scratch.
                // Either way the status register itself refuses to change,
                // which is what makes a locked-down board stay locked down.
                let locked =
                    state.sr2 & SR2_SRL != 0 || (state.sr1 & SR1_SRP != 0 && self.write_protect);
                if locked {
                    state.sr1 &= !SR1_WEL;
                    return;
                }
                for i in 0..usize::from(state.sr_in_len) {
                    let value = state.sr_in[i];
                    match first + i as u8 {
                        1 => state.sr1 = (state.sr1 & !SR1_WRITABLE) | (value & SR1_WRITABLE),
                        2 => state.sr2 = (state.sr2 & !SR2_WRITABLE) | (value & SR2_WRITABLE),
                        3 => state.sr3 = (state.sr3 & !SR3_WRITABLE) | (value & SR3_WRITABLE),
                        _ => {}
                    }
                }
                if self.qe_fixed {
                    state.sr2 |= SR2_QE;
                }
                state.sr1 &= !SR1_WEL;
            }
            Staged::PowerDown => state.powered_down = true,
            Staged::ReleasePowerDown => state.powered_down = false,
            Staged::EnableReset => state.reset_armed = true,
            Staged::Reset => {
                // A software reset returns the *volatile* machinery to its
                // power-on state: it does not un-protect the part, and it
                // certainly does not erase it. `ADS` is a status rather than a
                // setting, so it goes back to whatever `ADP` says the part
                // powers up as — which is exactly what makes `E9h` undone by a
                // reset and `B7h` not survive one.
                let (sr1, sr2, sr3) = (state.sr1 & SR1_WRITABLE, state.sr2, state.sr3);
                *state = State::new(self.addr_bytes, self.qe_fixed);
                state.sr1 = sr1;
                state.sr2 = sr2;
                state.sr3 |= sr3 & SR3_WRITABLE;
            }
            Staged::AddressBytes { bytes } => {
                if bytes == 4 {
                    state.sr3 |= SR3_ADS;
                } else {
                    state.sr3 &= !SR3_ADS;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// the erase granule
// ---------------------------------------------------------------------------

/// Begin an array read: `dummy` bytes between the address and the data.
fn start_read(state: &mut State, dummy: u8) {
    state.stream = Stream::Array;
    state.phase = Phase::Address;
    state.dummy = dummy;
}

/// The span an erase opcode covers, or `None` if it is not an erase.
///
/// The datasheet names "a 24-bit sector address" and "a 24-bit block address"
/// and never says which address bits are ignored; the alignment below is
/// derived from the memory organisation of §1 and §7.1.8 — 4,096 sectors, 256
/// blocks, one flat 24-bit space — so any address inside a granule selects it.
const fn erase_span(cmd: u8) -> Option<u64> {
    match cmd {
        CMD_SECTOR_ERASE => Some(SECTOR),
        CMD_HALF_BLOCK_ERASE => Some(HALF_BLOCK),
        CMD_BLOCK_ERASE => Some(BLOCK),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// A Winbond W25Q-class SPI NOR flash.
#[derive(Debug)]
pub struct SpiNor {
    shared: Arc<Shared>,
    pins: Arc<SlavePins>,
}

impl SpiNor {
    /// Validate `props`, allocate the array, and copy in the initial image.
    ///
    /// Properties:
    ///
    /// * `size` — how many bytes the part holds. Defaults to 16 MiB, a
    ///   W25Q128. Must be a power of two of at least one 64 KiB block, because
    ///   the capacity byte `9Fh` reports is its base-two logarithm.
    /// * `image` — the media slot holding the initial contents.
    /// * `bus`, `cs` — the named SPI bus and the chip select to answer on.
    /// * `mode` — 0 or 3. The part accepts both and the seam names one; see
    ///   the class summary.
    /// * `manufacturer`, `type`, `capacity`, `device` — the identifier bytes,
    ///   for a board modelling a part other than the default.
    /// * `unique-id` — the 64-bit factory serial `4Bh` reports.
    /// * `address-bytes` — 3 or 4. Parts above 128 Mbit power up in either.
    /// * `readonly` — `WP#` tied low.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property, [`Error::Config`] for an
    /// impossible size, an unsupported mode, or a chip select out of range.
    pub fn new(props: &Props) -> Result<SpiNor> {
        let mut r = props.reader();
        let size = r.or_size("size", DEFAULT_SIZE)?;
        let image = r
            .optional_media("image")?
            .map(crate::core::props::Media::to_bytes);
        let bus_name = r.optional_str("bus")?.map(String::from);
        let cs = r.or_range("cs", 0u64, 0..=(MAX_CHIP_SELECTS as u64 - 1))?;
        let mode = r.or_range("mode", 0u64, 0..=3)?;
        let manufacturer = r.or_range("manufacturer", u64::from(WINBOND), 0..=0xff)?;
        let kind = r.or_range("type", u64::from(TYPE_W25Q), 0..=0xff)?;
        let unique_id: u64 = r.or("unique-id", 0)?;
        let addr_bytes = r.or_range("address-bytes", 3u64, 3..=4)?;
        let write_protect = r.or("readonly", false)?;
        // The capacity byte is the density's logarithm, so it falls out of
        // `size` — but a board modelling a part whose byte disagrees with its
        // array can say so.
        let capacity = r.or_range("capacity", log2(size), 0..=0xff)?;
        // `90h`'s device id is one less than the capacity byte across the
        // whole W25Q family: 18h/17h for the 128 Mbit part, 17h/16h for the
        // 64 Mbit one.
        let device_id = r.or_range("device", capacity.saturating_sub(1), 0..=0xff)?;
        r.finish()?;

        if size < BLOCK || !size.is_power_of_two() {
            return Err(config(format!(
                "a serial flash of {size} byte(s): the capacity byte `9Fh` reports is a \
                 base-two logarithm, so the part is a power of two of at least one {BLOCK}-byte \
                 block"
            )));
        }
        if usize::try_from(size).is_err() {
            return Err(config(format!(
                "a flash of {size} byte(s) is larger than this host's address space"
            )));
        }
        if mode != 0 && mode != 3 {
            return Err(config(format!(
                "`mode` is {mode}; a W25Q samples on the rising edge of a clock that idles \
                 either low or high, which is SPI mode 0 or mode 3"
            )));
        }
        if let Some(image) = &image
            && image.len() as u64 > size
        {
            return Err(config(format!(
                "the bound image is {} byte(s) and the flash is {size}",
                image.len()
            )));
        }

        let array = Arc::new(RamStore::new(size));
        // Erased, not zeroed: an unwritten part reads all ones, and firmware
        // that finds zeroes concludes the whole array has been programmed.
        array
            .fill(0, size, 0xff)
            .map_err(|_| config(String::from("the flash array could not be erased")))?;
        if let Some(image) = image {
            array
                .write_at(0, &image)
                .map_err(|_| config(String::from("the flash refused its initial image")))?;
        }

        let addr_bytes = addr_bytes as u8;
        let qe_fixed = kind as u8 == TYPE_W25Q;
        let shared = Arc::new(Shared {
            array,
            size,
            format: Format::new(
                // Mode 0 and mode 3 both sample on the rising edge; they
                // differ only in where SCK rests between frames.
                if mode == 3 { Mode::Mode3 } else { Mode::Mode0 },
                8,
                BitOrder::MsbFirst,
            ),
            jedec: [manufacturer as u8, kind as u8, capacity as u8],
            device_id: device_id as u8,
            unique_id,
            addr_bytes,
            // The memory-type byte is an ordering option, not a density: `40h`
            // is the `-IQ`/`-JQ` part whose `QE` is fixed set, `70h` the
            // `-IM`/`-JM` part whose `QE` is programmable and clear at
            // power-up (§11, Ordering Information).
            qe_fixed,
            write_protect,
            state: Mutex::with_rank(LockRank::DEVICE, State::new(addr_bytes, qe_fixed)),
        });
        let pins = Arc::new(SlavePins::new(Arc::clone(&shared) as Arc<dyn SpiSlave>));
        let part = SpiNor { shared, pins };
        if let Some(name) = bus_name {
            let bus = buses::attach(props, &name)?;
            bus.attach(
                ChipSelect(cs as u8),
                Arc::clone(&part.shared) as Arc<dyn SpiSlave>,
            )?;
        }
        Ok(part)
    }

    /// How many bytes the part holds.
    #[must_use]
    pub fn size(&self) -> u64 {
        self.shared.size
    }

    /// The three bytes `9Fh` answers with.
    #[must_use]
    pub fn jedec_id(&self) -> [u8; 3] {
        self.shared.jedec
    }

    /// The part's wire pins, for a controller that drives them directly.
    #[must_use]
    pub fn pins(&self) -> &Arc<SlavePins> {
        &self.pins
    }

    /// This part as a bus slave, for a test or an embedder that wires its own
    /// [`SpiBus`](crate::bus::spi::SpiBus).
    #[must_use]
    pub fn slave(&self) -> Arc<dyn SpiSlave> {
        Arc::clone(&self.shared) as Arc<dyn SpiSlave>
    }

    /// The contents, for a test, a debugger, or a host that persists them.
    ///
    /// Never has a side effect and never touches the frame state machine —
    /// this is the side door [`crate::core::space::MemAttrs::debug`] would use
    /// if a serial part had an address window to read through.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the range runs off the end of the part.
    pub fn read_contents(&self, offset: u64, dst: &mut [u8]) -> Result<()> {
        self.shared
            .array
            .read_at(offset, dst)
            .map_err(|_| Error::State(format!("{offset:#x} is outside this flash")))
    }

    /// The whole contents as a fresh vector.
    #[must_use]
    pub fn contents(&self) -> Vec<u8> {
        let mut out = alloc::vec![0u8; self.shared.size as usize];
        let _ = self.shared.array.read_at(0, &mut out);
        out
    }

    /// Put `bytes` into the array at `offset`, ignoring flash semantics.
    ///
    /// The *loader's* door, not the guest's: how an initial image gets in, and
    /// deliberately not reachable over SPI, where a write can only clear bits.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the image runs off the end of the part.
    pub fn load_image(&self, offset: u64, bytes: &[u8]) -> Result<()> {
        self.shared.array.write_at(offset, bytes).map_err(|_| {
            config(format!(
                "an image of {} byte(s) at {offset:#x} does not fit in a flash of {}",
                bytes.len(),
                self.shared.size
            ))
        })
    }

    /// Status register 1, 2 or 3, for a test.
    #[must_use]
    pub fn status(&self, which: u8) -> u8 {
        let state = self.shared.state.lock();
        match which {
            1 => state.sr1,
            2 => state.sr2,
            3 => state.sr3,
            _ => 0,
        }
    }
}

/// The base-two logarithm of a power of two, saturating at 63.
fn log2(value: u64) -> u64 {
    u64::from(63 - value.max(1).leading_zeros())
}

impl Device for SpiNor {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: the part is already on its bus and a `wire`
        // statement connects its pins.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds, and the **contents survive both**. Flash is non-volatile:
        // that is the whole reason this device exists rather than a `ram`
        // object on a chip select.
        {
            let mut state = self.shared.state.lock();
            *state = State::new(self.shared.addr_bytes, self.shared.qe_fixed);
        }
        self.pins.reset();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bytes(&self.contents())?;
        let state = self.shared.state.lock();
        w.write_u8(state.phase.tag())?;
        w.write_u8(state.stream.tag())?;
        w.write_u8(match state.stream {
            Stream::Status(n) => n,
            _ => 0,
        })?;
        w.write_u8(state.out)?;
        w.write_u64(state.addr)?;
        w.write_u8(state.got)?;
        w.write_u8(state.dummy)?;
        w.write_u64(state.span)?;
        w.write_u64(state.count)?;
        w.write_u8(state.sr1)?;
        w.write_u8(state.sr2)?;
        w.write_u8(state.sr3)?;
        w.write_bool(state.powered_down)?;
        w.write_bool(state.reset_armed)?;
        // The staged command. A snapshot taken between a page program's last
        // data byte and the rising edge of CS is a snapshot with a program
        // pending, and restoring it as idle would silently swallow the write.
        w.write_u8(state.staged.tag())?;
        let (a, b) = match state.staged {
            Staged::Program { base } => (base, 0),
            Staged::Erase { base, span } => (base, span),
            Staged::WriteStatus { first } => (u64::from(first), 0),
            Staged::AddressBytes { bytes } => (u64::from(bytes), 0),
            _ => (0, 0),
        };
        w.write_u64(a)?;
        w.write_u64(b)?;
        w.write_u8(state.sr_in_len)?;
        for byte in state.sr_in {
            w.write_u8(byte)?;
        }
        w.write_u8(state.page.at)?;
        w.write_u32(state.page.latched)?;
        w.write_bytes(&state.page.data)?;
        let mut touched = alloc::vec![0u8; PAGE as usize / 8];
        for (i, set) in state.page.touched.iter().enumerate() {
            if *set {
                touched[i / 8] |= 1 << (i % 8);
            }
        }
        w.write_bytes(&touched)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bytes: &[u8] = r.read_bytes()?;
        if bytes.len() as u64 != self.shared.size {
            return Err(Error::State(format!(
                "snapshot has {} byte(s) of flash, this part has {}",
                bytes.len(),
                self.shared.size
            )));
        }
        self.shared
            .array
            .write_at(0, bytes)
            .map_err(|_| Error::State(String::from("the flash array refused the snapshot")))?;
        let phase = Phase::from_tag(r.read_u8()?)?;
        let stream_tag = r.read_u8()?;
        let which = r.read_u8()?;
        let stream = Stream::from_tag(stream_tag, which)?;
        let out = r.read_u8()?;
        let addr = r.read_u64()?;
        let got = r.read_u8()?;
        let dummy = r.read_u8()?;
        let span = r.read_u64()?.min(self.shared.size);
        let count = r.read_u64()?;
        let sr1 = r.read_u8()?;
        let sr2 = r.read_u8()?;
        let sr3 = r.read_u8()?;
        let powered_down = r.read_bool()?;
        let reset_armed = r.read_bool()?;
        let staged_tag = r.read_u8()?;
        let a = r.read_u64()?;
        let b = r.read_u64()?;
        let staged = match staged_tag {
            0 => Staged::None,
            1 => Staged::WriteEnable,
            2 => Staged::VolatileWriteEnable,
            3 => Staged::WriteDisable,
            4 => Staged::Program {
                base: a % self.shared.size,
            },
            5 => Staged::Erase {
                base: a % self.shared.size,
                span: b.min(self.shared.size),
            },
            6 => Staged::WriteStatus {
                first: (a as u8).clamp(1, 3),
            },
            7 => Staged::PowerDown,
            8 => Staged::ReleasePowerDown,
            9 => Staged::EnableReset,
            10 => Staged::Reset,
            11 => Staged::AddressBytes {
                bytes: if a == 4 { 4 } else { 3 },
            },
            other => {
                return Err(Error::State(format!(
                    "{other} is not an SPI flash staged command"
                )));
            }
        };
        let sr_in_len = r.read_u8()?.min(3);
        let mut sr_in = [0u8; 3];
        for byte in &mut sr_in {
            *byte = r.read_u8()?;
        }
        let page_at = r.read_u8()?;
        let latched = r.read_u32()?;
        let data: &[u8] = r.read_bytes()?;
        if data.len() != PAGE as usize {
            return Err(Error::State(format!(
                "a page-program latch of {} byte(s), and a page is {PAGE}",
                data.len()
            )));
        }
        let touched_bits: &[u8] = r.read_bytes()?;
        if touched_bits.len() != PAGE as usize / 8 {
            return Err(Error::State(String::from(
                "the page-program latch's coverage map is the wrong length",
            )));
        }
        let mut page = PageLatch::new();
        page.at = page_at;
        page.latched = latched;
        page.data.copy_from_slice(data);
        for i in 0..PAGE as usize {
            page.touched[i] = touched_bits[i / 8] & (1 << (i % 8)) != 0;
        }

        let mut state = self.shared.state.lock();
        *state = State {
            phase,
            stream,
            out,
            addr: addr % self.shared.size,
            got,
            dummy,
            span,
            count,
            sr1,
            sr2,
            sr3,
            powered_down,
            reset_armed,
            staged,
            sr_in,
            sr_in_len,
            page,
        };
        Ok(())
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        let line = match port {
            spi_pin::SCK_NAME => spi_pin::SCK,
            spi_pin::MOSI_NAME => spi_pin::MOSI,
            spi_pin::CS_NAME => spi_pin::CS,
            _ => return None,
        };
        Some(SinkPin {
            sink: self.pins.sink(line),
            line,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != spi_pin::MISO_NAME {
            return Err(Error::Config {
                at: String::from(port),
                message: format!("a serial flash drives only `{}`", spi_pin::MISO_NAME),
            });
        }
        self.pins.connect_miso(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == spi_pin::MISO_NAME {
            self.pins.publish_miso();
        }
    }
}

impl Instance for SpiNor {}

/// The `flash.spinor` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Winbond W25Q-class SPI NOR flash: JEDEC id, fast/dual/quad reads, page program, \
              sector and block erase, the three status registers",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "how many bytes the part holds, a power of two (default 16M, a W25Q128)",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot holding the initial contents; the rest stays erased",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the named SPI bus to attach to, for a transactional link",
        },
        PropertySpec {
            name: "cs",
            kind: ValueKind::Uint,
            required: false,
            summary: "which chip select on that bus (default 0)",
        },
        PropertySpec {
            name: "mode",
            kind: ValueKind::Uint,
            required: false,
            summary: "SPI mode 0 or 3; the part accepts both and the fabric names one",
        },
        PropertySpec {
            name: "manufacturer",
            kind: ValueKind::Uint,
            required: false,
            summary: "the JEP106 manufacturer byte `9Fh` returns (default 0xef, Winbond)",
        },
        PropertySpec {
            name: "type",
            kind: ValueKind::Uint,
            required: false,
            summary: "the memory-type byte `9Fh` returns (default 0x40, W25Q in standard SPI)",
        },
        PropertySpec {
            name: "capacity",
            kind: ValueKind::Uint,
            required: false,
            summary: "the capacity byte `9Fh` returns (default log2 of `size`)",
        },
        PropertySpec {
            name: "device",
            kind: ValueKind::Uint,
            required: false,
            summary: "the device id `90h` and `ABh` return (default one less than `capacity`)",
        },
        PropertySpec {
            name: "unique-id",
            kind: ValueKind::Uint,
            required: false,
            summary: "the 64-bit factory serial `4Bh` reports",
        },
        PropertySpec {
            name: "address-bytes",
            kind: ValueKind::Uint,
            required: false,
            summary: "3 or 4; which mode the part powers up in (default 3)",
        },
        PropertySpec {
            name: "readonly",
            kind: ValueKind::Bool,
            required: false,
            summary: "hold WP# low, so a status register with SRP0 set cannot be changed",
        },
    ],
    construct: |props| Ok(Box::new(SpiNor::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(SpiNor::new(props)?)))
}

/// What the validator should know about `flash.spinor`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size))
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("cs", ValueKind::Uint).range(0, MAX_CHIP_SELECTS as u64 - 1))
        .prop(PropSchema::new("mode", ValueKind::Uint).range(0, 3))
        .prop(PropSchema::new("manufacturer", ValueKind::Uint).range(0, 0xff))
        .prop(PropSchema::new("type", ValueKind::Uint).range(0, 0xff))
        .prop(PropSchema::new("capacity", ValueKind::Uint).range(0, 0xff))
        .prop(PropSchema::new("device", ValueKind::Uint).range(0, 0xff))
        .prop(PropSchema::new("unique-id", ValueKind::Uint))
        .prop(PropSchema::new("address-bytes", ValueKind::Uint).range(3, 4))
        .prop(PropSchema::new("readonly", ValueKind::Bool))
        .port(spi_pin::SCK_NAME, PortDir::In)
        .port(spi_pin::MOSI_NAME, PortDir::In)
        .port(spi_pin::CS_NAME, PortDir::In)
        .port(spi_pin::MISO_NAME, PortDir::Out)
}

#[cfg(test)]
mod tests;
