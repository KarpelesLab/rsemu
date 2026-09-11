//! The STM32 I²C peripheral — **the v2 block**, the one everything since the F0
//! carries.
//!
//! # Which one, and why it is a class of its own
//!
//! [`crate::dev::stm32::i2c`] models **v1** — the F1/F2/F4/L1 part, with `CCR`,
//! `DR` and the `SR1`-then-`SR2` clearing dance — and its header says the v2
//! block "belongs under a name of its own when somebody needs it. It is not a
//! mode of this one." This is that name.
//!
//! | | registers | how a transfer is driven |
//! | --- | --- | --- |
//! | I2C v1 — F1, F2, F4, L1 | `CR1` `CR2` `OAR1` `OAR2` `DR` `SR1` `SR2` `CCR` `TRISE` | event flags cleared by *read-then-do* sequences; software drives every byte |
//! | **I2C v2** — F0, F3, F7, L0, L4, L4+, L5, G0, G4, H7, U5, WB | `CR1` `CR2` `OAR1` `OAR2` `TIMINGR` `TIMEOUTR` `ISR` `ICR` `PECR` `RXDR` `TXDR` | `CR2` carries `NBYTES`, `RELOAD` and `AUTOEND`; the hardware runs the transfer |
//!
//! They are not a superset of one another in either direction, and a driver for
//! one does not resemble a driver for the other. Three differences do the work:
//!
//! * **The transfer is counted.** `CR2.NBYTES` says how many bytes this leg of
//!   the transfer is, and the hardware stops there by itself — with a STOP if
//!   `AUTOEND` is set, with `TCR` and a stretched clock if `RELOAD` is set, and
//!   with `TC` and a stretched clock if neither is. v1's software drives every
//!   byte and decides when to stop; v2's software programs a length.
//! * **Flags are cleared by writing `ICR`**, not by reading one register and
//!   then another. That makes this block far *less* hazardous for a debugger
//!   than v1 — but not free of hazard, because reading `RXDR` still pops the
//!   receive register, so [`MemAttrs::debug`](crate::core::space::MemAttrs::debug)
//!   still has to be honoured and a debug dump still has to change nothing.
//! * **`TIMINGR` sets the timing**, as two explicit half periods plus a
//!   prescaler, rather than `CCR` plus a `FREQ` field the software has to keep
//!   truthful.
//!
//! # Source
//!
//! ST **RM0351**, *STM32L4x5 and STM32L4x6 advanced Arm-based 32-bit MCUs*,
//! **Rev. 9**, chapter **39**, *Inter-integrated circuit (I2C) interface*. The
//! block is identical across the families that carry it, but the chapter is
//! numbered differently in each manual, so the manual and its revision are part
//! of the citation. Section numbers below (§39.4.7, §39.7.2 …) are RM0351
//! Rev. 9's.
//!
//! The bus itself is NXP **UM10204**; see [`crate::bus::i2c`]. No emulator was
//! consulted (`ROADMAP.md` §1); the Linux `i2c-stm32f7` driver is GPLv2 and was
//! not opened.
//!
//! # What is modelled
//!
//! **Master mode, completely**, in both link models:
//!
//! * The `NBYTES`/`RELOAD`/`AUTOEND` state machine of §39.4.7 — `TXIS` per byte
//!   out, `RXNE` per byte in, `TC` when a leg ends with software in charge,
//!   `TCR` when it ends with a reload pending, and the automatic STOP
//!   `AUTOEND` asks for.
//! * 7-bit and 10-bit master addressing including `HEAD10R`: the two-byte
//!   address, and the repeated START with `1111 0XX1` that turns a 10-bit write
//!   into a 10-bit read — or, with `HEAD10R` set, the short read header on its
//!   own.
//! * `NACKF` on an address or a byte nobody answered, **with the STOP the
//!   hardware generates by itself** (§39.4.8), and `STOPF` after it.
//! * `TIMINGR` as the actual SCL timing (§39.4.5), so a transfer costs the
//!   virtual time the guest's timing configuration asks for — and costs the
//!   same in both link models, because [`crate::bus::i2c`] fixes the
//!   half-period count of every bus event and only the controller decides what
//!   a half period lasts.
//! * SMBus **PEC** (§39.4.13): `PECEN`, `CR2.PECBYTE`, the CRC-8 itself, `PECR`
//!   and `PECERR`.
//! * The `I2Cx_EV` and `I2Cx_ER` interrupt outputs, gated by the six event
//!   enables and `ERRIE` exactly as §39.7.1 lists them, and the two DMA request
//!   outputs `TXDMAEN`/`RXDMAEN` raise, in the level style
//!   [`crate::dev::stm32::dma`] asks a peripheral for.
//!
//! **Slave mode, on the transactional link**: `OAR1` with its 7- or 10-bit own
//! address, `OAR2` with `OA2MSK`, the general call under `GCEN`, `ADDR` with
//! `DIR` and `ADDCODE`, `RXNE`/`TXIS` for the data phase, `NOSTRETCH` and the
//! `OVR` it makes possible, and software `NACK`.
//!
//! # What is not
//!
//! * **Slave mode on the wired link.** The reason is the structural one v1's
//!   header sets out and it has not changed: on real silicon one pair of pins
//!   carries both roles, so a wired slave needs a single bit engine that drives
//!   *and* listens on the same [`OpenDrain`](crate::bus::i2c::wires::OpenDrain)
//!   pair, and [`crate::bus::i2c::wires`] has the master engine and the slave
//!   engine separately. Bolting them together here would give a wired slave
//!   that behaves differently from the transactional one, which is the exact
//!   failure the bus was written to avoid. It is a day's work in `bus::i2c`,
//!   not a paragraph here — and until it is done, a machine that puts this
//!   controller on a `wired` link gets a master and nothing else.
//! * **`TIMEOUTR`.** `TIMEOUTA`, `TIMEOUTB`, `TIDLE`, `TIMOUTEN` and `TEXTEN`
//!   are storage. The SMBus timeouts measure how long SCL has been low and how
//!   long the bus has been idle, and this model has no idle to measure: it
//!   burns half periods only while an operation is in flight. `TIMEOUT` and
//!   `ALERT` are therefore never set by hardware here, only cleared by `ICR`.
//! * **The noise filters.** `DNF` and `ANFOFF` are storage: this model clocks
//!   in half periods and has no spikes to suppress.
//! * **`SBC`.** Slave byte control is stored and read back. It gates a slave's
//!   per-byte acknowledge through `NBYTES`/`RELOAD`, which needs the slave side
//!   to run the same counting machine the master side does; a slave that
//!   acknowledges whenever it has room is what this model does instead.
//! * **`WUPEN`.** There is no stop mode to wake from.
//!
//! Everything in that list reads back what was written, so a driver that
//! programs it and then checks does not see a wrong answer — it sees no effect.
//!
//! # Time
//!
//! **The scheduler owns it** (`CLAUDE.md`). A *lazily advanced* device
//! (`ROADMAP.md` §4.2) on the peripheral clock: it holds its own tick,
//! publishes the tick of its next SCL edge, and is caught up before any
//! register access.
//!
//! §39.4.5 gives the timings as `t_PRESC = (PRESC+1) × t_I2CCLK`,
//! `t_SCLL = (SCLL+1) × t_PRESC` and `t_SCLH = (SCLH+1) × t_PRESC`, which is
//! exactly the `(Tlow, Thigh)` pair [`crate::bus::i2c`]'s cost model wants: a
//! START is [`START_HALF_PERIODS`] half periods, a byte is
//! [`BYTE_HALF_PERIODS`], a STOP is [`STOP_HALF_PERIODS`], and the two halves
//! alternate. `SDADEL` and `SCLDEL` place the data edge *within* a bit rather
//! than changing the bit period, so they are storage — a model that clocks in
//! half periods has no sub-bit position for them to move.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::bus::i2c::wires::{MasterEvent, MasterOp, MasterWires, MasterWiresState, pin as line};
use crate::bus::i2c::{
    Ack, Address, BYTE_HALF_PERIODS, Direction, GENERAL_CALL, I2cBus, I2cSlave, Link,
    START_HALF_PERIODS, STOP_HALF_PERIODS, buses,
};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{AccessKind, LazyHandle};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSource};
use crate::machine::realize::Instance;

#[cfg(all(test, feature = "dev-at24c"))]
mod tests;

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.i2c-v2";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes of address space the register block occupies.
///
/// `TXDR` is the last register, at `0x28` (§39.7.12), so the file is `0x2c`
/// bytes. A peripheral aperture on these families is 0x400 bytes; a machine
/// file `mirror()`s the region across whatever window the SoC decodes.
pub const REGISTER_BYTES: u64 = 0x2c;

// ---------------------------------------------------------------------------
// CR1 (§39.7.1)
// ---------------------------------------------------------------------------

/// `CR1` bit 0: peripheral enable.
const CR1_PE: u32 = 1 << 0;
/// `CR1` bit 1: `TXIE`, `TXIS` raises the event interrupt.
const CR1_TXIE: u32 = 1 << 1;
/// `CR1` bit 2: `RXIE`.
const CR1_RXIE: u32 = 1 << 2;
/// `CR1` bit 3: `ADDRIE`.
const CR1_ADDRIE: u32 = 1 << 3;
/// `CR1` bit 4: `NACKIE`.
const CR1_NACKIE: u32 = 1 << 4;
/// `CR1` bit 5: `STOPIE`.
const CR1_STOPIE: u32 = 1 << 5;
/// `CR1` bit 6: `TCIE` — covers both `TC` and `TCR`.
const CR1_TCIE: u32 = 1 << 6;
/// `CR1` bit 7: `ERRIE`.
const CR1_ERRIE: u32 = 1 << 7;
/// `CR1` bit 14: `TXDMAEN`.
const CR1_TXDMAEN: u32 = 1 << 14;
/// `CR1` bit 15: `RXDMAEN`.
const CR1_RXDMAEN: u32 = 1 << 15;
/// `CR1` bit 17: `NOSTRETCH`, slave mode only.
const CR1_NOSTRETCH: u32 = 1 << 17;
/// `CR1` bit 19: `GCEN`, answer the general call.
const CR1_GCEN: u32 = 1 << 19;
/// `CR1` bit 23: `PECEN`.
const CR1_PECEN: u32 = 1 << 23;
/// Everything `CR1` defines: bits 0–12, 14–23. Bit 13 and 24–31 are reserved.
const CR1_MASK: u32 = 0x00ff_dfff;

// ---------------------------------------------------------------------------
// CR2 (§39.7.2)
// ---------------------------------------------------------------------------

/// `CR2` bits 9:0: the slave address. `SADD[7:1]` in 7-bit mode, all ten bits
/// in 10-bit mode.
const CR2_SADD: u32 = 0x3ff;
/// `CR2` bit 10: `RD_WRN`, the direction of the transfer this address opens.
const CR2_RD_WRN: u32 = 1 << 10;
/// `CR2` bit 11: `ADD10`, the address is ten bits.
const CR2_ADD10: u32 = 1 << 11;
/// `CR2` bit 12: `HEAD10R` — a 10-bit read sends only the read header.
const CR2_HEAD10R: u32 = 1 << 12;
/// `CR2` bit 13: `START`.
const CR2_START: u32 = 1 << 13;
/// `CR2` bit 14: `STOP`.
const CR2_STOP: u32 = 1 << 14;
/// `CR2` bit 15: `NACK`, slave mode.
const CR2_NACK: u32 = 1 << 15;
/// `CR2` bits 23:16: how many bytes this leg of the transfer is.
const CR2_NBYTES_SHIFT: u32 = 16;
/// `CR2` bit 24: `RELOAD`.
const CR2_RELOAD: u32 = 1 << 24;
/// `CR2` bit 25: `AUTOEND`.
const CR2_AUTOEND: u32 = 1 << 25;
/// `CR2` bit 26: `PECBYTE`.
const CR2_PECBYTE: u32 = 1 << 26;
/// Everything `CR2` defines: bits 0–26.
const CR2_MASK: u32 = 0x07ff_ffff;

/// The `NBYTES` field of a `CR2` value.
const fn nbytes(cr2: u32) -> u8 {
    ((cr2 >> CR2_NBYTES_SHIFT) & 0xff) as u8
}

// ---------------------------------------------------------------------------
// OAR1, OAR2, TIMINGR, TIMEOUTR (§39.7.3 – §39.7.6)
// ---------------------------------------------------------------------------

/// `OAR1` bit 10: `OA1MODE`, the own address is ten bits.
const OAR1_OA1MODE: u32 = 1 << 10;
/// `OAR1` bit 15: `OA1EN`.
const OAR1_OA1EN: u32 = 1 << 15;
/// Everything `OAR1` defines.
const OAR1_MASK: u32 = OAR1_OA1EN | OAR1_OA1MODE | 0x3ff;
/// `OAR2` bit 15: `OA2EN`.
const OAR2_OA2EN: u32 = 1 << 15;
/// Everything `OAR2` defines: `OA2[7:1]`, `OA2MSK[10:8]`, `OA2EN`.
const OAR2_MASK: u32 = OAR2_OA2EN | (0x7 << 8) | (0x7f << 1);

/// Everything `TIMINGR` defines (§39.7.5): `SCLL`, `SCLH`, `SDADEL`, `SCLDEL`,
/// `PRESC`.
const TIMINGR_MASK: u32 = 0xf0ff_ffff;
/// Everything `TIMEOUTR` defines (§39.7.6). Storage.
const TIMEOUTR_MASK: u32 = 0x8fff_9fff;

// ---------------------------------------------------------------------------
// ISR and ICR (§39.7.7, §39.7.8)
// ---------------------------------------------------------------------------

/// `ISR` bit 0: `TXE`, the transmit register is empty. **Reset value 1.**
const ISR_TXE: u32 = 1 << 0;
/// `ISR` bit 1: `TXIS`, the hardware wants the next byte.
const ISR_TXIS: u32 = 1 << 1;
/// `ISR` bit 2: `RXNE`, the receive register holds a byte.
const ISR_RXNE: u32 = 1 << 2;
/// `ISR` bit 3: `ADDR`, a slave address matched. Slave mode.
const ISR_ADDR: u32 = 1 << 3;
/// `ISR` bit 4: `NACKF`, a not-acknowledge was received.
const ISR_NACKF: u32 = 1 << 4;
/// `ISR` bit 5: `STOPF`, a STOP was detected.
const ISR_STOPF: u32 = 1 << 5;
/// `ISR` bit 6: `TC`, the transfer is complete and software is in charge.
const ISR_TC: u32 = 1 << 6;
/// `ISR` bit 7: `TCR`, the transfer is complete and `NBYTES` must be reloaded.
const ISR_TCR: u32 = 1 << 7;
/// `ISR` bit 8: `BERR`.
const ISR_BERR: u32 = 1 << 8;
/// `ISR` bit 9: `ARLO`.
const ISR_ARLO: u32 = 1 << 9;
/// `ISR` bit 10: `OVR`, slave overrun or underrun under `NOSTRETCH`.
const ISR_OVR: u32 = 1 << 10;
/// `ISR` bit 11: `PECERR`.
const ISR_PECERR: u32 = 1 << 11;
/// `ISR` bit 12: `TIMEOUT`. Never set here; see the module docs.
const ISR_TIMEOUT: u32 = 1 << 12;
/// `ISR` bit 13: `ALERT`. Never set here; see the module docs.
const ISR_ALERT: u32 = 1 << 13;
/// `ISR` bit 15: `BUSY`. Not latched — it is the bus's own state.
const ISR_BUSY: u32 = 1 << 15;
/// `ISR` bit 16: `DIR`, the direction the matched address asked for.
const ISR_DIR: u32 = 1 << 16;
/// `ISR` bits 23:17: `ADDCODE`, the address code that matched.
const ISR_ADDCODE_SHIFT: u32 = 17;
/// The `ADDCODE` field in place.
const ISR_ADDCODE_MASK: u32 = 0x7f << ISR_ADDCODE_SHIFT;

/// The flags [`CR1_ERRIE`] gates onto `I2Cx_ER` (§39.7.1).
const ISR_ERRORS: u32 = ISR_BERR | ISR_ARLO | ISR_OVR | ISR_PECERR | ISR_TIMEOUT | ISR_ALERT;

/// Everything `ICR` clears (§39.7.8): `ADDRCF`, `NACKCF`, `STOPCF`, `BERRCF`,
/// `ARLOCF`, `OVRCF`, `PECCF`, `TIMOUTCF`, `ALERTCF`.
///
/// **`TC` and `TCR` are not in it**, and that is this register's sharpest edge:
/// they are cleared by writing `CR2`, never by `ICR`.
const ICR_MASK: u32 = ISR_ADDR
    | ISR_NACKF
    | ISR_STOPF
    | ISR_BERR
    | ISR_ARLO
    | ISR_OVR
    | ISR_PECERR
    | ISR_TIMEOUT
    | ISR_ALERT;

/// What `ISR` holds after a `PE = 0` software reset (§39.4.1): `TXE` alone.
const ISR_RESET: u32 = ISR_TXE;

/// The pin names a machine description wires.
pub mod pin {
    /// The `I2Cx_EV` interrupt output, level driven.
    pub const EV: &str = "ev";
    /// The `I2Cx_ER` interrupt output, level driven.
    pub const ER: &str = "er";
    /// The transmit DMA request, raised while `TXDMAEN` and `TXIS` are both
    /// set. A level, in the style [`crate::dev::stm32::dma`] asks for.
    pub const TX_DRQ: &str = "tx-drq";
    /// The receive DMA request, raised while `RXDMAEN` and `RXNE` are both set.
    pub const RX_DRQ: &str = "rx-drq";
}

/// "Nothing scheduled".
const NO_EVENT: u64 = u64::MAX;

/// The rank this controller's own state takes.
///
/// The same rank the v1 block takes, and for the same reasons: above
/// [`crate::bus::i2c::WIRES_RANK`] because the controller calls *into* its bit
/// engine, and never held across a call into the engine, the fabric or a
/// sibling. Two controllers on one bus therefore never hold each other's.
const STATE_RANK: LockRank = LockRank::new(0x4700);

/// The SMBus PEC polynomial, `x^8 + x^2 + x + 1` (§39.4.13).
const PEC_POLY: u8 = 0x07;

/// One byte of the SMBus packet error check.
const fn crc8(mut crc: u8, byte: u8) -> u8 {
    crc ^= byte;
    let mut i = 0;
    while i < 8 {
        crc = if crc & 0x80 != 0 {
            (crc << 1) ^ PEC_POLY
        } else {
            crc << 1
        };
        i += 1;
    }
    crc
}

// ---------------------------------------------------------------------------
// The engine's position
// ---------------------------------------------------------------------------

/// What the master engine is doing, or is about to do.
///
/// Unlike v1's, these are **not** "waiting for software to run a clearing
/// sequence" states: v2 clears flags through `ICR` and the hardware never needs
/// software's permission to proceed except where the byte counter says so. A
/// stage names the bus operation in flight, or — while [`State::op`] is `None`
/// — the one the engine will submit next.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Stage {
    /// Not a master. `PE` may be clear, or the controller may simply be idle.
    #[default]
    Idle,
    /// A START, opening a new transfer.
    Starting,
    /// The one address byte of a 7-bit transfer.
    AddrSeven,
    /// The `1111 0XX0` header of a 10-bit address (UM10204 §3.1.11).
    AddrTenHeadW,
    /// The second byte of a 10-bit address.
    AddrTenLow,
    /// The repeated START that turns a 10-bit write into a 10-bit read.
    Restarting,
    /// The `1111 0XX1` read header of a 10-bit address.
    AddrTenHeadR,
    /// Master transmitter, in the data phase.
    Tx,
    /// Master receiver, in the data phase.
    Rx,
    /// A STOP.
    Stopping,
    /// The master owns the bus with nothing in flight: `TC` is set, or a NACK
    /// has been taken and the automatic STOP has not gone out yet. SCL is held
    /// low.
    Held,
}

/// A stable code for a stage, for the snapshot.
const fn stage_code(stage: Stage) -> u8 {
    match stage {
        Stage::Idle => 0,
        Stage::Starting => 1,
        Stage::AddrSeven => 2,
        Stage::AddrTenHeadW => 3,
        Stage::AddrTenLow => 4,
        Stage::Restarting => 5,
        Stage::AddrTenHeadR => 6,
        Stage::Tx => 7,
        Stage::Rx => 8,
        Stage::Stopping => 9,
        Stage::Held => 10,
    }
}

/// The inverse. An unknown code loads as idle rather than panicking: a snapshot
/// is untrusted input (`ROADMAP.md` §4.5).
const fn stage_from_code(code: u8) -> Stage {
    match code {
        1 => Stage::Starting,
        2 => Stage::AddrSeven,
        3 => Stage::AddrTenHeadW,
        4 => Stage::AddrTenLow,
        5 => Stage::Restarting,
        6 => Stage::AddrTenHeadR,
        7 => Stage::Tx,
        8 => Stage::Rx,
        9 => Stage::Stopping,
        10 => Stage::Held,
        _ => Stage::Idle,
    }
}

/// A bus event in flight, as this controller records it.
///
/// The same four cases as [`MasterOp`], kept separately because the
/// transactional link has no [`MasterWires`] to hold them and a snapshot has to
/// carry them either way.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Op {
    Start,
    Write(u8),
    Read(Ack),
    Stop,
}

impl Op {
    /// The half periods this event costs, taken from [`crate::bus::i2c`] so
    /// both link models charge the scheduler the same.
    const fn halves(self) -> u32 {
        match self {
            Op::Start => START_HALF_PERIODS,
            Op::Write(_) | Op::Read(_) => BYTE_HALF_PERIODS,
            Op::Stop => STOP_HALF_PERIODS,
        }
    }

    /// The wire engine's spelling of the same thing.
    const fn to_wire(self) -> MasterOp {
        match self {
            Op::Start => MasterOp::Start,
            Op::Write(b) => MasterOp::Write(b),
            Op::Read(a) => MasterOp::Read(a),
            Op::Stop => MasterOp::Stop,
        }
    }

    /// A stable code and operand, for the snapshot.
    const fn code(self) -> (u8, u8) {
        match self {
            Op::Start => (1, 0),
            Op::Write(b) => (2, b),
            Op::Read(a) => (3, if a.is_ack() { 1 } else { 0 }),
            Op::Stop => (4, 0),
        }
    }

    /// The inverse.
    const fn from_code(code: u8, operand: u8) -> Option<Op> {
        match code {
            1 => Some(Op::Start),
            2 => Some(Op::Write(operand)),
            3 => Some(Op::Read(if operand != 0 { Ack::Ack } else { Ack::Nack })),
            4 => Some(Op::Stop),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// A memory-mapped STM32 I²C peripheral, `I2C v2`.
#[derive(Debug)]
pub struct Stm32I2cV2 {
    shared: Arc<Shared>,
    region: RegionRef,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    /// How bytes reach the slaves. Fixed at construction and written down in
    /// the machine file, which is the whole point (`docs/buses/low-speed.md`).
    link: Link,
    /// The bus this controller drives in [`Link::Transactional`] mode, and the
    /// one its slave face hangs off.
    bus: Option<Arc<I2cBus>>,
    /// The bit engine it drives in [`Link::Wired`] mode.
    wires: Arc<MasterWires>,
    /// Domain ticks simulated, published for the scheduler's lock-free
    /// question. Mirrors `State::ticks`.
    ticks: AtomicU64,
    /// The tick of the next SCL half-period boundary, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The four outputs, connected at realize time.
    ev: Mutex<Option<WireSource>>,
    er: Mutex<Option<WireSource>>,
    tx_drq: Mutex<Option<WireSource>>,
    rx_drq: Mutex<Option<WireSource>>,
    /// The levels they are held at, so a debug read is free and the realize
    /// sweep has an answer.
    ev_level: AtomicBool,
    er_level: AtomicBool,
    tx_drq_level: AtomicBool,
    rx_drq_level: AtomicBool,
    /// The catch-up handle the register block syncs through.
    lazy: Mutex<Option<LazyHandle>>,
}

/// Everything the guest can see or change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    /// Domain ticks simulated. The authoritative copy; the atomic mirrors it.
    ticks: u64,
    cr1: u32,
    cr2: u32,
    oar1: u32,
    oar2: u32,
    timingr: u32,
    timeoutr: u32,
    /// The latched `ISR` flags, `DIR` and `ADDCODE` included. `BUSY` is not
    /// here: it is the bus's state, read at the moment of the access.
    isr: u32,
    /// `RXDR`.
    rxdr: u8,
    /// `TXDR`. `ISR.TXE` is the authority on whether it holds a byte.
    txdr: u8,
    /// The byte the engine is putting on the wire.
    ///
    /// Real state rather than a cache: once a byte leaves `TXDR` for the shift
    /// register, software may write `TXDR` again, and the packet error check
    /// has to cover what actually went out. The two links disagree about
    /// whether [`State::op`] still holds it by the time the event is applied —
    /// the transactional one consumes the op to perform it — so neither may be
    /// asked, and this field is the one answer both share.
    shift_out: u8,
    /// The running packet error check (§39.4.13), which is what `PECR` reads.
    pec: u8,
    /// **The byte counter.** Loaded from `CR2.NBYTES` at each START and at each
    /// reload, decremented per byte, and the thing the whole v2 block is built
    /// around — §39.4.7.
    count: u8,
    /// Where the master engine is.
    stage: Stage,
    /// Which way the master transfer in progress runs, latched at the START.
    dir: Direction,
    /// A STOP the hardware owes the bus: `AUTOEND`, or the one §39.4.8 makes it
    /// generate after a NACK.
    auto_stop: bool,
    /// Whether the slave face is in a transaction.
    slave_active: bool,
    /// The bus event in flight, mirrored here in both link models so the engine
    /// loop, the snapshot and `next_event_tick` ask one question.
    op: Option<Op>,
    /// [`Link::Transactional`] only: half periods still owed before the event
    /// takes effect.
    halves_left: u32,
    /// The tick the next half-period boundary falls on.
    next_edge: u64,
    /// Whether the half period about to elapse is the high one. It alternates,
    /// so one bit costs `t_SCLL + t_SCLH` however `TIMINGR` splits the two.
    high_half: bool,
}

impl Default for State {
    fn default() -> State {
        State {
            ticks: 0,
            cr1: 0,
            cr2: 0,
            oar1: 0,
            oar2: 0,
            timingr: 0,
            timeoutr: 0,
            // §39.7.7: every flag resets to zero except `TXE`, which is 1.
            isr: ISR_RESET,
            rxdr: 0,
            txdr: 0,
            shift_out: 0,
            pec: 0,
            count: 0,
            stage: Stage::Idle,
            dir: Direction::Write,
            auto_stop: false,
            slave_active: false,
            op: None,
            halves_left: 0,
            next_edge: 0,
            high_half: false,
        }
    }
}

impl State {
    /// Whether the peripheral is enabled.
    const fn enabled(&self) -> bool {
        self.cr1 & CR1_PE != 0
    }

    /// `t_SCLL` and `t_SCLH` in ticks of the peripheral clock (§39.4.5).
    ///
    /// `t_PRESC = (PRESC+1) × t_I2CCLK`, `t_SCLL = (SCLL+1) × t_PRESC`,
    /// `t_SCLH = (SCLH+1) × t_PRESC`. Every field is `+1`, so neither half can
    /// be zero — a zero-length half period would put the next event on the tick
    /// it was scheduled from, which the scheduler forbids, and `TIMINGR`'s
    /// encoding rules that out for free where `CCR`'s did not.
    const fn scl(&self) -> (u64, u64) {
        let presc = ((self.timingr >> 28) & 0xf) as u64 + 1;
        let scll = (self.timingr & 0xff) as u64 + 1;
        let sclh = ((self.timingr >> 8) & 0xff) as u64 + 1;
        (presc * scll, presc * sclh)
    }

    /// How long the half period about to elapse lasts.
    const fn half_len(&self) -> u64 {
        let (low, high) = self.scl();
        if self.high_half { high } else { low }
    }

    /// `ISR` as software reads it. `busy` comes from the bus, which is not this
    /// device's state.
    const fn read_isr(&self, busy: bool) -> u32 {
        if busy { self.isr | ISR_BUSY } else { self.isr }
    }

    /// Whether any of some `ISR` bits are set.
    const fn any(&self, bits: u32) -> bool {
        self.isr & bits != 0
    }

    /// Set some `ISR` bits.
    const fn set(&mut self, bits: u32) {
        self.isr |= bits;
    }

    /// Clear some `ISR` bits.
    const fn clear(&mut self, bits: u32) {
        self.isr &= !bits;
    }

    /// Whether this transfer carries a packet error check byte (§39.4.13).
    ///
    /// `PECBYTE` is only meaningful with `RELOAD` clear — the PEC is the last
    /// of `NBYTES`, and a reload means `NBYTES` is not the end.
    const fn pec_transfer(&self) -> bool {
        self.cr1 & CR1_PECEN != 0 && self.cr2 & CR2_PECBYTE != 0 && self.cr2 & CR2_RELOAD == 0
    }

    /// Whether the byte about to move is the PEC byte itself: the last of
    /// `NBYTES`.
    const fn pec_due(&self) -> bool {
        self.pec_transfer() && self.count == 1
    }

    /// The acknowledge a master receiver drives on the byte it is about to
    /// clock in.
    ///
    /// §39.4.7: the master NACKs the last byte of `NBYTES` — that is how the
    /// slave is told to stop transmitting (UM10204 §3.1.6) — **unless**
    /// `RELOAD` is set, in which case the transfer continues and the byte is
    /// acknowledged like any other.
    const fn rx_ack(&self) -> Ack {
        if self.count == 1 && self.cr2 & CR2_RELOAD == 0 {
            Ack::Nack
        } else {
            Ack::Ack
        }
    }

    /// The address byte the current address stage puts on the wire.
    const fn address_byte(&self) -> u8 {
        let sadd = self.cr2 & CR2_SADD;
        let rw = if matches!(self.dir, Direction::Read) {
            1
        } else {
            0
        };
        match self.stage {
            // §39.7.2: in 7-bit mode the address is `SADD[7:1]`; `SADD[0]` is
            // don't care and the direction bit takes its place on the wire.
            Stage::AddrSeven => ((sadd as u8) & 0xfe) | rw,
            // UM10204 §3.1.11's `1111 0XX`, carrying `SADD[9:8]`.
            Stage::AddrTenHeadW => 0xf0 | (((sadd >> 8) as u8 & 0x3) << 1),
            Stage::AddrTenHeadR => 0xf0 | (((sadd >> 8) as u8 & 0x3) << 1) | 1,
            Stage::AddrTenLow => sadd as u8,
            _ => 0,
        }
    }

    /// The ten-bit address `CR2` is pointing at.
    const fn ten(&self) -> u16 {
        (self.cr2 & CR2_SADD) as u16
    }

    /// Whether the peripheral is holding SCL low waiting for software.
    ///
    /// In master mode that is the byte counter's doing: `TXIS` unserved, `RXNE`
    /// unread, or `TC`/`TCR` set. In slave mode it is `ADDR` before `ADDRCF`,
    /// or an unserved data register, and `NOSTRETCH` turns it off (§39.4.9).
    const fn stretching(&self) -> bool {
        if self.slave_active {
            return self.cr1 & CR1_NOSTRETCH == 0
                && (self.any(ISR_ADDR | ISR_RXNE) || (self.any(ISR_DIR) && self.any(ISR_TXE)));
        }
        match self.stage {
            Stage::Tx => self.any(ISR_TXIS | ISR_TC | ISR_TCR),
            Stage::Rx => self.any(ISR_RXNE | ISR_TC | ISR_TCR),
            Stage::Held => self.any(ISR_TC | ISR_TCR),
            _ => false,
        }
    }

    /// The level `I2Cx_EV` should be at (§39.7.1).
    const fn ev(&self) -> bool {
        (self.cr1 & CR1_TXIE != 0 && self.any(ISR_TXIS))
            || (self.cr1 & CR1_RXIE != 0 && self.any(ISR_RXNE))
            || (self.cr1 & CR1_ADDRIE != 0 && self.any(ISR_ADDR))
            || (self.cr1 & CR1_NACKIE != 0 && self.any(ISR_NACKF))
            || (self.cr1 & CR1_STOPIE != 0 && self.any(ISR_STOPF))
            || (self.cr1 & CR1_TCIE != 0 && self.any(ISR_TC | ISR_TCR))
    }

    /// The level `I2Cx_ER` should be at (§39.7.1).
    const fn er(&self) -> bool {
        self.cr1 & CR1_ERRIE != 0 && self.any(ISR_ERRORS)
    }

    /// The level the transmit DMA request should be at (§39.4.11).
    const fn tx_drq(&self) -> bool {
        self.cr1 & CR1_TXDMAEN != 0 && self.any(ISR_TXIS)
    }

    /// The level the receive DMA request should be at.
    const fn rx_drq(&self) -> bool {
        self.cr1 & CR1_RXDMAEN != 0 && self.any(ISR_RXNE)
    }

    /// Which of this peripheral's own addresses `address` matches, and the
    /// `ADDCODE` it reports (§39.7.3, §39.7.4).
    ///
    /// `OAR1` first, then `OAR2` with its mask, then the general call.
    /// `OA2MSK` masks the *low* bits of `OA2[7:1]`: 0 compares all seven, 7
    /// compares none and so answers every address — "except those reserved",
    /// which §39.7.4 says explicitly and which [`Address::is_reserved`] knows.
    fn own_address(&self, address: Address) -> Option<u8> {
        if self.oar1 & OAR1_OA1EN != 0 {
            let ten = self.oar1 & OAR1_OA1MODE != 0;
            match address {
                Address::Seven(a) if !ten && u32::from(a) == (self.oar1 >> 1) & 0x7f => {
                    return Some(a);
                }
                Address::Ten(a) if ten && u32::from(a) == self.oar1 & 0x3ff => {
                    // §39.7.7: for a 10-bit address `ADDCODE` is the header
                    // plus the two most significant address bits.
                    return Some(0x78 | ((a >> 8) as u8 & 0x3));
                }
                _ => {}
            }
        }
        if self.oar2 & OAR2_OA2EN != 0
            && let Address::Seven(a) = address
        {
            let mask = (self.oar2 >> 8) & 0x7;
            let own = ((self.oar2 >> 1) & 0x7f) as u8;
            if (a >> mask) == (own >> mask) && !address.is_reserved() {
                return Some(a);
            }
        }
        if self.cr1 & CR1_GCEN != 0 && address == GENERAL_CALL {
            return Some(0);
        }
        None
    }
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Stm32I2cV2Shared");
        s.field("link", &self.link);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Stm32I2cV2 {
    /// Validate `props` and build the peripheral.
    ///
    /// Properties:
    ///
    /// * `link` — `"transactional"` or `"wired"`. **Required**, and
    ///   deliberately so: `docs/buses/low-speed.md` asks for this choice to be
    ///   made rather than defaulted into.
    /// * `bus` — the name of the [`I2cBus`] to drive. Required for
    ///   `transactional`, ignored for `wired`.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property or a missing required one,
    /// [`Error::Config`] for a `link` this module does not know or a
    /// transactional controller with no bus.
    pub fn new(props: &Props) -> Result<Stm32I2cV2> {
        let mut r = props.reader();
        let link_name = alloc::string::ToString::to_string(r.require_str("link")?);
        let bus_name = r.optional_str("bus")?.map(String::from);
        r.finish()?;

        let link = Link::from_name(&link_name).ok_or_else(|| Error::Config {
            at: String::from(CLASS_NAME),
            message: alloc::format!(
                "`link` is `{link_name}`; it must be one of {:?} — see docs/buses/low-speed.md \
                 for which to pick",
                Link::NAMES
            ),
        })?;
        if link == Link::Transactional && bus_name.is_none() {
            return Err(Error::Config {
                at: String::from(CLASS_NAME),
                message: String::from(
                    "a `transactional` controller reaches its slaves through a named bus; give it \
                     `bus = \"i2c1\"` and name the same bus on each device",
                ),
            });
        }
        let bus = bus_name
            .as_deref()
            .map(|name| buses::attach(props, name))
            .transpose()?;
        Stm32I2cV2::with_bus(link, bus)
    }

    /// A controller on a bus the caller already holds.
    ///
    /// What [`Stm32I2cV2::new`] ends up calling, and the way to build one
    /// without going through the named table — an embedder that owns its own
    /// [`I2cBus`], or a test that wants a bus nothing else can reach.
    ///
    /// A controller with a bus puts its **slave face** on it here, so that
    /// another master can address it. That face answers nothing until `OAR1` or
    /// `OAR2` is enabled, and it refuses every address while this controller is
    /// itself the master — which is how "a part cannot address itself" is
    /// modelled.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the bus already holds
    /// [`MAX_SLAVES`](crate::bus::i2c::MAX_SLAVES) devices.
    pub fn with_bus(link: Link, bus: Option<Arc<I2cBus>>) -> Result<Stm32I2cV2> {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(STATE_RANK, State::default()),
            link,
            bus: bus.clone(),
            wires: Arc::new(MasterWires::new()),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            ev: Mutex::with_rank(LockRank::WIRE, None),
            er: Mutex::with_rank(LockRank::WIRE, None),
            tx_drq: Mutex::with_rank(LockRank::WIRE, None),
            rx_drq: Mutex::with_rank(LockRank::WIRE, None),
            ev_level: AtomicBool::new(false),
            er_level: AtomicBool::new(false),
            tx_drq_level: AtomicBool::new(false),
            rx_drq_level: AtomicBool::new(false),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
        });
        if let Some(bus) = bus.as_ref() {
            let face = Arc::new(SlaveFace {
                shared: Arc::clone(&shared),
            });
            bus.attach(face as Arc<dyn I2cSlave>)?;
        }
        let port = Arc::new(RegisterPort {
            shared: Arc::clone(&shared),
        });
        let region = Arc::new(Region::io("i2c", REGISTER_BYTES, port as Arc<dyn MemOps>));
        Ok(Stm32I2cV2 { shared, region })
    }

    /// How this controller carries a byte.
    #[must_use]
    pub fn link(&self) -> Link {
        self.shared.link
    }

    /// The bus it drives transactionally, if it has one.
    #[must_use]
    pub fn bus(&self) -> Option<&Arc<I2cBus>> {
        self.shared.bus.as_ref()
    }

    /// Its wire-level engine, for a machine that drives the lines.
    #[must_use]
    pub fn wires(&self) -> &Arc<MasterWires> {
        &self.shared.wires
    }

    /// Domain ticks simulated.
    #[must_use]
    pub fn ticks(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    /// `ISR`, `BUSY` included, without any side effect.
    #[must_use]
    pub fn isr(&self) -> u32 {
        let busy = self.shared.bus_busy();
        self.shared.state.lock().read_isr(busy)
    }

    /// `PECR`.
    #[must_use]
    pub fn pec(&self) -> u8 {
        self.shared.state.lock().pec
    }

    /// Whether the peripheral is holding SCL low waiting for software.
    #[must_use]
    pub fn stretching(&self) -> bool {
        self.shared.state.lock().stretching()
    }

    /// The level `I2Cx_EV` is being driven to.
    #[must_use]
    pub fn ev_level(&self) -> Level {
        Level::from_bool(self.shared.ev_level.load(Ordering::Relaxed))
    }

    /// The level `I2Cx_ER` is being driven to.
    #[must_use]
    pub fn er_level(&self) -> Level {
        Level::from_bool(self.shared.er_level.load(Ordering::Relaxed))
    }

    /// The level the transmit DMA request is being driven to.
    #[must_use]
    pub fn tx_drq_level(&self) -> Level {
        Level::from_bool(self.shared.tx_drq_level.load(Ordering::Relaxed))
    }

    /// The level the receive DMA request is being driven to.
    #[must_use]
    pub fn rx_drq_level(&self) -> Level {
        Level::from_bool(self.shared.rx_drq_level.load(Ordering::Relaxed))
    }

    /// Run the controller until `target` domain ticks have passed in total.
    pub fn advance_to(&self, target: u64) {
        self.shared.advance_to(target);
    }
}

// ---------------------------------------------------------------------------
// The engine
// ---------------------------------------------------------------------------

impl Shared {
    /// Publish what the scheduler may ask for without taking a lock.
    fn publish(&self, state: &State) {
        self.ticks.store(state.ticks, Ordering::Relaxed);
        self.next_event.store(
            if state.op.is_some() {
                state.next_edge.max(state.ticks.saturating_add(1))
            } else {
                NO_EVENT
            },
            Ordering::Relaxed,
        );
    }

    /// Whether a transaction is open on the bus, for `ISR.BUSY`.
    ///
    /// **Takes no state lock**, and every caller reads it before locking: the
    /// fabric and the bit engine both rank *above* [`STATE_RANK`], so asking
    /// them while holding it would be a ladder violation.
    fn bus_busy(&self) -> bool {
        match self.link {
            Link::Wired => self.wires.busy(),
            Link::Transactional => self.bus.as_ref().is_some_and(|b| b.state().is_busy()),
        }
    }

    /// Bring the controller up to date before an access.
    ///
    /// A debug access advances nothing (`ROADMAP.md` §15, invariant 5).
    fn sync(&self, attrs: MemAttrs) {
        let handle = self.lazy.lock().clone();
        let Some(handle) = handle else {
            return;
        };
        let kind = if attrs.debug {
            AccessKind::Debug
        } else {
            AccessKind::Guest
        };
        // A refusal means catch-up for this device is already running further
        // up the stack. The access still has to be answered, and answering it
        // from where the controller stands is the only defined thing to do.
        let _ = handle.sync(kind);
    }

    /// Re-drive the interrupt and DMA-request outputs from the flags.
    ///
    /// Called with no lock of ours held: driving a wire reaches an interrupt
    /// controller or a DMA controller, which is another device (the re-entrancy
    /// contract).
    fn update_outputs(&self) {
        let (ev, er, tx, rx) = {
            let state = self.state.lock();
            (state.ev(), state.er(), state.tx_drq(), state.rx_drq())
        };
        self.ev_level.store(ev, Ordering::Relaxed);
        self.er_level.store(er, Ordering::Relaxed);
        self.tx_drq_level.store(tx, Ordering::Relaxed);
        self.rx_drq_level.store(rx, Ordering::Relaxed);
        // Each guard is taken and dropped on its own statement: four
        // `LockRank::WIRE` locks alive at once inside one array expression is a
        // ladder violation, and the debug ladder catches it on the first test.
        let ev_port = self.ev.lock().clone();
        let er_port = self.er.lock().clone();
        let tx_port = self.tx_drq.lock().clone();
        let rx_port = self.rx_drq.lock().clone();
        for (port, level) in [(ev_port, ev), (er_port, er), (tx_port, tx), (rx_port, rx)] {
            if let Some(port) = port {
                port.set(Level::from_bool(level));
            }
        }
    }

    /// Decide what the engine should do next, if anything.
    ///
    /// Called with the state lock held; the submission itself happens once it
    /// is released. A stage that names an operation *is* the pending
    /// instruction to perform it, so this is the whole of §39.4.7's master
    /// sequencing in one place.
    fn decide(state: &mut State) -> Option<Op> {
        if state.op.is_some() || !state.enabled() {
            return None;
        }
        match state.stage {
            // §39.4.7: "the START bit is set ... the master sends a START
            // condition followed by the slave address".
            Stage::Idle => {
                if state.cr2 & CR2_START == 0 {
                    return None;
                }
                state.stage = Stage::Starting;
                Some(Op::Start)
            }
            Stage::AddrSeven | Stage::AddrTenHeadW | Stage::AddrTenLow | Stage::AddrTenHeadR => {
                Some(Op::Write(state.address_byte()))
            }
            // UM10204 §3.1.11's Sr, between the 10-bit write address and the
            // read header that turns the transfer round.
            Stage::Restarting => Some(Op::Start),
            Stage::Tx => {
                if state.count == 0 {
                    return Shared::finish_or_turn(state);
                }
                if state.pec_due() {
                    // §39.4.13: the last byte is the PEC and hardware, not
                    // `TXDR`, provides it.
                    return Some(Op::Write(state.pec));
                }
                if state.any(ISR_TXE) {
                    // Nothing loaded. `TXIS` is up and SCL stays low.
                    return None;
                }
                let byte = state.txdr;
                state.set(ISR_TXE);
                state.clear(ISR_TXIS);
                Some(Op::Write(byte))
            }
            Stage::Rx => {
                if state.count == 0 {
                    return Shared::finish_or_turn(state);
                }
                if state.any(ISR_RXNE) {
                    // The guest has not read the last byte. SCL stays low.
                    return None;
                }
                Some(Op::Read(state.rx_ack()))
            }
            Stage::Held => Shared::finish_or_turn(state),
            // An operation is in flight; these stages are never reached with
            // `op` clear.
            Stage::Starting | Stage::Stopping => None,
        }
    }

    /// What to do when the byte counter has run out, or the master is simply
    /// holding the bus: the STOP the hardware owes, the one software asked for,
    /// or the repeated START that begins the next leg (§39.4.7).
    fn finish_or_turn(state: &mut State) -> Option<Op> {
        if state.auto_stop {
            state.auto_stop = false;
            state.stage = Stage::Stopping;
            return Some(Op::Stop);
        }
        if state.cr2 & CR2_STOP != 0 {
            state.stage = Stage::Stopping;
            return Some(Op::Stop);
        }
        if state.cr2 & CR2_START != 0 {
            state.stage = Stage::Starting;
            return Some(Op::Start);
        }
        None
    }

    /// Submit whatever [`Shared::decide`] chose, and schedule its first edge.
    ///
    /// Called with no lock held.
    fn pump(&self) {
        let op = {
            let mut state = self.state.lock();
            let Some(op) = Shared::decide(&mut state) else {
                self.publish(&state);
                return;
            };
            state.op = Some(op);
            if let Op::Write(byte) = op {
                state.shift_out = byte;
            }
            state.halves_left = op.halves();
            state.high_half = false;
            state.next_edge = state.ticks.saturating_add(state.half_len());
            self.publish(&state);
            op
        };
        // Outside the lock: the bit engine ranks below this state.
        if self.link == Link::Wired {
            self.wires.submit(op.to_wire());
        }
    }

    /// Run one SCL half period. Called with no lock held.
    fn half_step(&self) -> MasterEvent {
        match self.link {
            Link::Wired => self.wires.tick(),
            Link::Transactional => self.half_step_transactional(),
        }
    }

    /// The transactional half period: burn time, then perform the bus event.
    ///
    /// The stretch check is the transactional stand-in for looking at SCL, and
    /// it costs the same half period the wired path would burn — which is what
    /// keeps a guest's view of time identical under both.
    fn half_step_transactional(&self) -> MasterEvent {
        let Some(bus) = self.bus.as_ref() else {
            // A transactional controller with no bus cannot exist through
            // `new`, but a test may build one. Clock into the void.
            let mut state = self.state.lock();
            let op = state.op;
            state.op = None;
            return match op {
                Some(Op::Start) => MasterEvent::Started,
                Some(Op::Write(_)) => MasterEvent::Wrote(Ack::Nack),
                Some(Op::Read(_)) => MasterEvent::Read(0xff),
                Some(Op::Stop) => MasterEvent::Stopped,
                None => MasterEvent::Idle,
            };
        };
        if bus.stretching() {
            return MasterEvent::Stretched;
        }
        let op = {
            let mut state = self.state.lock();
            let Some(op) = state.op else {
                return MasterEvent::Idle;
            };
            state.halves_left = state.halves_left.saturating_sub(1);
            if state.halves_left > 0 {
                return MasterEvent::Working;
            }
            state.op = None;
            op
        };
        // Outward, with no lock of ours held.
        match op {
            Op::Start => MasterEvent::Started,
            Op::Write(byte) => MasterEvent::Wrote(self.transactional_byte(byte)),
            Op::Read(ack) => MasterEvent::Read(bus.read(ack)),
            Op::Stop => {
                bus.stop();
                MasterEvent::Stopped
            }
        }
    }

    /// One byte out, routed by what the stage says it is.
    ///
    /// An address byte is a START plus an address to [`I2cBus`] and eight bits
    /// plus an acknowledge to a wired slave, so this is the one place the two
    /// links have to be told apart — and it is written so that the *slave* sees
    /// the identical sequence of [`I2cSlave`] calls either way.
    fn transactional_byte(&self, byte: u8) -> Ack {
        let Some(bus) = self.bus.as_ref() else {
            return Ack::Nack;
        };
        let (stage, ten) = {
            let state = self.state.lock();
            (state.stage, state.ten())
        };
        match stage {
            // UM10204 §3.1.11's A1: the header alone is acknowledged by every
            // device whose top two address bits match, and the address is not
            // complete until the second byte.
            Stage::AddrTenHeadW => bus.ten_bit_header((byte >> 1) & 0b11),
            Stage::AddrTenLow => bus.start(Address::Ten(ten), Direction::Write),
            Stage::AddrTenHeadR => bus.start(Address::Ten(ten), Direction::Read),
            Stage::AddrSeven => {
                bus.start(Address::seven_from_byte(byte), Direction::from_bit(byte))
            }
            _ => bus.write(byte),
        }
    }

    /// Apply what a half period produced to the register file.
    ///
    /// Returns nothing: every outward action it implies is done by the callers,
    /// which run [`Shared::pump`] and [`Shared::update_outputs`] afterwards.
    fn apply(&self, event: MasterEvent) {
        let mut state = self.state.lock();
        match event {
            MasterEvent::Idle | MasterEvent::Working => {}
            MasterEvent::Stretched => {
                // No progress, and the half period is not counted against the
                // alternating low/high pair either: a stretched clock extends
                // the low period rather than replacing it.
                return;
            }
            MasterEvent::Started => {
                state.op = None;
                Shared::on_started(&mut state);
            }
            MasterEvent::Wrote(ack) => {
                state.op = None;
                let byte = state.shift_out;
                Shared::on_wrote(&mut state, byte, ack);
            }
            MasterEvent::Read(byte) => {
                state.op = None;
                Shared::on_read(&mut state, byte);
            }
            MasterEvent::Stopped => {
                state.op = None;
                // §39.4.7: `STOPF` is set when a STOP is detected, and the
                // transfer register is flushed.
                state.set(ISR_STOPF | ISR_TXE);
                state.clear(ISR_TXIS);
                state.cr2 &= !(CR2_START | CR2_STOP);
                state.count = 0;
                state.auto_stop = false;
                state.stage = Stage::Idle;
                // **`RXNE` survives the STOP**, and it has to: with `AUTOEND`
                // the hardware sends the STOP the instant the last byte is in,
                // so clearing it here would throw away the byte the whole
                // transfer was for. The v1 block learned this the expensive
                // way; the note is in `src/dev/stm32/i2c.rs`.
            }
            MasterEvent::ArbitrationLost => {
                // §39.4.10: "the peripheral automatically switches back to
                // slave mode" and `ARLO` is set.
                state.op = None;
                state.set(ISR_ARLO | ISR_TXE);
                state.clear(ISR_TXIS);
                state.cr2 &= !(CR2_START | CR2_STOP);
                state.count = 0;
                state.auto_stop = false;
                state.stage = Stage::Idle;
            }
        }
        // The half period alternates only when one actually elapsed.
        state.high_half = !state.high_half;
        self.publish(&state);
    }

    /// A START condition went out.
    ///
    /// Either the one that opens a transfer — which is where `NBYTES` is
    /// latched into the byte counter and the address sequence is chosen — or
    /// the repeated START inside a 10-bit read (§39.4.7, UM10204 §3.1.11).
    fn on_started(state: &mut State) {
        if state.stage == Stage::Restarting {
            state.stage = Stage::AddrTenHeadR;
            return;
        }
        // §39.4.7: "the number of bytes to be transferred is programmed in
        // NBYTES", and the counter is loaded when the transfer begins.
        state.count = nbytes(state.cr2);
        state.dir = if state.cr2 & CR2_RD_WRN != 0 {
            Direction::Read
        } else {
            Direction::Write
        };
        // §39.4.13: the packet error check covers the address byte onwards.
        state.pec = 0;
        state.clear(ISR_TC | ISR_TCR);
        state.stage = if state.cr2 & CR2_ADD10 == 0 {
            Stage::AddrSeven
        } else if state.dir == Direction::Read && state.cr2 & CR2_HEAD10R != 0 {
            // §39.7.2's `HEAD10R` = 1: "the master only sends the 1st 7 bits of
            // the 10-bit address, followed by Read direction."
            Stage::AddrTenHeadR
        } else {
            Stage::AddrTenHeadW
        };
    }

    /// A byte went out and the ninth clock came back.
    fn on_wrote(state: &mut State, byte: u8, ack: Ack) {
        let was_pec = state.stage == Stage::Tx && state.pec_due();
        if state.pec_transfer() && !was_pec {
            state.pec = crc8(state.pec, byte);
        }
        if !ack.is_ack() {
            Shared::nack(state);
            return;
        }
        match state.stage {
            Stage::AddrTenHeadW => state.stage = Stage::AddrTenLow,
            Stage::AddrTenLow => {
                if state.dir == Direction::Read {
                    state.stage = Stage::Restarting;
                } else {
                    Shared::address_done(state);
                }
            }
            Stage::AddrSeven | Stage::AddrTenHeadR => Shared::address_done(state),
            Stage::Tx => {
                state.count = state.count.saturating_sub(1);
                if state.count > 0 {
                    if state.any(ISR_TXE) {
                        state.set(ISR_TXIS);
                    }
                } else {
                    Shared::transfer_complete(state);
                }
            }
            _ => {}
        }
    }

    /// The address phase finished and the data phase begins.
    fn address_done(state: &mut State) {
        // §39.7.2: `START` is "cleared by hardware ... after the Start followed
        // by the address sequence is sent".
        state.cr2 &= !CR2_START;
        state.stage = if state.dir == Direction::Write {
            Stage::Tx
        } else {
            Stage::Rx
        };
        if state.count == 0 {
            // `NBYTES = 0` is a legitimate transfer: it is how firmware probes
            // for a device without writing to it.
            Shared::transfer_complete(state);
        } else if state.dir == Direction::Write && state.any(ISR_TXE) {
            // §39.7.7: `TXIS` is "set by hardware when the I2C_TXDR register is
            // empty and the data to be transmitted must be written".
            state.set(ISR_TXIS);
        }
    }

    /// Nobody acknowledged (§39.4.8).
    ///
    /// "In master mode, the STOP condition is automatically generated after a
    /// NACK reception", which is the difference from v1 that most changes what
    /// a driver has to do: there is no `AF` to notice and no STOP to program.
    fn nack(state: &mut State) {
        state.set(ISR_NACKF | ISR_TXE);
        state.clear(ISR_TXIS);
        state.cr2 &= !CR2_START;
        state.count = 0;
        state.auto_stop = true;
        state.stage = Stage::Held;
    }

    /// The byte counter reached zero (§39.4.7).
    ///
    /// The three-way fork this whole block exists for. `RELOAD` and `AUTOEND`
    /// are not alternatives to `TC` — they *replace* it, and a driver is
    /// written around which of the three it asked for.
    fn transfer_complete(state: &mut State) {
        if state.cr2 & CR2_RELOAD != 0 {
            // "TCR is set when NBYTES data have been transferred and RELOAD is
            // set"; SCL is stretched until a new `NBYTES` arrives. The stage
            // stays `Tx`/`Rx`, so the reload resumes the same leg rather than
            // re-addressing.
            state.set(ISR_TCR);
        } else if state.cr2 & CR2_AUTOEND != 0 {
            // "the STOP condition is automatically sent when NBYTES data are
            // transferred." No `TC`, and nothing for software to do.
            state.auto_stop = true;
            state.stage = Stage::Held;
        } else {
            // "TC is set when NBYTES data have been transferred and both RELOAD
            // and AUTOEND are cleared"; SCL is stretched until software writes
            // `START` (a repeated START) or `STOP`.
            state.set(ISR_TC);
            state.stage = Stage::Held;
        }
    }

    /// A byte came in.
    fn on_read(state: &mut State, byte: u8) {
        let was_pec = state.pec_due();
        state.count = state.count.saturating_sub(1);
        if was_pec {
            // §39.4.13: the PEC byte is compared, not delivered — `RXDR` never
            // sees it and `PECR` keeps the value this end computed, so a
            // mismatch is visible as both `PECERR` and a differing `PECR`.
            if byte != state.pec {
                state.set(ISR_PECERR);
            }
        } else {
            if state.pec_transfer() {
                state.pec = crc8(state.pec, byte);
            }
            state.rxdr = byte;
            state.set(ISR_RXNE);
        }
        if state.count == 0 {
            Shared::transfer_complete(state);
        }
    }

    /// Simulate forward to `target` domain ticks.
    ///
    /// Runs with **no lock held across an outward call**: each step decides
    /// what to do under the state lock, releases it, then drives the wire or
    /// reaches the bus (`core::device`, the re-entrancy contract).
    fn advance_to(&self, target: u64) {
        loop {
            let step = {
                let mut state = self.state.lock();
                if state.op.is_none() {
                    state.ticks = state.ticks.max(target);
                    self.publish(&state);
                    false
                } else if state.next_edge > target {
                    state.ticks = target.max(state.ticks);
                    self.publish(&state);
                    false
                } else {
                    state.ticks = state.next_edge;
                    true
                }
            };
            if !step {
                return;
            }
            let event = self.half_step();
            self.apply(event);
            {
                let mut state = self.state.lock();
                state.next_edge = state.ticks.saturating_add(state.half_len());
                self.publish(&state);
            }
            self.pump();
            self.update_outputs();
        }
    }
}

// ---------------------------------------------------------------------------
// The slave face
// ---------------------------------------------------------------------------

/// This controller seen from the bus: the slave half of §39.4.9.
///
/// Attached to the [`I2cBus`] at construction and inert until `OAR1` or `OAR2`
/// is enabled. Every method takes the state lock for a short critical section
/// and releases it before doing anything outward, which is the re-entrancy
/// contract read literally — the outward part here is only re-driving the
/// interrupt lines.
struct SlaveFace {
    shared: Arc<Shared>,
}

impl fmt::Debug for SlaveFace {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Stm32I2cV2Slave").finish_non_exhaustive()
    }
}

impl I2cSlave for SlaveFace {
    fn address(&self, address: Address, dir: Direction) -> Ack {
        let answer = {
            let mut state = self.shared.state.lock();
            // A part cannot address itself: while this controller is the master
            // its own slave face is deaf.
            if !state.enabled() || state.stage != Stage::Idle {
                return Ack::Nack;
            }
            match state.own_address(address) {
                None => {
                    if state.slave_active {
                        // §3.1.11 of UM10204: a repeated START that addressed
                        // somebody else ends our transaction.
                        state.slave_active = false;
                        state.clear(ISR_TXIS | ISR_DIR);
                        state.set(ISR_TXE);
                    }
                    return Ack::Nack;
                }
                Some(code) => {
                    state.slave_active = true;
                    // §39.7.7: `ADDCODE` is "updated with the received address
                    // when an address match event occurs", and `DIR` says which
                    // way the master asked to go.
                    state.clear(ISR_ADDCODE_MASK | ISR_DIR);
                    state.set(ISR_ADDR | (u32::from(code) << ISR_ADDCODE_SHIFT));
                    if dir == Direction::Read {
                        state.set(ISR_DIR);
                        if state.any(ISR_TXE) {
                            state.set(ISR_TXIS);
                        }
                    }
                    Ack::Ack
                }
            }
        };
        self.shared.update_outputs();
        answer
    }

    fn ten_bit_header(&self, high: u8) -> bool {
        let state = self.shared.state.lock();
        state.enabled()
            && state.stage == Stage::Idle
            && state.oar1 & (OAR1_OA1EN | OAR1_OA1MODE) == (OAR1_OA1EN | OAR1_OA1MODE)
            && ((state.oar1 >> 8) & 0x3) == u32::from(high & 0x3)
    }

    fn write(&self, byte: u8) -> Ack {
        let answer = {
            let mut state = self.shared.state.lock();
            if !state.slave_active {
                return Ack::Nack;
            }
            if state.any(ISR_RXNE) {
                // §39.4.9: with `NOSTRETCH` the byte the guest has not read is
                // lost and `OVR` says so. With stretching the master never gets
                // here, because it asks `stretching()` first.
                state.set(ISR_OVR);
                Ack::Ack
            } else {
                state.rxdr = byte;
                state.set(ISR_RXNE);
                if state.cr2 & CR2_NACK != 0 {
                    // §39.7.2: `NACK` is "cleared by hardware when the NACK is
                    // sent".
                    state.cr2 &= !CR2_NACK;
                    Ack::Nack
                } else {
                    Ack::Ack
                }
            }
        };
        self.shared.update_outputs();
        answer
    }

    fn read(&self) -> u8 {
        let mut state = self.shared.state.lock();
        if state.any(ISR_TXE) {
            // Nothing loaded. §39.4.9: an underrun under `NOSTRETCH` sends
            // `0xFF` — an undriven, pulled-up SDA — and sets `OVR`.
            state.set(ISR_OVR);
            return 0xff;
        }
        state.txdr
    }

    fn read_ack(&self, ack: Ack) {
        {
            let mut state = self.shared.state.lock();
            state.set(ISR_TXE);
            state.clear(ISR_TXIS);
            if ack.is_ack() {
                // The master wants another one.
                state.set(ISR_TXIS);
            }
        }
        self.shared.update_outputs();
    }

    fn stop(&self) {
        {
            let mut state = self.shared.state.lock();
            if !state.slave_active {
                return;
            }
            state.slave_active = false;
            state.set(ISR_STOPF | ISR_TXE);
            state.clear(ISR_TXIS | ISR_DIR);
        }
        self.shared.update_outputs();
    }

    fn stretching(&self) -> bool {
        let state = self.shared.state.lock();
        state.slave_active && state.stretching()
    }

    fn peek(&self) -> u8 {
        let state = self.shared.state.lock();
        if state.any(ISR_TXE) { 0xff } else { state.txdr }
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// What a register access asks for once the state lock is released.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum After {
    /// Nothing outward.
    Nothing,
    /// Something may now be startable, and the flags may have moved.
    Pump,
    /// `PE = 0`: abandon whatever is in flight and let both lines go.
    Reset,
}

/// The memory-mapped registers (§39.7).
struct RegisterPort {
    shared: Arc<Shared>,
}

impl fmt::Debug for RegisterPort {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("RegisterPort").finish_non_exhaustive()
    }
}

impl RegisterPort {
    /// Read one register.
    ///
    /// `debug` suppresses every side effect. v2 is gentler than v1 here —
    /// reading `ISR` clears nothing, which is the whole point of having `ICR` —
    /// but `RXDR` still pops the receive register and releases the stretch, so
    /// a debugger that dumped the block would eat the guest's byte.
    fn read_register(&self, offset: u64, debug: bool, busy: bool) -> (u32, After) {
        let mut state = self.shared.state.lock();
        match offset {
            0x00 => (state.cr1, After::Nothing),
            0x04 => (state.cr2, After::Nothing),
            0x08 => (state.oar1, After::Nothing),
            0x0c => (state.oar2, After::Nothing),
            0x10 => (state.timingr, After::Nothing),
            0x14 => (state.timeoutr, After::Nothing),
            0x18 => (state.read_isr(busy), After::Nothing),
            // §39.7.8: `ICR` is write-only.
            0x1c => (0, After::Nothing),
            0x20 => (u32::from(state.pec), After::Nothing),
            0x24 => {
                let value = u32::from(state.rxdr);
                if debug {
                    return (value, After::Nothing);
                }
                // §39.7.7: `RXNE` is "cleared when I2C_RXDR is read", which is
                // also what releases the stretched clock.
                state.clear(ISR_RXNE);
                (value, After::Pump)
            }
            0x28 => (u32::from(state.txdr), After::Nothing),
            _ => (0, After::Nothing),
        }
    }

    /// Write one register, reporting what has to happen once the lock is
    /// released.
    fn write_register(&self, offset: u64, value: u32) -> After {
        let mut state = self.shared.state.lock();
        match offset {
            0x00 => {
                let was_enabled = state.enabled();
                state.cr1 = value & CR1_MASK;
                if was_enabled && !state.enabled() {
                    // §39.4.1: "PE=0 ... the I2C performs a software reset" —
                    // the flags go, the lines are released, and the
                    // configuration registers stay.
                    let keep = State {
                        ticks: state.ticks,
                        cr1: state.cr1,
                        cr2: 0,
                        oar1: state.oar1,
                        oar2: state.oar2,
                        timingr: state.timingr,
                        timeoutr: state.timeoutr,
                        ..State::default()
                    };
                    *state = keep;
                    return After::Reset;
                }
                After::Pump
            }
            0x04 => {
                state.cr2 = value & CR2_MASK;
                if value & (CR2_START | CR2_STOP) != 0 {
                    // §39.7.7: `TC` is "cleared by software when START bit or
                    // STOP bit is set".
                    state.clear(ISR_TC);
                }
                if state.any(ISR_TCR) && nbytes(value) != 0 {
                    // §39.7.7: `TCR` is "cleared by software when NBYTES is
                    // written to a non-zero value". The counter reloads and the
                    // same leg continues — no address, no START.
                    state.clear(ISR_TCR);
                    state.count = nbytes(value);
                    if state.stage == Stage::Tx && state.any(ISR_TXE) {
                        state.set(ISR_TXIS);
                    }
                }
                After::Pump
            }
            0x08 => {
                // §39.7.3: "OA1[9:0] and OA1MODE should be written only when
                // OA1EN = 0", so an enabled own address only takes the enable
                // bit itself — which is exactly what makes a driver's
                // clear-then-program sequence necessary on real silicon.
                if state.oar1 & OAR1_OA1EN == 0 {
                    state.oar1 = value & OAR1_MASK;
                } else {
                    state.oar1 = (state.oar1 & !OAR1_OA1EN) | (value & OAR1_OA1EN);
                }
                After::Nothing
            }
            0x0c => {
                if state.oar2 & OAR2_OA2EN == 0 {
                    state.oar2 = value & OAR2_MASK;
                } else {
                    state.oar2 = (state.oar2 & !OAR2_OA2EN) | (value & OAR2_OA2EN);
                }
                After::Nothing
            }
            0x10 => {
                state.timingr = value & TIMINGR_MASK;
                After::Nothing
            }
            0x14 => {
                state.timeoutr = value & TIMEOUTR_MASK;
                After::Nothing
            }
            0x18 => {
                // §39.7.7: only `TXE` and `TXIS` are writable, and only to 1.
                // Writing `TXE` "flushes the TXDR register", which is how a
                // driver abandons a byte it no longer wants sent.
                if value & ISR_TXE != 0 {
                    state.set(ISR_TXE);
                }
                if value & ISR_TXIS != 0 {
                    state.set(ISR_TXIS);
                }
                After::Pump
            }
            0x1c => {
                state.clear(value & ICR_MASK);
                After::Pump
            }
            // §39.7.9, §39.7.11: `PECR` and `RXDR` are read-only.
            0x20 | 0x24 => After::Nothing,
            0x28 => {
                // §39.7.12: `TXDR` is written when `TXE` is set. A write with
                // the register already loaded is dropped rather than
                // overwriting the byte on its way out.
                if state.any(ISR_TXE) {
                    state.txdr = value as u8;
                    state.clear(ISR_TXE | ISR_TXIS);
                }
                After::Pump
            }
            _ => After::Nothing,
        }
    }

    /// Do what a handler asked for, with no lock held.
    fn finish(&self, after: After) {
        match after {
            After::Nothing => {}
            After::Pump => {
                self.shared.pump();
                self.shared.update_outputs();
            }
            After::Reset => {
                self.shared.wires.reset();
                self.shared.update_outputs();
            }
        }
    }
}

impl MemOps for RegisterPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if !matches!(dst.len(), 2 | 4) || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        // Before the state lock: the fabric and the bit engine both rank above
        // it, so asking either while holding it is a ladder violation.
        let busy = self.shared.bus_busy();
        let (value, after) = self.read_register(offset, attrs.debug, busy);
        match dst.len() {
            2 => dst.copy_from_slice(&(value as u16).to_le_bytes()),
            _ => dst.copy_from_slice(&value.to_le_bytes()),
        }
        self.finish(after);
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if !matches!(src.len(), 2 | 4) || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write would start a transfer, move a chip's address or
            // clear a flag, none of which the core can make harmless.
            return Err(BusError::BadAccess);
        }
        self.shared.sync(attrs);
        let value = match src.len() {
            2 => u32::from(u16::from_le_bytes([src[0], src[1]])),
            _ => u32::from_le_bytes([src[0], src[1], src[2], src[3]]),
        };
        let after = self.write_register(offset, value);
        self.finish(after);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints {
            min: Width::U16,
            max: Width::U32,
            natural_alignment: true,
            endian: Endian::Little,
            allow_bulk: false,
            ..AccessConstraints::IO
        }
    }
}

// ---------------------------------------------------------------------------
// Device
// ---------------------------------------------------------------------------

impl Device for Stm32I2cV2 {
    fn class(&self) -> &'static DeviceClass {
        &ST_I2C_V2_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and `wire`
        // statements connect the lines. The slave face went onto the bus in
        // `new`, which is allocation rather than an observable action.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut state = self.shared.state.lock();
            // The tick is *not* zeroed: `Machine::reset` does not rewind clock
            // domains (`ROADMAP.md` §4.2).
            let ticks = state.ticks;
            *state = State {
                ticks,
                ..State::default()
            };
            self.shared.publish(&state);
        }
        self.shared.wires.reset();
        self.shared.update_outputs();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u32(state.cr1)?;
        w.write_u32(state.cr2)?;
        w.write_u32(state.oar1)?;
        w.write_u32(state.oar2)?;
        w.write_u32(state.timingr)?;
        w.write_u32(state.timeoutr)?;
        w.write_u32(state.isr)?;
        w.write_u8(state.rxdr)?;
        w.write_u8(state.txdr)?;
        w.write_u8(state.shift_out)?;
        w.write_u8(state.pec)?;
        // The byte counter is *not* derivable from `CR2.NBYTES`: a reload has
        // moved it, and a snapshot taken half way through a leg has to come
        // back with the same number of bytes left to run.
        w.write_u8(state.count)?;
        w.write_u8(stage_code(state.stage))?;
        w.write_bool(state.dir == Direction::Read)?;
        w.write_bool(state.auto_stop)?;
        w.write_bool(state.slave_active)?;
        let (op, operand) = state.op.map_or((0, 0), Op::code);
        w.write_u8(op)?;
        w.write_u8(operand)?;
        w.write_u32(state.halves_left)?;
        w.write_u64(state.next_edge)?;
        w.write_bool(state.high_half)?;
        self.shared.wires.snapshot().write(w)
        // The interrupt and DMA outputs are not saved: they are a pure function
        // of the flags, and `load` re-derives and re-announces them.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let state = State {
            ticks: r.read_u64()?,
            cr1: r.read_u32()?,
            cr2: r.read_u32()?,
            oar1: r.read_u32()?,
            oar2: r.read_u32()?,
            timingr: r.read_u32()?,
            timeoutr: r.read_u32()?,
            isr: r.read_u32()?,
            rxdr: r.read_u8()?,
            txdr: r.read_u8()?,
            shift_out: r.read_u8()?,
            pec: r.read_u8()?,
            count: r.read_u8()?,
            stage: stage_from_code(r.read_u8()?),
            dir: if r.read_bool()? {
                Direction::Read
            } else {
                Direction::Write
            },
            auto_stop: r.read_bool()?,
            slave_active: r.read_bool()?,
            op: {
                let code = r.read_u8()?;
                let operand = r.read_u8()?;
                Op::from_code(code, operand)
            },
            halves_left: r.read_u32()?,
            next_edge: r.read_u64()?,
            high_half: r.read_bool()?,
        };
        let wires = MasterWiresState::read(r)?;
        {
            let mut slot = self.shared.state.lock();
            *slot = state;
            self.shared.publish(&slot);
        }
        self.shared.wires.restore(wires);
        self.shared.update_outputs();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        match port {
            line::SCL_NAME => Some(SinkPin {
                sink: self.shared.wires.sink(line::SCL, sources),
                line: line::SCL,
            }),
            line::SDA_NAME => Some(SinkPin {
                sink: self.shared.wires.sink(line::SDA, sources),
                line: line::SDA,
            }),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        match port {
            line::SCL_NAME => self.shared.wires.connect(line::SCL, source),
            line::SDA_NAME => self.shared.wires.connect(line::SDA, source),
            pin::EV => *self.shared.ev.lock() = Some(source),
            pin::ER => *self.shared.er.lock() = Some(source),
            pin::TX_DRQ => *self.shared.tx_drq.lock() = Some(source),
            pin::RX_DRQ => *self.shared.rx_drq.lock() = Some(source),
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!(
                        "an STM32 I2C v2 drives `{}` and `{}` — both open drain, and only ever \
                         low — plus the interrupt outputs `{}` and `{}` and the DMA requests `{}` \
                         and `{}`",
                        line::SCL_NAME,
                        line::SDA_NAME,
                        pin::EV,
                        pin::ER,
                        pin::TX_DRQ,
                        pin::RX_DRQ
                    ),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.wires.announce();
        self.shared.update_outputs();
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. A transfer takes real time, a guest polls `ISR` to find out how far
    /// it has got, and the answer has to be the one at the cycle of the poll.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Stm32I2cV2::advance_to(self, tick);
    }

    fn next_event_tick(&self) -> Option<u64> {
        match self.shared.next_event.load(Ordering::Relaxed) {
            NO_EVENT => None,
            tick => Some(tick),
        }
    }

    fn attach_lazy(&self, handle: LazyHandle) {
        *self.shared.lazy.lock() = Some(handle);
    }
}

impl Instance for Stm32I2cV2 {}

/// The `st.i2c-v2` device class.
pub static ST_I2C_V2_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 I2C v2 (F0/F3/F7/L0/L4/G0/G4/H7/U5): the NBYTES/RELOAD/AUTOEND transfer \
              machine, TIMINGR clocking, ISR/ICR, 7- and 10-bit addressing, SMBus PEC, \
              transactional or wired",
    properties: &[
        PropertySpec {
            name: "link",
            kind: ValueKind::Str,
            required: true,
            summary: "how bytes reach the slaves: `transactional` or `wired`",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the named I2C bus this controller drives, for `transactional`",
        },
    ],
    construct: |props| Ok(Box::new(Stm32I2cV2::new(props)?)),
};

/// Add [`ST_I2C_V2_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&ST_I2C_V2_CLASS)
}

/// Bind [`ST_I2C_V2_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Stm32I2cV2::new(props)?)))
}

/// What the validator should know about `st.i2c-v2`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(
            PropSchema::new("link", ValueKind::Str)
                .required()
                .values(Link::NAMES),
        )
        .prop(PropSchema::new("bus", ValueKind::Str))
        // Both bus lines are open drain, so each is an input *and* an output.
        .port(line::SCL_NAME, PortDir::InOut)
        .port(line::SDA_NAME, PortDir::InOut)
        .port(pin::EV, PortDir::Out)
        .port(pin::ER, PortDir::Out)
        .port(pin::TX_DRQ, PortDir::Out)
        .port(pin::RX_DRQ, PortDir::Out)
        .region("")
        .region("regs")
}
