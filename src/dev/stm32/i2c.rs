//! The STM32 I²C peripheral — **the F1/F2/F4/L1 one**, not the other one.
//!
//! # Which one, and why that has to be said
//!
//! "The STM32 I²C controller" names two completely different pieces of silicon,
//! and a driver for one does not resemble a driver for the other:
//!
//! | | registers | how a transfer is driven |
//! | --- | --- | --- |
//! | **I2C v1** — F1, F2, F4, L1 | `CR1` `CR2` `OAR1` `OAR2` `DR` `SR1` `SR2` `CCR` `TRISE` | event flags cleared by *read-then-do* sequences; software drives every byte |
//! | I2C v2 — F0, F3, F7, L0, L4, H7 | `CR1` `CR2` `OAR1` `OAR2` `TIMINGR` `ISR` `ICR` `TXDR` `RXDR` | `CR2` carries `NBYTES`, `AUTOEND` and `RELOAD`; the hardware runs the transfer |
//!
//! **This module is v1**, the awkward one, and it is the awkward one on
//! purpose: its clearing sequences are the sharpest test in this tree of the
//! [`MemAttrs::debug`](crate::core::space::MemAttrs::debug) rule. Reading `SR1`
//! and then reading `SR2` clears `ADDR`; a debugger that dumped the register
//! block would clear it for the guest, the guest's next poll would find it
//! clear, and its driver would hang — a bug that only appears when somebody
//! attaches gdb. So a debug read here arms nothing and clears nothing, and
//! `a_debug_dump_of_the_whole_block_clears_nothing` is the assertion.
//!
//! The v2 block belongs under a name of its own when somebody needs it. It is
//! not a mode of this one.
//!
//! # Source
//!
//! ST **RM0090**, *STM32F405/415, STM32F407/417, STM32F427/437 and
//! STM32F429/439 advanced Arm-based 32-bit MCUs*, **Rev. 4** (Doc ID 018909),
//! chapter **25**, *Inter-integrated circuit (I²C) interface*. Later revisions
//! renumber the chapter — it is §27 in Rev. 21 — so the revision is part of the
//! citation. Section numbers below (§25.3.3, §25.6.6 …) are Rev. 4's.
//!
//! The bus itself is NXP **UM10204**; see [`crate::bus::i2c`]. No emulator was
//! consulted (`ROADMAP.md` §1), and the Linux `i2c-stm32` driver is GPLv2 and
//! was not opened.
//!
//! # What is modelled
//!
//! **Master mode, completely**, in both link models:
//!
//! * The event sequence of §25.3.3 with its exact clearing rules — `EV5` (`SB`,
//!   cleared by reading `SR1` then writing `DR`), `EV6` (`ADDR`, cleared by
//!   reading `SR1` then `SR2`), `EV9` (`ADD10`), `EV8_1`/`EV8`/`EV8_2` (`TxE`,
//!   `BTF`) and `EV7` (`RxNE`).
//! * 7-bit and 10-bit master addressing, including the repeated START with
//!   `1111 0XX1` that turns a 10-bit write into a 10-bit read.
//! * `ACK`, so a master receiver ends a sequential read the way the datasheet
//!   of every part on the bus expects; `AF` when nothing answers; `ARLO` when
//!   another master wins the line; `BUSY` and `MSL`.
//! * `CCR`, `DUTY` and `F/S` as the actual SCL timing, so a transfer costs the
//!   virtual time the guest's clock configuration asks for.
//! * The peripheral's **own clock stretching**: "the EV5, EV6, EV9, EV8_1 and
//!   EV8_2 events stretch SCL low until the end of the corresponding software
//!   sequence" (§25.3.3, Figure 243 note 1). In [`Link::Wired`] that is a real
//!   level on a real net, which is what makes it visible to anything else
//!   watching.
//! * The `I2Cx_EV` and `I2Cx_ER` interrupt outputs, gated by `ITEVTEN`,
//!   `ITBUFEN` and `ITERREN` exactly as §25.6.2 lists them.
//!
//! # What is not
//!
//! * **Slave mode.** `OAR1`, `OAR2` and the `SR2` fields that go with it
//!   (`DUALF`, `GENCALL`, `SMBHOST`, `SMBDEFAULT`) are readable and writable
//!   and nothing answers to them. The reason is structural rather than a
//!   shortage of enthusiasm: on real silicon one pair of pins carries both
//!   roles, so a wired slave mode needs a single bit engine that drives *and*
//!   listens on the same [`OpenDrain`](crate::bus::i2c::wires::OpenDrain)
//!   pair. [`crate::bus::i2c::wires`] has that engine on the slave side and the
//!   master side separately, and gluing them at this level would give a
//!   transactional slave that behaves differently from the wired one — which is
//!   the exact failure this bus was written to avoid. It is a day's work in
//!   `bus::i2c`, not a paragraph here.
//! * **SMBus and PEC.** `SMBUS`, `SMBTYPE`, `ENARP`, `ENPEC`, `PEC`, `ALERT`,
//!   `PECERR`, `TIMEOUT` and `SMBALERT` are register storage. SMBus is a
//!   command layer and a timeout regime on top of I²C (`docs/buses/low-speed.md`)
//!   and belongs with a device that needs it.
//! * **DMA.** `DMAEN` and `LAST` are storage. There is no DMA controller for
//!   this part in the tree yet.
//! * **The noise filters.** `TRISE` and `FLTR` are storage: this model clocks
//!   in half periods and has no spikes to suppress.
//!
//! Everything in that list reads back what was written, so a driver that
//! programs it and then checks does not see a wrong answer — it sees no effect,
//! which is what the docs say.
//!
//! # Time
//!
//! **The scheduler owns it** (`CLAUDE.md`). A *lazily advanced* device
//! (`ROADMAP.md` §4.2) on the APB clock: it holds its own tick, publishes the
//! tick of its next SCL edge, and is caught up before any register access. One
//! bit period costs `Tlow + Thigh` ticks of that domain, derived from `CCR`,
//! `DUTY` and `F/S` by §25.6.8's own formulas — **and it costs the same in both
//! link models**, because [`crate::bus::i2c`] fixes the half-period count of
//! every bus event and only the controller decides what a half period lasts.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::bus::i2c::wires::{MasterEvent, MasterOp, MasterWires, MasterWiresState, pin as line};
use crate::bus::i2c::{
    Ack, Address, BYTE_HALF_PERIODS, Direction, I2cBus, Link, START_HALF_PERIODS,
    STOP_HALF_PERIODS, buses,
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
const CLASS_NAME: &str = "st.i2c";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many bytes of address space the register block occupies.
///
/// `FLTR` is the last register, at `0x24` (§25.6.10), and a peripheral aperture
/// on this family is 0x400 bytes. The region is the register file; a machine
/// file `mirror()`s it across whatever window the SoC decodes, as
/// `machines/spi-panel.machine` does.
pub const REGISTER_BYTES: u64 = 0x28;

// ---------------------------------------------------------------------------
// The register bits (§25.6)
// ---------------------------------------------------------------------------

/// `CR1` bit 0: peripheral enable (§25.6.1).
const CR1_PE: u32 = 1 << 0;
/// `CR1` bit 8: start generation.
const CR1_START: u32 = 1 << 8;
/// `CR1` bit 9: stop generation.
const CR1_STOP: u32 = 1 << 9;
/// `CR1` bit 10: return an acknowledge after a received byte.
const CR1_ACK: u32 = 1 << 10;
/// `CR1` bit 15: software reset.
const CR1_SWRST: u32 = 1 << 15;
/// Everything `CR1` defines. Bits 2 and 14 are reserved and read back zero.
const CR1_MASK: u32 = 0b1011_1111_1111_1011;

/// `CR2` bits 5:0: the peripheral clock frequency in MHz.
///
/// Storage: this model takes its timing from the clock domain the machine file
/// gives it, which is the same number expressed where the emulator can act on
/// it.
const CR2_FREQ: u32 = 0x3f;
/// `CR2` bit 8: error interrupt enable.
const CR2_ITERREN: u32 = 1 << 8;
/// `CR2` bit 9: event interrupt enable.
const CR2_ITEVTEN: u32 = 1 << 9;
/// `CR2` bit 10: buffer interrupt enable — `TxE`/`RxNE` raise `EV` too.
const CR2_ITBUFEN: u32 = 1 << 10;
/// Everything `CR2` defines.
const CR2_MASK: u32 = CR2_FREQ | CR2_ITERREN | CR2_ITEVTEN | CR2_ITBUFEN | (1 << 11) | (1 << 12);

/// Everything `OAR1` defines, bit 14 included — §25.6.3 says it "should always
/// be kept at 1 by software", so it is writable and reads back.
const OAR1_MASK: u32 = (1 << 15) | (1 << 14) | 0x3ff;
/// Everything `OAR2` defines.
const OAR2_MASK: u32 = 0xff;

/// `SR1` bit 0: `SB`, a START condition was generated.
const SR1_SB: u32 = 1 << 0;
/// `SR1` bit 1: `ADDR`, the address phase finished.
const SR1_ADDR: u32 = 1 << 1;
/// `SR1` bit 2: `BTF`, byte transfer finished with the data register unserved.
const SR1_BTF: u32 = 1 << 2;
/// `SR1` bit 3: `ADD10`, the 10-bit header went out.
const SR1_ADD10: u32 = 1 << 3;
/// `SR1` bit 4: `STOPF`, a STOP was detected. Slave-side; storage here.
const SR1_STOPF: u32 = 1 << 4;
/// `SR1` bit 6: `RxNE`, the data register holds a received byte.
const SR1_RXNE: u32 = 1 << 6;
/// `SR1` bit 7: `TxE`, the data register is empty.
const SR1_TXE: u32 = 1 << 7;
/// `SR1` bit 8: `BERR`, a misplaced START or STOP.
const SR1_BERR: u32 = 1 << 8;
/// `SR1` bit 9: `ARLO`, arbitration lost.
const SR1_ARLO: u32 = 1 << 9;
/// `SR1` bit 10: `AF`, acknowledge failure.
const SR1_AF: u32 = 1 << 10;
/// `SR1` bit 11: `OVR`, overrun or underrun.
const SR1_OVR: u32 = 1 << 11;
/// The `SR1` bits software clears by writing zero to them (§25.6.6's `rc_w0`),
/// which is also exactly §25.6.2's `ITERREN` list.
const SR1_ERRORS: u32 = SR1_BERR | SR1_ARLO | SR1_AF | SR1_OVR | (1 << 12) | (3 << 14);
/// The `SR1` bits that raise the event interrupt without `ITBUFEN`.
const SR1_EVENTS: u32 = SR1_SB | SR1_ADDR | SR1_ADD10 | SR1_STOPF | SR1_BTF;
/// `SR2` bit 0: `MSL`, the interface is in master mode.
const SR2_MSL: u32 = 1 << 0;
/// `SR2` bit 1: `BUSY`, a transaction is open on the bus.
const SR2_BUSY: u32 = 1 << 1;
/// `SR2` bit 2: `TRA`, data bytes are being transmitted rather than received.
const SR2_TRA: u32 = 1 << 2;

/// `CCR` bits 11:0: the clock control value (§25.6.8).
const CCR_VALUE: u32 = 0xfff;
/// `CCR` bit 14: the fast-mode duty cycle, 16/9 rather than 2.
const CCR_DUTY: u32 = 1 << 14;
/// `CCR` bit 15: fast mode rather than standard mode.
const CCR_FS: u32 = 1 << 15;
/// Everything `CCR` defines.
const CCR_MASK: u32 = CCR_VALUE | CCR_DUTY | CCR_FS;

/// Everything `TRISE` defines (§25.6.9). Storage.
const TRISE_MASK: u32 = 0x3f;
/// Everything `FLTR` defines (§25.6.10). Storage.
const FLTR_MASK: u32 = 0x1f;

/// The pin names a machine description wires.
pub mod pin {
    /// The `I2Cx_EV` interrupt output, level driven.
    pub const EV: &str = "ev";
    /// The `I2Cx_ER` interrupt output, level driven.
    pub const ER: &str = "er";
}

/// "Nothing scheduled".
const NO_EVENT: u64 = u64::MAX;

/// The rank this controller's own state takes.
///
/// Above [`crate::bus::i2c::WIRES_RANK`] because the controller calls *into* its
/// bit engine, and its neighbour [`crate::bus::i2c::FABRIC_RANK`] documents the
/// whole ladder. It is nevertheless never held across a call into the engine or
/// the fabric — every handler decides under the lock and acts outside it — so
/// the rank is belt and braces rather than the thing that makes this correct.
const STATE_RANK: LockRank = LockRank::new(0x4700);

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// Where the master engine is in a transfer (§25.3.3).
///
/// The stages that **stretch SCL** — the peripheral holding the clock down
/// until software does its part — are [`Stage::AddressWait`],
/// [`Stage::Address2Wait`], [`Stage::AddrWait`], and [`Stage::Tx`]/[`Stage::Rx`]
/// with `BTF` set. That is Figure 243's note 1, and in [`Link::Wired`] it needs
/// no code at all: the bit engine holds SCL low whenever no operation is in
/// flight, which is precisely what the note describes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Stage {
    /// Not a master. `PE` may be clear, or the bus may just be idle.
    #[default]
    Idle,
    /// A START condition is being generated.
    Starting,
    /// `SB` is set; waiting for `EV5` — read `SR1`, then write `DR`.
    AddressWait,
    /// The first address byte is going out.
    Addr1,
    /// `ADD10` is set; waiting for `EV9` — read `SR1`, then write `DR`.
    Address2Wait,
    /// The second address byte of a 10-bit address is going out.
    Addr2,
    /// `ADDR` is set; waiting for `EV6` — read `SR1`, then read `SR2`.
    AddrWait,
    /// Master transmitter.
    Tx,
    /// Master receiver.
    Rx,
    /// A STOP condition is being generated.
    Stopping,
    /// The master owns the bus with nothing in flight — after an `AF`, or after
    /// a transfer software has not ended yet. SCL stays low.
    Held,
}

/// A stable code for a stage, for the snapshot.
const fn stage_code(stage: Stage) -> u8 {
    match stage {
        Stage::Idle => 0,
        Stage::Starting => 1,
        Stage::AddressWait => 2,
        Stage::Addr1 => 3,
        Stage::Address2Wait => 4,
        Stage::Addr2 => 5,
        Stage::AddrWait => 6,
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
        2 => Stage::AddressWait,
        3 => Stage::Addr1,
        4 => Stage::Address2Wait,
        5 => Stage::Addr2,
        6 => Stage::AddrWait,
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

/// A memory-mapped STM32 I²C peripheral, `I2C v1`.
#[derive(Debug)]
pub struct Stm32I2c {
    shared: Arc<Shared>,
    region: RegionRef,
}

/// Everything both halves of the device reach.
struct Shared {
    state: Mutex<State>,
    /// How bytes reach the slaves. Fixed at construction and written down in
    /// the machine file, which is the whole point (`docs/buses/low-speed.md`).
    link: Link,
    /// The bus this controller drives in [`Link::Transactional`] mode.
    bus: Option<Arc<I2cBus>>,
    /// The bit engine it drives in [`Link::Wired`] mode.
    wires: Arc<MasterWires>,
    /// Domain ticks simulated, published for the scheduler's lock-free
    /// question. Mirrors `State::ticks`.
    ticks: AtomicU64,
    /// The tick of the next SCL half-period boundary, or [`NO_EVENT`].
    next_event: AtomicU64,
    /// The two interrupt outputs, connected at realize time.
    ev: Mutex<Option<WireSource>>,
    er: Mutex<Option<WireSource>>,
    /// The levels they are held at, so a debug read is free and the realize
    /// sweep has an answer.
    ev_level: AtomicBool,
    er_level: AtomicBool,
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
    ccr: u32,
    trise: u32,
    fltr: u32,
    /// The latched `SR1` flags.
    sr1: u32,
    /// The data register.
    dr: u8,
    /// A byte written to `DR` that the engine has not taken yet. `TxE` is its
    /// inverse while transmitting, which is what makes a driver's
    /// `while (!(SR1 & TxE));` mean something.
    tx_pending: bool,
    /// `SR2`'s `MSL`.
    msl: bool,
    /// `SR2`'s `TRA`.
    tra: bool,
    /// Where the engine is.
    stage: Stage,
    /// Which way the address phase asked for.
    dir: Direction,
    /// The 10-bit address a header started, so the repeated START that turns it
    /// into a read knows what it is addressing (UM10204 §3.1.11).
    ten: Option<u16>,
    /// Whether the master refused the last byte it read, which ends the
    /// sequential read (UM10204 §3.1.6) — no further byte may be clocked.
    rx_done: bool,
    /// Set by a **non-debug** read of `SR1`: the first half of every clearing
    /// sequence in §25.6.6.
    sr1_read: bool,
    /// The bus event in flight, mirrored here in both link models so the engine
    /// loop, the snapshot and `next_event_tick` ask one question.
    op: Option<Op>,
    /// [`Link::Transactional`] only: half periods still owed before the event
    /// takes effect.
    halves_left: u32,
    /// The tick the next half-period boundary falls on.
    next_edge: u64,
    /// Whether the half period about to elapse is the high one. It alternates,
    /// so one bit costs `Tlow + Thigh` however §25.6.8 splits the two.
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
            ccr: 0,
            // §25.6.9: `TRISE`'s reset value is 0x0002.
            trise: 2,
            fltr: 0,
            sr1: 0,
            dr: 0,
            tx_pending: false,
            msl: false,
            tra: false,
            stage: Stage::Idle,
            dir: Direction::Write,
            ten: None,
            rx_done: false,
            sr1_read: false,
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

    /// Tlow and Thigh in ticks of the peripheral clock (§25.6.8).
    ///
    /// The formulas are the reference manual's, verbatim. `CCR` is floored at
    /// one rather than at the datasheet's minimum of four: a zero half period
    /// would put the next event on the tick it was scheduled from, which the
    /// scheduler forbids, while a too-small one is only a guest programming a
    /// bus faster than the electrical specification allows — its business.
    const fn scl(&self) -> (u64, u64) {
        let ccr = (self.ccr & CCR_VALUE) as u64;
        let ccr = if ccr == 0 { 1 } else { ccr };
        if self.ccr & CCR_FS == 0 {
            // Standard mode: Thigh = Tlow = CCR * TPCLK1.
            (ccr, ccr)
        } else if self.ccr & CCR_DUTY == 0 {
            // Fast mode, DUTY = 0: Tlow = 2 * CCR, Thigh = CCR.
            (2 * ccr, ccr)
        } else {
            // Fast mode, DUTY = 1: Tlow = 16 * CCR, Thigh = 9 * CCR.
            (16 * ccr, 9 * ccr)
        }
    }

    /// How long the half period about to elapse lasts.
    const fn half_len(&self) -> u64 {
        let (low, high) = self.scl();
        if self.high_half { high } else { low }
    }

    /// `SR2` as software reads it. `busy` comes from the bus, which is not this
    /// device's state.
    const fn sr2(&self, busy: bool) -> u32 {
        let mut v = 0;
        if self.msl {
            v |= SR2_MSL;
        }
        if busy {
            v |= SR2_BUSY;
        }
        if self.tra {
            v |= SR2_TRA;
        }
        v
    }

    /// Set some `SR1` bits.
    const fn set(&mut self, bits: u32) {
        self.sr1 |= bits;
    }

    /// Clear some `SR1` bits.
    const fn clear(&mut self, bits: u32) {
        self.sr1 &= !bits;
    }

    /// Whether any of some `SR1` bits are set.
    const fn any(&self, bits: u32) -> bool {
        self.sr1 & bits != 0
    }

    /// Whether the peripheral is holding SCL low waiting for software
    /// (§25.3.3, Figure 243 note 1).
    const fn stretching(&self) -> bool {
        matches!(
            self.stage,
            Stage::AddressWait | Stage::Address2Wait | Stage::AddrWait
        ) || (matches!(self.stage, Stage::Tx | Stage::Rx) && self.any(SR1_BTF))
    }

    /// The level `I2Cx_EV` should be at (§25.6.2, `ITEVTEN`).
    const fn ev(&self) -> bool {
        if self.cr2 & CR2_ITEVTEN == 0 {
            return false;
        }
        if self.any(SR1_EVENTS) {
            return true;
        }
        self.cr2 & CR2_ITBUFEN != 0 && self.any(SR1_TXE | SR1_RXNE)
    }

    /// The level `I2Cx_ER` should be at (§25.6.2, `ITERREN`).
    const fn er(&self) -> bool {
        self.cr2 & CR2_ITERREN != 0 && self.any(SR1_ERRORS)
    }
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Stm32I2cShared");
        s.field("link", &self.link);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Stm32I2c {
    /// Validate `props` and build the peripheral.
    ///
    /// Properties:
    ///
    /// * `link` — `"transactional"` or `"wired"`. **Required**, and
    ///   deliberately so: `docs/buses/low-speed.md` asks for this choice to be
    ///   made rather than defaulted into, and `bus::spi` set the precedent.
    /// * `bus` — the name of the [`I2cBus`] to drive. Required for
    ///   `transactional`, ignored for `wired`.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for an unknown property or a missing required one,
    /// [`Error::Config`] for a `link` this module does not know or a
    /// transactional controller with no bus.
    pub fn new(props: &Props) -> Result<Stm32I2c> {
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
        Ok(Stm32I2c::with_bus(link, bus))
    }

    /// A controller on a bus the caller already holds.
    ///
    /// What [`Stm32I2c::new`] ends up calling, and the way to build one without
    /// going through the named table — an embedder that owns its own
    /// [`I2cBus`], or a test that wants a bus nothing else can reach.
    #[must_use]
    pub fn with_bus(link: Link, bus: Option<Arc<I2cBus>>) -> Stm32I2c {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(STATE_RANK, State::default()),
            link,
            bus,
            wires: Arc::new(MasterWires::new()),
            ticks: AtomicU64::new(0),
            next_event: AtomicU64::new(NO_EVENT),
            ev: Mutex::with_rank(LockRank::WIRE, None),
            er: Mutex::with_rank(LockRank::WIRE, None),
            ev_level: AtomicBool::new(false),
            er_level: AtomicBool::new(false),
            lazy: Mutex::with_rank(LockRank::WIRE, None),
        });
        let port = Arc::new(RegisterPort {
            shared: Arc::clone(&shared),
        });
        let region = Arc::new(Region::io("i2c", REGISTER_BYTES, port as Arc<dyn MemOps>));
        Stm32I2c { shared, region }
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

    /// `SR1`, without the side effect a guest read would have.
    #[must_use]
    pub fn sr1(&self) -> u32 {
        self.shared.state.lock().sr1
    }

    /// `SR2`, likewise.
    #[must_use]
    pub fn sr2(&self) -> u32 {
        let busy = self.shared.bus_busy();
        self.shared.state.lock().sr2(busy)
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

    /// Whether a transaction is open on the bus, for `SR2`'s `BUSY`.
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

    /// Re-drive both interrupt outputs from the flags.
    ///
    /// Called with no lock of ours held: driving a wire reaches an interrupt
    /// controller, which is another device (the re-entrancy contract).
    fn update_interrupts(&self) {
        let (ev, er) = {
            let state = self.state.lock();
            (state.ev(), state.er())
        };
        self.ev_level.store(ev, Ordering::Relaxed);
        self.er_level.store(er, Ordering::Relaxed);
        let ev_port = self.ev.lock().clone();
        let er_port = self.er.lock().clone();
        if let Some(port) = ev_port {
            port.set(Level::from_bool(ev));
        }
        if let Some(port) = er_port {
            port.set(Level::from_bool(er));
        }
    }

    /// Decide what the engine should do next, if anything.
    ///
    /// Returns the operation to submit. Called with the state lock held; the
    /// submission itself happens once it is released.
    fn decide(state: &mut State) -> Option<Op> {
        if state.op.is_some() || !state.enabled() {
            return None;
        }
        let start = state.cr1 & CR1_START != 0;
        let stop = state.cr1 & CR1_STOP != 0;
        match state.stage {
            // Not a master yet. §25.3.3: "Setting the START bit causes the
            // interface to generate a Start condition ... when the BUSY bit is
            // cleared." The bit engine's own START checks the line, so a busy
            // bus turns into an arbitration loss rather than a stall.
            Stage::Idle => start.then_some(Op::Start),
            // The bus is ours and nothing is moving: software decides.
            Stage::Held => {
                if stop {
                    Some(Op::Stop)
                } else if start {
                    Some(Op::Start)
                } else {
                    None
                }
            }
            // The address byte software wrote into `DR`, on its way out.
            Stage::Addr1 | Stage::Addr2 if state.tx_pending => {
                state.tx_pending = false;
                Some(Op::Write(state.dr))
            }
            Stage::Tx => {
                if state.tx_pending {
                    // The byte goes out before the STOP: §25.6.1 says a STOP is
                    // generated "after the current byte transfer", and a driver
                    // that writes the last byte and sets STOP in one breath is
                    // ordinary.
                    let byte = state.dr;
                    state.tx_pending = false;
                    state.set(SR1_TXE);
                    state.clear(SR1_BTF);
                    Some(Op::Write(byte))
                } else if stop {
                    Some(Op::Stop)
                } else if start {
                    Some(Op::Start)
                } else {
                    None
                }
            }
            Stage::Rx => {
                if stop {
                    Some(Op::Stop)
                } else if start {
                    Some(Op::Start)
                } else if state.rx_done || state.any(SR1_BTF) {
                    // Either we refused the last byte, which ends the read, or
                    // both the data register and the shift register are full —
                    // §25.6.6's `BTF`, where "the interface waits until BTF is
                    // cleared by a read in the DR register, stretching SCL low".
                    None
                } else {
                    let ack = if state.cr1 & CR1_ACK != 0 {
                        Ack::Ack
                    } else {
                        Ack::Nack
                    };
                    Some(Op::Read(ack))
                }
            }
            // Waiting for software, or an operation is already in flight.
            _ => None,
        }
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
            state.halves_left = op.halves();
            state.high_half = false;
            state.next_edge = state.ticks.saturating_add(state.half_len());
            state.stage = match op {
                Op::Start => Stage::Starting,
                Op::Stop => Stage::Stopping,
                _ => state.stage,
            };
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
    /// the identical sequence of [`I2cSlave`](crate::bus::i2c::I2cSlave) calls
    /// either way.
    fn transactional_byte(&self, byte: u8) -> Ack {
        let Some(bus) = self.bus.as_ref() else {
            return Ack::Nack;
        };
        let (stage, ten) = {
            let state = self.state.lock();
            (state.stage, state.ten)
        };
        match stage {
            Stage::Addr1 if Address::is_ten_bit_header(byte) => {
                match Direction::from_bit(byte) {
                    // The header alone (UM10204 §3.1.11's A1); the address is
                    // not complete until the second byte.
                    Direction::Write => bus.ten_bit_header((byte >> 1) & 0b11),
                    // The repeated START that turns a 10-bit write into a
                    // 10-bit read re-addresses the device the write matched.
                    Direction::Read => match ten {
                        Some(full) => bus.start(Address::Ten(full), Direction::Read),
                        None => Ack::Nack,
                    },
                }
            }
            Stage::Addr1 => bus.start(Address::seven_from_byte(byte), Direction::from_bit(byte)),
            Stage::Addr2 => {
                let high = ten.map_or(0, |t| t >> 8);
                let full = (high << 8) | u16::from(byte);
                bus.start(Address::Ten(full), Direction::Write)
            }
            _ => bus.write(byte),
        }
    }

    /// Apply what a half period produced to the register file.
    ///
    /// Returns nothing: every outward action it implies is done by the callers,
    /// which run [`Shared::pump`] and [`Shared::update_interrupts`] afterwards.
    fn apply(&self, event: MasterEvent) {
        let mut state = self.state.lock();
        match event {
            MasterEvent::Idle | MasterEvent::Working => {}
            MasterEvent::Stretched => {
                // No progress, and the half period is not counted against the
                // alternating Tlow/Thigh either: a stretched clock extends the
                // low period rather than replacing it.
                return;
            }
            MasterEvent::Started => {
                // §25.3.3: "Once the Start condition is sent: the SB bit is set
                // by hardware". The peripheral now stretches SCL until software
                // reads SR1 and writes the address into DR.
                state.op = None;
                state.msl = true;
                state.tra = false;
                state.rx_done = false;
                // §25.6.6: `TxE` and `BTF` are "cleared by hardware after a
                // start or a stop condition". `RxNE` is **not** in that list —
                // it is cleared by reading `DR`, and by nothing else short of
                // `PE = 0`.
                state.clear(SR1_TXE | SR1_BTF);
                state.set(SR1_SB);
                state.stage = Stage::AddressWait;
                // §25.6.1: START is "cleared by hardware when start is sent".
                state.cr1 &= !CR1_START;
            }
            MasterEvent::Wrote(ack) => {
                state.op = None;
                self.on_wrote(&mut state, ack);
            }
            MasterEvent::Read(byte) => {
                state.op = None;
                self.on_read(&mut state, byte);
            }
            MasterEvent::Stopped => {
                state.op = None;
                // §25.6.7: `MSL` is "cleared by hardware after detecting a Stop
                // condition"; §25.6.6: `TxE` and `BTF` are "cleared by hardware
                // ... after a start or a stop condition".
                state.msl = false;
                state.tra = false;
                state.rx_done = false;
                state.ten = None;
                state.tx_pending = false;
                // **`RxNE` survives the STOP.** §25.6.6 clears `TxE` and `BTF`
                // "after a start or a stop condition" and says nothing of the
                // kind about `RxNE`, which is cleared by reading `DR`. It has
                // to survive: a master receiver programs the STOP *before* the
                // last byte arrives (§25.3.3, "software must set the
                // STOP/START bit after reading the second last data byte"), so
                // clearing it here would throw away the byte the whole
                // transfer was for. Firmware that reads eight bytes gets seven
                // and hangs, which is exactly what `tests/stm32f407_i2c.rs`
                // caught and no unit test did.
                state.clear(SR1_TXE | SR1_BTF | SR1_SB | SR1_ADDR | SR1_ADD10);
                state.cr1 &= !CR1_STOP;
                state.stage = Stage::Idle;
            }
            MasterEvent::ArbitrationLost => {
                // §25.6.6: "After an ARLO event the interface switches back
                // automatically to Slave mode (M/SL=0)."
                state.op = None;
                state.set(SR1_ARLO);
                state.msl = false;
                state.tra = false;
                state.tx_pending = false;
                state.clear(SR1_TXE | SR1_BTF | SR1_SB | SR1_ADDR | SR1_ADD10);
                state.cr1 &= !(CR1_START | CR1_STOP);
                state.stage = Stage::Idle;
            }
        }
        // The half period alternates only when one actually elapsed.
        state.high_half = !state.high_half;
        self.publish(&state);
    }

    /// A byte went out and the ninth clock came back.
    fn on_wrote(&self, state: &mut State, ack: Ack) {
        if !ack.is_ack() {
            // §25.6.6: `AF` is "set by hardware when no acknowledge is
            // returned", and "ADDR is not set after a NACK reception". The
            // master keeps the bus; software must send a STOP.
            state.set(SR1_AF);
            state.stage = Stage::Held;
            return;
        }
        match state.stage {
            Stage::Addr1 => {
                let byte = state.dr;
                if Address::is_ten_bit_header(byte) && Direction::from_bit(byte) == Direction::Write
                {
                    // §25.3.3: "In 10-bit addressing mode, sending the header
                    // sequence causes ... the ADD10 bit is set by hardware."
                    state.ten = Some(u16::from((byte >> 1) & 0b11) << 8);
                    state.set(SR1_ADD10);
                    state.stage = Stage::Address2Wait;
                } else {
                    self.address_done(state);
                }
            }
            Stage::Addr2 => {
                let low = state.dr;
                state.ten = state.ten.map(|t| (t & 0x300) | u16::from(low));
                self.address_done(state);
            }
            Stage::Tx => {
                // §25.3.3: "When the acknowledge pulse is received, the TxE bit
                // is set by hardware", and if nothing was written meanwhile,
                // "BTF is set and the interface waits until BTF is cleared by a
                // write to I2C_DR, stretching SCL low."
                state.set(SR1_TXE);
                if !state.tx_pending {
                    state.set(SR1_BTF);
                }
            }
            _ => {}
        }
    }

    /// The address phase finished: `ADDR`, and a stretch until `EV6`.
    fn address_done(&self, state: &mut State) {
        // §25.6.7: `TRA` "is set depending on the R/W bit of the address byte,
        // at the end of total address phase".
        state.tra = state.dir == Direction::Write;
        state.set(SR1_ADDR);
        state.stage = Stage::AddrWait;
    }

    /// A byte came in.
    fn on_read(&self, state: &mut State, byte: u8) {
        if state.any(SR1_RXNE) {
            // §25.6.6: `BTF` is set "in reception when a new byte is received
            // (including ACK pulse) and DR has not been read yet (RxNE=1)".
            state.set(SR1_BTF);
        }
        state.dr = byte;
        state.set(SR1_RXNE);
        if state.cr1 & CR1_ACK == 0 {
            // We refused it, so the slave has stopped transmitting.
            state.rx_done = true;
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
            self.update_interrupts();
        }
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
    /// `SWRST`: abandon whatever is in flight and let both lines go.
    Reset,
    /// `ACK` moved while a byte is being clocked in; the acknowledge this model
    /// will drive on the ninth clock has to follow it.
    ReadAck(Ack),
}

/// The memory-mapped registers (§25.6).
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
    /// `debug` suppresses **every** side effect, and on this peripheral that is
    /// not a nicety: §25.6.6's flags are cleared by *reading* `SR1` and then
    /// reading `SR2` or touching `DR`, so a debugger that dumped the block
    /// would clear `ADDR` out from under the guest and hang its driver.
    fn read_register(&self, offset: u64, debug: bool, busy: bool) -> (u32, After) {
        let mut state = self.shared.state.lock();
        match offset {
            0x00 => (state.cr1, After::Nothing),
            0x04 => (state.cr2, After::Nothing),
            0x08 => (state.oar1, After::Nothing),
            0x0c => (state.oar2, After::Nothing),
            0x10 => {
                let value = u32::from(state.dr);
                if debug {
                    return (value, After::Nothing);
                }
                // §25.6.6: `RxNE` is "cleared by software reading ... the DR
                // register", and `BTF` "by either a read or write in the DR
                // register" — which is also what releases the stretch.
                state.clear(SR1_RXNE | SR1_BTF);
                state.sr1_read = false;
                (value, After::Pump)
            }
            0x14 => {
                if !debug {
                    // The first half of every clearing sequence in §25.6.6.
                    state.sr1_read = true;
                }
                (state.sr1, After::Nothing)
            }
            0x18 => {
                let value = state.sr2(busy);
                if debug || !state.sr1_read || !state.any(SR1_ADDR) {
                    return (value, After::Nothing);
                }
                // §25.6.6: `ADDR` "is cleared by software reading SR1 register
                // followed reading SR2". §25.6.7 adds that this happens "even if
                // the ADDR flag was set after reading I2C_SR1", which is why the
                // arming latch is not cleared by anything in between.
                state.clear(SR1_ADDR);
                state.sr1_read = false;
                if state.stage == Stage::AddrWait {
                    state.stage = if state.tra { Stage::Tx } else { Stage::Rx };
                    if state.tra {
                        // §25.3.3, `EV8_1`: the data register is empty and the
                        // master waits for the first byte.
                        state.set(SR1_TXE);
                    }
                }
                (value, After::Pump)
            }
            0x1c => (state.ccr, After::Nothing),
            0x20 => (state.trise, After::Nothing),
            0x24 => (state.fltr, After::Nothing),
            _ => (0, After::Nothing),
        }
    }

    /// Write one register, reporting what has to happen once the lock is
    /// released.
    fn write_register(&self, offset: u64, value: u32) -> After {
        let mut state = self.shared.state.lock();
        match offset {
            0x00 => {
                if value & CR1_SWRST != 0 {
                    // §25.6.1: "When set, the I2C is under reset state."
                    let ticks = state.ticks;
                    *state = State {
                        ticks,
                        cr1: value & CR1_MASK,
                        ..State::default()
                    };
                    return After::Reset;
                }
                let had_ack = state.cr1 & CR1_ACK != 0;
                let arm = state.sr1_read;
                state.cr1 = value & CR1_MASK;
                if arm && state.any(SR1_STOPF) {
                    // §25.6.6: `STOPF` is "cleared by software reading the SR1
                    // register followed by a write in the CR1 register".
                    state.clear(SR1_STOPF);
                    state.sr1_read = false;
                }
                if !state.enabled() && state.op.is_none() {
                    // §25.6.1: "the peripheral is disabled at the end of the
                    // current communication, when back to IDLE state. All bit
                    // resets due to PE=0 occur at the end of the communication."
                    state.sr1 = 0;
                    state.msl = false;
                    state.tra = false;
                    state.tx_pending = false;
                    state.stage = Stage::Idle;
                }
                let now_ack = state.cr1 & CR1_ACK != 0;
                if had_ack != now_ack && matches!(state.op, Some(Op::Read(_))) {
                    // The acknowledge belongs to the byte in the shift register
                    // (§25.6.1's `POS` = 0), so software clearing `ACK` after
                    // the second-last `RxNE` really does NACK the last byte.
                    let ack = if now_ack { Ack::Ack } else { Ack::Nack };
                    state.op = Some(Op::Read(ack));
                    return After::ReadAck(ack);
                }
                After::Pump
            }
            0x04 => {
                state.cr2 = value & CR2_MASK;
                After::Pump
            }
            0x08 => {
                state.oar1 = value & OAR1_MASK;
                After::Nothing
            }
            0x0c => {
                state.oar2 = value & OAR2_MASK;
                After::Nothing
            }
            0x10 => {
                let byte = value as u8;
                match state.stage {
                    Stage::AddressWait => {
                        // §25.3.3's `EV5`: "cleared by reading SR1 register
                        // followed by writing DR register with Address".
                        if state.sr1_read {
                            state.clear(SR1_SB);
                        }
                        state.sr1_read = false;
                        state.dr = byte;
                        state.dir = Direction::from_bit(byte);
                        state.tx_pending = true;
                        state.stage = Stage::Addr1;
                    }
                    Stage::Address2Wait => {
                        // `EV9`, the second byte of a 10-bit address.
                        if state.sr1_read {
                            state.clear(SR1_ADD10);
                        }
                        state.sr1_read = false;
                        state.dr = byte;
                        state.tx_pending = true;
                        state.stage = Stage::Addr2;
                    }
                    _ => {
                        state.dr = byte;
                        state.tx_pending = true;
                        state.clear(SR1_TXE | SR1_BTF);
                        state.sr1_read = false;
                    }
                }
                After::Pump
            }
            0x14 => {
                // §25.6.6's `rc_w0`: a zero written to an error bit clears it,
                // a one leaves it. Every other bit is read-only.
                state.sr1 &= value | !SR1_ERRORS;
                After::Pump
            }
            // `SR2` is read-only.
            0x18 => After::Nothing,
            0x1c => {
                state.ccr = value & CCR_MASK;
                After::Nothing
            }
            0x20 => {
                state.trise = value & TRISE_MASK;
                After::Nothing
            }
            0x24 => {
                state.fltr = value & FLTR_MASK;
                After::Nothing
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
                self.shared.update_interrupts();
            }
            After::Reset => {
                self.shared.wires.reset();
                self.shared.update_interrupts();
            }
            After::ReadAck(ack) => {
                if self.shared.link == Link::Wired {
                    self.shared.wires.set_read_ack(ack);
                }
                self.shared.pump();
                self.shared.update_interrupts();
            }
        }
    }
}

impl MemOps for RegisterPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        // §25.6: "The peripheral registers can be accessed by half-words
        // (16 bits) or words (32 bits)."
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

impl Device for Stm32I2c {
    fn class(&self) -> &'static DeviceClass {
        &ST_I2C_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and `wire`
        // statements connect the lines.
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
        self.shared.update_interrupts();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.shared.state.lock();
        w.write_u64(state.ticks)?;
        w.write_u32(state.cr1)?;
        w.write_u32(state.cr2)?;
        w.write_u32(state.oar1)?;
        w.write_u32(state.oar2)?;
        w.write_u32(state.ccr)?;
        w.write_u32(state.trise)?;
        w.write_u32(state.fltr)?;
        w.write_u32(state.sr1)?;
        w.write_u8(state.dr)?;
        w.write_bool(state.tx_pending)?;
        w.write_bool(state.msl)?;
        w.write_bool(state.tra)?;
        w.write_u8(stage_code(state.stage))?;
        w.write_bool(state.dir == Direction::Read)?;
        // Both halves are always written, so both are always read.
        w.write_bool(state.ten.is_some())?;
        w.write_u16(state.ten.unwrap_or(0))?;
        w.write_bool(state.rx_done)?;
        // The arming latch is architectural: a snapshot taken between the read
        // of `SR1` and the read of `SR2` has to resume *inside* that sequence,
        // or the guest's next `SR2` read stops clearing `ADDR`.
        w.write_bool(state.sr1_read)?;
        let (op, operand) = state.op.map_or((0, 0), Op::code);
        w.write_u8(op)?;
        w.write_u8(operand)?;
        w.write_u32(state.halves_left)?;
        w.write_u64(state.next_edge)?;
        w.write_bool(state.high_half)?;
        self.shared.wires.snapshot().write(w)
        // The interrupt outputs are not saved: they are a pure function of the
        // flags, and `load` re-derives and re-announces them.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let state = State {
            ticks: r.read_u64()?,
            cr1: r.read_u32()?,
            cr2: r.read_u32()?,
            oar1: r.read_u32()?,
            oar2: r.read_u32()?,
            ccr: r.read_u32()?,
            trise: r.read_u32()?,
            fltr: r.read_u32()?,
            sr1: r.read_u32()?,
            dr: r.read_u8()?,
            tx_pending: r.read_bool()?,
            msl: r.read_bool()?,
            tra: r.read_bool()?,
            stage: stage_from_code(r.read_u8()?),
            dir: if r.read_bool()? {
                Direction::Read
            } else {
                Direction::Write
            },
            ten: {
                let has = r.read_bool()?;
                let value = r.read_u16()?;
                has.then_some(value)
            },
            rx_done: r.read_bool()?,
            sr1_read: r.read_bool()?,
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
        self.shared.update_interrupts();
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
            _ => {
                return Err(Error::Config {
                    at: String::from(port),
                    message: alloc::format!(
                        "an STM32 I2C drives `{}` and `{}` — both open drain, and only ever low — \
                         plus the interrupt outputs `{}` and `{}`",
                        line::SCL_NAME,
                        line::SDA_NAME,
                        pin::EV,
                        pin::ER
                    ),
                });
            }
        }
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.wires.announce();
        self.shared.update_interrupts();
    }

    // -- lazily advanced (`ROADMAP.md` §4.2) ---------------------------------

    /// Yes. A transfer takes real time, a guest polls `SR1` to find out how far
    /// it has got, and the answer has to be the one at the cycle of the poll.
    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.shared.ticks.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        Stm32I2c::advance_to(self, tick);
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

impl Instance for Stm32I2c {}

/// The `st.i2c` device class.
pub static ST_I2C_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32 I2C v1 (F1/F2/F4/L1): master mode, 7- and 10-bit addressing, \
              CCR clocking, EV/ER interrupts, transactional or wired",
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
    construct: |props| Ok(Box::new(Stm32I2c::new(props)?)),
};

/// Add [`ST_I2C_CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&ST_I2C_CLASS)
}

/// Bind [`ST_I2C_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Stm32I2c::new(props)?)))
}

/// What the validator should know about `st.i2c`.
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
        .region("")
        .region("regs")
}
