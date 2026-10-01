//! One R-Car I²C controller, as a master on a named [`I2cBus`].
//!
//! # Register map
//!
//! All accesses are 32-bit. The names are descriptive; the controller's own
//! slave side is present only as storage.
//!
//! | Offset | Name | Here |
//! | --- | --- | --- |
//! | `0x00` | slave control | stored |
//! | `0x04` | `MCR`, master control | bit 0 requests a START, bit 1 a STOP (and a NACK for a byte being received), bits 3 and 7 enable; 0 aborts. Bit 5, bus busy, always reads 0 |
//! | `0x08` | slave status | stored |
//! | `0x0c` | `MSR`, master status | below; a write of 0 clears a bit, a 1 leaves it |
//! | `0x10` | slave interrupt enables | stored |
//! | `0x14` | `MIER`, master interrupt enables | same layout as `MSR` |
//! | `0x18` | clock control | stored |
//! | `0x1c` | slave address | stored |
//! | `0x20` | `MAR`, master address | `address << 1 | R/W̅` |
//! | `0x24` | data | write: the next byte to send; read: the last byte received |
//! | `0x28`–`0x34` | clock extras | stored |
//!
//! `MSR`: bit 0 `MAT` (the address was acknowledged), 1 `MDR` (a byte was
//! received), 2 `MDT` (a byte was sent), 3 `MDE` (ready for the next byte),
//! 4 `MST` (a STOP went out), 5 `MAL` (arbitration lost; never, with one
//! master), 6 `MNR` (the address or a byte was not acknowledged).
//!
//! # The sequence
//!
//! Transactional: nothing is clocked, and each step happens on the register
//! write the guest's driver uses to release it.
//!
//! * A START goes out on the `MSR` write that clears `MDE` while `MCR`
//!   requests one — the first message and a repeated START alike. The address
//!   either earns `MAT` or `MNR`; neither raises `MDE` or `MDR`.
//! * Writing: the byte in the data register goes out on the `MSR` write that
//!   clears `MDE` (and, straight after the address, `MAT`). An acknowledged
//!   byte raises `MDE`, or — with a STOP requested — sends the STOP and
//!   raises `MST`. A STOP requested with nothing left to send goes out at
//!   once.
//! * Reading: the first byte is clocked on the write that clears `MAT`, each
//!   later one on the write that clears `MDR`. A byte clocked while a STOP is
//!   requested is not acknowledged, and the write that clears its `MDR` sends
//!   the STOP — never in the same status as that byte's `MDR`, which a driver
//!   testing `MST` first would lose.
//! * A refused address or byte raises `MNR` and waits; the driver's STOP
//!   request then sends it and raises `MST`, leaving `MNR` set.
//!
//! The interrupt is the level `MSR & MIER`.
//!
//! # Sources
//!
//! No manual: the behaviour is what the Alphard navi's own kernel images,
//! disassembled as data, require of it — the values its driver writes, the
//! flags its interrupt handler tests and in which order, and the write-0-to-
//! clear reading its fixed masks only make sense under. The bus protocol is
//! UM10204's, through [`crate::bus::i2c`].

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::bus::i2c::{Ack, Address, Direction, I2cBus, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.i2c";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space one controller answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x38;

/// The controller's interrupt output.
pub const IRQ_PIN: &str = "irq";

const MCR: u64 = 0x04;
const MSR: u64 = 0x0c;
const MIER: u64 = 0x14;
const MAR: u64 = 0x20;
const DATA: u64 = 0x24;

const MCR_ESG: u32 = 1 << 0;
const MCR_FSB: u32 = 1 << 1;

const MAT: u32 = 1 << 0;
const MDR: u32 = 1 << 1;
const MDT: u32 = 1 << 2;
const MDE: u32 = 1 << 3;
const MST: u32 = 1 << 4;
const MNR: u32 = 1 << 6;
const MSR_MASK: u32 = 0x7f;

/// Where the master is in a transaction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
enum Phase {
    /// No transaction.
    #[default]
    Idle,
    /// The address was acknowledged; the first byte has not moved.
    AddrDone,
    /// Writing: a byte went out and was acknowledged. Also the bus held
    /// after a message that ended without a STOP.
    TxWait,
    /// Reading: a byte is in the data register.
    RxWait,
    /// The address or a byte was refused; waiting for the STOP request.
    NackWait,
}

impl Phase {
    fn code(self) -> u8 {
        match self {
            Phase::Idle => 0,
            Phase::AddrDone => 1,
            Phase::TxWait => 2,
            Phase::RxWait => 3,
            Phase::NackWait => 4,
        }
    }

    fn from_code(c: u8) -> Option<Phase> {
        Some(match c {
            0 => Phase::Idle,
            1 => Phase::AddrDone,
            2 => Phase::TxWait,
            3 => Phase::RxWait,
            4 => Phase::NackWait,
            _ => return None,
        })
    }
}

/// The guest-visible state.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct State {
    /// Every register as stored, by word index (`0x00`–`0x34`).
    regs: [u32; 14],
    phase: Phase,
    /// The direction of the transaction in progress.
    read: bool,
    /// A byte written to the data register and not yet sent.
    tx_pending: bool,
    /// The last byte received was not acknowledged.
    nack_last: bool,
    /// The last byte received.
    rx: u8,
}

impl State {
    fn reg(&self, offset: u64) -> u32 {
        self.regs[(offset / 4) as usize]
    }

    fn set(&mut self, offset: u64, v: u32) {
        self.regs[(offset / 4) as usize] = v;
    }

    fn mcr(&self) -> u32 {
        self.reg(MCR)
    }

    fn raise(&mut self, bits: u32) {
        let v = self.reg(MSR) | bits;
        self.set(MSR, v);
    }

    fn irq(&self) -> bool {
        self.reg(MSR) & self.reg(MIER) & MSR_MASK != 0
    }
}

/// What a register write asks of the bus, decided under the state lock and
/// carried out after it is released.
enum Step {
    None,
    Start(Address, Direction),
    Send(u8, bool),
    Receive(bool),
    Stop,
    Abort,
}

struct Registers {
    bus: Option<Arc<I2cBus>>,
    state: Mutex<State>,
    irq: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Registers").finish_non_exhaustive()
    }
}

impl Registers {
    fn refresh(&self) {
        let irq = self.state.lock().irq();
        let out = self.irq.lock().clone();
        if let Some(out) = out {
            out.set(Level::from_bool(irq));
        }
    }

    /// Decide what an `MSR` write of `v` sets in motion. `MSR` itself has
    /// already been updated.
    fn after_msr(s: &mut State, v: u32) -> Step {
        let mcr = s.mcr();
        let clears_mde = v & MDE == 0;
        if clears_mde && mcr & MCR_ESG != 0 && matches!(s.phase, Phase::Idle | Phase::TxWait) {
            let mar = s.reg(MAR);
            let Some(address) = Address::seven(((mar >> 1) & 0x7f) as u8) else {
                return Step::None;
            };
            return Step::Start(address, Direction::from_bit(mar as u8));
        }
        match s.phase {
            Phase::AddrDone if !s.read && clears_mde && v & MAT == 0 => Self::tx_step(s),
            Phase::TxWait if clears_mde => Self::tx_step(s),
            Phase::AddrDone if s.read && v & MAT == 0 => Step::Receive(mcr & MCR_FSB != 0),
            Phase::RxWait if v & MDR == 0 => {
                if s.nack_last {
                    Step::Stop
                } else {
                    Step::Receive(mcr & MCR_FSB != 0)
                }
            }
            _ => Step::None,
        }
    }

    fn tx_step(s: &mut State) -> Step {
        if s.tx_pending {
            s.tx_pending = false;
            Step::Send(s.reg(DATA) as u8, s.mcr() & MCR_FSB != 0)
        } else if s.mcr() & MCR_FSB != 0 {
            Step::Stop
        } else {
            Step::None
        }
    }

    /// Carry out `step` on the bus, with no lock held, and record what came
    /// of it.
    fn run(&self, step: Step) {
        let bus = self.bus.as_ref();
        match step {
            Step::None => {}
            Step::Start(address, dir) => {
                let ack = bus.map_or(Ack::Nack, |b| b.start(address, dir));
                let mut s = self.state.lock();
                s.read = dir == Direction::Read;
                s.nack_last = false;
                if ack == Ack::Ack {
                    s.raise(MAT);
                    s.phase = Phase::AddrDone;
                } else {
                    s.raise(MNR);
                    s.phase = Phase::NackWait;
                }
            }
            Step::Send(byte, stop) => {
                let ack = bus.map_or(Ack::Nack, |b| b.write(byte));
                if ack == Ack::Ack
                    && stop
                    && let Some(b) = bus
                {
                    b.stop();
                }
                let mut s = self.state.lock();
                if ack != Ack::Ack {
                    s.raise(MNR);
                    s.phase = Phase::NackWait;
                } else if stop {
                    s.raise(MDT | MST);
                    s.phase = Phase::Idle;
                } else {
                    s.raise(MDT | MDE);
                    s.phase = Phase::TxWait;
                }
            }
            Step::Receive(last) => {
                let ack = if last { Ack::Nack } else { Ack::Ack };
                let byte = bus.map_or(0xff, |b| b.read(ack));
                let mut s = self.state.lock();
                s.rx = byte;
                s.nack_last = last;
                s.raise(MDR);
                s.phase = Phase::RxWait;
            }
            Step::Stop => {
                if let Some(b) = bus {
                    b.stop();
                }
                let mut s = self.state.lock();
                s.raise(MST);
                s.phase = Phase::Idle;
                s.nack_last = false;
            }
            Step::Abort => {
                if let Some(b) = bus {
                    b.stop();
                }
            }
        }
    }

    fn write_reg(&self, offset: u64, v: u32) {
        let step = {
            let mut s = self.state.lock();
            match offset {
                MCR => {
                    s.set(MCR, v & 0xff);
                    if v == 0 {
                        let busy = s.phase != Phase::Idle;
                        s.phase = Phase::Idle;
                        s.tx_pending = false;
                        s.nack_last = false;
                        if busy { Step::Abort } else { Step::None }
                    } else if v & MCR_FSB != 0
                        && (s.phase == Phase::NackWait
                            || (s.phase == Phase::TxWait && !s.tx_pending))
                    {
                        Step::Stop
                    } else {
                        Step::None
                    }
                }
                MSR => {
                    let now = s.reg(MSR) & v & MSR_MASK;
                    s.set(MSR, now);
                    Self::after_msr(&mut s, v)
                }
                DATA => {
                    s.set(DATA, v & 0xff);
                    s.tx_pending = true;
                    Step::None
                }
                MIER => {
                    s.set(MIER, v & MSR_MASK);
                    Step::None
                }
                _ if offset < REGISTER_WINDOW_LEN => {
                    s.set(offset, v);
                    Step::None
                }
                _ => Step::None,
            }
        };
        self.run(step);
    }

    fn read_reg(&self, offset: u64) -> u32 {
        let s = self.state.lock();
        match offset {
            // Bit 5 is the bus busy: never, with one master on the bus.
            MCR => s.mcr() & !(1 << 5),
            DATA => u32::from(s.rx),
            _ if offset < REGISTER_WINDOW_LEN => s.reg(offset),
            _ => 0,
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        // Reads have no side effects, so a debugger's is the guest's.
        let v = self.read_reg(offset & !3);
        dst.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        let v = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        self.write_reg(offset & !3, v);
        self.refresh();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

/// One I²C controller.
#[derive(Debug)]
pub struct I2c {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl I2c {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a malformed or unknown property.
    pub fn new(props: &Props) -> Result<I2c> {
        let mut r = props.reader();
        let bus = r.optional_str("bus")?.map(ToString::to_string);
        r.finish()?;
        let bus = match bus {
            Some(name) => Some(buses::attach(props, &name)?),
            None => None,
        };
        Ok(I2c::with_bus(bus))
    }

    /// A controller mastering `bus`.
    #[must_use]
    pub fn with_bus(bus: Option<Arc<I2cBus>>) -> I2c {
        let regs = Arc::new(Registers {
            bus,
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            irq: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            "rcar.i2c",
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        I2c { regs, region }
    }

    /// Whether the interrupt output is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.regs.state.lock().irq()
    }
}

/// The `rcar.i2c` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "one R-Car I2C controller: a master on a named I2C bus, interrupt-driven",
    properties: &[PropertySpec {
        name: "bus",
        kind: ValueKind::Str,
        required: false,
        summary: "the I2C bus this controller masters, by name (none: every address is refused)",
    }],
    construct: |props| Ok(Box::new(I2c::new(props)?)),
};

impl Device for I2c {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.regs.state.lock() = State::default();
        self.regs.refresh();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("an I2C controller drives one pin, `irq`"),
            });
        }
        *self.regs.irq.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.regs.refresh();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.regs.state.lock().clone();
        for v in s.regs {
            w.write_u32(v)?;
        }
        w.write_u8(s.phase.code())?;
        w.write_bool(s.read)?;
        w.write_bool(s.tx_pending)?;
        w.write_bool(s.nack_last)?;
        w.write_u8(s.rx)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut regs = [0u32; 14];
        for v in &mut regs {
            *v = r.read_u32()?;
        }
        let phase = Phase::from_code(r.read_u8()?)
            .ok_or_else(|| Error::State(String::from("an unknown I2C master phase")))?;
        let s = State {
            regs,
            phase,
            read: r.read_bool()?,
            tx_pending: r.read_bool()?,
            nack_last: r.read_bool()?,
            rx: r.read_u8()?,
        };
        *self.regs.state.lock() = s;
        self.regs.refresh();
        Ok(())
    }
}

impl Instance for I2c {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(I2c::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str))
        .region("")
        .region("regs")
        .port(IRQ_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::i2c::I2cSlave;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    /// A target at 0x50: a byte register file with an auto-incrementing
    /// pointer, the first byte of a write setting it.
    #[derive(Debug)]
    struct Target {
        inner: Mutex<(u8, bool, [u8; 256])>,
    }

    impl Default for Target {
        fn default() -> Target {
            Target {
                inner: Mutex::new((0, false, [0; 256])),
            }
        }
    }

    impl I2cSlave for Target {
        fn address(&self, address: Address, _dir: Direction) -> Ack {
            let mut g = self.inner.lock();
            g.1 = true;
            if address == Address::Seven(0x50) {
                Ack::Ack
            } else {
                Ack::Nack
            }
        }
        fn write(&self, byte: u8) -> Ack {
            let mut g = self.inner.lock();
            if g.1 {
                g.0 = byte;
                g.1 = false;
            } else {
                let p = g.0;
                g.2[p as usize] = byte;
                g.0 = p.wrapping_add(1);
            }
            Ack::Ack
        }
        fn read(&self) -> u8 {
            let g = self.inner.lock();
            g.2[g.0 as usize]
        }
        fn read_ack(&self, _ack: Ack) {
            let mut g = self.inner.lock();
            g.0 = g.0.wrapping_add(1);
        }
        fn stop(&self) {}
    }

    fn controller() -> (I2c, Arc<Target>) {
        let bus = Arc::new(I2cBus::new());
        let t = Arc::new(Target::default());
        bus.attach(Arc::clone(&t) as Arc<dyn I2cSlave>).unwrap();
        (I2c::with_bus(Some(bus)), t)
    }

    fn w(c: &I2c, off: u64, v: u32) {
        c.regs
            .write(off, &v.to_le_bytes(), MemAttrs::DEFAULT)
            .unwrap();
    }

    fn r(c: &I2c, off: u64) -> u32 {
        let mut b = [0u8; 4];
        c.regs.read(off, &mut b, MemAttrs::DEFAULT).unwrap();
        u32::from_le_bytes(b)
    }

    /// The navi driver's interrupt handler, for a single message ending in
    /// a STOP: returns the bytes read and every status bit it saw.
    fn driver(c: &I2c, addr: u8, read: bool, out: &[u8], len: usize) -> (alloc::vec::Vec<u8>, u32) {
        let mut buf = alloc::vec::Vec::new();
        let mut remaining = len;
        let mut index = 0;
        w(c, MAR, u32::from(addr << 1) | u32::from(read));
        if !read {
            w(c, DATA, u32::from(out[0]));
            remaining -= 1;
            index = 1;
        }
        w(c, MSR, 0x0a);
        w(c, MCR, 0x89);
        w(c, MSR, 0x75);
        w(c, MIER, if read { 0x73 } else { 0x79 });
        let mut nack = false;
        let mut seen = 0;
        for _ in 0..100 {
            if !c.irq_asserted() {
                break;
            }
            let mut st = r(c, MSR) & 0x7f;
            seen |= st;
            if st & MNR != 0 {
                w(c, MCR, 0x8a);
                w(c, MIER, 0x10);
                st &= 0x7e;
                nack = true;
            }
            if st & MST != 0 {
                w(c, MSR, st & !MST);
                break;
            }
            if nack {
                continue;
            }
            if !read {
                if st & MAT != 0 {
                    w(c, MCR, 0x88);
                    if remaining == 0 {
                        w(c, MCR, 0x8a);
                    }
                    w(c, MSR, st & !0x0d);
                    st = r(c, MSR);
                }
                if st & MDE != 0 {
                    if remaining > 0 {
                        w(c, DATA, u32::from(out[index]));
                        index += 1;
                        remaining -= 1;
                        w(c, MSR, 0x77);
                    } else {
                        w(c, MCR, 0x8a);
                        w(c, MSR, 0x77);
                    }
                }
            } else {
                if st & MAT != 0 {
                    w(c, MCR, 0x88);
                    if remaining <= 1 {
                        w(c, MCR, 0x8a);
                    }
                    w(c, MSR, 0x70);
                    st = r(c, MSR);
                }
                if st & MDR != 0 {
                    if remaining > 0 {
                        buf.push(r(c, DATA) as u8);
                        remaining -= 1;
                        if remaining <= 1 {
                            w(c, MCR, 0x8a);
                        }
                    }
                    w(c, MSR, 0x7d);
                }
            }
        }
        let status = seen;
        w(c, MCR, 0);
        w(c, MSR, 0);
        w(c, MIER, 0);
        (buf, status)
    }

    #[test]
    fn a_write_then_a_read_round_trip_through_the_target() {
        let (c, t) = controller();
        let (_, st) = driver(&c, 0x50, false, &[0x10, 0xaa, 0xbb, 0xcc], 4);
        assert_ne!(st & MST, 0);
        assert_eq!(st & MNR, 0);
        assert_eq!(&t.inner.lock().2[0x10..0x13], &[0xaa, 0xbb, 0xcc]);
        // Point back at 0x10 and read three.
        driver(&c, 0x50, false, &[0x10], 1);
        let (got, st) = driver(&c, 0x50, true, &[], 3);
        assert_eq!(got, [0xaa, 0xbb, 0xcc]);
        assert_ne!(st & MST, 0);
        assert!(!c.irq_asserted(), "nothing raised after the cleanup");
    }

    #[test]
    fn a_single_byte_read_is_not_acknowledged_and_still_stops() {
        let (c, t) = controller();
        t.inner.lock().2[0] = 0x5a;
        let (got, st) = driver(&c, 0x50, true, &[], 1);
        assert_eq!(got, [0x5a]);
        assert_ne!(st & MST, 0);
    }

    #[test]
    fn an_absent_address_is_refused_and_the_stop_still_goes_out() {
        let (c, _) = controller();
        let (_, st) = driver(&c, 0x23, false, &[0x00], 1);
        assert_ne!(st & MNR, 0, "MNR stays set");
        assert_ne!(st & MST, 0);
    }

    #[test]
    fn the_bus_never_reads_busy() {
        let (c, _) = controller();
        w(&c, MCR, 0xff);
        assert_eq!(r(&c, MCR) & (1 << 5), 0);
    }

    #[test]
    fn status_bits_clear_by_writing_zero() {
        let (c, _) = controller();
        c.regs.state.lock().set(MSR, 0x7f);
        w(&c, MSR, 0x75);
        assert_eq!(r(&c, MSR), 0x75);
    }

    #[test]
    fn a_snapshot_round_trips() {
        let (c, _) = controller();
        w(&c, MAR, 0x50 << 1);
        w(&c, DATA, 0x42);
        w(&c, MSR, 0x0a);
        w(&c, MCR, 0x89);
        w(&c, MSR, 0x75);
        w(&c, MIER, 0x79);
        let save = |c: &I2c| {
            let mut shape = MachineShape::new();
            shape.add_device("i2c", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("i2c", CLASS.name, CLASS.version).unwrap();
                c.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&c);
        let (back, _) = controller();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("i2c", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let want = c.regs.state.lock().clone();
        assert_eq!(*back.regs.state.lock(), want);
        assert_eq!(save(&back), bytes);
    }
}
