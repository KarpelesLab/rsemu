//! An AKM AK77xx-family audio DSP on I²C, as far as its host sees it: the
//! command set that loads its coefficient and program RAMs, checks them with a
//! CRC, and reads and writes its control registers. The signal processing is
//! not modelled.
//!
//! # Commands
//!
//! A write transaction is one command byte and its operands; a read
//! transaction returns the answer to the last command written, across a STOP.
//!
//! | Command | Operands | Here |
//! | --- | --- | --- |
//! | `0xc0 + r` | data | write control register `r` (0–31) |
//! | `0x40 + r` | — | read control register `r` |
//! | `0xb4` | address (2), data | write coefficient RAM, 3-byte words from `address` |
//! | `0xb8` | address (2), data | write program RAM, 5-byte words from `address` |
//! | `0x72` | — | read the CRC of the last RAM write, high byte first |
//! | `0x34` | address (2) | read coefficient RAM from `address` |
//! | `0x80` | address (2), data (3) | stage one coefficient for a run-time update |
//! | `0x24` | — | read the staged update back: address, then data |
//! | `0xa4` | `00 00` | apply the staged update to coefficient RAM |
//! | `0x60` | — | read the device ID |
//!
//! The CRC is CRC-16/XMODEM (polynomial `0x1021`, initial value 0) over every
//! byte of the RAM write, the command and address included. Control registers
//! hold all eight bits and start at 0. Anything else is acknowledged and
//! ignored.
//!
//! # Sources
//!
//! The AKM AK7755 datasheet's serial interface chapter (the command set and
//! CRC check), and the navi's own echo-canceller library, disassembled as
//! data: its bring-up writes register 0x0f and reads it back, downloads its
//! coefficient and program images and compares the `0x72` answer with the
//! CRC above. The navi's part is an AK77xx whose register meanings differ
//! from the AK7755's in places; the commands do not.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::i2c::{Ack, Address, Direction, I2cBus, I2cSlave, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "akm.ak77xx";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// Control registers.
const REGISTERS: usize = 32;
/// Coefficient RAM, bytes (3-byte words).
const COEFFICIENT_RAM: usize = 0x2000 * 3;
/// Program RAM, bytes (5-byte words).
const PROGRAM_RAM: usize = 0x1000 * 5;
/// The longest write kept (the program image is 0x3c03 bytes).
const MAX_WRITE: usize = 0x1_0000;
/// What `0x60` answers \[G\]: the navi's library reads it and moves on.
const DEVICE_ID: u8 = 0x55;

/// CRC-16/XMODEM.
#[must_use]
pub fn crc16(bytes: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &b in bytes {
        crc ^= u16::from(b) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                crc << 1 ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    registers: [u8; REGISTERS],
    coefficients: Vec<u8>,
    program: Vec<u8>,
    crc: u16,
    /// The staged run-time coefficient write: address and data.
    staged: Vec<u8>,
    /// The write transaction in progress.
    write: Vec<u8>,
    writing: bool,
    /// The answer reads return, and how far they have got.
    answer: Vec<u8>,
    at: usize,
}

impl Default for State {
    fn default() -> State {
        State {
            registers: [0; REGISTERS],
            coefficients: alloc::vec![0; COEFFICIENT_RAM],
            program: alloc::vec![0; PROGRAM_RAM],
            crc: 0,
            staged: Vec::new(),
            write: Vec::new(),
            writing: false,
            answer: Vec::new(),
            at: 0,
        }
    }
}

impl State {
    /// Acts on a finished write transaction.
    fn command(&mut self) {
        let w = core::mem::take(&mut self.write);
        self.writing = false;
        let Some(&c) = w.first() else { return };
        let address = |w: &[u8]| -> usize {
            usize::from(w.get(1).copied().unwrap_or(0)) << 8
                | usize::from(w.get(2).copied().unwrap_or(0))
        };
        let store = |ram: &mut [u8], at: usize, data: &[u8]| {
            for (i, &b) in data.iter().enumerate() {
                if let Some(slot) = ram.get_mut(at + i) {
                    *slot = b;
                }
            }
        };
        self.at = 0;
        match c {
            0xc0..=0xdf => {
                if let Some(&v) = w.get(1) {
                    self.registers[usize::from(c - 0xc0)] = v;
                }
                self.answer.clear();
            }
            0x40..=0x5f => self.answer = alloc::vec![self.registers[usize::from(c - 0x40)]],
            0xb4 | 0xb8 => {
                let (ram, word) = if c == 0xb4 {
                    (&mut self.coefficients, 3)
                } else {
                    (&mut self.program, 5)
                };
                store(ram, address(&w) * word, w.get(3..).unwrap_or(&[]));
                self.crc = crc16(&w);
                self.answer.clear();
            }
            0x72 => self.answer = self.crc.to_be_bytes().to_vec(),
            0x34 => {
                let at = address(&w) * 3;
                self.answer = (0..64)
                    .map(|i| self.coefficients.get(at + i).copied().unwrap_or(0))
                    .collect();
            }
            0x80 => {
                self.staged = w[1..].to_vec();
                self.answer.clear();
            }
            0x24 => self.answer = self.staged.clone(),
            0xa4 => {
                if self.staged.len() >= 2 {
                    let at = address(&[0, self.staged[0], self.staged[1]]) * 3;
                    let data = self.staged[2..].to_vec();
                    store(&mut self.coefficients, at, &data);
                }
                self.answer.clear();
            }
            0x60 => self.answer = alloc::vec![DEVICE_ID],
            _ => self.answer.clear(),
        }
    }
}

struct Target {
    address: u8,
    state: Mutex<State>,
}

impl fmt::Debug for Target {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "akm.ak77xx@{:#x}", self.address)
    }
}

impl I2cSlave for Target {
    fn address(&self, address: Address, dir: Direction) -> Ack {
        if address != Address::Seven(self.address) {
            return Ack::Nack;
        }
        let mut s = self.state.lock();
        // A repeated start ends the write before it.
        if s.writing {
            s.command();
        }
        if dir == Direction::Write {
            s.writing = true;
            s.write.clear();
        } else {
            s.at = 0;
        }
        Ack::Ack
    }

    fn write(&self, byte: u8) -> Ack {
        let mut s = self.state.lock();
        if s.writing && s.write.len() < MAX_WRITE {
            s.write.push(byte);
        }
        Ack::Ack
    }

    fn read(&self) -> u8 {
        let s = self.state.lock();
        s.answer.get(s.at).copied().unwrap_or(0)
    }

    fn read_ack(&self, _ack: Ack) {
        self.state.lock().at += 1;
    }

    fn stop(&self) {
        let mut s = self.state.lock();
        if s.writing {
            s.command();
        }
    }
}

/// The DSP.
#[derive(Debug)]
pub struct Ak77xx {
    target: Arc<Target>,
    bus: Option<Arc<I2cBus>>,
}

impl Ak77xx {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a missing, malformed or unknown property.
    pub fn new(props: &Props) -> Result<Ak77xx> {
        let mut r = props.reader();
        let bus = r.require_str("bus")?;
        let address = r.or_range("address", 0x18u64, 0..=0x7f)? as u8;
        r.finish()?;
        let bus = buses::attach(props, bus)?;
        Ok(Ak77xx::with_bus(Some(bus), address))
    }

    /// One answering `address` on `bus`.
    #[must_use]
    pub fn with_bus(bus: Option<Arc<I2cBus>>, address: u8) -> Ak77xx {
        Ak77xx {
            target: Arc::new(Target {
                address,
                state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            }),
            bus,
        }
    }

    /// The bus-facing side.
    #[must_use]
    pub fn slave(&self) -> Arc<dyn I2cSlave> {
        Arc::clone(&self.target) as Arc<dyn I2cSlave>
    }
}

/// The device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an AKM AK77xx audio DSP's I2C command set: registers, RAM downloads and their CRC",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: true,
            summary: "the I2C bus it sits on, by name",
        },
        PropertySpec {
            name: "address",
            kind: ValueKind::Uint,
            required: false,
            summary: "its seven-bit address (default 0x18)",
        },
    ],
    construct: |props| Ok(Box::new(Ak77xx::new(props)?)),
};

impl Device for Ak77xx {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        if let Some(bus) = &self.bus {
            bus.attach(self.slave())?;
        }
        Ok(())
    }

    fn reset(&self, kind: ResetKind) {
        let mut s = self.target.state.lock();
        if kind == ResetKind::Cold {
            *s = State::default();
        } else {
            s.writing = false;
            s.write.clear();
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.target.state.lock().clone();
        w.write_bytes(&s.registers)?;
        w.write_bytes(&s.coefficients)?;
        w.write_bytes(&s.program)?;
        w.write_u16(s.crc)?;
        w.write_bytes(&s.staged)?;
        w.write_bytes(&s.write)?;
        w.write_bool(s.writing)?;
        w.write_bytes(&s.answer)?;
        w.write_u32(s.at as u32)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut s = State::default();
        let regs = r.read_bytes()?;
        let coefficients = r.read_bytes()?.to_vec();
        let program = r.read_bytes()?.to_vec();
        if regs.len() != REGISTERS
            || coefficients.len() != COEFFICIENT_RAM
            || program.len() != PROGRAM_RAM
        {
            return Err(Error::State(String::from(
                "an AK77xx snapshot of a different size",
            )));
        }
        s.registers.copy_from_slice(regs);
        s.coefficients = coefficients;
        s.program = program;
        s.crc = r.read_u16()?;
        s.staged = r.read_bytes()?.to_vec();
        s.write = r.read_bytes()?.to_vec();
        s.writing = r.read_bool()?;
        s.answer = r.read_bytes()?.to_vec();
        s.at = r.read_u32()? as usize;
        *self.target.state.lock() = s;
        Ok(())
    }
}

impl Instance for Ak77xx {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Ak77xx::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("address", ValueKind::Uint))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn on_bus() -> (Ak77xx, Arc<I2cBus>) {
        let bus = Arc::new(I2cBus::new());
        let dev = Ak77xx::with_bus(Some(Arc::clone(&bus)), 0x18);
        bus.attach(dev.slave()).unwrap();
        (dev, bus)
    }

    fn write(bus: &I2cBus, bytes: &[u8]) {
        assert_eq!(
            bus.start(Address::seven(0x18).unwrap(), Direction::Write),
            Ack::Ack
        );
        for &b in bytes {
            assert_eq!(bus.write(b), Ack::Ack);
        }
        bus.stop();
    }

    fn read(bus: &I2cBus, n: usize) -> Vec<u8> {
        assert_eq!(
            bus.start(Address::seven(0x18).unwrap(), Direction::Read),
            Ack::Ack
        );
        let out = (0..n)
            .map(|i| bus.read(if i + 1 == n { Ack::Nack } else { Ack::Ack }))
            .collect();
        bus.stop();
        out
    }

    #[test]
    fn the_crc_is_xmodem() {
        assert_eq!(crc16(b"123456789"), 0x31c3);
    }

    #[test]
    fn the_download_ready_check() {
        // The navi's bring-up: read 0x0f, write it with bit 0 set, read back.
        let (_dev, bus) = on_bus();
        write(&bus, &[0x4f]);
        let r = read(&bus, 1)[0];
        write(&bus, &[0xcf, r | 1]);
        write(&bus, &[0x4f]);
        assert_eq!(read(&bus, 1), [1]);
    }

    #[test]
    fn a_ram_download_reads_back_its_crc() {
        let (dev, bus) = on_bus();
        let mut image = alloc::vec![0xb4, 0x00, 0x00];
        image.extend((0..60u32).map(|i| (i * 7) as u8));
        write(&bus, &image);
        write(&bus, &[0x72]);
        assert_eq!(read(&bus, 2), crc16(&image).to_be_bytes());
        assert_eq!(dev.target.state.lock().coefficients[3], 21);
        write(&bus, &[0x34, 0x00, 0x01]);
        assert_eq!(read(&bus, 3), [21, 28, 35]);
    }

    #[test]
    fn a_staged_coefficient_applies() {
        let (dev, bus) = on_bus();
        write(&bus, &[0x80, 0x00, 0x02, 0xaa, 0xbb, 0xcc]);
        write(&bus, &[0x24]);
        assert_eq!(read(&bus, 5), [0x00, 0x02, 0xaa, 0xbb, 0xcc]);
        write(&bus, &[0xa4, 0x00, 0x00]);
        assert_eq!(
            dev.target.state.lock().coefficients[6..9],
            [0xaa, 0xbb, 0xcc]
        );
    }

    #[test]
    fn a_snapshot_round_trips() {
        let (dev, bus) = on_bus();
        write(&bus, &[0xc3, 0x04]);
        write(&bus, &[0xb8, 0x00, 0x01, 1, 2, 3, 4, 5]);
        let save = |d: &Ak77xx| {
            let mut shape = MachineShape::new();
            shape.add_device("dsp", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("dsp", CLASS.name, CLASS.version).unwrap();
                d.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&dev);
        let back = Ak77xx::with_bus(None, 0x18);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("dsp", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        assert_eq!(*back.target.state.lock(), *dev.target.state.lock());
        assert_eq!(save(&back), bytes);
    }
}
