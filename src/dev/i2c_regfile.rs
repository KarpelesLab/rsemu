//! A stand-in I²C target: a byte register file at one or more addresses.
//!
//! Most I²C peripherals — video decoders, audio codecs, serializers, clock
//! chips — look the same from the bus: the first byte of a write sets a
//! register pointer, later bytes are stored at it, and a read returns bytes
//! from it, the pointer advancing by one per byte either way. A board that
//! carries a dozen such parts whose behaviour beyond that nobody has
//! modelled yet still needs them to *answer*: a driver that gets a NACK from
//! its codec gives up, retries, or times out, and that changes what the
//! machine does next far more than whether the codec's registers mean
//! anything.
//!
//! `i2c.regfile` is that answer and nothing more. It acknowledges every
//! address in its `addresses` list, keeps a separate 256-byte map for each,
//! and reads back what was written. A register's initial value is 0. It is a
//! placeholder, said so by name; a part that needs behaviour gets a model of
//! its own.
//!
//! # Sources
//!
//! UM10204 for the bus side, through [`crate::bus::i2c`]; the register
//! pointer convention is the one every part this stands in for documents.

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
pub const CLASS_NAME: &str = "i2c.regfile";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// One address's registers and pointer.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Map {
    address: u8,
    pointer: u8,
    regs: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct State {
    maps: Vec<Map>,
    /// Which map the bus addressed, if any.
    current: Option<usize>,
    /// The next byte written sets the pointer.
    expect_pointer: bool,
}

struct Target {
    state: Mutex<State>,
}

impl fmt::Debug for Target {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("i2c.regfile")
    }
}

impl I2cSlave for Target {
    fn address(&self, address: Address, dir: Direction) -> Ack {
        let mut s = self.state.lock();
        let hit = match address {
            Address::Seven(a) => s.maps.iter().position(|m| m.address == a),
            Address::Ten(_) => None,
        };
        s.current = hit;
        s.expect_pointer = dir == Direction::Write;
        if hit.is_some() { Ack::Ack } else { Ack::Nack }
    }

    fn write(&self, byte: u8) -> Ack {
        let mut s = self.state.lock();
        let Some(i) = s.current else {
            return Ack::Nack;
        };
        if s.expect_pointer {
            s.expect_pointer = false;
            s.maps[i].pointer = byte;
        } else {
            let m = &mut s.maps[i];
            m.regs[usize::from(m.pointer)] = byte;
            m.pointer = m.pointer.wrapping_add(1);
        }
        Ack::Ack
    }

    fn read(&self) -> u8 {
        let s = self.state.lock();
        s.current
            .map_or(0xff, |i| s.maps[i].regs[usize::from(s.maps[i].pointer)])
    }

    fn read_ack(&self, _ack: Ack) {
        let mut s = self.state.lock();
        if let Some(i) = s.current {
            s.maps[i].pointer = s.maps[i].pointer.wrapping_add(1);
        }
    }

    fn stop(&self) {
        let mut s = self.state.lock();
        s.current = None;
        s.expect_pointer = false;
    }
}

/// The register-file target.
#[derive(Debug)]
pub struct RegFile {
    target: Arc<Target>,
    bus: Option<Arc<I2cBus>>,
    addresses: Vec<u8>,
}

/// Parse `"0x60, 0x48"` into seven-bit addresses.
fn parse_addresses(text: &str) -> core::result::Result<Vec<u8>, String> {
    let mut out = Vec::new();
    for item in text.split(',').map(str::trim).filter(|t| !t.is_empty()) {
        let v = if let Some(hex) = item.strip_prefix("0x").or_else(|| item.strip_prefix("0X")) {
            u8::from_str_radix(hex, 16)
        } else {
            item.parse::<u8>()
        }
        .map_err(|_| alloc::format!("`{item}` is not an address"))?;
        if v > 0x7f {
            return Err(alloc::format!("{v:#x} is not a seven-bit address"));
        }
        if out.contains(&v) {
            return Err(alloc::format!("{v:#x} is listed twice"));
        }
        out.push(v);
    }
    if out.is_empty() {
        return Err(String::from("no addresses"));
    }
    Ok(out)
}

fn fresh(addresses: &[u8]) -> State {
    State {
        maps: addresses
            .iter()
            .map(|&address| Map {
                address,
                pointer: 0,
                regs: alloc::vec![0; 256],
            })
            .collect(),
        current: None,
        expect_pointer: false,
    }
}

impl RegFile {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a missing, malformed or unknown property.
    pub fn new(props: &Props) -> Result<RegFile> {
        let mut r = props.reader();
        let bus = r.require_str("bus")?;
        let list = r.require_str("addresses")?;
        r.finish()?;
        let addresses = parse_addresses(list)
            .map_err(|why| Error::Property(alloc::format!("i2c.regfile `addresses`: {why}.")))?;
        let bus = buses::attach(props, bus)?;
        Ok(RegFile::with_bus(Some(bus), addresses))
    }

    /// One answering `addresses` on `bus`.
    #[must_use]
    pub fn with_bus(bus: Option<Arc<I2cBus>>, addresses: Vec<u8>) -> RegFile {
        RegFile {
            target: Arc::new(Target {
                state: Mutex::with_rank(LockRank::DEVICE, fresh(&addresses)),
            }),
            bus,
            addresses,
        }
    }

    /// The bus-facing side, for a test or a board that attaches it by hand.
    #[must_use]
    pub fn slave(&self) -> Arc<dyn I2cSlave> {
        Arc::clone(&self.target) as Arc<dyn I2cSlave>
    }

    /// Register `reg` of the map at `address`.
    #[must_use]
    pub fn register(&self, address: u8, reg: u8) -> Option<u8> {
        let s = self.target.state.lock();
        s.maps
            .iter()
            .find(|m| m.address == address)
            .map(|m| m.regs[usize::from(reg)])
    }
}

/// The `i2c.regfile` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a stand-in I2C target: a 256-byte register file with a pointer, at each listed address",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: true,
            summary: "the I2C bus it sits on, by name",
        },
        PropertySpec {
            name: "addresses",
            kind: ValueKind::Str,
            required: true,
            summary: "the seven-bit addresses it answers, comma-separated (\"0x60, 0x48\")",
        },
    ],
    construct: |props| Ok(Box::new(RegFile::new(props)?)),
};

impl Device for RegFile {
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
            *s = fresh(&self.addresses);
        } else {
            s.current = None;
            s.expect_pointer = false;
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.target.state.lock().clone();
        w.write_u32(s.maps.len() as u32)?;
        for m in &s.maps {
            w.write_u8(m.address)?;
            w.write_u8(m.pointer)?;
            w.write_bytes(&m.regs)?;
        }
        w.write_u8(s.current.map_or(0xff, |i| i as u8))?;
        w.write_bool(s.expect_pointer)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let n = r.read_u32()? as usize;
        if n != self.addresses.len() {
            return Err(Error::State(String::from(
                "an i2c.regfile snapshot answers a different number of addresses",
            )));
        }
        let mut maps = Vec::with_capacity(n);
        for _ in 0..n {
            let address = r.read_u8()?;
            let pointer = r.read_u8()?;
            let regs = r.read_bytes()?.to_vec();
            if regs.len() != 256 {
                return Err(Error::State(String::from(
                    "an i2c.regfile map is 256 bytes",
                )));
            }
            maps.push(Map {
                address,
                pointer,
                regs,
            });
        }
        let current = match r.read_u8()? {
            0xff => None,
            i if usize::from(i) < n => Some(usize::from(i)),
            _ => {
                return Err(Error::State(String::from(
                    "an i2c.regfile map out of range",
                )));
            }
        };
        let expect_pointer = r.read_bool()?;
        *self.target.state.lock() = State {
            maps,
            current,
            expect_pointer,
        };
        Ok(())
    }
}

impl Instance for RegFile {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(RegFile::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("addresses", ValueKind::Str).required())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn on_bus(addresses: &[u8]) -> (RegFile, Arc<I2cBus>) {
        let bus = Arc::new(I2cBus::new());
        let dev = RegFile::with_bus(Some(Arc::clone(&bus)), addresses.to_vec());
        bus.attach(dev.slave()).unwrap();
        (dev, bus)
    }

    #[test]
    fn a_write_sets_the_pointer_and_a_read_comes_back_from_it() {
        let (dev, bus) = on_bus(&[0x60, 0x48]);
        let a = |n| Address::seven(n).unwrap();
        assert_eq!(bus.start(a(0x60), Direction::Write), Ack::Ack);
        for b in [0x10, 0xaa, 0xbb] {
            assert_eq!(bus.write(b), Ack::Ack);
        }
        bus.stop();
        assert_eq!(dev.register(0x60, 0x11), Some(0xbb));
        assert_eq!(
            dev.register(0x48, 0x11),
            Some(0),
            "each address is its own map"
        );
        bus.start(a(0x60), Direction::Write);
        bus.write(0x10);
        assert_eq!(
            bus.start(a(0x60), Direction::Read),
            Ack::Ack,
            "repeated start"
        );
        assert_eq!(bus.read(Ack::Ack), 0xaa);
        assert_eq!(bus.read(Ack::Nack), 0xbb);
        bus.stop();
    }

    #[test]
    fn an_address_it_does_not_list_is_refused() {
        let (_dev, bus) = on_bus(&[0x60]);
        assert_eq!(
            bus.start(Address::seven(0x61).unwrap(), Direction::Write),
            Ack::Nack
        );
    }

    #[test]
    fn the_address_list_is_checked() {
        assert_eq!(parse_addresses("0x60, 72").unwrap(), [0x60, 72]);
        assert!(parse_addresses("0x80").is_err());
        assert!(parse_addresses("0x60,0x60").is_err());
        assert!(parse_addresses("").is_err());
    }

    #[test]
    fn a_snapshot_round_trips() {
        let (dev, bus) = on_bus(&[0x18, 0x20]);
        bus.start(Address::seven(0x20).unwrap(), Direction::Write);
        bus.write(0x05);
        bus.write(0x99);
        let save = |d: &RegFile| {
            let mut shape = MachineShape::new();
            shape.add_device("rf", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("rf", CLASS.name, CLASS.version).unwrap();
                d.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&dev);
        let (back, _) = on_bus(&[0x18, 0x20]);
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("rf", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let want = dev.target.state.lock().clone();
        assert_eq!(*back.target.state.lock(), want);
        assert_eq!(save(&back), bytes);
    }
}
