//! The R-Car IEBus controller — Toyota's AVC-LAN — on a bus with nothing
//! else on it: transmissions complete, nothing is received.
//!
//! The driver starts a frame and waits for its transmit-status interrupt;
//! when that never comes it times out, resets the controller and busy-waits
//! through the reset, from a real-time kernel thread. On the navi that
//! starved the software watchdog's thread and reset the board. With every
//! frame reported delivered the driver never recovers, and its reader blocks
//! until a frame arrives — which, here, none does.
//!
//! # Register map
//!
//! Byte registers; everything not listed is plain storage (the addresses,
//! length, clock set-up, and the transmit and receive buffers at `0x100` and
//! `0x200`).
//!
//! | Offset | Name | Here |
//! | --- | --- | --- |
//! | `0x01` | command | writing 2 starts a transmission, which completes at once |
//! | `0x10` | flags | reads 0: the bus is idle and the transmit buffer free |
//! | `0x11` | transmit status | bit 5 (sent) is set by a completed transmission; a write clears the bits written as 1 |
//! | `0x12` | transmit interrupt enable | storage |
//! | `0x14` | receive status | never set; a write clears the bits written as 1 |
//! | `0x15` | receive interrupt enable | storage |
//!
//! `irq` is a level, up while `(0x11 & 0x12) | (0x14 & 0x15)` is non-zero.
//!
//! Reporting every frame delivered is a choice: on a real bus with nobody
//! else on it a frame addressed to another unit would go unacknowledged, and
//! which status bit says so is not visible in the driver. The driver only
//! hands either outcome to its caller; a timeout is what it recovers from.
//!
//! # Sources
//!
//! Offsets, bits and the transmit handshake are read off the navi's own
//! kernel image (its `Iebdrv_*` driver) and `avc_lan.so`, disassembled as
//! data.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.iebus";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the controller answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x1000;

/// The interrupt output.
pub const IRQ_PIN: &str = "irq";

const COMMAND: u64 = 0x01;
const FLAGS: u64 = 0x10;
const TX_STATUS: u64 = 0x11;
const TX_ENABLE: u64 = 0x12;
const RX_STATUS: u64 = 0x14;
const RX_ENABLE: u64 = 0x15;

const COMMAND_SEND: u8 = 2;
const TX_SENT: u8 = 1 << 5;

/// The guest-visible state: the window's bytes. `FLAGS` is computed and
/// never stored.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    bytes: Vec<u8>,
}

impl Default for State {
    fn default() -> State {
        State {
            bytes: alloc::vec![0; REGISTER_WINDOW_LEN as usize],
        }
    }
}

impl State {
    fn get(&self, offset: u64) -> u8 {
        self.bytes[offset as usize]
    }

    fn read_byte(&self, offset: u64) -> u8 {
        if offset == FLAGS { 0 } else { self.get(offset) }
    }

    fn write_byte(&mut self, offset: u64, v: u8) {
        match offset {
            FLAGS => {}
            TX_STATUS | RX_STATUS => self.bytes[offset as usize] &= !v,
            COMMAND => {
                self.bytes[COMMAND as usize] = v;
                if v == COMMAND_SEND {
                    // Nobody to wait for: the frame is out.
                    self.bytes[TX_STATUS as usize] |= TX_SENT;
                }
            }
            _ => self.bytes[offset as usize] = v,
        }
    }

    fn irq(&self) -> bool {
        (self.get(TX_STATUS) & self.get(TX_ENABLE)) | (self.get(RX_STATUS) & self.get(RX_ENABLE))
            != 0
    }
}

#[derive(Debug)]
struct Shared {
    state: Mutex<State>,
    irq: Mutex<Option<WireSource>>,
}

impl Shared {
    fn drive(&self) {
        let level = self.state.lock().irq();
        let out = self.irq.lock().clone();
        if let Some(out) = out {
            out.set(Level::from_bool(level));
        }
    }
}

#[derive(Debug)]
struct Registers(Arc<Shared>);

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if offset + dst.len() as u64 > REGISTER_WINDOW_LEN {
            return Err(BusError::BadAccess);
        }
        // No register has a read side effect, so a debugger read is an
        // ordinary one.
        let s = self.0.state.lock();
        for (i, b) in dst.iter_mut().enumerate() {
            *b = s.read_byte(offset + i as u64);
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug || offset + src.len() as u64 > REGISTER_WINDOW_LEN {
            return Err(BusError::BadAccess);
        }
        {
            let mut s = self.0.state.lock();
            for (i, b) in src.iter().enumerate() {
                s.write_byte(offset + i as u64, *b);
            }
        }
        self.0.drive();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::IO
    }
}

/// The IEBus controller.
#[derive(Debug)]
pub struct IeBus {
    shared: Arc<Shared>,
    region: RegionRef,
}

impl IeBus {
    /// Build it. It takes no properties.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if any property is given.
    pub fn new(props: &Props) -> Result<IeBus> {
        props.reader().finish()?;
        Ok(IeBus::default())
    }
}

impl Default for IeBus {
    fn default() -> IeBus {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            irq: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            "rcar.iebus",
            REGISTER_WINDOW_LEN,
            Arc::new(Registers(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        IeBus { shared, region }
    }
}

/// The class descriptor.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the R-Car IEBus (AVC-LAN) controller on a silent bus: transmissions complete, nothing received",
    properties: &[],
    construct: |props| Ok(Box::new(IeBus::new(props)?)),
};

impl Device for IeBus {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.shared.state.lock() = State::default();
        self.shared.drive();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != IRQ_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("an IEBus controller drives one pin, `irq`"),
            });
        }
        *self.shared.irq.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.drive();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.shared.state.lock().clone();
        w.write_bytes(&s.bytes)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bytes = r.read_bytes()?;
        if bytes.len() as u64 != REGISTER_WINDOW_LEN {
            return Err(Error::State(alloc::format!(
                "an IEBus controller holds {REGISTER_WINDOW_LEN} bytes, the snapshot {}",
                bytes.len()
            )));
        }
        *self.shared.state.lock() = State {
            bytes: bytes.to_vec(),
        };
        self.shared.drive();
        Ok(())
    }
}

impl Instance for IeBus {}

/// Add the class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind the class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(IeBus::new(props)?)))
}

/// The validator schema.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir};
    ClassSchema::new(CLASS_NAME)
        .region("")
        .region("regs")
        .port(IRQ_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn w(b: &IeBus, off: u64, v: u8) {
        Registers(Arc::clone(&b.shared))
            .write(off, &[v], MemAttrs::DEFAULT)
            .unwrap();
    }

    fn r(b: &IeBus, off: u64) -> u8 {
        let mut x = [0u8; 1];
        Registers(Arc::clone(&b.shared))
            .read(off, &mut x, MemAttrs::DEFAULT)
            .unwrap();
        x[0]
    }

    #[test]
    fn a_transmission_completes_and_interrupts_when_enabled() {
        let b = IeBus::default();
        w(&b, TX_ENABLE, 0x2f);
        assert_eq!(r(&b, FLAGS), 0, "idle before sending");
        w(&b, 0x100, 0x55);
        w(&b, 0x07, 1);
        assert!(!b.shared.state.lock().irq());
        w(&b, COMMAND, COMMAND_SEND);
        assert_eq!(r(&b, TX_STATUS), TX_SENT);
        assert!(b.shared.state.lock().irq());
        // The handler writes back what it read.
        let st = r(&b, TX_STATUS);
        w(&b, TX_STATUS, st);
        assert_eq!(r(&b, TX_STATUS), 0);
        assert!(!b.shared.state.lock().irq());
    }

    #[test]
    fn a_completion_stays_quiet_while_disabled() {
        let b = IeBus::default();
        w(&b, COMMAND, COMMAND_SEND);
        assert_eq!(r(&b, TX_STATUS), TX_SENT);
        assert!(!b.shared.state.lock().irq());
    }

    #[test]
    fn nothing_is_received_and_flags_read_idle() {
        let b = IeBus::default();
        w(&b, RX_ENABLE, 0xaf);
        w(&b, FLAGS, 0xff);
        assert_eq!(r(&b, FLAGS), 0);
        assert_eq!(r(&b, RX_STATUS), 0);
        assert!(!b.shared.state.lock().irq());
    }

    #[test]
    fn buffers_and_set_up_are_storage() {
        let b = IeBus::default();
        w(&b, 0x18, 37);
        w(&b, 0x11f, 0xa5);
        assert_eq!(r(&b, 0x18), 37);
        assert_eq!(r(&b, 0x11f), 0xa5);
    }

    #[test]
    fn a_snapshot_round_trips() {
        let b = IeBus::default();
        w(&b, TX_ENABLE, 0x2f);
        w(&b, COMMAND, COMMAND_SEND);
        w(&b, 0x05, 0x10);
        let save = |b: &IeBus| {
            let mut shape = MachineShape::new();
            shape.add_device("ieb", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("ieb", CLASS.name, CLASS.version).unwrap();
                b.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&b);
        let back = IeBus::default();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("ieb", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let want = b.shared.state.lock().clone();
        assert_eq!(*back.shared.state.lock(), want);
        assert_eq!(save(&back), bytes);
    }
}
