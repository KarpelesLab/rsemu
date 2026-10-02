//! One channel of the R-Car CAN controller, on a bus with nothing else on
//! it: the mode handshake a driver needs to start the channel, transmissions
//! that complete, and a receive FIFO that stays empty.
//!
//! That is what a head unit sees with the car switched off, and it is enough
//! for a driver that parks its readers until a frame arrives: without the
//! handshake the channel never starts, and a reader that is refused at once
//! instead of parked retries in a loop — at real-time priority, on the navi,
//! starving its software watchdog's kernel thread until the board resets.
//!
//! # Register map
//!
//! Byte-addressed; the driver uses 8-, 16- and 32-bit accesses,
//! little-endian. Everything not listed is plain storage.
//!
//! | Offset | Width | Name | Here |
//! | --- | --- | --- | --- |
//! | `0x800 + n` | 8 | `MCTL[n]` | mailbox *n*'s control: writing bit 7 (transmit request) sends at once — bit 7 clears, bit 0 (sent) sets |
//! | `0x840` | 16 | `CTLR` | control; bits 9:8 are the operating mode |
//! | `0x842` | 16 | `STR` | status, computed: bit 8 while the mode is 1 or 3, bit 9 while it is 2, neither in mode 0 |
//! | `0x860` | 8 | `IER` | interrupt enables |
//! | `0x861` | 8 | `ISR` | interrupt status; a write keeps only the bits written as 1 |
//!
//! A completed transmission raises `ISR` bit 0. `irq` is a level, up while
//! `ISR & IER` is non-zero.
//!
//! # Sources
//!
//! The offsets, bit positions and the handshake are read off the navi's own
//! kernel image, disassembled as data: its start-up sequence writes the mode
//! into `CTLR` and polls `STR` for bit 8 (mode 3), then clears the mode and
//! polls for bits 9:8 clear; its transmit path sets bit 7 of a mailbox's
//! control byte and its abort check looks for bit 0. The register names are
//! the Renesas CAN module's. Receiving frames is not modelled.

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
pub const CLASS_NAME: &str = "rcar.can";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space one channel answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x1000;

/// The channel's interrupt output.
pub const IRQ_PIN: &str = "irq";

const MCTL: u64 = 0x800;
const MAILBOXES: u64 = 64;
const CTLR: u64 = 0x840;
const STR: u64 = 0x842;
const IER: u64 = 0x860;
const ISR: u64 = 0x861;

const MCTL_TRMREQ: u8 = 1 << 7;
const MCTL_SENTDATA: u8 = 1 << 0;
const ISR_SENT: u8 = 1 << 0;

/// The guest-visible state: the window's bytes. `STR` is computed and its
/// two bytes are never stored.
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
    fn str(&self) -> u16 {
        let ctlr = u16::from_le_bytes([self.bytes[CTLR as usize], self.bytes[CTLR as usize + 1]]);
        match (ctlr >> 8) & 3 {
            1 | 3 => 1 << 8,
            2 => 1 << 9,
            _ => 0,
        }
    }

    fn read_byte(&self, offset: u64) -> u8 {
        match offset {
            STR => self.str().to_le_bytes()[0],
            o if o == STR + 1 => self.str().to_le_bytes()[1],
            _ => self.bytes[offset as usize],
        }
    }

    fn write_byte(&mut self, offset: u64, v: u8) {
        match offset {
            o if o == STR || o == STR + 1 => {}
            ISR => self.bytes[ISR as usize] &= v,
            o if (MCTL..MCTL + MAILBOXES).contains(&o) && v & MCTL_TRMREQ != 0 => {
                // Nothing on the bus to wait for: the frame is gone.
                self.bytes[o as usize] = (v & !MCTL_TRMREQ) | MCTL_SENTDATA;
                self.bytes[ISR as usize] |= ISR_SENT;
            }
            _ => self.bytes[offset as usize] = v,
        }
    }

    fn irq(&self) -> bool {
        self.bytes[ISR as usize] & self.bytes[IER as usize] != 0
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

/// One CAN channel.
#[derive(Debug)]
pub struct Can {
    shared: Arc<Shared>,
    region: RegionRef,
}

impl Can {
    /// Build it. It takes no properties.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if any property is given.
    pub fn new(props: &Props) -> Result<Can> {
        props.reader().finish()?;
        Ok(Can::default())
    }
}

impl Default for Can {
    fn default() -> Can {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            irq: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            "rcar.can",
            REGISTER_WINDOW_LEN,
            Arc::new(Registers(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        Can { shared, region }
    }
}

/// The class descriptor.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "one R-Car CAN channel on a silent bus: the mode handshake, transmissions that complete, nothing received",
    properties: &[],
    construct: |props| Ok(Box::new(Can::new(props)?)),
};

impl Device for Can {
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
                message: String::from("a CAN channel drives one pin, `irq`"),
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
                "a CAN channel holds {REGISTER_WINDOW_LEN} bytes, the snapshot {}",
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

impl Instance for Can {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Can::new(props)?)))
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

    fn w(c: &Can, off: u64, bytes: &[u8]) {
        Registers(Arc::clone(&c.shared))
            .write(off, bytes, MemAttrs::DEFAULT)
            .unwrap();
    }

    fn r16(c: &Can, off: u64) -> u16 {
        let mut b = [0u8; 2];
        Registers(Arc::clone(&c.shared))
            .read(off, &mut b, MemAttrs::DEFAULT)
            .unwrap();
        u16::from_le_bytes(b)
    }

    fn r8(c: &Can, off: u64) -> u8 {
        let mut b = [0u8; 1];
        Registers(Arc::clone(&c.shared))
            .read(off, &mut b, MemAttrs::DEFAULT)
            .unwrap();
        b[0]
    }

    #[test]
    fn the_start_up_handshake_completes() {
        // The driver's sequence: reset mode, wait for STR bit 8; operating
        // mode, wait for bits 9:8 clear.
        let c = Can::default();
        let ctlr = r16(&c, CTLR);
        w(&c, CTLR, &((ctlr & 0xf8ff) | 0x0300).to_le_bytes());
        assert_ne!(r16(&c, STR) & 0x100, 0, "in reset");
        let ctlr = r16(&c, CTLR);
        w(&c, CTLR, &(ctlr & 0xfcff).to_le_bytes());
        assert_eq!(r16(&c, STR) & 0x300, 0, "operating");
        w(&c, CTLR, &0x0200u16.to_le_bytes());
        assert_eq!(r16(&c, STR), 0x200, "halted");
    }

    #[test]
    fn status_is_not_writable() {
        let c = Can::default();
        w(&c, STR, &[0xff, 0xff]);
        assert_eq!(r16(&c, STR), 0);
    }

    #[test]
    fn a_transmission_completes_and_interrupts_when_enabled() {
        let c = Can::default();
        w(&c, IER, &[ISR_SENT]);
        w(&c, MCTL + 32, &[0]);
        assert!(!c.shared.state.lock().irq());
        w(&c, MCTL + 32, &[MCTL_TRMREQ]);
        assert_eq!(r8(&c, MCTL + 32), MCTL_SENTDATA, "sent, request gone");
        assert!(c.shared.state.lock().irq());
        // The handler writes back what it did not handle.
        let isr = r8(&c, ISR);
        w(&c, ISR, &[isr & !ISR_SENT]);
        assert_eq!(r8(&c, ISR), 0);
        assert!(!c.shared.state.lock().irq());
    }

    #[test]
    fn the_rest_is_storage() {
        let c = Can::default();
        w(&c, 0x844, &0x1234_5678u32.to_le_bytes());
        let mut b = [0u8; 4];
        Registers(Arc::clone(&c.shared))
            .read(0x844, &mut b, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(u32::from_le_bytes(b), 0x1234_5678);
    }

    #[test]
    fn a_snapshot_round_trips() {
        let c = Can::default();
        w(&c, CTLR, &0x0300u16.to_le_bytes());
        w(&c, IER, &[0x11]);
        w(&c, MCTL + 3, &[MCTL_TRMREQ]);
        let save = |c: &Can| {
            let mut shape = MachineShape::new();
            shape.add_device("can", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("can", CLASS.name, CLASS.version).unwrap();
                c.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&c);
        let back = Can::default();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("can", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let want = c.shared.state.lock().clone();
        assert_eq!(*back.shared.state.lock(), want);
        assert_eq!(save(&back), bytes);
    }
}
