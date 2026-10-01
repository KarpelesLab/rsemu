//! One channel of the R-Car HSPI: a high-speed SPI master with eight-byte
//! FIFOs each way, run by programmed I/O.
//!
//! # Register map
//!
//! All accesses are 32-bit.
//!
//! | Offset | Name | Here |
//! | --- | --- | --- |
//! | `0x00` | `SPCR` | stored (10 bits): clock divider in 4:0, polarity (6), first-bit select (7), `CCO`/`CCA` (8, 9) |
//! | `0x04` | `SPSR` | status, below; bits 3 and 4 clear when written 0, the rest ignore writes |
//! | `0x08` | `SPSCR` | control (14 bits): bit 5 puts slave select under software, bit 6 is its level (low asserts), bit 8 enables the FIFOs (a falling edge flushes them), bits 11–13 are the interrupt enables |
//! | `0x0c` | `TBR` | a byte to send |
//! | `0x10` | `RBR` | the oldest received byte |
//! | `0x14` | `SPCR2` | stored |
//!
//! `SPSR`:
//!
//! | Bit | Meaning |
//! | --- | --- |
//! | 1 | the transmitter is idle and its FIFO empty |
//! | 4 | receive overrun: a byte arrived with the receive FIFO full (sticky) |
//! | 5 | the receive FIFO is empty |
//! | 8 | the transmit FIFO holds four bytes or fewer |
//! | 9 | the receive FIFO holds four bytes or more |
//! | 10 | the transmit FIFO is full |
//!
//! # Transfers
//!
//! SPI is full duplex, so every byte written to `TBR` is exchanged with the
//! slave on the channel's [`SpiBus`] at once, and the slave's byte lands in
//! the receive FIFO. A driver that writes eight bytes reads eight back; one
//! that never reads sees the overrun flag. With nothing on the bus the byte
//! that comes back is `0xff`, a pulled-up MISO.
//!
//! The transmitter is never seen busy: the byte is gone by the time the
//! write returns. The navi's driver keeps at most eight bytes in flight and
//! samples `SPSR` before every access, so that is indistinguishable from a
//! fast shifter.
//!
//! # Interrupt
//!
//! `irq` is a level: receive-not-empty under `SPSCR` bit 11, transmit-at-most-
//! half under bit 12, receive-at-least-half under bit 13. The driver's handler
//! never acknowledges anything and always claims the interrupt, so the line is
//! raised only while an enabled condition holds — a line raised without one
//! would be serviced forever.
//!
//! # Slave select
//!
//! With `SPSCR` bit 5 set the select follows bit 6 (low asserts). With it
//! clear the controller selects on its own: here from the first byte sent
//! until the driver turns every interrupt enable off with the receive FIFO
//! empty, which is how its transfers end. That hardware-select framing is an
//! inference; the software-select path is what the driver uses for its
//! multi-byte links.
//!
//! # Sources
//!
//! No manual: the register behaviour is read off the Alphard navi's own
//! kernel images, disassembled as data — the order of its accesses, the
//! masks it applies, and the conditions its interrupt handler and timeout
//! test. The channel-to-address assignment and the interrupt numbers are the
//! machine file's business.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::bus::spi::{ChipSelect, SpiBus, buses};
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
pub const CLASS_NAME: &str = "rcar.hspi";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space one channel answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x100;

/// The channel's interrupt output.
pub const IRQ_PIN: &str = "irq";

/// Bytes each FIFO holds.
pub const FIFO_DEPTH: usize = 8;

const SPCR: u64 = 0x00;
const SPSR: u64 = 0x04;
const SPSCR: u64 = 0x08;
const TBR: u64 = 0x0c;
const RBR: u64 = 0x10;
const SPCR2: u64 = 0x14;

const SPCR_MASK: u32 = 0x3ff;
const SPSCR_MASK: u32 = 0x3fff;
const SPCR2_MASK: u32 = 0x87ff;

const SPSR_TXIDLE: u32 = 1 << 1;
const SPSR_RXOO: u32 = 1 << 4;
const SPSR_RXEMPTY: u32 = 1 << 5;
const SPSR_TXHALF: u32 = 1 << 8;
const SPSR_RXHALF: u32 = 1 << 9;
/// Never set here: the transmitter drains on the write that fills it. Kept
/// for the tests, which check the driver's pre-write poll sees it clear.
#[cfg_attr(not(test), allow(dead_code))]
const SPSR_TXFULL: u32 = 1 << 10;
/// The `SPSR` bits a write of 0 clears.
const SPSR_STICKY: u32 = (1 << 3) | SPSR_RXOO;

const SPSCR_SOFT_SS: u32 = 1 << 5;
const SPSCR_SS_LEVEL: u32 = 1 << 6;
const SPSCR_FFEN: u32 = 1 << 8;
const SPSCR_RXNE_IE: u32 = 1 << 11;
const SPSCR_TXHALF_IE: u32 = 1 << 12;
const SPSCR_RXHALF_IE: u32 = 1 << 13;
const SPSCR_IE: u32 = SPSCR_RXNE_IE | SPSCR_TXHALF_IE | SPSCR_RXHALF_IE;

/// The guest-visible state.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct State {
    spcr: u32,
    spscr: u32,
    spcr2: u32,
    /// The sticky `SPSR` bits (3 and 4).
    sticky: u32,
    rx: VecDeque<u8>,
    /// The last byte popped, which an empty FIFO reads again.
    last_rx: u8,
    /// The controller's own select, in hardware-select mode.
    hw_selected: bool,
}

impl State {
    fn spsr(&self) -> u32 {
        let mut v = self.sticky | SPSR_TXIDLE | SPSR_TXHALF;
        if self.rx.is_empty() {
            v |= SPSR_RXEMPTY;
        }
        if self.rx.len() >= FIFO_DEPTH / 2 {
            v |= SPSR_RXHALF;
        }
        v
    }

    fn irq(&self) -> bool {
        let st = self.spsr();
        (self.spscr & SPSCR_RXNE_IE != 0 && st & SPSR_RXEMPTY == 0)
            || (self.spscr & SPSCR_TXHALF_IE != 0 && st & SPSR_TXHALF != 0)
            || (self.spscr & SPSCR_RXHALF_IE != 0 && st & SPSR_RXHALF != 0)
    }

    /// Whether the slave is selected.
    fn selected(&self) -> bool {
        if self.spscr & SPSCR_SOFT_SS != 0 {
            self.spscr & SPSCR_SS_LEVEL == 0
        } else {
            self.hw_selected
        }
    }
}

struct Registers {
    bus: Option<Arc<SpiBus>>,
    cs: ChipSelect,
    state: Mutex<State>,
    irq: Mutex<Option<WireSource>>,
    /// The select level last put on the bus.
    driven_select: Mutex<bool>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Registers")
            .field("cs", &self.cs)
            .finish_non_exhaustive()
    }
}

impl Registers {
    /// Drive the interrupt and the chip select from the state. Called with
    /// no lock held: the select reaches into the slave.
    fn refresh(&self) {
        let (irq, selected) = {
            let s = self.state.lock();
            (s.irq(), s.selected())
        };
        let out = self.irq.lock().clone();
        if let Some(out) = out {
            out.set(Level::from_bool(irq));
        }
        let changed = {
            let mut last = self.driven_select.lock();
            let changed = *last != selected;
            *last = selected;
            changed
        };
        if changed && let Some(bus) = &self.bus {
            bus.select(selected.then_some(self.cs));
        }
    }

    /// Shift `byte` out and the slave's byte in.
    fn send(&self, byte: u8) {
        {
            let mut s = self.state.lock();
            if s.spscr & SPSCR_FFEN == 0 {
                return;
            }
            if s.spscr & SPSCR_SOFT_SS == 0 {
                s.hw_selected = true;
            }
        }
        // Select before the first clock, outside the lock.
        self.refresh();
        let back = self
            .bus
            .as_ref()
            .map_or(0xff, |bus| bus.transfer(u32::from(byte)) as u8);
        let mut s = self.state.lock();
        if s.rx.len() >= FIFO_DEPTH {
            s.sticky |= SPSR_RXOO;
        } else {
            s.rx.push_back(back);
        }
    }

    fn write_reg(&self, offset: u64, v: u32) {
        if offset == TBR {
            self.send(v as u8);
            return;
        }
        let mut s = self.state.lock();
        match offset {
            SPCR => s.spcr = v & SPCR_MASK,
            SPSR => s.sticky &= v | !SPSR_STICKY,
            SPSCR => {
                let was = s.spscr;
                s.spscr = v & SPSCR_MASK;
                if was & SPSCR_FFEN != 0 && s.spscr & SPSCR_FFEN == 0 {
                    s.rx.clear();
                }
                // The driver ends a transfer by turning its interrupts off
                // with everything read back: that is where a hardware
                // select lets go.
                if s.spscr & SPSCR_IE == 0 && s.rx.is_empty() {
                    s.hw_selected = false;
                }
            }
            SPCR2 => s.spcr2 = v & SPCR2_MASK,
            _ => {}
        }
    }

    fn read_reg(&self, offset: u64, debug: bool) -> u32 {
        let mut s = self.state.lock();
        match offset {
            SPCR => s.spcr,
            SPSR => s.spsr(),
            SPSCR => s.spscr,
            RBR => {
                if debug {
                    return u32::from(s.rx.front().copied().unwrap_or(s.last_rx));
                }
                if let Some(b) = s.rx.pop_front() {
                    s.last_rx = b;
                }
                u32::from(s.last_rx)
            }
            SPCR2 => s.spcr2,
            _ => 0,
        }
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        let v = self.read_reg(offset & !3, attrs.debug);
        dst.copy_from_slice(&v.to_le_bytes());
        if !attrs.debug && offset & !3 == RBR {
            self.refresh();
        }
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

/// One HSPI channel.
#[derive(Debug)]
pub struct Hspi {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Hspi {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a malformed or unknown property.
    pub fn new(props: &Props) -> Result<Hspi> {
        let mut r = props.reader();
        let bus = r.optional_str("bus")?.map(ToString::to_string);
        let cs = r.or_range("cs", 0u64, 0..=7)? as u8;
        r.finish()?;
        let bus = match bus {
            Some(name) => Some(buses::attach(props, &name)?),
            None => None,
        };
        Ok(Hspi::with_bus(bus, ChipSelect(cs)))
    }

    /// A channel driving `bus`, selecting `cs` on it.
    #[must_use]
    pub fn with_bus(bus: Option<Arc<SpiBus>>, cs: ChipSelect) -> Hspi {
        let regs = Arc::new(Registers {
            bus,
            cs,
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            irq: Mutex::with_rank(LockRank::LEAF, None),
            driven_select: Mutex::with_rank(LockRank::LEAF, false),
        });
        let region: RegionRef = Arc::new(Region::io(
            "rcar.hspi",
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Hspi { regs, region }
    }

    /// Whether the interrupt output is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.regs.state.lock().irq()
    }
}

/// The `rcar.hspi` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "one R-Car HSPI channel: an SPI master with 8-byte FIFOs, by programmed I/O",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the SPI bus this channel masters, by name (none: MISO reads 0xff)",
        },
        PropertySpec {
            name: "cs",
            kind: ValueKind::Uint,
            required: false,
            summary: "the chip select it drives on that bus (default 0)",
        },
    ],
    construct: |props| Ok(Box::new(Hspi::new(props)?)),
};

impl Device for Hspi {
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
                message: String::from("an HSPI channel drives one pin, `irq`"),
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
        for v in [s.spcr, s.spscr, s.spcr2, s.sticky] {
            w.write_u32(v)?;
        }
        let rx: alloc::vec::Vec<u8> = s.rx.iter().copied().collect();
        w.write_bytes(&rx)?;
        w.write_u8(s.last_rx)?;
        w.write_bool(s.hw_selected)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let spcr = r.read_u32()?;
        let spscr = r.read_u32()?;
        let spcr2 = r.read_u32()?;
        let sticky = r.read_u32()?;
        let rx = r.read_bytes()?;
        if rx.len() > FIFO_DEPTH {
            return Err(Error::State(String::from(
                "an HSPI receive FIFO holds 8 bytes",
            )));
        }
        let rx: VecDeque<u8> = rx.iter().copied().collect();
        let last_rx = r.read_u8()?;
        let hw_selected = r.read_bool()?;
        let s = State {
            spcr,
            spscr,
            spcr2,
            sticky,
            rx,
            last_rx,
            hw_selected,
        };
        let selected = s.selected();
        *self.regs.state.lock() = s;
        // The select is a line this master holds: put it back without an
        // edge (`SpiBus::restore_select`).
        *self.regs.driven_select.lock() = selected;
        if selected && let Some(bus) = &self.regs.bus {
            bus.restore_select(Some(self.regs.cs));
        }
        let out = self.regs.irq.lock().clone();
        if let Some(out) = out {
            out.set(Level::from_bool(self.regs.state.lock().irq()));
        }
        Ok(())
    }
}

impl Instance for Hspi {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Hspi::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(PropSchema::new("cs", ValueKind::Uint).range(0, 7))
        .region("")
        .region("regs")
        .port(IRQ_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::spi::{Format, SpiSlave};
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    /// A slave that answers each byte with its complement and records the
    /// select edges.
    #[derive(Debug, Default)]
    struct Echo {
        log: Mutex<alloc::vec::Vec<i32>>,
    }

    impl SpiSlave for Echo {
        fn format(&self) -> Format {
            Format::default()
        }
        fn select(&self, selected: bool) {
            self.log.lock().push(if selected { -1 } else { -2 });
        }
        fn transfer(&self, mosi: u32) -> u32 {
            self.log.lock().push(mosi as i32);
            !mosi & 0xff
        }
    }

    fn w(h: &Hspi, off: u64, v: u32) {
        h.regs
            .write(off, &v.to_le_bytes(), MemAttrs::DEFAULT)
            .unwrap();
    }

    fn r(h: &Hspi, off: u64) -> u32 {
        let mut b = [0u8; 4];
        h.regs.read(off, &mut b, MemAttrs::DEFAULT).unwrap();
        u32::from_le_bytes(b)
    }

    fn channel() -> (Hspi, Arc<Echo>) {
        let bus = Arc::new(SpiBus::new());
        let echo = Arc::new(Echo::default());
        bus.attach(ChipSelect(0), Arc::clone(&echo) as Arc<dyn SpiSlave>)
            .unwrap();
        (Hspi::with_bus(Some(bus), ChipSelect(0)), echo)
    }

    #[test]
    fn every_byte_sent_brings_one_back() {
        let (h, echo) = channel();
        // Software select, asserted; FIFOs on.
        w(&h, SPSCR, SPSCR_SOFT_SS | SPSCR_FFEN | 1);
        assert_ne!(r(&h, SPSR) & SPSR_RXEMPTY, 0);
        for b in [0x02, 0x80, 0x01, 0x83] {
            assert_eq!(r(&h, SPSR) & SPSR_TXFULL, 0);
            w(&h, TBR, b);
        }
        assert_eq!(r(&h, SPSR) & SPSR_RXEMPTY, 0);
        assert_ne!(r(&h, SPSR) & SPSR_RXHALF, 0, "four in the receive FIFO");
        let back: alloc::vec::Vec<u32> = (0..4).map(|_| r(&h, RBR)).collect();
        assert_eq!(back, [0xfd, 0x7f, 0xfe, 0x7c]);
        assert_ne!(r(&h, SPSR) & SPSR_RXEMPTY, 0);
        assert_eq!(*echo.log.lock(), [-1, 0x02, 0x80, 0x01, 0x83]);
        w(&h, SPSCR, SPSCR_SOFT_SS | SPSCR_SS_LEVEL | SPSCR_FFEN);
        assert_eq!(echo.log.lock().last(), Some(&-2), "bit 6 deselects");
    }

    #[test]
    fn the_interrupt_follows_only_enabled_conditions() {
        let (h, _) = channel();
        w(&h, SPSCR, SPSCR_FFEN | 1);
        assert!(!h.irq_asserted(), "nothing enabled");
        w(&h, TBR, 0x55);
        assert!(!h.irq_asserted(), "a byte waiting is not enough by itself");
        w(&h, SPSCR, SPSCR_FFEN | 1 | SPSCR_RXNE_IE);
        assert!(h.irq_asserted(), "not-empty, enabled");
        r(&h, RBR);
        assert!(!h.irq_asserted(), "reading it back drops the line");
        w(
            &h,
            SPSCR,
            SPSCR_FFEN | 1 | SPSCR_TXHALF_IE | SPSCR_RXHALF_IE,
        );
        assert!(h.irq_asserted(), "an empty transmitter is under half");
    }

    #[test]
    fn a_ninth_unread_byte_overruns() {
        let (h, _) = channel();
        w(&h, SPSCR, SPSCR_FFEN | 1);
        for b in 0..9 {
            w(&h, TBR, b);
        }
        assert_ne!(r(&h, SPSR) & SPSR_RXOO, 0);
        w(&h, SPSR, r(&h, SPSR) & 0x7e7);
        assert_eq!(r(&h, SPSR) & SPSR_RXOO, 0, "cleared by writing 0");
        w(&h, SPSCR, 1);
        assert_ne!(r(&h, SPSR) & SPSR_RXEMPTY, 0, "dropping FFEN flushes");
    }

    #[test]
    fn hardware_select_spans_the_transfer() {
        let (h, echo) = channel();
        w(&h, SPSCR, SPSCR_FFEN | 1);
        w(&h, TBR, 0xaa);
        w(&h, SPSCR, SPSCR_FFEN | 1 | SPSCR_RXNE_IE);
        r(&h, RBR);
        w(&h, SPSCR, SPSCR_FFEN | 1);
        assert_eq!(*echo.log.lock(), [-1, 0xaa, -2]);
    }

    #[test]
    fn nothing_on_the_bus_reads_all_ones() {
        let h = Hspi::with_bus(None, ChipSelect(0));
        w(&h, SPSCR, SPSCR_FFEN | 1);
        w(&h, TBR, 0x12);
        assert_eq!(r(&h, RBR), 0xff);
    }

    #[test]
    fn a_debugger_read_of_the_receive_buffer_pops_nothing() {
        let (h, _) = channel();
        w(&h, SPSCR, SPSCR_FFEN | 1);
        w(&h, TBR, 0x0f);
        let mut b = [0u8; 4];
        h.regs.read(RBR, &mut b, MemAttrs::DEBUG).unwrap();
        assert_eq!(b[0], 0xf0);
        assert_eq!(r(&h, SPSR) & SPSR_RXEMPTY, 0, "still there");
    }

    #[test]
    fn a_snapshot_round_trips() {
        let (h, _) = channel();
        w(&h, SPCR, 0xce);
        w(&h, SPSCR, SPSCR_SOFT_SS | SPSCR_FFEN | 1);
        w(&h, TBR, 0x01);
        w(&h, TBR, 0x02);
        let save = |h: &Hspi| {
            let mut shape = MachineShape::new();
            shape.add_device("hspi", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("hspi", CLASS.name, CLASS.version).unwrap();
                h.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&h);
        let (back, _) = channel();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("hspi", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let want = h.regs.state.lock().clone();
        assert_eq!(*back.regs.state.lock(), want);
        assert_eq!(save(&back), bytes);
    }
}
