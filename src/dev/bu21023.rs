//! A ROHM BU21023-class resistive touch-screen controller on I²C, as its host
//! sees it: a register file with a downloaded firmware, a coordinate block,
//! and an active-low interrupt per sample.
//!
//! The navi's touch panel. The controller digitises a 4-wire resistive panel
//! to 10 bits per axis and runs a small program the host downloads at every
//! init; this model keeps what the host writes, swallows the download, and
//! produces the samples a finger would.
//!
//! # Registers
//!
//! The pointer auto-increments, except on the download port.
//!
//! | Register | Here |
//! | --- | --- |
//! | `0x20`–`0x23` | first point, X then Y: bits 9..2 in the first byte, bits 1..0 in the low bits of the second |
//! | `0x24`–`0x27` | second point, same packing (always 0: one finger) |
//! | `0x29` | bits 1..0 nonzero while touching |
//! | `0x2A` | status: bit 0 a sample is ready, bit 4 the firmware has started |
//! | `0x3E` | any write clears the status and releases `INT` |
//! | `0x40` | `01` holds the controller's CPU for a download: the start-up is re-armed |
//! | `0x72` | the program download port; bytes written here are not stored |
//! | other | plain storage, read back as written |
//!
//! # The interrupt
//!
//! `int` idles high and falls for each sample. The host's driver only sees an
//! edge it was waiting for, so a sample is presented only once the previous
//! one has been acknowledged (a write to `0x3E`) **and** [`GAP_TICKS`] of the
//! device clock have passed since; samples that arrive sooner queue, and a run
//! of moves collapses to the newest.
//!
//! When a download ends (the STOP after a write to `0x72`) the firmware
//! "starts": status reads `0x10` and `int` falls once. The driver waits for
//! exactly that before its post-init, and repeats the whole init when it does
//! not come, which is what the plain register file this replaced made it do.
//!
//! # Sources
//!
//! ROHM's BU21023 datasheet (Rev.004) for the part's class, address and
//! firmware download. The register meanings and the init and sample
//! sequences were observed on the navi: its user-space driver read as data,
//! and the I²C traffic of a run. The kernel half of the driver is GPL and
//! only its interrupt line and trigger were taken.

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use core::fmt;

use crate::bus::i2c::{Ack, Address, Direction, I2cBus, I2cSlave, buses};
use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::sched::{Budget, Consumed};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rohm.bu21023";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The interrupt output, active low.
pub const INT_PIN: &str = "int";

/// Device-clock ticks between an acknowledge and the next sample's edge: ten
/// milliseconds on the 1 kHz clock the navi gives it, longer than the driver
/// takes to finish one sample and go back to waiting.
pub const GAP_TICKS: u64 = 10;

/// The full scale of each axis.
const RAW_MAX: u32 = 0x3ff;

const REG_X1: usize = 0x20;
const REG_TOUCH: usize = 0x29;
const REG_STATUS: usize = 0x2a;
const REG_ACK: u8 = 0x3e;
const REG_CPU: u8 = 0x40;
const REG_DOWNLOAD: u8 = 0x72;

const STATUS_SAMPLE: u8 = 1 << 0;
const STATUS_STARTED: u8 = 1 << 4;

/// Queued samples kept at most; a host that floods faster than the guest
/// takes them loses the oldest.
const MAX_PENDING: usize = 32;

/// One reading: raw coordinates, and whether the finger is down.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Sample {
    x: u16,
    y: u16,
    down: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    regs: [u8; 256],
    /// The register pointer.
    ptr: u8,
    /// The next written byte selects the register.
    select: bool,
    /// Bytes have gone to the download port in this write.
    downloaded: bool,
    /// `int` is low.
    asserted: bool,
    /// The downloaded firmware is running: samples can be presented.
    running: bool,
    /// Ticks since `int` was released.
    since_ack: u64,
    pending: VecDeque<Sample>,
    /// Where the host pointer was last, so a move while up is not a sample.
    finger: Option<Sample>,
}

impl Default for State {
    fn default() -> State {
        State {
            regs: [0; 256],
            ptr: 0,
            select: false,
            downloaded: false,
            asserted: false,
            running: false,
            since_ack: GAP_TICKS,
            pending: VecDeque::new(),
            finger: None,
        }
    }
}

impl State {
    /// Latch `s` into the coordinate block and raise the interrupt.
    fn present(&mut self, s: Sample) {
        let (x, y) = if s.down { (s.x, s.y) } else { (0, 0) };
        let block = [
            (x >> 2) as u8,
            (x & 3) as u8,
            (y >> 2) as u8,
            (y & 3) as u8,
            0,
            0,
            0,
            0,
        ];
        self.regs[REG_X1..REG_X1 + 8].copy_from_slice(&block);
        self.regs[REG_TOUCH] = u8::from(s.down);
        self.regs[REG_STATUS] = STATUS_SAMPLE;
        self.asserted = true;
    }

    fn queue(&mut self, s: Sample) {
        if let Some(last) = self.pending.back_mut() {
            // A move behind a move: only the newest position matters.
            if last.down && s.down {
                *last = s;
                return;
            }
        }
        if self.pending.len() >= MAX_PENDING {
            self.pending.pop_front();
        }
        self.pending.push_back(s);
    }

    fn write_register(&mut self, byte: u8) {
        match self.ptr {
            REG_DOWNLOAD => {
                self.downloaded = true;
                return;
            }
            REG_ACK => {
                self.regs[REG_STATUS] = 0;
                self.asserted = false;
                self.since_ack = 0;
            }
            REG_CPU if byte == 0x01 => {
                // Held for a download: the firmware will start again.
                self.regs[REG_STATUS] = 0;
                self.asserted = false;
                self.running = false;
            }
            _ => {}
        }
        self.regs[self.ptr as usize] = byte;
        self.ptr = self.ptr.wrapping_add(1);
    }

    /// The write transaction is over.
    fn end_write(&mut self) {
        self.select = false;
        if self.downloaded {
            self.downloaded = false;
            self.regs[REG_STATUS] = STATUS_STARTED;
            self.asserted = true;
            self.running = true;
        }
    }
}

struct Target {
    address: u8,
    state: Mutex<State>,
    int: Mutex<Option<WireSource>>,
}

impl fmt::Debug for Target {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "rohm.bu21023@{:#x}", self.address)
    }
}

impl Target {
    /// Put `int` where the state says. Never called with the state lock held.
    fn drive(&self) {
        let low = self.state.lock().asserted;
        if let Some(out) = self.int.lock().clone() {
            out.set(Level::from_bool(!low));
        }
    }
}

impl I2cSlave for Target {
    fn address(&self, address: Address, dir: Direction) -> Ack {
        if address != Address::Seven(self.address) {
            return Ack::Nack;
        }
        {
            let mut s = self.state.lock();
            // A repeated start ends the write before it, keeping the pointer.
            s.end_write();
            if dir == Direction::Write {
                s.select = true;
            }
        }
        self.drive();
        Ack::Ack
    }

    fn write(&self, byte: u8) -> Ack {
        {
            let mut s = self.state.lock();
            if s.select {
                s.select = false;
                s.ptr = byte;
            } else {
                s.write_register(byte);
            }
        }
        self.drive();
        Ack::Ack
    }

    fn read(&self) -> u8 {
        let s = self.state.lock();
        s.regs[s.ptr as usize]
    }

    fn read_ack(&self, _ack: Ack) {
        let mut s = self.state.lock();
        s.ptr = s.ptr.wrapping_add(1);
    }

    fn stop(&self) {
        self.state.lock().end_write();
        self.drive();
    }
}

/// How host pixels become raw panel coordinates.
#[derive(Debug, Clone, Copy)]
struct Mapping {
    width: u32,
    height: u32,
    flip_x: bool,
    flip_y: bool,
}

impl Mapping {
    fn raw(&self, x: u32, y: u32) -> (u16, u16) {
        let scale = |v: u32, span: u32, flip: bool| {
            let v = v.min(span - 1);
            let r = v * RAW_MAX / (span - 1).max(1);
            (if flip { RAW_MAX - r } else { r }) as u16
        };
        (
            scale(x, self.width, self.flip_x),
            scale(y, self.height, self.flip_y),
        )
    }
}

/// The touch controller.
#[derive(Debug)]
pub struct Bu21023 {
    target: Arc<Target>,
    bus: Option<Arc<I2cBus>>,
    mapping: Mapping,
}

impl Bu21023 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] on a missing, malformed or unknown property.
    pub fn new(props: &Props) -> Result<Bu21023> {
        let mut r = props.reader();
        let bus = r.require_str("bus")?;
        let address = r.or_range("address", 0x5cu64, 0..=0x7f)? as u8;
        let width = r.or_range("width", 800u64, 2..=0xffff)? as u32;
        let height = r.or_range("height", 480u64, 2..=0xffff)? as u32;
        let flip_x = r.or("flip-x", false)?;
        let flip_y = r.or("flip-y", false)?;
        r.finish()?;
        let bus = buses::attach(props, bus)?;
        let mut dev = Bu21023::with_bus(Some(bus), address);
        dev.mapping = Mapping {
            width,
            height,
            flip_x,
            flip_y,
        };
        Ok(dev)
    }

    /// One answering `address` on `bus`, mapping an 800x480 screen.
    #[must_use]
    pub fn with_bus(bus: Option<Arc<I2cBus>>, address: u8) -> Bu21023 {
        Bu21023 {
            target: Arc::new(Target {
                address,
                state: Mutex::with_rank(LockRank::DEVICE, State::default()),
                int: Mutex::with_rank(LockRank::LEAF, None),
            }),
            bus,
            mapping: Mapping {
                width: 800,
                height: 480,
                flip_x: false,
                flip_y: false,
            },
        }
    }

    /// The bus-facing side.
    #[must_use]
    pub fn slave(&self) -> Arc<dyn I2cSlave> {
        Arc::clone(&self.target) as Arc<dyn I2cSlave>
    }

    /// The host's finger is at screen pixel `(x, y)`, down or not.
    ///
    /// A move with the finger up is not a sample; pressing, moving while
    /// down and lifting each are.
    pub fn touch(&self, x: u32, y: u32, down: bool) {
        let (rx, ry) = self.mapping.raw(x, y);
        let now = Sample { x: rx, y: ry, down };
        let mut s = self.target.state.lock();
        let was_down = s.finger.is_some_and(|f| f.down);
        if !down && !was_down {
            return;
        }
        if s.finger == Some(now) {
            return;
        }
        s.finger = Some(now);
        s.queue(now);
    }

    /// Whether `int` is asserted (low).
    #[must_use]
    pub fn interrupting(&self) -> bool {
        self.target.state.lock().asserted
    }

    /// Advance the clock by `ticks`, presenting the next sample when due.
    fn advance(&self, ticks: u64) {
        let fire = {
            let mut s = self.target.state.lock();
            s.since_ack = s.since_ack.saturating_add(ticks);
            if !s.asserted && s.since_ack >= GAP_TICKS && s.running {
                if let Some(next) = s.pending.pop_front() {
                    s.present(next);
                    true
                } else {
                    false
                }
            } else {
                false
            }
        };
        if fire {
            self.target.drive();
        }
    }
}

/// The device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "a ROHM BU21023-class resistive touch controller on I2C, fed by the host pointer",
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
            summary: "its seven-bit address (default 0x5c)",
        },
        PropertySpec {
            name: "width",
            kind: ValueKind::Uint,
            required: false,
            summary: "the screen width the host pointer spans, in pixels (default 800)",
        },
        PropertySpec {
            name: "height",
            kind: ValueKind::Uint,
            required: false,
            summary: "the screen height the host pointer spans, in pixels (default 480)",
        },
        PropertySpec {
            name: "flip-x",
            kind: ValueKind::Bool,
            required: false,
            summary: "the panel's X runs right to left",
        },
        PropertySpec {
            name: "flip-y",
            kind: ValueKind::Bool,
            required: false,
            summary: "the panel's Y runs bottom to top",
        },
    ],
    construct: |props| Ok(Box::new(Bu21023::new(props)?)),
};

impl Device for Bu21023 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        if let Some(bus) = &self.bus {
            bus.attach(self.slave())?;
        }
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.target.state.lock() = State::default();
        self.target.drive();
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != INT_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("the touch controller drives one pin, `int`"),
            });
        }
        *self.target.int.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.target.drive();
    }

    fn is_runnable(&self) -> bool {
        // To pace samples on its clock: the driver loses an edge that comes
        // before it is waiting again.
        true
    }

    fn run(&self, budget: Budget) -> Consumed {
        self.advance(budget.ticks);
        Consumed::new(budget.ticks)
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.target.state.lock().clone();
        w.write_bytes(&s.regs)?;
        w.write_u8(s.ptr)?;
        w.write_bool(s.select)?;
        w.write_bool(s.downloaded)?;
        w.write_bool(s.asserted)?;
        w.write_bool(s.running)?;
        w.write_u64(s.since_ack)?;
        w.write_seq_len(s.pending.len() as u64)?;
        for p in &s.pending {
            write_sample(w, *p)?;
        }
        w.write_bool(s.finger.is_some())?;
        write_sample(
            w,
            s.finger.unwrap_or(Sample {
                x: 0,
                y: 0,
                down: false,
            }),
        )
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut s = State::default();
        let regs = r.read_bytes()?;
        if regs.len() != s.regs.len() {
            return Err(Error::State(String::from(
                "a BU21023 snapshot with a different register file",
            )));
        }
        s.regs.copy_from_slice(regs);
        s.ptr = r.read_u8()?;
        s.select = r.read_bool()?;
        s.downloaded = r.read_bool()?;
        s.asserted = r.read_bool()?;
        s.running = r.read_bool()?;
        s.since_ack = r.read_u64()?;
        let n = r.read_seq_len(5)? as usize;
        if n > MAX_PENDING {
            return Err(Error::State(alloc::format!(
                "a BU21023 snapshot with {n} queued samples"
            )));
        }
        for _ in 0..n {
            s.pending.push_back(read_sample(r)?);
        }
        let has_finger = r.read_bool()?;
        let finger = read_sample(r)?;
        s.finger = has_finger.then_some(finger);
        *self.target.state.lock() = s;
        self.target.drive();
        Ok(())
    }
}

fn write_sample(w: &mut ChunkWriter<'_>, s: Sample) -> Result<()> {
    w.write_u16(s.x)?;
    w.write_u16(s.y)?;
    w.write_bool(s.down)
}

fn read_sample(r: &mut ChunkReader<'_>) -> Result<Sample> {
    Ok(Sample {
        x: r.read_u16()?,
        y: r.read_u16()?,
        down: r.read_bool()?,
    })
}

impl Instance for Bu21023 {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Bu21023::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str).required())
        .prop(PropSchema::new("address", ValueKind::Uint).range(0, 0x7f))
        .prop(PropSchema::new("width", ValueKind::Uint).range(2, 0xffff))
        .prop(PropSchema::new("height", ValueKind::Uint).range(2, 0xffff))
        .prop(PropSchema::new("flip-x", ValueKind::Bool))
        .prop(PropSchema::new("flip-y", ValueKind::Bool))
        .port(INT_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn dev() -> Bu21023 {
        Bu21023::with_bus(None, 0x5c)
    }

    fn write(d: &Bu21023, bytes: &[u8]) {
        let t = &d.target;
        assert_eq!(t.address(Address::Seven(0x5c), Direction::Write), Ack::Ack);
        for b in bytes {
            t.write(*b);
        }
        t.stop();
    }

    fn read(d: &Bu21023, reg: u8, n: usize) -> alloc::vec::Vec<u8> {
        let t = &d.target;
        t.address(Address::Seven(0x5c), Direction::Write);
        t.write(reg);
        t.address(Address::Seven(0x5c), Direction::Read);
        let mut out = alloc::vec::Vec::new();
        for i in 0..n {
            out.push(t.read());
            t.read_ack(if i + 1 == n { Ack::Nack } else { Ack::Ack });
        }
        t.stop();
        out
    }

    /// The driver's init, as seen on the navi, up to the start-up edge.
    fn init(d: &Bu21023) {
        write(d, &[0x40, 0x01]);
        write(d, &[0x3e, 0xff]);
        write(d, &[0x30, 0x4e]);
        assert_eq!(read(d, 0x30, 1), [0x4e], "verified by reading back");
        write(d, &[0x70, 0x00]);
        write(d, &[0x71, 0x00]);
        assert!(!d.interrupting());
        let mut download = alloc::vec![0x72];
        download.extend(core::iter::repeat_n(0xa5, 8192));
        write(d, &download);
        assert!(d.interrupting(), "the firmware's start-up edge");
        assert_eq!(read(d, 0x2a, 1), [0x10]);
        write(d, &[0x40, 0x03]);
        write(d, &[0x3e, 0xff]);
        assert!(!d.interrupting());
    }

    #[test]
    fn the_download_starts_the_firmware_and_a_new_hold_rearms_it() {
        let d = dev();
        init(&d);
        assert_eq!(read(&d, 0x72, 1), [0], "the download is not stored");
        write(&d, &[0x40, 0x01]);
        assert_eq!(read(&d, 0x2a, 1), [0], "held again");
        write(&d, &[0x72, 1, 2, 3]);
        assert!(d.interrupting());
    }

    #[test]
    fn a_tap_is_a_press_and_a_release_each_acknowledged() {
        let d = dev();
        init(&d);
        d.advance(GAP_TICKS);
        d.touch(799, 479, true);
        d.touch(799, 479, false);
        d.advance(1);
        assert!(d.interrupting(), "the press");
        let block = read(&d, 0x20, 12);
        let x = u16::from(block[0]) << 2 | u16::from(block[1] & 3);
        let y = u16::from(block[2]) << 2 | u16::from(block[3] & 3);
        assert_eq!((x, y), (0x3ff, 0x3ff), "the far corner is full scale");
        assert_eq!(block[9] & 3, 1, "touching");
        assert_eq!(block[10] & 1, 1, "a sample is ready");
        write(&d, &[0x3e, 0xff]);
        assert!(!d.interrupting());
        d.advance(GAP_TICKS - 1);
        assert!(!d.interrupting(), "not before the driver is waiting again");
        d.advance(1);
        assert!(d.interrupting(), "the release");
        assert_eq!(read(&d, 0x29, 1), [0]);
        assert_eq!(read(&d, 0x20, 4), [0, 0, 0, 0]);
    }

    #[test]
    fn moves_collapse_and_hovering_is_not_a_sample() {
        let d = dev();
        init(&d);
        d.advance(GAP_TICKS);
        d.touch(10, 10, false);
        d.advance(1);
        assert!(!d.interrupting(), "the pointer moved, no finger");
        d.touch(0, 0, true);
        d.touch(100, 0, true);
        d.touch(400, 240, true);
        d.advance(1);
        let block = read(&d, 0x20, 4);
        let x = u16::from(block[0]) << 2 | u16::from(block[1] & 3);
        assert_eq!(u32::from(x), 400 * 0x3ff / 799, "only the newest position");
    }

    #[test]
    fn nothing_is_presented_before_the_firmware_starts() {
        let d = dev();
        d.touch(5, 5, true);
        d.advance(100);
        assert!(!d.interrupting());
        init(&d);
        d.advance(GAP_TICKS);
        assert!(d.interrupting(), "the queued press, once started");
    }

    #[test]
    fn a_snapshot_round_trips() {
        let d = dev();
        init(&d);
        d.touch(1, 2, true);
        d.touch(3, 4, true);
        d.touch(3, 4, false);
        let save = |d: &Bu21023| {
            let mut shape = MachineShape::new();
            shape.add_device("tp", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("tp", CLASS.name, CLASS.version).unwrap();
                d.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&d);
        let back = dev();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("tp", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let restored = back.target.state.lock().clone();
        assert_eq!(restored, *d.target.state.lock());
        assert_eq!(save(&back), bytes);
    }
}
