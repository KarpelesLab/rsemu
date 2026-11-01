//! The interrupt controller in front of the R-Car H1's SH-4A "realtime
//! domain" core, as far as the core's firmware needs it to boot and to hear
//! the ARM: a mask over the sources, per-source enables beneath it, and the
//! request it hands the core — a level and the `INTEVT` code the firmware
//! dispatches on.
//!
//! # Registers
//!
//! Two windows of the page at `0xfe782000`, which the HPB-DMAC's status
//! mirror shares (its words `0x0f0` and `0x104` stay its own):
//!
//! | Offset | Here |
//! | --- | --- |
//! | `0x040` | mask: a write sets the written bits (masks those sources); reads the mask |
//! | `0x044` | a write clears the written mask bits; reads the mask |
//! | `0x290` | a write disables the written per-source bits \[G\]; reads the enables |
//! | `0x2a0` | a write enables the written per-source bits; reads the enables |
//!
//! Every other word in the two windows is plain storage (the firmware's
//! start-up writes priorities and configuration there that are not decoded).
//! Out of reset everything is masked and nothing enabled.
//!
//! # Sources
//!
//! | Input | Mask bit | Enable bit | `INTEVT` | Level |
//! | --- | --- | --- | --- | --- |
//! | `iccom` | 23 | — | `0x6c0` | 8 \[G\] |
//! | `tmu0` | 2 | 13 | `0x500` \[G\] | 14 \[G\] |
//! | `tmu1` | 2 | 14 | `0x520` | 14 \[G\] |
//! | `tmu2` | 2 | 15 \[G\] | `0x540` \[G\] | 14 \[G\] |
//!
//! Each input is a level. The highest-level unmasked, enabled source is
//! stored into the core's [`INTERRUPT_REQUEST`](ExportId::INTERRUPT_REQUEST)
//! cell (ties to the lowest code), or zero when none is pending; the
//! firmware's handler clears the source at the source, as on hardware.
//!
//! # Sources of these facts
//!
//! The SH-4A firmware's own interrupt set-up, disassembled as data: the codes
//! it installs handlers for (`0x6c0` for the mailbox, `0x520` for TMU2
//! channel 1) and the mask and enable writes beside each. The levels are not
//! decoded from its priority writes yet; they are chosen to sit between the
//! mailbox and the tick.

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{
    Device, DeviceClass, ExportId, PropertySpec, RealizeCtx, ResetKind, SinkPin,
};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSink};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.shintc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The mask window: `0x000..0x048` of the page.
pub const MASKS_LEN: u64 = 0x48;
/// The enable window: `0x290..0x2c0` of the page.
pub const ENABLES_BASE: u64 = 0x290;
/// Its length.
pub const ENABLES_LEN: u64 = 0x30;

const MASK_SET: u64 = 0x40;
const MASK_CLEAR: u64 = 0x44;
const ENABLE_CLEAR: u64 = 0x290;
const ENABLE_SET: u64 = 0x2a0;

/// One source: its input, mask bit, enable bit, code and level.
struct Line {
    port: &'static str,
    mask: u32,
    enable: Option<u32>,
    intevt: u32,
    level: u32,
}

const LINES: [Line; 4] = [
    Line {
        port: "iccom",
        mask: 23,
        enable: None,
        intevt: 0x6c0,
        level: 8,
    },
    Line {
        port: "tmu0",
        mask: 2,
        enable: Some(13),
        intevt: 0x500,
        level: 14,
    },
    Line {
        port: "tmu1",
        mask: 2,
        enable: Some(14),
        intevt: 0x520,
        level: 14,
    },
    Line {
        port: "tmu2",
        mask: 2,
        enable: Some(15),
        intevt: 0x540,
        level: 14,
    },
];

/// A request as the SH-4A core's interrupt-request cell takes it: the level
/// in bits 3:0, the `INTEVT` code from bit 16 (the core's `pack_request`,
/// kept here so this controller does not need the core built).
const fn pack(level: u32, intevt: u32) -> u64 {
    ((level & 0xf) as u64) | (((intevt & 0x3fff) as u64) << 16)
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// The mask window's words, the mask among them.
    masks: Vec<u32>,
    /// The enable window's words, the enables among them.
    enables: Vec<u32>,
    /// Each input's level.
    inputs: u32,
}

impl Default for State {
    fn default() -> State {
        let mut masks = alloc::vec![0; (MASKS_LEN / 4) as usize];
        masks[(MASK_SET / 4) as usize] = u32::MAX;
        State {
            masks,
            enables: alloc::vec![0; (ENABLES_LEN / 4) as usize],
            inputs: 0,
        }
    }
}

impl State {
    fn mask(&self) -> u32 {
        self.masks[(MASK_SET / 4) as usize]
    }

    fn enabled(&self) -> u32 {
        self.enables[((ENABLE_SET - ENABLES_BASE) / 4) as usize]
    }

    /// The request to hand the core, packed, or 0.
    fn request(&self) -> u64 {
        let (mask, enabled) = (self.mask(), self.enabled());
        LINES
            .iter()
            .enumerate()
            .filter(|(i, l)| {
                self.inputs >> i & 1 != 0
                    && mask >> l.mask & 1 == 0
                    && l.enable.is_none_or(|b| enabled >> b & 1 != 0)
            })
            .map(|(_, l)| l)
            .max_by_key(|l| (l.level, u32::MAX - l.intevt))
            .map_or(0, |l| pack(l.level, l.intevt))
    }
}

#[derive(Debug)]
struct Shared {
    state: Mutex<State>,
    cell: Mutex<Option<Arc<AtomicU64>>>,
}

impl Shared {
    fn drive(&self) {
        let request = self.state.lock().request();
        if let Some(cell) = self.cell.lock().clone() {
            cell.store(request, Ordering::Release);
        }
    }
}

/// One of the two register windows.
#[derive(Debug)]
struct Window {
    shared: Arc<Shared>,
    /// The window's first offset in the page.
    base: u64,
}

impl MemOps for Window {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        let at = self.base + (offset & !3);
        let s = self.shared.state.lock();
        let v = match at {
            MASK_SET | MASK_CLEAR => s.mask(),
            ENABLE_SET | ENABLE_CLEAR => s.enabled(),
            _ if self.base == 0 => s.masks[(at / 4) as usize],
            _ => s.enables[((at - ENABLES_BASE) / 4) as usize],
        };
        dst.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 || attrs.debug {
            return Err(BusError::BadAccess);
        }
        let v = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        let at = self.base + (offset & !3);
        {
            let mut s = self.shared.state.lock();
            let mask = (MASK_SET / 4) as usize;
            let enable = ((ENABLE_SET - ENABLES_BASE) / 4) as usize;
            match at {
                MASK_SET => s.masks[mask] |= v,
                MASK_CLEAR => s.masks[mask] &= !v,
                ENABLE_SET => s.enables[enable] |= v,
                ENABLE_CLEAR => s.enables[enable] &= !v,
                _ if self.base == 0 => s.masks[(at / 4) as usize] = v,
                _ => s.enables[((at - ENABLES_BASE) / 4) as usize] = v,
            }
        }
        self.shared.drive();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

/// One input line.
struct Input {
    shared: Arc<Shared>,
    index: usize,
}

impl fmt::Debug for Input {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Input({})", LINES[self.index].port)
    }
}

impl WireSink for Input {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        {
            let mut s = self.shared.state.lock();
            if level.is_high() {
                s.inputs |= 1 << self.index;
            } else {
                s.inputs &= !(1 << self.index);
            }
        }
        self.shared.drive();
    }
}

/// The SH-4A's interrupt controller.
#[derive(Debug)]
pub struct ShIntc {
    shared: Arc<Shared>,
    masks: RegionRef,
    enables: RegionRef,
    processor: String,
    pins: Mutex<Vec<Arc<Input>>>,
}

impl ShIntc {
    /// Build it. `processor` names the SH-4A it interrupts.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a missing `processor` or an unknown property.
    pub fn new(props: &Props) -> Result<ShIntc> {
        let mut r = props.reader();
        let processor = r
            .optional_link("processor")?
            .ok_or_else(|| Error::Property(String::from("processor: required")))?
            .as_str()
            .to_string();
        r.finish()?;
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            cell: Mutex::with_rank(LockRank::LEAF, None),
        });
        let window = |base, len| -> RegionRef {
            Arc::new(Region::io(
                CLASS_NAME,
                len,
                Arc::new(Window {
                    shared: Arc::clone(&shared),
                    base,
                }) as Arc<dyn MemOps>,
            ))
        };
        Ok(ShIntc {
            masks: window(0, MASKS_LEN),
            enables: window(ENABLES_BASE, ENABLES_LEN),
            shared,
            processor,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        })
    }

    /// Hand it the core's interrupt-request cell (what binding does).
    pub fn attach_cell(&self, cell: Arc<AtomicU64>) {
        *self.shared.cell.lock() = Some(cell);
        self.shared.drive();
    }
}

/// The class descriptor.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the R-Car H1 SH-4A's interrupt controller: mask, per-source enables, level and INTEVT to the core",
    properties: &[PropertySpec {
        name: "processor",
        kind: ValueKind::Link,
        required: true,
        summary: "the SH-4A instance it interrupts",
    }],
    construct: |props| Ok(Box::new(ShIntc::new(props)?)),
};

impl Device for ShIntc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        {
            let mut s = self.shared.state.lock();
            let inputs = s.inputs;
            *s = State::default();
            s.inputs = inputs;
        }
        self.shared.drive();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "masks" => Some(Arc::clone(&self.masks)),
            "enables" => Some(Arc::clone(&self.enables)),
            _ => None,
        }
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        let index = LINES.iter().position(|l| l.port == port)?;
        let pin = Arc::new(Input {
            shared: Arc::clone(&self.shared),
            index,
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: index as u32,
        })
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.shared.state.lock().clone();
        for v in s.masks.iter().chain(&s.enables) {
            w.write_u32(*v)?;
        }
        w.write_u32(s.inputs)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut s = State::default();
        for v in s.masks.iter_mut().chain(s.enables.iter_mut()) {
            *v = r.read_u32()?;
        }
        s.inputs = r.read_u32()?;
        *self.shared.state.lock() = s;
        self.shared.drive();
        Ok(())
    }
}

impl Instance for ShIntc {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        self.attach_cell(ctx.export_cell(&self.processor, ExportId::INTERRUPT_REQUEST)?);
        Ok(())
    }
}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(ShIntc::new(props)?)))
}

/// The validator schema.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("processor", ValueKind::Link))
        .region("")
        .region("masks")
        .region("enables");
    for l in &LINES {
        schema = schema.port(l.port, PortDir::In);
    }
    schema
}

#[cfg(test)]
mod tests {
    use super::pack as pack_request;
    use super::*;

    fn intc() -> (ShIntc, Arc<AtomicU64>) {
        let mut p = Props::new();
        p.insert(
            "processor",
            crate::core::props::Value::Link(crate::core::props::Link::new("sh").unwrap()),
        );
        let i = ShIntc::new(&p).unwrap();
        let cell = Arc::new(AtomicU64::new(0));
        i.attach_cell(Arc::clone(&cell));
        (i, cell)
    }

    fn window(i: &ShIntc, at: u64) -> (Window, u64) {
        let base = if at < MASKS_LEN { 0 } else { ENABLES_BASE };
        let w = Window {
            shared: Arc::clone(&i.shared),
            base,
        };
        (w, at - base)
    }

    fn w(i: &ShIntc, at: u64, v: u32) {
        let (win, off) = window(i, at);
        win.write(off, &v.to_le_bytes(), MemAttrs::DEFAULT).unwrap();
    }

    fn input(i: &ShIntc, port: &str, high: bool) {
        let pin = i.sink(port, &[]).unwrap();
        pin.sink
            .set_level(WireId(0), pin.line, Level::from_bool(high));
    }

    #[test]
    fn the_mailbox_interrupts_once_unmasked() {
        let (i, cell) = intc();
        input(&i, "iccom", true);
        assert_eq!(cell.load(Ordering::Acquire), 0, "masked out of reset");
        w(&i, MASK_CLEAR, 1 << 23);
        assert_eq!(cell.load(Ordering::Acquire), pack_request(8, 0x6c0));
        input(&i, "iccom", false);
        assert_eq!(cell.load(Ordering::Acquire), 0, "the handler cleared it");
    }

    #[test]
    fn a_timer_channel_needs_its_enable_too() {
        let (i, cell) = intc();
        input(&i, "tmu1", true);
        w(&i, MASK_CLEAR, 1 << 2);
        assert_eq!(cell.load(Ordering::Acquire), 0, "not enabled");
        w(&i, ENABLE_SET, 0x4000);
        assert_eq!(cell.load(Ordering::Acquire), pack_request(14, 0x520));
        // The tick outranks the mailbox.
        w(&i, MASK_CLEAR, 1 << 23);
        input(&i, "iccom", true);
        assert_eq!(cell.load(Ordering::Acquire), pack_request(14, 0x520));
        w(&i, MASK_SET, 1 << 2);
        assert_eq!(cell.load(Ordering::Acquire), pack_request(8, 0x6c0));
    }

    #[cfg(feature = "cpu-sh4a")]
    #[test]
    fn requests_pack_as_the_core_unpacks_them() {
        assert_eq!(pack(14, 0x520), crate::cpu::sh4a::pack_request(14, 0x520));
    }

    #[test]
    fn the_rest_is_storage() {
        let (i, _) = intc();
        w(&i, 0x34, 0x1414);
        let mut b = [0u8; 4];
        let (win, off) = window(&i, 0x34);
        win.read(off, &mut b, MemAttrs::DEFAULT).unwrap();
        assert_eq!(u32::from_le_bytes(b), 0x1414);
    }
}
