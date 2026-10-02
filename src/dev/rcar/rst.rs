//! The R-Car H1's secondary-core reset control: which Cortex-A9s are out of
//! reset, and where a core starts when it leaves.
//!
//! Only core 0 runs out of power-on reset. The kernel starts the others
//! itself: it writes the physical address of its secondary entry into the
//! **boot-address register**, powers the core's domain through SYSC, and
//! sets the core's two bits in the **reset-control register**. The core then
//! leaves reset and fetches its reset vector at the boot address — not at 0,
//! where the boot loader would park it for good. Once every core is up the
//! kernel writes 0 back to the boot address.
//!
//! # Registers
//!
//! Two windows, both 32-bit:
//!
//! | Region | Offset | Here |
//! | --- | --- | --- |
//! | `""` (`0xfe6cf000` on the H1) | `0x000` | reset control: core *n* (1–3) is held in reset unless both bit 8+*n* and bit 12+*n* are set; reads back what was written; core 0's bits power up set (`0x1100`) |
//! | | `0xfb0` | the block's lock-access key (`0xc5acce55`): storage, the lock is not enforced |
//! | | the rest | storage |
//! | `"bar"` (`0xfe700040`) | `0x0` | boot address: where a released core starts; 0 means its architectural reset vector |
//!
//! Core *n*'s `hold<n>` output is high while it is held; wire it to that
//! core's `hold` input. The boot address reaches the cores through their
//! [`RESET_ADDRESS`](ExportId::RESET_ADDRESS) cells, named by
//! `processors = [cpu1, cpu2, cpu3]`.
//!
//! Which of a core's two bits is which reset (core, debug, NEON…) is not
//! visible in the code that drives them; both are required here.
//!
//! # Sources
//!
//! Read off the navi's own kernel images (its secondary bring-up and the
//! completion that clears the boot address) and its boot loader's parking
//! loop, disassembled as data.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, ExportId, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicU64, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.rst";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the control window answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x1000;

/// How much the boot-address window answers.
pub const BAR_WINDOW_LEN: u64 = 4;

/// Secondary cores this controls: 1, 2 and 3.
const SECONDARIES: usize = 3;

const WORDS: usize = (REGISTER_WINDOW_LEN / 4) as usize;

/// Core 0's bits: out of reset from power-on.
const CTRL_RESET: u32 = 0x1100;

/// What a reset-address cell holds for "the architectural vector".
const NO_ADDRESS: u64 = u64::MAX;

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    words: Vec<u32>,
    bar: u32,
}

impl Default for State {
    fn default() -> State {
        let mut words = alloc::vec![0; WORDS];
        words[0] = CTRL_RESET;
        State { words, bar: 0 }
    }
}

impl State {
    /// Whether secondary core `n` (1–3) is held.
    fn held(&self, n: usize) -> bool {
        let bits = (1u32 << (8 + n)) | (1u32 << (12 + n));
        self.words[0] & bits != bits
    }
}

#[derive(Debug)]
struct Shared {
    state: Mutex<State>,
    holds: Mutex<[Option<WireSource>; SECONDARIES]>,
    cells: Mutex<Vec<Arc<AtomicU64>>>,
}

impl Shared {
    fn drive(&self) {
        let held: [bool; SECONDARIES] = {
            let s = self.state.lock();
            core::array::from_fn(|i| s.held(i + 1))
        };
        let outs = self.holds.lock().clone();
        for (out, held) in outs.iter().zip(held) {
            if let Some(out) = out {
                out.set(Level::from_bool(held));
            }
        }
    }

    fn publish_bar(&self) {
        let bar = self.state.lock().bar;
        let at = if bar == 0 { NO_ADDRESS } else { u64::from(bar) };
        for cell in self.cells.lock().iter() {
            cell.store(at, Ordering::Release);
        }
    }
}

fn word_of(src: &[u8]) -> Option<u32> {
    <[u8; 4]>::try_from(src).ok().map(u32::from_le_bytes)
}

#[derive(Debug)]
struct Control(Arc<Shared>);

impl MemOps for Control {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        let v = self.0.state.lock().words[(offset / 4) as usize];
        dst.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let Some(v) = word_of(src) else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        self.0.state.lock().words[(offset / 4) as usize] = v;
        if offset == 0 {
            self.0.drive();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

#[derive(Debug)]
struct BootAddress(Arc<Shared>);

impl MemOps for BootAddress {
    fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        let v = self.0.state.lock().bar;
        dst.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    fn write(&self, _offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let Some(v) = word_of(src) else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        self.0.state.lock().bar = v;
        self.0.publish_bar();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

/// The reset controller.
#[derive(Debug)]
pub struct Rst {
    shared: Arc<Shared>,
    control: RegionRef,
    bar: RegionRef,
    processors: Vec<String>,
}

impl Rst {
    /// Build it.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `processors` names more than three cores or
    /// is not a list of objects.
    pub fn new(props: &Props) -> Result<Rst> {
        let mut r = props.reader();
        let processors = match r.optional_list("processors")? {
            Some(list) => list
                .iter()
                .map(|v| Ok(v.to_link("processors")?.as_str().to_string()))
                .collect::<Result<Vec<_>>>()?,
            None => Vec::new(),
        };
        r.finish()?;
        if processors.len() > SECONDARIES {
            return Err(Error::Property(format!(
                "`processors` names {} cores; the reset control has three secondaries",
                processors.len()
            )));
        }
        Ok(Rst {
            processors,
            ..Rst::default()
        })
    }

    /// Give it the reset-address cells of the secondaries, in core order.
    pub fn attach_cells(&self, cells: Vec<Arc<AtomicU64>>) {
        *self.shared.cells.lock() = cells;
        self.shared.publish_bar();
    }
}

impl Default for Rst {
    fn default() -> Rst {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            holds: Mutex::with_rank(LockRank::LEAF, [None, None, None]),
            cells: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        });
        let control: RegionRef = Arc::new(Region::io(
            "rcar.rst",
            REGISTER_WINDOW_LEN,
            Arc::new(Control(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        let bar: RegionRef = Arc::new(Region::io(
            "rcar.rst.bar",
            BAR_WINDOW_LEN,
            Arc::new(BootAddress(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        Rst {
            shared,
            control,
            bar,
            processors: Vec::new(),
        }
    }
}

/// The class descriptor.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "R-Car H1 secondary-core reset control and boot address",
    properties: &[PropertySpec {
        name: "processors",
        kind: ValueKind::List,
        required: false,
        summary: "the secondary cores, in order (cpu1, cpu2, cpu3): where the boot address goes",
    }],
    construct: |props| Ok(Box::new(Rst::new(props)?)),
};

impl Device for Rst {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.shared.state.lock() = State::default();
        self.shared.publish_bar();
        self.shared.drive();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "regs" => Some(Arc::clone(&self.control)),
            "bar" => Some(Arc::clone(&self.bar)),
            _ => None,
        }
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let n = match port {
            "hold1" => 0,
            "hold2" => 1,
            "hold3" => 2,
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: String::from(
                        "the reset control drives `hold1`, `hold2` and `hold3`, one per secondary core",
                    ),
                });
            }
        };
        self.shared.holds.lock()[n] = Some(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.drive();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.shared.state.lock().clone();
        for v in &s.words {
            w.write_u32(*v)?;
        }
        w.write_u32(s.bar)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut words = Vec::with_capacity(WORDS);
        for _ in 0..WORDS {
            words.push(r.read_u32()?);
        }
        let bar = r.read_u32()?;
        *self.shared.state.lock() = State { words, bar };
        self.shared.publish_bar();
        self.shared.drive();
        Ok(())
    }
}

impl Instance for Rst {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let cells = self
            .processors
            .iter()
            .map(|path| ctx.export_cell(path, ExportId::RESET_ADDRESS))
            .collect::<Result<Vec<_>>>()?;
        self.attach_cells(cells);
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Rst::new(props)?)))
}

/// The validator schema.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("processors", ValueKind::List))
        .region("")
        .region("regs")
        .region("bar")
        .port("hold1", PortDir::Out)
        .port("hold2", PortDir::Out)
        .port("hold3", PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::wire::{Wire, WireId, WireSink};

    #[derive(Debug, Default)]
    struct Probe(Mutex<Vec<bool>>);

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.0.lock().push(level.is_high());
        }
    }

    fn w(ops: &dyn MemOps, off: u64, v: u32) {
        ops.write(off, &v.to_le_bytes(), MemAttrs::DEFAULT).unwrap();
    }

    fn r(ops: &dyn MemOps, off: u64) -> u32 {
        let mut b = [0u8; 4];
        ops.read(off, &mut b, MemAttrs::DEFAULT).unwrap();
        u32::from_le_bytes(b)
    }

    fn rig() -> (Rst, Vec<Arc<Probe>>) {
        let rst = Rst::default();
        let mut probes = Vec::new();
        for n in 1..=3 {
            let probe = Arc::new(Probe::default());
            let src = WireId(n);
            let wire = Wire::builder()
                .source(src)
                .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
                .build_shared();
            rst.connect(&format!("hold{n}"), WireSource::new(wire, src))
                .unwrap();
            probes.push(probe);
        }
        (rst, probes)
    }

    #[test]
    fn only_core_zero_is_out_of_reset_at_power_on() {
        let (rst, probes) = rig();
        rst.announce("");
        let control = Control(Arc::clone(&rst.shared));
        assert_eq!(r(&control, 0), CTRL_RESET);
        for p in &probes {
            assert_eq!(p.0.lock().last(), Some(&true), "secondaries held");
        }
    }

    #[test]
    fn both_bits_release_a_core_and_clearing_either_holds_it() {
        let (rst, probes) = rig();
        let control = Control(Arc::clone(&rst.shared));
        // The kernel's sequence for core 2: clear its bits, power it, set them.
        let v = r(&control, 0);
        w(&control, 0, v & !(0x1100 << 2));
        w(&control, 0, v | (0x1100 << 2));
        assert_eq!(probes[1].0.lock().last(), Some(&false), "core 2 released");
        assert_eq!(probes[0].0.lock().last(), Some(&true), "core 1 still held");
        w(&control, 0, (v | (0x1100 << 2)) & !(1 << 14));
        assert_eq!(
            probes[1].0.lock().last(),
            Some(&true),
            "one bit is not enough"
        );
    }

    #[test]
    fn the_boot_address_reaches_every_core_and_zero_means_the_vector() {
        let rst = Rst::default();
        let cells: Vec<_> = (0..3).map(|_| Arc::new(AtomicU64::new(0))).collect();
        rst.attach_cells(cells.clone());
        assert!(
            cells
                .iter()
                .all(|c| c.load(Ordering::Acquire) == NO_ADDRESS)
        );
        let bar = BootAddress(Arc::clone(&rst.shared));
        w(&bar, 0, 0x62f8_6000);
        assert!(
            cells
                .iter()
                .all(|c| c.load(Ordering::Acquire) == 0x62f8_6000)
        );
        assert_eq!(r(&bar, 0), 0x62f8_6000);
        w(&bar, 0, 0);
        assert!(
            cells
                .iter()
                .all(|c| c.load(Ordering::Acquire) == NO_ADDRESS)
        );
    }

    #[test]
    fn a_snapshot_round_trips() {
        let rst = Rst::default();
        let control = Control(Arc::clone(&rst.shared));
        w(&control, 0, 0x3300);
        w(&control, 0xfb0, 0xc5ac_ce55);
        w(&BootAddress(Arc::clone(&rst.shared)), 0, 0x8000_d000);
        let save = |rst: &Rst| {
            let mut shape = MachineShape::new();
            shape.add_device("rst", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("rst", CLASS.name, CLASS.version).unwrap();
                rst.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&rst);
        let back = Rst::default();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("rst", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        let want = rst.shared.state.lock().clone();
        assert_eq!(*back.shared.state.lock(), want);
        assert_eq!(save(&back), bytes);
    }
}
