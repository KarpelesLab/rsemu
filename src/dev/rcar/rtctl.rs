//! The R-Car H1's control of its SH-4A "realtime domain" core: the word that
//! holds it in reset and the one that says where it starts.
//!
//! # Register map
//!
//! All accesses are 32-bit; the rest of the 4 KiB page is plain storage.
//!
//! | Offset | Here |
//! | --- | --- |
//! | `0x04` | bit 0 holds the SH-4A in reset while set (`hold` follows it); bit 1 is stored, its meaning unknown |
//! | `0x40` | the boot address, an SH physical address; reads back as the ARM sees that memory |
//!
//! The ARM writes the boot address (`0x16e00000`, the firmware's SH physical
//! address), then `3`, then `2` to `0x04`: clearing bit 0 starts the core.
//! The SH-4A starts at `0xa0000000 | address` — the uncached P2 view of it —
//! published through its [`RESET_ADDRESS`](ExportId::RESET_ADDRESS) cell.
//!
//! The SH's 29-bit physical space is the ARM's `0x80000000` upwards, so the
//! word reads back with bit 31 set: the realtime-API module reads it back
//! and adds the image's header offset, and only `0x96e00000` finds the
//! header \[strong\]. Whether the hardware stores the translated address or
//! computes it is not known; the reading is all a guest sees.
//!
//! Out of reset the core is held \[G\]: nothing on the ARM side starts it
//! before writing the boot address.
//!
//! # Sources
//!
//! No manual. The register facts come from the navi's binaries, disassembled
//! as data — the kernel modules that start the SH-4A and read the boot
//! address back — and none of their (GPL-licensed) code is reproduced.

use alloc::boxed::Box;
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
pub const CLASS_NAME: &str = "rcar.rtctl";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the block answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x1000;

/// High while the SH-4A is held in reset.
pub const HOLD_PIN: &str = "hold";

const WORDS: usize = (REGISTER_WINDOW_LEN / 4) as usize;

/// The reset control word.
pub const CONTROL: u64 = 0x04;
/// The boot address.
pub const BOOT_ADDRESS: u64 = 0x40;

/// Bit 0 of the control word: held in reset.
const HELD: u32 = 1;

/// Where the SH's 29-bit physical space sits in the ARM's.
const SH_WINDOW: u32 = 0x8000_0000;
/// The SH-4A's uncached P2 segment.
const P2: u32 = 0xa000_0000;

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    words: Vec<u32>,
}

impl Default for State {
    fn default() -> State {
        let mut words = alloc::vec![0; WORDS];
        words[(CONTROL / 4) as usize] = HELD;
        State { words }
    }
}

impl State {
    fn word(&self, offset: u64) -> u32 {
        self.words[(offset / 4) as usize]
    }

    fn held(&self) -> bool {
        self.word(CONTROL) & HELD != 0
    }

    /// The address the core starts at.
    fn start(&self) -> u32 {
        P2 | (self.word(BOOT_ADDRESS) & 0x1fff_ffff)
    }
}

#[derive(Debug)]
struct Shared {
    state: Mutex<State>,
    hold: Mutex<Option<WireSource>>,
    cell: Mutex<Option<Arc<AtomicU64>>>,
}

impl Shared {
    /// Publishes the start address and drives `hold`. The address goes
    /// first, so the core reads the new one when the hold falls.
    fn drive(&self) {
        let (held, start) = {
            let s = self.state.lock();
            (s.held(), s.start())
        };
        if let Some(cell) = self.cell.lock().clone() {
            cell.store(u64::from(start), Ordering::Release);
        }
        let hold = self.hold.lock().clone();
        if let Some(out) = hold {
            out.set(Level::from_bool(held));
        }
    }
}

#[derive(Debug)]
struct Registers(Arc<Shared>);

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        let offset = offset & !3;
        let s = self.0.state.lock();
        let v = match offset {
            BOOT_ADDRESS => s.word(offset) | SH_WINDOW,
            _ => s.word(offset),
        };
        dst.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 || attrs.debug {
            return Err(BusError::BadAccess);
        }
        let v = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        self.0.state.lock().words[((offset & !3) / 4) as usize] = v;
        self.0.drive();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

/// The SH-4A reset and boot-address control.
#[derive(Debug)]
pub struct RtCtl {
    shared: Arc<Shared>,
    region: RegionRef,
    processor: String,
}

impl RtCtl {
    /// Build it. `processor` names the SH-4A whose reset address it sets.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a missing `processor` or an unknown property.
    pub fn new(props: &Props) -> Result<RtCtl> {
        let mut r = props.reader();
        let processor = r
            .optional_link("processor")?
            .ok_or_else(|| Error::Property(String::from("processor: required")))?
            .as_str()
            .to_string();
        r.finish()?;
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            hold: Mutex::with_rank(LockRank::LEAF, None),
            cell: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::new(Registers(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        Ok(RtCtl {
            shared,
            region,
            processor,
        })
    }

    /// Hand it the core's reset-address cell (what binding does).
    pub fn attach_cell(&self, cell: Arc<AtomicU64>) {
        *self.shared.cell.lock() = Some(cell);
        self.shared.drive();
    }
}

/// The class descriptor.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the R-Car H1's SH-4A reset hold and boot address",
    properties: &[PropertySpec {
        name: "processor",
        kind: ValueKind::Link,
        required: true,
        summary: "the SH-4A instance it starts",
    }],
    construct: |props| Ok(Box::new(RtCtl::new(props)?)),
};

impl Device for RtCtl {
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
        if port != HOLD_PIN {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("the SH-4A control drives one pin, `hold`"),
            });
        }
        *self.shared.hold.lock() = Some(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        self.shared.drive();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.shared.state.lock().clone();
        for v in s.words {
            w.write_u32(v)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut words = Vec::with_capacity(WORDS);
        for _ in 0..WORDS {
            words.push(r.read_u32()?);
        }
        *self.shared.state.lock() = State { words };
        self.shared.drive();
        Ok(())
    }
}

impl Instance for RtCtl {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        self.attach_cell(ctx.export_cell(&self.processor, ExportId::RESET_ADDRESS)?);
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(RtCtl::new(props)?)))
}

/// The validator schema.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("processor", ValueKind::Link))
        .region("")
        .region("regs")
        .port(HOLD_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctl() -> RtCtl {
        let mut p = Props::new();
        p.insert(
            "processor",
            crate::core::props::Value::Link(crate::core::props::Link::new("sh").unwrap()),
        );
        RtCtl::new(&p).unwrap()
    }

    fn w(c: &RtCtl, off: u64, v: u32) {
        Registers(Arc::clone(&c.shared))
            .write(off, &v.to_le_bytes(), MemAttrs::DEFAULT)
            .unwrap();
    }

    fn r(c: &RtCtl, off: u64) -> u32 {
        let mut b = [0u8; 4];
        Registers(Arc::clone(&c.shared))
            .read(off, &mut b, MemAttrs::DEFAULT)
            .unwrap();
        u32::from_le_bytes(b)
    }

    #[test]
    fn the_kernel_starts_the_core() {
        let c = ctl();
        let cell = Arc::new(AtomicU64::new(0));
        c.attach_cell(Arc::clone(&cell));
        assert!(c.shared.state.lock().held(), "held out of reset");
        w(&c, BOOT_ADDRESS, 0x16e0_0000);
        w(&c, CONTROL, 3);
        assert!(c.shared.state.lock().held());
        assert_eq!(cell.load(Ordering::Acquire), 0xb6e0_0000, "P2, uncached");
        w(&c, CONTROL, 2);
        assert!(!c.shared.state.lock().held(), "released");
        assert_eq!(r(&c, CONTROL), 2, "bit 1 is kept");
    }

    #[test]
    fn the_boot_address_reads_back_as_the_arm_sees_it() {
        let c = ctl();
        w(&c, BOOT_ADDRESS, 0x16e0_0000);
        assert_eq!(r(&c, BOOT_ADDRESS) + 0xb00, 0x96e0_0b00, "the image header");
        w(&c, 0x800, 0x55);
        assert_eq!(r(&c, 0x800), 0x55, "storage");
    }
}
