//! The R-Car H1's inter-core mailbox between the Cortex-A9s and the SH-4A
//! "realtime domain" (ICCOM, after the driver that uses it): two doorbell
//! words, one each way, each raising an interrupt on the other side while it
//! is non-zero, and the busy and status words the two sides keep beside them.
//! The messages themselves travel through shared DRAM the doorbells point at.
//!
//! # Register map
//!
//! All accesses are 32-bit. The page also holds the Cortex-A9 secondary
//! boot address at `0x40`, which belongs to `rcar.rst` and is mapped over
//! this window.
//!
//! | Offset | Written by | Meaning |
//! | --- | --- | --- |
//! | `0x04` | SH | 0 normal, 1 fatal (read with `EICR` bit 7) |
//! | `0x20` | SH | busy bits of the two ARM → SH buffers |
//! | `0x28` | ARM | busy bits of the two SH → ARM buffers |
//! | `0x30` | SH | `EICR`, the SH → ARM doorbell: `arm-irq` is up while it is non-zero |
//! | `0x70` | ARM | `IICR`, the ARM → SH doorbell: `sh-irq` is up while it is non-zero |
//!
//! A doorbell word: bits 1:0 `3` when a message segment is waiting, bits 5:4
//! the segment (single, first, middle, last), bit 6 which of the direction's
//! two buffers; on `EICR` only, bit 2 "booted" (the SH writes `0x07` once)
//! and bit 7 fatal. The receiver writes the word back to 0 when it has the
//! segment, or to `0x100` to refuse it, and a sender waits for 0. Everything
//! else in the window is plain storage.
//!
//! Neither side's interrupt is latched here: each is the level of its
//! doorbell, which is what both drivers' handlers assume — they read the
//! word, act, and clear it.
//!
//! # Sources
//!
//! No manual. Read off the navi's binaries, disassembled as data: the ARM
//! kernel module's boot wait and its send and receive paths, and the SH-4A
//! firmware's matching ones. Only the register facts above were taken; the
//! module is GPL-licensed and none of its code is reproduced.

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
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.iccom";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the block answers.
pub const REGISTER_WINDOW_LEN: u64 = 0x1000;

/// The interrupt towards the Cortex-A9s (the GIC).
pub const ARM_IRQ_PIN: &str = "arm-irq";
/// The interrupt towards the SH-4A (its interrupt controller).
pub const SH_IRQ_PIN: &str = "sh-irq";

const WORDS: usize = (REGISTER_WINDOW_LEN / 4) as usize;

/// The SH → ARM doorbell.
pub const EICR: u64 = 0x30;
/// The ARM → SH doorbell.
pub const IICR: u64 = 0x70;

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    words: Vec<u32>,
}

impl Default for State {
    fn default() -> State {
        State {
            words: alloc::vec![0; WORDS],
        }
    }
}

impl State {
    fn word(&self, offset: u64) -> u32 {
        self.words[(offset / 4) as usize]
    }
}

#[derive(Debug)]
struct Shared {
    state: Mutex<State>,
    arm: Mutex<Option<WireSource>>,
    sh: Mutex<Option<WireSource>>,
}

impl Shared {
    /// Puts both interrupt levels where the doorbells say.
    fn drive(&self) {
        let (eicr, iicr) = {
            let s = self.state.lock();
            (s.word(EICR), s.word(IICR))
        };
        let arm = self.arm.lock().clone();
        let sh = self.sh.lock().clone();
        if let Some(out) = arm {
            out.set(Level::from_bool(eicr != 0));
        }
        if let Some(out) = sh {
            out.set(Level::from_bool(iicr != 0));
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
        // No read has a side effect.
        let v = self.0.state.lock().word(offset & !3);
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

/// The ICCOM mailbox.
#[derive(Debug)]
pub struct Iccom {
    shared: Arc<Shared>,
    region: RegionRef,
}

impl Iccom {
    /// Build it. It takes no properties.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if any property is given.
    pub fn new(props: &Props) -> Result<Iccom> {
        props.reader().finish()?;
        Ok(Iccom::default())
    }
}

impl Default for Iccom {
    fn default() -> Iccom {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            arm: Mutex::with_rank(LockRank::LEAF, None),
            sh: Mutex::with_rank(LockRank::LEAF, None),
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::new(Registers(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        Iccom { shared, region }
    }
}

/// The class descriptor.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "the R-Car H1 ARM <-> SH-4A mailbox: two doorbells, each an interrupt on the other side while non-zero",
    properties: &[],
    construct: |props| Ok(Box::new(Iccom::new(props)?)),
};

impl Device for Iccom {
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
        let slot = match port {
            ARM_IRQ_PIN => &self.shared.arm,
            SH_IRQ_PIN => &self.shared.sh,
            _ => {
                return Err(Error::Config {
                    at: port.to_string(),
                    message: String::from("the ICCOM mailbox drives `arm-irq` and `sh-irq`"),
                });
            }
        };
        *slot.lock() = Some(source);
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

impl Instance for Iccom {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Iccom::new(props)?)))
}

/// The validator schema.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir};
    ClassSchema::new(CLASS_NAME)
        .region("")
        .region("regs")
        .port(ARM_IRQ_PIN, PortDir::Out)
        .port(SH_IRQ_PIN, PortDir::Out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn w(m: &Iccom, off: u64, v: u32) {
        Registers(Arc::clone(&m.shared))
            .write(off, &v.to_le_bytes(), MemAttrs::DEFAULT)
            .unwrap();
    }

    fn r(m: &Iccom, off: u64) -> u32 {
        let mut b = [0u8; 4];
        Registers(Arc::clone(&m.shared))
            .read(off, &mut b, MemAttrs::DEFAULT)
            .unwrap();
        u32::from_le_bytes(b)
    }

    fn levels(m: &Iccom) -> (bool, bool) {
        let s = m.shared.state.lock();
        (s.word(EICR) != 0, s.word(IICR) != 0)
    }

    #[test]
    fn the_boot_handshake() {
        // The ARM presets IICR before releasing the SH; the SH clears it,
        // then rings EICR with "booted"; the ARM sees it and clears it.
        let m = Iccom::default();
        w(&m, EICR, 0);
        w(&m, IICR, 0x50);
        assert_eq!(levels(&m), (false, true), "the SH would see a request");
        w(&m, IICR, 0);
        w(&m, 0x04, 0);
        w(&m, EICR, 0x07);
        assert_eq!(levels(&m), (true, false));
        assert_ne!(r(&m, EICR) & 4, 0, "what the ARM polls for");
        w(&m, EICR, 0);
        assert_eq!(levels(&m), (false, false));
    }

    #[test]
    fn a_refused_segment_keeps_the_doorbell_up() {
        let m = Iccom::default();
        w(&m, IICR, 0x33);
        w(&m, IICR, 0x100);
        assert_eq!(r(&m, IICR), 0x100);
        assert!(levels(&m).1, "non-zero is still a level");
    }

    #[test]
    fn the_rest_of_the_page_is_storage() {
        let m = Iccom::default();
        w(&m, 0x20, 2);
        w(&m, 0x60, 0xdead_beef);
        assert_eq!((r(&m, 0x20), r(&m, 0x60)), (2, 0xdead_beef));
        assert_eq!(levels(&m), (false, false));
    }

    #[test]
    fn a_snapshot_round_trips() {
        let m = Iccom::default();
        w(&m, EICR, 0x07);
        w(&m, 0x28, 1);
        let save = |m: &Iccom| {
            let mut shape = MachineShape::new();
            shape.add_device("iccom", CLASS.name).unwrap();
            let mut wr = StateWriter::new(shape);
            {
                let mut chunk = wr.chunk("iccom", CLASS.name, CLASS.version).unwrap();
                m.save(&mut chunk).unwrap();
            }
            wr.to_vec().unwrap()
        };
        let bytes = save(&m);
        let back = Iccom::default();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("iccom", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        back.load(&mut chunk.reader()).unwrap();
        assert_eq!(*back.shared.state.lock(), *m.shared.state.lock());
        assert_eq!(save(&back), bytes);
    }
}
