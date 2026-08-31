//! OAM DMA at `$4014` — the sprite copy that stalls the CPU.
//!
//! Source: the NESdev wiki, [DMA](https://www.nesdev.org/wiki/DMA) and
//! [PPU registers](https://www.nesdev.org/wiki/PPU_registers) (`OAMDATA`).
//!
//! # What the hardware does
//!
//! A write of `$XX` to `$4014` starts a unit inside the RP2A03 that copies the
//! 256 bytes at `$XX00`-`$XXFF` into the PPU's object attribute memory, one
//! byte at a time, by *reading* the source address and *writing* `$2004`. It is
//! not a block move: every byte goes through the ordinary bus, so it lands in
//! OAM at whatever `OAMADDR` currently holds and rotates the copy if that is
//! not zero — which several games rely on and one AccuracyCoin test checks.
//!
//! While it runs the CPU is halted, not merely slowed:
//!
//! ```text
//!   1 cycle    the write to $4014 itself
//!   1 cycle    a dummy read while the unit takes the bus
//!  +1 cycle    only if the halt lands on an odd ("put") CPU cycle
//! 512 cycles   256 read/write pairs
//! ```
//!
//! — 513 or 514 cycles of halt after the write, the odd one depending on the
//! alignment of the CPU's own cycle counter.
//!
//! # What is here, and what is not
//!
//! **The transfer is complete.** [`OamDma`] is a bus master: it reads each
//! source byte and writes it to `$2004` through the CPU's own address space, so
//! `OAMADDR` advances, the PPU's own catch-up runs on every write, and a source
//! page in ROM, work RAM or MMIO behaves exactly as the bus says it should.
//!
//! **The CPU stall is a seam.** Halting the 6502 needs the core's cooperation —
//! on real hardware the DMA unit pulls `RDY` low — and `cpu.mos6502` models no
//! `RDY` pin and has no way to be handed cycles from outside. So this device
//! *records* what it is owed, in [`OamDma::stall_owed`] and
//! [`OamDma::take_stall`], and nothing yet drains it. Closing the seam is a
//! change in `src/cpu/mos6502`, and it is one of two shapes:
//!
//! 1. a `rdy` sink pin on the core, driven by this device, that makes
//!    `run_budget` consume ticks without retiring instructions; or
//! 2. a `stall(cycles)` callback the core checks after every store, which is
//!    what [`take_stall`](OamDma::take_stall) is shaped for.
//!
//! [`take_stall`](OamDma::take_stall) returns [`TRANSFER_CYCLES`] — 513 — and
//! **not** the alignment cycle, because the only thing that knows whether the
//! halt lands on an odd CPU cycle is the CPU. Whichever seam lands adds
//! `cycles % 2` itself; see [`OamDma::take_stall`].
//!
//! Until then a machine copies its sprites correctly and runs 513 cycles per
//! frame too fast during the copy. Sprites appear; a test that counts cycles
//! across a DMA does not pass.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::{Arc, Weak};
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Result};
use crate::core::props::Props;
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
    RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description would use.
const CLASS_NAME: &str = "nes.oamdma";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The name of the region that decodes `$4014`.
pub const PORT: &str = "port";

/// How many bytes one transfer moves — the whole of OAM.
pub const OAM_LEN: u64 = 256;

/// Where the copied bytes are written: `OAMDATA`.
///
/// An address in the CPU's space, not an offset into the PPU: the unit really
/// does drive `$2004` on the bus, which is why the copy honours `OAMADDR` and
/// why a machine that maps the PPU somewhere else would need this to follow.
/// The NES maps the register block at `$2000` and no NES does otherwise, so it
/// is a constant rather than a property.
pub const OAM_DATA_ADDR: u64 = 0x2004;

/// CPU cycles one transfer halts the core for, excluding the alignment cycle.
///
/// One dummy read plus 256 read/write pairs. The extra cycle spent waiting for
/// an even ("get") cycle is the CPU's to add — see
/// [`OamDma::take_stall`].
pub const TRANSFER_CYCLES: u64 = 1 + 2 * OAM_LEN;

/// What the device and its memory port both hold.
struct Shared {
    /// Everything mutable, at `DEVICE` rank. Never held across the transfer:
    /// the bus handle is cloned out and the lock released first
    /// (`CLAUDE.md`, re-entrancy).
    state: Mutex<State>,
}

/// The unit's own state.
#[derive(Debug, Clone)]
struct State {
    /// The CPU's address space, as a bus master reaches it.
    ///
    /// `Weak` rather than `Arc`: this device is *inside* the space it reads
    /// through — the space owns the region that owns this — and a strong
    /// reference would be a cycle the machine could never drop.
    bus: Option<Weak<AddressSpace>>,
    /// The requester id accesses from this unit carry.
    requester: RequesterId,
    /// The last page written to `$4014`, which the register reads back as on
    /// hardware only through open bus — kept because it is the one piece of
    /// architectural state a snapshot would otherwise lose.
    page: u8,
    /// CPU cycles owed but not yet taken. See the [module docs](self).
    owed: u64,
    /// Transfers completed, for diagnostics and for the seam's own tests.
    transfers: u64,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let state = self.state.lock();
        f.debug_struct("Shared")
            .field("page", &state.page)
            .field("owed", &state.owed)
            .field("transfers", &state.transfers)
            .finish_non_exhaustive()
    }
}

impl Shared {
    /// Run one transfer from `page`, and record the stall it should have cost.
    ///
    /// The critical section ends before the first bus access: 512 accesses with
    /// this device's own lock held would put a `DEVICE`-ranked lock above the
    /// PPU's engine lock and above the address space's topology guard, which is
    /// the lock-order violation `core::sync` exists to catch.
    fn transfer(&self, page: u8) -> MemResult {
        let (bus, attrs) = {
            let mut state = self.state.lock();
            state.page = page;
            state.owed += TRANSFER_CYCLES;
            state.transfers += 1;
            (
                state.bus.as_ref().and_then(Weak::upgrade),
                MemAttrs::DEFAULT.with_requester(state.requester),
            )
        };
        // No space bound: `Instance::bind` refuses a machine that gets here, so
        // this is a hand-wired caller that forgot `attach_bus`. Refusing the
        // access is more honest than copying nothing and reporting success.
        let bus = bus.ok_or(BusError::BadAccess)?;

        let base = u64::from(page) << 8;
        for offset in 0..OAM_LEN {
            // A real read and a real write, one byte at a time. Not a bulk copy:
            // the source may be MMIO, and the destination is a register whose
            // write has side effects (`OAMADDR` advances).
            let byte = bus.read(base | offset, Width::U8, attrs)?;
            bus.write(OAM_DATA_ADDR, Width::U8, byte, attrs)?;
        }
        Ok(())
    }
}

/// The RP2A03's OAM DMA unit.
///
/// Cloneable handles onto one piece of hardware: [`Device::region`] hands the
/// one-byte `$4014` aperture to the address space while the machine keeps the
/// device.
#[derive(Debug)]
pub struct OamDma {
    shared: Arc<Shared>,
    /// `$4014`, built once at construction so two `map` statements naming it
    /// get one region.
    port: RegionRef,
}

impl OamDma {
    /// Validate properties and allocate. Performs no outward action.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Property`] for an unknown or ill-typed property.
    pub fn new(props: &Props) -> Result<OamDma> {
        props.reader().finish()?;
        Ok(OamDma::default())
    }

    /// Connect the CPU's address space, which this unit masters.
    ///
    /// The machine layer calls this from [`Instance::bind`]; a caller wiring a
    /// NES by hand calls it directly. Without it a write to `$4014` is a bus
    /// error rather than a silent no-op.
    pub fn attach_bus(&self, space: &Arc<AddressSpace>, requester: RequesterId) {
        let mut state = self.shared.state.lock();
        state.bus = Some(Arc::downgrade(space));
        state.requester = requester;
    }

    /// CPU cycles this unit has stalled the core for and not been drained of.
    ///
    /// See the [module docs](self): nothing drains it yet, so on a running
    /// machine this grows by [`TRANSFER_CYCLES`] per transfer and is the exact
    /// measure of how far ahead of real hardware the CPU is.
    #[must_use]
    pub fn stall_owed(&self) -> u64 {
        self.shared.state.lock().owed
    }

    /// Take the stall this unit owes, leaving nothing behind.
    ///
    /// **The interface the CPU needs.** A core that checked this after every
    /// store would charge itself the halt:
    ///
    /// ```ignore
    /// let owed = dma.take_stall();
    /// if owed != 0 {
    ///     // The unit waits for an even ("get") cycle before it starts, so a
    ///     // halt that lands on an odd cycle costs one more. Only the CPU
    ///     // knows its own parity, which is why it is added here and not
    ///     // inside the device.
    ///     self.cycles += owed + (self.cycles & 1);
    /// }
    /// ```
    ///
    /// `cpu.mos6502` has no such hook yet, so nothing calls this outside tests.
    pub fn take_stall(&self) -> u64 {
        let mut state = self.shared.state.lock();
        core::mem::take(&mut state.owed)
    }

    /// How many transfers have run since power-on.
    #[must_use]
    pub fn transfers(&self) -> u64 {
        self.shared.state.lock().transfers
    }

    /// The last page written to `$4014`.
    #[must_use]
    pub fn page(&self) -> u8 {
        self.shared.state.lock().page
    }
}

impl Default for OamDma {
    fn default() -> OamDma {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(
                LockRank::DEVICE,
                State {
                    bus: None,
                    requester: RequesterId::ANONYMOUS,
                    page: 0,
                    owed: 0,
                    transfers: 0,
                },
            ),
        });
        let port = Arc::new(MmioRegion::io(
            "nes.oamdma.4014",
            1,
            Arc::new(DmaPort {
                shared: Arc::clone(&shared),
            }) as Arc<dyn MemOps>,
        ));
        OamDma { shared, port }
    }
}

/// The one-byte window onto an [`OamDma`].
#[derive(Debug)]
struct DmaPort {
    shared: Arc<Shared>,
}

impl MemOps for DmaPort {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let ([byte], 0) = (dst, offset) else {
            return Err(BusError::BadAccess);
        };
        // `$4014` is write-only: nothing drives the bus on a read, so the master
        // gets back the byte it last drove itself. For an ordinary `LDA $4014`
        // that is `$40`, the high byte of its own operand.
        *byte = attrs.bus;
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let ([value], 0) = (src, offset) else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // A debug write would move 256 bytes and halt the CPU. The monitor
            // has to go through the device's own API to say it meant it.
            return Ok(());
        }
        self.shared.transfer(*value)
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U8, Endian::Little)
    }
}

impl Device for OamDma {
    fn class(&self) -> &'static DeviceClass {
        &OAM_DMA_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward. The unit is placed by a `map` statement and handed
        // its bus at bind time, which runs after every region is mapped —
        // exactly the ordering a bus master needs.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // /RES aborts a transfer in progress and clears the unit. The bus
        // handle is wiring, not state, and survives.
        let mut state = self.shared.state.lock();
        state.page = 0;
        state.owed = 0;
        state.transfers = 0;
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock();
        w.write_u8(state.page)?;
        w.write_u64(state.owed)?;
        w.write_u64(state.transfers)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let page = r.read_u8()?;
        let owed = r.read_u64()?;
        let transfers = r.read_u64()?;
        let mut state = self.shared.state.lock();
        state.page = page;
        state.owed = owed;
        state.transfers = transfers;
        Ok(())
    }

    /// The `$4014` aperture.
    ///
    /// The empty name gets it too: the unit decodes exactly one byte, so
    /// `map cpubus 0x4014 size 1 = dma` is unambiguous.
    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            PORT | "" => Some(Arc::clone(&self.port)),
            _ => None,
        }
    }
}

/// The machine layer's half: the unit is a bus master and needs a space.
impl Instance for OamDma {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| crate::core::Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "OAM DMA masters the CPU bus: add `space = cpubus` to the object that declares it",
            ),
        })?;
        self.attach_bus(space, ctx.requester());
        Ok(())
    }
}

/// The properties [`OAM_DMA_CLASS`] accepts: none.
static OAM_DMA_PROPERTIES: &[PropertySpec] = &[];

/// The device class, as `nes.oamdma` in a machine description.
pub static OAM_DMA_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "NES OAM DMA ($4014): copies a page into OAM through $2004, 513 cycles of CPU halt",
    properties: OAM_DMA_PROPERTIES,
    construct: |props| Ok(Box::new(OamDma::new(props)?) as Box<dyn Device>),
};

/// Add [`OAM_DMA_CLASS`] to a registry.
///
/// # Errors
///
/// [`crate::Error::Config`] if the class name is already taken.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&OAM_DMA_CLASS)
}

/// Bind [`OAM_DMA_CLASS`] into the machine graph.
///
/// # Errors
///
/// [`crate::Error::Config`] if the class name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(OamDma::new(props)?)))
}

/// What the validator should know about `nes.oamdma`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::ClassSchema;
    ClassSchema::new(CLASS_NAME).region(PORT)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{RamStore, Region};
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use alloc::vec::Vec;

    /// A CPU bus with 2 KiB of work RAM mirrored over `$0000-$1FFF`, a fake
    /// `$2004` that records what is written to it, and the unit at `$4014`.
    struct Bus {
        space: Arc<AddressSpace>,
        dma: OamDma,
        oam: Arc<Oam>,
    }

    /// Stands in for the PPU's `OAMDATA`: appends every byte written.
    #[derive(Debug, Default)]
    struct Oam {
        written: Mutex<Vec<u8>>,
    }

    impl MemOps for Oam {
        fn read(&self, _offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
            for byte in dst.iter_mut() {
                *byte = 0;
            }
            Ok(())
        }

        fn write(&self, _offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
            self.written.lock().extend_from_slice(src);
            Ok(())
        }

        fn constraints(&self) -> AccessConstraints {
            AccessConstraints::word(Width::U8, Endian::Little)
        }
    }

    fn bus() -> Bus {
        let space = Arc::new(AddressSpace::new("cpubus", 16));
        let ram = Arc::new(RamStore::new(0x800));
        let oam = Arc::new(Oam::default());
        let dma = OamDma::default();
        {
            let mut topo = space.topology();
            topo.map(
                Arc::new(
                    Region::mirror("wram", Arc::new(Region::ram("ram", ram)), 0x2000)
                        .expect("mirrors"),
                ),
                0,
            )
            .expect("maps");
            // Eight registers, mirrored the way the PPU's block is, so a write
            // to $2004 lands on the recorder.
            topo.map(
                Arc::new(MmioRegion::io(
                    "oamdata",
                    0x2000,
                    Arc::clone(&oam) as Arc<dyn MemOps>,
                )),
                0x2000,
            )
            .expect("maps");
            topo.map(dma.region(PORT).expect("port"), 0x4014)
                .expect("maps");
        }
        dma.attach_bus(&space, RequesterId::ANONYMOUS);
        Bus { space, dma, oam }
    }

    fn wr(space: &AddressSpace, addr: u64, value: u8) {
        space
            .write(addr, Width::U8, u64::from(value), MemAttrs::DEFAULT)
            .unwrap_or_else(|e| panic!("write {addr:#06x}: {e}"));
    }

    #[test]
    fn a_write_copies_two_hundred_and_fifty_six_bytes_through_2004() {
        let b = bus();
        for i in 0..256u64 {
            wr(&b.space, 0x0200 + i, (i as u8) ^ 0x5a);
        }
        wr(&b.space, 0x4014, 0x02);

        let written = b.oam.written.lock().clone();
        assert_eq!(written.len(), 256);
        for (i, byte) in written.iter().enumerate() {
            assert_eq!(*byte, (i as u8) ^ 0x5a, "byte {i}");
        }
        assert_eq!(b.dma.page(), 0x02);
        assert_eq!(b.dma.transfers(), 1);
    }

    #[test]
    fn the_source_page_is_the_written_byte_shifted_up() {
        let b = bus();
        // $0700, in the fourth mirror of work RAM.
        wr(&b.space, 0x0700, 0xc3);
        wr(&b.space, 0x4014, 0x07);
        assert_eq!(b.oam.written.lock()[0], 0xc3);
    }

    #[test]
    fn the_stall_is_recorded_and_drained_once() {
        let b = bus();
        assert_eq!(b.dma.stall_owed(), 0);
        wr(&b.space, 0x4014, 0x00);
        // 1 dummy read + 256 read/write pairs. The alignment cycle is the
        // CPU's, so it is not here.
        assert_eq!(b.dma.stall_owed(), 513);
        wr(&b.space, 0x4014, 0x00);
        assert_eq!(b.dma.stall_owed(), 1026, "two transfers accumulate");
        assert_eq!(b.dma.take_stall(), 1026);
        assert_eq!(b.dma.stall_owed(), 0, "draining leaves nothing");
    }

    #[test]
    fn a_debug_write_moves_nothing() {
        let b = bus();
        b.space
            .write(0x4014, Width::U8, 0x02, MemAttrs::DEBUG)
            .expect("accepted");
        assert!(b.oam.written.lock().is_empty());
        assert_eq!(b.dma.stall_owed(), 0);
        assert_eq!(b.dma.transfers(), 0);
    }

    #[test]
    fn the_register_reads_as_open_bus() {
        let b = bus();
        // Whatever the master last drove — for `LDA $4014` that is `$40`, the
        // high byte of its own operand.
        let value = b
            .space
            .read(0x4014, Width::U8, MemAttrs::DEFAULT.with_bus(0x40))
            .expect("answered");
        assert_eq!(value, 0x40, "$4014 is write-only");
        let value = b
            .space
            .read(0x4014, Width::U8, MemAttrs::DEFAULT.with_bus(0xa5))
            .expect("answered");
        assert_eq!(value, 0xa5, "and it really is the bus, not a constant");
    }

    #[test]
    fn an_unbound_bus_is_refused_rather_than_ignored() {
        let dma = OamDma::default();
        let space = AddressSpace::new("cpubus", 16);
        space
            .topology()
            .map(dma.region(PORT).expect("port"), 0x4014)
            .expect("maps");
        assert!(
            space
                .write(0x4014, Width::U8, 0x02, MemAttrs::DEFAULT)
                .is_err()
        );
    }

    #[test]
    fn the_device_does_not_keep_its_own_space_alive() {
        // The unit is inside the space it masters, so a strong reference would
        // be a cycle the machine could never drop.
        let b = bus();
        let weak = Arc::downgrade(&b.space);
        let Bus { space, dma, oam } = b;
        drop(space);
        drop(oam);
        assert!(weak.upgrade().is_none(), "the space leaked");
        // And the device notices, rather than copying from nowhere.
        assert!(dma.shared.transfer(0).is_err());
    }

    #[test]
    fn state_round_trips() {
        let b = bus();
        wr(&b.space, 0x4014, 0x03);

        let mut shape = MachineShape::new();
        shape.add_device("dma", CLASS_NAME).expect("unique path");
        let mut writer = StateWriter::new(shape);
        let mut chunk = writer
            .chunk("dma", CLASS_NAME, STATE_VERSION)
            .expect("one chunk");
        b.dma.save(&mut chunk).expect("saves");
        let bytes = writer.to_vec().expect("encodes");

        let other = OamDma::default();
        let reader = StateReader::new(&bytes).expect("decodes");
        let chunk = reader
            .load("dma", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .expect("finds the chunk");
        other.load(&mut chunk.reader()).expect("loads");
        assert_eq!(other.page(), b.dma.page());
        assert_eq!(other.stall_owed(), b.dma.stall_owed());
        assert_eq!(other.transfers(), b.dma.transfers());
    }

    #[test]
    fn a_reset_clears_the_unit() {
        let b = bus();
        wr(&b.space, 0x4014, 0x03);
        b.dma.reset(ResetKind::Cold);
        assert_eq!(b.dma.page(), 0);
        assert_eq!(b.dma.stall_owed(), 0);
        // The bus handle is wiring, not state: it survives, so the next write
        // still copies.
        wr(&b.space, 0x4014, 0x00);
        assert_eq!(b.dma.transfers(), 1);
    }

    #[test]
    fn an_unknown_property_is_refused() {
        let e = OamDma::new(&Props::new().with("page", 3u64)).expect_err("no properties");
        assert!(alloc::format!("{e}").contains("page"), "{e}");
    }
}
