//! The virtio over MMIO transport.
//!
//! # Source
//!
//! *Virtual I/O Device (VIRTIO) Version 1.2*, OASIS Standard: §4.2 ("Virtio
//! Over MMIO") for the register block below, §2.1 for the device status
//! handshake, §6 for the reserved feature bits. Nothing else — see
//! [`queue`](super::queue) for why no driver source was opened.
//!
//! # The register block
//!
//! ```text
//!   0x000 MagicValue  R    "virt"          0x070 Status          RW
//!   0x004 Version     R    2 (modern)      0x080 QueueDescLow    W
//!   0x008 DeviceID    R                    0x084 QueueDescHigh   W
//!   0x00c VendorID    R                    0x090 QueueDriverLow  W
//!   0x010 DeviceFeatures    R              0x094 QueueDriverHigh W
//!   0x014 DeviceFeaturesSel W              0x0a0 QueueDeviceLow  W
//!   0x020 DriverFeatures    W              0x0a4 QueueDeviceHigh W
//!   0x024 DriverFeaturesSel W              0x0fc ConfigGeneration R
//!   0x030 QueueSel    W                    0x100 device configuration
//!   0x034 QueueNumMax R
//!   0x038 QueueNum    W
//!   0x044 QueueReady  RW
//!   0x050 QueueNotify W
//!   0x060 InterruptStatus R
//!   0x064 InterruptACK    W
//! ```
//!
//! # Version 2 only, and why that is a feature
//!
//! `Version` reads 2, so a legacy driver walks away rather than programming a
//! `GuestPageSize` register this does not have. The legacy layout is a
//! different device with the same magic number, and half-implementing it
//! produces a machine that boots one kernel and corrupts another's disk.
//!
//! # When work happens
//!
//! A write to `QueueNotify` processes every available chain, synchronously,
//! inside the guest's own store instruction. That is the re-entrancy contract
//! of `ROADMAP.md` §4.7 taken at its word: the transport's state lock is
//! released *before* the backend is called, because the backend performs DMA
//! through the same address space the notify arrived on.
//!
//! And because it is the *same* address space, a driver can point a descriptor
//! at this device's own registers and be re-entered from inside a transfer. So
//! the work is **iterative, not recursive**, exactly as `dev/nvme`'s doorbell
//! engine is: the transport's notify path argues it where it bites.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;
use core::sync::atomic::{self, AtomicBool, AtomicU64};

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::space::{AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult};
use crate::core::space::{Region, RegionRef, RequesterId};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireSource};
use crate::machine::realize::{BindCtx, Instance};

use super::super::dt::{DtSource, NodeSpec};
use super::queue::{Descriptor, Layout, QUEUE_SIZE_MAX, Queue};
use super::{Backend, VENDOR_ID};

/// How much address space one virtio-mmio device occupies.
///
/// The registers end at `0x100` and the configuration space follows; 4 KiB is
/// what boards conventionally give each one, which also means one page.
pub const REGISTER_WINDOW_LEN: u64 = 0x1000;

/// Where the device-specific configuration space starts (§4.2.2).
pub const CONFIG_OFFSET: u64 = 0x100;

/// `MagicValue`: the ASCII bytes `virt`, little-endian (§4.2.2).
pub const MAGIC: u32 = 0x7472_6976;

/// The transport version this implements. 2 is the non-legacy interface.
pub const VERSION: u32 = 2;

// -- Status bits (§2.1) -----------------------------------------------------

/// The guest has noticed the device.
const STATUS_ACKNOWLEDGE: u32 = 1;
/// The guest has a driver for it.
const STATUS_DRIVER: u32 = 2;
/// The driver is done setting up and the device may be used.
const STATUS_DRIVER_OK: u32 = 4;
/// The driver has accepted a feature set the device can live with.
const STATUS_FEATURES_OK: u32 = 8;
/// Something went wrong that only a reset will clear.
const STATUS_NEEDS_RESET: u32 = 64;
/// The driver has given up on the device.
const STATUS_FAILED: u32 = 128;

/// `VIRTIO_F_VERSION_1` (§6): bit 32, and a modern device offers nothing
/// without it.
pub const F_VERSION_1: u64 = 1 << 32;

// -- InterruptStatus bits (§4.2.2) ------------------------------------------

/// The device has put something in a used ring.
const INT_USED_BUFFER: u32 = 1;
/// The configuration space changed.
const INT_CONFIG_CHANGE: u32 = 2;

/// Everything the driver can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    device_features_sel: u32,
    driver_features_sel: u32,
    /// The two halves of the 64-bit acknowledged feature set.
    driver_features: [u32; 2],
    queue_sel: u32,
    queues: Vec<QueueState>,
    status: u32,
    interrupt_status: u32,
    config_generation: u32,
}

/// One queue's configuration and the device's position in it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct QueueState {
    layout: Layout,
    /// How far into the available ring the device has consumed.
    last_avail: u16,
    /// The next index the device will write into the used ring.
    used_idx: u16,
}

impl State {
    fn new(queues: usize) -> State {
        State {
            device_features_sel: 0,
            driver_features_sel: 0,
            driver_features: [0; 2],
            queue_sel: 0,
            queues: alloc::vec![QueueState::default(); queues],
            status: 0,
            interrupt_status: 0,
            config_generation: 0,
        }
    }
}

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// The interrupt output and the DMA space, at [`LockRank::LEAF`] so they
    /// can be taken with nothing else held.
    links: Mutex<Links>,
    backend: Arc<dyn Backend>,
    /// Queues with a notification outstanding, one bit each.
    ///
    /// Derived state: it is what a `QueueNotify` write leaves behind for the
    /// engine to pick up, and it is empty whenever the engine is not running,
    /// so it is never serialized. See [`Registers::notify`].
    pending: AtomicU64,
    /// Whether a [`notify`](Registers::notify) pass is already in progress.
    engine: AtomicBool,
}

/// The most queues one virtio device can have here, because [`Registers`]
/// tracks outstanding notifications in a `u64`. Every device in this tree has
/// one; `VIRTIO_BLK_F_MQ` and a multiqueue NIC are what would push at it.
const MAX_QUEUES: usize = u64::BITS as usize;

/// What the machine gave this device.
#[derive(Debug, Default)]
struct Links {
    out: Option<WireSource>,
    space: Option<Arc<AddressSpace>>,
    requester: RequesterId,
    /// The net the interrupt pin drives, so the device tree can look its
    /// number up in the PLIC's pin table. See [`dt`](super::super::dt).
    irq_wire: Option<crate::core::wire::WireId>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("backend", &self.backend);
        match self.state.try_lock() {
            Some(state) => s.field("status", &state.status).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

/// A virtio device on the MMIO transport.
#[derive(Debug)]
pub struct VirtioMmio {
    regs: Arc<Registers>,
    region: RegionRef,
    class: &'static DeviceClass,
}

impl VirtioMmio {
    /// Wrap `backend` in the MMIO transport.
    #[must_use]
    pub fn new(backend: Arc<dyn Backend>, class: &'static DeviceClass) -> VirtioMmio {
        let queues = backend.queue_count().min(MAX_QUEUES);
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(queues)),
            links: Mutex::with_rank(LockRank::LEAF, Links::default()),
            backend,
            pending: AtomicU64::new(0),
            engine: AtomicBool::new(false),
        });
        let region: RegionRef = Arc::new(Region::io(
            "virtio.mmio",
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        VirtioMmio {
            regs,
            region,
            class,
        }
    }

    /// The backend this transport carries.
    #[must_use]
    pub fn backend(&self) -> &Arc<dyn Backend> {
        &self.regs.backend
    }

    /// The device status register, as the driver last wrote it.
    #[must_use]
    pub fn status(&self) -> u32 {
        self.regs.state.lock().status
    }

    /// Whether the interrupt line is asserted.
    #[must_use]
    pub fn irq_asserted(&self) -> bool {
        self.regs.state.lock().interrupt_status != 0
    }

    /// Give the device the address space its DMA traverses.
    ///
    /// A realized machine does this through [`Instance::bind`]; a test that
    /// wires one by hand calls it directly.
    pub fn attach_space(&self, space: Arc<AddressSpace>, requester: RequesterId) {
        let mut links = self.regs.links.lock();
        links.space = Some(space);
        links.requester = requester;
    }

    /// Process queue `index` as a `QueueNotify` write would.
    pub fn notify(&self, index: u32) {
        self.regs.notify(index);
    }

    /// Tell the driver the configuration space changed (§4.2.2).
    ///
    /// Bumps `ConfigGeneration` so a driver reading a multi-word configuration
    /// can tell that it straddled a change, and raises the configuration-change
    /// interrupt. Nothing in this build's two backends changes its
    /// configuration while running; a resizable disk or a network device whose
    /// link goes down would.
    pub fn signal_config_change(&self) {
        {
            let mut state = self.regs.state.lock();
            state.config_generation = state.config_generation.wrapping_add(1);
            state.interrupt_status |= INT_CONFIG_CHANGE;
        }
        self.regs.drive(true);
    }
}

impl Registers {
    /// Drive the interrupt line. Never called with the state lock held.
    fn drive(&self, asserted: bool) {
        let out = self.links.lock().out.clone();
        if let Some(out) = out {
            out.set(Level::from_bool(asserted));
        }
    }

    /// The feature word the driver is currently selecting.
    fn device_features(&self, sel: u32) -> u32 {
        let all = self.backend.features() | F_VERSION_1;
        match sel {
            0 => all as u32,
            1 => (all >> 32) as u32,
            // §6 reserves everything above bit 63; a selector past it reads
            // zero rather than wrapping around to word 0.
            _ => 0,
        }
    }

    /// Run every chain the driver has made available on `index`, and on
    /// whatever else is asked for while that is happening.
    ///
    /// # Why this is a loop and not a call
    ///
    /// The register block is in the address space this device masters, so a
    /// driver can point a descriptor at this device's own `QueueNotify` — and
    /// then the backend's write into that buffer re-enters this function from
    /// inside itself. It is not exotic: a `VIRTIO_BLK_T_IN` of a blank sector
    /// into a four-byte writable descriptor aimed at `0x…050` writes four
    /// zero bytes, and zero is a legal queue index. The device's position in
    /// the available ring is not committed until the pass ends, so the nested
    /// call would re-run the same chain, and the recursion has no bottom.
    ///
    /// The answer is the one `core::wire` already gives for a re-entrant level
    /// change and `dev/nvme` gives for a re-entrant doorbell: the work is
    /// **iterative, not recursive**. A notify records its queue and returns if
    /// a pass is already running; the running pass re-reads what has been
    /// recorded after every queue it drains. Depth is one whatever the guest
    /// builds.
    ///
    /// It is also what makes two harts notifying at once correct rather than
    /// merely lucky: the second records its queue and returns, and the first
    /// serves it. A driver already has to wait for the used ring rather than
    /// for the store to retire (§2.7.8), so that is not a promise being broken.
    ///
    /// No lock is held across any of it. The state lock is taken in short
    /// sections and released before the backend runs, because the backend does
    /// DMA through the same space this notify arrived on (`ROADMAP.md` §4.7).
    fn notify(&self, index: u32) {
        if index as usize >= self.backend.queue_count().min(MAX_QUEUES) {
            // No such queue: nothing to record, and `live_queue` would refuse
            // it anyway.
            return;
        }
        self.pending
            .fetch_or(1u64 << index, atomic::Ordering::Relaxed);
        if self.engine.swap(true, atomic::Ordering::Acquire) {
            return;
        }

        let mut worked = false;
        loop {
            let mask = self.pending.swap(0, atomic::Ordering::Relaxed);
            if mask != 0 {
                for queue in 0..MAX_QUEUES as u32 {
                    if mask & (1u64 << queue) != 0 {
                        worked |= self.run_queue(queue);
                    }
                }
                continue;
            }
            // Nothing left. Stand down — and then look once more, because a
            // notify that arrived between the swap above and this store saw
            // the engine running and left its bit for somebody.
            self.engine.store(false, atomic::Ordering::Release);
            if self.pending.load(atomic::Ordering::Relaxed) == 0 {
                break;
            }
            if self.engine.swap(true, atomic::Ordering::Acquire) {
                // Somebody else took the pass over. It will see the bit.
                break;
            }
        }

        if worked {
            let raise = self.state.lock().interrupt_status != 0;
            self.drive(raise);
        }
    }

    /// Drain queue `index`, returning whether anything was completed.
    fn run_queue(&self, index: u32) -> bool {
        let (space, requester) = {
            let links = self.links.lock();
            (links.space.clone(), links.requester)
        };
        let Some(space) = space else {
            return false;
        };
        let Some(queue) = self.live_queue(index) else {
            return false;
        };
        let q = Queue::new(queue.layout, &space, requester);
        let Ok(avail) = q.avail_idx() else {
            return false;
        };

        let mut last = queue.last_avail;
        let mut used = queue.used_idx;
        let mut did_work = false;
        // `avail` wraps at 16 bits, so the comparison is a difference and never
        // an ordering — the driver's index passing ours by 32768 is not a
        // reason to stop (§2.7.6).
        while last != avail {
            let Ok(head) = q.avail_head(last) else {
                break;
            };
            let Ok(chain) = q.chain(head) else {
                break;
            };
            let written = self.backend.handle(index as usize, &q, &chain);
            let Ok(next) = q.publish(used, head, written) else {
                break;
            };
            used = next;
            last = last.wrapping_add(1);
            did_work = true;
        }

        let mut state = self.state.lock();
        let Some(slot) = state.queues.get_mut(index as usize) else {
            return false;
        };
        // Only if this is still the queue that was drained. A `Status` write
        // of zero reached from inside `handle` — the register block is in the
        // space this device masters — resets every queue, and writing a
        // position back into a queue that no longer exists would resurrect it.
        if slot.layout == queue.layout {
            slot.last_avail = last;
            slot.used_idx = used;
        }
        if did_work {
            state.interrupt_status |= INT_USED_BUFFER;
        }
        did_work
    }

    /// The queue `index`, if the driver has finished setting it up and the
    /// device is running.
    fn live_queue(&self, index: u32) -> Option<QueueState> {
        let state = self.state.lock();
        if state.status & STATUS_DRIVER_OK == 0 {
            return None;
        }
        let slot = state.queues.get(index as usize).copied()?;
        slot.layout.is_live().then_some(slot)
    }

    /// Return every register to its power-on value and tell the backend.
    fn reset(&self) {
        {
            let mut state = self.state.lock();
            *state = State::new(state.queues.len());
        }
        // Outstanding notifications go with the queues they named. `engine` is
        // deliberately left alone: it belongs to whichever call frame is
        // running the pass, and a reset reached from inside one (a `Status`
        // write of zero through a descriptor) must not hand a second frame the
        // right to run.
        self.pending.store(0, atomic::Ordering::Relaxed);
        self.backend.reset();
        self.drive(false);
    }

    fn read_register(&self, offset: u64) -> u32 {
        let mut state = self.state.lock();
        match offset {
            0x000 => MAGIC,
            0x004 => VERSION,
            0x008 => self.backend.device_id(),
            0x00c => VENDOR_ID,
            0x010 => self.device_features(state.device_features_sel),
            0x034 => QUEUE_SIZE_MAX,
            0x044 => u32::from(
                state
                    .queues
                    .get(state.queue_sel as usize)
                    .is_some_and(|q| q.layout.ready),
            ),
            0x060 => state.interrupt_status,
            0x070 => state.status,
            0x0fc => state.config_generation,
            // Write-only registers, and every reserved word: read as zero
            // (§4.2.2). Better than a fault — a driver that reads back what it
            // wrote to `QueueNum` gets a wrong answer rather than a crash, and
            // the specification says it may not do that anyway.
            _ => {
                let _ = &mut state;
                0
            }
        }
    }

    fn write_register(&self, offset: u64, value: u32) {
        // The two writes that act outward — a notify and a reset — are done
        // after the state lock is released.
        enum After {
            Nothing,
            Notify(u32),
            Reset,
            Interrupt(bool),
        }
        let after = {
            let mut state = self.state.lock();
            let sel = state.queue_sel as usize;
            match offset {
                0x014 => {
                    state.device_features_sel = value;
                    After::Nothing
                }
                0x020 => {
                    let word = state.driver_features_sel.min(1) as usize;
                    if state.driver_features_sel < 2 {
                        state.driver_features[word] = value;
                    }
                    After::Nothing
                }
                0x024 => {
                    state.driver_features_sel = value;
                    After::Nothing
                }
                0x030 => {
                    state.queue_sel = value;
                    After::Nothing
                }
                0x038 => {
                    if let Some(q) = state.queues.get_mut(sel) {
                        // A driver may not ask for more than QueueNumMax, and
                        // the size must be a power of two (§2.7).
                        let size = value.min(QUEUE_SIZE_MAX);
                        q.layout.size = if size.is_power_of_two() { size } else { 0 };
                    }
                    After::Nothing
                }
                0x044 => {
                    if let Some(q) = state.queues.get_mut(sel) {
                        q.layout.ready = value & 1 != 0;
                        if !q.layout.ready {
                            // §4.2.2: writing zero after a reset of the queue
                            // means the driver is done with it.
                            q.last_avail = 0;
                            q.used_idx = 0;
                        }
                    }
                    After::Nothing
                }
                0x050 => After::Notify(value),
                0x064 => {
                    state.interrupt_status &= !value;
                    After::Interrupt(state.interrupt_status != 0)
                }
                0x070 => {
                    if value == 0 {
                        After::Reset
                    } else {
                        let mut status = value;
                        // §2.1: the device refuses FEATURES_OK if the driver
                        // did not accept a feature set it can work with. A
                        // modern device requires VIRTIO_F_VERSION_1.
                        if status & STATUS_FEATURES_OK != 0 {
                            let accepted = u64::from(state.driver_features[0])
                                | (u64::from(state.driver_features[1]) << 32);
                            if accepted & F_VERSION_1 == 0 {
                                status &= !STATUS_FEATURES_OK;
                            }
                        }
                        state.status = status
                            & (STATUS_ACKNOWLEDGE
                                | STATUS_DRIVER
                                | STATUS_DRIVER_OK
                                | STATUS_FEATURES_OK
                                | STATUS_NEEDS_RESET
                                | STATUS_FAILED);
                        After::Nothing
                    }
                }
                0x080 => {
                    set_low(&mut state, sel, |l| &mut l.desc, value);
                    After::Nothing
                }
                0x084 => {
                    set_high(&mut state, sel, |l| &mut l.desc, value);
                    After::Nothing
                }
                0x090 => {
                    set_low(&mut state, sel, |l| &mut l.avail, value);
                    After::Nothing
                }
                0x094 => {
                    set_high(&mut state, sel, |l| &mut l.avail, value);
                    After::Nothing
                }
                0x0a0 => {
                    set_low(&mut state, sel, |l| &mut l.used, value);
                    After::Nothing
                }
                0x0a4 => {
                    set_high(&mut state, sel, |l| &mut l.used, value);
                    After::Nothing
                }
                _ => After::Nothing,
            }
        };
        match after {
            After::Nothing => {}
            After::Notify(index) => self.notify(index),
            After::Reset => self.reset(),
            After::Interrupt(level) => self.drive(level),
        }
    }
}

/// Set the low half of one of a queue's ring addresses.
fn set_low(state: &mut State, sel: usize, field: fn(&mut Layout) -> &mut u64, value: u32) {
    if let Some(q) = state.queues.get_mut(sel) {
        let slot = field(&mut q.layout);
        *slot = (*slot & 0xffff_ffff_0000_0000) | u64::from(value);
    }
}

/// Set the high half of one of a queue's ring addresses.
fn set_high(state: &mut State, sel: usize, field: fn(&mut Layout) -> &mut u64, value: u32) {
    if let Some(q) = state.queues.get_mut(sel) {
        let slot = field(&mut q.layout);
        *slot = (*slot & 0xffff_ffff) | (u64::from(value) << 32);
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if offset >= CONFIG_OFFSET {
            // The configuration space is the device's own, and is accessed at
            // whatever width its layout uses (§4.2.2).
            self.backend.config_read(offset - CONFIG_OFFSET, dst);
            return Ok(());
        }
        if dst.len() != 4 || !offset.is_multiple_of(4) {
            // Every transport register is a naturally aligned 32-bit word.
            return Err(BusError::BadAccess);
        }
        dst.copy_from_slice(&self.read_register(offset).to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            // Writing `QueueNotify` performs I/O and writing `Status` resets
            // the device; neither can be made side-effect free.
            return Err(BusError::BadAccess);
        }
        if offset >= CONFIG_OFFSET {
            self.backend.config_write(offset - CONFIG_OFFSET, src);
            return Ok(());
        }
        if src.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        self.write_register(offset, u32::from_le_bytes([src[0], src[1], src[2], src[3]]));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Not `word(U32)`: the configuration space above `0x100` is read a
        // byte at a time by a driver that does not know its layout, so the
        // width check belongs in the handler where the offset is known.
        AccessConstraints {
            min: Width::U8,
            max: Width::U64,
            natural_alignment: true,
            endian: Endian::Little,
            allow_bulk: false,
            secure_only: false,
            privileged_only: false,
            drives_data_bus: true,
        }
    }
}

impl DtSource for Registers {
    fn dt_spec(&self) -> NodeSpec {
        let mut spec = NodeSpec::peripheral("virtio_mmio", &["virtio,mmio"]);
        spec.irq_wire = self.links.lock().irq_wire;
        spec
    }
}

impl Device for VirtioMmio {
    fn class(&self) -> &'static DeviceClass {
        self.class
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // What this region is, for the board's device-tree generator.
        super::super::dt::publish(
            ctx.hosts(),
            &self.region,
            Arc::downgrade(&self.regs) as Weak<dyn DtSource>,
        )
    }

    fn reset(&self, _kind: ResetKind) {
        self.regs.reset();
    }

    fn flush(&self) -> Result<()> {
        // The transport holds nothing a host can see; whatever is behind it
        // might.
        self.regs.backend.flush()
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "irq" {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from("a virtio-mmio device drives one pin, `irq`"),
            });
        }
        let mut links = self.regs.links.lock();
        // Recorded so the device tree can ask the PLIC which source this net
        // lands on, rather than having the number written down twice.
        links.irq_wire = Some(source.id());
        links.out = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        if port == "irq" {
            let asserted = self.regs.state.lock().interrupt_status != 0;
            self.regs.drive(asserted);
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        w.write_u32(state.device_features_sel)?;
        w.write_u32(state.driver_features_sel)?;
        w.write_u32(state.driver_features[0])?;
        w.write_u32(state.driver_features[1])?;
        w.write_u32(state.queue_sel)?;
        w.write_u32(state.status)?;
        w.write_u32(state.interrupt_status)?;
        w.write_u32(state.config_generation)?;
        w.write_seq_len(state.queues.len() as u64)?;
        for q in &state.queues {
            w.write_u32(q.layout.size)?;
            w.write_u64(q.layout.desc)?;
            w.write_u64(q.layout.avail)?;
            w.write_u64(q.layout.used)?;
            w.write_bool(q.layout.ready)?;
            w.write_u16(q.last_avail)?;
            w.write_u16(q.used_idx)?;
        }
        drop(state);
        self.regs.backend.save(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let queues = self.regs.state.lock().queues.len();
        let mut state = State::new(queues);
        state.device_features_sel = r.read_u32()?;
        state.driver_features_sel = r.read_u32()?;
        state.driver_features[0] = r.read_u32()?;
        state.driver_features[1] = r.read_u32()?;
        state.queue_sel = r.read_u32()?;
        state.status = r.read_u32()?;
        state.interrupt_status = r.read_u32()?;
        state.config_generation = r.read_u32()?;
        let count = r.read_seq_len(29)? as usize;
        if count != queues {
            return Err(Error::State(format!(
                "snapshot has {count} virtqueue(s), this device has {queues}"
            )));
        }
        for q in &mut state.queues {
            q.layout.size = r.read_u32()?;
            q.layout.desc = r.read_u64()?;
            q.layout.avail = r.read_u64()?;
            q.layout.used = r.read_u64()?;
            q.layout.ready = r.read_bool()?;
            q.last_avail = r.read_u16()?;
            q.used_idx = r.read_u16()?;
        }
        let asserted = state.interrupt_status != 0;
        *self.regs.state.lock() = state;
        self.regs.backend.load(r)?;
        self.regs.drive(asserted);
        Ok(())
    }
}

impl Instance for VirtioMmio {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: ctx.path().to_string(),
            message: String::from(
                "a virtio device is a bus master and needs the address space its \
                 descriptors live in (`space = mem`)",
            ),
        })?;
        self.attach_space(Arc::clone(space), ctx.requester());
        Ok(())
    }
}

/// A chain, split into what the device may read and what it may write.
///
/// A convenience for backends, which all want the same two numbers.
#[must_use]
pub fn chain_lengths(chain: &[Descriptor]) -> (u64, u64) {
    (Queue::readable_len(chain), Queue::writable_len(chain))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{RamStore, Region as CoreRegion};
    use crate::core::value::Width as W;

    /// A transport over a backend that records what it was asked to do.
    #[derive(Debug, Default)]
    struct Echo {
        calls: Mutex<u32>,
    }

    impl Backend for Echo {
        fn device_id(&self) -> u32 {
            0xbeef
        }

        fn queue_count(&self) -> usize {
            1
        }

        fn config_read(&self, _offset: u64, dst: &mut [u8]) {
            dst.fill(0xa5);
        }

        fn handle(&self, _queue: usize, q: &Queue<'_>, chain: &[Descriptor]) -> u32 {
            *self.calls.lock() += 1;
            q.write_chain(chain, 0, b"ok").unwrap_or(0) as u32
        }

        fn reset(&self) {
            *self.calls.lock() = 0;
        }
    }

    struct Fixture {
        device: VirtioMmio,
        space: Arc<AddressSpace>,
        echo: Arc<Echo>,
    }

    static ECHO_CLASS: DeviceClass = DeviceClass {
        name: "virtio.test",
        version: 1,
        summary: "a virtio device for the transport's own tests",
        properties: &[],
        construct: |_| Err(Error::Unimplemented("test only")),
    };

    const DESC: u64 = 0x1000;
    const AVAIL: u64 = 0x2000;
    const USED: u64 = 0x3000;
    const BUF: u64 = 0x4000;

    impl Fixture {
        fn new() -> Fixture {
            let echo = Arc::new(Echo::default());
            let device = VirtioMmio::new(Arc::clone(&echo) as Arc<dyn Backend>, &ECHO_CLASS);
            let space = AddressSpace::new("mem", 64);
            space
                .topology()
                .map(CoreRegion::ram("ram", Arc::new(RamStore::new(0x1_0000))), 0)
                .unwrap();
            let space = Arc::new(space);
            device.attach_space(Arc::clone(&space), RequesterId(2));
            Fixture {
                device,
                space,
                echo,
            }
        }

        fn read(&self, offset: u64) -> u32 {
            let mut bytes = [0u8; 4];
            self.device
                .regs
                .read(offset, &mut bytes, MemAttrs::DEFAULT)
                .expect("a word read is legal");
            u32::from_le_bytes(bytes)
        }

        fn write(&self, offset: u64, value: u32) {
            self.device
                .regs
                .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
                .expect("a word write is legal");
        }

        fn poke(&self, at: u64, width: W, value: u64) {
            self.space
                .write(at, width, value, MemAttrs::DEFAULT)
                .unwrap();
        }

        /// Take the device through the whole §2.1 handshake and set up queue 0.
        fn bring_up(&self) {
            self.write(0x070, STATUS_ACKNOWLEDGE);
            self.write(0x070, STATUS_ACKNOWLEDGE | STATUS_DRIVER);
            self.write(0x024, 1);
            self.write(0x020, 1); // VIRTIO_F_VERSION_1 is bit 32
            self.write(
                0x070,
                STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK,
            );
            self.write(0x030, 0);
            self.write(0x038, 8);
            self.write(0x080, DESC as u32);
            self.write(0x084, 0);
            self.write(0x090, AVAIL as u32);
            self.write(0x094, 0);
            self.write(0x0a0, USED as u32);
            self.write(0x0a4, 0);
            self.write(0x044, 1);
            self.write(
                0x070,
                STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK | STATUS_DRIVER_OK,
            );
        }

        /// Offer one writable descriptor at `BUF`.
        fn offer(&self, idx: u16) {
            self.poke(DESC, W::U64, BUF);
            self.poke(DESC + 8, W::U32, 8);
            self.poke(
                DESC + 12,
                W::U16,
                u64::from(super::super::queue::DESC_F_WRITE),
            );
            self.poke(DESC + 14, W::U16, 0);
            self.poke(AVAIL + 4, W::U16, 0);
            self.poke(AVAIL + 2, W::U16, u64::from(idx));
        }
    }

    #[test]
    fn the_identity_registers_are_what_a_driver_probes_for() {
        let f = Fixture::new();
        assert_eq!(f.read(0x000), MAGIC);
        assert_eq!(f.read(0x004), VERSION, "modern, not legacy");
        assert_eq!(f.read(0x008), 0xbeef);
        assert_eq!(f.read(0x00c), VENDOR_ID);
        assert_eq!(f.read(0x034), QUEUE_SIZE_MAX);
    }

    #[test]
    fn the_feature_words_are_selected_one_at_a_time() {
        let f = Fixture::new();
        f.write(0x014, 0);
        assert_eq!(f.read(0x010), 0, "nothing in the low word");
        f.write(0x014, 1);
        assert_eq!(f.read(0x010), 1, "VIRTIO_F_VERSION_1 is bit 32");
        f.write(0x014, 2);
        assert_eq!(f.read(0x010), 0, "and nothing above 63");
    }

    #[test]
    fn features_ok_is_refused_unless_the_driver_accepted_version_1() {
        // §2.1: the device gets to say no, and a driver that ignores this is
        // one that would otherwise be handed a legacy layout it did not ask
        // for.
        let f = Fixture::new();
        f.write(0x070, STATUS_ACKNOWLEDGE | STATUS_DRIVER);
        f.write(
            0x070,
            STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK,
        );
        assert_eq!(f.read(0x070) & STATUS_FEATURES_OK, 0, "refused");

        f.write(0x024, 1);
        f.write(0x020, 1);
        f.write(
            0x070,
            STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK,
        );
        assert_eq!(f.read(0x070) & STATUS_FEATURES_OK, STATUS_FEATURES_OK);
    }

    #[test]
    fn a_notify_before_driver_ok_does_nothing() {
        let f = Fixture::new();
        f.offer(1);
        f.write(0x050, 0);
        assert_eq!(*f.echo.calls.lock(), 0);
    }

    #[test]
    fn a_notify_runs_every_available_chain_and_raises_the_interrupt() {
        let f = Fixture::new();
        f.bring_up();
        f.offer(1);
        f.write(0x050, 0);
        assert_eq!(*f.echo.calls.lock(), 1);
        assert_eq!(f.read(0x060), INT_USED_BUFFER);
        assert!(f.device.irq_asserted());

        // The used ring got the head and the length.
        let head = f.space.read(USED + 4, W::U32, MemAttrs::DEBUG).unwrap();
        let len = f.space.read(USED + 8, W::U32, MemAttrs::DEBUG).unwrap();
        assert_eq!((head, len), (0, 2));
        assert_eq!(f.space.read(USED + 2, W::U16, MemAttrs::DEBUG).unwrap(), 1);

        // Acknowledging drops the line.
        f.write(0x064, INT_USED_BUFFER);
        assert_eq!(f.read(0x060), 0);
        assert!(!f.device.irq_asserted());
    }

    #[test]
    fn a_second_notify_with_nothing_new_does_no_work() {
        let f = Fixture::new();
        f.bring_up();
        f.offer(1);
        f.write(0x050, 0);
        f.write(0x064, INT_USED_BUFFER);
        f.write(0x050, 0);
        assert_eq!(*f.echo.calls.lock(), 1, "the ring has not moved");
        assert!(!f.device.irq_asserted());
    }

    #[test]
    fn a_queue_size_that_is_not_a_power_of_two_is_refused() {
        let f = Fixture::new();
        f.write(0x030, 0);
        f.write(0x038, 7);
        f.write(0x044, 1);
        assert_eq!(f.read(0x044), 1, "ready is what the driver wrote");
        // But the queue is not live, so a notify does nothing.
        f.write(0x070, STATUS_DRIVER_OK);
        f.write(0x050, 0);
        assert_eq!(*f.echo.calls.lock(), 0);
    }

    #[test]
    fn writing_zero_to_status_resets_everything() {
        let f = Fixture::new();
        f.bring_up();
        f.offer(1);
        f.write(0x050, 0);
        assert!(f.device.irq_asserted());
        f.write(0x070, 0);
        assert_eq!(f.read(0x070), 0);
        assert_eq!(f.read(0x060), 0);
        assert!(!f.device.irq_asserted());
        assert_eq!(*f.echo.calls.lock(), 0, "and the backend was told");
    }

    #[test]
    fn the_configuration_space_is_the_backends_and_is_byte_addressable() {
        let f = Fixture::new();
        let mut byte = [0u8; 1];
        f.device
            .regs
            .read(CONFIG_OFFSET + 3, &mut byte, MemAttrs::DEFAULT)
            .unwrap();
        assert_eq!(byte[0], 0xa5);
    }

    #[test]
    fn a_register_access_that_is_not_an_aligned_word_is_refused() {
        let f = Fixture::new();
        assert!(
            f.device
                .regs
                .read(0x002, &mut [0u8; 4], MemAttrs::DEFAULT)
                .is_err()
        );
        assert!(
            f.device
                .regs
                .read(0x000, &mut [0u8; 2], MemAttrs::DEFAULT)
                .is_err()
        );
        assert!(
            f.device
                .regs
                .write(0x070, &[0u8; 4], MemAttrs::DEBUG)
                .is_err(),
            "and a debug write is refused outright"
        );
    }

    /// A backend that writes four zero bytes into the chain — which is a
    /// `QueueNotify` of queue 0 if the driver aimed the descriptor at this
    /// device's own register block.
    #[derive(Debug, Default)]
    struct Zeros {
        calls: Mutex<u32>,
    }

    impl Backend for Zeros {
        fn device_id(&self) -> u32 {
            0xfeed
        }

        fn queue_count(&self) -> usize {
            1
        }

        fn config_read(&self, _offset: u64, dst: &mut [u8]) {
            dst.fill(0);
        }

        fn handle(&self, _queue: usize, q: &Queue<'_>, chain: &[Descriptor]) -> u32 {
            *self.calls.lock() += 1;
            q.write_chain(chain, 0, &[0u8; 4]).unwrap_or(0) as u32
        }

        fn reset(&self) {}
    }

    #[test]
    fn a_descriptor_aimed_at_this_devices_own_doorbell_does_not_recurse() {
        // The register block is in the address space this device masters, so a
        // driver can point a writable descriptor at `QueueNotify` and be
        // re-entered from inside its own transfer. Four zero bytes of disk data
        // are a notify of queue 0, which is not an exotic value: it is what a
        // blank sector holds. The device's position in the available ring is
        // not committed until the pass ends, so a recursive engine would re-run
        // the same chain for ever.
        let zeros = Arc::new(Zeros::default());
        let device = VirtioMmio::new(Arc::clone(&zeros) as Arc<dyn Backend>, &ECHO_CLASS);
        let space = AddressSpace::new("mem", 64);
        space
            .topology()
            .map(CoreRegion::ram("ram", Arc::new(RamStore::new(0x1_0000))), 0)
            .unwrap();
        // The device, in the space it masters.
        const SELF: u64 = 0x2_0000;
        space
            .topology()
            .map(device.region("").expect("a register window"), SELF)
            .unwrap();
        let space = Arc::new(space);
        device.attach_space(Arc::clone(&space), RequesterId(3));

        let poke = |at: u64, width: W, value: u64| {
            space.write(at, width, value, MemAttrs::DEFAULT).unwrap();
        };
        let set = |off: u64, value: u32| {
            poke(SELF + off, W::U32, u64::from(value));
        };

        set(0x070, STATUS_ACKNOWLEDGE | STATUS_DRIVER);
        set(0x024, 1);
        set(0x020, 1);
        set(
            0x070,
            STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK,
        );
        set(0x030, 0);
        set(0x038, 8);
        set(0x080, DESC as u32);
        set(0x090, AVAIL as u32);
        set(0x0a0, USED as u32);
        set(0x044, 1);
        set(
            0x070,
            STATUS_ACKNOWLEDGE | STATUS_DRIVER | STATUS_FEATURES_OK | STATUS_DRIVER_OK,
        );

        // One writable descriptor, four bytes, aimed at `QueueNotify`.
        poke(DESC, W::U64, SELF + 0x050);
        poke(DESC + 8, W::U32, 4);
        poke(
            DESC + 12,
            W::U16,
            u64::from(super::super::queue::DESC_F_WRITE),
        );
        poke(DESC + 14, W::U16, 0);
        poke(AVAIL + 4, W::U16, 0);
        poke(AVAIL + 2, W::U16, 1);

        set(0x050, 0);
        assert_eq!(
            *zeros.calls.lock(),
            1,
            "the chain ran once; the re-entrant notify found the engine busy"
        );
        assert_eq!(
            space.read(USED + 2, W::U16, MemAttrs::DEBUG).unwrap(),
            1,
            "and it was completed exactly once"
        );
    }

    #[test]
    fn a_snapshot_round_trips_the_transport() {
        use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

        let saved = Fixture::new();
        saved.bring_up();
        saved.offer(1);
        saved.write(0x050, 0);

        let mut shape = MachineShape::new();
        shape.add_device("vio", ECHO_CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("vio", ECHO_CLASS.name, ECHO_CLASS.version).unwrap();
            saved.device.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = Fixture::new();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load(
                "vio",
                ECHO_CLASS.name,
                ECHO_CLASS.version,
                &Migrations::new(),
            )
            .unwrap();
        restored.device.load(&mut chunk.reader()).unwrap();

        assert_eq!(restored.read(0x070), saved.read(0x070));
        assert_eq!(restored.read(0x060), INT_USED_BUFFER);
        assert!(restored.device.irq_asserted());
        // And the device's position in the ring came back: re-notifying with
        // the same available index must not run the chain twice.
        restored.offer(1);
        restored.write(0x050, 0);
        assert_eq!(*restored.echo.calls.lock(), 0);
    }
}
