//! CFI NOR flash: the query structure, and the Intel/Sharp extended command
//! set.
//!
//! A NOR flash part is not memory with a different name. Three properties make
//! it a *device*, and a model that drops any one of them is a model that lets
//! firmware succeed at something the silicon would have refused:
//!
//! 1. **A program can only clear bits.** Writing `0xf0` over `0x0f` leaves
//!    `0x00`, not `0xf0`. Every fault-tolerant-write scheme ever built — UEFI's
//!    included — depends on this, because it is what lets a record be marked
//!    "in progress" and later "valid" without erasing anything in between.
//! 2. **Setting bits takes an erase, and an erase takes a whole block.** There
//!    is no way to put a single bit back to one. Blocks need not be the same
//!    size: boot-block parts subdivide one end of the array, which is why
//!    [`Geometry`] is a list of regions rather than a divisor.
//! 3. **The array is not always what a read returns.** After a command the part
//!    answers with its status register, its identifier, or its CFI query table
//!    until it is told to go back to reading the array. Firmware polls status
//!    through exactly this window.
//!
//! # Which command set, and why
//!
//! CFI is only the *query*: it tells a driver which command set the part speaks
//! and what its geometry is. The two that matter are the AMD/Fujitsu set
//! (`0x0002`, unlock cycles at `0x555`/`0x2aa` and toggle-bit polling) and the
//! Intel/Sharp extended set (`0x0001`, a status register and single-cycle
//! setup/confirm pairs).
//!
//! This device implements **Intel/Sharp extended, `0x0001`**, in the shape the
//! Intel StrataFlash P30 family defines. The reason is the software: the UEFI
//! firmware this device exists to serve — EDK II's `VirtNorFlashDxe`, which is
//! BSD-2-Clause-Patent and so may be read — issues `0x70`/`0x50` status
//! commands, `0x40` word programs, `0xe8`/`0xd0` buffered programs,
//! `0x20`/`0xd0` block erases and `0x60`/`0xd0` block unlocks. Those are the
//! Intel set. Implementing the AMD set instead would be a device nothing in
//! this tree can talk to.
//!
//! # Two chips make a bus
//!
//! Boards do not usually wire one part to a 32-bit bus; they wire *two x16
//! parts in parallel*, one on each halfword. That is not a detail we could
//! hide: the driver above writes each command duplicated into both halves of a
//! 32-bit word and folds the two status registers back together, and the CFI
//! query describes **one device**, not the pair. So [`Geometry`] carries a bus
//! width and an interleave, every access is split into per-device lanes, and
//! each device has its own command state machine, status register and lock
//! bits. A `width = 2, interleave = 1` machine gets a single x16 part and the
//! same code path.
//!
//! # Time
//!
//! **Deliberately zero.** A program or an erase completes inside the bus cycle
//! that confirms it, so the status register already reads ready by the time the
//! guest polls it. This is a choice, not an omission:
//!
//! * Nothing a guest can do observes the difference except a timing loop, and
//!   the firmware that matters polls the status register — which is what a real
//!   part is *for*, and which terminates immediately here rather than after the
//!   spec's 1024 ms.
//! * The alternative is a scheduler event, and an event needs a clock domain.
//!   A NOR part has no clock input: it is asynchronous, timed by an internal
//!   oscillator that no board wires and no device tree describes. Giving this
//!   device a domain would mean inventing a crystal for it.
//!
//! The CFI query still reports the real timeouts (§4's fields at `0x1f`-`0x26`)
//! because those describe the part a driver thinks it is talking to, and a
//! driver that sizes its own timeout from them must get a plausible number.
//! What is *not* modelled is the busy window itself; a guest that needs one
//! should say so and this device will grow a domain.
//!
//! # Sources
//!
//! * **JEDEC JESD68.01**, *Common Flash Interface (CFI)*, and JEP137B, for the
//!   query structure: the `QRY` signature, the command-set and geometry fields
//!   at query offsets `0x10`-`0x2c`, and the erase-block-region descriptors
//!   that follow them.
//! * The **Intel/Sharp extended query table** (`PRI`) laid out by Intel's
//!   *Common Flash Interface and Command Sets* application note (AP-646), for
//!   the block-locking and suspend feature bits at `0x31` onwards.
//! * The **Intel StrataFlash Embedded Memory (P30) family** datasheet for the
//!   command encodings, the status-register bit assignments, the write-buffer
//!   sequence, and the fact that every block powers up locked.
//! * EDK II's `OvmfPkg/VirtNorFlashDxe` and `OvmfPkg/Library/VirtNorFlashDeviceLib`
//!   (BSD-2-Clause-Patent, read under `ROADMAP.md` §1's permissive-source rule)
//!   for which of those commands a real driver actually issues.
//!
//! No emulator source of any licence was consulted.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, Value, ValueKind};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, RamStore, Region, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::dev::medium::{self, Medium, Snapshot};
use crate::machine::realize::Instance;
use crate::machine::validate::{ClassSchema, PropSchema};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "flash.cfi";

/// The erase-block size a machine file gets when it does not say.
///
/// 256 KiB, which is the block size the UEFI variable store this device was
/// built for is laid out in.
pub const DEFAULT_BLOCK: u64 = 256 * 1024;

/// The bus width in bytes a machine file gets when it does not say.
pub const DEFAULT_WIDTH: u64 = 4;

/// How many devices sit in parallel on that bus when a machine file does not
/// say.
pub const DEFAULT_INTERLEAVE: u64 = 2;

/// Intel's JEDEC manufacturer identifier (JEP106, bank 1 code `0x89`).
pub const DEFAULT_MANUFACTURER: u16 = 0x0089;

// ---------------------------------------------------------------------------
// the command set (Intel/Sharp extended, 0x0001)
// ---------------------------------------------------------------------------

/// Read Array: the part goes back to answering with its contents.
const CMD_READ_ARRAY: u8 = 0xff;
/// Read Status Register.
const CMD_READ_STATUS: u8 = 0x70;
/// Clear Status Register: the sticky error bits go away.
const CMD_CLEAR_STATUS: u8 = 0x50;
/// Read Identifier: manufacturer, device, and the block lock configuration.
const CMD_READ_ID: u8 = 0x90;
/// Read CFI Query.
const CMD_READ_CFI: u8 = 0x98;
/// Word Program setup; the next write to the same address is the data.
const CMD_PROGRAM: u8 = 0x40;
/// The alternate encoding of the same setup cycle.
const CMD_PROGRAM_ALT: u8 = 0x10;
/// Block Erase setup; a confirm cycle must follow.
const CMD_ERASE: u8 = 0x20;
/// Write to Buffer setup.
const CMD_BUFFER: u8 = 0xe8;
/// Block Lock / configuration-register setup; a second cycle says which.
const CMD_LOCK_SETUP: u8 = 0x60;
/// The second cycle of a lock setup that *locks* the block.
const CMD_LOCK: u8 = 0x01;
/// The second cycle of a lock setup that locks the block *down*.
const CMD_LOCK_DOWN: u8 = 0x2f;
/// The second cycle of a read-configuration-register setup.
const CMD_READ_CONFIG: u8 = 0x03;
/// Program/Erase Suspend.
const CMD_SUSPEND: u8 = 0xb0;
/// Confirm — of an erase, of a buffered program, of an unlock, of a resume.
const CMD_CONFIRM: u8 = 0xd0;

/// SR.7, the write state machine's ready bit. Set means *not busy*.
const SR_READY: u16 = 0x80;
/// SR.5, block erase error.
const SR_ERASE_ERROR: u16 = 0x20;
/// SR.4, program error. Set together with [`SR_ERASE_ERROR`] it means the
/// command sequence itself was wrong.
const SR_PROGRAM_ERROR: u16 = 0x10;
/// SR.3, Vpp out of range. Never set here: this part has no separate Vpp.
const SR_VPP_ERROR: u16 = 0x08;
/// SR.1, the operation was refused because the block is locked.
const SR_LOCK_ERROR: u16 = 0x02;

/// The status register a part powers up with: ready, no errors.
const SR_RESET: u16 = SR_READY;

/// How many bytes one device will take in a single buffered program.
///
/// 64 per x16 device, which is 32 words — and, on the two-device bus this
/// board wires, the 128 bytes EDK II's driver writes at a time. Reported to
/// the guest in the CFI query at offset `0x2a` as its base-two logarithm.
const BUFFER_BYTES_PER_DEVICE: u64 = 64;

/// Query offset of the first erase-block region descriptor (JESD68.01 §4.3.3).
const QUERY_REGIONS: usize = 0x2d;

// ---------------------------------------------------------------------------
// geometry
// ---------------------------------------------------------------------------

/// A run of same-sized erase blocks.
///
/// Sizes are **bus** bytes: on a two-device bus a 256 KiB block is 128 KiB in
/// each device, and it is the bus figure that a `map` statement and a device
/// tree talk about.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockRegion {
    /// How many blocks in the run.
    pub count: u64,
    /// How many bus bytes each of them holds.
    pub size: u64,
}

/// How wide the bus is, how many parts share it, and how the array divides
/// into erase blocks.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Geometry {
    /// Bytes per bus access: 1, 2, 4 or 8.
    bus_width: u64,
    /// How many devices sit side by side on that bus.
    interleave: u64,
    /// The erase-block regions, in address order.
    regions: Vec<BlockRegion>,
    /// Their total size, cached.
    size: u64,
}

impl Geometry {
    /// A part whose blocks are all the same size.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the widths are not powers of two the other divides,
    /// if `block` does not divide `size`, or if a block is not a whole number
    /// of bus words in every device.
    pub fn uniform(size: u64, block: u64, bus_width: u64, interleave: u64) -> Result<Geometry> {
        // Widths first: "a three-byte bus" is a better complaint than "0x4000
        // does not divide by the block size you did not write down".
        check_widths(bus_width, interleave)?;
        if block == 0 || size == 0 || !size.is_multiple_of(block) {
            return Err(config(format!(
                "a flash of {size} byte(s) does not divide into blocks of {block}"
            )));
        }
        Geometry::new(
            alloc::vec![BlockRegion {
                count: size / block,
                size: block
            }],
            bus_width,
            interleave,
        )
    }

    /// A part with an explicit list of erase-block regions, in address order.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if there are no regions, if a width is implausible, or
    /// if a block is not a whole number of words in every device.
    pub fn new(regions: Vec<BlockRegion>, bus_width: u64, interleave: u64) -> Result<Geometry> {
        check_widths(bus_width, interleave)?;
        if regions.is_empty() {
            return Err(config(String::from(
                "a flash with no erase blocks cannot be erased, and so cannot be written",
            )));
        }
        let mut size = 0u64;
        for region in &regions {
            if region.count == 0 || region.size == 0 {
                return Err(config(String::from(
                    "an erase-block region holds at least one block of at least one byte",
                )));
            }
            if region.size % bus_width != 0 {
                return Err(config(format!(
                    "an erase block of {} byte(s) is not a whole number of {bus_width}-byte \
                     bus words",
                    region.size
                )));
            }
            // The CFI query reports a block size *per device*, in units of 256
            // bytes, so a block that is not a multiple of 256 device bytes
            // cannot be described to the guest at all.
            let per_device = region.size / interleave;
            if !per_device.is_multiple_of(256) {
                return Err(config(format!(
                    "an erase block of {} bus byte(s) is {per_device} byte(s) in each of \
                     {interleave} device(s), and the CFI query states a block size in units \
                     of 256",
                    region.size
                )));
            }
            size = size
                .checked_add(region.count.checked_mul(region.size).ok_or_else(|| {
                    config(String::from(
                        "an erase-block region larger than the address space",
                    ))
                })?)
                .ok_or_else(|| config(String::from("a flash larger than the address space")))?;
        }
        Ok(Geometry {
            bus_width,
            interleave,
            regions,
            size,
        })
    }

    /// Total size in bus bytes.
    #[must_use]
    pub fn size(&self) -> u64 {
        self.size
    }

    /// Bytes per bus access.
    #[must_use]
    pub fn bus_width(&self) -> u64 {
        self.bus_width
    }

    /// How many devices share the bus.
    #[must_use]
    pub fn interleave(&self) -> u64 {
        self.interleave
    }

    /// Bytes per access *per device*: 1 for an x8 part, 2 for an x16.
    #[must_use]
    pub fn device_width(&self) -> u64 {
        self.bus_width / self.interleave
    }

    /// The erase-block regions, in address order.
    #[must_use]
    pub fn regions(&self) -> &[BlockRegion] {
        &self.regions
    }

    /// How many erase blocks the part has in total.
    #[must_use]
    pub fn block_count(&self) -> u64 {
        self.regions.iter().map(|r| r.count).sum()
    }

    /// The index, base and size of the block `offset` lands in.
    #[must_use]
    pub fn block_at(&self, offset: u64) -> Option<(u64, u64, u64)> {
        let mut index = 0u64;
        let mut base = 0u64;
        for region in &self.regions {
            let span = region.count * region.size;
            if offset < base + span {
                let within = (offset - base) / region.size;
                return Some((index + within, base + within * region.size, region.size));
            }
            index += region.count;
            base += span;
        }
        None
    }
}

/// Check that `interleave` parts of a legal device width share `bus_width`.
fn check_widths(bus_width: u64, interleave: u64) -> Result<()> {
    if !matches!(bus_width, 1 | 2 | 4 | 8) {
        return Err(config(format!(
            "a bus is 1, 2, 4 or 8 bytes wide, not {bus_width}"
        )));
    }
    if interleave == 0 || !bus_width.is_multiple_of(interleave) {
        return Err(config(format!(
            "{interleave} device(s) do not share a {bus_width}-byte bus evenly"
        )));
    }
    let device_width = bus_width / interleave;
    if !matches!(device_width, 1 | 2) {
        return Err(config(format!(
            "a CFI part is x8 or x16, so {interleave} of them on a {bus_width}-byte bus would \
             each be {device_width} byte(s) wide"
        )));
    }
    Ok(())
}

fn config(message: String) -> Error {
    Error::Config {
        at: CLASS_NAME.to_string(),
        message,
    }
}

// ---------------------------------------------------------------------------
// one device's state machine
// ---------------------------------------------------------------------------

/// What a read of this device answers with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// The contents. What a part powers up in and returns to on `0xff`.
    Array,
    /// The status register, repeated at every address.
    Status,
    /// Manufacturer, device code, and the addressed block's lock bits.
    Id,
    /// The CFI query table.
    Cfi,
}

impl Mode {
    const fn tag(self) -> u8 {
        match self {
            Mode::Array => 0,
            Mode::Status => 1,
            Mode::Id => 2,
            Mode::Cfi => 3,
        }
    }

    fn from_tag(tag: u8) -> Result<Mode> {
        match tag {
            0 => Ok(Mode::Array),
            1 => Ok(Mode::Status),
            2 => Ok(Mode::Id),
            3 => Ok(Mode::Cfi),
            other => Err(Error::State(format!("{other} is not a flash read mode"))),
        }
    }
}

/// A command sequence that has begun and is waiting for its next cycle.
///
/// This is the state a snapshot has to carry that nothing else would think to:
/// a machine saved between `0x20` and its confirm is a machine with an erase
/// half-issued, and restoring it as idle would silently swallow the erase.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Pending {
    /// Nothing outstanding.
    None,
    /// `0x40` was written; the next write is the data.
    Program,
    /// `0x20` was written; the next write must be the confirm.
    Erase,
    /// `0x60` was written; the next write says lock, unlock, lock down, or
    /// read the configuration register.
    Lock,
    /// `0xe8` was written; the next write is the word count, less one.
    BufferCount,
    /// The count is known and this many words are still to arrive.
    BufferData {
        /// How many more words the buffer expects.
        left: u64,
    },
    /// The buffer is full; the next write must be the confirm.
    BufferConfirm,
}

impl Pending {
    const fn tag(&self) -> u8 {
        match self {
            Pending::None => 0,
            Pending::Program => 1,
            Pending::Erase => 2,
            Pending::Lock => 3,
            Pending::BufferCount => 4,
            Pending::BufferData { .. } => 5,
            Pending::BufferConfirm => 6,
        }
    }
}

/// One physical part on the bus.
#[derive(Debug, Clone)]
struct Chip {
    mode: Mode,
    pending: Pending,
    status: u16,
    /// Words latched by a buffered program and not yet committed, as
    /// (bus offset, value) pairs. A snapshot taken here restores a program
    /// that has been staged and not confirmed, which is exactly what the
    /// silicon would hold.
    buffer: Vec<(u64, u16)>,
    /// The volatile lock bit of each block.
    locked: Vec<bool>,
    /// The lock-down bit of each block: while set, the lock bit cannot be
    /// cleared.
    locked_down: Vec<bool>,
}

impl Chip {
    fn new(blocks: usize, locked: bool, down: bool) -> Chip {
        Chip {
            mode: Mode::Array,
            pending: Pending::None,
            status: SR_RESET,
            buffer: Vec::new(),
            locked: alloc::vec![locked; blocks],
            locked_down: alloc::vec![down; blocks],
        }
    }
}

// ---------------------------------------------------------------------------
// the array
// ---------------------------------------------------------------------------

/// The part (or parts) behind one address window: the contents, the per-device
/// command state machines, and the CFI query table they answer with.
///
/// This is what an [`AddressSpace`](crate::core::space::AddressSpace) dispatches
/// to; [`Cfi`] is the device that owns it.
pub struct Array {
    geom: Geometry,
    /// The contents. A [`RamStore`] because it is exactly the right thing: byte
    /// addressed, `Sync` without `unsafe`, and never handed out as a slice.
    array: Arc<RamStore>,
    chips: Mutex<Vec<Chip>>,
    /// True while every device is in [`Mode::Array`], so a read can skip the
    /// lock entirely. Firmware executes from flash, and a lock per instruction
    /// fetch is not a cost this device gets to impose.
    all_array: AtomicBool,
    /// Whether a program or an erase has changed the array since the contents
    /// last reached a [`Medium`](crate::dev::medium::Medium).
    ///
    /// A NOR part is storage, and storage that is never written back is a run
    /// whose guest kept nothing. This is what [`Cfi::flush`](Device::flush)
    /// tests before it moves megabytes: an image bank that nothing programmed
    /// costs a boolean rather than a copy.
    dirty: AtomicBool,
    /// The CFI query table of one device, indexed by query offset.
    query: Vec<u8>,
    manufacturer: u16,
    device_id: u16,
    /// Whether every block powers up locked, as an Intel P30 does.
    power_up_locked: bool,
    /// Whether the part is write protected — modelled as the lock-down bit
    /// held set, which is what a board that ties `WP#` low produces.
    read_only: bool,
}

impl fmt::Debug for Array {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Array")
            .field("geometry", &self.geom)
            .field("read_only", &self.read_only)
            .finish_non_exhaustive()
    }
}

impl Array {
    /// Build the parts described by `geom`, erased.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the array does not fit in this host's memory.
    pub fn new(geom: Geometry) -> Result<Array> {
        Array::with_options(geom, DEFAULT_MANUFACTURER, 0, true, false)
    }

    /// Build the parts, choosing the identifiers and the protection state.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the array does not fit in this host's memory.
    pub fn with_options(
        geom: Geometry,
        manufacturer: u16,
        device_id: u16,
        power_up_locked: bool,
        read_only: bool,
    ) -> Result<Array> {
        if usize::try_from(geom.size()).is_err() {
            return Err(config(format!(
                "a flash of {} byte(s) is larger than this host's address space",
                geom.size()
            )));
        }
        let array = Arc::new(RamStore::new(geom.size()));
        // Erased, not zeroed: an unwritten part reads all ones, and firmware
        // that finds zeroes concludes the whole array has been programmed.
        array.fill(0, geom.size(), 0xff).map_err(|_| {
            config(String::from(
                "the flash array could not be erased at construction",
            ))
        })?;
        let blocks = usize::try_from(geom.block_count())
            .map_err(|_| config(String::from("more erase blocks than this host can index")))?;
        let query = build_query(&geom);
        let chips = (0..geom.interleave())
            .map(|_| Chip::new(blocks, power_up_locked || read_only, read_only))
            .collect();
        Ok(Array {
            geom,
            array,
            chips: Mutex::with_rank(LockRank::DEVICE, chips),
            all_array: AtomicBool::new(true),
            dirty: AtomicBool::new(false),
            query,
            manufacturer,
            device_id,
            power_up_locked,
            read_only,
        })
    }

    /// The geometry these parts were built with.
    #[must_use]
    pub fn geometry(&self) -> &Geometry {
        &self.geom
    }

    /// The contents, for a test, a debugger, or a host that persists them.
    ///
    /// Never has a side effect and never touches a command state machine.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the range runs off the end of the part.
    pub fn read_contents(&self, offset: u64, dst: &mut [u8]) -> Result<()> {
        self.array
            .read_at(offset, dst)
            .map_err(|_| Error::State(format!("{offset:#x} is outside this flash")))
    }

    /// The whole contents as a fresh vector.
    ///
    /// This is how a run's flash is written back out so the next run starts
    /// from it — the thing that makes a UEFI variable survive a reboot.
    #[must_use]
    pub fn contents(&self) -> Vec<u8> {
        let mut out = alloc::vec![0u8; self.array.len() as usize];
        // Cannot fail: the length is the store's own.
        let _ = self.array.read_at(0, &mut out);
        out
    }

    /// Put `bytes` into the array at `offset`, ignoring flash semantics.
    ///
    /// This is the *loader's* door, not the guest's: it is how an initial image
    /// gets in, and it is deliberately not reachable from the bus, where a
    /// write can only ever clear bits.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if the image runs off the end of the part.
    pub fn load_image(&self, offset: u64, bytes: &[u8]) -> Result<()> {
        self.array.write_at(offset, bytes).map_err(|_| {
            config(format!(
                "an image of {} byte(s) at {offset:#x} does not fit in a flash of {}",
                bytes.len(),
                self.geom.size()
            ))
        })
    }

    /// Whether every device is answering with the array.
    #[must_use]
    pub fn is_reading_array(&self) -> bool {
        self.all_array.load(Ordering::Relaxed)
    }

    /// Whether a program or an erase has changed the array since the last
    /// [`mark_clean`](Array::mark_clean).
    ///
    /// [`load_image`](Array::load_image) deliberately does **not** set it: an
    /// image being put into a part is where the bytes came *from*, and writing
    /// them straight back out again would rewrite a file nothing had changed.
    #[must_use]
    pub fn is_dirty(&self) -> bool {
        self.dirty.load(Ordering::Relaxed)
    }

    /// Say that the contents have reached wherever they are kept.
    pub fn mark_clean(&self) {
        self.dirty.store(false, Ordering::Relaxed);
    }

    /// Put every device back the way it powers up.
    ///
    /// The **contents are not touched**: this is non-volatile memory, and a
    /// power cycle that restored the factory image would defeat the entire
    /// point of the device.
    pub fn reset(&self) {
        let mut chips = self.chips.lock();
        for chip in chips.iter_mut() {
            chip.mode = Mode::Array;
            chip.pending = Pending::None;
            chip.status = SR_RESET;
            chip.buffer.clear();
            let locked = self.power_up_locked || self.read_only;
            chip.locked.fill(locked);
            chip.locked_down.fill(self.read_only);
        }
        self.all_array.store(true, Ordering::Relaxed);
    }

    /// The status register of device `lane`, for tests.
    #[must_use]
    pub fn status(&self, lane: usize) -> Option<u16> {
        self.chips.lock().get(lane).map(|c| c.status)
    }

    /// Whether block `block` is locked in device `lane`, for tests.
    #[must_use]
    pub fn is_locked(&self, lane: usize, block: u64) -> Option<bool> {
        let chips = self.chips.lock();
        let chip = chips.get(lane)?;
        chip.locked.get(usize::try_from(block).ok()?).copied()
    }

    // -- the read path -----------------------------------------------------

    fn peek(&self, chips: &[Chip], offset: u64) -> MemResult<u8> {
        let dw = self.geom.device_width();
        let lane = ((offset / dw) % self.geom.interleave()) as usize;
        let chip = &chips[lane];
        let word = match chip.mode {
            Mode::Array => return self.array.read_u8(offset),
            Mode::Status => chip.status,
            Mode::Id => self.identifier(chip, offset),
            Mode::Cfi => self.query_word(offset),
        };
        Ok((word >> (8 * (offset % dw))) as u8)
    }

    /// What read-identifier mode answers at `offset`.
    ///
    /// The word index *within the addressed block* selects the field, which is
    /// what makes the block lock configuration readable per block: index 0 is
    /// the manufacturer, 1 the device, 2 the lock bits of the block the address
    /// falls in, and 5 the read configuration register.
    fn identifier(&self, chip: &Chip, offset: u64) -> u16 {
        let Some((block, base, _)) = self.geom.block_at(offset) else {
            return 0;
        };
        match (offset - base) / self.geom.bus_width() {
            0 => self.manufacturer,
            1 => self.device_id,
            2 => {
                let block = block as usize;
                u16::from(chip.locked.get(block).copied().unwrap_or(false))
                    | (u16::from(chip.locked_down.get(block).copied().unwrap_or(false)) << 1)
            }
            // The read configuration register: this part is asynchronous, so
            // every synchronous-read field reads zero.
            5 => 0,
            _ => 0,
        }
    }

    fn query_word(&self, offset: u64) -> u16 {
        let index = offset / self.geom.bus_width();
        usize::try_from(index)
            .ok()
            .and_then(|i| self.query.get(i))
            .map_or(0, |b| u16::from(*b))
    }

    // -- the write path ----------------------------------------------------

    fn command(&self, chip: &mut Chip, offset: u64, value: u16) {
        let cmd = (value & 0xff) as u8;
        match core::mem::replace(&mut chip.pending, Pending::None) {
            Pending::None => self.first_cycle(chip, cmd),
            Pending::Program => {
                self.program(chip, offset, value);
                chip.mode = Mode::Status;
            }
            Pending::Erase => {
                if cmd == CMD_CONFIRM {
                    self.erase(chip, offset);
                } else {
                    chip.status |= SR_ERASE_ERROR | SR_PROGRAM_ERROR;
                }
                chip.mode = Mode::Status;
            }
            Pending::Lock => self.lock_cycle(chip, offset, cmd),
            Pending::BufferCount => {
                // The count is written as "words less one", so 0 means one
                // word (P30 datasheet, Write to Buffer).
                let words = u64::from(value) + 1;
                if words > BUFFER_BYTES_PER_DEVICE / self.geom.device_width() {
                    chip.status |= SR_ERASE_ERROR | SR_PROGRAM_ERROR;
                    chip.mode = Mode::Status;
                } else {
                    chip.buffer.clear();
                    chip.pending = Pending::BufferData { left: words };
                }
            }
            Pending::BufferData { left } => {
                chip.buffer.push((offset, value));
                // Saturating rather than wrapping: `left` can also arrive
                // from a snapshot, and a corrupt one must not turn into four
                // billion outstanding words.
                chip.pending = match left.saturating_sub(1) {
                    0 => Pending::BufferConfirm,
                    left => Pending::BufferData { left },
                };
            }
            Pending::BufferConfirm => {
                if cmd == CMD_CONFIRM {
                    for (at, word) in core::mem::take(&mut chip.buffer) {
                        self.program(chip, at, word);
                    }
                } else {
                    chip.buffer.clear();
                    chip.status |= SR_ERASE_ERROR | SR_PROGRAM_ERROR;
                }
                chip.mode = Mode::Status;
            }
        }
    }

    fn first_cycle(&self, chip: &mut Chip, cmd: u8) {
        match cmd {
            CMD_READ_ARRAY => chip.mode = Mode::Array,
            CMD_READ_STATUS => chip.mode = Mode::Status,
            CMD_READ_ID => chip.mode = Mode::Id,
            CMD_READ_CFI => chip.mode = Mode::Cfi,
            // Clearing the status register says nothing about what the outputs
            // are muxed to, so the read mode is left exactly as it was.
            CMD_CLEAR_STATUS => {
                chip.status &= !(SR_ERASE_ERROR | SR_PROGRAM_ERROR | SR_VPP_ERROR | SR_LOCK_ERROR);
            }
            CMD_PROGRAM | CMD_PROGRAM_ALT => {
                chip.pending = Pending::Program;
                chip.mode = Mode::Status;
            }
            CMD_ERASE => {
                chip.pending = Pending::Erase;
                chip.mode = Mode::Status;
            }
            CMD_BUFFER => {
                chip.buffer.clear();
                chip.pending = Pending::BufferCount;
                chip.mode = Mode::Status;
            }
            CMD_LOCK_SETUP => {
                chip.pending = Pending::Lock;
                chip.mode = Mode::Status;
            }
            // Suspend and resume with nothing in progress: an operation here
            // finishes inside the cycle that starts it (see the module docs),
            // so there is never anything to suspend. The part still switches
            // its outputs to the status register, which is what the guest is
            // about to read.
            CMD_SUSPEND | CMD_CONFIRM => chip.mode = Mode::Status,
            // An unrecognised command is a command sequence error, and the
            // pair of error bits is how the Intel set says so.
            _ => {
                chip.status |= SR_ERASE_ERROR | SR_PROGRAM_ERROR;
                chip.mode = Mode::Status;
            }
        }
    }

    fn lock_cycle(&self, chip: &mut Chip, offset: u64, cmd: u8) {
        if cmd == CMD_READ_CONFIG {
            // `0x60` then `0x03` is Read Configuration Register, not a lock at
            // all; the register appears in the identifier space.
            chip.mode = Mode::Id;
            return;
        }
        chip.mode = Mode::Status;
        let Some((block, _, _)) = self.geom.block_at(offset) else {
            chip.status |= SR_ERASE_ERROR | SR_PROGRAM_ERROR;
            return;
        };
        let Ok(block) = usize::try_from(block) else {
            chip.status |= SR_ERASE_ERROR | SR_PROGRAM_ERROR;
            return;
        };
        match cmd {
            CMD_LOCK => chip.locked[block] = true,
            CMD_CONFIRM => {
                if chip.locked_down[block] {
                    // `WP#` low, or the block locked down on purpose: the lock
                    // bit cannot be cleared and the attempt is refused.
                    chip.status |= SR_LOCK_ERROR;
                } else {
                    chip.locked[block] = false;
                }
            }
            CMD_LOCK_DOWN => {
                chip.locked[block] = true;
                chip.locked_down[block] = true;
            }
            _ => chip.status |= SR_ERASE_ERROR | SR_PROGRAM_ERROR,
        }
    }

    /// Program one device-width word: **bits only ever go from one to zero**.
    fn program(&self, chip: &mut Chip, offset: u64, value: u16) {
        let Some((block, _, _)) = self.geom.block_at(offset) else {
            chip.status |= SR_PROGRAM_ERROR;
            return;
        };
        if usize::try_from(block).is_ok_and(|b| chip.locked.get(b).copied().unwrap_or(false)) {
            chip.status |= SR_LOCK_ERROR | SR_PROGRAM_ERROR;
            return;
        }
        self.dirty.store(true, Ordering::Relaxed);
        for i in 0..self.geom.device_width() {
            let at = offset + i;
            let Ok(old) = self.array.read_u8(at) else {
                chip.status |= SR_PROGRAM_ERROR;
                return;
            };
            let new = old & (value >> (8 * i)) as u8;
            if self.array.write_u8(at, new).is_err() {
                chip.status |= SR_PROGRAM_ERROR;
                return;
            }
        }
    }

    /// Erase the block `offset` falls in — **this device's bytes of it only**.
    ///
    /// On an interleaved bus each part erases its own lanes. Both parts get the
    /// command in the same 32-bit write, so the whole block comes back to ones;
    /// a machine that talked to one of them alone would see half of it change,
    /// which is what the silicon does.
    fn erase(&self, chip: &mut Chip, offset: u64) {
        let Some((block, base, size)) = self.geom.block_at(offset) else {
            chip.status |= SR_ERASE_ERROR;
            return;
        };
        if usize::try_from(block).is_ok_and(|b| chip.locked.get(b).copied().unwrap_or(false)) {
            chip.status |= SR_LOCK_ERROR | SR_ERASE_ERROR;
            return;
        }
        self.dirty.store(true, Ordering::Relaxed);
        let dw = self.geom.device_width();
        let stride = self.geom.bus_width();
        // `offset` is this device's own lane address, and it is a whole number
        // of device words by the time it reaches here, so its position inside
        // the bus word *is* the lane.
        let mut at = base + offset % stride;
        while at < base + size {
            if self.array.fill(at, dw, 0xff).is_err() {
                chip.status |= SR_ERASE_ERROR;
                return;
            }
            at += stride;
        }
    }

    fn refresh_fast_path(&self, chips: &[Chip]) {
        let all = chips.iter().all(|c| c.mode == Mode::Array);
        self.all_array.store(all, Ordering::Relaxed);
    }

    // -- snapshots ---------------------------------------------------------

    /// Everything except the contents: the command state machines.
    ///
    /// The contents are *not* here, because what to do about them is the
    /// medium's policy (`dev::medium::Snapshot`) and belongs to [`Cfi`]. The
    /// state below is the part's own and is captured whichever policy applies.
    fn save_state(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let chips = self.chips.lock();
        w.write_seq_len(chips.len() as u64)?;
        for chip in chips.iter() {
            w.write_u8(chip.mode.tag())?;
            w.write_u16(chip.status)?;
            w.write_u8(chip.pending.tag())?;
            if let Pending::BufferData { left } = chip.pending {
                w.write_u64(left)?;
            }
            w.write_seq_len(chip.buffer.len() as u64)?;
            for (at, word) in &chip.buffer {
                w.write_u64(*at)?;
                w.write_u16(*word)?;
            }
            w.write_seq_len(chip.locked.len() as u64)?;
            for i in 0..chip.locked.len() {
                w.write_u8(u8::from(chip.locked[i]) | (u8::from(chip.locked_down[i]) << 1))?;
            }
        }
        Ok(())
    }

    /// Put a captured array back, checking it is this part's size.
    fn restore_contents(&self, bytes: &[u8]) -> Result<()> {
        if bytes.len() as u64 != self.geom.size() {
            return Err(Error::State(format!(
                "snapshot has {} byte(s) of flash, this part has {}",
                bytes.len(),
                self.geom.size()
            )));
        }
        self.array
            .write_at(0, bytes)
            .map_err(|_| Error::State(String::from("the flash array refused the snapshot")))?;
        Ok(())
    }

    /// The command state machines, as [`save_state`](Array::save_state) wrote
    /// them.
    fn load_state(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let lanes = r.read_seq_len(1)?;
        let mut chips = self.chips.lock();
        if lanes != chips.len() as u64 {
            return Err(Error::State(format!(
                "snapshot has {lanes} device(s) on the bus, this part has {}",
                chips.len()
            )));
        }
        for chip in chips.iter_mut() {
            chip.mode = Mode::from_tag(r.read_u8()?)?;
            chip.status = r.read_u16()?;
            chip.pending = match r.read_u8()? {
                0 => Pending::None,
                1 => Pending::Program,
                2 => Pending::Erase,
                3 => Pending::Lock,
                4 => Pending::BufferCount,
                5 => {
                    let left = r.read_u64()?;
                    let max = BUFFER_BYTES_PER_DEVICE / self.geom.device_width();
                    if left == 0 || left > max {
                        return Err(Error::State(format!(
                            "a write buffer with {left} word(s) outstanding, and this part \
                             holds {max}"
                        )));
                    }
                    Pending::BufferData { left }
                }
                6 => Pending::BufferConfirm,
                other => {
                    return Err(Error::State(format!(
                        "{other} is not a flash command sequence"
                    )));
                }
            };
            let staged = r.read_seq_len(10)?;
            chip.buffer.clear();
            for _ in 0..staged {
                chip.buffer.push((r.read_u64()?, r.read_u16()?));
            }
            let blocks = r.read_seq_len(1)?;
            if blocks != chip.locked.len() as u64 {
                return Err(Error::State(format!(
                    "snapshot has {blocks} erase block(s), this part has {}",
                    chip.locked.len()
                )));
            }
            for i in 0..chip.locked.len() {
                let bits = r.read_u8()?;
                chip.locked[i] = bits & 1 != 0;
                chip.locked_down[i] = bits & 2 != 0;
            }
        }
        self.refresh_fast_path(&chips);
        Ok(())
    }
}

impl MemOps for Array {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let end = offset
            .checked_add(dst.len() as u64)
            .ok_or(BusError::BadAccess)?;
        if end > self.geom.size() {
            return Err(BusError::BadAccess);
        }
        // A debugger asked what is *in* the flash, not what the bus would
        // answer this microsecond; and it must not be able to tell the command
        // state machine apart from the contents. Reads here never have a side
        // effect either way, so honouring `debug` costs nothing and gains a
        // usable memory dump (`ROADMAP.md` §15, invariant 5).
        if attrs.debug || self.all_array.load(Ordering::Relaxed) {
            return self.array.read_at(offset, dst);
        }
        let chips = self.chips.lock();
        for (i, byte) in dst.iter_mut().enumerate() {
            *byte = self.peek(&chips, offset + i as u64)?;
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        // The rule this device exists to demonstrate: a debug write would
        // advance a command state machine, and a debugger that erased a block
        // by looking at it would be worse than no debugger.
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        let dw = self.geom.device_width();
        let len = src.len() as u64;
        if src.is_empty() || !len.is_multiple_of(dw) || !offset.is_multiple_of(dw) {
            return Err(BusError::BadAccess);
        }
        let end = offset.checked_add(len).ok_or(BusError::BadAccess)?;
        if end > self.geom.size() {
            return Err(BusError::BadAccess);
        }
        let interleave = self.geom.interleave();
        let mut chips = self.chips.lock();
        let mut at = offset;
        for chunk in src.chunks(dw as usize) {
            let lane = ((at / dw) % interleave) as usize;
            let mut value = 0u16;
            for (i, byte) in chunk.iter().enumerate() {
                value |= u16::from(*byte) << (8 * i);
            }
            self.command(&mut chips[lane], at, value);
            at += dw;
        }
        self.refresh_fast_path(&chips);
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Any width, any alignment, bursts allowed: firmware executes out of
        // this window and copies megabytes out of it, and both arrive as
        // whatever width the core felt like. The *write* side is much stricter
        // and enforces itself above, because a command cycle is a bus word.
        AccessConstraints::ANY
            .with_widths(Width::U8, Width::U64)
            .with_endian(Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// the CFI query table
// ---------------------------------------------------------------------------

/// Build the query structure of **one** device (JESD68.01 §4).
///
/// Every figure here describes a single part, not the bus: a driver reading it
/// already knows how many parts are interleaved, because it can see how many
/// copies of `QRY` come back in one bus word.
fn build_query(geom: &Geometry) -> Vec<u8> {
    let device_size = geom.size() / geom.interleave();
    // The erase-block region descriptors start at 0x2d and run four bytes
    // each, so the primary extended table begins where they end. For the
    // single-region part that is 0x31, which is the conventional value and the
    // one a driver that hard-codes it expects.
    let regions = geom.regions().len();
    let extended = QUERY_REGIONS + regions * 4;
    let mut q = alloc::vec![0u8; extended + 0x10];

    // §4.3.1, the identification string.
    q[0x10] = b'Q';
    q[0x11] = b'R';
    q[0x12] = b'Y';
    // Primary command set: Intel/Sharp extended, little end first.
    q[0x13] = 0x01;
    q[0x14] = 0x00;
    // Where the primary extended table lives. No alternate command set, so
    // 0x17..0x1a stay zero.
    q[0x15] = extended as u8;
    q[0x16] = (extended >> 8) as u8;

    // §4.3.2, the system interface description. Voltages are BCD: 2.7 V to
    // 3.6 V, with no separate programming supply.
    q[0x1b] = 0x27;
    q[0x1c] = 0x36;
    q[0x1d] = 0x00;
    q[0x1e] = 0x00;
    // Timeouts, as powers of two. See the module docs: the operations
    // themselves take no guest time, but a driver that sizes its own timeout
    // from these fields must read something plausible.
    q[0x1f] = 0x07; // typical word program: 128 us
    q[0x20] = 0x07; // typical buffer program: 128 us
    q[0x21] = 0x0a; // typical block erase: 1024 ms
    q[0x22] = 0x00; // chip erase: not supported
    q[0x23] = 0x01; // max word program: 2x typical
    q[0x24] = 0x01; // max buffer program: 2x typical
    q[0x25] = 0x02; // max block erase: 4x typical
    q[0x26] = 0x00; // chip erase: not supported

    // §4.3.3, the device geometry.
    q[0x27] = log2(device_size);
    q[0x28] = 0x02; // x8/x16 asynchronous interface
    q[0x29] = 0x00;
    q[0x2a] = log2(BUFFER_BYTES_PER_DEVICE); // max multi-byte write
    q[0x2b] = 0x00;
    q[0x2c] = regions as u8;
    for (i, region) in geom.regions().iter().enumerate() {
        let at = QUERY_REGIONS + i * 4;
        // "y-1" blocks, then the block size in units of 256 bytes, both little
        // end first — and both per device.
        let count = region.count - 1;
        let size = region.size / geom.interleave() / 256;
        q[at] = count as u8;
        q[at + 1] = (count >> 8) as u8;
        q[at + 2] = size as u8;
        q[at + 3] = (size >> 8) as u8;
    }

    // The Intel/Sharp primary extended table (AP-646).
    q[extended] = b'P';
    q[extended + 1] = b'R';
    q[extended + 2] = b'I';
    q[extended + 3] = b'1'; // major version
    q[extended + 4] = b'1'; // minor version
    // Optional features: erase suspend, program suspend, legacy block
    // lock/unlock, and instant individual block locking. No chip erase, no
    // queued erase, no protection registers, no page-mode or synchronous read.
    q[extended + 5] = 0b0010_1110;
    q[extended + 9] = 0x01; // program is supported after an erase suspend
    q[extended + 10] = 0x03; // block status: lock bit and lock-down bit
    q[extended + 11] = 0x00;
    q[extended + 12] = 0x30; // Vcc optimum, BCD: 3.0 V
    q[extended + 13] = 0x00; // no separate Vpp
    q
}

/// The base-two logarithm of a power of two, saturating at 63.
fn log2(value: u64) -> u8 {
    (63 - value.max(1).leading_zeros()) as u8
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// How many bytes move between the array and a medium at a time.
///
/// 64 KiB: big enough that a 32 MiB bank is five hundred calls rather than
/// eight million, and small enough that loading one never allocates a copy of
/// the whole part.
const LOAD_CHUNK: u64 = 64 * 1024;

/// The window an [`Array`] answers behind.
fn region_for(array: &Arc<Array>) -> RegionRef {
    Arc::new(Region::io(
        CLASS_NAME,
        array.geometry().size(),
        Arc::clone(array) as Arc<dyn MemOps>,
    ))
}

/// A CFI NOR flash part, or a set of them interleaved on one bus.
///
/// # Where the bytes live between runs
///
/// A part a guest *writes* is a storage device, and storage that only ever
/// reads is a machine whose guest keeps nothing. So the contents can come from,
/// and go back to, a [`Medium`] — the same seam `ata.disk` and `virtio.blk`
/// store their bytes behind, installed by the run under the media slot's own
/// name:
///
/// ```console
/// rsemu run q35-uefi --flash0 OVMF_CODE.fd --drive flash1=OVMF_VARS.fd
/// ```
///
/// The medium **wins over the media table**, exactly as it does for a drive: a
/// run that named a file meant it. [`Device::flush`] writes the array back at
/// the end of the run, and only when a program or an erase has actually changed
/// it — so a `readonly` firmware bank costs a boolean and a variable store that
/// a guest never wrote costs the same.
#[derive(Debug)]
pub struct Cfi {
    array: Arc<Array>,
    region: RegionRef,
    /// Where the contents came from and where [`flush`](Device::flush) puts
    /// them back. `None` is a part whose bytes live only in this process.
    media: Option<Arc<dyn Medium>>,
}

impl Cfi {
    /// Validate `props`, allocate the array, and copy in the initial image.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is missing or of the wrong kind;
    /// [`Error::Config`] if the geometry is impossible or the image does not
    /// fit.
    pub fn new(props: &Props) -> Result<Cfi> {
        let mut r = props.reader();
        let size = r.require_size("size")?;
        let width = r.or_range("width", DEFAULT_WIDTH, 1..=8)?;
        let interleave = r.or_range("interleave", DEFAULT_INTERLEAVE, 1..=8)?;
        let block = r.or_size("block", DEFAULT_BLOCK)?;
        let blocks = r.optional_list("blocks")?.map(<[Value]>::to_vec);
        let manufacturer =
            r.or_range("manufacturer", u64::from(DEFAULT_MANUFACTURER), 0..=0xffff)?;
        let device_id = r.or_range("device", 0u64, 0..=0xffff)?;
        let read_only = r.or("readonly", false)?;
        let power_up_locked = r.or("locked", true)?;
        let media = r.optional_media("image")?;
        let slot = media.map(crate::core::props::Media::name);
        let image = media.map(crate::core::props::Media::to_bytes);
        r.finish()?;

        // A medium the *host* installed under this bank's media slot name —
        // `--drive flash1=vars.fd`. It wins over the media table's bytes, the
        // way it does for a drive, because a run that named a file meant it.
        let supplied = match (props.hosts(), slot) {
            (Some(hosts), Some(name)) => medium::get(hosts, name)?.and_then(|slot| slot.take()),
            _ => None,
        };

        let geom = match blocks {
            Some(list) => Geometry::new(block_regions(&list)?, width, interleave)?,
            None => Geometry::uniform(size, block, width, interleave)?,
        };
        if geom.size() != size {
            return Err(config(format!(
                "the erase-block regions add up to {} byte(s) and `size` says {size}",
                geom.size()
            )));
        }
        let array = Arc::new(Array::with_options(
            geom,
            manufacturer as u16,
            device_id as u16,
            power_up_locked,
            read_only,
        )?);
        // The medium first, because it wins; the media table's bytes otherwise.
        if let Some(medium) = &supplied {
            // Exactly the bank's size, and not merely no larger. `flush` writes
            // the *whole* array back, so a medium that could not receive all of
            // it would silently keep a prefix — and a bank that came up half
            // erased because the file was short is a firmware that boots once
            // and never again. Say so at construction instead.
            if medium.capacity() != size {
                return Err(config(format!(
                    "the medium bound to this bank holds {} byte(s) and the flash is {size}; a \
                     bank and its backing file are the same size or neither can be written back",
                    medium.capacity()
                )));
            }
            let mut chunk = alloc::vec![0u8; LOAD_CHUNK.min(size) as usize];
            let mut at = 0u64;
            while at < size {
                let take = LOAD_CHUNK.min(size - at) as usize;
                medium
                    .read_at(at, &mut chunk[..take])
                    .map_err(|e| medium::error_at(at, e))?;
                array.load_image(at, &chunk[..take])?;
                at += take as u64;
            }
        } else if let Some(image) = image {
            if image.len() as u64 > size {
                return Err(config(format!(
                    "the bound image is {} byte(s) and the flash is {size}",
                    image.len()
                )));
            }
            array.load_image(0, &image)?;
        }
        Ok(Cfi {
            region: region_for(&array),
            array,
            media: supplied,
        })
    }

    /// Wrap an array that has already been built.
    ///
    /// No medium: the bytes live in this process and nothing writes them back.
    #[must_use]
    pub fn from_array(array: Arc<Array>) -> Cfi {
        Cfi {
            region: region_for(&array),
            array,
            media: None,
        }
    }

    /// The medium the contents came from, if the run installed one.
    #[must_use]
    pub fn medium(&self) -> Option<&Arc<dyn Medium>> {
        self.media.as_ref()
    }

    /// Write the array back to its medium, if it has one and anything changed.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the medium refused a write or the flush.
    pub fn write_back(&self) -> Result<()> {
        let Some(media) = &self.media else {
            return Ok(());
        };
        if !self.array.is_dirty() {
            return Ok(());
        }
        if media.is_read_only() {
            return Err(Error::State(format!(
                "this flash bank's medium ({}) is read only and the guest programmed it",
                media.describe()
            )));
        }
        let size = self.array.geometry().size();
        let mut chunk = alloc::vec![0u8; LOAD_CHUNK.min(size) as usize];
        let mut at = 0u64;
        while at < size {
            let take = LOAD_CHUNK.min(size - at) as usize;
            self.array.read_contents(at, &mut chunk[..take])?;
            media
                .write_at(at, &chunk[..take])
                .map_err(|e| medium::error_at(at, e))?;
            at += take as u64;
        }
        media.flush().map_err(|e| medium::error_at(0, e))?;
        self.array.mark_clean();
        Ok(())
    }

    /// Fill the array from the medium again, as construction did.
    ///
    /// What restoring a [`Snapshot::Reference`] chunk means for a part whose
    /// array is a *copy* of the medium rather than the medium itself.
    ///
    /// # Errors
    ///
    /// [`Error::State`] if the medium refused a read.
    pub fn reload_from_medium(&self) -> Result<()> {
        let Some(media) = &self.media else {
            return Ok(());
        };
        let size = self.array.geometry().size();
        let mut chunk = alloc::vec![0u8; LOAD_CHUNK.min(size) as usize];
        let mut at = 0u64;
        while at < size {
            let take = LOAD_CHUNK.min(size - at) as usize;
            media
                .read_at(at, &mut chunk[..take])
                .map_err(|e| medium::error_at(at, e))?;
            self.array.load_image(at, &chunk[..take])?;
            at += take as u64;
        }
        self.array.mark_clean();
        Ok(())
    }

    /// The parts behind the window.
    #[must_use]
    pub fn array(&self) -> &Arc<Array> {
        &self.array
    }
}

#[cfg(feature = "dev-riscv")]
impl crate::dev::riscv::dt::DtSource for Array {
    fn dt_spec(&self) -> crate::dev::riscv::dt::NodeSpec {
        // `cfi-flash` is the generic binding every CFI driver looks for, and
        // `bank-width` is how it learns the bus is wider than one part. The
        // address and size are supplied by the generator from the mapping, so
        // nothing here writes one down.
        crate::dev::riscv::dt::NodeSpec::peripheral("flash", &["cfi-flash"])
            .with_cells("bank-width", alloc::vec![self.geom.bus_width() as u32])
    }
}

/// Turn a machine file's `blocks = [count, size, ...]` list into regions.
fn block_regions(list: &[Value]) -> Result<Vec<BlockRegion>> {
    if list.is_empty() || !list.len().is_multiple_of(2) {
        return Err(config(String::from(
            "`blocks` is a list of count, size pairs, as in `blocks = [4, 16K, 63, 64K]`",
        )));
    }
    let number = |value: &Value| -> Result<u64> {
        match value {
            Value::Size(n) | Value::Uint(n) | Value::Addr(n) => Ok(*n),
            other => Err(config(format!(
                "`blocks` holds counts and sizes, not {other}"
            ))),
        }
    };
    let mut regions = Vec::with_capacity(list.len() / 2);
    for pair in list.chunks(2) {
        regions.push(BlockRegion {
            count: number(&pair[0])?,
            size: number(&pair[1])?,
        });
    }
    Ok(regions)
}

/// The `flash.cfi` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: 1,
    summary: "CFI NOR flash: real program and erase semantics, Intel/Sharp command set",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: true,
            summary: "how many bytes the whole window holds, as in `size = 32M`",
        },
        PropertySpec {
            name: "image",
            kind: ValueKind::Media,
            required: false,
            summary: "the media slot holding the initial contents; the rest stays erased",
        },
        PropertySpec {
            name: "block",
            kind: ValueKind::Size,
            required: false,
            summary: "the erase-block size when they are all the same (default 256K)",
        },
        PropertySpec {
            name: "blocks",
            kind: ValueKind::List,
            required: false,
            summary: "count, size pairs for a part whose blocks differ: `[4, 16K, 63, 64K]`",
        },
        PropertySpec {
            name: "width",
            kind: ValueKind::Uint,
            required: false,
            summary: "bus width in bytes: 1, 2, 4 or 8 (default 4)",
        },
        PropertySpec {
            name: "interleave",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many parts share that bus (default 2, so two x16 on 32 bits)",
        },
        PropertySpec {
            name: "manufacturer",
            kind: ValueKind::Uint,
            required: false,
            summary: "the JEDEC manufacturer identifier read back in identifier mode",
        },
        PropertySpec {
            name: "device",
            kind: ValueKind::Uint,
            required: false,
            summary: "the device code read back in identifier mode",
        },
        PropertySpec {
            name: "readonly",
            kind: ValueKind::Bool,
            required: false,
            summary: "hold WP# low: every block is locked down and nothing can be written",
        },
        PropertySpec {
            name: "locked",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether blocks power up locked, as an Intel P30 does (default true)",
        },
    ],
    construct: |props| Ok(Box::new(Cfi::new(props)?)),
};

impl Device for Cfi {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    #[cfg_attr(not(feature = "dev-riscv"), expect(unused_variables))]
    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // A `map` statement places the window; this says what the window *is*.
        //
        // The device tree generator is board-local today: it lives with the
        // RISC-V board because that is the only machine that generates a tree
        // (`dev::riscv::dt`). A flash part is not a RISC-V device, so the
        // publication is conditional rather than the module being moved — and
        // when `RealizeCtx` carries the machine graph (`ROADMAP.md` §4.4) this
        // block goes away entirely.
        #[cfg(feature = "dev-riscv")]
        crate::dev::riscv::dt::publish(
            ctx.hosts(),
            &self.region,
            alloc::sync::Arc::downgrade(&self.array)
                as alloc::sync::Weak<dyn crate::dev::riscv::dt::DtSource>,
        )?;
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds, and the contents survive both. Flash is non-volatile:
        // that is the whole reason this device exists rather than a `ram`
        // object with the same address.
        self.array.reset();
    }

    fn flush(&self) -> Result<()> {
        // What a guest asking would have got, without one asking: the run is
        // over and nothing else will program these bytes. A `flash.cfi` that
        // did not implement this would lose every variable the guest wrote and
        // never barriered, which is the defect `dev/usb/storage.rs` was fixed
        // for and the reason this is here rather than in the front end.
        self.write_back()
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // What the contents chunk *is* belongs to the medium, not to the part:
        // a `RamStore` behind a media slot says capture, a host file says
        // reference, and something that cannot be snapshotted at all says so.
        match self
            .media
            .as_ref()
            .map_or(Snapshot::Capture, |m| m.snapshot())
        {
            Snapshot::Capture => w.write_bytes(&self.array.contents())?,
            Snapshot::Reference => {
                // Write back *first*, for the reason `ata.disk` gives: the
                // reference is only worth anything if the file holds what the
                // guest had programmed at the instant the snapshot was taken.
                self.write_back()?;
                let describe = self
                    .media
                    .as_ref()
                    .map_or_else(String::new, |m| m.describe());
                w.write_bytes(describe.as_bytes())?;
            }
            Snapshot::Refuse => {
                return Err(Error::State(format!(
                    "this flash bank's medium ({}) refuses to be snapshotted",
                    self.media
                        .as_ref()
                        .map_or_else(String::new, |m| m.describe())
                )));
            }
        }
        self.array.save_state(w)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let bytes: &[u8] = r.read_bytes()?;
        match self
            .media
            .as_ref()
            .map_or(Snapshot::Capture, |m| m.snapshot())
        {
            Snapshot::Capture => self.array.restore_contents(bytes)?,
            Snapshot::Reference => {
                // The chunk holds *which* medium, and the bytes are still in
                // it. A snapshot taken of a capturing bank lands here as a
                // mismatched identity rather than as a silent misread.
                let want = self
                    .media
                    .as_ref()
                    .map_or_else(String::new, |m| m.describe());
                if bytes != want.as_bytes() {
                    return Err(Error::State(format!(
                        "the snapshot references a different medium: it names `{}` and this \
                         bank holds `{want}`",
                        String::from_utf8_lossy(&bytes[..bytes.len().min(120)])
                    )));
                }
                self.reload_from_medium()?;
            }
            Snapshot::Refuse => {
                return Err(Error::State(format!(
                    "this flash bank's medium ({}) refuses to be snapshotted",
                    self.media
                        .as_ref()
                        .map_or_else(String::new, |m| m.describe())
                )));
            }
        }
        self.array.load_state(r)
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "flash").then(|| Arc::clone(&self.region))
    }
}

impl Instance for Cfi {}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Cfi::new(props)?)))
}

/// What the validator should know about `flash.cfi`.
#[must_use]
pub fn schema() -> ClassSchema {
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size).required())
        .prop(PropSchema::new("image", ValueKind::Media))
        .prop(PropSchema::new("block", ValueKind::Size))
        .prop(PropSchema::new("blocks", ValueKind::List))
        .prop(PropSchema::new("width", ValueKind::Uint).range(1, 8))
        .prop(PropSchema::new("interleave", ValueKind::Uint).range(1, 8))
        .prop(PropSchema::new("manufacturer", ValueKind::Uint).range(0, 0xffff))
        .prop(PropSchema::new("device", ValueKind::Uint).range(0, 0xffff))
        .prop(PropSchema::new("readonly", ValueKind::Bool))
        .prop(PropSchema::new("locked", ValueKind::Bool))
        .region("")
        .region("flash")
}

#[cfg(test)]
mod tests;
