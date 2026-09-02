//! The PCI host bridge, and the thing every current PC firmware asks it for:
//! **RAM shadowing** through the Programmable Attribute Map.
//!
//! # What this part is, and why an "AT" has one
//!
//! An Intel **82441FX (PMC)**, the north bridge half of the 440FX PCIset. That
//! is a 1996 Pentium Pro part and the board this sits on is called `pc-at`, so
//! the choice needs saying out loud: `pc-at` names the *lineage* — the AT's port
//! map, its two cascaded 8259As, its 8042, its CMOS layout — and not a 1984
//! parts list. The board already carries an ELCR at 0x4d0/1, fast A20 at 0x92,
//! reset control at 0xcf9, an IDE cable, a 32-bit address space and a 486; none
//! of those is on a 1984 PC/AT either, and each of them is there for the same
//! reason this is. `machines/pc-at.machine` says the same thing where a reader
//! will meet it first.
//!
//! The PMC is *two* chips' worth of job in one: it is the bridge between the
//! host bus and PCI, **and** it is the DRAM controller. Both halves matter here.
//! The bridge half is why it answers configuration cycles at all; the memory
//! controller half is why it, and not some other device, owns the DRAM that
//! shadowing switches into view. That DRAM is main memory — the same array
//! `ram_low` is part of — and this device holds the 256 KiB of it that lives
//! under `0xc0000-0xfffff`, because a machine file cannot hand one device a
//! reference to another's store (`docs/platforms/pc-at.md`, "framework gaps").
//!
//! # What shadowing is, in the datasheet's own words
//!
//! *Intel 440FX PCIset: 82441FX (PMC) and 82442FX (DBX)*, order number
//! 290549-001, **§3.2.18, "PAM—Programmable Attribute Map Registers
//! (`PAM[6:0]`)"**, address offset **59h-5Fh**, default **00h**:
//!
//! > The PMC allows programmable memory attributes on 13 memory segments of
//! > various sizes in the 640-Kbyte to 1-Mbyte address range. […] Two bits are
//! > used to specify memory attributes for each memory segment.
//! >
//! > **RE** Read Enable. When RE=1, CPU read accesses to the corresponding
//! > memory segment are claimed by the PMC and directed to main memory.
//! > Conversely, when RE=0, the CPU read accesses are directed to PCI.
//! >
//! > **WE** Write Enable. When WE=1, CPU write accesses to the corresponding
//! > memory segment are claimed by the PMC and directed to main memory.
//! > Conversely, when WE=0, the CPU write accesses are directed to PCI.
//!
//! Each register holds two 4-bit fields. §3.2.18's Table 2, *Attribute Bit
//! Assignment*, gives the encoding: bits **`[7,6,3,2]`** are reserved, bits
//! **`[5,1]`** are `WE` and bits **`[4,0]`** are `RE` — so the low nibble governs
//! one window and the high nibble the next one up, and within a nibble the low
//! bit is read and the next is write.
//!
//! ```text
//!   7   6   5   4   3   2   1   0
//!  ---+---+---+---+---+---+---+---
//!   R | R | WE| RE| R | R | WE| RE
//!    high window        low window
//! ```
//!
//! And §3.2.18's Table 3, *PAM Registers and Associated Memory Segments*, says
//! which window is which. Twelve 16 KiB windows and one 64 KiB one:
//!
//! ```text
//!   PAM0[7:4]  0F0000-0FFFFF  64K  System BIOS
//!   PAM1[3:0]  0C0000-0C3FFF  16K  ISA add-on BIOS
//!   PAM1[7:4]  0C4000-0C7FFF  16K  ISA add-on BIOS
//!   PAM2[3:0]  0C8000-0CBFFF  16K  ISA add-on BIOS
//!   PAM2[7:4]  0CC000-0CFFFF  16K  ISA add-on BIOS
//!   PAM3[3:0]  0D0000-0D3FFF  16K  ISA add-on BIOS
//!   PAM3[7:4]  0D4000-0D7FFF  16K  ISA add-on BIOS
//!   PAM4[3:0]  0D8000-0DBFFF  16K  ISA add-on BIOS
//!   PAM4[7:4]  0DC000-0DFFFF  16K  ISA add-on BIOS
//!   PAM5[3:0]  0E0000-0E3FFF  16K  BIOS extension
//!   PAM5[7:4]  0E4000-0E7FFF  16K  BIOS extension
//!   PAM6[3:0]  0E8000-0EBFFF  16K  BIOS extension
//!   PAM6[7:4]  0EC000-0EFFFF  16K  BIOS extension
//!   PAM0[3:0]  reserved
//! ```
//!
//! §3.2.18 also states the recipe firmware follows, and it is the reason the
//! *write-only* combination has to work rather than being an oddity:
//!
//! > To shadow the BIOS, the attributes for that address range should be set to
//! > write only. The BIOS is shadowed by first doing a read of that address.
//! > This read is forwarded to the expansion bus. The CPU then does a write of
//! > the same address, which is directed to main memory. After the BIOS is
//! > shadowed, the attributes for that memory area are set to read only so that
//! > all writes are forwarded to the expansion bus.
//!
//! # How that maps onto `core::space`, which already had what it needs
//!
//! `ROADMAP.md`:553 lists "overlapping regions with priority — PCI BAR over
//! RAM; NES cartridge mappers; boot ROM shadowing" as a `core::space`
//! capability, and it is **already there**: [`map_with_priority`] plus the
//! flattener's rule that
//!
//! > reads and writes are resolved **separately**: the highest-priority mapping
//! > that permits reads need not be the one that permits writes
//!
//! (`core::space` module docs). So a PAM window is one mapping of this device's
//! DRAM at priority 1, sitting over whatever the board decodes at priority 0,
//! and the four attribute encodings are three [`Perms`] values on that one
//! mapping plus **no mapping at all**:
//!
//! ```text
//!   RE WE  datasheet     mapping       reads       writes
//!   0  0   Disabled      none          the bus     the bus
//!   1  0   Read Only     READ          the DRAM    the bus
//!   0  1   Write Only    WRITE         the bus     the DRAM
//!   1  1   Read/Write    RW            the DRAM    the DRAM
//! ```
//!
//! "Directed to PCI" and "falls through to the next mapping down" are the same
//! statement about the same decode, which is why no new region kind was needed.
//! [`Perms::EXEC`] rides along with [`Perms::READ`] because the firmware
//! *executes* out of the window it shadowed, and the bit is carried rather than
//! enforced (`core::space::Perms`).
//!
//! The *Disabled* row is not mapped with [`Perms::NONE`], and the difference is
//! not cosmetic: a mapping that claims a range and refuses both directions is
//! only equivalent to no mapping where something else is decoded underneath.
//! Over a hole it is the opposite, and it cost this board thirty-two bus faults
//! on every POST — `Registers::retopo` has the long form.
//!
//! Nothing in `core::space` was changed for this.
//!
//! # The one hard part: retopologising from inside a write handler
//!
//! A PAM write arrives as an `OUT` to `0xcfc`, so the I/O space's topology lock
//! is held for reading and the x86 core's own `BUS`-ranked lock is held above
//! it. [`AddressSpace::topology`] is therefore forbidden here — same rank
//! twice, and `core::sync`'s ladder panics on it in debug builds — and the
//! documented spelling, a [`Deferred`](crate::core::device::Deferred) action,
//! does not fit either: the queue is drained after an *event*, so the remap
//! would land a whole scheduler quantum later, thousands of instructions after
//! the `memcpy` it was supposed to redirect.
//!
//! So this uses [`AddressSpace::try_topology`], which is **order-exempt by
//! construction** — "a failed try-lock cannot join a deadlock cycle, so this is
//! order-exempt and never trips the rank check" — on a *different* space from
//! the one the access is travelling through. A try-lock cannot wait, so it
//! cannot be half of a cycle; the ladder's reason for forbidding the blocking
//! guard does not reach it.
//!
//! It can still *fail*, if something else is retopologising `mem` at that
//! instant. That is not swallowed: the device remembers that its mapping is
//! stale and re-applies at the next configuration access, at reset, and after a
//! snapshot load. Nothing on this board retopologises `mem` at all, so the path
//! is exercised by a test rather than by the machine.
//!
//! # What is not modelled
//!
//! Everything the 440FX has that shadowing does not need, and it is a long
//! list: DRAM row boundaries and timing (`DRB`, `DRAMC`, `DRAMT` — this machine
//! has no DIMMs to size), the fixed DRAM hole (`FDHC`), SMRAM (`SMRAM`), the
//! deturbo counter, the error registers, and the whole 82442FX data path. They
//! read as zero, which is what a reserved register reads as (Rev 2.1 §6.1), and
//! a firmware that wrote one would find it did nothing — which is honest,
//! because on this board it *would* do nothing.
//!
//! `PCICMD` bits are latched and read back but change nothing: this bridge has
//! no cycles to enable or disable, and pretending otherwise would be inventing
//! behaviour. §3.2.4 hardwires bits 0-4 and 7 and 9 anyway.
//!
//! [`map_with_priority`]: crate::core::space::TopologyGuard::map_with_priority
//! [`AddressSpace::topology`]: crate::core::space::AddressSpace::topology
//! [`AddressSpace::try_topology`]: crate::core::space::AddressSpace::try_topology

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;

use crate::bus::pci::{
    Bdf, CONFIG_PORT_WINDOW_LEN, ConfigPorts, ConfigSpace, PciBus, PciFunction, buses, config,
};
use crate::core::device::{Device, DeviceClass, ExportId, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AddressSpace, MappingId, MemAttrs, Perms, RamStore, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.pmc";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The 82441FX's device identification (datasheet §3.2.3, default `1237h`).
const DEVICE_ID: u16 = 0x1237;

/// Where the shadowable region starts: the low edge of `PAM1[3:0]`'s window.
pub const SHADOW_BASE: u64 = 0x000c_0000;

/// How much of it there is: `0xc0000`-`0xfffff`, 256 KiB.
pub const SHADOW_LEN: u64 = 0x0004_0000;

/// The priority the shadow DRAM is mapped at.
///
/// One above the default, so it sits over whatever the machine file decodes in
/// the same window — the ROM sockets on this board — and falls through to it
/// wherever [`Perms`] says the PMC does not claim the cycle.
const SHADOW_PRIORITY: i32 = 1;

/// Configuration offset of `PAM0` (datasheet §3.2.18).
const PAM0: u16 = 0x59;

/// How many PAM registers there are: `PAM0`-`PAM6` at 59h-5Fh.
const PAM_COUNT: u16 = 7;

/// One shadowable window: where it is, how big, and which nibble of which PAM
/// register controls it.
///
/// Straight out of §3.2.18's Table 3, in address order rather than the table's
/// register order, because that is the order the mappings are made in.
struct Window {
    /// Guest-physical base.
    base: u64,
    /// Length in bytes.
    len: u64,
    /// Configuration offset of the PAM register that governs it.
    reg: u16,
    /// How far to shift that register right to bring the nibble to bit 0: 0 for
    /// the low window, 4 for the high one.
    shift: u32,
}

/// The thirteen segments §3.2.18 names, in address order.
const WINDOWS: [Window; 13] = [
    Window {
        base: 0xc_0000,
        len: 0x4000,
        reg: PAM0 + 1,
        shift: 0,
    },
    Window {
        base: 0xc_4000,
        len: 0x4000,
        reg: PAM0 + 1,
        shift: 4,
    },
    Window {
        base: 0xc_8000,
        len: 0x4000,
        reg: PAM0 + 2,
        shift: 0,
    },
    Window {
        base: 0xc_c000,
        len: 0x4000,
        reg: PAM0 + 2,
        shift: 4,
    },
    Window {
        base: 0xd_0000,
        len: 0x4000,
        reg: PAM0 + 3,
        shift: 0,
    },
    Window {
        base: 0xd_4000,
        len: 0x4000,
        reg: PAM0 + 3,
        shift: 4,
    },
    Window {
        base: 0xd_8000,
        len: 0x4000,
        reg: PAM0 + 4,
        shift: 0,
    },
    Window {
        base: 0xd_c000,
        len: 0x4000,
        reg: PAM0 + 4,
        shift: 4,
    },
    Window {
        base: 0xe_0000,
        len: 0x4000,
        reg: PAM0 + 5,
        shift: 0,
    },
    Window {
        base: 0xe_4000,
        len: 0x4000,
        reg: PAM0 + 5,
        shift: 4,
    },
    Window {
        base: 0xe_8000,
        len: 0x4000,
        reg: PAM0 + 6,
        shift: 0,
    },
    Window {
        base: 0xe_c000,
        len: 0x4000,
        reg: PAM0 + 6,
        shift: 4,
    },
    // PAM0[7:4]: the system BIOS area, and the only 64 KiB one. PAM0[3:0] is
    // reserved, which is why this register's low nibble governs nothing.
    Window {
        base: 0xf_0000,
        len: 0x1_0000,
        reg: PAM0,
        shift: 4,
    },
];

/// How many windows there are, as a `usize` for array sizing.
const N: usize = WINDOWS.len();

/// The nibble's read-enable bit (§3.2.18 Table 2, bits `[4,0]`).
const RE: u8 = 0x1;
/// The nibble's write-enable bit (§3.2.18 Table 2, bits `[5,1]`).
const WE: u8 = 0x2;

/// Turn one attribute nibble into the terms its mapping answers on.
///
/// The whole of shadowing is this function; everything else is plumbing.
fn perms_of(nibble: u8) -> Perms {
    let mut p = Perms::NONE;
    if nibble & RE != 0 {
        // `EXEC` rides with `READ` because the firmware executes out of the
        // window it shadowed. The bit is carried rather than enforced
        // (`core::space::Perms`), so this costs nothing and means a consumer
        // that ever does enforce it gets the right answer.
        p = p.union(Perms::READ).union(Perms::EXEC);
    }
    if nibble & WE != 0 {
        p = p.union(Perms::WRITE);
    }
    p
}

/// The bridge's registers and the DRAM they switch into view.
///
/// Separate from [`Pmc`] because a [`PciFunction`] has to be reachable as an
/// `Arc<dyn PciFunction>` and `Device::realize` only ever has `&self` — the
/// same shape [`sysctl`](super::sysctl) uses for the same reason.
struct Registers {
    /// The 256 bytes of configuration space, and which of them the guest may
    /// move. At [`LockRank::DEVICE`], released before anything outward.
    config: Mutex<ConfigSpace>,
    /// The `0xcf8`/`0xcfc` window. Owns the `CONFADD` latch.
    ports: Arc<ConfigPorts>,
    /// The main memory that lives under `0xc0000-0xfffff`.
    dram: Arc<RamStore>,
    /// One alias per window, kept so the mappings can be made at bind time.
    windows: Vec<RegionRef>,
    /// The space the windows are mapped into, and their mapping ids. `None`
    /// until [`Instance::bind`]. At [`LockRank::LEAF`]: read, cloned and
    /// released before the topology guard is taken.
    ///
    /// A window's id is itself `None` while its PAM nibble is *Disabled*,
    /// because a disabled window is not mapped at all — see [`Self::retopo`].
    mapped: Mutex<Option<Mapped>>,
    /// Set when a retopology could not be performed at the instant it was
    /// asked for, so the next opportunity re-applies. Derived state: never
    /// serialized, and a load re-applies unconditionally anyway.
    stale: Mutex<bool>,
}

/// Where the windows went.
#[derive(Debug, Clone)]
struct Mapped {
    space: Arc<AddressSpace>,
    ids: Vec<Option<MappingId>>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        match self.config.try_lock() {
            Some(c) => s.field(
                "pam",
                &[
                    c.byte(PAM0),
                    c.byte(PAM0 + 1),
                    c.byte(PAM0 + 2),
                    c.byte(PAM0 + 3),
                    c.byte(PAM0 + 4),
                    c.byte(PAM0 + 5),
                    c.byte(PAM0 + 6),
                ],
            ),
            None => s.field("pam", &"<in use>"),
        };
        s.field("mapped", &self.mapped.try_lock().map(|m| m.is_some()))
            .finish()
    }
}

impl Registers {
    /// The header this part hardwires, from datasheet Table 1 and §3.2.2-§3.2.10.
    fn fresh_config(revision: u8) -> ConfigSpace {
        let mut c = ConfigSpace::new();
        c.hardwire(config::VENDOR_ID, u32::from(config::VENDOR_INTEL), 2);
        c.hardwire(config::DEVICE_ID, u32::from(DEVICE_ID), 2);
        // §3.2.4: default 0006h. Bits 1 and 2 are hardwired to 1 (the PMC
        // always allows PCI master access to main memory and is always a bus
        // master) and bit 0 to 0 (it does not respond to PCI I/O cycles).
        c.hardwire(config::COMMAND, 0x0006, 2);
        // §3.2.5: default 0280h — DEVSEL# timing 01b (medium) in bits 10:9 and
        // fast back-to-back in bit 7.
        c.hardwire(config::STATUS, 0x0280, 2);
        c.hardwire(config::REVISION_ID, u32::from(revision), 1);
        // §3.2.7: 060000h — base class 06 (bridge), sub-class 00 (host bridge),
        // programming interface 00.
        c.hardwire(config::CLASS_CODE, 0x00, 1);
        c.hardwire(
            config::CLASS_CODE + 1,
            u32::from(config::SUBCLASS_HOST_BRIDGE),
            1,
        );
        c.hardwire(config::CLASS_CODE + 2, u32::from(config::CLASS_BRIDGE), 1);
        // §3.2.9: header type 00h, the basic configuration space format. Not
        // multi-function: the PMC responds only to function 0 (§3.1.1).
        c.hardwire(config::HEADER_TYPE, 0x00, 1);

        // What the guest may move. §3.2.8's MLT and §3.2.10's BIST are
        // read/write and do nothing here; the PAM registers are the point.
        c.allow(config::COMMAND, 2);
        c.allow(config::LATENCY_TIMER, 1);
        c.allow(config::BIST, 1);
        c.allow(PAM0, PAM_COUNT);
        c
    }

    /// The terms each window should answer on, given the PAM registers now.
    fn perms(&self) -> Vec<Perms> {
        let c = self.config.lock();
        WINDOWS
            .iter()
            .map(|w| perms_of((c.byte(w.reg) >> w.shift) & 0xf))
            .collect()
    }

    /// Bring the mappings into line with `perms`. Returns whether it could be
    /// done.
    ///
    /// `blocking` picks which guard: the write guard where nothing is in
    /// flight — reset, a snapshot load, bind — and the order-exempt try-lock
    /// from inside a configuration write, where the blocking one would invert
    /// the ladder. See the module docs.
    ///
    /// # Why a disabled window is unmapped rather than mapped with no terms
    ///
    /// §3.2.18's *Disabled* encoding says "both read and write cycles are
    /// directed to the expansion bus", which is a statement that the DRAM does
    /// not answer — not that the bridge answers and refuses. Those are the same
    /// thing only where the board decodes something underneath. Over a hole
    /// they are opposite: `core::space` resolves each direction to the
    /// highest-priority mapping that permits it, and where *no* mapping permits
    /// it the winner is whatever is there, whose own permissions then raise
    /// [`BusError::Protected`](crate::core::space::BusError::Protected) — the
    /// space's `unassigned` policy never gets a say, because the range is not
    /// unassigned. On this board `0xd0000`-`0xdffff` has no ROM socket, so an
    /// option-ROM scan across it took thirty-two bus faults off a machine whose
    /// bus reads as ones. Unmapping the window makes the range genuinely
    /// unassigned again, which is what an ISA bus with nothing on it is.
    ///
    /// The single-direction encodings still claim the range in *both*
    /// directions, because one mapping is one range: a *Read Only* window over
    /// a hole refuses writes rather than dropping them. That is inherent in a
    /// per-mapping permission and it costs nothing here — firmware sets read
    /// only after it has shadowed something, which is over a ROM.
    fn retopo(&self, perms: &[Perms], blocking: bool) -> bool {
        // Cloned out and the lock released: nothing of this device's is held
        // while the space is retopologised, which is what the re-entrancy
        // contract asks for.
        let Some(mapped) = self.mapped.lock().clone() else {
            // Not bound yet. Not stale either: `bind` maps with the right terms
            // in the first place.
            return true;
        };
        let guard = if blocking {
            Some(mapped.space.topology())
        } else {
            mapped.space.try_topology()
        };
        let Some(mut topo) = guard else {
            *self.stale.lock() = true;
            return false;
        };
        let mut ids = Vec::with_capacity(N);
        for ((id, p), w) in mapped.ids.iter().zip(perms).zip(&WINDOWS) {
            let want = *p != Perms::NONE;
            ids.push(match (*id, want) {
                // Still claimed, and possibly on different terms. The only
                // error is "not a mapping of this space", which cannot happen:
                // these ids came from this guard's own space.
                (Some(id), true) => {
                    let _ = topo.reprotect(id, *p);
                    Some(id)
                }
                // Newly claimed.
                (None, true) => topo
                    .map_with(
                        crate::core::space::Mapping::new(
                            Arc::clone(&self.windows[ids.len()]),
                            w.base,
                        )
                        .with_priority(SHADOW_PRIORITY)
                        .with_perms(*p),
                    )
                    .ok(),
                // Disabled: the cycle belongs to the expansion bus.
                (Some(id), false) => {
                    let _ = topo.unmap(id);
                    None
                }
                (None, false) => None,
            });
        }
        drop(topo);
        *self.mapped.lock() = Some(Mapped {
            space: Arc::clone(&mapped.space),
            ids,
        });
        *self.stale.lock() = false;
        true
    }

    /// Bring the mappings into line with the PAM registers.
    fn sync(&self, blocking: bool) -> bool {
        let perms = self.perms();
        self.retopo(&perms, blocking)
    }

    /// Claim the space and put the thirteen windows in it on the terms the PAM
    /// registers currently ask for. **Retopology**, and legal here: `bind` runs
    /// during machine assembly with no access in flight.
    fn install(&self, space: &Arc<AddressSpace>) -> Result<()> {
        *self.mapped.lock() = Some(Mapped {
            space: Arc::clone(space),
            ids: alloc::vec![None; N],
        });
        // Out of reset every PAM register is `00h`, so this ordinarily maps
        // nothing at all; a snapshot loaded before bind would map what it says.
        self.sync(true);
        Ok(())
    }
}

impl PciFunction for Registers {
    fn config_read(&self, offset: u16, dst: &mut [u8], _attrs: MemAttrs) {
        // No `debug` branch: a configuration read of this bridge has no side
        // effects at all — there is no status bit here that a read clears and
        // no FIFO to pop — so a debugger's window may poll it freely.
        self.config.lock().read(offset, dst);
        // A retopology that could not happen when it was asked for gets its
        // next chance here. Deliberately *after* the read, and deliberately not
        // gated on `attrs.debug` being clear: re-applying what the guest
        // already wrote changes nothing a debugger could observe, and leaving
        // the machine's memory map disagreeing with its own registers is worse
        // than either.
        if *self.stale.lock() {
            self.sync(false);
        }
    }

    fn config_write(&self, offset: u16, src: &[u8], attrs: MemAttrs) {
        if attrs.debug {
            // A debug write here could move a PAM window under the guest's
            // feet, which is exactly what `MemAttrs::debug` exists to forbid.
            // `ConfigPorts` refuses one before it reaches here; this is the
            // second lock on the same door, for a caller that reaches the
            // function directly.
            return;
        }
        let (changed, perms) = {
            let mut c = self.config.lock();
            let changed = c.write(offset, src);
            let perms = WINDOWS
                .iter()
                .map(|w| perms_of((c.byte(w.reg) >> w.shift) & 0xf))
                .collect::<Vec<_>>();
            (changed, perms)
        };
        // Only when a PAM byte actually moved. Firmware rewrites the same
        // attributes constantly and re-flattening an address space for a write
        // that changed nothing is pure cost.
        let touches_pam =
            offset < PAM0 + PAM_COUNT && offset.saturating_add(src.len() as u16) > PAM0;
        if (changed && touches_pam) || *self.stale.lock() {
            self.retopo(&perms, false);
        }
    }
}

/// The 82441FX PCI and memory controller: a host bridge with PAM.
#[derive(Debug)]
pub struct Pmc {
    regs: Arc<Registers>,
    bus: Arc<PciBus>,
    at: Bdf,
    config_region: RegionRef,
    revision: u8,
    /// The sibling whose own decode lives inside `CONFADD`'s four bytes, if the
    /// machine file named one. Resolved at [`Instance::bind`].
    passthrough: Mutex<Option<String>>,
}

impl Pmc {
    /// Validate `props` and build the device.
    ///
    /// Allocation and validation only: the fabric handle is acquired here
    /// because acquiring a host object *is* allocation
    /// ([`core::hosts`](crate::core::hosts)), and nothing is announced onto it
    /// until [`realize`](Device::realize).
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property this class does not know, a device
    /// number outside 0-31, or a revision outside a byte;
    /// [`Error::Config`] if the `bus` name is already open as something else.
    pub fn new(props: &Props) -> Result<Pmc> {
        let mut r = props.reader();
        let bus_name = r.or_str("bus", "pci0")?.to_string();
        let device = r.or_range("device", 0u64, 0..=u64::from(crate::bus::pci::MAX_DEVICE))?;
        let revision = r.or_range("revision", 0u64, 0..=255)?;
        let passthrough = r
            .optional_link("passthrough")?
            .map(|l| String::from(l.as_str()));
        r.finish()?;
        let bus = buses::attach(props, &bus_name)?;
        // §3.1.1: "The PMC is always Device Number 0." The property exists so a
        // machine file can say so out loud, and so a second host bridge in some
        // future board is expressible; the default is the part's own answer.
        let at = Bdf::new(0, device as u8, 0)?;
        let pmc = Pmc::with_bus(bus, at, revision as u8)?;
        *pmc.passthrough.lock() = passthrough;
        Ok(pmc)
    }

    /// The same device, built from a fabric handle a test already has.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if a window does not fit its own store, which is a
    /// bug in this module's window table rather than anything a caller can cause.
    pub fn with_bus(bus: Arc<PciBus>, at: Bdf, revision: u8) -> Result<Pmc> {
        let dram = Arc::new(RamStore::new(SHADOW_LEN));
        let whole: RegionRef = Arc::new(Region::ram("pc.pmc.dram", Arc::clone(&dram)));
        let mut windows = Vec::with_capacity(N);
        for w in &WINDOWS {
            windows.push(Arc::new(Region::alias(
                alloc::format!("pc.pmc.shadow.{:05x}", w.base),
                Arc::clone(&whole),
                w.base - SHADOW_BASE,
                w.len,
            )?) as RegionRef);
        }
        let ports = Arc::new(ConfigPorts::new(Arc::clone(&bus)));
        let config_region: RegionRef = Arc::new(Region::io(
            "pc.pmc.config",
            CONFIG_PORT_WINDOW_LEN,
            Arc::clone(&ports) as Arc<dyn crate::core::space::MemOps>,
        ));
        Ok(Pmc {
            regs: Arc::new(Registers {
                config: Mutex::with_rank(LockRank::DEVICE, Registers::fresh_config(revision)),
                ports,
                dram,
                windows,
                mapped: Mutex::with_rank(LockRank::LEAF, None),
                stale: Mutex::with_rank(LockRank::LEAF, false),
            }),
            bus,
            at,
            config_region,
            revision,
            passthrough: Mutex::with_rank(LockRank::LEAF, None),
        })
    }

    /// The DRAM this bridge switches into `0xc0000-0xfffff`.
    ///
    /// For a test or a debugger that wants to see what the firmware copied
    /// there; the guest reaches it through the address space, when PAM says so.
    #[must_use]
    pub fn dram(&self) -> &Arc<RamStore> {
        &self.regs.dram
    }

    /// Where this bridge sits on its fabric.
    #[must_use]
    pub fn address(&self) -> Bdf {
        self.at
    }

    /// The current value of one PAM register, `0`-`6`.
    ///
    /// `None` for an index past `PAM6`.
    #[must_use]
    pub fn pam(&self, index: u16) -> Option<u8> {
        (index < PAM_COUNT).then(|| self.regs.config.lock().byte(PAM0 + index))
    }

    /// Map the thirteen shadow windows into `space`. **Retopology.**
    ///
    /// What [`Instance::bind`] does, reachable directly so a unit test can
    /// assemble a bridge without a machine — the same shape
    /// [`Dma8237::attach_bus`](super::dma::Dma8237::attach_bus) offers.
    ///
    /// # Errors
    ///
    /// Whatever the space refuses: a window that does not fit, or a nesting
    /// depth this space will not take.
    pub fn attach_space(&self, space: &Arc<AddressSpace>) -> Result<()> {
        self.regs.install(space)
    }
}

/// The `pc.pmc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an Intel 82441FX PCI host bridge, with the PAM registers that shadow the BIOS",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the PCI fabric this bridge is the root of (default `pci0`)",
        },
        PropertySpec {
            name: "device",
            kind: ValueKind::Uint,
            required: false,
            summary: "the device number it answers at on bus 0 (default 0, which is the part's own)",
        },
        PropertySpec {
            name: "revision",
            kind: ValueKind::Uint,
            required: false,
            summary: "the revision identification byte (default 0)",
        },
        PropertySpec {
            name: "passthrough",
            kind: ValueKind::Link,
            required: false,
            summary: "the sibling whose own decode lives inside CONFADD's four bytes at 0xcf8",
        },
    ],
    construct: |props| Ok(Box::new(Pmc::new(props)?)),
};

impl Device for Pmc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The one outward action: announcing itself onto the fabric. Nothing
        // observable happened before this (`CLAUDE.md`, two-phase construction).
        self.bus
            .attach(self.at, Arc::clone(&self.regs) as Arc<dyn PciFunction>)
    }

    fn reset(&self, kind: ResetKind) {
        // `PCIRST#` clears the configuration registers, and rsemu's warm reset
        // stands in for it: on this board the reset paths that reach a warm
        // reset are port 0x92 bit 0 and 0xcf9 bit 2, and the chipset's own
        // reset control register asserting PCIRST# is exactly what the latter
        // does. PAM back to 00h means the ROM is decoded again, which is the
        // state firmware expects to find at its reset vector.
        *self.regs.config.lock() = Registers::fresh_config(self.revision);
        self.regs.ports.reset();
        if kind == ResetKind::Cold {
            // Power clears memory; a reset line does not — the same rule the
            // `ram` object follows, and what makes a "did we come from
            // power-on?" check in a guest work.
            let _ = self.regs.dram.fill(0, SHADOW_LEN, 0);
        }
        // Blocking, and correct: a reset runs from the machine's own loop with
        // no access in flight.
        self.regs.sync(true);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            // `""` is the configuration port pair, because it is the only
            // region this device publishes.
            "" | "config" => Some(Arc::clone(&self.config_region)),
            _ => None,
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        // The writable configuration bytes. The hardwired ones are this model's
        // and are not restored from a snapshot — `ConfigSpace::restore` says
        // why — but saving all 256 keeps the chunk trivially diffable and
        // costs nothing.
        w.write_bytes(self.regs.config.lock().bytes())?;
        w.write_u32(self.regs.ports.address())?;
        // The shadow DRAM is guest-visible state: the firmware copies itself
        // into it and then executes out of it, so a snapshot that dropped it
        // would resume into 256 KiB of zeroes.
        let len = usize::try_from(SHADOW_LEN)
            .map_err(|_| Error::State(String::from("shadow larger than this host")))?;
        let mut bytes = alloc::vec![0u8; len];
        self.regs.dram.read_at(0, &mut bytes)?;
        w.write_bytes(&bytes)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let config: &[u8] = r.read_bytes()?;
        let address = r.read_u32()?;
        let dram: &[u8] = r.read_bytes()?;
        if dram.len() as u64 != SHADOW_LEN {
            return Err(Error::State(alloc::format!(
                "snapshot has {} byte(s) of shadow DRAM, this bridge has {SHADOW_LEN}",
                dram.len()
            )));
        }
        {
            let mut c = self.regs.config.lock();
            *c = Registers::fresh_config(self.revision);
            c.restore(config);
        }
        self.regs.ports.set_address(address);
        self.regs.dram.write_at(0, dram)?;
        // The memory map is a function of the PAM registers, so it is rebuilt
        // rather than saved (`CLAUDE.md`: derived state is never serialized).
        self.regs.sync(true);
        Ok(())
    }
}

impl Instance for Pmc {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "a host bridge decides what is decoded in 0xc0000-0xfffff, so it needs the \
                 space it decides for: add `space = mem` to the object that declares it",
            ),
        })?;
        self.attach_space(space)?;
        // `CONFADD` is four bytes the bridge claims only for a Dword access,
        // and on a PC the byte at 0xcf9 belongs to the south bridge. An address
        // space decodes by address alone, so this holds all four and hands the
        // narrow ones on — 82441FX §3.1.1's own "pass through".
        let wanted = self.passthrough.lock().clone();
        if let Some(path) = wanted {
            let handle =
                ctx.export_as::<super::PortPassthrough>(&path, ExportId::PORT_PASSTHROUGH)?;
            self.regs.ports.set_passthrough(Arc::clone(handle.ops()));
        }
        Ok(())
    }
}

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// [`Error::Config`] if the name is claimed.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if the class is bound twice.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Pmc::new(props)?)))
}

/// What the validator should know about `pc.pmc`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::PropSchema;
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(
            PropSchema::new("device", ValueKind::Uint)
                .range(0, u64::from(crate::bus::pci::MAX_DEVICE)),
        )
        .prop(PropSchema::new("revision", ValueKind::Uint).range(0, 255))
        .prop(PropSchema::new("passthrough", ValueKind::Link))
        .region("")
        .region("config")
}

#[cfg(test)]
mod tests;
