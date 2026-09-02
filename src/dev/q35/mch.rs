//! The q35 host bridge: an Intel 82Q35 (G)MCH, its PAM registers, and the
//! memory-mapped window onto configuration space.
//!
//! # What this part is
//!
//! Device 0, Function 0 of an Intel 3 Series chipset — the DRAM controller and
//! the root of the PCI Express hierarchy. It is the north half of a q35; the
//! south half is [`super::lpc`].
//!
//! It does three things here, and the third is the one a 440FX cannot do:
//!
//! 1. It **answers configuration cycles** as function 0:0.0, so a firmware
//!    that enumerates the bus finds a host bridge and knows what board it is
//!    on.
//! 2. It **shadows the BIOS** through seven PAM registers, exactly as
//!    [`crate::dev::pc::pmc`] does for the 440FX — with the registers at
//!    different offsets and the segment table identical.
//! 3. It **publishes ECAM**: `PCIEXBAR` names a window of *memory* in which
//!    the address is the configuration address, which is how software reaches
//!    a PCI Express function's 4 KiB of configuration space at all
//!    ([`super::ecam`]).
//!
//! The `0xcf8`/`0xcfc` port pair is still here, because a q35 still has it and
//! because every firmware starts there. ECAM is the addition, not the
//! replacement.
//!
//! # The registers, from the datasheet
//!
//! *Intel 3 Series Express Chipset Family Datasheet*, order number 316966-002,
//! Table 5-1 (`DRAM Controller Register Address Map (D0:F0)`):
//!
//! ```text
//!   00-01h  VID       Vendor Identification                8086h
//!   02-03h  DID       Device Identification                see §5.1.2
//!   04-05h  PCICMD    PCI Command                          0006h
//!   06-07h  PCISTS    PCI Status                           0090h
//!     08h   RID       Revision Identification                00h
//!   09-0Bh  CC        Class Code                          060000h
//!     0Eh   HDR       Header Type                            00h
//!   60-67h  PCIEXBAR  PCI Express Register Range Base   00000000E0000000h
//!     90h   PAM0      Programmable Attribute Map 0           00h
//!     ...
//!     96h   PAM6      Programmable Attribute Map 6           00h
//! ```
//!
//! **§5.1.2** gives the device identification per variant, and the number this
//! board wants is the first:
//!
//! > 29B0h = Intel 82Q35 GMCH
//! > 29C0h = Intel 82G33/82P35 (G)MCH
//! > 29D0h = Intel 82Q33 GMCH
//!
//! ## PAM, at 90h-96h rather than 59h-5Fh
//!
//! §5.1.18-§5.1.24. Thirteen segments between `0xc0000` and `0xfffff`, two per
//! register except `PAM0`, whose low nibble is reserved and whose high nibble
//! governs the 64 KiB system BIOS at `0xf0000`. Each nibble is two bits:
//!
//! > 00 = DRAM Disabled: All accesses are directed to DMI.
//! > 01 = Read Only: All reads are sent to DRAM. All writes are forwarded to DMI.
//! > 10 = Write Only: All writes are sent to DRAM. Reads are serviced by DMI.
//! > 11 = Normal DRAM Operation: All reads and writes are serviced by DRAM.
//!
//! That is bit-for-bit the 440FX's encoding at a different address, so the
//! mechanism [`crate::dev::pc::pmc`] documents at length is not re-argued here:
//! a window is one mapping of this device's own DRAM at priority 1 over
//! whatever the board decodes underneath, three [`Perms`] values plus **no
//! mapping at all** for *Disabled*. `pmc`'s module docs carry the argument for
//! why *Disabled* must be an absent mapping rather than a
//! [`Perms::NONE`] one, and it is as true here.
//!
//! The 256 KiB of DRAM under `0xc0000-0xfffff` belongs to this device for the
//! same reason it belongs to the 440FX's: the (G)MCH *is* the memory
//! controller, and a machine file cannot hand one device a reference to
//! another's store.
//!
//! ## PCIEXBAR, at 60h
//!
//! §5.1.16, a 64-bit register, default `00000000E0000000h`:
//!
//! ```text
//!  35:28  PCI Express Base Address (PCIEXBAR) [...] BIOS will program this
//!         register resulting in a base address for a contiguous memory
//!         address space; size is defined by bits 2:1 of this register.
//!
//!         PCI Express Base Address + Bus Number * 1 MB + Device Number *
//!         32 KB + Function Number * 4 KB
//!
//!     27  128 MB Base Address Mask (128ADMSK): This bit is either part of the
//!         PCI Express Base Address (R/W) or part of the Address Mask (RO,
//!         read 0b), depending on the value of bits 2:1 in this register.
//!     26  64 MB Base Address Mask (64ADMSK): [...]
//!    2:1  Length (LENGTH) [...] 00 = 256 MB (buses 0-255) [...] 01 = 128 MB
//!         (Buses 0-127) [...] 10 = 64 MB (Buses 0-63) [...] 11 = Reserved
//!      0  PCIEXBAR Enable (PCIEXBAREN)
//! ```
//!
//! So the writable address bits depend on `LENGTH`, and bits 27 and 26 read
//! back as zero whenever `LENGTH` has claimed them for the mask. That is
//! modelled literally: the latch keeps what was written to bits 35:26 and to
//! bits 2:0, and every *read* and every *use* of the value masks bits 27:26
//! against the length in force.
//!
//! # Moving the ECAM window from inside a configuration write
//!
//! Exactly the problem [`crate::bus::pci::bar`] solves for a BAR, and the same
//! answer: [`AddressSpace::try_topology`], which is order-exempt by
//! construction, plus a `stale` flag re-applied at the next configuration
//! access. It differs from a BAR in one respect worth naming and it is a
//! *simplification*: a `PCIEXBAR` write arriving through ECAM travels through
//! the **memory** space and so does the window it moves — the try-lock is
//! against a space the access is already inside, and it fails. Arriving through
//! `0xcfc` it travels through I/O space and the try succeeds. The stale flag
//! covers the first case, and the retry lands on the very next configuration
//! access of any kind, of which there is always at least one more: firmware
//! writes `PCIEXBAR` and then reads something back through it.
//!
//! Moving the window out from under the access that is moving it would be a
//! guest-visible fault either way, so the deferral is not a compromise: it is
//! what the hardware does too, in the sense that the cycle in flight completes
//! against the old decode.
//!
//! **But the retry needs somewhere to land, and on this board it cannot be the
//! next configuration access.** A q35 firmware reaches configuration space
//! through ECAM, so every one of its configuration accesses travels through the
//! memory space and every retry fails for the same reason the first attempt
//! did. A 440FX never met this, because it has only one route to configuration
//! space and that route is in the other space. So this bridge takes a **clock
//! domain** and asks the scheduler for the next tick while — and only while —
//! something is owed; `Device::advance_to` runs from the run loop with no
//! access in flight, which is the moment a topology guard is actually
//! available. The comment on those methods below carries the argument.
//!
//! # What is not modelled
//!
//! Everything in Table 5-1 that a boot does not read. `MCHBAR`, `PXPEPBAR`,
//! `DMIBAR`, `GGC`, `DEVEN`, the DRAM timing and rank registers, `REMAPBASE`,
//! `TOM`, `TOUUD`, `TOLUD`, the graphics stolen-memory registers, and
//! **SMRAM/ESMRAMC/TSEGMB** — the last of which is a real gap rather than an
//! omission, because SMM is how a q35's firmware protects itself and an SMM
//! guest will find `SMRAM` reading as its reset value and nothing happening.
//! They read as zero, which is what a reserved register reads as (Rev 2.1
//! §6.1), and a firmware that writes one finds it did nothing — honest, because
//! on this board it *would* do nothing.
//!
//! # Sources
//!
//! *Intel 3 Series Express Chipset Family Datasheet*, order number 316966-002:
//! Table 5-1 for the register map, §5.1.1-§5.1.2 for the identification,
//! §5.1.16 for `PCIEXBAR`, §5.1.18-§5.1.24 for `PAM0`-`PAM6`. *PCI Local Bus
//! Specification* Rev 2.1 §6.1 and §6.2 for the Type 00h header.
//!
//! No emulator source was consulted (`CLAUDE.md`, provenance).
//!
//! [`AddressSpace::try_topology`]: crate::core::space::AddressSpace::try_topology

use alloc::boxed::Box;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::bus::pci::{
    Bdf, CONFIG_PORT_WINDOW_LEN, ConfigPorts, ConfigSpace, PciBus, PciFunction, buses, config,
};
use crate::core::device::{Device, DeviceClass, ExportId, PropertySpec, RealizeCtx, ResetKind};
use crate::core::sched::LazyHandle;
use crate::core::space::{
    AddressSpace, Mapping, MappingId, MemAttrs, MemOps, Perms, RamStore, Region, RegionRef,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::{Error, Result};
use crate::machine::realize::{BindCtx, Instance};
use crate::machine::validate::ClassSchema;

use super::ecam::{self, Ecam};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "q35.mch";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// The 82Q35 GMCH's device identification (§5.1.2).
pub const DEVICE_ID_82Q35: u16 = 0x29b0;

/// The 82G33/82P35 (G)MCH's, which Table 5-1 prints as the map's default.
pub const DEVICE_ID_82G33: u16 = 0x29c0;

/// Where the shadowable region starts: the low edge of `PAM1`'s low segment.
pub const SHADOW_BASE: u64 = 0x000c_0000;

/// How much of it there is: `0xc0000`-`0xfffff`, 256 KiB.
pub const SHADOW_LEN: u64 = 0x0004_0000;

/// The priority the shadow DRAM is mapped at, one above the board's own decode.
///
/// The same number [`crate::dev::pc::pmc`] uses, for the same reason: a PAM
/// window sits over the ROM socket the machine file put there and falls through
/// to it wherever [`Perms`] says the bridge does not claim the cycle.
const SHADOW_PRIORITY: i32 = 1;

/// The priority the ECAM window is mapped at.
///
/// Above the shadow, because the two never overlap on any board a firmware
/// would build — ECAM lives above the top of DRAM and the shadow is under
/// 1 MiB — so this is a tie-break rule rather than a behaviour.
const ECAM_PRIORITY: i32 = 2;

/// Configuration offset of `PCIEXBAR` (§5.1.16).
const PCIEXBAR: u16 = 0x60;

/// How many bytes it occupies: 60h-67h.
const PCIEXBAR_LEN: u16 = 8;

/// `PCIEXBAR[0]`: the window decodes.
const PCIEXBAR_ENABLE: u64 = 1 << 0;

/// `PCIEXBAR[2:1]`: which of the three window sizes.
const PCIEXBAR_LENGTH: u64 = 0b110;

/// Every bit of `PCIEXBAR` a guest write can reach: 35:26 and 2:0.
///
/// Bits 27:26 are latched here and masked on the way out, because whether they
/// are address or mask depends on `LENGTH`, which the same write may be
/// changing (§5.1.16).
const PCIEXBAR_WRITABLE: u64 = 0x0000_000f_fc00_0000 | PCIEXBAR_ENABLE | PCIEXBAR_LENGTH;

/// Configuration offset of `PAM0` (§5.1.18).
const PAM0: u16 = 0x90;

/// How many PAM registers there are: `PAM0`-`PAM6` at 90h-96h.
const PAM_COUNT: u16 = 7;

/// One shadowable window: where it is, how big, and which nibble of which PAM
/// register controls it.
struct Window {
    /// Guest-physical base.
    base: u64,
    /// Length in bytes.
    len: u64,
    /// Configuration offset of the PAM register that governs it.
    reg: u16,
    /// How far to shift that register right to bring the nibble to bit 0.
    shift: u32,
}

/// A window's table entry, spelled once.
const fn seg(base: u64, len: u64, reg: u16, shift: u32) -> Window {
    Window {
        base,
        len,
        reg,
        shift,
    }
}

/// The thirteen segments §5.1.18-§5.1.24 name, in address order.
const WINDOWS: [Window; 13] = [
    seg(0xc_0000, 0x4000, PAM0 + 1, 0),
    seg(0xc_4000, 0x4000, PAM0 + 1, 4),
    seg(0xc_8000, 0x4000, PAM0 + 2, 0),
    seg(0xc_c000, 0x4000, PAM0 + 2, 4),
    seg(0xd_0000, 0x4000, PAM0 + 3, 0),
    seg(0xd_4000, 0x4000, PAM0 + 3, 4),
    seg(0xd_8000, 0x4000, PAM0 + 4, 0),
    seg(0xd_c000, 0x4000, PAM0 + 4, 4),
    seg(0xe_0000, 0x4000, PAM0 + 5, 0),
    seg(0xe_4000, 0x4000, PAM0 + 5, 4),
    seg(0xe_8000, 0x4000, PAM0 + 6, 0),
    seg(0xe_c000, 0x4000, PAM0 + 6, 4),
    // `PAM0`'s high nibble: the 64 KiB system BIOS area, and the only segment
    // that is not 16 KiB. `PAM0`'s low nibble is reserved (§5.1.18).
    seg(0xf_0000, 0x1_0000, PAM0, 4),
];

/// How many windows there are, as a `usize` for array sizing.
const N: usize = WINDOWS.len();

/// The nibble's read-enable bit: encoding `01` is *Read Only* (§5.1.18).
const RE: u8 = 0x1;
/// The nibble's write-enable bit: encoding `10` is *Write Only*.
const WE: u8 = 0x2;

/// Turn one attribute nibble into the terms its mapping answers on.
fn perms_of(nibble: u8) -> Perms {
    let mut p = Perms::NONE;
    if nibble & RE != 0 {
        // `EXEC` rides with `READ`: firmware executes out of the window it
        // shadowed, and the bit is carried rather than enforced.
        p = p.union(Perms::READ).union(Perms::EXEC);
    }
    if nibble & WE != 0 {
        p = p.union(Perms::WRITE);
    }
    p
}

/// Where this bridge's windows went.
#[derive(Debug, Clone)]
struct Mapped {
    space: Arc<AddressSpace>,
    /// One slot per PAM segment; `None` while that segment is *Disabled*.
    pam: Vec<Option<MappingId>>,
    /// The ECAM window, while `PCIEXBAR` says it decodes.
    ecam: Option<MappingId>,
}

/// The bridge's registers, the DRAM they switch into view, and the ECAM window.
///
/// Separate from [`Mch`] because a [`PciFunction`] has to be reachable as an
/// `Arc<dyn PciFunction>` and `Device::realize` only ever has `&self`.
struct Registers {
    /// The 256 bytes of configuration space, minus `PCIEXBAR`, which needs
    /// bit-level masking that [`ConfigSpace`]'s per-byte mask cannot express.
    /// At [`LockRank::DEVICE`], released before anything outward.
    config: Mutex<ConfigSpace>,
    /// `PCIEXBAR`'s latch, holding every bit a guest write can reach.
    /// [`LockRank::LEAF`]: read, copied out, released.
    pciexbar: Mutex<u64>,
    /// The main memory that lives under `0xc0000-0xfffff`.
    dram: Arc<RamStore>,
    /// One alias per PAM segment, kept so the mappings can be made at bind.
    windows: Vec<RegionRef>,
    /// One region per `LENGTH` encoding, all onto the same [`Ecam`]: a region
    /// has one length, and `PCIEXBAR` chooses between three.
    ecam: [RegionRef; 3],
    /// Where the windows went. `None` until [`Instance::bind`].
    mapped: Mutex<Option<Mapped>>,
    /// Set when a retopology could not happen when it was asked for. Derived
    /// state: never serialized, and a load re-applies unconditionally.
    ///
    /// An atomic rather than a [`Mutex`], because
    /// [`Device::next_event_tick`](crate::core::device::Device::next_event_tick)
    /// reads it and may not take a lock.
    stale: AtomicBool,
    /// The tick of this bridge's own clock domain it has been advanced to.
    ///
    /// The bridge does not *count* anything. This exists so that a stale
    /// retopology has a moment with no access in flight to happen in — see
    /// [`Device::advance_to`](crate::core::device::Device::advance_to) below.
    tick: AtomicU64,
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
        match self.pciexbar.try_lock() {
            Some(v) => s.field("pciexbar", &*v),
            None => s.field("pciexbar", &"<in use>"),
        };
        s.field("mapped", &self.mapped.try_lock().map(|m| m.is_some()))
            .field("stale", &self.stale.load(Ordering::Relaxed))
            .finish()
    }
}

/// Where the ECAM window sits and how big it is, or `None` if it does not
/// decode.
///
/// Free function rather than a method because it is pure arithmetic on the
/// latch and the tests want to check it against §5.1.16 directly.
#[must_use]
pub fn ecam_window(latch: u64) -> Option<(u64, u64)> {
    if latch & PCIEXBAR_ENABLE == 0 {
        return None;
    }
    let len = ecam::window_len(((latch & PCIEXBAR_LENGTH) >> 1) as u8)?;
    // The address bits below the window size are part of the mask and read as
    // zero, which is also what makes the base naturally aligned.
    Some((latch & !(len - 1) & 0x0000_000f_ffff_ffff, len))
}

impl Registers {
    /// The header this part hardwires, from Table 5-1 and §5.1.1-§5.1.10.
    fn fresh_config(device_id: u16, revision: u8) -> ConfigSpace {
        let mut c = ConfigSpace::new();
        c.hardwire(config::VENDOR_ID, u32::from(config::VENDOR_INTEL), 2);
        c.hardwire(config::DEVICE_ID, u32::from(device_id), 2);
        // Table 5-1: PCICMD default 0006h — memory space and bus master, with
        // I/O space clear, because a host bridge does not answer I/O cycles.
        c.hardwire(config::COMMAND, 0x0006, 2);
        // Table 5-1: PCISTS default 0090h — DEVSEL# timing and the capability
        // list bit. No capability is modelled, so the pointer stays zero and
        // the bit says only what the datasheet's default says.
        c.hardwire(config::STATUS, 0x0090, 2);
        c.hardwire(config::REVISION_ID, u32::from(revision), 1);
        // Table 5-1: CC 060000h — bridge, host bridge, no programming interface.
        c.hardwire(config::CLASS_CODE, 0x00, 1);
        c.hardwire(
            config::CLASS_CODE + 1,
            u32::from(config::SUBCLASS_HOST_BRIDGE),
            1,
        );
        c.hardwire(config::CLASS_CODE + 2, u32::from(config::CLASS_BRIDGE), 1);
        // Table 5-1: HDR 00h. Not multi-function: this device answers only
        // function 0. A real q35 has D0:F1 and D0:F2 as well, and neither is
        // modelled, so claiming to be multi-function would be a lie a firmware
        // could catch by probing.
        c.hardwire(config::HEADER_TYPE, 0x00, 1);

        // What the guest may move. `PCIEXBAR` is deliberately absent: it is
        // masked by hand in `config_write`.
        c.allow(config::COMMAND, 2);
        c.allow(config::LATENCY_TIMER, 1);
        c.allow(config::BIST, 1);
        c.allow(PAM0, PAM_COUNT);
        c
    }

    /// The value a read of `PCIEXBAR` sees.
    ///
    /// Bits 27:26 come back as zero whenever `LENGTH` has claimed them for the
    /// address mask (§5.1.16).
    fn pciexbar_value(&self) -> u64 {
        let raw = *self.pciexbar.lock();
        let keep = match (raw & PCIEXBAR_LENGTH) >> 1 {
            // 256 MB: bits 27:26 are mask, and read as zero.
            0 => !0x0c00_0000u64,
            // 128 MB: bit 27 is address, bit 26 is mask.
            1 => !0x0400_0000u64,
            // 64 MB: both are address. (`11` is reserved and lands here too;
            // the window is refused by `ecam_window`, so the bits are inert.)
            _ => !0u64,
        };
        raw & keep
    }

    /// The terms each PAM segment should answer on, given the registers now.
    fn perms(&self) -> Vec<Perms> {
        let c = self.config.lock();
        WINDOWS
            .iter()
            .map(|w| perms_of((c.byte(w.reg) >> w.shift) & 0xf))
            .collect()
    }

    /// Bring every mapping into line with the registers. Reports whether it
    /// could be done.
    ///
    /// `blocking` picks the guard: the write guard where nothing is in flight —
    /// bind, reset, a snapshot load — and the order-exempt try-lock from inside
    /// a configuration write, where the blocking one would invert the ladder.
    fn retopo(&self, perms: &[Perms], blocking: bool) -> bool {
        // Cloned out and the lock released: nothing of this device's is held
        // while the space is retopologised (`CLAUDE.md`, re-entrancy).
        let Some(mapped) = self.mapped.lock().clone() else {
            // Not bound yet. Not stale either: `bind` maps with the right terms
            // in the first place.
            return true;
        };
        let want_ecam = ecam_window(self.pciexbar_value());
        let guard = if blocking {
            Some(mapped.space.topology())
        } else {
            mapped.space.try_topology()
        };
        let Some(mut topo) = guard else {
            self.stale.store(true, Ordering::Relaxed);
            return false;
        };
        let mut pam = Vec::with_capacity(N);
        for ((id, p), w) in mapped.pam.iter().zip(perms).zip(&WINDOWS) {
            let want = *p != Perms::NONE;
            pam.push(match (*id, want) {
                (Some(id), true) => {
                    // The only error is "not a mapping of this space", which
                    // cannot happen: the id came from this guard's own space.
                    let _ = topo.reprotect(id, *p);
                    Some(id)
                }
                (None, true) => topo
                    .map_with(
                        Mapping::new(Arc::clone(&self.windows[pam.len()]), w.base)
                            .with_priority(SHADOW_PRIORITY)
                            .with_perms(*p),
                    )
                    .ok(),
                // Disabled: the cycle belongs to the expansion bus, and an
                // absent mapping is what says so — see `pmc`'s module docs.
                (Some(id), false) => {
                    let _ = topo.unmap(id);
                    None
                }
                (None, false) => None,
            });
        }
        // The ECAM window. Always unmapped before it is placed, because
        // `LENGTH` can change the region as well as the base and a remap
        // cannot change a mapping's length.
        if let Some(id) = mapped.ecam {
            let _ = topo.unmap(id);
        }
        let ecam = want_ecam.and_then(|(base, len)| {
            let region = self
                .ecam
                .iter()
                .find(|r| r.len() == len)
                .expect("every length `ecam_window` returns has a region");
            topo.map_with(
                Mapping::new(Arc::clone(region), base)
                    .with_priority(ECAM_PRIORITY)
                    .with_perms(Perms::RW),
            )
            .ok()
        });
        drop(topo);
        *self.mapped.lock() = Some(Mapped {
            space: Arc::clone(&mapped.space),
            pam,
            ecam,
        });
        self.stale.store(false, Ordering::Relaxed);
        true
    }

    /// Bring the mappings into line with the registers.
    fn sync(&self, blocking: bool) -> bool {
        let perms = self.perms();
        self.retopo(&perms, blocking)
    }

    /// Claim the space and place whatever the registers currently ask for.
    ///
    /// **Retopology**, and legal here: `bind` runs during machine assembly with
    /// no access in flight.
    fn install(&self, space: &Arc<AddressSpace>) {
        *self.mapped.lock() = Some(Mapped {
            space: Arc::clone(space),
            pam: alloc::vec![None; N],
            ecam: None,
        });
        self.sync(true);
    }
}

impl PciFunction for Registers {
    fn config_read(&self, offset: u16, dst: &mut [u8], _attrs: MemAttrs) {
        // No `debug` branch: a configuration read of this bridge has no side
        // effects — no status bit a read clears, no FIFO to pop — so a
        // debugger's window may poll it freely.
        self.config.lock().read(offset, dst);
        // `PCIEXBAR` on top, because its mask is per bit rather than per byte.
        let bar = self.pciexbar_value().to_le_bytes();
        for (i, slot) in dst.iter_mut().enumerate() {
            let at = offset.saturating_add(i as u16);
            if (PCIEXBAR..PCIEXBAR + PCIEXBAR_LEN).contains(&at) {
                *slot = bar[usize::from(at - PCIEXBAR)];
            }
        }
        // A retopology that could not happen when it was asked for gets its
        // next chance here. Deliberately after the read, and deliberately not
        // gated on `attrs.debug`: re-applying what the guest already wrote
        // changes nothing a debugger could observe, and a memory map that
        // disagrees with its own registers is worse than either.
        if self.stale.load(Ordering::Relaxed) {
            self.sync(false);
        }
    }

    fn config_write(&self, offset: u16, src: &[u8], attrs: MemAttrs) {
        if attrs.debug {
            // A debug write here could move a PAM window or the whole ECAM
            // aperture under the guest's feet, which is exactly what
            // `MemAttrs::debug` forbids. `ConfigPorts` and `Ecam` both refuse
            // one before it reaches here; this is the second lock on the door,
            // for a caller that reaches the function directly.
            return;
        }
        let touches = |first: u16, len: u16| {
            offset < first + len && offset.saturating_add(src.len() as u16) > first
        };
        let mut changed = false;
        if touches(PCIEXBAR, PCIEXBAR_LEN) {
            let mut latch = self.pciexbar.lock();
            let before = *latch;
            let mut bytes = latch.to_le_bytes();
            for (i, byte) in src.iter().enumerate() {
                let at = offset.saturating_add(i as u16);
                if (PCIEXBAR..PCIEXBAR + PCIEXBAR_LEN).contains(&at) {
                    bytes[usize::from(at - PCIEXBAR)] = *byte;
                }
            }
            *latch = u64::from_le_bytes(bytes) & PCIEXBAR_WRITABLE;
            changed |= *latch != before;
        }
        let perms = {
            let mut c = self.config.lock();
            changed |= c.write(offset, src) && touches(PAM0, PAM_COUNT);
            WINDOWS
                .iter()
                .map(|w| perms_of((c.byte(w.reg) >> w.shift) & 0xf))
                .collect::<Vec<_>>()
        };
        // Only when something that moves a mapping actually moved. Firmware
        // rewrites the same values constantly while it sizes, and re-flattening
        // an address space for a write that changed nothing is pure cost.
        if changed || self.stale.load(Ordering::Relaxed) {
            self.retopo(&perms, false);
        }
    }
}

/// The 82Q35 (G)MCH: a host bridge with PAM and ECAM.
#[derive(Debug)]
pub struct Mch {
    regs: Arc<Registers>,
    /// The `0xcf8`/`0xcfc` window, which owns the `CONFADD` latch.
    ///
    /// Held by the **device** rather than by [`Registers`], and that is not a
    /// filing decision — it is what keeps the object graph acyclic.
    /// [`ConfigPorts`] holds its fabric strongly, the fabric holds every
    /// function on it strongly, and `Registers` *is* a function on this fabric;
    /// putting the ports inside it would close the loop
    /// `fabric → registers → ports → fabric`, which is a leak that nothing ever
    /// collects. `Mch` is owned by the machine and is not on the bus, so the
    /// same handle here is a plain edge. LeakSanitizer found this through the
    /// `q35_chipset` fuzz target; [`super::ecam::Ecam`]'s own weak handle is
    /// the other half of the same problem.
    ports: Arc<ConfigPorts>,
    bus: Arc<PciBus>,
    at: Bdf,
    config_region: RegionRef,
    device_id: u16,
    revision: u8,
    /// The value `PCIEXBAR` comes out of reset holding. See [`CLASS`]'s
    /// property summary for why a board may want to set one.
    reset_pciexbar: u64,
    /// The sibling whose own decode lives inside `CONFADD`'s four bytes, if the
    /// machine file named one. Resolved at [`Instance::bind`].
    passthrough: Mutex<Option<String>>,
}

impl Mch {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a property this class does not know, a device
    /// number outside 0-31, or an identification or revision outside its
    /// width; [`Error::Config`] if the `bus` name is already open as something
    /// else, or if `ecam` names a base that is not aligned to a window.
    pub fn new(props: &crate::core::props::Props) -> Result<Mch> {
        let mut r = props.reader();
        let bus_name = r.or_str("bus", "pci0")?.to_string();
        let device = r.or_range("device", 0u64, 0..=u64::from(crate::bus::pci::MAX_DEVICE))?;
        let device_id = r.or_range("device-id", u64::from(DEVICE_ID_82Q35), 0..=0xffff)?;
        let revision = r.or_range("revision", 0u64, 0..=255)?;
        let ecam_base = r.or_size("ecam", 0)?;
        let passthrough = r
            .optional_link("passthrough")?
            .map(|l| String::from(l.as_str()));
        r.finish()?;
        let reset_pciexbar = if ecam_base == 0 {
            // §5.1.16's own default: the address is 0E0000000h and the enable
            // bit is clear, so nothing decodes until firmware sets it.
            0xe000_0000
        } else {
            if ecam_base & (ecam::WINDOW_LENGTHS[0] - 1) != 0 {
                return Err(Error::Config {
                    at: CLASS_NAME.to_string(),
                    message: alloc::format!(
                        "`ecam` is a 256 MiB window and PCIEXBAR's address field starts at bit \
                         28, so {ecam_base:#x} cannot be its base (datasheet 316966-002 §5.1.16)"
                    ),
                });
            }
            (ecam_base & 0x0000_000f_f000_0000) | PCIEXBAR_ENABLE
        };
        let bus = buses::attach(props, &bus_name)?;
        // §5.1: the DRAM controller is Device 0, Function 0. The property
        // exists so a machine file can say so out loud.
        let at = Bdf::new(0, device as u8, 0)?;
        let mch = Mch::with_bus(bus, at, device_id as u16, revision as u8, reset_pciexbar)?;
        *mch.passthrough.lock() = passthrough;
        Ok(mch)
    }

    /// The same device, built from a fabric handle a test already has.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if a PAM window does not fit its own store, which
    /// would be a bug in this module's table rather than anything a caller can
    /// cause.
    pub fn with_bus(
        bus: Arc<PciBus>,
        at: Bdf,
        device_id: u16,
        revision: u8,
        reset_pciexbar: u64,
    ) -> Result<Mch> {
        let dram = Arc::new(RamStore::new(SHADOW_LEN));
        let whole: RegionRef = Arc::new(Region::ram("q35.mch.dram", Arc::clone(&dram)));
        let mut windows = Vec::with_capacity(N);
        for w in &WINDOWS {
            windows.push(Arc::new(Region::alias(
                alloc::format!("q35.mch.shadow.{:05x}", w.base),
                Arc::clone(&whole),
                w.base - SHADOW_BASE,
                w.len,
            )?) as RegionRef);
        }
        let ports = Arc::new(ConfigPorts::new(Arc::clone(&bus)));
        let config_region: RegionRef = Arc::new(Region::io(
            "q35.mch.config",
            CONFIG_PORT_WINDOW_LEN,
            Arc::clone(&ports) as Arc<dyn MemOps>,
        ));
        // One `Ecam` behind three regions of the three lengths `PCIEXBAR` can
        // select: a region has one length, and the register chooses.
        let ecam_ops = Arc::new(Ecam::new(&bus));
        let ecam = ecam::WINDOW_LENGTHS.map(|len| {
            Arc::new(Region::io(
                ECAM_REGION,
                len,
                Arc::clone(&ecam_ops) as Arc<dyn MemOps>,
            )) as RegionRef
        });
        Ok(Mch {
            regs: Arc::new(Registers {
                config: Mutex::with_rank(
                    LockRank::DEVICE,
                    Registers::fresh_config(device_id, revision),
                ),
                pciexbar: Mutex::with_rank(LockRank::LEAF, reset_pciexbar),
                dram,
                windows,
                ecam,
                mapped: Mutex::with_rank(LockRank::LEAF, None),
                stale: AtomicBool::new(false),
                tick: AtomicU64::new(0),
            }),
            ports,
            bus,
            at,
            config_region,
            device_id,
            revision,
            reset_pciexbar,
            passthrough: Mutex::with_rank(LockRank::LEAF, None),
        })
    }

    /// The DRAM this bridge switches into `0xc0000-0xfffff`.
    #[must_use]
    pub fn dram(&self) -> &Arc<RamStore> {
        &self.regs.dram
    }

    /// Where this bridge sits on its fabric.
    #[must_use]
    pub fn address(&self) -> Bdf {
        self.at
    }

    /// The current value of one PAM register, `0`-`6`, or `None` past `PAM6`.
    #[must_use]
    pub fn pam(&self, index: u16) -> Option<u8> {
        (index < PAM_COUNT).then(|| self.regs.config.lock().byte(PAM0 + index))
    }

    /// `PCIEXBAR` as a read of it would see it.
    #[must_use]
    pub fn pciexbar(&self) -> u64 {
        self.regs.pciexbar_value()
    }

    /// Where the ECAM window decodes and how big it is, or `None`.
    #[must_use]
    pub fn ecam(&self) -> Option<(u64, u64)> {
        ecam_window(self.pciexbar())
    }

    /// Map this bridge's windows into `space`. **Retopology.**
    ///
    /// What [`Instance::bind`] does, reachable directly so a unit test can
    /// assemble a bridge without a machine.
    pub fn attach_space(&self, space: &Arc<AddressSpace>) {
        self.regs.install(space);
    }
}

/// The name every ECAM region carries, and the name [`super::acpi`] looks it up
/// by when it writes the `MCFG` table.
///
/// A region name is the seam here: the generator reads the base and the length
/// out of the address space that was actually built rather than being told
/// them, and this constant is what makes the two ends agree without a second
/// copy of either number.
pub const ECAM_REGION: &str = "q35.mch.ecam";

/// The `q35.mch` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "an Intel 82Q35 (G)MCH host bridge: PAM shadowing and the ECAM configuration window",
    properties: &[
        PropertySpec {
            name: "bus",
            kind: crate::core::props::ValueKind::Str,
            required: false,
            summary: "the PCI fabric this bridge is the root of (default `pci0`)",
        },
        PropertySpec {
            name: "device",
            kind: crate::core::props::ValueKind::Uint,
            required: false,
            summary: "the device number it answers at on bus 0 (default 0, which is the part's own)",
        },
        PropertySpec {
            name: "device-id",
            kind: crate::core::props::ValueKind::Uint,
            required: false,
            summary: "the device identification (default 0x29b0, the 82Q35 GMCH — datasheet §5.1.2)",
        },
        PropertySpec {
            name: "revision",
            kind: crate::core::props::ValueKind::Uint,
            required: false,
            summary: "the revision identification byte (default 0)",
        },
        PropertySpec {
            name: "ecam",
            kind: crate::core::props::ValueKind::Size,
            required: false,
            summary: "where PCIEXBAR comes out of reset pointing, enabled — a stand-in for the \
                      firmware initialisation this board does not have; 0 is the datasheet's own \
                      default, which is disabled",
        },
        PropertySpec {
            name: "passthrough",
            kind: crate::core::props::ValueKind::Link,
            required: false,
            summary: "the sibling whose own decode lives inside CONFADD's four bytes at 0xcf8",
        },
    ],
    construct: |props| Ok(Box::new(Mch::new(props)?)),
};

impl Device for Mch {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // The one outward action: announcing itself onto the fabric.
        self.bus
            .attach(self.at, Arc::clone(&self.regs) as Arc<dyn PciFunction>)
    }

    fn reset(&self, kind: ResetKind) {
        *self.regs.config.lock() = Registers::fresh_config(self.device_id, self.revision);
        *self.regs.pciexbar.lock() = self.reset_pciexbar;
        self.ports.reset();
        if kind == ResetKind::Cold {
            // Power clears memory; a reset line does not.
            let _ = self.regs.dram.fill(0, SHADOW_LEN, 0);
        }
        // Blocking, and correct: a reset runs from the machine's own loop with
        // no access in flight.
        self.regs.sync(true);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            // `""` is the configuration port pair, because it is the only
            // region a machine file maps: the ECAM window is placed by
            // `PCIEXBAR` and the PAM windows by the PAM registers.
            "" | "config" => Some(Arc::clone(&self.config_region)),
            _ => None,
        }
    }

    // -----------------------------------------------------------------------
    // A moment with no access in flight
    // -----------------------------------------------------------------------
    //
    // This bridge counts nothing, and it is a lazily advanced device anyway.
    // The reason is the one problem the try-lock cannot solve on its own.
    //
    // A configuration write moves a window in the memory space. Through
    // `0xcfc` that write travels through the *I/O* space and the try-lock on
    // the memory space succeeds. Through **ECAM** it travels through the memory
    // space itself, the try-lock fails, and the retry at the next
    // configuration access fails for exactly the same reason — for ever, if the
    // firmware only ever uses ECAM. Which a q35 firmware does.
    //
    // `pmc` and `bar` never met this, because a 440FX has no second route to
    // configuration space; their retry always eventually arrives through the
    // other space. So the stale flag needs somewhere to land that is not an
    // access at all, and `core::device`'s answer to "act outward once the
    // handler has returned" is the scheduler.
    //
    // `next_event_tick` therefore asks for the very next tick of this bridge's
    // clock domain **only while something is owed**, and returns `None`
    // otherwise, so an idle bridge costs the scheduler nothing. `advance_to` is
    // called with no lock held, from the run loop, which is precisely the
    // moment a topology guard is available. It still uses the try-lock: the
    // same handle can be reached from inside an access by a sibling's catch-up,
    // and a blocking guard there would invert the ladder.
    //
    // The cost is one clock domain on a host bridge, which is not a fiction —
    // the (G)MCH is the front-side bus's own controller, and
    // `machines/q35.machine` gives it the same `bus` oscillator the local
    // APIC's timer counts.

    fn is_lazy(&self) -> bool {
        true
    }

    fn current_tick(&self) -> u64 {
        self.regs.tick.load(Ordering::Relaxed)
    }

    fn advance_to(&self, tick: u64) {
        self.regs.tick.store(tick, Ordering::Relaxed);
        if self.regs.stale.load(Ordering::Relaxed) {
            self.regs.sync(false);
        }
    }

    fn next_event_tick(&self) -> Option<u64> {
        // Strictly greater than `current_tick`, or catch-up makes no progress.
        self.regs
            .stale
            .load(Ordering::Relaxed)
            .then(|| self.regs.tick.load(Ordering::Relaxed) + 1)
    }

    fn attach_lazy(&self, _handle: LazyHandle) {
        // Nothing to keep: this device never syncs itself from inside its own
        // access, because it has no counter an access could read stale.
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bytes(self.regs.config.lock().bytes())?;
        w.write_u64(*self.regs.pciexbar.lock())?;
        w.write_u32(self.ports.address())?;
        // The shadow DRAM is guest-visible state: firmware copies itself into
        // it and executes out of it.
        let len = usize::try_from(SHADOW_LEN)
            .map_err(|_| Error::State(String::from("shadow larger than this host")))?;
        let mut bytes = alloc::vec![0u8; len];
        self.regs.dram.read_at(0, &mut bytes)?;
        w.write_bytes(&bytes)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let config: &[u8] = r.read_bytes()?;
        let pciexbar = r.read_u64()?;
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
            *c = Registers::fresh_config(self.device_id, self.revision);
            c.restore(config);
        }
        // Masked exactly as a guest write is, so a corrupt or hand-written
        // snapshot cannot install bits the hardware could never hold.
        *self.regs.pciexbar.lock() = pciexbar & PCIEXBAR_WRITABLE;
        self.ports.set_address(address);
        self.regs.dram.write_at(0, dram)?;
        // The memory map is a function of the registers, so it is rebuilt
        // rather than saved (`CLAUDE.md`: derived state is never serialized).
        self.regs.sync(true);
        Ok(())
    }
}

impl Instance for Mch {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "a host bridge decides what is decoded in 0xc0000-0xfffff and where ECAM lands, \
                 so it needs the space it decides for: add `space = mem` to the object that \
                 declares it",
            ),
        })?;
        self.attach_space(space);
        // `CONFADD` is four bytes the north bridge claims only for a Dword
        // access, and the byte at 0xcf9 belongs to the south bridge.
        let wanted = self.passthrough.lock().clone();
        if let Some(path) = wanted {
            let handle = ctx
                .export_as::<crate::dev::pc::PortPassthrough>(&path, ExportId::PORT_PASSTHROUGH)?;
            self.ports.set_passthrough(Arc::clone(handle.ops()));
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Mch::new(props)?)))
}

/// What the validator should know about `q35.mch`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::core::props::ValueKind;
    use crate::machine::validate::PropSchema;
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("bus", ValueKind::Str))
        .prop(
            PropSchema::new("device", ValueKind::Uint)
                .range(0, u64::from(crate::bus::pci::MAX_DEVICE)),
        )
        .prop(PropSchema::new("device-id", ValueKind::Uint).range(0, 0xffff))
        .prop(PropSchema::new("revision", ValueKind::Uint).range(0, 255))
        .prop(PropSchema::new("ecam", ValueKind::Size))
        .prop(PropSchema::new("passthrough", ValueKind::Link))
        .region("")
        .region("config")
}
