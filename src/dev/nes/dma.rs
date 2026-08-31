//! The RP2A03's DMA unit — OAM DMA at `$4014` and the DMC's sample fetch.
//!
//! Source: the NESdev wiki, [DMA](https://www.nesdev.org/wiki/DMA),
//! [PPU registers](https://www.nesdev.org/wiki/PPU_registers) (`OAMDATA`) and
//! [APU DMC](https://www.nesdev.org/wiki/APU_DMC).
//!
//! # One unit, two customers
//!
//! There is exactly one cycle-stealing arbiter on the 2A03 die, and both the
//! sprite copy and the DMC's sample fetch go through it. They share a `/RDY`
//! line, a get/put cadence and a precedence rule, and every one of those is
//! guest-visible — so modelling them as two independent units would get the
//! overlap wrong in a way no amount of care inside either would fix. This is
//! that one unit.
//!
//! # What a transfer looks like from the bus
//!
//! The unit halts the CPU by pulling `/RDY` low. A 6502 can only be stopped on
//! a **read**: it finishes the read it is making — that read is the *halt
//! cycle* and really happens — and then re-drives the same address every cycle
//! until the line comes back up. On the cycles the unit is not itself driving
//! the bus those repeats are externally visible, which is how a DMA clocks a
//! controller port or bumps `$2007`'s address several times over. When `/RDY`
//! is released the CPU performs the read it was trying to make.
//!
//! Transfers happen on alternating **get** (read) and **put** (write) cycles,
//! which are the APU's two clock phases rather than the CPU's cycle parity as
//! such — `put-phase` says which is which, and it must match the APU's.
//!
//! ```text
//! OAM DMA     halt + [alignment] + 256 x (get read, put write)   513 or 514
//! DMC load    halt + dummy + [alignment] + get                   3 or 4
//! DMC reload  as above, but it starts trying on a put            3 or 4
//! ```
//!
//! A *load* fetch — one a `$4015` write scheduled — starts trying to halt on
//! the get cycle of the second APU cycle after that write, which is the third
//! or fourth CPU cycle after it. A *reload* — one the output unit scheduled by
//! emptying the sample buffer — starts trying on the next put. Either way, if
//! the CPU happens to be writing the halt fails and is retried the next cycle.
//!
//! # Bus conflicts
//!
//! The 2A03 decodes its own registers from **bits 4-0 of the DMA address and
//! bits 15-5 of the 6502 core's address**, and the core keeps its address while
//! halted. So a DMA get performed while the halted core sits on `$4000`-`$401F`
//! also activates the 2A03 register at `$4000 | (dma_addr & $1F)` — clearing
//! the frame interrupt flag, clocking a controller — and wire-ANDs that
//! register's value into the byte the DMA reads.

use alloc::boxed::Box;
use alloc::string::String;
use alloc::sync::{Arc, Weak};
use core::fmt;

use crate::core::device::{
    Arbitration, CycleGate, Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind,
};
use crate::core::error::{BusError, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region as MmioRegion, RegionRef,
    RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::dev::apu::{DMC_FETCH, DmaKind, DmcFetch};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description would use.
const CLASS_NAME: &str = "nes.oamdma";

/// The snapshot chunk version. Bump with the encoding, never on its own.
///
/// v2 carries the live transfer: v1 could not, because a transfer took no time
/// and so could never be caught half done.
const STATE_VERSION: u32 = 2;

/// The name of the region that decodes `$4014`.
pub const PORT: &str = "port";

/// How many bytes one transfer moves — the whole of OAM.
pub const OAM_LEN: u16 = 256;

/// Where the copied bytes are written: `OAMDATA`.
///
/// An address in the CPU's space, not an offset into the PPU: the unit really
/// does drive `$2004` on the bus, which is why the copy honours `OAMADDR` and
/// why a machine that maps the PPU somewhere else would need this to follow.
/// The NES maps the register block at `$2000` and no NES does otherwise, so it
/// is a constant rather than a property.
pub const OAM_DATA_ADDR: u64 = 0x2004;

/// The 2A03's own register block, which a DMA get can collide with.
const INTERNAL_BASE: u64 = 0x4000;

/// How much of an address the 2A03 decodes from the *core* rather than from the
/// DMA — see the [module docs](self).
const INTERNAL_MASK: u64 = 0xffe0;

/// CPU cycles one OAM transfer halts the core for when no alignment is needed.
///
/// One halt cycle plus 256 read/write pairs. A `$4014` write that lands on a
/// put costs one more, because the first get is then a cycle further away.
pub const TRANSFER_CYCLES: u64 = 1 + 2 * OAM_LEN as u64;

/// A sprite copy in progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Oam {
    /// The source page, as written to `$4014`.
    page: u8,
    /// How many bytes have been written to `$2004`.
    index: u16,
    /// The byte read on the last get, waiting for its put.
    latch: Option<u8>,
}

/// A DMC sample fetch the unit has taken up.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Job {
    serial: u64,
    addr: u16,
    /// The dummy cycle the wiki says always follows the halt.
    dummy: bool,
}

/// What one arbitrated cycle does to the bus, decided under the lock and
/// performed outside it (`CLAUDE.md`, re-entrancy).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Act {
    /// Release the core.
    Release,
    /// Hold it without driving the bus: its own read repeats, visibly.
    Hold,
    /// Read a sprite byte from this address.
    OamRead(u64),
    /// Write the latched sprite byte to `$2004`.
    OamWrite(u8),
    /// Fetch a DMC sample byte for the job with this serial.
    DmcRead(u16, u64),
}

/// What the device and its memory port both hold.
struct Shared {
    /// Everything mutable, at `DEVICE` rank. Never held across a bus access.
    state: Mutex<State>,
}

/// The unit's own state.
#[derive(Debug)]
struct State {
    /// The CPU's address space, as a bus master reaches it.
    ///
    /// `Weak` rather than `Arc`: this device is *inside* the space it reads
    /// through — the space owns the region that owns this — and a strong
    /// reference would be a cycle the machine could never drop.
    bus: Option<Weak<AddressSpace>>,
    /// The requester id accesses from this unit carry.
    requester: RequesterId,
    /// The DMC, whose fetch shares this arbiter.
    dmc: Option<Arc<DmcFetch>>,
    /// Which CPU cycles are puts: cycle `c` is a get iff `c - 1 + phase` is
    /// even. Must agree with the APU's own `put-phase`.
    phase: u64,
    /// The last page written to `$4014`.
    page: u8,
    /// A sprite copy, armed or running.
    oam: Option<Oam>,
    /// Whether the core is currently held.
    halted: bool,
    /// The DMC fetch being serviced.
    job: Option<Job>,
    /// Transfers completed, for diagnostics.
    transfers: u64,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let state = self.state.lock();
        f.debug_struct("Shared")
            .field("page", &state.page)
            .field("oam", &state.oam)
            .field("halted", &state.halted)
            .field("job", &state.job)
            .field("transfers", &state.transfers)
            .finish_non_exhaustive()
    }
}

/// Whether CPU cycle `cycle` is a get cycle.
///
/// The APU's own convention (`Core::on_put_cycle`): with `phase` zero the first
/// cycle after power-on is a get, and they alternate from there.
#[inline]
const fn is_get(cycle: u64, phase: u64) -> bool {
    (cycle.wrapping_sub(1).wrapping_add(phase)) & 1 == 0
}

/// The first cycle at or after `from` whose phase is the one asked for.
#[inline]
const fn phase_at_or_after(from: u64, phase: u64, get: bool) -> u64 {
    if is_get(from, phase) == get {
        from
    } else {
        from + 1
    }
}

/// A DMC fetch whose halt phase has been worked out.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Ready {
    serial: u64,
    addr: u16,
    /// The first cycle the unit may start trying to halt on.
    start: u64,
}

impl Shared {
    /// Arm a sprite copy. The bus is not touched: the transfer happens on the
    /// cycles the arbiter steals.
    fn arm_oam(&self, page: u8) {
        let mut state = self.state.lock();
        state.page = page;
        state.oam = Some(Oam {
            page,
            index: 0,
            latch: None,
        });
    }

    /// The DMC seam, cloned out so nothing is called with our lock held.
    fn dmc(&self) -> Option<Arc<DmcFetch>> {
        self.state.lock().dmc.clone()
    }

    /// The get/put phase this unit was configured with.
    fn phase(&self) -> u64 {
        self.state.lock().phase
    }

    /// The serial of the fetch being serviced, if any.
    fn job_serial(&self) -> Option<u64> {
        self.state.lock().job.map(|j| j.serial)
    }

    /// The bus and the attributes an access from this unit carries.
    fn master(&self) -> Option<(Arc<AddressSpace>, MemAttrs)> {
        let state = self.state.lock();
        let bus = state.bus.as_ref().and_then(Weak::upgrade)?;
        Some((bus, MemAttrs::DEFAULT.with_requester(state.requester)))
    }

    /// Decide what happens on the cycle after `cycle`.
    ///
    /// Split from performing it: the decision needs the lock and the access
    /// must not hold it.
    fn decide(&self, cycle: u64, write: bool, request: Option<Ready>, alive: Option<bool>) -> Act {
        let mut state = self.state.lock();
        let phase = state.phase;

        // A fetch withdrawn between the halt and its get is the wiki's aborted
        // DMA: drop it now, before it can spend its dummy cycle, so the core
        // loses the one halt cycle and nothing more.
        if alive == Some(false) {
            state.job = None;
        }

        let ready = request.filter(|r| cycle >= r.start);
        if !state.halted {
            if ready.is_none() && state.oam.is_none() {
                return Act::Release;
            }
            if write {
                // `/RDY` is only honoured on a read: the unit waits and tries
                // again next cycle.
                return Act::Release;
            }
            // The read that just happened was the halt cycle.
            state.halted = true;
            if let Some(r) = ready {
                state.job = Some(Job {
                    serial: r.serial,
                    addr: r.addr,
                    dummy: true,
                });
            }
        } else if state.job.is_none()
            && let Some(r) = ready
        {
            // A fetch scheduled while the core was already held for the sprite
            // copy joins that halt rather than starting its own.
            state.job = Some(Job {
                serial: r.serial,
                addr: r.addr,
                dummy: false,
            });
        }

        let next = cycle + 1;
        let get = is_get(next, phase);

        // The dummy cycle the wiki says always follows a DMC halt.
        if let Some(job) = &mut state.job
            && job.dummy
        {
            job.dummy = false;
            return Act::Hold;
        }

        // The DMC takes precedence over the sprite copy.
        if get && let Some(job) = state.job.take() {
            return Act::DmcRead(job.addr, job.serial);
        }

        if let Some(oam) = &mut state.oam {
            if let Some(byte) = oam.latch.take() {
                oam.index += 1;
                if oam.index == OAM_LEN {
                    state.oam = None;
                    state.transfers += 1;
                }
                return Act::OamWrite(byte);
            }
            if get {
                let addr = (u64::from(oam.page) << 8) | u64::from(oam.index);
                return Act::OamRead(addr);
            }
            // Waiting for a get: the alignment cycle.
            return Act::Hold;
        }

        if state.job.is_some() {
            // A fetch waiting for its get, on a put.
            return Act::Hold;
        }

        state.halted = false;
        Act::Release
    }

    /// Record the byte an [`Act::OamRead`] fetched.
    fn latch_oam(&self, byte: u8) {
        if let Some(oam) = &mut self.state.lock().oam {
            oam.latch = Some(byte);
        }
    }
}

/// The RP2A03's DMA unit.
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
    /// The object the machine file named as this unit's DMC, resolved at bind.
    dmc_link: Mutex<Option<String>>,
}

impl OamDma {
    /// Validate properties and allocate. Performs no outward action.
    ///
    /// # Errors
    ///
    /// [`crate::Error::Property`] for an unknown or ill-typed property.
    pub fn new(props: &Props) -> Result<OamDma> {
        let mut r = props.reader();
        let phase = r.or_range::<u64>("put-phase", 0, 0..=1)?;
        let dmc = r.optional_link("dmc")?.map(|l| String::from(l.as_str()));
        r.finish()?;
        let unit = OamDma::default();
        unit.shared.state.lock().phase = phase;
        *unit.dmc_link.lock() = dmc;
        Ok(unit)
    }

    /// Connect the CPU's address space, which this unit masters.
    ///
    /// The machine layer calls this from [`Instance::bind`]; a caller wiring a
    /// NES by hand calls it directly. Without it a `$4014` write arms a
    /// transfer whose accesses then fail, rather than one that silently copies
    /// nothing.
    pub fn attach_bus(&self, space: &Arc<AddressSpace>, requester: RequesterId) {
        let mut state = self.shared.state.lock();
        state.bus = Some(Arc::downgrade(space));
        state.requester = requester;
    }

    /// Connect the DMC whose sample fetch shares this arbiter.
    ///
    /// Optional: a board with no DMC — or a test that only cares about sprites
    /// — leaves it unset and the unit runs OAM DMA alone.
    pub fn attach_dmc(&self, dmc: Arc<DmcFetch>) {
        self.shared.state.lock().dmc = Some(dmc);
    }

    /// Which CPU cycles this unit treats as puts. Must match the APU's.
    pub fn set_put_phase(&self, phase: u64) {
        self.shared.state.lock().phase = phase & 1;
    }

    /// How many sprite copies have completed since power-on.
    #[must_use]
    pub fn transfers(&self) -> u64 {
        self.shared.state.lock().transfers
    }

    /// The last page written to `$4014`.
    #[must_use]
    pub fn page(&self) -> u8 {
        self.shared.state.lock().page
    }

    /// Whether the unit is holding the core off the bus.
    #[must_use]
    pub fn halted(&self) -> bool {
        self.shared.state.lock().halted
    }

    /// The `/RDY` arbiter, as the CPU takes it.
    #[must_use]
    pub fn gate(&self) -> Arc<dyn CycleGate> {
        Arc::new(Gate {
            shared: Arc::clone(&self.shared),
        })
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
                    dmc: None,
                    phase: 0,
                    page: 0,
                    oam: None,
                    halted: false,
                    job: None,
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
        OamDma {
            shared,
            port,
            dmc_link: Mutex::new(None),
        }
    }
}

/// The `/RDY` half of the unit, handed to the core.
///
/// A separate handle rather than `OamDma` itself, so the core holds only what
/// it needs.
#[derive(Debug)]
struct Gate {
    shared: Arc<Shared>,
}

impl Gate {
    /// Perform whatever [`Shared::decide`] chose, outside the state lock.
    fn perform(&self, act: Act, held: u64, dmc: Option<&Arc<DmcFetch>>) {
        let Some((bus, attrs)) = self.shared.master() else {
            return;
        };
        match act {
            Act::Release | Act::Hold => {}
            Act::OamRead(addr) => {
                let byte = self.read_with_conflict(&bus, attrs, addr, held);
                self.shared.latch_oam(byte);
            }
            Act::OamWrite(byte) => {
                let _ = bus.write(OAM_DATA_ADDR, Width::U8, u64::from(byte), attrs);
            }
            Act::DmcRead(addr, serial) => {
                let byte = self.read_with_conflict(&bus, attrs, u64::from(addr), held);
                if let Some(dmc) = dmc {
                    dmc.complete(serial, byte);
                }
            }
        }
    }

    /// One DMA read, including the 2A03's partial-decode bus conflict.
    ///
    /// The chip decodes its own `$4000`-`$401F` block from bits 4-0 of *this*
    /// address and bits 15-5 of the address the halted core is still driving.
    /// So a DMA get made while the core sits on that block reads two devices at
    /// once — with every side effect that implies — and the bus carries the
    /// wired-AND of what they drive (NESdev wiki, "DMA").
    fn read_with_conflict(&self, bus: &AddressSpace, attrs: MemAttrs, addr: u64, held: u64) -> u8 {
        let byte = bus.read(addr, Width::U8, attrs).unwrap_or(0) as u8;
        if held & INTERNAL_MASK != INTERNAL_BASE {
            return byte;
        }
        let alias = INTERNAL_BASE | (addr & 0x1f);
        if alias == addr {
            // The DMA is already reading that register: one responder, nothing
            // to conflict with.
            return byte;
        }
        let other = bus.read(alias, Width::U8, attrs).unwrap_or(0) as u8;
        byte & other
    }
}

impl CycleGate for Gate {
    fn arbitrate(&self, cycle: u64, held: u64, write: bool) -> Arbitration {
        let dmc = self.shared.dmc();
        // The DMC is on the same die and the same clock: its request appears on
        // an exact cycle, so it has to be caught up before it is asked.
        let (request, alive) = match &dmc {
            Some(dmc) => {
                dmc.sync();
                let phase = self.shared.phase();
                let request = dmc.request().map(|r| Ready {
                    serial: r.serial,
                    addr: r.addr,
                    // A load fetch starts trying on the get cycle of the second
                    // APU cycle after the write that scheduled it — the third
                    // or fourth CPU cycle. A reload starts on the next put.
                    start: match r.kind {
                        DmaKind::Load => phase_at_or_after(r.at + 3, phase, true),
                        DmaKind::Reload => phase_at_or_after(r.at + 1, phase, false),
                    },
                });
                let alive = self.shared.job_serial().map(|s| dmc.is_pending(s));
                (request, alive)
            }
            None => (None, None),
        };

        let act = self.shared.decide(cycle, write, request, alive);
        self.perform(act, held, dmc.as_ref());
        match act {
            Act::Release => Arbitration::Release,
            Act::Hold => Arbitration::Hold,
            _ => Arbitration::Steal,
        }
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
        self.shared.arm_oam(*value);
        Ok(())
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

    fn cycle_gate(&self) -> Option<Arc<dyn CycleGate>> {
        Some(self.gate())
    }

    fn reset(&self, _kind: ResetKind) {
        // /RES aborts a transfer in progress and clears the unit. The bus
        // handle, the DMC seam and the clock phase are wiring, not state.
        let mut state = self.shared.state.lock();
        state.page = 0;
        state.oam = None;
        state.halted = false;
        state.job = None;
        state.transfers = 0;
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.shared.state.lock();
        w.write_u8(state.page)?;
        w.write_u64(state.transfers)?;
        match state.oam {
            Some(oam) => {
                w.write_bool(true)?;
                w.write_u8(oam.page)?;
                w.write_u16(oam.index)?;
                w.write_bool(oam.latch.is_some())?;
                w.write_u8(oam.latch.unwrap_or(0))?;
            }
            None => {
                w.write_bool(false)?;
                w.write_u8(0)?;
                w.write_u16(0)?;
                w.write_bool(false)?;
                w.write_u8(0)?;
            }
        }
        w.write_bool(state.halted)?;
        match state.job {
            Some(job) => {
                w.write_bool(true)?;
                w.write_u64(job.serial)?;
                w.write_u16(job.addr)?;
                w.write_bool(job.dummy)
            }
            None => {
                w.write_bool(false)?;
                w.write_u64(0)?;
                w.write_u16(0)?;
                w.write_bool(false)
            }
        }
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let page = r.read_u8()?;
        let transfers = r.read_u64()?;
        let running = r.read_bool()?;
        let oam_page = r.read_u8()?;
        let index = r.read_u16()?;
        let latched = r.read_bool()?;
        let latch = r.read_u8()?;
        let halted = r.read_bool()?;
        let has_job = r.read_bool()?;
        let serial = r.read_u64()?;
        let addr = r.read_u16()?;
        let dummy = r.read_bool()?;

        let mut state = self.shared.state.lock();
        state.page = page;
        state.transfers = transfers;
        state.oam = running.then_some(Oam {
            page: oam_page,
            index,
            latch: latched.then_some(latch),
        });
        state.halted = halted;
        state.job = has_job.then_some(Job {
            serial,
            addr,
            dummy,
        });
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

/// The machine layer's half: the unit is a bus master, needs a space, and may
/// be linked to the DMC it shares its `/RDY` line with.
impl Instance for OamDma {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| crate::core::Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "the DMA unit masters the CPU bus: add `space = cpubus` to the object that \
                 declares it",
            ),
        })?;
        self.attach_bus(space, ctx.requester());

        let wanted = self.dmc_link.lock().clone();
        if let Some(name) = wanted {
            let peer = ctx.peer(&name).ok_or_else(|| crate::core::Error::Config {
                at: String::from(ctx.path()),
                message: alloc::format!("`dmc = {name}` names no object in this machine"),
            })?;
            let fetch = peer
                .interface(DMC_FETCH)
                .and_then(|any| any.downcast::<DmcFetch>().ok())
                .ok_or_else(|| crate::core::Error::Config {
                    at: String::from(ctx.path()),
                    message: alloc::format!(
                        "`dmc = {name}` names a device with no DMC to fetch for"
                    ),
                })?;
            self.attach_dmc(fetch);
        }
        Ok(())
    }
}

/// The properties [`OAM_DMA_CLASS`] accepts.
static OAM_DMA_PROPERTIES: &[PropertySpec] = &[
    PropertySpec {
        name: "put-phase",
        kind: ValueKind::Uint,
        required: false,
        summary: "which CPU cycles are puts (0 or 1); must match the APU's",
    },
    PropertySpec {
        name: "dmc",
        kind: ValueKind::Link,
        required: false,
        summary: "the APU whose DMC sample fetch shares this unit's /RDY line",
    },
];

/// The device class, as `nes.oamdma` in a machine description.
pub static OAM_DMA_CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "RP2A03 DMA unit: OAM DMA at $4014 and the DMC sample fetch, both halting the CPU",
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
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .region(PORT)
        .prop(PropSchema::new("put-phase", ValueKind::Uint).range(0, 1))
        .prop(PropSchema::new("dmc", ValueKind::Link))
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
        oam: Arc<Oam2004>,
    }

    /// Stands in for the PPU's `OAMDATA`: appends every byte written.
    #[derive(Debug, Default)]
    struct Oam2004 {
        written: Mutex<Vec<u8>>,
    }

    impl MemOps for Oam2004 {
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
        let oam = Arc::new(Oam2004::default());
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

    /// Run the arbiter the way a halted core would, from `from` onwards, and
    /// report what happened to each cycle.
    ///
    /// A `Hold` is the core's own read at `held` repeating on the bus.
    fn drive(dma: &OamDma, from: u64, held: u64) -> Vec<Arbitration> {
        let gate = dma.gate();
        let mut out = Vec::new();
        let mut cycle = from;
        loop {
            let a = gate.arbitrate(cycle, held, false);
            out.push(a);
            cycle += 1;
            if a == Arbitration::Release {
                return out;
            }
            assert!(out.len() < 1024, "the arbiter never released the core");
        }
    }

    #[test]
    fn a_write_copies_two_hundred_and_fifty_six_bytes_through_2004() {
        let b = bus();
        for i in 0..256u64 {
            wr(&b.space, 0x0200 + i, (i as u8) ^ 0x5a);
        }
        // The write only arms the copy; the cycles the arbiter steals perform it.
        wr(&b.space, 0x4014, 0x02);
        assert!(b.oam.written.lock().is_empty(), "nothing moves yet");
        drive(&b.dma, 1, 0x8000);

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
        wr(&b.space, 0x0700, 0xc3);
        wr(&b.space, 0x4014, 0x07);
        drive(&b.dma, 1, 0x8000);
        assert_eq!(b.oam.written.lock()[0], 0xc3);
    }

    #[test]
    fn a_copy_costs_513_cycles_from_a_get_and_514_from_a_put() {
        // With phase 0, odd cycles are gets. The `$4014` write on cycle `w` is
        // followed by the halt cycle on `w + 1`; the first get is `w + 2` when
        // `w` was a get, and `w + 3` — one alignment cycle later — when it was
        // a put.
        for (write_cycle, expected, holds) in [(1u64, 513usize, 0usize), (2, 514, 1)] {
            let b = bus();
            wr(&b.space, 0x4014, 0x02);
            let acts = drive(&b.dma, write_cycle + 1, 0x8000);
            // The halt cycle is the core's own read, which the core then has
            // to make again once the line comes back up — so the cycles it
            // loses are exactly the acts, closing `Release` included.
            assert_eq!(acts.len(), expected, "write on cycle {write_cycle}");
            assert_eq!(
                acts.iter().filter(|a| **a == Arbitration::Hold).count(),
                holds,
                "an alignment cycle only when the first get is a cycle further off"
            );
        }
    }

    #[test]
    fn a_write_cycle_cannot_be_halted() {
        let b = bus();
        wr(&b.space, 0x4014, 0x02);
        let gate = b.dma.gate();
        // Three write cycles in a row — an RMW then a push, say. The unit waits.
        for cycle in 1..=3 {
            assert_eq!(
                gate.arbitrate(cycle, 0x8000, true),
                Arbitration::Release,
                "cycle {cycle}"
            );
            assert!(!b.dma.halted());
        }
        // And takes the first read cycle it is offered.
        assert_ne!(gate.arbitrate(4, 0x8000, false), Arbitration::Release);
        assert!(b.dma.halted());
    }

    #[test]
    fn a_debug_write_moves_nothing() {
        let b = bus();
        b.space
            .write(0x4014, Width::U8, 0x02, MemAttrs::DEBUG)
            .expect("accepted");
        assert!(b.oam.written.lock().is_empty());
        assert_eq!(b.dma.transfers(), 0);
        assert_eq!(
            b.dma.gate().arbitrate(1, 0x8000, false),
            Arbitration::Release,
            "and arms nothing"
        );
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
    fn an_idle_unit_never_holds_the_core() {
        let b = bus();
        let gate = b.dma.gate();
        for cycle in 1..=8 {
            assert_eq!(gate.arbitrate(cycle, 0x8000, false), Arbitration::Release);
        }
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
        assert!(dma.shared.master().is_none());
    }

    #[test]
    fn state_round_trips() {
        let b = bus();
        wr(&b.space, 0x4014, 0x03);
        // Stopped half way through, which v1 of this chunk could not represent.
        let gate = b.dma.gate();
        for cycle in 1..=40 {
            gate.arbitrate(cycle, 0x8000, false);
        }

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
        assert_eq!(other.transfers(), b.dma.transfers());
        assert_eq!(other.halted(), b.dma.halted());
        // Copied out one at a time: two `DEVICE`-ranked locks at once is the
        // order violation `core::sync` exists to catch.
        let restored = other.shared.state.lock().oam;
        let original = b.dma.shared.state.lock().oam;
        assert_eq!(restored, original, "a half-finished copy survives");
    }

    #[test]
    fn a_reset_clears_the_unit() {
        let b = bus();
        wr(&b.space, 0x4014, 0x03);
        b.dma.reset(ResetKind::Cold);
        assert_eq!(b.dma.page(), 0);
        assert!(!b.dma.halted());
        // The bus handle is wiring, not state: it survives, so the next write
        // still copies.
        wr(&b.space, 0x4014, 0x00);
        drive(&b.dma, 1, 0x8000);
        assert_eq!(b.dma.transfers(), 1);
    }

    #[test]
    fn an_unknown_property_is_refused() {
        let e = OamDma::new(&Props::new().with("page", 3u64)).expect_err("no such property");
        assert!(alloc::format!("{e}").contains("page"), "{e}");
    }
}
