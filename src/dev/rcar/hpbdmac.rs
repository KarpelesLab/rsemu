//! The HPB-DMAC: R-Car's DMA controller for the peripherals behind the HPB
//! bridge — the SCIFs, the SD hosts, the audio blocks.
//!
//! Forty-four channels, each moving data between memory and a peripheral's
//! data register as that peripheral asks for it, with **two register sets**
//! per channel so a driver can queue the next buffer while the current one
//! runs.
//!
//! # Register map
//!
//! Channel `n`'s registers are at `0x40 × n` in the `channels` region:
//!
//! | Offset | Name | Here |
//! | --- | --- | --- |
//! | `0x00`, `0x04`, `0x08` | `DSAR0`, `DDAR0`, `DTCR0` | register set 0: source, destination, count of transfer units |
//! | `0x0c`, `0x10`, `0x14` | `DSAR1`, `DDAR1`, `DTCR1` | register set 1 |
//! | `0x18`, `0x1c` | `DSASR`, `DDASR` | the current source and destination (read-only) |
//! | `0x20` | `DTCSR` | units remaining in the current transfer (read-only) |
//! | `0x24` | `DPTR` | stored |
//! | `0x28` | `DCR` | bits 1:0: the transfer unit (byte, 16-bit, 32-bit); bit 5: the source address increments; bit 13: the destination does; bit 16: both register sets are in use |
//! | `0x2c` | `DCMDR` | a write of bit 0 or 1 queues the register set the driver just filled |
//! | `0x30` | `DSTPR` | a write of bit 0 stops the channel and drops what it had queued |
//! | `0x34` | `DSTSR` | bit 0: busy; bit 5: set 0 is the one to fill next |
//!
//! And in the `common` region:
//!
//! | Offset | Name | Here |
//! | --- | --- | --- |
//! | `0x0c`, `0x10` | `DINTSR0`, `DINTSR1` | transfer-end status, channels 0–31 and 32–43 |
//! | `0x14`, `0x18` | `DINTCR0`, `DINTCR1` | write 1 to clear |
//! | `0x1c`, `0x20` | `DINTMR0`, `DINTMR1` | a set bit enables the channel's interrupt |
//!
//! Everything else in either region is storage that reads back.
//!
//! # Interrupts
//!
//! The channels do not have GIC lines of their own. The SoC gathers them into
//! twelve **group** lines — `grp0`–`grp11`, GIC IDs 142–153 on the R-Car this
//! was traced on — and mirrors each channel's pending state into one of two
//! status words in the interrupt controller's block (the `status` region, at
//! `0x104` and `0x0f0`), which the kernel's demultiplexer reads to find the
//! channel. The mapping is fixed silicon wiring:
//!
//! | Group | Channels | Status bits |
//! | --- | --- | --- |
//! | 0 | 0–10 | `0x104` bits 0–10 |
//! | 1 | 11–13 | `0x104` bits 11–13 |
//! | 2 | 14–15 | `0x104` bits 14–15 |
//! | 3 | 16–19 | `0x104` bits 16–19 |
//! | 4 | 20 | `0x104` bit 20 |
//! | 5 | 21–23 | `0x104` bits 21–23 |
//! | 6 | 24 | `0x104` bit 24 |
//! | 7 | 25–27 | `0x104` bits 25–27 |
//! | 8 | 28–36, 42 | `0x0f0` bits 0–8, 11 |
//! | 9 | 39–41 | `0x104` bits 28–30 |
//! | 10 | 37–38 | `0x0f0` bits 9–10 |
//! | 11 | 43 | `0x0f0` bit 12 |
//!
//! A channel is pending when its `DINTSR` bit and its `DINTMR` enable are both
//! set; a group line is high while any of its channels is.
//!
//! # The two register sets
//!
//! With `DCR` bit 16 set, `DSTSR` bit 5 is the handshake. A driver reads it,
//! fills set 0 if it is set and set 1 if it is clear, then writes `DCMDR`;
//! the controller queues that set and flips the bit, so the next buffer goes
//! into the other one. A channel holds at most the one running and the one
//! queued behind it. With bit 16 clear only set 0 is used, and bit 5 stays
//! set.
//!
//! That bit is read off traces, not a manual: the SCIF channels run
//! double-buffered with `DCR` `0x5_0020`/`0x5_2000` and fill set 1 first; the
//! SD channels run with `0x2101` and fill set 0 every time.
//!
//! # Pacing
//!
//! Each channel has a request input, `dreq0`–`dreq43`, which a peripheral
//! drives (`rcar.scif`'s `tx-dreq`, `rcar.sdhi`'s `rx-dreq`, …). A channel
//! moves one byte at a time through the bus for as long as its request is
//! high and it has bytes left, so a sixteen-byte FIFO fills and the transfer
//! waits for it to drain, exactly as the silicon's does. A channel whose
//! request input nothing drives is always requesting: a memory-to-memory
//! copy.
//!
//! Time is zero within a burst, and a burst starts from the controller's own
//! register writes and from its scheduler slot — never from inside a request
//! wire's delivery, where the peripheral's own lowering of the request would
//! arrive only after the burst had run past it. A request that rises while
//! the CPU is elsewhere is therefore served at the controller's next slot.
//!
//! # Sources
//!
//! A black-box trace of the Alphard navi kernel's SCIF driver programming
//! it, and the register usage visible in that kernel's DMA helpers
//! (`docs/platforms/alphard-navi.md`). The R-Car hardware manual's HPB-DMAC
//! chapter was not available; every behaviour above is one the traced driver
//! depends on, and anything it does not use is stored and inert.

use alloc::boxed::Box;
use alloc::collections::{BTreeMap, VecDeque};
use alloc::string::{String, ToString};
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{AtomicBool, LockRank, Mutex, Ordering};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSink, WireSource};
use crate::machine::realize::{BindCtx, Instance};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "rcar.hpbdmac";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How many channels.
pub const CHANNELS: usize = 44;

/// The per-channel register block's stride.
const STRIDE: u64 = 0x40;

/// The `channels` region's length.
pub const CHANNELS_LEN: u64 = STRIDE * CHANNELS as u64;

/// The `common` region's length.
pub const COMMON_LEN: u64 = 0x170;

/// The `status` region's length: the interrupt controller's mirror block.
pub const STATUS_LEN: u64 = 0x1000;

/// How many group interrupt lines.
pub const GROUPS: usize = 12;

/// Which group line each channel raises.
const GROUP_OF: [u8; CHANNELS] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-10
    1, 1, 1, // 11-13
    2, 2, // 14-15
    3, 3, 3, 3, // 16-19
    4, // 20
    5, 5, 5, // 21-23
    6, // 24
    7, 7, 7, // 25-27
    8, 8, 8, 8, 8, 8, 8, 8, 8, // 28-36
    10, 10, // 37-38
    9, 9, 9,  // 39-41
    8,  // 42
    11, // 43
];

/// Where each channel's pending bit is mirrored: (status word offset, bit).
const MIRROR_OF: [(u16, u8); CHANNELS] = {
    let mut m = [(0x104u16, 0u8); CHANNELS];
    let mut n = 0;
    while n < 28 {
        m[n] = (0x104, n as u8);
        n += 1;
    }
    let mut k = 0;
    while k < 9 {
        m[28 + k] = (0x0f0, k as u8);
        k += 1;
    }
    m[37] = (0x0f0, 9);
    m[38] = (0x0f0, 10);
    m[39] = (0x104, 28);
    m[40] = (0x104, 29);
    m[41] = (0x104, 30);
    m[42] = (0x0f0, 11);
    m[43] = (0x0f0, 12);
    m
};

const DSASR: u64 = 0x18;
const DDASR: u64 = 0x1c;
const DTCSR: u64 = 0x20;
const DCR: u64 = 0x28;
const DCMDR: u64 = 0x2c;
const DSTPR: u64 = 0x30;
const DSTSR: u64 = 0x34;

/// The transfer unit. Traced, not documented to us: the SCIF channels run
/// with 0 and count bytes; the SD channels run with 1 and count an 8-byte SCR
/// as 4 — so 16-bit units, with the count in units.
const DCR_UNIT: u32 = 0b11;
const DCR_SAR_INC: u32 = 1 << 5;
const DCR_DOUBLE: u32 = 1 << 16;
const DCR_DAR_INC: u32 = 1 << 13;

const DSTSR_BUSY: u32 = 1 << 0;
const DSTSR_SET0_NEXT: u32 = 1 << 5;

const DINTSR0: u64 = 0x0c;
const DINTSR1: u64 = 0x10;
const DINTCR0: u64 = 0x14;
const DINTCR1: u64 = 0x18;
const DINTMR0: u64 = 0x1c;
const DINTMR1: u64 = 0x20;

/// One queued or running transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Xfer {
    sar: u32,
    dar: u32,
    left: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
struct Channel {
    /// `DSAR0`/`DDAR0`/`DTCR0` and `DSAR1`/`DDAR1`/`DTCR1`.
    sets: [[u32; 3]; 2],
    dcr: u32,
    /// `DSTSR` bit 5.
    set0_next: bool,
    active: Option<Xfer>,
    queued: VecDeque<Xfer>,
    /// `DTCSR` once the channel is idle: what the last transfer left.
    last_left: u32,
    /// `DSASR`/`DDASR` once idle.
    last_addr: (u32, u32),
    /// Registers stored without interpretation, by offset in the block.
    other: BTreeMap<u8, u32>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    ch: Vec<Channel>,
    status: u64,
    mask: u64,
    common: BTreeMap<u16, u32>,
}

impl State {
    fn new() -> State {
        State {
            ch: (0..CHANNELS).map(|_| Channel::default()).collect(),
            status: 0,
            mask: 0,
            common: BTreeMap::new(),
        }
    }
}

struct Shared {
    state: Mutex<State>,
    /// Each channel's request level, outside the state lock: a peripheral
    /// drives it from inside an access this controller made.
    dreq: Vec<AtomicBool>,
    /// Which request inputs something is wired to.
    wired: Vec<AtomicBool>,
    /// Set while a burst is running, so a request that rises during one is
    /// picked up by that burst rather than starting a nested one.
    busy: AtomicBool,
    bus: Mutex<Option<Weak<AddressSpace>>>,
    requester: Mutex<RequesterId>,
    irqs: Mutex<Vec<Option<WireSource>>>,
    /// The levels last driven on the interrupt outputs.
    driven: Mutex<u64>,
}

impl fmt::Debug for Shared {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Shared").finish_non_exhaustive()
    }
}

impl Shared {
    fn requesting(&self, n: usize) -> bool {
        !self.wired[n].load(Ordering::Acquire) || self.dreq[n].load(Ordering::Acquire)
    }

    /// Move bytes on every channel that can, until none can. Re-entrant
    /// calls (a peripheral raising its request from inside one of this
    /// loop's own accesses) return at once; the loop re-checks every channel
    /// after every byte, so it sees the change.
    fn run(&self) {
        if self.busy.swap(true, Ordering::AcqRel) {
            return;
        }
        let bus = self.bus.lock().as_ref().and_then(Weak::upgrade);
        let attrs = MemAttrs::DEFAULT.with_requester(*self.requester.lock());
        loop {
            let mut moved = false;
            for n in 0..CHANNELS {
                // One byte per channel per pass keeps a busy channel from
                // starving the others.
                let step = {
                    let mut s = self.state.lock();
                    let c = &mut s.ch[n];
                    match c.active {
                        Some(x) if x.left == 0 => {
                            // An empty transfer ends as soon as it starts.
                            c.last_left = 0;
                            c.last_addr = (x.sar, x.dar);
                            c.active = c.queued.pop_front();
                            s.status |= 1 << n;
                            moved = true;
                            None
                        }
                        Some(x) if self.requesting(n) => Some((x, c.dcr)),
                        _ => None,
                    }
                };
                let Some((x, dcr)) = step else {
                    continue;
                };
                let (width, step) = match dcr & DCR_UNIT {
                    0 => (Width::U8, 1),
                    1 => (Width::U16, 2),
                    _ => (Width::U32, 4),
                };
                let ok = match &bus {
                    Some(space) => space
                        .read(u64::from(x.sar), width, attrs)
                        .and_then(|v| space.write(u64::from(x.dar), width, v, attrs))
                        .is_ok(),
                    None => false,
                };
                let mut s = self.state.lock();
                let c = &mut s.ch[n];
                let Some(a) = c.active.as_mut() else {
                    continue;
                };
                if !ok {
                    // A bus error ends the transfer where it stood; the
                    // driver finds out from DTCSR.
                    c.last_left = a.left;
                    c.last_addr = (a.sar, a.dar);
                    c.active = c.queued.pop_front();
                    continue;
                }
                if dcr & DCR_SAR_INC != 0 {
                    a.sar = a.sar.wrapping_add(step);
                }
                if dcr & DCR_DAR_INC != 0 {
                    a.dar = a.dar.wrapping_add(step);
                }
                a.left -= 1;
                moved = true;
                if a.left == 0 {
                    c.last_left = 0;
                    c.last_addr = (a.sar, a.dar);
                    c.active = c.queued.pop_front();
                    s.status |= 1 << n;
                }
            }
            if !moved {
                break;
            }
        }
        self.busy.store(false, Ordering::Release);
        self.drive();
        // A request that rose after the last pass looked but before the flag
        // dropped would otherwise wait for the next event.
        let pending = (0..CHANNELS).any(|n| {
            self.requesting(n) && self.state.lock().ch[n].active.is_some_and(|x| x.left > 0)
        });
        if pending && bus.is_some() {
            self.run();
        }
    }

    /// Drive every interrupt output whose level changed: the per-channel
    /// lines and the group lines.
    fn drive(&self) {
        let pending = {
            let s = self.state.lock();
            s.status & s.mask
        };
        let groups = (0..CHANNELS)
            .filter(|n| pending & (1 << n) != 0)
            .fold(0u64, |g, n| g | 1 << GROUP_OF[n]);
        let levels = pending | groups << CHANNELS;
        let changed = {
            let mut last = self.driven.lock();
            let changed = *last ^ levels;
            *last = levels;
            changed
        };
        if changed == 0 {
            return;
        }
        let outs = self.irqs.lock().clone();
        // Outputs 0..CHANNELS are the channels, CHANNELS.. the groups.
        for (n, out) in outs.iter().enumerate() {
            if changed & (1 << n) != 0
                && let Some(out) = out
            {
                out.set(Level::from_bool(levels & (1 << n) != 0));
            }
        }
    }
}

/// The per-channel registers, as a region.
struct ChannelRegs(Arc<Shared>);
/// The common registers, as a region.
struct CommonRegs(Arc<Shared>);

impl fmt::Debug for ChannelRegs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("ChannelRegs")
    }
}

impl fmt::Debug for CommonRegs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("CommonRegs")
    }
}

fn word(src: &[u8]) -> Option<u32> {
    (src.len() == 4).then(|| u32::from_le_bytes([src[0], src[1], src[2], src[3]]))
}

impl MemOps for ChannelRegs {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        let n = (offset / STRIDE) as usize;
        let reg = offset % STRIDE;
        let s = self.0.state.lock();
        let c = &s.ch[n];
        let v = match reg {
            0x00..=0x14 => c.sets[(reg / 0x0c) as usize][((reg % 0x0c) / 4) as usize],
            DSASR => c.active.map_or(c.last_addr.0, |x| x.sar),
            DDASR => c.active.map_or(c.last_addr.1, |x| x.dar),
            DTCSR => c.active.map_or(c.last_left, |x| x.left),
            DCR => c.dcr,
            DSTSR => {
                (u32::from(c.active.is_some()) * DSTSR_BUSY)
                    | (u32::from(c.set0_next) * DSTSR_SET0_NEXT)
            }
            _ => c.other.get(&(reg as u8)).copied().unwrap_or(0),
        };
        // Reads have no side effects, so a debugger's is the guest's.
        dst.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        let v = word(src).ok_or(BusError::BadAccess)?;
        let n = (offset / STRIDE) as usize;
        let reg = offset % STRIDE;
        let kick = {
            let mut s = self.0.state.lock();
            let c = &mut s.ch[n];
            match reg {
                0x00..=0x14 => {
                    c.sets[(reg / 0x0c) as usize][((reg % 0x0c) / 4) as usize] = v;
                    false
                }
                DSASR | DDASR | DTCSR | DSTSR => false,
                DCR => {
                    c.dcr = v;
                    false
                }
                DCMDR if v & 3 != 0 => {
                    // Double-buffered, the sets alternate; otherwise set 0
                    // is the only one.
                    let set = if c.dcr & DCR_DOUBLE == 0 {
                        c.set0_next = true;
                        0
                    } else {
                        let set = if c.set0_next { 0 } else { 1 };
                        c.set0_next = !c.set0_next;
                        set
                    };
                    let [sar, dar, left] = c.sets[set];
                    let x = Xfer { sar, dar, left };
                    if c.active.is_none() {
                        c.active = Some(x);
                    } else {
                        c.queued.push_back(x);
                        while c.queued.len() > 1 {
                            c.queued.pop_front();
                        }
                    }
                    true
                }
                DSTPR if v & 1 != 0 => {
                    if let Some(x) = c.active.take() {
                        c.last_left = x.left;
                        c.last_addr = (x.sar, x.dar);
                    }
                    c.queued.clear();
                    false
                }
                _ => {
                    c.other.insert(reg as u8, v);
                    false
                }
            }
        };
        if kick {
            self.0.run();
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

impl MemOps for CommonRegs {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        let s = self.0.state.lock();
        let v = match offset {
            DINTSR0 => s.status as u32,
            DINTSR1 => (s.status >> 32) as u32,
            DINTMR0 => s.mask as u32,
            DINTMR1 => (s.mask >> 32) as u32,
            DINTCR0 | DINTCR1 => 0,
            _ => s.common.get(&(offset as u16)).copied().unwrap_or(0),
        };
        dst.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        let v = word(src).ok_or(BusError::BadAccess)?;
        {
            let mut s = self.0.state.lock();
            let lo = u64::from(v);
            let hi = u64::from(v) << 32;
            match offset {
                DINTCR0 => s.status &= !lo,
                DINTCR1 => s.status &= !hi,
                DINTMR0 => s.mask = (s.mask & !0xffff_ffff) | lo,
                DINTMR1 => s.mask = (s.mask & 0xffff_ffff) | hi,
                DINTSR0 | DINTSR1 => {}
                _ => {
                    s.common.insert(offset as u16, v);
                }
            }
        }
        self.0.drive();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

/// The interrupt controller's mirror of the channels' pending bits.
struct StatusRegs(Arc<Shared>);

impl fmt::Debug for StatusRegs {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("StatusRegs")
    }
}

impl MemOps for StatusRegs {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 {
            return Err(BusError::BadAccess);
        }
        let s = self.0.state.lock();
        let pending = s.status & s.mask;
        let mut v = 0u32;
        for (n, (word, bit)) in MIRROR_OF.iter().enumerate() {
            if u64::from(*word) == offset && pending & (1 << n) != 0 {
                v |= 1 << bit;
            }
        }
        if v == 0 && !matches!(offset, 0x104 | 0x0f0) {
            v = s
                .common
                .get(&(0x8000 | offset as u16))
                .copied()
                .unwrap_or(0);
        }
        dst.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if attrs.debug {
            return Err(BusError::BadAccess);
        }
        let v = word(src).ok_or(BusError::BadAccess)?;
        // The mirror words are read-only; the rest of the block is the
        // interrupt controller's configuration, stored and inert.
        if !matches!(offset, 0x104 | 0x0f0) {
            self.0.state.lock().common.insert(0x8000 | offset as u16, v);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

/// One channel's request input.
struct DreqPin(Arc<Shared>);

impl fmt::Debug for DreqPin {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("DreqPin")
    }
}

impl WireSink for DreqPin {
    fn set_level(&self, _src: WireId, line: u32, level: Level) {
        // Record, never act. This runs inside the request wire's delivery,
        // and a burst started here would make the very accesses that lower
        // the request while that wire is still delivering the rise -- so the
        // fall would queue behind the burst and the channel would read an
        // empty FIFO to the end of its count. The burst runs from the
        // controller's own scheduler slot and register writes instead, where
        // a peripheral's request changes arrive as they happen.
        let n = line as usize;
        if n < CHANNELS {
            self.0.dreq[n].store(level.is_high(), Ordering::Release);
        }
    }
}

/// The HPB-DMAC.
#[derive(Debug)]
pub struct HpbDmac {
    shared: Arc<Shared>,
    channels: RegionRef,
    common: RegionRef,
    status: RegionRef,
    pins: Mutex<Vec<Arc<DreqPin>>>,
}

impl HpbDmac {
    /// Build one. It takes no properties.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property was given.
    pub fn new(props: &Props) -> Result<HpbDmac> {
        props.reader().finish()?;
        Ok(HpbDmac::create())
    }

    /// Build one with nothing bound.
    #[must_use]
    pub fn create() -> HpbDmac {
        let shared = Arc::new(Shared {
            state: Mutex::with_rank(LockRank::DEVICE, State::new()),
            dreq: (0..CHANNELS).map(|_| AtomicBool::new(false)).collect(),
            wired: (0..CHANNELS).map(|_| AtomicBool::new(false)).collect(),
            busy: AtomicBool::new(false),
            bus: Mutex::with_rank(LockRank::LEAF, None),
            requester: Mutex::with_rank(LockRank::LEAF, RequesterId::ANONYMOUS),
            irqs: Mutex::with_rank(
                LockRank::LEAF,
                (0..CHANNELS + GROUPS).map(|_| None).collect(),
            ),
            driven: Mutex::with_rank(LockRank::LEAF, 0),
        });
        let channels: RegionRef = Arc::new(Region::io(
            "rcar.hpbdmac.channels",
            CHANNELS_LEN,
            Arc::new(ChannelRegs(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        let common: RegionRef = Arc::new(Region::io(
            "rcar.hpbdmac.common",
            COMMON_LEN,
            Arc::new(CommonRegs(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        let status: RegionRef = Arc::new(Region::io(
            "rcar.hpbdmac.status",
            STATUS_LEN,
            Arc::new(StatusRegs(Arc::clone(&shared))) as Arc<dyn MemOps>,
        ));
        HpbDmac {
            shared,
            channels,
            common,
            status,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// Give it the space it moves data in (what `bind` does from a machine
    /// file).
    pub fn attach_space(&self, space: &Arc<AddressSpace>) {
        *self.shared.bus.lock() = Some(Arc::downgrade(space));
    }
}

fn pin_number(port: &str, prefix: &str) -> Option<usize> {
    let n: usize = port.strip_prefix(prefix)?.parse().ok()?;
    (n < CHANNELS).then_some(n)
}

/// The `rcar.hpbdmac` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "R-Car HPB-DMAC: 44 peripheral DMA channels with double-buffered register sets",
    properties: &[],
    construct: |props| Ok(Box::new(HpbDmac::new(props)?)),
};

impl Device for HpbDmac {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.shared.state.lock() = State::new();
        self.shared.drive();
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        match name {
            "" | "channels" => Some(Arc::clone(&self.channels)),
            "common" => Some(Arc::clone(&self.common)),
            "status" => Some(Arc::clone(&self.status)),
            _ => None,
        }
    }

    fn sink(&self, port: &str, _sources: &[WireId]) -> Option<SinkPin> {
        let n = pin_number(port, "dreq")?;
        self.shared.wired[n].store(true, Ordering::Release);
        let pin = Arc::new(DreqPin(Arc::clone(&self.shared)));
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: n as u32,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let group = port
            .strip_prefix("grp")
            .and_then(|n| n.parse::<usize>().ok())
            .filter(|n| *n < GROUPS);
        let Some(n) = pin_number(port, "irq").or(group.map(|g| CHANNELS + g)) else {
            return Err(Error::Config {
                at: port.to_string(),
                message: String::from(
                    "the HPB-DMAC drives `irq0`..`irq43`, one per channel, and the group lines \
                     `grp0`..`grp11`",
                ),
            });
        };
        self.shared.irqs.lock()[n] = Some(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        *self.shared.driven.lock() = !0 >> 8;
        self.shared.drive();
    }

    fn is_runnable(&self) -> bool {
        true
    }

    fn run(&self, budget: crate::core::sched::Budget) -> crate::core::sched::Consumed {
        self.shared.run();
        crate::core::sched::Consumed::new(budget.ticks)
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.shared.state.lock();
        w.write_u64(s.status)?;
        w.write_u64(s.mask)?;
        w.write_seq_len(s.common.len() as u64)?;
        for (k, v) in &s.common {
            w.write_u16(*k)?;
            w.write_u32(*v)?;
        }
        let xfer = |w: &mut ChunkWriter<'_>, x: &Xfer| -> Result<()> {
            w.write_u32(x.sar)?;
            w.write_u32(x.dar)?;
            w.write_u32(x.left)
        };
        for c in &s.ch {
            for set in &c.sets {
                for v in set {
                    w.write_u32(*v)?;
                }
            }
            w.write_u32(c.dcr)?;
            w.write_bool(c.set0_next)?;
            w.write_bool(c.active.is_some())?;
            if let Some(x) = &c.active {
                xfer(w, x)?;
            }
            w.write_seq_len(c.queued.len() as u64)?;
            for x in &c.queued {
                xfer(w, x)?;
            }
            w.write_u32(c.last_left)?;
            w.write_u32(c.last_addr.0)?;
            w.write_u32(c.last_addr.1)?;
            w.write_seq_len(c.other.len() as u64)?;
            for (k, v) in &c.other {
                w.write_u8(*k)?;
                w.write_u32(*v)?;
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut s = State::new();
        s.status = r.read_u64()?;
        s.mask = r.read_u64()?;
        for _ in 0..r.read_seq_len(6)? {
            let k = r.read_u16()?;
            s.common.insert(k, r.read_u32()?);
        }
        let xfer = |r: &mut ChunkReader<'_>| -> Result<Xfer> {
            Ok(Xfer {
                sar: r.read_u32()?,
                dar: r.read_u32()?,
                left: r.read_u32()?,
            })
        };
        for c in &mut s.ch {
            for set in &mut c.sets {
                for v in set.iter_mut() {
                    *v = r.read_u32()?;
                }
            }
            c.dcr = r.read_u32()?;
            c.set0_next = r.read_bool()?;
            if r.read_bool()? {
                c.active = Some(xfer(r)?);
            }
            for _ in 0..r.read_seq_len(12)? {
                c.queued.push_back(xfer(r)?);
            }
            c.last_left = r.read_u32()?;
            c.last_addr = (r.read_u32()?, r.read_u32()?);
            for _ in 0..r.read_seq_len(5)? {
                let k = r.read_u8()?;
                c.other.insert(k, r.read_u32()?);
            }
        }
        *self.shared.state.lock() = s;
        self.shared.drive();
        Ok(())
    }
}

impl Instance for HpbDmac {
    fn bind(&self, ctx: &BindCtx<'_>) -> Result<()> {
        let space = ctx.space().ok_or_else(|| Error::Config {
            at: String::from(ctx.path()),
            message: String::from(
                "the HPB-DMAC is a bus master and needs the space it moves data in (`space = mem`)",
            ),
        })?;
        self.attach_space(space);
        *self.shared.requester.lock() = ctx.requester();
        Ok(())
    }
}

/// Add this class to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind this class into the machine graph.
///
/// # Errors
///
/// If the name is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(HpbDmac::new(props)?)))
}

/// The validator's view of this class.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir};
    let mut s = ClassSchema::new(CLASS_NAME)
        .region("")
        .region("channels")
        .region("common")
        .region("status");
    for g in 0..GROUPS {
        s = s.port(alloc::format!("grp{g}"), PortDir::Out);
    }
    for n in 0..CHANNELS {
        s = s
            .port(alloc::format!("dreq{n}"), PortDir::In)
            .port(alloc::format!("irq{n}"), PortDir::Out);
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::RamStore;

    fn board() -> (HpbDmac, Arc<AddressSpace>, Arc<RamStore>) {
        let ram = Arc::new(RamStore::new(0x1_0000));
        let space = Arc::new(AddressSpace::new("mem", 32));
        {
            let mut t = space.topology();
            t.map(Region::ram("ram", Arc::clone(&ram)), 0).unwrap();
        }
        let d = HpbDmac::create();
        d.attach_space(&space);
        (d, space, ram)
    }

    fn w(d: &HpbDmac, common: bool, off: u64, v: u32) {
        let region = if common { &d.common } else { &d.channels };
        let crate::core::space::RegionKind::Io(ops) = region.kind() else {
            unreachable!()
        };
        ops.write(off, &v.to_le_bytes(), MemAttrs::DEFAULT).unwrap();
    }

    fn r(d: &HpbDmac, common: bool, off: u64) -> u32 {
        let region = if common { &d.common } else { &d.channels };
        let crate::core::space::RegionKind::Io(ops) = region.kind() else {
            unreachable!()
        };
        let mut b = [0u8; 4];
        ops.read(off, &mut b, MemAttrs::DEFAULT).unwrap();
        u32::from_le_bytes(b)
    }

    #[test]
    fn an_unpaced_channel_copies_and_raises_its_status() {
        let (d, _space, ram) = board();
        ram.write_at(0x100, b"hello").unwrap();
        let ch = 6 * STRIDE;
        w(&d, false, ch + DCR, DCR_DOUBLE | DCR_SAR_INC | DCR_DAR_INC);
        assert_eq!(
            r(&d, false, ch + DSTSR) & DSTSR_SET0_NEXT,
            0,
            "fill set 1 first"
        );
        w(&d, false, ch + 0x0c, 0x100);
        w(&d, false, ch + 0x10, 0x200);
        w(&d, false, ch + 0x14, 5);
        w(&d, true, DINTMR0, 1 << 6);
        w(&d, false, ch + DCMDR, 1);
        let mut out = [0u8; 5];
        ram.read_at(0x200, &mut out).unwrap();
        assert_eq!(&out, b"hello");
        assert_eq!(r(&d, true, DINTSR0), 1 << 6);
        assert_eq!(r(&d, false, ch + DTCSR), 0);
        assert_ne!(
            r(&d, false, ch + DSTSR) & DSTSR_SET0_NEXT,
            0,
            "set 0 is next"
        );
        w(&d, true, DINTCR0, 1 << 6);
        assert_eq!(r(&d, true, DINTSR0), 0);
    }

    #[test]
    fn a_paced_channel_waits_for_its_request() {
        let (d, _space, ram) = board();
        ram.write_at(0x100, &[1, 2, 3]).unwrap();
        d.shared.wired[3].store(true, Ordering::Release);
        let ch = 3 * STRIDE;
        w(&d, false, ch + DCR, DCR_DOUBLE | DCR_SAR_INC);
        w(&d, false, ch + 0x0c, 0x100);
        w(&d, false, ch + 0x10, 0x300);
        w(&d, false, ch + 0x14, 3);
        w(&d, false, ch + DCMDR, 1);
        assert_eq!(
            r(&d, false, ch + DTCSR),
            3,
            "nothing moves without a request"
        );
        assert_ne!(r(&d, false, ch + DSTSR) & DSTSR_BUSY, 0);
        let pin = DreqPin(Arc::clone(&d.shared));
        pin.set_level(WireId(0), 3, Level::High);
        assert_eq!(
            r(&d, false, ch + DTCSR),
            3,
            "a rising request is served at the next slot"
        );
        d.shared.run();
        assert_eq!(r(&d, false, ch + DTCSR), 0);
        assert_eq!(
            ram.read_u8(0x300).unwrap(),
            3,
            "a fixed destination sees the last byte"
        );
    }

    #[test]
    fn a_finished_channel_shows_in_its_mirror_word() {
        let (d, _space, _ram) = board();
        let ch = 7 * STRIDE;
        w(&d, false, ch + DCR, DCR_DOUBLE);
        w(&d, false, ch + 0x14, 1);
        w(&d, true, DINTMR0, 1 << 7);
        w(&d, false, ch + DCMDR, 1);
        let crate::core::space::RegionKind::Io(ops) = d.status.kind() else {
            unreachable!()
        };
        let mut b = [0u8; 4];
        ops.read(0x104, &mut b, MemAttrs::DEFAULT).unwrap();
        assert_eq!(u32::from_le_bytes(b), 0x80, "channel 7 is bit 7 of 0x104");
        w(&d, true, DINTMR0, 0);
        ops.read(0x104, &mut b, MemAttrs::DEFAULT).unwrap();
        assert_eq!(u32::from_le_bytes(b), 0, "and only while enabled");
    }

    #[test]
    fn stopping_leaves_the_remainder_in_dtcsr() {
        let (d, _space, _ram) = board();
        d.shared.wired[0].store(true, Ordering::Release);
        w(&d, false, DCR, DCR_DOUBLE);
        w(&d, false, 0x14, 10);
        w(&d, false, DCMDR, 1);
        w(&d, false, DSTPR, 1);
        assert_eq!(r(&d, false, DTCSR), 10);
        assert_eq!(r(&d, false, DSTSR) & DSTSR_BUSY, 0);
    }

    #[test]
    fn a_single_buffered_channel_reuses_set_0_in_its_unit() {
        let (d, _space, ram) = board();
        ram.write_at(0x100, &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        let ch = 39 * STRIDE;
        w(&d, false, ch + DCR, DCR_DAR_INC | 1);
        for dst in [0x200, 0x300] {
            w(&d, false, ch, 0x100);
            w(&d, false, ch + 0x04, dst);
            w(&d, false, ch + 0x08, 2);
            w(&d, false, ch + DCMDR, 1);
            let mut out = [0u8; 4];
            ram.read_at(u64::from(dst), &mut out).unwrap();
            assert_eq!(out, [1, 2, 1, 2], "two 16-bit units from a fixed source");
            assert_ne!(r(&d, false, ch + DSTSR) & DSTSR_SET0_NEXT, 0);
        }
    }
}
