//! The ARM L2C-310 (PL310) level 2 cache controller: its register file, and
//! no cache.
//!
//! # Sources
//!
//! *CoreLink Level 2 Cache Controller L2C-310 Technical Reference Manual*
//! (ARM DDI 0246): chapter 3's register summary and descriptions — Cache ID
//! (§3.3.1), Cache Type (§3.3.2), Control (§3.3.3), Auxiliary Control
//! (§3.3.4), the Tag and Data RAM latency controls (§3.3.5), the event counter
//! and interrupt registers (§3.3.6-§3.3.7), the cache maintenance operations
//! (§3.3.10), the lockdown registers (§3.3.11), address filtering (§3.3.12),
//! debug, prefetch and power control (§3.3.13-§3.3.15) — and the Cache ID's
//! RTL release encoding of the revision history (appendix).
//!
//! No emulator source and no driver source of any licence was consulted.
//!
//! # Why there is no cache here
//!
//! Every guest access in this emulator already reaches memory that every
//! processor and every DMA master sees identically, so a model of the L2's
//! contents could only ever *hide* data it would then have to write back. What
//! a guest needs from an L2C-310 is the conversation: an ID to find it by, a
//! type register to size it from, an enable bit, and maintenance operations
//! that finish. So every register is stored and read back where the TRM makes
//! it read/write, reads as zero where it is write-only or reserved, and every
//! maintenance operation — by physical address, by index and way, by way, the
//! cache sync — **completes the instant it is written**. A driver that writes a
//! way mask to *Invalidate by Way* and then polls the register until the mask
//! has cleared reads zero on its first poll, which is exactly the answer the
//! silicon gives once the background operation has finished.
//!
//! # Configuration
//!
//! The way size and associativity are pins on the real part, latched into the
//! Auxiliary Control register at reset, and the Cache Type register reports
//! whatever Auxiliary Control holds. So the machine file states the cache as
//! `size` and `ways`, the reset value of Auxiliary Control is computed from
//! them (`aux` overrides it, and must agree), and Cache Type follows the live
//! register.
//!
//! # Which revision
//!
//! `revision = "r3p1"` by default, so Cache ID reads `0x410000c6`: r3p1 was the
//! current L2C-310 release through 2010-2011, which is when the SoCs this
//! class was written for (the Renesas R-Car H1, a 2011 part) were taped out.
//! Any release the TRM's appendix lists can be asked for instead.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::machine::realize::Instance;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "arm.l2c310";

/// The snapshot chunk version.
const STATE_VERSION: u32 = 1;

/// How much address space the register file occupies (DDI 0246 §3.2: 4 KiB).
pub const WINDOW_LEN: u64 = 0x1000;

/// Cache ID's fixed half: implementer `0x41` (Arm) in 31:24, part number
/// `0b0011` (L2C-310) in 9:6. The RTL release goes in 5:0.
const CACHE_ID_BASE: u32 = 0x4100_00c0;

/// The RTL release numbers Cache ID reports for each revision (DDI 0246,
/// revision history).
const REVISIONS: [(&str, u32); 7] = [
    ("r0p0", 0x0),
    ("r1p0", 0x2),
    ("r2p0", 0x4),
    ("r3p0", 0x5),
    ("r3p1", 0x6),
    ("r3p2", 0x8),
    ("r3p3", 0x9),
];

/// Cache Type's `ctype` field, 28:25: `0b11xy`, where x is lockdown by master
/// and y lockdown by line (DDI 0246 §3.3.2). This model implements both.
const CTYPE: u32 = 0b1111;

/// Auxiliary Control's associativity bit: set is 16-way.
const AUX_ASSOC_16: u32 = 1 << 16;
/// Auxiliary Control's way-size field, 19:17.
const AUX_WAY_SIZE_SHIFT: u32 = 17;
const AUX_WAY_SIZE: u32 = 0b111 << AUX_WAY_SIZE_SHIFT;
/// The bits Auxiliary Control implements: 0, 10-13, and 16-30 (DDI 0246
/// §3.3.4). The rest are reserved and read zero.
const AUX_MASK: u32 = 0x7fff_3c01;
/// Auxiliary Control's reset value before the way-size and associativity pins
/// are folded in: the cache replacement policy bit (25) resets to round-robin.
const AUX_RESET_BASE: u32 = 1 << 25;
/// The instruction and data prefetch enables, 29:28 — present in both
/// Auxiliary Control and Prefetch Control, one pair of bits seen twice
/// (DDI 0246 §3.3.14).
const PREFETCH_ENABLES: u32 = 0b11 << 28;
/// Prefetch Control's implemented bits: double linefill (30), the two
/// prefetch enables (29:28), double linefill on WRAP read disable (27),
/// prefetch drop (24), incr double linefill (23), not same ID on exclusive
/// sequence (21), and the prefetch offset (4:0).
const PREFETCH_MASK: u32 = 0x79a0_001f;

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    ctrl: u32,
    aux: u32,
    tag_latency: u32,
    data_latency: u32,
    event_ctrl: u32,
    /// Event counter configuration, counter 0 then counter 1.
    event_config: [u32; 2],
    /// Event counter values, counter 0 then counter 1. Nothing here counts
    /// events, so they hold what was written.
    event_value: [u32; 2],
    int_mask: u32,
    /// Raw interrupt status. No condition this model can reach raises one, so
    /// it stays zero; it is state because a snapshot of silicon would carry it.
    int_raw: u32,
    /// Data and instruction lockdown per master: `[d0, i0, d1, i1, …]`, in
    /// register order from `0x900`.
    lockdown: [u32; 16],
    lock_line: u32,
    filter_start: u32,
    filter_end: u32,
    debug: u32,
    prefetch: u32,
    power: u32,
}

impl State {
    fn new(config: &Config) -> State {
        State {
            ctrl: 0,
            aux: config.aux,
            tag_latency: config.tag_latency,
            data_latency: config.data_latency,
            event_ctrl: 0,
            event_config: [0; 2],
            event_value: [0; 2],
            int_mask: 0,
            int_raw: 0,
            lockdown: [0; 16],
            lock_line: 0,
            filter_start: 0,
            filter_end: 0,
            debug: 0,
            prefetch: config.aux & PREFETCH_ENABLES,
            power: 0,
        }
    }

    fn enabled(&self) -> bool {
        self.ctrl & 1 != 0
    }

    /// One bit per way, as the lockdown and by-way registers are sized.
    fn way_mask(&self) -> u32 {
        if self.aux & AUX_ASSOC_16 != 0 {
            0xffff
        } else {
            0xff
        }
    }
}

/// What the machine file configured, kept so a reset can put it back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Config {
    cache_id: u32,
    aux: u32,
    tag_latency: u32,
    data_latency: u32,
}

/// The register file.
struct Registers {
    state: Mutex<State>,
    config: Config,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("config", &self.config);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Registers {
    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        match offset {
            0x000 => self.config.cache_id,
            0x004 => {
                // DDI 0246 §3.3.2: the Dsize (23:12) and Isize (11:0) fields
                // both report the configured way size and associativity, with
                // a 32-byte line (line-length fields zero); unified, so H (24)
                // is clear.
                let way = (state.aux & AUX_WAY_SIZE) >> AUX_WAY_SIZE_SHIFT;
                let assoc = u32::from(state.aux & AUX_ASSOC_16 != 0);
                let size = (way << 8) | (assoc << 6);
                (CTYPE << 25) | (size << 12) | size
            }
            0x100 => state.ctrl,
            0x104 => state.aux,
            0x108 => state.tag_latency,
            0x10c => state.data_latency,
            0x200 => state.event_ctrl,
            0x204 => state.event_config[1],
            0x208 => state.event_config[0],
            0x20c => state.event_value[1],
            0x210 => state.event_value[0],
            0x214 => state.int_mask,
            0x218 => state.int_raw & state.int_mask,
            0x21c => state.int_raw,
            // Cache Sync and every maintenance operation: bit 0 (or the way
            // mask) says an operation is in progress, and none ever is.
            0x730 | 0x770 | 0x77c | 0x7b0 | 0x7b8 | 0x7bc | 0x7f0 | 0x7f8 | 0x7fc => 0,
            0x900..0x940 => state.lockdown[((offset - 0x900) / 4) as usize],
            0x950 => state.lock_line,
            // Unlock All Lines by Way: complete as soon as written.
            0x954 => 0,
            0xc00 => state.filter_start,
            0xc04 => state.filter_end,
            0xf40 => state.debug,
            0xf60 => state.prefetch,
            0xf80 => state.power,
            _ => 0,
        }
    }

    fn write_register(&self, offset: u64, value: u32) -> MemResult {
        let mut state = self.state.lock();
        match offset {
            0x100 => state.ctrl = value & 1,
            // Auxiliary Control and the RAM latency controls may only be
            // written with the cache disabled; with it enabled the write is
            // refused with SLVERR (DDI 0246 §3.3.4, §3.3.5).
            0x104 | 0x108 | 0x10c if state.enabled() => return Err(BusError::BadAccess),
            0x104 => {
                state.aux = value & AUX_MASK;
                state.prefetch = (state.prefetch & !PREFETCH_ENABLES) | (value & PREFETCH_ENABLES);
            }
            0x108 => state.tag_latency = value & 0x777,
            0x10c => state.data_latency = value & 0x777,
            0x200 => {
                // Bits 2:1 reset counter 1 and counter 0 and read zero; bit 0
                // enables counting.
                if value & 0b010 != 0 {
                    state.event_value[0] = 0;
                }
                if value & 0b100 != 0 {
                    state.event_value[1] = 0;
                }
                state.event_ctrl = value & 1;
            }
            0x204 => state.event_config[1] = value & 0x3f,
            0x208 => state.event_config[0] = value & 0x3f,
            0x20c => state.event_value[1] = value,
            0x210 => state.event_value[0] = value,
            0x214 => state.int_mask = value & 0x1ff,
            0x220 => state.int_raw &= !value,
            // Every maintenance operation, done: there is no line to find.
            0x730 | 0x770 | 0x77c | 0x7b0 | 0x7b8 | 0x7bc | 0x7f0 | 0x7f8 | 0x7fc | 0x954 => {}
            0x900..0x940 => {
                let mask = state.way_mask();
                state.lockdown[((offset - 0x900) / 4) as usize] = value & mask;
            }
            0x950 => state.lock_line = value & 1,
            0xc00 => state.filter_start = value & 0xfff0_0001,
            0xc04 => state.filter_end = value & 0xfff0_0000,
            0xf40 => state.debug = value & 0b111,
            0xf60 => {
                state.prefetch = value & PREFETCH_MASK;
                state.aux = (state.aux & !PREFETCH_ENABLES) | (value & PREFETCH_ENABLES);
            }
            0xf80 => state.power = value & 0b11,
            _ => {}
        }
        Ok(())
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        // No read here has a side effect, so `debug` needs nothing.
        dst.copy_from_slice(&self.read_register(offset).to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // Enabling a cache or starting a maintenance operation is not
            // something a debugger does by looking.
            return Err(BusError::BadAccess);
        }
        self.write_register(offset, u32::from_le_bytes([src[0], src[1], src[2], src[3]]))
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

/// An L2C-310 cache controller's register file.
#[derive(Debug)]
pub struct L2c310 {
    regs: Arc<Registers>,
    region: RegionRef,
}

/// The way-size encoding for `way` bytes (DDI 0246 §3.3.4: `0b001` is 16 KiB
/// up to `0b110`, 512 KiB).
fn way_code(way: u64) -> Option<u32> {
    match way {
        0x4000 => Some(0b001),
        0x8000 => Some(0b010),
        0x1_0000 => Some(0b011),
        0x2_0000 => Some(0b100),
        0x4_0000 => Some(0b101),
        0x8_0000 => Some(0b110),
        _ => None,
    }
}

impl L2c310 {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a cache the Auxiliary Control register cannot
    /// describe, an `aux` that disagrees with `size` and `ways`, an unknown
    /// revision, or a property this class does not know.
    pub fn new(props: &Props) -> Result<L2c310> {
        let mut r = props.reader();
        let size = r.or_size("size", 1024 * 1024)?;
        let ways = r.or_range("ways", 16u64, 8..=16)?;
        let aux = r.optional::<u64>("aux")?;
        let revision = r.or_str("revision", "r3p1")?;
        let tag_latency = r.or_range("tag-latency", 0u64, 0..=0x777)?;
        let data_latency = r.or_range("data-latency", 0u64, 0..=0x777)?;
        r.finish()?;
        if ways != 8 && ways != 16 {
            return Err(Error::Property(format!(
                "an L2C-310 is 8-way or 16-way, not {ways}-way"
            )));
        }
        let code = way_code(size / ways)
            .filter(|_| size.is_multiple_of(ways))
            .ok_or_else(|| {
                Error::Property(format!(
                    "{size} bytes in {ways} ways is {} bytes a way, and an L2C-310's way is a \
                     power of two from 16K to 512K",
                    size / ways
                ))
            })?;
        let pins = (code << AUX_WAY_SIZE_SHIFT) | if ways == 16 { AUX_ASSOC_16 } else { 0 };
        let aux = match aux {
            None => AUX_RESET_BASE | pins,
            Some(value) => {
                let value = u32::try_from(value)
                    .map_err(|_| Error::Property(String::from("`aux` is a 32-bit register")))?;
                if value & (AUX_WAY_SIZE | AUX_ASSOC_16) != pins {
                    return Err(Error::Property(format!(
                        "`aux = {value:#x}` describes a different cache from `size` and `ways`; \
                         its bits 19:16 must read {:#x}",
                        pins >> 16
                    )));
                }
                value & AUX_MASK
            }
        };
        let rtl = REVISIONS
            .iter()
            .find(|(name, _)| *name == revision)
            .map(|(_, rtl)| *rtl)
            .ok_or_else(|| {
                Error::Property(format!(
                    "`revision = \"{revision}\"` is not an L2C-310 release; the TRM lists \
                     r0p0, r1p0, r2p0, r3p0, r3p1, r3p2 and r3p3"
                ))
            })?;
        Ok(L2c310::build(Config {
            cache_id: CACHE_ID_BASE | rtl,
            aux,
            tag_latency: tag_latency as u32,
            data_latency: data_latency as u32,
        }))
    }

    fn build(config: Config) -> L2c310 {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(&config)),
            config,
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        L2c310 { regs, region }
    }
}

/// The `arm.l2c310` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "ARM L2C-310 (PL310) L2 cache controller: registers and instant maintenance",
    properties: &[
        PropertySpec {
            name: "size",
            kind: ValueKind::Size,
            required: false,
            summary: "total cache size (default 1M)",
        },
        PropertySpec {
            name: "ways",
            kind: ValueKind::Uint,
            required: false,
            summary: "associativity, 8 or 16 (default 16)",
        },
        PropertySpec {
            name: "aux",
            kind: ValueKind::Uint,
            required: false,
            summary: "Auxiliary Control's reset value (default computed from size and ways)",
        },
        PropertySpec {
            name: "revision",
            kind: ValueKind::Str,
            required: false,
            summary: "the release Cache ID reports, r0p0 to r3p3 (default \"r3p1\")",
        },
        PropertySpec {
            name: "tag-latency",
            kind: ValueKind::Uint,
            required: false,
            summary: "Tag RAM Latency Control's reset value (default 0)",
        },
        PropertySpec {
            name: "data-latency",
            kind: ValueKind::Uint,
            required: false,
            summary: "Data RAM Latency Control's reset value (default 0)",
        },
    ],
    construct: |props| Ok(Box::new(L2c310::new(props)?)),
};

impl Device for L2c310 {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        *self.regs.state.lock() = State::new(&self.regs.config);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let s = self.regs.state.lock();
        for word in [
            s.ctrl,
            s.aux,
            s.tag_latency,
            s.data_latency,
            s.event_ctrl,
            s.event_config[0],
            s.event_config[1],
            s.event_value[0],
            s.event_value[1],
            s.int_mask,
            s.int_raw,
            s.lock_line,
            s.filter_start,
            s.filter_end,
            s.debug,
            s.prefetch,
            s.power,
        ] {
            w.write_u32(word)?;
        }
        w.write_seq_len(s.lockdown.len() as u64)?;
        for word in s.lockdown {
            w.write_u32(word)?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut s = State::new(&self.regs.config);
        s.ctrl = r.read_u32()? & 1;
        s.aux = r.read_u32()? & AUX_MASK;
        s.tag_latency = r.read_u32()?;
        s.data_latency = r.read_u32()?;
        s.event_ctrl = r.read_u32()?;
        s.event_config[0] = r.read_u32()?;
        s.event_config[1] = r.read_u32()?;
        s.event_value[0] = r.read_u32()?;
        s.event_value[1] = r.read_u32()?;
        s.int_mask = r.read_u32()?;
        s.int_raw = r.read_u32()?;
        s.lock_line = r.read_u32()?;
        s.filter_start = r.read_u32()?;
        s.filter_end = r.read_u32()?;
        s.debug = r.read_u32()?;
        s.prefetch = r.read_u32()?;
        s.power = r.read_u32()?;
        let count = r.read_seq_len(4)?;
        if count != s.lockdown.len() as u64 {
            return Err(Error::State(format!(
                "snapshot has {count} lockdown registers, an L2C-310 has 16"
            )));
        }
        for word in &mut s.lockdown {
            *word = r.read_u32()?;
        }
        *self.regs.state.lock() = s;
        Ok(())
    }
}

impl Instance for L2c310 {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(L2c310::new(props)?)))
}

/// What the validator should know about `arm.l2c310`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PropSchema};
    ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("size", ValueKind::Size))
        .prop(PropSchema::new("ways", ValueKind::Uint).range(8, 16))
        .prop(PropSchema::new("aux", ValueKind::Uint).range(0, 0xffff_ffff))
        .prop(PropSchema::new("revision", ValueKind::Str))
        .prop(PropSchema::new("tag-latency", ValueKind::Uint).range(0, 0x777))
        .prop(PropSchema::new("data-latency", ValueKind::Uint).range(0, 0x777))
        .region("")
        .region("regs")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

    fn h1() -> L2c310 {
        L2c310::new(
            &Props::new()
                .with("size", 1024u64 * 1024)
                .with("ways", 16u64),
        )
        .unwrap()
    }

    fn read(l2: &L2c310, offset: u64) -> u32 {
        let mut b = [0u8; 4];
        l2.regs.read(offset, &mut b, MemAttrs::DEFAULT).unwrap();
        u32::from_le_bytes(b)
    }

    fn write(l2: &L2c310, offset: u64, value: u32) -> MemResult {
        l2.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
    }

    #[test]
    fn identification_describes_the_configured_cache() {
        let l2 = h1();
        assert_eq!(read(&l2, 0x000), 0x4100_00c6, "Arm, L2C-310, r3p1");
        // 1 MiB in 16 ways is 64 KiB a way: way-size code 0b011, 16-way.
        assert_eq!(read(&l2, 0x104), 0x0200_0000 | (0b011 << 17) | (1 << 16));
        let ty = read(&l2, 0x004);
        assert_eq!((ty >> 25) & 0xf, CTYPE);
        assert_eq!((ty >> 20) & 7, 0b011, "Dsize way size");
        assert_eq!((ty >> 18) & 1, 1, "Dsize 16-way");
        assert_eq!((ty >> 8) & 7, 0b011, "Isize way size");
        assert_eq!((ty >> 6) & 1, 1, "Isize 16-way");
        assert_eq!(ty & (1 << 24), 0, "unified");

        let small = L2c310::new(
            &Props::new()
                .with("size", 256u64 * 1024)
                .with("ways", 8u64)
                .with("revision", "r3p2"),
        )
        .unwrap();
        assert_eq!(read(&small, 0x000), 0x4100_00c8);
        assert_eq!(read(&small, 0x104), 0x0200_0000 | (0b010 << 17), "32K ways");
    }

    #[test]
    fn a_configuration_the_registers_cannot_describe_is_refused() {
        let bad = |props: Props| L2c310::new(&props).is_err();
        assert!(bad(Props::new().with("size", 3u64 * 1024 * 1024)));
        assert!(bad(Props::new().with("ways", 12u64)));
        assert!(bad(Props::new().with("revision", "r4p0")));
        assert!(
            bad(Props::new().with("aux", 0x0202_0000u64)),
            "a 16K-way aux"
        );
        let aux = L2c310::new(&Props::new().with("aux", 0x3207_0000u64)).unwrap();
        assert_eq!(
            read(&aux, 0x104),
            0x3207_0000,
            "an agreeing aux is taken whole"
        );
        assert_eq!(
            read(&aux, 0xf60),
            0x3000_0000,
            "and its prefetch enables alias"
        );
    }

    #[test]
    fn a_way_maintenance_operation_is_complete_when_polled() {
        let l2 = h1();
        for op in [0x77c, 0x7bc, 0x7fc] {
            write(&l2, op, 0xffff).unwrap();
            assert_eq!(read(&l2, op), 0, "{op:#x}: no way still in progress");
        }
        for op in [0x730, 0x770, 0x7b0, 0x7b8, 0x7f0, 0x7f8] {
            write(&l2, op, 0x8000_0000).unwrap();
            assert_eq!(read(&l2, op) & 1, 0, "{op:#x}: not in progress");
        }
    }

    #[test]
    fn the_configuration_registers_are_locked_while_the_cache_is_on() {
        let l2 = h1();
        write(&l2, 0x108, 0x111).unwrap();
        write(&l2, 0x100, 1).unwrap();
        assert_eq!(read(&l2, 0x100), 1);
        assert!(write(&l2, 0x104, 0).is_err(), "SLVERR");
        assert!(write(&l2, 0x108, 0x222).is_err());
        assert_eq!(read(&l2, 0x108), 0x111);
        write(&l2, 0x100, 0).unwrap();
        write(&l2, 0x104, 0xffff_ffff).unwrap();
        assert_eq!(read(&l2, 0x104), AUX_MASK);
    }

    #[test]
    fn stored_registers_read_back_and_write_only_ones_read_zero() {
        let l2 = h1();
        write(&l2, 0x900, 0xffff_ffff).unwrap();
        assert_eq!(read(&l2, 0x900), 0xffff, "sixteen ways");
        write(&l2, 0x93c, 0x1234).unwrap();
        assert_eq!(read(&l2, 0x93c), 0x1234, "master 7's instruction lockdown");
        write(&l2, 0xf60, 0xffff_ffff).unwrap();
        assert_eq!(read(&l2, 0xf60), PREFETCH_MASK);
        assert_eq!(read(&l2, 0x104) & PREFETCH_ENABLES, PREFETCH_ENABLES);
        write(&l2, 0xf80, 3).unwrap();
        assert_eq!(read(&l2, 0xf80), 3);
        write(&l2, 0xf40, 0xff).unwrap();
        assert_eq!(read(&l2, 0xf40), 7);
        write(&l2, 0x214, 0xffff).unwrap();
        assert_eq!(read(&l2, 0x214), 0x1ff);
        assert_eq!(read(&l2, 0x218), 0, "nothing raw to mask");
        write(&l2, 0x210, 55).unwrap();
        write(&l2, 0x200, 0b011).unwrap();
        assert_eq!(read(&l2, 0x210), 0, "counter 0 reset");
        assert_eq!(read(&l2, 0x200), 1, "reset bits read zero");
        assert_eq!(read(&l2, 0x220), 0);
        assert_eq!(read(&l2, 0x954), 0);
        assert!(
            l2.regs
                .write(
                    0x100,
                    &1u32.to_le_bytes(),
                    MemAttrs {
                        debug: true,
                        ..MemAttrs::DEFAULT
                    }
                )
                .is_err(),
            "a debugger cannot enable the cache"
        );
        assert_eq!(read(&l2, 0x100), 0);
    }

    fn snapshot(l2: &L2c310) -> alloc::vec::Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("l2", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("l2", CLASS.name, CLASS.version).unwrap();
            l2.save(&mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    #[test]
    fn state_round_trips_to_an_identical_hash() {
        let l2 = h1();
        write(&l2, 0x104, 0x7207_0000).unwrap();
        write(&l2, 0x908, 0xf0).unwrap();
        write(&l2, 0xf60, 0x0000_0007).unwrap();
        write(&l2, 0x100, 1).unwrap();
        let bytes = snapshot(&l2);
        let restored = h1();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("l2", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();
        assert_eq!(snapshot(&l2), snapshot(&restored));
        assert_eq!(read(&restored, 0x100), 1);
        restored.reset(ResetKind::Cold);
        assert_eq!(read(&restored, 0x104), read(&h1(), 0x104), "reset value");
    }
}
