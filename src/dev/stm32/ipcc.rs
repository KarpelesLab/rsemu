//! The STM32WB inter-processor communication controller.
//!
//! One class, `st.ipcc`: the doorbell block between the WB's two cores — the
//! Cortex-M4 that runs the application (processor 1, CPU1) and the Cortex-M0+
//! that runs the radio stack (processor 2, CPU2). It carries no data. Each of
//! its six channels is one flag per direction, and the mailbox itself is a
//! table in shared SRAM2 that the two sides agree on; the IPCC only says
//! "there is something in channel *n* for you" and "you may write channel *n*
//! again".
//!
//! # The register map
//!
//! RM0434's IPCC chapter, offsets as ST CMSIS `stm32wb55xx.h` (`IPCC_TypeDef`,
//! `IPCC_CommonTypeDef`) gives them. The first four registers are processor 1's
//! and the next four the same four for processor 2:
//!
//! | Offset | Register | What it holds |
//! | --- | --- | --- |
//! | `0x00` / `0x10` | `C1CR` / `C2CR` | `RXOIE` (bit 0) and `TXFIE` (bit 16): the two interrupt enables |
//! | `0x04` / `0x14` | `C1MR` / `C2MR` | `CHnOM` (bits 0–5) masks channel *n*'s RX-occupied interrupt, `CHnFM` (bits 16–21) its TX-free one |
//! | `0x08` / `0x18` | `C1SCR` / `C2SCR` | write-only: `CHnC` (bit *n*) clears the *other* side's flag, `CHnS` (bit 16+*n*) sets this side's |
//! | `0x0C` / `0x1C` | `C1TOC2SR` / `C2TOC1SR` | read-only: `CHnF` (bits 0–5), set while channel *n* holds a message for the other side |
//! | `0x3F0` | `HWCFGR` | `0x0000_0006`: six channels |
//! | `0x3F4` | `VERR` | `0x0000_0010`: revision 1.0 |
//! | `0x3F8` | `IPIDR` | `0x0010_0071` |
//! | `0x3FC` | `SIDR` | `0xA3C5_DD01` |
//!
//! The identification values and the `0xFFFF_FFFF` reset value of both mask
//! registers are ST's `STM32WB55_CM4.svd` (Apache-2.0), which states them as
//! reset values; the CMSIS header stops at `C2TOC1SR` and does not name the
//! identification block at all. A mask register's reserved bits read back as
//! that reset value — set — and writes do not move them.
//!
//! Every other offset in the kilobyte reads as zero and ignores writes.
//!
//! # A channel, end to end
//!
//! What ST's WPAN transport does with channel *n* in the CPU1→CPU2 direction:
//!
//! 1. CPU1 fills the mailbox slot in SRAM2, then writes `C1SCR.CHnS`.
//!    `C1TOC2SR.CHnF` sets.
//! 2. CPU2 sees it — its `RXOIE` interrupt, if it unmasked `CHnOM` in
//!    `C2MR`, or a poll of `C1TOC2SR` — reads the slot, and writes
//!    `C2SCR.CHnC`. `C1TOC2SR.CHnF` clears.
//! 3. CPU1, waiting with `TXFIE` set and `CHnFM` clear in `C1MR`, takes its
//!    TX-free interrupt and may write the channel again.
//!
//! The other direction is the mirror image through `C2SCR.CHnS`,
//! `C2TOC1SR` and `C1SCR.CHnC`.
//!
//! # The four interrupts
//!
//! Levels, recomputed after every write:
//!
//! | Output | High while |
//! | --- | --- |
//! | `c1-rx` | `C1CR.RXOIE` and some channel has `C2TOC1SR.CHnF` set with `C1MR.CHnOM` clear |
//! | `c1-tx` | `C1CR.TXFIE` and some channel has `C1TOC2SR.CHnF` clear with `C1MR.CHnFM` clear |
//! | `c2-rx` | `C2CR.RXOIE` and some channel has `C1TOC2SR.CHnF` set with `C2MR.CHnOM` clear |
//! | `c2-tx` | `C2CR.TXFIE` and some channel has `C2TOC1SR.CHnF` clear with `C2MR.CHnFM` clear |
//!
//! On an STM32WB55 `c1-rx` is NVIC line 44 (`IPCC_C1_RX_IRQn`) and `c1-tx`
//! line 45 (`IPCC_C1_TX_IRQn`) — `stm32wb55xx.h` — and the two `c2` lines go
//! to CPU2. The board file draws those wires; nothing here knows the numbers.
//!
//! # Which processor is asking
//!
//! None of these registers is banked by bus master: processor 1 drives the
//! `C1*` registers and processor 2 the `C2*` ones by address alone, which is
//! all a CPU2 model needs — it reaches `0x10`–`0x1C` through the ordinary
//! memory space like any other bus master. The block therefore ignores
//! `MemAttrs::requester`. A firmware on one core that writes the other core's
//! registers gets exactly what the hardware would give it.
//!
//! # The clock
//!
//! `enable` is `RCC_AHB3ENR.IPCCEN`, bit 20, and `reset` is
//! `RCC_AHB3RSTR.IPCCRST`, the same bit (`stm32wb55xx.h`). CPU2 has its own
//! enable for the block in `RCC_C2AHB3ENR`, bit 20 again; the `enable` pin ORs
//! every wire drawn to it, so a board that models both draws both. [`gate`]
//! has what a gated block does. The interrupt outputs hold their level while
//! the gate is shut, which is what most of the family's models do (see
//! [`gate`]'s note on level outputs).
//!
//! # Sources
//!
//! ST **RM0434**, the "Inter-processor communication controller (IPCC)"
//! chapter, for the channel protocol and the interrupt equations; ST CMSIS
//! `stm32wb55xx.h` (Apache-2.0) for the offsets, the bit positions and the
//! interrupt numbers; ST `STM32WB55_CM4.svd` (Apache-2.0) for the reset and
//! identification values. ST's BSD-3 WB HAL (`stm32wbxx_hal_ipcc.c`) was read
//! for what firmware polls, not copied. No emulator source of any licence was
//! consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::Props;
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSource};
use crate::dev::stm32::gate::{self, ClockGate, Gated};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.ipcc";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// Processor 1's RX-occupied interrupt (`IPCC_C1_RX`, NVIC 44 on a WB55).
pub const C1_RX_PIN: &str = "c1-rx";
/// Processor 1's TX-free interrupt (`IPCC_C1_TX`, NVIC 45 on a WB55).
pub const C1_TX_PIN: &str = "c1-tx";
/// Processor 2's RX-occupied interrupt.
pub const C2_RX_PIN: &str = "c2-rx";
/// Processor 2's TX-free interrupt.
pub const C2_TX_PIN: &str = "c2-tx";

/// The four outputs, in [`Output`] order.
const PINS: [&str; 4] = [C1_RX_PIN, C1_TX_PIN, C2_RX_PIN, C2_TX_PIN];

/// The aperture: one kilobyte, the identification block at its top.
pub const REGISTER_BYTES: u64 = 0x400;

/// How many channels each direction has (`HWCFGR.CHANNELS`).
pub const CHANNELS: u32 = 6;

/// One bit per channel.
const CH_MASK: u32 = (1 << CHANNELS) - 1;

/// `CxCR.RXOIE`, bit 0.
const CR_RXOIE: u32 = 1 << 0;
/// `CxCR.TXFIE`, bit 16.
const CR_TXFIE: u32 = 1 << 16;
/// The bits `CxCR` implements.
const CR_MASK: u32 = CR_RXOIE | CR_TXFIE;

/// Where `CxMR.CHnFM` and `CxSCR.CHnS` start.
const HIGH_SHIFT: u32 = 16;
/// The bits `CxMR` implements: `CHnOM` and `CHnFM`.
const MR_MASK: u32 = CH_MASK | (CH_MASK << HIGH_SHIFT);
/// `CxMR`'s reset value, every interrupt masked (ST `STM32WB55_CM4.svd`).
const MR_RESET: u32 = 0xffff_ffff;

/// `HWCFGR`: `CHANNELS = 6`.
const HWCFGR: u32 = 0x0000_0006;
/// `VERR`: `MAJREV = 1`, `MINREV = 0`.
const VERR: u32 = 0x0000_0010;
/// `IPIDR`.
const IPIDR: u32 = 0x0010_0071;
/// `SIDR`.
const SIDR: u32 = 0xa3c5_dd01;

/// The IPCC's register file.
///
/// Indexed by processor: `[0]` is processor 1's, `[1]` processor 2's. `sr[0]`
/// is `C1TOC2SR` — the channels processor 1 has filled — and `sr[1]` is
/// `C2TOC1SR`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct State {
    cr: [u32; 2],
    mr: [u32; 2],
    sr: [u32; 2],
}

impl Default for State {
    fn default() -> State {
        State {
            cr: [0; 2],
            mr: [MR_RESET; 2],
            sr: [0; 2],
        }
    }
}

impl State {
    /// Processor `p`'s RX-occupied level: a channel the *other* side filled,
    /// not masked here.
    const fn rx(&self, p: usize) -> bool {
        let other = self.sr[1 - p];
        self.cr[p] & CR_RXOIE != 0 && other & !self.mr[p] & CH_MASK != 0
    }

    /// Processor `p`'s TX-free level: a channel of *its own* that is empty,
    /// not masked here.
    const fn tx(&self, p: usize) -> bool {
        let free = !self.sr[p];
        self.cr[p] & CR_TXFIE != 0 && free & !(self.mr[p] >> HIGH_SHIFT) & CH_MASK != 0
    }

    /// All four outputs, in [`PINS`] order.
    const fn levels(&self) -> [bool; 4] {
        [self.rx(0), self.tx(0), self.rx(1), self.tx(1)]
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// The four interrupt outputs, in [`PINS`] order, taken at realize.
    irq: Mutex<[Option<WireSource>; 4]>,
    /// `RCC_AHB3ENR.IPCCEN` (and `RCC_C2AHB3ENR.IPCCEN`) and
    /// `RCC_AHB3RSTR.IPCCRST`, as a board's wires deliver them.
    gate: ClockGate,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// Drive every connected output to what the flags now say.
    ///
    /// Called with **no lock of this device held**, as the re-entrancy
    /// contract requires: a sink may call straight back in. A wire suppresses
    /// a repeat of its current level, so driving all four is cheap.
    fn refresh_irq(&self) {
        let levels = self.state.lock().levels();
        let sources = self.irq.lock().clone();
        for (source, high) in sources.iter().zip(levels) {
            if let Some(source) = source {
                source.set(Level::from_bool(high));
            }
        }
    }

    /// One register's value. Nothing here has a read side effect.
    fn read_register(&self, offset: u64) -> u32 {
        let state = self.state.lock();
        match offset {
            0x00 => state.cr[0],
            0x04 => state.mr[0],
            0x0c => state.sr[0],
            0x10 => state.cr[1],
            0x14 => state.mr[1],
            0x1c => state.sr[1],
            0x3f0 => HWCFGR,
            0x3f4 => VERR,
            0x3f8 => IPIDR,
            0x3fc => SIDR,
            // `CxSCR` is write-only, and the rest of the kilobyte is reserved.
            _ => 0,
        }
    }

    /// Write one register.
    fn write_register(&self, offset: u64, value: u32) {
        let mut state = self.state.lock();
        match offset {
            0x00 | 0x10 => state.cr[side(offset)] = value & CR_MASK,
            0x04 | 0x14 => {
                let p = side(offset);
                state.mr[p] = (state.mr[p] & !MR_MASK) | (value & MR_MASK);
            }
            0x08 | 0x18 => {
                let p = side(offset);
                // `CHnC` frees a channel the *other* side filled; `CHnS` fills
                // one of this side's own. Different flags, so one write may
                // carry both without an order between them mattering.
                state.sr[1 - p] &= !(value & CH_MASK);
                state.sr[p] |= (value >> HIGH_SHIFT) & CH_MASK;
            }
            // The status registers and the identification block are
            // read-only; everything else is reserved.
            _ => {}
        }
    }
}

/// Which processor's register `offset` is: 0 below `0x10`, 1 from it.
const fn side(offset: u64) -> usize {
    if offset < 0x10 { 0 } else { 1 }
}

impl Gated for Registers {
    fn clock_gate(&self) -> &ClockGate {
        &self.gate
    }

    fn gate_reset(&self) {
        // `RCC_AHB3RSTR.IPCCRST`: every channel empty, every interrupt masked
        // and disabled — which is what `Device::reset` does.
        *self.state.lock() = State::default();
        self.refresh_irq();
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        if offset >= REGISTER_BYTES {
            return Err(BusError::BadAccess);
        }
        if !self.gate.live() {
            // No `IPCCEN`, or `IPCCRST` standing: the block reads as zero
            // ([`gate`](super::gate)), for a debugger too.
            dst.fill(0);
            return Ok(());
        }
        // No register here changes on a read, so a debug read is the same
        // read (`MemAttrs::debug`, `ROADMAP.md` §15 invariant 5).
        let bytes = self.read_register(offset).to_le_bytes();
        for (slot, byte) in dst.iter_mut().zip(bytes) {
            *slot = byte;
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if offset >= REGISTER_BYTES {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write to `CxSCR` would ring a doorbell the other core
            // acts on. There is no harmless version.
            return Err(BusError::BadAccess);
        }
        if !self.gate.live() {
            // Not effective: a doorbell rung at an unclocked block is lost.
            return Ok(());
        }
        let mut word = [0u8; 4];
        for (slot, byte) in word.iter_mut().zip(src) {
            *slot = *byte;
        }
        self.write_register(offset, u32::from_le_bytes(word));
        // Outside the critical section, as the re-entrancy contract requires.
        self.refresh_irq();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Word access: every register is a 32-bit AHB register, and ST's HAL
        // and LL touch them no other way.
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32WB inter-processor communication controller.
#[derive(Debug)]
pub struct Ipcc {
    regs: Arc<Registers>,
    region: RegionRef,
}

/// One of the four interrupt outputs, for [`Ipcc::level`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Output {
    /// `c1-rx`.
    C1Rx,
    /// `c1-tx`.
    C1Tx,
    /// `c2-rx`.
    C2Rx,
    /// `c2-tx`.
    C2Tx,
}

impl Ipcc {
    /// Validate `props` and build the block.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property was given: this class takes none.
    pub fn new(props: &Props) -> Result<Ipcc> {
        let mut r = props.reader();
        // `clock` belongs to the machine and every class accepts it.
        r.touch("clock");
        r.finish()?;
        Ok(Ipcc::build())
    }

    /// Build one directly — the route a test takes.
    #[must_use]
    pub fn build() -> Ipcc {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            irq: Mutex::with_rank(LockRank::WIRE, [None, None, None, None]),
            gate: ClockGate::new(),
        });
        let region = Arc::new(Region::io(
            "ipcc",
            REGISTER_BYTES,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Ipcc { regs, region }
    }

    /// The level `output` is being driven to.
    #[must_use]
    pub fn level(&self, output: Output) -> Level {
        Level::from_bool(self.regs.state.lock().levels()[output as usize])
    }

    /// `C1TOC2SR`: the channels processor 1 has filled for processor 2.
    #[must_use]
    pub fn c1_to_c2(&self) -> u32 {
        self.regs.state.lock().sr[0]
    }

    /// `C2TOC1SR`: the channels processor 2 has filled for processor 1.
    #[must_use]
    pub fn c2_to_c1(&self) -> u32 {
        self.regs.state.lock().sr[1]
    }
}

impl Device for Ipcc {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // Nothing outward: a `map` statement places the region and the board's
        // `wire` statements reach the pins through `connect`.
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds: AHB register state with no battery behind it.
        *self.regs.state.lock() = State::default();
        self.regs.refresh_irq();
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = *self.regs.state.lock();
        for p in 0..2 {
            w.write_u32(state.cr[p])?;
            w.write_u32(state.mr[p])?;
            w.write_u32(state.sr[p])?;
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::default();
        for p in 0..2 {
            state.cr[p] = r.read_u32()?;
            state.mr[p] = r.read_u32()?;
            state.sr[p] = r.read_u32()?;
        }
        for p in 0..2 {
            if state.cr[p] & !CR_MASK != 0
                || state.mr[p] & !MR_MASK != MR_RESET & !MR_MASK
                || state.sr[p] & !CH_MASK != 0
            {
                return Err(Error::State(format!(
                    "snapshot has IPCC processor {} with CR={:#x} MR={:#x} SR={:#x}, \
                     which sets bits the block does not have",
                    p + 1,
                    state.cr[p],
                    state.mr[p],
                    state.sr[p]
                )));
            }
        }
        *self.regs.state.lock() = state;
        self.regs.refresh_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let Some(n) = PINS.iter().position(|p| *p == port) else {
            return Err(Error::Config {
                at: String::from(port),
                message: format!(
                    "an IPCC drives `{C1_RX_PIN}`, `{C1_TX_PIN}`, `{C2_RX_PIN}` and `{C2_TX_PIN}`"
                ),
            });
        };
        self.regs.irq.lock()[n] = Some(source);
        self.regs.refresh_irq();
        Ok(())
    }

    fn announce(&self, port: &str) {
        if PINS.contains(&port) {
            self.regs.refresh_irq();
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // `enable` and `reset`; the four interrupts are outputs.
        gate::sink(&self.regs, port, sources)
    }
}

impl Instance for Ipcc {}

/// The `st.ipcc` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32WB inter-processor communication controller: six doorbell channels each way",
    properties: &[],
    construct: |props| Ok(Box::new(Ipcc::new(props)?)),
};

/// Add [`CLASS`] to a registry.
///
/// # Errors
///
/// If something already claimed the name.
pub fn register(registry: &mut crate::core::Registry) -> Result<()> {
    registry.add(&CLASS)
}

/// Bind [`CLASS`] into the machine graph.
///
/// # Errors
///
/// If the class is already bound.
pub fn bind(bindings: &mut crate::machine::Bindings) -> Result<()> {
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Ipcc::new(props)?)))
}

/// What the validator should know about `st.ipcc`.
#[must_use]
pub fn schema() -> ClassSchema {
    let mut schema = ClassSchema::new(CLASS_NAME).region("").region("regs");
    for pin in PINS {
        schema = schema.port(pin, PortDir::Out);
    }
    // `RCC_AHB3ENR.IPCCEN` and `RCC_AHB3RSTR.IPCCRST`.
    gate::ports(schema)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicBool, Ordering};
    use crate::core::wire::{Wire, WireIdAllocator, WireSink};
    use alloc::vec::Vec;

    const C1CR: u64 = 0x00;
    const C1MR: u64 = 0x04;
    const C1SCR: u64 = 0x08;
    const C1TOC2SR: u64 = 0x0c;
    const C2CR: u64 = 0x10;
    const C2MR: u64 = 0x14;
    const C2SCR: u64 = 0x18;
    const C2TOC1SR: u64 = 0x1c;

    /// `CHnS` / `CHnFM` for channel `n` (1-based, as ST numbers them).
    const fn high(n: u32) -> u32 {
        1 << (HIGH_SHIFT + n - 1)
    }

    /// `CHnC` / `CHnOM` / `CHnF` for channel `n` (1-based).
    const fn low(n: u32) -> u32 {
        1 << (n - 1)
    }

    fn read(d: &Ipcc, offset: u64) -> u32 {
        let mut buf = [0u8; 4];
        d.regs
            .read(offset, &mut buf, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(buf)
    }

    fn peek(d: &Ipcc, offset: u64) -> u32 {
        let mut buf = [0u8; 4];
        d.regs
            .read(offset, &mut buf, MemAttrs::DEBUG)
            .expect("a debug word read is legal");
        u32::from_le_bytes(buf)
    }

    fn write(d: &Ipcc, offset: u64, value: u32) {
        d.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    #[derive(Debug, Default)]
    struct Probe {
        high: AtomicBool,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.high.store(level.is_high(), Ordering::Relaxed);
        }
    }

    impl Probe {
        fn high(&self) -> bool {
            self.high.load(Ordering::Relaxed)
        }
    }

    /// A probe on each of the four outputs, in [`PINS`] order.
    fn watch(d: &Ipcc) -> Vec<Arc<Probe>> {
        let ids = WireIdAllocator::new();
        PINS.iter()
            .map(|pin| {
                let id = ids.alloc();
                let probe = Arc::new(Probe::default());
                let wire = Wire::builder()
                    .source(id)
                    .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
                    .build_shared();
                Device::connect(d, pin, WireSource::new(wire, id)).expect("an IPCC output");
                probe
            })
            .collect()
    }

    fn levels(probes: &[Arc<Probe>]) -> [bool; 4] {
        [
            probes[0].high(),
            probes[1].high(),
            probes[2].high(),
            probes[3].high(),
        ]
    }

    // -----------------------------------------------------------------------
    // The register face
    // -----------------------------------------------------------------------

    #[test]
    fn reset_values_and_identification() {
        let d = Ipcc::build();
        assert_eq!(read(&d, C1CR), 0);
        assert_eq!(read(&d, C1MR), 0xffff_ffff, "every interrupt masked");
        assert_eq!(read(&d, C1TOC2SR), 0);
        assert_eq!(read(&d, C2CR), 0);
        assert_eq!(read(&d, C2MR), 0xffff_ffff);
        assert_eq!(read(&d, C2TOC1SR), 0);
        assert_eq!(read(&d, 0x3f0), 6, "six channels");
        assert_eq!(read(&d, 0x3f4), 0x10);
        assert_eq!(read(&d, 0x3f8), 0x0010_0071);
        assert_eq!(read(&d, 0x3fc), 0xa3c5_dd01);
        assert_eq!(read(&d, 0x20), 0, "reserved reads as zero");
    }

    #[test]
    fn control_and_mask_keep_only_their_bits() {
        let d = Ipcc::build();
        write(&d, C1CR, 0xffff_ffff);
        assert_eq!(read(&d, C1CR), CR_RXOIE | CR_TXFIE);
        write(&d, C2MR, 0);
        assert_eq!(read(&d, C2MR), !MR_MASK, "reserved bits stay at reset");
        write(&d, C2MR, low(3) | high(5));
        assert_eq!(read(&d, C2MR), !MR_MASK | low(3) | high(5));
        assert_eq!(read(&d, C1MR), 0xffff_ffff, "the other side is untouched");
    }

    #[test]
    fn set_and_clear_go_through_the_right_status_register() {
        let d = Ipcc::build();
        // CPU1 fills channels 1 and 6.
        write(&d, C1SCR, high(1) | high(6));
        assert_eq!(read(&d, C1TOC2SR), low(1) | low(6));
        assert_eq!(read(&d, C2TOC1SR), 0);
        // CPU1's own CHnC does not free its own channel: that is the other
        // side's to do.
        write(&d, C1SCR, low(1));
        assert_eq!(read(&d, C1TOC2SR), low(1) | low(6));
        // CPU2 takes channel 1.
        write(&d, C2SCR, low(1));
        assert_eq!(read(&d, C1TOC2SR), low(6));
        // CPU2 fills channel 2; CPU1 takes it.
        write(&d, C2SCR, high(2));
        assert_eq!(read(&d, C2TOC1SR), low(2));
        write(&d, C1SCR, low(2));
        assert_eq!(read(&d, C2TOC1SR), 0);
        // One write may carry both a set and a clear.
        write(&d, C2SCR, high(4) | low(6));
        assert_eq!(read(&d, C2TOC1SR), low(4));
        assert_eq!(read(&d, C1TOC2SR), 0);
        // Bits past channel 6 do nothing.
        write(&d, C1SCR, 0xffc0_ffc0);
        assert_eq!(read(&d, C1TOC2SR), 0);
    }

    #[test]
    fn the_scr_is_write_only_and_the_status_registers_read_only() {
        let d = Ipcc::build();
        write(&d, C1SCR, high(3));
        assert_eq!(read(&d, C1SCR), 0);
        assert_eq!(read(&d, C2SCR), 0);
        write(&d, C1TOC2SR, 0);
        write(&d, C2TOC1SR, 0x3f);
        assert_eq!(read(&d, C1TOC2SR), low(3));
        assert_eq!(read(&d, C2TOC1SR), 0);
        write(&d, 0x3f0, 0);
        assert_eq!(read(&d, 0x3f0), HWCFGR);
    }

    #[test]
    fn the_aperture_is_one_kilobyte() {
        let d = Ipcc::build();
        let mut buf = [0u8; 4];
        assert!(d.regs.read(0x400, &mut buf, MemAttrs::DEFAULT).is_err());
        assert!(d.regs.write(0x400, &buf, MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn debug_reads_match_and_debug_writes_are_refused() {
        let d = Ipcc::build();
        write(&d, C1SCR, high(2));
        assert_eq!(peek(&d, C1TOC2SR), read(&d, C1TOC2SR));
        assert_eq!(peek(&d, C1MR), 0xffff_ffff);
        let before = *d.regs.state.lock();
        let ring = high(5).to_le_bytes();
        assert!(d.regs.write(C1SCR, &ring, MemAttrs::DEBUG).is_err());
        assert_eq!(*d.regs.state.lock(), before, "no doorbell rang");
    }

    // -----------------------------------------------------------------------
    // The interrupts
    // -----------------------------------------------------------------------

    #[test]
    fn nothing_interrupts_out_of_reset() {
        let d = Ipcc::build();
        let probes = watch(&d);
        assert_eq!(levels(&probes), [false; 4]);
        // Enabled but every channel masked: still nothing.
        write(&d, C1CR, CR_RXOIE | CR_TXFIE);
        write(&d, C2CR, CR_RXOIE | CR_TXFIE);
        assert_eq!(levels(&probes), [false; 4]);
    }

    #[test]
    fn rx_occupied_follows_the_other_sides_flag_and_this_sides_mask() {
        let d = Ipcc::build();
        let probes = watch(&d);
        write(&d, C1CR, CR_RXOIE);
        write(&d, C1MR, !low(2));
        // CPU2 fills channel 1, which CPU1 has masked: no interrupt.
        write(&d, C2SCR, high(1));
        assert!(!probes[0].high());
        // Channel 2, which it has not.
        write(&d, C2SCR, high(2));
        assert!(probes[0].high(), "c1-rx");
        assert_eq!(d.level(Output::C1Rx), Level::High);
        // CPU1 takes channel 2 and the line drops, channel 1 still pending.
        write(&d, C1SCR, low(2));
        assert!(!probes[0].high());
        // Unmasking channel 1 raises it again.
        write(&d, C1MR, !(low(1) | low(2)));
        assert!(probes[0].high());
        // RXOIE off drops it.
        write(&d, C1CR, 0);
        assert!(!probes[0].high());
        // None of that reached CPU2's lines.
        assert!(!probes[2].high() && !probes[3].high());
    }

    #[test]
    fn tx_free_is_the_wpan_wait_for_the_other_side() {
        // ST's WPAN transport: fill a channel, then unmask its TX-free
        // interrupt and wait for CPU2 to take the message.
        let d = Ipcc::build();
        let probes = watch(&d);
        write(&d, C1CR, CR_RXOIE | CR_TXFIE);
        write(&d, C1SCR, high(4));
        write(&d, C1MR, !high(4));
        assert!(!probes[1].high(), "channel 4 is occupied");
        // CPU2 sees it on its RX line, once it unmasks channel 4.
        write(&d, C2CR, CR_RXOIE);
        write(&d, C2MR, !low(4));
        assert!(probes[2].high(), "c2-rx");
        // And frees it.
        write(&d, C2SCR, low(4));
        assert!(!probes[2].high());
        assert!(probes[1].high(), "c1-tx: channel 4 is free");
        // CPU1's handler masks it again.
        write(&d, C1MR, 0xffff_ffff);
        assert!(!probes[1].high());
    }

    #[test]
    fn processor_two_is_the_mirror_image() {
        let d = Ipcc::build();
        let probes = watch(&d);
        write(&d, C2CR, CR_TXFIE);
        write(&d, C2MR, !high(6));
        assert!(probes[3].high(), "c2-tx: channel 6 is free");
        write(&d, C2SCR, high(6));
        assert!(!probes[3].high());
        write(&d, C1SCR, low(6));
        assert!(probes[3].high());
        assert!(!probes[0].high() && !probes[1].high());
    }

    #[test]
    fn the_outputs_are_four_and_the_gate_adds_two_inputs() {
        let d = Ipcc::build();
        let id = WireId::new(9);
        let dummy = || WireSource::new(Wire::builder().source(id).build_shared(), id);
        assert!(Device::connect(&d, "irq", dummy()).is_err());
        for pin in PINS {
            assert_eq!(schema().port_named(pin).map(|p| p.dir), Some(PortDir::Out));
            assert!(Device::sink(&d, pin, &[id]).is_none(), "an output");
        }
        for pin in [gate::ENABLE_PIN, gate::RESET_PIN] {
            assert!(schema().port_named(pin).is_some());
            assert!(Device::sink(&d, pin, &[id]).is_some());
        }
    }

    // -----------------------------------------------------------------------
    // The clock gate and reset
    // -----------------------------------------------------------------------

    #[test]
    fn a_gated_block_reads_zero_and_drops_writes() {
        let d = Ipcc::build();
        write(&d, C1SCR, high(1));
        let id = WireId::new(1);
        let pin = Device::sink(&d, gate::ENABLE_PIN, &[id]).expect("`enable`");
        assert_eq!(read(&d, C1TOC2SR), 0);
        assert_eq!(read(&d, C1MR), 0);
        write(&d, C1SCR, high(2));
        pin.sink.set_level(id, 0, Level::High);
        assert_eq!(read(&d, C1TOC2SR), low(1), "kept, and the gated write lost");
    }

    #[test]
    fn the_reset_pin_and_device_reset_empty_every_channel() {
        let d = Ipcc::build();
        let probes = watch(&d);
        write(&d, C1CR, CR_TXFIE);
        write(&d, C1MR, 0);
        assert!(probes[1].high());
        write(&d, C1SCR, 0x3f << HIGH_SHIFT);
        let id = WireId::new(2);
        let pin = Device::sink(&d, gate::RESET_PIN, &[id]).expect("`reset`");
        pin.sink.set_level(id, 0, Level::High);
        pin.sink.set_level(id, 0, Level::Low);
        assert_eq!(*d.regs.state.lock(), State::default());
        assert_eq!(levels(&probes), [false; 4]);

        write(&d, C2SCR, high(3));
        Device::reset(&d, ResetKind::Warm);
        assert_eq!(d.c2_to_c1(), 0);
    }

    // -----------------------------------------------------------------------
    // Snapshots
    // -----------------------------------------------------------------------

    fn snapshot(d: &Ipcc) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("ipcc", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("ipcc", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(d, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    fn restore(d: &Ipcc, bytes: &[u8]) -> Result<()> {
        let reader = StateReader::new(bytes).unwrap();
        let chunk = reader
            .load("ipcc", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(d, &mut chunk.reader())
    }

    #[test]
    fn a_snapshot_round_trips_to_an_identical_state() {
        let saved = Ipcc::build();
        write(&saved, C1CR, CR_RXOIE | CR_TXFIE);
        write(&saved, C1MR, !(low(2) | high(1)));
        write(&saved, C2CR, CR_RXOIE);
        write(&saved, C2MR, !low(5));
        write(&saved, C1SCR, high(5));
        write(&saved, C2SCR, high(2));
        let bytes = snapshot(&saved);

        let restored = Ipcc::build();
        let probes = watch(&restored);
        restore(&restored, &bytes).unwrap();
        let before = *saved.regs.state.lock();
        let after = *restored.regs.state.lock();
        assert_eq!(after, before);
        for out in [Output::C1Rx, Output::C1Tx, Output::C2Rx, Output::C2Tx] {
            assert_eq!(restored.level(out), saved.level(out));
        }
        assert!(
            probes[0].high() && probes[2].high(),
            "restored levels driven"
        );
        assert_eq!(snapshot(&restored), bytes, "the state hash");
    }

    #[test]
    fn a_snapshot_with_bits_the_block_lacks_is_refused() {
        let d = Ipcc::build();
        let mut shape = MachineShape::new();
        shape.add_device("ipcc", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("ipcc", CLASS_NAME, STATE_VERSION).unwrap();
            for word in [0, MR_RESET, 1 << 7, 0, MR_RESET, 0] {
                chunk.write_u32(word).unwrap();
            }
        }
        let bytes = w.to_vec().unwrap();
        assert!(restore(&d, &bytes).is_err(), "a seventh channel");
    }

    #[test]
    fn the_class_takes_no_properties() {
        assert!(Ipcc::new(&Props::new()).is_ok());
        assert!(Ipcc::new(&Props::new().with("clock", 64_000_000u64)).is_ok());
        assert!(Ipcc::new(&Props::new().with("channels", 6u64)).is_err());
    }
}
