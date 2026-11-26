//! The STM32WB hardware semaphore.
//!
//! One class, `st.hsem`: thirty-two locks that both of the WB's cores — the
//! application Cortex-M4 (CPU1) and the radio Cortex-M0+ (CPU2) — can take
//! and give back atomically, plus an interrupt that tells a core a semaphore
//! it was waiting on has come free. ST's WB firmware takes them around
//! everything the two cores share: the RCC clock switch, the flash
//! controller, the RNG, the PKA. A board without one is a `HAL_Init` that
//! spins on its first `LL_HSEM_1StepLock`.
//!
//! # The register map
//!
//! Offsets as ST CMSIS `stm32wb55xx.h` gives them (`HSEM_TypeDef`,
//! `HSEM_Common_TypeDef`), the identification block from ST's
//! `STM32WB55_CM4.svd`:
//!
//! | Offset | Register | What it does |
//! | --- | --- | --- |
//! | `0x000 + 4n` | `Rn` | the semaphore, and its **two-step** lock: write `LOCK` with a `COREID` and `PROCID`, then read back to see who won |
//! | `0x080 + 4n` | `RLRn` | the **one-step** lock: a read takes the semaphore if it is free, and answers who holds it |
//! | `0x100` / `0x110` | `C1IER` / `C2IER` | which semaphores' freeing interrupts this core |
//! | `0x104` / `0x114` | `C1ICR` / `C2ICR` | write one to clear the status bit; reads zero |
//! | `0x108` / `0x118` | `C1ISR` / `C2ISR` | read-only: semaphores freed while enabled |
//! | `0x10C` / `0x11C` | `C1MISR` / `C2MISR` | read-only: `ISR & IER` |
//! | `0x140` | `CR` | write `KEY` and a `COREID`: frees every semaphore that core holds, if `KEY` matches `KEYR` |
//! | `0x144` | `KEYR` | the clear key, bits 31:16 |
//! | `0x3EC` | `HWCFGR2` | `0x0000_0084`: the valid bus-master ids, 4 and 8 |
//! | `0x3F0` | `HWCFGR1` | `0x0000_0220`: two interrupt lines, thirty-two semaphores |
//! | `0x3F4` | `VERR` | `0x0000_0020`: revision 2.0 |
//! | `0x3F8` | `IPIDR` | `0x0010_0072` |
//! | `0x3FC` | `SIDR` | `0xA3C5_DD01` |
//!
//! `Rn` holds `PROCID` in bits 7:0, `COREID` in 11:8 and `LOCK` at 31. A free
//! semaphore reads as zero. Every other offset in the kilobyte reads as zero
//! and ignores writes.
//!
//! # Who is asking
//!
//! The `COREID` the hardware records is the AHB bus master's: **4 for CPU1
//! and 8 for CPU2** (`HSEM_CR_COREID_CPU1` / `_CPU2` in `stm32wb55xx.h`, and
//! the `MASTERID1`/`MASTERID2` fields of `HWCFGR2`). Here that comes from
//! [`MemAttrs::requester`]: an access whose requester is the `cpu2-requester`
//! property is CPU2's, and **every other access is CPU1's** — the anonymous
//! requester, a DMA, a debugger. With the property unset, which is the
//! default, every access is CPU1's. That is the known deviation, and it costs
//! nothing on today's WB board: its CPU2 model does not use the semaphores.
//!
//! # The rules
//!
//! - **Two-step lock.** A write to `Rn` with `LOCK` set takes a free
//!   semaphore for the written `COREID` and `PROCID` — but only when the
//!   written `COREID` is the writer's own; a core cannot lock on another's
//!   behalf. A write to a semaphore already held changes nothing, by its
//!   holder or anybody else, which is why the protocol reads back.
//! - **One-step lock.** A read of `RLRn` takes a free semaphore for the
//!   reader's `COREID` with `PROCID` zero and answers the new value; on a
//!   held one it answers the holder and changes nothing. A **debug** read
//!   (`MemAttrs::debug`) never takes it: a debugger's memory window over the
//!   block would otherwise lock all thirty-two. Writes to `RLRn` do nothing.
//! - **Unlock.** A write to `Rn` with `LOCK` clear frees the semaphore when
//!   the written `COREID` and `PROCID` are the holder's and the `COREID` is
//!   the writer's own. Anything else is ignored.
//! - **Clear by key.** A write to `CR` whose `KEY` equals `KEYR.KEY` frees
//!   every semaphore held by the written `COREID`, whoever writes it — that is
//!   what one core does after resetting the other.
//! - **The free event.** Freeing semaphore *n*, by either route, sets bit *n*
//!   of `CxISR` for each core whose `CxIER` has bit *n* set at that moment.
//!   `CxMISR` is `CxISR & CxIER`, and the core's interrupt is high while
//!   `CxMISR` is nonzero. Clearing an `IER` bit masks a pending status rather
//!   than discarding it.
//!
//! On an STM32WB55 `c1-irq` is NVIC line 46 (`HSEM_IRQn`); `c2-irq` goes to
//! CPU2. The board file draws those wires.
//!
//! # The clock
//!
//! `enable` is `RCC_AHB3ENR.HSEMEN`, bit 19, and `reset` is
//! `RCC_AHB3RSTR.HSEMRST`, the same bit (`stm32wb55xx.h`); CPU2's
//! `RCC_C2AHB3ENR.HSEMEN` can be wired to `enable` too, which ORs its inputs.
//! [`gate`] has what a gated block does.
//!
//! # Sources
//!
//! ST **RM0434**, the "Hardware semaphore (HSEM)" chapter, for the lock,
//! unlock, clear and interrupt rules; ST CMSIS `stm32wb55xx.h` (Apache-2.0)
//! for the offsets, the field positions, the core ids and the interrupt
//! number; ST `STM32WB55_CM4.svd` (Apache-2.0) for the identification values.
//! ST's BSD-3 WB HAL (`stm32wbxx_hal_hsem.c`) and LL (`stm32wbxx_ll_hsem.h`)
//! were read for what firmware writes and compares, not copied. No emulator
//! source of any licence was consulted (`ROADMAP.md` §1).

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{
    AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef, RequesterId,
};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{Level, WireId, WireSource};
use crate::dev::stm32::gate::{self, ClockGate, Gated};
use crate::machine::Instance;
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name a machine description writes.
const CLASS_NAME: &str = "st.hsem";

/// The snapshot chunk version. Bump it with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// CPU1's interrupt (`HSEM_IRQn`, NVIC 46 on a WB55).
pub const C1_IRQ_PIN: &str = "c1-irq";
/// CPU2's interrupt.
pub const C2_IRQ_PIN: &str = "c2-irq";

/// The two outputs, by core.
const PINS: [&str; 2] = [C1_IRQ_PIN, C2_IRQ_PIN];

/// The aperture: one kilobyte, the identification block at its top.
pub const REGISTER_BYTES: u64 = 0x400;

/// How many semaphores (`HWCFGR1.NBSEM`).
pub const SEMAPHORES: usize = 32;

/// CPU1's `COREID`, its AHB bus-master id.
pub const COREID_CPU1: u32 = 0x4;
/// CPU2's.
pub const COREID_CPU2: u32 = 0x8;

/// `Rn.LOCK`, bit 31.
const R_LOCK: u32 = 1 << 31;
/// `Rn.COREID`, bits 11:8.
const COREID_SHIFT: u32 = 8;
const COREID_MASK: u32 = 0xf;
/// `Rn.PROCID`, bits 7:0.
const PROCID_MASK: u32 = 0xff;
/// Every bit `Rn` implements.
const R_MASK: u32 = R_LOCK | (COREID_MASK << COREID_SHIFT) | PROCID_MASK;

/// `KEYR.KEY` and `CR.KEY`, bits 31:16.
const KEY_MASK: u32 = 0xffff_0000;

/// `HWCFGR2`: `MASTERID1 = 4`, `MASTERID2 = 8`.
const HWCFGR2: u32 = 0x0000_0084;
/// `HWCFGR1`: `NBINT = 2`, `NBSEM = 32`.
const HWCFGR1: u32 = 0x0000_0220;
/// `VERR`: `MAJREV = 2`, `MINREV = 0`.
const VERR: u32 = 0x0000_0020;
/// `IPIDR`.
const IPIDR: u32 = 0x0010_0072;
/// `SIDR`.
const SIDR: u32 = 0xa3c5_dd01;

/// The semaphore's register file. `[0]` of each pair is CPU1's, `[1]` CPU2's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct State {
    sem: [u32; SEMAPHORES],
    ier: [u32; 2],
    isr: [u32; 2],
    keyr: u32,
}

impl State {
    /// Core `c`'s masked status, which is also its interrupt.
    const fn misr(&self, c: usize) -> u32 {
        self.isr[c] & self.ier[c]
    }

    /// Free semaphore `n` and raise its event for whichever core wants it.
    fn free(&mut self, n: usize) {
        self.sem[n] = 0;
        let bit = 1u32 << n;
        for c in 0..2 {
            if self.ier[c] & bit != 0 {
                self.isr[c] |= bit;
            }
        }
    }

    /// A one-step lock attempt by `core`: what `RLRn` answers.
    fn read_lock(&mut self, n: usize, core: u32) -> u32 {
        if self.sem[n] & R_LOCK == 0 {
            self.sem[n] = R_LOCK | (core << COREID_SHIFT);
        }
        self.sem[n]
    }

    /// A write of `value` to `Rn` by `core`.
    fn write_sem(&mut self, n: usize, value: u32, core: u32) {
        let coreid = (value >> COREID_SHIFT) & COREID_MASK;
        if coreid != core {
            // Not the writer's own id: neither a lock nor an unlock.
            return;
        }
        let held = self.sem[n];
        if value & R_LOCK != 0 {
            if held & R_LOCK == 0 {
                self.sem[n] = value & R_MASK;
            }
        } else if held & R_LOCK != 0 && held & !R_LOCK == value & (R_MASK & !R_LOCK) {
            self.free(n);
        }
    }

    /// A write of `value` to `CR`: clear every semaphore of `COREID`, if the
    /// key is right.
    fn clear_by_key(&mut self, value: u32) {
        if value & KEY_MASK != self.keyr & KEY_MASK {
            return;
        }
        let coreid = (value >> COREID_SHIFT) & COREID_MASK;
        for n in 0..SEMAPHORES {
            let held = self.sem[n];
            if held & R_LOCK != 0 && (held >> COREID_SHIFT) & COREID_MASK == coreid {
                self.free(n);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// The register block
// ---------------------------------------------------------------------------

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// The requester id that is CPU2, if the board named one.
    cpu2: Option<RequesterId>,
    /// The two interrupt outputs, by core, taken at realize.
    irq: Mutex<[Option<WireSource>; 2]>,
    /// `RCC_AHB3ENR.HSEMEN` and `RCC_AHB3RSTR.HSEMRST`, as a board's wires
    /// deliver them.
    gate: ClockGate,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("cpu2", &self.cpu2);
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state),
            None => s.field("state", &"<locked>"),
        };
        s.finish()
    }
}

impl Registers {
    /// The `COREID` of whoever made this access.
    fn core_of(&self, attrs: MemAttrs) -> u32 {
        if self.cpu2 == Some(attrs.requester) {
            COREID_CPU2
        } else {
            COREID_CPU1
        }
    }

    /// Drive both outputs to what the status says.
    ///
    /// Called with **no lock of this device held**, as the re-entrancy
    /// contract requires.
    fn refresh_irq(&self) {
        let levels = {
            let state = self.state.lock();
            [state.misr(0) != 0, state.misr(1) != 0]
        };
        let sources = self.irq.lock().clone();
        for (source, high) in sources.iter().zip(levels) {
            if let Some(source) = source {
                source.set(Level::from_bool(high));
            }
        }
    }

    /// Read one register. Only `RLRn` has a side effect, and only when
    /// `debug` is false.
    fn read_register(&self, offset: u64, attrs: MemAttrs) -> u32 {
        let mut state = self.state.lock();
        let n = (offset / 4) as usize % SEMAPHORES;
        match offset {
            0x000..=0x07c => state.sem[n],
            0x080..=0x0fc if attrs.debug => state.sem[n],
            0x080..=0x0fc => {
                let core = self.core_of(attrs);
                state.read_lock(n, core)
            }
            0x100 => state.ier[0],
            0x108 => state.isr[0],
            0x10c => state.misr(0),
            0x110 => state.ier[1],
            0x118 => state.isr[1],
            0x11c => state.misr(1),
            0x144 => state.keyr,
            0x3ec => HWCFGR2,
            0x3f0 => HWCFGR1,
            0x3f4 => VERR,
            0x3f8 => IPIDR,
            0x3fc => SIDR,
            // `CxICR` and `CR` read as zero, and the rest is reserved.
            _ => 0,
        }
    }

    /// Write one register.
    fn write_register(&self, offset: u64, value: u32, attrs: MemAttrs) {
        let mut state = self.state.lock();
        match offset {
            0x000..=0x07c => {
                let core = self.core_of(attrs);
                state.write_sem((offset / 4) as usize, value, core);
            }
            0x100 => state.ier[0] = value,
            0x104 => state.isr[0] &= !value,
            0x110 => state.ier[1] = value,
            0x114 => state.isr[1] &= !value,
            0x140 => state.clear_by_key(value),
            0x144 => state.keyr = value & KEY_MASK,
            // `RLRn`, the status registers and the identification block are
            // read-only; everything else is reserved.
            _ => {}
        }
    }
}

impl Gated for Registers {
    fn clock_gate(&self) -> &ClockGate {
        &self.gate
    }

    fn gate_reset(&self) {
        // `RCC_AHB3RSTR.HSEMRST`: every semaphore free, every interrupt off —
        // what `Device::reset` does. No free *event*: a reset is not an
        // unlock, and the enables it would be checked against reset too.
        *self.state.lock() = State::default();
        self.refresh_irq();
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if offset >= REGISTER_BYTES {
            return Err(BusError::BadAccess);
        }
        if !self.gate.live() {
            // No `HSEMEN`, or `HSEMRST` standing: the block reads as zero
            // ([`gate`](super::gate)) and an `RLRn` read takes nothing.
            dst.fill(0);
            return Ok(());
        }
        let bytes = self.read_register(offset, attrs).to_le_bytes();
        for (slot, byte) in dst.iter_mut().zip(bytes) {
            *slot = byte;
        }
        // A one-step lock changes no interrupt — only a free does — so there
        // is nothing to drive after a read.
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if offset >= REGISTER_BYTES {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // A debug write could take or free a lock one of the cores is
            // relying on. There is no harmless version.
            return Err(BusError::BadAccess);
        }
        if !self.gate.live() {
            // Not effective: nothing is locked or freed at an unclocked block.
            return Ok(());
        }
        let mut word = [0u8; 4];
        for (slot, byte) in word.iter_mut().zip(src) {
            *slot = *byte;
        }
        self.write_register(offset, u32::from_le_bytes(word), attrs);
        // Outside the critical section, as the re-entrancy contract requires.
        self.refresh_irq();
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Word access: ST's HAL and LL touch these registers no other way, and
        // a byte write of `Rn` that carried `LOCK` without `COREID` would be a
        // lock nobody could release.
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// The device
// ---------------------------------------------------------------------------

/// An STM32WB hardware semaphore block.
#[derive(Debug)]
pub struct Hsem {
    regs: Arc<Registers>,
    region: RegionRef,
}

impl Hsem {
    /// Validate `props` and build the block.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property is of the wrong kind or value, or if
    /// one this class does not know was given.
    pub fn new(props: &Props) -> Result<Hsem> {
        let mut r = props.reader();
        let cpu2 = r.optional::<u64>("cpu2-requester")?;
        r.touch("clock");
        r.finish()?;
        let cpu2 = match cpu2 {
            None => None,
            Some(id) => Some(RequesterId(u32::try_from(id).map_err(|_| {
                Error::Property(format!(
                    "`cpu2-requester` is a 32-bit requester id, not {id}"
                ))
            })?)),
        };
        Ok(Hsem::build(cpu2))
    }

    /// Build one directly — the route a test takes. `cpu2` is the requester
    /// whose accesses carry `COREID` 8; every other one is CPU1's.
    #[must_use]
    pub fn build(cpu2: Option<RequesterId>) -> Hsem {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::default()),
            cpu2,
            irq: Mutex::with_rank(LockRank::WIRE, [None, None]),
            gate: ClockGate::new(),
        });
        let region = Arc::new(Region::io(
            "hsem",
            REGISTER_BYTES,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Hsem { regs, region }
    }

    /// Semaphore `n`'s value, as `Rn` reads it. Takes nothing.
    ///
    /// # Panics
    ///
    /// If `n` is not below [`SEMAPHORES`].
    #[must_use]
    pub fn semaphore(&self, n: usize) -> u32 {
        self.regs.state.lock().sem[n]
    }

    /// The level CPU1's (`cpu2 == false`) or CPU2's interrupt is driven to.
    #[must_use]
    pub fn irq_level(&self, cpu2: bool) -> Level {
        Level::from_bool(self.regs.state.lock().misr(usize::from(cpu2)) != 0)
    }
}

impl Device for Hsem {
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
        for sem in state.sem {
            w.write_u32(sem)?;
        }
        for c in 0..2 {
            w.write_u32(state.ier[c])?;
            w.write_u32(state.isr[c])?;
        }
        w.write_u32(state.keyr)
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let mut state = State::default();
        for (n, slot) in state.sem.iter_mut().enumerate() {
            let sem = r.read_u32()?;
            // A free semaphore reads as zero, and a held one has only the
            // fields `Rn` implements.
            if sem & !R_MASK != 0 || (sem & R_LOCK == 0 && sem != 0) {
                return Err(Error::State(format!(
                    "snapshot has HSEM semaphore {n} at {sem:#010x}, which no lock produces"
                )));
            }
            *slot = sem;
        }
        for c in 0..2 {
            state.ier[c] = r.read_u32()?;
            state.isr[c] = r.read_u32()?;
        }
        state.keyr = r.read_u32()?;
        if state.keyr & !KEY_MASK != 0 {
            return Err(Error::State(format!(
                "snapshot has HSEM KEYR at {:#010x}, below bit 16",
                state.keyr
            )));
        }
        *self.regs.state.lock() = state;
        self.regs.refresh_irq();
        Ok(())
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let Some(c) = PINS.iter().position(|p| *p == port) else {
            return Err(Error::Config {
                at: String::from(port),
                message: format!("an HSEM drives `{C1_IRQ_PIN}` and `{C2_IRQ_PIN}`"),
            });
        };
        self.regs.irq.lock()[c] = Some(source);
        self.regs.refresh_irq();
        Ok(())
    }

    fn announce(&self, port: &str) {
        if PINS.contains(&port) {
            self.regs.refresh_irq();
        }
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        // `enable` and `reset`; both interrupts are outputs.
        gate::sink(&self.regs, port, sources)
    }
}

impl Instance for Hsem {}

/// The `st.hsem` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "STM32WB hardware semaphore: 32 locks shared by CPU1 and CPU2, and a free interrupt",
    properties: &[PropertySpec {
        name: "cpu2-requester",
        kind: ValueKind::Uint,
        required: false,
        summary: "the bus requester id that is CPU2 (COREID 8); unset, every access is CPU1's \
                  (COREID 4)",
    }],
    construct: |props| Ok(Box::new(Hsem::new(props)?)),
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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Hsem::new(props)?)))
}

/// What the validator should know about `st.hsem`.
#[must_use]
pub fn schema() -> ClassSchema {
    let schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("cpu2-requester", ValueKind::Uint))
        .region("")
        .region("regs")
        .port(C1_IRQ_PIN, PortDir::Out)
        .port(C2_IRQ_PIN, PortDir::Out);
    // `RCC_AHB3ENR.HSEMEN` and `RCC_AHB3RSTR.HSEMRST`.
    gate::ports(schema)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::sync::{AtomicBool, Ordering};
    use crate::core::wire::{Wire, WireIdAllocator, WireSink};
    use alloc::vec::Vec;

    const C1IER: u64 = 0x100;
    const C1ICR: u64 = 0x104;
    const C1ISR: u64 = 0x108;
    const C1MISR: u64 = 0x10c;
    const C2IER: u64 = 0x110;
    const C2ICR: u64 = 0x114;
    const C2ISR: u64 = 0x118;
    const C2MISR: u64 = 0x11c;
    const CR: u64 = 0x140;
    const KEYR: u64 = 0x144;

    /// The requester the tests make CPU2.
    const CPU2: RequesterId = RequesterId(2);

    const fn r(n: u64) -> u64 {
        4 * n
    }

    const fn rlr(n: u64) -> u64 {
        0x80 + 4 * n
    }

    /// `Rn`'s value for a lock by `core` with `procid`.
    const fn locked(core: u32, procid: u32) -> u32 {
        R_LOCK | (core << COREID_SHIFT) | procid
    }

    fn hsem() -> Hsem {
        Hsem::build(Some(CPU2))
    }

    fn attrs(cpu2: bool) -> MemAttrs {
        if cpu2 {
            MemAttrs::DEFAULT.with_requester(CPU2)
        } else {
            MemAttrs::DEFAULT
        }
    }

    fn read_as(d: &Hsem, cpu2: bool, offset: u64) -> u32 {
        let mut buf = [0u8; 4];
        d.regs
            .read(offset, &mut buf, attrs(cpu2))
            .expect("a word read is legal");
        u32::from_le_bytes(buf)
    }

    fn write_as(d: &Hsem, cpu2: bool, offset: u64, value: u32) {
        d.regs
            .write(offset, &value.to_le_bytes(), attrs(cpu2))
            .expect("a word write is legal");
    }

    fn read(d: &Hsem, offset: u64) -> u32 {
        read_as(d, false, offset)
    }

    fn write(d: &Hsem, offset: u64, value: u32) {
        write_as(d, false, offset, value);
    }

    fn peek(d: &Hsem, offset: u64) -> u32 {
        let mut buf = [0u8; 4];
        d.regs
            .read(offset, &mut buf, MemAttrs::DEBUG)
            .expect("a debug word read is legal");
        u32::from_le_bytes(buf)
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

    fn watch(d: &Hsem) -> Vec<Arc<Probe>> {
        let ids = WireIdAllocator::new();
        PINS.iter()
            .map(|pin| {
                let id = ids.alloc();
                let probe = Arc::new(Probe::default());
                let wire = Wire::builder()
                    .source(id)
                    .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
                    .build_shared();
                Device::connect(d, pin, WireSource::new(wire, id)).expect("an HSEM output");
                probe
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // The register face
    // -----------------------------------------------------------------------

    #[test]
    fn reset_values_and_identification() {
        let d = hsem();
        for n in 0..32 {
            assert_eq!(read(&d, r(n)), 0, "R{n} free");
        }
        for offset in [C1IER, C1ISR, C1MISR, C2IER, C2ISR, C2MISR, CR, KEYR] {
            assert_eq!(read(&d, offset), 0);
        }
        assert_eq!(read(&d, 0x3ec), 0x84, "MASTERID 4 and 8");
        assert_eq!(read(&d, 0x3f0), 0x220, "two interrupts, 32 semaphores");
        assert_eq!(read(&d, 0x3f4), 0x20);
        assert_eq!(read(&d, 0x3f8), 0x0010_0072);
        assert_eq!(read(&d, 0x3fc), 0xa3c5_dd01);
        let mut buf = [0u8; 4];
        assert!(d.regs.read(0x400, &mut buf, MemAttrs::DEFAULT).is_err());
    }

    // -----------------------------------------------------------------------
    // Locking
    // -----------------------------------------------------------------------

    #[test]
    fn the_two_step_lock_as_hal_hsem_take_uses_it() {
        // `HAL_HSEM_Take`: write PROCID | COREID_CURRENT | LOCK, read it back.
        let d = hsem();
        write(&d, r(5), locked(COREID_CPU1, 0x12));
        assert_eq!(read(&d, r(5)), locked(COREID_CPU1, 0x12), "taken");
        // Another process on the same core loses, and so does CPU2.
        write(&d, r(5), locked(COREID_CPU1, 0x34));
        assert_eq!(read(&d, r(5)), locked(COREID_CPU1, 0x12));
        write_as(&d, true, r(5), locked(COREID_CPU2, 0x12));
        assert_eq!(read_as(&d, true, r(5)), locked(COREID_CPU1, 0x12));
        // Reserved bits in the write are not stored.
        write(&d, r(6), locked(COREID_CPU1, 1) | 0x7000_f000);
        assert_eq!(read(&d, r(6)), locked(COREID_CPU1, 1));
    }

    #[test]
    fn a_core_cannot_lock_or_unlock_with_another_cores_id() {
        let d = hsem();
        write(&d, r(1), locked(COREID_CPU2, 0));
        assert_eq!(read(&d, r(1)), 0, "CPU1 writing CPU2's id takes nothing");
        write_as(&d, true, r(1), locked(COREID_CPU2, 7));
        assert_eq!(read(&d, r(1)), locked(COREID_CPU2, 7), "CPU2 itself can");
        write(&d, r(1), (COREID_CPU2 << COREID_SHIFT) | 7);
        assert_eq!(
            read(&d, r(1)),
            locked(COREID_CPU2, 7),
            "nor free it for CPU2"
        );
        write_as(&d, true, r(1), (COREID_CPU2 << COREID_SHIFT) | 7);
        assert_eq!(read(&d, r(1)), 0, "but CPU2 can");
    }

    #[test]
    fn unlock_needs_the_holders_coreid_and_procid() {
        let d = hsem();
        write(&d, r(9), locked(COREID_CPU1, 3));
        write(&d, r(9), (COREID_CPU1 << COREID_SHIFT) | 4);
        assert_eq!(read(&d, r(9)), locked(COREID_CPU1, 3), "wrong PROCID");
        // `HAL_HSEM_Release`: PROCID | COREID_CURRENT, no LOCK.
        write(&d, r(9), (COREID_CPU1 << COREID_SHIFT) | 3);
        assert_eq!(read(&d, r(9)), 0);
        // Releasing a free semaphore changes nothing and raises nothing.
        write(&d, C1IER, 1 << 9);
        write(&d, r(9), COREID_CPU1 << COREID_SHIFT);
        assert_eq!(read(&d, C1ISR), 0);
    }

    #[test]
    fn the_one_step_lock_as_ll_hsem_1steplock_uses_it() {
        // `LL_HSEM_1StepLock`: success is RLR reading LOCK | own COREID.
        let d = hsem();
        assert_eq!(read(&d, rlr(3)), locked(COREID_CPU1, 0), "taken");
        assert_eq!(read(&d, r(3)), locked(COREID_CPU1, 0));
        // CPU2's attempt answers CPU1, which is the failure it compares for.
        assert_eq!(read_as(&d, true, rlr(3)), locked(COREID_CPU1, 0));
        // CPU2 takes a free one with its own id.
        assert_eq!(read_as(&d, true, rlr(4)), locked(COREID_CPU2, 0));
        // RLRn is read-only.
        write(&d, rlr(4), 0);
        assert_eq!(read(&d, r(4)), locked(COREID_CPU2, 0));
        // And a one-step lock is freed like any other, PROCID zero.
        write(&d, r(3), COREID_CPU1 << COREID_SHIFT);
        assert_eq!(read(&d, r(3)), 0);
    }

    #[test]
    fn a_debug_read_of_rlr_takes_nothing() {
        let d = hsem();
        for n in 0..32 {
            assert_eq!(peek(&d, rlr(n)), 0);
            assert_eq!(peek(&d, r(n)), 0);
        }
        assert_eq!(d.semaphore(0), 0, "a memory window locked nothing");
        // Over a held one it shows the holder, as RLR does.
        write(&d, r(2), locked(COREID_CPU1, 9));
        assert_eq!(peek(&d, rlr(2)), locked(COREID_CPU1, 9));
        // And a debug write is refused outright.
        let buf = locked(COREID_CPU1, 1).to_le_bytes();
        assert!(d.regs.write(r(0), &buf, MemAttrs::DEBUG).is_err());
        assert_eq!(d.semaphore(0), 0);
    }

    #[test]
    fn without_cpu2_requester_every_access_is_cpu1() {
        let d = Hsem::build(None);
        assert_eq!(read_as(&d, true, rlr(0)), locked(COREID_CPU1, 0));
    }

    // -----------------------------------------------------------------------
    // Clear by key
    // -----------------------------------------------------------------------

    #[test]
    fn cr_clears_one_cores_semaphores_when_the_key_matches() {
        let d = hsem();
        write(&d, r(0), locked(COREID_CPU1, 1));
        write(&d, r(1), locked(COREID_CPU1, 2));
        write_as(&d, true, r(2), locked(COREID_CPU2, 3));
        write(&d, KEYR, 0xbeef_1234);
        assert_eq!(read(&d, KEYR), 0xbeef_0000, "KEY only");
        // The wrong key does nothing.
        write(&d, CR, 0xbeee_0000 | (COREID_CPU1 << COREID_SHIFT));
        assert_eq!(read(&d, r(0)), locked(COREID_CPU1, 1));
        // CPU1 clears CPU2's — the after-a-reset case.
        write(&d, CR, 0xbeef_0000 | (COREID_CPU2 << COREID_SHIFT));
        assert_eq!(read(&d, r(2)), 0);
        assert_eq!(read(&d, r(0)), locked(COREID_CPU1, 1), "CPU1's kept");
        write(&d, CR, 0xbeef_0000 | (COREID_CPU1 << COREID_SHIFT));
        assert_eq!(read(&d, r(0)), 0);
        assert_eq!(read(&d, r(1)), 0);
        assert_eq!(read(&d, CR), 0, "CR reads as zero");
    }

    #[test]
    fn a_zero_key_matches_the_reset_keyr() {
        let d = hsem();
        write(&d, r(7), locked(COREID_CPU1, 0));
        write(&d, CR, COREID_CPU1 << COREID_SHIFT);
        assert_eq!(read(&d, r(7)), 0);
    }

    // -----------------------------------------------------------------------
    // The free interrupt
    // -----------------------------------------------------------------------

    #[test]
    fn freeing_sets_status_for_the_cores_that_enabled_it() {
        let d = hsem();
        let probes = watch(&d);
        write(&d, r(4), locked(COREID_CPU1, 0));
        write(&d, r(5), locked(COREID_CPU1, 0));
        // `HAL_HSEM_ActivateNotification` on CPU2 for 4, on CPU1 for 5.
        write_as(&d, true, C2IER, 1 << 4);
        write(&d, C1IER, 1 << 5);
        write(&d, r(4), COREID_CPU1 << COREID_SHIFT);
        assert_eq!(read(&d, C2ISR), 1 << 4);
        assert_eq!(read(&d, C1ISR), 0, "CPU1 did not ask about 4");
        assert!(probes[1].high() && !probes[0].high());
        write(&d, r(5), COREID_CPU1 << COREID_SHIFT);
        assert_eq!(read(&d, C1MISR), 1 << 5);
        assert!(probes[0].high(), "c1-irq");
        assert_eq!(d.irq_level(false), Level::High);

        // `HAL_HSEM_IRQHandler`: read MISR, drop the enables, clear the status.
        let misr = read(&d, C1MISR);
        write(&d, C1IER, read(&d, C1IER) & !misr);
        assert!(!probes[0].high(), "masked");
        assert_eq!(read(&d, C1ISR), 1 << 5, "the status is kept, only masked");
        write(&d, C1ICR, misr);
        assert_eq!(read(&d, C1ISR), 0);
        assert_eq!(read(&d, C1ICR), 0);
        write_as(&d, true, C2ICR, 1 << 4);
        assert!(!probes[1].high());
    }

    #[test]
    fn the_status_only_sets_if_enabled_when_freed() {
        let d = hsem();
        write(&d, r(8), locked(COREID_CPU1, 0));
        write(&d, r(8), COREID_CPU1 << COREID_SHIFT);
        write(&d, C1IER, 1 << 8);
        assert_eq!(read(&d, C1ISR), 0, "the free came before the enable");
    }

    #[test]
    fn a_clear_by_key_raises_the_free_event_too() {
        let d = hsem();
        let probes = watch(&d);
        write(&d, C1IER, 0b11);
        write_as(&d, true, r(0), locked(COREID_CPU2, 0));
        write_as(&d, true, r(1), locked(COREID_CPU2, 0));
        write(&d, CR, COREID_CPU2 << COREID_SHIFT);
        assert_eq!(read(&d, C1ISR), 0b11);
        assert!(probes[0].high());
    }

    #[test]
    fn the_outputs_are_two_and_the_gate_adds_two_inputs() {
        let d = hsem();
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
    fn a_gated_block_reads_zero_takes_nothing_and_drops_writes() {
        let d = hsem();
        write(&d, r(0), locked(COREID_CPU1, 1));
        let id = WireId::new(1);
        let pin = Device::sink(&d, gate::ENABLE_PIN, &[id]).expect("`enable`");
        assert_eq!(read(&d, r(0)), 0);
        assert_eq!(read(&d, rlr(1)), 0);
        write(&d, r(0), COREID_CPU1 << COREID_SHIFT);
        pin.sink.set_level(id, 0, Level::High);
        assert_eq!(read(&d, r(0)), locked(COREID_CPU1, 1), "kept");
        assert_eq!(d.semaphore(1), 0, "the gated RLR read took nothing");
    }

    #[test]
    fn the_reset_pin_and_device_reset_free_everything_silently() {
        let d = hsem();
        let probes = watch(&d);
        write(&d, C1IER, u32::MAX);
        write(&d, r(3), locked(COREID_CPU1, 0));
        let id = WireId::new(2);
        let pin = Device::sink(&d, gate::RESET_PIN, &[id]).expect("`reset`");
        pin.sink.set_level(id, 0, Level::High);
        pin.sink.set_level(id, 0, Level::Low);
        assert_eq!(*d.regs.state.lock(), State::default());
        assert!(!probes[0].high());

        read(&d, rlr(6));
        Device::reset(&d, ResetKind::Cold);
        assert_eq!(d.semaphore(6), 0);
    }

    // -----------------------------------------------------------------------
    // Snapshots and properties
    // -----------------------------------------------------------------------

    fn snapshot(d: &Hsem) -> Vec<u8> {
        let mut shape = MachineShape::new();
        shape.add_device("hsem", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("hsem", CLASS_NAME, STATE_VERSION).unwrap();
            Device::save(d, &mut chunk).unwrap();
        }
        w.to_vec().unwrap()
    }

    fn restore(d: &Hsem, bytes: &[u8]) -> Result<()> {
        let reader = StateReader::new(bytes).unwrap();
        let chunk = reader
            .load("hsem", CLASS_NAME, STATE_VERSION, &Migrations::new())
            .unwrap();
        Device::load(d, &mut chunk.reader())
    }

    #[test]
    fn a_snapshot_round_trips_to_an_identical_state() {
        let saved = hsem();
        write(&saved, r(0), locked(COREID_CPU1, 0x55));
        read_as(&saved, true, rlr(31));
        write(&saved, C1IER, 1 << 2);
        write(&saved, r(2), locked(COREID_CPU1, 0));
        write(&saved, r(2), COREID_CPU1 << COREID_SHIFT);
        write_as(&saved, true, C2IER, 0xf0);
        write(&saved, KEYR, 0x1234_0000);
        let bytes = snapshot(&saved);

        let restored = hsem();
        let probes = watch(&restored);
        restore(&restored, &bytes).unwrap();
        let before = *saved.regs.state.lock();
        let after = *restored.regs.state.lock();
        assert_eq!(after, before);
        assert!(probes[0].high(), "the pending free survives and is driven");
        assert_eq!(restored.irq_level(true), saved.irq_level(true));
        assert_eq!(snapshot(&restored), bytes, "the state hash");
    }

    #[test]
    fn a_snapshot_no_lock_could_produce_is_refused() {
        let d = hsem();
        let mut shape = MachineShape::new();
        shape.add_device("hsem", CLASS_NAME).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("hsem", CLASS_NAME, STATE_VERSION).unwrap();
            // A free semaphore with an owner.
            chunk.write_u32(COREID_CPU1 << COREID_SHIFT).unwrap();
            for _ in 1..SEMAPHORES + 5 {
                chunk.write_u32(0).unwrap();
            }
        }
        let bytes = w.to_vec().unwrap();
        assert!(restore(&d, &bytes).is_err());
    }

    #[test]
    fn the_class_takes_cpu2_requester_and_nothing_else() {
        let d = Hsem::new(&Props::new().with("cpu2-requester", 2u64)).unwrap();
        assert_eq!(d.regs.cpu2, Some(RequesterId(2)));
        assert_eq!(Hsem::new(&Props::new()).unwrap().regs.cpu2, None);
        assert!(Hsem::new(&Props::new().with("cpu2-requester", 1u64 << 40)).is_err());
        assert!(Hsem::new(&Props::new().with("semaphores", 32u64)).is_err());
        assert!(Hsem::new(&Props::new().with("clock", 64_000_000u64)).is_ok());
    }
}
