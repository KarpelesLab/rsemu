//! The PLIC: the board's external interrupt controller.
//!
//! # Source
//!
//! *RISC-V Platform-Level Interrupt Controller Specification*
//! (<https://github.com/riscv/riscv-plic-spec>). The register map of §3 and the
//! operation of §§1-2 are all that is modelled here:
//!
//! ```text
//!   0x000000 + 4*i          priority of source i   (source 0 does not exist)
//!   0x001000 + 4*(i/32)     pending bits, read-only
//!   0x002000 + 0x80*c + …   interrupt enables for context c
//!   0x200000 + 0x1000*c     priority threshold for context c
//!   0x200004 + 0x1000*c     claim on read, complete on write
//! ```
//!
//! # Gateways, and why a claim is not an acknowledge
//!
//! The specification's interrupt *gateway* is the part people skip and then
//! spend an afternoon on. A level-sensitive source's gateway raises a request
//! when the line asserts, and then **forwards nothing more until the handler
//! completes** — which is what stops a still-asserted device re-entering its own
//! handler forever. So this model keeps three bits per source: the line level
//! (what the device is doing), *pending* (what the PLIC will offer), and
//! *claimed* (a handler is running). A claim moves pending to claimed; a
//! completion clears claimed and re-raises pending if the line is still high.
//!
//! # Contexts
//!
//! A context is a hart plus a privilege level. This board gives each hart two —
//! machine and supervisor — which is what a platform running SBI firmware under
//! an operating system needs: the firmware owns context `2h` and the kernel
//! context `2h+1`. `supervisor = false` gives one context per hart.
//!
//! # What the device tree gets from this
//!
//! The interrupt number in a device's `interrupts` property is *not* written
//! down twice. The machine file wires `uart.irq -> plic.irq10`, this block
//! records which [`WireId`] arrived on which pin, and
//! [`dt`](super::dt) joins the two. See [`DtSource::dt_plic_source`].

use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::ToString;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{BusError, Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::space::{AccessConstraints, MemAttrs, MemOps, MemResult, Region, RegionRef};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::value::{Endian, Width};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::Instance;

use super::dt::{DtSource, NodeKind, NodeSpec};

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "riscv.plic";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register block occupies (§3).
pub const REGISTER_WINDOW_LEN: u64 = 0x40_0000;

/// Base of the per-source priority registers.
const PRIORITY_BASE: u64 = 0x00_0000;
/// Base of the read-only pending bit array.
const PENDING_BASE: u64 = 0x00_1000;
/// Base of the per-context enable bit arrays.
const ENABLE_BASE: u64 = 0x00_2000;
/// Bytes of enable bits per context: 1024 sources, one bit each.
const ENABLE_STRIDE: u64 = 0x80;
/// Base of the per-context threshold and claim registers.
const CONTEXT_BASE: u64 = 0x20_0000;
/// Bytes of register space per context.
const CONTEXT_STRIDE: u64 = 0x1000;

/// The largest source number the register map has room for (§3): source 0 is
/// reserved as "no interrupt", so 1023 real ones fit.
pub const MAX_SOURCES: u64 = 1023;

/// The largest context count this model accepts.
///
/// Far below the specification's 15872. It is the point where the enable
/// arrays would run into the context block, which is the only limit that comes
/// from the register map rather than from taste.
pub const MAX_CONTEXTS: u64 = (CONTEXT_BASE - ENABLE_BASE) / ENABLE_STRIDE;

/// How many priority levels are implemented.
///
/// The specification leaves this to the platform and requires only that 0 means
/// "never interrupt". Three bits is what RISC-V boards conventionally provide
/// and what a guest discovers by writing ones and reading back.
const PRIORITY_MASK: u32 = 0x7;

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// Priority per source, indexed by source number. Slot 0 is the reserved
    /// source and stays zero.
    priority: Vec<u32>,
    /// The gateway has a request the PLIC will offer.
    pending: Vec<bool>,
    /// What the wire is doing, which is not the same thing: a level-sensitive
    /// source that is still asserted re-raises `pending` at completion.
    level: Vec<bool>,
    /// A handler has claimed this source and not completed it.
    claimed: Vec<bool>,
    /// `enable[context][source]`.
    enable: Vec<Vec<bool>>,
    /// Priority threshold per context: only strictly greater priorities are
    /// offered (§2).
    threshold: Vec<u32>,
}

impl State {
    fn new(sources: usize, contexts: usize) -> State {
        State {
            priority: alloc::vec![0; sources + 1],
            pending: alloc::vec![false; sources + 1],
            level: alloc::vec![false; sources + 1],
            claimed: alloc::vec![false; sources + 1],
            enable: alloc::vec![alloc::vec![false; sources + 1]; contexts],
            threshold: alloc::vec![0; contexts],
        }
    }
}

/// The register block, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// The external-interrupt line of each context, at [`LockRank::LEAF`].
    outs: Mutex<Vec<Option<WireSource>>>,
    /// Which source number each driving net lands on, for the device tree.
    /// Filled in by [`Device::sink`], read by [`DtSource::dt_plic_source`].
    wires: Mutex<BTreeMap<WireId, u32>>,
    sources: usize,
    contexts: usize,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        s.field("sources", &self.sources)
            .field("contexts", &self.contexts);
        match self.state.try_lock() {
            Some(state) => s.field("pending", &count(&state.pending)).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

/// How many of `bits` are set — for `Debug`, where the whole array is noise.
fn count(bits: &[bool]) -> usize {
    bits.iter().filter(|b| **b).count()
}

/// The platform-level interrupt controller.
#[derive(Debug)]
pub struct Plic {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The sinks handed out by [`Device::sink`], kept alive here — a net holds
    /// only a weak reference to a sink, so the device owns the strong one.
    pins: Mutex<Vec<Arc<SourcePin>>>,
    /// Whether each hart gets a supervisor context as well as a machine one.
    supervisor: bool,
    harts: usize,
}

impl Plic {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] for a source or hart count the register map cannot
    /// hold, or for a property this class does not know.
    pub fn new(props: &Props) -> Result<Plic> {
        let mut r = props.reader();
        let sources = r.or_range("sources", 31u64, 1..=MAX_SOURCES)?;
        let harts = r.or_range("harts", 1u64, 1..=MAX_CONTEXTS / 2)?;
        let supervisor = r.or("supervisor", true)?;
        r.finish()?;
        Ok(Plic::build(sources as usize, harts as usize, supervisor))
    }

    /// Build one directly, for a test or a hand-wired machine.
    #[must_use]
    pub fn build(sources: usize, harts: usize, supervisor: bool) -> Plic {
        let contexts = harts * if supervisor { 2 } else { 1 };
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(sources, contexts)),
            outs: Mutex::with_rank(LockRank::LEAF, alloc::vec![None; contexts]),
            wires: Mutex::with_rank(LockRank::LEAF, BTreeMap::new()),
            sources,
            contexts,
        });
        let region: RegionRef = Arc::new(Region::io(
            "riscv.plic",
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        Plic {
            regs,
            region,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            supervisor,
            harts,
        }
    }

    /// How many interrupt sources it implements.
    #[must_use]
    pub fn sources(&self) -> usize {
        self.regs.sources
    }

    /// How many contexts it serves.
    #[must_use]
    pub fn contexts(&self) -> usize {
        self.regs.contexts
    }

    /// Drive source `source`'s input line directly, as a wire would.
    pub fn set_source(&self, source: u32, level: bool) {
        self.regs.set_level(source, level);
    }

    /// Whether the gateway for `source` has a request outstanding.
    #[must_use]
    pub fn is_pending(&self, source: u32) -> bool {
        self.regs
            .state
            .lock()
            .pending
            .get(source as usize)
            .copied()
            .unwrap_or(false)
    }

    /// The context index for a hart and privilege level, as this board
    /// allocates them.
    #[must_use]
    pub fn context_of(&self, hart: usize, supervisor: bool) -> Option<usize> {
        let index = if self.supervisor {
            hart * 2 + usize::from(supervisor)
        } else if supervisor {
            return None;
        } else {
            hart
        };
        (index < self.regs.contexts).then_some(index)
    }

    /// How many harts the machine file said it serves.
    #[must_use]
    pub fn harts(&self) -> usize {
        self.harts
    }
}

impl Registers {
    /// The highest-priority enabled pending source for `context`, or 0.
    ///
    /// Ties break toward the lowest source number, which the specification
    /// leaves to the implementation and which every guest expects because it
    /// is the only stable answer.
    fn best(state: &State, context: usize) -> u32 {
        let mut best = 0u32;
        let mut best_priority = state.threshold[context];
        for source in 1..state.pending.len() {
            if !state.pending[source] || !state.enable[context][source] {
                continue;
            }
            let priority = state.priority[source];
            if priority > best_priority {
                best_priority = priority;
                best = source as u32;
            }
        }
        best
    }

    /// Which contexts should now have their external line asserted.
    fn evaluate(state: &State) -> Vec<bool> {
        (0..state.threshold.len())
            .map(|c| Self::best(state, c) != 0)
            .collect()
    }

    /// Drive the external lines. Never called with the state lock held.
    fn drive(&self, levels: &[bool]) {
        let sources: Vec<Option<WireSource>> = self.outs.lock().clone();
        for (source, on) in sources.iter().zip(levels) {
            if let Some(source) = source {
                source.set(Level::from_bool(*on));
            }
        }
    }

    /// A source's input line moved.
    fn set_level(&self, source: u32, level: bool) {
        let levels = {
            let mut state = self.state.lock();
            let Some(slot) = state.level.get_mut(source as usize) else {
                return;
            };
            if *slot == level {
                return;
            }
            *slot = level;
            if source != 0 {
                // The gateway forwards a request on assertion, and forwards
                // nothing more while a handler holds the claim (§1.5).
                if level && !state.claimed[source as usize] {
                    state.pending[source as usize] = true;
                }
                // A deassertion does *not* clear a request that has already
                // been forwarded: the PLIC has promised it to a hart.
            }
            Self::evaluate(&state)
        };
        self.drive(&levels);
    }

    /// Claim the highest-priority interrupt for `context`, or 0 for none.
    fn claim(&self, context: usize) -> u32 {
        let (source, levels) = {
            let mut state = self.state.lock();
            let source = Self::best(&state, context);
            if source != 0 {
                state.pending[source as usize] = false;
                state.claimed[source as usize] = true;
            }
            let levels = Self::evaluate(&state);
            (source, levels)
        };
        self.drive(&levels);
        source
    }

    /// Complete `source` for `context`.
    fn complete(&self, context: usize, source: u32) {
        let levels = {
            let mut state = self.state.lock();
            let index = source as usize;
            if index == 0 || index >= state.claimed.len() || !state.enable[context][index] {
                // §2: a completion for a source the context cannot see is
                // ignored, which is what keeps one context from cancelling
                // another's handler.
                return;
            }
            state.claimed[index] = false;
            if state.level[index] {
                // Still asserted, so the gateway forwards again immediately.
                state.pending[index] = true;
            }
            Self::evaluate(&state)
        };
        self.drive(&levels);
    }

    /// Pack the pending array into the word at `index`, as the read-only
    /// pending register reports it.
    fn pending_word(state: &State, index: usize) -> u32 {
        let mut word = 0u32;
        for bit in 0..32 {
            let source = index * 32 + bit;
            if state.pending.get(source).copied().unwrap_or(false) {
                word |= 1 << bit;
            }
        }
        word
    }

    /// Pack one word of a context's enable bits.
    fn enable_word(state: &State, context: usize, index: usize) -> u32 {
        let mut word = 0u32;
        for bit in 0..32 {
            let source = index * 32 + bit;
            if state.enable[context].get(source).copied().unwrap_or(false) {
                word |= 1 << bit;
            }
        }
        word
    }
}

/// What an access decodes to.
enum Reg {
    Priority(usize),
    Pending(usize),
    Enable(usize, usize),
    Threshold(usize),
    Claim(usize),
    /// Inside the window but not a register this block implements.
    None,
}

/// Decode a register-relative offset (§3).
fn decode(offset: u64, contexts: usize) -> Reg {
    if offset < PENDING_BASE {
        return Reg::Priority(((offset - PRIORITY_BASE) / 4) as usize);
    }
    if offset < ENABLE_BASE {
        return Reg::Pending(((offset - PENDING_BASE) / 4) as usize);
    }
    if offset < CONTEXT_BASE {
        let context = ((offset - ENABLE_BASE) / ENABLE_STRIDE) as usize;
        let word = (((offset - ENABLE_BASE) % ENABLE_STRIDE) / 4) as usize;
        if context >= contexts {
            return Reg::None;
        }
        return Reg::Enable(context, word);
    }
    let context = ((offset - CONTEXT_BASE) / CONTEXT_STRIDE) as usize;
    if context >= contexts {
        return Reg::None;
    }
    match (offset - CONTEXT_BASE) % CONTEXT_STRIDE {
        0 => Reg::Threshold(context),
        4 => Reg::Claim(context),
        _ => Reg::None,
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        if dst.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        let value = match decode(offset, self.contexts) {
            Reg::Priority(source) => {
                let state = self.state.lock();
                state.priority.get(source).copied().unwrap_or(0)
            }
            Reg::Pending(index) => Registers::pending_word(&self.state.lock(), index),
            Reg::Enable(context, word) => Registers::enable_word(&self.state.lock(), context, word),
            Reg::Threshold(context) => self.state.lock().threshold[context],
            Reg::Claim(context) => {
                if attrs.debug {
                    // Reading the claim register *is* the claim. A debugger
                    // that peeked here would steal the guest's interrupt
                    // (`ROADMAP.md` §15, invariant 5).
                    return Err(BusError::BadAccess);
                }
                self.claim(context)
            }
            Reg::None => 0,
        };
        dst.copy_from_slice(&value.to_le_bytes());
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        if src.len() != 4 || !offset.is_multiple_of(4) {
            return Err(BusError::BadAccess);
        }
        if attrs.debug {
            // Every writable register here changes which interrupt a hart
            // takes next; none of it can be made side-effect free.
            return Err(BusError::BadAccess);
        }
        let value = u32::from_le_bytes([src[0], src[1], src[2], src[3]]);
        match decode(offset, self.contexts) {
            Reg::Priority(source) => {
                if source == 0 || source > self.sources {
                    // Source 0 does not exist and its priority register is
                    // read-only zero (§3).
                    return Ok(());
                }
                let levels = {
                    let mut state = self.state.lock();
                    state.priority[source] = value & PRIORITY_MASK;
                    Registers::evaluate(&state)
                };
                self.drive(&levels);
            }
            // The pending array is read-only: a source becomes pending because
            // its gateway said so, never because software said so.
            Reg::Pending(_) | Reg::None => {}
            Reg::Enable(context, word) => {
                let levels = {
                    let mut state = self.state.lock();
                    for bit in 0..32 {
                        let source = word * 32 + bit;
                        if source == 0 || source >= state.enable[context].len() {
                            continue;
                        }
                        state.enable[context][source] = value & (1 << bit) != 0;
                    }
                    Registers::evaluate(&state)
                };
                self.drive(&levels);
            }
            Reg::Threshold(context) => {
                let levels = {
                    let mut state = self.state.lock();
                    state.threshold[context] = value & PRIORITY_MASK;
                    Registers::evaluate(&state)
                };
                self.drive(&levels);
            }
            Reg::Claim(context) => self.complete(context, value),
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // Every register in the block is 32 bits and naturally aligned (§3).
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

impl DtSource for Registers {
    fn dt_spec(&self) -> NodeSpec {
        NodeSpec {
            kind: NodeKind::Plic {
                ndev: self.sources as u32,
            },
            name: "plic",
            compatible: &["sifive,plic-1.0.0", "riscv,plic0"],
            cells: Vec::new(),
            strings: Vec::new(),
            irq_wire: None,
        }
    }

    fn dt_plic_source(&self, wire: WireId) -> Option<u32> {
        self.wires.lock().get(&wire).copied()
    }
}

/// One of the PLIC's interrupt inputs, as something a wire can drive.
///
/// Keeps a [`FanIn`] and wire-ORs its sources, because a wire hands each sink
/// the level of the driver that changed rather than the resolved level of the
/// net (§4.3) — and a shared interrupt line is the case that makes the
/// difference.
#[derive(Debug)]
pub struct SourcePin {
    regs: Arc<Registers>,
    source: u32,
    inputs: FanIn,
}

impl SourcePin {
    /// Connect PLIC source `source` to a net driven by `sources`.
    fn new(regs: Arc<Registers>, source: u32, sources: &[WireId]) -> SourcePin {
        SourcePin {
            regs,
            source,
            inputs: FanIn::new(sources),
        }
    }

    /// Which PLIC source this pin drives.
    #[must_use]
    pub fn source(&self) -> u32 {
        self.source
    }
}

impl WireSink for SourcePin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        self.regs
            .set_level(self.source, self.inputs.resolve(Resolve::Or).is_high());
    }
}

/// The `riscv.plic` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "RISC-V platform-level interrupt controller: priority, enable, threshold, claim",
    properties: &[
        PropertySpec {
            name: "sources",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many interrupt sources, numbered from 1 (default 31)",
        },
        PropertySpec {
            name: "harts",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many harts it serves (default 1)",
        },
        PropertySpec {
            name: "supervisor",
            kind: ValueKind::Bool,
            required: false,
            summary: "whether each hart also gets a supervisor context (default true)",
        },
    ],
    construct: |props| Ok(Box::new(Plic::new(props)?)),
};

impl Device for Plic {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, ctx: &mut RealizeCtx<'_>) -> Result<()> {
        // What this region is, for the board's device-tree generator.
        super::dt::publish(
            ctx.hosts(),
            &self.region,
            Arc::downgrade(&self.regs) as Weak<dyn DtSource>,
        )
    }

    fn reset(&self, _kind: ResetKind) {
        let levels = {
            let mut state = self.regs.state.lock();
            *state = State::new(self.regs.sources, self.regs.contexts);
            Registers::evaluate(&state)
        };
        self.regs.drive(&levels);
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let source = port.strip_prefix("irq")?.parse::<u32>().ok()?;
        if source == 0 || source as usize > self.regs.sources {
            return None;
        }
        // The one place both a source number and the nets that drive it are
        // known, which is exactly what the device tree needs (see the module
        // docs).
        {
            let mut wires = self.regs.wires.lock();
            for id in sources {
                wires.insert(*id, source);
            }
        }
        let pin = Arc::new(SourcePin::new(Arc::clone(&self.regs), source, sources));
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: source,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let context = self.pin_context(port).ok_or_else(|| unknown_pin(port))?;
        self.regs.outs.lock()[context] = Some(source);
        Ok(())
    }

    fn announce(&self, port: &str) {
        let Some(context) = self.pin_context(port) else {
            return;
        };
        let level = {
            let state = self.regs.state.lock();
            Registers::best(&state, context) != 0
        };
        let out = self.regs.outs.lock()[context].clone();
        if let Some(out) = out {
            out.set(Level::from_bool(level));
        }
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        w.write_seq_len(state.priority.len() as u64)?;
        for p in &state.priority {
            w.write_u32(*p)?;
        }
        for bits in [&state.pending, &state.level, &state.claimed] {
            for bit in bits {
                w.write_bool(*bit)?;
            }
        }
        w.write_seq_len(state.threshold.len() as u64)?;
        for t in &state.threshold {
            w.write_u32(*t)?;
        }
        for context in &state.enable {
            for bit in context {
                w.write_bool(*bit)?;
            }
        }
        Ok(())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let count = r.read_seq_len(4)? as usize;
        if count != self.regs.sources + 1 {
            return Err(Error::State(format!(
                "snapshot has {} source(s) of PLIC state, this block implements {}",
                count.saturating_sub(1),
                self.regs.sources
            )));
        }
        let mut state = State::new(self.regs.sources, self.regs.contexts);
        for slot in &mut state.priority {
            *slot = r.read_u32()?;
        }
        for bits in [&mut state.pending, &mut state.level, &mut state.claimed] {
            for slot in bits.iter_mut() {
                *slot = r.read_bool()?;
            }
        }
        let contexts = r.read_seq_len(4)? as usize;
        if contexts != self.regs.contexts {
            return Err(Error::State(format!(
                "snapshot has {contexts} PLIC context(s), this block serves {}",
                self.regs.contexts
            )));
        }
        for slot in &mut state.threshold {
            *slot = r.read_u32()?;
        }
        for context in &mut state.enable {
            for slot in context.iter_mut() {
                *slot = r.read_bool()?;
            }
        }
        let levels = {
            let mut live = self.regs.state.lock();
            *live = state;
            Registers::evaluate(&live)
        };
        self.regs.drive(&levels);
        Ok(())
    }
}

impl Instance for Plic {}

impl Plic {
    /// The context an output pin name refers to.
    fn pin_context(&self, port: &str) -> Option<usize> {
        for (prefix, supervisor) in [("meip", false), ("seip", true)] {
            if let Some(rest) = port.strip_prefix(prefix) {
                let hart = rest.parse::<usize>().ok()?;
                return self.context_of(hart, supervisor);
            }
        }
        None
    }
}

/// The error for a pin this block does not drive.
fn unknown_pin(port: &str) -> Error {
    Error::Config {
        at: port.to_string(),
        message: format!(
            "the PLIC drives `meip<hart>` and, where a supervisor context exists, \
             `seip<hart>`; `{port}` is neither"
        ),
    }
}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(Plic::new(props)?)))
}

/// How many source pins the validator knows about.
///
/// A schema is per class and cannot know how a given file configured the
/// instance, so the pins are declared up to a number comfortably past what a
/// board of this shape uses. A `wire … -> plic.irq99` on a 31-source block is
/// still refused — by [`Device::sink`], at realize time, with the source count
/// in hand.
pub const SCHEMA_SOURCES: u32 = 63;

/// What the validator should know about `riscv.plic`.
#[must_use]
pub fn schema() -> crate::machine::validate::ClassSchema {
    use crate::machine::validate::{ClassSchema, PortDir, PropSchema};
    let mut s = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("sources", ValueKind::Uint).range(1, MAX_SOURCES))
        .prop(PropSchema::new("harts", ValueKind::Uint).range(1, MAX_CONTEXTS / 2))
        .prop(PropSchema::new("supervisor", ValueKind::Bool))
        .region("")
        .region("regs");
    for source in 1..=SCHEMA_SOURCES {
        s = s.port(format!("irq{source}"), PortDir::In);
    }
    for hart in 0..8u32 {
        s = s
            .port(format!("meip{hart}"), PortDir::Out)
            .port(format!("seip{hart}"), PortDir::Out);
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::sync::{AtomicU32, Ordering};
    use crate::core::wire::{Wire, WireIdAllocator};

    fn plic() -> Plic {
        Plic::build(31, 1, true)
    }

    fn read(p: &Plic, offset: u64) -> u32 {
        let mut bytes = [0u8; 4];
        p.regs
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .expect("a word read is legal");
        u32::from_le_bytes(bytes)
    }

    fn write(p: &Plic, offset: u64, value: u32) {
        p.regs
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a word write is legal");
    }

    fn priority(source: u64) -> u64 {
        PRIORITY_BASE + 4 * source
    }

    fn enable(context: u64) -> u64 {
        ENABLE_BASE + ENABLE_STRIDE * context
    }

    fn threshold(context: u64) -> u64 {
        CONTEXT_BASE + CONTEXT_STRIDE * context
    }

    fn claim(context: u64) -> u64 {
        threshold(context) + 4
    }

    /// A sink that records the last level it was told.
    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
        }
    }

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) != 0
        }
    }

    /// A PLIC with `meip0` on a probe, and source 10 set up for context 0.
    fn armed() -> (Plic, Arc<Probe>) {
        let plic = plic();
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        let probe = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&probe) as Arc<dyn WireSink>, 0)
            .build_shared();
        plic.connect("meip0", WireSource::new(wire, id))
            .expect("the PLIC drives meip0");
        write(&plic, priority(10), 1);
        write(&plic, enable(0), 1 << 10);
        (plic, probe)
    }

    #[test]
    fn a_source_only_interrupts_when_enabled_and_above_the_threshold() {
        let (plic, probe) = armed();
        plic.set_source(10, true);
        assert!(probe.high());

        // Threshold 1 masks priority 1: the comparison is strictly greater.
        write(&plic, threshold(0), 1);
        assert!(!probe.high());
        write(&plic, threshold(0), 0);
        assert!(probe.high());

        // Disabling it in this context masks it too.
        write(&plic, enable(0), 0);
        assert!(!probe.high());
    }

    #[test]
    fn priority_zero_never_interrupts() {
        let (plic, probe) = armed();
        write(&plic, priority(10), 0);
        plic.set_source(10, true);
        assert!(!probe.high(), "priority 0 means never (spec §2)");
        assert!(plic.is_pending(10), "but the gateway still has the request");
    }

    #[test]
    fn a_claim_takes_the_highest_priority_and_leaves_the_rest() {
        let (plic, _probe) = armed();
        write(&plic, priority(5), 3);
        write(&plic, enable(0), (1 << 5) | (1 << 10));
        plic.set_source(5, true);
        plic.set_source(10, true);

        assert_eq!(read(&plic, claim(0)), 5, "priority 3 beats priority 1");
        assert_eq!(read(&plic, claim(0)), 10);
        assert_eq!(read(&plic, claim(0)), 0, "and then there are none");
    }

    #[test]
    fn ties_break_toward_the_lowest_source_number() {
        let (plic, _probe) = armed();
        write(&plic, priority(3), 1);
        write(&plic, priority(7), 1);
        write(&plic, enable(0), (1 << 3) | (1 << 7));
        plic.set_source(7, true);
        plic.set_source(3, true);
        assert_eq!(read(&plic, claim(0)), 3);
    }

    #[test]
    fn a_still_asserted_source_re_raises_at_completion_and_not_before() {
        // The gateway rule, and the one worth a test: a level-sensitive device
        // that has not been serviced must not re-enter its own handler.
        let (plic, probe) = armed();
        plic.set_source(10, true);
        assert_eq!(read(&plic, claim(0)), 10);
        assert!(!probe.high(), "claimed, so nothing is offered");
        assert!(!plic.is_pending(10));

        write(&plic, claim(0), 10);
        assert!(plic.is_pending(10), "the line is still asserted");
        assert!(probe.high());

        // And once the device drops its line, completing ends it.
        assert_eq!(read(&plic, claim(0)), 10);
        plic.set_source(10, false);
        write(&plic, claim(0), 10);
        assert!(!plic.is_pending(10));
        assert!(!probe.high());
    }

    #[test]
    fn a_completion_for_a_source_this_context_cannot_see_is_ignored() {
        let (plic, _probe) = armed();
        plic.set_source(10, true);
        assert_eq!(read(&plic, claim(0)), 10);
        // Context 1 has nothing enabled, so its completion must not cancel
        // context 0's handler.
        write(&plic, claim(1), 10);
        plic.set_source(10, false);
        write(&plic, claim(1), 10);
        assert!(!plic.is_pending(10));
        write(&plic, claim(0), 10);
        assert!(!plic.is_pending(10), "and now it really is done");
    }

    #[test]
    fn the_pending_array_is_read_only() {
        let (plic, _probe) = armed();
        write(&plic, PENDING_BASE, 0xffff_ffff);
        assert_eq!(read(&plic, PENDING_BASE), 0);
        plic.set_source(10, true);
        assert_eq!(read(&plic, PENDING_BASE), 1 << 10);
    }

    #[test]
    fn source_zero_does_not_exist() {
        let (plic, _probe) = armed();
        write(&plic, priority(0), 7);
        assert_eq!(read(&plic, priority(0)), 0);
        // Nor can it be enabled, whatever bit 0 of the enable word says.
        write(&plic, enable(0), 0xffff_ffff);
        plic.set_source(0, true);
        assert!(!plic.is_pending(0));
    }

    #[test]
    fn the_priority_register_reports_how_many_levels_there_are() {
        let (plic, _probe) = armed();
        write(&plic, priority(1), 0xffff_ffff);
        assert_eq!(read(&plic, priority(1)), PRIORITY_MASK);
    }

    #[test]
    fn a_debug_access_neither_claims_nor_configures() {
        let (plic, _probe) = armed();
        plic.set_source(10, true);
        let mut bytes = [0u8; 4];
        assert!(
            plic.regs
                .read(claim(0), &mut bytes, MemAttrs::DEBUG)
                .is_err(),
            "reading the claim register is the claim"
        );
        assert!(plic.is_pending(10), "so nothing was taken");
        assert!(
            plic.regs
                .write(priority(1), &1u32.to_le_bytes(), MemAttrs::DEBUG)
                .is_err()
        );
        // Ordinary registers are still readable by a debugger.
        plic.regs
            .read(priority(10), &mut bytes, MemAttrs::DEBUG)
            .expect("a debugger may look at a priority");
        assert_eq!(u32::from_le_bytes(bytes), 1);
    }

    #[test]
    fn an_access_that_is_not_an_aligned_word_is_refused() {
        let plic = plic();
        assert!(plic.regs.read(1, &mut [0u8; 4], MemAttrs::DEFAULT).is_err());
        assert!(plic.regs.read(0, &mut [0u8; 8], MemAttrs::DEFAULT).is_err());
        assert!(plic.regs.write(0, &[0u8; 1], MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn a_snapshot_round_trips_the_whole_block() {
        use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

        let (saved, _probe) = armed();
        saved.set_source(10, true);
        write(&saved, threshold(1), 2);
        write(&saved, enable(1), 1 << 3);

        let mut shape = MachineShape::new();
        shape.add_device("plic", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("plic", CLASS.name, CLASS.version).unwrap();
            saved.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = plic();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("plic", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.load(&mut chunk.reader()).unwrap();

        assert!(restored.is_pending(10));
        assert_eq!(read(&restored, priority(10)), 1);
        assert_eq!(read(&restored, threshold(1)), 2);
        assert_eq!(read(&restored, enable(1)), 1 << 3);
        assert_eq!(read(&restored, claim(0)), 10, "and it can still be claimed");
    }

    #[test]
    fn contexts_are_two_per_hart_with_supervisor_and_one_without() {
        let both = Plic::build(31, 2, true);
        assert_eq!(both.contexts(), 4);
        assert_eq!(both.context_of(1, true), Some(3));
        let machine_only = Plic::build(31, 2, false);
        assert_eq!(machine_only.contexts(), 2);
        assert_eq!(machine_only.context_of(1, true), None);
        assert_eq!(machine_only.context_of(1, false), Some(1));
    }

    #[test]
    fn a_source_pin_wire_ors_its_drivers() {
        // Two devices sharing one PLIC source is ordinary; the source must stay
        // asserted until both have dropped it (§4.3).
        let plic = plic();
        let ids = WireIdAllocator::new();
        let (a, b) = (ids.alloc(), ids.alloc());
        let pin = Device::sink(&plic, "irq10", &[a, b]).expect("source 10 exists");
        pin.sink.set_level(a, pin.line, Level::High);
        assert!(plic.is_pending(10));
        // Clear the gateway by claiming, then check the line is still up.
        write(&plic, priority(10), 1);
        write(&plic, enable(0), 1 << 10);
        assert_eq!(read(&plic, claim(0)), 10);
        pin.sink.set_level(a, pin.line, Level::Low);
        pin.sink.set_level(b, pin.line, Level::High);
        write(&plic, claim(0), 10);
        assert!(plic.is_pending(10), "b is still driving it");
    }

    #[test]
    fn the_pin_table_is_what_the_device_tree_reads_the_interrupt_number_from() {
        let plic = plic();
        let ids = WireIdAllocator::new();
        let id = ids.alloc();
        Device::sink(&plic, "irq10", &[id]).expect("source 10 exists");
        assert_eq!(plic.regs.dt_plic_source(id), Some(10));
        assert_eq!(plic.regs.dt_plic_source(ids.alloc()), None);
        assert!(
            Device::sink(&plic, "irq99", &[id]).is_none(),
            "out of range"
        );
        assert!(Device::sink(&plic, "irq0", &[id]).is_none(), "reserved");
    }
}
