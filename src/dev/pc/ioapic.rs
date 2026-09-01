//! An Intel 82093AA I/O APIC.
//!
//! # Sources
//!
//! * *Intel 82093AA I/O Advanced Programmable Interrupt Controller (IOAPIC)*
//!   datasheet, order 290566-001. The two-register memory window (§3.1), the
//!   identification, version and arbitration registers (§3.2.1-§3.2.3) and the
//!   redirection table with its remote IRR, trigger mode, polarity and mask
//!   (§3.2.4) all come from it.
//! * *Intel 64 and IA-32 Architectures Software Developer's Manual*, Volume 3A,
//!   §10.6.2, for the destination modes the redirection entry's destination
//!   field is interpreted in — they are the message's, not this part's.
//!
//! **No emulator source was consulted** (`CLAUDE.md`, provenance).
//!
//! # What it does
//!
//! It is a fan-in with a lookup table. Twenty-four interrupt input pins, one
//! 64-bit redirection entry each, and every entry says which vector to send and
//! which processors to send it to. There is no priority resolver and no
//! in-service register: the part turns a *pin* into a *message* and the local
//! APIC that receives it does the deciding.
//!
//! The one piece of state it keeps per interrupt is **remote IRR**, and it
//! exists for level-triggered lines. An edge is an event and is forwarded once.
//! A level is a condition, and forwarding it repeatedly while the device is
//! still asserting would flood the processor — so the entry latches remote IRR
//! when it sends, and clears it when the processor writes its end-of-interrupt
//! register for that vector. If the line is *still* asserted then, the entry
//! sends again, which is exactly what a shared PCI interrupt needs.
//!
//! # The wire contract
//!
//! A device must not invent a level for an input pin (`ROADMAP.md` §4.3), and
//! a level-triggered redirection entry is where that bug would live: an entry
//! unmasked while its line is already high must deliver, and an entry whose
//! line was never driven must not. So each pin keeps a [`FanIn`] over its
//! sources, an undriven pin sits low because that is what a fresh `FanIn`
//! holds, and *every* recomputation runs from the pin levels rather than from a
//! remembered decision.

use alloc::boxed::Box;
use alloc::format;
use alloc::string::{String, ToString};
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
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink};
use crate::dev::pc::apic::bus::{self, ApicBus, Delivery, EoiSink, Message, Shorthand};
use crate::machine::realize::Instance;
use crate::machine::validate::ClassSchema;

/// The class name a machine description writes.
pub const CLASS_NAME: &str = "pc.ioapic";

/// The snapshot chunk version. Bump with the encoding, never on its own.
const STATE_VERSION: u32 = 1;

/// How much address space the register window answers.
///
/// Two registers sixteen bytes apart (82093AA §3.1), which the part decodes in
/// a 32-byte window.
pub const REGISTER_WINDOW_LEN: u64 = 0x20;

/// The page the register window sits at on a PC when firmware has not moved it.
pub const DEFAULT_BASE: u64 = 0xfec0_0000;

/// How many interrupt inputs an 82093AA has.
pub const INPUTS: usize = 24;

/// The version this part reports (82093AA §3.2.2).
const VERSION: u32 = 0x11;

/// The index register, at offset 0 of the window.
const IOREGSEL: u64 = 0x00;
/// The data window, at offset 0x10.
const IOWIN: u64 = 0x10;

/// Indirect register 0: the identification register.
const IDX_ID: u8 = 0x00;
/// Indirect register 1: version, and the highest redirection entry.
const IDX_VERSION: u8 = 0x01;
/// Indirect register 2: the arbitration identification register.
const IDX_ARB: u8 = 0x02;
/// Indirect register 0x10: the first of the redirection table's two-word
/// entries.
const IDX_REDIR: u8 = 0x10;

/// A redirection entry's mask bit (bit 16).
const ENTRY_MASK: u64 = 1 << 16;
/// Its trigger mode bit (bit 15): set is level.
const ENTRY_LEVEL: u64 = 1 << 15;
/// Its remote IRR (bit 14), read-only to software.
const ENTRY_REMOTE_IRR: u64 = 1 << 14;
/// Its input pin polarity (bit 13): set is active low.
const ENTRY_ACTIVE_LOW: u64 = 1 << 13;
/// Its delivery status (bit 12), read-only and always idle here.
const ENTRY_DELIVERY_STATUS: u64 = 1 << 12;
/// Its destination mode (bit 11): set is logical.
const ENTRY_LOGICAL: u64 = 1 << 11;

/// Which bits of an entry software may change.
///
/// Delivery status and remote IRR are the part's, not the driver's; bits 17-55
/// are reserved and read back as zero.
const ENTRY_WRITABLE: u64 =
    0xff00_0000_0000_0000 | (0x0001_ffff & !(ENTRY_DELIVERY_STATUS | ENTRY_REMOTE_IRR));

/// A redirection entry as it comes out of reset: masked, and nothing else
/// (82093AA §3.2.4).
const ENTRY_RESET: u64 = ENTRY_MASK;

/// Everything the guest can see or change.
#[derive(Debug, Clone, PartialEq, Eq)]
struct State {
    /// The identification register's ID field, bits 24-27.
    id: u8,
    /// The index register: which indirect register the data window reaches.
    select: u8,
    /// The redirection table.
    redir: Vec<u64>,
    /// What each input pin is doing, as a raw level before the entry's
    /// polarity bit is applied. One bit per input, and **not** the same thing
    /// as a pending request: an edge-triggered line that has already been
    /// forwarded is still high.
    pins: u32,
}

impl State {
    /// A part with `inputs` interrupt inputs, out of reset.
    fn new(id: u8, inputs: usize) -> State {
        State {
            id,
            select: 0,
            redir: alloc::vec![ENTRY_RESET; inputs],
            pins: 0,
        }
    }

    /// Whether input `index` is asserting, which the entry's polarity decides.
    fn asserted(&self, index: usize) -> bool {
        let high = self.pins & (1 << index) != 0;
        high != (self.redir[index] & ENTRY_ACTIVE_LOW != 0)
    }

    /// The message entry `index` would send.
    fn message(&self, index: usize) -> Message {
        let entry = self.redir[index];
        Message {
            vector: entry as u8,
            delivery: Delivery(((entry >> 8) & 7) as u8),
            logical: entry & ENTRY_LOGICAL != 0,
            dest: (entry >> 56) as u8,
            level_triggered: entry & ENTRY_LEVEL != 0,
            assert: true,
        }
    }

    /// Decide what entry `index` should do now, given the pin level it can see.
    ///
    /// `rising` says whether this call is being made because the input just
    /// went from idle to asserted, which is the only thing an edge-triggered
    /// entry reacts to. Everything else — a write to the entry, an
    /// end-of-interrupt, a snapshot load — is a *re-evaluation*, and a
    /// re-evaluation may only send for a level-triggered entry, whose
    /// condition is still true by inspection.
    fn evaluate(&mut self, index: usize, rising: bool) -> Option<Message> {
        let entry = self.redir[index];
        if entry & ENTRY_MASK != 0 {
            return None;
        }
        if entry & ENTRY_LEVEL != 0 {
            if !self.asserted(index) || entry & ENTRY_REMOTE_IRR != 0 {
                return None;
            }
            self.redir[index] |= ENTRY_REMOTE_IRR;
            Some(self.message(index))
        } else {
            rising.then(|| self.message(index))
        }
    }
}

/// The register window, as something an address space can dispatch to.
struct Registers {
    state: Mutex<State>,
    /// The bus messages go out on and end-of-interrupt broadcasts come back
    /// from.
    bus: Arc<ApicBus>,
}

impl fmt::Debug for Registers {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Registers");
        match self.state.try_lock() {
            Some(state) => s.field("state", &*state).finish(),
            None => s.field("state", &"<in use>").finish(),
        }
    }
}

impl Registers {
    /// Put every message on the bus. **Never called with the state lock held**:
    /// a delivery lands in a local APIC that takes its own state lock and
    /// drives its processor's pin, and that processor may be the one whose
    /// register write got us here (`CLAUDE.md`, re-entrancy).
    fn send(&self, messages: &[Message]) {
        for message in messages {
            self.bus.deliver(*message, None, Shorthand::Dest);
        }
    }

    /// Drive one input pin.
    fn set_pin(&self, index: usize, high: bool) {
        let messages = {
            let mut state = self.state.lock();
            if index >= state.redir.len() {
                return;
            }
            let was = state.asserted(index);
            let bit = 1u32 << index;
            if high {
                state.pins |= bit;
            } else {
                state.pins &= !bit;
            }
            let rising = state.asserted(index) && !was;
            state
                .evaluate(index, rising)
                .into_iter()
                .collect::<Vec<_>>()
        };
        self.send(&messages);
    }

    /// Read one indirect register.
    fn read_indirect(&self, state: &State, index: u8) -> u32 {
        match index {
            IDX_ID => u32::from(state.id) << 24,
            // Bits 0-7 the version, bits 16-23 the number of redirection
            // entries less one (82093AA §3.2.2).
            IDX_VERSION => VERSION | ((state.redir.len() as u32 - 1) << 16),
            // The arbitration ID. On a part with no APIC bus to arbitrate for
            // it simply mirrors the ID, which is what a power-up value of the
            // ID does anyway.
            IDX_ARB => u32::from(state.id) << 24,
            _ => {
                let offset = index.wrapping_sub(IDX_REDIR) as usize;
                let entry = offset / 2;
                if index < IDX_REDIR || entry >= state.redir.len() {
                    // "Undefined" in the datasheet. Zero, because an emulator
                    // does not get to be undefined and a driver sizing the
                    // table by reading past it should see nothing.
                    return 0;
                }
                let word = state.redir[entry];
                if offset.is_multiple_of(2) {
                    word as u32
                } else {
                    (word >> 32) as u32
                }
            }
        }
    }

    /// Write one indirect register, reporting any message it set off.
    fn write_indirect(&self, state: &mut State, index: u8, value: u32) -> Option<Message> {
        match index {
            IDX_ID => {
                // Four bits: "the APIC ID serves as a physical name" and the
                // 82093AA carries it in bits 27:24 (§3.2.1).
                state.id = ((value >> 24) & 0x0f) as u8;
                None
            }
            // Version and arbitration are read-only.
            IDX_VERSION | IDX_ARB => None,
            _ => {
                let offset = index.wrapping_sub(IDX_REDIR) as usize;
                let entry = offset / 2;
                if index < IDX_REDIR || entry >= state.redir.len() {
                    return None;
                }
                let shift = if offset.is_multiple_of(2) { 0 } else { 32 };
                let half = 0xffff_ffffu64 << shift;
                let writable = ENTRY_WRITABLE & half;
                state.redir[entry] =
                    (state.redir[entry] & !writable) | ((u64::from(value) << shift) & writable);
                // Unmasking a level-triggered entry whose line is already
                // asserted has to deliver: the condition was true the whole
                // time and nothing else will ever announce it.
                state.evaluate(entry, false)
            }
        }
    }
}

impl EoiSink for Registers {
    /// A processor has finished with `vector`.
    ///
    /// Every level-triggered entry carrying that vector drops its remote IRR,
    /// and any whose line is still asserted sends again — which is the whole
    /// reason level-triggered entries have the bit.
    fn eoi(&self, vector: u8) {
        let messages = {
            let mut state = self.state.lock();
            let mut out = Vec::new();
            for index in 0..state.redir.len() {
                let entry = state.redir[index];
                if entry & ENTRY_LEVEL == 0
                    || entry & ENTRY_REMOTE_IRR == 0
                    || entry as u8 != vector
                {
                    continue;
                }
                state.redir[index] &= !ENTRY_REMOTE_IRR;
                out.extend(state.evaluate(index, false));
            }
            out
        };
        self.send(&messages);
    }
}

impl MemOps for Registers {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = dst else {
            return Err(BusError::BadAccess);
        };
        let state = self.state.lock();
        let value = match offset {
            IOREGSEL => u32::from(state.select),
            IOWIN => self.read_indirect(&state, state.select),
            // The window decodes two registers and nothing between them. A
            // debug read is no different: there is nothing there either way.
            _ => return Err(BusError::BadAccess),
        };
        let _ = attrs;
        let bytes = value.to_le_bytes();
        *a = bytes[0];
        *b = bytes[1];
        *c = bytes[2];
        *d = bytes[3];
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let [a, b, c, d] = src else {
            return Err(BusError::BadAccess);
        };
        if attrs.debug {
            // There is no harmless write. The index register is half of a
            // two-step protocol the guest is in the middle of, and the data
            // window unmasks interrupts (`ROADMAP.md` §15, invariant 5).
            return Err(BusError::BadAccess);
        }
        let value = u32::from_le_bytes([*a, *b, *c, *d]);
        let message = {
            let mut state = self.state.lock();
            match offset {
                IOREGSEL => {
                    state.select = value as u8;
                    None
                }
                IOWIN => {
                    let index = state.select;
                    self.write_indirect(&mut state, index, value)
                }
                _ => return Err(BusError::BadAccess),
            }
        };
        if let Some(message) = message {
            self.send(&[message]);
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        // 32 bits, naturally aligned: the part decodes two Dword registers
        // (82093AA §3.1), and a byte write to half an index would be a cycle
        // the chip does not claim.
        AccessConstraints::word(Width::U32, Endian::Little)
    }
}

// ---------------------------------------------------------------------------
// the device
// ---------------------------------------------------------------------------

/// An Intel 82093AA I/O APIC.
#[derive(Debug)]
pub struct IoApic {
    regs: Arc<Registers>,
    region: RegionRef,
    /// The device's own references to its input pins. A net holds only weak
    /// ones, so something has to keep them alive.
    pins: Mutex<Vec<Arc<InputPin>>>,
    /// The ID a reset restores, which is strapped on real silicon.
    reset_id: u8,
    /// How many interrupt inputs this part has.
    inputs: usize,
}

/// One interrupt input pin.
///
/// The [`FanIn`] is why this is one object per line: several devices may
/// wire-OR onto one input, which is what a shared PCI interrupt is, and a pin
/// told "low" must know whether some other driver is still asserting before it
/// withdraws.
#[derive(Debug)]
struct InputPin {
    regs: Arc<Registers>,
    index: usize,
    inputs: FanIn,
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        let high = self.inputs.resolve(Resolve::Or).is_high();
        self.regs.set_pin(self.index, high);
    }
}

impl IoApic {
    /// Validate `props` and build the device.
    ///
    /// # Errors
    ///
    /// [`Error::Property`](crate::core::Error::Property) if `id` or `inputs` is
    /// out of range, or if a property this class does not know was given.
    pub fn new(props: &Props) -> Result<IoApic> {
        let mut r = props.reader();
        let id = u8::try_from(r.or_range::<u64>("id", 0, 0..=15)?).unwrap_or(0);
        let inputs = r.or_range::<u64>("inputs", INPUTS as u64, 1..=INPUTS as u64)? as usize;
        let name = r.or_str("bus", bus::DEFAULT_NAME)?.to_string();
        r.finish()?;
        let bus = bus::attach(props, &name)?;
        Ok(IoApic::with_bus(id, inputs, bus))
    }

    /// One in the default configuration: ID 0, twenty-four inputs, on a bus of
    /// its own.
    #[must_use]
    pub fn default_device() -> IoApic {
        IoApic::with_bus(0, INPUTS, Arc::new(ApicBus::new()))
    }

    /// One with the ID and input count given, on `bus`.
    #[must_use]
    pub fn with_bus(id: u8, inputs: usize, bus: Arc<ApicBus>) -> IoApic {
        let regs = Arc::new(Registers {
            state: Mutex::with_rank(LockRank::DEVICE, State::new(id, inputs)),
            bus,
        });
        let region: RegionRef = Arc::new(Region::io(
            CLASS_NAME,
            REGISTER_WINDOW_LEN,
            Arc::clone(&regs) as Arc<dyn MemOps>,
        ));
        IoApic {
            regs,
            region,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            reset_id: id,
            inputs,
        }
    }

    /// The message bus this part sends on.
    #[must_use]
    pub fn bus(&self) -> &Arc<ApicBus> {
        &self.regs.bus
    }

    /// How many interrupt inputs it has.
    #[must_use]
    pub fn inputs(&self) -> usize {
        self.inputs
    }

    /// One redirection entry, for tests and the monitor.
    #[must_use]
    pub fn entry(&self, index: usize) -> Option<u64> {
        self.regs.state.lock().redir.get(index).copied()
    }

    /// Which input pin `port` names, if it names one.
    fn pin_number(port: &str, inputs: usize) -> Option<usize> {
        let index: usize = port.strip_prefix("irq")?.parse().ok()?;
        (index < inputs).then_some(index)
    }
}

/// The `pc.ioapic` device class.
pub static CLASS: DeviceClass = DeviceClass {
    name: CLASS_NAME,
    version: STATE_VERSION,
    summary: "Intel 82093AA I/O APIC",
    properties: &[
        PropertySpec {
            name: "id",
            kind: ValueKind::Uint,
            required: false,
            summary: "the APIC ID this part is strapped to, 0-15 (default 0)",
        },
        PropertySpec {
            name: "inputs",
            kind: ValueKind::Uint,
            required: false,
            summary: "how many interrupt inputs it has, 1-24 (default 24)",
        },
        PropertySpec {
            name: "bus",
            kind: ValueKind::Str,
            required: false,
            summary: "the APIC message bus this part sends on (default `apic`)",
        },
    ],
    construct: |props| Ok(Box::new(IoApic::new(props)?)),
};

impl Device for IoApic {
    fn class(&self) -> &'static DeviceClass {
        &CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        self.regs
            .bus
            .attach_eoi(Arc::downgrade(&self.regs) as Weak<dyn EoiSink>);
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Both kinds. Every entry comes back masked, which is what a driver
        // finding an unprogrammed part expects.
        //
        // The pin levels survive, for the same reason the 8259A's do: a line
        // the board is still holding has not moved, and nothing re-announces an
        // unchanged level. Nothing is delivered here either — an entry that has
        // just been masked cannot send, and a re-evaluation after the reset
        // would be inventing an edge.
        let mut state = self.regs.state.lock();
        let pins = state.pins;
        *state = State::new(self.reset_id, self.inputs);
        state.pins = pins;
    }

    fn region(&self, name: &str) -> Option<RegionRef> {
        matches!(name, "" | "regs").then(|| Arc::clone(&self.region))
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let index = IoApic::pin_number(port, self.inputs)?;
        // The fan-in can only be built now: it is told its sources at
        // construction and no `WireId` existed when this part was made.
        //
        // Nothing seeds the pin level from it, and nothing needs to: an
        // interrupt input idles low, a fresh `FanIn` holds every source low,
        // and `State::new` agrees with both. If either default ever moves, this
        // is where the level has to be adopted — a driver idling at its own
        // default announces no change, so the wire will never say it.
        let pin = Arc::new(InputPin {
            regs: Arc::clone(&self.regs),
            index,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: index as u32,
        })
    }

    fn connect(&self, port: &str, _source: crate::core::wire::WireSource) -> Result<()> {
        Err(Error::Config {
            at: port.to_string(),
            message: String::from(
                "an I/O APIC drives no wire: it sends messages on the APIC bus instead",
            ),
        })
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        let state = self.regs.state.lock();
        w.write_u8(state.id)?;
        w.write_u8(state.select)?;
        w.write_u32(state.pins)?;
        w.write_seq_len(state.redir.len() as u64)?;
        for entry in &state.redir {
            w.write_u64(*entry)?;
        }
        Ok(())
        // The bus and the pins are the machine's wiring, not this part's state.
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        let id = r.read_u8()?;
        let select = r.read_u8()?;
        let pins = r.read_u32()?;
        let count = r.read_seq_len(8)? as usize;
        if count != self.inputs {
            return Err(Error::State(format!(
                "snapshot has {count} redirection entries, this part has {}",
                self.inputs
            )));
        }
        let mut redir = Vec::with_capacity(count);
        for _ in 0..count {
            redir.push(r.read_u64()?);
        }
        let mut state = self.regs.state.lock();
        *state = State {
            id,
            select,
            redir,
            pins,
        };
        Ok(())
        // Nothing is delivered on the way in. A restored level-triggered entry
        // carries its own remote IRR, so the request it had already sent is
        // still recorded as sent; re-sending it here would double the interrupt
        // the snapshot was taken in the middle of.
    }
}

impl Instance for IoApic {}

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
    bindings.bind(CLASS_NAME, |props| Ok(Arc::new(IoApic::new(props)?)))
}

/// What the validator should know about `pc.ioapic`.
#[must_use]
pub fn schema() -> ClassSchema {
    use crate::machine::validate::{PortDir, PropSchema};
    let mut schema = ClassSchema::new(CLASS_NAME)
        .prop(PropSchema::new("id", ValueKind::Uint).range(0, 15))
        .prop(PropSchema::new("inputs", ValueKind::Uint).range(1, INPUTS as u64))
        .prop(PropSchema::new("bus", ValueKind::Str))
        .region("")
        .region("regs");
    for index in 0..INPUTS {
        schema = schema.port(format!("irq{index}"), PortDir::In);
    }
    schema
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::device::ResetKind;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::wire::{IntAckCycle, IntAckResponse, WireIdAllocator};
    use crate::dev::pc::apic::LocalApic;

    /// The vector the tests route their line to.
    const VECTOR: u8 = 0x33;
    /// Which input the tests use. Not zero, so an off-by-one in the index
    /// arithmetic cannot pass by landing on the entry it started at.
    const LINE: usize = 5;

    /// An I/O APIC and one local APIC on the same bus, with every input driven.
    struct Bench {
        io: IoApic,
        lapic: LocalApic,
        pins: Vec<Arc<dyn WireSink>>,
        src: WireId,
    }

    fn bench() -> Bench {
        let bus = Arc::new(ApicBus::new());
        let io = IoApic::with_bus(0, INPUTS, Arc::clone(&bus));
        let lapic = LocalApic::with_bus(0, true, Arc::clone(&bus));
        let ids = WireIdAllocator::new();
        let src = ids.alloc();
        let pins: Vec<Arc<dyn WireSink>> = (0..INPUTS)
            .map(|index| {
                io.sink(&format!("irq{index}"), &[src])
                    .expect("every input exists")
                    .sink
            })
            .collect();
        // What `realize` does for each of them. A unit test has no machine to
        // realize it, and nothing is delivered until both are on the bus.
        realize(&io);
        realize(&lapic);
        // The local APIC has to be software-enabled before it accepts anything
        // through its local vector table; the message inbox is open regardless,
        // which is what this test relies on.
        write_lapic(&lapic, 0x0f0, 0x1ff);
        Bench {
            io,
            lapic,
            pins,
            src,
        }
    }

    /// Run a device's `realize`, which is the only thing that puts it on the
    /// APIC bus. A unit test has no machine to do it.
    fn realize(device: &dyn Device) {
        let hosts = crate::core::hosts::HostObjects::new();
        let mut deferred = crate::core::device::Deferred::new();
        let mut ctx = crate::core::device::RealizeCtx::new(
            "test",
            crate::core::space::RequesterId::default(),
            &mut deferred,
            &hosts,
        );
        device.realize(&mut ctx).expect("realize cannot fail here");
        deferred.drain();
    }

    /// The `MemOps` behind a device's named region.
    fn ops(device: &dyn Device, name: &str) -> Arc<dyn MemOps> {
        match device.region(name).expect("the region exists").kind() {
            crate::core::space::RegionKind::Io(ops) => Arc::clone(ops),
            _ => unreachable!("a register block is an I/O region"),
        }
    }

    fn write_lapic(lapic: &LocalApic, offset: u64, value: u32) {
        ops(lapic, "regs")
            .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
            .expect("a 32-bit aligned write is legal");
    }

    fn read_lapic(lapic: &LocalApic, offset: u64) -> u32 {
        let mut bytes = [0u8; 4];
        ops(lapic, "regs")
            .read(offset, &mut bytes, MemAttrs::DEFAULT)
            .expect("a 32-bit aligned read is legal");
        u32::from_le_bytes(bytes)
    }

    impl Bench {
        /// Write one indirect register, the way a driver does: index, then
        /// data.
        fn write_indirect(&self, index: u8, value: u32) {
            self.poke(IOREGSEL, u32::from(index));
            self.poke(IOWIN, value);
        }

        fn read_indirect(&self, index: u8) -> u32 {
            self.poke(IOREGSEL, u32::from(index));
            self.peek(IOWIN)
        }

        fn poke(&self, offset: u64, value: u32) {
            self.io
                .regs
                .write(offset, &value.to_le_bytes(), MemAttrs::DEFAULT)
                .expect("a 32-bit aligned write is legal");
        }

        fn peek(&self, offset: u64) -> u32 {
            let mut bytes = [0u8; 4];
            self.io
                .regs
                .read(offset, &mut bytes, MemAttrs::DEFAULT)
                .expect("a 32-bit aligned read is legal");
            u32::from_le_bytes(bytes)
        }

        /// Program one redirection entry, low half then high half.
        fn program(&self, line: usize, low: u32, dest: u8) {
            self.write_indirect(IDX_REDIR + 2 * line as u8 + 1, u32::from(dest) << 24);
            self.write_indirect(IDX_REDIR + 2 * line as u8, low);
        }

        fn drive(&self, line: usize, level: Level) {
            self.pins[line].set_level(self.src, line as u32, level);
        }

        /// The local APIC's request register bit for `vector`.
        fn requested(&self, vector: u8) -> bool {
            let word = read_lapic(&self.lapic, 0x200 + 0x10 * u64::from(vector >> 5));
            word & (1 << (vector & 31)) != 0
        }

        fn ack(&self) -> IntAckResponse {
            self.lapic
                .int_ack("intr")
                .expect("a local APIC answers the acknowledge")
                .acknowledge(IntAckCycle::vector_only())
        }

        fn eoi(&self) {
            write_lapic(&self.lapic, 0x0b0, 0);
        }
    }

    #[test]
    fn the_version_register_says_how_many_inputs_there_are() {
        let b = bench();
        let version = b.read_indirect(IDX_VERSION);
        assert_eq!(version & 0xff, VERSION, "an 82093AA");
        assert_eq!(
            (version >> 16) & 0xff,
            INPUTS as u32 - 1,
            "twenty-four inputs, reported as the highest entry"
        );
    }

    #[test]
    fn the_identification_register_keeps_four_bits() {
        let b = bench();
        b.write_indirect(IDX_ID, 0xff << 24);
        assert_eq!(
            b.read_indirect(IDX_ID) >> 24,
            0x0f,
            "the 82093AA carries the ID in bits 27:24 (datasheet 3.2.1)"
        );
    }

    #[test]
    fn every_entry_comes_out_of_reset_masked() {
        let b = bench();
        for line in 0..INPUTS {
            assert_eq!(
                b.read_indirect(IDX_REDIR + 2 * line as u8) & (ENTRY_MASK as u32),
                ENTRY_MASK as u32,
                "entry {line}"
            );
        }
        b.drive(LINE, Level::High);
        assert!(!b.requested(VECTOR), "so nothing gets through");
    }

    #[test]
    fn an_edge_entry_sends_once_per_rising_edge() {
        let b = bench();
        b.program(LINE, u32::from(VECTOR), 0);
        b.drive(LINE, Level::High);
        assert!(b.requested(VECTOR), "the edge became a message");
        assert_eq!(b.ack(), IntAckResponse::Vector(u32::from(VECTOR)));
        assert!(!b.requested(VECTOR));

        // Still high, and that is not another edge.
        b.eoi();
        assert!(
            !b.requested(VECTOR),
            "a level that never fell sends nothing"
        );

        b.drive(LINE, Level::Low);
        b.drive(LINE, Level::High);
        assert!(b.requested(VECTOR), "and a fresh edge does");
    }

    #[test]
    fn a_level_entry_holds_its_remote_irr_until_the_end_of_interrupt() {
        let b = bench();
        b.program(LINE, (ENTRY_LEVEL as u32) | u32::from(VECTOR), 0);
        b.drive(LINE, Level::High);
        assert!(b.requested(VECTOR));
        assert_eq!(
            b.io.entry(LINE).unwrap() & ENTRY_REMOTE_IRR,
            ENTRY_REMOTE_IRR,
            "the entry latched that it had sent"
        );

        // The processor takes it. The line is still asserted, and the entry
        // must not flood: remote IRR is what stops it.
        assert_eq!(b.ack(), IntAckResponse::Vector(u32::from(VECTOR)));
        assert!(!b.requested(VECTOR));

        // The end-of-interrupt is broadcast to this part, which drops remote
        // IRR and, the line still being asserted, sends again. That is a shared
        // level-triggered interrupt working.
        b.eoi();
        assert!(
            b.requested(VECTOR),
            "still asserting, so it interrupts again"
        );
        assert_eq!(
            b.io.entry(LINE).unwrap() & ENTRY_REMOTE_IRR,
            ENTRY_REMOTE_IRR
        );

        // Once the device stops asserting, the next end-of-interrupt is the
        // last of it.
        assert_eq!(b.ack(), IntAckResponse::Vector(u32::from(VECTOR)));
        b.drive(LINE, Level::Low);
        b.eoi();
        assert!(!b.requested(VECTOR));
        assert_eq!(b.io.entry(LINE).unwrap() & ENTRY_REMOTE_IRR, 0);
    }

    #[test]
    fn unmasking_a_level_entry_whose_line_is_already_high_delivers() {
        // The wire contract: the condition was true the whole time, and nothing
        // else will ever announce it (`ROADMAP.md` 4.3).
        let b = bench();
        b.program(
            LINE,
            (ENTRY_MASK as u32) | (ENTRY_LEVEL as u32) | u32::from(VECTOR),
            0,
        );
        b.drive(LINE, Level::High);
        assert!(!b.requested(VECTOR), "masked, so nothing yet");
        b.write_indirect(
            IDX_REDIR + 2 * LINE as u8,
            (ENTRY_LEVEL as u32) | u32::from(VECTOR),
        );
        assert!(b.requested(VECTOR), "and unmasking delivers it");
    }

    #[test]
    fn unmasking_an_edge_entry_whose_line_is_already_high_does_not() {
        // The other half of the same contract: an edge is an event, and the
        // event is over. Inventing one here is the bug this asserts against.
        let b = bench();
        b.program(LINE, (ENTRY_MASK as u32) | u32::from(VECTOR), 0);
        b.drive(LINE, Level::High);
        b.write_indirect(IDX_REDIR + 2 * LINE as u8, u32::from(VECTOR));
        assert!(!b.requested(VECTOR));
    }

    #[test]
    fn an_active_low_entry_reads_the_pin_the_other_way_up() {
        let b = bench();
        b.program(
            LINE,
            (ENTRY_ACTIVE_LOW as u32) | (ENTRY_LEVEL as u32) | u32::from(VECTOR),
            0,
        );
        // The entry was written while the pin sat low, which for an active-low
        // input *is* asserted, so it delivers on the spot.
        assert!(b.requested(VECTOR));
        assert_eq!(b.ack(), IntAckResponse::Vector(u32::from(VECTOR)));
        b.drive(LINE, Level::High);
        b.eoi();
        assert!(!b.requested(VECTOR), "and a high pin is idle");
    }

    #[test]
    fn a_message_reaches_the_local_apic_the_destination_field_names() {
        let bus = Arc::new(ApicBus::new());
        let io = IoApic::with_bus(0, INPUTS, Arc::clone(&bus));
        let zero = LocalApic::with_bus(0, true, Arc::clone(&bus));
        let one = LocalApic::with_bus(1, false, Arc::clone(&bus));
        realize(&io);
        realize(&zero);
        realize(&one);
        write_lapic(&zero, 0x0f0, 0x1ff);
        write_lapic(&one, 0x0f0, 0x1ff);

        let ids = WireIdAllocator::new();
        let src = ids.alloc();
        let pin = io.sink("irq1", &[src]).unwrap().sink;

        // Written the long way, since the helper lives on `Bench`.
        for (index, value) in [
            (IDX_REDIR + 3, 1u32 << 24),
            (IDX_REDIR + 2, u32::from(VECTOR)),
        ] {
            io.regs
                .write(IOREGSEL, &u32::from(index).to_le_bytes(), MemAttrs::DEFAULT)
                .unwrap();
            io.regs
                .write(IOWIN, &value.to_le_bytes(), MemAttrs::DEFAULT)
                .unwrap();
        }
        pin.set_level(src, 1, Level::High);

        let word = read_lapic(&one, 0x200 + 0x10 * u64::from(VECTOR >> 5));
        assert_ne!(word & (1 << (VECTOR & 31)), 0, "APIC 1 has it");
        let word = read_lapic(&zero, 0x200 + 0x10 * u64::from(VECTOR >> 5));
        assert_eq!(word & (1 << (VECTOR & 31)), 0, "and APIC 0 does not");
    }

    #[test]
    fn a_debug_write_is_refused_and_a_debug_read_is_harmless() {
        let b = bench();
        b.write_indirect(IDX_VERSION, 0);
        // The index register is half of a protocol the guest is in the middle
        // of; a debugger that moved it would break the next write.
        assert!(
            b.io.regs
                .write(IOREGSEL, &0u32.to_le_bytes(), MemAttrs::DEBUG)
                .is_err()
        );
        let mut bytes = [0u8; 4];
        b.io.regs
            .read(IOWIN, &mut bytes, MemAttrs::DEBUG)
            .expect("but reading the window is free");
        assert_eq!(u32::from_le_bytes(bytes) & 0xff, VERSION);
        assert_eq!(
            b.peek(IOREGSEL),
            u32::from(IDX_VERSION),
            "and moved nothing"
        );
    }

    #[test]
    fn the_window_decodes_two_registers_and_nothing_between_them() {
        let b = bench();
        let mut bytes = [0u8; 4];
        assert!(b.io.regs.read(0x04, &mut bytes, MemAttrs::DEFAULT).is_err());
        assert!(b.io.regs.read(0x18, &mut bytes, MemAttrs::DEFAULT).is_err());
    }

    #[test]
    fn a_snapshot_round_trips_the_whole_part() {
        let saved = bench();
        saved.program(LINE, (ENTRY_LEVEL as u32) | u32::from(VECTOR), 2);
        saved.program(9, u32::from(VECTOR) + 1, 0);
        saved.drive(LINE, Level::High);
        saved.write_indirect(IDX_ID, 0x0e << 24);
        saved.poke(IOREGSEL, u32::from(IDX_REDIR + 4));

        let mut shape = MachineShape::new();
        shape.add_device("ioapic", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("ioapic", CLASS.name, CLASS.version).unwrap();
            saved.io.save(&mut chunk).unwrap();
        }
        let bytes = w.to_vec().unwrap();

        let restored = bench();
        let reader = StateReader::new(&bytes).unwrap();
        let chunk = reader
            .load("ioapic", CLASS.name, CLASS.version, &Migrations::new())
            .unwrap();
        restored.io.load(&mut chunk.reader()).unwrap();

        // Copied out one at a time: two parts' state locks are both at
        // `LockRank::DEVICE`.
        let after = restored.io.regs.state.lock().clone();
        let before = saved.io.regs.state.lock().clone();
        assert_eq!(after, before, "every field came back");
        assert!(
            !restored.requested(VECTOR),
            "and the interrupt already sent was not sent again"
        );

        let mut shape = MachineShape::new();
        shape.add_device("ioapic", CLASS.name).unwrap();
        let mut w = StateWriter::new(shape);
        {
            let mut chunk = w.chunk("ioapic", CLASS.name, CLASS.version).unwrap();
            restored.io.save(&mut chunk).unwrap();
        }
        assert_eq!(w.to_vec().unwrap(), bytes);
    }

    #[test]
    fn a_reset_masks_every_entry_again() {
        let b = bench();
        b.program(LINE, u32::from(VECTOR), 0);
        b.io.reset(ResetKind::Cold);
        assert_eq!(b.io.entry(LINE).unwrap(), ENTRY_MASK);
        b.drive(LINE, Level::High);
        assert!(!b.requested(VECTOR));
    }
}
