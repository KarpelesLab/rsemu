//! Wires: single-bit signal lines between devices (`ROADMAP.md` §4.3).
//!
//! A wire is the generic mechanism behind interrupt requests, GPIO pins, reset
//! lines, DMA request/acknowledge pairs and card-detect switches. The core has
//! no concept of an "interrupt": an i8259, an APIC, a GIC or the NES NMI line
//! are ordinary devices that happen to own [`WireSink`]s and [`WireSource`]s.
//!
//! # Why `set_level` carries the source
//!
//! ```text
//! apu.irq  ─┐
//!           ├─► cpu.irq     (wired-OR: the CPU sees IRQ while *either* asserts)
//! cart.irq ─┘
//! ```
//!
//! [`WireSink::set_level`] takes the asserting source's [`WireId`], and that
//! parameter is the whole design. Without it, a sink told "low" cannot know
//! whether the *other* driver is still asserting, so it drops a line that must
//! stay high — the classic shared-interrupt bug, and one that is unfixable
//! after the fact because the information was never passed. rsemu makes the
//! sink track which sources assert; [`FanIn`] is that bookkeeping, ready made,
//! and [`WireOr`] / [`WireAnd`] are it wrapped in a device so a machine
//! description can also make fan-in explicit.
//!
//! # Level and edge
//!
//! Wires carry levels. Edge semantics come from [`LevelToEdge`], a device that
//! *remembers the previous level* rather than a flag hidden inside a consumer,
//! so the remembered level lands in a snapshot like any other architectural
//! state (`ROADMAP.md` §4.5, invariant 6). It emits a transient pulse on its
//! output wire; [`EdgeLatch`] is the matching consumer.
//!
//! # Tri-state, and who resolves a net
//!
//! A [`Level`] is two-valued and stays that way: an input pin always reads
//! *some* level. An output **stage** has a third state — not driving — and
//! without it there is no open-drain bus, no `PUPDR` and no keypad matrix,
//! because "nobody is holding this line" cannot be said. So a driver presents a
//! [`Drive`]: Hi-Z, a pull resistor, or a stage at one of the rails.
//!
//! Who turns that into a level is a property of the **net**, and a machine file
//! picks it per net ([`NetMode`]):
//!
//! * [`NetMode::PerSink`], the default and what every interrupt line here is:
//!   the wire delivers the changed driver's own level and each sink resolves
//!   its own [`FanIn`], as described above.
//! * [`NetMode::Resolved`]: the wire resolves — strength beats polarity, the
//!   net's own [`Pull`] decides when every stage has let go, and opposition at
//!   equal strength is a fault that [`Wire::contention`] *counts* rather than
//!   guesses at — and hands every sink the resolved level, once per source. A
//!   sink that keeps a [`FanIn`] therefore stays correct without being taught
//!   any of this, since a fan-in whose every entry holds one level returns it
//!   under either [`Resolve`].
//!
//! Nothing about a net's resolution is state: it is a function of the drives, so
//! a net stays derived (`ROADMAP.md` §4.5) and a snapshot carries each driver's
//! [`Drive`] and nothing else.
//!
//! # Re-entrancy and cycles
//!
//! A sink notified of a level change may drive another wire from inside
//! `set_level` — an interrupt controller recomputing its output is exactly
//! that — so propagation is inherently re-entrant. Per CLAUDE.md's re-entrancy
//! contract, a [`Wire`] holds no lock across the outward call: its state is
//! atomic and is updated before delivery starts.
//!
//! Delivery is *iterative, not recursive*. A wire that is already delivering
//! records the new level, marks it pending and returns immediately; the
//! outermost delivery re-runs until the wire is quiescent. A cycle therefore
//! costs stack depth proportional to the length of the cycle rather than to the
//! number of times it goes round, and a genuine combinational loop (a
//! [`WireNot`] feeding its own input) terminates after [`Wire::SETTLE_LIMIT`]
//! passes with [`Wire::unsettled`] incremented, instead of overflowing the
//! stack. Such a loop is a machine-description error the resolver is expected
//! to reject; the runtime merely refuses to hang. Stack depth for an *acyclic*
//! graph is bounded by its depth, which is a property of the machine file.
//!
//! # Construction and ownership
//!
//! Sources and sinks are fixed when the wire is built, matching two-phase
//! device construction (`ROADMAP.md` §4.4): the resolver knows the whole graph,
//! so it knows every driver of every net. That is what lets the per-source
//! state be a plain array of atomics — no lock, no allocation while the machine
//! runs, and `Send + Sync` with no `unsafe` anywhere. The realize order is
//! therefore: construct devices, build the wires (naming each device as a
//! sink), then hand each device the [`WireSource`] it drives. Re-plugging a
//! wire (hot-plug) builds a new one and swaps it in.
//!
//! A device that both drives and listens — an interrupt controller with a
//! request line out and an acknowledge line in — closes an `Arc` cycle through
//! its wires, which would leak on teardown. [`WireBuilder::sink_weak`] is the
//! break: the machine owns the devices, the wire only refers to them.
//!
//! # Notes for later phases
//!
//! `core::device`, `core::state` and `core::sync` are still stubs, so the
//! combinators here are plain structs rather than `impl Device`, and snapshots
//! are exchanged as `Vec<(WireId, Level)>` rather than through a `StateWriter`.
//! Both are mechanical to retrofit; the state each object holds is already
//! separated from its diagnostics. Nothing here names `std::sync` — the wires
//! themselves use only `core::sync::atomic`, which every target has
//! (invariant 4), and the one lock in this module ([`IntAckHandlers`], a list
//! written at realize time and read on an interrupt) is a leaf-ranked
//! `core::sync::Mutex`.

use crate::core::sync::Mutex;
use alloc::boxed::Box;
use alloc::sync::{Arc, Weak};
use alloc::vec::Vec;
use core::fmt;
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering::SeqCst};

/// The state of a signal line.
///
/// Deliberately not `bool`: half the lines in a real machine are active-low,
/// and `Level::Low` at a call site says which end of the wire is meant where
/// `false` would not.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Level {
    /// Deasserted for an active-high line, asserted for an active-low one.
    #[default]
    Low,
    /// Asserted for an active-high line, deasserted for an active-low one.
    High,
}

impl Level {
    /// `High` for `true`, `Low` for `false`.
    #[inline]
    pub const fn from_bool(b: bool) -> Level {
        if b { Level::High } else { Level::Low }
    }

    /// `true` for `High`.
    #[inline]
    pub const fn as_bool(self) -> bool {
        matches!(self, Level::High)
    }

    /// Whether this is [`Level::High`].
    #[inline]
    pub const fn is_high(self) -> bool {
        matches!(self, Level::High)
    }

    /// Whether this is [`Level::Low`].
    #[inline]
    pub const fn is_low(self) -> bool {
        matches!(self, Level::Low)
    }

    /// The opposite level.
    #[inline]
    pub const fn inverted(self) -> Level {
        match self {
            Level::Low => Level::High,
            Level::High => Level::Low,
        }
    }
}

impl From<bool> for Level {
    #[inline]
    fn from(b: bool) -> Level {
        Level::from_bool(b)
    }
}

impl From<Level> for bool {
    #[inline]
    fn from(l: Level) -> bool {
        l.as_bool()
    }
}

/// A transition of a signal line.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Edge {
    /// Low to high.
    Rising,
    /// High to low.
    Falling,
}

impl Edge {
    /// The edge described by a transition, or `None` if the level did not move.
    #[inline]
    pub const fn between(from: Level, to: Level) -> Option<Edge> {
        match (from, to) {
            (Level::Low, Level::High) => Some(Edge::Rising),
            (Level::High, Level::Low) => Some(Edge::Falling),
            _ => None,
        }
    }
}

/// Which edges a detector reacts to.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum EdgeTrigger {
    /// Low-to-high only. The common interrupt case.
    #[default]
    Rising,
    /// High-to-low only, as for an active-low line.
    Falling,
    /// Either transition.
    Both,
}

impl EdgeTrigger {
    /// Whether `edge` fires this trigger.
    #[inline]
    pub const fn matches(self, edge: Edge) -> bool {
        matches!(
            (self, edge),
            (EdgeTrigger::Both, _)
                | (EdgeTrigger::Rising, Edge::Rising)
                | (EdgeTrigger::Falling, Edge::Falling)
        )
    }
}

/// How several drivers of one line combine.
///
/// Named after the electrical arrangement it models: `Or` is the wired-OR of
/// active-high drivers, `And` the wired-AND of open-drain drivers sharing a
/// pull-up. With no drivers at all the line sits at [`Resolve::idle`] — low for
/// `Or`, high for `And`, which is the pull-up.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum Resolve {
    /// The line is high while *any* source drives it high.
    #[default]
    Or,
    /// The line is high only while *every* source drives it high.
    And,
}

impl Resolve {
    /// The level of a line with no sources.
    #[inline]
    pub const fn idle(self) -> Level {
        match self {
            Resolve::Or => Level::Low,
            Resolve::And => Level::High,
        }
    }
}

/// What one driver puts on a net, as an output stage rather than as a level.
///
/// [`Level`] is what a net *is at*; this is what a driver *does to it*, and the
/// two are not the same thing. A CMOS push-pull stage holds the net at one rail
/// or the other; an open-drain stage either holds it low or lets go entirely;
/// a pull resistor holds it weakly and loses to anything actually driving.
/// Without the distinction there is no open-drain bus, no `PUPDR` and no keypad
/// matrix, because "not driving" cannot be said.
///
/// `Level` deliberately stays two-valued. A sink is a piece of silicon with an
/// input pin, and an input pin always reads *some* level: the third state lives
/// on the driver's side of the net and is resolved away before anything sees it
/// ([`Wire::resolve_net`]). A third `Level` variant would instead have handed a
/// Hi-Z to eight hundred call sites that read it as low.
///
/// Strength beats polarity: a strong driver overrules every weak one, and a net
/// with nothing but weak drivers settles where they say. Two drivers of equal
/// strength pulling opposite ways is a *fault* — a short, in hardware — and
/// [`Wire::contention`] counts it; [`Wire::resolve_net`] says what the model
/// answers meanwhile.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum Drive {
    /// Not driving: high impedance. An input pin, an analogue pin, an
    /// open-drain stage with a one in its output register, an open contact.
    #[default]
    HiZ,
    /// A pull-down resistor. Loses to any strong driver.
    WeakLow,
    /// A pull-up resistor. Loses to any strong driver. The one a keypad matrix
    /// is built on.
    WeakHigh,
    /// An output stage holding the net low.
    Low,
    /// An output stage holding the net high.
    High,
}

impl Drive {
    /// The strong drive of `level`: a push-pull output stage.
    #[inline]
    pub const fn strong(level: Level) -> Drive {
        match level {
            Level::Low => Drive::Low,
            Level::High => Drive::High,
        }
    }

    /// The weak drive of `level`: a pull resistor.
    #[inline]
    pub const fn weak(level: Level) -> Drive {
        match level {
            Level::Low => Drive::WeakLow,
            Level::High => Drive::WeakHigh,
        }
    }

    /// An open-drain stage asked for `level`: it pulls low, or lets go.
    ///
    /// The whole of `OTYPER = 1` (ST RM0090 §8.3.10) and of an I²C driver.
    #[inline]
    pub const fn open_drain(level: Level) -> Drive {
        match level {
            Level::Low => Drive::Low,
            Level::High => Drive::HiZ,
        }
    }

    /// The level this driver is holding, or `None` when it is not driving.
    #[inline]
    pub const fn level(self) -> Option<Level> {
        match self {
            Drive::HiZ => None,
            Drive::Low | Drive::WeakLow => Some(Level::Low),
            Drive::High | Drive::WeakHigh => Some(Level::High),
        }
    }

    /// Whether this drives nothing at all.
    #[inline]
    pub const fn is_hiz(self) -> bool {
        matches!(self, Drive::HiZ)
    }

    /// Whether this is an output stage rather than a resistor.
    #[inline]
    pub const fn is_strong(self) -> bool {
        matches!(self, Drive::Low | Drive::High)
    }

    /// Whether this is a pull resistor.
    #[inline]
    pub const fn is_weak(self) -> bool {
        matches!(self, Drive::WeakLow | Drive::WeakHigh)
    }

    /// The encoding used for an atomic slot and for a snapshot.
    ///
    /// `Low` is zero and `High` is one, so a net built before anything drives
    /// it still reads low, and a snapshot written when the byte meant a bare
    /// [`Level`] still decodes to what it meant.
    #[inline]
    pub const fn code(self) -> u8 {
        match self {
            Drive::Low => 0,
            Drive::High => 1,
            Drive::HiZ => 2,
            Drive::WeakLow => 3,
            Drive::WeakHigh => 4,
        }
    }

    /// Decode [`Drive::code`].
    ///
    /// `None` for a byte this build does not know, which is how a snapshot from
    /// a later version is diagnosed rather than silently taken as low.
    #[inline]
    pub const fn from_code(code: u8) -> Option<Drive> {
        match code {
            0 => Some(Drive::Low),
            1 => Some(Drive::High),
            2 => Some(Drive::HiZ),
            3 => Some(Drive::WeakLow),
            4 => Some(Drive::WeakHigh),
            _ => None,
        }
    }
}

impl From<Level> for Drive {
    #[inline]
    fn from(level: Level) -> Drive {
        Drive::strong(level)
    }
}

/// The resistor a net carries in its own right — a component on the board,
/// rather than inside any of the parts the net joins.
///
/// A pull-up on a printed circuit board is a fact about the *net*, not about
/// anything connected to it, which is why it belongs to the wire and is not a
/// fifth driver somebody has to invent a device to hold. It resolves exactly as
/// a weak [`Drive`] does, and fights an on-chip pull of the opposite polarity on
/// equal terms, because that is what two resistors do.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum Pull {
    /// No resistor. A net whose drivers have all let go then has no level of
    /// its own; [`Wire::resolve_net`] says what the model answers.
    #[default]
    None,
    /// A pull-up: the net idles high.
    Up,
    /// A pull-down: the net idles low.
    Down,
}

impl Pull {
    /// The weak drive this resistor contributes, if any.
    #[inline]
    pub const fn drive(self) -> Drive {
        match self {
            Pull::None => Drive::HiZ,
            Pull::Up => Drive::WeakHigh,
            Pull::Down => Drive::WeakLow,
        }
    }

    /// The word a machine file writes, or `None` for one that is not a pull.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Pull> {
        match name {
            "none" | "float" => Some(Pull::None),
            "up" => Some(Pull::Up),
            "down" => Some(Pull::Down),
            _ => None,
        }
    }

    /// The word a machine file writes for this pull.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Pull::None => "none",
            Pull::Up => "up",
            Pull::Down => "down",
        }
    }
}

/// Who turns a net's drivers into the level its sinks are told about.
///
/// Two answers, and a machine file picks one per net.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub enum NetMode {
    /// **Each sink does it.** The wire delivers the changed driver's own level,
    /// unresolved, and a sink with several drivers keeps a [`FanIn`] and
    /// combines them under its own [`Resolve`]. Every interrupt line in the
    /// tree works this way and it stays the default: a shared `/IRQ` is a
    /// wired-OR whichever end does the OR-ing, and the sink is the end that
    /// knows the polarity.
    #[default]
    PerSink,
    /// **The wire does it.** Drivers are tri-state ([`Drive`]), the net carries
    /// a [`Pull`] of its own, and every sink is told the *resolved* level.
    ///
    /// Wanted as soon as "nobody is driving" is a state distinct from "somebody
    /// is driving low": an open-drain bus, a GPIO pad whose `PUPDR` decides what
    /// an unconnected pin reads, a keypad matrix. A sink that keeps a [`FanIn`]
    /// stays correct without being changed, because the net hands it the
    /// resolved level *for every one of its sources* — so any [`Resolve`] over
    /// them returns that same level.
    Resolved(Pull),
}

impl NetMode {
    /// The net's own resistor; [`Pull::None`] for a per-sink net.
    #[inline]
    pub const fn pull(self) -> Pull {
        match self {
            NetMode::PerSink => Pull::None,
            NetMode::Resolved(pull) => pull,
        }
    }

    /// Whether the wire resolves rather than its sinks.
    #[inline]
    pub const fn is_resolved(self) -> bool {
        matches!(self, NetMode::Resolved(_))
    }
}

/// The identity of a wire source.
///
/// Every driver of a line has one, and it travels with each level change so a
/// sink can tell its drivers apart. Ids are assigned by the machine resolver —
/// [`WireIdAllocator`] is the counter it uses — and are unique within a
/// machine, not globally.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct WireId(
    /// The raw identifier. Public so a machine builder can assign ids from its
    /// own numbering and so ids are usable in `const` context.
    pub u64,
);

impl WireId {
    /// The reserved "no source" id, for a level with no meaningful origin: a
    /// test stimulus, a reset default.
    pub const NONE: WireId = WireId(0);

    /// An id with the given raw value.
    #[inline]
    pub const fn new(raw: u64) -> WireId {
        WireId(raw)
    }

    /// The raw value.
    #[inline]
    pub const fn raw(self) -> u64 {
        self.0
    }
}

/// A monotonic [`WireId`] counter.
///
/// One allocator per machine, never a global: ids must be reproducible for a
/// given machine description, and a process-wide counter would make them depend
/// on how many machines were built earlier (the determinism rule in CLAUDE.md).
#[derive(Debug)]
pub struct WireIdAllocator {
    next: AtomicUsize,
}

impl WireIdAllocator {
    /// A fresh allocator. The first id handed out is `WireId(1)`;
    /// [`WireId::NONE`] is never allocated.
    pub fn new() -> Self {
        WireIdAllocator {
            next: AtomicUsize::new(1),
        }
    }

    /// The next unused id.
    pub fn alloc(&self) -> WireId {
        WireId(self.next.fetch_add(1, SeqCst) as u64)
    }
}

impl Default for WireIdAllocator {
    fn default() -> Self {
        // Not derivable: a derived `Default` would start at zero and hand out
        // `WireId::NONE`.
        WireIdAllocator::new()
    }
}

/// Something that can be told a signal line changed.
///
/// `src` names the driver, `line` is the sink's own input pin number (chosen
/// when the sink was connected, so one device can host many inputs), and
/// `level` is that driver's new level — **not** the resolved level of the net.
/// A sink with more than one driver resolves them itself, normally by keeping a
/// [`FanIn`].
///
/// Implementations must be cheap and must not block: `set_level` runs inside
/// the caller's propagation. Driving another wire from within is allowed and
/// expected; see the module docs on re-entrancy.
pub trait WireSink: Send + Sync {
    /// Called when `src` changes the level it drives on this sink's `line`.
    fn set_level(&self, src: WireId, line: u32, level: Level);
}

/// The reverse half of a vectored interrupt line.
///
/// A [`WireSink`] carries a *level*, which is all an edge-triggered NMI or a
/// 6502's `/IRQ` ever needs. A vectored controller needs one thing more: when
/// the CPU decides to take the interrupt it runs an **acknowledge cycle**, and
/// the controller drives a vector back along the same piece of copper — that is
/// what the 8259A's two `INTA` pulses are, and what a GIC's `IAR` read is.
///
/// The direction matters. Without it a controller never learns that its request
/// was taken, so it cannot move the request from "pending" to "in service", and
/// end-of-interrupt has nothing to clear. Modelling it as a latched byte the
/// controller writes ahead of time gets the vector right and the priority
/// bookkeeping wrong.
///
/// So the acknowledge travels with the net rather than through a device handle:
/// the driving device offers one with [`Device::int_ack`](crate::core::device::Device::int_ack), the realizer hands
/// it to every sink on that net with [`Device::attach_int_ack`](crate::core::device::Device::attach_int_ack), and the sink
/// keeps a [`Weak`] reference — the machine owns devices and
/// a wire merely refers to them (§4.3's weak edge), so a CPU holding its
/// controller alive would be a cycle nothing could drop.
///
/// The core knows nothing about 8259As or GICs: this is a bus concept, like
/// [`WireSink`] itself.
pub trait IntAck: Send + Sync + fmt::Debug {
    /// The CPU has taken the interrupt and is running its acknowledge cycle.
    /// Report what this controller drives back, and apply whatever the cycle
    /// changes inside it.
    ///
    /// `cycle` is what the processor puts on the bus while it asks — a 68000
    /// presents the interrupt level on A3-A1, an 8086 presents nothing at all
    /// — so a controller can tell "you, at level 5" from "you, whoever you
    /// are". Answer [`IntAckResponse::Declined`] when the cycle is not this
    /// controller's, which is not the same as being asked and having no vector
    /// to give ([`IntAckResponse::Autovector`]); see [`IntAckResponse`].
    ///
    /// Called from the CPU's execution path with no device lock held on the
    /// CPU's side, so an implementation is free to take its own. It runs at
    /// most once per interrupt taken *per controller on the net*, and it must
    /// be prepared to be called when nothing is pending any more — a request
    /// can go away between the moment the CPU samples the pin and the moment
    /// it acknowledges, and every real controller answers that with a defined
    /// vector (the 8259A's spurious `IR7`).
    fn acknowledge(&self, cycle: IntAckCycle) -> IntAckResponse;
}

/// Which acknowledge handshake a processor runs — the shape of the cycle, not
/// the identity of the CPU.
///
/// An extensible enumeration rather than a Rust `enum` (CLAUDE.md, "type
/// conventions") on purpose: a controller normally asks
/// [`IntAckCycle::level`] and never looks at the kind at all, and the day a
/// fourth processor family arrives with a fifth thing to present, no
/// implementor of [`IntAck`] should have to be edited to keep compiling.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct IntAckKind(pub u16);

impl IntAckKind {
    /// The cycle presents nothing: "your request was taken, name your vector".
    ///
    /// The 8086's `INTA` pair and the 8259A that answers them. A cascade is
    /// still expressible — one controller delegating to another is a property
    /// of the wiring, not of the cycle.
    pub const VECTOR: IntAckKind = IntAckKind(0);
    /// The cycle presents a **priority level**, in [`IntAckCycle::level`].
    ///
    /// The 68000 drives the level being acknowledged on A3-A1 in CPU space,
    /// and every controller on the net decides whether it is the one being
    /// asked.
    pub const LEVEL: IntAckKind = IntAckKind(1);
    /// The interrupting device drives a **byte on the data bus**, and the CPU
    /// makes of it what its current mode says.
    ///
    /// The Z80: mode 2 combines the byte with `I` to address a vector table,
    /// mode 0 executes it as an opcode, and mode 1 ignores it entirely. The
    /// mode rides in [`IntAckCycle::mode`], because a daisy-chained peripheral
    /// answering in a machine running mode 1 is answering into the void and is
    /// entitled to know.
    pub const DATA_BUS: IntAckKind = IntAckKind(2);
}

/// What an acknowledge cycle presents to the controllers on the net.
///
/// One word: a [`kind`](IntAckCycle::kind) and a detail the kind interprets.
/// The typed accessors are the interface — [`level`](IntAckCycle::level) is
/// `None` on a machine whose acknowledge presents no level, which is exactly
/// the distinction a bare integer argument could not make.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct IntAckCycle {
    kind: IntAckKind,
    detail: u16,
}

impl IntAckCycle {
    /// A cycle of `kind`, carrying that kind's detail word.
    ///
    /// Prefer the named constructors; this is the escape hatch for a kind
    /// added after this file was written.
    #[inline]
    pub const fn new(kind: IntAckKind, detail: u16) -> IntAckCycle {
        IntAckCycle { kind, detail }
    }

    /// An [`IntAckKind::VECTOR`] cycle: nothing presented, answer with a
    /// vector.
    #[inline]
    pub const fn vector_only() -> IntAckCycle {
        IntAckCycle::new(IntAckKind::VECTOR, 0)
    }

    /// An [`IntAckKind::LEVEL`] cycle acknowledging `level`.
    #[inline]
    pub const fn at_level(level: u8) -> IntAckCycle {
        IntAckCycle::new(IntAckKind::LEVEL, level as u16)
    }

    /// An [`IntAckKind::DATA_BUS`] cycle run by a CPU in interrupt `mode`.
    #[inline]
    pub const fn data_bus(mode: u8) -> IntAckCycle {
        IntAckCycle::new(IntAckKind::DATA_BUS, mode as u16)
    }

    /// Which handshake this is.
    #[inline]
    pub const fn kind(self) -> IntAckKind {
        self.kind
    }

    /// The raw detail word, for a kind this build does not name.
    #[inline]
    pub const fn detail(self) -> u16 {
        self.detail
    }

    /// The priority level being acknowledged, or `None` when the cycle
    /// presents none.
    ///
    /// A controller wired to one `IPL` encoding compares this with its own and
    /// [declines](IntAckResponse::Declined) when they differ.
    #[inline]
    pub const fn level(self) -> Option<u8> {
        match self.kind {
            IntAckKind::LEVEL => Some(self.detail as u8),
            _ => None,
        }
    }

    /// The interrupt mode the CPU will interpret the answer in, or `None` when
    /// the cycle carries no mode.
    #[inline]
    pub const fn mode(self) -> Option<u8> {
        match self.kind {
            IntAckKind::DATA_BUS => Some(self.detail as u8),
            _ => None,
        }
    }
}

/// What a controller drives back during an acknowledge cycle.
///
/// A real `enum`, and this is the case CLAUDE.md means by "exhaustiveness is
/// genuinely wanted": the outcomes are what *terminates a bus cycle*, every
/// CPU has to do something different with each of them, and there is no
/// sensible fallback arm — a fourth outcome must be a compile error in the
/// three cores rather than silently take some other branch. The extensible
/// half of the seam is [`IntAckKind`], on the other side of the call, where
/// the implementors are many and the additions happen.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum IntAckResponse {
    /// Not mine: this controller is not the one being asked, and drives
    /// nothing. The cycle passes to the next controller on the net.
    ///
    /// A 68000 controller answers this when the level on A3-A1 is not its own.
    Declined,
    /// Mine, but I have no vector: the CPU must synthesise one.
    ///
    /// The 68000's `VPA`, which selects `AUTOVECTOR_BASE + level`. This
    /// **terminates** the cycle — a controller that asserts `VPA` has answered,
    /// and the controllers behind it never see the acknowledge.
    Autovector,
    /// Mine, and here is the vector.
    ///
    /// A vector number for a 68000 or an x86, the byte on the data bus for a
    /// Z80. Widened to `u32` for a controller whose answer indexes something
    /// larger, such as a GIC's `IAR`.
    Vector(u32),
}

impl IntAckResponse {
    /// The vector supplied, if one was.
    #[inline]
    pub const fn vector(self) -> Option<u32> {
        match self {
            IntAckResponse::Vector(vector) => Some(vector),
            _ => None,
        }
    }

    /// Whether this controller took the cycle, by any answer.
    #[inline]
    pub const fn answered(self) -> bool {
        !matches!(self, IntAckResponse::Declined)
    }
}

/// The controllers that answer one processor's acknowledge cycle, in the order
/// they were attached.
///
/// A CPU input pin is a net, and a machine can have **several** controllers
/// answering one processor: two 68000 interrupt controllers on different `IPL`
/// pins, or a Z80 daisy chain. So this is a list, not a slot, and an
/// acknowledge is offered to each in turn until one does not
/// [decline](IntAckResponse::Declined) — which is what the priority daisy chain
/// does in hardware, with attach order standing in for physical order.
/// Deterministic, because the realizer attaches in machine-file order
/// (CLAUDE.md, "determinism").
///
/// References are [`Weak`], always: the machine owns devices and a wire merely
/// refers to them (§4.3's weak edge), so a CPU that kept its controller alive
/// would close a cycle nothing could drop. A dead one is skipped.
///
/// The lock is a leaf, and is **released before each outward call** — the
/// re-entrancy contract forbids holding one across a call into another device,
/// and a controller answering an acknowledge drops its own request line, which
/// lands straight back on this CPU's pin.
#[derive(Debug, Default)]
pub struct IntAckHandlers {
    handlers: Mutex<Vec<Weak<dyn IntAck>>>,
}

impl IntAckHandlers {
    /// An empty list: nothing answers, so every cycle is declined.
    #[must_use]
    pub const fn new() -> IntAckHandlers {
        IntAckHandlers {
            handlers: Mutex::new(Vec::new()),
        }
    }

    /// Add a controller, at the end of the order.
    ///
    /// Attaching the same controller twice is a no-op: a 68000 controller that
    /// encodes level 5 drives `IPL0` and `IPL2`, so the realizer offers it on
    /// both nets, and it must not be asked — or answer — twice.
    pub fn attach(&self, ack: Weak<dyn IntAck>) {
        let mut handlers = self.handlers.lock();
        if handlers.iter().any(|existing| Weak::ptr_eq(existing, &ack)) {
            return;
        }
        handlers.push(ack);
    }

    /// Whether nothing at all answers.
    ///
    /// A CPU uses this to tell "the board has vectoring controllers, and none
    /// of them claimed this cycle" from "this board has none, so the answer is
    /// whatever its address decode does".
    pub fn is_empty(&self) -> bool {
        self.handlers.lock().is_empty()
    }

    /// How many controllers are attached, dead ones included.
    pub fn len(&self) -> usize {
        self.handlers.lock().len()
    }

    /// Forget every controller, as re-plugging a wire does.
    pub fn clear(&self) {
        self.handlers.lock().clear();
    }

    /// Run `cycle` past each controller until one answers.
    ///
    /// [`IntAckResponse::Declined`] if none does, which is also the answer for
    /// an empty list. What the CPU makes of that is the CPU's business: a
    /// 68000 board with no controller on the net autovectors, because its
    /// address decode is what asserts `VPA`.
    pub fn run(&self, cycle: IntAckCycle) -> IntAckResponse {
        let mut next = 0;
        loop {
            // Cloned out under the lock and asked outside it, one at a time:
            // no allocation on the interrupt path, and no lock held across the
            // call into the controller.
            let handler = {
                let handlers = self.handlers.lock();
                match handlers.get(next) {
                    Some(handler) => handler.clone(),
                    None => return IntAckResponse::Declined,
                }
            };
            next += 1;
            if let Some(ack) = handler.upgrade() {
                let response = ack.acknowledge(cycle);
                if response.answered() {
                    return response;
                }
            }
        }
    }
}

/// A processor's own interrupt controller, from the processor's side.
///
/// [`IntAck`] covers the controller that hangs *off* a pin. This covers the one
/// that is part of the processor: an x86 local APIC, an ARM GIC CPU interface,
/// a RISC-V CLIC. Two things a core cannot model without such a link, and
/// neither of them is a level a wire could carry:
///
/// * **Where a processor is started.** An x86 application processor is held in
///   a wait-for-SIPI state and begins executing at a page a Start-Up *message*
///   names (*MultiProcessor Specification* v1.4 §B.4, Intel SDM Vol 3A §8.4.3).
///   `RESET` restarts it at the reset vector, which is a different thing, so
///   the startup page has to arrive by a route that can carry eight bits.
/// * **The register that enables the controller.** `IA32_APIC_BASE` is a
///   *processor* register — `RDMSR`/`WRMSR` reach it — naming state that lives
///   in the *controller* (SDM Vol 3A §10.4.3). Clearing its enable bit makes
///   the controller transparent, and only the controller can do that to itself.
///
/// Wired exactly as [`IntAck`] is, and for the same reasons: the controller
/// offers one on the pin it drives with
/// [`Device::local_controller`](crate::core::device::Device::local_controller),
/// the realizer hands it to every sink on that net with
/// [`Device::attach_local_controller`](crate::core::device::Device::attach_local_controller),
/// and the processor keeps a [`Weak`] reference, because the machine owns
/// devices and a wire merely refers to them (§4.3's weak edge).
pub trait LocalController: Send + Sync + fmt::Debug {
    /// What the controller has for its processor at an instruction boundary.
    ///
    /// **Consuming**: a [`Startup::page`] reported once is not reported again,
    /// which is what makes a Start-Up a one-shot event rather than a level.
    ///
    /// Called from the processor's execution path once per instruction with no
    /// lock held on its side, so an implementation is free to take its own.
    fn take_startup(&self) -> Startup;

    /// The controller's own base and enable register, as the processor reads
    /// it — `IA32_APIC_BASE` on an x86.
    ///
    /// Defaulted to zero for a controller with no such register. A processor
    /// should treat that as "there is no register here" rather than reading a
    /// plausible zero back to a guest.
    fn base_register(&self) -> u64 {
        0
    }

    /// Write it.
    ///
    /// The processor has already rejected the values *it* knows are invalid —
    /// reserved bits above its own physical address width, and any read-only
    /// field — so what arrives here is a value the controller is expected to
    /// take.
    fn set_base_register(&self, _value: u64) {}
}

/// What a [`LocalController`] hands its processor at an instruction boundary.
///
/// Three separate facts rather than one enumeration, because a single ask can
/// legitimately report all three: an INIT accepted, the line already dropped
/// again, and a Start-Up latched behind it. That is precisely the sequence the
/// *MultiProcessor Specification* v1.4 §B.4 prescribes, and a processor that
/// was not running while it happened sees the whole of it in one ask.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Hash)]
pub struct Startup {
    /// An INIT has been accepted since the last ask: the processor performs its
    /// INIT reset and enters the wait-for-SIPI state.
    pub init: bool,
    /// The INIT line is *still* asserted, so the processor stays in reset
    /// rather than executing (SDM Vol 3A §10.6.1's level-triggered pair).
    pub held: bool,
    /// The page a Start-Up named. The processor leaves wait-for-SIPI and begins
    /// executing at `CS:IP = page << 8 : 0`.
    pub page: Option<u8>,
}

impl Startup {
    /// Nothing to report.
    pub const NONE: Startup = Startup {
        init: false,
        held: false,
        page: None,
    };

    /// Whether this reports anything at all.
    #[must_use]
    pub const fn is_none(self) -> bool {
        !self.init && !self.held && self.page.is_none()
    }
}

/// The data half of a DMA request line.
///
/// `DRQ` is a wire and carries a level, but the transfer that level asks for
/// moves *bytes* between the controller and the peripheral, over `DACK` and
/// `IOR`/`IOW` — and a wire cannot carry a byte. So the peripheral offers this
/// on its `DRQ` pin with [`Device::dma_peripheral`](crate::core::Device::dma_peripheral), and the controller, which is
/// the sink on that net, is handed it by the realizer.
///
/// The same shape as [`IntAck`] and for the same reason: the interesting half
/// of the transaction travels in the opposite direction to the level, and a
/// device handle is not something the machine layer can hand out. The
/// controller keeps a [`Weak`], because the machine owns devices.
///
/// One byte at a time, deliberately. An 8237 in single-transfer mode really
/// does give the bus back between bytes, and how long a burst runs is the
/// controller's decision — an interface shaped "hand me your whole buffer"
/// would put the transfer mode in the peripheral, which is the one place it is
/// not.
pub trait DmaPeripheral: Send + Sync + fmt::Debug {
    /// Take one byte *from* the peripheral, for a device-to-memory transfer.
    ///
    /// `terminal` is true on the byte the controller's count expires on, which
    /// is the `TC`/`EOP` pulse the peripheral uses to end its own operation.
    fn dma_read(&self, terminal: bool) -> u8;

    /// Give one byte *to* the peripheral, for a memory-to-device transfer.
    fn dma_write(&self, byte: u8, terminal: bool);

    /// Whether the peripheral still wants service.
    ///
    /// A controller checks this between bytes of a burst: a peripheral whose
    /// FIFO has drained drops `DRQ` in hardware, and this is that drop seen
    /// from the controller's side without waiting for a wire to propagate.
    fn dma_ready(&self) -> bool {
        true
    }
}

/// Per-source level state: the bookkeeping that makes wired-OR possible.
///
/// A sink with several drivers keeps one of these, updates it from
/// [`WireSink::set_level`], and asks [`FanIn::resolve`] for the level of the
/// net. When the APU deasserts its IRQ while the cartridge still asserts,
/// `resolve(Resolve::Or)` still answers `High`.
///
/// Whether a [`Drive::code`] reads as [`Level::High`].
///
/// Open-coded rather than `Drive::from_code(c).level()` because this runs once
/// per source on every interrupt delivery in the machine, and two chained
/// `match`es on the hot path is two more than it needs.
#[inline]
const fn code_is_high(code: u8) -> bool {
    // `High` and `WeakHigh`; everything else — including `HiZ` — reads low.
    matches!(code, 1 | 4)
}

/// The source list is fixed at construction, so updates are wait-free and
/// allocation-free, and the whole thing is `Send + Sync` with no lock.
///
/// # What a slot holds
///
/// A [`Drive`], not a [`Level`]: the same bookkeeping serves a wired-OR sink
/// and the tri-state resolution a [`NetMode::Resolved`] net performs, and one
/// store is better than two that can disagree. The level-shaped API
/// ([`FanIn::set`], [`FanIn::level_of`], [`FanIn::resolve`]) is the one nearly
/// every sink uses and it is unchanged: it records *strong* drives, which is
/// what a sink on a per-sink net is being told about. [`Drive::HiZ`] reads back
/// as [`Level::Low`] through that API and as itself through
/// [`FanIn::drive_of`] — so a sink that has not been taught about tri-state
/// cannot be handed one by accident, because only a resolved net produces one
/// and a resolved net resolves it away first.
#[derive(Debug)]
pub struct FanIn {
    /// Sorted and deduplicated, so lookup is a binary search and iteration
    /// order is deterministic.
    sources: Box<[WireId]>,
    /// One [`Drive::code`] per source.
    drives: Box<[AtomicU8]>,
}

impl FanIn {
    /// Track these sources, all initially [`Level::Low`].
    ///
    /// Duplicates are collapsed; order does not matter.
    pub fn new(sources: &[WireId]) -> Self {
        Self::with_drive(sources, Drive::Low)
    }

    /// Track these sources, all initially at `drive`.
    ///
    /// [`Drive::HiZ`] is what a tri-state net wants: nothing is driving a net
    /// nobody has driven yet, which is a different statement from "everybody is
    /// driving it low" and resolves differently under a [`Pull`].
    pub fn with_drive(sources: &[WireId], drive: Drive) -> Self {
        let mut ids: Vec<WireId> = sources.to_vec();
        ids.sort_unstable();
        ids.dedup();
        let drives: Vec<AtomicU8> = ids.iter().map(|_| AtomicU8::new(drive.code())).collect();
        FanIn {
            sources: ids.into_boxed_slice(),
            drives: drives.into_boxed_slice(),
        }
    }

    /// The tracked sources, in ascending id order.
    #[inline]
    pub fn sources(&self) -> &[WireId] {
        &self.sources
    }

    /// Whether `src` is one of the tracked sources.
    #[inline]
    pub fn contains(&self, src: WireId) -> bool {
        self.index_of(src).is_some()
    }

    #[inline]
    fn index_of(&self, src: WireId) -> Option<usize> {
        self.sources.binary_search(&src).ok()
    }

    #[inline]
    fn drive_at(&self, i: usize) -> Drive {
        // A slot only ever holds a code this build wrote, so the fallback is
        // unreachable; `Low` keeps it total without an `unwrap` in the hot path.
        Drive::from_code(self.drives[i].load(SeqCst)).unwrap_or(Drive::Low)
    }

    #[inline]
    fn level_at(&self, i: usize) -> Level {
        Level::from_bool(code_is_high(self.drives[i].load(SeqCst)))
    }

    #[inline]
    fn set_at(&self, i: usize, level: Level) -> bool {
        self.drive_at_set(i, Drive::strong(level))
    }

    #[inline]
    fn drive_at_set(&self, i: usize, drive: Drive) -> bool {
        self.drives[i].swap(drive.code(), SeqCst) != drive.code()
    }

    /// Record `src`'s level.
    ///
    /// Returns whether that changed anything: `false` both for a repeat of the
    /// level already recorded and for a source this `FanIn` does not track. Use
    /// [`FanIn::contains`] when the two need telling apart — an untracked
    /// source means the machine graph was built wrong.
    #[inline]
    pub fn set(&self, src: WireId, level: Level) -> bool {
        match self.index_of(src) {
            Some(i) => self.set_at(i, level),
            None => false,
        }
    }

    /// Record `src`'s drive, tri-state and all.
    ///
    /// [`FanIn::set`] is this with [`Drive::strong`] applied; the return value
    /// means the same thing.
    #[inline]
    pub fn set_drive(&self, src: WireId, drive: Drive) -> bool {
        match self.index_of(src) {
            Some(i) => self.drive_at_set(i, drive),
            None => false,
        }
    }

    /// The level last recorded for `src`, or `None` if it is not tracked.
    ///
    /// A source that is not driving reads [`Level::Low`]; ask
    /// [`FanIn::drive_of`] to tell that from a source holding the net low.
    #[inline]
    pub fn level_of(&self, src: WireId) -> Option<Level> {
        self.index_of(src).map(|i| self.level_at(i))
    }

    /// The drive last recorded for `src`, or `None` if it is not tracked.
    #[inline]
    pub fn drive_of(&self, src: WireId) -> Option<Drive> {
        self.index_of(src).map(|i| self.drive_at(i))
    }

    /// Whether any source is currently high.
    #[inline]
    pub fn any_high(&self) -> bool {
        self.drives.iter().any(|d| code_is_high(d.load(SeqCst)))
    }

    /// Whether every source is currently high. Vacuously true with no sources.
    #[inline]
    pub fn all_high(&self) -> bool {
        self.drives.iter().all(|d| code_is_high(d.load(SeqCst)))
    }

    /// Resolve the tracked drives the way a net does, under `pull`.
    ///
    /// The rule, and the reason it does not depend on the order sources were
    /// registered in (CLAUDE.md, *Determinism*): count the drivers at each
    /// strength and polarity, then take the strongest polarity that has any
    /// driver at all, with the net's own resistor counted among the weak ones.
    ///
    /// Two drivers of equal strength pulling opposite ways is a short circuit,
    /// and hardware has no defined answer for it. The model answers
    /// [`Level::Low`] — an output stage sinking to ground is what usually wins a
    /// real fight, and a defined answer beats an arbitrary one — and reports
    /// the fault through the second half of the return value, which
    /// [`Wire::contention`] counts. Two *weak* drivers in opposition is a
    /// resistor divider rather than a fault, but it is equally not a level, so
    /// it is reported the same way.
    ///
    /// A net with no driver and no pull is floating; it reads [`Level::Low`]
    /// and is also reported. Nothing here is state: the answer is a function of
    /// the drives alone, so a net stays derived (`ROADMAP.md` §4.5).
    pub fn resolve_drives(&self, pull: Pull) -> (Level, bool) {
        let (mut strong_low, mut strong_high) = (false, false);
        let (mut weak_low, mut weak_high) = (false, false);
        for i in 0..self.drives.len() {
            match self.drive_at(i) {
                Drive::HiZ => {}
                Drive::Low => strong_low = true,
                Drive::High => strong_high = true,
                Drive::WeakLow => weak_low = true,
                Drive::WeakHigh => weak_high = true,
            }
        }
        match pull.drive() {
            Drive::WeakLow => weak_low = true,
            Drive::WeakHigh => weak_high = true,
            _ => {}
        }
        if strong_low || strong_high {
            let fought = strong_low && strong_high;
            (Level::from_bool(strong_high && !strong_low), fought)
        } else if weak_low || weak_high {
            let fought = weak_low && weak_high;
            (Level::from_bool(weak_high && !weak_low), fought)
        } else {
            (Level::Low, true)
        }
    }

    /// The level of the net under the given resolution.
    #[inline]
    pub fn resolve(&self, mode: Resolve) -> Level {
        if self.sources.is_empty() {
            return mode.idle();
        }
        match mode {
            Resolve::Or => Level::from_bool(self.any_high()),
            Resolve::And => Level::from_bool(self.all_high()),
        }
    }

    /// Drive every source low, as a reset does.
    ///
    /// Records only: nothing is propagated, because a reset propagates through
    /// the devices' own reset paths.
    pub fn clear(&self) {
        self.clear_to(Drive::Low);
    }

    /// The same, to an arbitrary drive — [`Drive::HiZ`] for a tri-state net,
    /// whose reset state is "nobody is driving".
    pub fn clear_to(&self, drive: Drive) {
        for d in self.drives.iter() {
            d.store(drive.code(), SeqCst);
        }
    }

    /// The architectural state: every source and its level, in id order.
    pub fn snapshot(&self) -> Vec<(WireId, Level)> {
        self.sources
            .iter()
            .enumerate()
            .map(|(i, id)| (*id, self.level_at(i)))
            .collect()
    }

    /// The same, keeping the tri-state: every source and its drive, in id
    /// order. What a [`Wire`] writes to a snapshot.
    pub fn snapshot_drives(&self) -> Vec<(WireId, Drive)> {
        self.sources
            .iter()
            .enumerate()
            .map(|(i, id)| (*id, self.drive_at(i)))
            .collect()
    }

    /// Restore state produced by [`FanIn::snapshot`].
    ///
    /// Entries for sources this `FanIn` does not track are ignored: a snapshot
    /// taken from a differently shaped machine is diagnosed by the snapshot
    /// layer (`ROADMAP.md` §4.5), not here. Nothing is propagated — on load,
    /// every device restores its own state.
    pub fn restore(&self, state: &[(WireId, Level)]) {
        for (id, level) in state {
            if let Some(i) = self.index_of(*id) {
                self.drives[i].store(Drive::strong(*level).code(), SeqCst);
            }
        }
    }

    /// Restore state produced by [`FanIn::snapshot_drives`].
    pub fn restore_drives(&self, state: &[(WireId, Drive)]) {
        for (id, drive) in state {
            if let Some(i) = self.index_of(*id) {
                self.drives[i].store(drive.code(), SeqCst);
            }
        }
    }
}

/// How a wire holds a connected sink.
enum SinkRef {
    Strong(Arc<dyn WireSink>),
    /// Non-owning, for breaking the `Arc` cycle a device that both drives and
    /// listens would otherwise create.
    Weak(Weak<dyn WireSink>),
}

impl SinkRef {
    /// Run `f` against the sink, or nothing if a weak sink has been dropped.
    #[inline]
    fn with(&self, f: impl FnOnce(&dyn WireSink)) {
        match self {
            SinkRef::Strong(s) => f(&**s),
            SinkRef::Weak(w) => {
                if let Some(s) = w.upgrade() {
                    f(&*s);
                }
            }
        }
    }
}

/// One connected sink and the input pin it was connected to.
struct SinkPort {
    sink: SinkRef,
    line: u32,
}

/// A net: per-source level state plus fan-out to every connected sink.
///
/// Sources drive it with [`Wire::set`], or through a [`WireSource`] handle, and
/// every change is delivered to every sink as `set_level(src, line, level)` —
/// the driver's level, unresolved, so the sink can do its own fan-in. The wire
/// keeps the per-source levels so a repeated `set` costs nothing, so
/// [`Wire::refresh`] can re-announce state after a snapshot load, and so the
/// levels themselves can be snapshotted.
///
/// Built through [`Wire::builder`]; sources and sinks are fixed thereafter.
pub struct Wire {
    inputs: FanIn,
    mode: NetMode,
    /// One flag per source, set when its level moved and has not been
    /// delivered. This is what makes a re-entrant `set` cost an iteration
    /// rather than a stack frame.
    pending: Box<[AtomicBool]>,
    sinks: Box<[SinkPort]>,
    /// Held by whichever call is currently delivering. Not a lock over data: it
    /// protects no state, so nothing is held across the outward call.
    delivering: AtomicBool,
    unsettled: AtomicUsize,
    /// On a [`NetMode::Resolved`] net, the level last delivered, so an
    /// unchanged resolution costs nothing. Derived — a cache of
    /// [`Wire::resolve_net`] — so it is neither snapshotted nor restored; the
    /// sentinel below makes the first delivery unconditional.
    last_resolved: AtomicU8,
    contention: AtomicUsize,
}

/// A resolved wire's cached level before anything has been delivered.
const RESOLVED_UNKNOWN: u8 = 2;

impl Wire {
    /// How many delivery passes one outermost [`Wire::set`] runs before
    /// declaring the graph unsettled.
    ///
    /// Reached only by a combinational loop, which is a machine-description
    /// error; a legitimate graph settles in one pass, or in as many passes as
    /// its feedback path needs to reach a stable state.
    pub const SETTLE_LIMIT: u32 = 64;

    /// Start building a wire.
    pub fn builder() -> WireBuilder {
        WireBuilder::new()
    }

    /// Drive `src`'s level, delivering the change to every sink.
    ///
    /// Returns whether the level moved. `false` means either that `src` already
    /// drove this level — nothing is delivered, so a device may call this every
    /// cycle — or that `src` is not a source of this wire, for which see
    /// [`FanIn::set`].
    ///
    /// Delivery has finished when this returns, *except* when the call is
    /// re-entrant (a sink driving the wire that is notifying it) or when
    /// another thread is mid-delivery; then the change is handed to that
    /// delivery, which picks it up before it finishes. Level semantics make
    /// that indistinguishable from outside: the wire converges on the levels
    /// last written.
    pub fn set(&self, src: WireId, level: Level) -> bool {
        self.drive(src, Drive::strong(level))
    }

    /// Drive `src`'s output stage, delivering the change to every sink.
    ///
    /// [`Wire::set`] is this with [`Drive::strong`] applied, which is what an
    /// ordinary push-pull output does and what every driver in the tree did
    /// before tri-state existed. A [`Drive::HiZ`] on a [`NetMode::PerSink`] net
    /// is recorded but resolves as low for the sinks, since nothing on such a
    /// net can represent "not driving"; put the net in
    /// [`NetMode::Resolved`] to make it mean what it says.
    ///
    /// Returns whether the drive moved, exactly as [`Wire::set`] does.
    pub fn drive(&self, src: WireId, drive: Drive) -> bool {
        let Some(i) = self.inputs.index_of(src) else {
            return false;
        };
        if !self.inputs.drive_at_set(i, drive) {
            return false;
        }
        self.pending[i].store(true, SeqCst);
        self.deliver();
        true
    }

    /// Re-announce every source's current level to every sink.
    ///
    /// Wanted after a snapshot load, and after a reset that changed levels
    /// without propagating. Delivery is idempotent, so this is always safe.
    pub fn refresh(&self) {
        // A resolved net delivers only when its level *moves*, so forget what
        // was last delivered or a refresh after a snapshot load would decide
        // there was nothing to say.
        self.last_resolved.store(RESOLVED_UNKNOWN, SeqCst);
        for p in self.pending.iter() {
            p.store(true, SeqCst);
        }
        self.deliver();
    }

    /// Deliver pending changes until the wire is quiescent.
    ///
    /// The `delivering` flag is what keeps re-entrancy iterative: a nested or
    /// concurrent `set` has already recorded its level and marked it pending,
    /// so it can return and let this loop pick the change up.
    fn deliver(&self) {
        if self.delivering.swap(true, SeqCst) {
            // Someone else — possibly this very call stack — is delivering and
            // will observe what we just marked pending.
            return;
        }
        loop {
            let mut passes: u32 = 0;
            loop {
                let moved = match self.mode {
                    NetMode::PerSink => self.deliver_per_sink(),
                    NetMode::Resolved(pull) => self.deliver_resolved(pull),
                };
                if !moved {
                    break;
                }
                passes += 1;
                if passes >= Self::SETTLE_LIMIT {
                    // A combinational loop. Stop rather than spin: the recorded
                    // levels stay correct, the sinks may be stale, and
                    // `unsettled()` says so. The next external change starts a
                    // fresh, equally bounded attempt.
                    self.unsettled.fetch_add(1, SeqCst);
                    for p in self.pending.iter() {
                        p.store(false, SeqCst);
                    }
                    self.delivering.store(false, SeqCst);
                    return;
                }
            }
            self.delivering.store(false, SeqCst);
            // A change made between the last scan and the release above would
            // have seen the flag set and returned, so re-check before leaving.
            if !self.pending.iter().any(|p| p.load(SeqCst)) {
                return;
            }
            if self.delivering.swap(true, SeqCst) {
                return;
            }
        }
    }

    /// One pass of [`NetMode::PerSink`] delivery: each moved source's own
    /// level, to every sink, for the sinks to resolve. Returns whether
    /// anything was delivered.
    fn deliver_per_sink(&self) -> bool {
        let mut moved = false;
        for (i, src) in self.inputs.sources().iter().enumerate() {
            // Take the flag first, then read the level: a racing writer that
            // beats us to the level sets the flag again, so the worst case is
            // one redundant, idempotent delivery.
            if self.pending[i].swap(false, SeqCst) {
                moved = true;
                let level = self.inputs.level_at(i);
                for port in self.sinks.iter() {
                    port.sink.with(|s| s.set_level(*src, port.line, level));
                }
            }
        }
        moved
    }

    /// One pass of [`NetMode::Resolved`] delivery.
    ///
    /// The net resolves once, and the answer goes to every sink **once per
    /// source**. Repeating it per source is not waste: it is what keeps the
    /// forty sinks in the tree that keep a [`FanIn`] correct without being
    /// rewritten, since a fan-in whose every entry holds the resolved level
    /// returns that level under [`Resolve::Or`] and under [`Resolve::And`]
    /// alike. A sink that ignores `src` and takes the level sees the same
    /// thing repeated, which is idempotent.
    fn deliver_resolved(&self, pull: Pull) -> bool {
        let mut any_pending = false;
        for p in self.pending.iter() {
            if p.swap(false, SeqCst) {
                any_pending = true;
            }
        }
        if !any_pending {
            return false;
        }
        let (level, fought) = self.inputs.resolve_drives(pull);
        if fought {
            self.contention.fetch_add(1, SeqCst);
        }
        // A driver that moved without moving the net — one open-drain stage
        // letting go while another still holds the net low — is not news.
        if self.last_resolved.swap(u8::from(level.is_high()), SeqCst) == u8::from(level.is_high()) {
            return false;
        }
        for src in self.inputs.sources() {
            for port in self.sinks.iter() {
                port.sink.with(|s| s.set_level(*src, port.line, level));
            }
        }
        true
    }

    /// The per-source levels, for a sink or a snapshot to inspect.
    #[inline]
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The sources that may drive this wire, in ascending id order.
    #[inline]
    pub fn sources(&self) -> &[WireId] {
        self.inputs.sources()
    }

    /// The level `src` is driving, or `None` if it is not a source here.
    ///
    /// A source that is not driving reads [`Level::Low`]; [`Wire::drive_of`]
    /// tells that from a source holding the net low.
    #[inline]
    pub fn level_of(&self, src: WireId) -> Option<Level> {
        self.inputs.level_of(src)
    }

    /// The output stage `src` presents, or `None` if it is not a source here.
    #[inline]
    pub fn drive_of(&self, src: WireId) -> Option<Drive> {
        self.inputs.drive_of(src)
    }

    /// Who resolves this net, and the resistor it carries.
    #[inline]
    pub fn mode(&self) -> NetMode {
        self.mode
    }

    /// The level of the net under the given resolution.
    #[inline]
    pub fn resolve(&self, mode: Resolve) -> Level {
        self.inputs.resolve(mode)
    }

    /// The level of the net under **its own** resolution.
    ///
    /// For a [`NetMode::Resolved`] net this is the level its sinks are being
    /// told, computed from the drives and the net's [`Pull`] by
    /// [`FanIn::resolve_drives`] — which is where the rule, including what a
    /// short circuit and a floating net answer, is written down. For a
    /// [`NetMode::PerSink`] net there is no such thing, so this reports the
    /// wired-OR the default [`Resolve`] implies and every sink is still free to
    /// disagree.
    pub fn resolve_net(&self) -> Level {
        match self.mode {
            NetMode::PerSink => self.inputs.resolve(Resolve::Or),
            NetMode::Resolved(pull) => self.inputs.resolve_drives(pull).0,
        }
    }

    /// How many times this net has resolved a **contention**: two drivers of
    /// equal strength in opposition, or nothing driving it at all.
    ///
    /// A diagnostic, like [`Wire::unsettled`], and not snapshotted. A non-zero
    /// count on a [`NetMode::Resolved`] net means either a short in the machine
    /// description or a net that wants a [`Pull`] and has not been given one.
    /// It stays zero on a per-sink net, which cannot express either fault.
    pub fn contention(&self) -> usize {
        self.contention.load(SeqCst)
    }

    /// How many sinks are connected.
    #[inline]
    pub fn sink_count(&self) -> usize {
        self.sinks.len()
    }

    /// How many deliveries have given up at [`Wire::SETTLE_LIMIT`].
    ///
    /// A diagnostic, not architectural state: it is not snapshotted, and a
    /// non-zero value means the machine description contains a combinational
    /// loop through this wire.
    pub fn unsettled(&self) -> usize {
        self.unsettled.load(SeqCst)
    }

    /// The architectural state: every source and its level.
    pub fn snapshot(&self) -> Vec<(WireId, Level)> {
        self.inputs.snapshot()
    }

    /// The architectural state, keeping the tri-state: every source and its
    /// drive.
    ///
    /// What a machine writes. The *net's* level is derived from these and is
    /// never stored — the wire keeps a cache of it and
    /// [`Wire::refresh`] discards it — so `ROADMAP.md` §4.5's rule that a net
    /// is derived state survives tri-state unchanged. What is saved here is
    /// each driver's output stage, which is the driver's own state mirrored
    /// where the sinks can be handed it again on load.
    pub fn snapshot_drives(&self) -> Vec<(WireId, Drive)> {
        self.inputs.snapshot_drives()
    }

    /// Restore state from [`Wire::snapshot`] without delivering anything.
    ///
    /// Call [`Wire::refresh`] afterwards if the sinks do not restore their own
    /// input state.
    pub fn restore(&self, state: &[(WireId, Level)]) {
        self.inputs.restore(state);
    }

    /// Restore state from [`Wire::snapshot_drives`], delivering nothing.
    pub fn restore_drives(&self, state: &[(WireId, Drive)]) {
        self.inputs.restore_drives(state);
    }
}

impl fmt::Debug for Wire {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // Manual: a sink is a trait object and is not `Debug`.
        f.debug_struct("Wire")
            .field("inputs", &self.inputs)
            .field("mode", &self.mode)
            .field("sinks", &self.sinks.len())
            .field("unsettled", &self.unsettled.load(SeqCst))
            .field("contention", &self.contention.load(SeqCst))
            .finish()
    }
}

/// Builder for a [`Wire`].
///
/// Every driver of the net is declared before it runs, which is what the
/// machine resolver knows and what lets the wire's state be lock-free.
#[derive(Default)]
pub struct WireBuilder {
    sources: Vec<WireId>,
    sinks: Vec<SinkPort>,
    mode: NetMode,
}

impl WireBuilder {
    /// An empty builder: no sources, no sinks.
    pub fn new() -> Self {
        WireBuilder::default()
    }

    /// Declare a driver of this net.
    #[must_use]
    pub fn source(mut self, src: WireId) -> Self {
        self.sources.push(src);
        self
    }

    /// Declare several drivers.
    #[must_use]
    pub fn sources(mut self, srcs: &[WireId]) -> Self {
        self.sources.extend_from_slice(srcs);
        self
    }

    /// Connect a sink, delivering to its input pin `line`.
    ///
    /// Sinks are notified in connection order, which keeps a machine's
    /// behaviour reproducible.
    #[must_use]
    pub fn sink(mut self, sink: Arc<dyn WireSink>, line: u32) -> Self {
        self.sinks.push(SinkPort {
            sink: SinkRef::Strong(sink),
            line,
        });
        self
    }

    /// Connect a sink without owning it.
    ///
    /// For the case the module docs describe: a device that drives a wire whose
    /// own sink chain leads back to it would otherwise form an `Arc` cycle and
    /// leak. The machine holds the strong reference; a sink dropped before the
    /// wire is simply skipped.
    #[must_use]
    pub fn sink_weak(mut self, sink: Weak<dyn WireSink>, line: u32) -> Self {
        self.sinks.push(SinkPort {
            sink: SinkRef::Weak(sink),
            line,
        });
        self
    }

    /// Who resolves this net.
    ///
    /// Defaults to [`NetMode::PerSink`], which is what every net in the tree
    /// was before tri-state existed and what an interrupt line still wants.
    #[must_use]
    pub fn mode(mut self, mode: NetMode) -> Self {
        self.mode = mode;
        self
    }

    /// Shorthand for [`WireBuilder::mode`] with [`NetMode::Resolved`].
    #[must_use]
    pub fn resolved(self, pull: Pull) -> Self {
        self.mode(NetMode::Resolved(pull))
    }

    /// Finish. Nothing is delivered; devices announce their reset levels
    /// themselves.
    ///
    /// A per-sink net starts with every source at [`Level::Low`], as it always
    /// has. A resolved net starts with every source at [`Drive::HiZ`], because
    /// a part that has not been told to drive anything is not driving — which
    /// is the whole distinction the mode exists for, and getting it wrong here
    /// would make a pulled-up net read low until its first write.
    pub fn build(self) -> Wire {
        let idle = if self.mode.is_resolved() {
            Drive::HiZ
        } else {
            Drive::Low
        };
        let inputs = FanIn::with_drive(&self.sources, idle);
        let pending: Vec<AtomicBool> = inputs
            .sources()
            .iter()
            .map(|_| AtomicBool::new(false))
            .collect();
        Wire {
            inputs,
            mode: self.mode,
            pending: pending.into_boxed_slice(),
            sinks: self.sinks.into_boxed_slice(),
            delivering: AtomicBool::new(false),
            unsettled: AtomicUsize::new(0),
            last_resolved: AtomicU8::new(RESOLVED_UNKNOWN),
            contention: AtomicUsize::new(0),
        }
    }

    /// Finish, wrapped for sharing between the devices at both ends.
    pub fn build_shared(self) -> Arc<Wire> {
        Arc::new(self.build())
    }
}

impl fmt::Debug for WireBuilder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("WireBuilder")
            .field("sources", &self.sources)
            .field("sinks", &self.sinks.len())
            .field("mode", &self.mode)
            .finish()
    }
}

/// An output port: a wire plus the id this device drives it with.
///
/// What a device holds in order to assert a line. Bundling the two means a
/// device never has to remember to pass its own id, which is the sort of
/// mistake that produces a line nobody can deassert.
#[derive(Debug, Clone)]
pub struct WireSource {
    wire: Arc<Wire>,
    id: WireId,
}

impl WireSource {
    /// Bind `id`, which must be one of the wire's declared sources, to `wire`.
    pub fn new(wire: Arc<Wire>, id: WireId) -> Self {
        WireSource { wire, id }
    }

    /// The id this port drives with.
    #[inline]
    pub fn id(&self) -> WireId {
        self.id
    }

    /// The wire this port drives.
    #[inline]
    pub fn wire(&self) -> &Arc<Wire> {
        &self.wire
    }

    /// Drive `level`. Returns whether it changed, as [`Wire::set`] does.
    #[inline]
    pub fn set(&self, level: Level) -> bool {
        self.wire.set(self.id, level)
    }

    /// Present `drive` — an output stage rather than a level, so this is the
    /// one an open-drain pin, a pull resistor or a switch contact uses.
    /// Returns whether it changed, as [`Wire::drive`] does.
    #[inline]
    pub fn drive(&self, drive: Drive) -> bool {
        self.wire.drive(self.id, drive)
    }

    /// Stop driving: [`WireSource::drive`] with [`Drive::HiZ`].
    #[inline]
    pub fn release(&self) -> bool {
        self.drive(Drive::HiZ)
    }

    /// The output stage this port is currently presenting.
    #[inline]
    pub fn drive_state(&self) -> Drive {
        self.wire.drive_of(self.id).unwrap_or(Drive::HiZ)
    }

    /// The level of the whole net, as its own resolution computes it.
    ///
    /// What a bidirectional pin reads back: a part that drives a pin and then
    /// samples it is reading the net, not its own output stage, and on an
    /// open-drain net the two differ whenever somebody else is pulling.
    #[inline]
    pub fn net_level(&self) -> Level {
        self.wire.resolve_net()
    }

    /// Drive high.
    #[inline]
    pub fn raise(&self) -> bool {
        self.set(Level::High)
    }

    /// Drive low.
    #[inline]
    pub fn lower(&self) -> bool {
        self.set(Level::Low)
    }

    /// The level this port is currently driving, and [`Level::Low`] when it is
    /// driving nothing at all.
    ///
    /// [`WireSource::drive_state`] is the one that tells those apart, and
    /// [`WireSource::net_level`] is what the *pin* is at.
    #[inline]
    pub fn level(&self) -> Level {
        self.wire.level_of(self.id).unwrap_or(Level::Low)
    }

    /// Drive `active`, then immediately return to the opposite level.
    ///
    /// The transient an edge-triggered input latches. Both transitions are
    /// delivered to every sink, so a sink that only tracks levels sees the line
    /// end where it started — which is the point: a pulse means something only
    /// to something that latches it, such as [`EdgeLatch`]. A pulse driven onto
    /// a wire inside a propagation cycle can be coalesced away, so do not put
    /// an edge output in one.
    pub fn pulse(&self, active: Level) {
        self.set(active);
        self.set(active.inverted());
    }
}

/// `wire.split`: one input, many outputs.
///
/// A [`Wire`] already fans out to many sinks, so a split is not needed to
/// deliver to several places; it exists because a machine description often
/// wants the fan-out to be a named node — to renumber lines, to insert a
/// combinator on one branch only, or simply so the graph reads clearly.
///
/// Several drivers on the input are resolved (wired-OR by default) before being
/// forwarded, so a split never re-exports the bug it is meant to be neutral
/// about.
#[derive(Debug)]
pub struct WireSplit {
    inputs: FanIn,
    mode: Resolve,
    outs: Box<[WireSource]>,
}

impl WireSplit {
    /// The class name this device is registered under.
    pub const CLASS: &'static str = "wire.split";

    /// A split with wired-OR input resolution.
    pub fn new(sources: &[WireId], outs: Vec<WireSource>) -> Self {
        Self::with_resolve(sources, Resolve::Or, outs)
    }

    /// A split with an explicit input resolution.
    pub fn with_resolve(sources: &[WireId], mode: Resolve, outs: Vec<WireSource>) -> Self {
        WireSplit {
            inputs: FanIn::new(sources),
            mode,
            outs: outs.into_boxed_slice(),
        }
    }

    /// The per-source input state, for snapshotting.
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The current output level.
    pub fn level(&self) -> Level {
        self.inputs.resolve(self.mode)
    }

    /// Drive the outputs with the level the current inputs imply.
    ///
    /// A combinational device's output is a function of its inputs, but a wire
    /// only carries what has been driven onto it, and a freshly built or
    /// freshly loaded machine has driven nothing. Reset and snapshot-load call
    /// this — in topological order, so a chain of gates converges in one sweep
    /// — the same job [`Wire::refresh`] does for a net.
    pub fn announce(&self) {
        let out = self.level();
        for o in self.outs.iter() {
            o.set(out);
        }
    }
}

impl WireSink for WireSplit {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        if self.inputs.set(src, level) {
            let out = self.inputs.resolve(self.mode);
            for o in self.outs.iter() {
                o.set(out);
            }
        }
    }
}

/// `wire.or`: the explicit wired-OR combiner.
///
/// Its output is high while any input source is high. This is the device the
/// DSL's implicit fan-in expands into (`ROADMAP.md` §4.3, §5) — the same
/// resolution a [`FanIn`]-keeping sink performs, packaged so a machine file can
/// name it.
#[derive(Debug)]
pub struct WireOr {
    inputs: FanIn,
    out: WireSource,
}

impl WireOr {
    /// The class name this device is registered under.
    pub const CLASS: &'static str = "wire.or";

    /// Combine `sources` onto `out`.
    pub fn new(sources: &[WireId], out: WireSource) -> Self {
        WireOr {
            inputs: FanIn::new(sources),
            out,
        }
    }

    /// The per-source input state, for snapshotting.
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The current output level.
    pub fn level(&self) -> Level {
        self.inputs.resolve(Resolve::Or)
    }

    /// Drive the output with the level the current inputs imply; see
    /// [`WireSplit::announce`].
    pub fn announce(&self) {
        self.out.set(self.level());
    }
}

impl WireSink for WireOr {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        if self.inputs.set(src, level) {
            self.out.set(self.inputs.resolve(Resolve::Or));
        }
    }
}

/// `wire.and`: the wired-AND combiner.
///
/// Output high only while every input is high — an open-drain bus with a
/// pull-up, and how an "everyone is ready" line is built. With no inputs it
/// reads high, matching the pull-up.
#[derive(Debug)]
pub struct WireAnd {
    inputs: FanIn,
    out: WireSource,
}

impl WireAnd {
    /// The class name this device is registered under.
    pub const CLASS: &'static str = "wire.and";

    /// Combine `sources` onto `out`.
    pub fn new(sources: &[WireId], out: WireSource) -> Self {
        WireAnd {
            inputs: FanIn::new(sources),
            out,
        }
    }

    /// The per-source input state, for snapshotting.
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The current output level.
    pub fn level(&self) -> Level {
        self.inputs.resolve(Resolve::And)
    }

    /// Drive the output with the level the current inputs imply; see
    /// [`WireSplit::announce`].
    pub fn announce(&self) {
        self.out.set(self.level());
    }
}

impl WireSink for WireAnd {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        if self.inputs.set(src, level) {
            self.out.set(self.inputs.resolve(Resolve::And));
        }
    }
}

/// `wire.not`: inversion, for converting between active-high and active-low.
///
/// Inputs are resolved first (wired-OR by default, which makes a multi-input
/// `not` a NOR gate), then inverted.
#[derive(Debug)]
pub struct WireNot {
    inputs: FanIn,
    mode: Resolve,
    out: WireSource,
}

impl WireNot {
    /// The class name this device is registered under.
    pub const CLASS: &'static str = "wire.not";

    /// An inverter with wired-OR input resolution.
    pub fn new(sources: &[WireId], out: WireSource) -> Self {
        Self::with_resolve(sources, Resolve::Or, out)
    }

    /// An inverter with an explicit input resolution.
    pub fn with_resolve(sources: &[WireId], mode: Resolve, out: WireSource) -> Self {
        WireNot {
            inputs: FanIn::new(sources),
            mode,
            out,
        }
    }

    /// The per-source input state, for snapshotting.
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The current output level.
    pub fn level(&self) -> Level {
        self.inputs.resolve(self.mode).inverted()
    }

    /// Drive the output with the level the current inputs imply; see
    /// [`WireSplit::announce`]. An inverter especially needs this: its idle
    /// output is high, which is *not* where an undriven wire sits.
    pub fn announce(&self) {
        self.out.set(self.level());
    }
}

impl WireSink for WireNot {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        if self.inputs.set(src, level) {
            self.out.set(self.inputs.resolve(self.mode).inverted());
        }
    }
}

/// The snapshottable state of a [`LevelToEdge`].
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EdgeState {
    /// Each input source and the level it is driving.
    pub inputs: Vec<(WireId, Level)>,
    /// The resolved input level the detector last acted on. This is the field
    /// that makes edge detection a device rather than a flag: restore it and
    /// the detector resumes without inventing an edge that never happened.
    pub last: Level,
}

/// `wire.level-to-edge`: an edge detector with state.
///
/// Holds the previous resolved input level and emits a pulse on its output
/// ([`WireSource::pulse`]) for every transition its [`EdgeTrigger`] selects.
/// Because that previous level is a field rather than a flag hidden in a
/// consumer, it round-trips through a snapshot: a detector restored
/// mid-assertion does not manufacture a spurious interrupt, which is exactly
/// what "just keep a bool somewhere" produces.
#[derive(Debug)]
pub struct LevelToEdge {
    inputs: FanIn,
    mode: Resolve,
    trigger: EdgeTrigger,
    /// The last resolved input level. Architectural state.
    last: AtomicBool,
    /// Pulse polarity: the level the output is driven to on an edge.
    active: Level,
    out: WireSource,
    /// Diagnostic only, hence absent from [`EdgeState`].
    edges: AtomicUsize,
}

impl LevelToEdge {
    /// The class name this device is registered under.
    pub const CLASS: &'static str = "wire.level-to-edge";

    /// A detector with wired-OR inputs, emitting a high-going pulse.
    pub fn new(sources: &[WireId], trigger: EdgeTrigger, out: WireSource) -> Self {
        Self::with_options(sources, Resolve::Or, trigger, Level::High, out)
    }

    /// A detector with every knob spelled out: input resolution, which edges
    /// fire, and the polarity of the output pulse.
    pub fn with_options(
        sources: &[WireId],
        mode: Resolve,
        trigger: EdgeTrigger,
        active: Level,
        out: WireSource,
    ) -> Self {
        let inputs = FanIn::new(sources);
        let last = inputs.resolve(mode);
        LevelToEdge {
            inputs,
            mode,
            trigger,
            last: AtomicBool::new(last.as_bool()),
            active,
            out,
            edges: AtomicUsize::new(0),
        }
    }

    /// The per-source input state.
    pub fn inputs(&self) -> &FanIn {
        &self.inputs
    }

    /// The resolved input level the detector last acted on.
    pub fn last_level(&self) -> Level {
        Level::from_bool(self.last.load(SeqCst))
    }

    /// How many pulses have been emitted. A diagnostic, not architectural
    /// state, so it is deliberately absent from [`EdgeState`].
    pub fn edge_count(&self) -> usize {
        self.edges.load(SeqCst)
    }

    /// The state to write to a snapshot.
    pub fn snapshot(&self) -> EdgeState {
        EdgeState {
            inputs: self.inputs.snapshot(),
            last: self.last_level(),
        }
    }

    /// Restore snapshotted state. Emits nothing.
    pub fn restore(&self, state: &EdgeState) {
        self.inputs.restore(&state.inputs);
        self.last.store(state.last.as_bool(), SeqCst);
    }
}

impl WireSink for LevelToEdge {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        if !self.inputs.set(src, level) {
            return;
        }
        let now = self.inputs.resolve(self.mode);
        let before = Level::from_bool(self.last.swap(now.as_bool(), SeqCst));
        let Some(edge) = Edge::between(before, now) else {
            return;
        };
        if self.trigger.matches(edge) {
            self.edges.fetch_add(1, SeqCst);
            self.out.pulse(self.active);
        }
    }
}

/// The consumer half of edge semantics: latches a pulse until it is taken.
///
/// An edge-triggered input means something only if something remembers the
/// transient, and hardware does exactly this — a flip-flop the handler clears.
/// Interrupt controllers will embed the same behaviour; this is it on its own,
/// snapshottable, and useful as the far end of a [`LevelToEdge`].
#[derive(Debug)]
pub struct EdgeLatch {
    /// The level that counts as a pulse.
    active: Level,
    pending: AtomicBool,
    /// Diagnostic only.
    seen: AtomicUsize,
}

impl EdgeLatch {
    /// A latch that triggers on `active`.
    pub fn new(active: Level) -> Self {
        EdgeLatch {
            active,
            pending: AtomicBool::new(false),
            seen: AtomicUsize::new(0),
        }
    }

    /// Whether a pulse is latched.
    pub fn peek(&self) -> bool {
        self.pending.load(SeqCst)
    }

    /// Take the latched pulse, clearing it.
    pub fn take(&self) -> bool {
        self.pending.swap(false, SeqCst)
    }

    /// Clear without reporting, as a reset does.
    pub fn clear(&self) {
        self.pending.store(false, SeqCst);
    }

    /// How many pulses have been latched. Diagnostic, not architectural state.
    pub fn count(&self) -> usize {
        self.seen.load(SeqCst)
    }

    /// The latched flag, for a snapshot.
    pub fn snapshot(&self) -> bool {
        self.peek()
    }

    /// Restore the latched flag.
    pub fn restore(&self, pending: bool) {
        self.pending.store(pending, SeqCst);
    }
}

impl WireSink for EdgeLatch {
    fn set_level(&self, _src: WireId, _line: u32, level: Level) {
        if level == self.active {
            self.pending.store(true, SeqCst);
            self.seen.fetch_add(1, SeqCst);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    const A: WireId = WireId::new(1);
    const B: WireId = WireId::new(2);
    const C: WireId = WireId::new(3);
    const GATE: WireId = WireId::new(100);
    const GATE2: WireId = WireId::new(101);

    fn stage_id(n: u64) -> WireId {
        WireId::new(200 + n)
    }

    /// A sink that does what §4.3 says a sink must do: track which sources are
    /// asserting and resolve them itself.
    #[derive(Debug)]
    struct Irq {
        inputs: FanIn,
        asserted: AtomicBool,
        changes: AtomicUsize,
        calls: AtomicUsize,
        last_line: AtomicUsize,
    }

    impl Irq {
        fn new(sources: &[WireId]) -> Arc<Self> {
            Arc::new(Irq {
                inputs: FanIn::new(sources),
                asserted: AtomicBool::new(false),
                changes: AtomicUsize::new(0),
                calls: AtomicUsize::new(0),
                last_line: AtomicUsize::new(0),
            })
        }
        fn level(&self) -> Level {
            Level::from_bool(self.asserted.load(SeqCst))
        }
        fn changes(&self) -> usize {
            self.changes.load(SeqCst)
        }
    }

    impl WireSink for Irq {
        fn set_level(&self, src: WireId, line: u32, level: Level) {
            self.calls.fetch_add(1, SeqCst);
            self.last_line.store(line as usize, SeqCst);
            if self.inputs.set(src, level) {
                let now = self.inputs.resolve(Resolve::Or).as_bool();
                if self.asserted.swap(now, SeqCst) != now {
                    self.changes.fetch_add(1, SeqCst);
                }
            }
        }
    }

    /// A sink that only remembers the last level it was told, ignoring `src`.
    /// Fine on a single-source net, which is all it is used for here.
    #[derive(Debug)]
    struct Probe {
        level: AtomicBool,
        calls: AtomicUsize,
    }

    impl Probe {
        fn new() -> Arc<Self> {
            Arc::new(Probe {
                level: AtomicBool::new(false),
                calls: AtomicUsize::new(0),
            })
        }
        fn level(&self) -> Level {
            Level::from_bool(self.level.load(SeqCst))
        }
        fn calls(&self) -> usize {
            self.calls.load(SeqCst)
        }
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level.store(level.as_bool(), SeqCst);
            self.calls.fetch_add(1, SeqCst);
        }
    }

    /// A gate that reaches its target through a `Weak`, so a feedback loop can
    /// be built at all: the strong-reference wiring in this module is
    /// deliberately acyclic.
    #[derive(Debug)]
    struct WeakGate {
        inputs: FanIn,
        target: Weak<Wire>,
        id: WireId,
        invert: bool,
    }

    impl WireSink for WeakGate {
        fn set_level(&self, src: WireId, _line: u32, level: Level) {
            if !self.inputs.set(src, level) {
                return;
            }
            let out = self.inputs.resolve(Resolve::Or);
            let out = if self.invert { out.inverted() } else { out };
            if let Some(wire) = self.target.upgrade() {
                wire.set(self.id, out);
            }
        }
    }

    fn is_send_sync<T: Send + Sync>() {}

    #[test]
    fn core_types_are_send_and_sync() {
        // Threading is a configuration, not a later port (ROADMAP.md §0).
        is_send_sync::<Wire>();
        is_send_sync::<FanIn>();
        is_send_sync::<WireSource>();
        is_send_sync::<WireSplit>();
        is_send_sync::<WireOr>();
        is_send_sync::<WireAnd>();
        is_send_sync::<WireNot>();
        is_send_sync::<LevelToEdge>();
        is_send_sync::<EdgeLatch>();
        is_send_sync::<WireIdAllocator>();
    }

    // ---- the bug this design exists to prevent ----------------------------

    #[test]
    fn wired_or_holds_the_line_when_one_source_deasserts() {
        // The APU deasserts its IRQ while the cartridge is still asserting. A
        // sink that only knew "someone said low" would drop the line here.
        let cpu = Irq::new(&[A, B]);
        let wire = Wire::builder()
            .sources(&[A, B])
            .sink(cpu.clone(), 0)
            .build();

        assert_eq!(cpu.level(), Level::Low);

        wire.set(A, Level::High);
        assert_eq!(cpu.level(), Level::High);

        wire.set(B, Level::High);
        assert_eq!(cpu.level(), Level::High);

        wire.set(A, Level::Low);
        assert_eq!(
            cpu.level(),
            Level::High,
            "the line must stay high while another source asserts"
        );
        assert_eq!(wire.resolve(Resolve::Or), Level::High);

        // Only when the last source deasserts does the line drop.
        wire.set(B, Level::Low);
        assert_eq!(cpu.level(), Level::Low);

        // Two transitions of the resolved line, not four.
        assert_eq!(cpu.changes(), 2);
    }

    #[test]
    fn wired_or_through_an_explicit_or_device() {
        // The same again, with the fan-in made explicit as the DSL's resolver
        // would expand it.
        let probe = Probe::new();
        let out = Wire::builder()
            .source(GATE)
            .sink(probe.clone(), 0)
            .build_shared();
        let gate = Arc::new(WireOr::new(&[A, B], WireSource::new(out, GATE)));
        let net = Wire::builder()
            .sources(&[A, B])
            .sink(gate.clone(), 0)
            .build();

        net.set(A, Level::High);
        net.set(B, Level::High);
        assert_eq!(probe.level(), Level::High);
        net.set(A, Level::Low);
        assert_eq!(probe.level(), Level::High);
        assert_eq!(gate.level(), Level::High);
        net.set(B, Level::Low);
        assert_eq!(probe.level(), Level::Low);
    }

    #[test]
    fn a_source_that_was_never_declared_changes_nothing() {
        // The resolver rejects such a graph; the runtime must not corrupt the
        // net's state on the way to that diagnosis.
        let cpu = Irq::new(&[A]);
        let wire = Wire::builder().source(A).sink(cpu.clone(), 0).build();
        wire.set(A, Level::High);
        assert!(!wire.set(C, Level::Low));
        assert_eq!(cpu.level(), Level::High);
        assert_eq!(wire.level_of(C), None);
        assert!(!wire.inputs().contains(C));
    }

    #[test]
    fn repeating_a_level_delivers_nothing() {
        // Devices assert unconditionally every cycle; that must be free.
        let probe = Probe::new();
        let wire = Wire::builder().source(A).sink(probe.clone(), 0).build();
        assert!(wire.set(A, Level::High));
        assert!(!wire.set(A, Level::High));
        assert!(!wire.set(A, Level::High));
        assert_eq!(probe.calls(), 1);
    }

    // ---- fan-in and fan-out ------------------------------------------------

    #[test]
    fn wired_and_needs_every_source() {
        let probe = Probe::new();
        let out = Wire::builder()
            .source(GATE)
            .sink(probe.clone(), 0)
            .build_shared();
        let gate = Arc::new(WireAnd::new(&[A, B], WireSource::new(out, GATE)));
        let net = Wire::builder()
            .sources(&[A, B])
            .sink(gate.clone(), 0)
            .build();

        net.set(A, Level::High);
        assert_eq!(probe.level(), Level::Low);
        net.set(B, Level::High);
        assert_eq!(probe.level(), Level::High);
        net.set(A, Level::Low);
        assert_eq!(probe.level(), Level::Low);
        assert_eq!(gate.level(), Level::Low);
    }

    #[test]
    fn an_and_with_no_sources_reads_as_a_pull_up() {
        let idle = FanIn::new(&[]);
        assert_eq!(idle.resolve(Resolve::And), Level::High);
        assert_eq!(idle.resolve(Resolve::Or), Level::Low);
    }

    #[test]
    fn one_wire_fans_out_to_several_sinks() {
        let a = Probe::new();
        let b = Probe::new();
        let c = Irq::new(&[A]);
        let wire = Wire::builder()
            .source(A)
            .sink(a.clone(), 0)
            .sink(b.clone(), 7)
            .sink(c.clone(), 3)
            .build();
        assert_eq!(wire.sink_count(), 3);

        wire.set(A, Level::High);
        assert_eq!(a.level(), Level::High);
        assert_eq!(b.level(), Level::High);
        assert_eq!(c.level(), Level::High);
        assert_eq!(b.calls(), 1);
        assert_eq!(c.last_line.load(SeqCst), 3, "each sink gets its own line");

        wire.set(A, Level::Low);
        assert_eq!(a.level(), Level::Low);
        assert_eq!(b.level(), Level::Low);
        assert_eq!(c.level(), Level::Low);
    }

    #[test]
    fn a_weak_sink_is_skipped_once_it_is_dropped() {
        let probe = Probe::new();
        let weak: Weak<dyn WireSink> = Arc::downgrade(&(probe.clone() as Arc<dyn WireSink>));
        let wire = Wire::builder().source(A).sink_weak(weak, 0).build();
        wire.set(A, Level::High);
        assert_eq!(probe.level(), Level::High);

        drop(probe);
        // The sink is gone; delivery must be a no-op rather than a panic.
        assert!(wire.set(A, Level::Low));
    }

    #[test]
    fn split_forwards_to_every_output() {
        let x = Probe::new();
        let y = Probe::new();
        let out_x = Wire::builder()
            .source(GATE)
            .sink(x.clone(), 0)
            .build_shared();
        let out_y = Wire::builder()
            .source(GATE)
            .sink(y.clone(), 0)
            .build_shared();
        let split = Arc::new(WireSplit::new(
            &[A],
            vec![WireSource::new(out_x, GATE), WireSource::new(out_y, GATE)],
        ));
        let net = Wire::builder().source(A).sink(split.clone(), 0).build();

        net.set(A, Level::High);
        assert_eq!(x.level(), Level::High);
        assert_eq!(y.level(), Level::High);
        assert_eq!(split.level(), Level::High);
        net.set(A, Level::Low);
        assert_eq!(x.level(), Level::Low);
        assert_eq!(y.level(), Level::Low);
    }

    #[test]
    fn not_inverts_and_nors_multiple_inputs() {
        let probe = Probe::new();
        let out = Wire::builder()
            .source(GATE)
            .sink(probe.clone(), 0)
            .build_shared();
        let inv = Arc::new(WireNot::new(&[A, B], WireSource::new(out, GATE)));
        let net = Wire::builder()
            .sources(&[A, B])
            .sink(inv.clone(), 0)
            .build();

        // Nothing has been driven yet, so the probe has seen nothing; the
        // inverter's own view is correct from the start, and announcing it
        // puts the idle high onto the wire.
        assert_eq!(inv.level(), Level::High);
        assert_eq!(probe.level(), Level::Low);
        inv.announce();
        assert_eq!(probe.level(), Level::High);

        net.set(A, Level::High);
        assert_eq!(probe.level(), Level::Low);
        net.set(B, Level::High);
        assert_eq!(probe.level(), Level::Low);
        net.set(A, Level::Low);
        assert_eq!(probe.level(), Level::Low, "NOR: B still asserts");
        net.set(B, Level::Low);
        assert_eq!(probe.level(), Level::High);
        assert_eq!(inv.level(), Level::High);
    }

    // ---- edges -------------------------------------------------------------

    fn edge_rig(trigger: EdgeTrigger) -> (Wire, Arc<LevelToEdge>, Arc<EdgeLatch>) {
        let latch = Arc::new(EdgeLatch::new(Level::High));
        let out = Wire::builder()
            .source(GATE)
            .sink(latch.clone(), 0)
            .build_shared();
        let det = Arc::new(LevelToEdge::new(&[A], trigger, WireSource::new(out, GATE)));
        let net = Wire::builder().source(A).sink(det.clone(), 0).build();
        (net, det, latch)
    }

    #[test]
    fn rising_edges_only() {
        let (net, det, latch) = edge_rig(EdgeTrigger::Rising);

        net.set(A, Level::High);
        assert!(latch.take(), "a rising edge is latched");
        assert!(!latch.take(), "and taking it clears the latch");
        assert_eq!(det.edge_count(), 1);

        net.set(A, Level::Low);
        assert!(
            !latch.peek(),
            "a falling edge does not fire a rising trigger"
        );
        assert_eq!(det.edge_count(), 1);

        net.set(A, Level::High);
        assert!(latch.take());
        assert_eq!(det.edge_count(), 2);
        assert_eq!(latch.count(), 2);
    }

    #[test]
    fn falling_edges_only() {
        let (net, det, latch) = edge_rig(EdgeTrigger::Falling);

        net.set(A, Level::High);
        assert!(!latch.peek());
        assert_eq!(det.last_level(), Level::High);
        net.set(A, Level::Low);
        assert!(latch.take(), "a falling edge is latched");
        assert_eq!(det.edge_count(), 1);
    }

    #[test]
    fn both_edges() {
        let (net, det, latch) = edge_rig(EdgeTrigger::Both);
        net.set(A, Level::High);
        assert!(latch.take());
        net.set(A, Level::Low);
        assert!(latch.take());
        assert_eq!(det.edge_count(), 2);
    }

    #[test]
    fn an_edge_detector_resolves_its_inputs_before_detecting() {
        // Two sources on one detector: a second source asserting while the
        // first already does is not a new edge, and the first deasserting while
        // the second holds is not a falling edge either.
        let latch = Arc::new(EdgeLatch::new(Level::High));
        let out = Wire::builder()
            .source(GATE)
            .sink(latch.clone(), 0)
            .build_shared();
        let det = Arc::new(LevelToEdge::new(
            &[A, B],
            EdgeTrigger::Both,
            WireSource::new(out, GATE),
        ));
        let net = Wire::builder()
            .sources(&[A, B])
            .sink(det.clone(), 0)
            .build();

        net.set(A, Level::High);
        assert_eq!(det.edge_count(), 1);
        net.set(B, Level::High);
        assert_eq!(det.edge_count(), 1);
        net.set(A, Level::Low);
        assert_eq!(det.edge_count(), 1);
        net.set(B, Level::Low);
        assert_eq!(det.edge_count(), 2);
        assert!(latch.take());
    }

    #[test]
    fn a_pulse_leaves_the_line_where_it_started() {
        // The transient means something only to something that latches it.
        let probe = Probe::new();
        let latch = Arc::new(EdgeLatch::new(Level::High));
        let out = Wire::builder()
            .source(GATE)
            .sink(probe.clone(), 0)
            .sink(latch.clone(), 0)
            .build_shared();
        WireSource::new(out, GATE).pulse(Level::High);
        assert_eq!(probe.level(), Level::Low);
        assert_eq!(probe.calls(), 2, "both transitions are delivered");
        assert!(latch.take());
    }

    // ---- depth, re-entrancy, cycles ---------------------------------------

    #[test]
    fn propagation_survives_a_deep_chain() {
        // Sixteen inverters in series: an even number, so the far end follows
        // the near end. Also the stack-depth check.
        const STAGES: u64 = 16;
        let probe = Probe::new();

        // Built from the far end backwards, so each stage can be handed the
        // wire it drives.
        let mut wire = Wire::builder()
            .source(stage_id(STAGES))
            .sink(probe.clone(), 0)
            .build_shared();
        let mut stages: Vec<Arc<WireNot>> = Vec::new();
        for stage in (0..STAGES).rev() {
            let inv = Arc::new(WireNot::new(
                &[stage_id(stage)],
                WireSource::new(wire, stage_id(stage + 1)),
            ));
            stages.push(inv.clone());
            wire = Wire::builder()
                .source(stage_id(stage))
                .sink(inv, 0)
                .build_shared();
        }
        // Bring the chain up as a reset would: announce in topological order,
        // otherwise every stage's undriven output disagrees with its input.
        for inv in stages.iter().rev() {
            inv.announce();
        }
        assert_eq!(probe.level(), Level::Low, "an even chain follows its input");

        wire.set(stage_id(0), Level::High);
        assert_eq!(probe.level(), Level::High);
        wire.set(stage_id(0), Level::Low);
        assert_eq!(probe.level(), Level::Low);
        assert_eq!(wire.unsettled(), 0);
    }

    /// A sink that mirrors source `A` onto source `B` of the very wire that is
    /// notifying it: re-entrancy that settles, because its own change is
    /// idempotent.
    #[derive(Debug)]
    struct Mirror {
        target: Weak<Wire>,
        calls: AtomicUsize,
    }

    impl WireSink for Mirror {
        fn set_level(&self, src: WireId, _line: u32, level: Level) {
            self.calls.fetch_add(1, SeqCst);
            if src == A
                && let Some(wire) = self.target.upgrade()
            {
                wire.set(B, level);
            }
        }
    }

    #[test]
    fn a_sink_may_drive_the_wire_that_is_notifying_it() {
        let mut sink: Option<Arc<Mirror>> = None;
        let wire = Arc::new_cyclic(|me: &Weak<Wire>| {
            let m = Arc::new(Mirror {
                target: me.clone(),
                calls: AtomicUsize::new(0),
            });
            sink = Some(m.clone());
            Wire::builder().sources(&[A, B]).sink(m, 0).build()
        });
        let sink = sink.expect("built");

        wire.set(A, Level::High);
        assert_eq!(wire.level_of(B), Some(Level::High), "B mirrored A");
        assert_eq!(
            sink.calls.load(SeqCst),
            2,
            "the re-entrant change is delivered by the outer pass, once"
        );
        assert_eq!(
            wire.unsettled(),
            0,
            "a settling feedback path is not a loop"
        );

        wire.set(A, Level::Low);
        assert_eq!(wire.level_of(B), Some(Level::Low));
        assert_eq!(wire.unsettled(), 0);
    }

    #[test]
    fn a_combinational_loop_is_bounded_rather_than_fatal() {
        // An inverter wired back into its own input oscillates forever in
        // hardware and would recurse forever in a naive implementation. Here it
        // costs SETTLE_LIMIT passes and a diagnostic.
        let wire = Arc::new_cyclic(|me: &Weak<Wire>| {
            let gate = Arc::new(WeakGate {
                inputs: FanIn::new(&[A, GATE]),
                target: me.clone(),
                id: GATE,
                invert: true,
            });
            Wire::builder().sources(&[A, GATE]).sink(gate, 0).build()
        });

        // A high: the NOR output is low, which is where it already sits, so the
        // loop is stable.
        wire.set(A, Level::High);
        assert_eq!(wire.unsettled(), 0);

        // A low: the output must flip, which flips the input, which flips the
        // output — the classic ring oscillator.
        wire.set(A, Level::Low);
        assert!(
            wire.unsettled() > 0,
            "an oscillating loop must be reported, not hang or overflow"
        );
        // The wire is still usable afterwards; state is not corrupted.
        assert!(wire.level_of(GATE).is_some());
    }

    #[test]
    fn a_two_wire_loop_is_also_bounded() {
        // The cycle runs through two wires, so no single wire's guard catches
        // the re-entry on its first pass. The outer wire's settle limit still
        // ends it, and the stack stays two wires deep.
        let mut inner: Option<Arc<Wire>> = None;
        let w1 = Arc::new_cyclic(|w1: &Weak<Wire>| {
            // Non-inverting relay from w2 back onto w1's GATE2 source.
            let relay = Arc::new(WeakGate {
                inputs: FanIn::new(&[GATE]),
                target: w1.clone(),
                id: GATE2,
                invert: false,
            });
            let w2 = Wire::builder().source(GATE).sink(relay, 0).build_shared();
            inner = Some(w2.clone());
            // One inversion in the loop, so it cannot settle.
            let inv = Arc::new(WireNot::new(&[A, GATE2], WireSource::new(w2, GATE)));
            Wire::builder().sources(&[A, GATE2]).sink(inv, 0).build()
        });
        assert!(inner.is_some());

        w1.set(A, Level::High);
        w1.set(A, Level::Low);
        assert!(w1.unsettled() > 0, "an odd-inversion loop must be bounded");
    }

    // ---- state -------------------------------------------------------------

    #[test]
    fn wire_state_round_trips() {
        let cpu = Irq::new(&[A, B]);
        let wire = Wire::builder()
            .sources(&[A, B])
            .sink(cpu.clone(), 0)
            .build();
        wire.set(A, Level::High);
        let saved = wire.snapshot();
        assert_eq!(saved, vec![(A, Level::High), (B, Level::Low)]);

        wire.set(A, Level::Low);
        wire.set(B, Level::High);
        assert_ne!(wire.snapshot(), saved);

        wire.restore(&saved);
        assert_eq!(wire.snapshot(), saved);
        // Restore is silent, so the sink is stale until refreshed: that is the
        // contract, because on a real load the sink restores itself.
        assert_eq!(cpu.inputs.level_of(B), Some(Level::High));
        wire.refresh();
        assert_eq!(cpu.inputs.level_of(A), Some(Level::High));
        assert_eq!(cpu.inputs.level_of(B), Some(Level::Low));
        assert_eq!(cpu.level(), Level::High);
    }

    #[test]
    fn edge_detector_state_round_trips_without_inventing_an_edge() {
        let (net, det, latch) = edge_rig(EdgeTrigger::Rising);
        net.set(A, Level::High);
        assert!(latch.take());
        let saved = det.snapshot();
        assert_eq!(saved.last, Level::High);

        // Later in the timeline the line drops and rises again.
        net.set(A, Level::Low);
        net.set(A, Level::High);
        assert!(latch.take());
        assert_eq!(det.edge_count(), 2);

        // Loading the snapshot back must emit nothing, and must not re-arm the
        // detector for a level that was already high.
        det.restore(&saved);
        latch.clear();
        assert_eq!(det.snapshot(), saved);
        assert!(!latch.peek(), "restoring state emits nothing");
        net.set(A, Level::High);
        assert!(!latch.peek(), "already high: no edge");
    }

    #[test]
    fn latch_state_round_trips() {
        let latch = EdgeLatch::new(Level::High);
        latch.set_level(A, 0, Level::High);
        assert!(latch.snapshot());
        latch.clear();
        latch.restore(true);
        assert!(latch.take());
        assert!(!latch.peek());
    }

    #[test]
    fn fan_in_tracks_and_resolves() {
        let f = FanIn::new(&[B, A, A]);
        assert_eq!(f.sources(), &[A, B], "sorted and deduplicated");
        assert!(f.set(A, Level::High));
        assert!(!f.set(A, Level::High));
        assert_eq!(f.level_of(A), Some(Level::High));
        assert_eq!(f.resolve(Resolve::Or), Level::High);
        assert_eq!(f.resolve(Resolve::And), Level::Low);
        assert!(f.set(B, Level::High));
        assert!(f.all_high());
        assert_eq!(f.resolve(Resolve::And), Level::High);
        f.clear();
        assert!(!f.any_high());
        assert_eq!(f.resolve(Resolve::Or), Level::Low);
        assert!(!f.set(C, Level::High), "an untracked source is ignored");
    }

    #[test]
    fn ids_are_allocated_per_machine_and_never_zero() {
        let alloc = WireIdAllocator::new();
        let first = alloc.alloc();
        let second = alloc.alloc();
        assert_ne!(first, WireId::NONE);
        assert_ne!(first, second);
        assert_eq!(first.raw(), 1);
        // A second machine numbers its wires exactly the same way.
        let other = WireIdAllocator::default();
        assert_eq!(other.alloc(), first);
    }

    #[test]
    fn level_and_edge_helpers() {
        assert_eq!(Level::default(), Level::Low);
        assert_eq!(Level::High.inverted(), Level::Low);
        assert!(Level::from_bool(true).is_high());
        assert!(Level::from_bool(false).is_low());
        assert!(bool::from(Level::High));
        assert_eq!(Level::from(true), Level::High);
        assert_eq!(Edge::between(Level::Low, Level::High), Some(Edge::Rising));
        assert_eq!(Edge::between(Level::High, Level::Low), Some(Edge::Falling));
        assert_eq!(Edge::between(Level::High, Level::High), None);
        assert!(EdgeTrigger::Both.matches(Edge::Falling));
        assert!(!EdgeTrigger::Rising.matches(Edge::Falling));
        assert_eq!(Resolve::And.idle(), Level::High);
        assert_eq!(Resolve::default(), Resolve::Or);
    }

    #[test]
    fn source_port_drives_and_reads_back() {
        let probe = Probe::new();
        let wire = Wire::builder()
            .source(A)
            .sink(probe.clone(), 0)
            .build_shared();
        let port = WireSource::new(wire.clone(), A);
        assert_eq!(port.id(), A);
        assert_eq!(port.level(), Level::Low);
        assert!(port.raise());
        assert_eq!(port.level(), Level::High);
        assert_eq!(probe.level(), Level::High);
        assert!(port.lower());
        assert_eq!(port.level(), Level::Low);
        assert!(Arc::ptr_eq(port.wire(), &wire));
    }
    // -----------------------------------------------------------------------
    // The acknowledge cycle
    // -----------------------------------------------------------------------

    /// A controller that claims one level, or every cycle when it has no level
    /// of its own — the two shapes a real one comes in.
    #[derive(Debug)]
    struct Claiming {
        level: Option<u8>,
        answer: IntAckResponse,
        asked: AtomicUsize,
    }

    impl Claiming {
        fn new(level: Option<u8>, answer: IntAckResponse) -> Arc<Claiming> {
            Arc::new(Claiming {
                level,
                answer,
                asked: AtomicUsize::new(0),
            })
        }

        fn asked(&self) -> usize {
            self.asked.load(SeqCst)
        }
    }

    impl IntAck for Claiming {
        fn acknowledge(&self, cycle: IntAckCycle) -> IntAckResponse {
            self.asked.fetch_add(1, SeqCst);
            match self.level {
                Some(level) if cycle.level() != Some(level) => IntAckResponse::Declined,
                _ => self.answer,
            }
        }
    }

    #[test]
    fn a_cycle_carries_only_what_its_kind_presents() {
        let level = IntAckCycle::at_level(5);
        assert_eq!(level.kind(), IntAckKind::LEVEL);
        assert_eq!(level.level(), Some(5));
        assert_eq!(level.mode(), None);

        // The distinction a bare integer could not make: "no level" is not
        // "level zero".
        let plain = IntAckCycle::vector_only();
        assert_eq!(plain.kind(), IntAckKind::VECTOR);
        assert_eq!(plain.level(), None);
        assert_ne!(plain, IntAckCycle::at_level(0));

        let z80 = IntAckCycle::data_bus(2);
        assert_eq!(z80.mode(), Some(2));
        assert_eq!(z80.level(), None);
        assert_eq!(z80.detail(), 2);
    }

    #[test]
    fn declining_passes_the_cycle_on_and_answering_ends_it() {
        let handlers = IntAckHandlers::new();
        assert!(handlers.is_empty());
        assert_eq!(
            handlers.run(IntAckCycle::at_level(1)),
            IntAckResponse::Declined,
            "nothing attached declines"
        );

        let low = Claiming::new(Some(2), IntAckResponse::Vector(80));
        let high = Claiming::new(Some(5), IntAckResponse::Vector(96));
        handlers.attach(Arc::downgrade(&low) as Weak<dyn IntAck>);
        handlers.attach(Arc::downgrade(&high) as Weak<dyn IntAck>);
        assert_eq!(handlers.len(), 2);

        assert_eq!(
            handlers.run(IntAckCycle::at_level(5)),
            IntAckResponse::Vector(96)
        );
        assert_eq!(low.asked(), 1, "asked, and declined");
        assert_eq!(high.asked(), 1);

        assert_eq!(
            handlers.run(IntAckCycle::at_level(2)),
            IntAckResponse::Vector(80)
        );
        assert_eq!(low.asked(), 2);
        assert_eq!(high.asked(), 1, "a cycle that was taken is not passed on");

        // `VPA` is an answer, not a decline: it ends the cycle too.
        let vpa = Claiming::new(Some(3), IntAckResponse::Autovector);
        let behind = Claiming::new(None, IntAckResponse::Vector(112));
        let chain = IntAckHandlers::new();
        chain.attach(Arc::downgrade(&vpa) as Weak<dyn IntAck>);
        chain.attach(Arc::downgrade(&behind) as Weak<dyn IntAck>);
        assert_eq!(
            chain.run(IntAckCycle::at_level(3)),
            IntAckResponse::Autovector
        );
        assert_eq!(behind.asked(), 0);
        // And when the first one declines, the one behind it answers whatever
        // level it is asked at.
        assert_eq!(
            chain.run(IntAckCycle::at_level(4)),
            IntAckResponse::Vector(112)
        );
        assert_eq!(behind.asked(), 1);
    }

    #[test]
    fn the_same_controller_offered_twice_is_kept_once() {
        let handlers = IntAckHandlers::new();
        let pic = Claiming::new(None, IntAckResponse::Vector(8));
        handlers.attach(Arc::downgrade(&pic) as Weak<dyn IntAck>);
        handlers.attach(Arc::downgrade(&pic) as Weak<dyn IntAck>);
        assert_eq!(handlers.len(), 1, "one controller, two nets");
        assert_eq!(
            handlers.run(IntAckCycle::vector_only()),
            IntAckResponse::Vector(8)
        );
        assert_eq!(pic.asked(), 1);
    }

    #[test]
    fn a_controller_the_machine_has_dropped_is_skipped() {
        let handlers = IntAckHandlers::new();
        let gone = Claiming::new(None, IntAckResponse::Vector(1));
        let live = Claiming::new(None, IntAckResponse::Vector(2));
        handlers.attach(Arc::downgrade(&gone) as Weak<dyn IntAck>);
        handlers.attach(Arc::downgrade(&live) as Weak<dyn IntAck>);
        drop(gone);
        assert_eq!(
            handlers.run(IntAckCycle::vector_only()),
            IntAckResponse::Vector(2),
            "the weak edge is the point: a dead controller answers nothing"
        );
        handlers.clear();
        assert!(handlers.is_empty());
    }

    #[test]
    fn a_response_reports_what_it_supplied() {
        assert_eq!(IntAckResponse::Vector(0x40).vector(), Some(0x40));
        assert_eq!(IntAckResponse::Autovector.vector(), None);
        assert!(IntAckResponse::Autovector.answered());
        assert!(!IntAckResponse::Declined.answered());
    }

    // -- tri-state and per-net resolution ---------------------------------

    /// An open-drain stage: it pulls low or it lets go, nothing else.
    fn open_drain(wire: &Arc<Wire>, src: WireId, low: bool) {
        wire.drive(src, if low { Drive::Low } else { Drive::HiZ });
    }

    #[test]
    fn a_drive_says_which_of_three_things_a_stage_is_doing() {
        assert_eq!(Drive::strong(Level::High), Drive::High);
        assert_eq!(Drive::weak(Level::Low), Drive::WeakLow);
        assert_eq!(Drive::open_drain(Level::Low), Drive::Low);
        assert_eq!(
            Drive::open_drain(Level::High),
            Drive::HiZ,
            "an open-drain stage asked for a one lets go; that is the whole point"
        );
        assert_eq!(Drive::HiZ.level(), None);
        assert_eq!(Drive::WeakHigh.level(), Some(Level::High));
        assert!(Drive::Low.is_strong() && !Drive::WeakLow.is_strong());
        assert!(Drive::WeakLow.is_weak() && !Drive::Low.is_weak());
        assert!(Drive::HiZ.is_hiz());
        // `Low` and `High` keep codes 0 and 1, which is what lets a snapshot
        // written before tri-state existed still decode.
        assert_eq!(Drive::Low.code(), 0);
        assert_eq!(Drive::High.code(), 1);
        for d in [
            Drive::HiZ,
            Drive::WeakLow,
            Drive::WeakHigh,
            Drive::Low,
            Drive::High,
        ] {
            assert_eq!(Drive::from_code(d.code()), Some(d));
        }
        assert_eq!(Drive::from_code(5), None);
    }

    #[test]
    fn a_pull_up_holds_a_net_nobody_is_driving() {
        let probe = Probe::new();
        let wire = Wire::builder()
            .sources(&[A, B])
            .sink(probe.clone() as Arc<dyn WireSink>, 0)
            .resolved(Pull::Up)
            .build_shared();
        // Nothing has driven anything: every source starts Hi-Z, not low, which
        // is the distinction the mode exists for.
        assert_eq!(wire.drive_of(A), Some(Drive::HiZ));
        assert_eq!(wire.resolve_net(), Level::High);
        wire.refresh();
        assert_eq!(probe.level(), Level::High);
        // One open-drain stage pulls the whole net down...
        open_drain(&wire, A, true);
        assert_eq!(probe.level(), Level::Low);
        // ...and while it holds, the other letting go changes nothing.
        open_drain(&wire, B, false);
        assert_eq!(probe.level(), Level::Low);
        open_drain(&wire, A, false);
        assert_eq!(probe.level(), Level::High);
        assert_eq!(wire.contention(), 0);
    }

    #[test]
    fn a_pull_down_is_the_same_arrangement_upside_down() {
        let probe = Probe::new();
        let wire = Wire::builder()
            .sources(&[A])
            .sink(probe.clone() as Arc<dyn WireSink>, 0)
            .resolved(Pull::Down)
            .build_shared();
        wire.refresh();
        assert_eq!(probe.level(), Level::Low);
        wire.drive(A, Drive::High);
        assert_eq!(
            probe.level(),
            Level::High,
            "a strong driver beats a resistor"
        );
        wire.drive(A, Drive::HiZ);
        assert_eq!(probe.level(), Level::Low);
    }

    #[test]
    fn strength_beats_polarity_and_a_resistor_never_wins() {
        let wire = Wire::builder()
            .sources(&[A, B])
            .resolved(Pull::Up)
            .build_shared();
        wire.drive(A, Drive::WeakLow);
        assert_eq!(
            wire.resolve_net(),
            Level::Low,
            "two resistors in opposition is a divider, not a level; the model \
             answers low and counts it"
        );
        assert!(wire.contention() > 0);
        let before = wire.contention();
        wire.drive(B, Drive::High);
        assert_eq!(
            wire.resolve_net(),
            Level::High,
            "a stage overrules both resistors"
        );
        assert_eq!(
            wire.contention(),
            before,
            "one strong driver is not a fight"
        );
        wire.drive(A, Drive::Low);
        assert_eq!(wire.resolve_net(), Level::Low);
        assert!(
            wire.contention() > before,
            "two stages in opposition is a short"
        );
    }

    #[test]
    fn a_net_with_no_driver_and_no_resistor_is_reported_rather_than_guessed() {
        let wire = Wire::builder()
            .sources(&[A])
            .resolved(Pull::None)
            .build_shared();
        assert_eq!(wire.resolve_net(), Level::Low);
        wire.refresh();
        assert!(
            wire.contention() > 0,
            "a floating net has no level; answering low is a choice, and the \
             counter is how a machine description finds out it made it"
        );
    }

    #[test]
    fn the_resolution_does_not_depend_on_the_order_drivers_registered_in() {
        // The same three drives, declared both ways round, must resolve alike.
        let forward = Wire::builder()
            .sources(&[A, B, C])
            .resolved(Pull::Up)
            .build_shared();
        let backward = Wire::builder()
            .sources(&[C, B, A])
            .resolved(Pull::Up)
            .build_shared();
        for wire in [&forward, &backward] {
            wire.drive(A, Drive::WeakHigh);
            wire.drive(B, Drive::HiZ);
            wire.drive(C, Drive::Low);
        }
        assert_eq!(forward.resolve_net(), backward.resolve_net());
        assert_eq!(forward.resolve_net(), Level::Low);
        assert_eq!(forward.snapshot_drives(), backward.snapshot_drives());
    }

    #[test]
    fn a_fan_in_keeping_sink_stays_correct_on_a_resolved_net() {
        // The compatibility claim in `deliver_resolved`, asserted: a sink that
        // wired-ORs its sources — which is every non-test sink in the tree —
        // reports the net's resolved level without knowing tri-state exists.
        let irq = Irq::new(&[A, B]);
        let wire = Wire::builder()
            .sources(&[A, B])
            .sink(irq.clone() as Arc<dyn WireSink>, 0)
            .resolved(Pull::Up)
            .build_shared();
        wire.refresh();
        assert_eq!(
            irq.level(),
            Level::High,
            "the pull-up, through an OR-ing sink"
        );
        open_drain(&wire, B, true);
        assert_eq!(
            irq.level(),
            Level::Low,
            "an OR of all-low entries is low, which is what the net says"
        );
        open_drain(&wire, B, false);
        assert_eq!(irq.level(), Level::High);
    }

    #[test]
    fn a_per_sink_net_is_untouched_by_any_of_this() {
        // The default, and what every interrupt line in the tree is: the wire
        // delivers the driver's own level and the sink resolves.
        let irq = Irq::new(&[A, B]);
        let wire = Wire::builder()
            .sources(&[A, B])
            .sink(irq.clone() as Arc<dyn WireSink>, 0)
            .build_shared();
        assert_eq!(wire.mode(), NetMode::PerSink);
        assert_eq!(wire.drive_of(A), Some(Drive::Low), "still low, not Hi-Z");
        wire.set(A, Level::High);
        wire.set(B, Level::High);
        assert_eq!(irq.level(), Level::High);
        wire.set(A, Level::Low);
        assert_eq!(
            irq.level(),
            Level::High,
            "the shared-interrupt case: B still asserts"
        );
        assert_eq!(
            wire.contention(),
            0,
            "a per-sink net cannot express a short"
        );
    }

    #[test]
    fn a_resolved_net_says_nothing_when_a_driver_moves_without_moving_it() {
        let probe = Probe::new();
        let wire = Wire::builder()
            .sources(&[A, B])
            .sink(probe.clone() as Arc<dyn WireSink>, 0)
            .resolved(Pull::Up)
            .build_shared();
        open_drain(&wire, A, true);
        open_drain(&wire, B, true);
        let calls = probe.calls();
        // B letting go while A still holds the net down is not news.
        open_drain(&wire, B, false);
        assert_eq!(probe.calls(), calls);
        assert_eq!(probe.level(), Level::Low);
    }

    #[test]
    fn a_resolved_nets_drives_round_trip_through_a_snapshot() {
        let wire = Wire::builder()
            .sources(&[A, B])
            .resolved(Pull::Up)
            .build_shared();
        wire.drive(A, Drive::Low);
        wire.drive(B, Drive::WeakHigh);
        let saved = wire.snapshot_drives();
        assert_eq!(saved, vec![(A, Drive::Low), (B, Drive::WeakHigh)]);
        let restored = Wire::builder()
            .sources(&[A, B])
            .resolved(Pull::Up)
            .build_shared();
        restored.restore_drives(&saved);
        assert_eq!(
            restored.resolve_net(),
            wire.resolve_net(),
            "a level flattened to two states would have brought the net back \
             at the wrong rail"
        );
        assert_eq!(restored.snapshot_drives(), saved);
    }

    #[test]
    fn a_pull_is_spelled_the_way_a_machine_file_writes_it() {
        assert_eq!(Pull::from_name("up"), Some(Pull::Up));
        assert_eq!(Pull::from_name("down"), Some(Pull::Down));
        assert_eq!(Pull::from_name("none"), Some(Pull::None));
        assert_eq!(Pull::from_name("float"), Some(Pull::None));
        assert_eq!(Pull::from_name("pullup"), None);
        assert_eq!(Pull::Up.name(), "up");
        assert_eq!(Pull::Up.drive(), Drive::WeakHigh);
        assert_eq!(Pull::None.drive(), Drive::HiZ);
        assert_eq!(NetMode::Resolved(Pull::Down).pull(), Pull::Down);
        assert!(!NetMode::PerSink.is_resolved());
    }

    #[test]
    fn a_source_handle_can_let_go_and_read_the_net_back() {
        let wire = Wire::builder()
            .sources(&[A, B])
            .resolved(Pull::Up)
            .build_shared();
        let a = WireSource::new(Arc::clone(&wire), A);
        let b = WireSource::new(Arc::clone(&wire), B);
        a.drive(Drive::Low);
        assert_eq!(a.drive_state(), Drive::Low);
        assert_eq!(
            b.net_level(),
            Level::Low,
            "a pin reads the net, not its own stage — which is how an \
             open-drain bus is sensed at all"
        );
        assert_eq!(b.drive_state(), Drive::HiZ);
        a.release();
        assert_eq!(b.net_level(), Level::High);
    }
}
