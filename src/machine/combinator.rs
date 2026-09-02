//! The `wire.*` combinators: gates a machine file can put between two pins.
//!
//! `ROADMAP.md` §4.3 names five and says what they are for:
//!
//! > Ships with the standard combinators as ordinary devices: `wire.split`,
//! > `wire.or`, `wire.and`, `wire.not`, `wire.level-to-edge`. Interrupt
//! > controllers (i8259, APIC, GIC, PLIC, NES NMI line) are then just devices
//! > with wire sinks and sources — the core knows nothing about "interrupts".
//!
//! They live beside `ram` and `rom` in [`builtin`](super::builtin) rather than
//! under [`dev`](crate::dev) for the same reason those two do: they are not
//! models of any part, they are described entirely by `core::wire`, and every
//! catalog needs them, so a Cargo feature to hold four gates would be ceremony.
//!
//! # Why a board needs them at all
//!
//! Because a wire carries one level and real boards put logic in between. Three
//! cases already written down elsewhere in this tree:
//!
//! * **Active-low pins.** `pc::kbc`'s reset output is active low on the real
//!   8042, and a net here idles low and carries the *logical* assertion; a
//!   machine file that wants the electrical sense puts a `wire.not` in the way.
//!   `riscv::syscon` says the same about its own reset.
//! * **Gated interrupt routes.** An HPET's `LEG_RT_CNF` disconnects the 8254
//!   from IRQ0 and the RTC from IRQ8 when it is set (IA-PC HPET Specification
//!   rev 1.0a §2.3.5). That is a gate between three chips, not a register in
//!   any of them, and it is a `wire.and` with an inverted enable.
//! * **A pin driving two boards' worth of things**, where the machine file
//!   wants the fan-out named rather than implied.
//!
//! # What "combinational" costs and buys
//!
//! All four gates report [`Device::combinational`], which is what puts them in
//! the realize sweep's topological order — a gate is announced *after* whatever
//! drives it, so a freshly realized or freshly restored machine settles in one
//! pass instead of depending on declaration order. The price is that a cycle
//! made only of them is rejected rather than run, which is §4.3's rule and the
//! correct answer: a ring of inverters has no stable level to announce.
//!
//! [`Edge`] is the exception and is deliberately **sequential**: it remembers
//! the last level it saw, so it snapshots, and a cycle through it is an
//! ordinary handshake.
//!
//! # No state, except where there is
//!
//! A gate's output is a function of its inputs, and §4.3 says realize and load
//! both sweep the graph and announce every source in topological order. So the
//! three pure gates serialize nothing at all: whatever they held is recomputed
//! from what drives them, and saving it would be saving a cache. [`Edge`] saves
//! one bit, because "was the line already high" is not derivable from the level
//! alone — it is what decides whether the next notification is an edge.

use alloc::boxed::Box;
use alloc::string::ToString;
use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::device::{Device, DeviceClass, PropertySpec, RealizeCtx, ResetKind, SinkPin};
use crate::core::error::{Error, Result};
use crate::core::props::{Props, ValueKind};
use crate::core::state::{ChunkReader, ChunkWriter, Sink, Source};
use crate::core::sync::{LockRank, Mutex};
use crate::core::wire::{FanIn, Level, Resolve, WireId, WireSink, WireSource};
use crate::machine::realize::{Bindings, Instance};
use crate::machine::validate::{ClassSchema, PortDir, PropSchema};

/// The class name for an inverter.
const NOT_CLASS_NAME: &str = "wire.not";
/// The class name for an AND gate.
const AND_CLASS_NAME: &str = "wire.and";
/// The class name for an OR gate.
const OR_CLASS_NAME: &str = "wire.or";
/// The class name for a fan-out.
const SPLIT_CLASS_NAME: &str = "wire.split";
/// The class name for a level-to-edge converter.
const EDGE_CLASS_NAME: &str = "wire.level-to-edge";

/// The default width of a gate, and the largest a machine file may ask for.
///
/// Two because that is the gate everybody draws. Sixteen because a wider one is
/// almost certainly a machine file that meant to write two statements, and an
/// unbounded count is a pin table a typo can make enormous.
const DEFAULT_INPUTS: u64 = 2;
const MAX_PINS: u64 = 16;

/// Read the pin count a gate was asked for.
fn pin_count(props: &Props, name: &str) -> Result<usize> {
    let mut r = props.reader();
    let n: u64 = r.or_range(name, DEFAULT_INPUTS, 2..=MAX_PINS)?;
    r.finish()?;
    Ok(n as usize)
}

/// `inN` for a pin named `prefix`, or `None` if it is not one of ours.
fn indexed(port: &str, prefix: &str, count: usize) -> Option<usize> {
    let index: usize = port.strip_prefix(prefix)?.parse().ok()?;
    (index < count).then_some(index)
}

/// The error for a pin name a gate does not have.
fn unknown_pin(port: &str, what: &str) -> Error {
    Error::Config {
        at: port.to_string(),
        message: alloc::format!("a `{what}` has no pin by that name"),
    }
}

// ---------------------------------------------------------------------------
// the shared machinery
// ---------------------------------------------------------------------------

/// What every gate here is: some inputs, some outputs, and a rule.
///
/// One struct rather than four, because the four differ only in the rule and in
/// which side is plural — and a second copy of "remember the level, release the
/// lock, then drive" is a second place for the re-entrancy contract to be got
/// wrong.
#[derive(Debug)]
struct Gate {
    /// What each input pin is currently being told, one entry per pin.
    levels: Mutex<Vec<bool>>,
    /// Every net an output pin drives, flat. At [`LockRank::LEAF`] so a line
    /// can be driven with nothing else held.
    ///
    /// Flat, and not one slot per pin, because every output of every gate here
    /// carries the same level — that is what `wire.split` *is* — so which pin a
    /// net arrived on stops mattering the moment the name has been checked. It
    /// also makes a pin that drives two nets fall out for free, which
    /// [`Device::connect`] requires.
    outs: Mutex<Vec<WireSource>>,
    /// How the inputs combine.
    rule: Rule,
}

/// What a gate does with the levels it is holding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Rule {
    /// Every input high.
    And,
    /// Any input high.
    Or,
    /// The one input, inverted.
    Not,
    /// The one input, unchanged, on every output.
    Split,
}

impl Gate {
    fn new(rule: Rule, inputs: usize) -> Gate {
        Gate {
            levels: Mutex::with_rank(LockRank::DEVICE, alloc::vec![false; inputs]),
            outs: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            rule,
        }
    }

    /// What the outputs should read, given the inputs.
    fn resolve(&self) -> bool {
        let levels = self.levels.lock();
        match self.rule {
            Rule::And => levels.iter().all(|&l| l),
            Rule::Or => levels.iter().any(|&l| l),
            Rule::Not => !levels[0],
            Rule::Split => levels[0],
        }
    }

    /// Drive every output from the current inputs, with no lock held.
    ///
    /// The sources are cloned out and the lock released before anything is
    /// driven, which is the re-entrancy contract: an output of one of these
    /// reaches a CPU pin or another gate, and holding a lock across that is how
    /// a board deadlocks against itself.
    fn drive(&self) {
        let level = Level::from_bool(self.resolve());
        let outs: Vec<WireSource> = self.outs.lock().clone();
        for out in outs {
            out.set(level);
        }
    }

    /// One input pin moved.
    fn input(&self, index: usize, level: Level) {
        {
            let mut levels = self.levels.lock();
            let high = level == Level::High;
            if levels[index] == high {
                return;
            }
            levels[index] = high;
        }
        self.drive();
    }
}

/// One input pin, with the fan-in that makes a shared net correct.
#[derive(Debug)]
struct InputPin {
    gate: Arc<Gate>,
    index: usize,
    inputs: FanIn,
}

impl WireSink for InputPin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        // Wired-OR across the *net*, which is a different thing from what the
        // gate then does with the pin: two controllers sharing one input of an
        // AND gate must not let one of them drop the pin while the other
        // asserts (`core::wire`, module docs).
        self.gate
            .input(self.index, self.inputs.resolve(Resolve::Or));
    }
}

/// A gate a machine file can put between two pins.
///
/// See the [module docs](self). Instantiated as `wire.and`, `wire.or`,
/// `wire.not` or `wire.split`; which one decides how many pins it has and what
/// it does with them.
#[derive(Debug)]
pub struct Combinator {
    class: &'static DeviceClass,
    gate: Arc<Gate>,
    inputs: usize,
    outputs: usize,
    /// The sinks handed out by [`Device::sink`], kept alive here: a net holds
    /// only a weak reference to a sink, so the device owns the strong one.
    pins: Mutex<Vec<Arc<InputPin>>>,
}

impl Combinator {
    fn build(class: &'static DeviceClass, rule: Rule, inputs: usize, outputs: usize) -> Combinator {
        Combinator {
            class,
            gate: Arc::new(Gate::new(rule, inputs)),
            inputs,
            outputs,
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        }
    }

    /// An inverter: `in` high means `out` low.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if a property this class does not know was given.
    pub fn not(props: &Props) -> Result<Combinator> {
        props.reader().finish()?;
        Ok(Combinator::build(&NOT_CLASS, Rule::Not, 1, 1))
    }

    /// An AND gate over `inputs` pins, two by default.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `inputs` is out of range or unknown.
    pub fn and(props: &Props) -> Result<Combinator> {
        let n = pin_count(props, "inputs")?;
        Ok(Combinator::build(&AND_CLASS, Rule::And, n, 1))
    }

    /// An OR gate over `inputs` pins, two by default.
    ///
    /// A net with two sources already resolves as a wired-OR (§4.3), so this is
    /// for the machine file that wants the fan-in *named* — and for the one
    /// that needs an OR of things that are not both interrupt sources.
    ///
    /// # Errors
    ///
    /// As [`Combinator::and`].
    pub fn or(props: &Props) -> Result<Combinator> {
        let n = pin_count(props, "inputs")?;
        Ok(Combinator::build(&OR_CLASS, Rule::Or, n, 1))
    }

    /// A fan-out: `in` on `outputs` pins, two by default.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `outputs` is out of range or unknown.
    pub fn split(props: &Props) -> Result<Combinator> {
        let n = pin_count(props, "outputs")?;
        Ok(Combinator::build(&SPLIT_CLASS, Rule::Split, 1, n))
    }

    /// The level each output is currently being driven to.
    #[must_use]
    pub fn output(&self) -> bool {
        self.gate.resolve()
    }
}

impl Device for Combinator {
    fn class(&self) -> &'static DeviceClass {
        self.class
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // Nothing to reset. A gate holds no state of its own: its output is a
        // function of levels other devices are driving, and a reset of this
        // object does not reach across a pin and change one of those. Zeroing
        // the remembered levels here would invent an input level, which is the
        // one thing a device may never do (`CLAUDE.md`) — and would stick,
        // because an unchanged level is never re-announced.
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        let index = if self.inputs == 1 {
            (port == "in").then_some(0)?
        } else {
            indexed(port, "in", self.inputs)?
        };
        let pin = Arc::new(InputPin {
            gate: Arc::clone(&self.gate),
            index,
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin {
            sink: pin,
            line: index as u32,
        })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        let known = if self.outputs == 1 {
            port == "out"
        } else {
            indexed(port, "out", self.outputs).is_some()
        };
        if !known {
            return Err(unknown_pin(port, self.class.name));
        }
        // Appended rather than stored per pin: every output carries the same
        // level, and a pin driving two nets is handed two sources and must
        // drive both.
        self.gate.outs.lock().push(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        // What the realize sweep is for (§4.3): an undriven net sits low, which
        // contradicts an inverter's idle-high output, so a gate says what it is
        // driving as soon as it knows.
        self.gate.drive();
    }

    fn combinational(&self) -> bool {
        true
    }

    fn save(&self, _w: &mut ChunkWriter<'_>) -> Result<()> {
        // Deliberately empty: see the module docs. The output is a function of
        // levels other devices own and the load sweep re-announces every one of
        // them, so anything written here would be a cache of somebody else's
        // state — and a stale one, on a snapshot taken mid-sweep.
        Ok(())
    }

    fn load(&self, _r: &mut ChunkReader<'_>) -> Result<()> {
        Ok(())
    }
}

impl Instance for Combinator {}

// ---------------------------------------------------------------------------
// the edge detector
// ---------------------------------------------------------------------------

/// Which transitions [`Edge`] pulses on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Which {
    Rising,
    Falling,
    Both,
}

/// A level-to-edge converter: a pulse on `out` for each transition of `in`.
///
/// **Sequential**, and §4.3 says why it is a device rather than a flag on a
/// wire: "Level and edge semantics both, with the *edge detector as a device*
/// rather than a flag, so it snapshots correctly." The one bit it holds — was
/// the input already high — is exactly what a flag on a wire could not save.
#[derive(Debug)]
pub struct Edge {
    state: Arc<EdgeState>,
    /// The sinks handed out by [`Device::sink`], kept alive here: a net holds
    /// only a weak reference to a sink, so the device owns the strong one.
    pins: Mutex<Vec<Arc<EdgePin>>>,
}

/// [`Edge`]'s input pin.
#[derive(Debug)]
struct EdgePin {
    owner: Arc<EdgeState>,
    inputs: FanIn,
}

/// The half of an [`Edge`] a pin has to reach.
#[derive(Debug)]
struct EdgeState {
    which: Which,
    /// The last level seen on the input.
    level: Mutex<bool>,
    /// Every net the output pin drives.
    outs: Mutex<Vec<WireSource>>,
}

impl EdgeState {
    /// The input moved. Returns nothing; the pulse is driven here, outside the
    /// critical section.
    fn input(&self, level: Level) {
        let high = level == Level::High;
        let fire = {
            let mut held = self.level.lock();
            if *held == high {
                return;
            }
            *held = high;
            match self.which {
                Which::Rising => high,
                Which::Falling => !high,
                Which::Both => true,
            }
        };
        if !fire {
            return;
        }
        let outs: Vec<WireSource> = self.outs.lock().clone();
        for out in outs {
            out.pulse(Level::High);
        }
    }
}

impl WireSink for EdgePin {
    fn set_level(&self, src: WireId, _line: u32, level: Level) {
        self.inputs.set(src, level);
        self.owner.input(self.inputs.resolve(Resolve::Or));
    }
}

impl Edge {
    /// Build one. `edge` is `"rising"` (the default), `"falling"` or `"both"`.
    ///
    /// # Errors
    ///
    /// [`Error::Property`] if `edge` is none of those, or a property this class
    /// does not know was given.
    pub fn new(props: &Props) -> Result<Edge> {
        let mut r = props.reader();
        let which = match r.or_enum("edge", "rising", &["rising", "falling", "both"])? {
            "falling" => Which::Falling,
            "both" => Which::Both,
            // `or_enum` has already refused anything else by name, with the
            // message `core::props` writes for a misspelt enumeration.
            _ => Which::Rising,
        };
        r.finish()?;
        Ok(Edge {
            state: Arc::new(EdgeState {
                which,
                level: Mutex::with_rank(LockRank::DEVICE, false),
                outs: Mutex::with_rank(LockRank::LEAF, Vec::new()),
            }),
            pins: Mutex::with_rank(LockRank::LEAF, Vec::new()),
        })
    }

    /// Whether the input is currently high.
    #[must_use]
    pub fn input_high(&self) -> bool {
        *self.state.level.lock()
    }
}

impl Device for Edge {
    fn class(&self) -> &'static DeviceClass {
        &EDGE_CLASS
    }

    fn realize(&self, _ctx: &mut RealizeCtx<'_>) -> Result<()> {
        Ok(())
    }

    fn reset(&self, _kind: ResetKind) {
        // The remembered level survives, for the reason every other input pin
        // in this tree survives a reset: it is what another device is driving,
        // and forgetting it turns the next unchanged level into a fabricated
        // edge — or swallows a real one as a repeat.
    }

    fn sink(&self, port: &str, sources: &[WireId]) -> Option<SinkPin> {
        if port != "in" {
            return None;
        }
        let pin = Arc::new(EdgePin {
            owner: Arc::clone(&self.state),
            inputs: FanIn::new(sources),
        });
        self.pins.lock().push(Arc::clone(&pin));
        Some(SinkPin { sink: pin, line: 0 })
    }

    fn connect(&self, port: &str, source: WireSource) -> Result<()> {
        if port != "out" {
            return Err(unknown_pin(port, EDGE_CLASS_NAME));
        }
        self.state.outs.lock().push(source);
        Ok(())
    }

    fn announce(&self, _port: &str) {
        // Nothing. A pulse has no idle level to announce beyond the low a fresh
        // net already sits at, and driving one here would fabricate an edge the
        // machine never had — the same argument `pc::kbc` makes about its reset
        // pin.
    }

    fn save(&self, w: &mut ChunkWriter<'_>) -> Result<()> {
        w.write_bool(self.input_high())
    }

    fn load(&self, r: &mut ChunkReader<'_>) -> Result<()> {
        *self.state.level.lock() = r.read_bool()?;
        Ok(())
    }
}

impl Instance for Edge {}

// ---------------------------------------------------------------------------
// the classes
// ---------------------------------------------------------------------------

/// How many pins a gate has, as a property.
const PIN_COUNT_PROP: &str = "how many, between 2 and 16 (default 2)";

/// The `wire.not` device class.
pub static NOT_CLASS: DeviceClass = DeviceClass {
    name: NOT_CLASS_NAME,
    version: 1,
    summary: "an inverter: `out` is low while `in` is high",
    properties: &[],
    construct: |props| Ok(Box::new(Combinator::not(props)?)),
};

/// The `wire.and` device class.
pub static AND_CLASS: DeviceClass = DeviceClass {
    name: AND_CLASS_NAME,
    version: 1,
    summary: "an AND gate: `out` is high while every `inN` is",
    properties: &[PropertySpec {
        name: "inputs",
        kind: ValueKind::Uint,
        required: false,
        summary: PIN_COUNT_PROP,
    }],
    construct: |props| Ok(Box::new(Combinator::and(props)?)),
};

/// The `wire.or` device class.
pub static OR_CLASS: DeviceClass = DeviceClass {
    name: OR_CLASS_NAME,
    version: 1,
    summary: "an OR gate: `out` is high while any `inN` is",
    properties: &[PropertySpec {
        name: "inputs",
        kind: ValueKind::Uint,
        required: false,
        summary: PIN_COUNT_PROP,
    }],
    construct: |props| Ok(Box::new(Combinator::or(props)?)),
};

/// The `wire.split` device class.
pub static SPLIT_CLASS: DeviceClass = DeviceClass {
    name: SPLIT_CLASS_NAME,
    version: 1,
    summary: "a fan-out: every `outN` follows `in`",
    properties: &[PropertySpec {
        name: "outputs",
        kind: ValueKind::Uint,
        required: false,
        summary: PIN_COUNT_PROP,
    }],
    construct: |props| Ok(Box::new(Combinator::split(props)?)),
};

/// The `wire.level-to-edge` device class.
pub static EDGE_CLASS: DeviceClass = DeviceClass {
    name: EDGE_CLASS_NAME,
    version: 1,
    summary: "a pulse on `out` for each transition of `in`",
    properties: &[PropertySpec {
        name: "edge",
        kind: ValueKind::Str,
        required: false,
        summary: "`rising` (the default), `falling` or `both`",
    }],
    construct: |props| Ok(Box::new(Edge::new(props)?)),
};

/// Add every combinator to a registry.
///
/// # Errors
///
/// [`Error::Config`] if something already claimed one of the names.
pub fn register(registry: &mut crate::core::registry::Registry) -> Result<()> {
    registry.add(&NOT_CLASS)?;
    registry.add(&AND_CLASS)?;
    registry.add(&OR_CLASS)?;
    registry.add(&SPLIT_CLASS)?;
    registry.add(&EDGE_CLASS)
}

/// Bind every combinator into the machine graph.
///
/// # Errors
///
/// [`Error::Config`] if a name is already bound.
pub fn bind(bindings: &mut Bindings) -> Result<()> {
    bindings.bind(NOT_CLASS_NAME, |props| {
        Ok(Arc::new(Combinator::not(props)?))
    })?;
    bindings.bind(AND_CLASS_NAME, |props| {
        Ok(Arc::new(Combinator::and(props)?))
    })?;
    bindings.bind(OR_CLASS_NAME, |props| Ok(Arc::new(Combinator::or(props)?)))?;
    bindings.bind(SPLIT_CLASS_NAME, |props| {
        Ok(Arc::new(Combinator::split(props)?))
    })?;
    bindings.bind(EDGE_CLASS_NAME, |props| Ok(Arc::new(Edge::new(props)?)))
}

/// Every combinator's validator schema.
///
/// The plural side is declared as a **bank** of `MAX_PINS` rather than of
/// however many the object asked for, because the validator does not evaluate
/// properties: a schema built from the `inputs` value would have to be, and a
/// fixed short list would reject `in2` on a legal three-input gate. So the
/// spelling is checked here — `in3` is a pin and `inx` is a typo — and the
/// count is checked at realize, where the object exists.
#[must_use]
pub fn schemas() -> Vec<ClassSchema> {
    let width = MAX_PINS as u32;
    let counted = |name: &str, prop: &str, dir: PortDir, single: &str| {
        ClassSchema::new(name)
            .combinational()
            .prop(PropSchema::new(prop, ValueKind::Uint))
            .port_bank(if dir == PortDir::In { "in" } else { "out" }, dir, width)
            .port(
                single,
                if dir == PortDir::In {
                    PortDir::Out
                } else {
                    PortDir::In
                },
            )
    };
    alloc::vec![
        ClassSchema::new(NOT_CLASS_NAME)
            .combinational()
            .port("in", PortDir::In)
            .port("out", PortDir::Out),
        counted(AND_CLASS_NAME, "inputs", PortDir::In, "out"),
        counted(OR_CLASS_NAME, "inputs", PortDir::In, "out"),
        counted(SPLIT_CLASS_NAME, "outputs", PortDir::Out, "in"),
        ClassSchema::new(EDGE_CLASS_NAME)
            .prop(PropSchema::new("edge", ValueKind::Str))
            .port("in", PortDir::In)
            .port("out", PortDir::Out),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};
    use crate::core::wire::{Wire, WireIdAllocator};
    use core::sync::atomic::{AtomicU32, Ordering};

    /// A pin that remembers its level and counts every transition, so a pulse
    /// is distinguishable from a line that merely ended up back where it was.
    #[derive(Debug, Default)]
    struct Probe {
        level: AtomicU32,
        edges: AtomicU32,
    }

    impl WireSink for Probe {
        fn set_level(&self, _src: WireId, _line: u32, level: Level) {
            self.level
                .store(u32::from(level.is_high()), Ordering::Relaxed);
            self.edges.fetch_add(1, Ordering::Relaxed);
        }
    }

    impl Probe {
        fn high(&self) -> bool {
            self.level.load(Ordering::Relaxed) == 1
        }

        fn edges(&self) -> u32 {
            self.edges.load(Ordering::Relaxed)
        }
    }

    /// Wire `port` of `dev` to a fresh probe and announce it, as the realize
    /// sweep does.
    fn probe(dev: &dyn Device, port: &str) -> Arc<Probe> {
        let id = WireIdAllocator::new().alloc();
        let p = Arc::new(Probe::default());
        let wire = Wire::builder()
            .source(id)
            .sink(Arc::clone(&p) as Arc<dyn WireSink>, 0)
            .build_shared();
        dev.connect(port, WireSource::new(wire, id))
            .expect("the pin exists");
        dev.announce(port);
        p
    }

    /// A driver for one input pin: the sink, and the id to drive it with.
    struct Driver {
        sink: Arc<dyn WireSink>,
        src: WireId,
        line: u32,
    }

    impl Driver {
        fn set(&self, level: Level) {
            self.sink.set_level(self.src, self.line, level);
        }
    }

    fn driver(dev: &dyn Device, port: &str) -> Driver {
        let src = WireIdAllocator::new().alloc();
        let pin = dev.sink(port, &[src]).expect("the pin exists");
        Driver {
            sink: pin.sink,
            src,
            line: pin.line,
        }
    }

    fn props(pairs: &[(&str, u64)]) -> Props {
        let mut p = Props::new();
        for (k, v) in pairs {
            p = p.with(*k, *v);
        }
        p
    }

    #[test]
    fn an_inverter_idles_high_and_says_so_at_realize() {
        let not = Combinator::not(&Props::new()).expect("no properties");
        let out = probe(&not, "out");
        assert!(
            out.high(),
            "an undriven net sits low, which contradicts an inverter's idle \
             output — announcing it is exactly what the realize sweep is for"
        );
        let a = driver(&not, "in");
        a.set(Level::High);
        assert!(!out.high());
        a.set(Level::Low);
        assert!(out.high());
    }

    #[test]
    fn an_and_gate_needs_every_input() {
        let and = Combinator::and(&props(&[("inputs", 3)])).expect("three is legal");
        let out = probe(&and, "out");
        let pins: Vec<Driver> = (0..3)
            .map(|i| driver(&and, &alloc::format!("in{i}")))
            .collect();
        assert!(!out.high());
        pins[0].set(Level::High);
        pins[1].set(Level::High);
        assert!(!out.high(), "two of three");
        pins[2].set(Level::High);
        assert!(out.high(), "and the third");
        pins[1].set(Level::Low);
        assert!(!out.high());
    }

    #[test]
    fn an_or_gate_needs_only_one() {
        let or = Combinator::or(&Props::new()).expect("two by default");
        let out = probe(&or, "out");
        let (a, b) = (driver(&or, "in0"), driver(&or, "in1"));
        assert!(!out.high());
        b.set(Level::High);
        assert!(out.high());
        a.set(Level::High);
        b.set(Level::Low);
        assert!(out.high(), "the other source still asserts");
        a.set(Level::Low);
        assert!(!out.high());
    }

    #[test]
    fn a_split_drives_every_output() {
        let split = Combinator::split(&props(&[("outputs", 3)])).expect("three is legal");
        let outs: Vec<Arc<Probe>> = (0..3)
            .map(|i| probe(&split, &alloc::format!("out{i}")))
            .collect();
        driver(&split, "in").set(Level::High);
        for (i, out) in outs.iter().enumerate() {
            assert!(out.high(), "output {i}");
        }
    }

    #[test]
    fn a_gate_is_combinational_and_a_detector_is_not() {
        let and = Combinator::and(&Props::new()).expect("two by default");
        assert!(
            Device::combinational(&and),
            "§4.3's cycle rule rests on this"
        );
        let edge = Edge::new(&Props::new()).expect("rising by default");
        assert!(
            !Device::combinational(&edge),
            "an edge detector holds a bit, so a cycle through it is a handshake"
        );
    }

    #[test]
    fn an_edge_detector_pulses_on_the_transition_it_was_asked_for() {
        for (edge, rising, falling) in [("rising", 1, 0), ("falling", 0, 1), ("both", 1, 1)] {
            let dev = Edge::new(&Props::new().with("edge", edge)).expect("a legal edge");
            let out = probe(&dev, "out");
            let a = driver(&dev, "in");
            let before = out.edges();
            a.set(Level::High);
            // A pulse is two notifications: up, then back down.
            assert_eq!(
                out.edges() - before,
                rising * 2,
                "{edge}: the rising transition"
            );
            let mid = out.edges();
            a.set(Level::Low);
            assert_eq!(
                out.edges() - mid,
                falling * 2,
                "{edge}: the falling transition"
            );
            assert!(!out.high(), "{edge}: a pulse does not stay high");
        }
    }

    #[test]
    fn a_repeated_level_is_not_an_edge() {
        let dev = Edge::new(&Props::new()).expect("rising by default");
        let out = probe(&dev, "out");
        let a = driver(&dev, "in");
        a.set(Level::High);
        let after = out.edges();
        a.set(Level::High);
        assert_eq!(
            out.edges(),
            after,
            "the same level twice is not a transition"
        );
    }

    #[test]
    fn the_detector_remembers_its_level_across_a_snapshot() {
        let saved = Edge::new(&Props::new()).expect("rising by default");
        let _out = probe(&saved, "out");
        driver(&saved, "in").set(Level::High);
        assert!(saved.input_high());

        let image = |dev: &Edge| {
            let mut shape = MachineShape::new();
            shape.add_device("gate", EDGE_CLASS.name).unwrap();
            let mut w = StateWriter::new(shape);
            {
                let mut chunk = w
                    .chunk("gate", EDGE_CLASS.name, EDGE_CLASS.version)
                    .unwrap();
                Device::save(dev, &mut chunk).unwrap();
            }
            w.to_vec().unwrap()
        };
        let first = image(&saved);

        let restored = Edge::new(&Props::new()).expect("rising by default");
        let out = probe(&restored, "out");
        let a = driver(&restored, "in");
        let reader = StateReader::new(&first).unwrap();
        let chunk = reader
            .load(
                "gate",
                EDGE_CLASS.name,
                EDGE_CLASS.version,
                &Migrations::new(),
            )
            .unwrap();
        Device::load(&restored, &mut chunk.reader()).unwrap();
        assert_eq!(image(&restored), first, "the two images are identical");

        // And the restored detector behaves: the line it thinks is already high
        // is re-announced by the load sweep, and that must not be an edge.
        let before = out.edges();
        a.set(Level::High);
        assert_eq!(
            out.edges(),
            before,
            "a re-announced level turned into a fabricated pulse"
        );
        a.set(Level::Low);
        a.set(Level::High);
        assert!(out.edges() > before, "and a real one still fires");
    }

    #[test]
    fn a_pin_count_out_of_range_is_refused_and_so_is_a_typo() {
        assert!(
            Combinator::and(&props(&[("inputs", 1)])).is_err(),
            "one input"
        );
        assert!(
            Combinator::and(&props(&[("inputs", 17)])).is_err(),
            "too many"
        );
        assert!(Combinator::and(&props(&[("inpts", 2)])).is_err(), "a typo");
        assert!(Combinator::split(&props(&[("outputs", 0)])).is_err());
        assert!(
            Combinator::not(&props(&[("inputs", 2)])).is_err(),
            "no pins to pick"
        );
        assert!(
            Edge::new(&Props::new().with("edge", "sideways")).is_err(),
            "an edge that is neither"
        );
    }

    #[test]
    fn a_pin_a_gate_does_not_have_is_a_configuration_error() {
        let and = Combinator::and(&Props::new()).expect("two by default");
        let id = WireIdAllocator::new().alloc();
        let wire = Wire::builder().source(id).build_shared();
        assert!(
            Device::connect(&and, "in0", WireSource::new(wire, id)).is_err(),
            "an input is not an output"
        );
        assert!(Device::sink(&and, "in2", &[]).is_none(), "only two inputs");
        assert!(Device::sink(&and, "out", &[]).is_none());
        let not = Combinator::not(&Props::new()).expect("no properties");
        assert!(
            Device::sink(&not, "in0", &[]).is_none(),
            "one input, named `in`"
        );
        assert!(Device::sink(&not, "in", &[]).is_some());
    }

    #[test]
    fn every_combinator_has_a_class_a_binding_and_a_schema() {
        use crate::machine::validate::Classes;
        let mut reg = crate::core::registry::Registry::new();
        register(&mut reg).expect("nothing else claims these names");
        let mut b = Bindings::new();
        bind(&mut b).expect("nothing else binds them");
        let mut table = crate::machine::ClassTable::new();
        for schema in schemas() {
            table.insert(schema);
        }
        for name in [
            NOT_CLASS_NAME,
            AND_CLASS_NAME,
            OR_CLASS_NAME,
            SPLIT_CLASS_NAME,
            EDGE_CLASS_NAME,
        ] {
            assert!(table.get(name).is_some(), "{name} has no schema");
        }
        // And `WireCombinators` is what a caller with no registry asks, which
        // was an empty list until these existed — so §4.3's gates were
        // documented and unbuildable at the same time.
        let stock = crate::machine::validate::WireCombinators::new();
        assert!(stock.get(AND_CLASS_NAME).is_some());
        assert_eq!(stock.names().len(), 5);
    }
}
