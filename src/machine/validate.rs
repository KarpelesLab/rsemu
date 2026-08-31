//! The validator: does this machine make sense before anything is built?
//!
//! Fourth stage of §5's pipeline. The [resolver](crate::machine::resolver) has
//! already proved that every name refers to something; this stage asks the
//! questions that need to know *what* the something is:
//!
//! * does this device class exist, and does it take this property?
//! * is `unassigned = open-buss` a spelling of anything?
//! * is `wire ppu.nmi -> cpu.nmi` connecting a pin that exists, in a direction
//!   that makes sense?
//! * does `map cpubus 0xf000 size 0x2000` fit in a 16-bit space?
//! * does the wire graph settle, or has someone built an inverter ring?
//!
//! Everything property-shaped goes through [`core::props`], which already
//! produces the messages `ROADMAP.md` §4.4 asks for — "unknown property `clok`
//! (did you mean `clock`?)" — and this module's job is to attach a span to
//! them, not to write them again.
//!
//! [`core::props`]: crate::core::props
//!
//! # Where the class descriptions come from
//!
//! `core::registry` does not exist yet, so the schema is an input:
//! [`ClassTable`] is built by the caller and passed in. A caller with no table
//! still gets every check that does not need one — spaces, address ranges,
//! wire cycles — and the class-specific ones are skipped rather than guessed
//! at. When the registry lands, it grows a `ClassTable` and nothing here
//! changes.
//!
//! # Combinational versus sequential, and why the cycle rule is what it is
//!
//! §4.3 requires the resolver to reject wire cycles, and in the same breath
//! observes that a real IRQ/acknowledge loop *is* cyclic. Both are true, and
//! the distinction is whether a device forwards a level within the same
//! instant:
//!
//! * A **combinational** device (`wire.not`, `wire.or`, `wire.and`,
//!   `wire.split`) turns its inputs into its outputs with no state. A loop of
//!   these never settles, and the realize sweep — which announces levels in
//!   topological order (§4.3) — has no order to announce them in. That is the
//!   cycle this module rejects, naming every device in it.
//! * A **sequential** device (anything with state: a CPU, an interrupt
//!   controller, `wire.level-to-edge`) breaks the loop. It is the weak edge
//!   §4.3 says every wire cycle must have, and a handshake through one is a
//!   correct machine, not an error.
//!
//! A class nobody described is assumed sequential — a device is a state
//! machine until it says otherwise — except that the `wire.*` combinators §4.3
//! names are known combinational without a table, so an inverter ring is
//! caught in the default configuration.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::core::props::{Props, Value, ValueKind, check_enum, check_range};
use crate::machine::diag::Diagnostic;
use crate::machine::resolver::{MapTarget, ObjectId, PropSpans, Resolved, SpaceId};
use crate::machine::span::Span;

// ---------------------------------------------------------------------------
// Class descriptions
// ---------------------------------------------------------------------------

/// Which way a pin drives (`ROADMAP.md` §4.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PortDir {
    /// A sink: the device is told about levels.
    In,
    /// A source: the device announces levels.
    Out,
    /// Both, as an open-drain handshake line is.
    InOut,
}

impl PortDir {
    /// Whether a wire may start here.
    pub fn can_drive(self) -> bool {
        matches!(self, PortDir::Out | PortDir::InOut)
    }

    /// Whether a wire may end here.
    pub fn can_receive(self) -> bool {
        matches!(self, PortDir::In | PortDir::InOut)
    }

    /// The word an error message uses.
    pub fn as_str(self) -> &'static str {
        match self {
            PortDir::In => "an input",
            PortDir::Out => "an output",
            PortDir::InOut => "bidirectional",
        }
    }
}

/// One pin a device class has.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortSchema {
    /// Its name, as a `wire` statement spells it.
    pub name: String,
    /// Which way it drives.
    pub dir: PortDir,
}

impl PortSchema {
    /// A pin.
    pub fn new(name: impl Into<String>, dir: PortDir) -> PortSchema {
        PortSchema {
            name: name.into(),
            dir,
        }
    }
}

/// One property a device class takes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PropSchema {
    /// Its name.
    pub name: String,
    /// What kind of value it wants.
    pub kind: ValueKind,
    /// Whether it must be given.
    pub required: bool,
    /// The spellings allowed, for a string-valued enumeration. Empty means any.
    pub values: Vec<String>,
    /// An inclusive range, for an integer-valued property.
    pub range: Option<(u64, u64)>,
}

impl PropSchema {
    /// A property of the given kind.
    pub fn new(name: impl Into<String>, kind: ValueKind) -> PropSchema {
        PropSchema {
            name: name.into(),
            kind,
            required: false,
            values: Vec::new(),
            range: None,
        }
    }

    /// The same, but it must be given.
    #[must_use]
    pub fn required(mut self) -> PropSchema {
        self.required = true;
        self
    }

    /// Restrict it to a set of spellings.
    #[must_use]
    pub fn values(mut self, values: &[&str]) -> PropSchema {
        self.values = values.iter().map(|s| (*s).to_string()).collect();
        self
    }

    /// Restrict it to a range, inclusive at both ends.
    #[must_use]
    pub fn range(mut self, lo: u64, hi: u64) -> PropSchema {
        self.range = Some((lo, hi));
        self
    }
}

/// What a device class accepts: properties, pins, regions, and whether it is
/// combinational.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClassSchema {
    /// The registry key, as `object cpu "mos6502"` writes it.
    pub class: String,
    /// The properties it takes.
    pub props: Vec<PropSchema>,
    /// The pins it has.
    pub ports: Vec<PortSchema>,
    /// The regions a `map` may name, `""` for the whole device.
    pub regions: Vec<String>,
    /// Whether it forwards a level within one instant — see the module docs.
    pub combinational: bool,
}

impl ClassSchema {
    /// A class with nothing declared yet.
    pub fn new(class: impl Into<String>) -> ClassSchema {
        ClassSchema {
            class: class.into(),
            props: Vec::new(),
            ports: Vec::new(),
            regions: Vec::new(),
            combinational: false,
        }
    }

    /// Add a property.
    #[must_use]
    pub fn prop(mut self, prop: PropSchema) -> ClassSchema {
        self.props.push(prop);
        self
    }

    /// Add a pin.
    #[must_use]
    pub fn port(mut self, name: impl Into<String>, dir: PortDir) -> ClassSchema {
        self.ports.push(PortSchema::new(name, dir));
        self
    }

    /// Add a mappable region.
    #[must_use]
    pub fn region(mut self, name: impl Into<String>) -> ClassSchema {
        self.regions.push(name.into());
        self
    }

    /// Mark the class combinational: it forwards levels with no state.
    #[must_use]
    pub fn combinational(mut self) -> ClassSchema {
        self.combinational = true;
        self
    }

    /// The pin `name`, if it has one.
    pub fn port_named(&self, name: &str) -> Option<&PortSchema> {
        self.ports.iter().find(|p| p.name == name)
    }
}

/// Where the validator looks a device class up.
///
/// A seam, not an abstraction for its own sake: `core::registry` (§4.4) will
/// implement this, and until it exists a caller can hand over a
/// [`ClassTable`] built by hand.
pub trait Classes {
    /// The description of `class`, if this build has one.
    fn get(&self, class: &str) -> Option<&ClassSchema>;
    /// Every class known, in a deterministic order, for "did you mean".
    fn names(&self) -> Vec<&str>;
}

/// A [`Classes`] built by hand, in insertion order.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ClassTable {
    classes: Vec<ClassSchema>,
}

impl ClassTable {
    /// An empty table: every class-specific check is skipped.
    pub fn new() -> ClassTable {
        ClassTable {
            classes: Vec::new(),
        }
    }

    /// Add a class, replacing any earlier one of the same name.
    pub fn insert(&mut self, schema: ClassSchema) {
        if let Some(slot) = self.classes.iter_mut().find(|c| c.class == schema.class) {
            *slot = schema;
            return;
        }
        self.classes.push(schema);
    }

    /// Builder form of [`ClassTable::insert`].
    #[must_use]
    pub fn with(mut self, schema: ClassSchema) -> ClassTable {
        self.insert(schema);
        self
    }

    /// Whether the table describes nothing.
    pub fn is_empty(&self) -> bool {
        self.classes.is_empty()
    }

    /// How many classes it describes.
    pub fn len(&self) -> usize {
        self.classes.len()
    }
}

impl Classes for ClassTable {
    fn get(&self, class: &str) -> Option<&ClassSchema> {
        self.classes.iter().find(|c| c.class == class)
    }

    fn names(&self) -> Vec<&str> {
        self.classes.iter().map(|c| c.class.as_str()).collect()
    }
}

/// The `wire.*` combinators §4.3 ships, for callers with no registry.
///
/// Only what §4.3 states: which of them are combinational. Their pins are left
/// undeclared because §4.3 does not name them and inventing pin names would
/// make a wrong `wire` statement *pass* while a right one failed.
#[derive(Debug, Clone, Copy, Default)]
pub struct WireCombinators;

impl Classes for WireCombinators {
    fn get(&self, _class: &str) -> Option<&ClassSchema> {
        None
    }

    fn names(&self) -> Vec<&str> {
        Vec::new()
    }
}

/// What the validator insists on beyond the checks that are always run.
#[derive(Debug, Clone, Default)]
pub struct ValidateOptions {
    /// Fail when a device class is not in the table.
    ///
    /// Off by default because an empty table is the normal state until
    /// `core::registry` exists; a CLI with a real registry turns it on, so
    /// that a missing Cargo feature is reported at validate time rather than
    /// at realize time.
    pub require_known_classes: bool,
}

impl ValidateOptions {
    /// Default options.
    pub fn new() -> ValidateOptions {
        ValidateOptions::default()
    }

    /// Require every `object`'s class to be in the table.
    #[must_use]
    pub fn requiring_known_classes(mut self) -> ValidateOptions {
        self.require_known_classes = true;
        self
    }
}

// ---------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------

/// Check a resolved machine, returning the first thing wrong with it.
///
/// One diagnostic, like the parser: a validator that lists ten problems has
/// usually found one problem and nine consequences.
///
/// ```
/// use rsemu::machine::resolver::{ResolveOptions, resolve};
/// use rsemu::machine::sources::{NoIncludes, SourceMap};
/// use rsemu::machine::validate::{ClassTable, ValidateOptions, validate};
///
/// let mut map = SourceMap::new();
/// let root = map.add(
///     "m.machine",
///     "machine \"m\" {\n  space s { width = 16 }\n  object r \"ram\" { }\n  \
///      map s 0xf000 size 0x2000 = r\n}\n",
/// )
/// .expect("fits");
/// let machine = resolve(&mut map, root, &mut NoIncludes, &ResolveOptions::new())
///     .map_err(|d| map.to_error(&d))?;
///
/// // 0xf000 + 0x2000 does not fit in 16 bits.
/// let err = validate(&machine, &ClassTable::new(), &ValidateOptions::new())
///     .expect_err("out of range");
/// assert!(map.render(&err).contains("past the end"));
/// # Ok::<(), rsemu::Error>(())
/// ```
pub fn validate(
    machine: &Resolved,
    classes: &impl Classes,
    options: &ValidateOptions,
) -> Result<(), Diagnostic> {
    for space in &machine.spaces {
        check_props(
            &space.props,
            &space.prop_spans,
            &space_schema(),
            space.name_span,
            &format!("address space `{}`", space.name),
        )?;
    }

    for object in &machine.objects {
        match classes.get(&object.class) {
            Some(schema) => check_props(
                &object.props,
                &object.prop_spans,
                schema,
                object.name_span,
                &format!("object `{}`", object.name),
            )?,
            None if options.require_known_classes => {
                let known = names(classes.names().into_iter());
                return Err(Diagnostic::new(
                    object.class_span,
                    format!(
                        "unknown device class `{}` (is its feature enabled?); this build has \
                         {known}",
                        object.class
                    ),
                ));
            }
            None => {}
        }
    }

    for map in &machine.maps {
        check_mapping(machine, map, classes)?;
    }

    for wire in &machine.wires {
        check_wire(machine, wire, classes)?;
    }

    realize_order(machine, classes)?;
    Ok(())
}

/// The order the realize sweep should announce wire levels in (§4.3).
///
/// A wire whose source is itself driven through combinational logic comes
/// after the wires that drive it, so that a freshly realized or freshly
/// restored machine is consistent by the time the sweep ends. Returns the
/// indices into [`Resolved::wires`].
///
/// This is also where combinational wire cycles are rejected: an order exists
/// exactly when there is no such cycle, so the check and the answer are the
/// same computation.
pub fn realize_order(machine: &Resolved, classes: &impl Classes) -> Result<Vec<usize>, Diagnostic> {
    let count = machine.objects.len();
    let conducts: Vec<bool> = machine
        .objects
        .iter()
        .map(|o| is_combinational(&o.class, classes))
        .collect();

    // Edge a → b for every wire whose *destination* forwards levels onwards.
    let mut edges: Vec<Vec<(usize, usize)>> = alloc::vec![Vec::new(); count];
    for (i, wire) in machine.wires.iter().enumerate() {
        let (from, to) = (index(wire.from.object), index(wire.to.object));
        if from < count && to < count && conducts[to] {
            edges[from].push((to, i));
        }
    }

    // Iterative depth-first search: a machine file can nest deeply enough that
    // recursion here would be a stack overflow on somebody's input.
    let mut state = alloc::vec![0u8; count];
    let mut rank = alloc::vec![0usize; count];
    for start in 0..count {
        if state[start] != 0 {
            continue;
        }
        let mut stack: Vec<(usize, usize)> = alloc::vec![(start, 0)];
        state[start] = 1;
        while let Some((node, next)) = stack.pop() {
            if let Some((to, wire)) = edges[node].get(next).copied() {
                stack.push((node, next + 1));
                match state[to] {
                    1 => {
                        return Err(cycle_diagnostic(machine, &stack, node, to, wire));
                    }
                    0 => {
                        state[to] = 1;
                        stack.push((to, 0));
                    }
                    _ => {
                        rank[node] = rank[node].max(rank[to] + 1);
                    }
                }
            } else {
                state[node] = 2;
                for (to, _) in &edges[node] {
                    rank[node] = rank[node].max(rank[*to] + 1);
                }
            }
        }
    }

    // Deepest first: a source announces before whatever it feeds. Ties keep
    // file order, so the sweep is reproducible.
    let mut order: Vec<usize> = (0..machine.wires.len()).collect();
    order.sort_by_key(|i| {
        let from = index(machine.wires[*i].from.object);
        (core::cmp::Reverse(rank.get(from).copied().unwrap_or(0)), *i)
    });
    Ok(order)
}

/// Whether a class forwards a level with no state of its own.
fn is_combinational(class: &str, classes: &impl Classes) -> bool {
    if let Some(schema) = classes.get(class) {
        return schema.combinational;
    }
    // §4.3's standard combinators, minus the edge detector, which is a state
    // element precisely so that it snapshots correctly.
    matches!(class, "wire.split" | "wire.or" | "wire.and" | "wire.not")
}

/// "`a` → `b` → `a`", naming every device in the loop.
fn cycle_diagnostic(
    machine: &Resolved,
    stack: &[(usize, usize)],
    from: usize,
    to: usize,
    wire: usize,
) -> Diagnostic {
    let mut path: Vec<usize> = stack.iter().map(|(n, _)| *n).collect();
    if path.last() != Some(&from) {
        path.push(from);
    }
    let start = path.iter().position(|n| *n == to).unwrap_or(0);
    let mut names = String::new();
    for node in path.iter().skip(start) {
        names.push_str(&format!("`{}` → ", machine.objects[*node].name));
    }
    names.push_str(&format!("`{}`", machine.objects[to].name));
    Diagnostic::new(
        machine.wires[wire].span,
        format!(
            "wire cycle through combinational devices: {names}; one device in a wire loop must \
             hold state, or the realize sweep has no order to announce levels in"
        ),
    )
}

fn index(id: ObjectId) -> usize {
    id.0 as usize
}

/// A mapping's window must fit its space, and name a region that exists.
fn check_mapping(
    machine: &Resolved,
    map: &crate::machine::resolver::Mapping,
    classes: &impl Classes,
) -> Result<(), Diagnostic> {
    if let Some(bits) = space_bits(machine, map.space) {
        let limit = if bits >= 64 {
            u128::from(u64::MAX) + 1
        } else {
            1u128 << bits
        };
        let end = u128::from(map.base) + u128::from(map.size);
        if end > limit {
            let name = machine
                .space(map.space)
                .map_or("", |s| s.name.as_str())
                .to_string();
            return Err(Diagnostic::new(
                map.base_span.join(map.size_span),
                format!(
                    "this mapping ends at {end:#x}, past the end of `{name}`, which is {bits} \
                     bits wide"
                ),
            ));
        }
    }
    check_region(machine, &map.target, classes)
}

fn check_region(
    machine: &Resolved,
    target: &MapTarget,
    classes: &impl Classes,
) -> Result<(), Diagnostic> {
    match target {
        MapTarget::Region {
            object,
            region,
            span,
        } => {
            let Some(obj) = machine.object(*object) else {
                return Ok(());
            };
            let Some(schema) = classes.get(&obj.class) else {
                return Ok(());
            };
            let Some(region) = region else {
                return Ok(());
            };
            if !schema.regions.iter().any(|r| r == region) {
                let known = names(schema.regions.iter().map(String::as_str));
                return Err(Diagnostic::new(
                    *span,
                    format!(
                        "`{}` has no region `{region}`; `{}` provides {known}",
                        obj.name, obj.class
                    ),
                ));
            }
            Ok(())
        }
        MapTarget::Mirror { inner, .. } | MapTarget::Alias { inner, .. } => {
            check_region(machine, inner, classes)
        }
        MapTarget::Split { reads, writes, .. } => {
            check_region(machine, reads, classes)?;
            check_region(machine, writes, classes)
        }
    }
}

/// A wire's endpoints must be pins, pointing the right way.
fn check_wire(
    machine: &Resolved,
    wire: &crate::machine::resolver::Wire,
    classes: &impl Classes,
) -> Result<(), Diagnostic> {
    check_pin(machine, &wire.from, classes, true)?;
    check_pin(machine, &wire.to, classes, false)
}

fn check_pin(
    machine: &Resolved,
    pin: &crate::machine::resolver::Pin,
    classes: &impl Classes,
    driving: bool,
) -> Result<(), Diagnostic> {
    let Some(object) = machine.object(pin.object) else {
        return Ok(());
    };
    let Some(schema) = classes.get(&object.class) else {
        return Ok(());
    };
    let Some(port) = schema.port_named(&pin.port) else {
        let known = names(schema.ports.iter().map(|p| p.name.as_str()));
        return Err(Diagnostic::new(
            pin.span,
            format!(
                "`{}` has no pin `{}`; `{}` has {known}",
                object.name, pin.port, object.class
            ),
        ));
    };
    let ok = if driving {
        port.dir.can_drive()
    } else {
        port.dir.can_receive()
    };
    if !ok {
        return Err(Diagnostic::new(
            pin.span,
            format!(
                "`{}.{}` is {} and cannot be a wire's {}",
                object.name,
                pin.port,
                port.dir.as_str(),
                if driving { "source" } else { "destination" }
            ),
        ));
    }
    Ok(())
}

/// A space's address width, when it declared one.
fn space_bits(machine: &Resolved, id: SpaceId) -> Option<u32> {
    let space = machine.space(id)?;
    let width = space.props.get("width")?.as_uint()?;
    u32::try_from(width).ok()
}

/// Check one property bag against a schema, using `core::props` for the
/// message and this module only for the caret.
fn check_props(
    props: &Props,
    spans: &PropSpans,
    schema: &ClassSchema,
    fallback: Span,
    what: &str,
) -> Result<(), Diagnostic> {
    let allowed: Vec<&str> = schema.props.iter().map(|p| p.name.as_str()).collect();
    for (name, value) in props.iter() {
        if allowed.contains(&name) {
            continue;
        }
        // One entry at a time so the caret lands on the offending property;
        // the message, including "did you mean", is `core::props`'s.
        let one = Props::new().with(name, value.clone());
        if let Err(e) = one.check_known(&allowed) {
            return Err(Diagnostic::new(
                spans.get_or(name, fallback),
                format!("{what}: {e}"),
            ));
        }
    }
    for prop in &schema.props {
        let Some(value) = props.get(&prop.name) else {
            if prop.required {
                return Err(Diagnostic::new(
                    fallback,
                    format!("{what}: missing required property `{}`", prop.name),
                ));
            }
            continue;
        };
        let span = spans.get_or(&prop.name, fallback);
        check_kind(value, prop.kind, &prop.name).map_err(|m| Diagnostic::new(span, m))?;
        if !prop.values.is_empty() {
            let allowed: Vec<&str> = prop.values.iter().map(String::as_str).collect();
            let text = value.as_str().unwrap_or_default();
            check_enum(&prop.name, text, &allowed)
                .map_err(|e| Diagnostic::new(span, e.to_string()))?;
        }
        if let Some((lo, hi)) = prop.range
            && let Some(n) = value.as_uint()
        {
            check_range(&prop.name, n, lo..=hi)
                .map_err(|e| Diagnostic::new(span, e.to_string()))?;
        }
    }
    Ok(())
}

/// Type-check one value, deferring the wording to `core::props`.
fn check_kind(value: &Value, kind: ValueKind, name: &str) -> Result<(), String> {
    let outcome = match kind {
        ValueKind::Bool => value.to_bool(name).map(|_| ()),
        ValueKind::Int => value.to_int(name).map(|_| ()),
        ValueKind::Uint => value.to_uint(name).map(|_| ()),
        ValueKind::Size => value.to_size(name).map(|_| ()),
        ValueKind::Addr => value.to_addr(name).map(|_| ()),
        ValueKind::Duration => value.to_duration(name).map(|_| ()),
        ValueKind::Str => value.to_str(name).map(|_| ()),
        ValueKind::List => value.to_list(name).map(|_| ()),
        ValueKind::Map => value.to_map(name).map(|_| ()),
        ValueKind::Link => value.to_link(name).map(|_| ()),
        // A media property is written as the *name* of a slot, and realize
        // substitutes the bound bytes long after this runs — so a string is
        // exactly what a well-formed file has here.
        ValueKind::Media => match value {
            Value::Str(_) | Value::Media(_) => Ok(()),
            other => Err(crate::core::Error::Property(format!(
                "property `{name}`: expected the name of a media slot, found {} {other}",
                other.kind()
            ))),
        },
    };
    outcome.map_err(|e| e.to_string())
}

/// The properties every `space` takes (`ROADMAP.md` §4.1).
///
/// Built in rather than registered, because an address space is part of the
/// core, not a device class. `open-bus` is accepted because §5's example
/// writes it; it is a synonym for reads-as-ones until `core::space` grows a
/// last-value-on-the-bus policy.
fn space_schema() -> ClassSchema {
    ClassSchema::new("space")
        .prop(
            PropSchema::new("width", ValueKind::Uint)
                .required()
                .range(1, 64),
        )
        .prop(PropSchema::new("unassigned", ValueKind::Str).values(&[
            "fault",
            "open-bus",
            "read-as-ones",
            "read-as-zeros",
        ]))
        .prop(PropSchema::new("endian", ValueKind::Str).values(&["little", "big"]))
        .prop(PropSchema::new("log-unassigned", ValueKind::Bool))
}

/// `` `a`, `b` ``, or `none`.
fn names<'i>(iter: impl Iterator<Item = &'i str>) -> String {
    let mut out = String::new();
    let mut count = 0usize;
    for name in iter {
        if count == 8 {
            out.push_str(", …");
            break;
        }
        if count != 0 {
            out.push_str(", ");
        }
        out.push_str(&format!("`{name}`"));
        count += 1;
    }
    if count == 0 {
        out.push_str("none");
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::machine::resolver::{ResolveOptions, resolve};
    use crate::machine::sources::{NoIncludes, SourceMap};

    /// Resolve, validate, and render whatever comes out.
    fn check(text: &str, classes: &ClassTable, options: &ValidateOptions) -> Result<(), String> {
        let mut map = SourceMap::new();
        let root = map.add("m.machine", text).expect("fits");
        let machine = match resolve(&mut map, root, &mut NoIncludes, &ResolveOptions::new()) {
            Ok(m) => m,
            Err(d) => panic!("resolve failed: {}", map.render(&d)),
        };
        validate(&machine, classes, options).map_err(|d| map.render(&d))
    }

    fn error(text: &str, classes: &ClassTable) -> String {
        check(text, classes, &ValidateOptions::new()).expect_err("should fail")
    }

    fn cpu_class() -> ClassSchema {
        ClassSchema::new("mos6502")
            .prop(PropSchema::new("engine", ValueKind::Str).values(&["interp", "jit"]))
            .port("nmi", PortDir::In)
            .port("irq", PortDir::In)
            .port("sync", PortDir::Out)
            .region("regs")
    }

    #[test]
    fn the_nes_shape_validates() {
        let classes = ClassTable::new()
            .with(cpu_class())
            .with(ClassSchema::new("ram").prop(PropSchema::new("size", ValueKind::Size).required()))
            .with(
                ClassSchema::new("nes.ppu")
                    .port("nmi", PortDir::Out)
                    .region("regs"),
            );
        check(
            "machine \"nes\" {\n  \
               osc master = 236250000/11 Hz\n  \
               space cpubus { width = 16, unassigned = open-bus }\n  \
               object wram \"ram\" { size = 2K }\n  \
               object cpu \"mos6502\" { clock = master / 12, space = cpubus, engine = \"interp\" }\n  \
               object ppu \"nes.ppu\" { clock = master / 4 }\n  \
               map cpubus 0x0000 size 0x2000 = mirror(wram)\n  \
               map cpubus 0x2000 size 0x2000 = mirror(ppu.regs)\n  \
               wire ppu.nmi -> cpu.nmi\n\
             }\n",
            &classes,
            &ValidateOptions::new().requiring_known_classes(),
        )
        .expect("valid");
    }

    #[test]
    fn golden_unknown_property_reuses_the_property_systems_message() {
        assert_eq!(
            error(
                "machine \"m\" {\n  \
                   osc master = 1 MHz\n  \
                   object cpu \"mos6502\" { clock = master, engnie = \"interp\" }\n\
                 }\n",
                &ClassTable::new().with(cpu_class())
            ),
            "\
error: object `cpu`: unknown property `engnie` (did you mean `engine`?); known properties: `engine`
 --> m.machine:3:42
  |
3 |   object cpu \"mos6502\" { clock = master, engnie = \"interp\" }
  |                                          ^^^^^^"
        );
    }

    #[test]
    fn golden_a_property_whose_value_is_not_one_of_the_allowed_spellings() {
        assert_eq!(
            error(
                "machine \"m\" {\n  object cpu \"mos6502\" { engine = \"intrep\" }\n}\n",
                &ClassTable::new().with(cpu_class())
            ),
            "\
error: property `engine`: expected one of `interp`, `jit`; found \"intrep\" (did you mean `interp`?)
 --> m.machine:2:26
  |
2 |   object cpu \"mos6502\" { engine = \"intrep\" }
  |                          ^^^^^^"
        );
    }

    #[test]
    fn golden_a_space_property_that_is_out_of_range() {
        assert_eq!(
            error(
                "machine \"m\" {\n  space s { width = 65 }\n}\n",
                &ClassTable::new()
            ),
            "\
error: property `width`: 65 is out of range 1..=64
 --> m.machine:2:13
  |
2 |   space s { width = 65 }
  |             ^^^^^"
        );
    }

    #[test]
    fn golden_a_misspelled_unassigned_policy() {
        assert_eq!(
            error(
                "machine \"m\" {\n  space s { width = 16, unassigned = open-buss }\n}\n",
                &ClassTable::new()
            ),
            "\
error: property `unassigned`: expected one of `fault`, `open-bus`, `read-as-ones`, `read-as-zeros`; found \"open-buss\" (did you mean `open-bus`?)
 --> m.machine:2:25
  |
2 |   space s { width = 16, unassigned = open-buss }
  |                         ^^^^^^^^^^"
        );
    }

    #[test]
    fn golden_a_space_with_no_width() {
        assert_eq!(
            error("machine \"m\" {\n  space s { }\n}\n", &ClassTable::new()),
            "\
error: address space `s`: missing required property `width`
 --> m.machine:2:9
  |
2 |   space s { }
  |         ^"
        );
    }

    #[test]
    fn golden_a_mapping_that_does_not_fit_its_space() {
        assert_eq!(
            error(
                "machine \"m\" {\n  \
                   space s { width = 16 }\n  \
                   object r \"ram\" { }\n  \
                   map s 0xf000 size 0x2000 = r\n\
                 }\n",
                &ClassTable::new()
            ),
            "\
error: this mapping ends at 0x11000, past the end of `s`, which is 16 bits wide
 --> m.machine:4:9
  |
4 |   map s 0xf000 size 0x2000 = r
  |         ^^^^^^^^^^^^^^^^^^"
        );
    }

    #[test]
    fn golden_a_wire_endpoint_that_is_not_a_pin() {
        assert_eq!(
            error(
                "machine \"m\" {\n  \
                   object cpu \"mos6502\" { }\n  \
                   object ppu \"nes.ppu\" { }\n  \
                   wire ppu.nmi -> cpu.reset\n\
                 }\n",
                &ClassTable::new()
                    .with(cpu_class())
                    .with(ClassSchema::new("nes.ppu").port("nmi", PortDir::Out))
            ),
            "\
error: `cpu` has no pin `reset`; `mos6502` has `nmi`, `irq`, `sync`
 --> m.machine:4:19
  |
4 |   wire ppu.nmi -> cpu.reset
  |                   ^^^^^^^^^"
        );
    }

    #[test]
    fn golden_a_wire_driven_from_an_input() {
        assert_eq!(
            error(
                "machine \"m\" {\n  \
                   object a \"mos6502\" { }\n  \
                   object b \"mos6502\" { }\n  \
                   wire a.nmi -> b.irq\n\
                 }\n",
                &ClassTable::new().with(cpu_class())
            ),
            "\
error: `a.nmi` is an input and cannot be a wire's source
 --> m.machine:4:8
  |
4 |   wire a.nmi -> b.irq
  |        ^^^^^"
        );
    }

    #[test]
    fn golden_a_map_target_region_that_does_not_exist() {
        assert_eq!(
            error(
                "machine \"m\" {\n  \
                   space s { width = 16 }\n  \
                   object cpu \"mos6502\" { }\n  \
                   map s 0 size 0x20 = cpu.rgs\n\
                 }\n",
                &ClassTable::new().with(cpu_class())
            ),
            "\
error: `cpu` has no region `rgs`; `mos6502` provides `regs`
 --> m.machine:4:23
  |
4 |   map s 0 size 0x20 = cpu.rgs
  |                       ^^^^^^^"
        );
    }

    #[test]
    fn golden_an_unknown_device_class() {
        let classes = ClassTable::new().with(cpu_class());
        let rendered = check(
            "machine \"m\" {\n  object cpu \"mos6503\" { }\n}\n",
            &classes,
            &ValidateOptions::new().requiring_known_classes(),
        )
        .expect_err("unknown");
        assert_eq!(
            rendered,
            "\
error: unknown device class `mos6503` (is its feature enabled?); this build has `mos6502`
 --> m.machine:2:14
  |
2 |   object cpu \"mos6503\" { }
  |              ^^^^^^^^^"
        );
    }

    // -- wire cycles --------------------------------------------------------

    #[test]
    fn golden_a_combinational_wire_cycle_names_every_device_in_it() {
        assert_eq!(
            error(
                "machine \"m\" {\n  \
                   object n1 \"wire.not\" { }\n  \
                   object n2 \"wire.not\" { }\n  \
                   wire n1.out -> n2.in\n  \
                   wire n2.out -> n1.in\n\
                 }\n",
                &ClassTable::new()
            ),
            "\
error: wire cycle through combinational devices: `n1` → `n2` → `n1`; one device in a wire loop must hold state, or the realize sweep has no order to announce levels in
 --> m.machine:5:3
  |
5 |   wire n2.out -> n1.in
  |   ^^^^^^^^^^^^^^^^^^^^"
        );
    }

    #[test]
    fn a_loop_through_a_stateful_device_is_a_handshake_not_an_error() {
        // The IRQ/acknowledge loop §4.3 calls out: legal, because the CPU
        // holds state and is therefore the weak edge.
        check(
            "machine \"m\" {\n  \
               object cpu \"mos6502\" { }\n  \
               object pic \"i8259\" { }\n  \
               wire pic.out -> cpu.irq\n  \
               wire cpu.sync -> pic.ack\n\
             }\n",
            &ClassTable::new(),
            &ValidateOptions::new(),
        )
        .expect("a handshake is fine");
    }

    #[test]
    fn the_realize_sweep_announces_sources_before_what_they_drive() {
        let mut map = SourceMap::new();
        let root = map
            .add(
                "m.machine",
                "machine \"m\" {\n  \
                   object src \"gpio\" { }\n  \
                   object inv \"wire.not\" { }\n  \
                   object cpu \"mos6502\" { }\n  \
                   wire inv.out -> cpu.irq\n  \
                   wire src.out -> inv.in\n\
                 }\n",
            )
            .expect("fits");
        let machine = resolve(&mut map, root, &mut NoIncludes, &ResolveOptions::new())
            .unwrap_or_else(|d| panic!("{}", map.render(&d)));
        let order = realize_order(&machine, &ClassTable::new()).expect("acyclic");
        // `src -> inv` must be announced before `inv -> cpu`, whatever order
        // the file wrote them in.
        assert_eq!(order, alloc::vec![1, 0]);
    }

    #[test]
    fn a_class_may_declare_itself_combinational() {
        let classes = ClassTable::new().with(
            ClassSchema::new("my.gate")
                .combinational()
                .port("in", PortDir::In)
                .port("out", PortDir::Out),
        );
        let rendered = error(
            "machine \"m\" {\n  \
               object g1 \"my.gate\" { }\n  \
               object g2 \"my.gate\" { }\n  \
               wire g1.out -> g2.in\n  \
               wire g2.out -> g1.in\n\
             }\n",
            &classes,
        );
        assert!(rendered.contains("wire cycle"), "{rendered}");
    }

    #[test]
    fn an_empty_table_skips_the_checks_that_need_one() {
        check(
            "machine \"m\" {\n  \
               object cpu \"whatever\" { anything = 1 }\n  \
               object ppu \"whatever\" { }\n  \
               wire ppu.nmi -> cpu.nmi\n\
             }\n",
            &ClassTable::new(),
            &ValidateOptions::new(),
        )
        .expect("nothing to check against");
    }

    #[test]
    fn ports_answer_which_way_they_drive() {
        assert!(PortDir::InOut.can_drive() && PortDir::InOut.can_receive());
        assert!(!PortDir::In.can_drive());
        assert!(!PortDir::Out.can_receive());
        assert_eq!(PortDir::Out.as_str(), "an output");
    }

    #[test]
    fn a_table_replaces_rather_than_duplicates() {
        let table = ClassTable::new()
            .with(ClassSchema::new("a"))
            .with(ClassSchema::new("a").combinational());
        assert_eq!(table.len(), 1);
        assert!(table.get("a").expect("present").combinational);
        assert!(!ClassTable::new().with(ClassSchema::new("a")).is_empty());
        assert_eq!(WireCombinators.names(), Vec::<&str>::new());
        assert!(WireCombinators.get("wire.not").is_none());
    }
}
