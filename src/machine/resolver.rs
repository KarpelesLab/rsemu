//! The resolver: an AST becomes a machine a realizer can build.
//!
//! Third stage of §5's pipeline — `lex → parse → **resolve** → validate →
//! realize → run`. It does the five things §5 asks a machine description to
//! support, in this order:
//!
//! 1. **`include`** — load, parse, splice, with cycle detection.
//! 2. **`param`** — defaults, overridden from outside (`-p ram=8G`).
//! 3. **`template` / `for`** — instantiate and unroll, substituting parameters
//!    into expressions and into `cpu$i`-style names.
//! 4. **Declaration tables** — one namespace for oscillators, spaces and
//!    objects, with duplicates rejected.
//! 5. **Links** — `space = cpubus` and `wire ppu.nmi -> cpu.nmi` become
//!    [`SpaceId`]s and [`Pin`]s pointing at real declarations.
//!
//! What comes out is [`Resolved`]: no names left to look up, no expressions
//! left to evaluate, and a [`Span`] on everything so [`validate`] and the
//! realizer can still say `file:line:col` (`ROADMAP.md` §5).
//!
//! [`validate`]: mod@crate::machine::validate
//!
//! # The rules §5 leaves open
//!
//! §5 shows a language; it does not define one. Where the example under-
//! determines the semantics, this module picks, and each choice is written
//! down here because the next person to read a `.machine` file needs to know
//! them.
//!
//! ## A bare identifier is a link, a keyword, or a parameter — decided here
//!
//! `space = cpubus` and `unassigned = open-bus` are the same token shape. The
//! parser deliberately produces the same neutral [`Expr::Path`] for both, and
//! the resolver decides, in this order:
//!
//! 1. **A binding wins.** Loop variables, template arguments and `param`s are
//!    substituted *before* anything else looks at the name — they are the only
//!    names with a lexical scope, so an inner one must shadow.
//! 2. **A declared name is a link.** If the identifier names a declared `osc`,
//!    `space` or `object` — searching the enclosing template instances
//!    outwards — it becomes a [`Value::Link`].
//! 3. **Anything else is a string.** `open-bus` names no declaration, so it is
//!    `Value::Str("open-bus")`, and whether that is a legal spelling is the
//!    validator's question, answered against the property's own enumeration.
//!
//! The interesting case is 1 versus 2. A file where `ram` means the parameter
//! on one line and the RAM chip on the next is a file nobody can read, so
//! rather than silently preferring either, a name declared as *both* a `param`
//! and an `osc`/`space`/`object` is an **ambiguous** diagnostic naming both
//! declarations. Rename one. (Template arguments and loop variables do shadow,
//! because they are the one construct whose whole purpose is to be local.)
//!
//! A dotted name (`ppu.regs`) is always a link — no parameter can be dotted.
//!
//! ## Everything else this module had to decide
//!
//! * **`clock` and `space` on an object are structural**, not device
//!   properties. They name a clock domain (§4.2) and an address space (§4.1),
//!   both of which the *machine* owns, so they are lifted out of the property
//!   bag and resolved here; a device never sees them.
//! * **A clock is a scaled reference to one oscillator**, as in
//!   `master / 12`. §4.2 is explicit that a domain's root is a declared
//!   crystal, so a bare frequency (`clock = 5 MHz`) is refused with a message
//!   saying to declare an `osc`. The scale is exact rational arithmetic and is
//!   stored as `mul`/`div`, never as hertz — that is the whole point of §4.2.
//! * **Template instances namespace with `.`** — `instance core0 =
//!   cpu_complex(…)` declaring `cpu` gives `core0.cpu`. §4.4's device
//!   composition already reads `parent.child`, and a reference resolves by
//!   trying the innermost namespace first, so a template body naming `plic`
//!   finds its own `plic` if it has one and the outer one otherwise.
//! * **`include` is include-once.** Two files pulling in `pci-common.machine`
//!   splice it once, keyed on the canonical name the loader returns. The
//!   alternative is a duplicate-declaration error for the ordinary case, which
//!   would force include guards on a language that has no `#ifdef`.
//! * **File-scope statements are the machine's prelude.** An `include`d
//!   fragment is a list of `template`s and objects (§5); everything at file
//!   scope other than the `machine` block itself is spliced in ahead of the
//!   machine's own body, in file order.
//! * **`param` is machine-wide.** One namespace, whichever file declared it,
//!   because `-p ram=8G` names one knob and a scoped parameter would make that
//!   ambiguous. `param` and `template` are declarations, not statements, so
//!   writing either inside a `for` or a `template` body is an error rather
//!   than a repetition.
//! * **Fan-in is not rewritten.** §4.3 offers two ways to make wired-OR work
//!   and says rsemu takes the first: a sink tracks which sources assert, which
//!   [`core::wire::FanIn`] implements. So two `wire`s naming one destination
//!   resolve to two edges on one pin, and no combiner is synthesised.
//!
//! [`core::wire::FanIn`]: crate::core::wire::FanIn
//!
//! # Bounds
//!
//! Every recursion and every repetition is capped by [`ResolveOptions`], and
//! all arithmetic is checked: a machine file is untrusted input, so this
//! module returns diagnostics where it might otherwise recurse forever,
//! allocate a terabyte, or panic.

use alloc::borrow::ToOwned;
use alloc::boxed::Box;
use alloc::collections::{BTreeMap, BTreeSet};
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::core::props::{Duration, Link, Props, Value};
use crate::machine::ast::{
    BinOp, Expr, ForStmt, InstanceStmt, MapStmt, Name, NamePart, ObjectDecl, OscDecl, ParamDecl,
    Path, Property, SpaceDecl, Stmt, TemplateDecl, UnOp, WireStmt,
};
use crate::machine::diag::Diagnostic;
use crate::machine::lexer::{NumLit, NumUnit, Radix};
use crate::machine::rational::Rational;
use crate::machine::sources::{FileId, IncludeLoader, SourceMap};
use crate::machine::span::{Span, Spanned};

// ---------------------------------------------------------------------------
// Options
// ---------------------------------------------------------------------------

/// Knobs and limits for one resolution.
///
/// The limits are not tuning: they are what makes `for i in 0..2**40` a
/// diagnostic instead of an out-of-memory kill. Raise them for a genuinely
/// enormous machine, but raise them deliberately.
#[derive(Debug, Clone)]
pub struct ResolveOptions {
    /// Parameter overrides, as `-p name=text` pairs, in command-line order.
    ///
    /// The text is parsed with
    /// [`Value::parse_scalar`](crate::core::props::Value::parse_scalar), which
    /// is the same guesser the CLI would have to use anyway. A name that no
    /// `param` declares is an error, not a silent no-op.
    pub params: Vec<(String, String)>,
    /// Which `machine` block to resolve, when a file declares several.
    pub machine: Option<String>,
    /// How deep `include`, `template` and `for` may nest, together.
    pub max_depth: u32,
    /// How many statements the expansion may produce in total.
    pub max_statements: u32,
    /// How many times one `for` may repeat.
    pub max_iterations: u64,
    /// How many expression nodes substitution may produce in total.
    ///
    /// The limit a template bomb hits: substituting a parameter splices the
    /// argument's whole expression, so `t(a = a + a)` nested *n* deep is 2ⁿ
    /// nodes even though the file is *n* lines long.
    pub max_expr_nodes: u32,
}

impl Default for ResolveOptions {
    fn default() -> Self {
        ResolveOptions {
            params: Vec::new(),
            machine: None,
            max_depth: 32,
            max_statements: 65_536,
            max_iterations: 4_096,
            max_expr_nodes: 1_000_000,
        }
    }
}

impl ResolveOptions {
    /// Default options.
    pub fn new() -> ResolveOptions {
        ResolveOptions::default()
    }

    /// Add a parameter override, as `-p name=value` would.
    #[must_use]
    pub fn with_param(mut self, name: impl Into<String>, value: impl Into<String>) -> Self {
        self.params.push((name.into(), value.into()));
        self
    }

    /// Choose which `machine` block to resolve.
    #[must_use]
    pub fn with_machine(mut self, name: impl Into<String>) -> Self {
        self.machine = Some(name.into());
        self
    }
}

// ---------------------------------------------------------------------------
// The resolved machine
// ---------------------------------------------------------------------------

/// An oscillator's index in [`Resolved::oscillators`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct OscId(pub u32);

/// An address space's index in [`Resolved::spaces`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SpaceId(pub u32);

/// An object's index in [`Resolved::objects`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ObjectId(pub u32);

/// Where each property was written, kept beside the [`Props`] it describes.
///
/// [`Props`] is the device-facing bag and deliberately holds no spans; the
/// validator still has to put a caret under `clok = master / 12`. Ordered like
/// the properties themselves, so iteration is deterministic.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct PropSpans {
    entries: Vec<(String, Span)>,
}

impl PropSpans {
    /// An empty set.
    pub fn new() -> PropSpans {
        PropSpans {
            entries: Vec::new(),
        }
    }

    /// Record where `name` was written, replacing any earlier record.
    pub fn insert(&mut self, name: impl Into<String>, span: Span) {
        let name = name.into();
        for entry in &mut self.entries {
            if entry.0 == name {
                entry.1 = span;
                return;
            }
        }
        self.entries.push((name, span));
    }

    /// Where `name` was written.
    pub fn get(&self, name: &str) -> Option<Span> {
        self.entries
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, s)| *s)
    }

    /// Where `name` was written, or `fallback` when it was not written at all
    /// — a default has no span of its own.
    pub fn get_or(&self, name: &str, fallback: Span) -> Span {
        self.get(name).unwrap_or(fallback)
    }

    /// The properties and their spans, in order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, Span)> {
        self.entries.iter().map(|(n, s)| (n.as_str(), *s))
    }
}

/// A declared crystal (`ROADMAP.md` §4.2): the root of one clock domain tree.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Oscillator {
    /// Its fully qualified name.
    pub name: String,
    /// Its frequency in hertz, exactly — `236250000/11` is not an integer and
    /// is never rounded.
    pub hz: Rational,
    /// Where the name was written.
    pub name_span: Span,
    /// The whole declaration.
    pub span: Span,
}

/// A declared address space (`ROADMAP.md` §4.1).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Space {
    /// Its fully qualified name.
    pub name: String,
    /// Its properties, evaluated.
    pub props: Props,
    /// Where each property was written.
    pub prop_spans: PropSpans,
    /// Where the name was written.
    pub name_span: Span,
    /// The whole declaration.
    pub span: Span,
}

/// Which clock domain another domain hangs from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockParent {
    /// A declared crystal.
    Osc(OscId),
    /// Another object's domain, so that `cpu / 2` tracks the CPU exactly.
    Object(ObjectId),
}

/// An object's clock domain: a parent and an exact rational scale.
///
/// `mul`/`div` rather than a frequency, because §4.2's exactness is a property
/// of the *ratio*: `master / 12` and `master / 4` give exactly 3 PPU dots per
/// CPU cycle whatever the crystal turns out to be.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Clock {
    /// What this domain divides.
    pub parent: ClockParent,
    /// Numerator of the scale, in lowest terms.
    pub mul: u64,
    /// Denominator of the scale, in lowest terms; never zero.
    pub div: u64,
    /// Where the expression was written.
    pub span: Span,
}

/// A declared device (`ROADMAP.md` §4.4).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Object {
    /// Its fully qualified name, template prefix included.
    pub name: String,
    /// Its registry class, as written in quotes.
    pub class: String,
    /// Its clock domain, if it declared one.
    pub clock: Option<Clock>,
    /// The address space it sees, if it declared one.
    pub space: Option<SpaceId>,
    /// Its properties, evaluated, with `clock` and `space` removed.
    pub props: Props,
    /// Where each property was written.
    pub prop_spans: PropSpans,
    /// Where the class was written.
    pub class_span: Span,
    /// Where the name was written.
    pub name_span: Span,
    /// The whole declaration.
    pub span: Span,
}

/// What a `map` statement puts into an address space.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MapTarget {
    /// An object, or one named region of it: `apu.regs`.
    Region {
        /// The object.
        object: ObjectId,
        /// The region within it, or `None` for the object's whole region.
        region: Option<String>,
        /// Where it was written.
        span: Span,
    },
    /// `mirror(x)` — a window that repeats its target to fill the mapping.
    Mirror {
        /// What is mirrored.
        inner: Box<MapTarget>,
        /// Where it was written.
        span: Span,
    },
    /// `alias(x)` or `alias(x, offset)` — a non-repeating window.
    Alias {
        /// What is aliased.
        inner: Box<MapTarget>,
        /// Offset into the target.
        offset: u64,
        /// Where it was written.
        span: Span,
    },
    /// `split(reads, writes)` — one address, two devices.
    Split {
        /// Where a read goes.
        reads: Box<MapTarget>,
        /// Where a write goes.
        writes: Box<MapTarget>,
        /// Where it was written.
        span: Span,
    },
}

impl MapTarget {
    /// Where the target was written.
    pub fn span(&self) -> Span {
        match self {
            MapTarget::Region { span, .. }
            | MapTarget::Mirror { span, .. }
            | MapTarget::Alias { span, .. }
            | MapTarget::Split { span, .. } => *span,
        }
    }

    /// The object at the bottom of the target, past any windows.
    pub fn object(&self) -> ObjectId {
        match self {
            MapTarget::Region { object, .. } => *object,
            MapTarget::Mirror { inner, .. } | MapTarget::Alias { inner, .. } => inner.object(),
            // The read side is the one an access is most likely to be about,
            // and a target only reports one object.
            MapTarget::Split { reads, .. } => reads.object(),
        }
    }
}

/// One `map` statement, resolved.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Mapping {
    /// The space mapped into.
    pub space: SpaceId,
    /// The base address in that space.
    pub base: u64,
    /// The size of the window, in bytes.
    pub size: u64,
    /// What is mapped there.
    pub target: MapTarget,
    /// Per-mapping attributes from the trailing block (§4.1).
    pub props: Props,
    /// Where each attribute was written.
    pub prop_spans: PropSpans,
    /// Where the base address was written.
    pub base_span: Span,
    /// Where the size was written.
    pub size_span: Span,
    /// The whole statement.
    pub span: Span,
}

/// One end of a wire: an object and one of its pins.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Pin {
    /// The object.
    pub object: ObjectId,
    /// The pin's name, as the device knows it.
    pub port: String,
    /// Where the endpoint was written.
    pub span: Span,
}

/// One `wire` statement, resolved (`ROADMAP.md` §4.3).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Wire {
    /// The driving pin.
    pub from: Pin,
    /// The driven pin. Several wires may share one — that is wired-OR, and the
    /// sink tracks its sources.
    pub to: Pin,
    /// The whole statement.
    pub span: Span,
}

/// A machine description with every name resolved and every value evaluated.
///
/// The realizer's input. Ids index the vectors directly, and the vectors are
/// in declaration order, so two runs over one file produce identical output
/// (`CLAUDE.md`, determinism).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Resolved {
    /// The machine's name, as written in `machine "nes"`.
    pub name: String,
    /// Parameter values after overrides, in declaration order.
    pub params: Props,
    /// Where each parameter was declared.
    pub param_spans: PropSpans,
    /// The oscillators, in declaration order.
    pub oscillators: Vec<Oscillator>,
    /// The address spaces, in declaration order.
    pub spaces: Vec<Space>,
    /// The objects, in declaration order.
    pub objects: Vec<Object>,
    /// The memory map, in file order.
    pub maps: Vec<Mapping>,
    /// The wires, in file order.
    pub wires: Vec<Wire>,
    /// Where the machine's name was written.
    pub name_span: Span,
    /// The whole `machine` block.
    pub span: Span,
}

impl Resolved {
    /// An oscillator by id.
    pub fn oscillator(&self, id: OscId) -> Option<&Oscillator> {
        self.oscillators.get(id.0 as usize)
    }

    /// An address space by id.
    pub fn space(&self, id: SpaceId) -> Option<&Space> {
        self.spaces.get(id.0 as usize)
    }

    /// An object by id.
    pub fn object(&self, id: ObjectId) -> Option<&Object> {
        self.objects.get(id.0 as usize)
    }

    /// An object by fully qualified name.
    pub fn object_named(&self, name: &str) -> Option<(ObjectId, &Object)> {
        self.objects
            .iter()
            .enumerate()
            .find(|(_, o)| o.name == name)
            .map(|(i, o)| (ObjectId(u32::try_from(i).unwrap_or(u32::MAX)), o))
    }

    /// How many wires drive `pin`'s object and port — the fan-in §4.3 expects a
    /// sink to resolve.
    pub fn fan_in(&self, object: ObjectId, port: &str) -> usize {
        self.wires
            .iter()
            .filter(|w| w.to.object == object && w.to.port == port)
            .count()
    }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

/// Resolve a machine description.
///
/// `root` must already be in `map`; `loader` supplies any `include`d file (use
/// [`NoIncludes`](crate::machine::sources::NoIncludes) when there is no search
/// path). Included files are added to `map` as they are read, so the returned
/// diagnostic renders against it whichever file it points into.
///
/// ```
/// use rsemu::machine::resolver::{ResolveOptions, resolve};
/// use rsemu::machine::sources::{NoIncludes, SourceMap};
///
/// let mut map = SourceMap::new();
/// let root = map.add(
///     "nes.machine",
///     r#"machine "nes" {
///          osc master = 236250000/11 Hz
///          object cpu "mos6502" { clock = master / 12 }
///          object ppu "nes.ppu" { clock = master / 4 }
///          wire ppu.nmi -> cpu.nmi
///        }"#,
/// )
/// .expect("fits");
/// let machine = resolve(&mut map, root, &mut NoIncludes, &ResolveOptions::new())
///     .map_err(|d| map.to_error(&d))?;
///
/// // The CPU:PPU ratio is exact, and it is 3 whatever the crystal is.
/// assert_eq!(machine.objects.len(), 2);
/// assert_eq!(machine.wires.len(), 1);
/// # Ok::<(), rsemu::Error>(())
/// ```
pub fn resolve(
    map: &mut SourceMap,
    root: FileId,
    loader: &mut dyn IncludeLoader,
    options: &ResolveOptions,
) -> Result<Resolved, Diagnostic> {
    Resolver::new(options).run(map, root, loader)
}

// ---------------------------------------------------------------------------
// Stage 1: includes
// ---------------------------------------------------------------------------

/// One frame of the include stack: which file, and the `include` that pulled
/// it in.
#[derive(Debug, Clone)]
struct IncludeFrame {
    name: String,
    at: Span,
}

/// A statement plus the template-instance namespace it was expanded into.
#[derive(Debug, Clone)]
struct Scoped {
    /// `""` at machine level, `"core0."` inside an instance, and so on.
    scope: String,
    stmt: Stmt,
}

/// What a name refers to. One namespace for all three, so a link never has to
/// guess which table to look in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Decl {
    Osc(OscId),
    Space(SpaceId),
    Object(ObjectId),
}

impl Decl {
    fn what(self) -> &'static str {
        match self {
            Decl::Osc(_) => "oscillator",
            Decl::Space(_) => "address space",
            Decl::Object(_) => "object",
        }
    }
}

/// The resolver's working state.
struct Resolver<'a> {
    options: &'a ResolveOptions,
    /// Templates by name, from every file.
    templates: BTreeMap<String, TemplateDecl>,
    /// Parameter bindings, as expressions ready to substitute.
    params: BTreeMap<String, Binding>,
    /// Parameters in declaration order, for messages and for `Resolved`.
    param_order: Vec<String>,
    /// Declared names, in declaration order.
    decl_order: Vec<(String, Decl, Span)>,
    /// The same, indexed. `BTreeMap` rather than a hash map: nothing here may
    /// depend on iteration order (`CLAUDE.md`).
    decls: BTreeMap<String, usize>,
    /// How many statements the expansion has produced.
    produced: u32,
    /// How many expression nodes substitution has produced.
    ///
    /// A `Cell` because substitution runs behind `&self`, and the count has to
    /// be global: `template t(a) { instance i = t2(a = a + a) }` doubles its
    /// argument at every level, so a per-statement limit would not see the
    /// bomb until it had already been built.
    nodes: core::cell::Cell<u32>,
}

/// A name bound to an expression, and where the binding came from.
#[derive(Debug, Clone)]
struct Binding {
    expr: Expr,
    /// Where the binding was declared, for the "ambiguous name" note.
    span: Span,
    /// How many nodes `expr` has, so substituting it costs O(1) to charge.
    nodes: u32,
}

impl Binding {
    /// Bind `expr`, counting it once.
    fn new(expr: Expr, span: Span) -> Binding {
        let nodes = count_nodes(&expr);
        Binding { expr, span, nodes }
    }
}

/// How many nodes an expression has, saturating rather than overflowing.
fn count_nodes(expr: &Expr) -> u32 {
    let mut n: u32 = 1;
    match expr {
        Expr::Num(_) | Expr::Str(_) | Expr::Bool(_) | Expr::Path(_) => {}
        Expr::Call { args, .. } => {
            for a in args {
                n = n.saturating_add(count_nodes(a));
            }
        }
        Expr::Unary { operand, .. } => n = n.saturating_add(count_nodes(operand)),
        Expr::Binary { lhs, rhs, .. } => {
            n = n
                .saturating_add(count_nodes(lhs))
                .saturating_add(count_nodes(rhs));
        }
        Expr::List { items, .. } => {
            for i in items {
                n = n.saturating_add(count_nodes(i));
            }
        }
        Expr::Map { entries, .. } => {
            for e in entries {
                n = n.saturating_add(count_nodes(&e.value));
            }
        }
    }
    n
}

impl<'a> Resolver<'a> {
    fn new(options: &'a ResolveOptions) -> Resolver<'a> {
        Resolver {
            options,
            templates: BTreeMap::new(),
            params: BTreeMap::new(),
            param_order: Vec::new(),
            decl_order: Vec::new(),
            decls: BTreeMap::new(),
            produced: 0,
            nodes: core::cell::Cell::new(0),
        }
    }

    fn run(
        mut self,
        map: &mut SourceMap,
        root: FileId,
        loader: &mut dyn IncludeLoader,
    ) -> Result<Resolved, Diagnostic> {
        // 1. Read the whole include tree into one flat statement list.
        let root_name = map.name(root).unwrap_or("<root>").to_owned();
        let mut seen = BTreeSet::new();
        seen.insert(root_name.clone());
        let mut stack = alloc::vec![IncludeFrame {
            name: root_name,
            at: map.file_span(root).unwrap_or(Span::at(0)),
        }];
        let unit = map.parse(root)?;
        let stmts = self.splice(unit.stmts, map, loader, &mut stack, &mut seen)?;

        // 2. Pick the machine block and build its statement list: file-scope
        //    statements first (that is what an `include`d fragment is), then
        //    the machine's own body.
        let (machine, body) = self.select_machine(&stmts)?;

        // 3. Templates and parameters, from both scopes.
        self.collect_templates(&stmts)?;
        self.collect_templates(&machine.body)?;
        self.collect_params(&stmts, &machine.body)?;

        // 4. Expand instances and loops into a flat list of declarations.
        let mut expanded = Vec::new();
        let mut env = Env::new();
        let mut trail = Vec::new();
        self.expand(&body, "", &mut env, &mut trail, 0, &mut expanded)?;

        // 5. Tables, then values and links.
        self.build_tables(&expanded)?;
        self.finish(machine, &expanded)
    }

    /// Depth-first `include` splicing, in place and in order.
    fn splice(
        &mut self,
        stmts: Vec<Stmt>,
        map: &mut SourceMap,
        loader: &mut dyn IncludeLoader,
        stack: &mut Vec<IncludeFrame>,
        seen: &mut BTreeSet<String>,
    ) -> Result<Vec<Stmt>, Diagnostic> {
        let mut out = Vec::new();
        for stmt in stmts {
            match stmt {
                Stmt::Include(inc) => {
                    let from = stack.last().map_or("", |f| f.name.as_str()).to_owned();
                    if u32::try_from(stack.len()).unwrap_or(u32::MAX) > self.options.max_depth {
                        return Err(Diagnostic::new(
                            inc.span,
                            format!(
                                "`include` nests more than {} levels deep",
                                self.options.max_depth
                            ),
                        ));
                    }
                    let loaded = loader
                        .load(&inc.path.node, &from)
                        .map_err(|message| Diagnostic::new(inc.path.span, message))?;

                    // A cycle names the whole loop, which is the only form of
                    // the message that tells you where to cut it.
                    if let Some(frame) = stack.iter().find(|f| f.name == loaded.name) {
                        let mut path = String::new();
                        for f in stack.iter().skip_while(|f| f.name != loaded.name) {
                            path.push_str(&format!("`{}` → ", f.name));
                        }
                        path.push_str(&format!("`{}`", loaded.name));
                        let note = frame.at;
                        return Err(Diagnostic::new(
                            inc.path.span,
                            format!("include cycle: {path}"),
                        )
                        .with_note(note, "the cycle starts here"));
                    }
                    // Include-once: a fragment two files both need is spliced
                    // once, not twice with duplicate declarations.
                    if !seen.insert(loaded.name.clone()) {
                        continue;
                    }
                    let id = map.add(loaded.name.clone(), loaded.text)?;
                    let unit = map.parse(id)?;
                    stack.push(IncludeFrame {
                        name: loaded.name,
                        at: inc.span,
                    });
                    let inner = self.splice(unit.stmts, map, loader, stack, seen)?;
                    stack.pop();
                    out.extend(inner);
                }
                Stmt::Machine(mut m) => {
                    m.body = self.splice(m.body, map, loader, stack, seen)?;
                    out.push(Stmt::Machine(m));
                }
                Stmt::Template(mut t) => {
                    t.body = self.splice(t.body, map, loader, stack, seen)?;
                    out.push(Stmt::Template(t));
                }
                Stmt::For(mut f) => {
                    f.body = self.splice(f.body, map, loader, stack, seen)?;
                    out.push(Stmt::For(f));
                }
                other => out.push(other),
            }
        }
        Ok(out)
    }

    /// Find the `machine` block to resolve, and the statements that make it up.
    fn select_machine<'s>(
        &self,
        stmts: &'s [Stmt],
    ) -> Result<(&'s crate::machine::ast::MachineDecl, Vec<Stmt>), Diagnostic> {
        let machines: Vec<&crate::machine::ast::MachineDecl> = stmts
            .iter()
            .filter_map(|s| match s {
                Stmt::Machine(m) => Some(m),
                _ => None,
            })
            .collect();
        let Some(first) = machines.first() else {
            let span = stmts.first().map_or(Span::at(0), Stmt::span);
            return Err(Diagnostic::new(
                span,
                "no `machine` block: a description must declare one, as in `machine \"nes\" { … }`",
            ));
        };
        let chosen = match &self.options.machine {
            Some(want) => match machines.iter().find(|m| &m.name.node == want) {
                Some(m) => *m,
                None => {
                    let names = list(machines.iter().map(|m| m.name.node.as_str()));
                    return Err(Diagnostic::new(
                        first.name.span,
                        format!("no machine named `{want}`; this description declares {names}"),
                    ));
                }
            },
            None if machines.len() > 1 => {
                let names = list(machines.iter().map(|m| m.name.node.as_str()));
                return Err(Diagnostic::new(
                    machines[1].name.span,
                    format!(
                        "this description declares several machines ({names}); say which one to \
                         resolve"
                    ),
                )
                .with_note(first.name.span, "the first one is declared here"));
            }
            None => *first,
        };

        // Prelude first: an `include`d fragment declares objects and templates
        // that the machine is expected to already have.
        let mut body = Vec::new();
        for stmt in stmts {
            match stmt {
                Stmt::Machine(_) | Stmt::Param(_) | Stmt::Template(_) | Stmt::Include(_) => {}
                other => body.push(other.clone()),
            }
        }
        body.extend(chosen.body.iter().cloned());
        Ok((chosen, body))
    }

    /// Collect `template` declarations, rejecting duplicates.
    fn collect_templates(&mut self, stmts: &[Stmt]) -> Result<(), Diagnostic> {
        for stmt in stmts {
            let Stmt::Template(t) = stmt else { continue };
            let name = literal_name(&t.name, "a template name")?;
            if let Some(prev) = self.templates.get(&name) {
                return Err(Diagnostic::new(
                    t.name.span,
                    format!("template `{name}` is declared twice"),
                )
                .with_note(prev.name.span, "first declared here"));
            }
            self.templates.insert(name, t.clone());
        }
        Ok(())
    }

    /// Collect `param` declarations and apply the caller's overrides.
    fn collect_params(&mut self, file: &[Stmt], machine: &[Stmt]) -> Result<(), Diagnostic> {
        let mut decls: Vec<&ParamDecl> = Vec::new();
        let mut seen: BTreeMap<String, Span> = BTreeMap::new();
        for stmt in file.iter().chain(machine.iter()) {
            let Stmt::Param(p) = stmt else { continue };
            let name = literal_name(&p.name, "a parameter name")?;
            if let Some(prev) = seen.get(&name) {
                return Err(Diagnostic::new(
                    p.name.span,
                    format!("parameter `{name}` is declared twice"),
                )
                .with_note(*prev, "first declared here"));
            }
            seen.insert(name.clone(), p.name.span);
            self.param_order.push(name);
            decls.push(p);
        }
        // A `param` anywhere else is refused: `-p` names one knob, and a knob
        // that exists once per loop iteration is not one knob.
        reject_nested_params(file)?;
        reject_nested_params(machine)?;

        for (i, decl) in decls.iter().enumerate() {
            let name = &self.param_order[i];
            let override_text = self
                .options
                .params
                .iter()
                .rev()
                .find(|(n, _)| n == name)
                .map(|(_, v)| v.as_str());
            let expr = match (override_text, &decl.default) {
                (Some(text), _) => value_expr(&Value::parse_scalar(text), decl.name.span)
                    .ok_or_else(|| {
                        Diagnostic::new(
                            decl.name.span,
                            format!("`-p {name}={text}` cannot be used as a value here"),
                        )
                    })?,
                (None, Some(default)) => default.clone(),
                (None, None) => {
                    return Err(Diagnostic::new(
                        decl.span,
                        format!(
                            "parameter `{name}` has no default and was not given a value (pass \
                             `-p {name}=…`)"
                        ),
                    ));
                }
            };
            self.params
                .insert(name.clone(), Binding::new(expr, decl.name.span));
        }

        // An override nobody declared is a typo, and silently ignoring it is
        // how an afternoon disappears.
        for (name, _) in &self.options.params {
            if !self.params.contains_key(name) {
                let known = list(self.param_order.iter().map(String::as_str));
                let span = decls.first().map_or(Span::at(0), |d| d.span);
                return Err(Diagnostic::new(
                    span,
                    format!("no parameter named `{name}`; this machine declares {known}"),
                ));
            }
        }
        Ok(())
    }
}

/// `param` is only legal at file scope or directly inside a `machine`.
fn reject_nested_params(stmts: &[Stmt]) -> Result<(), Diagnostic> {
    for stmt in stmts {
        match stmt {
            Stmt::For(f) => {
                nested_param(&f.body)?;
                reject_nested_params(&f.body)?;
            }
            Stmt::Template(t) => {
                nested_param(&t.body)?;
                reject_nested_params(&t.body)?;
            }
            _ => {}
        }
    }
    Ok(())
}

fn nested_param(body: &[Stmt]) -> Result<(), Diagnostic> {
    for stmt in body {
        // Both are declarations rather than statements: repeating one per loop
        // iteration has no meaning, and silently ignoring it would surface as
        // "no template named …" pointing at the use rather than the mistake.
        let (span, what) = match stmt {
            Stmt::Param(p) => (p.span, "param"),
            Stmt::Template(t) => (t.span, "template"),
            _ => continue,
        };
        return Err(Diagnostic::new(
            span,
            format!("`{what}` must be declared at file scope or directly inside a `machine` block"),
        ));
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Stage 2: expansion
// ---------------------------------------------------------------------------

/// Lexically scoped bindings: parameters at the bottom, then one frame per
/// template instantiation or loop iteration.
#[derive(Debug)]
struct Env {
    frames: Vec<BTreeMap<String, Binding>>,
}

impl Env {
    fn new() -> Env {
        Env { frames: Vec::new() }
    }

    fn get<'e>(&'e self, name: &str, params: &'e BTreeMap<String, Binding>) -> Option<&'e Binding> {
        for frame in self.frames.iter().rev() {
            if let Some(b) = frame.get(name) {
                return Some(b);
            }
        }
        params.get(name)
    }
}

impl Resolver<'_> {
    /// Expand instances and loops, substituting bindings as it goes.
    ///
    /// `trail` is the stack of template names currently being instantiated, so
    /// recursion can be named rather than merely stopped.
    fn expand(
        &mut self,
        stmts: &[Stmt],
        scope: &str,
        env: &mut Env,
        trail: &mut Vec<(String, Span)>,
        depth: u32,
        out: &mut Vec<Scoped>,
    ) -> Result<(), Diagnostic> {
        if depth > self.options.max_depth {
            let span = stmts.first().map_or(Span::at(0), Stmt::span);
            return Err(Diagnostic::new(
                span,
                format!(
                    "`template` and `for` nest more than {} levels deep",
                    self.options.max_depth
                ),
            ));
        }
        for stmt in stmts {
            match stmt {
                // Already collected, or already spliced.
                Stmt::Param(_) | Stmt::Template(_) | Stmt::Include(_) => {}
                Stmt::Machine(m) => {
                    return Err(Diagnostic::new(
                        m.span,
                        "a `machine` block cannot appear inside another statement",
                    ));
                }
                Stmt::For(f) => self.expand_for(f, scope, env, trail, depth, out)?,
                Stmt::Instance(i) => self.expand_instance(i, scope, env, trail, depth, out)?,
                other => {
                    self.produced = self.produced.saturating_add(1);
                    if self.produced > self.options.max_statements {
                        return Err(Diagnostic::new(
                            other.span(),
                            format!(
                                "expansion produced more than {} statements",
                                self.options.max_statements
                            ),
                        ));
                    }
                    out.push(Scoped {
                        scope: scope.to_owned(),
                        stmt: self.substitute(other, env)?,
                    });
                }
            }
        }
        Ok(())
    }

    fn expand_for(
        &mut self,
        f: &ForStmt,
        scope: &str,
        env: &mut Env,
        trail: &mut Vec<(String, Span)>,
        depth: u32,
        out: &mut Vec<Scoped>,
    ) -> Result<(), Diagnostic> {
        let var = literal_name(&f.var, "a loop variable")?;
        let start = self.const_int(&f.start, env)?;
        let end = self.const_int(&f.end, env)?;
        // Widened, because `for i in i64::MIN..i64::MAX` must be a diagnostic
        // rather than an overflow panic in a debug build.
        let last = i128::from(end) - i128::from(!f.inclusive);
        if last < i128::from(start) {
            // An empty range is not an error — `for i in 0..0` is how a
            // parameterised count of zero has to spell itself.
            return Ok(());
        }
        let count = (last - i128::from(start) + 1).unsigned_abs();
        if count > u128::from(self.options.max_iterations) {
            return Err(Diagnostic::new(
                f.start.span().join(f.end.span()),
                format!(
                    "this loop runs {count} times; the limit is {}",
                    self.options.max_iterations
                ),
            ));
        }
        let mut value = i128::from(start);
        while value <= last {
            let Ok(index) = i64::try_from(value) else {
                return Ok(());
            };
            let mut frame = BTreeMap::new();
            frame.insert(
                var.clone(),
                Binding::new(int_expr(index, f.var.span), f.var.span),
            );
            env.frames.push(frame);
            let result = self.expand(&f.body, scope, env, trail, depth + 1, out);
            env.frames.pop();
            result?;
            value += 1;
        }
        Ok(())
    }

    fn expand_instance(
        &mut self,
        inst: &InstanceStmt,
        scope: &str,
        env: &mut Env,
        trail: &mut Vec<(String, Span)>,
        depth: u32,
        out: &mut Vec<Scoped>,
    ) -> Result<(), Diagnostic> {
        let template_name = literal_name(&inst.template, "a template name")?;
        let Some(template) = self.templates.get(&template_name).cloned() else {
            let known = list(self.templates.keys().map(String::as_str));
            return Err(Diagnostic::new(
                inst.template.span,
                format!("no template named `{template_name}`; declared templates are {known}"),
            ));
        };
        if let Some((_, at)) = trail.iter().find(|(n, _)| *n == template_name) {
            let mut path = String::new();
            for (n, _) in trail.iter().skip_while(|(n, _)| *n != template_name) {
                path.push_str(&format!("`{n}` → "));
            }
            path.push_str(&format!("`{template_name}`"));
            return Err(
                Diagnostic::new(inst.template.span, format!("template recursion: {path}"))
                    .with_note(*at, "the cycle starts here"),
            );
        }

        // Bind arguments in the *caller's* environment, so a template body
        // never sees a name from the call site by accident.
        let mut frame: BTreeMap<String, Binding> = BTreeMap::new();
        let mut positional = 0usize;
        let mut bound: Vec<String> = Vec::new();
        for arg in &inst.args {
            let (name, span) = match &arg.name {
                Some(n) => (literal_name(n, "an argument name")?, n.span),
                None => {
                    let Some(param) = template.params.get(positional) else {
                        return Err(Diagnostic::new(
                            arg.span,
                            format!(
                                "template `{template_name}` takes {} argument(s)",
                                template.params.len()
                            ),
                        ));
                    };
                    positional += 1;
                    (literal_name(&param.name, "a parameter name")?, arg.span)
                }
            };
            if !template
                .params
                .iter()
                .any(|p| p.name.as_literal() == Some(name.as_str()))
            {
                let known = list(template.params.iter().filter_map(|p| p.name.as_literal()));
                return Err(Diagnostic::new(
                    span,
                    format!(
                        "template `{template_name}` has no parameter `{name}`; it takes {known}"
                    ),
                ));
            }
            if bound.contains(&name) {
                return Err(Diagnostic::new(
                    span,
                    format!("argument `{name}` is given twice"),
                ));
            }
            bound.push(name.clone());
            frame.insert(
                name,
                Binding::new(self.substitute_expr(&arg.value, env)?, arg.span),
            );
        }
        for param in &template.params {
            let name = literal_name(&param.name, "a parameter name")?;
            if frame.contains_key(&name) {
                continue;
            }
            let Some(default) = &param.default else {
                return Err(Diagnostic::new(
                    inst.span,
                    format!(
                        "template `{template_name}` needs an argument for `{name}`, which has no \
                         default"
                    ),
                ));
            };
            // A default is evaluated in the template's own frame, so one
            // parameter may be defined in terms of another declared before it.
            let expr = {
                env.frames.push(frame.clone());
                let out = self.substitute_expr(default, env);
                env.frames.pop();
                out?
            };
            frame.insert(name, Binding::new(expr, param.span));
        }

        let instance = self.expand_name(&inst.name, env)?;
        let inner_scope = format!("{scope}{instance}.");
        env.frames.push(frame);
        trail.push((template_name, inst.template.span));
        let result = self.expand(&template.body, &inner_scope, env, trail, depth + 1, out);
        trail.pop();
        env.frames.pop();
        result
    }

    /// Substitute bindings through one declaration statement.
    fn substitute(&self, stmt: &Stmt, env: &Env) -> Result<Stmt, Diagnostic> {
        Ok(match stmt {
            Stmt::Osc(o) => Stmt::Osc(OscDecl {
                name: self.expanded_name(&o.name, env)?,
                freq: self.substitute_expr(&o.freq, env)?,
                unit: o.unit.clone(),
                span: o.span,
            }),
            Stmt::Space(s) => Stmt::Space(SpaceDecl {
                name: self.expanded_name(&s.name, env)?,
                props: self.substitute_props(&s.props, env)?,
                span: s.span,
            }),
            Stmt::Object(o) => Stmt::Object(ObjectDecl {
                name: self.expanded_name(&o.name, env)?,
                class: o.class.clone(),
                props: self.substitute_props(&o.props, env)?,
                span: o.span,
            }),
            Stmt::Map(m) => Stmt::Map(MapStmt {
                space: self.expanded_name(&m.space, env)?,
                base: self.substitute_expr(&m.base, env)?,
                size: self.substitute_expr(&m.size, env)?,
                target: self.substitute_expr(&m.target, env)?,
                props: self.substitute_props(&m.props, env)?,
                span: m.span,
            }),
            Stmt::Wire(w) => Stmt::Wire(WireStmt {
                from: self.substitute_path(&w.from, env)?,
                to: self.substitute_path(&w.to, env)?,
                span: w.span,
            }),
            other => other.clone(),
        })
    }

    fn substitute_props(&self, props: &[Property], env: &Env) -> Result<Vec<Property>, Diagnostic> {
        let mut out = Vec::with_capacity(props.len());
        for p in props {
            out.push(Property {
                name: self.expanded_name(&p.name, env)?,
                value: self.substitute_expr(&p.value, env)?,
                span: p.span,
            });
        }
        Ok(out)
    }

    fn substitute_path(&self, path: &Path, env: &Env) -> Result<Path, Diagnostic> {
        let mut segments = Vec::with_capacity(path.segments.len());
        for seg in &path.segments {
            segments.push(self.expanded_name(seg, env)?);
        }
        Ok(Path {
            segments,
            span: path.span,
        })
    }

    /// Replace bound single-segment paths with what they are bound to.
    ///
    /// Only single-segment: a parameter cannot be dotted, so `ppu.regs` is
    /// never touched even if a parameter happens to be called `ppu`.
    fn substitute_expr(&self, expr: &Expr, env: &Env) -> Result<Expr, Diagnostic> {
        self.charge(1, expr.span())?;
        Ok(match expr {
            Expr::Num(_) | Expr::Str(_) | Expr::Bool(_) => expr.clone(),
            Expr::Path(p) => {
                if p.segments.len() == 1
                    && let Some(name) = p.segments[0].as_literal()
                    && let Some(binding) = env.get(name, &self.params)
                {
                    self.charge(binding.nodes, expr.span())?;
                    return Ok(binding.expr.clone());
                }
                Expr::Path(self.substitute_path(p, env)?)
            }
            Expr::Call { callee, args, span } => {
                let mut out = Vec::with_capacity(args.len());
                for a in args {
                    out.push(self.substitute_expr(a, env)?);
                }
                Expr::Call {
                    callee: self.substitute_path(callee, env)?,
                    args: out,
                    span: *span,
                }
            }
            Expr::Unary { op, operand, span } => Expr::Unary {
                op: *op,
                operand: Box::new(self.substitute_expr(operand, env)?),
                span: *span,
            },
            Expr::Binary { op, lhs, rhs, span } => Expr::Binary {
                op: *op,
                lhs: Box::new(self.substitute_expr(lhs, env)?),
                rhs: Box::new(self.substitute_expr(rhs, env)?),
                span: *span,
            },
            Expr::List { items, span } => {
                let mut out = Vec::with_capacity(items.len());
                for i in items {
                    out.push(self.substitute_expr(i, env)?);
                }
                Expr::List {
                    items: out,
                    span: *span,
                }
            }
            Expr::Map { entries, span } => Expr::Map {
                entries: self.substitute_props(entries, env)?,
                span: *span,
            },
        })
    }

    /// Charge `n` expression nodes against the expansion budget.
    fn charge(&self, n: u32, span: Span) -> Result<(), Diagnostic> {
        let total = self.nodes.get().saturating_add(n);
        self.nodes.set(total);
        if total > self.options.max_expr_nodes {
            return Err(Diagnostic::new(
                span,
                format!(
                    "expansion produced more than {} expression nodes",
                    self.options.max_expr_nodes
                ),
            ));
        }
        Ok(())
    }

    /// A [`Name`] with its `$` substitutions evaluated, as text.
    fn expand_name(&self, name: &Name, env: &Env) -> Result<String, Diagnostic> {
        if let Some(text) = name.as_literal() {
            return Ok(text.to_owned());
        }
        let mut out = String::new();
        for part in &name.parts {
            match part {
                NamePart::Literal(text) => out.push_str(text),
                NamePart::Substitution(expr) => {
                    let expr = self.substitute_expr(expr, env)?;
                    out.push_str(&self.name_text(&expr)?);
                }
            }
        }
        // `bank${1/2}` would otherwise produce a name no reference can spell.
        if out.is_empty()
            || !out
                .chars()
                .all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '-')
        {
            return Err(Diagnostic::new(
                name.span,
                format!("`{out}` is not a valid name once substituted"),
            ));
        }
        Ok(out)
    }

    /// The same, wrapped back into a literal [`Name`] so the AST stays an AST.
    fn expanded_name(&self, name: &Name, env: &Env) -> Result<Name, Diagnostic> {
        let text = self.expand_name(name, env)?;
        Ok(Name {
            parts: alloc::vec![NamePart::Literal(text)],
            span: name.span,
        })
    }

    /// What a substitution contributes to a name.
    fn name_text(&self, expr: &Expr) -> Result<String, Diagnostic> {
        match expr {
            Expr::Str(s) => Ok(s.node.clone()),
            Expr::Bool(b) => Ok(b.node.to_string()),
            Expr::Path(p) if p.as_literal().is_some() => Ok(p.as_literal().unwrap_or_default()),
            other => {
                let value = other.eval_rational()?;
                match value.to_integer() {
                    Some(n) => Ok(n.to_string()),
                    None => Err(Diagnostic::new(
                        other.span(),
                        format!(
                            "`{}/{}` is not a whole number, so it cannot be part of a name",
                            value.numerator(),
                            value.denominator()
                        ),
                    )),
                }
            }
        }
    }

    /// A loop bound: an integer, after substitution.
    fn const_int(&self, expr: &Expr, env: &Env) -> Result<i64, Diagnostic> {
        let expr = self.substitute_expr(expr, env)?;
        let value = expr.eval_rational()?;
        match value.to_integer().and_then(|n| i64::try_from(n).ok()) {
            Some(n) => Ok(n),
            None => Err(Diagnostic::new(
                expr.span(),
                "a loop bound must be a whole number that fits in 64 bits",
            )),
        }
    }
}

// ---------------------------------------------------------------------------
// Stage 3: declaration tables
// ---------------------------------------------------------------------------

impl Resolver<'_> {
    fn build_tables(&mut self, stmts: &[Scoped]) -> Result<(), Diagnostic> {
        let (mut oscs, mut spaces, mut objects) = (0u32, 0u32, 0u32);
        for Scoped { scope, stmt } in stmts {
            let (name, span, decl) = match stmt {
                Stmt::Osc(o) => {
                    let d = Decl::Osc(OscId(oscs));
                    oscs += 1;
                    (o.name.as_literal().unwrap_or_default(), o.name.span, d)
                }
                Stmt::Space(s) => {
                    let d = Decl::Space(SpaceId(spaces));
                    spaces += 1;
                    (s.name.as_literal().unwrap_or_default(), s.name.span, d)
                }
                Stmt::Object(o) => {
                    let d = Decl::Object(ObjectId(objects));
                    objects += 1;
                    (o.name.as_literal().unwrap_or_default(), o.name.span, d)
                }
                _ => continue,
            };
            let full = format!("{scope}{name}");
            // One name cannot mean two things. A parameter and a declaration
            // sharing a name is the case §5 leaves open; it is refused here
            // rather than resolved by precedence at every use.
            if let Some(binding) = self.params.get(name) {
                return Err(Diagnostic::new(
                    span,
                    format!(
                        "`{name}` is ambiguous: it is declared both as a parameter and as {} \
                         `{full}`",
                        article(decl.what())
                    ),
                )
                .with_note(binding.span, "the parameter is declared here"));
            }
            if let Some(prev) = self.decls.get(&full).and_then(|i| self.decl_order.get(*i)) {
                return Err(Diagnostic::new(
                    span,
                    format!(
                        "`{full}` is declared twice, as {} and as {}",
                        prev.1.what(),
                        decl.what()
                    ),
                )
                .with_note(prev.2, "first declared here"));
            }
            self.decls.insert(full.clone(), self.decl_order.len());
            self.decl_order.push((full, decl, span));
        }
        Ok(())
    }

    /// The declaration `name` refers to from `scope`, innermost namespace
    /// first.
    fn lookup(&self, scope: &str, name: &str) -> Option<(String, Decl)> {
        for prefix in scopes(scope) {
            let full = format!("{prefix}{name}");
            if let Some(i) = self.decls.get(&full)
                && let Some((n, d, _)) = self.decl_order.get(*i)
            {
                return Some((n.clone(), *d));
            }
        }
        None
    }

    /// Where a declaration was written, for a note.
    fn decl_span(&self, name: &str) -> Option<Span> {
        self.decls
            .get(name)
            .and_then(|i| self.decl_order.get(*i))
            .map(|(_, _, s)| *s)
    }

    /// Names of a given kind, in declaration order, for "what was in scope".
    fn names_of(&self, want: fn(Decl) -> bool) -> Vec<&str> {
        self.decl_order
            .iter()
            .filter(|(_, d, _)| want(*d))
            .map(|(n, _, _)| n.as_str())
            .collect()
    }
}

/// The namespaces a reference in `scope` searches, innermost first.
///
/// `"a.b."` searches `a.b.`, then `a.`, then the machine's own namespace.
fn scopes(scope: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut rest = scope;
    while !rest.is_empty() {
        out.push(rest.to_owned());
        match rest.trim_end_matches('.').rfind('.') {
            Some(i) => rest = &rest[..=i],
            None => break,
        }
    }
    out.push(String::new());
    out
}

// ---------------------------------------------------------------------------
// Stage 4: values and links
// ---------------------------------------------------------------------------

impl Resolver<'_> {
    fn finish(
        &self,
        machine: &crate::machine::ast::MachineDecl,
        stmts: &[Scoped],
    ) -> Result<Resolved, Diagnostic> {
        let mut out = Resolved {
            name: machine.name.node.clone(),
            params: Props::new(),
            param_spans: PropSpans::new(),
            oscillators: Vec::new(),
            spaces: Vec::new(),
            objects: Vec::new(),
            maps: Vec::new(),
            wires: Vec::new(),
            name_span: machine.name.span,
            span: machine.span,
        };

        for name in &self.param_order {
            let Some(binding) = self.params.get(name) else {
                continue;
            };
            out.params
                .insert(name.clone(), self.value(&binding.expr, "")?);
            out.param_spans.insert(name.clone(), binding.span);
        }

        // Declarations first: a link may name an object declared later, and
        // §5's example does exactly that (`cart.irq` before any `cart`).
        for Scoped { scope, stmt } in stmts {
            match stmt {
                Stmt::Osc(o) => out.oscillators.push(self.oscillator(o, scope)?),
                Stmt::Space(s) => out.spaces.push(self.space_decl(s, scope)?),
                _ => {}
            }
        }
        for Scoped { scope, stmt } in stmts {
            if let Stmt::Object(o) = stmt {
                let object = self.object(o, scope)?;
                out.objects.push(object);
            }
        }
        for Scoped { scope, stmt } in stmts {
            match stmt {
                Stmt::Map(m) => out.maps.push(self.mapping(m, scope)?),
                Stmt::Wire(w) => out.wires.push(self.wire(w, scope)?),
                _ => {}
            }
        }

        self.check_clock_cycles(&out)?;
        Ok(out)
    }

    fn oscillator(&self, decl: &OscDecl, scope: &str) -> Result<Oscillator, Diagnostic> {
        let name = decl.name.as_literal().unwrap_or_default();
        let hz = decl.frequency_hz()?;
        if hz.numerator() <= 0 {
            return Err(Diagnostic::new(
                decl.freq.span(),
                "an oscillator's frequency must be greater than zero",
            ));
        }
        Ok(Oscillator {
            name: format!("{scope}{name}"),
            hz,
            name_span: decl.name.span,
            span: decl.span,
        })
    }

    fn space_decl(&self, decl: &SpaceDecl, scope: &str) -> Result<Space, Diagnostic> {
        let name = decl.name.as_literal().unwrap_or_default();
        let (props, prop_spans) = self.props(&decl.props, scope)?;
        Ok(Space {
            name: format!("{scope}{name}"),
            props,
            prop_spans,
            name_span: decl.name.span,
            span: decl.span,
        })
    }

    fn object(&self, decl: &ObjectDecl, scope: &str) -> Result<Object, Diagnostic> {
        let name = decl.name.as_literal().unwrap_or_default();
        let mut clock = None;
        let mut space = None;
        let mut rest = Vec::new();
        for p in &decl.props {
            match p.name.as_literal() {
                // `clock` and `space` belong to the machine, not the device.
                Some("clock") => clock = Some(self.clock(&p.value, scope)?),
                Some("space") => space = Some(self.space_link(&p.value, scope)?),
                _ => rest.push(p.clone()),
            }
        }
        let (props, prop_spans) = self.props(&rest, scope)?;
        Ok(Object {
            name: format!("{scope}{name}"),
            class: decl.class.node.clone(),
            clock,
            space,
            props,
            prop_spans,
            class_span: decl.class.span,
            name_span: decl.name.span,
            span: decl.span,
        })
    }

    fn mapping(&self, stmt: &MapStmt, scope: &str) -> Result<Mapping, Diagnostic> {
        let space_name = stmt.space.as_literal().unwrap_or_default();
        let space = match self.lookup(scope, space_name) {
            Some((_, Decl::Space(id))) => id,
            Some((full, other)) => {
                return Err(Diagnostic::new(
                    stmt.space.span,
                    format!(
                        "`{space_name}` is {} `{full}`, not an address space",
                        article(other.what())
                    ),
                )
                .with_note(
                    self.decl_span(&full).unwrap_or(stmt.space.span),
                    "declared here",
                ));
            }
            None => {
                let known = list(self.names_of(|d| matches!(d, Decl::Space(_))).into_iter());
                return Err(Diagnostic::new(
                    stmt.space.span,
                    format!("no address space named `{space_name}`; declared spaces are {known}"),
                ));
            }
        };
        let base = self
            .value(&stmt.base, scope)?
            .to_addr("map base")
            .map_err(|e| Diagnostic::new(stmt.base.span(), e.to_string()))?;
        let size = self
            .value(&stmt.size, scope)?
            .to_size("map size")
            .map_err(|e| Diagnostic::new(stmt.size.span(), e.to_string()))?;
        if size == 0 {
            return Err(Diagnostic::new(
                stmt.size.span(),
                "a mapping's size must be greater than zero",
            ));
        }
        if base.checked_add(size).is_none() {
            return Err(Diagnostic::new(
                stmt.base.span().join(stmt.size.span()),
                "this mapping runs past the end of a 64-bit address space",
            ));
        }
        let target = self.map_target(&stmt.target, scope, 0)?;
        let (props, prop_spans) = self.props(&stmt.props, scope)?;
        Ok(Mapping {
            space,
            base,
            size,
            target,
            props,
            prop_spans,
            base_span: stmt.base.span(),
            size_span: stmt.size.span(),
            span: stmt.span,
        })
    }

    fn map_target(&self, expr: &Expr, scope: &str, depth: u32) -> Result<MapTarget, Diagnostic> {
        if depth > 8 {
            return Err(Diagnostic::new(
                expr.span(),
                "a `map` target nests more than 8 levels deep",
            ));
        }
        match expr {
            Expr::Path(p) => {
                let (object, region) = self.object_path(p, scope, false)?;
                Ok(MapTarget::Region {
                    object,
                    region,
                    span: p.span,
                })
            }
            Expr::Call { callee, args, span } => {
                let name = callee.as_literal().unwrap_or_default();
                match name.as_str() {
                    "mirror" => {
                        let [inner] = args.as_slice() else {
                            return Err(Diagnostic::new(
                                *span,
                                "`mirror` takes exactly one argument: `mirror(wram)`",
                            ));
                        };
                        Ok(MapTarget::Mirror {
                            inner: Box::new(self.map_target(inner, scope, depth + 1)?),
                            span: *span,
                        })
                    }
                    "alias" => {
                        let (inner, offset) = match args.as_slice() {
                            [inner] => (inner, 0),
                            [inner, off] => {
                                let value = self.value(off, scope)?;
                                let off = value
                                    .to_addr("alias offset")
                                    .map_err(|e| Diagnostic::new(off.span(), e.to_string()))?;
                                (inner, off)
                            }
                            _ => {
                                return Err(Diagnostic::new(
                                    *span,
                                    "`alias` takes a target and an optional offset: \
                                     `alias(rom, 0x4000)`",
                                ));
                            }
                        };
                        Ok(MapTarget::Alias {
                            inner: Box::new(self.map_target(inner, scope, depth + 1)?),
                            offset,
                            span: *span,
                        })
                    }
                    "split" => {
                        let [reads, writes] = args.as_slice() else {
                            return Err(Diagnostic::new(
                                *span,
                                "`split` takes the read side and the write side: \
                                 `split(ports.port2, apu.frame)`",
                            ));
                        };
                        Ok(MapTarget::Split {
                            reads: Box::new(self.map_target(reads, scope, depth + 1)?),
                            writes: Box::new(self.map_target(writes, scope, depth + 1)?),
                            span: *span,
                        })
                    }
                    other => Err(Diagnostic::new(
                        callee.span,
                        format!(
                            "no map function named `{other}`; the map functions are `mirror`, \
                             `alias` and `split`"
                        ),
                    )),
                }
            }
            other => Err(Diagnostic::new(
                other.span(),
                "a `map` target must name an object, a region, or \
                 `mirror(…)`/`alias(…)`/`split(…)`",
            )),
        }
    }

    fn wire(&self, stmt: &WireStmt, scope: &str) -> Result<Wire, Diagnostic> {
        Ok(Wire {
            from: self.pin(&stmt.from, scope)?,
            to: self.pin(&stmt.to, scope)?,
            span: stmt.span,
        })
    }

    fn pin(&self, path: &Path, scope: &str) -> Result<Pin, Diagnostic> {
        if path.segments.len() < 2 {
            return Err(Diagnostic::new(
                path.span,
                "a wire endpoint must name a pin, as in `cpu.nmi`",
            ));
        }
        let (object, port) = self.object_path(path, scope, true)?;
        let Some(port) = port else {
            return Err(Diagnostic::new(
                path.span,
                "a wire endpoint must name a pin, as in `cpu.nmi`",
            ));
        };
        Ok(Pin {
            object,
            port,
            span: path.span,
        })
    }

    /// Split a dotted path into the object it names and whatever follows.
    ///
    /// Tries the innermost namespace first and, within one namespace, the
    /// longest object name first — so `core0.plic.in0` finds `core0.plic` in
    /// preference to a top-level `core0`.
    fn object_path(
        &self,
        path: &Path,
        scope: &str,
        want_port: bool,
    ) -> Result<(ObjectId, Option<String>), Diagnostic> {
        let mut segments = Vec::with_capacity(path.segments.len());
        for seg in &path.segments {
            match seg.as_literal() {
                Some(text) => segments.push(text),
                None => {
                    return Err(Diagnostic::new(seg.span, "this name was never substituted"));
                }
            }
        }
        let limit = if want_port {
            segments.len().saturating_sub(1)
        } else {
            segments.len()
        };
        for prefix in scopes(scope) {
            for take in (1..=limit).rev() {
                let candidate = format!("{prefix}{}", segments[..take].join("."));
                match self
                    .decls
                    .get(&candidate)
                    .and_then(|i| self.decl_order.get(*i))
                {
                    Some((full, Decl::Object(id), _)) => {
                        let rest = segments[take..].join(".");
                        let _ = full;
                        return Ok((*id, if rest.is_empty() { None } else { Some(rest) }));
                    }
                    Some((full, other, span)) if take == segments.len() => {
                        return Err(Diagnostic::new(
                            path.span,
                            format!(
                                "`{}` is {} `{full}`, not an object",
                                segments.join("."),
                                article(other.what())
                            ),
                        )
                        .with_note(*span, "declared here"));
                    }
                    _ => {}
                }
            }
        }
        let known = list(self.names_of(|d| matches!(d, Decl::Object(_))).into_iter());
        Err(Diagnostic::new(
            path.span,
            format!(
                "no object named `{}`; objects in scope are {known}",
                segments[..limit.max(1).min(segments.len())].join(".")
            ),
        ))
    }

    /// `space = cpubus`: a link that must land on an address space.
    fn space_link(&self, expr: &Expr, scope: &str) -> Result<SpaceId, Diagnostic> {
        let Expr::Path(p) = expr else {
            return Err(Diagnostic::new(
                expr.span(),
                "`space` must name a declared address space",
            ));
        };
        let name = p.as_literal().unwrap_or_default();
        match self.lookup(scope, &name) {
            Some((_, Decl::Space(id))) => Ok(id),
            Some((full, other)) => Err(Diagnostic::new(
                p.span,
                format!(
                    "`{name}` is {} `{full}`, not an address space",
                    article(other.what())
                ),
            )
            .with_note(self.decl_span(&full).unwrap_or(p.span), "declared here")),
            None => {
                let known = list(self.names_of(|d| matches!(d, Decl::Space(_))).into_iter());
                Err(Diagnostic::new(
                    p.span,
                    format!("no address space named `{name}`; declared spaces are {known}"),
                ))
            }
        }
    }

    /// `clock = master / 12`: one oscillator or domain, exactly scaled.
    fn clock(&self, expr: &Expr, scope: &str) -> Result<Clock, Diagnostic> {
        let (parent, ratio) = self.clock_parts(expr, scope)?;
        let Some(parent) = parent else {
            return Err(Diagnostic::new(
                expr.span(),
                "a `clock` must be derived from a declared oscillator, as in `master / 12`; a \
                 bare frequency needs an `osc` declaration of its own",
            ));
        };
        let num = ratio.numerator();
        let den = ratio.denominator();
        if num <= 0 {
            return Err(Diagnostic::new(
                expr.span(),
                "a clock scale must be greater than zero",
            ));
        }
        let (Ok(mul), Ok(div)) = (u64::try_from(num), u64::try_from(den)) else {
            return Err(Diagnostic::new(
                expr.span(),
                "this clock ratio does not fit in 64 bits",
            ));
        };
        Ok(Clock {
            parent,
            mul,
            div,
            span: expr.span(),
        })
    }

    /// The (parent, scale) pair of a clock expression. `None` parent means the
    /// expression is a pure number so far.
    fn clock_parts(
        &self,
        expr: &Expr,
        scope: &str,
    ) -> Result<(Option<ClockParent>, Rational), Diagnostic> {
        match expr {
            Expr::Path(p) => {
                // `cpu.clock` reads better than `cpu` when the intent is "the
                // same domain as the CPU", so both are accepted.
                let mut segments: Vec<&str> = Vec::new();
                for seg in &p.segments {
                    segments.push(seg.as_literal().unwrap_or_default());
                }
                if segments.last() == Some(&"clock") && segments.len() > 1 {
                    segments.pop();
                }
                let name = segments.join(".");
                match self.lookup(scope, &name) {
                    Some((_, Decl::Osc(id))) => {
                        Ok((Some(ClockParent::Osc(id)), Rational::from_int(1)))
                    }
                    Some((_, Decl::Object(id))) => {
                        Ok((Some(ClockParent::Object(id)), Rational::from_int(1)))
                    }
                    Some((full, Decl::Space(_))) => Err(Diagnostic::new(
                        p.span,
                        format!("`{full}` is an address space, not a clock"),
                    )),
                    None => {
                        let known = list(
                            self.names_of(|d| matches!(d, Decl::Osc(_) | Decl::Object(_)))
                                .into_iter(),
                        );
                        Err(Diagnostic::new(
                            p.span,
                            format!("no oscillator or object named `{name}`; in scope are {known}"),
                        ))
                    }
                }
            }
            Expr::Num(_) | Expr::Unary { .. } => Ok((None, expr.eval_rational()?)),
            Expr::Binary { op, lhs, rhs, span } => {
                let (lp, lr) = self.clock_parts(lhs, scope)?;
                let (rp, rr) = self.clock_parts(rhs, scope)?;
                if lp.is_some() && rp.is_some() {
                    return Err(Diagnostic::new(
                        *span,
                        "a clock may reference only one oscillator or domain",
                    ));
                }
                let parent = lp.or(rp);
                let scale = match op {
                    BinOp::Mul => lr.checked_mul(rr),
                    BinOp::Div => {
                        if rr == Rational::ZERO {
                            return Err(Diagnostic::new(rhs.span(), "division by zero"));
                        }
                        if rp.is_some() {
                            return Err(Diagnostic::new(
                                *span,
                                "a clock cannot be divided *by* another domain; write \
                                 `parent / 12`",
                            ));
                        }
                        lr.checked_div(rr)
                    }
                    // `master + 1` has no meaning: a domain is a ratio of its
                    // parent, not an offset from it.
                    BinOp::Add | BinOp::Sub | BinOp::Rem if parent.is_none() => {
                        return Ok((None, expr.eval_rational()?));
                    }
                    other => {
                        return Err(Diagnostic::new(
                            *span,
                            format!(
                                "`{}` cannot be applied to a clock; a domain is its parent \
                                 multiplied and divided by whole numbers",
                                other.as_str()
                            ),
                        ));
                    }
                };
                match scale {
                    Some(scale) => Ok((parent, scale)),
                    None => Err(Diagnostic::new(*span, "this clock ratio is out of range")),
                }
            }
            other => Err(Diagnostic::new(
                other.span(),
                "a `clock` must be a declared oscillator or domain, optionally scaled",
            )),
        }
    }

    /// The oscillator forest must be a forest (`ROADMAP.md` §4.2).
    fn check_clock_cycles(&self, out: &Resolved) -> Result<(), Diagnostic> {
        // 0 = unvisited, 1 = on the stack, 2 = done.
        let mut state = alloc::vec![0u8; out.objects.len()];
        for start in 0..out.objects.len() {
            if state[start] != 0 {
                continue;
            }
            let mut path: Vec<usize> = Vec::new();
            let mut at = start;
            // Every index below is bounds-checked: a resolved id always points
            // at an object, but a diagnostic pass must never be the thing that
            // panics.
            while let (Some(&mark), Some(object)) = (state.get(at), out.objects.get(at)) {
                if mark == 1 {
                    let from = path.iter().position(|i| *i == at).unwrap_or(0);
                    let mut names = String::new();
                    for i in path.iter().skip(from) {
                        names.push_str(&format!(
                            "`{}` → ",
                            out.objects.get(*i).map_or("?", |o| o.name.as_str())
                        ));
                    }
                    names.push_str(&format!("`{}`", object.name));
                    let head = path.get(from).and_then(|i| out.objects.get(*i));
                    let span = head.map_or(object.span, |o| o.clock.map_or(o.span, |c| c.span));
                    return Err(Diagnostic::new(
                        span,
                        format!("clock cycle: {names}; every domain must descend from an `osc`"),
                    ));
                }
                if mark == 2 {
                    break;
                }
                state[at] = 1;
                path.push(at);
                match object.clock.map(|c| c.parent) {
                    Some(ClockParent::Object(ObjectId(next))) => at = next as usize,
                    _ => break,
                }
            }
            for i in path {
                state[i] = 2;
            }
        }
        Ok(())
    }

    /// Evaluate a property block.
    fn props(&self, props: &[Property], scope: &str) -> Result<(Props, PropSpans), Diagnostic> {
        let mut out = Props::new();
        let mut spans = PropSpans::new();
        for p in props {
            let name = p.name.as_literal().unwrap_or_default().to_owned();
            if let Some(prev) = spans.get(&name) {
                return Err(Diagnostic::new(
                    p.name.span,
                    format!("property `{name}` is set twice"),
                )
                .with_note(prev, "first set here"));
            }
            out.insert(name.clone(), self.value(&p.value, scope)?);
            spans.insert(name, p.name.span);
        }
        Ok((out, spans))
    }

    /// Evaluate one expression to a [`Value`].
    fn value(&self, expr: &Expr, scope: &str) -> Result<Value, Diagnostic> {
        match expr {
            Expr::Num(n) => num_value(n),
            Expr::Str(s) => Ok(Value::Str(s.node.clone())),
            Expr::Bool(b) => Ok(Value::Bool(b.node)),
            Expr::Path(p) => self.path_value(p, scope),
            Expr::List { items, span } => {
                let mut out = Vec::with_capacity(items.len());
                for i in items {
                    out.push(self.value(i, scope)?);
                }
                let _ = span;
                Ok(Value::List(out))
            }
            Expr::Map { entries, .. } => Ok(Value::Map(self.props(entries, scope)?.0)),
            Expr::Unary {
                op: UnOp::Neg,
                operand,
                span,
            } => {
                let v = self.value(operand, scope)?;
                arith(BinOp::Sub, &Value::Int(0), &v, *span)
            }
            Expr::Binary { op, lhs, rhs, span } => {
                let a = self.value(lhs, scope)?;
                let b = self.value(rhs, scope)?;
                arith(*op, &a, &b, *span)
            }
            Expr::Call { callee, span, .. } => Err(Diagnostic::new(
                *span,
                format!(
                    "`{}(…)` is only meaningful as a `map` target",
                    callee.as_literal().unwrap_or_default()
                ),
            )),
        }
    }

    /// The bare-identifier rule: a declared name is a link, anything else is a
    /// string, and a name that is both a parameter and a declaration is an
    /// error rather than a guess.
    fn path_value(&self, path: &Path, scope: &str) -> Result<Value, Diagnostic> {
        let Some(text) = path.as_literal() else {
            return Err(Diagnostic::new(
                path.span,
                "this name was never substituted",
            ));
        };
        if path.segments.len() == 1 {
            // Bindings never reach here: substitution replaced them, and a
            // name that is both a binding and a declaration was rejected when
            // the tables were built.
            return match self.lookup(scope, &text) {
                Some((full, _)) => link(&full, path.span),
                // Not a declaration and not a binding: an enumeration keyword
                // such as `open-bus`, whose spelling the validator checks
                // against the property's own list.
                None => Ok(Value::Str(text)),
            };
        }
        // A dotted name is always a link; no binding can be dotted.
        let (object, rest) = self.object_path(path, scope, false)?;
        let name = self
            .decl_order
            .iter()
            .find(|(_, d, _)| *d == Decl::Object(object))
            .map(|(n, _, _)| n.clone())
            .unwrap_or(text);
        match rest {
            Some(rest) => link(&format!("{name}.{rest}"), path.span),
            None => link(&name, path.span),
        }
    }
}

/// Wrap a resolved name as a [`Value::Link`].
fn link(name: &str, span: Span) -> Result<Value, Diagnostic> {
    Link::new(name)
        .map(Value::Link)
        .map_err(|e| Diagnostic::new(span, e.to_string()))
}

/// `an oscillator` / `a space`, for a sentence.
fn article(what: &str) -> String {
    let first = what.chars().next().unwrap_or('x');
    if matches!(first, 'a' | 'e' | 'i' | 'o' | 'u') {
        format!("an {what}")
    } else {
        format!("a {what}")
    }
}

/// A literal name, or a diagnostic — used where a substitution makes no sense.
fn literal_name(name: &Name, what: &str) -> Result<String, Diagnostic> {
    match name.as_literal() {
        Some(text) => Ok(text.to_owned()),
        None => Err(Diagnostic::new(
            name.span,
            format!("{what} cannot contain a `$` substitution"),
        )),
    }
}

/// `` `a`, `b`, `c` `` — the candidate list every "what was in scope" message
/// ends with.
fn list<'i>(names: impl Iterator<Item = &'i str>) -> String {
    let mut out = String::new();
    let mut count = 0usize;
    for name in names {
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

/// A literal integer expression, for a loop variable's binding.
fn int_expr(value: i64, span: Span) -> Expr {
    if value < 0 {
        return Expr::Unary {
            op: UnOp::Neg,
            operand: Box::new(int_expr(value.saturating_neg(), span)),
            span,
        };
    }
    let digits = value.unsigned_abs();
    Expr::Num(Spanned::new(
        NumLit {
            value: digits,
            digits,
            radix: Radix::Dec,
            unit: NumUnit::None,
        },
        span,
    ))
}

/// A literal expression for a [`Value`], for a command-line override.
///
/// Scalars only: `-p` carries text, and a list or a map is not something a
/// shell word can honestly be.
fn value_expr(value: &Value, span: Span) -> Option<Expr> {
    let lit = |value: u64, unit: NumUnit| {
        Some(Expr::Num(Spanned::new(
            NumLit {
                value,
                digits: value,
                radix: Radix::Dec,
                unit,
            },
            span,
        )))
    };
    match value {
        Value::Bool(b) => Some(Expr::Bool(Spanned::new(*b, span))),
        Value::Str(s) => Some(Expr::Str(Spanned::new(s.clone(), span))),
        Value::Uint(u) => lit(*u, NumUnit::None),
        Value::Addr(a) => lit(*a, NumUnit::None),
        Value::Size(n) => lit(*n, NumUnit::Size(crate::machine::lexer::SizeUnit::Byte)),
        Value::Int(i) if *i >= 0 => lit(i.unsigned_abs(), NumUnit::None),
        Value::Int(i) => Some(Expr::Unary {
            op: UnOp::Neg,
            operand: Box::new(lit(i.unsigned_abs(), NumUnit::None)?),
            span,
        }),
        // A duration is stored in picoseconds and written in nanoseconds, so
        // anything finer than a nanosecond cannot round-trip through the DSL.
        Value::Duration(d) => {
            let picos = d.as_picos();
            if picos % 1_000 != 0 {
                return None;
            }
            lit(
                picos / 1_000,
                NumUnit::Duration(crate::machine::lexer::DurationUnit::Nanos),
            )
        }
        // Media is bound at realize time and has no literal form at all, so
        // it can never come back out as one.
        Value::List(_) | Value::Map(_) | Value::Link(_) | Value::Media(_) => None,
    }
}

/// A numeric literal's value, kinded by the suffix the user wrote.
///
/// A hexadecimal number with no suffix is an address: that is what `0x2000`
/// means in every machine file ever written, and it is the distinction
/// `rsemu describe` prints back (§4.4).
fn num_value(lit: &Spanned<NumLit>) -> Result<Value, Diagnostic> {
    Ok(match lit.node.unit {
        NumUnit::Size(_) => Value::Size(lit.node.value),
        NumUnit::Duration(_) => Value::Duration(
            Duration::from_nanos(lit.node.value)
                .ok_or_else(|| Diagnostic::new(lit.span, "this duration is out of range"))?,
        ),
        NumUnit::None if lit.node.radix == Radix::Hex => Value::Addr(lit.node.value),
        NumUnit::None => Value::Uint(lit.node.value),
    })
}

/// Arithmetic over [`Value`]s, keeping the most specific kind.
///
/// `4M / 2` is a size, `0x1000 + 0x20` is an address, and `2 + 3` is a plain
/// number. Mixing a duration with a size is refused rather than silently
/// producing one of them.
fn arith(op: BinOp, a: &Value, b: &Value, span: Span) -> Result<Value, Diagnostic> {
    let (Some(x), Some(y)) = (numeric(a), numeric(b)) else {
        return Err(Diagnostic::new(
            span,
            format!(
                "`{}` needs numbers on both sides, but found {} and {}",
                op.as_str(),
                a.kind(),
                b.kind()
            ),
        ));
    };
    let (l, r) = (kind_rank(a).unwrap_or(0), kind_rank(b).unwrap_or(0));
    // A size added to an address is ordinary address arithmetic; a duration
    // added to either is a category error, and silently picking one of the two
    // units is exactly the kind of guess §5's error messages exist to avoid.
    if (l == 3) != (r == 3) && l.min(r) > 0 {
        return Err(Diagnostic::new(
            span,
            format!(
                "`{}` cannot combine {} and {}",
                op.as_str(),
                a.kind(),
                b.kind()
            ),
        ));
    }
    let kind = l.max(r);
    let out = match op {
        BinOp::Add => x.checked_add(y),
        BinOp::Sub => x.checked_sub(y),
        BinOp::Mul => x.checked_mul(y),
        BinOp::Div => {
            if y == 0 {
                return Err(Diagnostic::new(span, "division by zero"));
            }
            x.checked_div(y)
        }
        BinOp::Rem => {
            if y == 0 {
                return Err(Diagnostic::new(span, "division by zero"));
            }
            x.checked_rem(y)
        }
    };
    let Some(out) = out else {
        return Err(Diagnostic::new(span, "this value is out of range"));
    };
    match kind {
        0 if out < 0 => i64::try_from(out)
            .map(Value::Int)
            .map_err(|_| Diagnostic::new(span, "this value is out of range")),
        0 => u64::try_from(out)
            .map(Value::Uint)
            .map_err(|_| Diagnostic::new(span, "this value is out of range")),
        rank => {
            let Ok(n) = u64::try_from(out) else {
                return Err(Diagnostic::new(
                    span,
                    format!("a {} cannot be negative or out of range", a.kind()),
                ));
            };
            Ok(match rank {
                1 => Value::Addr(n),
                2 => Value::Size(n),
                _ => Value::Duration(
                    Duration::from_nanos(n)
                        .ok_or_else(|| Diagnostic::new(span, "this duration is out of range"))?,
                ),
            })
        }
    }
}

/// A value as an integer for arithmetic; durations count in nanoseconds so the
/// result can be written back as one.
fn numeric(v: &Value) -> Option<i128> {
    match v {
        Value::Int(i) => Some(i128::from(*i)),
        Value::Uint(u) | Value::Size(u) | Value::Addr(u) => Some(i128::from(*u)),
        Value::Duration(d) => Some(i128::from(d.as_nanos())),
        _ => None,
    }
}

/// How specific a numeric kind is: a plain number yields to an address, which
/// yields to a size, which yields to a duration.
fn kind_rank(v: &Value) -> Option<u8> {
    Some(match v {
        Value::Int(_) | Value::Uint(_) => 0,
        Value::Addr(_) => 1,
        Value::Size(_) => 2,
        Value::Duration(_) => 3,
        _ => return None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::machine::sources::{MemoryLoader, NoIncludes};

    /// Resolve one file with no includes, returning the machine.
    fn machine(text: &str) -> Resolved {
        let mut map = SourceMap::new();
        let root = map.add("m.machine", text).expect("fits");
        match resolve(&mut map, root, &mut NoIncludes, &ResolveOptions::new()) {
            Ok(m) => m,
            Err(d) => panic!("{}", map.render(&d)),
        }
    }

    /// Resolve one file and render the diagnostic it must produce.
    fn error(text: &str) -> String {
        let mut map = SourceMap::new();
        let root = map.add("m.machine", text).expect("fits");
        let diag = resolve(&mut map, root, &mut NoIncludes, &ResolveOptions::new())
            .expect_err("should fail");
        map.render(&diag)
    }

    /// Resolve a file plus a search path, returning the machine.
    fn with_includes(text: &str, loader: &mut MemoryLoader, options: &ResolveOptions) -> Resolved {
        let mut map = SourceMap::new();
        let root = map.add("m.machine", text).expect("fits");
        match resolve(&mut map, root, loader, options) {
            Ok(m) => m,
            Err(d) => panic!("{}", map.render(&d)),
        }
    }

    /// The same, rendering the diagnostic instead.
    fn include_error(text: &str, loader: &mut MemoryLoader) -> String {
        let mut map = SourceMap::new();
        let root = map.add("m.machine", text).expect("fits");
        let diag =
            resolve(&mut map, root, loader, &ResolveOptions::new()).expect_err("should fail");
        map.render(&diag)
    }

    /// §5's worked example, with the cartridge its `wire cart.irq` implies.
    const NES: &str = r#"machine "nes" {
  param region = "ntsc"

  osc master = 236250000/11 Hz

  space cpubus  { width = 16, unassigned = open-bus }
  space ppubus  { width = 14, unassigned = open-bus }

  object ram "wram" { size = 2K }

  object cpu "mos6502" {
    clock  = master / 12
    space  = cpubus
    engine = "interp"
  }
  object ppu "nes.ppu" { clock = master / 4, space = ppubus }
  object apu "nes.apu" { clock = master / 12 }
  object cart "nes.cart" { }

  map cpubus 0x0000 size 0x2000 = mirror(ram)
  map cpubus 0x2000 size 0x2000 = mirror(ppu.regs)
  map cpubus 0x4000 size 0x0020 = apu.regs

  wire ppu.nmi   -> cpu.nmi
  wire apu.irq   -> cpu.irq
  wire cart.irq  -> cpu.irq
}
"#;

    #[test]
    fn the_nes_example_resolves_into_the_graph_it_describes() {
        let m = machine(NES);
        assert_eq!(m.name, "nes");
        assert_eq!(
            m.params.get("region"),
            Some(&Value::Str("ntsc".to_string()))
        );

        let names: Vec<&str> = m.objects.iter().map(|o| o.name.as_str()).collect();
        assert_eq!(names, ["ram", "cpu", "ppu", "apu", "cart"]);

        // The frequency stays rational; the ratios are integers and exact.
        assert_eq!(m.oscillators[0].hz.numerator(), 236_250_000);
        assert_eq!(m.oscillators[0].hz.denominator(), 11);
        let cpu = m.objects[1].clock.expect("cpu has a clock");
        let ppu = m.objects[2].clock.expect("ppu has a clock");
        assert_eq!((cpu.mul, cpu.div), (1, 12));
        assert_eq!((ppu.mul, ppu.div), (1, 4));
        assert_eq!(cpu.parent, ClockParent::Osc(OscId(0)));
        // 3 PPU dots per CPU cycle, by construction (ROADMAP.md §4.2).
        assert_eq!(cpu.div / ppu.div, 3);

        // `space = cpubus` is a link; `unassigned = open-bus` is a keyword.
        assert_eq!(m.objects[1].space, Some(SpaceId(0)));
        assert_eq!(
            m.spaces[0].props.get("unassigned"),
            Some(&Value::Str("open-bus".to_string()))
        );
        assert_eq!(m.spaces[0].props.get("width"), Some(&Value::Uint(16)));
        // Structural properties never reach the device.
        assert!(!m.objects[1].props.contains("clock"));
        assert!(!m.objects[1].props.contains("space"));
        assert_eq!(
            m.objects[1].props.get("engine"),
            Some(&Value::Str("interp".to_string()))
        );

        // `map cpubus 0x2000 size 0x2000 = mirror(ppu.regs)`
        assert_eq!(m.maps[1].base, 0x2000);
        assert_eq!(m.maps[1].size, 0x2000);
        let MapTarget::Mirror { inner, .. } = &m.maps[1].target else {
            panic!("expected a mirror");
        };
        assert_eq!(
            **inner,
            MapTarget::Region {
                object: ObjectId(2),
                region: Some("regs".to_string()),
                span: inner.span(),
            }
        );

        // Two sources on one sink: wired-OR, kept as two edges (§4.3).
        assert_eq!(m.wires.len(), 3);
        assert_eq!(m.fan_in(ObjectId(1), "irq"), 2);
        assert_eq!(m.wires[2].from.object, ObjectId(4));
        assert_eq!(m.wires[2].from.port, "irq");
    }

    #[test]
    fn a_split_map_target_resolves_both_sides() {
        let m = machine(
            "machine \"m\" {\n  \
             space cpubus { width = 16 }\n  \
             object a \"nes.input\" { }\n  \
             object b \"nes.apu\" { }\n  \
             map cpubus 0x4017 size 1 = split(a.port2, b.frame)\n\
             }\n",
        );
        let MapTarget::Split { reads, writes, .. } = &m.maps[0].target else {
            panic!("expected a split, got {:?}", m.maps[0].target);
        };
        assert!(matches!(**reads, MapTarget::Region { .. }), "{reads:?}");
        assert!(matches!(**writes, MapTarget::Region { .. }), "{writes:?}");
        // And a nonsense arity is a diagnostic, not a panic.
        let e = error(
            "machine \"m\" {\n  \
             space cpubus { width = 16 }\n  \
             object a \"nes.input\" { }\n  \
             map cpubus 0x4017 size 1 = split(a.port2)\n\
             }\n",
        );
        assert!(e.contains("the read side and the write side"), "{e}");
    }

    #[test]
    fn resolution_is_deterministic() {
        assert_eq!(machine(NES), machine(NES));
    }

    // -- params ------------------------------------------------------------

    #[test]
    fn a_parameter_default_can_be_overridden_from_outside() {
        const TEXT: &str =
            "machine \"m\" {\n  param ram = 4M\n  object r \"ram\" { size = ram }\n}\n";
        assert_eq!(
            machine(TEXT).objects[0].props.get("size"),
            Some(&Value::Size(4 << 20))
        );

        let mut map = SourceMap::new();
        let root = map.add("m.machine", TEXT).expect("fits");
        let opts = ResolveOptions::new().with_param("ram", "8G");
        let m = resolve(&mut map, root, &mut NoIncludes, &opts)
            .unwrap_or_else(|d| panic!("{}", map.render(&d)));
        assert_eq!(m.objects[0].props.get("size"), Some(&Value::Size(8 << 30)));
        assert_eq!(m.params.get("ram"), Some(&Value::Size(8 << 30)));
    }

    #[test]
    fn a_parameter_takes_part_in_arithmetic() {
        let m = machine(
            "machine \"m\" {\n  param ram = 4M\n  object r \"ram\" { size = ram / 2 + 1K }\n}\n",
        );
        assert_eq!(
            m.objects[0].props.get("size"),
            Some(&Value::Size((2 << 20) + 1024))
        );
    }

    #[test]
    fn golden_parameter_without_a_default() {
        assert_eq!(
            error("machine \"m\" {\n  param ram\n  object r \"ram\" { size = ram }\n}\n"),
            "\
error: parameter `ram` has no default and was not given a value (pass `-p ram=…`)
 --> m.machine:2:3
  |
2 |   param ram
  |   ^^^^^^^^^"
        );
    }

    #[test]
    fn golden_override_of_a_parameter_that_does_not_exist() {
        let mut map = SourceMap::new();
        let root = map
            .add("m.machine", "machine \"m\" {\n  param ram = 4M\n}\n")
            .expect("fits");
        let opts = ResolveOptions::new().with_param("rma", "8G");
        let diag = resolve(&mut map, root, &mut NoIncludes, &opts).expect_err("typo");
        assert_eq!(
            map.render(&diag),
            "\
error: no parameter named `rma`; this machine declares `ram`
 --> m.machine:2:3
  |
2 |   param ram = 4M
  |   ^^^^^^^^^^^^^^"
        );
    }

    #[test]
    fn golden_a_parameter_and_an_object_may_not_share_a_name() {
        assert_eq!(
            error("machine \"m\" {\n  param ram = 4M\n  object ram \"ram\" { }\n}\n"),
            "\
error: `ram` is ambiguous: it is declared both as a parameter and as an object `ram`
 --> m.machine:3:10
  |
3 |   object ram \"ram\" { }
  |          ^^^

note: the parameter is declared here
 --> m.machine:2:9
  |
2 |   param ram = 4M
  |         ^^^"
        );
    }

    // -- includes ----------------------------------------------------------

    #[test]
    fn an_include_is_spliced_in_place_and_only_once() {
        let mut loader = MemoryLoader::new()
            .with("common.machine", "object shared \"ram\" { size = 1K }\n")
            .with("a.machine", "include \"common.machine\"\n")
            .with("b.machine", "include \"common.machine\"\n");
        let m = with_includes(
            "include \"a.machine\"\ninclude \"b.machine\"\nmachine \"m\" { }\n",
            &mut loader,
            &ResolveOptions::new(),
        );
        // Include-once: two paths to one fragment splice it once.
        assert_eq!(m.objects.len(), 1);
        assert_eq!(m.objects[0].name, "shared");
    }

    #[test]
    fn golden_include_cycle_names_the_whole_cycle() {
        let mut loader = MemoryLoader::new()
            .with("a.machine", "include \"b.machine\"\n")
            .with("b.machine", "include \"a.machine\"\n");
        assert_eq!(
            include_error("include \"a.machine\"\nmachine \"m\" { }\n", &mut loader),
            "\
error: include cycle: `a.machine` → `b.machine` → `a.machine`
 --> b.machine:1:9
  |
1 | include \"a.machine\"
  |         ^^^^^^^^^^^

note: the cycle starts here
 --> m.machine:1:1
  |
1 | include \"a.machine\"
  | ^^^^^^^^^^^^^^^^^^^"
        );
    }

    #[test]
    fn golden_a_missing_include_reports_the_search_path() {
        let mut loader = MemoryLoader::new().with("pci.machine", "");
        assert_eq!(
            include_error("include \"pcie.machine\"\nmachine \"m\" { }\n", &mut loader),
            "\
error: no file named `pcie.machine`; the search path holds `pci.machine`
 --> m.machine:1:9
  |
1 | include \"pcie.machine\"
  |         ^^^^^^^^^^^^^^"
        );
    }

    #[test]
    fn an_error_in_an_included_file_points_into_that_file() {
        let mut loader = MemoryLoader::new().with("frag.machine", "object a \"x\" {\n  size = 1\n");
        let rendered = include_error("include \"frag.machine\"\nmachine \"m\" { }\n", &mut loader);
        assert!(rendered.contains("frag.machine:3:1"), "{rendered}");
    }

    // -- templates and loops ------------------------------------------------

    /// The phase-2 gate (§13): a template instantiated four times inside a
    /// loop, from an included file, with a parameter override.
    #[test]
    fn the_phase_two_gate_fixture_resolves() {
        let mut loader = MemoryLoader::new().with(
            "pci-common.machine",
            "template cpu_complex(id, clock, l2 = 512K) {\n  \
               object cpu$id \"riscv64\" { clock = clock, space = mem }\n  \
               object l2$id \"cache\" { size = l2 }\n  \
               wire cpu$id.irq -> plic.in$id\n\
             }\n",
        );
        let text = "include \"pci-common.machine\"\n\
                    param cores = 4\n\
                    param ram = 4M\n\
                    machine \"quad\" {\n  \
                      osc master = 1 GHz\n  \
                      space mem { width = 32 }\n  \
                      object plic \"riscv.plic\" { }\n  \
                      for i in 0..4 {\n    \
                        instance core$i = cpu_complex(id = i, clock = master / (i + 1))\n  \
                      }\n  \
                      for j in 0..=1 { object bank${j * 2} \"ram\" { size = ram / 2 } }\n\
                    }\n";
        let opts = ResolveOptions::new().with_param("ram", "8M");
        let m = with_includes(text, &mut loader, &opts);

        let names: Vec<&str> = m.objects.iter().map(|o| o.name.as_str()).collect();
        assert_eq!(
            names,
            [
                "plic",
                "core0.cpu0",
                "core0.l20",
                "core1.cpu1",
                "core1.l21",
                "core2.cpu2",
                "core2.l22",
                "core3.cpu3",
                "core3.l23",
                "bank0",
                "bank2",
            ]
        );
        // The template's `clock` argument arrived as an expression, not a
        // number, so it still resolves to the oscillator.
        let core2 = m.object_named("core2.cpu2").expect("declared").1;
        assert_eq!(
            core2.clock,
            Some(Clock {
                parent: ClockParent::Osc(OscId(0)),
                mul: 1,
                div: 3,
                span: core2.clock.expect("clock").span,
            })
        );
        // The default `l2 = 512K` applies where no argument was given.
        assert_eq!(
            m.object_named("core3.l23")
                .expect("declared")
                .1
                .props
                .get("size"),
            Some(&Value::Size(512 * 1024))
        );
        // `${j * 2}` computes a name; `ram / 2` uses the override.
        assert_eq!(
            m.object_named("bank2")
                .expect("declared")
                .1
                .props
                .get("size"),
            Some(&Value::Size(4 << 20))
        );
        // A wire out of the template reaches the machine-level `plic`, and its
        // source is the instance's own CPU.
        assert_eq!(m.wires.len(), 4);
        assert_eq!(m.wires[3].to.object, ObjectId(0));
        assert_eq!(m.wires[3].to.port, "in3");
        assert_eq!(
            m.objects[m.wires[3].from.object.0 as usize].name,
            "core3.cpu3"
        );
    }

    #[test]
    fn an_inner_declaration_shadows_an_outer_one_of_the_same_name() {
        let mut loader = MemoryLoader::new();
        let m = with_includes(
            "template t() {\n  object plic \"local\" { }\n  wire plic.out -> plic.in\n}\n\
             machine \"m\" {\n  object plic \"outer\" { }\n  instance c0 = t()\n}\n",
            &mut loader,
            &ResolveOptions::new(),
        );
        // Both endpoints find `c0.plic`, not the machine-level one.
        assert_eq!(m.objects[m.wires[0].from.object.0 as usize].name, "c0.plic");
        assert_eq!(m.objects[m.wires[0].to.object.0 as usize].name, "c0.plic");
    }

    #[test]
    fn golden_template_recursion_names_the_cycle() {
        assert_eq!(
            error(
                "template a() { instance x = b() }\n\
                 template b() { instance y = a() }\n\
                 machine \"m\" { instance z = a() }\n"
            ),
            "\
error: template recursion: `a` → `b` → `a`
 --> m.machine:2:29
  |
2 | template b() { instance y = a() }
  |                             ^

note: the cycle starts here
 --> m.machine:3:28
  |
3 | machine \"m\" { instance z = a() }
  |                            ^"
        );
    }

    #[test]
    fn golden_a_loop_that_would_never_finish() {
        assert_eq!(
            error("machine \"m\" {\n  for i in 0..100000 { object a$i \"x\" { } }\n}\n"),
            "\
error: this loop runs 100000 times; the limit is 4096
 --> m.machine:2:12
  |
2 |   for i in 0..100000 { object a$i \"x\" { } }
  |            ^^^^^^^^^"
        );
    }

    #[test]
    fn an_empty_range_declares_nothing() {
        let m = machine("machine \"m\" {\n  for i in 0..0 { object a$i \"x\" { } }\n}\n");
        assert!(m.objects.is_empty());
    }

    // -- links and cycles ---------------------------------------------------

    #[test]
    fn golden_an_unresolved_wire_endpoint_says_what_was_in_scope() {
        assert_eq!(
            error(
                "machine \"m\" {\n  \
                   object cpu \"mos6502\" { }\n  \
                   object ppu \"nes.ppu\" { }\n  \
                   wire pppu.nmi -> cpu.nmi\n\
                 }\n"
            ),
            "\
error: no object named `pppu`; objects in scope are `cpu`, `ppu`
 --> m.machine:4:8
  |
4 |   wire pppu.nmi -> cpu.nmi
  |        ^^^^^^^^"
        );
    }

    #[test]
    fn golden_a_wire_endpoint_must_name_a_pin() {
        assert_eq!(
            error("machine \"m\" {\n  object cpu \"mos6502\" { }\n  wire cpu -> cpu\n}\n"),
            "\
error: a wire endpoint must name a pin, as in `cpu.nmi`
 --> m.machine:3:8
  |
3 |   wire cpu -> cpu
  |        ^^^"
        );
    }

    #[test]
    fn golden_a_map_target_that_does_not_exist() {
        assert_eq!(
            error(
                "machine \"m\" {\n  \
                   space cpubus { width = 16 }\n  \
                   object ram \"ram\" { }\n  \
                   map cpubus 0 size 0x100 = mirror(rma)\n\
                 }\n"
            ),
            "\
error: no object named `rma`; objects in scope are `ram`
 --> m.machine:4:36
  |
4 |   map cpubus 0 size 0x100 = mirror(rma)
  |                                    ^^^"
        );
    }

    #[test]
    fn golden_a_map_into_a_space_that_does_not_exist() {
        assert_eq!(
            error(
                "machine \"m\" {\n  \
                   space cpubus { width = 16 }\n  \
                   object ram \"ram\" { }\n  \
                   map ppubus 0 size 0x100 = ram\n\
                 }\n"
            ),
            "\
error: no address space named `ppubus`; declared spaces are `cpubus`
 --> m.machine:4:7
  |
4 |   map ppubus 0 size 0x100 = ram
  |       ^^^^^^"
        );
    }

    #[test]
    fn golden_a_link_that_names_the_wrong_kind_of_thing() {
        assert_eq!(
            error(
                "machine \"m\" {\n  \
                   osc master = 1 MHz\n  \
                   object cpu \"mos6502\" { space = master }\n\
                 }\n"
            ),
            "\
error: `master` is an oscillator `master`, not an address space
 --> m.machine:3:34
  |
3 |   object cpu \"mos6502\" { space = master }
  |                                  ^^^^^^

note: declared here
 --> m.machine:2:7
  |
2 |   osc master = 1 MHz
  |       ^^^^^^"
        );
    }

    #[test]
    fn golden_a_duplicate_declaration() {
        assert_eq!(
            error(
                "machine \"m\" {\n  object cpu \"mos6502\" { }\n  object cpu \"riscv64\" { }\n}\n"
            ),
            "\
error: `cpu` is declared twice, as object and as object
 --> m.machine:3:10
  |
3 |   object cpu \"riscv64\" { }
  |          ^^^

note: first declared here
 --> m.machine:2:10
  |
2 |   object cpu \"mos6502\" { }
  |          ^^^"
        );
    }

    #[test]
    fn golden_a_clock_that_names_no_oscillator() {
        assert_eq!(
            error("machine \"m\" {\n  object cpu \"mos6502\" { clock = 5000000 }\n}\n"),
            "\
error: a `clock` must be derived from a declared oscillator, as in `master / 12`; a bare frequency needs an `osc` declaration of its own
 --> m.machine:2:34
  |
2 |   object cpu \"mos6502\" { clock = 5000000 }
  |                                  ^^^^^^^"
        );
    }

    #[test]
    fn golden_a_clock_cycle() {
        assert_eq!(
            error(
                "machine \"m\" {\n  \
                   object a \"x\" { clock = b / 2 }\n  \
                   object b \"x\" { clock = a / 2 }\n\
                 }\n"
            ),
            "\
error: clock cycle: `a` → `b` → `a`; every domain must descend from an `osc`
 --> m.machine:2:26
  |
2 |   object a \"x\" { clock = b / 2 }
  |                          ^^^^^"
        );
    }

    #[test]
    fn a_clock_may_hang_from_another_object() {
        let m = machine(
            "machine \"m\" {\n  \
               osc master = 1 MHz\n  \
               object cpu \"x\" { clock = master / 12 }\n  \
               object dma \"y\" { clock = cpu.clock / 2 }\n\
             }\n",
        );
        assert_eq!(
            m.objects[1].clock.map(|c| (c.parent, c.mul, c.div)),
            Some((ClockParent::Object(ObjectId(0)), 1, 2))
        );
    }

    // -- robustness ---------------------------------------------------------

    #[test]
    fn adversarial_shapes_produce_a_diagnostic_rather_than_a_panic() {
        for text in [
            "",
            "machine \"m\" { }",
            "param x = 1",
            "machine \"m\" { osc a = 0 Hz }",
            "machine \"m\" { object a \"c\" { clock = a } }",
            "machine \"m\" { space s { width = 16 } map s 0 size 0 = s }",
            "machine \"m\" { space s { width = 16 } object o \"c\" {} \
                             map s 0xffffffffffffffff size 0x10 = o }",
            "machine \"m\" { object a \"c\" { p = mirror(a) } }",
            "machine \"m\" { object a \"c\" { p = 1 % 0 } }",
            "machine \"m\" { object a \"c\" { p = \"s\" + 1 } }",
            "machine \"m\" { for i in 0..2 { param x = 1 } }",
            "machine \"m\" { for i in 0..2 { template t() { } } }",
            "machine \"m\" { instance a = nope() }",
            "template t(a) { } machine \"m\" { instance x = t() }",
            "template t(a) { } machine \"m\" { instance x = t(b = 1) }",
            "template t(a) { } machine \"m\" { instance x = t(1, 2) }",
            "template t(a) { } machine \"m\" { instance x = t(a = 1, a = 2) }",
            "machine \"m\" { object a$b \"c\" { } }",
            "machine \"m\" { for i in 0..2 { object a${i / 2} \"c\" { } } }",
            "machine \"a\" { } machine \"b\" { }",
            "machine \"m\" { object a \"c\" { } wire a.x -> a.y.z }",
            "machine \"m\" { object a \"c\" { } map a 0 size 1 = a }",
            "machine \"m\" { object a \"c\" { size = 1, size = 2 } }",
        ] {
            let mut map = SourceMap::new();
            let root = map.add("t.machine", text).expect("fits");
            if let Err(d) = resolve(&mut map, root, &mut NoIncludes, &ResolveOptions::new()) {
                let rendered = map.render(&d);
                assert!(rendered.starts_with("error: "), "{rendered}");
                assert!(rendered.contains(".machine:"), "{rendered}");
            }
        }
    }

    #[test]
    fn a_declaration_inside_a_loop_is_refused_where_it_is_written() {
        for (text, what) in [
            (
                "machine \"m\" {\n  for i in 0..2 { param x = 1 }\n}\n",
                "param",
            ),
            (
                "machine \"m\" {\n  for i in 0..2 { template t() { } }\n}\n",
                "template",
            ),
        ] {
            let rendered = error(text);
            assert!(
                rendered.contains(&format!(
                    "`{what}` must be declared at file scope or directly inside a `machine` block"
                )),
                "{rendered}"
            );
        }
    }

    #[test]
    fn a_loop_over_the_whole_of_i64_is_a_diagnostic_not_an_overflow() {
        let rendered = error(
            "machine \"m\" {\n  \
               for i in -9223372036854775808..9223372036854775807 { object a$i \"c\" { } }\n\
             }\n",
        );
        assert!(rendered.contains("the limit is 4096"), "{rendered}");
    }

    #[test]
    fn a_template_that_doubles_its_argument_hits_the_node_budget() {
        // Each level splices its argument twice, so `n` lines of file describe
        // 2ⁿ expression nodes. The file is small; the tree would not be.
        let mut text = String::from("template t0(a) { object x \"c\" { p = a } }\n");
        for level in 1..24 {
            text.push_str(&format!(
                "template t{level}(a) {{ instance i = t{} (a = a + a) }}\n",
                level - 1
            ));
        }
        text.push_str("machine \"m\" { instance top = t23(a = 1) }\n");
        let rendered = error(&text);
        assert!(
            rendered.contains("more than 1000000 expression nodes"),
            "{rendered}"
        );
    }

    #[test]
    fn deep_nesting_is_refused_rather_than_overflowing_the_stack() {
        let mut text = String::from("machine \"m\" {\n");
        for _ in 0..40 {
            text.push_str("for i in 0..1 {\n");
        }
        text.push_str("object a \"c\" { }\n");
        for _ in 0..41 {
            text.push_str("}\n");
        }
        let mut map = SourceMap::new();
        let root = map.add("t.machine", &text).expect("fits");
        let diag =
            resolve(&mut map, root, &mut NoIncludes, &ResolveOptions::new()).expect_err("too deep");
        assert!(map.render(&diag).contains("nest more than 32"));
    }

    #[test]
    fn scopes_search_outwards() {
        assert_eq!(scopes(""), alloc::vec![String::new()]);
        assert_eq!(
            scopes("a.b."),
            alloc::vec!["a.b.".to_string(), "a.".to_string(), String::new()]
        );
    }

    #[test]
    fn arithmetic_keeps_the_most_specific_kind() {
        let span = Span::at(0);
        let size = arith(BinOp::Div, &Value::Size(4096), &Value::Uint(2), span).expect("ok");
        assert_eq!(size, Value::Size(2048));
        let addr = arith(BinOp::Add, &Value::Addr(0x1000), &Value::Size(0x20), span).expect("ok");
        assert_eq!(addr, Value::Size(0x1020));
        let plain = arith(BinOp::Sub, &Value::Uint(1), &Value::Uint(4), span).expect("ok");
        assert_eq!(plain, Value::Int(-3));
        assert!(arith(BinOp::Div, &Value::Uint(1), &Value::Uint(0), span).is_err());
        assert!(
            arith(
                BinOp::Add,
                &Value::Size(1),
                &Value::Duration(Duration::from_picos(1000)),
                span
            )
            .is_err()
        );
    }
}
