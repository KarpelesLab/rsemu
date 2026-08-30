//! The abstract syntax tree of a `.machine` file.
//!
//! The tree is *syntactic*: it says what was written, not what it means. Names
//! are not resolved, `include`s are not read, `template`s are not expanded,
//! loops are not unrolled and device classes are not checked. Those are the
//! resolver's and validator's jobs (`ROADMAP.md` §5's pipeline), and this
//! module is deliberately ignorant of all of them.
//!
//! Two properties are load-bearing:
//!
//! * **Every node carries a [`Span`].** Any later stage can therefore produce a
//!   `file:line:col` and a caret for anything it objects to, and a printer can
//!   recover the exact spelling of a literal from the source.
//! * **Order is preserved everywhere.** Properties, statements and arguments
//!   are `Vec`s, never maps, so two runs over the same file produce the same
//!   tree and the same output (`CLAUDE.md`, determinism).
//!
//! # Seams left for later phases
//!
//! * [`IncludeStmt`] holds a path and nothing else: the search path and the
//!   loading belong to the caller, since the front end has no filesystem.
//! * [`TemplateDecl`], [`InstanceStmt`] and [`ForStmt`] are parsed and stored
//!   unexpanded. §13 names these as the three features most likely to be
//!   quietly deferred, so they are syntax from day one even though nothing
//!   instantiates them yet.
//! * [`Expr::eval_rational`] evaluates the literal-only subset. The general
//!   case needs parameter bindings, which the resolver owns.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::machine::diag::Diagnostic;
use crate::machine::lexer::NumLit;
use crate::machine::rational::Rational;
use crate::machine::span::{Span, Spanned};

/// A parsed file: a flat, ordered list of statements.
///
/// A file is not required to contain a `machine` block. An `include`d fragment
/// such as `pci-common.machine` is typically a list of `template`s and objects
/// that get spliced into the machine that included it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SourceUnit {
    /// Everything in the file, in source order.
    pub stmts: Vec<Stmt>,
    /// The whole file.
    pub span: Span,
}

/// One statement.
///
/// The language is a list of statements at every level, which is what makes
/// §5's "the graph must be readable by scanning the file" true: maps and wires
/// are statements, never properties buried inside an object.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Stmt {
    /// `machine "nes" { … }` — only legal at file scope.
    Machine(MachineDecl),
    /// `param region = "ntsc"`
    Param(ParamDecl),
    /// `osc master = 236250000/11 Hz`
    Osc(OscDecl),
    /// `space cpubus { width = 16 }`
    Space(SpaceDecl),
    /// `object cpu "mos6502" { … }`
    Object(ObjectDecl),
    /// `map cpubus 0x0000 size 0x2000 = mirror(wram)`
    Map(MapStmt),
    /// `wire ppu.nmi -> cpu.nmi`
    Wire(WireStmt),
    /// `include "pci-common.machine"`
    Include(IncludeStmt),
    /// `template cpu_complex(id, freq = 1 MHz) { … }`
    Template(TemplateDecl),
    /// `instance core0 = cpu_complex(id = 0)`
    Instance(InstanceStmt),
    /// `for i in 0..4 { … }`
    For(ForStmt),
}

impl Stmt {
    /// The span of the whole statement, keyword through closing brace.
    pub fn span(&self) -> Span {
        match self {
            Stmt::Machine(s) => s.span,
            Stmt::Param(s) => s.span,
            Stmt::Osc(s) => s.span,
            Stmt::Space(s) => s.span,
            Stmt::Object(s) => s.span,
            Stmt::Map(s) => s.span,
            Stmt::Wire(s) => s.span,
            Stmt::Include(s) => s.span,
            Stmt::Template(s) => s.span,
            Stmt::Instance(s) => s.span,
            Stmt::For(s) => s.span,
        }
    }
}

/// `machine "nes" { … }`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MachineDecl {
    /// The machine's name, as written in quotes.
    pub name: Spanned<String>,
    /// Its statements, in source order.
    pub body: Vec<Stmt>,
    /// The whole declaration.
    pub span: Span,
}

/// `param name = default` — a knob the CLI or environment can override
/// (`ROADMAP.md` §2: `rsemu run nes.machine -p ram=4M`).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParamDecl {
    /// The parameter name.
    pub name: Name,
    /// Its default. `None` means the caller must supply a value; the resolver
    /// is what enforces that.
    pub default: Option<Expr>,
    /// The whole declaration.
    pub span: Span,
}

/// The unit written after an oscillator's frequency.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FreqUnit {
    /// `Hz`
    Hz,
    /// `kHz`
    KHz,
    /// `MHz`
    MHz,
    /// `GHz`
    GHz,
}

impl FreqUnit {
    /// Hertz per unit.
    pub const fn scale(self) -> i128 {
        match self {
            FreqUnit::Hz => 1,
            FreqUnit::KHz => 1_000,
            FreqUnit::MHz => 1_000_000,
            FreqUnit::GHz => 1_000_000_000,
        }
    }

    /// The unit as written, for printing.
    pub const fn as_str(self) -> &'static str {
        match self {
            FreqUnit::Hz => "Hz",
            FreqUnit::KHz => "kHz",
            FreqUnit::MHz => "MHz",
            FreqUnit::GHz => "GHz",
        }
    }

    /// The unit for a spelling, or `None` if it is not one.
    ///
    /// `kHz` and `KHz` are both accepted; `HZ` and `hz` are not, because a
    /// closed set with a precise error beats silent case folding.
    pub fn from_spelling(s: &str) -> Option<FreqUnit> {
        Some(match s {
            "Hz" => FreqUnit::Hz,
            "kHz" | "KHz" => FreqUnit::KHz,
            "MHz" => FreqUnit::MHz,
            "GHz" => FreqUnit::GHz,
            _ => return None,
        })
    }
}

/// `osc master = 236250000/11 Hz`
///
/// The frequency is an expression rather than a number so that it can be
/// rational (§5's NES master clock is not an integer number of hertz) or
/// derived from a `param`. [`OscDecl::frequency_hz`] evaluates the literal
/// case exactly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OscDecl {
    /// The oscillator's name; other clocks divide it.
    pub name: Name,
    /// The frequency expression.
    pub freq: Expr,
    /// The unit. Required: `osc master = 236250000/11` with no unit is
    /// ambiguous about what a bare number means, and §5 always writes one.
    pub unit: Spanned<FreqUnit>,
    /// The whole declaration.
    pub span: Span,
}

impl OscDecl {
    /// The frequency in hertz, exactly, when it is written as literals.
    ///
    /// Returns a diagnostic if the expression names anything — those are the
    /// resolver's to evaluate once parameters are bound.
    pub fn frequency_hz(&self) -> Result<Rational, Diagnostic> {
        let base = self.freq.eval_rational()?;
        let scale = Rational::new(self.unit.node.scale(), 1)
            .ok_or_else(|| Diagnostic::new(self.unit.span, "frequency unit is out of range"))?;
        base.checked_mul(scale)
            .ok_or_else(|| Diagnostic::new(self.freq.span(), "frequency is out of range"))
    }
}

/// `space cpubus { width = 16, unassigned = open-bus }`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SpaceDecl {
    /// The address space's name.
    pub name: Name,
    /// Its properties, in source order.
    pub props: Vec<Property>,
    /// The whole declaration.
    pub span: Span,
}

/// `object cpu "mos6502" { clock = master / 12 }`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ObjectDecl {
    /// The instance name, which wires and maps refer to.
    pub name: Name,
    /// The device class, quoted because it is a registry key
    /// (`registry::create("pci.nvme", …)`, §4.4) rather than an identifier in
    /// this file. Whether the class exists is the validator's question.
    pub class: Spanned<String>,
    /// Its properties, in source order.
    pub props: Vec<Property>,
    /// The whole declaration.
    pub span: Span,
}

/// One `name = value` pair inside a block.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Property {
    /// The property name.
    pub name: Name,
    /// Its value.
    pub value: Expr,
    /// Name through value.
    pub span: Span,
}

/// `map cpubus 0x0000 size 0x2000 = mirror(wram)`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MapStmt {
    /// The address space being mapped into.
    pub space: Name,
    /// The base address.
    pub base: Expr,
    /// The size of the window.
    pub size: Expr,
    /// What is mapped there: an object, one of its regions, or a call such as
    /// `mirror(wram)`.
    pub target: Expr,
    /// Optional trailing block for per-mapping attributes — priority,
    /// endianness, and whatever §4.1 needs. Empty when none was written.
    pub props: Vec<Property>,
    /// The whole statement.
    pub span: Span,
}

/// `wire ppu.nmi -> cpu.nmi`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WireStmt {
    /// The source pin.
    pub from: Path,
    /// The destination pin. Several sources may name one destination; that is
    /// a wired-OR, and §5 says it is declared once per source.
    pub to: Path,
    /// The whole statement.
    pub span: Span,
}

/// `include "pci-common.machine"`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IncludeStmt {
    /// The path as written. Resolution against a search path, cycle detection
    /// and loading all belong to the caller.
    pub path: Spanned<String>,
    /// The whole statement.
    pub span: Span,
}

/// `template cpu_complex(id, clock = master / 12) { … }`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TemplateDecl {
    /// The template's name.
    pub name: Name,
    /// Its parameters, in declaration order.
    pub params: Vec<TemplateParam>,
    /// Its body, stored unexpanded.
    pub body: Vec<Stmt>,
    /// The whole declaration.
    pub span: Span,
}

/// One template parameter, with an optional default.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TemplateParam {
    /// The parameter name.
    pub name: Name,
    /// Its default value, if any.
    pub default: Option<Expr>,
    /// Name through default.
    pub span: Span,
}

/// `instance core0 = cpu_complex(id = 0, clock = master / 12)`
///
/// The names a template declares are expected to end up prefixed by the
/// instance name, which is why an instantiation is named rather than anonymous.
/// That prefixing is the resolver's rule to make; the parser only records what
/// was written.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InstanceStmt {
    /// The instance name, used as the namespace for what the template declares.
    pub name: Name,
    /// The template being instantiated.
    pub template: Name,
    /// Arguments, in source order.
    pub args: Vec<Arg>,
    /// The whole statement.
    pub span: Span,
}

/// One argument to a template instantiation: `id = 0`, or just `0`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Arg {
    /// The parameter being bound, or `None` for a positional argument.
    pub name: Option<Name>,
    /// The value.
    pub value: Expr,
    /// Name through value.
    pub span: Span,
}

/// `for i in 0..4 { object cpu$i "mos6502" { … } }`
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ForStmt {
    /// The loop variable, referred to as `i` in expressions and `$i` inside a
    /// name.
    pub var: Name,
    /// The first value.
    pub start: Expr,
    /// The bound.
    pub end: Expr,
    /// Whether `end` is included: `0..=3` rather than `0..4`.
    pub inclusive: bool,
    /// The body, stored unexpanded.
    pub body: Vec<Stmt>,
    /// The whole statement.
    pub span: Span,
}

/// A name, which may contain substitutions: `wram`, `cpu$i`, `bank${i + 1}`.
///
/// Indexed instantiation (§5) means a declaration's *name* is computed, so a
/// name is a small template rather than a string. The common case is a single
/// [`NamePart::Literal`]; [`Name::as_literal`] is the fast path for it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Name {
    /// The parts, in order.
    pub parts: Vec<NamePart>,
    /// The whole name.
    pub span: Span,
}

impl Name {
    /// The name as plain text, when it contains no substitution.
    pub fn as_literal(&self) -> Option<&str> {
        match self.parts.as_slice() {
            [NamePart::Literal(text)] => Some(text),
            _ => None,
        }
    }
}

/// One piece of a [`Name`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum NamePart {
    /// Literal text.
    Literal(String),
    /// `$i` or `${expr}` — substituted when the loop or template is expanded.
    Substitution(Expr),
}

/// A dotted reference: `cpu`, `ppu.nmi`, `apu.regs`.
///
/// The segments are not interpreted here. `ppu.regs` might be a region of an
/// object or a property of one; the resolver decides, and it needs the spans
/// this keeps.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Path {
    /// The segments, in order; never empty.
    pub segments: Vec<Name>,
    /// The whole path.
    pub span: Span,
}

impl Path {
    /// The path as plain text, when no segment contains a substitution.
    pub fn as_literal(&self) -> Option<String> {
        let mut out = String::new();
        for (i, seg) in self.segments.iter().enumerate() {
            if i > 0 {
                out.push('.');
            }
            out.push_str(seg.as_literal()?);
        }
        Some(out)
    }
}

/// A unary operator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UnOp {
    /// `-x`
    Neg,
}

/// A binary operator.
///
/// Arithmetic only. There is no comparison and no boolean logic: a machine
/// description declares a graph, and conditionals in a configuration language
/// are how configuration languages become bad programming languages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BinOp {
    /// `+`
    Add,
    /// `-`
    Sub,
    /// `*`
    Mul,
    /// `/` — also how a clock divider and a rational frequency are written.
    Div,
    /// `%`
    Rem,
}

impl BinOp {
    /// The operator as written.
    pub const fn as_str(self) -> &'static str {
        match self {
            BinOp::Add => "+",
            BinOp::Sub => "-",
            BinOp::Mul => "*",
            BinOp::Div => "/",
            BinOp::Rem => "%",
        }
    }
}

/// A value expression.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Expr {
    /// An integer, size, address or duration literal.
    Num(Spanned<NumLit>),
    /// A quoted string.
    Str(Spanned<String>),
    /// `true` or `false`.
    Bool(Spanned<bool>),
    /// A name or dotted reference: `master`, `cpubus`, `open-bus`, `ppu.regs`.
    Path(Path),
    /// `mirror(wram)` — a call, whose meaning the resolver supplies.
    Call {
        /// What is being called.
        callee: Path,
        /// Its arguments, in source order.
        args: Vec<Expr>,
        /// Callee through closing parenthesis.
        span: Span,
    },
    /// `-x`
    Unary {
        /// The operator.
        op: UnOp,
        /// What it applies to.
        operand: alloc::boxed::Box<Expr>,
        /// Operator through operand.
        span: Span,
    },
    /// `master / 12`
    Binary {
        /// The operator.
        op: BinOp,
        /// Left operand.
        lhs: alloc::boxed::Box<Expr>,
        /// Right operand.
        rhs: alloc::boxed::Box<Expr>,
        /// Left operand through right.
        span: Span,
    },
    /// `[1, 2, 3]`
    List {
        /// The elements, in order.
        items: Vec<Expr>,
        /// Brackets included.
        span: Span,
    },
    /// `{ a = 1, b = 2 }` — a map value, as distinct from a declaration block.
    Map {
        /// The entries, in source order.
        entries: Vec<Property>,
        /// Braces included.
        span: Span,
    },
}

impl Expr {
    /// Where the expression was written.
    pub fn span(&self) -> Span {
        match self {
            Expr::Num(n) => n.span,
            Expr::Str(s) => s.span,
            Expr::Bool(b) => b.span,
            Expr::Path(p) => p.span,
            Expr::Call { span, .. }
            | Expr::Unary { span, .. }
            | Expr::Binary { span, .. }
            | Expr::List { span, .. }
            | Expr::Map { span, .. } => *span,
        }
    }

    /// Evaluate the literal-only subset exactly.
    ///
    /// This is what makes `osc master = 236250000/11 Hz` a *rational* rather
    /// than a rounded number: the division is performed over [`Rational`], so
    /// no frequency is ever approximated. Anything naming a parameter, object
    /// or function is rejected — with a span — because binding those is the
    /// resolver's job, not the parser's.
    pub fn eval_rational(&self) -> Result<Rational, Diagnostic> {
        match self {
            Expr::Num(n) => Rational::new(i128::from(n.node.value), 1)
                .ok_or_else(|| Diagnostic::new(n.span, "number is out of range")),
            Expr::Unary { op, operand, span } => {
                let v = operand.eval_rational()?;
                match op {
                    UnOp::Neg => v
                        .checked_neg()
                        .ok_or_else(|| Diagnostic::new(*span, "value is out of range")),
                }
            }
            Expr::Binary { op, lhs, rhs, span } => {
                let a = lhs.eval_rational()?;
                let b = rhs.eval_rational()?;
                let out = match op {
                    BinOp::Add => a.checked_add(b),
                    BinOp::Sub => a.checked_sub(b),
                    BinOp::Mul => a.checked_mul(b),
                    BinOp::Div => {
                        if b == Rational::ZERO {
                            return Err(Diagnostic::new(rhs.span(), "division by zero"));
                        }
                        a.checked_div(b)
                    }
                    BinOp::Rem => match (a.to_integer(), b.to_integer()) {
                        (Some(_), Some(0)) => {
                            return Err(Diagnostic::new(rhs.span(), "division by zero"));
                        }
                        (Some(x), Some(y)) => Rational::new(x % y, 1),
                        _ => {
                            return Err(Diagnostic::new(
                                *span,
                                "`%` needs whole numbers on both sides",
                            ));
                        }
                    },
                };
                out.ok_or_else(|| Diagnostic::new(*span, "value is out of range"))
            }
            other => Err(Diagnostic::new(
                other.span(),
                "expected a constant number here; names are only known after resolution",
            )),
        }
    }
}

impl SourceUnit {
    /// A canonical, deterministic rendering of the tree, one statement per
    /// line.
    ///
    /// Spans are omitted and every expression is fully parenthesised, which
    /// makes it exactly what a golden test wants: precedence and structure are
    /// visible, and formatting differences in the input are not. It is not a
    /// round-trippable projection — that is `rsemu convert`'s job (§2) and it
    /// will read the same tree.
    pub fn dump(&self) -> String {
        let mut out = String::new();
        for stmt in &self.stmts {
            dump_stmt(stmt, 0, &mut out);
        }
        out
    }
}

/// Append `depth` levels of indentation.
fn indent(depth: usize, out: &mut String) {
    for _ in 0..depth {
        out.push_str("  ");
    }
}

/// Append one statement and its subtree.
fn dump_stmt(stmt: &Stmt, depth: usize, out: &mut String) {
    indent(depth, out);
    match stmt {
        Stmt::Machine(s) => {
            out.push_str(&format!("machine {} {{\n", quote(&s.name.node)));
            dump_body(&s.body, depth, out);
        }
        Stmt::Param(s) => {
            out.push_str(&format!("param {}", dump_name(&s.name)));
            if let Some(d) = &s.default {
                out.push_str(&format!(" = {}", dump_expr(d)));
            }
            out.push('\n');
        }
        Stmt::Osc(s) => {
            out.push_str(&format!(
                "osc {} = {}",
                dump_name(&s.name),
                dump_expr(&s.freq)
            ));
            out.push(' ');
            out.push_str(s.unit.node.as_str());
            out.push('\n');
        }
        Stmt::Space(s) => {
            out.push_str(&format!("space {} ", dump_name(&s.name)));
            dump_props(&s.props, out);
            out.push('\n');
        }
        Stmt::Object(s) => {
            out.push_str(&format!(
                "object {} {} ",
                dump_name(&s.name),
                quote(&s.class.node)
            ));
            dump_props(&s.props, out);
            out.push('\n');
        }
        Stmt::Map(s) => {
            out.push_str(&format!(
                "map {} {} size {} = {}",
                dump_name(&s.space),
                dump_expr(&s.base),
                dump_expr(&s.size),
                dump_expr(&s.target)
            ));
            if !s.props.is_empty() {
                out.push(' ');
                dump_props(&s.props, out);
            }
            out.push('\n');
        }
        Stmt::Wire(s) => {
            out.push_str(&format!(
                "wire {} -> {}\n",
                dump_path(&s.from),
                dump_path(&s.to)
            ));
        }
        Stmt::Include(s) => {
            out.push_str(&format!("include {}\n", quote(&s.path.node)));
        }
        Stmt::Template(s) => {
            out.push_str(&format!("template {}(", dump_name(&s.name)));
            for (i, p) in s.params.iter().enumerate() {
                if i > 0 {
                    out.push_str(", ");
                }
                out.push_str(&dump_name(&p.name));
                if let Some(d) = &p.default {
                    out.push_str(&format!(" = {}", dump_expr(d)));
                }
            }
            out.push_str(") {\n");
            dump_body(&s.body, depth, out);
        }
        Stmt::Instance(s) => {
            out.push_str(&format!(
                "instance {} = {}(",
                dump_name(&s.name),
                dump_name(&s.template)
            ));
            for (i, a) in s.args.iter().enumerate() {
                if i > 0 {
                    out.push_str(", ");
                }
                if let Some(n) = &a.name {
                    out.push_str(&format!("{} = ", dump_name(n)));
                }
                out.push_str(&dump_expr(&a.value));
            }
            out.push_str(")\n");
        }
        Stmt::For(s) => {
            out.push_str(&format!(
                "for {} in {}{}{} {{\n",
                dump_name(&s.var),
                dump_expr(&s.start),
                if s.inclusive { "..=" } else { ".." },
                dump_expr(&s.end)
            ));
            dump_body(&s.body, depth, out);
        }
    }
}

/// Append an indented body and its closing brace.
fn dump_body(body: &[Stmt], depth: usize, out: &mut String) {
    for stmt in body {
        dump_stmt(stmt, depth + 1, out);
    }
    indent(depth, out);
    out.push_str("}\n");
}

/// Append a `{ a = 1, b = 2 }` property block.
fn dump_props(props: &[Property], out: &mut String) {
    out.push('{');
    for (i, p) in props.iter().enumerate() {
        out.push_str(if i > 0 { ", " } else { " " });
        out.push_str(&format!("{} = {}", dump_name(&p.name), dump_expr(&p.value)));
    }
    out.push_str(if props.is_empty() { "}" } else { " }" });
}

/// Render a name, showing substitutions as they were written.
fn dump_name(name: &Name) -> String {
    let mut out = String::new();
    for part in &name.parts {
        match part {
            NamePart::Literal(text) => out.push_str(text),
            NamePart::Substitution(expr) => match expr {
                Expr::Path(p) if p.segments.len() == 1 && p.as_literal().is_some() => {
                    out.push('$');
                    out.push_str(&dump_path(p));
                }
                other => {
                    out.push_str("${");
                    out.push_str(&dump_expr(other));
                    out.push('}');
                }
            },
        }
    }
    out
}

/// Render a dotted path.
fn dump_path(path: &Path) -> String {
    let mut out = String::new();
    for (i, seg) in path.segments.iter().enumerate() {
        if i > 0 {
            out.push('.');
        }
        out.push_str(&dump_name(seg));
    }
    out
}

/// Render an expression, fully parenthesised.
fn dump_expr(expr: &Expr) -> String {
    match expr {
        Expr::Num(n) => n.node.value.to_string(),
        Expr::Str(s) => quote(&s.node),
        Expr::Bool(b) => b.node.to_string(),
        Expr::Path(p) => dump_path(p),
        Expr::Call { callee, args, .. } => {
            let mut out = format!("{}(", dump_path(callee));
            for (i, a) in args.iter().enumerate() {
                if i > 0 {
                    out.push_str(", ");
                }
                out.push_str(&dump_expr(a));
            }
            out.push(')');
            out
        }
        Expr::Unary { operand, .. } => format!("(-{})", dump_expr(operand)),
        Expr::Binary { op, lhs, rhs, .. } => {
            format!("({} {} {})", dump_expr(lhs), op.as_str(), dump_expr(rhs))
        }
        Expr::List { items, .. } => {
            let mut out = String::from("[");
            for (i, item) in items.iter().enumerate() {
                if i > 0 {
                    out.push_str(", ");
                }
                out.push_str(&dump_expr(item));
            }
            out.push(']');
            out
        }
        Expr::Map { entries, .. } => {
            let mut out = String::new();
            dump_props(entries, &mut out);
            out
        }
    }
}

/// Re-quote a string with escapes, so a dump is unambiguous.
fn quote(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\x{:02x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::machine::lexer::{NumUnit, Radix};

    fn num(value: u64) -> Expr {
        Expr::Num(Spanned::new(
            NumLit {
                value,
                digits: value,
                radix: Radix::Dec,
                unit: NumUnit::None,
            },
            Span::at(0),
        ))
    }

    #[test]
    fn literal_expressions_evaluate_exactly() {
        let e = Expr::Binary {
            op: BinOp::Div,
            lhs: alloc::boxed::Box::new(num(236_250_000)),
            rhs: alloc::boxed::Box::new(num(11)),
            span: Span::at(0),
        };
        let r = e.eval_rational().expect("literal");
        assert_eq!(r.numerator(), 236_250_000);
        assert_eq!(r.denominator(), 11);
    }

    #[test]
    fn evaluation_refuses_names_and_zero_divisors() {
        let name = Expr::Path(Path {
            segments: alloc::vec![Name {
                parts: alloc::vec![NamePart::Literal("master".to_string())],
                span: Span::at(0),
            }],
            span: Span::at(0),
        });
        assert!(name.eval_rational().is_err());

        let div0 = Expr::Binary {
            op: BinOp::Div,
            lhs: alloc::boxed::Box::new(num(1)),
            rhs: alloc::boxed::Box::new(num(0)),
            span: Span::at(0),
        };
        assert_eq!(
            div0.eval_rational().expect_err("zero").message,
            "division by zero"
        );
    }

    #[test]
    fn quoting_is_reversible_looking() {
        assert_eq!(quote("a\"b\\c\n\t\r\u{1}"), "\"a\\\"b\\\\c\\n\\t\\r\\x01\"");
    }

    #[test]
    fn frequency_units_scale() {
        assert_eq!(FreqUnit::from_spelling("MHz"), Some(FreqUnit::MHz));
        assert_eq!(FreqUnit::from_spelling("kHz"), Some(FreqUnit::KHz));
        assert_eq!(FreqUnit::from_spelling("KHz"), Some(FreqUnit::KHz));
        assert_eq!(FreqUnit::from_spelling("hz"), None);
        let osc = OscDecl {
            name: Name {
                parts: alloc::vec![NamePart::Literal("x".to_string())],
                span: Span::at(0),
            },
            freq: num(21),
            unit: Spanned::new(FreqUnit::MHz, Span::at(0)),
            span: Span::at(0),
        };
        assert_eq!(
            osc.frequency_hz().expect("literal").to_integer(),
            Some(21_000_000)
        );
    }
}
