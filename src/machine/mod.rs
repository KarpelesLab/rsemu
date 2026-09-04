//! The machine description language (`ROADMAP.md` §5).
//!
//! A `.machine` file is how a person describes a machine to rsemu: oscillators,
//! address spaces, devices, the memory map and the wires between them. It is
//! the framework's user interface, and §5 is blunt about why that matters —
//! most people meet rsemu through a syntax error.
//!
//! # What is here
//!
//! The whole front end, and the layer that turns its output into a machine:
//!
//! ```text
//! lex → parse (spans preserved) → resolve → validate → realize → run
//! ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//!                          all of it, ending in [`build`]
//! ```
//!
//! | Module | Role |
//! | --- | --- |
//! | [`builtin`] | the classes the language ships with: `ram` |
//! | [`catalog`] | what this build can emulate: classes, bindings, machines |
//! | [`span`] | byte-offset spans, and mapping them to `file:line:col` |
//! | [`diag`] | one precise error, rendered with the line and a caret |
//! | [`lexer`] | hand-written tokenizer, no generator and no regex |
//! | [`ast`] | the syntax tree, with a span on every node |
//! | [`parser`] | recursive descent, depth-guarded |
//! | [`rational`] | exact frequencies, because 236250000/11 Hz is not an integer |
//! | [`sources`] | several files in one span space, and the `include` seam |
//! | [`resolver`] | params, includes, templates, loops, links, cycles |
//! | [`mod@validate`] | classes, properties, pins, address ranges, wire cycles |
//! | [`mod@realize`] | construct, map, wire, bind, reset, sweep |
//! | [`mod@machine`] | the assembled [`Machine`], its snapshot and its run loop |
//!
//! # What is not here, and where it plugs in
//!
//! * **The device registry as a validator input.** [`validate()`] takes a
//!   [`ClassTable`] rather than reaching for `core::registry`, and [`build`]
//!   passes whatever the caller put in
//!   [`BuildOptions::classes`]. It cannot derive one: `DeviceClass` declares a
//!   class's *properties* but not its pins or its mappable regions, so a table
//!   built from the registry would reject every `map x = dev.regs` as naming a
//!   region the class does not have.
//! * **The JSON projection** (`rsemu convert`, §2), which will read the same
//!   AST: one AST, two syntaxes.
//! * **File loading.** This module is `no_std` and never touches a
//!   filesystem. An `include` goes through
//!   [`sources::IncludeLoader`], so the caller owns the search
//!   path and the sandbox.
//!
//! # Example
//!
//! ```
//! use rsemu::machine::resolver::ResolveOptions;
//! use rsemu::machine::resolve_file;
//!
//! let text = r#"
//! machine "nes" {
//!   param region = "ntsc"
//!   osc master = 236250000/11 Hz     # not an integer number of hertz
//!   space cpubus { width = 16, unassigned = open-bus }
//!   object cpu "mos6502" { clock = master / 12, space = cpubus }
//!   object ppu "nes.ppu" { clock = master / 4 }
//!   wire ppu.nmi -> cpu.nmi
//! }
//! "#;
//! let machine = resolve_file("nes.machine", text, &ResolveOptions::new())?;
//!
//! // The crystal stays rational; the ratios are exact integers (§4.2).
//! assert_eq!(machine.oscillators[0].hz.denominator(), 11);
//! let cpu = machine.objects[0].clock.expect("a clock");
//! let ppu = machine.objects[1].clock.expect("a clock");
//! assert_eq!((cpu.div, ppu.div), (12, 4));
//! assert_eq!(machine.wires[0].to.port, "nmi");
//! # Ok::<(), rsemu::Error>(())
//! ```

pub mod ast;
pub mod builtin;
pub mod catalog;
pub mod combinator;
pub mod diag;
pub mod lexer;
// `machine::machine` reads oddly, but the type it holds is `Machine` and the
// module is where a reader looks for it.
#[allow(clippy::module_inception)]
pub mod machine;
pub mod parser;
pub mod rational;
pub mod realize;
pub mod resolver;
pub mod sources;
pub mod span;
pub mod timeline;
pub mod validate;

#[cfg(test)]
mod tests;

pub use crate::machine::ast::SourceUnit;
pub use crate::machine::diag::Diagnostic;
pub use crate::machine::machine::Machine;
pub use crate::machine::parser::parse;
pub use crate::machine::rational::Rational;
pub use crate::machine::realize::{
    BindCtx, Bindings, Instance, MediaTable, Peer, RealizeOptions, SinkPin, realize, realize_with,
};
pub use crate::machine::resolver::{ResolveOptions, Resolved, resolve};
pub use crate::machine::sources::{IncludeLoader, SourceMap};
pub use crate::machine::span::{SourceFile, Span, Spanned};
pub use crate::machine::timeline::{DEFAULT_CADENCE, Timeline};
pub use crate::machine::validate::{ClassTable, ValidateOptions, validate};

/// Parse a machine description, reporting failures as [`Error::Config`].
///
/// The convenience entry point for everything above the front end: the error
/// carries `file:line:col` in `at` and the message plus a caret in `message`,
/// so a CLI can print it with `eprintln!("{err}")` and be done
/// (`ROADMAP.md` §5: errors carry file:line:col and a caret, always).
///
/// Use [`parse`] directly when the caller wants the [`Diagnostic`] itself — to
/// render it differently, or to attach it to a larger report.
///
/// [`Error::Config`]: crate::core::Error::Config
pub fn parse_file(name: &str, text: &str) -> crate::core::Result<SourceUnit> {
    let src = SourceFile::new(name, text);
    parse(&src).map_err(|d| d.to_error(&src))
}

/// Parse and resolve a self-contained machine description.
///
/// The whole pipeline short of validation, for a description that needs no
/// `include` and no search path — the common case for a test, a fixture or an
/// embedded string. A description that includes other files, or that should be
/// checked against a device registry, wants [`SourceMap`] plus [`resolve`] and
/// [`validate()`] directly, so that diagnostics can name whichever file they
/// point into.
///
/// [`Error::Config`]: crate::core::Error::Config
pub fn resolve_file(
    name: &str,
    text: &str,
    options: &ResolveOptions,
) -> crate::core::Result<Resolved> {
    let mut map = SourceMap::new();
    let root = map.add(name, text).map_err(|d| map.to_error(&d))?;
    resolve(&mut map, root, &mut sources::NoIncludes, options).map_err(|d| map.to_error(&d))
}

/// Everything the pipeline needs beyond the source text and the registry.
///
/// A struct rather than eight arguments, and owned rather than borrowed, so a
/// caller can build one once and reuse it for every machine it loads.
#[derive(Debug, Default)]
pub struct BuildOptions {
    /// Parameter overrides and expansion limits.
    pub resolve: ResolveOptions,
    /// What the validator insists on beyond its always-on checks.
    pub validate: ValidateOptions,
    /// Scheduler configuration.
    pub realize: realize::RealizeOptions,
    /// Class descriptions for the validator. Empty skips the class-specific
    /// checks; see the module docs for why the registry cannot supply this.
    pub classes: ClassTable,
    /// The classes that take part in the memory map and the wire graph.
    pub bindings: Bindings,
}

impl BuildOptions {
    /// Defaults: no parameter overrides, no class table, no bindings.
    pub fn new() -> BuildOptions {
        BuildOptions::default()
    }

    /// Use `bindings` for the classes that connect to the machine graph.
    #[must_use]
    pub fn with_bindings(mut self, bindings: Bindings) -> BuildOptions {
        self.bindings = bindings;
        self
    }

    /// Validate against `classes`.
    #[must_use]
    pub fn with_classes(mut self, classes: ClassTable) -> BuildOptions {
        self.classes = classes;
        self
    }

    /// Bind a media slot, as `rsemu run … --cart smb.nes` would.
    ///
    /// The bytes reach whichever object's machine file names the slot; see
    /// [`MediaTable`].
    #[must_use]
    pub fn with_media(
        mut self,
        slot: impl Into<alloc::string::String>,
        bytes: impl Into<alloc::sync::Arc<[u8]>>,
    ) -> BuildOptions {
        self.realize.media.insert(slot, bytes);
        self
    }

    /// Override a `param`, as `rsemu run … -p ram=4M` would.
    #[must_use]
    pub fn with_param(
        mut self,
        name: impl Into<alloc::string::String>,
        value: impl Into<alloc::string::String>,
    ) -> BuildOptions {
        self.resolve = core::mem::take(&mut self.resolve).with_param(name, value);
        self
    }
}

/// The whole pipeline: source text to a machine that can run.
///
/// ```text
/// lex → parse → resolve → validate → realize
/// ```
///
/// Front-end failures are rendered against the source, so the error carries
/// `file:line:col` and a caret (§5). Realize-time failures name the instance
/// instead — see [`mod@realize`] for why that is a seam rather than a choice.
///
/// ```
/// use rsemu::core::Registry;
/// use rsemu::machine::{BuildOptions, build};
///
/// // No device features are enabled in this build, so the machine this
/// // registry can assemble is one with no devices in it — which still has
/// // spaces, a scheduler and a snapshot.
/// let machine = build(
///     "empty.machine",
///     r#"machine "empty" { space cpubus { width = 16, unassigned = read-as-ones } }"#,
///     &Registry::new(),
///     &BuildOptions::new(),
/// )?;
/// assert_eq!(machine.name(), "empty");
/// assert_eq!(machine.spaces().len(), 1);
/// assert!(machine.devices().is_empty());
/// # Ok::<(), rsemu::Error>(())
/// ```
///
/// # Errors
///
/// A syntax error, an unresolved name, a failed validation, or anything
/// [`realize()`] refuses.
pub fn build(
    name: &str,
    text: &str,
    registry: &crate::core::Registry,
    options: &BuildOptions,
) -> crate::core::Result<Machine> {
    let mut map = SourceMap::new();
    let root = map.add(name, text).map_err(|d| map.to_error(&d))?;
    let resolved = resolve(&mut map, root, &mut sources::NoIncludes, &options.resolve)
        .map_err(|d| map.to_error(&d))?;
    validate(&resolved, &options.classes, &options.validate).map_err(|d| map.to_error(&d))?;
    realize_with(&resolved, registry, &options.bindings, &options.realize)
}
