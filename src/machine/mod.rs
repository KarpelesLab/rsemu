//! The machine description language (`ROADMAP.md` §5).
//!
//! A `.machine` file is how a person describes a machine to rsemu: oscillators,
//! address spaces, devices, the memory map and the wires between them. It is
//! the framework's user interface, and §5 is blunt about why that matters —
//! most people meet rsemu through a syntax error.
//!
//! # What is here
//!
//! Everything up to, but not including, building anything:
//!
//! ```text
//! lex → parse (spans preserved) → resolve → validate → realize → run
//! ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   ~~~~~~~~~~~~~
//!                   this module                        not yet written
//! ```
//!
//! | Module | Role |
//! | --- | --- |
//! | [`span`] | byte-offset spans, and mapping them to `file:line:col` |
//! | [`diag`] | one precise error, rendered with the line and a caret |
//! | [`lexer`] | hand-written tokenizer, no generator and no regex |
//! | [`ast`] | the syntax tree, with a span on every node |
//! | [`parser`] | recursive descent, depth-guarded |
//! | [`rational`] | exact frequencies, because 236250000/11 Hz is not an integer |
//! | [`sources`] | several files in one span space, and the `include` seam |
//! | [`resolver`] | params, includes, templates, loops, links, cycles |
//! | [`mod@validate`] | classes, properties, pins, address ranges, wire cycles |
//!
//! # What is not here, and where it plugs in
//!
//! * **The realizer** — constructing devices, mapping regions, connecting
//!   wires. It reads [`resolver::Resolved`], which is why that type carries a
//!   span on everything: a failure at realize time still owes the user a
//!   caret.
//! * **The device registry.** [`validate()`] takes a
//!   [`ClassTable`](validate::ClassTable) as an argument rather than reaching
//!   for `core::registry`, which does not exist yet; when it does, it
//!   implements [`validate::Classes`] and nothing else moves.
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
pub mod diag;
pub mod lexer;
pub mod parser;
pub mod rational;
pub mod resolver;
pub mod sources;
pub mod span;
pub mod validate;

#[cfg(test)]
mod tests;

pub use crate::machine::ast::SourceUnit;
pub use crate::machine::diag::Diagnostic;
pub use crate::machine::parser::parse;
pub use crate::machine::rational::Rational;
pub use crate::machine::resolver::{ResolveOptions, Resolved, resolve};
pub use crate::machine::sources::{IncludeLoader, SourceMap};
pub use crate::machine::span::{SourceFile, Span, Spanned};
pub use crate::machine::validate::{ValidateOptions, validate};

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
