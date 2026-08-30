//! The machine description language (`ROADMAP.md` §5).
//!
//! A `.machine` file is how a person describes a machine to rsemu: oscillators,
//! address spaces, devices, the memory map and the wires between them. It is
//! the framework's user interface, and §5 is blunt about why that matters —
//! most people meet rsemu through a syntax error.
//!
//! # What is here
//!
//! The **front end**, which is the first two stages of §5's pipeline:
//!
//! ```text
//! lex → parse (spans preserved) → resolve → validate → realize → run
//! ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//!         this module                       not yet written
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
//!
//! # What is not here, and where it plugs in
//!
//! * **The resolver** — parameters, `include` search paths, `template`
//!   expansion, loop unrolling, link resolution, cycle detection. The AST keeps
//!   all of these unexpanded and spanned, which is exactly what a resolver
//!   needs; nothing in this module interprets a name.
//! * **The validator** — does this device class exist, does it take this
//!   property, is this bus compatible. That needs `core::registry`, which does
//!   not exist yet.
//! * **The realizer** and the JSON projection (`rsemu convert`, §2). The
//!   projection will read this same AST: one AST, two syntaxes.
//! * **File loading.** The front end is `no_std` and takes a `&str` plus a name
//!   for diagnostics. Reading files, and choosing which directories an
//!   `include` may name, belong to the caller.
//!
//! # Example
//!
//! ```
//! use rsemu::machine::{parse_file, ast::Stmt};
//!
//! let text = r#"
//! machine "nes" {
//!   osc master = 236250000/11 Hz     # not an integer number of hertz
//!   object cpu "mos6502" { clock = master / 12 }
//! }
//! "#;
//! let unit = parse_file("nes.machine", text)?;
//! let Stmt::Machine(m) = &unit.stmts[0] else { unreachable!() };
//! assert_eq!(m.name.node, "nes");
//!
//! let Stmt::Osc(osc) = &m.body[0] else { unreachable!() };
//! let hz = osc.frequency_hz().expect("a literal frequency");
//! assert_eq!((hz.numerator(), hz.denominator()), (236_250_000, 11));
//! # Ok::<(), rsemu::Error>(())
//! ```

pub mod ast;
pub mod diag;
pub mod lexer;
pub mod parser;
pub mod rational;
pub mod span;

#[cfg(test)]
mod tests;

pub use crate::machine::ast::SourceUnit;
pub use crate::machine::diag::Diagnostic;
pub use crate::machine::parser::parse;
pub use crate::machine::rational::Rational;
pub use crate::machine::span::{SourceFile, Span, Spanned};

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
