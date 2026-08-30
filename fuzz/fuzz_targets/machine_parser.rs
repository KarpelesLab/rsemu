#![no_main]
//! The `.machine` front end must never panic on any input.
//!
//! This is the target ROADMAP.md §13's phase-2 gate names: it must survive one
//! CPU-hour from a seeded corpus with zero crashes and zero timeouts.
//!
//! The claim under test is written down in `src/machine/parser.rs`:
//!
//! > Nesting is capped at [`MAX_DEPTH`], which bounds both parser recursion and
//! > the recursion in dropping the tree, so no input — truncated, adversarial,
//! > or merely enthusiastic — can overflow the stack. Nothing here indexes,
//! > slices or does unchecked arithmetic on untrusted values.
//!
//! Both halves of the pipeline are in scope, because a `.machine` file is
//! something a user hands us and the error path runs on exactly the inputs the
//! happy path rejects:
//!
//! * **lex → parse.** No panic, and the returned tree is dropped here (bounded
//!   depth is a claim about `Drop` as much as about the parser).
//! * **the diagnostic.** `Diagnostic::render` maps a span back to
//!   `file:line:col` and slices out the offending line to hang a caret under.
//!   That is byte-offset arithmetic over untrusted text with multi-byte
//!   characters in it, and it runs on *every* rejected input — so a panic there
//!   is a panic on malformed input, which is the whole point. `SourceFile`
//!   documents that it clamps rather than rejects; this checks it.
//! * **constant evaluation.** `Expr::eval_rational` and `OscDecl::frequency_hz`
//!   do exact `i128` rational arithmetic on literals the input chose. Every
//!   operation there is `checked_*` by design; that is a claim about overflow
//!   on adversarial input, so it is fuzzed rather than trusted.
//! * **`SourceUnit::dump`.** A second recursive walk of the tree, and the one
//!   the golden error tests depend on.

use libfuzzer_sys::fuzz_target;
use rsemu::machine::ast::{Expr, MachineDecl, Name, NamePart, Path, Property, Stmt};
use rsemu::machine::lexer::tokenize;
use rsemu::machine::span::SourceFile;
use rsemu::machine::{parse, parse_file};

/// Spans are `u32` byte offsets, so a source file over 4 GiB is outside the
/// type's range and outside anything a person writes by hand. libFuzzer's
/// default `-max_len` is far below this; the cap only stops a corpus entry
/// someone pasted in by mistake from turning one iteration into a timeout,
/// which the phase gate counts as a failure.
const MAX_INPUT: usize = 1 << 20;

fuzz_target!(|data: &[u8]| {
    if data.len() > MAX_INPUT {
        return;
    }

    // The front end takes `&str`, so bytes have to become text somehow. Lossy
    // conversion rather than "skip non-UTF-8 inputs" because it maps *every*
    // input to a parse, keeping the fuzzer's coverage feedback meaningful:
    // discarding an input teaches libFuzzer nothing about the mutation that
    // produced it. Valid UTF-8 passes through byte-identical, so a corpus entry
    // that is a real `.machine` file is fuzzed as itself.
    let text = String::from_utf8_lossy(data);
    let src = SourceFile::new("fuzz.machine", &text);

    // The lexer on its own first: it is reachable through `parse`, but a
    // location lookup for every token exercises `SourceFile::location`'s
    // clamping over spans this input actually produced, including spans that
    // land mid-character.
    if let Ok(tokens) = tokenize(&src) {
        for token in &tokens {
            let _ = src.position(token.span.start);
            let _ = src.position(token.span.end);
            let loc = src.location(token.span.start);
            let _ = src.line_text(loc.line);
            let _ = token.kind.describe();
        }
    }

    match parse(&src) {
        Ok(unit) => {
            // A canonical rendering walks the whole tree recursively; if the
            // depth guard is doing its job this cannot overflow the stack.
            let _ = unit.dump();

            for stmt in &unit.stmts {
                visit_stmt(stmt);
            }

            // Same input through the convenience entry point, which is what
            // every caller above the front end actually uses.
            let _ = parse_file("fuzz.machine", &text);

            // Dropping the tree is the other half of the depth claim, and it
            // happens here at the end of the scope.
        }
        Err(diag) => {
            // The error path: rendering is byte arithmetic over the input.
            let rendered = diag.render(&src);
            assert!(
                !rendered.is_empty(),
                "a diagnostic must render to something a person can read"
            );
            let _ = diag.to_error(&src);
        }
    }
});

/// Walk a statement, evaluating every constant expression it contains.
fn visit_stmt(stmt: &Stmt) {
    let _ = stmt.span();
    match stmt {
        Stmt::Machine(m) => visit_machine(m),
        Stmt::Param(p) => {
            visit_name(&p.name);
            if let Some(default) = &p.default {
                visit_expr(default);
            }
        }
        Stmt::Osc(osc) => {
            // The §5 motivating case: `236250000/11 Hz` must stay exact. The
            // scaling multiply is `checked_*`, so an enormous literal times a
            // GHz scale is an error rather than a wrap.
            let _ = osc.frequency_hz();
            visit_name(&osc.name);
            visit_expr(&osc.freq);
        }
        Stmt::Space(s) => {
            visit_name(&s.name);
            visit_props(&s.props);
        }
        Stmt::Object(o) => {
            visit_name(&o.name);
            visit_props(&o.props);
        }
        Stmt::Map(m) => {
            visit_name(&m.space);
            visit_expr(&m.base);
            visit_expr(&m.size);
            visit_expr(&m.target);
            visit_props(&m.props);
        }
        Stmt::Wire(w) => {
            visit_path(&w.from);
            visit_path(&w.to);
        }
        Stmt::Include(_) => {}
        Stmt::Template(t) => {
            visit_name(&t.name);
            for param in &t.params {
                visit_name(&param.name);
                if let Some(default) = &param.default {
                    visit_expr(default);
                }
            }
            for inner in &t.body {
                visit_stmt(inner);
            }
        }
        Stmt::Instance(i) => {
            visit_name(&i.name);
            visit_name(&i.template);
            for arg in &i.args {
                if let Some(name) = &arg.name {
                    visit_name(name);
                }
                visit_expr(&arg.value);
            }
        }
        Stmt::For(f) => {
            visit_name(&f.var);
            visit_expr(&f.start);
            visit_expr(&f.end);
            for inner in &f.body {
                visit_stmt(inner);
            }
        }
    }
}

/// A name is a small template: `bank${j * 2}` holds an expression, so indexed
/// instantiation puts fuzzer-chosen arithmetic in a place that does not look
/// like an expression position.
fn visit_name(name: &Name) {
    let _ = name.as_literal();
    for part in &name.parts {
        match part {
            NamePart::Literal(_) => {}
            NamePart::Substitution(expr) => visit_expr(expr),
        }
    }
}

fn visit_path(path: &Path) {
    let _ = path.as_literal();
    for segment in &path.segments {
        visit_name(segment);
    }
}

fn visit_machine(machine: &MachineDecl) {
    for stmt in &machine.body {
        visit_stmt(stmt);
    }
}

fn visit_props(props: &[Property]) {
    for prop in props {
        visit_expr(&prop.value);
    }
}

/// Evaluate an expression and recurse into it.
///
/// Recursion is safe here for the same reason `dump` is: the tree the parser
/// produced is capped at `parser::MAX_DEPTH` levels.
fn visit_expr(expr: &Expr) {
    // Names are rejected (they are the resolver's job), literals are folded
    // exactly. Either way it must not panic — including on `1/0`, on `%` with
    // a rational operand, and on `i128::MIN`-adjacent negation.
    let _ = expr.eval_rational();
    let _ = expr.span();

    match expr {
        Expr::Num(_) | Expr::Str(_) | Expr::Bool(_) => {}
        Expr::Path(path) => visit_path(path),
        Expr::Call { callee, args, .. } => {
            visit_path(callee);
            for arg in args {
                visit_expr(arg);
            }
        }
        Expr::Unary { operand, .. } => visit_expr(operand),
        Expr::Binary { lhs, rhs, .. } => {
            visit_expr(lhs);
            visit_expr(rhs);
        }
        Expr::List { items, .. } => {
            for item in items {
                visit_expr(item);
            }
        }
        Expr::Map { entries, .. } => visit_props(entries),
    }
}
