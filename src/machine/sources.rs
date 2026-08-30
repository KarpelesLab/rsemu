//! Several files in one span space, and the seam through which `include`
//! reads them.
//!
//! The front end handles one file at a time: a [`Span`] is two `u32` offsets
//! and nothing says which file they point into. `include` (`ROADMAP.md` §5)
//! breaks that, and there are two ways out — widen every span with a file
//! index, or give each file a base offset in one shared coordinate space. This
//! module does the second: spans stay two `u32`s, the parser is untouched, and
//! a [`SourceMap`] turns any global span back into `file:line:col`.
//!
//! Rebasing costs one walk of the tree per file ([`SourceMap::parse`] does it),
//! which is cheaper than the span widening it avoids and, unlike a file index,
//! cannot be forgotten at a call site.
//!
//! # Loading is the caller's job
//!
//! `machine/` is `no_std` and must never touch a filesystem — the browser
//! build has none, and *which* directories an `include` may name is a policy
//! question a library has no business answering. So an include is resolved
//! through an [`IncludeLoader`] the caller supplies. [`MemoryLoader`] covers
//! tests and embedders that already hold their files; a CLI implements the
//! trait over its search path in a dozen lines.

use alloc::borrow::ToOwned;
use alloc::boxed::Box;
use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use crate::core::Error;
use crate::machine::ast::{
    Arg, Expr, MachineDecl, Name, NamePart, Path, Property, SourceUnit, Stmt, TemplateParam,
};
use crate::machine::diag::{Diagnostic, Sources};
use crate::machine::parser::parse;
use crate::machine::span::{SourceFile, Span};

/// A file in a [`SourceMap`], stable for the map's lifetime.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct FileId(pub u32);

/// One loaded file: the name diagnostics print, its text, and its base offset.
#[derive(Debug)]
struct Entry {
    name: String,
    text: String,
    base: u32,
}

/// The files a resolution reads, sharing one span coordinate space.
///
/// Every file gets a base offset; a span produced by parsing file *f* is
/// shifted by *f*'s base, so a span identifies both a file and a range within
/// it. Bases only ever grow, so [`SourceMap::locate`] is a binary search.
///
/// ```
/// use rsemu::machine::sources::SourceMap;
///
/// let mut map = SourceMap::new();
/// let root = map.add("nes.machine", "machine \"nes\" {}\n")?;
/// let unit = map.parse(root)?;
/// assert_eq!(unit.stmts.len(), 1);
/// # Ok::<(), rsemu::machine::Diagnostic>(())
/// ```
#[derive(Debug, Default)]
pub struct SourceMap {
    files: Vec<Entry>,
    next: u32,
}

impl SourceMap {
    /// An empty map.
    pub fn new() -> SourceMap {
        SourceMap {
            files: Vec::new(),
            next: 0,
        }
    }

    /// Add a file, returning its id.
    ///
    /// Fails when the files together would exceed 4 GiB, which is the same cap
    /// [`Span`] imposes on a single file.
    pub fn add(
        &mut self,
        name: impl Into<String>,
        text: impl Into<String>,
    ) -> Result<FileId, Diagnostic> {
        let name = name.into();
        let text = text.into();
        let len = u32::try_from(text.len()).ok();
        // One byte of slack after each file so that an end-of-file span, which
        // sits exactly at `base + len`, still belongs to the file it came from.
        let next = len
            .and_then(|len| self.next.checked_add(len))
            .and_then(|end| end.checked_add(1));
        let Some(next) = next else {
            return Err(Diagnostic::new(
                Span::at(self.next.saturating_sub(1)),
                format!(
                    "`{name}` does not fit: machine descriptions are limited to 4 GiB in total"
                ),
            ));
        };
        let base = self.next;
        self.next = next;
        let id = FileId(u32::try_from(self.files.len()).unwrap_or(u32::MAX));
        self.files.push(Entry { name, text, base });
        Ok(id)
    }

    /// How many files are loaded.
    pub fn len(&self) -> usize {
        self.files.len()
    }

    /// Whether no files are loaded.
    pub fn is_empty(&self) -> bool {
        self.files.is_empty()
    }

    /// The name a file was added under, as diagnostics print it.
    pub fn name(&self, id: FileId) -> Option<&str> {
        self.entry(id).map(|e| e.name.as_str())
    }

    /// A file's text.
    pub fn text(&self, id: FileId) -> Option<&str> {
        self.entry(id).map(|e| e.text.as_str())
    }

    /// A file's base offset in the shared span space.
    pub fn base(&self, id: FileId) -> Option<u32> {
        self.entry(id).map(|e| e.base)
    }

    /// The whole of a file, as one span.
    pub fn file_span(&self, id: FileId) -> Option<Span> {
        let e = self.entry(id)?;
        let len = u32::try_from(e.text.len()).unwrap_or(u32::MAX);
        Some(Span::new(e.base, e.base.saturating_add(len)))
    }

    /// The id of the file a global span points into.
    pub fn file_at(&self, span: Span) -> Option<FileId> {
        self.index_at(span.start)
            .map(|i| FileId(u32::try_from(i).unwrap_or(u32::MAX)))
    }

    /// Parse a file, with every span already rebased into the shared space.
    ///
    /// The parser knows nothing about the map; this shifts the tree afterwards,
    /// which is the one place that has to remember to.
    pub fn parse(&self, id: FileId) -> Result<SourceUnit, Diagnostic> {
        let Some(entry) = self.entry(id) else {
            return Err(Diagnostic::new(Span::at(0), "no such source file"));
        };
        let src = SourceFile::new(&entry.name, &entry.text);
        let mut unit = parse(&src).map_err(|d| shift_diagnostic(d, entry.base))?;
        shift_unit(&mut unit, entry.base);
        Ok(unit)
    }

    /// Render a diagnostic against these files, rustc-style.
    pub fn render(&self, diag: &Diagnostic) -> String {
        diag.render_in(self)
    }

    /// Convert a diagnostic to the crate error type.
    pub fn to_error(&self, diag: &Diagnostic) -> Error {
        diag.to_error_in(self)
    }

    fn entry(&self, id: FileId) -> Option<&Entry> {
        self.files.get(id.0 as usize)
    }

    /// The index of the file containing `offset`, by binary search over bases.
    fn index_at(&self, offset: u32) -> Option<usize> {
        if self.files.is_empty() {
            return None;
        }
        let mut lo = 0usize;
        let mut hi = self.files.len() - 1;
        while lo < hi {
            // Round up so the loop always makes progress.
            let mid = lo + (hi - lo).div_ceil(2);
            if self.files[mid].base <= offset {
                lo = mid;
            } else {
                hi = mid - 1;
            }
        }
        Some(lo)
    }
}

impl Sources for SourceMap {
    fn locate(&self, span: Span) -> (SourceFile<'_>, Span) {
        // An empty map still has to render something: a diagnostic that cannot
        // be printed would hide the error it is reporting.
        let Some(entry) = self.index_at(span.start).and_then(|i| self.files.get(i)) else {
            return (SourceFile::new("<none>", ""), Span::at(0));
        };
        let src = SourceFile::new(&entry.name, &entry.text);
        let local = Span::new(
            span.start.saturating_sub(entry.base),
            span.end.saturating_sub(entry.base),
        );
        (src, local)
    }
}

// ---------------------------------------------------------------------------
// Loading
// ---------------------------------------------------------------------------

/// A file an [`IncludeLoader`] found.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Included {
    /// The **canonical** name of the file that was found.
    ///
    /// Canonical is load-bearing twice over: include-cycle detection compares
    /// these, and so does include-once. Two `include`s that reach the same file
    /// by different spellings must return the same name here, or the cycle
    /// check will not see the loop. The loader owns that decision because only
    /// it knows what its namespace means.
    pub name: String,
    /// The file's text.
    pub text: String,
}

/// How the resolver reads an `include`.
///
/// The path is passed exactly as written in the file; resolving it against a
/// search path, refusing to escape a sandbox, and reading the bytes are all the
/// implementor's. Returning `Err` produces a diagnostic pointing at the
/// `include` statement, with the message as written here — so say what was
/// tried, not just "not found".
pub trait IncludeLoader {
    /// Load the file `path` names, as referred to from `from`.
    ///
    /// `from` is the canonical name of the including file, so a loader can
    /// implement relative includes.
    fn load(&mut self, path: &str, from: &str) -> Result<Included, String>;
}

impl<F> IncludeLoader for F
where
    F: FnMut(&str, &str) -> Result<Included, String>,
{
    fn load(&mut self, path: &str, from: &str) -> Result<Included, String> {
        self(path, from)
    }
}

impl IncludeLoader for Box<dyn IncludeLoader + '_> {
    fn load(&mut self, path: &str, from: &str) -> Result<Included, String> {
        (**self).load(path, from)
    }
}

/// A loader that refuses every `include`.
///
/// The right default for a caller that has no filesystem and did not intend to
/// support includes: the error says so rather than pretending the file is
/// missing.
#[derive(Debug, Clone, Copy, Default)]
pub struct NoIncludes;

impl IncludeLoader for NoIncludes {
    fn load(&mut self, path: &str, _from: &str) -> Result<Included, String> {
        Err(format!(
            "cannot include `{path}`: this build was given no include loader"
        ))
    }
}

/// A loader over files already in memory.
///
/// What tests use, and what a browser embedder wants: the search path is the
/// map's key set, and a name is canonical because it is a key.
#[derive(Debug, Clone, Default)]
pub struct MemoryLoader {
    files: BTreeMap<String, String>,
}

impl MemoryLoader {
    /// An empty set of files.
    pub fn new() -> MemoryLoader {
        MemoryLoader {
            files: BTreeMap::new(),
        }
    }

    /// Add a file under `name`.
    #[must_use]
    pub fn with(mut self, name: impl Into<String>, text: impl Into<String>) -> MemoryLoader {
        self.files.insert(name.into(), text.into());
        self
    }

    /// Add a file under `name`, replacing any previous one.
    pub fn insert(&mut self, name: impl Into<String>, text: impl Into<String>) {
        self.files.insert(name.into(), text.into());
    }
}

impl IncludeLoader for MemoryLoader {
    fn load(&mut self, path: &str, _from: &str) -> Result<Included, String> {
        match self.files.get(path) {
            Some(text) => Ok(Included {
                name: path.to_owned(),
                text: text.clone(),
            }),
            None => {
                let mut names = String::new();
                for (i, name) in self.files.keys().enumerate() {
                    if i != 0 {
                        names.push_str(", ");
                    }
                    names.push_str(&format!("`{name}`"));
                }
                if names.is_empty() {
                    names.push_str("nothing");
                }
                Err(format!(
                    "no file named `{path}`; the search path holds {names}"
                ))
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Rebasing
// ---------------------------------------------------------------------------

/// `span`, moved into the shared coordinate space of a file based at `base`.
fn shift(span: Span, base: u32) -> Span {
    Span::new(
        span.start.saturating_add(base),
        span.end.saturating_add(base),
    )
}

/// A parse diagnostic, moved into the shared space.
fn shift_diagnostic(mut diag: Diagnostic, base: u32) -> Diagnostic {
    diag.span = shift(diag.span, base);
    if let Some(note) = &mut diag.note {
        note.span = shift(note.span, base);
    }
    diag
}

/// Move a whole parsed file into the shared space.
///
/// Exhaustive by construction: every arm below destructures its node, so adding
/// a field with a span to the AST fails to compile here rather than silently
/// producing a diagnostic that points at the wrong file.
fn shift_unit(unit: &mut SourceUnit, base: u32) {
    if base == 0 {
        return;
    }
    unit.span = shift(unit.span, base);
    for stmt in &mut unit.stmts {
        shift_stmt(stmt, base);
    }
}

fn shift_stmt(stmt: &mut Stmt, base: u32) {
    match stmt {
        Stmt::Machine(MachineDecl { name, body, span }) => {
            name.span = shift(name.span, base);
            for s in body {
                shift_stmt(s, base);
            }
            *span = shift(*span, base);
        }
        Stmt::Param(s) => {
            shift_name(&mut s.name, base);
            if let Some(d) = &mut s.default {
                shift_expr(d, base);
            }
            s.span = shift(s.span, base);
        }
        Stmt::Osc(s) => {
            shift_name(&mut s.name, base);
            shift_expr(&mut s.freq, base);
            s.unit.span = shift(s.unit.span, base);
            s.span = shift(s.span, base);
        }
        Stmt::Space(s) => {
            shift_name(&mut s.name, base);
            shift_props(&mut s.props, base);
            s.span = shift(s.span, base);
        }
        Stmt::Object(s) => {
            shift_name(&mut s.name, base);
            s.class.span = shift(s.class.span, base);
            shift_props(&mut s.props, base);
            s.span = shift(s.span, base);
        }
        Stmt::Map(s) => {
            shift_name(&mut s.space, base);
            shift_expr(&mut s.base, base);
            shift_expr(&mut s.size, base);
            shift_expr(&mut s.target, base);
            shift_props(&mut s.props, base);
            s.span = shift(s.span, base);
        }
        Stmt::Wire(s) => {
            shift_path(&mut s.from, base);
            shift_path(&mut s.to, base);
            s.span = shift(s.span, base);
        }
        Stmt::Include(s) => {
            s.path.span = shift(s.path.span, base);
            s.span = shift(s.span, base);
        }
        Stmt::Template(s) => {
            shift_name(&mut s.name, base);
            for TemplateParam {
                name,
                default,
                span,
            } in &mut s.params
            {
                shift_name(name, base);
                if let Some(d) = default {
                    shift_expr(d, base);
                }
                *span = shift(*span, base);
            }
            for st in &mut s.body {
                shift_stmt(st, base);
            }
            s.span = shift(s.span, base);
        }
        Stmt::Instance(s) => {
            shift_name(&mut s.name, base);
            shift_name(&mut s.template, base);
            for Arg { name, value, span } in &mut s.args {
                if let Some(n) = name {
                    shift_name(n, base);
                }
                shift_expr(value, base);
                *span = shift(*span, base);
            }
            s.span = shift(s.span, base);
        }
        Stmt::For(s) => {
            shift_name(&mut s.var, base);
            shift_expr(&mut s.start, base);
            shift_expr(&mut s.end, base);
            for st in &mut s.body {
                shift_stmt(st, base);
            }
            s.span = shift(s.span, base);
        }
    }
}

fn shift_props(props: &mut [Property], base: u32) {
    for Property { name, value, span } in props {
        shift_name(name, base);
        shift_expr(value, base);
        *span = shift(*span, base);
    }
}

fn shift_name(name: &mut Name, base: u32) {
    for part in &mut name.parts {
        if let NamePart::Substitution(e) = part {
            shift_expr(e, base);
        }
    }
    name.span = shift(name.span, base);
}

fn shift_path(path: &mut Path, base: u32) {
    for seg in &mut path.segments {
        shift_name(seg, base);
    }
    path.span = shift(path.span, base);
}

fn shift_expr(expr: &mut Expr, base: u32) {
    match expr {
        Expr::Num(n) => n.span = shift(n.span, base),
        Expr::Str(s) => s.span = shift(s.span, base),
        Expr::Bool(b) => b.span = shift(b.span, base),
        Expr::Path(p) => shift_path(p, base),
        Expr::Call { callee, args, span } => {
            shift_path(callee, base);
            for a in args {
                shift_expr(a, base);
            }
            *span = shift(*span, base);
        }
        Expr::Unary { operand, span, .. } => {
            shift_expr(operand, base);
            *span = shift(*span, base);
        }
        Expr::Binary { lhs, rhs, span, .. } => {
            shift_expr(lhs, base);
            shift_expr(rhs, base);
            *span = shift(*span, base);
        }
        Expr::List { items, span } => {
            for i in items {
                shift_expr(i, base);
            }
            *span = shift(*span, base);
        }
        Expr::Map { entries, span } => {
            shift_props(entries, base);
            *span = shift(*span, base);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn spans_from_two_files_locate_to_their_own_file() {
        let mut map = SourceMap::new();
        let a = map.add("a.machine", "param x = 1\n").expect("fits");
        let b = map.add("b.machine", "param y = 2\n").expect("fits");
        assert_eq!(map.base(a), Some(0));
        assert_eq!(map.base(b), Some(13));

        let unit_b = map.parse(b).expect("parses");
        let span = unit_b.stmts[0].span();
        assert_eq!(map.file_at(span), Some(b));
        let (src, local) = map.locate(span);
        assert_eq!(src.name(), "b.machine");
        assert_eq!(local.start, 0);
        assert_eq!(src.position(local.start), "b.machine:1:1");
    }

    #[test]
    fn a_parse_error_in_the_second_file_names_the_second_file() {
        let mut map = SourceMap::new();
        map.add("a.machine", "param x = 1\n").expect("fits");
        let b = map.add("b.machine", "param y = ?\n").expect("fits");
        let diag = map.parse(b).expect_err("should fail");
        let rendered = map.render(&diag);
        assert!(rendered.contains("b.machine:1:11"), "{rendered}");
    }

    #[test]
    fn an_end_of_file_span_still_belongs_to_its_own_file() {
        let mut map = SourceMap::new();
        let a = map.add("a.machine", "machine \"a\" {\n").expect("fits");
        map.add("b.machine", "param y = 2\n").expect("fits");
        let diag = map.parse(a).expect_err("unclosed brace");
        assert_eq!(map.file_at(diag.span), Some(a));
        assert!(map.render(&diag).contains("a.machine"));
    }

    #[test]
    fn locating_an_empty_map_renders_rather_than_failing() {
        let map = SourceMap::new();
        let diag = Diagnostic::new(Span::at(7), "nothing here");
        assert!(map.render(&diag).starts_with("error: nothing here"));
    }

    #[test]
    fn the_memory_loader_lists_what_it_has() {
        let mut loader = MemoryLoader::new().with("pci.machine", "param x = 1\n");
        assert_eq!(
            loader.load("pci.machine", "root").expect("found").text,
            "param x = 1\n"
        );
        let err = loader.load("missing", "root").expect_err("absent");
        assert_eq!(
            err,
            "no file named `missing`; the search path holds `pci.machine`"
        );
    }

    #[test]
    fn no_includes_says_so() {
        let err = NoIncludes.load("x.machine", "root").expect_err("refused");
        assert!(err.contains("no include loader"), "{err}");
    }

    #[test]
    fn a_closure_is_a_loader() {
        let mut loader = |path: &str, _from: &str| {
            Ok(Included {
                name: path.to_string(),
                text: String::new(),
            })
        };
        assert_eq!(loader.load("x", "y").expect("ok").name, "x");
    }
}
