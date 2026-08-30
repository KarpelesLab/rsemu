//! The known-failures ledger — a list that is only ever allowed to shrink.
//!
//! `CLAUDE.md` (CPU cores): *a core lands with its conformance suite and a
//! known-failures ledger that only ever shrinks.* Two halves, and the second is
//! the one that needs enforcing:
//!
//! * A failure that is **not** in the ledger fails the suite. That is the
//!   obvious half.
//! * A ledger entry whose test now **passes** also fails the suite, with an
//!   instruction to delete the line. Without that, the ledger silently becomes
//!   a list of things that used to be broken, and the next person to add an
//!   entry has no idea which of them are load-bearing.
//!
//! Staleness is only checked for tests that actually ran, so narrowing a run to
//! one opcode does not condemn the rest of the file.
//!
//! Format — one entry per line, `#` to end of line is a comment:
//!
//! ```text
//! 8b                    # ANE #imm: unstable, depends on an analog magic constant
//! ab :: ab 5c 21        # a single vector, by its upstream name
//! ```

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

/// One excused failure.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Entry {
    /// The opcode this entry covers.
    pub(crate) opcode: u8,
    /// A specific vector name, or `None` to cover every vector of the opcode.
    pub(crate) vector: Option<String>,
    /// The justification written after `#`. Required — an unexplained entry is
    /// indistinguishable from a forgotten one.
    pub(crate) note: String,
    /// 1-based line number, for error messages.
    pub(crate) line: usize,
}

/// A parsed ledger file.
#[derive(Debug)]
pub(crate) struct Ledger {
    /// Where it came from, for error messages.
    pub(crate) path: PathBuf,
    /// The entries, in file order.
    pub(crate) entries: Vec<Entry>,
}

/// A ledger that could not be parsed.
#[derive(Debug)]
pub(crate) struct ParseError {
    /// 1-based line number.
    pub(crate) line: usize,
    /// What was wrong.
    pub(crate) msg: String,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "line {}: {}", self.line, self.msg)
    }
}

impl Ledger {
    /// Load a ledger. A missing file is an empty ledger — the strictest state,
    /// which is the right default for a core that has not been run yet.
    pub(crate) fn load(path: &Path) -> Result<Ledger, ParseError> {
        let text = std::fs::read_to_string(path).unwrap_or_default();
        let entries = parse(&text)?;
        Ok(Ledger {
            path: path.to_path_buf(),
            entries,
        })
    }

    /// Does the ledger excuse this failure?
    pub(crate) fn excuses(&self, opcode: u8, vector: &str) -> bool {
        self.entries
            .iter()
            .any(|e| e.opcode == opcode && e.vector.as_deref().is_none_or(|name| name == vector))
    }

    /// Entries covering an opcode.
    pub(crate) fn entries_for(&self, opcode: u8) -> impl Iterator<Item = &Entry> {
        self.entries.iter().filter(move |e| e.opcode == opcode)
    }
}

fn parse(text: &str) -> Result<Vec<Entry>, ParseError> {
    let mut out: Vec<Entry> = Vec::new();
    for (i, raw) in text.lines().enumerate() {
        let line = i + 1;
        let (body, note) = match raw.split_once('#') {
            Some((body, note)) => (body.trim(), note.trim()),
            None => (raw.trim(), ""),
        };
        if body.is_empty() {
            continue;
        }
        let (op_text, vector) = match body.split_once("::") {
            Some((op, name)) => (op.trim(), Some(name.trim().to_string())),
            None => (body, None),
        };
        let opcode = u8::from_str_radix(op_text, 16).map_err(|_| ParseError {
            line,
            msg: format!("{op_text:?} is not a two-digit hex opcode"),
        })?;
        if op_text.len() != 2 {
            return Err(ParseError {
                line,
                msg: format!("opcode {op_text:?} must be written as two hex digits"),
            });
        }
        if note.is_empty() {
            return Err(ParseError {
                line,
                msg: "every entry needs a `# why` note; an unexplained entry is a forgotten one"
                    .into(),
            });
        }
        if vector.as_deref() == Some("") {
            return Err(ParseError {
                line,
                msg: "`::` with no vector name".into(),
            });
        }
        if let Some(dup) = out
            .iter()
            .find(|e| e.opcode == opcode && e.vector == vector)
        {
            return Err(ParseError {
                line,
                msg: format!("duplicates the entry on line {}", dup.line),
            });
        }
        out.push(Entry {
            opcode,
            vector,
            note: note.to_string(),
            line,
        });
    }
    Ok(out)
}

/// What the ledger says about a completed run.
#[derive(Debug, Default)]
pub(crate) struct Verdict {
    /// Failures nobody has excused, as `(opcode, vector name)`.
    pub(crate) unexcused: Vec<(u8, String)>,
    /// Entries that no longer match a failure — the ledger must shrink.
    pub(crate) stale: Vec<Entry>,
    /// Failures the ledger excused.
    pub(crate) excused: usize,
}

impl Verdict {
    /// Is the run acceptable?
    pub(crate) fn is_ok(&self) -> bool {
        self.unexcused.is_empty() && self.stale.is_empty()
    }

    /// A report a human can act on.
    pub(crate) fn describe(&self, ledger: &Ledger) -> String {
        let mut s = String::new();
        if !self.unexcused.is_empty() {
            let _ = writeln!(s, "{} unexcused failure(s):", self.unexcused.len());
            for (op, name) in self.unexcused.iter().take(40) {
                let _ = writeln!(s, "  {op:02x} :: {name}");
            }
            if self.unexcused.len() > 40 {
                let _ = writeln!(s, "  ... and {} more", self.unexcused.len() - 40);
            }
            let _ = writeln!(
                s,
                "Fix the core. If a failure is genuinely expected, add a line to {} \
                 with a note saying why.",
                ledger.path.display()
            );
        }
        if !self.stale.is_empty() {
            let _ = writeln!(
                s,
                "\n{} ledger entr(y/ies) now pass — the ledger only ever shrinks, \
                 so delete them from {}:",
                self.stale.len(),
                ledger.path.display()
            );
            for e in &self.stale {
                let _ = writeln!(
                    s,
                    "  line {}: {:02x}{}",
                    e.line,
                    e.opcode,
                    match &e.vector {
                        Some(v) => format!(" :: {v}"),
                        None => String::new(),
                    }
                );
            }
        }
        s
    }
}

/// Compare a run against the ledger.
///
/// `ran` is the set of opcodes that were actually executed; entries for opcodes
/// outside it are left alone rather than reported stale.
pub(crate) fn judge(ledger: &Ledger, ran: &[u8], failures: &[(u8, String)]) -> Verdict {
    let mut v = Verdict::default();
    for (opcode, name) in failures {
        if ledger.excuses(*opcode, name) {
            v.excused += 1;
        } else {
            v.unexcused.push((*opcode, name.clone()));
        }
    }
    for entry in &ledger.entries {
        if !ran.contains(&entry.opcode) {
            continue;
        }
        let matched = failures.iter().any(|(op, name)| {
            *op == entry.opcode && entry.vector.as_deref().is_none_or(|v| v == name)
        });
        if !matched {
            v.stale.push(entry.clone());
        }
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ledger_of(text: &str) -> Ledger {
        Ledger {
            path: PathBuf::from("<test>"),
            entries: parse(text).unwrap(),
        }
    }

    #[test]
    fn comments_and_blank_lines_are_ignored() {
        let l = ledger_of("# header\n\n   \na9  # because\n");
        assert_eq!(l.entries.len(), 1);
        assert_eq!(l.entries[0].opcode, 0xa9);
        assert_eq!(l.entries[0].note, "because");
        assert_eq!(l.entries[0].line, 4);
    }

    #[test]
    fn a_vector_scoped_entry_only_excuses_that_vector() {
        let l = ledger_of("8b :: 8b 12 34  # unstable magic constant\n");
        assert!(l.excuses(0x8b, "8b 12 34"));
        assert!(!l.excuses(0x8b, "8b 56 78"));
        assert!(!l.excuses(0xab, "8b 12 34"));
    }

    #[test]
    fn an_opcode_entry_excuses_every_vector_of_it() {
        let l = ledger_of("ab  # LXA: analog behaviour\n");
        assert!(l.excuses(0xab, "anything at all"));
    }

    #[test]
    fn an_entry_without_a_note_is_rejected() {
        assert!(parse("a9\n").is_err());
    }

    #[test]
    fn malformed_entries_are_rejected() {
        assert!(parse("zz # nope\n").is_err());
        assert!(parse("9 # single digit\n").is_err());
        assert!(parse("8b :: # empty name\n").is_err());
        assert!(parse("a9 # one\na9 # two\n").is_err());
    }

    #[test]
    fn an_unexcused_failure_fails_the_run() {
        let l = ledger_of("");
        let v = judge(&l, &[0xa9], &[(0xa9, "a9 00 01".into())]);
        assert!(!v.is_ok());
        assert_eq!(v.unexcused.len(), 1);
        assert!(v.describe(&l).contains("a9 :: a9 00 01"));
    }

    #[test]
    fn an_excused_failure_passes_the_run() {
        let l = ledger_of("a9 # known bad\n");
        let v = judge(&l, &[0xa9], &[(0xa9, "a9 00 01".into())]);
        assert!(v.is_ok());
        assert_eq!(v.excused, 1);
    }

    #[test]
    fn a_ledger_entry_that_now_passes_fails_the_run() {
        // This is the half that makes "only ever shrinks" real.
        let l = ledger_of("a9 # was broken once\n");
        let v = judge(&l, &[0xa9], &[]);
        assert!(!v.is_ok());
        assert_eq!(v.stale.len(), 1);
        assert!(v.describe(&l).contains("only ever shrinks"));
    }

    #[test]
    fn entries_for_opcodes_that_did_not_run_are_left_alone() {
        let l = ledger_of("8b # unstable\n");
        let v = judge(&l, &[0xa9], &[]);
        assert!(
            v.is_ok(),
            "narrowing a run must not condemn the rest of the ledger"
        );
    }

    #[test]
    fn a_missing_file_loads_as_an_empty_ledger() {
        let l = Ledger::load(Path::new("/nonexistent/ledger-3f9a.txt")).unwrap();
        assert!(l.entries.is_empty());
    }
}
