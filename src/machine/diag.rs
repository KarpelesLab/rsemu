//! Diagnostics: one precise error, rendered with the offending line and a
//! caret.
//!
//! Most people meet rsemu through a syntax error (`ROADMAP.md` §5), so the
//! renderer is part of the language, not a debugging aid. Two shapes come out
//! of the same [`Diagnostic`]:
//!
//! * [`Diagnostic::render`] — the full rustc-style block, for a terminal.
//! * [`Diagnostic::to_error`] — an [`Error::Config`] whose `at` is
//!   `file:line:col` and whose `message` carries the short text plus the
//!   caret snippet, so that a bare `eprintln!("{err}")` still shows the line.
//!
//! The front end reports **one** error and stops. Error recovery would let it
//! list several, but a resynchronising parser routinely invents cascade errors,
//! and one accurate message beats four guesses.

use alloc::format;
use alloc::string::{String, ToString};

use crate::core::Error;
use crate::machine::span::{SourceFile, Span};

/// A secondary label attached to a diagnostic.
///
/// Exists for the case that matters most in a braces-and-blocks language: the
/// error is at end of file, but the *mistake* is the `{` on line 3.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Note {
    /// What this second location contributes.
    pub message: String,
    /// Where it is.
    pub span: Span,
}

/// A single, located error from the machine-description front end.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Diagnostic {
    /// The one-line message, lower-case and without a trailing period, in
    /// rustc's house style.
    pub message: String,
    /// The source range the caret points at.
    pub span: Span,
    /// An optional second location that explains the first.
    pub note: Option<Note>,
}

impl Diagnostic {
    /// A diagnostic with no secondary label.
    pub fn new(span: Span, message: impl Into<String>) -> Self {
        Diagnostic {
            message: message.into(),
            span,
            note: None,
        }
    }

    /// Attach a secondary label, consuming and returning `self`.
    #[must_use]
    pub fn with_note(mut self, span: Span, message: impl Into<String>) -> Self {
        self.note = Some(Note {
            message: message.into(),
            span,
        });
        self
    }

    /// The full terminal rendering, without a trailing newline.
    ///
    /// ```text
    /// error: expected `}`, found end of file
    ///  --> nes.machine:8:1
    ///   |
    /// 8 |
    ///   | ^
    /// ```
    pub fn render(&self, src: &SourceFile<'_>) -> String {
        self.render_in(src)
    }

    /// The same rendering, over a whole set of files.
    ///
    /// A resolver diagnostic can point at two files at once — an `include`
    /// cycle's note belongs to the file that closed the loop, not the one that
    /// opened it — so each label is located independently. For a single
    /// [`SourceFile`] this is exactly [`Diagnostic::render`].
    pub fn render_in(&self, sources: &impl Sources) -> String {
        let (src, span) = sources.locate(self.span);
        let mut out = format!("error: {}\n", self.message);
        out.push_str(&header(&src, span));
        out.push_str(&snippet(&src, span));
        if let Some(note) = &self.note {
            let (nsrc, nspan) = sources.locate(note.span);
            out.push_str(&format!("\nnote: {}\n", note.message));
            out.push_str(&header(&nsrc, nspan));
            out.push_str(&snippet(&nsrc, nspan));
        }
        // Every block ends with a newline from `snippet`; trim the last one so
        // the caller controls the final line break.
        while out.ends_with('\n') {
            out.pop();
        }
        out
    }

    /// Convert to the crate error type.
    ///
    /// `at` carries `file:line:col`, so the message deliberately omits the
    /// `-->` line that would repeat it — [`Error::Config`]'s `Display` prints
    /// `at`, then the message, then the caret block.
    pub fn to_error(&self, src: &SourceFile<'_>) -> Error {
        self.to_error_in(src)
    }

    /// The same conversion, over a whole set of files.
    ///
    /// See [`Diagnostic::render_in`] for why the two labels are located
    /// separately.
    pub fn to_error_in(&self, sources: &impl Sources) -> Error {
        let (src, span) = sources.locate(self.span);
        let mut message = self.message.clone();
        message.push('\n');
        message.push_str(&snippet(&src, span));
        if let Some(note) = &self.note {
            let (nsrc, nspan) = sources.locate(note.span);
            message.push_str(&format!(
                "note: {} (at {})\n",
                note.message,
                nsrc.position(nspan.start)
            ));
            message.push_str(&snippet(&nsrc, nspan));
        }
        while message.ends_with('\n') {
            message.pop();
        }
        Error::Config {
            at: src.position(span.start),
            message,
        }
    }
}

/// Where a [`Span`] points, for rendering.
///
/// The front end has one file and needs no indirection; the resolver splices
/// several files into one span space and does. Implementing this rather than
/// widening [`Span`] with a file index keeps every span in the AST two `u32`s,
/// which is what makes parsing a large included tree cheap.
pub trait Sources {
    /// The file `span` falls in, and `span` rebased into that file's offsets.
    ///
    /// Must never fail: a diagnostic that cannot be printed is worse than the
    /// error it describes, so an unknown span resolves to whatever file is
    /// nearest rather than to nothing.
    fn locate(&self, span: Span) -> (SourceFile<'_>, Span);
}

impl Sources for SourceFile<'_> {
    fn locate(&self, span: Span) -> (SourceFile<'_>, Span) {
        (*self, span)
    }
}

/// The ` --> file:line:col` line, padded to the line-number gutter.
fn header(src: &SourceFile<'_>, span: Span) -> String {
    let loc = src.location(span.start);
    let width = digits(loc.line);
    format!(
        "{:width$}--> {}\n",
        "",
        src.position(span.start),
        width = width
    )
}

/// The three-line gutter block: rule, source line, caret line.
fn snippet(src: &SourceFile<'_>, span: Span) -> String {
    let loc = src.location(span.start);
    let line = src.line_text(loc.line);
    let width = digits(loc.line);

    // The caret starts under `loc.col` and runs to the end of the span, or to
    // the end of this line for a span that covers several.
    let end = src.location(span.end);
    let line_chars = line.chars().count();
    let end_col = if end.line == loc.line {
        end.col as usize
    } else {
        line_chars + 1
    };
    let carets = end_col.saturating_sub(loc.col as usize).max(1);

    // Pad with the line's own leading characters so tabs stay aligned with
    // however the terminal renders them.
    let mut pad = String::new();
    for c in line.chars().take(loc.col.saturating_sub(1) as usize) {
        pad.push(if c == '\t' { '\t' } else { ' ' });
    }

    let mut out = format!("{:width$} |\n", "", width = width);
    // No trailing space on an empty line: a golden file should not depend on
    // invisible characters.
    if line.is_empty() {
        out.push_str(&format!(
            "{:>width$} |\n",
            loc.line.to_string(),
            width = width
        ));
    } else {
        out.push_str(&format!(
            "{:>width$} | {}\n",
            loc.line.to_string(),
            line,
            width = width
        ));
    }
    out.push_str(&format!("{:width$} | {}", "", pad, width = width));
    for _ in 0..carets {
        out.push('^');
    }
    out.push('\n');
    out
}

/// Decimal width of a line number, for the gutter.
fn digits(mut n: u32) -> usize {
    let mut d = 1;
    while n >= 10 {
        n /= 10;
        d += 1;
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;

    #[test]
    fn renders_message_location_and_caret() {
        let text = "machine \"nes\" {\n  param x = 1\n}\n";
        let src = SourceFile::new("nes.machine", text);
        let d = Diagnostic::new(Span::new(18, 23), "unknown statement `param`");
        assert_eq!(
            d.render(&src),
            "\
error: unknown statement `param`\n \
--> nes.machine:2:3\n  \
|\n\
2 |   param x = 1\n  \
|   ^^^^^"
        );
    }

    #[test]
    fn a_note_adds_a_second_block() {
        let text = "a {\nb\n";
        let src = SourceFile::new("m", text);
        let d = Diagnostic::new(Span::at(6), "expected `}`, found end of file")
            .with_note(Span::new(2, 3), "unclosed `{` here");
        assert_eq!(
            d.render(&src),
            "\
error: expected `}`, found end of file\n \
--> m:3:1\n  \
|\n\
3 |\n  \
| ^\n\
\n\
note: unclosed `{` here\n \
--> m:1:3\n  \
|\n\
1 | a {\n  \
|   ^"
        );
    }

    #[test]
    fn to_error_puts_the_location_in_at_and_the_caret_in_message() {
        let src = SourceFile::new("m", "x = 1\n");
        let d = Diagnostic::new(Span::new(4, 5), "bad value");
        let err = d.to_error(&src);
        match &err {
            Error::Config { at, message } => {
                assert_eq!(at, "m:1:5");
                assert_eq!(message, "bad value\n  |\n1 | x = 1\n  |     ^");
            }
            other => panic!("wrong variant: {other:?}"),
        }
        // Display is what a CLI prints, and it must be self-contained.
        assert_eq!(
            err.to_string(),
            "m:1:5: bad value\n  |\n1 | x = 1\n  |     ^"
        );
    }

    #[test]
    fn the_gutter_widens_with_the_line_number() {
        let mut text = String::new();
        for _ in 0..11 {
            text.push_str("x\n");
        }
        text.push_str("bad\n");
        let src = SourceFile::new("m", &text);
        let d = Diagnostic::new(Span::new(22, 25), "here");
        assert_eq!(
            d.render(&src),
            "error: here\n  --> m:12:1\n   |\n12 | bad\n   | ^^^"
        );
    }

    #[test]
    fn a_multi_line_span_carets_only_its_first_line() {
        let src = SourceFile::new("m", "abc\ndef\n");
        let d = Diagnostic::new(Span::new(1, 6), "spans two lines");
        assert!(d.render(&src).ends_with("1 | abc\n  |  ^^"));
    }

    #[test]
    fn tabs_in_the_indent_are_preserved_in_the_caret_padding() {
        let src = SourceFile::new("m", "\tx = 1\n");
        let d = Diagnostic::new(Span::new(1, 2), "here");
        assert!(d.render(&src).ends_with("1 | \tx = 1\n  | \t^"));
    }

    #[test]
    fn digits_counts_decimal_places() {
        assert_eq!(digits(0), 1);
        assert_eq!(digits(9), 1);
        assert_eq!(digits(10), 2);
        assert_eq!(digits(u32::MAX), 10);
    }
}
