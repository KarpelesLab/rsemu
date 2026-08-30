//! Byte-offset spans and the source text they point into.
//!
//! Every node the parser produces carries a [`Span`]. That is what makes
//! `file:line:col` plus a caret possible for *any* node (`ROADMAP.md` §5), and
//! it is also what keeps a future JSON projection lossless: the exact spelling
//! of a literal is never stored twice, it is recovered from the source through
//! its span.
//!
//! Line and column numbers are computed on demand rather than tracked while
//! lexing. Diagnostics are rare and machine files are small, so the linear scan
//! costs nothing that matters and the lexer stays a single cursor.

use alloc::string::{String, ToString};

/// A half-open byte range `[start, end)` within one source file.
///
/// Byte offsets rather than line/column pairs: the lexer has offsets for free,
/// two offsets fit in a register pair, and rendering needs the source text
/// anyway. Offsets are `u32`, which caps a machine description at 4 GiB — the
/// parser refuses anything larger rather than truncating.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Span {
    /// Byte offset of the first byte covered.
    pub start: u32,
    /// Byte offset one past the last byte covered.
    pub end: u32,
}

impl Span {
    /// A span covering `[start, end)`.
    ///
    /// `end` is clamped up to `start`, so a span is never reversed however it
    /// was built.
    pub const fn new(start: u32, end: u32) -> Self {
        Span {
            start,
            end: if end < start { start } else { end },
        }
    }

    /// An empty span at `offset`, used for "here, between two things" errors
    /// such as end of file.
    pub const fn at(offset: u32) -> Self {
        Span {
            start: offset,
            end: offset,
        }
    }

    /// Length in bytes.
    pub const fn len(self) -> u32 {
        self.end - self.start
    }

    /// Whether the span covers no bytes.
    pub const fn is_empty(self) -> bool {
        self.start == self.end
    }

    /// The smallest span covering both `self` and `other`.
    pub const fn join(self, other: Span) -> Span {
        let start = if self.start < other.start {
            self.start
        } else {
            other.start
        };
        let end = if self.end > other.end {
            self.end
        } else {
            other.end
        };
        Span::new(start, end)
    }
}

/// A value paired with the source range it came from.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Spanned<T> {
    /// The value.
    pub node: T,
    /// Where it was written.
    pub span: Span,
}

impl<T> Spanned<T> {
    /// Pair a value with a span.
    pub const fn new(node: T, span: Span) -> Self {
        Spanned { node, span }
    }
}

/// A 1-based line and column, as a human counts them.
///
/// Columns count **characters**, not bytes, so a caret under an identifier
/// that follows a non-ASCII comment still lands in the right place.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Location {
    /// 1-based line number.
    pub line: u32,
    /// 1-based column number, in characters.
    pub col: u32,
}

/// One `.machine` source file: a name for diagnostics plus its text.
///
/// The front end never touches the filesystem — `no_std` forbids it and the
/// caller owns the include search path anyway (`ROADMAP.md` §5). A file is a
/// name and a `&str`; loading is somebody else's job.
#[derive(Debug, Clone, Copy)]
pub struct SourceFile<'a> {
    name: &'a str,
    text: &'a str,
}

impl<'a> SourceFile<'a> {
    /// Wrap source text with the name to print in diagnostics.
    pub const fn new(name: &'a str, text: &'a str) -> Self {
        SourceFile { name, text }
    }

    /// The name used in diagnostics, usually a path.
    pub const fn name(&self) -> &'a str {
        self.name
    }

    /// The source text.
    pub const fn text(&self) -> &'a str {
        self.text
    }

    /// The line and column an offset falls on.
    ///
    /// An offset past the end of the file, or inside a multi-byte character,
    /// is clamped rather than rejected: a diagnostic must never be the thing
    /// that fails.
    pub fn location(&self, offset: u32) -> Location {
        let mut off = offset as usize;
        if off > self.text.len() {
            off = self.text.len();
        }
        while off > 0 && !self.text.is_char_boundary(off) {
            off -= 1;
        }
        let before = &self.text[..off];
        let line = before.bytes().filter(|b| *b == b'\n').count() + 1;
        let line_start = before.rfind('\n').map_or(0, |i| i + 1);
        let col = self.text[line_start..off].chars().count() + 1;
        Location {
            line: u32::try_from(line).unwrap_or(u32::MAX),
            col: u32::try_from(col).unwrap_or(u32::MAX),
        }
    }

    /// The text of a 1-based line, without its terminator.
    ///
    /// Returns `""` for a line past the end, which is the honest answer for an
    /// error reported at end of file.
    pub fn line_text(&self, line: u32) -> &'a str {
        if line == 0 {
            return "";
        }
        self.text.lines().nth(line as usize - 1).unwrap_or("")
    }

    /// `name:line:col` for an offset — the location half of an
    /// [`Error::Config`](crate::core::Error::Config).
    pub fn position(&self, offset: u32) -> String {
        let loc = self.location(offset);
        let mut s = String::from(self.name);
        s.push(':');
        s.push_str(&loc.line.to_string());
        s.push(':');
        s.push_str(&loc.col.to_string());
        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spans_join_and_measure() {
        let a = Span::new(3, 7);
        let b = Span::new(10, 12);
        assert_eq!(a.len(), 4);
        assert!(!a.is_empty());
        assert!(Span::at(5).is_empty());
        assert_eq!(a.join(b), Span::new(3, 12));
        assert_eq!(b.join(a), Span::new(3, 12));
        // A reversed range collapses instead of underflowing len().
        assert_eq!(Span::new(9, 4).len(), 0);
    }

    #[test]
    fn locations_are_one_based() {
        let src = SourceFile::new("m.machine", "abc\ndef\n");
        assert_eq!(src.location(0), Location { line: 1, col: 1 });
        assert_eq!(src.location(2), Location { line: 1, col: 3 });
        assert_eq!(src.location(4), Location { line: 2, col: 1 });
        assert_eq!(src.position(5), "m.machine:2:2");
    }

    #[test]
    fn columns_count_characters_not_bytes() {
        // Two-byte characters in a comment must not shift the caret.
        let src = SourceFile::new("m", "# é é\nx");
        assert_eq!(src.location(7), Location { line: 1, col: 6 });
    }

    #[test]
    fn out_of_range_offsets_clamp() {
        let src = SourceFile::new("m", "ab");
        assert_eq!(src.location(999), Location { line: 1, col: 3 });
        // Mid-character offsets round down to the character start.
        let src = SourceFile::new("m", "é");
        assert_eq!(src.location(1), Location { line: 1, col: 1 });
        assert_eq!(src.line_text(9), "");
        assert_eq!(src.line_text(0), "");
    }

    #[test]
    fn line_text_drops_terminators() {
        let src = SourceFile::new("m", "one\r\ntwo\n");
        assert_eq!(src.line_text(1), "one");
        assert_eq!(src.line_text(2), "two");
        assert_eq!(src.line_text(3), "");
    }
}
