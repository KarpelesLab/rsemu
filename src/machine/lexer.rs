//! The hand-written lexer for `.machine` files.
//!
//! No generator and no regex — the dependency budget is zero (`ROADMAP.md` §0)
//! and the token set is small enough that a cursor over `&str` is the clearest
//! implementation anyway.
//!
//! # Deliberate decisions
//!
//! * **Keywords are not reserved.** `machine`, `object`, `size` and friends are
//!   ordinary [`TokenKind::Ident`]s; the parser gives them meaning by position.
//!   `size` is a keyword in `map cpubus 0 size 2K` *and* a property name in
//!   `object ram "wram" { size = 2K }`, so reserving words would break the
//!   language shown in §5.
//! * **Hyphens can appear inside identifiers.** `unassigned = open-bus` is in
//!   the §5 example, so `-` continues an identifier when it sits directly
//!   between an identifier character and an ASCII letter. The cost is that
//!   binary subtraction needs a space or a parenthesis (`n - 1`, not `n-1`) —
//!   `n-m` lexes as one identifier. That is the same trade CSS makes, and it is
//!   the only ambiguity in the token grammar.
//! * **Newlines are not tokens.** Property and statement separators are
//!   optional commas; the grammar is written so that no expression can run past
//!   the end of a line and swallow the next statement (see
//!   [`parser`](super::parser)).
//! * **No floats.** Determinism forbids them in the time path (`CLAUDE.md`), so
//!   the language has integers and exact rationals (`236250000/11`) and nothing
//!   that rounds.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::machine::diag::Diagnostic;
use crate::machine::span::{SourceFile, Span};

/// The base an integer literal was written in.
///
/// Kept so that a printer can reproduce `0x2000` as `0x2000` rather than
/// `8192`: an address written in hex is a decision by the person who wrote it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Radix {
    /// `0b1010`
    Bin,
    /// `0o755`
    Oct,
    /// `1234`
    Dec,
    /// `0x2000`
    Hex,
}

/// A binary size suffix, all powers of 1024.
///
/// 1024 rather than 1000 because these describe memories: `-p ram=8G` in §2
/// means 8 GiB, and nobody has ever wanted a 8 000 000 000-byte DIMM.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SizeUnit {
    /// `B` — bytes, scale 1.
    Byte,
    /// `K`, `KB`, `Ki`, `KiB` — 1024.
    Kilo,
    /// `M`, `MB`, `Mi`, `MiB` — 1024².
    Mega,
    /// `G`, `GB`, `Gi`, `GiB` — 1024³.
    Giga,
    /// `T`, `TB`, `Ti`, `TiB` — 1024⁴.
    Tera,
}

impl SizeUnit {
    /// Bytes per unit.
    pub const fn scale(self) -> u64 {
        match self {
            SizeUnit::Byte => 1,
            SizeUnit::Kilo => 1 << 10,
            SizeUnit::Mega => 1 << 20,
            SizeUnit::Giga => 1 << 30,
            SizeUnit::Tera => 1 << 40,
        }
    }
}

/// A duration suffix. Values are normalised to nanoseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DurationUnit {
    /// `ns`
    Nanos,
    /// `us`
    Micros,
    /// `ms`
    Millis,
    /// `s`
    Secs,
}

impl DurationUnit {
    /// Nanoseconds per unit.
    pub const fn scale(self) -> u64 {
        match self {
            DurationUnit::Nanos => 1,
            DurationUnit::Micros => 1_000,
            DurationUnit::Millis => 1_000_000,
            DurationUnit::Secs => 1_000_000_000,
        }
    }
}

/// What a numeric literal's suffix said it was.
///
/// The property system (`ROADMAP.md` §4.4) distinguishes an int from a size
/// from a duration, so the lexer records which one was written instead of
/// leaving the resolver to guess from magnitude.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NumUnit {
    /// No suffix: a plain integer, and by convention an address when written in
    /// hex.
    None,
    /// A size suffix; [`NumLit::value`] is in bytes.
    Size(SizeUnit),
    /// A duration suffix; [`NumLit::value`] is in nanoseconds.
    Duration(DurationUnit),
}

/// An integer literal, with the suffix already applied.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NumLit {
    /// The value after scaling by the suffix: bytes for a size, nanoseconds for
    /// a duration, the number itself otherwise.
    pub value: u64,
    /// The digits as written, before any suffix was applied.
    pub digits: u64,
    /// The base the digits were written in.
    pub radix: Radix,
    /// The suffix, if any.
    pub unit: NumUnit,
}

/// One token's kind and payload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum TokenKind {
    /// A bare word, possibly hyphenated: `cpubus`, `open-bus`, `mos6502`.
    Ident(String),
    /// A double-quoted string, with escapes already resolved.
    Str(String),
    /// An integer literal.
    Num(NumLit),
    /// `{`
    LBrace,
    /// `}`
    RBrace,
    /// `(`
    LParen,
    /// `)`
    RParen,
    /// `[`
    LBracket,
    /// `]`
    RBracket,
    /// `,`
    Comma,
    /// `=`
    Eq,
    /// `->`
    Arrow,
    /// `.`
    Dot,
    /// `..`
    DotDot,
    /// `..=`
    DotDotEq,
    /// `+`
    Plus,
    /// `-`
    Minus,
    /// `*`
    Star,
    /// `/`
    Slash,
    /// `%`
    Percent,
    /// `$`, which introduces a substitution inside a name (`cpu$i`).
    Dollar,
    /// The end of the file. Always the last token, so the parser never has to
    /// check for an empty stream.
    Eof,
}

impl TokenKind {
    /// The token as it would be written, for `expected …` messages.
    ///
    /// Returns `None` for tokens whose text depends on the payload.
    pub const fn symbol(&self) -> Option<&'static str> {
        Some(match self {
            TokenKind::LBrace => "{",
            TokenKind::RBrace => "}",
            TokenKind::LParen => "(",
            TokenKind::RParen => ")",
            TokenKind::LBracket => "[",
            TokenKind::RBracket => "]",
            TokenKind::Comma => ",",
            TokenKind::Eq => "=",
            TokenKind::Arrow => "->",
            TokenKind::Dot => ".",
            TokenKind::DotDot => "..",
            TokenKind::DotDotEq => "..=",
            TokenKind::Plus => "+",
            TokenKind::Minus => "-",
            TokenKind::Star => "*",
            TokenKind::Slash => "/",
            TokenKind::Percent => "%",
            TokenKind::Dollar => "$",
            _ => return None,
        })
    }

    /// How a diagnostic refers to this token: `` `}` ``, `` `cpubus` ``,
    /// `a string literal`, `end of file`.
    pub fn describe(&self) -> String {
        match self {
            TokenKind::Ident(name) => format!("`{name}`"),
            TokenKind::Str(_) => "a string literal".to_string(),
            TokenKind::Num(_) => "a number".to_string(),
            TokenKind::Eof => "end of file".to_string(),
            other => match other.symbol() {
                Some(sym) => format!("`{sym}`"),
                // Unreachable: every non-payload kind above has a symbol.
                None => "a token".to_string(),
            },
        }
    }
}

/// A token and where it was written.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Token {
    /// Kind and payload.
    pub kind: TokenKind,
    /// Source range, comments and whitespace excluded.
    pub span: Span,
}

/// Turn source text into a token vector ending in [`TokenKind::Eof`].
///
/// Eager rather than streaming: machine files are small, the parser wants
/// lookahead, and a `Vec` makes the parser a plain index. Returns the first
/// lexical error, which is also the only one — see [`Diagnostic`].
pub fn tokenize(src: &SourceFile<'_>) -> Result<Vec<Token>, Diagnostic> {
    Lexer::new(src)?.run()
}

/// The cursor. `pos` is always on a UTF-8 character boundary.
struct Lexer<'a> {
    text: &'a str,
    pos: usize,
}

impl<'a> Lexer<'a> {
    fn new(src: &SourceFile<'a>) -> Result<Self, Diagnostic> {
        if u32::try_from(src.text().len()).is_err() {
            return Err(Diagnostic::new(
                Span::at(0),
                "machine description is larger than 4 GiB",
            ));
        }
        Ok(Lexer {
            text: src.text(),
            pos: 0,
        })
    }

    fn peek(&self) -> Option<char> {
        self.text[self.pos..].chars().next()
    }

    fn peek_nth(&self, n: usize) -> Option<char> {
        self.text[self.pos..].chars().nth(n)
    }

    fn bump(&mut self) -> Option<char> {
        let c = self.peek()?;
        self.pos += c.len_utf8();
        Some(c)
    }

    fn eat(&mut self, want: char) -> bool {
        if self.peek() == Some(want) {
            self.pos += want.len_utf8();
            true
        } else {
            false
        }
    }

    /// A span from `start` to the cursor. `start` and `pos` are both bounded by
    /// the length check in [`Lexer::new`], so the casts cannot truncate.
    fn span_from(&self, start: usize) -> Span {
        Span::new(start as u32, self.pos as u32)
    }

    fn run(mut self) -> Result<Vec<Token>, Diagnostic> {
        let mut out = Vec::new();
        loop {
            self.skip_trivia();
            let start = self.pos;
            let Some(c) = self.peek() else {
                out.push(Token {
                    kind: TokenKind::Eof,
                    span: self.span_from(start),
                });
                return Ok(out);
            };
            let kind = match c {
                '{' => self.punct(TokenKind::LBrace),
                '}' => self.punct(TokenKind::RBrace),
                '(' => self.punct(TokenKind::LParen),
                ')' => self.punct(TokenKind::RParen),
                '[' => self.punct(TokenKind::LBracket),
                ']' => self.punct(TokenKind::RBracket),
                ',' => self.punct(TokenKind::Comma),
                '=' => self.punct(TokenKind::Eq),
                '+' => self.punct(TokenKind::Plus),
                '*' => self.punct(TokenKind::Star),
                '/' => self.punct(TokenKind::Slash),
                '%' => self.punct(TokenKind::Percent),
                '$' => self.punct(TokenKind::Dollar),
                '-' => {
                    self.pos += 1;
                    if self.eat('>') {
                        TokenKind::Arrow
                    } else {
                        TokenKind::Minus
                    }
                }
                '.' => {
                    self.pos += 1;
                    if self.eat('.') {
                        if self.eat('=') {
                            TokenKind::DotDotEq
                        } else {
                            TokenKind::DotDot
                        }
                    } else {
                        TokenKind::Dot
                    }
                }
                '"' => self.lex_string()?,
                c if c.is_ascii_digit() => self.lex_number()?,
                c if is_ident_start(c) => self.lex_ident(),
                other => {
                    self.pos += other.len_utf8();
                    return Err(Diagnostic::new(
                        self.span_from(start),
                        format!("unexpected character `{other}`"),
                    ));
                }
            };
            out.push(Token {
                kind,
                span: self.span_from(start),
            });
        }
    }

    /// Consume a one-character token.
    fn punct(&mut self, kind: TokenKind) -> TokenKind {
        self.pos += 1;
        kind
    }

    /// Whitespace and `#` comments. There is no block comment: a machine file
    /// is line-oriented and `#` is what §5's example uses.
    fn skip_trivia(&mut self) {
        loop {
            match self.peek() {
                Some(c) if c.is_whitespace() => {
                    self.pos += c.len_utf8();
                }
                Some('#') => {
                    while let Some(c) = self.peek() {
                        if c == '\n' {
                            break;
                        }
                        self.pos += c.len_utf8();
                    }
                }
                _ => return,
            }
        }
    }

    fn lex_ident(&mut self) -> TokenKind {
        let start = self.pos;
        self.pos += 1; // the start character is ASCII by `is_ident_start`.
        loop {
            match self.peek() {
                Some(c) if is_ident_continue(c) => self.pos += 1,
                // `open-bus` is one identifier; `n - 1` is arithmetic. The
                // hyphen joins only when a letter follows it directly.
                Some('-') if self.peek_nth(1).is_some_and(|c| c.is_ascii_alphabetic()) => {
                    self.pos += 1;
                }
                _ => break,
            }
        }
        TokenKind::Ident(self.text[start..self.pos].to_string())
    }

    fn lex_string(&mut self) -> Result<TokenKind, Diagnostic> {
        let open = self.pos;
        self.pos += 1; // the opening quote
        let mut value = String::new();
        loop {
            let Some(c) = self.bump() else {
                return Err(Diagnostic::new(
                    Span::new(open as u32, open as u32 + 1),
                    "unterminated string literal",
                ));
            };
            match c {
                '"' => return Ok(TokenKind::Str(value)),
                // A string never spans lines: the missing quote is far more
                // likely than a deliberate multi-line class name, and stopping
                // here keeps the error near the mistake.
                '\n' => {
                    return Err(Diagnostic::new(
                        Span::new(open as u32, open as u32 + 1),
                        "unterminated string literal",
                    ));
                }
                '\\' => value.push(self.lex_escape()?),
                other => value.push(other),
            }
        }
    }

    /// The body of an escape sequence, cursor just past the backslash.
    fn lex_escape(&mut self) -> Result<char, Diagnostic> {
        let start = self.pos - 1;
        let Some(c) = self.bump() else {
            return Err(Diagnostic::new(
                self.span_from(start),
                "unterminated string literal",
            ));
        };
        Ok(match c {
            'n' => '\n',
            'r' => '\r',
            't' => '\t',
            '0' => '\0',
            '\\' => '\\',
            '"' => '"',
            'x' => {
                let mut value: u32 = 0;
                for _ in 0..2 {
                    let Some(d) = self.peek().and_then(|c| c.to_digit(16)) else {
                        return Err(Diagnostic::new(
                            self.span_from(start),
                            "`\\x` needs exactly two hexadecimal digits",
                        ));
                    };
                    self.pos += 1;
                    value = value * 16 + d;
                }
                if value > 0x7f {
                    return Err(Diagnostic::new(
                        self.span_from(start),
                        "`\\x` escapes are limited to `\\x00`-`\\x7f`; write the character itself",
                    ));
                }
                char::from_u32(value).unwrap_or('\0')
            }
            other => {
                return Err(Diagnostic::new(
                    self.span_from(start),
                    format!("unknown escape `\\{other}`"),
                ));
            }
        })
    }

    fn lex_number(&mut self) -> Result<TokenKind, Diagnostic> {
        let start = self.pos;
        let mut radix = Radix::Dec;
        if self.peek() == Some('0') {
            match self.peek_nth(1) {
                Some('x' | 'X') => radix = Radix::Hex,
                Some('b' | 'B') => radix = Radix::Bin,
                Some('o' | 'O') => radix = Radix::Oct,
                _ => {}
            }
            if radix != Radix::Dec {
                self.pos += 2;
            }
        }
        let base = match radix {
            Radix::Bin => 2,
            Radix::Oct => 8,
            Radix::Dec => 10,
            Radix::Hex => 16,
        };

        let digits_start = self.pos;
        let mut digits: u64 = 0;
        let mut any = false;
        loop {
            match self.peek() {
                // `_` is a separator anywhere after the first digit.
                Some('_') if any => self.pos += 1,
                Some(c) => {
                    let Some(d) = c.to_digit(16).filter(|_| c.is_ascii_alphanumeric()) else {
                        break;
                    };
                    if d >= base {
                        // `0b12`: stop here rather than lexing `1` and `2` as
                        // two numbers, which would produce a baffling parse
                        // error further along.
                        if c.is_ascii_digit() {
                            let span = Span::new(self.pos as u32, self.pos as u32 + 1);
                            return Err(Diagnostic::new(
                                span,
                                format!("invalid digit `{c}` in {} literal", radix_name(radix)),
                            ));
                        }
                        break; // a letter: the suffix starts here
                    }
                    self.pos += 1;
                    any = true;
                    digits = match digits
                        .checked_mul(u64::from(base))
                        .and_then(|n| n.checked_add(u64::from(d)))
                    {
                        Some(n) => n,
                        None => {
                            self.skip_number_tail();
                            return Err(Diagnostic::new(
                                self.span_from(start),
                                "integer literal does not fit in 64 bits",
                            ));
                        }
                    };
                }
                None => break,
            }
        }
        if !any {
            self.skip_number_tail();
            return Err(Diagnostic::new(
                self.span_from(start),
                format!(
                    "expected digits after `{}`",
                    &self.text[start..digits_start]
                ),
            ));
        }

        // The suffix is the whole trailing run of word characters, matched as
        // a unit: `1ms` is a millisecond, never a mebibyte followed by `s`, and
        // `2Kb2` is one bad suffix rather than two adjacent numbers.
        let suffix_start = self.pos;
        while let Some(c) = self.peek() {
            if c.is_ascii_alphanumeric() || c == '_' {
                self.pos += 1;
            } else {
                break;
            }
        }
        let suffix = &self.text[suffix_start..self.pos];
        let suffix_span = Span::new(suffix_start as u32, self.pos as u32);
        let unit = if suffix.is_empty() {
            NumUnit::None
        } else if radix != Radix::Dec {
            return Err(Diagnostic::new(
                suffix_span,
                format!(
                    "suffix `{suffix}` is only allowed on decimal literals; write `{}` in decimal",
                    &self.text[start..suffix_start]
                ),
            ));
        } else {
            match unit_for(suffix) {
                Some(u) => u,
                None => {
                    return Err(Diagnostic::new(
                        suffix_span,
                        format!(
                            "unknown suffix `{suffix}`; expected a size (`K`, `M`, `G`, `T`) or a duration (`ns`, `us`, `ms`, `s`)"
                        ),
                    ));
                }
            }
        };

        let scale = match unit {
            NumUnit::None => 1,
            NumUnit::Size(u) => u.scale(),
            NumUnit::Duration(u) => u.scale(),
        };
        let Some(value) = digits.checked_mul(scale) else {
            return Err(Diagnostic::new(
                self.span_from(start),
                "integer literal does not fit in 64 bits once its suffix is applied",
            ));
        };

        Ok(TokenKind::Num(NumLit {
            value,
            digits,
            radix,
            unit,
        }))
    }

    /// Swallow the rest of a malformed number so its span covers what the user
    /// wrote, not just the prefix that parsed.
    fn skip_number_tail(&mut self) {
        while let Some(c) = self.peek() {
            if c.is_ascii_alphanumeric() || c == '_' {
                self.pos += 1;
            } else {
                break;
            }
        }
    }
}

/// The suffix table. Durations are matched exactly and first, so that `ms` is
/// milliseconds rather than a case-folded `MB`; sizes are case-insensitive
/// because `4M` and `4m` are the same DIMM.
fn unit_for(suffix: &str) -> Option<NumUnit> {
    let duration = match suffix {
        "ns" => Some(DurationUnit::Nanos),
        "us" => Some(DurationUnit::Micros),
        "ms" => Some(DurationUnit::Millis),
        "s" => Some(DurationUnit::Secs),
        _ => None,
    };
    if let Some(d) = duration {
        return Some(NumUnit::Duration(d));
    }
    let mut folded = String::with_capacity(suffix.len());
    for c in suffix.chars() {
        folded.push(c.to_ascii_lowercase());
    }
    let size = match folded.as_str() {
        "b" => SizeUnit::Byte,
        "k" | "kb" | "ki" | "kib" => SizeUnit::Kilo,
        "m" | "mb" | "mi" | "mib" => SizeUnit::Mega,
        "g" | "gb" | "gi" | "gib" => SizeUnit::Giga,
        "t" | "tb" | "ti" | "tib" => SizeUnit::Tera,
        _ => return None,
    };
    Some(NumUnit::Size(size))
}

/// The word a diagnostic uses for a base.
fn radix_name(radix: Radix) -> &'static str {
    match radix {
        Radix::Bin => "a binary",
        Radix::Oct => "an octal",
        Radix::Dec => "a decimal",
        Radix::Hex => "a hexadecimal",
    }
}

/// Whether `c` may start an identifier.
fn is_ident_start(c: char) -> bool {
    c.is_ascii_alphabetic() || c == '_'
}

/// Whether `c` may continue an identifier, hyphens aside.
fn is_ident_continue(c: char) -> bool {
    c.is_ascii_alphanumeric() || c == '_'
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kinds(text: &str) -> Vec<TokenKind> {
        let src = SourceFile::new("t", text);
        tokenize(&src)
            .expect("should lex")
            .into_iter()
            .map(|t| t.kind)
            .collect()
    }

    fn error(text: &str) -> String {
        let src = SourceFile::new("t", text);
        tokenize(&src).expect_err("should fail").message
    }

    fn num(text: &str) -> NumLit {
        match &kinds(text)[0] {
            TokenKind::Num(n) => *n,
            other => panic!("not a number: {other:?}"),
        }
    }

    #[test]
    fn punctuation_and_the_arrow() {
        assert_eq!(
            kinds("{ } ( ) [ ] , = -> . .. ..= + - * / % $"),
            alloc::vec![
                TokenKind::LBrace,
                TokenKind::RBrace,
                TokenKind::LParen,
                TokenKind::RParen,
                TokenKind::LBracket,
                TokenKind::RBracket,
                TokenKind::Comma,
                TokenKind::Eq,
                TokenKind::Arrow,
                TokenKind::Dot,
                TokenKind::DotDot,
                TokenKind::DotDotEq,
                TokenKind::Plus,
                TokenKind::Minus,
                TokenKind::Star,
                TokenKind::Slash,
                TokenKind::Percent,
                TokenKind::Dollar,
                TokenKind::Eof,
            ]
        );
    }

    #[test]
    fn comments_run_to_end_of_line() {
        assert_eq!(
            kinds("a # comment ) ] } \n b"),
            alloc::vec![
                TokenKind::Ident("a".to_string()),
                TokenKind::Ident("b".to_string()),
                TokenKind::Eof
            ]
        );
        // A comment at end of file needs no newline to close it.
        assert_eq!(kinds("# only a comment"), alloc::vec![TokenKind::Eof]);
    }

    #[test]
    fn identifiers_may_contain_hyphens_between_letters() {
        assert_eq!(
            kinds("open-bus"),
            alloc::vec![TokenKind::Ident("open-bus".to_string()), TokenKind::Eof]
        );
        // Digits do not attach: `n-1` is subtraction.
        assert_eq!(
            kinds("n-1"),
            alloc::vec![
                TokenKind::Ident("n".to_string()),
                TokenKind::Minus,
                TokenKind::Num(NumLit {
                    value: 1,
                    digits: 1,
                    radix: Radix::Dec,
                    unit: NumUnit::None,
                }),
                TokenKind::Eof
            ]
        );
        // And a trailing hyphen is not swallowed.
        assert_eq!(
            kinds("a -> b"),
            alloc::vec![
                TokenKind::Ident("a".to_string()),
                TokenKind::Arrow,
                TokenKind::Ident("b".to_string()),
                TokenKind::Eof
            ]
        );
    }

    #[test]
    fn numbers_in_every_base_with_separators() {
        assert_eq!(num("1234").value, 1234);
        assert_eq!(num("0x2000").value, 0x2000);
        assert_eq!(num("0X2000").radix, Radix::Hex);
        assert_eq!(num("0b1010").value, 0b1010);
        assert_eq!(num("0o755").value, 0o755);
        assert_eq!(num("236_250_000").value, 236_250_000);
        assert_eq!(num("0xdead_beef").value, 0xdead_beef);
    }

    #[test]
    fn size_and_duration_suffixes_scale_the_value() {
        assert_eq!(num("2K").value, 2048);
        assert_eq!(num("2K").digits, 2);
        assert_eq!(num("2K").unit, NumUnit::Size(SizeUnit::Kilo));
        assert_eq!(num("4M").value, 4 << 20);
        assert_eq!(num("8G").value, 8 << 30);
        assert_eq!(num("1T").value, 1 << 40);
        assert_eq!(num("512KiB").value, 512 * 1024);
        assert_eq!(num("4m").unit, NumUnit::Size(SizeUnit::Mega));
        // `ms` is a duration, not a case-folded `MB`.
        assert_eq!(num("1ms").unit, NumUnit::Duration(DurationUnit::Millis));
        assert_eq!(num("1ms").value, 1_000_000);
        assert_eq!(num("2s").value, 2_000_000_000);
        assert_eq!(num("100ns").value, 100);
    }

    #[test]
    fn strings_resolve_escapes() {
        assert_eq!(
            kinds(r#""a\tb\n\"c\\\x41""#),
            alloc::vec![TokenKind::Str("a\tb\n\"c\\A".to_string()), TokenKind::Eof]
        );
    }

    #[test]
    fn lexical_errors_say_what_is_wrong() {
        assert_eq!(error("\"abc"), "unterminated string literal");
        assert_eq!(error("\"abc\n\""), "unterminated string literal");
        assert_eq!(error(r#""a\q""#), "unknown escape `\\q`");
        assert_eq!(
            error(r#""a\xZZ""#),
            "`\\x` needs exactly two hexadecimal digits"
        );
        assert_eq!(error("0b12"), "invalid digit `2` in a binary literal");
        assert_eq!(error("0x"), "expected digits after `0x`");
        assert_eq!(
            error("18446744073709551616"),
            "integer literal does not fit in 64 bits"
        );
        assert_eq!(
            error("18446744073709551615K"),
            "integer literal does not fit in 64 bits once its suffix is applied"
        );
        assert!(error("12qux").starts_with("unknown suffix `qux`"));
        assert!(error("0x10K").starts_with("suffix `K` is only allowed on decimal literals"));
        assert_eq!(error("@"), "unexpected character `@`");
    }

    #[test]
    fn spans_cover_exactly_the_token() {
        let text = "  cpubus  ";
        let src = SourceFile::new("t", text);
        let toks = tokenize(&src).expect("should lex");
        assert_eq!(toks[0].span, Span::new(2, 8));
        assert_eq!(&text[2..8], "cpubus");
        // Eof is an empty span at the end of the file.
        assert_eq!(toks[1].span, Span::at(10));
    }

    #[test]
    fn non_ascii_never_splits_a_character() {
        // The cursor must stay on character boundaries even when it gives up.
        let err = error("# héllo\nλ");
        assert_eq!(err, "unexpected character `λ`");
    }

    #[test]
    fn describe_is_what_the_parser_prints() {
        assert_eq!(TokenKind::RBrace.describe(), "`}`");
        assert_eq!(TokenKind::Eof.describe(), "end of file");
        assert_eq!(TokenKind::Ident("cpu".to_string()).describe(), "`cpu`");
        assert_eq!(TokenKind::Str(String::new()).describe(), "a string literal");
    }
}
