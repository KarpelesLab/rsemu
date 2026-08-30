//! A hand-rolled JSON reader.
//!
//! Why not a crate: the dependency policy (`CLAUDE.md`) permits six first-party
//! crates and nothing else, and none of them parse JSON. The SingleStepTests
//! vector format is small, regular and machine-generated, so a reader is a few
//! hundred lines — cheaper than an exception to the rule.
//!
//! Two layers, because the two callers want different things:
//!
//! * [`Reader`] is a streaming cursor. It never allocates for structure, so the
//!   vector runner can walk an 800 MB corpus straight into its own structs.
//! * [`Value`] is a DOM built on top of it, for callers that want convenience
//!   over throughput. Objects keep **insertion order** — a `HashMap` here would
//!   put iteration order into a test's output, which is exactly what
//!   `CLAUDE.md` forbids.

use std::fmt;

/// Where in the input a parse gave up, and why.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Error {
    /// Byte offset of the failure.
    pub(crate) offset: usize,
    /// Human-readable cause.
    pub(crate) msg: String,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "JSON error at byte {}: {}", self.offset, self.msg)
    }
}

impl std::error::Error for Error {}

/// Result of a JSON operation.
pub(crate) type Result<T> = std::result::Result<T, Error>;

/// A JSON number, kept exact when it is an integer.
///
/// Vector data is integral throughout; the float arm exists so the reader is
/// honest about arbitrary JSON rather than silently truncating.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Number {
    /// An integer that fit in `i64`.
    Int(i64),
    /// Anything else.
    Float(f64),
}

impl Number {
    /// The value as an `i64`, or `None` if it was not an integer.
    pub(crate) fn as_i64(self) -> Option<i64> {
        match self {
            Number::Int(i) => Some(i),
            Number::Float(_) => None,
        }
    }
}

/// A parsed JSON document.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Value {
    /// `null`.
    Null,
    /// `true` / `false`.
    Bool(bool),
    /// A number.
    Num(Number),
    /// A string, with escapes resolved.
    Str(String),
    /// An array.
    Array(Vec<Value>),
    /// An object, in the order the members appeared.
    Object(Vec<(String, Value)>),
}

impl Value {
    /// Parse a whole document, rejecting trailing garbage.
    pub(crate) fn parse(input: &[u8]) -> Result<Value> {
        let mut r = Reader::new(input);
        let v = r.value()?;
        if !r.at_end() {
            return Err(r.err("trailing data after top-level value"));
        }
        Ok(v)
    }

    /// Look up an object member by key. `None` for non-objects.
    pub(crate) fn get(&self, key: &str) -> Option<&Value> {
        match self {
            Value::Object(members) => members.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }

    /// The string contents, if this is a string.
    pub(crate) fn as_str(&self) -> Option<&str> {
        match self {
            Value::Str(s) => Some(s),
            _ => None,
        }
    }

    /// The integer contents, if this is an integral number.
    pub(crate) fn as_i64(&self) -> Option<i64> {
        match self {
            Value::Num(n) => n.as_i64(),
            _ => None,
        }
    }

    /// The elements, if this is an array.
    pub(crate) fn as_array(&self) -> Option<&[Value]> {
        match self {
            Value::Array(items) => Some(items),
            _ => None,
        }
    }
}

/// A streaming cursor over JSON text.
///
/// The methods are deliberately primitive: the caller drives the shape of the
/// document it expects, which is both faster and a stricter check than walking
/// a DOM after the fact.
#[derive(Debug)]
pub(crate) struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    /// Wrap a byte slice. UTF-8 is validated only inside strings.
    pub(crate) fn new(buf: &'a [u8]) -> Self {
        // A UTF-8 BOM is common in the wild and illegal in JSON; skip it rather
        // than fail with a baffling message about byte 0.
        let pos = if buf.starts_with(&[0xef, 0xbb, 0xbf]) {
            3
        } else {
            0
        };
        Reader { buf, pos }
    }

    /// The current byte offset, for error reporting.
    pub(crate) fn offset(&self) -> usize {
        self.pos
    }

    /// Build an error at the current position.
    pub(crate) fn err(&self, msg: impl Into<String>) -> Error {
        Error {
            offset: self.pos,
            msg: msg.into(),
        }
    }

    /// Advance past whitespace.
    pub(crate) fn skip_ws(&mut self) {
        while let Some(&b) = self.buf.get(self.pos) {
            if b == b' ' || b == b'\t' || b == b'\n' || b == b'\r' {
                self.pos += 1;
            } else {
                break;
            }
        }
    }

    /// Is the whole input consumed?
    pub(crate) fn at_end(&mut self) -> bool {
        self.skip_ws();
        self.pos >= self.buf.len()
    }

    /// The next non-whitespace byte, without consuming it.
    pub(crate) fn peek(&mut self) -> Option<u8> {
        self.skip_ws();
        self.buf.get(self.pos).copied()
    }

    /// Consume `b` if it is next; report whether it was.
    pub(crate) fn eat(&mut self, b: u8) -> bool {
        if self.peek() == Some(b) {
            self.pos += 1;
            true
        } else {
            false
        }
    }

    /// Consume `b`, or fail.
    pub(crate) fn expect(&mut self, b: u8) -> Result<()> {
        if self.eat(b) {
            Ok(())
        } else {
            let found = match self.peek() {
                Some(c) => format!("{:?}", c as char),
                None => "end of input".to_string(),
            };
            Err(self.err(format!("expected {:?}, found {found}", b as char)))
        }
    }

    /// Read a string literal, resolving escapes.
    pub(crate) fn string(&mut self) -> Result<String> {
        self.expect(b'"')?;
        let mut out = String::new();
        loop {
            let b = *self
                .buf
                .get(self.pos)
                .ok_or_else(|| self.err("unterminated string"))?;
            self.pos += 1;
            match b {
                b'"' => return Ok(out),
                b'\\' => {
                    let e = *self
                        .buf
                        .get(self.pos)
                        .ok_or_else(|| self.err("truncated escape"))?;
                    self.pos += 1;
                    match e {
                        b'"' => out.push('"'),
                        b'\\' => out.push('\\'),
                        b'/' => out.push('/'),
                        b'b' => out.push('\u{8}'),
                        b'f' => out.push('\u{c}'),
                        b'n' => out.push('\n'),
                        b'r' => out.push('\r'),
                        b't' => out.push('\t'),
                        b'u' => out.push(self.escape_u()?),
                        other => {
                            return Err(self.err(format!("unknown escape \\{}", other as char)));
                        }
                    }
                }
                // Control characters are not allowed raw inside a JSON string.
                0x00..=0x1f => return Err(self.err("raw control character in string")),
                _ => {
                    // Copy the rest of this UTF-8 sequence verbatim.
                    let start = self.pos - 1;
                    let len = utf8_len(b).ok_or_else(|| self.err("invalid UTF-8 lead byte"))?;
                    let end = start + len;
                    let raw = self
                        .buf
                        .get(start..end)
                        .ok_or_else(|| self.err("truncated UTF-8"))?;
                    let s = std::str::from_utf8(raw).map_err(|_| self.err("invalid UTF-8"))?;
                    out.push_str(s);
                    self.pos = end;
                }
            }
        }
    }

    /// A `\uXXXX` escape, joining surrogate pairs.
    fn escape_u(&mut self) -> Result<char> {
        let hi = self.hex4()?;
        if !(0xd800..0xdc00).contains(&hi) {
            return char::from_u32(u32::from(hi)).ok_or_else(|| self.err("invalid code point"));
        }
        // High surrogate: a low surrogate must follow, or the text is not
        // representable as a Rust `char` at all.
        if self.buf.get(self.pos) != Some(&b'\\') || self.buf.get(self.pos + 1) != Some(&b'u') {
            return Err(self.err("unpaired high surrogate"));
        }
        self.pos += 2;
        let lo = self.hex4()?;
        if !(0xdc00..0xe000).contains(&lo) {
            return Err(self.err("high surrogate not followed by a low surrogate"));
        }
        let c = 0x1_0000 + ((u32::from(hi) - 0xd800) << 10) + (u32::from(lo) - 0xdc00);
        char::from_u32(c).ok_or_else(|| self.err("invalid surrogate pair"))
    }

    fn hex4(&mut self) -> Result<u16> {
        let raw = self
            .buf
            .get(self.pos..self.pos + 4)
            .ok_or_else(|| self.err("truncated \\u escape"))?;
        let mut v: u16 = 0;
        for &d in raw {
            let n = match d {
                b'0'..=b'9' => d - b'0',
                b'a'..=b'f' => d - b'a' + 10,
                b'A'..=b'F' => d - b'A' + 10,
                _ => return Err(self.err("non-hex digit in \\u escape")),
            };
            v = (v << 4) | u16::from(n);
        }
        self.pos += 4;
        Ok(v)
    }

    /// Read a number.
    pub(crate) fn number(&mut self) -> Result<Number> {
        self.skip_ws();
        let start = self.pos;
        if self.buf.get(self.pos) == Some(&b'-') {
            self.pos += 1;
        }
        let int_start = self.pos;
        while matches!(self.buf.get(self.pos), Some(b'0'..=b'9')) {
            self.pos += 1;
        }
        if self.pos == int_start {
            return Err(self.err("expected a number"));
        }
        // JSON forbids leading zeros; accepting them would let "01" parse as 1
        // and leave a stray digit that the caller would then trip over.
        if self.buf[int_start] == b'0' && self.pos - int_start > 1 {
            return Err(Error {
                offset: int_start,
                msg: "leading zero in number".into(),
            });
        }
        let mut is_float = false;
        if self.buf.get(self.pos) == Some(&b'.') {
            is_float = true;
            self.pos += 1;
            while matches!(self.buf.get(self.pos), Some(b'0'..=b'9')) {
                self.pos += 1;
            }
        }
        if matches!(self.buf.get(self.pos), Some(b'e' | b'E')) {
            is_float = true;
            self.pos += 1;
            if matches!(self.buf.get(self.pos), Some(b'+' | b'-')) {
                self.pos += 1;
            }
            while matches!(self.buf.get(self.pos), Some(b'0'..=b'9')) {
                self.pos += 1;
            }
        }
        // The slice is ASCII by construction, so from_utf8 cannot fail.
        let text = std::str::from_utf8(&self.buf[start..self.pos])
            .map_err(|_| self.err("non-ASCII number"))?;
        if !is_float && let Ok(i) = text.parse::<i64>() {
            return Ok(Number::Int(i));
        }
        text.parse::<f64>().map(Number::Float).map_err(|_| Error {
            offset: start,
            msg: format!("malformed number {text:?}"),
        })
    }

    /// Read a number that must be an unsigned integer in `0..=max`.
    ///
    /// The vector format is full of small unsigned fields; checking the range
    /// here turns a corrupt corpus into a clear message instead of a wrapped
    /// byte that silently fails a comparison much later.
    pub(crate) fn u64_in(&mut self, max: u64) -> Result<u64> {
        let at = self.pos;
        match self.number()? {
            Number::Int(i) if i >= 0 && (i as u64) <= max => Ok(i as u64),
            other => Err(Error {
                offset: at,
                msg: format!("{other:?} out of range 0..={max}"),
            }),
        }
    }

    /// Consume a literal word such as `true`.
    fn keyword(&mut self, word: &str) -> Result<()> {
        if self.buf[self.pos..].starts_with(word.as_bytes()) {
            self.pos += word.len();
            Ok(())
        } else {
            Err(self.err(format!("expected {word}")))
        }
    }

    /// Walk an array, calling `f` once per element with the element index.
    ///
    /// `f` must consume exactly one value. Returns the element count.
    pub(crate) fn array<F>(&mut self, mut f: F) -> Result<usize>
    where
        F: FnMut(&mut Reader<'a>, usize) -> Result<()>,
    {
        self.expect(b'[')?;
        let mut n = 0;
        if self.eat(b']') {
            return Ok(0);
        }
        loop {
            f(self, n)?;
            n += 1;
            if self.eat(b',') {
                continue;
            }
            self.expect(b']')?;
            return Ok(n);
        }
    }

    /// Walk an object, calling `f` once per member with the key.
    ///
    /// `f` must consume exactly one value.
    pub(crate) fn object<F>(&mut self, mut f: F) -> Result<()>
    where
        F: FnMut(&mut Reader<'a>, &str) -> Result<()>,
    {
        self.expect(b'{')?;
        if self.eat(b'}') {
            return Ok(());
        }
        loop {
            let key = self.string()?;
            self.expect(b':')?;
            f(self, &key)?;
            if self.eat(b',') {
                continue;
            }
            self.expect(b'}')?;
            return Ok(());
        }
    }

    /// Parse any value into the DOM.
    pub(crate) fn value(&mut self) -> Result<Value> {
        match self.peek() {
            None => Err(self.err("unexpected end of input")),
            Some(b'n') => {
                self.keyword("null")?;
                Ok(Value::Null)
            }
            Some(b't') => {
                self.keyword("true")?;
                Ok(Value::Bool(true))
            }
            Some(b'f') => {
                self.keyword("false")?;
                Ok(Value::Bool(false))
            }
            Some(b'"') => Ok(Value::Str(self.string()?)),
            Some(b'[') => {
                let mut items = Vec::new();
                self.array(|r, _| {
                    items.push(r.value()?);
                    Ok(())
                })?;
                Ok(Value::Array(items))
            }
            Some(b'{') => {
                let mut members = Vec::new();
                self.object(|r, k| {
                    members.push((k.to_string(), r.value()?));
                    Ok(())
                })?;
                Ok(Value::Object(members))
            }
            Some(_) => Ok(Value::Num(self.number()?)),
        }
    }

    /// Consume and discard one value of any shape.
    pub(crate) fn skip_value(&mut self) -> Result<()> {
        self.value().map(|_| ())
    }
}

/// Length in bytes of the UTF-8 sequence introduced by `b`.
fn utf8_len(b: u8) -> Option<usize> {
    match b {
        0x00..=0x7f => Some(1),
        0xc2..=0xdf => Some(2),
        0xe0..=0xef => Some(3),
        0xf0..=0xf4 => Some(4),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalars_round_trip() {
        assert_eq!(Value::parse(b"null").unwrap(), Value::Null);
        assert_eq!(Value::parse(b" true ").unwrap(), Value::Bool(true));
        assert_eq!(Value::parse(b"false").unwrap(), Value::Bool(false));
        assert_eq!(Value::parse(b"-42").unwrap().as_i64(), Some(-42));
        assert_eq!(Value::parse(b"0").unwrap().as_i64(), Some(0));
    }

    #[test]
    fn floats_stay_floats_and_integers_stay_exact() {
        // 2^53+1 is the classic value a float-only parser destroys.
        assert_eq!(
            Value::parse(b"9007199254740993").unwrap().as_i64(),
            Some(9_007_199_254_740_993)
        );
        assert!(matches!(
            Value::parse(b"1.5").unwrap(),
            Value::Num(Number::Float(_))
        ));
        assert!(matches!(
            Value::parse(b"1e3").unwrap(),
            Value::Num(Number::Float(_))
        ));
    }

    #[test]
    fn strings_resolve_escapes_including_surrogate_pairs() {
        let v = Value::parse(br#""a\nb\u0041\ud83d\ude00\\""#).unwrap();
        assert_eq!(v.as_str(), Some("a\nbA\u{1f600}\\"));
    }

    #[test]
    fn multibyte_utf8_survives_verbatim() {
        let v = Value::parse("\"héllo — 世界\"".as_bytes()).unwrap();
        assert_eq!(v.as_str(), Some("héllo — 世界"));
    }

    #[test]
    fn objects_keep_insertion_order() {
        let v = Value::parse(br#"{"z":1,"a":2,"m":3}"#).unwrap();
        let Value::Object(members) = &v else {
            panic!("not an object")
        };
        let keys: Vec<&str> = members.iter().map(|(k, _)| k.as_str()).collect();
        assert_eq!(keys, ["z", "a", "m"]);
    }

    #[test]
    fn nested_structure_parses() {
        let v = Value::parse(br#"{"a":[1,[2,{"b":null}],[]],"c":{}}"#).unwrap();
        assert_eq!(v.get("a").unwrap().as_array().unwrap().len(), 3);
        assert_eq!(v.get("c").unwrap(), &Value::Object(Vec::new()));
    }

    #[test]
    fn malformed_input_is_rejected() {
        for bad in [
            &b"{"[..],
            b"[1,]",
            b"{\"a\" 1}",
            b"\"unterminated",
            b"tru",
            b"01",          // leading zero
            b"[1] [2]",     // two top-level values
            b"\"\\q\"",     // unknown escape
            b"\"\\ud83d\"", // unpaired surrogate
            b"",            // empty
            b"[1,2",        // unterminated array
        ] {
            assert!(
                Value::parse(bad).is_err(),
                "should have rejected {:?}",
                String::from_utf8_lossy(bad)
            );
        }
    }

    #[test]
    fn errors_carry_an_offset() {
        let e = Value::parse(br#"{"a":1,"b":}"#).unwrap_err();
        assert_eq!(e.offset, 11);
        assert!(e.to_string().contains("byte 11"), "{e}");
    }

    #[test]
    fn a_bom_is_tolerated() {
        assert_eq!(
            Value::parse("\u{feff}7".as_bytes()).unwrap().as_i64(),
            Some(7)
        );
    }

    #[test]
    fn range_checked_integers_reject_out_of_range() {
        let mut r = Reader::new(b"300");
        assert!(r.u64_in(255).is_err());
        let mut r = Reader::new(b"255");
        assert_eq!(r.u64_in(255).unwrap(), 255);
        let mut r = Reader::new(b"-1");
        assert!(r.u64_in(255).is_err());
    }

    #[test]
    fn streaming_walks_without_building_a_dom() {
        let mut r = Reader::new(br#"[{"k":1},{"k":2},{"k":3}]"#);
        let mut sum = 0i64;
        let n = r
            .array(|r, _| {
                r.object(|r, key| {
                    assert_eq!(key, "k");
                    sum += r.number()?.as_i64().unwrap();
                    Ok(())
                })
            })
            .unwrap();
        assert_eq!((n, sum), (3, 6));
        assert!(r.at_end());
    }

    #[test]
    fn skip_value_consumes_exactly_one_value() {
        let mut r = Reader::new(br#"[{"a":[1,2,{"b":"c"}]},9]"#);
        r.expect(b'[').unwrap();
        r.skip_value().unwrap();
        assert!(r.eat(b','));
        assert_eq!(r.number().unwrap().as_i64(), Some(9));
        assert_eq!(r.offset(), 24);
    }
}
