//! Dynamic property values and typed extraction (`ROADMAP.md` §4.4).
//!
//! Every device is constructed from a bag of named properties: `new(props)`
//! validates and allocates, `realize(ctx)` acts (`ROADMAP.md` §4.4). This
//! module is that bag, plus the scalar syntaxes the machine-description lexer
//! (§5) turns text into.
//!
//! # Why this is not `serde`
//!
//! The dependency policy forbids it (§0), but even if it did not: the value set
//! here is ten variants and it is closed, while the thing that actually matters
//! is the error text. Most people meet rsemu through a misconfigured machine
//! file, so an extraction failure has to say **which property**, **what was
//! expected**, and **what was found** — and, where it can, guess what the user
//! meant:
//!
//! ```text
//! property `size`: expected size, found string "big"
//! unknown property `clok` (did you mean `clock`?)
//! property `width`: 65 is out of range 1..=64
//! property `engine`: expected one of `interp`, `jit`; found "intrep" (did you mean `interp`?)
//! ```
//!
//! A derive macro would give us generality we do not need in exchange for
//! messages we cannot control.
//!
//! # Shape
//!
//! - [`Value`] — the dynamic value: int, uint, bool, string, [size](Value::Size),
//!   [address](Value::Addr), [duration](Duration), list, map, [link](Link).
//! - [`Props`] — an **insertion-ordered** name → value map. Order is part of the
//!   contract: `CLAUDE.md` forbids hash iteration order from reaching anything
//!   guest-visible, and error messages that reorder themselves between runs are
//!   their own kind of bug.
//! - [`Reader`] — a borrowing cursor over a [`Props`] that remembers which names
//!   were asked for, so [`Reader::finish`] can report the ones nobody wanted.
//!   A typo'd property must be an error, never a silent default.
//! - [`parse_size`], [`parse_uint`], [`parse_int`], [`parse_bool`],
//!   [`parse_addr`], [`parse_duration`] — the scalar syntaxes, shared with §5's
//!   lexer.
//!
//! # Example
//!
//! ```
//! use rsemu::core::props::{Props, Reader, Value};
//!
//! let props = Props::new()
//!     .with("size", Value::Size(2 * 1024))
//!     .with("readonly", Value::Bool(true));
//!
//! let mut r = Reader::new(&props);
//! let size = r.require_size("size")?;
//! let ro = r.or("readonly", false)?;
//! let fill = r.or("fill", 0u64)?; // absent: the default stands
//! r.finish()?; // nothing left over, so no typo
//!
//! assert_eq!((size, ro, fill), (2048, true, 0));
//! # Ok::<(), rsemu::Error>(())
//! ```
//!
//! # Types borrowed from modules that do not exist yet
//!
//! Guest addresses will get a `GuestAddr` newtype in `core::space`, which is
//! still a stub. [`Value::Addr`] therefore carries a plain `u64` for now and
//! will be narrowed when that type lands; the accessor names already say
//! "addr" so the change is local to this file.

use alloc::borrow::ToOwned;
use alloc::format;
use alloc::string::String;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::fmt;
use core::ops::RangeInclusive;

use crate::core::error::{Error, Result};
use crate::core::hosts::{HostKind, HostObjects};

// ---------------------------------------------------------------------------
// Duration
// ---------------------------------------------------------------------------

/// A span of virtual time, in picoseconds.
///
/// Integer-only and deliberately not `core::time::Duration`: the time path
/// takes no floats (`CLAUDE.md`), and `core::time::Duration`'s split
/// seconds/nanos representation is awkward to compare and to snapshot.
///
/// Picoseconds because the unit has to express a *period* as well as a
/// timeout — a 4 GHz clock has a 250 ps period — while `u64` still covers
/// about 213 days, far more than any device-configuration span.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Duration {
    picos: u64,
}

impl Duration {
    /// A zero-length span.
    pub const ZERO: Duration = Duration { picos: 0 };

    /// The longest representable span (about 213 days).
    pub const MAX: Duration = Duration { picos: u64::MAX };

    /// Builds a duration from picoseconds.
    #[inline]
    pub const fn from_picos(picos: u64) -> Duration {
        Duration { picos }
    }

    /// Builds a duration from nanoseconds, or `None` on overflow.
    pub const fn from_nanos(nanos: u64) -> Option<Duration> {
        match nanos.checked_mul(1_000) {
            Some(picos) => Some(Duration { picos }),
            None => None,
        }
    }

    /// Builds a duration from microseconds, or `None` on overflow.
    pub const fn from_micros(micros: u64) -> Option<Duration> {
        match micros.checked_mul(1_000_000) {
            Some(picos) => Some(Duration { picos }),
            None => None,
        }
    }

    /// Builds a duration from milliseconds, or `None` on overflow.
    pub const fn from_millis(millis: u64) -> Option<Duration> {
        match millis.checked_mul(1_000_000_000) {
            Some(picos) => Some(Duration { picos }),
            None => None,
        }
    }

    /// Builds a duration from whole seconds, or `None` on overflow.
    pub const fn from_secs(secs: u64) -> Option<Duration> {
        match secs.checked_mul(1_000_000_000_000) {
            Some(picos) => Some(Duration { picos }),
            None => None,
        }
    }

    /// The span in picoseconds — the exact stored value.
    #[inline]
    pub const fn as_picos(self) -> u64 {
        self.picos
    }

    /// The span in nanoseconds, truncated towards zero.
    #[inline]
    pub const fn as_nanos(self) -> u64 {
        self.picos / 1_000
    }

    /// The span in microseconds, truncated towards zero.
    #[inline]
    pub const fn as_micros(self) -> u64 {
        self.picos / 1_000_000
    }

    /// The span in milliseconds, truncated towards zero.
    #[inline]
    pub const fn as_millis(self) -> u64 {
        self.picos / 1_000_000_000
    }

    /// The span in whole seconds, truncated towards zero.
    #[inline]
    pub const fn as_secs(self) -> u64 {
        self.picos / 1_000_000_000_000
    }
}

impl fmt::Display for Duration {
    /// Prints the largest unit that divides the span exactly, so a parsed
    /// duration round-trips through its own text (`10ms` in, `10ms` out).
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        const UNITS: [(u64, &str); 7] = [
            (3_600_000_000_000_000, "h"),
            (60_000_000_000_000, "m"),
            (1_000_000_000_000, "s"),
            (1_000_000_000, "ms"),
            (1_000_000, "us"),
            (1_000, "ns"),
            (1, "ps"),
        ];
        if self.picos == 0 {
            return f.write_str("0s");
        }
        for (scale, name) in UNITS {
            if self.picos.is_multiple_of(scale) {
                return write!(f, "{}{}", self.picos / scale, name);
            }
        }
        write!(f, "{}ps", self.picos)
    }
}

// ---------------------------------------------------------------------------
// Media
// ---------------------------------------------------------------------------

/// A blob of host-supplied media: a ROM image, a disk image, a firmware blob.
///
/// # Why media is a property and not a file name
///
/// A device needs the *bytes*, and nothing below `host/` may open a file
/// (`CLAUDE.md`: the caller owns file access, and `no_std` has no filesystem to
/// open one with). So a machine description names a **media slot** — a string
/// like `rom = "cart"` — and whoever realizes the machine binds that slot to
/// bytes it obtained however it likes: `rsemu run nes.machine --cart smb.nes`,
/// a wasm embedder handing over an `ArrayBuffer`, a test with a `const`.
/// Realize substitutes the bound blob for the slot name before the device is
/// constructed, so a device sees only [`Value::Media`] and never a path.
///
/// The bytes are behind an `Arc<[u8]>`, so cloning a `Value` holding a 4 MiB
/// ROM is a refcount bump; [`Debug`](fmt::Debug) prints the name and the length
/// rather than the contents, because a 40 KiB hex dump in an error message
/// helps nobody.
#[derive(Clone, PartialEq, Eq)]
pub struct Media {
    name: String,
    bytes: Arc<[u8]>,
}

impl Media {
    /// Media named `name`, holding `bytes`.
    pub fn new(name: impl Into<String>, bytes: impl Into<Arc<[u8]>>) -> Media {
        Media {
            name: name.into(),
            bytes: bytes.into(),
        }
    }

    /// The slot name this blob was bound to, for error messages.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The bytes.
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// A cheap clone of the byte handle.
    pub fn to_bytes(&self) -> Arc<[u8]> {
        Arc::clone(&self.bytes)
    }

    /// How many bytes there are.
    pub fn len(&self) -> u64 {
        self.bytes.len() as u64
    }

    /// Whether the blob is empty — usually a truncated download.
    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }
}

impl fmt::Debug for Media {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Media")
            .field("name", &self.name)
            .field("len", &self.bytes.len())
            .finish()
    }
}

// ---------------------------------------------------------------------------
// Link
// ---------------------------------------------------------------------------

/// A reference to another object by path, such as `cpubus` or `ppu.regs`.
///
/// Unresolved by construction: the machine-description resolver (`ROADMAP.md`
/// §5) turns links into real edges after the whole graph is built, because a
/// file may name an object before it is declared. All this type promises is
/// that the path is *syntactically* a path.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Link(String);

impl Link {
    /// Validates and wraps a path.
    ///
    /// A path is one or more dot-separated segments; a segment is a non-empty
    /// run of ASCII alphanumerics, `_` or `-`.
    pub fn new(path: impl Into<String>) -> Result<Link> {
        let path = path.into();
        if path.is_empty() {
            return Err(prop_err("a link path cannot be empty".to_owned()));
        }
        for segment in path.split('.') {
            if segment.is_empty() {
                return Err(prop_err(format!(
                    "`{path}` is not a valid link path: empty path segment"
                )));
            }
            if let Some(bad) = segment
                .chars()
                .find(|c| !c.is_ascii_alphanumeric() && *c != '_' && *c != '-')
            {
                return Err(prop_err(format!(
                    "`{path}` is not a valid link path: unexpected `{bad}` \
                     (segments are alphanumerics, `_` or `-`, separated by `.`)"
                )));
            }
        }
        Ok(Link(path))
    }

    /// The path as written.
    #[inline]
    pub fn as_str(&self) -> &str {
        &self.0
    }

    /// The dot-separated segments, in order.
    pub fn segments(&self) -> impl Iterator<Item = &str> {
        self.0.split('.')
    }

    /// The first segment — the object the path starts from.
    pub fn root(&self) -> &str {
        self.0.split('.').next().unwrap_or(&self.0)
    }
}

impl fmt::Display for Link {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

// ---------------------------------------------------------------------------
// Value
// ---------------------------------------------------------------------------

/// What kind of thing a [`Value`] is, for error messages and introspection.
///
/// `rsemu describe pci.nvme` prints the expected kind of every property, so
/// this needs a name a user can act on — "unsigned integer", not "Uint".
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ValueKind {
    /// A signed integer.
    Int,
    /// An unsigned integer.
    Uint,
    /// A boolean.
    Bool,
    /// A string.
    Str,
    /// A byte count, written with an optional binary multiplier (`512M`).
    Size,
    /// A guest address, usually written in hex.
    Addr,
    /// A span of time (`10ms`).
    Duration,
    /// An ordered list of values.
    List,
    /// A nested property map.
    Map,
    /// A reference to another object by path.
    Link,
    /// Host-supplied bytes, named in the file and bound at realize time.
    Media,
}

impl ValueKind {
    /// The name used in error messages.
    pub const fn as_str(self) -> &'static str {
        match self {
            ValueKind::Int => "signed integer",
            ValueKind::Uint => "unsigned integer",
            ValueKind::Bool => "boolean",
            ValueKind::Str => "string",
            ValueKind::Size => "size",
            ValueKind::Addr => "address",
            ValueKind::Duration => "duration",
            ValueKind::List => "list",
            ValueKind::Map => "map",
            ValueKind::Link => "link",
            ValueKind::Media => "media",
        }
    }
}

impl fmt::Display for ValueKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A property value.
///
/// Ten variants, closed on purpose (`ROADMAP.md` §4.4). [`Value::Size`],
/// [`Value::Addr`] and [`Value::Uint`] all hold a `u64` and differ only in what
/// the user wrote — which is exactly the point, because it is what the error
/// message and `rsemu describe` output say.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Value {
    /// A signed integer, as in `offset = -4`.
    Int(i64),
    /// An unsigned integer, as in `count = 4`.
    Uint(u64),
    /// A boolean, as in `readonly = true`.
    Bool(bool),
    /// A string, as in `region = "ntsc"`.
    Str(String),
    /// A byte count, as in `size = 512M`. Always in bytes once parsed.
    Size(u64),
    /// A guest address, as in `base = 0xa0000`.
    ///
    /// A plain `u64` until `core::space` grows its `GuestAddr` newtype.
    Addr(u64),
    /// A span of time, as in `timeout = 10ms`.
    Duration(Duration),
    /// An ordered list, as in `irqs = [3, 5]`.
    List(Vec<Value>),
    /// A nested map, as in `bar0 = { size = 4K }`.
    Map(Props),
    /// A reference to another object, as in `space = cpubus`.
    Link(Link),
    /// Host-supplied bytes bound to the slot a property named — see [`Media`].
    ///
    /// The only variant a machine file cannot write directly: it says
    /// `rom = "cart"` and realize substitutes the bytes bound to `cart`.
    Media(Media),
}

impl Value {
    /// The kind of this value.
    pub fn kind(&self) -> ValueKind {
        match self {
            Value::Int(_) => ValueKind::Int,
            Value::Uint(_) => ValueKind::Uint,
            Value::Bool(_) => ValueKind::Bool,
            Value::Str(_) => ValueKind::Str,
            Value::Size(_) => ValueKind::Size,
            Value::Addr(_) => ValueKind::Addr,
            Value::Duration(_) => ValueKind::Duration,
            Value::List(_) => ValueKind::List,
            Value::Map(_) => ValueKind::Map,
            Value::Link(_) => ValueKind::Link,
            Value::Media(_) => ValueKind::Media,
        }
    }

    /// Whether this value is one of the interchangeable numeric kinds.
    ///
    /// `size = 2048`, `size = 2K` and `base = 0x800` are the same thing to the
    /// user, so the numeric kinds coerce into one another (a negative
    /// [`Value::Int`] still cannot become a `u64`). Every other kind is strict:
    /// a string is never a number, however numeric it looks, because
    /// `size = "2K"` is a quoting mistake worth reporting.
    pub fn is_numeric(&self) -> bool {
        matches!(
            self,
            Value::Int(_) | Value::Uint(_) | Value::Size(_) | Value::Addr(_)
        )
    }

    /// The boolean, if this is one.
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Value::Bool(b) => Some(*b),
            _ => None,
        }
    }

    /// The signed integer, if this is a numeric value that fits in an `i64`.
    pub fn as_int(&self) -> Option<i64> {
        match self {
            Value::Int(i) => Some(*i),
            Value::Uint(u) | Value::Size(u) | Value::Addr(u) => i64::try_from(*u).ok(),
            _ => None,
        }
    }

    /// The unsigned integer, if this is a non-negative numeric value.
    pub fn as_uint(&self) -> Option<u64> {
        match self {
            Value::Uint(u) | Value::Size(u) | Value::Addr(u) => Some(*u),
            Value::Int(i) => u64::try_from(*i).ok(),
            _ => None,
        }
    }

    /// The byte count, if this is a non-negative numeric value.
    pub fn as_size(&self) -> Option<u64> {
        self.as_uint()
    }

    /// The address, if this is a non-negative numeric value.
    pub fn as_addr(&self) -> Option<u64> {
        self.as_uint()
    }

    /// The string, if this is one.
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Str(s) => Some(s.as_str()),
            _ => None,
        }
    }

    /// The duration, if this is one.
    pub fn as_duration(&self) -> Option<Duration> {
        match self {
            Value::Duration(d) => Some(*d),
            _ => None,
        }
    }

    /// The list elements, if this is a list.
    pub fn as_list(&self) -> Option<&[Value]> {
        match self {
            Value::List(items) => Some(items.as_slice()),
            _ => None,
        }
    }

    /// The nested map, if this is a map.
    pub fn as_map(&self) -> Option<&Props> {
        match self {
            Value::Map(m) => Some(m),
            _ => None,
        }
    }

    /// The link, if this is one.
    pub fn as_link(&self) -> Option<&Link> {
        match self {
            Value::Link(l) => Some(l),
            _ => None,
        }
    }

    /// The bound media, if this is some.
    pub fn as_media(&self) -> Option<&Media> {
        match self {
            Value::Media(m) => Some(m),
            _ => None,
        }
    }

    /// Extracts a boolean, naming `prop` in the error.
    pub fn to_bool(&self, prop: &str) -> Result<bool> {
        match self {
            Value::Bool(b) => Ok(*b),
            // The overwhelmingly common way to get this wrong is quoting.
            Value::Str(s) if matches!(s.as_str(), "true" | "false") => Err(type_error_hint(
                prop,
                ValueKind::Bool,
                self,
                "booleans are written without quotes",
            )),
            _ => Err(type_error(prop, ValueKind::Bool, self)),
        }
    }

    /// Extracts a signed integer, naming `prop` in the error.
    pub fn to_int(&self, prop: &str) -> Result<i64> {
        match self.as_int() {
            Some(i) => Ok(i),
            None if self.is_numeric() => Err(prop_err(format!(
                "property `{prop}`: {self} does not fit in a signed 64-bit integer"
            ))),
            None => Err(type_error(prop, ValueKind::Int, self)),
        }
    }

    /// Extracts an unsigned integer, naming `prop` in the error.
    pub fn to_uint(&self, prop: &str) -> Result<u64> {
        self.to_unsigned(prop, ValueKind::Uint)
    }

    /// Extracts a byte count, naming `prop` in the error.
    pub fn to_size(&self, prop: &str) -> Result<u64> {
        self.to_unsigned(prop, ValueKind::Size)
    }

    /// Extracts an address, naming `prop` in the error.
    pub fn to_addr(&self, prop: &str) -> Result<u64> {
        self.to_unsigned(prop, ValueKind::Addr)
    }

    fn to_unsigned(&self, prop: &str, want: ValueKind) -> Result<u64> {
        match self.as_uint() {
            Some(u) => Ok(u),
            // Distinguish "wrong type" from "right type, impossible value":
            // `size = -1` deserves to be told it is negative, not that a
            // signed integer is not a size.
            None if self.is_numeric() => Err(prop_err(format!(
                "property `{prop}`: expected {want}, found the negative value {self}"
            ))),
            None => Err(type_error(prop, want, self)),
        }
    }

    /// Extracts a duration, naming `prop` in the error.
    pub fn to_duration(&self, prop: &str) -> Result<Duration> {
        match self {
            Value::Duration(d) => Ok(*d),
            // A bare number is not a duration in any unit we get to choose for
            // the user; say what to add.
            _ if self.is_numeric() => Err(type_error_hint(
                prop,
                ValueKind::Duration,
                self,
                "durations need a unit, as in `10ms`",
            )),
            _ => Err(type_error(prop, ValueKind::Duration, self)),
        }
    }

    /// Extracts a string, naming `prop` in the error.
    pub fn to_str(&self, prop: &str) -> Result<&str> {
        self.as_str()
            .ok_or_else(|| type_error(prop, ValueKind::Str, self))
    }

    /// Extracts a list, naming `prop` in the error.
    pub fn to_list(&self, prop: &str) -> Result<&[Value]> {
        self.as_list()
            .ok_or_else(|| type_error(prop, ValueKind::List, self))
    }

    /// Extracts a nested map, naming `prop` in the error.
    pub fn to_map(&self, prop: &str) -> Result<&Props> {
        self.as_map()
            .ok_or_else(|| type_error(prop, ValueKind::Map, self))
    }

    /// Extracts a link, naming `prop` in the error.
    pub fn to_link(&self, prop: &str) -> Result<&Link> {
        self.as_link()
            .ok_or_else(|| type_error(prop, ValueKind::Link, self))
    }

    /// Extracts bound media, naming `prop` in the error.
    ///
    /// A [`Value::Str`] here means the machine file named a media slot that
    /// nothing was bound to — realize substitutes bound slots before a device
    /// is constructed, so a surviving string is an unbound one.
    pub fn to_media(&self, prop: &str) -> Result<&Media> {
        match self {
            Value::Media(m) => Ok(m),
            Value::Str(slot) => Err(type_error_hint(
                prop,
                ValueKind::Media,
                self,
                &format!("nothing is bound to the media slot `{slot}`"),
            )),
            _ => Err(type_error(prop, ValueKind::Media, self)),
        }
    }

    /// Guesses the kind of a scalar written as text.
    ///
    /// For `rsemu run nes.machine -p ram=4M`, where there is no declaration to
    /// say what `ram` should be. The machine-description lexer (§5) knows the
    /// expected kind from the device class and should call the specific parser
    /// instead — this one has to guess, and where the syntaxes overlap it
    /// prefers a size (`1m` is one mebibyte, not one minute).
    ///
    /// Anything that is not a recognised scalar becomes a [`Value::Str`], so
    /// this never fails.
    pub fn parse_scalar(text: &str) -> Value {
        let s = text.trim();
        if let Ok(b) = parse_bool(s) {
            return Value::Bool(b);
        }
        if let Ok(u) = parse_uint(s) {
            return Value::Uint(u);
        }
        if let Ok(i) = parse_int(s) {
            return Value::Int(i);
        }
        if let Ok(n) = parse_size(s) {
            return Value::Size(n);
        }
        if let Ok(d) = parse_duration(s) {
            return Value::Duration(d);
        }
        Value::Str(s.to_owned())
    }
}

impl fmt::Display for Value {
    /// Prints a value the way a machine file would write it, because that is
    /// the form the reader of an error message has to go and edit.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::Int(i) => write!(f, "{i}"),
            Value::Uint(u) => write!(f, "{u}"),
            Value::Bool(b) => write!(f, "{b}"),
            Value::Str(s) => write!(f, "\"{s}\""),
            Value::Size(n) => f.write_str(&format_size(*n)),
            Value::Addr(a) => write!(f, "{a:#x}"),
            Value::Duration(d) => write!(f, "{d}"),
            Value::List(items) => {
                f.write_str("[")?;
                for (i, item) in items.iter().enumerate() {
                    if i != 0 {
                        f.write_str(", ")?;
                    }
                    write!(f, "{item}")?;
                }
                f.write_str("]")
            }
            Value::Map(m) => {
                f.write_str("{")?;
                for (i, (name, value)) in m.iter().enumerate() {
                    if i != 0 {
                        f.write_str(",")?;
                    }
                    write!(f, " {name} = {value}")?;
                }
                f.write_str(" }")
            }
            Value::Link(l) => write!(f, "{l}"),
            // Not the file's own spelling — there is none — so it says what is
            // actually there, which is what a reader needs to see.
            Value::Media(m) => write!(f, "media `{}` ({} bytes)", m.name(), m.len()),
        }
    }
}

/// Renders a byte count with the largest binary multiplier that divides it.
fn format_size(n: u64) -> String {
    const SUFFIXES: [&str; 6] = ["K", "M", "G", "T", "P", "E"];
    if n != 0 {
        // Walk down so the largest exact unit wins: 1048576 is `1M`, not `1024K`.
        for (i, suffix) in SUFFIXES.iter().enumerate().rev() {
            let scale = 1u64 << (10 * (i as u32 + 1));
            if n.is_multiple_of(scale) {
                return format!("{}{}", n / scale, suffix);
            }
        }
    }
    format!("{n}")
}

impl From<bool> for Value {
    fn from(b: bool) -> Value {
        Value::Bool(b)
    }
}

impl From<i64> for Value {
    fn from(i: i64) -> Value {
        Value::Int(i)
    }
}

impl From<u64> for Value {
    fn from(u: u64) -> Value {
        Value::Uint(u)
    }
}

impl From<u32> for Value {
    fn from(u: u32) -> Value {
        Value::Uint(u as u64)
    }
}

impl From<&str> for Value {
    fn from(s: &str) -> Value {
        Value::Str(s.to_owned())
    }
}

impl From<String> for Value {
    fn from(s: String) -> Value {
        Value::Str(s)
    }
}

impl From<Duration> for Value {
    fn from(d: Duration) -> Value {
        Value::Duration(d)
    }
}

impl From<Link> for Value {
    fn from(l: Link) -> Value {
        Value::Link(l)
    }
}

impl From<Media> for Value {
    fn from(m: Media) -> Value {
        Value::Media(m)
    }
}

impl From<Props> for Value {
    fn from(p: Props) -> Value {
        Value::Map(p)
    }
}

impl From<Vec<Value>> for Value {
    fn from(items: Vec<Value>) -> Value {
        Value::List(items)
    }
}

// ---------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------

fn prop_err(message: String) -> Error {
    Error::Property(message)
}

fn type_error(prop: &str, want: ValueKind, found: &Value) -> Error {
    prop_err(format!(
        "property `{prop}`: expected {want}, found {} {found}",
        found.kind()
    ))
}

fn type_error_hint(prop: &str, want: ValueKind, found: &Value, hint: &str) -> Error {
    prop_err(format!(
        "property `{prop}`: expected {want}, found {} {found} ({hint})",
        found.kind()
    ))
}

/// Prefixes a parse error with the property it came from.
///
/// The scalar parsers are also the DSL lexer's, where the caller supplies a
/// `file:line:col` instead, so they do not know a property name themselves.
fn at_prop(prop: &str, e: Error) -> Error {
    match e {
        Error::Property(message) => prop_err(format!("property `{prop}`: {message}")),
        other => other,
    }
}

// ---------------------------------------------------------------------------
// Scalar parsing
// ---------------------------------------------------------------------------

/// Splits a leading numeric literal from whatever follows it.
///
/// Returns `(number, rest)`. The number may carry a sign, a radix prefix and
/// `_` separators; `rest` is everything after, which for a size or duration is
/// its unit. A radix prefix is only recognised when a digit of that radix
/// follows, so `0B` is zero bytes rather than a truncated binary literal.
fn split_number(s: &str) -> (&str, &str) {
    let b = s.as_bytes();
    let mut i = 0;
    if i < b.len() && (b[i] == b'+' || b[i] == b'-') {
        i += 1;
    }
    let mut radix = 10;
    if i + 2 <= b.len() && b[i] == b'0' {
        let marker = match b[i + 1] {
            b'x' | b'X' => Some(16),
            b'b' | b'B' => Some(2),
            b'o' | b'O' => Some(8),
            _ => None,
        };
        if let Some(r) = marker
            && b.len() > i + 2
            && (b[i + 2] as char).is_digit(r)
        {
            radix = r;
            i += 2;
        }
    }
    while i < b.len() {
        let c = b[i] as char;
        if c == '_' || c.is_digit(radix) {
            i += 1;
        } else {
            break;
        }
    }
    s.split_at(i)
}

/// Parses the sign, radix prefix and digits of `num`, rejecting overflow.
///
/// `orig` and `what` only shape the message. Returns `(negative, magnitude)`
/// so the caller decides what a sign means for its own type.
fn digits_to_u64(orig: &str, num: &str, what: &str) -> Result<(bool, u64)> {
    let mut s = num;
    let mut negative = false;
    if let Some(rest) = s.strip_prefix('-') {
        negative = true;
        s = rest;
    } else if let Some(rest) = s.strip_prefix('+') {
        s = rest;
    }

    let mut radix: u32 = 10;
    for (prefix, r) in [
        ("0x", 16),
        ("0X", 16),
        ("0b", 2),
        ("0B", 2),
        ("0o", 8),
        ("0O", 8),
    ] {
        if let Some(rest) = s.strip_prefix(prefix)
            && rest.starts_with(|c: char| c.is_digit(r))
        {
            radix = r;
            s = rest;
            break;
        }
    }

    if s.is_empty() {
        return Err(prop_err(format!(
            "`{orig}` is not a valid {what}: no digits"
        )));
    }
    if s.starts_with('_') || s.ends_with('_') {
        return Err(prop_err(format!(
            "`{orig}` is not a valid {what}: `_` may only separate digits"
        )));
    }

    let mut value: u64 = 0;
    let mut digits = 0usize;
    for c in s.chars() {
        if c == '_' {
            continue;
        }
        let d = c.to_digit(radix).ok_or_else(|| {
            prop_err(format!(
                "`{orig}` is not a valid {what}: `{c}` is not a base-{radix} digit"
            ))
        })?;
        // Overflow is an error, never a wrap: a machine file asking for
        // 2^64 bytes of RAM has a bug the user needs told about.
        value = value
            .checked_mul(radix as u64)
            .and_then(|v| v.checked_add(d as u64))
            .ok_or_else(|| {
                prop_err(format!(
                    "`{orig}` is not a valid {what}: value overflows 64 bits"
                ))
            })?;
        digits += 1;
    }
    if digits == 0 {
        return Err(prop_err(format!(
            "`{orig}` is not a valid {what}: no digits"
        )));
    }
    Ok((negative, value))
}

fn parse_unsigned(text: &str, what: &str) -> Result<u64> {
    let s = text.trim();
    let (num, rest) = split_number(s);
    if !rest.is_empty() {
        return Err(prop_err(format!(
            "`{text}` is not a valid {what}: unexpected `{rest}`"
        )));
    }
    let (negative, value) = digits_to_u64(text, num, what)?;
    if negative && value != 0 {
        return Err(prop_err(format!(
            "`{text}` is not a valid {what}: it is negative"
        )));
    }
    Ok(value)
}

/// Parses an unsigned integer: decimal, `0x` hex, `0b` binary, `0o` octal,
/// with `_` separators anywhere between digits.
///
/// A leading zero is *not* octal — `0755` is seven hundred and fifty-five.
/// C's rule silently changes the value of an ordinary-looking number, which is
/// exactly the kind of surprise a config file must not contain. Overflow is
/// rejected rather than wrapped.
///
/// ```
/// use rsemu::core::props::parse_uint;
/// assert_eq!(parse_uint("0xdead_beef").unwrap(), 0xdead_beef);
/// assert_eq!(parse_uint("0b1010").unwrap(), 10);
/// assert_eq!(parse_uint("0o755").unwrap(), 0o755);
/// assert_eq!(parse_uint("0755").unwrap(), 755);
/// assert!(parse_uint("18446744073709551616").is_err());
/// ```
pub fn parse_uint(text: &str) -> Result<u64> {
    parse_unsigned(text, "unsigned integer")
}

/// Parses a guest address. Same syntax as [`parse_uint`]; hex is the usual
/// form and the error message says "address".
pub fn parse_addr(text: &str) -> Result<u64> {
    parse_unsigned(text, "address")
}

/// Parses a signed integer. Same syntax as [`parse_uint`] plus a leading `+`
/// or `-`; overflow in either direction is rejected.
pub fn parse_int(text: &str) -> Result<i64> {
    let what = "signed integer";
    let s = text.trim();
    let (num, rest) = split_number(s);
    if !rest.is_empty() {
        return Err(prop_err(format!(
            "`{text}` is not a valid {what}: unexpected `{rest}`"
        )));
    }
    let (negative, magnitude) = digits_to_u64(text, num, what)?;
    // Via i128 so that -9223372036854775808 is representable and
    // +9223372036854775808 is not, without any wrapping cast.
    let wide = if negative {
        -(magnitude as i128)
    } else {
        magnitude as i128
    };
    i64::try_from(wide).map_err(|_| {
        prop_err(format!(
            "`{text}` is not a valid {what}: value overflows 64 bits"
        ))
    })
}

/// Parses a boolean: `true`/`false`, `yes`/`no`, `on`/`off`, case-insensitively.
///
/// `1` and `0` are deliberately *not* booleans — they are integers, and a
/// property that accepts both would make `enabled = 2` meaningful by accident.
pub fn parse_bool(text: &str) -> Result<bool> {
    let s = text.trim();
    if s.eq_ignore_ascii_case("true")
        || s.eq_ignore_ascii_case("yes")
        || s.eq_ignore_ascii_case("on")
    {
        return Ok(true);
    }
    if s.eq_ignore_ascii_case("false")
        || s.eq_ignore_ascii_case("no")
        || s.eq_ignore_ascii_case("off")
    {
        return Ok(false);
    }
    Err(prop_err(format!(
        "`{text}` is not a valid boolean (expected true/false, yes/no or on/off)"
    )))
}

/// Parses a byte count with an optional multiplier suffix: `4K`, `512M`, `2G`,
/// `1KiB`, `0x1000`, `4096`.
///
/// **Every multiplier is binary**, `KB` included: `1KB` is 1024 bytes, not
/// 1000. Disk vendors won that argument in the marketplace and lost it in
/// hardware documentation — a memory map, a ROM bank and a page size are all
/// powers of two, and a `size = 512M` region that came out 12 MiB short would
/// be a very confusing bug. The `i` in `KiB` is accepted and means the same
/// thing, so a user who spells it out gets what they expect.
///
/// Suffixes are `K`, `M`, `G`, `T`, `P`, `E`, each optionally followed by `i`
/// and/or `B`, case-insensitively; a bare `B` or no suffix means bytes.
/// A product that does not fit in a `u64` is an error, never a wrap.
///
/// ```
/// use rsemu::core::props::parse_size;
/// assert_eq!(parse_size("2K").unwrap(), 2048);
/// assert_eq!(parse_size("1KiB").unwrap(), 1024);
/// assert_eq!(parse_size("512M").unwrap(), 512 * 1024 * 1024);
/// assert!(parse_size("16E").is_err()); // 16 EiB does not fit in 64 bits
/// ```
pub fn parse_size(text: &str) -> Result<u64> {
    let what = "size";
    let s = text.trim();
    let (num, suffix) = split_number(s);
    let (negative, value) = digits_to_u64(text, num, what)?;
    if negative && value != 0 {
        return Err(prop_err(format!(
            "`{text}` is not a valid {what}: a byte count cannot be negative"
        )));
    }

    let mult = size_multiplier(text, suffix)?;
    value.checked_mul(mult).ok_or_else(|| {
        prop_err(format!(
            "`{text}` is not a valid {what}: it overflows a 64-bit byte count"
        ))
    })
}

fn size_multiplier(orig: &str, suffix: &str) -> Result<u64> {
    if suffix.is_empty() {
        return Ok(1);
    }
    let bad = || {
        prop_err(format!(
            "`{orig}` is not a valid size: unknown suffix `{suffix}` \
             (expected K, M, G, T, P or E, optionally followed by i and/or B — all binary)"
        ))
    };
    let mut chars = suffix.chars();
    let head = chars.next().ok_or_else(bad)?.to_ascii_lowercase();
    let tail: String = chars.flat_map(|c| c.to_lowercase()).collect();
    if head == 'b' {
        // A bare `B` is bytes; `Bi` and `BB` are nonsense.
        return if tail.is_empty() { Ok(1) } else { Err(bad()) };
    }
    let exponent = match head {
        'k' => 1u32,
        'm' => 2,
        'g' => 3,
        't' => 4,
        'p' => 5,
        'e' => 6,
        _ => return Err(bad()),
    };
    match tail.as_str() {
        "" | "i" | "b" | "ib" => {}
        _ => return Err(bad()),
    }
    // 1024^6 = 2^60, so this never overflows the shift.
    Ok(1u64 << (10 * exponent))
}

/// Parses a duration: a number and a unit, optionally repeated (`1h30m`).
///
/// Units are `ps`, `ns`, `us` (or `µs`), `ms`, `s`, `m` (minutes) and `h`,
/// case-insensitively. The unit is mandatory: a bare number would mean
/// whatever this module happened to pick, which is how a 10-second timeout
/// becomes a 10-millisecond one.
///
/// ```
/// use rsemu::core::props::parse_duration;
/// assert_eq!(parse_duration("10ms").unwrap().as_nanos(), 10_000_000);
/// assert_eq!(parse_duration("1h30m").unwrap().as_secs(), 5400);
/// assert!(parse_duration("10").is_err()); // no unit
/// ```
pub fn parse_duration(text: &str) -> Result<Duration> {
    let what = "duration";
    let s = text.trim();
    if s.is_empty() {
        return Err(prop_err(format!("`{text}` is not a valid {what}: empty")));
    }
    if s.starts_with('-') {
        return Err(prop_err(format!(
            "`{text}` is not a valid {what}: it is negative"
        )));
    }

    let mut total: u64 = 0;
    let mut rest = s;
    while !rest.is_empty() {
        let (num, after) = split_number(rest);
        if num.is_empty() {
            return Err(prop_err(format!(
                "`{text}` is not a valid {what}: expected a number at `{rest}`"
            )));
        }
        let (_, value) = digits_to_u64(text, num, what)?;

        let unit_len = after
            .find(|c: char| !c.is_alphabetic())
            .unwrap_or(after.len());
        let (unit, tail) = after.split_at(unit_len);
        if unit.is_empty() {
            return Err(prop_err(format!(
                "`{text}` is not a valid {what}: `{num}` has no unit \
                 (expected ps, ns, us, ms, s, m or h)"
            )));
        }
        let scale = duration_scale(unit).ok_or_else(|| {
            prop_err(format!(
                "`{text}` is not a valid {what}: unknown unit `{unit}` \
                 (expected ps, ns, us, ms, s, m or h)"
            ))
        })?;

        let overflow = || {
            prop_err(format!(
                "`{text}` is not a valid {what}: it overflows a 64-bit picosecond count"
            ))
        };
        total = value
            .checked_mul(scale)
            .and_then(|part| total.checked_add(part))
            .ok_or_else(overflow)?;
        rest = tail;
    }
    Ok(Duration::from_picos(total))
}

fn duration_scale(unit: &str) -> Option<u64> {
    // `µs` may arrive as U+00B5 (MICRO SIGN) or U+03BC (GREEK SMALL LETTER MU);
    // both are what a user's keyboard produced and neither is a typo.
    let lower: String = unit.chars().flat_map(|c| c.to_lowercase()).collect();
    Some(match lower.as_str() {
        "ps" => 1,
        "ns" => 1_000,
        "us" | "\u{b5}s" | "\u{3bc}s" => 1_000_000,
        "ms" => 1_000_000_000,
        "s" | "sec" => 1_000_000_000_000,
        "m" | "min" => 60_000_000_000_000,
        "h" | "hr" => 3_600_000_000_000_000,
        _ => return None,
    })
}

// ---------------------------------------------------------------------------
// FromValue
// ---------------------------------------------------------------------------

/// Extraction of a Rust type from a [`Value`], with the property name in hand.
///
/// Implemented for the obvious scalars so [`Reader::require`] and friends stay
/// one method each instead of one per type. Devices may implement it for their
/// own configuration types — an enum parsed from a string, say — and inherit
/// the whole reader API, including default and unknown-property handling.
pub trait FromValue: Sized {
    /// What to say this type is when extraction fails.
    const EXPECTED: &'static str;

    /// Extracts `Self`, or explains what went wrong with `prop`.
    fn from_value(prop: &str, value: &Value) -> Result<Self>;
}

impl FromValue for bool {
    const EXPECTED: &'static str = "boolean";
    fn from_value(prop: &str, value: &Value) -> Result<Self> {
        value.to_bool(prop)
    }
}

impl FromValue for u64 {
    const EXPECTED: &'static str = "unsigned integer";
    fn from_value(prop: &str, value: &Value) -> Result<Self> {
        value.to_uint(prop)
    }
}

impl FromValue for i64 {
    const EXPECTED: &'static str = "signed integer";
    fn from_value(prop: &str, value: &Value) -> Result<Self> {
        value.to_int(prop)
    }
}

/// Narrower integers extract through `u64`/`i64` and then range-check, so
/// `width = 70000` on a `u16` property is told it does not fit rather than
/// being truncated to 4464.
macro_rules! impl_from_value_narrow {
    ($($t:ty => $via:ty, $name:literal;)*) => {$(
        impl FromValue for $t {
            const EXPECTED: &'static str = $name;
            fn from_value(prop: &str, value: &Value) -> Result<Self> {
                let wide = <$via as FromValue>::from_value(prop, value)?;
                <$t>::try_from(wide).map_err(|_| {
                    prop_err(format!(
                        "property `{prop}`: {wide} does not fit in a {}",
                        $name
                    ))
                })
            }
        }
    )*};
}

impl_from_value_narrow! {
    u8  => u64, "8-bit unsigned integer";
    u16 => u64, "16-bit unsigned integer";
    u32 => u64, "32-bit unsigned integer";
    i8  => i64, "8-bit signed integer";
    i16 => i64, "16-bit signed integer";
    i32 => i64, "32-bit signed integer";
}

impl FromValue for String {
    const EXPECTED: &'static str = "string";
    fn from_value(prop: &str, value: &Value) -> Result<Self> {
        value.to_str(prop).map(ToOwned::to_owned)
    }
}

impl FromValue for Duration {
    const EXPECTED: &'static str = "duration";
    fn from_value(prop: &str, value: &Value) -> Result<Self> {
        value.to_duration(prop)
    }
}

impl FromValue for Link {
    const EXPECTED: &'static str = "link";
    fn from_value(prop: &str, value: &Value) -> Result<Self> {
        value.to_link(prop).cloned()
    }
}

impl FromValue for Media {
    const EXPECTED: &'static str = "media";
    fn from_value(prop: &str, value: &Value) -> Result<Self> {
        value.to_media(prop).cloned()
    }
}

impl FromValue for Props {
    const EXPECTED: &'static str = "map";
    fn from_value(prop: &str, value: &Value) -> Result<Self> {
        value.to_map(prop).cloned()
    }
}

impl FromValue for Vec<Value> {
    const EXPECTED: &'static str = "list";
    fn from_value(prop: &str, value: &Value) -> Result<Self> {
        value.to_list(prop).map(<[Value]>::to_vec)
    }
}

impl FromValue for Value {
    const EXPECTED: &'static str = "value";
    fn from_value(_prop: &str, value: &Value) -> Result<Self> {
        Ok(value.clone())
    }
}

// ---------------------------------------------------------------------------
// Props
// ---------------------------------------------------------------------------

/// A device's properties: names to [`Value`]s, in the order they were set.
///
/// Backed by a `Vec` of pairs rather than a `HashMap`, for two reasons.
/// Determinism: `CLAUDE.md` forbids hash iteration order from reaching
/// anything guest-visible, and this feeds device construction, `rsemu
/// describe`, and the JSON projection of a machine file. And *insertion* order
/// specifically, rather than a `BTreeMap`'s sorted order, because a machine
/// file that round-trips through `rsemu convert` should come back in the order
/// its author wrote — that is what makes a diff reviewable.
///
/// Lookup is linear. A device has a handful of properties and they are read
/// once at construction, so a hash would cost more than it saved.
///
/// # The host-object table rides along
///
/// A property is data, and a device also needs things that are not data: the
/// character port behind `port = "console"`, the pad behind
/// `pads = "player1"`. Those used to come from a process-wide `static`, which
/// meant two machines built in one process shared them. They now come from the
/// build's own [`HostObjects`], carried here because `Props` is what a
/// constructor is given — see [`Props::host`] and
/// [`core::hosts`](crate::core::hosts).
///
/// It is deliberately **not** part of equality or of the insertion order: two
/// property sets with the same properties are the same properties, whichever
/// build they were read for. That also keeps `Value::Map` comparable.
#[derive(Debug, Clone, Default)]
pub struct Props {
    entries: Vec<(String, Value)>,
    /// The build's host objects, when these properties are being read during a
    /// build. `None` for a `Props` a test or an embedder assembled by hand.
    hosts: Option<Arc<HostObjects>>,
}

impl PartialEq for Props {
    fn eq(&self, other: &Props) -> bool {
        self.entries == other.entries
    }
}

impl Eq for Props {}

impl Props {
    /// An empty set.
    pub fn new() -> Props {
        Props {
            entries: Vec::new(),
            hosts: None,
        }
    }

    /// Sets a property, replacing any previous value **in place**.
    ///
    /// Keeping the original position matters: a `param` overridden from the
    /// command line should not jump to the end of the file's ordering.
    pub fn insert(&mut self, name: impl Into<String>, value: impl Into<Value>) {
        let name = name.into();
        let value = value.into();
        for entry in &mut self.entries {
            if entry.0 == name {
                entry.1 = value;
                return;
            }
        }
        self.entries.push((name, value));
    }

    /// Builder form of [`Props::insert`], for tests and machine construction.
    #[must_use]
    pub fn with(mut self, name: impl Into<String>, value: impl Into<Value>) -> Props {
        self.insert(name, value);
        self
    }

    /// Removes a property, returning its value.
    pub fn remove(&mut self, name: &str) -> Option<Value> {
        let index = self.entries.iter().position(|(n, _)| n == name)?;
        Some(self.entries.remove(index).1)
    }

    /// The value of `name`, if set.
    pub fn get(&self, name: &str) -> Option<&Value> {
        self.entries.iter().find(|(n, _)| n == name).map(|(_, v)| v)
    }

    /// The value of `name`, or an error saying it is required.
    pub fn require(&self, name: &str) -> Result<&Value> {
        self.get(name)
            .ok_or_else(|| prop_err(format!("missing required property `{name}`")))
    }

    /// Whether `name` is set.
    pub fn contains(&self, name: &str) -> bool {
        self.get(name).is_some()
    }

    /// How many properties are set.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Whether no properties are set.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The properties, in insertion order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, &Value)> {
        self.entries.iter().map(|(n, v)| (n.as_str(), v))
    }

    /// The property names, in insertion order.
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.entries.iter().map(|(n, _)| n.as_str())
    }

    /// Fails if any property is not in `allowed`.
    ///
    /// The validate stage (`ROADMAP.md` §5) calls this with the device class's
    /// declared property list. A property nobody accepts is nearly always a
    /// typo, and silently ignoring it is how a user spends an afternoon
    /// wondering why `clok = master / 12` did nothing.
    pub fn check_known(&self, allowed: &[&str]) -> Result<()> {
        let unknown: Vec<&str> = self.names().filter(|n| !allowed.contains(n)).collect();
        unknown_error(&unknown, allowed)
    }

    /// A reader over these properties.
    pub fn reader(&self) -> Reader<'_> {
        Reader::new(self)
    }

    // -- host objects ------------------------------------------------------

    /// The build's host-object table, if these properties came from one.
    pub fn hosts(&self) -> Option<&Arc<HostObjects>> {
        self.hosts.as_ref()
    }

    /// Read these properties against `hosts`.
    ///
    /// The realizer calls this once per device, just before construction. A
    /// caller assembling `Props` by hand calls it to put a device in touch with
    /// a table it already holds.
    pub fn set_hosts(&mut self, hosts: Arc<HostObjects>) {
        self.hosts = Some(hosts);
    }

    /// Builder form of [`Props::set_hosts`].
    #[must_use]
    pub fn with_hosts(mut self, hosts: Arc<HostObjects>) -> Props {
        self.set_hosts(hosts);
        self
    }

    /// The host object called `name` under `kind`, creating it on first
    /// mention.
    ///
    /// This is how `port = "console"` becomes an `Arc<CharPort>` the host can
    /// also reach. Call it from `new(props)`: acquiring a host object is
    /// allocation, not an outward action — [`core::hosts`](crate::core::hosts)
    /// argues the case.
    ///
    /// **A `Props` with no table gets a private object**, freshly made and
    /// reachable by nobody else. That is the honest answer for a device a unit
    /// test built directly: there is no build, so there is nothing to rendezvous
    /// with. A test that wants both ends holds a [`HostObjects`] and passes it
    /// in with [`Props::with_hosts`], or builds the device against a handle
    /// directly.
    ///
    /// # Errors
    ///
    /// [`Error::Config`] if another type is already open under that kind and
    /// name.
    pub fn host<T, F>(&self, kind: HostKind, name: &str, make: F) -> Result<Arc<T>>
    where
        T: core::any::Any + Send + Sync,
        F: FnOnce() -> T,
    {
        match &self.hosts {
            Some(hosts) => hosts.open(kind, name, make),
            None => Ok(Arc::new(make())),
        }
    }
}

impl FromIterator<(String, Value)> for Props {
    fn from_iter<I: IntoIterator<Item = (String, Value)>>(iter: I) -> Props {
        let mut props = Props::new();
        for (name, value) in iter {
            props.insert(name, value);
        }
        props
    }
}

impl Extend<(String, Value)> for Props {
    fn extend<I: IntoIterator<Item = (String, Value)>>(&mut self, iter: I) {
        for (name, value) in iter {
            self.insert(name, value);
        }
    }
}

impl IntoIterator for Props {
    type Item = (String, Value);
    type IntoIter = alloc::vec::IntoIter<(String, Value)>;
    fn into_iter(self) -> Self::IntoIter {
        self.entries.into_iter()
    }
}

impl<'a> IntoIterator for &'a Props {
    type Item = (&'a str, &'a Value);
    type IntoIter = core::iter::Map<
        core::slice::Iter<'a, (String, Value)>,
        fn(&'a (String, Value)) -> (&'a str, &'a Value),
    >;
    fn into_iter(self) -> Self::IntoIter {
        fn split(entry: &(String, Value)) -> (&str, &Value) {
            (entry.0.as_str(), &entry.1)
        }
        self.entries.iter().map(split as fn(_) -> _)
    }
}

// ---------------------------------------------------------------------------
// Reader
// ---------------------------------------------------------------------------

/// A cursor over a [`Props`] that remembers what was asked for.
///
/// Reading through a `Reader` and finishing with [`Reader::finish`] is how a
/// device gets unknown-property detection for free: whatever nobody asked for
/// is either a typo or a property this build does not support, and both are
/// errors worth raising at construction time rather than at 3 a.m.
///
/// ```
/// use rsemu::core::props::{Props, Reader, Value};
///
/// let props = Props::new().with("siez", Value::Size(1024));
/// let mut r = Reader::new(&props);
/// let _ = r.or("size", 4096u64)?;
/// let err = r.finish().unwrap_err().to_string();
/// assert!(err.contains("did you mean `size`?"), "{err}");
/// # Ok::<(), rsemu::Error>(())
/// ```
#[derive(Debug)]
pub struct Reader<'a> {
    props: &'a Props,
    /// Parallel to `props.entries`: which ones something asked for.
    seen: Vec<bool>,
    /// Every name asked for, known or not — the candidate set for "did you
    /// mean", which is more useful than the set that happened to be present.
    asked: Vec<String>,
}

impl<'a> Reader<'a> {
    /// Starts reading `props`.
    pub fn new(props: &'a Props) -> Reader<'a> {
        Reader {
            props,
            seen: alloc::vec![false; props.len()],
            asked: Vec::new(),
        }
    }

    /// The properties being read.
    pub fn props(&self) -> &'a Props {
        self.props
    }

    /// Looks a property up, recording the interest either way.
    fn lookup(&mut self, name: &str) -> Option<&'a Value> {
        if !self.asked.iter().any(|n| n == name) {
            self.asked.push(name.to_owned());
        }
        let index = self.props.entries.iter().position(|(n, _)| n == name)?;
        if let Some(seen) = self.seen.get_mut(index) {
            *seen = true;
        }
        self.props.entries.get(index).map(|(_, v)| v)
    }

    /// Marks a property as read without extracting it.
    ///
    /// For the cases the reader cannot type-check itself — a `clock` expression
    /// the clock module resolves, say — so that [`Reader::finish`] does not
    /// then report it as unknown.
    pub fn touch(&mut self, name: &str) -> Option<&'a Value> {
        self.lookup(name)
    }

    /// Extracts a required property.
    pub fn require<T: FromValue>(&mut self, name: &str) -> Result<T> {
        match self.lookup(name) {
            Some(value) => T::from_value(name, value),
            None => Err(prop_err(format!(
                "missing required property `{name}` (expected {})",
                T::EXPECTED
            ))),
        }
    }

    /// Extracts a property if it is set.
    pub fn optional<T: FromValue>(&mut self, name: &str) -> Result<Option<T>> {
        match self.lookup(name) {
            Some(value) => T::from_value(name, value).map(Some),
            None => Ok(None),
        }
    }

    /// Extracts a property, falling back to `default` when it is not set.
    pub fn or<T: FromValue>(&mut self, name: &str, default: T) -> Result<T> {
        Ok(self.optional(name)?.unwrap_or(default))
    }

    /// Extracts a required property and checks it against an inclusive range.
    pub fn require_range<T>(&mut self, name: &str, range: RangeInclusive<T>) -> Result<T>
    where
        T: FromValue + PartialOrd + fmt::Display,
    {
        let value: T = self.require(name)?;
        check_range(name, value, range)
    }

    /// Extracts a property with a default, then checks it against a range.
    ///
    /// The default is range-checked too: a device whose own default is out of
    /// its own range is a bug, and this is where it surfaces.
    pub fn or_range<T>(&mut self, name: &str, default: T, range: RangeInclusive<T>) -> Result<T>
    where
        T: FromValue + PartialOrd + fmt::Display,
    {
        let value = self.or(name, default)?;
        check_range(name, value, range)
    }

    /// Extracts a required string, borrowed from the properties.
    pub fn require_str(&mut self, name: &str) -> Result<&'a str> {
        match self.lookup(name) {
            Some(value) => value.to_str(name),
            None => Err(prop_err(format!(
                "missing required property `{name}` (expected string)"
            ))),
        }
    }

    /// Extracts a string if it is set.
    pub fn optional_str(&mut self, name: &str) -> Result<Option<&'a str>> {
        match self.lookup(name) {
            Some(value) => value.to_str(name).map(Some),
            None => Ok(None),
        }
    }

    /// Extracts a string, falling back to `default`.
    pub fn or_str(&mut self, name: &str, default: &'a str) -> Result<&'a str> {
        Ok(self.optional_str(name)?.unwrap_or(default))
    }

    /// Extracts a required byte count (`512M`).
    pub fn require_size(&mut self, name: &str) -> Result<u64> {
        match self.lookup(name) {
            Some(value) => value.to_size(name),
            None => Err(prop_err(format!(
                "missing required property `{name}` (expected size)"
            ))),
        }
    }

    /// Extracts a byte count, falling back to `default`.
    pub fn or_size(&mut self, name: &str, default: u64) -> Result<u64> {
        match self.lookup(name) {
            Some(value) => value.to_size(name),
            None => Ok(default),
        }
    }

    /// Extracts a required guest address.
    pub fn require_addr(&mut self, name: &str) -> Result<u64> {
        match self.lookup(name) {
            Some(value) => value.to_addr(name),
            None => Err(prop_err(format!(
                "missing required property `{name}` (expected address)"
            ))),
        }
    }

    /// Extracts a guest address, falling back to `default`.
    pub fn or_addr(&mut self, name: &str, default: u64) -> Result<u64> {
        match self.lookup(name) {
            Some(value) => value.to_addr(name),
            None => Ok(default),
        }
    }

    /// Extracts a required list.
    pub fn require_list(&mut self, name: &str) -> Result<&'a [Value]> {
        match self.lookup(name) {
            Some(value) => value.to_list(name),
            None => Err(prop_err(format!(
                "missing required property `{name}` (expected list)"
            ))),
        }
    }

    /// Extracts a list if it is set.
    pub fn optional_list(&mut self, name: &str) -> Result<Option<&'a [Value]>> {
        match self.lookup(name) {
            Some(value) => value.to_list(name).map(Some),
            None => Ok(None),
        }
    }

    /// Extracts a required nested map.
    pub fn require_map(&mut self, name: &str) -> Result<&'a Props> {
        match self.lookup(name) {
            Some(value) => value.to_map(name),
            None => Err(prop_err(format!(
                "missing required property `{name}` (expected map)"
            ))),
        }
    }

    /// Extracts a nested map if it is set.
    pub fn optional_map(&mut self, name: &str) -> Result<Option<&'a Props>> {
        match self.lookup(name) {
            Some(value) => value.to_map(name).map(Some),
            None => Ok(None),
        }
    }

    /// Extracts required media — a ROM image, a disk image, a firmware blob.
    pub fn require_media(&mut self, name: &str) -> Result<&'a Media> {
        match self.lookup(name) {
            Some(value) => value.to_media(name),
            None => Err(prop_err(format!(
                "missing required property `{name}` (expected media: name a slot and bind it, \
                 as in `{name} = \"cart\"` with `--{name} <file>`)"
            ))),
        }
    }

    /// Extracts media if it is set.
    pub fn optional_media(&mut self, name: &str) -> Result<Option<&'a Media>> {
        match self.lookup(name) {
            Some(value) => value.to_media(name).map(Some),
            None => Ok(None),
        }
    }

    /// Extracts a required link.
    pub fn require_link(&mut self, name: &str) -> Result<&'a Link> {
        match self.lookup(name) {
            Some(value) => value.to_link(name),
            None => Err(prop_err(format!(
                "missing required property `{name}` (expected link)"
            ))),
        }
    }

    /// Extracts a link if it is set.
    pub fn optional_link(&mut self, name: &str) -> Result<Option<&'a Link>> {
        match self.lookup(name) {
            Some(value) => value.to_link(name).map(Some),
            None => Ok(None),
        }
    }

    /// Extracts a required string and checks it is one of `allowed`.
    ///
    /// For the closed string sets the DSL is full of (`engine = "interp"`,
    /// `unassigned = open-bus`). The error lists the whole set and suggests the
    /// nearest match, because "invalid value" without the alternatives just
    /// sends the user to the source.
    pub fn require_enum(&mut self, name: &str, allowed: &[&str]) -> Result<&'a str> {
        let value = self.require_str(name)?;
        check_enum(name, value, allowed)
    }

    /// Extracts a string from a closed set, falling back to `default`.
    pub fn or_enum(&mut self, name: &str, default: &'a str, allowed: &[&str]) -> Result<&'a str> {
        let value = self.or_str(name, default)?;
        check_enum(name, value, allowed)
    }

    /// The properties nobody asked for, in insertion order.
    pub fn unused(&self) -> Vec<&'a str> {
        self.props
            .entries
            .iter()
            .enumerate()
            .filter(|(i, _)| !self.seen.get(*i).copied().unwrap_or(false))
            .map(|(_, (name, _))| name.as_str())
            .collect()
    }

    /// Fails if any property went unread.
    ///
    /// Call this at the end of `Device::new`. Anything left is a name this
    /// device does not know, which is a typo far more often than it is
    /// deliberate.
    pub fn finish(self) -> Result<()> {
        let unused = self.unused();
        let known: Vec<&str> = self.asked.iter().map(String::as_str).collect();
        unknown_error(&unused, &known)
    }
}

/// Builds the unknown-property error, or `Ok(())` when there is none.
fn unknown_error(unknown: &[&str], known: &[&str]) -> Result<()> {
    if unknown.is_empty() {
        return Ok(());
    }
    let mut message = String::new();
    message.push_str(if unknown.len() == 1 {
        "unknown property "
    } else {
        "unknown properties "
    });
    for (i, name) in unknown.iter().enumerate() {
        if i != 0 {
            message.push_str(", ");
        }
        message.push_str(&format!("`{name}`"));
        if let Some(suggestion) = suggest(name, known) {
            message.push_str(&format!(" (did you mean `{suggestion}`?)"));
        }
    }
    if known.is_empty() {
        message.push_str("; this object takes no properties");
    } else {
        message.push_str("; known properties: ");
        for (i, name) in known.iter().enumerate() {
            if i != 0 {
                message.push_str(", ");
            }
            message.push_str(&format!("`{name}`"));
        }
    }
    Err(prop_err(message))
}

/// Range check shared by [`Reader::require_range`] and [`Reader::or_range`].
pub fn check_range<T>(prop: &str, value: T, range: RangeInclusive<T>) -> Result<T>
where
    T: PartialOrd + fmt::Display,
{
    if range.contains(&value) {
        Ok(value)
    } else {
        Err(prop_err(format!(
            "property `{prop}`: {value} is out of range {}..={}",
            range.start(),
            range.end()
        )))
    }
}

/// Membership check shared by [`Reader::require_enum`] and [`Reader::or_enum`].
pub fn check_enum<'a>(prop: &str, value: &'a str, allowed: &[&str]) -> Result<&'a str> {
    if allowed.contains(&value) {
        return Ok(value);
    }
    let mut message = format!("property `{prop}`: expected one of ");
    for (i, name) in allowed.iter().enumerate() {
        if i != 0 {
            message.push_str(", ");
        }
        message.push_str(&format!("`{name}`"));
    }
    message.push_str(&format!("; found \"{value}\""));
    if let Some(suggestion) = suggest(value, allowed) {
        message.push_str(&format!(" (did you mean `{suggestion}`?)"));
    }
    Err(prop_err(message))
}

/// The nearest candidate to `name`, if one is near enough to be worth naming.
///
/// The threshold scales with length so that short names do not collide (`irq`
/// should not "mean" `iru`) while a longer one tolerates a transposition, which
/// costs two edits.
/// The candidate closest to `name`, if one is close enough to be worth naming.
///
/// Public because every layer that resolves a name by string needs it and they
/// must agree: an unknown *property* and an unknown *object link* should be
/// equally helpful, and two thresholds would make one of them quietly worse.
/// The distance is optimal string alignment, so a transposition costs one —
/// `siez` for `size` is a slip, and plain Levenshtein scores it 2, outside any
/// threshold short enough to stay quiet on three-letter names like `irq`.
///
/// Returns `None` rather than a poor guess: sending a reader after a name that
/// was never going to work is worse than saying nothing.
///
/// ```
/// # use rsemu::core::props::suggest;
/// assert_eq!(suggest("siez", &["size", "base"]), Some("size"));
/// assert_eq!(suggest("completely-different", &["size", "base"]), None);
/// ```
pub fn suggest<'a>(name: &str, candidates: &[&'a str]) -> Option<&'a str> {
    let limit = (name.chars().count() / 3).max(1);
    let mut best: Option<(usize, &str)> = None;
    for candidate in candidates {
        let distance = edit_distance(name, candidate);
        if distance <= limit && best.is_none_or(|(d, _)| distance < d) {
            best = Some((distance, candidate));
        }
    }
    best.map(|(_, candidate)| candidate)
}

/// Case-insensitive optimal string alignment distance.
///
/// Levenshtein plus adjacent transposition, because `siez` for `size` is the
/// single most common way to mistype a name and plain Levenshtein charges two
/// edits for it — which puts it outside any threshold short enough to be safe.
fn edit_distance(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.chars().flat_map(char::to_lowercase).collect();
    let b: Vec<char> = b.chars().flat_map(char::to_lowercase).collect();
    let (n, m) = (a.len(), b.len());
    if n == 0 {
        return m;
    }
    if m == 0 {
        return n;
    }
    // Flat (n+1) x (m+1) matrix; property names are a handful of characters,
    // so the allocation is cheaper than the index gymnastics of rolling rows
    // when a transposition needs the row before last.
    let width = m + 1;
    let mut d = alloc::vec![0usize; (n + 1) * width];
    for i in 0..=n {
        d[i * width] = i;
    }
    for (j, slot) in d.iter_mut().take(width).enumerate() {
        *slot = j;
    }
    for i in 1..=n {
        for j in 1..=m {
            let cost = usize::from(a[i - 1] != b[j - 1]);
            let mut best = (d[(i - 1) * width + j] + 1)
                .min(d[i * width + j - 1] + 1)
                .min(d[(i - 1) * width + j - 1] + cost);
            if i > 1 && j > 1 && a[i - 1] == b[j - 2] && a[i - 2] == b[j - 1] {
                best = best.min(d[(i - 2) * width + j - 2] + 1);
            }
            d[i * width + j] = best;
        }
    }
    d[n * width + m]
}

/// Parses a property value from text, naming `prop` on failure.
///
/// A thin bridge for the paths that hold text and a property name but no
/// parsed value yet — CLI overrides, environment variables, the JSON
/// projection.
pub fn parse_as(prop: &str, kind: ValueKind, text: &str) -> Result<Value> {
    let value = match kind {
        ValueKind::Int => parse_int(text).map(Value::Int),
        ValueKind::Uint => parse_uint(text).map(Value::Uint),
        ValueKind::Bool => parse_bool(text).map(Value::Bool),
        ValueKind::Str => Ok(Value::Str(text.to_owned())),
        ValueKind::Size => parse_size(text).map(Value::Size),
        ValueKind::Addr => parse_addr(text).map(Value::Addr),
        ValueKind::Duration => parse_duration(text).map(Value::Duration),
        ValueKind::Link => Link::new(text).map(Value::Link),
        ValueKind::List | ValueKind::Map => Err(prop_err(format!(
            "a {kind} cannot be written as a bare scalar"
        ))),
        // Text names a media *slot*, never its contents; whoever realizes the
        // machine binds the bytes. Producing a `Value::Str` here would be a
        // lie the device only discovers at construction.
        ValueKind::Media => Err(prop_err(String::from(
            "media is bound by name at realize time, not written in a file",
        ))),
    };
    value.map_err(|e| at_prop(prop, e))
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::ToString;
    use alloc::vec;

    // -- sizes --------------------------------------------------------------

    #[test]
    fn every_size_suffix_is_binary() {
        assert_eq!(parse_size("0").unwrap(), 0);
        assert_eq!(parse_size("4096").unwrap(), 4096);
        assert_eq!(parse_size("4K").unwrap(), 4 * 1024);
        assert_eq!(parse_size("2K").unwrap(), 2048);
        assert_eq!(parse_size("512M").unwrap(), 512 * 1024 * 1024);
        assert_eq!(parse_size("2G").unwrap(), 2 * 1024 * 1024 * 1024);
        assert_eq!(parse_size("1T").unwrap(), 1u64 << 40);
        assert_eq!(parse_size("1P").unwrap(), 1u64 << 50);
        assert_eq!(parse_size("1E").unwrap(), 1u64 << 60);
        assert_eq!(parse_size("8E").unwrap(), 8u64 << 60);
    }

    #[test]
    fn size_suffix_spellings_agree() {
        for spelling in ["1K", "1k", "1KiB", "1kib", "1KB", "1kb", "1Ki", "1ki"] {
            assert_eq!(parse_size(spelling).unwrap(), 1024, "{spelling}");
        }
        // A bare B is bytes, including on zero, which must not be mistaken for
        // a truncated `0b` binary literal.
        assert_eq!(parse_size("7B").unwrap(), 7);
        assert_eq!(parse_size("0B").unwrap(), 0);
        // Hex with a multiplier is legal; the number syntax is shared.
        assert_eq!(parse_size("0x10K").unwrap(), 16 * 1024);
        assert_eq!(parse_size("0x2000").unwrap(), 0x2000);
    }

    #[test]
    fn size_overflow_is_rejected_not_wrapped() {
        // 16 EiB is exactly 2^64.
        let e = parse_size("16E").unwrap_err().to_string();
        assert!(e.contains("16E"), "{e}");
        assert!(e.contains("overflow"), "{e}");
        assert!(parse_size("32E").is_err());
        assert!(parse_size("18446744073709551616").is_err());
        // Just inside the boundary still works.
        assert_eq!(parse_size("15E").unwrap(), 15u64 << 60);
    }

    #[test]
    fn a_bad_size_suffix_says_what_is_allowed() {
        let e = parse_size("4Q").unwrap_err().to_string();
        assert!(e.contains("`4Q`"), "{e}");
        assert!(e.contains("unknown suffix `Q`"), "{e}");
        assert!(e.contains("K, M, G, T, P or E"), "{e}");
        assert!(parse_size("1Kx").is_err());
        assert!(parse_size("1BB").is_err());
        assert!(parse_size("-4K").is_err());
        assert!(parse_size("K").is_err());
    }

    // -- numbers ------------------------------------------------------------

    #[test]
    fn number_radix_forms() {
        assert_eq!(parse_uint("0x1234").unwrap(), 0x1234);
        assert_eq!(parse_uint("0X1234").unwrap(), 0x1234);
        assert_eq!(parse_uint("0xdeadBEEF").unwrap(), 0xdead_beef);
        assert_eq!(parse_uint("0b1010_0101").unwrap(), 0xa5);
        assert_eq!(parse_uint("0o755").unwrap(), 0o755);
        assert_eq!(parse_uint("0O755").unwrap(), 0o755);
        assert_eq!(parse_uint("1_000_000").unwrap(), 1_000_000);
        assert_eq!(parse_uint("  42  ").unwrap(), 42);
        // A leading zero is decimal, never octal: C's rule silently changes
        // the value of an ordinary-looking literal.
        assert_eq!(parse_uint("0755").unwrap(), 755);
        assert_eq!(parse_uint("0").unwrap(), 0);
    }

    #[test]
    fn number_errors_are_specific() {
        let e = parse_uint("0xzz").unwrap_err().to_string();
        assert!(e.contains("0xzz"), "{e}");
        let e = parse_uint("_1").unwrap_err().to_string();
        assert!(e.contains("`_` may only separate digits"), "{e}");
        assert!(parse_uint("1_").is_err());
        assert!(parse_uint("12x").is_err());
        assert!(parse_uint("").is_err());
        assert!(parse_uint("-1").is_err());
        // Doubled separators are ugly but unambiguous, like Rust's own.
        assert_eq!(parse_uint("1__0").unwrap(), 10);
        let e = parse_uint("-1").unwrap_err().to_string();
        assert!(e.contains("negative"), "{e}");
    }

    #[test]
    fn integer_overflow_is_rejected_at_both_ends() {
        assert_eq!(parse_uint("18446744073709551615").unwrap(), u64::MAX);
        assert!(parse_uint("18446744073709551616").is_err());
        assert_eq!(parse_int("-9223372036854775808").unwrap(), i64::MIN);
        assert_eq!(parse_int("9223372036854775807").unwrap(), i64::MAX);
        assert!(parse_int("9223372036854775808").is_err());
        assert!(parse_int("-9223372036854775809").is_err());
        assert_eq!(parse_int("+7").unwrap(), 7);
        assert_eq!(parse_int("-0x10").unwrap(), -16);
    }

    #[test]
    fn addresses_use_the_number_syntax() {
        assert_eq!(parse_addr("0x1234").unwrap(), 0x1234);
        assert_eq!(parse_addr("0xffff_ffff").unwrap(), 0xffff_ffff);
        let e = parse_addr("nowhere").unwrap_err().to_string();
        assert!(e.contains("address"), "{e}");
    }

    // -- booleans and durations --------------------------------------------

    #[test]
    fn booleans_accept_the_usual_spellings_but_not_numbers() {
        for t in ["true", "TRUE", "yes", "on", " True "] {
            assert!(parse_bool(t).unwrap(), "{t}");
        }
        for f in ["false", "No", "OFF"] {
            assert!(!parse_bool(f).unwrap(), "{f}");
        }
        assert!(parse_bool("1").is_err());
        let e = parse_bool("maybe").unwrap_err().to_string();
        assert!(e.contains("true/false"), "{e}");
    }

    #[test]
    fn durations_parse_every_unit() {
        assert_eq!(parse_duration("1ps").unwrap().as_picos(), 1);
        assert_eq!(parse_duration("1ns").unwrap().as_picos(), 1_000);
        assert_eq!(parse_duration("1us").unwrap().as_nanos(), 1_000);
        assert_eq!(parse_duration("1\u{b5}s").unwrap().as_nanos(), 1_000);
        assert_eq!(parse_duration("10ms").unwrap().as_nanos(), 10_000_000);
        assert_eq!(parse_duration("2s").unwrap().as_millis(), 2_000);
        assert_eq!(parse_duration("3m").unwrap().as_secs(), 180);
        assert_eq!(parse_duration("1h").unwrap().as_secs(), 3600);
        assert_eq!(parse_duration("1h30m").unwrap().as_secs(), 5400);
        assert_eq!(parse_duration("1s500ms").unwrap().as_millis(), 1500);
    }

    #[test]
    fn a_duration_without_a_unit_is_an_error() {
        let e = parse_duration("10").unwrap_err().to_string();
        assert!(e.contains("no unit"), "{e}");
        assert!(e.contains("ms"), "{e}");
        let e = parse_duration("10fortnights").unwrap_err().to_string();
        assert!(e.contains("unknown unit"), "{e}");
        assert!(parse_duration("-1s").is_err());
        assert!(parse_duration("").is_err());
        // 213 days fits; a year does not.
        assert!(parse_duration("5000h").unwrap().as_secs() > 0);
        let e = parse_duration("9000h").unwrap_err().to_string();
        assert!(e.contains("overflow"), "{e}");
    }

    // -- extraction errors --------------------------------------------------

    #[test]
    fn wrong_type_names_the_property_the_expectation_and_the_find() {
        let v = Value::Str("big".into());
        let e = v.to_size("size").unwrap_err().to_string();
        assert!(e.contains("`size`"), "{e}");
        assert!(e.contains("expected size"), "{e}");
        assert!(e.contains("found string"), "{e}");
        assert!(e.contains("\"big\""), "{e}");

        let e = Value::List(vec![])
            .to_uint("count")
            .unwrap_err()
            .to_string();
        assert!(
            e.contains("`count`") && e.contains("expected unsigned integer"),
            "{e}"
        );
        assert!(e.contains("found list"), "{e}");

        let e = Value::Bool(true).to_link("space").unwrap_err().to_string();
        assert!(
            e.contains("expected link") && e.contains("found boolean"),
            "{e}"
        );
    }

    #[test]
    fn common_mistakes_get_a_hint() {
        let e = Value::Str("true".into())
            .to_bool("readonly")
            .unwrap_err()
            .to_string();
        assert!(e.contains("without quotes"), "{e}");

        let e = Value::Uint(10)
            .to_duration("timeout")
            .unwrap_err()
            .to_string();
        assert!(e.contains("need a unit"), "{e}");
        assert!(e.contains("10ms"), "{e}");

        let e = Value::Int(-1).to_size("size").unwrap_err().to_string();
        assert!(e.contains("negative"), "{e}");
    }

    #[test]
    fn numeric_kinds_coerce_but_strings_never_do() {
        assert_eq!(Value::Uint(5).to_size("s").unwrap(), 5);
        assert_eq!(Value::Size(5).to_uint("s").unwrap(), 5);
        assert_eq!(Value::Addr(5).to_size("s").unwrap(), 5);
        assert_eq!(Value::Int(5).to_uint("s").unwrap(), 5);
        assert_eq!(Value::Uint(5).to_int("s").unwrap(), 5);
        assert!(Value::Str("5".into()).to_uint("s").is_err());
        let e = Value::Uint(u64::MAX).to_int("s").unwrap_err().to_string();
        assert!(e.contains("does not fit"), "{e}");
    }

    #[test]
    fn narrow_integers_range_check_instead_of_truncating() {
        let props = Props::new().with("width", 70000u32);
        let mut r = props.reader();
        let e = r.require::<u16>("width").unwrap_err().to_string();
        assert!(e.contains("70000"), "{e}");
        assert!(e.contains("16-bit unsigned integer"), "{e}");
    }

    // -- Props --------------------------------------------------------------

    #[test]
    fn iteration_follows_insertion_order() {
        let mut props = Props::new();
        for name in ["zeta", "alpha", "middle", "beta"] {
            props.insert(name, 1u64);
        }
        assert_eq!(
            props.names().collect::<Vec<_>>(),
            ["zeta", "alpha", "middle", "beta"]
        );
        // Overwriting keeps the original position: a CLI override must not
        // reorder the file it overrides.
        props.insert("alpha", 2u64);
        assert_eq!(
            props.names().collect::<Vec<_>>(),
            ["zeta", "alpha", "middle", "beta"]
        );
        assert_eq!(props.get("alpha"), Some(&Value::Uint(2)));
        assert_eq!(props.len(), 4);

        props.remove("middle");
        assert_eq!(props.names().collect::<Vec<_>>(), ["zeta", "alpha", "beta"]);
    }

    #[test]
    fn iteration_order_is_stable_across_identical_builds() {
        let build = || {
            Props::new()
                .with("size", Value::Size(2048))
                .with("base", Value::Addr(0x8000))
                .with("name", "wram")
        };
        let a: Vec<_> = build().names().map(ToOwned::to_owned).collect();
        let b: Vec<_> = build().names().map(ToOwned::to_owned).collect();
        assert_eq!(a, b);
        assert_eq!(a, ["size", "base", "name"]);
    }

    #[test]
    fn missing_required_property_says_what_it_wanted() {
        let props = Props::new();
        let mut r = props.reader();
        let e = r.require::<u64>("clock").unwrap_err().to_string();
        assert!(e.contains("missing required property `clock`"), "{e}");
        assert!(e.contains("unsigned integer"), "{e}");
        assert!(Props::new().require("clock").is_err());
    }

    #[test]
    fn defaults_and_optionals() {
        let props = Props::new().with("size", Value::Size(4096));
        let mut r = props.reader();
        assert_eq!(r.require_size("size").unwrap(), 4096);
        assert_eq!(r.or_size("stride", 16).unwrap(), 16);
        assert!(!r.or("readonly", false).unwrap());
        assert_eq!(r.or_str("name", "unnamed").unwrap(), "unnamed");
        assert_eq!(r.optional::<u64>("irq").unwrap(), None);
        r.finish().unwrap();
    }

    // -- unknown properties -------------------------------------------------

    #[test]
    fn a_typo_is_an_error_with_a_suggestion() {
        let props = Props::new()
            .with("size", Value::Size(2048))
            .with("clok", Value::Uint(12));
        let mut r = props.reader();
        let _ = r.require_size("size").unwrap();
        let _ = r.or("clock", 0u64).unwrap();
        let e = r.finish().unwrap_err().to_string();
        assert!(e.contains("unknown property `clok`"), "{e}");
        assert!(e.contains("did you mean `clock`?"), "{e}");
        assert!(e.contains("known properties: `size`, `clock`"), "{e}");
    }

    #[test]
    fn several_unknowns_are_reported_together_in_order() {
        let props = Props::new()
            .with("zzz", 1u64)
            .with("size", Value::Size(1))
            .with("aaa", 1u64);
        let mut r = props.reader();
        let _ = r.require_size("size").unwrap();
        let e = r.finish().unwrap_err().to_string();
        assert!(e.starts_with("unknown properties `zzz`, `aaa`"), "{e}");
    }

    // -- media --------------------------------------------------------------

    #[test]
    fn media_is_cheap_to_clone_and_says_nothing_about_its_contents() {
        let image: &[u8] = &[0u8; 4096];
        let media = Media::new("cart", image);
        assert_eq!(media.name(), "cart");
        assert_eq!(media.len(), 4096);
        assert!(!media.is_empty());
        // A 4 KiB hex dump in an error message helps nobody, and a ROM is
        // measured in megabytes.
        let shown = format!("{media:?}");
        assert!(shown.contains("cart") && shown.contains("4096"), "{shown}");
        assert!(shown.len() < 60, "debug output is a summary: {shown}");
        // Display is the same promise, for the `{value}` in a property error.
        let shown = Value::Media(media.clone()).to_string();
        assert!(shown.contains("cart") && shown.contains("4096"), "{shown}");
    }

    #[test]
    fn an_unbound_media_slot_is_told_apart_from_a_type_error() {
        // A surviving string means realize found nothing bound to the slot,
        // which is a different problem from writing `rom = 4096`.
        let named = Props::new().with("rom", "cart");
        let e = named.reader().require_media("rom").unwrap_err().to_string();
        assert!(e.contains("cart"), "{e}");
        assert!(e.contains("nothing is bound"), "{e}");

        let wrong = Props::new().with("rom", 4096u64);
        let e = wrong.reader().require_media("rom").unwrap_err().to_string();
        assert!(e.contains("expected media"), "{e}");

        // And a missing one says how to supply it, because "missing required
        // property" alone does not tell you that a media slot is a thing.
        let empty = Props::new();
        let e = empty.reader().require_media("rom").unwrap_err().to_string();
        assert!(e.contains("--rom"), "{e}");
        assert!(empty.reader().optional_media("rom").unwrap().is_none());
    }

    #[test]
    fn media_cannot_be_written_as_text() {
        // `-p rom=smb.nes` must not silently produce a slot named `smb.nes`;
        // media is bound by the caller, never parsed out of a machine file.
        let e = parse_as("rom", ValueKind::Media, "smb.nes")
            .unwrap_err()
            .to_string();
        assert!(e.contains("bound by name"), "{e}");
        assert_eq!(ValueKind::Media.as_str(), "media");
    }

    #[test]
    fn check_known_is_the_validate_stage_form() {
        let props = Props::new().with("size", Value::Size(1)).with("siez", 1u64);
        let e = props
            .check_known(&["size", "base"])
            .unwrap_err()
            .to_string();
        assert!(
            e.contains("`siez`") && e.contains("did you mean `size`?"),
            "{e}"
        );
        assert!(props.check_known(&["size", "siez"]).is_ok());

        let e = Props::new()
            .with("anything", 1u64)
            .check_known(&[])
            .unwrap_err()
            .to_string();
        assert!(e.contains("takes no properties"), "{e}");
    }

    #[test]
    fn touch_suppresses_the_unknown_report() {
        let props = Props::new().with("clock", "master / 12");
        let mut r = props.reader();
        assert!(r.touch("clock").is_some());
        r.finish().unwrap();
    }

    #[test]
    fn suggestions_do_not_fire_on_unrelated_names() {
        assert_eq!(suggest("clok", &["clock", "size"]), Some("clock"));
        assert_eq!(suggest("engien", &["engine"]), Some("engine"));
        // A transposition costs one edit, not two.
        assert_eq!(suggest("siez", &["size", "base"]), Some("size"));
        assert_eq!(edit_distance("siez", "size"), 1);
        assert_eq!(suggest("frobnicate", &["clock", "size"]), None);
        // Three-letter names tolerate one edit, no more, so near-misses
        // between genuinely different short names stay silent.
        assert_eq!(suggest("irq", &["iru"]), Some("iru"));
        assert_eq!(suggest("irq", &["dma"]), None);
    }

    // -- ranges and enums ---------------------------------------------------

    #[test]
    fn range_checking_reports_the_bound_it_broke() {
        let props = Props::new().with("width", 65u64);
        let mut r = props.reader();
        let e = r
            .require_range::<u64>("width", 1..=64)
            .unwrap_err()
            .to_string();
        assert!(e.contains("`width`"), "{e}");
        assert!(e.contains("65 is out of range 1..=64"), "{e}");

        let props = Props::new().with("width", 16u64);
        let mut r = props.reader();
        assert_eq!(r.require_range::<u64>("width", 1..=64).unwrap(), 16);
        // The default is checked too, so a device cannot ship one out of range.
        let props = Props::new();
        let mut r = props.reader();
        assert_eq!(r.or_range::<u64>("width", 8, 1..=64).unwrap(), 8);
        assert!(r.or_range::<u64>("other", 99, 1..=64).is_err());
    }

    #[test]
    fn enum_checking_lists_the_alternatives() {
        let props = Props::new().with("engine", "intrep");
        let mut r = props.reader();
        let e = r
            .require_enum("engine", &["interp", "jit", "auto"])
            .unwrap_err()
            .to_string();
        assert!(e.contains("`engine`"), "{e}");
        assert!(e.contains("expected one of `interp`, `jit`, `auto`"), "{e}");
        assert!(e.contains("found \"intrep\""), "{e}");
        assert!(e.contains("did you mean `interp`?"), "{e}");

        let props = Props::new().with("engine", "jit");
        let mut r = props.reader();
        assert_eq!(r.require_enum("engine", &["interp", "jit"]).unwrap(), "jit");
        let props = Props::new();
        let mut r = props.reader();
        assert_eq!(
            r.or_enum("engine", "interp", &["interp", "jit"]).unwrap(),
            "interp"
        );
    }

    // -- links, lists, maps -------------------------------------------------

    #[test]
    fn links_validate_their_path_syntax() {
        let link = Link::new("ppu.regs").unwrap();
        assert_eq!(link.as_str(), "ppu.regs");
        assert_eq!(link.root(), "ppu");
        assert_eq!(link.segments().collect::<Vec<_>>(), ["ppu", "regs"]);
        assert!(Link::new("cpu0").is_ok());
        assert!(Link::new("a-b.c_d").is_ok());
        assert!(Link::new("").is_err());
        assert!(Link::new("a..b").is_err());
        let e = Link::new("a.b!").unwrap_err().to_string();
        assert!(e.contains("unexpected `!`"), "{e}");
    }

    #[test]
    fn lists_and_nested_maps_extract() {
        let inner = Props::new().with("size", Value::Size(4096));
        let props = Props::new()
            .with("irqs", vec![Value::Uint(3), Value::Uint(5)])
            .with("bar0", inner)
            .with("space", Link::new("cpubus").unwrap());
        let mut r = props.reader();
        assert_eq!(r.require_list("irqs").unwrap().len(), 2);
        assert_eq!(
            r.require_map("bar0").unwrap().require("size").unwrap(),
            &Value::Size(4096)
        );
        assert_eq!(r.require_link("space").unwrap().as_str(), "cpubus");
        r.finish().unwrap();
    }

    // -- display and scalar guessing ---------------------------------------

    #[test]
    fn values_print_the_way_a_machine_file_writes_them() {
        assert_eq!(Value::Size(512 * 1024 * 1024).to_string(), "512M");
        assert_eq!(Value::Size(1024).to_string(), "1K");
        assert_eq!(Value::Size(1536).to_string(), "1536");
        assert_eq!(Value::Size(0).to_string(), "0");
        assert_eq!(Value::Addr(0x2000).to_string(), "0x2000");
        assert_eq!(Value::Str("ntsc".into()).to_string(), "\"ntsc\"");
        assert_eq!(Value::Int(-4).to_string(), "-4");
        assert_eq!(
            Value::List(vec![Value::Uint(3), Value::Uint(5)]).to_string(),
            "[3, 5]"
        );
        assert_eq!(
            Value::Map(Props::new().with("size", Value::Size(2048))).to_string(),
            "{ size = 2K }"
        );
        assert_eq!(parse_duration("10ms").unwrap().to_string(), "10ms");
        assert_eq!(parse_duration("1h30m").unwrap().to_string(), "90m");
        assert_eq!(Duration::ZERO.to_string(), "0s");
    }

    #[test]
    fn scalar_guessing_covers_the_cli_override_case() {
        assert_eq!(Value::parse_scalar("4M"), Value::Size(4 * 1024 * 1024));
        assert_eq!(Value::parse_scalar("0x8000"), Value::Uint(0x8000));
        assert_eq!(Value::parse_scalar("42"), Value::Uint(42));
        assert_eq!(Value::parse_scalar("-42"), Value::Int(-42));
        assert_eq!(Value::parse_scalar("true"), Value::Bool(true));
        assert_eq!(
            Value::parse_scalar("10ms"),
            Value::Duration(Duration::from_millis(10).unwrap())
        );
        assert_eq!(Value::parse_scalar("ntsc"), Value::Str("ntsc".into()));
        // A guessed size still reads as a number wherever a number is wanted.
        assert_eq!(Value::parse_scalar("4M").to_uint("ram").unwrap(), 4194304);
    }

    #[test]
    fn parse_as_prefixes_the_property_name() {
        let e = parse_as("ram", ValueKind::Size, "4Q")
            .unwrap_err()
            .to_string();
        assert!(e.contains("property `ram`:"), "{e}");
        assert!(e.contains("unknown suffix `Q`"), "{e}");
        assert_eq!(
            parse_as("base", ValueKind::Addr, "0x8000").unwrap(),
            Value::Addr(0x8000)
        );
        assert!(parse_as("x", ValueKind::List, "1,2").is_err());
    }

    #[test]
    fn props_are_send_and_sync() {
        // Devices are `Send + Sync` from the first commit (`CLAUDE.md`), and
        // they hold their properties.
        fn assert_send_sync<T: Send + Sync>() {}
        assert_send_sync::<Props>();
        assert_send_sync::<Value>();
        assert_send_sync::<Duration>();
        assert_send_sync::<Link>();
    }
}
