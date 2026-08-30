#![no_main]
//! Snapshot differential: whatever the writer wrote, the reader must read back.
//!
//! `state_decoder` proves the reader survives garbage. That is a *safety*
//! property and it is blind to the failure that actually loses a user's save
//! state: a field written one way and read back another, which produces no
//! crash at all and a subtly wrong machine a million cycles later. Every
//! `save`/`load` pair in rsemu is required to round-trip to an identical state
//! hash (CLAUDE.md, "Devices"); this fuzzes the primitive layer that every one
//! of those pairs is built out of.
//!
//! Four properties:
//!
//! 1. **Structure survives.** Shape and chunks come back equal to what went in.
//! 2. **Values survive.** A fuzzer-chosen sequence of typed values — every
//!    width, signed and unsigned, bools, byte arrays, strings, sequence
//!    lengths — is written into a chunk and read back in the same order,
//!    compared value by value. A framing bug in any one encoder shows up as a
//!    mismatch, usually in the *next* value rather than the guilty one, which
//!    is why the whole sequence is compared rather than one value at a time.
//! 3. **Writing is deterministic.** The same `StateWriter` emitted twice
//!    produces identical bytes, whatever order chunks were added in — the
//!    property `ROADMAP.md` §0's hash-the-state method rests on.
//! 4. **The encoding is canonical.** Decoding and re-encoding is byte-identical.
//!
//! # Structured input without `arbitrary`
//!
//! The input is decoded by hand from the raw byte stream ([`Gen`]) rather than
//! through `arbitrary`'s derive. Two reasons: the fuzz crate is the one place
//! external dependencies are tolerated and it should still take as few as it
//! can, and a `Gen` that is defined here cannot change the meaning of every
//! committed corpus entry when a dependency is bumped.

use std::collections::BTreeMap;

use libfuzzer_sys::fuzz_target;
use rsemu::core::state::{MachineShape, Migrations, Sink, Source, StateReader, StateWriter};

/// Bounds on generated structure. Small on purpose: coverage comes from the
/// variety of *shapes*, not from their size, and a fuzz iteration that
/// allocates megabytes is a timeout waiting to happen.
const MAX_DEVICES: usize = 8;
const MAX_REGIONS: usize = 8;
const MAX_STRINGS: usize = 6;
const MAX_CHUNKS: usize = 8;
const MAX_VALUES: usize = 24;
const MAX_STRING_LEN: usize = 24;
const MAX_BLOB_LEN: usize = 64;

/// A tiny structured-input decoder over the fuzzer's byte stream.
///
/// Every accessor is total: running out of input yields zeros and empty
/// strings rather than failing, so a truncated corpus entry degrades into a
/// smaller test case instead of being discarded.
struct Gen<'a> {
    data: &'a [u8],
    pos: usize,
}

impl<'a> Gen<'a> {
    fn new(data: &'a [u8]) -> Self {
        Gen { data, pos: 0 }
    }

    fn is_empty(&self) -> bool {
        self.pos >= self.data.len()
    }

    fn u8(&mut self) -> u8 {
        let byte = self.data.get(self.pos).copied().unwrap_or(0);
        self.pos = self.pos.saturating_add(1);
        byte
    }

    fn u32(&mut self) -> u32 {
        let mut out = 0u32;
        for _ in 0..4 {
            out = (out << 8) | u32::from(self.u8());
        }
        out
    }

    fn u64(&mut self) -> u64 {
        (u64::from(self.u32()) << 32) | u64::from(self.u32())
    }

    fn u128(&mut self) -> u128 {
        (u128::from(self.u64()) << 64) | u128::from(self.u64())
    }

    /// A count in `0..=max`.
    fn count(&mut self, max: usize) -> usize {
        if max == 0 {
            0
        } else {
            usize::from(self.u8()) % (max + 1)
        }
    }

    /// Up to `max` raw bytes.
    fn blob(&mut self, max: usize) -> Vec<u8> {
        let want = self.count(max);
        // `u8` saturates `pos` past the end of the input rather than tracking
        // exhaustion, so clamp before slicing: past the end there is nothing
        // left to take and the answer is an empty blob.
        let start = self.pos.min(self.data.len());
        let len = want.min(self.data.len() - start);
        let end = start + len;
        let out = self.data[start..end].to_vec();
        self.pos = end;
        out
    }

    /// A string built from raw bytes.
    ///
    /// Lossy conversion, so the generated strings include multi-byte
    /// characters and replacement characters — the interesting cases for a
    /// length-prefixed UTF-8 encoding whose reader validates rather than
    /// replaces.
    fn string(&mut self, max: usize) -> String {
        let bytes = self.blob(max);
        String::from_utf8_lossy(&bytes).into_owned()
    }
}

/// One typed value, written with a [`Sink`] method and read back with the
/// matching [`Source`] one.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Value {
    U8(u8),
    U16(u16),
    U32(u32),
    U64(u64),
    U128(u128),
    I8(i8),
    I16(i16),
    I32(i32),
    I64(i64),
    I128(i128),
    Bool(bool),
    Bytes(Vec<u8>),
    Str(String),
    /// A sequence: a length written with `write_seq_len`, then that many `u8`
    /// elements. Written as a unit because `read_seq_len` deliberately refuses
    /// a count whose elements could not fit in the bytes remaining — the
    /// anti-out-of-memory rule — so a bare length with nothing after it is a
    /// snapshot the reader is *right* to reject and not a round-trip failure.
    Seq(Vec<u8>),
}

impl Value {
    fn generate(pick: &mut Gen<'_>) -> Value {
        match pick.u8() % 14 {
            0 => Value::U8(pick.u8()),
            1 => Value::U16(pick.u32() as u16),
            2 => Value::U32(pick.u32()),
            3 => Value::U64(pick.u64()),
            4 => Value::U128(pick.u128()),
            5 => Value::I8(pick.u8() as i8),
            6 => Value::I16(pick.u32() as i16),
            7 => Value::I32(pick.u32() as i32),
            8 => Value::I64(pick.u64() as i64),
            9 => Value::I128(pick.u128() as i128),
            10 => Value::Bool(pick.u8() % 2 == 1),
            11 => Value::Bytes(pick.blob(MAX_BLOB_LEN)),
            12 => Value::Str(pick.string(MAX_STRING_LEN)),
            _ => Value::Seq(pick.blob(8)),
        }
    }

    fn write<S: Sink + ?Sized>(&self, sink: &mut S) {
        let result = match self {
            Value::U8(v) => sink.write_u8(*v),
            Value::U16(v) => sink.write_u16(*v),
            Value::U32(v) => sink.write_u32(*v),
            Value::U64(v) => sink.write_u64(*v),
            Value::U128(v) => sink.write_u128(*v),
            Value::I8(v) => sink.write_i8(*v),
            Value::I16(v) => sink.write_i16(*v),
            Value::I32(v) => sink.write_i32(*v),
            Value::I64(v) => sink.write_i64(*v),
            Value::I128(v) => sink.write_i128(*v),
            Value::Bool(v) => sink.write_bool(*v),
            Value::Bytes(v) => sink.write_bytes(v),
            Value::Str(v) => sink.write_str(v),
            Value::Seq(items) => {
                sink.write_seq_len(items.len() as u64).and_then(|()| {
                    items
                        .iter()
                        .try_for_each(|item| sink.write_u8(*item))
                })
            }
        };
        result.expect("writing into a chunk buffer cannot fail");
    }

    /// Read the value back, returning what was found so the caller can compare
    /// whole sequences rather than assert per value.
    fn read<'a, S: Source<'a> + ?Sized>(&self, src: &mut S) -> Value {
        match self {
            Value::U8(_) => Value::U8(src.read_u8().expect("u8 round-trips")),
            Value::U16(_) => Value::U16(src.read_u16().expect("u16 round-trips")),
            Value::U32(_) => Value::U32(src.read_u32().expect("u32 round-trips")),
            Value::U64(_) => Value::U64(src.read_u64().expect("u64 round-trips")),
            Value::U128(_) => Value::U128(src.read_u128().expect("u128 round-trips")),
            Value::I8(_) => Value::I8(src.read_i8().expect("i8 round-trips")),
            Value::I16(_) => Value::I16(src.read_i16().expect("i16 round-trips")),
            Value::I32(_) => Value::I32(src.read_i32().expect("i32 round-trips")),
            Value::I64(_) => Value::I64(src.read_i64().expect("i64 round-trips")),
            Value::I128(_) => Value::I128(src.read_i128().expect("i128 round-trips")),
            Value::Bool(_) => Value::Bool(src.read_bool().expect("bool round-trips")),
            Value::Bytes(_) => Value::Bytes(src.read_bytes().expect("bytes round-trip").to_vec()),
            Value::Str(_) => Value::Str(src.read_string().expect("a string round-trips")),
            // One byte per element, which is what was written.
            Value::Seq(_) => {
                let n = src.read_seq_len(1).expect("a sequence length round-trips");
                let items = (0..n)
                    .map(|_| src.read_u8().expect("a sequence element round-trips"))
                    .collect();
                Value::Seq(items)
            }
        }
    }
}

/// What the writer was asked to store, so the reader can be held to it.
struct Expected {
    class: String,
    version: u32,
    payload: Vec<u8>,
    values: Vec<Value>,
}

fuzz_target!(|data: &[u8]| {
    if data.is_empty() {
        return;
    }
    let mut pick = Gen::new(data);

    // ---- the machine shape ------------------------------------------------
    let mut shape = MachineShape::new();
    for _ in 0..pick.count(MAX_DEVICES) {
        let path = pick.string(MAX_STRING_LEN);
        let class = pick.string(MAX_STRING_LEN);
        // A duplicate instance path is a deliberate error, not a shape to
        // build; the first one wins and the rest are dropped, exactly as a
        // machine builder would have to handle it.
        let _ = shape.add_device(&path, &class);
    }
    for _ in 0..pick.count(MAX_REGIONS) {
        let space = pick.string(MAX_STRING_LEN);
        let name = pick.string(MAX_STRING_LEN);
        let base = pick.u64();
        let size = pick.u64();
        shape.add_region(&space, &name, base, size);
    }
    for _ in 0..pick.count(MAX_STRINGS) {
        let feature = pick.string(MAX_STRING_LEN);
        shape.add_feature(&feature);
    }
    for _ in 0..pick.count(MAX_STRINGS) {
        let arch = pick.string(MAX_STRING_LEN);
        shape.add_arch(&arch);
    }

    // ---- the chunks -------------------------------------------------------
    let mut writer = StateWriter::new(shape.clone());
    let mut expected: BTreeMap<String, Expected> = BTreeMap::new();

    for _ in 0..pick.count(MAX_CHUNKS) {
        if pick.is_empty() {
            break;
        }
        let path = pick.string(MAX_STRING_LEN);
        let class = pick.string(MAX_STRING_LEN);
        let version = pick.u32();

        // An empty path and a duplicate path are both refused by the writer by
        // design; skip them rather than record an expectation the writer never
        // accepted.
        if path.is_empty() || expected.contains_key(&path) {
            continue;
        }

        let values: Vec<Value> = (0..pick.count(MAX_VALUES))
            .map(|_| Value::generate(&mut pick))
            .collect();

        let mut payload = Vec::new();
        for value in &values {
            value.write(&mut payload);
        }

        writer
            .raw_chunk(&path, &class, version, &payload)
            .expect("a fresh non-empty path must be accepted");
        expected.insert(
            path,
            Expected {
                class,
                version,
                payload,
                values,
            },
        );
    }

    assert_eq!(writer.len(), expected.len());
    assert_eq!(writer.is_empty(), expected.is_empty());
    assert_eq!(writer.shape(), &shape);

    // ---- write ------------------------------------------------------------
    let bytes = writer.to_vec().expect("writing into a Vec cannot fail");

    // Determinism: the same state emitted twice is the same bytes. `write_to`
    // takes `&self` precisely so this can be asked.
    let again = writer.to_vec().expect("writing into a Vec cannot fail");
    assert!(
        bytes == again,
        "the same snapshot emitted twice produced different bytes"
    );

    // ---- read back --------------------------------------------------------
    let reader = match StateReader::new(&bytes) {
        Ok(reader) => reader,
        Err(e) => panic!("a snapshot this build wrote failed to parse: {e}"),
    };

    assert_eq!(
        reader.shape(),
        &shape,
        "the machine shape did not survive the round trip"
    );
    reader
        .check_shape(&shape)
        .expect("a snapshot must match the shape it was written from");

    let listed: Vec<_> = reader.chunks().collect();
    assert_eq!(
        listed.len(),
        expected.len(),
        "chunk count changed across the round trip"
    );

    let migrations = Migrations::new();

    for (info, (path, want)) in listed.iter().zip(expected.iter()) {
        // Chunks come back in ascending path order regardless of the order they
        // were added in: `expected` is a BTreeMap, so zipping the two is itself
        // the ordering assertion.
        assert_eq!(info.path, path.as_str(), "chunks came back in the wrong order");
        assert_eq!(info.class, want.class, "chunk `{path}` changed class");
        assert_eq!(info.version, want.version, "chunk `{path}` changed version");
        assert_eq!(info.len, want.payload.len(), "chunk `{path}` changed size");

        let loaded = reader
            .load(path, &want.class, want.version, &migrations)
            .expect("a chunk must load at the version it was written at");
        assert!(
            loaded.data() == want.payload.as_slice(),
            "chunk `{path}` payload changed across the round trip"
        );

        // The values, one at a time and in order: this is the differential the
        // no-panic targets cannot perform.
        let mut chunk = loaded.reader();
        let got: Vec<Value> = want.values.iter().map(|v| v.read(&mut chunk)).collect();
        assert!(
            got == want.values,
            "chunk `{path}`: typed values changed across the round trip"
        );
        chunk
            .end()
            .expect("reading every value written must consume the whole chunk");
    }

    // ---- canonical form ---------------------------------------------------
    // Decoding and re-encoding must reproduce the bytes exactly, which is what
    // makes a state hash a stable identity rather than an artefact of how the
    // snapshot happened to be built.
    let mut rewriter = StateWriter::new(reader.shape().clone());
    for info in &listed {
        let (class, version, payload) = reader.load_raw(info.path).expect("a listed chunk loads");
        rewriter
            .raw_chunk(info.path, class, version, payload)
            .expect("re-adding a chunk the reader produced must be accepted");
    }
    let reencoded = rewriter.to_vec().expect("writing into a Vec cannot fail");
    assert!(
        reencoded == bytes,
        "decode then re-encode was not byte-identical"
    );
});
