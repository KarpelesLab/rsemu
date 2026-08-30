#![no_main]
//! The snapshot reader is a parser on untrusted input; it must never panic.
//!
//! A save state is a file a user hands us — downloaded, emailed, or produced by
//! an older build — so `core::state` states the contract in its own words:
//!
//! > it never panics, never indexes without a bounds check, never trusts a
//! > length field it has not compared against the bytes actually remaining, and
//! > never allocates proportional to a claimed count.
//!
//! Three properties are checked here, in increasing strength:
//!
//! 1. **No panic.** `StateReader::new` over arbitrary bytes, then every
//!    accessor on whatever it produced.
//! 2. **Self-consistency.** A snapshot that parsed must agree with itself: its
//!    chunks are in ascending, unique path order (the reader promises to reject
//!    anything else), `find`/`load_raw`/`load` agree with `chunks()`, and the
//!    shape it carries matches itself with an empty diff.
//! 3. **Canonical form.** `core::state` claims a snapshot has *exactly one*
//!    valid encoding, so that "re-encode what you decoded" is byte-identical —
//!    which is what makes the project's hash-the-state regression method
//!    meaningful. That is asserted here against the fuzzer's own input, which
//!    is a much harsher test than a round-trip from a writer: the input is a
//!    byte string libFuzzer built, and if the reader accepts two spellings of
//!    one snapshot this finds the pair.
//!
//! The typed `ChunkReader` decoders (`read_str`, `read_bytes`, `read_seq_len`)
//! are driven over each chunk payload too, since a chunk's *contents* are
//! untrusted for exactly the same reason its framing is.

use libfuzzer_sys::fuzz_target;
use rsemu::core::state::{ChunkReader, Migrations, Source, StateReader, StateWriter};

/// Cap on the typed-decoder driver, so a chunk full of zero-length reads cannot
/// turn one iteration into a timeout. The phase gate counts a timeout as a
/// failure, so the harness must not manufacture them.
const MAX_STEPS: usize = 4096;

fuzz_target!(|data: &[u8]| {
    // Anything that fails to parse has already proved the only thing this
    // target can prove about it: it did not panic.
    let Ok(reader) = StateReader::new(data) else {
        return;
    };

    let _ = reader.codec();
    let _ = reader.integrity();
    let shape = reader.shape().clone();

    // A shape is its own twin: the diff against itself must be empty, and
    // `check_shape` must agree with `diff_shape` about that.
    let diff = reader.diff_shape(&shape);
    assert!(
        diff.is_empty(),
        "a snapshot's shape differs from itself: {diff}"
    );
    assert_eq!(diff.len(), 0, "an empty diff must have length zero");
    reader
        .check_shape(&shape)
        .expect("check_shape must accept the shape the snapshot carries");

    // Exercise the diff machinery's accessors and Display on the shape's own
    // contents; every one of these strings came from the input.
    for (path, class) in shape.devices() {
        assert_eq!(shape.device_class(path), Some(class.as_str()));
    }
    for region in shape.regions() {
        let _ = region.to_string();
    }

    let chunks: Vec<_> = reader.chunks().collect();

    for pair in chunks.windows(2) {
        assert!(
            pair[0].path < pair[1].path,
            "chunks must be unique and ascending, got `{}` then `{}`",
            pair[0].path,
            pair[1].path
        );
    }

    let migrations = Migrations::new();

    for info in &chunks {
        // `find` and `load_raw` must describe the same chunk `chunks()` did.
        let found = reader
            .find(info.path)
            .expect("a listed chunk must be findable by its path");
        assert_eq!(found, *info, "find disagrees with the chunk listing");

        let (class, version, payload) = reader
            .load_raw(info.path)
            .expect("a listed chunk must load raw");
        assert_eq!(class, info.class);
        assert_eq!(version, info.version);
        assert_eq!(payload.len(), info.len);

        // Loading at the stored version with no migrations registered is the
        // identity case and must succeed; anything else is a bug in the
        // migration chain rather than in the input.
        let loaded = reader
            .load(info.path, info.class, info.version, &migrations)
            .expect("loading a chunk at its own version needs no migration");
        assert_eq!(loaded.path(), info.path);
        assert_eq!(loaded.class(), info.class);
        assert_eq!(loaded.version(), info.version);
        assert_eq!(loaded.stored_version(), info.version);
        assert!(!loaded.migrated(), "no migration was registered");
        assert_eq!(loaded.data(), payload);

        // Asking for a class the chunk was not written by, or a version older
        // than the one on the wire, must be an error rather than a panic.
        let _ = reader.load(info.path, "no.such.class", info.version, &migrations);
        let _ = reader.load(
            info.path,
            info.class,
            info.version.wrapping_add(1),
            &migrations,
        );

        drive_typed_reader(payload);
    }

    // Paths that are not in the snapshot: the "not found" path builds an error
    // message out of the paths that *are* there, which is more string handling
    // over untrusted bytes.
    assert!(reader.find("/definitely/not/here").is_none());
    let _ = reader.load_raw("/definitely/not/here");

    // Canonical form. Re-encoding what was decoded must reproduce the input
    // byte for byte.
    //
    // One legitimate asymmetry, and the reason this is a conditional rather
    // than an assertion on every input: `StateWriter::chunk` refuses an empty
    // instance path, while the reader has no such rule, so a hand-built
    // snapshot with an empty path decodes but cannot be re-encoded. Skipping is
    // the honest thing to do — the alternative would report the writer's
    // validation as a round-trip bug.
    let mut writer = StateWriter::new(shape);
    for info in &chunks {
        let Ok((class, version, payload)) = reader.load_raw(info.path) else {
            return;
        };
        if writer.raw_chunk(info.path, class, version, payload).is_err() {
            return;
        }
    }
    let reencoded = writer.to_vec().expect("writing into a Vec cannot fail");
    assert!(
        reencoded == data,
        "a snapshot has exactly one valid encoding, but decoding {} bytes and \
         re-encoding produced {} different bytes",
        data.len(),
        reencoded.len()
    );
});

/// Drive the typed chunk decoders over a payload, choosing operations from the
/// payload itself.
///
/// A device's `load` is a sequence of `read_*` calls whose shape the device
/// chooses; no fuzzer can guess a particular device's sequence, so this drives
/// an input-chosen one instead. What is being tested is that every decoder
/// bounds-checks against the bytes remaining rather than trusting the length it
/// just read — `read_bytes`, `read_str` and `read_seq_len` each read a `u64`
/// out of the payload and would be the classic huge-allocation bug if they did
/// not.
fn drive_typed_reader(payload: &[u8]) {
    let mut reader = ChunkReader::new(payload);
    let mut steps = 0usize;

    while reader.remaining() > 0 && steps < MAX_STEPS {
        steps += 1;
        let Ok(op) = reader.read_u8() else { break };
        let before = reader.remaining();
        let ok = match op % 16 {
            0 => reader.read_u8().is_ok(),
            1 => reader.read_u16().is_ok(),
            2 => reader.read_u32().is_ok(),
            3 => reader.read_u64().is_ok(),
            4 => reader.read_u128().is_ok(),
            5 => reader.read_i8().is_ok(),
            6 => reader.read_i16().is_ok(),
            7 => reader.read_i32().is_ok(),
            8 => reader.read_i64().is_ok(),
            9 => reader.read_i128().is_ok(),
            10 => reader.read_bool().is_ok(),
            11 => reader.read_bytes().is_ok(),
            12 => reader.read_str().is_ok(),
            13 => reader.read_string().is_ok(),
            14 => reader.read_seq_len(1).is_ok(),
            _ => reader.read_seq_len(32).is_ok(),
        };
        assert!(
            reader.remaining() <= before,
            "a reader must never grow the bytes remaining"
        );
        if !ok {
            break;
        }
        let _ = reader.position();
    }

    // `end` is the "did the loader read every field the saver wrote" check. It
    // is allowed to fail; it is not allowed to panic.
    let _ = reader.end();
}
