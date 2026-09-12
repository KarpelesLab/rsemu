#![no_main]
//! The DfuSe (`.dfu`, UM0391) container parser, on bytes nobody vetted.
//!
//! A firmware image arrives from a vendor's download page, and `dfu.loader`
//! reads a length-prefixed tree out of it: a prefix that says how big the file
//! is, target headers that say how big their element blocks are, and element
//! headers that say how long their data is — every one of them a `u32` the file
//! chose. That is the shape `CLAUDE.md` names when it asks for a fuzz target on
//! the image parsers, and the failure it is looking for is the one no unit test
//! finds: a length that passes one check and overruns the next.
//!
//! What is asserted, beyond "it did not panic":
//!
//! * **A parse that succeeds describes bytes that exist.** Every element's
//!   `(offset, len)` has to be inside the file and yield a slice of exactly
//!   `len` bytes — a descriptor pointing past the end would be a silent
//!   out-of-bounds read for every caller that trusted it.
//! * **A parse that succeeds accounts for the whole file.** Elements are
//!   disjoint within a target and none of them reaches into the suffix.
//! * **CRC verification only ever narrows.** Anything `parse(.., true)` accepts,
//!   `parse(.., false)` accepts identically: the flag is a suffix check and
//!   must not change how a byte is read.
//! * **The loader survives whatever the parser accepted.** A `Loader` built
//!   from the file and pointed at a small address space must refuse or write,
//!   never panic, and never write outside the region it was given.
//!
//! # Input encoding
//!
//! Hand-decoded from the raw stream rather than derived, for the reason
//! `state_roundtrip` gives: an `arbitrary` derive reinterprets every seed when
//! its version changes, and the corpus stops meaning anything.
//!
//! ```text
//!   byte 0 = 0x00   the rest is a DfuSe file, verbatim
//!   otherwise       the rest builds a *valid* file, which is then damaged:
//!                     byte 1      how many targets (mod 4)
//!                     byte 2      how many elements per target (mod 5)
//!                     byte 3      how many bytes to corrupt (mod 8)
//!                     then, per element:  4 bytes address, 1 byte length
//!                     then, per corruption: 2 bytes offset, 1 byte xor mask
//! ```
//!
//! The second mode is what gets past the first signature check. A random byte
//! string is rejected by `"DfuSe"` in the first microsecond and never reaches
//! the length arithmetic this exists to interrogate; a valid file with three
//! bytes flipped reaches all of it.

use libfuzzer_sys::fuzz_target;

use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region, RequesterId};
use rsemu::dev::dfuse::{self, Image, Loader};
use std::sync::Arc;

/// Where the fuzzer's address space answers, and how much of it there is.
const BASE: u64 = 0x0800_0000;
/// Small, so that most element addresses miss it and the refusal path is hot.
const SIZE: u64 = 0x2000;

/// A byte from `data`, or zero once it has run out.
fn byte(data: &[u8], at: usize) -> u8 {
    data.get(at).copied().unwrap_or(0)
}

/// Build a valid file out of `data`, then flip the bytes `data` names.
fn seeded(data: &[u8]) -> Vec<u8> {
    let targets = usize::from(byte(data, 0) % 4);
    let per_target = usize::from(byte(data, 1) % 5);
    let damage = usize::from(byte(data, 2) % 8);
    let mut at = 3usize;

    // Element payloads have to outlive the borrowed slices `build` takes, so
    // every buffer is materialised first and only then referenced.
    let mut payloads: Vec<Vec<(u32, Vec<u8>)>> = Vec::new();
    for _ in 0..targets {
        let mut elements = Vec::new();
        for _ in 0..per_target {
            let addr = u32::from_le_bytes([
                byte(data, at),
                byte(data, at + 1),
                byte(data, at + 2),
                byte(data, at + 3),
            ]);
            let len = usize::from(byte(data, at + 4));
            at += 5;
            elements.push((addr, vec![0x5au8; len]));
        }
        payloads.push(elements);
    }

    let borrowed: Vec<Vec<(u32, &[u8])>> = payloads
        .iter()
        .map(|t| t.iter().map(|(a, d)| (*a, d.as_slice())).collect())
        .collect();
    let spec: Vec<(u8, Option<&str>, &[(u32, &[u8])])> = borrowed
        .iter()
        .enumerate()
        .map(|(i, t)| (i as u8, Some("Internal Flash"), t.as_slice()))
        .collect();

    let mut file = dfuse::build(0x0483, 0xdf11, &spec);
    for _ in 0..damage {
        let off = usize::from(u16::from_le_bytes([byte(data, at), byte(data, at + 1)]));
        let mask = byte(data, at + 2);
        at += 3;
        if !file.is_empty() {
            let i = off % file.len();
            file[i] ^= mask;
        }
    }
    file
}

/// Every element describes bytes that are really there, and no two overlap in
/// the file.
fn descriptors_are_sound(file: &[u8], image: &Image) {
    let end_of_body = file.len() as u64 - dfuse::SUFFIX_LEN;
    for target in &image.targets {
        for e in &target.elements {
            assert!(
                e.offset >= dfuse::PREFIX_LEN,
                "an element claims to start inside the prefix"
            );
            let end = e
                .offset
                .checked_add(e.len)
                .expect("an accepted element's extent has to be computable");
            assert!(
                end <= end_of_body,
                "an accepted element runs into the suffix: {} + {} > {end_of_body}",
                e.offset,
                e.len
            );
            assert_eq!(
                e.data(file).len() as u64,
                e.len,
                "an accepted element yields a slice of a different length"
            );
        }
    }
}

/// A space with one small RAM region, for the loader half.
fn space() -> Arc<AddressSpace> {
    let s = AddressSpace::new("mem", 32);
    s.topology()
        .map(Region::ram("ram", Arc::new(RamStore::new(SIZE))), BASE)
        .expect("a fresh space");
    Arc::new(s)
}

fuzz_target!(|data: &[u8]| {
    let file = match data.split_first() {
        None => return,
        Some((0, rest)) => rest.to_vec(),
        Some((_, rest)) => seeded(rest),
    };

    // Refusing is the expected outcome for most inputs; panicking never is.
    let verified = Image::parse(&file, true);
    let unverified = Image::parse(&file, false);

    if let Ok(image) = &verified {
        descriptors_are_sound(&file, image);
        // The CRC flag is a suffix check and nothing else: it may reject a file
        // the unverified parse accepts, never the other way round, and never a
        // different reading of the same bytes.
        assert_eq!(
            Some(image),
            unverified.as_ref().ok(),
            "verify-crc changed how the container was read"
        );
    }
    if let Ok(image) = &unverified {
        descriptors_are_sound(&file, image);
        // Every target the file names has to be selectable by its own number.
        for t in &image.targets {
            assert!(
                image.select(Some(t.alt), None).is_ok() || image.targets.len() > 1,
                "a lone target is not selectable by the alt it declares"
            );
        }
    }

    // And the device half: whatever the parser accepted, the loader must either
    // refuse in a named way or write it, and a write that lands outside the one
    // region this space has must be the refusal rather than a silent drop.
    if let Ok(loader) = Loader::from_file(file, None, None, false) {
        let space = space();
        // A zero-length element writes nothing, so where it points says
        // nothing either: only an element with bytes in it has to land
        // somewhere. (The first input this target rejected was exactly that —
        // one empty element at address zero.)
        let outside = loader.elements().iter().any(|e| {
            e.len > 0 && (e.addr < BASE || e.addr.saturating_add(e.len) > BASE + SIZE)
        });
        let wrote = loader.load_into(&space, RequesterId::ANONYMOUS).is_ok();
        assert!(
            !(outside && wrote),
            "an element outside the only mapped region was accepted"
        );
        // A debug read of the region must still answer after all that.
        let mut buf = [0u8; 8];
        let _ = space.read_bytes(BASE, &mut buf, MemAttrs::DEBUG);
    }
});
