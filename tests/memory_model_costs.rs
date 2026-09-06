//! The two numbers behind the memory-model boundaries `core::space::store` and
//! `core::sync` record: what single-copy atomicity would cost, and how long the
//! host keeps a store invisible.
//!
//! Both are `#[ignore]`d — they are measurements, not gates, and a gate that
//! depends on a host's store buffer is a flake. They live here rather than in a
//! scratch file because both are quoted in module documentation as decisions,
//! and a quoted number nobody can re-derive is a number that rots.
//!
//! Run with `cargo test --release --test memory_model_costs -- --ignored
//! --nocapture --test-threads=1`. Release matters: the first measurement is a
//! handful of instructions and a debug build measures the bounds checks. One
//! test thread matters too, because the second measurement is about what two
//! threads see of each other and a third one competing for the same cores
//! moves the answer.

#![cfg(feature = "std")]

use std::hint::black_box;
use std::sync::atomic::{AtomicU8, AtomicU64, AtomicUsize, Ordering};
use std::sync::{Arc, Barrier};
use std::time::Instant;

// ---------------------------------------------------------------------------
// What single-copy atomicity would cost the RAM store
// ---------------------------------------------------------------------------

/// Bytes in the model stores. Small enough to stay in L1, so this measures the
/// access shape rather than the memory system.
const LEN: usize = 1 << 16;
/// Accesses per timed run.
const REPS: usize = 4_000_000;

/// What `RamStore` does today: a byte loop over `Vec<AtomicU8>`.
///
/// Per-byte atomicity and no `unsafe` at all — and, because the bytes are
/// independent, no single-copy atomicity for anything wider than one of them.
struct Bytes(Vec<AtomicU8>);

impl Bytes {
    fn new() -> Bytes {
        Bytes((0..LEN).map(|_| AtomicU8::new(0)).collect())
    }

    #[inline]
    fn write(&self, off: usize, src: &[u8]) {
        for (i, b) in src.iter().enumerate() {
            self.0[off + i].store(*b, Ordering::Relaxed);
        }
    }

    #[inline]
    fn read(&self, off: usize, dst: &mut [u8]) {
        for (i, b) in dst.iter_mut().enumerate() {
            *b = self.0[off + i].load(Ordering::Relaxed);
        }
    }
}

/// The alternative that keeps every rule the crate has: `Vec<AtomicU64>`, so a
/// whole word is one atomic access, with sub-word writes done by
/// compare-exchange because there is no other way to change part of a word
/// without letting a wide reader see the halves separately.
///
/// This is the shape whose cost decided the question, and the compare-exchange
/// is where the cost is.
struct Words(Vec<AtomicU64>);

impl Words {
    fn new() -> Words {
        Words((0..LEN / 8).map(|_| AtomicU64::new(0)).collect())
    }

    #[inline]
    fn write(&self, off: usize, src: &[u8]) {
        let n = src.len();
        let (word, lo) = (off / 8, off % 8);
        assert!(lo + n <= 8, "the model only does within-word accesses");
        let mut value = 0u64;
        for (i, b) in src.iter().enumerate() {
            value |= u64::from(*b) << ((lo + i) * 8);
        }
        if n == 8 {
            self.0[word].store(value, Ordering::Relaxed);
            return;
        }
        let mask = ((1u64 << (n * 8)) - 1) << (lo * 8);
        let cell = &self.0[word];
        let mut cur = cell.load(Ordering::Relaxed);
        loop {
            match cell.compare_exchange_weak(
                cur,
                (cur & !mask) | value,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => return,
                Err(now) => cur = now,
            }
        }
    }

    #[inline]
    fn read(&self, off: usize, dst: &mut [u8]) {
        let (word, lo) = (off / 8, off % 8);
        assert!(lo + dst.len() <= 8);
        let value = self.0[word].load(Ordering::Relaxed);
        for (i, b) in dst.iter_mut().enumerate() {
            *b = (value >> ((lo + i) * 8)) as u8;
        }
    }
}

/// Best of five, after three warm-up runs. Best rather than mean because the
/// question is what the access costs, not what the machine's other tenants do.
fn timed(name: &str, f: impl Fn()) {
    for _ in 0..3 {
        f();
    }
    let mut best = f64::MAX;
    for _ in 0..5 {
        let start = Instant::now();
        f();
        best = best.min(start.elapsed().as_secs_f64() * 1e9 / REPS as f64);
    }
    println!("{name}: {best:.2} ns/op");
}

/// `space::store`'s "What removing it would cost, measured".
///
/// Recorded there, on the author's host: today 0.74/0.89/1.25/1.99 ns to store
/// 1/2/4/8 bytes, against 3.86/4.05/4.24/1.55 for the word-and-CAS shape. The
/// +3 ns on every sub-word store is what decided it, against a ~25 ns whole
/// store through `SpaceView::write_span`.
#[test]
#[ignore = "a measurement, not a gate"]
fn what_single_copy_atomicity_would_cost() {
    let bytes = Bytes::new();
    let words = Words::new();
    for width in [1usize, 2, 4, 8] {
        // The offset walks a cache line at a time and the width is opaque to
        // the optimiser, because in the real store both are dynamic.
        let offsets = |i: usize| ((i * 64) % (LEN - 8)) & !7;
        timed(&format!("AtomicU8  store {width}B"), || {
            let src = vec![0xa5u8; width];
            for i in 0..REPS {
                bytes.write(offsets(i), black_box(&src));
            }
        });
        timed(&format!("AtomicU64 store {width}B"), || {
            let src = vec![0xa5u8; width];
            for i in 0..REPS {
                words.write(offsets(i), black_box(&src));
            }
        });
        timed(&format!("AtomicU8  load  {width}B"), || {
            let mut dst = vec![0u8; width];
            for i in 0..REPS {
                bytes.read(offsets(i), black_box(&mut dst));
            }
            black_box(&dst);
        });
        timed(&format!("AtomicU64 load  {width}B"), || {
            let mut dst = vec![0u8; width];
            for i in 0..REPS {
                words.read(offsets(i), black_box(&mut dst));
            }
            black_box(&dst);
        });
    }
}

// ---------------------------------------------------------------------------
// How long the host keeps a store invisible
// ---------------------------------------------------------------------------

/// One store-buffer litmus round over the primitive `RamStore` is built from.
///
/// Two threads, each storing to its own byte and then loading the other's, with
/// `pad` units of work in between standing in for the emulator's own cost per
/// guest instruction. Both loads returning zero is the outcome a guest's
/// `MFENCE` — or `DMB`, or `FENCE` — exists to forbid, and which no core in the
/// tree currently emits anything to prevent.
fn store_buffer_rounds(pad: u64, rounds: usize) -> usize {
    let x = Arc::new(AtomicU8::new(0));
    let y = Arc::new(AtomicU8::new(0));
    let seen = [Arc::new(AtomicU8::new(9)), Arc::new(AtomicU8::new(9))];
    let both_zero = Arc::new(AtomicUsize::new(0));
    // Three: the two litmus threads and the referee that reads the result and
    // resets the round. Without a referee the two threads have to agree about
    // when a round ended, which needs the ordering the test is measuring.
    let gate = Arc::new(Barrier::new(3));

    std::thread::scope(|s| {
        for (who, slot) in seen.iter().enumerate() {
            let (x, y) = (Arc::clone(&x), Arc::clone(&y));
            let out = Arc::clone(slot);
            let gate = Arc::clone(&gate);
            s.spawn(move || {
                for _ in 0..rounds {
                    gate.wait();
                    let (mine, theirs) = if who == 0 { (&x, &y) } else { (&y, &x) };
                    mine.store(1, Ordering::Relaxed);
                    for i in 0..pad {
                        black_box(i);
                    }
                    out.store(theirs.load(Ordering::Relaxed), Ordering::Relaxed);
                    gate.wait();
                }
            });
        }
        for _ in 0..rounds {
            gate.wait();
            gate.wait();
            if seen.iter().all(|r| r.load(Ordering::Relaxed) == 0) {
                both_zero.fetch_add(1, Ordering::Relaxed);
            }
            x.store(0, Ordering::Relaxed);
            y.store(0, Ordering::Relaxed);
            for r in &seen {
                r.store(9, Ordering::Relaxed);
            }
        }
    });
    both_zero.load(Ordering::Relaxed)
}

/// `core::sync`'s "The ladder is about deadlock, not about the guest's memory
/// model".
///
/// Recorded there, on the author's x86-64 host: tens to hundreds of forbidden
/// outcomes in 200 000 rounds with little or nothing between the store and the
/// load, and none at all once about forty nanoseconds separate them. The count
/// itself is scheduler noise; where it reaches zero is the number that matters,
/// because that is the window a guest barrier would have to cover.
#[test]
#[ignore = "a measurement, not a gate"]
fn how_long_the_host_keeps_a_store_invisible() {
    const ROUNDS: usize = 200_000;
    for pad in [0u64, 1, 2, 4, 8, 16, 32, 64, 128] {
        let n = store_buffer_rounds(pad, ROUNDS);
        println!("pad={pad:>4}: forbidden outcome {n} / {ROUNDS}");
    }
}
