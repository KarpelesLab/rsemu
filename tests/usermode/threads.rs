//! The level-3 threading guest: an ordinary `std` Rust program with threads.
//!
//! `scripts/fetch-testdata.sh usermode-guests` builds this for every
//! architecture `src/usermode/proof.rs` can run, into the git-ignored corpus
//! directory. Nothing is downloaded and nothing is committed but this source
//! file — the corpus rule (CLAUDE.md, Testing) applies to a compiler's output
//! as much as to a downloaded ROM.
//!
//! Where `hello.rs` proves the *process* image — the loader, the auxiliary
//! vector, thread-local storage, the heap — this one proves the **scheduling
//! contract**, which is the third of `ROADMAP.md` phase 5b's three
//! deliverables and the one nothing had exercised. `usermode::ThreadSet` was
//! a tick-quantum round robin with a snapshot and no user; this is the user.
//!
//! Three things happen here and each is a different part of the seam:
//!
//! * **`thread::spawn` and `join`** are `clone(CLONE_VM|CLONE_THREAD|…)` plus
//!   a `futex` wait on the child's `clear_child_tid` word. Nothing else
//!   implements `join`: the exiting thread zeroes that word and wakes it, and
//!   a kernel that forgets leaves every joiner blocked for ever.
//! * **A contended atomic counter** is the one thing a round robin can get
//!   silently wrong. Every increment is `amoadd` on RISC-V and an
//!   `ldaxr`/`stlxr` pair on an AArch64 part without `FEAT_LSE`, and the total
//!   at the end is a number that is either right or is evidence.
//! * **A mutex and a condition variable** reach `FUTEX_WAIT` with several
//!   threads parked on one word and one `FUTEX_WAKE` releasing them, which is
//!   the path a spin loop never gets to.
//!
//! What it prints is evidence rather than decoration: the joined values say
//! every thread ran its closure and returned, and the counter says none of
//! their increments was lost.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;

/// Threads, and increments each. Small enough to run under an interpreter in
/// a test, large enough that a preemption lands inside the increment loop
/// rather than between two runs of it.
const THREADS: u32 = 4;
const EACH: u64 = 10_000;

fn main() {
    let counter = Arc::new(AtomicU64::new(0));
    let mut workers = Vec::new();
    for i in 0..THREADS {
        let counter = Arc::clone(&counter);
        workers.push(thread::spawn(move || {
            for _ in 0..EACH {
                counter.fetch_add(1, Ordering::SeqCst);
            }
            i
        }));
    }
    let mut joined: Vec<u32> = workers.into_iter().map(|w| w.join().unwrap()).collect();
    joined.sort_unstable();
    println!("joined {joined:?}");
    println!("counter = {}", counter.load(Ordering::SeqCst));

    // A rendezvous: three threads block on a condition variable and one
    // notify releases all of them. `Mutex` and `Condvar` are futexes all the
    // way down, and a thread that blocks with no deadline is only ever
    // runnable again because somebody woke it.
    let gate = Arc::new((Mutex::new(false), Condvar::new()));
    let mut sleepers = Vec::new();
    for _ in 0..3 {
        let gate = Arc::clone(&gate);
        sleepers.push(thread::spawn(move || {
            let (open, bell) = &*gate;
            let mut open = open.lock().unwrap();
            while !*open {
                open = bell.wait(open).unwrap();
            }
        }));
    }
    {
        let (open, bell) = &*gate;
        *open.lock().unwrap() = true;
        bell.notify_all();
    }
    for s in sleepers {
        s.join().unwrap();
    }
    println!("rendezvous ok");
}
