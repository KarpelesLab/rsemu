//! Whether a guest barrier changes an observable outcome, and at which level
//! of the emulator it can be shown to.
//!
//! `cpu::arm::a64`, `cpu::riscv` and `cpu::x86` retired every data barrier as
//! a no-op until this file existed. The argument for changing that is in
//! `core::sync` and `docs/techniques/memory-models.md`: a guest instruction's
//! accesses do leave in program order — the emulator never reorders them — but
//! each becomes a **relaxed host atomic**, so the host's own model applies
//! underneath, and a barrier is the guest asking for something its own
//! baseline does not give it. The classic instrument is the **store-buffer**
//! litmus test (Sewell et al., *x86-TSO*): two threads, each storing to its own
//! location and then loading the other's, where *both loads returning zero* is
//! the outcome `MFENCE`, `DMB` and `FENCE` exist to forbid.
//!
//! # Which threading mode
//!
//! Everything concurrent here is the shape
//! [`ThreadingMode::Parallel`](rsemu::core::sched::ThreadingMode::Parallel)
//! gives a machine: one host thread per guest core, running at the same time.
//! It is opt-in (`--threading parallel`, or a `threading` statement in a
//! machine file) and no board in `machines/` selects it.
//! [`Deterministic`](rsemu::core::sched::ThreadingMode::Deterministic) — the
//! default, and the mode every golden state hash comes from — runs both cores
//! on one host thread, and [`sequential_interleaving_cannot_produce_it`] is
//! that mode's *worst case*: a switch after every single instruction. It cannot
//! produce the outcome and the reason is structural rather than statistical,
//! so that one is a gate. Under
//! [`Accel`](rsemu::core::sched::ThreadingMode::Accel) the question does not
//! arise: the host's own silicon performs the guest's accesses.
//!
//! # What was found, on an x86-64 host
//!
//! Three levels, the same round structure, 200 000 rounds each (a release
//! build; see [`ROUNDS`]):
//!
//! | what sits between the store and the load | forbidden outcome |
//! | --- | --- |
//! | nothing — two bare relaxed `AtomicU8`s | 2 to 751, run to run |
//! | `core::sync::fence(SeqCst)` — what `IrHost::fence` now emits | **0** |
//! | a relaxed `fetch_or` on an unrelated word | **0** |
//! | the same word, tested before it is set — `mark_dirty` today | 27 to 166 |
//! | `RamStore::write_at` then `read_at` — the emulator's own guest RAM | 16 to 61 |
//! | two `cpu::x86` cores, `mov [X],1` then `mov eax,[Y]` | **0** |
//!
//! The fourth and fifth rows are the finding, and between them they record a
//! premise that was true when this file was written and is not true now.
//!
//! **`RamStore` used to contain a barrier on this host, by accident.** Every
//! one of its writes ends in
//! [`mark_dirty`](rsemu::core::space::RamStore::mark_dirty), which used to set
//! its bit with an unconditional `AtomicU64::fetch_or` — a *relaxed* atomic
//! read-modify-write, which on x86-64 is a `lock or`, and a locked instruction
//! is a full barrier (*Intel SDM* volume 3 §9.2.5). So a guest store to RAM was
//! followed by a store-buffer drain whether anybody wanted one or not, on the
//! interpreter and in a translated block alike — `jit::Tlb::note_fast_store`
//! marks the same bitmap after an inlined store.
//!
//! An accident is worth exactly as much as an accident, and this one was
//! bought at 3.6 ns of a 25.4 ns store. `mark_dirty` now tests the bit before
//! it sets it, which in the steady state is a plain relaxed load and no locked
//! instruction at all — so the fifth row has moved from **0** to the same order
//! of magnitude as the unfenced control, and the fourth row is the attribution:
//! the same word, the same access, only the branch added, and the barrier is
//! gone. What that gave up, and the safe-point condition that makes giving it
//! up sound, is argued in full on `mark_dirty` itself.
//!
//! Nothing that was a guarantee changed, because none of this ever was one: it
//! is x86-only (`fetch_or(Relaxed)` on AArch64 is `ldsetr`, which orders
//! nothing), it covered only the store-then-load case, and it did nothing for a
//! barrier between two *loads* or between two *loads and a store*. The second
//! row is the guarantee, and the three interpreters and the x86 backend all
//! emit it: `IrHost::fence`, `A64::host_fence`, and `jit::x86::compile`'s
//! `mfence`.
//!
//! The last row is the same statement about the emulator's *cost*, and it is
//! still **0** even now that the dirty bit no longer drains anything: a guest
//! store and the guest load after it are separated by a whole interpreted
//! instruction, and `tests/memory_model_costs.rs` puts the host's window at
//! about forty nanoseconds. So the interpreter's own overhead closes it
//! without help. That is not a reason to keep the guest barrier a no-op — it
//! is a reason the omission cost nothing *so far*, on *one* host, and it is
//! why the row that shows the change had to be built out of bare atomics
//! rather than out of guest instructions.
//!
//! # What is asserted and what is printed
//!
//! How often a relaxed run tears is a property of the host's scheduler, so the
//! control prints its count and asserts only that the two threads overlapped.
//! The fenced rows assert **zero**, and that assertion is sound on every host:
//! a sequentially consistent fence on both sides of a store-buffer test forbids
//! the outcome in the language's model, not merely on this silicon.
//!
//! # The rows that need a weakly ordered host
//!
//! Three fixes shipped with no test that can fail on an x86-64 host. Each says
//! so in its own doc comment, and each names *this file* as where its row
//! belongs "taken on a weakly ordered host". Two of them are here now:
//!
//! | row | what it gates | where the code is |
//! | --- | --- | --- |
//! | a bus lock between the store and the load | `BusLock`'s two `SeqCst` fences | `core::space::buslock` |
//! | the same, with the fences removed | the negative control for the row above | this file |
//! | two `cpu.arm.a64` cores, `STLR`/`LDAR` | `Exec::host_fence` on an acquire/release access | `cpu::arm::a64::exec` |
//! | the same two cores, `STR`, `DMB ISH`, `LDR` | `Exec::host_fence` on a barrier | `cpu::arm::a64::exec` |
//! | the same two cores, `STR`/`LDR` | the negative control for the two above | this file |
//!
//! The third fix — the JIT's `mfence` lowering (`jit::x86::Assembler::mfence`,
//! emitted by `jit::x86::compile`) — has **no row here and cannot have one**.
//! That backend is gated on `target_arch = "x86_64"`; there is no AArch64 host
//! backend, so on the only host where the row would mean anything the code
//! under test does not exist. `jit::x86::tests` asserts the bytes are emitted
//! where the guest barrier is, which is the whole of what is checkable until a
//! second host backend exists. Said here rather than left to be discovered.
//!
//! ## Why a *negative* control, and not just the fenced row
//!
//! Every fenced row below asserts **zero**, and that assertion is sound on
//! every host — which is exactly why, on its own, it proves nothing about the
//! fences. A row that would also read zero with the fences deleted is a green
//! tick for a property nothing checked. So each fenced row is paired with the
//! same critical section minus the fences, reconstructed here
//! ([`Unfenced`]), and on a weakly ordered host that pair is the measurement:
//! the fenced arm is zero, the unfenced arm is not.
//!
//! On x86-64 both arms are zero — the mutex's own `lock cmpxchg` and the
//! relaxed `fetch_add` on the transaction counter are each a full barrier
//! there — and that is the whole reason these rows had to wait for a runner.
//!
//! # Required mode
//!
//! `RSEMU_WEAK_MEMORY_REQUIRED=1` turns two *printed* counts into assertions:
//!
//! * the bare-relaxed control must produce the forbidden outcome, or the
//!   instrument is not sensitive on this host and every zero below is vacuous;
//! * [`Between::DirtyBit`] must produce it too, which is the discriminator
//!   between a weakly ordered host and the accident this file was written to
//!   name — `fetch_or(Relaxed)` is `lock or` on x86-64 and `ldsetr` on
//!   AArch64, and only on the second is it not a barrier.
//!
//! Both are probabilistic, so both are retried ([`ATTEMPTS`]) before they
//! fail. A red here is not "the emulator is broken": it is "this runner did
//! not reorder anything, so do not believe the green ticks next to it". The
//! counts are printed either way, which is what makes that diagnosable.
//!
//! ```text
//!   RSEMU_WEAK_MEMORY_REQUIRED=1 cargo test --release --all-features \
//!       --test memory_model_litmus -- --nocapture --test-threads=1
//! ```
//!
//! `--release` because [`ROUNDS`] is ten times smaller without it, and
//! `--test-threads=1` because three of these run three threads apiece and a
//! second one competing for the same cores moves every count in this file.
//!
//! # What a green run does not prove
//!
//! A weak-memory bug is probabilistic. A litmus that finds nothing has failed
//! to find something; it has not shown there is nothing to find. What the
//! required-mode pair above buys is the *lower* bound — that this host tears
//! at all, at some measured rate, under this instrument — and nothing more.
//! Every zero in this file should be read as "not observed in [`ROUNDS`]
//! rounds on this host", which is why the unfenced controls are printed next
//! to them.
//!
//! `std::thread` rather than `core::sync::Pool`, for the reason
//! `tests/smp_single_copy_atomicity.rs` gives: a test whose whole purpose is to
//! make two cores collide has to be able to say what a thread is.
//!
//! # …and the same litmus against a whole machine
//!
//! Everything above spawns its own threads. That is not what `ROADMAP.md` §8's
//! gate asks for — it asks about **SMP emulation** — so the `machine` module at
//! the bottom of this file runs the store-buffer test, and a message-passing
//! test beside it, as guest programs on two boards that declare `threading
//! parallel` in their own machine files. Its own documentation is the argument;
//! two things about it belong up here.
//!
//! **It is a blunter instrument, measurably.** An interpreted guest instruction
//! plus the scheduler's per-access accounting separate the store from the load,
//! which is about as wide as the host's whole store-buffer window, so the
//! machine-level store-buffer rows read zero in *every* arm — fenced and
//! unfenced alike, on a strongly and a weakly ordered host. They check that a
//! board still executes the barrier; the pair up here is what discriminates.
//!
//! **The message-passing shape is the one that found something**, because its
//! reader is already spinning when its writer stores and so needs no
//! rendezvous at all. What it found was a torn load rather than a reordering —
//! `RamStore` has no single-copy atomicity — and
//! `machine::rv::a_torn_flag_load_is_visible_through_a_whole_machine` is that,
//! reproduced and `#[ignore]`d.

#![cfg(feature = "std")]

use std::sync::atomic::{AtomicU8, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Barrier};

use rsemu::core::space::{BusLock, RamStore};
use rsemu::core::sync as rsync;
use rsemu::core::sync::LockRank;

/// Rounds per run: the same 200 000 `tests/memory_model_costs.rs` uses, so the
/// counts here and there are comparable — in a release build.
///
/// A tenth of that under `debug_assertions`, because two of these run 200 000
/// rounds of a *guest* program through the x86 interpreter and an unoptimised
/// interpreter turns a five-second file into a minute of `cargo test`. The
/// assertions do not depend on the count: the ones with teeth are "zero" and
/// "the two threads overlapped", and both hold at either size. What a short
/// run costs is sensitivity in the control, which is printed rather than
/// asserted for exactly that reason.
const ROUNDS: usize = if cfg!(debug_assertions) {
    20_000
} else {
    200_000
};

/// How many times a required-mode sensitivity check is re-run before it fails.
///
/// The checks are searches, not proofs, so a single empty run is weak evidence
/// and four are much stronger: on any host that reorders at all, each run is
/// [`ROUNDS`] independent chances, and the arithmetic is in the module
/// documentation. Four rather than one because a false red here costs a
/// re-run of the whole job and teaches nobody anything; four rather than forty
/// because a host that needs forty attempts is a host whose zeros below are
/// not worth much either, and that is what the check exists to say.
const ATTEMPTS: usize = 4;

/// Whether this run is on a host chosen for its memory model.
///
/// The same shape as `RSEMU_CROSSHOST_REQUIRED` in `scripts/check.sh`: a
/// developer's run prints what it found, and the job that exists *for* the
/// finding fails when there is none. A runner whose provisioning changed under
/// us would otherwise report a held gate that nothing was holding.
fn weak_memory_required() -> bool {
    match std::env::var("RSEMU_WEAK_MEMORY_REQUIRED") {
        Ok(v) => !v.is_empty() && v != "0",
        Err(_) => false,
    }
}

/// Where the two flags live in the model store, a page apart so neither
/// prefetching nor false sharing decides the answer.
const FLAG: [u64; 2] = [0x2000, 0x9000];

/// What one round of the litmus puts between the store and the load.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Between {
    /// Nothing at all: the control, and the shape `core::sync`'s analysis
    /// describes.
    Nothing,
    /// [`rsync::fence`] at [`Ordering::SeqCst`] — exactly what `IrHost::fence`
    /// and the three interpreters now execute for a guest barrier.
    Fence,
    /// A relaxed `fetch_or` on an unrelated word: what
    /// [`RamStore::mark_dirty`] did after every write until the round that
    /// priced it, reproduced on its own so that the level below can be
    /// attributed to it rather than to luck.
    ///
    /// Kept although `mark_dirty` no longer does this, because it is the
    /// control that explains the row under it. Without it, "through `RamStore`"
    /// changing from zero to hundreds is a number with no cause attached.
    DirtyBit,
    /// A relaxed *load* of the same word, and the `fetch_or` only if the bit
    /// is clear: what [`RamStore::mark_dirty`] does now.
    ///
    /// The bit is set on the first round and every round after it takes the
    /// branch, so this is the steady state — a plain load, no locked
    /// instruction, no barrier.
    DirtyBitTested,
    /// Nothing — but the store and the load go through [`RamStore`] rather
    /// than through a bare atomic, so the dirty bit is really in the path.
    ThroughRamStore,
    /// A whole [`BusLock`] transaction: `acquire`, then the guard's `Drop`.
    ///
    /// The row `core::space::buslock`'s own test module says it cannot write.
    /// Each thread takes **its own** lock, because two threads sharing one
    /// would be serialized by the exclusion and the litmus would be a
    /// tautology — the module documentation's "a mutex critical section over a
    /// word the other thread never touches", which is also what a real bus
    /// lock's *barrier* has to hold up without any contention at all.
    BusLock,
    /// [`Unfenced`]: the same transaction with the two `SeqCst` fences deleted.
    ///
    /// The negative control, and the only thing that makes the row above an
    /// assertion about the fences rather than about the mutex.
    Unfenced,
}

/// `BusLock` with the two fences deleted, and nothing else changed.
///
/// Reconstructed here rather than reached through a flag on the real type: the
/// fences are the thing under test, and a switch that turned them off would be
/// shipped code with a mode nothing in the emulator ever selects. This is a
/// transcription of `BusLock::acquire`, `BusLock::claim` and
/// `BusLockGuard::drop` — the same `Mutex` at the same
/// [`LockRank::BUS_LOCK`], the same `held` flag stored `Release` then
/// `Relaxed`-cleared in the same order, the same `Relaxed` `fetch_add` on the
/// transaction counter — minus `fence(SeqCst)`.
///
/// If `buslock.rs` changes shape, this stops being the control it claims to
/// be. That is a real maintenance cost and it is the price of having any
/// regression net for those fences at all.
struct Unfenced {
    inner: rsync::Mutex<()>,
    held: rsync::AtomicBool,
    taken: AtomicU64,
}

impl Unfenced {
    fn new() -> Unfenced {
        Unfenced {
            inner: rsync::Mutex::with_rank(LockRank::BUS_LOCK, ()),
            held: rsync::AtomicBool::new(false),
            taken: AtomicU64::new(0),
        }
    }

    /// One whole transaction: everything `BusLock` does except order anything.
    fn transaction(&self) {
        let inner = self.inner.lock();
        self.held.store(true, Ordering::Release);
        self.taken.fetch_add(1, Ordering::Relaxed);
        self.held.store(false, Ordering::Release);
        drop(inner);
    }
}

/// What a run of the litmus produced.
#[derive(Debug)]
struct Outcome {
    /// Rounds in which **both** loads returned zero: the forbidden outcome.
    both_zero: usize,
    /// Rounds in which at least one load returned zero.
    ///
    /// The proof that the two threads really did overlap. A run in which one
    /// thread finished before the other started would report no forbidden
    /// outcome and mean nothing, and this is how that is told apart.
    witnessed: usize,
}

/// One store-buffer run: two threads, a referee, `ROUNDS` rounds.
///
/// Three threads rather than two, for the reason `memory_model_costs` gives:
/// without a referee the two litmus threads would have to agree about when a
/// round ended, which needs the very ordering the test is measuring.
fn store_buffer(between: Between) -> Outcome {
    let ram = Arc::new(RamStore::new(0x1_0000));
    let flag = [Arc::new(AtomicU8::new(0)), Arc::new(AtomicU8::new(0))];
    // Two words, one per thread, so the model of the dirty bitmap contends
    // exactly as little as the real one does.
    let dirty = Arc::new([AtomicU64::new(0), AtomicU64::new(0)]);
    let seen = [Arc::new(AtomicU32::new(9)), Arc::new(AtomicU32::new(9))];
    let both_zero = Arc::new(AtomicU32::new(0));
    let witnessed = Arc::new(AtomicU32::new(0));
    let gate = Arc::new(Barrier::new(3));

    std::thread::scope(|s| {
        for who in 0..2 {
            let ram = Arc::clone(&ram);
            let flag = [Arc::clone(&flag[0]), Arc::clone(&flag[1])];
            let dirty = Arc::clone(&dirty);
            let out = Arc::clone(&seen[who]);
            let gate = Arc::clone(&gate);
            s.spawn(move || {
                let them = 1 - who;
                // One lock per thread, built once. The other thread never
                // touches either, so nothing here serializes the two — see
                // `Between::BusLock`.
                let bus = BusLock::new();
                let unfenced = Unfenced::new();
                for _ in 0..ROUNDS {
                    gate.wait();
                    let value = if between == Between::ThroughRamStore {
                        ram.write_u8(FLAG[who], 1).expect("the store lands");
                        u32::from(ram.read_u8(FLAG[them]).expect("the load reads"))
                    } else {
                        flag[who].store(1, Ordering::Relaxed);
                        match between {
                            Between::Nothing | Between::ThroughRamStore => {}
                            Between::Fence => rsync::fence(rsync::Ordering::SeqCst),
                            Between::DirtyBit => {
                                dirty[who].fetch_or(1, Ordering::Relaxed);
                            }
                            Between::DirtyBitTested => {
                                if dirty[who].load(Ordering::Relaxed) & 1 == 0 {
                                    dirty[who].fetch_or(1, Ordering::Relaxed);
                                }
                            }
                            Between::BusLock => drop(bus.acquire()),
                            Between::Unfenced => unfenced.transaction(),
                        }
                        u32::from(flag[them].load(Ordering::Relaxed))
                    };
                    out.store(value, Ordering::Relaxed);
                    gate.wait();
                }
            });
        }
        for _ in 0..ROUNDS {
            gate.wait();
            gate.wait();
            let read = [
                seen[0].load(Ordering::Relaxed),
                seen[1].load(Ordering::Relaxed),
            ];
            if read.iter().all(|v| *v == 0) {
                both_zero.fetch_add(1, Ordering::Relaxed);
            }
            if read.contains(&0) {
                witnessed.fetch_add(1, Ordering::Relaxed);
            }
            for who in 0..2 {
                flag[who].store(0, Ordering::Relaxed);
                ram.write_u8(FLAG[who], 0).expect("the reset lands");
                seen[who].store(9, Ordering::Relaxed);
            }
        }
    });

    Outcome {
        both_zero: both_zero.load(Ordering::Relaxed) as usize,
        witnessed: witnessed.load(Ordering::Relaxed) as usize,
    }
}

/// Run one arm until it produces the forbidden outcome, or [`ATTEMPTS`] times.
///
/// Only the two arms required mode gates on are searched. Everywhere else a
/// single run is right: those assert **zero**, and re-running until a run came
/// back empty would be searching for the answer we wanted.
fn search(between: Between, label: &str) -> Outcome {
    let attempts = if weak_memory_required() { ATTEMPTS } else { 1 };
    let mut last = None;
    for attempt in 1..=attempts {
        let out = store_buffer(between);
        println!(
            "{label}: forbidden outcome {} / {ROUNDS} ({} overlapped, attempt {attempt}/{attempts})",
            out.both_zero, out.witnessed
        );
        let tore = out.both_zero > 0;
        last = Some(out);
        if tore {
            break;
        }
    }
    last.expect("attempts is at least one")
}

/// The control: with nothing between the store and the load, the host produces
/// the outcome a guest barrier exists to forbid.
///
/// Printed rather than asserted — how often depends on the host's scheduler,
/// and on a machine that happens to run the two threads one after the other it
/// is zero. What is asserted is that the run means something.
///
/// Under `RSEMU_WEAK_MEMORY_REQUIRED` the count itself is asserted: a job that
/// exists to run this instrument on a weakly ordered host has to fail when the
/// instrument found nothing, or every zero it reports beside it is a green tick
/// for a property nothing checked.
#[test]
fn a_relaxed_store_and_the_load_after_it_can_be_reordered() {
    let out = search(Between::Nothing, "nothing between");
    assert!(
        out.witnessed > 0,
        "no round ever saw the other thread's zero, so the run proves nothing"
    );
    if weak_memory_required() {
        assert!(
            out.both_zero > 0,
            "RSEMU_WEAK_MEMORY_REQUIRED is set and {ATTEMPTS} runs of {ROUNDS} \
             rounds over two bare relaxed atomics never reordered anything. \
             This is not a defect in the emulator: it says this host did not \
             overlap the two threads, so the zeros the rest of this file \
             reports are vacuous and must not be read as gates."
        );
    }
}

/// The fix, at the level the fix is written: one host fence.
///
/// Zero is asserted rather than printed because it is not a property of this
/// host. A `SeqCst` fence between the store and the load on **both** sides of a
/// store-buffer test forbids the outcome in the C++20 model Rust follows, so a
/// non-zero count here would be a defect in the language implementation or in
/// this test, not a slow day for the scheduler.
#[test]
fn a_host_fence_forbids_it() {
    let out = store_buffer(Between::Fence);
    println!(
        "core::sync::fence(SeqCst): forbidden outcome {} / {ROUNDS} ({} rounds overlapped)",
        out.both_zero, out.witnessed
    );
    assert!(out.witnessed > 0, "the two threads must overlap");
    assert_eq!(
        out.both_zero, 0,
        "a sequentially consistent fence on both sides cannot permit this"
    );
}

/// **The finding, and the round that ended it.** `RamStore`'s dirty bitmap was
/// a barrier on a host whose relaxed read-modify-write is a locked
/// instruction; it is not one any more, and this is where that shows.
///
/// Three arms, because attributing the third to the first two is the whole
/// point: a relaxed `fetch_or` on a word nothing else touches, the same word
/// with the test-before-set [`RamStore::mark_dirty`] now does, and then the
/// real `RamStore` write and read that used to contain the first and now
/// contain the second.
///
/// Printed, not asserted, in every arm. Nothing in the language promised the
/// old zero — it was a property of how x86-64 implements a `lock or`, and on
/// AArch64 the same `fetch_or(Relaxed)` orders nothing at all — so a test that
/// asserted zero would have been asserting the accident, which is the opposite
/// of what this file is for. The same reasoning forbids asserting a *non*-zero
/// now: how often an unfenced store-buffer test reorders is the host's
/// business, and a machine that ran the two threads consecutively would print
/// zero for a reason that has nothing to do with the change.
#[test]
fn the_dirty_bitmap_is_no_longer_a_barrier_on_a_locked_host() {
    // `search` rather than one `store_buffer`: this arm is the required-mode
    // discriminator below, and a search that must find something is retried
    // before it is believed.
    let bit = search(Between::DirtyBit, "a relaxed fetch_or between");
    let tested = store_buffer(Between::DirtyBitTested);
    let store = store_buffer(Between::ThroughRamStore);
    println!(
        "test-before-set between:     forbidden outcome {} / {ROUNDS} ({} overlapped)",
        tested.both_zero, tested.witnessed
    );
    println!(
        "through RamStore:            forbidden outcome {} / {ROUNDS} ({} overlapped)",
        store.both_zero, store.witnessed
    );
    assert!(
        bit.witnessed > 0 && tested.witnessed > 0 && store.witnessed > 0,
        "every run must overlap or none of them says anything"
    );
    if weak_memory_required() {
        // The discriminator, and the reason this arm rather than the bare
        // control is the second required check. `fetch_or(Relaxed)` is
        // `lock or` on x86-64 and `ldsetr` on AArch64: a barrier on the first
        // and nothing at all on the second. A host where this tears is a host
        // where the accident named above is absent, which is the precondition
        // for every fenced row in this file meaning anything.
        //
        // `ThroughRamStore` is deliberately *not* required. It is the same
        // access with the store's bounds check, page lookup and dirty-bit
        // arithmetic in front of it, and that work narrows the window on its
        // own — `tests/memory_model_costs.rs` puts the close at about forty
        // nanoseconds on the author's host. Requiring it would be gating on
        // how fast a runner executes `RamStore::write_u8`.
        assert!(
            bit.both_zero > 0,
            "RSEMU_WEAK_MEMORY_REQUIRED is set and a relaxed read-modify-write \
             between the store and the load suppressed the outcome over \
             {ATTEMPTS} runs. That is the x86-64 accident this file is named \
             after, so this host is not the weakly ordered one the job asked \
             for — check the runner label before believing anything below."
        );
    }
}

/// **The bus lock's two fences**, which no test on an x86-64 host can gate.
///
/// `core::space::buslock`'s own test module says so in as many words, and names
/// this file as where the row belongs. Two arms:
///
/// * a whole [`BusLock`] transaction between the store and the load — the code
///   a `LOCK`-prefixed guest instruction runs, fences and all;
/// * [`Unfenced`], the same transaction with `fence(SeqCst)` deleted from
///   `claim` and from the guard's `Drop`, and nothing else changed.
///
/// The first asserts zero and that assertion is sound everywhere: two `SeqCst`
/// fences bracketing the critical section forbid the outcome in the language's
/// model. What only a weakly ordered host can add is the second arm — on
/// x86-64 it is also zero, because the mutex's `lock cmpxchg` and the
/// `fetch_add` on the transaction counter are each a full barrier there, and
/// the pair therefore says nothing. Where the second arm tears and the first
/// does not, the fences are doing the work the module documentation claims.
///
/// The second arm is printed, not asserted, on every host. Requiring it would
/// gate on a mutex round trip being slower than the store buffer's drain,
/// which is a property of the runner rather than of this crate.
#[test]
fn a_bus_lock_forbids_it_and_the_same_transaction_unfenced_need_not() {
    let locked = store_buffer(Between::BusLock);
    let unfenced = store_buffer(Between::Unfenced);
    println!(
        "a bus lock between:          forbidden outcome {} / {ROUNDS} ({} overlapped)",
        locked.both_zero, locked.witnessed
    );
    println!(
        "the same, fences deleted:    forbidden outcome {} / {ROUNDS} ({} overlapped)",
        unfenced.both_zero, unfenced.witnessed
    );
    assert!(
        locked.witnessed > 0 && unfenced.witnessed > 0,
        "both runs must overlap or neither says anything"
    );
    assert_eq!(
        locked.both_zero, 0,
        "a bus lock fences on both sides; this outcome is what those fences forbid"
    );
}

// ---------------------------------------------------------------------------
// The same litmus in guest instructions
// ---------------------------------------------------------------------------

/// Two `cpu::x86` cores on two host threads, running the store-buffer test as
/// **guest code**, with and without the `MFENCE` this round stopped ignoring.
#[cfg(feature = "cpu-x86")]
mod guest {
    use super::{Arc, AtomicU32, Barrier, Ordering, ROUNDS};

    use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region};
    use rsemu::core::value::Width;
    use rsemu::cpu::x86::{Config, Features, Variant, X86};

    /// The two flags and the two result slots, in the guest's first sixteen
    /// bits so a real-mode `moffs` encoding can name them.
    const FLAG: [u16; 2] = [0x2000, 0x3000];
    const SEEN: [u16; 2] = [0x2010, 0x3010];
    /// Where each core's program is placed.
    const ENTRY: [u64; 2] = [0x0100, 0x0200];
    /// What the referee writes into a result slot before a round, so that a
    /// core which never ran is distinguishable from one that loaded zero.
    const UNSET: u64 = 9;

    /// One core's program.
    ///
    /// ```text
    ///   mov dword [mine], 1
    ///   mfence                 ; only in the fenced variant
    ///   mov eax, [theirs]
    ///   mov [seen], eax
    /// ```
    ///
    /// `66 C7 /0` is `MOV Ev, Iz` and the ModRM byte `06` is the 16-bit
    /// direct-address form, so the destination is `ds:imm16` with `DS` zero;
    /// `66 A1`/`66 A3` are `MOV eAX, moffs` and its reverse. `0F AE F0` is
    /// `MFENCE`, which needs a part with SSE2 — hence [`Variant::X86_64`]
    /// rather than the 486 the neighbouring tests use.
    fn program(who: usize, fence: bool) -> Vec<u8> {
        let (mine, theirs, seen) = (FLAG[who], FLAG[1 - who], SEEN[who]);
        let mut c = vec![0x66, 0xc7, 0x06, (mine & 0xff) as u8, (mine >> 8) as u8];
        c.extend_from_slice(&1u32.to_le_bytes());
        if fence {
            c.extend_from_slice(&[0x0f, 0xae, 0xf0]);
        }
        c.extend_from_slice(&[0x66, 0xa1, (theirs & 0xff) as u8, (theirs >> 8) as u8]);
        c.extend_from_slice(&[0x66, 0xa3, (seen & 0xff) as u8, (seen >> 8) as u8]);
        c
    }

    /// How many instructions [`program`] assembles to.
    fn length(fence: bool) -> usize {
        if fence { 4 } else { 3 }
    }

    fn space(fence: bool) -> Arc<AddressSpace> {
        let space = Arc::new(AddressSpace::new("mem", 32));
        space
            .topology()
            .map(Region::ram("ram", Arc::new(RamStore::new(0x4_0000))), 0)
            .expect("256 KiB at zero");
        for (who, at) in ENTRY.iter().enumerate() {
            space
                .write_bytes(*at, &program(who, fence), MemAttrs::DEFAULT)
                .expect("the program lands");
        }
        space
    }

    /// A core in real mode at `entry`.
    ///
    /// The first `step` performs the power-on reset, which discards any
    /// register file written before it — so it is spent here, exactly as
    /// `tests/smp_single_copy_atomicity.rs` does, and only then is the core
    /// placed.
    fn core(space: &Arc<AddressSpace>, entry: u64) -> Arc<X86> {
        let cpu = Arc::new(X86::new(
            Config::default()
                .with_variant(Variant::X86_64)
                .with_features(Features::X86_64),
        ));
        cpu.attach_space(Arc::clone(space));
        cpu.step();
        let mut regs = cpu.regs();
        regs.cs = 0;
        regs.ds = 0;
        regs.es = 0;
        regs.ss = 0;
        regs.rip = entry;
        cpu.set_regs(regs);
        cpu
    }

    /// Reset both flags and both result slots, and read what the last round
    /// left. Runs on the referee, with both cores parked at the barrier.
    fn round_end(space: &Arc<AddressSpace>) -> [u64; 2] {
        let mut seen = [0u64; 2];
        for who in 0..2 {
            seen[who] = space
                .read(u64::from(SEEN[who]), Width::U32, MemAttrs::DEFAULT)
                .expect("the slot reads back");
        }
        for who in 0..2 {
            for at in [FLAG[who], SEEN[who]] {
                let value = if at == SEEN[who] { UNSET } else { 0 };
                space
                    .write(u64::from(at), Width::U32, value, MemAttrs::DEFAULT)
                    .expect("the reset lands");
            }
        }
        seen
    }

    /// What one guest run produced.
    struct Guest {
        both_zero: usize,
        witnessed: usize,
        never_ran: usize,
    }

    /// The litmus, in guest instructions, on two host threads.
    fn run(fence: bool) -> Guest {
        let space = space(fence);
        let insns = length(fence);
        let both_zero = Arc::new(AtomicU32::new(0));
        let witnessed = Arc::new(AtomicU32::new(0));
        let never_ran = Arc::new(AtomicU32::new(0));
        let gate = Arc::new(Barrier::new(3));
        std::thread::scope(|s| {
            for (who, at) in ENTRY.iter().enumerate() {
                let cpu = core(&space, *at);
                let gate = Arc::clone(&gate);
                s.spawn(move || {
                    for _ in 0..ROUNDS {
                        gate.wait();
                        // Rewinding is what makes a round a round: the program
                        // is straight-line, so the core is placed back at its
                        // entry rather than looping, and no `HLT` has to be
                        // un-halted.
                        let mut regs = cpu.regs();
                        regs.rip = ENTRY[who];
                        cpu.set_regs(regs);
                        for _ in 0..insns {
                            cpu.step();
                        }
                        gate.wait();
                    }
                });
            }
            for _ in 0..ROUNDS {
                gate.wait();
                gate.wait();
                let seen = round_end(&space);
                if seen.contains(&UNSET) {
                    never_ran.fetch_add(1, Ordering::Relaxed);
                } else {
                    if seen.iter().all(|v| *v == 0) {
                        both_zero.fetch_add(1, Ordering::Relaxed);
                    }
                    if seen.contains(&0) {
                        witnessed.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        });
        Guest {
            both_zero: both_zero.load(Ordering::Relaxed) as usize,
            witnessed: witnessed.load(Ordering::Relaxed) as usize,
            never_ran: never_ran.load(Ordering::Relaxed) as usize,
        }
    }

    /// The `MFENCE` arm, end to end: decoded, gated on SSE, and executed as a
    /// host fence.
    ///
    /// `never_ran` is the assertion that keeps this honest. Before this round
    /// `MFENCE` retired as a no-op, and if it ever stopped decoding — an
    /// invalid-opcode fault, a gate that rejected it in real mode — the two
    /// instructions after it would not run, the result slot would still hold
    /// [`UNSET`], and every count below would be zero for the wrong reason.
    #[test]
    fn a_guest_mfence_between_the_store_and_the_load() {
        let out = run(true);
        println!(
            "guest, with MFENCE:    forbidden outcome {} / {ROUNDS} ({} overlapped)",
            out.both_zero, out.witnessed
        );
        assert_eq!(
            out.never_ran, 0,
            "a round did not reach the load after the MFENCE"
        );
        assert!(out.witnessed > 0, "the two cores must overlap");
        assert_eq!(
            out.both_zero, 0,
            "the guest executed the instruction that forbids this"
        );
    }

    /// The same programs without the `MFENCE`, so the fenced count above is
    /// read against something.
    ///
    /// On an x86-64 host this is **also** zero, and that is the measurement
    /// rather than the fix: a whole interpreted instruction separates the store
    /// from the load, and `tests/memory_model_costs.rs` puts the host's
    /// store-buffer window at about forty nanoseconds — well inside it. It used
    /// to be zero for a second reason as well, the locked `fetch_or` in
    /// `RamStore::mark_dirty`; that one is gone and this row did not move,
    /// which is the cleanest evidence that the interpreter's own overhead was
    /// always doing the work here. So the count is printed, not asserted.
    #[test]
    fn the_same_programs_without_one() {
        let out = run(false);
        println!(
            "guest, without MFENCE: forbidden outcome {} / {ROUNDS} ({} overlapped)",
            out.both_zero, out.witnessed
        );
        assert_eq!(out.never_ran, 0, "a round did not run to completion");
        assert!(out.witnessed > 0, "the two cores must overlap");
    }

    /// The mode every golden state hash comes from cannot produce the outcome,
    /// and the argument is structural rather than statistical.
    ///
    /// One host thread, a switch after **every single instruction** — finer
    /// than any quantum `ThreadingMode::Deterministic` will ever hand out. Each
    /// core's own store precedes its own load in program order, so a
    /// sequential interleaving that had both loads read zero would need
    /// `A.store < A.load < B.store < B.load < A.store`. There is no such
    /// order, so this is a gate rather than a sample: whatever it cannot
    /// produce, that mode cannot produce.
    ///
    /// Randomised rather than strictly alternating for the reason
    /// `tests/smp_single_copy_atomicity.rs` records: a fixed one-for-one
    /// schedule phase-locks and stops overlapping the two programs at all. The
    /// generator is a fixed-seed LCG, so this stays a deterministic test of the
    /// deterministic mode.
    #[test]
    fn sequential_interleaving_cannot_produce_it() {
        // Fewer rounds than the threaded runs: this one is a proof with a
        // witness rather than a search, and it runs both cores on this thread.
        const N: usize = 20_000;
        let space = space(false);
        let cores = [core(&space, ENTRY[0]), core(&space, ENTRY[1])];
        let insns = length(false);
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let (mut both_zero, mut witnessed) = (0usize, 0usize);
        for _ in 0..N {
            for who in 0..2 {
                let mut regs = cores[who].regs();
                regs.rip = ENTRY[who];
                cores[who].set_regs(regs);
            }
            let mut left = [insns, insns];
            while left[0] + left[1] > 0 {
                seed = seed
                    .wrapping_mul(6_364_136_223_846_793_005)
                    .wrapping_add(1_442_695_040_888_963_407);
                let which = ((seed >> 33) & 1) as usize;
                let which = if left[which] > 0 { which } else { 1 - which };
                cores[which].step();
                left[which] -= 1;
            }
            let seen = round_end(&space);
            assert!(
                seen.iter().all(|v| *v != UNSET),
                "a round did not run to completion"
            );
            if seen.iter().all(|v| *v == 0) {
                both_zero += 1;
            }
            if seen.contains(&0) {
                witnessed += 1;
            }
        }
        println!(
            "interleaved:           forbidden outcome {both_zero} / {N} ({witnessed} overlapped)"
        );
        assert!(
            witnessed > 0,
            "the schedule never overlapped the two programs, so it proves nothing"
        );
        assert_eq!(
            both_zero, 0,
            "one host thread produced an outcome no sequential interleaving admits"
        );
    }
}

// ---------------------------------------------------------------------------
// The same litmus in AArch64 guest instructions
// ---------------------------------------------------------------------------

/// Two `cpu.arm.a64` cores on two host threads, running the store-buffer test
/// as **guest code**, with `STLR`/`LDAR`, with `DMB ISH`, and with neither.
///
/// The row `cpu::arm::a64::exec`'s [`Exec::host_fence`] doc comment says is
/// missing. Its argument, restated: this core issues the guest's accesses in
/// program order, but each becomes a *relaxed* host operation, so the host's
/// model applies underneath, and `LDAR`, `STLR` and `DMB` are the guest asking
/// for ordering its baseline does not give it. Until `host_fence` existed they
/// were an ordinary load, an ordinary store and nothing at all.
///
/// # Why this needs a weakly ordered host and the x86 module below does not
///
/// The same reason the module documentation gives twice over. On x86-64 the
/// store's `RamStore::mark_dirty` is a `lock or` and the only reordering
/// x86-TSO permits is the one it forbids, so all three arms read zero and the
/// fenced ones prove nothing. On AArch64 that `fetch_or` is `ldsetr` and
/// orders nothing, so the unfenced arm is free to tear and the two fenced arms
/// are a claim about `host_fence` rather than about the host.
///
/// Note what is *not* claimed: the unfenced arm is printed. A whole interpreted
/// guest instruction separates the store from the load, and
/// `tests/memory_model_costs.rs` measures that window closing at tens of
/// nanoseconds — so an unfenced arm reading zero is an ordinary outcome and
/// means the emulator's own overhead covered the window on this run, not that
/// the guest got the ordering it asked for.
#[cfg(feature = "cpu-arm-a64")]
mod guest_a64 {
    use super::{Arc, AtomicU32, Barrier, Ordering, ROUNDS};

    use rsemu::core::space::{AddressSpace, MemAttrs, RamStore, Region};
    use rsemu::core::value::Width;
    use rsemu::cpu::arm::a64::{Config, Cpu};

    /// The two flags and the two result slots. Word-aligned, because an
    /// acquire/release access must be aligned whatever `SCTLR_EL1.A` says.
    const FLAG: [u64; 2] = [0x2000, 0x3000];
    const SEEN: [u64; 2] = [0x2010, 0x3010];
    /// Where each core's program is placed, and where its round body starts.
    const ENTRY: [u64; 2] = [0x0100, 0x0400];
    /// Four setup instructions precede the body.
    const BODY: u64 = 16;
    /// What the referee writes into a result slot before a round, so a core
    /// that never ran is distinguishable from one that loaded zero.
    const UNSET: u64 = 9;

    /// What sits between the guest's store and the guest's load.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    enum Fenced {
        /// `STR` then `LDR`: the control, and what this core did before
        /// `host_fence` existed.
        Neither,
        /// `STLR` then `LDAR`. A64's acquire/release is RCsc (DDI 0487 B2.3),
        /// so a Store-Release before a Load-Acquire in program order is
        /// ordered — which is precisely the outcome below.
        AcquireRelease,
        /// `STR`, `DMB ISH`, `LDR`: the explicit barrier, `Op::Dmb`.
        Dmb,
    }

    // `MOVZ Xd, #imm16`; `MOVZ Wd, #imm16`; the unsigned-offset `STR`/`LDR`
    // with a zero immediate; `STLR`/`LDAR`; and `DMB ISH`. Encodings from
    // DDI 0487, C6.2 (`MOVZ`, `STR`, `LDR`, `STLR`, `LDAR`) and C6.2.79
    // (`DMB`); the barrier's `CRm` of `0b1011` is the inner-shareable domain,
    // full system, which is B2.3.7's mapping for a sequentially consistent
    // access on this architecture.
    fn movz64(rd: u32, imm: u64) -> u32 {
        0xd280_0000 | ((imm as u32 & 0xffff) << 5) | rd
    }
    fn movz32(rd: u32, imm: u32) -> u32 {
        0x5280_0000 | ((imm & 0xffff) << 5) | rd
    }
    fn str32(rt: u32, rn: u32) -> u32 {
        0xb900_0000 | (rn << 5) | rt
    }
    fn ldr32(rt: u32, rn: u32) -> u32 {
        0xb940_0000 | (rn << 5) | rt
    }
    fn stlr32(rt: u32, rn: u32) -> u32 {
        0x889f_fc00 | (rn << 5) | rt
    }
    fn ldar32(rt: u32, rn: u32) -> u32 {
        0x88df_fc00 | (rn << 5) | rt
    }
    const DMB_ISH: u32 = 0xd503_3bbf;

    /// One core's program: four setup instructions, then the round body.
    ///
    /// ```text
    ///   movz x0, #mine        ; setup, executed once
    ///   movz x1, #theirs
    ///   movz x2, #seen
    ///   movz w3, #1
    ///   str/stlr w3, [x0]     ; the body, re-entered every round
    ///   dmb ish               ; only in the DMB variant
    ///   ldr/ldar w4, [x1]
    ///   str  w4, [x2]
    /// ```
    ///
    /// The body is re-entered by moving `PC`, not by looping: a branch would
    /// be a fourth or fifth instruction inside the window under test, and the
    /// setup's registers survive a `set_pc`.
    fn program(who: usize, barrier: Fenced) -> Vec<u32> {
        let (mine, theirs, seen) = (FLAG[who], FLAG[1 - who], SEEN[who]);
        let mut code = vec![
            movz64(0, mine),
            movz64(1, theirs),
            movz64(2, seen),
            movz32(3, 1),
        ];
        match barrier {
            Fenced::Neither => {
                code.push(str32(3, 0));
                code.push(ldr32(4, 1));
            }
            Fenced::AcquireRelease => {
                code.push(stlr32(3, 0));
                code.push(ldar32(4, 1));
            }
            Fenced::Dmb => {
                code.push(str32(3, 0));
                code.push(DMB_ISH);
                code.push(ldr32(4, 1));
            }
        }
        code.push(str32(4, 2));
        code
    }

    /// How many instructions the body is.
    fn body_len(barrier: Fenced) -> usize {
        if barrier == Fenced::Dmb { 4 } else { 3 }
    }

    fn space(barrier: Fenced) -> Arc<AddressSpace> {
        let space = Arc::new(AddressSpace::new("mem", 64));
        space
            .topology()
            .map(Region::ram("ram", Arc::new(RamStore::new(0x4_0000))), 0)
            .expect("256 KiB at zero");
        for (who, at) in ENTRY.iter().enumerate() {
            for (i, word) in program(who, barrier).iter().enumerate() {
                space
                    .write(
                        at + 4 * i as u64,
                        Width::U32,
                        u64::from(*word),
                        MemAttrs::DEFAULT,
                    )
                    .expect("the program lands");
            }
        }
        space
    }

    /// A Cortex-A53 at `entry`, with the four setup instructions already run.
    ///
    /// The MMU is off out of reset, so these are physical addresses and
    /// nothing has to be mapped.
    fn core(space: &Arc<AddressSpace>, entry: u64) -> Cpu {
        let cpu = Cpu::new(Config::cortex_a53().with_reset_vector(entry));
        cpu.attach_space(Arc::clone(space));
        for _ in 0..4 {
            cpu.step();
        }
        cpu
    }

    /// Reset both flags and both slots, and read what the last round left.
    fn round_end(space: &Arc<AddressSpace>) -> [u64; 2] {
        let mut seen = [0u64; 2];
        for who in 0..2 {
            seen[who] = space
                .read(SEEN[who], Width::U32, MemAttrs::DEFAULT)
                .expect("the slot reads back");
            space
                .write(FLAG[who], Width::U32, 0, MemAttrs::DEFAULT)
                .expect("the reset lands");
            space
                .write(SEEN[who], Width::U32, UNSET, MemAttrs::DEFAULT)
                .expect("the reset lands");
        }
        seen
    }

    /// What one guest run produced.
    struct Guest {
        both_zero: usize,
        witnessed: usize,
        never_ran: usize,
    }

    /// The litmus, in AArch64 guest instructions, on two host threads.
    fn run(barrier: Fenced) -> Guest {
        let space = space(barrier);
        let insns = body_len(barrier);
        let both_zero = Arc::new(AtomicU32::new(0));
        let witnessed = Arc::new(AtomicU32::new(0));
        let never_ran = Arc::new(AtomicU32::new(0));
        let gate = Arc::new(Barrier::new(3));
        std::thread::scope(|s| {
            for (who, at) in ENTRY.iter().enumerate() {
                let cpu = core(&space, *at);
                let gate = Arc::clone(&gate);
                s.spawn(move || {
                    let body = ENTRY[who] + BODY;
                    for _ in 0..ROUNDS {
                        gate.wait();
                        cpu.set_pc(body);
                        for _ in 0..insns {
                            cpu.step();
                        }
                        gate.wait();
                    }
                });
            }
            for _ in 0..ROUNDS {
                gate.wait();
                gate.wait();
                let seen = round_end(&space);
                if seen.contains(&UNSET) {
                    never_ran.fetch_add(1, Ordering::Relaxed);
                } else {
                    if seen.iter().all(|v| *v == 0) {
                        both_zero.fetch_add(1, Ordering::Relaxed);
                    }
                    if seen.contains(&0) {
                        witnessed.fetch_add(1, Ordering::Relaxed);
                    }
                }
            }
        });
        Guest {
            both_zero: both_zero.load(Ordering::Relaxed) as usize,
            witnessed: witnessed.load(Ordering::Relaxed) as usize,
            never_ran: never_ran.load(Ordering::Relaxed) as usize,
        }
    }

    /// `STLR` then `LDAR`, end to end: decoded, and executed as host fences.
    ///
    /// `never_ran` is what keeps this honest, for the reason the x86 module
    /// gives: if either instruction stopped decoding the two after it would not
    /// run, the slot would still hold [`UNSET`], and the count would be zero
    /// for entirely the wrong reason.
    #[test]
    fn a_guest_store_release_and_load_acquire() {
        let out = run(Fenced::AcquireRelease);
        println!(
            "a64 guest, STLR/LDAR:  forbidden outcome {} / {ROUNDS} ({} overlapped)",
            out.both_zero, out.witnessed
        );
        assert_eq!(out.never_ran, 0, "a round did not reach the load");
        assert!(out.witnessed > 0, "the two cores must overlap");
        assert_eq!(
            out.both_zero, 0,
            "A64 acquire/release is RCsc: a Store-Release before a Load-Acquire \
             in program order is ordered, and this is the outcome that orders"
        );
    }

    /// `DMB ISH` between an ordinary store and an ordinary load.
    ///
    /// The other half of what `host_fence` serves — `Op::Dmb` is the explicit
    /// barrier, and a guest that uses plain accesses plus a barrier is the far
    /// more common shape than one that uses `STLR`/`LDAR`.
    #[test]
    fn a_guest_dmb_between_the_store_and_the_load() {
        let out = run(Fenced::Dmb);
        println!(
            "a64 guest, DMB ISH:    forbidden outcome {} / {ROUNDS} ({} overlapped)",
            out.both_zero, out.witnessed
        );
        assert_eq!(out.never_ran, 0, "a round did not reach the load");
        assert!(out.witnessed > 0, "the two cores must overlap");
        assert_eq!(
            out.both_zero, 0,
            "the guest executed the instruction that forbids this"
        );
    }

    /// The same programs with neither, so the two counts above are read
    /// against something.
    ///
    /// Printed rather than asserted on every host, including a weakly ordered
    /// one. See the module's note: an interpreted instruction separates the
    /// store from the load, so zero here is an ordinary result and says only
    /// that the emulator's own overhead covered the window on this run.
    #[test]
    fn the_same_programs_with_neither() {
        let out = run(Fenced::Neither);
        println!(
            "a64 guest, STR/LDR:    forbidden outcome {} / {ROUNDS} ({} overlapped)",
            out.both_zero, out.witnessed
        );
        assert_eq!(out.never_ran, 0, "a round did not run to completion");
        assert!(out.witnessed > 0, "the two cores must overlap");
    }
}

// ---------------------------------------------------------------------------
// The same litmus, driven by a whole machine in `parallel`
// ---------------------------------------------------------------------------

/// The litmus tests run against a **machine** rather than against two host
/// threads this file spawned, which is what `ROADMAP.md` §8's gate asks for.
///
/// Everything above builds its own concurrency: `std::thread::scope`, an
/// `AddressSpace` two `Exec`s share, and a referee that resets the flags
/// between rounds. That is a sharp instrument and it stays — it pins both
/// threads in a tight loop and maximises collisions — but it exercises no
/// scheduler, no round rendezvous, no safe point, no realizer and no machine
/// file. The gate is about *SMP emulation*, so the litmus has to be able to
/// say something about a board.
///
/// This module says it. Two fixtures, one per guest architecture —
/// `machines/tests/smp-parallel.machine` (two RV64 harts) and
/// `machines/tests/smp-parallel-a64.machine` (two Neoverse-N1-class cores) —
/// each declaring `threading parallel` in the file, each built with nothing
/// passed in but a worker count, each running a litmus program written in
/// guest instructions.
///
/// # What the mode costs, and what replaces it
///
/// **There is no golden.** `Machine::state_hash` is refused outside a
/// deterministic mode, so nothing here can be compared against a recorded
/// number. What replaces it is `docs/techniques/parallel-execution.md`'s
/// ranked list: an outcome a barrier forbids, and underneath it a witness that
/// the two processors collided at all.
///
/// **There is no referee between rounds.** A host-thread litmus stops both
/// threads after every round and resets the flags; a machine cannot be stopped
/// that finely without spending a whole quantum per litmus round. So the
/// harness is *in the guest*: both processors run a self-refereeing program
/// that paces itself against the other with monotone counters, records its own
/// observation per round into RAM, and parks. Rust reads the record once, at
/// the end, after the machine has stopped. That is how a real litmus runner
/// works too, and it means the machine runs freely — a round begins and ends
/// in the middle of the litmus loop rather than around it, which is the part
/// that tests the scheduler.
///
/// **Nothing may assume the ordering under test.** Each processor writes only
/// its own record and Rust joins the two afterwards, so no processor ever has
/// to observe the other's store to decide an outcome. That matters more than
/// it sounds: a guest-side tally would have had to read a word the other
/// processor had just written, and a stale read there would have manufactured
/// exactly the violation this file exists to look for.
///
/// # The two shapes
///
/// **SB**, the store-buffer test the rest of this file is about: each
/// processor stores to its own flag and then loads the other's, and *both*
/// loads returning a stale value is the outcome `FENCE` and `DMB` exist to
/// forbid. It needs the two processors inside the window at the same instant,
/// so each round is gated on a monotone `ready` counter the other publishes —
/// the tightest rendezvous available without an atomic, and deliberately
/// without one, because an atomic in the harness would order the very thing
/// under test.
///
/// **MP**, message passing: the writer stores `data` then `flag`; the reader
/// spins until `flag` moves and then reads `data`. A stale `data` behind a
/// fresh `flag` is the violation. It is here because it does **not** need a
/// tight rendezvous — the reader is already spinning when the writer stores —
/// so where SB's sensitivity depends on two interpreted instructions landing
/// within nanoseconds of each other, MP's does not. On a machine that
/// difference is the whole ballgame, and the measurements in
/// `docs/techniques/parallel-execution.md` say by how much.
///
/// # What a green run here does and does not prove
///
/// Not a proof of absence. The fenced rows assert zero and that assertion is
/// sound on every host by the language's model, exactly as the host-thread
/// rows above are; what a weakly ordered runner adds is the **unfenced** arm
/// printed beside them. Read the counts, never the tick.
///
/// And read [`Litmus::plain_lost`] before any of them. Every number in this
/// module is worthless in a run where the two processors never overlapped, and
/// that one — plain, non-atomic increments lost to the other processor landing
/// inside them — is how a run says whether it did.
#[cfg(any(feature = "cpu-riscv", feature = "cpu-arm-a64"))]
mod machine {
    use rsemu::core::device::ResetKind;
    use rsemu::core::sched::ThreadingMode;
    use rsemu::core::space::MemAttrs;
    use rsemu::core::value::Width;
    use rsemu::machine::{Machine, catalog};

    /// Litmus rounds one machine run performs.
    ///
    /// Two orders of magnitude below [`super::ROUNDS`], and the instrument is
    /// the reason rather than impatience: a round here is a dozen
    /// *interpreted* guest instructions per processor plus whatever spinning
    /// the two do to stay in step, where a round up there is two host
    /// instructions and a barrier. What that costs in wall time is measured in
    /// `docs/techniques/parallel-execution.md`.
    ///
    /// A tenth again under `debug_assertions`, for the reason
    /// [`super::ROUNDS`] gives: an unoptimised interpreter turns this file
    /// into minutes of `cargo test`.
    pub(crate) const ROUNDS: u32 = if cfg!(debug_assertions) {
        2_000
    } else {
        20_000
    };

    /// Where both fixtures map their shared RAM.
    pub(crate) const RAM: u64 = 0x0010_0000;

    /// The litmus scratch, as offsets into that RAM.
    ///
    /// The scalars are sixty-four bytes apart so no two of them share a cache
    /// line on any host this runs on, and all of them are inside 2 KiB so a
    /// RISC-V `lw`/`sw` twelve-bit signed offset reaches every one from a
    /// single base register.
    pub(crate) mod at {
        /// The two SB flags, one per processor.
        pub(crate) const FLAG: [u64; 2] = [0x000, 0x040];
        /// The monotone round counter each processor publishes so the other
        /// can wait for it.
        pub(crate) const READY: [u64; 2] = [0x100, 0x140];
        /// The collision witness: one word both processors increment with a
        /// plain load, add and store.
        pub(crate) const PLAIN: u64 = 0x200;
        /// Where each processor writes its round count before parking.
        pub(crate) const DONE: [u64; 2] = [0x240, 0x280];
        /// MP's payload, written before the flag.
        pub(crate) const MP_DATA: u64 = 0x2c0;
        /// MP's flag, written after the payload and waited on by the reader.
        pub(crate) const MP_FLAG: u64 = 0x300;
        /// The reader's acknowledgement, which paces the writer.
        pub(crate) const MP_ACK: u64 = 0x340;
        /// MP violations the reader counted: a stale payload behind a fresh
        /// flag. Only the reader writes it.
        pub(crate) const MP_BAD: u64 = 0x380;
        /// The flag value and the payload value of the last MP violation the
        /// reader saw.
        ///
        /// Not decoration: the ping-pong makes the flag value the reader
        /// observes provably equal to its own round number, so a recorded flag
        /// that is *not* that number is a torn load and nothing else. See
        /// `a_torn_flag_load_is_visible_through_a_whole_machine`.
        pub(crate) const MP_SAW_FLAG: u64 = 0x3c0;
        /// The payload that went with it.
        pub(crate) const MP_SAW_DATA: u64 = 0x400;
        /// One byte per round per processor: non-zero if that processor's SB
        /// load was stale.
        ///
        /// A byte rather than a word so that both arrays fit in the fixture's
        /// 64 KiB with room to spare, and an *array* rather than a counter so
        /// that the two processors' observations can be joined **after** the
        /// machine has stopped. Joining them in the guest would mean one
        /// processor reading a word the other had just written, which is the
        /// one thing a memory-model test may not do.
        pub(crate) const RESULT: [u64; 2] = [0x1000, 0x6000];
    }

    /// What sits between the guest's store and the guest's load.
    ///
    /// Named once for both architectures; each backend maps them onto its own
    /// instructions and says which in its own module.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub(crate) enum Barrier {
        /// Nothing at all: the control, and the negative control for the
        /// others.
        None,
        /// The architecture's explicit data barrier — `FENCE` on RISC-V,
        /// `DMB ISH` on A64.
        Fence,
        /// Acquire/release accesses instead of a barrier: `STLR`/`LDAR`.
        ///
        /// A64 only. RISC-V has no acquire/release form of an ordinary load or
        /// store — `Zalasr` is not ratified and this core does not implement
        /// it — so the RISC-V backend does not offer this arm rather than
        /// silently assembling something else.
        AcquireRelease,
    }

    /// What one machine run of one arm produced.
    #[derive(Debug)]
    pub(crate) struct Litmus {
        /// Whether both processors reached their round count and parked.
        ///
        /// This module's `guest::Guest::never_ran`: a run that did
        /// not finish counted a prefix of the rounds it claims, and every
        /// number beside this one has to be read that way.
        pub(crate) finished: bool,
        /// How far each processor got.
        pub(crate) done: [u64; 2],
        /// SB: rounds in which **both** loads were stale — the forbidden
        /// outcome.
        pub(crate) both_stale: u64,
        /// SB: rounds in which at least one was, which is how an SB run says
        /// the two processors were ever inside the window together.
        pub(crate) witness: u64,
        /// MP: rounds in which the reader saw a fresh flag and a stale
        /// payload — the forbidden outcome.
        pub(crate) mp_bad: u64,
        /// The flag and payload values the last MP violation was made of.
        ///
        /// `(flag, payload)`. Both zero when there was none. The flag half is
        /// the diagnosis: the ping-pong pins it to the reader's own round
        /// number, so anything else is a torn load.
        pub(crate) mp_saw: (u64, u64),
        /// Plain, non-atomic increments lost to the other processor.
        ///
        /// The collision witness ranked fourth in
        /// `docs/techniques/parallel-execution.md`, and the number to read
        /// before any of the others: nothing lost is a run in which nothing
        /// here is evidence.
        pub(crate) plain_lost: u64,
        /// How many machine rounds the run took.
        pub(crate) quanta: usize,
    }

    /// A two-pass assembler over fixed-width instruction words.
    ///
    /// Both guest architectures encode every instruction in exactly four
    /// bytes, so a label lands at the same index on both passes whatever the
    /// branch displacements turn out to be: the first pass records where the
    /// labels are and the second emits against them. That is the whole
    /// mechanism, and it exists because the programs below have eight branches
    /// apiece and counting instructions by hand is how a litmus test acquires
    /// a bug that reads as a memory-model finding.
    pub(crate) struct Asm {
        code: Vec<u32>,
        labels: Vec<(&'static str, usize)>,
        second: bool,
    }

    impl Asm {
        /// Mark the current position.
        pub(crate) fn label(&mut self, name: &'static str) {
            if !self.second {
                self.labels.push((name, self.code.len()));
            }
        }

        /// Emit one instruction word.
        pub(crate) fn push(&mut self, word: u32) {
            self.code.push(word);
        }

        /// The byte displacement from the instruction about to be emitted to
        /// `name`.
        ///
        /// Zero on the first pass, where the label may not have been reached
        /// yet. Nothing reads the first pass's code.
        #[must_use]
        pub(crate) fn disp(&self, name: &str) -> i32 {
            let to = self
                .labels
                .iter()
                .find(|(n, _)| *n == name)
                .map_or(0, |(_, i)| *i);
            (to as i32 - self.code.len() as i32) * 4
        }
    }

    /// Run `emit` twice and keep the second pass's words.
    pub(crate) fn assemble(emit: impl Fn(&mut Asm)) -> Vec<u32> {
        let mut first = Asm {
            code: Vec::new(),
            labels: Vec::new(),
            second: false,
        };
        emit(&mut first);
        let mut second = Asm {
            code: Vec::new(),
            labels: first.labels,
            second: true,
        };
        emit(&mut second);
        second.code
    }

    /// Little-endian bytes for a ROM image.
    #[must_use]
    pub(crate) fn rom(words: &[u32]) -> Vec<u8> {
        let mut out = Vec::with_capacity(words.len() * 4);
        for w in words {
            out.extend_from_slice(&w.to_le_bytes());
        }
        out
    }

    /// Build the board, saying **nothing** about threading.
    ///
    /// The mode comes from the file, exactly as `tests/parallel_smp_boards.rs`
    /// asserts it does. `workers` is a property of the run and is the only
    /// thing set here — two of them, because a `parallel` machine whose run
    /// gave it no workers runs its jobs inline and is parallel in name only.
    fn board(name: &str, source: &str, image: Vec<u8>) -> Machine {
        let mut options = catalog::build_options().expect("the catalog agrees with itself");
        options.realize.scheduler.workers = 2;
        options.realize.media.insert("code", image);
        let registry = catalog::registry().expect("a registry");
        match rsemu::machine::build(name, source, &registry, &options) {
            Ok(m) => m,
            Err(e) => panic!("{name} does not realize: {e}"),
        }
    }

    fn word(m: &Machine, off: u64) -> u64 {
        m.space("mem")
            .expect("the fixture's one space")
            .read(RAM + off, Width::U32, MemAttrs::DEFAULT)
            .expect("a mapped word")
    }

    fn byte(m: &Machine, off: u64) -> u64 {
        m.space("mem")
            .expect("the fixture's one space")
            .read(RAM + off, Width::U8, MemAttrs::DEFAULT)
            .expect("a mapped byte")
    }

    /// Build the board, run it until both processors park, and read the
    /// record out of guest RAM.
    ///
    /// Both stopping conditions are bounds rather than timeouts: a test that
    /// hangs is worse than one that fails, and a run that does not finish is
    /// reported through [`Litmus::finished`] rather than by never returning.
    /// Five hundred machine rounds in which not one litmus round completed is
    /// the "this is not making progress" line; two hundred thousand rounds is
    /// the backstop. A healthy run on the author's host takes fifty to
    /// ninety.
    pub(crate) fn run(name: &str, source: &str, image: Vec<u8>, rounds: u32) -> Litmus {
        let mut m = board(name, source, image);
        assert_eq!(
            m.threading_mode(),
            ThreadingMode::Parallel,
            "{name} must select `parallel` in the file; nothing here passes a mode"
        );
        // And the consequence of that, restated where it bites: there is no
        // golden for anything below.
        assert!(
            m.state_hash().is_err(),
            "a parallel state hash is a sample, not a baseline"
        );
        m.reset(ResetKind::Cold);
        let want = u64::from(rounds);
        let mut quanta = 0usize;
        let (mut seen, mut idle) = (0u64, 0usize);
        for _ in 0..200_000 {
            if word(&m, at::DONE[0]) >= want && word(&m, at::DONE[1]) >= want {
                break;
            }
            // Progress, not a round count, is what the give-up is on. How many
            // litmus rounds fit in a machine round is a property of the pool:
            // with two workers both processors run inside every round and it
            // is hundreds, while a pool that runs its jobs inline gets one or
            // two, because each processor spins out its budget waiting at a
            // gate the other cannot pass until it is dispatched. Both are
            // supported configurations and the second is three orders of
            // magnitude slower, so a fixed bound would either fail it or stop
            // being a hang guard. `at::PLAIN` moves once per processor per
            // round in every program here, which makes it the one universal
            // progress signal.
            let now = word(&m, at::PLAIN);
            idle = if now == seen { idle + 1 } else { 0 };
            seen = now;
            if idle >= 500 {
                break;
            }
            m.run_quantum().expect("the machine advances");
            quanta += 1;
        }
        // Two more rounds so the join at each round's end publishes everything
        // both processors wrote. They are parked by now, so nothing read after
        // this can still move.
        m.run_quantum().expect("a round");
        m.run_quantum().expect("a round");
        let done = [word(&m, at::DONE[0]), word(&m, at::DONE[1])];
        let plain = word(&m, at::PLAIN);
        // The join: one byte per round per processor, read after the machine
        // has stopped, so no ordering the guest did not have is assumed.
        let (mut both_stale, mut witness) = (0u64, 0u64);
        for i in 0..u64::from(rounds) {
            let stale = [
                byte(&m, at::RESULT[0] + i) != 0,
                byte(&m, at::RESULT[1] + i) != 0,
            ];
            if stale[0] && stale[1] {
                both_stale += 1;
            }
            if stale[0] || stale[1] {
                witness += 1;
            }
        }
        Litmus {
            finished: done[0] >= want && done[1] >= want,
            done,
            both_stale,
            witness,
            mp_bad: word(&m, at::MP_BAD),
            mp_saw: (word(&m, at::MP_SAW_FLAG), word(&m, at::MP_SAW_DATA)),
            plain_lost: (2 * want).saturating_sub(plain),
            quanta,
        }
    }

    /// The checks every arm of every architecture makes, so no backend can
    /// quietly skip one.
    ///
    /// `finished` first, because a run that stopped short counted a prefix.
    /// Then the collision witness, printed rather than asserted — how often
    /// two processors collide is the host scheduler's business — but shouted
    /// about when it is zero, since that is a run whose zeros mean nothing.
    pub(crate) fn common(label: &str, out: &Litmus) {
        println!(
            "{label}: {} of {} plain increments lost, {} machine rounds",
            out.plain_lost,
            2 * u64::from(ROUNDS),
            out.quanta,
        );
        assert!(
            out.finished,
            "{label}: the run did not finish — processor 0 reached {}, processor 1 reached {}, \
             of {ROUNDS}. Every count beside this one is a prefix.",
            out.done[0], out.done[1]
        );
        if out.plain_lost == 0 {
            println!(
                "warning: {label} lost no plain increments, so the two processors never \
                 collided in this run and nothing it reports is evidence about ordering. \
                 That is a property of this host, not a failure — but a run that never \
                 collides can never observe a reordering either."
            );
        }
        if super::weak_memory_required() {
            // The machine-level analogue of the two required checks at the top
            // of this file, and the only one this module can honestly make.
            //
            // What it is *not* is a requirement that the unfenced arm produce
            // the forbidden outcome. Up there that requirement is sound: two
            // host instructions separate the store from the load, and a host
            // that never reorders across that window is a host whose zeros are
            // vacuous. Down here a whole interpreted guest instruction and the
            // scheduler's per-access accounting sit in the same gap, and the
            // measurements in `docs/techniques/parallel-execution.md` say the
            // window closes on both hosts: zero is the expected reading, so
            // requiring otherwise would gate on how slowly a runner interprets.
            // What can be required is that the two processors collided at all,
            // which is the premise every count here rests on.
            assert!(
                out.plain_lost > 0,
                "RSEMU_WEAK_MEMORY_REQUIRED is set and {label} lost none of its {} \
                 plain increments, so the board's two processors never landed inside \
                 each other. This is not a defect in the emulator: it says this runner \
                 did not overlap them, so nothing this module reports is evidence.",
                2 * u64::from(ROUNDS)
            );
        }
    }

    /// Two RV64 harts of `machines/tests/smp-parallel.machine`, running SB and
    /// MP as guest code.
    ///
    /// The barrier under test is `FENCE`, which `cpu::riscv::exec` retires as
    /// `core::sync::fence(SeqCst)` — the ordering sets are decoded and then
    /// ignored, so `fence rw,rw` and `fence w,w` are the same host instruction
    /// and the programs below still use the architecturally correct one.
    ///
    /// There is no `STLR`/`LDAR` arm. RV has no acquire/release form of an
    /// ordinary load or store — `Zalasr` is not ratified and this core does
    /// not implement it — and an arm that quietly assembled an `AMO` with
    /// `aq`/`rl` set would be testing an atomic rather than an access.
    ///
    /// # Encodings
    ///
    /// *The RISC-V Instruction Set Manual, Volume I: Unprivileged ISA*, RV32I
    /// base for the loads, stores and branches; §2.7 for `FENCE`, whose
    /// predecessor and successor sets are the four bits `PI PO PR PW`, so
    /// `rw,rw` is `0x0330_000F` and `w,w` is `0x0110_000F`; volume II for
    /// `mhartid`. The helpers deliberately mirror
    /// `tests/parallel_smp_boards.rs`'s, which assert a different property on
    /// the same board — the two programs should be legibly the same shape.
    #[cfg(feature = "cpu-riscv")]
    mod rv {
        use super::{Asm, Barrier, Litmus, RAM, ROUNDS, assemble, at, common, rom, run};

        /// The board: two RV64 harts, one crystal, `threading parallel` in the
        /// file.
        const BOARD: &str = include_str!("../machines/tests/smp-parallel.machine");

        const ZERO: u32 = 0;
        const T0: u32 = 5;
        const T1: u32 = 6;
        const T2: u32 = 7;
        const S0: u32 = 8;
        const A0: u32 = 10;
        const A1: u32 = 11;
        const A2: u32 = 12;
        const A3: u32 = 13;
        const A5: u32 = 15;
        const A6: u32 = 16;
        const A7: u32 = 17;

        /// `mhartid`, the machine-mode CSR that says which processor this is.
        const MHARTID: u32 = 0xf14;

        const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
            (((imm as u32) & 0xfff) << 20) | (rs1 << 15) | (rd << 7) | 0x13
        }

        const fn lui(rd: u32, imm: u32) -> u32 {
            ((imm & 0xf_ffff) << 12) | (rd << 7) | 0x37
        }

        const fn load(rd: u32, rs1: u32, off: i32, funct3: u32) -> u32 {
            (((off as u32) & 0xfff) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | 0x03
        }

        const fn lw(rd: u32, rs1: u32, off: i32) -> u32 {
            load(rd, rs1, off, 0b010)
        }

        const fn store(rs2: u32, rs1: u32, off: i32, funct3: u32) -> u32 {
            let imm = (off as u32) & 0xfff;
            ((imm >> 5) << 25)
                | (rs2 << 20)
                | (rs1 << 15)
                | (funct3 << 12)
                | ((imm & 0x1f) << 7)
                | 0x23
        }

        const fn sw(rs2: u32, rs1: u32, off: i32) -> u32 {
            store(rs2, rs1, off, 0b010)
        }

        const fn sb(rs2: u32, rs1: u32, off: i32) -> u32 {
            store(rs2, rs1, off, 0b000)
        }

        const fn branch(rs1: u32, rs2: u32, off: i32, funct3: u32) -> u32 {
            let imm = (off as u32) & 0x1fff;
            (((imm >> 12) & 1) << 31)
                | (((imm >> 5) & 0x3f) << 25)
                | (rs2 << 20)
                | (rs1 << 15)
                | (funct3 << 12)
                | (((imm >> 1) & 0xf) << 8)
                | (((imm >> 11) & 1) << 7)
                | 0x63
        }

        const fn bne(rs1: u32, rs2: u32, off: i32) -> u32 {
            branch(rs1, rs2, off, 0b001)
        }

        const fn blt(rs1: u32, rs2: u32, off: i32) -> u32 {
            branch(rs1, rs2, off, 0b100)
        }

        const fn bge(rs1: u32, rs2: u32, off: i32) -> u32 {
            branch(rs1, rs2, off, 0b101)
        }

        /// `jal x0, off` — an unconditional jump that keeps no return address.
        const fn j(off: i32) -> u32 {
            let imm = (off as u32) & 0x1f_ffff;
            (((imm >> 20) & 1) << 31)
                | (((imm >> 1) & 0x3ff) << 21)
                | (((imm >> 11) & 1) << 20)
                | (((imm >> 12) & 0xff) << 12)
                | 0x6f
        }

        /// `slt rd, rs1, rs2` — one if `rs1 < rs2` signed, zero otherwise.
        ///
        /// The whole of "was that load stale", branch-free, so both harts
        /// execute the identical instruction sequence whatever they observed.
        /// A branch here would make one hart's round a cycle longer than the
        /// other's exactly when they disagreed, which is a feedback loop
        /// between the outcome and the pacing.
        const fn slt(rd: u32, rs1: u32, rs2: u32) -> u32 {
            (rs2 << 20) | (rs1 << 15) | (0b010 << 12) | (rd << 7) | 0x33
        }

        /// `csrr rd, csr`, which is `csrrs rd, csr, x0`.
        const fn csrr(rd: u32, csr: u32) -> u32 {
            (csr << 20) | (0b010 << 12) | (rd << 7) | 0x73
        }

        /// `fence pred, succ`, with the sets in *Unprivileged ISA* §2.7's bit
        /// order.
        const fn fence(pred: u32, succ: u32) -> u32 {
            (pred << 24) | (succ << 20) | 0x0f
        }

        /// The read and write bits of a `FENCE` ordering set.
        const R: u32 = 0b0010;
        const W: u32 = 0b0001;

        /// `jal x0, 0` — where a finished hart parks.
        const PARK: u32 = 0x0000_006f;

        fn li(a: &mut Asm, rd: u32, value: i32) {
            let hi = ((value as u32).wrapping_add(0x800) >> 12) & 0xf_ffff;
            let lo = value.wrapping_sub((hi << 12) as i32);
            if hi == 0 {
                a.push(addi(rd, ZERO, lo));
                return;
            }
            a.push(lui(rd, hi));
            a.push(addi(rd, rd, lo));
        }

        /// The store-buffer litmus, in RV64 instructions, for both harts out of
        /// one ROM.
        ///
        /// ```text
        ///   csrr a7, mhartid
        ///   li   s0, RAM ; li a5, 0 ; li a6, rounds
        ///   bne  a7, x0, hart1
        ///   ...            ; a0=flag[me] a1=flag[them] a2=result[me]
        ///   ...            ; a3=ready[me] t0=ready[them] (per hart)
        /// top:
        ///   addi a5, a5, 1
        ///   sw   a5, 0(a3)         ; ready[me] = i
        /// spin:
        ///   lw   t1, 0(t0)
        ///   blt  t1, a5, spin      ; until ready[them] >= i
        ///   sw   a5, 0(a0)         ; ---- flag[me] = i
        ///   fence rw,rw            ;      the arm under test
        ///   lw   t1, 0(a1)         ; ---- v = flag[them]
        ///   slt  t2, t1, a5        ; stale = v < i
        ///   sb   t2, 0(a2)         ; record[me][i] = stale
        ///   addi a2, a2, 1
        ///   lw   t2, PLAIN(s0)     ; the collision witness, plainly
        ///   addi t2, t2, 1
        ///   sw   t2, PLAIN(s0)
        ///   blt  a5, a6, top
        ///   sw   a5, DONE[me](s0)
        ///   jal  x0, 0
        /// ```
        ///
        /// The gate is `>=` rather than `==` and that is load-bearing twice
        /// over. It cannot deadlock when the two harts are dispatched one
        /// after the other rather than at once — the inline-pool case, which
        /// is every `no_std` host and the no-threads browser build — and it
        /// makes "stale" mean *strictly behind*, so a hart that has raced a
        /// round ahead reads as fresh rather than as a violation.
        fn store_buffer(barrier: Barrier, rounds: u32) -> Vec<u32> {
            assert_ne!(
                barrier,
                Barrier::AcquireRelease,
                "RV has no acquire/release ordinary access; see the module documentation"
            );
            assemble(move |a| {
                a.push(csrr(A7, MHARTID));
                li(a, S0, RAM as i32);
                li(a, A5, 0);
                li(a, A6, rounds as i32);
                let d = a.disp("hart1");
                a.push(bne(A7, ZERO, d));
                for (me, them) in [(0usize, 1usize), (1, 0)] {
                    a.push(addi(A0, S0, at::FLAG[me] as i32));
                    a.push(addi(A1, S0, at::FLAG[them] as i32));
                    li(a, A2, (RAM + at::RESULT[me]) as i32);
                    a.push(addi(A3, S0, at::READY[me] as i32));
                    a.push(addi(T0, S0, at::READY[them] as i32));
                    a.push(addi(A7, S0, at::DONE[me] as i32));
                    if me == 0 {
                        let d = a.disp("top");
                        a.push(j(d));
                        a.label("hart1");
                    }
                }
                a.label("top");
                a.push(addi(A5, A5, 1));
                a.push(sw(A5, A3, 0));
                a.label("spin");
                a.push(lw(T1, T0, 0));
                let d = a.disp("spin");
                a.push(blt(T1, A5, d));
                // ---- the window ----
                a.push(sw(A5, A0, 0));
                if barrier == Barrier::Fence {
                    a.push(fence(R | W, R | W));
                }
                a.push(lw(T1, A1, 0));
                // --------------------
                a.push(slt(T2, T1, A5));
                a.push(sb(T2, A2, 0));
                a.push(addi(A2, A2, 1));
                a.push(lw(T2, S0, at::PLAIN as i32));
                a.push(addi(T2, T2, 1));
                a.push(sw(T2, S0, at::PLAIN as i32));
                let d = a.disp("top");
                a.push(blt(A5, A6, d));
                a.push(sw(A5, A7, 0));
                a.push(PARK);
            })
        }

        /// The message-passing litmus, in RV64 instructions.
        ///
        /// ```text
        ///   hart 0, the writer            hart 1, the reader
        /// wtop:                         rtop:
        ///   addi a5, a5, 1                addi a5, a5, 1
        ///   sw   a5, 0(a0)  ; data=i    rspin:
        ///   fence w,w                     lw   t1, 0(a1)   ; f = flag
        ///   sw   a5, 0(a1)  ; flag=i      blt  t1, a5, rspin
        /// wack:                           fence r,r
        ///   lw   t1, 0(a2)                lw   t2, 0(a0)   ; d = data
        ///   blt  t1, a5, wack             bge  t2, a5, rok ; d >= i is fine
        ///   <plain increment>             <bad++>
        ///   blt  a5, a6, wtop           rok:
        ///   sw   a5, DONE0(s0)            sw   a5, 0(a2)   ; ack = i
        ///   jal  x0, 0                    <plain increment>
        ///                                 blt  a5, a6, rtop
        ///                                 sw   a5, DONE1(s0)
        ///                                 jal  x0, 0
        /// ```
        ///
        /// # Which value the reader compares against, and why it matters
        ///
        /// `sensitive` picks it. The obvious choice is the flag value the
        /// reader **observed** — call it `f` — because `data >= f` is what the
        /// architecture actually promises. The ping-pong makes that choice
        /// equivalent to comparing against the reader's own round number `i`:
        /// the writer cannot reach round `i+1` until the reader has
        /// acknowledged round `i`, so when the spin exits, `f == i`.
        ///
        /// Equivalent, that is, **unless the load of the flag tore**. The
        /// reader spins on `flag` while the writer stores it, `RamStore` has
        /// no single-copy atomicity (see
        /// `docs/techniques/parallel-execution.md`, "what does not work yet"),
        /// and a four-byte load mixing the old word with the new can return a
        /// value *larger* than either — `0x0ff` under `0x100` yields `0x1ff`.
        /// Compared against `f` that reads as a message-passing violation;
        /// compared against `i` it does not.
        ///
        /// So the rows that gate use `i`, which cannot manufacture a violation
        /// out of a torn load and loses nothing by the argument above, and
        /// `a_torn_flag_load_is_visible_through_a_whole_machine` uses `f` and
        /// exists to report the tear rather than to gate on it.
        ///
        /// `fence w,w` on the writer and `fence r,r` on the reader are the
        /// architecturally correct sets. This core retires both as a full host
        /// fence, so the distinction is documentation today and a regression
        /// net when it stops being one.
        fn message_passing(barrier: Barrier, rounds: u32, sensitive: bool) -> Vec<u32> {
            assert_ne!(
                barrier,
                Barrier::AcquireRelease,
                "RV has no acquire/release ordinary access; see the module documentation"
            );
            assemble(move |a| {
                a.push(csrr(A7, MHARTID));
                li(a, S0, RAM as i32);
                li(a, A5, 0);
                li(a, A6, rounds as i32);
                let d = a.disp("reader");
                a.push(bne(A7, ZERO, d));

                // ---- the writer ----
                a.push(addi(A0, S0, at::MP_DATA as i32));
                a.push(addi(A1, S0, at::MP_FLAG as i32));
                a.push(addi(A2, S0, at::MP_ACK as i32));
                a.push(addi(A7, S0, at::DONE[0] as i32));
                a.label("wtop");
                a.push(addi(A5, A5, 1));
                a.push(sw(A5, A0, 0));
                if barrier == Barrier::Fence {
                    a.push(fence(W, W));
                }
                a.push(sw(A5, A1, 0));
                a.label("wack");
                a.push(lw(T1, A2, 0));
                let d = a.disp("wack");
                a.push(blt(T1, A5, d));
                a.push(lw(T2, S0, at::PLAIN as i32));
                a.push(addi(T2, T2, 1));
                a.push(sw(T2, S0, at::PLAIN as i32));
                let d = a.disp("wtop");
                a.push(blt(A5, A6, d));
                a.push(sw(A5, A7, 0));
                a.push(PARK);

                // ---- the reader ----
                a.label("reader");
                a.push(addi(A0, S0, at::MP_DATA as i32));
                a.push(addi(A1, S0, at::MP_FLAG as i32));
                a.push(addi(A2, S0, at::MP_ACK as i32));
                a.push(addi(A3, S0, at::MP_BAD as i32));
                a.push(addi(A7, S0, at::DONE[1] as i32));
                a.label("rtop");
                a.push(addi(A5, A5, 1));
                a.label("rspin");
                a.push(lw(T1, A1, 0));
                let d = a.disp("rspin");
                a.push(blt(T1, A5, d));
                if barrier == Barrier::Fence {
                    a.push(fence(R, R));
                }
                a.push(lw(T2, A0, 0));
                let d = a.disp("rok");
                a.push(bge(T2, if sensitive { T1 } else { A5 }, d));
                // The violation path, which also records what it was made of.
                // Two stores nothing else reads, off the measured path.
                a.push(sw(T1, S0, at::MP_SAW_FLAG as i32));
                a.push(sw(T2, S0, at::MP_SAW_DATA as i32));
                a.push(lw(T2, A3, 0));
                a.push(addi(T2, T2, 1));
                a.push(sw(T2, A3, 0));
                a.label("rok");
                a.push(sw(A5, A2, 0));
                a.push(lw(T2, S0, at::PLAIN as i32));
                a.push(addi(T2, T2, 1));
                a.push(sw(T2, S0, at::PLAIN as i32));
                let d = a.disp("rtop");
                a.push(blt(A5, A6, d));
                a.push(sw(A5, A7, 0));
                a.push(PARK);
            })
        }

        fn sb_run(barrier: Barrier) -> Litmus {
            run(
                "smp-parallel.machine",
                BOARD,
                rom(&store_buffer(barrier, ROUNDS)),
                ROUNDS,
            )
        }

        fn mp_run(barrier: Barrier) -> Litmus {
            run(
                "smp-parallel.machine",
                BOARD,
                rom(&message_passing(barrier, ROUNDS, false)),
                ROUNDS,
            )
        }

        /// **SB with `FENCE`, on a machine.** Zero is asserted.
        ///
        /// Sound on every host and therefore not, on its own, evidence about
        /// the fence: `the_same_sb_without_one` is the arm that gives it
        /// something to be read against.
        #[test]
        fn a_guest_fence_between_the_store_and_the_load_on_a_machine() {
            let out = sb_run(Barrier::Fence);
            common("machine rv, SB, FENCE   ", &out);
            println!(
                "machine rv, SB, FENCE   : forbidden outcome {} / {ROUNDS} ({} rounds overlapped)",
                out.both_stale, out.witness
            );
            assert_eq!(
                out.both_stale, 0,
                "a full fence on both harts cannot permit this"
            );
        }

        /// The same board and program with the `FENCE` deleted, so the row
        /// above is read against something.
        ///
        /// Printed, never asserted, on every host including a weakly ordered
        /// one — for the reason `guest_a64` gives about its own
        /// unfenced arm and one more that is specific to a machine: between the
        /// store and the load sits a whole *interpreted* guest instruction plus
        /// the scheduler's per-access cycle accounting, which is a far wider
        /// separation than the two host instructions the top of this file uses.
        /// A zero here is the expected reading, not a held gate.
        #[test]
        fn the_same_sb_without_one() {
            let out = sb_run(Barrier::None);
            common("machine rv, SB, neither ", &out);
            println!(
                "machine rv, SB, neither : forbidden outcome {} / {ROUNDS} ({} rounds overlapped)",
                out.both_stale, out.witness
            );
        }

        /// **MP with `FENCE`, on a machine.** Zero is asserted.
        ///
        /// The row SB cannot be on a machine. The reader is already spinning
        /// when the writer stores, so the window is the writer's own two
        /// stores rather than a rendezvous between two interpreters — nothing
        /// in the harness has to land two guest instructions within
        /// nanoseconds of each other for this to be able to fail.
        #[test]
        fn a_guest_fence_between_the_payload_and_the_flag() {
            let out = mp_run(Barrier::Fence);
            common("machine rv, MP, FENCE   ", &out);
            println!(
                "machine rv, MP, FENCE   : forbidden outcome {} / {ROUNDS}",
                out.mp_bad
            );
            assert_eq!(
                out.mp_bad, 0,
                "the writer fenced between the payload and the flag, and the reader \
                 between the flag and the payload; a stale payload behind a fresh flag \
                 is what that pair forbids"
            );
        }

        /// The same, with both fences deleted: the negative control for the
        /// row above.
        #[test]
        fn the_same_mp_without_one() {
            let out = mp_run(Barrier::None);
            common("machine rv, MP, neither ", &out);
            println!(
                "machine rv, MP, neither : forbidden outcome {} / {ROUNDS}",
                out.mp_bad
            );
        }

        /// **A defect, reproduced through a whole machine and not fixed here.**
        /// `RamStore` has no single-copy atomicity, and a guest spinning on a
        /// word another processor is storing can load a value that was never
        /// in memory.
        ///
        /// Not a new defect: `docs/techniques/parallel-execution.md` lists it
        /// first under "what does not work yet" and
        /// `tests/smp_single_copy_atomicity.rs` measures it — a `Vec<AtomicU8>`
        /// accessed by a byte loop, so a four-byte load racing a four-byte
        /// store returns a mixture of the two words, which *Intel SDM* vol. 3
        /// §9.1.1, ARM DDI 0487 B2.2.1 and *RISC-V Unprivileged ISA* §1.4 each
        /// forbid outright. What is new is the level: that file spawns two host
        /// threads over one `AddressSpace`, and this one is a board, in
        /// `parallel`, running a guest program a kernel would recognise.
        ///
        /// It is worth having at this level because a torn load here is not a
        /// curiosity. It is indistinguishable, to the guest, from the
        /// message-passing violation a missing barrier would cause — and it
        /// found this file first: the `FENCE` row asserted zero, saw one
        /// violation in twenty thousand rounds on an x86-64 host where store
        /// ordering cannot be the cause, and the recorded pair below was what
        /// said which of the two it was.
        ///
        /// `#[ignore]` because it is a search for a rare event rather than a
        /// gate, and because reporting a known defect must not turn the suite
        /// red. Run it with `--ignored`; it fails only if the run did not
        /// complete, or if the pair it recorded was an ordering violation
        /// rather than a tear.
        ///
        /// **Measured**, x86-64 Linux, release: 11 tears in two million RISC-V
        /// rounds and 13 in two million A64 ones — twenty-five invocations of
        /// each, four runs of twenty thousand apiece. Every single recorded
        /// pair was a byte-carry boundary, which is what makes the diagnosis
        /// certain rather than plausible: `flag=8959 payload=8704` is `0x22ff`
        /// under `0x2200`, `flag=4095 payload=3840` is `0x0fff` under
        /// `0x0f00`, `flag=19967 payload=19712` is `0x4dff` under `0x4d00`.
        /// Each is the low byte of the *previous* round's value wearing the
        /// high bytes of this one — a value that was never in memory. Only one
        /// round in 256 crosses such a boundary, so the rate among rounds that
        /// could possibly tear is about 1 500 per million.
        ///
        /// ```text
        /// cargo test --release --all-features --test memory_model_litmus \
        ///     -- --ignored --nocapture a_torn_flag_load
        /// ```
        #[test]
        #[ignore = "a search for a rare known defect, not a gate; see the doc comment"]
        fn a_torn_flag_load_is_visible_through_a_whole_machine() {
            // Fenced, so that ordering cannot be an explanation for anything
            // seen: on x86-64 a store-store reordering is forbidden outright,
            // and the guest asked for one anyway.
            let mut tears = 0u64;
            for attempt in 1..=super::super::ATTEMPTS {
                let out = run(
                    "smp-parallel.machine",
                    BOARD,
                    rom(&message_passing(Barrier::Fence, ROUNDS, true)),
                    ROUNDS,
                );
                common("machine rv, MP, torn    ", &out);
                let (flag, data) = out.mp_saw;
                println!(
                    "machine rv, MP, torn    : {} apparent violations / {ROUNDS} \
                     (attempt {attempt}/{}); last was flag={flag} payload={data}",
                    out.mp_bad,
                    super::super::ATTEMPTS,
                );
                if out.mp_bad > 0 {
                    // The ping-pong pins the observed flag to the reader's own
                    // round number, and the payload is written before the flag
                    // with a fence between: `payload + 1 == flag` would be an
                    // ordering violation, and anything else is a torn load.
                    assert_ne!(
                        flag,
                        data + 1,
                        "the flag was exactly one ahead of the payload, which is a \
                         genuine store-store reordering rather than a torn load. On \
                         an x86-64 host that cannot happen; on a weakly ordered one it \
                         would be a defect in `Op::Fence`, not in `RamStore`."
                    );
                    tears += out.mp_bad;
                }
            }
            println!(
                "machine rv: {tears} torn flag loads over {} runs of {ROUNDS} rounds",
                super::super::ATTEMPTS
            );
        }
    }

    /// Two Neoverse-N1-class cores of `machines/tests/smp-parallel-a64.machine`,
    /// running SB and MP as guest code.
    ///
    /// The A64 leg of the gate, and the one the `aarch64 (weak memory)` CI job
    /// exists for. Three arms, the same three
    /// `guest_a64` has: `DMB ISH`, `STLR`/`LDAR`, and neither.
    /// What is different is what they run on — a board, in `parallel`, with
    /// the scheduler dispatching both cores into every round.
    ///
    /// # Encodings
    ///
    /// *Arm Architecture Reference Manual for A-profile*, DDI 0487: C6.2 for
    /// `MOVZ`, `MOVK`, `ADD` (immediate), `SUBS` (the `CMP` alias), `CSINC`
    /// (the `CSET` alias), `LDR`/`STR`/`STRB` (unsigned offset),
    /// `LDAR`/`STLR`, `B`, `B.cond` and `CBNZ`; C6.2.79 for `DMB`, whose `CRm`
    /// of `0b1011` is the inner-shareable domain, full system — B2.3.7's
    /// mapping for a sequentially consistent access on this architecture.
    /// D17.2.90 for `MPIDR_EL1`, whose Aff0 is the low byte and is the only
    /// thing the fixture varies between the two cores.
    #[cfg(feature = "cpu-arm-a64")]
    mod a64 {
        use super::{Asm, Barrier, Litmus, RAM, ROUNDS, assemble, at, common, rom, run};

        /// The board: two A64 cores, one crystal, `threading parallel` in the
        /// file.
        const BOARD: &str = include_str!("../machines/tests/smp-parallel-a64.machine");

        /// The zero register, in the position an encoding's `Rt`/`Rd` field
        /// takes it.
        const ZR: u32 = 31;

        // Condition codes, DDI 0487 C1.2.4.
        const GE: u32 = 0b1010;
        const LT: u32 = 0b1011;

        /// `MOVZ Wd, #imm16` — and, with `hw` set, `MOVZ Xd, #imm16, LSL #16`.
        const fn movz(rd: u32, imm: u32, hw: u32, sf: u32) -> u32 {
            (sf << 31) | 0x5280_0000 | (hw << 21) | ((imm & 0xffff) << 5) | rd
        }

        /// `MOVK Wd, #imm16, LSL #16`.
        const fn movk16(rd: u32, imm: u32) -> u32 {
            0x7280_0000 | (1 << 21) | ((imm & 0xffff) << 5) | rd
        }

        /// `ADD Xd, Xn, #imm12` and, with `sh`, `#imm12, LSL #12`.
        ///
        /// The shifted form is what reaches the two result arrays: they are at
        /// 0x1000 and 0x6000 from the base, and an unshifted `imm12` stops at
        /// 0xfff.
        const fn add_imm64(rd: u32, rn: u32, imm: u32, sh: u32) -> u32 {
            0x9100_0000 | (sh << 22) | ((imm & 0xfff) << 10) | (rn << 5) | rd
        }

        /// `ADD Wd, Wn, #imm12`.
        const fn add_imm32(rd: u32, rn: u32, imm: u32) -> u32 {
            0x1100_0000 | ((imm & 0xfff) << 10) | (rn << 5) | rd
        }

        /// `AND Xd, Xn, Xm`.
        const fn and64(rd: u32, rn: u32, rm: u32) -> u32 {
            0x8a00_0000 | (rm << 16) | (rn << 5) | rd
        }

        /// `CMP Wn, Wm`, which is `SUBS WZR, Wn, Wm`.
        const fn cmp32(rn: u32, rm: u32) -> u32 {
            0x6b00_0000 | (rm << 16) | (rn << 5) | ZR
        }

        /// `CSET Wd, cond`, which is `CSINC Wd, WZR, WZR, invert(cond)`.
        ///
        /// `inverted` is what goes in the encoding's `cond` field, so a caller
        /// asking for "one if less-than" passes [`GE`]. Written that way round
        /// deliberately: the alias inverts and a helper that inverted again
        /// would be a silent off-by-one-condition.
        const fn cset32(rd: u32, inverted: u32) -> u32 {
            0x1a80_0400 | (ZR << 16) | (inverted << 12) | (ZR << 5) | rd
        }

        /// `MRS Xt, MPIDR_EL1` — op0 3, op1 0, CRn 0, CRm 0, op2 5.
        const fn mrs_mpidr(rt: u32) -> u32 {
            0xd538_00a0 | rt
        }

        /// `STR Wt, [Xn]` and `LDR Wt, [Xn]`, unsigned offset zero.
        const fn str32(rt: u32, rn: u32) -> u32 {
            0xb900_0000 | (rn << 5) | rt
        }
        const fn ldr32(rt: u32, rn: u32) -> u32 {
            0xb940_0000 | (rn << 5) | rt
        }
        /// `STRB Wt, [Xn]`.
        const fn strb(rt: u32, rn: u32) -> u32 {
            0x3900_0000 | (rn << 5) | rt
        }
        /// `STLR Wt, [Xn]` and `LDAR Wt, [Xn]`.
        const fn stlr32(rt: u32, rn: u32) -> u32 {
            0x889f_fc00 | (rn << 5) | rt
        }
        const fn ldar32(rt: u32, rn: u32) -> u32 {
            0x88df_fc00 | (rn << 5) | rt
        }

        /// `B label`.
        const fn b(off: i32) -> u32 {
            0x1400_0000 | (((off >> 2) as u32) & 0x03ff_ffff)
        }
        /// `B.cond label`.
        const fn bcond(cond: u32, off: i32) -> u32 {
            0x5400_0000 | ((((off >> 2) as u32) & 0x7ffff) << 5) | cond
        }
        /// `CBNZ Xt, label`.
        const fn cbnz64(rt: u32, off: i32) -> u32 {
            0xb500_0000 | ((((off >> 2) as u32) & 0x7ffff) << 5) | rt
        }

        /// `DMB ISH`.
        const DMB_ISH: u32 = 0xd503_3bbf;
        /// `B .` — where a finished core parks.
        const PARK: u32 = 0x1400_0000;

        /// Load a 32-bit constant into `Wd`.
        fn li32(a: &mut Asm, rd: u32, value: u32) {
            a.push(movz(rd, value & 0xffff, 0, 0));
            if value >> 16 != 0 {
                a.push(movk16(rd, value >> 16));
            }
        }

        /// `ADD Xd, X28, #off`, choosing the shifted form when it is needed.
        ///
        /// Every offset in [`at`] is either below 0x1000 or an exact multiple
        /// of 0x1000, which is what makes this total rather than a hazard.
        fn base_plus(a: &mut Asm, rd: u32, off: u64) {
            assert!(
                off < 0x1000 || off.is_multiple_of(0x1000),
                "offset {off:#x} is not reachable"
            );
            if off < 0x1000 {
                a.push(add_imm64(rd, BASE, off as u32, 0));
            } else {
                a.push(add_imm64(rd, BASE, (off >> 12) as u32, 1));
            }
        }

        // Registers. X28 holds the RAM base; X8 the collision-witness word;
        // W15 the round number and W16 the round count; W5, W6 and W17 are
        // scratch. Everything else is a pointer set up once per core.
        const BASE: u32 = 28;
        const PLAIN_P: u32 = 8;
        const I: u32 = 15;
        const N: u32 = 16;
        const V: u32 = 5;
        const TMP: u32 = 6;
        const ACC: u32 = 17;
        const ID: u32 = 9;
        const ONE: u32 = 10;

        /// The setup both programs share: which core am I, where is RAM.
        fn preamble(a: &mut Asm, rounds: u32, other: &'static str) {
            a.push(mrs_mpidr(ID));
            a.push(movz(ONE, 1, 0, 1));
            a.push(and64(ID, ID, ONE));
            a.push(movz(BASE, (RAM >> 16) as u32, 1, 1));
            base_plus(a, PLAIN_P, at::PLAIN);
            a.push(movz(I, 0, 0, 0));
            li32(a, N, rounds);
            let d = a.disp(other);
            a.push(cbnz64(ID, d));
        }

        /// The collision witness: three instructions on a word the
        /// architecture promises nothing about.
        fn plain_increment(a: &mut Asm) {
            a.push(ldr32(ACC, PLAIN_P));
            a.push(add_imm32(ACC, ACC, 1));
            a.push(str32(ACC, PLAIN_P));
        }

        /// The store-buffer litmus, in A64 instructions, for both cores out of
        /// one ROM.
        ///
        /// ```text
        ///   mrs  x9, mpidr_el1 ; and x9, x9, #1 ; movz x28, #0x10, lsl #16
        ///   cbnz x9, core1
        ///   ...                    ; x0=flag[me] x1=flag[them] x2=record[me]
        ///   ...                    ; x3=ready[me] x4=ready[them] x7=done[me]
        /// top:
        ///   add   w15, w15, #1
        ///   str   w15, [x3]        ; ready[me] = i
        /// spin:
        ///   ldr   w5, [x4]
        ///   cmp   w5, w15
        ///   b.lt  spin             ; until ready[them] >= i
        ///   stlr? w15, [x0]        ; ---- flag[me] = i
        ///   dmb   ish?             ;      the arm under test
        ///   ldar? w5, [x1]         ; ---- v = flag[them]
        ///   cmp   w5, w15
        ///   cset  w6, lt           ; stale = v < i, branch-free
        ///   strb  w6, [x2]
        ///   add   x2, x2, #1
        ///   <plain increment>
        ///   cmp   w15, w16
        ///   b.lt  top
        ///   str   w15, [x7]
        ///   b     .
        /// ```
        ///
        /// `CSET` rather than a branch so both cores execute the identical
        /// sequence whatever they observed: a branch would make one core's
        /// round a cycle longer than the other's exactly when they disagreed,
        /// which is a feedback loop between the outcome and the pacing.
        fn store_buffer(barrier: Barrier, rounds: u32) -> Vec<u32> {
            assemble(move |a| {
                preamble(a, rounds, "core1");
                for me in 0..2usize {
                    let them = 1 - me;
                    base_plus(a, 0, at::FLAG[me]);
                    base_plus(a, 1, at::FLAG[them]);
                    base_plus(a, 2, at::RESULT[me]);
                    base_plus(a, 3, at::READY[me]);
                    base_plus(a, 4, at::READY[them]);
                    base_plus(a, 7, at::DONE[me]);
                    if me == 0 {
                        let d = a.disp("top");
                        a.push(b(d));
                        a.label("core1");
                    }
                }
                a.label("top");
                a.push(add_imm32(I, I, 1));
                a.push(str32(I, 3));
                a.label("spin");
                a.push(ldr32(V, 4));
                a.push(cmp32(V, I));
                let d = a.disp("spin");
                a.push(bcond(LT, d));
                // ---- the window ----
                a.push(if barrier == Barrier::AcquireRelease {
                    stlr32(I, 0)
                } else {
                    str32(I, 0)
                });
                if barrier == Barrier::Fence {
                    a.push(DMB_ISH);
                }
                a.push(if barrier == Barrier::AcquireRelease {
                    ldar32(V, 1)
                } else {
                    ldr32(V, 1)
                });
                // --------------------
                a.push(cmp32(V, I));
                a.push(cset32(TMP, GE));
                a.push(strb(TMP, 2));
                a.push(add_imm64(2, 2, 1, 0));
                plain_increment(a);
                a.push(cmp32(I, N));
                let d = a.disp("top");
                a.push(bcond(LT, d));
                a.push(str32(I, 7));
                a.push(PARK);
            })
        }

        /// The message-passing litmus, in A64 instructions.
        ///
        /// The writer stores the payload, then the flag; the reader spins on
        /// the flag and then reads the payload, and a payload behind the flag
        /// value it observed is the violation. `STLR` on the flag store and
        /// `LDAR` on the flag load is the canonical release/acquire form of
        /// it; `DMB ISH` on both sides is the barrier form.
        ///
        /// `sensitive` picks what the reader compares the payload against —
        /// the flag value it observed, or its own round number. The RISC-V
        /// module's `message_passing` argues at length why those are the same
        /// thing except when the flag load tore, and why the rows that gate
        /// use the round number.
        fn message_passing(barrier: Barrier, rounds: u32, sensitive: bool) -> Vec<u32> {
            assemble(move |a| {
                preamble(a, rounds, "reader");

                // ---- the writer ----
                base_plus(a, 0, at::MP_DATA);
                base_plus(a, 1, at::MP_FLAG);
                base_plus(a, 2, at::MP_ACK);
                base_plus(a, 7, at::DONE[0]);
                a.label("wtop");
                a.push(add_imm32(I, I, 1));
                a.push(str32(I, 0));
                if barrier == Barrier::Fence {
                    a.push(DMB_ISH);
                }
                a.push(if barrier == Barrier::AcquireRelease {
                    stlr32(I, 1)
                } else {
                    str32(I, 1)
                });
                a.label("wack");
                a.push(ldr32(V, 2));
                a.push(cmp32(V, I));
                let d = a.disp("wack");
                a.push(bcond(LT, d));
                plain_increment(a);
                a.push(cmp32(I, N));
                let d = a.disp("wtop");
                a.push(bcond(LT, d));
                a.push(str32(I, 7));
                a.push(PARK);

                // ---- the reader ----
                a.label("reader");
                base_plus(a, 0, at::MP_DATA);
                base_plus(a, 1, at::MP_FLAG);
                base_plus(a, 2, at::MP_ACK);
                base_plus(a, 3, at::MP_BAD);
                base_plus(a, 11, at::MP_SAW_FLAG);
                base_plus(a, 12, at::MP_SAW_DATA);
                base_plus(a, 7, at::DONE[1]);
                a.label("rtop");
                a.push(add_imm32(I, I, 1));
                a.label("rspin");
                a.push(if barrier == Barrier::AcquireRelease {
                    ldar32(V, 1)
                } else {
                    ldr32(V, 1)
                });
                a.push(cmp32(V, I));
                let d = a.disp("rspin");
                a.push(bcond(LT, d));
                if barrier == Barrier::Fence {
                    a.push(DMB_ISH);
                }
                a.push(ldr32(TMP, 0));
                a.push(cmp32(TMP, if sensitive { V } else { I }));
                let d = a.disp("rok");
                a.push(bcond(GE, d));
                // The violation path, which also records what it was made of.
                a.push(str32(V, 11));
                a.push(str32(TMP, 12));
                a.push(ldr32(ACC, 3));
                a.push(add_imm32(ACC, ACC, 1));
                a.push(str32(ACC, 3));
                a.label("rok");
                a.push(str32(I, 2));
                plain_increment(a);
                a.push(cmp32(I, N));
                let d = a.disp("rtop");
                a.push(bcond(LT, d));
                a.push(str32(I, 7));
                a.push(PARK);
            })
        }

        fn sb_run(barrier: Barrier) -> Litmus {
            run(
                "smp-parallel-a64.machine",
                BOARD,
                rom(&store_buffer(barrier, ROUNDS)),
                ROUNDS,
            )
        }

        fn mp_run(barrier: Barrier) -> Litmus {
            run(
                "smp-parallel-a64.machine",
                BOARD,
                rom(&message_passing(barrier, ROUNDS, false)),
                ROUNDS,
            )
        }

        /// **SB with `DMB ISH`, on a machine.** Zero is asserted.
        #[test]
        fn a_guest_dmb_between_the_store_and_the_load_on_a_machine() {
            let out = sb_run(Barrier::Fence);
            common("machine a64, SB, DMB    ", &out);
            println!(
                "machine a64, SB, DMB    : forbidden outcome {} / {ROUNDS} ({} rounds overlapped)",
                out.both_stale, out.witness
            );
            assert_eq!(
                out.both_stale, 0,
                "the guest executed the instruction that forbids this"
            );
        }

        /// **SB with `STLR`/`LDAR`, on a machine.** Zero is asserted.
        ///
        /// A64 acquire/release is RCsc (DDI 0487 B2.3), so a Store-Release
        /// before a Load-Acquire in program order is ordered — which is
        /// precisely this outcome.
        #[test]
        fn a_guest_store_release_and_load_acquire_on_a_machine() {
            let out = sb_run(Barrier::AcquireRelease);
            common("machine a64, SB, STLR   ", &out);
            println!(
                "machine a64, SB, STLR   : forbidden outcome {} / {ROUNDS} ({} rounds overlapped)",
                out.both_stale, out.witness
            );
            assert_eq!(
                out.both_stale, 0,
                "A64 acquire/release is RCsc: a Store-Release before a Load-Acquire in \
                 program order is ordered, and this is the outcome that orders"
            );
        }

        /// The same programs with neither, so the two rows above are read
        /// against something. Printed on every host.
        #[test]
        fn the_same_sb_with_neither() {
            let out = sb_run(Barrier::None);
            common("machine a64, SB, neither", &out);
            println!(
                "machine a64, SB, neither: forbidden outcome {} / {ROUNDS} ({} rounds overlapped)",
                out.both_stale, out.witness
            );
        }

        /// **MP with `DMB ISH`, on a machine.** Zero is asserted.
        #[test]
        fn a_guest_dmb_between_the_payload_and_the_flag() {
            let out = mp_run(Barrier::Fence);
            common("machine a64, MP, DMB    ", &out);
            println!(
                "machine a64, MP, DMB    : forbidden outcome {} / {ROUNDS}",
                out.mp_bad
            );
            assert_eq!(
                out.mp_bad, 0,
                "a stale payload behind a fresh flag is what DMB forbids"
            );
        }

        /// **MP with `STLR`/`LDAR`, on a machine.** Zero is asserted.
        ///
        /// The canonical release/acquire message pass, and the row that most
        /// nearly resembles what a real SMP guest kernel does when it publishes
        /// a structure and then a pointer to it.
        #[test]
        fn a_guest_store_release_publishes_the_payload() {
            let out = mp_run(Barrier::AcquireRelease);
            common("machine a64, MP, STLR   ", &out);
            println!(
                "machine a64, MP, STLR   : forbidden outcome {} / {ROUNDS}",
                out.mp_bad
            );
            assert_eq!(
                out.mp_bad, 0,
                "a Load-Acquire that observes a Store-Release observes everything \
                 sequenced before it"
            );
        }

        /// The same with neither: the negative control for both MP rows.
        #[test]
        fn the_same_mp_with_neither() {
            let out = mp_run(Barrier::None);
            common("machine a64, MP, neither", &out);
            println!(
                "machine a64, MP, neither: forbidden outcome {} / {ROUNDS}",
                out.mp_bad
            );
        }

        /// The A64 half of the RISC-V module's
        /// `a_torn_flag_load_is_visible_through_a_whole_machine`, and it is
        /// the same defect: `RamStore` is architecture-independent, so a guest
        /// spinning on a word another processor is storing can load a value
        /// that was never in memory whichever core is doing the spinning.
        ///
        /// `#[ignore]` for the same two reasons — it is a search for a rare
        /// event, and reporting a known defect must not turn the suite red.
        #[test]
        #[ignore = "a search for a rare known defect, not a gate; see the doc comment"]
        fn a_torn_flag_load_is_visible_through_a_whole_machine() {
            let mut tears = 0u64;
            for attempt in 1..=super::super::ATTEMPTS {
                let out = run(
                    "smp-parallel-a64.machine",
                    BOARD,
                    rom(&message_passing(Barrier::Fence, ROUNDS, true)),
                    ROUNDS,
                );
                common("machine a64, MP, torn   ", &out);
                let (flag, data) = out.mp_saw;
                println!(
                    "machine a64, MP, torn   : {} apparent violations / {ROUNDS} \
                     (attempt {attempt}/{}); last was flag={flag} payload={data}",
                    out.mp_bad,
                    super::super::ATTEMPTS,
                );
                if out.mp_bad > 0 {
                    assert_ne!(
                        flag,
                        data + 1,
                        "the flag was exactly one ahead of the payload, which is a \
                         genuine store-store reordering rather than a torn load — a \
                         defect in `Op::Dmb`, not in `RamStore`"
                    );
                    tears += out.mp_bad;
                }
            }
            println!(
                "machine a64: {tears} torn flag loads over {} runs of {ROUNDS} rounds",
                super::super::ATTEMPTS
            );
        }
    }
}
