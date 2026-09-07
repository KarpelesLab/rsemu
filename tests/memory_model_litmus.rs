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
//! It is opt-in (`--threading parallel`) and no machine file selects it.
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
//! | nothing — two bare relaxed `AtomicU8`s | 7 to 751, run to run |
//! | `core::sync::fence(SeqCst)` — what `IrHost::fence` now emits | **0** |
//! | a relaxed `fetch_or` on an unrelated word | **0** |
//! | `RamStore::write_at` then `read_at` — the emulator's own guest RAM | **0** |
//! | two `cpu::x86` cores, `mov [X],1` then `mov eax,[Y]` | **0** |
//!
//! The third row is the finding, and it corrects the premise the other rows
//! were written to test. **`RamStore` already contains a barrier on this host,
//! by accident.** Every one of its writes ends in
//! [`mark_dirty`](rsemu::core::space::RamStore::mark_dirty), which sets a bit
//! with `AtomicU64::fetch_or` — a *relaxed* atomic read-modify-write, which on
//! x86-64 is a `lock or`, and a locked instruction is a full barrier (*Intel
//! SDM* volume 3 §9.2.5). So a guest store to RAM is followed by a store-buffer
//! drain whether anybody wanted one or not, on the interpreter and in a
//! translated block alike — `jit::Tlb::note_fast_store` marks the same bitmap
//! after an inlined store.
//!
//! That accident is worth exactly as much as an accident: it is x86-only
//! (`fetch_or(Relaxed)` on AArch64 is `ldsetr`, which orders nothing), it
//! covers only the store-then-load case, it does nothing for a barrier between
//! two *loads* or between two *loads and a store*, and it would disappear the
//! day somebody batched the dirty bitmap or wrote it with a plain `store`. The
//! fence is what makes the guarantee the guest asked for a guarantee.
//!
//! The fifth row is the same statement about the emulator's *cost*: a guest
//! store and the guest load after it are separated by a whole interpreted
//! instruction, and `tests/memory_model_costs.rs` puts the host's window at
//! about forty nanoseconds. So even without the dirty bit the interpreter's
//! own overhead would close it. Neither of those is a reason to keep the guest
//! barrier a no-op — they are reasons the omission has cost nothing *so far*,
//! on *one* host.
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
    /// [`RamStore::mark_dirty`] does after every write, reproduced on its own
    /// so that the level below can be attributed to it rather than to luck.
    DirtyBit,
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

/// **The finding.** `RamStore`'s dirty bitmap is already a barrier on a host
/// whose relaxed read-modify-write is a locked instruction.
///
/// Two arms, because attributing the second to the first is the whole point:
/// a relaxed `fetch_or` on a word nothing else touches, and then the real
/// `RamStore` write and read that contain one.
///
/// Printed, not asserted. Nothing in the language promises this — it is a
/// property of how x86-64 implements a `lock or`, and on AArch64 the same
/// `fetch_or(Relaxed)` orders nothing at all. A test that asserted zero here
/// would be asserting the accident, which is the opposite of what this file is
/// for.
#[test]
fn the_dirty_bitmap_is_already_a_barrier_on_a_locked_host() {
    let bit = search(Between::DirtyBit, "a relaxed fetch_or between");
    let store = store_buffer(Between::ThroughRamStore);
    println!(
        "through RamStore:            forbidden outcome {} / {ROUNDS} ({} overlapped)",
        store.both_zero, store.witnessed
    );
    assert!(
        bit.witnessed > 0 && store.witnessed > 0,
        "both runs must overlap or neither says anything"
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
    /// rather than the fix: the module documentation has why — the guest store
    /// ends in `RamStore::mark_dirty`, whose `fetch_or` is a locked
    /// instruction, and a whole interpreted instruction separates the store
    /// from the load in any case. So the count is printed, not asserted.
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
