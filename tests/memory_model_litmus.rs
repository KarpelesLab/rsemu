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
//! `std::thread` rather than `core::sync::Pool`, for the reason
//! `tests/smp_single_copy_atomicity.rs` gives: a test whose whole purpose is to
//! make two cores collide has to be able to say what a thread is.

#![cfg(feature = "std")]

use std::sync::atomic::{AtomicU8, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Barrier};

use rsemu::core::space::RamStore;
use rsemu::core::sync as rsync;

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

/// The control: with nothing between the store and the load, the host produces
/// the outcome a guest barrier exists to forbid.
///
/// Printed rather than asserted — how often depends on the host's scheduler,
/// and on a machine that happens to run the two threads one after the other it
/// is zero. What is asserted is that the run means something.
#[test]
fn a_relaxed_store_and_the_load_after_it_can_be_reordered() {
    let out = store_buffer(Between::Nothing);
    println!(
        "nothing between: forbidden outcome {} / {ROUNDS} ({} rounds overlapped)",
        out.both_zero, out.witnessed
    );
    assert!(
        out.witnessed > 0,
        "no round ever saw the other thread's zero, so the run proves nothing"
    );
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
    let bit = store_buffer(Between::DirtyBit);
    let tested = store_buffer(Between::DirtyBitTested);
    let store = store_buffer(Between::ThroughRamStore);
    println!(
        "a relaxed fetch_or between:  forbidden outcome {} / {ROUNDS} ({} overlapped)",
        bit.both_zero, bit.witnessed
    );
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
