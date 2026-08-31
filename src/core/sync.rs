//! The concurrency portability seam (`ROADMAP.md` §4.7).
//!
//! Nothing under `core/`, `cpu/`, `dev/`, `machine/` or `ir/` may name
//! `std::sync` or `std::thread`. Everything goes through here, so a device is
//! written once and runs on a threaded host, in a browser tab with no
//! `SharedArrayBuffer`, and on bare metal without a line changing.
//!
//! # Backends
//!
//! The API is identical across backends; one is selected at compile time.
//!
//! | Backend | Primitives | Selected when |
//! | --- | --- | --- |
//! | [`native_std`] | `std::sync` + `std::thread` | `std`, non-wasm |
//! | `single` | atomic cells, jobs run inline, waiting is a panic | everything else |
//! | `native-raw` | futex / `WaitOnAddress` by raw syscall | *not implemented* |
//! | `wasm-atomics` | shared memory + `Atomics.wait` | *not implemented* |
//!
//! `single` is not a degraded mode to be tolerated: it is the **reference
//! semantics**. It is the only backend that can *detect* the mistakes the other
//! three merely survive — a lock taken twice, a job that re-enters its
//! submitter — because with one thread those are unambiguously bugs rather than
//! contention. `ROADMAP.md` §4.7 requires a machine to produce the same state
//! hash under `single` and under `native-std`; the tests at the bottom of this
//! file run one identical workload through both and compare.
//!
//! `single` creates no threads. It does not get to assume the *process* has one
//! — a `no_std` build on a hosted target selects it, and `cargo test` runs that
//! build on parallel libtest threads — so its locks exclude for real. What is
//! single-threaded is the waiting, not the safety; see [`Global`].
//!
//! # Machine state and process-wide state
//!
//! [`Mutex`] and [`RwLock`] are for state a machine owns, and under `single` an
//! acquisition that would block is reported as the deadlock it is. A `static`
//! is not machine state: it is reachable from every thread in the process, the
//! test harness's included, so contention on it is legitimate and the lock must
//! wait. That is [`Global`], and the rule is mechanical — **if it lives in a
//! `static`, it is a `Global`** — enforced by a test in this file that reads
//! the crate's own source.
//!
//! The two unimplemented backends are extension points, not omissions. Both
//! plug in at the same place: add a module beside `single` exporting the same
//! items, then extend the `cfg` on the re-export block at the end of this file.
//! Each is marked `EXTENSION POINT` where it will attach.
//!
//! # Jobs, not threads
//!
//! There is no `spawn`. Background work is submitted to a [`Pool`] and returns
//! a [`Handle`]. This is forced by wasm — a Web Worker cannot be created
//! synchronously from arbitrary code, so the embedder builds the pool up front
//! and hands it in — and it is the better design anyway: thread count becomes a
//! machine property, work is schedulable, and nothing buried in a device model
//! can quietly create an OS thread.
//!
//! # Lock order
//!
//! Every lock carries a [`LockRank`]. In debug builds, acquiring a lock whose
//! rank is less than or equal to the highest rank this thread already holds
//! panics on the spot. Deadlock becomes a reproducible local panic instead of a
//! hang that only shows up on a reviewer's machine under load. The check costs
//! nothing in release builds, where it compiles away entirely.
//!
//! The default rank for [`Mutex::new`] is [`LockRank::LEAF`], which nests under
//! anything and *under which nothing may be taken* — including another leaf. A
//! lock that legitimately nests must say so with [`Mutex::with_rank`]. This is
//! deliberately strict: the ladder is only worth having if the default forces
//! the question.
//!
//! # Re-entrancy
//!
//! `ROADMAP.md` §4.7 replaces "never hold a lock across a call out" (which
//! forbids the phase-3 machine — OAM DMA and a BAR-moving config write both
//! need it) with a contract: mutate your own state in a short critical section,
//! **release it**, and only then call outward. This module supplies the
//! enforcement, not the good intentions:
//!
//! - Under `single`, a blocking acquire of a held lock panics rather than
//!   deadlocking, and a job runs inline at [`Pool::submit`], so a job that
//!   re-enters its submitter's critical section is caught immediately.
//! - Under every backend, the rank check turns "took two locks at once" into a
//!   panic naming both ranks.
//! - Under every backend, `try_lock` answers "am I already in here?" without
//!   blocking, which is how a handler detects its own re-entry portably.
//!
//! # What is deliberately absent
//!
//! - **`Condvar`.** `ROADMAP.md` §4.7 lists one, but a condition variable has
//!   no meaning under `single`: a wait that only another thread can satisfy is
//!   a guaranteed hang, so the `single` implementation could only ever panic.
//!   Exporting a primitive that is a compile-time-invisible hang on a
//!   first-class target is worse than not having it. The two places the core
//!   actually needs to wait — job completion and the stop-the-world barrier —
//!   are [`Handle::join`] and [`Pool::quiesce`], which `single` answers
//!   truthfully because the work has already run.
//! - **Poisoning.** `std` mutexes poison on panic; nothing else here can, and a
//!   `Result` on every `lock()` that is `Err` on exactly one backend is a trap.
//!   [`Mutex::lock`] returns the guard and ignores poisoning.

// Paths to the standard `core` are written `::core::` throughout: this module
// *is* `crate::core::sync`, and the leading `::` removes any doubt about which
// `core` a reader is looking at.
use ::core::fmt;
use ::core::marker::PhantomData;

// Atomics are re-exported rather than reimplemented — they are already
// portable, and a hot path that had to go through a wrapper would not be a hot
// path. Device code says `sync::AtomicU32`, never `core::sync::atomic`, so the
// seam stays the single import site (CLAUDE.md, Concurrency).
//
// The 64-bit atomics are gated: a 32-bit bare-metal target (thumbv7, rv32) has
// no `AtomicU64`, and a clock or performance counter that wants one there has
// to say so rather than failing to build.
//
// EXTENSION POINT (bare metal): where `target_has_atomic = "64"` is absent, a
// lock-backed `AtomicU64` shim can be provided behind these same names.
pub use ::core::sync::atomic::{
    AtomicBool, AtomicI8, AtomicI16, AtomicI32, AtomicIsize, AtomicPtr, AtomicU8, AtomicU16,
    AtomicU32, AtomicUsize, Ordering, compiler_fence, fence,
};
#[cfg(target_has_atomic = "64")]
pub use ::core::sync::atomic::{AtomicI64, AtomicU64};

/// Which concurrency backend this build selected.
///
/// Exposed because "which rsemu is this?" is a real question with a
/// build-specific answer (see [`crate::build_info`]), and because the
/// cross-backend equivalence check of `ROADMAP.md` §4.7 has to name what it
/// compared.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Backend {
    /// Creates no threads: jobs run inline, and a lock that would block says
    /// so rather than waiting.
    Single,
    /// `std::sync` primitives and a pool of `std::thread` workers.
    NativeStd,
    /// Futex / `WaitOnAddress` by raw syscall. Not implemented yet.
    NativeRaw,
    /// Shared linear memory and `Atomics.wait`. Not implemented yet.
    WasmAtomics,
}

impl Backend {
    /// The backend's name as `ROADMAP.md` §4.7 spells it.
    pub const fn name(self) -> &'static str {
        match self {
            Backend::Single => "single",
            Backend::NativeStd => "native-std",
            Backend::NativeRaw => "native-raw",
            Backend::WasmAtomics => "wasm-atomics",
        }
    }

    /// Whether a job submitted to a [`Pool`] can run on another thread.
    ///
    /// False for [`Backend::Single`], where `submit` runs the job inline.
    pub const fn is_threaded(self) -> bool {
        !matches!(self, Backend::Single)
    }
}

impl fmt::Display for Backend {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// The backend this build selected.
pub const BACKEND: Backend = if cfg!(all(feature = "std", not(target_family = "wasm"))) {
    Backend::NativeStd
} else {
    Backend::Single
};

// ---------------------------------------------------------------------------
// Lock ranks
// ---------------------------------------------------------------------------

/// A lock's position in the global acquisition order.
///
/// Locks must be acquired in **strictly increasing** rank. Debug builds assert
/// it (`ROADMAP.md` §4.7, CLAUDE.md); release builds compile the check away.
///
/// The named ranks are spaced by `0x1000` so one can be inserted between two
/// existing ranks without renumbering the ladder — but adding a rank is a claim
/// about the machine's call graph, so it belongs in this file with a sentence
/// saying why, not in the module that happened to need it.
///
/// The ladder, outermost first, follows the direction calls travel:
///
/// ```text
/// MACHINE -> TOPOLOGY -> SCHED -> BUS -> DEVICE -> WIRE -> POOL -> LEAF
/// ```
///
/// The scheduler dispatches an access into a bus, which routes it to a device,
/// which drives a wire. [`LockRank::WIRE`] sits *below* [`LockRank::DEVICE`]
/// even though a wire also delivers back into devices, because the re-entrancy
/// contract requires the wire's own lock to be released before its observers
/// are called — so the reverse edge never exists while a lock is held.
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct LockRank(pub u16);

impl LockRank {
    /// Exempt from the order check entirely: neither recorded nor validated.
    ///
    /// The escape hatch for a lock that genuinely sits outside the ladder — a
    /// diagnostic counter, a test fixture. Using it to silence a real violation
    /// converts a debug panic into a production deadlock, so it wants a comment
    /// every time it appears.
    pub const UNCHECKED: LockRank = LockRank(0x0000);

    /// Whole-machine lifecycle and configuration. The outermost lock there is.
    pub const MACHINE: LockRank = LockRank(0x1000);

    /// Address-space topology: the region tree, mappings, the generation
    /// counter. Held by a remap, which then calls into buses.
    pub const TOPOLOGY: LockRank = LockRank(0x2000);

    /// The event queue and virtual clock (`ROADMAP.md` §4.2).
    pub const SCHED: LockRank = LockRank(0x3000);

    /// A bus fabric's routing state.
    pub const BUS: LockRank = LockRank(0x4000);

    /// A device model's own state — the common case for a device author.
    pub const DEVICE: LockRank = LockRank(0x5000);

    /// An interrupt or GPIO line's level and observer list (`ROADMAP.md` §4.3).
    pub const WIRE: LockRank = LockRank(0x6000);

    /// Task-pool internals: the job queue and its bookkeeping.
    ///
    /// Ranked last of the named ranks so that submitting a job while holding a
    /// device lock is *legal* — submission queues work, it does not call
    /// outward — while a job queue that reached back for a device lock would be
    /// caught.
    pub const POOL: LockRank = LockRank(0x7000);

    /// The default for [`Mutex::new`]: nests under anything, holds nothing.
    ///
    /// Acquiring any lock — including another leaf — while holding a leaf is a
    /// violation. Most locks really are leaves; the ones that are not have to
    /// say so, which is the point.
    pub const LEAF: LockRank = LockRank(0xffff);

    /// A rank from a raw value, for ladders defined outside this file.
    pub const fn new(rank: u16) -> LockRank {
        LockRank(rank)
    }

    /// The name of a well-known rank, or `None` for a custom one.
    pub const fn name(self) -> Option<&'static str> {
        Some(match self {
            LockRank::UNCHECKED => "UNCHECKED",
            LockRank::MACHINE => "MACHINE",
            LockRank::TOPOLOGY => "TOPOLOGY",
            LockRank::SCHED => "SCHED",
            LockRank::BUS => "BUS",
            LockRank::DEVICE => "DEVICE",
            LockRank::WIRE => "WIRE",
            LockRank::POOL => "POOL",
            LockRank::LEAF => "LEAF",
            _ => return None,
        })
    }

    /// Records this rank as held by the current thread, checking the order.
    ///
    /// Panics in debug builds if this rank is less than or equal to the highest
    /// rank the thread already holds. The returned guard releases the rank when
    /// dropped; guards may be dropped in any order.
    ///
    /// The locks in this module call it for you. Call it directly when you own
    /// a lock-shaped resource that is not one of ours — a host mutex behind the
    /// `host/` line, an accelerator ioctl that serialises a vCPU.
    #[must_use = "the rank is held until the returned guard is dropped"]
    pub fn enter(self) -> RankGuard {
        #[cfg(debug_assertions)]
        if self != LockRank::UNCHECKED {
            if let Some(held) = held_rank()
                && self <= held
            {
                panic!("lock order violation: acquiring {self} while holding {held}");
            }
            rank_track::push(self.0);
        }
        RankGuard {
            #[cfg(debug_assertions)]
            rank: self,
            _not_send: PhantomData,
        }
    }

    /// Records this rank as held without checking the order.
    ///
    /// For a *non-blocking* acquisition (`try_lock` and friends). A try-lock
    /// cannot join a deadlock cycle — it reports failure instead of waiting —
    /// so out-of-order try-locks are legal. It still records, because a
    /// blocking acquire made underneath one must be checked.
    #[must_use = "the rank is held until the returned guard is dropped"]
    pub fn enter_nonblocking(self) -> RankGuard {
        #[cfg(debug_assertions)]
        if self != LockRank::UNCHECKED {
            rank_track::push(self.0);
        }
        RankGuard {
            #[cfg(debug_assertions)]
            rank: self,
            _not_send: PhantomData,
        }
    }
}

impl fmt::Debug for LockRank {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.name() {
            Some(name) => write!(f, "LockRank({name})"),
            None => write!(f, "LockRank({:#06x})", self.0),
        }
    }
}

impl fmt::Display for LockRank {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.name() {
            Some(name) => f.write_str(name),
            None => write!(f, "rank {:#06x}", self.0),
        }
    }
}

/// Evidence that the current thread holds a [`LockRank`].
///
/// Released on drop. Out-of-order drops are fine: the tracker removes this
/// guard's own rank rather than assuming a stack discipline it cannot enforce.
///
/// Not `Send`: which ranks a thread holds is that thread's business.
#[derive(Debug)]
pub struct RankGuard {
    #[cfg(debug_assertions)]
    rank: LockRank,
    _not_send: PhantomData<*const ()>,
}

impl Drop for RankGuard {
    fn drop(&mut self) {
        #[cfg(debug_assertions)]
        if self.rank != LockRank::UNCHECKED {
            rank_track::remove(self.rank.0);
        }
    }
}

/// The highest rank the current thread currently holds.
///
/// Always `None` in release builds: rank tracking is a debug-only facility.
pub fn held_rank() -> Option<LockRank> {
    #[cfg(debug_assertions)]
    {
        rank_track::max().map(LockRank)
    }
    #[cfg(not(debug_assertions))]
    {
        None
    }
}

/// Whether acquiring `rank` now would violate the lock order.
///
/// The non-panicking form of the check [`LockRank::enter`] performs, for tests
/// and for diagnostics that would rather report than abort. Always `false` in
/// release builds.
pub fn violates_lock_order(rank: LockRank) -> bool {
    if rank == LockRank::UNCHECKED {
        return false;
    }
    matches!(held_rank(), Some(held) if rank <= held)
}

/// Per-thread rank bookkeeping, compiled only into debug builds.
///
/// Three storage strategies, because there is no portable per-thread storage
/// below `std` — `#[thread_local]` is unstable — and because "no `std`" does
/// not imply "one thread":
///
///  1. **Thread-local**, whenever `std` is reachable: the `std` feature, or a
///     unit-test build. The libtest harness runs `#[test]` functions on
///     parallel threads even when the crate under test is `no_std`, and it
///     links `std` regardless, so tests take this path whatever the feature set
///     says.
///  2. **Process globals**, where the target cannot create a thread at all:
///     bare metal (`target_os = "none"`) and wasm without the threads proposal.
///     A shared stack is exact there because there is only ever one stack.
///  3. **Nothing at all**, for what is left: a hosted `no_std` build, and
///     threaded wasm. Both have threads and no TLS to separate them, and a
///     shared stack would have one thread inventing violations in another —
///     which is not a weaker check but a false one, and a flaky panic is worse
///     than no panic. Note that `cfg(test)` is *not* set when the library is
///     compiled for an integration test, so this is the path a
///     `--no-default-features` `tests/` binary takes.
///
/// EXTENSION POINT (`native-raw`): case 3 is where the check goes missing, and
/// a TLS slot from the raw-syscall layer, keyed the same way, is what restores
/// it. `native-raw` must not be selected before it has one.
#[cfg(all(debug_assertions, any(feature = "std", test)))]
mod rank_track {
    #[cfg(all(test, not(feature = "std")))]
    extern crate std;

    use ::core::cell::RefCell;

    /// Deeper than this and the lock graph is the bug, not the tracker.
    const CAPACITY: usize = 32;

    /// The ranks held, unordered: `max` scans, so removal need not shift.
    struct Held {
        ranks: [u16; CAPACITY],
        depth: usize,
    }

    impl Held {
        const fn new() -> Held {
            Held {
                ranks: [0; CAPACITY],
                depth: 0,
            }
        }
    }

    std::thread_local! {
        static HELD: RefCell<Held> = const { RefCell::new(Held::new()) };
    }

    /// Runs `f` over this thread's held set, or does nothing if the thread's
    /// TLS is already torn down (a lock taken in a destructor: rare, not an
    /// error, and certainly not worth a panic on the way out).
    fn with<R: Default>(f: impl FnOnce(&mut Held) -> R) -> R {
        HELD.try_with(|held| f(&mut held.borrow_mut()))
            .unwrap_or_default()
    }

    pub(super) fn push(rank: u16) {
        with(|held| {
            assert!(
                held.depth < CAPACITY,
                "more than {CAPACITY} locks held at once; the lock graph is the bug"
            );
            held.ranks[held.depth] = rank;
            held.depth += 1;
        });
    }

    pub(super) fn remove(rank: u16) {
        with(|held| {
            if let Some(at) = held.ranks[..held.depth].iter().rposition(|&r| r == rank) {
                held.depth -= 1;
                held.ranks[at] = held.ranks[held.depth];
            }
        });
    }

    pub(super) fn max() -> Option<u16> {
        with(|held| held.ranks[..held.depth].iter().copied().max())
    }
}

/// Rank bookkeeping on a target with one thread and no thread-local storage;
/// see the sibling module's documentation (case 2) for why globals are exact.
#[cfg(all(
    debug_assertions,
    not(any(feature = "std", test)),
    any(
        target_os = "none",
        all(target_family = "wasm", not(target_feature = "atomics"))
    )
))]
mod rank_track {
    use ::core::sync::atomic::{AtomicU16, AtomicUsize, Ordering};

    const CAPACITY: usize = 32;

    // Relaxed throughout: this cfg is reachable only on a target that cannot
    // create a thread, so there is one stack and nothing to order against. The
    // atomics buy safe interior mutability in a `static`, not synchronisation.
    static RANKS: [AtomicU16; CAPACITY] = [const { AtomicU16::new(0) }; CAPACITY];
    static DEPTH: AtomicUsize = AtomicUsize::new(0);

    pub(super) fn push(rank: u16) {
        let depth = DEPTH.load(Ordering::Relaxed);
        assert!(
            depth < CAPACITY,
            "more than {CAPACITY} locks held at once; the lock graph is the bug"
        );
        RANKS[depth].store(rank, Ordering::Relaxed);
        DEPTH.store(depth + 1, Ordering::Relaxed);
    }

    pub(super) fn remove(rank: u16) {
        let depth = DEPTH.load(Ordering::Relaxed);
        for at in (0..depth).rev() {
            if RANKS[at].load(Ordering::Relaxed) == rank {
                let last = RANKS[depth - 1].load(Ordering::Relaxed);
                RANKS[at].store(last, Ordering::Relaxed);
                DEPTH.store(depth - 1, Ordering::Relaxed);
                return;
            }
        }
    }

    pub(super) fn max() -> Option<u16> {
        let depth = DEPTH.load(Ordering::Relaxed);
        (0..depth).map(|at| RANKS[at].load(Ordering::Relaxed)).max()
    }
}

/// Rank bookkeeping where there is neither thread-local storage nor a
/// guarantee of one thread; see the sibling modules' documentation (case 3).
///
/// The ladder is not checked in this configuration. A shared stack would report
/// one thread's holdings as another's, so the choice is between no check and a
/// check that fires at random — and a lock-order panic nobody can reproduce
/// teaches the reader that the tool lies. The `std` and unit-test paths above
/// keep the ladder honest for every build a person actually debugs.
#[cfg(all(
    debug_assertions,
    not(any(feature = "std", test)),
    not(any(
        target_os = "none",
        all(target_family = "wasm", not(target_feature = "atomics"))
    ))
))]
mod rank_track {
    pub(super) fn push(_rank: u16) {}

    pub(super) fn remove(_rank: u16) {}

    pub(super) fn max() -> Option<u16> {
        None
    }
}

// ---------------------------------------------------------------------------
// The `single` backend
// ---------------------------------------------------------------------------

/// The `single` backend: no threads, and the reference semantics.
///
/// Locks are atomic cells; [`Pool::submit`] runs the job inline and hands back
/// a [`Handle`] that already holds the answer. Used by the no-threads browser
/// build, bare metal, and the deterministic test runner (`ROADMAP.md` §4.7,
/// §11.3).
///
/// This backend submits no work to a second thread, so within a machine every
/// operation that *would* block is a bug rather than contention, and it says so
/// with a panic instead of hanging. That is what makes it the reference: a
/// suite run under `single` fails loudly where `native-std` merely gets lucky.
/// The one place the assumption does not hold is a `static`, which is why there
/// is a [`Global`](super::Global) — see below.
///
/// # Why this module contains `unsafe`
///
/// `ROADMAP.md` §0 sanctions this module as one of six that may opt back in.
/// The core requires `Send + Sync` on everything, and `RefCell` is not `Sync`,
/// so there is no safe way to build a lock that satisfies the bound. The cells
/// here are `UnsafeCell` with hand-written `Send`/`Sync` impls; the SAFETY
/// comments on those impls carry the argument.
///
/// # What "single-threaded" does and does not mean here
///
/// `single` creates no threads. It does **not** get to assume the process has
/// only one: the libtest harness runs `#[test]` functions on parallel threads
/// whatever the crate under test says, and a `no_std` build on a hosted target
/// selects this backend anyway. Anything reachable from a `static` is therefore
/// reachable from several threads at once in a perfectly ordinary `cargo test`.
///
/// So the exclusion here is **real**, not notional: the flag is an atomic and
/// the acquire/release pair is a genuine read-modify-write, which is what makes
/// [`Mutex`]'s `Sync` impl true rather than merely conventional. What stays
/// single-threaded is the *waiting*: an acquisition that would block panics
/// instead, because with one thread that can only be a deadlock. Process-wide
/// state, where a second thread is legitimate and waiting is the right answer,
/// belongs in [`Global`](super::Global) rather than in a `static Mutex`.
///
/// This module is public in every build, not only where it is selected, so the
/// equivalence tests can run both backends in one binary. Outside those tests,
/// use the re-exports at the top of [`crate::core::sync`] and let the `cfg`
/// choose.
// Crate-private, deliberately. Nothing outside this crate should be picking a
// backend by hand: the selected one is re-exported publicly below, so nothing
// legitimate is lost, and the in-crate equivalence tests still see both.
// Three allows, one reason. On a target where `native_std` is selected this
// module is compiled but not re-exported, so its items are both dead and
// unreachable — expected, since it stays compiled only so the equivalence tests
// can instantiate both backends in one binary. The items must still be declared
// `pub` because the `no_std` path re-exports them publicly. Unused crate-private
// items are dropped by the compiler rather than shipped.
#[allow(unsafe_code, dead_code, unreachable_pub)]
pub(crate) mod single {
    use super::{LockRank, RankGuard};
    use ::core::cell::UnsafeCell;
    use ::core::fmt;
    use ::core::marker::PhantomData;
    use ::core::ops::{Deref, DerefMut};
    use ::core::sync::atomic::{AtomicBool, AtomicIsize, AtomicU8, Ordering};

    /// The claim/release pair both locks are built on.
    ///
    /// A read-modify-write where the target has one, and a plain load/store
    /// where it does not. The fallback is not a weaker lock on those targets:
    /// a core with no compare-and-swap (thumbv6m, and the reason [`Once`] is
    /// written the way it is) has no way to run two threads over this memory,
    /// so the pair is atomic with respect to everything that can observe it.
    /// Where CAS exists — every target in the matrix, plus the libtest harness
    /// that made this necessary — the RMW is what makes the `Sync` impls below
    /// true statements rather than house style.
    mod exclusion {
        use ::core::sync::atomic::{AtomicBool, AtomicIsize, Ordering};

        /// Claim `flag`, reporting whether it was **already** claimed.
        #[cfg(target_has_atomic = "8")]
        #[inline]
        pub(super) fn claim(flag: &AtomicBool) -> bool {
            flag.compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
                .is_err()
        }

        #[cfg(not(target_has_atomic = "8"))]
        #[inline]
        pub(super) fn claim(flag: &AtomicBool) -> bool {
            if flag.load(Ordering::Acquire) {
                return true;
            }
            flag.store(true, Ordering::Relaxed);
            false
        }

        /// Release a claim taken by [`claim`].
        #[inline]
        pub(super) fn release(flag: &AtomicBool) {
            flag.store(false, Ordering::Release);
        }

        /// Take a share of `state`, or report that a writer holds it.
        #[cfg(target_has_atomic = "ptr")]
        #[inline]
        pub(super) fn share(state: &AtomicIsize) -> bool {
            let mut seen = state.load(Ordering::Relaxed);
            loop {
                if seen < 0 {
                    return false;
                }
                match state.compare_exchange_weak(
                    seen,
                    seen + 1,
                    Ordering::Acquire,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => return true,
                    Err(actual) => seen = actual,
                }
            }
        }

        #[cfg(not(target_has_atomic = "ptr"))]
        #[inline]
        pub(super) fn share(state: &AtomicIsize) -> bool {
            let seen = state.load(Ordering::Acquire);
            if seen < 0 {
                return false;
            }
            state.store(seen + 1, Ordering::Relaxed);
            true
        }

        /// Give up a share taken by [`share`].
        #[cfg(target_has_atomic = "ptr")]
        #[inline]
        pub(super) fn unshare(state: &AtomicIsize) {
            state.fetch_sub(1, Ordering::Release);
        }

        #[cfg(not(target_has_atomic = "ptr"))]
        #[inline]
        pub(super) fn unshare(state: &AtomicIsize) {
            let seen = state.load(Ordering::Relaxed);
            state.store(seen - 1, Ordering::Release);
        }

        /// Take `state` exclusively, or report that somebody holds it.
        #[cfg(target_has_atomic = "ptr")]
        #[inline]
        pub(super) fn seize(state: &AtomicIsize) -> bool {
            state
                .compare_exchange(0, -1, Ordering::Acquire, Ordering::Relaxed)
                .is_ok()
        }

        #[cfg(not(target_has_atomic = "ptr"))]
        #[inline]
        pub(super) fn seize(state: &AtomicIsize) -> bool {
            if state.load(Ordering::Acquire) != 0 {
                return false;
            }
            state.store(-1, Ordering::Relaxed);
            true
        }

        /// Give up the exclusive hold taken by [`seize`].
        #[inline]
        pub(super) fn relinquish(state: &AtomicIsize) {
            state.store(0, Ordering::Release);
        }
    }

    /// A mutual-exclusion lock with nobody expected to exclude.
    ///
    /// Locking claims a flag and panics if it was already claimed. On the
    /// targets this backend is selected for there is one thread, so a lock that
    /// is already held can only be this thread's, and waiting for yourself is a
    /// hang; the panic names the situation while the stack still shows both
    /// acquisitions. State that genuinely *is* shared between threads — a
    /// process-wide table, which a `static` makes reachable from the test
    /// harness's threads as much as from anyone's — wants
    /// [`Global`](super::Global), which waits instead of reporting.
    pub struct Mutex<T: ?Sized> {
        rank: LockRank,
        locked: AtomicBool,
        data: UnsafeCell<T>,
    }

    // SAFETY: sending the whole lock to another thread moves the flag and the
    // `UnsafeCell` with it, so no two threads ever hold a reference to the same
    // one. This is the bound `std::sync::Mutex` carries, for the same reason.
    unsafe impl<T: ?Sized + Send> Send for Mutex<T> {}

    // SAFETY: `&Mutex<T>` hands out `&mut T` through a guard, so the guard must
    // be the only live handle to the value for as long as it exists. That is a
    // property of the flag, not of the target:
    //
    //  * `locked` is an `AtomicBool` and a claim is a single `compare_exchange`
    //    (a plain load/store only where the target has no CAS at all, which is
    //    a target that cannot run two threads over this memory in the first
    //    place). At most one caller observes the transition false -> true, so
    //    at most one guard exists at a time.
    //  * The claim is `Acquire` and the release is `Release`, so everything the
    //    previous holder wrote through its guard happens-before everything the
    //    next holder reads. Without that ordering the exclusion would be real
    //    and the data still racy.
    //
    // An earlier version of this argument rested on "single-threaded by
    // construction" — selection, structure, and a thread-id tripwire — and it
    // was **false**. `BACKEND` picks `single` for any `no_std` build, including
    // one on a hosted target, and `cargo test` runs that build's tests on
    // parallel libtest threads. A `static Mutex` was therefore reachable from
    // two threads through entirely safe code, which is a data race on the flag
    // and undefined behaviour regardless of how tidily it happened to surface.
    // Soundness is now established by the primitive rather than by a convention
    // about who calls it; what remains single-threaded is only the *waiting*,
    // which is a liveness policy and cannot be unsound. See [`Global`] for the
    // type process-wide state is supposed to use.
    //
    // `T: Send` and not `T: Sync`, matching `std::sync::Mutex`: the guard is an
    // exclusive borrow, so `T` is never shared, only moved between threads.
    unsafe impl<T: ?Sized + Send> Sync for Mutex<T> {}

    impl<T> Mutex<T> {
        /// A lock at [`LockRank::LEAF`] — nests under anything, holds nothing.
        pub const fn new(value: T) -> Mutex<T> {
            Mutex::with_rank(LockRank::LEAF, value)
        }

        /// A lock at an explicit rank, for one that legitimately nests.
        pub const fn with_rank(rank: LockRank, value: T) -> Mutex<T> {
            Mutex {
                rank,
                locked: AtomicBool::new(false),
                data: UnsafeCell::new(value),
            }
        }

        /// Consumes the lock and returns the protected value.
        pub fn into_inner(self) -> T {
            self.data.into_inner()
        }
    }

    impl<T: ?Sized> Mutex<T> {
        /// This lock's rank in the global acquisition order.
        pub fn rank(&self) -> LockRank {
            self.rank
        }

        /// Acquires the lock.
        ///
        /// # Panics
        ///
        /// If the lock is already held — under one thread that is a deadlock,
        /// reported rather than performed — or if the rank order is violated.
        pub fn lock(&self) -> MutexGuard<'_, T> {
            let rank = self.rank.enter();
            assert!(
                !exclusion::claim(&self.locked),
                "recursive lock of a `single` Mutex ({}): this deadlocks on a threaded backend",
                self.rank
            );
            MutexGuard {
                lock: self,
                _rank: rank,
                _not_send: PhantomData,
            }
        }

        /// Acquires the lock, or returns `None` if it is already held.
        ///
        /// The portable way to ask "am I already inside this critical
        /// section?": it answers on every backend without blocking and without
        /// panicking.
        pub fn try_lock(&self) -> Option<MutexGuard<'_, T>> {
            let rank = self.rank.enter_nonblocking();
            if exclusion::claim(&self.locked) {
                return None;
            }
            Some(MutexGuard {
                lock: self,
                _rank: rank,
                _not_send: PhantomData,
            })
        }

        /// Borrows the value directly, given exclusive access to the lock.
        pub fn get_mut(&mut self) -> &mut T {
            self.data.get_mut()
        }
    }

    impl<T: Default> Default for Mutex<T> {
        fn default() -> Mutex<T> {
            Mutex::new(T::default())
        }
    }

    impl<T> From<T> for Mutex<T> {
        fn from(value: T) -> Mutex<T> {
            Mutex::new(value)
        }
    }

    impl<T: ?Sized + fmt::Debug> fmt::Debug for Mutex<T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            let mut s = f.debug_struct("Mutex");
            s.field("rank", &self.rank);
            match self.try_lock() {
                Some(guard) => s.field("data", &&*guard).finish(),
                None => s.field("data", &"<locked>").finish(),
            }
        }
    }

    /// Exclusive access to the value inside a [`Mutex`], released on drop.
    pub struct MutexGuard<'a, T: ?Sized> {
        lock: &'a Mutex<T>,
        _rank: RankGuard,
        _not_send: PhantomData<*const ()>,
    }

    impl<T: ?Sized> Deref for MutexGuard<'_, T> {
        type Target = T;

        fn deref(&self) -> &T {
            // SAFETY: this guard exists because its constructor observed the
            // flag go false -> true atomically, and nothing sets it back until
            // the guard drops, so no second guard — on this thread or any other
            // — can exist for this guard's lifetime.
            unsafe { &*self.lock.data.get() }
        }
    }

    impl<T: ?Sized> DerefMut for MutexGuard<'_, T> {
        fn deref_mut(&mut self) -> &mut T {
            // SAFETY: as `deref`, plus `&mut self` proving this guard — the
            // only live handle to the value — is uniquely borrowed.
            unsafe { &mut *self.lock.data.get() }
        }
    }

    impl<T: ?Sized> Drop for MutexGuard<'_, T> {
        fn drop(&mut self) {
            exclusion::release(&self.lock.locked);
        }
    }

    impl<T: ?Sized + fmt::Debug> fmt::Debug for MutexGuard<'_, T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            fmt::Debug::fmt(&**self, f)
        }
    }

    /// A reader-writer lock with no readers to run in parallel.
    ///
    /// Kept honest anyway: several readers may hold it at once, a writer
    /// excludes them, and violating that panics rather than waiting.
    pub struct RwLock<T: ?Sized> {
        rank: LockRank,
        /// `-1` for a writer, otherwise the reader count.
        state: AtomicIsize,
        data: UnsafeCell<T>,
    }

    // SAFETY: as `Mutex`, above.
    unsafe impl<T: ?Sized + Send> Send for RwLock<T> {}

    // SAFETY: as `Mutex`, above — the state is an atomic and every transition
    // is a read-modify-write, so a writer excludes every other holder for real
    // — with the usual `RwLock` addition: a read guard hands out `&T` to what
    // may be several holders at once, so sharing the lock requires `T: Sync` as
    // well as `T: Send`.
    unsafe impl<T: ?Sized + Send + Sync> Sync for RwLock<T> {}

    impl<T> RwLock<T> {
        /// A lock at [`LockRank::LEAF`].
        pub const fn new(value: T) -> RwLock<T> {
            RwLock::with_rank(LockRank::LEAF, value)
        }

        /// A lock at an explicit rank.
        pub const fn with_rank(rank: LockRank, value: T) -> RwLock<T> {
            RwLock {
                rank,
                state: AtomicIsize::new(0),
                data: UnsafeCell::new(value),
            }
        }

        /// Consumes the lock and returns the protected value.
        pub fn into_inner(self) -> T {
            self.data.into_inner()
        }
    }

    impl<T: ?Sized> RwLock<T> {
        /// This lock's rank in the global acquisition order.
        pub fn rank(&self) -> LockRank {
            self.rank
        }

        /// Acquires shared access.
        ///
        /// # Panics
        ///
        /// If a writer holds the lock, or if the rank order is violated.
        pub fn read(&self) -> RwLockReadGuard<'_, T> {
            let rank = self.rank.enter();
            assert!(
                exclusion::share(&self.state),
                "read of a `single` RwLock ({}) held for writing: this deadlocks on a threaded \
                 backend",
                self.rank
            );
            RwLockReadGuard {
                lock: self,
                _rank: rank,
                _not_send: PhantomData,
            }
        }

        /// Acquires exclusive access.
        ///
        /// # Panics
        ///
        /// If the lock is held at all, or if the rank order is violated.
        pub fn write(&self) -> RwLockWriteGuard<'_, T> {
            let rank = self.rank.enter();
            assert!(
                exclusion::seize(&self.state),
                "write of a `single` RwLock ({}) that is already held: this deadlocks on a \
                 threaded backend",
                self.rank
            );
            RwLockWriteGuard {
                lock: self,
                _rank: rank,
                _not_send: PhantomData,
            }
        }

        /// Acquires shared access, or returns `None` if a writer holds it.
        pub fn try_read(&self) -> Option<RwLockReadGuard<'_, T>> {
            let rank = self.rank.enter_nonblocking();
            if !exclusion::share(&self.state) {
                return None;
            }
            Some(RwLockReadGuard {
                lock: self,
                _rank: rank,
                _not_send: PhantomData,
            })
        }

        /// Acquires exclusive access, or returns `None` if the lock is held.
        pub fn try_write(&self) -> Option<RwLockWriteGuard<'_, T>> {
            let rank = self.rank.enter_nonblocking();
            if !exclusion::seize(&self.state) {
                return None;
            }
            Some(RwLockWriteGuard {
                lock: self,
                _rank: rank,
                _not_send: PhantomData,
            })
        }

        /// Borrows the value directly, given exclusive access to the lock.
        pub fn get_mut(&mut self) -> &mut T {
            self.data.get_mut()
        }
    }

    impl<T: Default> Default for RwLock<T> {
        fn default() -> RwLock<T> {
            RwLock::new(T::default())
        }
    }

    impl<T> From<T> for RwLock<T> {
        fn from(value: T) -> RwLock<T> {
            RwLock::new(value)
        }
    }

    impl<T: ?Sized + fmt::Debug> fmt::Debug for RwLock<T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            let mut s = f.debug_struct("RwLock");
            s.field("rank", &self.rank);
            match self.try_read() {
                Some(guard) => s.field("data", &&*guard).finish(),
                None => s.field("data", &"<locked>").finish(),
            }
        }
    }

    /// Shared access to the value inside an [`RwLock`], released on drop.
    pub struct RwLockReadGuard<'a, T: ?Sized> {
        lock: &'a RwLock<T>,
        _rank: RankGuard,
        _not_send: PhantomData<*const ()>,
    }

    impl<T: ?Sized> Deref for RwLockReadGuard<'_, T> {
        type Target = T;

        fn deref(&self) -> &T {
            // SAFETY: this guard exists because its constructor atomically
            // raised a non-negative reader count, and the count stays positive
            // until the guard drops, so no writer can seize the lock meanwhile
            // and no `&mut T` to the value exists.
            unsafe { &*self.lock.data.get() }
        }
    }

    impl<T: ?Sized> Drop for RwLockReadGuard<'_, T> {
        fn drop(&mut self) {
            exclusion::unshare(&self.lock.state);
        }
    }

    impl<T: ?Sized + fmt::Debug> fmt::Debug for RwLockReadGuard<'_, T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            fmt::Debug::fmt(&**self, f)
        }
    }

    /// Exclusive access to the value inside an [`RwLock`], released on drop.
    pub struct RwLockWriteGuard<'a, T: ?Sized> {
        lock: &'a RwLock<T>,
        _rank: RankGuard,
        _not_send: PhantomData<*const ()>,
    }

    impl<T: ?Sized> Deref for RwLockWriteGuard<'_, T> {
        type Target = T;

        fn deref(&self) -> &T {
            // SAFETY: this guard exists because its constructor atomically took
            // the state from `0` to `-1`, which every other acquisition refuses
            // to touch, so this is the only live handle to the value.
            unsafe { &*self.lock.data.get() }
        }
    }

    impl<T: ?Sized> DerefMut for RwLockWriteGuard<'_, T> {
        fn deref_mut(&mut self) -> &mut T {
            // SAFETY: as `deref`, plus `&mut self` proving the borrow is unique.
            unsafe { &mut *self.lock.data.get() }
        }
    }

    impl<T: ?Sized> Drop for RwLockWriteGuard<'_, T> {
        fn drop(&mut self) {
            exclusion::relinquish(&self.lock.state);
        }
    }

    impl<T: ?Sized + fmt::Debug> fmt::Debug for RwLockWriteGuard<'_, T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            fmt::Debug::fmt(&**self, f)
        }
    }

    /// One-time initialisation.
    ///
    /// Built on an atomic rather than a cell so it needs no `unsafe` of its
    /// own, and on plain loads and stores rather than a read-modify-write so it
    /// works on a target with no compare-and-swap at all (thumbv6m). The
    /// intermediate `RUNNING` state exists purely to catch a `call_once` that
    /// re-enters itself, which would otherwise observe an uninitialised value
    /// and return as if it were ready.
    ///
    /// The load/store pair means this is **not** a cross-thread rendezvous:
    /// two threads arriving together could both run the initialiser. That is
    /// the same restriction [`Mutex`] states — waiting is what `single` does
    /// not do — so process-wide state wants [`Global`](super::Global), whose
    /// value is `const`-constructible and needs no initialiser to race over.
    #[derive(Debug)]
    pub struct Once {
        state: AtomicU8,
    }

    const INCOMPLETE: u8 = 0;
    const RUNNING: u8 = 1;
    const COMPLETE: u8 = 2;

    impl Once {
        /// A `Once` that has not run.
        pub const fn new() -> Once {
            Once {
                state: AtomicU8::new(INCOMPLETE),
            }
        }

        /// Runs `f` unless it has already run.
        ///
        /// # Panics
        ///
        /// If called from within its own initialiser, or if a previous
        /// initialiser panicked — matching `std::sync::Once`, which poisons.
        pub fn call_once(&self, f: impl FnOnce()) {
            match self.state.load(Ordering::Relaxed) {
                INCOMPLETE => {
                    self.state.store(RUNNING, Ordering::Relaxed);
                    f();
                    self.state.store(COMPLETE, Ordering::Relaxed);
                }
                COMPLETE => {}
                _ => panic!("Once re-entered, or a previous initialiser panicked"),
            }
        }

        /// Whether the initialiser has run to completion.
        pub fn is_completed(&self) -> bool {
            self.state.load(Ordering::Relaxed) == COMPLETE
        }
    }

    impl Default for Once {
        fn default() -> Once {
            Once::new()
        }
    }

    /// A task pool with nowhere else to run tasks.
    ///
    /// [`Pool::submit`] runs the job before it returns. Eagerly, not lazily:
    /// submitted background work must happen even if the caller drops the
    /// handle, which is the normal case for a JIT tier-up or a snapshot
    /// compression.
    #[derive(Debug)]
    pub struct Pool {
        requested_workers: usize,
    }

    impl Pool {
        /// Builds a pool. `workers` is recorded and otherwise ignored: this
        /// backend has no threads to put them on.
        pub fn new(workers: usize) -> Pool {
            Pool {
                requested_workers: workers,
            }
        }

        /// The number of worker threads: always zero here.
        ///
        /// Zero means "jobs run inline on the submitting thread", which is a
        /// real answer rather than a missing one, and a caller sizing work
        /// should read it as one.
        pub fn workers(&self) -> usize {
            0
        }

        /// What the machine configuration asked for, whether or not it was
        /// available — so a diagnostic can say "you asked for 8 workers on a
        /// target that has none".
        pub fn requested_workers(&self) -> usize {
            self.requested_workers
        }

        /// Runs `job` immediately and returns a handle holding its result.
        ///
        /// The bounds match the threaded backends exactly — `Send + 'static` —
        /// so code developed against `single` compiles unchanged against
        /// `native-std`. A seam whose bounds relax on the easy target is not a
        /// seam.
        pub fn submit<F, T>(&self, job: F) -> Handle<T>
        where
            F: FnOnce() -> T + Send + 'static,
            T: Send + 'static,
        {
            Handle { value: Some(job()) }
        }

        /// Waits until every submitted job has finished: already true here.
        ///
        /// This is the stop-the-world barrier of `ROADMAP.md` §4.7. It is a
        /// no-op under `single` rather than an unsupported operation, which is
        /// what lets the safe-point protocol be written once.
        pub fn quiesce(&self) {}
    }

    /// The result of a submitted job.
    #[derive(Debug)]
    pub struct Handle<T> {
        value: Option<T>,
    }

    impl<T> Handle<T> {
        /// Waits for the job and returns its result.
        pub fn join(mut self) -> T {
            self.value
                .take()
                .expect("a `single` job's result is produced at submit time")
        }

        /// Whether the job has finished: always true here.
        pub fn is_finished(&self) -> bool {
            true
        }
    }
}

// ---------------------------------------------------------------------------
// The `native-std` backend
// ---------------------------------------------------------------------------

/// The `native-std` backend: `std::sync` primitives and `std::thread` workers.
///
/// A thin layer over `std`, deliberately — the interesting decisions (ranks,
/// jobs instead of spawn, no poisoning) live in the seam, not in the backend.
/// What this module adds to `std` is the rank instrumentation on every
/// acquisition and a pool that owns its threads.
///
/// This is the only module under `core/` permitted to name `std::sync` or
/// `std::thread`; that is what being the seam means.
#[cfg(all(feature = "std", not(target_family = "wasm")))]
pub mod native_std {
    use super::{LockRank, RankGuard};
    use ::core::fmt;
    use ::core::marker::PhantomData;
    use ::core::ops::{Deref, DerefMut};
    use std::boxed::Box;
    use std::collections::VecDeque;
    use std::panic::{AssertUnwindSafe, catch_unwind, resume_unwind};
    use std::sync::{Arc, Condvar, PoisonError, TryLockError};
    use std::thread::{self, JoinHandle};
    use std::vec::Vec;

    /// A mutual-exclusion lock.
    ///
    /// Poisoning is dropped on the floor; see the module documentation of
    /// [`crate::core::sync`]. A panic while a device lock is held leaves the
    /// device in whatever state it reached, exactly as it does under `single`.
    pub struct Mutex<T: ?Sized> {
        rank: LockRank,
        inner: std::sync::Mutex<T>,
    }

    impl<T> Mutex<T> {
        /// A lock at [`LockRank::LEAF`] — nests under anything, holds nothing.
        pub const fn new(value: T) -> Mutex<T> {
            Mutex::with_rank(LockRank::LEAF, value)
        }

        /// A lock at an explicit rank, for one that legitimately nests.
        pub const fn with_rank(rank: LockRank, value: T) -> Mutex<T> {
            Mutex {
                rank,
                inner: std::sync::Mutex::new(value),
            }
        }

        /// Consumes the lock and returns the protected value.
        pub fn into_inner(self) -> T {
            self.inner
                .into_inner()
                .unwrap_or_else(PoisonError::into_inner)
        }
    }

    impl<T: ?Sized> Mutex<T> {
        /// This lock's rank in the global acquisition order.
        pub fn rank(&self) -> LockRank {
            self.rank
        }

        /// Acquires the lock, blocking until it is available.
        ///
        /// # Panics
        ///
        /// In debug builds, if the rank order is violated. A deadlock caught on
        /// this backend is a deadlock caught for all of them.
        pub fn lock(&self) -> MutexGuard<'_, T> {
            let rank = self.rank.enter();
            MutexGuard {
                inner: self.inner.lock().unwrap_or_else(PoisonError::into_inner),
                _rank: rank,
                _not_send: PhantomData,
            }
        }

        /// Acquires the lock, or returns `None` if it is held.
        pub fn try_lock(&self) -> Option<MutexGuard<'_, T>> {
            let rank = self.rank.enter_nonblocking();
            let inner = match self.inner.try_lock() {
                Ok(inner) => inner,
                Err(TryLockError::Poisoned(poison)) => poison.into_inner(),
                Err(TryLockError::WouldBlock) => return None,
            };
            Some(MutexGuard {
                inner,
                _rank: rank,
                _not_send: PhantomData,
            })
        }

        /// Borrows the value directly, given exclusive access to the lock.
        pub fn get_mut(&mut self) -> &mut T {
            self.inner.get_mut().unwrap_or_else(PoisonError::into_inner)
        }
    }

    impl<T: Default> Default for Mutex<T> {
        fn default() -> Mutex<T> {
            Mutex::new(T::default())
        }
    }

    impl<T> From<T> for Mutex<T> {
        fn from(value: T) -> Mutex<T> {
            Mutex::new(value)
        }
    }

    impl<T: ?Sized + fmt::Debug> fmt::Debug for Mutex<T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            let mut s = f.debug_struct("Mutex");
            s.field("rank", &self.rank);
            match self.try_lock() {
                Some(guard) => s.field("data", &&*guard).finish(),
                None => s.field("data", &"<locked>").finish(),
            }
        }
    }

    /// Exclusive access to the value inside a [`Mutex`], released on drop.
    pub struct MutexGuard<'a, T: ?Sized> {
        inner: std::sync::MutexGuard<'a, T>,
        _rank: RankGuard,
        _not_send: PhantomData<*const ()>,
    }

    impl<T: ?Sized> Deref for MutexGuard<'_, T> {
        type Target = T;

        fn deref(&self) -> &T {
            &self.inner
        }
    }

    impl<T: ?Sized> DerefMut for MutexGuard<'_, T> {
        fn deref_mut(&mut self) -> &mut T {
            &mut self.inner
        }
    }

    impl<T: ?Sized + fmt::Debug> fmt::Debug for MutexGuard<'_, T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            fmt::Debug::fmt(&**self, f)
        }
    }

    /// A reader-writer lock.
    pub struct RwLock<T: ?Sized> {
        rank: LockRank,
        inner: std::sync::RwLock<T>,
    }

    impl<T> RwLock<T> {
        /// A lock at [`LockRank::LEAF`].
        pub const fn new(value: T) -> RwLock<T> {
            RwLock::with_rank(LockRank::LEAF, value)
        }

        /// A lock at an explicit rank.
        pub const fn with_rank(rank: LockRank, value: T) -> RwLock<T> {
            RwLock {
                rank,
                inner: std::sync::RwLock::new(value),
            }
        }

        /// Consumes the lock and returns the protected value.
        pub fn into_inner(self) -> T {
            self.inner
                .into_inner()
                .unwrap_or_else(PoisonError::into_inner)
        }
    }

    impl<T: ?Sized> RwLock<T> {
        /// This lock's rank in the global acquisition order.
        pub fn rank(&self) -> LockRank {
            self.rank
        }

        /// Acquires shared access, blocking until it is available.
        pub fn read(&self) -> RwLockReadGuard<'_, T> {
            let rank = self.rank.enter();
            RwLockReadGuard {
                inner: self.inner.read().unwrap_or_else(PoisonError::into_inner),
                _rank: rank,
                _not_send: PhantomData,
            }
        }

        /// Acquires exclusive access, blocking until it is available.
        pub fn write(&self) -> RwLockWriteGuard<'_, T> {
            let rank = self.rank.enter();
            RwLockWriteGuard {
                inner: self.inner.write().unwrap_or_else(PoisonError::into_inner),
                _rank: rank,
                _not_send: PhantomData,
            }
        }

        /// Acquires shared access, or returns `None` if a writer holds it.
        pub fn try_read(&self) -> Option<RwLockReadGuard<'_, T>> {
            let rank = self.rank.enter_nonblocking();
            let inner = match self.inner.try_read() {
                Ok(inner) => inner,
                Err(TryLockError::Poisoned(poison)) => poison.into_inner(),
                Err(TryLockError::WouldBlock) => return None,
            };
            Some(RwLockReadGuard {
                inner,
                _rank: rank,
                _not_send: PhantomData,
            })
        }

        /// Acquires exclusive access, or returns `None` if the lock is held.
        pub fn try_write(&self) -> Option<RwLockWriteGuard<'_, T>> {
            let rank = self.rank.enter_nonblocking();
            let inner = match self.inner.try_write() {
                Ok(inner) => inner,
                Err(TryLockError::Poisoned(poison)) => poison.into_inner(),
                Err(TryLockError::WouldBlock) => return None,
            };
            Some(RwLockWriteGuard {
                inner,
                _rank: rank,
                _not_send: PhantomData,
            })
        }

        /// Borrows the value directly, given exclusive access to the lock.
        pub fn get_mut(&mut self) -> &mut T {
            self.inner.get_mut().unwrap_or_else(PoisonError::into_inner)
        }
    }

    impl<T: Default> Default for RwLock<T> {
        fn default() -> RwLock<T> {
            RwLock::new(T::default())
        }
    }

    impl<T> From<T> for RwLock<T> {
        fn from(value: T) -> RwLock<T> {
            RwLock::new(value)
        }
    }

    impl<T: ?Sized + fmt::Debug> fmt::Debug for RwLock<T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            let mut s = f.debug_struct("RwLock");
            s.field("rank", &self.rank);
            match self.try_read() {
                Some(guard) => s.field("data", &&*guard).finish(),
                None => s.field("data", &"<locked>").finish(),
            }
        }
    }

    /// Shared access to the value inside an [`RwLock`], released on drop.
    pub struct RwLockReadGuard<'a, T: ?Sized> {
        inner: std::sync::RwLockReadGuard<'a, T>,
        _rank: RankGuard,
        _not_send: PhantomData<*const ()>,
    }

    impl<T: ?Sized> Deref for RwLockReadGuard<'_, T> {
        type Target = T;

        fn deref(&self) -> &T {
            &self.inner
        }
    }

    impl<T: ?Sized + fmt::Debug> fmt::Debug for RwLockReadGuard<'_, T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            fmt::Debug::fmt(&**self, f)
        }
    }

    /// Exclusive access to the value inside an [`RwLock`], released on drop.
    pub struct RwLockWriteGuard<'a, T: ?Sized> {
        inner: std::sync::RwLockWriteGuard<'a, T>,
        _rank: RankGuard,
        _not_send: PhantomData<*const ()>,
    }

    impl<T: ?Sized> Deref for RwLockWriteGuard<'_, T> {
        type Target = T;

        fn deref(&self) -> &T {
            &self.inner
        }
    }

    impl<T: ?Sized> DerefMut for RwLockWriteGuard<'_, T> {
        fn deref_mut(&mut self) -> &mut T {
            &mut self.inner
        }
    }

    impl<T: ?Sized + fmt::Debug> fmt::Debug for RwLockWriteGuard<'_, T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            fmt::Debug::fmt(&**self, f)
        }
    }

    /// One-time initialisation.
    #[derive(Debug)]
    pub struct Once {
        inner: std::sync::Once,
    }

    impl Once {
        /// A `Once` that has not run.
        pub const fn new() -> Once {
            Once {
                inner: std::sync::Once::new(),
            }
        }

        /// Runs `f` unless it has already run, blocking any other thread that
        /// arrives while it is running.
        ///
        /// # Panics
        ///
        /// If called from within its own initialiser, or if a previous
        /// initialiser panicked.
        pub fn call_once(&self, f: impl FnOnce()) {
            self.inner.call_once(f);
        }

        /// Whether the initialiser has run to completion.
        pub fn is_completed(&self) -> bool {
            self.inner.is_completed()
        }
    }

    impl Default for Once {
        fn default() -> Once {
            Once::new()
        }
    }

    /// A job queued for a worker.
    type Job = Box<dyn FnOnce() + Send + 'static>;

    /// The queue, plus the two conditions anyone waits on.
    struct Shared {
        state: std::sync::Mutex<State>,
        /// A job arrived, or the pool is shutting down.
        work: Condvar,
        /// The queue drained and no job is in flight.
        idle: Condvar,
    }

    struct State {
        jobs: VecDeque<Job>,
        running: usize,
        shutdown: bool,
    }

    /// A pool of worker threads.
    ///
    /// Created once, by the embedder or the machine, with a worker count that
    /// comes from the machine configuration. Dropping it drains the queue and
    /// joins every worker, so no job is lost and no thread outlives the pool.
    ///
    /// `workers = 0` is not an error: the pool then runs jobs inline at
    /// [`Pool::submit`], exactly as `single` does. That makes the deterministic
    /// threading mode of `ROADMAP.md` §4.7 reachable on a threaded host without
    /// recompiling for another backend.
    pub struct Pool {
        shared: Arc<Shared>,
        workers: Vec<JoinHandle<()>>,
        requested_workers: usize,
    }

    impl Pool {
        /// Builds a pool with `workers` worker threads.
        ///
        /// If the host refuses a thread, the pool keeps the ones it got; if it
        /// got none, jobs run inline. Failing to start a background worker must
        /// cost throughput, never correctness, so it is not an error the caller
        /// has to handle.
        pub fn new(workers: usize) -> Pool {
            let shared = Arc::new(Shared {
                state: std::sync::Mutex::new(State {
                    jobs: VecDeque::new(),
                    running: 0,
                    shutdown: false,
                }),
                work: Condvar::new(),
                idle: Condvar::new(),
            });

            let mut threads = Vec::with_capacity(workers);
            for index in 0..workers {
                let shared = Arc::clone(&shared);
                let built = thread::Builder::new()
                    .name(std::format!("rsemu-pool-{index}"))
                    .spawn(move || worker(&shared));
                match built {
                    Ok(handle) => threads.push(handle),
                    Err(_) => break,
                }
            }

            Pool {
                shared,
                workers: threads,
                requested_workers: workers,
            }
        }

        /// The number of worker threads actually running.
        ///
        /// Zero means jobs run inline on the submitting thread.
        pub fn workers(&self) -> usize {
            self.workers.len()
        }

        /// What the machine configuration asked for, whether or not the host
        /// provided it.
        pub fn requested_workers(&self) -> usize {
            self.requested_workers
        }

        /// Queues `job` and returns a handle to its result.
        ///
        /// Jobs are dispatched in submission order. Which one *finishes* first
        /// is not defined — that is what having workers means — so anything
        /// order-sensitive comes back through [`Handle::join`] or the event
        /// queue, never through the order results happen to land in.
        pub fn submit<F, T>(&self, job: F) -> Handle<T>
        where
            F: FnOnce() -> T + Send + 'static,
            T: Send + 'static,
        {
            let slot = Arc::new(Slot {
                value: std::sync::Mutex::new(None),
                done: Condvar::new(),
            });

            if self.workers.is_empty() {
                // Inline, and deliberately without `catch_unwind`: with no
                // worker in between, the job's panic is the caller's panic,
                // which is what `single` does too.
                let value = job();
                *slot.value.lock().unwrap_or_else(PoisonError::into_inner) = Some(Ok(value));
                return Handle { slot };
            }

            let filled = Arc::clone(&slot);
            let boxed: Job = Box::new(move || {
                let result = catch_unwind(AssertUnwindSafe(job));
                let mut value = filled.value.lock().unwrap_or_else(PoisonError::into_inner);
                *value = Some(result);
                drop(value);
                filled.done.notify_all();
            });

            let mut state = self
                .shared
                .state
                .lock()
                .unwrap_or_else(PoisonError::into_inner);
            state.jobs.push_back(boxed);
            drop(state);
            self.shared.work.notify_one();

            Handle { slot }
        }

        /// Blocks until the queue is empty and no job is in flight.
        ///
        /// The barrier the stop-the-world protocol waits on (`ROADMAP.md`
        /// §4.7). It says nothing about jobs submitted after it returns.
        pub fn quiesce(&self) {
            let mut state = self
                .shared
                .state
                .lock()
                .unwrap_or_else(PoisonError::into_inner);
            while !state.jobs.is_empty() || state.running > 0 {
                state = self
                    .shared
                    .idle
                    .wait(state)
                    .unwrap_or_else(PoisonError::into_inner);
            }
        }
    }

    impl Drop for Pool {
        fn drop(&mut self) {
            {
                let mut state = self
                    .shared
                    .state
                    .lock()
                    .unwrap_or_else(PoisonError::into_inner);
                state.shutdown = true;
            }
            self.shared.work.notify_all();
            for worker in self.workers.drain(..) {
                // A worker only unwinds if the pool's own bookkeeping panicked;
                // a job's panic is caught and delivered through its `Handle`.
                // Either way, the remaining workers still have to be joined.
                let _ = worker.join();
            }
        }
    }

    impl fmt::Debug for Pool {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            let pending = self
                .shared
                .state
                .try_lock()
                .map(|state| state.jobs.len())
                .ok();
            f.debug_struct("Pool")
                .field("workers", &self.workers.len())
                .field("requested_workers", &self.requested_workers)
                .field("pending", &pending)
                .finish()
        }
    }

    /// A worker: take a job, run it, account for it, repeat until shutdown.
    ///
    /// The queue lock is released before the job runs — the re-entrancy
    /// contract applies to the seam's own code first (`ROADMAP.md` §4.7).
    fn worker(shared: &Arc<Shared>) {
        loop {
            let mut state = shared.state.lock().unwrap_or_else(PoisonError::into_inner);
            let job = loop {
                if let Some(job) = state.jobs.pop_front() {
                    break job;
                }
                if state.shutdown {
                    return;
                }
                state = shared
                    .work
                    .wait(state)
                    .unwrap_or_else(PoisonError::into_inner);
            };
            state.running += 1;
            drop(state);

            job();

            let mut state = shared.state.lock().unwrap_or_else(PoisonError::into_inner);
            state.running -= 1;
            let quiet = state.running == 0 && state.jobs.is_empty();
            drop(state);
            if quiet {
                shared.idle.notify_all();
            }
        }
    }

    struct Slot<T> {
        value: std::sync::Mutex<Option<thread::Result<T>>>,
        done: Condvar,
    }

    /// The result of a submitted job.
    ///
    /// Dropping a handle detaches the job rather than cancelling it: submitted
    /// work always runs. Cancellation, if it is ever wanted, belongs in the job
    /// as a flag it checks, not in the pool.
    pub struct Handle<T> {
        slot: Arc<Slot<T>>,
    }

    impl<T> Handle<T> {
        /// Waits for the job and returns its result.
        ///
        /// # Panics
        ///
        /// Resumes the job's panic here, on the joining thread, the way
        /// `std::thread::JoinHandle` reports one. A panicking job must neither
        /// take the pool down with it nor vanish unnoticed.
        pub fn join(self) -> T {
            let mut value = self
                .slot
                .value
                .lock()
                .unwrap_or_else(PoisonError::into_inner);
            loop {
                if let Some(result) = value.take() {
                    drop(value);
                    return match result {
                        Ok(value) => value,
                        Err(panic) => resume_unwind(panic),
                    };
                }
                value = self
                    .slot
                    .done
                    .wait(value)
                    .unwrap_or_else(PoisonError::into_inner);
            }
        }

        /// Whether the job has finished, without waiting for it.
        pub fn is_finished(&self) -> bool {
            self.slot
                .value
                .try_lock()
                .map(|value| value.is_some())
                .unwrap_or(false)
        }
    }

    impl<T> fmt::Debug for Handle<T> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            f.debug_struct("Handle")
                .field("finished", &self.is_finished())
                .finish()
        }
    }
}

// ---------------------------------------------------------------------------
// The tripwire that used to be here
// ---------------------------------------------------------------------------
//
// There used to be a tripwire here: a `Cell<Option<ThreadId>>` on every
// `single` primitive that panicked when a second thread touched it, on the
// theory that a `single` lock reaching two threads was always a mistake. It
// was deleted along with the theory. `BACKEND` selects `single` for every
// `no_std` build, hosted ones included, and `cargo test` runs that build's
// tests on parallel libtest threads — so a `static` holding one is reached
// from several threads in an entirely ordinary run, and the tripwire's own
// `Cell` was part of the race it claimed to detect. The locks now exclude for
// real (see `single`'s SAFETY comments) and `Global`, below, is where state
// that outlives any one machine goes.

// ---------------------------------------------------------------------------
// Backend selection
// ---------------------------------------------------------------------------
//
// EXTENSION POINT: both remaining backends attach here.
//
//   * `wasm-atomics` claims `target_family = "wasm"` with the threads proposal
//     (`target_feature = "atomics"`), which today falls through to `single`.
//   * `native-raw` claims hosted builds without `std`, which today also fall
//     through to `single` — correct, but serial. It must not be selected before
//     the rank tracker above has thread-local storage on that target.
//
// Adding one is a `cfg` change here plus a module beside `single`; nothing else
// in the crate moves, which is the entire point of the seam.

#[cfg(all(feature = "std", not(target_family = "wasm")))]
pub use native_std::{
    Handle, Mutex, MutexGuard, Once, Pool, RwLock, RwLockReadGuard, RwLockWriteGuard,
};
#[cfg(not(all(feature = "std", not(target_family = "wasm"))))]
pub use single::{
    Handle, Mutex, MutexGuard, Once, Pool, RwLock, RwLockReadGuard, RwLockWriteGuard,
};

// ---------------------------------------------------------------------------
// Process-wide state
// ---------------------------------------------------------------------------

/// A lock for state that no machine owns.
///
/// [`Mutex`] is for state some machine is responsible for, and under `single`
/// it treats an acquisition that would block as a deadlock and says so. That is
/// the right answer for a device register and the wrong one for a `static`: a
/// process-wide table is reachable from every thread in the process — including
/// the ones the test harness makes, which is how this crate found out it needed
/// this type — so contention there is legitimate and the lock has to wait.
///
/// `Global` waits. On a threaded backend it is the backend's blocking
/// acquisition; under `single` it spins on `try_lock`, which costs one
/// uncontended attempt on a target that genuinely has one thread and resolves
/// in nanoseconds where it does not, because everything below is a table
/// insertion that calls nothing outward.
///
/// # Which one to use
///
/// Use `Global` if and only if the value lives in a `static`. Everything else
/// belongs to a machine and wants [`Mutex`], whose panic is a real diagnostic.
/// The test at the bottom of this file enforces the first half of that rule by
/// reading the source: a `static` holding a [`Mutex`] or an [`RwLock`] fails
/// the build rather than waiting to fail one run in three.
///
/// # Re-entrancy
///
/// Taking a `Global` while already holding it is a deadlock on every backend,
/// exactly as it is for `std::sync::Mutex`. Debug builds catch it before it can
/// happen: `Global` enters its [`LockRank`] with the ordered check, so a second
/// acquisition of a rank the thread already holds panics naming both. A `Global`
/// at [`LockRank::UNCHECKED`] gives that up, as anything unchecked does.
pub struct Global<T: ?Sized> {
    rank: LockRank,
    /// The waiting is `Global`'s own business and the rank is entered above, so
    /// the inner lock is [`LockRank::UNCHECKED`]: it exists for the exclusion
    /// and contributes nothing to the ladder.
    inner: Mutex<T>,
}

impl<T> Global<T> {
    /// Process-wide state at [`LockRank::LEAF`] — nests under anything.
    pub const fn new(value: T) -> Global<T> {
        Global::with_rank(LockRank::LEAF, value)
    }

    /// Process-wide state at an explicit rank, for one that legitimately nests.
    pub const fn with_rank(rank: LockRank, value: T) -> Global<T> {
        Global {
            rank,
            inner: Mutex::with_rank(LockRank::UNCHECKED, value),
        }
    }

    /// Consumes the lock and returns the protected value.
    pub fn into_inner(self) -> T {
        self.inner.into_inner()
    }
}

impl<T: ?Sized> Global<T> {
    /// This lock's rank in the global acquisition order.
    pub fn rank(&self) -> LockRank {
        self.rank
    }

    /// Acquires the lock, waiting for another thread if one holds it.
    ///
    /// # Panics
    ///
    /// In debug builds, if the rank order is violated — which is also how a
    /// thread taking the same `Global` twice is caught before it hangs.
    pub fn lock(&self) -> GlobalGuard<'_, T> {
        let rank = self.rank.enter();
        GlobalGuard {
            inner: self.wait(),
            _rank: rank,
            _not_send: PhantomData,
        }
    }

    /// Acquires the lock, or returns `None` if it is held right now.
    ///
    /// The portable re-entrancy probe, as on [`Mutex`]. Note that a `None` here
    /// may mean "another thread has it" rather than "I have it": that is the
    /// difference between process-wide state and a machine's own.
    pub fn try_lock(&self) -> Option<GlobalGuard<'_, T>> {
        let rank = self.rank.enter_nonblocking();
        Some(GlobalGuard {
            inner: self.inner.try_lock()?,
            _rank: rank,
            _not_send: PhantomData,
        })
    }

    /// Borrows the value directly, given exclusive access to the lock.
    pub fn get_mut(&mut self) -> &mut T {
        self.inner.get_mut()
    }

    /// The wait itself, which is the only part that differs by backend.
    #[cfg(all(feature = "std", not(target_family = "wasm")))]
    fn wait(&self) -> MutexGuard<'_, T> {
        self.inner.lock()
    }

    /// Under `single` there is no primitive to block on, and on the targets it
    /// is selected for there is usually nothing to block for: the loop turns
    /// once. Where a second thread does exist — the test harness, threaded
    /// wasm — it holds this for a table lookup and an `Arc` clone, so spinning
    /// costs less than the machinery to avoid it would.
    #[cfg(not(all(feature = "std", not(target_family = "wasm"))))]
    fn wait(&self) -> MutexGuard<'_, T> {
        loop {
            if let Some(guard) = self.inner.try_lock() {
                return guard;
            }
            ::core::hint::spin_loop();
        }
    }
}

impl<T: Default> Default for Global<T> {
    fn default() -> Global<T> {
        Global::new(T::default())
    }
}

impl<T: ?Sized + fmt::Debug> fmt::Debug for Global<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let mut s = f.debug_struct("Global");
        s.field("rank", &self.rank);
        match self.try_lock() {
            Some(guard) => s.field("data", &&*guard).finish(),
            None => s.field("data", &"<locked>").finish(),
        }
    }
}

/// Exclusive access to the value inside a [`Global`], released on drop.
pub struct GlobalGuard<'a, T: ?Sized> {
    inner: MutexGuard<'a, T>,
    _rank: RankGuard,
    _not_send: PhantomData<*const ()>,
}

impl<T: ?Sized> ::core::ops::Deref for GlobalGuard<'_, T> {
    type Target = T;

    fn deref(&self) -> &T {
        &self.inner
    }
}

impl<T: ?Sized> ::core::ops::DerefMut for GlobalGuard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        &mut self.inner
    }
}

impl<T: ?Sized + fmt::Debug> fmt::Debug for GlobalGuard<'_, T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(&**self, f)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::sync::Arc;
    use alloc::vec::Vec;

    // The harness links `std` whatever the feature set says, and two tests here
    // need it whatever the feature set says too: the one that puts a `static
    // Global` under real thread pressure, and the one that reads the crate's
    // own source. Both exist because *tests* are threaded — the mistake they
    // guard is invisible to a build that only ever runs a machine — so naming
    // `std::thread` here is the seam testing itself, not `core/` reaching for
    // the host.
    #[cfg(not(feature = "std"))]
    extern crate std;

    #[cfg(all(feature = "std", not(target_family = "wasm")))]
    use ::core::sync::atomic::{AtomicUsize, Ordering};

    /// Every core type is `Send + Sync` from phase 1 (`ROADMAP.md` §0). This is
    /// a compile-time claim, so the test body is the instantiation.
    fn assert_send_sync<T: Send + Sync>() {}

    #[test]
    fn the_seam_is_send_and_sync_on_every_backend() {
        assert_send_sync::<Global<u64>>();
        assert_send_sync::<single::Mutex<u64>>();
        assert_send_sync::<single::RwLock<u64>>();
        assert_send_sync::<single::Once>();
        assert_send_sync::<single::Pool>();
        assert_send_sync::<single::Handle<u64>>();
        #[cfg(all(feature = "std", not(target_family = "wasm")))]
        {
            assert_send_sync::<native_std::Mutex<u64>>();
            assert_send_sync::<native_std::RwLock<u64>>();
            assert_send_sync::<native_std::Once>();
            assert_send_sync::<native_std::Pool>();
            assert_send_sync::<native_std::Handle<u64>>();
        }
    }

    #[test]
    fn the_selected_backend_names_itself() {
        // A build that cannot say which backend it is cannot report a
        // determinism divergence usefully.
        assert_eq!(BACKEND.is_threaded(), BACKEND != Backend::Single);
        #[cfg(all(feature = "std", not(target_family = "wasm")))]
        assert_eq!(BACKEND, Backend::NativeStd);
        #[cfg(not(all(feature = "std", not(target_family = "wasm"))))]
        assert_eq!(BACKEND, Backend::Single);
    }

    // -- Lock ranks ---------------------------------------------------------

    #[test]
    fn the_ladder_is_strictly_ordered_outermost_first() {
        // The ladder is a claim about the machine's call graph; if the numbers
        // ever stop agreeing with the documented order, the assertion it drives
        // is enforcing something nobody meant.
        let ladder = [
            LockRank::MACHINE,
            LockRank::TOPOLOGY,
            LockRank::SCHED,
            LockRank::BUS,
            LockRank::DEVICE,
            LockRank::WIRE,
            LockRank::POOL,
            LockRank::LEAF,
        ];
        for pair in ladder.windows(2) {
            assert!(
                pair[0] < pair[1],
                "{:?} must precede {:?}",
                pair[0],
                pair[1]
            );
        }
        assert!(LockRank::UNCHECKED < LockRank::MACHINE);
        assert!(ladder.iter().all(|rank| rank.name().is_some()));
        assert!(LockRank::new(0x1234).name().is_none());
    }

    #[cfg(debug_assertions)]
    #[test]
    fn holding_a_rank_forbids_that_rank_and_every_coarser_one() {
        assert_eq!(held_rank(), None);
        let outer = LockRank::TOPOLOGY.enter();

        assert!(violates_lock_order(LockRank::MACHINE), "coarser");
        assert!(violates_lock_order(LockRank::TOPOLOGY), "the same rank");
        assert!(!violates_lock_order(LockRank::DEVICE), "finer");
        assert!(!violates_lock_order(LockRank::UNCHECKED), "exempt");
        assert_eq!(held_rank(), Some(LockRank::TOPOLOGY));

        let inner = LockRank::DEVICE.enter();
        assert_eq!(held_rank(), Some(LockRank::DEVICE));

        // Guards may be released out of order: dropping the *outer* one first
        // must leave the inner rank held, not the other way round.
        drop(outer);
        assert_eq!(held_rank(), Some(LockRank::DEVICE));
        drop(inner);
        assert_eq!(held_rank(), None);
    }

    #[cfg(debug_assertions)]
    #[test]
    fn a_leaf_holds_nothing_under_it_not_even_another_leaf() {
        // The default rank is a leaf precisely so that nesting has to be
        // declared. Two unrelated `Mutex::new` locks held at once is the mistake
        // this catches.
        let leaf = LockRank::LEAF.enter();
        assert!(violates_lock_order(LockRank::LEAF));
        assert!(violates_lock_order(LockRank::DEVICE));
        drop(leaf);
    }

    #[cfg(debug_assertions)]
    #[test]
    fn unchecked_ranks_neither_record_nor_check() {
        let a = LockRank::UNCHECKED.enter();
        assert_eq!(held_rank(), None, "an exempt rank is not recorded");
        let b = LockRank::UNCHECKED.enter();
        assert_eq!(held_rank(), None);
        drop((a, b));
    }

    #[cfg(debug_assertions)]
    #[test]
    fn a_try_lock_records_its_rank_without_checking_the_order() {
        // A non-blocking acquisition cannot join a deadlock cycle, so it is
        // allowed out of order — but a blocking acquire underneath it must
        // still be checked, which requires it to have been recorded.
        let fine = LockRank::DEVICE.enter();
        let out_of_order = LockRank::BUS.enter_nonblocking();
        assert_eq!(held_rank(), Some(LockRank::DEVICE));
        assert!(violates_lock_order(LockRank::BUS));
        drop((out_of_order, fine));
        assert_eq!(held_rank(), None);
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "lock order violation")]
    fn acquiring_out_of_order_panics_naming_both_ranks() {
        let _device = LockRank::DEVICE.enter();
        let _bus = LockRank::BUS.enter();
    }

    #[cfg(not(debug_assertions))]
    #[test]
    fn rank_tracking_costs_nothing_in_release() {
        let _held = LockRank::DEVICE.enter();
        assert_eq!(held_rank(), None);
        assert!(!violates_lock_order(LockRank::MACHINE));
    }

    /// One suite, run against every backend compiled into this build.
    ///
    /// `ROADMAP.md` §4.7 requires `single` and `native-std` to be observably
    /// identical; the only way to hold that line is to write the observations
    /// once. Everything in here is single-threaded on purpose — that is the
    /// intersection where both backends must agree exactly.
    macro_rules! backend_suite {
        ($name:ident, $backend:path) => {
            mod $name {
                use super::assert_send_sync;
                use ::core::sync::atomic::{AtomicUsize, Ordering};
                use alloc::sync::Arc;
                use alloc::vec::Vec;
                use $backend as sync;
                use $crate::core::sync::{LockRank, held_rank, violates_lock_order};

                #[test]
                fn a_guard_is_exclusive_and_releases_on_drop() {
                    let cell = sync::Mutex::new(7u32);
                    {
                        let mut guard = cell.lock();
                        *guard += 1;
                        assert!(
                            cell.try_lock().is_none(),
                            "a held lock must refuse a second acquisition"
                        );
                    }
                    assert_eq!(*cell.lock(), 8);
                    assert_eq!(cell.into_inner(), 8);
                }

                #[test]
                fn try_lock_is_the_portable_reentrancy_probe() {
                    // The re-entrancy contract needs a way to ask "am I already
                    // inside my own critical section?" that neither blocks nor
                    // panics, on every backend. This is it.
                    let state = sync::Mutex::new(0u8);
                    let outer = state.try_lock().expect("uncontended");
                    assert!(state.try_lock().is_none());
                    drop(outer);
                    assert!(state.try_lock().is_some());
                }

                #[test]
                fn rwlock_shares_readers_and_excludes_a_writer() {
                    let cell = sync::RwLock::new(5u32);
                    {
                        // Two readers at once, from one thread: `try_read` so
                        // that the rank check (which forbids re-entering a
                        // blocking acquire) is not what is under test.
                        let a = cell.try_read().expect("first reader");
                        let b = cell.try_read().expect("second reader");
                        assert_eq!(*a + *b, 10);
                        assert!(cell.try_write().is_none(), "a reader excludes a writer");
                    }
                    {
                        let mut w = cell.try_write().expect("uncontended");
                        *w = 6;
                        assert!(cell.try_read().is_none(), "a writer excludes a reader");
                    }
                    assert_eq!(*cell.read(), 6);
                }

                #[test]
                fn once_runs_exactly_once() {
                    let once = sync::Once::new();
                    let mut runs = 0u32;
                    assert!(!once.is_completed());
                    once.call_once(|| runs += 1);
                    once.call_once(|| runs += 1);
                    assert_eq!(runs, 1);
                    assert!(once.is_completed());
                }

                #[test]
                fn joining_in_submission_order_is_deterministic() {
                    // Completion order is a property of the thread count;
                    // *join* order is a property of the caller. Only the second
                    // may be relied on, and it must be identical on every
                    // backend or no state hash ever matches.
                    let pool = sync::Pool::new(4);
                    let handles: Vec<_> = (0..16u64).map(|i| pool.submit(move || i * i)).collect();
                    let results: Vec<u64> = handles.into_iter().map(|h| h.join()).collect();
                    let expected: Vec<u64> = (0..16u64).map(|i| i * i).collect();
                    assert_eq!(results, expected);
                }

                #[test]
                fn dropped_handles_still_run_and_quiesce_waits_for_them() {
                    // Fire-and-forget is the common case for background work
                    // (tier-up, snapshot compression). Dropping the handle
                    // detaches the job; it does not cancel it.
                    let pool = sync::Pool::new(3);
                    let done = Arc::new(AtomicUsize::new(0));
                    for _ in 0..32 {
                        let done = Arc::clone(&done);
                        // The handle dies at the end of this statement.
                        pool.submit(move || done.fetch_add(1, Ordering::SeqCst));
                    }
                    pool.quiesce();
                    assert_eq!(done.load(Ordering::SeqCst), 32);
                }

                #[test]
                fn a_zero_worker_pool_runs_jobs_inline() {
                    // The deterministic threading mode, available without
                    // changing backend: `workers()` reports what really exists,
                    // and zero means "on the caller's thread".
                    let pool = sync::Pool::new(0);
                    assert_eq!(pool.workers(), 0);
                    assert_eq!(pool.requested_workers(), 0);
                    let handle = pool.submit(|| 42u8);
                    assert!(
                        handle.is_finished(),
                        "an inline job is done before submit returns"
                    );
                    assert_eq!(handle.join(), 42);
                }

                #[test]
                fn ranked_locks_nest_in_ladder_order() {
                    let outer = sync::Mutex::with_rank(LockRank::TOPOLOGY, 1u32);
                    let inner = sync::Mutex::with_rank(LockRank::DEVICE, 2u32);
                    let a = outer.lock();
                    let b = inner.lock();
                    assert_eq!(*a + *b, 3);
                    assert_eq!(outer.rank(), LockRank::TOPOLOGY);
                    assert_eq!(inner.rank(), LockRank::DEVICE);
                    #[cfg(debug_assertions)]
                    assert_eq!(held_rank(), Some(LockRank::DEVICE));
                }

                #[cfg(debug_assertions)]
                #[test]
                #[should_panic(expected = "lock order violation")]
                fn nesting_against_the_ladder_panics() {
                    let bus = sync::Mutex::with_rank(LockRank::BUS, 0u32);
                    let sched = sync::Mutex::with_rank(LockRank::SCHED, 0u32);
                    let _inner = bus.lock();
                    let _outer = sched.lock();
                }

                #[cfg(debug_assertions)]
                #[test]
                #[should_panic(expected = "lock order violation")]
                fn recursive_locking_is_caught_as_an_order_violation() {
                    // Uniform across backends: `single` would panic on the flag
                    // and `native-std` would deadlock, but the rank check fires
                    // first on both, so the diagnosis is the same everywhere.
                    let device = sync::Mutex::with_rank(LockRank::DEVICE, 0u32);
                    let _first = device.lock();
                    let _second = device.lock();
                }

                #[test]
                fn a_lock_releases_its_rank_when_the_guard_drops() {
                    let device = sync::Mutex::with_rank(LockRank::DEVICE, 0u32);
                    {
                        let _held = device.lock();
                        assert!(violates_lock_order(LockRank::DEVICE) == cfg!(debug_assertions));
                    }
                    assert!(!violates_lock_order(LockRank::DEVICE));
                    assert_eq!(held_rank(), None);
                }

                #[test]
                fn the_state_hash_does_not_depend_on_the_worker_count() {
                    // `ROADMAP.md` §0: whatever the thread count, background
                    // work never changes guest-visible results. A workload
                    // whose answer moved with `workers` would be reporting a
                    // race, and this is the cheapest place to notice one.
                    let baseline = state_hash(0);
                    for workers in [1, 2, 4, 7] {
                        assert_eq!(state_hash(workers), baseline, "with {workers} workers");
                    }
                }

                #[test]
                fn types_are_send_and_sync() {
                    assert_send_sync::<sync::Mutex<u64>>();
                    assert_send_sync::<sync::RwLock<u64>>();
                    assert_send_sync::<sync::Pool>();
                }

                /// A miniature machine: shared state, background jobs, and a
                /// one-time init, reduced to a state hash.
                ///
                /// The equivalence test below runs this under both backends and
                /// compares. Nothing in it may depend on completion order —
                /// that is exactly the discipline `ROADMAP.md` §4.7 demands of
                /// a device that wants to keep its state hash.
                pub(super) fn state_hash(workers: usize) -> u64 {
                    let ready = sync::Once::new();
                    let seed = sync::RwLock::with_rank(LockRank::MACHINE, 0u64);
                    ready.call_once(|| *seed.write() = 0x9e37_79b9_7f4a_7c15);
                    let base = *seed.read();

                    let pool = sync::Pool::new(workers);
                    let accumulator = Arc::new(sync::Mutex::with_rank(LockRank::DEVICE, 0u64));

                    let handles: Vec<_> = (0..24u64)
                        .map(|i| {
                            let accumulator = Arc::clone(&accumulator);
                            pool.submit(move || {
                                let step = base.wrapping_mul(i.wrapping_add(1)) ^ (i << 7);
                                // Commutative under the lock, so the result does
                                // not depend on which worker got there first.
                                let mut acc = accumulator.lock();
                                *acc = acc.wrapping_add(step);
                                step
                            })
                        })
                        .collect();

                    let mut hash = base;
                    for handle in handles {
                        hash = hash.rotate_left(7) ^ handle.join();
                    }
                    pool.quiesce();
                    hash ^ *accumulator.lock()
                }
            }
        };
    }

    backend_suite!(single_backend, crate::core::sync::single);
    #[cfg(all(feature = "std", not(target_family = "wasm")))]
    backend_suite!(native_std_backend, crate::core::sync::native_std);

    /// The §4.7 requirement, in miniature: the same workload, the same answer,
    /// under both backends in the same binary.
    #[cfg(all(feature = "std", not(target_family = "wasm")))]
    #[test]
    fn single_and_native_std_agree_on_the_state_hash() {
        for workers in [0, 1, 4] {
            assert_eq!(
                single_backend::state_hash(workers),
                native_std_backend::state_hash(workers),
                "backends diverged with {workers} workers"
            );
        }
    }

    // -- `single`-specific reference semantics ------------------------------

    #[test]
    #[cfg_attr(debug_assertions, should_panic(expected = "lock order violation"))]
    #[cfg_attr(not(debug_assertions), should_panic(expected = "recursive lock"))]
    fn single_reports_a_recursive_lock_instead_of_hanging() {
        // The reason `single` is the reference: a threaded backend can only
        // hang here, and a hang is not a test failure anyone can read.
        let device = single::Mutex::new(0u32);
        let _first = device.lock();
        let _second = device.lock();
    }

    #[test]
    #[should_panic(expected = "recursive lock")]
    fn single_catches_a_job_that_re_enters_its_submitters_critical_section() {
        // The re-entrancy contract, enforced: the job runs inline at `submit`,
        // so taking the lock the submitter still holds is caught here rather
        // than becoming a rare deadlock on a threaded host. `UNCHECKED` keeps
        // the rank machinery out of it so the *inline execution* is what the
        // test is about.
        let pool = single::Pool::new(0);
        let state = Arc::new(single::Mutex::with_rank(LockRank::UNCHECKED, 0u32));
        let held = state.lock();
        let inner = Arc::clone(&state);
        let _ = pool.submit(move || *inner.lock());
        drop(held);
    }

    #[test]
    fn single_serialises_submissions_completely() {
        // Under `single` the pool is a strict evaluator: submission order *is*
        // execution order, which is what makes it the deterministic runner.
        let pool = single::Pool::new(8);
        assert_eq!(pool.workers(), 0, "`single` has no workers to report");
        assert_eq!(pool.requested_workers(), 8, "but it remembers the request");
        let order = Arc::new(single::Mutex::new(Vec::new()));
        let mut handles = Vec::new();
        for i in 0..8u64 {
            let order = Arc::clone(&order);
            handles.push(pool.submit(move || {
                order.lock().push(i);
                i
            }));
        }
        assert_eq!(*order.lock(), (0..8u64).collect::<Vec<_>>());
        assert!(handles.iter().all(single::Handle::is_finished));
    }

    // -- `native-std`-specific threading ------------------------------------

    #[cfg(all(feature = "std", not(target_family = "wasm")))]
    mod threaded {
        use super::*;
        use std::panic::{AssertUnwindSafe, catch_unwind};
        use std::thread;

        #[test]
        fn a_mutex_actually_excludes_across_worker_threads() {
            let pool = native_std::Pool::new(4);
            assert_eq!(pool.workers(), 4);
            let counter = Arc::new(native_std::Mutex::new(0u64));
            let handles: Vec<_> = (0..8)
                .map(|_| {
                    let counter = Arc::clone(&counter);
                    pool.submit(move || {
                        for _ in 0..2_000 {
                            *counter.lock() += 1;
                        }
                    })
                })
                .collect();
            for handle in handles {
                handle.join();
            }
            assert_eq!(*counter.lock(), 16_000);
        }

        #[test]
        fn jobs_run_off_the_submitting_thread_when_there_are_workers() {
            let here = thread::current().id();
            let pool = native_std::Pool::new(2);
            assert_ne!(pool.submit(move || thread::current().id()).join(), here);

            // ...and on it when there are none, which is the whole reason
            // `workers() == 0` is a supported configuration rather than a bug.
            let inline = native_std::Pool::new(0);
            assert_eq!(inline.submit(move || thread::current().id()).join(), here);
        }

        #[test]
        fn a_panicking_job_is_reported_at_join_and_the_pool_survives() {
            let pool = native_std::Pool::new(1);
            let handle = pool.submit(|| panic!("job exploded"));
            let caught = catch_unwind(AssertUnwindSafe(move || handle.join()));
            assert!(
                caught.is_err(),
                "the panic must surface at join, not vanish"
            );
            // The worker that ran it is still there.
            assert_eq!(pool.submit(|| 5u32).join(), 5);
        }

        #[cfg(debug_assertions)]
        #[test]
        fn rank_tracking_is_per_thread() {
            // Two threads holding the same rank at once is normal; only nesting
            // *within* a thread is a violation. A global tracker would fail
            // this, which is why the std path uses thread-local storage.
            let outer = LockRank::MACHINE.enter();
            let pool = native_std::Pool::new(1);
            let observed = pool
                .submit(|| {
                    let inner = LockRank::MACHINE.enter();
                    let seen = held_rank();
                    drop(inner);
                    seen
                })
                .join();
            assert_eq!(observed, Some(LockRank::MACHINE));
            assert_eq!(held_rank(), Some(LockRank::MACHINE));
            drop(outer);
        }

        #[test]
        fn quiesce_waits_for_work_already_in_flight() {
            let pool = native_std::Pool::new(2);
            let done = Arc::new(AtomicUsize::new(0));
            for _ in 0..8 {
                let done = Arc::clone(&done);
                // The handle dies at the end of this statement, so `quiesce` is
                // the only thing left that knows the job is outstanding.
                pool.submit(move || {
                    // Busy work rather than a sleep: the seam has no clock, and
                    // `core/` may not read the host's one (CLAUDE.md).
                    let mut acc = 0u64;
                    for i in 0..200_000u64 {
                        acc = acc.wrapping_add(i);
                    }
                    ::core::hint::black_box(acc);
                    done.fetch_add(1, Ordering::SeqCst);
                });
            }
            pool.quiesce();
            assert_eq!(done.load(Ordering::SeqCst), 8);
        }
    }

    // -- process-wide state ------------------------------------------------

    #[test]
    fn a_global_guard_is_exclusive_and_releases_on_drop() {
        let cell = Global::new(7u32);
        {
            let mut guard = cell.lock();
            *guard += 1;
            assert!(
                cell.try_lock().is_none(),
                "a held lock must refuse a second acquisition"
            );
        }
        assert_eq!(*cell.lock(), 8);
        assert_eq!(cell.rank(), LockRank::LEAF);
        let mut owned = Global::with_rank(LockRank::MACHINE, 1u8);
        assert_eq!(owned.rank(), LockRank::MACHINE);
        *owned.get_mut() = 2;
        assert_eq!(owned.into_inner(), 2);
        assert_eq!(cell.into_inner(), 8);
    }

    /// Process-wide state under the pressure that produced [`Global`].
    ///
    /// This is the workload that broke: `cargo test --no-default-features
    /// --features machine-nes` selects `single`, libtest runs its tests on
    /// parallel threads, and the `static Mutex` behind `dev::nes::pads` was
    /// reached from two of them about one run in three. It surfaced as a tidy
    /// "recursive lock" panic, but the flag reporting it was a `Cell<bool>`
    /// read and written from two threads, so the diagnostic was itself the
    /// undefined behaviour. Same shape here, against the type that is allowed
    /// to have it, on whichever backend this build selected.
    #[test]
    fn a_static_global_survives_the_whole_harness_hammering_it() {
        use alloc::collections::BTreeMap;
        use alloc::format;
        use alloc::string::String;

        // Deliberately a `static`: a `Global` on the stack would prove nothing,
        // since the bug is exactly that a `static` outlives and escapes the one
        // thread that made it.
        static TABLE: Global<BTreeMap<String, u64>> = Global::new(BTreeMap::new());

        const THREADS: u64 = 8;
        const KEYS: u64 = 250;

        std::thread::scope(|scope| {
            for _ in 0..THREADS {
                scope.spawn(|| {
                    for key in 0..KEYS {
                        // Every thread visits every key, so they collide on the
                        // table rather than politely partitioning it.
                        let mut table = TABLE.lock();
                        *table.entry(format!("k{key}")).or_insert(0) += 1;
                    }
                });
            }
        });

        let table = TABLE.lock();
        assert_eq!(table.len() as u64, KEYS);
        assert!(
            table.values().all(|&seen| seen == THREADS),
            "every key must have been incremented once per thread"
        );
    }
}
