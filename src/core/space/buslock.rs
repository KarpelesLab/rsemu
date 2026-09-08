//! The **bus lock**: exclusion held across the read *and* the write of one
//! indivisible read-modify-write, and a full barrier at each end of it.
//!
//! Two guarantees, not one, and the second is the one that is easy to forget —
//! "a bus lock is a barrier" below has why x86 needs it and what it cost to
//! notice that a mutex does not supply it.
//!
//! # Why a space owns one, and why it is not the exclusive monitor
//!
//! [`ExclusiveMonitor`](super::ExclusiveMonitor) serves the load-reserved /
//! store-conditional pair, and everything cheap about it comes from the pair
//! being *optimistic*: a reservation answers "has anyone written since I
//! claimed it", both architectures license it to fail spuriously, and the
//! guest's own retry loop absorbs the failure. That licence is what buys a
//! lock-free table consulted by one acquire load on every store in the
//! machine.
//!
//! x86 has no pair. `LOCK CMPXCHG`, `LOCK XADD` and the implicitly locked
//! `XCHG` with a memory operand (*Intel SDM* volume 2, `XCHG`) are
//! **pessimistic and unconditional**: the read-modify-write is simply required
//! not to be interleaved with, there is no status flag with which to report a
//! failure and no retry loop in the guest to catch one. Built on a monitor
//! that may clear spuriously, `LOCK XADD` would not retry — it would return a
//! wrong answer.
//!
//! So the primitive is a lock, and it belongs to the
//! [`AddressSpace`](super::AddressSpace) for the
//! same reason the monitor does: a space is one coherence domain, and that is
//! the only thing the two mechanisms have to agree about. They do not know
//! about each other and do not have to — a locked write still leaves through
//! `SpaceView::write_span`, so it breaks a sibling's reservation on the way
//! past, and an x86 core sharing a space with an AArch64 one gets that without
//! ever registering a monitor slot.
//!
//! # A bus lock is a barrier, not only an exclusion
//!
//! Exclusion is the half that is easy to see and it is not the whole
//! requirement. A `LOCK`-prefixed instruction is architecturally a **full
//! barrier**: *Intel SDM* volume 3 §9.1.2 and §9.2.5 put locked operations in
//! one total order and make them serialising with respect to this processor's
//! other loads and stores, so nothing before one may be reordered past it and
//! nothing after one may be reordered ahead of it. Mutual exclusion between
//! lock holders is the first property. It is not the second, and for a while
//! this object had only the first.
//!
//! A mutex supplies acquire on the way in and release on the way out, and both
//! are deliberately **one-way**: an access may move *into* a critical section
//! and may not move out of it. That leaves the store-then-load direction open
//! straight through the middle. Take Linux's `smp_mb()`, which on x86-64 is
//! not `MFENCE` but `lock addl $0, -4(%rsp)`:
//!
//! ```text
//! guest CPU 0                  guest CPU 1
//! mov dword [x], 1             mov dword [y], 1
//! lock addl [rsp-4], 0         lock addl [rsp-4], 0
//! mov eax, [y]                 mov ebx, [x]
//! ```
//!
//! `eax == 0 && ebx == 0` is the store-buffer outcome x86-TSO forbids and the
//! outcome those two locked instructions are in the program to forbid. With
//! exclusion alone the emulator permits it: the guest's store to `x` is a
//! relaxed host atomic that may sink past the acquire *into* the critical
//! section, the guest's load of `y` is a relaxed host atomic that may hoist
//! above the release into the same critical section, and once both are inside,
//! nothing orders a store before a later load. Both host threads do it at once
//! and both loads come back zero.
//!
//! Note what the exclusion contributed there: nothing. `-4(%rsp)` is each
//! core's own stack, so the two masters never address the same word and the
//! mutex never blocks. The single property that program wanted from the
//! instruction was the one the object did not have.
//!
//! ## Why it did not show up
//!
//! Because on an x86-64 host the barrier was already there by accident, twice
//! over. `Mutex::lock` is a compare-and-swap on both backends — `sync`'s
//! `single` claims with `compare_exchange(Acquire, Relaxed)`, `native_std`
//! hands off to `std` — and a compare-and-swap on x86-64 is `lock cmpxchg`,
//! which is a full barrier by the same §9.2.5 the guest's own `LOCK` relies
//! on. `RamStore::mark_dirty`'s `fetch_or` was a second one, for the reason
//! `core::sync` records — until it stopped being unconditional and that half
//! of the cover went away, which changes nothing here because the fences below
//! were never relying on it. Both were properties of the *host's* instruction
//! set rather than of anything written here: lower that same `compare_exchange`
//! for AArch64 and it is `ldaxr`/`stxr`, an acquire and nothing more, which
//! orders a prior store against a later load not at all.
//!
//! Measured rather than argued — the store-buffer litmus, 200 000 rounds on an
//! x86-64 host, with nothing between the store and the load, with a
//! [`fence`](crate::core::sync::fence) at `SeqCst`, and with a mutex critical
//! section over a word the other thread never touches: tens of forbidden
//! outcomes for the first, **zero** for the other two, run after run. The
//! mutex is indistinguishable from the fence *here*, which is exactly why the
//! gap survived being looked at, and why no test on an x86-64 host can gate
//! it. `tests/memory_model_litmus.rs` is the same instrument at three levels.
//!
//! ## What is emitted, and why two of them
//!
//! A `SeqCst` fence once the mutex is held, in `BusLock::claim`, and a
//! second in [`BusLockGuard`]'s `Drop` before the mutex is given back. One at
//! each end, because either alone leaves half the requirement:
//!
//! * the fence at acquire puts every access this master made *before* the
//!   locked instruction ahead of the locked read-modify-write;
//! * the fence at release puts the locked read-modify-write ahead of every
//!   access this master makes *after* it. Without it a subsequent load may
//!   still hoist above the locked write — the release store does not stop it,
//!   for the same one-way reason the acquire does not stop the store — and
//!   another observer could see that load's value before the write it is
//!   architecturally ordered behind.
//!
//! The guarantee, stated once: **between the fence a bus lock takes and the
//! fence it gives back, this master's accesses are globally ordered against
//! everything it did before and everything it does after** — which is what
//! *Intel SDM* volume 3 §9.1.2 requires of a locked instruction, and is the
//! reason the guard is taken around the whole instruction rather than around
//! either access.
//!
//! What it is *not*: this fences one master's own accesses around its own
//! locked instruction. It does not give a sibling core's *plain* accesses
//! x86-TSO's ordering among themselves — emulating TSO on a weakly ordered
//! host is a per-access question about [`RamStore`](super::RamStore), not a
//! question about this lock, and `docs/techniques/memory-models.md` is where
//! it belongs.
//!
//! # What it costs, and where
//!
//! Nothing at all on an ordinary access. Nothing in this file is reachable
//! from the read or write path; a core that never executes a locked
//! instruction never touches it. That is the whole reason a *lock* is
//! affordable here and was not affordable in the monitor: the monitor sits on
//! the store path, and this sits on the `LOCK` path. The difference is four
//! orders of magnitude: `docs/platforms/pc64.md` measured **0.21 M**
//! `LOCK`-prefixed instructions in nine hundred guest seconds of a `6.6.67`
//! Linux boot, against the billions of stores the same run makes.
//!
//! The two fences ride the same path and are charged the same way. A `SeqCst`
//! fence prices at a couple of nanoseconds against a store
//! (`tests/memory_model_costs.rs`), and the acquire they bracket already costs
//! ~13 ns per locked instruction, so the barrier is a single-digit percentage
//! of an operation a guest performs a fifth of a million times in fifteen
//! minutes. It is the cheapest thing in this file and it is the only part of
//! it the guest's own correctness argument depends on.
//!
//! # The rank, and why it cannot deadlock
//!
//! [`LockRank::BUS_LOCK`], strictly between `BUS` and `DEVICE`. `sync`'s
//! ladder entry has the argument for both bounds; what is upheld *here* is the
//! invariant that makes them sufficient:
//!
//! > A bus lock is taken by a bus master at an instruction boundary, **before
//! > that instruction has issued any access**, and released when the
//! > instruction ends.
//!
//! Nothing below `BUS_LOCK` in the ladder is held at the moment it is taken,
//! so the classic inversion — one master holding the bus lock and waiting for
//! a device lock while another master holds that device lock and waits for the
//! bus lock — has no second half: the master waiting for the bus lock is
//! between instructions and holds no device lock. A device never takes a bus
//! lock at all; only a master issuing a locked transaction does.
//!
//! Re-entrancy is caught rather than survived. Under the `single` backend a
//! second blocking acquire on one thread panics naming the rank, which is what
//! a locked access that somehow reached a second locked access would be.
//!
//! # What it does *not* make atomic
//!
//! Locked against locked, and that is the honest claim. A **plain** store by
//! another observer can still land between the read and the write of a locked
//! read-modify-write, where hardware would have held it off — and between a
//! store-conditional's consultation of the exclusive monitor and its own write,
//! which is the same window reached through the other object. Devices and DMA
//! are the same case, left the same way.
//!
//! One residual, then, not two, and it is written down once — the interleaving,
//! what the manuals say about whether an affected guest is well defined, the
//! measured rate and the measured price of closing it — in
//! `docs/techniques/memory-models.md`, "Not kept: a plain store against another
//! master's atomic". [`ExclusiveMonitor`](super::ExclusiveMonitor)'s "What is
//! left" is the same pointer from the other end.
//!
//! **Two claims this section used to make are withdrawn**, because a later
//! round measured them and they were false.
//!
//! * "No real guest asks for it, because a plain store racing a locked
//!   read-modify-write on the same word is a data race in the guest's own
//!   terms." It is not a data race in anyone's terms. *Intel SDM* volume 3
//!   §9.1.2.2 gives the `LOCK#` signal "exclusive use of any shared memory
//!   while the signal is asserted" — against every other access, not only
//!   against another locked one — and RISC-V's atomicity axiom and Arm's
//!   global-monitor guarantee say the same for the pair. At the source level
//!   `AtomicU64::store(v, Relaxed)` lowers to a plain `mov`/`str`/`sd` and
//!   `compare_exchange_weak` lowers to a locked instruction or an `LR`/`SC`
//!   loop, and racing the two on one atomic is well-defined C++ and Rust that
//!   the model requires to work.
//! * That the residual was rare. In the reproducer the doc records it costs
//!   **112 to 2 416 updates of 814 to 4 786 committed store-conditionals**, in
//!   twenty of twenty sequential and six of six parallel runs.
//!
//! What survives unchanged is the *decision*: closing it means every store in
//! the machine taking this lock, which measures at +144% on the store path and
//! a ninefold collapse of aggregate store throughput at eight masters, and that
//! is not a price the emulator pays for a mode nothing selects by default.
//!
//! The shape that would close it without a cost on the store path is to compose
//! the two objects — reserve the granule in the monitor before the read, check
//! it still holds before the write, reissue when it does not — and it is
//! written down here rather than built because a reissue re-runs the side
//! effects of an MMIO operand, which is a worse failure than the one it fixes.
//!
//! ## Which threading mode can see it
//!
//! Not [`ThreadingMode::Deterministic`], and the argument is structural rather
//! than statistical. That mode runs every runnable on one host thread, so the
//! finest grain it can interleave at is one whole instruction: between a locked
//! instruction's read and its write, *nothing executes*. `Parallel` is where
//! the residual lives, and it is opt-in — no machine file in the tree selects
//! it; `--threading parallel` on the command line does. Under `Accel` the
//! question does not arise, because the host's silicon performs the guest's
//! read-modify-write and this object is never reached.
//!
//! The JIT does not widen any of this, and it is worth checking rather than
//! assuming, because `IrHost::spent` can leave a block part-way at an
//! instruction boundary. **No core lifts an atomic instruction into a block at
//! all**: the x86 frontend refuses a `LOCK` prefix and `XCHG` with a memory
//! operand outright (`cpu::x86::lift`), the AArch64 one excludes the exclusives
//! and the LSE atomics, and the RISC-V one excludes the whole `A` extension. So
//! there is no block for a budget to leave in the middle of an atomic, and
//! every locked read-modify-write in the tree runs through the interpreter with
//! this lock held.
//!
//! There is exactly one way the deterministic claim can fail, and it is worth
//! writing down because it is not obvious: a **lazily advanced device** is
//! caught up from inside the access that dispatches to it (`ROADMAP.md` §4.2)
//! and `LazyDevice::advance_to` is free to touch its own bus. So a locked
//! read-modify-write whose operand is *MMIO* can have another master's write
//! land inside it even on one thread. One whose operand is **RAM** cannot: a
//! RAM access dispatches to no device, so there is nowhere for a catch-up to
//! happen. No device in the tree writes the word a locked instruction is
//! addressing while serving that instruction's own access, so this is a shape
//! to keep in mind rather than a defect that has been observed.
//!
//! ## The sharper form, with a number
//!
//! Stated as a lost update the residual takes a detector to catch — the value
//! left behind looks like one a legal ordering could have produced, and it
//! takes a writer publishing each value only after its store has *returned*,
//! plus a read-back, to prove otherwise. The doc named above builds that
//! detector. There is also a second form that needs none of it.
//! [`RamStore`](super::RamStore) is a
//! `Vec<AtomicU8>` and every access to it is a **byte loop**, so a plain
//! four-byte store is four independent stores — and a locked instruction's read
//! that overlaps one comes back holding a *mixture of the old and the new
//! word*, a value that was never in memory. All three architectures forbid that
//! outright for a naturally aligned access (*Intel SDM* volume 3 §9.1.1; ARM
//! DDI 0487 B2.2.1; RISC-V Unprivileged ISA §1.4).
//!
//! `tests/smp_single_copy_atomicity.rs` is the instrument. Sixty thousand
//! `LOCK XADD`s racing sixty thousand alternating plain stores, two host
//! threads, debug: **90 to 138 torn reads**, with the bus taken all sixty
//! thousand times. The lock was working; the plain store went through it
//! anyway, because a plain store does not ask. The same file runs the same two
//! programs on one host thread with a randomised instruction-by-instruction
//! schedule — finer than any quantum the scheduler hands out — and tears zero
//! times while demonstrably seeing the writer's other value thousands of times.
//!
//! Note what that measurement does *not* say. The tearing is
//! [`RamStore`](super::RamStore)'s byte loop, not this lock's doing, and
//! `space::store`'s "What per-byte atomicity is not" has the cost of the ways
//! to remove it — three shapes, all re-derivable from
//! `tests/memory_model_costs.rs`, none of them taken. Removing it would leave
//! the residual in its lost-update form, which is what this section originally
//! described and what would still be here.
//!
//! It also does not say the tearing belongs to `LOCK`, or to x86. Every
//! naturally aligned load of two bytes or more, on every architecture in the
//! tree, can come back torn against a racing store through this path. What is
//! x86's alone is that the torn read is the read half of an instruction the
//! architecture requires to be indivisible, which is what makes it *visible*
//! here rather than merely present.
//!
//! ## The cheap approximation, and why it is refused
//!
//! The obvious way to have it both ways is to put [`BusLock::held`] on the
//! store path — one relaxed load, the same shape as the monitor's fast path —
//! and take the lock only when it says somebody has the bus. It looks free and
//! it is not correct: a store that read the flag as false while the locked
//! instruction was in the act of claiming the bus proceeds anyway, so its bytes
//! can still land between the read and the write. Making the check and the
//! store one indivisible act means the store taking the lock, which is the cost
//! the design exists to avoid, or a shared/exclusive pair whose reader side is
//! a read-modify-write and measures like one (`tests/memory_model_costs.rs`:
//! ~4 ns, against ~1 ns for the store it would be guarding).
//!
//! So the choice is not "cheap fix or no fix". It is between a documented
//! boundary and a change that makes the same defect several orders of magnitude
//! rarer without removing it — which is strictly worse, because what is left is
//! a bug nobody can reproduce.
//!
//! # It is not state
//!
//! A bus lock is never held across an instruction boundary, so a snapshot
//! taken at a safe point can never observe one held. There is nothing to
//! serialize and no chunk version moves.
//!
//! [`ThreadingMode::Deterministic`]: crate::core::sched::ThreadingMode::Deterministic

use core::fmt;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::core::sync::{LockRank, Mutex, MutexGuard, fence};

/// One coherence domain's bus lock.
///
/// The `buslock` module documentation has the design, the rank argument and
/// the limits. Reached as [`AddressSpace::bus_lock`](super::AddressSpace::bus_lock).
pub struct BusLock {
    /// The exclusion itself. `()`, because what is protected is not a value in
    /// this process — it is every byte the space can reach, for the length of
    /// one guest instruction.
    inner: Mutex<()>,
    /// Whether a locked transaction is in flight, readable without acquiring.
    ///
    /// A diagnostic, and what lets a test pin the *span* the guard covers
    /// rather than only the fact that it was taken. Not a substitute for the
    /// lock: reading it says what was true a moment ago.
    held: AtomicBool,
    /// How many locked transactions this space has served.
    ///
    /// A statistic. Nothing in the emulation path reads it and it is not
    /// serialized — but it is the only way a single-threaded test can assert
    /// that an instruction took the lock at all, which is what separates
    /// "`LOCK` is honoured" from "`LOCK` is ignored and the test passed
    /// anyway".
    taken: AtomicU64,
}

impl fmt::Debug for BusLock {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BusLock")
            .field("held", &self.held.load(Ordering::Relaxed))
            .field("taken", &self.taken.load(Ordering::Relaxed))
            .finish()
    }
}

impl Default for BusLock {
    fn default() -> Self {
        BusLock::new()
    }
}

impl BusLock {
    /// An unheld bus lock.
    #[must_use]
    pub fn new() -> BusLock {
        BusLock {
            inner: Mutex::with_rank(LockRank::BUS_LOCK, ()),
            held: AtomicBool::new(false),
            taken: AtomicU64::new(0),
        }
    }

    /// Take the bus for one indivisible read-modify-write.
    ///
    /// Call it at an instruction boundary, before the instruction has issued
    /// any access, and hold it until the instruction ends — the `buslock`
    /// module documentation says why that ordering is what makes the rank
    /// sound, and why holding it across the *whole* instruction rather than
    /// around either access is what makes the two fences land where a locked
    /// instruction's architectural barrier lands.
    ///
    /// Emits a `SeqCst` fence once the bus is held, and the guard emits
    /// another before giving it back. Exclusion alone is not what x86 asks of
    /// a `LOCK` prefix; see the module's "a bus lock is a barrier".
    ///
    /// # Panics
    ///
    /// If the caller already holds a lock of this rank or coarser, which the
    /// rank check reports in debug builds. Under the `single` backend a
    /// re-entrant acquire panics in release builds too, rather than hanging.
    pub fn acquire(&self) -> BusLockGuard<'_> {
        let inner = self.inner.lock();
        self.claim();
        BusLockGuard {
            lock: self,
            _inner: inner,
        }
    }

    /// [`acquire`](BusLock::acquire), but `None` rather than waiting.
    ///
    /// For a caller with an answer for "somebody else has the bus" — a
    /// monitor, a test. A failed try-lock cannot join a deadlock cycle, so it
    /// is order-exempt.
    #[must_use]
    pub fn try_acquire(&self) -> Option<BusLockGuard<'_>> {
        let inner = self.inner.try_lock()?;
        self.claim();
        Some(BusLockGuard {
            lock: self,
            _inner: inner,
        })
    }

    /// The bookkeeping both acquisitions share, once the mutex is ours.
    ///
    /// The fence is the leading half of the barrier, not bookkeeping: it is
    /// what puts every access this master made *before* the locked instruction
    /// ahead of the locked read-modify-write. The mutex's own acquire cannot,
    /// because acquire is one-way — see the module's "a bus lock is a barrier".
    fn claim(&self) {
        fence(Ordering::SeqCst);
        self.held.store(true, Ordering::Release);
        self.taken.fetch_add(1, Ordering::Relaxed);
    }

    /// Whether *a* locked transaction is in flight on this space.
    ///
    /// Not *this* one: a handler reached by an ordinary access sees `true`
    /// while another master holds the bus, so this answers "is the bus busy",
    /// which is a diagnostic and a test hook rather than an identity. A device
    /// that needs to know whether the access it is serving is the locked one
    /// would need that on [`MemAttrs`](super::MemAttrs), and nothing has asked
    /// for it yet.
    #[must_use]
    pub fn held(&self) -> bool {
        self.held.load(Ordering::Acquire)
    }

    /// How many locked transactions this space has served since it was built.
    #[must_use]
    pub fn taken(&self) -> u64 {
        self.taken.load(Ordering::Relaxed)
    }
}

/// The bus, held. Released on drop.
///
/// Not `Send`: a lock guard belongs to the thread that took it, and the whole
/// point is that the instruction which took it is the one that gives it back.
pub struct BusLockGuard<'a> {
    lock: &'a BusLock,
    _inner: MutexGuard<'a, ()>,
}

impl fmt::Debug for BusLockGuard<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BusLockGuard").finish_non_exhaustive()
    }
}

impl Drop for BusLockGuard<'_> {
    /// Fences, then clears the flag, then — as the struct's own fields drop
    /// after its `Drop` body — releases the mutex.
    ///
    /// Both orderings are load-bearing. The fence is the trailing half of the
    /// barrier: it is what puts the locked read-modify-write ahead of every
    /// access this master makes *after* the instruction, which the mutex's
    /// release store does not do, because release is one-way and a later load
    /// may hoist straight past it. Clearing the flag ahead of the mutex is so
    /// that no observer ever sees the bus free and still flagged as held.
    fn drop(&mut self) {
        fence(Ordering::SeqCst);
        self.lock.held.store(false, Ordering::Release);
    }
}

/// # Why nothing below tests the barrier
///
/// Because on an x86-64 host nothing can, and a test that cannot fail is worse
/// than an admission that one is missing.
///
/// The instrument would be the store-buffer litmus with a bus lock between the
/// store and the load. Run at 200 000 rounds on this host it returns zero
/// forbidden outcomes with the fences and zero without them, because the
/// mutex's own `lock cmpxchg` is already a full barrier here — the accident
/// the module documentation names. The same litmus over two bare relaxed
/// atomics does produce the outcome, tens of times per 200 000 rounds, so the
/// instrument is sensitive; it is the subject that is immune.
///
/// So the fences are gated by the *architecture* rather than by a test on this
/// machine, and the regression net for them is a weakly ordered host. CI has
/// one: the `aarch64 (weak memory)` job on `ubuntu-24.04-arm` runs
/// `tests/memory_model_litmus.rs`, which carries the row — a whole [`BusLock`]
/// transaction between the store and the load, beside an `Unfenced` control
/// that is this code with the two fences deleted and nothing else changed.
/// The fenced arm asserting zero is sound everywhere and therefore proves
/// nothing on its own; only the pair, on a host that shows the control
/// tearing, says the fences are what forbids it. Nothing is asserted here,
/// because a green tick on this machine would still be for a property this
/// machine cannot check.
#[cfg(test)]
mod tests {
    use alloc::sync::Arc;

    use super::*;
    use crate::core::space::{AddressSpace, MemAttrs, MonitorSlot, RamStore, Region};
    use crate::core::value::Width;

    fn space() -> Arc<AddressSpace> {
        let space = Arc::new(AddressSpace::new("bus", 32));
        space
            .topology()
            .map(Region::ram("ram", Arc::new(RamStore::new(0x1000))), 0)
            .expect("the map fits");
        space
    }

    #[test]
    fn a_bus_lock_starts_free_and_reports_itself_held() {
        let lock = BusLock::new();
        assert!(!lock.held());
        assert_eq!(lock.taken(), 0);
        {
            let _g = lock.acquire();
            assert!(lock.held(), "a transaction is in flight");
            assert_eq!(lock.taken(), 1);
        }
        assert!(!lock.held(), "and the guard gave it back");
        assert_eq!(lock.taken(), 1, "the count is transactions, not holders");
    }

    #[test]
    fn a_second_acquire_is_refused_rather_than_waited_for() {
        let lock = BusLock::new();
        let held = lock.acquire();
        assert!(
            lock.try_acquire().is_none(),
            "the bus is taken; a try must say so"
        );
        assert_eq!(
            lock.taken(),
            1,
            "and a refused try is not a transaction: the bookkeeping belongs \
             after the exclusion, not before it"
        );
        assert!(lock.held(), "nor does it disturb the holder's flag");
        drop(held);
        assert!(lock.try_acquire().is_some(), "and free again afterwards");
    }

    #[test]
    fn ordinary_accesses_run_underneath_it() {
        // The point of the rank: the access path takes the space's topology
        // guard *under* the bus lock, and that is legal only because it is a
        // try-lock. If this ever stops holding, every locked instruction
        // panics on the rank check in a debug build.
        let space = space();
        let _bus = space.bus_lock().acquire();
        space
            .write(0x40, Width::U64, 0x1234, MemAttrs::DEFAULT)
            .expect("the write lands");
        assert_eq!(
            space.read(0x40, Width::U64, MemAttrs::DEFAULT),
            Ok(0x1234),
            "and reads back under the same lock"
        );
    }

    #[test]
    fn a_locked_write_still_breaks_a_sibling_reservation() {
        // The composition claim: the two objects do not know about each other
        // and do not have to, because a locked write is still a write.
        let space = space();
        let slot = MonitorSlot::new(Arc::clone(&space), 3).expect("a free slot");
        slot.reserve(0x40);
        {
            let _bus = space.bus_lock().acquire();
            space
                .write(0x40, Width::U64, 7, MemAttrs::DEFAULT)
                .expect("the write lands");
        }
        assert!(!slot.holds(), "a locked store is an observer's store");
    }

    /// The rank claim, checked rather than asserted in prose.
    ///
    /// `BUS_LOCK` has to be **above** `BUS` — a core takes it holding its own
    /// execution lock — and **below** every bus fabric in the `0x4000`-`0x5000`
    /// band, because a locked access reaches one of those while it is held.
    /// The lowest of them is `spi::FABRIC_RANK` at `0x4400`, which is why the
    /// number is `0x4100` and not the round `0x4800` that would have collided
    /// with `spi::SHIFTER_RANK`.
    #[cfg(debug_assertions)]
    #[test]
    fn the_rank_sits_above_bus_and_below_every_fabric() {
        use crate::core::sync::{held_rank, violates_lock_order};

        let lock = BusLock::new();
        // A CPU's session mutex, which is what a core is holding when it takes
        // the bus.
        let _session = LockRank::BUS.enter();
        let _bus = lock.acquire();
        assert_eq!(
            held_rank(),
            Some(LockRank::BUS_LOCK),
            "the bus lock records itself; `UNCHECKED` here would silence every \
             inversion underneath it"
        );
        assert!(
            !violates_lock_order(LockRank::new(0x4400)),
            "the lowest bus fabric rank must still be takeable under the bus lock"
        );
        assert!(
            !violates_lock_order(LockRank::DEVICE),
            "and so must a device handler's own lock"
        );
        assert!(
            violates_lock_order(LockRank::BUS),
            "while the fabric rank itself is above it, not below"
        );
    }

    #[test]
    fn a_bus_lock_is_one_coherence_domains_and_not_the_process_wide_one() {
        let a = space();
        let b = space();
        let _held = a.bus_lock().acquire();
        assert!(a.bus_lock().held());
        assert!(
            !b.bus_lock().held(),
            "another space's bus is untouched by this one's"
        );
        assert!(
            b.bus_lock().try_acquire().is_some(),
            "and can still be taken"
        );
    }
}
