//! The **bus lock**: exclusion held across the read *and* the write of one
//! indivisible read-modify-write.
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
//! read-modify-write, where hardware would have held it off. Closing that
//! would mean every store in the machine taking this lock — serialising the
//! whole fabric to buy something no real guest asks for, because a plain store
//! racing a locked read-modify-write on the same word is a data race in the
//! *guest's* own terms: every spinlock, refcount and futex has `LOCK` on both
//! sides of the contention. Devices and DMA are the same case, left the same
//! way.
//!
//! The shape that would close it without a cost on the store path is to
//! compose the two objects — reserve the granule in the monitor before the
//! read, check it still holds before the write, reissue when it does not — and
//! it is written down here rather than built because a reissue re-runs the
//! side effects of an MMIO operand, which is a worse failure than the one it
//! fixes.
//!
//! # It is not state
//!
//! A bus lock is never held across an instruction boundary, so a snapshot
//! taken at a safe point can never observe one held. There is nothing to
//! serialize and no chunk version moves.

use core::fmt;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};

use crate::core::sync::{LockRank, Mutex, MutexGuard};

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
    /// sound.
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
    fn claim(&self) {
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
    /// Clears the flag *before* the mutex is released: a `Drop` body runs
    /// ahead of the struct's own fields, so no observer ever sees the bus free
    /// and still flagged as held.
    fn drop(&mut self) {
        self.lock.held.store(false, Ordering::Release);
    }
}

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
