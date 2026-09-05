//! The **global exclusive monitor**: the shared half of a load-reserved /
//! store-conditional pair.
//!
//! # Why a space owns one
//!
//! A reservation is not a property of a core. It is a claim on a piece of
//! memory, and the whole point of the claim is that *somebody else* can
//! invalidate it. A monitor that lives inside one core can only ever be broken
//! by that core, so on a multiprocessor every `sc.d`/`stxr` succeeds and every
//! contended update is lost — which is exactly the defect this module exists
//! to close. The object all the cores of one coherent domain share is the
//! [`AddressSpace`], so that is where it lives.
//!
//! Both architectures describe the same two-part structure and this models
//! both parts:
//!
//! * **AArch64** (DDI 0487, B2.9 "Synchronization and semaphores") names a
//!   *local monitor* and a *global monitor* as separate state and requires a
//!   store-exclusive to pass both. The core's own `State::exclusive` is the
//!   local monitor; a slot here is its entry in the global one. Among the
//!   events that clear the global monitor is "a store by another observer to
//!   the same reservation granule", and the architecture also permits a
//!   monitor to be cleared **spuriously** — a licence this module uses exactly
//!   twice, and says so both times.
//! * **RISC-V** (Unprivileged ISA, "Load-Reserved/Store-Conditional
//!   Instructions") has a single *reservation set* rather than two monitors,
//!   but the same requirement: an `SC` must fail if another hart wrote the set
//!   since the `LR`. It also imposes an **eventuality guarantee** — a
//!   constrained LR/SC sequence must eventually succeed — which is what stops
//!   this being implementable by clearing everything on every store.
//!
//! # The granule is architectural, and it differs
//!
//! A reservation covers a naturally aligned block, not one byte. Too small and
//! a store to the other half of the word the guest is protecting slips past;
//! too large and unrelated traffic breaks reservations forever, which is how
//! the eventuality guarantee is lost. RISC-V's set must contain at least the
//! naturally aligned word the `LR` read (8 bytes here); AArch64's granule is
//! `IMPLEMENTATION DEFINED` between 8 and 2048 bytes and this model uses 16,
//! because the 64-bit exclusive *pair* is a single 16-byte access that has to
//! fit inside one granule.
//!
//! So the granule is per participant, not per space, and it is carried in the
//! slot: a reservation is stored as its granule base with the shift packed
//! into the low bits, which are zero by definition because the base is aligned
//! to `1 << shift`. Three bits are always free (the smallest granule is 8
//! bytes) and they hold `shift - 3`, so shifts 3..=10 — granules of 8 bytes to
//! 1 KiB — round-trip through one `u64` and one atomic load.
//!
//! # What it costs on the store path
//!
//! [`ExclusiveMonitor::note_store`] is called from `SpaceView::write_span`,
//! which every guest store, every DMA burst and every ROM load funnels
//! through, so its cost is paid by every board whether or not that board has
//! ever executed an exclusive. It is **one acquire load of one `u64`** in the
//! case that matters — no reservation outstanding anywhere in the space, which
//! is the state a machine is in for all but a few instructions around each
//! acquire/release pair. Only when that count is non-zero does it look at the
//! slots, and then it walks the *set bits*, so the work is proportional to the
//! number of outstanding reservations rather than to the size of the table.
//!
//! No lock, at any rank. The store path is reached with the core's own
//! `BUS`-ranked execution lock held and the space's topology lock taken for
//! reading inside it — the second of those is legal only because it is a
//! *try*-lock, which `core::sync` records without checking the order. A
//! blocking lock added here would have to rank above `BUS`, and it would
//! serialise every store in the machine to buy something the guest asks for a
//! few instructions in every million.
//!
//! # What x86 needs, and why it is not this
//!
//! x86 has no reservation. `LOCK CMPXCHG` and the implicitly locked `XCHG` are
//! *pessimistic and unconditional*: the read-modify-write has to be indivisible
//! to every other observer, and there is no status flag with which to report a
//! failure, because there is no retry loop in the guest to catch one. This
//! object cannot serve that, and the reason is the same licence that makes it
//! cheap — a monitor that may be cleared spuriously would turn `LOCK XADD` into
//! a wrong answer rather than into a retry.
//!
//! The primitive that fits is a **bus lock**: exclusion held across the read
//! *and* the write of one locked instruction, taken by the core before it
//! issues either. It belongs on the [`AddressSpace`] too, because a space is
//! the coherence domain and that is the only thing the two mechanisms have to
//! agree about — but it is a different object, and its cost lands only on
//! locked accesses rather than on every store. Ranked, it would sit between
//! `LockRank::BUS` and `LockRank::DEVICE`: above `BUS` because the core takes
//! it while holding its own execution lock, and below `DEVICE` because a
//! `LOCK`ed access to an MMIO region reaches a device handler while it is held.
//!
//! The two compose without knowing about each other, which is the argument for
//! one owner rather than two subsystems: a locked write still goes out through
//! `SpaceView::write_span`, so it breaks reservations on its way past, and an
//! x86 core sharing a space with an AArch64 one — `machines/tests/heterogeneous.machine`
//! is that board — gets that for free without ever registering a slot here.
//!
//! # It is derived state and it is not serialized
//!
//! The architectural reservation stays where it always was: the core's own
//! chunk, in the field it always used, in the layout it always had. What is
//! *here* is the shared broadcast of it, and a snapshot restore brings it back
//! empty — a restored core whose reservation was outstanding at save time sees
//! its next store-conditional fail.
//!
//! That is deliberate and it is legal. AArch64 permits a spurious monitor
//! clear outright; RISC-V permits an `SC` to fail for implementation-specific
//! reasons, and its eventuality guarantee is not threatened by a one-off
//! event, because the guest's retry loop simply takes the reservation again.
//! The alternative — republishing a reservation whose *physical* address the
//! snapshot does not record — would mean re-walking a page table from inside
//! `load`, where a fault has nowhere to go, and would fail in the unsafe
//! direction if the mapping had changed since.

use alloc::boxed::Box;
use alloc::sync::Arc;
use core::fmt;
use core::sync::atomic::{AtomicU64, Ordering};

use super::AddressSpace;

/// How many participants one space's monitor can watch at once.
///
/// One slot per core attached to the space, claimed when the core is given the
/// space and released when it is given another one or dropped. 256 is far more
/// than any board here has, and the ceiling that matters is
/// [`usermode`](crate::usermode), where a guest thread *is* a core: a process
/// with more than 256 live threads gets slots for the first 256 and the rest
/// keep the core-local monitor they would have had anyway.
pub const MONITOR_SLOTS: usize = 256;

/// [`MONITOR_SLOTS`] bits, as `u64` words.
const WORDS: usize = MONITOR_SLOTS / 64;

/// The narrowest granule the slot packing admits: 8 bytes, which is RISC-V's.
pub const MIN_GRANULE_SHIFT: u32 = 3;

/// The widest: 1 KiB. AArch64 allows up to 2048 bytes, but the packing needs
/// three free low bits of the base and a shift of 11 would need four.
pub const MAX_GRANULE_SHIFT: u32 = 10;

/// One participant's registration in one space's [`ExclusiveMonitor`].
///
/// Carries the granule as well as the slot because the granule is fixed for
/// the life of the registration — it is a property of the core's architecture
/// — and packing it into the slot value is what lets a store consult a
/// reservation of any width with a single atomic load.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MonitorId {
    slot: u16,
    shift: u8,
}

impl MonitorId {
    /// Which slot this is, for tests and diagnostics.
    #[must_use]
    pub fn index(self) -> usize {
        self.slot as usize
    }

    /// The reservation granule, in bytes.
    #[must_use]
    pub fn granule(self) -> u64 {
        1u64 << self.shift
    }
}

/// The shared reservation table of one address space.
///
/// The `monitor` module documentation has the structure, what it costs, and
/// why it is not serialized.
pub struct ExclusiveMonitor {
    /// How many bits are set across `live`.
    ///
    /// Redundant with `live`, and the whole reason the store path is one load:
    /// a machine with no reservation outstanding reads this, sees zero, and
    /// touches nothing else.
    ///
    /// **First field, and the reason the slot table is the only boxed part.**
    /// An [`AddressSpace`] holds the monitor inline, so this word sits in the
    /// space's own allocation next to the fields the access path has already
    /// touched; behind a `Box` it was a second dependent load on every store in
    /// the machine, and measured as one.
    outstanding: AtomicU64,
    /// Which of those slots currently hold a reservation.
    live: [AtomicU64; WORDS],
    /// Which slots [`ExclusiveMonitor::register`] has handed out.
    claimed: [AtomicU64; WORDS],
    /// The reserved granule per slot, packed as `base | (shift - 3)`.
    ///
    /// Meaningful only while that slot's `live` bit is set. Two kilobytes, and
    /// the only part a store with nothing outstanding never reads — so this is
    /// the part that is boxed.
    slots: Box<[AtomicU64; MONITOR_SLOTS]>,
}

impl fmt::Debug for ExclusiveMonitor {
    /// The slot array is 256 entries and printing it would bury every `{:?}`
    /// of an [`AddressSpace`]; what a reader wants is how many reservations
    /// are outstanding.
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("ExclusiveMonitor")
            .field("outstanding", &self.outstanding.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Default for ExclusiveMonitor {
    fn default() -> Self {
        ExclusiveMonitor::new()
    }
}

impl ExclusiveMonitor {
    /// An empty monitor, every slot free.
    #[must_use]
    pub fn new() -> ExclusiveMonitor {
        #[expect(
            clippy::declare_interior_mutable_const,
            reason = "the initializer for an array of atomics; there is no other spelling"
        )]
        const ZERO: AtomicU64 = AtomicU64::new(0);
        ExclusiveMonitor {
            outstanding: AtomicU64::new(0),
            live: [ZERO; WORDS],
            claimed: [ZERO; WORDS],
            slots: Box::new([ZERO; MONITOR_SLOTS]),
        }
    }

    /// Claim a slot for a core whose reservation granule is `1 << shift`
    /// bytes.
    ///
    /// `None` when every slot is taken, which is a core-count ceiling rather
    /// than an error: the caller keeps the core-local monitor it would have
    /// had anyway.
    ///
    /// # Panics
    ///
    /// If `shift` is outside [`MIN_GRANULE_SHIFT`]..=[`MAX_GRANULE_SHIFT`].
    /// That is a coding error in a CPU core, never a guest condition.
    #[must_use]
    pub fn register(&self, shift: u32) -> Option<MonitorId> {
        assert!(
            (MIN_GRANULE_SHIFT..=MAX_GRANULE_SHIFT).contains(&shift),
            "a reservation granule of 2^{shift} bytes does not fit a monitor slot"
        );
        for (w, word) in self.claimed.iter().enumerate() {
            let mut seen = word.load(Ordering::Relaxed);
            while seen != u64::MAX {
                let bit = (!seen).trailing_zeros();
                match word.compare_exchange_weak(
                    seen,
                    seen | (1u64 << bit),
                    Ordering::AcqRel,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => {
                        let id = MonitorId {
                            slot: (w * 64 + bit as usize) as u16,
                            shift: shift as u8,
                        };
                        // A recycled slot must not start out live. `release`
                        // already clears it; doing it again here costs nothing
                        // and makes the invariant local to this function.
                        self.clear(id);
                        return Some(id);
                    }
                    Err(now) => seen = now,
                }
            }
        }
        None
    }

    /// Give a slot back. The reservation it held, if any, is dropped.
    pub fn release(&self, id: MonitorId) {
        self.clear(id);
        let (w, bit) = split(id.slot);
        self.claimed[w].fetch_and(!bit, Ordering::AcqRel);
    }

    /// Record that `id`'s core has reserved the granule containing
    /// guest-physical `addr`.
    ///
    /// Replaces whatever that slot held: a core has one reservation.
    pub fn reserve(&self, id: MonitorId, addr: u64) {
        let (w, bit) = split(id.slot);
        // Dropped *first*, so that a concurrent `note_store` cannot read the
        // new slot value under the old reservation's live bit and clear a
        // reservation that had not been taken when its store was issued.
        self.clear(id);
        let shift = u32::from(id.shift);
        let mask = (1u64 << shift) - 1;
        let code = u64::from(id.shift) - u64::from(MIN_GRANULE_SHIFT);
        self.slots[id.slot as usize].store((addr & !mask) | code, Ordering::Relaxed);
        // Release: the slot value has to be visible to anyone who sees the
        // bit, or a store would consult a stale granule.
        if self.live[w].fetch_or(bit, Ordering::Release) & bit == 0 {
            self.outstanding.fetch_add(1, Ordering::Release);
        }
    }

    /// Whether `id`'s reservation is still standing.
    ///
    /// The address is not an argument: the *local* monitor — the core's own
    /// field — is what says which address was reserved, and this says whether
    /// anything has broken it since. Both have to agree for a
    /// store-conditional to succeed, which is exactly AArch64's two-monitor
    /// rule and, for RISC-V, two halves of one reservation set.
    #[must_use]
    pub fn holds(&self, id: MonitorId) -> bool {
        let (w, bit) = split(id.slot);
        self.live[w].load(Ordering::Acquire) & bit != 0
    }

    /// Drop `id`'s reservation, if it has one.
    pub fn clear(&self, id: MonitorId) {
        let (w, bit) = split(id.slot);
        if self.live[w].fetch_and(!bit, Ordering::AcqRel) & bit != 0 {
            self.outstanding.fetch_sub(1, Ordering::Release);
        }
    }

    /// Break every reservation covering any of the `bytes` bytes at `addr`.
    ///
    /// Called for every store that reaches the space, from any master —
    /// AArch64's "a store by another observer to the same granule", RISC-V's
    /// write to the reservation set. A store by the reserving core *itself* is
    /// included, which is what both architectures require and is also why the
    /// caller does not have to say who it is.
    ///
    /// Addresses are **guest-physical**. Two harts with different page tables
    /// contending for one lock is the ordinary case, and a virtual key would
    /// miss it.
    #[inline]
    pub fn note_store(&self, addr: u64, bytes: u64) {
        // The whole hot path, on every board, in the state a machine is in
        // almost all of the time.
        if self.outstanding.load(Ordering::Acquire) == 0 {
            return;
        }
        self.break_covering(addr, bytes);
    }

    /// The rest of [`ExclusiveMonitor::note_store`], out of line so the common
    /// case stays a load and a branch.
    fn break_covering(&self, addr: u64, bytes: u64) {
        // A zero-length transfer touches nothing and breaks nothing. It cannot
        // arrive from `SpaceView::write_span`, which returns before it gets
        // here, but the method is public and "no bytes" has exactly one
        // sensible answer.
        let end = addr.saturating_add(bytes);
        for (w, word) in self.live.iter().enumerate() {
            let mut set = word.load(Ordering::Acquire);
            while set != 0 {
                let bit = set.trailing_zeros();
                set &= set - 1;
                let index = w * 64 + bit as usize;
                let packed = self.slots[index].load(Ordering::Relaxed);
                let shift = (packed & 7) + u64::from(MIN_GRANULE_SHIFT);
                let base = packed & !((1u64 << shift) - 1);
                let granule_end = base.saturating_add(1u64 << shift);
                if addr < granule_end && base < end {
                    self.clear(MonitorId {
                        slot: index as u16,
                        shift: shift as u8,
                    });
                }
            }
        }
    }

    /// How many reservations are outstanding across the whole space.
    ///
    /// A statistic, for tests and for the monitor command; nothing in the
    /// emulation path reads it except the store fast path's own zero check.
    #[must_use]
    pub fn outstanding(&self) -> u64 {
        self.outstanding.load(Ordering::Relaxed)
    }
}

/// The word and bit a slot index lands on.
#[inline]
fn split(slot: u16) -> (usize, u64) {
    let slot = slot as usize;
    (slot / 64, 1u64 << (slot % 64))
}

/// One core's registration in one space's monitor, released when dropped.
///
/// Held beside the `Arc<AddressSpace>` it belongs to, because a core that is
/// given a different space — or dropped — has to give its slot back, and doing
/// that by hand is how a 256-slot table leaks itself empty.
#[derive(Debug)]
pub struct MonitorSlot {
    space: Arc<AddressSpace>,
    id: MonitorId,
}

impl MonitorSlot {
    /// Register in `space`'s monitor with a granule of `1 << shift` bytes.
    ///
    /// `None` when the space's slots are all taken; see [`MONITOR_SLOTS`].
    #[must_use]
    pub fn new(space: Arc<AddressSpace>, shift: u32) -> Option<MonitorSlot> {
        let id = space.monitor().register(shift)?;
        Some(MonitorSlot { space, id })
    }

    /// Reserve the granule containing guest-physical `addr`.
    #[inline]
    pub fn reserve(&self, addr: u64) {
        self.space.monitor().reserve(self.id, addr);
    }

    /// Whether this core's reservation is still standing.
    #[inline]
    #[must_use]
    pub fn holds(&self) -> bool {
        self.space.monitor().holds(self.id)
    }

    /// Drop this core's reservation.
    #[inline]
    pub fn clear(&self) {
        self.space.monitor().clear(self.id);
    }

    /// The registration itself.
    #[must_use]
    pub fn id(&self) -> MonitorId {
        self.id
    }
}

impl Drop for MonitorSlot {
    fn drop(&mut self) {
        self.space.monitor().release(self.id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::AddressSpace;

    fn space() -> Arc<AddressSpace> {
        Arc::new(AddressSpace::new("mon", 32))
    }

    #[test]
    fn a_sibling_store_into_the_granule_breaks_the_reservation() {
        let m = ExclusiveMonitor::new();
        let a = m.register(3).expect("a free slot");
        let b = m.register(3).expect("a free slot");
        m.reserve(a, 0x1000);
        m.reserve(b, 0x2000);
        assert_eq!(m.outstanding(), 2);
        // B's store lands in A's granule — the whole point of the object.
        m.note_store(0x1004, 4);
        assert!(!m.holds(a), "a store into the granule breaks it");
        assert!(m.holds(b), "and leaves an unrelated one alone");
    }

    #[test]
    fn a_store_outside_the_granule_leaves_it_alone() {
        // The eventuality guarantee lives here: clearing too eagerly is how a
        // constrained LR/SC sequence never completes.
        let m = ExclusiveMonitor::new();
        let a = m.register(3).expect("a free slot");
        m.reserve(a, 0x1000);
        m.note_store(0x1008, 8);
        m.note_store(0x0ff8, 8);
        assert!(m.holds(a));
    }

    #[test]
    fn the_granule_is_the_reserving_cores_own_width() {
        // Two participants with different granules in one table, which is what
        // the packed shift is for.
        let m = ExclusiveMonitor::new();
        let rv = m.register(3).expect("a free slot");
        let a64 = m.register(4).expect("a free slot");
        m.reserve(rv, 0x2000);
        m.reserve(a64, 0x2000);
        // 8 bytes past the base: outside RISC-V's granule, inside AArch64's.
        m.note_store(0x2008, 8);
        assert!(m.holds(rv), "8 bytes on is a different RISC-V granule");
        assert!(!m.holds(a64), "but the same 16-byte AArch64 one");
    }

    #[test]
    fn an_unaligned_reservation_is_taken_on_its_granule() {
        let m = ExclusiveMonitor::new();
        let a = m.register(4).expect("a free slot");
        m.reserve(a, 0x1234);
        m.note_store(0x1230, 1);
        assert!(!m.holds(a), "0x1230 and 0x1234 are one 16-byte granule");
    }

    #[test]
    fn an_unaligned_reservation_does_not_widen_its_granule() {
        // The packing puts `shift - 3` in the low bits of the base *because
        // the base is aligned and they are free*. Storing the address
        // unmasked would let its own low bits be read back as a granule code,
        // and the reservation would silently cover up to 1 KiB — legal on
        // paper, and a forward-progress hazard in practice, because unrelated
        // traffic anywhere in that kilobyte would fail the store-conditional.
        let m = ExclusiveMonitor::new();
        let a = m.register(3).expect("a free slot");
        m.reserve(a, 0x1005);
        m.note_store(0x1080, 8);
        assert!(
            m.holds(a),
            "0x1080 is 128 bytes away from an eight-byte granule"
        );
        m.note_store(0x1000, 8);
        assert!(!m.holds(a), "and 0x1000 is inside it");
    }

    #[test]
    fn a_zero_length_transfer_breaks_nothing() {
        let m = ExclusiveMonitor::new();
        let a = m.register(3).expect("a free slot");
        m.reserve(a, 0x1000);
        m.note_store(0x1000, 0);
        assert!(m.holds(a), "no bytes were written, so nothing was broken");
    }

    #[test]
    fn a_store_that_straddles_the_granule_breaks_it() {
        let m = ExclusiveMonitor::new();
        let a = m.register(3).expect("a free slot");
        m.reserve(a, 0x1000);
        // A bulk write ending inside the granule, starting well before it.
        m.note_store(0x0f00, 0x104);
        assert!(!m.holds(a));
    }

    #[test]
    fn the_fast_path_is_a_no_op_with_nothing_outstanding() {
        let m = ExclusiveMonitor::new();
        let a = m.register(3).expect("a free slot");
        assert_eq!(m.outstanding(), 0);
        m.note_store(0x1000, 8);
        assert!(!m.holds(a));
        m.reserve(a, 0x1000);
        assert_eq!(m.outstanding(), 1);
        m.clear(a);
        assert_eq!(m.outstanding(), 0, "the count follows the bits exactly");
        m.clear(a);
        assert_eq!(m.outstanding(), 0, "and a second clear does not underflow");
    }

    #[test]
    fn a_released_slot_comes_back_free_and_empty() {
        let m = ExclusiveMonitor::new();
        let a = m.register(3).expect("a free slot");
        m.reserve(a, 0x1000);
        m.release(a);
        assert_eq!(m.outstanding(), 0, "release drops the reservation");
        let b = m.register(4).expect("the slot is free again");
        assert_eq!(b.index(), a.index());
        assert!(!m.holds(b));
    }

    #[test]
    fn the_table_has_a_ceiling_and_says_so_rather_than_wrapping() {
        let m = ExclusiveMonitor::new();
        let ids: alloc::vec::Vec<_> = (0..MONITOR_SLOTS)
            .map(|_| m.register(3).expect("a free slot"))
            .collect();
        assert_eq!(m.register(3), None, "the 257th core gets no slot");
        // Every slot is distinct, which is what makes `holds` a per-core
        // question.
        for (i, id) in ids.iter().enumerate() {
            assert_eq!(id.index(), i);
        }
    }

    #[test]
    fn a_dropped_slot_gives_itself_back() {
        let space = space();
        {
            let slot = MonitorSlot::new(Arc::clone(&space), 4).expect("a free slot");
            slot.reserve(0x40);
            assert!(slot.holds());
            assert_eq!(space.monitor().outstanding(), 1);
        }
        assert_eq!(
            space.monitor().outstanding(),
            0,
            "dropping the registration drops the reservation with it"
        );
        // And the slot itself is back in the pool.
        let again = MonitorSlot::new(Arc::clone(&space), 3).expect("a free slot");
        assert_eq!(again.id().index(), 0);
    }

    #[test]
    fn granule_bytes_round_trip_through_the_id() {
        let m = ExclusiveMonitor::new();
        assert_eq!(m.register(3).expect("free").granule(), 8);
        assert_eq!(m.register(4).expect("free").granule(), 16);
        assert_eq!(m.register(10).expect("free").granule(), 1024);
    }

    #[test]
    #[should_panic(expected = "does not fit a monitor slot")]
    fn a_granule_the_packing_cannot_carry_is_a_coding_error() {
        let m = ExclusiveMonitor::new();
        let _ = m.register(11);
    }

    #[test]
    fn every_slot_index_packs_and_unpacks() {
        // `split` is the only arithmetic between a slot number and the bit it
        // owns, and an off-by-one there would silently alias two cores.
        for slot in 0..MONITOR_SLOTS {
            let (w, bit) = split(slot as u16);
            assert_eq!(w, slot / 64);
            assert_eq!(bit.trailing_zeros() as usize, slot % 64);
        }
    }

    #[test]
    fn a_reserve_replaces_the_previous_granule() {
        let m = ExclusiveMonitor::new();
        let a = m.register(3).expect("a free slot");
        m.reserve(a, 0x1000);
        m.reserve(a, 0x2000);
        assert_eq!(m.outstanding(), 1, "one core, one reservation");
        m.note_store(0x1000, 8);
        assert!(m.holds(a), "the old granule is no longer watched");
        m.note_store(0x2000, 8);
        assert!(!m.holds(a));
    }

    #[test]
    fn a_bulk_write_breaks_a_reservation_it_only_reaches_its_end_of() {
        // The length the space reports has to be the transfer's, not one
        // byte: a DMA burst that starts below a reserved granule and runs
        // through it is an observer's store to that granule, and a monitor
        // told only about the first byte would let it past.
        use crate::core::space::{MemAttrs, RamStore, Region};
        use alloc::vec;
        let space = space();
        let store = Arc::new(RamStore::new(0x1000));
        space
            .topology()
            .map(Region::ram("ram", store), 0)
            .expect("the map fits");
        let slot = MonitorSlot::new(Arc::clone(&space), 3).expect("a free slot");
        slot.reserve(0x100);
        space
            .write_bytes(0x80, &vec![0xab; 0x100], MemAttrs::DEFAULT)
            .expect("the burst lands");
        assert!(
            !slot.holds(),
            "the burst covered 0x80..0x180, and the reservation is at 0x100"
        );
    }

    #[test]
    fn a_debug_read_of_the_space_does_not_break_a_reservation() {
        // `MemAttrs::debug` must have no side effects, and a cleared
        // reservation is one. The write path is where that is enforced; this
        // pins the direction the enforcement has to take.
        use crate::core::space::{MemAttrs, RamStore, Region};
        use crate::core::value::Width;
        let space = space();
        let store = Arc::new(RamStore::new(0x1000));
        space
            .topology()
            .map(Region::ram("ram", store), 0)
            .expect("the map fits");
        let slot = MonitorSlot::new(Arc::clone(&space), 3).expect("a free slot");
        slot.reserve(0x40);
        space
            .write(0x40, Width::U64, 1, MemAttrs::DEBUG)
            .expect("the write lands");
        assert!(slot.holds(), "a debug write is not an observer's store");
        space
            .write(0x40, Width::U64, 2, MemAttrs::DEFAULT)
            .expect("the write lands");
        assert!(!slot.holds(), "an ordinary one is");
    }
}
