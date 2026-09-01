//! Tests for the level-3 seam, and the proof guest.
//!
//! The interesting one is at the bottom: a hand-assembled RV64 program that
//! writes to file descriptor 1 and exits, serviced by a handler outside
//! `core/` that knows nothing this module knows. It is `ROADMAP.md` phase 5b's
//! gate for rsemu's half, and it is written the way a downstream crate would
//! have to write it — through the public surface, with no reach into anything
//! private.

use alloc::sync::Arc;
use alloc::vec::Vec;

use crate::core::clock::GlobalTime;
use crate::core::error::{BusError, Error};
use crate::core::value::Width;

use super::{Answer, GuestClock, Journal, JournalMode, Prot, Tag, ThreadSet, UserMemory};

// ---------------------------------------------------------------------------
// The memory map
// ---------------------------------------------------------------------------

#[test]
fn a_fresh_map_has_nothing_in_it() {
    let mem = UserMemory::new(48);
    assert!(mem.mappings().is_empty());
    assert_eq!(mem.mapped_bytes(), 0);
    // Nothing is mapped, so the guest faults — which is what "no devices in
    // it" has to mean at address zero as much as anywhere else.
    let mut buf = [0u8; 4];
    assert!(mem.read_bytes(0, &mut buf).is_err());
}

#[test]
fn a_mapping_exists_where_it_was_put() {
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x2000, Prot::RW, "anon").unwrap();
    let maps = mem.mappings();
    assert_eq!(maps.len(), 1);
    assert_eq!(maps[0].base, 0x1000);
    assert_eq!(maps[0].len, 0x2000);
    assert_eq!(maps[0].name, "anon");
    assert_eq!(mem.mapped_bytes(), 0x2000);

    mem.write_bytes(0x1500, b"hello").unwrap();
    let mut buf = [0u8; 5];
    mem.read_bytes(0x1500, &mut buf).unwrap();
    assert_eq!(&buf, b"hello");
}

#[test]
fn a_mapping_is_reachable_through_the_address_space() {
    // The whole reason for building on `core::space`: what the consumer maps
    // is what a core executing in this map sees, with no second memory model.
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x1000, Prot::RW, "anon").unwrap();
    mem.write_bytes(0x1000, &[0xaa, 0xbb, 0xcc, 0xdd]).unwrap();
    let value = mem
        .space()
        .read(
            0x1000,
            crate::core::Width::U32,
            crate::core::space::MemAttrs::DEFAULT,
        )
        .unwrap();
    assert_eq!(value, 0xddcc_bbaa);
}

#[test]
fn placement_is_deterministic_and_top_down() {
    let a = UserMemory::new(48);
    let b = UserMemory::new(48);
    for mem in [&a, &b] {
        let first = mem.map(0x1000, Prot::RW, "one").unwrap();
        let second = mem.map(0x2000, Prot::RW, "two").unwrap();
        assert!(second < first, "placement grows downwards");
    }
    // Two maps, the same calls, the same addresses. This is what makes a
    // level-3 run reproducible: nothing about placement consults the host.
    assert_eq!(a.mappings(), b.mappings());
}

#[test]
fn placement_steps_over_what_is_already_there() {
    let mem = UserMemory::new(32);
    let top = 1u64 << 32;
    mem.map_at(top - 0x2000, 0x1000, Prot::RW, "fixed").unwrap();
    let placed = mem.map(0x1000, Prot::RW, "placed").unwrap();
    assert!(
        placed + 0x1000 <= top - 0x2000 || placed >= top - 0x1000,
        "a placed mapping must not overlap a fixed one: {placed:#x}"
    );
    assert_eq!(mem.mappings().len(), 2);
}

#[test]
fn unmapping_the_middle_splits_and_keeps_the_bytes() {
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x3000, Prot::RW, "anon").unwrap();
    mem.write_bytes(0x1000, b"head").unwrap();
    mem.write_bytes(0x3000, b"tail").unwrap();

    mem.unmap(0x2000, 0x1000).unwrap();
    let maps = mem.mappings();
    assert_eq!(maps.len(), 2, "the hole splits the range in two");
    assert_eq!((maps[0].base, maps[0].len), (0x1000, 0x1000));
    assert_eq!((maps[1].base, maps[1].len), (0x3000, 0x1000));

    let mut buf = [0u8; 4];
    mem.read_bytes(0x1000, &mut buf).unwrap();
    assert_eq!(&buf, b"head");
    mem.read_bytes(0x3000, &mut buf).unwrap();
    assert_eq!(&buf, b"tail", "a split must carry the bytes with it");

    // And the hole really is a hole.
    assert!(mem.read_bytes(0x2000, &mut buf).is_err());
}

#[test]
fn unmapping_nothing_is_not_an_error() {
    let mem = UserMemory::new(48);
    // The caller asked for the range to be gone, and it is.
    mem.unmap(0x8000, 0x1000).unwrap();
    assert!(mem.mappings().is_empty());
}

#[test]
fn a_fixed_mapping_replaces_what_was_there() {
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x2000, Prot::RW, "old").unwrap();
    mem.write_bytes(0x1000, b"old").unwrap();
    mem.map_at(0x1000, 0x1000, Prot::RW, "new").unwrap();

    let maps = mem.mappings();
    assert_eq!(maps.len(), 2);
    assert_eq!(maps[0].name, "new");
    let mut buf = [0u8; 3];
    mem.read_bytes(0x1000, &mut buf).unwrap();
    assert_eq!(&buf, b"\0\0\0", "a replaced range comes back zeroed");
}

#[test]
fn permission_is_checked_on_the_consumers_accesses() {
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x1000, Prot::READ, "ro").unwrap();
    let mut buf = [0u8; 4];
    mem.read_bytes(0x1000, &mut buf).unwrap();
    assert_eq!(
        mem.write_bytes(0x1000, b"no"),
        Err(Error::Bus(BusError::BadAccess)),
        "a guest pointer into a read-only range must be refused"
    );
    // The loader's path ignores permission on purpose: filling a read-only
    // text segment is the whole reason it exists.
    mem.init_bytes(0x1000, b"yes").unwrap();
    mem.read_bytes(0x1000, &mut buf).unwrap();
    assert_eq!(&buf[..3], b"yes");
}

#[test]
fn a_prot_none_range_exists_and_permits_nothing() {
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x1000, Prot::NONE, "guard").unwrap();
    // It exists as bookkeeping...
    assert_eq!(mem.mappings().len(), 1);
    assert_eq!(mem.mapping_at(0x1000).unwrap().prot, Prot::NONE);
    // ...and it exists in the address space too, permitting nothing. The two
    // faults are different facts and a consumer needs both: a reserved range
    // that refuses everything is not the same thing as an address nobody
    // decodes, and only one of them means "you never asked for this".
    assert_eq!(
        mem.space()
            .read(0x1000, Width::U8, crate::core::space::MemAttrs::DEFAULT),
        Err(BusError::Protected)
    );
    assert_eq!(
        mem.space()
            .read(0x9000, Width::U8, crate::core::space::MemAttrs::DEFAULT),
        Err(BusError::Unassigned)
    );
}

#[test]
fn the_guest_cannot_write_a_read_only_range() {
    // The gap this closes: `Prot` used to be enforced only on the accesses
    // this module made *for* the guest. A store the guest issued itself went
    // to an address space that had never heard of permission.
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x1000, Prot::RX, "text").unwrap();
    mem.init_bytes(0x1000, b"code").unwrap();

    let attrs = crate::core::space::MemAttrs::DEFAULT;
    assert_eq!(
        mem.space().read(0x1000, Width::U8, attrs),
        Ok(u64::from(b'c'))
    );
    assert_eq!(
        mem.space().write(0x1000, Width::U8, 0xff, attrs),
        Err(BusError::Protected),
        "a guest store into a text segment must fault"
    );
    assert_eq!(
        mem.space().read(0x1000, Width::U8, attrs),
        Ok(u64::from(b'c')),
        "and change nothing"
    );
    assert!(
        !mem.resolve_write_fault(0x1000).unwrap(),
        "the fault is a real one, not a sharing fault, so there is nothing to resolve"
    );
}

#[test]
fn a_fork_shares_its_pages_until_one_side_writes() {
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x1000, Prot::RW, "anon").unwrap();
    // A second, separate range: the break is per *range*, not per page, so
    // this is what proves one range's break leaves another's sharing alone.
    mem.map_at(0x2000, 0x1000, Prot::RW, "[heap]").unwrap();
    mem.write_bytes(0x1000, b"parent").unwrap();

    let child = mem.duplicate().unwrap();
    assert!(mem.is_shared(0x1000), "the parent gave up exclusive use");
    assert!(child.is_shared(0x1000), "and the child never had it");

    // Both sides can still read, and see the same bytes.
    let mut buf = [0u8; 6];
    child.read_bytes(0x1000, &mut buf).unwrap();
    assert_eq!(&buf, b"parent");

    // A guest store on the child's side faults rather than corrupting the
    // parent, and the consumer's fault handler resolves it.
    let attrs = crate::core::space::MemAttrs::DEFAULT;
    assert_eq!(
        child.space().write(0x1000, Width::U8, b'c'.into(), attrs),
        Err(BusError::Protected)
    );
    assert!(
        child.resolve_write_fault(0x1000).unwrap(),
        "a shared page's write fault is resolvable"
    );
    assert!(!child.is_shared(0x1000));
    // Reissued, as a consumer would after restarting the instruction.
    child
        .space()
        .write(0x1000, Width::U8, b'c'.into(), attrs)
        .unwrap();

    mem.read_bytes(0x1000, &mut buf).unwrap();
    assert_eq!(&buf, b"parent", "the parent's bytes are its own");
    child.read_bytes(0x1000, &mut buf).unwrap();
    assert_eq!(&buf, b"carent");

    // The parent's other range is still shared, and the parent's own store
    // faults there too until it writes.
    assert!(mem.is_shared(0x2000));
    assert!(child.is_shared(0x2000));
    assert_eq!(
        mem.space().write(0x2000, Width::U8, 1, attrs),
        Err(BusError::Protected)
    );
    assert!(mem.resolve_write_fault(0x2000).unwrap());
    assert!(!mem.is_shared(0x2000));
}

#[test]
fn a_consumer_side_write_breaks_the_sharing_itself() {
    // A consumer writing on the guest's behalf holds nothing the address space
    // needs and has no instruction to restart, so it must not be handed a
    // fault to resolve. The guest's own store is the case that must.
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x1000, Prot::RW, "anon").unwrap();
    let child = mem.duplicate().unwrap();
    assert!(child.is_shared(0x1000));

    child.write_bytes(0x1000, b"child!").unwrap();
    assert!(!child.is_shared(0x1000));
    let mut buf = [0u8; 6];
    mem.read_bytes(0x1000, &mut buf).unwrap();
    assert_eq!(&buf, &[0u8; 6], "the parent is untouched");
}

#[test]
fn protecting_a_whole_forked_range_keeps_it_shared() {
    // `mprotect` after a `fork` is the common case, and copying there would
    // undo the point of a lazy fork. Only a *split* materialises.
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x1000, Prot::RW, "anon").unwrap();
    let child = mem.duplicate().unwrap();

    child.protect(0x1000, 0x1000, Prot::READ).unwrap();
    assert!(child.is_shared(0x1000), "nothing was copied");
    assert_eq!(child.mapping_at(0x1000).unwrap().prot, Prot::READ);
    assert!(
        !child.resolve_write_fault(0x1000).unwrap(),
        "and a write fault there is now a real one"
    );
}

#[test]
fn sharing_does_not_survive_a_snapshot() {
    // Derived state, and the rule is `ROADMAP.md` §15's: the bytes are
    // architectural and are saved; who happened to be sharing them is not.
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x1000, Prot::RW, "anon").unwrap();
    mem.write_bytes(0x1000, b"bytes").unwrap();
    let _child = mem.duplicate().unwrap();
    assert!(mem.is_shared(0x1000));

    let mut bytes: Vec<u8> = Vec::new();
    mem.save(&mut bytes).unwrap();
    let restored = UserMemory::new(48);
    let mut source = crate::core::state::SliceSource::new(&bytes);
    restored.load(&mut source).unwrap();

    assert!(!restored.is_shared(0x1000));
    let mut buf = [0u8; 5];
    restored.read_bytes(0x1000, &mut buf).unwrap();
    assert_eq!(&buf, b"bytes");
}

#[test]
fn protect_splits_and_changes_only_the_range_asked_for() {
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x3000, Prot::RW, "anon").unwrap();
    mem.protect(0x2000, 0x1000, Prot::READ).unwrap();

    let maps = mem.mappings();
    assert_eq!(maps.len(), 3);
    assert_eq!(maps[0].prot, Prot::RW);
    assert_eq!(maps[1].prot, Prot::READ);
    assert_eq!(maps[2].prot, Prot::RW);
    mem.write_bytes(0x1000, b"a").unwrap();
    assert!(mem.write_bytes(0x2000, b"a").is_err());
    mem.write_bytes(0x3000, b"a").unwrap();
}

#[test]
fn an_unaligned_range_is_refused() {
    let mem = UserMemory::new(48);
    assert!(mem.map_at(0x1001, 0x1000, Prot::RW, "x").is_err());
    assert!(mem.map_at(0x1000, 0x1001, Prot::RW, "x").is_err());
    assert!(mem.map(0, Prot::RW, "x").is_err());
}

#[test]
fn a_range_that_does_not_fit_the_space_is_refused() {
    let mem = UserMemory::new(32);
    assert!(
        mem.map_at((1u64 << 32) - 0x1000, 0x2000, Prot::RW, "x")
            .is_err()
    );
}

#[test]
fn prot_prints_the_way_a_memory_map_does() {
    assert_eq!(alloc::format!("{}", Prot::RW), "rw-");
    assert_eq!(alloc::format!("{}", Prot::RX), "r-x");
    assert_eq!(alloc::format!("{}", Prot::NONE), "---");
    assert_eq!(alloc::format!("{}", Prot::RWX), "rwx");
    assert!(Prot::RWX.contains(Prot::WRITE));
    assert!(!Prot::RX.contains(Prot::WRITE));
    assert_eq!(Prot::READ.union(Prot::WRITE), Prot::RW);
    assert!(Prot::NONE.is_none());
}

#[test]
fn a_duplicate_is_a_separate_map_with_the_same_bytes() {
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x1000, Prot::RW, "anon").unwrap();
    mem.write_bytes(0x1000, b"parent").unwrap();

    let child = mem.duplicate().unwrap();
    let mut buf = [0u8; 6];
    child.read_bytes(0x1000, &mut buf).unwrap();
    assert_eq!(&buf, b"parent");

    child.write_bytes(0x1000, b"child!").unwrap();
    mem.read_bytes(0x1000, &mut buf).unwrap();
    assert_eq!(&buf, b"parent", "the copy must not alias the original");
}

#[test]
fn a_map_round_trips_through_a_snapshot() {
    let mem = UserMemory::new(48);
    mem.map_at(0x1000, 0x1000, Prot::RX, "text").unwrap();
    mem.map_at(0x8000, 0x2000, Prot::RW, "[heap]").unwrap();
    mem.map_at(0x20000, 0x1000, Prot::NONE, "[guard]").unwrap();
    mem.init_bytes(0x1000, b"code").unwrap();
    mem.write_bytes(0x8000, b"data").unwrap();

    let mut bytes: Vec<u8> = Vec::new();
    mem.save(&mut bytes).unwrap();

    let restored = UserMemory::new(48);
    let mut source = crate::core::state::SliceSource::new(&bytes);
    restored.load(&mut source).unwrap();

    assert_eq!(restored.mappings(), mem.mappings());
    let mut again: Vec<u8> = Vec::new();
    restored.save(&mut again).unwrap();
    assert_eq!(
        state_hash(&again),
        state_hash(&bytes),
        "a save/load/save round trip must reproduce an identical state hash"
    );
}

#[test]
fn a_snapshot_of_a_different_width_is_refused() {
    let mem = UserMemory::new(48);
    let mut bytes: Vec<u8> = Vec::new();
    mem.save(&mut bytes).unwrap();
    let other = UserMemory::new(32);
    let mut source = crate::core::state::SliceSource::new(&bytes);
    assert!(other.load(&mut source).is_err());
}

// ---------------------------------------------------------------------------
// The clock
// ---------------------------------------------------------------------------

#[test]
fn the_clock_starts_at_zero_and_advances_by_what_ran() {
    let clock = GuestClock::new();
    assert_eq!(clock.ticks(), 0);
    assert_eq!(clock.nanos(), 0);
    clock.advance(1_000);
    assert_eq!(clock.ticks(), 1_000);
    // One tick per nanosecond at the default rate. The nanosecond reading is
    // a *floor* of an exactly tracked fixed-point position (§4.2), so it can
    // sit one nanosecond low; what matters is that the error is bounded and
    // does not accumulate, which is checked below.
    assert!(clock.nanos().abs_diff(1_000) <= 1, "{}", clock.nanos());
    clock.advance(999_999_000);
    assert_eq!(clock.ticks(), 1_000_000_000);
    assert!(
        clock.nanos().abs_diff(1_000_000_000) <= 1,
        "the fixed-point residual must not accumulate: {}",
        clock.nanos()
    );
}

#[test]
fn the_clock_can_jump_to_a_deadline() {
    let clock = GuestClock::new();
    let deadline = clock.at_tick(5_000);
    clock.advance_to(deadline);
    assert_eq!(clock.ticks(), 5_000);
    assert_eq!(clock.tick_of(deadline), 5_000);
    // Jumping backwards does nothing rather than going wrong.
    clock.advance_to(clock.at_tick(10));
    assert_eq!(clock.ticks(), 5_000);
}

#[test]
fn the_clock_round_trips_through_a_snapshot() {
    let clock = GuestClock::new();
    clock.advance(123_456);
    let mut bytes: Vec<u8> = Vec::new();
    clock.save(&mut bytes).unwrap();

    let restored = GuestClock::new();
    let mut source = crate::core::state::SliceSource::new(&bytes);
    restored.load(&mut source).unwrap();
    assert_eq!(restored.ticks(), clock.ticks());
    assert_eq!(restored.nanos(), clock.nanos());

    let mut again: Vec<u8> = Vec::new();
    restored.save(&mut again).unwrap();
    assert_eq!(state_hash(&again), state_hash(&bytes));
}

// ---------------------------------------------------------------------------
// The journal
// ---------------------------------------------------------------------------

#[test]
fn a_live_journal_asks_the_host_and_keeps_nothing() {
    let journal = Journal::new();
    assert_eq!(journal.mode(), JournalMode::Live);
    let answer = journal
        .ask(GlobalTime::ZERO, Tag(7), || Answer::value(42))
        .unwrap();
    assert_eq!(answer.value, 42);
    assert!(journal.is_empty());
}

#[test]
fn a_recorded_run_replays_without_the_host() {
    let journal = Journal::with_mode(JournalMode::Record);
    let at = |n: u64| GlobalTime::from_nanos(n);
    journal
        .ask(at(10), Tag(63), || Answer::with_bytes(5, b"hello".to_vec()))
        .unwrap();
    journal.ask(at(20), Tag(64), || Answer::value(5)).unwrap();
    assert_eq!(journal.len(), 2);

    journal.set_mode(JournalMode::Replay);
    // The closure must not run: a replay has no host, which is the whole
    // point — a recorded run replays in a browser with the files deleted.
    let first = journal
        .ask(at(10), Tag(63), || panic!("replay consulted the host"))
        .unwrap();
    assert_eq!(first.value, 5);
    assert_eq!(first.bytes, b"hello");
    let second = journal
        .ask(at(20), Tag(64), || panic!("replay consulted the host"))
        .unwrap();
    assert_eq!(second.value, 5);
    assert_eq!(journal.remaining(), 0);
}

#[test]
fn a_replay_that_diverges_says_so_at_the_point_it_diverged() {
    let journal = Journal::with_mode(JournalMode::Record);
    journal
        .ask(GlobalTime::from_nanos(10), Tag(63), || Answer::value(1))
        .unwrap();
    journal.set_mode(JournalMode::Replay);

    // A different question.
    let wrong_tag = journal.ask(GlobalTime::from_nanos(10), Tag(64), || Answer::value(1));
    assert!(matches!(wrong_tag, Err(Error::State(_))));

    // The same question at a different virtual time — the guest took a
    // different path to get here.
    journal.set_mode(JournalMode::Replay);
    let wrong_time = journal.ask(GlobalTime::from_nanos(11), Tag(63), || Answer::value(1));
    assert!(matches!(wrong_time, Err(Error::State(_))));

    // And running off the end.
    journal.set_mode(JournalMode::Replay);
    journal
        .ask(GlobalTime::from_nanos(10), Tag(63), || Answer::value(1))
        .unwrap();
    let past_end = journal.ask(GlobalTime::from_nanos(20), Tag(63), || Answer::value(1));
    assert!(matches!(past_end, Err(Error::State(_))));
}

#[test]
fn a_journal_round_trips_through_a_snapshot() {
    let journal = Journal::with_mode(JournalMode::Record);
    journal
        .ask(GlobalTime::from_nanos(10), Tag(63), || {
            Answer::with_bytes(5, b"hello".to_vec())
        })
        .unwrap();
    journal
        .ask(GlobalTime::from_nanos(20), Tag(78), || Answer::value(0))
        .unwrap();

    let mut bytes: Vec<u8> = Vec::new();
    journal.save(&mut bytes).unwrap();

    let restored = Journal::new();
    let mut source = crate::core::state::SliceSource::new(&bytes);
    restored.load(&mut source).unwrap();
    assert_eq!(restored.len(), 2);

    let mut again: Vec<u8> = Vec::new();
    restored.save(&mut again).unwrap();
    assert_eq!(state_hash(&again), state_hash(&bytes));

    restored.set_mode(JournalMode::Replay);
    let first = restored
        .ask(GlobalTime::from_nanos(10), Tag(63), Answer::default)
        .unwrap();
    assert_eq!(first.bytes, b"hello");
}

// ---------------------------------------------------------------------------
// The thread set
// ---------------------------------------------------------------------------

/// A core that consumes its whole budget and never exits. Enough to show what
/// the scheduler does with time, without needing an architecture.
#[derive(Debug, Default)]
struct Spinner {
    ticks: crate::core::sync::AtomicU64,
    mask: crate::core::sync::AtomicU32,
    pc: crate::core::sync::AtomicU64,
    sp: crate::core::sync::AtomicU64,
}

impl crate::core::exec::ExitingCore for Spinner {
    fn exit_mask(&self) -> crate::core::exec::ExitMask {
        crate::core::exec::ExitMask::from_bits(self.mask.load(crate::core::sync::Ordering::Relaxed))
    }
    fn set_exit_mask(&self, mask: crate::core::exec::ExitMask) {
        self.mask
            .store(mask.bits(), crate::core::sync::Ordering::Relaxed);
    }
    fn run_to_exit(&self, budget: crate::core::sched::Budget) -> crate::core::exec::Run {
        self.ticks
            .fetch_add(budget.ticks, crate::core::sync::Ordering::Relaxed);
        crate::core::exec::Run::completed(crate::core::sched::Consumed::new(budget.ticks))
    }
    fn pc(&self) -> u64 {
        self.pc.load(crate::core::sync::Ordering::Relaxed)
    }
    fn set_pc(&self, pc: u64) {
        self.pc.store(pc, crate::core::sync::Ordering::Relaxed);
    }
    fn sp(&self) -> u64 {
        self.sp.load(crate::core::sync::Ordering::Relaxed)
    }
    fn set_sp(&self, sp: u64) {
        self.sp.store(sp, crate::core::sync::Ordering::Relaxed);
    }
}

#[test]
fn an_empty_set_has_nothing_to_run() {
    let threads = ThreadSet::new(Arc::new(GuestClock::new()));
    assert!(threads.is_empty());
    assert!(threads.run_next().is_none());
}

#[test]
fn threads_take_turns_in_id_order() {
    let threads = ThreadSet::new(Arc::new(GuestClock::new()));
    threads.set_quantum(100);
    let a = threads.insert(Arc::new(Spinner::default()));
    let b = threads.insert(Arc::new(Spinner::default()));
    let c = threads.insert(Arc::new(Spinner::default()));

    let order: Vec<_> = (0..6).map(|_| threads.run_next().unwrap().thread).collect();
    assert_eq!(order, alloc::vec![a, b, c, a, b, c]);
    // And time is exactly what ran: six quanta of a hundred ticks.
    assert_eq!(threads.clock().ticks(), 600);
}

#[test]
fn a_blocked_thread_does_not_run() {
    let threads = ThreadSet::new(Arc::new(GuestClock::new()));
    threads.set_quantum(100);
    let a = threads.insert(Arc::new(Spinner::default()));
    let b = threads.insert(Arc::new(Spinner::default()));

    assert!(threads.block(b, None));
    for _ in 0..4 {
        assert_eq!(threads.run_next().unwrap().thread, a);
    }
    assert!(threads.wake(b));
    assert_eq!(threads.run_next().unwrap().thread, b);
}

#[test]
fn every_thread_blocked_with_no_deadline_is_reported_rather_than_spun_on() {
    let threads = ThreadSet::new(Arc::new(GuestClock::new()));
    let a = threads.insert(Arc::new(Spinner::default()));
    threads.block(a, None);
    assert!(
        threads.run_next().is_none(),
        "a deadlock is a condition the consumer must handle, not a spin"
    );
}

#[test]
fn a_sleeping_thread_wakes_by_jumping_virtual_time() {
    let clock = Arc::new(GuestClock::new());
    let threads = ThreadSet::new(Arc::clone(&clock));
    threads.set_quantum(100);
    let a = threads.insert(Arc::new(Spinner::default()));
    let deadline = clock.at_tick(1_000_000);
    threads.block(a, Some(deadline));

    // No host sleep anywhere: time simply *is* the deadline now.
    let stop = threads.run_next().expect("the sleeper wakes");
    assert_eq!(stop.thread, a);
    assert_eq!(clock.ticks(), 1_000_000 + 100);
}

#[test]
fn a_schedule_round_trips_through_a_snapshot() {
    let clock = Arc::new(GuestClock::new());
    let threads = ThreadSet::new(Arc::clone(&clock));
    threads.set_quantum(64);
    let a = threads.insert(Arc::new(Spinner::default()));
    let b = threads.insert(Arc::new(Spinner::default()));
    threads.run_next().unwrap();
    threads.block(b, Some(clock.at_tick(999)));

    let mut bytes: Vec<u8> = Vec::new();
    threads.save(&mut bytes).unwrap();

    // A restore is *insert the cores, then load the schedule*: a `ThreadSet`
    // cannot conjure an `ExitingCore`.
    let restored = ThreadSet::new(Arc::new(GuestClock::new()));
    let ra = restored.insert(Arc::new(Spinner::default()));
    let rb = restored.insert(Arc::new(Spinner::default()));
    assert_eq!((ra, rb), (a, b));
    let mut source = crate::core::state::SliceSource::new(&bytes);
    restored.load(&mut source).unwrap();

    assert_eq!(restored.state(a), threads.state(a));
    assert_eq!(restored.state(b), threads.state(b));
    assert_eq!(restored.quantum(), 64);

    let mut again: Vec<u8> = Vec::new();
    restored.save(&mut again).unwrap();
    assert_eq!(
        state_hash(&again),
        state_hash(&bytes),
        "a save/load/save round trip must reproduce an identical state hash"
    );
}

#[test]
fn a_schedule_naming_a_thread_that_is_not_there_is_refused() {
    let threads = ThreadSet::new(Arc::new(GuestClock::new()));
    threads.insert(Arc::new(Spinner::default()));
    let mut bytes: Vec<u8> = Vec::new();
    threads.save(&mut bytes).unwrap();

    let empty = ThreadSet::new(Arc::new(GuestClock::new()));
    let mut source = crate::core::state::SliceSource::new(&bytes);
    assert!(empty.load(&mut source).is_err());
}

// ---------------------------------------------------------------------------
// A state hash, for the round-trip tests
// ---------------------------------------------------------------------------

/// FNV-1a over a snapshot's bytes.
///
/// The round-trip rule (CLAUDE.md, "Devices") asks for an identical *state
/// hash*, and until `purecrypto` is wired in for snapshot integrity (§4.5) the
/// honest stand-in is any function that changes when the bytes do. Not
/// cryptographic and not claimed to be.
fn state_hash(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in bytes {
        h ^= u64::from(*b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

// ---------------------------------------------------------------------------
// The proof guest: RV64 user mode, `write(1, ..)` then `exit(0)`
// ---------------------------------------------------------------------------

#[cfg(feature = "cpu-riscv")]
mod riscv_guest {
    use super::*;

    use crate::core::exec::{ExitMask, ExitReason, ExitingCore};
    use crate::cpu::riscv::{Config, Hart, csr::Priv};

    /// `addi rd, rs1, imm` — and, with `rs1 = x0`, the `li rd, imm` a
    /// hand-assembled program is mostly made of.
    ///
    /// Volume I, "Integer Register-Immediate Instructions": the I-type layout
    /// is `imm[11:0] | rs1 | funct3 | rd | opcode`.
    const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
        ((imm as u32) << 20) | (rs1 << 15) | (rd << 7) | 0b001_0011
    }

    /// `lui rd, imm20` — U-type, `imm[31:12] | rd | opcode`.
    const fn lui(rd: u32, imm20: u32) -> u32 {
        (imm20 << 12) | (rd << 7) | 0b011_0111
    }

    /// `ecall` — the one encoding that matters here.
    const ECALL: u32 = 0x0000_0073;

    /// Where the program is loaded, and where its stack goes.
    const TEXT: u64 = 0x1_0000;
    const STACK_TOP: u64 = 0x8000_0000;
    const MESSAGE: u64 = 0x2_0000;

    /// The Linux RV64 syscall numbers this test's handler answers.
    ///
    /// Facts about an ABI the *consumer* owns (§2.1) — they are here because
    /// the test is standing in for that consumer, not because rsemu knows
    /// them. Nothing outside this module names them.
    const SYS_WRITE: u64 = 64;
    const SYS_EXIT: u64 = 93;

    /// Load `value` into `rd` with a `lui`/`addi` pair.
    ///
    /// The `addi` immediate is sign extended, so the upper half is
    /// pre-compensated when the lower half has its top bit set — the standard
    /// `li` expansion, and a fact about the encoding rather than a trick.
    fn li(rd: u32, value: u64) -> [u32; 2] {
        let lo = (value & 0xfff) as i32;
        let lo = if lo & 0x800 != 0 { lo - 0x1000 } else { lo };
        let hi = ((value as i64 - i64::from(lo)) >> 12) as u32 & 0xf_ffff;
        [lui(rd, hi), addi(rd, rd, lo)]
    }

    /// The whole guest: `write(1, MESSAGE, len)` then `exit(0)`.
    fn program(len: u64) -> Vec<u32> {
        let mut out = Vec::new();
        out.extend(li(10, 1)); // a0 = fd 1
        out.extend(li(11, MESSAGE)); // a1 = buffer
        out.extend(li(12, len)); // a2 = length
        out.extend(li(17, SYS_WRITE)); // a7 = __NR_write
        out.push(ECALL);
        out.extend(li(10, 0)); // a0 = status 0
        out.extend(li(17, SYS_EXIT)); // a7 = __NR_exit
        out.push(ECALL);
        out
    }

    /// Assemble the guest into a fresh level-3 memory map, and return a hart
    /// that is about to execute it in user mode.
    fn boot(message: &[u8]) -> (Arc<UserMemory>, Arc<Hart>) {
        let mem = Arc::new(UserMemory::new(48));
        mem.map_at(TEXT, 0x1000, Prot::RX, "text").unwrap();
        mem.map_at(MESSAGE, 0x1000, Prot::READ, "rodata").unwrap();
        mem.map_at(STACK_TOP - 0x4000, 0x4000, Prot::RW, "[stack]")
            .unwrap();

        let code: Vec<u8> = program(message.len() as u64)
            .iter()
            .flat_map(|w| w.to_le_bytes())
            .collect();
        mem.init_bytes(TEXT, &code).unwrap();
        mem.init_bytes(MESSAGE, message).unwrap();

        // No PMP, so a user-mode access matching no entry is permitted — the
        // specification's own reading of an implementation with none, and the
        // right shape for a hart with no firmware to program it.
        let cfg = Config {
            pmp_count: 0,
            ..Config::rv64gc()
        }
        .with_reset_vector(TEXT);
        let hart = Arc::new(Hart::new(cfg));
        hart.attach_space(Arc::clone(mem.space()));

        // Two registers is all it takes to start a thread, which is why they
        // are the two on `ExitingCore`.
        hart.set_pc(TEXT);
        hart.set_x(2, STACK_TOP - 0x100);
        // And drop to user mode, where an `ecall` is the guest asking its
        // environment for something rather than a supervisor call.
        let mut csrs = hart.csrs();
        csrs.priv_mode = Priv::User;
        hart.set_csrs(csrs);

        (mem, hart)
    }

    /// The gate: a hand-assembled static program writes to fd 1 and exits,
    /// serviced from outside the core, with no toolchain and no corpus.
    #[test]
    fn a_hand_assembled_guest_writes_to_fd_one_and_exits() {
        let (mem, hart) = boot(b"hello from level 3\n");
        hart.set_exit_mask(ExitMask::USER);

        let clock = Arc::new(GuestClock::new());
        let threads = ThreadSet::new(Arc::clone(&clock));
        let id = threads.insert(Arc::clone(&hart) as Arc<dyn crate::core::exec::ExitingCore>);

        // Everything below this line is the *consumer's* half: it knows what
        // register carries a syscall number and what number 64 means, and
        // rsemu knows neither.
        let mut output: Vec<u8> = Vec::new();
        let mut status = None;
        let mut syscalls = 0;

        while status.is_none() {
            let stop = threads.run_next().expect("the guest is runnable");
            let Some(exit) = stop.exit else { continue };
            assert_eq!(
                exit.reason,
                ExitReason::SYSCALL,
                "the guest faulted at {:#x} (cause {})",
                exit.pc,
                exit.detail
            );
            syscalls += 1;
            assert!(syscalls < 10, "the guest is looping");

            match hart.x(17) {
                SYS_WRITE => {
                    let (fd, buf, len) = (hart.x(10), hart.x(11), hart.x(12));
                    assert_eq!(fd, 1);
                    let mut bytes = alloc::vec![0u8; len as usize];
                    mem.read_bytes(buf, &mut bytes).unwrap();
                    output.extend_from_slice(&bytes);
                    hart.set_x(10, len);
                }
                SYS_EXIT => {
                    status = Some(hart.x(10) as i32);
                    threads.remove(id);
                }
                other => panic!("the guest made an unexpected call {other}"),
            }
        }

        assert_eq!(output, b"hello from level 3\n");
        assert_eq!(status, Some(0));
        assert_eq!(syscalls, 2);
        assert!(threads.is_empty());
        assert!(clock.ticks() > 0, "virtual time advanced by what executed");
    }

    #[test]
    fn the_exit_leaves_the_program_counter_past_the_ecall() {
        let (_mem, hart) = boot(b"x");
        hart.set_exit_mask(ExitMask::USER);
        let run = hart.run_to_exit_ticks(1_000);
        let exit = run.exit.expect("the guest reaches its first ecall");
        assert_eq!(exit.reason, ExitReason::SYSCALL);
        assert_eq!(exit.len, 4);
        assert_eq!(
            hart.pc(),
            exit.pc + 4,
            "a syscall resumes past the instruction, so resuming is the default"
        );
        assert_eq!(exit.resume_pc(), hart.pc());
    }

    #[test]
    fn rewinding_to_the_exit_pc_re_executes_the_call() {
        // The block-and-retry shape a syscall kernel needs: a call that cannot
        // be answered yet is *un*-executed by putting the program counter
        // back, and the guest asks again.
        let (_mem, hart) = boot(b"x");
        hart.set_exit_mask(ExitMask::USER);
        let first = hart.run_to_exit_ticks(1_000).exit.unwrap();
        crate::core::exec::ExitingCore::set_pc(&*hart, first.pc);
        let second = hart.run_to_exit_ticks(1_000).exit.unwrap();
        assert_eq!(first.pc, second.pc);
        assert_eq!(first.reason, second.reason);
        assert_eq!(hart.x(17), SYS_WRITE, "the arguments are unchanged too");
    }

    #[test]
    fn without_the_mask_the_ecall_vectors_into_the_guest_as_it_always_did() {
        // The seam is opt-in: an unmasked hart is exactly the level-1 core
        // every machine in `machines/` runs today.
        let (_mem, hart) = boot(b"x");
        assert_eq!(hart.exit_mask(), ExitMask::NONE);
        // Step until something traps. `mcause` starts at zero and this guest's
        // first trap is its `ecall`.
        for _ in 0..100 {
            let (_, exit) = hart.step_to_exit();
            assert!(exit.is_none(), "an unmasked hart never exits");
            if hart.csrs().mcause != 0 {
                break;
            }
        }
        assert_eq!(
            hart.csrs().mcause,
            crate::cpu::riscv::csr::cause::ECALL_U,
            "the trap went where the architecture says it goes"
        );
    }

    #[test]
    fn a_fault_exits_with_the_address_and_the_access_that_caused_it() {
        let (_mem, hart) = boot(b"x");
        hart.set_exit_mask(ExitMask::USER);
        // Point the guest at nothing.
        hart.set_pc(0x7000_0000);
        let exit = hart.run_to_exit_ticks(100).exit.expect("a fetch faults");
        assert_eq!(exit.reason, ExitReason::FAULT);
        assert_eq!(exit.access, crate::core::exec::Access::Execute);
        assert_eq!(exit.pc, 0x7000_0000);
        assert_eq!(
            hart.pc(),
            0x7000_0000,
            "a fault resumes *at* the instruction, so mapping a page and \
             carrying on works"
        );
    }

    #[test]
    fn a_store_fault_names_a_write() {
        let (mem, hart) = boot(b"x");
        hart.set_exit_mask(ExitMask::USER);
        // `sd x0, 0(x0)` — a store to address zero, which is never mapped.
        let sd = 0x0000_3023u32;
        mem.init_bytes(TEXT, &sd.to_le_bytes()).unwrap();
        hart.set_pc(TEXT);
        let exit = hart.run_to_exit_ticks(100).exit.expect("the store faults");
        assert_eq!(exit.reason, ExitReason::FAULT);
        assert_eq!(exit.access, crate::core::exec::Access::Write);
        assert_eq!(exit.address, 0);
    }

    /// A snapshot taken **mid-process** — between the two syscalls — restores
    /// into a fresh set of objects and the program finishes normally.
    ///
    /// This is the shape phase 5b's product gate asks for, reduced to what
    /// rsemu owns. It works because a syscall exit is a safe point
    /// (`core::exec`): the guest has finished an instruction, so what has to be
    /// written down is four ordinary `save` calls and nothing about a
    /// half-executed anything.
    #[test]
    fn a_snapshot_taken_between_two_syscalls_restores_and_continues() {
        use crate::core::device::Device;
        use crate::core::state::{MachineShape, Migrations, StateReader, StateWriter};

        let mut shape = MachineShape::new();
        shape.add_device("/cpu0", "cpu.riscv").unwrap();
        shape.add_device("/mem", "usermode.memory").unwrap();
        shape.add_device("/clock", "usermode.clock").unwrap();
        shape.add_device("/threads", "usermode.threads").unwrap();

        // Run until the first syscall, and stop there.
        let (mem, hart) = boot(b"resumed\n");
        hart.set_exit_mask(ExitMask::USER);
        let clock = Arc::new(GuestClock::new());
        let threads = ThreadSet::new(Arc::clone(&clock));
        threads.insert(Arc::clone(&hart) as Arc<dyn ExitingCore>);
        let stop = threads.run_next().expect("runnable");
        let exit = stop.exit.expect("the first ecall");
        assert_eq!(exit.reason, ExitReason::SYSCALL);
        assert_eq!(hart.x(17), SYS_WRITE);

        // Write everything down, mid-process.
        let mut writer = StateWriter::new(shape);
        hart.save(&mut writer.chunk("/cpu0", "cpu.riscv", 1).unwrap())
            .unwrap();
        mem.save(&mut writer.chunk("/mem", "usermode.memory", 1).unwrap())
            .unwrap();
        clock
            .save(&mut writer.chunk("/clock", "usermode.clock", 1).unwrap())
            .unwrap();
        threads
            .save(&mut writer.chunk("/threads", "usermode.threads", 1).unwrap())
            .unwrap();
        let bytes = writer.to_vec().unwrap();

        // A fresh everything, built the way a consumer would build it from a
        // configuration, then loaded.
        let (mem2, hart2) = boot(b"this text is overwritten by the snapshot");
        hart2.set_exit_mask(ExitMask::USER);
        let clock2 = Arc::new(GuestClock::new());
        let threads2 = ThreadSet::new(Arc::clone(&clock2));
        threads2.insert(Arc::clone(&hart2) as Arc<dyn ExitingCore>);

        let reader = StateReader::new(&bytes).unwrap();
        let migrations = Migrations::new();
        let cpu = reader.load("/cpu0", "cpu.riscv", 1, &migrations).unwrap();
        hart2.load(&mut cpu.reader()).unwrap();
        let saved_mem = reader
            .load("/mem", "usermode.memory", 1, &migrations)
            .unwrap();
        mem2.load(&mut saved_mem.reader()).unwrap();
        let saved_clock = reader
            .load("/clock", "usermode.clock", 1, &migrations)
            .unwrap();
        clock2.load(&mut saved_clock.reader()).unwrap();
        let saved_threads = reader
            .load("/threads", "usermode.threads", 1, &migrations)
            .unwrap();
        threads2.load(&mut saved_threads.reader()).unwrap();

        assert_eq!(clock2.ticks(), clock.ticks());
        assert_eq!(hart2.pc(), hart.pc());
        assert_eq!(hart2.x(17), SYS_WRITE);

        // Now finish the program on the restored side: answer the syscall the
        // snapshot was taken in the middle of, and carry on.
        let mut output: Vec<u8> = Vec::new();
        let len = hart2.x(12);
        let mut buf = alloc::vec![0u8; len as usize];
        mem2.read_bytes(hart2.x(11), &mut buf).unwrap();
        output.extend_from_slice(&buf);
        hart2.set_x(10, len);

        let stop = threads2.run_next().expect("runnable");
        let exit = stop.exit.expect("the exit call");
        assert_eq!(exit.reason, ExitReason::SYSCALL);
        assert_eq!(hart2.x(17), SYS_EXIT);
        assert_eq!(hart2.x(10), 0);
        assert_eq!(
            output, b"resumed\n",
            "the restored map carries the guest's own data, not the fresh one's"
        );
    }

    #[test]
    fn a_run_is_reproducible_and_replays_without_a_host() {
        // The determinism gate in miniature: the same program run twice makes
        // the same calls at the same virtual instants, and a recorded run
        // replays with the closure that reached the host removed entirely.
        let transcript = |journal: &Journal| {
            let (mem, hart) = boot(b"deterministic\n");
            hart.set_exit_mask(ExitMask::USER);
            let clock = Arc::new(GuestClock::new());
            let threads = ThreadSet::new(Arc::clone(&clock));
            threads.insert(Arc::clone(&hart) as Arc<dyn crate::core::exec::ExitingCore>);
            let mut log: Vec<(u64, u64, u64)> = Vec::new();
            loop {
                let stop = threads.run_next().expect("runnable");
                let Some(exit) = stop.exit else { continue };
                assert_eq!(exit.reason, ExitReason::SYSCALL);
                let nr = hart.x(17);
                log.push((clock.nanos(), nr, hart.x(10)));
                if nr == SYS_EXIT {
                    return log;
                }
                // The write's *result* is the external answer, so it is the
                // thing the journal owns. In replay the closure never runs.
                let len = hart.x(12);
                let answer = journal
                    .ask(clock.now(), Tag(nr as u32), || {
                        let mut bytes = alloc::vec![0u8; len as usize];
                        mem.read_bytes(hart.x(11), &mut bytes).unwrap();
                        Answer::with_bytes(len, bytes)
                    })
                    .unwrap();
                hart.set_x(10, answer.value);
            }
        };

        let recording = Journal::with_mode(JournalMode::Record);
        let first = transcript(&recording);
        let second = transcript(&Journal::new());
        assert_eq!(first, second, "two runs of one program must agree exactly");

        recording.set_mode(JournalMode::Replay);
        let replayed = transcript(&recording);
        assert_eq!(replayed, first);
        assert_eq!(recording.remaining(), 0);
    }
}
