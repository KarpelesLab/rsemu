//! The `parallel` threading mode, on a machine with two CPUs (`ROADMAP.md`
//! §4.2).
//!
//! `deterministic` runs everything; `parallel` is the mode §4.2 calls *"the
//! default for interactive use"*, and until now it returned
//! `ModeUnimplemented`. What this file measures rather than asserts:
//!
//! * every runnable really lands on a **host thread of its own** — one stays on
//!   the thread that drives the round, the rest are dispatched — and all of
//!   them are inside their `run` call at the same instant, proved by a
//!   rendezvous that only completes if they overlap;
//! * `machines/tests/heterogeneous.machine` — a RISC-V hart and a 6502, two
//!   spaces, differing endianness, one shared RAM region — **runs**, and the
//!   two guests see each other through the shared region;
//! * the ranked lock ladder holds while two CPU session mutexes (both
//!   [`LockRank::BUS`]) are held on two threads at once, which is the first
//!   time that has ever happened;
//! * a lazily-advanced device reached by two CPUs at once is *contention*, not
//!   the re-entrancy `SchedError::LazyDeviceBusy` reports under one thread;
//! * a world stop reaches **real CPU interpreters**: both cores on that board
//!   unwind at their next instruction rather than at the end of the round;
//! * a snapshot taken with the world stopped restores and continues;
//! * and [`Machine::state_hash`] **refuses** outside a deterministic mode, so a
//!   golden cannot be blessed against a parallel run by accident.
//!
//! [`LockRank::BUS`]: rsemu::core::sync::LockRank::BUS
//! [`Machine::state_hash`]: rsemu::machine::Machine::state_hash

#![cfg(all(feature = "cpu-riscv", feature = "cpu-mos6502"))]

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::thread::ThreadId;
use std::time::{Duration, Instant};

use rsemu::core::clock::{ClockForest, GlobalTime, Rational};
use rsemu::core::sched::{
    AccessKind, Budget, Consumed, LazyDevice, LazyHandle, Runnable, Scheduler, SchedulerConfig,
    ThreadingMode,
};
use rsemu::core::space::MemAttrs;
use rsemu::core::sync::Mutex;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// The fixture `ROADMAP.md` §5 and §13's phase-2 gate both name.
const HETEROGENEOUS: &str = include_str!("../machines/tests/heterogeneous.machine");

// ---------------------------------------------------------------------------
// the two guest programs
// ---------------------------------------------------------------------------

/// The RISC-V half, assembled by hand from *The RISC-V Instruction Set Manual,
/// Volume I: Unprivileged ISA*, chapter 2 (RV32I) and chapter 4 (RV64I).
///
/// ```text
///   1000  lui  t0, 0x100          # t0 = 0x00100000, the shared region
///   1004  addi t1, x0, 0
///   1008  addi t1, t1, 1          # loop:
///   100c  sd   t1, 0x100(t0)      # a 64-bit counter of its own
///   1010  lbu  t2, 0(t0)          # the 6502's counter, low byte
///   1014  beq  t2, x0, loop       # nothing there yet
///   1018  addi t3, x0, 0x55
///   101c  sb   t3, 0x108(t0)      # "I have seen the 6502"
///   1020  j    loop
/// ```
///
/// `lbu` is a byte access, so the byte the 6502 wrote means the same thing on
/// either side of the endianness difference — which is the point of reading
/// each other's *bytes* rather than each other's words.
const RV_CODE: &[u32] = &[
    0x0010_02b7, // lui  t0, 0x100
    0x0000_0313, // addi t1, x0, 0
    0x0013_0313, // addi t1, t1, 1
    0x1062_8023, // sd   t1, 0x100(t0)
    0x0002_c383, // lbu  t2, 0(t0)
    0xfe03_8ae3, // beq  t2, x0, -12
    0x0550_0e13, // addi t3, x0, 0x55
    0x11c2_8423, // sb   t3, 0x108(t0)
    0xfe9f_f06f, // j    -24
];

/// The 6502 half, assembled by hand from the *MCS6500 Family Programming
/// Manual*'s opcode table.
///
/// ```text
///   e000  lda #$00 / sta $4000 / sta $4001 / sta $4002
///   e00b  inc $4000            ; loop:
///   e00e  bne +3
///   e010  inc $4001
///   e013  lda $4108            ; the RISC-V's flag
///   e016  beq loop
///   e018  lda #$aa
///   e01a  sta $4002            ; "I have seen the RISC-V"
///   e01d  jmp loop
/// ```
///
/// The shared window sits at `$4000`, so `$4000`/`$4001` are shared bytes 0 and
/// 1 and `$4108` is shared byte `0x108` — the byte the hart writes above.
const MOS_CODE: &[u8] = &[
    0xa9, 0x00, // lda #$00
    0x8d, 0x00, 0x40, // sta $4000
    0x8d, 0x01, 0x40, // sta $4001
    0x8d, 0x02, 0x40, // sta $4002
    0xee, 0x00, 0x40, // loop: inc $4000
    0xd0, 0x03, //       bne +3
    0xee, 0x01, 0x40, //       inc $4001
    0xad, 0x08, 0x41, //       lda $4108
    0xf0, 0xf3, //       beq loop
    0xa9, 0xaa, //       lda #$aa
    0x8d, 0x02, 0x40, //       sta $4002
    0x4c, 0x0b, 0xe0, //       jmp loop
];

/// Where the 6502 fetches its reset vector, as an offset into an 8 KiB ROM
/// mapped at `$e000`.
const RESET_VECTOR: usize = 0x1ffc;

/// The 6502's ROM image: the program at the bottom, `$e000` in the vector.
fn mos_rom() -> Vec<u8> {
    let mut rom = vec![0u8; 8 * 1024];
    rom[..MOS_CODE.len()].copy_from_slice(MOS_CODE);
    rom[RESET_VECTOR] = 0x00;
    rom[RESET_VECTOR + 1] = 0xe0;
    rom
}

/// The hart's ROM image: the program at the bottom of a 4 KiB ROM mapped at
/// `0x1000`, which is where the machine file resets it to.
fn rv_rom() -> Vec<u8> {
    let mut rom = vec![0u8; 4 * 1024];
    for (i, word) in RV_CODE.iter().enumerate() {
        rom[i * 4..i * 4 + 4].copy_from_slice(&word.to_le_bytes());
    }
    rom
}

/// Build the fixture in `mode`, with `workers` pool threads.
fn board(mode: ThreadingMode, workers: usize) -> Machine {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.scheduler.mode = mode;
    options.realize.scheduler.workers = workers;
    options.realize.media.insert("rvcode", rv_rom());
    options.realize.media.insert("moscode", mos_rom());
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build("heterogeneous.machine", HETEROGENEOUS, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the fixture does not realize: {e}"),
    }
}

/// One byte of the shared region, read through the 6502's window so the read
/// crosses the endianness boundary the way the guest's does.
fn shared(m: &Machine, offset: u64) -> u64 {
    m.space("big")
        .expect("the big-endian space")
        .read(0x4000 + offset, Width::U8, MemAttrs::DEFAULT)
        .expect("a mapped byte")
}

// ---------------------------------------------------------------------------
// two threads, and the proof that it is two
// ---------------------------------------------------------------------------

/// A runnable that records the host thread it ran on and refuses to finish
/// until every other one has started.
///
/// The rendezvous is the whole test. Recording several thread ids over many
/// rounds would only show that *some* round used a second worker; waiting for
/// the others shows they were inside their `run` calls **at the same instant**,
/// which is what "a thread per CPU" means. If the round serialised them, no
/// wait could ever be satisfied — so the bound below is a deadline for the
/// failure, not a tolerance.
#[derive(Debug)]
struct Rendezvous {
    want: usize,
    seen: Arc<AtomicUsize>,
    threads: Arc<Mutex<Vec<ThreadId>>>,
    all_here: Arc<AtomicUsize>,
}

impl Runnable for Rendezvous {
    fn run(&mut self, budget: Budget) -> Consumed {
        self.threads.lock().push(std::thread::current().id());
        self.seen.fetch_add(1, Ordering::AcqRel);
        let deadline = Instant::now() + Duration::from_secs(5);
        while self.seen.load(Ordering::Acquire) < self.want && Instant::now() < deadline {
            std::hint::spin_loop();
        }
        if self.seen.load(Ordering::Acquire) >= self.want {
            self.all_here.fetch_add(1, Ordering::AcqRel);
        }
        Consumed::new(budget.ticks)
    }
}

#[test]
fn every_runnable_is_inside_its_run_call_on_a_thread_of_its_own() {
    // Three runnables, three oscillators. Three rather than two so the claim is
    // unambiguous: a round keeps **one** runnable on the driving thread and
    // dispatches the rest, so three runnables means two worker threads plus
    // this one, and "they all overlapped" cannot be satisfied by any two of
    // them taking turns.
    const CORES: usize = 3;

    let mut forest = ClockForest::new();
    let mut domains = Vec::new();
    for i in 0..CORES {
        // One runnable per tree: two on one oscillator share its unit counter,
        // so the second would be handed a budget of zero and never run at all.
        let osc = forest
            .add_oscillator(&format!("xtal{i}"), Rational::integer(1_000_000))
            .unwrap();
        domains.push(forest.add_domain(&format!("core{i}"), osc, 1, 1).unwrap());
    }

    let seen = Arc::new(AtomicUsize::new(0));
    let threads = Arc::new(Mutex::new(Vec::new()));
    let all_here = Arc::new(AtomicUsize::new(0));

    let mut sched = Scheduler::new(
        forest,
        SchedulerConfig {
            mode: ThreadingMode::Parallel,
            workers: CORES,
            ..SchedulerConfig::default()
        },
    );
    assert_eq!(
        sched.pool().expect("parallel mode builds a pool").workers(),
        CORES,
        "this host refused a worker thread; the rest of the test is meaningless without it"
    );
    for domain in domains {
        sched.add_runnable(
            domain,
            Box::new(Rendezvous {
                want: CORES,
                seen: Arc::clone(&seen),
                threads: Arc::clone(&threads),
                all_here: Arc::clone(&all_here),
            }),
        );
    }

    sched.run_quantum().expect("a parallel round");

    // `ThreadId` is only `Eq`, so distinctness is counted rather than set-ed.
    let ids = threads.lock().clone();
    assert_eq!(
        all_here.load(Ordering::Acquire),
        CORES,
        "the runnables never all overlapped: the round serialised them"
    );
    assert_eq!(ids.len(), CORES, "every runnable ran");
    for (i, a) in ids.iter().enumerate() {
        for b in &ids[i + 1..] {
            assert_ne!(a, b, "two runnables shared a host thread");
        }
    }
    let here = std::thread::current().id();
    assert_eq!(
        ids.iter().filter(|id| **id == here).count(),
        1,
        "exactly one runnable stays on the thread that drives the round; the rest are dispatched"
    );
}

// ---------------------------------------------------------------------------
// a contended lazily-advanced device
// ---------------------------------------------------------------------------

/// A device whose catch-up takes long enough that two threads reaching it at
/// once really do collide.
#[derive(Debug)]
struct SlowDevice {
    tick: u64,
    /// How many times catch-up actually advanced it.
    advances: Arc<AtomicU64>,
}

impl LazyDevice for SlowDevice {
    fn current_tick(&self) -> u64 {
        self.tick
    }

    fn advance_to(&mut self, tick: u64) {
        // Long enough to be caught in the act, and it is real work rather than
        // a sleep, because nothing below `host/` may sleep.
        let mut sum = 0u64;
        for i in 0..2_000u64 {
            sum = sum.wrapping_add(i);
        }
        std::hint::black_box(sum);
        self.tick = tick;
        self.advances.fetch_add(1, Ordering::Relaxed);
    }
}

/// A runnable that catches one shared device up on every tick it publishes.
#[derive(Debug)]
struct Sampler {
    handle: LazyHandle,
    errors: Arc<AtomicU64>,
}

impl Runnable for Sampler {
    fn run(&mut self, budget: Budget) -> Consumed {
        for _ in 0..64 {
            if self.handle.sync(AccessKind::Guest).is_err() {
                self.errors.fetch_add(1, Ordering::Relaxed);
            }
        }
        Consumed::new(budget.ticks)
    }
}

#[test]
fn two_cpus_reaching_one_lazy_device_is_contention_not_re_entrancy() {
    let mut forest = ClockForest::new();
    let a = forest
        .add_oscillator("a", Rational::integer(1_000_000))
        .unwrap();
    let b = forest
        .add_oscillator("b", Rational::integer(1_000_000))
        .unwrap();
    let da = forest.add_domain("da", a, 1, 1).unwrap();
    let db = forest.add_domain("db", b, 1, 1).unwrap();

    let mut sched = Scheduler::new(
        forest,
        SchedulerConfig {
            mode: ThreadingMode::Parallel,
            workers: 2,
            ..SchedulerConfig::default()
        },
    );
    let advances = Arc::new(AtomicU64::new(0));
    let dev = sched.add_lazy_device(
        da,
        Box::new(SlowDevice {
            tick: 0,
            advances: Arc::clone(&advances),
        }),
    );
    let handle = sched.lazy_handle(dev).unwrap();
    let errors = Arc::new(AtomicU64::new(0));
    for domain in [da, db] {
        sched.add_runnable(
            domain,
            Box::new(Sampler {
                handle: handle.clone(),
                errors: Arc::clone(&errors),
            }),
        );
    }

    for _ in 0..40 {
        sched.run_quantum().expect("a parallel round");
    }
    assert_eq!(
        errors.load(Ordering::Relaxed),
        0,
        "a catch-up that found the slot taken by the *other thread* reported re-entrancy"
    );
    assert!(advances.load(Ordering::Relaxed) > 0, "the device advanced");
}

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// Run until both guests have seen each other, or give up.
///
/// Returns the virtual time it took. Neither guest's progress is a function of
/// the other's *rate* — each spins until the other's byte arrives — so this
/// terminates in both threading modes, which is exactly what makes it a fair
/// comparison between them.
fn run_until_both_saw_each_other(m: &mut Machine) -> GlobalTime {
    let start = m.now();
    for _ in 0..200 {
        m.run_for(GlobalTime::from_nanos(1_000_000))
            .expect("a round");
        if shared(m, 0x108) == 0x55 && shared(m, 2) == 0xaa {
            return m.now().saturating_sub(start);
        }
    }
    panic!(
        "after 200 ms of virtual time: 6502 counter {:#04x}{:#04x}, riscv flag {:#04x}, \
         6502 flag {:#04x}",
        shared(m, 1),
        shared(m, 0),
        shared(m, 0x108),
        shared(m, 2)
    );
}

#[test]
fn the_heterogeneous_board_runs_both_cpus_in_parallel() {
    let mut m = board(ThreadingMode::Parallel, 2);
    m.reset(rsemu::core::device::ResetKind::Cold);
    run_until_both_saw_each_other(&mut m);

    // Both guests ran, and each saw the other through one RAM object mapped
    // into two spaces of different widths and different endianness.
    assert_eq!(shared(&m, 0x108), 0x55, "the hart saw the 6502");
    assert_eq!(shared(&m, 2), 0xaa, "the 6502 saw the hart");
    let counter = shared(&m, 0) | (shared(&m, 1) << 8);
    assert!(counter > 0, "the 6502's own counter moved");
    assert!(
        m.now() > GlobalTime::ZERO,
        "virtual time moved: {:?}",
        m.now()
    );
}

/// Every runnable device's clock-domain tick count, in device order.
fn tick_counts(m: &Machine) -> Vec<u64> {
    m.devices()
        .iter()
        .filter(|d| d.runnable().is_some())
        .filter_map(|d| d.domain())
        .map(|domain| m.clocks().ticks(domain).expect("a tick count"))
        .collect()
}

#[test]
fn a_real_core_unwinds_at_its_next_instruction_when_the_flag_is_raised() {
    // The safe point on actual CPU interpreters rather than test doubles. Both
    // cores on this board consult their exit flag between instructions, so a
    // round taken with the world stopped consumes a handful of ticks instead of
    // the whole budget.
    let mut m = board(ThreadingMode::Parallel, 2);
    m.reset(rsemu::core::device::ResetKind::Cold);
    m.run_for(GlobalTime::from_nanos(5_000_000)).expect("a run");

    let free = tick_counts(&m);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("a round");
    let after_free = tick_counts(&m);

    let guard = m.stop_the_world();
    let stopped = tick_counts(&m);
    m.run_for(GlobalTime::from_nanos(1_000_000))
        .expect("a round under the guard");
    let after_stopped = tick_counts(&m);
    drop(guard);

    for (i, (&before_free, &before_stopped)) in free.iter().zip(&stopped).enumerate() {
        let ran_free = after_free[i] - before_free;
        let ran_stopped = after_stopped[i] - before_stopped;
        assert!(ran_free > 0, "runnable {i} did nothing to compare against");
        assert!(
            ran_stopped * 4 < ran_free,
            "runnable {i} executed {ran_stopped} ticks under a world stop against {ran_free} \
             without one; it is not consulting its exit flag"
        );
    }
}

#[test]
fn the_same_board_runs_the_same_way_in_deterministic_mode() {
    // Not the same *state hash* — §4.2 says parallel gives that up — but the
    // same observable outcome, which is what makes the parallel result mean
    // something rather than being a run of its own private machine.
    let mut m = board(ThreadingMode::Deterministic, 0);
    m.reset(rsemu::core::device::ResetKind::Cold);
    run_until_both_saw_each_other(&mut m);
    assert_eq!(shared(&m, 0x108), 0x55);
    assert_eq!(shared(&m, 2), 0xaa);
}

#[test]
fn deterministic_mode_is_still_bit_reproducible_on_this_board() {
    let mut hashes = Vec::new();
    for _ in 0..3 {
        let mut m = board(ThreadingMode::Deterministic, 0);
        m.reset(rsemu::core::device::ResetKind::Cold);
        m.run_for(GlobalTime::from_nanos(20_000_000))
            .expect("a run");
        hashes.push(m.state_hash().expect("deterministic mode hashes"));
    }
    assert_eq!(hashes[0], hashes[1]);
    assert_eq!(hashes[1], hashes[2]);
}

#[test]
fn a_state_hash_is_refused_outside_a_deterministic_mode() {
    let m = board(ThreadingMode::Parallel, 2);
    let err = m
        .state_hash()
        .expect_err("a parallel state hash is a sample, not a baseline");
    let text = err.to_string();
    assert!(text.contains("parallel"), "{text}");
    assert!(text.contains("reproducible"), "{text}");
    // The escape hatch still works, and its name is the documentation.
    assert!(m.nondeterministic_state_hash().is_ok());
}

#[test]
fn a_snapshot_taken_with_the_world_stopped_restores_and_continues() {
    let mut m = board(ThreadingMode::Parallel, 2);
    m.reset(rsemu::core::device::ResetKind::Cold);
    run_until_both_saw_each_other(&mut m);

    let (bytes, before) = {
        // The safe-point protocol: every runnable's exit flag is raised, the
        // generation is bumped, and the pool is quiesced. Nothing is executing
        // while the guard is alive, which is what a snapshot needs (§4.7).
        let guard = m.stop_the_world();
        assert!(guard.generation() >= 1);
        assert!(m.safe_point().stop_requested());
        (
            m.save().expect("a snapshot"),
            m.nondeterministic_state_hash().expect("a hash"),
        )
    };
    assert!(
        !m.safe_point().stop_requested(),
        "the guard let the world go"
    );

    let mut restored = board(ThreadingMode::Parallel, 2);
    restored.load(&bytes).expect("the snapshot loads");
    assert_eq!(
        restored.nondeterministic_state_hash().expect("a hash"),
        before,
        "a restored machine is the machine that was saved"
    );

    // And it keeps running from there, on two threads, rather than merely
    // deserialising.
    let counter_before = shared(&restored, 0) | (shared(&restored, 1) << 8);
    restored
        .run_for(GlobalTime::from_nanos(20_000_000))
        .expect("a run after restore");
    let counter_after = shared(&restored, 0) | (shared(&restored, 1) << 8);
    assert_ne!(
        counter_after, counter_before,
        "the restored machine executed"
    );
}

#[test]
fn the_lock_ladder_holds_with_two_cpus_inside_it_at_once() {
    // Both CPUs hold their own session mutex — `LockRank::BUS` — while
    // dispatching into the shared region's store, which is the first time two
    // threads have ever been inside that band together. The debug rank tracker
    // is thread-local wherever `std` is reachable, so a violation is a panic
    // here rather than a hang somewhere else; repetition is the test, because
    // one clean run of a threading test says almost nothing.
    for _ in 0..8 {
        let mut m = board(ThreadingMode::Parallel, 2);
        m.reset(rsemu::core::device::ResetKind::Cold);
        run_until_both_saw_each_other(&mut m);
    }
}

#[test]
fn a_parallel_machine_with_no_workers_still_runs() {
    // The no-threads browser build and bare metal both land here: the pool has
    // nowhere to put a job, so it runs it inline. §11.3 calls that a supported
    // configuration rather than a fallback, so it has to actually work.
    let mut m = board(ThreadingMode::Parallel, 0);
    assert_eq!(
        m.scheduler().pool().expect("a pool").workers(),
        0,
        "no workers were asked for and none were made"
    );
    m.reset(rsemu::core::device::ResetKind::Cold);
    run_until_both_saw_each_other(&mut m);
    assert_eq!(shared(&m, 0x108), 0x55);
    assert_eq!(shared(&m, 2), 0xaa);
}

// ---------------------------------------------------------------------------
// a wire between two runnables, on two threads
// ---------------------------------------------------------------------------

/// The Apple 1 is the other two-runnable machine in the tree, and it is the one
/// where the two runnables are **wired together**: the PIA sits on its own
/// 60 Hz oscillator, paces the display, and drives the 6502's IRQ line.
///
/// The heterogeneous board proves two CPUs sharing memory. This proves the
/// other half of the ladder — `LockRank::DEVICE` reaching `LockRank::WIRE` on a
/// worker thread while the core holds `LockRank::BUS` on another — which is the
/// edge the re-entrancy contract exists for and which had never had two threads
/// in it.
#[cfg(feature = "machine-apple1")]
mod wired {
    use super::*;

    fn apple1(mode: ThreadingMode, workers: usize) -> Machine {
        let entry = catalog::machine("apple1").expect("this build ships apple1");
        let mut options = catalog::build_options().expect("the catalog agrees with itself");
        options.realize.scheduler.mode = mode;
        options.realize.scheduler.workers = workers;
        // RSMON, rsemu's own monitor: the board's `rom` slot has to be bound or
        // the 6502 fetches its reset vector out of nothing.
        options
            .realize
            .media
            .insert("rom", rsemu::dev::apple1::RSMON.as_slice());
        let registry = catalog::registry().expect("a registry");
        rsemu::machine::build(entry.name, entry.source, &registry, &options)
            .expect("the board realizes")
    }

    #[test]
    fn a_wire_between_two_runnables_survives_two_threads() {
        // Repeated, because a threading test that passes once has said almost
        // nothing. Any rank violation is a panic on the offending thread, which
        // the pool delivers at `join`.
        for _ in 0..12 {
            let mut m = apple1(ThreadingMode::Parallel, 2);
            m.reset(rsemu::core::device::ResetKind::Cold);
            m.run_for(GlobalTime::from_nanos(20_000_000))
                .expect("a parallel run");
            let cpu = m
                .device("cpu")
                .expect("the core")
                .domain()
                .expect("a domain");
            assert!(
                m.clocks().ticks(cpu).expect("a tick count") > 0,
                "the 6502 executed"
            );
        }
    }

    #[test]
    fn the_same_board_is_still_bit_identical_in_deterministic_mode() {
        // The regression that matters: nothing above changed what the default
        // mode computes.
        let mut hashes = Vec::new();
        for _ in 0..3 {
            let mut m = apple1(ThreadingMode::Deterministic, 0);
            m.reset(rsemu::core::device::ResetKind::Cold);
            m.run_for(GlobalTime::from_nanos(200_000_000))
                .expect("a run");
            hashes.push(m.state_hash().expect("deterministic mode hashes"));
        }
        assert_eq!(hashes[0], hashes[1]);
        assert_eq!(hashes[1], hashes[2]);
    }
}
