//! A **whole machine** running its two processors at once, selected by the
//! machine file rather than by a flag (`ROADMAP.md` §4.2).
//!
//! `tests/parallel_threading.rs` proves the scheduler's half — that runnables
//! land on host threads of their own, that the safe point reaches real
//! interpreters, that a snapshot survives the world stopping. It does it on
//! `machines/tests/heterogeneous.machine`, which is two cores on **two**
//! crystals: the configuration the clock model was designed for, and not the
//! one any SMP board has.
//!
//! This file is the other one. `machines/tests/smp-parallel.machine` is two
//! RV64 harts on **one** crystal sharing one RAM — the shape of
//! `riscv-virt-smp`, `arm64-virt-smp`, `pc-at-smp` and `q35-linux-smp` — and it
//! carries a `threading parallel` statement of its own, so what is tested here
//! is the path a user takes: a board that says what it is, built with nobody
//! passing a mode.
//!
//! # What it asserts
//!
//! * **The file chose the mode.** Nothing in [`board`] names a threading mode;
//!   the machine comes back in `parallel` anyway, and a
//!   [`Machine::state_hash`] taken from it is refused.
//! * **A run still overrides a board.** `RealizeOptions::with_threading` puts
//!   the same file back into `deterministic`, which is what keeps the
//!   regression suite's promise available on a board that would rather be fast.
//! * **Both processors execute**, on one oscillator, in the same round — each
//!   keeps a private counter and both of them move.
//! * **`AMOADD` loses nothing** across two real harts on a real board. That is
//!   `tests/riscv_amo_atomicity.rs`'s claim, moved from two hand-spawned host
//!   threads onto the machine a user actually runs; it is a strictly stronger
//!   statement, because everything between the guest instruction and the RAM
//!   is now in the picture.
//! * **The degenerate case is pinned rather than described.** One crystal and
//!   two runnables is outside the clock model in both modes
//!   (`Scheduler::ticks_until_after`), and what saves it is
//!   [`SchedulerConfig::max_ticks_per_quantum`] capping the first runnable's
//!   share. Raise the cap above the round's whole span and the second hart
//!   gets a budget of zero — for ever. That is a real limitation of every
//!   shipped `-smp` board and it has a test of its own here, so it cannot
//!   quietly stop being true in either direction.
//!
//! # What it is not
//!
//! Not a memory-model gate. A lost `AMOADD` update is a value no interleaving
//! could have produced, so any host that runs the two harts at once shows it
//! (`tests/riscv_amo_atomicity.rs` argues that at length). A missing *barrier*
//! is not like that: an x86-64 host hides the reordering before and after the
//! fix, so nothing here can gate ordering, and `tests/memory_model_litmus.rs`
//! remains where that is measured.
//!
//! [`Machine::state_hash`]: rsemu::machine::Machine::state_hash
//! [`SchedulerConfig::max_ticks_per_quantum`]: rsemu::core::sched::SchedulerConfig::max_ticks_per_quantum

// `std` because `parallel` is a threading mode and a `no_std` build has no
// threads — the same reason `parallel_threading.rs` gives.
#![cfg(all(feature = "cpu-riscv", feature = "std"))]

use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::sched::ThreadingMode;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::machine::{Machine, catalog};

/// The fixture: two harts, one crystal, `threading parallel` in the file.
const SMP_PARALLEL: &str = include_str!("../machines/tests/smp-parallel.machine");

/// Where the fixture maps its RAM.
const RAM: u64 = 0x0010_0000;

/// The word both harts increment atomically.
const SHARED: u64 = RAM;

/// The word both harts increment **plainly** — a load, an add and a store on a
/// word the architecture promises nothing about.
///
/// This is the witness, and it is here for the reason
/// `tests/riscv_amo_atomicity.rs` gives: an equality that holds because the
/// two harts never collided proves nothing at all. A run in which this reaches
/// its full `2N` is a run whose atomic result is not evidence, and the tests
/// below say so out loud rather than passing quietly.
const PLAIN: u64 = RAM + 0x200;

/// The base of the two private counters, one word apart, indexed by `mhartid`.
const PRIVATE: u64 = RAM + 0x100;

/// How many times each hart goes round. Large enough that the two overlap for
/// a long time and small enough that a debug build finishes quickly.
const N: i32 = 20_000;

// ---------------------------------------------------------------------------
// the guest program
// ---------------------------------------------------------------------------
//
// Encodings from *The RISC-V Instruction Set Manual, Volume I: Unprivileged
// ISA* (RV32I/RV64I base and the "A" standard extension) and volume II for
// `mhartid`. The helpers mirror `tests/riscv_amo_atomicity.rs`'s, deliberately:
// the two files assert the same architectural promise at two different levels,
// and the programs should be legibly the same program.

const A0: u32 = 10;
const A1: u32 = 11;
const A2: u32 = 12;
const A3: u32 = 13;
const A4: u32 = 14;
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

fn li(rd: u32, value: i32) -> Vec<u32> {
    let hi = ((value as u32).wrapping_add(0x800) >> 12) & 0xf_ffff;
    let lo = value.wrapping_sub((hi << 12) as i32);
    if hi == 0 {
        return vec![addi(rd, 0, lo)];
    }
    vec![lui(rd, hi), addi(rd, rd, lo)]
}

const fn lw(rd: u32, rs1: u32, off: i32) -> u32 {
    (((off as u32) & 0xfff) << 20) | (rs1 << 15) | (0b010 << 12) | (rd << 7) | 0x03
}

const fn sw(rs2: u32, rs1: u32, off: i32) -> u32 {
    let imm = (off as u32) & 0xfff;
    ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (0b010 << 12) | ((imm & 0x1f) << 7) | 0x23
}

const fn bne(rs1: u32, rs2: u32, off: i32) -> u32 {
    let imm = (off as u32) & 0x1fff;
    (((imm >> 12) & 1) << 31)
        | (((imm >> 5) & 0x3f) << 25)
        | (rs2 << 20)
        | (rs1 << 15)
        | (0b001 << 12)
        | (((imm >> 1) & 0xf) << 8)
        | (((imm >> 11) & 1) << 7)
        | 0x63
}

/// `add rd, rs1, rs2`.
const fn add(rd: u32, rs1: u32, rs2: u32) -> u32 {
    (rs2 << 20) | (rs1 << 15) | (rd << 7) | 0x33
}

/// `slli rd, rs1, shamt` — RV64, so the shift amount is six bits.
const fn slli(rd: u32, rs1: u32, shamt: u32) -> u32 {
    ((shamt & 0x3f) << 20) | (rs1 << 15) | (0b001 << 12) | (rd << 7) | 0x13
}

/// `csrr rd, csr`, which is `csrrs rd, csr, x0`.
const fn csrr(rd: u32, csr: u32) -> u32 {
    (csr << 20) | (0b010 << 12) | (rd << 7) | 0x73
}

/// `amoadd.w x0, rs2, (rs1)` — increment a word and discard the old value,
/// with `aq` and `rl` both clear.
const fn amoadd_w(rs2: u32, rs1: u32) -> u32 {
    (rs2 << 20) | (rs1 << 15) | (0b010 << 12) | 0x2f
}

/// `jal x0, 0` — where a finished hart parks.
const PARK: u32 = 0x0000_006f;

/// The program both harts run out of one ROM.
///
/// ```text
///   csrr a7, mhartid
///   li   a0, SHARED
///   li   a5, PLAIN
///   li   a3, PRIVATE
///   slli a6, a7, 2
///   add  a3, a3, a6          ; this hart's own word
///   li   a1, 1
///   li   a2, N
/// top:
///   amoadd.w x0, a1, (a0)    ; the shared counter, atomically
///   lw   a4, 0(a5)
///   addi a4, a4, 1
///   sw   a4, 0(a5)           ; the shared counter, plainly — the witness
///   lw   a4, 0(a3)
///   addi a4, a4, 1
///   sw   a4, 0(a3)           ; this hart's own count; nobody races it
///   addi a2, a2, -1
///   bne  a2, x0, top
///   jal  x0, 0
/// ```
///
/// The order inside the loop is the whole reason the assertion is sound. The
/// atomic comes **first** and the private counter last, so a private counter
/// that reads `N` means that hart has already retired all `N` of its atomics —
/// and the test can therefore wait for a moment at which the shared word must
/// be exactly `2N` rather than sampling one mid-iteration.
fn program() -> Vec<u32> {
    let mut c = vec![csrr(A7, MHARTID)];
    c.extend(li(A0, SHARED as i32));
    c.extend(li(A5, PLAIN as i32));
    c.extend(li(A3, PRIVATE as i32));
    c.push(slli(A6, A7, 2));
    c.push(add(A3, A3, A6));
    c.extend(li(A1, 1));
    c.extend(li(A2, N));
    let top = c.len();
    c.push(amoadd_w(A1, A0));
    c.push(lw(A4, A5, 0));
    c.push(addi(A4, A4, 1));
    c.push(sw(A4, A5, 0));
    c.push(lw(A4, A3, 0));
    c.push(addi(A4, A4, 1));
    c.push(sw(A4, A3, 0));
    c.push(addi(A2, A2, -1));
    let here = c.len();
    c.push(bne(A2, 0, (top as i32 - here as i32) * 4));
    c.push(PARK);
    c
}

fn rom() -> Vec<u8> {
    let mut out = Vec::new();
    for word in program() {
        out.extend_from_slice(&word.to_le_bytes());
    }
    out
}

// ---------------------------------------------------------------------------
// the board
// ---------------------------------------------------------------------------

/// Build the fixture, saying **nothing** about threading.
///
/// That is the point of the test: `workers` is a property of the run and is
/// set here, the mode is a property of the board and is not.
fn board(workers: usize) -> Machine {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.scheduler.workers = workers;
    options.realize.media.insert("code", rom());
    let registry = catalog::registry().expect("a registry");
    match rsemu::machine::build("smp-parallel.machine", SMP_PARALLEL, &registry, &options) {
        Ok(m) => m,
        Err(e) => panic!("the fixture does not realize: {e}"),
    }
}

fn word(m: &Machine, at: u64) -> u64 {
    m.space("mem")
        .expect("the fixture's one space")
        .read(at, Width::U32, MemAttrs::DEFAULT)
        .expect("a mapped word")
}

/// Run until both harts have parked, or give up.
///
/// Returns whether they both finished. A bound rather than a timeout, because
/// a test that hangs is worse than one that fails.
fn run_until_both_parked(m: &mut Machine) -> bool {
    for _ in 0..200_000 {
        if word(m, PRIVATE) == N as u64 && word(m, PRIVATE + 4) == N as u64 {
            // Two more rounds so the join at each round's end publishes
            // everything both harts wrote. Both are parked in `jal x0, 0` by
            // now, so nothing this observes can still move.
            m.run_quantum().expect("a round");
            m.run_quantum().expect("a round");
            return true;
        }
        m.run_quantum().expect("the machine advances");
    }
    false
}

// ---------------------------------------------------------------------------
// the machine file chose the mode
// ---------------------------------------------------------------------------

#[test]
fn the_machine_file_selects_parallel_and_nobody_else_had_to() {
    let m = board(2);
    assert_eq!(
        m.threading_mode(),
        ThreadingMode::Parallel,
        "`threading parallel` in the file is what put it there"
    );
    // And the consequence is the same one `--threading parallel` has, which is
    // what makes the statement honest rather than decorative.
    let err = m
        .state_hash()
        .expect_err("a parallel state hash is a sample, not a baseline");
    assert!(err.to_string().contains("parallel"), "{err}");
    assert!(m.nondeterministic_state_hash().is_ok());
}

#[test]
fn a_run_still_overrides_the_board() {
    // The regression suite's escape hatch: a board that would rather be fast
    // can still be made reproducible, and this is the call that does it.
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.threading = Some(ThreadingMode::Deterministic);
    options.realize.media.insert("code", rom());
    let registry = catalog::registry().expect("a registry");
    let m = rsemu::machine::build("smp-parallel.machine", SMP_PARALLEL, &registry, &options)
        .expect("the fixture realizes");
    assert_eq!(m.threading_mode(), ThreadingMode::Deterministic);
    m.state_hash().expect("and the hash comes back");
}

#[test]
fn the_same_board_is_still_bit_reproducible_when_a_run_asks_for_it() {
    let mut hashes = Vec::new();
    for _ in 0..3 {
        let mut options = catalog::build_options().expect("the catalog agrees with itself");
        options.realize.threading = Some(ThreadingMode::Deterministic);
        options.realize.media.insert("code", rom());
        let registry = catalog::registry().expect("a registry");
        let mut m =
            rsemu::machine::build("smp-parallel.machine", SMP_PARALLEL, &registry, &options)
                .expect("the fixture realizes");
        m.reset(ResetKind::Cold);
        m.run_for(GlobalTime::from_nanos(2_000_000)).expect("a run");
        hashes.push(m.state_hash().expect("deterministic mode hashes"));
    }
    assert_eq!(hashes[0], hashes[1]);
    assert_eq!(hashes[1], hashes[2]);
}

// ---------------------------------------------------------------------------
// both processors, on one crystal, in the same rounds
// ---------------------------------------------------------------------------

#[test]
fn both_harts_on_one_crystal_execute() {
    let mut m = board(2);
    m.reset(ResetKind::Cold);
    assert!(
        run_until_both_parked(&mut m),
        "one of the two harts never finished: hart 0 got to {}, hart 1 to {}, of {N}",
        word(&m, PRIVATE),
        word(&m, PRIVATE + 4)
    );
}

/// The claim `tests/riscv_amo_atomicity.rs` makes about two hand-spawned host
/// threads, made here about a whole machine.
///
/// A lost update is a value no interleaving of the two programs could have
/// produced, so this is an equality and not a printed count. `2N` exactly:
/// each hart retired `N` atomic increments, and every one of them landed.
#[test]
fn an_amo_loses_no_updates_across_two_harts_of_a_real_board() {
    let mut m = board(2);
    m.reset(ResetKind::Cold);
    assert!(run_until_both_parked(&mut m), "the harts did not finish");
    let plain = word(&m, PLAIN);
    // The witness first, and printed rather than asserted: how often the two
    // harts collided is a property of the host's scheduler, not of this tree.
    eprintln!(
        "two harts, {N} increments each: the plain counter lost {} of {}, \
         the atomic one lost {}",
        2 * N as u64 - plain,
        2 * N,
        2 * N as u64 - word(&m, SHARED)
    );
    assert_eq!(
        word(&m, SHARED),
        2 * N as u64,
        "an AMOADD went missing: {} of {} increments landed",
        word(&m, SHARED),
        2 * N
    );
    if plain == 2 * N as u64 {
        eprintln!(
            "warning: the plain counter reached its full {}, so the two harts never \
             collided in this run and the equality above is not evidence about atomicity. \
             That is a property of this host, not a failure — but a run that never \
             collides can never catch a lost update either.",
            2 * N
        );
    }
}

/// The same board in the mode the regression suite uses, so the parallel
/// result above means something rather than being a run of its own machine.
#[test]
fn the_same_program_reaches_the_same_answer_deterministically() {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.threading = Some(ThreadingMode::Deterministic);
    options.realize.media.insert("code", rom());
    let registry = catalog::registry().expect("a registry");
    let mut m = rsemu::machine::build("smp-parallel.machine", SMP_PARALLEL, &registry, &options)
        .expect("the fixture realizes");
    m.reset(ResetKind::Cold);
    assert!(run_until_both_parked(&mut m), "the harts did not finish");
    assert_eq!(word(&m, SHARED), 2 * N as u64);
}

/// A parallel machine whose run gave it no workers still runs, and reaches the
/// same answer.
///
/// The no-threads browser build (§11.3) and every `no_std` host land here: the
/// pool runs jobs inline, so the mode degenerates to submission order. It is a
/// supported configuration, not a fallback — and it is also what a *library*
/// caller gets by default, because `realize` will not spawn threads a caller
/// did not ask for. `rsemu run` sizes the pool itself; a caller embedding the
/// machine has to set `SchedulerConfig::workers` or it has selected a mode
/// that is parallel in name only.
#[test]
fn a_file_selected_parallel_machine_with_no_workers_still_runs() {
    let mut m = board(0);
    assert_eq!(m.threading_mode(), ThreadingMode::Parallel);
    m.reset(ResetKind::Cold);
    assert!(run_until_both_parked(&mut m), "the harts did not finish");
    assert_eq!(word(&m, SHARED), 2 * N as u64);
}

// ---------------------------------------------------------------------------
// the degenerate configuration, pinned
// ---------------------------------------------------------------------------

/// Two runnables on one oscillator starve the second one whenever the tick cap
/// does not bite.
///
/// This is not a bug in the parallel round; it is the clock model, and
/// `Scheduler::ticks_until_after` says so in both modes. A tree has one unit
/// counter, so the span between `now` and the round's target is shared out:
/// the first runnable is offered all of it and the second is offered what is
/// left. What normally leaves something is
/// [`SchedulerConfig::max_ticks_per_quantum`], which caps the first at 10 000
/// of the round's 100 000 ticks on this board.
///
/// Raise the cap past the whole span and hart 1 is handed a budget of zero,
/// every round, for ever. The test asserts exactly that, so the limitation is
/// a fact in the suite rather than a paragraph — and so that a fix, when one
/// comes, has to come here and delete it.
///
/// The fix is not a bigger cap. It is two oscillators, which is what the
/// hardware has (§4.2, "as many roots as the real board has crystals") and
/// what `machines/tests/heterogeneous.machine` uses.
///
/// [`SchedulerConfig::max_ticks_per_quantum`]: rsemu::core::sched::SchedulerConfig::max_ticks_per_quantum
#[test]
fn one_oscillator_and_no_tick_cap_starves_the_second_hart() {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.scheduler.workers = 2;
    // Far beyond the ~100 000 ticks a 1 ms round covers at 100 MHz, so the
    // first runnable is offered the whole span and reserves it.
    options.realize.scheduler.max_ticks_per_quantum = u64::MAX;
    options.realize.media.insert("code", rom());
    let registry = catalog::registry().expect("a registry");
    let mut m = rsemu::machine::build("smp-parallel.machine", SMP_PARALLEL, &registry, &options)
        .expect("the fixture realizes");
    m.reset(ResetKind::Cold);
    for _ in 0..200 {
        m.run_quantum().expect("the machine advances");
    }
    assert!(
        word(&m, PRIVATE) > 0,
        "hart 0 did not run either, so this test is measuring something else"
    );
    assert_eq!(
        word(&m, PRIVATE + 4),
        0,
        "hart 1 executed, so the tick reservation no longer starves it — good news, \
         and this test and the paragraph in docs/techniques/parallel-execution.md \
         both need deleting"
    );
}
