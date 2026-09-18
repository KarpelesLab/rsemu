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
//! * **Both processors run at the crystal's whole rate.** One crystal and two
//!   runnables used to be outside the clock model: a tree has one counter, and
//!   the two harts were first given whatever a rate-blind cap left over, then
//!   half a round each. Each runnable now executes the round from a position of
//!   its own and the crystal's counter stands at the slower of them
//!   (`Scheduler::advance_runnable`), so each hart retires the 100 MHz this
//!   board declares — which is asserted, to the cycle, below.
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
//! That file's `machine` module now runs on this same fixture, and on
//! `machines/tests/smp-parallel-a64.machine` beside it — the store-buffer and
//! message-passing litmus tests as guest programs on a board in `parallel`,
//! which is the ordering half of the move this file made for atomicity. What
//! it found, and how much less sensitive a machine is than two threads in a
//! tight loop, is in `docs/techniques/parallel-execution.md`.
//!
//! [`Machine::state_hash`]: rsemu::machine::Machine::state_hash

// `std` because `parallel` is a threading mode and a `no_std` build has no
// threads — the same reason `parallel_threading.rs` gives.
#![cfg(all(feature = "cpu-riscv", feature = "std"))]

use std::sync::Arc;

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::core::device::ResetKind;
use rsemu::core::sched::ThreadingMode;
use rsemu::core::space::MemAttrs;
use rsemu::core::value::Width;
use rsemu::cpu::riscv::Hart;
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
// one crystal, two processors, each at the crystal's rate
// ---------------------------------------------------------------------------

/// Two runnables on one oscillator both run, with **no tick cap at all**.
///
/// The first of three answers this file has pinned. A tree has one unit
/// counter, and hart 1 was once handed whatever a rate-blind 10 000-tick cap
/// left of the round — nothing, once the cap went — and then half the round.
/// Each runnable is now offered its own clock's whole span
/// (`Scheduler::advance_runnable`); this test keeps the parallel half of that
/// claim, and the one below measures the rate.
#[test]
fn one_oscillator_and_no_tick_cap_runs_both_harts() {
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.scheduler.workers = 2;
    // No explicit ceiling: the only thing bounding a budget is the round.
    options.realize.scheduler.max_ticks_per_quantum = None;
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
        "hart 0 did not run, so this test is measuring something else"
    );
    assert!(
        word(&m, PRIVATE + 4) > 0,
        "hart 1 was starved with no cap in play, which is the defect \
         Scheduler::round_allowance exists to have fixed"
    );
}

/// `j .` minus four: `jal x0, -4`, the other half of a two-instruction loop.
const JUMP_BACK: u32 = 0xffdf_f06f;

/// `addi a0, a0, 1` then `j` back to it, in both harts: nothing but
/// instruction fetches, and no data access for the two to contend over.
fn counting_rom() -> Vec<u8> {
    let mut out = Vec::new();
    for word in [addi(A0, A0, 1), JUMP_BACK] {
        out.extend_from_slice(&word.to_le_bytes());
    }
    out
}

/// Two harts on one 100 MHz crystal, **each** retiring 100 MHz.
///
/// A crystal clocks every chip on it at once; two processors on one
/// oscillator do not take turns. The board used to run each of these harts at
/// half the rate the file declares, because a round divided one crystal's
/// span between the runnables on it. Here the claim is measured the way a
/// guest would feel it: each hart's own count of the cycles it spent, against
/// what 100 MHz owes the span, and the loop counter each one advanced against
/// the instructions it retired.
/// Deterministic, so the number is exact rather than a sample.
#[test]
fn two_harts_on_one_crystal_each_retire_the_crystals_whole_rate() {
    let harts: Arc<Captured<Hart>> = Arc::new(Captured::new());
    let kept = Arc::clone(&harts);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.realize.threading = Some(ThreadingMode::Deterministic);
    options.bindings.replace("cpu.riscv", move |props| {
        let hart = Arc::new(Hart::from_props(props)?);
        kept.push(&hart);
        Ok(hart)
    });
    options.realize.media.insert("code", counting_rom());
    let registry = catalog::registry().expect("a registry");
    let mut m = rsemu::machine::build("smp-parallel.machine", SMP_PARALLEL, &registry, &options)
        .expect("the fixture realizes");
    m.reset(ResetKind::Cold);
    let harts = harts.all();
    assert_eq!(harts.len(), 2, "both harts were captured");

    m.run_until(GlobalTime::from_nanos(1_000_000))
        .expect("a run");
    let before: Vec<(u64, u64, u64)> = harts
        .iter()
        .map(|h| (h.cycles(), h.instret(), h.x(A0)))
        .collect();
    let span_ns = 20_000_000;
    m.run_until(GlobalTime::from_nanos(1_000_000 + span_ns))
        .expect("a run");
    // 100 MHz for 20 ms.
    let owed = 2_000_000u64;
    for (n, (hart, (c0, i0, a0))) in harts.iter().zip(before).enumerate() {
        let cycles = hart.cycles() - c0;
        let retired = hart.instret() - i0;
        let loops = hart.x(A0) - a0;
        eprintln!(
            "hart {n}: {cycles} cycles of {owed} owed, {retired} instructions, {loops} iterations"
        );
        assert!(
            cycles.abs_diff(owed) <= 2,
            "hart {n} retired {cycles} cycles where its crystal owes {owed}"
        );
        assert!(
            (2 * loops).abs_diff(retired) <= 2,
            "hart {n}'s {loops} iterations of two instructions do not account for {retired} \
             retired"
        );
        assert!(retired > 0, "hart {n} executed nothing");
    }
}

/// A processor that leads its crystal comes back from a snapshot leading it by
/// exactly as much, so a restored machine runs on exactly as the saved one did.
///
/// Two harts on one crystal, one at the crystal's rate and one at a third of
/// it: a round's target is rarely a whole tick of the slower one, so the faster
/// ends each round up to two units ahead of the crystal's counter, which stands
/// at the slower (`Scheduler::advance_runnable`). That lead is machine state.
/// The tick counts a snapshot already stores say how many *ticks* ahead the
/// fast hart is and not where inside a tick the crystal stands, so the clock
/// chunk carries the lead itself — and only when there is one, so no machine
/// with one runnable per crystal writes a byte more than it used to.
#[test]
fn a_lead_on_a_shared_crystal_survives_a_snapshot() {
    let source = SMP_PARALLEL.replacen(
        "object cpu1 \"cpu.riscv\" {\n    clock  = core",
        "object cpu1 \"cpu.riscv\" {\n    clock  = core / 3",
        1,
    );
    assert_ne!(
        source, SMP_PARALLEL,
        "the fixture's second hart was re-rated"
    );
    let build = || {
        let mut options = catalog::build_options().expect("the catalog agrees with itself");
        options.realize.threading = Some(ThreadingMode::Deterministic);
        options.realize.media.insert("code", counting_rom());
        let registry = catalog::registry().expect("a registry");
        let mut m = rsemu::machine::build("smp-lead.machine", &source, &registry, &options)
            .expect("the fixture realizes");
        m.reset(ResetKind::Cold);
        m
    };
    let mut a = build();
    // Two milliseconds: 200 000 units, which is not a whole number of the
    // slow hart's three-unit ticks.
    a.run_until(GlobalTime::from_nanos(2_000_000))
        .expect("a run");
    let fast = a
        .device("cpu0")
        .and_then(|d| d.domain())
        .expect("a clocked hart");
    let lead = a.clocks().lead(fast).expect("a domain");
    assert!(
        lead > 0,
        "the fast hart should end a round ahead of its crystal"
    );
    let snapshot = a.save().expect("a snapshot");

    let mut b = build();
    b.load(&snapshot).expect("it loads");
    assert_eq!(b.clocks().lead(fast).unwrap(), lead, "the lead came back");
    assert_eq!(b.state_hash().unwrap(), a.state_hash().unwrap());

    let until = GlobalTime::from_nanos(7_000_000);
    a.run_until(until).expect("a run");
    b.run_until(until).expect("a run");
    assert_eq!(
        b.state_hash().unwrap(),
        a.state_hash().unwrap(),
        "the restored machine diverged from the one it was saved from"
    );
}
