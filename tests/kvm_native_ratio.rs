//! **How close an accelerated guest gets to native**, measured rather than
//! claimed.
//!
//! `ROADMAP.md` phase 7's gate ends *"an accelerated guest reaches **≥ 80 % of
//! native** on the same CPU-bound workload, on the reference host"*. That
//! number had never been attempted. This file attempts it, and is as careful
//! about what it does **not** measure as about what it does.
//!
//! # The workload, and why this one
//!
//! A 32-bit xorshift — `x ^= x << 13; x ^= x >> 17; x ^= x << 5` — iterated a
//! few hundred million times. Three properties earn it the job:
//!
//! * **It is a dependency chain.** Every operation needs the previous one's
//!   result, so the loop runs at the *latency* of six one-cycle ALU
//!   operations and not at the width of the machine. A latency-bound loop is
//!   the fairest possible comparison between two compilations of one
//!   algorithm, because there is nothing for a scheduler, a vectoriser or an
//!   unroller to win: both sides are pinned to the same six cycles by the
//!   silicon.
//! * **It cannot be closed-formed.** A multiply-accumulate recurrence can be;
//!   LLVM would then compute the native side in constant time and the ratio
//!   would be a fiction. A xorshift has no closed form and is left as a loop.
//! * **It has an answer.** Both sides start from the same seed and run the
//!   same count, so [`the_accelerated_guest_and_the_native_loop_agree`]
//!   asserts they produce the *same 32-bit word*. Two timings of two
//!   different computations would not be a ratio of anything; this is the
//!   check that makes them one workload.
//!
//! The guest runs it in 16-bit real mode with `0x66` operand-size prefixes, so
//! the instructions executing on the host are `shl`/`shr`/`xor` on 32-bit
//! registers — the same instructions rustc emits for the Rust loop. There is
//! **one guest entry and no exit until the `hlt`**: nothing about this
//! measurement is amortised over a device model.
//!
//! # What the number covers, and what it does not
//!
//! It covers *the thing the gate names*: guest instructions executing on host
//! silicon, wall clock to wall clock, against the same algorithm compiled for
//! the host. That is the ceiling — the best an accelerated guest can do.
//!
//! On the machine this was written on — an i9-14900K, 400 million iterations,
//! three consecutive runs — it measured **99.7 %, 99.9 % and 101.2 % of
//! native**, both sides agreeing on `0x69d59332`. (A cold first run reads
//! nearer 94 %; that is why each side is run twice and the faster taken.) The
//! gate is 80 %; a pure-execution workload is not where an accelerator loses,
//! and the number says so.
//!
//! It does **not** cover, and no honest reading of it should be stretched to:
//!
//! * **Anything with exits in it.** A guest that touches a device leaves
//!   hardware, and `accel::cpu` then reads and writes the whole architectural
//!   state around the exit ([`state::store_from_vcpu`]). The second test here,
//!   [`the_cost_of_leaving_hardware_is_measured_not_assumed`], measures that
//!   round trip separately: **one to three microseconds** on the same host,
//!   run to run — three to four orders of magnitude more than an instruction.
//!   That figure carries this harness's own `KVM_GET_REGS`, `KVM_SET_REGS`
//!   and answer read as well as the exit itself, so it is an upper bound on
//!   the kernel's part of it. A workload's real speed is this ratio *minus*
//!   its exit rate
//!   times that cost, and the exit rate belongs to the workload rather than to
//!   this module — so 99 % here and a device-bound guest being slow are the
//!   same measurement, not a contradiction.
//! * **A board.** Under [`ThreadingMode::Parallel`] a scheduler round costs
//!   what it costs, virtual time is still the emulated grid, and an
//!   accelerated processor consumes its whole budget however long the host
//!   took. A board's *throughput* is a different measurement with a different
//!   denominator and it is not made here.
//! * **Codegen equality.** The two sides are the same *algorithm*, not the
//!   same bytes: this crate denies `unsafe_code`, so a test cannot execute a
//!   byte buffer natively to make them literally identical. The workload is
//!   chosen so that this cannot matter much — a latency-bound chain runs at
//!   its chain length on either compilation — and the test prints
//!   **cycles per iteration for both sides** so a reader can see whether that
//!   held on their host rather than taking it on faith.
//! * **Frequency.** Turbo, thermal and SMT neighbours move both sides. Each
//!   side is run twice and the faster run is taken, which removes a cold
//!   start but not a hostile machine.
//!
//! Skips cleanly with no `/dev/kvm`, and — because a timing assertion on a
//! shared or virtualised CI box is a flake generator — the **ratio is
//! asserted only when `RSEMU_ACCEL_BENCH` is set**. Without it the test still
//! runs the workload, still checks that the two sides agree, and still prints
//! the number.
//!
//! ```text
//! RSEMU_ACCEL_BENCH=1 cargo test --release --all-features \
//!   --test kvm_native_ratio -- --nocapture
//! ```
//!
//! [`state::store_from_vcpu`]: rsemu::accel::state::store_from_vcpu
//! [`ThreadingMode::Parallel`]: rsemu::core::sched::ThreadingMode::Parallel

#![cfg(all(feature = "accel-kvm", target_os = "linux", target_arch = "x86_64"))]

use std::hint::black_box;
use std::sync::Arc;
use std::time::{Duration, Instant};

use rsemu::accel::kvm::{Kvm, Vcpu, Vm};
use rsemu::core::exec::ExitReason;
use rsemu::core::space::{AddressSpace, RamStore, Region};

/// The seed both sides start from. Non-zero, because zero is xorshift's fixed
/// point and would make every count produce the same answer.
const SEED: u32 = 0x1357_9bdf;

/// How many iterations, unless `RSEMU_ACCEL_BENCH_ITERS` says otherwise.
///
/// Six cycles an iteration on a modern part is a few hundred million
/// iterations to the second, which is long enough that a millisecond of
/// measurement noise is a tenth of a percent and short enough that
/// `cargo test` does not stall.
const ITERS: u64 = 400_000_000;

/// Guest-physical address of the two-page RAM slot's code.
const CODE: u64 = 0x1000;
/// Where the guest leaves its answer, and where the harness reads it.
const ANSWER: u64 = 0x0100;

// ---------------------------------------------------------------------------
// the workload, twice
// ---------------------------------------------------------------------------

/// The workload, in Rust, for the host to compile for itself.
///
/// `black_box` on both ends so that a constant seed and a constant count
/// cannot be folded away; nothing inside the loop, because a barrier there
/// would lengthen the chain and make the native side artificially slow —
/// which would flatter the ratio, which is the direction an honest benchmark
/// must not err in.
fn native(seed: u32, iterations: u64) -> u32 {
    let mut x = black_box(seed);
    for _ in 0..black_box(iterations) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
    }
    black_box(x)
}

/// The same loop as x86 machine code, for a 16-bit real-mode guest.
///
/// Every arithmetic instruction carries the `0x66` operand-size prefix, so
/// what the host executes is the 32-bit form — the same instruction the Rust
/// loop compiles to. `iterations` must fit in 32 bits, which
/// [`iterations`](fn.iterations.html) enforces.
fn guest_code(seed: u32, iterations: u32) -> Vec<u8> {
    let mut code = Vec::new();
    // mov eax, seed
    code.extend_from_slice(&[0x66, 0xb8]);
    code.extend_from_slice(&seed.to_le_bytes());
    // mov ecx, iterations
    code.extend_from_slice(&[0x66, 0xb9]);
    code.extend_from_slice(&iterations.to_le_bytes());

    let loop_start = code.len();
    for (op, imm) in [(0xe2u8, 13u8), (0xeau8, 17u8), (0xe2u8, 5u8)] {
        // mov edx, eax ; shl/shr edx, imm ; xor eax, edx
        code.extend_from_slice(&[0x66, 0x89, 0xc2]);
        code.extend_from_slice(&[0x66, 0xc1, op, imm]);
        code.extend_from_slice(&[0x66, 0x31, 0xd0]);
    }
    // dec ecx
    code.extend_from_slice(&[0x66, 0x49]);
    // jnz loop_start
    let back = code.len() + 2 - loop_start;
    let rel = i8::try_from(-(i64::from(u32::try_from(back).expect("a short loop"))))
        .expect("the loop body fits a rel8");
    code.extend_from_slice(&[0x75, rel as u8]);

    // mov [ANSWER], eax  — the moffs form, with a 16-bit displacement because
    // this is 16-bit addressing.
    code.extend_from_slice(&[0x66, 0xa3]);
    code.extend_from_slice(&(ANSWER as u16).to_le_bytes());
    // hlt: leaves hardware, which is how the harness knows it is done.
    code.push(0xf4);
    code
}

/// How many iterations to run, from the environment or [`ITERS`].
fn iterations() -> u64 {
    std::env::var("RSEMU_ACCEL_BENCH_ITERS")
        .ok()
        .and_then(|v| v.parse().ok())
        .filter(|n| *n > 0 && *n <= u64::from(u32::MAX))
        .unwrap_or(ITERS)
}

// ---------------------------------------------------------------------------
// the guest
// ---------------------------------------------------------------------------

/// A bare guest: two pages of RAM at physical zero and nothing else, so that
/// the only exit the workload can take is its own `hlt`.
struct Bare {
    _kvm: Kvm,
    vm: Vm,
    ram: Arc<RamStore>,
    vcpu: Vcpu,
}

impl Bare {
    /// Build one with `code` loaded at [`CODE`], in 16-bit real mode with
    /// every segment based at zero.
    fn new(kvm: Kvm, code: &[u8]) -> Bare {
        let vm = kvm.create_vm().expect("KVM_CREATE_VM");
        let ram = Arc::new(RamStore::new(2 * 0x1000));
        vm.set_memory_region(0, 0, &ram).expect("memory slot 0");
        ram.write_at(CODE, code).expect("load the workload");

        let mem = Arc::new(AddressSpace::new("mem", 20));
        mem.topology()
            .map(Arc::new(Region::ram("ram", Arc::clone(&ram))), 0)
            .expect("map RAM");

        let vcpu = vm.create_vcpu(0, mem, None).expect("KVM_CREATE_VCPU");
        let mut sregs = vcpu.sregs().expect("KVM_GET_SREGS");
        for seg in [
            &mut sregs.cs,
            &mut sregs.ds,
            &mut sregs.es,
            &mut sregs.fs,
            &mut sregs.gs,
            &mut sregs.ss,
        ] {
            seg.base = 0;
            seg.selector = 0;
        }
        vcpu.set_sregs(&sregs).expect("KVM_SET_SREGS");
        Bare {
            _kvm: kvm,
            vm,
            ram,
            vcpu,
        }
    }

    /// Point the guest back at the top of the workload.
    fn rewind(&self) {
        let mut regs = self.vcpu.regs().expect("KVM_GET_REGS");
        regs.rip = CODE;
        // Bit 1 is hard-wired to one; VMX refuses an entry without it.
        regs.rflags = 0x2;
        self.vcpu.set_regs(&regs).expect("KVM_SET_REGS");
    }

    /// Run to the `hlt` and report the wall time it took and the answer.
    fn run(&self) -> (Duration, u32) {
        self.rewind();
        let started = Instant::now();
        let run = self.vcpu.run_until_exit(64).expect("the guest runs");
        let elapsed = started.elapsed();
        let exit = run.exit.expect("the workload halts");
        assert_eq!(
            exit.reason,
            ExitReason::HALT,
            "the guest left hardware for something other than its own `hlt`: \
             {exit:?}"
        );
        let mut word = [0u8; 4];
        self.ram.read_at(ANSWER, &mut word).expect("the answer");
        (elapsed, u32::from_le_bytes(word))
    }
}

/// The faster of two runs of `f`, to drop a cold start.
fn best<T>(mut f: impl FnMut() -> (Duration, T)) -> (Duration, T) {
    let first = f();
    let second = f();
    if second.0 < first.0 { second } else { first }
}

/// Cycles per iteration, given a duration, a count and a frequency.
///
/// Reported as a *number a reader can sanity-check*: the chain is three
/// shifts and three xors, so a host that eliminates `mov` at rename should
/// show about six on both sides, and a side that shows very much more is the
/// side whose codegen did not do what this file claims.
fn cycles_per_iteration(elapsed: Duration, iterations: u64, ghz: f64) -> f64 {
    elapsed.as_secs_f64() * ghz * 1e9 / iterations as f64
}

/// The host's nominal frequency, only for the printed cycles-per-iteration
/// figure — nothing is asserted against it.
///
/// KVM reports the TSC frequency it gives guests, which on an invariant-TSC
/// part is the **base** frequency rather than the turbo one the core is
/// actually running at. Dividing by the smaller number gives the smaller
/// answer, so the printed figure is a *lower* bound on cycles per iteration:
/// a part turboing to 5.7 GHz off a 3.19 GHz base prints about 3.6 where the
/// chain really costs about 6.4, which is the six-cycle chain plus the loop.
/// It is a sanity check on codegen, not a measurement.
fn tsc_ghz(vm: &Vm) -> Option<f64> {
    vm.tsc_khz().ok().map(|khz| khz as f64 / 1e6)
}

// ---------------------------------------------------------------------------
// the measurements
// ---------------------------------------------------------------------------

/// **The ratio.** One guest entry, no exits, against the same algorithm
/// compiled for the host.
#[test]
fn the_accelerated_guest_and_the_native_loop_agree() {
    if !Kvm::is_available() {
        println!("kvm ratio: no usable /dev/kvm on this host, skipping");
        return;
    }
    let kvm = match Kvm::open() {
        Ok(kvm) => kvm,
        Err(e) if e.is_unavailable() => return,
        Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
    };

    let n = iterations();
    let bare = Bare::new(
        kvm,
        &guest_code(SEED, u32::try_from(n).expect("checked by `iterations`")),
    );

    let (guest_time, guest_answer) = best(|| bare.run());
    let (native_time, native_answer) = best(|| {
        let started = Instant::now();
        let answer = native(SEED, n);
        (started.elapsed(), answer)
    });

    // The check that makes these one workload rather than two.
    assert_eq!(
        guest_answer, native_answer,
        "the guest and the native loop computed different answers, so there is \
         no ratio to report: guest {guest_answer:#010x}, native {native_answer:#010x}"
    );

    let ratio = native_time.as_secs_f64() / guest_time.as_secs_f64();
    println!("kvm ratio: {n} iterations of xorshift32, answer {guest_answer:#010x}");
    if let Some(ghz) = tsc_ghz(&bare.vm) {
        println!(
            "kvm ratio: guest {:?} ({:.2} cycles/iteration at {ghz:.3} GHz base)",
            guest_time,
            cycles_per_iteration(guest_time, n, ghz)
        );
        println!(
            "kvm ratio: native {:?} ({:.2} cycles/iteration at {ghz:.3} GHz base)",
            native_time,
            cycles_per_iteration(native_time, n, ghz)
        );
    } else {
        println!("kvm ratio: guest {guest_time:?}, native {native_time:?}");
    }
    println!("kvm ratio: the guest is {:.1} % of native", ratio * 100.0);

    if std::env::var("RSEMU_ACCEL_BENCH").is_ok() {
        assert!(
            ratio >= 0.80,
            "phase 7's gate is 80 % of native and this host measured {:.1} %",
            ratio * 100.0
        );
    } else {
        println!(
            "kvm ratio: RSEMU_ACCEL_BENCH is unset, so the 80 % gate is reported \
             and not asserted"
        );
    }
}

/// **What one exit costs**, so the ratio above is not read as a whole answer.
///
/// The same guest, entered a great many times on a workload short enough that
/// the entry dominates: the difference between a long run and the sum of the
/// short ones is the cost of leaving and re-entering hardware. That number is
/// what a device-touching workload pays per access, and it is the reason this
/// file refuses to call the ratio above "rsemu's speed".
#[test]
fn the_cost_of_leaving_hardware_is_measured_not_assumed() {
    if !Kvm::is_available() {
        println!("kvm exit cost: no usable /dev/kvm on this host, skipping");
        return;
    }
    let kvm = match Kvm::open() {
        Ok(kvm) => kvm,
        Err(e) if e.is_unavailable() => return,
        Err(e) => panic!("/dev/kvm is present but unusable: {e}"),
    };

    // Short enough that the round trip is most of what is being timed, and
    // repeated enough that the timer's resolution is not.
    const SHORT: u32 = 1_000;
    const ROUNDS: u32 = 2_000;
    let bare = Bare::new(kvm, &guest_code(SEED, SHORT));

    let started = Instant::now();
    for _ in 0..ROUNDS {
        let _ = bare.run();
    }
    let round_trips = started.elapsed();

    let long = Bare::new(Kvm::open().expect("a second handle"), &{
        let total = u32::try_from(u64::from(SHORT) * u64::from(ROUNDS)).expect("fits");
        guest_code(SEED, total)
    });
    let (straight, _) = long.run();

    let overhead = round_trips.saturating_sub(straight);
    println!(
        "kvm exit cost: {ROUNDS} entries of {SHORT} iterations took {round_trips:?}; \
         one entry of {} took {straight:?}",
        u64::from(SHORT) * u64::from(ROUNDS)
    );
    println!(
        "kvm exit cost: {:?} per entry-and-exit round trip — an upper bound, because \
         it carries this harness's own `KVM_GET_REGS`, `KVM_SET_REGS` and answer read \
         as well as the exit",
        overhead / ROUNDS
    );
}
