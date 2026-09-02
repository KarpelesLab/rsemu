//! The conformance runner for the A64 suite this project **generates**.
//!
//! `ROADMAP.md` §0: *accuracy is measured, never asserted.* This is the
//! measurement, and getting one at all took a different route from every other
//! core in the tree, which is the part worth explaining.
//!
//! # There is no corpus to download
//!
//! Every other core here is gated on somebody else's vectors. AArch64 has
//! none that can be used:
//!
//! * **SingleStepTests** covers 65x02, Z80, 8086/8088/80286/80386, 68000,
//!   SPC700, V20 and R3000. There is no AArch64 repository.
//! * **Arm's Architecture Validation Suite** is licensed to implementers and
//!   is not public.
//! * **`rems-project/sail-arm`** is permissively licensed but is a *model*,
//!   not a corpus: extracting vectors from it needs a Sail/OCaml toolchain and
//!   would be a project of its own.
//! * **`kvm-unit-tests`** has an arm64 target and is GPL-2.0. Running it as an
//!   emulated guest is ordinary use and shipping it here is not
//!   (`ROADMAP.md` §1) — but it is a *machine* gate needing a GIC and a timer,
//!   not a core one.
//!
//! So this suite is built rather than fetched. `rustc` targets
//! `aarch64-unknown-none` with no C toolchain — `rust-lld` ships inside the
//! Rust toolchain and is the target's default linker — so a guest program can
//! be compiled from Rust source in this repository, run to a `BRK`, and have
//! its registers read.
//!
//! # Does generating a corpus really sidestep the licence problem?
//!
//! For redistribution, yes and completely: the sources under `tests/a64/` are
//! ours and MIT, nothing is downloaded, and no corpus is committed as a binary
//! because the binaries are built into an ignored directory like every other
//! suite here.
//!
//! For *evidence*, it depends entirely on where the expected values come from,
//! and that is the question the licence answer can hide. A corpus we write and
//! then check against our own reading of DDI 0487 proves only that two parts
//! of one head agree. So the suite is built around two sources of expectation
//! that are **not** this project:
//!
//! 1. **`rustc`'s constant evaluator** as a floating-point oracle. Each
//!    expectation in `fp_arith.rs`, `fp_random.rs`, `fp_convert.rs` and
//!    `fp_natural.rs` is computed on the host
//!    at compile time by `rustc_apfloat` — a port of LLVM's `APFloat`, sharing
//!    no code and no authorship with `src/float` — and the same expression is
//!    computed again at run time by the guest, where `black_box` forces real
//!    `FADD`/`FCVTZS` instructions. IEEE 754 §5.1 makes those operations
//!    correctly rounded and therefore unique, so a disagreement means one of
//!    two independent implementations is wrong. That is a genuine oracle.
//! 2. **LLVM's instruction selector** as an encoding generator. The guests are
//!    ordinary Rust; which instructions they contain is `rustc`'s choice, not
//!    ours. A `u128` multiply, a `%` by a variable, a `f64 as u32`, a
//!    `compare_exchange` — each lowers to encodings nobody here thought to
//!    write a unit test for, and running them end to end through fetch,
//!    decode, execute and the exception path is exactly what a unit test
//!    cannot do.
//!
//! Where neither applies — NaN payload propagation, the exception flags in
//! `FPSR`, the rounding modes `FPCR` selects, `FCMP`'s four-way `NZCV`, the
//! `CPACR_EL1` trap — the expectations *are* ours, transcribed from DDI 0487,
//! and `fp_rules.rs` says so on every case. Those are directed tests that
//! happen to run in a guest, not conformance evidence, and this file does not
//! pretend otherwise.
//!
//! # Advanced SIMD, and where its oracle stops
//!
//! `fp_natural.rs` is the second guest built around source (2), and it is the
//! one that could not exist before the vector instructions did. The other
//! floating-point guests route every operand through `black_box` as a bit
//! pattern and never write a floating-point literal, because LLVM materialises
//! `0.0` with `MOVI Dd, #0` — an Advanced SIMD encoding — and vectorises any
//! loop long enough to be worth it. `fp_natural.rs` removes both contortions:
//! literals are literals, the arrays are long, and the vectoriser is left
//! alone. For the encodings LLVM then chooses, the oracle is genuinely two
//! independent computations of the same function — a vectorised loop against
//! `rustc_apfloat` evaluating the scalar one — and it covers `MOVI`, `DUP`,
//! `UMOV`, `ADDV`, the lanewise arithmetic and compares, `USHLL`/`UADDW`,
//! `EXT`, `REV64` and both conversion directions.
//!
//! It stops there, and the boundary is worth naming rather than blurring:
//!
//! * **Only what the compiler emits.** `TBL`/`TBX`, the permutes, `LD2`–`LD4`,
//!   the reductions other than `ADDV`, the `MOVI` shift and `MSL` forms, `INS`
//!   and `SMOV` — nothing in ordinary Rust produces them, so nothing here is
//!   an independent check of them. Their tests are directed ones in
//!   `super::tests`, written from DDI 0487, and they prove that two parts of
//!   one head agree and nothing more.
//! * **Only where the vectoriser preserves the operation order.** It will not
//!   reassociate floating point, which is exactly why a vectorised loop and a
//!   scalar one must agree bit for bit — but it also means no floating-point
//!   vector *reduction* is ever produced, so `FADDP`, `FMAXV` and their
//!   relatives have no oracle here either.
//! * **Decode, separately.** The table was diffed against `llvm-mc` over
//!   sampled words in the Advanced SIMD and structure-load encoding spaces,
//!   comparing not the disassembly but whether the *interpreter* accepts the
//!   word at all. Over 40 000 words of data processing and 30 000 of loads and
//!   stores, nothing this core accepts is rejected by `llvm-mc`. That is a
//!   check on the masks, and it says nothing about the semantics.
//!
//! # Running it
//!
//! ```text
//! scripts/fetch-testdata.sh a64-tests
//! RSEMU_A64_TESTS=testdata/a64-tests cargo test --all-features a64_conformance -- --nocapture
//! ```
//!
//! `RSEMU_A64_TESTS_ONLY` narrows a run to the binaries whose names contain
//! one of a comma-separated list. Without `RSEMU_A64_TESTS` the test prints
//! how to build the corpus and passes, so `cargo test` stays hermetic and
//! offline.
//!
//! # The protocol
//!
//! Each guest ends at `BRK #0` with `x0` zero for success or a 1-based case
//! number, `x1` what it produced, `x2` what it should have produced, and `x3`
//! a tag naming the subtest. The core is run with [`ExitMask::BREAKPOINT`],
//! [`ExitMask::FAULT`] and [`ExitMask::SYSCALL`] armed, so a guest that takes
//! an unexpected `UNDEFINED` or data abort **leaves the core** with the
//! faulting PC and `ESR` in hand rather than vectoring into a `VBAR_EL1` it
//! never set up. That is worth more than a vector table: the diagnosis is "an
//! instruction at 0x40000abc raised EC 0x00" rather than a hang.
//!
//! # Sources
//!
//! *Arm Architecture Reference Manual for A-profile architecture* (DDI 0487)
//! and IEEE 754-2019, cited case by case in the guest sources. No emulator
//! source of any licence was consulted (`ROADMAP.md` §1).

use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use std::path::Path;

use crate::core::exec::{Exit, ExitMask, ExitReason, ExitingCore};
use crate::core::sched::Budget;
use crate::core::space::{AddressSpace, RamStore, Region, UnassignedPolicy};

use super::elf::Elf;
use super::{Config, Cpu};

/// Where `tests/a64/link.ld` puts a guest, and where `a64-mini` puts DRAM.
const RAM_BASE: u64 = 0x4000_0000;

/// How much RAM to give it. The guests are a few tens of kilobytes.
const RAM_SIZE: u64 = 16 << 20;

/// How many bus accesses a guest may make before it is called a hang.
///
/// The longest of these charges a few hundred thousand; this is two orders of
/// magnitude of headroom, so a timeout means a genuine loop rather than a slow
/// test.
const ACCESS_LIMIT: u64 = 40_000_000;

// ---------------------------------------------------------------------------
// The known-failures ledger
// ---------------------------------------------------------------------------

/// Failures this core is known to have, each with a mandatory reason.
///
/// `CLAUDE.md` (CPU cores): *a core lands with its conformance suite and a
/// known-failures ledger that only ever shrinks.* Both halves are enforced
/// below — an unexcused failure fails the suite, and an entry whose test now
/// passes fails it too, with an instruction to delete the line. Without the
/// second half a ledger silently becomes a list of things that used to be
/// broken.
///
/// A line is `(binary, why)`. It excuses **every** case in that binary, which
/// is deliberately coarse: a per-case excuse would let a regression hide
/// behind an unrelated known failure in the same file.
///
/// **This list is empty.** At the commit that added this file, all six
/// binaries passed — 12 000 differentially-checked floating-point vectors
/// among them — and it stayed empty when the seventh, `fp_natural`, arrived
/// with the Advanced SIMD slice.
const LEDGER: &[(&str, &str)] = &[];

// ---------------------------------------------------------------------------
// Running one binary
// ---------------------------------------------------------------------------

/// What running one guest produced.
#[derive(Debug, PartialEq, Eq)]
enum Outcome {
    /// The guest reported success.
    Pass,
    /// The guest reported a failing case.
    Failed {
        /// The 1-based case number in `x0`.
        case: u64,
        /// What it produced, from `x1`.
        got: u64,
        /// What it should have produced, from `x2`.
        want: u64,
        /// Which subtest, from `x3`.
        tag: u64,
    },
    /// The guest raised an exception the suite did not ask for.
    Trapped {
        /// Where.
        pc: u64,
        /// `ESR_EL1`'s exception class and syndrome, as the exit carried them.
        esr: u64,
        /// The instruction there, disassembled — which is nearly always the
        /// whole diagnosis, because the usual cause is an encoding this core
        /// does not implement.
        text: String,
    },
    /// The access limit was reached.
    Timeout {
        /// Where it was when the budget ran out.
        pc: u64,
    },
    /// The file is not one of ours.
    Skipped(String),
}

/// Load and run one built guest, reporting what it did and how much of the
/// machine it moved.
///
/// The second number is not decoration. `docs/testing/README.md` records the
/// mistake this directory has already made once: a ratio on its own cannot
/// tell a clean run from a run that measured nothing, and a guest whose body
/// was optimised away reports success just as loudly as one that ran.
fn run_one(path: &Path, cfg: Config) -> (Outcome, u64) {
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(e) => return (Outcome::Skipped(format!("unreadable: {e}")), 0),
    };
    let elf = match Elf::parse(&bytes) {
        Ok(e) => e,
        Err(e) => return (Outcome::Skipped(e.to_string()), 0),
    };

    let ram = Arc::new(RamStore::new(RAM_SIZE));
    for segment in &elf.segments {
        if segment.addr < RAM_BASE || segment.addr + segment.mem_len > RAM_BASE + RAM_SIZE {
            return (
                Outcome::Skipped(format!(
                    "segment at {:#x} does not fit in RAM",
                    segment.addr
                )),
                0,
            );
        }
        let at = segment.addr - RAM_BASE;
        ram.write_at(at, &segment.bytes).expect("in range");
        // `.bss` must read as zero. A fresh `RamStore` already does, but a
        // guest that relies on it should rely on the *loader*.
        let filled = segment.bytes.len() as u64;
        if segment.mem_len > filled {
            ram.fill(at + filled, segment.mem_len - filled, 0)
                .expect("in range");
        }
    }

    // Nothing but RAM is mapped, and a hole faults rather than reading zero:
    // a guest that runs off the end should fail loudly.
    let space = AddressSpace::new("mem", 64).with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), RAM_BASE)
        .expect("RAM fits");

    let cpu = Cpu::new(cfg.with_reset_vector(elf.entry));
    cpu.attach_space(Arc::new(space));
    // Everything the guest can raise leaves the core, so a fault is a
    // diagnosis rather than a jump into an unmapped vector table.
    cpu.set_exit_mask(
        ExitMask::NONE
            .with(ExitReason::BREAKPOINT)
            .with(ExitReason::FAULT)
            .with(ExitReason::SYSCALL),
    );

    let mut used = 0u64;
    while used < ACCESS_LIMIT {
        let run = cpu.run_to_exit(Budget::of(100_000));
        used += run.consumed.ticks;
        let Some(exit) = run.exit else {
            continue;
        };
        return (classify(&cpu, &exit), used);
    }
    (Outcome::Timeout { pc: cpu.pc() }, used)
}

/// Turn the exit a guest produced into an outcome.
fn classify(cpu: &Cpu, exit: &Exit) -> Outcome {
    if exit.reason != ExitReason::BREAKPOINT {
        let text = cpu
            .disassemble_physical(exit.pc, 1)
            .first()
            .map_or_else(|| "??".to_string(), |d| d.text.clone());
        return Outcome::Trapped {
            pc: exit.pc,
            esr: exit.detail,
            text,
        };
    }
    let case = cpu.x(0);
    if case == 0 {
        Outcome::Pass
    } else {
        Outcome::Failed {
            case,
            got: cpu.x(1),
            want: cpu.x(2),
            tag: cpu.x(3),
        }
    }
}

impl Outcome {
    /// One line describing a failure, or `None` for a pass.
    fn failure(&self) -> Option<String> {
        match self {
            Outcome::Pass | Outcome::Skipped(_) => None,
            Outcome::Failed {
                case,
                got,
                want,
                tag,
            } => Some(format!(
                "case {case} (subtest {tag}) got {got:#018x}, want {want:#018x}"
            )),
            Outcome::Trapped { pc, esr, text } => Some(format!(
                "unexpected exception at {pc:#x}: EC {:#04x} ISS {:#x} — `{text}`",
                esr >> 26,
                esr & 0x01ff_ffff
            )),
            Outcome::Timeout { pc } => {
                Some(format!("no result within the access budget, pc {pc:#x}"))
            }
        }
    }
}

// ---------------------------------------------------------------------------
// The suite
// ---------------------------------------------------------------------------

/// Run the built corpus, or explain why it did not.
///
/// Not `#[ignore]`d: a skipped test that says nothing is how a suite quietly
/// stops running. This one prints the command that would have built it.
#[test]
fn a64_conformance() {
    let Ok(dir) = std::env::var("RSEMU_A64_TESTS") else {
        println!(
            "a64 conformance: build the corpus with `scripts/fetch-testdata.sh \
             a64-tests`, then set RSEMU_A64_TESTS to the directory it wrote \
             (default testdata/a64-tests). Nothing is downloaded — the guests \
             are `tests/a64/*.rs`, compiled for aarch64-unknown-none."
        );
        return;
    };
    let only = std::env::var("RSEMU_A64_TESTS_ONLY").ok();
    let dir = Path::new(&dir);

    let mut entries: Vec<_> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.is_file() && p.extension().is_some_and(|e| e == "elf"))
        .collect();
    // Sorted, so a run is reproducible and two runs diff cleanly.
    entries.sort();

    // Neoverse N1 rather than the default part: it has every feature this core
    // implements, so a guest built with `-C target-cpu=neoverse-n1` — or one
    // `rustc` chose an `LSE` atomic for — runs. The feature *gate* is proved
    // by the crate's own unit tests, not by refusing to run the suite.
    let cfg = Config::neoverse_n1();

    let mut passed = Vec::new();
    let mut failures = Vec::new();
    let mut skipped = 0usize;
    let mut charged = 0u64;

    for path in &entries {
        let name = path
            .file_stem()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        if let Some(only) = &only
            && !only.split(',').any(|f| name.contains(f.trim()))
        {
            continue;
        }
        let (outcome, accesses) = run_one(path, cfg);
        if let Outcome::Skipped(why) = &outcome {
            skipped += 1;
            println!("skipped {name}: {why}");
            continue;
        }
        charged += accesses;
        match outcome.failure() {
            None => {
                println!("pass {name} ({accesses} bus accesses)");
                passed.push(name);
            }
            Some(why) => {
                println!("FAIL {name}: {why}");
                failures.push((name, why));
            }
        }
    }

    println!(
        "a64 conformance: {} passed, {} failed, {skipped} skipped, \
         {charged} bus accesses charged",
        passed.len(),
        failures.len()
    );
    assert!(
        !passed.is_empty() || !failures.is_empty(),
        "no guest binaries under {} — did the build script run?",
        dir.display()
    );
    // A guest that reported success without executing anything is the failure
    // mode a pass count cannot see.
    assert!(
        charged > 100_000 || !failures.is_empty(),
        "the suite passed having charged only {charged} bus accesses, \
         which is too few to have run anything"
    );

    // The ledger, both ways round.
    let mut complaints: Vec<String> = Vec::new();
    for (name, why) in &failures {
        match LEDGER.iter().find(|(binary, _)| binary == name) {
            Some((_, excuse)) => println!("  known failure {name}: {excuse}"),
            None => complaints.push(format!("{name}: {why}")),
        }
    }
    for (binary, excuse) in LEDGER {
        if passed.iter().any(|name| name == binary) {
            complaints.push(format!(
                "{binary} passes but is still in the ledger (\"{excuse}\") — \
                 delete the line; the ledger only ever shrinks"
            ));
        }
    }
    assert!(complaints.is_empty(), "\n  {}", complaints.join("\n  "));
}
