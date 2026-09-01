//! The RISC-V lifter against the RISC-V interpreter, over a generated corpus.
//!
//! CLAUDE.md, "CPU cores": an IR frontend *is differentially tested against
//! the interpreter forever*, and the interpreter is the oracle. The comparison
//! itself lives in [`cpu::riscv::differential`], because
//! `fuzz_targets/riscv_lift.rs` drives the same function; this file is the
//! half of it that runs in a plain `cargo test`, offline, with no fuzzer and
//! no downloaded corpus.
//!
//! The programs are generated rather than written, because the bugs a
//! hand-written suite finds are the ones its author thought of. Generation is
//! a 64-bit LCG with a fixed seed, so the corpus is the same on every machine
//! and in every run (`ROADMAP.md` §0) — a failure here is reproducible from
//! the seed printed beside it, and a new failure is a real regression rather
//! than a different draw.
//!
//! Every case runs on both a hart that performs misaligned accesses and one
//! that traps them, because the [`Align`](rsemu::ir::Align) a memory op
//! carries is a frontend decision and the two answers are guest-visible.

#![cfg(feature = "cpu-riscv-lift")]

use rsemu::cpu::riscv::Config;
use rsemu::cpu::riscv::csr::Extensions;
use rsemu::cpu::riscv::differential::{Case, Verdict, compare, synthesize};

/// A 64-bit linear congruential generator — Knuth's MMIX multiplier and
/// increment.
///
/// A named, fixed generator rather than anything from the host: the corpus has
/// to be identical everywhere, and a hash of the run index would give a
/// different sequence the day the hasher changes.
struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0
    }
}

/// One generated program of up to `len` instructions.
fn program(rng: &mut Lcg, len: usize) -> Vec<u32> {
    (0..len)
        .map(|_| {
            let bits = rng.next();
            synthesize((bits >> 40) as u32, bits as u32)
        })
        .collect()
}

/// Run `count` generated cases from `seed` on `cfg`, and report what they
/// covered.
///
/// Returns `(agreed, trapped, nothing)`. A [`Verdict::Trapped`] is a real
/// result — both engines stopped at the same instruction — and a
/// [`Verdict::Nothing`] means the first instruction was outside the subset, so
/// counting them is how the test can assert it is actually exercising the
/// lifter rather than measuring an empty block a thousand times.
fn sweep(cfg: Config, seed: u64, count: usize) -> (usize, usize, usize) {
    let mut rng = Lcg(seed);
    let (mut agreed, mut trapped, mut nothing) = (0, 0, 0);
    for n in 0..count {
        let len = 1 + (rng.next() % 12) as usize;
        let case = Case::seeded(program(&mut rng, len)).with_config(cfg);
        match compare(&case) {
            Ok(Verdict::Agreed { .. }) => agreed += 1,
            Ok(Verdict::Trapped { .. }) => trapped += 1,
            Ok(Verdict::Nothing) => nothing += 1,
            Err(e) => panic!("case {n} of seed {seed:#x} diverged:\n{e}"),
        }
    }
    (agreed, trapped, nothing)
}

#[test]
fn a_generated_corpus_agrees_with_the_interpreter_on_a_bare_rv64i_hart() {
    let (agreed, trapped, nothing) = sweep(Config::rv64i(), 0x5eed_0001, 2_000);
    // The corpus has to be doing something: most cases must lift and run to
    // completion, and the ones that do not are counted rather than hidden.
    assert!(
        agreed > 1_000,
        "only {agreed} of 2000 cases ran to completion ({trapped} trapped, {nothing} lifted \
         nothing) — the generator has stopped producing programs in the subset"
    );
    assert!(
        trapped > 0,
        "no case reached a trap, so the fault-agreement column was never tested"
    );
}

#[test]
fn a_generated_corpus_agrees_on_a_hart_that_traps_misaligned_accesses() {
    // `Config::misaligned` is in the cache key and in every memory op's
    // `Align`, so it is a second frontend to test rather than a second run of
    // the first.
    let mut strict = Config::rv64i();
    strict.misaligned = false;
    let (agreed, trapped, _) = sweep(strict, 0x5eed_0002, 2_000);
    assert!(
        agreed > 500,
        "only {agreed} of 2000 cases ran to completion"
    );
    assert!(
        trapped > 0,
        "a hart that traps misaligned accesses must have trapped somewhere"
    );
}

#[test]
fn a_generated_corpus_agrees_on_a_core_with_compressed_instructions() {
    // With `C` the lifter takes two extra paths: `isa::expand`, and `JALR`,
    // whose target alignment is only discharged when a two-byte target is
    // legal. The generated words are still 32-bit encodings, so what changes
    // is the configuration rather than the corpus.
    let mut cfg = Config::rv64i();
    cfg.ext = Extensions {
        c: true,
        ..Extensions::I
    };
    let (agreed, _, _) = sweep(cfg, 0x5eed_0003, 2_000);
    assert!(
        agreed > 1_000,
        "only {agreed} of 2000 cases ran to completion"
    );
}

#[test]
fn a_long_program_lifts_up_to_the_block_limit_and_still_agrees() {
    // The short programs above mostly end at a branch or an access. This one
    // is long enough to reach `lift::MAX_INSNS`, which is the other way a
    // block ends and the one where the tick column is longest.
    let mut rng = Lcg(0x5eed_0004);
    let mut agreed = 0;
    for n in 0..200 {
        // Arithmetic only: forms 0..29 are the integer computation half of the
        // subset, so nothing here ends the block early.
        let program: Vec<u32> = (0..96)
            .map(|_| {
                let bits = rng.next();
                synthesize((bits >> 40) as u32 % 30, bits as u32)
            })
            .collect();
        let case = Case::seeded(program);
        match compare(&case) {
            Ok(Verdict::Agreed { insns, .. }) => {
                assert_eq!(insns, 64, "an arithmetic-only program fills the block");
                agreed += 1;
            }
            Ok(other) => panic!("case {n} produced {other:?}, not a full block"),
            Err(e) => panic!("case {n} diverged:\n{e}"),
        }
    }
    assert_eq!(agreed, 200);
}

// ---------------------------------------------------------------------------
// The cached and chained path
// ---------------------------------------------------------------------------

/// Run `count` generated cases from `seed` through the translation runtime.
///
/// Same corpus, same oracle, a different subject: `compare_cached` runs many
/// blocks through `jit::Dispatcher`, so a case here exercises the block cache,
/// block chaining, the page filter, and the software TLB on every access. The
/// sweeps above cover none of those — a single block is never served twice, no
/// exit is ever patched, and nothing is ever invalidated.
///
/// Returns `(agreed, trapped, nothing, chained)`. The last is what lets a test
/// assert that chaining actually happened, rather than that nothing broke
/// while it did not.
#[cfg(feature = "jit")]
fn cached_sweep(cfg: Config, seed: u64, count: usize, blocks: usize) -> (usize, usize, usize, u64) {
    use rsemu::cpu::riscv::differential::measure_cached;

    let mut rng = Lcg(seed);
    let (mut agreed, mut trapped, mut nothing, mut chained) = (0, 0, 0, 0u64);
    for n in 0..count {
        let len = 1 + (rng.next() % 12) as usize;
        let case = Case::seeded(program(&mut rng, len)).with_config(cfg);
        match measure_cached(&case, blocks) {
            Ok(run) => {
                chained += run.chained;
                match run.verdict {
                    Verdict::Agreed { .. } => agreed += 1,
                    Verdict::Trapped { .. } => trapped += 1,
                    Verdict::Nothing => nothing += 1,
                }
            }
            Err(e) => panic!("case {n} of seed {seed:#x} diverged on the cached path:\n{e}"),
        }
    }
    (agreed, trapped, nothing, chained)
}

#[cfg(feature = "jit")]
#[test]
fn the_generated_corpus_agrees_through_the_block_cache_and_the_software_tlb() {
    let (agreed, trapped, nothing, chained) = cached_sweep(Config::rv64i(), 0x5eed_0001, 2_000, 8);
    assert!(
        agreed > 1_000,
        "only {agreed} of 2000 cases ran to completion ({trapped} trapped, {nothing} lifted \
         nothing)"
    );
    assert!(
        chained > 0,
        "not one exit was ever patched, so chaining was never exercised"
    );
}

#[cfg(feature = "jit")]
#[test]
fn the_cached_path_agrees_on_a_hart_that_traps_misaligned_accesses_too() {
    // The misalignment policy is in the cache key, so serving a block lifted
    // under one policy to a hart running the other is exactly the bug the key
    // exists to prevent. Running the whole corpus both ways is how that stays
    // true.
    let mut strict = Config::rv64i();
    strict.misaligned = false;
    let (agreed, trapped, _, _) = cached_sweep(strict, 0x5eed_0002, 1_000, 8);
    assert!(
        agreed > 200,
        "only {agreed} of 1000 cases ran to completion"
    );
    assert!(trapped > 0, "the fault column was never tested");
}

#[cfg(feature = "jit")]
#[test]
fn the_cached_path_agrees_on_a_core_with_compressed_instructions() {
    let mut cfg = Config::rv64i();
    cfg.ext = Extensions {
        c: true,
        ..Extensions::I
    };
    let (agreed, _, _, _) = cached_sweep(cfg, 0x5eed_0003, 1_000, 8);
    assert!(
        agreed > 500,
        "only {agreed} of 1000 cases ran to completion"
    );
}

#[cfg(feature = "jit")]
#[test]
fn a_long_cached_run_stays_in_agreement_for_a_hundred_blocks() {
    // The sweeps above run eight blocks, which is enough to chain but not
    // enough for a cached block to be re-served many times or for the page
    // filter to see much traffic. This one runs a hundred, on programs whose
    // branches make the block graph revisit itself.
    let mut rng = Lcg(0x5eed_0005);
    for n in 0..200 {
        let case = Case::seeded(program(&mut rng, 10));
        if let Err(e) = rsemu::cpu::riscv::differential::compare_cached(&case, 100) {
            panic!("case {n} diverged after up to a hundred blocks:\n{e}");
        }
    }
}
