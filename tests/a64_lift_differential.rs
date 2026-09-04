//! The AArch64 IR frontend against the interpreter, over a generated corpus.
//!
//! CLAUDE.md's "CPU cores" rule makes the interpreter the oracle and
//! `cpu::arm::a64::lift` differentially tested against it *forever*.
//! `cpu::arm::a64::differential` is the comparison — every general register,
//! the stack pointer, all four `PSTATE` flags, the program counter, the cycle
//! counter, the static tick column and guest memory — and this file is what
//! drives it over enough programs to find something.
//!
//! # The corpus is generated, not written
//!
//! A fixed 64-bit LCG with Knuth's MMIX constants, so the corpus is
//! byte-identical on every machine and in every run and a failure is
//! reproducible from the seed and the case index alone. The **high** bits of
//! each draw pick the instruction form and the low bits pick the operands, so
//! two instructions in sequence do not share their register numbers.
//!
//! # Why the coverage floors are asserted
//!
//! A sweep that stopped exercising the lifter would keep passing: every case
//! would come back [`Verdict::Nothing`] and every comparison would trivially
//! hold. So each test asserts a floor on how many cases *agreed* and — where
//! the fault path is what is being exercised — that at least one **trapped**,
//! which is what keeps the precise-state column live.

#![cfg(feature = "cpu-arm-a64-lift")]

use rsemu::cpu::arm::a64::Config;
use rsemu::cpu::arm::a64::differential::{Case, Verdict, compare, synthesize};
use rsemu::cpu::arm::a64::isa::Nzcv;
use rsemu::cpu::arm::a64::lift::Shape;

/// Knuth's MMIX linear congruential generator.
///
/// Named and fixed rather than a hasher: the point of a generated corpus is
/// that it is the *same* corpus everywhere, so a divergence found in CI is
/// reproducible on a developer's machine from the seed alone.
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

/// A program of `len` instructions, closed by something outside the subset so
/// a run cannot fall off the end into the data window.
fn program(rng: &mut Lcg, len: usize) -> Vec<u32> {
    let mut out: Vec<u32> = (0..len)
        .map(|_| {
            let bits = rng.next();
            synthesize((bits >> 40) as u32, bits as u32)
        })
        .collect();
    // `svc #0`: outside the lifted subset, so the block ends here rather than
    // lifting whatever the next page holds.
    out.push(0xd400_0001);
    out
}

/// What a sweep found, so a test can assert it exercised something.
#[derive(Default, Debug)]
struct Coverage {
    agreed: usize,
    trapped: usize,
    nothing: usize,
    insns: usize,
}

fn sweep(cfg: Config, seed: u64, count: usize, shape: Shape) -> Coverage {
    sweep_with(cfg, seed, count, shape, false)
}

fn sweep_with(cfg: Config, seed: u64, count: usize, shape: Shape, strict: bool) -> Coverage {
    let mut rng = Lcg(seed);
    let mut cover = Coverage::default();
    for n in 0..count {
        let len = 1 + (rng.next() % 12) as usize;
        // The starting flags vary per case, because a conditional select or a
        // `B.cond` in the *first* instruction reads what it was given rather
        // than what the program set — and a corpus that always started at
        // `NZCV = 0` would take one side of every such branch.
        let nzcv = Nzcv::from_nibble(rng.next() as u32);
        let mut case = Case::seeded(program(&mut rng, len))
            .with_config(cfg)
            .with_shape(shape)
            .with_nzcv(nzcv);
        if strict {
            case = case.strict();
        }
        match compare(&case) {
            Ok(Verdict::Agreed { insns, .. }) => {
                cover.agreed += 1;
                cover.insns += insns;
            }
            Ok(Verdict::Trapped { insns }) => {
                cover.trapped += 1;
                cover.insns += insns;
            }
            Ok(Verdict::Nothing) => cover.nothing += 1,
            Err(e) => panic!("case {n} of seed {seed:#x} diverged under {shape:?}:\n{e}"),
        }
    }
    cover
}

#[test]
fn a_generated_corpus_agrees_with_the_interpreter_on_a_cortex_a53() {
    let cover = sweep(Config::cortex_a53(), 0x5eed_0001, 2000, Shape::Trace);
    assert!(cover.agreed > 1500, "{cover:?}");
    assert!(
        cover.trapped > 0,
        "the fault path was never reached: {cover:?}"
    );
    assert!(cover.insns > 5_000, "{cover:?}");
}

#[test]
fn the_same_corpus_agrees_on_a_part_with_the_large_system_atomics() {
    // A Neoverse N1 decodes `CAS` and `SWP` where a Cortex-A53 must not, so
    // the two end a block in different places over identical bytes — which is
    // exactly why `Features` is in the cache key.
    let cover = sweep(Config::neoverse_n1(), 0x5eed_0002, 1500, Shape::Trace);
    assert!(cover.agreed > 600, "{cover:?}");
    assert!(cover.trapped > 0, "{cover:?}");
}

#[test]
fn the_same_corpus_agrees_on_a_core_that_checks_alignment() {
    // `SCTLR_EL1.A` turns every unaligned access into a fault instead of a
    // per-byte split, which is a different `Align` in every `MemOp` and a
    // different tick count on every access.
    let cover = sweep_with(Config::cortex_a53(), 0x5eed_0003, 1500, Shape::Trace, true);
    assert!(cover.agreed > 500, "{cover:?}");
    assert!(cover.trapped > 0, "{cover:?}");
}

#[test]
fn every_shape_agrees_with_the_interpreter_over_the_whole_corpus() {
    // The same seed for all three, so a divergence names the shape rather than
    // the case.
    let mut total_trapped = 0usize;
    for shape in [Shape::BasicBlock, Shape::Extended, Shape::Trace] {
        let cover = sweep(Config::cortex_a53(), 0x5eed_0011, 1000, shape);
        assert!(cover.agreed > 300, "{shape:?}: {cover:?}");
        total_trapped += cover.trapped;
    }
    assert!(total_trapped > 0, "the fault path was never reached");
}

#[test]
fn a_trace_retires_far_more_instructions_per_block_than_a_basic_block() {
    // Not a speed claim, a *coverage* claim: if merging stopped happening the
    // trace shape would silently become the extended one and nothing else in
    // this file would notice.
    let basic = sweep(Config::cortex_a53(), 0x5eed_0012, 400, Shape::BasicBlock);
    let trace = sweep(Config::cortex_a53(), 0x5eed_0012, 400, Shape::Trace);
    assert!(
        trace.insns > basic.insns,
        "basic {basic:?} against trace {trace:?}"
    );
}

#[test]
fn a_long_program_lifts_up_to_the_block_limit_and_still_agrees() {
    // Arithmetic only — the low forms — so nothing ends the block early and
    // every case reaches the instruction limit.
    let mut rng = Lcg(0x5eed_0004);
    for n in 0..200 {
        let program: Vec<u32> = (0..96)
            .map(|_| {
                let bits = rng.next();
                synthesize((bits >> 40) as u32 % 41, bits as u32)
            })
            .collect();
        let case = Case::seeded(program);
        match compare(&case) {
            Ok(Verdict::Agreed { .. } | Verdict::Trapped { .. } | Verdict::Nothing) => {}
            Err(e) => panic!("case {n} diverged:\n{e}"),
        }
    }
}

#[cfg(feature = "jit")]
mod cached {
    use super::*;
    use rsemu::cpu::arm::a64::differential::measure_cached;

    fn cached_sweep(seed: u64, count: usize, blocks: usize) -> (Coverage, u64, u64) {
        let mut rng = Lcg(seed);
        let mut cover = Coverage::default();
        let (mut chained, mut compiled) = (0u64, 0u64);
        for n in 0..count {
            let len = 1 + (rng.next() % 12) as usize;
            let case = Case::seeded(program(&mut rng, len));
            match measure_cached(&case, blocks) {
                Ok(run) => {
                    chained += run.chained;
                    compiled += run.compiled;
                    match run.verdict {
                        Verdict::Agreed { insns, .. } => {
                            cover.agreed += 1;
                            cover.insns += insns;
                        }
                        Verdict::Trapped { insns } => {
                            cover.trapped += 1;
                            cover.insns += insns;
                        }
                        Verdict::Nothing => cover.nothing += 1,
                    }
                }
                Err(e) => panic!("case {n} of seed {seed:#x} diverged through the cache:\n{e}"),
            }
        }
        (cover, chained, compiled)
    }

    #[test]
    fn the_generated_corpus_agrees_through_the_block_cache_and_the_chain() {
        let (cover, chained, _) = cached_sweep(0x5eed_0001, 1500, 8);
        assert!(cover.agreed > 600, "{cover:?}");
        assert!(cover.trapped > 0, "{cover:?}");
        assert!(chained > 0, "no exit was ever patched to its successor");
    }

    #[test]
    fn a_long_cached_run_stays_in_agreement_for_a_hundred_blocks() {
        let mut rng = Lcg(0x5eed_0005);
        for n in 0..200 {
            let case = Case::seeded(program(&mut rng, 10));
            if let Err(e) = measure_cached(&case, 100) {
                panic!("case {n} diverged over a hundred blocks:\n{e}");
            }
        }
    }
}

#[cfg(all(feature = "jit-x86", target_os = "linux", target_arch = "x86_64"))]
mod compiled {
    use super::*;
    use rsemu::cpu::arm::a64::differential::measure_compiled;

    #[test]
    fn the_generated_corpus_agrees_when_it_is_compiled_to_host_code() {
        // Three sweeps, one per shape, each with its own seed so a divergence
        // names the shape.
        for (seed, shape) in [
            (0xc0de_0001u64, Shape::BasicBlock),
            (0xc0de_0002, Shape::Extended),
            (0xc0de_0003, Shape::Trace),
        ] {
            let mut rng = Lcg(seed);
            let (mut agreed, mut blocks, mut compiled) = (0usize, 0usize, 0u64);
            for n in 0..600 {
                let len = 1 + (rng.next() % 12) as usize;
                let case = Case::seeded(program(&mut rng, len)).with_shape(shape);
                match measure_compiled(&case, 8) {
                    Ok(run) => {
                        blocks += run.blocks;
                        compiled += run.compiled;
                        if matches!(run.verdict, Verdict::Agreed { .. }) {
                            agreed += 1;
                        }
                    }
                    Err(e) => panic!("case {n} of seed {seed:#x} diverged compiled:\n{e}"),
                }
            }
            assert!(agreed > 200, "{shape:?}: agreed {agreed}");
            // Without this a backend that had silently stopped compiling would
            // pass this file forever.
            assert!(
                compiled * 2 > blocks as u64,
                "{shape:?}: {compiled} of {blocks} blocks were compiled"
            );
        }
    }
}
