//! The native column that `web/check.mjs`'s browser column is read against.
//!
//! `ROADMAP.md` §11.4 asks one question about the WebAssembly backend — *is a
//! wasm module faster than the IR interpreter in a browser?* — and until an
//! embedder existed there was no honest way to answer it. There is now, and
//! the answer is two numbers rather than one:
//!
//! * **In a browser**, `web/check.mjs` §1c runs this same guest for this same
//!   span through `rsemu_jit_guest_run` and times it. The engine there
//!   compiles each generated module once and runs it.
//! * **Here**, the identical call on a native host, where the only thing that
//!   can run a generated module is `jit::wasm::exec` — a wasm interpreter, so
//!   a block is interpreted twice over and `jit-wasm` is the slowest of the
//!   four engines by construction.
//!
//! The comparison is only worth anything because it is the *same function*:
//! `crate::wasm::rsemu_jit_guest_run` builds the guest, runs the span and
//! hashes the result, and both callers do nothing but call it and look at a
//! clock. A benchmark here and a harness there, each with its own fixture,
//! would be two workloads with one name.
//!
//! # What is measured
//!
//! Seven RV64I instructions in a loop, with the scratch word on the page after
//! the code so the block is compiled once rather than invalidated every pass
//! (`src/wasm.rs`'s `GUEST` says why that distinction decides what gets
//! measured). Every instruction is in the lifted subset, so the wasm backend
//! lowers the block rather than refusing it — the `compiled` column is what
//! says so, and a run whose `compiled` is zero is a measurement of the
//! interpreter under a different name.
//!
//! # Running it
//!
//! ```sh
//! cargo bench --bench wasm_jit_embedder \
//!     --no-default-features --features wasm,jit-wasm,cpu-riscv-lift
//! ```
//!
//! `jit-host` appears only where its backend does (x86-64 or aarch64 Linux
//! with `jit-x86`/`jit-arm64`); elsewhere `Jit::new` falls back to the
//! portable backend, which is its own documented rule and is why the column is
//! labelled with what it got rather than with what was asked for.

use std::time::Instant;

use rsemu::wasm::{rsemu_jit_guest_run, rsemu_jit_guest_stat};

/// Quanta and ticks-per-quantum, matching `web/check.mjs` §1c exactly.
///
/// Changing either here without changing it there turns the two halves of one
/// measurement into two measurements, so they are stated in both places and
/// the doc comment in each names the other.
const QUANTA: u32 = 2000;
const BUDGET: u64 = 20000;

fn main() {
    let reps: u32 = std::env::args()
        .nth(1)
        .and_then(|a| a.parse().ok())
        .unwrap_or(3);

    println!(
        "rsemu wasm-JIT embedder benchmark — {QUANTA} quanta x {BUDGET} ticks, best of {reps}\n"
    );
    // `compiled` counts blocks that ran as *generated code* rather than on
    // `ir::Interp`, so it is zero for both `interp` and `jit` — the portable
    // backend executes the IR — and it is the column that says the wasm
    // backend was reached rather than silently refusing every block.
    println!(
        "{:<10} {:>10} {:>9} {:>12} {:>12} {:>10}",
        "engine", "ms", "vs interp", "instructions", "compiled", "in modules"
    );

    let mut baseline = f64::NAN;
    let mut want_hash = None;
    for (name, engine) in [
        ("interp", 0u32),
        ("jit", 1),
        ("jit-host", 2),
        ("jit-wasm", 3),
    ] {
        // Warmed up, then best of `reps`: the first pass pays for the lift and
        // the compile of every block, which is a real cost but not the one
        // this table is about — `docs/techniques/wasm-jit.md`'s instantiation
        // section is where that belongs.
        rsemu_jit_guest_run(engine, 200, 2000);
        let mut best = f64::INFINITY;
        let mut hash = 0;
        for _ in 0..reps {
            let started = Instant::now();
            hash = rsemu_jit_guest_run(engine, QUANTA, BUDGET);
            best = best.min(started.elapsed().as_secs_f64() * 1000.0);
        }
        if name == "interp" {
            baseline = best;
        }
        // Every engine is the same guest or the benchmark is meaningless, and
        // this is the cheapest place to find out it is not.
        match want_hash {
            None => want_hash = Some(hash),
            Some(want) => assert_eq!(
                hash, want,
                "`{name}` is not running the same guest as the interpreter"
            ),
        }
        println!(
            "{:<10} {:>10.0} {:>8.2}x {:>12} {:>12} {:>10}",
            name,
            best,
            baseline / best,
            rsemu_jit_guest_stat(7),
            rsemu_jit_guest_stat(1),
            rsemu_jit_guest_stat(4),
        );
    }

    println!(
        "\n`in modules` is zero on every native host by construction: instantiating a\n\
         module is something only an embedder can do, and rsemu's embedder is the page.\n\
         `web/check.mjs --jit` is the same run with one behind it."
    );
}
