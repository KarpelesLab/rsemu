//! A real AArch64 Linux boot, measured — the workload `ROADMAP.md` §8 is about.
//!
//! The three dispatch benchmarks beside this one (`a64_dispatch`,
//! `jit_dispatch`, `x86_dispatch`) each time a hand-written loop through the
//! mechanism ladder. That is the right shape for attributing a *mechanism*,
//! and it is the wrong shape for deciding what to optimise next: a
//! seven-instruction loop that fits one block has no block-boundary cost, no
//! cold translations, no self-modifying code and no MMU. §8's gate is
//! wall-clock on a real guest, so this benchmark runs one.
//!
//! What it prints is a **census**, not a ladder: how long the boot took, and
//! then the numbers a profile has to be divided by to mean anything — blocks
//! executed, guest instructions retired inside them, how many were reached by
//! a patched exit, how many accesses the inlined TLB probe served. A host
//! instruction count from callgrind is only interpretable against those.
//!
//! ```text
//!   scripts/fetch-testdata.sh arm64-linux arm64-initramfs
//!
//!   RSEMU_ARM64_KERNEL=testdata/arm64/linux \
//!   RSEMU_ARM64_INITRD=testdata/arm64/initramfs.cpio \
//!       cargo bench --features machine-arm64-virt,cpu-arm-a64-lift,jit-x86 \
//!           --bench a64_linux_boot
//!
//!   … -- --seconds 20 --engine jit-host --reps 3
//! ```
//!
//! With no kernel it **skips loudly** and prints the two lines above, exactly
//! as `tests/engine_longrun.rs` does: the kernel is a GPL-2.0 binary, running
//! one as an emulated guest is ordinary use and committing one here would be
//! redistribution (`CLAUDE.md`, *Testing*).
//!
//! # Under callgrind
//!
//! This host drifts several nanoseconds per instruction under concurrent
//! load, so a wall clock is not a measurement of a change smaller than a few
//! percent. Host instructions are, and they are what the attribution in
//! `docs/testing/` is taken with:
//!
//! ```text
//!   valgrind --tool=callgrind --smc-check=all-non-file --cache-sim=no \
//!       target/release/deps/a64_linux_boot-… --seconds 20 --reps 1
//! ```
//!
//! `--smc-check=all-non-file` is not optional: the generated code lives in an
//! anonymous mapping this process writes and then executes, and valgrind's
//! default translation cache would run the bytes it saw first. `--hash` is off
//! by default for the same reason: hashing this board walks a gigabyte of
//! guest RAM, and it would otherwise be a fifth of the profile.
//!
//! # What that profile said the first time it was taken
//!
//! Twenty guest seconds of the boot, `engine = "jit-host"`, 53 930 067 136
//! host instructions against 154 233 793 guest instructions retired — **350
//! host instructions per guest instruction**, of which the code the JIT
//! generated was **twenty-five**.
//!
//! | | share | per guest instruction |
//! | --- | --- | --- |
//! | the dispatch loop (`Cpu::advance`, everything inlined into it) | 24.3% | 85 |
//! | replaying deferred charges and boundaries (`flush_thunk`) | 19.3% | 68 |
//! | the address space and the software TLB | 18.4% | 64 |
//! | admitting a block: the entry fetch, its walk, the interrupt check | 13.7% | 48 |
//! | reading a guest register (`get_slot_thunk`) | 7.8% | 27 |
//! | **the code the JIT generated** | **7.7%** | **27** |
//! | lifting, verifying, allocating and compiling | 2.6% | 9 |
//! | the instructions the interpreter took (1.2% of them) | 1.9% | 7 |
//!
//! Those account for 95.7% of the run. 2.7 points of the address-space row are
//! a single call: `RamStore::fill` zeroing a gigabyte of guest RAM at reset,
//! which is a startup cost and not a rate.
//!
//! The number that decides all of it is at the bottom of the census this
//! prints: **6.44 guest instructions per block**. Every per-block cost above
//! is divided by that, and it is 6.44 rather than the frontend's limit of 64
//! because `cpu::arm::a64::lift` ends a block at every store and at every
//! computed branch. So the roadmap's phase-8 list — superblocks, cross-block
//! register allocation, memory-op fusion — is aimed at the 7.7%, and the
//! measurement says the block length is aimed at the 56%.
//!
//! # What was done about it: the store no longer ends a block
//!
//! `cpu::arm::a64::lift::Smc` is that measurement acted on. A store into the
//! page a block was lifted from is now noticed by the **host**, which sees the
//! guest-physical page of every store the block makes and compares it against
//! the one the block's own bytes came from; a match retires the run's tick
//! allowance and the block leaves at its next guest instruction boundary. The
//! frontend emits nothing for it, so what changed is only where a block ends.
//! `cpu::arm::a64::lift`'s module docs have the argument, including why
//! `cpu::x86::lift`'s in-block guard — which compares *linear* pages and is
//! therefore refused under paging — could not simply be adopted.
//!
//! The same twenty seconds, the same binary but for `engine.rs`'s `SMC`
//! constant, the same 154 233 958 guest instructions retired and the same
//! `Machine::state_hash`:
//!
//! | | ends the block | the host guard |
//! | --- | --- | --- |
//! | **host instructions** | 51 758 768 516 | **44 220 119 264** (−14.56%) |
//! | blocks executed | 23 935 454 | **14 283 856** (−40.3%) |
//! | **guest instructions per block** | **6.44** | **10.80** |
//! | distinct blocks lifted | 17 256 | 12 173 |
//! | *and then, as a share of each run:* | | |
//! | the dispatch loop (`Cpu::advance`) | 24.80% | 18.60% |
//! | replaying deferred bookkeeping (`flush_thunk`) | 19.11% | 21.66% |
//! | `admit` | 5.23% | 3.81% |
//! | the entry translation (`Exec::translate`) | 4.15% | 3.42% |
//! | reading a guest register (`get_slot_thunk`) | 3.67% | 3.44% |
//! | the per-block TLB resync (`jit::Tlb::sync`) | 2.01% | 1.44% |
//!
//! Every per-block row fell by a third to two fifths, which is what a block
//! length that went up by 1.68× buys. The replay is the exception and it is
//! the informative one: it fell by 3.2% in absolute terms and *rose* as a
//! share, because what a replay costs is mostly the events in it — the same
//! thing `ir::hoist_slot_reads` found when it removed a third of the calls and
//! only a twentieth of the cost. **What is left to aim at has moved**: the
//! replay is now the largest row in this profile, and the next thing that
//! shortens it is fewer events rather than fewer blocks.

use std::sync::Arc;
use std::time::{Duration, Instant};

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::cpu::arm::a64::Cpu;
use rsemu::machine::{Machine, build, catalog};

fn main() {
    let args = Args::parse(std::env::args().skip(1));
    let Some(kernel) = fixture("RSEMU_ARM64_KERNEL") else {
        skip();
        return;
    };
    let initrd = fixture("RSEMU_ARM64_INITRD").unwrap_or_default();

    println!(
        "rsemu arm64-virt Linux boot — engine={}, {} virtual seconds, {} rep(s)\n",
        args.engine, args.seconds, args.reps
    );
    println!(
        "{:<10} {:>9} {:>12} {:>10} {:>18}",
        "rep", "wall (s)", "MIPS", "blocks/s", "state hash"
    );

    let mut best = Duration::MAX;
    let mut census = None;
    for rep in 0..args.reps {
        let (elapsed, stats, hash) = boot(&kernel, &initrd, &args);
        let mips = stats.retired as f64 / elapsed.as_secs_f64() / 1e6;
        let bps = stats.blocks as f64 / elapsed.as_secs_f64() / 1e6;
        println!(
            "{:<10} {:>9.3} {:>12.2} {:>9.2}M {:>#18x}",
            rep,
            elapsed.as_secs_f64(),
            mips,
            bps,
            hash
        );
        if elapsed < best {
            best = elapsed;
            census = Some(stats);
        }
    }

    let Some(s) = census else { return };
    let retired = s.retired.max(1) as f64;
    let blocks = s.blocks.max(1) as f64;
    println!("\nthe census, from the fastest rep — every rate is per guest instruction\n");
    row("blocks executed", s.blocks, s.blocks as f64 / retired);
    row(
        "  compiled to host code",
        s.compiled,
        s.compiled as f64 / blocks,
    );
    row(
        "  reached by a patched exit",
        s.chained,
        s.chained as f64 / blocks,
    );
    row(
        "distinct blocks lifted",
        s.translated,
        s.translated as f64 / blocks,
    );
    row("guest insns retired in blocks", s.retired, 1.0);
    row(
        "guest insns interpreted",
        s.interpreted,
        s.interpreted as f64 / retired,
    );
    row(
        "inlined-probe loads",
        s.fast_loads,
        s.fast_loads as f64 / retired,
    );
    row(
        "inlined-probe stores",
        s.fast_stores,
        s.fast_stores as f64 / retired,
    );
    row(
        "translations a block's store killed",
        s.smc,
        s.smc as f64 / retired,
    );
    row(
        "translations an interpreted store killed",
        s.smc_interpreted,
        s.smc_interpreted as f64 / retired,
    );
    println!("\nguest instructions per block: {:.2}", retired / blocks);
    println!(
        "the fraction retired inside a block: {:.2}%",
        100.0 * retired / (retired + s.interpreted as f64)
    );
}

fn row(what: &str, n: u64, per: f64) {
    println!("{what:<42} {n:>14} {per:>10.4}");
}

/// One boot, timed.
fn boot(
    kernel: &[u8],
    initrd: &[u8],
    args: &Args,
) -> (Duration, rsemu::cpu::arm::a64::JitStats, u64) {
    let (mut machine, cpu) = board(kernel, initrd, args);
    let span = GlobalTime::from_nanos(args.seconds * 1_000_000_000);
    let start = Instant::now();
    machine.run_for(span).expect("the machine advances");
    let elapsed = start.elapsed();
    let stats = cpu.jit_stats().unwrap_or_default();
    // Off by default under a profiler: hashing a machine walks every byte of
    // guest RAM, which on this board is a gigabyte, and it lands in the
    // profile as if it were emulation. `--hash` is what makes it a
    // *determinism* check again — the two engines must agree on it, and the
    // number is the same one `rsemu run` prints.
    let hash = if args.hash {
        machine.state_hash().expect("the machine hashes")
    } else {
        0
    };
    (elapsed, stats, hash)
}

/// `arm64-virt` from the catalog, with a handle on its core.
///
/// The handle is what makes this a census rather than a stopwatch, and there
/// is no route from a `dyn Device` to a `Cpu` — `core::device` keeps `Any` out
/// of the supertrait chain deliberately — so the core is captured as it is
/// built, the way `tests/a64_engines.rs` captures one.
fn board(kernel: &[u8], initrd: &[u8], args: &Args) -> (Machine, Arc<Cpu>) {
    let cpus: Arc<Captured<Cpu>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cpus);
    let mut bindings = catalog::bindings().expect("this build's bindings");
    bindings.replace("cpu.arm.a64", move |props| {
        let cpu = Arc::new(Cpu::from_props(props)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    let entry = catalog::machine("arm64-virt").expect("this build ships it");
    let options = catalog::build_options()
        .expect("the catalog agrees with itself")
        .with_bindings(bindings)
        .with_media("kernel", kernel)
        .with_media("initrd", initrd)
        .with_media("disk", Vec::new())
        .with_param("ram", "1G")
        .with_param("engine", &args.engine)
        .with_param(
            "cmdline",
            "earlycon=pl011,0x9000000 console=ttyAMA0 rdinit=/init",
        );
    let registry = catalog::registry().expect("a registry");
    let machine = build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("arm64-virt does not build: {e}"));
    let cpu = cpus.take().expect("the binding captured the core");
    (machine, cpu)
}

fn fixture(var: &str) -> Option<Vec<u8>> {
    let path = std::env::var(var).ok()?;
    Some(
        std::fs::read(&path)
            .unwrap_or_else(|e| panic!("{var} names `{path}`, which will not read: {e}")),
    )
}

fn skip() {
    println!(
        "a64_linux_boot: no kernel, so nothing was measured.\n\n\
         \x20 scripts/fetch-testdata.sh arm64-linux arm64-initramfs\n\n\
         \x20 RSEMU_ARM64_KERNEL=testdata/arm64/linux \\\n\
         \x20 RSEMU_ARM64_INITRD=testdata/arm64/initramfs.cpio \\\n\
         \x20     cargo bench --features machine-arm64-virt,cpu-arm-a64-lift,jit-x86 \\\n\
         \x20         --bench a64_linux_boot"
    );
}

/// What the command line can change.
struct Args {
    /// How much *virtual* time to run, in seconds.
    seconds: u64,
    /// `interp`, `jit` or `jit-host`.
    engine: String,
    reps: usize,
    /// Take a state hash at the end of each rep. Off by default: it walks a
    /// gigabyte of guest RAM and would otherwise be a quarter of a profile.
    hash: bool,
}

impl Args {
    fn parse(args: impl Iterator<Item = String>) -> Args {
        let mut out = Args {
            seconds: 20,
            engine: "jit-host".to_string(),
            reps: 1,
            hash: false,
        };
        let mut it = args.peekable();
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--seconds" => out.seconds = next(&mut it, "--seconds").parse().expect("a number"),
                "--engine" => out.engine = next(&mut it, "--engine"),
                "--reps" => out.reps = next(&mut it, "--reps").parse().expect("a number"),
                "--hash" => out.hash = true,
                // libtest passes these through `cargo bench`; ignore them
                // rather than fail, since `harness = false` means they are not
                // ours to interpret.
                "--bench" | "--test" => {}
                other => panic!("a64_linux_boot: unknown argument `{other}`"),
            }
        }
        out
    }
}

fn next(it: &mut impl Iterator<Item = String>, what: &str) -> String {
    it.next()
        .unwrap_or_else(|| panic!("a64_linux_boot: {what} wants a value"))
}
