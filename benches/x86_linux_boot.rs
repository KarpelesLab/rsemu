//! A real x86-64 Linux boot, measured — the x86 half of `benches/a64_linux_boot`.
//!
//! `benches/x86_dispatch` beside this one times a hand-written loop through the
//! frontend's mechanism ladder. That is the right shape for attributing a
//! *mechanism* and the wrong one for deciding what to optimise next: a loop
//! that fits one block has no block-boundary cost, no cold translations, no
//! self-modifying code and no page tables. `ROADMAP.md` §8's gate is
//! wall-clock on a real guest, so this benchmark boots one.
//!
//! What it prints is a **census**, not a ladder: how long the boot took, and
//! then the numbers a profile has to be divided by to mean anything — blocks
//! executed, guest instructions retired inside them, how many were reached by
//! a patched exit, how many translations a store threw away. A host
//! instruction count from callgrind is only interpretable against those.
//!
//! ```text
//!   scripts/fetch-testdata.sh x86-linux initramfs-x86
//!
//!   RSEMU_X86_KERNEL=testdata/x86/bzImage \
//!   RSEMU_X86_INITRD=testdata/x86/initramfs-x86.cpio \
//!       cargo bench --features machine-pc64,cpu-x86-lift,jit,jit-x86 \
//!           --bench x86_linux_boot
//!
//!   … -- --seconds 120 --engine jit-host --reps 3
//! ```
//!
//! With no kernel it **skips loudly** and prints the two lines above, exactly
//! as `tests/engine_longrun.rs` does: the kernel is a GPL-2.0 binary, running
//! one as an emulated guest is ordinary use and committing one here would be
//! redistribution (`CLAUDE.md`, *Testing*).
//!
//! The board is `tests/engine_longrun.rs`'s `pc64`, with its command line and
//! its reasons: `nokaslr` because this board has no firmware, so nothing ever
//! loaded a count into the 8254 and a kernel drawing entropy from the read-back
//! command's null-count bit spins in the decompressor forever, and
//! `cryptomgr.notests` because it is the difference between a few hundred guest
//! seconds of boot and a few thousand.
//!
//! # Under callgrind
//!
//! A wall clock on a loaded host is not a measurement of a change smaller than
//! a few percent. Host instructions are:
//!
//! ```text
//!   valgrind --tool=callgrind --smc-check=all-non-file --cache-sim=no \
//!       target/release/deps/x86_linux_boot-… --seconds 120 --reps 1
//! ```
//!
//! `--smc-check=all-non-file` is not optional: the generated code lives in an
//! anonymous mapping this process writes and then executes, and valgrind's
//! default translation cache would run the bytes it saw first. `--hash` is off
//! by default for the same reason it is on the A64 benchmark: hashing this
//! board walks every byte of its extended memory, and it would otherwise be a
//! slice of the profile that is not emulation.
//!
//! **A nine-hundred-second boot does not finish under callgrind in useful
//! time.** On an idle host it starts at about 480 M host instructions a second
//! and is under 40 M once the kernel proper is running, because `--smc-check`
//! makes valgrind re-check its own translations of the mapping the JIT writes
//! and the kernel phase is where a hundred thousand distinct blocks are
//! compiled into it. Profile a shorter span — `--seconds 120` is the
//! decompressor and finishes in minutes — and take the census natively over
//! the nine hundred, which costs under a minute.
//!
//! # What that profile said the first time it was taken
//!
//! 120 guest seconds, `engine = "jit-host"`, 117 100 730 428 host instructions
//! against 179 470 710 guest instructions — **652 host instructions per guest
//! instruction, of which the code the JIT generated was 54**. The address
//! space is 22.3% of it and the dispatch loop 14.6%; the largest per-block
//! terms are all divided by how long a block is, and that was 5.21 guest
//! instructions over the whole boot against A64's 6.44.
//!
//! `cpu::x86::lift::Smc::HostGuard` is that measurement acted on and took the
//! divisor to 12.17. `docs/platforms/pc64.md`, *"Where the host instructions
//! go"*, has the attribution either side of it and the method in full.
//!
//! # And what it said next, which is the row this census gained two lines for
//!
//! With the divisor fixed, the largest single item left was the address space
//! at 146 host instructions per guest instruction — because x86 published no
//! inlined memory path and every load and store a block made took a call. It
//! publishes one now, in long mode, and the two rows at the bottom of the
//! census are how far it reaches: **0.187 loads and 0.107 stores per guest
//! instruction** over the whole nine hundred, served by a probe in generated
//! code with no call at all.
//!
//! On the same hundred and twenty guest seconds either side of that change:
//! **82 149 446 793 host instructions to 61 067 485 370, −25.7%** — 458 per
//! guest instruction retired in a block to 340 — with the census identical in
//! every row and the nine-hundred-second `Machine::state_hash` unchanged at
//! `0xb996f48fb92dd24b`. `docs/platforms/pc64.md`, *"The inlined memory
//! path"*, has the per-function attribution and what the design refuses.

use std::sync::Arc;
use std::time::{Duration, Instant};

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::cpu::x86::{JitStats, Variant, X86};
use rsemu::host::chardev::CharPort;
use rsemu::machine::{Machine, build, catalog};

fn main() {
    let args = Args::parse(std::env::args().skip(1));
    let Some(kernel) = fixture("RSEMU_X86_KERNEL") else {
        skip();
        return;
    };
    let initrd = fixture("RSEMU_X86_INITRD").unwrap_or_default();

    println!(
        "rsemu pc64 Linux boot — engine={}, {} virtual seconds, {} rep(s)\n",
        args.engine, args.seconds, args.reps
    );
    println!(
        "{:<10} {:>9} {:>12} {:>10} {:>18}",
        "rep", "wall (s)", "MIPS", "blocks/s", "state hash"
    );

    let mut best = Duration::MAX;
    let mut census = None;
    let mut transcript = Vec::new();
    for rep in 0..args.reps {
        let (elapsed, stats, hash, said) = boot(&kernel, &initrd, &args);
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
            transcript = said;
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
        "translations a store killed",
        s.invalidated,
        s.invalidated as f64 / retired,
    );
    row(
        "loads served by an inlined probe",
        s.fast_loads,
        s.fast_loads as f64 / retired,
    );
    row(
        "stores served by an inlined probe",
        s.fast_stores,
        s.fast_stores as f64 / retired,
    );
    println!("\nguest instructions per block: {:.2}", retired / blocks);
    println!(
        "the fraction retired inside a block: {:.2}%",
        100.0 * retired / (retired + s.interpreted as f64)
    );
    if args.console {
        println!("\nthe console, from the fastest rep:\n");
        println!("{}", String::from_utf8_lossy(&transcript));
    } else {
        println!(
            "how far it got: {} console bytes, last line {:?}",
            transcript.len(),
            last_line(&transcript)
        );
    }
}

fn row(what: &str, n: u64, per: f64) {
    println!("{what:<42} {n:>14} {per:>10.4}");
}

/// The last non-empty line the guest printed, for "did this boot get anywhere".
fn last_line(said: &[u8]) -> String {
    String::from_utf8_lossy(said)
        .lines()
        .rev()
        .find(|l| !l.trim().is_empty())
        .unwrap_or("")
        .trim_end()
        .to_string()
}

/// One boot, timed.
fn boot(kernel: &[u8], initrd: &[u8], args: &Args) -> (Duration, JitStats, u64, Vec<u8>) {
    let (mut machine, cpu, console) = board(kernel, initrd, args);
    // A slice rather than one span, because `PORT_CAPACITY` is 64 KiB and a
    // kernel that fills it has its output dropped — which is a different guest
    // from the one `tests/engine_longrun.rs` runs, where the harness drains
    // every quantum. `Machine::run_for` is additive (`ROADMAP.md` §11.6), so
    // running the span in pieces reaches the same state as running it whole.
    let slice = GlobalTime::from_nanos(100_000_000);
    let mut said = Vec::new();
    let mut left = args.seconds * 10;
    let start = Instant::now();
    while left > 0 {
        machine.run_for(slice).expect("the machine advances");
        console.drain_into(&mut said);
        left -= 1;
    }
    let elapsed = start.elapsed();
    console.drain_into(&mut said);
    let stats = cpu.jit_stats().unwrap_or_default();
    // Off by default under a profiler: hashing a machine walks every byte of
    // guest RAM, and it lands in the profile as if it were emulation.
    // `--hash` is what makes it a *determinism* check again — the two engines
    // must agree on it, and the number is the same one `rsemu run` prints.
    let hash = if args.hash {
        machine.state_hash().expect("the machine hashes")
    } else {
        0
    };
    (elapsed, stats, hash, said)
}

/// `pc64` from the catalog, with a handle on its core and on its console.
///
/// The handle is what makes this a census rather than a stopwatch, and there
/// is no route from a `dyn Device` to an `X86` — `core::device` keeps `Any` out
/// of the supertrait chain deliberately — so the core is captured as it is
/// built, the way `tests/engine_longrun.rs` captures one.
fn board(kernel: &[u8], initrd: &[u8], args: &Args) -> (Machine, Arc<X86>, Arc<CharPort>) {
    let cpus: Arc<Captured<X86>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cpus);
    let mut bindings = catalog::bindings().expect("this build's bindings");
    bindings.replace("cpu.x86", move |props| {
        let cpu = Arc::new(X86::from_props_defaulting(props, Variant::X86_64)?);
        kept.push(&cpu);
        Ok(cpu)
    });
    let entry = catalog::machine("pc64").expect("this build ships it");
    let port = "boot.console";
    let options = catalog::build_options()
        .expect("the catalog agrees with itself")
        .with_bindings(bindings)
        .with_media("kernel", kernel)
        .with_media("initrd", initrd)
        .with_param("engine", &args.engine)
        .with_param("extmem", &args.ram)
        .with_param("cmdline", &args.cmdline)
        .with_param("console", port);
    let registry = catalog::registry().expect("a registry");
    let machine = build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("pc64 does not build with engine={}: {e}", args.engine));
    let console = rsemu::host::chardev::ports::open(&options.realize.hosts, port)
        .expect("the 16550 opened this port under the same name");
    let cpu = cpus.take().expect("the binding captured the core");
    (machine, cpu, console)
}

fn fixture(var: &str) -> Option<Vec<u8>> {
    let path = std::env::var(var).ok().filter(|p| !p.is_empty())?;
    Some(
        std::fs::read(&path)
            .unwrap_or_else(|e| panic!("{var} names `{path}`, which will not read: {e}")),
    )
}

fn skip() {
    println!(
        "x86_linux_boot: no kernel, so nothing was measured.\n\n\
         \x20 scripts/fetch-testdata.sh x86-linux initramfs-x86\n\n\
         \x20 RSEMU_X86_KERNEL=testdata/x86/bzImage \\\n\
         \x20 RSEMU_X86_INITRD=testdata/x86/initramfs-x86.cpio \\\n\
         \x20     cargo bench --features machine-pc64,cpu-x86-lift,jit,jit-x86 \\\n\
         \x20         --bench x86_linux_boot"
    );
}

/// What the command line can change.
struct Args {
    /// How much *virtual* time to run, in seconds.
    ///
    /// **24, where it was 120.** Not a shorter profile: a guest second of this
    /// board is 4.96 times the processor work it used to be, because
    /// `SchedulerConfig::max_ticks_per_quantum` capped a round at ten thousand
    /// of its 100 MHz core's ticks instead of the quantum's hundred thousand,
    /// so this is the same span of the boot in the same wall clock. Every
    /// guest-second figure in this file's documentation was taken at the old
    /// rate; divide by 4.96 to get the guest time that buys it now.
    /// `docs/techniques/execution-budgets.md` has the arithmetic.
    seconds: u64,
    /// `interp`, `jit` or `jit-host`.
    engine: String,
    reps: usize,
    /// Take a state hash at the end of each rep. Off by default: it walks the
    /// board's whole extended memory and would otherwise be part of a profile.
    hash: bool,
    /// Print everything the guest said rather than its last line.
    console: bool,
    ram: String,
    cmdline: String,
}

impl Args {
    fn parse(args: impl Iterator<Item = String>) -> Args {
        let mut out = Args {
            seconds: 24,
            engine: "jit-host".to_string(),
            reps: 1,
            hash: false,
            console: false,
            ram: std::env::var("RSEMU_X86_RAM").unwrap_or_else(|_| "256M".to_string()),
            cmdline: std::env::var("RSEMU_X86_CMDLINE").unwrap_or_else(|_| {
                "console=ttyS0,115200 earlyprintk=ttyS0,115200 nokaslr cryptomgr.notests"
                    .to_string()
            }),
        };
        let mut it = args.peekable();
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--seconds" => out.seconds = next(&mut it, "--seconds").parse().expect("a number"),
                "--engine" => out.engine = next(&mut it, "--engine"),
                "--reps" => out.reps = next(&mut it, "--reps").parse().expect("a number"),
                "--hash" => out.hash = true,
                "--console" => out.console = true,
                "--ram" => out.ram = next(&mut it, "--ram"),
                "--cmdline" => out.cmdline = next(&mut it, "--cmdline"),
                // libtest passes these through `cargo bench`; ignore them
                // rather than fail, since `harness = false` means they are not
                // ours to interpret.
                "--bench" | "--test" => {}
                other => panic!("x86_linux_boot: unknown argument `{other}`"),
            }
        }
        out
    }
}

fn next(it: &mut impl Iterator<Item = String>, what: &str) -> String {
    it.next()
        .unwrap_or_else(|| panic!("x86_linux_boot: {what} wants a value"))
}
