//! A real RV64 Linux boot, measured — the RISC-V half of `ROADMAP.md` §8.
//!
//! `benches/a64_linux_boot.rs` is the same benchmark for the other 64-bit
//! core, and it says at length why a *census* rather than a mechanism ladder is
//! the right shape for deciding what to optimise next. The short version: the
//! three dispatch benchmarks each time a hand-written loop, and a
//! seven-instruction loop that fits one block has no block-boundary cost, no
//! cold translations, no self-modifying code and no MMU. §8's gate is
//! wall-clock on a real guest, so this runs one.
//!
//! ```text
//!   scripts/fetch-testdata.sh opensbi linux initramfs
//!
//!   RSEMU_RISCV_FIRMWARE=testdata/riscv/fw_jump.bin \
//!   RSEMU_RISCV_KERNEL=testdata/riscv/linux \
//!   RSEMU_RISCV_INITRD=testdata/riscv/initramfs.cpio \
//!       cargo bench --features machine-riscv-virt,cpu-riscv-lift,jit-x86 \
//!           --bench riscv_linux_boot
//!
//!   … -- --seconds 20 --engine jit-host --reps 3
//! ```
//!
//! With no firmware it **skips loudly** and prints those lines, exactly as
//! `tests/engine_longrun.rs` and the A64 benchmark do: the kernel is a GPL-2.0
//! binary, running one as an emulated guest is ordinary use and committing one
//! here would be redistribution (`CLAUDE.md`, *Testing*). OpenSBI is
//! BSD-2-Clause and could be committed; it is fetched anyway, because the rule
//! is about the repository rather than about any one file.
//!
//! # Under callgrind
//!
//! A wall clock on a loaded host does not resolve a few percent. Host
//! instructions do, and they are what the attribution below was taken with:
//!
//! ```text
//!   valgrind --tool=callgrind --smc-check=all-non-file --cache-sim=no \
//!       target/release/deps/riscv_linux_boot-… --seconds 20 --reps 1
//! ```
//!
//! `--smc-check=all-non-file` is not optional: the generated code lives in an
//! anonymous mapping this process writes and then executes, and valgrind's
//! default translation cache would run the bytes it saw first. `--hash` is off
//! by default for the same reason it is off there — hashing this board walks
//! every byte of guest RAM, and it would otherwise be a large share of the
//! profile.
//!
//! # What that profile said the first time it was taken
//!
//! Twenty guest seconds of OpenSBI's `fw_jump` handing over to a Debian
//! `riscv64` kernel, `engine = "jit-host"`, 512 MiB of guest RAM — which on
//! this board's clock reaches `ftrace: allocating …`, so what is profiled is
//! firmware and early kernel init rather than userspace. A ramdisk is bound,
//! so a longer `--seconds` carries the same run on to a busybox shell; twenty
//! is what the A64 benchmark uses and what makes the two comparable. The
//! numbers and the attribution are in `docs/platforms/riscv-virt.md`, "What a
//! boot profile says"; the census this program prints is what every per-block
//! row there is divided by.
//!
//! The one number to carry away is at the bottom of that census: **guest
//! instructions per block**. Every per-block cost — the dispatch loop, the
//! deferred-charge replay, the entry translation and the interrupt check
//! `admit` does per block — is divided by it, and the frontend's limit is 64.

use std::sync::Arc;
use std::time::{Duration, Instant};

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::cpu::riscv::Hart;
use rsemu::machine::{Machine, build, catalog};

/// Where OpenSBI's `fw_jump` hands control on, and where an RV64 `Image`
/// expects to be. Compiled into the firmware, so it is not ours to choose.
const PAYLOAD: u64 = 0x8020_0000;

fn main() {
    let args = Args::parse(std::env::args().skip(1));
    let Some(firmware) = fixture("RSEMU_RISCV_FIRMWARE") else {
        skip();
        return;
    };
    let kernel = fixture("RSEMU_RISCV_KERNEL").unwrap_or_default();
    let initrd = fixture("RSEMU_RISCV_INITRD").unwrap_or_default();

    println!(
        "rsemu riscv-virt Linux boot — engine={}, {} virtual ns, {} rep(s)\n",
        args.engine, args.nanos, args.reps
    );
    println!(
        "{:<10} {:>9} {:>12} {:>10} {:>18}",
        "rep", "wall (s)", "MIPS", "blocks/s", "state hash"
    );

    let mut best = Duration::MAX;
    let mut census = None;
    let mut printed = Vec::new();
    for rep in 0..args.reps {
        let (elapsed, stats, hash, out) = boot(&firmware, &kernel, &initrd, &args);
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
            printed = out;
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
    if args.console {
        println!("\nwhat the guest printed:\n");
        println!("{}", String::from_utf8_lossy(&printed));
    } else {
        println!(
            "the guest printed {} bytes to its console (--console to see them)",
            printed.len()
        );
    }
}

fn row(what: &str, n: u64, per: f64) {
    println!("{what:<42} {n:>14} {per:>10.4}");
}

/// One boot, timed.
fn boot(
    firmware: &[u8],
    kernel: &[u8],
    initrd: &[u8],
    args: &Args,
) -> (Duration, rsemu::cpu::riscv::JitStats, u64, Vec<u8>) {
    let (mut machine, hart, console) = board(firmware, kernel, initrd, args);
    let span = GlobalTime::from_nanos(args.nanos);
    let start = Instant::now();
    machine.run_for(span).expect("the machine advances");
    let elapsed = start.elapsed();
    let stats = hart.jit_stats().unwrap_or_default();
    // Off by default under a profiler: hashing a machine walks every byte of
    // guest RAM, and it would land in the profile as if it were emulation.
    // `--hash` is what makes it a *determinism* check again — the three
    // engines must agree on it, which is what
    // `tests/riscv_virt_engines.rs` gates on.
    let hash = if args.hash {
        machine.state_hash().expect("the machine hashes")
    } else {
        0
    };
    (elapsed, stats, hash, console.drain())
}

/// `riscv-virt` from the catalog, with a handle on its hart and its console.
///
/// The handle is what makes this a census rather than a stopwatch, and there
/// is no route from a `dyn Device` to a `Hart` — `core::device` keeps `Any`
/// out of the supertrait chain deliberately — so the hart is captured as it is
/// built, the way `tests/riscv_virt_engines.rs` captures one.
fn board(
    firmware: &[u8],
    kernel: &[u8],
    initrd: &[u8],
    args: &Args,
) -> (Machine, Arc<Hart>, Arc<rsemu::host::chardev::CharPort>) {
    let harts: Arc<Captured<Hart>> = Arc::new(Captured::new());
    let kept = Arc::clone(&harts);
    let mut bindings = catalog::bindings().expect("this build's bindings");
    bindings.replace("cpu.riscv", move |props| {
        let hart = Arc::new(Hart::from_props(props)?);
        kept.push(&hart);
        Ok(hart)
    });
    let entry = catalog::machine("riscv-virt").expect("this build ships it");
    let mut options = catalog::build_options()
        .expect("the catalog agrees with itself")
        .with_bindings(bindings)
        .with_media("firmware", firmware)
        .with_media("flash0", &[][..])
        .with_media("flash1", &[][..])
        .with_media("initrd", initrd)
        .with_media("disk", &[][..])
        .with_param("ram", &args.ram)
        .with_param("engine", &args.engine)
        .with_param("console", "bench.riscv.console")
        .with_param("power", "bench.riscv.power")
        .with_param("cmdline", "console=ttyS0 earlycon=sbi rdinit=/init");

    // The kernel is a *payload*: `riscv-virt` has one loader for the firmware
    // slot and one for the ramdisk, and OpenSBI's `fw_jump` hands control to a
    // fixed address that neither covers. A third loader is spliced in at the
    // end of the description, which is where it has to go — `Machine::reset`
    // runs devices in declaration order and a cold reset zeroes DRAM, so an
    // image written before that would be erased by the reset that ends
    // realize. `src/dev/riscv/tests.rs`'s `RSEMU_RISCV_PAYLOAD` does the same
    // splice for the same reason.
    let source = if kernel.is_empty() {
        String::from(entry.source)
    } else {
        options = options.with_media("payload", kernel);
        with_payload(entry.source, PAYLOAD)
    };

    options.realize.scheduler.max_ticks_per_quantum = (args.cap > 0).then_some(args.cap);
    let registry = catalog::registry().expect("a registry");
    let machine = build(entry.name, &source, &registry, &options)
        .unwrap_or_else(|e| panic!("riscv-virt does not build: {e}"));
    let console = rsemu::host::chardev::ports::open(&options.realize.hosts, "bench.riscv.console")
        .expect("the UART opened it");
    let hart = harts.take().expect("the binding captured the hart");
    (machine, hart, console)
}

/// The description with one more `riscv.loader` in it, staging the `payload`
/// media slot at `addr`.
fn with_payload(source: &str, addr: u64) -> String {
    let end = source
        .rfind('}')
        .expect("a machine description ends with a brace");
    let mut out = String::from(&source[..end]);
    out.push_str(&format!(
        "\n  object payload \"riscv.loader\" {{\n    space = mem\n    image = \"payload\"\n    \
         addr  = {addr:#x}\n  }}\n"
    ));
    out.push_str(&source[end..]);
    out
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
        "riscv_linux_boot: no firmware, so nothing was measured.\n\n\
         \x20 scripts/fetch-testdata.sh opensbi linux initramfs\n\n\
         \x20 RSEMU_RISCV_FIRMWARE=testdata/riscv/fw_jump.bin \\\n\
         \x20 RSEMU_RISCV_KERNEL=testdata/riscv/linux \\\n\
         \x20 RSEMU_RISCV_INITRD=testdata/riscv/initramfs.cpio \\\n\
         \x20     cargo bench --features machine-riscv-virt,cpu-riscv-lift,jit-x86 \\\n\
         \x20         --bench riscv_linux_boot"
    );
}

/// What the command line can change.
struct Args {
    /// How much *virtual* time to run, in seconds. Only ever set from
    /// `--seconds`; `nanos` is what the run actually uses.
    seconds: u64,
    /// The span to run, in nanoseconds. `--nanos` names it directly and
    /// `--seconds` multiplies up into it.
    ///
    /// Nanoseconds rather than seconds because a guest second of this board
    /// stopped being a usable unit: its processor used to execute ten million
    /// ticks a guest second, because `SchedulerConfig::max_ticks_per_quantum`
    /// capped a round at a rate-blind ten thousand whatever the machine file
    /// declared, and it executes the billion the file says now. The default
    /// below is the old `--seconds 20` divided by that hundred, and it
    /// reproduces that run's census to the instruction: 154 233 793 guest
    /// instructions retired, 10.80 per block.
    nanos: u64,
    /// `SchedulerConfig::max_ticks_per_quantum`, for reproducing the
    /// measurement that removed it. Zero — the default — is `None`, which is
    /// the shipping configuration: a budget bounded by the round alone.
    cap: u64,
    /// `interp`, `jit` or `jit-host`.
    engine: String,
    reps: usize,
    /// How much guest RAM. A kernel with a ramdisk wants more than the board's
    /// default 128 MiB.
    ram: String,
    /// Take a state hash at the end of each rep. Off by default: it walks
    /// every byte of guest RAM and would otherwise be a large share of a
    /// profile.
    hash: bool,
    /// Print what the guest sent to its console.
    console: bool,
}

impl Args {
    fn parse(args: impl Iterator<Item = String>) -> Args {
        let mut out = Args {
            seconds: 20,
            nanos: 200_000_000,
            cap: 0,
            engine: "jit-host".to_string(),
            reps: 1,
            ram: "512M".to_string(),
            hash: false,
            console: false,
        };
        let mut it = args.peekable();
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--seconds" => {
                    out.seconds = next(&mut it, "--seconds").parse().expect("a number");
                    out.nanos = out.seconds * 1_000_000_000;
                }
                "--nanos" => out.nanos = next(&mut it, "--nanos").parse().expect("a number"),
                "--cap" => out.cap = next(&mut it, "--cap").parse().expect("a number"),
                "--engine" => out.engine = next(&mut it, "--engine"),
                "--reps" => out.reps = next(&mut it, "--reps").parse().expect("a number"),
                "--ram" => out.ram = next(&mut it, "--ram"),
                "--hash" => out.hash = true,
                "--console" => out.console = true,
                // libtest passes these through `cargo bench`; ignore them
                // rather than fail, since `harness = false` means they are not
                // ours to interpret.
                "--bench" | "--test" => {}
                other => panic!("riscv_linux_boot: unknown argument `{other}`"),
            }
        }
        out
    }
}

fn next(it: &mut impl Iterator<Item = String>, what: &str) -> String {
    it.next()
        .unwrap_or_else(|| panic!("riscv_linux_boot: {what} wants a value"))
}
