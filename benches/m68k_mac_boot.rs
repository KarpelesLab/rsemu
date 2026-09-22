//! A real Macintosh ROM boot on a 68000, timed — the workload the m68k
//! translated engine exists to accelerate.
//!
//! `benches/a64_linux_boot.rs` is the shape this follows: a **census** rather
//! than a mechanism ladder, because the question is not what one mechanism is
//! worth on a synthetic loop but where twelve virtual seconds of Apple's own
//! code actually go. `tests/m68k_lift_rate.rs` measures the same two boards
//! for *coverage* and asserts the state hashes agree; this one measures wall
//! clock, and prints the hash so a rep that drifted is visible rather than
//! merely fast.
//!
//! ```text
//!   RSEMU_MAC_ROM_DIR=/path/to/roms \
//!       cargo bench --no-default-features \
//!           --features std,machine-mac-plus,machine-mac-classic,cpu-m68k-lift,jit \
//!           --bench m68k_mac_boot -- --board mac-plus --engine jit-host --reps 6
//! ```
//!
//! With no ROM directory it **skips loudly**: no byte of a Macintosh ROM is in
//! this repository and each one is read in place out of the user's own
//! directory (`CLAUDE.md`, *Provenance*). Nothing here disassembles a ROM —
//! what is printed is a wall clock and this engine's own counters.
//!
//! `m68k-mini` is in the board list and is **synthetic**: a hand-written
//! caller and subroutine, the same firmware `tests/m68k_lift_rate.rs` uses. It
//! is there so the instrument runs on a machine with no media, and its number
//! is evidence about that program rather than about 68000 code.

use std::sync::Arc;
use std::time::{Duration, Instant};

use rsemu::core::Captured;
use rsemu::core::clock::GlobalTime;
use rsemu::cpu::m68k::{Engine, M68k};
use rsemu::machine::{Machine, build, catalog};

fn main() {
    let args = Args::parse(std::env::args().skip(1));
    let engine = match args.engine.as_str() {
        "interp" => Engine::Interp,
        "jit" => Engine::Jit,
        "jit-host" => Engine::JitHost,
        other => panic!("m68k_mac_boot: unknown engine `{other}`"),
    };

    let Some(media) = media_for(&args.board) else {
        return;
    };

    println!(
        "rsemu {} boot — engine={}, {} virtual ms, {} rep(s)\n",
        args.board, args.engine, args.millis, args.reps
    );
    println!(
        "{:<6} {:>10} {:>12} {:>18}",
        "rep", "wall (ms)", "MIPS", "state hash"
    );

    let mut best = Duration::MAX;
    let mut census = None;
    for rep in 0..args.reps {
        let (elapsed, stats, hash) = run(&args, engine, &media);
        let insns = stats.map_or(0, |s| s.retired + s.interpreted + s.faults);
        let mips = insns as f64 / elapsed.as_secs_f64() / 1e6;
        println!(
            "{:<6} {:>10.1} {:>12.2} {:>#18x}",
            rep,
            elapsed.as_secs_f64() * 1e3,
            mips,
            hash
        );
        if elapsed < best {
            best = elapsed;
            census = stats;
        }
    }
    println!(
        "\nbest of {}: {:.1} ms",
        args.reps,
        best.as_secs_f64() * 1e3
    );

    let Some(s) = census else { return };
    let retired = s.retired.max(1) as f64;
    let executed = s.executed.max(1) as f64;
    println!("\nthe census, from the fastest rep\n");
    row("block executions", s.executed);
    row("  compiled to host code", s.compiled);
    row("  reached by a patched exit", s.chained);
    row("  entered by a direct link", s.linked);
    row("distinct blocks lifted", s.lifted);
    row(
        "translations a block's store killed",
        s.invalidated_in_block,
    );
    row(
        "translations an interpreted store killed",
        s.invalidated_interpreted,
    );
    row("guest insns retired in blocks", s.retired);
    row("guest insns interpreted", s.interpreted);
    row("runs that faulted", s.faults);
    row("runs that left on the tick allowance", s.spent);
    row("runs that stopped at a decline", s.declined);
    println!("\nguest instructions per block: {:.2}", retired / executed);
    println!(
        "the fraction retired inside a block: {:.2}%",
        100.0 * retired / (retired + s.interpreted as f64 + s.faults as f64)
    );
}

fn row(what: &str, n: u64) {
    println!("{what:<40} {n:>14}");
}

/// One run of the board, timed.
fn run(
    args: &Args,
    engine: Engine,
    media: &[(&'static str, Vec<u8>)],
) -> (Duration, Option<rsemu::cpu::m68k::JitStats>, u64) {
    let (mut machine, cpu) = board(&args.board, engine, media);
    let span = GlobalTime::from_nanos(args.millis * 1_000_000);
    let start = Instant::now();
    machine.run_for(span).expect("the machine advances");
    let elapsed = start.elapsed();
    let stats = cpu.jit_stats();
    let hash = if args.hash {
        machine.state_hash().expect("the machine hashes")
    } else {
        0
    };
    (elapsed, stats, hash)
}

/// The named board, with a handle on its 68000.
///
/// The engine is set on the **core** rather than in the machine file: both
/// Macintosh boards say `engine = "interp"` and belong to somebody else.
/// `M68k::with_engine` is the same seam `engine = "jit"` reaches, so this
/// measures each board as shipped with one property moved — the pattern
/// `tests/m68k_lift_rate.rs` established.
fn board(name: &str, engine: Engine, media: &[(&'static str, Vec<u8>)]) -> (Machine, Arc<M68k>) {
    let cores: Arc<Captured<M68k>> = Arc::new(Captured::new());
    let kept = Arc::clone(&cores);
    let mut options = catalog::build_options().expect("the catalog agrees with itself");
    options.bindings.replace("cpu.m68k", move |props| {
        let cpu = Arc::new(M68k::from_props(props)?.with_engine(engine));
        kept.push(&cpu);
        Ok(cpu)
    });
    if name.starts_with("mac-") {
        rsemu::host::display::mac::capture::install(&mut options).expect("a capture table");
    }
    for (slot, bytes) in media {
        options.realize.media.insert(*slot, bytes.clone());
    }
    let entry = catalog::machine(name).unwrap_or_else(|| panic!("this build ships {name}"));
    let registry = catalog::registry().expect("a registry");
    let machine = build(entry.name, entry.source, &registry, &options)
        .unwrap_or_else(|e| panic!("{name} does not build: {e}"));
    let cpu = cores.last().expect("the binding captured the processor");
    (machine, cpu)
}

/// The media each board needs, or `None` — having said why — when a ROM this
/// repository does not ship is missing.
fn media_for(board: &str) -> Option<Vec<(&'static str, Vec<u8>)>> {
    match board {
        "m68k-mini" => Some(vec![("firmware", mini_firmware())]),
        "mac-plus" => Some(vec![
            ("macrom", rom("Mac-Plus.ROM", 128 * 1024)?),
            ("floppy", Vec::new()),
            ("hd0", Vec::new()),
        ]),
        "mac-classic" => Some(vec![
            ("macrom", rom("Classic.ROM", 512 * 1024)?),
            ("floppy", Vec::new()),
        ]),
        other => panic!("m68k_mac_boot: unknown board `{other}`"),
    }
}

/// Read `file` out of `RSEMU_MAC_ROM_DIR`, trimmed to `len`.
fn rom(file: &str, len: usize) -> Option<Vec<u8>> {
    let Ok(dir) = std::env::var("RSEMU_MAC_ROM_DIR") else {
        println!(
            "m68k_mac_boot: set RSEMU_MAC_ROM_DIR to a directory holding {file} to measure a \
             real Macintosh ROM boot; nothing was measured.\n\n\
             \x20 RSEMU_MAC_ROM_DIR=… cargo bench --bench m68k_mac_boot -- --engine jit-host"
        );
        return None;
    };
    let path = std::path::Path::new(&dir).join(file);
    let Ok(bytes) = std::fs::read(&path) else {
        println!(
            "m68k_mac_boot: {} is not there; nothing measured",
            path.display()
        );
        return None;
    };
    if bytes.len() < len {
        println!(
            "m68k_mac_boot: {} is {} bytes and the socket takes {len}; nothing measured",
            path.display(),
            bytes.len()
        );
        return None;
    }
    Some(bytes[..len].to_vec())
}

/// The `m68k-mini` firmware, byte for byte the one `tests/m68k_lift_rate.rs`
/// assembles: a caller that calls a subroutine in a `DBF` loop.
fn mini_firmware() -> Vec<u8> {
    let mut image = vec![0u8; 0x0500];
    image[0..4].copy_from_slice(&0x0020_0000u32.to_be_bytes());
    image[4..8].copy_from_slice(&0x0000_0400u32.to_be_bytes());
    let caller: &[u16] = &[
        0x2e7c, 0x0020, 0x0000, 0x227c, 0x0010, 0x1000, 0x303c, 0x7fff, 0x4eb9, 0x0000, 0x0420,
        0x51c8, 0xfff8, 0x60fe,
    ];
    let callee: &[u16] = &[
        0x4e56, 0xfffc, 0x2f02, 0x2411, 0xd481, 0xe58a, 0x2282, 0x3221, 0xd269, 0x0002, 0x0c41,
        0x1234, 0x6702, 0x5341, 0x241f, 0x4e5e, 0x4e75,
    ];
    for (base, code) in [(0x0400usize, caller), (0x0420, callee)] {
        for (i, word) in code.iter().enumerate() {
            let at = base + 2 * i;
            image[at..at + 2].copy_from_slice(&word.to_be_bytes());
        }
    }
    image
}

/// What the command line can change.
struct Args {
    board: String,
    engine: String,
    /// How much *virtual* time to run, in milliseconds. Twelve thousand is
    /// where `tests/m68k_lift_rate.rs` and the Macintosh goldens stop: past
    /// reset, the memory test, the chime and the device probes, into the
    /// insert-disk loop.
    ///
    /// Zero means "whatever the board is good for", which is that twelve
    /// seconds on a Macintosh and **eight milliseconds** on `m68k-mini`: its
    /// firmware walks an address register down through RAM two bytes an
    /// iteration and runs off the bottom of it in about a second of guest
    /// time, so a longer run measures a double fault rather than a program.
    millis: u64,
    reps: usize,
    /// Take a state hash at the end of each rep. **On** by default here,
    /// unlike the 64-bit boards: a compact Macintosh has at most four
    /// megabytes of RAM, so hashing one is not a measurable share of the run,
    /// and a rep that drifted is exactly what a timing harness must not hide.
    hash: bool,
}

impl Args {
    fn parse(args: impl Iterator<Item = String>) -> Args {
        let mut out = Args {
            board: "mac-plus".to_string(),
            engine: "jit-host".to_string(),
            millis: 0,
            reps: 6,
            hash: true,
        };
        let mut it = args.peekable();
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--board" => out.board = next(&mut it, "--board"),
                "--engine" => out.engine = next(&mut it, "--engine"),
                "--millis" => out.millis = next(&mut it, "--millis").parse().expect("a number"),
                "--seconds" => {
                    out.millis =
                        next(&mut it, "--seconds").parse::<u64>().expect("a number") * 1_000;
                }
                "--reps" => out.reps = next(&mut it, "--reps").parse().expect("a number"),
                "--no-hash" => out.hash = false,
                // libtest passes these through `cargo bench`; ignore them
                // rather than fail, since `harness = false` means they are not
                // ours to interpret.
                "--bench" | "--test" => {}
                other => panic!("m68k_mac_boot: unknown argument `{other}`"),
            }
        }
        if out.millis == 0 {
            out.millis = if out.board == "m68k-mini" { 8 } else { 12_000 };
        }
        out
    }
}

fn next(it: &mut impl Iterator<Item = String>, what: &str) -> String {
    it.next().unwrap_or_else(|| panic!("{what} needs a value"))
}
