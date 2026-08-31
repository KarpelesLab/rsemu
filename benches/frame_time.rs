//! Frame-time benchmark: how long does the host take to emulate one frame?
//!
//! `ROADMAP.md` §13's phase-3 gate is not a throughput number, it is a *latency
//! distribution*:
//!
//! > three named commercial titles hold 60 emulated fps with 99th-percentile
//! > frame times under 16.6 ms on the reference host
//!
//! A mean would hide the thing that decides whether a game feels right — one
//! frame in a hundred taking 40 ms is visible and a 3 ms average does not
//! mention it — so this reports the whole distribution: minimum, median, p90,
//! p99, and the worst sample seen.
//!
//! # What it cannot measure
//!
//! **The commercial-title half of that gate is unmet for licensing reasons, not
//! technical ones.** `CLAUDE.md` forbids committing or fetching a commercial
//! ROM, so the committed workloads are all synthetic (`tests/workload`). Point
//! `RSEMU_BENCH_NES_ROM` at your own cartridge and the NES row measures that
//! instead — which is the only lawful way for us to run the gate as written,
//! and it is a measurement someone has to reproduce rather than one we can
//! publish from CI.
//!
//! # Running it
//!
//! ```text
//! cargo bench --all-features
//! cargo bench --all-features -- --frames 3000        # longer, tighter tail
//! cargo bench --all-features -- --only nes-ntsc
//! cargo bench --all-features -- --smoke              # CI: does it still work
//! RSEMU_BENCH_NES_ROM=~/roms/mine.nes cargo bench --all-features -- --only nes-ntsc
//! ```
//!
//! **`cargo bench` only.** The bench profile is `-O` with debug assertions off;
//! the same code under `cargo test` is three to ten times slower and the number
//! means nothing. There is no `criterion` here and there will not be: the
//! dependency policy is absolute and a hand-rolled `Instant` loop with an
//! explicit percentile is all this measurement needs.
//!
//! # Why it asserts a hash
//!
//! Every workload's warm-up phase ends exactly where `tests/frame_hash.rs`'s
//! golden checkpoint sits, and this checks it. A benchmark whose guest quietly
//! stopped doing the work looks like an enormous speedup, and that is the most
//! expensive mistake available in this file.

#[path = "../tests/workload/mod.rs"]
mod workload;

use std::time::Instant;

use workload::Workload;

fn main() {
    let args = Args::parse(std::env::args().skip(1));
    let workloads = workload::all();
    if workloads.is_empty() {
        println!("no machine features are enabled; there is nothing to measure");
        println!("try: cargo bench --all-features");
        return;
    }

    // A frame time is meaningless without the machine it was measured on, and
    // "the reference host" is not defined anywhere yet, so every run says what
    // it ran on. Best-effort and Linux-shaped: this is a benchmark's banner,
    // not an API.
    let frames = if args.smoke { 30 } else { args.frames };
    println!("rsemu frame-time benchmark");
    println!("  host      {}", host());
    println!("  profile   {}", profile());
    println!("  {frames} timed frame(s) per workload, after each one's own warm-up");
    if let Some(rom) = workload::nes_rom_override() {
        println!("  {}={rom}", workload::NES_ROM_ENV);
    }
    println!();

    let mut ran = 0usize;
    for w in &workloads {
        if let Some(only) = args.only.as_deref()
            && only != w.name
        {
            continue;
        }
        ran += 1;
        report(&measure(w, frames));
    }

    if ran == 0 {
        println!(
            "`--only {}` matched no workload",
            args.only.unwrap_or_default()
        );
        std::process::exit(1);
    }
    println!(
        "ROADMAP.md §13's phase-3 gate names three commercial titles. None is here,\n\
         and none can be: they cannot be committed or fetched (CLAUDE.md). Every row\n\
         above is a ROM this repository generated, so a `pass` means the machine can\n\
         hold the frame rate on a workload of our own choosing — necessary, and a long\n\
         way from sufficient. Set RSEMU_BENCH_NES_ROM to a cartridge you own to run the\n\
         gate as written; that measurement belongs to whoever ran it, not to CI."
    );
}

// ---------------------------------------------------------------------------
// measurement
// ---------------------------------------------------------------------------

/// What one workload's run produced.
struct Measured {
    name: &'static str,
    what: &'static str,
    /// Wall-clock nanoseconds per emulated frame, one sample per frame.
    samples: Vec<u64>,
    /// Virtual nanoseconds one frame covers, so ×real-time is derivable.
    frame_period_ns: u64,
    /// Frames the display completed, or `None` for a machine with no display.
    rendered: Option<u64>,
    /// The state hash at the end of the warm-up, and whether it matched.
    warm_hash: u64,
    golden: Option<u64>,
}

/// Run a workload and time every frame after its warm-up.
fn measure(w: &Workload, frames: u32) -> Measured {
    let mut booted = w.boot();

    // The warm-up is the regression's own run length, so its end is a point the
    // golden file has an expected hash for.
    booted.step_many(w.frames);
    let warm_hash = booted.machine.state_hash().expect("a state hash");
    // Not when the run is somebody's own cartridge: the golden file describes
    // the generated ROM, and comparing the two would fail on purpose.
    let golden = (w.name != "nes-ntsc" || workload::nes_rom_override().is_none())
        .then(workload::goldens)
        .and_then(|g| {
            g.get(w.name)
                .and_then(|rows| rows.iter().find(|row| row.frame == w.frames))
                .map(|row| row.state)
        });
    assert!(
        golden.is_none_or(|want| want == warm_hash),
        "`{}` did not reach its recorded state after {} warm-up frames \
         (want {:#018x}, got {warm_hash:#018x}). The guest is not doing the work \
         this benchmark claims to measure; fix that before reading any number below.",
        w.name,
        w.frames,
        golden.unwrap_or_default(),
    );

    let before_frames = booted
        .capture
        .as_ref()
        .map(workload::Capture::frame_counter);
    let mut samples = Vec::with_capacity(frames as usize);
    for _ in 0..frames {
        let started = Instant::now();
        booted.step();
        // `as u64` rather than `as_nanos() as u64`: a frame that took more than
        // 584 years is not a measurement problem.
        samples.push(started.elapsed().as_nanos().min(u128::from(u64::MAX)) as u64);
    }
    let rendered = booted
        .capture
        .as_ref()
        .map(workload::Capture::frame_counter)
        .zip(before_frames)
        .map(|(after, before)| after - before);

    Measured {
        name: w.name,
        what: w.what,
        samples,
        frame_period_ns: booted.frame_period_ns(),
        rendered,
        warm_hash,
        golden,
    }
}

/// Print one workload's row.
fn report(m: &Measured) {
    let mut sorted = m.samples.clone();
    sorted.sort_unstable();
    let n = sorted.len();
    assert!(n > 0, "`{}` produced no samples", m.name);

    let total: u128 = sorted.iter().map(|s| u128::from(*s)).sum();
    let mean = (total / n as u128) as u64;
    let p = |q: f64| sorted[(((n - 1) as f64) * q).round() as usize];

    println!("{} — {}", m.name, m.what);
    println!(
        "  frame period {:.3} ms of virtual time ({:.2} Hz)",
        m.frame_period_ns as f64 / 1e6,
        1e9 / m.frame_period_ns as f64
    );
    println!(
        "  min {:>8.3}  p50 {:>8.3}  mean {:>8.3}  p90 {:>8.3}  p99 {:>8.3}  max {:>8.3}  (ms)",
        ms(sorted[0]),
        ms(p(0.50)),
        ms(mean),
        ms(p(0.90)),
        ms(p(0.99)),
        ms(sorted[n - 1]),
    );

    // Emulated fps: how many of this machine's frames one wall-clock second
    // buys. `×real-time` is the same number against the machine's own rate, and
    // is what says whether a guest could run at speed at all.
    let mean_fps = 1e9 / mean as f64;
    let p99_fps = 1e9 / p(0.99) as f64;
    let realtime = m.frame_period_ns as f64 / mean as f64;
    println!("  {mean_fps:.1} fps at the mean, {p99_fps:.1} fps at p99, {realtime:.1}x real time");

    // The gate's *shape*, applied to this row. Deliberately not called "the
    // gate met": ROADMAP.md §13 asks for three commercial titles and this is a
    // ROM we generated, so passing here is necessary and nowhere near
    // sufficient. 16.6 ms is the number the roadmap gives — one NTSC frame to
    // one decimal place.
    let p99_ms = ms(p(0.99));
    println!(
        "  gate shape (p99 < 16.6 ms and >= 60 fps): {}",
        if p99_ms < 16.6 && mean_fps >= 60.0 {
            format!("pass, {:.2}x headroom at p99", 16.6 / p99_ms)
        } else {
            format!("FAIL, p99 is {p99_ms:.3} ms")
        }
    );

    match m.rendered {
        Some(rendered) => println!(
            "  {rendered} frames actually reached the scanout over {} measured spans",
            n
        ),
        None => println!("  no display device in this build; timing only"),
    }
    println!(
        "  state hash after warm-up {:#018x} ({})",
        m.warm_hash,
        match m.golden {
            Some(_) => "matches the golden file",
            None => "no golden recorded for this workload",
        }
    );
    println!();
}

fn ms(nanos: u64) -> f64 {
    nanos as f64 / 1e6
}

/// Which profile this binary was built with, so a number is never read out of
/// context.
///
/// `debug_assertions` is the honest proxy: `cargo bench` and `--release` clear
/// it, `cargo test` does not.
/// The CPU this ran on, so a recorded number stays interpretable.
///
/// `/proc/cpuinfo` where there is one and the target triple otherwise. A
/// benchmark banner, not a portable API — nothing depends on the string.
fn host() -> String {
    let model = std::fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|text| {
            text.lines()
                .find(|l| l.starts_with("model name"))
                .and_then(|l| l.split_once(':'))
                .map(|(_, v)| v.trim().to_owned())
        })
        .unwrap_or_else(|| String::from("unknown CPU"));
    format!(
        "{model} ({} {})",
        std::env::consts::OS,
        std::env::consts::ARCH
    )
}

fn profile() -> &'static str {
    if cfg!(debug_assertions) {
        "DEBUG — every number below is meaningless; use `cargo bench`"
    } else {
        "optimised (debug assertions off)"
    }
}

// ---------------------------------------------------------------------------
// arguments
// ---------------------------------------------------------------------------

/// The benchmark's own command line, hand-parsed.
struct Args {
    frames: u32,
    only: Option<String>,
    smoke: bool,
}

impl Args {
    fn parse<I: Iterator<Item = String>>(args: I) -> Args {
        // 600 frames is ten virtual seconds at 60 Hz: enough samples that the
        // 99th percentile is the sixth-worst rather than an outlier, and few
        // enough that the whole table runs in seconds.
        let mut out = Args {
            frames: 600,
            only: None,
            smoke: false,
        };
        let mut args = args.peekable();
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--frames" => {
                    out.frames = args
                        .next()
                        .and_then(|v| v.parse().ok())
                        .expect("--frames takes a count");
                }
                "--only" => out.only = args.next(),
                // Prove the harness still runs without spending CI's time on a
                // number nobody will read off a shared runner.
                "--smoke" => out.smoke = true,
                // libtest's flags, which `cargo bench` passes through even to a
                // `harness = false` target. Ignored rather than rejected.
                "--bench" | "--nocapture" | "--test" => {}
                other => panic!("unknown argument `{other}`"),
            }
        }
        out
    }
}
