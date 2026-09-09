//! `rsemu run --trace` says what the run did, and changes nothing about it.
//!
//! # The one assertion this file exists for
//!
//! **A trace must not change what the guest does or when.** The scheduler owns
//! time, and a counter that perturbs it is a defect rather than a feature — so
//! the headline here is [`tracing_does_not_change_what_the_guest_does`]: the
//! same workload, run with tracing off and then with every channel on, reaching
//! the identical state hash. That is not a proxy for the property, it is the
//! property: `Machine::state_hash` is the whole machine, and §0's regression
//! method is exactly "run deterministically for N virtual units and compare
//! this".
//!
//! It also catches something narrower and easy to get wrong. The `cpu` channel
//! reaches a core's statistics by **replacing that class's constructor**
//! (`host::trace::install`), because there is no route from a `dyn Device` to a
//! concrete one; that constructor call is therefore written twice, once in the
//! core's own `bind` and once in `host::trace`. A drift between them — a
//! different default variant, a missing `as_i8086` — builds a subtly different
//! machine, and this test is what fails when it happens.
//!
//! # And a second determinism claim, about the trace itself
//!
//! There is no timestamp and no wall-clock figure anywhere in the output, by
//! design, so **two traces of one workload are byte-identical**. That makes
//! `diff` between two of them a real answer to "what did that change do", and
//! it is asserted here rather than left as a property of the current
//! implementation.
//!
//! The apple1 is the board for both, for the reason `tests/cli_capture.rs`
//! gives: one processor, a monitor in ROM, no image to fetch, and a run that
//! costs a fraction of a second.

#![cfg(all(feature = "cli", feature = "machine-apple1", feature = "trace"))]

use std::path::PathBuf;
use std::process::Command;

/// A scratch path nobody else in this run will pick.
fn scratch(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("rsemu-trace-{}-{name}", std::process::id()))
}

/// Run the shipped binary and hand back success, stdout and stderr.
fn run(args: &[&str]) -> (bool, String, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_rsemu"))
        .args(args)
        .output()
        .expect("the binary this test was built alongside");
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

/// The `state hash 0x…` line a run prints.
fn state_hash(stdout: &str) -> String {
    stdout
        .lines()
        .find(|l| l.starts_with("state hash "))
        .unwrap_or_else(|| panic!("a deterministic run prints its state hash:\n{stdout}"))
        .to_string()
}

/// One row of a trace, by name.
fn row(text: &str, name: &str) -> u64 {
    for line in text.lines() {
        let mut fields = line.split_whitespace();
        if fields.next() == Some(name) {
            return fields
                .next()
                .and_then(|v| v.parse().ok())
                .unwrap_or_else(|| panic!("`{name}` has no number after it:\n{text}"));
        }
    }
    panic!("no row named `{name}`:\n{text}")
}

/// **The headline.** Tracing is an observation, not a change.
#[test]
fn tracing_does_not_change_what_the_guest_does() {
    let path = scratch("determinism.trace");
    let _ = std::fs::remove_file(&path);

    let (ok, plain, err) = run(&["run", "apple1", "--for", "1s", "--headless"]);
    assert!(ok, "the control run failed: {err}");

    let (ok, traced, err) = run(&[
        "run",
        "apple1",
        "--for",
        "1s",
        "--headless",
        "--trace",
        &format!("all={}", path.display()),
    ]);
    assert!(ok, "the traced run failed: {err}");

    assert_eq!(
        state_hash(&plain),
        state_hash(&traced),
        "tracing changed the machine, which is a defect and not a feature"
    );

    // And the whole summary, not only the hash: the tick totals the run prints
    // are the same numbers, so nothing about *when* anything happened moved
    // either.
    let strip = |text: &str| -> String {
        text.lines()
            .filter(|l| !l.starts_with("trace "))
            .collect::<Vec<_>>()
            .join("\n")
    };
    assert_eq!(strip(&plain), strip(&traced), "the run itself differs");

    let _ = std::fs::remove_file(&path);
}

/// Two traces of one workload are the same bytes.
///
/// Which is only true because there is no timestamp and no wall-clock figure in
/// the format, and no map without an order behind it. It is worth asserting
/// rather than assuming, because it is what makes `diff` between two traces a
/// regression test rather than a diff of two clocks.
#[test]
fn a_trace_of_a_deterministic_run_is_itself_deterministic() {
    let one = scratch("repeat-1.trace");
    let two = scratch("repeat-2.trace");
    for path in [&one, &two] {
        let _ = std::fs::remove_file(path);
        let (ok, _, err) = run(&[
            "run",
            "apple1",
            "--for",
            "1s",
            "--headless",
            "--trace",
            &format!("all={}", path.display()),
        ]);
        assert!(ok, "{err}");
    }
    let first = std::fs::read_to_string(&one).expect("the first trace");
    let second = std::fs::read_to_string(&two).expect("the second trace");
    assert_eq!(
        first, second,
        "two traces of one workload differ, so something in the format is not \
         deterministic — a timestamp, a wall clock, or a map with no order"
    );
    assert!(
        !first.to_ascii_lowercase().contains("wall")
            && !first.to_ascii_lowercase().contains("elapsed"),
        "host time does not belong in a trace:\n{first}"
    );
    let _ = std::fs::remove_file(&one);
    let _ = std::fs::remove_file(&two);
}

/// The `sched` channel counts the machine's own scheduler rounds.
///
/// The apple1 runs a millisecond a round, so a guest second is a thousand of
/// them that gave a runnable a budget — plus one *idle* round per slice of the
/// headless loop, where `run_until` declines a boundary the deadline falls
/// inside. Both numbers are what `docs/testing/long-run.md` means by "quanta
/// per guest second", and until this channel existed neither was printable.
#[test]
fn the_sched_channel_reports_the_rounds_and_the_declined_boundaries() {
    let path = scratch("sched.trace");
    let _ = std::fs::remove_file(&path);
    let (ok, _, err) = run(&[
        "run",
        "apple1",
        "--for",
        "1s",
        "--headless",
        "--trace",
        &format!("sched={}", path.display()),
    ]);
    assert!(ok, "{err}");
    let text = std::fs::read_to_string(&path).expect("the trace");

    assert_eq!(
        row(&text, "sched.span-ns"),
        1_000_000_000,
        "the rounds add up to exactly the run that was asked for:\n{text}"
    );
    let quanta = row(&text, "sched.quanta");
    let idle = row(&text, "sched.quanta.idle");
    assert_eq!(
        quanta - idle,
        1_000,
        "a one-millisecond quantum over a guest second is a thousand rounds \
         that ran something:\n{text}"
    );
    assert!(
        idle > 0,
        "the headless loop declines a boundary per slice:\n{text}"
    );
    // The histogram: a thousand rounds a millisecond long, which is bucket 20
    // (2^19..2^20 ns).
    assert_eq!(
        row(&text, "sched.span-ns.log2.20"),
        1_000,
        "the distribution and the total disagree:\n{text}"
    );
    // Only `sched` was asked for, so nothing else is in the file.
    assert!(!text.contains("\nclock."), "{text}");
    let _ = std::fs::remove_file(&path);
}

/// The `mmio` channel counts each device aperture, and only device apertures.
///
/// The apple1's monitor sits in a keyboard poll, so the PIA is read tens of
/// thousands of times a guest second and written a handful — and the RAM and
/// ROM the same loop is fetching from produce **no rows at all**, which is the
/// visible half of the cost argument: the hook is inside the `FlatTarget::Io`
/// arm, so an ordinary load or store never reaches it.
#[test]
fn the_mmio_channel_counts_device_apertures_and_nothing_else() {
    let path = scratch("mmio.trace");
    let _ = std::fs::remove_file(&path);
    let (ok, _, err) = run(&[
        "run",
        "apple1",
        "--for",
        "1s",
        "--headless",
        "--trace",
        &format!("mmio={}", path.display()),
    ]);
    assert!(ok, "{err}");
    let text = std::fs::read_to_string(&path).expect("the trace");

    let reads = row(&text, "mmio.pia.read");
    let writes = row(&text, "mmio.pia.write");
    assert!(
        reads > 1_000,
        "the monitor polls the PIA far more often than this:\n{text}"
    );
    assert_eq!(
        (reads, writes),
        (row(&text, "mmio.read"), row(&text, "mmio.write")),
        "one aperture on this board, so the totals are its rows:\n{text}"
    );
    // The ROM the same loop fetches from and the RAM it writes are regions
    // too, and neither is counted: this channel is MMIO, which is what makes
    // it free for everything else.
    assert!(!text.contains("mmio.ram"), "{text}");
    assert!(!text.contains("mmio.rom"), "{text}");
    let _ = std::fs::remove_file(&path);
}

/// Two columns, sorted, `#` for anything that is not a count.
#[test]
fn the_format_is_a_plain_two_column_table_a_script_can_read() {
    let path = scratch("format.trace");
    let _ = std::fs::remove_file(&path);
    let (ok, _, err) = run(&[
        "run",
        "apple1",
        "--for",
        "100ms",
        "--headless",
        "--trace",
        &format!("all={}", path.display()),
    ]);
    assert!(ok, "{err}");
    let text = std::fs::read_to_string(&path).expect("the trace");

    assert!(text.starts_with("# rsemu-trace 1\n"), "{text}");
    assert!(text.contains("# machine         apple1\n"), "{text}");
    assert!(text.contains("# state-hash      0x"), "{text}");

    let mut previous = String::new();
    let mut rows = 0;
    for line in text.lines() {
        if line.starts_with('#') {
            continue;
        }
        let fields: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(fields.len(), 2, "a row is a name and a number: `{line}`");
        assert!(
            fields[1].parse::<u64>().is_ok(),
            "the second field is a number: `{line}`"
        );
        assert!(
            previous.as_str() < fields[0],
            "rows are sorted, so two traces diff usefully: `{previous}` then `{}`",
            fields[0]
        );
        previous = fields[0].to_string();
        rows += 1;
    }
    assert!(rows > 4, "a trace with almost nothing in it:\n{text}");
    let _ = std::fs::remove_file(&path);
}

/// No file means stdout, the way `--capture` does it.
#[test]
fn a_trace_with_no_file_goes_to_stdout() {
    let (ok, stdout, err) = run(&[
        "run",
        "apple1",
        "--for",
        "10ms",
        "--headless",
        "--trace",
        "clock",
    ]);
    assert!(ok, "{err}");
    assert!(stdout.contains("# rsemu-trace 1\n"), "{stdout}");
    assert!(stdout.contains("clock.cpu.ticks"), "{stdout}");
}

/// Two channels, two files, and each file has only what was asked of it.
#[test]
fn channels_can_go_to_different_files() {
    let a = scratch("split-a.trace");
    let b = scratch("split-b.trace");
    let _ = std::fs::remove_file(&a);
    let _ = std::fs::remove_file(&b);
    let (ok, _, err) = run(&[
        "run",
        "apple1",
        "--for",
        "10ms",
        "--headless",
        "--trace",
        &format!("sched={}", a.display()),
        "--trace",
        &format!("clock={}", b.display()),
    ]);
    assert!(ok, "{err}");
    let sched = std::fs::read_to_string(&a).expect("the sched trace");
    let clock = std::fs::read_to_string(&b).expect("the clock trace");
    assert!(
        sched.contains("sched.quanta") && !sched.contains("clock."),
        "{sched}"
    );
    assert!(
        clock.contains("clock.cpu.ticks") && !clock.contains("sched."),
        "{clock}"
    );
    let _ = std::fs::remove_file(&a);
    let _ = std::fs::remove_file(&b);
}

/// A board whose processor keeps no translation statistics says so.
///
/// A 6502 has no lifter, so "zero blocks executed" would be a false answer to a
/// question this machine cannot be asked. The distinction matters because the
/// obvious reading of a zero is "the JIT is not working".
#[test]
fn a_board_with_no_translated_core_says_so_rather_than_printing_zeroes() {
    let (ok, stdout, err) = run(&[
        "run",
        "apple1",
        "--for",
        "10ms",
        "--headless",
        "--trace",
        "cpu",
    ]);
    assert!(ok, "{err}");
    assert!(
        stdout.contains("no processor in this machine keeps translation statistics"),
        "{stdout}"
    );
    assert!(!stdout.contains("cpu.blocks"), "{stdout}");
}

/// Every way of asking for something impossible is refused, before the run.
#[test]
fn a_flag_that_cannot_be_honoured_is_refused_rather_than_ignored() {
    // A channel this build has never heard of, with the list of the ones it
    // has.
    let (ok, _, err) = run(&["run", "apple1", "--for", "10ms", "--trace", "wires"]);
    assert!(!ok, "a misspelt channel must not run the machine");
    assert!(err.contains("--trace wires"), "{err}");
    assert!(
        err.contains("`sched`") && err.contains("`cpu`") && err.contains("`all`"),
        "{err}"
    );

    // The same channel twice: the second would silently win.
    let (ok, _, err) = run(&[
        "run", "apple1", "--for", "10ms", "--trace", "sched", "--trace", "sched=x",
    ]);
    assert!(!ok, "{err}");
    assert!(err.contains("named twice"), "{err}");

    // A value that is not `<what>[=<file>]`.
    let (ok, _, err) = run(&["run", "apple1", "--for", "10ms", "--trace", "=x"]);
    assert!(!ok, "{err}");
    assert!(err.contains("--trace wants"), "{err}");

    // And `--trace` with nothing after it.
    let (ok, _, err) = run(&["run", "apple1", "--for", "10ms", "--trace"]);
    assert!(!ok, "{err}");
    assert!(err.contains("--trace needs a value"), "{err}");
}

/// A destination that cannot be written fails the run rather than being lost.
#[test]
fn a_trace_that_cannot_be_written_fails_the_run() {
    let (ok, _, err) = run(&[
        "run",
        "apple1",
        "--for",
        "10ms",
        "--headless",
        "--trace",
        "sched=/nonexistent-directory-for-a-test/x.trace",
    ]);
    assert!(!ok, "a trace that went nowhere is not a successful run");
    assert!(err.contains("--trace"), "{err}");
}
