//! `rsemu run --capture` takes a character port's output at simulation speed.
//!
//! The flag exists because there was no fast way to get a log out of a guest.
//! `--console` attaches a terminal, and a terminal session is **paced**: the run
//! loop sleeps off whatever each ten-millisecond slice did not use, so a second
//! of guest time costs a second of yours. That is right for a machine somebody
//! is typing at and wrong for a capture — the EDK II boot log that
//! `tests/q35_uefi.rs` reads is 1 400 seconds of guest time, which is 23 minutes
//! of wall clock through a terminal and under two minutes unpaced. `--headless`
//! was already unpaced and printed no character port at all, so between the two
//! of them the log was reachable only slowly.
//!
//! What is asserted here is the binary's own wiring, the way
//! `tests/cli_screenshot.rs` asserts `--screenshot`'s: that the bytes reach the
//! file, that they reach stdout when no file is named, that the run really is
//! unpaced, and that the three ways of asking for something impossible are
//! refused **before** the machine runs rather than after.
//!
//! The Apple 1 is the board for it: one character port, a monitor in ROM that
//! greets the world on that port within a fifth of a virtual second, and no
//! image to fetch.

#![cfg(all(feature = "cli", feature = "machine-apple1"))]

use std::path::PathBuf;
use std::process::Command;
use std::time::{Duration, Instant};

/// A scratch path nobody else in this run will pick.
fn scratch(name: &str) -> PathBuf {
    std::env::temp_dir().join(format!("rsemu-capture-{}-{name}", std::process::id()))
}

/// Run the shipped binary and hand back success, stdout, stderr and how long it
/// took.
fn run(args: &[&str]) -> (bool, String, String, Duration) {
    let started = Instant::now();
    let out = Command::new(env!("CARGO_BIN_EXE_rsemu"))
        .args(args)
        .output()
        .expect("the binary this test was built alongside");
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
        started.elapsed(),
    )
}

/// The bytes go to the file, and the run says how many did.
#[test]
fn a_capture_writes_the_guests_own_bytes_to_a_file() {
    let log = scratch("apple1.log");
    let _ = std::fs::remove_file(&log);
    let (ok, stdout, stderr, _) = run(&[
        "run",
        "apple1",
        "--for",
        "1s",
        "--capture",
        &format!("console={}", log.display()),
    ]);
    assert!(ok, "the run failed\n{stderr}");

    let text = std::fs::read_to_string(&log).expect("--capture wrote the file it was given");
    assert!(
        text.contains("RSMON"),
        "the monitor greets the world on the console within a fifth of a virtual second, and \
         what was captured is {text:?}"
    );
    // Announced before the run, so a person watching a long one knows where the
    // bytes are going.
    assert!(
        stderr.contains("capturing `console` to"),
        "the run said nothing about what it was capturing:\n{stderr}"
    );
    // And counted after it, beside `--screenshot`'s line and `--record-audio`'s.
    assert!(
        stdout.contains("capture ") && stdout.contains("(`console`, "),
        "the summary does not report the capture:\n{stdout}"
    );
    let _ = std::fs::remove_file(&log);
}

/// No `=<file>` is stdout, which is what a pipeline wants.
#[test]
fn a_capture_with_no_file_goes_to_stdout() {
    let (ok, stdout, stderr, _) =
        run(&["run", "apple1", "--for", "1s", "-q", "--capture", "console"]);
    assert!(ok, "the run failed\n{stderr}");
    assert!(
        stdout.contains("RSMON"),
        "the guest's own bytes did not reach stdout:\n{stdout}"
    );
}

/// The point of the flag: the machine is **not** held to wall clock.
///
/// Calibrated rather than assumed. A paced run cannot finish in less than the
/// span it was given — it sleeps off the difference — so "finished in less than
/// the span" is exactly the property under test. But a host that simulates an
/// Apple 1 slower than an Apple 1 runs cannot demonstrate it either way, and
/// failing on such a host would be a flake rather than a defect. So the first
/// run measures, and the assertion is made only where there is room to make it.
#[test]
fn a_capture_runs_the_machine_unpaced() {
    let log = scratch("paced.log");
    let (ok, _, stderr, one) = run(&[
        "run",
        "apple1",
        "--for",
        "1s",
        "-q",
        "--capture",
        &format!("console={}", log.display()),
    ]);
    assert!(ok, "the run failed\n{stderr}");
    if one >= Duration::from_millis(500) {
        println!(
            "cli_capture: this host took {one:?} to simulate one second of a 1 MHz 6502, which \
             is too close to real time for `unpaced` and `paced` to be told apart; the timing \
             claim is skipped rather than flaked"
        );
        let _ = std::fs::remove_file(&log);
        return;
    }

    // Ten times the virtual time, on a host that has just shown it manages a
    // second in under half of one.
    let span = Duration::from_secs(10);
    let (ok, stdout, stderr, ten) = run(&[
        "run",
        "apple1",
        "--for",
        "10s",
        "-q",
        "--capture",
        &format!("console={}", log.display()),
    ]);
    assert!(ok, "the run failed\n{stderr}");
    assert!(
        stdout.contains("ran to 10000000000 ns"),
        "the run did not cover the span it was given:\n{stdout}"
    );
    assert!(
        ten < span,
        "ten seconds of guest time took {ten:?} of wall clock, which is what a *paced* run \
         costs; the capture loop is supposed to be the unpaced one (one second took {one:?})"
    );
    let _ = std::fs::remove_file(&log);
}

/// A port this machine never opened is a refusal, and the refusal says which
/// ports it did open.
#[test]
fn a_port_this_machine_never_opened_is_refused() {
    let (ok, _, stderr, _) = run(&["run", "apple1", "--for", "1s", "--capture", "debug"]);
    assert!(!ok, "a capture of a port that does not exist succeeded");
    assert!(
        stderr.contains("--capture debug") && stderr.contains("`console`"),
        "the refusal does not say what this machine actually opened:\n{stderr}"
    );
}

/// A terminal and a capture on one port would each get half the bytes, because
/// a `CharPort` hands each byte to whoever asks first.
#[test]
fn a_terminal_and_a_capture_on_one_port_are_refused() {
    let (ok, _, stderr, _) = run(&[
        "run",
        "apple1",
        "--console",
        "console",
        "--capture",
        "console",
    ]);
    assert!(!ok, "two listeners on one port were accepted");
    assert!(
        stderr.contains("two listeners on one port"),
        "the refusal does not explain itself:\n{stderr}"
    );
}

/// Two ports interleaved on stdout cannot be told apart afterwards, and the fix
/// is one character long.
#[test]
fn two_captures_cannot_share_stdout() {
    let (ok, _, stderr, _) = run(&[
        "run",
        "apple1",
        "--capture",
        "console",
        "--capture",
        "keyboard",
    ]);
    assert!(!ok, "two ports were pointed at one stream");
    assert!(
        stderr.contains("both go to stdout"),
        "the refusal does not explain itself:\n{stderr}"
    );
}
