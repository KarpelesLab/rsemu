//! `rsemu run --for <span>` runs for that span, console or no console.
//!
//! # The defect
//!
//! A machine that opened a character port goes to the binary's console loop,
//! and that loop has a rule for scripts: once stdin is at end-of-input *and*
//! the machine has gone quiet for two virtual seconds, there is nobody left to
//! type at it, so `printf … | rsemu run apple1` finishes rather than hanging.
//!
//! Applied on top of an explicit `--for`, that guess silently shortened the
//! run. `rsemu run amiga-a500 --for 12s --screenshot x.png` with stdin not a
//! terminal stopped at two virtual seconds — before Kickstart 1.3 draws
//! anything — and wrote a blank grey screen, exit zero, no warning. The picture
//! was not wrong; it was taken ten seconds early. A machine that has gone quiet
//! is not a machine that has finished: an Amiga waiting on its own timers moves
//! nothing through a serial port for an entire boot.
//!
//! # Why the binary is run rather than the rule tested
//!
//! The rule is three booleans and testing them here would assert that an `if`
//! is the `if` it is. What failed was the *combination* — a console attached,
//! stdin at EOF, a deadline given — and only a real process has all three.
//! `Command::output` gives stdin as `/dev/null`, which is exactly the shape
//! that broke: a scripted or CI run, where nobody is watching the terminal to
//! notice the run ended early.
//!
//! The Apple 1 is the machine that shows it: `mos.6821` opens the port, so this
//! goes through the console loop, RSMON prints its prompt and then waits
//! forever — quiet, which is what makes the idle rule fire.

#![cfg(feature = "cli")]

/// The virtual time a run reached, from the binary's own summary line.
///
/// `summarise` prints `ran to <n> ns of virtual time` on stdout as the last
/// thing a run does, so its absence is a failure in itself: the console loop
/// used to be able to return without ever getting there.
#[allow(dead_code)]
fn ran_to(stdout: &str) -> u64 {
    let line = stdout
        .lines()
        .find(|l| l.starts_with("ran to "))
        .unwrap_or_else(|| panic!("no summary line in:\n{stdout}"));
    line.trim_start_matches("ran to ")
        .split_whitespace()
        .next()
        .and_then(|n| n.parse().ok())
        .unwrap_or_else(|| panic!("cannot read a span out of {line:?}"))
}

/// Four seconds asked for, four seconds run — with a terminal attached and
/// nothing on stdin.
///
/// Four rather than three because the rule it guards against is two virtual
/// seconds of quiet: a run that stopped the moment RSMON finished its prompt
/// would land near 2.1 s, and the margin has to be wider than that is
/// uncertain. The run is paced to real time, so it costs about four seconds of
/// wall clock and that is the price of testing the loop a person uses.
#[cfg(feature = "machine-apple1")]
#[test]
fn a_console_session_runs_the_whole_span_it_was_given() {
    let out = std::process::Command::new(env!("CARGO_BIN_EXE_rsemu"))
        .args(["run", "apple1", "--for", "4s"])
        .output()
        .expect("the binary this test was built alongside");
    assert!(
        out.status.success(),
        "rsemu run apple1 --for 4s failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let reached = ran_to(&String::from_utf8_lossy(&out.stdout));
    assert!(
        reached >= 4_000_000_000,
        "--for 4s stopped at {reached} ns; the idle rule cut the run short"
    );
}
