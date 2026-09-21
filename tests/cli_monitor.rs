//! The monitor console, through the shipped binary, with the script arriving
//! on a pipe.
//!
//! `src/host/monitor/tests.rs` drives the command engine directly, which is
//! most of the coverage. This file exists for the three things that only the
//! binary can answer, and each of them has been a defect somewhere in this tree
//! before:
//!
//! * **The flag is refused rather than ignored.** `--mon` in a build without
//!   the feature, and `--mon` beside `--gdb`, both have to say which thing they
//!   cannot do. A silently ranked pair would leave somebody who asked for two
//!   owners of the machine's clock with no way to tell which one they got.
//!
//! * **A session reaches the same state a headless run does.** This is the
//!   determinism claim the console rests on, and it can only be made by running
//!   the two side by side: `rsemu run apple1 --for 2s --headless` and a session
//!   that types `run 2s`. The console attaches a recorder, a timeline that
//!   takes keyframes, the trace counters and the debug-halt level, and *none*
//!   of those may move the number.
//!
//! * **Nothing a session looks at changes the machine.** The whole of
//!   `MemAttrs::debug` in one assertion: read every device's registers through
//!   the console and the state hash is the number it was before.
//!
//! No TTY anywhere. `Command` with a piped stdin is what a session looks like
//! to the program, and dropping the pipe is what end-of-input looks like.

#![cfg(all(feature = "cli", feature = "monitor", feature = "machine-apple1"))]

use std::io::Write;
use std::process::{Command, Stdio};

/// Run the shipped binary with `script` on its stdin.
///
/// Dropping the handle after the write is what gives the child end of input,
/// which is the session's `quit` when a script forgets to say so.
fn session(args: &[&str], script: &str) -> (bool, String, String) {
    let mut child = Command::new(env!("CARGO_BIN_EXE_rsemu"))
        .args(args)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("the binary this test was built alongside");
    child
        .stdin
        .take()
        .expect("a piped stdin")
        .write_all(script.as_bytes())
        .expect("the script is written");
    let out = child.wait_with_output().expect("the session ends");
    (
        out.status.success(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

/// Run the binary with nothing on its stdin.
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

/// The first `0x…` on a line mentioning a hash.
fn hash_in(text: &str) -> String {
    text.lines()
        .find_map(|line| line.split_whitespace().find(|w| w.starts_with("0x")))
        .unwrap_or_else(|| panic!("no hash in:\n{text}"))
        .to_string()
}

/// The last virtual instant a session printed.
fn last_instant(text: &str) -> u64 {
    let line = text
        .lines()
        .rfind(|l| l.trim_end().ends_with(" ns"))
        .unwrap_or_else(|| panic!("no instant in:\n{text}"));
    line.trim_end()
        .trim_end_matches(" ns")
        .split_whitespace()
        .next_back()
        .and_then(|n| n.parse().ok())
        .unwrap_or_else(|| panic!("cannot read an instant out of {line:?}"))
}

#[test]
fn a_piped_script_is_a_session() {
    let (ok, stdout, stderr) = session(
        &["monitor", "apple1"],
        "# a script may explain itself\nstatus\ndevices\nquit\n",
    );
    assert!(ok, "the session failed\n{stderr}");
    assert!(
        stderr.contains("monitor attached to \"apple1\""),
        "no banner:\n{stderr}"
    );
    assert!(stdout.contains("(rsemu)"), "no prompt in:\n{stdout}");
    assert!(
        stdout.contains("stopped at 0 ns"),
        "the machine did not start stopped:\n{stdout}"
    );
    assert!(
        stdout.contains("apple1.pia"),
        "`devices` did not list the board's devices:\n{stdout}"
    );
}

#[test]
fn end_of_input_ends_the_session_without_a_quit() {
    // A script that forgot to say `quit` must not hang on a closed pipe.
    let (ok, stdout, stderr) = session(&["monitor", "apple1", "-q"], "time\n");
    assert!(ok, "the session failed\n{stderr}");
    assert!(stdout.contains(" ns"), "`time` said nothing:\n{stdout}");
}

#[test]
fn the_subcommand_and_the_flag_are_one_session() {
    let script = "run 100ms\nhash\nquit\n";
    let (ok_a, from_subcommand, _) = session(&["monitor", "apple1", "-q"], script);
    let (ok_b, from_flag, _) = session(&["run", "apple1", "--mon", "-q"], script);
    assert!(ok_a && ok_b, "one of the two sessions failed");
    assert_eq!(
        hash_in(&from_subcommand),
        hash_in(&from_flag),
        "`rsemu monitor` and `rsemu run --mon` are not the same session"
    );
}

#[test]
fn a_session_reaches_the_state_a_headless_run_reaches() {
    // The claim the console rests on. `Machine::run_until` is additive, so a
    // span typed at a prompt, the same span in pieces, and the same span run
    // headless all land on one state — and neither the recorder the session
    // attaches, nor the timeline taking keyframes, nor the counters, nor the
    // debug-halt level broadcast to every device may move it.
    let (ok, headless, stderr) = run(&["run", "apple1", "--for", "2s", "--headless", "-q"]);
    assert!(ok, "the headless run failed\n{stderr}");
    let expected = hash_in(&headless);

    let (ok, whole, stderr) = session(&["monitor", "apple1", "-q"], "run 2s\nhash\nquit\n");
    assert!(ok, "the session failed\n{stderr}");
    assert_eq!(
        hash_in(&whole),
        expected,
        "a monitor session and a headless run of the same span disagree"
    );

    let (ok, pieces, stderr) = session(
        &["monitor", "apple1", "-q"],
        "run 500ms\nrun 500ms\nrun 500ms\nrun 500ms\nhash\nquit\n",
    );
    assert!(ok, "the session failed\n{stderr}");
    assert_eq!(
        hash_in(&pieces),
        expected,
        "four quarters of a span and the whole of it disagree"
    );
}

#[test]
fn looking_at_a_machine_does_not_change_it() {
    // `MemAttrs::debug` in one assertion, end to end. Every read below is one
    // that would otherwise have a side effect somewhere in the tree: the
    // Apple 1's PIA clears its key-waiting flag when `$D010` is read, and the
    // whole point of the attribute is that the console's read does not.
    let inspect = "\
run 300ms
hash
x d010 4
x d011 4
x d012 4
x d013 4
xp d010 16
device pia
device cpu
regs
cpus
clocks
wires
sched
map
spaces
devices
hash
quit
";
    let (ok, stdout, stderr) = session(&["monitor", "apple1", "-q"], inspect);
    assert!(ok, "the session failed\n{stderr}");
    let hashes: Vec<&str> = stdout
        .lines()
        .filter(|l| l.trim().starts_with("0x") && l.trim().len() == 18)
        .collect();
    assert_eq!(
        hashes.len(),
        2,
        "expected the two `hash` lines and got {}:\n{stdout}",
        hashes.len()
    );
    assert_eq!(
        hashes[0], hashes[1],
        "sixteen commands' worth of inspection moved the machine"
    );
}

#[test]
fn a_write_through_the_console_does_change_it() {
    // The other half: if nothing the console did could ever change the state
    // hash, the test above would be about a number that does not move.
    let (ok, stdout, stderr) = session(
        &["monitor", "apple1", "-q"],
        "hash\nwrite 0x20 de ad be ef\nx 20 4\nhash\nquit\n",
    );
    assert!(ok, "the session failed\n{stderr}");
    assert!(
        stdout.contains("de ad be ef"),
        "the write did not land:\n{stdout}"
    );
    let hashes: Vec<&str> = stdout
        .lines()
        .filter(|l| l.trim().starts_with("0x") && l.trim().len() == 18)
        .collect();
    assert_eq!(hashes.len(), 2, "expected two hashes in:\n{stdout}");
    assert_ne!(
        hashes[0], hashes[1],
        "patching four bytes of guest RAM did not change the state hash"
    );
}

#[test]
fn a_session_can_go_back() {
    let (ok, stdout, stderr) = session(
        &["monitor", "apple1", "-q"],
        "run 2s\ntimeline\nrewind 1s\ntime\nquit\n",
    );
    assert!(ok, "the session failed\n{stderr}");
    assert!(
        stdout.contains("keyframes"),
        "the session kept no timeline:\n{stdout}"
    );
    assert!(
        stdout.contains("rewound to"),
        "`rewind` did not report where it landed:\n{stdout}"
    );
    // A machine at 2s that rewound a second is at about one, and certainly not
    // still at two.
    let landed = last_instant(&stdout);
    assert!(
        landed < 2_000_000_000,
        "the machine did not move back: still at {landed} ns"
    );
}

#[test]
fn the_console_and_a_debugger_cannot_both_own_the_clock() {
    let (ok, _, stderr) = run(&["run", "apple1", "--mon", "--gdb", ":0"]);
    assert!(!ok, "--mon and --gdb together were accepted");
    assert!(
        stderr.contains("--mon") && stderr.contains("--gdb"),
        "the refusal does not name both:\n{stderr}"
    );
}

#[test]
fn an_unknown_command_is_refused_and_the_session_carries_on() {
    let (ok, stdout, stderr) = session(&["monitor", "apple1", "-q"], "chocolate\nstatus\nquit\n");
    assert!(ok, "one bad command ended the session\n{stderr}");
    assert!(
        stdout.contains("unknown command `chocolate`"),
        "no refusal in:\n{stdout}"
    );
    assert!(
        stdout.contains("machine    \"apple1\""),
        "the session did not carry on:\n{stdout}"
    );
}

#[test]
fn for_bounds_a_session_that_never_says_how_far_to_run() {
    // `cont` with no span is the interactive spelling, and a piped session has
    // nobody to press Ctrl-C. `--for` is what bounds it, and the banner says so
    // before anybody types anything.
    let (ok, stdout, stderr) = session(
        &["monitor", "apple1", "--for", "200ms"],
        "cont\ntime\nquit\n",
    );
    assert!(ok, "the session failed\n{stderr}");
    assert!(
        stderr.contains("--for bounds this session"),
        "the banner did not mention the bound:\n{stderr}"
    );
    let landed = last_instant(&stdout);
    assert!(
        (190_000_000..=200_000_000).contains(&landed),
        "an unbounded `cont` with a 200ms bound stopped at {landed} ns"
    );
}

#[test]
fn the_usage_names_both_spellings_and_keeps_them_apart() {
    // `--monitor <name>` is a guest ROM image and `--mon` is this console. Two
    // meanings of one word on one command line is a bug waiting to be typed,
    // and the help has to keep them visibly separate.
    let (ok, stdout, _) = run(&["--help"]);
    assert!(ok);
    assert!(
        stdout.contains("monitor <machine>"),
        "no subcommand in the usage"
    );
    assert!(stdout.contains("--mon "), "no --mon in the usage");
    assert!(
        stdout.contains("--monitor <name>"),
        "the ROM-image flag went missing"
    );
}

#[test]
fn the_build_says_it_has_a_monitor() {
    let (ok, stdout, _) = run(&["--version"]);
    assert!(ok);
    assert!(
        stdout.contains("monitor"),
        "`--version` does not list the feature:\n{stdout}"
    );
}
