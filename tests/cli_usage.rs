//! What the binary says to somebody who has typed the wrong thing.
//!
//! Every test here is a command line a new user plausibly types on their first
//! afternoon, and each one used to answer with something that was false,
//! missing, or unhelpful:
//!
//! * `rsemu machines` advertised flags `rsemu run` does not take;
//! * a forgotten `--cart` was reported as a list of seventeen slots the user
//!   had never named, and never mentioned the flag;
//! * a file given to a slot the machine does not have was dropped in silence;
//! * `--drive` worked only for slots on a hard-coded list;
//! * an output path that could not be written was discovered after the run;
//! * `rsemu run --help` was an unknown option.
//!
//! The binary is run rather than its functions called, for the reason
//! `tests/cli_span.rs` gives: what failed each time was what a process prints
//! and exits with, and only a process has those.

#![cfg(feature = "cli")]

use std::process::Command;

/// Run the shipped binary: exit code, stdout, stderr.
fn run(args: &[&str]) -> (Option<i32>, String, String) {
    let out = Command::new(env!("CARGO_BIN_EXE_rsemu"))
        .args(args)
        .output()
        .expect("the binary this test was built alongside");
    (
        out.status.code(),
        String::from_utf8_lossy(&out.stdout).into_owned(),
        String::from_utf8_lossy(&out.stderr).into_owned(),
    )
}

/// A file that certainly exists, for a flag that wants one to read.
#[allow(dead_code)]
const SOME_FILE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/Cargo.toml");

/// A scratch path nobody else in this run will pick.
#[allow(dead_code)]
fn scratch(name: &str) -> std::path::PathBuf {
    std::env::temp_dir().join(format!("rsemu-usage-{}-{name}", std::process::id()))
}

/// Every flag `rsemu machines` prints is one `rsemu run` accepts.
///
/// The listing printed `--<slot> <file>` for every media slot in the catalog,
/// and only fifteen slots have such a flag: `--kernel`, `--firmware`,
/// `--kickstart`, `--macrom`, `--nvme0` and the rest were advertised and then
/// refused as unknown options. Each advertised flag is handed to `run` here,
/// with a machine that does not exist so nothing is built, and must get past
/// the argument parser.
#[test]
fn machines_advertises_only_flags_that_run_takes() {
    let (code, stdout, stderr) = run(&["machines"]);
    assert_eq!(code, Some(0), "{stderr}");

    let mut flags: Vec<String> = Vec::new();
    let mut media_lines = 0;
    for line in stdout.lines() {
        let Some(rest) = line.trim_start().strip_prefix("media ") else {
            continue;
        };
        if !line.starts_with(' ') {
            continue;
        }
        media_lines += 1;
        for spelling in rest.split(", ") {
            let flag = spelling.split(' ').next().unwrap_or_default().to_string();
            assert!(
                flag.starts_with("--"),
                "not a flag: {spelling:?} in {line:?}"
            );
            if flag == "--media" {
                assert!(
                    spelling.ends_with("=<file>"),
                    "`--media` takes <slot>=<file>: {spelling:?}"
                );
            } else {
                assert!(
                    spelling.ends_with(" <file>"),
                    "a slot's own flag takes a file: {spelling:?}"
                );
            }
            if !flags.contains(&flag) {
                flags.push(flag);
            }
        }
    }
    if media_lines == 0 {
        // A build with no machine that takes media has nothing to advertise.
        return;
    }
    for flag in &flags {
        let value = if flag == "--media" {
            "slot=/nonexistent"
        } else {
            "/nonexistent"
        };
        let (_, _, stderr) = run(&["run", "no-such-machine", flag, value]);
        assert!(
            !stderr.contains("unknown option"),
            "`rsemu machines` advertises {flag}, and `rsemu run` answers: {stderr}"
        );
    }
}

/// A class this build lacks is reported at the line that names it.
///
/// It was the one front-end mistake with no location: `registry: unknown
/// device class …`, and the reader was left to search the file.
#[test]
fn an_unknown_class_is_reported_where_it_is_written() {
    let file = scratch("typo.machine");
    std::fs::write(
        &file,
        "machine \"typo\" {\n  space bus { width = 16, unassigned = open-bus }\n  \
         object a \"no.such.class\" { size = 4K }\n}\n",
    )
    .expect("a scratch machine file");
    let path = file.to_str().expect("a UTF-8 temp dir");
    let (code, _, stderr) = run(&["run", path, "--headless", "--for", "1ms"]);
    let _ = std::fs::remove_file(&file);
    assert_eq!(code, Some(1), "{stderr}");
    assert!(
        stderr.contains("unknown device class `no.such.class`"),
        "{stderr}"
    );
    assert!(stderr.contains(&format!("{path}:3:12")), "{stderr}");
    assert!(stderr.contains("^^^^^^^^^^^^^^^"), "no caret: {stderr}");
}
