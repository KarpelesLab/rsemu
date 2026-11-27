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
