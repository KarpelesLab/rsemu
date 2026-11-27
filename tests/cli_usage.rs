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

/// `rsemu run --help` prints the help.
#[test]
fn run_help_is_the_help_and_not_an_unknown_option() {
    for flag in ["--help", "-h"] {
        let (code, stdout, stderr) = run(&["run", flag]);
        assert_eq!(code, Some(0), "{stderr}");
        assert!(stdout.contains("RUN OPTIONS"), "{stdout}");
        assert!(!stderr.contains("unknown option"), "{stderr}");
    }
}

/// Two refusals whose text had a hole in it.
///
/// Each message was a string literal continued over a line, and each lost its
/// backslash, so the next line's indentation was printed as part of the
/// sentence: `give one of                          them a file`.
#[test]
fn a_refusal_is_one_sentence_with_no_gaps_in_it() {
    let (code, _, stderr) = run(&["run", "any", "--capture", "a", "--capture", "b"]);
    assert_eq!(code, Some(2), "{stderr}");
    assert!(stderr.contains("both go to stdout"), "{stderr}");
    assert!(!stderr.contains("  "), "a gap in: {stderr:?}");

    let (code, _, stderr) = run(&["run", "any", "--console", "tty", "--capture", "tty"]);
    assert_eq!(code, Some(2), "{stderr}");
    assert!(stderr.contains("two listeners on one port"), "{stderr}");
    assert!(!stderr.contains("  "), "a gap in: {stderr:?}");
}

/// A NES with no cartridge says `--cart`, and nothing about slots it lacks.
///
/// It used to say: "`rom` names the media slot `cart`, which nothing is bound
/// to; bound slots are `cd0`, `cdrom`, `df0`, `disk`, `ext`, `flash`, …" —
/// seventeen empty defaults the binary had bound on the user's behalf, and no
/// mention of the flag that fixes it.
#[cfg(feature = "machine-nes")]
#[test]
fn a_forgotten_cartridge_names_the_flag_that_supplies_one() {
    let (code, stdout, stderr) = run(&["run", "nes-ntsc", "--headless", "--for", "1ms"]);
    assert_eq!(code, Some(2), "{stderr}");
    assert!(stderr.contains("`cart`"), "{stderr}");
    assert!(stderr.contains("--cart <file>"), "{stderr}");
    for unrelated in ["flash0", "cdrom", "kwi", "vgabios"] {
        assert!(!stderr.contains(unrelated), "{unrelated} in: {stderr}");
    }
    assert!(!stdout.contains("ran to"), "nothing ran: {stdout}");
}

/// The same for a slot with no flag of its own: it is spelled `--media`.
#[cfg(feature = "machine-z80-mini")]
#[test]
fn a_missing_slot_with_no_flag_of_its_own_is_spelled_with_media() {
    let (code, _, stderr) = run(&["run", "z80-mini", "--headless", "--for", "1ms"]);
    assert_eq!(code, Some(2), "{stderr}");
    assert!(stderr.contains("--media firmware=<file>"), "{stderr}");
}

/// A file given to a slot the machine does not have is said to be unused.
///
/// `rsemu run apple1 --cart game.nes` and `--media rmo=monitor.bin` both ran
/// exactly as if the flag had not been typed: exit zero, the same state hash,
/// not a word. The run is still a valid one, so this is a warning.
#[cfg(feature = "machine-apple1")]
#[test]
fn a_file_for_a_slot_the_machine_lacks_is_reported_as_unused() {
    for flag in [
        ["--cart", SOME_FILE],
        ["--media", &format!("rmo={SOME_FILE}")],
    ] {
        let (code, _, stderr) = run(&[
            "run",
            "apple1",
            "--headless",
            "--for",
            "1ms",
            flag[0],
            flag[1],
        ]);
        assert_eq!(code, Some(0), "{stderr}");
        assert!(stderr.contains("warning"), "{flag:?}: {stderr}");
        assert!(stderr.contains("not used"), "{flag:?}: {stderr}");
        // And what the machine does have, so a typo can be seen for one.
        assert!(stderr.contains("`rom`"), "{flag:?}: {stderr}");
    }

    // The slot the machine names draws no warning.
    let (code, _, stderr) = run(&[
        "run",
        "apple1",
        "--headless",
        "--for",
        "1ms",
        "--rom",
        SOME_FILE,
    ]);
    assert!(!stderr.contains("warning"), "{code:?}: {stderr}");
}

/// An output file that cannot be written is refused before the run.
///
/// All three are written when the run ends, so a misspelt directory used to
/// cost the whole span: the machine ran, the summary was printed, and only
/// then did `--record-input` report that there was nowhere to put the log.
#[cfg(feature = "machine-apple1")]
#[test]
fn an_output_that_cannot_be_written_is_refused_before_the_run() {
    let (code, stdout, stderr) = run(&[
        "run",
        "apple1",
        "--headless",
        "--for",
        "1ms",
        "--record-input",
        "/no-such-directory-rsemu/session.trace",
    ]);
    assert_eq!(code, Some(2), "{stderr}");
    assert!(stderr.contains("--record-input"), "{stderr}");
    assert!(stderr.contains("cannot be written"), "{stderr}");
    assert!(!stdout.contains("ran to"), "it ran first: {stdout}");

    // A path that can be written is still written, and the probe that asked
    // leaves nothing of its own behind: the file holds the log.
    let log = scratch("session.trace");
    let _ = std::fs::remove_file(&log);
    let (code, stdout, stderr) = run(&[
        "run",
        "apple1",
        "--headless",
        "--for",
        "1ms",
        "--record-input",
        log.to_str().expect("a UTF-8 temp dir"),
    ]);
    assert_eq!(code, Some(0), "{stderr}");
    assert!(stdout.contains("ran to"), "{stdout}");
    let written = std::fs::metadata(&log).map(|m| m.len()).unwrap_or(0);
    let _ = std::fs::remove_file(&log);
    assert!(written > 0, "the input log was not written");
}

/// `--drive` backs whichever slot it names, not only the ones on a list.
///
/// The drive's device finds its image by the slot's name, which it learns
/// from a media binding — and the binary bound an empty one only for a
/// hard-coded set of slots. `sata0` was not in it, so the one flag that binds
/// the slot was refused with "nothing is bound to `sata0`".
#[cfg(all(feature = "machine-ahci-mini", feature = "dev-blk"))]
#[test]
fn a_drive_binds_the_slot_it_names() {
    let image = scratch("sata0.img");
    std::fs::write(&image, vec![0u8; 1 << 20]).expect("a scratch image");
    let spec = format!("sata0={}", image.to_str().expect("a UTF-8 temp dir"));
    let (code, stdout, stderr) = run(&[
        "run",
        "ahci-mini",
        "--headless",
        "--for",
        "1ms",
        "--drive",
        &spec,
    ]);
    let _ = std::fs::remove_file(&image);
    assert_eq!(code, Some(0), "{stderr}");
    assert!(stdout.contains("ran to"), "{stdout}");
    assert!(!stderr.contains("never picked up"), "{stderr}");
}

/// `pc-at-smp` starts on rsemu's own firmware, as its machine file says.
///
/// `machines/pc-at-smp.machine` opens with `rsemu run pc-at-smp --hd0
/// disk.img` and says it "boots with nothing supplied". The built-in BIOS was
/// offered by machine stem and this stem was not in the list.
#[cfg(all(feature = "machine-pc-at-smp", feature = "fw-pcbios"))]
#[test]
fn the_two_processor_at_starts_on_the_built_in_firmware() {
    let (code, stdout, stderr) = run(&["run", "pc-at-smp", "--headless", "--for", "1ms"]);
    assert_eq!(code, Some(0), "{stderr}");
    assert!(stdout.contains("ran to"), "{stdout}");
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

/// A build without the gdb stub says so, rather than "unknown command".
///
/// The help lists `debug` in every build. `monitor` and `--gdb` have always
/// named the missing feature; the subcommand was simply not matched.
#[cfg(not(feature = "gdb"))]
#[test]
fn debug_without_the_stub_names_the_feature() {
    let (code, _, stderr) = run(&["debug", "any"]);
    assert_eq!(code, Some(2), "{stderr}");
    assert!(stderr.contains("`gdb` feature"), "{stderr}");
    assert!(!stderr.contains("unknown command"), "{stderr}");
}

/// A disk the host cannot hold is a refused property, not a core dump.
///
/// `-p hd0size=1024G` — one letter from the `1024M` somebody meant — aborted
/// the process with `memory allocation of 1099511631871 bytes failed`: the
/// drive sized a `RamStore` from the number and the store could only allocate
/// infallibly. No exit code, no message of rsemu's own, and a core file.
#[cfg(feature = "machine-pc-at")]
#[test]
fn a_disk_larger_than_the_host_is_refused_not_aborted() {
    let (code, _, stderr) = run(&[
        "run",
        "pc-at",
        "--headless",
        "--for",
        "1ms",
        "-p",
        "hd0size=1024G",
    ]);
    assert_eq!(code, Some(1), "{stderr}");
    assert!(
        stderr.contains("cannot allocate 1099511627776 bytes"),
        "{stderr}"
    );
    assert!(!stderr.contains("memory allocation of"), "{stderr}");
}

/// `describe` with nothing that could be a class name suggests nothing.
///
/// Every string contains the empty one, so `rsemu describe ""` was told it
/// might have meant whichever class sorts first.
#[test]
fn an_empty_class_name_gets_no_suggestion() {
    for name in ["", "."] {
        let (code, _, stderr) = run(&["describe", name]);
        assert_eq!(code, Some(1), "{stderr}");
        assert!(stderr.contains("unknown device class"), "{stderr}");
        assert!(!stderr.contains("did you mean"), "`{name}`: {stderr}");
    }
}

/// A `--console` naming a port the machine never opened is refused under
/// `--headless` too.
///
/// The name was only looked at on the path that attaches a terminal, which
/// `--headless` skips: the run went to its end and exited 0, and the typo
/// waited for the day somebody dropped `--headless` from the script.
#[cfg(feature = "machine-apple1")]
#[test]
fn a_console_that_does_not_exist_is_refused_even_headless() {
    let (code, stdout, stderr) = run(&[
        "run",
        "apple1",
        "--headless",
        "--console",
        "nosuch",
        "--for",
        "1ms",
    ]);
    assert_eq!(code, Some(2), "stdout: {stdout}\nstderr: {stderr}");
    assert!(
        stderr.contains("no character port named `nosuch`"),
        "{stderr}"
    );
    assert!(stderr.contains("`console`"), "the ports it has: {stderr}");

    // And a port that does exist is still accepted and still not attached.
    let (code, stdout, stderr) = run(&[
        "run",
        "apple1",
        "--headless",
        "--console",
        "console",
        "--for",
        "1ms",
    ]);
    assert_eq!(code, Some(0), "{stderr}");
    assert!(stdout.contains("ran to"), "{stdout}");
}
