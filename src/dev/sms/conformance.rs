//! The whole-machine conformance runner for the Master System.
//!
//! `ROADMAP.md` §0: *accuracy is measured, never asserted*. The Z80's own runner
//! (`cpu::z80::conformance`) measures the processor on a hand-built minimal bus
//! and passes `zexall` 67/67 there. This one runs the **shipped machine** —
//! `machines/sms-ntsc.machine`, realized through the catalog exactly as
//! `rsemu run sms-ntsc` would — which is a different claim: that a Z80 fetching
//! through a bank-switched cartridge, with a VDP holding `/INT` and a scheduler
//! interleaving three lazily-advanced devices, still executes the same
//! instructions.
//!
//! # What there is to run
//!
//! Less than one would like, and the honest summary is worth writing down.
//!
//! The Master System's test-ROM ecosystem is small and almost all of it reports
//! **on screen**: FluBBa's *SMS VDP Test* and sverx's *SMS Test Suite* draw their
//! results and, in the second case, want buttons pressed. Neither has a
//! documented pass/fail memory location, so automating either means hashing a
//! framebuffer against a picture nobody has published — a harness that would
//! assert what this emulator happens to do rather than what the hardware does.
//!
//! The exception is **ZEXALL-SMS**, Maxim's port of Frank Cringle's Z80
//! instruction exerciser, which writes its verdict to the
//! [SDSC debug console](super::sdsc) a character at a time. That is what this
//! module runs, and it is why `sms.sdsc` exists.
//!
//! It is **GPL-2.0** — the licence file ships inside the archive — so it is
//! fetched at test time and never committed. Running a GPL program as an
//! emulated guest is ordinary use; redistributing it is not ours to do
//! (`CLAUDE.md`, Testing).
//!
//! # Running
//!
//! ```text
//! scripts/fetch-testdata.sh sms
//! RSEMU_CONFORMANCE=1 cargo test --release --all-features sms::conformance -- --nocapture
//! ```
//!
//! | Variable | Points at |
//! | --- | --- |
//! | `RSEMU_SMS_ZEXALL_DIR` | a directory holding `zexdoc.sms` and `zexall.sms` |
//! | `RSEMU_SMS_ZEXALL_ROM` | which of them to run (default `zexall.sms`, the harder) |
//! | `RSEMU_SMS_FRAMES` | emulated frames the ROM may run for (default 600000) |
//!
//! **The budget matters here more than in most suites.** The exerciser's own
//! README warns that its longest single test takes "well over an hour" on real
//! hardware; the tests are ordered fastest-first precisely so that a truncated
//! run is still informative. So this runner reports what it saw, fails on any
//! test that *ran and disagreed*, and says plainly when it stopped early rather
//! than calling a truncated run a pass.
//!
//! # What it says today
//!
//! `zexall.sms` — the harder of the two, which checks the undocumented flag
//! bits 3 and 5 as well — reports **79/79 tests agreed, run complete**, in 221
//! seconds of wall clock in a release build. The ledger below is empty and this
//! is why.
//!
//! Without the gate, or without a corpus, the runner prints why it is doing
//! nothing and passes. `cargo test` offline stays green; that is a rule, not a
//! convenience.
//!
//! # Reading the result out
//!
//! There is deliberately no route from a `dyn Device` to an [`SdscConsole`] —
//! the core keeps `Any` out of the supertrait chain on purpose — so this reads
//! the console's text out of its **snapshot chunk**, the way
//! `ROADMAP.md` §4.5 already promises anyone can. The Game Boy's runner reads
//! blargg's serial transcript the same way.
//!
//! [`SdscConsole`]: super::sdsc::SdscConsole

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use std::path::PathBuf;

use crate::core::state::{ChunkReader, MachineShape, Migrations, Source, StateReader, StateWriter};
use crate::machine::{Machine, catalog};

/// The master gate.
const GATE: &str = "RSEMU_CONFORMANCE";

/// Overrides the corpus root. Defaults to `<repo>/testdata`.
const TESTDATA: &str = "RSEMU_TESTDATA";

/// How many emulated frames the exerciser may run for by default.
///
/// 600 000 frames is about two and three quarter emulated *hours*, which is
/// what a complete `zexall.sms` takes — measured, not guessed: the whole list
/// finishes in 221 seconds of wall clock in a release build on the reference
/// machine. Lower it with `RSEMU_SMS_FRAMES` for a quick partial run; the tests
/// are ordered fastest-first, so a truncated one still says something.
///
/// A debug build is roughly forty times slower, which is why every instruction
/// for running this suite says `--release`.
const DEFAULT_FRAMES: u64 = 600_000;

/// How many scheduler quanta to run between checks of the console.
///
/// A quantum here is bounded by the VDP's nearest boundary — a line has two, so
/// 342 pixels at most. A few thousand of them is a handful of frames.
const QUANTA_PER_CHECK: u32 = 4096;

/// The line the exerciser prints when it has run everything.
const DONE: &str = "Tests complete";

/// The known-failures ledger for the whole-machine exerciser run.
///
/// `ROADMAP.md` §0 asks for a ledger that **only ever shrinks**, and this is it.
/// It is empty: the Z80 passes `zexall` 67/67 on its own bus, and nothing about
/// putting it in a Master System should change an instruction's result. An entry
/// here would mean the *machine* broke something the core gets right, which is
/// the most interesting kind of failure this suite can report.
const LEDGER: &[&str] = &[];

fn enabled() -> bool {
    matches!(
        std::env::var(GATE).as_deref(),
        Ok("1") | Ok("true") | Ok("yes")
    )
}

fn testdata_root() -> PathBuf {
    match std::env::var_os(TESTDATA) {
        Some(dir) => PathBuf::from(dir),
        None => PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("testdata"),
    }
}

fn frame_limit() -> u64 {
    std::env::var("RSEMU_SMS_FRAMES")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(DEFAULT_FRAMES)
}

/// The exerciser image, or the reason there is nothing to run.
fn corpus() -> Option<PathBuf> {
    if !enabled() {
        println!("SKIP sms-zexall: set {GATE}=1 to run conformance suites");
        return None;
    }
    let dir = match std::env::var_os("RSEMU_SMS_ZEXALL_DIR") {
        Some(d) => PathBuf::from(d),
        None => testdata_root().join("sms-zexall"),
    };
    let name = std::env::var("RSEMU_SMS_ZEXALL_ROM").unwrap_or_else(|_| "zexall.sms".to_string());
    let rom = dir.join(&name);
    if !rom.is_file() {
        println!("SKIP sms-zexall: {} not found", rom.display());
        println!("      fetch it with: scripts/fetch-testdata.sh sms");
        return None;
    }
    Some(rom)
}

// ---------------------------------------------------------------------------
// Reading a device's state without downcasting it
// ---------------------------------------------------------------------------

/// One device's snapshot chunk, as bytes.
fn chunk_of(machine: &Machine, path: &str) -> Option<Vec<u8>> {
    let entry = machine.device(path)?;
    let class = entry.class();
    let mut writer = StateWriter::new(MachineShape::new());
    {
        let mut chunk = writer.chunk(path, class.name, class.version).ok()?;
        entry.device().save(&mut chunk).ok()?;
    }
    let bytes = writer.to_vec().ok()?;
    let reader = StateReader::new(&bytes).ok()?;
    let chunk = reader
        .load(path, class.name, class.version, &Migrations::new())
        .ok()?;
    Some(chunk.into_data())
}

/// Everything the guest has printed to the debug console.
///
/// `sms.sdsc` writes its chunk version, then the log as a length-prefixed
/// string.
fn console_text(machine: &Machine) -> Option<String> {
    let data = chunk_of(machine, "sdsc")?;
    let mut r = ChunkReader::new(&data);
    let _version = r.read_u32().ok()?;
    r.read_string().ok()
}

/// How many frames the VDP has finished.
///
/// `sms.vdp` writes its version, three length-prefixed arrays, thirteen scalars,
/// then dot, line, dots and frame.
fn frames(machine: &Machine) -> Option<u64> {
    let data = chunk_of(machine, "vdp")?;
    let mut r = ChunkReader::new(&data);
    let _version = r.read_u32().ok()?;
    let _vram = r.read_bytes().ok()?;
    let _cram = r.read_bytes().ok()?;
    let _regs = r.read_bytes().ok()?;
    let _addr = r.read_u16().ok()?;
    for _ in 0..4 {
        r.read_u8().ok()?;
    }
    let _latch = ();
    let _status = r.read_u8().ok()?;
    let _line_counter = r.read_u8().ok()?;
    let _line_irq = r.read_bool().ok()?;
    let _vscroll = r.read_u8().ok()?;
    let _dot = r.read_u64().ok()?;
    let _line = r.read_u16().ok()?;
    let _dots = r.read_u64().ok()?;
    r.read_u64().ok()
}

// ---------------------------------------------------------------------------
// Parsing what the exerciser says
// ---------------------------------------------------------------------------

/// One test's verdict.
#[derive(Debug, PartialEq, Eq)]
struct Verdict {
    name: String,
    passed: bool,
    detail: String,
}

/// Read the exerciser's transcript.
///
/// Each test prints its name padded with dots, then either `OK` and a newline,
/// or a newline followed by ` CRC <found> expected <wanted>`. So a line ending
/// in `OK` is a pass and a line beginning with ` CRC ` is the previous line's
/// failure.
fn parse(text: &str) -> Vec<Verdict> {
    let mut out: Vec<Verdict> = Vec::new();
    let mut pending: Option<String> = None;
    for line in text.lines() {
        let trimmed = line.trim_end_matches('\r');
        if let Some(rest) = trimmed.strip_prefix(" CRC ") {
            let name = pending.take().unwrap_or_else(|| "<unnamed>".to_string());
            out.push(Verdict {
                name: tidy(&name),
                passed: false,
                detail: rest.trim().to_string(),
            });
        } else if let Some(name) = trimmed.strip_suffix("OK") {
            pending = None;
            out.push(Verdict {
                name: tidy(name),
                passed: true,
                detail: String::new(),
            });
        } else if !trimmed.trim().is_empty() {
            pending = Some(trimmed.to_string());
        }
    }
    out
}

/// A test's name without the dot padding the exerciser aligns it with.
fn tidy(name: &str) -> String {
    name.trim_end_matches(['.', ' ']).trim().to_string()
}

// ---------------------------------------------------------------------------
// The runner
// ---------------------------------------------------------------------------

#[test]
fn zexall_on_the_shipped_machine() {
    let Some(path) = corpus() else {
        return;
    };
    let rom = match std::fs::read(&path) {
        Ok(bytes) => bytes,
        Err(e) => {
            println!("SKIP sms-zexall: cannot read {}: {e}", path.display());
            return;
        }
    };
    let mut machine = match catalog::build_catalog("sms-ntsc", &[("cart", &rom)]) {
        Ok(m) => m,
        Err(e) => panic!("`machine-sms` is on but sms-ntsc will not realize: {e}"),
    };

    let limit = frame_limit();
    let mut complete = false;
    // The frame counter is the budget, and it cannot stall the way the Game
    // Boy's can: this VDP counts lines whether or not the display is enabled,
    // because the counters a program polls keep running through a blanked
    // screen. So one limit is enough here where the Game Boy needed two.
    while frames(&machine).unwrap_or(u64::MAX) < limit {
        for _ in 0..QUANTA_PER_CHECK {
            if machine.run_quantum().is_err() {
                break;
            }
        }
        if console_text(&machine).is_some_and(|t| t.contains(DONE)) {
            complete = true;
            break;
        }
    }

    let text = console_text(&machine).unwrap_or_default();
    let verdicts = parse(&text);
    let passed = verdicts.iter().filter(|v| v.passed).count();
    let failed: Vec<&Verdict> = verdicts.iter().filter(|v| !v.passed).collect();

    let mut report = format!(
        "sms-zexall ({}): {passed}/{} tests agreed{}\n",
        path.file_name().unwrap_or_default().to_string_lossy(),
        verdicts.len(),
        if complete {
            ", run complete"
        } else {
            ", run TRUNCATED by the frame budget"
        }
    );
    for verdict in &failed {
        report.push_str(&format!("  FAIL {}: {}\n", verdict.name, verdict.detail));
    }
    println!("{report}");

    if verdicts.is_empty() {
        panic!(
            "the exerciser printed nothing in {limit} frames — the debug console, the mapper or \
             the core is not working at all:\n{text:?}"
        );
    }

    let unexpected: Vec<&&Verdict> = failed
        .iter()
        .filter(|v| !LEDGER.contains(&v.name.as_str()))
        .collect();
    assert!(
        unexpected.is_empty(),
        "{} test(s) disagreed and are not in the ledger:\n{}",
        unexpected.len(),
        unexpected
            .iter()
            .map(|v| format!("  {}: {}", v.name, v.detail))
            .collect::<Vec<_>>()
            .join("\n")
    );
    // A ledger only ever shrinks, so an entry that has started passing is a
    // failure of the ledger rather than of the emulator — it must be deleted.
    for entry in LEDGER {
        assert!(
            failed.iter().any(|v| v.name == *entry),
            "`{entry}` is in the ledger but passed; the ledger only ever shrinks, so remove it"
        );
    }
}

#[cfg(test)]
mod parsing {
    use super::{parse, tidy};

    #[test]
    fn a_passing_line_is_a_name_and_ok() {
        let v = parse("ld hl,(nnnn)........OK\n");
        assert_eq!(v.len(), 1);
        assert!(v[0].passed);
        assert_eq!(v[0].name, "ld hl,(nnnn)");
    }

    #[test]
    fn a_failing_test_names_itself_on_the_line_before_its_crc() {
        let text = "add hl,<bc,de,hl,sp>..\n CRC 12345678 expected 9abcdef0\n";
        let v = parse(text);
        assert_eq!(v.len(), 1);
        assert!(!v[0].passed);
        assert_eq!(v[0].name, "add hl,<bc,de,hl,sp>");
        assert_eq!(v[0].detail, "12345678 expected 9abcdef0");
    }

    #[test]
    fn a_banner_line_is_not_a_verdict() {
        let text = "Z80 instruction exerciser\n* SMS Mode 4\nld a,i..OK\nTests complete\n";
        let v = parse(text);
        assert_eq!(v.len(), 1, "only the one test line counts");
        assert_eq!(v[0].name, "ld a,i");
    }

    #[test]
    fn padding_dots_are_not_part_of_a_name() {
        assert_eq!(tidy("neg...................."), "neg");
    }
}
