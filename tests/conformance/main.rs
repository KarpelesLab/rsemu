//! The conformance harness: the suites that measure whether a core is right.
//!
//! `ROADMAP.md` §0 — accuracy is measured, never asserted. §12 lists what
//! measures it and §13 makes three of those a phase-3 gate.
//!
//! # Bring-up order
//!
//! The three suites here are deliberately ordered by how much machine they
//! need, and that is the order to make them pass in:
//!
//! 1. **SingleStepTests/65x02** — pure CPU. A flat 64 KiB of RAM, one
//!    instruction per vector, cycle-by-cycle bus traces. No PPU, no APU, no
//!    cartridge, no scheduler. This is the suite a 6502 author iterates against
//!    all day, and the one this harness invests the most in.
//! 2. **nestest** — CPU plus a minimal bus. A cartridge and work RAM, no
//!    rendering. Cumulative: it catches the drift a per-instruction suite
//!    structurally cannot.
//! 3. **AccuracyCoin** — the whole machine. CPU, PPU, APU, DMA and cartridge
//!    running together at the right clock alignments. Its runner reports per
//!    test so it is useful long before all of that exists.
//!
//! # Why SingleStepTests is run twice
//!
//! `src/cpu/mos6502/conformance.rs` runs the same corpus against the same core
//! behind `RSEMU_65X02_DIR`, and keeping both was a decision rather than an
//! oversight. They are not the same measurement:
//!
//! * The in-crate runner compares the final RAM cells the vector **lists**;
//!   this one compares the whole 64 KiB, so a write to an address no vector
//!   mentions fails here and passes there.
//! * Only this one can cross-check the core's *claimed* cycle count against
//!   the number of bus accesses it actually made — the in-crate runner drives
//!   the interpreter directly and has no second number to disagree with.
//! * They reach the core by different routes. The in-crate runner builds a
//!   fresh `Mos6502` per vector and sets execution state through a private
//!   field; this one drives one long-lived core through the public API only,
//!   which is how a machine drives it. That difference is not academic: it is
//!   what turned up `JAM` latching `halted` across vectors when the seam was
//!   first wired.
//! * The ledger discipline `ROADMAP.md` §0 asks for lives here
//!   (`ledgers/sst-nes6502.txt`), keyed per vector.
//!
//! Four and a half seconds for 2 560 000 vectors on a 32-thread machine. The
//! duplication is cheap and it is not really duplication.
//!
//! # Running
//!
//! ```text
//! scripts/fetch-testdata.sh --all
//! RSEMU_CONFORMANCE=1 cargo test --test conformance -- --nocapture
//! ```
//!
//! Without `RSEMU_CONFORMANCE=1`, or without a corpus, or in a build whose
//! feature set excludes the component a suite needs, that suite prints why it
//! is skipping and passes. `cargo test` offline stays green — that is a rule
//! (`CLAUDE.md`, Testing), not a convenience.
//!
//! What is **not** a skip is a seam nobody wired up.
//! [`every_seam_this_build_can_bind_is_bound`] fails on a plain `cargo test
//! --all-features`, with no gate and no corpus, if a component this build
//! contains has no adapter in `cpu.rs` or `machine.rs`. `nestest` spent months
//! printing `SKIP … no 6502 core is bound`, indistinguishable at a glance from
//! a corpus that had not been fetched, and passing while measuring nothing.
//!
//! See `docs/testing/README.md`.

// Parts of the CPU-facing contract are exercised only by the runners' own
// tests, or only by a build with a different feature set. Deleting whatever is
// unreferenced today is how a harness ends up designed by whatever happened to
// be written first.
#![allow(dead_code)]

mod accuracycoin;
mod cpu;
mod harness;
mod json;
mod ledger;
mod machine;
mod mock;
mod nestest;
mod sst;

use std::fmt::Write as _;
use std::path::{Path, PathBuf};
use std::sync::Mutex;

use harness::{Skip, gated};

/// Where the ledgers live. Committed; the corpora they describe are not.
fn ledger_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("conformance")
        .join("ledgers")
        .join(format!("{name}.txt"))
}

// ---------------------------------------------------------------------------
// Gate 1 — SingleStepTests/65x02
// ---------------------------------------------------------------------------

#[test]
fn sst_65x02_nes6502() {
    run_sst(cpu::Variant::Ricoh2A03, "sst-nes6502");
}

#[test]
fn sst_65x02_nmos6502() {
    // The decimal-mode-enabled original. Not on the phase-3 gate — the NES has
    // no working decimal mode — but the same core with `decimal: true` must
    // pass it, and the corpus is right there.
    run_sst(cpu::Variant::Nmos6502, "sst-6502");
}

fn run_sst(variant: cpu::Variant, suite: &str) {
    let fetch = "scripts/fetch-testdata.sh sst-65x02";
    let dir = gated!(
        suite,
        harness::require(sst::corpus_dir(&harness::testdata_root(), variant), fetch)
    );

    // The directory exists — `require` just said so — so a listing that fails
    // is a broken corpus or a broken filesystem, not an absent one. Skipping
    // on it would be a pass that measured nothing.
    let mut files = match sst::opcode_files(&dir) {
        Ok(files) => files,
        Err(e) => panic!("{} exists but cannot be listed: {e}", dir.display()),
    };
    assert!(
        !files.is_empty(),
        "{} exists but holds no NN.json opcode files — the fetch was incomplete.\n\
         Re-run: {fetch}",
        dir.display()
    );
    if let Some(only) = sst::opcode_filter() {
        files.retain(|(op, _)| only.contains(op));
        println!(
            "note: RSEMU_SST_OPCODES narrowed the run to {} opcode(s)",
            files.len()
        );
        assert!(
            !files.is_empty(),
            "RSEMU_SST_OPCODES matched no opcode file under {}",
            dir.display()
        );
    }

    let first_core = match cpu::require_cpu(variant) {
        Ok(core) => core,
        Err(skip) => {
            // No core in this build — but the corpus is here, so at least
            // prove it is intact and that the parser understands it. A fetch
            // that silently produced half a file should not wait for a
            // feature-enabled build to discover it.
            let (opcode, path) = &files[0];
            match harness::read(path).map(|bytes| sst::parse_vectors(&bytes)) {
                Ok(Ok(vectors)) => println!(
                    "note: corpus is readable — opcode {opcode:02x} parsed {} vectors from {}",
                    vectors.len(),
                    path.display()
                ),
                Ok(Err(e)) => panic!("corpus at {} is malformed: {e}", path.display()),
                Err(e) => panic!("cannot read the corpus: {e}"),
            }
            skip.report(suite);
            return;
        }
    };

    let threads = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
        .min(files.len());
    println!(
        "{suite}: {} opcode file(s) across {threads} thread(s)",
        files.len()
    );

    let ran: Vec<u8> = files.iter().map(|&(op, _)| op).collect();
    let queue = Mutex::new(files);
    let reports: Mutex<Vec<sst::OpcodeReport>> = Mutex::new(Vec::new());
    let errors: Mutex<Vec<String>> = Mutex::new(Vec::new());
    // The core `require_cpu` already built goes to whichever worker asks first;
    // the rest build their own. Nothing is shared between them.
    let spare = Mutex::new(Some(first_core));

    std::thread::scope(|scope| {
        for _ in 0..threads {
            scope.spawn(|| {
                let mut core = match spare.lock().unwrap().take() {
                    Some(core) => core,
                    None => cpu::new_cpu(variant).expect("require_cpu proved a core exists"),
                };
                loop {
                    let Some((opcode, path)) = queue.lock().unwrap().pop() else {
                        break;
                    };
                    let bytes = match harness::read(&path) {
                        Ok(b) => b,
                        Err(e) => {
                            errors.lock().unwrap().push(e);
                            continue;
                        }
                    };
                    let vectors = match sst::parse_vectors(&bytes) {
                        Ok(v) => v,
                        Err(e) => {
                            errors
                                .lock()
                                .unwrap()
                                .push(format!("{}: {e}", path.display()));
                            continue;
                        }
                    };
                    let report = sst::run_opcode(core.as_mut(), opcode, &vectors);
                    reports.lock().unwrap().push(report);
                }
            });
        }
    });

    let errors = errors.into_inner().unwrap();
    assert!(
        errors.is_empty(),
        "corpus problems:\n  {}",
        errors.join("\n  ")
    );

    let mut reports = reports.into_inner().unwrap();
    reports.sort_by_key(|r| r.opcode);
    summarise_sst(suite, &reports, &ran);
}

fn summarise_sst(suite: &str, reports: &[sst::OpcodeReport], ran: &[u8]) {
    let total: usize = reports.iter().map(|r| r.total).sum();
    let failed: usize = reports.iter().map(|r| r.failed.len()).sum();
    // "0/0 vectors passed" is the last shape a vacuous pass can take here: the
    // files were found, listed and parsed, and every one of them was empty.
    assert!(
        total > 0,
        "{suite}: {} opcode file(s) ran but not one vector — the corpus is empty",
        reports.len()
    );
    println!("{suite}: {}/{total} vectors passed", total - failed);

    let mut body = String::new();
    let _ = writeln!(body, "{suite}: {}/{total} vectors passed\n", total - failed);
    for r in reports.iter().filter(|r| !r.is_clean()) {
        let _ = writeln!(
            body,
            "{:02x}: {}/{} failed — {}",
            r.opcode,
            r.failed.len(),
            r.total,
            r.categories.join(", ")
        );
        body.push_str(&r.details);
    }
    harness::write_report(&format!("{suite}.txt"), &body);

    let failures: Vec<(u8, String)> = reports
        .iter()
        .flat_map(|r| r.failed.iter().map(move |name| (r.opcode, name.clone())))
        .collect();

    let path = ledger_path(suite);
    let ledger = match ledger::Ledger::load(&path) {
        Ok(l) => l,
        Err(e) => panic!("{}: {e}", path.display()),
    };
    let verdict = ledger::judge(&ledger, ran, &failures);
    if verdict.excused > 0 {
        println!(
            "  {} failure(s) excused by {}",
            verdict.excused,
            path.display()
        );
    }
    assert!(verdict.is_ok(), "\n{}", verdict.describe(&ledger));
}

// ---------------------------------------------------------------------------
// Gate 2 — nestest
// ---------------------------------------------------------------------------

#[test]
fn nestest_trace() {
    let suite = "nestest";
    let fetch = "scripts/fetch-testdata.sh nestest";
    let dir = gated!(
        suite,
        harness::require(harness::testdata_root().join("nestest"), fetch)
    );

    let rom_path = dir.join("nestest.nes");
    let log_path = dir.join("nestest.log");
    for p in [&rom_path, &log_path] {
        if !p.exists() {
            Skip::NoCorpus {
                path: p.clone(),
                fetch,
            }
            .report(suite);
            return;
        }
    }

    let log_text = match harness::read(&log_path) {
        Ok(bytes) => String::from_utf8_lossy(&bytes).into_owned(),
        Err(e) => panic!("{e}"),
    };
    let log = match nestest::parse_log(&log_text) {
        Ok(log) => log,
        Err(e) => panic!("{e}"),
    };
    assert!(
        !log.is_empty(),
        "{} parsed to zero instructions",
        log_path.display()
    );
    println!("{suite}: reference log has {} instructions", log.len());

    let image = match harness::read(&rom_path) {
        Ok(b) => b,
        Err(e) => panic!("{e}"),
    };
    let mut bus = match nestest::NestestBus::from_ines(&image) {
        Ok(bus) => bus,
        Err(e) => panic!("{}: {e}", rom_path.display()),
    };

    // Independent of any CPU: the log's first line must describe the ROM that
    // was fetched. If it does not, the two artefacts do not belong together and
    // every later comparison would be meaningless.
    let first = &log[0];
    let (entry, cyc) = nestest::automated_entry();
    assert_eq!(
        first.regs, entry,
        "the log's first line is not the automated-mode entry state"
    );
    assert_eq!(
        first.cyc, cyc,
        "the log does not start at the post-reset cycle count"
    );
    let rom_bytes = bus.peek3(entry.pc);
    assert_eq!(
        &rom_bytes[..first.bytes.len()],
        first.bytes.as_slice(),
        "the ROM at $C000 does not hold the instruction the log's first line names — \
         nestest.nes and nestest.log are from different sources"
    );
    println!("  ROM and log agree at the entry point");

    let mut core = gated!(suite, cpu::require_cpu(cpu::Variant::Ricoh2A03));

    let strict = harness::flag("RSEMU_NESTEST_DISASM");
    let report = nestest::compare(core.as_mut(), &mut bus, &log, strict);
    println!(
        "  {} of {} instructions matched",
        report.matched, report.expected
    );
    if report.unmapped > 0 {
        println!(
            "  note: {} access(es) fell outside RAM and the cartridge \
             (this bus models neither PPU nor APU)",
            report.unmapped
        );
    }

    if let Some(divergence) = &report.divergence {
        harness::write_report("nestest.txt", divergence);
        panic!("\n{divergence}");
    }

    let (documented, unofficial) = report.result_codes;
    assert_eq!(
        (documented, unofficial),
        (0, 0),
        "the trace matched but the ROM reported failures: \
         ${:04X} = {documented:02X} (documented opcodes), \
         ${:04X} = {unofficial:02X} (unofficial opcodes)",
        nestest::RESULT_ADDRS.0,
        nestest::RESULT_ADDRS.1
    );
    assert!(report.is_clean());
}

// ---------------------------------------------------------------------------
// Gate 3 — AccuracyCoin
// ---------------------------------------------------------------------------

#[test]
fn accuracycoin_whole_machine() {
    let suite = "accuracycoin";
    let fetch = "scripts/fetch-testdata.sh accuracycoin";
    let dir = gated!(
        suite,
        harness::require(harness::testdata_root().join("accuracycoin"), fetch)
    );

    let rom_path = dir.join("AccuracyCoin.nes");
    if !rom_path.exists() {
        Skip::NoCorpus {
            path: rom_path,
            fetch,
        }
        .report(suite);
        return;
    }
    let image = match harness::read(&rom_path) {
        Ok(b) => b,
        Err(e) => panic!("{e}"),
    };

    let mut nes = gated!(suite, machine::require_nes(&image));

    let report = accuracycoin::run(nes.as_mut());
    let described = report.describe();
    println!("{described}");
    harness::write_report("accuracycoin.txt", &described);

    // Ledgered the same way as the vector suite: a failure has to be written
    // down with a reason, and an entry that starts passing has to be deleted.
    // Entries are keyed by result address, which is stable across ROM releases
    // in a way the display name is not.
    let path = ledger_path("accuracycoin");
    let ledger = match ledger::Ledger::load(&path) {
        Ok(l) => l,
        Err(e) => panic!("{}: {e}", path.display()),
    };
    let failures: Vec<(u8, String)> = report
        .results
        .iter()
        .filter(|(_, o)| *o != accuracycoin::Outcome::Pass)
        .map(|(t, o)| {
            (
                page_key(t.result),
                format!("{:04X} {} [{o:?}]", t.result, t.name),
            )
        })
        .collect();
    let ran: Vec<u8> = (0..=0xff).collect();
    let verdict = ledger::judge(&ledger, &ran, &failures);
    assert_eq!(
        report.status,
        accuracycoin::RunStatus::Complete,
        "\n{described}"
    );
    assert!(verdict.is_ok(), "\n{}", verdict.describe(&ledger));
}

/// The ledger keys entries by a byte; for AccuracyCoin that byte is the low
/// half of the result address, which is unique across the 130 tests.
fn page_key(result_addr: u16) -> u8 {
    (result_addr & 0xff) as u8
}

// ---------------------------------------------------------------------------
// Always-on checks
// ---------------------------------------------------------------------------

/// The ledgers must parse even when nothing runs.
///
/// A ledger with a typo in it would otherwise be discovered on the day someone
/// finally has a corpus, a core and a deadline.
#[test]
fn every_ledger_parses() {
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/conformance/ledgers");
    let entries = std::fs::read_dir(&dir).expect("the ledger directory is committed");
    let mut checked = 0;
    for entry in entries {
        let path = entry.unwrap().path();
        if path.extension().and_then(|e| e.to_str()) != Some("txt") {
            continue;
        }
        ledger::Ledger::load(&path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        checked += 1;
    }
    assert!(checked >= 3, "expected a ledger per suite, found {checked}");
}

/// Every seam this build *can* bind **is** bound.
///
/// The check that makes an unwired harness impossible to mistake for a
/// fetch-nothing one. It needs no gate, no corpus and no environment: whether
/// a component is compiled in is a build fact, and whether the adapter for it
/// exists is a fact about this directory. `nestest` reported `SKIP` for months
/// because those two facts were reported through the same channel; here they
/// are not, and a plain `cargo test --all-features` fails the day someone
/// removes an adapter.
///
/// It deliberately does not look at `RSEMU_CONFORMANCE`. A seam that is broken
/// should fail on the developer's laptop, not only on the machine that has the
/// corpora.
#[test]
fn every_seam_this_build_can_bind_is_bound() {
    if cpu::cpu_is_built() {
        assert!(
            cpu::have_cpu(),
            "`{}` are on but tests/conformance/cpu.rs binds no core",
            cpu::CPU_FEATURES
        );
        // Not just constructible: it has to execute. A core that is bound but
        // wired to nothing would still report a green skip-free run of zero.
        let mut core = cpu::new_cpu(cpu::Variant::Ricoh2A03).expect("just checked");
        let mut bus = mock::RamBus::with(&[(0x0200, 0xa9), (0x0201, 0x42)]);
        core.set_regs(cpu::Regs {
            pc: 0x0200,
            s: 0xfd,
            p: 0x24,
            ..cpu::Regs::default()
        });
        let cycles = core.step(&mut bus);
        assert_eq!((cycles, core.regs().a, core.regs().pc), (2, 0x42, 0x0202));
    } else {
        println!(
            "note: this build cannot drive a 6502 ({} not all on)",
            cpu::CPU_FEATURES
        );
    }

    if machine::machine_is_built() {
        let rom = machine::blank_nrom();
        match machine::new_nes(&rom) {
            Ok(mut nes) => {
                nes.run_frames(1);
            }
            Err(e) => panic!(
                "`{}` are on but no NES could be built: {e}",
                machine::MACHINE_FEATURES
            ),
        }
    } else {
        println!(
            "note: this build cannot run a NES ({} not all on)",
            machine::MACHINE_FEATURES
        );
    }
}

/// Corpora must never be committed. This is a licensing rule before it is a
/// size one, so it gets an assertion rather than a note in a README.
#[test]
fn no_corpus_is_committed() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let output = std::process::Command::new("git")
        .arg("-C")
        .arg(root)
        .args(["ls-files", "--", "testdata"])
        .output();
    // A git that ran and *failed* has an empty stdout, which would otherwise
    // read as "nothing is tracked" — a pass that checked nothing. Both that
    // and a git that would not run at all are reported as what they are.
    let output = match output {
        Ok(output) if output.status.success() => output,
        Ok(output) => {
            println!(
                "note: `git ls-files` exited {}, skipping the committed-corpus check: {}",
                output.status,
                String::from_utf8_lossy(&output.stderr).trim()
            );
            return;
        }
        Err(e) => {
            println!("note: git is unavailable ({e}), skipping the committed-corpus check");
            return;
        }
    };
    let tracked = String::from_utf8_lossy(&output.stdout);
    assert!(
        tracked.trim().is_empty(),
        "these corpus files are tracked by git and must not be:\n{tracked}"
    );
}
