//! Gating, corpus discovery, and skip reporting.
//!
//! Two rules from `CLAUDE.md` shape everything here:
//!
//! 1. Corpora are downloaded into a git-ignored directory and **never
//!    committed**. Several are copyleft or unlicensed; running them is fine,
//!    redistributing them is not.
//! 2. `cargo test` offline must stay green. So every suite here is opt-in and
//!    a missing corpus is a *skip*, printed, never a failure.
//!
//! A skip is loud rather than silent: the reason is printed and, where it
//! matters, so is the command that would fix it. A conformance suite that
//! quietly does nothing is worse than one that is not written.

use std::path::{Path, PathBuf};

/// The master gate. Nothing in this binary touches a corpus unless it is set.
pub(crate) const GATE: &str = "RSEMU_CONFORMANCE";

/// Overrides the corpus root. Defaults to `<repo>/testdata`.
pub(crate) const TESTDATA_ENV: &str = "RSEMU_TESTDATA";

/// Is the conformance gate open?
pub(crate) fn enabled() -> bool {
    matches!(
        std::env::var(GATE).as_deref(),
        Ok("1") | Ok("true") | Ok("yes")
    )
}

/// Is an optional boolean switch on?
pub(crate) fn flag(name: &str) -> bool {
    matches!(
        std::env::var(name).as_deref(),
        Ok("1") | Ok("true") | Ok("yes")
    )
}

/// The root of the downloaded corpora.
pub(crate) fn testdata_root() -> PathBuf {
    match std::env::var_os(TESTDATA_ENV) {
        Some(dir) => PathBuf::from(dir),
        None => PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("testdata"),
    }
}

/// Where per-run reports are written. Under `target/`, so it is already ignored.
pub(crate) fn report_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("target")
        .join("conformance")
}

/// Write a report, best-effort. A failure to write one must never turn a
/// passing suite red, so this only warns.
pub(crate) fn write_report(name: &str, body: &str) {
    let dir = report_dir();
    if let Err(e) = std::fs::create_dir_all(&dir) {
        eprintln!("note: could not create {}: {e}", dir.display());
        return;
    }
    let path = dir.join(name);
    match std::fs::write(&path, body) {
        Ok(()) => println!("  report: {}", path.display()),
        Err(e) => eprintln!("note: could not write {}: {e}", path.display()),
    }
}

// ---------------------------------------------------------------------------
// Running code that is allowed to be wrong
// ---------------------------------------------------------------------------

/// Run `f` with the panic hook muted on this thread.
///
/// The code under test is exactly the code most likely to panic, and a core
/// that panics on ten thousand vectors would otherwise print ten thousand
/// backtraces before the report that explains them. The hook is installed once,
/// globally, and defers to the previous hook whenever the thread-local flag is
/// off — so a genuine assertion failure elsewhere in this binary still prints.
pub(crate) fn quietly<T>(f: impl FnOnce() -> T) -> T {
    use std::cell::Cell;
    use std::sync::Once;

    thread_local! {
        static QUIET: Cell<bool> = const { Cell::new(false) };
    }
    static HOOK: Once = Once::new();

    HOOK.call_once(|| {
        let previous = std::panic::take_hook();
        std::panic::set_hook(Box::new(move |info| {
            if !QUIET.with(|q| q.get()) {
                previous(info);
            }
        }));
    });

    QUIET.with(|q| q.set(true));
    let out = f();
    QUIET.with(|q| q.set(false));
    out
}

/// Call `f`, converting a panic into its message.
///
/// One broken opcode must not take down the other 255, and a core that panics
/// halfway through a trace should still get a report saying where.
pub(crate) fn catching<T>(f: impl FnOnce() -> T) -> Result<T, String> {
    quietly(|| std::panic::catch_unwind(std::panic::AssertUnwindSafe(f)))
        .map_err(|payload| panic_message(&*payload))
}

/// The human-readable part of a panic payload, if there is one.
///
/// `&*payload`, never `&payload`: the latter unsize-coerces the *box* into
/// `dyn Any`, and every downcast then misses.
pub(crate) fn panic_message(payload: &(dyn std::any::Any + Send)) -> String {
    if let Some(s) = payload.downcast_ref::<&str>() {
        (*s).to_string()
    } else if let Some(s) = payload.downcast_ref::<String>() {
        s.clone()
    } else {
        "<non-string panic payload>".to_string()
    }
}

/// Why a suite did not run. Every arm is a skip, never a failure.
///
/// **There is deliberately no arm for "the harness was never wired up."** That
/// was the shape `nestest` hid in for months: `SKIP nestest: no 6502 core is
/// bound in tests/conformance/cpu.rs`, printed next to a genuine
/// missing-corpus skip, indistinguishable from it at a glance, and green.
/// A skip may only ever mean something about the *environment* — the gate is
/// closed, the corpus was not fetched, this build does not include the
/// component. A seam that a build does contain the component for and still
/// cannot bind is a defect in `tests/conformance/`, and the functions that
/// bind seams assert rather than returning a `Skip` (`cpu::require_cpu`,
/// `machine::require_nes`).
#[derive(Debug)]
pub(crate) enum Skip {
    /// The gate env var is not set.
    Gated,
    /// The corpus directory or file is missing.
    NoCorpus {
        /// What was looked for.
        path: PathBuf,
        /// The command that would fetch it.
        fetch: &'static str,
    },
    /// The component the suite needs is not compiled into this build.
    ///
    /// A fact about the feature set, not about the harness: `cargo test` with
    /// default features links no 6502 and no NES, and a suite that needs one
    /// has nothing to measure. `--all-features` — which CI runs — makes this
    /// arm unreachable, so it can never hide an unwired seam.
    NotBuilt {
        /// What is missing, in prose.
        component: &'static str,
        /// The Cargo features that would supply it, as `--features` takes them.
        feature: &'static str,
    },
}

impl Skip {
    /// Print the reason and return, so the test passes with an explanation.
    pub(crate) fn report(self, suite: &str) {
        match self {
            Skip::Gated => {
                println!("SKIP {suite}: set {GATE}=1 to run conformance suites");
            }
            Skip::NoCorpus { path, fetch } => {
                println!("SKIP {suite}: corpus not found at {}", path.display());
                println!("      fetch it with: {fetch}");
            }
            Skip::NotBuilt { component, feature } => {
                println!("SKIP {suite}: this build cannot provide {component}");
                println!("      rebuild with: cargo test --features {feature}");
            }
        }
    }
}

/// Check the gate and that `path` exists, returning it or the reason to skip.
pub(crate) fn require(path: PathBuf, fetch: &'static str) -> Result<PathBuf, Skip> {
    if !enabled() {
        return Err(Skip::Gated);
    }
    if !path.exists() {
        return Err(Skip::NoCorpus { path, fetch });
    }
    Ok(path)
}

/// Read a file, or explain which one could not be read.
pub(crate) fn read(path: &Path) -> Result<Vec<u8>, String> {
    std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))
}

/// The macro every suite opens with: skip and return, or bind the value.
///
/// Written as a macro rather than a helper because the early `return` has to
/// happen in the `#[test]` function itself.
macro_rules! gated {
    ($suite:expr, $expr:expr) => {
        match $expr {
            Ok(v) => v,
            Err(skip) => {
                skip.report($suite);
                return;
            }
        }
    };
}

pub(crate) use gated;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_missing_corpus_is_a_skip_not_a_failure() {
        // Whatever the ambient environment, asking for a path that cannot
        // exist yields a Skip rather than panicking.
        let missing = testdata_root().join("definitely-not-here-8f2a");
        let outcome = require(missing, "scripts/fetch-testdata.sh nestest");
        assert!(outcome.is_err());
    }

    #[test]
    fn the_testdata_root_is_overridable() {
        // Not by mutating the environment — that races other tests — but the
        // default must at least sit inside the repo, since that is what
        // .gitignore covers.
        let root = testdata_root();
        assert!(
            root.ends_with("testdata") || std::env::var_os(TESTDATA_ENV).is_some(),
            "unexpected corpus root {}",
            root.display()
        );
    }
}
