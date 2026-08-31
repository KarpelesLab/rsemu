//! `riscv-arch-test` — the official RISC-V architectural certification tests.
//!
//! `ROADMAP.md` §13 names this suite on the phase-5 gate alongside booting
//! Linux, and §0 asks for the number rather than the claim. This file is where
//! the number comes from.
//!
//! # What the suite is, and what running it means
//!
//! Each test is a small assembly program that exercises one instruction (or
//! one privileged mechanism) across a generated set of operand and encoding
//! corner cases, and writes every result it produces into a contiguous
//! **signature** region delimited by the symbols `begin_signature` and
//! `end_signature`. The test itself contains no expected values — conformance
//! is defined as *our signature equals the reference model's signature, byte
//! for byte*, for a reference model configured to describe the same hart.
//!
//! Upstream drives that comparison with RISCOF (until 3.9.1) or the ACT4
//! framework (after it): Python, Ruby, `uv`, `mise` and a UDB gem, wrapped
//! around exactly two operations that matter — build the tests, and diff the
//! signatures. **Neither framework is used here**, and that is a deliberate
//! decision rather than a shortcut:
//!
//! * `scripts/fetch-testdata.sh riscv-arch-test` does the building. The
//!   per-test applicability rules RISCOF reads out of each test's
//!   `RVTEST_CASE(...)` macro — an ISA regular expression and a handful of
//!   named DUT parameters — are reimplemented there in about forty lines of
//!   shell, and the `def X=Y` clauses of the case that applies become the
//!   `-D` flags the test is built with. That is the whole of RISCOF's
//!   test-selection logic for this suite.
//! * The same script generates the reference signatures once, with the
//!   **Sail RISC-V model** (BSD-2-Clause), and writes them beside the ELFs.
//!   Sail is run as a black box: a downloaded binary, invoked with
//!   `--test-signature`. No part of it is read (`CLAUDE.md`, Provenance —
//!   though as a permissive project it would be allowed to be).
//! * This file does the diffing, against the interpreter, in-process.
//!
//! So the corpus on disk is *ELFs plus reference signatures*, and running the
//! suite needs no Python, no toolchain and no reference model at all. That is
//! the same shape as every other corpus here: fetched, never committed, read
//! by a runner in this directory.
//!
//! # What is run, and what is deliberately not
//!
//! The core this suite is pointed at is **RV64GC with M, S and U mode and
//! Sv39** (`rsemu::cpu::riscv::Config::rv64gc`). The suite is organised by
//! extension, so the fetch script declares that hart's ISA string
//! (`RV64IMAFDCZicsr_Zifencei`) and builds the directories that apply to it:
//!
//! | Directory | Corpus name | Tests | Covers |
//! | --- | --- | --- | --- |
//! | `rv64i_m/I` | `rv64-I` | 51 | the base integer ISA |
//! | `rv64i_m/M` | `rv64-M` | 13 | integer multiply and divide |
//! | `rv64i_m/A` | `rv64-A` | 18 | `LR`/`SC` and the AMOs |
//! | `rv64i_m/F` | `rv64-F` | 18 | binary32 |
//! | `rv64i_m/D` | `rv64-D` | 27 | binary64 |
//! | `rv64i_m/C` | `rv64-C` | 35 of 47 | the compressed encodings |
//! | `rv64i_m/Zifencei` | `rv64-Zifencei` | 1 | `FENCE.I` |
//! | `rv64i_m/privilege` | `rv64-privilege` | 18 | traps: `ECALL`, `EBREAK`, misaligned access |
//!
//! **181 of 181 match** as of 2026-09-01, over 94 152 signature words and
//! 475 424 retired instructions, with an empty ledger
//! (`tests/conformance/ledgers/riscv-arch-test.txt`). The runner prints those
//! last two numbers and asserts they are non-zero, because a ratio on its own
//! cannot tell a clean run from a run that measured nothing.
//!
//! The twelve `C` tests that are not built are the Zcb encodings (`c.lbu`,
//! `c.sext.b`, `c.mul` …); the ISA selector drops them because this hart does
//! not implement Zcb, exactly as it would for any other DUT. `skipped.txt` in
//! the fetched corpus lists them by name with that reason, and the runner
//! prints the count, so an unimplemented extension cannot read as a pass.
//!
//! **Not run, and not counted anywhere as passing.** An unrun suite that goes
//! unmentioned reads as a pass, so every directory the suite has is on one of
//! these two lists:
//!
//! | Directory | Tests | Why not |
//! | --- | --- | --- |
//! | `rv64i_m/B` | 43 | Zba, Zbb and Zbs are not implemented |
//! | `rv64i_m/K` | 53 | the scalar cryptography extensions are not implemented |
//! | `rv64i_m/Zfh` | 18 | no half-precision |
//! | `rv64i_m/Zfinx` | 138 | no `Zfinx`/`Zdinx` — this core has an `f` register file |
//! | `rv64i_m/D_Zfa`, `F_Zfa` | 39 | Zfa is not implemented |
//! | `rv64i_m/Zicond` | 2 | not implemented |
//! | `rv64i_m/Zacas` | 3 | not implemented |
//! | `rv64i_m/Zcmop`, `Zimop` | 48 | not implemented |
//! | `rv64i_m/CMO` | 1 | no cache-management operations |
//! | `rv64i_m/Zcb` (inside `C`) | 12 | not implemented; see above |
//! | `rv64i_m/Svadu` | 3 | Svadu is *runtime-selectable* A/D update through `menvcfg.ADUE`. This core updates A and D in the page-table walk unconditionally and has no such control, so the tests do not describe it either way. |
//! | `rv64i_m/P_unratified` | 311 | unratified, and not implemented |
//! | `rv32i_m/*` | 409 | the same tests for a 32-bit hart. rsemu runs RV32 from the same core (`Config::xlen`) and the runner already picks the width from the ELF class — but `rv32i_m/D` alone is 313 MB of generated source, four of its fused-multiply-add tests being 63 MB each, and `riscv-tests` already covers RV32 across eight families. The reason is disk and minutes, not confidence; `ARCH_TEST_ARCHS` in the fetch script is the one word that changes it. |
//! | hypervisor, vector | — | this tag of the suite has no `H` or `V` directory at all, and this core implements neither |
//!
//! So the number below is a statement about RV64 M/S/U-mode IMAFDC and
//! nothing else. There is no hypervisor or vector *pass* in it, because there
//! is no hypervisor or vector *test* in the corpus it reads.
//!
//! # The reference has to describe the same hart
//!
//! This is the failure mode worth knowing about, because it looks exactly
//! like a defect in the core and is not one.
//!
//! The first run of this suite scored 178 of 181. `rv64-privilege/ecall`,
//! `rv64-privilege/ebreak` and `rv64-C/cebreak-01` each differed in three signature
//! words, and the informative one was a single bit: the first word of the trap
//! signature, which `arch_test.h` packs as
//! `zeroes | vector | entry-size | mode`. rsemu recorded a four-word entry and
//! the reference a six-word one — because `arch_test.h`'s `xcpt_sig_sv` reads
//! `misa`, and Sail's *default* configuration has the hypervisor extension
//! enabled, which widens the entry. The other two words followed from that:
//! the reference's handler went down the H-mode save-area path and never
//! returned to the instruction after the `ECALL`, so the two stores the test
//! makes there did not happen.
//!
//! Both models were right about their own hart. The fix was
//! `scripts/riscv-arch-test/sail-config.json`, which turns off H — along with
//! V, B, the crypto extensions, Zcb, Zfa, Zfh, Sv48 and Sv57, all of which
//! Sail also enables by default and none of which rsemu implements. **No
//! change was made to the core, and none was warranted.** With the reference
//! configured to match, all 181 agreed.
//!
//! The ledger's header says the same thing in the imperative: before writing
//! down a difference, check whether the model was asked to describe the right
//! machine.
//!
//! # Running it
//!
//! ```text
//! scripts/fetch-testdata.sh riscv-arch-test
//! RSEMU_CONFORMANCE=1 cargo test --release --all-features --test conformance -- --nocapture riscv
//! ```
//!
//! `RSEMU_ARCH_TEST_ONLY` narrows a run to the tests whose name contains any
//! of a comma-separated list of substrings — `rv64-I/add,privilege`.

use std::fmt;
use std::path::{Path, PathBuf};

/// Where the fetch script links every test, and where this runner puts RAM.
pub(crate) const RAM_BASE: u64 = 0x8000_0000;

/// How much RAM a test gets.
///
/// The largest signature in the corpus ends just past `0x8009_5000`, so this
/// is a little over a hundred times what any test touches. It costs nothing:
/// `RamStore` allocates lazily and the runner builds one per test.
pub(crate) const RAM_SIZE: u64 = 16 << 20;

/// How many instructions a test may retire before it is called a hang.
///
/// The whole 181-test corpus retires 475 424 instructions between them, so no
/// single test comes within three orders of magnitude of this. A test that
/// reaches it has looped — which is what a mishandled trap looks like, since
/// the suite's handler returns to the faulting instruction when it cannot work
/// out how to step over it, and a `j .` spin is what `RVMODEL_HALT` leaves
/// behind if the completion store never lands.
pub(crate) const STEP_LIMIT: u64 = 20_000_000;

// ---------------------------------------------------------------------------
// What a runner reports
// ---------------------------------------------------------------------------

/// The addresses a linked test image advertises.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Layout {
    /// The entry point.
    pub(crate) entry: u64,
    /// Where `RVMODEL_HALT` stores its completion word.
    pub(crate) tohost: u64,
    /// The first byte of the signature.
    pub(crate) begin_signature: u64,
    /// One past the last byte of the signature.
    pub(crate) end_signature: u64,
}

/// What running one test image produced.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) enum Outcome {
    /// The test reached `RVMODEL_HALT`. The signature it left behind follows,
    /// one 32-bit little-endian word per element, low address first.
    Halted {
        /// The signature words, in address order.
        signature: Vec<u32>,
        /// How many instructions it took.
        instret: u64,
    },
    /// The step limit was reached with `tohost` still zero.
    Timeout {
        /// Where it was looping.
        pc: u64,
        /// `mcause` as the hart left it.
        mcause: u64,
        /// `mepc` as the hart left it.
        mepc: u64,
        /// `mtval` as the hart left it.
        mtval: u64,
    },
    /// The file is not a test image this runner can load.
    BadImage(String),
}

/// How much work a run actually did.
///
/// Two numbers with one job: proving the run was not vacuous. "181 of 181
/// signatures match" is equally true of 181 tests that halted on their first
/// instruction with an empty signature, and the suite asserts both of these
/// are non-zero rather than trusting the ratio.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(crate) struct Effort {
    /// Signature words compared against the reference.
    pub(crate) words: u64,
    /// Guest instructions retired.
    pub(crate) instret: u64,
}

impl Effort {
    /// Fold another test's effort in.
    pub(crate) fn add(&mut self, other: Effort) {
        self.words += other.words;
        self.instret += other.instret;
    }
}

/// A failure, in the shape the ledger and the report want.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Failure {
    /// The test's name, as the ledger keys it.
    pub(crate) test: String,
    /// What went wrong. May be several lines: a signature difference lists
    /// the words that differ.
    pub(crate) reason: String,
}

impl fmt::Display for Failure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.test, self.reason)
    }
}

// ---------------------------------------------------------------------------
// The seam.
// ---------------------------------------------------------------------------

/// A hart this runner can drive.
///
/// Deliberately two methods. Everything the suite needs — loading an ELF,
/// finding its symbols, running it, reading its signature back — happens on
/// the far side of [`Runner::run`], because all of it needs the core's own ELF
/// loader and address space and none of it exists in a build without
/// `cpu-riscv`. [`Runner::probe`] exists only so
/// `every_seam_this_build_can_bind_is_bound` has something it can *execute*:
/// a seam that constructs a hart and then drives nothing would still report a
/// clean run of zero tests.
pub(crate) trait Runner: Send + Sync {
    /// Load one linked test image, run it to `RVMODEL_HALT`, and report what
    /// happened along with the layout the image advertised.
    fn run(&self, image: &[u8], step_limit: u64) -> (Option<Layout>, Outcome);

    /// Execute `code` at [`RAM_BASE`] for `steps` instructions, then read back
    /// `x[reg]`. For the seam check, not for the suite.
    fn probe(&self, code: &[u8], steps: u64, reg: u32) -> u64;
}

/// The Cargo features that let this harness drive a RISC-V hart.
///
/// `std` for the same reason `cpu::CPU_FEATURES` names it: this binary runs
/// its suites on threads, and the `single` backend of `core::sync` is sound
/// only where a second thread cannot exist.
pub(crate) const CPU_FEATURES: &str = "cpu-riscv,std";

/// Are those features on for this build?
pub(crate) fn cpu_is_built() -> bool {
    cfg!(all(feature = "cpu-riscv", feature = "std"))
}

/// Construct a runner, or `None` if this build has no RISC-V core.
#[cfg(all(feature = "cpu-riscv", feature = "std"))]
pub(crate) fn new_runner() -> Option<Box<dyn Runner>> {
    Some(Box::new(adapter::Adapter))
}

/// No RISC-V core in this build.
#[cfg(not(all(feature = "cpu-riscv", feature = "std")))]
pub(crate) fn new_runner() -> Option<Box<dyn Runner>> {
    None
}

/// Is a runner available at all?
pub(crate) fn have_cpu() -> bool {
    new_runner().is_some()
}

/// A runner, or the *reason* there is none — and only one reason is allowed.
///
/// The same rule as `cpu::require_cpu`, for the same reason: "the corpus was
/// not fetched" and "nobody wired the adapter up" must never come out of this
/// directory looking alike. The first is a fact about the machine and stays a
/// skip; the second is a defect here, and the build already knows whether the
/// core exists, so it is asserted.
///
/// # Panics
///
/// If `cpu-riscv` is compiled in and [`new_runner`] still returns `None`.
pub(crate) fn require_runner() -> Result<Box<dyn Runner>, crate::harness::Skip> {
    match new_runner() {
        Some(runner) => Ok(runner),
        None => {
            assert!(
                !cpu_is_built(),
                "`{CPU_FEATURES}` are on but tests/conformance/riscv.rs binds no hart: \
                 riscv-arch-test would skip and pass while measuring nothing. \
                 Implement `new_runner`."
            );
            Err(crate::harness::Skip::NotBuilt {
                component: "a RISC-V hart",
                feature: CPU_FEATURES,
            })
        }
    }
}

// ---------------------------------------------------------------------------
// Corpus discovery
// ---------------------------------------------------------------------------

/// One test in the fetched corpus: an ELF and the reference signature for it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Test {
    /// The corpus directory and file stem, e.g. `rv64-I/add-01`. The ledger
    /// key and the report key.
    pub(crate) name: String,
    /// The built ELF.
    pub(crate) elf: PathBuf,
    /// The reference signature the model produced for it.
    pub(crate) reference: PathBuf,
}

/// Collect the corpus under `root`, sorted, so two runs diff cleanly.
///
/// An ELF with no reference beside it is returned all the same, with a
/// `reference` path that does not exist: a half-generated corpus must be
/// reported as broken rather than quietly shrink the denominator.
pub(crate) fn collect(root: &Path) -> Result<Vec<Test>, String> {
    let elf_root = root.join("elf");
    let ref_root = root.join("ref");
    let mut out = Vec::new();
    let suites = read_sorted(&elf_root)?;
    for suite in suites {
        if !suite.is_dir() {
            continue;
        }
        let suite_name = file_name(&suite);
        for elf in read_sorted(&suite)? {
            if elf.extension().and_then(|e| e.to_str()) != Some("elf") {
                continue;
            }
            let stem = elf
                .file_stem()
                .map(|s| s.to_string_lossy().into_owned())
                .unwrap_or_default();
            out.push(Test {
                name: format!("{suite_name}/{stem}"),
                reference: ref_root.join(&suite_name).join(format!("{stem}.sig")),
                elf,
            });
        }
    }
    out.sort_by(|a, b| a.name.cmp(&b.name));
    Ok(out)
}

fn file_name(path: &Path) -> String {
    path.file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_default()
}

fn read_sorted(dir: &Path) -> Result<Vec<PathBuf>, String> {
    let mut out: Vec<PathBuf> = std::fs::read_dir(dir)
        .map_err(|e| format!("{}: {e}", dir.display()))?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .collect();
    out.sort();
    Ok(out)
}

/// Parse a reference signature: one 32-bit word per line as hex, lowest
/// address first, which is what `--signature-granularity=4` produces.
///
/// This is the format the reference model writes and the format RISCOF diffs,
/// so it is the format kept on disk — parsing it here rather than converting
/// at fetch time keeps the corpus exactly as the model produced it, byte for
/// byte, which is the thing a reference is for.
pub(crate) fn parse_signature(text: &str) -> Result<Vec<u32>, String> {
    let mut out = Vec::new();
    for (i, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() {
            continue;
        }
        let word = u32::from_str_radix(line, 16)
            .map_err(|_| format!("line {}: {line:?} is not a 32-bit hex word", i + 1))?;
        out.push(word);
    }
    if out.is_empty() {
        return Err("the signature is empty".to_string());
    }
    Ok(out)
}

/// How many differing words a failure spells out before summarising.
///
/// A wrong instruction usually differs in one or two words and a wrong trap
/// path in a handful, so a dozen is enough to read the shape of the failure
/// off the report; a core that is wholesale wrong would fill the file.
const DIFF_LIMIT: usize = 12;

/// Compare a run against its reference and describe any difference.
pub(crate) fn compare(ours: &[u32], theirs: &[u32]) -> Option<String> {
    if ours.len() != theirs.len() {
        return Some(format!(
            "signature is {} word(s), the reference is {}",
            ours.len(),
            theirs.len()
        ));
    }
    let differing: Vec<(usize, u32, u32)> = ours
        .iter()
        .zip(theirs)
        .enumerate()
        .filter(|(_, (a, b))| a != b)
        .map(|(i, (a, b))| (i, *a, *b))
        .collect();
    if differing.is_empty() {
        return None;
    }
    // Word index *and* byte offset: the index is what the reference file is
    // numbered by and the offset is what a disassembly of the test's signature
    // stores is numbered by, and a diagnosis needs both.
    let mut s = format!(
        "{} of {} signature word(s) differ",
        differing.len(),
        ours.len()
    );
    for (at, got, want) in differing.iter().take(DIFF_LIMIT) {
        s.push_str(&format!(
            "\n    word {at} (begin_signature+{:#x}): {got:08x}, want {want:08x}",
            at * 4
        ));
    }
    if differing.len() > DIFF_LIMIT {
        s.push_str(&format!(
            "\n    ... and {} more",
            differing.len() - DIFF_LIMIT
        ));
    }
    Some(s)
}

// ---------------------------------------------------------------------------
// The adapter
// ---------------------------------------------------------------------------

/// Bridging `rsemu::cpu::riscv::Hart` onto [`Runner`].
///
/// A fresh hart, address space and `RamStore` per test. That is not thrift
/// being ignored: the suite's tests write page tables, program CSRs and take
/// traps, and reusing a hart between them would make one test's fallout the
/// next one's starting state — which is exactly the class of bug a conformance
/// run is supposed to find rather than create.
///
/// Nothing but RAM is mapped, and an access outside it faults rather than
/// reading zero. The reference model has a CLINT at `0x0200_0000`; no test in
/// the built corpus touches it (none of the interrupt tests are selected for
/// this hart), so mapping something there would be inventing a device to
/// answer a question nobody asks. If a future test does touch it, this runner
/// reports a bus fault rather than silently agreeing with a model that had a
/// timer.
#[cfg(all(feature = "cpu-riscv", feature = "std"))]
mod adapter {
    use std::sync::Arc;

    use rsemu::core::space::{AddressSpace, RamStore, Region, UnassignedPolicy};
    use rsemu::cpu::riscv::elf::Elf;
    use rsemu::cpu::riscv::{Config, Hart};

    use super::{Layout, Outcome, RAM_BASE, RAM_SIZE, Runner};

    /// The unit that owns the bridge. Stateless: every call builds its own
    /// machine, so one adapter can serve every worker thread.
    #[derive(Debug)]
    pub(super) struct Adapter;

    /// RAM, an address space over it, and a hart aimed at `entry`.
    ///
    /// `rv64` picks between the two configurations of the *same* core that
    /// `ROADMAP.md` §6 insists stay one core: RV64GC and RV32GC differ by a
    /// construction property and nothing else. The corpus carries both, so
    /// this runner has to as well — and the ELF class is the honest way to
    /// choose, since it is what the test was assembled for.
    fn machine(entry: u64, rv64: bool) -> (Arc<RamStore>, Hart) {
        let ram = Arc::new(RamStore::new(RAM_SIZE));
        let space = AddressSpace::new("mem", 64).with_unassigned(UnassignedPolicy::FAULT);
        space
            .topology()
            .map(Region::ram("ram", Arc::clone(&ram)), RAM_BASE)
            .expect("RAM fits a 64-bit space");
        let cfg = if rv64 {
            Config::rv64gc()
        } else {
            Config::rv32gc()
        };
        let hart = Hart::new(cfg.with_reset_vector(entry));
        hart.attach_space(Arc::new(space));
        (ram, hart)
    }

    /// Read `len` bytes of guest memory at `addr`.
    fn read(ram: &RamStore, addr: u64, len: u64) -> Vec<u8> {
        (0..len)
            .map(|i| ram.read_u8(addr - RAM_BASE + i).unwrap_or(0))
            .collect()
    }

    impl Runner for Adapter {
        fn run(&self, image: &[u8], step_limit: u64) -> (Option<Layout>, Outcome) {
            let elf = match Elf::parse(image) {
                Ok(elf) => elf,
                Err(e) => return (None, Outcome::BadImage(e.to_string())),
            };
            // All four are written by the DUT macros in
            // `scripts/riscv-arch-test/model_test.h`. An image without them is
            // not a test from this corpus, and guessing at where its signature
            // lives would be how a runner reports a confident wrong answer.
            let mut missing = Vec::new();
            let mut symbol = |name: &str| match elf.symbol(name) {
                Some(addr) => addr,
                None => {
                    missing.push(name.to_string());
                    0
                }
            };
            let layout = Layout {
                entry: elf.entry,
                tohost: symbol("tohost"),
                begin_signature: symbol("begin_signature"),
                end_signature: symbol("end_signature"),
            };
            if !missing.is_empty() {
                return (
                    None,
                    Outcome::BadImage(format!("no {} symbol(s)", missing.join(", "))),
                );
            }
            let top = RAM_BASE + RAM_SIZE;
            for (what, addr) in [
                ("tohost", layout.tohost),
                ("begin_signature", layout.begin_signature),
                ("end_signature", layout.end_signature),
            ] {
                if !(RAM_BASE..=top).contains(&addr) {
                    return (
                        Some(layout),
                        Outcome::BadImage(format!("{what} at {addr:#x} is outside RAM")),
                    );
                }
            }
            if layout.end_signature < layout.begin_signature {
                return (
                    Some(layout),
                    Outcome::BadImage("end_signature is below begin_signature".into()),
                );
            }

            let (ram, hart) = machine(layout.entry, elf.is_64);
            for segment in &elf.segments {
                if segment.addr < RAM_BASE || segment.addr + segment.mem_len > top {
                    return (
                        Some(layout),
                        Outcome::BadImage(format!(
                            "segment at {:#x} does not fit in RAM",
                            segment.addr
                        )),
                    );
                }
                let at = segment.addr - RAM_BASE;
                ram.write_at(at, &segment.bytes).expect("in range");
                // Past the file image is `.bss`, which must read as zero. A
                // fresh RamStore already does; doing it here means the test
                // depends on the loader rather than on that.
                if segment.mem_len > segment.bytes.len() as u64 {
                    ram.fill(
                        at + segment.bytes.len() as u64,
                        segment.mem_len - segment.bytes.len() as u64,
                        0,
                    )
                    .expect("in range");
                }
            }

            let tohost_offset = layout.tohost - RAM_BASE;
            let read_tohost = || {
                let mut v = 0u64;
                for k in 0..8 {
                    v |= u64::from(ram.read_u8(tohost_offset + k).unwrap_or(0)) << (8 * k);
                }
                v
            };

            let mut halted = false;
            for _ in 0..step_limit {
                hart.step();
                if read_tohost() != 0 {
                    halted = true;
                    break;
                }
            }
            if !halted {
                let csrs = hart.csrs();
                return (
                    Some(layout),
                    Outcome::Timeout {
                        pc: hart.pc(),
                        mcause: csrs.mcause,
                        mepc: csrs.mepc,
                        mtval: csrs.mtval,
                    },
                );
            }

            // The signature is compared a 32-bit word at a time, which is how
            // the reference model writes it. A region that is not a whole
            // number of words is padded with zero at the top, exactly as the
            // model's `--signature-granularity=4` does, so the two line up.
            let len = layout.end_signature - layout.begin_signature;
            let padded = len.div_ceil(4) * 4;
            let bytes = read(&ram, layout.begin_signature, padded);
            let signature = bytes
                .as_chunks::<4>()
                .0
                .iter()
                .map(|w| u32::from_le_bytes(*w))
                .collect();
            (
                Some(layout),
                Outcome::Halted {
                    signature,
                    instret: hart.instret(),
                },
            )
        }

        fn probe(&self, code: &[u8], steps: u64, reg: u32) -> u64 {
            let (ram, hart) = machine(RAM_BASE, true);
            ram.write_at(0, code).expect("in range");
            for _ in 0..steps {
                hart.step();
            }
            hart.x(reg)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_signature_parses() {
        let sig = parse_signature("deadbeef\n00000000\n\n0000002a\n").unwrap();
        assert_eq!(sig, [0xdead_beef, 0, 0x2a]);
    }

    #[test]
    fn a_malformed_signature_is_rejected() {
        assert!(parse_signature("not hex\n").is_err());
        // Empty is a broken corpus, not a zero-length signature: every test in
        // this suite writes at least the canary.
        assert!(parse_signature("\n\n").is_err());
    }

    #[test]
    fn an_identical_signature_compares_clean() {
        assert_eq!(compare(&[1, 2, 3], &[1, 2, 3]), None);
    }

    #[test]
    fn a_difference_names_every_word_up_to_the_limit() {
        let why = compare(&[1, 9, 9], &[1, 2, 3]).expect("differs");
        assert!(why.contains("2 of 3"), "{why}");
        assert!(why.contains("word 1 (begin_signature+0x4)"), "{why}");
        assert!(why.contains("word 2 (begin_signature+0x8)"), "{why}");
        assert!(why.contains("00000009, want 00000002"), "{why}");
    }

    #[test]
    fn a_wholesale_difference_is_summarised_rather_than_dumped() {
        let ours: Vec<u32> = (0..100).collect();
        let theirs: Vec<u32> = (0..100).map(|w| w + 1).collect();
        let why = compare(&ours, &theirs).expect("differs");
        assert!(why.contains("100 of 100"), "{why}");
        assert_eq!(why.matches("want").count(), DIFF_LIMIT, "{why}");
        assert!(
            why.contains(&format!("and {} more", 100 - DIFF_LIMIT)),
            "{why}"
        );
    }

    #[test]
    fn a_length_difference_is_reported_as_one() {
        let why = compare(&[1], &[1, 2]).expect("differs");
        assert!(why.contains("1 word(s), the reference is 2"), "{why}");
    }
}
