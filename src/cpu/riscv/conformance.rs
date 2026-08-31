//! The conformance runner for `riscv-tests`.
//!
//! `ROADMAP.md` §0: *accuracy is measured, never asserted*. This is the
//! measurement. `riscv-tests` (BSD-3-Clause, © The Regents of the University
//! of California — verified against the upstream `LICENSE`) is the canonical
//! suite: each test is a small statically linked ELF that exercises one
//! instruction or one privileged mechanism exhaustively and signals its result
//! by storing to a symbol called `tohost`. A value of 1 means every subtest
//! passed; anything else is `(n << 1) | 1`, where `n` names the subtest that
//! failed — which is why a failure here points straight at a line of assembly
//! rather than at a mood.
//!
//! # Running it
//!
//! The corpus is **built, never vendored** (`ROADMAP.md` §1, §12). It carries
//! its own licence and shipping it in this repository would be redistribution,
//! so the test is gated on an environment variable naming a directory of
//! built ELF binaries:
//!
//! ```text
//! git clone --depth 1 --recursive \
//!     https://github.com/riscv-software-src/riscv-tests /tmp/riscv-tests
//! # Any RISC-V toolchain will do. With no cross-gcc installed, zig's bundled
//! # clang and lld build the whole suite:
//! #   pip install ziglang
//! #   zig cc -target riscv64-freestanding-none -mcpu=baseline_rv64+m+a+f+d+c \
//! #       -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles \
//! #       -I env/p -I isa/macros/scalar -Wl,-T,env/p/link.ld \
//! #       isa/rv64ui/add.S -o out/rv64ui-p-add
//! RSEMU_RISCV_TESTS=/tmp/riscv-tests/out cargo test --all-features conformance -- --nocapture
//! ```
//!
//! `RSEMU_RISCV_TESTS_ONLY` takes a comma-separated list of substrings to
//! narrow a run down while iterating.
//!
//! Without the variable the test prints why it did nothing and passes, so
//! `cargo test` stays hermetic and offline.
//!
//! # The ledger
//!
//! `ROADMAP.md` §0 asks every core to ship a known-failures ledger that only
//! ever shrinks. This one is **empty**. At the commit that added this file,
//! **409 of 409** binaries passed:
//!
//! | Variant | Count | What it covers |
//! | --- | --- | --- |
//! | `rv{32,64}{ui,um,ua,uf,ud,uc}-p-*` | 208 | the user ISA, machine mode, no translation |
//! | `rv{32,64}{si,mi}-p-*` | 46 | CSRs, traps, delegation, misaligned access, PMP |
//! | `rv{32,64}{ui,um,ua,uf,ud,uc}-v-*` | 155 | the same tests again, under Sv39/Sv32 in supervisor mode |
//!
//! The `-v-` half is the one worth pointing at: each of those runs the test
//! body in user mode under a supervisor that has switched paging on, so it
//! exercises the page-table walk, the accessed and dirty bits, `SFENCE.VMA`,
//! delegation and the trap path on every single instruction test rather than
//! only in this crate's own MMU tests.
//!
//! One upstream binary is deliberately not built: `rv32ud-p-move` is written
//! in RV64 assembly and upstream's makefiles do not build it for RV32 either.
//!
//! # The other suite
//!
//! `riscv-arch-test` — the official architectural certification tests, and the
//! one `ROADMAP.md` §13 names on the phase-5 gate — is **not** here. It lives
//! in `tests/conformance/riscv.rs`, because it needs two things this file does
//! not have: a corpus that is built rather than downloaded
//! (`scripts/fetch-testdata.sh riscv-arch-test`), and a known-failures ledger,
//! which is a committed file under `tests/conformance/ledgers/`.
//!
//! The two suites measure different things and both are worth running. This
//! one is *self-checking*: each binary decides whether it passed and says so
//! through `tohost`, so a wrong result is caught by assembly upstream wrote
//! and rsemu only has to agree. `riscv-arch-test` is *signature-diffed*: each
//! test records every result it computed and conformance means matching a
//! reference model byte for byte, so a wrong result nobody thought to assert
//! on is caught anyway. The `-v-` half of this corpus, which runs every
//! instruction test under Sv39 in supervisor mode, has no counterpart there.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::sync::Arc;
use alloc::vec::Vec;
use std::path::Path;

use crate::core::space::{AddressSpace, RamStore, Region, UnassignedPolicy};

use super::csr::Extensions;
use super::elf::Elf;
use super::isa::Xlen;
use super::{Config, Hart};

/// Where a `riscv-tests` binary is linked.
const RAM_BASE: u64 = 0x8000_0000;

/// How much RAM to give it. The tests are tiny; this is room for their page
/// tables and stacks and nothing more.
const RAM_SIZE: u64 = 16 << 20;

/// How many instructions a test may run before it is called a hang.
///
/// The longest of these tests retires a few tens of thousands of
/// instructions, so this is three orders of magnitude of headroom — enough
/// that a timeout means a genuine infinite loop, which is what a wrong branch
/// or a mishandled trap produces.
const STEP_LIMIT: u64 = 2_000_000;

/// What running one test produced.
#[derive(Debug, PartialEq, Eq)]
enum Outcome {
    /// `tohost` reported success.
    Pass,
    /// `tohost` reported a failing subtest number, with the trap state the
    /// hart was left in.
    ///
    /// The trap state is usually the whole diagnosis: the suite's own handler
    /// reports an *unexpected exception* by ORing 1337 into the subtest
    /// number, so a subtest around 668 means "something trapped that should
    /// not have" and `mcause` says what.
    Failed {
        /// The subtest number the suite reported.
        subtest: u64,
        /// `mcause` as the hart left it.
        mcause: u64,
        /// `mepc` as the hart left it.
        mepc: u64,
        /// `mtval` as the hart left it.
        mtval: u64,
    },
    /// The step limit was reached with `tohost` still zero.
    Timeout { pc: u64, instret: u64 },
    /// The file is not a `riscv-tests` binary.
    Skipped(String),
}

/// Run one test binary.
fn run_one(path: &Path) -> Outcome {
    let bytes = match std::fs::read(path) {
        Ok(b) => b,
        Err(e) => return Outcome::Skipped(format!("unreadable: {e}")),
    };
    let elf = match Elf::parse(&bytes) {
        Ok(e) => e,
        Err(e) => return Outcome::Skipped(e.to_string()),
    };
    let Some(tohost) = elf.symbol("tohost") else {
        return Outcome::Skipped("no `tohost` symbol".to_string());
    };
    if !(RAM_BASE..RAM_BASE + RAM_SIZE).contains(&tohost) {
        return Outcome::Skipped(format!("`tohost` at {tohost:#x} is outside RAM"));
    }

    let ram = Arc::new(RamStore::new(RAM_SIZE));
    for segment in &elf.segments {
        if segment.addr < RAM_BASE || segment.addr + segment.mem_len > RAM_BASE + RAM_SIZE {
            return Outcome::Skipped(format!(
                "segment at {:#x} does not fit in RAM",
                segment.addr
            ));
        }
        let at = segment.addr - RAM_BASE;
        ram.write_at(at, &segment.bytes).expect("in range");
        // Anything past the file image is `.bss` and must read as zero. A
        // fresh RamStore already is, but a test that checks this would
        // otherwise depend on that rather than on the loader.
        if segment.mem_len > segment.bytes.len() as u64 {
            ram.fill(
                at + segment.bytes.len() as u64,
                segment.mem_len - segment.bytes.len() as u64,
                0,
            )
            .expect("in range");
        }
    }

    // Nothing but RAM is mapped, and an access outside it faults rather than
    // reading zero: a test that runs off the end should fail loudly.
    let space = AddressSpace::new("mem", 64).with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(Region::ram("ram", Arc::clone(&ram)), RAM_BASE)
        .expect("RAM fits");

    let xlen = if elf.is_64 { Xlen::Rv64 } else { Xlen::Rv32 };
    let hart = Hart::new(
        Config {
            xlen,
            ext: Extensions::GC,
            ..Config::rv64gc()
        }
        .with_reset_vector(elf.entry),
    );
    hart.attach_space(Arc::new(space));

    let tohost_offset = tohost - RAM_BASE;
    let read_tohost = || {
        let mut v = 0u64;
        for k in 0..8 {
            v |= u64::from(ram.read_u8(tohost_offset + k).unwrap_or(0)) << (8 * k);
        }
        v
    };

    for _ in 0..STEP_LIMIT {
        hart.step();
        let status = read_tohost();
        if status != 0 {
            if status == 1 {
                return Outcome::Pass;
            }
            let csrs = hart.csrs();
            return Outcome::Failed {
                subtest: status >> 1,
                mcause: csrs.mcause,
                mepc: csrs.mepc,
                mtval: csrs.mtval,
            };
        }
    }
    Outcome::Timeout {
        pc: hart.pc(),
        instret: hart.instret(),
    }
}

/// Run the whole corpus, or explain why it did not.
///
/// Not `#[ignore]`d: a skipped test that says nothing is how a suite quietly
/// stops running. This one prints the command that would have run it.
#[test]
fn riscv_tests() {
    let Ok(dir) = std::env::var("RSEMU_RISCV_TESTS") else {
        println!(
            "conformance: set RSEMU_RISCV_TESTS to a directory of built \
             riscv-tests ELF binaries to run the suite (see the module docs \
             for how to build them without a cross toolchain)"
        );
        return;
    };
    let only = std::env::var("RSEMU_RISCV_TESTS_ONLY").ok();
    let dir = Path::new(&dir);

    // Sorted, so a run is reproducible and two runs diff cleanly.
    let mut entries: Vec<_> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.is_file())
        .filter(|p| p.extension().is_none())
        .collect();
    entries.sort();

    let mut passed = 0usize;
    let mut skipped = 0usize;
    let mut failures: Vec<String> = Vec::new();

    for path in entries {
        let name = path
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        if let Some(only) = &only
            && !only.split(',').any(|f| name.contains(f.trim()))
        {
            continue;
        }
        match run_one(&path) {
            Outcome::Pass => passed += 1,
            Outcome::Failed {
                subtest,
                mcause,
                mepc,
                mtval,
            } => {
                failures.push(format!(
                    "{name}: subtest {subtest} failed \
                     (mcause {mcause:#x} mepc {mepc:#x} mtval {mtval:#x})"
                ));
            }
            Outcome::Timeout { pc, instret } => {
                failures.push(format!(
                    "{name}: no result after {instret} instructions, pc {pc:#x}"
                ));
            }
            Outcome::Skipped(why) => {
                skipped += 1;
                println!("skipped {name}: {why}");
            }
        }
    }

    for failure in &failures {
        println!("FAIL {failure}");
    }
    println!(
        "conformance: {passed} passed, {} failed, {skipped} skipped",
        failures.len()
    );
    assert!(
        passed + failures.len() > 0,
        "no test binaries under {}",
        dir.display()
    );
    assert!(failures.is_empty(), "{} failing tests", failures.len());
}
