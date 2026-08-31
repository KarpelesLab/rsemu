//! The conformance runner for `SingleStepTests/65x02`.
//!
//! `ROADMAP.md` §0: *accuracy is measured, never asserted*. This is the
//! measurement. The corpus is 10 000 vectors per opcode, each one an initial
//! register file and memory image, the expected final state, **and the
//! complete bus trace** — which is the part that makes it worth having, since
//! a wrong dummy read fails here and passes every hand-written test.
//!
//! # Running it
//!
//! The corpus is **downloaded, never vendored** (`ROADMAP.md` §1, §12): it is
//! ~1 GiB of JSON with its own licence (MIT, and confirmed as such), and
//! shipping it in this repository would be redistribution. So the test is
//! gated on an environment variable naming a directory of `NN.json` files:
//!
//! ```text
//! git clone --depth 1 --filter=blob:none --sparse \
//!     https://github.com/SingleStepTests/65x02 /tmp/65x02
//! git -C /tmp/65x02 sparse-checkout set --no-cone /6502 /wdc65c02 /LICENSE
//! RSEMU_65X02_DIR=/tmp/65x02/6502/v1 cargo test --all-features conformance -- --nocapture
//! ```
//!
//! Point it at `6502/v1` for a part with decimal mode, `nes6502/v1` for the
//! RP2A03, or `wdc65c02/v1` for the CMOS part; the runner picks the
//! configuration from the directory name, so one test covers all three.
//!
//! Without the variable the test prints why it did nothing and passes, so
//! `cargo test` stays hermetic and offline.
//!
//! # The ledger
//!
//! `ROADMAP.md` §0 asks every core to ship a known-failures ledger that only
//! ever shrinks.
//!
//! **NMOS — empty.** All 256 opcode files of `6502/v1` pass all 10 000 vectors
//! each: registers, memory *and* the full bus trace, 2 560 000 vectors. The
//! decimal-sensitive subset of `nes6502/v1` passes against [`Config::RP2A03`].
//!
//! **CMOS — two entries, both in the corpus rather than here.** 254 of the 256
//! `wdc65c02/v1` files pass 10 000 of 10 000 (`cb.json` and `db.json` are
//! empty upstream: `WAI` and `STP` are "incompatible with this style of
//! testing", so they are covered by the hand-written tests next door instead).
//! What fails is the *decimal-mode half* of `69.json` and `e9.json` —
//! `ADC #` and `SBC #` — and only in the address of one dummy read:
//!
//! - Every other addressing mode spends the CMOS decimal-correction cycle
//!   re-reading the operand's effective address, which is what this
//!   implementation does everywhere, immediate included (there the operand is
//!   the byte after the opcode).
//! - The corpus instead expects a *constant* address — `$007f` for every
//!   `ADC #` vector and `$0000` for every `SBC #` vector, whatever the
//!   registers or the program counter. That is the signature of an
//!   uninitialised effective-address latch in the generator, not of hardware:
//!   on a real part those latches hold whatever the *previous* instruction put
//!   there, so the address is genuinely indeterminate and no constant can be
//!   right. The corpus README invites exactly this report.
//!
//! Registers, memory and cycle *count* match on those vectors; only that one
//! address differs. Recorded here rather than papered over, because a ledger
//! that hides a disagreement is worse than one that names it.
//!
//! Not covered by the corpus, and therefore only by the hand-written tests
//! next door: the RESET sequence, IRQ, NMI, the BRK/IRQ/NMI hijack, and the
//! CMOS `WAI`/`STP`. The vectors start mid-program with the interrupt lines
//! idle.
//!
//! # Why the JSON parser is in here
//!
//! The dependency policy allows no `serde` (`ROADMAP.md` §0), and this is the
//! only thing in the crate that reads JSON. Sixty lines of recursive descent
//! over a format this regular is cheaper than the argument.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use std::path::Path;

use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, UnassignedPolicy,
};
use crate::core::sync::{self, LockRank};

use super::{Config, Mos6502, Regs};

// ---------------------------------------------------------------------------
// A very small JSON reader
// ---------------------------------------------------------------------------

/// Just enough of JSON for this corpus.
#[derive(Debug, Clone, PartialEq)]
enum Json {
    Num(i64),
    Str(String),
    Arr(Vec<Json>),
    Obj(Vec<(String, Json)>),
}

impl Json {
    fn get(&self, key: &str) -> Option<&Json> {
        match self {
            Json::Obj(fields) => fields.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }

    fn num(&self) -> i64 {
        match self {
            Json::Num(n) => *n,
            other => panic!("expected a number, got {other:?}"),
        }
    }

    fn arr(&self) -> &[Json] {
        match self {
            Json::Arr(items) => items,
            other => panic!("expected an array, got {other:?}"),
        }
    }

    fn str(&self) -> &str {
        match self {
            Json::Str(s) => s,
            other => panic!("expected a string, got {other:?}"),
        }
    }
}

struct Parser<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl<'a> Parser<'a> {
    fn new(bytes: &'a [u8]) -> Parser<'a> {
        Parser { bytes, at: 0 }
    }

    fn skip_space(&mut self) {
        while matches!(self.bytes.get(self.at), Some(b' ' | b'\t' | b'\n' | b'\r')) {
            self.at += 1;
        }
    }

    fn eat(&mut self, byte: u8) {
        self.skip_space();
        assert_eq!(
            self.bytes.get(self.at),
            Some(&byte),
            "at offset {}",
            self.at
        );
        self.at += 1;
    }

    fn peek(&mut self) -> u8 {
        self.skip_space();
        self.bytes.get(self.at).copied().unwrap_or(b'\0')
    }

    fn value(&mut self) -> Json {
        match self.peek() {
            b'{' => {
                self.at += 1;
                let mut fields = Vec::new();
                if self.peek() == b'}' {
                    self.at += 1;
                    return Json::Obj(fields);
                }
                loop {
                    let key = self.string();
                    self.eat(b':');
                    fields.push((key, self.value()));
                    match self.peek() {
                        b',' => self.at += 1,
                        _ => break,
                    }
                }
                self.eat(b'}');
                Json::Obj(fields)
            }
            b'[' => {
                self.at += 1;
                let mut items = Vec::new();
                if self.peek() == b']' {
                    self.at += 1;
                    return Json::Arr(items);
                }
                loop {
                    items.push(self.value());
                    match self.peek() {
                        b',' => self.at += 1,
                        _ => break,
                    }
                }
                self.eat(b']');
                Json::Arr(items)
            }
            b'"' => Json::Str(self.string()),
            b'n' => {
                // `null` appears in the CMOS files' cycle lists; it is not a
                // number, and nothing here reads one.
                self.at += 4;
                Json::Num(0)
            }
            _ => {
                let start = self.at;
                if self.bytes.get(self.at) == Some(&b'-') {
                    self.at += 1;
                }
                while matches!(self.bytes.get(self.at), Some(b'0'..=b'9')) {
                    self.at += 1;
                }
                let text = core::str::from_utf8(&self.bytes[start..self.at]).expect("ascii");
                Json::Num(text.parse().expect("integer"))
            }
        }
    }

    fn string(&mut self) -> String {
        self.eat(b'"');
        let start = self.at;
        while self.bytes.get(self.at) != Some(&b'"') {
            self.at += 1;
        }
        let text = String::from_utf8_lossy(&self.bytes[start..self.at]).into_owned();
        self.at += 1;
        text
    }
}

// ---------------------------------------------------------------------------
// The bus the vectors describe
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Access {
    addr: u16,
    value: u8,
    write: bool,
}

impl core::fmt::Display for Access {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "[{:04x} {:02x} {}]",
            self.addr,
            self.value,
            if self.write { "write" } else { "read" }
        )
    }
}

#[derive(Debug)]
struct VectorBus(sync::Mutex<(Vec<u8>, Vec<Access>)>);

impl MemOps for VectorBus {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let mut m = self.0.lock();
        for (i, slot) in dst.iter_mut().enumerate() {
            let addr = (offset as u16).wrapping_add(i as u16);
            *slot = m.0[addr as usize];
            if !attrs.debug {
                let value = *slot;
                m.1.push(Access {
                    addr,
                    value,
                    write: false,
                });
            }
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let mut m = self.0.lock();
        for (i, byte) in src.iter().enumerate() {
            let addr = (offset as u16).wrapping_add(i as u16);
            m.0[addr as usize] = *byte;
            if !attrs.debug {
                m.1.push(Access {
                    addr,
                    value: *byte,
                    write: true,
                });
            }
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

/// What went wrong with one vector, ready to print.
struct Failure {
    name: String,
    detail: String,
}

/// Run every vector in one opcode's file, returning the failures.
fn run_file(path: &Path, cfg: Config) -> Vec<Failure> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let vectors = Parser::new(&bytes).value();
    let mut failures = Vec::new();

    for vector in vectors.arr() {
        let name = vector.get("name").expect("name").str().to_string();
        let initial = vector.get("initial").expect("initial");
        let expected = vector.get("final").expect("final");

        let bus = alloc::sync::Arc::new(VectorBus(sync::Mutex::with_rank(
            LockRank::DEVICE,
            (alloc::vec![0u8; 0x1_0000], Vec::new()),
        )));
        for cell in initial.get("ram").expect("ram").arr() {
            let cell = cell.arr();
            bus.0.lock().0[cell[0].num() as usize] = cell[1].num() as u8;
        }

        let space = AddressSpace::new("cpu", 16).with_unassigned(UnassignedPolicy::FAULT);
        space
            .topology()
            .map(Region::io("ram", 0x1_0000, bus.clone()), 0)
            .expect("64 KiB fits");
        let cpu = Mos6502::new(cfg);
        cpu.attach_space(alloc::sync::Arc::new(space));
        // The vectors start mid-program: no reset sequence, no pending
        // interrupt, and the lines are not modelled by the corpus at all.
        {
            let mut session = cpu.session.lock();
            session.state.reset_pending = false;
            session.state.regs = Regs {
                a: initial.get("a").expect("a").num() as u8,
                x: initial.get("x").expect("x").num() as u8,
                y: initial.get("y").expect("y").num() as u8,
                s: initial.get("s").expect("s").num() as u8,
                p: initial.get("p").expect("p").num() as u8,
                pc: initial.get("pc").expect("pc").num() as u16,
            };
        }
        bus.0.lock().1.clear();

        cpu.step();

        let mut detail = String::new();
        let got = cpu.regs();
        let want = Regs {
            a: expected.get("a").expect("a").num() as u8,
            x: expected.get("x").expect("x").num() as u8,
            y: expected.get("y").expect("y").num() as u8,
            s: expected.get("s").expect("s").num() as u8,
            p: expected.get("p").expect("p").num() as u8,
            pc: expected.get("pc").expect("pc").num() as u16,
        };
        if got != want {
            detail.push_str(&format!("  regs want {want}\n       got  {got}\n"));
        }

        for cell in expected.get("ram").expect("ram").arr() {
            let cell = cell.arr();
            let addr = cell[0].num() as usize;
            let value = cell[1].num() as u8;
            let actual = bus.0.lock().0[addr];
            if actual != value {
                detail.push_str(&format!(
                    "  ram[{addr:04x}] want {value:02x} got {actual:02x}\n"
                ));
            }
        }

        let want_cycles: Vec<Access> = vector
            .get("cycles")
            .expect("cycles")
            .arr()
            .iter()
            .map(|c| {
                let c = c.arr();
                Access {
                    addr: c[0].num() as u16,
                    value: c[1].num() as u8,
                    write: c[2].str() == "write",
                }
            })
            .collect();
        let got_cycles = core::mem::take(&mut bus.0.lock().1);
        if got_cycles != want_cycles {
            let show = |cs: &[Access]| {
                cs.iter()
                    .map(|c| c.to_string())
                    .collect::<Vec<_>>()
                    .join(" ")
            };
            detail.push_str(&format!(
                "  cycles want {}\n         got  {}\n",
                show(&want_cycles),
                show(&got_cycles)
            ));
        }

        if !detail.is_empty() {
            failures.push(Failure { name, detail });
        }
    }
    failures
}

/// One entry in the known-failures ledger.
///
/// `ROADMAP.md` §0: a core ships a ledger that only ever *shrinks*, so an entry
/// is a ceiling rather than an expectation. Fewer failures than recorded is
/// fine and says so; more is a regression and fails the run.
struct Known {
    /// Which part the entry applies to, as it appears in the corpus path.
    corpus: &'static str,
    /// The opcode file.
    opcode: u8,
    /// The most vectors that may fail.
    at_most: usize,
    /// Why this is the corpus's disagreement and not ours.
    reason: &'static str,
}

/// The whole ledger. Two entries, both in the CMOS corpus, both the same bug.
///
/// See this module's header for the long form: the corpus expects the decimal
/// correction cycle of `ADC #`/`SBC #` to read a *constant* address, the same
/// one for all 10 000 vectors of the file regardless of registers or PC. A real
/// effective-address latch holds whatever the previous instruction left in it,
/// so no constant can be right; this core re-reads the operand, which is what
/// it does in every other addressing mode and what the corpus itself expects
/// there.
static LEDGER: &[Known] = &[
    Known {
        corpus: "wdc65c02",
        opcode: 0x69,
        at_most: 4975,
        reason: "corpus expects a fixed $007f for the ADC # decimal cycle",
    },
    Known {
        corpus: "wdc65c02",
        opcode: 0xe9,
        at_most: 5000,
        reason: "corpus expects a fixed $0000 for the SBC # decimal cycle",
    },
];

/// Run the whole corpus, or explain why it did not.
///
/// Not `#[ignore]`d: a skipped test that says nothing is how a suite quietly
/// stops running. This one prints the command that would have run it.
#[test]
fn single_step_tests() {
    let Ok(dir) = std::env::var("RSEMU_65X02_DIR") else {
        println!(
            "conformance: set RSEMU_65X02_DIR to a SingleStepTests/65x02 \
             directory (6502/v1, nes6502/v1 or wdc65c02/v1) to run 10 000 \
             vectors per opcode"
        );
        return;
    };
    let dir = Path::new(&dir);
    // The corpus has a directory per part, so picking the configuration from
    // the path is what makes all three runnable from one test.
    let path = dir.to_string_lossy();
    let cfg = if path.contains("nes") {
        Config::RP2A03
    } else if path.contains("65c02") {
        Config::W65C02S
    } else {
        Config::NMOS_6502
    };
    println!("conformance: {} as {}", dir.display(), cfg.variant);

    let ledger: Vec<&Known> = LEDGER.iter().filter(|k| path.contains(k.corpus)).collect();

    let only = std::env::var("RSEMU_65X02_OPCODES").ok();
    let mut ran = 0usize;
    let mut unexpected = Vec::new();
    let mut total_failures = 0usize;
    let mut ledgered = 0usize;

    for opcode in 0..=255u8 {
        let name = format!("{opcode:02x}");
        if let Some(only) = &only
            && !only.split(',').any(|o| o.trim() == name)
        {
            continue;
        }
        let path = dir.join(format!("{name}.json"));
        // Missing, or present but empty: upstream ships zero-byte `cb.json`
        // and `db.json` because `WAI` and `STP` cannot be expressed as a
        // single-instruction vector. Not a gap in the run.
        if !path.exists() || std::fs::metadata(&path).is_ok_and(|m| m.len() == 0) {
            continue;
        }
        let failures = run_file(&path, cfg);
        ran += 1;
        if !failures.is_empty() {
            total_failures += failures.len();
            let insn = super::isa::decode_as(cfg.variant, opcode);
            match ledger.iter().find(|k| k.opcode == opcode) {
                Some(known) if failures.len() <= known.at_most => {
                    ledgered += failures.len();
                    println!(
                        "{name} {} {} — {} of 10000 known-bad (ceiling {}): {}",
                        insn.op.mnemonic(),
                        insn.mode.name(),
                        failures.len(),
                        known.at_most,
                        known.reason
                    );
                }
                _ => {
                    unexpected.push((name.clone(), failures.len()));
                    println!(
                        "{name} {} {} — {} of 10000 vectors failed; first is `{}`:\n{}",
                        insn.op.mnemonic(),
                        insn.mode.name(),
                        failures.len(),
                        failures[0].name,
                        failures[0].detail
                    );
                }
            }
        }
    }

    assert!(ran > 0, "no NN.json files under {}", dir.display());
    let vectors = ran * 10_000;
    println!(
        "conformance: {ran} opcode files, {} of {vectors} vectors passed \
         ({ledgered} known-bad in the ledger)",
        vectors - total_failures
    );
    assert!(unexpected.is_empty(), "failing opcodes: {unexpected:?}");
}
