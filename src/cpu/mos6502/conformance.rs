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
//! git clone --depth 1 https://github.com/SingleStepTests/65x02 /tmp/65x02
//! RSEMU_65X02_DIR=/tmp/65x02/6502/v1 cargo test --all-features conformance -- --nocapture
//! ```
//!
//! Point it at `6502/v1` for a part with decimal mode, or `nes6502/v1` for the
//! RP2A03; the runner picks the configuration from the directory name.
//!
//! Without the variable the test prints why it did nothing and passes, so
//! `cargo test` stays hermetic and offline.
//!
//! # The ledger
//!
//! `ROADMAP.md` §0 asks every core to ship a known-failures ledger that only
//! ever shrinks. This one is **empty**: at the commit that added this file,
//! all 256 opcode files of `6502/v1` passed all 10 000 vectors each —
//! registers, memory *and* the full bus trace, 2 560 000 vectors — and the
//! decimal-sensitive subset of `nes6502/v1` passed against
//! [`Config::RP2A03`]. Anything that fails later is a regression, not a known
//! gap, and belongs in a fix rather than in a list.
//!
//! Not covered by the corpus, and therefore only by the hand-written tests
//! next door: the RESET sequence, IRQ, NMI, and the BRK/IRQ/NMI hijack. The
//! vectors start mid-program with the interrupt lines idle.
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

        let mut space = AddressSpace::new("cpu", 16).with_unassigned(UnassignedPolicy::FAULT);
        space
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

/// Run the whole corpus, or explain why it did not.
///
/// Not `#[ignore]`d: a skipped test that says nothing is how a suite quietly
/// stops running. This one prints the command that would have run it.
#[test]
fn single_step_tests() {
    let Ok(dir) = std::env::var("RSEMU_65X02_DIR") else {
        println!(
            "conformance: set RSEMU_65X02_DIR to a SingleStepTests/65x02 \
             directory (6502/v1 or nes6502/v1) to run 10 000 vectors per opcode"
        );
        return;
    };
    let dir = Path::new(&dir);
    // The NES's core has no decimal mode, and the corpus has a directory per
    // part; picking the configuration from the path is what makes both
    // runnable from one test.
    let cfg = if dir.to_string_lossy().contains("nes") {
        Config::RP2A03
    } else {
        Config::NMOS_6502
    };

    let only = std::env::var("RSEMU_65X02_OPCODES").ok();
    let mut ran = 0usize;
    let mut failed_opcodes = Vec::new();
    let mut total_failures = 0usize;

    for opcode in 0..=255u8 {
        let name = format!("{opcode:02x}");
        if let Some(only) = &only
            && !only.split(',').any(|o| o.trim() == name)
        {
            continue;
        }
        let path = dir.join(format!("{name}.json"));
        if !path.exists() {
            continue;
        }
        let failures = run_file(&path, cfg);
        ran += 1;
        if !failures.is_empty() {
            total_failures += failures.len();
            failed_opcodes.push((name.clone(), failures.len()));
            let insn = super::isa::decode(opcode);
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

    assert!(ran > 0, "no NN.json files under {}", dir.display());
    println!("conformance: {ran} opcode files, {total_failures} failing vectors");
    assert!(
        failed_opcodes.is_empty(),
        "failing opcodes: {failed_opcodes:?}"
    );
}
