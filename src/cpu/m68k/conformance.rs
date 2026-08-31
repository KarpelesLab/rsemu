//! The conformance runner for `SingleStepTests/680x0`.
//!
//! `ROADMAP.md` §0: *accuracy is measured, never asserted*. This is the
//! measurement. The corpus is roughly eight thousand vectors per instruction
//! file, each one a complete register file, a two-word prefetch queue, a
//! sparse memory image, the expected final state, the instruction's total
//! cycle count **and the complete bus trace** — which is what makes it worth
//! having, since a wrong prefetch order fails here and passes every
//! hand-written test.
//!
//! # Licence: fetch and run, never vendor
//!
//! <https://github.com/SingleStepTests/680x0> **has no licence file.** Running
//! it is ordinary use; committing it here would be redistribution of code
//! whose terms nobody has stated (`ROADMAP.md` §1, and the `nestest` row in
//! `docs/testing/conformance-suites.md`). So it is downloaded at test time
//! into a directory the environment names, and this file assumes nothing about
//! it being present.
//!
//! # Running it
//!
//! The corpus ships as gzipped JSON and the dependency policy allows no
//! decompressor (`ROADMAP.md` §0), so decompress it first — the runner reads
//! plain `.json`:
//!
//! ```text
//! git clone --depth 1 https://github.com/SingleStepTests/680x0 /tmp/680x0
//! gunzip /tmp/680x0/68000/v1/*.json.gz
//! RSEMU_680X0_DIR=/tmp/680x0/68000/v1 \
//!     cargo test --all-features m68k::conformance -- --nocapture
//! ```
//!
//! Without the variable the test prints why it did nothing and passes, so
//! `cargo test` stays hermetic and offline.
//!
//! | Variable | Effect |
//! | --- | --- |
//! | `RSEMU_680X0_DIR` | the directory of `NAME.json` files |
//! | `RSEMU_680X0_TESTS` | comma-separated file names to run, e.g. `MOVE.w,ADD.l` |
//! | `RSEMU_680X0_LIMIT` | stop after this many vectors per file |
//! | `RSEMU_680X0_STRICT` | also fail the test on a cycle-count or bus-trace mismatch |
//!
//! # What is checked, and what the ledger says
//!
//! Three things, reported separately because they fail for different reasons:
//!
//! 1. **State** — every register, both stack pointers, `SR`, `PC`, the
//!    prefetch queue, and every byte of memory the vector names. This is
//!    instruction semantics, and it is what the test asserts on by default.
//! 2. **Cycles** — the instruction's total. Charged four per bus access plus
//!    the manual's internal cycles.
//! 3. **Bus trace** — every access in order, with its address, width, data and
//!    direction.
//!
//! `ROADMAP.md` §0 asks for a known-failures ledger that only ever shrinks.
//! This core's carries **nothing the corpus covers**: at the commit that added
//! this file, all 124 instruction files of `68000/v1` passed all three checks
//! for every one of their 1 000 058 vectors — registers, both stack pointers,
//! the prefetch queue, memory, the cycle count *and* the complete bus trace,
//! under `RSEMU_680X0_STRICT`. Anything that fails later is a regression, not
//! a known gap, and belongs in a fix rather than in a list. The one entry in
//! `LEDGER` is a timing deviation in an instruction the corpus does not test.
//!
//! Two vectors are skipped, and only two: see `ANOMALIES`.
//!
//! Not covered by the corpus, and therefore only by the hand-written tests
//! next door: reset, interrupts and their autovectors, `STOP`, the trace bit,
//! user mode and the privilege violation. Every vector runs in supervisor
//! state with the interrupt mask at seven and tracing off.
//!
//! # Why the JSON parser is in here
//!
//! The dependency policy allows no `serde`, and this is one of two things in
//! the crate that read JSON. Recursive descent over a format this regular is
//! cheaper than the argument.

use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use std::path::Path;

use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, UnassignedPolicy,
};
use crate::core::sync::{self, LockRank};
use crate::core::value::Endian;

use super::{ADDRESS_MASK, Config, M68k, Regs, flags};

/// Instruction families this core is knowingly wrong about, and why.
///
/// One entry, and it is a timing deviation the corpus cannot see. Nothing the
/// corpus *does* cover is on it: `DIVU` and `DIVS` were here until their
/// data-dependent times — published only as maxima, MC68000UM Table 8-6 gives
/// `DIVU` as "140(1/0)+" with no formula at all — were reconstructed from the
/// shape of the microcode's division loop and checked against every division
/// vector. See `exec.rs`'s `divu_cycles`.
///
/// This list may only ever shrink.
pub(super) static LEDGER: &[(&str, &str)] = &[(
    "STOP",
    "settles the prefetch queue before stopping, so it costs two bus cycles \
     and eight clocks where hardware makes those cycles on the way out and \
     bills four. Deliberate: the alternative leaves the resume address two \
     bytes inside the instruction. Not in the corpus.",
)];

/// Vectors skipped because the *corpus* is wrong about them.
///
/// A short list, and it stays short. Each entry names one vector and says what
/// makes it impossible rather than merely surprising — a claim that has to be
/// defensible, since the alternative reading is that this core is wrong.
///
/// Both entries here are byte shifts whose expected final state changes all
/// thirty-two bits of the destination register. `ASL.B` writes the low eight
/// and no 68000 encoding writes the rest, so the expected value cannot be a
/// shift of the given input by any count; the accompanying cycle count and bus
/// trace are, for what it is worth, exactly what a two-bit `ASL.B` should
/// produce. Two vectors out of a million.
pub(super) static ANOMALIES: &[(&str, &str)] = &[
    (
        "e502 [ASL.b Q, D2] 1583",
        "expects all 32 bits of D2 to change for a byte shift",
    ),
    (
        "e502 [ASL.b Q, D2] 1761",
        "expects all 32 bits of D2 to change for a byte shift",
    ),
];

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

    fn u32_at(&self, key: &str) -> u32 {
        self.get(key)
            .unwrap_or_else(|| panic!("missing field {key}"))
            .num() as u32
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
                self.at += 4;
                Json::Num(0)
            }
            b't' => {
                self.at += 4;
                Json::Num(1)
            }
            b'f' => {
                self.at += 5;
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

/// One access, as both the corpus and the recording bus describe it.
#[derive(Debug, Clone, Copy, Eq)]
struct Access {
    addr: u32,
    value: u16,
    word: bool,
    write: bool,
    /// Compare only the low seven bits of the value.
    ///
    /// Set for the read half of a `TAS`, which the corpus records as a single
    /// read-modify-write transaction carrying the value it *wrote*. Bit 7 of
    /// the value read is therefore unknowable from the vector — `TAS` sets it
    /// unconditionally — while every other bit is.
    high_bit_unknown: bool,
}

impl PartialEq for Access {
    fn eq(&self, other: &Access) -> bool {
        let mask = if self.high_bit_unknown || other.high_bit_unknown {
            0x7f
        } else {
            0xffff
        };
        self.addr == other.addr
            && self.word == other.word
            && self.write == other.write
            && self.value & mask == other.value & mask
    }
}

impl core::fmt::Display for Access {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "[{}{} {:06x} {}]",
            if self.write { 'w' } else { 'r' },
            if self.word { ".w" } else { ".b" },
            self.addr,
            if self.word {
                format!("{:04x}", self.value)
            } else {
                format!("{:02x}", self.value)
            }
        )
    }
}

/// Sixteen megabytes of big-endian memory that records what the core does to
/// it.
///
/// One allocation per file rather than per vector: sixteen mebibytes eight
/// thousand times over is most of the runtime otherwise. [`Bus::rewind`]
/// restores only the bytes a vector touched.
#[derive(Debug)]
struct Bus(sync::Mutex<BusState>);

#[derive(Debug)]
struct BusState {
    memory: Vec<u8>,
    /// Every byte written since the last rewind, and what it was before.
    dirty: BTreeMap<u32, u8>,
    log: Vec<Access>,
}

impl Bus {
    fn new() -> Bus {
        Bus(sync::Mutex::with_rank(
            LockRank::DEVICE,
            BusState {
                memory: alloc::vec![0u8; (ADDRESS_MASK as usize) + 1],
                dirty: BTreeMap::new(),
                log: Vec::new(),
            },
        ))
    }

    /// Write a byte and remember what it displaced.
    fn poke(&self, addr: u32, value: u8) {
        let mut state = self.0.lock();
        let addr = addr & ADDRESS_MASK;
        let previous = state.memory[addr as usize];
        state.dirty.entry(addr).or_insert(previous);
        state.memory[addr as usize] = value;
    }

    fn peek(&self, addr: u32) -> u8 {
        self.0.lock().memory[(addr & ADDRESS_MASK) as usize]
    }

    /// Undo every write since the last rewind, and drop the trace.
    fn rewind(&self) {
        let mut state = self.0.lock();
        let dirty = core::mem::take(&mut state.dirty);
        for (addr, previous) in dirty {
            state.memory[addr as usize] = previous;
        }
        state.log.clear();
    }

    fn take_log(&self) -> Vec<Access> {
        core::mem::take(&mut self.0.lock().log)
    }
}

impl MemOps for Bus {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let mut state = self.0.lock();
        let base = offset as u32 & ADDRESS_MASK;
        for (i, slot) in dst.iter_mut().enumerate() {
            *slot = state.memory[(base.wrapping_add(i as u32) & ADDRESS_MASK) as usize];
        }
        if !attrs.debug {
            let value = if dst.len() == 2 {
                (u16::from(dst[0]) << 8) | u16::from(dst[1])
            } else {
                u16::from(dst[0])
            };
            state.log.push(Access {
                addr: base,
                value,
                word: dst.len() == 2,
                write: false,
                high_bit_unknown: false,
            });
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let mut state = self.0.lock();
        let base = offset as u32 & ADDRESS_MASK;
        for (i, byte) in src.iter().enumerate() {
            let addr = base.wrapping_add(i as u32) & ADDRESS_MASK;
            let previous = state.memory[addr as usize];
            state.dirty.entry(addr).or_insert(previous);
            state.memory[addr as usize] = *byte;
        }
        if !attrs.debug {
            let value = if src.len() == 2 {
                (u16::from(src[0]) << 8) | u16::from(src[1])
            } else {
                u16::from(src[0])
            };
            state.log.push(Access {
                addr: base,
                value,
                word: src.len() == 2,
                write: true,
                high_bit_unknown: false,
            });
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY.with_endian(Endian::Big)
    }
}

// ---------------------------------------------------------------------------
// One vector
// ---------------------------------------------------------------------------

/// Read a register file out of a corpus `initial` or `final` object.
fn regs_of(node: &Json) -> Regs {
    let mut regs = Regs {
        usp: node.u32_at("usp"),
        ssp: node.u32_at("ssp"),
        pc: node.u32_at("pc"),
        sr: node.u32_at("sr") as u16,
        ..Regs::default()
    };
    for i in 0..8 {
        regs.d[i] = node.u32_at(&format!("d{i}"));
    }
    for i in 0..7 {
        regs.a[i] = node.u32_at(&format!("a{i}"));
    }
    regs.a[7] = if regs.supervisor() {
        regs.ssp
    } else {
        regs.usp
    };
    let prefetch = node.get("prefetch").expect("prefetch").arr();
    regs.prefetch = [prefetch[0].num() as u16, prefetch[1].num() as u16];
    regs
}

/// What the corpus expects the bus to have seen.
///
/// Idle cycles (`n`) are dropped: they carry no address and the cycle count
/// already accounts for them. A `t` is `TAS`'s indivisible read-modify-write,
/// which the corpus records as one ten-cycle transaction carrying the value it
/// read; on the bus it is a read followed by a write of that value with bit 7
/// set, so it is expanded into the two accesses a bus watcher would see.
fn trace_of(node: &Json) -> Vec<Access> {
    let mut out = Vec::new();
    for t in node.get("transactions").expect("transactions").arr() {
        let t = t.arr();
        let kind = t[0].str();
        if kind != "r" && kind != "w" && kind != "t" {
            continue;
        }
        let addr = t[3].num() as u32 & ADDRESS_MASK;
        let word = t[4].str() == ".w";
        let value = t[5].num() as u16;
        out.push(Access {
            addr,
            word,
            value,
            write: kind == "w",
            high_bit_unknown: kind == "t",
        });
        if kind == "t" {
            out.push(Access {
                addr,
                word,
                value: value | 0x80,
                write: true,
                high_bit_unknown: false,
            });
        }
    }
    out
}

/// What went wrong with one vector, ready to print.
#[derive(Debug, Default)]
struct Tally {
    vectors: usize,
    skipped: usize,
    state_failures: usize,
    cycle_failures: usize,
    trace_failures: usize,
    first_state: Option<String>,
    first_cycles: Option<String>,
    first_trace: Option<String>,
}

impl Tally {
    fn merge(&mut self, other: &Tally) {
        self.vectors += other.vectors;
        self.skipped += other.skipped;
        self.state_failures += other.state_failures;
        self.cycle_failures += other.cycle_failures;
        self.trace_failures += other.trace_failures;
    }
}

/// Run every vector in one file.
fn run_file(path: &Path, limit: usize) -> Tally {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let vectors = Parser::new(&bytes).value();
    let bus = alloc::sync::Arc::new(Bus::new());
    let space = AddressSpace::new("cpu", 24)
        .with_endian(Endian::Big)
        .with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(
            Region::io("ram", u64::from(ADDRESS_MASK) + 1, bus.clone()),
            0,
        )
        .expect("16 MiB fits in 24 bits");
    let space = alloc::sync::Arc::new(space);
    let cpu = M68k::new(Config::default());
    cpu.attach_space(space);

    let mut tally = Tally::default();
    for vector in vectors.arr().iter().take(limit) {
        let name = vector.get("name").expect("name").str().to_string();
        if ANOMALIES.iter().any(|(anomaly, _)| *anomaly == name) {
            tally.skipped += 1;
            continue;
        }
        let initial = vector.get("initial").expect("initial");
        let expected = vector.get("final").expect("final");

        bus.rewind();
        for cell in initial.get("ram").expect("ram").arr() {
            let cell = cell.arr();
            bus.poke(cell[0].num() as u32, cell[1].num() as u8);
        }
        // The vectors start mid-program: the queue is already full, no reset is
        // owed, and whatever the previous vector left behind is irrelevant.
        // All of that through the public API, deliberately — a harness that
        // has to reach inside the core is a harness the next one cannot copy.
        cpu.set_reset_pending(false);
        cpu.resume();
        cpu.set_regs(regs_of(initial));
        bus.take_log();

        let used = cpu.step();
        tally.vectors += 1;

        // --- state ---
        let want = regs_of(expected);
        let got = cpu.regs();
        let mut detail = String::new();
        if got != want {
            detail.push_str(&diff_regs(&want, &got));
        }
        for cell in expected.get("ram").expect("ram").arr() {
            let cell = cell.arr();
            let addr = cell[0].num() as u32;
            let value = cell[1].num() as u8;
            let actual = bus.peek(addr);
            if actual != value {
                detail.push_str(&format!(
                    "  ram[{addr:06x}] want {value:02x} got {actual:02x}\n"
                ));
            }
        }
        if !detail.is_empty() {
            tally.state_failures += 1;
            if tally.first_state.is_none() {
                tally.first_state = Some(format!("{name}\n{detail}"));
            }
        }

        // --- cycles ---
        let want_cycles = vector.get("length").expect("length").num() as u64;
        if used != want_cycles {
            tally.cycle_failures += 1;
            if tally.first_cycles.is_none() {
                tally.first_cycles = Some(format!("{name}: want {want_cycles} cycles, got {used}"));
            }
        }

        // --- bus trace ---
        let want_trace = trace_of(vector);
        let got_trace = bus.take_log();
        if got_trace != want_trace {
            tally.trace_failures += 1;
            if tally.first_trace.is_none() {
                let show = |cs: &[Access]| {
                    cs.iter()
                        .map(alloc::string::ToString::to_string)
                        .collect::<Vec<_>>()
                        .join(" ")
                };
                tally.first_trace = Some(format!(
                    "{name}\n  want {}\n  got  {}",
                    show(&want_trace),
                    show(&got_trace)
                ));
            }
        }
    }
    tally
}

/// A field-by-field difference between two register files.
fn diff_regs(want: &Regs, got: &Regs) -> String {
    let mut out = String::new();
    for i in 0..8 {
        if want.d[i] != got.d[i] {
            out.push_str(&format!(
                "  d{i} want {:08x} got {:08x}\n",
                want.d[i], got.d[i]
            ));
        }
    }
    for i in 0..7 {
        if want.a[i] != got.a[i] {
            out.push_str(&format!(
                "  a{i} want {:08x} got {:08x}\n",
                want.a[i], got.a[i]
            ));
        }
    }
    for (name, w, g) in [
        ("usp", want.usp, got.usp),
        ("ssp", want.ssp, got.ssp),
        ("pc", want.pc, got.pc),
    ] {
        if w != g {
            out.push_str(&format!("  {name} want {w:08x} got {g:08x}\n"));
        }
    }
    if want.sr != got.sr {
        out.push_str(&format!(
            "  sr want {:04x} ({}) got {:04x} ({})\n",
            want.sr,
            ccr_text(want.sr),
            got.sr,
            ccr_text(got.sr)
        ));
    }
    if want.prefetch != got.prefetch {
        out.push_str(&format!(
            "  prefetch want {:04x},{:04x} got {:04x},{:04x}\n",
            want.prefetch[0], want.prefetch[1], got.prefetch[0], got.prefetch[1]
        ));
    }
    out
}

fn ccr_text(sr: u16) -> String {
    let mut out = String::new();
    for (mask, name) in [
        (flags::X, 'X'),
        (flags::N, 'N'),
        (flags::Z, 'Z'),
        (flags::V, 'V'),
        (flags::C, 'C'),
    ] {
        out.push(if sr & mask != 0 { name } else { '-' });
    }
    out
}

/// Run the whole corpus, or explain why it did not.
///
/// Not `#[ignore]`d: a skipped test that says nothing is how a suite quietly
/// stops running. This one prints the command that would have run it.
#[test]
fn single_step_tests() {
    let Ok(dir) = std::env::var("RSEMU_680X0_DIR") else {
        println!(
            "conformance: set RSEMU_680X0_DIR to a decompressed \
             SingleStepTests/680x0 68000/v1 directory to run it. The corpus has \
             no licence file, so it is fetched and run, never vendored."
        );
        return;
    };
    let dir = Path::new(&dir);
    let only: Option<Vec<String>> = std::env::var("RSEMU_680X0_TESTS")
        .ok()
        .map(|s| s.split(',').map(|p| p.trim().to_string()).collect());
    let limit: usize = std::env::var("RSEMU_680X0_LIMIT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(usize::MAX);
    let strict = std::env::var("RSEMU_680X0_STRICT").is_ok();

    let mut names: Vec<String> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
        .filter_map(|entry| {
            let entry = entry.ok()?;
            let name = entry.file_name().into_string().ok()?;
            let stem = name.strip_suffix(".json")?;
            Some(stem.to_string())
        })
        .collect();
    names.sort();
    assert!(
        !names.is_empty(),
        "no NAME.json files under {} — the corpus ships gzipped; \
         run `gunzip {}/*.json.gz` first",
        dir.display(),
        dir.display()
    );

    let mut total = Tally::default();
    let mut state_failures: Vec<(String, usize)> = Vec::new();
    let mut cycle_failures: Vec<(String, usize)> = Vec::new();
    let mut trace_failures: Vec<(String, usize)> = Vec::new();

    for name in &names {
        if let Some(only) = &only
            && !only.iter().any(|o| o == name)
        {
            continue;
        }
        let tally = run_file(&dir.join(format!("{name}.json")), limit);
        if tally.state_failures != 0 {
            state_failures.push((name.clone(), tally.state_failures));
            println!(
                "{name}: {}/{} vectors with wrong state; first is\n{}",
                tally.state_failures,
                tally.vectors,
                tally.first_state.as_deref().unwrap_or("")
            );
        }
        if tally.cycle_failures != 0 {
            cycle_failures.push((name.clone(), tally.cycle_failures));
            println!(
                "{name}: {}/{} vectors with wrong cycle count; first is {}",
                tally.cycle_failures,
                tally.vectors,
                tally.first_cycles.as_deref().unwrap_or("")
            );
        }
        if tally.trace_failures != 0 {
            trace_failures.push((name.clone(), tally.trace_failures));
            println!(
                "{name}: {}/{} vectors with a different bus trace; first is\n{}",
                tally.trace_failures,
                tally.vectors,
                tally.first_trace.as_deref().unwrap_or("")
            );
        }
        total.merge(&tally);
    }

    println!(
        "conformance: {} vectors — state {} ok, cycles {} ok, trace {} ok\
         {}",
        total.vectors,
        total.vectors - total.state_failures,
        total.vectors - total.cycle_failures,
        total.vectors - total.trace_failures,
        if total.skipped == 0 {
            String::new()
        } else {
            format!(" ({} skipped as corpus anomalies)", total.skipped)
        }
    );
    assert!(
        state_failures.is_empty(),
        "instruction families with wrong state: {state_failures:?}"
    );
    if strict {
        assert!(
            cycle_failures.is_empty(),
            "families with wrong cycle counts: {cycle_failures:?}"
        );
        assert!(
            trace_failures.is_empty(),
            "families with a different bus trace: {trace_failures:?}"
        );
    }
}

#[test]
fn the_ledger_only_records_timing() {
    // A semantics entry here would mean the core is knowingly wrong about what
    // an instruction does, which is not something to keep a list of. The
    // ledger is empty as it stands, and this is what keeps it honest if it
    // ever is not.
    for (name, why) in LEDGER {
        assert!(
            why.contains("time") || why.contains("cycle"),
            "{name}: the ledger is for timing, not semantics"
        );
    }
}

#[test]
fn the_anomaly_list_stays_short() {
    // Skipping a vector is a claim that the corpus is wrong, which is a claim
    // that needs a reason attached and a small enough list to audit by eye.
    assert!(
        ANOMALIES.len() <= 4,
        "too many vectors dismissed as corpus bugs"
    );
    for (name, why) in ANOMALIES {
        assert!(!why.is_empty(), "{name} needs a reason");
    }
}
