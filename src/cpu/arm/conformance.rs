//! The conformance runner for `SingleStepTests/ARM7TDMI`.
//!
//! `ROADMAP.md` §0: *accuracy is measured, never asserted*. This is as much of
//! the measurement as a public corpus makes available for ARM, and it is worth
//! being precise about how much that is.
//!
//! # What the corpus covers, and what it does not
//!
//! `SingleStepTests/ARM7TDMI` is MIT-licensed, is marked **experimental**
//! upstream, and describes an **ARM7TDMI** — an **ARMv4T** part. It therefore
//! validates the subset this core shares with ARMv4T:
//!
//! - data processing and the barrel shifter, in every addressing mode;
//! - `MUL`/`MLA` and the long multiplies;
//! - `LDR`/`STR`/`LDRH`/`LDRSB`/`LDRSH`, `LDM`/`STM`, `SWP`;
//! - `B`/`BL`/`BX`, `MRS`/`MSR`, `SWI`, mode banking.
//!
//! It says **nothing** about everything ARMv5 added, which is exactly the part
//! of this core with no independent oracle:
//!
//! - `CLZ`, `BKPT`, and both forms of `BLX`;
//! - the DSP (E) extensions: `QADD`/`QSUB`/`QDADD`/`QDSUB`, the `SMLA<x><y>`
//!   family, `LDRD`/`STRD`, `PLD`;
//! - ARMv5's interworking loads — `LDR pc` and `LDM {pc}` change instruction
//!   set in v5 and do not in v4T, so the corpus would actively disagree;
//! - the v5 rule that `MUL` leaves `C` alone where v4 destroyed it;
//! - the exception model beyond `SWI`, and the coprocessor interface.
//!
//! Those are covered by the hand-written tests in [`super::tests`], and that is
//! the only place they are covered. Anyone reading a green run here should read
//! this paragraph too.
//!
//! # Running it
//!
//! The corpus is **downloaded, never vendored** (`ROADMAP.md` §1, §12). Point
//! an environment variable at a directory of `.json` files:
//!
//! ```text
//! git clone --depth 1 https://github.com/SingleStepTests/ARM7TDMI /tmp/arm7tdmi
//! # the upstream files are gzipped; there is no decompressor in this crate's
//! # dependency budget, so unpack them first
//! gunzip -r /tmp/arm7tdmi
//! RSEMU_ARM7TDMI_DIR=/tmp/arm7tdmi/v1 cargo test --features cpu-arm conformance -- --nocapture
//! ```
//!
//! Without the variable the test prints why it did nothing and passes, so
//! `cargo test` stays hermetic and offline.
//!
//! # The schema this reads, and what happens if it is wrong
//!
//! Each file is an array of vectors. A vector is an object with `initial` and
//! `final` register states and an optional `transactions` array:
//!
//! ```text
//! { "initial": { "R": [16 words], "R_fiq": [7], "R_svc": [2], "R_abt": [2],
//!                "R_irq": [2], "R_und": [2], "CPSR": w, "SPSR": [5] },
//!   "final":   { ... the same shape ... },
//!   "opcode": w,
//!   "transactions": [ { "kind": 0|1, "size": 1|2|4, "addr": w, "data": w }, ... ] }
//! ```
//!
//! `R` is the *currently visible* file, selected by `CPSR`'s mode; the
//! `R_<mode>` arrays hold the banks that are not currently visible. `SPSR` is
//! ordered `fiq, svc, abt, irq, und` — [`SPSR_ORDER`] is the one line to change
//! if that turns out to be wrong.
//!
//! **The runner does not guess.** A file whose vectors do not have these keys
//! fails with a message naming the file and the missing key rather than
//! silently passing, because a conformance suite that quietly measures nothing
//! is worse than none. The schema above was written from the corpus's
//! documentation rather than by running it; if upstream differs,
//! [`CpuState::from_json`] and [`Transaction::from_json`] are the two functions
//! to adjust, and [`the self-test`](schema_round_trips_through_the_runner)
//! keeps them honest in the meantime.
//!
//! # Bus traces are compared by content, not by order
//!
//! The corpus records the ARM7TDMI's cycle-by-cycle bus activity, including
//! the prefetch unit's speculative fetches. This core deliberately does not
//! model a prefetch pipeline (see `super::exec`'s timing model) — it fetches
//! exactly the instruction it executes. Comparing the trace position by
//! position would therefore fail on every vector for a reason that is a
//! documented modelling choice rather than a bug. So writes are checked by
//! address and value, and reads are used to prime memory.
//!
//! # The ledger
//!
//! `ROADMAP.md` §0 asks every core to ship a known-failures ledger that only
//! ever shrinks. **This one has not been established**: the corpus was not
//! available in the environment this core was written in, so the honest ledger
//! is "unknown, and the runner exists so that the first person with the corpus
//! can write it". That is a gap, and it is stated here rather than implied by
//! an empty list.
//!
//! # Why the JSON parser is in here
//!
//! The dependency policy allows no `serde` (`ROADMAP.md` §0), and this is one
//! of only two things in the crate that read JSON. Recursive descent over a
//! format this regular is cheaper than the argument.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, UnassignedPolicy,
};
use crate::core::sync::{self, LockRank};

use super::{Arm, Config, Mode, Regs, psr};

/// The order the corpus stores the five `SPSR`s in.
///
/// Ours are indexed by [`Mode::spsr_index`], which is `fiq, irq, svc, abt,
/// und`. This maps corpus position to ours.
pub(super) const SPSR_ORDER: [usize; 5] = [
    0, // fiq
    2, // svc
    3, // abt
    1, // irq
    4, // und
];

// ---------------------------------------------------------------------------
// A very small JSON reader
// ---------------------------------------------------------------------------

/// Just enough of JSON for this corpus.
#[derive(Debug, Clone, PartialEq)]
enum Json {
    Null,
    Bool(bool),
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

    fn num(&self) -> Option<i64> {
        match self {
            Json::Num(n) => Some(*n),
            Json::Bool(b) => Some(i64::from(*b)),
            _ => None,
        }
    }

    fn arr(&self) -> Option<&[Json]> {
        match self {
            Json::Arr(items) => Some(items),
            _ => None,
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

    fn peek(&mut self) -> u8 {
        self.skip_space();
        self.bytes.get(self.at).copied().unwrap_or(b'\0')
    }

    fn eat(&mut self, byte: u8) -> Result<(), String> {
        if self.peek() != byte {
            return Err(format!("expected `{}` at offset {}", byte as char, self.at));
        }
        self.at += 1;
        Ok(())
    }

    fn value(&mut self) -> Result<Json, String> {
        match self.peek() {
            b'{' => {
                self.at += 1;
                let mut fields = Vec::new();
                if self.peek() == b'}' {
                    self.at += 1;
                    return Ok(Json::Obj(fields));
                }
                loop {
                    let key = self.string()?;
                    self.eat(b':')?;
                    fields.push((key, self.value()?));
                    if self.peek() == b',' {
                        self.at += 1;
                    } else {
                        break;
                    }
                }
                self.eat(b'}')?;
                Ok(Json::Obj(fields))
            }
            b'[' => {
                self.at += 1;
                let mut items = Vec::new();
                if self.peek() == b']' {
                    self.at += 1;
                    return Ok(Json::Arr(items));
                }
                loop {
                    items.push(self.value()?);
                    if self.peek() == b',' {
                        self.at += 1;
                    } else {
                        break;
                    }
                }
                self.eat(b']')?;
                Ok(Json::Arr(items))
            }
            b'"' => Ok(Json::Str(self.string()?)),
            b't' => {
                self.at += 4;
                Ok(Json::Bool(true))
            }
            b'f' => {
                self.at += 5;
                Ok(Json::Bool(false))
            }
            b'n' => {
                self.at += 4;
                Ok(Json::Null)
            }
            _ => {
                let start = self.at;
                if self.bytes.get(self.at) == Some(&b'-') {
                    self.at += 1;
                }
                while matches!(self.bytes.get(self.at), Some(b'0'..=b'9')) {
                    self.at += 1;
                }
                if start == self.at {
                    return Err(format!("not a value at offset {start}"));
                }
                let text = core::str::from_utf8(&self.bytes[start..self.at])
                    .map_err(|_| "non-ascii number".to_string())?;
                // The corpus is generated from 32-bit unsigned words, which fit
                // an i64 without loss; anything larger is a schema surprise.
                text.parse()
                    .map(Json::Num)
                    .map_err(|_| format!("integer out of range at offset {start}"))
            }
        }
    }

    fn string(&mut self) -> Result<String, String> {
        self.eat(b'"')?;
        let start = self.at;
        while self.bytes.get(self.at).is_some_and(|b| *b != b'"') {
            self.at += 1;
        }
        let text = String::from_utf8_lossy(&self.bytes[start..self.at]).into_owned();
        self.at += 1;
        Ok(text)
    }
}

// ---------------------------------------------------------------------------
// The vector schema
// ---------------------------------------------------------------------------

/// One side of a vector: the whole architectural register file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct CpuState {
    r: [u32; 16],
    r_fiq: [u32; 7],
    r_svc: [u32; 2],
    r_abt: [u32; 2],
    r_irq: [u32; 2],
    r_und: [u32; 2],
    cpsr: u32,
    spsr: [u32; 5],
}

fn words<const N: usize>(json: &Json, key: &str) -> Result<[u32; N], String> {
    let items = json
        .get(key)
        .ok_or_else(|| format!("missing key `{key}`"))?
        .arr()
        .ok_or_else(|| format!("key `{key}` is not an array"))?;
    if items.len() != N {
        return Err(format!(
            "key `{key}` has {} entries, expected {N}",
            items.len()
        ));
    }
    let mut out = [0u32; N];
    for (slot, item) in out.iter_mut().zip(items) {
        *slot = item
            .num()
            .ok_or_else(|| format!("key `{key}` holds a non-number"))? as u32;
    }
    Ok(out)
}

fn word(json: &Json, key: &str) -> Result<u32, String> {
    Ok(json
        .get(key)
        .ok_or_else(|| format!("missing key `{key}`"))?
        .num()
        .ok_or_else(|| format!("key `{key}` is not a number"))? as u32)
}

impl CpuState {
    /// Read one side of a vector.
    fn from_json(json: &Json) -> Result<CpuState, String> {
        Ok(CpuState {
            r: words::<16>(json, "R")?,
            r_fiq: words::<7>(json, "R_fiq")?,
            r_svc: words::<2>(json, "R_svc")?,
            r_abt: words::<2>(json, "R_abt")?,
            r_irq: words::<2>(json, "R_irq")?,
            r_und: words::<2>(json, "R_und")?,
            cpsr: word(json, "CPSR")?,
            spsr: words::<5>(json, "SPSR")?,
        })
    }

    /// Turn it into this core's register file.
    fn to_regs(self) -> Regs {
        let mut regs = Regs::new();
        regs.cpsr = self.cpsr;
        regs.r = self.r;
        // The corpus stores R15 as the register *reads* — the instruction plus
        // eight in ARM state, plus four in Thumb. This core stores the
        // instruction's own address.
        regs.r[15] = pc_of(&self);
        regs.banked_r8_r12[1].copy_from_slice(&self.r_fiq[..5]);
        regs.banked_sp_lr[1] = [self.r_fiq[5], self.r_fiq[6]];
        regs.banked_sp_lr[2] = self.r_irq;
        regs.banked_sp_lr[3] = self.r_svc;
        regs.banked_sp_lr[4] = self.r_abt;
        regs.banked_sp_lr[5] = self.r_und;
        // The User bank has no `R_usr` array: when the visible file *is* the
        // User bank, `R` already holds it, and otherwise the corpus does not
        // say.
        if regs.mode().bank() == 0 {
            regs.banked_sp_lr[0] = [self.r[13], self.r[14]];
        }
        for (corpus, ours) in SPSR_ORDER.iter().enumerate() {
            regs.spsr[*ours] = self.spsr[corpus];
        }
        regs
    }

    /// Read this core's register file back out in the corpus's shape.
    fn from_regs(regs: &Regs) -> CpuState {
        let mut spsr = [0u32; 5];
        for (corpus, ours) in SPSR_ORDER.iter().enumerate() {
            spsr[corpus] = regs.spsr[*ours];
        }
        let fiq_sp_lr = [
            regs.reg_in_mode(Mode::FIQ, 13),
            regs.reg_in_mode(Mode::FIQ, 14),
        ];
        let mut r_fiq = [0u32; 7];
        for (i, slot) in r_fiq.iter_mut().enumerate().take(5) {
            *slot = regs.reg_in_mode(Mode::FIQ, 8 + i as u8);
        }
        r_fiq[5] = fiq_sp_lr[0];
        r_fiq[6] = fiq_sp_lr[1];
        let bank = |mode: Mode| [regs.reg_in_mode(mode, 13), regs.reg_in_mode(mode, 14)];
        // Put R15 back in the corpus's pipelined form, so this is the exact
        // inverse of `to_regs`.
        let mut r = regs.r;
        r[15] = r[15].wrapping_add(if regs.cpsr & psr::T != 0 { 4 } else { 8 });
        CpuState {
            r,
            r_fiq,
            r_svc: bank(Mode::SUPERVISOR),
            r_abt: bank(Mode::ABORT),
            r_irq: bank(Mode::IRQ),
            r_und: bank(Mode::UNDEFINED),
            cpsr: regs.cpsr,
            spsr,
        }
    }
}

/// One recorded bus access.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Transaction {
    write: bool,
    size: u32,
    addr: u32,
    data: u32,
}

impl Transaction {
    fn from_json(json: &Json) -> Result<Transaction, String> {
        Ok(Transaction {
            write: word(json, "kind")? != 0,
            size: word(json, "size")?,
            addr: word(json, "addr")?,
            data: word(json, "data")?,
        })
    }
}

// ---------------------------------------------------------------------------
// The bus the vectors describe
// ---------------------------------------------------------------------------

/// Memory primed from a vector's read transactions, recording every write.
#[derive(Debug, Default)]
struct VectorMemory {
    bytes: BTreeMap<u32, u8>,
    writes: Vec<(u32, u32, u32)>,
}

#[derive(Debug)]
struct VectorBus(sync::Mutex<VectorMemory>);

impl MemOps for VectorBus {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let m = self.0.lock();
        for (i, slot) in dst.iter_mut().enumerate() {
            let addr = (offset as u32).wrapping_add(i as u32);
            // A byte the corpus never mentioned reads as zero: the vector does
            // not constrain it, so any value is as correct as any other and
            // zero is the reproducible one.
            *slot = m.bytes.get(&addr).copied().unwrap_or(0);
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let mut m = self.0.lock();
        let mut value = 0u32;
        for (i, byte) in src.iter().enumerate() {
            let addr = (offset as u32).wrapping_add(i as u32);
            m.bytes.insert(addr, *byte);
            if i < 4 {
                value |= u32::from(*byte) << (8 * i);
            }
        }
        let addr = offset as u32;
        let size = src.len() as u32;
        m.writes.push((addr, size, value));
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

// ---------------------------------------------------------------------------
// Running one vector
// ---------------------------------------------------------------------------

/// What went wrong with one vector, ready to print.
#[derive(Debug)]
struct Failure {
    index: usize,
    detail: String,
}

/// Run one vector and describe how it differed, if it did.
fn run_vector(vector: &Json, index: usize) -> Result<Option<Failure>, String> {
    let initial = CpuState::from_json(
        vector
            .get("initial")
            .ok_or_else(|| "missing key `initial`".to_string())?,
    )?;
    let expected = CpuState::from_json(
        vector
            .get("final")
            .ok_or_else(|| "missing key `final`".to_string())?,
    )?;

    let mut memory = VectorMemory::default();
    let mut expected_writes = Vec::new();
    if let Some(items) = vector.get("transactions").and_then(Json::arr) {
        for item in items {
            let t = Transaction::from_json(item)?;
            if t.write {
                expected_writes.push((t.addr, t.size, t.data & size_mask(t.size)));
            } else {
                for i in 0..t.size {
                    let byte = (t.data >> (8 * i)) as u8;
                    memory.bytes.insert(t.addr.wrapping_add(i), byte);
                }
            }
        }
    }
    // The instruction itself, where the corpus names it separately from the
    // fetch transaction.
    if let Some(opcode) = vector.get("opcode").and_then(Json::num) {
        let pc = pc_of(&initial);
        for i in 0..4u32 {
            memory
                .bytes
                .insert(pc.wrapping_add(i), ((opcode as u32) >> (8 * i)) as u8);
        }
    }

    let bus = alloc::sync::Arc::new(VectorBus(sync::Mutex::with_rank(LockRank::DEVICE, memory)));
    let space = AddressSpace::new("cpu", 32).with_unassigned(UnassignedPolicy::FAULT);
    space
        .topology()
        .map(Region::io("ram", 1 << 32, bus.clone()), 0)
        .map_err(|e| e.to_string())?;

    // The corpus is an ARM7TDMI, which stores R15 as the instruction plus
    // twelve; that is the one implementation-defined value it can observe.
    let cpu = Arm::new(Config::ARM7TDMI);
    cpu.attach_space(alloc::sync::Arc::new(space));
    // The vectors start mid-program with the lines idle, so there is no reset
    // sequence and no pending interrupt to model.
    cpu.set_regs(initial.to_regs());
    {
        // Consume the reset the core owes without letting it run.
        let mut session = cpu.session.lock();
        session.state.reset_pending = false;
    }
    bus.0.lock().writes.clear();

    cpu.step();

    let mut detail = String::new();
    let got = CpuState::from_regs(&cpu.regs());
    if got.r != expected.r {
        for (i, (a, b)) in got.r.iter().zip(expected.r.iter()).enumerate() {
            if a != b {
                detail.push_str(&format!("  r{i}: want {b:08x} got {a:08x}\n"));
            }
        }
    }
    if got.cpsr != expected.cpsr {
        detail.push_str(&format!(
            "  cpsr: want {:08x} got {:08x}\n",
            expected.cpsr, got.cpsr
        ));
    }
    if got.spsr != expected.spsr {
        detail.push_str(&format!(
            "  spsr: want {:08x?} got {:08x?}\n",
            expected.spsr, got.spsr
        ));
    }
    for (name, a, b) in [
        ("fiq", &got.r_fiq[..], &expected.r_fiq[..]),
        ("svc", &got.r_svc[..], &expected.r_svc[..]),
        ("abt", &got.r_abt[..], &expected.r_abt[..]),
        ("irq", &got.r_irq[..], &expected.r_irq[..]),
        ("und", &got.r_und[..], &expected.r_und[..]),
    ] {
        if a != b {
            detail.push_str(&format!("  {name} bank: want {b:08x?} got {a:08x?}\n"));
        }
    }

    let writes = core::mem::take(&mut bus.0.lock().writes);
    for want in &expected_writes {
        if !writes
            .iter()
            .any(|(addr, size, data)| (*addr, *size, *data & size_mask(*size)) == *want)
        {
            detail.push_str(&format!(
                "  missing write: {:08x} size {} data {:08x}\n",
                want.0, want.1, want.2
            ));
        }
    }

    Ok((!detail.is_empty()).then_some(Failure { index, detail }))
}

/// The low bits a transaction of this size actually carries.
fn size_mask(size: u32) -> u32 {
    match size {
        1 => 0xff,
        2 => 0xffff,
        _ => u32::MAX,
    }
}

/// Where the vector's instruction lives.
///
/// The corpus stores `R15` in its pipelined form — the instruction's address
/// plus eight in ARM state, plus four in Thumb — because that is what the
/// register reads as. This core stores the instruction's own address, so the
/// pipeline offset comes back off.
fn pc_of(state: &CpuState) -> u32 {
    let ahead = if state.cpsr & psr::T != 0 { 4 } else { 8 };
    state.r[15].wrapping_sub(ahead)
}

/// Every `.json` file under `dir`, one level of subdirectories included.
fn json_files(dir: &Path) -> Vec<PathBuf> {
    let mut out = Vec::new();
    let Ok(entries) = std::fs::read_dir(dir) else {
        return out;
    };
    for entry in entries.flatten() {
        let path = entry.path();
        if path.is_dir() {
            out.extend(json_files(&path));
        } else if path.extension().is_some_and(|e| e == "json") {
            out.push(path);
        }
    }
    out.sort();
    out
}

/// Run the whole corpus, or explain why it did not.
///
/// Not `#[ignore]`d: a skipped test that says nothing is how a suite quietly
/// stops running. This one prints the command that would have run it.
#[test]
fn single_step_tests() {
    let Ok(dir) = std::env::var("RSEMU_ARM7TDMI_DIR") else {
        println!(
            "conformance: set RSEMU_ARM7TDMI_DIR to an unpacked \
             SingleStepTests/ARM7TDMI directory to run it. Note that the corpus \
             is ARMv4T and validates none of this core's ARMv5 additions — see \
             this module's documentation."
        );
        return;
    };
    let dir = Path::new(&dir);
    let files = json_files(dir);
    assert!(!files.is_empty(), "no .json files under {}", dir.display());

    let only = std::env::var("RSEMU_ARM7TDMI_FILES").ok();
    let mut ran_files = 0usize;
    let mut ran_vectors = 0usize;
    let mut total_failures = 0usize;
    let mut failing_files: Vec<(String, usize)> = Vec::new();

    for path in &files {
        let name = path
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_default();
        if let Some(only) = &only
            && !only.split(',').any(|f| f.trim() == name)
        {
            continue;
        }
        let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        let parsed = Parser::new(&bytes)
            .value()
            .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        let vectors = parsed
            .arr()
            .unwrap_or_else(|| panic!("{}: the file is not an array of vectors", path.display()));

        ran_files += 1;
        let mut failures = Vec::new();
        for (index, vector) in vectors.iter().enumerate() {
            ran_vectors += 1;
            match run_vector(vector, index) {
                Ok(None) => {}
                Ok(Some(failure)) => failures.push(failure),
                // A schema mismatch is a loud failure, not a silent pass: a
                // conformance suite that quietly measures nothing is worse than
                // no suite at all.
                Err(e) => panic!(
                    "{}: vector {index} does not match the expected schema: {e}\n\
                     See this module's documentation for the shape it reads.",
                    path.display()
                ),
            }
        }
        if !failures.is_empty() {
            total_failures += failures.len();
            failing_files.push((name.clone(), failures.len()));
            println!(
                "{name}: {} of {} vectors failed; the first is #{}:\n{}",
                failures.len(),
                vectors.len(),
                failures[0].index,
                failures[0].detail
            );
        }
    }

    println!("conformance: {ran_files} files, {ran_vectors} vectors, {total_failures} failures");
    assert!(failing_files.is_empty(), "failing files: {failing_files:?}");
}

#[test]
fn schema_round_trips_through_the_runner() {
    // A synthetic vector in the documented shape, so the parser, the state
    // mapping and the comparison are exercised by an ordinary `cargo test`
    // even with no corpus present. `e3a0_0042` is `MOV r0, #0x42`.
    let json = br#"[{
      "initial": {
        "R": [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4104],
        "R_fiq": [0,0,0,0,0,0,0], "R_svc": [0,0], "R_abt": [0,0],
        "R_irq": [0,0], "R_und": [0,0],
        "CPSR": 31, "SPSR": [0,0,0,0,0]
      },
      "final": {
        "R": [66,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4108],
        "R_fiq": [0,0,0,0,0,0,0], "R_svc": [0,0], "R_abt": [0,0],
        "R_irq": [0,0], "R_und": [0,0],
        "CPSR": 31, "SPSR": [0,0,0,0,0]
      },
      "opcode": 3818913858,
      "transactions": []
    }]"#;
    let parsed = Parser::new(json).value().expect("the schema parses");
    let vectors = parsed.arr().expect("an array of vectors");
    // The corpus stores R15 pipelined, so 4104 is the instruction at 0x1000
    // and 4108 is the instruction at 0x1004.
    let outcome = run_vector(&vectors[0], 0).expect("the schema is understood");
    assert!(outcome.is_none(), "{outcome:?}");
}

#[test]
fn a_missing_key_is_reported_rather_than_ignored() {
    let json = br#"[{"initial": {"R": [0]}, "final": {}}]"#;
    let parsed = Parser::new(json).value().unwrap();
    let vectors = parsed.arr().unwrap();
    let err = run_vector(&vectors[0], 0).unwrap_err();
    assert!(err.contains("R"), "{err}");
}
