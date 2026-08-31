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
//! RSEMU_ARM7TDMI_DIR=/tmp/arm7tdmi/v1 cargo test --features cpu-arm-aprofile conformance -- --nocapture
//! ```
//!
//! Without the variable the test prints why it did nothing and passes, so
//! `cargo test` stays hermetic and offline.
//!
//! The upstream files are a packed binary format; `v1/transcode_json.py` in
//! the corpus turns each `.json.bin` into the `.json` this reads. They are
//! bulky — about 120 MB per file — so transcode what you need rather than all
//! forty-five at once. `RSEMU_ARM7TDMI_FILES` restricts the run to a
//! comma-separated list of file stems, and `RSEMU_ARM7TDMI_TRACE=<index>`
//! prints every access this core made for one vector, which is how you
//! diagnose a disagreement rather than guessing at it.
//!
//! # The schema
//!
//! Each file is an array of vectors. A vector is an object with `initial` and
//! `final` register states, an `opcode`, a `base_addr` and a `transactions`
//! array:
//!
//! ```text
//! { "initial": { "R": [16 words], "R_fiq": [7], "R_svc": [2], "R_abt": [2],
//!                "R_irq": [2], "R_und": [2], "CPSR": w, "SPSR": [5],
//!                "pipeline": [2], "access": w },
//!   "final":   { ... the same shape ... },
//!   "opcode": w, "base_addr": w,
//!   "transactions": [ { "kind": 0|1|2, "size": 1|2|4, "addr": w, "data": w,
//!                       "cycle": w, "access": w }, ... ] }
//! ```
//!
//! Four things about it are not obvious, and all four were established by
//! reading the corpus rather than by assuming:
//!
//! - **`R` is the User/System bank**, not the currently visible file. The
//!   `R_<mode>` arrays hold the other banks *always*, whatever mode `CPSR`
//!   names. [`CpuState::to_regs`] therefore loads `R` while in System mode and
//!   only then switches to the vector's real mode, letting the core's own
//!   banking move the values; [`CpuState::from_regs`] is the exact inverse.
//! - **`SPSR` is ordered `fiq, svc, abt, irq, und`** — confirmed by finding
//!   vectors where an `S`-bit data-processing instruction restores `CPSR` and
//!   checking which entry it came from. [`SPSR_ORDER`] is the mapping.
//! - **`kind` is 0 for a fetch, 1 for a data read, 2 for a data write** —
//!   confirmed from `SWPB` vectors, where the `kind == 1` value is what the
//!   destination register ends up holding.
//! - **`addr` is the address the core drove**, low bits included; the memory
//!   system drops them and answers with the aligned word or halfword, which is
//!   what `data` holds. See [`align`].
//!
//! `R15` is stored pipelined — the instruction plus eight in ARM state, plus
//! four in Thumb — so [`pc_of`] takes the offset back off. `pipeline` holds the
//! two instruction words the reference had already fetched and `access` the
//! cycle type; this core models neither, and neither is compared.
//!
//! **The runner does not guess.** A file whose vectors do not have these keys
//! fails with a message naming the file and the missing key rather than
//! silently passing, because a conformance suite that quietly measures nothing
//! is worse than none.
//!
//! # The instruction is injected, not stored
//!
//! The reference had already prefetched the instruction under test, so its
//! memory at `base_addr` holds something else entirely — and an `LDM` whose
//! range covers its own address reads *that*, which the transaction list
//! records. Priming the opcode into the byte map would make the two collide.
//! This core does fetch, and its fetch is always its first access, so the
//! opcode gets a one-shot channel of its own ([`VectorMemory::inject`]).
//!
//! For the same reason **fetch transactions do not prime memory**: they
//! describe the refill of the instructions *after* this one, and the reference
//! answers a fetch and a data read at the same address with different values.
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
//! ever shrinks. **It is empty.** All forty-four usable files pass every
//! vector: 2 200 000 vectors across the ARM and Thumb sets, with no failures
//! and nothing on a known-failures list.
//!
//! What is *not* empty, and is deliberately kept separate, is two lists of
//! places where this core and the corpus disagree on purpose:
//!
//! - [`REJECTED_FILES`] — files that are simply wrong, excluded by name.
//! - [`architectural_divergence`] — cases where ARMv4T and ARMv5TE genuinely
//!   differ, so a vector the corpus gets right for an ARM7TDMI is one this
//!   core must get differently. A vector is excused only if it actually failed
//!   *and* its failure has the shape the divergence predicts.
//!
//! Neither is a known-failures ledger: nothing in either will ever be "fixed",
//! because fixing it would mean implementing ARMv4T or copying a bug.
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
        // Start in System mode, where `Regs::r` *is* the User bank, so that
        // `R` can be loaded verbatim. Switching to the vector's real mode
        // afterwards is what moves the User values into their shadow slot and
        // the mode's own values into view — the same code path guest `MSR`
        // takes, so the mapping cannot disagree with the core about banking.
        regs.cpsr = u32::from(Mode::SYSTEM.0);
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
        for (corpus, ours) in SPSR_ORDER.iter().enumerate() {
            regs.spsr[*ours] = self.spsr[corpus];
        }
        regs.write_cpsr(self.cpsr);
        regs
    }

    /// Read this core's register file back out in the corpus's shape.
    ///
    /// The exact inverse of [`CpuState::to_regs`]: step back into System mode
    /// on a copy, which returns the User bank to view and parks the current
    /// mode's registers in their shadow, then read every bank out of its slot.
    fn from_regs(regs: &Regs) -> CpuState {
        let mut spsr = [0u32; 5];
        for (corpus, ours) in SPSR_ORDER.iter().enumerate() {
            spsr[corpus] = regs.spsr[*ours];
        }
        // Capture what the corpus wants before `set_mode` rewrites the mode
        // field of the copy.
        let cpsr = regs.cpsr;
        let pipelined = regs.r[15].wrapping_add(if cpsr & psr::T != 0 { 4 } else { 8 });
        let mut flat = *regs;
        flat.set_mode(Mode::SYSTEM);
        let mut r = flat.r;
        r[15] = pipelined;
        let mut r_fiq = [0u32; 7];
        r_fiq[..5].copy_from_slice(&flat.banked_r8_r12[1]);
        r_fiq[5] = flat.banked_sp_lr[1][0];
        r_fiq[6] = flat.banked_sp_lr[1][1];
        CpuState {
            r,
            r_fiq,
            r_svc: flat.banked_sp_lr[3],
            r_abt: flat.banked_sp_lr[4],
            r_irq: flat.banked_sp_lr[2],
            r_und: flat.banked_sp_lr[5],
            cpsr,
            spsr,
        }
    }
}

/// One recorded bus access.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Transaction {
    write: bool,
    fetch: bool,
    size: u32,
    addr: u32,
    data: u32,
}

impl Transaction {
    /// `kind` is `0` for an instruction fetch, `1` for a data read and `2` for
    /// a data write.
    ///
    /// Established from the corpus itself rather than assumed: in an `SWPB`
    /// vector the `kind == 1` transaction's data is what the destination
    /// register ends up holding, and the `kind == 2` transaction's data is the
    /// source register's low byte. Getting this backwards makes every load
    /// look like a store.
    const FETCH: u32 = 0;
    const WRITE: u32 = 2;

    fn from_json(json: &Json) -> Result<Transaction, String> {
        let kind = word(json, "kind")?;
        Ok(Transaction {
            write: kind == Transaction::WRITE,
            fetch: kind == Transaction::FETCH,
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
    /// Every access this core made, in order, when tracing is on.
    trace: Option<Vec<(bool, u32, u32, u32)>>,
    /// The instruction under test, served to the first access and no other.
    ///
    /// The corpus **injects** the opcode rather than storing it: the reference
    /// had already prefetched it, so its memory at `base_addr` holds whatever
    /// the generator put there — and an `LDM` whose range covers its own
    /// address reads that other value, which the transaction list records.
    /// Priming the opcode into the byte map would make the two collide and one
    /// of them would have to lose. This core does fetch, and its fetch is
    /// always its first access, so the instruction gets its own channel.
    inject: Option<(u32, u32)>,
}

#[derive(Debug)]
struct VectorBus(sync::Mutex<VectorMemory>);

impl MemOps for VectorBus {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let mut m = self.0.lock();
        let mut value = 0u32;
        if let Some((at, opcode)) = m.inject
            && at == offset as u32
        {
            m.inject = None;
            for (i, slot) in dst.iter_mut().enumerate() {
                *slot = (opcode >> (8 * i)) as u8;
                value |= u32::from(*slot) << (8 * i);
            }
            if let Some(trace) = m.trace.as_mut() {
                trace.push((false, offset as u32, dst.len() as u32, value));
            }
            return Ok(());
        }
        for (i, slot) in dst.iter_mut().enumerate() {
            let addr = (offset as u32).wrapping_add(i as u32);
            // A byte the corpus never mentioned reads as zero: the vector does
            // not constrain it, so any value is as correct as any other and
            // zero is the reproducible one.
            *slot = m.bytes.get(&addr).copied().unwrap_or(0);
            if i < 4 {
                value |= u32::from(*slot) << (8 * i);
            }
        }
        if let Some(trace) = m.trace.as_mut() {
            trace.push((false, offset as u32, dst.len() as u32, value));
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
        if let Some(trace) = m.trace.as_mut() {
            trace.push((true, addr, size, value));
        }
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
    /// Set when this vector fails for a documented ARMv4T-versus-ARMv5TE
    /// reason rather than because the core is wrong.
    divergence: Option<&'static str>,
    /// A coarse name for *what* differed, so a run reports classes of
    /// disagreement rather than fifty thousand individual diffs. Two vectors
    /// that fail the same way are one finding.
    signature: String,
    detail: String,
}

/// Whether this vector should print a full trace of what the core did.
///
/// `RSEMU_ARM7TDMI_TRACE=<index>` — combine it with `RSEMU_ARM7TDMI_FILES` to
/// pin one vector. Diagnosing a corpus disagreement means knowing which
/// accesses *we* made, and reconstructing that by hand from the encoding is
/// how an afternoon disappears.
fn tracing(index: usize) -> bool {
    std::env::var("RSEMU_ARM7TDMI_TRACE")
        .ok()
        .and_then(|v| v.trim().parse::<usize>().ok())
        .is_some_and(|want| want == index)
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
    // The instruction under test is never in the transaction list: the
    // reference had already prefetched it, and the fetches that *are* listed
    // are the refill. It arrives in `opcode` instead. Prime it first, so a
    // listed read of an overlapping address still wins.
    if let Some(opcode) = vector.get("opcode").and_then(Json::num) {
        let width = if initial.cpsr & psr::T != 0 { 2 } else { 4 };
        // At the address the *fetch* uses, which drops the low bits.
        let pc = pc_of(&initial) & !(width - 1);
        memory.inject = Some((pc, opcode as u32));
    }
    let mut expected_writes = Vec::new();
    if let Some(items) = vector.get("transactions").and_then(Json::arr) {
        for item in items {
            let t = Transaction::from_json(item)?;
            if t.write {
                expected_writes.push((align(t.addr, t.size), t.size, t.data & size_mask(t.size)));
            } else {
                // **Data reads only.** A fetch transaction describes the refill
                // of the instructions *after* this one, which this core does
                // not perform — and the reference answers a fetch and a data
                // read at the same address with different values, because the
                // generator drew them from different random sources. An `LDM`
                // whose range covers an address the refill also touched would
                // otherwise read the fetch's value instead of the data one.
                if t.fetch {
                    continue;
                }
                // `addr` is the address the *core* drove, low bits and all;
                // the memory system ignores them and answers with the aligned
                // word or halfword, which is what `data` holds. Priming at the
                // unaligned address would put the bytes one or two lanes over
                // and every unaligned `LDR` would read rubbish.
                let base = align(t.addr, t.size);
                for i in 0..t.size {
                    let byte = (t.data >> (8 * i)) as u8;
                    memory.bytes.insert(base.wrapping_add(i), byte);
                }
            }
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
    let trace = tracing(index);
    {
        let mut m = bus.0.lock();
        m.writes.clear();
        if trace {
            m.trace = Some(Vec::new());
        }
    }
    if trace {
        println!("vector {index}: initial {}", cpu.regs());
    }

    cpu.step();

    if trace {
        for (write, addr, size, value) in bus.0.lock().trace.take().unwrap_or_default() {
            println!(
                "  {} {addr:08x} size {size} data {value:08x}",
                if write { "write" } else { "read " }
            );
        }
        println!("vector {index}: final   {}", cpu.regs());
    }

    let mut detail = String::new();
    let mut signature = Vec::new();
    let got = CpuState::from_regs(&cpu.regs());
    if got.r != expected.r {
        for (i, (a, b)) in got.r.iter().zip(expected.r.iter()).enumerate() {
            if a != b {
                detail.push_str(&format!("  r{i}: want {b:08x} got {a:08x}\n"));
                signature.push(format!("r{i}"));
            }
        }
    }
    if got.cpsr != expected.cpsr {
        detail.push_str(&format!(
            "  cpsr: want {:08x} got {:08x}\n",
            expected.cpsr, got.cpsr
        ));
        signature.push("cpsr".to_string());
    }
    if got.spsr != expected.spsr {
        detail.push_str(&format!(
            "  spsr: want {:08x?} got {:08x?}\n",
            expected.spsr, got.spsr
        ));
        signature.push("spsr".to_string());
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
            signature.push(format!("{name} bank"));
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
            signature.push("write".to_string());
        }
    }

    let opcode = vector.get("opcode").and_then(Json::num).unwrap_or(0) as u32;
    let signature = signature.join("+");
    let divergence =
        architectural_divergence(opcode, initial.cpsr & psr::T != 0, initial.cpsr, &signature);
    Ok((!detail.is_empty()).then_some(Failure {
        index,
        divergence,
        signature,
        detail,
    }))
}

/// The address a transaction of this size actually reaches on the bus.
///
/// A core drives `R15` or `Rn` unchanged and the memory system drops the low
/// address bits; the corpus records the former and answers with the latter.
fn align(addr: u32, size: u32) -> u32 {
    addr & !(size.saturating_sub(1))
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

/// Corpus files this runner refuses to measure against, and why.
///
/// **A corpus is evidence, not authority.** Where a vector disagrees with the
/// ARM ARM, the manual wins and the disagreement is written down here rather
/// than absorbed into the core — `ROADMAP.md` §0's "the interpreter is the
/// oracle", applied one level up. These are not known failures waiting to be
/// fixed: matching them would make the core *less* correct, so they are
/// excluded by name and the reason travels with the exclusion.
pub(super) const REJECTED_FILES: &[(&str, &str)] = &[(
    "thumb_undefined_bcc",
    "every vector in the file is wrong. ARM ARM A6.1 and A7.1.14: a Thumb \
     conditional branch with cond == 0b1110 is UNDEFINED (0b1111 is SWI), and \
     the architecture reserves it precisely so the encoding can be trapped. \
     The corpus treats cond 0b1110 as an unconditional branch. Reported \
     upstream as SingleStepTests/ARM7TDMI#2, and against the emulator the \
     vectors were generated from as nba-emu/NanoBoyAdvance#395. This core \
     raises the Undefined Instruction exception; see \
     `tests::an_undefined_thumb_encoding_is_still_undefined`.",
)];

/// Why a file is in [`REJECTED_FILES`].
fn rejection_reason(name: &str) -> &'static str {
    REJECTED_FILES
        .iter()
        .find(|(f, _)| *f == name)
        .map_or("", |(_, why)| *why)
}

/// Places where **ARMv4T and ARMv5TE genuinely differ**, so a vector the
/// corpus gets right for an ARM7TDMI is one this core must get differently.
///
/// Distinct from [`REJECTED_FILES`], which is for vectors that are simply
/// wrong. These are correct — for the wrong architecture. Neither is a
/// known-failures ledger in `ROADMAP.md` §0's sense: nothing here will ever be
/// "fixed", because fixing it would mean implementing ARMv4T.
///
/// A vector is only excused if it *also* actually failed; a divergence that
/// turns out not to matter for a given vector still counts as a pass.
fn architectural_divergence(
    opcode: u32,
    thumb: bool,
    cpsr: u32,
    signature: &str,
) -> Option<&'static str> {
    if thumb {
        // ARMv4's `MUL` corrupts `C`; ARMv5 leaves it alone (ARM ARM A7.1.41).
        // The Thumb encoding has the same difference as the ARM one.
        return match super::thumb::decode(opcode as u16) {
            super::thumb::Thumb::Alu {
                op: super::thumb::AluOp::Mul,
                ..
            } if signature == "cpsr" => Some("ARMv4 corrupts C on a Thumb MUL; ARMv5 preserves it"),
            // `BLX Rm` is an ARMv5T addition (ARM ARM A7.1.12). On an ARMv4T
            // the same encoding is `BX` with `H1` set, which sets no link
            // register — so the reference leaves `R14` alone where this core
            // writes the return address.
            super::thumb::Thumb::BranchExchange { link: true, .. }
                if link_register_only(signature) =>
            {
                Some("Thumb BLX Rm is an ARMv5T addition; ARMv4T has no link here")
            }
            // ARMv5 made a load into `R15` interwork on bit 0; ARMv4T stays in
            // whichever state it was in. `POP {pc}` is the common case, and an
            // empty Thumb register list is UNPREDICTABLE but loads `R15` on an
            // ARM7TDMI.
            super::thumb::Thumb::PushPop {
                load: true,
                extra,
                list,
            } if (extra || list == 0) && interworking_shaped(signature) => {
                Some("ARMv5 POP/LDM into R15 interworks on bit 0; ARMv4T does not")
            }
            super::thumb::Thumb::BlockTransfer {
                load: true,
                list: 0,
                ..
            } if interworking_shaped(signature) => {
                Some("ARMv5 POP/LDM into R15 interworks on bit 0; ARMv4T does not")
            }
            _ => None,
        };
    }
    // Reuse the real decoder rather than re-deriving bit patterns here: a
    // classifier that disagrees with the interpreter about what an encoding is
    // would excuse the wrong vectors.
    let decoded = super::isa::decode(opcode);
    if !decoded.passes(cpsr) {
        return None;
    }
    let has_spsr = Mode((cpsr & psr::MODE) as u8).spsr_index().is_some();
    match decoded.insn {
        // TSTP/TEQP/CMPP/CMNP: in ARMv4 and earlier, a compare with S set and
        // Rd == R15 copied SPSR into CPSR instead of setting the flags — the
        // 26-bit-mode compatibility form. ARMv5 removed it and leaves the
        // encoding UNPREDICTABLE with Rd as SBZ, so this core sets the flags
        // and ignores Rd (ARM ARM A4.1.x, the "TEQP" notes).
        super::isa::Insn::DataProc { op, s, rd, .. }
            if s && rd & 0xf == 15 && !op.writes_result() && has_spsr && signature == "cpsr" =>
        {
            Some(
                "ARMv4 TSTP/TEQP/CMPP/CMNP: the corpus restores CPSR from SPSR; \
                  ARMv5 removed the form and this core sets the flags",
            )
        }
        // ARMv5 made a load into R15 an interworking branch; ARMv4T branched
        // within ARM state and ignored bit 0 (ARM ARM A4.1.23, A4.1.20).
        super::isa::Insn::LoadStore { load: true, rd, .. }
            if rd & 0xf == 15 && interworking_shaped(signature) =>
        {
            Some("ARMv5 LDR into R15 interworks on bit 0; ARMv4T does not")
        }
        super::isa::Insn::BlockTransfer {
            load: true, list, ..
        } if list & 0x8000 != 0 && interworking_shaped(signature) => {
            Some("ARMv5 LDM with R15 in the list interworks on bit 0; ARMv4T does not")
        }
        // `LDR`/`STR`/`LDRH`/... with `Rn == R15` *and* base writeback, which
        // ARM ARM A5.2.5 and A5.3.4 call UNPREDICTABLE outright. The reference
        // is not self-consistent about it either: a plain register write to
        // R15 leaves it at `value + 4` there (which is what this core does,
        // and where we agree on `MRS`), a branch leaves it at `value + 8`
        // (agreed on `MOV pc`, `MUL`, `BX`), and this case alone lands at
        // `value + 12`. There is no architectural rule to prefer, so this core
        // keeps the one it can state.
        super::isa::Insn::LoadStore { rn, index, .. }
        | super::isa::Insn::LoadStoreExtra { rn, index, .. }
            if rn & 0xf == 15 && index.writes_base() && signature == "r15" =>
        {
            Some(
                "LDR/STR with Rn == R15 and writeback is UNPREDICTABLE; the                  reference's R15 bookkeeping for it disagrees with its own                  handling of every other write to R15",
            )
        }
        // An ARM-state `MSR` that sets the T bit. The reference does the
        // ordinary "+4" advance on its ARM-pipelined R15 and leaves it that
        // way while CPSR now says Thumb — where for `BX` it reloads the
        // pipeline properly and we agree exactly. That is an inconsistency in
        // the reference's bookkeeping for an operation the architecture calls
        // UNPREDICTABLE (A4.1.39's note on the T bit), not a difference in
        // what either model would execute next. Only `r15` can differ, which
        // is what the signature pins down.
        super::isa::Insn::Msr {
            spsr: false, mask, ..
        } if mask & 1 != 0 && signature == "r15" => Some(
            "MSR setting the T bit: the reference leaves R15 ARM-pipelined              while CPSR says Thumb, unlike its own BX",
        ),
        // ARMv4 left C UNPREDICTABLE (in practice destroyed) after MUL with S;
        // ARMv5 leaves it alone (ARM ARM A4.1.40).
        super::isa::Insn::Mul { s: true, .. } | super::isa::Insn::MulLong { s: true, .. }
            if signature == "cpsr" =>
        {
            Some("ARMv4 corrupts C on a flag-setting multiply; ARMv5 preserves it")
        }
        _ => None,
    }
}

/// Whether a failure touches only the link register, in whichever bank.
fn link_register_only(signature: &str) -> bool {
    signature
        .split('+')
        .all(|part| part == "r14" || part.ends_with(" bank"))
}

/// Whether a failure looks like the interworking difference and nothing else.
///
/// An excuse that swallows a wrong register value is worse than no excuse at
/// all, so the shape has to match: only the state a state change would move.
fn interworking_shaped(signature: &str) -> bool {
    signature
        .split('+')
        .all(|part| part == "cpsr" || part == "r15")
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
    let mut passed_files: Vec<(String, usize, usize, usize)> = Vec::new();

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
        if REJECTED_FILES.iter().any(|(f, _)| *f == name) {
            println!("{name}: skipped — {}", rejection_reason(&name));
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
        let mut excused: BTreeMap<&'static str, usize> = BTreeMap::new();
        for (index, vector) in vectors.iter().enumerate() {
            ran_vectors += 1;
            match run_vector(vector, index) {
                Ok(None) => {}
                Ok(Some(failure)) => match failure.divergence {
                    Some(why) => *excused.entry(why).or_default() += 1,
                    None => failures.push(failure),
                },
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
        let excused_total: usize = excused.values().sum();
        passed_files.push((
            name.clone(),
            vectors.len() - failures.len() - excused_total,
            excused_total,
            vectors.len(),
        ));
        for (why, count) in &excused {
            println!("{name}: {count} vectors excused — {why}");
        }
        if failures.is_empty() {
            println!(
                "{name}: {} of {} passed{}",
                vectors.len() - excused_total,
                vectors.len() - excused_total,
                if excused_total == 0 {
                    String::new()
                } else {
                    format!(" ({excused_total} excused)")
                }
            );
        } else {
            total_failures += failures.len();
            failing_files.push((name.clone(), failures.len()));
            println!(
                "{name}: {} of {} passed ({excused_total} excused); {} failed",
                vectors.len() - failures.len() - excused_total,
                vectors.len() - excused_total,
                failures.len(),
            );
            // Group by what differed. Fifty thousand diffs of the same shape
            // are one finding, and printing them as one is the difference
            // between a diagnosis and a wall of hex.
            let mut classes: BTreeMap<&str, (usize, &Failure)> = BTreeMap::new();
            for failure in &failures {
                let entry = classes
                    .entry(failure.signature.as_str())
                    .or_insert((0, failure));
                entry.0 += 1;
            }
            for (signature, (count, example)) in classes {
                println!("    [{signature}] x{count}, e.g. #{}:", example.index);
                print!("{}", example.detail);
            }
        }
    }

    println!("conformance: {ran_files} files, {ran_vectors} vectors, {total_failures} failures");
    for (name, passed, excused, total) in &passed_files {
        println!(
            "  {name}: {passed}/{} passed, {excused} excused, {total} in file",
            total - excused
        );
    }
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
