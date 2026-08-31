//! The conformance runner for `SingleStepTests/8088`.
//!
//! `ROADMAP.md` §0: *accuracy is measured, never asserted*. On x86 there is no
//! other honest option, and this is the measurement. The corpus is not one
//! emulator checked against another: it was captured from an **AMD D8088 dated
//! 1982**, running in maximum mode with an Arduino interposer driving the
//! pins, so what it records is what the silicon did — undefined flags
//! included.
//!
//! Ten thousand vectors per opcode (fewer for the string and shift forms,
//! which are long), each one an initial register file, an initial memory
//! image, the initial prefetch queue, the expected final state, and the
//! complete cycle-by-cycle bus trace.
//!
//! # Running it
//!
//! The corpus is **downloaded, never vendored** (`ROADMAP.md` §1, §12): it is
//! several gigabytes with its own licence (MIT, © 2024 SingleStepTests, and
//! confirmed as such), and shipping it here would be redistribution. The test
//! is gated on an environment variable naming a directory of decompressed
//! `NN.json` files:
//!
//! ```text
//! mkdir -p /tmp/8088 && cd /tmp/8088
//! curl -sL https://api.github.com/repos/SingleStepTests/8088/contents/v2 \
//!   | grep download_url | cut -d'"' -f4 \
//!   | xargs -P8 -I{} sh -c 'curl -sL {} | gzip -dc > $(basename {} .gz)'
//! RSEMU_8088_DIR=/tmp/8088 cargo test --release --all-features x86::conformance -- --nocapture
//! ```
//!
//! Without the variable the test prints why it did nothing and passes, so
//! `cargo test` stays hermetic and offline.
//!
//! Two further variables:
//!
//! - `RSEMU_8088_OPCODES=F6.6,D4` runs only the named files.
//! - `RSEMU_8088_BUS=1` also checks the **data bus trace**: every memory and
//!   I/O access the instruction made, in order, with its address and value,
//!   pulled out of the corpus's per-cycle capture. See below for why the code
//!   fetches are excluded.
//!
//! # What is checked, and what is not
//!
//! Checked: the final register file including every flag bit, the final
//! contents of every memory location the vector names, and — under
//! `RSEMU_8088_BUS` — the ordered sequence of operand reads, operand writes
//! and I/O transfers.
//!
//! Not checked: T-states, and therefore the instruction's duration and the
//! interleaving of prefetches with operand accesses. Reproducing those needs a
//! model of the bus interface unit's overlap with the execution unit, which
//! this core deliberately does not have yet ([`exec`](super)). The **code
//! fetches are filtered out of the trace comparison for exactly that reason**:
//! *which* bytes are fetched is determined by the instruction, but *when* is
//! determined by timing this core does not simulate. Everything left is
//! determined by the instruction's semantics alone, so it is a fair gate — and
//! a strict one, since a wrong operand order or a phantom access fails it.
//!
//! # The measurement
//!
//! At the commit that added this file, with `RSEMU_8088_BUS=1` and the whole
//! `v2` corpus — 323 opcode files, 3 007 000 vectors:
//!
//! ```text
//! conformance: 2974160 of 3007000 vectors passed across 323 opcode files
//!              (registers, memory and the operand bus trace)
//! ```
//!
//! Every failure is in the same place: the flags Intel documents as
//! *undefined* after `IMUL`, `DIV` and `IDIV`. All 317 other opcode files pass
//! every vector, flags included. [`KNOWN_FAILURES`] is the ledger
//! `ROADMAP.md` §0 asks for, annotated with what is missing and why.
//!
//! # Why the JSON parser is in here
//!
//! The dependency policy allows no `serde` (`ROADMAP.md` §0). This one is
//! purpose-built rather than general: it streams one vector at a time so a
//! 40 MiB opcode file never exists as a parse tree, and it skips the `cycles`
//! array without allocating unless the bus check asked for it. That is most of
//! why the run takes minutes rather than hours.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use std::path::Path;

use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, UnassignedPolicy,
};
use crate::core::sync::{self, LockRank};

use super::{Config, I8086, Reg, Regs};

/// Opcodes this core does not yet match, with the reason and the count at the
/// commit that measured it.
///
/// `ROADMAP.md` §0 asks for a ledger that only ever shrinks. Every entry here
/// is the same gap seen from six angles: the flags Intel documents as
/// *undefined* after a multiply or a divide. Everything else about these
/// instructions — quotient, remainder, product, the divide-error vector, the
/// return address it pushes, and the whole operand bus trace — matches the
/// hardware on every vector.
///
/// The undefined flags after `MUL` are a simple function of the result and are
/// modelled exactly ([`exec::Exec::mul_flags`](super)). After `IMUL` and the
/// divides they are not: they are the residue of the microcoded shift-add and
/// shift-subtract loops, and reproducing them needs those loops step for step
/// rather than the approximation in [`exec`](super). Closing this means
/// modelling the 8086's multiply and divide microcode, which is a piece of
/// work in its own right and is deliberately not in this milestone.
const KNOWN_FAILURES: &[(&str, &str)] = &[
    (
        "F6.5",
        "IMUL r/m8: sign, zero, parity and auxiliary results, all documented \
         as undefined, after a signed multiply whose microcode adjusts signs \
         around the magnitude loop (3414 of 10000 vectors)",
    ),
    ("F7.5", "IMUL r/m16: as F6.5 (3499 of 10000 vectors)"),
    (
        "F6.6",
        "DIV r/m8: the six documented-undefined arithmetic flags after the \
         division loop. The carry is exact; the rest are the last trial \
         subtraction's, which is close but not the microcode's own final \
         state (5855 of 10000 vectors)",
    ),
    ("F7.6", "DIV r/m16: as F6.6 (5782 of 10000 vectors)"),
    (
        "F6.7",
        "IDIV r/m8: as F6.6, plus the sign-correction steps the signed divide \
         performs after the loop (7157 of 10000 vectors)",
    ),
    ("F7.7", "IDIV r/m16: as F6.7 (7133 of 10000 vectors)"),
];

// ---------------------------------------------------------------------------
// A purpose-built reader for this corpus
// ---------------------------------------------------------------------------

/// One operand or I/O transfer, as the corpus's cycle trace records it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Access {
    /// Physical address for memory, port number for I/O.
    addr: u32,
    value: u8,
    write: bool,
    io: bool,
}

impl core::fmt::Display for Access {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "[{}{:05x} {:02x} {}]",
            if self.io { "io:" } else { "" },
            self.addr,
            self.value,
            if self.write { "w" } else { "r" }
        )
    }
}

/// Half of a vector: the registers, memory and queue at one instant.
#[derive(Debug, Default)]
struct Snapshot {
    /// `(register name, value)`. The `final` half lists only what changed.
    regs: Vec<(String, u16)>,
    ram: Vec<(u32, u8)>,
    queue: Vec<u8>,
}

/// One test vector.
#[derive(Debug, Default)]
struct Vector {
    name: String,
    initial: Snapshot,
    expected: Snapshot,
    accesses: Vec<Access>,
}

struct Parser<'a> {
    bytes: &'a [u8],
    at: usize,
    /// Whether to decode the `cycles` array or skip past it.
    want_cycles: bool,
}

impl<'a> Parser<'a> {
    fn new(bytes: &'a [u8], want_cycles: bool) -> Parser<'a> {
        Parser {
            bytes,
            at: 0,
            want_cycles,
        }
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

    fn eat(&mut self, byte: u8) {
        let got = self.peek();
        assert_eq!(
            got, byte,
            "expected `{}` at offset {}",
            byte as char, self.at
        );
        self.at += 1;
    }

    /// A string, returned as a borrowed slice — the corpus has no escapes.
    fn str_ref(&mut self) -> &'a str {
        self.eat(b'"');
        let start = self.at;
        while self.bytes.get(self.at).is_some_and(|b| *b != b'"') {
            self.at += 1;
        }
        let text = core::str::from_utf8(&self.bytes[start..self.at]).unwrap_or("");
        self.at += 1;
        text
    }

    fn number(&mut self) -> i64 {
        self.skip_space();
        let start = self.at;
        if self.bytes.get(self.at) == Some(&b'-') {
            self.at += 1;
        }
        while matches!(self.bytes.get(self.at), Some(b'0'..=b'9')) {
            self.at += 1;
        }
        core::str::from_utf8(&self.bytes[start..self.at])
            .expect("ascii")
            .parse()
            .expect("integer")
    }

    /// Consume one value of any shape without building anything from it.
    fn skip_value(&mut self) {
        match self.peek() {
            b'{' | b'[' => {
                let close = if self.peek() == b'{' { b'}' } else { b']' };
                self.at += 1;
                if self.peek() == close {
                    self.at += 1;
                    return;
                }
                loop {
                    if close == b'}' {
                        let _ = self.str_ref();
                        self.eat(b':');
                    }
                    self.skip_value();
                    if self.peek() == b',' {
                        self.at += 1;
                    } else {
                        break;
                    }
                }
                self.eat(close);
            }
            b'"' => {
                let _ = self.str_ref();
            }
            b't' => self.at += 4,
            b'f' => self.at += 5,
            b'n' => self.at += 4,
            _ => {
                let _ = self.number();
            }
        }
    }

    /// `[[addr, value], ...]`
    fn cell_list(&mut self) -> Vec<(u32, u8)> {
        let mut out = Vec::new();
        self.eat(b'[');
        if self.peek() == b']' {
            self.at += 1;
            return out;
        }
        loop {
            self.eat(b'[');
            let addr = self.number() as u32;
            self.eat(b',');
            let value = self.number() as u8;
            self.eat(b']');
            out.push((addr, value));
            if self.peek() == b',' {
                self.at += 1;
            } else {
                break;
            }
        }
        self.eat(b']');
        out
    }

    /// `[n, n, ...]`
    fn byte_list(&mut self) -> Vec<u8> {
        let mut out = Vec::new();
        self.eat(b'[');
        if self.peek() == b']' {
            self.at += 1;
            return out;
        }
        loop {
            out.push(self.number() as u8);
            if self.peek() == b',' {
                self.at += 1;
            } else {
                break;
            }
        }
        self.eat(b']');
        out
    }

    fn snapshot(&mut self) -> Snapshot {
        let mut snap = Snapshot::default();
        self.eat(b'{');
        loop {
            let key = self.str_ref();
            self.eat(b':');
            match key {
                "regs" => {
                    self.eat(b'{');
                    if self.peek() == b'}' {
                        self.at += 1;
                    } else {
                        loop {
                            let name = self.str_ref().to_string();
                            self.eat(b':');
                            let value = self.number() as u16;
                            snap.regs.push((name, value));
                            if self.peek() == b',' {
                                self.at += 1;
                            } else {
                                break;
                            }
                        }
                        self.eat(b'}');
                    }
                }
                "ram" => snap.ram = self.cell_list(),
                "queue" => snap.queue = self.byte_list(),
                _ => self.skip_value(),
            }
            if self.peek() == b',' {
                self.at += 1;
            } else {
                break;
            }
        }
        self.eat(b'}');
        snap
    }

    /// Pull the operand and I/O transfers out of the per-cycle capture.
    ///
    /// The address is latched on ALE (bit 0 of the pin field) and the transfer
    /// completes on T3, when the 8288's read or advanced-write strobe is
    /// active and the data bus is valid. Code fetches are dropped: see the
    /// module docs.
    fn cycles(&mut self) -> Vec<Access> {
        let mut out = Vec::new();
        // Nothing is attributed until the first address latch: a vector whose
        // queue was pre-filled begins mid-way through the bus cycle that
        // filled it, and that cycle belongs to the harness's setup rather
        // than to the instruction.
        let mut latched: Option<(u32, String)> = None;
        self.eat(b'[');
        if self.peek() == b']' {
            self.at += 1;
            return out;
        }
        loop {
            self.eat(b'[');
            let pin = self.number();
            self.eat(b',');
            let bus = self.number() as u32;
            self.eat(b',');
            let _segment = self.str_ref();
            self.eat(b',');
            let memory = self.str_ref().as_bytes().to_vec();
            self.eat(b',');
            let io = self.str_ref().as_bytes().to_vec();
            self.eat(b',');
            let _bhe = self.number();
            self.eat(b',');
            let data = self.number() as u8;
            self.eat(b',');
            let bus_status = self.str_ref();
            self.eat(b',');
            let t_state = self.str_ref();
            self.eat(b',');
            let _queue_op = self.str_ref();
            self.eat(b',');
            let _queue_byte = self.number();
            self.eat(b']');

            if pin & 1 != 0 {
                latched = Some((bus, bus_status.to_string()));
            }
            if let Some((address, ref status)) = latched
                && (t_state == "T3" || t_state == "Tw")
                && status != "CODE"
            {
                let mem_read = memory.first() == Some(&b'R');
                let mem_write = memory.get(1) == Some(&b'A');
                let io_read = io.first() == Some(&b'R');
                let io_write = io.get(1) == Some(&b'A');
                if mem_read || mem_write || io_read || io_write {
                    out.push(Access {
                        addr: address,
                        value: data,
                        write: mem_write || io_write,
                        io: io_read || io_write,
                    });
                }
            }

            if self.peek() == b',' {
                self.at += 1;
            } else {
                break;
            }
        }
        self.eat(b']');
        out
    }

    /// One vector object.
    fn vector(&mut self) -> Vector {
        let mut v = Vector::default();
        self.eat(b'{');
        loop {
            let key = self.str_ref();
            self.eat(b':');
            match key {
                "name" => v.name = self.str_ref().to_string(),
                "initial" => v.initial = self.snapshot(),
                "final" => v.expected = self.snapshot(),
                "cycles" => {
                    if self.want_cycles {
                        v.accesses = self.cycles();
                    } else {
                        self.skip_value();
                    }
                }
                _ => self.skip_value(),
            }
            if self.peek() == b',' {
                self.at += 1;
            } else {
                break;
            }
        }
        self.eat(b'}');
        v
    }
}

// ---------------------------------------------------------------------------
// The bus the vectors describe
// ---------------------------------------------------------------------------

#[derive(Debug)]
struct BusState {
    cells: Vec<u8>,
    log: Vec<Access>,
    /// Every address written since the last reset, so a vector can be undone
    /// without clearing a megabyte.
    dirty: Vec<u32>,
    io: bool,
}

/// A recording memory or I/O space.
///
/// Reads of the I/O space return `0xff`, which is what the corpus's notes say
/// a bare 8088 with nothing on the bus produced.
#[derive(Debug)]
struct VectorBus(sync::Mutex<BusState>);

impl VectorBus {
    fn new(len: usize, io: bool) -> VectorBus {
        VectorBus(sync::Mutex::with_rank(
            LockRank::DEVICE,
            BusState {
                cells: alloc::vec![if io { 0xff } else { 0x00 }; len],
                log: Vec::new(),
                dirty: Vec::new(),
                io,
            },
        ))
    }

    /// Undo everything the previous vector did.
    fn reset(&self, initial: &[(u32, u8)]) {
        let mut m = self.0.lock();
        let fill = if m.io { 0xff } else { 0x00 };
        for addr in core::mem::take(&mut m.dirty) {
            m.cells[addr as usize] = fill;
        }
        m.log.clear();
        for (addr, value) in initial {
            m.cells[*addr as usize] = *value;
            m.dirty.push(*addr);
        }
    }

    fn cell(&self, addr: u32) -> u8 {
        self.0.lock().cells[addr as usize]
    }

    fn take_log(&self) -> Vec<Access> {
        core::mem::take(&mut self.0.lock().log)
    }

    /// Every address written since the last reset, oldest first.
    fn written(&self) -> Vec<u32> {
        self.0.lock().dirty.clone()
    }
}

impl MemOps for VectorBus {
    fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
        let mut m = self.0.lock();
        let io = m.io;
        for (i, slot) in dst.iter_mut().enumerate() {
            let addr = (offset as usize + i) % m.cells.len();
            *slot = m.cells[addr];
            if !attrs.debug {
                let value = *slot;
                m.log.push(Access {
                    addr: addr as u32,
                    value,
                    write: false,
                    io,
                });
            }
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
        let mut m = self.0.lock();
        let io = m.io;
        for (i, byte) in src.iter().enumerate() {
            let addr = (offset as usize + i) % m.cells.len();
            m.cells[addr] = *byte;
            m.dirty.push(addr as u32);
            if !attrs.debug {
                m.log.push(Access {
                    addr: addr as u32,
                    value: *byte,
                    write: true,
                    io,
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

/// Apply one `"name": value` pair from the corpus to a register file.
fn apply_reg(regs: &mut Regs, name: &str, value: u16) {
    let reg = Reg::from_name(name).unwrap_or_else(|| panic!("unknown register `{name}`"));
    reg.set(regs, value);
}

/// The corpus's register order, for a readable diff.
fn regs_from(base: Regs, list: &[(String, u16)]) -> Regs {
    let mut regs = base;
    for (name, value) in list {
        apply_reg(&mut regs, name, *value);
    }
    regs
}

/// Run every vector in one opcode's file, returning the failures.
fn run_file(path: &Path, cfg: Config, check_bus: bool) -> (usize, Vec<Failure>) {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let mut parser = Parser::new(&bytes, check_bus);
    let mut failures = Vec::new();
    let mut ran = 0usize;

    let memory = alloc::sync::Arc::new(VectorBus::new(0x10_0000, false));
    let ports = alloc::sync::Arc::new(VectorBus::new(0x1_0000, true));

    let mem_space = AddressSpace::new("mem", 20).with_unassigned(UnassignedPolicy::FAULT);
    mem_space
        .topology()
        .map(Region::io("ram", 0x10_0000, memory.clone()), 0)
        .expect("1 MiB fits in 20 bits");
    let io_space = AddressSpace::new("io", 16).with_unassigned(UnassignedPolicy::FAULT);
    io_space
        .topology()
        .map(Region::io("ports", 0x1_0000, ports.clone()), 0)
        .expect("64 KiB fits in 16 bits");
    let mem_space = alloc::sync::Arc::new(mem_space);
    let io_space = alloc::sync::Arc::new(io_space);

    parser.eat(b'[');
    if parser.peek() == b']' {
        return (0, failures);
    }
    loop {
        let vector = parser.vector();
        ran += 1;

        memory.reset(&vector.initial.ram);
        ports.reset(&[]);

        let cpu = I8086::new(cfg);
        cpu.attach_space(mem_space.clone());
        cpu.attach_io_space(io_space.clone());
        let initial = regs_from(Regs::new(), &vector.initial.regs);
        {
            // The vectors start mid-program: no reset sequence, and the
            // interrupt lines are not modelled by the corpus at all.
            let mut session = cpu.session.lock();
            session.state.reset_pending = false;
            session.state.regs = initial;
        }
        cpu.set_prefetch_queue(&vector.initial.queue)
            .expect("the corpus never over-fills the queue");
        memory.take_log();
        ports.take_log();

        cpu.step();

        let mut detail = String::new();
        let got = cpu.regs();
        let want = regs_from(initial, &vector.expected.regs);
        if got != want {
            detail.push_str(&format!("  regs want {want}\n       got  {got}\n"));
            for reg in Reg::ALL {
                let (a, b) = (reg.get(&want), reg.get(&got));
                if a != b {
                    detail.push_str(&format!("    {reg}: want {a:04x} got {b:04x}\n"));
                }
            }
        }

        for (addr, value) in &vector.expected.ram {
            let actual = memory.cell(*addr);
            if actual != *value {
                detail.push_str(&format!(
                    "  ram[{addr:05x}] want {value:02x} got {actual:02x}\n"
                ));
            }
        }
        // A cell the vector does not mention must still hold what it started
        // with. Without this a stray write outside the listed cells would pass
        // unnoticed, which is precisely the bug an addressing-mode mistake
        // produces.
        for addr in memory.written() {
            if vector.expected.ram.iter().any(|(a, _)| *a == addr) {
                continue;
            }
            let want = vector
                .initial
                .ram
                .iter()
                .find(|(a, _)| *a == addr)
                .map_or(0, |(_, v)| *v);
            let actual = memory.cell(addr);
            if actual != want {
                detail.push_str(&format!(
                    "  ram[{addr:05x}] was not supposed to change: want {want:02x} \
                     got {actual:02x}\n"
                ));
            }
        }

        let mut got: Vec<Access> = memory.take_log();
        got.extend(ports.take_log());
        if check_bus {
            let want = &vector.accesses;
            let got = strip_prefetch(got, want);
            if got != *want {
                let show = |cs: &[Access]| {
                    cs.iter()
                        .map(ToString::to_string)
                        .collect::<Vec<_>>()
                        .join(" ")
                };
                detail.push_str(&format!(
                    "  bus want {}\n      got  {}\n",
                    show(want),
                    show(&got)
                ));
            }
        }

        if !detail.is_empty() {
            failures.push(Failure {
                name: vector.name.clone(),
                detail,
            });
        }

        if parser.peek() == b',' {
            parser.at += 1;
        } else {
            break;
        }
    }
    parser.eat(b']');
    (ran, failures)
}

/// Drop this core's instruction fetches from a recorded access list.
///
/// The corpus tags each bus cycle with the 8088's status lines, so it knows a
/// `CODE` fetch when it sees one; an `AddressSpace` carries no such tag, and
/// inventing one in `core::space` for a single test would be exactly the
/// device-specific leak `ROADMAP.md` §0 forbids. So the fetches are separated
/// structurally, by walking both lists together: an access that matches the
/// next expected one is kept, a write or an I/O transfer is *always* kept
/// because a fetch is neither, and any other read is taken to be a prefetch
/// and dropped.
///
/// What that gate catches: a missing operand access, one with the wrong
/// address or value, one in the wrong order, and every spurious or missing
/// write. What it cannot catch: a spurious operand *read*, which is
/// indistinguishable from the prefetching this core does not schedule the way
/// hardware does.
fn strip_prefetch(got: Vec<Access>, want: &[Access]) -> Vec<Access> {
    let mut out = Vec::with_capacity(want.len());
    let mut next = 0usize;
    for access in got {
        if next < want.len() && want[next] == access {
            out.push(access);
            next += 1;
        } else if access.write || access.io {
            out.push(access);
        }
    }
    out
}

/// Run the whole corpus, or explain why it did not.
///
/// Not `#[ignore]`d: a skipped test that says nothing is how a suite quietly
/// stops running. This one prints the command that would have run it.
#[test]
fn single_step_tests() {
    let Ok(dir) = std::env::var("RSEMU_8088_DIR") else {
        println!(
            "conformance: set RSEMU_8088_DIR to a decompressed SingleStepTests/8088 \
             v2 directory to run 10 000 vectors per opcode; see the module docs \
             for the four-line fetch command"
        );
        return;
    };
    let dir = Path::new(&dir);
    let check_bus = std::env::var("RSEMU_8088_BUS").is_ok_and(|v| v != "0");
    let only = std::env::var("RSEMU_8088_OPCODES").ok();

    let mut names: Vec<String> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
        .filter_map(|entry| {
            let name = entry.ok()?.file_name().into_string().ok()?;
            let stem = name.strip_suffix(".json")?;
            (stem != "metadata").then(|| stem.to_string())
        })
        .collect();
    names.sort();
    assert!(
        !names.is_empty(),
        "no NN.json files under {}",
        dir.display()
    );

    let mut files = 0usize;
    let mut total = 0usize;
    let mut total_failures = 0usize;
    let mut failed: Vec<(String, usize)> = Vec::new();

    for name in &names {
        if let Some(only) = &only
            && !only.split(',').any(|o| o.trim().eq_ignore_ascii_case(name))
        {
            continue;
        }
        let (ran, failures) = run_file(&dir.join(format!("{name}.json")), Config::I8088, check_bus);
        files += 1;
        total += ran;
        if !failures.is_empty() {
            total_failures += failures.len();
            failed.push((name.clone(), failures.len()));
            println!(
                "{name}: {} of {ran} vectors failed; first is `{}`:\n{}",
                failures.len(),
                failures[0].name,
                failures[0].detail
            );
        }
    }

    println!(
        "conformance: {} of {total} vectors passed across {files} opcode files{}",
        total - total_failures,
        if check_bus {
            " (registers, memory and the operand bus trace)"
        } else {
            " (registers and memory; set RSEMU_8088_BUS=1 to add the bus trace)"
        }
    );
    let expected: Vec<&str> = KNOWN_FAILURES.iter().map(|(name, _)| *name).collect();
    let (known, unexpected): (Vec<_>, Vec<_>) = failed
        .iter()
        .partition(|(name, _)| expected.contains(&name.as_str()));
    if !known.is_empty() {
        let counted: usize = known.iter().map(|(_, n)| *n).sum();
        println!(
            "conformance: {counted} of those are the known undefined-flag gap on \
             {} opcode(s); see KNOWN_FAILURES",
            known.len()
        );
    }
    assert!(
        unexpected.is_empty(),
        "opcodes failing outside the ledger: {unexpected:?}"
    );
}
