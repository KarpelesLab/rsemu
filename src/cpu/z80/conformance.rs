//! The conformance runner for `SingleStepTests/z80`.
//!
//! `ROADMAP.md` §0: *accuracy is measured, never asserted*. This is the
//! measurement. The corpus is 1 000 vectors for each of 1 604 encodings —
//! every base, `CB`, `ED`, `DD`, `FD`, `DDCB` and `FDCB` instruction that does
//! anything — and each vector carries an initial machine state, the expected
//! final one, **a T-state-by-T-state bus trace** and, for the I/O
//! instructions, the port transaction. The trace is the part that makes it
//! worth having: a misplaced internal cycle fails here and passes every
//! hand-written test.
//!
//! The vectors also carry three registers no programming model mentions —
//! `wz`, `q` and `p` — which is what makes this suite able to check
//! [`MEMPTR`](super::Regs::wz), the `SCF`/`CCF` flag latch and the `LD A,I`
//! parity quirk directly rather than by inference.
//!
//! # Running it
//!
//! The corpus is **downloaded, never vendored** (`ROADMAP.md` §1, §12): it is
//! ~1.3 GiB of JSON with its own licence (MIT, © 2024 SingleStepTests), and
//! shipping it in this repository would be redistribution. So the test is
//! gated on an environment variable naming the directory of `*.json` files:
//!
//! ```text
//! git clone --depth 1 https://github.com/SingleStepTests/z80 /tmp/z80
//! RSEMU_Z80_DIR=/tmp/z80/v1 cargo test --all-features z80::conformance -- --nocapture
//! ```
//!
//! Without the variable the test prints why it did nothing and passes, so
//! `cargo test` stays hermetic and offline. `RSEMU_Z80_FILES` restricts the
//! run to a comma-separated list of file stems (`"00,dd cb __ 06"`) while
//! chasing one encoding.
//!
//! # What is compared
//!
//! Everything the vector states: all sixteen visible registers, the shadow
//! set, `I`, `R`, `WZ`, both interrupt flip-flops, the interrupt mode, the
//! `EI`, `Q` and `LD A,I` latches, every named RAM cell, the port
//! transaction, and the full T-state trace expanded from
//! [`CycleLog`](super::CycleLog).
//!
//! # The ledger
//!
//! `ROADMAP.md` §0 asks every core to ship a known-failures ledger that only
//! ever shrinks. It is at the bottom of this file, in [`KNOWN_FAILURES`], and
//! the test fails if a file *outside* it fails. It is **empty**: at the commit
//! that added this file the core passed all 1 604 000 vectors, and both
//! `zexdoc` and `zexall` ran clean ([`zex_exerciser`]).
//!
//! Not covered by the corpus, and therefore only by the hand-written tests
//! next door: `RESET`, `NMI`, `INT` in all three modes, and the `HALT` state.
//! The vectors start mid-program with the interrupt lines idle.
//!
//! # Why the JSON parser is in here
//!
//! The dependency policy allows no `serde` (`ROADMAP.md` §0), and this is the
//! only thing in this module that reads JSON. A hundred lines of recursive
//! descent over a format this regular is cheaper than the argument.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use std::path::Path;

use crate::core::space::{
    AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, UnassignedPolicy,
};
use crate::core::sync::{self, LockRank};

use super::{Config, MCycle, Regs, Z80};

// ---------------------------------------------------------------------------
// A very small JSON reader
// ---------------------------------------------------------------------------

/// Just enough of JSON for this corpus.
#[derive(Debug, Clone, PartialEq)]
enum Json {
    Null,
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

    /// A pin value, where `null` means "the bus was disconnected and the value
    /// does not matter".
    fn opt_num(&self) -> Option<i64> {
        match self {
            Json::Null => None,
            Json::Num(n) => Some(*n),
            other => panic!("expected a number or null, got {other:?}"),
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
                // `null` marks a pin whose value is electrically undefined.
                self.at += 4;
                Json::Null
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
// The buses the vectors describe
// ---------------------------------------------------------------------------

/// 64 KiB of RAM that answers every address, so a vector's expectations are
/// about the CPU and never about the memory map.
#[derive(Debug)]
struct VectorRam(sync::Mutex<Vec<u8>>);

impl MemOps for VectorRam {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let m = self.0.lock();
        for (i, slot) in dst.iter_mut().enumerate() {
            let addr = (offset as u16).wrapping_add(i as u16);
            *slot = m[addr as usize];
        }
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let mut m = self.0.lock();
        for (i, byte) in src.iter().enumerate() {
            let addr = (offset as u16).wrapping_add(i as u16);
            m[addr as usize] = *byte;
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

/// One I/O transaction, as the vectors record them.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Port {
    addr: u16,
    value: u8,
    write: bool,
}

impl core::fmt::Display for Port {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "[{:04x} {:02x} {}]",
            self.addr,
            self.value,
            if self.write { "w" } else { "r" }
        )
    }
}

/// The port space, which records what was asked of it.
///
/// A read answers with the value the vector says the device drove, which the
/// runner plants before the step; a vector performs at most one transaction.
#[derive(Debug)]
struct VectorPorts(sync::Mutex<(Option<u8>, Vec<Port>)>);

impl MemOps for VectorPorts {
    fn read(&self, offset: u64, dst: &mut [u8], _attrs: MemAttrs) -> MemResult {
        let mut m = self.0.lock();
        let value = m.0.unwrap_or(0xff);
        for slot in dst.iter_mut() {
            *slot = value;
        }
        m.1.push(Port {
            addr: offset as u16,
            value,
            write: false,
        });
        Ok(())
    }

    fn write(&self, offset: u64, src: &[u8], _attrs: MemAttrs) -> MemResult {
        let mut m = self.0.lock();
        for (i, byte) in src.iter().enumerate() {
            m.1.push(Port {
                addr: (offset as u16).wrapping_add(i as u16),
                value: *byte,
                write: true,
            });
        }
        Ok(())
    }

    fn constraints(&self) -> AccessConstraints {
        AccessConstraints::ANY
    }
}

// ---------------------------------------------------------------------------
// Expanding a cycle log into the corpus's T-state trace
// ---------------------------------------------------------------------------

/// One T-state of pin state, in the corpus's own encoding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Pins {
    /// `None` where the corpus wrote `null` and the value is undefined.
    addr: Option<u16>,
    value: Option<u8>,
    /// The `"rwmi"` string, as four bits: read, write, MREQ, IORQ.
    flags: u8,
}

const PIN_READ: u8 = 0b1000;
const PIN_WRITE: u8 = 0b0100;
const PIN_MREQ: u8 = 0b0010;
const PIN_IORQ: u8 = 0b0001;

impl core::fmt::Display for Pins {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let ch = |bit: u8, c: char| if self.flags & bit != 0 { c } else { '-' };
        write!(f, "[")?;
        match self.addr {
            Some(a) => write!(f, "{a:04x} ")?,
            None => write!(f, "---- ")?,
        }
        match self.value {
            Some(v) => write!(f, "{v:02x} ")?,
            None => write!(f, "-- ")?,
        }
        write!(
            f,
            "{}{}{}{}]",
            ch(PIN_READ, 'r'),
            ch(PIN_WRITE, 'w'),
            ch(PIN_MREQ, 'm'),
            ch(PIN_IORQ, 'i')
        )
    }
}

fn parse_pins(item: &Json) -> Pins {
    let fields = item.arr();
    let flags = fields[2].str().bytes().fold(0u8, |acc, b| {
        acc | match b {
            b'r' => PIN_READ,
            b'w' => PIN_WRITE,
            b'm' => PIN_MREQ,
            b'i' => PIN_IORQ,
            _ => 0,
        }
    });
    Pins {
        addr: fields[0].opt_num().map(|n| n as u16),
        value: fields[1].opt_num().map(|n| n as u8),
        flags,
    }
}

/// Expand the core's M-cycle log into the per-T-state trace the corpus keeps.
///
/// The shapes are fixed by the bus timing and are the whole point of the
/// comparison: a fetch drives its address for two T-states and the refresh
/// address for two more; a read latches its byte on the last T-state and a
/// write drives it on the middle one; an I/O cycle carries the extra wait
/// state the Z80 inserts for it.
fn expand(log: &super::CycleLog) -> Vec<Pins> {
    let mut out = Vec::with_capacity(24);
    for c in log.cycles() {
        let a = Some(c.addr);
        let r = Some(c.refresh);
        match c.kind {
            MCycle::Fetch | MCycle::Ack => {
                let request = if c.kind == MCycle::Fetch {
                    PIN_READ | PIN_MREQ
                } else {
                    PIN_READ | PIN_IORQ
                };
                out.push(Pins {
                    addr: a,
                    value: None,
                    flags: 0,
                });
                // An acknowledge is an M1 stretched by two automatic wait
                // states, and the corpus shows them before the request.
                for _ in 0..(c.tstates - 4) {
                    out.push(Pins {
                        addr: a,
                        value: None,
                        flags: 0,
                    });
                }
                out.push(Pins {
                    addr: a,
                    value: None,
                    flags: request,
                });
                out.push(Pins {
                    addr: r,
                    value: Some(c.value),
                    flags: 0,
                });
                out.push(Pins {
                    addr: r,
                    value: None,
                    flags: 0,
                });
            }
            MCycle::Read => {
                out.push(Pins {
                    addr: a,
                    value: None,
                    flags: 0,
                });
                out.push(Pins {
                    addr: a,
                    value: None,
                    flags: PIN_READ | PIN_MREQ,
                });
                out.push(Pins {
                    addr: a,
                    value: Some(c.value),
                    flags: 0,
                });
            }
            MCycle::Write => {
                out.push(Pins {
                    addr: a,
                    value: None,
                    flags: 0,
                });
                out.push(Pins {
                    addr: a,
                    value: Some(c.value),
                    flags: PIN_WRITE | PIN_MREQ,
                });
                out.push(Pins {
                    addr: a,
                    value: None,
                    flags: 0,
                });
            }
            MCycle::PortRead | MCycle::PortWrite => {
                let write = c.kind == MCycle::PortWrite;
                out.push(Pins {
                    addr: a,
                    value: None,
                    flags: 0,
                });
                out.push(Pins {
                    addr: a,
                    value: None,
                    flags: 0,
                });
                out.push(Pins {
                    addr: a,
                    value: if write { Some(c.value) } else { None },
                    flags: if write { PIN_WRITE } else { PIN_READ } | PIN_IORQ,
                });
                out.push(Pins {
                    addr: a,
                    value: if write { None } else { Some(c.value) },
                    flags: 0,
                });
            }
            MCycle::Internal => {
                for _ in 0..c.tstates {
                    out.push(Pins {
                        addr: a,
                        value: None,
                        flags: 0,
                    });
                }
            }
        }
    }
    out
}

/// Compare one T-state against the corpus, honouring its `null` wildcards.
fn pins_match(want: &Pins, got: &Pins) -> bool {
    if want.flags != got.flags {
        return false;
    }
    if let Some(a) = want.addr
        && Some(a) != got.addr
    {
        return false;
    }
    // A `null` data pin means "disconnected, so the value does not matter" —
    // the corpus's own README says so, and honouring it is what lets one
    // trace format describe both wait states and real transfers.
    if let Some(v) = want.value
        && Some(v) != got.value
    {
        return false;
    }
    true
}

// ---------------------------------------------------------------------------
// One vector
// ---------------------------------------------------------------------------

/// The whole visible machine state a vector names.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Snapshot {
    regs: Regs,
    iff1: bool,
    iff2: bool,
    im: u8,
    ei: bool,
    p: bool,
    q: u8,
}

impl core::fmt::Display for Snapshot {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "{} iff={}{} im={} ei={} p={} q={:02x}",
            self.regs,
            u8::from(self.iff1),
            u8::from(self.iff2),
            self.im,
            u8::from(self.ei),
            u8::from(self.p),
            self.q
        )
    }
}

fn read_snapshot(j: &Json) -> Snapshot {
    let u8f = |k: &str| j.get(k).unwrap_or_else(|| panic!("missing {k}")).num() as u8;
    let u16f = |k: &str| j.get(k).unwrap_or_else(|| panic!("missing {k}")).num() as u16;
    Snapshot {
        regs: Regs {
            a: u8f("a"),
            f: u8f("f"),
            b: u8f("b"),
            c: u8f("c"),
            d: u8f("d"),
            e: u8f("e"),
            h: u8f("h"),
            l: u8f("l"),
            ix: u16f("ix"),
            iy: u16f("iy"),
            sp: u16f("sp"),
            pc: u16f("pc"),
            i: u8f("i"),
            r: u8f("r"),
            wz: u16f("wz"),
            af_alt: u16f("af_"),
            bc_alt: u16f("bc_"),
            de_alt: u16f("de_"),
            hl_alt: u16f("hl_"),
        },
        iff1: u8f("iff1") != 0,
        iff2: u8f("iff2") != 0,
        im: u8f("im"),
        ei: u8f("ei") != 0,
        p: u8f("p") != 0,
        q: u8f("q"),
    }
}

/// What went wrong with one vector, ready to print.
struct Failure {
    name: String,
    detail: String,
}

/// Run every vector in one file, returning the failures.
fn run_file(path: &Path, cfg: Config) -> Vec<Failure> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
    let vectors = Parser::new(&bytes).value();
    let mut failures = Vec::new();

    for vector in vectors.arr() {
        let name = vector.get("name").expect("name").str().to_string();
        let initial = vector.get("initial").expect("initial");
        let expected = vector.get("final").expect("final");

        let ram = alloc::sync::Arc::new(VectorRam(sync::Mutex::with_rank(
            LockRank::DEVICE,
            alloc::vec![0u8; 0x1_0000],
        )));
        for cell in initial.get("ram").expect("ram").arr() {
            let cell = cell.arr();
            ram.0.lock()[cell[0].num() as usize] = cell[1].num() as u8;
        }

        // The vector states its one port transaction up front, which is also
        // how the value an `IN` reads gets onto the bus.
        let want_ports: Vec<Port> = vector
            .get("ports")
            .map(|p| {
                p.arr()
                    .iter()
                    .map(|t| {
                        let t = t.arr();
                        Port {
                            addr: t[0].num() as u16,
                            value: t[1].num() as u8,
                            write: t[2].str() == "w",
                        }
                    })
                    .collect()
            })
            .unwrap_or_default();
        let driven = want_ports.iter().find(|p| !p.write).map(|p| p.value);
        let ports = alloc::sync::Arc::new(VectorPorts(sync::Mutex::with_rank(
            LockRank::DEVICE,
            (driven, Vec::new()),
        )));

        let mem = AddressSpace::new("cpu", 16).with_unassigned(UnassignedPolicy::FAULT);
        mem.topology()
            .map(Region::io("ram", 0x1_0000, ram.clone()), 0)
            .expect("64 KiB fits");
        let io = AddressSpace::new("io", 16).with_unassigned(UnassignedPolicy::FAULT);
        io.topology()
            .map(Region::io("ports", 0x1_0000, ports.clone()), 0)
            .expect("64 KiB fits");

        let cpu = Z80::new(cfg);
        cpu.attach_space(alloc::sync::Arc::new(mem));
        cpu.attach_io_space(alloc::sync::Arc::new(io));

        // The vectors start mid-program: no reset sequence, and the interrupt
        // lines are not modelled by the corpus at all.
        let want = read_snapshot(expected);
        {
            let start = read_snapshot(initial);
            let mut session = cpu.session.lock();
            session.state.reset_pending = false;
            session.state.regs = start.regs;
            session.state.iff1 = start.iff1;
            session.state.iff2 = start.iff2;
            session.state.im = start.im;
            session.state.ei_pending = start.ei;
            session.state.after_ld_ir = start.p;
            session.state.q = start.q;
        }

        cpu.step();

        let mut detail = String::new();
        let got = {
            let s = cpu.session.lock();
            Snapshot {
                regs: s.state.regs,
                iff1: s.state.iff1,
                iff2: s.state.iff2,
                im: s.state.im,
                ei: s.state.ei_pending,
                p: s.state.after_ld_ir,
                q: s.state.q,
            }
        };
        if got != want {
            detail.push_str(&format!("  want {want}\n  got  {got}\n"));
        }

        for cell in expected.get("ram").expect("ram").arr() {
            let cell = cell.arr();
            let addr = cell[0].num() as usize;
            let value = cell[1].num() as u8;
            let actual = ram.0.lock()[addr];
            if actual != value {
                detail.push_str(&format!(
                    "  ram[{addr:04x}] want {value:02x} got {actual:02x}\n"
                ));
            }
        }

        let got_ports = core::mem::take(&mut ports.0.lock().1);
        if got_ports != want_ports {
            let show = |ps: &[Port]| {
                ps.iter()
                    .map(|p| p.to_string())
                    .collect::<Vec<_>>()
                    .join(" ")
            };
            detail.push_str(&format!(
                "  ports want {}\n        got  {}\n",
                show(&want_ports),
                show(&got_ports)
            ));
        }

        let want_cycles: Vec<Pins> = vector
            .get("cycles")
            .expect("cycles")
            .arr()
            .iter()
            .map(parse_pins)
            .collect();
        let log = cpu.last_cycles();
        let got_cycles = expand(&log);
        let mismatch = got_cycles.len() != want_cycles.len()
            || want_cycles
                .iter()
                .zip(&got_cycles)
                .any(|(w, g)| !pins_match(w, g));
        if mismatch {
            let show = |cs: &[Pins]| {
                cs.iter()
                    .map(|c| c.to_string())
                    .collect::<Vec<_>>()
                    .join(" ")
            };
            detail.push_str(&format!(
                "  {} T-states wanted, {} charged\n  want {}\n  got  {}\n",
                want_cycles.len(),
                got_cycles.len(),
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

/// Files this core is known not to pass, and why.
///
/// `ROADMAP.md` §0 asks for a ledger that only ever shrinks; **this one is
/// empty**. At the commit that added this file every one of the 1 604 files
/// passed all 1 000 of its vectors — registers, the shadow set, `WZ`, `Q`,
/// both flip-flops, memory, ports and the complete T-state trace.
const KNOWN_FAILURES: &[(&str, &str)] = &[];

/// Run the whole corpus, or explain why it did not.
///
/// Not `#[ignore]`d: a skipped test that says nothing is how a suite quietly
/// stops running. This one prints the command that would have run it.
#[test]
fn single_step_tests() {
    let Ok(dir) = std::env::var("RSEMU_Z80_DIR") else {
        println!(
            "conformance: set RSEMU_Z80_DIR to a SingleStepTests/z80 `v1` \
             directory to run 1 000 vectors for each of 1 604 encodings"
        );
        return;
    };
    let dir = Path::new(&dir);
    let only = std::env::var("RSEMU_Z80_FILES").ok();

    let mut files: Vec<String> = std::fs::read_dir(dir)
        .unwrap_or_else(|e| panic!("{}: {e}", dir.display()))
        .filter_map(|entry| {
            let path = entry.ok()?.path();
            if path.extension()?.to_str()? != "json" {
                return None;
            }
            Some(path.file_stem()?.to_str()?.to_string())
        })
        .collect();
    files.sort();
    assert!(!files.is_empty(), "no *.json files under {}", dir.display());

    let cfg = Config::NMOS;
    let mut ran = 0usize;
    let mut vectors = 0usize;
    let mut total_failures = 0usize;
    let mut unexpected: Vec<(String, usize)> = Vec::new();

    for stem in &files {
        if let Some(only) = &only
            && !only.split(',').any(|o| o.trim() == stem)
        {
            continue;
        }
        let failures = run_file(&dir.join(format!("{stem}.json")), cfg);
        ran += 1;
        vectors += 1000;
        if failures.is_empty() {
            continue;
        }
        total_failures += failures.len();
        let known = KNOWN_FAILURES.iter().any(|(f, _)| f == stem);
        println!(
            "{stem}{} — {} of 1000 vectors failed; first is `{}`:\n{}",
            if known { " (known)" } else { "" },
            failures.len(),
            failures[0].name,
            failures[0].detail
        );
        if !known {
            unexpected.push((stem.clone(), failures.len()));
        }
    }

    println!(
        "conformance: {ran} files, {vectors} vectors, {total_failures} failing \
         ({} unexpected files)",
        unexpected.len()
    );
    assert!(unexpected.is_empty(), "failing files: {unexpected:?}");
}

// ---------------------------------------------------------------------------
// The zexall / zexdoc exerciser
// ---------------------------------------------------------------------------

/// Run one of Frank Cringle's Z80 instruction exercisers.
///
/// `zexdoc` and `zexall` are CP/M `.COM` programs that run every instruction
/// over thousands of operand combinations and CRC the results against a table
/// taken from real hardware. `zexdoc` masks the undocumented flag bits;
/// `zexall` does not, which makes it the harder of the two and the reason bits
/// 3 and 5 are modelled here from the start.
///
/// The binaries carry their own licence and are **never vendored**
/// (`ROADMAP.md` §1, §12) — running one as an emulated guest is ordinary use,
/// shipping it would be redistribution. So this is gated on a path:
///
/// ```text
/// RSEMU_Z80_ZEX=/tmp/zexdoc.com cargo test --release --all-features \
///     z80::conformance::zex -- --nocapture
/// ```
///
/// The harness is the minimum CP/M an exerciser needs: the program at
/// `$0100`, the BDOS entry at `$0005` whose console calls this intercepts, and
/// `$0000` as the exit vector.
#[test]
fn zex_exerciser() {
    let Ok(path) = std::env::var("RSEMU_Z80_ZEX") else {
        println!(
            "conformance: set RSEMU_Z80_ZEX to a zexdoc.com or zexall.com to \
             run the instruction exerciser (several minutes)"
        );
        return;
    };
    let image = std::fs::read(&path).unwrap_or_else(|e| panic!("{path}: {e}"));

    let ram = alloc::sync::Arc::new(crate::core::space::RamStore::new(0x1_0000));
    for (i, byte) in image.iter().enumerate() {
        ram.write_u8(0x0100 + i as u64, *byte).expect("fits");
    }
    // `RET` at the BDOS entry: this harness services the call before the CPU
    // reaches the opcode, and the return is then the guest's own.
    ram.write_u8(0x0005, 0xc9).expect("fits");
    // The warm-boot vector, which the exerciser jumps to when it is done.
    ram.write_u8(0x0000, 0x76).expect("fits");

    let space = AddressSpace::new("cpu", 16);
    space
        .topology()
        .map(Region::ram("ram", ram.clone()), 0)
        .expect("64 KiB fits");
    let cpu = Z80::new(Config::NMOS);
    cpu.attach_space(alloc::sync::Arc::new(space));
    cpu.request_reset();
    cpu.step();
    cpu.set_reg(super::Reg::Pc, 0x0100);
    cpu.set_reg(super::Reg::Sp, 0xf000);

    let mut out = String::new();
    let mut errors = 0usize;
    let mut steps = 0u64;
    loop {
        let pc = cpu.reg(super::Reg::Pc);
        if pc == 0x0000 {
            break;
        }
        if pc == 0x0005 {
            let regs = cpu.regs();
            match regs.c {
                // BDOS 2: write the character in E.
                2 => out.push(regs.e as char),
                // BDOS 9: write the `$`-terminated string at DE.
                9 => {
                    let mut at = regs.de();
                    loop {
                        let byte = ram.read_u8(u64::from(at)).expect("mapped");
                        if byte == b'$' {
                            break;
                        }
                        out.push(byte as char);
                        at = at.wrapping_add(1);
                    }
                }
                other => panic!("unexpected BDOS call {other}"),
            }
            // Print as the guest produces it: a run this long is worth
            // watching, and a hang is then obvious from where it stopped.
            while let Some(end) = out.find('\n') {
                let line: String = out.drain(..=end).collect();
                let line = line.trim_end();
                if line.contains("ERROR") {
                    errors += 1;
                }
                println!("zex: {line}");
            }
        }
        cpu.step();
        steps += 1;
        assert!(steps < 20_000_000_000, "the exerciser did not terminate");
    }
    let tail = out.trim_end();
    if !tail.is_empty() {
        if tail.contains("ERROR") {
            errors += 1;
        }
        println!("zex: {tail}");
    }
    println!(
        "zex: {} — {steps} instructions, {} T-states, {errors} failing tests",
        std::path::Path::new(&path).display(),
        cpu.cycles()
    );
    assert_eq!(errors, 0, "the exerciser reported CRC mismatches");
}
