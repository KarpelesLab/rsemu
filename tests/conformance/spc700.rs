//! The `SingleStepTests/spc700` vector runner — the SPC700 core's oracle.
//!
//! MIT-licensed, 1000 vectors per opcode, all 256 opcodes. Each vector is an
//! initial register file and sparse RAM, the expected final state, and the
//! **cycle-by-cycle bus trace**, in which an internal cycle appears as
//! `[null, null, "wait"]` and a read whose data the generator did not pin
//! down as `[addr, null, "read"]`. The trace is compared cycle for cycle:
//! address, direction, and data wherever the vector states it.
//!
//! Corpus: <https://github.com/SingleStepTests/spc700>, pinned by
//! `scripts/fetch-testdata.sh sst-spc700`; never committed. Ledger:
//! `ledgers/sst-spc700.txt`, keyed per vector by its upstream name
//! (`8f :: 8F 0042`).
//!
//! `SLEEP` and `STOP` never finish, so their vectors record a fixed window of
//! the halted processor; the runner steps until it has produced as many
//! cycles as the vector lists, which for every other opcode is one step.

#[cfg(feature = "cpu-spc700")]
use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use crate::json::{Reader, Result as JsonResult, Value};

/// How many failing vectors per opcode get a full diff in the report.
const DETAIL_CAP: usize = 3;

/// One recorded bus cycle. `None` fields are the vector's nulls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Cycle {
    pub(crate) addr: Option<u16>,
    pub(crate) value: Option<u8>,
    /// `b'r'`, `b'w'` or `b'i'` (internal).
    pub(crate) kind: u8,
}

/// A register file plus sparse RAM.
#[derive(Debug, Clone, Default)]
pub(crate) struct State {
    pub(crate) pc: u16,
    pub(crate) a: u8,
    pub(crate) x: u8,
    pub(crate) y: u8,
    pub(crate) sp: u8,
    pub(crate) psw: u8,
    pub(crate) ram: Vec<(u16, u8)>,
}

/// One vector.
#[derive(Debug, Clone, Default)]
pub(crate) struct Vector {
    pub(crate) name: String,
    pub(crate) initial: State,
    pub(crate) expected: State,
    pub(crate) cycles: Vec<Cycle>,
}

/// Where the corpus lives.
pub(crate) fn corpus_dir(root: &Path) -> PathBuf {
    root.join("sst-spc700").join("v1")
}

/// Parse one opcode file.
pub(crate) fn parse_vectors(bytes: &[u8]) -> JsonResult<Vec<Vector>> {
    let mut out = Vec::new();
    let mut r = Reader::new(bytes);
    r.array(|r, _| {
        out.push(parse_vector(r)?);
        Ok(())
    })?;
    if !r.at_end() {
        return Err(r.err("trailing data after the vector array"));
    }
    Ok(out)
}

fn parse_vector(r: &mut Reader<'_>) -> JsonResult<Vector> {
    let mut v = Vector::default();
    r.object(|r, key| {
        match key {
            "name" => v.name = r.string()?,
            "initial" => v.initial = parse_state(r)?,
            "final" => v.expected = parse_state(r)?,
            "cycles" => {
                r.array(|r, _| {
                    v.cycles.push(parse_cycle(r)?);
                    Ok(())
                })?;
            }
            _ => r.skip_value()?,
        }
        Ok(())
    })?;
    Ok(v)
}

fn parse_state(r: &mut Reader<'_>) -> JsonResult<State> {
    let mut s = State::default();
    r.object(|r, key| {
        match key {
            "pc" => s.pc = r.u64_in(0xffff)? as u16,
            "a" => s.a = r.u64_in(0xff)? as u8,
            "x" => s.x = r.u64_in(0xff)? as u8,
            "y" => s.y = r.u64_in(0xff)? as u8,
            "sp" => s.sp = r.u64_in(0xff)? as u8,
            "psw" => s.psw = r.u64_in(0xff)? as u8,
            "ram" => {
                r.array(|r, _| {
                    let mut pair = (0u16, 0u8);
                    r.array(|r, i| {
                        match i {
                            0 => pair.0 = r.u64_in(0xffff)? as u16,
                            1 => pair.1 = r.u64_in(0xff)? as u8,
                            _ => return Err(r.err("ram entry has more than two fields")),
                        }
                        Ok(())
                    })?;
                    s.ram.push(pair);
                    Ok(())
                })?;
            }
            _ => r.skip_value()?,
        }
        Ok(())
    })?;
    Ok(s)
}

fn parse_cycle(r: &mut Reader<'_>) -> JsonResult<Cycle> {
    let mut c = Cycle {
        addr: None,
        value: None,
        kind: b'r',
    };
    let n = r.array(|r, i| {
        let v = r.value()?;
        let int = |v: &Value, max: i64| v.as_i64().filter(|n| (0..=max).contains(n));
        match i {
            0 => c.addr = int(&v, 0xffff).map(|n| n as u16),
            1 => c.value = int(&v, 0xff).map(|n| n as u8),
            2 => {
                c.kind = match v.as_str() {
                    Some("read") => b'r',
                    Some("write") => b'w',
                    Some("wait") => b'i',
                    other => return Err(r.err(format!("unknown cycle type {other:?}"))),
                }
            }
            _ => return Err(r.err("cycle entry has more than three fields")),
        }
        Ok(())
    })?;
    if n != 3 {
        return Err(r.err("cycle entry must be [address, value, type]"));
    }
    Ok(c)
}

/// One opcode's results.
#[derive(Debug, Default)]
pub(crate) struct OpcodeReport {
    pub(crate) opcode: u8,
    pub(crate) total: usize,
    pub(crate) failed: Vec<String>,
    pub(crate) details: String,
}

#[cfg(feature = "cpu-spc700")]
mod run {
    use super::*;
    use rsemu::cpu::spc700::{Bus, Core, Mode, Regs};

    /// Flat 64 KiB that records every cycle.
    pub(super) struct TraceBus {
        pub(super) ram: Box<[u8; 0x10000]>,
        pub(super) trace: Vec<Cycle>,
    }

    impl Bus for TraceBus {
        fn read(&mut self, addr: u16) -> u8 {
            let v = self.ram[addr as usize];
            self.trace.push(Cycle {
                addr: Some(addr),
                value: Some(v),
                kind: b'r',
            });
            v
        }
        fn write(&mut self, addr: u16, value: u8) {
            self.ram[addr as usize] = value;
            self.trace.push(Cycle {
                addr: Some(addr),
                value: Some(value),
                kind: b'w',
            });
        }
        fn idle(&mut self) {
            self.trace.push(Cycle {
                addr: None,
                value: None,
                kind: b'i',
            });
        }
    }

    fn show(c: &Cycle) -> String {
        let a = c.addr.map_or("----".into(), |a| format!("{a:04x}"));
        let v = c.value.map_or("--".into(), |v| format!("{v:02x}"));
        format!("{a}:{v}:{}", c.kind as char)
    }

    /// Run one vector; `None` on a pass, the diff on a failure.
    pub(super) fn run_one(bus: &mut TraceBus, v: &Vector) -> Option<String> {
        for &(a, b) in &v.initial.ram {
            bus.ram[a as usize] = b;
        }
        bus.trace.clear();
        let i = &v.initial;
        let mut core = Core::new();
        core.mode = Mode::Running;
        core.regs = Regs {
            a: i.a,
            x: i.x,
            y: i.y,
            sp: i.sp,
            psw: i.psw,
            pc: i.pc,
        };
        let want = v.cycles.len();
        while bus.trace.len() < want && bus.trace.len() < 64 {
            core.step(bus);
        }
        let mut diff = String::new();
        let e = &v.expected;
        let got = core.regs;
        let regs_ok =
            (got.a, got.x, got.y, got.sp, got.psw, got.pc) == (e.a, e.x, e.y, e.sp, e.psw, e.pc);
        if !regs_ok {
            let _ = writeln!(
                diff,
                "    regs: want A={:02x} X={:02x} Y={:02x} SP={:02x} PSW={:02x} PC={:04x}\n          got  {got}",
                e.a, e.x, e.y, e.sp, e.psw, e.pc
            );
        }
        for &(a, b) in &e.ram {
            let have = bus.ram[a as usize];
            if have != b {
                let _ = writeln!(diff, "    ram[{a:04x}]: want {b:02x} got {have:02x}");
            }
        }
        let trace_ok = bus.trace.len() == want
            && bus.trace.iter().zip(&v.cycles).all(|(g, w)| {
                g.kind == w.kind
                    && (w.addr.is_none() || g.addr == w.addr)
                    && (w.value.is_none() || g.value == w.value)
            });
        if !trace_ok {
            let want: Vec<String> = v.cycles.iter().map(show).collect();
            let got: Vec<String> = bus.trace.iter().map(show).collect();
            let _ = writeln!(
                diff,
                "    cycles want {}\n           got  {}",
                want.join(" "),
                got.join(" ")
            );
        }
        // Put RAM back to zero where this vector touched it, so the next one
        // starts from the same blank slate.
        for &(a, _) in v.initial.ram.iter().chain(&e.ram) {
            bus.ram[a as usize] = 0;
        }
        for c in &bus.trace {
            if let (Some(a), b'w') = (c.addr, c.kind) {
                bus.ram[a as usize] = 0;
            }
        }
        if diff.is_empty() { None } else { Some(diff) }
    }

    pub(super) fn run_opcode(opcode: u8, vectors: &[Vector]) -> OpcodeReport {
        let mut bus = TraceBus {
            ram: Box::new([0; 0x10000]),
            trace: Vec::new(),
        };
        let mut report = OpcodeReport {
            opcode,
            total: vectors.len(),
            ..OpcodeReport::default()
        };
        for v in vectors {
            if let Some(d) = run_one(&mut bus, v) {
                if report.failed.len() < DETAIL_CAP {
                    let _ = writeln!(report.details, "  {}\n{d}", v.name);
                }
                report.failed.push(v.name.clone());
            }
        }
        report
    }
}

/// Run the whole suite, or report why not.
pub(crate) fn run_suite(suite: &str, ledger_path: &Path) {
    use crate::harness::{self, Skip};
    let fetch = "scripts/fetch-testdata.sh sst-spc700";
    let dir = match harness::require(corpus_dir(&harness::testdata_root()), fetch) {
        Ok(d) => d,
        Err(skip) => return skip.report(suite),
    };
    let mut files = crate::sst::opcode_files(&dir)
        .unwrap_or_else(|e| panic!("{} exists but cannot be listed: {e}", dir.display()));
    assert!(
        !files.is_empty(),
        "{} holds no opcode files; re-run {fetch}",
        dir.display()
    );
    if let Some(only) = crate::sst::opcode_filter() {
        files.retain(|(op, _)| only.contains(op));
    }

    #[cfg(not(feature = "cpu-spc700"))]
    {
        let (_, path) = &files[0];
        let bytes = harness::read(path).unwrap_or_else(|e| panic!("{e}"));
        let vectors = parse_vectors(&bytes).unwrap_or_else(|e| panic!("{}: {e}", path.display()));
        println!(
            "note: corpus readable — {} vectors in {}",
            vectors.len(),
            path.display()
        );
        Skip::NotBuilt {
            component: "an SPC700 core",
            feature: "cpu-spc700",
        }
        .report(suite);
        let _ = ledger_path;
    }

    #[cfg(feature = "cpu-spc700")]
    {
        let _ = Skip::Gated;
        let ran: Vec<u8> = files.iter().map(|&(op, _)| op).collect();
        let threads = std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1)
            .min(files.len());
        let queue = std::sync::Mutex::new(files);
        let reports = std::sync::Mutex::new(Vec::new());
        std::thread::scope(|scope| {
            for _ in 0..threads {
                scope.spawn(|| {
                    loop {
                        let Some((op, path)) = queue.lock().unwrap().pop() else {
                            break;
                        };
                        let bytes = harness::read(&path).unwrap_or_else(|e| panic!("{e}"));
                        let vectors = parse_vectors(&bytes)
                            .unwrap_or_else(|e| panic!("{}: {e}", path.display()));
                        let report = run::run_opcode(op, &vectors);
                        reports.lock().unwrap().push(report);
                    }
                });
            }
        });
        let mut reports = reports.into_inner().unwrap();
        reports.sort_by_key(|r| r.opcode);
        let total: usize = reports.iter().map(|r| r.total).sum();
        let failed: usize = reports.iter().map(|r| r.failed.len()).sum();
        assert!(total > 0, "{suite}: the corpus is empty");
        println!("{suite}: {}/{total} vectors passed", total - failed);

        let mut body = format!("{suite}: {}/{total} vectors passed\n\n", total - failed);
        for r in reports.iter().filter(|r| !r.failed.is_empty()) {
            let _ = writeln!(
                body,
                "{:02x}: {}/{} failed",
                r.opcode,
                r.failed.len(),
                r.total
            );
            body.push_str(&r.details);
        }
        harness::write_report(&format!("{suite}.txt"), &body);

        let failures: Vec<(u8, String)> = reports
            .iter()
            .flat_map(|r| r.failed.iter().map(move |n| (r.opcode, n.clone())))
            .collect();
        let ledger = crate::ledger::Ledger::load(ledger_path)
            .unwrap_or_else(|e| panic!("{}: {e}", ledger_path.display()));
        let verdict = crate::ledger::judge(&ledger, &ran, &failures);
        if verdict.excused > 0 {
            println!(
                "  {} failure(s) excused by {}",
                verdict.excused,
                ledger_path.display()
            );
        }
        assert!(verdict.is_ok(), "\n{}", verdict.describe(&ledger));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EXAMPLE: &[u8] = br#"[{"name":"8F 0000","initial":{"pc":58382,"a":47,"x":167,"y":141,"sp":102,"psw":159,"ram":[[156,174],[58382,143],[58383,23],[58384,156]]},"final":{"a":47,"x":167,"y":141,"sp":102,"pc":58385,"psw":159,"ram":[[156,23],[58382,143],[58383,23],[58384,156]]},"cycles":[[58382,143,"read"],[58383,23,"read"],[58384,156,"read"],[156,174,"read"],[156,23,"write"],[null,null,"wait"],[1,null,"read"]]}]"#;

    #[test]
    fn a_vector_parses_nulls_included() {
        let v = parse_vectors(EXAMPLE).expect("parses");
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].name, "8F 0000");
        assert_eq!(v[0].initial.pc, 58382);
        assert_eq!(v[0].expected.ram[0], (156, 23));
        assert_eq!(v[0].cycles.len(), 7);
        assert_eq!(
            v[0].cycles[5],
            Cycle {
                addr: None,
                value: None,
                kind: b'i'
            }
        );
        assert_eq!(v[0].cycles[6].value, None);
        assert_eq!(v[0].cycles[6].addr, Some(1));
    }
}
