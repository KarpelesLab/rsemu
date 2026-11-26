//! The `SingleStepTests/65816` vector runner, for `cpu::w65c816`.
//!
//! MIT-licensed (© Thomas Harte et al.), 512 files: one per opcode per mode,
//! `NN.e.json` for emulation and `NN.n.json` for native, 10 000 vectors each
//! (100 for `MVN`/`MVP`, whose vectors run a whole move). Each vector is an
//! initial register file and sparse memory over a flat 16 MiB, the final ones,
//! and the bus trace — address, data and the **pin states** of every cycle:
//! VDA, VPA, VPB, RWB, E, M, X and MLB. The trace includes internal
//! operations, with the address they drive and no data, which is what makes
//! this the measurement worth having: it checks every dummy cycle the
//! datasheet's table 5-7 lists.
//!
//! Fetched by `scripts/fetch-testdata.sh sst-65816` (pinned commit; about
//! 2.7 GB); never committed.
//!
//! # How the core is driven
//!
//! Through its public API only, as a machine drives it. A flat 16 MiB region
//! records every access the core makes, and an
//! [`AccessTiming`](rsemu::cpu::w65c816::AccessTiming) provider records every
//! cycle it is asked about — that hook is told about internal operations too,
//! with their pins, which is how the trace is rebuilt without anything
//! test-only in the core. The two logs are merged: each cycle with VDA or VPA
//! set must have exactly one memory access behind it, at the same address,
//! and each cycle without must have none.
//!
//! The corpus's environment "does not activate RAM unless one of VDA, VPA or
//! VPB is active", so the emulation-mode modify cycle of a read-modify-write
//! (RWB low, VDA and VPA low) writes nothing, and neither does the core.

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use crate::json::{Reader, Result as JsonResult, Value};

/// A hard ceiling on cycles per vector, so a runaway move or a core stuck in
/// a loop fails rather than hangs. The longest vectors are 1 000-byte block
/// moves at seven cycles a byte.
const MAX_CYCLES: usize = 8_000;

/// The address a stopped cycle is recorded at.
pub(crate) const STOPPED: u32 = u32::MAX;

/// How many failing vectors per file get a full diff.
const DETAIL_CAP: usize = 3;

/// The register file as the corpus spells it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct Regs {
    pub(crate) pc: u16,
    pub(crate) s: u16,
    pub(crate) p: u8,
    pub(crate) a: u16,
    pub(crate) x: u16,
    pub(crate) y: u16,
    pub(crate) dbr: u8,
    pub(crate) d: u16,
    pub(crate) pbr: u8,
    pub(crate) e: bool,
}

impl std::fmt::Display for Regs {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "PC:{:02x}:{:04x} S:{:04x} P:{:02x} A:{:04x} X:{:04x} Y:{:04x} DBR:{:02x} D:{:04x} E:{}",
            self.pbr,
            self.pc,
            self.s,
            self.p,
            self.a,
            self.x,
            self.y,
            self.dbr,
            self.d,
            u8::from(self.e)
        )
    }
}

/// A processor state plus its sparse memory.
#[derive(Debug, Clone, Default)]
pub(crate) struct State {
    pub(crate) regs: Regs,
    pub(crate) ram: Vec<(u32, u8)>,
}

/// One recorded cycle: address, data (`None` where nothing was selected), and
/// the eight-character pin string, `dpvrwemxl` style.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Cycle {
    pub(crate) addr: u32,
    pub(crate) value: Option<u8>,
    pub(crate) pins: String,
}

impl std::fmt::Display for Cycle {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.value {
            _ if self.addr == STOPPED => write!(f, "stopped   {}", self.pins),
            Some(v) => write!(f, "{:06x} {v:02x} {}", self.addr, self.pins),
            None => write!(f, "{:06x} -- {}", self.addr, self.pins),
        }
    }
}

/// One test vector.
#[derive(Debug, Clone, Default)]
pub(crate) struct Vector {
    pub(crate) name: String,
    pub(crate) initial: State,
    pub(crate) expected: State,
    pub(crate) cycles: Vec<Cycle>,
}

/// Parse one file: a JSON array of vectors.
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
    let (mut init, mut fin) = (false, false);
    r.object(|r, key| {
        match key {
            "name" => v.name = r.string()?,
            "initial" => {
                v.initial = parse_state(r)?;
                init = true;
            }
            "final" => {
                v.expected = parse_state(r)?;
                fin = true;
            }
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
    if !init || !fin {
        return Err(r.err("vector is missing an `initial` or `final` section"));
    }
    Ok(v)
}

fn parse_state(r: &mut Reader<'_>) -> JsonResult<State> {
    let mut s = State::default();
    r.object(|r, key| {
        match key {
            "pc" => s.regs.pc = r.u64_in(0xffff)? as u16,
            "s" => s.regs.s = r.u64_in(0xffff)? as u16,
            "p" => s.regs.p = r.u64_in(0xff)? as u8,
            "a" => s.regs.a = r.u64_in(0xffff)? as u16,
            "x" => s.regs.x = r.u64_in(0xffff)? as u16,
            "y" => s.regs.y = r.u64_in(0xffff)? as u16,
            "dbr" => s.regs.dbr = r.u64_in(0xff)? as u8,
            "d" => s.regs.d = r.u64_in(0xffff)? as u16,
            "pbr" => s.regs.pbr = r.u64_in(0xff)? as u8,
            "e" => s.regs.e = r.u64_in(1)? == 1,
            "ram" => {
                r.array(|r, _| {
                    let (mut addr, mut val) = (0u32, 0u8);
                    let n = r.array(|r, i| {
                        match i {
                            0 => addr = r.u64_in(0xff_ffff)? as u32,
                            1 => val = r.u64_in(0xff)? as u8,
                            _ => return Err(r.err("ram entry has more than two fields")),
                        }
                        Ok(())
                    })?;
                    if n != 2 {
                        return Err(r.err("ram entry must be [address, value]"));
                    }
                    s.ram.push((addr, val));
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
        addr: 0,
        value: None,
        pins: String::new(),
    };
    let n = r.array(|r, i| {
        match i {
            // `null` is the corpus's mark for "the processor has stopped":
            // the cycle after `WAI` or `STP`, which drives nothing at all.
            0 => {
                c.addr = match r.value()? {
                    Value::Null => STOPPED,
                    v => match v.as_i64() {
                        Some(n @ 0..=0xff_ffff) => n as u32,
                        _ => return Err(r.err("cycle address is not 24 bits or null")),
                    },
                }
            }
            1 => {
                c.value = match r.value()? {
                    Value::Null => None,
                    v => match v.as_i64() {
                        Some(n @ 0..=255) => Some(n as u8),
                        _ => return Err(r.err("cycle data is not a byte or null")),
                    },
                }
            }
            2 => {
                c.pins = r.string()?;
                if c.pins.len() != 8 {
                    return Err(r.err(format!("pin string {:?} is not 8 characters", c.pins)));
                }
            }
            _ => return Err(r.err("cycle entry has more than three fields")),
        }
        Ok(())
    })?;
    if n != 3 {
        return Err(r.err("cycle entry must be [address, value, pins]"));
    }
    Ok(c)
}

/// Where the vector files live.
pub(crate) fn corpus_dir(root: &Path) -> PathBuf {
    root.join("sst-65816").join("v1")
}

/// Every vector file present, as `(opcode, mode, path)`, in order.
pub(crate) fn files(dir: &Path) -> std::io::Result<Vec<(u8, char, PathBuf)>> {
    let mut found = Vec::new();
    for entry in std::fs::read_dir(dir)? {
        let path = entry?.path();
        let Some(name) = path.file_name().and_then(|s| s.to_str()) else {
            continue;
        };
        let Some(stem) = name.strip_suffix(".json") else {
            continue;
        };
        let Some((op, mode)) = stem.split_once('.') else {
            continue;
        };
        let Ok(op) = u8::from_str_radix(op, 16) else {
            continue;
        };
        let mode = match mode {
            "e" => 'e',
            "n" => 'n',
            _ => continue,
        };
        found.push((op, mode, path));
    }
    found.sort_by_key(|a| (a.0, a.1));
    Ok(found)
}

/// The outcome of one file.
#[derive(Debug)]
pub(crate) struct FileReport {
    pub(crate) opcode: u8,
    pub(crate) mode: char,
    pub(crate) total: usize,
    pub(crate) failed: Vec<String>,
    pub(crate) details: String,
    pub(crate) categories: Vec<String>,
}

#[cfg(feature = "cpu-w65c816")]
pub(crate) use runner::Runner;

#[cfg(feature = "cpu-w65c816")]
mod runner {
    use std::fmt::Write as _;
    use std::sync::{Arc, Mutex};

    use rsemu::core::space::{
        AccessConstraints, AddressSpace, MemAttrs, MemOps, MemResult, Region, UnassignedPolicy,
    };
    use rsemu::cpu::w65c816::{AccessTiming, BusCycle, Pins, Regs as CoreRegs, W65c816};

    use super::{Cycle, DETAIL_CAP, FileReport, MAX_CYCLES, Regs, Vector};

    /// What the core did on the bus, in order.
    #[derive(Debug, Default)]
    struct Log {
        /// Every cycle the timing hook was asked about.
        cycles: Vec<BusCycle>,
        /// Every access that reached memory: address, value, write.
        accesses: Vec<(u32, u8, bool)>,
        /// Whether recording is on (off during the bookkeeping resets).
        live: bool,
        overrun: bool,
    }

    /// A flat 16 MiB of RAM that records every access.
    #[derive(Debug)]
    struct Bus {
        mem: Mutex<Vec<u8>>,
        log: Arc<Mutex<Log>>,
    }

    impl MemOps for Bus {
        fn read(&self, offset: u64, dst: &mut [u8], attrs: MemAttrs) -> MemResult {
            let mem = self.mem.lock().unwrap();
            for (i, slot) in dst.iter_mut().enumerate() {
                let a = ((offset as usize) + i) & 0xff_ffff;
                *slot = mem[a];
                if !attrs.debug {
                    let mut log = self.log.lock().unwrap();
                    if log.live {
                        log.accesses.push((a as u32, *slot, false));
                    }
                }
            }
            Ok(())
        }

        fn write(&self, offset: u64, src: &[u8], attrs: MemAttrs) -> MemResult {
            if attrs.debug {
                return Ok(());
            }
            let mut mem = self.mem.lock().unwrap();
            for (i, b) in src.iter().enumerate() {
                let a = ((offset as usize) + i) & 0xff_ffff;
                mem[a] = *b;
                let mut log = self.log.lock().unwrap();
                if log.live {
                    log.accesses.push((a as u32, *b, true));
                }
            }
            Ok(())
        }

        fn constraints(&self) -> AccessConstraints {
            AccessConstraints::ANY
        }
    }

    /// The timing hook as a recorder: every cycle costs one clock.
    #[derive(Debug)]
    struct Recorder(Arc<Mutex<Log>>);

    impl AccessTiming for Recorder {
        fn clocks(&self, cycle: BusCycle) -> u64 {
            let mut log = self.0.lock().unwrap();
            if log.live {
                if log.cycles.len() >= MAX_CYCLES {
                    log.overrun = true;
                } else {
                    log.cycles.push(cycle);
                }
            }
            1
        }
    }

    /// One long-lived core, a bus and a recorder.
    pub(crate) struct Runner {
        cpu: W65c816,
        bus: Arc<Bus>,
        log: Arc<Mutex<Log>>,
        /// Addresses written by the last vector, to be zeroed before the next.
        dirty: Vec<u32>,
    }

    impl Runner {
        pub(crate) fn new() -> Runner {
            let log = Arc::new(Mutex::new(Log::default()));
            let bus = Arc::new(Bus {
                mem: Mutex::new(vec![0; 1 << 24]),
                log: Arc::clone(&log),
            });
            let space = AddressSpace::new("cpu", 24).with_unassigned(UnassignedPolicy::FAULT);
            space
                .topology()
                .map(Region::io("ram", 1 << 24, bus.clone()), 0)
                .expect("16 MiB fits a 24-bit space");
            let cpu = W65c816::new();
            cpu.attach_space(Arc::new(space));
            cpu.attach_timing(Arc::new(Recorder(Arc::clone(&log))));
            // The power-on reset runs here, unrecorded, against zeroed RAM.
            cpu.step();
            Runner {
                cpu,
                bus,
                log,
                dirty: Vec::new(),
            }
        }

        fn load(&mut self, v: &Vector) {
            if self.cpu.is_halted()
                || self.cpu.is_waiting()
                || self.cpu.pending_interrupt().is_some()
            {
                self.cpu.request_reset();
                self.cpu.step();
            }
            let mut mem = self.bus.mem.lock().unwrap();
            for a in self.dirty.drain(..) {
                mem[a as usize] = 0;
            }
            for &(a, b) in &v.initial.ram {
                mem[a as usize] = b;
                self.dirty.push(a);
            }
            drop(mem);
            let r = v.initial.regs;
            self.cpu.set_regs(CoreRegs {
                a: r.a,
                x: r.x,
                y: r.y,
                s: r.s,
                d: r.d,
                pc: r.pc,
                pbr: r.pbr,
                dbr: r.dbr,
                p: r.p,
                e: r.e,
            });
            let mut log = self.log.lock().unwrap();
            log.cycles.clear();
            log.accesses.clear();
            log.overrun = false;
            log.live = true;
        }

        /// Run one vector, returning the mismatch categories and a diff.
        pub(crate) fn run(
            &mut self,
            v: &Vector,
            block_move: bool,
        ) -> Result<(), (Vec<String>, String)> {
            self.load(v);
            let start = self.cpu.regs().pc24();
            let stepped = crate::harness::catching(|| {
                self.cpu.step();
                // A block move re-executes itself until the count runs out;
                // the vector covers the whole move.
                if block_move {
                    while self.cpu.regs().pc24() == start {
                        if self.log.lock().unwrap().overrun {
                            break;
                        }
                        self.cpu.step();
                    }
                }
            });
            let mut log = self.log.lock().unwrap();
            log.live = false;
            let mut reasons = Vec::new();
            let mut detail = String::new();
            if let Err(msg) = stepped {
                reasons.push("panic".to_string());
                let _ = writeln!(detail, "  the core panicked: {msg}");
            }
            if log.overrun {
                reasons.push("cycle overrun".to_string());
            }

            // Registers.
            let c = self.cpu.regs();
            let got = Regs {
                pc: c.pc,
                s: c.s,
                p: c.p,
                a: c.a,
                x: c.x,
                y: c.y,
                dbr: c.dbr,
                d: c.d,
                pbr: c.pbr,
                e: c.e,
            };
            if got != v.expected.regs {
                reasons.push("registers".to_string());
                let _ = writeln!(
                    detail,
                    "  registers:\n    expected {}\n    actual   {got}",
                    v.expected.regs
                );
            }

            // The trace, rebuilt from the two logs.
            let mut trace = Vec::with_capacity(log.cycles.len());
            let mut accesses = log.accesses.iter();
            let mut desync = false;
            for bc in &log.cycles {
                let p = bc.pins;
                let value = if p.is_internal() {
                    bc.data
                } else {
                    match accesses.next() {
                        Some(&(a, val, w)) if a == bc.addr && w == p.is_write() => Some(val),
                        _ => {
                            desync = true;
                            None
                        }
                    }
                };
                trace.push(Cycle {
                    addr: bc.addr,
                    value,
                    pins: pin_string(p),
                });
            }
            // `WAI` and `STP` end with the processor driving nothing.
            if self.cpu.is_waiting() || self.cpu.is_halted() {
                trace.push(Cycle {
                    addr: super::STOPPED,
                    value: None,
                    pins: "--------".to_string(),
                });
            }
            if desync || accesses.next().is_some() {
                reasons.push("accesses disagree with the cycles reported".to_string());
            }
            if trace != v.cycles {
                reasons.push(if trace.len() == v.cycles.len() {
                    "bus trace".to_string()
                } else {
                    "cycle count".to_string()
                });
                let _ = writeln!(
                    detail,
                    "  bus trace ({} expected, {} recorded):",
                    v.cycles.len(),
                    trace.len()
                );
                for i in 0..v.cycles.len().max(trace.len()).min(40) {
                    let e = v.cycles.get(i);
                    let a = trace.get(i);
                    let mark = if e == a { ' ' } else { '<' };
                    let show = |c: Option<&Cycle>| c.map_or("—".to_string(), |c| c.to_string());
                    let _ = writeln!(detail, "    {i:3}  {:<22} {:<22} {mark}", show(e), show(a));
                }
            }

            // Memory: every listed final byte, and nothing else written.
            let mem = self.bus.mem.lock().unwrap();
            let mut bad = Vec::new();
            for &(a, b) in &v.expected.ram {
                if mem[a as usize] != b {
                    bad.push((a, b, mem[a as usize]));
                }
            }
            for &(a, _, w) in &log.accesses {
                if w && !v.expected.ram.iter().any(|&(x, _)| x == a) {
                    bad.push((a, 0xff, mem[a as usize]));
                }
            }
            drop(mem);
            for &(a, _, w) in &log.accesses {
                if w {
                    self.dirty.push(a);
                }
            }
            if !bad.is_empty() {
                reasons.push("memory".to_string());
                for (a, want, have) in bad.iter().take(6) {
                    let _ = writeln!(
                        detail,
                        "    ${a:06x}: expected {want:02x}, actual {have:02x}"
                    );
                }
            }
            if reasons.is_empty() {
                Ok(())
            } else {
                Err((reasons, detail))
            }
        }
    }

    fn pin_string(p: Pins) -> String {
        let mut s = String::with_capacity(8);
        s.push(if p.has(Pins::VDA) { 'd' } else { '-' });
        s.push(if p.has(Pins::VPA) { 'p' } else { '-' });
        s.push(if p.has(Pins::VPB) { 'v' } else { '-' });
        s.push(if p.is_write() { 'w' } else { 'r' });
        s.push(if p.has(Pins::E) { 'e' } else { '-' });
        s.push(if p.has(Pins::M) { 'm' } else { '-' });
        s.push(if p.has(Pins::X) { 'x' } else { '-' });
        s.push(if p.has(Pins::ML) { 'l' } else { '-' });
        s
    }

    /// Run every vector in one file.
    pub(crate) fn run_file(
        runner: &mut Runner,
        opcode: u8,
        mode: char,
        vectors: &[Vector],
    ) -> FileReport {
        let mut report = FileReport {
            opcode,
            mode,
            total: vectors.len(),
            failed: Vec::new(),
            details: String::new(),
            categories: Vec::new(),
        };
        let block_move = matches!(opcode, 0x44 | 0x54);
        for v in vectors {
            if let Err((reasons, detail)) = runner.run(v, block_move) {
                if report.failed.len() < DETAIL_CAP {
                    let _ = writeln!(report.details, "  {} — {}", v.name, reasons.join(", "));
                    report.details.push_str(&detail);
                }
                for r in reasons {
                    if !report.categories.contains(&r) {
                        report.categories.push(r);
                    }
                }
                report.failed.push(v.name.clone());
            }
        }
        report
    }
}

#[cfg(feature = "cpu-w65c816")]
pub(crate) use runner::run_file;

/// Whether this build has the core.
pub(crate) fn cpu_is_built() -> bool {
    cfg!(feature = "cpu-w65c816")
}

/// One summary line per failing file, for the report.
pub(crate) fn summary(reports: &[FileReport]) -> String {
    let mut out = String::new();
    for r in reports.iter().filter(|r| !r.failed.is_empty()) {
        let _ = writeln!(
            out,
            "{:02x}.{}: {}/{} failed — {}",
            r.opcode,
            r.mode,
            r.failed.len(),
            r.total,
            r.categories.join(", ")
        );
        out.push_str(&r.details);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    const README_EXAMPLE: &[u8] = br#"[{
        "name": "3d e 1",
        "initial": {"pc": 9900, "s": 2191, "p": 171, "a": 25345, "x": 100, "y": 124,
            "dbr": 26, "d": 50304, "pbr": 111, "e": 1,
            "ram": [[1751932, 14], [7284398, 187], [7284397, 24], [7284396, 61]]},
        "final": {"pc": 9903, "s": 2191, "p": 43, "a": 25344, "x": 100, "y": 124,
            "dbr": 26, "d": 50304, "pbr": 111, "e": 1,
            "ram": [[1751932, 14], [7284398, 187], [7284397, 24], [7284396, 61]]},
        "cycles": [[7284396, 61, "dp-remx-"], [7284397, 24, "-p-remx-"],
                   [7284398, 187, "-p-remx-"], [1751932, 14, "d--remx-"]]
    }]"#;

    #[test]
    fn the_upstream_example_parses() {
        let v = parse_vectors(README_EXAMPLE).unwrap();
        assert_eq!(v.len(), 1);
        assert_eq!(v[0].initial.regs.pbr, 111);
        assert!(v[0].initial.regs.e);
        assert_eq!(v[0].cycles[3].addr, 1751932);
        assert_eq!(v[0].cycles[3].pins, "d--remx-");
    }

    #[test]
    fn a_null_data_byte_is_an_internal_cycle() {
        let src = br#"[{"name":"ea e 1","initial":{"ram":[]},"final":{"ram":[]},
            "cycles":[[1, null, "---remx-"]]}]"#;
        let v = parse_vectors(src).unwrap();
        assert_eq!(v[0].cycles[0].value, None);
    }

    #[cfg(feature = "cpu-w65c816")]
    #[test]
    fn a_real_vector_passes_and_a_wrong_one_fails() {
        // `1e e 1` from the pinned corpus (MIT, Thomas Harte et al.): ASL
        // $xxxx,X in emulation mode, whose modify cycle writes the old value
        // with nothing selected. (The README's own example predates the
        // corpus's current generation and is not used: its emulation-mode
        // stack pointer is one the hardware cannot hold.)
        let src = br#"[{"name": "1e e 1", "initial": {"pc": 28226, "s": 61422, "p": 114,
            "a": 14375, "x": 234, "y": 91, "dbr": 221, "d": 60973, "pbr": 198, "e": 1,
            "ram": [[14533725, 234], [13004356, 195], [13004355, 115], [13004354, 30]]},
          "final": {"pc": 28229, "s": 494, "p": 241, "a": 14375, "x": 234, "y": 91,
            "dbr": 221, "d": 60973, "pbr": 198, "e": 1,
            "ram": [[14533725, 212], [13004356, 195], [13004355, 115], [13004354, 30]]},
          "cycles": [[13004354, 30, "dp-remx-"], [13004355, 115, "-p-remx-"],
            [13004356, 195, "-p-remx-"], [14533469, null, "---remx-"],
            [14533725, 234, "d--remxl"], [14533725, 234, "---wemxl"],
            [14533725, 212, "d--wemxl"]]}]"#;
        let v = parse_vectors(src).unwrap();
        let mut r = Runner::new();
        let report = run_file(&mut r, 0x1e, 'e', &v);
        assert!(report.failed.is_empty(), "{}", report.details);

        // The same vector with the dummy cycle's pins altered must fail on
        // the trace and nothing else.
        let mut bad = v.clone();
        bad[0].cycles[5].pins = "d--wemxl".to_string();
        let report = run_file(&mut r, 0x1e, 'e', &bad);
        assert_eq!(report.categories, ["bus trace"]);
    }
}
