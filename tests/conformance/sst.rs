//! The `SingleStepTests/65x02` vector runner — gate one for a 6502 core.
//!
//! MIT-licensed, 10 000 vectors per opcode, each one an initial register file
//! and sparse RAM, a final register file and sparse RAM, and the **full
//! cycle-by-cycle bus trace**. That last part is what makes it the suite worth
//! building well: it checks the dummy reads and the write ordering that every
//! other 6502 suite takes on faith, and it needs no machine at all — a flat
//! 64 KiB of RAM and one instruction per vector.
//!
//! Corpus: <https://github.com/SingleStepTests/65x02> (MIT, © Tom Harte and
//! contributors). Fetched by `scripts/fetch-testdata.sh sst-65x02`; never
//! committed.
//!
//! Format, per `nes6502/README.md` upstream:
//!
//! ```json
//! { "name": "a9 c3 7a",
//!   "initial": { "pc": 33710, "s": 215, "a": 22, "x": 214, "y": 9, "p": 162,
//!                "ram": [[33710, 169], [33711, 195]] },
//!   "final":   { "pc": 33712, ... },
//!   "cycles":  [[33710, 169, "read"], [33711, 195, "read"]] }
//! ```

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use crate::cpu::{Bus6502, Cpu6502, Regs, Variant, flags_str};
use crate::json::{Reader, Result as JsonResult};

/// How many failing vectors get a full diff before the report starts counting
/// only. Five is enough to see a pattern; ten thousand is enough to fill a disk.
const DETAIL_CAP: usize = 5;

/// A hard ceiling on bus accesses per instruction. The longest documented 6502
/// instruction is 8 cycles (indirect,Y RMW); 64 leaves room for a core that is
/// merely wrong rather than looping, and turns a hang into a diagnosable
/// failure.
const MAX_CYCLES: usize = 64;

// ---------------------------------------------------------------------------
// The vector model
// ---------------------------------------------------------------------------

/// One recorded bus access.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Cycle {
    /// Address on the bus.
    pub(crate) addr: u16,
    /// Data on the bus.
    pub(crate) value: u8,
    /// True for a write, false for a read.
    pub(crate) write: bool,
}

impl Cycle {
    fn kind(&self) -> &'static str {
        if self.write { "write" } else { "read" }
    }
}

/// A processor state plus the sparse RAM that goes with it.
#[derive(Debug, Clone, Default)]
pub(crate) struct State {
    /// Register file.
    pub(crate) regs: Regs,
    /// Sparse memory contents, as `(address, value)`.
    pub(crate) ram: Vec<(u16, u8)>,
}

/// One test vector.
#[derive(Debug, Clone, Default)]
pub(crate) struct Vector {
    /// The upstream name, e.g. `"a9 c3 7a"` — the opcode and its operands.
    pub(crate) name: String,
    /// State before the instruction.
    pub(crate) initial: State,
    /// State after it.
    pub(crate) expected: State,
    /// The expected bus trace, one entry per cycle.
    pub(crate) cycles: Vec<Cycle>,
}

/// Parse one opcode file: a JSON array of 10 000 vectors.
///
/// Written against the streaming reader rather than a DOM: the whole corpus is
/// roughly 800 MB of JSON, and building a tree for it before looking at it
/// would dominate the runtime of the suite.
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
    let mut seen_initial = false;
    let mut seen_final = false;
    r.object(|r, key| {
        match key {
            "name" => v.name = r.string()?,
            "initial" => {
                v.initial = parse_state(r)?;
                seen_initial = true;
            }
            "final" => {
                v.expected = parse_state(r)?;
                seen_final = true;
            }
            "cycles" => {
                r.array(|r, _| {
                    v.cycles.push(parse_cycle(r)?);
                    Ok(())
                })?;
            }
            // Unknown members are skipped rather than rejected: the upstream
            // format has gained fields before (the 8088 corpus carries extras)
            // and a new one is not a reason to stop testing the 6502.
            _ => r.skip_value()?,
        }
        Ok(())
    })?;
    if !seen_initial || !seen_final {
        return Err(r.err("vector is missing an `initial` or `final` section"));
    }
    Ok(v)
}

fn parse_state(r: &mut Reader<'_>) -> JsonResult<State> {
    let mut s = State::default();
    r.object(|r, key| {
        match key {
            "pc" => s.regs.pc = r.u64_in(0xffff)? as u16,
            "s" => s.regs.s = r.u64_in(0xff)? as u8,
            "a" => s.regs.a = r.u64_in(0xff)? as u8,
            "x" => s.regs.x = r.u64_in(0xff)? as u8,
            "y" => s.regs.y = r.u64_in(0xff)? as u8,
            "p" => s.regs.p = r.u64_in(0xff)? as u8,
            "ram" => {
                r.array(|r, _| {
                    let mut addr = 0u16;
                    let mut val = 0u8;
                    let n = r.array(|r, i| {
                        match i {
                            0 => addr = r.u64_in(0xffff)? as u16,
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
        value: 0,
        write: false,
    };
    let n = r.array(|r, i| {
        match i {
            0 => c.addr = r.u64_in(0xffff)? as u16,
            1 => c.value = r.u64_in(0xff)? as u8,
            2 => {
                let kind = r.string()?;
                c.write = match kind.as_str() {
                    "read" => false,
                    "write" => true,
                    other => return Err(r.err(format!("unknown cycle type {other:?}"))),
                };
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

// ---------------------------------------------------------------------------
// The bus
// ---------------------------------------------------------------------------

/// A flat 64 KiB of RAM that records every access.
///
/// The corpus states plainly that the whole address space is RAM — the NES
/// memory map is deliberately not modelled here, because this suite tests the
/// core and nothing else.
#[derive(Debug)]
pub(crate) struct TraceBus {
    mem: Vec<u8>,
    trace: Vec<Cycle>,
}

/// Panic message raised when a core exceeds [`MAX_CYCLES`], to convert a
/// runaway instruction into a reported failure instead of a hung test binary.
const CYCLE_OVERRUN: &str = "rsemu-conformance: cycle overrun";

impl TraceBus {
    /// A zeroed bus.
    pub(crate) fn new() -> Self {
        TraceBus {
            mem: vec![0; 0x1_0000],
            trace: Vec::with_capacity(16),
        }
    }

    /// Reset to zeros and apply a vector's sparse initial RAM.
    fn load(&mut self, ram: &[(u16, u8)]) {
        self.mem.fill(0);
        for &(addr, val) in ram {
            self.mem[addr as usize] = val;
        }
        self.trace.clear();
    }

    fn guard(&self) {
        assert!(self.trace.len() < MAX_CYCLES, "{}", CYCLE_OVERRUN);
    }
}

impl Default for TraceBus {
    fn default() -> Self {
        Self::new()
    }
}

impl Bus6502 for TraceBus {
    fn read(&mut self, addr: u16) -> u8 {
        self.guard();
        let value = self.mem[addr as usize];
        self.trace.push(Cycle {
            addr,
            value,
            write: false,
        });
        value
    }

    fn write(&mut self, addr: u16, value: u8) {
        self.guard();
        self.mem[addr as usize] = value;
        self.trace.push(Cycle {
            addr,
            value,
            write: true,
        });
    }
}

// ---------------------------------------------------------------------------
// Running one vector
// ---------------------------------------------------------------------------

/// Everything that can be wrong with one vector's outcome.
#[derive(Debug, Default)]
pub(crate) struct Mismatch {
    /// One short line per category, for grouping failures in the summary.
    pub(crate) reasons: Vec<String>,
    /// The full diff, for a human.
    pub(crate) detail: String,
}

impl Mismatch {
    fn note(&mut self, reason: impl Into<String>) {
        self.reasons.push(reason.into());
    }
}

/// Run one vector. `bus` is reused across vectors to avoid reallocating 64 KiB
/// ten thousand times per opcode.
pub(crate) fn run_vector(
    cpu: &mut dyn Cpu6502,
    bus: &mut TraceBus,
    v: &Vector,
) -> Result<(), Mismatch> {
    bus.load(&v.initial.ram);
    cpu.set_regs(v.initial.regs);

    // The core under test is exactly the code most likely to panic or run away,
    // and one bad opcode must not take down the other 255.
    let stepped = crate::harness::catching(|| cpu.step(bus));

    let mut m = Mismatch::default();
    let claimed = match stepped {
        Ok(n) => Some(n),
        Err(message) => {
            if message == CYCLE_OVERRUN {
                m.note("cycle overrun");
                let _ = writeln!(
                    m.detail,
                    "  the core made more than {MAX_CYCLES} bus accesses for one instruction"
                );
            } else {
                m.note("panic");
                let _ = writeln!(m.detail, "  the core panicked: {message}");
            }
            None
        }
    };

    let actual = cpu.regs();
    diff_regs(&mut m, v.expected.regs, actual);
    diff_cycles(&mut m, &v.cycles, &bus.trace, claimed);
    diff_memory(&mut m, v, &bus.mem);

    if m.reasons.is_empty() { Ok(()) } else { Err(m) }
}

fn diff_regs(m: &mut Mismatch, want: Regs, got: Regs) {
    if want == got {
        return;
    }
    m.note("registers");
    let _ = writeln!(m.detail, "  registers:");
    let _ = writeln!(m.detail, "    expected  {want}");
    let _ = writeln!(m.detail, "    actual    {got}");
    if want.p != got.p {
        let diff = want.p ^ got.p;
        let names: String = crate::cpu::FLAG_NAMES
            .iter()
            .filter(|&&(bit, _)| diff & bit != 0)
            .map(|&(_, ch)| ch)
            .collect();
        let _ = writeln!(
            m.detail,
            "    P differs in {names}  (expected {}, actual {})",
            flags_str(want.p),
            flags_str(got.p)
        );
    }
}

fn diff_cycles(m: &mut Mismatch, want: &[Cycle], got: &[Cycle], claimed: Option<u32>) {
    if let Some(n) = claimed
        && n as usize != got.len()
    {
        m.note("cycle count disagrees with bus accesses");
        let _ = writeln!(
            m.detail,
            "  the core reported {n} cycles but made {} bus accesses",
            got.len()
        );
    }
    if want == got {
        return;
    }
    if want.len() != got.len() {
        m.note("cycle count");
    } else {
        m.note("bus trace");
    }
    let _ = writeln!(
        m.detail,
        "  bus trace ({} expected, {} recorded):",
        want.len(),
        got.len()
    );
    let _ = writeln!(m.detail, "     #  expected              actual");
    for i in 0..want.len().max(got.len()) {
        let e = want.get(i);
        let a = got.get(i);
        let mark = if e == a { ' ' } else { '<' };
        let _ = writeln!(
            m.detail,
            "    {i:2}  {:<20}  {:<20} {mark}",
            cycle_str(e),
            cycle_str(a)
        );
    }
}

fn cycle_str(c: Option<&Cycle>) -> String {
    match c {
        Some(c) => format!("${:04X} = {:02X} {}", c.addr, c.value, c.kind()),
        None => "—".to_string(),
    }
}

/// Compare the whole 64 KiB, not just the addresses the vector lists.
///
/// The listed `final` addresses are what upstream checks, but a stray write
/// anywhere else is a real bug that a sparse comparison would miss entirely,
/// and the full compare costs a single `memcmp`-shaped scan.
fn diff_memory(m: &mut Mismatch, v: &Vector, actual: &[u8]) {
    let mut want = vec![0u8; 0x1_0000];
    for &(addr, val) in &v.initial.ram {
        want[addr as usize] = val;
    }
    for &(addr, val) in &v.expected.ram {
        want[addr as usize] = val;
    }
    if want == actual {
        return;
    }
    m.note("memory");
    let _ = writeln!(m.detail, "  memory:");
    let mut shown = 0;
    for (addr, (&w, &a)) in want.iter().zip(actual.iter()).enumerate() {
        if w == a {
            continue;
        }
        if shown == 8 {
            let _ = writeln!(m.detail, "    ... (further differences elided)");
            break;
        }
        let _ = writeln!(
            m.detail,
            "    ${addr:04X}: expected {w:02X}, actual {a:02X}"
        );
        shown += 1;
    }
}

// ---------------------------------------------------------------------------
// Running an opcode file
// ---------------------------------------------------------------------------

/// The outcome of one opcode's 10 000 vectors.
#[derive(Debug)]
pub(crate) struct OpcodeReport {
    /// The opcode, e.g. `0xa9`.
    pub(crate) opcode: u8,
    /// How many vectors ran.
    pub(crate) total: usize,
    /// Names of the vectors that failed, in file order.
    pub(crate) failed: Vec<String>,
    /// Diffs for the first few failures.
    pub(crate) details: String,
    /// Failure categories seen, deduplicated, for a one-line summary.
    pub(crate) categories: Vec<String>,
}

impl OpcodeReport {
    /// Did every vector pass?
    pub(crate) fn is_clean(&self) -> bool {
        self.failed.is_empty()
    }
}

/// Run every vector in one opcode file.
pub(crate) fn run_opcode(cpu: &mut dyn Cpu6502, opcode: u8, vectors: &[Vector]) -> OpcodeReport {
    let mut bus = TraceBus::new();
    let mut report = OpcodeReport {
        opcode,
        total: vectors.len(),
        failed: Vec::new(),
        details: String::new(),
        categories: Vec::new(),
    };
    for v in vectors {
        if let Err(m) = run_vector(cpu, &mut bus, v) {
            if report.failed.len() < DETAIL_CAP {
                let _ = writeln!(
                    report.details,
                    "  {:02x} vector {:?} — {}",
                    opcode,
                    v.name,
                    m.reasons.join(", ")
                );
                report.details.push_str(&m.detail);
                report.details.push('\n');
            }
            for reason in m.reasons {
                if !report.categories.contains(&reason) {
                    report.categories.push(reason);
                }
            }
            report.failed.push(v.name.clone());
        }
    }
    report
}

// ---------------------------------------------------------------------------
// Corpus discovery
// ---------------------------------------------------------------------------

/// Where a variant's `v1` vector files live.
pub(crate) fn corpus_dir(root: &Path, variant: Variant) -> PathBuf {
    root.join("sst-65x02").join(variant.corpus_dir()).join("v1")
}

/// Every opcode file present, as `(opcode, path)`, in opcode order.
///
/// Missing opcodes are not an error: the corpus omits the JAM/KIL opcodes,
/// which have no defined instruction to test, and fetching a subset while
/// bringing a core up is a legitimate workflow.
pub(crate) fn opcode_files(dir: &Path) -> std::io::Result<Vec<(u8, PathBuf)>> {
    let mut found = Vec::new();
    for entry in std::fs::read_dir(dir)? {
        let path = entry?.path();
        let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else {
            continue;
        };
        if path.extension().and_then(|e| e.to_str()) != Some("json") {
            continue;
        }
        if stem.len() != 2 {
            continue;
        }
        let Ok(opcode) = u8::from_str_radix(stem, 16) else {
            continue;
        };
        found.push((opcode, path));
    }
    found.sort_by_key(|&(op, _)| op);
    Ok(found)
}

/// Parse an opcode subset spec: a comma-separated list of hex opcodes and
/// inclusive hex ranges, e.g. `a9,ad,b1` or `a0-af,ea`.
pub(crate) fn parse_opcode_spec(spec: &str) -> Vec<u8> {
    let mut out = Vec::new();
    for part in spec.split(',').map(str::trim).filter(|s| !s.is_empty()) {
        match part.split_once('-') {
            Some((lo, hi)) => {
                if let (Ok(lo), Ok(hi)) = (u8::from_str_radix(lo, 16), u8::from_str_radix(hi, 16))
                    && lo <= hi
                {
                    out.extend(lo..=hi);
                }
            }
            None => {
                if let Ok(op) = u8::from_str_radix(part, 16) {
                    out.push(op);
                }
            }
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}

/// An opcode subset from `RSEMU_SST_OPCODES`.
///
/// Bringing a core up one addressing mode at a time is how this actually gets
/// used, and waiting on 2.5 million vectors to see whether `LDA #` works is a
/// good way to stop running the suite.
pub(crate) fn opcode_filter() -> Option<Vec<u8>> {
    let spec = std::env::var("RSEMU_SST_OPCODES").ok()?;
    let out = parse_opcode_spec(&spec);
    if out.is_empty() { None } else { Some(out) }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mock::{BrokenLda, MockCpu};

    /// One real vector, copied byte for byte from the upstream README's
    /// worked example (`nes6502/README.md`) — an `LDA (indirect),Y` that
    /// crosses a page boundary, so it exercises the dummy read too.
    const README_EXAMPLE: &[u8] = br#"[
      { "name": "b1 71 8b",
        "initial": { "pc": 9023, "s": 240, "a": 47, "x": 162, "y": 170, "p": 170,
          "ram": [[9023,177],[9024,113],[9025,139],[113,169],[114,89],[22867,214],[23123,37]] },
        "final": { "pc": 9025, "s": 240, "a": 37, "x": 162, "y": 170, "p": 40,
          "ram": [[113,169],[114,89],[9023,177],[9024,113],[9025,139],[22867,214],[23123,37]] },
        "cycles": [[9023,177,"read"],[9024,113,"read"],[113,169,"read"],[114,89,"read"],
                   [22867,214,"read"],[23123,37,"read"]] }
    ]"#;

    #[test]
    fn the_upstream_example_parses_field_for_field() {
        let v = parse_vectors(README_EXAMPLE).unwrap();
        assert_eq!(v.len(), 1);
        let v = &v[0];
        assert_eq!(v.name, "b1 71 8b");
        assert_eq!(
            v.initial.regs,
            Regs {
                pc: 9023,
                s: 240,
                a: 47,
                x: 162,
                y: 170,
                p: 170
            }
        );
        assert_eq!(
            v.expected.regs,
            Regs {
                pc: 9025,
                s: 240,
                a: 37,
                x: 162,
                y: 170,
                p: 40
            }
        );
        assert_eq!(v.initial.ram.len(), 7);
        assert_eq!(v.cycles.len(), 6);
        assert_eq!(
            v.cycles[4],
            Cycle {
                addr: 22867,
                value: 214,
                write: false
            }
        );
        assert!(v.cycles.iter().all(|c| !c.write));
    }

    #[test]
    fn a_write_cycle_round_trips() {
        let src = br#"[{"name":"8d","initial":{"pc":0,"s":0,"a":0,"x":0,"y":0,"p":0,"ram":[]},
                        "final":{"pc":0,"s":0,"a":0,"x":0,"y":0,"p":0,"ram":[]},
                        "cycles":[[4660,171,"write"]]}]"#;
        let v = parse_vectors(src).unwrap();
        assert_eq!(
            v[0].cycles[0],
            Cycle {
                addr: 0x1234,
                value: 0xab,
                write: true
            }
        );
    }

    #[test]
    fn corrupt_vectors_are_rejected_rather_than_silently_wrong() {
        for bad in [
            // p out of range
            &br#"[{"initial":{"p":300,"ram":[]},"final":{"ram":[]},"cycles":[]}]"#[..],
            // cycle type we do not know
            br#"[{"initial":{"ram":[]},"final":{"ram":[]},"cycles":[[1,2,"fetch"]]}]"#,
            // ram entry of the wrong arity
            br#"[{"initial":{"ram":[[1]]},"final":{"ram":[]},"cycles":[]}]"#,
            // missing final
            br#"[{"initial":{"ram":[]},"cycles":[]}]"#,
        ] {
            assert!(parse_vectors(bad).is_err());
        }
    }

    #[test]
    fn a_correct_core_passes_a_real_vector() {
        // MockCpu implements exactly three opcodes, LDA # among them. This is
        // the harness testing itself: parser, bus, comparison and diff all run
        // for real, with no 6502 core in the tree.
        let src = br#"[{"name":"a9 c3 7a",
          "initial":{"pc":33710,"s":215,"a":22,"x":214,"y":9,"p":162,
                     "ram":[[33710,169],[33711,195],[33712,122]]},
          "final":{"pc":33712,"s":215,"a":195,"x":214,"y":9,"p":160,
                   "ram":[[33710,169],[33711,195],[33712,122]]},
          "cycles":[[33710,169,"read"],[33711,195,"read"]]}]"#;
        let vectors = parse_vectors(src).unwrap();
        let mut cpu = MockCpu::default();
        let report = run_opcode(&mut cpu, 0xa9, &vectors);
        assert!(report.is_clean(), "{}", report.details);
        assert_eq!(report.total, 1);
    }

    #[test]
    fn a_wrong_core_fails_and_the_diff_names_what_is_wrong() {
        let src = br#"[{"name":"a9 c3 7a",
          "initial":{"pc":33710,"s":215,"a":22,"x":214,"y":9,"p":162,
                     "ram":[[33710,169],[33711,195],[33712,122]]},
          "final":{"pc":33712,"s":215,"a":195,"x":214,"y":9,"p":160,
                   "ram":[[33710,169],[33711,195],[33712,122]]},
          "cycles":[[33710,169,"read"],[33711,195,"read"]]}]"#;
        let vectors = parse_vectors(src).unwrap();
        let mut cpu = BrokenLda::default();
        let report = run_opcode(&mut cpu, 0xa9, &vectors);
        assert_eq!(report.failed.len(), 1);
        assert!(
            report.categories.contains(&"registers".to_string()),
            "{:?}",
            report.categories
        );
        // The N flag is the one BrokenLda gets wrong, and the diff says so.
        assert!(
            report.details.contains("P differs in N"),
            "{}",
            report.details
        );
    }

    #[test]
    fn a_stray_write_outside_the_listed_addresses_is_caught() {
        // NOP is a 2-cycle no-op; StrayWrite scribbles on $1234 as well.
        let src = br#"[{"name":"ea",
          "initial":{"pc":0,"s":0,"a":0,"x":0,"y":0,"p":0,"ram":[[0,234],[1,0]]},
          "final":{"pc":1,"s":0,"a":0,"x":0,"y":0,"p":0,"ram":[[0,234],[1,0]]},
          "cycles":[[0,234,"read"],[1,0,"read"]]}]"#;
        let vectors = parse_vectors(src).unwrap();
        let mut cpu = crate::mock::StrayWrite::default();
        let report = run_opcode(&mut cpu, 0xea, &vectors);
        assert_eq!(report.failed.len(), 1);
        assert!(
            report.categories.iter().any(|c| c == "memory"),
            "{:?}",
            report.categories
        );
        assert!(report.details.contains("$1234"), "{}", report.details);
    }

    #[test]
    fn a_runaway_core_is_reported_not_hung() {
        let src = br#"[{"name":"ea",
          "initial":{"pc":0,"s":0,"a":0,"x":0,"y":0,"p":0,"ram":[[0,234]]},
          "final":{"pc":1,"s":0,"a":0,"x":0,"y":0,"p":0,"ram":[[0,234]]},
          "cycles":[[0,234,"read"],[1,0,"read"]]}]"#;
        let vectors = parse_vectors(src).unwrap();
        let mut cpu = crate::mock::Runaway;
        let report = run_opcode(&mut cpu, 0xea, &vectors);
        assert!(
            report.categories.iter().any(|c| c == "cycle overrun"),
            "{:?}",
            report.categories
        );
    }

    #[test]
    fn a_panicking_core_is_reported_not_fatal() {
        let src = br#"[{"name":"ea",
          "initial":{"pc":0,"s":0,"a":0,"x":0,"y":0,"p":0,"ram":[[0,234]]},
          "final":{"pc":1,"s":0,"a":0,"x":0,"y":0,"p":0,"ram":[[0,234]]},
          "cycles":[[0,234,"read"],[1,0,"read"]]}]"#;
        let vectors = parse_vectors(src).unwrap();
        let mut cpu = crate::mock::Panicky::default();
        let report = run_opcode(&mut cpu, 0xea, &vectors);
        assert!(
            report.categories.iter().any(|c| c == "panic"),
            "{:?}",
            report.categories
        );
        assert!(
            report.details.contains("unimplemented opcode"),
            "{}",
            report.details
        );
    }

    #[test]
    fn a_lying_cycle_count_is_caught_even_when_the_trace_matches() {
        let src = br#"[{"name":"ea",
          "initial":{"pc":0,"s":0,"a":0,"x":0,"y":0,"p":0,"ram":[[0,234],[1,0]]},
          "final":{"pc":1,"s":0,"a":0,"x":0,"y":0,"p":0,"ram":[[0,234],[1,0]]},
          "cycles":[[0,234,"read"],[1,0,"read"]]}]"#;
        let vectors = parse_vectors(src).unwrap();
        let mut cpu = crate::mock::LyingCycleCount::default();
        let report = run_opcode(&mut cpu, 0xea, &vectors);
        assert!(
            report
                .categories
                .iter()
                .any(|c| c.contains("cycle count disagrees")),
            "{:?}",
            report.categories
        );
    }

    #[test]
    fn the_opcode_filter_understands_lists_and_ranges() {
        assert_eq!(parse_opcode_spec("a9, ad ,b1"), vec![0xa9, 0xad, 0xb1]);
        assert_eq!(parse_opcode_spec("00-03"), vec![0, 1, 2, 3]);
        assert_eq!(parse_opcode_spec("ea,ea"), vec![0xea]);
        assert_eq!(parse_opcode_spec("ff-00"), Vec::<u8>::new());
        assert_eq!(parse_opcode_spec("zz"), Vec::<u8>::new());
    }
}
