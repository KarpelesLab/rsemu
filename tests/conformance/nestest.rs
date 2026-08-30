//! `nestest` trace comparison — gate two for a 6502 core.
//!
//! The classic 6502 bring-up. `nestest.nes` has an automated mode: entered at
//! `PC = $C000` rather than through the reset vector, it exercises the whole
//! instruction set — documented opcodes first, then the unofficial ones — and
//! stops without ever needing a PPU frame or a button press. `nestest.log` is a
//! reference trace of that run, one line per instruction, with the register file
//! and cycle count *before* each instruction executes.
//!
//! What makes it worth running after SingleStepTests, which is strictly more
//! detailed: it is a **cumulative** check. A vector suite resets the world
//! before every instruction, so a core can pass all 2.5 million vectors and
//! still drift on the first branch that mispredicts a page cross, because
//! nothing ever asks it to run 8 991 instructions in a row and still agree on
//! the cycle count. This does.
//!
//! Neither the ROM nor the log carries a licence, so both are fetch-only
//! (`docs/testing/conformance-suites.md`).
//!
//! ## Reference line
//!
//! ```text
//! C000  4C F5 C5  JMP $C5F5                       A:00 X:00 Y:00 P:24 SP:FD PPU:  0, 21 CYC:7
//! ```
//!
//! The disassembly column is Nintendulator's formatting convention, not
//! anything the hardware specifies, so it is compared only on request
//! (`RSEMU_NESTEST_DISASM=1`). The registers and the cycle count are compared
//! always; the PPU columns need a PPU and are reported, not asserted.

use std::fmt::Write as _;

use crate::cpu::{Bus6502, Cpu6502, Regs, flags_str};

/// State the log says the CPU should be in, immediately before one instruction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct TraceLine {
    /// 1-based line number in the log.
    pub(crate) line_no: usize,
    /// Registers at the start of the instruction.
    pub(crate) regs: Regs,
    /// The instruction's bytes, as the log printed them (1 to 3 of them).
    pub(crate) bytes: Vec<u8>,
    /// The disassembly column, trimmed.
    pub(crate) disasm: String,
    /// `(scanline, dot)` from the `PPU:` column, if present.
    pub(crate) ppu: Option<(u32, u32)>,
    /// Total CPU cycles elapsed before this instruction.
    pub(crate) cyc: u64,
}

impl TraceLine {
    /// The reference line, rebuilt in the log's own shape for side-by-side diffs.
    pub(crate) fn render(&self) -> String {
        let bytes: Vec<String> = self.bytes.iter().map(|b| format!("{b:02X}")).collect();
        format!(
            "{:04X}  {:<8}  {:<31} A:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X} CYC:{}",
            self.regs.pc,
            bytes.join(" "),
            self.disasm,
            self.regs.a,
            self.regs.x,
            self.regs.y,
            self.regs.p,
            self.regs.s,
            self.cyc
        )
    }
}

/// A malformed reference log.
#[derive(Debug)]
pub(crate) struct ParseError {
    /// 1-based line number.
    pub(crate) line: usize,
    /// What was wrong.
    pub(crate) msg: String,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "nestest.log line {}: {}", self.line, self.msg)
    }
}

/// Parse the whole reference log.
pub(crate) fn parse_log(text: &str) -> Result<Vec<TraceLine>, ParseError> {
    text.lines()
        .enumerate()
        .filter(|(_, l)| !l.trim().is_empty())
        .map(|(i, l)| parse_line(i + 1, l))
        .collect()
}

fn parse_line(line_no: usize, line: &str) -> Result<TraceLine, ParseError> {
    let err = |msg: &str| ParseError {
        line: line_no,
        msg: msg.to_string(),
    };

    if line.len() < 6 {
        return Err(err("line is too short to hold a program counter"));
    }
    let pc = u16::from_str_radix(&line[0..4], 16).map_err(|_| err("bad program counter"))?;

    // Columns 6..14 hold up to three space-separated hex bytes. The width is
    // fixed in every generator of this format, but trailing spaces vary, so the
    // field is split rather than sliced byte by byte.
    let byte_col = line
        .get(6..14)
        .ok_or_else(|| err("no opcode-byte column"))?;
    let mut bytes = Vec::new();
    for tok in byte_col.split_whitespace() {
        bytes.push(u8::from_str_radix(tok, 16).map_err(|_| err("bad opcode byte"))?);
    }
    if bytes.is_empty() {
        return Err(err("no opcode bytes"));
    }

    // The register block is at the end of the line and is the only place `A:`
    // appears — the disassembly column can contain a bare `A` (as in `LSR A`)
    // but never `A:`.
    let regs_at = line.rfind("A:").ok_or_else(|| err("no register block"))?;
    let disasm = line.get(14..regs_at).unwrap_or("").trim().to_string();
    let tail = &line[regs_at..];

    let hex = |key: &str| -> Result<u8, ParseError> {
        let at = tail
            .find(key)
            .ok_or_else(|| err(&format!("no {key} field")))?;
        let raw = tail
            .get(at + key.len()..at + key.len() + 2)
            .ok_or_else(|| err(&format!("truncated {key} field")))?;
        u8::from_str_radix(raw, 16).map_err(|_| err(&format!("bad {key} field")))
    };

    let regs = Regs {
        pc,
        a: hex("A:")?,
        x: hex("X:")?,
        y: hex("Y:")?,
        p: hex("P:")?,
        s: hex("SP:")?,
    };

    let ppu = tail.find("PPU:").and_then(|at| {
        let rest = &tail[at + 4..];
        let end = rest.find("CYC:").unwrap_or(rest.len());
        let (a, b) = rest[..end].split_once(',')?;
        Some((a.trim().parse().ok()?, b.trim().parse().ok()?))
    });

    let cyc_at = tail.find("CYC:").ok_or_else(|| err("no CYC field"))?;
    let cyc = tail[cyc_at + 4..]
        .trim()
        .parse::<u64>()
        .map_err(|_| err("bad CYC field"))?;

    Ok(TraceLine {
        line_no,
        regs,
        bytes,
        disasm,
        ppu,
        cyc,
    })
}

// ---------------------------------------------------------------------------
// The bus
// ---------------------------------------------------------------------------

/// The smallest bus `nestest`'s automated mode needs: work RAM and the
/// cartridge.
///
/// No PPU, no APU, no controllers — automated mode is CPU-only by design, which
/// is exactly why this gate comes before AccuracyCoin. Accesses outside the two
/// mapped windows are counted and reported rather than faulted: if the count is
/// non-zero the comparison is still valid, but the caveat belongs in the report.
#[derive(Debug)]
pub(crate) struct NestestBus {
    ram: [u8; 0x800],
    prg: Vec<u8>,
    /// Accesses that hit no modelled device.
    pub(crate) unmapped: u64,
}

impl NestestBus {
    /// Build a bus from an iNES image.
    pub(crate) fn from_ines(image: &[u8]) -> Result<NestestBus, String> {
        if image.len() < 16 || &image[0..4] != b"NES\x1a" {
            return Err("not an iNES image".into());
        }
        let prg_banks = usize::from(image[4]);
        if prg_banks == 0 {
            return Err("iNES header claims zero PRG banks".into());
        }
        let trainer = if image[6] & 0x04 != 0 { 512 } else { 0 };
        let start = 16 + trainer;
        let len = prg_banks * 16 * 1024;
        let prg = image
            .get(start..start + len)
            .ok_or_else(|| format!("image is shorter than its {prg_banks}-bank header claims"))?
            .to_vec();
        Ok(NestestBus {
            ram: [0; 0x800],
            prg,
            unmapped: 0,
        })
    }

    /// A side-effect-free peek, for reading the result codes afterwards.
    pub(crate) fn peek(&self, addr: u16) -> u8 {
        match addr {
            0x0000..=0x1fff => self.ram[usize::from(addr) & 0x7ff],
            0x8000..=0xffff => {
                // NROM-128 mirrors its single 16 KiB bank into both windows.
                let off = usize::from(addr - 0x8000) % self.prg.len();
                self.prg[off]
            }
            _ => 0,
        }
    }

    /// The three bytes at `pc`, for comparing against the log's byte column.
    pub(crate) fn peek3(&self, pc: u16) -> [u8; 3] {
        [
            self.peek(pc),
            self.peek(pc.wrapping_add(1)),
            self.peek(pc.wrapping_add(2)),
        ]
    }
}

impl Bus6502 for NestestBus {
    fn read(&mut self, addr: u16) -> u8 {
        if !matches!(addr, 0x0000..=0x1fff | 0x8000..=0xffff) {
            self.unmapped += 1;
        }
        self.peek(addr)
    }

    fn write(&mut self, addr: u16, value: u8) {
        match addr {
            0x0000..=0x1fff => self.ram[usize::from(addr) & 0x7ff] = value,
            // Writes to ROM are dropped, which is what an NROM cart does.
            0x8000..=0xffff => {}
            _ => self.unmapped += 1,
        }
    }
}

/// The state `nestest` expects when entered in automated mode.
///
/// `P = $24` is bits 5 and 2 (the always-set bit and I), `S = $FD` is the stack
/// pointer after the three pushes a reset performs, and the run is credited with
/// the 7 cycles the reset sequence took — all three are what the reference log's
/// first line asserts.
pub(crate) fn automated_entry() -> (Regs, u64) {
    (
        Regs {
            pc: 0xc000,
            s: 0xfd,
            a: 0,
            x: 0,
            y: 0,
            p: 0x24,
        },
        7,
    )
}

/// Result codes `nestest` leaves in RAM: `$02` for the documented opcodes and
/// `$03` for the unofficial ones. `00` in both means everything passed.
pub(crate) const RESULT_ADDRS: (u16, u16) = (0x0002, 0x0003);

// ---------------------------------------------------------------------------
// The comparison
// ---------------------------------------------------------------------------

/// How much of the log to print before a divergence.
const CONTEXT: usize = 8;

/// What the comparison found.
#[derive(Debug)]
pub(crate) struct Report {
    /// Instructions that matched before the first divergence.
    pub(crate) matched: usize,
    /// The whole log's length.
    pub(crate) expected: usize,
    /// The divergence, if there was one.
    pub(crate) divergence: Option<String>,
    /// `$02` and `$03` after the run.
    pub(crate) result_codes: (u8, u8),
    /// Accesses that hit no modelled device.
    pub(crate) unmapped: u64,
}

impl Report {
    /// Did the trace match to the end and did the ROM report success?
    pub(crate) fn is_clean(&self) -> bool {
        self.divergence.is_none() && self.matched == self.expected && self.result_codes == (0, 0)
    }
}

/// Run the ROM and compare against the log, stopping at the first divergence.
pub(crate) fn compare(
    cpu: &mut dyn Cpu6502,
    bus: &mut NestestBus,
    log: &[TraceLine],
    check_disasm: bool,
) -> Report {
    let (entry, mut cyc) = automated_entry();
    cpu.set_regs(entry);

    let mut divergence = None;
    let mut matched = 0;

    for (i, want) in log.iter().enumerate() {
        let got = cpu.regs();
        let bytes = bus.peek3(got.pc);
        let got_bytes = &bytes[..want.bytes.len().min(3)];

        let mut problems: Vec<String> = Vec::new();
        if got.pc != want.regs.pc {
            problems.push(format!(
                "PC: expected ${:04X}, got ${:04X}",
                want.regs.pc, got.pc
            ));
        }
        if got.a != want.regs.a {
            problems.push(format!(
                "A: expected {:02X}, got {:02X}",
                want.regs.a, got.a
            ));
        }
        if got.x != want.regs.x {
            problems.push(format!(
                "X: expected {:02X}, got {:02X}",
                want.regs.x, got.x
            ));
        }
        if got.y != want.regs.y {
            problems.push(format!(
                "Y: expected {:02X}, got {:02X}",
                want.regs.y, got.y
            ));
        }
        if got.p != want.regs.p {
            problems.push(format!(
                "P: expected {:02X} [{}], got {:02X} [{}]",
                want.regs.p,
                flags_str(want.regs.p),
                got.p,
                flags_str(got.p)
            ));
        }
        if got.s != want.regs.s {
            problems.push(format!(
                "SP: expected {:02X}, got {:02X}",
                want.regs.s, got.s
            ));
        }
        if cyc != want.cyc {
            problems.push(format!(
                "CYC: expected {}, got {} (drift {})",
                want.cyc,
                cyc,
                cyc as i64 - want.cyc as i64
            ));
        }
        // Only meaningful once PC agrees; otherwise it is noise about the
        // wrong instruction.
        if got.pc == want.regs.pc && got_bytes != want.bytes.as_slice() {
            problems.push(format!(
                "opcode bytes: expected {}, ROM holds {}",
                hex_bytes(&want.bytes),
                hex_bytes(got_bytes)
            ));
        }
        if check_disasm
            && problems.is_empty()
            && let Some(text) = cpu.disassemble(got.pc, got_bytes)
            && text != want.disasm
        {
            problems.push(format!(
                "disassembly: expected {:?}, got {:?}",
                want.disasm, text
            ));
        }

        if !problems.is_empty() {
            divergence = Some(render_divergence(log, i, &got, cyc, &problems));
            break;
        }

        matched += 1;
        match crate::harness::catching(|| cpu.step(bus)) {
            Ok(cycles) => cyc += u64::from(cycles),
            Err(message) => {
                // A core that dies mid-trace is still worth a report: the
                // context below says which instruction it died on.
                divergence = Some(render_divergence(
                    log,
                    i,
                    &got,
                    cyc,
                    &[format!(
                        "the core panicked while executing this instruction: {message}"
                    )],
                ));
                matched -= 1;
                break;
            }
        }
    }

    Report {
        matched,
        expected: log.len(),
        divergence,
        result_codes: (bus.peek(RESULT_ADDRS.0), bus.peek(RESULT_ADDRS.1)),
        unmapped: bus.unmapped,
    }
}

fn hex_bytes(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn render_divergence(
    log: &[TraceLine],
    at: usize,
    got: &Regs,
    cyc: u64,
    problems: &[String],
) -> String {
    let mut s = String::new();
    let want = &log[at];
    let _ = writeln!(
        s,
        "first divergence at log line {} (instruction {} of {})",
        want.line_no,
        at + 1,
        log.len()
    );
    let _ = writeln!(s, "\n  ...the last {CONTEXT} instructions that matched:");
    for line in &log[at.saturating_sub(CONTEXT)..at] {
        let _ = writeln!(s, "    {}", line.render());
    }
    let _ = writeln!(s, "\n  expected: {}", want.render());
    let _ = writeln!(
        s,
        "  actual:   {:04X}  {:<8}  {:<31} A:{:02X} X:{:02X} Y:{:02X} P:{:02X} SP:{:02X} CYC:{}",
        got.pc, "", "", got.a, got.x, got.y, got.p, got.s, cyc
    );
    let _ = writeln!(s, "\n  what differs:");
    for p in problems {
        let _ = writeln!(s, "    {p}");
    }
    let _ = writeln!(s, "\n  the next few reference lines, for orientation:");
    for line in log.iter().skip(at + 1).take(3) {
        let _ = writeln!(s, "    {}", line.render());
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    // The first three lines of the reference log, verbatim.
    const SAMPLE: &str = concat!(
        "C000  4C F5 C5  JMP $C5F5                       A:00 X:00 Y:00 P:24 SP:FD PPU:  0, 21 CYC:7\n",
        "C5F5  A2 00     LDX #$00                        A:00 X:00 Y:00 P:24 SP:FD PPU:  0, 30 CYC:10\n",
        "C5F7  86 00     STX $00 = 00                    A:00 X:00 Y:00 P:26 SP:FD PPU:  0, 36 CYC:12\n",
    );

    #[test]
    fn the_reference_format_parses_field_for_field() {
        let log = parse_log(SAMPLE).unwrap();
        assert_eq!(log.len(), 3);

        assert_eq!(
            log[0].regs,
            Regs {
                pc: 0xc000,
                a: 0,
                x: 0,
                y: 0,
                p: 0x24,
                s: 0xfd
            }
        );
        assert_eq!(log[0].bytes, vec![0x4c, 0xf5, 0xc5]);
        assert_eq!(log[0].disasm, "JMP $C5F5");
        assert_eq!(log[0].ppu, Some((0, 21)));
        assert_eq!(log[0].cyc, 7);

        // Two-byte instruction, and a disassembly column carrying an operand
        // annotation the format appends after `=`.
        assert_eq!(log[2].bytes, vec![0x86, 0x00]);
        assert_eq!(log[2].disasm, "STX $00 = 00");
        assert_eq!(log[2].regs.p, 0x26);
        assert_eq!(log[2].cyc, 12);
    }

    #[test]
    fn the_accumulator_addressing_mode_does_not_confuse_the_register_scan() {
        // `LSR A` puts a bare `A` in the disassembly column; the register block
        // is still found, because only it contains `A:`.
        let line = "C68A  4A        LSR A                           \
                    A:55 X:FF Y:15 P:65 SP:FB PPU:233, 30 CYC:26000";
        let parsed = parse_line(1, line).unwrap();
        assert_eq!(parsed.disasm, "LSR A");
        assert_eq!(parsed.regs.a, 0x55);
        assert_eq!(parsed.cyc, 26000);
    }

    #[test]
    fn a_log_without_ppu_columns_still_parses() {
        let line = "C000  4C F5 C5  JMP $C5F5                       \
                    A:00 X:00 Y:00 P:24 SP:FD CYC:7";
        let parsed = parse_line(1, line).unwrap();
        assert_eq!(parsed.ppu, None);
        assert_eq!(parsed.cyc, 7);
    }

    #[test]
    fn malformed_lines_are_rejected() {
        assert!(parse_log("garbage\n").is_err());
        assert!(parse_log("C000\n").is_err());
        // Register block missing entirely.
        assert!(parse_log("C000  4C F5 C5  JMP $C5F5\n").is_err());
    }

    #[test]
    fn an_ines_image_maps_into_both_rom_windows() {
        let mut image = vec![0u8; 16 + 16 * 1024];
        image[0..4].copy_from_slice(b"NES\x1a");
        image[4] = 1; // one 16 KiB PRG bank
        image[16] = 0xaa; // first byte of PRG
        image[16 + 0x3fff] = 0xbb; // last byte
        let bus = NestestBus::from_ines(&image).unwrap();
        assert_eq!(bus.peek(0x8000), 0xaa);
        assert_eq!(bus.peek(0xc000), 0xaa, "NROM-128 mirrors its bank");
        assert_eq!(bus.peek(0xffff), 0xbb);
    }

    #[test]
    fn ram_mirrors_every_2_kib() {
        let mut image = vec![0u8; 16 + 16 * 1024];
        image[0..4].copy_from_slice(b"NES\x1a");
        image[4] = 1;
        let mut bus = NestestBus::from_ines(&image).unwrap();
        bus.write(0x0003, 0xa5);
        for base in [0x0000u16, 0x0800, 0x1000, 0x1800] {
            assert_eq!(bus.peek(base + 3), 0xa5);
        }
        assert_eq!(bus.unmapped, 0);
    }

    #[test]
    fn a_bad_image_is_rejected() {
        assert!(NestestBus::from_ines(b"not a rom").is_err());
        assert!(
            NestestBus::from_ines(b"NES\x1a\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")
                .is_err()
        );
    }

    /// A three-instruction log the mock core can actually satisfy, proving the
    /// comparison loop, the cycle accounting and the divergence report all work
    /// without a 6502 in the tree.
    fn mock_setup() -> (Vec<TraceLine>, NestestBus) {
        // LDA #$C3 ; NOP ; STA $0200 — laid out at $C000 in a synthetic NROM.
        let program = [0xa9, 0xc3, 0xea, 0x8d, 0x00, 0x02];
        let mut image = vec![0u8; 16 + 16 * 1024];
        image[0..4].copy_from_slice(b"NES\x1a");
        image[4] = 1;
        // $C000 is offset 0 of the mirrored 16 KiB bank.
        image[16..16 + program.len()].copy_from_slice(&program);
        let bus = NestestBus::from_ines(&image).unwrap();

        let log = parse_log(concat!(
            "C000  A9 C3     LDA #$C3                        A:00 X:00 Y:00 P:24 SP:FD CYC:7\n",
            "C002  EA        NOP                             A:C3 X:00 Y:00 P:A4 SP:FD CYC:9\n",
            "C003  8D 00 02  STA $0200 = 00                  A:C3 X:00 Y:00 P:A4 SP:FD CYC:11\n",
        ))
        .unwrap();
        (log, bus)
    }

    #[test]
    fn a_correct_core_walks_the_whole_log() {
        let (log, mut bus) = mock_setup();
        let mut cpu = crate::mock::MockCpu::default();
        let report = compare(&mut cpu, &mut bus, &log, false);
        assert_eq!(report.matched, 3, "{:?}", report.divergence);
        assert!(report.divergence.is_none());
        assert_eq!(report.unmapped, 0);
    }

    #[test]
    fn a_wrong_core_diverges_and_the_report_says_where_and_why() {
        let (log, mut bus) = mock_setup();
        let mut cpu = crate::mock::BrokenLda::default();
        let report = compare(&mut cpu, &mut bus, &log, false);
        // The first instruction's *entry* state is right, so it matches; the
        // divergence shows up on the second line, where N should have been set.
        assert_eq!(report.matched, 1);
        let text = report.divergence.expect("should have diverged");
        assert!(text.contains("first divergence at log line 2"), "{text}");
        assert!(text.contains("P: expected A4"), "{text}");
        assert!(
            text.contains("the last 8 instructions that matched"),
            "{text}"
        );
    }

    #[test]
    fn a_cycle_count_drift_is_reported_with_its_sign() {
        let (log, mut bus) = mock_setup();
        let mut cpu = crate::mock::LyingCycleCount::default();
        // LyingCycleCount executes NOP semantics for everything, so it diverges
        // on registers too — but the cycle drift must be named explicitly.
        let report = compare(&mut cpu, &mut bus, &log, false);
        let text = report.divergence.expect("should have diverged");
        assert!(text.contains("CYC: expected 9, got 10 (drift 1)"), "{text}");
    }

    #[test]
    fn a_core_that_dies_mid_trace_still_produces_a_report() {
        let (log, mut bus) = mock_setup();
        let mut cpu = crate::mock::Panicky::default();
        let report = compare(&mut cpu, &mut bus, &log, false);
        assert_eq!(report.matched, 0);
        let text = report.divergence.expect("should have reported the panic");
        assert!(text.contains("the core panicked while executing"), "{text}");
        assert!(text.contains("unimplemented opcode"), "{text}");
    }

    #[test]
    fn disassembly_is_compared_only_when_asked() {
        let (log, mut bus) = mock_setup();
        let mut cpu = crate::mock::MockCpu::default();
        // The mock prints `STA $0200`; the log column says `STA $0200 = 00`.
        // Strict mode must notice, permissive mode must not.
        assert!(
            compare(&mut cpu, &mut bus, &log, false)
                .divergence
                .is_none()
        );

        let (log, mut bus) = mock_setup();
        let mut cpu = crate::mock::MockCpu::default();
        let strict = compare(&mut cpu, &mut bus, &log, true);
        let text = strict.divergence.expect("strict mode should have objected");
        assert!(text.contains("disassembly"), "{text}");
    }
}
