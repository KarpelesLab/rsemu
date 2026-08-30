//! Cross-cutting tests for the machine-description front end.
//!
//! Per-module behaviour is tested beside the code it belongs to; what lives
//! here needs the whole pipeline:
//!
//! * the worked NES example from `ROADMAP.md` §5, parsed into the expected
//!   tree — the language's acceptance test;
//! * the phase-2 gate fixture: a `template` instantiated four times inside a
//!   loop, from an `include`d file, with `param` overrides (§13);
//! * **golden error messages**, asserted character for character, because §13
//!   asks for error-message golden tests and because a diagnostic that drifts
//!   silently is a regression nobody notices;
//! * robustness: every truncation of a valid file, and a pile of generated
//!   garbage, must produce an error rather than a panic.

use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::core::Error;
use crate::machine::ast::{Expr, Stmt};
use crate::machine::span::SourceFile;
use crate::machine::{parse, parse_file};

/// §5's worked example, copied exactly, comments and all.
const NES: &str = r#"machine "nes" {
  param region = "ntsc"

  # One crystal, so every domain below is exactly related to every other.
  # The literal is rational because the real frequency is not an integer;
  # it affects wall-clock rate only, never the CPU:PPU ratio.
  osc master = 236250000/11 Hz           # 21477272.72… — NTSC colorburst × 6

  space cpubus  { width = 16, unassigned = open-bus }
  space ppubus  { width = 14, unassigned = open-bus }

  object ram "wram" { size = 2K }

  object cpu "mos6502" {
    clock  = master / 12                 # PPU advances exactly 3 dots per cycle
    space  = cpubus
    engine = "interp"
  }
  object ppu "nes.ppu" { clock = master / 4, space = ppubus }
  object apu "nes.apu" { clock = master / 12 }

  map cpubus 0x0000 size 0x2000 = mirror(wram)      # 2K mirrored 4×
  map cpubus 0x2000 size 0x2000 = mirror(ppu.regs)
  map cpubus 0x4000 size 0x0020 = apu.regs

  wire ppu.nmi   -> cpu.nmi
  wire apu.irq   -> cpu.irq
  wire cart.irq  -> cpu.irq                          # wired-OR, declared once
}
"#;

/// The phase-2 gate fixture in miniature: `include`, `template`, indexed
/// instantiation and `param` override in one file (§13).
const TEMPLATED: &str = r#"# A fragment pulled in from the search path.
include "pci-common.machine"

param cores = 4
param ram   = 4M

template cpu_complex(id, clock, l2 = 512K) {
  object cpu$id "riscv64" { clock = clock, space = mem$id }
  object l2$id "cache"    { size = l2 }
  wire cpu$id.irq -> plic.in$id
}

machine "quad" {
  osc master = 1 GHz

  for i in 0..4 {
    instance core$i = cpu_complex(id = i, clock = master / (i + 1))
  }

  for j in 0..=1 {
    object bank${j * 2} "ram" { size = ram / 2 }
  }
}
"#;

fn dump(name: &str, text: &str) -> String {
    let src = SourceFile::new(name, text);
    parse(&src)
        .unwrap_or_else(|d| panic!("{}", d.render(&src)))
        .dump()
}

fn render_error(text: &str) -> String {
    let src = SourceFile::new("nes.machine", text);
    parse(&src).expect_err("should fail").render(&src)
}

#[test]
fn the_nes_example_parses_into_the_expected_tree() {
    assert_eq!(
        dump("nes.machine", NES),
        r#"machine "nes" {
  param region = "ntsc"
  osc master = (236250000 / 11) Hz
  space cpubus { width = 16, unassigned = open-bus }
  space ppubus { width = 14, unassigned = open-bus }
  object ram "wram" { size = 2048 }
  object cpu "mos6502" { clock = (master / 12), space = cpubus, engine = "interp" }
  object ppu "nes.ppu" { clock = (master / 4), space = ppubus }
  object apu "nes.apu" { clock = (master / 12) }
  map cpubus 0 size 8192 = mirror(wram)
  map cpubus 8192 size 8192 = mirror(ppu.regs)
  map cpubus 16384 size 32 = apu.regs
  wire ppu.nmi -> cpu.nmi
  wire apu.irq -> cpu.irq
  wire cart.irq -> cpu.irq
}
"#
    );
}

#[test]
fn the_nes_master_clock_is_an_exact_rational() {
    let unit = parse_file("nes.machine", NES).expect("parses");
    let Stmt::Machine(machine) = &unit.stmts[0] else {
        panic!("expected a machine block");
    };
    assert_eq!(machine.name.node, "nes");

    let Stmt::Osc(osc) = &machine.body[1] else {
        panic!("expected the oscillator");
    };
    assert_eq!(osc.name.as_literal(), Some("master"));
    let hz = osc.frequency_hz().expect("a literal frequency");
    assert_eq!(hz.numerator(), 236_250_000);
    assert_eq!(hz.denominator(), 11);
    // 21477272.72…: exactly what §5 says it is, and not an integer.
    assert!(!hz.is_integer());

    // The point of keeping it rational: the CPU:PPU ratio stays exactly 3.
    let cpu = hz
        .checked_div(crate::machine::Rational::from_int(12))
        .expect("in range");
    let ppu = hz
        .checked_div(crate::machine::Rational::from_int(4))
        .expect("in range");
    assert_eq!(ppu.checked_div(cpu).and_then(|r| r.to_integer()), Some(3));
}

#[test]
fn the_nes_examples_statements_are_the_graph_it_describes() {
    let unit = parse_file("nes.machine", NES).expect("parses");
    let Stmt::Machine(machine) = &unit.stmts[0] else {
        panic!("expected a machine block");
    };

    let objects: Vec<&str> = machine
        .body
        .iter()
        .filter_map(|s| match s {
            Stmt::Object(o) => o.name.as_literal(),
            _ => None,
        })
        .collect();
    assert_eq!(objects, ["ram", "cpu", "ppu", "apu"]);

    let maps: Vec<(u64, u64)> = machine
        .body
        .iter()
        .filter_map(|s| match s {
            Stmt::Map(m) => match (&m.base, &m.size) {
                (Expr::Num(base), Expr::Num(size)) => Some((base.node.value, size.node.value)),
                _ => None,
            },
            _ => None,
        })
        .collect();
    assert_eq!(maps, [(0x0000, 0x2000), (0x2000, 0x2000), (0x4000, 0x20)]);

    // Three sources, one destination: the wired-OR §5 mentions is expressible
    // without the parser knowing what a wired-OR is.
    let wires: Vec<(String, String)> = machine
        .body
        .iter()
        .filter_map(|s| match s {
            Stmt::Wire(w) => Some((
                w.from.as_literal().unwrap_or_default(),
                w.to.as_literal().unwrap_or_default(),
            )),
            _ => None,
        })
        .collect();
    assert_eq!(
        wires,
        [
            ("ppu.nmi".to_string(), "cpu.nmi".to_string()),
            ("apu.irq".to_string(), "cpu.irq".to_string()),
            ("cart.irq".to_string(), "cpu.irq".to_string()),
        ]
    );
}

#[test]
fn include_template_and_indexed_instantiation_parse() {
    assert_eq!(
        dump("quad.machine", TEMPLATED),
        r#"include "pci-common.machine"
param cores = 4
param ram = 4194304
template cpu_complex(id, clock, l2 = 524288) {
  object cpu$id "riscv64" { clock = clock, space = mem$id }
  object l2$id "cache" { size = l2 }
  wire cpu$id.irq -> plic.in$id
}
machine "quad" {
  osc master = 1 GHz
  for i in 0..4 {
    instance core$i = cpu_complex(id = i, clock = (master / (i + 1)))
  }
  for j in 0..=1 {
    object bank${(j * 2)} "ram" { size = (ram / 2) }
  }
}
"#
    );
}

#[test]
fn parsing_is_deterministic() {
    // No hashing anywhere in the front end, so two runs are byte-identical.
    assert_eq!(dump("nes.machine", NES), dump("nes.machine", NES));
    assert_eq!(dump("q.machine", TEMPLATED), dump("q.machine", TEMPLATED));
}

// ---- golden diagnostics --------------------------------------------------
//
// These four are asserted exactly. They are the messages a first-time user
// sees, so a change here is a decision, not an accident.

#[test]
fn golden_missing_brace() {
    assert_eq!(
        render_error("machine \"nes\" {\n  object ram \"wram\"\n"),
        "\
error: expected `}`, found end of file
 --> nes.machine:3:1
  |
3 |
  | ^

note: this `{` is never closed
 --> nes.machine:1:15
  |
1 | machine \"nes\" {
  |               ^"
    );
}

#[test]
fn golden_unknown_keyword() {
    assert_eq!(
        render_error("machine \"nes\" {\n  objekt ram \"wram\" { size = 2K }\n}\n"),
        "\
error: unknown statement `objekt`; expected one of `machine`, `param`, `osc`, `space`, `object`, `map`, `wire`, `include`, `template`, `instance`, `for`
 --> nes.machine:2:3
  |
2 |   objekt ram \"wram\" { size = 2K }
  |   ^^^^^^"
    );
}

#[test]
fn golden_bad_number() {
    assert_eq!(
        render_error("machine \"nes\" {\n  object ram \"wram\" { size = 2Kb2 }\n}\n"),
        "\
error: unknown suffix `Kb2`; expected a size (`K`, `M`, `G`, `T`) or a duration (`ns`, `us`, `ms`, `s`)
 --> nes.machine:2:31
  |
2 |   object ram \"wram\" { size = 2Kb2 }
  |                               ^^^"
    );
}

#[test]
fn golden_unterminated_string() {
    assert_eq!(
        render_error("machine \"nes\" {\n  object ram \"wram { size = 2K }\n}\n"),
        "\
error: unterminated string literal
 --> nes.machine:2:14
  |
2 |   object ram \"wram { size = 2K }
  |              ^"
    );
}

#[test]
fn golden_error_through_the_crate_error_type() {
    // What `rsemu run nes.machine` prints: location, message, caret, once.
    let err = parse_file(
        "nes.machine",
        "machine \"nes\" {\n  osc master = 1 Hertz\n}\n",
    )
    .expect_err("should fail");
    assert_eq!(
        err.to_string(),
        "\
nes.machine:2:18: expected a frequency unit (`Hz`, `kHz`, `MHz` or `GHz`), found `Hertz`
  |
2 |   osc master = 1 Hertz
  |                  ^^^^^"
    );
    let Error::Config { at, .. } = &err else {
        panic!("expected a config error");
    };
    assert_eq!(at, "nes.machine:2:18");
}

// ---- robustness ----------------------------------------------------------

#[test]
fn every_truncation_of_a_valid_file_is_an_error_not_a_panic() {
    for source in [NES, TEMPLATED] {
        for cut in 0..=source.len() {
            if !source.is_char_boundary(cut) {
                continue;
            }
            let src = SourceFile::new("t.machine", &source[..cut]);
            // The result is irrelevant; not panicking is the assertion. A
            // prefix that happens to be a whole file parses fine.
            if let Err(d) = parse(&src) {
                // Rendering must survive every span the parser can produce,
                // including one at end of file.
                assert!(!d.render(&src).is_empty());
            }
        }
    }
}

#[test]
fn generated_garbage_never_panics() {
    // A deterministic LCG rather than a random source: the dependency budget
    // is zero, and a fuzz failure nobody can reproduce is not a finding. The
    // real fuzz target (§13) is a separate, longer-running job.
    const ALPHABET: &[u8] = b"machine{}\"#$-0123456789xKHz/*+.,=>()[] \n\t_objectmapwirefor..=";
    let mut state: u64 = 0x2545_f491_4f6c_dd1d;
    let mut next = move || {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (state >> 33) as usize
    };

    for case in 0..2000 {
        let len = case % 97;
        let mut text = String::with_capacity(len);
        for _ in 0..len {
            let byte = ALPHABET[next() % ALPHABET.len()];
            text.push(byte as char);
        }
        let src = SourceFile::new("fuzz.machine", &text);
        if let Err(d) = parse(&src) {
            assert!(!d.render(&src).is_empty());
        }
    }
}

#[test]
fn adversarial_shapes_are_refused_cleanly() {
    for text in [
        "",
        "\u{0}",
        "\u{feff}machine \"a\" {}",
        "machine",
        "machine \"",
        "machine \"a\" { osc x = 1/0 Hz }",
        "param x = 0x",
        "param x = 99999999999999999999999999",
        "param x = 1 % 0",
        "for i in 0..0 {}",
        "wire . -> .",
        "object $ \"c\"",
        "map m 0 size 0 = ",
        "include",
        "template t(",
        "instance a = ",
        "space s { a = { b = { c = 1 } } }",
        "#",
        "$",
        "..",
        "->",
    ] {
        let src = SourceFile::new("t.machine", text);
        if let Err(d) = parse(&src) {
            let rendered = d.render(&src);
            assert!(rendered.starts_with("error: "), "{rendered}");
            assert!(rendered.contains("t.machine:"), "{rendered}");
        }
    }
}

#[test]
fn a_deeply_nested_tree_is_refused_rather_than_overflowing_the_stack() {
    // Both nesting axes, at a depth that would certainly overflow if the guard
    // were missing. The cap also bounds the depth of the tree that gets
    // dropped, which is the second way a parser like this blows the stack.
    let mut text = String::from("param x = ");
    for _ in 0..100_000 {
        text.push('(');
    }
    let src = SourceFile::new("t.machine", &text);
    assert!(parse(&src).is_err());

    let mut blocks = String::new();
    for _ in 0..100_000 {
        blocks.push_str("template t {");
    }
    let src = SourceFile::new("t.machine", &blocks);
    assert!(parse(&src).is_err());

    let mut lists = String::from("param x = ");
    for _ in 0..100_000 {
        lists.push('[');
    }
    let src = SourceFile::new("t.machine", &lists);
    assert!(parse(&src).is_err());
}
