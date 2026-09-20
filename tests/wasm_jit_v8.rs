//! The generated modules, handed to a real WebAssembly engine.
//!
//! `src/jit/wasm/exec.rs` is a wasm interpreter written from the *WebAssembly
//! Core Specification*, and it is the only thing that executes these modules
//! in-crate — so on its own it checks the code generator against **this
//! project's reading** of §4 and §5 and nothing more. A backend whose encoder
//! and whose executor were wrong in the same direction would pass every test
//! in `src/jit/wasm/tests.rs`.
//!
//! This closes that gap where a real engine is available. `node` ships V8,
//! which implements the same specification and did not read our code:
//!
//! * **Every module the backend produces is validated.**
//!   `WebAssembly.validate` is the whole of §3, and it is exactly the check
//!   that catches a malformed LEB128, a label depth past every enclosing
//!   block, a `local.get` past the declared count, a type index that names no
//!   type, a section out of order and a stack that does not balance. None of
//!   those is something `exec` looks for — it is a producer's decoder, not a
//!   validator.
//! * **One module is instantiated and run**, against JS stub imports, and its
//!   result and its frame are compared byte for byte against the reference
//!   executor over the same stubs. That is an agreement check between V8 and
//!   `exec`, so a disagreement is a finding about one of them either way.
//!
//! # What it needs, and what it does when it is not there
//!
//! `node` on `PATH`, and nothing else — no package, no network, no file
//! written outside the temporary directory (the script goes in on stdin). When
//! there is no `node` the test **skips**, loudly, because a conformance check
//! that silently became a no-op is worse than no conformance check. CI's
//! `wasm` job already has node for `web/check.mjs`, so this runs there.

#![cfg(all(feature = "jit-wasm", feature = "std"))]

use std::io::Write;
use std::process::{Command, Stdio};

use rsemu::ir::{Block, BlockBuilder, Cond, Const, InsnStart, MemOp, Opcode, RegSlot, Temp, Type};
use rsemu::jit::wasm::exec;

/// The `node` binary, or `None` if this machine has none.
fn node() -> Option<String> {
    let name = std::env::var("RSEMU_NODE").unwrap_or_else(|_| String::from("node"));
    let ok = Command::new(&name)
        .arg("--version")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .is_ok_and(|s| s.success());
    ok.then_some(name)
}

fn skip(what: &str) {
    println!("SKIP {what}: no `node` on PATH (set RSEMU_NODE to name one)");
}

/// Run `script` under node, returning its standard output.
fn run_node(node: &str, script: &str) -> String {
    let mut child = Command::new(node)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("node starts");
    child
        .stdin
        .take()
        .expect("a piped stdin")
        .write_all(script.as_bytes())
        .expect("the script is written");
    let out = child.wait_with_output().expect("node finishes");
    assert!(
        out.status.success(),
        "node failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8_lossy(&out.stdout).into_owned()
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

// ---------------------------------------------------------------------------
// The corpus
// ---------------------------------------------------------------------------

fn at(pc: u64, live: &[(u16, Temp)]) -> InsnStart {
    InsnStart {
        pc,
        next_pc: pc + 4,
        ticks: 0,
        live: live.iter().map(|&(s, t)| (RegSlot(s), t)).collect(),
    }
}

/// Blocks covering every lowering the backend has, so validation sees every
/// instruction sequence it can emit rather than a sample.
fn corpus() -> Vec<(&'static str, Block)> {
    let mut out: Vec<(&'static str, Block)> = Vec::new();

    // Arithmetic and logic, at both widths.
    for (name, ty) in [("arith32", Type::I32), ("arith64", Type::I64)] {
        let mut b = BlockBuilder::new(0x100, 0);
        b.insn_start(at(0x100, &[]));
        let x = b.imm(ty, Const::Int(0x0123_4567));
        let y = b.imm(ty, Const::Int(0x89ab_cdef));
        let results = [
            b.binary(Opcode::ADD, ty, x, y),
            b.binary(Opcode::SUB, ty, x, y),
            b.binary(Opcode::MUL, ty, x, y),
            b.unary(Opcode::NEG, ty, x),
            b.binary(Opcode::AND, ty, x, y),
            b.binary(Opcode::OR, ty, x, y),
            b.binary(Opcode::XOR, ty, x, y),
            b.unary(Opcode::NOT, ty, x),
            b.binary(Opcode::ANDC, ty, x, y),
            b.unary(Opcode::CLZ, ty, x),
            b.unary(Opcode::CTZ, ty, x),
            b.unary(Opcode::POPCOUNT, ty, x),
            b.binary(Opcode::SHL, ty, x, y),
            b.binary(Opcode::SHR, ty, x, y),
            b.binary(Opcode::SAR, ty, x, y),
            b.binary(Opcode::ROTL, ty, x, y),
            b.binary(Opcode::ROTR, ty, x, y),
        ];
        let live: Vec<(u16, Temp)> = results
            .iter()
            .enumerate()
            .map(|(i, &t)| (i as u16 % 8, t))
            .collect();
        b.insn_start(at(0x104, &live));
        b.exit_tb();
        out.push((name, b.finish()));
    }

    // Every condition, in both shapes that consume one.
    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, &[]));
    let x = b.imm(Type::I64, Const::Int(7));
    let y = b.imm(Type::I64, Const::Int(9));
    let mut live = Vec::new();
    for (i, cond) in [
        Cond::Eq,
        Cond::Ne,
        Cond::LtS,
        Cond::LeS,
        Cond::GtS,
        Cond::GeS,
        Cond::LtU,
        Cond::LeU,
        Cond::GtU,
        Cond::GeU,
    ]
    .into_iter()
    .enumerate()
    {
        let set = b.setcond(cond, Type::I64, x, y);
        let dst = b.temp(Type::I64);
        b.emit_raw(
            Opcode::MOVCOND,
            Type::I64,
            Some(dst),
            None,
            &[set, x, y],
            None,
            None,
            0,
        );
        live.push((i as u16 % 8, dst));
    }
    b.insn_start(at(0x104, &live));
    b.exit_tb();
    out.push(("conditions", b.finish()));

    // Bitfields and byte swaps.
    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, &[]));
    let wide = b.imm(Type::I64, Const::Int(0x89ab_cdef_0123_4567));
    let narrow = b.unary(Opcode::TRUNC, Type::I32, wide);
    let swapped = b.temp(Type::I64);
    b.emit_raw(
        Opcode::BSWAP,
        Type::I64,
        Some(swapped),
        None,
        &[wide],
        Some(Const::Int(16)),
        None,
        0,
    );
    let field = b.temp(Type::I64);
    b.emit_raw(
        Opcode::EXTRACT,
        Type::I64,
        Some(field),
        None,
        &[wide],
        None,
        None,
        rsemu::ir::bitfield_aux(12, 20),
    );
    let put = b.temp(Type::I64);
    b.emit_raw(
        Opcode::DEPOSIT,
        Type::I64,
        Some(put),
        None,
        &[wide, narrow],
        None,
        None,
        rsemu::ir::bitfield_aux(8, 16),
    );
    let ext = b.unary(Opcode::EXT_S, Type::I64, narrow);
    b.insn_start(at(
        0x104,
        &[(0, swapped), (1, field), (2, put), (3, ext), (4, narrow)],
    ));
    b.exit_tb();
    out.push(("bitfields", b.finish()));

    // The import-calling ops: a slot read, a load, a store, a charge — and two
    // boundaries, so both note kinds appear.
    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, &[]));
    let slot = b.get_slot(Type::I64, RegSlot(3));
    b.charge(5);
    b.insn_start(at(0x104, &[(3, slot)]));
    let addr = b.imm(Type::I64, Const::Int(0x2000));
    let v = b.load(Type::I64, addr, MemOp::load(rsemu::core::value::Width::U32));
    b.store(
        Type::I64,
        addr,
        v,
        MemOp::store(rsemu::core::value::Width::U16),
    );
    b.insn_start(at(0x108, &[(3, slot), (4, v)]));
    b.exit_tb();
    out.push(("memory", b.finish()));

    // Two forward branch targets, which is the nesting a superblock's side
    // exits produce and the one thing a validator is most likely to reject.
    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, &[]));
    let s1 = b.imm(Type::I1, Const::Int(1));
    let s2 = b.imm(Type::I1, Const::Int(0));
    let one = b.imm(Type::I64, Const::Int(1));
    let mut acc = b.imm(Type::I64, Const::Int(0));
    let far = b.emit_raw(Opcode::BRCOND, Type::I1, None, None, &[s2], None, None, 0);
    let near = b.emit_raw(Opcode::BRCOND, Type::I1, None, None, &[s1], None, None, 0);
    acc = b.binary(Opcode::ADD, Type::I64, acc, one);
    let near_target = b.next_index();
    acc = b.binary(Opcode::ADD, Type::I64, acc, one);
    let far_target = b.next_index();
    acc = b.binary(Opcode::ADD, Type::I64, acc, one);
    b.patch_aux(near, near_target as u32);
    b.patch_aux(far, far_target as u32);
    b.insn_start(at(0x104, &[(0, acc)]));
    b.exit_tb();
    out.push(("branches", b.finish()));

    // The two terminators that carry a successor.
    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, &[]));
    b.emit_raw(
        Opcode::GOTO_TB,
        Type::I64,
        None,
        None,
        &[],
        Some(Const::Int(0x2000)),
        None,
        0,
    );
    out.push(("goto", b.finish()));

    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, &[]));
    let pc = b.imm(Type::I64, Const::Int(0x3000));
    b.emit_raw(
        Opcode::LOOKUP_AND_GOTO,
        Type::I64,
        None,
        None,
        &[pc],
        None,
        None,
        0,
    );
    out.push(("lookup", b.finish()));

    out
}

// ---------------------------------------------------------------------------

#[test]
fn every_generated_module_is_valid_webassembly() {
    let Some(node) = node() else {
        return skip("every_generated_module_is_valid_webassembly");
    };
    let mut script = String::from(
        "const cases = [];\n\
         const add = (name, hex) => cases.push([name, \
           Uint8Array.from(hex.match(/../g).map(b => parseInt(b, 16)))]);\n",
    );
    let corpus = corpus();
    for (name, block) in &corpus {
        let compiled = rsemu::jit::wasm::compile(block)
            .unwrap_or_else(|e| panic!("`{name}` is built from the compiled set: {e}"));
        script.push_str(&format!("add({name:?}, {:?});\n", hex(compiled.module())));
    }
    script.push_str(
        "let bad = 0;\n\
         for (const [name, bytes] of cases) {\n\
         \x20 if (!WebAssembly.validate(bytes)) { console.log('INVALID ' + name); bad++; continue; }\n\
         \x20 try { new WebAssembly.Module(bytes); } catch (e) {\n\
         \x20   console.log('REFUSED ' + name + ': ' + e.message); bad++; continue; }\n\
         \x20 console.log('ok ' + name);\n\
         }\n\
         console.log('checked ' + cases.length + ', bad ' + bad);\n",
    );
    let out = run_node(&node, &script);
    assert!(
        out.contains(&format!("checked {}, bad 0", corpus.len())),
        "V8 rejected a module this backend emitted:\n{out}"
    );
}

#[test]
fn v8_and_the_reference_executor_agree_about_a_module_they_both_run() {
    let Some(node) = node() else {
        return skip("v8_and_the_reference_executor_agree_about_a_module_they_both_run");
    };
    // The arithmetic block: every import it calls is a boundary note, which a
    // stub answers with zero on both sides, so the two engines are executing
    // the same program against the same environment and the frame is the whole
    // observable result.
    let corpus = corpus();
    let (_, block) = corpus
        .iter()
        .find(|(n, _)| *n == "arith64")
        .expect("the corpus has it");
    let compiled = rsemu::jit::wasm::compile(block).expect("compiles");
    let program = exec::parse(compiled.module()).expect("decodes");

    struct Stub;
    impl exec::Env for Stub {
        fn call(&mut self, _func: u32, _args: &[i64], _mem: &mut [u8]) -> i64 {
            0
        }
    }
    let frame = compiled.frame_bytes();
    let mut mem = vec![0u8; 65536];
    let want_status = exec::run(&program, &[0, 0], &mut mem, &mut Stub).expect("runs");
    let want_frame = hex(&mem[..frame]);

    let script = format!(
        "const bytes = Uint8Array.from({:?}.match(/../g).map(b => parseInt(b, 16)));\n\
         const memory = new WebAssembly.Memory({{ initial: 1 }});\n\
         const zero = () => 0;\n\
         const zero64 = () => 0n;\n\
         const i = new WebAssembly.Instance(new WebAssembly.Module(bytes), {{\n\
         \x20 e: {{ m: memory, g: zero64, l: zero, s: zero, n: zero }} }});\n\
         const status = i.exports.b(0, 0);\n\
         const view = new Uint8Array(memory.buffer, 0, {frame});\n\
         const hex = Array.from(view).map(b => b.toString(16).padStart(2, '0')).join('');\n\
         console.log('status ' + status);\n\
         console.log('frame ' + hex);\n",
        hex(compiled.module())
    );
    let out = run_node(&node, &script);
    assert!(
        out.contains(&format!("status {want_status}")),
        "V8 and the reference executor disagree about the status.\n\
         reference: {want_status}\nnode said:\n{out}"
    );
    assert!(
        out.contains(&format!("frame {want_frame}")),
        "V8 and the reference executor disagree about the frame.\n\
         reference: {want_frame}\nnode said:\n{out}"
    );
}
