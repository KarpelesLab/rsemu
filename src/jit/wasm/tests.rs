//! The backend's own differential: a generated module against `ir::Interp`.
//!
//! CLAUDE.md makes a guest's interpreter the oracle for its frontend; one
//! level down, [`Interp`] is the oracle for every host backend. This is that
//! differential **at the IR**, and unlike [`jit::arm64`](crate::jit::arm64)'s
//! it runs everywhere: blocks over the compiled opcode set, executed twice
//! against two identical hosts, compared on every observable temporary, every
//! guest slot, the tick count, guest memory, the boundary count and the
//! outcome.
//!
//! "Every observable temporary" is narrower here than on a native backend and
//! the difference is real: a temporary no [`InsnStart`] names lives in a wasm
//! local, which stops existing when the function returns, so the comparison is
//! over the write-through set. That is not a gap in the checking — it is the
//! set the *guest* can observe, because publishing is what makes a temporary
//! reach architectural state — but it is worth knowing which set is compared,
//! and it is why nearly every block below publishes what it computed.

use alloc::collections::BTreeMap;
use alloc::string::{String, ToString};
use alloc::vec::Vec;

use crate::core::error::BusError;
use crate::core::space::MemResult;
use crate::core::value::Width;
use crate::ir::{
    Block, BlockBuilder, Cond, Const, InsnStart, Interp, IrHost, MemOp, Opcode, Outcome, RegSlot,
    Sign, Temp, Type, bitfield_aux, verify,
};
use crate::jit::CodeRef;

use super::abi::{frame_bytes, temp_offset};
use super::compile::{Refusal, compile, compiles};
use super::exec;
use super::rt::Engine;

/// How many guest state slots the generator uses.
const SLOTS: u16 = 8;
/// Where the test machine's RAM starts.
const BASE: u64 = 0x1000;
/// How much of it there is.
const RAM: u64 = 512;

// ---------------------------------------------------------------------------
// The host both engines run against
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
struct Host {
    slots: BTreeMap<u16, u128>,
    ram: BTreeMap<u64, u8>,
    ticks: u64,
    boundaries: Vec<u64>,
    /// The tick count past which [`IrHost::spent`] answers `true`.
    allowance: u64,
    loads: u64,
    stores: u64,
}

impl Host {
    fn new() -> Host {
        let mut h = Host {
            slots: BTreeMap::new(),
            ram: BTreeMap::new(),
            ticks: 0,
            boundaries: Vec::new(),
            allowance: u64::MAX,
            loads: 0,
            stores: 0,
        };
        for i in 0..RAM {
            h.ram
                .insert(BASE + i, (i as u8).wrapping_mul(31).wrapping_add(7));
        }
        for s in 0..SLOTS {
            h.slots
                .insert(s, (u128::from(s) * 0x0123_4567_89ab_cdef) ^ 0x5a5a_5a5a);
        }
        h
    }
}

impl IrHost for Host {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        self.slots.get(&slot.0).copied().unwrap_or(0)
    }

    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        self.slots.insert(slot.0, value);
    }

    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        self.loads += 1;
        // A per-access tick, so the dynamic column `InsnStart::ticks` does not
        // carry is non-zero and the two engines have to agree on it too.
        self.ticks += 1;
        let bytes = mem.size.bytes();
        if addr < BASE || addr + bytes > BASE + RAM {
            return Err(BusError::Unassigned);
        }
        let mut out = 0u64;
        for i in 0..bytes {
            out |= u64::from(self.ram.get(&(addr + i)).copied().unwrap_or(0)) << (8 * i);
        }
        Ok(out)
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        self.stores += 1;
        self.ticks += 1;
        let bytes = mem.size.bytes();
        if addr < BASE || addr + bytes > BASE + RAM {
            return Err(BusError::Unassigned);
        }
        for i in 0..bytes {
            self.ram.insert(addr + i, (value >> (8 * i)) as u8);
        }
        Ok(())
    }

    fn charge(&mut self, ticks: u64) {
        self.ticks += ticks;
    }

    fn insn_start(&mut self, mark: &InsnStart) {
        self.boundaries.push(mark.pc);
    }

    fn spent(&self) -> bool {
        self.ticks >= self.allowance
    }
}

/// Everything a run of one block can be compared on.
#[derive(Debug, PartialEq, Eq)]
struct Columns {
    outcome: Option<Outcome>,
    error: Option<String>,
    slots: BTreeMap<u16, u128>,
    ram: BTreeMap<u64, u8>,
    ticks: u64,
    boundaries: Vec<u64>,
    loads: u64,
    stores: u64,
}

fn columns(out: &crate::core::error::Result<Outcome>, host: Host) -> Columns {
    Columns {
        outcome: out.as_ref().ok().cloned(),
        error: out.as_ref().err().map(ToString::to_string),
        slots: host.slots,
        ram: host.ram,
        ticks: host.ticks,
        boundaries: host.boundaries,
        loads: host.loads,
        stores: host.stores,
    }
}

fn interpreted(block: &Block, mut host: Host) -> (Columns, Interp) {
    let mut interp = Interp::new();
    let out = interp.run(block, &mut host);
    (columns(&out, host), interp)
}

fn compiled(block: &Block, mut host: Host) -> (Columns, Engine, CodeRef) {
    let mut engine = Engine::with_capacity(4);
    let code = engine
        .compile(block)
        .unwrap_or_else(|e| panic!("the backend refused a block it says it compiles: {e}"));
    let out = engine
        .run(block, code, &mut host)
        .expect("a module just compiled is live");
    (columns(&out, host), engine, code)
}

/// Run `block` both ways and assert every column agrees.
fn agree(block: &Block) {
    verify(block).unwrap_or_else(|e| panic!("the test built a block verify rejects: {e}"));
    let host = Host::new();
    let (want, interp) = interpreted(block, host.clone());
    let (got, engine, code) = compiled(block, host);
    assert_eq!(want.outcome, got.outcome, "outcome");
    assert_eq!(want.error, got.error, "error");
    assert_eq!(want.ticks, got.ticks, "ticks");
    assert_eq!(want.boundaries, got.boundaries, "boundaries");
    assert_eq!(want.slots, got.slots, "guest slots");
    assert_eq!(want.ram, got.ram, "guest memory");
    assert_eq!(want.loads, got.loads, "loads");
    assert_eq!(want.stores, got.stores, "stores");
    assert_eq!(
        interp.ticks(),
        engine.ticks(),
        "the engines' own tick column"
    );
    assert_eq!(interp.boundaries(), engine.boundaries(), "boundary count");
    assert_eq!(interp.mark(), engine.mark(), "the boundary reached");
    for i in 0..block.temp_count() as u32 {
        let t = Temp(i);
        if let Some(got) = engine.temp_value(code, t) {
            let want = interp
                .temp_value(t)
                .expect("the interpreter holds every temporary");
            assert_eq!(
                u128::from(got),
                want,
                "t{i} differs: interpreted {want:#x}, compiled {got:#x}"
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Building blocks, over the raw builder
// ---------------------------------------------------------------------------

fn at(pc: u64, ticks: u64, live: &[(u16, Temp)]) -> InsnStart {
    InsnStart {
        pc,
        next_pc: pc + 4,
        ticks,
        live: live.iter().map(|&(s, t)| (RegSlot(s), t)).collect(),
    }
}

/// A builder standing at one open boundary.
fn started() -> BlockBuilder {
    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, 0, &[]));
    b
}

/// Close a block with an exit boundary publishing `live`, and an `exit_tb`.
fn wrap(mut b: BlockBuilder, live: &[(u16, Temp)]) -> Block {
    b.insn_start(at(0x104, 0, live));
    b.exit_tb();
    b.finish()
}

fn bswap(b: &mut BlockBuilder, ty: Type, a: Temp, lane: Option<u32>) -> Temp {
    let dst = b.temp(ty);
    b.emit_raw(
        Opcode::BSWAP,
        ty,
        Some(dst),
        None,
        &[a],
        lane.map(|l| Const::Int(u128::from(l))),
        None,
        0,
    );
    dst
}

fn bitfield(b: &mut BlockBuilder, op: Opcode, ty: Type, srcs: &[Temp], pos: u32, len: u32) -> Temp {
    let dst = b.temp(ty);
    b.emit_raw(
        op,
        ty,
        Some(dst),
        None,
        srcs,
        None,
        None,
        bitfield_aux(pos, len),
    );
    dst
}

fn movcond_sel(b: &mut BlockBuilder, ty: Type, sel: Temp, t: Temp, f: Temp) -> Temp {
    let dst = b.temp(ty);
    b.emit_raw(
        Opcode::MOVCOND,
        ty,
        Some(dst),
        None,
        &[sel, t, f],
        None,
        None,
        0,
    );
    dst
}

/// A forward `brcond` on a one-bit selector, yielding its instruction index so
/// the target can be patched once it exists.
fn brcond_sel(b: &mut BlockBuilder, sel: Temp) -> usize {
    b.emit_raw(Opcode::BRCOND, Type::I1, None, None, &[sel], None, None, 0)
}

fn goto_tb(b: &mut BlockBuilder, pc: u64) {
    b.emit_raw(
        Opcode::GOTO_TB,
        Type::I64,
        None,
        None,
        &[],
        Some(Const::Int(u128::from(pc))),
        None,
        0,
    );
}

fn lookup_and_goto(b: &mut BlockBuilder, pc: Temp) {
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
}

// ---------------------------------------------------------------------------
// Encoding, without executing
// ---------------------------------------------------------------------------

#[test]
fn every_emitted_module_decodes_and_declares_what_it_imports() {
    let mut b = started();
    let a = b.imm(Type::I64, Const::Int(0x1234_5678_9abc_def0));
    let c = b.binary(Opcode::ADD, Type::I64, a, a);
    let out = wrap(b, &[(0, c)]);
    let m = compile(&out).expect("a two-op block compiles");
    let p = exec::parse(m.module()).expect("the encoder and the decoder agree");
    // One local per temporary, plus two scratch `i64` and one scratch `i32`.
    assert_eq!(p.local_count(), out.temp_count() + 3);
    // The header is the specification's, byte for byte (§5.5.16).
    assert_eq!(&m.module()[..8], &super::emit::HEADER);
    // The frame layout the ABI documents is the one the compiler sizes.
    assert_eq!(temp_offset(0), 8);
    assert_eq!(m.frame_bytes(), frame_bytes(out.temp_count()));
}

#[test]
fn a_refusal_names_what_stopped_it() {
    // An op no lowering exists for.
    let mut b = started();
    let a = b.imm(Type::I64, Const::Int(1));
    let _ = b.binary(Opcode::DIV_S, Type::I64, a, a);
    let out = wrap(b, &[]);
    assert_eq!(compile(&out).err(), Some(Refusal::Op(Opcode::DIV_S)));

    // A type no wasm local holds.
    let mut b = started();
    let a = b.imm(Type::I128, Const::Int(1));
    let _ = b.binary(Opcode::ADD, Type::I128, a, a);
    let out = wrap(b, &[]);
    assert_eq!(compile(&out).err(), Some(Refusal::Type(Type::I128)));

    // And the message says which, because a refusal with no reason is how a
    // backend's coverage silently rots.
    assert!(
        alloc::format!("{}", Refusal::Op(Opcode::PHI)).contains("phi"),
        "a refusal must name the op"
    );
}

#[test]
fn the_compiled_set_is_what_the_module_docs_say_it_is() {
    for op in [
        Opcode::DIV_S,
        Opcode::DIV_U,
        Opcode::REM_S,
        Opcode::REM_U,
        Opcode::ADDC,
        Opcode::SUBB,
        Opcode::MULU2,
        Opcode::MULS2,
        Opcode::MULHSU,
        Opcode::ROTLC,
        Opcode::ROTRC,
        Opcode::FENCE,
        Opcode::CMPXCHG,
        Opcode::XCHG,
        Opcode::FETCH_ADD,
        Opcode::LD_EXCL,
        Opcode::ST_EXCL,
        Opcode::CALL_HELPER,
        Opcode::PHI,
    ] {
        assert!(
            !compiles(op),
            "{op} is refused in the docs and lowered here"
        );
    }
    for op in [
        Opcode::ADD,
        Opcode::LD,
        Opcode::ST,
        Opcode::INSN_START,
        Opcode::LOOKUP_AND_GOTO,
    ] {
        assert!(compiles(op), "{op} is in the list and must lower");
    }
}

#[test]
fn a_backward_branch_is_refused_rather_than_mis_nested() {
    // wasm's structured control flow cannot express one, and a backend that
    // silently emitted a forward `br` for it would be a miscompile rather than
    // a missing feature.
    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, 0, &[]));
    let one = b.imm(Type::I1, Const::Int(1));
    let branch = brcond_sel(&mut b, one);
    b.patch_aux(branch, 0);
    b.exit_tb();
    let out = b.finish();
    assert!(matches!(compile(&out), Err(Refusal::Shape(_))));
}

#[test]
fn a_frame_write_through_happens_only_for_a_temporary_a_boundary_names() {
    let mut b = started();
    let hidden = b.imm(Type::I64, Const::Int(1));
    let named = b.binary(Opcode::ADD, Type::I64, hidden, hidden);
    let out = wrap(b, &[(3, named)]);
    let m = compile(&out).expect("compiles");
    assert!(
        m.written_through(named),
        "a published temporary is readable"
    );
    assert!(
        !m.written_through(hidden),
        "a temporary nothing publishes stays in a wasm local"
    );
}

// ---------------------------------------------------------------------------
// Executed: agreement with the oracle
// ---------------------------------------------------------------------------

#[test]
fn arithmetic_and_logic_agree_with_the_oracle_at_every_width() {
    for ty in [Type::I32, Type::I64] {
        for (a, c) in [
            (0x0123_4567_89ab_cdefu64, 0xfedc_ba98_7654_3210u64),
            (0, 1),
            (u64::MAX, 1),
            (0x8000_0000, 0x8000_0000),
        ] {
            let mut b = started();
            let x = b.imm(ty, Const::Int(u128::from(a)));
            let y = b.imm(ty, Const::Int(u128::from(c)));
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
            ];
            // Eight slots and twelve results, so they share; the last write to
            // a slot wins, identically in both engines.
            let live: Vec<(u16, Temp)> = results
                .iter()
                .enumerate()
                .map(|(i, &t)| (i as u16 % SLOTS, t))
                .collect();
            agree(&wrap(b, &live));
        }
    }
}

#[test]
fn a_bit_count_is_within_the_type_and_not_the_host_word() {
    // The zero input is the case a `ctz` lowered as a bare `i64.ctz` gets
    // wrong: wasm answers 64 and the IR answers the type's width.
    for ty in [Type::I1, Type::I32, Type::I64] {
        for v in [0u128, 1, 0x8000_0000] {
            if v >= 1u128 << ty.bits() {
                continue;
            }
            let mut b = started();
            let x = b.imm(ty, Const::Int(v));
            let live = [
                (0u16, b.unary(Opcode::CLZ, ty, x)),
                (1, b.unary(Opcode::CTZ, ty, x)),
                (2, b.unary(Opcode::POPCOUNT, ty, x)),
            ];
            agree(&wrap(b, &live));
        }
    }
}

#[test]
fn a_shift_out_of_range_takes_the_interpreters_answer_and_not_wasms() {
    // wasm reduces a shift count modulo 64 (core specification §4.4.1);
    // `Interp` takes the mathematical answer. Both are legal readings of the
    // IR's "undefined", and they are different numbers — so a block that ran
    // compiled once and interpreted once would diverge if this were left to
    // the host. 64 and 70 are the two that catch it.
    for ty in [Type::I32, Type::I64] {
        for amount in [0u64, 1, 7, 31, 32, 63, 64, 70, 200] {
            let mut b = started();
            let x = b.imm(
                ty,
                Const::Int(0xdead_beef_1234_5678 & ((1u128 << ty.bits()) - 1)),
            );
            let n = b.imm(ty, Const::Int(u128::from(amount)));
            let live = [
                (0u16, b.binary(Opcode::SHL, ty, x, n)),
                (1, b.binary(Opcode::SHR, ty, x, n)),
                (2, b.binary(Opcode::SAR, ty, x, n)),
                (3, b.binary(Opcode::ROTL, ty, x, n)),
                (4, b.binary(Opcode::ROTR, ty, x, n)),
            ];
            agree(&wrap(b, &live));
        }
    }
}

#[test]
fn a_negative_arithmetic_shift_replicates_its_sign_within_the_type() {
    for ty in [Type::I32, Type::I64] {
        let top = 1u128 << (ty.bits() - 1);
        for amount in [1u64, 4, 64, 100] {
            let mut b = started();
            let x = b.imm(ty, Const::Int(top | 0x1234));
            let n = b.imm(ty, Const::Int(u128::from(amount)));
            let live = [(0u16, b.binary(Opcode::SAR, ty, x, n))];
            agree(&wrap(b, &live));
        }
    }
}

#[test]
fn extension_truncation_bitfields_and_byte_swaps_agree() {
    let mut b = started();
    let wide = b.imm(Type::I64, Const::Int(0x89ab_cdef_0123_4567));
    let narrow = b.unary(Opcode::TRUNC, Type::I32, wide);
    let a = bswap(&mut b, Type::I64, wide, None);
    let c = bswap(&mut b, Type::I64, wide, Some(16));
    let d = bswap(&mut b, Type::I32, narrow, Some(32));
    let e = bitfield(&mut b, Opcode::EXTRACT, Type::I64, &[wide], 12, 20);
    let f = bitfield(&mut b, Opcode::DEPOSIT, Type::I64, &[wide, narrow], 8, 16);
    let live = [
        (0u16, narrow),
        (1, b.unary(Opcode::EXT_S, Type::I64, narrow)),
        (2, b.unary(Opcode::EXT_Z, Type::I64, narrow)),
        (3, a),
        (4, c),
        (5, d),
        (6, e),
        (7, f),
    ];
    agree(&wrap(b, &live));
}

#[test]
fn every_condition_agrees_in_setcond_and_in_movcond() {
    for ty in [Type::I32, Type::I64] {
        for (a, c) in [
            (1u64, 2u64),
            (2, 1),
            (1, 1),
            (u64::MAX, 1),
            (1, u64::MAX),
            (0x8000_0000, 0x7fff_ffff),
        ] {
            let mut b = started();
            let mask = (1u128 << ty.bits()) - 1;
            let x = b.imm(ty, Const::Int(u128::from(a) & mask));
            let y = b.imm(ty, Const::Int(u128::from(c) & mask));
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
                let set = b.setcond(cond, ty, x, y);
                let pick = movcond_sel(&mut b, ty, set, x, y);
                live.push((i as u16 % SLOTS, set));
                live.push(((i as u16 + 1) % SLOTS, pick));
            }
            agree(&wrap(b, &live));
        }
    }
}

#[test]
fn a_forward_branch_skips_exactly_what_it_jumps_over() {
    // The nesting `Nest` builds, with the branch taken and not taken — the
    // shape a superblock's side exit has.
    for taken in [0u128, 1] {
        let mut b = BlockBuilder::new(0x100, 0);
        b.insn_start(at(0x100, 0, &[]));
        let sel = b.imm(Type::I1, Const::Int(taken));
        let one = b.imm(Type::I64, Const::Int(1));
        let acc = b.imm(Type::I64, Const::Int(0));
        let branch = brcond_sel(&mut b, sel);
        let skipped = b.binary(Opcode::ADD, Type::I64, acc, one);
        let after = b.binary(Opcode::ADD, Type::I64, skipped, one);
        let target = b.next_index();
        b.patch_aux(branch, target as u32);
        b.insn_start(at(0x104, 0, &[(RegSlot(0).0, after), (RegSlot(1).0, acc)]));
        b.exit_tb();
        agree(&b.finish());
    }
}

#[test]
fn two_branch_targets_nest_rather_than_overlap() {
    // Two distinct forward targets means two `block`s, and the inner one must
    // be the nearer target or a `br` lands in the wrong place. Four selector
    // combinations, so every path through the nesting is executed.
    for first in [0u128, 1] {
        for second in [0u128, 1] {
            let mut b = BlockBuilder::new(0x100, 0);
            b.insn_start(at(0x100, 0, &[]));
            let s1 = b.imm(Type::I1, Const::Int(first));
            let s2 = b.imm(Type::I1, Const::Int(second));
            let one = b.imm(Type::I64, Const::Int(1));
            let mut acc = b.imm(Type::I64, Const::Int(0));
            let far = brcond_sel(&mut b, s2);
            let near = brcond_sel(&mut b, s1);
            acc = b.binary(Opcode::ADD, Type::I64, acc, one);
            let near_target = b.next_index();
            acc = b.binary(Opcode::ADD, Type::I64, acc, one);
            let far_target = b.next_index();
            acc = b.binary(Opcode::ADD, Type::I64, acc, one);
            b.patch_aux(near, near_target as u32);
            b.patch_aux(far, far_target as u32);
            b.insn_start(at(0x104, 0, &[(0, acc)]));
            b.exit_tb();
            agree(&b.finish());
        }
    }
}

#[test]
fn loads_and_stores_reach_the_host_with_their_descriptor_intact() {
    for (width, sign) in [
        (Width::U8, Sign::Unsigned),
        (Width::U8, Sign::Signed),
        (Width::U16, Sign::Signed),
        (Width::U32, Sign::Unsigned),
        (Width::U32, Sign::Signed),
        (Width::U64, Sign::Unsigned),
    ] {
        let mut b = started();
        let addr = b.imm(Type::I64, Const::Int(u128::from(BASE + 16)));
        let mut ld = MemOp::load(width);
        ld.sign = sign;
        let v = b.load(Type::I64, addr, ld);
        let dest = b.imm(Type::I64, Const::Int(u128::from(BASE + 64)));
        b.store(Type::I64, dest, v, MemOp::store(width));
        agree(&wrap(b, &[(0, v), (1, addr)]));
    }
}

#[test]
fn a_faulting_access_stops_the_block_where_the_interpreter_stops_it() {
    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, 0, &[]));
    let good = b.imm(Type::I64, Const::Int(u128::from(BASE)));
    let first = b.load(Type::I64, good, MemOp::load(Width::U32));
    b.charge(3);
    b.insn_start(at(0x104, 3, &[(0, first)]));
    // Outside the mapping, so the host answers with a bus error.
    let bad = b.imm(Type::I64, Const::Int(0xdead_0000));
    let _ = b.load(Type::I64, bad, MemOp::load(Width::U32));
    b.insn_start(at(0x108, 3, &[(1, first)]));
    b.exit_tb();
    let out = b.finish();
    agree(&out);
    // And it really does fault, at the boundary it faulted at.
    let (want, _) = interpreted(&out, Host::new());
    match want.outcome {
        Some(Outcome::Fault(f)) => {
            assert_eq!(f.error, BusError::Unassigned);
            assert_eq!(f.pc, 0x104);
            // The three `charge` ticks. The tick the first load spent is the
            // *host's* and is not in `Interp::ticks`, which is what
            // `Fault::retired_ticks` counts — so the two engines agreeing on
            // this number is agreement about the same column.
            assert_eq!(f.retired_ticks, 3);
        }
        other => panic!("the test block must actually fault: {other:?}"),
    }
}

#[test]
fn a_spent_allowance_leaves_the_block_at_a_boundary_and_not_at_an_exit() {
    // Two boundaries plus the exit boundary, and an allowance that runs out at
    // the second. `Interp` never asks at the block's first boundary nor at an
    // exit boundary, and the compiled form decides the second of those
    // statically — so this is the test that the static decision matches.
    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, 0, &[]));
    b.charge(10);
    let one = b.imm(Type::I64, Const::Int(1));
    b.insn_start(at(0x104, 10, &[(0, one)]));
    b.charge(10);
    let two = b.binary(Opcode::ADD, Type::I64, one, one);
    b.insn_start(at(0x108, 20, &[(0, two)]));
    b.exit_tb();
    let block = b.finish();

    let mut spent_once = false;
    for allowance in [u64::MAX, 25, 15, 5] {
        let mut host = Host::new();
        host.allowance = allowance;
        let (want, interp) = interpreted(&block, host.clone());
        let (got, engine, _) = compiled(&block, host);
        assert_eq!(want.outcome, got.outcome, "allowance {allowance}");
        assert_eq!(want.slots, got.slots, "allowance {allowance}");
        assert_eq!(want.ticks, got.ticks, "allowance {allowance}");
        assert_eq!(interp.boundaries(), engine.boundaries());
        spent_once |= matches!(want.outcome, Some(Outcome::Spent { .. }));
    }
    assert!(
        spent_once,
        "no allowance in this test ever ran out, so it checked nothing"
    );
}

#[test]
fn a_slot_read_of_a_shadowed_slot_sees_the_temporary_and_not_the_host() {
    // Guest state is published lazily, so a `get_slot` of a slot the pending
    // boundary binds has to publish first. The compiled path reaches that
    // through the same import, over the frame's write-through copy — which is
    // the one thing a backend holding temporaries in registers has to arrange
    // and this one gets from the write-through set.
    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, 0, &[]));
    let fresh = b.imm(Type::I64, Const::Int(0xabc_def));
    b.insn_start(at(0x104, 0, &[(2, fresh)]));
    let read = b.get_slot(Type::I64, RegSlot(2));
    b.insn_start(at(0x108, 0, &[(2, fresh), (5, read)]));
    b.exit_tb();
    agree(&b.finish());
}

#[test]
fn the_three_terminators_say_where_they_go() {
    let mut b = started();
    goto_tb(&mut b, 0x2000);
    let block = b.finish();
    let (want, _) = interpreted(&block, Host::new());
    let (got, ..) = compiled(&block, Host::new());
    assert_eq!(want.outcome, Some(Outcome::Goto { pc: 0x2000 }));
    assert_eq!(want.outcome, got.outcome);

    let mut b = started();
    let pc = b.imm(Type::I64, Const::Int(0x3000));
    lookup_and_goto(&mut b, pc);
    let block = b.finish();
    let (want, _) = interpreted(&block, Host::new());
    let (got, ..) = compiled(&block, Host::new());
    assert_eq!(want.outcome, Some(Outcome::Lookup { pc: 0x3000 }));
    assert_eq!(want.outcome, got.outcome);

    let block = wrap(started(), &[]);
    let (want, _) = interpreted(&block, Host::new());
    let (got, ..) = compiled(&block, Host::new());
    assert_eq!(want.outcome, Some(Outcome::Exit));
    assert_eq!(want.outcome, got.outcome);
}

// ---------------------------------------------------------------------------
// The module table
// ---------------------------------------------------------------------------

#[test]
fn an_evicted_module_stops_being_live_and_nothing_else_does() {
    // `ROADMAP.md` §11.4: a bounded table with LRU eviction. The property that
    // matters is not which module goes — it is that a `CodeRef` into the slot
    // that was reused stops answering `is_live`, so the dispatcher recompiles
    // instead of entering a module that is no longer there.
    let mut engine = Engine::with_capacity(2);
    let a = wrap(started(), &[]);
    let mut bb = started();
    let one = bb.imm(Type::I64, Const::Int(1));
    let b = wrap(bb, &[(0, one)]);
    let mut cb = started();
    let two = cb.imm(Type::I64, Const::Int(2));
    let c = wrap(cb, &[(0, two)]);

    let ra = engine.compile(&a).expect("compiles");
    let rb = engine.compile(&b).expect("compiles");
    assert!(engine.is_live(ra) && engine.is_live(rb));
    // Touch `a`, so `b` is the least recently used.
    let mut host = Host::new();
    let _ = engine.run(&a, ra, &mut host).expect("live");
    let rc = engine.compile(&c).expect("compiles");
    assert!(engine.is_live(ra), "the recently used module stayed");
    assert!(!engine.is_live(rb), "the cold module was evicted");
    assert!(engine.is_live(rc));
    assert_eq!(engine.stats().evicted, 1);
    // And the stale handle is refused rather than run.
    assert!(engine.run(&b, rb, &mut host).is_none());
}

#[test]
fn a_module_that_is_run_twice_gives_the_same_answer_twice() {
    // The frame is reused across runs, so a run that read a stale temporary
    // out of it would pass once and fail here.
    let mut b = started();
    let x = b.imm(Type::I64, Const::Int(9));
    let y = b.binary(Opcode::ADD, Type::I64, x, x);
    let block = wrap(b, &[(4, y)]);
    let mut engine = Engine::with_capacity(2);
    let code = engine.compile(&block).expect("compiles");
    let mut first = Host::new();
    let mut second = Host::new();
    let a = engine.run(&block, code, &mut first).expect("live");
    let b = engine.run(&block, code, &mut second).expect("live");
    assert_eq!(a.ok(), b.ok());
    assert_eq!(first.slots, second.slots);
    assert_eq!(engine.stats().executed, 2);
    assert_eq!(engine.stats().compiled, 1);
    assert!(engine.stats().bytes > 0);
}

#[test]
fn a_block_that_falls_off_its_end_is_an_error_and_not_a_trap() {
    // `verify` rejects this shape; the fuzz targets can still build it, and a
    // wasm backend that let it become a trap would lose the message.
    let mut b = BlockBuilder::new(0x100, 0);
    b.insn_start(at(0x100, 0, &[]));
    let block = b.finish();
    let mut engine = Engine::with_capacity(1);
    let code = engine
        .compile(&block)
        .expect("a terminator-less block still lowers");
    let mut host = Host::new();
    let out = engine.run(&block, code, &mut host).expect("live");
    assert!(out.is_err(), "it must be an error: {out:?}");
    // And the interpreter says the same thing about it.
    let (want, _) = interpreted(&block, Host::new());
    assert!(want.error.is_some());
}
