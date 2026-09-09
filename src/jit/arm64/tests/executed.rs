//! The backend's own differential: generated A64 against `ir::Interp`.
//!
//! **This module only exists on an aarch64 Linux host.** Everything in it runs
//! compiled code, and compiled code here is A64 — so on the machine this
//! backend was written on, none of it is compiled at all. That is the shape of
//! the problem rather than a gap in the testing: an encoder can be checked
//! against the manual anywhere, and *agreement with the oracle* can only be
//! checked where the code runs.
//!
//! CLAUDE.md makes a guest's interpreter the oracle for its frontend; one level
//! down, [`Interp`](crate::ir::Interp) is the oracle for every host backend.
//! This is the differential *at the IR*: random blocks over the compiled
//! opcode set, run twice against two identical hosts, compared on every
//! temporary, every guest slot, the tick count, guest memory, the boundary
//! count and the outcome. It is `jit::x86::tests`' harness with the generator
//! narrowed to the ops this backend lowers, deliberately kept recognisable so
//! that the two can be read against each other.
//!
//! Both hosts here carry a real [`Tlb`], so the inlined fast path is exercised
//! by construction and its answer is compared against the same TLB reached
//! through [`IrHost::load`].

use alloc::collections::BTreeMap;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::space::{AddressSpace, MemAttrs, MemResult, RamStore, Region, UnassignedPolicy};
use crate::core::value::Width;
use crate::ir::{
    AccessKind, Block, BlockBuilder, Cond, Const, InsnStart, Interp, IrHost, MemOp, Opcode,
    RegSlot, Sign, Temp, Type, bitfield_aux, verify,
};
use crate::jit::{Context, FastMem, MemPlan, Tlb};

use super::super::compile::Regs;
use super::super::rt::Engine;

/// Where the test machine's RAM lives.
const BASE: u64 = 0x2000_0000;
/// Four pages, so an address can miss the mapping and fault.
const RAM: u64 = 4 * 4096;
/// The world a *load* happens in.
const WORLD: Context = Context {
    level: 3,
    translating: false,
};
/// The world a *store* happens in — deliberately not [`WORLD`], so a backend
/// that read the load plan's tag bits for a store is distinguishable from a
/// correct one.
const STORE_WORLD: Context = Context {
    level: 2,
    translating: false,
};
/// How many guest state slots the generator uses.
const SLOTS: u16 = 8;

// ---------------------------------------------------------------------------
// The host both engines run against
// ---------------------------------------------------------------------------

/// What a host was asked to do, in order.
///
/// Compared between the two engines, because agreeing on the final state while
/// making a different sequence of calls is exactly the class of bug a
/// state-only comparison misses.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Event {
    Charge(u64),
    Boundary(u64),
    Publish(u16, u128),
}

struct Scratch {
    slots: BTreeMap<u16, u128>,
    ram: Arc<RamStore>,
    tlb: Tlb,
    /// Whether the backend may inline the fast path. Off is the control: the
    /// same block must produce the same everything with every load a call.
    inline: bool,
    ticks: u64,
    log: Vec<Event>,
    /// `(address, width)` of every store the backend served itself.
    inlined_stores: Vec<(u64, u64)>,
    /// The tick allowance, or `None` for a host that never stops a block.
    allowance: Option<u64>,
}

impl Scratch {
    fn new(inline: bool) -> Scratch {
        let ram = Arc::new(RamStore::new(RAM));
        for i in 0..RAM {
            ram.write_u8(i, (i.wrapping_mul(31) ^ (i >> 5)) as u8)
                .expect("in range");
        }
        let space = AddressSpace::new("mem", 64).with_unassigned(UnassignedPolicy::FAULT);
        space
            .topology()
            .map(Region::ram("ram", Arc::clone(&ram)), BASE)
            .expect("one region maps");
        let space = Arc::new(space);
        let mut slots = BTreeMap::new();
        for s in 0..SLOTS {
            slots.insert(s, u128::from(u64::from(s) * 0x1111 + 7));
        }
        Scratch {
            slots,
            ram,
            // **Two** entries, over four guest pages, so pages collide: the
            // tag compare is then the only thing standing between a probe and
            // another page's bytes.
            tlb: Tlb::with_entries(space, 2),
            inline,
            ticks: 0,
            log: Vec::new(),
            inlined_stores: Vec::new(),
            allowance: None,
        }
    }

    /// The same host, leaving a block at the first boundary past `ticks`.
    fn within(inline: bool, ticks: u64) -> Scratch {
        Scratch {
            allowance: Some(ticks),
            ..Scratch::new(inline)
        }
    }

    /// The bytes of guest RAM, for the memory comparison.
    fn bytes(&self) -> Vec<u8> {
        let mut out = vec![0u8; RAM as usize];
        self.ram.read_at(0, &mut out).expect("the whole store");
        out
    }
}

impl IrHost for Scratch {
    fn read_slot(&mut self, slot: RegSlot) -> u128 {
        self.slots
            .get(&slot.0)
            .copied()
            .unwrap_or_else(|| u128::from(slot.0) * 0x1000_0001 + 3)
    }

    fn write_slot(&mut self, slot: RegSlot, value: u128) {
        self.slots.insert(slot.0, value);
        self.log.push(Event::Publish(slot.0, value));
    }

    fn load(&mut self, mem: &MemOp, addr: u64) -> MemResult<u64> {
        // One bus access, one tick — the same rule `note_fast_load` keeps.
        self.ticks += 1;
        self.tlb.read(
            AccessKind::Load,
            addr,
            addr,
            mem.size,
            WORLD,
            MemAttrs::DEFAULT,
        )
    }

    fn store(&mut self, mem: &MemOp, addr: u64, value: u64) -> MemResult {
        // `IrHost::store` documents that the value arrives already truncated
        // to the access width.
        assert_eq!(
            value,
            value & mem.size.mask(),
            "a store reached the host untruncated"
        );
        self.ticks += 1;
        self.tlb
            .write(addr, addr, mem.size, value, STORE_WORLD, MemAttrs::DEFAULT)
    }

    fn charge(&mut self, ticks: u64) {
        self.ticks += ticks;
        self.log.push(Event::Charge(ticks));
    }

    fn insn_start(&mut self, mark: &InsnStart) {
        self.log.push(Event::Boundary(mark.pc));
    }

    fn spent(&self) -> bool {
        self.allowance.is_some_and(|a| self.ticks >= a)
    }
}

impl FastMem for Scratch {
    fn load_plan(&mut self) -> Option<MemPlan> {
        self.inline.then(|| self.tlb.plan(AccessKind::Load, WORLD))
    }

    fn note_fast_load(&mut self) {
        // Exactly what `load` charges for one access, and nothing else.
        self.ticks += 1;
    }

    fn store_plan(&mut self) -> Option<MemPlan> {
        self.inline
            .then(|| self.tlb.plan(AccessKind::Store, STORE_WORLD))
    }

    fn note_fast_store(&mut self, addr: u64, bytes: u64) {
        self.ticks += 1;
        assert_eq!(
            self.tlb.note_fast_store(addr, bytes),
            Some(addr),
            "an inlined store must report where it landed"
        );
        self.inlined_stores.push((addr, bytes));
    }
}

// ---------------------------------------------------------------------------
// The comparison
// ---------------------------------------------------------------------------

/// Run `block` on both engines under both register policies, and assert they
/// agreed about everything.
///
/// Returns whether the compiled run happened at all, so a caller can tell a
/// clean agreement from a block the backend refused.
fn agree(block: &Block, inline: bool) -> bool {
    let frame = agree_under(block, inline, Regs::Frame, None);
    let scan = agree_under(block, inline, Regs::Scan, None);
    assert_eq!(
        frame, scan,
        "one policy compiled and the other did not\n{block}"
    );
    scan
}

/// Both policies again, with a tick allowance the block may not fit.
fn agree_within(block: &Block, inline: bool, allowance: u64) -> bool {
    let frame = agree_under(block, inline, Regs::Frame, Some(allowance));
    let scan = agree_under(block, inline, Regs::Scan, Some(allowance));
    assert_eq!(
        frame, scan,
        "one policy compiled and the other did not\n{block}"
    );
    scan
}

/// What a differential run fills the temporary frame with the second time.
const POISON: u64 = 0xa5a5_5a5a_dead_beef;

/// One policy's run against the interpreter.
fn agree_under(block: &Block, inline: bool, regs: Regs, allowance: Option<u64>) -> bool {
    verify(block).expect("the generator produces well-formed blocks");

    let scratch = |inline| match allowance {
        Some(ticks) => Scratch::within(inline, ticks),
        None => Scratch::new(inline),
    };
    let mut oracle_host = scratch(inline);
    let mut interp = Interp::new();
    let oracle = interp.run(block, &mut oracle_host);

    let mut engine = Engine::with_capacity(1 << 18).expect("a code buffer");
    engine.set_regs(regs);
    let code = match engine.compile(block) {
        Ok(c) => c,
        Err(_) => return false,
    };
    // Zero the frame first: `Engine::run` does not clear it between blocks, so
    // starting from zeroes is what makes the comparison below reproducible.
    engine.seed_frame(0, block.temp_count());
    let mut host = scratch(inline);
    let subject = engine
        .run(block, code, &mut host)
        .expect("the code was compiled in this generation");

    match (&oracle, &subject) {
        (Ok(a), Ok(b)) => assert_eq!(a, b, "the outcome differs\n{block}"),
        (Err(a), Err(b)) => assert_eq!(
            alloc::format!("{a}"),
            alloc::format!("{b}"),
            "the error differs\n{block}"
        ),
        _ => panic!("one engine failed and the other did not: {oracle:?} vs {subject:?}\n{block}"),
    }

    // Every temporary the run *kept*. A register-allocated one is gone once
    // the epilogue has restored the caller's registers, and `temp_value` says
    // so rather than handing back the frame's zero.
    //
    // **Not on a run that left part-way through**: a block that leaves at a
    // boundary abandons whatever its region had computed past that boundary,
    // so those temporaries differ by construction. What must agree there is
    // the stopping boundary's own live mapping.
    let mut compared = 0;
    if allowance.is_none() {
        for t in 0..block.temp_count() {
            let temp = Temp(t as u32);
            let want = interp.temp_value(temp).expect("allocated");
            if let Some(got) = engine.temp_value(temp) {
                assert_eq!(want, u128::from(got), "temporary {temp} differs\n{block}");
                compared += 1;
            }
        }
        if regs == Regs::Frame {
            assert_eq!(
                compared,
                block.temp_count(),
                "the control policy must keep every temporary\n{block}"
            );
        }
    } else {
        let stopped = interp
            .mark()
            .and_then(|m| block.marks().get(m as usize))
            .map(|m| m.live.as_slice())
            .unwrap_or_default();
        for &(slot, temp) in stopped {
            let want = interp.temp_value(temp).expect("allocated");
            let got = engine
                .temp_value(temp)
                .expect("a boundary's live temporary is frame-backed");
            assert_eq!(
                want,
                u128::from(got),
                "temporary {temp}, live for slot {} at the boundary the run left \
                 at, differs\n{block}",
                slot.0
            );
            compared += 1;
        }
    }
    let _ = compared;
    // `ROADMAP.md` §9's precise-exception contract, asserted on every block
    // rather than only on the ones that fault.
    for mark in block.marks() {
        for &(slot, temp) in &mark.live {
            assert!(
                engine.temp_value(temp).is_some(),
                "{temp} is named live for slot {} at {:#x} and the frame does not hold it\n{block}",
                slot.0,
                mark.pc
            );
        }
    }
    assert_eq!(
        oracle_host.slots, host.slots,
        "guest state differs\n{block}"
    );
    assert_eq!(oracle_host.ticks, host.ticks, "ticks differ\n{block}");
    assert_eq!(
        oracle_host.log, host.log,
        "the two engines asked the host to do different things\n{block}"
    );
    assert_eq!(
        oracle_host.bytes(),
        host.bytes(),
        "guest memory differs\n{block}"
    );
    assert_eq!(
        interp.ticks(),
        engine.ticks(),
        "the charged column differs\n{block}"
    );
    assert_eq!(
        interp.boundaries(),
        engine.boundaries(),
        "the retired instruction count differs\n{block}"
    );
    assert_eq!(
        interp.mark(),
        engine.mark(),
        "the boundary differs\n{block}"
    );

    // ---- and none of it depended on the frame being zero ------------------
    //
    // The engine keeps one temporary frame at the high-water mark of every
    // block it has run and does not clear it, so a temporary whose definition
    // the executed path branched over holds an earlier block's leftovers
    // rather than a zero. Nothing reachable may read one; this is what says
    // so, because a backend that did read one would still have agreed with the
    // interpreter above, where both of them saw zeroes.
    let kept: Vec<Option<u64>> = (0..block.temp_count())
        .map(|t| engine.temp_value(Temp(t as u32)))
        .collect();
    engine.seed_frame(POISON, block.temp_count());
    let mut other = scratch(inline);
    let again = engine
        .run(block, code, &mut other)
        .expect("the code was compiled in this generation");
    match (&subject, &again) {
        (Ok(a), Ok(b)) => assert_eq!(a, b, "the outcome moved with the frame\n{block}"),
        (Err(a), Err(b)) => assert_eq!(
            alloc::format!("{a}"),
            alloc::format!("{b}"),
            "the error moved with the frame\n{block}"
        ),
        _ => panic!("the frame's contents decided whether the block failed\n{block}"),
    }
    assert_eq!(
        host.slots, other.slots,
        "guest state moved with the frame\n{block}"
    );
    assert_eq!(
        host.ticks, other.ticks,
        "ticks moved with the frame\n{block}"
    );
    assert_eq!(
        host.log, other.log,
        "the host was asked to do different things\n{block}"
    );
    assert_eq!(
        host.bytes(),
        other.bytes(),
        "guest memory moved with the frame\n{block}"
    );
    assert_eq!(
        host.inlined_stores, other.inlined_stores,
        "the inlined stores moved with the frame\n{block}"
    );
    for (t, was) in kept.iter().enumerate() {
        let temp = Temp(t as u32);
        match (was, engine.temp_value(temp)) {
            (Some(a), Some(b)) => assert!(
                b == *a || b == POISON,
                "temporary {temp} came back {b:#x}, which is neither the value the \
                 zeroed frame produced ({a:#x}) nor the fill\n{block}"
            ),
            (None, None) => {}
            _ => panic!(
                "{temp} changed whether the frame holds it between two runs of the \
                 same code\n{block}"
            ),
        }
    }
    true
}

// ---------------------------------------------------------------------------
// The generator
// ---------------------------------------------------------------------------

/// A tiny deterministic source of randomness.
///
/// xorshift64*, so a seed reproduces a case exactly — a differential harness
/// whose failures cannot be replayed is a harness nobody can fix a bug with.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n.max(1)
    }
    fn pick<T: Copy>(&mut self, xs: &[T]) -> T {
        xs[self.below(xs.len() as u64) as usize]
    }
}

const CONDS: [Cond; 10] = [
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
];

const WIDTHS: [Width; 4] = [Width::U8, Width::U16, Width::U32, Width::U64];

/// A random block over the compiled opcode set.
///
/// Shaped like something a frontend would emit — boundaries, charges, a live
/// map that grows, one side exit, a terminator. The opcode menu is narrower
/// than the x86 harness's by exactly the ops this backend refuses: a generator
/// that emitted them would spend its blocks on the interpreter and prove
/// nothing about the code generator.
fn random_block(seed: u64, insns: usize) -> Block {
    let mut r = Rng(seed ^ 0x9e37_79b9_7f4a_7c15);
    let mut b = BlockBuilder::new(BASE, 0);
    let mut pool: Vec<(Temp, Type)> = Vec::new();
    let mut live: Vec<(RegSlot, Temp)> = Vec::new();
    let mut ticks = 0u64;
    let mut pc = BASE;

    // Something to start from, so the first op has operands of every type it
    // may ask for.
    for ty in [Type::I32, Type::I64, Type::I32, Type::I64, Type::I64] {
        let t = b.imm(ty, Const::Int(u128::from(r.next())));
        pool.push((t, ty));
    }

    let mut side_exit_at = if insns > 4 {
        Some(2 + r.below(insns as u64 - 3) as usize)
    } else {
        None
    };

    for step in 0..insns {
        b.insn_start(InsnStart {
            pc,
            next_pc: pc + 4,
            ticks,
            live: live.clone(),
        });
        // Sometimes nothing: an instruction that charges no ticks is ordinary,
        // and it is the only way the commit flag's *clearing* becomes visible.
        if r.below(4) != 0 {
            let charge = 1 + r.below(3);
            b.charge(charge);
            ticks += charge;
        }
        pc += 4;

        if side_exit_at == Some(step) {
            side_exit_at = None;
            // A superblock's side exit: a `brcond` that jumps *over* an inline
            // exit sequence, which is the shape `cpu::riscv::lift` emits. The
            // condition is constant at run time but opaque to the builder, and
            // which one decides whether the block leaves through its side exit
            // or runs on to its own terminator.
            let sel = of_type(&mut r, &pool, Type::I64);
            let cond = if r.below(2) == 0 { Cond::LtU } else { Cond::Eq };
            let taken = b.setcond(cond, Type::I64, sel, sel);
            let over = b.emit_raw(
                Opcode::BRCOND,
                Type::I64,
                None,
                None,
                &[taken],
                None,
                None,
                0,
            );
            let t = b.imm(Type::I64, Const::Int(u128::from(pc)));
            let mut exit_live = live.clone();
            exit_live.push((RegSlot(SLOTS), t));
            b.insn_start(InsnStart {
                pc,
                next_pc: pc,
                ticks,
                live: exit_live,
            });
            b.exit_tb();
            b.patch_aux(over, b.next_index() as u32);
        }

        let made = emit_one(&mut b, &mut r, &pool);
        for entry in made {
            pool.push(entry);
        }
        // Rebind a slot to something recent, so the live map is not static and
        // publication has to pick the right boundary.
        if !pool.is_empty() && r.below(3) != 0 {
            let slot = RegSlot(r.below(u64::from(SLOTS)) as u16);
            let (t, _) = pool[pool.len() - 1];
            match live.iter_mut().find(|(s, _)| *s == slot) {
                Some(entry) => entry.1 = t,
                None => live.push((slot, t)),
            }
        }
    }

    let t = b.imm(Type::I64, Const::Int(u128::from(pc)));
    live.push((RegSlot(SLOTS), t));
    b.insn_start(InsnStart {
        pc,
        next_pc: pc,
        ticks,
        live,
    });
    // All three terminators, because they are three different `Outcome`s and a
    // backend that collapsed two of them would still leave the guest's state
    // right — the dispatcher would simply go to the wrong place next.
    match r.below(3) {
        0 => {
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
        1 => {
            b.emit_raw(
                Opcode::LOOKUP_AND_GOTO,
                Type::I64,
                None,
                None,
                &[t],
                None,
                None,
                0,
            );
        }
        _ => b.exit_tb(),
    }
    b.finish()
}

/// A temporary of exactly `ty`, or one of any type when there is none.
fn of_type(r: &mut Rng, pool: &[(Temp, Type)], ty: Type) -> Temp {
    let matching: Vec<Temp> = pool
        .iter()
        .filter(|(_, t)| *t == ty)
        .map(|(temp, _)| *temp)
        .collect();
    if matching.is_empty() {
        pool[r.below(pool.len() as u64) as usize].0
    } else {
        matching[r.below(matching.len() as u64) as usize]
    }
}

/// Emit one random operation, returning the temporaries it defined.
#[allow(clippy::too_many_lines)]
fn emit_one(b: &mut BlockBuilder, r: &mut Rng, pool: &[(Temp, Type)]) -> Vec<(Temp, Type)> {
    let ty = r.pick(&[Type::I32, Type::I64, Type::I64]);
    let w = ty.bits();
    let any = |r: &mut Rng| of_type(r, pool, ty);

    match r.below(19) {
        0 => {
            let op = r.pick(&[
                Opcode::ADD,
                Opcode::SUB,
                Opcode::MUL,
                Opcode::AND,
                Opcode::OR,
                Opcode::XOR,
                Opcode::ANDC,
            ]);
            let (x, y) = (any(r), any(r));
            vec![(b.binary(op, ty, x, y), ty)]
        }
        1 => {
            let op = r.pick(&[Opcode::NOT, Opcode::NEG]);
            let x = any(r);
            vec![(b.unary(op, ty, x), ty)]
        }
        2 => {
            let op = r.pick(&[Opcode::SHL, Opcode::SHR, Opcode::SAR]);
            let (x, y) = (any(r), any(r));
            vec![(b.binary(op, ty, x, y), ty)]
        }
        3 => {
            // A shift by a *chosen* amount, so the in-range path is not reached
            // only by luck — a random 64-bit operand is out of range almost
            // always — and so the boundary is reached at all. Exactly `w` is
            // the case the IR calls undefined and the two engines still have to
            // agree about: A64's `LSLV` masks the amount and would quietly
            // shift by zero where the interpreter takes the mathematical
            // answer, which is why this backend emits the `CSEL`.
            let op = r.pick(&[Opcode::SHL, Opcode::SHR, Opcode::SAR]);
            let x = any(r);
            let n = match r.below(6) {
                0 => u64::from(w) - 1,
                1 => u64::from(w),
                2 => u64::from(w) + 1,
                3 => 0,
                _ => r.below(u64::from(w)),
            };
            let amount = b.imm(Type::I64, Const::Int(u128::from(n)));
            vec![(b.binary(op, ty, x, amount), ty)]
        }
        4 => {
            // The rotates, whose amount A64 *does* take modulo the width for
            // free — and a left rotate, which it has no instruction for at all.
            let op = r.pick(&[Opcode::ROTL, Opcode::ROTR]);
            let x = of_type(r, pool, ty);
            // Zero often, because `ROTL` by zero is the case the negate-then-
            // rotate-right identity is least obviously right about.
            let n = if r.below(3) == 0 {
                b.imm(ty, Const::Int(0))
            } else {
                any(r)
            };
            vec![(b.binary(op, ty, x, n), ty)]
        }
        5 => {
            let op = r.pick(&[Opcode::CLZ, Opcode::CTZ]);
            // Zero often, because the zero input is the case a lowering gets
            // wrong: `CLZ` must answer the type's width and `CTZ`, through
            // `RBIT`, the same.
            let x = if r.below(3) == 0 {
                b.imm(ty, Const::Int(0))
            } else {
                any(r)
            };
            vec![(b.unary(op, ty, x), ty)]
        }
        6 => {
            let cond = r.pick(&CONDS);
            let (x, y) = (any(r), any(r));
            vec![(b.setcond(cond, ty, x, y), Type::I1)]
        }
        7 => {
            // `movcond` on a one-bit selector.
            let sel = b.setcond(r.pick(&CONDS), ty, any(r), any(r));
            let dst = b.temp(ty);
            let (t, f) = (any(r), any(r));
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
            vec![(sel, Type::I1), (dst, ty)]
        }
        8 => {
            // `movcond` in its compare-and-select shape.
            let dst = b.temp(ty);
            let (x, y, t, f) = (any(r), any(r), any(r), any(r));
            b.emit_raw(
                Opcode::MOVCOND,
                ty,
                Some(dst),
                None,
                &[x, y, t, f],
                None,
                Some(r.pick(&CONDS)),
                0,
            );
            vec![(dst, ty)]
        }
        9 => {
            let op = r.pick(&[Opcode::EXT_S, Opcode::EXT_Z, Opcode::TRUNC]);
            let x = any(r);
            vec![(b.unary(op, ty, x), ty)]
        }
        10 => {
            // Whole-type and narrow-lane both: A64 has `REV16` and `REV32` for
            // exactly the lanes the IR names, which is the one place this
            // backend is a single instruction where x86 is a cascade.
            let x = any(r);
            let lane = match r.below(3) {
                0 => 16u64,
                1 => 32,
                _ => u64::from(w),
            };
            if lane > u64::from(w) {
                return vec![(b.unary(Opcode::BSWAP, ty, x), ty)];
            }
            let dst = b.temp(ty);
            b.emit_raw(
                Opcode::BSWAP,
                ty,
                Some(dst),
                None,
                &[x],
                Some(Const::Int(u128::from(lane))),
                None,
                0,
            );
            vec![(dst, ty)]
        }
        11 => {
            let len = 1 + r.below(u64::from(w)) as u32;
            let pos = r.below(u64::from(w - len) + 1) as u32;
            let dst = b.temp(ty);
            let x = any(r);
            b.emit_raw(
                Opcode::EXTRACT,
                ty,
                Some(dst),
                None,
                &[x],
                None,
                None,
                bitfield_aux(pos, len),
            );
            vec![(dst, ty)]
        }
        12 => {
            let len = 1 + r.below(u64::from(w)) as u32;
            let pos = r.below(u64::from(w - len) + 1) as u32;
            let dst = b.temp(ty);
            let (into, what) = (any(r), any(r));
            b.emit_raw(
                Opcode::DEPOSIT,
                ty,
                Some(dst),
                None,
                &[into, what],
                None,
                None,
                bitfield_aux(pos, len),
            );
            vec![(dst, ty)]
        }
        13 => {
            let slot = r.below(u64::from(SLOTS)) as u16;
            vec![(b.get_slot(Type::I64, RegSlot(slot)), Type::I64)]
        }
        14 | 15 => {
            // A load. Mostly inside RAM, and sometimes not, because the fault
            // path is the half of `ROADMAP.md` §9 that is hard.
            let size = r.pick(&WIDTHS);
            let addr = address(b, r, size);
            let mut mem = MemOp::load(size);
            mem.sign = if r.below(2) == 0 {
                Sign::Unsigned
            } else {
                Sign::Signed
            };
            mem.volatile = r.below(2) == 0;
            vec![(b.load(Type::I64, addr, mem), Type::I64)]
        }
        16 => {
            let size = r.pick(&WIDTHS);
            let addr = address(b, r, size);
            let value = any(r);
            let mut mem = MemOp::store(size);
            mem.volatile = r.below(2) == 0;
            b.store(Type::I64, addr, value, mem);
            Vec::new()
        }
        17 => {
            // A destination *narrower* than the operation: the shape `setcond`
            // has by construction, and the one that needs both of the
            // interpreter's masks rather than one.
            let dst = b.temp(Type::I32);
            let (x, y) = (of_type(r, pool, Type::I64), of_type(r, pool, Type::I64));
            b.emit_raw(
                r.pick(&[Opcode::ADD, Opcode::OR, Opcode::MUL]),
                Type::I64,
                Some(dst),
                None,
                &[x, y],
                None,
                None,
                0,
            );
            vec![(dst, Type::I32)]
        }
        18 => {
            // A barrier, in among the loads, the stores and the faults. It
            // defines nothing, so what it exercises is the *shape* it makes: a
            // region boundary with no call on it, an extra flush the allocator
            // has to have been told about, and `Ctx::committed` written
            // between two replays. On this host it is also the instruction the
            // `aarch64` CI job exists to have a machine for.
            b.emit_raw(Opcode::FENCE, Type::I64, None, None, &[], None, None, 0);
            Vec::new()
        }
        _ => {
            let ty = r.pick(&[Type::I32, Type::I64]);
            vec![(b.imm(ty, Const::Int(u128::from(r.next()))), ty)]
        }
    }
}

/// An address temporary: usually a naturally aligned one inside RAM, sometimes
/// misaligned, and sometimes off the end.
fn address(b: &mut BlockBuilder, r: &mut Rng, size: Width) -> Temp {
    let value = match r.below(8) {
        0 => BASE + RAM + r.below(0x1000),
        1 => (BASE + r.below(RAM - 8)) | 1,
        _ => BASE + (r.below(RAM - 8) & !(size.bytes() - 1)),
    };
    b.imm(Type::I64, Const::Int(u128::from(value)))
}

// ---------------------------------------------------------------------------
// The tests
// ---------------------------------------------------------------------------

#[test]
fn a_thousand_random_blocks_agree_with_the_interpreter() {
    let mut compiled = 0;
    for seed in 0..1000u64 {
        let block = random_block(seed, 6 + (seed % 12) as usize);
        if agree(&block, seed % 2 == 0) {
            compiled += 1;
        }
    }
    // The generator emits only ops this backend lowers, so a low rate means
    // the compiler is refusing on *shape* — which would make the whole
    // differential vacuous without failing a single assertion.
    assert!(
        compiled > 950,
        "only {compiled} of 1000 blocks reached the code generator"
    );
}

#[test]
fn a_thousand_random_blocks_leave_at_the_same_boundary_under_an_allowance() {
    // The other half of the seam: with a tick allowance set, the two engines
    // must leave at the *same* guest instruction and not merely produce the
    // same answer at the end.
    let mut compiled = 0;
    for seed in 0..1000u64 {
        let block = random_block(seed ^ 0x5eed, 6 + (seed % 12) as usize);
        if agree_within(&block, seed % 2 == 0, 3 + seed % 17) {
            compiled += 1;
        }
    }
    assert!(compiled > 950, "only {compiled} of 1000 blocks compiled");
}

#[test]
fn the_inlined_fast_path_answers_what_the_call_answers() {
    // The same block, twice, with the host publishing a plan and then not.
    // Every load and store must produce the same value, the same ticks and the
    // same memory whether the probe served it or the thunk did — which is what
    // makes `ROADMAP.md` §9.1's first mechanism a speed-up rather than a second
    // implementation of the memory path.
    for seed in 0..300u64 {
        let block = random_block(seed ^ 0xf00d, 10);
        assert_eq!(
            agree(&block, true),
            agree(&block, false),
            "inlining changed whether the block compiled"
        );
    }
}

#[test]
fn the_inlined_probe_really_serves_the_accesses() {
    // A block of aligned in-range accesses to one page, so every probe after
    // the first is a hit. Without this the differential above would pass just
    // as happily on a backend whose fast path never fired.
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    b.charge(1);
    for i in 0..8u64 {
        let addr = b.imm(Type::I64, Const::Int(u128::from(BASE + i * 8)));
        let v = b.load(Type::I64, addr, MemOp::load(Width::U64));
        b.store(Type::I64, addr, v, MemOp::store(Width::U64));
    }
    b.exit_tb();
    let block = b.finish();
    assert!(agree(&block, true), "the block compiles");

    let mut engine = Engine::with_capacity(1 << 18).expect("a code buffer");
    let code = engine.compile(&block).expect("it compiles");
    let mut host = Scratch::new(true);
    // The first access of each kind fills its entry through the slow path;
    // the rest are hits.
    let _ = engine.run(&block, code, &mut host).expect("it runs");
    let stats = engine.stats();
    assert!(
        stats.fast_loads >= 7,
        "only {} of 8 loads were served inline",
        stats.fast_loads
    );
    assert!(
        stats.fast_stores >= 7,
        "only {} of 8 stores were served inline",
        stats.fast_stores
    );
    // And the inlined stores reported the width they actually wrote, which
    // nothing else checks: a store that claimed eight bytes for one would mark
    // the wrong granules dirty and log the wrong page.
    assert!(
        host.inlined_stores.iter().all(|(_, bytes)| *bytes == 8),
        "an inlined store reported a width it did not write: {:?}",
        host.inlined_stores
    );
}

#[test]
fn a_block_that_faults_reports_the_interpreters_exact_state() {
    // A load past the end of RAM, after a charge, with a temporary live at the
    // boundary: the minimum shape `ROADMAP.md` §9's precise-exception
    // requirement needs. `agree` compares the whole `Fault` — the error, the
    // instruction index, the boundary, both tick columns and the restartable
    // flag — so this is that contract on this host.
    for &addr in &[BASE + RAM, BASE + RAM + 0x800, BASE - 8] {
        let mut b = BlockBuilder::new(BASE, 0);
        let x = b.imm(Type::I64, Const::Int(0x1234));
        b.insn_start(InsnStart {
            pc: BASE,
            next_pc: BASE + 4,
            ticks: 0,
            live: vec![(RegSlot(0), x)],
        });
        b.charge(3);
        let a = b.imm(Type::I64, Const::Int(u128::from(addr)));
        let _ = b.load(Type::I64, a, MemOp::load(Width::U64));
        b.insn_start(InsnStart {
            pc: BASE + 4,
            next_pc: BASE + 8,
            ticks: 3,
            live: vec![(RegSlot(0), x)],
        });
        b.exit_tb();
        let block = b.finish();
        assert!(agree(&block, true), "a faulting block compiles");
        assert!(
            agree(&block, false),
            "and so does it without the inline path"
        );
    }
}
