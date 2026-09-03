//! The backend's own differential: generated code against `ir::Interp`.
//!
//! CLAUDE.md makes a guest's interpreter the oracle for its frontend; one level
//! down, [`Interp`](crate::ir::Interp) is the oracle for every host backend,
//! and it says so in its own module docs. The two guest-level harnesses in
//! `cpu::riscv::differential` and `cpu::x86::differential` cover the compiled
//! path as those guests actually exercise it — which is the test that matters
//! — but they can only reach the ops their frontends emit, and only in the
//! shapes those frontends build.
//!
//! So this is the differential *at the IR*: random blocks over the whole
//! compiled opcode set, run twice against two identical hosts, compared on
//! every temporary, every guest slot, the tick count, guest memory, the
//! boundary count and the outcome. It is what covers the corners a frontend
//! reaches rarely and a fault reaches never — an out-of-range shift, a
//! `bsr` on zero, a signed compare of two `i32`s that are negative, a
//! `movcond` whose selector is a comparison, a load that faults halfway
//! through a block with eight registers live in temporaries.
//!
//! Both hosts here carry a real [`Tlb`], so the inlined fast path is exercised
//! by construction and its answer is compared against the same TLB reached
//! through [`IrHost::load`].

use alloc::collections::BTreeMap;
use alloc::sync::Arc;
use alloc::vec;
use alloc::vec::Vec;

use crate::core::error::BusError;
use crate::core::space::{AddressSpace, MemAttrs, MemResult, RamStore, Region, UnassignedPolicy};
use crate::core::value::Width;
use crate::ir::{
    AccessKind, Block, BlockBuilder, Cond, Const, InsnStart, Interp, IrHost, MemOp, Opcode,
    Outcome, RegSlot, Sign, Temp, Type, bitfield_aux, verify,
};
use crate::jit::{Context, FastMem, LoadPlan, Tlb};

use super::compile::Regs;
use super::rt::Engine;

/// Where the test machine's RAM lives.
const BASE: u64 = 0x2000_0000;
/// Four pages, so an address can miss the mapping and fault.
const RAM: u64 = 4 * 4096;
/// The world an access happens in.
const WORLD: Context = Context {
    level: 3,
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
/// state-only comparison misses: a charge folded into its neighbour, a
/// boundary announced twice, a slot published that should have stayed in a
/// temporary.
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
            // another page's bytes, and a backend that dropped it is caught
            // rather than merely lucky. A large table would give every page an
            // index of its own and hide the whole question.
            tlb: Tlb::with_entries(space, 2),
            inline,
            ticks: 0,
            log: Vec::new(),
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
        // A slot this harness did not seed still answers something specific to
        // its **number**, rather than the same zero every unseeded slot would
        // give. Generated code carries a slot number to the thunk as an
        // immediate, and a zero for every number out of range is exactly the
        // answer that cannot tell a truncated one from an intact one — see
        // `a_slot_number_wider_than_a_byte_reaches_the_host_intact`.
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
        // to the access width. Every host in the tree truncates again on its
        // way to the bus, so a backend that skipped it would look correct
        // everywhere — which is exactly why the contract is checked where it
        // is stated rather than assumed.
        assert_eq!(
            value,
            value & mem.size.mask(),
            "a store reached the host untruncated"
        );
        self.ticks += 1;
        self.tlb
            .write(addr, addr, mem.size, value, WORLD, MemAttrs::DEFAULT)
    }

    fn charge(&mut self, ticks: u64) {
        self.ticks += ticks;
        self.log.push(Event::Charge(ticks));
    }

    fn insn_start(&mut self, mark: &InsnStart) {
        self.log.push(Event::Boundary(mark.pc));
    }
}

impl FastMem for Scratch {
    fn load_plan(&mut self) -> Option<LoadPlan> {
        self.inline.then(|| self.tlb.plan(AccessKind::Load, WORLD))
    }

    fn note_fast_load(&mut self) {
        // Exactly what `load` charges for one access, and nothing else.
        self.ticks += 1;
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
///
/// Both policies, because the register allocator is the thing most able to
/// make a backend wrong in a way only some blocks show:
/// [`Regs::Frame`] is the backend as it stood before it — every temporary in
/// the frame, every operand a load — and it is what says whether a divergence
/// is the allocation or the lowering.
fn agree(block: &Block, inline: bool) -> bool {
    let frame = agree_under(block, inline, Regs::Frame);
    let scan = agree_under(block, inline, Regs::Scan);
    assert_eq!(
        frame, scan,
        "one policy compiled and the other did not\n{block}"
    );
    scan
}

/// One policy's run against the interpreter.
fn agree_under(block: &Block, inline: bool, regs: Regs) -> bool {
    verify(block).expect("the generator produces well-formed blocks");

    let mut oracle_host = Scratch::new(inline);
    let mut interp = Interp::new();
    let oracle = interp.run(block, &mut oracle_host);

    let mut engine = Engine::with_capacity(1 << 18).expect("a code buffer");
    engine.set_regs(regs);
    let code = match engine.compile(block) {
        Ok(c) => c,
        Err(_) => return false,
    };
    let mut host = Scratch::new(inline);
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
    // so rather than handing back the frame's zero — see its documentation.
    // Under `Regs::Frame` nothing is register-allocated, so this is still the
    // whole comparison it used to be on every block the corpus generates.
    let mut compared = 0;
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
    // `ROADMAP.md` §9's precise-exception contract, asserted on every block
    // rather than only on the ones that fault: the exception path materializes
    // architectural state out of the frame, so every temporary a boundary
    // names has to be readable from it whatever the allocator did.
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
/// map that grows, one side exit, a terminator — because a backend bug that
/// only appears in an arrangement no frontend produces is not worth finding
/// before the ones that appear in arrangements they do.
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
        // and it is the only way the commit flag's *clearing* becomes visible —
        // a charge commits, so a boundary followed by one is committed however
        // the boundary left the flag.
        if r.below(4) != 0 {
            let charge = 1 + r.below(3);
            b.charge(charge);
            ticks += charge;
        }
        pc += 4;

        if side_exit_at == Some(step) {
            side_exit_at = None;
            // A superblock's side exit: a `brcond` that jumps *over* an inline
            // exit sequence, which is the shape `cpu::riscv::lift` emits.
            // A condition that is constant *at run time* but opaque to the
            // builder: `x < x` never holds and `x == x` always does. Which one
            // decides whether this block leaves through its side exit or runs
            // on to its own terminator — and taking the exit every time was a
            // real defect in this generator, because it made the whole tail of
            // every block, its three terminators included, unreachable.
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
///
/// Type-correct on purpose. The IR permits an instruction's type to differ
/// from its operands' and `ir::Interp` then computes at the *operand's* width
/// — see `Compiler::src_typed` — so a generator that mixed them would spend
/// its time on blocks no frontend emits and that the backend refuses.
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

    match r.below(22) {
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
            // agree about: x86 masks the count and would quietly shift by zero,
            // where the interpreter takes the mathematical answer, which is why
            // this backend emits the compare (see `compile`'s module docs).
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
            let op = r.pick(&[Opcode::ROTL, Opcode::ROTR]);
            let (x, y) = (any(r), any(r));
            vec![(b.binary(op, ty, x, y), ty)]
        }
        5 => {
            let op = r.pick(&[Opcode::CLZ, Opcode::CTZ, Opcode::POPCOUNT]);
            // Zero often, because the zero input is the case `bsr` leaves
            // undefined and the one a lowering gets wrong.
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
            // Whole-type and narrow-lane both: ARM's `REV16` and x86's 16-bit
            // swap are lanes inside the type, and they lower differently.
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
            let op = r.pick(&[Opcode::MULU2, Opcode::MULS2]);
            let low = b.temp(ty);
            let high = b.temp(ty);
            let (x, y) = (any(r), any(r));
            b.emit_raw(op, ty, Some(low), Some(high), &[x, y], None, None, 0);
            vec![(low, ty), (high, ty)]
        }
        14 => {
            let op = r.pick(&[Opcode::ROTLC, Opcode::ROTRC]);
            let value = b.temp(ty);
            let carry_out = b.temp(Type::I1);
            let x = any(r);
            // The carry in is one bit by construction, which the verifier
            // insists on and which is what makes the (N+1)-bit rotate one.
            let c = b.imm(Type::I1, Const::Int(u128::from(r.below(2))));
            b.emit_raw(op, ty, Some(value), Some(carry_out), &[x, c], None, None, 0);
            vec![(c, Type::I1), (value, ty), (carry_out, Type::I1)]
        }
        15 => {
            let slot = r.below(u64::from(SLOTS)) as u16;
            vec![(b.get_slot(Type::I64, RegSlot(slot)), Type::I64)]
        }
        16 | 17 => {
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
        18 => {
            let size = r.pick(&WIDTHS);
            let addr = address(b, r, size);
            let value = any(r);
            let mut mem = MemOp::store(size);
            mem.volatile = r.below(2) == 0;
            b.store(Type::I64, addr, value, mem);
            Vec::new()
        }
        19 => {
            // A destination *narrower* than the operation. Not exotic: it is
            // the shape `setcond` has by construction, and it is the one that
            // needs both of the interpreter's masks rather than one.
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
fn the_call_map_the_allocator_is_given_matches_what_the_lowerings_emit() {
    // The one input `linear_scan` cannot check for itself. If `calls_out` says
    // an op makes no call and its lowering does, a value in a volatile
    // register is destroyed by the callee and the block computes with
    // whatever the host left there — which no test that does not happen to
    // land that temporary in `r8` will show.
    //
    // So this counts the `call [r14 + disp32]` sequences in the emitted bytes
    // and compares them against what the map claims, on a block whose
    // immediates are chosen so that none of them can spell that sequence by
    // accident.
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    b.charge(1);
    let x = b.get_slot(Type::I64, RegSlot(1));
    let one = b.imm(Type::I64, Const::Int(1));
    let sum = b.binary(Opcode::ADD, Type::I64, x, one);
    let shifted = b.binary(Opcode::SHL, Type::I64, sum, one);
    let _ = b.unary(Opcode::POPCOUNT, Type::I64, shifted);
    let addr = b.imm(Type::I64, Const::Int(u128::from(BASE + 64)));
    // One load the backend inlines — two calls, the fast path's tick and the
    // slow path's own — and one store, which is always a call.
    let value = b.load(Type::I64, addr, MemOp::load(Width::U64));
    b.store(Type::I64, addr, value, MemOp::store(Width::U64));
    b.insn_start(InsnStart {
        pc: BASE + 4,
        next_pc: BASE + 8,
        ticks: 1,
        live: vec![(RegSlot(0), sum)],
    });
    b.exit_tb();
    let block = b.finish();
    verify(&block).expect("well formed");

    // `41 ff 96 disp32` is `call [r14 + disp32]`, and it is the only call this
    // backend emits.
    fn calls(code: &[u8]) -> usize {
        code.windows(3).filter(|w| *w == [0x41, 0xff, 0x96]).count()
    }
    let mut want = 0usize;
    for inst in block.insts() {
        want += match inst.op {
            Opcode::GET_SLOT | Opcode::ST => 1,
            // A load carries the slow path's call whatever happens, and the
            // inlined probe adds `note_fast_load` on top of it.
            Opcode::LD => 2,
            // A charge and a boundary emit nothing at all: their work is the
            // region's flush, counted below.
            _ => 0,
        };
    }
    want += a_flush_before(&block).iter().filter(|f| **f).count();
    for regs in [Regs::Frame, Regs::Scan] {
        let compiled = super::compile::compile_with(&block, regs).expect("compiles");
        assert_eq!(
            calls(compiled.code()),
            want,
            "the emitted calls and the map disagree under {regs:?}"
        );
    }
}

/// Whether the backend's lowering of `op` calls into the host *after* reading
/// its operands and before writing its results.
///
/// Written out here rather than read off `compile::calls_inside`, and the
/// duplication is the point: `the_call_map_the_allocator_is_given_matches_what_
/// the_lowerings_emit` checks this against the *emitted bytes*, and the test
/// below checks the allocator was given it. A single shared constant would
/// make both of them agree with a wrong answer.
fn a_call_site(op: Opcode) -> bool {
    matches!(op, Opcode::LD | Opcode::ST | Opcode::GET_SLOT)
}

/// Where the backend replays a region's deferred bookkeeping, worked out again
/// rather than read off `compile::plan_flushes`.
///
/// Deliberately the *other* formulation: `plan_flushes` carries the region
/// start forward in one pass, and this searches backwards for it per
/// instruction. The two agreeing is evidence; one of them reading the other
/// would not be.
fn a_flush_before(block: &Block) -> Vec<bool> {
    let insts = block.insts();
    let n = insts.len();
    let mut is_target = vec![false; n];
    for inst in insts {
        if inst.op == Opcode::BRCOND && (inst.aux as usize) < n {
            is_target[inst.aux as usize] = true;
        }
    }
    let boundary = |i: usize| {
        is_target[i]
            || a_call_site(insts[i].op)
            || insts[i].op == Opcode::BRCOND
            || insts[i].op.is_terminator()
    };
    (0..n)
        .map(|i| {
            if !boundary(i) {
                return false;
            }
            // The region runs from the previous boundary — inclusive, because
            // a boundary starts the next region at itself — up to here.
            let from = (0..i).rev().find(|j| boundary(*j)).unwrap_or(0);
            (from..i).any(|j| matches!(insts[j].op, Opcode::CHARGE | Opcode::INSN_START))
        })
        .collect()
}

/// Assert the allocator's central invariant over one block.
fn no_volatile_register_spans_a_call(block: &Block) {
    let live = crate::ir::Liveness::compute(block);
    let compiled = super::compile::compile_with(block, Regs::Scan).expect("compiles");
    let calls: Vec<bool> = block.insts().iter().map(|i| a_call_site(i.op)).collect();
    let flushes = a_flush_before(block);
    for (temp, lo, hi) in live.intervals() {
        let crate::ir::Home::Reg(r) = compiled.home(temp) else {
            continue;
        };
        // Strictly between for a call *inside* an instruction: an operand is
        // read before it and a result is written after it. **Up to and
        // including** for a flush, which runs in the gap ahead of an
        // instruction and therefore before it has read anything — the one
        // asymmetry `ir::CallSites` exists for.
        let inside = ((lo as usize + 1)..(hi as usize)).any(|i| calls[i]);
        let gap = ((lo as usize + 1)..=(hi as usize)).any(|i| flushes.get(i) == Some(&true));
        if inside || gap {
            assert!(
                super::compile::SAVED.contains(&r),
                "{temp} is live across a call in r{r}, which is not callee-saved\n{block}"
            );
        }
        assert!(
            super::compile::SAVED.contains(&r) || super::compile::VOLATILE.contains(&r),
            "{temp} took r{r}, which is not a register the backend hands out"
        );
    }
}

#[test]
fn a_value_read_at_the_instruction_a_flush_runs_ahead_of_needs_a_saved_register() {
    // The shape that separates `CallSites`' two arrays, and the one that would
    // be silently miscompiled if the flush were recorded as a call *inside*
    // the instruction it precedes. `addr` is defined at one instruction and
    // read by the next, and the region's flush runs in the gap between them —
    // before the load has read its address into an argument register. As an
    // `inside` call it would not cross an interval that ends there, so a
    // volatile register would look safe and `flush_thunk` would destroy it.
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    b.charge(2);
    let addr = b.imm(Type::I64, Const::Int(u128::from(BASE + 64)));
    let value = b.load(Type::I64, addr, MemOp::load(Width::U64));
    b.insn_start(InsnStart {
        pc: BASE + 4,
        next_pc: BASE + 8,
        ticks: 2,
        live: vec![(RegSlot(0), value)],
    });
    b.exit_tb();
    let block = b.finish();
    verify(&block).expect("well formed");

    // Non-vacuity: the load really is the flush point, and `addr` really does
    // end its life there.
    let flushes = a_flush_before(&block);
    assert!(flushes[3], "the load is where the region is replayed");
    let live = crate::ir::Liveness::compute(&block);
    let life = live.life(addr).expect("addr is live");
    assert_eq!((life.def, life.last_use), (Some(2), Some(3)));

    no_volatile_register_spans_a_call(&block);
    let compiled = super::compile::compile_with(&block, Regs::Scan).expect("compiles");
    if let crate::ir::Home::Reg(r) = compiled.home(addr) {
        assert!(
            super::compile::SAVED.contains(&r),
            "the address took volatile r{r} across the flush"
        );
    }
    assert!(agree(&block, true));
}

#[test]
fn a_charge_a_branch_jumps_over_is_never_charged() {
    // The reason a `brcond` is a region boundary. The skipped range holds a
    // charge and a boundary, and the flush at the branch target replays the
    // range the *fall-through* executed — so a taken branch must arrive after
    // that flush, with nothing of its own pending. Getting this wrong charges
    // ticks for a guest instruction that did not run, which `ROADMAP.md` §0
    // makes a state-hash divergence.
    for take in [false, true] {
        let mut b = BlockBuilder::new(BASE, 0);
        b.insn_start(InsnStart {
            pc: BASE,
            next_pc: BASE + 4,
            ticks: 0,
            live: Vec::new(),
        });
        b.charge(5);
        // `x < x` never holds; `x == x` always does.
        let x = b.imm(Type::I64, Const::Int(3));
        let cond = if take { Cond::Eq } else { Cond::LtU };
        let sel = b.setcond(cond, Type::I64, x, x);
        let over = b.emit_raw(Opcode::BRCOND, Type::I64, None, None, &[sel], None, None, 0);
        // The skipped range: a whole guest instruction's worth of bookkeeping.
        b.insn_start(InsnStart {
            pc: BASE + 4,
            next_pc: BASE + 8,
            ticks: 5,
            live: Vec::new(),
        });
        b.charge(9);
        b.patch_aux(over, b.next_index() as u32);
        b.insn_start(InsnStart {
            pc: BASE + 8,
            next_pc: BASE + 12,
            ticks: if take { 5 } else { 14 },
            live: vec![(RegSlot(0), x)],
        });
        b.charge(1);
        b.exit_tb();
        let block = b.finish();
        verify(&block).expect("well formed");
        assert!(agree(&block, true), "{block}");

        // and the number itself, so the assertion is not only "both engines
        // agree" but "they agree on the right answer".
        let mut host = Scratch::new(true);
        let mut interp = Interp::new();
        interp.run(&block, &mut host).expect("runs");
        assert_eq!(interp.ticks(), if take { 6 } else { 15 });
    }
}

#[test]
fn a_malformed_charge_or_boundary_is_refused_rather_than_replayed_wrong() {
    // The two shapes the deferred bookkeeping cannot represent. `plan` refuses
    // them rather than defaulting, because `ir::Interp` reports each as an
    // **error** — so a compiled block that charged zero, or that skipped a
    // boundary it could not resolve, would be a different answer rather than a
    // slower one. Nothing else in this file builds either shape: the builder's
    // `charge` and `insn_start` cannot, and the generator uses them.
    let mut b = BlockBuilder::new(BASE, 0);
    b.emit_raw(Opcode::CHARGE, Type::I64, None, None, &[], None, None, 0);
    b.exit_tb();
    let no_count = b.finish();

    let mut b = BlockBuilder::new(BASE, 0);
    b.emit_raw(
        Opcode::INSN_START,
        Type::I64,
        None,
        None,
        &[],
        None,
        None,
        7,
    );
    b.exit_tb();
    let no_record = b.finish();

    for (what, block) in [
        ("a charge with no count", &no_count),
        ("a boundary marker with no record", &no_record),
    ] {
        for regs in [Regs::Frame, Regs::Scan] {
            assert!(
                matches!(
                    super::compile::compile_with(block, regs),
                    Err(super::compile::Refusal::Shape(_))
                ),
                "{what} compiled under {regs:?}"
            );
        }
        // and the interpreter, which is where a refused block goes, really
        // does call it an error rather than running it.
        let mut host = Scratch::new(true);
        Interp::new().run(block, &mut host).expect_err(what);
    }
}

#[test]
fn a_slot_number_wider_than_a_byte_reaches_the_host_intact() {
    // The generator's slot space is eight wide, so nothing else in this file
    // can tell a slot number masked to sixteen bits from one masked to eight —
    // and generated code carries that number to the thunk as an immediate it
    // has to narrow *somewhere*.
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    b.charge(1);
    let wide = b.get_slot(Type::I64, RegSlot(0x1234));
    let narrow = b.get_slot(Type::I64, RegSlot(0x34));
    let sum = b.binary(Opcode::SUB, Type::I64, wide, narrow);
    b.insn_start(InsnStart {
        pc: BASE + 4,
        next_pc: BASE + 8,
        ticks: 1,
        live: vec![(RegSlot(0), sum), (RegSlot(1), wide)],
    });
    b.exit_tb();
    let block = b.finish();
    verify(&block).expect("well formed");
    // Non-vacuity: the two slots really do answer differently, so a backend
    // that truncated `0x1234` to `0x34` would read the wrong one.
    let mut host = Scratch::new(true);
    assert_ne!(
        host.read_slot(RegSlot(0x1234)),
        host.read_slot(RegSlot(0x34))
    );
    assert!(agree(&block, true));
}

#[test]
fn a_fault_reports_the_ticks_that_were_charged_not_the_column_the_frontend_wrote() {
    // `ir`'s "Known gaps" records that nothing checks `InsnStart::ticks`
    // against the charges before it, and `Interp` answers a fault with what it
    // **charged**. Every block this file's generator builds has an honest
    // column, so the two are indistinguishable there; this one lies, and both
    // engines have to be wrong in the same way.
    let mut b = BlockBuilder::new(BASE, 0);
    let x = b.imm(Type::I64, Const::Int(0x1234_5678));
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        // The lie: nothing has been charged yet.
        ticks: 0x9999,
        live: vec![(RegSlot(0), x)],
    });
    b.charge(2);
    let a = b.imm(Type::I64, Const::Int(u128::from(BASE + RAM + 0x800)));
    let _ = b.load(Type::I64, a, MemOp::load(Width::U64));
    b.insn_start(InsnStart {
        pc: BASE + 4,
        next_pc: BASE + 8,
        ticks: 0x9999,
        live: vec![(RegSlot(0), x)],
    });
    b.exit_tb();
    let block = b.finish();
    verify(&block).expect("well formed");
    assert!(agree(&block, true));

    let mut engine = Engine::with_capacity(1 << 16).expect("a code buffer");
    let code = engine.compile(&block).expect("compiles");
    let mut host = Scratch::new(true);
    let out = engine
        .run(&block, code, &mut host)
        .expect("live")
        .expect("no error");
    let Outcome::Fault(fault) = out else {
        panic!("that address is off the end of RAM: {out:?}");
    };
    // Zero, because the boundary came before the charge — not `0x9999`.
    assert_eq!(fault.retired_ticks, 0);
    assert_eq!(fault.charged_ticks, 2);
}

#[test]
fn a_guest_instructions_bookkeeping_costs_no_code_at_all() {
    // The non-vacuity assertion for the whole deferral: a boundary and a
    // charge used to be nineteen instructions and two calls, and are now
    // nothing. So a region's code size must not depend on how many guest
    // instructions are in it — which is a property no differential can show,
    // because a backend that emitted all nineteen again would still be right.
    fn bytes(insns: u64) -> usize {
        let mut b = BlockBuilder::new(BASE, 0);
        for i in 0..insns {
            b.insn_start(InsnStart {
                pc: BASE + i * 4,
                next_pc: BASE + (i + 1) * 4,
                ticks: i,
                live: Vec::new(),
            });
            b.charge(1);
        }
        b.exit_tb();
        let block = b.finish();
        super::compile::compile_with(&block, Regs::Scan)
            .expect("compiles")
            .code()
            .len()
    }
    let one = bytes(1);
    assert_eq!(one, bytes(8));
    assert_eq!(one, bytes(64));
}

#[test]
fn no_value_that_outlives_a_call_is_left_where_a_call_can_destroy_it() {
    // The allocator's central invariant, checked structurally, because it is
    // exactly the one a differential cannot be relied on to show: whether a
    // value in `r10` survives `IrHost::charge` depends on what the compiler
    // did to `charge`, not on what this backend did. A run that happens to
    // pass is not evidence.
    for seed in 0..200u64 {
        no_volatile_register_spans_a_call(&random_block(seed ^ 0x5eed, 20));
    }
}

#[test]
fn a_block_with_more_live_values_than_registers_still_agrees() {
    // Seven host registers against blocks that keep dozens of values alive:
    // this is where the spill path, the steal heuristic and both banks
    // running dry are actually exercised. The shorter corpus above almost
    // never runs out.
    let mut spilled = 0usize;
    let mut total = 0usize;
    for seed in 0..120u64 {
        let block = random_block(seed ^ 0x0fu64, 40);
        assert!(agree(&block, true));
        let compiled = super::compile::compile_with(&block, Regs::Scan).expect("compiles");
        for t in 0..block.temp_count() {
            total += 1;
            if compiled.home(Temp(t as u32)) == crate::ir::Home::Frame {
                spilled += 1;
            }
        }
    }
    assert!(
        spilled * 4 > total,
        "only {spilled} of {total} temporaries spilled; the pressure is not real"
    );
}

#[test]
fn a_value_read_before_it_is_assigned_reads_the_frame_the_interpreter_reads() {
    // `verify` rejects this block, and `compile` is public and the dispatcher
    // does not verify — so the backend has to answer, and the only answer that
    // agrees with the oracle is the frame's zero. A register would hold
    // whatever the last temporary to own it left there.
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    b.charge(1);
    let later = b.temp(Type::I64);
    let seen = b.binary(Opcode::ADD, Type::I64, later, later);
    // Assign it afterwards, so it is a temporary with a definition that comes
    // too late rather than one with none at all.
    let seven = b.imm(Type::I64, Const::Int(7));
    b.emit_raw(
        Opcode::MOV,
        Type::I64,
        Some(later),
        None,
        &[seven],
        None,
        None,
        0,
    );
    b.insn_start(InsnStart {
        pc: BASE + 4,
        next_pc: BASE + 8,
        ticks: 1,
        live: vec![(RegSlot(0), seen), (RegSlot(1), later)],
    });
    b.exit_tb();
    let block = b.finish();
    verify(&block).expect_err("the verifier is the real answer to this shape");

    let mut oracle = Scratch::new(true);
    let mut interp = Interp::new();
    interp
        .run(&block, &mut oracle)
        .expect("the interpreter runs it");

    let mut engine = Engine::with_capacity(1 << 16).expect("a code buffer");
    let code = engine.compile(&block).expect("compiles");
    let mut host = Scratch::new(true);
    engine
        .run(&block, code, &mut host)
        .expect("live")
        .expect("ok");
    assert_eq!(oracle.slots, host.slots, "{block}");
}

#[test]
fn the_allocator_actually_places_values_in_registers() {
    // Non-vacuity, and it is the assertion the rest of this file rests on: the
    // differential compares the temporaries `temp_value` hands back, so a
    // corpus where the allocator quietly gave up would still pass every other
    // test in here while measuring nothing.
    let mut in_regs = 0usize;
    let mut total = 0usize;
    let mut written_through = 0usize;
    for seed in 0..200u64 {
        let block = random_block(seed, 12);
        let compiled = super::compile::compile_with(&block, Regs::Scan).expect("compiles");
        in_regs += compiled.in_registers();
        total += block.temp_count();
        written_through += (0..block.temp_count())
            .filter(|t| compiled.frame_backed(Temp(*t as u32)))
            .count();
    }
    assert!(
        in_regs * 3 > total,
        "only {in_regs} of {total} temporaries got a register"
    );
    // and some of them still reach the frame, because a boundary names them:
    // if none did, the precise-state assertion above would be vacuous too.
    assert!(written_through > 0 && written_through < total);

    // The control really is the control.
    for seed in 0..20u64 {
        let block = random_block(seed, 12);
        let compiled = super::compile::compile_with(&block, Regs::Frame).expect("compiles");
        assert_eq!(compiled.in_registers(), 0);
    }
}

#[test]
fn a_thousand_random_blocks_agree_with_the_interpreter() {
    let mut compiled = 0;
    for seed in 0..1000u64 {
        let block = random_block(seed, 6 + (seed % 11) as usize);
        if agree(&block, true) {
            compiled += 1;
        }
    }
    assert_eq!(
        compiled, 1000,
        "every generated block is within the backend"
    );
}

#[test]
fn the_inlined_fast_path_answers_what_the_call_answers() {
    // The control for the TLB inlining: the same blocks with the fast path
    // switched off must reach the same everything. A backend whose inlined
    // probe disagreed with the host's own `load` would otherwise show up only
    // where the two happened to be compared.
    for seed in 0..300u64 {
        let block = random_block(seed ^ 0xdead, 8);
        assert!(agree(&block, false));
        assert!(agree(&block, true));
    }
}

#[test]
fn a_block_that_faults_reports_the_interpreters_exact_state() {
    // A fault in the middle of a long block, with a dozen temporaries live and
    // guest state never written back: `ROADMAP.md` §9's hard half. `agree`
    // compares `Fault` field for field, including the mark, the boundary PC,
    // the retired and charged tick columns and `restartable`.
    let mut faulted = 0;
    for seed in 0..400u64 {
        let block = random_block(seed ^ 0xf417, 12);
        agree(&block, true);
        let mut host = Scratch::new(true);
        let mut interp = Interp::new();
        if matches!(interp.run(&block, &mut host), Ok(Outcome::Fault(_))) {
            faulted += 1;
        }
    }
    assert!(
        faulted > 20,
        "only {faulted} of 400 generated blocks faulted; the fault path is barely covered"
    );
}

/// A block that loads from `addr` after charging, with `x` live at the
/// boundary — the minimum shape a precise-state test needs.
fn faulting_block(addr: u64) -> Block {
    let mut b = BlockBuilder::new(BASE, 0);
    let x = b.imm(Type::I64, Const::Int(0x1234_5678));
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: vec![(RegSlot(0), x)],
    });
    b.charge(2);
    let a = b.imm(Type::I64, Const::Int(u128::from(addr)));
    let mut mem = MemOp::load(Width::U64);
    mem.volatile = false;
    let _ = b.load(Type::I64, a, mem);
    b.insn_start(InsnStart {
        pc: BASE + 4,
        next_pc: BASE + 8,
        ticks: 2,
        live: vec![(RegSlot(0), x)],
    });
    b.exit_tb();
    b.finish()
}

#[test]
fn a_fault_carries_the_boundary_it_started_at_and_the_ticks_that_retired() {
    let block = faulting_block(BASE + RAM + 0x800);
    agree(&block, true);

    let mut engine = Engine::with_capacity(1 << 16).expect("a code buffer");
    let code = engine.compile(&block).expect("compiles");
    let mut host = Scratch::new(true);
    let out = engine
        .run(&block, code, &mut host)
        .expect("live")
        .expect("no error");
    let Outcome::Fault(fault) = out else {
        panic!("that address is off the end of RAM: {out:?}");
    };
    assert_eq!(fault.pc, BASE, "the faulting instruction's own boundary");
    // Retired is the count *as of the boundary*, which is before this guest
    // instruction's own charge; charged is what had actually been spent when
    // the access failed. `ROADMAP.md` §9 makes restart-versus-resume a
    // per-architecture policy, so both numbers are reported and neither is
    // reconciled here — and the two engines have to agree about both.
    assert_eq!(fault.retired_ticks, 0);
    assert_eq!(fault.charged_ticks, 2);
    // Not restartable, and that is `Interp`'s answer too: a `charge` commits,
    // because a tick the guest has been billed for cannot be un-billed by
    // re-running the instruction that spent it.
    assert!(!fault.restartable);
    // and the register that was live in a temporary reached guest state.
    assert_eq!(host.slots.get(&0).copied(), Some(0x1234_5678));
}

#[test]
fn a_retry_after_a_commit_is_rejected_rather_than_delivered() {
    // `Interp` refuses a `Retry` once the guest instruction has changed
    // something the world can see, because a retry would re-run the committed
    // half. The compiled path has to refuse it in the same place.
    struct Retrying {
        committed: bool,
    }
    impl IrHost for Retrying {
        fn read_slot(&mut self, _slot: RegSlot) -> u128 {
            0
        }
        fn write_slot(&mut self, _slot: RegSlot, _value: u128) {}
        fn load(&mut self, _mem: &MemOp, _addr: u64) -> MemResult<u64> {
            Err(BusError::Retry)
        }
        fn store(&mut self, _mem: &MemOp, _addr: u64, _value: u64) -> MemResult {
            self.committed = true;
            Ok(())
        }
        fn charge(&mut self, _ticks: u64) {}
        fn insn_start(&mut self, _mark: &InsnStart) {}
    }
    impl FastMem for Retrying {}

    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    // No charge: a charge commits, and this case is about the *store*
    // committing. With one here the mutation "a store does not commit"
    // survives, which is how this line came to be written down.
    let a = b.imm(Type::I64, Const::Int(u128::from(BASE)));
    let v = b.imm(Type::I64, Const::Int(0));
    b.store(Type::I64, a, v, MemOp::store(Width::U8));
    let _ = b.load(Type::I64, a, MemOp::load(Width::U8));
    b.insn_start(InsnStart {
        pc: BASE + 4,
        next_pc: BASE + 8,
        ticks: 0,
        live: Vec::new(),
    });
    b.exit_tb();
    let block = b.finish();

    let mut engine = Engine::with_capacity(1 << 16).expect("a code buffer");
    let code = engine.compile(&block).expect("compiles");
    let mut host = Retrying { committed: false };
    let got = engine.run(&block, code, &mut host).expect("live");
    assert!(host.committed);
    assert!(
        matches!(got, Err(crate::core::error::Error::Bus(BusError::Retry))),
        "{got:?}"
    );

    // and the oracle says the same thing about the same block.
    let mut interp = Interp::new();
    let mut oracle = Retrying { committed: false };
    let want = interp.run(&block, &mut oracle);
    assert!(matches!(
        want,
        Err(crate::core::error::Error::Bus(BusError::Retry))
    ));
}

#[test]
fn an_op_the_backend_does_not_lower_is_refused_and_not_miscompiled() {
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    let x = b.imm(Type::I64, Const::Int(3));
    let y = b.imm(Type::I64, Const::Int(4));
    let _ = b.binary(Opcode::DIV_U, Type::I64, x, y);
    b.exit_tb();
    let block = b.finish();
    let mut engine = Engine::with_capacity(1 << 16).expect("a code buffer");
    assert_eq!(
        engine.compile(&block),
        Err(super::Refusal::Op(Opcode::DIV_U))
    );
    assert_eq!(engine.stats().refused, 1);
}

#[test]
fn a_destination_wider_than_its_operation_is_refused() {
    // The IR permits it and the verifier does not check it, and the two masks
    // `Interp` applies then stop composing into one. Refused rather than
    // guessed; the interpreter runs the block, and every other backend-level
    // claim is unaffected.
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    let x = b.imm(Type::I64, Const::Int(0xffff_ffff_ffff_ffff));
    let wide = b.temp(Type::I64);
    b.emit_raw(
        Opcode::ADD,
        Type::I32,
        Some(wide),
        None,
        &[x, x],
        None,
        None,
        0,
    );
    b.exit_tb();
    let block = b.finish();
    verify(&block).expect("the verifier permits it, which is the point");
    let mut engine = Engine::with_capacity(1 << 16).expect("a code buffer");
    assert!(matches!(
        engine.compile(&block),
        Err(super::Refusal::Shape(_))
    ));
}

#[test]
fn a_backward_branch_is_refused_because_nothing_would_stop_it() {
    // `Interp` has a step limit; generated code does not, so a block the
    // verifier would reject must be refused here rather than run.
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    let sel = b.imm(Type::I1, Const::Int(1));
    b.emit_raw(Opcode::BRCOND, Type::I64, None, None, &[sel], None, None, 0);
    b.exit_tb();
    let block = b.finish();
    let mut engine = Engine::with_capacity(1 << 16).expect("a code buffer");
    assert!(matches!(
        engine.compile(&block),
        Err(super::Refusal::Shape(_))
    ));
}

#[test]
fn a_full_code_buffer_recompiles_rather_than_failing() {
    // The buffer's whole invalidation story, end to end: fill it, watch the
    // reset, and check that a handle from before it is refused rather than
    // followed into whatever took its place.
    let block = random_block(7, 8);
    let mut engine = Engine::with_capacity(4096).expect("a code buffer");
    let first = engine.compile(&block).expect("compiles");
    let mut last = first;
    for _ in 0..64 {
        last = engine.compile(&block).expect("compiles");
    }
    assert!(engine.stats().resets > 0, "the buffer never filled");
    assert!(
        !engine.is_live(first),
        "a handle from before a reset is dead"
    );
    assert!(engine.is_live(last));
    let mut host = Scratch::new(true);
    assert!(engine.run(&block, first, &mut host).is_none());
    assert!(engine.run(&block, last, &mut host).is_some());
}

#[test]
fn the_inlined_probe_really_serves_the_loads() {
    // A benchmark, and a table, and a claim about a TLB — with nothing
    // asserting the fast path was ever taken. This is that assertion.
    let mut b = BlockBuilder::new(BASE, 0);
    b.insn_start(InsnStart {
        pc: BASE,
        next_pc: BASE + 4,
        ticks: 0,
        live: Vec::new(),
    });
    b.charge(1);
    let addr = b.imm(Type::I64, Const::Int(u128::from(BASE + 64)));
    for _ in 0..16 {
        let _ = b.load(Type::I64, addr, MemOp::load(Width::U64));
    }
    b.insn_start(InsnStart {
        pc: BASE + 4,
        next_pc: BASE + 8,
        ticks: 1,
        live: Vec::new(),
    });
    b.exit_tb();
    let block = b.finish();

    let mut engine = Engine::with_capacity(1 << 16).expect("a code buffer");
    let code = engine.compile(&block).expect("compiles");
    let mut host = Scratch::new(true);
    engine
        .run(&block, code, &mut host)
        .expect("live")
        .expect("ok");
    // The first load misses and fills through the host; the rest are inlined.
    assert_eq!(engine.stats().fast_loads, 15, "{:?}", host.tlb.stats());
    assert_eq!(host.ticks, 17, "one charge and sixteen accesses");

    // With the plan withheld, not one of them is.
    let mut engine = Engine::with_capacity(1 << 16).expect("a code buffer");
    let code = engine.compile(&block).expect("compiles");
    let mut host = Scratch::new(false);
    engine
        .run(&block, code, &mut host)
        .expect("live")
        .expect("ok");
    assert_eq!(engine.stats().fast_loads, 0);
    assert_eq!(host.ticks, 17, "and the ticks are the same either way");
}
