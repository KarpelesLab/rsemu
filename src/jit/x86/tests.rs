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
        self.slots.get(&slot.0).copied().unwrap_or(0)
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
        self.inline.then(|| LoadPlan {
            set: self.tlb.fast_set(AccessKind::Load),
            ctx: WORLD,
        })
    }

    fn note_fast_load(&mut self) {
        // Exactly what `load` charges for one access, and nothing else.
        self.ticks += 1;
    }
}

// ---------------------------------------------------------------------------
// The comparison
// ---------------------------------------------------------------------------

/// Run `block` on both engines and assert they agreed about everything.
///
/// Returns whether the compiled run happened at all, so a caller can tell a
/// clean agreement from a block the backend refused.
fn agree(block: &Block, inline: bool) -> bool {
    verify(block).expect("the generator produces well-formed blocks");

    let mut oracle_host = Scratch::new(inline);
    let mut interp = Interp::new();
    let oracle = interp.run(block, &mut oracle_host);

    let mut engine = Engine::with_capacity(1 << 18).expect("a code buffer");
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

    for t in 0..block.temp_count() {
        let temp = Temp(t as u32);
        let want = interp.temp_value(temp).expect("allocated");
        let got = u128::from(engine.temp_value(temp).expect("allocated"));
        assert_eq!(want, got, "temporary {temp} differs\n{block}");
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
