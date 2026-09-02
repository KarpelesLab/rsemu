#![no_main]
//! The IR verifier, and the optimiser pass that runs behind it.
//!
//! `CLAUDE.md` asks for a fuzz target on every parser. `ir::verify` is one:
//! it reads a [`Block`] a *frontend* built, and every frontend is a program
//! that turns attacker-controlled guest bytes into IR. A lifter with a bug —
//! an operand read from the wrong slot, a temporary numbered off the end of
//! its table, a carry that came out `i32` — hands the verifier a block no test
//! ever wrote by hand, and the verifier's whole job is to answer rather than
//! to crash. It runs between the frontend and a backend precisely so a defect
//! is *named* here instead of miscompiled there.
//!
//! Then it drives dead-code elimination over the same block, because the pass
//! and the verifier make a claim about each other that only a fuzzer can
//! check across arbitrary shapes:
//!
//! > **A block the verifier accepts survives elimination and still verifies.**
//! > Elimination never invents an instruction, never touches a boundary
//! > record, never removes an effect, and is a fixed point after one run.
//!
//! That is the property that matters, because the pass exists to delete
//! things: the flags-as-temporaries decision (`src/ir/mod.rs`) is a debt until
//! DCE removes the parity and half-carry temporaries nothing reads, and a pass
//! that also removes a store, a charge, a dummy read or the temporary a
//! boundary names is a *silent* miscompile — no crash, a guest that diverges
//! a million cycles later.
//!
//! Elimination is driven over **every** generated block, malformed ones
//! included, and asserted to terminate without panicking there too: a pass is
//! ordinarily run after the verifier, but nothing in the type system says so,
//! and a pass that panics on a block the verifier would have rejected is a
//! pass that panics the day someone reorders the pipeline.
//!
//! # Input encoding
//!
//! A five-byte header, then a stream of one-byte selectors each followed by
//! its own operands. Decoded by hand rather than through `arbitrary`'s derive,
//! for the reason `state_roundtrip` gives: a dependency bump must not
//! reinterpret every committed seed.
//!
//! ```text
//!   header  pp pp pp pp kk       entry PC, cache key
//!   0x00 pppp pppp tt nn (ss tt)*  insn_start: PC, ticks, nn live (slot, temp)
//!   0x01 kk                      charge kk ticks
//!   0x02 ty vv                   an immediate
//!   0x03 op a b                  a binary op
//!   0x04 op a                    a unary op
//!   0x05 op a b c                add/subtract with carry
//!   0x06 cc ty a b               setcond
//!   0x07 mm ty a                 a load, mm choosing the descriptor
//!   0x08 mm ty a v               a store
//!   0x09 a                       brcond
//!   0x0a ty a                    a helper call with one argument
//!   0x0b ty                      allocate a temporary and never assign it
//!   0x0c ty a                    a load with no descriptor: malformed
//!   0x0d ty a b                  an atomic read-modify-write
//!   0x0e                         exit_tb, and stop
//!   0x0f                         goto_tb, and carry on past it
//! ```
//!
//! Every accessor is total and every byte decodes into *something*, so an
//! arbitrary input is an arbitrary block rather than a rejection: there is no
//! file format to rediscover and the mutator is productive from the first
//! byte. The committed seeds are the shapes worth starting *from* rather than
//! a grammar to learn — the parity flag with and without a boundary naming it,
//! a dummy read beside a plain load, and one block that is malformed on
//! purpose.
//!
//! # Biased towards blocks that verify, not away from the rejections
//!
//! A uniformly random block is rejected by the first rule it meets, and a
//! target where nothing verifies would test the verifier's error paths
//! thoroughly and its agreement with the optimiser not at all. Three biases
//! fix that, each of them one rule the fuzzer would otherwise re-derive on
//! every input: a boundary is opened before anything else, so a charge is
//! attributable; a carry in is usually a fresh `i1`; and reaching a terminator
//! normally ends the block. Every one of them is still reachable in the wrong
//! direction — `0x0f` walks past a terminator, `0x05` takes a wrongly typed
//! carry a quarter of the time, and extra boundaries and charges come from the
//! stream — so the rejection arms stay live.
//!
//! Temporary numbers follow the same shape: a temporary something has already
//! assigned, unless the byte is at least `0xf0`, above which it names one that
//! was allocated and never assigned, and above `0xf8` one that was never
//! allocated at all.

use libfuzzer_sys::fuzz_target;

use rsemu::core::value::Width;
use rsemu::ir::{
    AccessKind, Align, Block, BlockBuilder, Cond, Const, Endian, Home, InsnStart, Liveness, MemOp,
    MemSpace, Opcode, RegBanks, RegSlot, SegId, Sign, Temp, Type, eliminate_dead_code, linear_scan,
    verify,
};

/// How many selectors one input may drive. A block a frontend builds is tens
/// of instructions, not thousands, and an unbounded loop here is a timeout
/// rather than a finding.
const MAX_INSTS: usize = 96;
/// How many temporaries one boundary may name as live guest state.
const MAX_LIVE: usize = 6;

/// A tiny structured-input decoder over the fuzzer's byte stream.
///
/// Total by construction: running out of input yields zeros, so a truncated
/// corpus entry degrades into a smaller block instead of being discarded.
struct Gen<'a> {
    data: &'a [u8],
    pos: usize,
    /// Temporaries some earlier instruction has assigned, so an operand can
    /// name a value that exists.
    assigned: Vec<Temp>,
    /// Temporaries allocated and never assigned, so an operand can name the
    /// verifier's "used before it is assigned" case on purpose.
    pending: Vec<Temp>,
}

impl<'a> Gen<'a> {
    fn new(data: &'a [u8]) -> Gen<'a> {
        Gen {
            data,
            pos: 0,
            assigned: Vec::new(),
            pending: Vec::new(),
        }
    }

    fn is_empty(&self) -> bool {
        self.pos >= self.data.len()
    }

    fn u8(&mut self) -> u8 {
        let byte = self.data.get(self.pos).copied().unwrap_or(0);
        self.pos = self.pos.saturating_add(1);
        byte
    }

    fn u32(&mut self) -> u32 {
        let mut out = 0u32;
        for _ in 0..4 {
            out = (out << 8) | u32::from(self.u8());
        }
        out
    }

    /// A count in `0..=max`.
    fn count(&mut self, max: usize) -> usize {
        if max == 0 {
            0
        } else {
            usize::from(self.u8()) % (max + 1)
        }
    }

    /// A temporary: usually one that has been assigned, sometimes not.
    ///
    /// The proportions are the whole design of this generator. Only-valid
    /// numbers never reach the verifier's rejection arms; only-wild numbers
    /// make every block malformed for the same trivial reason, and then the
    /// interesting half of this target — what the pass does to a block the
    /// verifier *accepted* — never runs at all. So: an assigned temporary
    /// unless the byte is at least `0xf0`, above which it is one that was
    /// allocated and never assigned, and above `0xf8` a number that was never
    /// allocated at all.
    fn temp(&mut self) -> Temp {
        let n = u32::from(self.u8());
        if n >= 0xf8 {
            return Temp(n);
        }
        let pool = if n >= 0xf0 {
            &self.pending
        } else {
            &self.assigned
        };
        match pool.len() {
            0 => Temp(n),
            len => pool[(n as usize) % len],
        }
    }

    /// Record a temporary an instruction just assigned.
    fn assign(&mut self, temp: Temp) {
        self.assigned.push(temp);
    }

    fn ty(&mut self) -> Type {
        const TYPES: [Type; 7] = [
            Type::I1,
            Type::I32,
            Type::I64,
            Type::I128,
            Type::F32,
            Type::F64,
            Type::V128,
        ];
        TYPES[usize::from(self.u8()) % TYPES.len()]
    }

    fn cond(&mut self) -> Cond {
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
        CONDS[usize::from(self.u8()) % CONDS.len()]
    }

    /// An access descriptor, every field of it chosen by the input.
    ///
    /// `volatile` above all: it is the flag that decides whether a load may be
    /// eliminated, so it has to vary, and the assertions below hold it to
    /// keeping every volatile access.
    fn mem_op(&mut self, kind: AccessKind) -> MemOp {
        let bits = self.u8();
        const SIZES: [Width; 4] = [Width::U8, Width::U16, Width::U32, Width::U64];
        const ENDIANS: [Endian; 3] = [Endian::Little, Endian::Big, Endian::AsRegion];
        const ALIGNS: [Align; 4] = [Align::None, Align::Fault, Align::Split, Align::Rotate];
        MemOp {
            size: SIZES[usize::from(bits) % SIZES.len()],
            sign: if bits & 0x04 == 0 {
                Sign::Unsigned
            } else {
                Sign::Signed
            },
            space: if bits & 0x08 == 0 {
                MemSpace::MEM
            } else {
                MemSpace::IO
            },
            seg: if bits & 0x10 == 0 {
                None
            } else {
                Some(SegId(bits >> 5))
            },
            endian: ENDIANS[usize::from(bits >> 5) % ENDIANS.len()],
            align: ALIGNS[usize::from(bits >> 6) % ALIGNS.len()],
            kind,
            volatile: bits & 0x20 != 0,
        }
    }
}

/// The binary ops a generated block draws from.
const BINARY: [Opcode; 18] = [
    Opcode::ADD,
    Opcode::SUB,
    Opcode::MUL,
    Opcode::DIV_S,
    Opcode::DIV_U,
    Opcode::REM_S,
    Opcode::REM_U,
    Opcode::AND,
    Opcode::OR,
    Opcode::XOR,
    Opcode::ANDC,
    Opcode::SHL,
    Opcode::SHR,
    Opcode::SAR,
    Opcode::ROTL,
    Opcode::ROTR,
    Opcode::MULHSU,
    Opcode::DEPOSIT,
];

/// The unary ops a generated block draws from.
const UNARY: [Opcode; 9] = [
    Opcode::MOV,
    Opcode::NEG,
    Opcode::NOT,
    Opcode::EXT_S,
    Opcode::EXT_Z,
    Opcode::TRUNC,
    Opcode::CLZ,
    Opcode::CTZ,
    // The one this whole pass exists for: x86's PF and the Z80's P/V.
    Opcode::POPCOUNT,
];

/// The atomics, every one of which must survive elimination.
const ATOMIC: [Opcode; 8] = [
    Opcode::FETCH_ADD,
    Opcode::FETCH_AND,
    Opcode::FETCH_OR,
    Opcode::FETCH_XOR,
    Opcode::FETCH_SMIN,
    Opcode::FETCH_SMAX,
    Opcode::FETCH_UMIN,
    Opcode::FETCH_UMAX,
];

/// Build a block from the fuzzer's bytes.
fn build(pick: &mut Gen<'_>) -> Block {
    let entry_pc = u64::from(pick.u32());
    let key = u64::from(pick.u8());
    let mut b = BlockBuilder::new(entry_pc, key);

    // Open a guest instruction before anything else. A charge outside one is a
    // tick nothing can attribute and the verifier rejects the block on the
    // spot — correct, and if it happened on most inputs this target would
    // spend its whole budget rediscovering that one rule.
    b.insn_start(InsnStart {
        pc: entry_pc,
        next_pc: entry_pc.wrapping_add(2),
        ticks: 0,
        live: Vec::new(),
    });

    // Whether a terminator has already been emitted. Reaching one normally
    // ends the block; the `goto_tb` selector deliberately carries on past it,
    // which is the "a terminator must be the last instruction" arm.
    let mut terminated = false;

    for _ in 0..MAX_INSTS {
        if pick.is_empty() || terminated {
            break;
        }
        match pick.u8() % 16 {
            0x00 => {
                let pc = u64::from(pick.u32());
                let ticks = u64::from(pick.u8());
                let live = (0..pick.count(MAX_LIVE))
                    .map(|_| (RegSlot(u16::from(pick.u8())), pick.temp()))
                    .collect();
                b.insn_start(InsnStart {
                    pc,
                    next_pc: pc.wrapping_add(2),
                    ticks,
                    live,
                });
            }
            0x01 => b.charge(u64::from(pick.u8())),
            0x02 => {
                let ty = pick.ty();
                let value = Const::Int(u128::from(pick.u8()));
                let t = b.imm(ty, value);
                pick.assign(t);
            }
            0x03 => {
                let op = BINARY[usize::from(pick.u8()) % BINARY.len()];
                let ty = pick.ty();
                let (a, c) = (pick.temp(), pick.temp());
                let t = b.binary(op, ty, a, c);
                pick.assign(t);
            }
            0x04 => {
                let op = UNARY[usize::from(pick.u8()) % UNARY.len()];
                let ty = pick.ty();
                let a = pick.temp();
                let t = b.unary(op, ty, a);
                pick.assign(t);
            }
            0x05 => {
                let op = if pick.u8().is_multiple_of(2) {
                    Opcode::ADDC
                } else {
                    Opcode::SUBB
                };
                let ty = pick.ty();
                let (a, c) = (pick.temp(), pick.temp());
                // A carry in is one bit or the block is malformed, so most of
                // the time it is a fresh `i1` — otherwise every carry op in
                // the corpus dies on the same type error and the ops after it
                // are never reached.
                let carry = if pick.u8().is_multiple_of(4) {
                    pick.temp()
                } else {
                    let t = b.imm(Type::I1, Const::Int(u128::from(pick.u8() & 1)));
                    pick.assign(t);
                    t
                };
                let (value, carry_out) = b.addc(op, ty, a, c, carry);
                pick.assign(value);
                pick.assign(carry_out);
            }
            0x06 => {
                let cond = pick.cond();
                let ty = pick.ty();
                let (a, c) = (pick.temp(), pick.temp());
                let t = b.setcond(cond, ty, a, c);
                pick.assign(t);
            }
            0x07 => {
                let mem = pick.mem_op(AccessKind::Load);
                let ty = pick.ty();
                let addr = pick.temp();
                let t = b.load(ty, addr, mem);
                pick.assign(t);
            }
            0x08 => {
                let mem = pick.mem_op(AccessKind::Store);
                let ty = pick.ty();
                let (addr, value) = (pick.temp(), pick.temp());
                b.store(ty, addr, value, mem);
            }
            0x09 => {
                let cond = pick.temp();
                b.emit_void(Opcode::BRCOND, Type::I1, &[cond]);
            }
            0x0a => {
                let ty = pick.ty();
                let arg = pick.temp();
                let t = b.emit(Opcode::CALL_HELPER, ty, &[arg]);
                pick.assign(t);
            }
            0x0b => {
                // Allocated and never assigned, which is legal on its own and
                // is how a use-before-def block comes about.
                let ty = pick.ty();
                let t = b.temp(ty);
                pick.pending.push(t);
            }
            0x0c => {
                // A load with no MemOp: malformed by construction, and the arm
                // of the verifier no well-behaved builder can reach.
                let ty = pick.ty();
                let addr = pick.temp();
                let t = b.emit(Opcode::LD, ty, &[addr]);
                pick.assign(t);
            }
            0x0d => {
                let op = ATOMIC[usize::from(pick.u8()) % ATOMIC.len()];
                let ty = pick.ty();
                let (addr, value) = (pick.temp(), pick.temp());
                let t = b.emit(op, ty, &[addr, value]);
                pick.assign(t);
            }
            0x0e => {
                b.exit_tb();
                terminated = true;
            }
            _ => {
                // Deliberately does *not* stop: a block that carries on past a
                // terminator is the one the verifier must reject by position
                // rather than by content.
                b.emit_void(Opcode::GOTO_TB, Type::I64, &[]);
            }
        }
    }

    // Most inputs end in a terminator, so most blocks reach the verifier's
    // later rules rather than stopping at "does not end in a terminator"; the
    // rest exercise that arm.
    if !terminated && !pick.u8().is_multiple_of(4) {
        b.exit_tb();
    }
    b.finish()
}

/// Whether an instruction is one dead-code elimination is allowed to remove.
///
/// The rule from `ir::pass`, restated here on purpose: a target that imported
/// the pass's own predicate would agree with it by construction and prove
/// nothing.
fn eliminable(block: &Block, index: usize) -> bool {
    let inst = &block.insts()[index];
    let has_result = inst.dst.is_some() || inst.dst2.is_some();
    has_result
        && !inst.op.is_terminator()
        && !inst.op.has_side_effect()
        && !inst.mem.is_some_and(|m| m.volatile)
}

/// How many instructions of `block` must survive elimination whatever the
/// liveness says.
fn effect_count(block: &Block) -> usize {
    (0..block.insts().len())
        .filter(|i| !eliminable(block, *i))
        .count()
}

fuzz_target!(|data: &[u8]| {
    if data.is_empty() {
        return;
    }
    let mut pick = Gen::new(data);
    let block = build(&mut pick);

    // The dump is what a differential failure gets reported as, so it runs on
    // every block including the malformed ones: a `Display` that panics on a
    // bad block panics exactly when someone is trying to debug one.
    let _ = format!("{block}");

    // The claim under test, part one: whatever the bytes said, this answers.
    let accepted = verify(&block).is_ok();

    // Part two: and so does the pass, on a block the verifier has not vetted.
    let out = eliminate_dead_code(&block);

    // Nothing is invented, nothing is renumbered, and the boundary records —
    // which an INSN_START names by index and a fault reads by number — are
    // untouched.
    assert!(out.insts().len() <= block.insts().len());
    assert_eq!(out.entry_pc, block.entry_pc);
    assert_eq!(out.key, block.key);
    assert_eq!(out.temp_count(), block.temp_count());
    assert_eq!(out.marks(), block.marks(), "a boundary record moved");

    // No effect went away: stores, atomics, fences, helpers, charges,
    // boundaries, terminators, result-free instructions, and every volatile
    // access — the 6502's dummy read among them, whose value is discarded and
    // whose bus cycle is the point.
    assert_eq!(
        effect_count(&out),
        effect_count(&block),
        "elimination removed an instruction that has an effect:\n{block}"
    );

    // What survives is either an effect or a value something needs. This is
    // the pass being *minimal* rather than merely safe: a DCE that kept
    // everything would satisfy every assertion above.
    let live = Liveness::compute(&out);
    for i in 0..out.insts().len() {
        if !eliminable(&out, i) {
            continue;
        }
        let inst = &out.insts()[i];
        let needed = [inst.dst, inst.dst2]
            .into_iter()
            .flatten()
            .any(|t| live.is_live(t) || live.life(t).is_none());
        assert!(needed, "a dead instruction survived at {i}:\n{out}");
    }

    // One pass reaches the fixed point: a chain — the popcount feeding the
    // mask feeding the parity flag — goes in one backward walk, not one link
    // per run.
    let again = eliminate_dead_code(&out);
    assert_eq!(again.insts(), out.insts(), "the pass is not idempotent");

    // And the property the two halves make together.
    if accepted {
        verify(&out).expect("a block the verifier accepted stopped verifying after elimination");
    }

    // Liveness is defined on any block, verified or not, and a register
    // allocator will consume `intervals()` in exactly this order.
    let intervals = Liveness::compute(&block).intervals();
    assert!(
        intervals
            .windows(2)
            .all(|w| (w[0].1, w[0].0) <= (w[1].1, w[1].0)),
        "intervals came back out of order"
    );
    for (temp, lo, hi) in &intervals {
        assert!(lo <= hi, "{temp} has a backwards live range");
    }

    // And the allocator that consumes them, over the same arbitrary blocks.
    // Three properties, each of which is a silent miscompile rather than a
    // crash when it stops holding.
    let live = Liveness::compute(&block);
    let calls: Vec<bool> = block
        .insts()
        .iter()
        .map(|i| matches!(i.op, Opcode::LD | Opcode::ST | Opcode::CALL_HELPER))
        .collect();
    let banks = RegBanks {
        saved: &[0, 1, 2],
        volatile: &[3, 4, 5, 6],
    };
    let alloc = linear_scan(&block, &live, &banks, &calls);

    // 1. The same block allocates the same way every time. Register
    //    assignment decides guest-visible state, so a hashed iteration order
    //    or an unstable sort in here would make a state hash stop being an
    //    identity (`ROADMAP.md` §0).
    assert_eq!(
        alloc,
        linear_scan(&block, &live, &banks, &calls),
        "the allocation is not reproducible"
    );

    // 2. No two temporaries whose live ranges overlap share a register. The
    //    one property the whole algorithm exists to keep, and the one a
    //    differential shows only when the two values happen to differ.
    for (a, alo, ahi) in &intervals {
        let Home::Reg(ra) = alloc.home(*a) else {
            continue;
        };
        for (b, blo, bhi) in &intervals {
            if b <= a {
                continue;
            }
            let Home::Reg(rb) = alloc.home(*b) else {
                continue;
            };
            // Half-open at the join: an instruction reads its operands before
            // it writes its results, so a range ending where another begins
            // may share.
            if ra == rb {
                assert!(
                    ahi <= blo || bhi <= alo,
                    "{a} [{alo},{ahi}] and {b} [{blo},{bhi}] both took r{ra}:\n{block}"
                );
            }
        }
    }

    // 3. Everything a boundary names reaches the frame, whatever else it got.
    //    That is `ROADMAP.md` §9's precise-exception requirement expressed at
    //    the allocator: the exception path materializes architectural state
    //    out of the frame and has nowhere else to look.
    for mark in block.marks() {
        for (_, temp) in &mark.live {
            if temp.index() < block.temp_count() {
                assert!(
                    alloc.frame_backed(*temp),
                    "{temp} is named live at {:#x} and would not reach the frame:\n{block}",
                    mark.pc
                );
            }
        }
    }
});
