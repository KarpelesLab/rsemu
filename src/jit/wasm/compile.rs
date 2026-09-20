//! The code generator: one [`Block`] in, one WebAssembly module out.
//!
//! Safe Rust, `no_std + alloc`, gated to no host at all. It produces a
//! `Vec<u8>` that is a complete `WebAssembly.Module` and a table of [`MemOp`]
//! descriptors the imports index; [`exec`](mod@super::exec) is one thing that
//! can run the result and an embedder's own engine is the other.
//!
//! # Three things this does not do that the native backends do
//!
//! **No register allocation.** `ir::linear_scan` decides where a temporary
//! lives because a host has a fixed number of registers and a frame; wasm has
//! neither. A function may declare as many locals as it likes and the
//! *engine's* backend allocates registers for them, so every IR temporary gets
//! its own `i64` local and the allocator is skipped. That is the single
//! largest simplification wasm buys, and it is why this backend is a third of
//! the size of either native one.
//!
//! **No fixups.** wasm control flow is structured (core specification §2.4.8),
//! so a branch names a *label depth* rather than a byte displacement. A
//! block's forward `brcond`s become nested `block`s opened at the function's
//! head and closed at their targets, and there is nothing to patch afterwards.
//! `Nest` below is that nesting.
//!
//! **No inlined TLB probe.** `ROADMAP.md` §9.1's first mechanism is the one
//! thing this backend leaves on the table: a guest load is an import call
//! straight to [`IrHost::load`](crate::ir::IrHost::load), not a mask, a
//! compare, an add and a `mov` over [`FastSet`](crate::jit::FastSet). It could
//! be — linear memory is byte-addressed and the TLB's answer is a byte offset
//! — and `docs/techniques/wasm-jit.md` says why it is not yet: the probe's
//! win is measured against *the cost of the call it avoids*, and on wasm that
//! call is an import call whose cost is the embedder's business and is not
//! known here. Measuring it needs an embedder; inventing it does not.
//!
//! # One value representation
//!
//! Every temporary is an `i64` local holding the value **canonically masked to
//! its type**, exactly as [`Interp`](crate::ir::Interp) holds it: an `i32`
//! temporary never carries bits above bit 32, and an `i1` never carries bits
//! above bit 1. An `i32` operation is therefore an `i64` operation followed by
//! a mask, which costs one instruction and removes an entire class of
//! divergence — the native backends both have a sub-register story and both
//! had to write down what canonical means. Here there is one width and the
//! rule is uniform.
//!
//! # What compiles, and what does not
//!
//! [`compiles`] is the list. A block containing anything else is **refused**
//! and runs on the interpreter, which is `ROADMAP.md` §9's answer for an
//! unsupported host applied to an unsupported block, and it is what makes a
//! partial backend a correct one. A [`Refusal`] always names what stopped it.
//!
//! Refused, and each for a stated reason:
//!
//! * **the divides and remainders** — wasm's `i64.div_s` **traps** on a zero
//!   divisor and on `INT_MIN / -1` (core specification §4.3.2), and a trap
//!   unwinds through the embedder rather than returning a status. The IR says
//!   a frontend owes the guard, so a guarded divide would be safe; a block
//!   whose guard this backend cannot see would not be. Both native backends
//!   refuse these for their own reasons and this one refuses them for a
//!   sharper one.
//! * **`mulu2`/`muls2`/`mulhsu`** — wasm has no 64×64→128 multiply at all, so
//!   a widening multiply is a four-way 32-bit decomposition. Worth writing
//!   when a frontend this backend runs emits one; not before.
//! * **`addc`/`subb`, `rotlc`/`rotrc`** — as in both native backends.
//! * **the atomics, the exclusives and `fence`** — a guest atomic has to reach
//!   the host's, and `IrHost::rmw` is that seam. Threaded wasm has its own
//!   atomic instructions, but they operate on the *embedder's* linear memory
//!   rather than on whatever a `RamStore` resolved to, so this is a seam
//!   question and not an encoding one.
//! * **`call_helper`** — a helper publishes the pending boundary and may
//!   return two results; both are reachable, and neither is reachable without
//!   an import whose shape no frontend this backend runs needs yet.
//! * **`phi`** — superblocks, and the SSA form they need, are a frontend
//!   question first.
//! * **anything wider than 64 bits, and both float types** — there is no wasm
//!   value type for an `i128`, and a guest float is a helper call into
//!   `float::soft` by design (`ROADMAP.md` §9.1), never a host `f64`.

use alloc::vec;
use alloc::vec::Vec;

use crate::ir::{Block, Cond, Inst, MemOp, Opcode, Sign, Temp, Type, bitfield_parts};

use super::abi::{self, LOCAL_CTX, LOCAL_FRAME, LOCAL_TEMPS, func, name, status, temp_offset};
use super::emit::{Func, FuncType, module, op, ty};

/// Why a block was not compiled.
///
/// Never an error: the IR interpreter is always the fallback
/// (`ROADMAP.md` §9, "Backends"), so a refusal costs speed on that block and
/// nothing else. Every variant names something specific, because "the JIT did
/// not take it" with no reason attached is how a backend's coverage silently
/// rots.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Refusal {
    /// An opcode this backend does not lower. See [`compiles`].
    Op(Opcode),
    /// A type wasm has no value type for: anything wider than 64 bits, and
    /// both float types.
    Type(Type),
    /// The block is shaped in a way the compiler will not take: a branch that
    /// is not forward or leaves the block, a missing terminator, an operand
    /// count that does not match the op, a bitfield outside its type.
    Shape(&'static str),
    /// More temporaries than a wasm function may declare locals for.
    TooManyTemps,
}

impl core::fmt::Display for Refusal {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Refusal::Op(op) => write!(f, "no lowering for `{op}`"),
            Refusal::Type(t) => write!(f, "wasm has no value type for an `{t}`"),
            Refusal::Shape(what) => write!(f, "{what}"),
            Refusal::TooManyTemps => f.write_str("the block declares more temporaries than locals"),
        }
    }
}

/// Whether this backend lowers `op`.
///
/// Everything `cpu::riscv::lift` emits for RV64I, plus the neighbours that
/// cost nothing once their family is in. The module docs say what is missing
/// and why each one is missing.
#[must_use]
pub fn compiles(op: Opcode) -> bool {
    matches!(
        op,
        Opcode::MOV
            | Opcode::GET_SLOT
            | Opcode::EXT_S
            | Opcode::EXT_Z
            | Opcode::TRUNC
            | Opcode::BSWAP
            | Opcode::DEPOSIT
            | Opcode::EXTRACT
            | Opcode::ADD
            | Opcode::SUB
            | Opcode::MUL
            | Opcode::NEG
            | Opcode::AND
            | Opcode::OR
            | Opcode::XOR
            | Opcode::NOT
            | Opcode::ANDC
            | Opcode::SHL
            | Opcode::SHR
            | Opcode::SAR
            | Opcode::ROTL
            | Opcode::ROTR
            | Opcode::CLZ
            | Opcode::CTZ
            | Opcode::POPCOUNT
            | Opcode::SETCOND
            | Opcode::MOVCOND
            | Opcode::BRCOND
            | Opcode::LD
            | Opcode::ST
            | Opcode::CHARGE
            | Opcode::INSN_START
            | Opcode::GOTO_TB
            | Opcode::EXIT_TB
            | Opcode::LOOKUP_AND_GOTO
    )
}

/// One block, lowered.
#[derive(Debug, Clone)]
pub struct Compiled {
    module: Vec<u8>,
    mems: Vec<MemOp>,
    temps: usize,
    through: Vec<bool>,
}

impl Compiled {
    /// The complete module, ready for `WebAssembly.Module` or
    /// [`exec`](super::exec).
    #[inline]
    #[must_use]
    pub fn module(&self) -> &[u8] {
        &self.module
    }

    /// The access descriptors the load and store imports index.
    ///
    /// A side table rather than an encoding in the immediate, exactly as both
    /// native backends carry one: a [`MemOp`] is nine fields and generated
    /// code has no business reconstructing it.
    #[inline]
    #[must_use]
    pub fn mem_ops(&self) -> &[MemOp] {
        &self.mems
    }

    /// How many temporaries the block declares.
    #[inline]
    #[must_use]
    pub fn temps(&self) -> usize {
        self.temps
    }

    /// How many bytes of linear memory the frame needs.
    #[inline]
    #[must_use]
    pub fn frame_bytes(&self) -> usize {
        abi::frame_bytes(self.temps)
    }

    /// Whether `temp` is written through to the frame at its definition.
    ///
    /// True exactly for the temporaries some [`InsnStart`](crate::ir::InsnStart)
    /// names, which are the ones the Rust side has to be able to read: to
    /// publish architectural state at a fault, at a spent allowance or at the
    /// end of a block, and to answer a [`Opcode::GET_SLOT`] of a slot the
    /// pending boundary shadows.
    #[inline]
    #[must_use]
    pub fn written_through(&self, temp: Temp) -> bool {
        self.through.get(temp.index()).copied().unwrap_or(false)
    }
}

/// Lower `block`.
///
/// # Errors
///
/// A [`Refusal`] naming what stopped it. Never an `Error`: a refused block is
/// interpreted, which is a speed difference and not a semantic one.
pub fn compile(block: &Block) -> Result<Compiled, Refusal> {
    Compiler::new(block)?.run()
}

/// The forward-branch nesting.
///
/// A `brcond` in an IR block jumps forward to an instruction index. wasm has
/// no such jump: it has `br`, which leaves a structured block. So one `block`
/// is opened at the function's head per distinct target, **largest target
/// outermost**, and closed at that target's own index. At any point the open
/// blocks are exactly those whose targets are still ahead, nested with the
/// nearest target innermost — so a branch to target `t` is a `br` to the depth
/// at which `t` sits in the ascending list of remaining targets.
///
/// Properly nested by construction, which is the property wasm validation
/// demands and the reason this is a sorted list rather than a graph algorithm.
/// A block whose branches are not all forward is **refused**; every frontend
/// in the tree emits forward branches only, and both native backends refuse
/// the same shape for the same reason.
#[derive(Debug)]
struct Nest {
    /// Every distinct branch target, ascending.
    targets: Vec<usize>,
    /// How many of them have already been closed.
    closed: usize,
}

impl Nest {
    fn new(block: &Block) -> Result<Nest, Refusal> {
        let n = block.insts().len();
        let mut targets = Vec::new();
        for (at, inst) in block.insts().iter().enumerate() {
            if inst.op != Opcode::BRCOND {
                continue;
            }
            let t = inst.aux as usize;
            if t <= at {
                return Err(Refusal::Shape("a branch that is not forward"));
            }
            if t >= n {
                return Err(Refusal::Shape("a branch target outside the block"));
            }
            if !targets.contains(&t) {
                targets.push(t);
            }
        }
        targets.sort_unstable();
        Ok(Nest { targets, closed: 0 })
    }

    /// The label depth a branch to `target` takes, from the instruction at
    /// `at`.
    fn depth(&self, target: usize) -> u32 {
        // The open blocks are `targets[closed..]`, innermost first because the
        // smallest remaining target was opened last.
        self.targets[self.closed..]
            .iter()
            .position(|&t| t == target)
            .map(|p| p as u32)
            .unwrap_or(0)
    }
}

struct Compiler<'a> {
    block: &'a Block,
    f: Func,
    mems: Vec<MemOp>,
    through: Vec<bool>,
    nest: Nest,
    /// The local holding a scratch `i64`.
    s0: u32,
    /// A second scratch `i64`.
    s1: u32,
    /// The local holding an import's `i32` answer.
    sc: u32,
}

impl<'a> Compiler<'a> {
    fn new(block: &'a Block) -> Result<Compiler<'a>, Refusal> {
        let temps = block.temp_count();
        // One local per temporary, plus two scratch `i64` and one scratch
        // `i32`, plus the two parameters. A local index is a `u32` (core
        // specification §5.4.3), so a block that would not fit that index
        // space is refused rather than wrapped — which is also the only thing
        // this arithmetic is for, so its result is the count itself.
        let highest = u32::try_from(temps)
            .ok()
            .and_then(|t| t.checked_add(LOCAL_TEMPS + 3))
            .ok_or(Refusal::TooManyTemps)?;
        let temps32 = highest - LOCAL_TEMPS - 3;

        // Which temporaries the Rust side must be able to read. A boundary's
        // live map is the whole answer: nothing else can observe a temporary.
        let mut through = vec![false; temps];
        for mark in block.marks() {
            for &(_, t) in &mark.live {
                if let Some(slot) = through.get_mut(t.index()) {
                    *slot = true;
                }
            }
        }

        Ok(Compiler {
            block,
            f: Func::new(temps32 + 2, 1),
            mems: Vec::new(),
            through,
            nest: Nest::new(block)?,
            s0: LOCAL_TEMPS + temps32,
            s1: LOCAL_TEMPS + temps32 + 1,
            sc: LOCAL_TEMPS + temps32 + 2,
        })
    }

    fn run(mut self) -> Result<Compiled, Refusal> {
        // Open one block per branch target, largest first, so they close in
        // ascending order and stay properly nested.
        for _ in 0..self.nest.targets.len() {
            self.f.block();
        }

        for at in 0..self.block.insts().len() {
            while self
                .nest
                .targets
                .get(self.nest.closed)
                .is_some_and(|&t| t == at)
            {
                self.f.op(op::END);
                self.nest.closed += 1;
            }
            self.inst(at)?;
        }
        while self.nest.closed < self.nest.targets.len() {
            self.f.op(op::END);
            self.nest.closed += 1;
        }

        // Falling off the end means the block had no reachable terminator,
        // which `verify` rejects and the fuzz targets can still build. It is
        // an `Err` from the engine rather than a trap, because a trap is the
        // one failure a wasm embedder cannot hand back as a value.
        self.f.i64_const(status::ERROR);

        let types = vec![
            FuncType {
                params: vec![ty::I32, ty::I32],
                results: vec![ty::I64],
            },
            FuncType {
                params: vec![ty::I32, ty::I32],
                results: vec![ty::I64],
            },
            FuncType {
                params: vec![ty::I32, ty::I32, ty::I64, ty::I32],
                results: vec![ty::I32],
            },
            FuncType {
                params: vec![ty::I32, ty::I32, ty::I64, ty::I64, ty::I32],
                results: vec![ty::I32],
            },
            FuncType {
                params: vec![ty::I32, ty::I32, ty::I64],
                results: vec![ty::I32],
            },
        ];
        let imports = [
            (name::MODULE, name::SLOT, 1),
            (name::MODULE, name::LOAD, 2),
            (name::MODULE, name::STORE, 3),
            (name::MODULE, name::NOTE, 4),
        ];
        let bytes = module(
            &types,
            &imports,
            (name::MODULE, name::MEMORY),
            name::BLOCK,
            0,
            &self.f.finish(),
        );

        Ok(Compiled {
            module: bytes,
            mems: self.mems,
            temps: self.block.temp_count(),
            through: self.through,
        })
    }

    // ---- the pieces every lowering is built from --------------------------

    /// The local a temporary lives in, checking that the block allocated it.
    fn local(&self, t: Temp) -> Result<u32, Refusal> {
        if t.index() >= self.block.temp_count() {
            return Err(Refusal::Shape(
                "an operand was never allocated in this block",
            ));
        }
        Ok(LOCAL_TEMPS + t.0)
    }

    fn push(&mut self, t: Temp) -> Result<(), Refusal> {
        let l = self.local(t)?;
        self.f.op_u(op::LOCAL_GET, l);
        Ok(())
    }

    /// Mask the value on top of the stack to `bits`.
    fn mask(&mut self, bits: u32) {
        if bits >= 64 {
            return;
        }
        self.f.i64_const(mask_bits(bits) as i64);
        self.f.op(op::I64_AND);
    }

    /// Sign-extend the low `bits` of the value on top of the stack.
    ///
    /// A `shl`/`shr_s` pair rather than `i64.extend32_s`: the MVP set is what
    /// every embedder accepts, and the sign-extension operators are a later
    /// proposal. See [`emit`](super::emit)'s module docs.
    fn sext(&mut self, bits: u32) {
        if bits >= 64 {
            return;
        }
        let k = i64::from(64 - bits);
        self.f.i64_const(k);
        self.f.op(op::I64_SHL);
        self.f.i64_const(k);
        self.f.op(op::I64_SHR_S);
    }

    /// Store what is on the stack into an instruction's destination.
    ///
    /// Masks to the **destination temporary's** declared type, which is what
    /// `Interp::set` does — not to `inst.ty`, which is the type the operation
    /// was performed at and is not always the same thing.
    fn write(&mut self, inst: &Inst) -> Result<(), Refusal> {
        let dst = inst
            .dst
            .ok_or(Refusal::Shape("this op must have a destination"))?;
        self.write_temp(dst)
    }

    fn write_temp(&mut self, dst: Temp) -> Result<(), Refusal> {
        let ty = self
            .block
            .type_of(dst)
            .ok_or(Refusal::Shape("a result was never allocated in this block"))?;
        self.mask(ty.bits());
        let l = self.local(dst)?;
        self.f.op_u(op::LOCAL_SET, l);
        if self.through[dst.index()] {
            // Write-through: the frame copy is how the Rust side materializes
            // architectural state at a fault, at a spent allowance and at the
            // block's end. Three instructions, at a definition only, and only
            // for the temporaries a boundary names.
            self.f.op_u(op::LOCAL_GET, LOCAL_FRAME);
            self.f.op_u(op::LOCAL_GET, l);
            self.f.mem64(op::I64_STORE, temp_offset(dst.0));
        }
        Ok(())
    }

    /// `if (answer != 0) return answer`, over an import's `i32` result.
    fn check(&mut self) {
        self.f.op_u(op::LOCAL_TEE, self.sc);
        self.f.if_();
        self.f.op_u(op::LOCAL_GET, self.sc);
        self.f.op(op::I64_EXTEND_I32_U);
        self.f.op(op::RETURN);
        self.f.op(op::END);
    }

    /// Leave the block with a status, having put `pc` in the frame's out word.
    fn leave(&mut self, code: i64) {
        self.f.i64_const(code);
        self.f.op(op::RETURN);
    }

    /// Evaluate a comparison at `w` bits, leaving an `i32` 0/1 on the stack.
    fn compare(&mut self, cond: Cond, w: u32, a: Temp, b: Temp) -> Result<(), Refusal> {
        let signed = matches!(cond, Cond::LtS | Cond::LeS | Cond::GtS | Cond::GeS);
        self.push(a)?;
        if signed {
            self.sext(w);
        }
        self.push(b)?;
        if signed {
            self.sext(w);
        }
        self.f.op(match cond {
            Cond::Eq => op::I64_EQ,
            Cond::Ne => op::I64_NE,
            Cond::LtS => op::I64_LT_S,
            Cond::LeS => op::I64_LE_S,
            Cond::GtS => op::I64_GT_S,
            Cond::GeS => op::I64_GE_S,
            Cond::LtU => op::I64_LT_U,
            Cond::LeU => op::I64_LE_U,
            Cond::GtU => op::I64_GT_U,
            Cond::GeU => op::I64_GE_U,
        });
        Ok(())
    }

    /// A one-bit selector temporary, as an `i32` 0/1.
    fn selector(&mut self, t: Temp) -> Result<(), Refusal> {
        self.push(t)?;
        self.f.i64_const(1);
        self.f.op(op::I64_AND);
        self.f.op(op::I64_EQZ);
        self.f.op(op::I32_EQZ);
        Ok(())
    }

    // ---- one instruction ---------------------------------------------------

    #[allow(clippy::too_many_lines)]
    fn inst(&mut self, at: usize) -> Result<(), Refusal> {
        // The block outlives `self` borrows: pulled out as its own reference so
        // that reading an instruction's operands does not hold a borrow of the
        // compiler across the emitting calls below, and so that nothing here
        // allocates per instruction.
        let b: &'a Block = self.block;
        let inst = &b.insts()[at];
        let op_code = inst.op;
        if !compiles(op_code) {
            return Err(Refusal::Op(op_code));
        }
        let ty = inst.ty;
        if !holdable(ty) {
            return Err(Refusal::Type(ty));
        }
        let srcs: &'a [Temp] = b.srcs(at);
        for t in inst.dst.iter().chain(inst.dst2.iter()).chain(srcs.iter()) {
            match b.type_of(*t) {
                Some(t) if holdable(t) => {}
                Some(t) => return Err(Refusal::Type(t)),
                None => {
                    return Err(Refusal::Shape(
                        "a temporary this op names was never allocated",
                    ));
                }
            }
        }
        let w = ty.bits();
        let src = |i: usize| -> Result<Temp, Refusal> {
            srcs.get(i)
                .copied()
                .ok_or(Refusal::Shape("too few source operands"))
        };

        match op_code {
            // ---- Data movement -------------------------------------------
            Opcode::MOV => {
                match (srcs.first(), inst.imm) {
                    (Some(&s), _) => self.push(s)?,
                    (None, Some(c)) => {
                        let dst_bits = inst.dst.and_then(|d| b.type_of(d)).map_or(w, Type::bits);
                        let v = (c.bits() & u128::from(mask_bits(dst_bits))) as u64;
                        self.f.i64_const(v as i64);
                    }
                    (None, None) => {
                        return Err(Refusal::Shape("a mov needs a source or an immediate"));
                    }
                }
                self.write(inst)?;
            }
            Opcode::GET_SLOT => {
                self.f.op_u(op::LOCAL_GET, LOCAL_CTX);
                self.f.i32_const(i32::from(inst.aux as u16));
                self.f.op_u(op::CALL, func::SLOT);
                self.write(inst)?;
            }
            Opcode::EXT_S => {
                let s = src(0)?;
                let from = b.type_of(s).ok_or(Refusal::Shape(
                    "the source was never allocated in this block",
                ))?;
                self.push(s)?;
                self.sext(from.bits());
                self.write(inst)?;
            }
            Opcode::EXT_Z | Opcode::TRUNC => {
                self.push(src(0)?)?;
                self.write(inst)?;
            }
            Opcode::BSWAP => {
                let lane = match inst.imm {
                    Some(c) => u32::try_from(c.bits())
                        .map_err(|_| Refusal::Shape("the lane width is absurd"))?,
                    None => w,
                };
                if lane < 8 || !lane.is_multiple_of(8) || !w.is_multiple_of(lane) {
                    return Err(Refusal::Shape(
                        "the lane width must be whole bytes and divide the type",
                    ));
                }
                let s = src(0)?;
                let bytes = lane / 8;
                let mut first = true;
                for base in (0..w).step_by(lane as usize) {
                    for i in 0..bytes {
                        let from = base + 8 * i;
                        let to = base + 8 * (bytes - 1 - i);
                        self.push(s)?;
                        if from > 0 {
                            self.f.i64_const(i64::from(from));
                            self.f.op(op::I64_SHR_U);
                        }
                        self.f.i64_const(0xff);
                        self.f.op(op::I64_AND);
                        if to > 0 {
                            self.f.i64_const(i64::from(to));
                            self.f.op(op::I64_SHL);
                        }
                        if first {
                            first = false;
                        } else {
                            self.f.op(op::I64_OR);
                        }
                    }
                }
                self.write(inst)?;
            }
            Opcode::DEPOSIT => {
                let (pos, len) = bitfield_parts(inst.aux);
                let field = field_mask(pos, len, w)
                    .ok_or(Refusal::Shape("the bitfield does not fit within the type"))?;
                self.push(src(0)?)?;
                self.f.i64_const(!field as i64);
                self.f.op(op::I64_AND);
                self.push(src(1)?)?;
                if pos > 0 {
                    self.f.i64_const(i64::from(pos));
                    self.f.op(op::I64_SHL);
                }
                self.f.i64_const(field as i64);
                self.f.op(op::I64_AND);
                self.f.op(op::I64_OR);
                self.write(inst)?;
            }
            Opcode::EXTRACT => {
                let (pos, len) = bitfield_parts(inst.aux);
                let field = field_mask(pos, len, w)
                    .ok_or(Refusal::Shape("the bitfield does not fit within the type"))?;
                self.push(src(0)?)?;
                self.f.i64_const(field as i64);
                self.f.op(op::I64_AND);
                if pos > 0 {
                    self.f.i64_const(i64::from(pos));
                    self.f.op(op::I64_SHR_U);
                }
                self.write(inst)?;
            }

            // ---- Arithmetic and logic -------------------------------------
            Opcode::NEG => {
                self.f.i64_const(0);
                self.push(src(0)?)?;
                self.f.op(op::I64_SUB);
                self.write(inst)?;
            }
            Opcode::NOT => {
                self.push(src(0)?)?;
                self.f.i64_const(-1);
                self.f.op(op::I64_XOR);
                self.write(inst)?;
            }
            Opcode::ADD | Opcode::SUB | Opcode::MUL | Opcode::AND | Opcode::OR | Opcode::XOR => {
                self.push(src(0)?)?;
                self.push(src(1)?)?;
                self.f.op(match op_code {
                    Opcode::ADD => op::I64_ADD,
                    Opcode::SUB => op::I64_SUB,
                    Opcode::MUL => op::I64_MUL,
                    Opcode::AND => op::I64_AND,
                    Opcode::OR => op::I64_OR,
                    _ => op::I64_XOR,
                });
                self.write(inst)?;
            }
            Opcode::ANDC => {
                self.push(src(0)?)?;
                self.push(src(1)?)?;
                self.f.i64_const(-1);
                self.f.op(op::I64_XOR);
                self.f.op(op::I64_AND);
                self.write(inst)?;
            }

            // ---- Shifts ----------------------------------------------------
            //
            // The IR leaves an out-of-range shift undefined and every frontend
            // guards it, but "undefined" is not "whatever the host does": a
            // block may run compiled on one pass and interpreted on the next,
            // and the two must be indistinguishable to the guest
            // (`ROADMAP.md` §0). wasm reduces a shift count modulo 64 (core
            // specification §4.3.2); `Interp` takes the mathematical answer.
            // So the mathematical answer is selected explicitly here, at three
            // instructions a shift, rather than inherited.
            Opcode::SHL | Opcode::SHR => {
                let a = src(0)?;
                let amount = src(1)?;
                self.push(a)?;
                self.push(amount)?;
                self.f.op(if op_code == Opcode::SHL {
                    op::I64_SHL
                } else {
                    op::I64_SHR_U
                });
                self.f.i64_const(0);
                self.push(amount)?;
                self.f.i64_const(i64::from(w));
                self.f.op(op::I64_LT_U);
                self.f.op(op::SELECT);
                self.write(inst)?;
            }
            Opcode::SAR => {
                let a = src(0)?;
                let amount = src(1)?;
                self.push(a)?;
                self.sext(w);
                self.f.op_u(op::LOCAL_TEE, self.s0);
                self.push(amount)?;
                self.f.op(op::I64_SHR_S);
                // Out of range: the sign, replicated — which is `m` for a
                // negative operand and zero otherwise, exactly as `Interp`.
                self.f.op_u(op::LOCAL_GET, self.s0);
                self.f.i64_const(63);
                self.f.op(op::I64_SHR_S);
                self.push(amount)?;
                self.f.i64_const(i64::from(w));
                self.f.op(op::I64_LT_U);
                self.f.op(op::SELECT);
                self.write(inst)?;
            }
            Opcode::ROTL | Opcode::ROTR => {
                // A rotate is defined for every amount, so it reduces rather
                // than saturating. At 64 bits `i64.rotl` would do, but the
                // shift pair below is correct there too — `a >> 64` reduces to
                // `a >> 0`, and `a | a` is `a` — so there is one lowering
                // rather than two, and the 64-bit case is covered by the same
                // test as every other width.
                let a = src(0)?;
                let amount = src(1)?;
                self.push(amount)?;
                self.f.i64_const(i64::from(w));
                self.f.op(op::I64_REM_U);
                if op_code == Opcode::ROTR {
                    self.f.op_u(op::LOCAL_SET, self.s1);
                    self.f.i64_const(i64::from(w));
                    self.f.op_u(op::LOCAL_GET, self.s1);
                    self.f.op(op::I64_SUB);
                    self.f.i64_const(i64::from(w));
                    self.f.op(op::I64_REM_U);
                }
                self.f.op_u(op::LOCAL_SET, self.s0);
                self.push(a)?;
                self.f.op_u(op::LOCAL_GET, self.s0);
                self.f.op(op::I64_SHL);
                self.push(a)?;
                self.f.i64_const(i64::from(w));
                self.f.op_u(op::LOCAL_GET, self.s0);
                self.f.op(op::I64_SUB);
                self.f.op(op::I64_SHR_U);
                self.f.op(op::I64_OR);
                self.write(inst)?;
            }

            // ---- Bit counting ----------------------------------------------
            Opcode::CLZ => {
                self.push(src(0)?)?;
                self.f.op(op::I64_CLZ);
                if w < 64 {
                    self.f.i64_const(i64::from(64 - w));
                    self.f.op(op::I64_SUB);
                }
                self.write(inst)?;
            }
            Opcode::CTZ => {
                // `i64.ctz(0)` is 64; within a narrower type the answer is the
                // type's width, so the zero case is selected explicitly.
                self.push(src(0)?)?;
                self.f.op_u(op::LOCAL_TEE, self.s0);
                self.f.op(op::I64_CTZ);
                self.f.i64_const(i64::from(w));
                self.f.op_u(op::LOCAL_GET, self.s0);
                self.f.i64_const(0);
                self.f.op(op::I64_NE);
                self.f.op(op::SELECT);
                self.write(inst)?;
            }
            Opcode::POPCOUNT => {
                self.push(src(0)?)?;
                self.f.op(op::I64_POPCNT);
                self.write(inst)?;
            }

            // ---- Compare and branch -----------------------------------------
            Opcode::SETCOND => {
                let cond = inst
                    .cond
                    .ok_or(Refusal::Shape("a comparison needs a condition"))?;
                self.compare(cond, w, src(0)?, src(1)?)?;
                self.f.op(op::I64_EXTEND_I32_U);
                self.write(inst)?;
            }
            Opcode::MOVCOND => {
                let (t, f, cond) = match (inst.cond, srcs.len()) {
                    (Some(cond), 4) => (src(2)?, src(3)?, Some((cond, src(0)?, src(1)?))),
                    (_, 3) => (src(1)?, src(2)?, None),
                    _ => {
                        return Err(Refusal::Shape(
                            "a movcond takes a selector and two values, or a condition and four",
                        ));
                    }
                };
                self.push(t)?;
                self.push(f)?;
                match cond {
                    Some((cond, a, b)) => self.compare(cond, w, a, b)?,
                    None => self.selector(src(0)?)?,
                }
                self.f.op(op::SELECT);
                self.write(inst)?;
            }
            Opcode::BRCOND => {
                match (inst.cond, srcs.len()) {
                    (Some(cond), 2) => self.compare(cond, w, src(0)?, src(1)?)?,
                    (_, 1) => self.selector(src(0)?)?,
                    _ => {
                        return Err(Refusal::Shape(
                            "a brcond takes a selector, or a condition and two values",
                        ));
                    }
                }
                let depth = self.nest.depth(inst.aux as usize);
                self.f.op_u(op::BR_IF, depth);
            }

            // ---- Memory -------------------------------------------------------
            Opcode::LD => {
                let mem = inst
                    .mem
                    .ok_or(Refusal::Shape("a memory op needs a MemOp descriptor"))?;
                let slot = self.mem_slot(mem);
                self.f.op_u(op::LOCAL_GET, LOCAL_CTX);
                self.f.i32_const(slot);
                self.push(src(0)?)?;
                self.f.i32_const(at as i32);
                self.f.op_u(op::CALL, func::LOAD);
                self.check();
                // The value the import left in the frame's out word.
                self.f.op_u(op::LOCAL_GET, LOCAL_FRAME);
                self.f.mem64(op::I64_LOAD, 0);
                let bits = mem.size.bits();
                match mem.sign {
                    Sign::Unsigned => self.mask(bits),
                    Sign::Signed => {
                        self.mask(bits);
                        self.sext(bits);
                    }
                }
                self.write(inst)?;
            }
            Opcode::ST => {
                let mem = inst
                    .mem
                    .ok_or(Refusal::Shape("a memory op needs a MemOp descriptor"))?;
                let slot = self.mem_slot(mem);
                self.f.op_u(op::LOCAL_GET, LOCAL_CTX);
                self.f.i32_const(slot);
                self.push(src(0)?)?;
                self.push(src(1)?)?;
                self.f.i64_const(mem.size.mask() as i64);
                self.f.op(op::I64_AND);
                self.f.i32_const(at as i32);
                self.f.op_u(op::CALL, func::STORE);
                self.check();
            }

            // ---- Control and side effects ---------------------------------------
            Opcode::CHARGE => {
                let ticks = inst
                    .imm
                    .ok_or(Refusal::Shape("a charge needs a tick count"))?
                    .bits() as u64;
                self.f.op_u(op::LOCAL_GET, LOCAL_CTX);
                self.f.i32_const(abi::note::CHARGE);
                self.f.i64_const(ticks as i64);
                self.f.op_u(op::CALL, func::NOTE);
                self.f.op(op::DROP);
            }
            Opcode::INSN_START => {
                if b.marks().get(inst.aux as usize).is_none() {
                    return Err(Refusal::Shape("the boundary marker points at no record"));
                }
                // Whether a terminator follows is a *static* property, so the
                // "is this an exit boundary" test `Interp` makes at run time is
                // folded into the note's kind here. `ir::interp`'s INSN_START
                // arm says why an exit boundary is never asked at.
                let exit = b
                    .insts()
                    .get(at + 1)
                    .is_some_and(|next| next.op.is_terminator());
                self.f.op_u(op::LOCAL_GET, LOCAL_CTX);
                self.f.i32_const(if exit {
                    abi::note::BOUNDARY_EXIT
                } else {
                    abi::note::BOUNDARY
                });
                self.f.i64_const(i64::from(inst.aux));
                self.f.op_u(op::CALL, func::NOTE);
                self.check();
            }
            Opcode::GOTO_TB => {
                let pc = inst
                    .imm
                    .ok_or(Refusal::Shape("a goto_tb needs its successor's PC"))?
                    .bits() as u64;
                self.f.op_u(op::LOCAL_GET, LOCAL_FRAME);
                self.f.i64_const(pc as i64);
                self.f.mem64(op::I64_STORE, 0);
                self.leave(status::GOTO);
            }
            Opcode::EXIT_TB => self.leave(status::EXIT),
            Opcode::LOOKUP_AND_GOTO => {
                self.f.op_u(op::LOCAL_GET, LOCAL_FRAME);
                self.push(src(0)?)?;
                self.f.mem64(op::I64_STORE, 0);
                self.leave(status::LOOKUP);
            }

            other => return Err(Refusal::Op(other)),
        }
        Ok(())
    }

    /// The index of `mem` in the side table, interning equal descriptors.
    ///
    /// Every field of the descriptor is carried rather than checked — the
    /// segment, the byte order, the misalignment policy, the address space —
    /// because the import hands the whole thing to `IrHost::load` and
    /// `IrHost::store`, which is the same path the interpreter takes. A
    /// backend that inlined an access would have to start caring; this one
    /// does not, which is why an `IO`-space access or a big-endian region
    /// costs nothing extra here.
    fn mem_slot(&mut self, mem: MemOp) -> i32 {
        if let Some(i) = self.mems.iter().position(|m| *m == mem) {
            return i as i32;
        }
        self.mems.push(mem);
        (self.mems.len() - 1) as i32
    }
}

/// Whether a wasm local can hold a value of this type.
fn holdable(ty: Type) -> bool {
    matches!(ty, Type::I1 | Type::I32 | Type::I64)
}

/// A mask with `bits` low bits set, saturating at 64.
const fn mask_bits(bits: u32) -> u64 {
    if bits >= 64 {
        u64::MAX
    } else {
        (1u64 << bits) - 1
    }
}

/// The mask of a `len`-bit field at `pos`, or `None` if it leaves the type.
const fn field_mask(pos: u32, len: u32, width: u32) -> Option<u64> {
    if len == 0 || pos + len > width || width > 64 {
        return None;
    }
    Some(mask_bits(len) << pos)
}
