//! The RISC-V frontend: guest instructions lifted into [`ir::Block`](crate::ir::Block)s.
//!
//! `ROADMAP.md` §9's translation pipeline has two halves that never meet — a
//! frontend that turns guest bytes into IR, and a backend that turns IR into
//! something callable. This is the first frontend. RISC-V goes first because
//! its interpreter is the strongest oracle in the tree (riscv-arch-test
//! 181/181, riscv-tests 409/409) and because the ISA has **no condition
//! flags**, so this exercises the IR's structure — boundaries, ticks, the
//! register mapping — without simultaneously exercising the flags design that
//! [`ir`](crate::ir)'s decision 1 is about.
//!
//! # The subset, exactly
//!
//! A documented subset done exactly beats a broad one done approximately, so
//! this lifts **RV64I integer computation, memory and control flow** and
//! nothing else:
//!
//! * `LUI`, `AUIPC`, and every register-immediate and register-register
//!   integer ALU operation, including the RV64 `*W` word forms.
//! * Every load and store: `LB`/`LH`/`LW`/`LD` and their unsigned forms,
//!   `SB`/`SH`/`SW`/`SD`.
//! * The six conditional branches, `JAL`, and `JALR`.
//!
//! Deliberately **not** lifted, each ending the block with a terminator that
//! hands the PC back to the interpreter: `M`, `A`, `F`, `D`, every CSR
//! instruction, `ECALL`/`EBREAK`/`MRET`/`SRET`/`WFI`/`SFENCE.VMA`, both
//! fences, and RV32 as a whole ([`lift`] refuses an RV32 configuration
//! outright rather than silently mis-widening). A compressed encoding *is*
//! lifted when the core has `C`, because [`isa::expand`] turns it into exactly
//! one of the above — the same single description the interpreter and the
//! disassembler read (CLAUDE.md, "CPU cores"). This lifter is the **third**
//! consumer of [`isa::TABLE`]'s rows, never a fourth table: the `fmt` column
//! decides which register fields an encoding reads, and the `op` column
//! decides what it means.
//!
//! # Ticks, and where the block has to end
//!
//! [`ir`](crate::ir)'s decision 2: a tick count is a hashed *output*, so a
//! block that charges 7 where the interpreter charged 8 fails the phase-5
//! state-hash gate. The interpreter charges one tick per **bus access**
//! (`cpu::riscv::exec`), at exactly three kinds of site:
//!
//! | Site | Count | Static? |
//! | --- | --- | --- |
//! | instruction fetch | 1 per halfword — so 2 for an uncompressed instruction, 1 for a compressed one | **yes**, from the encoding |
//! | a load or store | 1 when aligned; `bytes` when misaligned and the core performs the split | no — depends on the run-time address |
//! | a page-table read during a walk | 0 on a TLB hit, 1 per level on a miss | no — depends on the TLB |
//!
//! Only the first is a static property of the bytes, so only the first is
//! emitted as [`Opcode::CHARGE`]. The other two force two structural rules,
//! and the rules are how this frontend stays exact instead of guessing:
//!
//! * **A block never leaves the page it started on.** The fetch translation is
//!   then resolved once, at block entry, exactly as the interpreter resolves it
//!   for the first instruction — so no fetch *inside* the block can miss the
//!   TLB, walk, or fault, and `charge(1)`/`charge(2)` per instruction is the
//!   whole fetch cost. Crossing a page would make every later instruction's
//!   cumulative tick column a guess. See [`Stop::Page`].
//! * **A load or store is the last guest instruction in its block.** Its
//!   access count is `1` or `bytes`, plus zero to three walk reads, none of it
//!   known at lift time. [`InsnStart::ticks`] is a *static* cumulative column,
//!   so an instruction whose charge is data-dependent may have nothing after
//!   it. The [`Opcode::LD`]/[`Opcode::ST`] therefore charges for itself — it
//!   is the only thing that knows how many accesses it made — and this
//!   frontend emits no charge for it at all. See [`Stop::Access`].
//!
//! That second rule is also why every load and store here is
//! [`MemOp::volatile`]: the access spends ticks and can fault, both
//! guest-visible, so dead-code elimination may not remove one whose value is
//! discarded — `lw x0, 0(a0)` really does read the bus.
//!
//! # Guest state: the slot numbering
//!
//! [`RegSlot`] is numbered by the frontend, and [`ir`](crate::ir)'s decision 3
//! requires it to cover the guest-visible state a fault needs — which on this
//! core is larger than `x[0..32]`:
//!
//! | Slot | State |
//! | --- | --- |
//! | `0..=31` | the integer registers `x0`..`x31` ([`x_slot`]) |
//! | `32` | the program counter ([`PC`]) |
//! | `33` | the `LR` reservation ([`RESERVATION`]) |
//!
//! `State::f`, `State::csrs` and `State::wfi` have no slots because nothing in
//! the subset can reach them; `State::cycles` is [`InsnStart::ticks`];
//! `State::debt` and `State::faults` are host bookkeeping rather than
//! architectural state.
//!
//! [`RESERVATION`] is in the numbering and is never bound to a temporary here,
//! and that is a statement rather than an oversight: a store in this subset
//! *does* break a reservation — `exec::store` clears it when the address
//! shares the reserved eight-byte block — but whether it does depends on the
//! run-time address, so it is the [`Opcode::ST`]'s own business, exactly as it
//! is the interpreter's `store()`'s. The slot exists so a later `A` frontend
//! and any consumer of a fault's state agree on its number.
//!
//! [`PC`] is bound only at the block's **exit boundary**, because at every
//! other boundary the PC is [`InsnStart::pc`], a constant.
//!
//! # Reading and writing guest registers
//!
//! The IR has no "read a guest register" op — the only channel between a block
//! and the architectural state is [`InsnStart::live`], which maps a slot to
//! the temporary holding it. This frontend therefore uses two conventions,
//! both expressed in ops the IR already defines:
//!
//! * **A write is a rebinding.** Nothing is emitted: the slot simply maps to
//!   the result temporary from here on, and the next boundary records it.
//! * **A read is [`Opcode::GET_SLOT`]**, naming its slot directly. A slot
//!   absent from a boundary's map is not dead: it means the slot's value is
//!   still in the CPU state and no temporary shadows it.
//!
//! `x0` is hard-wired zero, and both halves of that fold away here because the
//! register number is a decode constant: a read of `x0` becomes a zero
//! immediate, and a write to `x0` is not merely discarded — for a pure ALU
//! instruction the whole computation is skipped, since nothing observes it.
//!
//! # Termination
//!
//! Every block ends in [`Opcode::EXIT_TB`], preceded by the exit boundary that
//! carries the outgoing register map and the [`PC`] slot. Block chaining
//! ([`Opcode::GOTO_TB`], [`Opcode::LOOKUP_AND_GOTO`]) needs a successor-linking
//! design that does not exist yet, and inventing half of one here would be
//! worse than returning to the dispatcher.
//!
//! # Sources
//!
//! *The RISC-V Instruction Set Manual, Volume I: Unprivileged ISA*
//! (CC-BY-4.0), RV32I/RV64I base chapters: the shift-amount masking rule, the
//! `SLTIU` sign-then-compare rule, `JALR`'s cleared low bit, and the
//! instruction-address-misaligned condition on a taken branch or jump. No
//! emulator source of any licence was opened for any part of this file
//! (`ROADMAP.md` §1).

use alloc::vec::Vec;

use crate::core::error::{Error, Result};
use crate::core::value::Width;
use crate::ir::{
    AccessKind, Align, Block, BlockBuilder, Cond, Const, Endian, InsnStart, MemOp, MemSpace,
    Opcode, RegSlot, Sign, Temp, Type,
};

use super::isa::{self, Fmt, Op, Xlen};
use super::{Config, PAGE_MASK};

// ---------------------------------------------------------------------------
// The slot numbering
// ---------------------------------------------------------------------------

/// The slot holding integer register `x`*n*.
///
/// # Panics
///
/// Never: `n` is masked to five bits, because every caller derives it from a
/// five-bit instruction field.
#[inline]
#[must_use]
pub const fn x_slot(n: u32) -> RegSlot {
    RegSlot((n & 31) as u16)
}

/// The slot holding the program counter.
///
/// Bound only at a block's exit boundary; at every other boundary the PC is
/// [`InsnStart::pc`] and a temporary for it would be a second source of truth.
pub const PC: RegSlot = RegSlot(32);

/// The slot holding the `LR` reservation.
///
/// Never bound by this frontend — see the module docs. It is numbered here so
/// that the `A` frontend, the fault path and a snapshot consumer cannot
/// disagree about which slot it is.
pub const RESERVATION: RegSlot = RegSlot(33);

/// One past the highest slot this frontend numbers.
pub const SLOT_COUNT: u16 = 34;

// ---------------------------------------------------------------------------
// Inputs and outputs
// ---------------------------------------------------------------------------

/// Where the lifter reads guest instruction bytes.
///
/// Halfwords rather than words because that is the unit RISC-V fetches in and
/// the unit the interpreter charges for: `isa::is_32bit` is decided on the
/// first halfword, and a 32-bit instruction's two halves are two accesses.
///
/// Implemented for every `FnMut(u64) -> Option<u16>`, so a caller can pass a
/// closure over an address space, a snapshot, or a slice of bytes. `None`
/// means "cannot be read here" and ends the block ([`Stop::Unreadable`]) —
/// the lifter never invents an encoding.
pub trait InsnSource {
    /// The halfword at guest address `addr`, or `None` if it is unreadable.
    fn halfword(&mut self, addr: u64) -> Option<u16>;
}

impl<F: FnMut(u64) -> Option<u16>> InsnSource for F {
    #[inline]
    fn halfword(&mut self, addr: u64) -> Option<u16> {
        self(addr)
    }
}

/// Why a block stopped where it did.
///
/// Reported rather than inferred, because "the block is short" has six
/// different causes and only one of them is a gap in the subset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Stop {
    /// An encoding outside the subset. It was not lifted; the block's exit PC
    /// is its address, so the interpreter executes it next.
    Unsupported,
    /// A load or store, which was lifted and must be the block's last guest
    /// instruction because its tick charge is data-dependent (module docs).
    Access,
    /// A branch, `JAL` or `JALR`, which was lifted and transfers control.
    Transfer,
    /// The next instruction would leave the page the block started on.
    Page,
    /// The caller's instruction limit.
    Limit,
    /// The instruction bytes could not be read.
    Unreadable,
}

/// A lifted block, and what is true about it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Lifted {
    /// The block. Always ends in a terminator and always passes
    /// [`verify`](crate::ir::verify).
    pub block: Block,
    /// Why lifting stopped.
    pub stop: Stop,
    /// How many guest instructions were lifted. Zero is legal and means the
    /// block's first instruction was outside the subset.
    pub insns: usize,
}

/// How many guest instructions [`lift`] will take by default.
///
/// A block is bounded by its page anyway; this bounds a block of `nop`s in a
/// tight page and keeps one translation's cost predictable.
pub const MAX_INSNS: usize = 64;

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

/// Lift the guest instructions at `entry_pc` into a translation block.
///
/// Reads at most `max_insns` instructions, never leaves `entry_pc`'s page, and
/// always produces a well-formed block — including when nothing could be
/// lifted, in which case the block is just the exit boundary and a terminator
/// and `Lifted::insns` is zero.
///
/// # Errors
///
/// [`Error::Unimplemented`] for an RV32 configuration. RV32 keeps register
/// values sign-extended into 64 bits while addresses are truncated to 32
/// (`isa::Xlen`), which is a second lowering for every op here rather than a
/// flag; doing it badly would be worse than not doing it.
pub fn lift<S: InsnSource>(
    cfg: &Config,
    entry_pc: u64,
    src: &mut S,
    max_insns: usize,
) -> Result<Lifted> {
    if !matches!(cfg.xlen, Xlen::Rv64) {
        return Err(Error::Unimplemented("the RISC-V IR frontend is RV64 only"));
    }

    let mut lf = Lifter::new(cfg, entry_pc);
    let page = entry_pc & !PAGE_MASK;
    let mut pc = entry_pc;
    let mut insns = 0usize;

    let stop = loop {
        if insns >= max_insns {
            break Stop::Limit;
        }
        if pc & !PAGE_MASK != page {
            break Stop::Page;
        }
        let Some(low) = src.halfword(pc) else {
            break Stop::Unreadable;
        };
        // The fetch charge, straight off the encoding: one access per
        // halfword, which is what `exec::fetch` spends.
        let (word, len, fetch) = if isa::is_32bit(low) {
            if pc.wrapping_add(2) & !PAGE_MASK != page {
                break Stop::Page;
            }
            let Some(high) = src.halfword(pc.wrapping_add(2)) else {
                break Stop::Unreadable;
            };
            (u32::from(low) | (u32::from(high) << 16), 4u64, 2u64)
        } else if cfg.ext.c {
            // Volume I defines every compressed encoding as an alias for one
            // 32-bit instruction, so expansion is the whole of `C` here too.
            match isa::expand(low, cfg.xlen) {
                Some(word) => (word, 2u64, 1u64),
                None => break Stop::Unsupported,
            }
        } else {
            break Stop::Unsupported;
        };

        let next_pc = pc.wrapping_add(len);
        match lf.insn(word, pc, next_pc, fetch) {
            Flow::Rejected => break Stop::Unsupported,
            Flow::Continue => {
                insns += 1;
                pc = next_pc;
            }
            Flow::Access => {
                insns += 1;
                pc = next_pc;
                break Stop::Access;
            }
            Flow::Transfer => {
                insns += 1;
                pc = next_pc;
                break Stop::Transfer;
            }
        }
    };

    Ok(Lifted {
        block: lf.finish(pc),
        stop,
        insns,
    })
}

/// The block cache key: every configuration bit this lift depends on.
///
/// [`Block::key`] is the rest of the cache key beside the entry PC. Identical
/// guest bytes lift differently under a different `C` setting (`JALR`'s
/// alignment guarantee, and whether a 16-bit encoding is an instruction at
/// all) and under a different misalignment policy (the [`Align`] a memory op
/// carries), so both belong here or a cache returns the wrong translation.
fn key(cfg: &Config) -> u64 {
    let mut key = 0u64;
    if cfg.ext.c {
        key |= 1;
    }
    if cfg.misaligned {
        key |= 2;
    }
    if matches!(cfg.xlen, Xlen::Rv64) {
        key |= 4;
    }
    key
}

// ---------------------------------------------------------------------------
// The plan: what an encoding means, decided before anything is emitted
// ---------------------------------------------------------------------------

/// What lifting one instruction will emit.
///
/// Every encoding is classified — and every static precondition checked —
/// *before* a single op is emitted, so the emitter is total and a rejected
/// instruction leaves no debris in the block. Splitting it this way is not
/// bookkeeping: register reads must be materialized before the instruction's
/// boundary marker, which means the decision to lift at all has to come first.
#[derive(Debug, Clone, Copy)]
enum Plan {
    /// Integer computation writing `rd`.
    Alu(Alu),
    /// A load of `size`, extended per `sign`.
    Load { size: Width, sign: Sign },
    /// A store of `size`.
    Store { size: Width },
    /// A conditional branch to a statically known, statically aligned target.
    Branch { cond: Cond, target: u64 },
    /// `JAL` to a statically known, statically aligned target.
    Jal { target: u64 },
    /// `JALR`. Only planned on a core with `C`, where clearing the low bit
    /// makes the target aligned by construction.
    Jalr,
}

/// The shape of an integer computation, resolved down to IR opcodes.
///
/// Immediates are already sign-extended into 64 bits by `isa`, and shift
/// amounts are already range-checked, so nothing here can fail.
#[derive(Debug, Clone, Copy)]
enum Alu {
    /// A whole result known at lift time: `LUI`, and `AUIPC` because the PC is.
    Const(u64),
    /// `rd = rs1 op imm`.
    RegImm { op: Opcode, imm: u64 },
    /// `rd = (rs1 cond imm) as 0/1`.
    SetCondImm { cond: Cond, imm: u64 },
    /// `rd = rs1 op shamt`, shamt a decode constant below 64.
    ShiftImm { op: Opcode, shamt: u32 },
    /// `rd = rs1 op rs2`.
    RegReg { op: Opcode },
    /// `rd = (rs1 cond rs2) as 0/1`.
    SetCond { cond: Cond },
    /// `rd = rs1 op (rs2 & 63)` — Volume I masks a register shift amount to
    /// the register width, which is also the guard [`Opcode::SHL`] requires
    /// each frontend to emit for itself.
    ShiftReg { op: Opcode },
    /// `rd = sext32(trunc32(rs1) op imm)` — the `*W` immediate forms.
    WordImm { op: Opcode, imm: u32 },
    /// `rd = sext32(trunc32(rs1) op shamt)`, shamt below 32.
    WordShiftImm { op: Opcode, shamt: u32 },
    /// `rd = sext32(trunc32(rs1) op trunc32(rs2))`.
    WordReg { op: Opcode },
    /// `rd = sext32(trunc32(rs1) op (rs2 & 31))`.
    WordShiftReg { op: Opcode },
}

/// Which register fields an encoding reads, from [`isa::TABLE`]'s `fmt`
/// column.
///
/// The operand shape is already described once, for the disassembler; reading
/// it here is what keeps this frontend a consumer of that description rather
/// than a fourth copy of it (CLAUDE.md, "CPU cores").
const fn reads(fmt: Fmt) -> (bool, bool) {
    match fmt {
        // `Load` is `rd, imm(rs1)`, which is every load and `JALR`.
        Fmt::I | Fmt::Shift | Fmt::Load => (true, false),
        Fmt::R | Fmt::Store | Fmt::Branch => (true, true),
        _ => (false, false),
    }
}

/// The alignment a jump or branch target must have.
///
/// Volume I: without `C` an instruction address is four-byte aligned, and a
/// taken transfer to anything else raises instruction-address-misaligned *at
/// the transfer*. With `C` it is two-byte aligned.
const fn target_align_mask(cfg: &Config) -> u64 {
    if cfg.ext.c { 1 } else { 3 }
}

/// Decide what an encoding means, or reject it.
#[allow(clippy::too_many_lines)]
fn classify(cfg: &Config, op: Op, word: u32, pc: u64) -> Option<Plan> {
    let imm_i = isa::imm_i(word) as u64;
    let plan = match op {
        // -- LUI / AUIPC: both fold to a constant, AUIPC because the PC is one
        Op::Lui => Plan::Alu(Alu::Const(isa::imm_u(word) as u64)),
        Op::Auipc => Plan::Alu(Alu::Const(pc.wrapping_add(isa::imm_u(word) as u64))),

        // -- register-immediate ------------------------------------------
        Op::Addi => Plan::Alu(Alu::RegImm {
            op: Opcode::ADD,
            imm: imm_i,
        }),
        Op::Xori => Plan::Alu(Alu::RegImm {
            op: Opcode::XOR,
            imm: imm_i,
        }),
        Op::Ori => Plan::Alu(Alu::RegImm {
            op: Opcode::OR,
            imm: imm_i,
        }),
        Op::Andi => Plan::Alu(Alu::RegImm {
            op: Opcode::AND,
            imm: imm_i,
        }),
        Op::Slti => Plan::Alu(Alu::SetCondImm {
            cond: Cond::LtS,
            imm: imm_i,
        }),
        // Volume I: the immediate is sign-extended *first* and compared as
        // unsigned, which is what makes `sltiu rd, rs, 1` the "is zero" idiom.
        Op::Sltiu => Plan::Alu(Alu::SetCondImm {
            cond: Cond::LtU,
            imm: imm_i,
        }),
        Op::Slli | Op::Srli | Op::Srai => {
            let shamt = isa::shamt(word);
            // A shift amount at or above the register width is not an
            // instruction on RV64; the interpreter raises illegal-instruction,
            // so the block ends here rather than lifting a trap.
            if shamt >= 64 {
                return None;
            }
            Plan::Alu(Alu::ShiftImm {
                op: shift_opcode(op),
                shamt,
            })
        }

        // -- register-register -------------------------------------------
        Op::Add => Plan::Alu(Alu::RegReg { op: Opcode::ADD }),
        Op::Sub => Plan::Alu(Alu::RegReg { op: Opcode::SUB }),
        Op::Xor => Plan::Alu(Alu::RegReg { op: Opcode::XOR }),
        Op::Or => Plan::Alu(Alu::RegReg { op: Opcode::OR }),
        Op::And => Plan::Alu(Alu::RegReg { op: Opcode::AND }),
        Op::Slt => Plan::Alu(Alu::SetCond { cond: Cond::LtS }),
        Op::Sltu => Plan::Alu(Alu::SetCond { cond: Cond::LtU }),
        Op::Sll | Op::Srl | Op::Sra => Plan::Alu(Alu::ShiftReg {
            op: shift_opcode(op),
        }),

        // -- RV64 word forms ---------------------------------------------
        Op::Addiw => Plan::Alu(Alu::WordImm {
            op: Opcode::ADD,
            imm: imm_i as u32,
        }),
        Op::Slliw | Op::Srliw | Op::Sraiw => Plan::Alu(Alu::WordShiftImm {
            op: shift_opcode(op),
            // The encoding fixes bit 25, so this is already below 32; the mask
            // says so rather than relying on the reader to check the table.
            shamt: isa::shamt(word) & 31,
        }),
        Op::Addw => Plan::Alu(Alu::WordReg { op: Opcode::ADD }),
        Op::Subw => Plan::Alu(Alu::WordReg { op: Opcode::SUB }),
        Op::Sllw | Op::Srlw | Op::Sraw => Plan::Alu(Alu::WordShiftReg {
            op: shift_opcode(op),
        }),

        // -- loads and stores ---------------------------------------------
        Op::Lb => Plan::Load {
            size: Width::U8,
            sign: Sign::Signed,
        },
        Op::Lbu => Plan::Load {
            size: Width::U8,
            sign: Sign::Unsigned,
        },
        Op::Lh => Plan::Load {
            size: Width::U16,
            sign: Sign::Signed,
        },
        Op::Lhu => Plan::Load {
            size: Width::U16,
            sign: Sign::Unsigned,
        },
        Op::Lw => Plan::Load {
            size: Width::U32,
            sign: Sign::Signed,
        },
        Op::Lwu => Plan::Load {
            size: Width::U32,
            sign: Sign::Unsigned,
        },
        Op::Ld => Plan::Load {
            size: Width::U64,
            sign: Sign::Signed,
        },
        Op::Sb => Plan::Store { size: Width::U8 },
        Op::Sh => Plan::Store { size: Width::U16 },
        Op::Sw => Plan::Store { size: Width::U32 },
        Op::Sd => Plan::Store { size: Width::U64 },

        // -- control flow --------------------------------------------------
        Op::Beq | Op::Bne | Op::Blt | Op::Bge | Op::Bltu | Op::Bgeu => {
            let target = pc.wrapping_add(isa::imm_b(word) as u64);
            // A misaligned target only faults when the branch is *taken*,
            // which is a run-time fact; rather than lift a conditional trap,
            // the block ends before a branch that could raise one.
            if target & target_align_mask(cfg) != 0 {
                return None;
            }
            Plan::Branch {
                cond: branch_cond(op),
                target,
            }
        }
        Op::Jal => {
            let target = pc.wrapping_add(isa::imm_j(word) as u64);
            if target & target_align_mask(cfg) != 0 {
                return None;
            }
            Plan::Jal { target }
        }
        // Volume I clears the computed target's low bit rather than checking
        // it, so on a core with `C` — where two-byte alignment is enough — the
        // target can never be misaligned and the check is discharged here.
        // Without `C` it is a run-time test this IR has no way to express, so
        // `JALR` is out of the subset on such a core.
        Op::Jalr if cfg.ext.c => Plan::Jalr,

        _ => return None,
    };
    Some(plan)
}

/// The IR opcode for a shift, in either the doubleword or the word family.
const fn shift_opcode(op: Op) -> Opcode {
    match op {
        Op::Slli | Op::Sll | Op::Slliw | Op::Sllw => Opcode::SHL,
        Op::Srai | Op::Sra | Op::Sraiw | Op::Sraw => Opcode::SAR,
        _ => Opcode::SHR,
    }
}

/// The IR condition a branch tests.
const fn branch_cond(op: Op) -> Cond {
    match op {
        Op::Beq => Cond::Eq,
        Op::Bne => Cond::Ne,
        Op::Blt => Cond::LtS,
        Op::Bge => Cond::GeS,
        Op::Bltu => Cond::LtU,
        _ => Cond::GeU,
    }
}

// ---------------------------------------------------------------------------
// The lifter
// ---------------------------------------------------------------------------

/// What lifting one instruction did to the block.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Flow {
    /// Nothing was emitted; the instruction is outside the subset.
    Rejected,
    /// Lifted; the block may continue.
    Continue,
    /// Lifted, and it made a memory access, so the block must end.
    Access,
    /// Lifted, and it transferred control, so the block must end.
    Transfer,
}

/// One translation in progress.
struct Lifter<'a> {
    cfg: &'a Config,
    b: BlockBuilder,
    /// Which temporary holds each integer register, where one does. `x[0]` is
    /// never bound: the register is hard-wired zero.
    x: [Option<Temp>; 32],
    /// The block's one zero immediate, shared by every `x0` read.
    zero: Option<Temp>,
    /// Ticks charged so far, counted from block entry.
    ticks: u64,
    /// The temporary holding the exit PC, once a transfer has set one.
    pc_out: Option<Temp>,
    /// The exit PC's value where it is a constant, for the exit boundary's
    /// `pc` field.
    static_exit: Option<u64>,
}

impl<'a> Lifter<'a> {
    fn new(cfg: &'a Config, entry_pc: u64) -> Lifter<'a> {
        Lifter {
            cfg,
            b: BlockBuilder::new(entry_pc, key(cfg)),
            x: [None; 32],
            zero: None,
            ticks: 0,
            pc_out: None,
            static_exit: None,
        }
    }

    /// Materialize a 64-bit constant.
    fn konst(&mut self, value: u64) -> Temp {
        self.b.imm(Type::I64, Const::Int(u128::from(value)))
    }

    /// Materialize a 32-bit constant, for the `*W` word family.
    fn konst32(&mut self, value: u32) -> Temp {
        self.b.imm(Type::I32, Const::Int(u128::from(value)))
    }

    /// The block's shared zero.
    fn zero(&mut self) -> Temp {
        match self.zero {
            Some(t) => t,
            None => {
                let t = self.konst(0);
                self.zero = Some(t);
                t
            }
        }
    }

    /// Read guest register `n` into a temporary.
    ///
    /// `x0` folds to a constant zero — the register number is a decode
    /// constant, so the hard-wired-zero rule costs nothing at run time. Any
    /// other register that no temporary yet shadows is read with
    /// [`Opcode::GET_SLOT`].
    fn read_x(&mut self, n: u32) -> Temp {
        if n == 0 {
            return self.zero();
        }
        match self.x[n as usize] {
            Some(t) => t,
            None => {
                let t = self.b.get_slot(Type::I64, RegSlot(n as u16));
                self.x[n as usize] = Some(t);
                t
            }
        }
    }

    /// Bind guest register `n` to a temporary.
    ///
    /// A write to `x0` is discarded, and because `n` is a decode constant the
    /// interpreter's write guard has no run-time cost here at all.
    fn write_x(&mut self, n: u32, t: Temp) {
        if n != 0 {
            self.x[n as usize] = Some(t);
        }
    }

    /// An operand the plan needs; the fallback cannot happen, because
    /// [`reads`] and [`classify`] agree by construction on which fields an
    /// encoding uses.
    fn need(&mut self, t: Option<Temp>) -> Temp {
        match t {
            Some(t) => t,
            None => self.zero(),
        }
    }

    /// The register slots a temporary currently shadows, in slot order.
    ///
    /// Slot order rather than binding order: `ROADMAP.md` §0's determinism
    /// rule reaches the IR too, and this vector is hashed by anything that
    /// hashes a block.
    fn live_regs(&self) -> Vec<(RegSlot, Temp)> {
        let mut live = Vec::new();
        for (n, temp) in self.x.iter().enumerate() {
            if let Some(t) = temp {
                live.push((x_slot(n as u32), *t));
            }
        }
        live
    }

    /// The misalignment policy a memory op carries.
    ///
    /// [`Align::Split`] is x86's rule — translate every piece before writing
    /// any — and RISC-V's is not that: `exec::store` translates and writes
    /// byte by byte, so a fault on the second page leaves the first half
    /// written. What the ISA actually says is that the implementation either
    /// performs a misaligned access or raises, which is exactly
    /// [`Align::None`] and [`Align::Fault`].
    const fn align(&self) -> Align {
        if self.cfg.misaligned {
            Align::None
        } else {
            Align::Fault
        }
    }

    /// Lift one instruction.
    fn insn(&mut self, word: u32, pc: u64, next_pc: u64, fetch: u64) -> Flow {
        let Some(row) = isa::decode(word, self.cfg.xlen) else {
            return Flow::Rejected;
        };
        // The subset is the base integer set. Anything else is a whole
        // extension away and ends the block; a core built without the
        // extension would raise illegal-instruction anyway.
        if !matches!(row.ext, isa::Ext::I) {
            return Flow::Rejected;
        }
        let Some(plan) = classify(self.cfg, row.op, word, pc) else {
            return Flow::Rejected;
        };

        let rd = isa::rd(word);
        let rs1 = isa::rs1(word);
        let rs2 = isa::rs2(word);

        // `x0` as a pure ALU destination: nothing observes the result, so the
        // computation — and with it the operand reads — folds away entirely.
        // Only pure computation folds; a load still makes its access.
        let folded = rd == 0 && matches!(plan, Plan::Alu(_));

        // Operands are materialized *before* the boundary, so the boundary's
        // live map names them and a fault here reconstructs the architectural
        // register from the temporary that shadows it.
        let (mut a, mut b) = (None, None);
        if !folded {
            let (r1, r2) = reads(row.fmt);
            if r1 {
                a = Some(self.read_x(rs1));
            }
            if r2 {
                b = Some(self.read_x(rs2));
            }
        }

        let live = self.live_regs();
        self.b.insn_start(InsnStart {
            pc,
            next_pc,
            ticks: self.ticks,
            live,
        });
        self.b.charge(fetch);
        self.ticks += fetch;

        if folded {
            return Flow::Continue;
        }

        match plan {
            Plan::Alu(alu) => {
                let v = self.emit_alu(alu, a, b);
                self.write_x(rd, v);
                Flow::Continue
            }
            Plan::Load { size, sign } => {
                let base = self.need(a);
                let off = self.konst(isa::imm_i(word) as u64);
                let addr = self.b.binary(Opcode::ADD, Type::I64, base, off);
                let mem = MemOp {
                    size,
                    sign,
                    space: MemSpace::MEM,
                    seg: None,
                    endian: Endian::Little,
                    align: self.align(),
                    kind: AccessKind::Load,
                    // The access spends a tick and can fault, both
                    // guest-visible, so DCE may not remove it (module docs).
                    volatile: true,
                };
                let v = self.b.load(Type::I64, addr, mem);
                self.write_x(rd, v);
                Flow::Access
            }
            Plan::Store { size } => {
                let base = self.need(a);
                let value = self.need(b);
                let off = self.konst(isa::imm_s(word) as u64);
                let addr = self.b.binary(Opcode::ADD, Type::I64, base, off);
                let mem = MemOp {
                    size,
                    sign: Sign::Unsigned,
                    space: MemSpace::MEM,
                    seg: None,
                    endian: Endian::Little,
                    align: self.align(),
                    kind: AccessKind::Store,
                    volatile: true,
                };
                self.b.store(Type::I64, addr, value, mem);
                Flow::Access
            }
            Plan::Branch { cond, target } => {
                let lhs = self.need(a);
                let rhs = self.need(b);
                let taken = self.b.setcond(cond, Type::I64, lhs, rhs);
                let then = self.konst(target);
                let other = self.konst(next_pc);
                // MOVCOND's operands are the condition and then the two
                // values, in `cond ? then : else` order.
                let sel = self
                    .b
                    .emit(Opcode::MOVCOND, Type::I64, &[taken, then, other]);
                self.pc_out = Some(sel);
                Flow::Transfer
            }
            Plan::Jal { target } => {
                if rd != 0 {
                    let link = self.konst(next_pc);
                    self.write_x(rd, link);
                }
                let t = self.konst(target);
                self.pc_out = Some(t);
                self.static_exit = Some(target);
                Flow::Transfer
            }
            Plan::Jalr => {
                // The target is computed from `rs1` before the link is bound,
                // which is what makes `jalr ra, 0(ra)` correct.
                let base = self.need(a);
                let off = self.konst(isa::imm_i(word) as u64);
                let sum = self.b.binary(Opcode::ADD, Type::I64, base, off);
                let mask = self.konst(!1u64);
                let target = self.b.binary(Opcode::AND, Type::I64, sum, mask);
                if rd != 0 {
                    let link = self.konst(next_pc);
                    self.write_x(rd, link);
                }
                self.pc_out = Some(target);
                Flow::Transfer
            }
        }
    }

    /// Emit an integer computation and return its result.
    fn emit_alu(&mut self, alu: Alu, a: Option<Temp>, b: Option<Temp>) -> Temp {
        match alu {
            Alu::Const(v) => self.konst(v),
            Alu::RegImm { op, imm } => {
                let lhs = self.need(a);
                let rhs = self.konst(imm);
                self.b.binary(op, Type::I64, lhs, rhs)
            }
            Alu::SetCondImm { cond, imm } => {
                let lhs = self.need(a);
                let rhs = self.konst(imm);
                let bit = self.b.setcond(cond, Type::I64, lhs, rhs);
                self.b.unary(Opcode::EXT_Z, Type::I64, bit)
            }
            Alu::ShiftImm { op, shamt } => {
                let lhs = self.need(a);
                let rhs = self.konst(u64::from(shamt));
                self.b.binary(op, Type::I64, lhs, rhs)
            }
            Alu::RegReg { op } => {
                let lhs = self.need(a);
                let rhs = self.need(b);
                self.b.binary(op, Type::I64, lhs, rhs)
            }
            Alu::SetCond { cond } => {
                let lhs = self.need(a);
                let rhs = self.need(b);
                let bit = self.b.setcond(cond, Type::I64, lhs, rhs);
                self.b.unary(Opcode::EXT_Z, Type::I64, bit)
            }
            Alu::ShiftReg { op } => {
                let lhs = self.need(a);
                let raw = self.need(b);
                let mask = self.konst(63);
                let sh = self.b.binary(Opcode::AND, Type::I64, raw, mask);
                self.b.binary(op, Type::I64, lhs, sh)
            }
            Alu::WordImm { op, imm } => {
                let lhs = self.need(a);
                let lhs32 = self.b.unary(Opcode::TRUNC, Type::I32, lhs);
                let rhs32 = self.konst32(imm);
                let r = self.b.binary(op, Type::I32, lhs32, rhs32);
                self.b.unary(Opcode::EXT_S, Type::I64, r)
            }
            Alu::WordShiftImm { op, shamt } => {
                let lhs = self.need(a);
                let lhs32 = self.b.unary(Opcode::TRUNC, Type::I32, lhs);
                let sh = self.konst32(shamt);
                let r = self.b.binary(op, Type::I32, lhs32, sh);
                self.b.unary(Opcode::EXT_S, Type::I64, r)
            }
            Alu::WordReg { op } => {
                let lhs = self.need(a);
                let rhs = self.need(b);
                let lhs32 = self.b.unary(Opcode::TRUNC, Type::I32, lhs);
                let rhs32 = self.b.unary(Opcode::TRUNC, Type::I32, rhs);
                let r = self.b.binary(op, Type::I32, lhs32, rhs32);
                self.b.unary(Opcode::EXT_S, Type::I64, r)
            }
            Alu::WordShiftReg { op } => {
                let lhs = self.need(a);
                let raw = self.need(b);
                let lhs32 = self.b.unary(Opcode::TRUNC, Type::I32, lhs);
                let raw32 = self.b.unary(Opcode::TRUNC, Type::I32, raw);
                let mask = self.konst32(31);
                let sh = self.b.binary(Opcode::AND, Type::I32, raw32, mask);
                let r = self.b.binary(op, Type::I32, lhs32, sh);
                self.b.unary(Opcode::EXT_S, Type::I64, r)
            }
        }
    }

    /// Close the block: the exit boundary, then the terminator.
    ///
    /// The exit boundary is a boundary that begins no instruction. It carries
    /// the outgoing register map and the [`PC`] slot, which is the only thing
    /// that tells a dispatcher where to resume; its `pc` field is the exit PC
    /// where that is a constant, and the program-order continuation otherwise.
    fn finish(mut self, program_order_pc: u64) -> Block {
        let pc = match self.pc_out {
            Some(t) => t,
            None => self.konst(program_order_pc),
        };
        let mut live = self.live_regs();
        live.push((PC, pc));
        let at = self.static_exit.unwrap_or(program_order_pc);
        self.b.insn_start(InsnStart {
            pc: at,
            next_pc: at,
            ticks: self.ticks,
            live,
        });
        self.b.exit_tb();
        self.b.finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::space::{AddressSpace, RamStore, Region};
    use crate::cpu::riscv::Hart;
    use crate::cpu::riscv::csr::Extensions;
    use crate::ir::verify;
    use alloc::sync::Arc;
    use alloc::vec;

    // -- assembly ---------------------------------------------------------
    //
    // Encoders rather than pasted hex, so a test says what it means. `isa`'s
    // own tests already prove they agree with the decoder.

    const fn i_type(opcode: u32, funct3: u32, rd: u32, rs1: u32, imm: i32) -> u32 {
        opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | (((imm as u32) & 0xfff) << 20)
    }
    const fn r_type(opcode: u32, funct3: u32, funct7: u32, rd: u32, rs1: u32, rs2: u32) -> u32 {
        opcode | (rd << 7) | (funct3 << 12) | (rs1 << 15) | (rs2 << 20) | (funct7 << 25)
    }
    const fn s_type(funct3: u32, rs1: u32, rs2: u32, imm: i32) -> u32 {
        let imm = imm as u32;
        0x23 | ((imm & 0x1f) << 7)
            | (funct3 << 12)
            | (rs1 << 15)
            | (rs2 << 20)
            | (((imm >> 5) & 0x7f) << 25)
    }
    const fn b_type(funct3: u32, rs1: u32, rs2: u32, imm: i32) -> u32 {
        let imm = imm as u32;
        0x63 | (((imm >> 11) & 1) << 7)
            | (((imm >> 1) & 0xf) << 8)
            | (funct3 << 12)
            | (rs1 << 15)
            | (rs2 << 20)
            | (((imm >> 5) & 0x3f) << 25)
            | (((imm >> 12) & 1) << 31)
    }
    const fn j_type(rd: u32, imm: i32) -> u32 {
        let imm = imm as u32;
        0x6f | (rd << 7)
            | (((imm >> 12) & 0xff) << 12)
            | (((imm >> 11) & 1) << 20)
            | (((imm >> 1) & 0x3ff) << 21)
            | (((imm >> 20) & 1) << 31)
    }
    const fn addi(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0x13, 0, rd, rs1, imm)
    }
    const fn add(rd: u32, rs1: u32, rs2: u32) -> u32 {
        r_type(0x33, 0, 0, rd, rs1, rs2)
    }
    const fn sub(rd: u32, rs1: u32, rs2: u32) -> u32 {
        r_type(0x33, 0, 0x20, rd, rs1, rs2)
    }
    const fn sll(rd: u32, rs1: u32, rs2: u32) -> u32 {
        r_type(0x33, 1, 0, rd, rs1, rs2)
    }
    const fn slt(rd: u32, rs1: u32, rs2: u32) -> u32 {
        r_type(0x33, 2, 0, rd, rs1, rs2)
    }
    const fn addw(rd: u32, rs1: u32, rs2: u32) -> u32 {
        r_type(0x3b, 0, 0, rd, rs1, rs2)
    }
    const fn slli(rd: u32, rs1: u32, shamt: u32) -> u32 {
        i_type(0x13, 1, rd, rs1, shamt as i32)
    }
    const fn lui(rd: u32, imm: u32) -> u32 {
        0x37 | (rd << 7) | (imm & 0xffff_f000)
    }
    const fn auipc(rd: u32, imm: u32) -> u32 {
        0x17 | (rd << 7) | (imm & 0xffff_f000)
    }
    const fn lb(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0x03, 0, rd, rs1, imm)
    }
    const fn lwu(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0x03, 6, rd, rs1, imm)
    }
    const fn ld(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0x03, 3, rd, rs1, imm)
    }
    const fn sd(rs1: u32, rs2: u32, imm: i32) -> u32 {
        s_type(3, rs1, rs2, imm)
    }
    const fn beq(rs1: u32, rs2: u32, imm: i32) -> u32 {
        b_type(0, rs1, rs2, imm)
    }
    const fn jalr(rd: u32, rs1: u32, imm: i32) -> u32 {
        i_type(0x67, 0, rd, rs1, imm)
    }
    const ECALL: u32 = 0x0000_0073;
    const MUL: u32 = 0x0230_02b3; // mul x5, x0, x0

    // -- harness ----------------------------------------------------------

    /// Where the test programs live. Deliberately page-aligned and far from
    /// `0x8000_0000`, which `lui` would sign-extend.
    const BASE: u64 = 0x2000_0000;

    /// A program in memory, as the lifter reads it.
    struct Bytes {
        base: u64,
        words: Vec<u32>,
    }

    impl InsnSource for Bytes {
        fn halfword(&mut self, addr: u64) -> Option<u16> {
            let off = addr.checked_sub(self.base)?;
            let word = *self.words.get((off / 4) as usize)?;
            Some(if off % 4 == 0 {
                word as u16
            } else {
                (word >> 16) as u16
            })
        }
    }

    /// Lift a program at [`BASE`], asserting that the verifier accepts it.
    ///
    /// Every test goes through here, which is how "verify accepts every block
    /// this frontend produces" is asserted everywhere rather than once.
    fn lift_at(cfg: &Config, base: u64, words: &[u32]) -> Lifted {
        let mut src = Bytes {
            base,
            words: words.to_vec(),
        };
        let lifted = lift(cfg, base, &mut src, MAX_INSNS).expect("RV64 lifts");
        verify(&lifted.block).unwrap_or_else(|e| panic!("{e}\n{}", lifted.block));
        lifted
    }

    fn rv64i(words: &[u32]) -> Lifted {
        lift_at(&Config::rv64i(), BASE, words)
    }

    /// The ops in a block, as mnemonics, so a test can say what it expects.
    fn ops(block: &Block) -> Vec<&'static str> {
        block.insts().iter().map(|i| i.op.name()).collect()
    }

    /// What the interpreter charges for the first `n` instructions of the same
    /// program — the oracle for every tick assertion here.
    fn interpreter_ticks(cfg: Config, words: &[u32], n: usize) -> u64 {
        let ram = Arc::new(RamStore::new(0x1_0000));
        for (w, word) in words.iter().enumerate() {
            for (k, byte) in word.to_le_bytes().iter().enumerate() {
                ram.write_u8(w as u64 * 4 + k as u64, *byte).unwrap();
            }
        }
        let space = AddressSpace::new("mem", 64);
        space.topology().map(Region::ram("ram", ram), BASE).unwrap();
        let hart = Hart::new(cfg.with_reset_vector(BASE));
        hart.attach_space(Arc::new(space));
        for _ in 0..n {
            hart.step();
        }
        hart.cycles()
    }

    /// The cumulative tick column at the block's exit boundary.
    fn block_ticks(block: &Block) -> u64 {
        block.marks().last().expect("a block has boundaries").ticks
    }

    // -- end to end --------------------------------------------------------

    /// A guest whose whole state is thirty-four slots and no memory.
    ///
    /// The point of this harness is that it knows nothing about RISC-V: the
    /// slot numbering is the frontend's, and the backend treats it as opaque.
    #[derive(Debug, Default)]
    struct Slots {
        state: alloc::collections::BTreeMap<u16, u128>,
        ticks: u64,
        boundaries: Vec<u64>,
    }

    impl crate::ir::IrHost for Slots {
        fn read_slot(&mut self, slot: crate::ir::RegSlot) -> u128 {
            self.state.get(&slot.0).copied().unwrap_or(0)
        }

        fn write_slot(&mut self, slot: crate::ir::RegSlot, value: u128) {
            self.state.insert(slot.0, value);
        }

        fn load(
            &mut self,
            _mem: &crate::ir::MemOp,
            _addr: u64,
        ) -> crate::core::space::MemResult<u64> {
            Err(crate::core::error::BusError::Unassigned)
        }

        fn store(
            &mut self,
            _mem: &crate::ir::MemOp,
            _addr: u64,
            _value: u64,
        ) -> crate::core::space::MemResult {
            Err(crate::core::error::BusError::Unassigned)
        }

        fn charge(&mut self, ticks: u64) {
            self.ticks += ticks;
        }

        fn insn_start(&mut self, mark: &InsnStart) {
            self.boundaries.push(mark.pc);
        }
    }

    #[test]
    fn a_lifted_block_verifies_and_then_runs_on_the_portable_backend() {
        // The whole phase-5 path in one test: guest bytes in, IR out, the
        // verifier accepts it, the backend executes it, and the answer is the
        // one the guest's own semantics demand.
        let l = rv64i(&[addi(5, 0, 7), addi(6, 5, 3), ECALL]);
        assert_eq!(l.insns, 2);
        verify(&l.block).expect("a lifted block must verify");

        let mut host = Slots::default();
        let outcome = crate::ir::Interp::new()
            .run(&l.block, &mut host)
            .expect("the block executes");
        assert_eq!(outcome, crate::ir::Outcome::Exit);

        // x5 = 0 + 7, x6 = x5 + 3. Published at the exit boundary, which is
        // what makes a write a rebinding rather than a store.
        assert_eq!(host.state.get(&5), Some(&7));
        assert_eq!(host.state.get(&6), Some(&10));
        // Two instructions, two halfword fetches each: the same four ticks the
        // interpreter charges for the same two instructions.
        assert_eq!(host.ticks, 4);
        assert_eq!(
            host.ticks,
            interpreter_ticks(Config::rv64i(), &[addi(5, 0, 7), addi(6, 5, 3)], 2)
        );
    }

    #[test]
    fn dead_code_elimination_preserves_a_lifted_block() {
        // Three pieces written independently — the frontend, the pass, and the
        // backend — meeting for the first time. A slot read whose result is
        // named live at a boundary must survive DCE, or the block stops being
        // able to reconstruct architectural state at a fault.
        let l = rv64i(&[addi(5, 0, 7), addi(6, 5, 3), ECALL]);
        let lean = crate::ir::eliminate_dead_code(&l.block);
        verify(&lean).expect("an optimised block must still verify");

        let mut before = Slots::default();
        let mut after = Slots::default();
        let out_before = crate::ir::Interp::new().run(&l.block, &mut before).unwrap();
        let out_after = crate::ir::Interp::new().run(&lean, &mut after).unwrap();

        assert_eq!(out_before, out_after);
        assert_eq!(before.state, after.state);
        assert_eq!(
            before.ticks, after.ticks,
            "DCE may not change the tick count"
        );
    }

    // -- the subset --------------------------------------------------------

    #[test]
    fn a_register_immediate_alu_op_lifts_to_a_read_a_constant_and_an_add() {
        let l = rv64i(&[addi(5, 1, 7), ECALL]);
        assert_eq!(l.insns, 1);
        assert_eq!(l.stop, Stop::Unsupported);
        assert_eq!(
            ops(&l.block),
            // The read of x1 precedes the boundary, so the boundary's live map
            // can name it; the exit boundary's constant is the resume PC.
            vec![
                "get_slot",   // x1 in
                "insn_start", //
                "charge",     // two halfword fetches
                "mov",        // the immediate
                "add",        //
                "mov",        // the exit PC
                "insn_start", // the exit boundary
                "exit_tb",
            ]
        );
        // x5 now lives in the add's result, and x1 in the read.
        let exit = l.block.marks().last().unwrap();
        let slots: Vec<u16> = exit.live.iter().map(|(s, _)| s.0).collect();
        assert_eq!(slots, vec![1, 5, PC.0]);
    }

    #[test]
    fn register_register_and_word_forms_lift() {
        let l = rv64i(&[add(5, 1, 2), sub(6, 1, 2), addw(7, 1, 2), ECALL]);
        assert_eq!(l.insns, 3);
        let names = ops(&l.block);
        assert!(names.contains(&"add"), "{names:?}");
        assert!(names.contains(&"sub"), "{names:?}");
        // The word form truncates, operates in i32, and sign-extends back.
        assert!(names.contains(&"trunc"), "{names:?}");
        assert!(names.contains(&"ext_s"), "{names:?}");
        let word_add = l
            .block
            .insts()
            .iter()
            .find(|i| i.op == Opcode::ADD && i.ty == Type::I32)
            .expect("addw computes in i32");
        assert_eq!(l.block.type_of(word_add.dst.unwrap()), Some(Type::I32));
    }

    #[test]
    fn a_register_shift_masks_its_amount_to_the_register_width() {
        // Volume I masks a register shift amount to xlen-1, and Opcode::SHL is
        // undefined out of range, so the guard has to be explicit.
        let l = rv64i(&[sll(5, 1, 2), ECALL]);
        let and = l
            .block
            .insts()
            .iter()
            .find(|i| i.op == Opcode::AND)
            .expect("the shift amount is masked");
        let mask = l.block.srcs(
            l.block
                .insts()
                .iter()
                .position(|i| core::ptr::eq(i, and))
                .unwrap(),
        )[1];
        let def = l
            .block
            .insts()
            .iter()
            .find(|i| i.dst == Some(mask))
            .expect("the mask is a constant");
        assert_eq!(def.imm, Some(Const::Int(63)));
    }

    #[test]
    fn a_shift_by_an_immediate_needs_no_guard_and_an_out_of_range_one_is_rejected() {
        let l = rv64i(&[slli(5, 1, 63), ECALL]);
        assert_eq!(l.insns, 1);
        assert!(!ops(&l.block).contains(&"and"));
        // shamt 64 is not an RV64 instruction at all: the interpreter raises
        // illegal-instruction, so the block ends rather than lifting a trap.
        let bad = 0x13 | (5 << 7) | (1 << 12) | (1 << 15) | (64 << 20);
        let l = rv64i(&[bad, ECALL]);
        assert_eq!(l.insns, 0);
        assert_eq!(l.stop, Stop::Unsupported);
    }

    #[test]
    fn set_less_than_widens_the_one_bit_result() {
        let l = rv64i(&[slt(5, 1, 2), ECALL]);
        let names = ops(&l.block);
        assert!(names.contains(&"setcond"), "{names:?}");
        assert!(names.contains(&"ext_z"), "{names:?}");
        let cmp = l
            .block
            .insts()
            .iter()
            .find(|i| i.op == Opcode::SETCOND)
            .unwrap();
        assert_eq!(cmp.cond, Some(Cond::LtS));
        assert_eq!(l.block.type_of(cmp.dst.unwrap()), Some(Type::I1));
    }

    #[test]
    fn lui_and_auipc_fold_to_constants() {
        let l = rv64i(&[lui(5, 0x1234_5000), auipc(6, 0x1000), ECALL]);
        assert_eq!(l.insns, 2);
        // Neither reads a register, and neither computes anything: AUIPC's
        // addend is the PC, which the lifter knows.
        assert!(!ops(&l.block).contains(&"add"));
        let constants: Vec<u128> = l
            .block
            .insts()
            .iter()
            .filter(|i| i.op == Opcode::MOV)
            .filter_map(|i| i.imm.map(Const::bits))
            .collect();
        assert!(constants.contains(&0x1234_5000), "{constants:x?}");
        assert!(
            constants.contains(&u128::from(BASE + 4 + 0x1000)),
            "{constants:x?}"
        );
    }

    // -- x0 ----------------------------------------------------------------

    #[test]
    fn a_write_to_x0_folds_the_whole_computation_away() {
        let l = rv64i(&[add(0, 1, 2), ECALL]);
        assert_eq!(l.insns, 1);
        // No add, and not even the reads of x1 and x2: nothing observes them.
        assert_eq!(
            ops(&l.block),
            vec!["insn_start", "charge", "mov", "insn_start", "exit_tb"]
        );
        let exit = l.block.marks().last().unwrap();
        assert_eq!(exit.live.len(), 1, "only the PC is live");
        assert_eq!(exit.live[0].0, PC);
    }

    #[test]
    fn a_read_of_x0_is_a_zero_constant_shared_across_the_block() {
        let l = rv64i(&[add(5, 0, 0), add(6, 0, 0), ECALL]);
        assert_eq!(l.insns, 2);
        // One zero, not four reads: x0 is hard-wired, and the number is a
        // decode constant.
        let zeros = l
            .block
            .insts()
            .iter()
            .filter(|i| i.imm == Some(Const::Int(0)))
            .count();
        assert_eq!(zeros, 1);
        // x0 never appears in a live map, because no temporary shadows it.
        for mark in l.block.marks() {
            assert!(mark.live.iter().all(|(s, _)| *s != x_slot(0)));
        }
    }

    // -- ticks -------------------------------------------------------------

    #[test]
    fn fetch_charges_match_the_interpreter() {
        let program = [addi(5, 0, 1), add(6, 5, 5), lui(7, 0x1000), ECALL];
        let l = rv64i(&program);
        assert_eq!(l.insns, 3);
        // Two accesses per uncompressed instruction: the two halfword fetches
        // `exec::fetch` makes, because either half may fault on its own page.
        assert_eq!(block_ticks(&l.block), 6);
        assert_eq!(interpreter_ticks(Config::rv64i(), &program, 3), 6);
    }

    #[test]
    fn a_compressed_instruction_charges_one_fetch() {
        // `c.addi x5, 1` in the low halfword, then `ecall`.
        let c_addi: u16 = 0x0285;
        let mut cfg = Config::rv64gc();
        cfg.pmp_count = 0;
        let words = [u32::from(c_addi) | (u32::from(c_addi) << 16), ECALL];
        let l = lift_at(&cfg, BASE, &words);
        assert_eq!(l.insns, 2);
        // One halfword fetched, one access charged — per instruction.
        assert_eq!(block_ticks(&l.block), 2);
        assert_eq!(interpreter_ticks(cfg, &words, 2), 2);
    }

    #[test]
    fn the_tick_column_is_cumulative_and_never_runs_backwards() {
        let l = rv64i(&[addi(5, 0, 1), addi(6, 0, 2), ECALL]);
        let ticks: Vec<u64> = l.block.marks().iter().map(|m| m.ticks).collect();
        assert_eq!(ticks, vec![0, 2, 4]);
    }

    #[test]
    fn a_memory_op_ends_the_block_and_charges_nothing_itself() {
        let program = [ld(5, 1, 8), addi(6, 0, 1)];
        let l = rv64i(&program);
        assert_eq!(l.insns, 1);
        assert_eq!(l.stop, Stop::Access);
        // Only the fetch is charged here: the access count is 1 when aligned
        // and `bytes` when not, plus walk reads on a TLB miss, so the LD
        // accounts for itself and nothing may follow it in the block.
        assert_eq!(block_ticks(&l.block), 2);
        let charges: Vec<u128> = l
            .block
            .insts()
            .iter()
            .filter(|i| i.op == Opcode::CHARGE)
            .filter_map(|i| i.imm.map(Const::bits))
            .collect();
        assert_eq!(charges, vec![2]);
    }

    // -- loads and stores ---------------------------------------------------

    #[test]
    fn loads_carry_their_width_sign_and_misalignment_policy() {
        for (word, size, sign) in [
            (lb(5, 1, 4), Width::U8, Sign::Signed),
            (lwu(5, 1, 4), Width::U32, Sign::Unsigned),
            (ld(5, 1, 4), Width::U64, Sign::Signed),
        ] {
            let l = rv64i(&[word]);
            let mem = l
                .block
                .insts()
                .iter()
                .find(|i| i.op == Opcode::LD)
                .and_then(|i| i.mem)
                .expect("a load is in the block");
            assert_eq!(mem.size, size);
            assert_eq!(mem.sign, sign);
            assert_eq!(mem.kind, AccessKind::Load);
            assert_eq!(mem.endian, Endian::Little);
            // rv64i() performs misaligned accesses, so no alignment fault.
            assert_eq!(mem.align, Align::None);
            // The bus cycle is guest-visible: DCE may not remove it.
            assert!(mem.volatile);
        }

        // A core that traps misaligned accesses says so in the descriptor.
        let mut strict = Config::rv64i();
        strict.misaligned = false;
        let l = lift_at(&strict, BASE, &[ld(5, 1, 4)]);
        let mem = l
            .block
            .insts()
            .iter()
            .find(|i| i.op == Opcode::LD)
            .and_then(|i| i.mem)
            .unwrap();
        assert_eq!(mem.align, Align::Fault);
    }

    #[test]
    fn a_store_reads_both_registers_and_writes_none() {
        let l = rv64i(&[sd(1, 2, 16)]);
        assert_eq!(l.stop, Stop::Access);
        let st = l
            .block
            .insts()
            .iter()
            .find(|i| i.op == Opcode::ST)
            .expect("a store is in the block");
        assert!(st.dst.is_none());
        assert_eq!(st.mem.unwrap().size, Width::U64);
        // x1 and x2 are read; nothing but the PC is written.
        let exit = l.block.marks().last().unwrap();
        let slots: Vec<u16> = exit.live.iter().map(|(s, _)| s.0).collect();
        assert_eq!(slots, vec![1, 2, PC.0]);
    }

    #[test]
    fn a_load_into_x0_still_makes_its_access() {
        // The value is discarded, but the bus cycle and its tick are not.
        let l = rv64i(&[ld(0, 1, 0)]);
        assert!(ops(&l.block).contains(&"ld"));
        let exit = l.block.marks().last().unwrap();
        assert!(exit.live.iter().all(|(s, _)| *s != x_slot(0)));
    }

    // -- control flow -------------------------------------------------------

    #[test]
    fn a_conditional_branch_selects_between_two_constant_pcs() {
        let l = rv64i(&[beq(1, 2, 8), addi(5, 0, 1)]);
        assert_eq!(l.insns, 1);
        assert_eq!(l.stop, Stop::Transfer);
        let names = ops(&l.block);
        assert!(names.contains(&"setcond"), "{names:?}");
        assert!(names.contains(&"movcond"), "{names:?}");
        // The selected value is the exit PC.
        let sel = l
            .block
            .insts()
            .iter()
            .find(|i| i.op == Opcode::MOVCOND)
            .unwrap();
        let exit = l.block.marks().last().unwrap();
        assert_eq!(exit.live.last().copied(), Some((PC, sel.dst.unwrap())));
        // The two candidates are the taken target and the fall-through.
        let constants: Vec<u128> = l
            .block
            .insts()
            .iter()
            .filter_map(|i| i.imm.map(Const::bits))
            .collect();
        assert!(constants.contains(&u128::from(BASE + 8)), "{constants:x?}");
        assert!(constants.contains(&u128::from(BASE + 4)), "{constants:x?}");
    }

    #[test]
    fn jal_links_a_constant_and_exits_at_a_known_pc() {
        let l = rv64i(&[j_type(1, 8), addi(5, 0, 1)]);
        assert_eq!(l.stop, Stop::Transfer);
        let exit = l.block.marks().last().unwrap();
        // The exit boundary names the target, because JAL's is a constant.
        assert_eq!(exit.pc, BASE + 8);
        // x1 holds the return address.
        assert!(exit.live.iter().any(|(s, _)| *s == x_slot(1)));
    }

    #[test]
    fn jal_with_no_link_register_emits_no_link() {
        let l = rv64i(&[j_type(0, 8)]);
        let exit = l.block.marks().last().unwrap();
        assert_eq!(exit.live.len(), 1, "only the PC");
    }

    #[test]
    fn jalr_clears_the_low_bit_and_needs_a_core_with_c() {
        // Without C the misalignment check is a run-time test this IR cannot
        // express, so JALR is out of the subset.
        let l = rv64i(&[jalr(1, 2, 4)]);
        assert_eq!(l.insns, 0);
        assert_eq!(l.stop, Stop::Unsupported);

        let mut cfg = Config::rv64gc();
        cfg.pmp_count = 0;
        let l = lift_at(&cfg, BASE, &[jalr(1, 2, 4)]);
        assert_eq!(l.insns, 1);
        assert_eq!(l.stop, Stop::Transfer);
        let masks: Vec<u128> = l
            .block
            .insts()
            .iter()
            .filter_map(|i| i.imm.map(Const::bits))
            .collect();
        assert!(masks.contains(&u128::from(!1u64)), "{masks:x?}");
    }

    #[test]
    fn a_branch_to_a_misaligned_target_is_out_of_the_subset_without_c() {
        // imm 2 is a legal B-type immediate and a four-byte-misaligned target,
        // so a taken branch would raise instruction-address-misaligned.
        let l = rv64i(&[beq(1, 2, 2)]);
        assert_eq!(l.insns, 0);
        assert_eq!(l.stop, Stop::Unsupported);
    }

    // -- where blocks end ---------------------------------------------------

    #[test]
    fn a_block_ends_cleanly_at_an_unsupported_opcode() {
        for unsupported in [ECALL, MUL] {
            let l = rv64i(&[addi(5, 0, 1), unsupported, addi(6, 0, 2)]);
            assert_eq!(l.insns, 1);
            assert_eq!(l.stop, Stop::Unsupported);
            // The exit PC is the unsupported instruction's address, so the
            // interpreter picks up exactly where the block gave up.
            let exit = l.block.marks().last().unwrap();
            assert_eq!(exit.pc, BASE + 4);
            assert_eq!(l.block.insts().last().unwrap().op, Opcode::EXIT_TB);
        }
    }

    #[test]
    fn a_block_whose_very_first_instruction_is_unsupported_is_still_well_formed() {
        let l = rv64i(&[ECALL]);
        assert_eq!(l.insns, 0);
        assert_eq!(ops(&l.block), vec!["mov", "insn_start", "exit_tb"]);
        assert_eq!(l.block.marks()[0].pc, BASE);
        assert_eq!(block_ticks(&l.block), 0);
    }

    #[test]
    fn a_block_never_leaves_the_page_it_started_on() {
        // Start two instructions before the page end; the third would cross.
        let base = BASE + 0x1000 - 8;
        let l = lift_at(
            &Config::rv64i(),
            base,
            &[addi(5, 0, 1), addi(6, 0, 2), addi(7, 0, 3)],
        );
        assert_eq!(l.insns, 2);
        assert_eq!(l.stop, Stop::Page);
        assert_eq!(l.block.marks().last().unwrap().pc, base + 8);
    }

    #[test]
    fn the_instruction_limit_ends_a_block() {
        let mut src = Bytes {
            base: BASE,
            words: vec![addi(5, 0, 1); 8],
        };
        let l = lift(&Config::rv64i(), BASE, &mut src, 3).expect("RV64 lifts");
        verify(&l.block).expect("a limited block still verifies");
        assert_eq!(l.insns, 3);
        assert_eq!(l.stop, Stop::Limit);
    }

    #[test]
    fn unreadable_bytes_end_a_block_rather_than_inventing_an_encoding() {
        let l = rv64i(&[addi(5, 0, 1)]);
        assert_eq!(l.insns, 1);
        assert_eq!(l.stop, Stop::Unreadable);
    }

    #[test]
    fn rv32_is_refused_rather_than_mis_widened() {
        let mut src = Bytes {
            base: BASE,
            words: vec![addi(5, 0, 1)],
        };
        let err =
            lift(&Config::rv32gc(), BASE, &mut src, MAX_INSNS).expect_err("RV32 is not lifted yet");
        assert!(matches!(err, Error::Unimplemented(_)), "{err}");
    }

    // -- the block as a whole ----------------------------------------------

    #[test]
    fn the_cache_key_separates_configurations_that_lift_differently() {
        let plain = rv64i(&[addi(5, 0, 1)]).block.key;
        let mut with_c = Config::rv64i();
        with_c.ext = Extensions {
            c: true,
            ..Extensions::I
        };
        let compressed = lift_at(&with_c, BASE, &[addi(5, 0, 1)]).block.key;
        assert_ne!(plain, compressed);

        let mut strict = Config::rv64i();
        strict.misaligned = false;
        assert_ne!(plain, lift_at(&strict, BASE, &[addi(5, 0, 1)]).block.key);
    }

    #[test]
    fn every_boundary_names_only_temporaries_that_already_exist() {
        // The verifier checks this, and lift_at asserts the verifier; this
        // says why it matters — a fault at a boundary materializes exactly
        // these temporaries into architectural state.
        let l = rv64i(&[addi(5, 1, 1), add(6, 5, 2), sd(6, 5, 0)]);
        assert_eq!(l.insns, 3);
        for mark in l.block.marks() {
            for (_, t) in &mark.live {
                assert!(t.index() < l.block.temp_count());
            }
        }
        // The reservation is numbered but never bound: a store breaks it at
        // run time, inside the ST, exactly as `exec::store` does.
        for mark in l.block.marks() {
            assert!(mark.live.iter().all(|(s, _)| *s != RESERVATION));
        }
    }

    #[test]
    fn a_longer_straight_line_block_matches_the_interpreter_tick_for_tick() {
        let program = [
            lui(5, 0x1000),
            addi(5, 5, 0x20),
            slli(6, 5, 3),
            slt(7, 5, 6),
            addw(8, 5, 6),
            sub(9, 6, 5),
            ECALL,
        ];
        let l = rv64i(&program);
        assert_eq!(l.insns, 6);
        assert_eq!(block_ticks(&l.block), 12);
        assert_eq!(interpreter_ticks(Config::rv64i(), &program, 6), 12);
    }
}
